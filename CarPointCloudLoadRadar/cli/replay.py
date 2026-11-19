#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cli/replay.py — 把历史帧 / 任意点云目录重新跑一遍检测

和 cli/verify.py 的分工：
    verify.py   是"回归闸门" —— 只认带 reason.txt 的样本，逐条比对，退出码非 0 即失败。
    replay.py   是"批量重跑器" —— 目录里有什么就跑什么，不要求有标准答案，
                输出判定表，可选地重新落盘。

为什么回放**不经过停稳检测**？
    停稳检测（StabilityDetector）比较的是"相邻两帧的质心差"——
    它假设相邻帧是同一辆车在连续时刻的画面。
    而 baseline 里的每一份 .xyz 都是**一辆已经停稳的车的独立快照**，
    两两之间相差几米，直接喂给停稳检测只会得到"永远在运动"，永远测不了。
    所以回放模式对每一份帧直接做一次完整检测，不做时序门控。
    这与原版一致：原版也是这样"一帧一判"地离线复算的。
    （实时链路里才需要停稳检测，那是 cli/run.py 的事。）

────────────────────────────────────────────────────────────────────────
用法
────────────────────────────────────────────────────────────────────────

    # 重跑 baseline 下所有帧，打印判定表
    python3 cli/replay.py

    # 只跑一个文件
    python3 cli/replay.py --file baseline/TEST001/20260721_160519.xyz

    # 指定目录、只跑前 20 帧、只看不一致的
    python3 cli/replay.py --dir baseline --limit 20 -q

    # 重新落盘到另一个目录（会被判定理由归类）
    python3 cli/replay.py --out logs/replayed

    # 机器可读
    python3 cli/replay.py --json logs/replay.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
import tempfile

if __package__:
    from . import _common
else:
    import _common
_common.setup_console()

import numpy as np

from app.settings import (baseline_rel, load_settings, load_calib, load_devices,
                          ensure_dir, config_fingerprint)
from app.pointcloud import (load_xyz, load_reason, apply_roi, save_xyz,
                            save_reason, Frame)
from app.detect import detect_empty, evaluate_empty, check_dims
from app.plate import safe_dir_name
from app.sample_paths import Sample, Discovery, discover_samples, REPLAY_NOTE
from app.archive_reader import load_archive, INPUT_REPLAY_NOTE


def _resolve(path: str) -> str:
    return path if os.path.isabs(path) else os.path.join(_common.ensure_root(), path)


def detect_one(path: str, calib, dev, use_roi: bool):
    """对一帧独立做完整检测。返回 (ok, reason, diag, frame, res, err)."""
    try:
        fr = path if isinstance(path, Frame) else load_xyz(path)
    except Exception as e:                                  # noqa: BLE001
        return None, "", None, None, None, f"{type(e).__name__}: {e}"
    if use_roi:
        fr = apply_roi(fr, calib.roi)
    if fr.n == 0:
        return None, "", None, fr, None, "ROI 裁剪后无点云"
    try:
        res = detect_empty(fr, calib)
        cloud = res.cloud if res is not None else fr.xyz
        # ★ 必须传 ground_z，与服务、cli/verify.py 同口径（2026-09-24）：第 10 步的兜底
        #   和"平面离地上限"都靠它，不传就不生效，重跑结论会与服务不一样。
        ok, reason, dg = evaluate_empty(res, fr.n, cloud, dev, ground_z=calib.detection.ground_z)
        return bool(ok), reason, dg, fr, res, ""
    except Exception as e:                                  # noqa: BLE001
        return None, "", None, fr, None, f"{type(e).__name__}: {e}"


def collect_samples(single: str, directory: str, limit: int = 0) -> Discovery:
    if limit < 0:
        raise ValueError("样本数量限制不能为负数")
    if single:
        p = _resolve(single)
        return Discovery([Sample(p, role="explicit")] if os.path.isfile(p) else [])
    return discover_samples(_resolve(directory), role="observation", limit=limit)


def collect(single: str, directory: str, limit: int = 0):
    found = collect_samples(single, directory, limit)
    found.report()
    return [sample.path for sample in found.samples]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="replay.py", description="批量 / 单帧重跑检测（离线，不经停稳门控）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("用法")[0])
    ap.add_argument("--file", default="", help="只回放这一个 .xyz")
    ap.add_argument("--dir", default=baseline_rel(),
                    help="回放目录（默认 %s）" % baseline_rel())
    ap.add_argument("--limit", type=int, default=0, help="最多跑多少帧（0=全部）")
    ap.add_argument("--channels", default="", help="多通道配置路径（取标定与门槛）")
    ap.add_argument("--no-roi", action="store_true", help="不裁 ROI")
    ap.add_argument("--archive-input", action="store_true",
                    help="从归档原行关系恢复主 ROI 输入，再用当前配置诊断（不二次裁 ROI）")
    ap.add_argument("--out", default="", help="把结果重新落盘到该目录")
    ap.add_argument("--json", default="", help="结果写 JSON")
    ap.add_argument("--realtime", action="store_true",
                    help="按 ReplaySource 的节奏回放（带间隔），默认全速")
    ap.add_argument("-q", "--quiet", action="store_true", help="只打印非'空车'的帧")
    args = ap.parse_args(argv)
    if args.archive_input and (args.file or args.no_roi):
        ap.error("--archive-input 只用于目录发现的完整归档，不能与 --file/--no-roi 混用")

    if args.out and not args.file:
        # 输出不能回灌为下一轮旧样本，更不能与自己刚生成的 reason 比较。
        source_dir = os.path.normcase(os.path.realpath(_resolve(args.dir)))
        target_dir = os.path.normcase(os.path.realpath(_resolve(args.out)))
        try:
            common = os.path.commonpath((source_dir, target_dir))
        except ValueError:                   # Windows 不同盘符必然不相交。
            common = ""
        if common in (source_dir, target_dir):
            ap.error("目录回放的 --out 与 --dir 必须相互独立，不能相等或互相包含")

    st = load_settings(args.channels or "")
    enabled = st.enabled() or st.channels
    if not enabled:
        print("× channels.json 里没有通道")
        return 1
    ch = enabled[0]
    calib = load_calib(ch.abs_config_path)
    dev = load_devices(ch.abs_devices_path)

    if args.limit < 0:
        ap.error("--limit 不能为负数")
    found = collect_samples(args.file, args.dir, args.limit)
    found.report()
    files = [sample.path for sample in found.samples]
    if not files:
        print(f"× 没有找到可回放的 .xyz（--file {args.file!r} / --dir {args.dir!r}）")
        return 1

    print("=" * 78)
    print("载重雷达 —— 离线重跑（每帧独立判定，不经停稳门控）")
    print("=" * 78)
    print(f"来源       : {args.file or _resolve(args.dir)}")
    print(f"帧数       : {len(files)}")
    print(f"标定/门槛  : {os.path.basename(calib.path)} / {os.path.basename(dev.path)}")
    if any(sample.archived for sample in found.samples):
        print("回放边界   : " + (INPUT_REPLAY_NOTE if args.archive_input else REPLAY_NOTE))
        print("配置边界   : 按归档通道选择当前配置重算，不声称复现归档时配置")
    if args.file:
        print("单文件诊断 : 显式选择角色；不代表整包完整性或原始多帧回放验证")
    output_run = ""
    if args.out:
        ensure_dir(_resolve(args.out))
        output_run = tempfile.mkdtemp(prefix="replay-", dir=_resolve(args.out))
        print(f"重新落盘   : {output_run}")
    print("-" * 78)
    print(f"{'帧':<44}{'点数':>8}  {'判定':<22}{'与原版'}")

    rows = []
    t0 = time.time()
    n_ok = n_bad = n_mismatch = n_compared = n_errors = 0
    interval = ch.replay_interval if args.realtime else 0.0
    channels = {item.id: item for item in st.channels}
    contexts = {ch.id: (calib, dev)}

    for i, sample in enumerate(found.samples, 1):
        path = sample.path
        archive_info = {}
        try:
            if sample.archived:
                if sample.channel not in channels:
                    raise ValueError("归档通道没有当前配置: " + sample.channel)
                if sample.channel not in contexts:
                    current = channels[sample.channel]
                    contexts[sample.channel] = (load_calib(current.abs_config_path),
                                                load_devices(current.abs_devices_path))
                current_calib, current_dev = contexts[sample.channel]
                archive = load_archive(sample)
                target = (archive.primary_input() if args.archive_input else archive.role("observation")).frame()
                archive_info = dict(manifest_sha256=archive.manifest_sha256,
                                    configuration_hash=archive.metadata["configuration_hash"],
                                    capabilities=archive.capabilities,
                                    current_config_fingerprint=config_fingerprint(current_calib, current_dev),
                                    primary_members=len(archive.metadata.get("provenance", {}).get("segments", [])))
            else:
                if args.archive_input:
                    raise ValueError("--archive-input 拒绝无来源清单的裸 XYZ")
                current_calib, current_dev = calib, dev
                target = path
            ok, reason, dg, fr, res, err = detect_one(target, current_calib, current_dev,
                                                     not args.no_roi and not args.archive_input)
        except (OSError, ValueError) as exc:
            ok, reason, dg, fr, res, err = None, "", None, None, None, str(exc)
        # observation 与原多帧 detection 输入不等价，不挪用其 reason 作标准答案。
        want = "" if sample.archived else load_reason(path)
        rel = _rel(path, args.dir, args.file)

        row = {"i": i, "file": rel, "path": path, "n_points": int(fr.n) if fr else 0,
               "empty": (None if ok is None else bool(ok)), "reason": reason,
               "recorded": want, "error": err,
               "match": (None if not want or ok is None else (want == reason)),
               "role": sample.role, "measurement_id": sample.measurement_id or None,
               "channel": sample.channel or ch.id,
               "replay_note": ((INPUT_REPLAY_NOTE if args.archive_input else REPLAY_NOTE) if sample.archived else ""),
               "input_kind": "original_primary_roi" if args.archive_input else "last_observation" if sample.archived else "legacy",
               "archive": archive_info,
               "diagnostics": dg.to_dict() if dg is not None else None}
        if err:
            n_bad += 1
            n_errors += 1
        elif ok:
            n_ok += 1
        else:
            n_bad += 1
        if want and ok is not None:
            n_compared += 1
            if want != reason:
                n_mismatch += 1

        if not (args.quiet and ok is True):
            m = "—" if row["match"] is None else ("√" if row["match"] else "×")
            name = rel if len(rel) <= 42 else "…" + rel[-41:]
            print(f"{name:<44}{row['n_points']:>8}  {reason:<22}{m}")
            if err:
                print(f"     × {err}")
            elif args.quiet is False and dg is not None and not ok:
                print(f"     内点={dg.inliers} 占比={dg.ratio:.4f} "
                      f"spanX={dg.span_x:.2f} spanY={dg.span_y:.2f} "
                      f"z_above={'—' if dg.z_above != dg.z_above else format(dg.z_above, '+.3f')}")

        # 可选重新落盘：按理由分类存，方便肉眼归类比对
        if args.out and fr is not None and ok is not None:
            outdir = os.path.join(output_run, safe_dir_name(reason), sample.output_key)
            ensure_dir(outdir)
            base = os.path.join(outdir, os.path.basename(path)[:-4])
            cloud = res.cloud if res is not None else fr.xyz
            # 检测云可能是输入的子集，ring 必须沿真实行索引选择。
            indices = getattr(res, "source_indices", None)
            rings = (fr.ring[indices] if fr.ring is not None and indices is not None
                     else fr.ring if len(cloud) == fr.n else None)
            save_xyz(base + ".xyz", Frame(cloud, rings, seq=fr.seq, ts=fr.ts))
            save_reason(base + ".xyz.reason.txt", reason)
            row["output"] = base + ".xyz"

        rows.append(row)
        if interval:
            time.sleep(interval)

    dt = time.time() - t0
    print("-" * 78)
    print(f"完成：{len(files)} 帧，空车 {n_ok} / 非空或异常 {n_bad}，"
          f"耗时 {dt:.2f}s（{len(files)/max(dt,1e-6):.1f} 帧/s）")
    if n_mismatch:
        print(f"▲  与 reason.txt 不一致 {n_mismatch} 帧 —— 请跑 cli/verify.py 看详情")
    elif n_compared:
        print(f"已比较 {n_compared} 帧，与记录的 reason.txt 一致")
    else:
        print("未执行理由回归比较：0 个可比较的 reason.txt；重跑成功不代表回归通过")

    if args.json:
        out = _resolve(args.json)
        ensure_dir(os.path.dirname(out))
        with open(out, "w", encoding="utf-8") as f:
            json.dump({"source": args.file or _resolve(args.dir), "frames": len(files),
                       "empty": n_ok, "not_empty": n_bad, "mismatch": n_mismatch,
                       "compared": n_compared, "errors": n_errors,
                       "comparison_status": ("not_run" if not n_compared else "mismatch" if n_mismatch
                                             else "partial" if n_compared < len(files) else "matched"),
                       "discovery_errors": found.errors, "output_run": output_run,
                       "seconds": round(dt, 3), "rows": rows},
                      f, ensure_ascii=False, indent=2)
        print(f"结果已写入 : {out}")
    print("=" * 78)
    return 0 if n_mismatch == 0 and n_errors == 0 and not found.errors else 1


def _rel(path: str, directory: str, single: str) -> str:
    if single:
        return os.path.basename(path)
    d = _resolve(directory)
    try:
        return os.path.relpath(path, d).replace("\\", "/")
    except ValueError:
        return path


if __name__ == "__main__":
    raise SystemExit(main())
