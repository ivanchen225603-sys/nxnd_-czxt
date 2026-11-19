#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cli/verify.py — 回归验证：Python 判定  vs  原版 reason.txt

────────────────────────────────────────────────────────────────────────
这个脚本是干什么的
────────────────────────────────────────────────────────────────────────

把历史样本目录里每一份 `.xyz` 重新跑一遍本项目的检测与判定链，
再把结果和**原版程序当时写下的** `.xyz.reason.txt` 逐条比对。

为什么要这么比？
    原版 exe（calibrate_test.exe）已经停产、作者失联，跑不起来了。
    但它当年跑出来的 reason.txt 还在 —— 那 10 份文本就是"标准答案"，
    是原版判定链在这 10 个真实场景上的输出记录。
    本项目的目标是"行为与原版一致"，所以拿它当回归基线：
    只要 10/10 对得上，就说明移植没有走样；一旦哪天改了阈值/顺序，
    这里会立刻变红。改 detect.py / devices.json 之后**必须**先跑这个。

为什么不是"oracle 对拍"？
    真正的对拍（同一份输入同时喂原版 DLL 和本项目，逐字节比）需要
    原版 exe 能跑 + 那份现场配置。两者都不在了。
    reason.txt 是退而求其次、但完全够用的等价物：
    它固定了"输入 → 判定理由"这个映射，正是我们要守住的东西。

────────────────────────────────────────────────────────────────────────
用法
────────────────────────────────────────────────────────────────────────

    # 基本回归（默认扫 baseline）
    python3 cli/verify.py

    # 指定样本目录 / 只看不一致的
    python3 cli/verify.py --dir baseline --quiet

    # 打印每份样本的诊断数字（内点、占比、span、floor_z …）
    python3 cli/verify.py --detail

    # 输出机器可读的结果，塞进 CI / 巡检
    python3 cli/verify.py --json logs/verify.json

    # 没有 reason.txt 的样本也一起跑（只显示判定，不参与打分）
    python3 cli/verify.py --allow-missing

退出码
    0 = 全部一致（或没有可比的样本）
    1 = 存在不一致  →  现场遇到先看下面的排查提示

排查
    不一致时优先看这三件事（脚本会在末尾自动列出）：
      ① config/config.json 的 processing.yaw_offset_deg / roi
      ② devices.json 的 7 个 floor_gate_* 门槛
      ③ 样本是不是被外部工具重新裁剪过（点数与当年不同）
"""
from __future__ import annotations

import argparse
import glob
import hashlib
import io
import json
import os
import time
import sys
from collections import Counter

import _common
_common.setup_console()

import numpy as np

from app.settings import baseline_rel, root, load_calib, load_devices
from app.pointcloud import load_xyz, load_reason, apply_roi, file_sha256
from app.detect import detect_empty, evaluate_empty

# 判定理由里"非空车"的那一类，用于统计混淆
NON_EMPTY = "非空车"


def discover(directory: str, recursive: bool = True, allow_missing: bool = False):
    """找出所有「有/没有 reason.txt」的样本。

    返回 [(xyz_path, reason_path_or_None), ...]，按路径排序，保证可比对。
    """
    pat = os.path.join(directory, "**", "*.xyz") if recursive else \
        os.path.join(directory, "*.xyz")
    out = []
    for p in sorted(glob.glob(pat, recursive=recursive)):
        rp = p + ".reason.txt"
        if os.path.exists(rp):
            out.append((p, rp))
        elif allow_missing:
            out.append((p, None))
    return out


# ═══════════════════════════════════════════════════════════════════
# ★ 基线校验和清单（2026-09-17，docs/14 §2.3）
# ═══════════════════════════════════════════════════════════════════
#
# 为什么需要：这道闸门的全部效力都来自 `baseline` 里那 10 份样本 ——
# 点云是输入、`.reason.txt` 是标准答案。**任何一方被无声改动，闸门就悄悄变松，
# 而它照样报"全部通过"。** 能想到的几种走法：
#
#   · 编辑器打开 .reason.txt 顺手改了换行/编码（CRLF、加 BOM）；
#   · `tools/fix_plate_dirs.py --apply` 被误跑到 baseline 上
#     —— CLAUDE.md 专门警告过这条，它会改变 verify.py 的文件集；
#   · 有人"顺手"往基线里塞一份新样本，而它恰好是通过的
#     —— 分母变大、难度变低，也是变松；
#   · 某一份被删掉。
#
# 所以这里记下每个文件的 SHA-256。**不一致就直接失败**，
# 而不是打一行提示了事 —— 一个悄悄变松的回归闸门比没有闸门更危险。
#
# ▲ 清单自己不进 `discover()`（它只认 *.xyz），所以放在 baseline 里是安全的，
#   而且跟着基线一起走。

MANIFEST_NAME = "MANIFEST.json"


def _manifest_files(directory: str):
    """基线目录里所有参与闸门的文件：点云 + 标准答案。按相对路径排序。"""
    out = []
    for xyz_path, reason_path in discover(directory, recursive=True,
                                          allow_missing=True):
        out.append(xyz_path)
        if reason_path:
            out.append(reason_path)
    return sorted(set(out))


def build_manifest(directory: str) -> dict:
    files = {}
    for f in _manifest_files(directory):
        rel = os.path.relpath(f, directory).replace("\\", "/")
        files[rel] = {"sha256": file_sha256(f), "bytes": os.path.getsize(f)}
    return {
        "note": ("回归基线的校验和。verify.py 每次都会核对 —— 不一致即失败。"
                 "改动基线是**有意为之**时，跑 `python3 cli/verify.py "
                 "--update-manifest` 重新生成，并在提交信息里写明为什么。"),
        "generated": time.strftime("%Y-%m-%d %H:%M:%S"),
        "count": len(files),
        "files": files,
    }


def check_manifest(directory: str):
    """核对基线。返回 (状态, 说明行列表)。

    状态："ok" / "missing"（还没生成清单）/ "bad"（对不上）。
    `missing` **不算失败** —— 不能因为一个以前不存在的文件去罚人；
    但 `bad` 一定失败，那正是我们要拦的。
    """
    mp = os.path.join(directory, MANIFEST_NAME)
    if not os.path.exists(mp):
        return "missing", [
            "▲ 没有 %s —— 基线**没有被校验和保护**。" % MANIFEST_NAME,
            "  跑 `python3 cli/verify.py --update-manifest` 生成一份，"
            "之后基线被无声改动就会被拦下来。"]
    try:
        with io.open(mp, encoding="utf-8") as f:
            man = json.load(f)
        want = man.get("files") or {}
    except Exception as e:                                   # noqa: BLE001
        return "bad", ["× %s 读不出来（%s: %s）" % (MANIFEST_NAME, type(e).__name__, e)]

    now = {}
    for f in _manifest_files(directory):
        rel = os.path.relpath(f, directory).replace("\\", "/")
        now[rel] = {"sha256": file_sha256(f), "bytes": os.path.getsize(f)}

    changed = sorted(k for k in set(want) & set(now)
                     if want[k].get("sha256") != now[k]["sha256"])
    gone = sorted(set(want) - set(now))
    extra = sorted(set(now) - set(want))
    if not (changed or gone or extra):
        return "ok", ["√ 基线校验和一致（%d 个文件）" % len(now)]

    msg = ["× **基线与清单对不上** —— 闸门可能已经悄悄变松了："]
    for k in changed:
        msg.append("    内容被改：%s" % k)
        msg.append("        清单 %s（%d 字节）-> 现在 %s（%d 字节）"
                   % (want[k].get("sha256", "?")[:12], want[k].get("bytes", -1),
                      now[k]["sha256"][:12], now[k]["bytes"]))
    for k in gone:
        msg.append("    **少了**：%s（分母变小 = 闸门变松）" % k)
    for k in extra:
        msg.append("    **多了**：%s（往基线里塞样本同样是变松）" % k)
    msg.append("  确实是有意改动的话：`python3 cli/verify.py --update-manifest`，")
    msg.append("  并在提交信息里写明为什么 —— 这道闸门是本项目"
               "「移植没走样」的唯一硬证据。")
    return "bad", msg


# ═══════════════════════════════════════════════════════════════════
# ★ 预期分歧表（`floor_gate_z_above_mode = "plane"` 时才生效）
# ═══════════════════════════════════════════════════════════════════
#
# 10 份基线的"标准答案"是**原厂的输出**，不是"车到底空不空"。
# 2026-09-16 查实：其中 4 份**本身就是空车**，原厂把它们误判成了载货。
#
# 依据（docs/12 §4.1）：把底面平面的**绝对高度**摊开看 ——
#     报「底面内点占比过低」的 6 份：平面离地 3.30~3.55 m  = **货面**
#     报「底面过高，疑似载物」的 4 份：平面离地 1.51~1.64 m = **底板**
#     2026-09-15 两辆现场确认的空车：      1.52~1.55 m  <- 一致到 3 cm
# 那 4 份的平面上方 2.4 m 只有侧壁掠射点，没有货面那种密集带。
#
# 原厂误判的原因在判定链第 10 步：它拿「平面在**原点**的 z」去比
# 「**整片**点云的最低点」，底板一斜就差出落差的一半 ——
# 而空载自卸车底板本来就向车头翘 3.8~4.1 度。
# 换成倾斜无关的度量（mode="plane"）之后，这 4 份就走到第 11 步「空车」。
#
# ▲ 这张表**只放行一种跃迁**：原版「底面过高，疑似载物」-> 本项目「空车」。
#   同一份样本出现任何**别的**不一致，照样算失败。
#   ▲ 不要把这 4 份从基线里删掉 —— 闸门会悄悄变松。
#   ▲ 现在空车真值只有 2 份，这 4 份属于**推断**。攒够正样本之后
#     应当人工复核这张表，并把依据更新到 docs/12 §7。
EXPECTED_DIVERGENCE_PLANE = {
    "瀹丄09692D/20260721_190013.xyz",
    "瀹丄G3661/20260721_190520.xyz",
    "钂機02002F/20260721_191646.xyz",
    "钂檆02002F/20260721_191646.xyz",     # 与上一份内容相同（基线里本就有重复）
}
EXPECTED_FROM = "底面过高，疑似载物"
EXPECTED_TO = "空车"


def expected_divergence(rel: str, expected: str, actual: str, dev) -> bool:
    """这一份的不一致是不是**预期之内**的。"""
    if str(getattr(getattr(dev, "gate", dev), "z_above_mode", "origin")) != "plane":
        return False
    return (rel in EXPECTED_DIVERGENCE_PLANE
            and expected == EXPECTED_FROM and actual == EXPECTED_TO)


def judge_one(xyz_path: str, calib, dev, use_roi: bool):
    """对单份样本跑一遍检测与判定，返回 (ok, reason, diag, nframe, err)."""
    try:
        fr = load_xyz(xyz_path)
    except Exception as e:                                  # noqa: BLE001
        return None, "", None, 0, f"读取失败: {type(e).__name__}: {e}"
    if use_roi:
        fr = apply_roi(fr, calib.roi)
    if fr.n == 0:
        return None, "", None, 0, "ROI 裁剪后没有点云"
    try:
        res = detect_empty(fr, calib)
        cloud = res.cloud if res is not None else fr.xyz
        # ★ 必须把 ground_z 传进去 —— 第 10 步的底板露出兜底靠它。
        #   不传的话这道闸门**测不到**那条判据，会给出虚假的通过。
        ok, reason, dg = evaluate_empty(res, fr.n, cloud, dev,
                                        ground_z=calib.detection.ground_z)
        return bool(ok), reason, dg, fr.n, ""
    except Exception as e:                                  # noqa: BLE001
        return None, "", None, 0, f"检测异常: {type(e).__name__}: {e}"


def _fmt_diag(dg) -> str:
    if dg is None:
        return "（无诊断）"
    za = dg.z_above
    za_s = "—" if za != za else f"{za:+.3f}"
    fz = "—" if dg.floor_z != dg.floor_z else f"{dg.floor_z:.3f}"
    mn = "—" if dg.min_z != dg.min_z else f"{dg.min_z:.3f}"
    return (f"点数={dg.n_points} 内点={dg.inliers} 占比={dg.ratio:.4f} "
            f"spanX={dg.span_x:.2f} spanY={dg.span_y:.2f} "
            f"|nz|={dg.normal_z:.3f} floor_z={fz} min_z={mn} z_above={za_s} "
            f"yaw={dg.yaw_deg:+.2f}°")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="verify.py", description="Python 判定 vs 原版 reason.txt 回归验证",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("用法")[0])
    p.add_argument("--dir", default=baseline_rel(),
                   help="历史样本目录（默认 %s）" % baseline_rel())
    p.add_argument("--no-recursive", action="store_true",
                   help="只扫目录第一层，不进子目录")
    p.add_argument("--no-roi", action="store_true",
                   help="不裁 ROI（默认会裁；历史样本当年已裁过，裁剪是幂等的）")
    p.add_argument("--allow-missing", action="store_true",
                   help="没有 reason.txt 的样本也跑，只显示判定不计分")
    p.add_argument("--dedupe", action="store_true",
                   help="按文件内容去重（历史样本里可能有同一份被复制到多个目录）")
    p.add_argument("--detail", action="store_true",
                   help="每份样本都打印诊断数字")
    p.add_argument("--strict", action="store_true",
                   help="连「预期内的分歧」也算失败 —— 验证与原厂逐字一致")
    p.add_argument("--gate-mode", choices=("origin", "plane"), default=None,
                   help="仅本次验证覆盖判定模式，不改配置；origin 配合 --strict 验收原厂兼容")
    p.add_argument("--json", default="", metavar="PATH",
                   help="把结果写成 JSON（便于 CI 比对）")
    p.add_argument("--channels", default="", help="多通道配置路径（取第 1 条启用通道的标定）")
    p.add_argument("-q", "--quiet", action="store_true", help="只打印不一致的")
    p.add_argument("--update-manifest", action="store_true",
                   help="重新生成基线校验和清单（**有意改动基线时才用**）")
    p.add_argument("--no-manifest", action="store_true",
                   help="跳过基线校验和核对（排查用，正式跑别加）")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    base = _common.ensure_root()

    directory = args.dir if os.path.isabs(args.dir) else os.path.join(base, args.dir)
    if not os.path.isdir(directory):
        print(f"× 样本目录不存在: {directory}")
        return 1

    # 标定与门槛取自「第一条启用通道」——历史样本只有一套标定，够用
    from app.settings import load_settings
    st = load_settings(args.channels or "")
    enabled = st.enabled()
    if not enabled:
        print("× channels.json 里没有启用的通道")
        return 1
    ch = enabled[0]
    calib = load_calib(ch.abs_config_path)
    dev = load_devices(ch.abs_devices_path)
    if args.gate_mode is not None:
        dev.gate.z_above_mode = args.gate_mode

    # ★ 先核对基线本身有没有被动过（docs/14 §2.3）。
    #   放在跑判定**之前**：基线都不对了，再比对结论没有意义。
    if args.update_manifest:
        man = build_manifest(directory)
        mp = os.path.join(directory, MANIFEST_NAME)
        with io.open(mp, "w", encoding="utf-8") as f:
            f.write(json.dumps(man, ensure_ascii=False, indent=2))
        print("√ 已写入 %s（%d 个文件）" % (mp, man["count"]))
        print("  ▲ 这等于宣告「基线现在就该是这样」—— 提交时请写明改了什么、为什么。")
        return 0
    if not args.no_manifest:
        state, lines = check_manifest(directory)
        for ln in lines:
            print(ln)
        if state == "bad":
            return 1
        print()

    samples = discover(directory, recursive=not args.no_recursive,
                       allow_missing=args.allow_missing)
    if not samples:
        print(f"▲  {directory} 下没有找到可验证的样本（*.xyz + *.reason.txt）")
        print("    历史样本应放在 %s/<车牌>/<时间>.xyz 与同名 .reason.txt"
              % baseline_rel())
        return 0

    # 内容去重：历史样本里同一份点云可能被复制到多个目录（尤其是 GBK 乱码的
    # 车牌目录），统计"独立样本数"时应当排掉，否则会虚高分母。
    dup_map = {}
    for xyz_path, _ in samples:
        try:
            with open(xyz_path, "rb") as f:
                h = hashlib.sha256(f.read()).hexdigest()
        except OSError:
            h = xyz_path
        dup_map.setdefault(h, []).append(xyz_path)
    dup_groups = {h: v for h, v in dup_map.items() if len(v) > 1}
    if dup_groups and not args.dedupe:
        print("▲  检测到内容重复的样本（同一份点云出现在多个目录）：")
        for h, v in dup_groups.items():
            print("    " + "  <->  ".join(os.path.relpath(x, directory).replace("\\", "/")
                                         for x in v))
        print("    （加 --dedupe 可只统计唯一内容；不改则按文件数计分）")
        print("-" * 78)
    if args.dedupe:
        seen, uniq = set(), []
        for xyz_path, rp in samples:
            try:
                with open(xyz_path, "rb") as f:
                    h = hashlib.sha256(f.read()).hexdigest()
            except OSError:
                h = xyz_path
            if h in seen:
                continue
            seen.add(h)
            uniq.append((xyz_path, rp))
        print(f"（--dedupe：{len(samples)} 份文件 → {len(uniq)} 份唯一内容）")
        samples = uniq

    print("=" * 78)
    print("载重雷达 —— 判定链回归验证")
    print("=" * 78)
    print(f"样本目录   : {directory}")
    print(f"标定       : {calib.path}")
    print(f"判定门槛   : {dev.path}")
    print(f"判定模式   : {dev.gate.z_above_mode}（仅当前验证进程）")
    print(f"裁剪 ROI   : {'否（--no-roi）' if args.no_roi else calib.roi}")
    print(f"样本数量   : {len(samples)}")
    print("-" * 78)

    rows = []
    n_cmp = n_pass = n_fail = 0
    expected_c = Counter()
    actual_c = Counter()
    n_known = 0                    # 预期之内的分歧（见 EXPECTED_DIVERGENCE_PLANE）

    for xyz_path, reason_path in samples:
        rel = os.path.relpath(xyz_path, directory).replace("\\", "/")
        expected = load_reason(xyz_path) if reason_path else ""
        ok, reason, dg, nframe, err = judge_one(xyz_path, calib, dev, not args.no_roi)

        row = {
            "sample": rel,
            "path": xyz_path,
            "expected": expected,
            "actual": reason,
            "n_points": int(nframe),
            "error": err,
            "match": None,
            "diagnostics": dg.to_dict() if dg is not None else None,
        }
        if err:
            row["match"] = False
            rows.append(row)
            print(f"× {rel}")
            print(f"     {err}")
            n_fail += 1
            continue

        has_truth = bool(reason_path)
        if has_truth:
            n_cmp += 1
            expected_c[expected or "(空)"] += 1
            actual_c[reason] += 1
            row["match"] = (reason == expected)
            if row["match"]:
                n_pass += 1
            elif not args.strict and expected_divergence(rel, expected, reason, dev):
                # 预期之内的分歧：不算失败，但**单独计数并逐条打印**，
                # 绝不静默放过 —— 它是"我们故意与原厂不一致"的地方。
                row["match"] = None
                row["expected_divergence"] = True
                n_known += 1
            else:
                n_fail += 1

        mark = ("√" if row["match"] else
                ("!" if row.get("expected_divergence") else
                 ("×" if has_truth else "·")))
        if args.quiet and row["match"]:
            rows.append(row)
            continue
        print(f"{mark} {rel}   [{nframe} 点]")
        if has_truth:
            print(f"     原版={expected!r}   本项目={reason!r}")
            if row.get("expected_divergence"):
                print("     ! 预期内的分歧：这一份本身就是空车，原厂误判成了载货")
                print("       依据：底面平面落在底板高度（见 cli/verify.py 顶部的说明）")
        else:
            print(f"     （无 reason.txt）本项目={reason!r}")
        if args.detail or (has_truth and row["match"] is False):
            print(f"     {_fmt_diag(dg)}")
        rows.append(row)

    # ── 汇总 ────────────────────────────────────────────────
    print("-" * 78)
    if n_cmp:
        extra = f" + {n_known} 项预期内分歧" if n_known else ""
        print(f"结果：{n_pass}/{n_cmp} 一致{extra}"
              + ("  √ 全部通过" if n_fail == 0 else f"  × {n_fail} 项不一致"))
        if n_known:
            mode = getattr(getattr(dev, "gate", dev), "z_above_mode", "origin")
            print(f"     （第 10 步用 z_above_mode={mode!r}：上面 ! 那 {n_known} 份"
                  f"本身就是空车，与原厂不一致是**故意的**，见 docs/12 §4.1）")
    else:
        print("结果：没有带 reason.txt 的样本，无法打分")
    print(f"原版理由分布 : {dict(expected_c)}")
    print(f"本项目分布   : {dict(actual_c)}")

    if n_fail:
        print("\n排查建议（按优先级）：")
        print("  1) 先看上面 × 那几行的诊断数字，对不上的是哪一步：")
        print("     点数不足 → devices.json.min_save_points / ROI 裁太狠")
        print("     内点/占比 → config.json 的 floor_gate_distance_m 或 ROI")
        print("     z_above   → processing.ground_z、z_down、yaw_offset_deg")
        print("  2) 确认样本没被别的工具重新裁剪过（点数应与当年一致）")
        print("  3) 改动 detect.py 的判定顺序前，先读它的文档注释")

    if args.json:
        out = os.path.join(base, args.json) if not os.path.isabs(args.json) else args.json
        os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            json.dump({
                "directory": directory,
                "calib": calib.path,
                "devices": dev.path,
                "gate_mode": dev.gate.z_above_mode,
                "strict": args.strict,
                "roi": list(map(list, calib.roi)) if calib.roi is not None else None,
                "compared": n_cmp, "passed": n_pass, "failed": n_fail,
                "expected_dist": dict(expected_c), "actual_dist": dict(actual_c),
                "rows": rows,
            }, f, ensure_ascii=False, indent=2)
        print(f"\n结果已写入 : {out}")

    print("=" * 78)
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
