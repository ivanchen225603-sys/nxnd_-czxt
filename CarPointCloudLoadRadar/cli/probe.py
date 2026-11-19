#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cli/probe.py — 用真实抓包反推解码参数（现场校准第二步）

上游是 cli/capture.py 抓下来的原始报文。本工具不依赖任何"猜"，
它做的是**受控实验**：拿同一份抓包，把「距离换算模式」4 种可能、
「z 方向」2 种可能都试一遍，把每种组合解出来的点云与历史空车样本的
分位数对齐，谁最接近就选谁。

它从抓包里回答 4 个问题
    ① 报文布局对不对   —— 包长、方位角递进步长，是否与 config.json 一致
    ② 距离换算取哪种   —— raw_offset / scale / scale_offset / scale_plus
    ③ z 方向对不对     —— z_down 是 true 还是 false（看地板在 -5 还是 +5）
    ④ 哪几个字节是啥   —— 强度字节、标志字节的候选位置

为什么能这么做？
    历史样本（baseline 里的 .xyz）是**原版程序当年解出来的点云**，
    它们代表"正确答案长什么样"：点数、x/y/z 分位数都有稳定的量级。
    一种解码模式如果解出来的点云和历史样本对不上，那它一定是错的。
    这就是把"历史样本"当成解码器的"标准答案"来用 —— 不需要原版 exe。

────────────────────────────────────────────────────────────────────────
用法
────────────────────────────────────────────────────────────────────────

    # 标准用法：抓包 + 历史参照
    python3 cli/probe.py --capture re/captures/live_20260912.bin --compare baseline

    # 怀疑 z 方向反了，强制按另一种试
    python3 cli/probe.py --capture live.bin --compare baseline --no-z-down

    # 只体检报文布局，不做历史比对
    python3 cli/probe.py --capture live.bin

    # 存档结论
    python3 cli/probe.py --capture live.bin --compare baseline --json logs/probe.json

输出末尾会直接给出**该往 config/channels.json 里填什么**，照抄即可。
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import _common
_common.setup_console()

from app.settings import baseline_rel, load_settings, load_calib
from app.pointcloud import AngleCache
from app.radar_probe import (load_capture, history_percentiles, build_report,
                             format_report)


def _resolve(path: str) -> str:
    return path if os.path.isabs(path) else os.path.join(_common.ensure_root(), path)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="probe.py", description="用真实抓包反推解码参数",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("用法")[0])
    ap.add_argument("--capture", required=True, help="抓包文件（cli/capture.py 的输出）")
    ap.add_argument("--compare", default="", metavar="DIR",
                    help="历史样本目录，作为解码参照（强烈建议给，如 %s）" % baseline_rel())
    ap.add_argument("--channels", default="", help="多通道配置路径")
    ap.add_argument("--channel", default="", help="用哪条通道的标定")
    ap.add_argument("--expect-ground", type=float, default=None, metavar="Z",
                    help="★ 期望的地面 z（米，向下为负）。空场时 ROI 里绝大多数点就是地面，"
                         "它离雷达多远 = 实测吊装高度。不给则取配置的 detection.ground_z。"
                         "这是**主判据** —— 分位数吻合度是弱证据，跨场景会选错")
    ap.add_argument("--no-expect-ground", action="store_true",
                    help="不用地面判据（退回旧的分位数比对，只在排查判据本身时用）")
    ap.add_argument("--limit-history", type=int, default=0,
                    help="参照历史样本最多用几份（0=全部）")
    ap.add_argument("--z-down", dest="z_down", action="store_true", default=None,
                    help="强制 z 向下（默认取配置）")
    ap.add_argument("--no-z-down", dest="z_down", action="store_false",
                    help="强制 z 向上")
    ap.add_argument("--json", default="", help="报告写 JSON")
    ap.add_argument("--text", default="", help="报告写纯文本（默认打印到终端）")
    ap.add_argument("-q", "--quiet", action="store_true", help="只打印结论")
    args = ap.parse_args(argv)

    st = load_settings(args.channels or "")
    chans = st.channels
    if args.channel:
        chans = [c for c in chans if c.id == args.channel] or chans
    ch = (st.enabled() or chans)[0]
    calib = load_calib(ch.abs_config_path)
    ac = AngleCache(ch.abs_angle_cache).load()
    z_down = ch.lidar.z_down if args.z_down is None else bool(args.z_down)

    cap = _resolve(args.capture)
    if not os.path.exists(cap):
        print(f"× 抓包文件不存在: {cap}")
        print("   先用 cli/capture.py 抓一份，例如：")
        print("   " + _common.PYEXE + " cli/capture.py --count 3000 --out re/captures/live.bin")
        return 1

    packets = load_capture(cap, packet_size=calib.packet.size)
    if not packets:
        print(f"× 抓包里没有解析出报文: {cap}")
        return 1

    print("=" * 78)
    print("雷达解码参数探测")
    print("=" * 78)
    print(f"抓包       : {cap}  ({len(packets)} 个报文)")
    print(f"标定       : {os.path.basename(calib.path)}  "
          f"（yaw_offset {calib.processing.yaw_offset_deg}°，"
          f"ground_z {calib.detection.ground_z}）")
    print(f"角度表     : config/angle_cache.csv  {ac.check()}")
    print(f"z 方向     : z_down = {z_down}")

    # ── 历史参照 ────────────────────────────────────────────
    reference = None
    hist_count = 0
    hist_errs = []
    hist_dir = ""
    if args.compare:
        hist_dir = _resolve(args.compare)
        if not os.path.isdir(hist_dir):
            print(f"▲  参照目录不存在，跳过历史比对: {hist_dir}")
            hist_dir = ""
        else:
            reference, hist_count, hist_errs = history_percentiles(
                hist_dir, roi=calib.roi, limit=args.limit_history)
            if reference:
                print(f"历史参照   : {hist_dir}  取 {hist_count} 份样本")
                zq = reference.get("z")
                print(f"             历史 z 2/50/98 = "
                      f"{zq[0]:.2f} / {zq[1]:.2f} / {zq[2]:.2f} m")
            else:
                print(f"▲  参照目录里没有可用样本: {hist_dir}")
                for e in hist_errs[:3]:
                    print(f"     · {e}")
    else:
        print("历史参照   : （未提供，无法比对 z 方向与量级 —— 建议加 --compare %s）"
              % baseline_rel())
    print("-" * 78)

    # 地面判据：显式给的优先；否则取配置里的 ground_z（现场应当先用
    # cli/lane_setup.py 空场量一次，再拿实测吊装高度核对这个值）。
    expect_ground = None
    if not args.no_expect_ground:
        expect_ground = (args.expect_ground if args.expect_ground is not None
                         else float(getattr(calib.detection, "ground_z", 0.0) or 0.0))
        if not expect_ground:
            expect_ground = None
        elif args.expect_ground is None and not args.quiet:
            print(f"地面判据   : 取配置的 detection.ground_z = {expect_ground:.2f} m"
                  f"（可用 --expect-ground 覆盖为实测值）")
    rep = build_report(cap, packets, calib.packet, calib.processing, ac,
                       calib.roi, z_down=z_down, reference=reference,
                       history_dir=hist_dir, hist_count=hist_count,
                       history_errors=hist_errs, expect_ground=expect_ground)

    text = format_report(rep)
    if not args.quiet:
        print(text)

    # format_report 已经把「结论」和「建议写回」打印过了，这里只补一段
    # 可直接复制粘贴的 JSON（注意：channels.json 是严格 JSON，别加注释）。
    reco = rep["recommended"] or {}
    if reco:
        print("-" * 78)
        print("可直接粘贴进 config/channels.json → channels[].lidar 的片段：")
        print("-" * 78)
        snippet = {}
        if "range_mode" in reco:
            snippet["range_mode"] = reco["range_mode"]
        snippet["z_down"] = bool(reco.get("z_down", z_down))
        if "angle_scale" in reco:
            snippet["angle_scale"] = reco["angle_scale"]
        print(json.dumps(snippet, ensure_ascii=False, indent=2))
        print("\n改完务必回归一次：" + _common.PYEXE + " cli/verify.py")
        if "angle_scale" in reco:
            print("（angle_scale 若与现有一致可不动；改动前先确认历史样本仍 10/10）")
    else:
        print("\n▲  没能给出建议 —— 抓包可能不是本雷达的报文，"
              "或 packet 段的偏移与现场不一致。")

    if args.json:
        out = _resolve(args.json)
        os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            json.dump(rep, f, ensure_ascii=False, indent=2)
        print(f"\n报告(JSON) : {out}")
    if args.text:
        out = _resolve(args.text)
        os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            f.write(text + "\n" + "\n".join(rep["verdict"]) + "\n")
        print(f"报告(文本) : {out}")

    print("=" * 78)
    return 0 if reco else 2


if __name__ == "__main__":
    raise SystemExit(main())
