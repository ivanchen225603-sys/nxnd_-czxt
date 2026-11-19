#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cli/baseline.py — 空车基线的生成 / 查看 / 校验

体积计算必须有「空车基线」：它记录**空车时每个 (x,y) 单元的车厢底板高度**，
装货后用"顶面高度 - 底板高度"做柱状积分才算得出体积（见 detect.compute_volume）。
原厂从 data/in 读这个文件，但现场从未生成过 —— 所以体积功能一直是
「无空车基线」。本工具就是补这一环。

用法
    # ① 用一份空车点云建基线（最常用：先让空车停进检测位，跑一次落盘，再拿它建）
    python3 cli/baseline.py make --xyz baseline/宁A88635W/20260721_165641.xyz --channel CH4

    # ② 直接从正在跑的数据源抓一帧空车建基线（车已停稳在位时用）
    python3 cli/baseline.py capture --channel CH4

    # ③ 看看现在有哪些基线、是否可用
    python3 cli/baseline.py show

    # ④ 拿一份载货点云验算体积（标定 voxel 用）
    python3 cli/baseline.py check --channel CH1 --xyz <载货.xyz>

★ 现场标定建议：装一车**已知体积**的货跑 ④，把实测与真值比一比，
   再决定要不要调 --voxel（默认 0.10 m）。
"""
from __future__ import annotations

import argparse
import os
import time

import _common
_common.setup_console()

from app.detect import (EmptyBaseline, baseline_quality, compute_volume,
                        detect_empty, detect_loaded)
from app.pointcloud import load_xyz, apply_roi
from app.radar_source import build_source
from app.plate import BASELINE_SUFFIX, EMPTY_PLATE, baseline_file


def _path_for(ch, plate: str) -> str:
    """★ 空车基线**按车牌存**（2026-09-23）：空车只上一次车道、载重走哪条道
    不一定，所以基线属于车不属于道。与服务 `ChannelWorker.baseline_path_for`
    走同一个 `app.plate.baseline_file` —— 各拼各的，CLI 标定的基线服务就查不到。"""
    return baseline_file(ch.abs_baseline_dir, plate)


def _require_plate(args) -> str:
    """make / capture / check 必须指定车牌。**绝不能存成「未知车牌」** ——
    那样所有认不出牌的车会共用这一份基线，恰好重新造出要修掉的错。"""
    plate = (getattr(args, "plate", "") or "").strip()
    if not plate or plate == EMPTY_PLATE:
        print("× 必须用 --plate 指定这辆车的车牌。")
        print("  空车基线按车牌存：空车只上一次车道，载重时走哪条道不一定，")
        print("  只有车牌能把同一辆车的空车/载重两趟串起来。")
        return ""
    return plate


def _describe(b: EmptyBaseline) -> str:
    x0, x1, y0, y1, z0, z1 = b.box
    return (f"底板 z={b.floor_z:.3f} m   点数={b.n}\n"
            f"    包围盒 x[{x0:.2f},{x1:.2f}]  y[{y0:.2f},{y1:.2f}]  z[{z0:.2f},{z1:.2f}]\n"
            f"    生成于 {b.created or '(未记录)'}")


def _build(frame, calib, tag: str):
    res = detect_empty(frame, calib)
    if res is None:
        return None, "六面拟合失败（点数不足或底面拟合不出来）—— 这一帧不能当基线"
    fl = res.plane("floor")
    if not fl.valid:
        return None, (f"底面不够水平（|法向z|={abs(fl.c):.4f} < 0.99）—— 这一帧不能当基线。"
                      f"确认车确实停正、ROI 没把地面/邻道扫进来")
    b = EmptyBaseline.from_empty_result(res, tag)
    if b is None:
        return None, "点云过少，无法建基线"
    return b, ""


def cmd_make(args, ctx):
    ch = ctx.channels[0]
    if ch.id != "CH4":
        print("空车基线只能由CH4生成")
        return 1
    plate = _require_plate(args)
    if not plate:
        return 1
    frame = load_xyz(args.xyz)
    n0 = frame.n
    if not args.no_roi:
        frame = apply_roi(frame, ctx.calib.roi)
    print(f"点云       : {args.xyz}")
    print(f"             {n0} 点 → ROI 裁剪后 {frame.n} 点")
    print(f"             {frame.summary()}")
    b, err = _build(frame, ctx.calib, f"{time.strftime('%Y-%m-%d %H:%M:%S')} {plate} 来自 {os.path.basename(args.xyz)} CH4")
    if b is None:
        print(f"× {err}")
        return 1
    p = _path_for(ch, plate)
    if os.path.exists(p) and not args.force:
        print(f"× {p} 已存在。确认要覆盖请加 --force（旧基线会被直接替换）")
        return 1
    b.source_channel = "CH4"
    b.save(p)
    print(f"√ {plate} 的空车基线已写入 {p}")
    print("    " + _describe(b).replace("\n", "\n    "))
    return 0


def cmd_capture(args, ctx):
    ch = ctx.channels[0]
    if ch.id != "CH4":
        print("空车基线只能由CH4生成")
        return 1
    plate = _require_plate(args)
    if not plate:
        return 1
    src = build_source(ch, ctx.calib, ctx.angle, mode=args.mode or None)
    print(f"数据源     : {src.kind}  ({src.name})")
    print(f"等待一帧点数 ≥ {args.min_points} 的空车点云（最多等 {args.wait:.0f}s）…")
    t0 = time.time()
    got = []
    try:
        # ★ 多收几帧 —— 不是为了合并（合并会把车厢盒撑长），
        #   而是为了**验证这几帧确实是同一个静止场景**。见 detect.baseline_quality。
        while time.time() - t0 < args.wait and len(got) < args.frames:
            f = src.read()
            if f is not None and f.n >= args.min_points:
                got.append(f)
                continue
            time.sleep(0.02)
    finally:
        src.close()
    if not got:
        print(f"× {args.wait:.0f}s 内没等到足够的点云（最少 {args.min_points} 点）。"
              f"确认车已停进检测位、雷达在出数（cli/capture.py 先验一下）")
        return 1
    best = got[-1]
    print(f"取到 {len(got)} 帧，用末帧: {best.summary()}")
    q = baseline_quality([f.xyz for f in got])
    if q["n"] >= 2:
        print(f"重复性     : 帧间互相关最低 {q['r_min']:.3f}"
              f"（要求 >= 0.95）{'  √' if q['ok'] else '  ×'}")
        if not q["ok"] and not args.force:
            print("× 这几帧不是同一个静止场景 —— 车可能还在动，或场地里有别的东西在变。")
            print("  2026-09-15 现场那两次空车标定就是这样（r=0.799 / 0.865，"
                  "而同批载货是 0.986 / 1.000）；")
            print("  拿它建基线会让这辆车的体积一直偏约 4%，而且**事后看不出来**。")
            print("  等车彻底停稳再标一次；确实要硬存就加 --force。")
            return 1
    elif len(got) == 1:
        print("▲ 只收到 1 帧，**没法验重复性** —— 这次标定的可信度未知。")
    b, err = _build(best, ctx.calib, f"{time.strftime('%Y-%m-%d %H:%M:%S')} {plate} 现场抓取 {ch.id}")
    if b is None:
        print(f"× {err}")
        return 1
    p = _path_for(ch, plate)
    if os.path.exists(p) and not args.force:
        print(f"× {p} 已存在，要覆盖请加 --force")
        return 1
    b.source_channel = "CH4"
    b.save(p)
    print(f"√ {plate} 的空车基线已写入 {p}")
    print("    " + _describe(b).replace("\n", "\n    "))
    return 0


def cmd_show(args, ctx):
    """列出**已标定的车辆**（按车牌），不再按通道 —— 基线属于车不属于道。"""
    d = ctx.channels[0].abs_baseline_dir if ctx.channels else ""
    print(f"基线目录 {d}（四条车道共用）")
    print("-" * 78)
    try:
        names = sorted(os.listdir(d))
    except OSError:
        names = []
    plates, legacy = [], []
    for name in names:
        if not name.endswith(BASELINE_SUFFIX):
            continue
        stem = name[: -len(BASELINE_SUFFIX)]
        (legacy if stem.upper().startswith("CH") and stem[2:].isdigit() else plates).append(stem)
    if not plates:
        print("× 还没有任何车辆的空车基线 —— 体积会一直报「无空车基线」。")
        print("  空车停进CH4后：cli/baseline.py capture --channel CH4 --plate <车牌>")
    for plate in plates:
        b = EmptyBaseline.load(os.path.join(d, plate + BASELINE_SUFFIX))
        print(f"── {plate} ".ljust(78, "─"))
        if b is None or not b.n:
            print("    × 基线文件坏了或是空的，重新标定")
        else:
            print("    √ " + _describe(b).replace("\n", "\n    "))
    for stem in legacy:
        print(f"▲ 发现旧的车道级基线 {stem}{BASELINE_SUFFIX} —— 它**不会再被使用**"
              "（基线已改为按车牌存）。确认无用后可以删掉。")
    return 0

def cmd_check(args, ctx):
    ch = next((c for c in ctx.settings.channels if c.id == "CH4"), ctx.channels[0])
    plate = _require_plate(args)
    if not plate:
        return 1
    b = EmptyBaseline.load(_path_for(ch, plate))
    if b is None or b.source_channel != "CH4":
        print(f"× {plate} 没有已确认CH4来源的基线，请在CH4重新采集（--plate {plate}）")
        return 1
    frame = load_xyz(args.xyz)
    if not args.no_roi:
        frame = apply_roi(frame, ctx.calib.roi)
    print(f"基线       : {_describe(b)}")
    print(f"载货点云   : {frame.summary()}")
    lr = detect_loaded(frame, ctx.calib)
    vr = compute_volume(b, lr, voxel=args.voxel, shared_baseline=True)
    print("-" * 78)
    print(f"体积结果   : {vr.text or vr.message}")
    if args.truth > 0:
        err = (vr.volume_m3 - args.truth) / args.truth * 100.0
        print(f"真值 {args.truth:.3f} m^3 → 相对误差 {err:+.1f}%")
        print("    偏大多半是边缘单元量化（格子被货占一半也整格算），把 --voxel 调小试试；")
        print("    偏小多半是货堆顶面回波稀疏，或 ROI 把货堆边缘裁掉了。")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="baseline.py", description="空车基线生成 / 查看 / 校验",
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        p.add_argument("--channels", default="", help="多通道配置路径")
        p.add_argument("--channel", action="append", default=[], metavar="ID")
        p.add_argument("-q", "--quiet", action="store_true")

    p = sub.add_parser("make", help="用一份空车 .xyz 建基线")
    common(p); p.add_argument("--xyz", required=True); p.add_argument("--no-roi", action="store_true")
    p.add_argument("--plate", default="", help="这辆车的车牌（基线按车牌存，必填）")
    p.add_argument("--force", action="store_true", help="覆盖已有基线")
    p.set_defaults(f=cmd_make)

    p = sub.add_parser("capture", help="从数据源抓一帧空车建基线")
    common(p); p.add_argument("--mode", default="", help="覆盖模式 live/replay/simulate")
    p.add_argument("--plate", default="", help="这辆车的车牌（基线按车牌存，必填）")
    p.add_argument("--wait", type=float, default=30.0, help="最多等多少秒")
    p.add_argument("--min-points", type=int, default=20000, help="最少点数")
    p.add_argument("--frames", type=int, default=3,
                   help="收几帧做重复性校验（不合并，只用末帧；1 = 不校验）")
    p.add_argument("--force", action="store_true",
                   help="覆盖已有基线；重复性不合格时也强行写入")
    p.set_defaults(f=cmd_capture)

    p = sub.add_parser("show", help="列出已标定空车基线的车辆（按车牌）")
    common(p); p.set_defaults(f=cmd_show)

    p = sub.add_parser("check", help="用载货点云验算体积")
    common(p); p.add_argument("--xyz", required=True); p.add_argument("--no-roi", action="store_true")
    p.add_argument("--plate", default="", help="这辆车的车牌（基线按车牌存，必填）")
    p.add_argument("--voxel", type=float, default=0.10, help="积分单元边长（米）")
    p.add_argument("--truth", type=float, default=0.0, help="已知真值 m^3，给了就算相对误差")
    p.set_defaults(f=cmd_check)

    args = ap.parse_args(argv)
    ctx = _common.load_ctx(args)
    return args.f(args, ctx)


if __name__ == "__main__":
    raise SystemExit(main())
