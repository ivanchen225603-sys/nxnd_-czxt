#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cli/inspect_cloud.py — 单帧点云体检（现场排查第一把手术刀）

▲ 文件名为什么不叫 inspect.py？
    脚本所在目录（cli/）会被 Python 加进 sys.path 的最前面，于是 cli/ 下的
    任何文件名都会**遮蔽同名标准库**。一开始这个文件就叫 inspect.py，
    结果它把标准库的 inspect 顶掉了 —— 而 dataclasses 内部要 import inspect，
    导致所有工具启动即崩（AttributeError: module '_common' has no attribute …）。
    这类错误信息极具误导性（报在 _common 上，其实根源是文件名）。
    所以这里改成 inspect_cloud.py。
    记一条项目约定：cli/ 下不要用「标准库同名」的文件名
    （inspect / types / code / copy / test / token / parser / select / io / signal …）。

给一份 `.xyz`（或一份抓包解码出来的帧），把它"解剖"给你看：

    ① 基本统计   点数、bbox、质心、x/y/z 的 2/50/98 分位
    ② 六面拟合   前/后/左/右/上/底 六个平面：法向、内点数、是否有效
    ③ 判定链     完整走一遍 evaluate_empty，打印 11 步里卡在哪一步
    ④ 门槛对照  把实测值与 devices.json 的 7 个门槛并排，超/欠一目了然
    ⑤ 导出       --ply / --csv / --json，随手丢给上游或存档

为什么需要它？
    看板上只有一个结论（"空车"/"底面内点占比过低"）。
    当结论反直觉时，必须能下钻到数字：是点数不够？内点不够？还是 span 不够？
    这个脚本就是那个下钻入口 —— 不改配置、不写盘、纯只读。

────────────────────────────────────────────────────────────────────────
用法
────────────────────────────────────────────────────────────────────────

    # 体检一份样本
    python3 cli/inspect_cloud.py baseline/TEST001/20260721_160519.xyz

    # 只看 XYZ 分布 + 导出点云，给 CloudCompare 看
    python3 cli/inspect_cloud.py sample.xyz --ply out.ply

    # 切成 z 薄片看（沿高度方向看某一层有什么）
    python3 cli/inspect_cloud.py sample.xyz --slab -3.0 -2.0

    # 把 ROI 关掉，看原始点云（判断是不是被裁过头）
    python3 cli/inspect_cloud.py sample.xyz --no-roi

    # 机器可读
    python3 cli/inspect_cloud.py sample.xyz --json out.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import _common
_common.setup_console()

import numpy as np

from app.settings import load_calib, load_devices, load_settings, root
from app.pointcloud import (load_xyz, load_reason, apply_roi, export_ply,
                            export_csv)
from app.detect import (detect_empty, evaluate_empty, check_dims,
                        reason_detail)

PLANE_ORDER = ("front", "back", "left", "right", "top", "floor")
PLANE_CN = {"front": "前", "back": "后", "left": "左", "right": "右",
            "top": "上", "floor": "底"}


def _pcts(x: np.ndarray):
    if len(x) == 0:
        return (float("nan"),) * 3
    q = np.percentile(x, [2, 50, 98])
    return float(q[0]), float(q[1]), float(q[2])


def _resolve(path: str) -> str:
    if os.path.isabs(path):
        return path
    return os.path.join(_common.ensure_root(), path)


def _gate_table(dg, dev) -> str:
    """把实测值与门槛并排，超/欠用符号标出来。"""
    g = dev.gate
    L = []
    add = L.append
    add(f"{'项目':<18}{'实测':>12}{'门槛':>12}   判定")
    add("-" * 58)

    def row(name, val, thr, ok, fmt="{:.4f}", op="≥"):
        mark = "√ 过" if ok else "× 卡"
        add(f"{name:<18}{fmt.format(val):>12}{fmt.format(thr):>12}   {mark}  ({op} {fmt.format(thr)})")

    row("底面点数占比", dg.ratio, g.min_inlier_ratio, dg.ratio >= g.min_inlier_ratio,
        op="≥")
    add(f"{'底面内点数':<18}{dg.inliers:>12d}{g.min_inliers:>12d}   "
        f"{'√ 过' if dg.inliers >= g.min_inliers else '× 卡'}  (≥ {g.min_inliers})")
    row("底面法向|z|", dg.normal_z, g.normal_z_min, dg.normal_z >= g.normal_z_min, op="≥")
    row("内点 span X (m)", dg.span_x, g.min_span_x_m, dg.span_x >= g.min_span_x_m, op="≥")
    row("内点 span Y (m)", dg.span_y, g.min_span_y_m, dg.span_y >= g.min_span_y_m, op="≥")
    add(f"{'最少保存点数':<18}{dg.n_points:>12d}{dev.min_save_points:>12d}   "
        f"{'√ 过' if dg.n_points >= dev.min_save_points else '× 卡'}  (≥ {dev.min_save_points})")
    za = dg.z_above
    if za == za:
        add(f"{'底面高出最低点':<18}{za:>12.4f}{g.z_above_min_max_m:>12.4f}   "
            f"{'√ 过' if za <= g.z_above_min_max_m else '× 卡'}  (≤ {g.z_above_min_max_m})")
    add(f"{'点到面距离门槛':<18}{'—':>12}{g.distance_m:>12.4f}   （内点判据 ≤ 门槛）")
    return "\n".join(L)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="inspect_cloud.py", description="单帧点云体检",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("用法")[0])
    ap.add_argument("xyz", help="要体检的 .xyz 文件")
    ap.add_argument("--channels", default="", help="多通道配置路径")
    ap.add_argument("--no-roi", action="store_true", help="不裁 ROI（看原始点云）")
    ap.add_argument("--slab", nargs=2, type=float, metavar=("Z0", "Z1"),
                    help="只看 z∈[Z0,Z1] 的薄片")
    ap.add_argument("--ply", default="", help="导出 PLY 到该路径")
    ap.add_argument("--csv", default="", help="导出 CSV 到该路径")
    ap.add_argument("--json", default="", help="导出体检报告 JSON")
    ap.add_argument("-q", "--quiet", action="store_true", help="只打印结论")
    args = ap.parse_args(argv)

    path = _resolve(args.xyz)
    if not os.path.exists(path):
        print(f"× 找不到文件: {path}")
        return 1

    st = load_settings(args.channels or "")
    enabled = st.enabled() or st.channels
    ch = enabled[0]
    calib = load_calib(ch.abs_config_path)
    dev = load_devices(ch.abs_devices_path)

    try:
        fr = load_xyz(path)
    except Exception as e:                                  # noqa: BLE001
        print(f"× 读取失败: {type(e).__name__}: {e}")
        return 1

    n_raw = fr.n
    if args.slab:
        fr = fr.slice_z(args.slab[0], args.slab[1])
    if not args.no_roi:
        fr = apply_roi(fr, calib.roi)

    rep = {"file": path, "n_raw": int(n_raw), "n_used": int(fr.n),
           "roi_applied": not args.no_roi,
           "roi": None if args.no_roi or calib.roi is None else
                  [list(map(float, a)) for a in calib.roi],
           "slab": args.slab, "reason_recorded": load_reason(path)}
    # 原来只在非 -q 分支里赋值、后面却无条件使用 —— 加 -q 就 UnboundLocalError（2026-09-24 查实）
    rec = rep["reason_recorded"]

    if not args.quiet:
        print("=" * 78)
        print("单帧点云体检")
        print("=" * 78)
        print(f"文件         : {path}")
        print(f"原始点数     : {n_raw}")
        if args.slab:
            print(f"z 薄片       : [{args.slab[0]}, {args.slab[1]}]")
        print(f"裁剪后点数   : {fr.n}" + ("" if not args.no_roi else "  （--no-roi）"))
        if rec:
            print(f"原版记录理由 : {rec!r}")

    if fr.n == 0:
        print("\n× 裁剪后没有点云 —— ROI 可能裁过头了，试加 --no-roi 看原始点云")
        return 2

    xyz = fr.xyz.astype(np.float64)
    lo, hi = xyz.min(axis=0), xyz.max(axis=0)
    cen = xyz.mean(axis=0)
    pct = {ax: _pcts(xyz[:, i]) for i, ax in enumerate("xyz")}
    rep["bbox"] = {"lo": lo.tolist(), "hi": hi.tolist()}
    rep["centroid"] = cen.tolist()
    rep["extent"] = (hi - lo).tolist()
    rep["percentiles"] = {k: list(v) for k, v in pct.items()}

    if not args.quiet:
        print("-" * 78)
        print("① 基本统计")
        print("-" * 78)
        print(f"bbox     x[{lo[0]:.3f}, {hi[0]:.3f}]  "
              f"y[{lo[1]:.3f}, {hi[1]:.3f}]  z[{lo[2]:.3f}, {hi[2]:.3f}]")
        print(f"尺寸     {hi[0]-lo[0]:.3f} × {hi[1]-lo[1]:.3f} × {hi[2]-lo[2]:.3f} (m)")
        print(f"质心     ({cen[0]:+.3f}, {cen[1]:+.3f}, {cen[2]:+.3f})")
        for ax in "xyz":
            a, b, c = pct[ax]
            print(f"  {ax} 2/50/98 分位: {a:+.3f} / {b:+.3f} / {c:+.3f}")

    # ── ② 六面拟合 ──────────────────────────────────────────
    res = detect_empty(fr, calib)
    planes = {}
    if res is not None:
        d = res.to_dict()
        planes = d.get("planes", {})
    rep["planes"] = planes
    rep["dims"] = list(res.dims) if res is not None else [0.0, 0.0, 0.0]

    if not args.quiet:
        print("-" * 78)
        print("② 六面拟合")
        print("-" * 78)
        if res is None:
            print("  detect_empty 未产出结果（点数不足以拟合，或底面拟合彻底失败）")
        else:
            print(f"{'面':<5}{'有效':<6}{'内点':>8}{'a':>9}{'b':>9}{'c':>9}{'d':>10}")
            for name in PLANE_ORDER:
                p = planes.get(name, {})
                v = "√" if p.get("valid") else "—"
                print(f"{PLANE_CN[name]:<5}{v:<6}{p.get('inliers', 0):>8}"
                      f"{p.get('a', 0.0):>9.3f}{p.get('b', 0.0):>9.3f}"
                      f"{p.get('c', 0.0):>9.3f}{p.get('d', 0.0):>10.3f}")
            print(f"车厢尺寸 : {res.dims[0]:.3f} × {res.dims[1]:.3f} × {res.dims[2]:.3f} (m)")

    # ── ③ 判定链 ────────────────────────────────────────────
    # ★ 传 ground_z，与服务同口径（2026-09-24）：第 10 步兜底与"平面离地上限"都靠它。
    ok, reason, dg = evaluate_empty(res, fr.n, res.cloud if res else fr.xyz, dev,
                                    ground_z=calib.detection.ground_z)
    dim_ok, dim_msg = (check_dims(res, dev) if res is not None else (True, ""))
    # ★ reason 本身分不出第 8 步（x 跨度）和第 9 步（y 跨度）—— 两者返回
    #   **同一个**字符串「底面覆盖范围不足」，而现场处置完全不同：
    #   x 不够是车停歪/ROI 横向偏，y 不够是车厢太短/被切。
    #   reason 是红线（与原厂逐字一致，改了破 verify.py），所以在诊断侧补。
    detail = reason_detail(dg, dev)
    rep["verdict"] = {"empty": bool(ok), "reason": reason,
                      "reason_detail": detail,
                      "diagnostics": dg.to_dict(),
                      "dim_ok": bool(dim_ok), "dim_message": dim_msg}

    print("-" * 78)
    print("③ 判定结果")
    print("-" * 78)
    print(f"结论     : {'空车' if ok else '非空/异常'}    理由: {reason!r}")
    if detail:
        print(f"卡在哪   : {detail}")
    if rec:
        same = (rec == reason)
        print(f"与原版   : {'√ 一致' if same else '× 不一致（原版 ' + repr(rec) + '）'}")
    if dim_msg:
        print(f"尺寸提示 : {dim_msg}")

    if not args.quiet:
        print("-" * 78)
        print("④ 门槛对照")
        print("-" * 78)
        print(_gate_table(dg, dev))

    # ── ⑤ 导出 ──────────────────────────────────────────────
    if args.ply:
        p = _resolve(args.ply)
        export_ply(fr, p)
        print(f"\nPLY 已写出 : {p}")
    if args.csv:
        p = _resolve(args.csv)
        export_csv(fr, p)
        print(f"CSV 已写出 : {p}")
    if args.json:
        p = _resolve(args.json)
        os.makedirs(os.path.dirname(p) or ".", exist_ok=True)
        with open(p, "w", encoding="utf-8") as f:
            json.dump(rep, f, ensure_ascii=False, indent=2)
        print(f"JSON 已写出: {p}")

    # 本工具是"体检"不是"闸门"：判定为空车与否都不算失败，
    # 只要分析跑完就返回 0，方便在脚本里串起来跑一批而不被中断。
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
