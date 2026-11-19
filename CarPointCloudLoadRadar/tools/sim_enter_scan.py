#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/sim_enter_scan.py —— 「进场途中扫」能否补上车厢前端的货面数据（docs/48 §4.4）

▲ 方法论教训（本脚本第一版踩过，留档防重走）：
  把停稳点云整体平移 -s 再映射回 +s 是**恒等变换**，覆盖逐位不变 ——
  采集到的点里已经烤死了原位置的视线几何，刚体平移把"缺口"也一起搬走，
  它**不等于**车真的停在别的位置（雷达没动，视线全变了）。
  同理，"环向径向采样间距"模型也被数据否决：环距实测 0.325 度，
  预测 d=5 m 处覆盖 ~100%，实测只有 32~41%。
  => 真正的机制是**掠射回波衰减**（docs/13 §14：俯角掉到 16~28 度回波断掉）。

所以本脚本只做两件**可从数据直接支撑**的事：
  [1] 从两辆真值车的实测点云量出「覆盖率和离天底水平距离」的传递曲线
      （等价 docs/48 §4.1，按格重算）——它已包含几何+回波强度全部物理；
  [2] 把车厢前端各格的"停车位置距离"沿进场方向回退 s 米，查曲线，
      得到补满前端所需的后退量；再给出配准误差的体积敏感性（实测扰动）。
"""
from __future__ import annotations

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import numpy as np                                              # noqa: E402

from app.detect import (EmptyBaseline, VOLUME_CELLS, analyze_load,  # noqa: E402
                        compute_volume_auto, detect_empty, detect_loaded)
from app.pointcloud import apply_roi, load_xyz                  # noqa: E402
from app.settings import load_calib                             # noqa: E402

RAW = os.path.join(ROOT, "data", "field", "raw", "CH4")
CELL = 0.10


def load_frame(ts: str, calib):
    return apply_roi(load_xyz(os.path.join(RAW, "CH4_20260915_%s_3of3.xyz" % ts)), calib.roi)


def cover_by_distance(a: np.ndarray, box, cell: float = CELL):
    """盒内按格算覆盖，并按"格心离天底水平距离"分桶 -> 传递曲线。"""
    bx0, bx1, by0, by1 = box
    nx = max(1, int(round((bx1 - bx0) / cell)))
    ny = max(1, int(round((by1 - by0) / cell)))
    ix = np.floor((a[:, 0] - bx0) / cell).astype(np.int64)
    iy = np.floor((a[:, 1] - by0) / cell).astype(np.int64)
    m = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    hit = np.zeros((ny, nx), bool)
    hit[iy[m], ix[m]] = True
    # 格心坐标与离天底距离
    cy = by0 + (np.arange(ny) + 0.5) * cell
    cx = bx0 + (np.arange(nx) + 0.5) * cell
    dist = np.hypot(cx[None, :], cy[:, None])            # (ny, nx)
    return hit, dist


def curve_table(hit: np.ndarray, dist: np.ndarray, band: float = 0.5):
    rows = []
    for lo in np.arange(0.0, 6.5, band):
        m = (dist >= lo) & (dist < lo + band) & np.isfinite(dist)
        if m.sum():
            rows.append((lo, float(hit[m].mean())))
    return rows


def front_band_stats(hit: np.ndarray, dist: np.ndarray, y1: float, by0: float,
                     cell: float = CELL, width: float = 1.5):
    """盒前端最外 width 米、按当前"停车位置距离"分段的覆盖率。"""
    m = dist >= (y1 - width) - 1e-9
    return float(hit[m].mean()), int(m.sum())


def run(plate: str, ets: str, lts: str, weight: float, calib, out):
    print("=" * 78)
    print("%s   磅单 %.2f t" % (plate, weight))
    print("=" * 78)

    empty = load_frame(ets, calib)
    res = detect_empty(empty, calib)
    base = EmptyBaseline.from_empty_result(res, "sim_enter")
    bc = np.asarray(base.cloud, np.float64)
    near = np.abs(bc[:, 2] - base.bed_z_at(bc[:, 0], bc[:, 1])) <= 0.5
    box = (float(bc[near, 0].min()), float(bc[near, 0].max()),
           float(bc[near, 1].min()), float(bc[near, 1].max()))
    print("  积分域: x[%.2f,%.2f] y[%.2f,%.2f]  长 %.2f m"
          % (box[0], box[1], box[2], box[3], box[3] - box[2]))

    main = load_frame(lts, calib)
    loaded = detect_loaded(main, calib)
    a0 = np.asarray(main.xyz, np.float64)

    hit, dist = cover_by_distance(a0, box)
    print()
    print("  [1] 实测传递曲线（本车 0.5 m 距离带的格子覆盖率）")
    for lo, cov in curve_table(hit, dist):
        print("      距天底 %.1f~%.1f m: 覆盖 %5.1f%%" % (lo, lo + 0.5, cov * 100))
    y1 = box[3]
    cov0, n0 = front_band_stats(hit, dist, y1, box[2])
    print("      前端最外 1.5 m（停车时距天底 %.2f~%.2f m）: 覆盖 %.1f%%（%d 格）"
          % (y1 - 1.5, y1, cov0 * 100, n0))

    # [2] 进场预测：前端各格在车还差 s 米时位于 距天底 d-s 处，查曲线
    print()
    print("  [2] 进场预测（把前端格的距离按 -s 回退后查上面这条实测曲线）")
    band = dist >= (y1 - 1.5) - 1e-9
    d_front = dist[band]                      # 停车时这些格距天底
    for s in (1.0, 1.5, 2.0, 2.5, 3.0):
        d2 = np.clip(d_front - s, 0.0, None)
        # 查曲线：取所在 0.5 m 桶的实测覆盖
        idx = np.minimum((d2 / 0.5).astype(int), 12)
        lut = np.array([c for _, c in curve_table(hit, dist)] + [1.0])
        pred = lut[idx].mean()
        print("      后退 %4.1f m -> 前端带落到距天底 %.2f~%.2f m，预测覆盖 ~%5.1f%%"
              % (s, d2.min(), d2.max(), pred * 100))

    # [3] 配准误差敏感性（有效扰动：extra 货面整体错位 e 米后的体积漂移）
    prof = analyze_load(bc)
    bed_z = float(prof.bed_z) if prof.ok else None
    kw = dict(cells=VOLUME_CELLS, spread_target=.03, floor_band_m=.5,
              bed_shift_max_m=.5, dy=None, bed_z=bed_z, bed_evidence=None,
              bed_scope=None, volume_context=None, volume_scope=None)
    v0 = compute_volume_auto(base, loaded, **dict(kw, loaded_extra=None))
    print()
    print("  [3] 配准误差敏感性（进场帧错位 e 米 -> 体积漂移；extra 机制现成）")
    print("      只用停稳帧: %.3f m3  密度 %.4f  hole %.1f%%"
          % (v0.volume_m3, weight / v0.volume_m3, v0.hole_ratio * 100))
    for e in (0.05, 0.10, 0.25):
        # 用"主帧 + 复制错位帧"模拟：错位的进场帧喂给高度图后的体积变化
        bad = [(a0 + np.array([0, e, 0])).astype(np.float32),
               (a0 - np.array([0, e, 0])).astype(np.float32)]
        vr = compute_volume_auto(base, loaded, **dict(kw, loaded_extra=bad))
        print("      e=%+.2f m -> %.3f m3  密度 %.4f  漂移 %+.2f%%"
              % (e, vr.volume_m3, weight / vr.volume_m3,
                 (vr.volume_m3 / v0.volume_m3 - 1) * 100))
    print()


def main() -> int:
    calib = load_calib(os.path.join(ROOT, "config", "config.json"))
    print("前提：雷达转速 10 Hz（帧周期约 100 ms）。0.5 m/s 进场 -> 每帧位移 5 cm，")
    print("      帧内拖影 5 cm < 体素 0.10 m；1.0 m/s 时拖影与体素同量级，需逐帧补偿。")
    print()
    for plate, ets, lts, w in (("宁A81530M", "161309", "152838", 53.60),
                               ("宁A42264Z", "161538", "153502", 48.64)):
        run(plate, ets, lts, w, calib, None)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
