#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
detect.py — 检测算法与判定链

包含四件事：
    1. detect_empty   六面拟合 → 判空车        （对应 detectempty.dll）
    2. detect_loaded  四面拟合 + 体积          （对应 detectloaded.dll）
    3. StabilityDetector 停稳判定              （对应 stopdetect.dll）
    4. evaluate_empty 判定链                   （对应 calibrate_test.exe 内的判定逻辑）

★ 已验证的部分
    evaluate_empty 的**判定顺序与阈值**逐条对齐了 calibrate_test.exe @0x1400124C0 的反汇编，
    并在 10/10 历史样本上与真实 reason.txt 完全一致。
    StabilityDetector 的偏差度量经受控实验精确验证（误差 < 1e-7）。

☆ 仍需现场校准的部分（代码里都标了 TODO(现场)）
    · 载货体积的离散方式 —— 原版依赖"空车基线"，而现场从未生成过基线，
      所以这段逻辑在原系统里**一次都没跑过**，没有任何可比对的输出。
    · 雷达 UDP 解码 —— 见 app/radar/packet.py
"""
from __future__ import annotations

import os
import math
import time
import threading
from dataclasses import dataclass, field, asdict, replace
from collections import deque
from typing import Deque, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

from .pointcloud import Frame
from .geometry import Plane, fit_plane_pca, fit_plane_ransac, correct_heading
from .bed_reference import BedReferenceEvidence, BedReferenceScope, resolve_bed_shift
from .volume_context import VolumeContext, context_error, integrate_context
from .occlusion_fill import OcclusionFillInfo

# ═══════════════════════════════════════════════════════ 判定理由常量
# 与原厂 calibrate_test.exe 的 .rdata 中文字符串完全一致（顺序即判定优先级）

R_INSUFFICIENT_POINTS = "点云数量不足"
R_EMPTY_FAILED = "空车检测失败"
R_NO_FLOOR = "未见厢底平面"
R_FLOOR_NOT_LEVEL = "底面不够水平"
R_TOO_FEW_CORRECTION = "校正点云过少"
R_FLOOR_FEW_INLIERS = "底面内点不足"
R_FLOOR_LOW_RATIO = "底面内点占比过低"
R_FLOOR_COVERAGE = "底面覆盖范围不足"
R_FLOOR_TOO_HIGH = "底面过高，疑似载物"
R_PASS_EMPTY = "空车"
R_SIZE_BAD = "尺寸不达标"
R_NO_BASELINE = "无空车基线"
R_VOLUME_DONE = "已完成载货体积计算"
R_VOLUME_SKIPPED = "未测体积"

# ═══════════════════════════════════════════════════════════════════
# 体积可用性的两道门槛 —— **只在这里定义一次**
# ═══════════════════════════════════════════════════════════════════
#
# 以前这两个数在 `VolumeResult.confidence()` 的默认参数和
# `load_radar_contract.build_measurement()` 里**各写了一份**，改一处
# 另一处不会跟着动。收到这里来。
#
# VOLUME_HOLE_MAX —— 体积里有多少落在**真空洞**上（不含厢壁盲区）。
#   量的是**这一次测量的质量**：扫得全不全。
#   实测（2026-09-18，两辆有磅单真值的车）13.7% / 14.8%，
#   门槛 15% 是够得着的，也确实有区分度。
# 2026-11-03：此值仅保留内部诊断/档位选择兼容性，不再决定正式发布或低置信标签。
VOLUME_HOLE_MAX = 0.15
#
# VOLUME_SPREAD_MAX —— 条件范围的相对宽度 `(hi - lo) / hi`。
#   ▲▲ **这个数是按已经退役的口径标定的，现有几何下够不着。**
#   · 它来自 docs/13 §17：当时 `bound_lo/hi` 是"格子扫描的上下界"，
#     自适应格子能把它压到 1.6% / 1.8%，于是门槛定 3%；
#   · 而 `volume_context.integrate_context()` 换成了**声明模型下的区间**，
#     上界要把**没观测到的面积**按 `max_height_m` 整个算进去。
#   · 2026-09-18 实测那两辆车：未观测面积占 34% / 39%，
#     扫遍 5 档格子 × 4 档货高上限 × 误差预算减半，
#     **spread 最低只能到 46%**（体积估值本身很稳，密度 0.950~0.994）。
#   => 也就是说这道门槛**永远关着**：`volumeStatus` 不可能成为 "success"，
#      煤量永远进不了正式入库。这不是哪辆车的问题，是几何决定的。
#   ▲▲ **2026-09-18 现场决定：这条不再当闸门**（`None` = 不设门槛）。
#     理由是现场给的：「用户没时间看这个数字，最终会进入第三方系统，
#     他们会去处理」。也就是说质量筛选放到下游做，源端只管**把数和
#     质量指标都报出去**，不在这里拦。
#     · `bound_spread` 照常计算、照常写进结果与上报载荷，**一个字段没少**；
#     · 变的只是它不再参与 `volumeStatus` 的判定。
#   ▲ 真要重新启用，把 `None` 换回一个数即可；但先读 §3.2：
#     现有几何下它的下限是 46%，设成 3% 等于永久关闭。
#   ▲ 后台侧 2026-09-22 已对齐：原先 `LoadRadarPublicationPolicy` /
#     `LoadRadarService.Publish` / `LoadRadarRepository.Publish` **三处**
#     各写一份 `InRange(BoundSpread, 0m, 0.03m)`（源端放行它照样拒发煤量），
#     现已合并成一条规则、阈值改成配置 `LoadRadar:VolumeBoundSpreadMax`，
#     默认 null = 不设门槛。要恢复硬门槛在后台配置里给值即可。
#   见 docs/33-体积可用性门槛.md。
VOLUME_SPREAD_MAX = None

PLANE_ORDER = ("floor", "roof", "left", "right", "front", "back")
PLANE_CN = {"floor": "底面", "roof": "顶面", "left": "左壁",
            "right": "右壁", "front": "前壁", "back": "后壁"}


# ═══════════════════════════════════════════════════════ 空车检测

@dataclass
class EmptyResult:
    """对应原版 detectempty 的 DetectResult（184 字节结构体）。"""
    planes: List[Plane] = field(default_factory=lambda: [Plane() for _ in range(6)])
    dims: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    cloud: np.ndarray = field(default_factory=lambda: np.zeros((0, 3), np.float32))
    yaw: float = 0.0
    ok: bool = False
    # 附加追溯信息，不参与任何拟合、判定或原厂回归输出。
    source_indices: Optional[np.ndarray] = None
    source_rotation: Optional[np.ndarray] = None

    @property
    def fit_ok(self) -> bool:
        """`ok` 的**正名**：它的含义是"六面拟合成功了"，与空不空车没关系。

        ▲ `ok` 这个名字与直觉相反，已经咬过一次：2026-09-17 查实落盘
          `meta.empty` 写的就是 `res.ok`，137 份现场样本里 **115 份**
          标着 empty=True 而实际是载货车（docs/13 §0）。
          在"按车牌存空车基线"那套设计里这是致命的 —— 照它挑基线 =
          把载货点云存成空车基线，之后那辆车的体积恒为接近 0 且不报错。

        `ok` 保留不动（看板契约 `to_dict()` 里钉着它）；
        **新代码一律用 `fit_ok`**，一眼就知道它不是"空车"的意思。
        判定链的结论要用 `evaluate_empty()` 的返回值，不是这里。
        """
        return bool(self.ok)

    def plane(self, name: str) -> Plane:
        return self.planes[PLANE_ORDER.index(name)]

    def to_dict(self) -> Dict:
        return {
            # `ok` 与 `fit_ok` 是同一个值，两个都给：
            # 前者为了不破看板契约，后者为了让读的人不误解（见 fit_ok 的说明）。
            "ok": self.ok,
            "fit_ok": self.fit_ok,
            "yaw_rad": round(self.yaw, 6),
            "yaw_deg": round(math.degrees(self.yaw), 4),
            "dims": {"x": round(self.dims[0], 4), "y": round(self.dims[1], 4),
                     "z": round(self.dims[2], 4)},
            "planes": {PLANE_ORDER[i]: {**{k: round(v, 6) for k, v in
                                          zip(("a", "b", "c", "d"), p.as_tuple())},
                                        "valid": bool(p.valid), "inliers": int(p.inliers),
                                        "rms": round(p.rms, 5)}
                       for i, p in enumerate(self.planes)},
        }


def _pca_oriented(pts: np.ndarray) -> Optional[Plane]:
    """
    PCA 最小二乘平面 + 朝向规范化。

    朝向规范化的意义：车厢这种结构化场景里，同一个面会以 ±n 两种等价形式出现，
    不统一的话「底面法向.z」的符号会随机翻转，后续 |c| 判断与 dims 都会飘。
    规则：若法向已明显偏向 z 轴（|c| > 0.5），就统一让 c > 0（即"朝上"）。
    """
    p = fit_plane_pca(pts)
    if p is None:
        return None
    n = p.norm()
    if n > 1e-9 and abs(p.c) > 0.5 * n and p.c < 0:
        p = p.flip()
    return p


CARRIAGE_JITTER = (
    {}, {"w_lo": 2.10}, {"w_lo": 2.20}, {"y_bin": 0.20}, {"y_bin": 0.30},
    {"gap_m": 0.40}, {"gap_m": 0.60}, {"x_lim": 1.5}, {"x_lim": 1.7},
)


def carriage_box(xyz: np.ndarray, y_bin: float = 0.25,
                 w_lo: float = 2.15, w_hi: float = 2.80,
                 gap_m: float = 0.5, jump_m: float = 0.8,
                 min_pts: int = 15, x_lim: float = 1.6,
                 refine: bool = True, step: float = 0.05, w_slack: float = 0.15,
                 z_band: Optional[Tuple[float, float]] = None
                 ) -> Optional[dict]:
    """按**横截面**把车厢从整片点云里切出来。返回 dict 或 None。

    与 `carriage_y_window` 的区别（两者互不替代）：
      · `carriage_y_window` 沿 y **数点数**，只回答"要不要把地面甩掉"；
      · 本函数沿 y **量截面宽度**，回答"车厢本体在哪"。

    判据（逐 y 格）：
      ① 宽度 w = x 的 p2..p98 落在 [w_lo, w_hi]（**带 w_slack 回差**，见下）。
         车厢内宽是现场最稳的量：
         72 份样本（62 现场 + 10 回归基线）实测 2.24~2.53 m，**标准差 0.070 m**。
         驾驶室外侧、门架、地面、旁边的车都会明显更宽或更窄。
      ② 顶面高度相邻格不跳变（|dtop| <= jump_m）。驾驶室顶比厢顶低约 1.4 m，
         一跳就断开。
      ③ 断口容忍 gap_m —— 驾驶室与车厢之间有物理缝。
    车厢 = 满足 ①②③ 的连续段里点数最多的那一段，端点再做亚格精修。

    ★ **对已裁 ROI 的点云不依赖 ground_z**（判定链与落盘 .xyz 都是这种）。
      2026-09-16 实测：加不加"离地 0.3~4.8 m"的高度带，72 份样本端点差
      中位 0.000 m、最大 0.04 m —— 宽度判据本身就把地面甩掉了：车所在的
      那一段 y 上，车把两侧地面全遮住了（雷达吊在 5.2 m、车高 4 m、半宽
      1.25 m，地面阴影伸到 |x| = 5.4 m，远超 ROI 的 |x| <= 2），所以车厢格
      只有车厢自己（2.4~2.6 m 宽），空车道格是整片 4 m 宽的地面。
      这条很重要：`ground_z` 是配置里写死的绝对高度，是本项目已知最大的
      弱点（见 CLAUDE.md 设计第 3 条），车厢定位不该再挂到它上面。

    ▲ **未裁 ROI 的整帧必须给 z_band**（`cli/lane_setup.py --dump-raw` 存的
      那种，`cli/roi_plot.py` 读的就是）。厂房顶在离地 5.1~5.5 m，正好也在
      |x| < x_lim 里，会把每一格的截面宽度顶满 -> 一份都切不出来。
      这时传 `z_band=(ground_z + 0.30, ground_z + 4.80)`。
      ROI 的 z 顶盖是 -0.75（离地 4.48 m），本来就把屋顶挡在外面，所以
      已裁 ROI 的点云传 None 即可。

    ▲ **x 准，y 不准 —— 用之前必看 `carriage_confidence()`。**
      · x / 内宽：72/72 都切得出，标准差 0.070 m，可以当准确值用；
      · y / 两端：在参数抖动下只有 **57%** 稳定（摆幅 <= 0.25 m），
        其余 43% 摆 0.30~6.25 m，且**厢长本身**也摇（28/72 摆幅 > 0.5 m）。
        根因是传感器几何不是算法：俯视单雷达在车厢远端接近掠射，
        前端覆盖只有 43~71%，数据里根本没有清晰的端面。
      所以下游要么只用 x，要么先问置信度、不可信的交给人。
    """
    if xyz is None or len(xyz) < 300:
        return None
    m = np.abs(xyz[:, 0]) < x_lim
    if z_band is not None:
        m &= (xyz[:, 2] > z_band[0]) & (xyz[:, 2] < z_band[1])
    p = xyz[m]
    if len(p) < 300:
        return None
    # ★ 分格**锚在世界坐标的整数倍**上，不要锚在 p[:,1].min()。
    #   min() 会被几个杂点左右，每帧都不一样 -> 整张网格跟着平移 ->
    #   每一处"断没断开"的判定都可能翻面。2026-09-16 实测 CH4_132902：
    #   真实空档 0.75 m，按 min() 锚定只量到 0.50 m（正好不触发 gap_m=0.5），
    #   于是驾驶室被并进车厢，三帧结果摆 5.28 m。
    lo = float(np.floor(p[:, 1].min() / y_bin) * y_bin)
    hi = float(np.ceil(p[:, 1].max() / y_bin) * y_bin)
    edges = np.arange(lo, hi + y_bin * 0.5, y_bin)
    if len(edges) < 6:
        return None
    nb = len(edges) - 1
    idx = np.clip(np.digitize(p[:, 1], edges) - 1, 0, nb - 1)
    # 按格号**稳定**排序一次再切片：每格拿到的点与 p[idx == i] 完全相同（同一批行、同一顺序），
    # 结果逐位不变；原来每格都扫一遍全部点（2026-10-01 提速）。
    # 格号用 int16 排：numpy 对 16 位整数的稳定排序走基数排序，比 int64 快得多。
    order = np.argsort(idx.astype(np.int16) if nb < 32767 else idx, kind="stable")
    ps = p[order]
    starts = np.searchsorted(idx[order], np.arange(nb + 1))
    ok = np.zeros(nb, dtype=bool)          # 够严格，可以**开一段**
    cont = np.zeros(nb, dtype=bool)        # 够松，可以**续一段**（回差）
    top = np.full(nb, np.nan)
    cnt = np.zeros(nb, dtype=np.int64)
    wid = np.full(nb, np.nan)
    ymin = np.full(nb, np.nan)          # 格内实际最小/最大 y，用来量真实空档
    ymax = np.full(nb, np.nan)
    for i in range(nb):
        s = ps[starts[i]:starts[i + 1]]
        if len(s) < min_pts:
            continue
        w = float(np.percentile(s[:, 0], 98) - np.percentile(s[:, 0], 2))
        wid[i] = w
        cnt[i] = len(s)
        top[i] = float(np.percentile(s[:, 2], 98))
        ymin[i] = float(s[:, 1].min())
        ymax[i] = float(s[:, 1].max())
        ok[i] = w_lo <= w <= w_hi
        cont[i] = (w_lo - w_slack) <= w <= (w_hi + w_slack)
    if not ok.any():
        return None
    # ★ 宽度判据带**回差**：开一段要够严（ok），续一段可以放宽 w_slack（cont），
    #   但端点只由严格格定 —— 松格只负责"桥接"，不许把端点往外拽。
    #
    #   为什么需要：雷达正下方只打到货面顶，偏开才扫到厢壁顶，于是
    #   **y≈0 处量出来的宽度系统性偏小**（"天底腰"）。2026-09-16 实测 62 份：
    #   55 份天底更窄，中位窄 0.062 m、p90 窄 0.138 m。
    #   CH2_143529 的天底腰是 2.14，正好卡在 w_lo=2.15 下面 —— 三帧里有一帧
    #   两格都没过，断口凑成 0.75 m，**车厢从中间被劈成两半**，
    #   再按点数选走了左边那半（5.25 m，真车厢 11.15 m）。
    #   w_slack=0.15 是照天底腰的 p90 定的。
    runs, cur, last = [], None, None
    for i in range(nb):
        if not cont[i]:
            continue
        # ★ 空档按**点的实际位置**量，不要数空格子数。
        #   数格子会把真实空档量成"格数 x y_bin"，误差最大一整格 ——
        #   卡在阈值上的那些（CH4_132902 真实 0.75 m 被量成 0.50 m）就会漏断。
        brk = cur is not None and (
            (ymin[i] - ymax[last]) > gap_m
            or abs(top[i] - top[last]) > jump_m)
        if cur is None or brk:
            if not ok[i]:
                continue                   # 松格不许起头
            if cur is not None:
                runs.append(cur)
            cur = [i, i]
        elif ok[i]:
            cur[1] = i                     # 只有严格格才把端点往外推
        last = i
    if cur is not None:
        runs.append(cur)
    if not runs:
        return None
    best = max(runs, key=lambda r: cnt[r[0]:r[1] + 1].sum())
    y0, y1 = float(edges[best[0]]), float(edges[best[1] + 1])
    if refine:
        y0 = _carriage_edge(p, y0, y_bin, w_lo, w_hi, +1, step)
        y1 = _carriage_edge(p, y1, y_bin, w_lo, w_hi, -1, step)
    if y1 - y0 < 2.0:
        return None
    sel = p[(p[:, 1] >= y0) & (p[:, 1] <= y1)]
    if len(sel) < 100:
        return None
    return {
        "y0": y0, "y1": y1, "length": y1 - y0,
        "x0": float(np.percentile(sel[:, 0], 1)),
        "x1": float(np.percentile(sel[:, 0], 99)),
        "width": float(np.nanmedian(wid[best[0]:best[1] + 1])),
        "n_points": int(len(sel)), "n_runs": len(runs),
    }


def _carriage_edge(p: np.ndarray, guess: float, y_bin: float,
                   w_lo: float, w_hi: float, sign: int, step: float,
                   win: float = 0.30, need: int = 3) -> float:
    """端点亚格精修：在 guess +- 一格内以 step 扫，用 win 厚的滑窗量宽度。

    用滑窗而不是单片，是因为 0.05 m 一片只有几十个点，p2..p98 会被噪声打散。
    sign=+1 找后沿（从小 y 往大 y 扫），-1 找前沿。
    """
    cands = np.arange(guess - y_bin, guess + y_bin + step, step)
    if sign < 0:
        cands = cands[::-1]
    # 先用一次掩码截出所有窗口覆盖的那一窄条 y，再在窄条里逐窗筛：每个窗口取到的点
    # 与在整片 p 上筛完全相同（同一批行、同一顺序），只是不再每窗扫全部点（2026-10-01 提速）。
    lo_y = float(min(cands.min(), cands.max())) - win
    hi_y = float(max(cands.min(), cands.max())) + win
    band = p[(p[:, 1] >= lo_y) & (p[:, 1] < hi_y)]
    run = 0
    for c in cands:
        a, b = (c, c + win) if sign > 0 else (c - win, c)
        s = band[(band[:, 1] >= a) & (band[:, 1] < b)]
        if len(s) < 12:
            run = 0
            continue
        w = float(np.percentile(s[:, 0], 98) - np.percentile(s[:, 0], 2))
        if w_lo <= w <= w_hi:
            run += 1
            if run >= need:
                return float(c - sign * (need - 1) * step)
        else:
            run = 0
    return float(guess)


def carriage_confidence(xyz: np.ndarray, tol: float = 0.25,
                        z_band: Optional[Tuple[float, float]] = None) -> dict:
    """自举置信度 —— **已弃用，不要在新代码里用它**。

    弃用理由（2026-09-16 实测，下面 ▲▲ 那段有完整数据）：它判"可信"的 15 组里
    真稳的只有 6 组（**40%**），与"存疑"组几乎无区分度。要判帧间稳不稳，
    用 `carriage_agreement(多帧)` —— 它量的就是要的那件事。
    ▲ 2026-09-19 补：也别指望 `carriage_box` 返回的 `n_runs` 当可信标记 ——
      62 组实测 r(n_runs, 厢长离散) = **-0.458**（方向是反的），
      `n_runs==1` 的组反而更不稳（离散 1.400 vs 1.250）。

    做法：同一片点云用 CARRIAGE_JITTER 里各组参数各切一次，看答案稳不稳。

    为什么必须有这个：`carriage_box` 的 y 端点是一个**离散**决策
    （"哪一段赢"），边界附近参数动一点答案就整段翻面 —— 2026-09-16 实测
    `w_lo` 只挪 0.05 就有 9/72 份的端点跳 6 m 之多。这种错法不会自己暴露：
    同一辆车连续帧给的答案完全一致（真-同车对上厢长差 0.00~0.20 m），
    **它是可复现的偏差，不是噪声**，光看重复性会以为很准。

    ▲▲ **它量的是"对参数敏不敏感"，不是"准不准"，也预测不了帧间一致性。**
      2026-09-16 拿 58 组真-停稳三连帧（质心 y 位移 <= 0.10 m、车点数变化 < 2%）
      验证过：判"可信"的 15 组里，帧间端点离散真的 <= 0.30 m 的只有 **6 组
      （40%）**；判"可信"组的帧间离散中位 **2.60 m**，判"存疑"组 2.96 m ——
      **几乎没有区分度**。要判帧间稳不稳，用 `carriage_agreement()` 直接量。
      这里留着它只是因为"参数一动答案就跳"本身值得知道。

    返回 {"ok", "box", "sway", "length_sway", "n"}：
      ok          所有抖动解的端点摆幅 <= tol
      box         基准参数下的解（ok=False 时仍给，供人参考）
      sway        端点最大摆幅（米）
      length_sway 厢长最大摆幅（米）
    """
    out = []
    for kw in CARRIAGE_JITTER:
        b = carriage_box(xyz, z_band=z_band, **kw)
        if b is None:
            return {"ok": False, "box": None, "sway": float("inf"),
                    "length_sway": float("inf"), "n": 0}
        out.append(b)
    y0 = [b["y0"] for b in out]
    y1 = [b["y1"] for b in out]
    ln = [b["length"] for b in out]
    sway = max(max(y0) - min(y0), max(y1) - min(y1))
    return {"ok": bool(sway <= tol), "box": out[0], "sway": float(sway),
            "length_sway": float(max(ln) - min(ln)), "n": len(out)}



def carriage_agreement(clouds, tol: float = 0.30,
                       z_band: Optional[Tuple[float, float]] = None) -> dict:
    """多帧一致性 —— 判"这一次切得可不可信"**唯一靠得住**的办法。

    同一辆停着的车连拍几帧，各切一次；端点都落在 tol 内才算可信。
    与 `carriage_confidence`（单帧抖参数）的区别是它量的就是要的那件事，
    不用拐弯去猜。

    2026-09-16 实测 58 组真-停稳三连帧（`cli/lane_setup.py --dump-raw` 存的，
    相隔 1.5 s，质心 y 位移 <= 0.10 m、车点数变化 < 2%）：
      · 厢长帧间离散 中位 **1.23 m** / 最大 4.40 m
      · 后沿 y0 离散 中位 **1.30 m** / 最大 4.50 m
      · 内宽（x）离散 中位 0.04 m
    也就是说 **y 端点在帧间本来就摆 1 米量级**，这不是本函数能修好的，
    它只负责把摆得厉害的那些**报出来**而不是静默给一个数。
    根因是传感器几何：俯视单雷达在车厢远端接近掠射，前端覆盖只有 43~71%，
    数据里根本没有清晰的端面（见 docs/08 §17）。

    ▲ **它分不出"算法不稳"与"车在动"** —— 所以顺带把车动没动也量出来
      （`moved`）：车厢高度带内的点数变化 > 2% 或 y 中位数挪了 > 0.10 m，
      就认定这几帧之间车在动，此时端点对不上**不是算法的错**。
      口径与 `StabilityDetector` 那套无关，是离线复核用的粗判。
      2026-09-16 实测 62 组里有 5 组属于这种（最狠的 CH4_132902，
      y 中位挪 0.14 m、点数变 6.6%）。

    clouds 少于 2 片时返回 ok=False 并说明 —— **单帧判不了一致性**。
    """
    boxes, stat = [], []
    for a in (clouds or []):
        b = carriage_box(a, z_band=z_band)
        if b is None:
            return {"ok": False, "n": len(clouds or []), "box": None,
                    "sway": float("inf"), "length_sway": float("inf"),
                    "moved": False, "why": "有帧切不出车厢"}
        boxes.append(b)
        m = np.abs(a[:, 0]) < 1.6
        if z_band is not None:
            m &= (a[:, 2] > z_band[0]) & (a[:, 2] < z_band[1])
        q = a[m]
        stat.append((len(q), float(np.median(q[:, 1])) if len(q) else 0.0))
    if len(boxes) < 2:
        return {"ok": False, "n": len(boxes),
                "box": boxes[0] if boxes else None,
                "sway": float("nan"), "length_sway": float("nan"),
                "moved": False,
                "why": "只有 %d 帧，量不了一致性" % len(boxes)}
    ns = [s[0] for s in stat]
    ys = [s[1] for s in stat]
    dn = (max(ns) - min(ns)) / float(max(max(ns), 1))
    dy = max(ys) - min(ys)
    moved = bool(dn > 0.02 or dy > 0.10)
    y0 = [b["y0"] for b in boxes]
    y1 = [b["y1"] for b in boxes]
    ln = [b["length"] for b in boxes]
    sway = max(max(y0) - min(y0), max(y1) - min(y1))
    mid = boxes[len(boxes) // 2]
    if sway <= tol:
        why = ""
    elif moved:
        why = "这几帧之间车在动（点数变 %.1f%%、y 中位挪 %.2f m），端点对不上不怪算法" % (
            dn * 100.0, dy)
    else:
        why = "帧间端点摆 %.2f m" % sway
    return {"ok": bool(sway <= tol), "n": len(boxes), "box": mid,
            "sway": float(sway), "length_sway": float(max(ln) - min(ln)),
            "moved": moved, "why": why}


def carriage_y_window(xyz: np.ndarray, bin_m: float, gap_m: float,
                      occupy: float, below_m: float = 1.0
                      ) -> Optional[Tuple[float, float]]:
    """车厢在 y 上占了哪一段 —— 用来把"空车道上的地面"甩掉。

    为什么需要：短车厢（牵引车 + 短高栏挂）只占 ROI 19 m 窗口的三成，
    车厢前方留出 6 m 以上空车道，雷达在天底附近直视地面。地面一进 ROI，
    底面种子（z 最低 z_percentile%）就被地面点占掉，底面要么拟合不出来
    （报"未见厢底平面"），要么 min_z 被拉到地面上、`floor_z > min_z + 0.08`
    这一步跟着失真。2026-09-15 实测：短厢 6/6 有地面点（min_z 到 -5.90），
    长厢 0/42。

    做法：
      ① 按 bin_m 分格，格内点数 >= occupy x 最大格 的算"被占据"；
      ② 被占据的格连成段，段内允许 gap_m 的断口；
      ③ 点数最多的那一段是**主体**（磅秤在雷达正下方，被测车必定最密）；
      ④ ★ 其余段**只有明显比主体低**（最低点低出 below_m 以上）才丢掉 ——
         那才是地面。其它一律并回窗口。

    below_m 怎么定的（2026-09-16 实测）：真正的地面污染，旁支比主体低
    **1.73~3.62 m**（车厢底板离地就有这么高）；而车厢自己被遮挡断出来的
    旁支只低 0.52 m（`CH2_112025` 的车厢后段）。取 1.0 两边各留 1.7 倍余量。
    设成 0.5 会把那一段车厢当地面丢掉，切掉车厢后端 0.76 m。

    ▲ 第 ④ 条是保命的，别省。只按"取点数最多的那一段"会**切到车厢**：
      车厢被遮挡断成两段时，后半段会被整段丢掉。2026-09-16 实测 53 辆里
      有 8 辆被切，最狠的 `CH3_141207` 切掉车厢后端 **1.95 m**，
      而车厢被切会直接少算体积（docs/08 §10.5）。加上第 ④ 条之后降到 0 辆。

    ▲ 断口要给够。10 份回归基线**每一份**都有 0.5~2.5 m 的内部断口
      （货堆表面起伏 + 遮挡），按"必须连续"切会把它们全切短。
      实测 gap_m=1.5 / occupy=0.02 时 10 份基线的结论一个字不变。
      occupy 放到 0.10 就有 1 份基线翻车。

    返回 None 表示"不该收窄"（点太少 / 没有该丢的段），调用方原样用整片点云。
    """
    if gap_m <= 0 or xyz is None or len(xyz) < 100:
        return None
    # ★ 非有限值必须在这里挡住，与同模块的 carriage_box 同一条纪律。
    #   没有这一行时，全 NaN 点云会让下面的 np.arange(lo, hi + bin_m, bin_m)
    #   抛 "arange: cannot compute length" —— 报错里没有任何线索指向"点云含
    #   非有限值"。服务侧被停稳层的 frame_nonfinite 早退挡着走不到这里，
    #   但 cli/replay.py、cli/inspect_cloud.py、cli/baseline.py、cli/verify.py
    #   都是**直接对文件**调 detect_empty 的，一份损坏的 .xyz 就会踩到。
    #   见 docs/52 第三节。
    if xyz.ndim != 2 or xyz.shape[1] < 3 or not np.isfinite(xyz[:, 1:3]).all():
        return None
    y, z = xyz[:, 1], xyz[:, 2]
    lo, hi = float(y.min()), float(y.max())
    if hi - lo < bin_m * 3:
        return None
    edges = np.arange(lo, hi + bin_m, bin_m)
    if len(edges) < 4:
        return None
    hist, _ = np.histogram(y, bins=edges)
    if not hist.any():
        return None
    occ = hist >= max(1.0, occupy * hist.max())
    gap_bins = int(round(gap_m / bin_m))
    runs, cur, last = [], None, None
    for i, o in enumerate(occ):
        if not o:
            continue
        if cur is None or i - last - 1 > gap_bins:
            if cur is not None:
                runs.append(cur)
            cur = [i, i]
        else:
            cur[1] = i
        last = i
    if cur is not None:
        runs.append(cur)
    if len(runs) < 2:
        return None                      # 只有一段 = 没什么可丢的

    def span(r):
        return float(edges[r[0]]), float(edges[r[1] + 1])

    def zmin(r):
        a, b = span(r)
        m = (y >= a) & (y < b)
        return float(z[m].min()) if m.any() else float("inf")

    main = max(runs, key=lambda r: hist[r[0]:r[1] + 1].sum())
    base = zmin(main)
    keep = [r for r in runs if r is main or zmin(r) >= base - below_m]
    if len(keep) == len(runs):
        return None                      # 没有一段够低，说明没有地面污染，不收窄
    a = min(span(r)[0] for r in keep)
    b = max(span(r)[1] for r in keep)
    return a, b


def fit_floor(xyz: np.ndarray, cfg) -> Optional[Plane]:
    """
    底面拟合 —— 复现 detectempty.dll 内部 floor 流程（★ 已在 10/10 历史样本上验证）：

        1. 取 z 的低分位（config.detect_empty.floor.z_percentile，默认 30）作为种子点集
           —— 依据是"车厢地板是低处的密集面"
        2. PCA 拟合种子点集
        3. 按 distance_threshold 收紧内点 → 用内点重新 PCA → 迭代到收敛或 max_iterations
        4. |c| < normal_z_threshold(0.99) 则置 valid=False（交给判定链报「底面不够水平」）

    ▲ 这里**故意不用 RANSAC**。曾经试过用 RANSAC + 法向先验，结果在
    「车厢点云在 z 方向只有 ~1m 厚」的薄板型样本上，RANSAC 会挑出内点更多的
    倾斜面，使 |c| < 0.99 → 底面 invalid → 判定链在第 2 步就报「空车检测失败」，
    而真值是第 7 步的「底面内点占比过低」。10 个历史样本里有 4 个踩了这个坑。
    结论：底面必须用"最低处种子 + PCA"，不能靠 RANSAC 选内点最多的面。
    改这里之前请先跑 cli/verify.py。
    """
    if len(xyz) < 3:
        return None
    z = xyz[:, 2]
    seed = xyz[z <= np.percentile(z, cfg.floor.z_percentile)]
    if len(seed) < 3:
        seed = xyz
    plane = _pca_oriented(seed)
    if plane is None:
        return None

    thr = cfg.floor.distance_threshold
    for _ in range(max(1, int(cfg.floor.max_iterations))):
        inl = xyz[plane.abs_distance(xyz) <= thr]
        if len(inl) < 3:
            break
        nxt = _pca_oriented(inl)
        if nxt is None:
            break
        delta = (abs(nxt.a - plane.a) + abs(nxt.b - plane.b)
                 + abs(nxt.c - plane.c) + abs(nxt.d - plane.d))
        plane = nxt
        if delta < 1e-6:
            break

    inl = xyz[plane.abs_distance(xyz) <= thr]
    plane.inliers = int(len(inl))
    if len(inl):
        r = plane.abs_distance(inl)
        plane.rms = float(np.sqrt((r * r).mean()))
    plane.valid = abs(plane.c) >= cfg.floor.normal_z_threshold
    return plane


def fit_boundary_plane(xyz: np.ndarray, axis: int, sign: int, thr: float,
                       window: Optional[Tuple[float, float]] = None,
                       seed_band_m: float = 0.20,
                       select: Optional[np.ndarray] = None) -> Optional[Plane]:
    """
    拟合车厢的一个**边界面**（左/右壁、前/后壁、顶面）。底面不走这里（见 fit_floor）。

    sign=-1 取该轴负向那一侧，+1 取正向那一侧。

    ── 为什么不能"取该轴外侧 30% 当种子" ──────────────────────────
    原先 fit_axis_plane 的做法是 `percentile(v, 70)` / `percentile(v, 30)`，
    对一个 12 m 长的车厢，30% 就是 3.6 m 厚的一整块点云 —— 里面绝大多数是
    **地板**，不是端壁。PCA 拟出来的面被地板拽到车厢中间，实测车长量成
    10.97 m（真值 12.0），车宽量成 1.70 m（真值 2.4）。

    正确做法：边界面是一层**薄壳**，种子必须只取极值附近的窄带。
        ① 候选范围：配置窗口（若给了）或该侧半边；
           配置窗口够不到真实极值时自动放宽，并在 note 里标出来 ——
           现场看到"窗口自动放宽"就知道该去校准那个窗口了。
        ② 稳健极值：用 1%/99% 分位而不是 min/max，避开孤立噪点把窗口拽偏。
        ③ 种子 = 极值往内 seed_band_m（默认 0.20 m）的窄带。
        ④ PCA → 按 thr 收紧内点 → 重拟合 → 定向。

    取样策略属于「功能等价实现」（原版未逐行还原），现场拟合不稳时优先调这里。
    """
    if len(xyz) < 3:
        return None
    pool = xyz[select] if select is not None else xyz
    if len(pool) < 3:
        return None
    v_all = pool[:, axis]
    q = 1.0 if sign < 0 else 99.0
    widened = False

    if window is not None:
        lo, hi = float(window[0]), float(window[1])
        if lo > hi:
            lo, hi = hi, lo
        m = (v_all >= lo) & (v_all <= hi)
        ext = float(np.percentile(v_all, q))
        if int(m.sum()) >= 3 and lo <= ext <= hi:
            pool = pool[m]
        else:
            widened = True          # 窗口够不到真实极值 —— 标定过窄
    if len(pool) < 3:
        return None

    v = pool[:, axis]
    edge = float(np.percentile(v, q))
    seed = pool[(v <= edge + seed_band_m) if sign < 0 else (v >= edge - seed_band_m)]
    if len(seed) < 3:
        return None

    plane = fit_plane_pca(seed)
    if plane is None:
        return None
    inl = pool[plane.abs_distance(pool) <= thr]
    if len(inl) >= 3:
        refit = fit_plane_pca(inl)
        if refit is not None:
            plane = refit
    plane = plane.orient_towards(axis, sign)
    m = plane.abs_distance(pool) <= thr
    plane.inliers = int(m.sum())
    r = plane.abs_distance(pool[m])
    plane.rms = float(np.sqrt((r * r).mean())) if len(r) else 0.0
    plane.note = "取样窗口自动放宽（配置窗口够不到真实边界）" if widened else ""
    return plane


def detect_empty(frame: Frame, cfg) -> Optional[EmptyResult]:
    """
    六面拟合（对应 detectempty.dll::detect）。

    cfg 需要是 settings.CalibCfg。
    注意：调用方负责先做 ROI 裁剪（原版也是在送检测前裁的）。

    返回 None 仅表示"根本没法拟合"（点数不足 / 底面拟合失败）；
    若底面拟合出来了但不够水平，会正常返回结果并把 floor.valid 置 False，
    由判定链第 3 步报「未见厢底平面」。
    """
    xyz, yaw, _R = correct_heading(frame.xyz, cfg.heading.pct, cfg.heading.min_correction)
    source_indices = np.arange(frame.n, dtype=np.int64)
    if len(xyz) < 3:
        return None

    # ★ 按车厢收窄 y（默认关闭；y_trim_gap_m=0 时下面这段一个点都不动）
    #
    #   短车厢（牵引车 + 短高栏挂，厢长 5.25~6.25 m）只占 ROI 19 m 窗口的三成，
    #   车厢前方留出 6 m 以上空车道，雷达在天底附近直视地面 —— 地面就这么进了 ROI。
    #   2026-09-15 实测上午 53 辆：短厢 **6/6** 有地面点（min_z 到 -5.90，
    #   地面 -5.23），长厢 **0/42**；判定链停在"未见厢底平面"的比例 50% vs 11%。
    #
    #   收窄作用在**整条链**上（不只是底面拟合）。只收窄底面拟合是不自洽的：
    #   底面变成了车厢底板，而 min_z 还是整片 ROI 的最低点（地面），
    #   `floor_z > min_z + 0.08` 这一步照样会把车判成"底面过高，疑似载物" ——
    #   等于白收。实测只收窄底面时 CH4 110959/111107 正是这样从
    #   "底面内点占比过低"掉进"底面过高"，min_z 一动没动（-5.90）。
    #
    #   ▲ 这是**红线内的改动**（动了判定链看到的点云）。2026-09-16 实测：
    #     · `cli/verify.py` 开关两种状态都是 **10/10**；
    #     · 上午 53 辆：短厢 5/6 结论变（3 份从"未见厢底平面"恢复出底面平面，
    #       min_z 从 -5.9 回到 -2.3），长厢只有 1/47 变（且仍是"非空"）。
    #   参数怎么定的、为什么断口要给到 1.5 m，见 carriage_y_window 的说明与 docs/08 §13。
    win = carriage_y_window(xyz, float(getattr(cfg.floor, "y_trim_bin_m", 0.5)),
                            float(getattr(cfg.floor, "y_trim_gap_m", 0.0)),
                            float(getattr(cfg.floor, "y_trim_occupy", 0.02)),
                            float(getattr(cfg.floor, "y_trim_below_m", 1.0)))
    if win is not None:
        mask = (xyz[:, 1] >= win[0]) & (xyz[:, 1] <= win[1])
        sub = xyz[mask]
        # 收得只剩两成就不敢信了 —— 宁可不收窄，也不能把车厢自己切掉
        if len(sub) >= max(3, int(len(xyz) * 0.2)):
            xyz = sub
            source_indices = source_indices[mask]

    res = EmptyResult(cloud=xyz)
    res.yaw = yaw
    res.source_indices = source_indices
    res.source_rotation = _R.copy()

    floor = fit_floor(xyz, cfg)
    if floor is None:
        return None
    res.planes[0] = floor

    band = float(getattr(cfg.wall, "seed_band_m", 0.20))
    wthr = cfg.wall.distance_threshold

    # ── 顶面：z 高侧 ────────────────────────────────────
    res.planes[1] = fit_boundary_plane(
        xyz, 2, +1, cfg.roof.distance_threshold, seed_band_m=band) or Plane()

    # ── 左右壁：配置窗口为准，够不到真实车壁时自动放宽 ──
    res.planes[2] = fit_boundary_plane(
        xyz, 0, -1, wthr, window=cfg.wall.left_x_range, seed_band_m=band) or Plane()
    res.planes[3] = fit_boundary_plane(
        xyz, 0, +1, wthr, window=cfg.wall.right_x_range, seed_band_m=band) or Plane()

    # ── 前后壁：y 两端 ──────────────────────────────────
    res.planes[4] = fit_boundary_plane(xyz, 1, -1, wthr, seed_band_m=band) or Plane()
    res.planes[5] = fit_boundary_plane(xyz, 1, +1, wthr, seed_band_m=band) or Plane()

    # ── 三对面之间的间距（原版结构体 +0x78，就是这三个 float）──
    #
    # ★ 稳健性处理：只有当两个面在**该轴上的法向分量都足够大**（≥0.7）时才算间距。
    #   否则会出现「用两个几乎竖直的面去算 x 间距」这种退化情况，
    #   结果是 195899 这样的垃圾值（曾在硅胶自测里实测到）。
    #   宁可返回 0（表示"这个维度没量出来"），也不要返回一个会被误读的天文数字。
    #
    # ★ dims 的顺序是 (x 宽, y 长, z 高) —— 与 to_dict()/check_dims()/看板一致。
    #   曾经这里按 (floor|roof, left|right, front|back) 的**平面对顺序**往
    #   列表里 append，得到的其实是 (高, 宽, 长)，而 check_dims 按 (宽, 长, 高)
    #   解包 → 拿"宽"去比"最小长度 10m" → **任何车都报「尺寸不达标：长 2.4 < 10.0」**，
    #   .meta.json 和看板上的 x/y/z 也全是错位的。现在按轴号显式定位，不靠顺序。
    #
    # 稳健性处理：只有当两个面在**该轴上的法向分量都足够大**（≥0.7）时才算间距。
    #   否则会出现「用两个几乎竖直的面去算 x 间距」这种退化情况，
    #   结果是 195899 这样的垃圾值（曾在自测里实测到）。
    #   宁可返回 0（表示"这个维度没量出来"），也不要返回一个会被误读的天文数字。
    dims = [0.0, 0.0, 0.0]
    for i, j, axis in ((2, 3, 0), (4, 5, 1), (0, 1, 2)):
        #  (左,右)->x        (前,后)->y        (底,顶)->z
        pi_, pj_ = res.planes[i], res.planes[j]
        def k(p):
            return abs((p.a, p.b, p.c)[axis])
        if pi_.valid and pj_.valid and k(pi_) >= 0.7 and k(pj_) >= 0.7:
            a, b = pi_.intercept(axis), pj_.intercept(axis)
            if a == a and b == b and abs(b - a) < 50.0:
                dims[axis] = abs(b - a)
    res.dims = tuple(dims)
    res.ok = floor.valid
    return res


# ═══════════════════════════════════════════════════════ 载货车检测

def _cargo_top(xyz: np.ndarray, cfg) -> Optional[Plane]:
    """
    在车厢内部找「货物顶面」。

    这是「底面过高，疑似载物」之外的第二种载货判据，原版 detectloaded 输出的
    那个 z 方向平面（z ≈ 货堆顶）就是它。
    策略：取车厢中段（y 的中间 60%）的点，用 RANSAC 拟合水平面。

    这里**用 RANSAC 是合适的**：货堆顶面在车厢里只占一小部分点，
    不像地板那样是"最低处的密集面"，靠分位取种子取不到。
    """
    if len(xyz) < 50:
        return None
    y0, y1 = np.percentile(xyz[:, 1], [20, 80])
    band = xyz[(xyz[:, 1] >= y0) & (xyz[:, 1] <= y1)]
    if len(band) < 50:
        band = xyz
    prior = Plane(0.0, 0.0, 1.0, 0.0, True)          # 先验：法向朝 +z
    p = fit_plane_ransac(band, cfg.roof.distance_threshold, iters=64, prior=prior)
    if p is None:
        return None
    if float(p.normal() @ prior.normal()) < 0:
        p = p.flip()
    # ★ 胜出面必须**够水平**才算"货顶"（2026-09-23，两轮审查 M2）：
    #   RANSAC 只按内点数选面 —— y 中段的左右壁是贯穿整段的竖直强平面，
    #   起伏的货面大概率争不过它，胜出的"货顶"其实是厢壁：近竖直面
    #   （|n_z|≈0）的竖向间距会算出 NaN 并原样进 JSON。
    #   取 0.9（约 26 度）：货面允许起伏，竖直壁一眼可分；够不水平就
    #   承认"没找到货顶"，交给调用方的 Plane() 兜底（那才是真实语义）。
    if abs(float(p.normal()[2])) < 0.9:
        return None
    return p


@dataclass
class LoadedResult:
    """对应原版 detectloaded 的 LoadedDetectResult（≈0x80 字节）。"""
    planes: List[Plane] = field(default_factory=lambda: [Plane() for _ in range(4)])
    spans: Tuple[float, float] = (0.0, 0.0)
    cloud: np.ndarray = field(default_factory=lambda: np.zeros((0, 3), np.float32))
    yaw: float = 0.0
    ok: bool = False
    source_rotation: Optional[np.ndarray] = None

    def to_dict(self) -> Dict:
        return {
            "ok": self.ok,
            "spans": {"x": round(self.spans[0], 4), "z": round(self.spans[1], 4)},
            "planes": [{"a": round(p.a, 6), "b": round(p.b, 6), "c": round(p.c, 6),
                        "d": round(p.d, 6), "valid": bool(p.valid)} for p in self.planes],
        }


def detect_loaded(frame: Frame, cfg) -> LoadedResult:
    """
    载货车检测（对应 detectloaded.dll::detect）。

    原版输出 4 个平面：左右壁（x 方向两面）+ 一个 y 方向面 + 一个 z 方向面（货堆顶）。
    这里按同样的语义拟合。
    TODO(现场)：原版这 4 个平面的具体取样策略未还原，若体积偏差大，优先调这里。
    """
    res = LoadedResult()
    xyz, yaw, rotation = correct_heading(frame.xyz, cfg.heading.pct, cfg.heading.min_correction)
    res.yaw = yaw
    res.source_rotation = rotation
    res.cloud = xyz
    if len(xyz) < 20:
        return res

    lx = xyz[xyz[:, 0] <= np.percentile(xyz[:, 0], 25)]
    rx = xyz[xyz[:, 0] >= np.percentile(xyz[:, 0], 75)]
    for i, (pts, sg) in enumerate(((lx, -1), (rx, +1))):
        p = fit_plane_ransac(pts, cfg.wall.distance_threshold, iters=64) if len(pts) >= 3 else None
        res.planes[i] = (p.orient_towards(0, sg) if p else Plane())

    y = xyz[:, 1]
    fp = None
    fpts = xyz[y <= np.percentile(y, 20)]
    if len(fpts) >= 3:
        fp = fit_plane_ransac(fpts, cfg.wall.distance_threshold, iters=64)
    res.planes[2] = (fp.orient_towards(1, -1) if fp else Plane())

    top = _cargo_top(xyz, cfg)
    res.planes[3] = top or Plane()

    if res.planes[0].valid and res.planes[1].valid:
        a = res.planes[0].intercept(0)
        b = res.planes[1].intercept(0)
        res.spans = (abs(b - a) if (a == a and b == b) else 0.0,
                     abs(res.planes[3].intercept(2)) if res.planes[3].valid else 0.0)
    res.ok = res.planes[3].valid
    return res


# ═══════════════════════════════════════════════════════ 空车基线 + 体积

@dataclass
class EmptyBaseline:
    """
    空车基线 —— 原版由 detectloaded.LoadEmptyBaseline(path) 从 data/in 读取。

    现场从未生成过任何基线，所以这段在原系统里从未被跑到。
    本项目用一个 npz 存「空车点云 + 车厢内腔盒」，语义直观、便于人工检查。
    """
    floor_z: float = 0.0
    box: Tuple[float, float, float, float, float, float] = (0, 0, 0, 0, 0, 0)
    cloud: np.ndarray = field(default_factory=lambda: np.zeros((0, 3), np.float32))
    n: int = 0
    created: str = ""
    # ★ 底板**平面**（a,b,c,d），2026-09-16 加。
    #
    #   原来只存一个标量 floor_z（平面在原点的截距），基线没覆盖到的格子
    #   一律拿它兜底。但**底板是斜的** —— 空载自卸车向车头翘 3.8~4.1 度，
    #   沿 y 落差 0.68~0.84 m。拿一个常数当整片底板，在没覆盖到的格子上
    #   最大会差 0.4 m；而基线的覆盖率实测只有 79%（多帧合并后 85~92%，
    #   docs/08 §10.11），也就是**有 8%~21% 的格子吃这个误差**。
    #   存下平面之后，那些格子按平面逐格求值，倾斜就被吃掉了。
    #
    #   旧的 npz 没有这个字段 -> bed_plane 为 None -> 行为与从前完全一致。
    bed_plane: Optional[Tuple[float, float, float, float]] = None
    source_channel: str = ""

    def bed_z_at(self, x, y):
        """底板在 (x, y) 处的高度。没有平面就退回常数 floor_z（旧行为）。"""
        p = self.bed_plane
        if p is None or len(p) == 0 or abs(p[2]) < 1e-9:
            return np.full(np.shape(x), float(self.floor_z), dtype=np.float64)
        a, b, c, d = p
        return -(a * np.asarray(x, np.float64) + b * np.asarray(y, np.float64) + d) / c

    def save(self, path: str) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
        kw = {}
        if self.source_channel:
            kw["source_channel"] = self.source_channel
        if self.bed_plane is not None and len(self.bed_plane):
            kw["bed_plane"] = np.asarray(self.bed_plane, np.float64)
        np.savez_compressed(path, floor_z=self.floor_z, box=np.asarray(self.box, np.float64),
                            cloud=self.cloud, created=self.created, **kw)

    @staticmethod
    def load(path: str) -> Optional["EmptyBaseline"]:
        if not os.path.exists(path):
            return None
        # ★ NpzFile 用 with 收口（2026-09-23，低严重度批次）：service 在
        #   Windows 上用 tempfile + os.replace 原子替换基线文件，返回时
        #   若句柄还悬着（非 CPython 实现下），os.replace 会 PermissionError。
        with np.load(path, allow_pickle=False) as z:
            # 老基线没有 bed_plane —— 取不到就是 None，行为退回从前
            bp = None
            if "bed_plane" in getattr(z, "files", []):
                v = np.asarray(z["bed_plane"], np.float64).ravel()
                if len(v) == 4 and np.isfinite(v).all():
                    bp = tuple(float(t) for t in v)
            return EmptyBaseline(float(z["floor_z"]), tuple(float(x) for x in z["box"]),
                                 z["cloud"], int(len(z["cloud"])), str(z["created"]),
                                 bed_plane=bp, source_channel=str(z["source_channel"]) if "source_channel" in z.files else "")

    @staticmethod
    def from_empty_result(res: Optional[EmptyResult], created: str = "") -> Optional["EmptyBaseline"]:
        """从一次空车检测结果建基线。点云为空 / 底面拟合失败时返回 None，
        **不要**在这里抛异常 —— 调用方多半是现场操作，报 None 比崩掉好定位。"""
        if res is None or res.cloud is None or len(res.cloud) < 50:
            return None
        cloud = np.asarray(res.cloud, np.float32)
        lo, hi = cloud.min(axis=0), cloud.max(axis=0)
        fl = res.plane("floor")
        fz = float(fl.intercept(2)) if fl.valid else float("nan")
        if not (fz == fz):                      # 底面无效 → 退回点云最低处
            fz = float(lo[2])
        # ★ 底板平面：优先用双平面量出来的（它只拿"看得见底板"的格子拟，
        #   不会被货面/横梁带偏）；量不出来就退回六面拟合的 floor 平面；
        #   再不行就 None（=退回旧的常数兜底）。
        bp = None
        try:
            lp = analyze_load(np.asarray(cloud, np.float64))
            if lp.ok and lp.bed_plane.valid and abs(lp.bed_plane.c) > 1e-9:
                bp = (float(lp.bed_plane.a), float(lp.bed_plane.b),
                      float(lp.bed_plane.c), float(lp.bed_plane.d))
        except Exception:                                   # noqa: BLE001
            bp = None
        if bp is None and fl.valid and abs(fl.c) > 1e-9:
            bp = (float(fl.a), float(fl.b), float(fl.c), float(fl.d))
        return EmptyBaseline(
            floor_z=fz,
            box=(float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1]),
                 fz, float(hi[2])),
            cloud=cloud, n=len(cloud), created=created, bed_plane=bp)


@dataclass
class VolumeResult:
    volume_m3: float = 0.0
    occupied_voxels: int = 0
    voxel_size: float = 0.1
    ok: bool = False
    message: str = ""
    text: str = ""     # 原版 VolumeResult +0x98 是一个 std::string（摘要文本）
    # ── 以下是本项目新增的**可信度**字段（2026-09-17，docs/13 §16）────────
    #   原版没有这些。它们不改变体积本身，只是把"这个数有多可信"摊开写，
    #   免得下游拿到一个孤零零的吨数当真值用。
    coverage: float = float("nan")      # 车厢盒内载货侧的实测覆盖率
    # filled = wall + hole 是历史诊断分解；壁格也可能只有单侧观测。
    # 现有 hole/spread 门槛暂保兼容，不证明壁后或两端的隐藏体积。
    filled_cells: int = 0               # = wall_cells + hole_cells
    filled_ratio: float = 0.0           # = wall_ratio + hole_ratio（**只看趋势**）
    wall_cells: int = 0                 # 厢壁盲区格（结构性，每车都有）
    wall_ratio: float = 0.0             # 体积里落在厢壁格上的比例（0~1）
    hole_cells: int = 0                 # 真正没数据的格子（不含厢壁格）
    hole_ratio: float = 0.0             # 估计体积中普通空洞所占比例
    frames_used: int = 1                # 货面用了几帧
    bound_lo: float = float("nan")      # 显式独立模型条件下的下界
    bound_hi: float = float("nan")      # 含全部未知域的条件上界
    bound_spread: float = float("nan")  # (hi-lo)/hi，不是现场误差保证
    range_kind: str = "unavailable"
    range_status: str = "missing"
    range_reason: str = "缺少独立物理条件"
    range_reference_id: Optional[str] = None
    range_conditions: Dict = field(default_factory=dict)
    range_assumptions: Tuple[str, ...] = ()
    domain_area_m2: float = float("nan")
    observed_area_m2: float = float("nan")
    unknown_area_m2: float = float("nan")
    unfilled_area_m2: float = float("nan")
    support_cell_m: float = float("nan")
    measured_volume_m3: float = float("nan")
    interpolated_volume_m3: float = float("nan")
    extrapolated_volume_m3: float = float("nan")
    cell_auto: bool = False             # 格子是不是自动选的
    bed_shift_applied_m: float = 0.0
    bed_reference_status: str = "not_requested"
    bed_reference_source: Optional[str] = None
    bed_reference_id: Optional[str] = None
    bed_reference_uncertainty_m: Optional[float] = None
    bed_reference_reason: str = ""
    # 逐车交叉校核（见 cross_check_volume）。**不进 to_dict**：归档里原精度体积结果按键集合
    # 严格校验（measurement_runtime），加键会让所有旧归档读不出来；结论另写进 text。
    cross_check: Optional["CrossCheck"] = field(default=None, compare=False, repr=False)
    # 积分域与外推检验（见 _compute_volume_impl）、逐车误差估计（见 estimate_volume_uncertainty）。
    # 同上，不进 to_dict，结论写进 text。
    domain: Optional["VolumeDomainInfo"] = field(default=None, compare=False, repr=False)
    uncertainty: Optional["VolumeUncertainty"] = field(default=None, compare=False, repr=False)
    # 遮挡上限补洞（见 app/occlusion_fill.py，fill_bound=True 时才有）。同上，不进 to_dict。
    occlusion: Optional["OcclusionFillInfo"] = field(default=None, compare=False, repr=False)
    declared_model_used: bool = field(default=False, compare=False, repr=False)
    baseline_registration: Optional[dict] = field(default=None, compare=False, repr=False)

    @staticmethod
    def _finite(value) -> bool:
        try:
            return (not isinstance(value, (bool, np.bool_)) and
                    isinstance(value, (int, float, np.integer, np.floating)) and math.isfinite(value))
        except OverflowError:
            return False

    def estimate_validity_errors(self) -> List[str]:
        """估计可用性与范围可用性分开；缺独立条件不丢掉有限估值。"""
        why = []
        if self.ok is not True:
            why.append("体积计算未成功")
        if not self._finite(self.volume_m3) or self.volume_m3 < 0:
            why.append("体积必须为有限非负值")
        if not self._finite(self.voxel_size) or self.voxel_size <= 0:
            why.append("网格尺寸必须有限且为正")
        if not self._finite(self.coverage) or not 0 < self.coverage <= 1:
            why.append("缺少有效观测覆盖")
        for key in ("filled_ratio", "wall_ratio", "hole_ratio"):
            value = getattr(self, key)
            if not self._finite(value) or not 0 <= value <= 1:
                why.append(key + " 缺失或超出比例范围")
        return why

    def range_validity_errors(self) -> List[str]:
        why = []
        if self.range_kind != "conditional_cell_mean" or self.range_status != "conditional":
            why.append(self.range_reason or "缺少有效条件范围")
        if not self._finite(self.bound_spread) or not 0 <= self.bound_spread <= 1:
            why.append("bound_spread 缺失或超出比例范围")
        if not all(self._finite(value) for value in (self.bound_lo, self.bound_hi)):
            why.append("体积范围缺失或非有限")
        elif self.bound_lo < 0 or self.bound_hi < self.bound_lo:
            why.append("体积上下界顺序无效")
        elif self._finite(self.volume_m3):
            tolerance = 1e-9 * max(1.0, abs(self.volume_m3), self.bound_hi)
            if not self.bound_lo - tolerance <= self.volume_m3 <= self.bound_hi + tolerance:
                why.append("估计体积不在声明范围内")
        return why

    def validity_errors(self) -> List[str]:
        """保留旧质量校验入口；auto仅凭估值校验决定是否丢弃候选。"""
        return self.estimate_validity_errors() + self.range_validity_errors()

    def to_dict(self, raw: bool = False) -> Dict:
        # 内部失败默认值保持兼容；对外不能把未计算解释成实测零煤量。
        volume = (self.volume_m3 if self.ok is True and self._finite(self.volume_m3)
                  and self.volume_m3 >= 0 else None)
        d = {"volume_m3": volume, "occupied_voxels": self.occupied_voxels,
             "voxel_size": self.voxel_size, "ok": self.ok, "message": self.message,
             "text": self.text,
             "filled_cells": self.filled_cells,
             "filled_ratio": self.filled_ratio,
             "wall_cells": self.wall_cells,
             "wall_ratio": self.wall_ratio,
             "hole_cells": self.hole_cells,
             "hole_ratio": self.hole_ratio,
             "frames_used": self.frames_used,
             "cell_auto": self.cell_auto,
             "bed_shift_applied_m": self.bed_shift_applied_m,
             "bed_reference_status": self.bed_reference_status,
             "bed_reference_source": self.bed_reference_source,
             "bed_reference_id": self.bed_reference_id,
             "bed_reference_uncertainty_m": self.bed_reference_uncertainty_m,
             "bed_reference_reason": self.bed_reference_reason,
             # 保留归档字段兼容性，不再把已计算结果标为低置信或要求审核。
             "low_confidence": False,
             "estimate_ok": not self.estimate_validity_errors(),
             "estimate_reason": "；".join(self.estimate_validity_errors()),
             "range_ok": self.range_ok()[0],
             "range_gate_reason": self.range_ok()[1]}
        for k in ("coverage", "bound_lo", "bound_hi", "bound_spread", "range_kind", "range_status",
                  "range_reason", "range_reference_id", "range_assumptions", "domain_area_m2",
                  "observed_area_m2", "unknown_area_m2", "unfilled_area_m2", "support_cell_m", "measured_volume_m3",
                  "interpolated_volume_m3", "extrapolated_volume_m3"):
            d[k] = getattr(self, k)
        # 条件字典只含验证后的JSON基本类型；返回独立拷贝，不能修改结果本身。
        d["range_conditions"] = asdict(self)["range_conditions"]
        for key, value in d.items():
            if isinstance(value, (float, np.floating)):
                if not math.isfinite(value):
                    d[key] = None
                elif not raw and key in ("bound_lo", "bound_hi"):
                    # 展示舍入只能向外扩，不能把零宽真值范围四舍五入到真值之外。
                    scaled = float(value) * 10000.
                    if math.isfinite(scaled):
                        rounded = (math.floor(scaled) if key == "bound_lo" else math.ceil(scaled)) / 10000.
                        d[key] = min(float(value), rounded) if key == "bound_lo" else max(float(value), rounded)
                    else:
                        d[key] = float(value)
                else:
                    d[key] = float(value) if raw else round(float(value), 4)
        if self.baseline_registration is not None:
            d["baseline_registration"] = self.baseline_registration
        return d

    def estimate_ok(self, hole_max: float = VOLUME_HOLE_MAX) -> Tuple[bool, str]:
        """**只问估值本身能不能用**：算成功了吗、比例合法吗、空洞多不多。

        与 `range_ok()` 分开的理由（2026-09-18 实测）：范围宽不宽是
        **传感器几何 + 声明模型**的属性（未观测面积 34%~39% 要按
        `max_height_m` 整个算进上界），而不是这一次测量测得好不好。
        两件事混成一个布尔之后，下游只看到"低置信"，分不出到底是
        "这车没扫好"还是"这套几何本来就给不出窄区间" —— 现场会去查车，
        而该查的是门槛。见 `VOLUME_SPREAD_MAX` 上面的说明。
        """
        why = self.estimate_validity_errors()
        if self.bed_reference_status == "rejected":
            why.append("独立底板参考未通过校验")
        if not (self._finite(hole_max) and 0 <= hole_max <= 1):
            why.append("体积质量门槛非法")
        elif not why and self.hole_ratio > hole_max:
            why.append("空洞外推占 %.0f%%（> %.0f%%）" % (self.hole_ratio * 100, hole_max * 100))
        return (not why), "；".join(why)

    def range_ok(self, spread_max: Optional[float] = VOLUME_SPREAD_MAX) -> Tuple[bool, str]:
        """**只问声明范围够不够窄**。缺范围也算不通过。

        `spread_max=None` 表示**不设宽度门槛**（现场 2026-09-18 的决定，
        见 `VOLUME_SPREAD_MAX` 上面的说明）——范围本身照算照报，
        只是不拿它拦 `volumeStatus`。
        """
        why = self.range_validity_errors()
        if self.bed_reference_status == "applied" and (
                not self._finite(self.bed_reference_uncertainty_m) or
                self.range_conditions.get("applied_bed_shift_error_m") != self.bed_reference_uncertainty_m):
            why.append("底板参考误差尚未计入体积范围")
        if spread_max is not None and not (self._finite(spread_max) and 0 <= spread_max <= 1):
            why.append("体积质量门槛非法")
        elif spread_max is not None and not why:
            sp = self.bound_spread
            if sp == sp and sp > spread_max:
                why.append("上下界差 %.1f%%（> %.1f%%）" % (sp * 100, spread_max * 100))
        return (not why), "；".join(why)

    def confidence(self, hole_max: float = VOLUME_HOLE_MAX,
                   spread_max: Optional[float] = VOLUME_SPREAD_MAX) -> Tuple[bool, str]:
        """条件范围与既有hole/spread门槛；通过仍依赖外部模型条件真实性。

        ▲ 行为与从前**逐条一致**（两道门槛都要过）。拆出来的
          `estimate_ok()` / `range_ok()` 是为了让下游能分辨卡在哪一条，
          不是放宽。
        """
        why = self.estimate_validity_errors() if self.baseline_registration is not None and not self.declared_model_used else self.validity_errors()
        if self.bed_reference_status == "rejected":
            why.append("独立底板参考未通过校验")
        if self.bed_reference_status == "applied" and (
                not self._finite(self.bed_reference_uncertainty_m) or
                self.range_conditions.get("applied_bed_shift_error_m") != self.bed_reference_uncertainty_m):
            why.append("底板参考误差尚未计入体积范围")
        gates = [hole_max] + ([] if spread_max is None else [spread_max])
        if not all(self._finite(v) and 0 <= v <= 1 for v in gates):
            why.append("体积质量门槛非法")
        if why:
            return False, "；".join(why)
        if self.hole_ratio > hole_max:
            why.append("空洞外推占 %.0f%%（> %.0f%%）" % (self.hole_ratio * 100,
                                                          hole_max * 100))
        sp = self.bound_spread
        if spread_max is not None and sp == sp and sp > spread_max:
            why.append("上下界差 %.1f%%（> %.1f%%）" % (sp * 100, spread_max * 100))
        return (not why), "；".join(why)


# 高度图每格的取值方式（见 _height_map）。默认 max；median 只作对照，见下。
SURFACE_STATS = ("max", "median")


def _height_map(pts: np.ndarray, x0: float, y0: float, nx: int, ny: int,
                cell: float, stat: str = "max") -> Tuple[np.ndarray, np.ndarray]:
    """
    把点云压成俯视高度图：每个 (x,y) 单元取该列**最高**的 z。

    返回 (zmax, hit)：
        zmax (ny, nx) float64 —— 该单元的最高点 z；没有点的单元为 nan
        hit  (ny, nx) bool    —— 该单元有没有点

    "取最高"是由传感器决定的：雷达吊装在车厢正上方俯视，一条激光打下去
    只会被**最上面**的那个表面挡住，下面是什么看不到。所以单元里最高的点
    就是这一列的可见顶面 —— 空车时是底板，装货后是货堆顶。

    ▲ stat="median"（2026-10-02，对照用）：每格取点的中位高度。取最高点对粗糙的煤面
      系统性偏高（格内取的是块煤的峰，噪声 1 cm、每格上百点就偏约 2.5 cm），底板平整、
      偏得少，相减剩下一个正偏差；均匀采样时中位数的偏差可能较小，但掠射处每格只有几个点、
      又只看得见块煤朝向雷达的一面，它也不是真值。需已知体积标定或独立体积测量判断；磅单
      只有重量，不能独立选择取值方式。默认仍取最高，中位只在说明里并报。
    """
    # 先排域外点再转整数，极大但有限的额外观测不能在索引转换时溢出。
    pts = pts[(pts[:, 0] >= x0) & (pts[:, 0] < x0 + nx * cell) &
              (pts[:, 1] >= y0) & (pts[:, 1] < y0 + ny * cell)]
    ix = np.floor((pts[:, 0] - x0) / cell).astype(np.int64)
    iy = np.floor((pts[:, 1] - y0) / cell).astype(np.int64)
    m = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    ix, iy, z = ix[m], iy[m], pts[m, 2].astype(np.float64)
    flat = iy * nx + ix
    if stat == "median":
        zmap = np.full(ny * nx, np.nan, np.float64)
        if len(flat):
            order = np.lexsort((z, flat))
            fs, zs = flat[order], z[order]
            starts = np.flatnonzero(np.r_[True, fs[1:] != fs[:-1]])
            counts = np.diff(np.r_[starts, len(fs)])
            zmap[fs[starts]] = 0.5 * (zs[starts + (counts - 1) // 2] + zs[starts + counts // 2])
        hit = np.isfinite(zmap)
        return zmap.reshape(ny, nx), hit.reshape(ny, nx)
    zmax = np.full(ny * nx, -np.inf, np.float64)
    np.maximum.at(zmax, flat, z)
    hit = np.isfinite(zmax)
    zmax[~hit] = np.nan
    return zmax.reshape(ny, nx), hit.reshape(ny, nx)


def _relative_height_map(pts, baseline, dy, x0, y0, nx, ny, cell, stat, pose=None):
    """逐点减去底板和已采纳姿态，再分格，避免采样位置混入货高。"""
    values = np.asarray(pts, np.float64)
    relative = values[(values[:, 0] >= x0) & (values[:, 0] < x0 + nx * cell)
                      & (values[:, 1] >= y0) & (values[:, 1] < y0 + ny * cell)]
    relative[:, 2] -= baseline.bed_z_at(relative[:, 0], relative[:, 1] - dy)
    if pose is not None:
        relative[:, 2] -= pose.at(relative[:, 0], relative[:, 1])
    return _height_map(relative, x0, y0, nx, ny, cell, stat)


def _fill_holes(Z: np.ndarray, iters: int = 200, domain: Optional[np.ndarray] = None) -> np.ndarray:
    """共用非周期内核；原锚点固定、未知持续松弛，仅在 domain 内传播。

    补点是模型估计，不能把未扫描区当作已知货面；数值收敛不证明物理误差。
    """
    from .height_fill import fill_height_holes
    return fill_height_holes(Z, iters=iters, domain=domain)


def _rear_edge(pts: np.ndarray, step: float = 0.25, min_pts: int = 40
               ) -> Optional[float]:
    """车厢后沿（-y 那一头）的 y 坐标。找不到返回 None。

    从 -y 往前扫，第一段"够密且接着还够密"的行就是后沿：
    某一行点数 >= min_pts，且随后 3 行里至少 2 行也够 —— 单独一行可能是
    几个杂散点，连着才是真结构。

    ★ 门槛必须是**绝对值**，不能取"最大行数的百分之几"。
      俯视雷达天底那一行点数是别处的几十倍（实测一行 2 万点），
      按比例定门槛会被它抬到 400，把真正的后沿（一行两三百点）整段跳过。
      2026-09-16 用比例门槛估出 +2.03，用绝对门槛估出 +1.00，
      而后者与 cli/volume_calc.py 独立算出的值一致。

    ★ 为什么用**后沿**不用前沿：俯视雷达天底附近点最密，越往前掠射角越大。
      实测车厢前端（+y）的栅格覆盖率只有 43%~71%，后端有 77%~88%。
    """
    if pts is None or len(pts) < 200:
        return None
    y = pts[:, 1]
    lo, hi = float(y.min()), float(y.max())
    span = hi - lo
    if (not math.isfinite(span) or not math.isfinite(step) or step <= 0 or
            span < step * 4 or span / step > 4_000_000 or lo + step == lo):
        return None
    edges = np.arange(lo, hi, step)
    cnt = np.histogram(y, bins=np.append(edges, edges[-1] + step))[0]
    ok = cnt >= int(min_pts)
    for i in range(len(ok) - 3):
        if ok[i] and int(ok[i + 1:i + 4].sum()) >= 2:
            return float(edges[i])
    return None


@dataclass(frozen=True)
class RegistrationDecision:
    dy_m: float
    status: str
    reason: str


def estimate_registration(baseline: "EmptyBaseline", loaded: Optional["LoadedResult"],
                          above_m: float = 0.30, max_dy_m: float = 2.5) -> RegistrationDecision:
    """估"把基线往 +y 挪多少"才能对上这一车，明确区分resolved与unresolved。

    两边都取**高出底板平面 above_m 以上**的点（厢壁在空车和载货里都看得见，
    是唯一可比的特征），各自找车厢后沿，相减即得。

    ▲ 早期两个走不通的估法，记下来免得重走：
      · 基线用**底板**点、载货用**货面**点找后沿 —— 量的不是同一个特征，
        货可能堆不到车厢最后头，估出 +1.88/+1.98（真值附近是 +1.00）。
      · 最大化两个足迹的 IoU —— 被 +y 的驾驶室/门架拽偏，估出 **-0.90**，
        连符号都反了。

    ▲ 估不出或超过 max_dy_m 就返回 0（不配准）。不配准只是少算 2%~3%，
      配反了要多错 10% 以上（实测 dy 从 -1.5 到 +2.0，体积差 18%）。
    """
    def unresolved(reason):
        return RegistrationDecision(0., "unresolved", reason)
    if loaded is None or baseline is None or baseline.n == 0:
        return unresolved("缺基线或载货点云")
    if not all(VolumeResult._finite(v) and v >= 0 for v in (above_m, max_dy_m)):
        return unresolved("配准门槛非法")
    try:
        bc = np.asarray(baseline.cloud, np.float64)
        lc = np.asarray(loaded.cloud, np.float64)
        for cloud in (bc, lc):
            if cloud.ndim != 2 or cloud.shape[1] != 3 or not np.isfinite(cloud).all():
                return unresolved("配准点云非法")
            if len(cloud):
                span = float(cloud[:, 1].max()) - float(cloud[:, 1].min())
                if not math.isfinite(span) or span / .25 > 4_000_000:
                    return unresolved("自动配准范围非法或过大")
        if len(bc) != baseline.n or not VolumeResult._finite(baseline.floor_z):
            return unresolved("配准基线非法")
        if baseline.bed_plane is not None:
            plane = np.asarray(baseline.bed_plane, np.float64)
            if plane.shape != (4,) or not np.isfinite(plane).all() or abs(plane[2]) < 1e-9:
                return unresolved("配准底板平面非法")
    except (ValueError, TypeError, OverflowError):
        return unresolved("配准输入格式非法")
    if len(bc) < 200 or len(lc) < 200:
        return unresolved("点太少")
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            bp = bc[(bc[:, 2] - baseline.bed_z_at(bc[:, 0], bc[:, 1])) > above_m]
            lp = lc[(lc[:, 2] - baseline.bed_z_at(lc[:, 0], lc[:, 1])) > above_m]
    except (FloatingPointError, OverflowError):
        return unresolved("配准相对高度溢出")
    rb, rl = _rear_edge(bp), _rear_edge(lp)
    if rb is None or rl is None:
        return unresolved("找不到车厢后沿")
    d = rl - rb
    if abs(d) > float(max_dy_m):
        return unresolved("估出 %+.2f m 超过上限 %.1f m，不配准" % (d, max_dy_m))
    return RegistrationDecision(float(d), "resolved", "按车厢后沿对齐 %+.2f m" % d)


def estimate_dy(baseline: "EmptyBaseline", loaded: Optional["LoadedResult"],
                above_m: float = 0.30, max_dy_m: float = 2.5) -> Tuple[float, str]:
    """兼容旧的二元返回；运行条件必须使用带明确状态的estimate_registration。"""
    decision = estimate_registration(baseline, loaded, above_m, max_dy_m)
    return decision.dy_m, decision.reason


# ★ 横向配准（2026-10-01）—— 只配纵向 dy 不够。
#   空车与载货是两次停车，左右位置也不一样；跨车道时两台雷达的坐标更不同。
#   2026-09-28 宁A66912A（载货 CH3、空车 CH4）横向错开 0.30 m：
#   体积 35.99 m^3，把基线挪齐后 30.30 m^3 —— 多算 19%。
#   机理：厢壁格（基线里高出底板平面 0.25 m 的格子）是按**基线**的厢壁位置划的；
#   载货车的厢壁一错开，它的壁顶就落进基线的"底板格"，一整列 1.6 m 高的壁顶
#   被当成煤（11 m 长、0.10 m 宽的一列约 1.8 m^3）。
#   ▲ 体积对 dx 是一条很窄的谷，所以**估出来就用、不设死区**：
#     宁A05922F（同道，估出 +0.015）往错的方向挪 3 cm 体积 +2.4%、5 cm +5%；
#     而估计本身很稳 —— 同一次停车三帧之间中心只散 1~3 mm。
# ▲ 上限按跨道最坏情形定：50 份归档里车厢中心 CH2 +0.17~+0.37、CH3 -0.28~-0.06、
#   CH4 -0.02~+0.25 m，空车在一条道标定、载货走另一条道时相对偏移可到约 0.65 m。
#   防错估靠的是下面两道校核（两法一致 + 外包同宽），上限只兜底。
LATERAL_MAX_DX_M = 0.8
# ▲ carriage_box 默认 x_lim=1.6 会切掉右壁在 +1.63 m 的 CH2 车
#   （50 份归档里 4 份，车厢盒中心偏 2 cm）；这里取现行 ROI 的 x 半宽。改 ROI 时要跟着看。
LATERAL_X_LIM_M = 2.0


@dataclass(frozen=True)
class LateralDecision:
    dx_m: float      # 把基线往 +x 挪多少（= 载货车厢中心 - 基线车厢中心）；不挪时为 0
    status: str      # resolved（照此挪）/ unresolved（估不出或不可信，不挪）
    reason: str
    # 两片点云各自切出的车厢盒（空车, 载货），姿态估计直接复用 —— carriage_box 每次 30~100 ms，
    # 两个估计各切一遍会让每次体积计算多出一倍的时间。不参与比较。
    boxes: Optional[Tuple[Optional[dict], Optional[dict]]] = field(default=None, compare=False, repr=False)


def _side_wall_envelope(cloud: np.ndarray, box: dict, bed_z_at, above_m: float
                        ) -> Optional[Tuple[float, float]]:
    """车厢中段（box 的 y 范围 20%~80%）里高出底板平面 above_m 的点，x 的 p0.5 / p99.5。

    空车里这些点只剩厢壁，载货里是货面 + 壁顶 —— 两边的外包都由**壁顶外缘**决定，
    比的是同一个特征。两端不用：前端掠射、后端可能有尾板/门架。
    """
    y0, y1 = float(box["y0"]), float(box["y1"])
    span = y1 - y0
    p = cloud[(cloud[:, 1] >= y0 + 0.2 * span) & (cloud[:, 1] <= y1 - 0.2 * span)]
    if len(p):
        p = p[(p[:, 2] - bed_z_at(p[:, 0], p[:, 1])) > above_m]
    if len(p) < 200:
        return None
    lo, hi = np.percentile(p[:, 0], [0.5, 99.5])
    return float(lo), float(hi)


def estimate_lateral_registration(baseline: "EmptyBaseline", loaded: Optional["LoadedResult"],
                                  above_m: float = 0.30, max_dx_m: float = LATERAL_MAX_DX_M,
                                  agree_m: float = 0.06, width_tol_m: float = 0.15
                                  ) -> LateralDecision:
    """估"把基线往 +x 挪多少"才能对上这一车；估不出或不可信就不挪（dx=0，即旧行为）。

    两种估法**都要过**才采用，数值取第一种：
      ① 侧壁外包：车厢中段、高出底板平面 above_m 的点的 x 外包中心（见上）。
      ② 车厢盒：carriage_box 的 x0/x1 中心。只作校核 —— 它的 p1/p99 掺了底板点，
         在空车云上系统性偏 2~3.5 cm，载货云上 1 cm 以内（50 份归档实测）。
    以下任一条不挪：
      · 两法差超过 agree_m —— 50 份归档两两组合最大 0.048 m；
      · 两边外包宽度差超过 width_tol_m —— 同一辆车实测 0.016 / 0.054 m（9-28 两对），
        一侧壁被 ROI 截掉时会明显变窄，两法会一起偏，只有宽度看得出来；
      · 偏移超过 max_dx_m。
    ▲ 载货点按基线平面**原位**求值、不扣 dy：底板沿 y 的坡度约 5%，dy 到 2.5 m 也只差
      0.13 m，而外包由壁顶（离底板 1.5 m 以上）决定，所以与 dy 无关 ——
      compute_volume_auto 因此可以先估一次、各档共用。
    """
    def unresolved(reason, boxes=None):
        return LateralDecision(0., "unresolved", reason, boxes)
    if loaded is None or baseline is None or baseline.n == 0:
        return unresolved("缺基线或载货点云")
    if not all(VolumeResult._finite(v) and v >= 0 for v in (above_m, max_dx_m, agree_m, width_tol_m)):
        return unresolved("门槛非法")
    try:
        bc = np.asarray(baseline.cloud, np.float64)
        lc = np.asarray(loaded.cloud, np.float64)
        for cloud in (bc, lc):
            if cloud.ndim != 2 or cloud.shape[1] != 3 or not np.isfinite(cloud).all():
                return unresolved("点云非法")
        if len(bc) != baseline.n or not VolumeResult._finite(baseline.floor_z):
            return unresolved("基线非法")
        if baseline.bed_plane is not None:
            plane = np.asarray(baseline.bed_plane, np.float64)
            if plane.shape != (4,) or not np.isfinite(plane).all() or abs(plane[2]) < 1e-9:
                return unresolved("底板平面非法")
    except (ValueError, TypeError, OverflowError):
        return unresolved("输入格式非法")
    if len(bc) < 300 or len(lc) < 300:
        return unresolved("点太少")
    # carriage_box 按 0.25 m 分格逐格扫，范围要先限住（ROI 的 y 只有 19 m）。
    if any(float(c[:, 1].max()) - float(c[:, 1].min()) > 100.0 for c in (bc, lc)):
        return unresolved("点云 y 范围过大")
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            cb = carriage_box(bc, x_lim=LATERAL_X_LIM_M)
            cl = carriage_box(lc, x_lim=LATERAL_X_LIM_M)
            boxes = (cb, cl)
            if cb is None or cl is None:
                return unresolved("切不出车厢（%s）" % "、".join(
                    name for name, box in (("空车", cb), ("载货", cl)) if box is None), boxes)
            eb = _side_wall_envelope(bc, cb, baseline.bed_z_at, above_m)
            el = _side_wall_envelope(lc, cl, baseline.bed_z_at, above_m)
    except (FloatingPointError, OverflowError, ValueError):
        return unresolved("数值运算非有限")
    if eb is None or el is None:
        return unresolved("车厢中段侧壁点不足", boxes)
    dx_wall = (el[0] + el[1]) / 2 - (eb[0] + eb[1]) / 2
    dx_box = (cl["x0"] + cl["x1"]) / 2 - (cb["x0"] + cb["x1"]) / 2
    wb, wl = eb[1] - eb[0], el[1] - el[0]
    if not all(math.isfinite(v) for v in (dx_wall, dx_box, wb, wl)):
        return unresolved("数值运算非有限")
    if abs(dx_wall - dx_box) > agree_m:
        return unresolved("两种估法不一致：侧壁 %+.3f / 车厢盒 %+.3f m" % (dx_wall, dx_box), boxes)
    if abs(wb - wl) > width_tol_m:
        return unresolved("两边车厢外包宽度差 %.2f m（空车 %.2f / 载货 %.2f），"
                          "疑似一侧被截或不是同一辆车" % (abs(wb - wl), wb, wl), boxes)
    if abs(dx_wall) > max_dx_m:
        return unresolved("估出 %+.2f m 超过上限 %.1f m" % (dx_wall, max_dx_m), boxes)
    return LateralDecision(float(dx_wall), "resolved",
                           "侧壁 %+.3f / 车厢盒 %+.3f m" % (dx_wall, dx_box), boxes)


def _shift_baseline_x(baseline: "EmptyBaseline", dx: float) -> "EmptyBaseline":
    """基线整体沿 x 平移 dx：点云 x、车厢盒 x0/x1、底板平面的 d（d' = d - a*dx）。

    平面 a*x + b*y + c*z + d = 0 挪到 x+dx 处取同样的高度，所以 d 要减 a*dx。
    floor_z 不动 —— 它只作货高上限的参考，纵向 dy 也没有动它。
    """
    cloud = np.asarray(baseline.cloud, np.float64).copy()
    cloud[:, 0] += dx
    box = [float(v) for v in baseline.box]
    box[0] += dx
    box[1] += dx
    plane = baseline.bed_plane
    if plane is not None and len(plane) == 4:
        a, b, c, d = (float(v) for v in plane)
        plane = (a, b, c, d - a * dx)
    return replace(baseline, cloud=cloud, box=tuple(box), bed_plane=plane)


# ★ 姿态对齐（2026-10-01）—— 高差 + 横滚 + 俯仰。
#   载货车厢底板被煤盖住看不见，体积拿空车基线的底板当参考。同一辆车的车厢是一个整体，
#   两次停车之间它的高低、左右倾、前后倾都可能不同 —— 壁顶两次都看得见，逐段比一比就知道。
#   ▲ 不只是跨车道：满载时悬挂和轮胎会压下去。2026-09-28 宁A05922F 同一条道（CH4）
#     载货比空车低 4.1~5.2 cm（壁顶估计换参数都在这个范围；同一台雷达两帧之间地面
#     一致到 ±2 mm，不是雷达漂）。不修就是按空车的底板高度算煤厚，系统性少算一个下沉量。
#   ▲ 对体积起作用的几乎只有车厢中心的高差：横滚、俯仰在对称积分里基本抵消
#     （9-28 两对：只加横滚+俯仰体积变 -0.03% / +0.4%）。仍按整个场逐格加，
#     免得积分区中心与估计中心不重合时把俯仰项漏掉。
#   ▲ 高差估计依赖 dy 准：dy 偏 0.25 m，宁A05922F 估出的高差跟着变 1.2~2.5 cm
#     （车厢沿 y 有约 3° 的坡，壁顶还有台阶），而且 dy 错位本身还会让前后端板落进底板格
#     —— 后者要靠纵向配准解决，这里补不了。
POSE_SEG_M = 0.5          # 沿 y 分段
POSE_BAND_M = 0.20        # 壁顶条带：从局部外缘往里
POSE_BIN_M = 0.025        # 条带内按 x 分细格，每格取第 POSE_TOP_RANK 高，再取最高格
POSE_TOP_RANK = 3
POSE_MIN_SEGMENTS = 8     # 每侧壁至少这么多有效段……
POSE_MIN_SPAN_M = 4.0     # ……且跨这么长
POSE_MAX_SIGMA_M = 0.015  # 中心高差的标准误差上限
POSE_MAX_DZ_M = 0.15
POSE_MAX_ROLL_DEG = 8.0
POSE_MAX_PITCH_DEG = 6.0
POSE_MAX_TWIST = 0.02     # 两侧壁前后倾之差（m/m）；实测同一辆车 <= 0.01


@dataclass(frozen=True)
class PoseDecision:
    status: str                                 # resolved（照此修参考面）/ unresolved（不修）
    reason: str
    dx_m: float = 0.0                           # 估计时用的横向/纵向偏移，必须与积分时一致
    dy_m: float = 0.0
    left: Tuple[float, float] = (0.0, 0.0)      # 左壁高差直线（截距, 斜率），y 取相对 y_mid
    right: Tuple[float, float] = (0.0, 0.0)
    x_left: float = -1.0                        # 载货坐标里两侧壁外缘的 x
    x_right: float = 1.0
    y_mid: float = 0.0
    sigma_m: float = 0.0                        # 中心高差的标准误差（误差估计用）
    # 条件协方差的顺序：中心高差、横滚坡度、俯仰坡度。给定保留段和配准，假定段间及两壁误差独立；
    # 不包含段间相关、筛段后的选择偏差或上游配准不确定性，不能当作完整测量误差。
    conditional_covariance: Tuple[Tuple[float, ...], ...] = ()
    adopted_components: Tuple[bool, bool, bool] = (False, False, False)
    estimated_components: Tuple[float, ...] = ()  # 门控前的三个分量；不显著时仍保留给诊断
    estimated: bool = False                    # 通过整体质量门禁并算出条件误差，即使三个分量都未采用

    def at(self, x, y) -> np.ndarray:
        """载货坐标 (x, y) 处的高差（载货 - 空车）：两侧壁直线之间按 x 线性插值。"""
        x = np.asarray(x, np.float64)
        y = np.asarray(y, np.float64)
        if self.status != "resolved":
            return np.zeros(np.broadcast(x, y).shape)
        dl = self.left[0] + self.left[1] * (y - self.y_mid)
        dr = self.right[0] + self.right[1] * (y - self.y_mid)
        return dl + (dr - dl) * (x - self.x_left) / (self.x_right - self.x_left)


def _wall_top_series(cloud: np.ndarray, bed_z_at, edges: np.ndarray, guess: float,
                     side: str, check_coal: bool, slope: float = 0.0
                     ) -> Tuple[np.ndarray, np.ndarray, int]:
    """一侧壁逐段的壁顶高度（折算到段中点）。返回 (壁顶 z, 局部外缘 x, 因煤剔除的段数)；取不到为 nan。

    壁顶 = 局部外缘往里 POSE_BAND_M 条带内、按 x 分细格取第 POSE_TOP_RANK 高、再取最高格。
    ▲ 不能用整条带的分位数（p95/p98）：空车内侧面整面可见、载货大半被煤挡住，
      条带里"面上的点"与"顶上的点"比例两种状态不一样，分位数会落在不同位置
      （9-28 实测个别段因此偏 8~10 cm）。取最高处，两种状态量的是同一个特征。
    ▲ 外缘逐段找，不用车厢中段的外包：墙沿车厢不完全平直，固定条带到端部会偏出去。
    ▲ 载货一侧若内侧煤面够到壁顶（第 3 高 >= 壁顶 - 3 cm），这一段的"壁顶"可能是煤，剔除。
    ▲ slope 是这片点云壁顶沿 y 的坡度：先按它把段内的点拉平再取最高。不拉平的话取到的是
      段内坡上的高端，两次扫描前后倾不同时差值里会混进 (|坡度差|) x 段长/2 ——
      宁A66912A 空车俯仰 +3.8°、载货 +1.1°，不拉平要偏约 1 cm。
    """
    sgn = 1.0 if side == "L" else -1.0
    above = cloud[(cloud[:, 2] - bed_z_at(cloud[:, 0], cloud[:, 1])) > 0.30]
    near = above[np.abs(above[:, 0] - guess) < 0.35]
    near = near[np.argsort(near[:, 1], kind="stable")]
    bounds = np.searchsorted(near[:, 1], edges)
    tops = np.full(len(edges) - 1, np.nan)
    xs = np.full(len(edges) - 1, np.nan)
    n_coal = 0
    for i in range(len(edges) - 1):
        s = near[bounds[i]:bounds[i + 1]]
        if len(s) < 30:
            continue
        edge = float(np.percentile(s[:, 0], 0.5 if side == "L" else 99.5))
        inner = sgn * (s[:, 0] - edge)
        flat = s[:, 2] - slope * (s[:, 1] - 0.5 * (edges[i] + edges[i + 1]))
        top = -math.inf
        for b0 in np.arange(-0.02, POSE_BAND_M, POSE_BIN_M):
            z = flat[(inner >= b0) & (inner < b0 + POSE_BIN_M)]
            if len(z) >= POSE_TOP_RANK + 2:
                top = max(top, float(np.partition(z, -POSE_TOP_RANK)[-POSE_TOP_RANK]))
        if not math.isfinite(top):
            continue
        if check_coal:
            cz = flat[(inner > POSE_BAND_M + 0.05) & (inner <= POSE_BAND_M + 0.35)]
            if len(cz) >= 15 and float(np.partition(cz, -3)[-3]) >= top - 0.03:
                n_coal += 1
                continue
        tops[i], xs[i] = top, edge
    return tops, xs, n_coal


def _robust_wall_line(y: np.ndarray, d: np.ndarray) -> Optional[dict]:
    """一侧壁逐段高差的抗离群直线：重复中位数斜率（一半以下的坏段拉不动）-> 截距取中位
    -> 剔除 |残差| > max(3*MAD, 3 cm) 的段 -> 剩下的最小二乘重拟合。

    坏段的来源都见过：立柱/台阶在两次扫描里没对上；宁A66912A 右壁后段的边板空车时放低、
    装货时立起来（载货反而高 22~37 cm）—— 车厢局部真的变了形。
    """
    m = np.isfinite(d)
    y, d = y[m], d[m]
    n = len(d)
    if n < 4:
        return None
    dyy = y[None, :] - y[:, None]
    off = ~np.eye(n, dtype=bool)
    slopes = ((d[None, :] - d[:, None]) / np.where(off, dyy, 1.0))[off].reshape(n, n - 1)
    slope = float(np.median(np.median(slopes, axis=1)))
    icpt = float(np.median(d - slope * y))
    r = d - (icpt + slope * y)
    mad = 1.4826 * float(np.median(np.abs(r - np.median(r))))
    keep = np.abs(r) <= max(3.0 * mad, 0.03)
    if int(keep.sum()) < 3:
        return None
    A = np.column_stack([np.ones(int(keep.sum())), y[keep]])
    coef = np.linalg.lstsq(A, d[keep], rcond=None)[0]
    res = d[keep] - A @ coef
    sigma = float(np.sqrt(np.sum(res ** 2) / max(1, int(keep.sum()) - 2)))
    # 斜率方差含实际纵向跨度：var_b = sigma^2 / sum((y - mean(y))^2)。缺段后 y 的均值不一定为零，
    # 截距与斜率的协方差也必须保留，供中心高差/横滚/俯仰的联合误差传播使用。
    covariance = sigma ** 2 * np.linalg.inv(A.T @ A)
    var_a = float(covariance[0, 0])
    return dict(a=float(coef[0]), b=float(coef[1]), sigma=sigma, var_a=var_a,
                var_b=float(covariance[1, 1]), cov_ab=float(covariance[0, 1]),
                n=int(keep.sum()), span=float(np.ptp(y[keep])))


def _paired_clouds(baseline: "EmptyBaseline", loaded: Optional["LoadedResult"]):
    """偏航 / 姿态 / 交叉校核共用的输入校验：合格返回 (空车点云, 载货点云)，否则返回原因字符串。"""
    if loaded is None or baseline is None or baseline.n == 0:
        return "缺基线或载货点云"
    try:
        bc = np.asarray(baseline.cloud, np.float64)
        lc = np.asarray(loaded.cloud, np.float64)
        for cloud in (bc, lc):
            if cloud.ndim != 2 or cloud.shape[1] != 3 or not np.isfinite(cloud).all():
                return "点云非法"
        if len(bc) != baseline.n or not VolumeResult._finite(baseline.floor_z):
            return "基线非法"
        if baseline.bed_plane is not None:
            plane = np.asarray(baseline.bed_plane, np.float64)
            if plane.shape != (4,) or not np.isfinite(plane).all() or abs(plane[2]) < 1e-9:
                return "底板平面非法"
    except (ValueError, TypeError, OverflowError):
        return "输入格式非法"
    if len(bc) < 300 or len(lc) < 300:
        return "点太少"
    # carriage_box 按 0.25 m 分格逐格扫，范围要先限住（ROI 的 y 只有 19 m）。
    if any(float(c[:, 1].max()) - float(c[:, 1].min()) > 100.0 for c in (bc, lc)):
        return "点云 y 范围过大"
    return bc, lc


def _profiles_for(bc: np.ndarray, lc: np.ndarray, baseline: "EmptyBaseline", loaded_bed, dy: float, boxes):
    """切车厢（有现成的就复用）再取逐段壁顶；切不出时返回原因字符串。"""
    cb, cl = boxes if boxes is not None else (carriage_box(bc, x_lim=LATERAL_X_LIM_M),
                                               carriage_box(lc, x_lim=LATERAL_X_LIM_M))
    if cb is None or cl is None:
        return "切不出车厢（%s）" % "、".join(name for name, box in (("空车", cb), ("载货", cl)) if box is None)
    return _wall_profiles(bc, lc, baseline, loaded_bed, dy, cb, cl)


def shared_wall_profiles(baseline: "EmptyBaseline", loaded: Optional["LoadedResult"], dx: float, dy: float,
                         boxes=None):
    """按 dx / dy 一次算好逐段壁顶，供偏航与姿态估计共用（compute_volume_auto / compute_volume 里用）。
    出任何问题都返回原因字符串 —— 估计函数拿到字符串就按"估不出"处理。"""
    got = _paired_clouds(baseline, loaded)
    if isinstance(got, str):
        return got
    if not (VolumeResult._finite(dx) and VolumeResult._finite(dy)):
        return "偏移非法"
    bc, lc = got
    dx, dy = float(dx), float(dy)

    def loaded_bed(x, y):
        return baseline.bed_z_at(np.asarray(x) - dx, np.asarray(y) - dy)
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            return _profiles_for(bc, lc, baseline, loaded_bed, dy, boxes)
    except (FloatingPointError, OverflowError, ValueError, np.linalg.LinAlgError):
        return "数值运算非有限"


def _wall_profiles(bc: np.ndarray, lc: np.ndarray, baseline: "EmptyBaseline", loaded_bed,
                   dy: float, cb: dict, cl: dict):
    """两次扫描两侧壁的逐段壁顶，段取两次车厢的重合段、坐标用载货的。偏航、姿态对齐与交叉校核共用。

    返回 dict：eb/el（空车/载货中段外包）、edges（载货坐标的段界）、ym（段中点相对 y_mid）、
    y_mid、以及 "L"/"R" -> (空车壁顶, 载货壁顶, 载货局部外缘 x, 因煤剔除的段数, 空车局部外缘 x)；
    空车外缘在空车自己的坐标里（未挪 dx）。取不到时返回原因字符串。
    loaded_bed 是载货点处的基线底板高度（只用来筛掉底板以下的点）。
    """
    eb = _side_wall_envelope(bc, cb, baseline.bed_z_at, 0.30)
    el = _side_wall_envelope(lc, cl, loaded_bed, 0.30)
    if eb is None or el is None:
        return "车厢中段侧壁点不足"
    y0 = max(float(cl["y0"]), float(cb["y0"]) + dy) + 0.3
    y1 = min(float(cl["y1"]), float(cb["y1"]) + dy) - 0.3
    if y1 - y0 < POSE_MIN_SPAN_M:
        return "两次车厢重合段只有 %.1f m" % max(0.0, y1 - y0)
    edges = np.arange(y0, y1 - POSE_SEG_M * 0.5, POSE_SEG_M)
    edges = np.append(edges, edges[-1] + POSE_SEG_M)
    y_mid = (y0 + y1) / 2
    ym = (edges[:-1] + edges[1:]) / 2 - y_mid
    out = dict(eb=eb, el=el, edges=edges, ym=ym, y_mid=y_mid)
    for side, ge, gl in (("L", eb[0], el[0]), ("R", eb[1], el[1])):
        # 两遍：先不拉平粗估各自壁顶的坡度（段内取高端只带来常数偏移，坡度不受影响），
        # 再按各自的坡度拉平重取。
        tb, xb, _ = _wall_top_series(bc, baseline.bed_z_at, edges - dy, ge, side, False)
        tl, xl, nc = _wall_top_series(lc, loaded_bed, edges, gl, side, True)
        fb, fl = _robust_wall_line(ym, tb), _robust_wall_line(ym, tl)
        if fb is not None:
            tb, xb, _ = _wall_top_series(bc, baseline.bed_z_at, edges - dy, ge, side, False, fb["b"])
        if fl is not None:
            tl, xl, nc = _wall_top_series(lc, loaded_bed, edges, gl, side, True, fl["b"])
        out[side] = (tb, tl, xl, nc, xb)
    return out


def estimate_pose_registration(baseline: "EmptyBaseline", loaded: Optional["LoadedResult"],
                               dx: float = 0.0, dy: float = 0.0,
                               boxes: Optional[Tuple[Optional[dict], Optional[dict]]] = None,
                               profiles=None) -> PoseDecision:
    """估载货车厢相对空车基线的高差场（载货坐标），估不出或不可信就不修（= 旧行为）。

    dx / dy 必须与积分时挪基线用的一致 —— 逐段对应关系由它们决定。
    boxes 是横向估计对**同一对点云**切好的车厢盒（LateralDecision.boxes），有就复用；
    profiles 是按同一 dx / dy 算好的逐段壁顶（_wall_profiles 的结果），有就复用。
    以下任一条不修：任一侧壁有效段 < POSE_MIN_SEGMENTS 或跨不到 POSE_MIN_SPAN_M；
    中心高差标准误差 > POSE_MAX_SIGMA_M；高差/横滚/俯仰超过上限；两侧壁前后倾之差
    > POSE_MAX_TWIST（不像同一个车厢的整体移动）。通过后，高差/横滚坡度/俯仰坡度分别按自身
    两倍条件标准误判断，不显著的分量置零。两壁采用同一俯仰坡度，不把局部扭转带进刚性参考面。
    """
    if not (VolumeResult._finite(dx) and VolumeResult._finite(dy)):
        return PoseDecision("unresolved", "偏移非法")
    dx, dy = float(dx), float(dy)

    def unresolved(reason):                     # 估不出也记下所用偏移，免得各档反复重估
        return PoseDecision("unresolved", reason, dx, dy)
    if loaded is None or baseline is None or baseline.n == 0:
        return unresolved("缺基线或载货点云")
    try:
        bc = np.asarray(baseline.cloud, np.float64)
        lc = np.asarray(loaded.cloud, np.float64)
        for cloud in (bc, lc):
            if cloud.ndim != 2 or cloud.shape[1] != 3 or not np.isfinite(cloud).all():
                return unresolved("点云非法")
        if len(bc) != baseline.n or not VolumeResult._finite(baseline.floor_z):
            return unresolved("基线非法")
        if baseline.bed_plane is not None:
            plane = np.asarray(baseline.bed_plane, np.float64)
            if plane.shape != (4,) or not np.isfinite(plane).all() or abs(plane[2]) < 1e-9:
                return unresolved("底板平面非法")
    except (ValueError, TypeError, OverflowError):
        return unresolved("输入格式非法")
    if len(bc) < 300 or len(lc) < 300:
        return unresolved("点太少")
    if any(float(c[:, 1].max()) - float(c[:, 1].min()) > 100.0 for c in (bc, lc)):
        return unresolved("点云 y 范围过大")

    def loaded_bed(x, y):                       # 载货点处、挪齐后的基线底板高度
        return baseline.bed_z_at(np.asarray(x) - dx, np.asarray(y) - dy)
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            prof = profiles if profiles is not None else _profiles_for(bc, lc, baseline, loaded_bed, dy, boxes)
            if isinstance(prof, str):
                return unresolved(prof)
            ym, y_mid = prof["ym"], prof["y_mid"]
            lines, xpos, n_coal = {}, {}, 0
            for side, name in (("L", "左"), ("R", "右")):
                tb, tl, xl, nc, _xb = prof[side]
                n_coal += nc
                line = _robust_wall_line(ym, tl - tb)
                if line is None or line["n"] < POSE_MIN_SEGMENTS or line["span"] < POSE_MIN_SPAN_M:
                    return unresolved("%s壁有效段不足（%d 段，跨 %.1f m）" % (
                        name, 0 if line is None else line["n"], 0.0 if line is None else line["span"]))
                lines[side] = line
                xpos[side] = float(np.median(xl[np.isfinite(xl)]))
    except (FloatingPointError, OverflowError, ValueError, np.linalg.LinAlgError):
        return unresolved("数值运算非有限")
    L, R = lines["L"], lines["R"]
    width = xpos["R"] - xpos["L"]
    if not (math.isfinite(width) and width > 1.0):
        return unresolved("两侧壁位置非法")
    c0 = (L["a"] + R["a"]) / 2
    sigma0 = 0.5 * math.sqrt(L["var_a"] + R["var_a"])
    roll_slope = (R["a"] - L["a"]) / width
    pitch_slope = (L["b"] + R["b"]) / 2
    roll = math.degrees(math.atan(roll_slope))
    pitch = math.degrees(math.atan(pitch_slope))
    twist = R["b"] - L["b"]
    if not all(math.isfinite(v) for v in (c0, sigma0, roll, pitch, twist)):
        return unresolved("数值运算非有限")
    summary = "中心高差 %+.1f±%.1f cm，横滚 %+.2f°，俯仰 %+.2f°" % (c0 * 100, sigma0 * 100, roll, pitch)
    if n_coal:
        summary += "，剔除煤面够到壁顶的 %d 段" % n_coal
    if sigma0 > POSE_MAX_SIGMA_M:
        return unresolved("中心高差不确定度 ±%.1f cm 超过 %.1f cm" % (sigma0 * 100, POSE_MAX_SIGMA_M * 100))
    if abs(c0) > POSE_MAX_DZ_M:
        return unresolved("中心高差 %+.1f cm 超过上限 %.0f cm" % (c0 * 100, POSE_MAX_DZ_M * 100))
    if abs(roll) > POSE_MAX_ROLL_DEG or abs(pitch) > POSE_MAX_PITCH_DEG:
        return unresolved("倾角超过上限（横滚 %+.1f°，俯仰 %+.1f°）" % (roll, pitch))
    if abs(twist) > POSE_MAX_TWIST:
        return unresolved("两侧壁前后倾相差 %.1f cm/m，不像同一个车厢的整体移动" % (abs(twist) * 100))
    # 原代码仅以中心升降是否显著否决整个姿态，纯俯仰会被误关；薄煤负半边截为零后不再能抵消。
    # 两壁拟合先按独立误差组成4维协方差，再投影到三个姿态分量，保留截距/斜率的相关项。
    source_cov = np.zeros((4, 4), np.float64)
    for start, line in ((0, L), (2, R)):
        source_cov[start:start + 2, start:start + 2] = (
            (line["var_a"], line["cov_ab"]), (line["cov_ab"], line["var_b"]))
    transform = np.array([[0.5, 0.0, 0.5, 0.0], [-1.0 / width, 0.0, 1.0 / width, 0.0],
                          [0.0, 0.5, 0.0, 0.5]])
    covariance = transform @ source_cov @ transform.T
    if not np.isfinite(covariance).all() or np.any(np.diag(covariance) < 0):
        return unresolved("姿态条件协方差非法")
    components = (float(c0), float(roll_slope), float(pitch_slope))
    errors = np.sqrt(np.diag(covariance))
    adopted = tuple(bool(abs(value) > 0.0 and abs(value) >= 2.0 * error)
                    for value, error in zip(components, errors))
    applied_c, applied_r, applied_p = (value if use else 0.0 for value, use in zip(components, adopted))
    summary += "，逐项两倍条件标准误：" + "、".join(
        name + ("采用" if use else "置零") for name, use in zip(("高差", "横滚", "俯仰"), adopted))
    return PoseDecision(
        "resolved" if any(adopted) else "unresolved", summary, dx, dy,
        (applied_c - applied_r * width / 2, applied_p), (applied_c + applied_r * width / 2, applied_p),
        xpos["L"], xpos["R"], y_mid, float(sigma0),
        conditional_covariance=tuple(tuple(float(v) for v in row) for row in covariance),
        adopted_components=adopted, estimated_components=components, estimated=True)


# ★ 偏航配准（2026-10-02）—— 车厢绕竖直轴的转角。
#   空车、载货各自做航向矫正，两次的结果差半度左右：9-28 两对 +0.49° / +0.71°，
#   左右两侧壁各自量出的斜率 0.74/0.97 与 1.20/1.27 cm/m，逐段残差只有 0.9~1.6 cm。
#   横向配准只对齐了车厢中段，两端还各差 3~6 cm —— 而体积对横向错位是窄谷（错 3 cm 就 +2.4%），
#   端部的壁顶照样落进底板格。9-28 两车在旧积分盒上补偏航后 +2.9% / -1.8%；
#   在内腔积分域上约 +0.1% / -0.3%。比较必须注明积分域，不能把旧盒+偏航误称为10-01口径。
#   ▲ 只补斜率，不补逐段外缘拟合出的残余横移：宁A05922F 两侧壁残余横移 +0.5 / +3.0 cm，
#     两侧往外分开了 2.5 cm，更像满载时侧板被撑开而不是整车横移；横移仍用横向配准的估计。
YAW_MAX_SLOPE = 0.035      # 约 2°
YAW_AGREE = 0.006          # 两侧壁斜率差上限（0.6 cm/m）；9-28 两对实测 0.23 / 0.07 cm/m


@dataclass(frozen=True)
class YawDecision:
    status: str              # resolved（照此转）/ unresolved（不转）
    reason: str
    slope: float = 0.0       # 载货坐标里：基线 x 要加 slope x (y - y_mid)
    y_mid: float = 0.0       # 转轴所在的 y（载货坐标）
    dx_m: float = 0.0        # 估计时用的横向/纵向偏移，必须与积分时一致
    dy_m: float = 0.0


def estimate_yaw_registration(baseline: "EmptyBaseline", loaded: Optional["LoadedResult"],
                              dx: float = 0.0, dy: float = 0.0,
                              boxes: Optional[Tuple[Optional[dict], Optional[dict]]] = None,
                              profiles=None) -> YawDecision:
    """估载货车厢相对空车基线的偏航：两侧壁逐段外缘之差（载货 - 挪过 dx 的空车）沿车厢的斜率。

    两侧壁各拟合一条抗离群直线（同姿态对齐），斜率取平均。以下任一条不补（= 旧行为）：
    任一侧壁有效段不足；两侧斜率差超过 YAW_AGREE（整车转动时两侧必须一样）；超过 YAW_MAX_SLOPE。
    boxes / profiles 同 estimate_pose_registration。
    """
    if not (VolumeResult._finite(dx) and VolumeResult._finite(dy)):
        return YawDecision("unresolved", "偏移非法")
    dx, dy = float(dx), float(dy)

    def unresolved(reason):                     # 估不出也记下所用偏移，免得各档反复重估
        return YawDecision("unresolved", reason, 0.0, 0.0, dx, dy)
    got = _paired_clouds(baseline, loaded)
    if isinstance(got, str):
        return unresolved(got)
    bc, lc = got

    def loaded_bed(x, y):
        return baseline.bed_z_at(np.asarray(x) - dx, np.asarray(y) - dy)
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            prof = profiles if profiles is not None else _profiles_for(bc, lc, baseline, loaded_bed, dy, boxes)
            if isinstance(prof, str):
                return unresolved(prof)
            ym, y_mid = prof["ym"], prof["y_mid"]
            lines = {}
            for side, name in (("L", "左"), ("R", "右")):
                _tb, _tl, xl, _nc, xb = prof[side]
                line = _robust_wall_line(ym, xl - (xb + dx))
                if line is None or line["n"] < POSE_MIN_SEGMENTS or line["span"] < POSE_MIN_SPAN_M:
                    return unresolved("%s壁外缘有效段不足（%d 段，跨 %.1f m）" % (
                        name, 0 if line is None else line["n"], 0.0 if line is None else line["span"]))
                lines[side] = line
    except (FloatingPointError, OverflowError, ValueError, np.linalg.LinAlgError):
        return unresolved("数值运算非有限")
    kl, kr = lines["L"]["b"], lines["R"]["b"]
    if not (math.isfinite(kl) and math.isfinite(kr)):
        return unresolved("数值运算非有限")
    k = (kl + kr) / 2
    detail = "左壁 %+.2f / 右壁 %+.2f cm/m" % (kl * 100, kr * 100)
    if abs(kl - kr) > YAW_AGREE:
        return unresolved("两侧壁不一致（%s），不像整车转动" % detail)
    if abs(k) > YAW_MAX_SLOPE:
        return unresolved("%+.2f° 超过上限 %.1f°" % (math.degrees(math.atan(k)),
                                                     math.degrees(math.atan(YAW_MAX_SLOPE))))
    return YawDecision("resolved", detail, float(k), float(y_mid), dx, dy)


def _shear_baseline_x(baseline: "EmptyBaseline", k: float, y0: float) -> "EmptyBaseline":
    """基线沿 x 剪切：x' = x + k (y - y0)，小角度下就是绕 y0 处转 atan(k)。

    底板平面 a*x + b*y + c*z + d = 0 换成 x' 后：b' = b - a*k，d' = d + a*k*y0；车厢盒 x 范围取四角。
    """
    cloud = np.asarray(baseline.cloud, np.float64).copy()
    cloud[:, 0] += k * (cloud[:, 1] - y0)
    x0, x1, by0, by1, z0, z1 = (float(v) for v in baseline.box)
    box = (min(x0 + k * (by0 - y0), x0 + k * (by1 - y0)), max(x1 + k * (by0 - y0), x1 + k * (by1 - y0)),
           by0, by1, z0, z1)
    plane = baseline.bed_plane
    if plane is not None and len(plane) == 4:
        a, b, c, d = (float(v) for v in plane)
        plane = (a, b - a * k, c, d + a * k * y0)
    return replace(baseline, cloud=cloud, box=box, bed_plane=plane)


# 复算前预留网格时用：横向挪动 + 偏航剪切在车厢两端（半长按 15 m 算）最多再撑开这么多。
ALIGN_X_MARGIN_M = LATERAL_MAX_DX_M + YAW_MAX_SLOPE * 15.0


# ★ 内腔积分域（2026-10-02）—— 积分只在两侧壁**内侧面**之间、两端底板之间做。
#   旧积分盒 = 底板点包围盒、横向再按货的范围放宽（限幅 ±0.3 m），9-28 两对宽 2.58 / 2.61 m，
#   而侧壁内侧面量出的内宽只有 2.40 / 2.36 m —— 多出来的是厢壁自己的占地：壁格的货高按邻居补，
#   等于把厢壁当成煤，实测多算 2.35 / 2.52 m^3（8.0% / 8.3%）。盒边按格心取舍，
#   体积还随网格原点挪几厘米变 3.7%~6.2%。现在：
#     · 两侧：空车侧壁内侧面（离底板 0.3~1.2 m 的侧面点逐段取 x 中位，抗离群直线），
#       随横向配准、偏航一起挪；
#     · 两端：空车底板带（底板平面 ±0.5 m）纵向 p0.2~p99.8，随 dy 挪 —— 9-28 两对 11.30 / 10.84 m，
#       双雷达独立量的内长 11.343 / 10.834 m；
#     · 边界格按落在域内的面积计权（相位噪声降到 0.2%）；
#     · 载货点离内侧面不到 INNER_SIDE_GAP_M 的不用：壁面在煤面以上那一截的回波，
#       按格取最高就成了"煤顶"。0.05 m 不够（横向偏 3 cm 体积就 +2.7%）；0.08~0.15 m 之间体积一致到
#       0.1%，说明按邻居补没有偏。取 0.15 m：横向配准对齐的是壁顶外缘，内侧面还可能差约 2 cm
#       （9-28 两对 +1.9 / +2.2 cm），0.10 m 时宁A05922F 往这一侧再偏 2 cm 体积就 +1.5%，0.15 m 时不跳；
#     · 两端：空车端板占的（纵向, 离底板高）范围膨胀 INNER_END_DILATE_M 后，落进去的载货点不用 ——
#       端板顶往里倾（宁A05922F 前板根部 +6.10、顶部 +6.00~+6.05，宁A66912A 前端 1.3~1.5 m 高处
#       还有一道往里伸的檐），最高处在积分域里面，按格取最高会把板顶当成煤，实测多算约 3%。
#       膨胀 0.15 m 时 dy 偏 0.25 m 掩模就错位、体积回升 3%~4%；0.30 m 时 dy 偏 ±0.25 m 只变
#       ±0.6%，dy 准时比 0.15 m 只多 0.1% / 0.65%。dy 按车厢后沿逐行（0.25 m）找，偏一行并非不可能。
#   被剔掉点的格按邻居补，算结构盲区（壁格），不算空洞 —— 否则 hole_ratio 会结构性涨十几个点。
#   ▲ 只在横向位置可信时用：横向估不出就退回旧积分盒（= 旧行为），内侧面放在没对齐的地方更糟。
INNER_FACE_BAND_M = (0.30, 1.20)
INNER_FACE_WINDOW_M = 0.35       # 内侧面点离壁顶外缘不超过这么远
INNER_FACE_MIN_PTS = 20          # 每段至少这么多侧面点
INNER_WIDTH_BAND_M = (1.8, 3.0)
INNER_MAX_TAPER = 0.01           # 两侧内侧面斜率差上限（m/m）：空车车厢两壁应当平行
INNER_FLOOR_BAND_M = 0.5
INNER_END_PCT = (0.2, 99.8)
INNER_MIN_LENGTH_M = 2.0
INNER_SIDE_GAP_M = 0.15
INNER_END_BAND_M = 0.6           # 两端各这么长里找端板
INNER_END_MIN_H_M = 0.15
INNER_END_CORE_M = 1.0           # 只看车厢中线两侧这么宽（避开侧壁）
INNER_END_GRID_M = 0.05
INNER_END_DILATE_M = 0.30
INNER_END_DILATE_H_M = 0.05


@dataclass(frozen=True)
class InnerDomain:
    status: str                                 # resolved（照此积分）/ unresolved（退回旧积分盒）
    reason: str
    left: Tuple[float, float] = (0.0, 0.0)      # 空车坐标里的内侧面：x = 截距 + 斜率 x (y - y_c)
    right: Tuple[float, float] = (0.0, 0.0)
    y_c: float = 0.0
    y0: float = 0.0                             # 空车坐标里的内腔两端
    y1: float = 0.0
    # 端板占据表（空车坐标）：(纵向起点, 离底板高起点, 格宽, 已膨胀的 bool 表[纵向, 高度])；
    # 两端都没见到端板时为 None。不参与比较。
    ends: Optional[Tuple[float, float, float, np.ndarray]] = field(default=None, compare=False, repr=False)

    def edges(self, y, dx: float = 0.0, dy: float = 0.0, k: float = 0.0, y_piv: float = 0.0):
        """载货坐标 y 处两侧内侧面的 x（基线挪 dx / dy、绕 y_piv 剪切 k 之后）。"""
        y = np.asarray(y, np.float64)
        u = y - dy - self.y_c
        shift = dx + k * (y - y_piv)
        return (self.left[0] + self.left[1] * u + shift, self.right[0] + self.right[1] * u + shift)

    def in_end_walls(self, y_base, h) -> np.ndarray:
        """空车坐标纵向 y_base、离（载货时）底板高 h 的点是否落在膨胀后的端板范围里。"""
        y_base = np.asarray(y_base, np.float64)
        out = np.zeros(y_base.shape, bool)
        if self.ends is None:
            return out
        ylo, hlo, g, table = self.ends
        iy = np.floor((y_base - ylo) / g)
        ih = np.floor((np.asarray(h, np.float64) - hlo) / g)
        ok = (iy >= 0) & (iy < table.shape[0]) & (ih >= 0) & (ih < table.shape[1])
        out[ok] = table[iy[ok].astype(np.int64), ih[ok].astype(np.int64)]
        return out


def _strip_grid_weights(gx, gy, cell, ya, yb, edges):
    """直线侧壁之间的积分域与每格的精确交叠面积占比，包含部分端行。"""
    low = np.clip(gy - cell / 2, ya, yb)
    high = np.clip(gy + cell / 2, ya, yb)
    left0, right0 = edges(low)
    left1, right1 = edges(high)

    def left_fraction(edge0, edge1, xlow):
        # 每一水平截面内，侧线左边占一格的比例是 clip((edge-xlow)/cell, 0, 1)。
        # 侧线在端行内仍会变化；在穿过格边的两个断点分段积分，不能只量格心。
        lo = (np.minimum(edge0, edge1)[:, None] - xlow) / cell
        hi = (np.maximum(edge0, edge1)[:, None] - xlow) / cell
        span = np.abs(edge1 - edge0)[:, None] / cell
        start = np.clip(np.divide(-lo, span, out=np.zeros_like(lo), where=span > 0), 0., 1.)
        end = np.clip(np.divide(1. - lo, span, out=np.ones_like(lo), where=span > 0), 0., 1.)
        return (np.clip(lo, 0., 1.) + np.clip(hi, 0., 1.)) * (end - start) / 2 + 1. - end

    # 限制精确积分的临时数组；保留全部格子及同一浮点公式。
    weight = np.empty((len(gy), len(gx)), dtype=np.float64)
    columns = min(len(gx), 262144)
    rows = max(1, 262144 // columns)
    for start_y in range(0, len(gy), rows):
        rs = slice(start_y, start_y + rows)
        for start_x in range(0, len(gx), columns):
            cs = slice(start_x, start_x + columns)
            xlow = gx[cs][None, :] - cell / 2
            wx = np.clip(left_fraction(right0[rs], right1[rs], xlow)
                         - left_fraction(left0[rs], left1[rs], xlow), 0., 1.)
            weight[rs, cs] = wx * np.clip((high[rs] - low[rs]) / cell, 0., 1.)[:, None]
    return weight


def _dilate_bool(occ: np.ndarray, k0: int, k1: int) -> np.ndarray:
    """二维 bool 表按矩形结构元膨胀（两个方向各 k0 / k1 格），不回绕。"""
    out = occ.copy()
    for d in range(1, k0 + 1):
        out[d:, :] |= occ[:-d, :]
        out[:-d, :] |= occ[d:, :]
    mid = out.copy()
    for d in range(1, k1 + 1):
        out[:, d:] |= mid[:, :-d]
        out[:, :-d] |= mid[:, d:]
    return out


def estimate_inner_domain(baseline: "EmptyBaseline", box: Optional[dict] = None) -> InnerDomain:
    """从空车基线量内腔：两侧内侧面直线、两端底板范围、两端端板占据（见上）。量不出就不用。

    box 是这片空车点云切好的车厢盒（LateralDecision.boxes[0]），有就复用。只与基线有关，
    compute_volume_auto 估一次、各档共用。
    """
    def unresolved(reason):
        return InnerDomain("unresolved", reason)
    if baseline is None or baseline.n == 0:
        return unresolved("缺基线")
    try:
        bc = np.asarray(baseline.cloud, np.float64)
        if bc.ndim != 2 or bc.shape[1] != 3 or not np.isfinite(bc).all():
            return unresolved("点云非法")
        if len(bc) != baseline.n or not VolumeResult._finite(baseline.floor_z):
            return unresolved("基线非法")
        if baseline.bed_plane is not None:
            plane = np.asarray(baseline.bed_plane, np.float64)
            if plane.shape != (4,) or not np.isfinite(plane).all() or abs(plane[2]) < 1e-9:
                return unresolved("底板平面非法")
    except (ValueError, TypeError, OverflowError):
        return unresolved("输入格式非法")
    if len(bc) < 300:
        return unresolved("点太少")
    if float(bc[:, 1].max()) - float(bc[:, 1].min()) > 100.0:
        return unresolved("点云 y 范围过大")
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            cb = box if box is not None else carriage_box(bc, x_lim=LATERAL_X_LIM_M)
            if cb is None:
                return unresolved("空车切不出车厢")
            hgt = bc[:, 2] - baseline.bed_z_at(bc[:, 0], bc[:, 1])
            eb = _side_wall_envelope(bc, cb, baseline.bed_z_at, 0.30)
            if eb is None:
                return unresolved("空车车厢中段侧壁点不足")
            ya, yb = float(cb["y0"]) + 0.3, float(cb["y1"]) - 0.3
            if yb - ya < POSE_MIN_SPAN_M:
                return unresolved("空车车厢只有 %.1f m" % max(0.0, yb - ya))
            seg = np.arange(ya, yb - POSE_SEG_M * 0.5, POSE_SEG_M)
            seg = np.append(seg, seg[-1] + POSE_SEG_M)
            y_c = (ya + yb) / 2
            ym = (seg[:-1] + seg[1:]) / 2 - y_c
            face = bc[(hgt > INNER_FACE_BAND_M[0]) & (hgt < INNER_FACE_BAND_M[1])]
            lines = {}
            for side, name, guess in (("L", "左", eb[0]), ("R", "右", eb[1])):
                s = face[np.abs(face[:, 0] - guess) < INNER_FACE_WINDOW_M]
                s = s[np.argsort(s[:, 1], kind="stable")]
                cut = np.searchsorted(s[:, 1], seg)
                xs = np.full(len(seg) - 1, np.nan)
                for i in range(len(seg) - 1):
                    if cut[i + 1] - cut[i] >= INNER_FACE_MIN_PTS:
                        xs[i] = float(np.median(s[cut[i]:cut[i + 1], 0]))
                line = _robust_wall_line(ym, xs)
                if line is None or line["n"] < POSE_MIN_SEGMENTS or line["span"] < POSE_MIN_SPAN_M:
                    return unresolved("%s壁内侧面有效段不足（%d 段）" % (name, 0 if line is None else line["n"]))
                lines[side] = line
            L, R = lines["L"], lines["R"]
            width = R["a"] - L["a"]
            if not INNER_WIDTH_BAND_M[0] <= width <= INNER_WIDTH_BAND_M[1]:
                return unresolved("内宽 %.2f m 不在 %.1f~%.1f m" % (width, *INNER_WIDTH_BAND_M))
            if abs(R["b"] - L["b"]) > INNER_MAX_TAPER:
                return unresolved("两侧内侧面不平行（相差 %.1f cm/m）" % (abs(R["b"] - L["b"]) * 100))
            bed = bc[np.abs(hgt) <= INNER_FLOOR_BAND_M]
            if len(bed) < 200:
                return unresolved("空车底板点不足")
            y0, y1 = (float(v) for v in np.percentile(bed[:, 1], INNER_END_PCT))
            if y1 - y0 < INNER_MIN_LENGTH_M:
                return unresolved("内长只有 %.2f m" % (y1 - y0))
            # 端板：车厢中线两侧 INNER_END_CORE_M 内、两端各 INNER_END_BAND_M 里高出底板的点。
            cx = (L["a"] + R["a"]) / 2 + (L["b"] + R["b"]) / 2 * (bc[:, 1] - y_c)
            m = ((np.abs(bc[:, 0] - cx) < INNER_END_CORE_M) & (hgt > INNER_END_MIN_H_M)
                 & ((bc[:, 1] < y0 + INNER_END_BAND_M) | (bc[:, 1] > y1 - INNER_END_BAND_M)))
            ends, seen = None, []
            if np.any(m):
                g = INNER_END_GRID_M
                ky = int(round(INNER_END_DILATE_M / g))
                kh = int(round(INNER_END_DILATE_H_M / g))
                py, ph = bc[m, 1], hgt[m]
                ylo = float(py.min()) - (ky + 1) * g
                hlo = float(ph.min()) - (kh + 1) * g
                ny = int((float(py.max()) - ylo) / g) + ky + 2
                nh = int((float(ph.max()) - hlo) / g) + kh + 2
                if ny * nh > 2_000_000:
                    return unresolved("端板范围过大")
                occ = np.zeros((ny, nh), bool)
                occ[((py - ylo) / g).astype(np.int64), ((ph - hlo) / g).astype(np.int64)] = True
                ends = (ylo, hlo, g, _dilate_bool(occ, ky, kh))
                seen = [name for name, sel in (("后", py < y0 + INNER_END_BAND_M),
                                               ("前", py > y1 - INNER_END_BAND_M)) if np.any(sel)]
    except (FloatingPointError, OverflowError, ValueError, np.linalg.LinAlgError):
        return unresolved("数值运算非有限")
    reason = "内宽 %.2f m、内长 %.2f m，%s" % (width, y1 - y0,
                                            ("%s端板已圈出" % "、".join(seen)) if seen else "两端没见到端板")
    return InnerDomain("resolved", reason, (L["a"], L["b"]), (R["a"], R["b"]), float(y_c), y0, y1, ends)


@dataclass(frozen=True)
class FillErrorPart:
    """互斥补点分区的留出检验；未估用 nan，不能混同于已知为零。"""
    kind: str                              # hole / side / end
    filled_cells: int
    filled_volume_m3: float
    error_m3: float
    reason: str
    holdout_cells: int = 0


@dataclass(frozen=True)
class VolumeDomainInfo:
    """这一次积分实际用的域与外推检验（不进 to_dict）。误差估计靠它决定扰动哪些量。"""
    kind: str                   # inner（内腔积分域）/ box（旧积分盒）
    area_m2: float              # 积分域面积（边界格按权）
    hole_m3: float              # 普通空洞补出来的体积（不含壁格）
    fill_err_m3: float          # 已估补点分量的合成指标；完整程度看 fill_parts
    lateral_known: bool         # 左右位置可信
    dx_m: float
    dy_m: float
    pose_sigma_m: float         # 成功估计的中心高差条件标准误；不显著但未采用时仍保留
    fill_parts: Tuple[FillErrorPart, ...] = ()
    fill_check_done: bool = False


# ★ 外推检验（2026-10-02，误差估计用）。补洞补出来的体积分两类，分开检验：
#   · 两端（掠射，扫不全）：从端部往里找第一行实测占比 >= VOLUME_ERR_END_COVER 的行，把从它往里
#     VOLUME_ERR_END_BAND_M 的实测格连同更外面的实测格一起藏起来重补 —— 这一段就成了纯外推；
#     补出来的与实测的相对差，乘这一端（那一行以外）空洞补出来的体积。比实际情形悲观：
#     实际补洞时端部还有零星实测格当锚点。
#   · 中段：把中段空洞的形状往两端各挪 VOLUME_ERR_FILL_SHIFT_M 盖在实测格上重补（内插），
#     相对差（取较大的一次）乘中段空洞补出来的体积。
#   三段平方和只作为合成指标，不声称相互独立或给出置信区间。
#   ▲ 量不出"端部扫不到的那一截煤面与邻居系统性不同"（例如煤一直堆到端板）—— 那要进场 / 离场帧。
VOLUME_ERR_FILL_SHIFT_M = 1.5
VOLUME_ERR_END_COVER = 0.6
VOLUME_ERR_END_BAND_M = 0.5


def _fill_holdout(load_map: np.ndarray, box: np.ndarray, ref: np.ndarray, weight: np.ndarray,
                  hole: np.ndarray, hw: np.ndarray, cell: float, ceiling: float, h_min: float) -> float:
    """load_map 是补洞**前**的货面高度图（没测到为 nan），hw 是最终的（货高 x 权）。
    返回补洞体积的误差估计（m^3）；做不成返回 nan。"""
    measured = box & np.isfinite(load_map) & np.isfinite(ref)
    n_dom = box.sum(axis=1)
    rows = np.flatnonzero(n_dom > 0)
    if len(rows) < 10:
        return float("nan")
    cover = measured.sum(axis=1) / np.maximum(n_dom, 1)
    area = cell * cell

    def rel_error(test, hide):
        if int(np.count_nonzero(test)) < 10:
            return float("nan")
        trial = load_map.copy()
        trial[hide] = np.nan
        refill = _fill_holes(trial, domain=box)[test]
        if not np.isfinite(refill).all():
            return float("nan")
        vols = []
        for z in (load_map[test], refill):
            h = np.clip(z - ref[test], 0.0, ceiling)
            vols.append(float(np.sum(np.where(h >= h_min, h, 0.0) * weight[test])))
        return abs(vols[1] - vols[0]) / vols[0] if vols[0] > 0 else float("nan")

    parts, inner = [], [int(rows[0]), int(rows[-1])]
    kb = max(1, int(round(VOLUME_ERR_END_BAND_M / cell)))
    for side, order in ((0, rows), (1, rows[::-1])):           # 后端（y 小）、前端
        good = np.flatnonzero(cover[order] >= VOLUME_ERR_END_COVER)
        if not len(good):
            return float("nan")
        g = int(good[0])
        inner[side] = int(order[g])
        outer, band = order[:g], order[g:g + kb]
        end_hole = np.zeros_like(box)
        end_hole[outer] = hole[outer]
        v_hole = float(hw[end_hole].sum()) * area
        if v_hole <= 0:
            continue
        test = np.zeros_like(box)
        test[band] = measured[band]
        hide = test.copy()
        hide[outer] = measured[outer]
        rel = rel_error(test, hide)
        if not math.isfinite(rel):
            return float("nan")
        parts.append(rel * v_hole)
    mid = np.zeros_like(box)
    if inner[0] <= inner[1]:
        mid[inner[0]:inner[1] + 1] = hole[inner[0]:inner[1] + 1]
    v_mid = float(hw[mid].sum()) * area
    if v_mid > 0:
        k = max(1, int(round(VOLUME_ERR_FILL_SHIFT_M / cell)))
        rels = []
        for s in (k, -k):
            shifted = np.zeros_like(mid)
            if abs(s) < mid.shape[0]:
                if s > 0:
                    shifted[s:] = mid[:-s]
                else:
                    shifted[:s] = mid[-s:]
            test = shifted & measured
            rel = rel_error(test, test)
            if math.isfinite(rel):
                rels.append(rel)
        if not rels:
            return float("nan")
        parts.append(max(rels) * v_mid)
    return math.sqrt(sum(v * v for v in parts))


def _strip_holdout(load_map: np.ndarray, box: np.ndarray, ref: np.ndarray, weight: np.ndarray,
                   target: np.ndarray, test: np.ndarray, hide: np.ndarray,
                   cell: float, ceiling: float, h_min: float) -> Tuple[float, int, str]:
    """只用真实观测验证邻近条带，货高差按待补区域面积换算为体积敏感度。

    hide 同时移除条带外侧的观测，避免模拟外推时仍借用外侧锚点。
    未观测条带内部的真实形状无法由此证明；最后 0.15m 贴壁系统差仍未覆盖。
    """
    measured = box & np.isfinite(load_map) & np.isfinite(ref)
    test = test & measured
    count = int(np.count_nonzero(test))
    if count < 10 or float(weight[test].sum()) <= 0:
        return float("nan"), count, "邻近实测条带不足10格"
    trial = load_map.copy()
    trial[hide & measured] = np.nan
    predicted = _fill_holes(trial, domain=box)[test]
    if not np.isfinite(predicted).all():
        return float("nan"), count, "留出条带无法重补"
    truth_h = np.clip(load_map[test] - ref[test], 0.0, ceiling)
    predict_h = np.clip(predicted - ref[test], 0.0, ceiling)
    truth_h = np.where(truth_h >= h_min, truth_h, 0.0)
    predict_h = np.where(predict_h >= h_min, predict_h, 0.0)
    error_h = float(np.sum(np.abs(predict_h - truth_h) * weight[test]) / weight[test].sum())
    error_v = error_h * float(weight[target].sum()) * cell * cell
    return error_v, count, "邻近条带留出；未覆盖不可见区域的系统差"


def _fill_error_parts(load_map: np.ndarray, box: np.ndarray, ref: np.ndarray, weight: np.ndarray,
                      filled: np.ndarray, hw: np.ndarray, cell: float, ceiling: float, h_min: float,
                      side_distance: Tuple[np.ndarray, np.ndarray], end_distance: Tuple[np.ndarray, np.ndarray],
                      end_region: np.ndarray, do_check: bool) -> Tuple[FillErrorPart, ...]:
    """贴壁优先、端板其次、普通空洞最后，三类覆盖全部补点格且不重叠。"""
    dl, dr = side_distance
    side_region = np.minimum(dl, dr) <= INNER_SIDE_GAP_M + cell / 2
    side = filled & side_region
    end = filled & ~side & end_region
    hole = filled & ~side & ~end
    masks = (("hole", hole), ("side", side), ("end", end))
    parts = []
    for kind, target in masks:
        count = int(np.count_nonzero(target))
        volume = float(hw[target].sum()) * cell * cell
        if not count:
            parts.append(FillErrorPart(kind, 0, 0.0, 0.0, "本类没有补点格"))
            continue
        if not do_check:
            parts.append(FillErrorPart(kind, count, volume, float("nan"), "未执行留出检验"))
            continue
        if kind == "hole":
            error = _fill_holdout(load_map, box, ref, weight, target, hw, cell, ceiling, h_min)
            parts.append(FillErrorPart(kind, count, volume, error,
                                      "普通空洞留出" if math.isfinite(error) else "普通空洞没有足够留出支持"))
            continue
        errors, counts, failures = [], 0, []
        distances = side_distance if kind == "side" else end_distance
        # 每侧独立验证；其中一侧无支持时本分量未估，不把另一侧当成整个分量。
        for index, distance in enumerate(distances):
            this = target & (distance <= distances[1 - index])
            if not np.any(this):
                continue
            if kind == "side":
                low, high = INNER_SIDE_GAP_M, INNER_SIDE_GAP_M * 2
            else:
                # 端板区内侧选同宽条带。距端点宽度取本端掩模的实际范围。
                region = end_region & box & (distance <= distances[1 - index])
                if not np.any(region):
                    failures.append("端板区域缺少空间范围")
                    continue
                low = max(cell, float(np.max(distance[region])) + cell / 2)
                high = low * 2
            test = (distance >= low) & (distance < high) & ~side_region
            hide = distance < high
            error, n_test, reason = _strip_holdout(load_map, box, ref, weight, this, test, hide,
                                                  cell, ceiling, h_min)
            counts += n_test
            if math.isfinite(error):
                errors.append(error)
            else:
                failures.append("%s侧：%s" % ("第一" if index == 0 else "第二", reason))
        error = math.sqrt(sum(e * e for e in errors)) if errors and not failures else float("nan")
        reason = "；".join(failures) if failures else "邻近条带留出；未覆盖不可见区域的系统差"
        parts.append(FillErrorPart(kind, count, volume, error, reason, counts))
    return tuple(parts)


# ★ 进场帧补两端（2026-10-02 离线写成；2026-10-24 起服务传 enter_extra，见 app/enter_volume.py、docs/92）。
#   车往里开时两端先后经过雷达附近，停稳后扫不全的前后端那时看得清（9-28 两车：
#   停稳 3 帧覆盖前端 2 m 57% / 72%、后端 54% / 54%，加上 20 帧进场帧后前端 100% / 100%、后端 99% / 84%）。
#   docs/81 拿"整帧直接对停稳帧"做过，早期帧位移估计在网格、横向区域之间差几十厘米到几米，判为不能用。
#   这里换成**相邻帧链式**：相邻两帧只差 0.1~0.5 m、视角几乎一样，逐对配准残差约 1 cm；
#   最后一帧对停稳帧配准做锚，往前逐帧累加。直接配准在近端是尖锐的，用来校核链式结果
#   （9-28 两车近端两者差 <= 4~5 cm；远端直接配准代价曲线很平、会顶到搜索边界，正是 docs/81 踩的坑）。
#   ▲ 进场帧要逐帧扣高度：宁A05922F（CH4）离停车点 3.4 m 时整车比停稳时低约 13 cm、沿车厢均匀
#     （不是俯仰，也不是只在远端的掠射偏差），越近越小，像是这条道最后几米地面有坡/起伏；
#     宁A66912A（CH3）不到 3 cm。按与停稳帧重叠的格拟合 a + b*y 扣掉。
#   ▲ 只补停稳帧**没测到**的格，不改任何实测格；贴壁、端板、内腔以外的点照样不用（只在内腔积分域里用）。
#   ▲ 只在两辆车上真实配准过。服务接入时补了端板往下 8 cm 的保护、新格两帧支持，
#     缓冲满了按位移抽稀保住早期帧（docs/92 §3）；更多同牌空车 + 磅单车次仍待现场。
ENTER_STEP_MAX_M = 0.6           # 相邻帧最大位移（9-28 两车实测 0.11~0.54 m）
ENTER_STEP_RESID_M = 0.02        # 相邻帧配准后高差中位上限
ENTER_ANCHOR_RANGE_M = 0.6       # 最后一帧离停稳位置最多这么远
ENTER_RESID_MAX_M = 0.03         # 扣完高度后与停稳帧重叠格的高差中位上限
ENTER_AGREE_M = 0.06             # 直接配准尖锐（>= ENTER_SHARP）时与链式结果之差上限
ENTER_SHARP = 1.5
# 相邻帧配准也要有清楚的最优：最优 ±0.1 m 外的最小代价至少是最优的这么多倍。煤面刮得很平、两侧壁沿车厢
# 又一样时，代价曲线是平的 —— 残差照样小、位移却是乱的（合成平煤面实测估成 +0.01~+0.17，真值 0.05~2.0），
# 只靠残差门槛拦不住。
ENTER_STEP_SHARP = 2.0
ENTER_MIN_OVERLAP = 100
ENTER_CELL_M = 0.10
ENTER_X_BAND_M = 1.4
# 2026-10-24 接入服务时补的两道保护（docs/91 §5.2 的后端高锚点）：
ENTER_END_GUARD_STEPS_M = (0.0, 0.04, 0.08)  # 端板掩模往下多查到 8 cm（逐帧扣高度实测 -4.5~+0.5 cm）
ENTER_MIN_SUPPORT = 2                        # 新格至少两帧进场帧都打到才填（只有一帧可用时退回 1）


@dataclass(frozen=True)
class EnterRegistration:
    status: str                     # resolved（有可用帧）/ unresolved
    reason: str
    # 配准、扣完高度后的进场帧（载货坐标），只补空洞用。不参与比较。
    clouds: Tuple[np.ndarray, ...] = field(default=(), compare=False, repr=False)
    shifts: Tuple[float, ...] = ()  # 每帧沿 y 挪了多少才对上停稳位置（时间顺序）
    used: int = 0


def _enter_cost(mov: np.ndarray, ref_z: np.ndarray, ref_hit: np.ndarray, grid, shifts) -> np.ndarray:
    x0, y0, nx, ny = grid
    out = np.full(len(shifts), np.inf)
    for i, s in enumerate(shifts):
        q = mov.copy()
        q[:, 1] += s
        z, hit = _height_map(q, x0, y0, nx, ny, ENTER_CELL_M)
        both = hit & ref_hit
        if int(both.sum()) >= ENTER_MIN_OVERLAP:
            # 减掉整体高差再取中位：两帧之间车会整体升降（宁A05922F 进场 3.4 m 低约 13 cm），
            # 斜坡煤面上整体高差会被读成纵向位移；高度另行逐帧扣（见 register_enter_frames）。
            d = z[both] - ref_z[both]
            out[i] = float(np.median(np.abs(d - np.median(d))))
    return out


def _enter_shift(mov, ref_z, ref_hit, grid, lo, hi) -> Tuple[float, float, float]:
    """先 4 cm 粗搜再 1 cm 细搜；返回 (位移, 残差中位, 尖锐度 = 最优 ±0.1 m 外的最小代价 / 最优代价)。"""
    coarse = np.arange(lo, hi + 1e-9, 0.04)
    cc = _enter_cost(mov, ref_z, ref_hit, grid, coarse)
    if not np.isfinite(cc).any():
        return float("nan"), float("inf"), 0.0
    s0 = float(coarse[int(np.argmin(cc))])
    fine = np.arange(max(lo, s0 - 0.04), min(hi, s0 + 0.04) + 1e-9, 0.01)
    fc = _enter_cost(mov, ref_z, ref_hit, grid, fine)
    i = int(np.argmin(fc))
    s, c = float(fine[i]), float(fc[i])
    far = np.abs(coarse - s) > 0.1
    other = cc[far & np.isfinite(cc)]
    sharp = 0.0
    if len(other):
        alternative = float(np.min(other))
        if c > 0:
            sharp = alternative / c
        elif c == 0 and alternative > 0:
            # 精确重合有唯一零谷时最可信；所有位移都为零（平面无纹理）仍没有定位证据。
            sharp = float("inf")
    return s, c, sharp


def register_enter_frames(loaded: Optional[LoadedResult], frames: Sequence[np.ndarray],
                          direction: int = 1) -> EnterRegistration:
    """进场帧（已按主货面同一个旋转对齐，时间顺序、最早在前）配到停稳位置（见上）。

    direction：这些帧相对停稳位置偏在哪一侧。+1 = 车还没开到位（进场，车头朝 +y 驶入，
    帧需要往 +y 挪才对上）；-1 = 车已经越过停稳位置继续向前（向前离场的离场帧倒序后，
    帧需要往 -y 挪）。2026-11-02 现场确认载货车测完**向前**离场。
    """
    def unresolved(reason):
        return EnterRegistration("unresolved", reason)
    if loaded is None or not frames:
        return unresolved("缺载货点云或进场帧")
    try:
        lc = np.asarray(loaded.cloud, np.float64)
        clouds = [np.asarray(f, np.float64) for f in frames]
        for c in [lc] + clouds:
            if c.ndim != 2 or c.shape[1] != 3 or not np.isfinite(c).all():
                return unresolved("点云非法")
    except (TypeError, ValueError, OverflowError):
        return unresolved("输入格式非法")
    if len(lc) < 300 or any(len(c) < 300 for c in clouds):
        return unresolved("点太少")
    cx = float(np.median(lc[:, 0]))
    lo_y = float(lc[:, 1].min()) - 1.0
    span = float(lc[:, 1].max()) - lo_y + 1.0
    if not 0 < span < 100:
        return unresolved("点云 y 范围非法")
    grid = (cx - ENTER_X_BAND_M, lo_y, int(math.ceil(2 * ENTER_X_BAND_M / ENTER_CELL_M)),
            int(math.ceil(span / ENTER_CELL_M)))

    def band(c):
        return c[np.abs(c[:, 0] - cx) < ENTER_X_BAND_M]
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            lb = band(lc)
            ref_z, ref_hit = _height_map(lb, grid[0], grid[1], grid[2], grid[3], ENTER_CELL_M)
            bands = [band(c) for c in clouds]
            # 锚：最后一帧直接对停稳帧
            sign = 1 if direction >= 0 else -1

            def search(limit):
                # 允许 0.1 m 反向余量，主方向搜到 limit；方向相反时整段镜像
                return (-0.1, limit) if sign > 0 else (-limit, 0.1)
            anchor, a_res, a_sharp = _enter_shift(bands[-1], ref_z, ref_hit, grid, *search(ENTER_ANCHOR_RANGE_M))
            if not (math.isfinite(anchor) and a_res <= ENTER_STEP_RESID_M and a_sharp >= ENTER_STEP_SHARP):
                return unresolved("最后一帧对不上停稳位置（残差 %.3f m、尖锐度 %.1f）" % (a_res, a_sharp))
            shifts = [float("nan")] * len(bands)
            shifts[-1] = anchor
            for i in range(len(bands) - 2, -1, -1):
                nz, nh = _height_map(bands[i + 1], grid[0], grid[1], grid[2], grid[3], ENTER_CELL_M)
                step, res, sharp = _enter_shift(bands[i], nz, nh, grid, *search(ENTER_STEP_MAX_M))
                if not (math.isfinite(step) and res <= ENTER_STEP_RESID_M and sharp >= ENTER_STEP_SHARP):
                    break                       # 链断了：更早的帧都不用（位移留 nan）
                shifts[i] = shifts[i + 1] + step
            ok_from = next(i for i in range(len(bands)) if math.isfinite(shifts[i]))
            used, out, kept_shifts, checked = 0, [], [], 0
            for i in range(ok_from, len(bands)):
                q = clouds[i].copy()
                q[:, 1] += shifts[i]
                z, hit = _height_map(band(q), grid[0], grid[1], grid[2], grid[3], ENTER_CELL_M)
                both = hit & ref_hit
                if int(both.sum()) < ENTER_MIN_OVERLAP:
                    continue
                # 直接配准尖锐时校核链式结果
                direct, _, sharp = _enter_shift(bands[i], ref_z, ref_hit, grid,
                                                shifts[i] - 0.5, shifts[i] + 0.5)
                if sharp >= ENTER_SHARP:
                    checked += 1
                    if abs(direct - shifts[i]) > ENTER_AGREE_M:
                        return unresolved("第 %d 帧直接配准 %+.2f 与链式 %+.2f m 对不上" % (i + 1, direct, shifts[i]))
                # 扣高度：重叠格上拟合 dz = a + b*(y - y0)
                yy = grid[1] + (np.nonzero(both)[0] + 0.5) * ENTER_CELL_M
                dz = z[both] - ref_z[both]
                A = np.column_stack([np.ones(len(yy)), yy - grid[1]])
                coef = np.linalg.lstsq(A, dz, rcond=None)[0]
                res = dz - A @ coef
                keep = np.abs(res - np.median(res)) <= max(3 * 1.4826 * np.median(np.abs(res - np.median(res))), 0.03)
                coef = np.linalg.lstsq(A[keep], dz[keep], rcond=None)[0]
                resid = float(np.median(np.abs(dz - A @ coef)))
                if resid > ENTER_RESID_MAX_M:
                    continue
                q[:, 2] -= coef[0] + coef[1] * (q[:, 1] - grid[1])
                out.append(q)
                kept_shifts.append(shifts[i])
                used += 1
    except (FloatingPointError, OverflowError, ValueError, np.linalg.LinAlgError):
        return unresolved("数值运算非有限")
    if not used:
        return unresolved("没有通过校核的进场帧")
    return EnterRegistration("resolved", "用 %d/%d 帧，位移 %+.2f~%+.2f m，直接配准校核 %d 帧" % (
        used, len(bands), min(kept_shifts), max(kept_shifts), checked), tuple(out), tuple(kept_shifts), used)


# ★ 逐车交叉校核（2026-10-01）—— 只用这一车自己两次扫描的相对高度另算一遍体积：
#     煤厚 = 空车扫描量的壁高 - 载货扫描里煤面比同侧、同位置壁顶低多少；
#     体积 = 实测格的平均煤厚（按行、列效应折算到整个内腔，见下）x 内宽 x 内长。
#   它不挪基线、不用基线逐格的底板高度、不划厢壁格、不做二维补洞，所以主算法特有的几类粗错 ——
#   横向/纵向没配上让厢壁或端板落进底板格（宁A66912A 不对齐时 +19%）、积分范围圈错、
#   两端外推偏离 —— 它不跟着错，两者一比就露出来。
#   ▲ 它**不是**完全独立的：同一套扫描、同一套壁顶识别；"空车壁高 + 载货壁顶"推出载货时
#     底板这个关系，姿态对齐用的也是它 —— 所以底板参照本身的错它查不出来。
#   ▲ 它自己就有 ±3~5% 的误差，所以门槛取 10%，结果只当参考：
#     超出参考值时只记录差异诊断，直接交付主算法体积，也不改 low_confidence。
#   ▲▲ 平均煤厚**不能**直接取"中间带实测格的平均"（2026-10-02 改）：第一版就是这么取的，
#     9-28 两对比内腔积分高 10.1% / 9.9%，分解下来是它自己的两个偏差，主算法没错：
#       · 只看中间带：煤靠壁处低（宁A05922F 左壁边 0.68~0.87 m、中间 1.1~1.15 m），偏高约 5%；
#       · 只算实测格：两端煤低（后端 1 m 平均 0.41 m）又扫不全（实测 28% / 41%），
#         实测格扎堆在煤高的中段，又偏高约 6%。
#     行均值 + 列均值 - 总均值只对均衡采样成立。2026-10-03 修成联合最小二乘，
#     行、列效应各自居中，再按整个内腔的面积积分。缺测高象限的 19.2 m^3 合成车，旧式只给 16.61。
#     联合拟合仍是假设：观测图断连、薄桥、模型残差大或留出不稳时明确不给校核体积。
#     不能用棋盘格硬分两半：若棋盘索引就是拟合行列，每半会拆成两个不连通的观测分量。
CROSS_CHECK_MAX_DIFF = 0.10
CROSS_CHECK_WALL_GAP_M = 0.30     # 两侧"载货底板直线"取在离壁这么远处（那里的空车底板高度才可靠）
CROSS_CHECK_SIDE_GAP_M = 0.15     # 煤面点离估计的内侧面这么近的不用，免得混进壁面点（壁顶外沿厚度不确定）
CROSS_CHECK_WALL_T_M = 0.05       # 侧壁厚：内宽 = 外包宽 - 2 x 壁厚
CROSS_CHECK_CELL_M = 0.10        # 0.2 m 格在斜坡煤面上取最高点偏高（脊形煤面合成场景约 +5%），与主算法常用的一档对齐
CROSS_CHECK_END_GAP_M = 0.30     # 两端各这么长的实测格不用：端板往里倾，板顶落在内长以内会被当成煤顶
CROSS_CHECK_ROW_M = CROSS_CHECK_CELL_M  # 联合模型用实际细行，避免缺测改变0.5m粗行里采到的纵向部位
CROSS_CHECK_BED_PERCENTILES = (0.2, 99.8)  # 与内腔端点同一定义，旧0.5/99.5对均匀完整底板必截短1%
CROSS_CHECK_MIN_CELLS = 3         # 一行 / 一列至少这么多实测格才算它自己的平均
# 以下是 2026-10-03 本轮验收参数，不是现场精度保证；不按两车密度调参。
CROSS_CHECK_MODEL_LIMIT = 0.03    # 留出误差与体积变动的相对上限
CROSS_CHECK_RESIDUAL_LIMIT = 0.05  # 模型失配独立门槛：本轮先按5%验收，不冒充已标定的噪声分布
CROSS_CHECK_MAX_UNSUPPORTED = 0.15  # 没有足够实测支持的行/列覆盖面积并集上限
CROSS_CHECK_MIN_GRAPH_GAP = 0.02  # 归一化观测图的第二特征值，过小代表连接只靠很少的格
CROSS_CHECK_MAX_CONDITION = 100.0
CROSS_CHECK_HOLDOUT_SEED = 20261003
CROSS_CHECK_MAX_NODES = 384       # 细行模型的明确资源上限；校核不可用不影响主积分


@dataclass(frozen=True)
class CrossCheck:
    status: str                         # done / unavailable
    reason: str
    volume_m3: float = float("nan")
    mean_depth_m: float = float("nan")
    width_m: float = float("nan")
    length_m: float = float("nan")
    cells: int = 0
    # 仅内存诊断，不加入 VolumeResult.to_dict 的既有归档合同。
    diagnostics: Dict = field(default_factory=dict, compare=False, repr=False)


def _cross_check_graph(row: np.ndarray, col: np.ndarray, nr: int, nc: int) -> dict:
    """行列为顶点、实测格为边。连通不等于稳健，另查桥边和归一化谱间隙。"""
    edges = np.unique(row * nc + col)
    adj = [[] for _ in range(nr + nc)]
    for edge in edges:
        r, c = int(edge // nc), nr + int(edge % nc)
        adj[r].append(c)
        adj[c].append(r)
    seen, low, tick, bridges = {}, {}, 0, 0

    def visit(v, parent=-1):
        nonlocal tick, bridges
        seen[v] = low[v] = tick
        tick += 1
        for u in adj[v]:
            if u == parent:
                continue
            if u not in seen:
                visit(u, v)
                low[v] = min(low[v], low[u])
                bridges += low[u] > seen[v]
            else:
                low[v] = min(low[v], seen[u])
    components = 0
    for v in range(nr + nc):
        if v not in seen:
            components += 1
            visit(v)
    degree = np.array([len(a) for a in adj], np.float64)
    gap = 0.0
    if components == 1 and len(degree) > 1:
        lap = np.eye(nr + nc)
        for v, ns in enumerate(adj):
            for u in ns:
                lap[v, u] = -1.0 / math.sqrt(degree[v] * degree[u])
        gap = float(np.linalg.eigvalsh(lap)[1])
    return dict(components=components, bridges=int(bridges), spectral_gap=gap,
                min_neighbors=int(degree.min()) if len(degree) else 0)


def _cross_check_fit(row, col, values, counts, row_len, col_w):
    """同时拟合常数、行效应、列效应；两个居中约束只消除规范自由度。"""
    nr, nc = len(row_len), len(col_w)
    n = len(values)
    design = np.zeros((n + 2, 1 + nr + nc))
    design[:n, 0] = 1.0
    design[np.arange(n), 1 + row] = 1.0
    design[np.arange(n), 1 + nr + col] = 1.0
    weight = np.sqrt(counts)
    design[:n] *= weight[:, None]
    # 约束可以同时精确满足，不改变实测格的最小二乘目标。
    design[n, 1:1 + nr] = math.sqrt(float(counts.sum())) / nr
    design[n + 1, 1 + nr:] = math.sqrt(float(counts.sum())) / nc
    rhs = np.r_[values * weight, 0.0, 0.0]
    beta, _resid, rank, singular = np.linalg.lstsq(design, rhs, rcond=None)
    condition = float(singular[0] / singular[-1]) if singular[-1] > 0 else float("inf")
    pred = beta[0] + beta[1 + row] + beta[1 + nr + col]
    mean = float(beta[0] + np.dot(beta[1:1 + nr], row_len) / sum(row_len)
                 + np.dot(beta[1 + nr:], col_w) / sum(col_w))
    return dict(beta=beta, prediction=pred, mean=mean, rank=int(rank),
                expected_rank=design.shape[1], condition=condition)


def _cross_check_joint_model(row, col, values, row_len, col_w) -> dict:
    """联合行列模型及可辨识/稳定性检查。返回不可用时，绝不把伪逆结果当校核体积。"""
    row, col = np.asarray(row, np.int64), np.asarray(col, np.int64)
    values = np.asarray(values, np.float64)
    row_len, col_w = np.asarray(row_len, np.float64), np.asarray(col_w, np.float64)
    nr, nc = len(row_len), len(col_w)
    diag = dict(model="joint-row-column-v1", threshold_source="2026-10-03验收参数，非现场精度保证",
                holdout_relative_limit=CROSS_CHECK_MODEL_LIMIT, model_residual_limit=CROSS_CHECK_RESIDUAL_LIMIT,
                max_unsupported_fraction=CROSS_CHECK_MAX_UNSUPPORTED)

    def reject(reason):
        return dict(status="unavailable", reason=reason, diagnostics=diag)
    if nr + nc > CROSS_CHECK_MAX_NODES:
        return reject("行列参数数目超过校核预算（%d > %d）" % (nr + nc, CROSS_CHECK_MAX_NODES))
    if (nr < 2 or nc < 2 or len(values) < 12 or row.shape != values.shape or col.shape != values.shape
            or not np.isfinite(values).all() or not np.isfinite(row_len).all() or not np.isfinite(col_w).all()
            or np.any(row_len <= 0) or np.any(col_w <= 0)
            or np.any((row < 0) | (row >= nr)) or np.any((col < 0) | (col >= nc))):
        return reject("联合行列输入不足或非法")
    row_ok = np.bincount(row, minlength=nr) >= CROSS_CHECK_MIN_CELLS
    col_ok = np.bincount(col, minlength=nc) >= CROSS_CHECK_MIN_CELLS
    use = row_ok[row] & col_ok[col]
    # 去掉不足支持的顶点后重新收敛，避免把只连在已删除列上的行当成已观测。
    while True:
        rr = np.bincount(row[use], minlength=nr) >= CROSS_CHECK_MIN_CELLS
        cc = np.bincount(col[use], minlength=nc) >= CROSS_CHECK_MIN_CELLS
        nxt = rr[row] & cc[col]
        if np.array_equal(nxt, use):
            row_ok, col_ok = rr, cc
            break
        use = nxt
    row_missing = 1.0 - float(sum(row_len[row_ok]) / sum(row_len))
    col_missing = 1.0 - float(sum(col_w[col_ok]) / sum(col_w))
    missing = row_missing + col_missing - row_missing * col_missing
    diag.update(unsupported_row_fraction=row_missing, unsupported_column_fraction=col_missing,
                unsupported_union_fraction=missing)
    if missing > CROSS_CHECK_MAX_UNSUPPORTED or row_ok.sum() < 2 or col_ok.sum() < 2:
        return reject("未获足够支持的行列覆盖 %.1f%% 内腔，超过 %.0f%%" % (
            missing * 100, CROSS_CHECK_MAX_UNSUPPORTED * 100))
    # 每个拟合格可能含数个纵向细格；格均值联合拟合，细格数作权重。
    # 模型检验对应实际细格尺度，不把缺测改变粗行内部采样造成的偏差当成行效应。
    ri, ci = np.flatnonzero(row_ok), np.flatnonzero(col_ok)
    rr, cc = np.searchsorted(ri, row[use]), np.searchsorted(ci, col[use])
    keys, inv = np.unique(rr * len(ci) + cc, return_inverse=True)
    counts = np.bincount(inv).astype(np.float64)
    means = np.bincount(inv, weights=values[use]) / counts
    rr, cc = keys // len(ci), keys % len(ci)
    graph = _cross_check_graph(rr, cc, len(ri), len(ci))
    diag.update(graph)
    if graph["components"] != 1:
        return reject("行列观测图分成 %d 个分量，缺测区不可辨识" % graph["components"])
    if graph["bridges"] or graph["min_neighbors"] < 2 or graph["spectral_gap"] < CROSS_CHECK_MIN_GRAPH_GAP:
        return reject("行列观测图连接过薄（桥边 %d，谱间隙 %.4f）" % (graph["bridges"], graph["spectral_gap"]))

    # 缺测少量行列时，对其效应按物理位置插值，面积仍计入整个内腔。
    rp = np.cumsum(row_len) - row_len / 2
    cp = np.cumsum(col_w) - col_w / 2
    r_map = np.array([np.interp(rp, rp[ri], eye) for eye in np.eye(len(ri))]).T
    c_map = np.array([np.interp(cp, cp[ci], eye) for eye in np.eye(len(ci))]).T
    rw, cw = row_len @ r_map, col_w @ c_map
    fit = _cross_check_fit(rr, cc, means, counts, rw, cw)
    diag.update(rank=fit["rank"], expected_rank=fit["expected_rank"], condition=fit["condition"])
    if fit["rank"] != fit["expected_rank"] or fit["condition"] > CROSS_CHECK_MAX_CONDITION:
        return reject("联合行列模型秩不足或条件数过大")
    depth = float(fit["mean"])
    scale = max(0.10, abs(depth))
    residual = means - fit["prediction"]
    rmse = math.sqrt(float(np.average(residual ** 2, weights=counts)))
    diag.update(model_rmse_m=rmse, model_relative_rmse=rmse / scale,
                observed_fit_cells=len(means), target_rows=nr, target_columns=nc)
    if rmse / scale > CROSS_CHECK_RESIDUAL_LIMIT:
        return reject("行列加性模型残差 %.1f%% 超过 %.0f%%，不能作为校核" % (
            rmse / scale * 100, CROSS_CHECK_RESIDUAL_LIMIT * 100))

    def holdout(mask):
        train = ~mask
        if int(mask.sum()) < 4:
            return None
        check = _cross_check_graph(rr[train], cc[train], len(ri), len(ci))
        if check["components"] != 1 or check["min_neighbors"] < 2 or check["bridges"]:
            return None
        model = _cross_check_fit(rr[train], cc[train], means[train], counts[train], rw, cw)
        if model["rank"] != model["expected_rank"] or model["condition"] > CROSS_CHECK_MAX_CONDITION:
            return None
        beta = model["beta"]
        pred = beta[0] + beta[1 + rr[mask]] + beta[1 + len(ri) + cc[mask]]
        err = means[mask] - pred
        return dict(cells=int(mask.sum()), relative_rmse=math.sqrt(float(np.average(err ** 2, weights=counts[mask]))) / scale,
                    relative_bias=abs(float(np.average(err, weights=counts[mask]))) / scale,
                    relative_volume_shift=abs(model["mean"] - depth) / scale)

    # 空间分散且保持连通的固定种子留出。两半硬切不能保证每个子图可辨识。
    rng = np.random.default_rng(CROSS_CHECK_HOLDOUT_SEED)
    dispersed = []
    for _ in range(3):
        trial = None
        for _attempt in range(12):
            mask = np.zeros(len(means), bool)
            mask[rng.choice(len(means), max(4, int(len(means) * 0.15)), replace=False)] = True
            trial = holdout(mask)
            if trial is not None:
                break
        if trial is not None:
            dispersed.append(trial)
    # 连续二维块留出，检查相邻格相关或局部脊形被分散留出掩盖的情形。
    blocks = []
    bh, bw = max(2, len(ri) // 3), max(2, len(ci) // 3)
    starts = ((0, 0), (0, len(ci) - bw), (len(ri) - bh, 0),
              (len(ri) - bh, len(ci) - bw), ((len(ri) - bh) // 2, (len(ci) - bw) // 2))
    for r0, c0 in dict.fromkeys(starts):
        mask = (rr >= r0) & (rr < r0 + bh) & (cc >= c0) & (cc < c0 + bw)
        trial = holdout(mask)
        if trial is not None:
            blocks.append(trial)
    diag.update(holdout_seed=CROSS_CHECK_HOLDOUT_SEED, dispersed_holdouts=dispersed, block_holdouts=blocks)
    if len(dispersed) < 3 or len(blocks) < 2:
        return reject("保持连通的留出不足（分散 %d/3，连续块 %d/2），无法检查拟合稳定性" % (
            len(dispersed), len(blocks)))
    worst = max(max(t["relative_rmse"], t["relative_bias"], t["relative_volume_shift"])
                for t in dispersed + blocks)
    diag["holdout_worst_relative"] = worst
    if worst > CROSS_CHECK_MODEL_LIMIT:
        return reject("留出检查偏差 %.1f%% 超过 %.0f%%，校核模型不稳定" % (
            worst * 100, CROSS_CHECK_MODEL_LIMIT * 100))
    return dict(status="done", reason="联合行列拟合及连通留出通过；不可见区仍依赖加性假设",
                mean_depth_m=depth, diagnostics=diag)


def cross_check_volume(baseline: "EmptyBaseline", loaded: Optional["LoadedResult"], dy: float = 0.0,
                       boxes: Optional[Tuple[Optional[dict], Optional[dict]]] = None) -> CrossCheck:
    """逐车交叉校核体积（见上）。不用横向偏移：两侧壁各比各的，载货车的壁就是它自己的参照。

    dy 用积分时的同一个值 —— 只用来把空车逐段壁高对到载货的位置。
    内长取空车底板点的纵向范围（p0.2~p99.8，与内腔端点同定义；比车厢盒端点稳，
    车厢盒的 y 帧间能摆 1 m 以上）。boxes 同 estimate_pose_registration。
    """
    def unavailable(reason, diagnostics=None):
        return CrossCheck("unavailable", reason, diagnostics=diagnostics or {})
    if loaded is None or baseline is None or baseline.n == 0:
        return unavailable("缺基线或载货点云")
    if not VolumeResult._finite(dy):
        return unavailable("纵向偏移非法")
    dy = float(dy)
    try:
        bc = np.asarray(baseline.cloud, np.float64)
        lc = np.asarray(loaded.cloud, np.float64)
        for cloud in (bc, lc):
            if cloud.ndim != 2 or cloud.shape[1] != 3 or not np.isfinite(cloud).all():
                return unavailable("点云非法")
        if len(bc) != baseline.n or not VolumeResult._finite(baseline.floor_z):
            return unavailable("基线非法")
        if baseline.bed_plane is not None:
            plane = np.asarray(baseline.bed_plane, np.float64)
            if plane.shape != (4,) or not np.isfinite(plane).all() or abs(plane[2]) < 1e-9:
                return unavailable("底板平面非法")
    except (ValueError, TypeError, OverflowError):
        return unavailable("输入格式非法")
    if len(bc) < 300 or len(lc) < 300:
        return unavailable("点太少")
    if any(float(c[:, 1].max()) - float(c[:, 1].min()) > 100.0 for c in (bc, lc)):
        return unavailable("点云 y 范围过大")

    def loaded_bed(x, y):
        return baseline.bed_z_at(np.asarray(x), np.asarray(y) - dy)
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            cb, cl = boxes if boxes is not None else (carriage_box(bc, x_lim=LATERAL_X_LIM_M),
                                                       carriage_box(lc, x_lim=LATERAL_X_LIM_M))
            if cb is None or cl is None:
                return unavailable("切不出车厢（%s）" % "、".join(
                    name for name, box in (("空车", cb), ("载货", cl)) if box is None))
            prof = _wall_profiles(bc, lc, baseline, loaded_bed, dy, cb, cl)
            if isinstance(prof, str):
                return unavailable(prof)
            eb, el, ym, y_mid = prof["eb"], prof["el"], prof["ym"], prof["y_mid"]
            yb = (prof["edges"][:-1] + prof["edges"][1:]) / 2 - dy      # 空车坐标里的段中点
            floors = {}
            for side, name, xin in (("L", "左", eb[0] + CROSS_CHECK_WALL_GAP_M),
                                    ("R", "右", eb[1] - CROSS_CHECK_WALL_GAP_M)):
                tb, tl, _xl, _nc, _xb = prof[side]
                height = tb - baseline.bed_z_at(np.full(len(yb), xin), yb)    # 空车壁高
                line = _robust_wall_line(ym, tl - height)                     # 载货时这一侧的底板
                if line is None or line["n"] < POSE_MIN_SEGMENTS or line["span"] < POSE_MIN_SPAN_M:
                    return unavailable("%s壁有效段不足（%d 段，跨 %.1f m）" % (
                        name, 0 if line is None else line["n"], 0.0 if line is None else line["span"]))
                floors[side] = line
            width = float(el[1] - el[0]) - 2 * CROSS_CHECK_WALL_T_M
            bed = bc[np.abs(bc[:, 2] - baseline.bed_z_at(bc[:, 0], bc[:, 1])) <= 0.5]
            if len(bed) < 200:
                return unavailable("空车底板点不足")
            ylo, yhi = (float(v) for v in np.percentile(bed[:, 1], CROSS_CHECK_BED_PERCENTILES))
            length = yhi - ylo
            gap = CROSS_CHECK_SIDE_GAP_M
            x_lo = float(el[0]) + CROSS_CHECK_WALL_T_M + gap
            x_hi = float(el[1]) - CROSS_CHECK_WALL_T_M - gap
            if not (width > 1.0 and length > 2.0 and x_hi - x_lo > 0.5):
                return unavailable("内腔尺寸非法（内宽 %.2f m，内长 %.2f m）" % (width, length))
            y_lo = ylo + dy
            sel = lc[(lc[:, 0] >= x_lo) & (lc[:, 0] < x_hi) & (lc[:, 1] >= y_lo) & (lc[:, 1] < yhi + dy)]
            c = CROSS_CHECK_CELL_M
            # 固定物理格线，避免裁剪边的毫米变化让煤面跳变两边落进同一格；
            # 边界格按实际交叠宽度积分，不把向外取整的部分当成内腔。
            x_grid, y_grid = math.floor(x_lo / c) * c, math.floor(y_lo / c) * c
            nx = int(math.ceil((x_hi - x_grid) / c))
            ny = int(math.ceil((y_lo + length - y_grid) / c))
            ix = np.clip(((sel[:, 0] - x_grid) / c).astype(np.int64), 0, nx - 1)
            iy = np.clip(((sel[:, 1] - y_grid) / c).astype(np.int64), 0, ny - 1)
            top = np.full(nx * ny, -np.inf)
            np.maximum.at(top, iy * nx + ix, sel[:, 2])
            hit = np.flatnonzero(np.isfinite(top))
            if len(hit) < 50:
                return unavailable("车厢内实测格不足（%d 格）" % len(hit))
            col = hit % nx
            cx = (np.maximum(x_grid + col * c, x_lo) + np.minimum(x_grid + (col + 1) * c, x_hi)) / 2
            cy = (np.maximum(y_grid + (hit // nx) * c, y_lo)
                  + np.minimum(y_grid + (hit // nx + 1) * c, y_lo + length)) / 2
            fl, fr = floors["L"], floors["R"]
            zl = fl["a"] + fl["b"] * (cy - y_mid)
            zr = fr["a"] + fr["b"] * (cy - y_mid)
            xa, xb = float(el[0]) + CROSS_CHECK_WALL_GAP_M, float(el[1]) - CROSS_CHECK_WALL_GAP_M
            floor = zl + (zr - zl) * (cx - xa) / (xb - xa)
            use = (cy >= y_lo + CROSS_CHECK_END_GAP_M) & (cy < y_lo + length - CROSS_CHECK_END_GAP_M)
            if int(use.sum()) < 50:
                return unavailable("车厢内实测格不足（%d 格）" % int(use.sum()))
            depth_cell = np.maximum(top[hit] - floor, 0.0)[use]
            col, row = col[use], (hit // nx)[use]
            mu = float(np.mean(depth_cell))
            # 联合估计行列效应，不能在缺测不均匀时把两个边际平均直接相加。
            row_len = np.minimum(y_grid + (np.arange(ny) + 1) * c, y_lo + length) - np.maximum(
                y_grid + np.arange(ny) * c, y_lo)
            col_w = np.minimum(x_grid + (np.arange(nx) + 1) * c, x_hi) - np.maximum(
                x_grid + np.arange(nx) * c, x_lo)
            col_w[0] += gap                        # 贴壁不用的两条按最近一列外推
            col_w[-1] += gap
            joint = _cross_check_joint_model(row, col, depth_cell, row_len, col_w)
            diagnostics = joint["diagnostics"]
            diagnostics["bed_endpoint_percentiles"] = list(CROSS_CHECK_BED_PERCENTILES)
            diagnostics.update(row_cell_m=c, column_cell_m=c, grid_origin_xy=[x_grid, y_grid],
                               boundary_area_weighted=True, width_m=width, length_m=length,
                               observed_fine_cells=len(depth_cell))
            diagnostics["excluded_boundary_fraction"] = 1.0 - max(0.0, width - 2 * gap) * max(
                0.0, length - 2 * CROSS_CHECK_END_GAP_M) / (width * length)
            if joint["status"] != "done":
                return unavailable(joint["reason"], diagnostics)
            depth = joint["mean_depth_m"]
    except (FloatingPointError, OverflowError, ValueError, np.linalg.LinAlgError):
        return unavailable("数值运算非有限")
    volume = max(0.0, depth) * width * length
    if not math.isfinite(volume):
        return unavailable("数值运算非有限")
    return CrossCheck("done", "联合行列煤厚 %.3f m（%d 格，实测格均值 %.3f）x 内宽 %.2f m x 内长 %.2f m；%s" % (
        depth, len(depth_cell), mu, width, length, joint["reason"]),
        volume, depth, width, length, len(depth_cell), diagnostics)


def baseline_quality(clouds: Sequence[np.ndarray], bin_m: float = 0.10,
                     x_lim: float = 1.1, min_r: float = 0.95) -> dict:
    """采空车基线之前，先确认这几帧**确实是同一个静止场景**。

    为什么必须有这道闸门（2026-09-17 实测）：

    | 采集 | 三帧沿 y 的剖面互相关 r |
    |---|---|
    | 宁A81530M **载货** | 0.986 ~ 0.993 |
    | 宁A42264Z **载货** | **1.000** |
    | 宁A81530M **空车** | 0.865 ~ 0.873 |
    | 宁A42264Z **空车** | **0.799** ~ 0.836 |

    载货那两次是真静止（逐帧几乎逐点相同）；**两次空车采集都不是** ——
    而且不是简单平移（试过把剖面平移再对齐，最好也只到 0.91~0.96）。
    后果：拿不同的空车帧建基线，同一辆车的体积差 4%
    （宁A42264Z 52.15 / 54.43 / 53.38）。

    ▲ 这个问题**不该在算体积那一端修**。试过把积分盒改由"货自己的范围"定，
      实测帧间跨度从 0.27 m(0%) 劣化到 18.12 m(45%) —— 位置更稳的盒子
      会把位置错了那一帧的前段货整块切掉，反而更糟（见 compute_volume 里的注释）。
      正确的地方是**别把这种采集存成基线**。

    做法：把每帧压成沿 y 的点数剖面（只取中央 x 带，避开厢壁与旁边的结构），
    两两算归一化互相关。全部 >= min_r 才算合格。

    返回 {"ok", "r_min", "r_all", "n", "why"}。
    clouds 少于 2 片时 ok=False —— **单帧判不了重复性**。
    """
    profs = []
    for a in (clouds or []):
        a = np.asarray(a, np.float64)
        if a.ndim != 2 or a.shape[1] < 3 or len(a) < 500:
            continue
        m = np.abs(a[:, 0]) < x_lim
        y = a[m, 1]
        if len(y) < 500:
            continue
        edges = np.arange(np.floor(y.min()), np.ceil(y.max()) + bin_m, bin_m)
        if len(edges) < 8:
            continue
        h, _ = np.histogram(y, bins=edges)
        profs.append((edges[0], h.astype(np.float64)))
    if len(profs) < 2:
        return {"ok": False, "r_min": float("nan"), "r_all": [], "n": len(profs),
                "why": "只有 %d 帧可用，判不了重复性" % len(profs)}
    # 对齐到同一个 y 起点再比
    lo = min(p0 for p0, _ in profs)
    hi = max(p0 + len(h) * bin_m for p0, h in profs)
    n = int(round((hi - lo) / bin_m)) + 1
    mat = []
    for p0, h in profs:
        v = np.zeros(n)
        k = int(round((p0 - lo) / bin_m))
        v[k:k + len(h)] = h
        mat.append(v)
    rs = []
    for i in range(len(mat)):
        for j in range(i + 1, len(mat)):
            a, b = mat[i], mat[j]
            d = float(np.linalg.norm(a) * np.linalg.norm(b))
            rs.append(float(np.dot(a, b) / d) if d > 0 else 0.0)
    r_min = min(rs) if rs else 0.0
    ok = r_min >= min_r
    return {"ok": bool(ok), "r_min": r_min, "r_all": rs, "n": len(mat),
            "why": "" if ok else
                   "帧间互相关最低只有 %.3f（要求 >= %.2f）—— "
                   "这几帧不是同一个静止场景，别拿它建基线" % (r_min, min_r)}


# ═══════════════════════════════════════════════════════════════════
# 自动挑空车基线的闸门 —— 阈值都是 2026-09-18 在 124 份现场样本上量的
# ═══════════════════════════════════════════════════════════════════
#
# 为什么需要这一套：现场车太多，"每辆车人工点一次激活"不可行，
# 基线必须能自动挑、自动更新。而失败是**静默且永久**的 ——
# 把载货点云存成空车基线，这辆车之后的体积恒为接近 0 且不报错。
# 所以判据要**保守**（宁可漏，不可错），而且要**多条联合**。
#
# ▲▲ **不要照 docs/13 §12.1 那条"z 厚度 1.3 m 干净间隙"单独用。**
#    那是 CH4 下午 8 份子集的假象。124 份全量实测：z 厚度是**连续谱**
#    （p5 0.37 / p50 0.77 / p95 2.31），**1.2~2.4 之间压着 43 份（35%）**。
#    底面离地单独用也一样，1.6~2.6 之间有 38 份（31%）。
#    两条**联合**才收得住：8 份有真值的样本上 8/8 正确，
#    而全量只有 5.6% 通过 —— 低于预期的空车比例（约 22%），
#    也就是它确实在漏，方向是对的。
#
# ★★ 主判据：**底板在整个车厢内露出的比例**。
#   车厢足迹按 0.2 m 分格，每格取最低回波，看它落不落在底板高度带里。
#   它直接对应要证的两件事：
#     · "看到的是底板" —— 最低面落在物理上可能的底板高度；
#     · "厢内没有明显货物" —— 而且**整个车厢范围内**都如此。
#
#   实测（124 份现场样本）分布是**强双峰**的：
#       0%~10%  : 111 份（90%）  载货
#      10%~90%  :   4 份（3.2%） <- 真正判不准的
#      90%~100% :   9 份（7.3%） 空车
#   8 份有真值的：空车 98%/100%，载货 6 份全部 **0%**。
#
#   ▲ 对比：先用的是"z 厚度 >= 2.2"，模糊带占 **35%** —— 差一个数量级。
#     而且厚度判据**漏半装载**：前段有煤、尾部露底时，厚度和底面
#     仍然是空车的读数。露出比例会掉到 12%~14%，剖面还能看出货堆在哪一段
#     （实测 20260914_165712：100/19/0/0/0/0/0/0）。
EMPTY_BED_EXPOSURE_MIN = 0.90
# 低面得是个**平面** —— 底板是制造出来的，煤面是糙的。
#   实测：空车 0.021 / 0.025，载货 0.042~0.250。
#   ▲ 单独用不行：9-13 那天坏数据的残差只有 0.013（比真空车还平，
#     那是一片被误切进来的地面），它靠"底板露出"拦。
EMPTY_BED_PLANE_RMS_MAX = 0.06
# 保留：厚度不再当判据，但仍随证据报出来，便于事后复核。
EMPTY_THICKNESS_MIN = 2.2
# 底面离地：车厢带内最低面相对 ground_z 的高度。
#   ▲ **必须是一个带，不能只设上界。** 只设上界会把 2026-09-13 那天
#     ROI/yaw 配错产出的坏数据放进来 —— 实测它的"底面离地 0.22 m"，
#     而自卸车底板离地 1.2~1.5 m，物理上不可能（见 docs/03 §2.6）。
#     那三份的 meta 理由还恰恰写着"空车"，单看判定链会直接中招。
#   已知真值：空车 1.26 / 1.31，载货 2.62~3.31。
EMPTY_FLOOR_BAND_M = (0.9, 1.9)
# 车厢尺寸的物理量程。宽度窗口与 carriage_box 一致；
# 长度实测 A 型 10.45~13.90、B 型 6.40~6.50，留足余量。
EMPTY_LENGTH_BAND_M = (4.0, 16.0)
EMPTY_WIDTH_BAND_M = (2.15, 2.80)
# 换基线时"还是同一个车厢吗"。内宽实测 72 份标准差 0.067 m，
# 给 0.25 m 容差（约 3.7 倍标准差）；长度帧间离散中位 0.15 m，给 1.0 m。
#   拦的是：牵引车换了挂车而车牌没变 —— 车牌对应的车厢变了，
#   旧基线作废，但**不能**把新车厢悄悄当成同一辆车的基线。
EMPTY_WIDTH_TOL_M = 0.25
EMPTY_LENGTH_TOL_M = 1.0
# 单调性：新候选不能比现有基线"更满"，口径与主判据一致（底板露出）。
#   两份真空车的露出是 98.2% 与 99.7%，差 1.5 个百分点；给 5 个百分点容差。
#   ▲ 这条才是真正的安全网：主判据的阈值只有 2 份空车真值撑着，
#     而"露得比旧基线还少就不许替换"与主判据判得准不准**无关**。
EMPTY_EXPOSURE_REGRESS = 0.05


def carriage_profile(xyz: np.ndarray, ground_z: float,
                     x_lim: float = 1.1) -> Optional[dict]:
    """量一份点云的车厢几何：厢长、内宽、底面离地、车厢带内 z 厚度。

    厚度与底面都只取**车厢带内**（`carriage_box` 切出来的 y 段）且
    `|x| < x_lim` 的点 —— 避开厢壁与旁边的结构，量的才是"厢里面"。
    切不出车厢时返回 None，**不猜**。
    """
    try:
        points = np.asarray(xyz, np.float64)
    except (ValueError, TypeError):
        return None
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 500:
        return None
    if not np.isfinite(points).all() or not math.isfinite(float(ground_z)):
        return None
    box = carriage_box(points)
    if box is None:
        return None
    band = points[(points[:, 1] >= box["y0"]) & (points[:, 1] <= box["y1"])
                  & (np.abs(points[:, 0]) < float(x_lim))]
    if len(band) < 500:
        return None
    lo, hi = (float(v) for v in np.percentile(band[:, 2], [1, 99]))
    if not all(math.isfinite(v) for v in (lo, hi)):
        return None

    # ── 底板露出比例 ──────────────────────────────────────────
    #   车厢足迹按 cell 分格，**每格取最低回波**，看它落不落在底板的
    #   物理高度带里。这是"看到的是底板、而且厢内没有明显货物"的直接证据：
    #     · 是底板 —— 最低面落在物理上可能的底板高度（自卸车 1.2~1.5 m）；
    #     · 没有货 —— 而且**整个车厢范围内**都如此。
    #
    #   ▲ 高度必须用**离地绝对值**，不能拿点云自己的最低点当参考。
    #     用自参考的话，载货车"货面最低 30 cm"也会被算成底板 ——
    #     实测那样算出来载货是 42%~50%，与空车的 62%~66% 根本分不开。
    #     改成绝对高度之后是 0% vs 98%~100%。
    cell = 0.20
    nx = int(2.0 * float(x_lim) / cell) + 2
    ix = np.floor((band[:, 0] + float(x_lim)) / cell).astype(np.int64)
    iy = np.floor((band[:, 1] - box["y0"]) / cell).astype(np.int64)
    key = iy * nx + ix
    order = np.argsort(key, kind="stable")
    sorted_key = key[order]
    heights = band[order, 2] - float(ground_z)
    starts = np.r_[0, np.flatnonzero(np.diff(sorted_key)) + 1]
    lows = np.minimum.reduceat(heights, starts)
    exposed = (lows >= EMPTY_FLOOR_BAND_M[0]) & (lows <= EMPTY_FLOOR_BAND_M[1])
    # 沿车厢分 8 段 —— **半装载会集中在一头**，总比例看不出来，剖面看得出来。
    #   实测 20260914_165712：总露出只有 11.9%，而剖面是
    #   100% / 19% / 0 / 0 / 0 / 0 / 0 / 0 —— 货堆在前段、尾部露底。
    seg_y = band[order, 1][starts]
    seg = np.clip(((seg_y - box["y0"]) / max(float(box["length"]), 1e-6) * 8).astype(np.int64), 0, 7)
    profile = [round(float(exposed[seg == i].mean()), 3) if bool((seg == i).any()) else None
               for i in range(8)]

    # ── 低面是不是一个**平面** ────────────────────────────────
    #   底板是制造出来的平面，煤面是糙的。拿最低 0.25 m 内的点拟合，看残差。
    #   实测：空车 0.021 / 0.025，载货 0.042~0.250。
    #   ▲ 单独用不行 —— 9-13 那天的坏数据残差只有 0.013（比真空车还平，
    #     因为那是一片被误切进来的地面）。它靠"底板露出"那条拦。
    low = band[band[:, 2] <= lo + 0.25]
    rms = float("nan")
    if len(low) >= 200:
        plane = fit_plane_pca(low)
        if plane.valid:
            residual = np.abs(low @ np.array([plane.a, plane.b, plane.c]) + plane.d)
            rms = float(np.sqrt(float((residual ** 2).mean())))

    return {"length_m": float(box["length"]), "width_m": float(box["width"]),
            "floor_above_ground_m": lo - float(ground_z),
            "thickness_m": hi - lo, "band_points": int(len(band)),
            "y0": float(box["y0"]), "y1": float(box["y1"]),
            "footprint_cells": int(len(lows)),
            "bed_exposure": float(exposed.mean()),
            "bed_exposure_profile": profile,
            "bed_plane_rms_m": rms}


def empty_evidence(xyz: np.ndarray, ground_z: float, *,
                   clouds: Optional[Sequence[np.ndarray]] = None,
                   baseline: Optional["EmptyBaseline"] = None,
                   degraded: bool = False, duplicate: bool = False) -> dict:
    """这一份能不能**自动**拿去当空车基线。逐条给出实测值、门槛和余量。

    给谁用：现场车太多，人工逐车点"激活"不可行。本函数把"能不能自动更新
    基线"所需的证据算齐，随测量一起上报，由**授权方**（后台）据此决定。
    源端只出证据，不自我授权 —— 激活权限在业务侧。

    七道闸门，**任何一条不过就整体不过**：

    | # | 闸门 | 拦什么 |
    |---|---|---|
    | 1 | 完整帧 | 丢包造成的假几何 |
    | 2 | 非重复测量 | `session_timeout` 强开会话的同一辆车 |
    | 3 | 车厢可切出且尺寸在量程内 | 切不出/离谱的几何 |
    | 4 | 底面离地在物理带内 | ★ 2026-09-13 那类坏数据（底面 0.22 m） |
    | 5 | 车厢带内 z 厚度够 | ★ 把载货存成空车基线 |
    | 6 | 多帧场景静止 | 车还在动（实测 r=0.799/0.865 的两次采集） |
    | 7 | 与现有基线同车厢、且不更满 | 换挂车、逐步劣化 |

    ▲ **兜底方向**：不过就**保留旧基线**，不是"用新的"。
      漏一次下次再来；错一次这辆车的体积就永远接近 0 且不报错。
    ▲ 闸门 6 需要多帧（`clouds`）。只给一帧时这条记 `skipped`，
      **整体判不通过** —— 单帧判不了重复性。
    ▲ 闸门 7 只在已有基线时才检；首次建基线记 `first_baseline`，
      仍然通过，但把"这是首份"写进理由，供审计。

    ▲▲ **阈值的地基很薄**：全项目只有 2 份空车真值（2026-09-15 下午）。
      上面那些数是拿 124 份样本的分布 + 这 2 份正样本定的，8 份有真值的
      样本上 8/8 正确，但分母就是 8。真正的安全网是闸门 7（同车厢 +
      不更满）—— 哪怕空车判错了，只要新的比旧的更满就不会替换。
      攒够正样本后要回头复核这些常数。
    """
    gates = []

    def gate(name, passed, value=None, limit=None, note=""):
        gates.append({"name": name, "passed": bool(passed), "value": value,
                      "limit": limit, "note": note})
        return bool(passed)

    ok = gate("完整帧", not degraded, note="残帧的内点/占比偏低，几何不可信")
    ok = gate("非重复测量", not duplicate,
              note="带 duplicate_of 的是同一辆车的重复记录") and ok

    profile = carriage_profile(xyz, ground_z)
    if profile is None:
        gate("车厢可切出", False, note="carriage_box 切不出车厢，不猜几何")
        return {"ok": False, "gates": gates, "profile": None,
                "reason": "切不出车厢，无法判断是否空车"}
    gate("车厢可切出", True, value=profile["band_points"], note="车厢带内点数")

    lo_l, hi_l = EMPTY_LENGTH_BAND_M
    ok = gate("厢长在量程内", lo_l <= profile["length_m"] <= hi_l,
              value=round(profile["length_m"], 3), limit=[lo_l, hi_l]) and ok
    lo_w, hi_w = EMPTY_WIDTH_BAND_M
    ok = gate("内宽在量程内", lo_w <= profile["width_m"] <= hi_w,
              value=round(profile["width_m"], 3), limit=[lo_w, hi_w]) and ok

    # ★ 核心两条：证明**看到的是底板**，而且**厢内没有明显货物**。
    exposure = profile["bed_exposure"]
    ok = gate("底板在整个车厢内露出", exposure >= EMPTY_BED_EXPOSURE_MIN,
              value=round(exposure, 4), limit=EMPTY_BED_EXPOSURE_MIN,
              note="逐格最低回波落在底板高度带的比例；剖面见 profile") and ok

    rms = profile["bed_plane_rms_m"]
    ok = gate("低面确实是个平面", math.isfinite(rms) and rms <= EMPTY_BED_PLANE_RMS_MAX,
              value=None if not math.isfinite(rms) else round(rms, 4),
              limit=EMPTY_BED_PLANE_RMS_MAX,
              note="底板是制造出来的平面，煤面是糙的") and ok

    frames = [c for c in (clouds or []) if c is not None]
    if len(frames) < 2:
        gate("多帧场景静止", False, value=len(frames), limit=2,
             note="单帧判不了重复性，必须多帧")
        ok = False
    else:
        quality = baseline_quality(frames)
        ok = gate("多帧场景静止", quality["ok"], value=round(quality["r_min"], 4),
                  limit=0.95, note=quality["why"] or "帧间互相关") and ok

    if baseline is None or not getattr(baseline, "n", 0):
        # 基线按车牌存（2026-09-23 起），调用方传入的是这块车牌的基线 —— 措辞别再写"本通道"
        gate("与现有基线一致", True, note="first_baseline：这块车牌尚无基线，这是首份")
    else:
        old = carriage_profile(np.asarray(baseline.cloud, np.float64), ground_z)
        if old is None:
            gate("与现有基线一致", True, note="旧基线切不出车厢，无法比较，按首份处理")
        else:
            d_len = abs(profile["length_m"] - old["length_m"])
            d_wid = abs(profile["width_m"] - old["width_m"])
            ok = gate("与现有基线同一车厢",
                      d_len <= EMPTY_LENGTH_TOL_M and d_wid <= EMPTY_WIDTH_TOL_M,
                      value={"d_length_m": round(d_len, 3), "d_width_m": round(d_wid, 3)},
                      limit={"length": EMPTY_LENGTH_TOL_M, "width": EMPTY_WIDTH_TOL_M},
                      note="对不上说明换了车厢，旧基线作废但不能悄悄替换") and ok
            regress = old["bed_exposure"] - profile["bed_exposure"]
            ok = gate("不比现有基线更满", regress <= EMPTY_EXPOSURE_REGRESS,
                      value=round(regress, 4), limit=EMPTY_EXPOSURE_REGRESS,
                      note="新候选底板露得更少 = 更满，基线只许越换越空") and ok

    failed = [g["name"] for g in gates if not g["passed"]]
    return {"ok": bool(ok and not failed), "gates": gates, "profile": profile,
            "reason": "全部闸门通过，可作为自动基线候选" if (ok and not failed)
                      else "未通过：" + "、".join(failed)}


def compute_volume(baseline: Optional[EmptyBaseline], loaded: Optional[LoadedResult],
                   voxel: float = 0.10, bed_z: Optional[float] = None,
                   bed_shift_max_m: float = 0.5, fill_holes: bool = True,
                   floor_band_m: float = 0.5,
                   dy: Optional[float] = None,
                   loaded_extra: Optional[Sequence[np.ndarray]] = None,
                   bed_evidence: Optional[BedReferenceEvidence] = None,
                   bed_scope: Optional[BedReferenceScope] = None,
                   volume_context: Optional[VolumeContext] = None,
                   volume_scope: Optional[BedReferenceScope] = None,
                   dx: Union[None, float, LateralDecision] = None,
                   pose: Union[None, bool, PoseDecision] = None,
                   yaw: Union[None, float, YawDecision] = None,
                   inner: Union[None, bool, InnerDomain] = None,
                   surface_stat: str = "max",
                   fill_check: bool = True,
                   enter_extra: Optional[Sequence[np.ndarray]] = None,
                   fill_bound: bool = False,
                   shared_baseline: bool = False,
                   *,
                   verified_floor_box: bool = False
                   ) -> VolumeResult:
    """计算柱状积分估计；任何中间数值溢出均拒绝，不转成零货高。

    fill_check：做不做外推检验（见 _fill_holdout，误差估计用）；候选扫描与扰动重算传 False 省时间。
    enter_extra：配好位置的进场帧（register_enter_frames 的 clouds），只补停稳帧没测到的格；默认不用。
    fill_bound：补洞后把未测格夹进视线上限与坡度包络（见 app/occlusion_fill.py）；默认不用，服务开。
    verified_floor_box：仅在调用方已核实基线盒的 XY 边界就是完整底板边界时传 True。
    仅用于完整底板的回退积分路径，普通 ROI 和采样范围不能用作已核实的边界。"""
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            return _compute_volume_impl(baseline, loaded, voxel, bed_z,
                                        bed_shift_max_m, fill_holes, floor_band_m,
                                        dy, loaded_extra, bed_evidence, bed_scope, volume_context, volume_scope,
                                        dx, pose, yaw, inner, surface_stat, fill_check, enter_extra,
                                        fill_bound, shared_baseline, verified_floor_box=verified_floor_box)
    except (FloatingPointError, OverflowError):
        return VolumeResult(voxel_size=voxel, message=R_VOLUME_SKIPPED,
                            text=R_VOLUME_SKIPPED + "（中间数值运算非有限）")


def _compute_volume_impl(baseline: Optional[EmptyBaseline], loaded: Optional[LoadedResult],
                   voxel: float = 0.10, bed_z: Optional[float] = None,
                   bed_shift_max_m: float = 0.5, fill_holes: bool = True,
                   floor_band_m: float = 0.5,
                   dy: Optional[float] = None,
                   loaded_extra: Optional[Sequence[np.ndarray]] = None,
                   bed_evidence: Optional[BedReferenceEvidence] = None,
                   bed_scope: Optional[BedReferenceScope] = None,
                   volume_context: Optional[VolumeContext] = None,
                   volume_scope: Optional[BedReferenceScope] = None,
                   dx: Union[None, float, LateralDecision] = None,
                   pose: Union[None, bool, PoseDecision] = None,
                   yaw: Union[None, float, YawDecision] = None,
                   inner: Union[None, bool, InnerDomain] = None,
                   surface_stat: str = "max",
                   fill_check: bool = True,
                   enter_extra: Optional[Sequence[np.ndarray]] = None,
                   fill_bound: bool = False,
                   shared_baseline: bool = False,
                   *,
                   verified_floor_box: bool = False
                   ) -> VolumeResult:
    """
    载货体积 = Σ 单元面积 ×（装货后的顶面高度 - 空车时的底板高度）。

    ── 为什么不是"数占据体素" ────────────────────────────────────
    原先的实现是：把载货点云和基线点云都体素化，数"基线里没有、装货后有"
    的体素个数 × 体素体积。这个做法对**实心占据栅格**（CT、多视角重建）是对的，
    但对**单个俯视激光雷达**是错的 —— 雷达只能看到货堆的**表面**，
    货堆顶面和底板之间的空腔里一个点都没有，那些体素永远不会被"占据"。
    于是算出来的是一层壳的体积（≈ 表面积 × 体素边长），而不是货的体积。

    实测（2.2 × 11.0 × 1.4 m 的货堆，真值 33.9 m^3）：
        旧实现  5.16 m^3  = 真值的 15%
    体素边长越小，这个偏差还越大（壳更薄），完全不可用。

    ── 现在的做法：柱状积分（俯视激光雷达的标准解法）────────────
    对每个 cell×cell 的 (x,y) 单元：
        z_load = 装货后该单元的最高点（货堆顶，看不到的下面默认是实心的）
        z_base = 空车基线里该单元的最高点（车厢底板；基线没覆盖到的格子
                 按**底板平面**逐格求值，而不是拿常数 floor_z 糊 —— 底板是斜的）
        h = max(0, z_load - z_base)              ← 负的说明比空车还低，按 0 计
        V += cell^2 × h
    这正是"空车基线"存在的意义：它给出每个单元的底板高度，
    所以底板不平、有凹槽、有工装的车厢也能算对。

    ▲ 前提假设：货堆是**实心**的（顶面以下到底板都是货）。
       散料、码垛、集装箱都成立；空心的笼车/框架车会高估。
       这是俯视单雷达的固有局限，要更准只能加侧向雷达或称重。

    TODO(现场)：用一车已知体积的货标定一次，核对 cell 取值（默认 0.10 m）。
    """
    vr = VolumeResult(voxel_size=voxel)

    def reject(detail: str) -> VolumeResult:
        vr.message = R_VOLUME_SKIPPED
        vr.text = R_VOLUME_SKIPPED + "（" + detail + "）"
        return vr

    if baseline is None or baseline.n == 0:
        vr.message = R_NO_BASELINE
        vr.text = R_NO_BASELINE
        return vr
    # ★ 走到这里 baseline 一定是**非空**的（上面那条已经拦掉没基线的情况），
    #   所以 loaded=None 只可能是**载货四面拟合没成功**，不是没有基线。
    #
    #   2026-09-23 实测：仿真载货车拟合失败时，这里报的是「无空车基线」——
    #   而 data/in/CH2.baseline.npz 明明刚标定好、186 193 点。
    #   现场照这句话会去**重新标定一份根本没问题的基线**，
    #   而真正该看的是"这一帧为什么拟不出货面"（点太少 / 掠射角太大 / ROI 切了）。
    #   原注释说"调用方没有基线时才传 None"—— 那条路径其实在上面就返回了，
    #   这里只剩拟合失败这一种可能。
    if loaded is None:
        return reject("载货拟合未成功，取不到货面（基线是好的，不用重标）")
    if not VolumeResult._finite(voxel) or voxel <= 0:
        return reject("体素尺寸非法")
    cell = float(voxel)
    if (not VolumeResult._finite(floor_band_m) or floor_band_m <= 0 or
            not VolumeResult._finite(bed_shift_max_m) or bed_shift_max_m < 0 or
            bed_z is not None and not VolumeResult._finite(bed_z) or
            dy is not None and not VolumeResult._finite(dy)):
        return reject("底板参数或纵向偏移非法")
    if not (dx is None or VolumeResult._finite(dx.dx_m if isinstance(dx, LateralDecision) else dx)):
        return reject("横向偏移非法")
    if not (pose is None or isinstance(pose, (bool, PoseDecision))):
        return reject("姿态对齐参数非法")
    if not (yaw is None or isinstance(yaw, YawDecision) or (VolumeResult._finite(yaw) and float(yaw) == 0.0)):
        return reject("偏航参数非法（只接受自动估计、现成估计或 0 关闭）")
    if not (inner is None or inner is False or isinstance(inner, InnerDomain)):
        return reject("内腔参数非法（只接受自动估计、现成估计或 False 关闭）")
    if surface_stat not in SURFACE_STATS:
        return reject("高度图取值方式非法（只接受 %s）" % " / ".join(SURFACE_STATS))
    try:
        bounds = np.asarray(baseline.box, np.float64)
        lcb = np.asarray(loaded.cloud, np.float64)
        base_cloud = np.asarray(baseline.cloud, np.float64)
    except (TypeError, ValueError, OverflowError):
        return reject("基线或载货坐标格式非法")
    if (bounds.shape != (6,) or not np.isfinite(bounds).all() or
            not VolumeResult._finite(baseline.floor_z)):
        return reject("基线包围盒或底板高度非法")
    x0, x1, y0, y1, z0, z1 = bounds
    if not (x1 > x0 and y1 > y0 and z1 >= z0):
        return reject("基线包围盒非法")
    for cloud in (base_cloud, lcb):
        if cloud.ndim != 2 or cloud.shape[1] != 3 or not np.isfinite(cloud).all():
            return reject("点云必须为有限的三维坐标")
    if len(base_cloud) != baseline.n:
        return reject("基线点数与云图不符")
    if len(lcb) < 50:
        return reject("点云过少")
    if baseline.bed_plane is not None:
        try:
            plane = np.asarray(baseline.bed_plane, np.float64)
        except (TypeError, ValueError, OverflowError):
            return reject("基线底板平面非法")
        if plane.shape != (4,) or not np.isfinite(plane).all() or abs(plane[2]) < 1e-9:
            return reject("基线底板平面非法")
    parts = [lcb]
    if loaded_extra is not None:
        for extra in loaded_extra:
            if extra is None:
                continue
            try:
                part = np.asarray(extra, np.float64)
            except (TypeError, ValueError, OverflowError):
                return reject("额外货面坐标格式非法")
            if part.ndim != 2 or part.shape[1] != 3 or not np.isfinite(part).all():
                return reject("额外货面必须为有限三维坐标")
            if len(part):
                parts.append(part)
    enter_parts = []
    for extra in (enter_extra or ()):
        try:
            part = np.asarray(extra, np.float64)
        except (TypeError, ValueError, OverflowError):
            return reject("进场帧坐标格式非法")
        if part.ndim != 2 or part.shape[1] != 3 or not np.isfinite(part).all():
            return reject("进场帧必须为有限三维坐标")
        if len(part):
            enter_parts.append(part)
    dy_note = ""
    registration_ok = True
    if dy is None:
        # 自动配准自身会按 0.25 m 分桶，须在分配前独立限制原云范围。
        for cloud in (base_cloud, lcb):
            span = float(cloud[:, 1].max()) - float(cloud[:, 1].min())
            if not math.isfinite(span) or span / .25 > 4_000_000:
                return reject("自动配准范围非法或过大")
        if shared_baseline:
            registration = estimate_registration(baseline, loaded)
            registration_ok = registration.status == "resolved"
            dy, dy_note = (registration.dy_m if registration_ok else 0.), registration.reason
        else:
            dy, dy_note = estimate_dy(baseline, loaded)
    if not VolumeResult._finite(dy):
        return reject("纵向配准未得到有限偏移")
    dy = float(dy)

    decision = resolve_bed_shift(baseline, parts, bed_z, bed_evidence, bed_scope, dy, bed_shift_max_m)
    vr.bed_shift_applied_m = decision.shift_m
    vr.bed_reference_status = decision.status
    vr.bed_reference_source = decision.source
    vr.bed_reference_id = decision.reference_id
    vr.bed_reference_uncertainty_m = decision.uncertainty_m
    vr.bed_reference_reason = decision.reason
    error = context_error(volume_context, volume_scope, baseline, parts, dy,
                          decision.uncertainty_m if decision.status == "applied" else 0.)
    if error is None and decision.status == "applied" and bed_scope != volume_scope:
        error = "独立底板参考与体积条件不属于同一测量身份"
    if error is None and not shared_baseline:
        vr.declared_model_used = True
        vr.occlusion = OcclusionFillInfo("skipped", "正式模型路径未使用遮挡上限补洞")
        try:
            values = integrate_context(volume_context, baseline, parts, dy, decision, cell, fill_holes)
        except ValueError:
            return reject("估计网格非法或过大")
        for key, value in values.items():
            setattr(vr, key, value)
        if "volume_m3" not in values:
            return reject(vr.range_reason)
        if not math.isfinite(vr.volume_m3):
            return reject("积分结果非有限")
        vr.ok, vr.message = True, R_VOLUME_DONE
        vr.text = ("载货体积 %.3f m^3（固定支持 %.3fm，插值网格 %.2fm；"
                   "正式模型路径：进/离场帧和遮挡上限补洞未参与计算）" % (vr.volume_m3, vr.support_cell_m, cell))
        return vr
    vr.range_reason = error
    vr.range_status = "missing" if volume_context is None else "rejected"

    # ★ 横向配准（见 estimate_lateral_registration）只在这条无声明条件的估计路径上做：
    #   · 声明模型路径（上面的 integrate_context）按**原基线摘要**核定条件与绑定，
    #     在那里挪基线会破坏绑定。TODO(现场)：启用声明模型前，volume_provider 合同
    #     要先把横向偏移纳进去。
    #   · 已采纳的独立底板参考同理：它在原基线坐标里核定位置与高度，挪了基线就对不上
    #     —— 自动估的不挪并写明，显式给的直接拒绝。现场没有这类证据（恒为 None）。
    #   与 dy 一样挪的是基线（点云、车厢盒、底板平面），不动载货点云。
    #   dx=None 自动估；传 LateralDecision 用现成的估计（compute_volume_auto 各档共用）；
    #   传数值直接用；传 0.0 不配准。
    # lateral_known：左右位置可信（自动估出来了，或调用方显式给了偏移）—— 内腔积分域要靠它摆位置。
    lateral_note, lateral_boxes, lateral_known = "", None, True
    if dx is None or isinstance(dx, LateralDecision):
        lateral = dx if dx is not None else estimate_lateral_registration(baseline, loaded)
        lateral_boxes = lateral.boxes
        dx = lateral.dx_m if lateral.status == "resolved" else 0.0
        if lateral.status != "resolved":
            lateral_note = "（横向未对齐：%s）" % lateral.reason
            lateral_known = False
        elif decision.status == "applied":
            lateral_note = "（横向未对齐：已采纳的独立底板参考按原基线坐标核定，估出 %+.3f m）" % dx
            dx = 0.0
            lateral_known = False
        else:
            lateral_note = "（横向对齐 %+.3f m：%s）" % (dx, lateral.reason)
    elif dx and decision.status == "applied":
        return reject("横向偏移不能与已采纳的独立底板参考同时使用")
    base0 = baseline
    dx = float(dx)
    if dx:
        baseline = _shift_baseline_x(baseline, dx)
        x0, x1 = float(baseline.box[0]), float(baseline.box[1])

    # 偏航与姿态共用同一份逐段壁顶（按这里最终的 dx / dy 算），要用时才算、只算一次。
    shared = {}

    def wall_profiles():
        if "p" not in shared:
            shared["p"] = shared_wall_profiles(base0, loaded, dx, dy, lateral_boxes)
        return shared["p"]

    # ★ 偏航配准（见 estimate_yaw_registration）：横向挪完再绕车厢中部转。
    #   左右位置自动估不出来时不单独补偏航（那时的原因往往也会把偏航带偏）；
    #   已采纳的独立底板参考同横向配准，按原基线坐标核定，不转。
    #   yaw=None 自动估；传 YawDecision 用现成的估计（偏移对不上就重估）；传 0 不做。
    yaw_note, yaw_k, yaw_piv = "", 0.0, 0.0
    if yaw is None or isinstance(yaw, YawDecision):
        if decision.status == "applied":
            yaw_note = "（偏航未对齐：已采纳的独立底板参考按原基线坐标核定）"
        elif lateral_note.startswith("（横向未对齐"):
            yaw_note = "（偏航未对齐：左右位置没对上，不单独补偏航）"
        else:
            if not (isinstance(yaw, YawDecision) and abs(yaw.dx_m - dx) <= 1e-9 and abs(yaw.dy_m - dy) <= 1e-9):
                yaw = estimate_yaw_registration(base0, loaded, dx, dy, lateral_boxes, wall_profiles())
            if yaw.status == "resolved":
                baseline = _shear_baseline_x(baseline, yaw.slope, yaw.y_mid - dy)
                x0, x1 = float(baseline.box[0]), float(baseline.box[1])
                yaw_k, yaw_piv = float(yaw.slope), float(yaw.y_mid)
                yaw_note = "（偏航对齐 %+.2f°：%s）" % (math.degrees(math.atan(yaw.slope)), yaw.reason)
            else:
                yaw_note = "（偏航未对齐：%s）" % yaw.reason

    # ★ 姿态对齐（见 estimate_pose_registration）：同样只在这条路径上做、理由同上。
    #   已采纳的独立底板参考本身就是这一车实测的底板高度，再叠壁顶高差会重复修正。
    #   pose=None 自动估；传 PoseDecision 用现成的估计（估计时的偏移与这里对不上就重估）；
    #   传 False 不做。
    pose_note, pose_used = "", None
    if pose is not False:
        if decision.status == "applied":
            pose_note = "（姿态未对齐：已采纳的独立底板参考）"
        else:
            if not (isinstance(pose, PoseDecision) and abs(pose.dx_m - dx) <= 1e-9
                    and abs(pose.dy_m - dy) <= 1e-9):
                pose = estimate_pose_registration(base0, loaded, dx, dy, lateral_boxes, wall_profiles())
            if pose.status == "resolved":
                pose_used = pose
                pose_note = "（姿态对齐：%s）" % pose.reason
            else:
                pose_note = "（姿态未对齐：%s）" % pose.reason

    if shared_baseline:
        pose_resolved = pose_used is not None or pose is False or isinstance(pose, PoseDecision) and pose.estimated
        yaw_resolved = isinstance(yaw, (float, int)) or isinstance(yaw, YawDecision) and yaw.status == "resolved"
        resolved = registration_ok and lateral_known and yaw_resolved and pose_resolved
        vr.baseline_registration = dict(policy="ch4-shared-v1",
            registrationStatus="resolved" if resolved else "baseline_fallback",
            registrationReason="；".join(filter(None, (dy_note, lateral_note, yaw_note, pose_note))),
            dxM=dx, dyM=dy, yawSlope=yaw_k, yawPivotYM=yaw_piv,
            pose=None if pose_used is None else dict(left=list(pose_used.left), right=list(pose_used.right),
                xLeft=pose_used.x_left, xRight=pose_used.x_right, yMid=pose_used.y_mid))
        if error is None:
            # 声明条件绑定原CH4基线：将货面逆变换到原横向/姿态坐标，不挪原基线。
            aligned = []
            for part in parts:
                cloud = part.copy()
                if pose_used is not None:
                    cloud[:, 2] -= pose_used.at(part[:, 0], part[:, 1])
                cloud[:, 0] -= dx + yaw_k * (cloud[:, 1] - yaw_piv)
                aligned.append(cloud)
            from .bed_reference import surface_digest
            context = replace(volume_context, surface_sha256=surface_digest(aligned))
            values = integrate_context(context, base0, aligned, dy, decision, cell, fill_holes)
            for key, value in values.items():
                setattr(vr, key, value)
            if "volume_m3" not in values:
                return reject(vr.range_reason)
            vr.declared_model_used, vr.ok, vr.message = True, True, R_VOLUME_DONE
            vr.occlusion = OcclusionFillInfo("skipped", "正式模型路径未使用遮挡上限补洞")
            vr.text = "载货体积 %.3f m^3（CH4共享基线几何配准；进/离场帧和补洞未参与计算）" % vr.volume_m3
            return vr

    # ★ 内腔积分域（见 estimate_inner_domain）：只在补洞、左右位置可信、没采纳独立底板参考时用；
    #   用不了就退回下面的旧积分盒（= 旧行为）。inner=None 自动估；传 InnerDomain 用现成的；传 False 不用。
    inner_dom, inner_note = None, ""
    if fill_holes and inner is not False and decision.status != "applied":
        if not lateral_known:
            inner_note = "（内腔未用：左右位置没对上，退回旧积分盒）"
        else:
            if not isinstance(inner, InnerDomain):
                inner = estimate_inner_domain(base0, lateral_boxes[0] if lateral_boxes else None)
            if inner.status == "resolved":
                inner_dom = inner
            else:
                inner_note = "（内腔未用：%s，退回旧积分盒）" % inner.reason

    # ★ 网格要同时罩住**挪过 dy 的基线**和**载货点云**。
    #   只按基线盒划格的话，配准把基线往 +y 挪之后，载货车尾部就落到格子外
    #   被整段丢掉 —— 合成场景实测正好丢 1 m / 10 m = 10%
    #   （tests/test_empty_audit.py::TestVolumeRegistration
    #     ::test_registration_fixes_a_misaligned_baseline 抓到的）。
    #   真实基线盒取的是整帧包围盒，够大，所以现场数据没暴露这个问题。
    if len(loaded.cloud):
        lcb = np.asarray(loaded.cloud, np.float64)
        x0 = min(x0, float(lcb[:, 0].min()))
        x1 = max(x1, float(lcb[:, 0].max()))
        y0 = min(y0, float(lcb[:, 1].min()))
        y1 = max(y1, float(lcb[:, 1].max()))
    if dy:
        y0 = min(y0, baseline.box[2] + dy)
        y1 = max(y1, baseline.box[3] + dy)

    width, length = (float(x1) - float(x0)) / cell, (float(y1) - float(y0)) / cell
    if not all(math.isfinite(value) and 0 < value <= 4_000_000 for value in (width, length)):
        return reject("网格范围非法或过大")
    nx = max(1, int(math.ceil(width)))
    ny = max(1, int(math.ceil(length)))
    if nx * ny > 4_000_000:                    # 约 4M 单元，防止配置写错把内存吃光
        vr.message = R_VOLUME_SKIPPED
        vr.text = R_VOLUME_SKIPPED + f"（网格 {nx}×{ny} 过大，请增大 voxel）"
        return vr

    # ★ 纵向配准（dy）—— 三种用法，注释曾自相矛盾，2026-09-23 统一口径：
    #     dy=None（默认）-> 自动按车厢后沿估（estimate_registration：厢壁
    #                       是唯一可比特征，绝对门槛找后沿；估不出/超上限
    #                       则按 unresolved 处理，见上方实现）；
    #     给具体数值 -> 直接用它（离线工具 cli/volume_calc.py 会打印它算的
    #                       dy，人或工具给值走这条）；
    #     给 0.0 -> 不配准（基线与载货原位相减）。
    #
    #   空车与载货是两次停车，纵向位置不一样：2026-09-15 实测 CH4 21 辆车
    #   车厢后沿 y0 的 std 0.864 m、极差 4.00 m。基线没对齐就整体错位，
    #   车厢中段的货高会被减到厢底以外去。
    #
    #   dy 的语义：**把基线往 +y 挪多少**才能对上这一车（= 载货车厢后沿 - 基线车厢后沿）。
    #   实现上挪的是基线（参考面 + 车厢盒），不是载货点云 ——
    #   挪载货点云会把它移出积分区，2026-09-16 用错方法测过一次，
    #   得出"配准只值 8% 而且方向是反的"的假结论。
    #
    #   ▲ **自动估为何只用"车厢后沿法"**：三种估法在同一对数据上给出三个答案，
    #     连符号都不一致 —— 车厢后沿法（绝对门槛）+1.00 / +1.75，后沿法（相对
    #     门槛）+2.03 / +2.07，足迹 IoU 最大化 -0.90 / -0.90（符号都反，被
    #     驾驶室/门架拽偏，已否决）。后沿法与独立算的 cli/volume_calc.py
    #     吻合（+1.00），两辆真值车最终密度 0.99 / 0.94 落回 1.0 以下。
    #     配准总共只值 2%~3% 的体积，估不准比不配更危险 —— 所以自动估
    #     带硬上限（超 ±2.5 m 不配，见 estimate_registration）。
    if dy:
        bl_cloud = np.asarray(baseline.cloud, np.float64).copy()
        bl_cloud[:, 1] += dy
    else:
        bl_cloud = np.asarray(baseline.cloud, np.float64)
    base_z, base_hit = _relative_height_map(
        bl_cloud, baseline, dy, x0, y0, nx, ny, cell, surface_stat)
    # ★ 货面高度图可以用**多帧**填（`loaded_extra`），但**盒子、配准、
    #   车厢范围一律只用主帧**。两件事必须分开：
    #     · 多帧填面：把两端掠射区那些"这一帧没打到、下一帧打到了"的格子补上。
    #       2026-09-17 实测两辆真值车：覆盖 78.4%->91.5% / 79.8%->85.1%，
    #       "换一帧就换一个数"的敏感性从 **20% 掉到 0%**，
    #       两车密度一致性从差 4.8% 收到 1.3%。
    #     · ▲ **绝不能把多帧点云揉成一片再定盒子** —— 合并会把车厢盒撑长
    #       （docs/08 §17.12：1 帧 11.5 m -> 3 帧 14.35 m），积分区域跟着变大。
    #       实测预合并算出 69.11 m3，而正确值约 55~56。
    #   所以下面只有 `_height_map` 吃多帧，`bl_cloud` / `loaded.cloud` 那几处不动。
    n_frames = len(parts)
    lc_surf = np.vstack(parts) if n_frames > 1 else lcb
    load_z, load_hit = _relative_height_map(
        lc_surf, baseline, dy, x0, y0, nx, ny, cell, surface_stat, pose_used)
    raw_hit = load_hit          # 剔除贴壁/端板点之前的命中：分得清"没打到"和"打到的是壁"

    # ★ 基线没覆盖到的单元：按**底板平面逐格求值**，不要拿一个常数糊过去。
    #
    #   "车厢底板本来就是平的"这句话只对了一半 —— 它是**平面**没错，但它是**斜的**：
    #   空载自卸车向车头翘 3.8~4.1 度，沿 y 落差 0.68~0.84 m（2026-09-15 实测）。
    #   拿常数 floor_z（平面在原点的截距）兜底，在车厢两端最大差 0.4 m，
    #   而基线的覆盖率实测只有 79%（多帧合并 85~92%，docs/08 §10.11）——
    #   也就是 8%~21% 的格子在吃这个误差。24 m^2 的车厢上，0.2 m 的平均偏差
    #   就是 4.8 m^3，压在 50 m^3 的载货上是 10%。
    #   老基线没存平面时 bed_z_at() 自动退回常数，行为与从前一模一样。
    gx = x0 + (np.arange(nx) + 0.5) * cell
    gy = y0 + (np.arange(ny) + 0.5) * cell
    gxx, gyy = np.meshgrid(gx, gy)
    plane_ref = baseline.bed_z_at(gxx, gyy - float(dy))     # 基线一起挪
    if not np.isfinite(plane_ref).all():
        return reject("基线底板参考面包含非有限值")

    # 薄煤也能被拟成平面；高度接近基线不等于确认裸露底板。
    # 缺独立证据保持原基线，身份/实际输入绑定和有效时间检查见bed_reference。
    shift = decision.shift_m
    vr.bed_shift_applied_m = shift
    vr.bed_reference_status = decision.status
    vr.bed_reference_source = decision.source
    vr.bed_reference_id = decision.reference_id
    vr.bed_reference_uncertainty_m = decision.uncertainty_m
    vr.bed_reference_reason = decision.reason
    shift_note = ("（%s，修正 %+.3f m）" % (decision.reason, shift)
                  if decision.status != "not_requested" else "")

    # ★ 参考面：基线**明显高出底板平面**的格子也换成平面值（2026-09-16）
    #
    #   基线云是整片 ROI，里面既有厢底也有**厢壁**和残留物。厢壁比底板高
    #   1.5~1.8 m，拿它当"底板"，那几格的货高会少算一大截。
    #   实测车厢盒内有 **17%~19%** 的格子属于这一类。
    #   （cli/volume_calc.py 早就这么做了 —— "512 格实测值比平面高 0.25 m 以上
    #     （厢壁/残留物），改用平面值"。这边补上，两处结论才能对得上。）
    wall = base_hit & (base_z > 0.25) if fill_holes else np.zeros_like(base_hit)
    if fill_holes:
        ref = np.where(base_hit & ~wall, base_z, 0.0) + shift
    else:
        ref = np.where(base_hit, base_z, 0.0) + shift
    # 高度图已相对逐点底板/姿态，保留实测底板残差，未知格取零残差。

    # ★ 只在**车厢盒**内积分，并把载货侧的空洞补上（2026-09-16）
    #
    #   这三件事是一个整体，拆开做都不对：
    #     · 不补洞 -> 掠射角下载货侧只覆盖 45%~60%，一半格子被当成"没有货"，
    #       实测整整低估 30%（密度算出 1.37~1.41 t/m^3，物理上不可能）；
    #     · 补洞但不限范围 -> 松弛会一路渗到车厢外面去，实测冲到 87 m^3、
    #       密度 0.56，比不补还离谱；
    #     · 车厢盒不圈准 -> 厢壁那几格的参考面是壁顶，货高少算 1.5 m 以上。
    #
    #   车厢盒怎么圈：基线里**表面落在底板平面 ±floor_band_m 内**的格子就是
    #   看得见的厢底，取它们的包围盒。实测给出 10.80 x 2.50 m / 10.70 x 2.40 m，
    #   与 cli/roi_audit.find_boxes 量的 10.75 x 2.68 一致到 0.2 m
    #   （窄的那点是厢壁 —— 货本来就只堆在两壁之间）。
    #
    #   fill_holes=False 保留不补洞路径；高度仍统一使用逐点底板相对量。
    # ★ 厢壁格是**盲区**：那一列被厢壁挡着，看不到里面有没有货。
    #   不能只把**参考面**换成平面而把载货侧照旧用实测 —— 那一格的实测高度
    #   也是**壁顶**，h 就变成整个厢壁的高度，凭空造出货来。
    #   合成场景实测：240 个壁格造出 3.06 m^3，而那辆车厢里什么都没有
    #   （tests/test_field_fixes.py::TestVolume::test_empty_vehicle_is_near_zero 抓到的）。
    #   历史估计将壁格视为空洞按邻居补；这不证明壁后隐藏货面。
    weight = None
    end_removed_hit = np.zeros_like(load_hit)
    if inner_dom is not None:
        # 内腔积分域（见 estimate_inner_domain）：载货点只用离两侧内侧面 >= INNER_SIDE_GAP_M、
        # 落在两端之间、不在端板范围里的；边界格按落在域内的面积计权。
        ly = lc_surf[:, 1]
        el, er = inner_dom.edges(ly, dx, dy, yaw_k, yaw_piv)
        ya, yb = inner_dom.y0 + dy, inner_dom.y1 + dy
        inside = (ly >= ya) & (ly <= yb) & (lc_surf[:, 0] >= el) & (lc_surf[:, 0] <= er)
        keep = inside & (lc_surf[:, 0] >= el + INNER_SIDE_GAP_M) & (lc_surf[:, 0] <= er - INNER_SIDE_GAP_M)
        n_side = int(np.count_nonzero(inside & ~keep))
        idx = np.flatnonzero(keep)
        sub = lc_surf[idx]
        floor_l = baseline.bed_z_at(sub[:, 0], sub[:, 1] - dy)
        if pose_used is not None:
            floor_l = floor_l + pose_used.at(sub[:, 0], sub[:, 1])
        endw = inner_dom.in_end_walls(sub[:, 1] - dy, sub[:, 2] - floor_l)
        n_end = int(np.count_nonzero(endw))
        if n_end:
            end_removed_hit = _height_map(sub[endw], x0, y0, nx, ny, cell, surface_stat)[1]
        keep[idx[endw]] = False
        load_z, load_hit = _relative_height_map(
            lc_surf[keep], baseline, dy, x0, y0, nx, ny, cell, surface_stat, pose_used)
        gl, gr = inner_dom.edges(gy, dx, dy, yaw_k, yaw_piv)
        weight = _strip_grid_weights(gx, gy, cell, ya, yb,
                                     lambda y: inner_dom.edges(y, dx, dy, yaw_k, yaw_piv))
        inner_note = "（内腔积分：%s，贴壁剔除 %d 点、端板剔除 %d 点）" % (inner_dom.reason, n_side, n_end)
        if enter_parts:
            # 进场帧（见 register_enter_frames）：同样的贴壁/两端/端板规则筛点，每帧按格取值、各帧取中位，
            # 只填停稳帧没打到的格（打到了但是壁的格不填）。
            maps = []
            for part in enter_parts:
                py = part[:, 1]
                pl, pr = inner_dom.edges(py, dx, dy, yaw_k, yaw_piv)
                sub = part[(py >= ya) & (py <= yb) & (part[:, 0] >= pl + INNER_SIDE_GAP_M)
                           & (part[:, 0] <= pr - INNER_SIDE_GAP_M)]
                fl = baseline.bed_z_at(sub[:, 0], sub[:, 1] - dy)
                if pose_used is not None:
                    fl = fl + pose_used.at(sub[:, 0], sub[:, 1])
                # 进场帧的高度是逐帧线性扣过的（register_enter_frames），扣的那几厘米会把贴着端板
                # 上沿的点推出掩模（07665D 后端 25 个源点里 14 个本来会命中，docs/91 §5.2）。
                # 所以往下多查 ENTER_END_GUARD_M：扣高度前后任一时刻落进端板范围都不用。
                hz = sub[:, 2] - fl
                endw = np.zeros(len(sub), bool)
                for drop in ENTER_END_GUARD_STEPS_M:
                    endw |= inner_dom.in_end_walls(sub[:, 1] - dy, hz - drop)
                sub = sub[~endw]
                maps.append(_relative_height_map(
                    sub, baseline, dy, x0, y0, nx, ny, cell, surface_stat, pose_used)[0])
            stack = np.sort(np.stack(maps), axis=0)                  # nan 排在最后
            n_ok = np.isfinite(stack).sum(axis=0)
            lo = np.take_along_axis(stack, np.maximum((n_ok - 1) // 2, 0)[None], 0)[0]
            hi = np.take_along_axis(stack, np.minimum(n_ok // 2, len(maps) - 1)[None], 0)[0]
            ez = np.full(lo.shape, np.nan)
            ez[n_ok > 0] = 0.5 * (lo[n_ok > 0] + hi[n_ok > 0])
            # 单帧支持的新格不填：一帧的残余配准误差或一块结构回波会经补洞传播到周边（同上 §5.2）。
            add = ((weight > 0) & ~raw_hit & ~wall & np.isfinite(ez)
                   & (n_ok >= min(ENTER_MIN_SUPPORT, len(maps))))
            load_z = np.where(add, ez, load_z)
            load_hit = load_hit | add
            raw_hit = raw_hit | add
            inner_note += "（进场帧补了 %d 格）" % int(np.count_nonzero(add))
    load_map = np.where(load_hit & ~wall, load_z, np.nan)
    pre_fill = load_map                         # 补洞前（外推检验用；_fill_holes 不改输入）
    box = np.zeros_like(load_hit) if fill_holes else np.ones_like(load_hit)
    if weight is not None:
        box = weight > 0
        if np.any(box):
            load_map = _fill_holes(load_map, domain=box)
    elif fill_holes:
        # ★ 车厢盒直接用**点**的范围定，不要用格子索引。
        #   格子里取的是 max —— 粗格只要沾到一点厢壁，整格就被判成"壁"，
        #   于是靠边那一圈被剔掉，盒子跟着格子一起缩。
        #   实测：0.10 m 时盒是 10.80 x 2.50 m，0.30 m 时缩到 9.90 x 2.10 m
        #   （面积 -23%），体积跟着掉 21% —— 一个本该与格子无关的量被格子牵着走。
        #   改成按点定盒之后，各档格子给出的盒基本一致。
        bc = bl_cloud                                       # 已经挪过 dy
        if len(bc) >= 50:
            bz_at = baseline.bed_z_at(bc[:, 0], bc[:, 1] - float(dy))
            bed_pt = np.abs(bc[:, 2] - bz_at) <= float(floor_band_m)
            if int(bed_pt.sum()) >= 200:
                # ★ 补过偏航的基线，车厢在网格里是平行四边形：宽度按剪切**前**的 x 量，再逐行平移。
                #   直接取剪切后点的 x 外包，会把两侧的三角形包进盒子、补洞时填成煤
                #   （面积约 斜率 x 车长^2：0.5° 的 11 m 车厢约 1 m^2；合成车厢实测多算 4%）。
                #   没补偏航时 yaw_k = 0，下面与原来逐位相同。
                # ▲ y 向的盒子曾经想改成"由货自己的范围定" —— **实测更糟，已回退**。
                #   动机是对的：3 帧空车的底板点包围盒**长度**稳到 0.07 m
                #   （10.83/10.90/10.87），**位置却滑 2.1~2.3 m**（换百分位代替
                #   min/max 也没用，不是离群点）；而货自己的 y 范围三帧完全一致。
                #   但把两者取交集之后，位置错了的那一帧会把**前段货整块切掉**：
                #   实测宁A81530M 的帧间跨度从 0.27 m(0%) 劣化到 18.12 m(45%)。
                #   结论：底板点盒虽然位置不稳，但它至少**包住**了货；
                #   拿一个更"准"但会切掉数据的盒去换它是亏的。
                #   真正的根子不在盒子，在**空车采集本身不可重复**
                #   （载货三帧互相关 r=0.986~1.000，空车只有 0.799~0.873），
                #   所以解法在采基线那一端 —— 见 `baseline_quality()`。
                # ★ 横向再按**货自己的范围**放宽一点。
                #   壁根处的底板被厢壁挡着看不见，所以"看得见的底板"比车厢内宽
                #   系统性偏窄：2026-09-16 实测宁A42264Z 底板量到 2.45 m，
                #   而货的逐行跨度是 2.54~2.62 m —— 差的那 0.1~0.17 m 是真有货的。
                #   ▲ 限幅 ±0.3 m：货的范围会被驾驶室/门架污染，不能全信。
                if verified_floor_box is True and bed_pt.all():
                    # 使用剪切前的物理边界。格心采样不缩小该域，载货点也不向外扩展。
                    bx0, bx1 = float(base0.box[0]) + dx, float(base0.box[1]) + dx
                    by0, by1 = float(base0.box[2]) + dy, float(base0.box[3]) + dy
                else:
                    # 未核实边界或有壁面、环境点的基线，仍按实测底板子集定域。
                    ux = bc[:, 0] - yaw_k * (bc[:, 1] - yaw_piv)
                    bx0, bx1 = float(ux[bed_pt].min()), float(ux[bed_pt].max())
                    by0, by1 = float(bc[bed_pt, 1].min()), float(bc[bed_pt, 1].max())
                    lc0 = np.asarray(loaded.cloud, np.float64)
                    lift = pose_used.at(lc0[:, 0], lc0[:, 1]) if pose_used is not None else 0.0
                    sel = ((lc0[:, 1] >= by0) & (lc0[:, 1] <= by1)
                           & ((lc0[:, 2] - baseline.bed_z_at(lc0[:, 0],
                                                             lc0[:, 1] - float(dy)) - lift) > 0.30))
                    if int(sel.sum()) >= 200:
                        cx = lc0[sel, 0] - yaw_k * (lc0[sel, 1] - yaw_piv)
                        bx0 = min(bx0, max(bx0 - 0.3, float(np.percentile(cx, 0.5))))
                        bx1 = max(bx1, min(bx1 + 0.3, float(np.percentile(cx, 99.5))))
                # 保留原有最小有效跨度准入；确认区域后按真实交叠面积计权，
                # 不能把边界格全算或全丢，否则非整数尺寸会随格宽、原点变化。
                i0 = max(0, int(math.ceil((by0 - y0) / cell - 0.5)))
                i1 = min(ny, int(math.floor((by1 - y0) / cell - 0.5)) + 1)
                rows = np.arange(i0, i1)
                off = yaw_k * (gy[rows] - yaw_piv)
                jj0 = np.maximum(0, np.ceil((bx0 + off - x0) / cell - 0.5).astype(np.int64))
                jj1 = np.minimum(nx, np.floor((bx1 + off - x0) / cell - 0.5).astype(np.int64) + 1)
                if i1 - i0 >= 3 and int((jj1 - jj0).min()) >= 3:
                    weight = _strip_grid_weights(
                        gx, gy, cell, by0, by1,
                        lambda y: (bx0 + yaw_k * (y - yaw_piv), bx1 + yaw_k * (y - yaw_piv)))
                    box = weight > 0
                    # 部分边界格不能借用盒外煤面或结构的高度；面积与点筛选必须同域。
                    def in_box(points):
                        u = points[:, 0] - yaw_k * (points[:, 1] - yaw_piv)
                        return ((u >= bx0) & (u <= bx1)
                                & (points[:, 1] >= by0) & (points[:, 1] <= by1))

                    base_z, base_hit = _relative_height_map(
                        bc[in_box(bc)], baseline, dy, x0, y0, nx, ny, cell, surface_stat)
                    wall = base_hit & (base_z > 0.25)
                    ref = np.where(base_hit & ~wall, base_z, 0.0) + shift
                    load_z, load_hit = _relative_height_map(
                        lc_surf[in_box(lc_surf)], baseline, dy, x0, y0, nx, ny, cell, surface_stat, pose_used)
                    raw_hit = load_hit
                    load_map = np.where(load_hit & ~wall, load_z, np.nan)
                    pre_fill = load_map
                    load_map = _fill_holes(load_map, domain=box)
        # 圈不出有效积分区就拒绝，不能把整个网格当成已确认车厢。
    # ★ 遮挡上限补洞（2026-10-24，见 app/occlusion_fill.py）：松弛补出来的未测格再夹进
    #   "视线已经穿过的高度"与堆积角坡度包络之间。视线只用停稳帧（雷达在原点），
    #   含被贴壁/端板规则剔掉的点 —— 它们照样是真回波，视线照样穿过了前面的格。
    occlusion_maps = None
    if fill_bound and fill_holes and np.any(box):
        from .occlusion_fill import bound_unknown_cells, ray_ceiling, repose_descent, shadowed_cells
        floor_grid = baseline.bed_z_at(gxx, gyy - float(dy))
        if pose_used is not None:
            floor_grid = floor_grid + pose_used.at(gxx, gyy)
        ceiling_map = ray_ceiling(lc_surf, x0, y0, nx, ny, cell, floor_grid, domain=box)
        # 只有一个回波都没有的格才可能是阴影；贴壁/端板格有回波、只是被剔掉了，照旧按松弛补。
        unknown_now = box & ~np.isfinite(pre_fill) & ~raw_hit
        shadow_map = shadowed_cells(repose_descent(pre_fill, box, cell), pre_fill, unknown_now,
                                    x0, y0, cell, floor_grid)
        load_map, lo_map, hi_map, occlusion_counts = bound_unknown_cells(
            load_map, pre_fill, box, ceiling_map, cell, shadow=shadow_map)
        occlusion_maps = (lo_map, hi_map)
    if not np.any(box):
        return reject("没有有效底板积分区域")
    if weight is None:
        weight = box.astype(np.float64)         # 旧积分盒：盒内每格整格计，下面与原来逐位相同
    if not np.isfinite(ref[box]).all():
        return reject("有效域底板参考非有限")
    measured = box & load_hit & ~wall & np.isfinite(ref)
    if not np.any(measured):
        return reject("有效积分区域没有实测货面或底板参考")
    have = np.isfinite(load_map) & np.isfinite(ref) & box

    with np.errstate(over="ignore", invalid="ignore"):
        heights = load_map[have] - ref[have]
    ceiling = float(z1) - float(baseline.floor_z) - min(0.0, shift)
    if not np.isfinite(heights).all() or not math.isfinite(ceiling):
        return reject("高度差或货高上限运算非有限")
    h = np.zeros_like(load_map)
    h[have] = heights
    # 低于底板的按 0（雷达噪声/底板缝隙），高出车厢顶的截断到基线盒上沿
    h = np.clip(h, 0.0, max(0.0, ceiling))
    # 只有明显高出底板才算货，滤掉测量噪声与底板本身的起伏
    h = np.where(h >= max(0.01, cell * 0.2), h, 0.0)

    hw = h * weight                             # 边界格按落在积分域里的面积计
    cells = int(np.count_nonzero(h > 0))
    vr.occupied_voxels = cells                 # 语义：参与积分的单元数
    vr.frames_used = n_frames
    # ★ 覆盖率与"外推占比" —— 不改体积，只是把可信度摊开写（docs/13 §16.3）。
    #   `filled` = 车厢盒内、载货侧**没有实测**（没打到，或被厢壁挡着）
    #   而靠补洞给出高度的格子。它们贡献的体积就是"外推出来的那部分"。
    #   实测两辆真值车单帧时占 10~13%，而这个比例**随装载量上升**（docs/13 §15），
    #   所以它天然就是"这一车有多可信"的指标，比一个固定的系统误差有用。
    filled = box & (~load_hit | wall) & np.isfinite(load_map) if fill_holes         else np.zeros_like(box)
    n_filled = int(np.count_nonzero(filled))
    nbox = int(np.count_nonzero(box))
    vr.coverage = (float(weight[box & load_hit & ~wall].sum()) / float(weight[box].sum())
                   if nbox else float("nan"))
    vr.filled_cells = n_filled
    total = float(hw.sum())
    vr.filled_ratio = (float(hw[filled].sum()) / total) if total > 0 else 0.0
    # 无独立条件路径只保留历史hole/wall估值分解，不再提供伪物理范围。
    # 此路径fill_holes=False还会改变历史积分域，不能拿它作同域下界。
    # 内腔积分域里，打到了、但只打到壁面/端板（点被剔掉）的格是结构盲区，归壁格不归空洞。
    hole = box & (~raw_hit) & (~wall) & np.isfinite(load_map)
    n_hole = int(np.count_nonzero(hole))
    vr.hole_cells = n_hole
    vr.hole_ratio = (float(hw[hole].sum()) / total) if total > 0 else 0.0
    # 壁格 = 补出来的里面**不是**真空洞的那一半。摆出来才看得出
    # filled 里混着一个"每车都有"的恒定项（见字段上面的说明）。
    wallf = filled & (~hole)
    vr.wall_cells = int(np.count_nonzero(wallf))
    vr.wall_ratio = (float(hw[wallf].sum()) / total) if total > 0 else 0.0
    h_min = max(0.01, cell * 0.2)
    if inner_dom is not None:
        side_distance = (gxx - gl[:, None], gr[:, None] - gxx)
        end_distance = (gyy - ya, yb - gyy)
        end_region = end_removed_hit.copy()
        if inner_dom.ends is not None:
            # 投影端板掩模的纵向范围：即使本帧完全没打到端板格，也归端板外推项。
            ey0, _, ec, occupied = inner_dom.ends
            ey = np.floor((gy - dy - ey0) / ec).astype(np.int64)
            valid_y = (ey >= 0) & (ey < occupied.shape[0])
            projected = np.zeros(ny, bool)
            projected[valid_y] = occupied.any(axis=1)[ey[valid_y]]
            end_region |= np.broadcast_to(projected[:, None], box.shape)
    else:
        # 旧盒只具备积分边界，不能把最近边界分类误读为已经识别了真实壁面。
        first_col = np.argmax(box, axis=1)
        last_col = nx - 1 - np.argmax(box[:, ::-1], axis=1)
        side_distance = (gxx - (x0 + first_col[:, None] * cell),
                         x0 + (last_col[:, None] + 1) * cell - gxx)
        domain_rows = np.flatnonzero(box.any(axis=1))
        end_distance = (gyy - (y0 + domain_rows[0] * cell),
                        y0 + (domain_rows[-1] + 1) * cell - gyy)
        end_region = wallf & (np.minimum(*end_distance) <= INNER_END_BAND_M)
    fill_parts = _fill_error_parts(pre_fill, box, ref, weight, filled, hw, cell,
                                   max(0.0, ceiling), h_min, side_distance, end_distance,
                                   end_region, fill_check)
    fill_known = [part.error_m3 for part in fill_parts if math.isfinite(part.error_m3)]
    fill_error = math.sqrt(sum(v * v for v in fill_known)) if fill_known else float("nan")
    pose_sigma = (float(pose.sigma_m) if isinstance(pose, PoseDecision)
                  and (pose.status == "resolved" or getattr(pose, "estimated", False))
                  and decision.status != "applied" else float("nan"))
    vr.domain = VolumeDomainInfo(
        "inner" if inner_dom is not None else "box", float(weight[box].sum()) * cell * cell,
        float(hw[hole].sum()) * cell * cell, fill_error,
        bool(lateral_known), float(dx), float(dy), pose_sigma,
        fill_parts, bool(fill_check))
    # 中位货高及旧box顶均不是独立上界；没有条件只能保留诊断估值。
    if n_filled:
        shift_note += "（补了 %d 个空洞格，占体积 %.0f%%）" % (
            n_filled, vr.filled_ratio * 100)
    if n_frames > 1:
        shift_note += "（货面用了 %d 帧）" % n_frames
    if dy_note:
        shift_note += "（配准：%s）" % dy_note
    if enter_parts and inner_dom is None:
        inner_note += "（进场帧未用：只在内腔积分域里补）"
    shift_note += lateral_note + yaw_note + pose_note + inner_note
    if occlusion_maps is not None:

        def bounded_volume(values):
            # 与主积分同一条高度管线：同域、同参考、同截断与低厚阈值，只换未测格的取值。
            usable = np.isfinite(values) & np.isfinite(ref) & box
            hh = np.zeros_like(values)
            with np.errstate(over="ignore", invalid="ignore"):
                hh[usable] = values[usable] - ref[usable]
            hh = np.clip(hh, 0.0, max(0.0, ceiling))
            hh = np.where(hh >= h_min, hh, 0.0)
            return float((hh * weight).sum()) * cell * cell
        vr.occlusion = OcclusionFillInfo(
            "applied", "", volume_lo_m3=bounded_volume(occlusion_maps[0]),
            volume_hi_m3=bounded_volume(occlusion_maps[1]), **occlusion_counts)
        shift_note += vr.occlusion.text()
    elif fill_bound:
        vr.occlusion = OcclusionFillInfo("skipped", "未补洞或没有积分域")
        shift_note += vr.occlusion.text()
    vr.volume_m3 = total * cell * cell
    if not math.isfinite(vr.volume_m3):
        return reject("积分结果非有限")
    vr.ok = True
    vr.message = R_VOLUME_DONE
    vr.text = (f"载货体积 {vr.volume_m3:.3f} m^3"
               f"（{cells} 个 {cell:.2f}m 单元，最大货高 {float(h.max()):.2f} m）"
               + shift_note)
    return vr


# 候选仅用于未知区估计；不得用粗化后的命中率缩小条件物理范围。
VOLUME_CELLS = (0.10, 0.15, 0.20, 0.25, 0.30)


# ★ 逐车误差诊断（2026-10-03）：保持主算法口径，分项报告敏感度与未估项。
#   · 前后（dy）：按 ±VOLUME_ERR_DY_M 重算；只有原本自动估计的偏航、姿态才重估。dy 按车厢后沿逐行（0.25 m）找，
#     与独立算法差 3~7 cm；
#   · 左右（dx）：左右位置可信时按 ±VOLUME_ERR_DX_M 重算（侧壁外包与内侧面推出的残余横移 1.9~2.2 cm）。
#     偏航、姿态沿用原估计 —— 它们对 dx 的依赖只经底板平面的横坡，2 cm 内可忽略；
#   · 高低：姿态对齐的中心高差标准误差 x 积分面积；
#   · 外推：普通空洞、贴壁条带、端板区各自留出检验，互不重叠。
#   平方和只是已估分量的合成指标：扰动幅度、条件标准误与留出残差不是同种统计量，且共用扫描数据。
#   另并报逐格取中位的体积作对照。磅单只有重量，不能单独判断最高点与中位数哪一个更接近真实体积。
#   ▲ δV 不含：配错车（车牌）、空车基线本身采得不好（baseline_quality 管）、煤面以下不实（俯视单雷达
#     的固有假设）、不可见区域煤面与邻居的系统差、逐格取值的系统差，不是总精度或置信区间。
#   ▲ 每车多算约 5 次积分（实测约 +1 s，在本车道线程上）。
VOLUME_ERR_DY_M = 0.10
VOLUME_ERR_DX_M = 0.02


@dataclass(frozen=True)
class VolumeUncertainty:
    total_m3: float
    dy_m3: float                # 各项为 nan 表示这一项没做成或不适用
    dx_m3: float
    pose_m3: float
    fill_m3: float
    median_m3: float            # 逐格取中位时的体积（对照）
    fill_parts: Tuple[FillErrorPart, ...] = ()

    @property
    def missing_parts(self) -> Tuple[str, ...]:
        names = (("前后", self.dy_m3), ("左右", self.dx_m3), ("高低", self.pose_m3))
        missing = [name for name, value in names if not math.isfinite(value)]
        if self.fill_parts:
            labels = {"hole": "空洞", "side": "贴壁", "end": "端板"}
            missing.extend(labels.get(part.kind, part.kind) for part in self.fill_parts
                           if not math.isfinite(part.error_m3))
        elif not math.isfinite(self.fill_m3):
            missing.append("外推")
        return tuple(missing)

    def text(self, volume: float) -> str:
        def part(name, v):
            return "%s ±%.2f" % (name, v) if math.isfinite(v) else "%s 未估" % name
        if math.isfinite(self.total_m3) and volume > 0:
            summary = "已估分量合成指标 ±%.2f m^3（%.1f%%）" % (self.total_m3, self.total_m3 / volume * 100)
        else:
            summary = "合成指标未估"
        if self.missing_parts:
            summary += "，缺：" + "、".join(self.missing_parts)
        if self.fill_parts:
            labels = {"hole": "空洞", "side": "贴壁", "end": "端板"}
            fill_text = "外推（%s）" % "、".join(part(labels.get(p.kind, p.kind), p.error_m3)
                                              for p in self.fill_parts)
        else:
            fill_text = part("外推", self.fill_m3)
        out = "（误差估计：%s；%s、%s；非总精度或置信区间）" % (
            summary, "、".join(part(n, v) for n, v in (("前后", self.dy_m3), ("左右", self.dx_m3),
                                                     ("高低", self.pose_m3))), fill_text)
        if math.isfinite(self.median_m3) and volume > 0:
            out += "（逐格取中位为 %.2f m^3，%+.1f%%）" % (self.median_m3, (self.median_m3 / volume - 1) * 100)
        return out


def estimate_volume_uncertainty(baseline: Optional[EmptyBaseline], loaded: Optional[LoadedResult],
                                result: VolumeResult, fill_holes: bool = True,
                                *, reestimate_yaw: Optional[bool] = None,
                                reestimate_pose: Optional[bool] = None,
                                **kw) -> Optional[VolumeUncertainty]:
    """按 result 实际用的格子与偏移扰动重算（见上）。kw 是算 result 时的其余参数；
    result 不是估计路径的成功结果时返回 None。"""
    info = result.domain
    if (info is None or result.ok is not True or not VolumeResult._finite(result.volume_m3)
            or result.volume_m3 <= 0):
        return None
    v0, cell = float(result.volume_m3), float(result.voxel_size)
    if reestimate_yaw is None:
        reestimate_yaw = kw.get("yaw") is None
    if reestimate_pose is None:
        reestimate_pose = kw.get("pose") is None
    kw = {k: v for k, v in kw.items() if k not in ("voxel", "fill_holes", "fill_check")}
    stale = [(name, estimator) for name, cls, estimator, automatic in (
        ("yaw", YawDecision, estimate_yaw_registration, reestimate_yaw),
        ("pose", PoseDecision, estimate_pose_registration, reestimate_pose))
        if not automatic and isinstance(kw.get(name), cls)
        and (abs(kw[name].dx_m - info.dx_m) > 1e-9 or abs(kw[name].dy_m - info.dy_m) > 1e-9)]
    if stale:
        # 主计算按已有契约会重估偏移标签过期的显式对象，扰动也必须先取得同一份当前几何。
        lateral = kw.get("dx")
        boxes = lateral.boxes if isinstance(lateral, LateralDecision) else None
        profiles = shared_wall_profiles(baseline, loaded, info.dx_m, info.dy_m, boxes)
        for name, estimator in stale:
            kw[name] = estimator(baseline, loaded, info.dx_m, info.dy_m, boxes, profiles)

    def run(**over):
        r = compute_volume(baseline, loaded, voxel=cell, fill_holes=fill_holes, fill_check=False,
                           **dict(kw, **over))
        return float(r.volume_m3) if r.ok is True and VolumeResult._finite(r.volume_m3) else float("nan")

    def spread(values):
        # 只有一侧扰动成功不能代表双向敏感度完整，必须明确未估。
        return max(abs(v - v0) for v in values) if values and all(math.isfinite(v) for v in values) else float("nan")

    dy_values = []
    for e in (-VOLUME_ERR_DY_M, VOLUME_ERR_DY_M):
        shifted_dy = info.dy_m + e
        over = dict(dy=shifted_dy)
        for name, cls, automatic in (("yaw", YawDecision, reestimate_yaw),
                                      ("pose", PoseDecision, reestimate_pose)):
            if automatic:
                over[name] = None
            elif isinstance(kw.get(name), cls):
                # 显式估计固定几何参数，仅更新其偏移标签，避免 compute_volume 因标签不符又自行重估。
                over[name] = replace(kw[name], dy_m=shifted_dy)
        dy_values.append(run(**over))
    d_dy = spread(dy_values)
    d_dx = float("nan")
    if info.lateral_known:
        yaw, pose = kw.get("yaw"), kw.get("pose")
        vals = []
        for e in (-VOLUME_ERR_DX_M, VOLUME_ERR_DX_M):
            over = dict(dx=info.dx_m + e, dy=info.dy_m)
            if isinstance(yaw, YawDecision):
                over["yaw"] = replace(yaw, dx_m=info.dx_m + e)
            if isinstance(pose, PoseDecision):
                over["pose"] = replace(pose, dx_m=info.dx_m + e)
            vals.append(run(**over))
        d_dx = spread(vals)
    d_pose = info.pose_sigma_m * info.area_m2 if math.isfinite(info.pose_sigma_m) else float("nan")
    d_fill = info.fill_err_m3
    parts = [v for v in (d_dy, d_dx, d_pose, d_fill) if math.isfinite(v)]
    total = math.sqrt(sum(v * v for v in parts)) if parts else float("nan")
    median = run(surface_stat="median", dy=info.dy_m)
    return VolumeUncertainty(total, d_dy, d_dx, d_pose, d_fill, median, info.fill_parts)


def compute_volume_auto(baseline: Optional[EmptyBaseline],
                        loaded: Optional[LoadedResult],
                        cells: Sequence[float] = VOLUME_CELLS,
                        spread_target: float = 0.03,
                        **kw) -> VolumeResult:
    """扫描插值候选；固定支持条件不随候选变化，缺范围仍保留有限估值。"""
    if not VolumeResult._finite(spread_target) or not 0 <= spread_target <= 1:
        return VolumeResult(message=R_VOLUME_SKIPPED,
                            text=R_VOLUME_SKIPPED + "（自适应差异门槛非法）")
    try:
        candidates = tuple(cells)
    except TypeError:
        candidates = ()
    if not candidates or any(not VolumeResult._finite(c) or c <= 0 for c in candidates):
        return VolumeResult(message=R_VOLUME_SKIPPED,
                            text=R_VOLUME_SKIPPED + "（自适应体素候选非法）")
    # ▲▲ **这个扫描目前选不出东西来** —— 2026-09-18 实测查实，两种情形一样：
    #
    #   · **没有声明范围时**（现场当前状态，没有 volume_provider 文件）：
    #     `bound_spread` 是 nan，`sp` 恒为 inf，后面每一档的
    #     `sp < best_sp` 都不成立 —— 扫完 5 档必然还是返回第一档。
    #   · **有声明范围时**：`bound_lo/hi` 由 `volume_context.support_cell_m`
    #     的**固定支持网格**算出，`voxel` 只进到 `integrate_context` 的
    #     插值填充块（`app/volume_context.py:217`）。也就是说
    #     **`bound_spread` 在构造上就与 voxel 无关**，实测两辆真值车
    #     5 档格子读数完全相同（54.8% / 60.7%）。
    #     这个不变性是**有意的**，而且被三条用例钉着
    #     （`test_auto_scans_all_candidates_without_shrinking_fixed_unknown`
    #     等）—— 防的是"靠粗化格子把未知面积做小"。
    #
    #   => 于是本函数恒等于"取第一个估值合格的档"，代价是 5 倍 CPU。
    #
    #   ▲ **没有改成扫一档就停。** 试过，省下来的 0.23 s 是**每辆车一次**、
    #     在后台收尾线程上，现场根本不在乎；而那三条用例挡的是一类真实回归。
    #     拿被测试钉住的防护换一个无关紧要的省耗，不划算。
    #   ▲ 真要让它名副其实，得换一个**随格子变化**的判据。`hole_ratio` 就是：
    #     两辆真值车 0.10 档 13.2%/9.5%，0.20 档只有 5.1%/2.3%
    #     —— 因为 0.10 m 比车厢两端的扫描线间距还细，格子整个掉进缝里。
    #     但换判据会改变对外报的体积数，要业务确认，
    #     见 `docs/33-体积可用性门槛.md` §5。
    best = None
    # ★ 先从 **kw 里接住 fill_holes（2026-09-23，低严重度批次）：原来它
    #   与下面固定的 fill_holes=True 同名相撞 —— 调用方传了只会得到
    #   TypeError，而不是"生效"。默认仍 True，与既有行为一致。
    fill = bool(kw.pop("fill_holes", True))
    auto_yaw, auto_pose = kw.get("yaw") is None, kw.get("pose") is None
    # ★ 横向配准只估一次（2026-10-01）：它要对两片点云各跑一遍 carriage_box
    #   （实测各约 30 ms），放进逐档循环就是 5 倍；而它与格子无关，
    #   所以先估好、以 LateralDecision 传给每一档，说明文字与直接调用时一致。
    if kw.get("dx") is None:
        kw["dx"] = estimate_lateral_registration(baseline, loaded)
    # 偏航与姿态同理只估一次，且共用同一份逐段壁顶（还要再量一遍两侧壁）。它们要用积分时的
    # dx / dy：dx 取上面的横向估计；dy 没给就用 compute_volume 内部同一个函数先估，结果完全一样。
    lat, dy = kw.get("dx"), kw.get("dy")
    boxes = lat.boxes if isinstance(lat, LateralDecision) else None
    if dy is None and baseline is not None and loaded is not None:
        dy = estimate_dy(baseline, loaded)[0]
    dx = (lat.dx_m if lat.status == "resolved" else 0.0) if isinstance(lat, LateralDecision) else lat
    if ((kw.get("pose") is None or kw.get("yaw") is None)
            and VolumeResult._finite(dx) and VolumeResult._finite(dy)):
        profiles = shared_wall_profiles(baseline, loaded, float(dx), float(dy), boxes)
        if kw.get("yaw") is None:
            kw["yaw"] = estimate_yaw_registration(baseline, loaded, float(dx), float(dy), boxes, profiles)
        if kw.get("pose") is None:
            kw["pose"] = estimate_pose_registration(baseline, loaded, float(dx), float(dy), boxes, profiles)
    # 内腔只与基线有关，同样估一次（复用横向估计切好的空车车厢盒）。
    if kw.get("inner") is None and fill and baseline is not None:
        kw["inner"] = estimate_inner_domain(baseline, boxes[0] if boxes else None)
    want_check = bool(kw.pop("fill_check", True))
    for c in candidates:
        vr = compute_volume(baseline, loaded, voxel=c, fill_holes=fill, fill_check=False, **kw)
        if vr.estimate_validity_errors():
            continue                    # 退化结果不算候选，继续试下一档
        sp = vr.bound_spread if not vr.range_validity_errors() else float("inf")
        best_sp = best.bound_spread if best is not None and not best.range_validity_errors() else float("inf")
        if best is None or sp < best_sp:
            best = vr
        vr.cell_auto = True
        if sp <= spread_target and vr.confidence(spread_max=spread_target)[0]:
            best = vr
            break
    if best is None:
        vr = VolumeResult(voxel_size=float(candidates[0]))
        vr.message = R_NO_BASELINE if baseline is None else R_VOLUME_SKIPPED
        return vr
    # 外推检验只给选中的这一档做（候选扫描时跳过，省掉 4 档的重补），体积与原来逐位相同。
    if want_check and best.domain is not None and not best.domain.fill_check_done:
        again = compute_volume(baseline, loaded, voxel=best.voxel_size, fill_holes=fill, **kw)
        if again.ok is True and again.volume_m3 == best.volume_m3:
            best.domain = again.domain
    # 逐车误差估计（见 estimate_volume_uncertainty）：只写说明、挂在 uncertainty 上，不改体积。
    unc = estimate_volume_uncertainty(baseline, loaded, best, fill,
                                      reestimate_yaw=auto_yaw, reestimate_pose=auto_pose, **kw)
    if unc is not None:
        best.uncertainty = unc
        best.text += unc.text(best.volume_m3)
    # 逐车交叉校核（见 cross_check_volume）：每车一次，只写说明、挂在 cross_check 上，不改体积。
    if VolumeResult._finite(best.volume_m3) and best.volume_m3 > 0 and VolumeResult._finite(dy):
        cc = cross_check_volume(baseline, loaded, float(dy), boxes)
        best.cross_check = cc
        if cc.status == "done" and cc.volume_m3 > 0:
            diff = best.volume_m3 / cc.volume_m3 - 1.0
            best.text += "（交叉校核 %.2f m^3，主算法比它 %+.1f%%）" % (cc.volume_m3, diff * 100)
            if abs(diff) > CROSS_CHECK_MAX_DIFF:
                best.text += "（诊断：与交叉校核相差 %+.1f%%，参考值 ±%.0f%%）" % (
                    diff * 100, CROSS_CHECK_MAX_DIFF * 100)
        else:
            best.text += "（交叉校核不可用：%s）" % cc.reason
    return best


# ═══════════════════════════════════════════════════════ 停稳检测

class StabilityState:
    IDLE = 0        # 无车 / 点数不足
    BASELINE = 1    # 已建立参考帧，但仍在动
    STABLE = 2      # 停稳

    CN = {0: "空闲", 1: "运动中", 2: "已停稳"}


@dataclass
class StabilityResult:
    state: int = StabilityState.IDLE
    deviation: float = 0.0
    stable: bool = False
    n_points: int = 0            # ROI 内总点数（含地面）
    elapsed: float = 0.0
    n_vehicle_points: int = 0    # 其中**高于地面**的点数 —— 有车判据用的就是它
    degraded: bool = False       # 本帧残缺（突发丢包），偏差不可信、已跳过
    fresh: bool = False          # 身份递增且时间连续的新观测
    stationary_sec: float = 0.0
    speed_mps: Optional[float] = None
    drift_m: float = 0.0
    shape_speed_mps: float = 0.0
    yaw_speed_mps: float = 0.0
    motion_observable: bool = False
    motion_pending: bool = False     # 微小快速变化待后续新帧复核；本帧不可测量
    instant_speed_mps: float = 0.0    # 原始相邻帧速度，与回归速度分开保留
    jitter_confirmed: bool = False   # 已用后续新帧确认有界往返或低净速度
    # 本帧使 moving 为真的条件名（已排序）。空元组 = 本帧没有任何条件触发。
    # 现场只看得到"运动中"三个字，查不出卡在哪一条 —— 这个字段就是为此。
    triggers: Tuple[str, ...] = ()
    reason: str = ""

    def to_dict(self) -> Dict:
        return {"state": self.state, "state_cn": StabilityState.CN.get(self.state, "?"),
                "deviation": round(self.deviation, 6), "stable": self.stable,
                "n_points": self.n_points, "elapsed": round(self.elapsed, 2),
                "n_vehicle_points": self.n_vehicle_points,
                "degraded": self.degraded, "fresh": self.fresh,
                "stationary_sec": round(self.stationary_sec, 3),
                "speed_mps": self.speed_mps, "drift_m": self.drift_m,
                "shape_speed_mps": self.shape_speed_mps, "yaw_speed_mps": self.yaw_speed_mps,
                "motion_observable": self.motion_observable,
                "motion_pending": self.motion_pending,
                "instant_speed_mps": self.instant_speed_mps,
                "jitter_confirmed": self.jitter_confirmed,
                "triggers": list(self.triggers),
                "reason": self.reason}


class StabilityDetector:
    """连续停稳门控（功能等价实现），与原厂逐帧偏差诊断分离。

    deviation 仍为相邻完整帧整云质心距离；停稳须由车辆候选点的分位特征、
    单调时间、速度和固定锚点位移共同证明。地面不参与运动特征。
    TODO(现场): 缺省门槛需用已知轨迹标定；ROI 中高于地面的静态设施须排除。
    仅靠点云不能识别任意完全遮挡或几何不可辨识的运动。
    """

    def __init__(self, stability_delta: float = 0.05, vehicle_threshold: int = 3000,
                 warn_sec: float = 30.0, ground_z: float = -5.0,
                 vehicle_min_height_m: float = 0.30,
                 vehicle_release_ratio: float = 0.8,
                 release_frames: int = 3, break_ratio: float = 6.0,
                 count_ratio: float = 0.005,
                 stationary_min_sec: float = 2.0,
                 stationary_speed_mps: float = 0.02,
                 stationary_drift_m: float = 0.03,
                 stationary_speed_window_sec: float = 0.5,
                 stationary_max_gap_sec: float = 0.75,
                 stationary_jitter_m: float = 0.0031,
                 stationary_shape_speed_mps: float = 0.0,
                 stationary_yaw_speed_mps: float = 0.0):
        self.delta = stability_delta
        self.vehicle_threshold = vehicle_threshold
        self.warn_sec = warn_sec
        self.ground_z = float(ground_z)
        self.vehicle_min_height_m = float(vehicle_min_height_m)
        self.vehicle_release_ratio = min(1.0, max(0.1, float(vehicle_release_ratio)))
        # 保留旧配置字段用于诊断兼容，不能再让抖动回差放行可疑测量。
        self.release_frames = max(1, int(release_frames))
        self.break_ratio = max(1.0, float(break_ratio))
        self.count_ratio = max(0.0, float(count_ratio))
        self.count_avg, self.count_gap = 3, 5
        self._n_hist: Deque[int] = deque(maxlen=self.count_avg * 2 + self.count_gap)
        for name, value in (
                ("stationary_min_sec", stationary_min_sec),
                ("stationary_speed_mps", stationary_speed_mps),
                ("stationary_drift_m", stationary_drift_m),
                ("stationary_speed_window_sec", stationary_speed_window_sec),
                ("stationary_max_gap_sec", stationary_max_gap_sec),
                ("stationary_jitter_m", stationary_jitter_m)):
            if not np.isfinite(value) or value <= 0:
                raise ValueError(name + " 必须为有限正数")
            setattr(self, name, float(value))
        # ★ 形变(shape)与旋转(yaw)侧的**独立**门槛（2026-09-21 现场实测定下）
        #
        #   在此之前这两条与平移速度共用 `stationary_speed_mps`(0.02)，而它们的
        #   噪声底高一个数量级 —— 现场 6 个会话 1945 个**真静止**帧实测：
        #       drift       p99 0.0077  超门槛  0.0%
        #       speed       p99 0.0077  超门槛  0.0%
        #       shape_speed p99 0.0894  超门槛 23.4%
        #       yaw_speed   p99 0.0743  超门槛 46.4%
        #   两者同时达标只有约 34~46% 的帧，于是连续 2 秒（20 帧）全达标的概率
        #   是 1e-7 量级 —— **停得死死的车永远判不稳，一辆都测不出来**。
        #   实测最大 stationary_sec 只有 0.5~0.7 s，从未接近 2.0。
        #
        #   ▲ `yaw_speed` = 角速度 x 车厢半长(约 5.4 m)，航向估计抖 0.1 度就换算成
        #     0.094 m/s，是旧门槛的 4.7 倍。184 个**明确移动**帧上它最低只有
        #     0.0002 —— 与静止分布**完全重叠**，本就没有判别力。
        #
        #   ▲▲ 放宽它们**不会**放过移动的车：门槛取 0.10 时，184 个移动帧
        #     被误放过 **0.0%**。真正拦住移动车辆的是 `speed`
        #     （门槛 0.020，移动时最低 0.0513，余量 2.6 倍）与 `count` / `immediate`。
        #
        #   0.0（默认）= 回退到 stationary_speed_mps，**行为与从前逐位一致**。
        for name, value in (("stationary_shape_speed_mps", stationary_shape_speed_mps),
                            ("stationary_yaw_speed_mps", stationary_yaw_speed_mps)):
            if not np.isfinite(value) or value < 0:
                raise ValueError(name + " 必须为有限非负数")
            setattr(self, name, float(value) if value > 0 else self.stationary_speed_mps)
        if self.stationary_speed_window_sec > self.stationary_min_sec:
            raise ValueError("速度窗口不能长于连续停稳时间")
        if self.stationary_max_gap_sec >= self.stationary_min_sec:
            raise ValueError("最大观测间隔必须短于连续停稳时间")
        if self.stationary_jitter_m >= self.stationary_drift_m:
            raise ValueError("抖动复核位移必须小于固定锚点位移门槛")
        # 独立调用者也可从控制线程撤销观测；与整个 detect 共用可重入边界。
        self._state_lock = threading.RLock()
        self.reset()

    def vehicle_points(self, frame: Frame) -> int:
        cut = self.ground_z + self.vehicle_min_height_m
        return int(np.count_nonzero(np.isfinite(frame.xyz).all(axis=1)
                                    & (frame.xyz[:, 2] > cut)))

    @staticmethod
    def _translation_position(features: np.ndarray) -> np.ndarray:
        """平移共用车辆点的水平中位位置，形变仍使用完整三档、三轴特征。"""
        return features[1, :2]

    def reset(self) -> None:
        """明确新场景才清身份；观测故障用 invalidate，不伪造车辆离场。"""
        with self._state_lock:
            self._reset_locked()

    def _reset_locked(self) -> None:
        self._ref: Optional[np.ndarray] = None
        self._present = False
        self._entered = time.monotonic()
        self._last_received: Optional[float] = None
        self._frame_expired = False
        self._last_seq = 0
        self._last_ts: Optional[float] = None
        self._last_gen: Optional[int] = None
        self.invalidate("重新等待连续观测")

    def invalidate(self, reason: str = "观测中断") -> None:
        with self._state_lock:
            self._invalidate_locked(reason)

    def _invalidate_locked(self, reason: str) -> None:
        self._state = StabilityState.BASELINE if self._present else StabilityState.IDLE
        self._candidate_t: Optional[float] = None
        self._candidate_capture_t: Optional[float] = None
        self._anchor: Optional[np.ndarray] = None
        self._motion: Deque[Tuple[float, float, np.ndarray]] = deque()
        self._shape_motion: Deque[tuple] = deque()
        self._shape_anchor: Optional[np.ndarray] = None
        self._heading_anchor: Optional[float] = None
        self._last_heading: Optional[float] = None
        self._last_translation_position: Optional[np.ndarray] = None
        self._translation_pending: Optional[Tuple[float, float, np.ndarray]] = None
        self._n_hist.clear()
        self._over = 0
        self._reason = reason

    def expire(self) -> bool:
        """没有新帧也须撤销停稳许可；短暂轮询空读不算断流。"""
        with self._state_lock:
            return self._expire_locked()

    def _expire_locked(self) -> bool:
        if (not self._frame_expired and self._last_received is not None
                and time.monotonic() - self._last_received > self.stationary_max_gap_sec):
            self._frame_expired = True
            self.invalidate("新帧超时")
            return True
        return False

    @property
    def state(self) -> int:
        with self._state_lock:
            return self._state

    def detect(self, frame: Frame, trusted: bool = True) -> StabilityResult:
        with self._state_lock:
            self._last_pose = None
            return self._detect_locked(frame, trusted)

    def last_pose(self):
        """最近一次 detect 的车辆水平中位位置、航向（可为 None）与半径；无可用车辆观测时为 None。

        只读副产物，供 CH4 位置停稳触发使用；不进入 StabilityResult、不参与本检测器判定。
        """
        with self._state_lock:
            return getattr(self, "_last_pose", None)

    def _detect_locked(self, frame: Frame, trusted: bool) -> StabilityResult:
        trusted = bool(trusted and frame.dropped_packets == 0)
        now = time.monotonic()
        finite = bool(np.isfinite(frame.xyz).all())
        n_veh = self.vehicle_points(frame)
        r = StabilityResult(n_points=frame.n, elapsed=max(0.0, now - self._entered),
                            n_vehicle_points=n_veh)
        # 原厂兼容诊断不作为速度或测量许可；残缺帧不得污染参考质心。
        c = frame.centroid() if finite and frame.n else None
        if c is not None and self._ref is not None:
            r.deviation = float(np.linalg.norm(c - self._ref))
        if trusted and c is not None:
            self._ref = c

        seq, ts, gen = frame.seq, frame.ts, frame.gen
        identity_ok = (isinstance(seq, (int, np.integer)) and seq > 0
                       and np.isfinite(ts) and ts > 0)
        capture_current = identity_ok and 0 <= time.time() - ts <= self.stationary_max_gap_sec
        same_gen = self._last_gen is None or gen == self._last_gen
        gap = (self._last_received is not None
               and now - self._last_received > self.stationary_max_gap_sec)
        time_ok = self._last_received is None or now > self._last_received
        fresh = (identity_ok and capture_current and same_gen and seq > self._last_seq
                 and (self._last_ts is None or ts > self._last_ts) and time_ok)
        # 墙钟回拨只作废当前窗口，下一次新的时戳可重新建立观测；序号仍为高水位。
        if identity_ok and same_gen and seq > self._last_seq:
            self._last_seq, self._last_ts = seq, float(ts)
        if self._last_gen is None:
            self._last_gen = gen
        self._last_received = now
        self._frame_expired = False
        r.fresh = bool(fresh)
        if gap:
            self.invalidate("完整观测间隔超时")
        usable = bool(trusted and finite and fresh)
        gate = (self.vehicle_threshold * self.vehicle_release_ratio
                if self._present else self.vehicle_threshold)
        if usable and n_veh < gate:
            self._present = False
            self._entered = now
            self._ref = None
            self.invalidate("无车")
            r.state, r.reason = self._state, self._reason
            return r
        if trusted and finite and n_veh >= gate:
            if not self._present:
                self._entered = now
            self._present = True
        if not usable:
            self.invalidate("帧不完整" if not trusted else
                            ("点云含非有限值" if not finite else "帧身份或时间不连续"))
            r.state, r.reason, r.degraded = self._state, self._reason, True
            r.triggers = ("frame_incomplete" if not trusted else
                          "frame_nonfinite" if not finite else "frame_identity",)
            return r

        vehicle = frame.xyz[frame.xyz[:, 2] > self.ground_z + self.vehicle_min_height_m]
        if len(vehicle) < 3:
            self.invalidate("车辆观测不足")
            r.state, r.reason = self._state, self._reason
            r.triggers = ("vehicle_sparse",)
            return r
        features = np.percentile(vehicle, [10.0, 50.0, 90.0], axis=0)
        # 整体平移和形状变化分开估计。单个尾分位的随机抽样变化不是刚体速度。
        shape = features - features.mean(axis=0)
        xy = vehicle[:, :2].astype(np.float64)
        xy -= xy.mean(axis=0)
        eig, axes = np.linalg.eigh(xy.T @ xy / len(xy))
        heading = None
        previous_heading = self._last_heading
        radius = float(np.percentile(np.linalg.norm(xy, axis=1), 95))
        # 主轴接近等向时不伪造方向；此时仍检查平移和形状，旋转可观测性有限。
        if eig[-1] > 2 * max(eig[0], 1e-9):
            angle = float(np.arctan2(axes[1, -1], axes[0, -1]))
            if self._last_heading is not None:
                delta = angle - self._last_heading
                angle = self._last_heading + .5 * np.arctan2(np.sin(2 * delta), np.cos(2 * delta))
            heading = angle
        self._last_heading = heading
        spread = np.std(vehicle.astype(np.float64), axis=0)
        spread_speed = 0.0
        if self._shape_motion:
            p = self._shape_motion[-1]
            # 尺度变化也是重新运动，不能等长窗回归稀释后才撤销许可。
            # 高频丢点会抬高本量并保守拒判；不能把随机重采样当作固定射线。
            spread_speed = float(np.max(np.abs(spread - p[5]))) / min(now - p[0], ts - p[1])
        self._shape_motion.append((now, float(ts), shape, heading, radius, spread))
        while (len(self._shape_motion) > 3
               and min(now - self._shape_motion[1][0], ts - self._shape_motion[1][1])
               >= self.stationary_min_sec):
            self._shape_motion.popleft()
        if len(self._shape_motion) > 512:
            self._shape_motion.popleft()
        shape_span = min(now - self._shape_motion[0][0], ts - self._shape_motion[0][1])
        shape_ready = len(self._shape_motion) >= 3 and shape_span + 1e-9 >= self.stationary_min_sec
        shape_speed, shape_drift, yaw_speed, yaw_drift = spread_speed, 0.0, 0.0, 0.0
        mean_shape = None
        if shape_ready:
            shape_rows = list(self._shape_motion)
            shapes = np.stack([x[2] for x in shape_rows])
            mean_shape = shapes.mean(axis=0)
            for index, current in ((0, now), (1, ts)):
                times = np.array([x[index] - current for x in shape_rows])
                times -= times.mean()
                slopes = np.sum(times[:, None, None] * shapes, axis=0) / float(times @ times)
                shape_speed = max(shape_speed, float(np.max(np.linalg.norm(slopes, axis=1))))
            if self._shape_anchor is not None:
                shape_drift = float(np.max(np.linalg.norm(mean_shape - self._shape_anchor, axis=1)))
        if heading is not None:
            if self._heading_anchor is not None:
                yaw_drift = abs(heading - self._heading_anchor) * radius
            previous = list(self._shape_motion)[-2:-1]
            if previous and previous[0][3] is not None:
                p = previous[0]
                yaw_speed = abs(heading - p[3]) * max(radius, p[4]) / min(now - p[0], ts - p[1])
        self._n_hist.append(n_veh)
        moving_count = False
        if self.count_ratio > 0 and len(self._n_hist) >= 2:
            h = list(self._n_hist)
            k = min(self.count_avg, len(h) // 2)
            old_avg, new_avg = sum(h[:k]) / k, sum(h[-k:]) / k
            moving_count = old_avg > 0 and abs(new_avg - old_avg) / old_avg > self.count_ratio

        immediate_speed = 0.0
        immediate_distance = 0.0
        immediate_span = 0.0
        # 三个平移门控统一取p50_xy位置；三档差的中位数不等于p50位置差。
        position = self._translation_position(features)
        self._last_pose = (np.array(position, dtype=np.float64), heading, radius)
        if self._motion:
            last_t, last_capture_t, _ = self._motion[-1]
            # 地面不参与车辆位置；可见面的尾部变化仍交给shape/spread判断。
            immediate_distance = float(np.linalg.norm(position - self._last_translation_position))
            immediate_span = min(now - last_t, ts - last_capture_t)
            immediate_speed = immediate_distance / immediate_span
        # 毫米级共模跳动与起步首帧不可区分：先禁测，随后以真实新帧复核。
        # 只复核平移；形变、偏航、点数突变和固定锚点门槛仍独立否决。
        translation_moving = False
        # 误差与允许微动可能同轴叠加，单步或累计残差超过预算才可直接判运动。
        # 预算内的超速仅能待复核，不能取得当前帧测量许可。
        if (immediate_speed > self.stationary_speed_mps
                and immediate_distance > self.stationary_jitter_m + self.stationary_speed_mps * immediate_span):
            translation_moving = True
            self._translation_pending = None
        elif self._translation_pending is not None:
            pending_t, pending_ts, pending_center = self._translation_pending
            pending_span = min(now - pending_t, ts - pending_ts)
            pending_distance = float(np.linalg.norm(position - pending_center))
            if max(now - pending_t, ts - pending_ts) > self.stationary_speed_window_sec:
                translation_moving = True
                self._translation_pending = None
            elif pending_distance <= self.stationary_speed_mps * pending_span + 1e-7:
                # 允许的低速微动会叠加在抖动上，须先用真实时间跨度复核净速度。
                # 1e-7米仅补p50坐标的数值舍入，不是额外的毫米容忍量。
                # 单步大跳已在上面否决，固定锚点和回归速度仍独立限制累计运动。
                r.jitter_confirmed = True
                self._translation_pending = None
            elif pending_distance > self.stationary_jitter_m + self.stationary_speed_mps * pending_span:
                translation_moving = True
                self._translation_pending = None
            else:
                r.motion_pending = True
        elif immediate_speed > self.stationary_speed_mps:
            self._translation_pending = (last_t, last_capture_t, self._last_translation_position.copy())
            r.motion_pending = True
        r.instant_speed_mps = immediate_speed
        self._last_translation_position = position
        self._motion.append((now, float(ts), position))
        # 保留恰好跨越速度窗的首样本，避免浮点/采样频率令窗口始终不足。
        while (len(self._motion) > 3
               and min(now - self._motion[1][0], ts - self._motion[1][1])
               >= self.stationary_speed_window_sec):
            self._motion.popleft()
        # 异常超高频数据不能无限占内存；过密流不构成足够的时间证据。
        if len(self._motion) > 512:
            self._motion.popleft()
        duration = min(now - self._motion[0][0], ts - self._motion[0][1])
        speed = None
        if len(self._motion) >= 3 and duration + 1e-9 >= self.stationary_speed_window_sec:
            y = np.stack([x[2] for x in self._motion])
            speeds = []
            for index, current in ((0, now), (1, ts)):
                t = np.array([x[index] - current for x in self._motion], dtype=np.float64)
                t -= t.mean()
                slopes = np.sum(t[:, None] * y, axis=0) / float(np.dot(t, t))
                speeds.append(float(np.linalg.norm(slopes)))
            # 即时速度由有界复核处理；混回回归值会让已确认的抖动再次清零。
            speed = max(speeds)
        r.speed_mps = speed
        drift = (float(np.linalg.norm(position - self._translation_position(self._anchor)))
                 if self._anchor is not None else 0.0)
        r.drift_m = max(drift, shape_drift, yaw_drift)
        # 用户确认5毫米/秒可按停稳处理，不另加净位移蠕行否决。
        # 2秒连续观察与3秒沉降保留；微小瞬时超速先禁测，再由新帧复核。
        # 复核受单步位移、速度窗和净速度共同约束；不能等待超时强行测量。
        # 工程噪声用例及10秒代码验证见docs/43；不代表现场准确率已验证。
        # 门槛取值依据（为什么不能放宽到 0.5 m/s）见 docs/50「移动门槛放宽评估」。
        # 简记：0.5 m/s 是 0.02 的 25 倍，会让车在 5 s 测量链路里走 2.5 m
        #（实测精度 68 mm 的 37 倍），且实测放宽后仍拦得住行驶车 —— 解决不了抖动。
        # 抖动的真修法是让 immediate_speed 脱离单帧 dt（见 docs/42 第 3 节）。
        triggers = []
        if heading is None:
            triggers.append("heading_none")
        if previous_heading is None:
            triggers.append("heading_first")
        if moving_count:
            triggers.append("count")
        if drift > self.stationary_drift_m:
            triggers.append("drift")
        if translation_moving:
            triggers.append("immediate")
        if shape_speed > self.stationary_shape_speed_mps:
            triggers.append("shape_speed")
        if shape_drift > self.stationary_drift_m:
            triggers.append("shape_drift")
        if yaw_speed > self.stationary_yaw_speed_mps:
            triggers.append("yaw_speed")
        if yaw_drift > self.stationary_drift_m:
            triggers.append("yaw_drift")
        if speed is not None and speed > self.stationary_speed_mps:
            triggers.append("regression")
        # gap 只记账不参与判定：它已经在上面 invalidate 过（锚点已清），
        # 掺进 moving 会把 reason 从「等待连续停稳」改写成「车辆运动或观测变化」
        # —— 那是改行为，本次只要归因。
        r.triggers = tuple(["gap"] + triggers if gap else triggers)
        moving = bool(triggers)
        if moving:
            self._translation_pending = None
            r.motion_pending = False
            r.jitter_confirmed = False
        elif r.motion_pending:
            r.triggers += ("translation_pending",)
        if self._anchor is None or moving:
            self._anchor, self._candidate_t = features, now
            self._candidate_capture_t = float(ts)
            self._shape_anchor = mean_shape
            self._heading_anchor = heading
        elif self._shape_anchor is None and mean_shape is not None:
            self._shape_anchor = mean_shape
        r.shape_speed_mps, r.yaw_speed_mps = shape_speed, yaw_speed
        r.stationary_sec = max(0.0, min(now - self._candidate_t, ts - self._candidate_capture_t))
        r.motion_observable = heading is not None
        r.stable = bool(not moving and not r.motion_pending and r.motion_observable and shape_ready and speed is not None
                        and r.stationary_sec + 1e-9 >= self.stationary_min_sec)
        self._state = StabilityState.STABLE if r.stable else StabilityState.BASELINE
        self._reason = ("车辆方向不可观测" if not r.motion_observable else "连续停稳" if r.stable else
                        ("车辆运动或观测变化" if moving else
                         "等待微小位移复核" if r.motion_pending else "等待连续停稳"))
        r.state, r.reason = self._state, self._reason
        return r

    def timed_out(self) -> bool:
        with self._state_lock:
            return self.warn_sec > 0 and time.monotonic() - self._entered > self.warn_sec

# ═══════════════════════════════════════════════════════ 判定链

# 判定链里属于"没看清"而不是"看清了、有货"的那几步：
#   1 点数不足 / 2 六面拟合失败 / 3 未见厢底平面 / 4 底面不够水平 / 5 校正点云过少。
# 第 6~10 步都是**拿到了底面、量出了数**之后判的，属于"证据充分"。
# ▲ 与 `load_radar_contract.build_measurement` 里的 `early_failure` **取同一集合** ——
#   那边早就这么分了，这里只是给它一个名字，不要造第二套定义。
#   （第 4 步在现配置下不可达：`gate.normal_z_min` 与
#    `detect_empty.floor.normal_z_threshold` 都是 0.99，第 3 步会先拦。
#    但那是**配置相关**，改任一个值它就活了，所以照样归类，不当死代码删。）
NO_EVIDENCE_STEPS = (1, 2, 3, 4, 5)


@dataclass
class Diagnostics:
    n_points: int = 0
    inliers: int = 0
    ratio: float = 0.0
    span_x: float = 0.0
    span_y: float = 0.0
    normal_z: float = 0.0
    floor_z: float = float("nan")
    min_z: float = float("nan")
    yaw_deg: float = 0.0
    # ★ 第 10 步**实际用的那个量**（2026-09-16 起不再是派生属性）。
    #   以前它恒等于 floor_z - min_z；现在 gate.z_above_mode="plane" 时
    #   用的是"点在平面下方深度的 p99.9"，两者不是一回事。
    #   看板/inspect_cloud/verify 都拿这个数去跟门槛比，所以它必须是
    #   **判定真正用过的那个**，否则看的人会对着一个解释不了结论的数字发呆。
    z_above_used: float = float("nan")
    z_above_mode: str = "origin"
    # ★ 第 10 步兜底用到的三个量。nan = 没算（关闭 / 拿不到 ground_z / 车厢切不出来）。
    #   ▲ bed_exposure 依赖固定的离地高度带，而**每辆车的车厢高度不固定**，
    #     所以现场用的是下面两条**自参考**的量。见 settings.GateCfg.bed_flatness_max。
    bed_exposure: float = float("nan")
    bed_flatness: float = float("nan")      # 底板拟合残差 RMS（底板是造出来的平面，煤面糙）
    bed_thickness: float = float("nan")     # 车厢带内 z 厚度（壁顶 - 最低可见面）
    # ★ 平面在原点处离地多高（floor_z - ground_z）。nan = 拿不到 ground_z。
    #   空车底板实测 1.30~1.64 m，载货拟合面 2.93~3.75 m。
    #   floor_height_blocked = 第 10 步是被"平面离地上限"拦下的（见 GateCfg.floor_height_max_m）。
    floor_height: float = float("nan")
    floor_height_blocked: bool = False
    # ★ 下面三个只做**诊断**，判定链一个字都不看它们（2026-09-16 加）。
    #   加它们是因为现场只能看到一句理由，看不出"离通过差多少"。
    #   见 docs/12 §5 第 1.2 条。
    floor_tilt_deg: float = float("nan")   # 底面平面的倾角（空载自卸车实测 3.8~4.1 度）
    freeboard: float = float("nan")        # 余高 = 壁顶(z 的 p99.5) - 平面在原点的 z
    step: int = 0                          # 判定链走到第几步被拦（11 = 通过）
    # 第 7 步占比门槛折算成的"等效内点数"。摆出来是因为它与第 6 步的绝对
    # 门槛差着一个数量级，而现场只看得到"占比过低"这四个字（见 verdict）。
    ratio_equiv_inliers: float = float("nan")

    @property
    def verdict(self) -> str:
        """把"没看清"与"看清了、有货"分开 —— 判定链本身分不出。

        1~10 步**全部** `return False`，于是「证据不足」与「判有货」压成了
        同一个答案，现场无从判断该**重测**还是该**拦截**。实测 196 帧里
        判 False 的 186 帧中，真正证据不足只有 **12 帧（6.5%）**。

        ▲ 纯诊断，**从 `step` 派生不另存字段** —— 否则两者会漂移。
          判定链的返回值一个字节没动，`cli/verify.py` 不受影响。
        """
        step = int(self.step or 0)
        if step == 11:
            return "PASS_EMPTY"
        if step in NO_EVIDENCE_STEPS:
            return "NO_EVIDENCE"
        return "REJECT_LOADED" if step else ''

    @property
    def z_above(self) -> float:
        if self.z_above_used == self.z_above_used:
            return self.z_above_used
        # 还没走到第 10 步（在更早的步骤被拦下）—— 退回旧口径，仅供显示
        if self.floor_z != self.floor_z or self.min_z != self.min_z:
            return float("nan")
        return self.floor_z - self.min_z

    def to_dict(self) -> Dict:
        d = asdict(self)
        d["z_above"] = None if self.z_above != self.z_above else round(self.z_above, 4)
        for k in ("z_above_used", "floor_tilt_deg", "freeboard", "floor_height"):
            v = getattr(self, k)
            d[k] = None if v != v else round(v, 4)
        d["floor_z"] = None if self.floor_z != self.floor_z else round(self.floor_z, 4)
        d["min_z"] = None if self.min_z != self.min_z else round(self.min_z, 4)
        d["ratio"] = round(self.ratio, 5)
        return d


@dataclass
class LoadProfile:
    """双平面（底板 + 货面）的分析结果。见 analyze_load()。"""
    ok: bool = False
    cells: int = 0                      # 参与分析的格子数
    bed_z: float = float("nan")         # 底板面高度（平面在原点的 z）
    cargo_z: float = float("nan")       # 货面高度
    cargo_ratio: float = 0.0            # 有货的格子占比（0 = 底板全露着）
    cargo_depth: float = float("nan")   # 货面平均高出底板多少（米）
    bed_plane: Plane = field(default_factory=Plane)
    cargo_plane: Plane = field(default_factory=Plane)
    message: str = ""

    def to_dict(self) -> Dict:
        f = lambda v: None if v != v else round(float(v), 4)   # noqa: E731
        return {"ok": bool(self.ok), "cells": int(self.cells),
                "bed_z": f(self.bed_z), "cargo_z": f(self.cargo_z),
                "cargo_ratio": round(float(self.cargo_ratio), 4),
                "cargo_depth": f(self.cargo_depth),
                "message": self.message}


def analyze_load(cloud: np.ndarray, cell_m: float = 0.25, min_cell_pts: int = 4,
                 seed_pct: float = 30.0, cargo_min_h: float = 0.15,
                 iters: int = 6, inset_m: float = 0.5) -> LoadProfile:
    """双平面：同时找**底板**与**货面**，并给出"装了多大一片、装多高"。

    ▲ 先说清楚它解决不了什么
    ------------------------
    **俯视单视角下，装满的车与底板更高的空车在几何上无法区分** ——
    货把底板完全盖住之后，雷达看到的就是一个平面，它是底板还是货面，
    单靠这一片点云说不出来。这是传感器布置的固有极限，不是算法能补的。
    （原算法第 10 步"平面下方有没有东西"正是在绕这个限制：
      装满时厢壁与货之间的缝、边角露出的底板会在平面下方留下点。）

    它真正解决的是 **半装载**（docs/12 §4.4）
    ----------------------------------------
    `CH4_110959` 实测：车厢后半段底板露着（离地 1.19 m），前半段装到 2.9~3.3 m。
    单平面拟合落在底板上，于是"余高"算出 2.77 m —— **比两辆确认空车
    （2.26/2.32）还大**。任何"看一个平面 + 它上面空多少"的判据在这种车上都会
    判成空车。双平面把它拆开：底板一片、货面一片，再报**有货的面积占多少**。

    做法
    ----
      ① 把俯视 footprint 切成 cell_m 的格子，每格取 z 的 p90 当"这一格看到的表面"
         （俯视看到的就是最上面那层）；
      ② 只留**内部格**（四邻都有数据）—— 剔掉厢壁那一圈；
      ③ 拿最低的 seed_pct% 内部格拟一个平面，再按 cargo_min_h 迭代收紧 ——
         这就是**底板面**；
      ④ 表面高出底板面 cargo_min_h 以上的格子算"有货"；
      ⑤ 底板格与有货格各自 PCA 拟一个平面。

    ★ 第 ③ 步必须**相对平面**量，不能用绝对高度。空载自卸车底板向车头翘
      3.8~4.1 度、沿 y 落差 0.68~0.84 m，按绝对高度切会把**斜着的底板自己**
      切成"底板 + 货"：实测确认空车的 cargo_ratio 会报成 85%（2026-09-16 踩过）。
      这与判定链第 10 步栽的是同一个跟头（docs/12 §4.2）。

    ★ 第 ② 步也不能省。厢壁被掠射扫到，边缘那一圈格子的"表面"是壁顶，
      一律会被算成有货 —— 2.5 m 宽的车厢按 0.25 m 切只有 10 格宽，
      两侧各一圈就是 20%。

    ★ 用**格子的表面**而不是原始点：厢壁在整个高度上都有点，直接对点云做高度
      直方图会被糊成一片（docs/12 §4.3 踩过：拿"平面上方有没有点"判空满，
      确认空车也显示"填满了"）。按格子取表面就只剩"这一格最上面是什么"。

    ▲ 纯诊断，判定链默认**一个字都不看它**。
    """
    out = LoadProfile()
    if cloud is None or len(cloud) < 200:
        out.message = "点太少"
        return out
    # ★ 非有限值必须挡在这里，与同模块的 carriage_box / carriage_y_window 同一条纪律。
    #   没有这一行时，下面的 np.ceil(...) 取整会抛
    #     全 NaN -> ValueError: cannot convert float NaN to integer
    #     含 inf -> OverflowError: cannot convert float infinity to integer
    #   报错里没有任何线索指向"点云含非有限值"。服务侧被停稳层的
    #   frame_nonfinite 早退挡着走不到这里，但 cli/inspect_cloud.py、
    #   cli/replay.py 这些现场工具是**直接对文件**调的 —— 一份坏 .xyz
    #   就会在现场得到那个毫无指向性的报错。见 docs/55 M3。
    cloud = np.asarray(cloud)
    if cloud.ndim != 2 or cloud.shape[1] < 3 or not np.isfinite(cloud[:, :3]).all():
        out.message = "点云含非有限值或形状不对"
        return out
    x, y, z = cloud[:, 0], cloud[:, 1], cloud[:, 2]
    nx = int(np.ceil((x.max() - x.min()) / cell_m)) + 1
    ny = int(np.ceil((y.max() - y.min()) / cell_m)) + 1
    if nx < 4 or ny < 4 or nx * ny > 4_000_000:
        out.message = "格子尺寸不合适"
        return out
    ix = np.clip(((x - x.min()) / cell_m).astype(np.int64), 0, nx - 1)
    iy = np.clip(((y - y.min()) / cell_m).astype(np.int64), 0, ny - 1)

    # ① 每格的表面高度 = z 的 p90。用排序分组，避免逐格 Python 循环。
    flat = ix * ny + iy
    order = np.argsort(flat, kind="stable")
    fs, zs = flat[order], z[order]
    cut = np.flatnonzero(np.diff(fs)) + 1
    starts = np.concatenate([[0], cut])
    ends = np.concatenate([cut, [len(fs)]])
    surf = np.full(nx * ny, np.nan)
    for a_, b_ in zip(starts, ends):
        if b_ - a_ >= min_cell_pts:
            surf[fs[a_]] = np.percentile(zs[a_:b_], 90)
    grid = surf.reshape(nx, ny)

    # ② 只留内部格（四邻都有数据）—— 剔掉厢壁那一圈
    inner = ~np.isnan(grid)
    for _ in range(max(1, int(round(inset_m / cell_m)))):
        e = np.zeros_like(inner)
        e[1:-1, 1:-1] = (inner[1:-1, 1:-1] & inner[:-2, 1:-1] & inner[2:, 1:-1]
                         & inner[1:-1, :-2] & inner[1:-1, 2:])
        inner = e
    gi, gj = np.nonzero(inner)
    if len(gi) < 20:
        out.message = "内部格太少（车厢太窄或点太稀）"
        return out
    cx = x.min() + (gi + 0.5) * cell_m
    cy = y.min() + (gj + 0.5) * cell_m
    cz = grid[gi, gj]
    cells = np.column_stack([cx, cy, cz])
    out.cells = int(len(cells))

    # ③ 底板面：最低的 seed_pct% 内部格做种子，再迭代收紧（与 fit_floor 同一套路数）
    plane = _pca_oriented(cells[cz <= np.percentile(cz, seed_pct)])
    if plane is None:
        out.message = "底板面拟合失败"
        return out
    for _ in range(max(1, int(iters))):
        h = cz - (-(plane.a * cx + plane.b * cy + plane.d) / plane.c)             if abs(plane.c) > 1e-9 else None
        if h is None:
            break
        keep = np.abs(h) <= cargo_min_h
        if keep.sum() < 10:
            break
        nxt = _pca_oriented(cells[keep])
        if nxt is None or abs(nxt.c) < 1e-9:
            break
        plane = nxt
    if abs(plane.c) < 1e-9:
        out.message = "底板面近乎竖直，不可信"
        return out
    out.bed_plane = plane
    out.bed_z = -plane.d / plane.c

    # ④ 相对底板面的高度 -> 有货的格子
    h = cz - (-(plane.a * cx + plane.b * cy + plane.d) / plane.c)
    cargo = h > cargo_min_h

    # ★ 货是**一片**，结构是**一条线** —— 把只有一格宽的东西剔掉。
    #
    #   2026-09-16 实测：两辆车厢里什么都没有的车，"高出底板"的格子全是
    #   **横跨整个车宽、只有一格厚**的直线：
    #     `CH4_110959`  y=+0.78 一排 7 格，高度恒定离地 4.02 m  -> 挂车前挡板
    #     `CH4_161309`  y=-2.14 / +1.36 / +4.36 三排           -> 车厢横梁
    #   不剔掉它们，空车的 cargo_ratio 会报成 8%~9%，与半装载混在一起。
    #   做法：对"有货"的格子做一次四邻侵蚀 —— 一格厚的线整条消失，
    #   成片的货堆只掉一圈边。
    if cargo.any():
        nxg = int(gi.max()) + 2
        nyg = int(gj.max()) + 2
        m2 = np.zeros((nxg, nyg), dtype=bool)
        m2[gi[cargo], gj[cargo]] = True
        keep2 = np.zeros_like(m2)
        keep2[1:-1, 1:-1] = (m2[1:-1, 1:-1] & m2[:-2, 1:-1] & m2[2:, 1:-1]
                             & m2[1:-1, :-2] & m2[1:-1, 2:])
        cargo = keep2[gi, gj]
    out.cargo_ratio = float(cargo.mean())
    if cargo.any():
        out.cargo_depth = float(np.median(h[cargo]))
        out.cargo_z = out.bed_z + out.cargo_depth
        pl = _pca_oriented(cells[cargo]) if cargo.sum() >= 6 else None
        if pl is not None:
            out.cargo_plane = pl
    out.ok = True
    r = out.cargo_ratio
    out.message = ("底板全露着" if r < 0.05 else
                   ("基本装满" if r > 0.90 else
                    "半装载：%.0f%% 的面积有货，平均高出底板 %.2f m"
                    % (100 * r, out.cargo_depth)))
    return out


def reason_detail(dg: "Diagnostics", dev) -> str:
    """把"到底卡在哪、差多少"说成一句人话。**不改 reason 本身。**

    为什么需要（docs/14 §3.1）：判定链的第 8 步（内点 x 跨度）与第 9 步
    （内点 y 跨度）返回的是**同一个** reason 字符串「底面覆盖范围不足」，
    只有 `dg.step` 能区分。而这两者的现场处置完全不同：

        第 8 步（x 不够）-> 车停歪了、或 ROI 在横向偏了；
        第 9 步（y 不够）-> 车厢太短、或车厢被 ROI 切掉了一截。

    看 `reason.txt` 或看板的人分不出是哪一个，只能去翻 `gates`。

    ▲ **不能改 reason 字符串** —— 它与原厂逐字一致，改了直接破 `cli/verify.py`
      （那是本项目"移植没走样"的唯一硬证据）。所以只能在**诊断侧**补，
      让人看到的地方多一行说明，而落盘的 reason.txt 一个字节都不动。

    返回形如：
        `第 8 步｜内点 X 跨度 1.62 < 1.80（差 0.18）`
        `第 11 步｜11 步全过 = 空车`
    拿不到诊断时返回空串。
    """
    if dg is None:
        return ""
    step = int(getattr(dg, "step", 0) or 0)
    if step == 11:
        return "第 11 步｜11 步全过 = 空车"
    try:
        rows = gate_report(dg, dev)
    except Exception:                                        # noqa: BLE001
        return ""
    hit = None
    for r in rows:
        if r.get("state") == "block":
            hit = r
            break
    if hit is None:
        return "第 %d 步｜（没有哪一条被判成不合格，多半是诊断不全）" % step if step else ""
    v, lim, op = hit.get("value"), hit.get("limit"), hit.get("op", "")
    want = {">=": "<", "<=": ">"}.get(op, "!=")          # 不合格时的实际关系
    # 有几步没有数值门槛（比如第 2 步"六面拟合失败"、第 3 步"未见厢底平面"），
    # 那里 value/limit 是 None —— 只报是哪一步，别去 float(None)。
    if not isinstance(v, (int, float)) or not isinstance(lim, (int, float)):
        # 这几步是"成/不成"的布尔判据，没有数值可比 —— 明说"未通过"，
        # 否则只写条目名（"厢底平面有效"）读起来像是通过了。
        return "第 %s 步｜%s：未通过" % (hit.get("step"), hit.get("name"))
    if v != v:
        return "第 %s 步｜%s 没算出来（NaN）" % (hit.get("step"), hit.get("name"))
    gap = hit.get("margin")
    tail = ""
    if isinstance(gap, (int, float)) and gap == gap:
        tail = "（差 %.3g）" % abs(gap)
    return ("第 %s 步｜%s %.4g %s %.4g%s" % (hit.get("step"), hit.get("name"),
                                             float(v), want, float(lim), tail)
            + _load_hint(dg, hit, getattr(dev, "gate", dev)))


def _load_hint(dg: "Diagnostics", hit, gate) -> str:
    """第 6/7 步补一句"这车到底装了多少"。

    实测第 7 步 161 帧：**结论全对，理由全错**。真实原因是拟合面落在
    **货堆顶**（余高中位 0.59 m、最大 1.44 m，全在满载区间 0.18~1.40；
    z_above 中位 0.377 m 是门槛的 4.7 倍），报出来的却是"内点占比过低"。
    判定不能改（reason 与原厂逐字一致），但可以把物理量摆在旁边，
    免得现场顺着"占比"去调 `min_inlier_ratio` —— 那个方向救不了任何一辆空车。
    """
    if int(hit.get("step") or 0) == 10 and getattr(dg, "floor_height_blocked", False):
        return "；平面离地过高 —— 拟合到的是刮平的货面，不是底板（空车底板实测 1.30~1.64 m）"
    if int(hit.get("step") or 0) not in (6, 7):
        return ""
    bits = []
    fb, za = dg.freeboard, dg.z_above_used
    if fb == fb:
        bits.append("余高 %.2f m（空车实测 2.17~2.32，满载 0.18~1.40）" % fb)
    if za == za:
        over = "超" if za > gate.z_above_min_max_m else "未超"
        bits.append("z_above %.3f m %s第 10 步门槛 %.3g（即便放宽本步也会卡在那里）"
                    % (za, over, gate.z_above_min_max_m))
    equiv = hit.get("equiv_inliers")
    if equiv:
        bits.append("内点 %d；本步等效要求 %d，而第 6 步只要求 %d"
                    % (int(hit.get("inliers") or 0), int(equiv), int(gate.min_inliers)))
    return ("；" + "，".join(bits)) if bits else ""


def gate_report(dg: "Diagnostics", dev) -> List[Dict]:
    """把 11 步判定链摊成一张表：每一步量的是什么、实测多少、门槛多少、余量多少。

    为什么需要：现场只能看到一句理由（比如「底面过高，疑似载物」），
    **看不出离通过差多少**。2026-09-15 两辆空车的 z_above 是 0.318/0.349、
    门槛 0.08 —— 差 4 倍还是差 1%，处理方式完全不同，而现在的输出分不出来。

    返回每一步一个 dict：
        step      第几步（1~11）
        name      这一步在量什么
        value     实测值（None = 这一步没有数值判据，或根本没走到）
        limit     门槛（None = 无数值门槛）
        op        "<=" 或 ">="：value 要满足哪一侧
        margin    离门槛还有多少余量（正数 = 过了并且富余这么多）
        state     "pass" 过了 / "block" 就是它拦下的 / "skip" 没走到
    ▲ 纯诊断，判定链一个字都不看它。
    """
    gate = getattr(dev, "gate", dev)
    msp = int(getattr(dev, "min_save_points", 20000))
    hit = int(getattr(dg, "step", 0) or 0)
    rows = [
        (1, "点数", float(dg.n_points), float(msp), ">="),
        (2, "六面拟合", None, None, ""),
        (3, "厢底平面有效", None, None, ""),
        (4, "底面法向 |nz|", dg.normal_z, gate.normal_z_min, ">="),
        (5, "校正后点数", None, None, ""),
        (6, "底面内点数", float(dg.inliers), float(gate.min_inliers), ">="),
        # ▲ 第 7 步与第 6 步量的是**同一件事**，但严格度差着一个数量级：
        #   min_inliers=600 vs 0.15 x 约 10 万点 = 约 15000 个内点，差约 25 倍。
        #   于是第 6 步实际上被架空，第 7 步在独自干活 —— 而现场只看得到
        #   "占比过低"四个字，会去调错的那一个。等效值见下面的 equiv_inliers。
        (7, "底面内点占比", dg.ratio, gate.min_inlier_ratio, ">="),
        (8, "内点 X 跨度", dg.span_x, gate.min_span_x_m, ">="),
        (9, "内点 Y 跨度", dg.span_y, gate.min_span_y_m, ">="),
        # 第 10 步有两道判据；表只有 11 行，所以这一行显示**真正拦下的那一道**。
        ((10, "底面离地高度", dg.floor_height, float(getattr(gate, "floor_height_max_m", 0.0)), "<=")
         if getattr(dg, "floor_height_blocked", False) else
         (10, "底面高出量 z_above", dg.z_above_used, gate.z_above_min_max_m, "<=")),
        (11, "全部通过", None, None, ""),
    ]
    out = []
    for step, name, val, lim, op in rows:
        if not hit:
            # ★ step==0 = 没走到任何一步（手工构造的 Diagnostics / 判定链
            #   中途异常）：全表标 pass 是方向性误导 —— 看板会展示一张
            #   "全部通过"的表。如实标 unknown（2026-09-23，低严重度批次）。
            state = "unknown"
        elif step > hit:
            state = "skip"                 # 短路了，这一步根本没走到
        elif step == hit:
            state = "block" if step != 11 else "pass"
        else:
            state = "pass"
        margin = None
        if val is not None and lim is not None and val == val:
            margin = (val - lim) if op == ">=" else (lim - val)
        row = {
            "step": step, "name": name, "op": op,
            "value": (None if val is None or val != val else round(float(val), 5)),
            "limit": (None if lim is None else round(float(lim), 5)),
            "margin": (None if margin is None else round(float(margin), 5)),
            "state": state,
        }
        if step in (6, 7):
            # 两步换算到同一量纲（绝对内点数），现场一眼看出谁在真正把关。
            equiv = float(getattr(dg, "ratio_equiv_inliers", float("nan")))
            row["inliers"] = int(dg.inliers)
            row["equiv_inliers"] = (None if equiv != equiv else int(round(equiv)))
        out.append(row)
    return out


def evaluate_empty(res: Optional[EmptyResult], n_input: int, cloud: np.ndarray,
                   dev, min_save_points: Optional[int] = None,
                   ground_z: Optional[float] = None
                   ) -> Tuple[bool, str, Diagnostics]:
    """
    空车判定链 —— 逐条对齐 calibrate_test.exe @0x1400124C0 的反汇编。

        1. n < min_save_points           → 点云数量不足
        2. 无有效检测结果                 → 空车检测失败
        3. floor.valid == 0              → 未见厢底平面
        4. |floor.normal.z| < normal_z_min → 底面不够水平
        5. 判定点云 < 3 点                → 校正点云过少
        6. 内点数 < min_inliers           → 底面内点不足
        7. 内点占比 < min_inlier_ratio    → 底面内点占比过低
        8. 内点 span_x < min_span_x_m     → 底面覆盖范围不足
        9. 内点 span_y < min_span_y_m     → 底面覆盖范围不足
       10. z_above > z_above_max          → 底面过高，疑似载物
           （z_above 的算法由 gate.z_above_mode 决定：
            "origin" = 原厂的 floor_z(原点) - min_z(全局)；
            "plane"  = 点在平面下方深度的 p99.9，倾斜无关）
           plane 档另有一道：平面在原点离地 > floor_height_max_m → 同一理由
           （2026-09-24 加，拦刮平的满载煤面；0 = 关闭，需要 ground_z）
       11. 全部通过                        → 空车

    内点判据：点到拟合底面的**归一化距离** ≤ floor_gate_distance_m。

    参数 dev 可以是：
        · settings.DevicesCfg  —— 首选。门槛取 dev.gate，最少点数取 dev.min_save_points
        · settings.GateCfg     —— 只给门槛，最少点数走 min_save_points 参数/默认值
    这样既贴合原版结构（最少点数与 7 个门槛分属不同字段），又能被单测直接调用。

    该顺序与阈值已在 10/10 历史样本上与真实 reason.txt 完全一致（见 cli/verify.py）。
    修改任何一步之前先跑回归，否则会悄悄改变现场行为。
    """
    gate = getattr(dev, "gate", dev)
    if min_save_points is None:
        min_save_points = int(getattr(dev, "min_save_points", 20000))
    dg = Diagnostics(n_points=n_input)
    if n_input < min_save_points:
        dg.step = 1
        return False, R_INSUFFICIENT_POINTS, dg
    # 第 2 步：detect 根本没产出结果（点数不足 / 底面连拟合都失败）
    # 第 3 步：产出了结果，但底面不够水平被判无效 → 「未见厢底平面」
    #   注：原版这两步的边界（第 2 步到底在什么条件下触发）无法从反汇编完全确定，
    #   因为 10 个历史样本都落在第 6~10 步，从没走到这两个分支。
    #   这里按"语义最自然"的方式划分；若现场遇到，以实际现象为准再调。
    if res is None or len(cloud) == 0:
        dg.step = 2
        return False, R_EMPTY_FAILED, dg
    fl = res.plane("floor")
    dg.yaw_deg = math.degrees(res.yaw)
    if not fl.valid:
        dg.step = 3
        return False, R_NO_FLOOR, dg

    dg.normal_z = abs(fl.c)
    # 纯诊断（判定链不看）：底面倾角。空载自卸车底板向车头翘 3.8~4.1 度，
    # 正是它让原厂的第 10 步把空车判成载货 —— 摊在这里让现场一眼看见。
    dg.floor_tilt_deg = math.degrees(math.acos(min(1.0, dg.normal_z)))
    if dg.normal_z < gate.normal_z_min:
        dg.step = 4
        return False, R_FLOOR_NOT_LEVEL, dg
    if len(cloud) < 3:
        dg.step = 5
        return False, R_TOO_FEW_CORRECTION, dg

    # 纯诊断（判定链不看）：底面高度与余高，**尽早算出来** ——
    # 即使后面在第 6/7 步就被拦下，现场也能看到"这车到底装了多少"。
    # 余高 = 壁顶(z 的 p99.5) - 平面在原点的 z：空车实测 2.17~2.32 m、
    # 满载 0.18~1.40 m。▲ 但它**不能单独当判据**：半装载的车底板露一半时
    # 余高能到 2.77 m，比空车还大（docs/12 §4.4）。
    dg.min_z = float(cloud[:, 2].min())
    if abs(fl.c) > 1e-6:
        dg.floor_z = -fl.d / fl.c
        dg.freeboard = float(np.percentile(cloud[:, 2], 99.5)) - dg.floor_z
        if ground_z is not None:
            dg.floor_height = dg.floor_z - float(ground_z)
        # ★ z_above 也**无条件算出来**（2026-09-19 挪上来）。
        #   以前它写在第 10 步的分支里，于是在第 6/7 步被拦的帧上永远是 NaN ——
        #   实测第 7 步 **161/161 全 NaN**，而补算出来中位 **0.377 m**、
        #   门槛 0.08，**99% 的帧在第 10 步照样会被拦**。
        #   也就是说这个物理量一直存在，只是没人看得到，
        #   现场因此会误以为"放宽占比就能救空车"。
        #   ▲ 值与从前**逐位相同**，第 10 步只是改成读这里算好的结果。
        dg.z_above_mode = mode = str(getattr(gate, "z_above_mode", "origin"))
        if mode == "plane":
            plane_z = -(fl.a * cloud[:, 0] + fl.b * cloud[:, 1] + fl.d) / fl.c
            dg.z_above_used = float(np.percentile(plane_z - cloud[:, 2], 99.9))
        else:
            dg.z_above_used = dg.floor_z - dg.min_z

    dist = fl.abs_distance(cloud)
    mask = dist <= gate.distance_m
    dg.inliers = int(mask.sum())
    dg.ratio = dg.inliers / float(len(cloud))
    dg.ratio_equiv_inliers = gate.min_inlier_ratio * float(len(cloud))
    if dg.inliers:
        inl = cloud[mask]
        dg.span_x = float(inl[:, 0].max() - inl[:, 0].min())
        dg.span_y = float(inl[:, 1].max() - inl[:, 1].min())

    if dg.inliers < gate.min_inliers:
        dg.step = 6
        return False, R_FLOOR_FEW_INLIERS, dg
    if dg.ratio < gate.min_inlier_ratio:
        dg.step = 7
        return False, R_FLOOR_LOW_RATIO, dg
    if dg.span_x < gate.min_span_x_m:
        dg.step = 8
        return False, R_FLOOR_COVERAGE, dg
    if dg.span_y < gate.min_span_y_m:
        dg.step = 9
        return False, R_FLOOR_COVERAGE, dg

    # 第 10 步：度量已在上面算好（见 settings.GateCfg.z_above_mode）。
    #   "origin"（原厂）：平面在**原点**的 z 减**整片**点云最低点。
    #     两个量不在同一个地方取，底板一斜就差出落差的一半。
    #   "plane"（倾斜无关）：点在平面**下方**深度的 p99.9。
    if abs(fl.c) > 1e-6:
        # ★ 平面离地上限（2026-09-24 现场加，见 settings.GateCfg.floor_height_max_m）：
        #   上面两种 z_above 量的都是"平面**下方**还有没有东西"，
        #   而煤面被刮平时平面**本身**就是货面、下面全是煤 —— z_above 天生量不出来。
        #   ▲ 只在 plane 档、拿得到 ground_z、门槛 > 0 时生效；否则行为逐位不变。
        max_h = float(getattr(gate, "floor_height_max_m", 0.0) or 0.0)
        if (max_h > 0 and dg.z_above_mode == "plane"
                and dg.floor_height == dg.floor_height and dg.floor_height > max_h):
            dg.floor_height_blocked = True
            dg.step = 10
            return False, R_FLOOR_TOO_HIGH, dg
        if dg.z_above_used > gate.z_above_min_max_m:
            # ★★ 第 10 步兜底：用**这辆车自己量出来**的两条证据放行真空车。
            #
            #   为什么需要：z_above 在空车与载货之间**完全穿插** ——
            #   4 份空车真值 0.0409/0.0607/0.3308/0.3617，载货 0.3024/0.9422，
            #   载货那个 0.3024 正夹在空车中间。**调门槛救不了任何一辆空车**。
            #
            #   用哪两条（62 份全量实测，都零重叠）：
            #     ① 底板平整度  空车 0.0189~0.0244  其余 58 份最小 0.0421
            #     ② 车厢 z 厚度 空车 2.423~2.957    其余 58 份最大 1.692
            #   两条都与"车厢离地多高"无关 —— 而底板露出要靠固定的离地带，
            #   62 份里已有 7 份载货的最低面落进带内（1.828~1.9），靠不住。
            #
            #   ▲ **必须同时满足**：只看平整度，铺平的非煤货物（砂石/板材）
            #     会骗过它（合成载货场景的货顶是完美平面）；只看厚度，
            #     124 份全量是连续谱，1.2~2.4 之间压着 43 份模糊带。
            #
            #   ▲ 只在 z_above 已经超标时才算它 —— 没超标的本来就放行，
            #     不必多花这一次 carriage_profile（实测约 30 ms）。
            #   ▲ 拿不到 ground_z 或车厢切不出来时**不放行**，回到原行为：
            #     宁可漏判成载货，也不能把载货放成空车（那会被存成空车基线）。
            #   ▲▲ **只在 plane 档生效**。origin 档是"原厂行为"的定义，
            #      必须与 calibrate_test.exe 逐字一致 —— 那 10 份回归基线里
            #      有 4 份锁的正是原厂在这一步的误判，不能被新规则改掉。
            #      (`cli/verify.py --gate-mode origin --strict` 钉着这件事)
            need = (gate.bed_flatness_max > 0 or gate.bed_thickness_min > 0
                    or gate.bed_exposure_min > 0)
            if (need and ground_z is not None
                    and getattr(gate, "z_above_mode", "origin") == "plane"):
                try:
                    prof = carriage_profile(np.asarray(cloud, np.float64), float(ground_z))
                except Exception:                            # noqa: BLE001
                    prof = None
                if prof is not None:
                    dg.bed_exposure = float(prof.get("bed_exposure", float("nan")))
                    dg.bed_flatness = float(prof.get("bed_plane_rms_m", float("nan")))
                    dg.bed_thickness = float(prof.get("thickness_m", float("nan")))

            def _within(value: float, limit: float, upper: bool) -> bool:
                """限为 0 = 这一条不参与；值算不出来（nan）一律**不放行**。"""
                if limit <= 0:
                    return True
                if value != value:
                    return False
                return value <= limit if upper else value >= limit

            if not (need
                    and _within(dg.bed_flatness, gate.bed_flatness_max, True)
                    and _within(dg.bed_thickness, gate.bed_thickness_min, False)
                    and _within(dg.bed_exposure, gate.bed_exposure_min, False)):
                dg.step = 10
                return False, R_FLOOR_TOO_HIGH, dg
    dg.step = 11
    return True, R_PASS_EMPTY, dg


def check_dims(res: EmptyResult, dev) -> Tuple[bool, str]:
    """车厢尺寸达标检查（对应 exe 中「尺寸不达标 / 要求长」的提示）。"""
    dx, dy, dz = res.dims
    bad = []
    if dx < dev.dims_min_width_m:
        bad.append(f"宽 {dx:.2f} < {dev.dims_min_width_m}")
    if dy < dev.dims_min_length_m:
        bad.append(f"长 {dy:.2f} < {dev.dims_min_length_m}")
    if dz < dev.dims_min_height_m:
        bad.append(f"高 {dz:.2f} < {dev.dims_min_height_m}")
    if bad:
        return False, f"{R_SIZE_BAD}：" + "，".join(bad)
    return True, ""
