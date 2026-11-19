#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
geometry.py — 平面拟合与坐标矫正的原语

这里只放「无状态、可单测」的几何函数。检测流程见 detect.py。

▲ 与原厂的关系
    原厂 detectempty.dll 的平面拟合实现细节（RANSAC 具体形式、迭代终止条件）
    无法从剥离了 PDB 的机器码中逐行还原。本模块给出的是**功能等价**的实现：

        · 已在 10/10 历史样本上产出与原程序完全一致的判定结论
          （见 cli/verify.py）
        · 轴距/门槛语义与原版一致（参数全部来自 config.json / devices.json）
        · 若上现场后发现某个样本结论不一致，先用 cli/inspect.py 导出
          平面参数与内点占比，再回来微调这里的 fit_* 函数即可。

    换句话说：这里是可以放心改的地方，改完跑 cli/verify.py 回归即可。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

AXIS_NAME = ("x", "y", "z")


@dataclass
class Plane:
    """平面  a·x + b·y + c·z + d = 0，法向 (a,b,c) 已归一化为单位向量。"""
    a: float = 0.0
    b: float = 0.0
    c: float = 0.0
    d: float = 0.0
    valid: bool = False
    inliers: int = 0            # 命中点数（诊断用，原版结构体里没有，但现场很有用）
    rms: float = 0.0            # 内点残差均方根（诊断用）
    note: str = ""              # 拟合过程的说明（如"窗口自动放宽"），只用于现场排查

    # ── 基本量 ────────────────────────────────────────────
    def normal(self) -> np.ndarray:
        return np.array([self.a, self.b, self.c], np.float64)

    def norm(self) -> float:
        return math.sqrt(self.a * self.a + self.b * self.b + self.c * self.c)

    def distance(self, pts: np.ndarray) -> np.ndarray:
        """带符号距离（本实现内部法向已单位化，故直接点乘）。"""
        n = self.norm()
        if n <= 1e-9:
            return np.zeros(len(pts), np.float64)
        return (pts @ self.normal() + self.d) / n

    def abs_distance(self, pts: np.ndarray) -> np.ndarray:
        return np.abs(self.distance(pts))

    def intercept(self, axis: int) -> float:
        """平面与某坐标轴的截距（法向该分量过小时返回 nan）。"""
        k = (self.a, self.b, self.c)[axis]
        return -self.d / k if abs(k) > 1e-9 else float("nan")

    def is_horizontal(self, thr: float = 0.99) -> bool:
        return abs(self.c) >= thr

    def flip(self) -> "Plane":
        return Plane(-self.a, -self.b, -self.c, -self.d, self.valid, self.inliers, self.rms)

    def orient_towards(self, axis: int, sign: int) -> "Plane":
        """让法向在该轴上的分量为 sign 方向。"""
        return self if self.normal()[axis] * sign >= 0 else self.flip()

    def as_tuple(self) -> Tuple[float, float, float, float]:
        return (self.a, self.b, self.c, self.d)

    def __repr__(self) -> str:
        return (f"Plane({self.a:+.5f},{self.b:+.5f},{self.c:+.5f},{self.d:+.5f}"
                f" valid={int(self.valid)} inl={self.inliers} rms={self.rms:.4f})")


# ─────────────────────────────────────────────── 拟合

def _plane_from_normal(n: np.ndarray, p0: np.ndarray) -> Plane:
    n = np.asarray(n, np.float64)
    nn = np.linalg.norm(n)
    if nn <= 1e-12:
        return Plane()
    n = n / nn
    d = -float(n @ p0)
    return Plane(float(n[0]), float(n[1]), float(n[2]), d, True)


def fit_plane_pca(pts: np.ndarray) -> Optional[Plane]:
    """最小二乘平面（PCA 最小特征向量）。对薄而长的面最稳。

    ▲ 退化判据用的是 **w[1]**，不是 w[0]。
        eigh 升序返回三个特征值：w[0] 是沿法向的方差（＝平面残差），
        w[1]/w[2] 是平面内两个方向的展布。
        · 一个**完美平面**的 w[0] 恰好等于 0 —— 这是最理想的输入，不是退化；
          曾经写成 `if w[0] <= 0: return None`，结果实测：车厢地板扫得足够平
          （float32 下残差下溢到 0）时 fit_plane_pca 返回 None
          → fit_floor 返回 None → detect_empty 返回 None
          → 判定链第 2 步误报「空车检测失败」，而真实情况是底面拟合得完美。
        · 真正的退化是"点共线或重合"，此时**平面内**没有两个独立方向，
          表现为 w[1] ≈ 0 —— 这才是该拒绝的情况。

    另：协方差在 float64 下累加。点云本身是 float32，6 万个 ~25 量级的数
    在 float32 里累加有效位只剩 ~3 位，会让 |法向 z| 在 0.99 门槛附近抖动。
    """
    if len(pts) < 3:
        return None
    q64 = np.asarray(pts, np.float64)
    c = q64.mean(axis=0)
    q = q64 - c
    cov = q.T @ q / len(q)
    try:
        w, v = np.linalg.eigh(cov)
    except np.linalg.LinAlgError:
        return None
    if not np.all(np.isfinite(w)):
        return None
    # 退化判据取**相对**尺度：绝对阈值会随点云量纲失效
    # （共线点集的 w[1] 只是相对 w[2] 极小，绝对值未必小于某个常数）。
    if w[2] <= 0 or w[1] <= w[2] * 1e-12:
        # 点共线 / 全部重合 —— 平面内没有两个独立方向，平面不可定
        return None
    p = _plane_from_normal(v[:, 0], c)
    p.inliers = len(pts)
    r = p.abs_distance(pts)
    p.rms = float(np.sqrt((r * r).mean()))
    return p


def fit_plane_ransac(pts: np.ndarray, thr: float, iters: int = 64,
                     min_inliers: int = 3, seed: int = 0,
                     prior: Optional[Plane] = None) -> Optional[Plane]:
    """
    RANSAC 平面拟合。

    prior 用于「给个先验方向」：在车厢这种结构化场景里，
    先验能大幅提高成功率（例如已知要找水平面时，只接受法向接近 ±z 的假设）。
    """
    n = len(pts)
    if n < 3:
        return None
    if n <= 24:
        return fit_plane_pca(pts)
    rng = np.random.default_rng(seed)
    best: Optional[Plane] = None
    best_cnt = -1
    for _ in range(iters):
        idx = rng.choice(n, 3, replace=False)
        p = fit_plane_pca(pts[idx])
        if p is None:
            continue
        if prior is not None and float(p.normal() @ prior.normal()) < 0.0:
            p = p.flip()
        d = p.abs_distance(pts)
        cnt = int((d <= thr).sum())
        if cnt > best_cnt:
            best_cnt, best = cnt, p
    if best is None or best_cnt < min_inliers:
        return None
    # 用全部内点重拟合
    inl = pts[best.abs_distance(pts) <= thr]
    if len(inl) >= 3:
        refined = fit_plane_pca(inl)
        if refined is not None:
            if prior is not None and float(refined.normal() @ prior.normal()) < 0.0:
                refined = refined.flip()
            best = refined
            # ★ inliers 必须与**最终平面**严格对应（2026-09-23，低严重度批次）：
            #   原来把重拟合前的全点内点数写回 —— 平面已是 refine 后的，
            #   内点数却还是旧的，诊断字段张冠李戴。
            best_cnt = int((best.abs_distance(pts) <= thr).sum())
    best.inliers = best_cnt
    return best


def refine_inliers(pts: np.ndarray, plane: Plane, thr: float,
                   max_iters: int = 10, tol: float = 1e-7) -> Tuple[Plane, np.ndarray]:
    """
    迭代重拟合：收紧内点 → 重新拟合 → 直到参数收敛。
    对应 config.detect_empty.floor 的 max_iterations。
    返回 (平面, 内点布尔掩码)。
    """
    cur = plane
    mask = cur.abs_distance(pts) <= thr
    for _ in range(max(1, max_iters)):
        if int(mask.sum()) < 3:
            break
        nxt = fit_plane_pca(pts[mask])
        if nxt is None:
            break
        delta = (abs(nxt.a - cur.a) + abs(nxt.b - cur.b)
                 + abs(nxt.c - cur.c) + abs(nxt.d - cur.d))
        cur = nxt
        mask = cur.abs_distance(pts) <= thr
        if delta < tol:
            break
    cur.inliers = int(mask.sum())
    if cur.inliers:
        r = cur.abs_distance(pts[mask])
        cur.rms = float(np.sqrt((r * r).mean()))
    return cur, mask


# ─────────────────────────────────────────────── 航向矫正

def correct_heading(xyz: np.ndarray, pct: float = 15.0,
                    min_correction: float = 0.005
                    ) -> Tuple[np.ndarray, float, np.ndarray]:
    """
    把车长轴摆到 Y 轴。

    做法：水平投影后做 PCA，主方向即车长方向；
    用 pct 做双向分位裁剪，避免侧壁/地面噪声把主方向拉偏。

    返回 (矫正后的点云, 偏航角(弧度), 旋转矩阵)
        · 偏航角小于 min_correction 时**不做旋转**（与原版行为一致，
          避免在本来就很正的数据上引入浮点噪声）

    ▲ 能力边界（实测，见 tests/test_geometry.py::TestHeading）
        本函数是"小残差回正器"，**不是**通用航向估计器：
        大角度早就由 processing.yaw_offset_deg 矫正过，这里只抹平 ±3° 内的残差。
        它内部按 x 分位裁剪（去侧壁噪声），隐含假设 x 是车宽方向且点云已大致轴对齐。
        对**完全对称的矩形平板**，裁剪后形状仍关于 x 轴对称 → 主方向不可辨识
        （两特征值接近），实测 1°/2°/3° 的旋转都估成 0.00°。
        真实样本上残差 |yaw| < 3°（已验证），所以对管线没有影响。
        若现场确需估计大航向，别硬改这里 —— 应优先调 yaw_offset_deg；
        要换成通用估计器的话，请一并更新这段注释与对应单测。
    """
    # ★ 出口 dtype 必须**恒定**为 float64。
    #   以前只有"真的转了"才 `xyz @ R.T`（结果 float64），不转就原样返回 float32 —— 
    #   于是下游 detect_empty / evaluate_empty 拿到的点云精度随"这一帧转没转"变化，
    #   同一份数据在 |yaw| 跨过 min_correction 前后会给出略不同的内点数与占比。
    #   判定链在 min_inlier_ratio 门槛附近本来就敏感，不该再引入这种抖动。
    xyz64 = np.asarray(xyz, np.float64)
    if len(xyz) < 10:
        return xyz64, 0.0, np.eye(3, dtype=np.float64)
    lo, hi = np.percentile(xyz[:, 0], [pct, 100.0 - pct])
    m = (xyz[:, 0] >= lo) & (xyz[:, 0] <= hi)
    q = xyz[m][:, :2] if int(m.sum()) >= 10 else xyz[:, :2]
    c = q.mean(axis=0)
    d = q - c
    cov = d.T @ d / len(q)
    try:
        w, v = np.linalg.eigh(cov)
    except np.linalg.LinAlgError:
        return xyz64, 0.0, np.eye(3, dtype=np.float64)
    axis = v[:, int(np.argmax(w))]
    yaw = math.atan2(float(axis[0]), float(axis[1]))
    # 车轴无向，折到 (-90°, 90°]
    while yaw > math.pi / 2:
        yaw -= math.pi
    while yaw <= -math.pi / 2:
        yaw += math.pi
    if abs(yaw) < min_correction:
        return xyz64, 0.0, np.eye(3, dtype=np.float64)
    cs, sn = math.cos(-yaw), math.sin(-yaw)
    R = np.array([[cs, -sn, 0.0], [sn, cs, 0.0], [0.0, 0.0, 1.0]], np.float64)
    return np.ascontiguousarray(xyz64 @ R.T), yaw, R


def rotate_about_z(pts: np.ndarray, deg: float) -> np.ndarray:
    r = math.radians(deg)
    cs, sn = math.cos(r), math.sin(r)
    R = np.array([[cs, -sn, 0.0], [sn, cs, 0.0], [0.0, 0.0, 1.0]])
    return np.ascontiguousarray(pts @ R.T)
