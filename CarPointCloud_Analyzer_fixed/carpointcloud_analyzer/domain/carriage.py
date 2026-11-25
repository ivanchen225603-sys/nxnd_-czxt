"""车厢内部剖面分析：底板面、两侧壁高、装载填充率、点云质量。

本模块是"车厢内部长什么样"的单一事实来源，供三处共用，避免各自重算导致口径分歧：
    * ``volume.LoadStatusClassifier`` —— 无基准时按填充率判空车/载货；
    * ``carriage.repair_low_wall``    —— 空车时用高侧壁镜像修补扫描不全的低侧；
    * ``dimensions.DimensionCalculator`` —— 车厢高、两侧壁高、空容积。

坐标约定（与 geometry.py 的局部坐标一致，legacy 源坐标系同构）：
    axis 0 = W 车宽，axis 1 = L 车长，axis 2 = H 高度。

实测经验（均来自真实采集数据，改动前请先理解这些约束）：
    * 底板面不能用低分位数：底板点层因测距噪声有 ±5cm 厚度，低分位会咬在
      层的下沿使壁深偏大约 5cm。改用最低密集层的密度峰中位数。
    * 两侧壁高必须分侧测量：雷达单侧安装时一侧扫不全、壁顶点缺失，该侧测值
      系统性偏低（实测差 0.39m）。两侧栏板物理等高，故真实壁高取扫描完整的
      较高侧，低侧标记为 scan_incomplete_side。
    * 填充率的分母必须用壁顶而非整体分位数：载货时整体 98 分位会取到货面。
    * 长度方向的稀疏统计用于识别远端点缺失（雷达远端衰减），据此给出
      measurement_confidence，让下游知道该次测量是否需要人工复核。
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from scipy.spatial import cKDTree

from carpointcloud_analyzer.domain.box_geometry import (
    assign_box_faces,
    find_end_wall_centers,
    mask_points_away_from,
    mask_uncovered_on_faces,
    rotate_wl,
)
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.longitudinal_measurement import _side_candidates
from carpointcloud_analyzer.domain.models import BoxCarriageBox, BoxCompletionResult, CarriageSplitResult
from carpointcloud_analyzer.domain.wall_patch import complete_end_patch, complete_side_end_strip
from carpointcloud_analyzer.domain.runtime import logger
from carpointcloud_analyzer.domain.utils import occupancy_extent
from carpointcloud_analyzer.domain.utils.cells import unique_cells


@dataclass
class CarriageProfile:
    """车厢内部剖面。所有高度均为局部坐标绝对值（非相对底板）。"""

    floor_z: float                          # 底板面高度
    wall_top_left: Optional[float]           # W 负方向一侧壁顶
    wall_top_right: Optional[float]          # W 正方向一侧壁顶
    wall_top: float                          # 取扫描完整的较高侧
    wall_depth: float                        # 壁深 = wall_top - floor_z
    wall_top_diff: Optional[float]           # 两侧壁顶差
    scan_incomplete_side: Optional[str]      # 扫描不全的一侧：'left' / 'right' / None
    interior_w_range: Tuple[float, float]    # 内部区域的 W 区间
    interior_mask: np.ndarray                # 相对 carriage_points 的内部掩码
    # 填充率三指标（空车判别用）
    median_fill_ratio: float
    high_cell_fraction: float
    mid_band_density: float
    n_grid_cells: int
    # 点云质量
    sparse_bin_fraction: float
    edge_sparse_length: Tuple[float, float]
    measurement_confidence: str              # 测量置信度：high / medium / low
    warnings: List[str] = field(default_factory=list)
    dense_bin_mask: Optional[np.ndarray] = None
    bin_index: Optional[np.ndarray] = None
    valid: bool = True                       # 内部点过少时为 False

    def to_quality_dict(self) -> Dict[str, Any]:
        """将点云质量指标转换为 API/JSON 使用的驼峰字段。

        Returns:
            包含稀疏比例、两端缺失长度、置信度和告警的字典。
        """
        return {
            "sparseBinFraction": round(self.sparse_bin_fraction, 3),
            "edgeSparseLengthM": [round(v, 2) for v in self.edge_sparse_length],
            "measurementConfidence": self.measurement_confidence,
            "wallScanIncompleteSide": self.scan_incomplete_side,
            "wallTopDiffM": (
                round(self.wall_top_diff, 3) if self.wall_top_diff is not None else None
            ),
            "warnings": list(self.warnings),
        }


def _lowest_strong_plane(z: np.ndarray, bin_w: float, min_fraction: float = 0.02) -> float:
    """最低的强水平面高度。

    Args:
        z: 高度数组。
        bin_w: 直方图 bin 宽（m）。
        min_fraction: 认定为"强"所需的最小点数占比。

    Returns:
        最低强峰的高度；没有任何峰达标时退回 5 分位。

    Notes:
        这里只需要一个"地面在哪"的下界参照，用来把磅面/路面整层抬掉；
        它不参与任何对外尺寸，所以取到磅面还是取到旁边低 0.7m 的路面都不影响
        壁深（差值）的正确性。
    """
    if len(z) == 0:
        return 0.0
    edges = np.arange(z.min(), z.max() + bin_w, bin_w)
    if len(edges) < 2:
        return float(z.min())
    counts, _ = np.histogram(z, bins=edges)
    strong = np.where(counts >= len(z) * min_fraction)[0]
    if len(strong) == 0:
        return float(np.percentile(z, 5))
    return float(edges[strong[0]] + bin_w / 2)


def anchor_carriage_walls(
    carriage_points: np.ndarray, config: Config
) -> Optional[Tuple[float, float, float, float]]:
    """按物理特征锚定两道侧壁及车厢纵向范围。

    Args:
        carriage_points: 局部 W/L/H 坐标系下的车厢候选点云。
        config: 提供锚定 bin 宽、门槛与车宽先验。

    Returns:
        ``(w_lo, w_hi, l_lo, l_hi)``；无法可靠锚定时返回 ``None``。

    Notes:
        两道侧壁是贯穿车厢的连续竖直面，抬离地面后它们是横向最稠密的两列 ——
        这个特征不依赖"点集边界就是车厢边界"的假设，因此簇里混进台面构筑物
        也不会被带偏。纵向范围再由壁上点"既稠密又高"的连续段界定，把车头
        （矮而稀疏）和更远的构筑物排除在外。
    """
    if len(carriage_points) < config.carriage_interior_min_points:
        return None
    w, length, h = (
        carriage_points[:, 0], carriage_points[:, 1], carriage_points[:, 2]
    )
    bin_w = config.carriage_anchor_bin
    deck = _lowest_strong_plane(h, bin_w)
    body = h > deck + config.carriage_anchor_deck_clearance
    if body.sum() < config.carriage_interior_min_points:
        return None
    bw, bl, bh = w[body], length[body], h[body]

    # --- 两道侧壁：横向最稠密的两列 ---
    edges = np.arange(bw.min(), bw.max() + bin_w, bin_w)
    if len(edges) < 3:
        return None
    counts, _ = np.histogram(bw, bins=edges)
    centers = edges[:-1] + bin_w / 2
    dense = centers[counts >= counts.max() * config.carriage_anchor_wall_density_ratio]
    if len(dense) < 2:
        return None
    w_lo, w_hi = float(dense.min()), float(dense.max())
    width_lo, width_hi = config.carriage_anchor_width_range
    if not width_lo <= w_hi - w_lo <= width_hi:
        return None

    # --- 纵向范围：壁上点既稠密又高的最长连续段 ---
    band = config.rib_wall_band
    on_wall = (np.abs(bw - w_lo) <= band) | (np.abs(bw - w_hi) <= band)
    if on_wall.sum() < config.carriage_interior_min_points:
        return None
    wall_l, wall_h = bl[on_wall], bh[on_wall]
    step = max(bin_w * 5, 0.2)
    l_edges = np.arange(wall_l.min(), wall_l.max() + step, step)
    if len(l_edges) < 3:
        return None
    l_centers = l_edges[:-1] + step / 2
    tops = np.full(len(l_centers), -np.inf)
    populations = np.zeros(len(l_centers))  # 仅用于 15 点最低样本量判断
    for i, center in enumerate(l_centers):
        seg = (wall_l >= center - step / 2) & (wall_l < center + step / 2)
        populations[i] = seg.sum()
        if seg.sum() >= 15:
            tops[i] = np.percentile(wall_h[seg], 98.0)
    measured = np.isfinite(tops)
    if not measured.any():
        return None
    # 只用高度判据：车头/底盘的壁带点顶多到 2m 出头，车厢栏板顶在 4m 以上，
    # 两者分得很开。再叠一个点数门槛会在输入本就干净时把车厢两端削掉
    # （实测把 5.4m 车厢削成 2.5m），而稀疏本身正是远端衰减的正常现象。
    top_threshold = deck + (tops[measured].max() - deck) * config.carriage_anchor_wall_height_ratio
    keep = measured & (tops >= top_threshold)

    best: Optional[Tuple[int, int]] = None
    start: Optional[int] = None
    for i, flag in enumerate(np.append(keep, False)):
        if flag and start is None:
            start = i
        elif not flag and start is not None:
            if best is None or i - start > best[1] - best[0]:
                best = (start, i)
            start = None
    if best is None:
        return None
    l_lo = float(l_centers[best[0]] - step / 2)
    l_hi = float(l_centers[best[1] - 1] + step / 2)
    if l_hi - l_lo < config.carriage_anchor_min_length:
        return None
    return w_lo, w_hi, l_lo, l_hi


def floor_surface(interior_z: np.ndarray) -> float:
    """估计底板（或货面）表面高度：最低 0.6m 区间内密度峰所在层的中位数。

    低分位数（如 p5）会咬在点层下沿导致壁深偏大；密度峰更贴近物理表面，
    且对底板下方的少量穿透点不敏感。
    """
    z_low = float(np.percentile(interior_z, 1.0))
    band = interior_z[interior_z < z_low + 0.6]
    if len(band) < 50:
        return float(np.percentile(interior_z, 5.0))
    hist, edges = np.histogram(band, bins=12)
    peak = int(np.argmax(hist))
    layer = band[(band >= edges[peak] - 0.05) & (band <= edges[peak + 1] + 0.05)]
    return float(np.median(layer)) if len(layer) else float(np.percentile(interior_z, 5.0))


def spatially_balanced_cloud(points: np.ndarray, config: Config) -> np.ndarray:
    """最终云按细空间格等权取样，重复补点不能改变载荷表面的票数。"""
    values = np.unique(np.asarray(points, dtype=float), axis=0)
    return _voxel_keep_first(values, config.carriage_grid_cell / 4)


class CarriageAnalyzer:
    """从车厢点云构建 :class:`CarriageProfile`。"""

    def __init__(self, config: Config):
        """初始化车厢内部剖面分析器。

        Args:
            config: 网格、壁带、稀疏判断和最少点数等算法配置。
        """
        self.config = config

    def analyze(self, carriage_points: np.ndarray) -> CarriageProfile:
        """估计底板、两侧壁顶、内部填充率和沿车长点云质量。

        Args:
            carriage_points: 局部 W/L/H 坐标系下形状为 ``(N, 3)`` 的车厢点云。

        Returns:
            供空载判断、尺寸计算和侧壁修补共同使用的车厢剖面。

        Notes:
            内部有效点不足时不会抛异常，而是返回 ``valid=False`` 的降级剖面。
        """
        cfg = self.config
        # 默认口径是横向分位数即壁位。既有车型一直稳定走这条路径，所以它必须
        # 先跑，并且在结果物理上成立时原样返回 —— 侧壁锚定只是它失效后的兜底，
        # 不改变任何一次本来就正确的测量。
        profile = self._profile_from_bounds(carriage_points, None)
        if not cfg.carriage_anchor_enable or self._profile_is_plausible(profile):
            return profile

        anchor = anchor_carriage_walls(carriage_points, cfg)
        if anchor is None:
            logger.warning(
                "默认剖面壁深 %.3fm 物理上不成立，且两道侧壁未能锚定"
                "（横向无成对稠密列或间距超出 %.1f~%.1fm），剖面不可信",
                profile.wall_depth, *cfg.carriage_anchor_width_range,
            )
            return profile

        w_lo, w_hi, l_lo, l_hi = anchor
        band = cfg.rib_wall_band
        inside = (
            (carriage_points[:, 1] >= l_lo) & (carriage_points[:, 1] <= l_hi)
            & (carriage_points[:, 0] >= w_lo - band)
            & (carriage_points[:, 0] <= w_hi + band)
        )
        if inside.sum() < cfg.carriage_interior_min_points:
            return profile
        retry = self._profile_from_bounds(carriage_points[inside], (w_lo, w_hi))
        if not self._profile_is_plausible(retry):
            return profile
        logger.warning(
            "默认剖面壁深 %.3fm 物理上不成立，改用侧壁锚定："
            "W=%.2f/%.2f（外沿宽 %.2fm），L=%.2f~%.2f（%.2fm），壁深 %.3fm",
            profile.wall_depth, w_lo, w_hi, w_hi - w_lo,
            l_lo, l_hi, l_hi - l_lo, retry.wall_depth,
        )
        return retry

    def _profile_is_plausible(self, profile: CarriageProfile) -> bool:
        """剖面是否可能来自一节真实车厢。

        Args:
            profile: 待判定的车厢剖面。

        Returns:
            壁深落在物理区间内且剖面有效时为 ``True``。

        Notes:
            只看壁深：它是失效时最先崩掉的量（实测 0.044m，实际 2.72m），
            判据越窄，误触发兜底、改动既有车型行为的机会越小。
        """
        low, high = self.config.sanity_vehicle_height
        return bool(profile.valid and low <= profile.wall_depth <= high)

    def _profile_from_bounds(
        self,
        carriage_points: np.ndarray,
        bounds: Optional[Tuple[float, float]],
    ) -> CarriageProfile:
        """在给定的两侧壁位置下计算剖面。

        Args:
            carriage_points: 局部 W/L/H 坐标系下的车厢点云。
            bounds: 两侧壁的 W 坐标；``None`` 表示沿用横向 2/98 分位数。

        Returns:
            车厢剖面。
        """
        cfg = self.config
        band = cfg.rib_wall_band
        if bounds is None:
            w_lo, w_hi = np.percentile(carriage_points[:, 0], [2.0, 98.0])
        else:
            w_lo, w_hi = bounds

        width, height = carriage_points[:, 0], carriage_points[:, 2]
        inner_lo, inner_hi = w_lo + band, w_hi - band
        interior_mask = (width > inner_lo) & (width < inner_hi)
        if interior_mask.sum() < cfg.carriage_interior_min_points or inner_hi <= inner_lo:
            logger.warning("车厢内部有效点不足，剖面分析降级")
            return self._degraded(carriage_points, (inner_lo, inner_hi), interior_mask)

        interior_z = height[interior_mask]
        floor_z = floor_surface(interior_z)

        # --- 两侧壁顶分侧测量 ---
        def _side_top(mask: np.ndarray) -> Optional[float]:
            """计算单侧具有足够点数时的稳健壁顶高度。

            Args:
                mask: 相对于当前车厢点云的单侧布尔掩码。

            Returns:
                该侧高度的 98 分位数；少于 30 点时返回 ``None``。
            """
            return float(np.percentile(height[mask], 98.0)) if mask.sum() >= 30 else None

        top_left = _side_top(width <= inner_lo)
        top_right = _side_top(width >= inner_hi)
        tops = [t for t in (top_left, top_right) if t is not None]
        # 两侧栏板物理等高：测值偏低的一侧是雷达扫描不全（点缺失只会让测量
        # 偏低不会偏高），故真实壁顶取较高侧
        wall_top = max(tops) if tops else float(np.percentile(height, 98.0))
        wall_diff = (
            abs(top_left - top_right)
            if top_left is not None and top_right is not None else None
        )
        incomplete_side = None
        warnings: List[str] = []
        if wall_diff is not None and wall_diff > cfg.carriage_wall_diff_tolerance:
            incomplete_side = "left" if top_left < top_right else "right"
            message = (
                f"两侧壁高测量不一致（左 {top_left - floor_z:.2f}m / "
                f"右 {top_right - floor_z:.2f}m，差 {wall_diff:.2f}m）："
                f"判定为雷达对 {incomplete_side} 侧扫描不全，按扫描完整的较高侧取值"
            )
            warnings.append(message)
            logger.warning(message)

        wall_depth = max(wall_top - floor_z, 1e-6)

        # --- 沿车长的点云质量（远端缺失检测）---
        interior = carriage_points[interior_mask]
        quality = self._length_quality(interior[:, 1], cfg.carriage_quality_bin)
        warnings.extend(quality["warnings"])
        for message in quality["warnings"]:
            logger.warning(message)

        # --- 填充率三指标 ---
        metrics = self._fill_metrics(interior, floor_z, wall_depth)

        return CarriageProfile(
            floor_z=floor_z,
            wall_top_left=top_left,
            wall_top_right=top_right,
            wall_top=wall_top,
            wall_depth=float(wall_depth),
            wall_top_diff=wall_diff,
            scan_incomplete_side=incomplete_side,
            interior_w_range=(float(inner_lo), float(inner_hi)),
            interior_mask=interior_mask,
            median_fill_ratio=metrics["median_fill"],
            high_cell_fraction=metrics["high_cell_fraction"],
            mid_band_density=metrics["mid_band_density"],
            n_grid_cells=metrics["n_cells"],
            sparse_bin_fraction=quality["sparse_fraction"],
            edge_sparse_length=quality["edge_sparse_length"],
            measurement_confidence=quality["confidence"],
            warnings=warnings,
            dense_bin_mask=quality["dense_mask"],
            bin_index=quality["bin_index"],
            valid=metrics["n_cells"] > 0,
        )

    # ------------------------------------------------------------------
    def _fill_metrics(
        self, interior: np.ndarray, floor_z: float, wall_depth: float
    ) -> Dict[str, float]:
        """车厢内部俯视网格的装载填充率指标。

        逐格取表面高（90 分位），归一化填充率 = (表面 - 底板) / 壁深。
        """
        cell = self.config.carriage_grid_cell
        l_index = ((interior[:, 1] - interior[:, 1].min()) / cell).astype(int)
        w_index = ((interior[:, 0] - interior[:, 0].min()) / cell).astype(int)
        key = l_index.astype(np.int64) * 100000 + w_index
        order = np.argsort(key, kind="stable")
        key_sorted = key[order]
        z_sorted = interior[order, 2]
        boundaries = np.flatnonzero(np.diff(key_sorted)) + 1
        fills = []
        for chunk in np.split(z_sorted, boundaries):
            if len(chunk) < 3:
                continue
            surface = np.percentile(chunk, 90.0)
            fills.append(np.clip((surface - floor_z) / wall_depth, 0.0, 1.2))
        if not fills:
            return {
                "median_fill": 0.0, "high_cell_fraction": 0.0,
                "mid_band_density": 0.0, "n_cells": 0,
            }
        fills = np.asarray(fills)
        normalized = (interior[:, 2] - floor_z) / wall_depth
        return {
            "median_fill": float(np.median(fills)),
            "high_cell_fraction": float((fills > 0.5).mean()),
            "mid_band_density": float(((normalized > 0.2) & (normalized < 0.8)).mean()),
            "n_cells": int(len(fills)),
        }

    def _length_quality(self, length: np.ndarray, bin_width: float) -> Dict[str, Any]:
        """沿车长方向的点密度质量：稀疏占比、两端连续缺失长度、置信度。"""
        l_min, l_max = float(length.min()), float(length.max())
        n_bins = max(4, int(np.ceil((l_max - l_min) / bin_width)))
        index = np.clip(((length - l_min) / bin_width).astype(int), 0, n_bins - 1)
        counts = np.bincount(index, minlength=n_bins)
        reference = float(np.median(counts[counts > 0])) if (counts > 0).any() else 0.0
        sparse = counts < max(3, reference * 0.2)

        def _run_length(flags: np.ndarray) -> float:
            """把布尔数组起始处连续稀疏 bin 数换算为物理长度。

            Args:
                flags: 从待检查边缘向内部排列的稀疏标记。

            Returns:
                连续稀疏区长度，单位为米。
            """
            total = 0
            for flag in flags:
                if not flag:
                    break
                total += 1
            return total * bin_width

        head_missing = _run_length(sparse)
        tail_missing = _run_length(sparse[::-1])
        sparse_fraction = float(sparse.mean())
        if sparse_fraction < 0.10:
            confidence = "high"
        elif sparse_fraction < 0.30:
            confidence = "medium"
        else:
            confidence = "low"
        warnings: List[str] = []
        worst = max(head_missing, tail_missing)
        if worst >= 0.6:
            warnings.append(
                f"车厢一端存在约 {worst:.1f}m 连续稀疏/缺失区，"
                "长度与容积按占用跨度估计"
            )
        if sparse_fraction >= 0.30:
            warnings.append(f"车厢稀疏区占比 {sparse_fraction:.0%}，测量置信度低")
        return {
            "sparse_fraction": sparse_fraction,
            "edge_sparse_length": (head_missing, tail_missing),
            "confidence": confidence,
            "warnings": warnings,
            "dense_mask": ~sparse,
            "bin_index": index,
        }

    def _degraded(
        self,
        carriage_points: np.ndarray,
        w_range: Tuple[float, float],
        interior_mask: np.ndarray,
    ) -> CarriageProfile:
        """内部点不足时的降级剖面：仍给出可用的底板/壁顶，但标记 valid=False。"""
        height = carriage_points[:, 2]
        floor_z = floor_surface(height)
        wall_top = float(np.percentile(height, 98.0))
        return CarriageProfile(
            floor_z=floor_z,
            wall_top_left=None,
            wall_top_right=None,
            wall_top=wall_top,
            wall_depth=max(wall_top - floor_z, 1e-6),
            wall_top_diff=None,
            scan_incomplete_side=None,
            interior_w_range=w_range,
            interior_mask=interior_mask,
            median_fill_ratio=0.0,
            high_cell_fraction=0.0,
            mid_band_density=0.0,
            n_grid_cells=0,
            sparse_bin_fraction=1.0,
            edge_sparse_length=(0.0, 0.0),
            measurement_confidence="low",
            warnings=["车厢内部有效点不足，剖面分析降级"],
            valid=False,
        )


def repair_low_wall(
    carriage_points: np.ndarray, profile: CarriageProfile, config: Config
) -> Tuple[np.ndarray, Dict[str, Any]]:
    """用扫描完整的高侧壁镜像生成低侧壁候选；不作空载证明。

    车厢两侧栏板物理等高，一侧测值偏低是雷达扫描几何导致该侧壁顶点缺失。
    以两侧壁中心线的中点为对称面，把高侧壁上高于低侧现有壁顶的点镜像到
    低侧 —— 镜像点保留壁顶沿车长方向的真实起伏，比整体拉伸更接近实际结构。

    Returns:
        (合成的新点云(M,3)，说明字典)。无需修补时返回空数组。
    """
    empty: Tuple[np.ndarray, Dict[str, Any]] = (np.empty((0, 3)), {"repaired": False})
    top_left, top_right = profile.wall_top_left, profile.wall_top_right
    if top_left is None or top_right is None:
        return empty
    if abs(top_left - top_right) <= config.carriage_wall_diff_tolerance:
        return empty

    width = carriage_points[:, 0]
    inner_lo, inner_hi = profile.interior_w_range
    low_is_left = top_left < top_right
    low_band = carriage_points[width <= inner_lo] if low_is_left else carriage_points[width >= inner_hi]
    high_band = carriage_points[width >= inner_hi] if low_is_left else carriage_points[width <= inner_lo]
    if len(low_band) < 50 or len(high_band) < 50:
        return empty

    mirror_axis = (float(np.median(low_band[:, 0])) + float(np.median(high_band[:, 0]))) / 2.0

    # 低侧逐 bin 壁顶：只镜像高于低侧现有壁顶的高侧点，不与已有点重叠
    bin_width = config.carriage_quality_bin
    l_min = float(carriage_points[:, 1].min())
    low_index = ((low_band[:, 1] - l_min) / bin_width).astype(int)
    low_top_by_bin: Dict[int, float] = {}
    for b in np.unique(low_index):
        zb = low_band[low_index == b, 2]
        if len(zb) >= 5:
            low_top_by_bin[int(b)] = float(np.percentile(zb, 98.0))
    global_low_top = min(top_left, top_right)

    high_index = ((high_band[:, 1] - l_min) / bin_width).astype(int)
    threshold = np.array([low_top_by_bin.get(int(b), global_low_top) for b in high_index])
    donor = high_band[high_band[:, 2] > threshold + 0.02]
    if len(donor) == 0:
        return empty

    mirrored = donor.copy()
    mirrored[:, 0] = 2.0 * mirror_axis - donor[:, 0]
    side = "left" if low_is_left else "right"
    logger.info(
        "低侧壁镜像修补：%s 侧补 %d 点（两侧壁顶差 %.2fm）",
        side, len(mirrored), abs(top_left - top_right),
    )
    return mirrored, {
        "repaired": True,
        "side": side,
        "synthesizedPoints": int(len(mirrored)),
        "wallTopDiffBefore": round(float(abs(top_left - top_right)), 3),
    }


@dataclass(frozen=True)
class WallCompletionResult:
    """车厢壁面候选补点；实测点独立保留，坐标为车辆局部 W/L/H。"""

    points: np.ndarray
    side_points: np.ndarray
    front_points: np.ndarray
    rear_points: np.ndarray
    metadata: Dict[str, Any]


def _tail_anchored_rear_plane(split: CarriageSplitResult) -> Optional[Tuple[float, float, float]]:
    """后内壁仅在紧贴本次整车尾端时，才作为补点选面的参考。"""
    vehicle_length = float(split.head_y - split.tail_y)
    rear_tail_gap = float(split.carriage_start_y - split.tail_y)
    if (split.rear_wall_plane is None or not split.boundaries_are_inner_faces
            or not np.isfinite([vehicle_length, rear_tail_gap]).all()
            or vehicle_length <= 0
            or not 0 <= rear_tail_gap <= min(0.25, 0.03 * vehicle_length)):
        return None
    return split.rear_wall_plane


def complete_empty_carriage_walls(
    vehicle_points: np.ndarray,
    carriage_points: np.ndarray,
    split: CarriageSplitResult,
    profile: CarriageProfile,
    config: Config,
    *,
    bridge_long_side_gaps: bool | frozenset[tuple[str, str]] = False,
) -> WallCompletionResult:
    """按车厢纵向中心面对称补全侧壁、前壁和后壁。

    函数名保留兼容；调用方在载荷判断之前生成候选，并检查端壁内表面和
    剖面质量。合成点不能提供判空证据。侧壁沿用“高侧补低侧”；前后壁只镜像端壁带内、底板
    以上的实测点，并在 3-D 邻域内去重。这样不会复制车头、底板或环境点。
    ``bridge_long_side_gaps`` 指定顶部断线的（侧壁，端壁）。双端壁均有实测面、
    同高侧壁有连续回波，才从该端沿侧壁连接扫描盲段。旧调用传 True 时检查所有端点。
    """
    empty_points = np.empty((0, 3), dtype=float)
    skipped = WallCompletionResult(
        points=empty_points,
        side_points=empty_points,
        front_points=empty_points,
        rear_points=empty_points,
        metadata={"completed": False, "reason": "quality_gate_not_met"},
    )
    if not split.boundaries_are_inner_faces or not profile.valid:
        return skipped

    vehicle = np.asarray(vehicle_points, dtype=float)
    carriage = np.asarray(carriage_points, dtype=float)
    if (
        vehicle.ndim != 2
        or carriage.ndim != 2
        or vehicle.shape[1:] != (3,)
        or carriage.shape[1:] != (3,)
        or len(vehicle) == 0
        or len(carriage) == 0
        or not np.all(np.isfinite(vehicle))
        or not np.all(np.isfinite(carriage))
    ):
        return skipped

    mirror_axis = _wall_symmetry_axis(carriage, profile)
    if mirror_axis is None:
        return WallCompletionResult(
            points=empty_points,
            side_points=empty_points,
            front_points=empty_points,
            rear_points=empty_points,
            metadata={"completed": False, "reason": "side_wall_axis_unavailable"},
        )

    side_points = empty_points
    side_info: Dict[str, Any] = {"repaired": False}
    if config.carriage_repair_low_wall:
        side_points, side_info = repair_low_wall(carriage, profile, config)

    base = np.vstack([vehicle, side_points]) if len(side_points) else vehicle.copy()
    side_source = np.vstack((carriage, side_points)) if len(side_points) else carriage
    side_planes = {
        "left" if sign < 0 else "right": face.coefficients
        for _, _, sign, face in _side_candidates(
            side_source, float(split.carriage_start_y), float(split.split_y),
            profile.floor_z, profile.wall_top,
            min_coverage=.20, min_length_coverage=.50,
        )
    }
    tree = cKDTree(base)
    end_results: Dict[str, np.ndarray] = {}
    end_metadata: Dict[str, Dict[str, Any]] = {}
    vehicle_length = float(split.head_y - split.tail_y)
    rear_tail_gap = float(split.carriage_start_y - split.tail_y)
    rear_plane = _tail_anchored_rear_plane(split)
    for name, boundary in (
        ("rearWall", float(split.carriage_start_y)),
        ("frontWall", float(split.split_y)),
    ):
        if not config.carriage_complete_end_walls:
            end_results[name] = empty_points
            end_metadata[name] = {
                "completed": False,
                "reason": "end_wall_completion_disabled",
                "observedPoints": 0,
                "synthesizedPoints": 0,
            }
            continue

        observed = _select_end_wall_points(
            vehicle,
            boundary,
            profile,
            config,
        )
        patch, model = complete_end_patch(
            observed, boundary, mirror_axis, profile, config,
            interior_sign=1 if name == "rearWall" else -1,
            side_planes=side_planes,
            verified_inner_plane=(
                rear_plane if name == "rearWall" else None
            ),
        )
        if name == "rearWall" and model.get("faceSelection") == "tail_anchored_inner_face":
            model["rearTailGapM"] = rear_tail_gap
            model["vehicleLengthM"] = vehicle_length
        if not model['valid']:
            added = empty_points
            reason = model['reason']
        else:
            # 只镜像有实测二维面支持且横向对称成立的壁点，不能复制贴壁散货。
            residual = np.abs(observed[:, 1] - np.c_[observed[:, [0, 2]],
                                                      np.ones(len(observed))]
                              @ model['coefficients'])
            mirrored = observed[residual <= .025].copy() if model['symmetryApplied'] else empty_points
            mirrored[:, 0] = 2.0 * mirror_axis - mirrored[:, 0]
            inner_lo, inner_hi = profile.interior_w_range
            width_lo = inner_lo - config.carriage_completion_width_margin
            width_hi = inner_hi + config.carriage_completion_width_margin
            mirrored = mirrored[
                (mirrored[:, 0] >= width_lo)
                & (mirrored[:, 0] <= width_hi)
            ]
            candidates = np.vstack([mirrored, patch])
            distances, _ = tree.query(candidates, k=1, workers=-1)
            accepted = distances > config.carriage_completion_dedup_radius
            model['acceptedPointCount'] = int(np.count_nonzero(accepted[len(mirrored):]))
            added = candidates[accepted]
            added = _voxel_keep_first(
                added,
                config.carriage_completion_voxel,
            )
            reason = "ok" if len(added) else "already_symmetric"

        end_results[name] = added
        end_metadata[name] = {
            "completed": bool(len(added)),
            "reason": reason,
            "observedPoints": int(len(observed)),
            "synthesizedPoints": int(len(added)),
            "innerFaceL": round(boundary, 6),
            "model": model,
        }
        if len(added):
            base = np.vstack([base, added])
            tree = cKDTree(base)

    corner_parts = []
    corner_info = []
    both_ends_modeled = all(
        end_metadata[name].get("model", {}).get("valid") is True
        for name in ("rearWall", "frontWall")
    )
    bridge_ends = () if isinstance(bridge_long_side_gaps, bool) else bridge_long_side_gaps
    for name, boundary, direction in (
        ("rearWall", float(split.carriage_start_y), 1),
        ("frontWall", float(split.split_y), -1),
    ):
        model = end_metadata[name].get("model", {})
        for side_name, plane in side_planes.items():
            strip, info = complete_side_end_strip(
                side_source, plane, model, boundary, direction,
                config.carriage_completion_end_wall_band,
                opposite_boundary_l=(
                    float(split.split_y if name == "rearWall" else split.carriage_start_y)
                    if both_ends_modeled and (bridge_long_side_gaps is True
                                              or (side_name, name) in bridge_ends) else None
                ),
            )
            if not len(strip):
                continue
            distances, _ = tree.query(strip, k=1, workers=-1)
            accepted = _voxel_keep_first(
                strip[distances > config.carriage_completion_dedup_radius],
                config.carriage_completion_voxel,
            )
            if len(accepted):
                corner_parts.append(accepted)
                corner_info.append({"end": name, "side": side_name,
                                    "synthesizedPoints": int(len(accepted)), **info})
                base = np.vstack((base, accepted))
                tree = cKDTree(base)

    rear_points = end_results["rearWall"]
    front_points = end_results["frontWall"]
    parts = [part for part in (side_points, front_points, rear_points, *corner_parts)
             if len(part)]
    completed_points = np.vstack(parts) if parts else empty_points
    metadata = {
        "completed": bool(len(completed_points)),
        "method": "left_right_symmetry_local_gap_fill",
        "symmetryAxisW": round(float(mirror_axis), 6),
        "synthesizedPoints": int(len(completed_points)),
        "sideWall": side_info,
        "frontWall": end_metadata["frontWall"],
        "rearWall": end_metadata["rearWall"],
        "cornerSeams": corner_info,
        "parameters": {
            "endWallHalfBandM": config.carriage_completion_end_wall_band,
            "dedupRadiusM": config.carriage_completion_dedup_radius,
            "voxelM": config.carriage_completion_voxel,
            "floorClearanceM": config.carriage_completion_floor_clearance,
        },
    }
    logger.info(
        "车厢候选对称补点：侧壁 %d、前壁 %d、后壁 %d、角线接缝 %d，共 %d 点",
        len(side_points),
        len(front_points),
        len(rear_points),
        sum(len(part) for part in corner_parts),
        len(completed_points),
    )
    return WallCompletionResult(
        points=completed_points,
        side_points=side_points,
        front_points=front_points,
        rear_points=rear_points,
        metadata=metadata,
    )


def _wall_symmetry_axis(
    carriage_points: np.ndarray,
    profile: CarriageProfile,
) -> Optional[float]:
    """用两侧壁稳健中位位置确定车厢纵向中心面 W 坐标。"""
    width = carriage_points[:, 0]
    inner_lo, inner_hi = profile.interior_w_range
    left = carriage_points[width <= inner_lo]
    right = carriage_points[width >= inner_hi]
    if len(left) < 50 or len(right) < 50:
        return None
    return (
        float(np.median(left[:, 0])) + float(np.median(right[:, 0]))
    ) / 2.0


def _select_end_wall_points(
    vehicle_points: np.ndarray,
    boundary_l: float,
    profile: CarriageProfile,
    config: Config,
) -> np.ndarray:
    """选取单块端壁附近的实测点，明确排除底板及端壁范围外结构。"""
    inner_lo, inner_hi = profile.interior_w_range
    return vehicle_points[
        (np.abs(vehicle_points[:, 1] - boundary_l)
         <= config.carriage_completion_end_wall_band)
        & (vehicle_points[:, 0]
           >= inner_lo - config.carriage_completion_width_margin)
        & (vehicle_points[:, 0]
           <= inner_hi + config.carriage_completion_width_margin)
        & (vehicle_points[:, 2]
           >= profile.floor_z + config.carriage_completion_floor_clearance)
        & (vehicle_points[:, 2]
           <= profile.wall_top + config.carriage_completion_height_margin)
    ]


def _voxel_keep_first(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """按体素稳定去重并保留原始镜像坐标，不移动合成点。"""
    if len(points) == 0:
        return points
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, first = unique_cells(keys, return_index=True)
    return points[np.sort(first)]


def complete_box_carriage(
    carriage_points: np.ndarray,
    config: Config,
) -> BoxCompletionResult:
    """厢式车 / 集装箱车厢的表面补全，输出接近规则的长方体。

    与 :func:`complete_empty_carriage_walls` 的分工：那一个靠"把观测到的一侧镜像
    到另一侧"补开顶栏板车，前提是至少有一侧被扫全。厢式车没有这个前提 —— 顶面和
    两个侧面可能同时只剩边缘。实测一台厢式货车（0907 数据，卷尺真值 6.06×2.55×2.90）
    各面覆盖密度：后端面 2605 点/m²、顶面 686、右侧面 471、前端面 255、左侧面 101，
    中段 6.5m 里有 2.5m 只剩一条 0.17m 宽的细线，**左侧壁在中段完全没有点**。

    重建策略：

    1. **先估偏航再重建。** 箱体相对输入 L 轴的偏航角会直接转成箱宽误差 —— 实测
       1.99° 偏航在 6m 箱长上产生 20.9cm 横向偏移，两端面的 W 区间因此错开，箱宽
       从 −3.7cm 恶化到 −18.7cm。函数内部先把点云旋到箱体自身轴上，最后再把合成
       点转回输入坐标，因此输出始终是一个规则长方体。
    2. **箱长**取两个端面峰的间距（实测 −6cm）。
    3. **箱宽**取两个端面占用跨度的均值。单取一侧会偏：实测后端面 2.635、前端面
       2.514，均值 2.575（+2.5cm）比交集（−3.6cm）和并集（+8.5cm）都准。
    4. **箱高的下沿取两个端面下沿中较高者**，因为污染只会向下延伸 —— 靠车头一侧的
       端面带混进底盘和轮子，Z 一直连到 0.30，比箱体低 1.2m。**上沿取所有观测面
       （含两侧壁）上沿中最高者**，因为遮挡只会向下截断：实测右侧壁顶 4.130、后端面
       4.142、前端面 4.071，取最高的一侧，与"两侧壁物理等高、按扫描完整的较高侧
       取值"是同一条现场约定。

    Args:
        carriage_points: 车厢点云，车辆局部 W/L/H 坐标（axis 0=W, 1=L, 2=H）。
            允许箱体相对 L 轴有小角度偏航，函数会自行对齐。
        config: 补全参数。

    Returns:
        ``BoxCompletionResult``。``points`` 是合成点，调用方必须与实测点分开记账：
        本项目的硬规则是合成点只进导出点云和人工复核，绝不参与尺寸、载荷或拉筋
        的计算，否则等于用合成数据反过来证明自身正确。

    Notes:
        箱底可能根本不可观测。实测该车 46 帧里箱体正下方全部为 0 点 —— 箱体自身
        完全遮挡了下方。取到的下沿 1.49 相对卷尺真值缺 25cm，这是观测能力的限制，
        调任何估计量都补不回来；``box.height`` 应当理解为下界。
    """
    empty = np.empty((0, 3), dtype=float)

    def skipped(reason: str) -> BoxCompletionResult:
        """条件不满足时返回空补全结果，由调用方保持原有流程。"""
        return BoxCompletionResult(empty, None, {"completed": False, "reason": reason})

    if not config.carriage_box_completion_enable:
        return skipped("box_completion_disabled")

    points = np.asarray(carriage_points, dtype=float)
    if points.ndim != 2 or points.shape[1:] != (3,) or len(points) < 100:
        return skipped("insufficient_points")
    if not np.all(np.isfinite(points)):
        return skipped("non_finite_points")

    yaw = _estimate_box_yaw(points, config)
    if yaw is None:
        return skipped("end_walls_not_found")
    aligned = rotate_wl(points, -yaw)

    box = _reconstruct_box_from_end_walls(aligned, config)
    if box is None:
        return skipped("end_walls_not_found")

    synthetic, face_ids = _resample_box_surfaces(box, config)
    if len(synthetic) == 0:
        return skipped("no_surface_generated")

    # 只保留"实测没有覆盖到"的位置。已经扫到的地方不需要合成点，重复堆叠既会
    # 破坏后续按点密度工作的算法（底板密度峰、拉筋找峰、occupancy_extent 门槛），
    # 也会在真墙旁边再糊一层对不齐的合成墙。
    owners = assign_box_faces(aligned, box, config.carriage_box_face_assign_band)
    covered = mask_uncovered_on_faces(
        synthetic, face_ids, aligned, owners,
        config.carriage_box_surface_dedup_radius,
    )
    synthetic, face_ids = synthetic[covered], face_ids[covered]

    synthetic = rotate_wl(synthetic, yaw)
    synthetic = synthetic[
        mask_points_away_from(
            synthetic, points, config.carriage_completion_dedup_radius
        )
    ]
    synthetic = _voxel_keep_first(synthetic, config.carriage_completion_voxel)

    logger.info(
        "厢式车厢补全：箱体 %.2f×%.2f×%.2fm，偏航 %+.2f°（前端面 %d 点 / 后端面 %d 点），"
        "合成 %d 点",
        box.length, box.width, box.height, np.degrees(yaw),
        box.front_points, box.rear_points, len(synthetic),
    )
    metadata = {
        "completed": True,
        "method": "end_wall_box_resample",
        "box": box.to_dict(),
        "yawDeg": round(float(np.degrees(yaw)), 3),
        "syntheticPoints": int(len(synthetic)),
        "observedPoints": int(len(points)),
        "surfaceSpacingM": config.carriage_box_surface_spacing,
        "dedupRadiusM": config.carriage_completion_dedup_radius,
        "faceDedupRadiusM": config.carriage_box_surface_dedup_radius,
    }
    return BoxCompletionResult(points=synthetic, box=box, metadata=metadata)


def _estimate_box_yaw(points: np.ndarray, config: Config) -> Optional[float]:
    """由两个端面的横向中心估计箱体相对 L 轴的偏航角（弧度）。"""
    centers = find_end_wall_centers(points, config)
    if centers is None:
        return None
    band = config.carriage_box_end_wall_band
    lateral = []
    for center in centers:
        wall = points[np.abs(points[:, 1] - center) <= band]
        if len(wall) < config.carriage_box_min_end_wall_points:
            return None
        w_lo, w_hi = occupancy_extent(wall[:, 0], bin_width=0.05, min_points=5)
        lateral.append((w_lo + w_hi) / 2.0)
    span = centers[1] - centers[0]
    if abs(span) < 1e-6:
        return None
    yaw = float(np.arctan2(lateral[1] - lateral[0], span))
    limit = np.deg2rad(config.carriage_box_max_yaw_deg)
    if abs(yaw) > limit:
        logger.warning(
            "箱体偏航 %.1f° 超过上限 %.1f°，按不旋转处理",
            np.degrees(yaw), config.carriage_box_max_yaw_deg,
        )
        return 0.0
    return yaw


def _reconstruct_box_from_end_walls(
    points: np.ndarray, config: Config
) -> Optional[BoxCarriageBox]:
    """用两个端面重建厢体长方体，上沿再并入较高一侧的侧壁。"""
    centers = find_end_wall_centers(points, config)
    if centers is None:
        return None

    band = config.carriage_box_end_wall_band
    walls = []
    for center in centers:
        wall = points[np.abs(points[:, 1] - center) <= band]
        if len(wall) < config.carriage_box_min_end_wall_points:
            return None
        walls.append(wall)

    # 横断面必须用占用跨度而不是分位数：箱体的顶边和底边采样天然稀疏，1~99 分位
    # 会把它们当噪声裁掉（实测同一面箱宽 占用 2.514 / 分位 2.428，真值 2.55）。
    spans = [
        (occupancy_extent(wall[:, 0], bin_width=0.05, min_points=5),
         occupancy_extent(wall[:, 2], bin_width=0.05, min_points=5))
        for wall in walls
    ]
    # 箱宽取两端面均值：单取一侧会偏（实测 2.635 / 2.514，均值 2.575，真值 2.55）。
    w_lo = float(np.mean([span[0][0] for span in spans]))
    w_hi = float(np.mean([span[0][1] for span in spans]))
    # 下沿取较高者：污染只向下延伸（靠车头一侧混进底盘，Z 连到 0.30）。
    h_lo = float(max(span[1][0] for span in spans))
    # 上沿取最高者：遮挡只向下截断，哪一面看得更高就以它为准。
    h_hi = float(max(span[1][1] for span in spans))

    # 再并入两侧壁各自观测到的顶：两侧壁物理等高，一侧测值偏低是该侧扫描不全。
    side_top = _tallest_side_wall_top(points, w_lo, w_hi, centers, config)
    if side_top is not None and side_top > h_hi:
        logger.info("箱体上沿由较高一侧的侧壁修正：%.3f → %.3f", h_hi, side_top)
        h_hi = side_top

    if w_hi - w_lo < 0.5 or h_hi - h_lo < 0.5:
        return None

    l_first, l_second = centers
    if l_first >= l_second:
        front_l, rear_l = l_first, l_second
        front_points, rear_points = len(walls[0]), len(walls[1])
    else:
        front_l, rear_l = l_second, l_first
        front_points, rear_points = len(walls[1]), len(walls[0])
    return BoxCarriageBox(
        front_l=front_l,
        rear_l=rear_l,
        w_range=(w_lo, w_hi),
        h_range=(h_lo, h_hi),
        front_points=int(front_points),
        rear_points=int(rear_points),
    )


def _tallest_side_wall_top(
    points: np.ndarray,
    w_lo: float,
    w_hi: float,
    centers: Tuple[float, float],
    config: Config,
) -> Optional[float]:
    """两侧壁中扫描更完整的一侧的顶高。

    实测该车左侧壁在中段一个点都没有、右侧壁顶 4.130；只看端面会漏掉这 6cm。
    """
    band = config.carriage_box_end_wall_band
    inner_lo = min(centers) + band
    inner_hi = max(centers) - band
    middle = points[(points[:, 1] > inner_lo) & (points[:, 1] < inner_hi)]
    if len(middle) < 50:
        return None
    tops = []
    for edge in (w_lo, w_hi):
        side = middle[np.abs(middle[:, 0] - edge) <= config.carriage_box_side_wall_band]
        if len(side) < 50:
            continue
        tops.append(occupancy_extent(side[:, 2], bin_width=0.05, min_points=5)[1])
    return max(tops) if tops else None


def _resample_box_surfaces(
    box: BoxCarriageBox, config: Config
) -> Tuple[np.ndarray, np.ndarray]:
    """在厢体的六个面上按固定间距重采样，得到一个闭合长方体。

    Returns:
        ``(points, face_ids)``；``face_ids`` 按 :data:`BOX_FACE_AXES` 编号，
        后续要按面做覆盖判定，必须知道每个合成点属于哪个面。
    """
    step = config.carriage_box_surface_spacing
    l_axis = np.arange(box.rear_l, box.front_l + step, step)
    w_axis = np.arange(box.w_range[0], box.w_range[1] + step, step)
    h_axis = np.arange(box.h_range[0], box.h_range[1] + step, step)
    if len(l_axis) < 2 or len(w_axis) < 2 or len(h_axis) < 2:
        return np.empty((0, 3), dtype=float), np.empty(0, dtype=int)

    faces: List[np.ndarray] = []
    ids: List[np.ndarray] = []

    def add(face_id: int, points: np.ndarray) -> None:
        """登记一个面的重采样点及其面编号。"""
        faces.append(points)
        ids.append(np.full(len(points), face_id, dtype=int))

    for face_id, w in enumerate(box.w_range):          # 0 左侧面 / 1 右侧面
        grid_l, grid_h = np.meshgrid(l_axis, h_axis, indexing="ij")
        add(face_id, np.column_stack([
            np.full(grid_l.size, w), grid_l.ravel(), grid_h.ravel()
        ]))
    for offset, l_value in enumerate((box.rear_l, box.front_l)):   # 2 后壁 / 3 前壁
        grid_w, grid_h = np.meshgrid(w_axis, h_axis, indexing="ij")
        add(2 + offset, np.column_stack([
            grid_w.ravel(), np.full(grid_w.size, l_value), grid_h.ravel()
        ]))
    for offset, h in enumerate(box.h_range):           # 4 底面 / 5 顶面
        grid_l, grid_w = np.meshgrid(l_axis, w_axis, indexing="ij")
        add(4 + offset, np.column_stack([
            grid_w.ravel(), grid_l.ravel(), np.full(grid_l.size, h)
        ]))
    return np.vstack(faces), np.concatenate(ids)
