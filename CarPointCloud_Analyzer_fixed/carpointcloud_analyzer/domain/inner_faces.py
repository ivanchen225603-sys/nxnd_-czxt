"""实测内壁面定位；密度峰仅用于搜索，不作为物理边界。

仅消费未补点的车辆局部 W/L/H 点。面内分格等权拟合，避免近端密度偏置；
薄梁、稀疏附件和没有表面峰的体积散点不能作为内壁。
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np

from carpointcloud_analyzer.domain.utils.cells import unique_cells
from carpointcloud_analyzer.domain.utils.planes import consolidate_plane, deterministic_plane_hypothesis


FACE_TOLERANCE_M = 0.025
FACE_CELL_M = 0.20
FACE_MIN_POINTS = 100
FACE_MIN_CELLS = 12
FACE_MIN_CONTRAST = 2.5
FACE_SEARCH_HALF_BAND_M = 0.35


@dataclass(frozen=True)
class ObservedFace:
    """axis 坐标 = coefficients @ [其余两轴, 1]。"""

    axis: int
    coefficients: np.ndarray
    cells: np.ndarray

    def at(self, first: float, second: float) -> float:
        """在指定面内坐标量取实测拟合面。"""
        return float(self.coefficients @ [first, second, 1.0])


def supported_faces(points: np.ndarray, axis: int) -> list[ObservedFace]:
    """提取最多三块有二维覆盖和密度峰支持的近轴向平面。"""
    return peel_supported_faces(points, axis, 3)[0]


def peel_supported_faces(
    points: np.ndarray, axis: int, max_passes: int,
) -> tuple[list[ObservedFace], np.ndarray]:
    """在固定原始支持域内连续剥离；返回实测面及尚未处理的点。"""
    if len(points) < FACE_MIN_POINTS:
        return [], points
    free = [i for i in range(3) if i != axis]
    original = points[np.lexsort(points.T[::-1])]
    remaining = original
    origin = np.min(original[:, free], axis=0)
    original_grid = np.floor((original[:, free] - origin) / FACE_CELL_M).astype(int)
    _, cell_ids = unique_cells(original_grid, return_inverse=True)
    full_span = np.ptp(original[:, free], axis=0)
    result = []
    for _ in range(max_passes):
        if len(remaining) < FACE_MIN_POINTS:
            break
        try:
            plane, ids = deterministic_plane_hypothesis(
                remaining, FACE_TOLERANCE_M, num_iterations=256
            )
            plane, mask = consolidate_plane(remaining, plane, ids, FACE_TOLERANCE_M)
        except ValueError:
            break
        candidate = remaining[mask]
        remaining = remaining[~mask]
        if len(candidate) < FACE_MIN_POINTS or abs(plane[axis]) < 0.985:
            continue
        cells = _cell_medians(candidate, free, origin)
        if len(cells) < FACE_MIN_CELLS:
            continue
        # 只覆盖一条横梁/立柱的点，不能推断整面墙。
        spans = np.ptp(cells[:, free], axis=0)
        if np.any(spans < np.maximum(0.4, full_span * 0.5)):
            continue
        grid = np.floor((cells[:, free] - origin) / FACE_CELL_M).astype(int)
        if any(len(np.unique(grid[:, i])) < 3 for i in range(2)):
            continue
        if any(len(np.unique(grid[:, i])) < 0.7 * (np.ptp(grid[:, i]) + 1)
               for i in range(2)):
            # 顶梁和低位壁面可能被一块斜平面串起，但中部没有任何实测支持。
            continue
        in_plane = cells[:, free] - np.mean(cells[:, free], axis=0)
        _, _, axes = np.linalg.svd(in_plane, full_matrices=False)
        if np.ptp(in_plane @ axes[-1]) < 0.35:
            # 对角线可以同时跨过很多 W/H 格，却仍只是一根筋，不能冒充二维壁面。
            continue
        if len(cells) < 0.3 * np.prod(np.ptp(grid, axis=0) + 1):
            continue
        design = np.column_stack((cells[:, free], np.ones(len(cells))))
        coeff = np.linalg.lstsq(design, cells[:, axis], rcond=None)[0]
        if np.linalg.norm(coeff[:2]) > 0.175:
            continue
        residual = np.abs(original[:, axis] - np.column_stack(
            (original[:, free], np.ones(len(original)))
        ) @ coeff)
        # 拟合与峰值证据都按同一面内格等权，不能让密集扫描带替整面投票。
        # 每格在原有近面/侧带内归一化；没有观测的格不贡献证据。
        near_cells = np.bincount(cell_ids, weights=residual <= FACE_TOLERANCE_M)
        flank_cells = np.bincount(cell_ids, weights=(residual > FACE_TOLERANCE_M)
                                 & (residual <= 3 * FACE_TOLERANCE_M))
        total = near_cells + flank_cells
        occupied = total > 0
        near = np.sum(near_cells[occupied] / total[occupied])
        flank = np.sum(flank_cells[occupied] / total[occupied])
        # 比较同等厚度密度；均匀体积点即使可被 RANSAC 切出薄片，也没有表面峰。
        if 2.0 * near < FACE_MIN_CONTRAST * max(flank, 1):
            continue
        if np.percentile(np.abs(design @ coeff - cells[:, axis]), 90) > FACE_TOLERANCE_M:
            continue
        result.append(ObservedFace(axis, coeff, cells))
    return result, remaining


def _cell_medians(points: np.ndarray, free: list[int], origin: np.ndarray) -> np.ndarray:
    """按面内网格取中位点，拒绝仅有一两点的空洞边缘。"""
    grid = np.floor((points[:, free] - origin) / FACE_CELL_M).astype(int)
    _, ids, counts = unique_cells(grid, return_inverse=True, return_counts=True)
    # 稳定排序后每格的点仍按原顺序相邻，逐格切片与逐格布尔筛选取到同一组点。
    order = np.argsort(ids, kind='stable')
    stops = np.cumsum(counts)
    return np.array([
        np.median(points[order[stop - count:stop]], axis=0)
        for count, stop in zip(counts, stops) if count >= 4
    ]).reshape(-1, 3)


def measure_end_faces(
    points: np.ndarray, rear_core: float, front_core: float,
) -> Optional[tuple[ObservedFace, ObservedFace, float, float]]:
    """在两个端壁候选周围取主体实测面；返回统一的宽度/高度参考。"""
    bands = [points[np.abs(points[:, 1] - core) <= FACE_SEARCH_HALF_BAND_M]
             for core in (rear_core, front_core)]
    if any(len(band) < FACE_MIN_POINTS for band in bands):
        return None
    # 共同可见的主体高度，不把顶部内折梁或底板的横向密集条带当端壁。
    bounds = [np.percentile(band[:, [0, 2]], [5, 80], axis=0) for band in bands]
    w_lo = max(bound[0, 0] for bound in bounds)
    w_hi = min(np.percentile(band[:, 0], 95) for band in bands)
    h_lo = max(bound[0, 1] for bound in bounds)
    h_hi = min(bound[1, 1] for bound in bounds)
    if w_hi - w_lo < 0.8 or h_hi - h_lo < 0.5:
        return None
    w_ref, h_ref = (w_lo + w_hi) / 2, (h_lo + h_hi) / 2
    chosen = []
    for band, inward in zip(bands, (1, -1)):
        area = band[(band[:, 0] >= w_lo) & (band[:, 0] <= w_hi)
                    & (band[:, 2] >= h_lo) & (band[:, 2] <= h_hi)]
        faces = supported_faces(area, 1)
        if not faces:
            return None
        # 两个可见平行面都具备主体覆盖时，取面向厢内的一面，不平均壁厚。
        max_cells = max(len(face.cells) for face in faces)
        faces = [face for face in faces if len(face.cells) >= max_cells * 0.5]
        chosen.append(max(faces, key=lambda face: inward * face.at(w_ref, h_ref)))
    return chosen[0], chosen[1], w_ref, h_ref
