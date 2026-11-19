"""按已定位的横向杆轴补齐遮挡段，保持原点坐标不变。"""

from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree

from carpointcloud_analyzer.domain.models import RibInfo
from carpointcloud_analyzer.domain.longitudinal_measurement import MAX_RIB_EXTRAPOLATION_M
from carpointcloud_analyzer.domain.rib_support import CELL_M, MIN_TRANSVERSE_AXIS_W, conflicting_rib_sides, profile_support_points, stable_axis_extension, validated_rib_support


RIB_SAMPLE_SPACING_M = 0.025
RIB_EXISTING_RADIUS_M = 0.07
RIB_MINIMUM_CLEARANCE_M = 0.012
RIB_WALL_CLEARANCE_M = 0.055
RIB_CROSS_SECTION_OFFSETS_M = (-0.008, 0.0, 0.008)


def complete_rib_gaps(
    points: np.ndarray,
    ribs: list[RibInfo],
    interior_w_range: tuple[float, float],
    rear_l: float,
    front_l: float,
    floor_h: float,
    top_h: float,
    *, observed_points: np.ndarray | None = None,
) -> tuple[np.ndarray, dict]:
    """只在有稳定横杆轴线的缺口插点，不覆盖已有扫描点。"""
    empty = np.empty((0, 3), dtype=float)
    bounds = np.asarray(interior_w_range, dtype=float)
    if (not ribs or len(points) == 0 or bounds.shape != (2,)
            or not np.isfinite(bounds).all() or bounds[1] - bounds[0] < .5
            or not np.isfinite([rear_l, front_l, floor_h, top_h]).all()
            or front_l <= rear_l or top_h <= floor_h):
        return empty, {'completed': False, 'pointCount': 0, 'ribs': []}
    tree = cKDTree(points)
    observed = np.unique(points if observed_points is None else observed_points, axis=0)
    observed_tree = cKDTree(observed)
    parts = []
    details = []
    w_values = np.arange(bounds[0] + RIB_WALL_CLEARANCE_M,
                         bounds[1] - RIB_WALL_CLEARANCE_M,
                         RIB_SAMPLE_SPACING_M)
    offsets = np.array([(0., dl, dh) for dl in RIB_CROSS_SECTION_OFFSETS_M
                        for dh in RIB_CROSS_SECTION_OFFSETS_M])
    for rib in ribs:
        midpoint = np.asarray(rib.axis_midpoint, dtype=float)
        direction = np.asarray(rib.axis_direction, dtype=float)
        support = np.asarray(rib.axis_observed_w_range, dtype=float)
        if (midpoint.shape != (3,) or direction.shape != (3,)
                or support.shape != (2,) or not np.isfinite([midpoint, direction]).all()
                or not np.isfinite(support).all() or support[1] - support[0] < .35
                or np.linalg.norm(direction) < 1e-6
                or abs(direction[0]) / np.linalg.norm(direction) < MIN_TRANSVERSE_AXIS_W):
            continue
        conflicts = conflicting_rib_sides(observed, observed_tree, rib, floor_h, top_h)
        diagnostics = {}
        model = validated_rib_support(observed, observed_tree, rib, floor_h, top_h, diagnostics=diagnostics)
        if model is None:
            details.append({'ribId': rib.rib_id, 'pointCount': 0,
                            'conflictingSides': sorted(conflicts),
                            'reason': 'whole_rib_axis_unresolved', 'supportValidation': diagnostics})
            continue
        centers = model.evaluate(w_values)
        valid = ((centers[:, 1] > rear_l + .12) & (centers[:, 1] < front_l - .12)
                 & (centers[:, 2] > floor_h - .04) & (centers[:, 2] < top_h + .08))
        centers = centers[valid]
        # 内部洞、短末端缺口可补；较长缺端须覆盖半厢宽且留出预测一致。
        # 孤岛不能扩大支持范围。
        lower, upper = model.cells[:, 0].min(), model.cells[:, 0].max()
        if (0 <= lower - bounds[0] <= MAX_RIB_EXTRAPOLATION_M
                or stable_axis_extension(model, bounds[0], bounds[1] - bounds[0])):
            lower = bounds[0] + RIB_WALL_CLEARANCE_M
        if (0 <= bounds[1] - upper <= MAX_RIB_EXTRAPOLATION_M
                or stable_axis_extension(model, bounds[1], bounds[1] - bounds[0])):
            upper = bounds[1] - RIB_WALL_CLEARANCE_M
        centers = centers[(centers[:, 0] >= lower) & (centers[:, 0] <= upper)]
        if 'left' in conflicts:
            centers = centers[centers[:, 0] >= support[0]]
        if 'right' in conflicts:
            centers = centers[centers[:, 0] <= support[1]]
        # 有足够杆形支持的格不加密；邻近壁面或不足三点的格不算杆已补齐。
        # 对全部当前点另作小半径避让，既不覆盖原点，也不叠出第二条轴。
        if len(centers):
            supported = profile_support_points(observed, observed_tree, model,
                                               floor_h, top_h, RIB_EXISTING_RADIUS_M)
            distances, _ = cKDTree(supported).query(centers, k=1)
            existing, _ = tree.query(centers, k=1)
            keys, counts = np.unique(np.floor(supported[:, 0] / CELL_M).astype(int), return_counts=True)
            occupied = keys[counts >= 3]
            sparse = ~np.isin(np.floor(centers[:, 0] / CELL_M).astype(int), occupied)
            centers = centers[((distances > RIB_EXISTING_RADIUS_M) | sparse)
                              & (existing > RIB_MINIMUM_CLEARANCE_M)]
        # 截面采样仅填轴线附近的小孔；不把整块端壁或车棚复制为横杆。
        added = (centers[:, None, :] + offsets).reshape(-1, 3)
        parts.append(added)
        details.append({'ribId': rib.rib_id, 'pointCount': int(len(added)),
                        'conflictingSides': sorted(conflicts)})
    result = np.vstack(parts) if parts else empty
    return result, {'completed': bool(len(result)), 'pointCount': int(len(result)),
                    'ribs': details}
