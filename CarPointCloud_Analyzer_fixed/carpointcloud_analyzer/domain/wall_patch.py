"""基于本次实测端壁面的局部补洞；合成点不构成载荷或实测覆盖证据。"""

from typing import Protocol

import numpy as np
from scipy.spatial import ConvexHull, QhullError, cKDTree

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.inner_faces import FACE_CELL_M, FACE_TOLERANCE_M, ObservedFace, supported_faces


MAX_END_PATCH_POINTS = 20_000
MIN_SURFACE_SPACING_M = 0.01
# 仅连接小于端壁 4cm 补点网格间距的角部缝；大面积缺面仍需本次观测支持。
MAX_SIDE_END_SEAM_GAP_M = 0.03
SEAM_HEIGHT_STEP_M = 0.02
# 角线与侧壁面重叠 1mm，避免补点后再次拟合侧壁时的亚毫米漂移重新打开接缝。
SIDE_END_SEAM_OVERLAP_M = 0.001
MAX_SIDE_END_LENGTH_GAP_M = 0.35
SIDE_END_STRIP_STEP_M = 0.04
SIDE_END_DONOR_BIN_M = 0.10
SIDE_END_DONOR_MIN_BIN_POINTS = 4
# 只用于原本选面歧义的有界回退，不改变实测贴面、镜像或支撑域的容差。
RELAXED_END_FACE_MAX_SEPARATION_M = 0.10
RELAXED_END_FACE_MAX_CENTER_SPREAD_M = 0.015
RELAXED_END_FACE_MAX_BOUNDARY_OFFSET_M = 0.01
RELAXED_END_FACE_MIN_SUPPORT_RATIO = 1.5
# 前壁顶篷布可能形成一块向车厢外倾斜的完整面；只有同时找到贴近
# 结构边界的近竖直内壁，才可把这两层区分开，不能按点数或距离随意选面。
INNER_LAYER_MAX_BOUNDARY_OFFSET_M = 0.02
INNER_LAYER_CENTER_GAP_RANGE_M = (0.025, 0.06)
INNER_LAYER_MAX_HEIGHT_SLOPE = 0.03
OUTER_LAYER_MIN_HEIGHT_SLOPE = 0.06
INNER_LAYER_MIN_CELL_RATIO = 1.0
INNER_LAYER_MIN_SPAN_RATIO = 0.8
# 仅在后壁靠近整车尾端且分割阶段已有独立实测内壁面时启用。
TAIL_INNER_MAX_BOUNDARY_OFFSET_M = 0.02
TAIL_INNER_MIN_REFERENCE_MARGIN_M = 0.005
TAIL_INNER_MIN_SPAN_RATIO = 0.9


class _WallProfile(Protocol):
    floor_z: float
    wall_top: float
    interior_w_range: tuple[float, float]


def _hull(points: np.ndarray) -> ConvexHull | None:
    """共线或退化支持不能补成二维壁面。"""
    if len(points) < 3:
        return None
    try:
        return ConvexHull(points)
    except QhullError:
        return None


def _inside(points: np.ndarray, hull: ConvexHull) -> np.ndarray:
    return np.all(points @ hull.equations[:, :2].T + hull.equations[:, 2] <= 1e-9,
                  axis=1)


def _clip_polygon(polygon: np.ndarray, constraints: np.ndarray) -> np.ndarray:
    """将支撑多边形同步裁入生成域，避免元数据宣称未实际补点的外推区域。"""
    for normal_w, normal_h, offset in constraints:
        if len(polygon) < 3:
            return np.empty((0, 2))
        normal = np.array([normal_w, normal_h])
        result = []
        previous = polygon[-1]
        previous_value = float(previous @ normal + offset)
        for current in polygon:
            value = float(current @ normal + offset)
            if (value <= 0) != (previous_value <= 0):
                fraction = previous_value / (previous_value - value)
                result.append(previous + fraction * (current - previous))
            if value <= 0:
                result.append(current)
            previous, previous_value = current, value
        polygon = np.asarray(result).reshape(-1, 2)
    return polygon


def _grid(bounds: np.ndarray, spacing: float) -> tuple[np.ndarray, float]:
    """先限制网格总格数再分配；极小配置间距也不能造成无界内存分配。"""
    spans = bounds[1] - bounds[0]
    spacing = max(spacing, MIN_SURFACE_SPACING_M)
    # 一维跨度项也受约束，不能仅按面积限制狭长面的内存。
    spacing = max(spacing, float(np.max(spans)) / (MAX_END_PATCH_POINTS - 1))
    counts = np.maximum(2, np.ceil(spans / spacing).astype(int) + 1)
    while int(counts[0]) * int(counts[1]) > MAX_END_PATCH_POINTS:
        spacing *= max(1.1, np.sqrt(np.prod(counts) / MAX_END_PATCH_POINTS))
        counts = np.maximum(2, np.ceil(spans / spacing).astype(int) + 1)
    w, h = np.meshgrid(np.linspace(bounds[0, 0], bounds[1, 0], counts[0]),
                       np.linspace(bounds[0, 1], bounds[1, 1], counts[1]))
    return np.column_stack((w.ravel(), h.ravel())), spacing


def _horizontal_extent(polygon: np.ndarray, height: float) -> tuple[float, float] | None:
    """求实测支撑多边形在指定高度的横向截线。"""
    widths = []
    for start, end in zip(polygon, np.roll(polygon, -1, axis=0)):
        low, high = sorted((start[1], end[1]))
        if not low <= height <= high:
            continue
        if abs(end[1] - start[1]) < 1e-9:
            widths.extend((float(start[0]), float(end[0])))
        else:
            fraction = (height - start[1]) / (end[1] - start[1])
            widths.append(float(start[0] + fraction * (end[0] - start[0])))
    return (min(widths), max(widths)) if widths else None


def _side_end_seams(
    polygons: list[np.ndarray], face_coefficients: np.ndarray,
    side_planes: dict[str, np.ndarray], boundary_l: float, band: float,
    width_range: tuple[float, float], point_budget: int,
) -> tuple[np.ndarray, list[np.ndarray], list[dict]]:
    """只在两面已拟合且横向相差不超过 3cm 时，生成真实角线的窄接缝。"""
    points, supports, diagnostics = [], [], []
    a, b, c = face_coefficients
    for polygon in polygons:
        lower, upper = np.min(polygon[:, 1]), np.max(polygon[:, 1])
        if upper - lower < SEAM_HEIGHT_STEP_M:
            continue
        heights = np.linspace(lower, upper,
                              max(2, int(np.ceil((upper - lower) / SEAM_HEIGHT_STEP_M)) + 1))
        for side_name, sign in (("left", -1), ("right", 1)):
            plane = np.asarray(side_planes.get(side_name), dtype=float)
            if plane.shape != (3,) or not np.isfinite(plane).all():
                continue
            sa, sb, sc = plane
            denominator = 1 - sa * a
            if abs(denominator) < .5:
                continue
            rows = []
            for height in heights:
                extent = _horizontal_extent(polygon, float(height))
                if extent is None:
                    rows.append(None)
                    continue
                edge = extent[0] if sign < 0 else extent[1]
                side_w = float((sa * (b * height + c) + sb * height + sc) / denominator)
                gap = sign * (side_w - edge)
                seam_w = side_w + sign * SIDE_END_SEAM_OVERLAP_M
                side_l = a * seam_w + b * height + c
                if (not np.isfinite(side_w) or not 0 < gap <= MAX_SIDE_END_SEAM_GAP_M
                        or not width_range[0] <= seam_w <= width_range[1]
                        or abs(side_l - boundary_l) > band):
                    rows.append(None)
                else:
                    rows.append((edge, seam_w, float(height), float(gap)))
            run = []

            def flush() -> None:
                if len(run) < 2 or len(points) + 2 * len(run) > point_budget:
                    return
                strip = np.array([[w, h] for edge, side_w, h, _ in run
                                  for w in (edge, side_w)])
                hull = _hull(strip)
                if hull is None:
                    return
                support = strip[hull.vertices]
                supports.append(support)
                points.extend(np.column_stack((strip[:, 0],
                                               strip @ np.array([a, b]) + c,
                                               strip[:, 1])).tolist())
                diagnostics.append({"side": side_name, "pointCount": 2 * len(run),
                                    "maxGapM": float(max(row[3] for row in run)),
                                    "heightRangeM": [run[0][2], run[-1][2]]})

            for row in [*rows, None]:
                if row is None:
                    flush()
                    run = []
                else:
                    run.append(row)
    return (np.asarray(points, dtype=float).reshape(-1, 3), supports, diagnostics)


def complete_side_end_strip(
    side_source: np.ndarray, side_coefficients: np.ndarray, end_model: dict,
    boundary_l: float, interior_sign: int, end_band: float,
    *, opposite_boundary_l: float | None = None,
) -> tuple[np.ndarray, dict]:
    """连接端壁与同侧实测壁；双端有效时可跨越长扫描盲段。"""
    empty = np.empty((0, 3), dtype=float)
    if end_model.get("valid") is not True or interior_sign not in (-1, 1):
        return empty, {"completed": False, "reason": "end_face_unavailable"}
    side = np.asarray(side_source, dtype=float)
    plane = np.asarray(side_coefficients, dtype=float)
    face = np.asarray(end_model.get("coefficients"), dtype=float)
    if (side.ndim != 2 or side.shape[1:] != (3,) or len(side) < 100
            or plane.shape != (3,) or face.shape != (3,)
            or not np.isfinite(side).all() or not np.isfinite(plane).all()
            or not np.isfinite(face).all()):
        return empty, {"completed": False, "reason": "side_face_unavailable"}
    a, b, c = face
    sa, sb, sc = plane
    denominator = 1 - sa * a
    if abs(denominator) < .5:
        return empty, {"completed": False, "reason": "face_intersection_unavailable"}
    polygons = [np.asarray(item, dtype=float)
                for item in end_model.get("supportPolygonsWH", [])]
    hulls = [_hull(item) for item in polygons]
    hulls = [item for item in hulls if item is not None]
    if not hulls:
        return empty, {"completed": False, "reason": "end_support_unavailable"}
    h_lo = min(float(np.min(item[:, 1])) for item in polygons)
    h_hi = max(float(np.max(item[:, 1])) for item in polygons)
    heights = np.linspace(h_lo, h_hi,
                          max(2, int(np.ceil((h_hi - h_lo) / SIDE_END_STRIP_STEP_M)) + 1))
    residual = np.abs(side[:, 0] - (sa * side[:, 1] + sb * side[:, 2] + sc))
    side = side[residual <= .035]
    rows = []
    for height in heights:
        width = float((sa * (b * height + c) + sb * height + sc) / denominator)
        length = float(a * width + b * height + c)
        corner = np.array([[width, height]])
        if not any(_inside(corner, hull)[0] for hull in hulls):
            continue
        distance = interior_sign * (side[:, 1] - length)
        donor = distance[(np.abs(side[:, 2] - height) <= .05)
                         & (distance > max(end_band, SIDE_END_STRIP_STEP_M))
                         & (distance <= MAX_SIDE_END_LENGTH_GAP_M)]
        if len(donor) >= SIDE_END_DONOR_MIN_BIN_POINTS:
            # 小缺口沿用原有行为，保证既有完整车型的补点结果不变。
            gap = float(np.max(donor))
            long_gap = False
        else:
            # 不用固定距离否决雷达盲区：两块端壁均已验证时，从端壁角线
            # 接到同高、连续两格有实测支持的最近侧壁段。只填端壁之间，
            # 不把孤立横杆或车头点当作侧壁，也不越过对侧端壁。
            if opposite_boundary_l is None or not np.isfinite(opposite_boundary_l):
                continue
            max_distance = interior_sign * (opposite_boundary_l - length) - end_band
            if max_distance <= MAX_SIDE_END_LENGTH_GAP_M:
                continue
            far = distance[(np.abs(side[:, 2] - height) <= .05)
                           & (distance > MAX_SIDE_END_LENGTH_GAP_M)
                           & (distance < max_distance)]
            if len(far) < 2 * SIDE_END_DONOR_MIN_BIN_POINTS:
                continue
            edges = np.arange(MAX_SIDE_END_LENGTH_GAP_M,
                              max_distance + SIDE_END_DONOR_BIN_M,
                              SIDE_END_DONOR_BIN_M)
            if len(edges) < 3:
                continue
            counts, _ = np.histogram(far, bins=edges)
            supported = np.flatnonzero(
                (counts[:-1] >= SIDE_END_DONOR_MIN_BIN_POINTS)
                & (counts[1:] >= SIDE_END_DONOR_MIN_BIN_POINTS)
            )
            if not len(supported):
                continue
            first = int(supported[0])
            gap = float(min(max_distance, edges[first + 1]))
            long_gap = True
        rows.append((float(height), length, gap, long_gap))
    parts, gaps = [], []
    long_gap_rows = 0
    height_step = float(heights[1] - heights[0])
    for height, length, gap, long_gap in rows:
        if long_gap:
            # 单个高度的远处回波可能是横杆或车架；只有邻近高度也在
            # 相似纵向位置遇到侧壁，才把长盲段解释为连续车厢壁。
            coherent = any(
                other_long and 0 < abs(other_height - height) <= 1.1 * height_step
                and abs(other_gap - gap) <= 0.6
                for other_height, _, other_gap, other_long in rows
            )
            if not coherent:
                continue
            long_gap_rows += 1
        lengths = length + interior_sign * np.linspace(
            0, gap, max(2, int(np.ceil(gap / SIDE_END_STRIP_STEP_M)) + 1),
        )
        widths = sa * lengths + sb * height + sc
        parts.append(np.column_stack((widths, lengths,
                                      np.full(len(lengths), height))))
        gaps.append(gap)
    if not parts:
        return empty, {"completed": False, "reason": "side_end_gap_unsupported"}
    result = np.vstack(parts)
    return result, {"completed": True, "candidatePointCount": int(len(result)),
                    "maximumGapM": float(max(gaps)),
                    "longGapHeightRows": long_gap_rows,
                    "heightRangeM": [float(np.min(result[:, 2])),
                                     float(np.max(result[:, 2]))]}


def _face_separation(first: ObservedFace, second: ObservedFace) -> float:
    """在双方实际支持格的并集上比较，避免只在中心交点判断两面相同。"""
    comparison = np.vstack((first.cells[:, [0, 2]], second.cells[:, [0, 2]]))
    design = np.column_stack((comparison, np.ones(len(comparison))))
    return float(np.max(np.abs(design @ (second.coefficients - first.coefficients))))


def _relaxed_dominant_face(
    faces: list[ObservedFace], center: np.ndarray, boundary_l: float,
) -> tuple[ObservedFace | None, dict]:
    """中心近重合且有明显主面的局部拟合允许有界补洞，不平均候选面。"""
    ranked = sorted(faces, key=lambda face: -len(face.cells))
    chosen = ranked[0]
    support_ratio = len(chosen.cells) / len(ranked[1].cells)
    center_spread = float(np.ptp([face.at(*center) for face in ranked]))
    boundary_offset = abs(chosen.at(*center) - boundary_l)
    max_separation = max(_face_separation(first, second)
                         for i, first in enumerate(ranked) for second in ranked[i + 1:])
    if (support_ratio < RELAXED_END_FACE_MIN_SUPPORT_RATIO
            or center_spread > RELAXED_END_FACE_MAX_CENTER_SPREAD_M + 1e-9
            or boundary_offset > RELAXED_END_FACE_MAX_BOUNDARY_OFFSET_M + 1e-9
            or max_separation > RELAXED_END_FACE_MAX_SEPARATION_M + 1e-9):
        return None, {}
    return chosen, {
        "originalReason": "ambiguous_end_faces",
        "candidateCellCounts": [int(len(face.cells)) for face in ranked],
        "supportRatio": float(support_ratio), "centerSpreadM": center_spread,
        "selectedBoundaryOffsetM": float(boundary_offset),
        "maxSurfaceSeparationM": max_separation,
        "limits": {
            "minSupportRatio": RELAXED_END_FACE_MIN_SUPPORT_RATIO,
            "maxCenterSpreadM": RELAXED_END_FACE_MAX_CENTER_SPREAD_M,
            "maxBoundaryOffsetM": RELAXED_END_FACE_MAX_BOUNDARY_OFFSET_M,
            "maxSurfaceSeparationM": RELAXED_END_FACE_MAX_SEPARATION_M,
        },
    }


def _inner_face_beside_sloped_cover(
    faces: list[ObservedFace], center: np.ndarray, boundary_l: float,
    interior_sign: int | None,
) -> tuple[ObservedFace | None, dict]:
    """仅将近竖直内壁与外侧斜盖形成的双层端面解歧。"""
    if interior_sign not in (-1, 1) or len(faces) != 2:
        return None, {}
    ordered = sorted(faces, key=lambda face: face.at(*center))
    inner, outer = (ordered[0], ordered[1]) if interior_sign == -1 else (ordered[1], ordered[0])
    inner_l, outer_l = inner.at(*center), outer.at(*center)
    gap = interior_sign * (inner_l - outer_l)
    inner_span = np.ptp(inner.cells[:, [0, 2]], axis=0)
    outer_span = np.ptp(outer.cells[:, [0, 2]], axis=0)
    if (
        not INNER_LAYER_CENTER_GAP_RANGE_M[0] <= gap <= INNER_LAYER_CENTER_GAP_RANGE_M[1]
        or abs(inner_l - boundary_l) > INNER_LAYER_MAX_BOUNDARY_OFFSET_M
        or abs(inner.coefficients[1]) > INNER_LAYER_MAX_HEIGHT_SLOPE
        or abs(outer.coefficients[1]) < OUTER_LAYER_MIN_HEIGHT_SLOPE
        or len(inner.cells) < len(outer.cells) * INNER_LAYER_MIN_CELL_RATIO
        or np.any(inner_span < outer_span * INNER_LAYER_MIN_SPAN_RATIO)
    ):
        return None, {}
    return inner, {
        'originalReason': 'ambiguous_end_faces',
        'innerFaceL': float(inner_l), 'outerFaceL': float(outer_l),
        'centerGapM': float(gap),
        'innerHeightSlope': float(inner.coefficients[1]),
        'outerHeightSlope': float(outer.coefficients[1]),
        'innerCellCount': int(len(inner.cells)),
        'outerCellCount': int(len(outer.cells)),
        'selection': 'near_boundary_vertical_inner_face_beside_sloped_outer_cover',
    }


def _tail_anchored_inner_face(
    faces: list[ObservedFace], center: np.ndarray, boundary_l: float,
    verified_inner_plane: tuple[float, float, float] | None,
) -> tuple[ObservedFace | None, dict]:
    """用分割阶段的实测后内壁面核对尾端双面，不按单点远近猜测。"""
    reference = np.asarray(verified_inner_plane, dtype=float)
    if (len(faces) != 2 or reference.shape != (3,)
            or not np.isfinite(reference).all()
            or abs(float(reference @ [*center, 1.]) - boundary_l)
            > TAIL_INNER_MAX_BOUNDARY_OFFSET_M):
        return None, {}
    scored = []
    for face in faces:
        design = np.column_stack((face.cells[:, [0, 2]], np.ones(len(face.cells))))
        disagreement = np.abs(design @ (face.coefficients - reference))
        scored.append((float(np.percentile(disagreement, 90)), face))
    scored.sort(key=lambda item: item[0])
    best_gap, chosen = scored[0]
    other_gap, other = scored[1]
    chosen_span = np.ptp(chosen.cells[:, [0, 2]], axis=0)
    other_span = np.ptp(other.cells[:, [0, 2]], axis=0)
    if (
        best_gap > FACE_TOLERANCE_M
        or other_gap - best_gap < TAIL_INNER_MIN_REFERENCE_MARGIN_M
        or other_gap <= FACE_TOLERANCE_M
        or len(chosen.cells) < len(other.cells) * RELAXED_END_FACE_MIN_SUPPORT_RATIO
        or np.any(chosen_span < other_span * TAIL_INNER_MIN_SPAN_RATIO)
        or chosen.at(*center) <= other.at(*center)
        or abs(chosen.at(*center) - boundary_l) > TAIL_INNER_MAX_BOUNDARY_OFFSET_M
        or _face_separation(chosen, other) > RELAXED_END_FACE_MAX_SEPARATION_M
    ):
        return None, {}
    return chosen, {
        "originalReason": "ambiguous_end_faces",
        "selection": "tail_anchored_inner_face",
        "candidateCellCounts": [int(len(chosen.cells)), int(len(other.cells))],
        "selectedReferenceP90M": best_gap,
        "otherReferenceP90M": other_gap,
        "selectedBoundaryOffsetM": abs(chosen.at(*center) - boundary_l),
        "maxSurfaceSeparationM": _face_separation(chosen, other),
    }


def _choose_face(
    points: np.ndarray, boundary_l: float, band: float, *, diagnostics: dict | None = None,
    interior_sign: int | None = None,
    verified_inner_plane: tuple[float, float, float] | None = None,
) -> tuple[ObservedFace | None, str]:
    """原成功选面保持；近重合局部拟合的有界放宽必须披露来源。"""
    center = np.median(points[:, [0, 2]], axis=0)
    faces = supported_faces(points, axis=1)
    nearby = [face for face in faces if abs(face.at(*center) - boundary_l) <= band]
    if not nearby:
        return None, "supported_end_face_unavailable"
    nearby.sort(key=lambda face: abs(face.at(*center) - boundary_l))
    chosen = nearby[0]
    # 两块都具有二维支持的平行壁面不能被平均成一个不存在的面。
    # 若弱面在中心处与主面几乎重合，它可能只是同一弯曲端壁的另一种局部拟合。
    # 只有主面至少有两倍格数、且主面更接近已验证端壁时才忽略弱面；
    # 两块支撑相当的面默认仍按歧义拒绝；只有下方可证明为
    # 贴结构边界的竖直内壁＋外侧斜盖时，才选择内壁。
    ignored_weaker = False
    for face in nearby[1:]:
        if _face_separation(chosen, face) > FACE_TOLERANCE_M:
            center_gap = abs(face.at(*center) - chosen.at(*center))
            if (len(chosen.cells) >= 2 * len(face.cells)
                    and center_gap <= 0.01
                    and abs(chosen.at(*center) - boundary_l)
                    < abs(face.at(*center) - boundary_l)):
                ignored_weaker = True
                continue
            relaxed, evidence = _relaxed_dominant_face(nearby, center, boundary_l)
            if relaxed is not None:
                if diagnostics is not None:
                    diagnostics.update(evidence)
                return relaxed, "dominant_near_coincident_relaxed"
            inner, evidence = _inner_face_beside_sloped_cover(
                nearby, center, boundary_l, interior_sign,
            )
            if inner is not None:
                if diagnostics is not None:
                    diagnostics.update(evidence)
                return inner, "inner_face_beside_sloped_cover"
            if interior_sign == 1 and verified_inner_plane is not None:
                inner, evidence = _tail_anchored_inner_face(
                    nearby, center, boundary_l, verified_inner_plane,
                )
                if inner is not None:
                    if diagnostics is not None:
                        diagnostics.update(evidence)
                    return inner, "tail_anchored_inner_face"
            return None, "ambiguous_end_faces"
    return chosen, "dominant_near_coincident_face" if ignored_weaker else "ok"


def complete_end_patch(
    observed: np.ndarray, boundary_l: float, mirror_axis: float,
    profile: _WallProfile, config: Config, *, interior_sign: int | None = None,
    side_planes: dict[str, np.ndarray] | None = None,
    verified_inner_plane: tuple[float, float, float] | None = None,
) -> tuple[np.ndarray, dict]:
    """在实测二维端壁支撑及其可用镜像支撑内补网格，不补底板、不外推高度。

    ``coefficients`` 始终为实测面 L=aW+bH+c。明显横向倾斜的端壁只在原支撑内
    补洞，不将镜像点拼成扭曲平面。``supportPolygonsWH`` 是实际生成域的并集；
    ``supportPolygonWH`` 仅是整体外包凸包，不能独自作为测量支撑证明。
    调用方仍须对网格和已有点做三维去重，再记录 acceptedPointCount。
    """
    empty = np.empty((0, 3), dtype=float)
    metadata = {
        "valid": False, "source": "current_observed_end_wall",
        "coefficients": None, "supportPolygonWH": [], "supportPolygonsWH": [],
        "observedPointCount": 0, "modelGeneratedPoints": 0,
        "symmetryApplied": False, "heightExtrapolated": False,
        "reason": "invalid_end_patch_input",
    }
    points = np.asarray(observed, dtype=float)
    settings = np.array([
        boundary_l, mirror_axis, profile.floor_z, profile.wall_top,
        *profile.interior_w_range, config.carriage_completion_end_wall_band,
        config.carriage_completion_floor_clearance,
        config.carriage_completion_height_margin, config.carriage_completion_width_margin,
        config.carriage_box_surface_spacing,
    ], dtype=float)
    if (points.ndim != 2 or points.shape[1:] != (3,) or not len(points)
            or not np.all(np.isfinite(points)) or not np.all(np.isfinite(settings))
            or config.carriage_completion_end_wall_band <= 0
            or config.carriage_box_surface_spacing <= 0):
        return empty, metadata
    band = config.carriage_completion_end_wall_band
    w_lo = profile.interior_w_range[0] - config.carriage_completion_width_margin
    w_hi = profile.interior_w_range[1] + config.carriage_completion_width_margin
    h_lo = profile.floor_z + config.carriage_completion_floor_clearance
    h_hi = profile.wall_top + config.carriage_completion_height_margin
    points = points[(np.abs(points[:, 1] - boundary_l) <= band)
                    & (points[:, 0] >= w_lo) & (points[:, 0] <= w_hi)
                    & (points[:, 2] >= h_lo) & (points[:, 2] <= h_hi)]
    if not len(points):
        return empty, metadata
    selection_diagnostic: dict = {}
    face, reason = _choose_face(
        points, boundary_l, band, diagnostics=selection_diagnostic,
        interior_sign=interior_sign, verified_inner_plane=verified_inner_plane,
    )
    metadata["reason"] = reason
    if face is None:
        return empty, metadata
    metadata["faceSelection"] = reason
    if selection_diagnostic:
        metadata["faceSelectionRelaxation"] = selection_diagnostic
    wh = points[:, [0, 2]]
    residual = np.abs(points[:, 1] - np.column_stack((wh, np.ones(len(wh))))
                      @ face.coefficients)
    # 每个 face cell 已有至少四个实测点；孤立的近面离群点不能扩大支撑凸包。
    near_cells, _ = cKDTree(face.cells[:, [0, 2]]).query(wh, p=np.inf)
    support = wh[(residual <= FACE_TOLERANCE_M) & (near_cells <= FACE_CELL_M)]
    original = _hull(support)
    if original is None:
        metadata["reason"] = "degenerate_support"
        return empty, metadata
    polygon = support[original.vertices]
    polygons = [polygon]
    mirror_delta = np.max(np.abs(2 * (polygon[:, 0] - mirror_axis)
                                 * face.coefficients[0]))
    if mirror_delta <= FACE_TOLERANCE_M:
        mirrored = polygon.copy()
        mirrored[:, 0] = 2 * mirror_axis - mirrored[:, 0]
        mirrored_hull = _hull(mirrored)
        if mirrored_hull is not None:
            polygons.append(mirrored[mirrored_hull.vertices])
            metadata["symmetryApplied"] = True
    else:
        metadata["symmetryReason"] = "transversely_sloped_observed_face"
    a, b, c = face.coefficients
    constraints = np.array([
        [-1, 0, w_lo], [1, 0, -w_hi], [0, -1, h_lo], [0, 1, -h_hi],
        [a, b, c - boundary_l - band], [-a, -b, boundary_l - band - c],
    ])
    clipped_polygons, hulls = [], []
    for item in polygons:
        clipped = _clip_polygon(item, constraints)
        hull = _hull(clipped)
        if hull is not None:
            clipped_polygons.append(clipped[hull.vertices])
            hulls.append(hull)
    polygons = clipped_polygons
    if not polygons:
        metadata["reason"] = "support_outside_carriage"
        return empty, metadata
    overall = np.vstack(polygons)
    bounds = np.vstack((np.min(overall, axis=0), np.max(overall, axis=0)))
    bounds[0] = np.maximum(bounds[0], [w_lo, h_lo])
    bounds[1] = np.minimum(bounds[1], [w_hi, h_hi])
    if np.any(bounds[1] <= bounds[0]):
        metadata["reason"] = "support_outside_carriage"
        return empty, metadata
    grid, spacing = _grid(bounds, config.carriage_box_surface_spacing)
    supported = np.logical_or.reduce([_inside(grid, hull) for hull in hulls])
    grid = grid[supported]
    positions = np.column_stack((grid, np.ones(len(grid)))) @ face.coefficients
    result = np.column_stack((grid[:, 0], positions, grid[:, 1]))
    # 在 W/H 域内也不得由斜面公式跑出已验证端壁搜索带。
    result = result[np.abs(result[:, 1] - boundary_l) <= band]
    if side_planes:
        seam_points, seam_polygons, seam_info = _side_end_seams(
            polygons, face.coefficients, side_planes, boundary_l, band,
            (w_lo, w_hi), MAX_END_PATCH_POINTS - len(result),
        )
        if len(seam_points):
            result = np.vstack((result, seam_points))
            polygons.extend(seam_polygons)
            overall = np.vstack(polygons)
            metadata["sideEndSeams"] = seam_info
    overall_hull = _hull(overall)
    metadata.update({
        "valid": True, "reason": "ok", "coefficients": face.coefficients.tolist(),
        "supportPolygonWH": overall[overall_hull.vertices].tolist(),
        "supportPolygonsWH": [item.tolist() for item in polygons],
        "observedPointCount": int(len(support)), "observedCellCount": int(len(face.cells)),
        "modelGeneratedPoints": int(len(result)), "surfaceSpacingM": float(spacing),
        "maximumModelPoints": MAX_END_PATCH_POINTS,
    })
    return result, metadata
