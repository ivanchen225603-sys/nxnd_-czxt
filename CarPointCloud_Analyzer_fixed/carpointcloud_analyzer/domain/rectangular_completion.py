"""当前实测对称面支撑的短矩形开顶车厢四壁、底板镜像补点。

原始点保持原值和顺序；合成点只填有支撑的面内空洞，业务统一消费最终云。
"""

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.carriage import CarriageAnalyzer
from carpointcloud_analyzer.domain.dimensions import _measure_continuous_side_height
from carpointcloud_analyzer.domain.inner_faces import FACE_TOLERANCE_M, supported_faces
from carpointcloud_analyzer.domain.models import BoxCarriageBox
from carpointcloud_analyzer.domain.utils import occupancy_extent, robust_span
from carpointcloud_analyzer.domain.utils.cells import unique_cells


END_BAND_M = 0.24
SIDE_BAND_M = 0.18
FACE_RESIDUAL_M = 0.06
END_BOUNDARY_TOLERANCE_M = 0.12
ENVELOPE_BAND_M = 0.05
GUTTER_BAND_M = 0.35
GUTTER_MIN_RECESS_M = 0.03
END_GUTTER_DEEP_RECESS_M = 0.12
MIN_SURFACE_SPACING_M = 0.022
MAX_SURFACE_SPACING_M = 0.045
MIN_FLOOR_BINS = 5
MIN_FLOOR_RUN_M = 1.0
MAX_FLOOR_CENTER_GAP_FRACTION = 0.16
FLOOR_BIN_M = 0.15
MIN_FLOOR_BIN_POINTS = 20
MIN_FLOOR_SPACING_M = 0.02
MAX_FLOOR_SPACING_M = 0.05


@dataclass(frozen=True)
class RectangularCompletion:
    """原点不变的合成点及逐区域可审计计数。"""

    points: np.ndarray
    metadata: dict
    floor_points: np.ndarray


def _empty() -> np.ndarray:
    return np.empty((0, 3), dtype=float)


def _surface_spacing(source: np.ndarray, area: float) -> float:
    """按实测面密度取网格步长，避免补片突变成过密或稀疏扫描线。"""
    return float(np.clip(np.sqrt(area / max(len(source), 1)),
                         MIN_SURFACE_SPACING_M, MAX_SURFACE_SPACING_M))


def _surface_cell_count(points: np.ndarray, axes: tuple[int, int]) -> int:
    """按覆盖格数而非点数选择完整半幅，避免单条密集扫描线误导。"""
    if not len(points):
        return 0
    return len(unique_cells(np.floor(points[:, axes] / .10).astype(int)))


def _surface_holes(
    observed_face: np.ndarray, candidate: np.ndarray,
    axes: tuple[int, int], existing: np.ndarray, spacing: float,
) -> np.ndarray:
    """仅追加细网格中无实测邻点的候选；不覆盖任何已有扫描点。"""
    if not len(candidate):
        return _empty()
    if len(observed_face):
        near = cKDTree(observed_face[:, axes]).query(
            candidate[:, axes], workers=-1)[0]
        candidate = candidate[near > spacing * .68]
    if not len(candidate):
        return _empty()
    return candidate[cKDTree(existing).query(candidate, workers=-1)[0]
                     > spacing * .68]


def _sample_shape_depths(
    source: np.ndarray, inlier_axes: tuple[int, int], depth_axis: int,
    query: np.ndarray, spacing: float,
) -> np.ndarray:
    """源侧面内坐标处采样实测壁深中位数；无近邻处为 NaN。

    镜像补点的壁深不能只投影到全局拟合平面：顶部导水槽、波纹等面内
    凹凸同样是当前实测形状，应随镜像一起补到缺失侧。最近邻超过 1.6
    倍网格步长视为源侧空洞，由调用方回退拟合平面。
    """
    if not len(source):
        return np.full(len(query), np.nan)
    tree = cKDTree(source[:, list(inlier_axes)])
    distances, indices = tree.query(query, k=6, workers=-1)
    near = distances[:, 0] <= spacing * 1.6
    result = np.full(len(query), np.nan)
    result[near] = np.median(source[:, depth_axis][indices[near]], axis=1)
    return result


def _complete_end(
    points: np.ndarray, existing: np.ndarray, box: BoxCarriageBox, boundary: float,
    axis_w: float, floor_h: float, top_h: float,
) -> tuple[np.ndarray, dict]:
    band = points[(np.abs(points[:, 1] - boundary) <= END_BAND_M)
                  & (points[:, 2] >= floor_h + 0.02)
                  & (points[:, 2] <= top_h + 0.05)]
    faces = supported_faces(band, 1)
    faces = [face for face in faces
             if abs(face.at(axis_w, (floor_h + top_h) / 2) - boundary)
             <= END_BOUNDARY_TOLERANCE_M]
    if not faces:
        return _empty(), {'reason': 'observed_end_face_unavailable', 'synthesizedPoints': 0}
    face = max(faces, key=lambda item: len(item.cells))
    plane = np.c_[band[:, [0, 2]], np.ones(len(band))] @ face.coefficients
    source = band[np.abs(band[:, 1] - plane) <= FACE_RESIDUAL_M]
    left = source[source[:, 0] < axis_w]
    right = source[source[:, 0] >= axis_w]
    source_half = ('left' if _surface_cell_count(left, (0, 2))
                   >= _surface_cell_count(right, (0, 2)) else 'right')
    bottom_h = max(box.h_range[0], floor_h - .10)
    upper_h = box.h_range[1]
    spacing = _surface_spacing(source, box.width * (upper_h - bottom_h))
    w_grid = np.arange(box.w_range[0] + spacing / 2,
                       box.w_range[1], spacing)
    h_grid = np.arange(bottom_h + spacing / 2, upper_h, spacing)
    # 完整半幅决定墙面；另一半沿 W 中面对称，不在缺失处另拟一个面。
    # 壁深优先取源侧镜像位置的实测形状（含导水槽等凹凸），源侧空洞
    # 处才回退拟合平面。
    ww, hh = np.meshgrid(w_grid, h_grid)
    source_ww = (np.minimum(ww, 2 * axis_w - ww) if source_half == 'left'
                 else np.maximum(ww, 2 * axis_w - ww))
    shape_query = np.column_stack((source_ww.ravel(), hh.ravel()))
    shape_source = band[np.abs(band[:, 1] - boundary)
                        <= END_BOUNDARY_TOLERANCE_M]
    shape = _sample_shape_depths(shape_source, (0, 2), 1, shape_query, spacing)
    plane_depth = (np.c_[shape_query, np.ones(len(shape_query))]
                   @ face.coefficients)
    depth = np.where(np.isnan(shape), plane_depth, shape)
    candidate = np.column_stack((ww.ravel(), depth, hh.ravel()))
    candidate = candidate[np.abs(candidate[:, 1] - boundary)
                          <= END_BOUNDARY_TOLERANCE_M]
    added = _surface_holes(source, candidate, (0, 2), existing, spacing)
    return added, {
        'reason': 'observed_half_face_mirrored_full_surface' if len(added)
                  else 'no_surface_gap',
        'sourceHalf': source_half, 'spacingM': round(spacing, 4),
        'completedHeightRangeM': [round(bottom_h, 4), round(upper_h, 4)],
        'observedFaceCells': len(face.cells), 'sourcePoints': len(source),
        'synthesizedPoints': len(added),
    }


def _complete_sides(
    points: np.ndarray, existing: np.ndarray, box: BoxCarriageBox,
    axis_w: float, floor_h: float, top_h: float,
) -> tuple[np.ndarray, dict]:
    body = points[(points[:, 1] >= box.rear_l + .02)
                  & (points[:, 1] <= box.front_l - .02)
                  & (points[:, 2] >= floor_h + .02)
                  & (points[:, 2] <= top_h + .05)]
    source_candidates = []
    for name, wall_w in (('leftWall', box.w_range[0]),
                         ('rightWall', box.w_range[1])):
        band = body[np.abs(body[:, 0] - wall_w) <= SIDE_BAND_M]
        faces = [face for face in supported_faces(band, 0)
                 if abs(face.at((box.rear_l + box.front_l) / 2,
                                (floor_h + top_h) / 2) - wall_w) <= SIDE_BAND_M]
        if faces:
            face = max(faces, key=lambda item: len(item.cells))
            plane = np.c_[band[:, [1, 2]], np.ones(len(band))] @ face.coefficients
            source = band[np.abs(band[:, 0] - plane) <= FACE_RESIDUAL_M]
            source_candidates.append((len(face.cells), name, face, source))
    if not source_candidates:
        return _empty(), {'reason': 'observed_side_face_unavailable',
                          'synthesizedPoints': 0}
    _, source_name, face, source = max(source_candidates, key=lambda item: item[0])
    bottom_h = max(box.h_range[0], floor_h - .10)
    upper_h = box.h_range[1]
    spacing = _surface_spacing(source, box.length * (upper_h - bottom_h))
    l_grid = np.arange(box.rear_l + spacing / 2, box.front_l, spacing)
    h_grid = np.arange(bottom_h + spacing / 2, upper_h, spacing)
    # 壁深同样优先取源侧实测形状（水槽、波纹随镜像传递），源侧空洞
    # 处回退拟合平面；两块面板共享同一形状场，只翻转 W。
    ll, hh = np.meshgrid(l_grid, h_grid)
    shape_query = np.column_stack((ll.ravel(), hh.ravel()))
    source_wall_w = box.w_range[0] if source_name == 'leftWall' else box.w_range[1]
    shape_source = body[np.abs(body[:, 0] - source_wall_w) <= SIDE_BAND_M]
    shape = _sample_shape_depths(shape_source, (1, 2), 0, shape_query, spacing)
    plane_depth = (np.c_[shape_query, np.ones(len(shape_query))]
                   @ face.coefficients)
    depth = np.where(np.isnan(shape), plane_depth, shape)
    source_panel = np.column_stack((depth, ll.ravel(), hh.ravel()))
    mirror_panel = source_panel.copy()
    mirror_panel[:, 0] = 2 * axis_w - source_panel[:, 0]
    opposite = 'rightWall' if source_name == 'leftWall' else 'leftWall'
    parts = []
    counts = {}
    for name, candidate in ((source_name, source_panel), (opposite, mirror_panel)):
        target_w = box.w_range[0] if name == 'leftWall' else box.w_range[1]
        candidate = candidate[np.abs(candidate[:, 0] - target_w) <= SIDE_BAND_M]
        observed = body[np.abs(body[:, 0] - target_w) <= SIDE_BAND_M]
        plane = np.c_[observed[:, [1, 2]], np.ones(len(observed))] @ face.coefficients
        if name != source_name:
            plane = 2 * axis_w - plane
        observed = observed[np.abs(observed[:, 0] - plane) <= FACE_RESIDUAL_M]
        current = np.vstack((existing, *parts)) if parts else existing
        added = _surface_holes(observed, candidate, (1, 2), current, spacing)
        parts.append(added)
        counts[name] = {
            'reason': 'observed_side_mirrored_full_surface' if len(added)
                      else 'no_surface_gap',
            'sourceSide': source_name, 'observedFaceCells': len(face.cells),
            'sourcePoints': len(source), 'spacingM': round(spacing, 4),
            'completedHeightRangeM': [round(bottom_h, 4), round(upper_h, 4)],
            'synthesizedPoints': len(added),
        }
    return np.vstack(parts), counts


def _complete_floor(
    points: np.ndarray, existing: np.ndarray, box: BoxCarriageBox,
    inner_w: tuple[float, float], floor_h: float,
) -> tuple[np.ndarray, dict]:
    w0, w1 = inner_w
    floor = points[(points[:, 0] >= w0 + .08) & (points[:, 0] <= w1 - .08)
                   & (points[:, 1] >= box.rear_l + .12)
                   & (points[:, 1] <= box.front_l - .12)
                   & (np.abs(points[:, 2] - floor_h) <= .08)]
    if len(floor) < MIN_FLOOR_BINS * MIN_FLOOR_BIN_POINTS:
        return _empty(), {'reason': 'floor_support_insufficient', 'synthesizedPoints': 0}
    indices = np.floor((floor[:, 1] - box.rear_l) / FLOOR_BIN_M).astype(int)
    supported = []
    for index in np.unique(indices):
        slab = floor[indices == index]
        if (len(slab) >= MIN_FLOOR_BIN_POINTS
                and np.ptp(slab[:, 0]) >= (w1 - w0) * .55):
            supported.append((float(np.median(slab[:, 1])),
                              float(np.median(slab[:, 2]))))
    if (len(supported) < MIN_FLOOR_BINS
            or supported[-1][0] - supported[0][0] < MIN_FLOOR_RUN_M):
        return _empty(), {'reason': 'floor_longitudinal_support_insufficient',
                          'synthesizedPoints': 0}
    middle_l = (box.rear_l + box.front_l) / 2
    rear = floor[floor[:, 1] <= middle_l]
    front = floor[floor[:, 1] > middle_l]
    source_half = 'rear' if len(rear) >= len(front) else 'front'
    source = rear if source_half == 'rear' else front
    supported_l = [length for length, _ in supported
                   if (length <= middle_l if source_half == 'rear'
                       else length > middle_l)]
    if len(supported_l) < MIN_FLOOR_BINS:
        return _empty(), {'reason': 'floor_half_support_insufficient',
                          'synthesizedPoints': 0}
    near_center = max(supported_l) if source_half == 'rear' else min(supported_l)
    center_gap = 2 * abs(middle_l - near_center)
    if center_gap > box.length * MAX_FLOOR_CENTER_GAP_FRACTION:
        return _empty(), {'reason': 'floor_center_gap_unbounded',
                          'centerGapM': round(center_gap, 4),
                          'synthesizedPoints': 0}
    # 每个 15cm 实测小格等权拟合底板，不让一条密集扫描线主导坡度。
    grid = np.floor(source[:, :2] / FLOOR_BIN_M).astype(int)
    _, inverse = unique_cells(grid, return_inverse=True)
    samples = np.array([np.median(source[inverse == i], axis=0)
                        for i in range(int(inverse.max()) + 1)])
    coefficients = np.linalg.lstsq(
        np.c_[samples[:, :2], np.ones(len(samples))], samples[:, 2], rcond=None)[0]
    if np.linalg.norm(coefficients[:2]) > .15:
        return _empty(), {'reason': 'floor_plane_slope_unreasonable',
                          'synthesizedPoints': 0}
    observed_area = (w1 - w0 - .16) * (supported[-1][0] - supported[0][0])
    spacing = float(np.clip(np.sqrt(observed_area / len(floor)),
                            MIN_FLOOR_SPACING_M, MAX_FLOOR_SPACING_M))
    w_grid = np.arange(w0 + .08, w1 - .08, spacing)
    l_grid = np.arange(box.rear_l + .12, box.front_l - .12, spacing)
    if not len(w_grid) or not len(l_grid):
        return _empty(), {'reason': 'floor_grid_empty', 'synthesizedPoints': 0}
    ww, ll = np.meshgrid(w_grid, l_grid)
    # 前后对称只用于确定缺失区域，不能将底板基面的坡度也反向镜像。
    # H=aW+bL+c 在目标 L 上求值，保证倾斜建系/真实斜底仍是一块连续平面，
    # 不在中线造出 V 形折角。源半幅决定拟合，已有点不移动。
    hh = coefficients[0] * ww + coefficients[1] * ll + coefficients[2]
    candidate = np.column_stack((ww.ravel(), ll.ravel(), hh.ravel()))
    # 上方已有实测回波的位置不是底板缺测孔。三维去重无法发现货面下的
    # 假底板，按水平投影保守留洞（包括杆件下方），不修改任何实测点。
    interior = points[(points[:, 0] >= w0 + .08) & (points[:, 0] <= w1 - .08)
                      & (points[:, 1] >= box.rear_l + .12)
                      & (points[:, 1] <= box.front_l - .12)]
    heights = np.c_[interior[:, :2], np.ones(len(interior))] @ coefficients
    elevated = interior[interior[:, 2] > heights + .12]
    blocked = np.zeros(len(candidate), dtype=bool)
    if len(elevated):
        distances, _ = cKDTree(elevated[:, :2]).query(candidate[:, :2], k=1)
        blocked = distances <= FLOOR_BIN_M
        candidate = candidate[~blocked]
    added = _surface_holes(floor, candidate, (0, 1), existing, spacing)
    mirrored = added[:, 1] > middle_l if source_half == 'rear' \
        else added[:, 1] < middle_l
    return added, {
        'reason': 'observed_floor_front_back_mirror' if len(added)
                  else 'no_surface_gap',
        'sourceHalf': source_half, 'mirrorPlaneL': round(middle_l, 6),
        'centerSeamM': round(center_gap, 4),
        'observedFloorBins': len(supported), 'spacingM': round(spacing, 4),
        'synthesizedPoints': len(added),
        'occludedFloorCandidates': int(np.count_nonzero(blocked)),
        'mirroredHalfPoints': int(np.count_nonzero(mirrored)),
        'heightModel': 'continuous_floor_plane',
        'heightExtrapolated': bool(len(added) and (
            added[:, 1].min() < source[:, 1].min()
            or added[:, 1].max() > source[:, 1].max())),
    }


def complete_rectangular_floor(
    points: np.ndarray, box: BoxCarriageBox, inner_w: tuple[float, float],
    floor_h: float, config: Config, *, observed_points: np.ndarray | None = None,
) -> tuple[np.ndarray, dict]:
    """已有四壁模型仅补底板；支撑和遮挡证据取本次实测点。"""
    values = np.asarray(points if observed_points is None else observed_points, dtype=float)
    if (not config.carriage_box_completion_enable or values.ndim != 2
            or values.shape[1:] != (3,) or not len(values)
            or not np.isfinite(values).all() or not np.isfinite(floor_h)):
        return _empty(), {'reason': 'floor_support_insufficient', 'synthesizedPoints': 0}
    return _complete_floor(values, np.asarray(points, dtype=float), box, inner_w, floor_h)


def complete_rectangular_interior_gaps(
    points: np.ndarray, box: BoxCarriageBox,
    inner_w: tuple[float, float], floor_h: float, top_h: float, config: Config,
    *, existing_points: np.ndarray | None = None,
) -> RectangularCompletion:
    """完整镜像补四壁及底板；任何原始点都不修改。"""
    empty = _empty()
    values = np.asarray(points, dtype=float)
    existing = values if existing_points is None else np.asarray(existing_points, dtype=float)
    axis_w = (inner_w[0] + inner_w[1]) / 2
    outer_axis = sum(box.w_range) / 2
    if (not config.carriage_box_completion_enable or values.ndim != 2
            or values.shape[1:] != (3,) or len(values) < 1000
            or not np.isfinite(values).all()
            or existing.ndim != 2 or existing.shape[1:] != (3,)
            or not len(existing) or not np.isfinite(existing).all()
            or not np.isfinite([axis_w, floor_h, top_h]).all()
            or abs(axis_w - outer_axis) > .10 or top_h - floor_h < .8):
        return RectangularCompletion(empty, {
            'completed': False, 'reason': 'rectangular_symmetry_not_verified',
            'synthesizedPoints': 0,
        }, empty)
    rear, rear_info = _complete_end(
        values, existing, box, box.rear_l, axis_w, floor_h, top_h)
    front, front_info = _complete_end(
        values, np.vstack((existing, rear)), box, box.front_l, axis_w, floor_h, top_h)
    sides, side_info = _complete_sides(
        values, np.vstack((existing, rear, front)), box, axis_w, floor_h, top_h)
    floor, floor_info = _complete_floor(
        values, np.vstack((existing, rear, front, sides)), box, inner_w, floor_h)
    parts = [part for part in (rear, front, sides, floor) if len(part)]
    result = np.vstack(parts) if parts else empty
    return RectangularCompletion(result, {
        'completed': bool(len(result)),
        'method': 'observed_rectangular_four_wall_and_floor_mirror',
        'symmetryPlane': {'coordinateFrame': 'vehicle_local_WLH',
                          'normalWLH': [1., 0., 0.], 'offsetW': round(axis_w, 6)},
        'floorMirrorPlaneL': floor_info.get('mirrorPlaneL'),
        'regionPointCounts': {
            'rearWall': len(rear), 'frontWall': len(front),
            'sideWalls': len(sides), 'frontFloor': len(floor),
        },
        'rearWall': rear_info, 'frontWall': front_info,
        'sideWalls': side_info, 'frontFloor': floor_info,
        'synthesizedPoints': len(result),
        'observedPointCount': len(existing),
        'sourceSupportPointCount': len(values),
        'observedPointsModified': 0,
        'intensity': {'status': 'unavailable', 'reason': 'source_pcd_xyz_only'},
        'syntheticUsedForLoadEvidence': False,
        'syntheticUsedAsObservedInnerEdge': False,
    }, floor)


def completed_cloud_dimensions(
    observed: np.ndarray, synthetic: np.ndarray, box: BoxCarriageBox,
    *, percentile_low: float, percentile_high: float,
) -> dict | None:
    """补点后完整点云的外包络尺寸。

    observed 须是与输出净化 PCD 相同的补点前实测局部坐标，synthetic 是
    本次镜像补点。整车口径与正式尺寸一致：长度占用跨度、宽高分位数；
    箱体口径取四壁选择带内的三轴占用跨度。合成点被限制在观测箱体
    边界容差内，整车包络差应接近零；箱体包络可能因补齐缺失壁面而
    变完整，该差异在此显式披露。正式箱体外宽使用补点后包络。
    """
    values = np.asarray(observed, dtype=float)
    extra = np.asarray(synthetic, dtype=float)
    if (not len(extra) or values.ndim != 2 or values.shape[1:] != (3,)
            or extra.ndim != 2 or extra.shape[1:] != (3,)
            or not np.isfinite(values).all() or not np.isfinite(extra).all()):
        return None

    def vehicle_envelope(cloud: np.ndarray) -> dict:
        lo, hi = occupancy_extent(cloud[:, 1])
        return {
            'lengthM': round(float(hi - lo), 4),
            'widthM': round(robust_span(cloud[:, 0], percentile_low, percentile_high), 4),
            'heightM': round(robust_span(cloud[:, 2], percentile_low, percentile_high), 4),
        }

    def carriage_envelope(cloud: np.ndarray) -> dict | None:
        # 带容差收紧到箱体边界附近，避免把箱外车架/悬伸计入箱体包络；
        # occupancy 的每 bin 点数门槛再排除孤立外凸物（门锁、把手）。
        band = cloud[(cloud[:, 1] >= box.rear_l - ENVELOPE_BAND_M)
                     & (cloud[:, 1] <= box.front_l + ENVELOPE_BAND_M)
                     & (cloud[:, 0] >= box.w_range[0] - ENVELOPE_BAND_M)
                     & (cloud[:, 0] <= box.w_range[1] + ENVELOPE_BAND_M)
                     & (cloud[:, 2] >= box.h_range[0] - ENVELOPE_BAND_M)
                     & (cloud[:, 2] <= box.h_range[1] + ENVELOPE_BAND_M)]
        if not len(band):
            return None
        spans = {}
        for key, axis in (('lengthM', 1), ('widthM', 0), ('heightM', 2)):
            lo, hi = occupancy_extent(band[:, axis])
            spans[key] = round(float(hi - lo), 4)
        return spans

    completed = np.vstack((values, extra))
    observed_vehicle = vehicle_envelope(values)
    completed_vehicle = vehicle_envelope(completed)
    observed_carriage = carriage_envelope(values)
    completed_carriage = carriage_envelope(completed)
    if observed_carriage is None or completed_carriage is None:
        return None
    return {
        'source': 'observed_plus_synthetic_envelope',
        'definition': 'completed_cloud_envelope',
        'completedCloud': {'vehicle': completed_vehicle,
                           'carriageOuter': completed_carriage},
        'observedOnly': {'vehicle': observed_vehicle,
                         'carriageOuter': observed_carriage},
        'maxVehicleEnvelopeDeltaM': round(max(
            abs(completed_vehicle[key] - observed_vehicle[key])
            for key in observed_vehicle), 4),
        'maxCarriageEnvelopeDeltaM': round(max(
            abs(completed_carriage[key] - observed_carriage[key])
            for key in observed_carriage), 4),
        'observedPointCount': int(len(values)),
        'synthesizedPointCount': int(len(extra)),
    }


def _face_peak(values: np.ndarray) -> float | None:
    """主体壁面深度：1cm 直方图众数峰的中位（防外凸锁杆拉偏中位）。"""
    if not len(values) or not np.isfinite(values).all():
        return None
    if float(values.min()) == float(values.max()):
        return float(values[0])
    bins = np.arange(float(values.min()), float(values.max()) + .01, .01)
    counts, _ = np.histogram(values, bins=bins)
    peak = int(np.argmax(counts))
    selected = ((values >= bins[peak]) & (values < bins[peak + 1]))
    # np.histogram 的最后一格包含右边界，取中位时保持相同的装桶定义。
    if peak == len(counts) - 1:
        selected |= values == bins[peak + 1]
    return float(np.median(values[selected]))


def _gutter_floor(values: np.ndarray, main: float, inward: int,
                  deep: bool = False) -> tuple[float, float, int]:
    """水槽底深度；返回（最窄净空面, 深层峰面, 凹进点数）。

    端壁（deep=True）顶部水槽带存在多层结构：门楣台阶（约 5~12cm）与
    顶梁内凹层（20~30cm，开顶箱顶沿加强梁）。此时最窄净空面取凹进点
    中最内侧 10% 的中位（与人工在点云上量取最内可触结构一致），深层
    峰面取凹进 >=12cm 区域内 2cm 直方图最高峰（count>=15）的中位。
    侧壁（deep=False）凹进均匀，取最内侧 20% 的中位。凹进点不足 10
    个或凹进不足 3cm 时判无水槽证据，返回主面。
    """
    recessed = values[values * inward > main * inward + GUTTER_MIN_RECESS_M]
    if len(recessed) < 10:
        return main, main, 0
    ordered = np.sort(recessed)
    fraction = 10 if deep else 20
    inner = (ordered[-max(3, len(ordered) * fraction // 100):] if inward > 0
             else ordered[:max(3, len(ordered) * fraction // 100)])
    narrow = float(np.median(inner))
    deep_values = recessed[(recessed - main) * inward >= END_GUTTER_DEEP_RECESS_M]
    layer = narrow
    if len(deep_values) >= 15:
        bins = np.arange(float(deep_values.min()), float(deep_values.max()) + .02, .02)
        counts, edges = np.histogram(deep_values, bins=bins)
        peak = int(np.argmax(counts))
        if counts[peak] >= 15:
            layer = float(np.median(
                deep_values[(deep_values >= edges[peak]) & (deep_values < edges[peak + 1])]))
    if (narrow - main) * inward < GUTTER_MIN_RECESS_M:
        return main, main, 0
    return narrow, layer, len(recessed)


def completed_inner_dimensions(
    observed: np.ndarray, synthetic: np.ndarray, box: BoxCarriageBox,
    floor_z: float, wall_top: float, config: Config | None = None,
) -> dict | None:
    """从实测与补点组成的最终云图计算集装箱内长、内宽、内高。

    仅适用于已识别的短矩形箱车（集装箱）：调用方须先经
    ``observe_rectangular_body`` 按现场长宽窗口（默认 5.7~6.4m ×
    2.4~2.7m）确认车型并完成四壁镜像补点。普通长车（栏板车/厢式
    长车）不进入该路径——长度窗口即车型门禁，长车 ``observed_box``
    为 None，走原 ``complete_empty_carriage_walls`` 分支，不得调用
    本函数。

    observed 须是与输出净化 PCD 相同的补点前实测局部坐标，synthetic 是
    本次镜像补点，二者均在车辆局部 WLH 坐标。口径（经 11:49 真实箱车
    与人工 CloudCompare 实测对齐验证）：

    * 主体壁面：各壁水槽带以下区域 1cm 直方图众数峰（全高可用内尺寸，
      与 ISO 标准内长口径同源）；
    * 扣水槽净空：水槽高度带（顶沿向下 35cm）内最内连续结构面的间距。
      侧壁为顶侧梁内面（凹进约 12cm）；端壁分层取最窄面（顶梁内凹
      最内 10% 中位，约 24~26cm）与层中心（约 21~22cm）双口径；
    * 内高：与普通车厢共用侧壁上沿归属、同段底板和两侧有效段中位数。

    返回的水槽净空长宽及内高是该车型的业务尺寸来源；同时保留主体
    壁面和顶梁分层值用于解释测量口径。输入非法时显式返回 None。
    """
    values = np.asarray(observed, dtype=float)
    extra = np.asarray(synthetic, dtype=float)
    if (not len(extra) or values.ndim != 2 or values.shape[1:] != (3,)
            or extra.ndim != 2 or extra.shape[1:] != (3,)
            or not np.isfinite(values).all() or not np.isfinite(extra).all()
            or not np.isfinite([floor_z, wall_top]).all()
            or wall_top <= floor_z):
        return None
    full = np.vstack((values, extra))
    top = box.h_range[1]
    gutter_lo = top - GUTTER_BAND_M
    mid_l = (full[:, 1] >= box.rear_l + .5) & (full[:, 1] <= box.front_l - .5)
    mid_w = (full[:, 0] >= box.w_range[0] + .3) & (full[:, 0] <= box.w_range[1] - .3)

    faces = {}
    for name, axis, boundary, inward, deep in (
            ('leftWall', 0, box.w_range[0], +1, False),
            ('rightWall', 0, box.w_range[1], -1, False),
            ('rearWall', 1, box.rear_l, +1, True),
            ('frontWall', 1, box.front_l, -1, True),
    ):
        # 侧壁在 L 中部量取（避开端壁角部）；端壁在 W 中部量取（避开
        # 角柱），带宽 0.35 覆盖顶梁内凹层（最深约 30cm）。
        band_width = SIDE_BAND_M + .05 if axis == 0 else .35
        lateral = mid_l if axis == 0 else mid_w
        band = full[(np.abs(full[:, axis] - boundary) <= band_width) & lateral]
        if len(band) < 20:
            return None
        main = _face_peak(band[band[:, 2] < gutter_lo, axis])
        if main is None:
            return None
        narrow, layer, count = _gutter_floor(
            band[(band[:, 2] >= gutter_lo) & (band[:, 2] < top + .02), axis],
            main, inward, deep)
        faces[name] = {'mainM': round(main, 4), 'narrowM': round(narrow, 4),
                       'layerM': round(layer, 4),
                       'recessM': round((narrow - main) * inward, 4),
                       'evidencePoints': int(count)}
    left, right = faces['leftWall'], faces['rightWall']
    rear, front = faces['rearWall'], faces['frontWall']
    cfg = config or Config()
    carriage = full[(full[:, 1] >= box.rear_l) & (full[:, 1] <= box.front_l)]
    # 已确认的主体壁面与内折沿属于同一侧壁。仅为测高把平行壁层展开到
    # 主壁面：H/L 不变，避免未扫到折角连接面时把内折沿当成另一块孤立物。
    # 只展开原面残差内的点；车厢云、补点和实际宽度坐标均不改写。
    height_cloud = carriage.copy()
    wall_mask = np.zeros(len(carriage), dtype=bool)
    for face in (left, right):
        mask = ((np.abs(carriage[:, 0] - face['mainM']) <= FACE_TOLERANCE_M)
                | (np.abs(carriage[:, 0] - face['narrowM']) <= FACE_TOLERANCE_M))
        height_cloud[mask, 0] = face['mainM']
        wall_mask |= mask
    interior = ((carriage[:, 0] > left['narrowM']) & (carriage[:, 0] < right['narrowM']))
    height_cloud = height_cloud[wall_mask | interior]
    profile = CarriageAnalyzer(cfg).analyze(height_cloud)
    height, height_metadata = _measure_continuous_side_height(height_cloud, profile, cfg)
    return {
        'source': 'observed_plus_synthetic_completed_cloud',
        'definition': 'completed_cloud_gutter_clearance_business_dimensions',
        # 显式标明适用车型：仅短矩形箱车（集装箱）路径输出本对象；
        # 普通长车 observed_box 为 None，不产生该字段。
        'appliesTo': 'observed_short_rectangular_box_vehicle',
        'gutterBandM': GUTTER_BAND_M,
        'widthMainM': round(right['mainM'] - left['mainM'], 4),
        'widthGutterClearanceM': round(right['narrowM'] - left['narrowM'], 4),
        'lengthMainM': round(front['mainM'] - rear['mainM'], 4),
        'lengthGutterClearanceM': round(front['narrowM'] - rear['narrowM'], 4),
        'lengthGutterLayerM': round(front['layerM'] - rear['layerM'], 4),
        'heightM': round(height, 4) if np.isfinite(height) else None,
        'heightMeasurement': height_metadata,
        'perFace': faces,
    }
