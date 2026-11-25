"""量最终车厢云的主体侧壁前后端长度，排除顶部篷布附加部分。"""

from dataclasses import replace

import numpy as np

from carpointcloud_analyzer.domain.inner_faces import (
    FACE_CELL_M, FACE_SEARCH_HALF_BAND_M, FACE_TOLERANCE_M,
)
from carpointcloud_analyzer.domain.longitudinal_measurement import _side_candidates
from carpointcloud_analyzer.domain.models import LongitudinalMeasurement
from carpointcloud_analyzer.domain.utils import occupancy_extent
from carpointcloud_analyzer.domain.utils.cells import unique_cells


# 沿用主体壁面搜索带，避开底板及顶部篷布附加件。
BODY_TOP_FRACTION = .75
BODY_FLOOR_CLEARANCE_M = .15
BALANCE_GRID_M = .02
MIN_COMMON_HEIGHT_M = .40


def _balanced_points(points):
    """每个三维占用格取最靠近格心的原点，不求夹层均值；格内薄层不分辨。"""
    values = np.unique(points, axis=0)
    grid = np.floor(values / BALANCE_GRID_M).astype(np.int64)
    center_distance = np.sum((values - (grid + .5) * BALANCE_GRID_M) ** 2, axis=1)
    order = np.argsort(center_distance, kind='stable')
    _, indices = unique_cells(grid[order], return_index=True)
    return values[order[indices]]


def _body_height_band(floor, top):
    hlo = floor + BODY_FLOOR_CLEARANCE_M
    return hlo, max(floor + BODY_TOP_FRACTION * (top - floor), min(top - .05, hlo + 4 * FACE_CELL_M))


def _side_extent_stages(values, rear, front, hlo, hhi):
    return (
        ('body_height_band', (values[:, 2] >= hlo) & (values[:, 2] <= hhi)
         & (values[:, 1] >= rear - FACE_SEARCH_HALF_BAND_M)
         & (values[:, 1] <= front + FACE_SEARCH_HALF_BAND_M)),
        ('top_view_between_end_walls', (values[:, 1] >= rear) & (values[:, 1] <= front)),
    )


def _near_side(values, face):
    predicted = np.c_[values[:, 1:3], np.ones(len(values))] @ face.coefficients
    return np.abs(values[:, 0] - predicted) <= 3 * FACE_TOLERANCE_M


def _reaches_end_walls(side_rear, side_front, rear, front):
    return side_rear <= rear + FACE_SEARCH_HALF_BAND_M and side_front >= front - FACE_SEARCH_HALF_BAND_M


def select_rib_side_reference(measurement, observed, rear, front, floor, top, completion):
    """在覆盖达标且到达两端的实测壁面上定位拉筋，完整侧优先。"""
    seams = completion.get('cornerSeams', [])
    patched = {seam['side'] for seam in seams if seam.get('synthesizedPoints', 0) > 0}
    repaired = {seam['side'] for seam in seams
                if seam.get('longGapHeightRows', 0) > 0}
    mirror = completion.get('sideWall', {})
    if mirror.get('repaired'):
        repaired.add(mirror['side'])
    # 覆盖只计分割出的实测侧壁，沿用选面的二维及纵向门槛。先查未补侧，
    # 再查常规角缝、镜像或长桥所在侧；同级按原始覆盖排序。都量不到时保留原测线。
    candidates = _side_candidates(observed, rear, front, floor, top)
    def repair_priority(item):
        side = 'left' if item[2] < 0 else 'right'
        return side in repaired, side in patched

    candidates.sort(key=repair_priority)
    stages = _side_extent_stages(observed, rear, front, *_body_height_band(floor, top))
    for coverage, length_coverage, sign, face in candidates:
        near = _near_side(observed, face)
        for stage, window in stages:
            points = _balanced_points(observed[near & window])
            if not len(points):
                continue
            side_rear, side_front = occupancy_extent(points[:, 1])
            if _reaches_end_walls(side_rear, side_front, rear, front):
                side = 'left' if sign < 0 else 'right'
                if (side == measurement.metadata.get('selectedSide')
                        and side not in repaired and side not in patched):
                    return measurement
                reference = dict(source='observed_side_wall', selectedSide=side,
                                 sidePlane=face.coefficients.tolist(), sideCoverage=float(coverage),
                                 sideLongitudinalCoverage=float(length_coverage), sideExtentStage=stage)
                return replace(measurement, side_plane=tuple(face.coefficients), metadata=reference)
    return measurement


def measure_final_inner_end_faces(points, rear, front, floor, top,
                                  ground_plane_local=None, inner_face_endpoints=False,
                                  front_endpoint=None,
                                  front_endpoint_metadata=None) -> LongitudinalMeasurement:
    """量主体侧壁的前后端跨度；保留调用名，拉筋仍投影到所选侧壁。

    主体高度沿用原搜索带，排除前后顶部篷布附加部分。将主体面投影到纵向后
    量占用跨度，避免局部遮挡在各高度层形成不同端点。有标定面时将纵轴投影
    到地磅平面，不能让不同高度的侧壁端点把高度差混入长度。
    ``inner_face_endpoints`` 表示 ``rear``/``front`` 是已实测的后、前端壁内表面；
    此时侧壁只用于确认两端都到达端壁并给出测线，内长端点就取两内表面。
    ``front_endpoint`` 为顶部开口前沿（与车头距同一前端）；给出时前端点改取它，
    侧壁到达前端壁的核验仍以前端壁内表面 ``front`` 为准。``front_endpoint_metadata``
    为开口前沿诊断（含量不到的原因），原样记入 ``topOpeningFrontEdge``。
    """
    metadata = {
        'source': 'current_final_carriage_cloud',
        'method': 'body_side_wall_longitudinal_extent',
        'definition': 'main_side_wall_front_to_rear_length',
        'occupancyGridM': BALANCE_GRID_M,
        'faceToleranceM': FACE_TOLERANCE_M,
        'valid': False,
        **({'topOpeningFrontEdge': front_endpoint_metadata}
           if front_endpoint_metadata is not None else {}),
    }

    if inner_face_endpoints:
        metadata['definition'] = ('side_wall_length_between_end_wall_inner_faces'
                                  if front_endpoint is None else
                                  'rear_end_wall_inner_face_to_top_opening_front_edge')

    def unavailable(reason):
        return LongitudinalMeasurement(None, None, None, {**metadata, 'reason': reason})

    values = np.asarray(points, dtype=float)
    if (values.ndim != 2 or values.shape[1:] != (3,) or len(values) < 200
            or not np.isfinite(values).all() or not np.isfinite([rear, front, floor, top]).all()
            or front - rear <= 2.0 or top - floor <= .5):
        return unavailable('invalid_final_cloud')
    sides = _side_candidates(values, rear, front, floor, top,
                             min_coverage=.20, min_length_coverage=.50)
    if not sides:
        return unavailable('inner_side_plane_unavailable')
    hlo, hhi = _body_height_band(floor, top)
    if hhi - hlo < MIN_COMMON_HEIGHT_M + .10:
        return unavailable('insufficient_body_height')
    # 侧别按原二维覆盖排序；原分割锚点只限定搜索区，不替代实测侧壁端点。
    # 先在主体高度带内量取，篷布附加部分因高于主体带而不计。俯视雷达在
    # 远端常只看到侧壁上部（或底板相对局部系倾斜），高度带内侧壁到不了
    # 端壁时，改在俯视中量侧壁全高占用，并截止于两道端壁：端壁之外的
    # 顶部篷布附加部分同样不计。
    stages = _side_extent_stages(values, rear, front, hlo, hhi)
    short_of_end_walls = False
    for stage, window in stages:
        for coverage, longitudinal_coverage, sign, side_face in sides:
            side = _balanced_points(values[_near_side(values, side_face) & window])
            if not len(side):
                continue
            rear_l, front_l = occupancy_extent(side[:, 1])
            # 主体侧壁两端须到达两道端壁的搜索带，不能把较短的可见段当成内长。
            if not _reaches_end_walls(rear_l, front_l, rear, front):
                short_of_end_walls = True
                continue
            axis = None
            if ground_plane_local is not None:
                normal = np.asarray(ground_plane_local[:3], dtype=float)
                normal = normal / np.linalg.norm(normal)
                axis = np.array([0., 1., 0.]) - normal[1] * normal
                axis /= np.linalg.norm(axis)
                rear_l, front_l = occupancy_extent(side @ axis)
                metadata.update(longitudinalAxisWLH=axis.tolist(),
                                coordinateFrame='weighbridge_horizontal_longitudinal')
            if inner_face_endpoints:
                # 业务内长为前后端壁内表面的距离。侧壁到达两端已由上面的搜索带
                # 核验；端点取分割实测的两内表面（与车头距前壁、底板起点同一位置），
                # 不计侧壁伸进端壁厚度与顶沿的部分，也不受末端扫描间距影响。
                # 有标定面时在侧壁测线中部高度处投影到磅面水平纵轴。
                w_ref, h_ref = (float(np.median(side[:, 0])), float(np.median(side[:, 2])))
                front_end = front if front_endpoint is None else float(front_endpoint)
                ends = np.array([[w_ref, rear, h_ref], [w_ref, front_end, h_ref]])
                rear_l, front_l = ends @ axis if axis is not None else ends[:, 1]
                metadata.update(endpointSource=(
                    'end_wall_inner_faces' if front_endpoint is None
                    else 'rear_end_wall_inner_face_to_top_opening_front_edge'))
            metadata.update(valid=True, reason='ok', selectedSide='left' if sign < 0 else 'right',
                            sidePlane=side_face.coefficients.tolist(), sideCoverage=float(coverage),
                            sideLongitudinalCoverage=float(longitudinal_coverage),
                            ribReference='selected_inner_side_axis_projection',
                            bodyHeightRangeM=([float(hlo), float(hhi)]
                                              if stage == 'body_height_band' else None),
                            **({'sideExtentStage': stage}
                               if stage != 'body_height_band' else {}),
                            sideBodyPointCount=len(side),
                            lengthM=float(front_l - rear_l), rearM=float(rear_l), frontM=float(front_l))
            return LongitudinalMeasurement(float(rear_l), float(front_l),
                                           tuple(float(x) for x in side_face.coefficients), metadata,
                                           longitudinal_axis=None if axis is None else tuple(axis))
        if not short_of_end_walls:
            break
    return unavailable('side_wall_body_short_of_end_walls' if short_of_end_walls
                       else 'side_wall_endpoints_unobserved')
