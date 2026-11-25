"""当前内长的侧壁选面、拉筋投影及纵向分段诊断。"""

from copy import deepcopy
from typing import Optional

import numpy as np

from carpointcloud_analyzer.domain.inner_faces import supported_faces
from carpointcloud_analyzer.domain.models import LongitudinalMeasurement, RibInfo
from carpointcloud_analyzer.domain.utils.cells import unique_cells

# 检测器排除壁旁杆件，允许有限外推并披露；缺少半根横杆不能无限延长。
# 最终云补齐后，侧壁前方的遮挡带可达约 30 cm；仍保持有限外推。
MAX_RIB_EXTRAPOLATION_M = 0.30


def _finite_diagnostic(value):
    """缺测诊断也必须可严格序列化；禁止把NaN/Infinity带入HTTP响应。"""
    if isinstance(value, dict):
        return {key: _finite_diagnostic(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite_diagnostic(item) for item in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


def _finite_number(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and np.isfinite(value))


def _interior_bridge_count(row):
    """桥接内部格弱于直接支持格：只认正整数，布尔、负数与缺项一律按 0。"""
    value = row.get('bridgedInteriorCellCount')
    return value if isinstance(value, int) and not isinstance(value, bool) and value > 0 else 0


def _vector3(value):
    try:
        vector = np.asarray(value, dtype=float)
        return vector if vector.shape == (3,) and np.isfinite(vector).all() else None
    except (TypeError, ValueError, OverflowError):
        return None


def _side_candidates(points, rear, front, floor, top, *,
                     min_coverage=.5, min_length_coverage=.85):
    """按等面积网格覆盖选侧，不能按点数、最短长度或固定左右选侧。"""
    body = points[(points[:, 1] > rear + .5) & (points[:, 1] < front - .5)
                  & (points[:, 2] > floor + .15) & (points[:, 2] < top - .10)]
    if len(body) < 200:
        return []
    wlo, whi = np.percentile(body[:, 0], [1, 99])
    center, margin = (wlo + whi) / 2, (whi - wlo) / 4
    if margin < .3:
        return []
    n_l = max(3, int(np.ceil((front - rear - 1.) / .25)))
    n_h = max(2, int(np.ceil((top - floor - .25) / .20)))
    origin = np.array([rear + .5, floor + .15])
    step = np.array([(front - rear - 1.) / n_l, (top - floor - .25) / n_h])
    results = []
    for sign in (-1, 1):
        side = body[(body[:, 0] - center) * sign > margin]
        faces = supported_faces(side, 0)
        if not faces:
            continue
        eligible = []
        for face in faces:
            residual = np.abs(np.c_[side[:, 1:3], np.ones(len(side))] @ face.coefficients
                              - side[:, 0])
            observed = side[residual <= .06]
            grid = np.floor((observed[:, 1:3] - origin) / step).astype(int)
            keys, counts = unique_cells(grid, return_counts=True)
            keys = keys[counts >= 3]
            keys = keys[(keys[:, 0] >= 0) & (keys[:, 0] < n_l)
                        & (keys[:, 1] >= 0) & (keys[:, 1] < n_h)]
            coverage = len(keys) / (n_l * n_h)
            length_coverage = len(np.unique(keys[:, 0])) / n_l
            if coverage >= min_coverage and length_coverage >= min_length_coverage:
                eligible.append((coverage, length_coverage, sign, face))
        if eligible:
            # 内外两层均可见时只从有足够二维支持的内侧面取测线，
            # 不能因外壳点多就沿外壳量“内长”；细横梁无法通过上述覆盖。
            results.append(min(eligible, key=lambda item: sign * item[3].at(
                (rear + front) / 2, (floor + top) / 2)))
    # 同分时固定左侧，保证点序变化不改变选择；不以端点测出的长短排序。
    return sorted(results, key=lambda item: (-item[0], -item[1], item[2]))


def rib_projections(ribs: list[RibInfo], measurement: LongitudinalMeasurement) -> Optional[list[dict]]:
    """核验同侧轴交点；局部杆身 L 共识单独标源，不冒充接触点。"""
    if not measurement.valid or measurement.side_plane is None:
        return None
    if len(measurement.side_plane) != 3 or not np.isfinite(measurement.side_plane).all():
        return None
    if measurement.rib_side_projections is not None:
        # 按ID而不是发现/导出列表顺序关联；已尝试后缺测不能悄悄借旧轴回填。
        rows = measurement.rib_side_projections
        if not isinstance(rows, list):
            rows = []
        reference = measurement.metadata.get('ribSideReference')
        expected_side = (reference or measurement.metadata).get('selectedSide')
        expected_plane = _vector3(reference.get('sidePlane') if reference else measurement.side_plane)
        result = []
        for rib in ribs:
            matches = [row for row in rows if isinstance(row, dict) and row.get('ribId') == rib.rib_id]
            if len(matches) != 1:
                result.append({'ribId': rib.rib_id, 'valid': False, 'positionM': None,
                               'reason': 'rib_side_identity_unavailable'})
                continue
            row = _finite_diagnostic(matches[0])
            position, extrapolation = row.get('positionM'), row.get('extrapolationM')
            center_consensus = (row.get('positionDefinition')
                                == 'direct_local_rib_longitudinal_consensus')
            if center_consensus:
                side_distance = row.get('sideSupportDistanceM')
                spread = row.get('positionSpreadP10P90M')
                support_valid = (
                    extrapolation is None
                    and _finite_number(side_distance) and 0 <= side_distance <= .30
                    and _finite_number(spread) and 0 <= spread <= .05)
            else:
                support_valid = (_finite_number(extrapolation)
                                 and 0 <= extrapolation <= MAX_RIB_EXTRAPOLATION_M)
            if row.get('valid') is not True or (
                    not _finite_number(position)
                    or not measurement.rear_l < position < measurement.front_l
                    or not support_valid):
                if row.get('valid') is not False:
                    row['reason'] = 'invalid_refined_rib_position'
                row.update(valid=False, positionM=None)
            if not row.get('valid'):
                row['positionM'] = None
            if row.get('valid'):
                plane = _vector3(row.get('referencePlaneWLH'))
                intersection = _vector3(row.get('intersectionWLH'))
                if (row.get('selectedSide') != expected_side or expected_plane is None
                        or plane is None or not np.allclose(plane, expected_plane, atol=1e-9, rtol=0)
                        or intersection is None
                        or abs(measurement.project_longitudinal(intersection) - position) > 1e-9
                        or abs(intersection[0] - np.r_[intersection[1:], 1.] @ plane) > 1e-8):
                    row.update(valid=False, positionM=None, reason='rib_side_reference_mismatch')
            result.append(row)
        return _check_rib_order(ribs, result)
    a, b, c = measurement.side_plane
    result = []
    for rib in ribs:
        missing = {'ribId': rib.rib_id, 'valid': False, 'positionM': None,
                   'reason': 'rib_axis_unavailable'}
        if (rib.axis_midpoint is None or rib.axis_direction is None
                or rib.axis_observed_w_range is None):
            result.append(missing)
            continue
        point, direction = np.asarray(rib.axis_midpoint), np.asarray(rib.axis_direction)
        observed = np.asarray(rib.axis_observed_w_range)
        if (point.shape != (3,) or direction.shape != (3,) or observed.shape != (2,)
                or not np.isfinite([point, direction]).all() or not np.isfinite(observed).all()
                or observed[1] <= observed[0] or np.linalg.norm(direction) < 1e-6):
            result.append(missing)
            continue
        direction = direction / np.linalg.norm(direction)
        denominator = direction[0] - a * direction[1] - b * direction[2]
        if abs(denominator) < .5:
            result.append({**missing, 'reason': 'rib_axis_nearly_parallel_to_side'})
            continue
        t = (a * point[1] + b * point[2] + c - point[0]) / denominator
        intersection = point + t * direction
        extrapolation = max(float(observed[0] - intersection[0]),
                            float(intersection[0] - observed[1]), 0.)
        if extrapolation > MAX_RIB_EXTRAPOLATION_M:
            result.append({**missing, 'reason': 'rib_side_extrapolation_exceeds_limit',
                           'extrapolationM': extrapolation, 'observedWRangeM': observed.tolist()})
            continue
        projected = float(measurement.project_longitudinal(intersection))
        if not measurement.rear_l <= projected <= measurement.front_l:
            result.append({**missing, 'reason': 'rib_side_projection_outside_inner_length'})
            continue
        result.append({'ribId': rib.rib_id, 'valid': True, 'positionM': projected,
                       'extrapolationM': extrapolation,
                       'observedWRangeM': observed.tolist()})
    return _check_rib_order(ribs, result)


def _check_rib_order(ribs, result):
    # 同侧投影不能相交或重合，不靠重排序掩盖错误身份关联。
    order = sorted(range(len(ribs)), key=lambda index: -ribs[index].y_position)
    previous = None
    crossed = set()
    for index in order:
        if not result[index]['valid']:
            continue
        if previous is not None and result[index]['positionM'] >= result[previous]['positionM'] - 1e-6:
            crossed.update((previous, index))
        previous = index
    for index in crossed:
        result[index].update(valid=False, positionM=None, reason='rib_axes_cross_on_selected_side')
    return result


def rib_positions(ribs: list[RibInfo], measurement: LongitudinalMeasurement) -> Optional[list[float]]:
    """缺少同侧投影证据时，不借拉筋中点冒充。"""
    projections = rib_projections(ribs, measurement)
    return (None if projections is None or not all(row['valid'] for row in projections)
            else [row['positionM'] for row in projections])


def measurement_metadata(measurement: LongitudinalMeasurement, ribs: list[RibInfo]) -> dict:
    projections = rib_projections(ribs, measurement)
    valid = projections is not None and all(row['valid'] for row in projections)
    extrapolation = max((row['extrapolationM'] for row in projections or []
                         if _finite_number(row.get('extrapolationM'))), default=0.)
    # 保留几何来源和弱支持证据，但不产生依赖人工审核的业务状态。
    bridged = sum(_interior_bridge_count(row) for row in projections or [])
    metadata = deepcopy(measurement.metadata)
    center_consensus_used = False
    if measurement.rib_side_projections is not None:
        by_id = {row['ribId']: row for row in projections or []}
        positions = [by_id.get(rib.rib_id, {}).get('positionM')
                     for rib in sorted(ribs, key=lambda item: -item.y_position)]

        def intervals(front, rear):
            nodes = [front, *positions, rear]
            return [float(start - end) if start is not None and end is not None and start >= end
                    else None for start, end in zip(nodes[:-1], nodes[1:])]

        center_consensus_used = any(
            row.get('positionDefinition') == 'direct_local_rib_longitudinal_consensus'
            for row in projections or [])
        metadata.update(ribPositionSource=(
                            'mixed_side_intersection_and_supported_rod_center'
                            if center_consensus_used
                            else 'local_observed_rib_axis_same_side_intersection'),
                        ribPositionDefinition=(
                            'longitudinal_projection_or_supported_rod_center_not_weld_or_edge'
                            if center_consensus_used
                            else 'longitudinal_projection_not_weld_or_edge_measurement'),
                        ribIntervalsM=intervals(measurement.front_l, measurement.rear_l))
        upper = metadata.get('upperReference')
        if upper is not None:
            ends = upper.get('endpoints', {})
            front = ends.get('front', {})
            rear = ends.get('rear', {})
            upper['ribIntervalsM'] = intervals(front.get('positionM') if front.get('valid') else None,
                                               rear.get('positionM') if rear.get('valid') else None)
            upper['ribPositionDefinition'] = 'same_rib_L_projection_with_upper_wall_references'
    return {**metadata, 'ribProjections': projections,
            'ribPositionsValid': valid,
            'maximumRibExtrapolationM': MAX_RIB_EXTRAPOLATION_M,
            'inferredGeometry': bool(measurement.metadata.get('completionUsed')
                                     or extrapolation > .03 or bridged
                                     or center_consensus_used or not valid)}
