"""已发现拉筋的同侧局部定位；不改根数、原轴线或原始点云。

发现阶段的内部搜索带有意避开侧壁，不能当作端部实测缺失范围。
本模块统一从当前最终车厢云取支持，邻域形状在未裁成细管的点云中验证，
防止把墙面/底板裁成一条细带后误称横杆。输出仍是杆轴与指定测量面的
交点及纵向投影，不声称观测到了焊点或固定板的接触边缘。

格占用连续判据只允许把被两侧直接支持格夹住、且格内仍有实测点的 1~2 格
低密度内部空格补成等权格；端部空格、真空洞与整段丢失仍判缺测，因此
observedWRangeM 两端始终只由直接支持格决定。
"""

import numpy as np
from scipy.spatial import cKDTree

from carpointcloud_analyzer.domain.longitudinal_measurement import MAX_RIB_EXTRAPOLATION_M
from carpointcloud_analyzer.domain.rib_support import MIN_TRANSVERSE_AXIS_W, continuous_support_runs, conflicting_rib_sides, stable_axis_prediction, validated_rib_support

TUBE_RADIUS_M = .07
SIDE_WINDOW_M = .85
CELL_M = .05
CELL_MIN_POINTS = 3
INTERIOR_CELL_MIN_POINTS = 2
MAX_INTERIOR_CELL_GAP = 2
MAX_INTERIOR_BRIDGE_CELLS = 2
NEIGHBOR_RADIUS_M = .12


def _axis(rib):
    if rib.axis_midpoint is None or rib.axis_direction is None:
        return None
    point, direction = np.asarray(rib.axis_midpoint, dtype=float), np.asarray(rib.axis_direction, dtype=float)
    if point.shape != (3,) or direction.shape != (3,) or not np.isfinite([point, direction]).all():
        return None
    norm = np.linalg.norm(direction)
    if not np.isfinite(norm) or norm < 1e-6:
        return None
    direction = direction / norm
    return (point, direction) if abs(direction[0]) >= MIN_TRANSVERSE_AXIS_W else None


def _intersection(point, direction, plane):
    a, b, c = plane
    denominator = direction[0] - a * direction[1] - b * direction[2]
    if abs(denominator) < .5:
        return None
    return point + ((a * point[1] + b * point[2] + c - point[0]) / denominator) * direction


def _bridge_interior_cells(occupied, counts):
    """仅在逐格连续判据本会失败时，补齐低密度内部格；绝不移动观测范围端点。

    CELL_MIN_POINTS 门槛用来稳住格心位置和实测跨度两端，零散点不能把范围撑大。
    夹在两个直接支持格之间的空格不改动 observedWRangeM，所以放行只命中
    INTERIOR_CELL_MIN_POINTS 个点的内部格；单段与整根合计都不得超过
    MAX_INTERIOR_BRIDGE_CELLS。端部空格任一侧没有直接支持格，一律不补。

    已有连续支持（最大格间隔 ≤ MAX_INTERIOR_CELL_GAP）时直接原样返回，使
    本来就能出杆位的车次结果逐位不变，改动只落在原先被误判缺测的杆上。
    """
    if len(occupied) < 3:
        return occupied, []
    if np.max(np.diff(occupied)) <= MAX_INTERIOR_CELL_GAP:
        return occupied, []
    known = set(occupied)
    bridged, run = [], []
    for key in range(min(occupied), max(occupied) + 1):
        if key not in known:
            run.append(key)
            continue
        if (run and len(run) <= MAX_INTERIOR_BRIDGE_CELLS
                and len(bridged) + len(run) <= MAX_INTERIOR_BRIDGE_CELLS
                and all(counts.get(cell, 0) >= INTERIOR_CELL_MIN_POINTS for cell in run)):
            bridged.extend(run)
        run = []
    if not bridged:
        return occupied, []
    return sorted(known.union(bridged)), sorted(bridged)


def _fit(cells, target_w, plane):
    design = np.c_[cells[:, 0] - target_w, np.ones(len(cells))]
    coefficients = np.linalg.lstsq(design, cells[:, 1:], rcond=None)[0]
    point = np.r_[target_w, coefficients[1]]
    direction = np.r_[1., coefficients[0]]
    direction /= np.linalg.norm(direction)
    intersection = _intersection(point, direction, plane)
    residual = np.percentile(np.abs(design @ coefficients - cells[:, 1:]), 90, axis=0)
    return intersection, direction, residual


def _whole_rib_position(values, tree, rib, measurement, floor_h, top_h, diagnostics):
    """短窗口不足时，用最终云额外的全杆支持求同一个侧面交点。"""
    side = measurement.metadata['selectedSide']
    sign = 1 if side == 'left' else -1
    plane = np.asarray(measurement.side_plane)
    model = validated_rib_support(values, tree, rib, floor_h, top_h, diagnostics=diagnostics)
    if model is None or side in conflicting_rib_sides(values, tree, rib, floor_h, top_h):
        return None
    cells = model.cells
    distances = sign * (cells[:, 0] - np.c_[cells[:, 1:3], np.ones(len(cells))] @ plane)
    # 必须真有窗口外的额外连续杆段，不能仅将原短窗口换一套宽松门槛。
    if not continuous_support_runs(cells[distances > SIDE_WINDOW_M]):
        return None
    near_runs = continuous_support_runs(cells[(distances > .04) & (distances < SIDE_WINDOW_M)])
    if not near_runs:
        return None
    def side_distance(run):
        return float(np.min(sign * (run[:, 0] - np.c_[run[:, 1:3], np.ones(len(run))] @ plane)))

    near_cells = min(near_runs, key=side_distance)
    nearest = side_distance(near_cells)
    if not 0 <= nearest <= MAX_RIB_EXTRAPOLATION_M:
        return None
    intersection = model.intersect_side(plane)
    if intersection is None:
        return None
    near, _, _ = _fit(near_cells, intersection[0], plane)
    if near is None:
        return None
    if (np.any(np.abs(near[1:] - intersection[1:]) > [.03, .10])
            and not stable_axis_prediction(model, intersection[0])):
        return None
    wlo, whi = float(cells[:, 0].min()), float(cells[:, 0].max())
    extrapolation = float(max(wlo - intersection[0], intersection[0] - whi, 0.))
    projected = float(measurement.project_longitudinal(intersection))
    if (extrapolation > MAX_RIB_EXTRAPOLATION_M
            or not measurement.rear_l < projected < measurement.front_l):
        return None
    return {'ribId': rib.rib_id, 'valid': True, 'reason': 'ok',
            'method': 'final_cloud_whole_rib_axis_side_intersection',
            'positionM': projected, 'extrapolationM': extrapolation,
            'observedWRangeM': [wlo, whi], 'supportCellsWLH': cells.tolist(),
            'supportCellCount': len(cells), 'residualP90M': model.residual.tolist(),
            'intersectionWLH': intersection.tolist(), 'selectedSide': side,
            'referencePlaneWLH': plane.tolist(),
            'positionDefinition': 'whole_rib_axis_at_side_measurement_plane_projected_L'}


def refine_rib_side_positions(points, ribs, measurement, floor_h, top_h,
                              *, final_cloud=False):
    """在同一车厢云中定位杆与内壁面的纵向交点。

    ``final_cloud`` 保留调用兼容；补点后仍须通过相同的几何检查，
    不能用发现阶段的旧轴覆盖局部杆件不成立的结论。
    """
    def missing(rib, reason):
        return {'ribId': rib.rib_id, 'valid': False, 'positionM': None, 'reason': reason,
                'method': 'final_cloud_rib_axis_side_intersection'}

    if not ribs:
        return []
    try:
        values = np.asarray(points, dtype=float)
        plane = np.asarray(measurement.side_plane, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return [missing(rib, 'invalid_rib_side_input') for rib in ribs]
    side = measurement.metadata.get('selectedSide')
    if (not measurement.valid or plane.shape != (3,) or not np.isfinite(plane).all()
            or side not in ('left', 'right') or values.ndim != 2 or values.shape[1:] != (3,)
            or len(values) < 20 or not np.isfinite(values).all()
            or not np.isfinite([floor_h, top_h]).all() or top_h <= floor_h):
        return [missing(rib, 'invalid_rib_side_input') for rib in ribs]
    # 去重、排序使重复回波和点序不改变支持强度或网格中位数。
    values = np.unique(values, axis=0)
    tree = cKDTree(values)
    sign = 1 if side == 'left' else -1  # 从测量面向车厢内部为正。
    distance_to_side = sign * (values[:, 0] - np.c_[values[:, 1:3], np.ones(len(values))] @ plane)
    result = []
    for rib in ribs:
        row = missing(rib, 'insufficient_local_rib_support')
        axis = _axis(rib)
        if axis is None:
            result.append({**row, 'reason': 'rib_axis_unavailable'})
            continue
        point, direction = axis
        prior_range = np.asarray(rib.axis_observed_w_range, dtype=float)
        if (prior_range.shape != (2,) or not np.isfinite(prior_range).all()
                or prior_range[1] <= prior_range[0]):
            result.append({**row, 'reason': 'rib_seed_support_unavailable'})
            continue
        seed = _intersection(point, direction, plane)
        if seed is None:
            result.append({**row, 'reason': 'rib_axis_nearly_parallel_to_side'})
            continue
        distance = np.linalg.norm(np.cross(values - point, direction), axis=1)
        ids = np.flatnonzero((distance < TUBE_RADIUS_M)
                             & (distance_to_side > .04) & (distance_to_side < SIDE_WINDOW_M)
                             & (values[:, 2] > floor_h + .08) & (values[:, 2] < top_h + .05))
        # 在未裁细管的邻域判断线性/方向。墙面/固定板的二维支持不能作杆端证据。
        supported = []
        for index, neighbors in zip(ids, tree.query_ball_point(values[ids], NEIGHBOR_RADIUS_M)):
            if len(neighbors) < 6:
                continue
            eigenvalues, eigenvectors = np.linalg.eigh(np.cov(values[neighbors].T))
            if (eigenvalues[1] / max(eigenvalues[2], 1e-12) < .35
                    and abs(eigenvectors[:, -1] @ direction) > .9):
                supported.append(index)
        observed = values[supported]
        row['supportPointCount'] = len(observed)
        if len(observed) < 20:
            result.append(row)
            continue
        keys = np.floor(sign * (observed[:, 0] - seed[0]) / CELL_M).astype(int)
        counts = {int(key): int(np.count_nonzero(keys == key)) for key in np.unique(keys)}
        occupied = sorted(key for key, count in counts.items() if count >= CELL_MIN_POINTS)
        row['supportCellCount'] = len(occupied)
        # 回波局部变稀时管内点仍在，只是每格不足 3 个；被两侧直接支持格夹住的
        # 内部空格可补进等权格，端部空格不可，故实测范围端点仍只由直接支持格决定。
        occupied, bridged = _bridge_interior_cells(occupied, counts)
        if bridged:
            row['bridgedInteriorCellCount'] = len(bridged)
        cells = np.array([np.median(observed[keys == key], axis=0) for key in occupied])
        # 等空间格权重，连续覆盖和跨度必须成立；零散点不能撑大实测范围。
        if (len(cells) < 7 or np.ptp(cells[:, 0]) < .30
                or len(occupied) / (max(occupied) - min(occupied) + 1) < .7
                or np.max(np.diff(occupied)) > MAX_INTERIOR_CELL_GAP):
            result.append({**row, 'reason': 'disconnected_local_rib_support'})
            continue
        anchor = cells[(cells[:, 0] >= prior_range[0]) & (cells[:, 0] <= prior_range[1])]
        if (len(anchor) < 3 or np.ptp(anchor[:, 0]) < .10
                or np.median(np.linalg.norm(np.cross(anchor - point, direction), axis=1)) > .05):
            result.append({**row, 'reason': 'local_rib_not_connected_to_seed_support'})
            continue
        intersection, local_direction, residual = _fit(cells, seed[0], plane)
        row['residualP90M'] = residual.tolist()
        if (intersection is None or not np.isfinite(intersection).all()
                or np.max(residual) > .035 or abs(local_direction @ direction) < .96):
            result.append({**row, 'reason': 'ambiguous_local_rib_axis'})
            continue
        # 格占用连续不代表同一根杆：相邻平行支架可在这里拼出低残差的假斜杆。
        slopes = local_direction[1:] / local_direction[0]
        width_steps = np.diff(cells[:, 0])
        steps = np.diff(cells[:, 1:], axis=0)
        jumps = np.abs(steps - width_steps[:, None] * slopes)
        # 整段回归的斜率会被错位支架拉偏，反过来抵消真实的纵向跳变。
        # 用相邻杆段斜率中位数去趋势，正常斜杆保留，补洞也不能掩盖断差。
        longitudinal_slope = np.median(steps[:, 0] / width_steps)
        jumps[:, 0] = np.abs(steps[:, 0] - width_steps * longitudinal_slope)
        if np.any(jumps[:, 0] > .035):
            # L 本身断裂时，中位数可能落在支架与杆之间，不能称纵向共识。
            # 此明确歧义也不可由末尾最终云的旧轴回退重新放行。
            result.append({**row, 'reason': 'local_rib_longitudinal_discontinuity'})
            continue
        if np.any(jumps > [.035, .07]):
            # 仅高度跳变会令三维杆轴断续，但纵向 L 位置仍可由整段
            # 同侧实测杆点直接约束。这里只发布杆身 L 中位位置，不冒称
            # 已量到杆与侧壁的物理交点，也不放宽局部交点外推上限。
            longitudinal = cells[:, 1]
            p10, p90 = np.percentile(longitudinal, [10., 90.])
            side_distances = sign * (
                cells[:, 0] - np.c_[cells[:, 1:3], np.ones(len(cells))] @ plane)
            nearest_side = float(np.min(side_distances))
            position = float(np.median(longitudinal))
            representative_h = float(cells[np.argmin(side_distances), 2])
            reference_w = float(np.r_[position, representative_h, 1.] @ plane)
            reference_point = [reference_w, position, representative_h]
            projected = float(measurement.project_longitudinal(reference_point))
            discontinuities = int(np.count_nonzero(np.any(jumps > [.035, .07], axis=1)))
            if (len(cells) >= 10 and np.ptp(cells[:, 0]) >= .45
                    and p90 - p10 <= .05 and abs(position - rib.y_position) <= .04
                    and abs(slopes[0]) <= .10 and discontinuities <= 2
                    and 0 <= nearest_side <= .30
                    and measurement.rear_l < projected < measurement.front_l):
                row.update(valid=True, reason='ok_supported_rod_center',
                           method='local_rib_longitudinal_consensus',
                           positionM=projected,
                           positionDefinition='direct_local_rib_longitudinal_consensus',
                           extrapolationM=None,
                           sideSupportDistanceM=nearest_side,
                           positionSpreadP10P90M=float(p90 - p10),
                           discontinuityCount=discontinuities,
                           observedWRangeM=[float(np.min(cells[:, 0])),
                                            float(np.max(cells[:, 0]))],
                           supportCellsWLH=cells.tolist(),
                           intersectionWLH=reference_point,
                           referencePointDefinition='side_plane_at_observed_rod_L_not_contact',
                           selectedSide=side, referencePlaneWLH=plane.tolist())
                result.append(row)
                continue
            result.append({**row, 'reason': 'local_rib_axis_discontinuity'})
            continue
        # 同一截面重复出现分离的纵向细带时，不把双影/平行杆的中位数当真杆。
        split_cells = 0
        sparse_split_keys = []
        for key in occupied:
            group = observed[keys == key]
            separated = False
            sparse_separated = False
            for coordinate in (1, 2):
                offsets = np.sort(group[:, coordinate] - slopes[coordinate - 1] * group[:, 0])
                runs = np.split(offsets, np.flatnonzero(np.diff(offsets) > .020) + 1)
                band_centers = [(len(run), np.median(run)) for run in runs if len(run) >= 2]
                centers = [center for count, center in band_centers if count >= 3]
                separated |= len(centers) > 1 and np.ptp(centers) > .040
                sparse_centers = [center for _, center in band_centers]
                sparse_separated |= len(sparse_centers) > 1 and np.ptp(sparse_centers) > .040
            split_cells += separated
            if sparse_separated:
                sparse_split_keys.append(key)
        sparse_runs = np.split(sparse_split_keys, np.flatnonzero(np.diff(sparse_split_keys) > 1) + 1)
        # 每簇仅两点时，连续至少三格仍是两条杆线；不能取其中位数发布假轴。
        if (split_cells >= max(3, int(np.ceil(len(cells) * .25)))
                or any(len(run) >= 3 for run in sparse_runs)):
            result.append({**row, 'reason': 'multiple_local_rib_axes',
                           'splitCellCount': int(split_cells),
                           'sparseSplitWidthCellKeys': [int(key) for key in sparse_split_keys]})
            continue
        # 近侧半段与全段交点互验，防远处密杆把弯曲/分叉的固定端拉走。
        near_count = max(5, (len(cells) + 1) // 2)
        near, _, _ = _fit(cells[:near_count], seed[0], plane)
        if near is None:
            result.append({**row, 'reason': 'unstable_local_rib_intersection'})
            continue
        sensitivity = np.abs(near[1:] - intersection[1:])
        row['fitSensitivityM'] = sensitivity.tolist()
        if (sensitivity[0] > .03 or sensitivity[1] > .10
                or abs(intersection[1] - seed[1]) > .07
                or abs(intersection[2] - seed[2]) > .12):
            result.append({**row, 'reason': 'unstable_local_rib_intersection'})
            continue
        # 范围由有连续空间支持的格中心确定，不能借孤立边点或旧外推端点扩张。
        wlo, whi = float(cells[:, 0].min()), float(cells[:, 0].max())
        extrapolation = max(wlo - intersection[0], intersection[0] - whi, 0.)
        row.update(observedWRangeM=[wlo, whi], extrapolationM=float(extrapolation),
                   supportCellsWLH=cells.tolist(), intersectionWLH=intersection.tolist(),
                   selectedSide=side, referencePlaneWLH=plane.tolist(),
                   positionDefinition='local_rib_axis_at_side_measurement_plane_projected_L')
        projected = float(measurement.project_longitudinal(intersection))
        if extrapolation > MAX_RIB_EXTRAPOLATION_M:
            result.append({**row, 'reason': 'rib_side_extrapolation_exceeds_limit'})
        elif not measurement.rear_l < projected < measurement.front_l:
            result.append({**row, 'reason': 'rib_side_projection_outside_inner_length'})
        else:
            result.append({**row, 'valid': True, 'reason': 'ok', 'positionM': projected})
    for index, (rib, row) in enumerate(zip(ribs, result)):
        # 明确的双轴不是点数不足，不能由更长的单轴部分将它稀释。
        if not row['valid'] and row['reason'] != 'multiple_local_rib_axes':
            diagnostics = {}
            recovered = _whole_rib_position(values, tree, rib, measurement, floor_h, top_h, diagnostics)
            if recovered is not None:
                result[index] = {**recovered, 'localWindowReason': row['reason']}
            elif diagnostics:
                row['wholeRibSupport'] = diagnostics
                if diagnostics['reason'] == 'multiple_whole_rib_axes':
                    # 全杆已经找到竞争轴时，不能再把最初短窗口的缺点/断带当作主因。
                    row['localWindowReason'] = row['reason']
                    row['reason'] = 'multiple_whole_rib_axes'
    return result
