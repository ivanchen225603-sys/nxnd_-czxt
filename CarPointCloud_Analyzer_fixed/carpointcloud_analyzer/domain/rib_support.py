"""补杆与贴壁定位共用的当前车厢云细杆几何，不把孤岛或异轴支架接成杆。"""

from dataclasses import dataclass

import numpy as np

CELL_M = .05
NEIGHBOR_RADIUS_M = .12
AXIS_LONGITUDINAL_TOLERANCE_M = .035
COMPETING_AXIS_RADIUS_M = .15
MIN_TRANSVERSE_AXIS_W = .5


def axis_support_points(values, tree, point, direction, floor_h, top_h, radius):
    """在完整邻域验证一维杆形，不能将壁面裁成细管后冒充杆。"""
    direction = direction / np.linalg.norm(direction)
    distance = np.linalg.norm(np.cross(values - point, direction), axis=1)
    ids = np.flatnonzero((distance < radius) & (values[:, 2] > floor_h + .08)
                         & (values[:, 2] < top_h + .05))
    return _linear_neighborhood_support(values, tree, ids, np.broadcast_to(direction, (len(ids), 3)))


def _linear_neighborhood_support(values, tree, ids, directions):
    """两种杆形搜索共用完整邻域的一维性与切向验证。"""
    supported = []
    for index, direction, neighbors in zip(ids, directions, tree.query_ball_point(values[ids], NEIGHBOR_RADIUS_M)):
        if len(neighbors) < 6:
            continue
        eigenvalues, eigenvectors = np.linalg.eigh(np.cov(values[neighbors].T))
        if (eigenvalues[1] / max(eigenvalues[2], 1e-12) < .35
                and abs(eigenvectors[:, -1] @ direction) > .9):
            supported.append(index)
    return values[supported]


def axis_support_cells(values, tree, point, direction, floor_h, top_h, radius):
    """将一维杆支持归入等权宽度格，避免单侧密度改变轴线。"""
    observed = axis_support_points(values, tree, point, direction, floor_h, top_h, radius)
    if not len(observed):
        return np.empty((0, 3))
    keys = np.floor(observed[:, 0] / CELL_M).astype(int)
    return np.asarray([np.median(observed[keys == key], axis=0)
                       for key in np.unique(keys) if np.count_nonzero(keys == key) >= 3]
                      ).reshape(-1, 3)


def profile_support_points(values, tree, model, floor_h, top_h, radius):
    """沿已验证杆形关联当前云，不用旧直线细管丢弃弯曲杆的端部。"""
    centers = model.evaluate(values[:, 0])
    ids = np.flatnonzero((np.linalg.norm(values - centers, axis=1) < radius)
                         & (values[:, 2] > floor_h + .08) & (values[:, 2] < top_h + .05))
    widths = values[ids, 0]
    directions = model.evaluate(widths + .01) - model.evaluate(widths - .01)
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    return _linear_neighborhood_support(values, tree, ids, directions)


def continuous_support_runs(cells):
    """至少三个相邻格、10cm 的连续杆段；孤立小片不能扩张补点范围。"""
    keys = np.floor(cells[:, 0] / CELL_M).astype(int)
    runs = np.split(cells, np.flatnonzero(np.diff(keys) > 1) + 1)
    return [run for run in runs if len(run) >= 3 and np.ptp(run[:, 0]) >= .10 - 1e-9]


def stable_axis_extension(model, target_w: float, inner_width: float) -> bool:
    """覆盖半厢宽，且连续留出块不改变壁边预测时允许有界补端。"""
    cells = model.cells
    span = float(np.ptp(cells[:, 0]))
    if (span < inner_width * .5 or len(cells) * CELL_M < inner_width * .5
            or max(cells[0, 0] - target_w, target_w - cells[-1, 0], 0.) > span):
        return False
    return stable_axis_prediction(model, target_w)


def stable_axis_prediction(model, target_w: float) -> bool:
    """连续留出块验证整个杆身对同一目标横坐标的预测。"""
    cells = model.cells
    span = float(np.ptp(cells[:, 0]))
    expected = model.evaluate(np.asarray([target_w]))[0, 1:]
    # 两个仅半长的短拟合会把厘米级格心抖动放大成远端不一致。
    # 连续五折留出仍移除整片支持，而不是只验证交错的邻近重复点。
    for held_out in np.array_split(np.arange(len(cells)), 5):
        retained = np.delete(cells, held_out, axis=0)
        if len(retained) < 7 or np.ptp(retained[:, 0]) < span * .5:
            return False
        candidate = _fit_support_profile(retained, height_degree=len(model.height_coefficients) - 1)
        endpoint = candidate.evaluate(np.asarray([target_w]))[0, 1:]
        if np.any(np.abs(endpoint - expected) > [.03, .10]):
            return False
    return True


@dataclass(frozen=True)
class RibSupportAxis:
    """当前云等权重拟合的单根杆形及有限支持范围，纵向 L 始终线性。"""

    point: np.ndarray
    direction: np.ndarray
    cells: np.ndarray
    residual: np.ndarray
    height_coefficients: np.ndarray

    def evaluate(self, widths: np.ndarray) -> np.ndarray:
        """L 由当前杆身线性拟合；H 在支持内平滑拟合，支持外仅沿端部切线延伸。"""
        widths = np.asarray(widths, dtype=float)
        longitudinal = self.point[1] + (widths - self.point[0]) * self.direction[1] / self.direction[0]
        height = np.polynomial.polynomial.polyval(widths - self.point[0], self.height_coefficients)
        if len(self.height_coefficients) > 2:
            for left in (True, False):
                edge = self.cells[0 if left else -1, 0]
                outside = widths < edge if left else widths > edge
                if not np.any(outside):
                    continue
                near = self.cells[np.abs(self.cells[:, 0] - edge) <= .40]
                if len(near) < 5:
                    near = self.cells[:5] if left else self.cells[-5:]
                design = np.c_[near[:, 0] - edge, np.ones(len(near))]
                slope = np.linalg.lstsq(design, near[:, 2], rcond=None)[0][0]
                edge_h = np.polynomial.polynomial.polyval(edge - self.point[0], self.height_coefficients)
                height[outside] = edge_h + (widths[outside] - edge) * slope
        return np.c_[widths, longitudinal, height]

    def intersect_side(self, plane: np.ndarray) -> np.ndarray | None:
        """与完整 W=aL+bH+c 内壁方程求交，不把弯曲杆重新裁进旧直线细管。"""
        a, b, c = plane
        width = float(a * self.point[1] + b * self.point[2] + c)
        for _ in range(32):
            point = self.evaluate(np.asarray([width]))[0]
            tangent = self.evaluate(np.asarray([width + .001, width - .001]))
            direction = tangent[0] - tangent[1]
            direction /= np.linalg.norm(direction)
            if abs(direction[0] - a * direction[1] - b * direction[2]) < .5:
                return None
            updated = float(a * point[1] + b * point[2] + c)
            if not np.isfinite(updated):
                return None
            if abs(updated - width) < 1e-10:
                return point
            width = updated
        return None


def _fit_support_profile(cells, *, height_degree=1) -> RibSupportAxis:
    """纵向位置与高度形状分别拟合，弯曲高度不改变纵向测距定义。"""
    target_w = float(np.median(cells[:, 0]))
    relative = cells[:, 0] - target_w
    coefficients = np.linalg.lstsq(np.c_[relative, np.ones(len(cells))], cells[:, 1:], rcond=None)[0]
    height_coefficients = np.polynomial.polynomial.polyfit(relative, cells[:, 2], height_degree)
    point = np.r_[target_w, coefficients[1]]
    direction = np.r_[1., coefficients[0]]
    direction /= np.linalg.norm(direction)
    model = RibSupportAxis(point, direction, cells, np.zeros(2), height_coefficients)
    residual = np.percentile(np.abs(model.evaluate(cells[:, 0])[:, 1:] - cells[:, 1:]), 90, axis=0)
    return RibSupportAxis(point, direction, cells, residual, height_coefficients)


def _persistent_step(cells):
    """只把有两侧连续段支持的台阶判为断轴；单格往返抖动不是另一根杆。"""
    widths = np.diff(cells[:, 0])
    steps = np.diff(cells[:, 1:], axis=0)
    slopes = np.median(steps / widths[:, None], axis=0)
    jumps = np.abs(steps - widths[:, None] * slopes)
    for index in np.flatnonzero(np.any(jumps > [.035, .07], axis=1)):
        before = cells[max(0, index - 2):index + 1]
        after = cells[index + 1:index + 4]
        if len(before) < 3 or len(after) < 3:
            continue
        # 相邻斜率中位数去趋势，不能让整根回归把平行台阶摊成斜杆。
        common_w = (cells[index, 0] + cells[index + 1, 0]) * .5
        centers = []
        for run in (before, after):
            centers.append(np.median(run[:, 1:] - (run[:, 0] - common_w)[:, None] * slopes, axis=0))
        if np.any(np.abs(centers[0] - centers[1]) > [.035, .07]):
            return True
    return False


def validated_rib_support(values, tree, rib, floor_h, top_h, *, _profile=None,
                          _passes_remaining=2, diagnostics=None) -> RibSupportAxis | None:
    """验证完整候选路径；低阶杆形重关联失败时再验证三次候选，不放宽门槛。"""
    details = {}
    model = _validated_rib_profile(values, tree, rib, floor_h, top_h, _profile=_profile,
                                   _passes_remaining=_passes_remaining, diagnostics=details)
    if model is None and details.get('reason') == 'whole_rib_shape_unresolved':
        # 二次模型在初始短段刚好通过，扩展到真实端部后可能不成立。
        # 不能让这个早停选择遮蔽同一云中有效的三次曲杆；双轴/断阶失败不重试。
        alternative_details = {}
        alternative = _validated_rib_profile(
            values, tree, rib, floor_h, top_h, _profile=_profile,
            _passes_remaining=_passes_remaining, diagnostics=alternative_details,
            height_degrees=(3,),
        )
        if alternative is not None:
            model, details = alternative, {}
        elif alternative_details.get('reason') in (
                'multiple_whole_rib_axes', 'persistent_whole_rib_axis_step'):
            details = alternative_details
    if diagnostics is not None:
        diagnostics.update(details)
    return model


def _validated_rib_profile(values, tree, rib, floor_h, top_h, *, _profile=None,
                           _passes_remaining=2, diagnostics=None, height_degrees=(2, 3)):
    """验证全杆形状、种子关联、持续断差及双轴；曲杆最多两轮有界再关联。"""
    def reject(reason, **details):
        if diagnostics is not None:
            diagnostics.update(reason=reason, **details)
        return None

    point = np.asarray(rib.axis_midpoint, dtype=float)
    direction = np.asarray(rib.axis_direction, dtype=float)
    prior = np.asarray(rib.axis_observed_w_range, dtype=float)
    if (point.shape != (3,) or direction.shape != (3,) or prior.shape != (2,)
            or not np.isfinite([point, direction]).all() or not np.isfinite(prior).all()
            or np.linalg.norm(direction) < 1e-6 or prior[1] <= prior[0]):
        return reject('invalid_rib_seed_axis')
    direction = direction / np.linalg.norm(direction)
    if abs(direction[0]) < MIN_TRANSVERSE_AXIS_W:
        return reject('rib_seed_axis_not_transverse')
    observed = (axis_support_points(values, tree, point, direction, floor_h, top_h, .07)
                if _profile is None else profile_support_points(values, tree, _profile, floor_h, top_h, .07))
    if len(observed) < 20:
        return reject('insufficient_whole_rib_points', supportPointCount=len(observed))
    keys = np.floor(observed[:, 0] / CELL_M).astype(int)
    occupied = [key for key in np.unique(keys) if np.count_nonzero(keys == key) >= 3]
    cells = np.asarray([np.median(observed[keys == key], axis=0) for key in occupied]).reshape(-1, 3)
    runs = continuous_support_runs(cells)
    if not runs:
        return reject('no_continuous_whole_rib_segment')
    cells = np.vstack(runs)
    anchor = cells[(cells[:, 0] >= prior[0]) & (cells[:, 0] <= prior[1])]
    if (len(cells) < 7 or np.ptp(cells[:, 0]) < .30 or len(anchor) < 3
            or np.ptp(anchor[:, 0]) < .10):
        return reject('insufficient_seed_connected_support', supportCellCount=len(cells))
    model = _fit_support_profile(cells)
    if model.residual[1] > .035 and len(cells) >= 14 and np.ptp(cells[:, 0]) >= .6:
        # 仅当高度直线不成立时考虑低阶曲线；交错留出验证避免为噪声增阶。
        for degree in height_degrees:
            candidate = _fit_support_profile(cells, height_degree=degree)
            if candidate.residual[1] > .035:
                continue
            errors = []
            for phase in (0, 1):
                train, held_out = cells[phase::2], cells[1 - phase::2]
                fit = _fit_support_profile(train, height_degree=degree)
                errors.extend(np.abs(fit.evaluate(held_out[:, 0])[:, 2] - held_out[:, 2]))
            if np.percentile(errors, 90) <= .035:
                model = candidate
                break
    if np.max(model.residual) > .035 or abs(model.direction @ direction) < .96:
        return reject('whole_rib_shape_unresolved', residualP90M=model.residual.tolist())
    steps = np.diff(cells[:, 1:], axis=0)
    widths = np.diff(cells[:, 0])
    slopes = np.median(steps / widths[:, None], axis=0)
    if _persistent_step(cells):
        return reject('persistent_whole_rib_axis_step')
    split_keys = []
    curved_height = model.evaluate(observed[:, 0])[:, 2] if len(model.height_coefficients) > 2 else None
    for key in occupied:
        group = observed[keys == key]
        separated = False
        for coordinate in (1, 2):
            trend = (curved_height[keys == key]
                     if coordinate == 2 and curved_height is not None
                     else slopes[coordinate - 1] * group[:, 0])
            offsets = np.sort(group[:, coordinate] - trend)
            bands = np.split(offsets, np.flatnonzero(np.diff(offsets) > .020) + 1)
            centers = [np.median(band) for band in bands if len(band) >= 3]
            separated |= len(centers) > 1 and np.ptp(centers) > .040
        if separated:
            split_keys.append(key)
    # 近侧短段的双轴不能被远处单轴格的数量稀释。
    split_runs = np.split(split_keys, np.flatnonzero(np.diff(split_keys) > 1) + 1)
    if (any(len(run) >= 3 for run in split_runs)
            or len(split_keys) >= max(3, int(np.ceil(len(occupied) * .25)))):
        return reject('multiple_whole_rib_axes', splitCellCount=len(split_keys),
                      splitWidthCellKeys=[int(key) for key in split_keys])
    expanded = (_profile is None or np.ptp(cells[:, 0]) > np.ptp(_profile.cells[:, 0]) + CELL_M)
    if expanded and _passes_remaining > 0 and len(model.height_coefficients) > 2:
        return _validated_rib_profile(values, tree, rib, floor_h, top_h,
                                      _profile=model, _passes_remaining=_passes_remaining - 1,
                                      diagnostics=diagnostics, height_degrees=height_degrees)
    return model


def conflicting_rib_sides(values, tree, rib, floor_h, top_h):
    """同一候选附近若有连续异轴杆段，该侧不能由种子轴补成新杆。"""
    point = np.asarray(rib.axis_midpoint, dtype=float)
    direction = np.asarray(rib.axis_direction, dtype=float)
    cells = axis_support_cells(values, tree, point, direction, floor_h, top_h,
                               COMPETING_AXIS_RADIUS_M)
    if not len(cells):
        return set()
    expected = point + ((cells[:, 0] - point[0]) / direction[0])[:, None] * direction
    conflict = cells[np.abs(cells[:, 1] - expected[:, 1]) > AXIS_LONGITUDINAL_TOLERANCE_M]
    runs = np.split(conflict, np.flatnonzero(np.diff(conflict[:, 0]) > CELL_M * 2.01) + 1)
    sides = set()
    for run in runs:
        if len(run) < 3 or np.ptp(run[:, 0]) < .10 - 1e-9:
            continue
        if run[:, 0].max() < rib.axis_observed_w_range[0]:
            sides.add('left')
        elif run[:, 0].min() > rib.axis_observed_w_range[1]:
            sides.add('right')
        else:
            sides.update(('left', 'right'))
    return sides
