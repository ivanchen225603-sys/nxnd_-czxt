"""全杆轮廓的合成几何回归：噪声、曲高与持续双轴必须分别判断。"""

import numpy as np
import pytest
from scipy.spatial import cKDTree

from carpointcloud_analyzer.domain import rib_support
from carpointcloud_analyzer.domain.models import LongitudinalMeasurement, RibInfo
from carpointcloud_analyzer.domain.rib_completion import complete_rib_gaps
from carpointcloud_analyzer.domain.rib_side_measurement import refine_rib_side_positions
from carpointcloud_analyzer.domain.rib_support import (
    _fit_support_profile,
    _persistent_step,
    axis_support_points,
    stable_axis_extension,
    validated_rib_support,
)


def _height(widths):
    return 1.35 - .30 * widths + .18 * (widths - .20) ** 2


def _curve_cells(mirror=1):
    widths = np.arange(-.625, 1.026, .05)
    cells = np.c_[mirror * widths, 4. + .06 * widths, _height(widths)]
    return cells[np.argsort(cells[:, 0])]


def _curve_cloud(mirror=1):
    widths = np.arange(-.65, 1.05, .005)
    points = _tube_points(widths, _height(widths), mirror)
    slope_h, intercept_h = np.polyfit(widths, _height(widths), 1)
    rib = RibInfo(
        1, 4., 0., intercept_h, (-.55, .55), (1., 2.), len(points), 1.1, .03,
        model_type='axis', axis_midpoint=(0., 4., intercept_h),
        axis_direction=(mirror, .06, slope_h), axis_observed_w_range=(-.55, .55),
    )
    return points, rib


def _tube_points(widths, heights, mirror):
    centers = np.c_[mirror * widths, 4. + .06 * widths, heights]
    angles = np.arange(8) * np.pi / 4
    offsets = np.c_[np.zeros(8), .004 * np.cos(angles), .004 * np.sin(angles)]
    return (centers[:, None, :] + offsets).reshape(-1, 3)


def _curved_end_cloud(mirror):
    widths = np.arange(-.95, .951, .005)
    points = _tube_points(widths, 1.25 - .30 * widths + .25 * widths ** 2, mirror)
    rib = RibInfo(
        1, 4., 0., 1.28, (-.55, .55), (1., 2.), len(points), 1.1, .03,
        model_type='axis', axis_midpoint=(0., 4., 1.28),
        axis_direction=(mirror, .06, -.30), axis_observed_w_range=(-.55, .55),
    )
    return points, rib


def _initial_curved_profile(points, rib):
    tree = cKDTree(points)
    observed = axis_support_points(points, tree, np.asarray(rib.axis_midpoint),
                                   np.asarray(rib.axis_direction), 0., 2.4, .07)
    keys = np.floor(observed[:, 0] / .05).astype(int)
    cells = np.array([np.median(observed[keys == key], axis=0)
                      for key in np.unique(keys) if np.count_nonzero(keys == key) >= 3])
    return tree, observed, _fit_support_profile(cells, height_degree=2)


def _sparse_near_wall_cloud(mirror, count):
    widths = np.arange(-.295, 1.046, .005)
    dense = _tube_points(widths, 1.28 - .30 * widths, mirror)
    widths = np.repeat(np.arange(-1.025, -.30, .05), count)
    offsets = np.tile([0.] if count == 1 else [-.004, .004], len(widths) // count)
    sparse = np.c_[mirror * widths, 4. + .06 * widths + offsets, 1.28 - .30 * widths]
    _, rib = _curved_end_cloud(mirror)
    measurement = LongitudinalMeasurement(
        0., 9., (0., 0., mirror * -1.12),
        {'selectedSide': 'left' if mirror == 1 else 'right'},
    )
    return np.vstack((sparse, dense)), rib, measurement


@pytest.mark.parametrize('mirror, offset, index', [
    (-1, -.05, 5), (1, -.04, 15), (-1, -.035, 25),
    (1, .035, 5), (-1, .04, 15), (1, .05, 25),
])
def test_single_longitudinal_cell_excursion_is_not_a_persistent_step(mirror, offset, index):
    widths = np.arange(-.775, .826, .05)
    cells = np.c_[mirror * widths, 4. + .18 * widths, 1.35 + .12 * widths]
    cells[index, 1] += offset
    cells = cells[np.argsort(cells[:, 0])]
    original = cells.copy()

    assert _persistent_step(cells) is False
    np.testing.assert_array_equal(cells, original)


@pytest.mark.parametrize('mirror, offset, slope_l', [
    (-1, -.045, -.20), (1, .045, -.20),
    (-1, .045, 0.), (1, -.045, 0.),
    (-1, -.045, .18), (1, .045, .18),
])
def test_persistent_parallel_longitudinal_step_is_not_fitted_into_a_diagonal(
        mirror, offset, slope_l):
    widths = np.arange(-.775, .826, .05)
    cells = np.c_[mirror * widths, 4. + slope_l * widths, 1.35 + .12 * widths]
    cells[widths < 0., 1] += offset
    cells = cells[np.argsort(cells[:, 0])]

    # 整体拟合的低残差不能代替两段之间的真实断差检查。
    assert _fit_support_profile(cells).residual[0] < .035
    assert _persistent_step(cells) is True


@pytest.mark.parametrize('mirror', [-1, 1])
@pytest.mark.parametrize('degree', [2, 3])
def test_curved_height_preserves_longitudinal_fit_and_uses_bounded_end_lines(mirror, degree):
    cells = _curve_cells(mirror)
    original = cells.copy()
    model = _fit_support_profile(cells, height_degree=degree)

    np.testing.assert_allclose(model.evaluate(cells[:, 0]), cells, atol=1e-12)
    for edge, sign in ((cells[0, 0], -1), (cells[-1, 0], 1)):
        widths = edge + sign * np.array([0., .1, .2, .3])
        evaluated = model.evaluate(widths)
        np.testing.assert_allclose(evaluated[:, 1], 4. + .06 * mirror * widths, atol=1e-12)
        np.testing.assert_allclose(np.diff(evaluated[:, 2], n=2), 0., atol=1e-12)
        assert abs(evaluated[-1, 2] - _height(mirror * widths[-1])) > .02
        assert np.isfinite(evaluated).all()
    np.testing.assert_array_equal(cells, original)


@pytest.mark.parametrize('mirror', [-1, 1])
@pytest.mark.parametrize('base_width', [-1.12, .20])
def test_curved_profile_intersects_tilted_side_plane_without_reusing_seed_line(mirror, base_width):
    model = _fit_support_profile(_curve_cells(mirror), height_degree=2)
    target_w = mirror * base_width
    target_h = (_height(base_width) if base_width == .20
                else model.evaluate(np.array([target_w]))[0, 2])
    expected = np.array([target_w, 4. + .06 * base_width, target_h])
    plane = np.array([.04, .05, target_w - .04 * expected[1] - .05 * expected[2]])

    actual = model.intersect_side(plane)

    assert actual is not None
    np.testing.assert_allclose(actual, expected, atol=1e-8)
    assert actual[0] == pytest.approx(plane @ np.r_[actual[1:], 1.], abs=1e-8)


@pytest.mark.parametrize('mirror', [-1, 1])
def test_extension_still_requires_half_width_and_limits_distance_to_support_span(mirror):
    cells = _curve_cells(mirror)
    model = _fit_support_profile(cells, height_degree=2)
    span = np.ptp(cells[:, 0])

    assert stable_axis_extension(model, mirror * -1.12, 2. * span + .01) is False
    beyond_span = cells[0, 0] - span - .01 if mirror == 1 else cells[-1, 0] + span + .01
    assert stable_axis_extension(model, beyond_span, 2.24) is False


@pytest.mark.parametrize('mirror', [-1, 1])
def test_single_curved_rod_has_a_valid_height_profile(mirror):
    points, rib = _curve_cloud(mirror)
    original = points.copy()
    model = validated_rib_support(points, cKDTree(points), rib, 0., 2.4)

    assert model is not None
    assert len(model.height_coefficients) > 2
    widths = np.array([-.2, .2, .6])
    evaluated = model.evaluate(mirror * widths)
    np.testing.assert_allclose(evaluated[:, 1], 4. + .06 * widths, atol=.001)
    np.testing.assert_allclose(evaluated[:, 2], _height(widths), atol=.001)
    np.testing.assert_array_equal(points, original)


@pytest.mark.parametrize('mirror', [-1, 1])
@pytest.mark.parametrize('coordinate, offset', [(1, .026), (2, .03)])
def test_continuous_double_returns_are_not_diluted_by_a_long_single_curved_rod(
        mirror, coordinate, offset):
    points, rib = _curve_cloud(mirror)
    near = (mirror * points[:, 0] >= -.40) & (mirror * points[:, 0] < -.15)
    delta = np.zeros(3)
    delta[coordinate] = offset
    points = np.vstack((points[~near], points[near] - delta, points[near] + delta))
    tree = cKDTree(points)
    observed = axis_support_points(points, tree, np.asarray(rib.axis_midpoint),
                                   np.asarray(rib.axis_direction), 0., 2.4, .07)
    widths = mirror * observed[:, 0]
    baseline = 4. + .06 * widths if coordinate == 1 else _height(widths)
    residual = observed[:, coordinate] - baseline
    # 负例必须真的留下连续三格双簇，不能因预筛选把两簇都删光而空洞通过。
    for lower in (-.35, -.30, -.25):
        selected = (widths >= lower) & (widths < lower + .05)
        assert np.count_nonzero(selected & (residual < -.02)) >= 3
        assert np.count_nonzero(selected & (residual > .02)) >= 3

    assert validated_rib_support(points, tree, rib, 0., 2.4) is None


@pytest.mark.parametrize('mirror', [-1, 1])
def test_curved_profile_reassociates_real_ends_without_widening_the_tube(mirror, monkeypatch):
    points, rib = _curved_end_cloud(mirror)
    original = points.copy()
    tree, initial_points, initial = _initial_curved_profile(points, rib)
    assert np.max(np.abs(initial_points[:, 0])) < .70
    assert _fit_support_profile(initial.cells).residual[1] > .035
    assert initial.residual[1] < .001

    associated = rib_support.profile_support_points(points, tree, initial, 0., 2.4, .07)

    assert associated[:, 0].min() < -.90
    assert associated[:, 0].max() > .90
    assert set(map(tuple, associated)).issubset(set(map(tuple, points)))
    assert np.max(np.linalg.norm(associated - initial.evaluate(associated[:, 0]), axis=1)) < .07
    calls = []
    associate = rib_support.profile_support_points

    def record_association(values, current_tree, model, floor_h, top_h, radius):
        calls.append(radius)
        return associate(values, current_tree, model, floor_h, top_h, radius)

    monkeypatch.setattr(rib_support, 'profile_support_points', record_association)
    model = validated_rib_support(points, tree, rib, 0., 2.4)

    assert model is not None
    assert 1 <= len(calls) <= 2
    assert all(radius == .07 for radius in calls)
    assert model.cells[0, 0] < -.90
    assert model.cells[-1, 0] > .90
    widths = np.array([-.9, 0., .9])
    evaluated = model.evaluate(mirror * widths)
    np.testing.assert_allclose(evaluated[:, 1], 4. + .06 * widths, atol=.001)
    np.testing.assert_allclose(evaluated[:, 2], 1.25 - .30 * widths + .25 * widths ** 2, atol=.001)
    np.testing.assert_array_equal(points, original)


def test_profile_reassociation_stops_after_two_expanding_support_rounds(monkeypatch):
    points, rib = _curved_end_cloud(1)
    associate = rib_support.profile_support_points
    calls = []

    def progressively_visible_support(values, tree, model, floor_h, top_h, radius):
        calls.append(radius)
        associated = associate(values, tree, model, floor_h, top_h, radius)
        # 每轮都还有真实点可扩张，仍不允许无界迭代追到全部端部。
        limit = .65 + .10 * len(calls)
        return associated[np.abs(associated[:, 0]) < limit]

    monkeypatch.setattr(rib_support, 'profile_support_points', progressively_visible_support)
    model = validated_rib_support(points, cKDTree(points), rib, 0., 2.4)

    assert model is not None
    assert calls == [.07, .07]
    assert .80 < np.max(np.abs(model.cells[:, 0])) < .85


@pytest.mark.parametrize('mirror', [-1, 1])
def test_profile_reassociation_does_not_admit_a_parallel_rod_outside_the_same_radius(mirror):
    actual, rib = _curved_end_cloud(mirror)
    _, _, initial = _initial_curved_profile(actual, rib)
    decoy = actual + [0., .13, 0.]
    points = np.vstack((actual, decoy))

    associated = rib_support.profile_support_points(points, cKDTree(points), initial, 0., 2.4, .07)

    assert associated[:, 0].min() < -.90
    assert associated[:, 0].max() > .90
    assert set(map(tuple, associated)).issubset(set(map(tuple, actual)))


@pytest.mark.parametrize('mirror', [-1, 1])
@pytest.mark.parametrize('coordinate, offset', [(1, .026), (2, .03)])
def test_curved_end_reassociation_rechecks_continuous_double_returns(mirror, coordinate, offset):
    actual, rib = _curved_end_cloud(mirror)
    _, initial_points, initial = _initial_curved_profile(actual, rib)
    assert np.max(np.abs(initial_points[:, 0])) < .70
    end = (mirror * actual[:, 0] >= -.90) & (mirror * actual[:, 0] < -.70)
    delta = np.zeros(3)
    delta[coordinate] = offset
    points = np.vstack((actual[~end], actual[end] - delta, actual[end] + delta))
    tree = cKDTree(points)
    associated = rib_support.profile_support_points(points, tree, initial, 0., 2.4, .07)
    widths = mirror * associated[:, 0]
    baseline = (4. + .06 * widths if coordinate == 1
                else 1.25 - .30 * widths + .25 * widths ** 2)
    residual = associated[:, coordinate] - baseline
    for lower in (-.85, -.80, -.75):
        selected = (widths >= lower) & (widths < lower + .05)
        assert np.count_nonzero(selected & (residual < -.02)) >= 3
        assert np.count_nonzero(selected & (residual > .02)) >= 3

    assert validated_rib_support(points, tree, rib, 0., 2.4) is None


@pytest.mark.parametrize('mirror', [-1, 1])
def test_straight_rod_does_not_enter_curved_profile_reassociation(mirror, monkeypatch):
    widths = np.arange(-.95, .951, .005)
    points = _tube_points(widths, 1.28 - .30 * widths, mirror)
    _, rib = _curved_end_cloud(mirror)

    def unexpected_association(*args, **kwargs):
        raise AssertionError('Straight rods must keep their existing support path')

    monkeypatch.setattr(rib_support, 'profile_support_points', unexpected_association)
    model = validated_rib_support(points, cKDTree(points), rib, 0., 2.4)

    assert model is not None
    assert len(model.height_coefficients) == 2


@pytest.mark.parametrize('mirror', [-1, 1])
@pytest.mark.parametrize('count', [1, 2])
def test_completion_fills_sparse_near_wall_cells_before_final_localization(mirror, count):
    points, rib, measurement = _sparse_near_wall_cloud(mirror, count)
    original = points.copy()
    assert validated_rib_support(points, cKDTree(points), rib, 0., 2.4) is not None
    before = refine_rib_side_positions(points, [rib], measurement, 0., 2.4, final_cloud=True)[0]
    assert before['valid'] is False
    assert before['positionM'] is None
    near = points[mirror * points[:, 0] < -.35]
    _, counts = np.unique(np.floor(near[:, 0] / .05).astype(int), return_counts=True)
    np.testing.assert_array_equal(counts, count)

    added, info = complete_rib_gaps(points, [rib], (-1.12, 1.12), 0., 9., 0., 2.4)

    near_added = added[(mirror * added[:, 0] > -1.05) & (mirror * added[:, 0] < -.35)]
    assert len(near_added) > 0
    assert info['completed'] is True
    assert info['pointCount'] == len(added)
    # 旧“任意点距离7cm内就不补”会把这些每格仅1~2点的位置全部跳过。
    nearest, _ = cKDTree(points).query(near_added, k=1)
    assert np.all(nearest < .07)
    final = np.vstack((points, added))
    row = refine_rib_side_positions(final, [rib], measurement, 0., 2.4, final_cloud=True)[0]
    assert row['valid'] is True
    assert row['positionM'] == pytest.approx(4. - .06 * 1.12, abs=.003)
    np.testing.assert_array_equal(points, original)
    np.testing.assert_array_equal(final[:len(points)], original)


@pytest.mark.parametrize('mirror', [-1, 1])
@pytest.mark.parametrize('conflict', ['double_axis', 'offset_bracket'])
def test_sparse_cell_completion_does_not_override_multiple_or_offset_axes(mirror, conflict):
    points, rib, measurement = _sparse_near_wall_cloud(mirror, 2)
    if conflict == 'double_axis':
        points = np.vstack((points - [0., .026, 0.], points + [0., .026, 0.]))
    else:
        widths = np.arange(-1.045, -.805, .005)
        bracket = _tube_points(widths, 1.28 - .30 * widths, mirror) + [0., .045, 0.]
        points = np.vstack((points, bracket))
    original = points.copy()

    added, info = complete_rib_gaps(points, [rib], (-1.12, 1.12), 0., 9., 0., 2.4)

    assert not np.any(mirror * added[:, 0] < -.55)
    if conflict == 'double_axis':
        assert len(added) == 0
        assert info['ribs'][0]['supportValidation']['reason'] == 'multiple_whole_rib_axes'
    row = refine_rib_side_positions(np.vstack((points, added)), [rib], measurement,
                                    0., 2.4, final_cloud=True)[0]
    assert row['valid'] is False, row
    assert row['positionM'] is None
    np.testing.assert_array_equal(points, original)


@pytest.mark.parametrize('mirror', [-1, 1])
@pytest.mark.parametrize('coordinate, offset', [(1, .026), (2, .03)])
def test_multiple_axis_diagnostics_count_actual_split_width_cells(mirror, coordinate, offset):
    widths = np.arange(-.975, .976, .05)
    actual = _tube_points(widths, 1.28 - .30 * widths, mirror)
    _, rib = _curved_end_cloud(mirror)
    delta = np.zeros(3)
    delta[coordinate] = offset
    points = np.vstack((actual - delta, actual + delta))
    tree = cKDTree(points)
    observed = axis_support_points(points, tree, np.asarray(rib.axis_midpoint),
                                   np.asarray(rib.axis_direction), 0., 2.4, .07)
    observed_w = mirror * observed[:, 0]
    baseline = 4. + .06 * observed_w if coordinate == 1 else 1.28 - .30 * observed_w
    residual = observed[:, coordinate] - baseline
    keys = np.floor(observed[:, 0] / .05).astype(int)
    expected_keys = [int(key) for key in np.unique(keys)
                     if np.count_nonzero((keys == key) & (residual < -.02)) >= 3
                     and np.count_nonzero((keys == key) & (residual > .02)) >= 3]
    assert len(expected_keys) >= 3
    diagnostics = {}

    model = validated_rib_support(points, tree, rib, 0., 2.4, diagnostics=diagnostics)

    assert model is None
    assert diagnostics['reason'] == 'multiple_whole_rib_axes'
    assert diagnostics['splitCellCount'] == len(expected_keys)
    assert diagnostics['splitWidthCellKeys'] == expected_keys
    added, info = complete_rib_gaps(points, [rib], (-1.12, 1.12), 0., 9., 0., 2.4)
    assert len(added) == 0
    assert info['ribs'][0]['supportValidation'] == diagnostics


@pytest.mark.parametrize('mirror', [-1, 1])
@pytest.mark.parametrize('parallel_factor', [.99, 1.])
def test_profile_side_intersection_rejects_nearly_parallel_geometry(mirror, parallel_factor):
    model = _fit_support_profile(_curve_cells(mirror), height_degree=2)
    a = mirror * parallel_factor / .06

    assert model.intersect_side(np.array([a, 0., -4. * a])) is None
