"""同侧拉筋精定位的独立合成几何与对抗契约，不使用现场人工尺寸。"""

from copy import deepcopy
from dataclasses import replace
import json

import numpy as np
import pytest

from carpointcloud_analyzer.domain.longitudinal_measurement import (
    measurement_metadata,
    rib_projections,
)
from carpointcloud_analyzer.domain.models import LongitudinalMeasurement, RibInfo
from carpointcloud_analyzer.domain.rib_completion import complete_rib_gaps
from carpointcloud_analyzer.domain.rib_side_measurement import refine_rib_side_positions


def measurement():
    return LongitudinalMeasurement(
        0., 9., (0., 0., -1.12),
        {'selectedSide': 'left', 'completionUsed': False,
         'upperReference': {'endpoints': {'front': {'valid': False},
                                         'rear': {'valid': False}}}},
    )


def rib(rib_id=1, longitudinal=4., slope_l=.025, slope_h=.012):
    return RibInfo(
        rib_id, longitudinal, 0., 1.55, (-.55, .55), (1.4, 1.7), 90, 1.1, .03,
        model_type='axis', axis_midpoint=(0., longitudinal, 1.55),
        axis_direction=(1., slope_l, slope_h),
        # 发现阶段有意排壁，不能把这段截短范围作为原始云端部可见性。
        axis_observed_w_range=(-.55, .55),
    )


def rod_points(item, start=-1.045, stop=-.285, step=.005, radius=.008, widths=None):
    widths = np.arange(start, stop, step) if widths is None else np.asarray(widths)
    theta = np.arange(8) * np.pi / 4
    w, angle = np.meshgrid(widths, theta, indexing='ij')
    point = np.asarray(item.axis_midpoint)
    direction = np.asarray(item.axis_direction)
    along = (w.ravel() - point[0]) / direction[0]
    return np.c_[w.ravel(),
                 point[1] + along * direction[1] + radius * np.cos(angle.ravel()),
                 point[2] + along * direction[2] + radius * np.sin(angle.ravel())]


def plate(w_min=-1.04, w_max=-.28, l_min=3.65, l_max=4.35, height=1.55):
    w, longitudinal = np.meshgrid(np.arange(w_min, w_max, .012),
                                 np.arange(l_min, l_max, .012))
    return np.c_[w.ravel(), longitudinal.ravel(), np.full(w.size, height)]


def fake_inner_wall(width=-.85, longitudinal=4.):
    longitudinal, height = np.meshgrid(np.arange(longitudinal - .35, longitudinal + .35, .01),
                                      np.arange(1.2, 1.9, .01))
    return np.c_[np.full(longitudinal.size, width), longitudinal.ravel(), height.ravel()]


def thin_band(points, start, stop, keep=2):
    """把 [start, stop) 宽度带的点抽稀到 keep 个。

    现场回波变稀（而非整段丢失）时，管内点仍在、也仍与杆轴共线，但不足 3 个，
    旧实现就产不出格心。保留带内首末两点，避免格心中位被人为拉到带边。
    """
    inside = np.flatnonzero((points[:, 0] >= start) & (points[:, 0] < stop))
    if len(inside) <= keep:
        return points
    kept = inside[np.linspace(0, len(inside) - 1, keep).astype(int)]
    return np.delete(points, np.setdiff1d(inside, kept), axis=0)


def thin_cells(points, keys, keep=2):
    """按 5cm 格逐格抽稀到 keep 个（与实现同一起点 x=-1.12）。"""
    for key in keys:
        points = thin_band(points, -1.12 + .05 * key, -1.12 + .05 * (key + 1), keep=keep)
    return points


def refine(points, ribs, current=None, *, final_cloud=False):
    return refine_rib_side_positions(points, ribs, current or measurement(), 0., 2.,
                                     final_cloud=final_cloud)


def test_export_checks_rib_plane_separately_from_length_side_and_preserves_closure():
    item = rib()
    reference = measurement()
    rows = refine(rod_points(item), [item], reference)
    length = replace(reference, side_plane=(0., 0., 1.2), rib_side_projections=rows,
                     metadata={'selectedSide': 'right', 'ribSideReference': {
                         'selectedSide': 'left', 'sidePlane': list(reference.side_plane)}})
    exported = measurement_metadata(length, [item])
    assert exported['selectedSide'] == 'right'
    assert exported['ribProjections'][0]['selectedSide'] == 'left'
    assert exported['ribPositionsValid']
    assert sum(exported['ribIntervalsM']) == pytest.approx(length.length)
    rows[0] = {**rows[0], 'selectedSide': 'right'}
    assert rib_projections([item], length)[0]['reason'] == 'rib_side_reference_mismatch'


def test_refined_rib_and_axis_fallback_share_horizontal_length_coordinate():
    c, s = np.cos(np.arctan(.06)), np.sin(np.arctan(.06))
    rotation = np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])
    original = rib()
    midpoint = np.asarray(original.axis_midpoint) @ rotation
    rotated = replace(original, y_position=float(midpoint[1]),
                      axis_midpoint=tuple(midpoint),
                      axis_direction=tuple(np.asarray(original.axis_direction) @ rotation))
    current = replace(measurement(), longitudinal_axis=(0., c, -s))
    rows = refine_rib_side_positions(rod_points(original) @ rotation, [rotated], current, -.6, 2.6)
    assert rows[0]['valid'], rows
    expected = original.axis_midpoint[1] - 1.12*original.axis_direction[1]
    assert rows[0]['positionM'] == pytest.approx(expected, abs=.01)
    current.rib_side_projections = rows
    assert rib_projections([rotated], current)[0]['valid']
    # 旧轴路径也必须投影到同一坐标，不能把局部 L 混回已校正的前后端点。
    current.rib_side_projections = None
    full_axis = replace(rotated, axis_observed_w_range=(-1.1, 1.1))
    assert rib_projections([full_axis], current)[0]['positionM'] == pytest.approx(expected, abs=.01)


def assert_missing(rows):
    assert rows
    assert all(row['valid'] is False and row['positionM'] is None for row in rows)
    assert all(row['reason'] and row['reason'] != 'ok' for row in rows)
    json.dumps(rows, allow_nan=False)


def projection_row(rib_id, position):
    return {'ribId': rib_id, 'valid': True, 'positionM': position, 'extrapolationM': .1,
            'selectedSide': 'left', 'referencePlaneWLH': [0., 0., -1.12],
            'intersectionWLH': [-1.12, position, 1.55]}


def test_current_observed_end_recovers_truncated_discovery_axis_without_mutation():
    item = rib()
    current = measurement()
    points = rod_points(item)
    original_points = points.copy()
    original_rib, original_measurement = deepcopy(item), deepcopy(current)
    assert not rib_projections([item], current)[0]['valid']

    rows = refine(points, [item], current)
    assert rows[0]['valid']
    assert rows[0]['positionM'] == pytest.approx(4. - 1.12 * .025, abs=.003)
    assert rows[0]['extrapolationM'] <= .25
    assert rows[0]['observedWRangeM'][0] < -.95
    assert rows[0]['supportCellCount'] >= 7
    assert 'source' not in rows[0]
    json.dumps(rows, allow_nan=False)
    np.testing.assert_array_equal(points, original_points)
    assert item == original_rib
    assert current == original_measurement


@pytest.mark.parametrize('decoy', ['flat_board', 'inner_wall', 'both'])
@pytest.mark.parametrize('final_cloud', [False, True])
def test_wall_or_fixing_plate_without_a_rod_cannot_become_endpoint_support(decoy, final_cloud):
    points = (plate() if decoy == 'flat_board' else fake_inner_wall()
              if decoy == 'inner_wall' else np.vstack([plate(), fake_inner_wall()]))
    assert_missing(refine(points, [rib()], final_cloud=final_cloud))


@pytest.mark.parametrize('complete', [False, True])
def test_final_cloud_cannot_revive_conflicting_offset_rod_segments(complete):
    item = rib(slope_l=0., slope_h=0.)
    parts = []
    for start, stop, offset in [(-1.045, -.805, .04), (-.795, -.555, -.04),
                                (-.545, -.285, .04)]:
        part = rod_points(item, start=start, stop=stop, radius=.004)
        part[:, 1] += offset
        parts.append(part)
    points = np.vstack(parts)
    if complete:
        added, _ = complete_rib_gaps(points, [item], (-1.12, 1.12), 0., 9., 0., 2.)
        points = np.vstack((points, added))
    current = measurement()
    rows = refine(points, [item], current, final_cloud=True)
    assert_missing(rows)
    assert rows[0]['reason'] == 'ambiguous_local_rib_axis'
    current.rib_side_projections = rows
    assert_missing(rib_projections([item], current))
    assert measurement_metadata(current, [item])['ribIntervalsM'] == [None, None]


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('slope_l', [-.18, 0., .20])
def test_completed_collinear_rod_keeps_valid_final_position(side, slope_l):
    item = rib(slope_l=slope_l, slope_h=0.)
    points = np.vstack((rod_points(item, start=-1.045, stop=-.805),
                        rod_points(item, start=-.55, stop=.55),
                        rod_points(item, start=.805, stop=1.045)))
    original = points.copy()
    added, _ = complete_rib_gaps(points, [item], (-1.12, 1.12), 0., 9., 0., 2.)
    current = measurement()
    current.metadata['selectedSide'] = side
    current.side_plane = (0., 0., -1.12 if side == 'left' else 1.12)
    row = refine(np.vstack((points, added)), [item], current, final_cloud=True)[0]
    assert row['valid'] is True
    assert row['positionM'] == pytest.approx(4. + current.side_plane[2] * slope_l, abs=.003)
    np.testing.assert_array_equal(points, original)


@pytest.mark.parametrize('side, offset_l, complete, slope_l', [
    ('left', -.04, False, -.18), ('right', .04, True, .20),
    ('left', .04, True, 0.), ('right', -.04, False, .20),
    ('left', -.04, True, .20), ('right', .04, False, 0.),
    ('left', .04, False, -.18), ('right', -.04, True, -.18),
])
def test_final_cloud_cannot_join_an_isolated_near_wall_bracket_to_the_seed(
        side, offset_l, complete, slope_l):
    item = rib(slope_l=slope_l, slope_h=0.)
    seed = rod_points(item, start=-.55, stop=-.285, radius=.004)
    bracket = rod_points(item, start=-1.045, stop=-.805, radius=.004)
    bracket[:, 1] += offset_l
    points = np.vstack((seed, bracket))
    current = measurement()
    if side == 'right':
        points = points * [-1., 1., 1.]
        item.axis_direction = (-1., slope_l, 0.)
        current.side_plane = (0., 0., 1.12)
        current.metadata['selectedSide'] = side
    original = points.copy()
    if complete:
        added, info = complete_rib_gaps(points, [item], (-1.12, 1.12), 0., 9., 0., 2.)
        assert info['ribs'][0]['conflictingSides'] == [side]
        assert not np.any(added[:, 0] < -.55 if side == 'left' else added[:, 0] > .55)
        points = np.vstack((points, added))
    row = refine(points, [item], current, final_cloud=True)[0]
    assert_missing([row])
    assert row['reason'] == 'disconnected_local_rib_support'
    assert row == refine(points, [item], current, final_cloud=False)[0]
    current.rib_side_projections = [row]
    assert measurement_metadata(current, [item])['ribIntervalsM'] == [None, None]
    np.testing.assert_array_equal(points[:len(original)], original)


def test_rod_position_is_not_attracted_to_dense_offset_fixing_plate():
    item = rib()
    actual = rod_points(item)
    # 邻近且不共线的固定板比真杆点数多；正确答案只能来自杆轴。
    decoy = plate(l_min=4.12, l_max=4.7)
    baseline = refine(actual, [item])[0]
    assert baseline['valid']
    once = refine(np.vstack([actual, decoy]), [item])[0]
    repeated = refine(np.vstack([actual, np.repeat(decoy, 5, axis=0)]), [item])[0]
    assert once['valid'] and repeated['valid']
    assert once['positionM'] == pytest.approx(baseline['positionM'], abs=.003)
    assert repeated['positionM'] == pytest.approx(once['positionM'], abs=1e-12)


@pytest.mark.parametrize('side, offset_l, slope_l', [
    ('left', -.14, -.18), ('right', -.14, .20),
    ('left', -.12, 0.), ('right', -.12, -.18),
    ('left', .12, .20), ('right', .12, 0.),
    ('left', .14, -.18), ('right', .14, .20),
])
def test_completion_cannot_create_parallel_support_beside_an_offset_bracket(side, offset_l, slope_l):
    item = rib(slope_l=slope_l, slope_h=0.)
    seed = rod_points(item, start=-.55, stop=.55, radius=.004)
    bracket = rod_points(item, start=-1.045, stop=-.805, radius=.004)
    bracket[:, 1] += offset_l
    points = np.vstack((seed, bracket))
    current = measurement()
    if side == 'right':
        points[:, 0] *= -1
        item.axis_direction = (-1., slope_l, 0.)
        current.side_plane = (0., 0., 1.12)
        current.metadata['selectedSide'] = side
    added, _ = complete_rib_gaps(points, [item], (-1.12, 1.12), 0., 9., 0., 2.)
    assert not np.any(added[:, 0] < -.55 if side == 'left' else added[:, 0] > .55)
    assert_missing(refine(np.vstack((points, added)), [item], current, final_cloud=True))


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('island', [0, 1, 3])
def test_completion_does_not_turn_central_only_support_into_a_near_wall_observation(side, island):
    item = rib(slope_l=0., slope_h=0.)
    points = rod_points(item, start=-.55, stop=.55)
    for index in range(island):
        widths = np.linspace(-1.035 + index * .1, -1.005 + index * .1, 7)
        points = np.vstack((points, np.c_[widths, np.full(7, 4.), np.full(7, 1.55)]))
    current = measurement()
    if side == 'right':
        points[:, 0] *= -1
        item.axis_direction = (-1., 0., 0.)
        current.side_plane = (0., 0., 1.12)
        current.metadata['selectedSide'] = side
    added, _ = complete_rib_gaps(points, [item], (-1.12, 1.12), 0., 9., 0., 2.)
    assert not np.any(np.abs(added[:, 0]) > .55)
    assert_missing(refine(np.vstack((points, added)), [item], current, final_cloud=True))


def test_opposite_side_visibility_does_not_fill_selected_side_missing_endpoint():
    item = rib()
    points = rod_points(item, start=.285, stop=1.045)
    assert_missing(refine(points, [item]))
    right = replace(measurement(), side_plane=(0., 0., 1.12), metadata={'selectedSide': 'right'})
    assert refine(points, [item], right)[0]['valid']


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('slope_l', [-.18, .20])
def test_whole_rod_recovers_short_side_window_using_same_final_cloud(side, slope_l):
    item = rib(slope_l=slope_l, slope_h=.06)
    points = np.vstack((rod_points(item, start=-1.045, stop=-.85),
                        rod_points(item, start=-.1, stop=.8)))
    current = measurement()
    if side == 'right':
        points[:, 0] *= -1
        item.axis_direction = (-1., slope_l, .06)
        current.side_plane = (0., 0., 1.12)
        current.metadata['selectedSide'] = side
    row = refine(points, [item], current, final_cloud=True)[0]
    assert row['valid']
    assert row['method'] == 'final_cloud_whole_rib_axis_side_intersection'
    assert row['positionM'] == pytest.approx(4. - 1.12 * slope_l, abs=.003)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_long_stable_rod_completes_missing_end_before_final_localization(side):
    item = rib(slope_l=.06, slope_h=.12)
    points = rod_points(item, start=-.35, stop=1.045)
    current = measurement()
    if side == 'right':
        points[:, 0] *= -1
        item.axis_direction = (-1., .06, .12)
        current.side_plane = (0., 0., 1.12)
        current.metadata['selectedSide'] = side
    before = points.copy()
    assert_missing(refine(points, [item], current))
    added, _ = complete_rib_gaps(points, [item], (-1.12, 1.12), 0., 9., 0., 2.)
    assert len(added) > 0
    row = refine(np.vstack((points, added)), [item], current, final_cloud=True)[0]
    assert row['valid']
    assert row['positionM'] == pytest.approx(4. - 1.12 * .06, abs=.003)
    np.testing.assert_array_equal(points, before)


@pytest.mark.parametrize('offset', [-.045, .045])
def test_whole_rod_cannot_hide_offset_near_wall_segment(offset):
    item = rib(slope_l=0., slope_h=0.)
    near = rod_points(item, start=-1.045, stop=-.85, radius=.004)
    near[:, 1] += offset
    points = np.vstack((near, rod_points(item, start=-.1, stop=.8, radius=.004)))
    added, _ = complete_rib_gaps(points, [item], (-1.12, 1.12), 0., 9., 0., 2.)
    assert_missing(refine(np.vstack((points, added)), [item], final_cloud=True))


def test_completion_refits_current_rod_instead_of_copying_discovery_axis():
    actual = rib(slope_l=.08, slope_h=.12)
    seed = rib(slope_l=.06, slope_h=.12)
    points = rod_points(actual, start=-.35, stop=1.045)
    added, _ = complete_rib_gaps(points, [seed], (-1.12, 1.12), 0., 9., 0., 2.)
    assert len(added) > 0
    assert np.max(np.abs(added[:, 1] - (4. + added[:, 0] * .08))) <= .0081
    row = refine(np.vstack((points, added)), [seed], final_cloud=True)[0]
    assert row['valid']
    assert row['positionM'] == pytest.approx(4. - 1.12 * .08, abs=.003)


@pytest.mark.parametrize('complete', [False, True])
def test_long_single_axis_cannot_dilute_near_side_double_returns(complete):
    item = rib(slope_l=0., slope_h=0.)
    near = rod_points(item, start=-1.045, stop=-.645)
    points = np.vstack((near + [0., .026, 0.], near - [0., .026, 0.],
                        rod_points(item, start=-.645, stop=1.045)))
    if complete:
        added, _ = complete_rib_gaps(points, [item], (-1.12, 1.12), 0., 9., 0., 2.)
        assert len(added) == 0
        points = np.vstack((points, added))
    row = refine(points, [item], final_cloud=True)[0]
    assert_missing([row])
    assert row['reason'] == 'multiple_local_rib_axes'


def test_near_wall_isolated_patch_cannot_extend_far_rod_support_range():
    item = rib()
    far = rod_points(item, start=-.78, stop=-.285)
    island = rod_points(item, start=-1.02, stop=-1.005)
    rows = refine(np.vstack([far, island]), [item])
    assert_missing(rows)


def test_two_thinned_interior_cells_are_bridged_without_moving_observed_range():
    item = rib()
    clean = rod_points(item)
    baseline = refine(clean, [item])[0]
    assert baseline['valid']
    assert baseline['supportCellCount'] == 16

    # 只把管内回波抽稀：点仍在、仍与杆轴共线，但每格不足 3 个。
    thinned = thin_cells(clean, (7, 8), keep=2)
    row = refine(thinned, [item])[0]
    assert row['valid']
    assert row['reason'] == 'ok'
    assert row['bridgedInteriorCellCount'] == 2
    assert isinstance(row['bridgedInteriorCellCount'], int)
    assert row['supportCellCount'] == 14
    assert row['positionM'] == pytest.approx(4. - 1.12 * .025, abs=.003)
    assert row['positionM'] == pytest.approx(baseline['positionM'], abs=.005)
    # 被桥接的格严格在内部，实测范围端点只能由直接支持格决定。
    np.testing.assert_allclose(row['observedWRangeM'], baseline['observedWRangeM'], atol=1e-9)
    json.dumps(row, allow_nan=False)


def test_three_thinned_interior_cells_stay_disconnected():
    item = rib()
    thinned = thin_cells(rod_points(item), (7, 8, 9), keep=2)
    rows = refine(thinned, [item])
    assert_missing(rows)
    assert rows[0]['reason'] == 'disconnected_local_rib_support'
    assert 'bridgedInteriorCellCount' not in rows[0]


def test_thinned_interior_cell_needs_more_than_one_point():
    item = rib()
    thinned = thin_cells(rod_points(item), (7, 8), keep=1)
    rows = refine(thinned, [item])
    assert_missing(rows)
    assert rows[0]['reason'] == 'disconnected_local_rib_support'
    assert 'bridgedInteriorCellCount' not in rows[0]


def test_thinned_lead_in_cells_do_not_extend_observed_range():
    item = rib()
    thinned = thin_cells(rod_points(item), (1, 2), keep=2)
    row = refine(thinned, [item])[0]
    # 端部格没有两侧直接支持，即使格内仍有 2 个实测点也不得补，范围不得被推回近壁端。
    assert 'bridgedInteriorCellCount' not in row
    assert row['observedWRangeM'][0] > -.98
    assert row['valid']
    assert row['extrapolationM'] == pytest.approx(1.12 + row['observedWRangeM'][0], abs=1e-12)


@pytest.mark.parametrize('distance, accepted', [(.29, True), (.295, True), (.305, False)])
def test_thirty_centimetre_limit_uses_supported_cells_not_seed_axis(distance, accepted):
    item = rib(slope_l=0., slope_h=0.)
    widths = -1.12 + distance + np.arange(11) * .052
    row = refine(rod_points(item, widths=widths), [item])[0]
    assert row['valid'] is accepted
    assert row['extrapolationM'] == pytest.approx(distance, abs=1e-12)
    if not accepted:
        assert row['reason'] == 'rib_side_extrapolation_exceeds_limit'


@pytest.mark.parametrize('slope_l, slope_h', [(.20, .08), (-.18, -.06)])
def test_true_slanted_rod_uses_three_dimensional_side_intersection(slope_l, slope_h):
    item = rib(slope_l=slope_l, slope_h=slope_h)
    row = refine(rod_points(item), [item])[0]
    assert row['valid']
    np.testing.assert_allclose(row['intersectionWLH'], [-1.12, 4. - 1.12 * slope_l,
                                                     1.55 - 1.12 * slope_h], atol=.003)


def test_tilted_side_plane_uses_both_longitudinal_and_height_components():
    item = rib(slope_l=.15, slope_h=.08)
    plane = (.025, .010, -1.245)
    current = replace(measurement(), side_plane=plane)
    a, b, c = plane
    width = (a * 4. + b * 1.55 + c) / (1. - a * .15 - b * .08)
    expected = [width, 4. + width * .15, 1.55 + width * .08]
    row = refine(rod_points(item), [item], current)[0]
    assert row['valid']
    np.testing.assert_allclose(row['intersectionWLH'], expected, atol=.003)


def test_refinement_is_deterministic_under_permutation_and_duplicate_density():
    item = rib()
    points = rod_points(item)
    baseline = refine(points, [item])
    changed = np.vstack([points, np.repeat(points[::5], 15, axis=0)])
    np.random.default_rng(814).shuffle(changed)
    assert refine(changed, [item]) == baseline


def test_mirror_and_coordinate_translation_preserve_local_geometry():
    item = rib()
    points = rod_points(item)
    baseline = refine(points, [item])[0]
    assert baseline['valid']
    mirrored_item = replace(item, axis_direction=(-1., .025, .012),
                            axis_observed_w_range=tuple(-value for value
                                                        in reversed(item.axis_observed_w_range)))
    mirrored_measurement = replace(measurement(), side_plane=(0., 0., 1.12),
                                  metadata={'selectedSide': 'right'})
    mirrored = refine(points * [-1., 1., 1.], [mirrored_item], mirrored_measurement)[0]
    assert mirrored['valid']
    assert mirrored['positionM'] == pytest.approx(baseline['positionM'], abs=1e-10)

    shift = np.array([2.3, 12.7, 3.4])
    translated_item = replace(item, y_position=item.y_position + shift[1],
                              axis_midpoint=tuple(np.asarray(item.axis_midpoint) + shift),
                              axis_observed_w_range=tuple(value + shift[0]
                                                          for value in item.axis_observed_w_range))
    translated_measurement = replace(measurement(), rear_l=12.7, front_l=21.7,
                                     side_plane=(0., 0., -1.12 + shift[0]))
    translated = refine_rib_side_positions(points + shift, [translated_item], translated_measurement,
                                           shift[2], 2. + shift[2])[0]
    assert translated['valid']
    assert translated['positionM'] == pytest.approx(baseline['positionM'] + shift[1], abs=.003)


@pytest.mark.parametrize('invalid', [np.empty((0, 3)), np.zeros((30, 2)), np.zeros(30),
                                   np.full((30, 3), np.nan), np.full((30, 3), np.inf)])
def test_invalid_point_arrays_are_explicit_missing_and_strict_json_safe(invalid):
    assert_missing(refine(invalid, [rib()]))


@pytest.mark.parametrize('bad_value', [np.nan, np.inf, -np.inf])
def test_nonfinite_point_does_not_escape_into_evidence(bad_value):
    points = np.vstack([rod_points(rib()), [bad_value, 4., 1.55]])
    assert_missing(refine(points, [rib()]))


def test_single_point_repeated_cannot_satisfy_support_count():
    points = np.repeat([[-.8, 3.98, 1.54]], 5000, axis=0)
    assert_missing(refine(points, [rib()]))


@pytest.mark.parametrize('change', [dict(side_plane=None), dict(side_plane=(np.nan, 0., 0.)),
                                  dict(metadata={}), dict(rear_l=None)])
def test_invalid_measurement_does_not_fall_back_to_another_side(change):
    assert_missing(refine(rod_points(rib()), [rib()], replace(measurement(), **change)))


def test_stable_rib_ids_survive_export_sorting_and_metadata_does_not_mutate():
    back = rib(rib_id=71, longitudinal=2.7, slope_l=-.035)
    front = rib(rib_id=12, longitudinal=6.8, slope_l=.025)
    items = [back, front]
    points = np.vstack([rod_points(back), rod_points(front)])
    current = measurement()
    current.rib_side_projections = refine(points, items, current)
    assert all(row['valid'] for row in current.rib_side_projections)
    before = deepcopy(current)
    rows = rib_projections([front, back], current)
    assert [row['ribId'] for row in rows] == [12, 71]
    expected = [6.8 - 1.12 * .025, 2.7 + 1.12 * .035]
    assert [row['positionM'] for row in rows] == pytest.approx(expected, abs=.003)
    metadata = measurement_metadata(current, items)
    assert sum(metadata['ribIntervalsM']) == pytest.approx(9.)
    assert metadata['inferredGeometry']
    assert current == before
    json.dumps(metadata, allow_nan=False)


def test_one_missing_rib_keeps_reliable_outer_interval_and_never_uses_old_axis():
    front = rib(rib_id=1, longitudinal=6.8)
    back = rib(rib_id=2, longitudinal=2.7)
    # 旧轴故意“可用”；精定位已尝试缺测之后不得借它填回。
    back.axis_observed_w_range = (-1.2, 1.2)
    current = measurement()
    current.rib_side_projections = refine(rod_points(front), [front, back], current)
    projections = rib_projections([front, back], current)
    assert projections[0]['valid'] and not projections[1]['valid']
    assert projections[1]['positionM'] is None
    metadata = measurement_metadata(current, [front, back])
    assert metadata['ribIntervalsM'][0] == pytest.approx(9. - (6.8 - 1.12 * .025), abs=.003)
    assert metadata['ribIntervalsM'][1:] == [None, None]
    assert metadata['ribPositionsValid'] is False
    assert metadata['inferredGeometry'] is True


def test_two_detected_ribs_cannot_claim_one_observed_rod_and_emit_zero_interval():
    observed = rib(longitudinal=4., slope_l=0., slope_h=0.)
    candidates = [rib(rib_id=1, longitudinal=4.025, slope_l=0., slope_h=0.),
                  rib(rib_id=2, longitudinal=3.975, slope_l=0., slope_h=0.)]
    current = measurement()
    current.rib_side_projections = refine(rod_points(observed), candidates, current)
    assert all(row['valid'] for row in current.rib_side_projections)
    final = rib_projections(candidates, current)
    assert_missing(final)
    assert measurement_metadata(current, candidates)['ribIntervalsM'] == [None, None, None]


def test_crossed_refined_endpoints_do_not_reorder_rib_identity_to_make_positive_gaps():
    items = [rib(rib_id=9, longitudinal=5.), rib(rib_id=4, longitudinal=4.)]
    current = measurement()
    current.rib_side_projections = [
        projection_row(9, 3.9), projection_row(4, 4.1),
    ]
    rows = rib_projections(items, current)
    assert_missing(rows)
    assert all(row['reason'] == 'rib_axes_cross_on_selected_side' for row in rows)


def test_duplicate_refinement_identity_is_missing_not_first_match_wins():
    item = rib()
    current = measurement()
    rows = refine(rod_points(item), [item], current)
    assert rows[0]['valid']
    current.rib_side_projections = [rows[0], deepcopy(rows[0])]
    projected = rib_projections([item], current)
    assert_missing(projected)
    assert projected[0]['reason'] == 'rib_side_identity_unavailable'


@pytest.mark.parametrize('coordinate', [1, 2])
def test_two_parallel_returns_in_one_tube_do_not_invent_a_midline(coordinate):
    item = rib(slope_l=0., slope_h=0.)
    first = rod_points(item, radius=.006)
    second = first.copy()
    first[:, coordinate] -= .026
    second[:, coordinate] += .026
    # 两条实测线相隔5.2cm，旧种子轴在空隙中；不能把两条线的中位数当成真杆。
    assert_missing(refine(np.vstack([first, second]), [item]))


def test_missing_refinement_evidence_is_recursively_json_safe_without_mutating_source():
    current = measurement()
    row = {'ribId': 1, 'valid': False, 'positionM': np.inf,
           'extrapolationM': np.nan, 'reason': 'ambiguous_local_rib_axis',
           'supportCellsWLH': [[np.nan, 1., np.inf]],
           'nested': {'range': [-np.inf, 1.]}}
    current.rib_side_projections = [row]
    projected = rib_projections([rib()], current)
    assert_missing(projected)
    json.dumps(measurement_metadata(current, [rib()]), allow_nan=False)
    assert np.isposinf(row['positionM']) and np.isnan(row['extrapolationM'])
    assert np.isnan(row['supportCellsWLH'][0][0])


@pytest.mark.parametrize('final_cloud', [False, True])
@pytest.mark.parametrize('offset_l', [-.045, .045])
@pytest.mark.parametrize('offset_h', [0., .06])
def test_offset_bracket_and_seed_rod_cannot_be_stitched_into_a_false_diagonal(
        final_cloud, offset_l, offset_h):
    item = rib(slope_l=0., slope_h=0.)
    seed_support = rod_points(item, start=-.645, stop=-.285, radius=.004)
    bracket = rod_points(item, start=-1.045, stop=-.665, radius=.004)
    bracket[:, 1] += offset_l
    bracket[:, 2] += offset_h / 2
    seed_support[:, 2] -= offset_h / 2
    # 旧种子段和近壁支架各自平直，4.5cm跳变不能被整体回归抹成小斜率。
    rows = refine(np.vstack([seed_support, bracket]), [item], final_cloud=final_cloud)
    assert_missing(rows)
    assert rows[0]['reason'] == 'local_rib_longitudinal_discontinuity'


@pytest.mark.parametrize('final_cloud', [False, True])
@pytest.mark.parametrize('height_step', [-.09, .09])
@pytest.mark.parametrize('sparse', [False, True])
def test_height_jump_keeps_continuous_longitudinal_consensus(final_cloud, height_step, sparse):
    item = rib(slope_l=0., slope_h=0.)
    seed_support = rod_points(item, start=-.645, stop=-.285, radius=.004)
    near_support = rod_points(item, start=-1.045, stop=-.665, radius=.004)
    seed_support[:, 2] -= height_step / 2
    near_support[:, 2] += height_step / 2
    points = np.vstack([seed_support, near_support])
    if sparse:
        points = thin_cells(points, (3, 4))
    # 同一纵向位置只在高度上断续，仍可发布直接支持的 L，不外推物理交点。
    row = refine(points, [item], final_cloud=final_cloud)[0]
    assert row['valid'] is True
    assert row['reason'] == 'ok_supported_rod_center'
    assert row['method'] == 'local_rib_longitudinal_consensus'
    assert row['positionM'] == pytest.approx(4., abs=.003)
    assert row['extrapolationM'] is None
    assert row['discontinuityCount'] == 1
    if sparse:
        assert row['bridgedInteriorCellCount'] == 2
    assert 'source' not in row
    json.dumps(row, allow_nan=False)


@pytest.mark.parametrize('field, value', [
    ('selectedSide', 'right'), ('referencePlaneWLH', [0., 0., 1.12]),
    ('intersectionWLH', [-1.12, 4.01, 1.55]), ('intersectionWLH', [-1.10, 4., 1.55]),
])
def test_refined_evidence_must_match_current_measurement_reference(field, value):
    current = measurement()
    row = projection_row(1, 4.)
    row[field] = value
    current.rib_side_projections = [row]
    rows = rib_projections([rib()], current)
    assert_missing(rows)
    assert rows[0]['reason'] == 'rib_side_reference_mismatch'


def test_bridged_interior_cells_are_marked_as_inferred_geometry():
    item = rib()
    current = measurement()
    clean = {**projection_row(1, 4.), 'extrapolationM': .01}
    current.rib_side_projections = [clean]
    metadata = measurement_metadata(current, [item])
    assert metadata['ribProjections'][0]['valid']
    assert metadata['inferredGeometry'] is False
    json.dumps(metadata, allow_nan=False)

    current.rib_side_projections = [{**clean, 'bridgedInteriorCellCount': 2}]
    metadata = measurement_metadata(current, [item])
    assert metadata['ribProjections'][0]['valid']
    assert metadata['ribProjections'][0]['bridgedInteriorCellCount'] == 2
    assert metadata['inferredGeometry'] is True
    json.dumps(metadata, allow_nan=False)
