"""主体侧壁前后端测长；顶部篷布附加件和端壁内衬不定义侧壁长度。"""
from carpointcloud_analyzer.domain.inner_faces import peel_supported_faces
from pathlib import Path
import numpy as np
import pytest
from carpointcloud_analyzer.domain.inner_length_measurement import (
    _balanced_points, measure_final_inner_end_faces,
)


def _wall(length, width=(-1.2, 1.2), height=(0., 2.), step=.025, slope=0.):
    w, h = np.meshgrid(np.arange(width[0], width[1]+1e-8, step), np.arange(height[0], height[1]+1e-8, step))
    return np.c_[w.ravel(), length+slope*h.ravel(), h.ravel()]


def _sides(start=0., end=10., height=2.):
    length, h = np.meshgrid(np.arange(start, end+1e-8, .05), np.arange(0., height+1e-8, .05))
    return np.vstack([np.c_[np.full(length.size, side), length.ravel(), h.ravel()] for side in (-1.2, 1.2)])


def _measure(points):
    return measure_final_inner_end_faces(points, 0., 10., 0., 2.)


def test_complete_side_walls_define_length_without_end_walls():
    result = _measure(_sides())
    assert result.valid, result.metadata
    assert result.length == pytest.approx(10., abs=1e-6)
    assert result.metadata['method'] == 'body_side_wall_longitudinal_extent'


@pytest.mark.parametrize('pitch', [-np.arctan(.06), np.arctan(.06)])
def test_pitched_frame_projects_side_wall_to_calibrated_horizontal_axis(pitch):
    c, s = np.cos(pitch), np.sin(pitch)
    rotation = np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])
    points = _sides() @ rotation
    result = measure_final_inner_end_faces(
        points, 0., 10*c, points[:, 2].min(), points[:, 2].max(), np.r_[rotation[2], 0.])
    assert result.valid, result.metadata
    assert result.length == pytest.approx(10., abs=1e-6)
    assert result.project_longitudinal(np.array([[0., 3., 0.], [0., 3., 2.]]) @ rotation) == pytest.approx([3., 3.])


@pytest.mark.parametrize('front', [9.86, 9.9, 10.])
def test_front_inner_liner_does_not_shorten_main_side_wall(front):
    result = _measure(np.vstack((_sides(), _wall(0.), _wall(front))))
    assert result.valid, result.metadata
    assert result.length == pytest.approx(10., abs=1e-6)


@pytest.mark.parametrize('offset', [.02, .035, .05])
def test_s_bent_end_wall_does_not_define_main_side_wall_length(offset):
    wall = _wall(9.9)
    wall[:, 1] += offset*np.tanh((wall[:, 2]-1.)/.08)
    result = _measure(np.vstack((_sides(), _wall(0.), wall)))
    assert result.valid
    assert result.length == pytest.approx(10., abs=1e-6)


@pytest.mark.parametrize('front', [False, True])
def test_end_top_canopy_extension_does_not_lengthen_main_side(front):
    low, high = (10., 10.3) if front else (-.3, 0.)
    extension = _sides(low, high)
    extension = extension[extension[:, 2] >= 1.8]
    result = _measure(np.vstack((_sides(), extension)))
    assert result.valid
    assert result.length == pytest.approx(10., abs=1e-6)


def test_better_supported_right_side_is_used_when_left_ends_are_missing():
    sides = _sides()
    sides = sides[(sides[:, 0] > 0) | ((sides[:, 1] > 1.) & (sides[:, 1] < 9.))]
    result = _measure(sides)
    assert result.valid
    assert result.metadata['selectedSide'] == 'right'
    assert result.length == pytest.approx(10., abs=1e-6)


def test_side_wall_short_of_end_walls_is_unavailable_not_shortened():
    """侧壁在端壁前中途消失属于未观测段，不能用较短可见段或端壁锚点充当内长。"""
    result = _measure(_sides(end=6.))
    assert not result.valid
    assert result.length is None
    assert result.metadata['reason'] == 'side_wall_body_short_of_end_walls'


def test_verified_inner_faces_end_the_length_inside_end_wall_thickness():
    """用户口径：内长截止到端壁内表面，侧壁伸进端壁厚度的部分不计。"""
    walls = np.vstack((_sides(-.05, 10.05), _wall(-.05), _wall(10.05)))
    without = _measure(walls)
    assert without.length == pytest.approx(10.1, abs=1e-6)
    result = measure_final_inner_end_faces(walls, 0., 10., 0., 2., inner_face_endpoints=True)
    assert result.valid, result.metadata
    assert result.length == pytest.approx(10., abs=1e-9)
    assert result.metadata['definition'] == 'side_wall_length_between_end_wall_inner_faces'
    assert result.metadata['endpointSource'] == 'end_wall_inner_faces'


def test_inner_face_endpoints_do_not_depend_on_last_side_sample():
    # 侧壁末端采样落在内表面前 3.8cm 时，端点仍是实测内表面而不是末个采样。
    sides = _sides(0., 9.962)
    result = measure_final_inner_end_faces(sides, 0., 10., 0., 2., inner_face_endpoints=True)
    assert result.valid, result.metadata
    assert (result.rear_l, result.front_l) == pytest.approx((0., 10.), abs=1e-9)


def test_top_opening_front_edge_replaces_front_end_but_not_reach_check():
    """车头距与内长共用顶部开口前沿；侧壁仍须到达前端壁内表面。"""
    sides = _sides(0., 10.)
    result = measure_final_inner_end_faces(sides, 0., 10., 0., 2., inner_face_endpoints=True,
                                           front_endpoint=9.82)
    assert result.valid, result.metadata
    assert (result.rear_l, result.front_l) == pytest.approx((0., 9.82), abs=1e-9)
    assert result.metadata['definition'] == 'rear_end_wall_inner_face_to_top_opening_front_edge'
    short = measure_final_inner_end_faces(_sides(0., 9.5), 0., 10., 0., 2.,
                                          inner_face_endpoints=True, front_endpoint=9.82)
    assert not short.valid
    assert short.metadata['reason'] == 'side_wall_body_short_of_end_walls'


def test_side_wall_short_of_inner_face_band_stays_unavailable():
    result = measure_final_inner_end_faces(_sides(end=6.), 0., 10., 0., 2.,
                                           inner_face_endpoints=True)
    assert not result.valid
    assert result.metadata['reason'] == 'side_wall_body_short_of_end_walls'


def _oblique_visible_sides():
    """俯视远端只见侧壁上部、近端只见下部：主体高度带内的段到不了端壁。"""
    length, h = np.meshgrid(np.arange(0., 10.+1e-8, .025), np.arange(0., 2.+1e-8, .025))
    keep = (h >= .25*length - .75) & (h <= .25*length)
    return np.vstack([np.c_[np.full(keep.sum(), side), length[keep], h[keep]]
                      for side in (-1.2, 1.2)])


def test_top_view_side_extent_reaches_end_walls_when_body_band_falls_short():
    result = _measure(_oblique_visible_sides())
    assert result.valid, result.metadata
    assert result.metadata['sideExtentStage'] == 'top_view_between_end_walls'
    assert result.length == pytest.approx(10., abs=1e-6)


@pytest.mark.parametrize('front', [False, True])
def test_top_view_side_extent_excludes_canopy_beyond_end_walls(front):
    low, high = (10.05, 10.3) if front else (-.3, -.05)
    extension = _sides(low, high)
    extension = extension[extension[:, 2] >= 1.8]
    result = _measure(np.vstack((_oblique_visible_sides(), extension)))
    assert result.valid, result.metadata
    assert result.length == pytest.approx(10., abs=1e-6)


def test_body_band_result_has_no_top_view_stage_marker():
    result = _measure(_sides())
    assert 'sideExtentStage' not in result.metadata


@pytest.mark.parametrize('start,end', [(.3, 10.), (0., 9.7), (-.3, 10.3)])
def test_side_wall_within_end_wall_band_keeps_measured_extent(start, end):
    """端点在端壁搜索带内时仍按实测侧壁端点量取，锚点不替代端点。"""
    result = _measure(_sides(start, end))
    assert result.valid, result.metadata
    assert result.rear_l == pytest.approx(start, abs=1e-6)
    assert result.front_l == pytest.approx(end, abs=1e-6)


@pytest.mark.parametrize('height', [.9, 1., 1.1])
def test_low_complete_walls_keep_their_side_length(height):
    result = measure_final_inner_end_faces(_sides(height=height), 0., 10., 0., height)
    assert result.valid
    assert result.length == pytest.approx(10., abs=1e-6)


def test_position_translation_preserves_internal_length():
    cloud = _sides()
    delta = np.array([.4, 13., 4.])
    result = _measure(cloud)
    shifted = measure_final_inner_end_faces(cloud+delta, 13., 23., 4., 6.)
    assert result.valid and shifted.valid
    assert shifted.length == pytest.approx(result.length, abs=1e-6)
    assert shifted.rear_l == pytest.approx(result.rear_l+13.)


def test_duplicate_points_order_and_dense_floor_do_not_define_side_length():
    cloud = _sides()
    duplicate = np.vstack((cloud, cloud[::3], np.repeat([[0., 5., 0.]], 40000, axis=0)))
    np.random.default_rng(17).shuffle(duplicate)
    assert _measure(duplicate).length == pytest.approx(_measure(cloud).length, abs=1e-6)


def test_only_end_walls_or_top_bars_do_not_supply_a_body_side_wall():
    assert not _measure(np.vstack((_wall(0.), _wall(10.)))).valid
    sides = _sides()
    assert not _measure(sides[sides[:, 2] >= 1.9]).valid


def test_single_stray_point_does_not_extend_body_side():
    cloud = _sides()
    assert _measure(np.vstack((cloud, [[-1.2, 10.3, .8]]))).length == pytest.approx(10., abs=1e-6)


def test_peeling_single_wall_noise_does_not_create_an_inner_surface():
    random = np.random.default_rng(17)
    count = 20000
    cloud = np.c_[random.uniform(-1.05, 1.05, count), random.normal(0., .02, count), random.uniform(.15, 1.5, count)]
    faces, _ = peel_supported_faces(_balanced_points(cloud), 1, 24)
    assert len(faces) == 1
    assert abs(faces[0].at(0., .9)) < .005


def test_recorded_front_wall_residuals_do_not_become_a_crossing_plane():
    # -23363 最终车厢云的主体前端区域；这是候选误检回归，不冒充尺量真值。
    fixture = Path(__file__).parents[1] / 'fixtures' / '23363_front_body.npz'
    if not fixture.is_file():
        pytest.skip('requires the local recorded point cloud')
    with np.load(fixture) as data:
        points = data['points']
    faces, _ = peel_supported_faces(points, 1, 24)
    assert len(faces) == 1
    assert faces[0].at(0., 0.) == pytest.approx(3.878040881196915, abs=1e-10)


def test_recorded_secondary_face_cannot_borrow_a_peak_from_dense_scan_bands():
    # -23364 的面拟合已经分格等权，逐点峰值计票却让密集扫描带主导选面。
    # 位置断言固定原有主面的算法结果，不作为尺量真值。
    fixture = Path(__file__).parents[1] / 'fixtures' / '23364_front_body.npz'
    if not fixture.is_file():
        pytest.skip('requires the local recorded point cloud')
    with np.load(fixture) as data:
        points = data['points']
    faces, _ = peel_supported_faces(points, 1, 24)
    assert len(faces) == 1
    assert faces[0].at(0., 0.) == pytest.approx(3.881917839576006, abs=1e-10)
