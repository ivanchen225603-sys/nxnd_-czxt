"""车厢尾壁不能冒充车头；定向和测距只使用当前实测点。"""

import numpy as np
import pytest

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.dimensions import DimensionCalculator
from carpointcloud_analyzer.domain.models import CarriageSplitResult
from carpointcloud_analyzer.domain.heading import cab_requires_heading_reversal


@pytest.mark.parametrize('plane', [None, (0., 0., 5.)])
def test_wall_thickness_slice_is_not_a_cab(plane):
    rng = np.random.default_rng(4)
    points = rng.uniform([-1., 5.02, 1.39], [1., 5.06, 1.41], (120, 3))
    split = CarriageSplitResult(np.empty((0, 3)), 5., -5., 5.08, -5.1,
                                front_wall_plane=plane)
    assert not DimensionCalculator(Config()).measure_cab_to_front_wall_at_height(
        points, split, np.array([0., 0., 1., 0.]))
    assert not split.cab_measurement_valid
    assert split.cab_measurement_head_y is None
    assert split.cab_measurement_reason == 'insufficient_separated_cab_slice_points'


def test_dense_wall_slice_does_not_bias_sparse_real_cab_percentile():
    rng = np.random.default_rng(12)
    wall = rng.uniform([-1., 5.02, 1.39], [1., 5.06, 1.41], (20000, 3))
    cab = rng.uniform([-1., 7.8, 1.39], [1., 8., 1.41], (120, 3))
    split = CarriageSplitResult(np.empty((0, 3)), 5., -5., 8.1, -5.1,
                                front_wall_plane=(0., 0., 5.))
    assert DimensionCalculator(Config()).measure_cab_to_front_wall_at_height(
        np.vstack([wall, cab]), split, np.array([0., 0., 1., 0.]))
    assert split.cab_measurement_point_count == len(cab)
    assert split.cab_to_front_wall == pytest.approx(np.percentile(cab[:, 1], 99) - 5.)


def test_wall_exclusion_uses_pointwise_tilted_face_not_scalar_boundary():
    rng = np.random.default_rng(6)
    # split_y 是较高参考面处的交点；1.4m 截面处真实前壁为5.14，
    # 车头5.6在该面之前，却仍小于scalar split_y。不能被旧L>split_y筛掉。
    points = rng.uniform([-.2, 5.59, 1.39], [.2, 5.61, 1.41], (120, 3))
    split = CarriageSplitResult(np.empty((0, 3)), 5.7, -5., 6., -5.1,
                                front_wall_plane=(0., .1, 5.))
    assert DimensionCalculator(Config()).measure_cab_to_front_wall_at_height(
        points, split, np.array([0., 0., 1., 0.]))
    assert split.cab_measurement_front_y == pytest.approx(5.14)


def test_only_wall_in_narrow_band_does_not_prevent_wider_cab_band():
    rng = np.random.default_rng(8)
    wall = rng.uniform([-1., 5.02, 1.39], [1., 5.06, 1.41], (2000, 3))
    cab = rng.uniform([-1., 7.8, 1.43], [1., 8., 1.44], (120, 3))
    split = CarriageSplitResult(np.empty((0, 3)), 5., -5., 8.1, -5.1)
    assert DimensionCalculator(Config()).measure_cab_to_front_wall_at_height(
        np.vstack([wall, cab]), split, np.array([0., 0., 1., 0.]))
    assert split.cab_measurement_half_band == .05
    assert split.cab_measurement_point_count == len(cab)


def test_container_gutter_clearance_reference_defines_cab_distance():
    """集装箱车头距量到前端水槽净空面，而不是端壁主面。"""
    rng = np.random.default_rng(21)
    # 前壁主面 L=5.0；水槽净空面 L=4.72（凹进 28cm）；车头在 7.8~8.0。
    wall = rng.uniform([-1., 4.98, 1.39], [1., 5.02, 1.41], (3000, 3))
    cab = rng.uniform([-1., 7.8, 1.39], [1., 8., 1.41], (120, 3))
    split = CarriageSplitResult(np.empty((0, 3)), 5., -5., 8.1, -5.1)
    assert DimensionCalculator(Config()).measure_cab_to_front_wall_at_height(
        np.vstack([wall, cab]), split, np.array([0., 0., 1., 0.]),
        front_reference_l=4.72)
    assert split.cab_measurement_front_source == 'container_front_gutter_clearance'
    assert split.cab_measurement_front_y == pytest.approx(4.72)
    assert split.cab_to_front_wall == pytest.approx(np.percentile(cab[:, 1], 99) - 4.72)


def test_top_opening_front_edge_defines_long_carriage_cab_distance():
    """普通长车车头距量到顶部开口前沿，与内长前端同一位置。"""
    rng = np.random.default_rng(22)
    wall = rng.uniform([-1., 4.98, 1.39], [1., 5.02, 1.41], (3000, 3))
    cab = rng.uniform([-1., 7.8, 1.39], [1., 8., 1.41], (120, 3))
    split = CarriageSplitResult(np.empty((0, 3)), 5., -5., 8.1, -5.1,
                                front_wall_plane=(0., 0., 5.))
    assert DimensionCalculator(Config()).measure_cab_to_front_wall_at_height(
        np.vstack([wall, cab]), split, np.array([0., 0., 1., 0.]),
        front_reference_l=4.82, front_reference_source='top_opening_front_edge')
    assert split.cab_measurement_front_source == 'top_opening_front_edge'
    assert split.cab_to_front_wall == pytest.approx(np.percentile(cab[:, 1], 99) - 4.82)


def _carriage_and_cab(length=10., cab_width=2.8):
    rng = np.random.default_rng(41)
    walls = [np.column_stack([rng.uniform(-1.2, 1.2, 5000),
                             rng.normal(end, .005, 5000), rng.uniform(1.5, 3., 5000)])
             for end in (0., length)]
    floor = np.column_stack([rng.uniform(-1.2, 1.2, 5000),
                             rng.uniform(0., length, 5000), np.full(5000, 1.5)])
    carriage = np.vstack([*walls, floor])
    cab = rng.uniform([-cab_width / 2, -4., .5], [cab_width / 2, -1., 3.], (12000, 3))
    split = CarriageSplitResult(carriage, length, 0., length + .5, -4.,
                                boundaries_are_inner_faces=True)
    return carriage, cab, split


@pytest.mark.parametrize('length,cab_width', [(7., 1.8), (10., 2.8), (13., 3.0)])
def test_external_measured_cab_overrides_narrow_tail_for_different_sizes(length, cab_width):
    carriage, cab, split = _carriage_and_cab(length, cab_width)
    # 大量尾部附件回波，纵向只有一薄层；不能凭点数认作车头。
    tail = np.random.default_rng(9).uniform(
        [-1., length + .4, 1.], [1., length + .5, 2.5], (20000, 3))
    points = np.vstack([carriage, cab, tail])
    original = points.copy()
    assert cab_requires_heading_reversal(points, split)
    assert cab_requires_heading_reversal(points[::-1], split)
    np.testing.assert_array_equal(points, original)


@pytest.mark.parametrize('both_sides', [False, True])
def test_positive_cab_or_both_sides_supported_preserves_initial_direction(both_sides):
    carriage, cab, split = _carriage_and_cab()
    positive_cab = cab.copy()
    positive_cab[:, 1] = split.split_y - positive_cab[:, 1]
    points = np.vstack([carriage, positive_cab, cab if both_sides else carriage[:0]])
    assert not cab_requires_heading_reversal(points, split)


@pytest.mark.parametrize('kind', [
    'wall', 'ground', 'pole', 'repeated', 'sparse', 'disconnected', 'diagonal_rod',
])
def test_insufficient_external_structure_cannot_force_direction(kind):
    carriage, cab, split = _carriage_and_cab()
    if kind == 'wall':
        cab[:, 1] = -.4
    elif kind == 'ground':
        cab[:, 2] = 0.
    elif kind == 'pole':
        cab[:, 0] *= .03
    elif kind == 'repeated':
        cab[:] = cab[0]
    elif kind == 'sparse':
        cab = cab[:50]
    elif kind == 'disconnected':
        cab[:, 1] = np.where(cab[:, 1] < -2.5, -4., -1.)
    elif kind == 'diagonal_rod':
        t = np.linspace(0., 1., len(cab))
        cab = np.column_stack([-1.1 + 2.2 * t, -4. + 2.5 * t, .5 + 2. * t])
        cab += np.random.default_rng(42).normal(0., .003, cab.shape)
    assert not cab_requires_heading_reversal(np.vstack([carriage, cab]), split)


def test_disconnected_wall_and_thin_beam_cannot_pool_body_coverage():
    carriage, _, split = _carriage_and_cab()
    rng = np.random.default_rng(43)
    wall = rng.uniform([-1.2, -.45, 1.5], [1.2, -.4, 3.], (12000, 3))
    beam = rng.uniform([-.03, -2., 1.37], [.03, -1., 1.43], (12000, 3))
    for external in (wall, beam, np.vstack([wall, beam])):
        assert not cab_requires_heading_reversal(np.vstack([carriage, external]), split)


def test_unverified_walls_cannot_override_direction():
    carriage, cab, split = _carriage_and_cab()
    split.boundaries_are_inner_faces = False
    assert not cab_requires_heading_reversal(np.vstack([carriage, cab]), split)


@pytest.mark.parametrize('yaw', [0., .8])
def test_pipeline_reorients_all_consumers_and_restores_source_coordinates(monkeypatch, yaw):
    from types import SimpleNamespace
    from unittest.mock import Mock

    from carpointcloud_analyzer.application import pipeline
    from carpointcloud_analyzer.domain.geometry import CoordinateTransform, LocalCarriageExtractor
    from carpointcloud_analyzer.domain.models import LoadStatusResult, RibInfo, VehicleDimensions

    carriage, cab, _ = _carriage_and_cab()
    initial = np.vstack([carriage, cab])
    rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0.],
                         [np.sin(yaw), np.cos(yaw), 0.], [0., 0., 1.]])
    transform = CoordinateTransform(np.array([3., -2., .4]), rotation,
                                    vertical_alignment=1., vertical_source='weighbridge')
    source = transform.to_source(initial)
    original = source.copy()
    expected_local = initial * [-1., -1., 1.]
    expected_split = LocalCarriageExtractor(Config()).split(expected_local)
    analyzer = pipeline.PointCloudAnalyzer(Config(save_clean_pointcloud=False))
    monkeypatch.setattr(analyzer.geometry, 'process', lambda *args: (initial, transform))
    real_split = analyzer.local_splitter.split
    splitter = Mock(side_effect=real_split)
    monkeypatch.setattr(analyzer.local_splitter, 'split', splitter)
    measure = Mock()
    monkeypatch.setattr(analyzer.dim_calc, 'measure_cab_to_front_wall_at_height', measure)
    dimensions = VehicleDimensions(14., 2.8, 2.5, 10., 2.4, 1.5)
    calc = Mock(return_value=dimensions)
    monkeypatch.setattr(analyzer.dim_calc, 'calculate', calc)
    profile = SimpleNamespace(interior_w_range=(-1.2, 1.2), floor_z=0., wall_top=2.5,
                              measurement_confidence=1.)
    monkeypatch.setattr(analyzer.floor_analyzer, 'analyze', Mock(return_value=None))
    profile_call = Mock(return_value=profile)
    monkeypatch.setattr(analyzer.carriage_analyzer, 'analyze', profile_call)
    load = Mock(return_value=LoadStatusResult('Empty', 1., 'test', {}))
    monkeypatch.setattr(analyzer, '_assess_load', load)
    floor_measure = Mock(wraps=analyzer._measure_current_floor)
    monkeypatch.setattr(analyzer, '_measure_current_floor', floor_measure)
    completion = Mock(return_value=SimpleNamespace(points=np.empty((0, 3)), metadata={}))
    monkeypatch.setattr(pipeline, 'complete_empty_carriage_walls', completion)
    rods = [RibInfo(0, position, 0., 2., (-1., 1.), (2., 2.), 100, 2., .1)
            for position in (-8., -2., -5.)]
    detect = Mock(return_value=rods)
    monkeypatch.setattr(analyzer.local_rib_detector, 'detect', detect)
    build = Mock(return_value=dict(integrationPayloads={}, dimensions=None,
                                   measurementStatus={}, loadStatus={}, wallCompletion={}))
    monkeypatch.setattr(pipeline.ResultExporter, 'build', build)
    ground = np.array([.02, -.03, 1., -.4])
    scene = SimpleNamespace(preprocessed=SimpleNamespace(primary_vehicle_points=source),
                            vertical_reference=ground[:3], vertical_source='weighbridge',
                            site_frame=None, height_plane_analysis=ground,
                            height_plane_source=ground, height_plane_calibrated=True)
    analyzer._analyze_standard(scene, pipeline._AnalysisMode('test.pcd', False, False, False))

    assert splitter.call_count == 2
    for call in (measure, calc, completion):
        np.testing.assert_array_equal(call.call_args.args[0], expected_local)
    actual_split = measure.call_args.args[1]
    assert actual_split.split_y == pytest.approx(expected_split.split_y)
    assert actual_split.front_wall_plane == pytest.approx(expected_split.front_wall_plane)
    np.testing.assert_array_equal(profile_call.call_args.args[0], expected_split.carriage_points)
    assert load.call_args.args[0] is actual_split
    corrected = floor_measure.call_args.args[4]
    np.testing.assert_allclose(corrected.to_source(expected_local), source, atol=1e-12)
    np.testing.assert_array_equal(corrected.rotation[:, 2], transform.rotation[:, 2])
    assert np.linalg.det(corrected.rotation) == pytest.approx(1.)
    np.testing.assert_allclose(measure.call_args.args[2], corrected.plane_to_local(ground))
    local_ground = corrected.plane_to_local(ground)
    np.testing.assert_allclose(expected_local @ local_ground[:3] + local_ground[3],
                               source @ ground[:3] + ground[3], atol=1e-12)
    assert detect.call_count == 2
    np.testing.assert_allclose(detect.call_args.args[0], expected_split.carriage_points, atol=1e-12)
    assert [rib.y_position for rib in build.call_args.args[0]] == [-2., -5., -8.]
    assert [rib.rib_id for rib in build.call_args.args[0]] == [1, 2, 3]
    np.testing.assert_array_equal(source, original)
    np.testing.assert_array_equal(transform.rotation, rotation)
