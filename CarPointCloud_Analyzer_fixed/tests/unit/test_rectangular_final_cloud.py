"""矩形外壳的最终点集、判空、量尺、拉筋及导出必须一致。"""

from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from carpointcloud_analyzer import Config
from carpointcloud_analyzer.domain.box_geometry import rotate_wl
from carpointcloud_analyzer.application.exporters import ResultExporter
from carpointcloud_analyzer.domain.geometry import CoordinateTransform
from carpointcloud_analyzer.application.pipeline import PointCloudAnalyzer, _AnalysisMode
from carpointcloud_analyzer.application.review_measurements import (
    analyze_rectangular_final_cloud, build_review_dimension_payload,
)
from carpointcloud_analyzer.domain.wall_reference import complete_rectangular_wall_reference
from test_review_measurements import shell_with_cab


def surface(height=1.5, end=5.8):
    width, length = np.meshgrid(np.linspace(-.95, .95, 49), np.linspace(.2, end, 141))
    return np.c_[width.ravel(), length.ravel(), np.full(width.size, height)]


@pytest.fixture
def shell():
    points = shell_with_cab()
    return points, complete_rectangular_wall_reference(points, Config())


def analyze(points, reference, config=None, ground=True):
    return analyze_rectangular_final_cloud(
        points, reference, CoordinateTransform(np.zeros(3), np.eye(3)),
        np.array([0., 0., 1., 0.]) if ground else None, ground, config or Config())


def export(final, config=None):
    return ResultExporter.build(
        final.ribs, final.dimensions, final.split, final.load_status, 'test.pcd',
        config or Config(), profile=final.profile, floor_profile=final.floor,
        wall_completion=final.completion)


def test_completed_floor_is_used_for_load_and_height_not_original_only(shell):
    points, reference = shell
    original = points.copy()
    assert analyze(points, reference).load_status.status == 'Uncertain'
    reference = replace(reference, points=np.vstack((reference.points, surface())))
    final = analyze(points, reference)
    assert final.load_status.status == 'Empty'
    assert final.floor is not None
    assert final.dimensions.carriage_height == pytest.approx(2.6, abs=.04)
    assert final.dimensions.height_measurement['floorH'] == pytest.approx(1.5)
    assert export(final)['integrationPayloads']['vehicleDimensions']['detectStatus'] == 'success'
    np.testing.assert_array_equal(points, original)
    np.testing.assert_allclose(final.points[:len(points)], original, atol=1e-12)


def test_unresolved_floor_wall_span_is_not_reported_as_inner_height(shell):
    from carpointcloud_analyzer.application.reference_outputs import build_rectangular_result

    points, reference = shell
    final = analyze(points, reference)
    assert final.load_status.status == 'Uncertain'
    assert np.isnan(final.dimensions.carriage_height)
    assert final.review['carriageVisibleWallSpanM'] == pytest.approx(2.6, abs=.04)
    result = build_rectangular_result(
        final.review, reference, CoordinateTransform(np.zeros(3), np.eye(3)), None,
        final.load_status.to_dict(), Config(), 'test.pcd', '', 0, exported_result=export(final))
    assert result['reviewMeasurements']['dimensions']['carriage']['height'] is None
    assert result['reviewMeasurements']['carriageHeight']['innerHeightM'] is None
    assert result['integrationPayloads']['vehicleDimensions']['dimensions']['trailerBodyHeight'] is None
    assert result['integrationPayloads']['vehicleDimensions']['dimensions']['trailerLength'] > 0


@pytest.mark.parametrize('flip,yaw', [(False, 0), (True, 0), (False, 4), (True, -4)])
def test_completion_returns_same_reference_frame_and_does_not_move_original(shell, flip, yaw):
    points, reference = shell
    points = np.vstack((points, surface(end=2.95)))
    if flip:
        points[:, :2] *= -1
        reference.points[:, :2] *= -1
        box = type(reference.box)(
            -reference.box.rear_l, -reference.box.front_l,
            (-reference.box.w_range[1], -reference.box.w_range[0]),
            reference.box.h_range, reference.box.front_points, reference.box.rear_points)
        reference = replace(reference, box=box)
    points = rotate_wl(points, np.deg2rad(yaw))
    reference = replace(reference, points=rotate_wl(reference.points, np.deg2rad(yaw)))
    reference.metadata['yawDeg'] += yaw
    original = points.copy()
    final = analyze(points, reference)
    assert final.load_status.status == 'Empty'
    assert final.completion['frontFloor']['synthesizedPoints'] > 0
    assert final.dimensions.carriage_length == pytest.approx(6., abs=.005)
    assert final.dimensions.carriage_width == pytest.approx(2.5, abs=.005)
    np.testing.assert_array_equal(points, original)
    np.testing.assert_allclose(final.points[:len(points)], original, atol=1e-12)


def test_synthetic_floor_is_not_load_evidence(shell, monkeypatch):
    import carpointcloud_analyzer.application.review_measurements as review

    points, reference = shell
    captured = []
    classify = review.LoadStatusClassifier.classify

    def capture(self, cloud, profile=None):
        captured.append(cloud.copy())
        return classify(self, cloud, profile)

    monkeypatch.setattr(review.LoadStatusClassifier, 'classify', capture)
    final = analyze(np.vstack((points, surface(end=2.95))), reference)

    def unobserved_floor(cloud):
        return ((np.abs(cloud[:, 0]) < .9) & (cloud[:, 1] > 3.1) & (cloud[:, 1] < 5.8)
                & (np.abs(cloud[:, 2] - 1.5) < .02))

    assert final.completion['frontFloor']['synthesizedPoints'] > 0
    assert np.count_nonzero(unobserved_floor(final.points)) > 0
    assert np.count_nonzero(unobserved_floor(captured[0])) == 0
    assert final.load_status.status == 'Empty'


def test_vehicle_height_is_top_above_weighbridge(shell):
    points, reference = shell
    final = analyze(np.vstack((points, surface())), reference)
    assert final.dimensions.vehicle_height == pytest.approx(4.1, abs=.02)
    assert final.dimensions.vehicle_height_measurement['definition'] == 'top_above_ground_plane'
    assert final.review['fields']['vehicle.height'] == 'completed_cloud_top_above_weighbridge'


@pytest.mark.parametrize('height', [2.5, 3.7])
def test_high_flat_cargo_does_not_become_empty_through_floor_completion(shell, height):
    points, reference = shell
    final = analyze(np.vstack((points, surface(height))), reference)
    assert final.load_status.status != 'Empty'
    result = export(final)
    assert result['floorProfile'] is None
    assert result['ribs'] == []
    if final.load_status.status == 'Loaded':
        assert not result['integrationPayloads']
    else:
        assert result['integrationPayloads']['vehicleDimensions']['dimensions']['trailerBodyHeight'] is None


def test_missing_cab_measurement_preserves_other_final_dimensions(shell):
    points, reference = shell
    final = analyze(np.vstack((points, surface())), reference, ground=False)
    result = export(final)
    payload = result['integrationPayloads']['vehicleDimensions']
    assert payload['dimensions']['cabToTrailerFrontDist'] is None
    assert payload['dimensions']['trailerLength'] == pytest.approx(600., abs=.5)
    assert payload['dimensions']['trailerWidth'] == pytest.approx(250., abs=.5)
    assert payload['dimensions']['trailerBodyHeight'] > 250.


def test_inner_height_uses_final_floor_not_shell_bottom(shell):
    points, reference = shell
    final = analyze(np.vstack((points, surface(1.6))), reference)
    assert final.load_status.status == 'Empty'
    assert final.dimensions.height_measurement['floorH'] == pytest.approx(1.6)
    assert final.dimensions.carriage_height == pytest.approx(2.5, abs=.04)


@pytest.mark.parametrize('spacing', [.02, .04])
def test_floor_scan_stripes_cannot_become_ribs_after_completion(shell, spacing):
    points, reference = shell
    w, length = np.meshgrid(np.arange(-1., 1., spacing), np.arange(.15, 5.85, spacing))
    floor = np.c_[w.ravel(), length.ravel(), np.full(w.size, reference.box.h_range[0])]
    final = analyze(np.vstack((points, floor)), reference)
    assert final.load_status.status == 'Empty'
    assert final.ribs == []
    assert export(final)['integrationPayloads']['vehicleDimensions']['actualRibCount'] == 0


def test_ribs_and_intervals_are_detected_in_final_cloud(shell):
    points, reference = shell
    w, theta = np.meshgrid(np.arange(-1.19, 1.191, .01), np.arange(8) * np.pi / 4)
    ribs = [np.c_[w.ravel(), position + .008 * np.cos(theta.ravel()),
                  3.8 + .008 * np.sin(theta.ravel())]
            for position in (1.5, 3., 4.5)]
    # 仅最终云含杆件，原输入没有；检测和同侧交点均必须读取最终点集。
    reference = replace(reference, points=np.vstack((reference.points, surface(), *ribs)))
    final = analyze(points, reference)
    assert final.load_status.status == 'Empty'
    assert len(final.ribs) == 3
    assert final.review['ribPositionsM'] == pytest.approx([4.5, 3., 1.5], abs=.01)
    assert final.review['ribIntervals'] == pytest.approx([1.5] * 4, abs=.01)
    payload = export(final)['integrationPayloads']['vehicleDimensions']
    assert payload['actualRibCount'] == 3
    distances = [row['distToPrev'] for row in payload['ribs']]
    assert len(distances) == 4
    assert sum(distances) == pytest.approx(payload['dimensions']['trailerLength'])


def test_partial_floor_and_high_surface_are_not_filled_into_empty(shell):
    points, reference = shell
    low = surface(end=2.95)
    high = surface(3.2)
    high = high[high[:, 1] > 3.05]
    final = analyze(np.vstack((points, low, high)), reference)
    assert final.completion['frontFloor']['occludedFloorCandidates'] > 0
    assert final.load_status.status != 'Empty'
    assert not export(final)['integrationPayloads']


@pytest.mark.parametrize('field,value', [('length', 31.), ('width', 4.5), ('height', 6.),
                                         ('width', float('inf'))])
def test_legacy_conversion_keeps_computed_values_but_rejects_nonfinite(shell, field, value):
    points, reference = shell
    final = analyze(np.vstack((points, surface())), reference)
    review = deepcopy(final.review)
    review['dimensions']['carriage'][field] = value
    if not np.isfinite(value):
        with pytest.raises(ValueError, match='invalid review measurement'):
            build_review_dimension_payload(review, Config(), 'test.pcd')
    else:
        payload = build_review_dimension_payload(review, Config(), 'test.pcd')
        key = {'length': 'trailerLength', 'width': 'trailerWidth', 'height': 'trailerBodyHeight'}[field]
        assert payload['dimensions'][key] == value * 100
        assert payload['detectStatus'] == 'success'


def test_formal_export_uses_same_range_policy_as_standard_branch(shell):
    points, reference = shell
    final = analyze(np.vstack((points, surface())), reference)
    cfg = Config(sanity_vehicle_width=(1., 2.4))
    result = export(final, cfg)
    assert result['measurementStatus']['calculationSucceeded']
    assert result['integrationPayloads']['vehicleDimensions']['dimensions']['trailerWidth'] > 240
    assert result['integrationPayloads']['vehicleDimensions']['detectStatus'] == 'success'


def test_pipeline_archives_exact_final_cloud_not_wall_only(shell, monkeypatch):
    import carpointcloud_analyzer.application.pipeline as pipeline

    points, reference = shell
    points = np.vstack((points, surface(end=2.95)))
    cfg = Config(save_clean_pointcloud=True, carriage_wall_reference_front_lip_enable=False)
    analyzer = PointCloudAnalyzer(cfg)
    captured = {}
    actual_analyze = pipeline.analyze_rectangular_final_cloud

    def capture_analysis(*args, **kwargs):
        captured['final'] = actual_analyze(*args, **kwargs)
        return captured['final']

    def capture_save(output, path):
        captured['saved'] = output.copy()
        return 'final.pcd'

    monkeypatch.setattr(pipeline, 'analyze_rectangular_final_cloud', capture_analysis)
    monkeypatch.setattr(analyzer, '_save_clean_pointcloud', capture_save)
    scene = SimpleNamespace(
        preprocessed=SimpleNamespace(primary_vehicle_points=points, ground_plane=None),
        measurement_points=None, site_frame=None,
        height_plane_analysis=np.array([0., 0., 1., 0.]), height_plane_calibrated=True)
    result = analyzer._analyze_rectangular(
        (points, CoordinateTransform(np.zeros(3), np.eye(3))), reference, scene,
        _AnalysisMode('original.pcd', False, False, False))
    assert result['loadStatus']['status'] == 'Empty'
    assert result['pcd'] == 'final.pcd'
    assert result['pcdPointCount'] == len(captured['final'].points)
    assert len(captured['saved']) > len(points) + len(reference.points)
    np.testing.assert_array_equal(captured['saved'], captured['final'].points)
    expected = export(captured['final'], cfg)['integrationPayloads']
    expected['vehicleDimensions'].update(idempotencyKey='original', sourceFile='original.pcd')
    assert result['integrationPayloads'] == expected
