"""矩形最终云全字段：方向、固定高度截面和无底板时的壁面跨度。"""

import numpy as np
import pytest

from carpointcloud_analyzer import Config
from carpointcloud_analyzer.domain.box_geometry import rotate_wl
from carpointcloud_analyzer.domain.geometry import CoordinateTransform
from carpointcloud_analyzer.application.review_measurements import calculate_review_measurements
from carpointcloud_analyzer.domain.wall_reference import complete_rectangular_wall_reference


def shell_with_cab():
    rng = np.random.default_rng(88)
    walls = [np.column_stack([rng.uniform(-1.25, 1.25, 6000),
                              rng.normal(end, .005, 6000), rng.uniform(1.5, 4.1, 6000)])
             for end in [0., 6.]]
    rim = np.column_stack([np.full(1000, 1.25), rng.uniform(0, 6, 1000),
                           rng.uniform(3.95, 4.1, 1000)])
    cab = np.column_stack([rng.uniform(-1, 1, 2000), rng.uniform(7.5, 10, 2000),
                           rng.normal(1.4, .005, 2000)])
    return np.vstack([*walls, rim, cab])


def final_wall_height(points, reference):
    final = np.vstack((points, reference.points))
    walls = final[(final[:, 2] >= reference.box.h_range[0] - .05)
                  & (final[:, 2] <= reference.box.h_range[1] + .05)]
    return float(np.ptp(walls[:, 2]))


@pytest.mark.parametrize('flip,yaw', [(False, 0), (True, 0), (False, 4), (True, -4)])
def test_all_review_values_include_cab_measurement_and_model_height(flip, yaw):
    p = shell_with_cab()
    if flip:
        p[:, :2] *= -1
    p = rotate_wl(p, np.deg2rad(yaw))
    cfg = Config()
    reference = complete_rectangular_wall_reference(p, cfg)
    assert reference.box is not None, reference.metadata
    transform = CoordinateTransform(np.zeros(3), np.eye(3))
    result = calculate_review_measurements(p, reference, transform, np.array([0, 0, 1., 0]), True, cfg)
    assert result['dimensions']['vehicle']['length'] == pytest.approx(10, abs=.1)
    assert result['dimensions']['carriage']['length'] == pytest.approx(6, abs=.05)
    assert result['cabToTrailerFrontDist'] == pytest.approx(3.97, abs=.1)
    assert result['cabMeasurement']['heightAboveGround'] == 1.4
    assert result['cabMeasurement']['pointCount'] > 100
    assert result['modelLowerEdgeToWeighbridgeHeight'] == pytest.approx(1.5, abs=.06)
    assert result['dimensions']['carriage']['height'] == pytest.approx(2.6, abs=.05)
    assert result['dimensions']['carriage']['height'] == final_wall_height(p, reference)
    assert result['fields']['dimensions.carriage.height'] == 'completed_cloud_wall_height'
    assert result['carriageHeight']['exteriorModelHeightM'] == pytest.approx(2.6, abs=.06)
    assert result['vehicleTopToWeighbridgeHeight'] == pytest.approx(4.1, abs=.05)
    assert result['trailerBottomToGroundHeight'] is None
    assert result['floorProfile'] is None and result['highLowPlateHeight'] is None
    assert result['ribCount'] is None and result['detectedRibCount'] == 0
    assert result['wireCount'] is None and result['fields']['wires'] == 'detector_not_implemented'
    reference.points[:] = 1e6
    with pytest.raises(ValueError, match='completed rectangular wall bounds'):
        calculate_review_measurements(p, reference, transform, np.array([0, 0, 1., 0]), True, cfg)


@pytest.mark.parametrize('estimated_plane', [None, np.array([0., 0., 1., 0.])])
def test_ground_height_is_not_invented_without_calibration(estimated_plane):
    p = shell_with_cab()
    cfg = Config()
    reference = complete_rectangular_wall_reference(p, cfg)
    result = calculate_review_measurements(p, reference, CoordinateTransform(np.zeros(3), np.eye(3)),
                                           estimated_plane, False, cfg)
    assert result['dimensions']['vehicle']['length'] > 9
    assert result['cabToTrailerFrontDist'] is None
    assert result['cabMeasurement']['reason'] == 'weighbridge_plane_unavailable'
    assert result['modelLowerEdgeToWeighbridgeHeight'] is None
    assert result['modelUpperEdgeToWeighbridgeHeight'] is None


def test_partial_wall_completion_does_not_require_full_box_span():
    """端壁已观测到时仅补中间缺口，最终尺寸仍从实测加补点共同量取。"""
    from dataclasses import replace
    p = shell_with_cab()
    cfg = Config()
    reference = complete_rectangular_wall_reference(p, cfg)
    transform = CoordinateTransform(np.zeros(3), np.eye(3))
    expected = calculate_review_measurements(p, reference, transform, None, False, cfg)
    middle = reference.points[(reference.points[:, 1] > 1.) & (reference.points[:, 1] < 5.)]
    assert len(middle) > 0 and np.ptp(middle[:, 1]) < reference.box.length - 1.
    actual = calculate_review_measurements(p, replace(reference, points=middle), transform, None, False, cfg)
    assert actual['dimensions']['carriage']['length'] == pytest.approx(expected['dimensions']['carriage']['length'], abs=.02)
    assert actual['dimensions']['carriage']['width'] == pytest.approx(expected['dimensions']['carriage']['width'], abs=.02)


@pytest.mark.parametrize('observed', [2.62, 2.68])
def test_completed_cloud_height_tracks_current_observation_anchor(observed):
    from carpointcloud_analyzer.application.review_measurements import build_review_dimension_payload

    p = shell_with_cab()
    # 仅调整厢壁实际观测跨度，保持车头与独立竖直基准不变。
    mask = p[:, 1] < 6.1
    p[mask, 2] = 4.1 - (4.1 - p[mask, 2]) * observed / 2.6
        # 车型识别窗口变化不得改变任何高度：补全云的高度仍由本次观测锚定。
    values = []
    for window in [(5.7, 6.4), (5.5, 6.6), (5.9, 6.1)]:
        cfg = Config(carriage_wall_reference_profile_length_range=window)
        ref = complete_rectangular_wall_reference(p, cfg)
        result = calculate_review_measurements(p, ref, CoordinateTransform(np.zeros(3), np.eye(3)),
                                               None, False, cfg)
        inner = result['dimensions']['carriage']['height']
        assert inner == pytest.approx(observed, abs=.03)
        # 外形模型高与内高同源，不再有独立的配置外高。
        assert result['carriageHeight']['exteriorModelHeightM'] == pytest.approx(inner, abs=.03)
        payload = build_review_dimension_payload(result, cfg, 'source.pcd')
        assert payload['dimensions']['trailerBodyHeight'] == round(inner * 100, 1)
        assert payload['detectStatus'] == 'success'
        values.append(inner)
    assert np.ptp(values) < 1e-12


def test_unmatched_reference_measures_final_cloud_without_profile_metadata():
    p = shell_with_cab()
    cfg = Config(carriage_wall_reference_profile_length_range=(10., 13.))
    ref = complete_rectangular_wall_reference(p, cfg)
    result = calculate_review_measurements(p, ref, CoordinateTransform(np.zeros(3), np.eye(3)),
                                           None, False, cfg)
    assert result['dimensions']['carriage']['height'] == final_wall_height(p, ref)
    assert 'carriageHeight' not in result


def test_completed_cloud_height_does_not_depend_on_observed_height_metadata():
    p = shell_with_cab()
    cfg = Config()
    ref = complete_rectangular_wall_reference(p, cfg)
    ref.metadata.pop('observedHeightM')
    result = calculate_review_measurements(
        p, ref, CoordinateTransform(np.zeros(3), np.eye(3)), None, False, cfg)
    assert result['dimensions']['carriage']['height'] == pytest.approx(2.6, abs=.05)


def test_completed_cloud_geometry_changes_measured_height():
    p = shell_with_cab()
    cfg = Config()
    ref = complete_rectangular_wall_reference(p, cfg)
    transform = CoordinateTransform(np.zeros(3), np.eye(3))
    before = calculate_review_measurements(p, ref, transform, None, False, cfg)
    before_height = final_wall_height(p, ref)
    top = np.max(ref.points[:, 2])
    ref.points[ref.points[:, 2] >= top - .001, 2] += .02
    after = calculate_review_measurements(p, ref, transform, None, False, cfg)
    assert before['dimensions']['carriage']['height'] == before_height
    assert after['dimensions']['carriage']['height'] == final_wall_height(p, ref)
    assert after['dimensions']['carriage']['height'] - before_height == pytest.approx(.02, abs=.001)
