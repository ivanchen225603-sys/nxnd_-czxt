"""最终云底板来源及短箱底板范围门禁回归。"""

from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from carpointcloud_analyzer.domain.carriage import CarriageAnalyzer
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.floor import (
    CASE_FLAT, CASE_FULL_SLOPE, CASE_STEP_90, CASE_STEP_RAMP,
    FloorProfile, FloorSegment,
)
from carpointcloud_analyzer.domain.geometry import CoordinateTransform
from carpointcloud_analyzer.domain.models import (
    BoxCarriageBox, CarriageSplitResult, LoadStatusResult, LongitudinalMeasurement,
)
from carpointcloud_analyzer.application.pipeline import PointCloudAnalyzer


_PLANE = np.array([0., 0., 1., 1.5])
_TRANSFORM = CoordinateTransform(np.zeros(3), np.eye(3))
_EMPTY = LoadStatusResult('Empty', 1., 'test', {})
_BOX = BoxCarriageBox(6., 0., (-1.2, 1.2), (0., 2.8), 1000, 1000)


def _flat_cloud():
    w, longitudinal = np.meshgrid(np.linspace(-.95, .95, 50),
                                  np.linspace(.2, 5.8, 150))
    floor = np.c_[w.ravel(), longitudinal.ravel(), np.zeros(w.size)]
    longitudinal, height = np.meshgrid(np.linspace(0., 6., 150),
                                       np.linspace(0., 2.8, 50))
    walls = [np.c_[np.full(longitudinal.size, side),
                   longitudinal.ravel(), height.ravel()] for side in (-1.2, 1.2)]
    return np.vstack((floor, *walls))


def _measure(analyzer, split, profile, final, modeled=None):
    return analyzer._measure_current_floor(
        split, profile, _EMPTY, None, _TRANSFORM, _PLANE, _PLANE, True,
        modeled, final_carriage_points=final,
    )


@pytest.mark.parametrize('missing_measurement', [None, LongitudinalMeasurement(
    None, None, None, {'reason': 'end_faces_unavailable'})])
def test_missing_length_still_measures_floor_from_final_cloud(missing_measurement):
    analyzer = PointCloudAnalyzer(Config())
    final = _flat_cloud()
    profile = CarriageAnalyzer(analyzer.config).analyze(final)
    # 原分割副本的点不足以测底板；最终云有完整有效底板。
    split = CarriageSplitResult(final[:2], 6., 0., 10., 0.,
                                boundaries_are_inner_faces=True,
                                longitudinal_measurement=missing_measurement)
    result = _measure(analyzer, split, profile, final)
    assert result is not None
    assert result.case == CASE_FLAT
    assert result.segments[0].height_to_ground == pytest.approx(1.5)
    assert len(split.carriage_points) == 2


def test_rectangular_flat_floor_keeps_calibrated_height():
    analyzer = PointCloudAnalyzer(Config())
    final = _flat_cloud()
    profile = CarriageAnalyzer(analyzer.config).analyze(final)
    split = CarriageSplitResult(final, 6., 0., 10., 0., observed_outer_box=_BOX)
    result = _measure(analyzer, split, profile, final)
    assert result is not None
    assert result.case == CASE_FLAT
    assert result.measured_height_range is None  # 此属性专供整体坡面。
    assert result.segments[0].height_to_ground == pytest.approx(1.5)
    assert result.ground_plane_source == 'weighbridge_calibration'


def test_rectangular_final_floor_has_no_duplicate_model_weight(monkeypatch):
    analyzer = PointCloudAnalyzer(Config())
    raw = np.array([[0., 1., 0.], [0., 2., 0.]])
    modeled = np.array([[0., 3., .04], [0., 4., .04]])
    final = np.vstack((raw, modeled, [[1.2, 3., 2.8]]))
    split = CarriageSplitResult(raw, 6., 0., 10., 0., observed_outer_box=_BOX)
    seen = []
    monkeypatch.setattr(analyzer, '_analyze_floor_profile',
                        lambda selected, *args: seen.append(selected.carriage_points))
    _measure(analyzer, split, SimpleNamespace(floor_z=0.), final, modeled)
    np.testing.assert_array_equal(seen[0], np.vstack((raw, modeled)))
    # 兼容无最终云的旧离线入口，同样只引入一份补点。
    _measure(analyzer, split, SimpleNamespace(floor_z=0.), None, modeled)
    np.testing.assert_array_equal(seen[1], seen[0])


@pytest.mark.parametrize('case, heights, accepted', [
    (CASE_FLAT, [1.5], True),
    (CASE_STEP_90, [1.5, 1.9, 1.5], True),
    (CASE_STEP_RAMP, [1.5, 1.9, 1.5], True),
    (CASE_STEP_90, [1.5, 1.5 + 2.8 * .30], True),
    (CASE_STEP_90, [1.5, 1.5 + 2.8 * .30 + 1e-6], False),
    (CASE_FLAT, [], False),
    (CASE_STEP_90, [1.5, float('nan')], False),
    (CASE_STEP_RAMP, [1.5, float('inf')], False),
])
def test_rectangular_floor_range_preserves_original_box_limit(
    monkeypatch, case, heights, accepted,
):
    analyzer = PointCloudAnalyzer(Config())
    points = np.array([[0., 1., 0.]])
    split = CarriageSplitResult(points, 6., 0., 10., 0., observed_outer_box=_BOX)
    floor = FloorProfile(case, _PLANE, 'weighbridge_calibration', segments=[
        FloorSegment(index + 1, 'test', height, 2.)
        for index, height in enumerate(heights)
    ])
    monkeypatch.setattr(analyzer, '_analyze_floor_profile', lambda *args: floor)
    result = _measure(analyzer, split, SimpleNamespace(floor_z=0.), points)
    assert (result is not None) is accepted


def test_rectangular_slope_uses_all_profile_bins_not_segment_extrema(monkeypatch):
    analyzer = PointCloudAnalyzer(Config())
    points = np.array([[0., 1., 0.]])
    split = CarriageSplitResult(points, 6., 0., 10., 0., observed_outer_box=_BOX)
    floor = FloorProfile(
        CASE_FULL_SLOPE, _PLANE, 'weighbridge_calibration',
        segments=[FloorSegment(1, 'slope', 1.5, 6.)],
        profile_s=np.arange(4.), profile_h=np.array([1.5, 1.6, 1.7, 2.4]),
    )
    monkeypatch.setattr(analyzer, '_analyze_floor_profile', lambda *args: floor)
    assert _measure(analyzer, split, SimpleNamespace(floor_z=0.), points) is None
    floor = replace(floor, profile_h=np.array([1.5, 1.6, 1.7, 1.8]))
    assert _measure(analyzer, split, SimpleNamespace(floor_z=0.), points) is floor
    # 无有效坡面剖面时，不用少数等效测量点代替全长极值。
    floor = replace(floor, profile_h=None)
    assert _measure(analyzer, split, SimpleNamespace(floor_z=0.), points) is None
