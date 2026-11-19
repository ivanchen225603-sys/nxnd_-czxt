"""坡面整体高度极值与既有三段阶梯测量的独立性回归。"""

import numpy as np
import pytest

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.floor import (
    CASE_FLAT,
    CASE_FULL_SLOPE,
    CASE_STEP_90,
    CASE_STEP_RAMP,
    FloorProfile,
    FloorProfileAnalyzer,
)


def _slope_profile(s, h, carriage_length=12.0):
    """直接构造有效底板分箱，三段仍调用正式的规格实现。"""
    samples_s = np.asarray(s, dtype=float)
    samples_h = np.asarray(h, dtype=float)
    analyzer = FloorProfileAnalyzer(Config())
    return FloorProfile(
        case=CASE_FULL_SLOPE,
        ground_plane=np.array([0.0, 0.0, 1.0, 0.0]),
        ground_plane_source="weighbridge_calibration",
        segments=analyzer._build_segments(
            CASE_FULL_SLOPE, samples_s, samples_h, carriage_length, None
        ),
        profile_s=samples_s,
        profile_h=samples_h,
    )


def test_descending_slope_range_includes_observed_rear_end():
    samples_s = np.linspace(0.1, 11.9, 60)
    profile = _slope_profile(samples_s, 2.6 - samples_s * 0.05)

    assert profile.measured_height_range == pytest.approx((2.005, 2.595))
    # P3 必须仍是 s=8m 的等效阶梯高度，不能偷换成后端最低值。
    assert [segment.height_to_ground for segment in profile.segments] == pytest.approx(
        [2.595, 2.4, 2.2]
    )
    assert len(profile.segments) == 3
    assert sum(segment.length_to_prev for segment in profile.segments) == 12.0


def test_ascending_slope_range_includes_observed_rear_maximum():
    samples_s = np.linspace(0.1, 11.9, 60)
    profile = _slope_profile(samples_s, 2.0 + samples_s * 0.05)

    assert profile.measured_height_range == pytest.approx((2.005, 2.595))
    assert [segment.height_to_ground for segment in profile.segments] == pytest.approx(
        [2.005, 2.2, 2.4]
    )


def test_nonlinear_slope_range_includes_unsampled_middle_extremum():
    samples_s = np.linspace(0.1, 11.9, 60)
    samples_h = 1.8 + 0.0125 * (samples_s - 6.1) ** 2
    profile = _slope_profile(samples_s, samples_h)

    observed_low, observed_high = profile.measured_height_range
    assert observed_low == pytest.approx(1.8)
    assert observed_high == pytest.approx(2.25)
    assert min(segment.height_to_ground for segment in profile.segments) > observed_low


def test_partial_slope_coverage_does_not_extrapolate_unobserved_rear():
    samples_s = np.linspace(1.1, 8.9, 40)
    profile = _slope_profile(samples_s, 2.6 - samples_s * 0.05)

    assert profile.measured_height_range == pytest.approx((2.155, 2.545))
    # 不向未扫描到的 s=12m 延长拟合线，更不能输出该位置的 2.0m。
    assert profile.measured_height_range[0] > 2.0


@pytest.mark.parametrize("case", [CASE_FLAT, CASE_STEP_90, CASE_STEP_RAMP])
def test_other_floor_cases_keep_existing_representative_height_semantics(case):
    profile = _slope_profile([0.1, 1.1, 2.1, 3.1], [1.5, 1.6, 1.4, 1.5])
    profile.case = case

    assert profile.measured_height_range is None


@pytest.mark.parametrize(
    "samples_s,samples_h",
    [
        (None, None),
        ([0.1, 1.1, 2.1, 3.1], None),
        (None, [1.0, 1.1, 1.2, 1.3]),
        ([], []),
        ([0.1, 1.1, 2.1], [1.0, 1.1, 1.2]),
        ([0.1, 1.1, 2.1, 3.1], [1.0, 1.1, 1.2]),
        ([[0.1, 1.1], [2.1, 3.1]], [[1.0, 1.1], [1.2, 1.3]]),
        ([0.1, 1.1, 2.1, 3.1], [1.0, np.nan, 1.2, 1.3]),
        ([0.1, 1.1, 2.1, 3.1], [1.0, np.inf, 1.2, 1.3]),
        ([0.1, np.nan, 2.1, 3.1], [1.0, 1.1, 1.2, 1.3]),
        ([3.1, 2.1, 1.1, 0.1], [1.0, 1.1, 1.2, 1.3]),
        ([0.1, 1.1, 1.1, 3.1], [1.0, 1.1, 1.2, 1.3]),
    ],
)
def test_missing_or_invalid_profile_keeps_legacy_fallback(samples_s, samples_h):
    profile = FloorProfile(
        case=CASE_FULL_SLOPE,
        ground_plane=None,
        ground_plane_source="weighbridge_calibration",
        profile_s=samples_s,
        profile_h=samples_h,
    )

    assert profile.measured_height_range is None


def test_height_range_does_not_change_existing_floor_json_contract():
    profile = _slope_profile([0.1, 4.1, 8.1, 11.9], [2.6, 2.4, 2.2, 2.0])
    payload = profile.to_dict()

    assert set(payload) == {
        "case",
        "caseName",
        "heightReference",
        "referencePlane",
        "stepAngleDeg",
        "segments",
        "confidence",
        "warnings",
    }
    assert len(payload["segments"]) == 3
