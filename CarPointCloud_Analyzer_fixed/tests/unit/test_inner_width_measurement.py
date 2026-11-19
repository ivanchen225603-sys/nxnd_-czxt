"""净宽只由同位置双侧实测面支持，不由外包络或补点决定。"""

import numpy as np
import pytest

from carpointcloud_analyzer.domain.side_wall_measurement import (
    _supported_side_faces,
    measure_inner_width,
)


def _walls(*, width=2.4, yaw=0.0, varying=False):
    length, height = np.meshgrid(np.arange(0, 10.001, 0.05),
                                np.arange(1.5, 3.501, 0.05), indexing="ij")
    length, height = length.ravel(), height.ravel()
    half = np.full(len(length), width / 2 * np.sqrt(1 + yaw * yaw))
    if varying:
        half += 0.02 * (length - 5)
    rng = np.random.default_rng(6002)
    return np.vstack([
        np.column_stack((yaw * (length - 5) + sign * half
                         + rng.normal(0, 0.002, len(length)), length, height))
        for sign in (-1, 1)
    ])


def _measure(points):
    return measure_inner_width(points, 0, 10, 1.5, 3.5)


def test_observed_pair_measures_inner_faces_not_outer_frame():
    inner = _walls()
    # 外层也有完整二维支持，不能把它和内层平均，或把整个外包络作为净宽。
    outside = _walls(width=2.7)
    result = _measure(np.vstack([inner, outside]))
    assert result.width_m == pytest.approx(2.4, abs=0.006)
    assert result.metadata["valid"] is True
    assert result.metadata["source"] == "measurement_input_carriage_cloud"
    assert result.metadata["fullHeightMinimumClearance"] is False


def test_missing_intervals_only_use_common_observed_windows():
    points = _walls()
    # 右侧只有约半段，另一半不能镜像后冒充实测；已有共同段仍可量取。
    points = points[(points[:, 0] < 0) | ((points[:, 1] > 2) & (points[:, 1] < 8))]
    result = _measure(points)
    assert result.width_m == pytest.approx(2.4, abs=0.006)
    assert 0.3 <= result.metadata["longitudinalCoverage"] < 1
    assert result.metadata["pairedWindowCount"] < result.metadata["candidateWindowCount"]
    assert result.metadata["observedLongitudinalRangeM"][0] > 2
    assert result.metadata["observedLongitudinalRangeM"][1] < 8


def test_sparse_rectangular_width_requires_independent_shape_evidence_and_consistency():
    points = _walls()
    sparse = points[(points[:, 0] < 0) | ((points[:, 1] > 2) & (points[:, 1] < 4.2))]
    assert _measure(sparse).width_m is None
    inferred = measure_inner_width(sparse, 0, 10, 1.5, 3.5,
                                   allow_sparse_rectangular=True)
    assert inferred.width_m == pytest.approx(2.4, abs=.006)
    assert inferred.metadata["method"] == "sparse_rectangular_paired_faces"
    assert inferred.metadata["longitudinalCoverage"] < 0.3
    one_side = sparse[sparse[:, 0] < 0]
    assert measure_inner_width(one_side, 0, 10, 1.5, 3.5,
                               allow_sparse_rectangular=True).width_m is None
    tapered = _walls(varying=True)
    tapered = tapered[(tapered[:, 0] < 0)
                      | ((tapered[:, 1] > 2) & (tapered[:, 1] < 4.2))]
    assert measure_inner_width(tapered, 0, 10, 1.5, 3.5,
                               allow_sparse_rectangular=True).width_m is None


def test_one_missing_side_cannot_create_a_measured_width():
    points = _walls()
    result = _measure(points[points[:, 0] < 0])
    assert result.width_m is None
    assert result.metadata["valid"] is False


def test_exact_quantized_side_planes_do_not_overrun_histogram():
    points = _walls()
    points[:, 0] = np.where(points[:, 0] < 0, -1.2, 1.2)
    result = _measure(points)
    assert result.width_m == pytest.approx(2.4, abs=1e-12)


def test_diagonal_line_cannot_fake_two_dimensional_wall_support():
    length = np.linspace(-0.49, 0.49, 2000)
    points = np.column_stack((np.full(len(length), -1.2), length, 0.55 * length))
    assert _supported_side_faces(points, np.array([0.0, 0.0])) == []


def test_isolated_remote_outlier_cannot_expand_histogram_memory():
    points = np.vstack([_walls(), [[1e8, 5, 2.5], [-1e8, 5, 2.5]]])
    assert _measure(points).width_m == pytest.approx(2.4, abs=0.006)


def test_only_a_top_beam_does_not_supply_a_side_face():
    points = _walls()
    points = points[(points[:, 0] < 0) | (points[:, 2] > 3.3)]
    assert _measure(points).width_m is None


def test_diagonal_ribs_and_isolated_points_do_not_replace_walls():
    points = _walls()
    t = np.linspace(-1, 1, 600)
    ribs = np.vstack([
        np.column_stack((1.2 * t, np.full(len(t), length), 2.5 + 0.8 * t))
        for length in (2, 4, 6, 8)
    ])
    rng = np.random.default_rng(16)
    noise = rng.uniform([-1.6, 0, 1.5], [1.6, 10, 3.5], (800, 3))
    result = _measure(np.vstack([points, ribs, noise]))
    assert result.width_m == pytest.approx(2.4, abs=0.01)
    # 单侧壁加斜筋不能生成另一侧的二维表面。
    incomplete = np.vstack([points[points[:, 0] < 0], ribs])
    assert _measure(incomplete).width_m is None


def test_uniform_volume_has_no_surface_density_peak():
    rng = np.random.default_rng(81)
    points = rng.uniform([-1.3, 0, 1.5], [1.3, 10, 3.5], (60000, 3))
    assert _measure(points).width_m is None


def test_small_yaw_uses_perpendicular_span():
    result = _measure(_walls(yaw=0.1))
    assert result.width_m == pytest.approx(2.4, abs=0.008)


def test_varying_width_reports_median_and_records_narrowest_section():
    # 宽度沿车长 2.2～2.6m 线性变化；中位约 2.4m，最靠近端部的窗口（L=1m）约 2.24m 另行记录。
    result = _measure(_walls(varying=True))
    assert result.width_m == pytest.approx(2.4, abs=0.015)
    assert result.metadata["narrowestWindow"]["widthM"] == pytest.approx(2.4 + .04 * (1 - 5), abs=0.01)
    low, high = result.metadata["widthRangeP10P90M"]
    assert low < 2.3
    assert high > 2.5
    assert high - low > 0.2
    assert result.metadata["definition"] == "median_paired_side_rim_spacing"


def test_point_order_density_and_translation_do_not_define_width():
    points = _walls()
    expected = _measure(points)
    rng = np.random.default_rng(4)
    shuffled = _measure(points[rng.permutation(len(points))])
    assert shuffled.width_m == pytest.approx(expected.width_m, abs=1e-12)
    dense_patch = points[(points[:, 1] < 3) & (points[:, 0] < 0)]
    dense = _measure(np.vstack([points] + [dense_patch] * 7))
    assert dense.width_m == pytest.approx(expected.width_m, abs=0.002)
    translated = measure_inner_width(points + [8, 21, 3], 21, 31, 4.5, 6.5)
    assert translated.width_m == pytest.approx(expected.width_m, abs=0.002)


@pytest.mark.parametrize("bounds", [
    (0, 0, 1.5, 3.5), (10, 0, 1.5, 3.5), (0, 10, 3.5, 1.5),
    (0, np.inf, 1.5, 3.5), (0, 10, np.nan, 3.5),
])
def test_invalid_bounds_return_explicit_reason(bounds):
    result = measure_inner_width(_walls(), *bounds)
    assert result.width_m is None
    assert result.metadata["reason"] in {"invalid_carriage_bounds", "non_finite_input"}


@pytest.mark.parametrize("points", [np.empty((0, 3)), np.empty((20, 2)),
                                     np.full((100, 3), np.nan)])
def test_invalid_points_do_not_raise_or_invent_width(points):
    result = _measure(points)
    assert result.width_m is None
    assert result.metadata["valid"] is False


def test_corrugated_rims_report_same_position_median_width():
    points = _walls()
    # 两侧墙各有 3cm 深、周期 0.2m 的竖向波纹，波峰朝厢内。
    points[:, 0] -= np.sign(points[:, 0]) * .03 * (1 + np.sin(2 * np.pi * points[:, 1] / .2)) / 2
    result = _measure(points)
    # 上沿同位置宽度在2.34～2.40m之间变化，中位值2.37m；沿用平墙6mm容差。
    assert result.width_m == pytest.approx(2.37, abs=.006)


def test_inner_liner_on_one_side_wall_is_the_wall():
    # 整面内衬板在左侧墙内 5cm；雷达看不到衬板后的原墙。净宽量到内衬表面。
    points = _walls()
    points[points[:, 0] < 0, 0] += .05
    result = _measure(points)
    assert result.width_m == pytest.approx(2.35, abs=.006)


def test_single_stray_point_inside_a_window_is_not_a_wall():
    clean = _measure(_walls())
    noisy = _measure(np.vstack((_walls(), [[-1.17, 5., 2.5]])))
    assert noisy.width_m == pytest.approx(clean.width_m, abs=1e-9)


def test_lower_liner_does_not_change_side_rim_width():
    points = _walls()
    points[(points[:, 0] < 0) & (points[:, 2] < 2.5), 0] += .10
    result = _measure(points)
    assert result.width_m == pytest.approx(2.4, abs=.006)


def test_outward_sloping_sides_are_measured_at_upper_edges():
    points = _walls()
    points[:, 0] += np.sign(points[:, 0]) * .05 * (points[:, 2] - 1.5)
    result = _measure(points)
    assert result.width_m == pytest.approx(2.6, abs=.006)


def test_top_end_canopy_extensions_do_not_define_rim_width():
    points = _walls()
    extra = points[(points[:, 1] < .3) | (points[:, 1] > 9.7)].copy()
    extra[:, 2] += 1.
    extra[:, 0] *= 1.2
    result = _measure(np.vstack((points, extra)))
    assert result.width_m == pytest.approx(2.4, abs=.006)
