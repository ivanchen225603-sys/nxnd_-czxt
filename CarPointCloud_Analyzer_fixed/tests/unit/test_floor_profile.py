"""底板剖面四种情况的判别与取值测试。

真车数据目前只有底板平整的空车（情况 1），情况 2/3/4 用合成点云覆盖。
合成时刻意加入 ±5mm 的测距噪声，避免算法只在无噪理想数据上成立。

坐标约定与 geometry.py 一致：axis0=W 车宽，axis1=L 车长，axis2=H 高度。
车头（车厢前壁）在 L 大的一端。
"""

import numpy as np
import pytest

from carpointcloud_analyzer import Config
from carpointcloud_analyzer.domain.floor import (
    CASE_FLAT,
    CASE_FULL_SLOPE,
    CASE_STEP_90,
    CASE_STEP_RAMP,
    FloorProfileAnalyzer,
)

FRONT_L = 6.0          # 车厢前壁（靠车头）
REAR_L = -6.0          # 车厢后壁
CARRIAGE_LENGTH = 12.0
INTERIOR_W = (-1.0, 1.0)
GROUND_H = -2.0        # 地面在局部 H 轴上的位置


def _plane_flat_ground():
    """局部坐标下的水平地平面，法向朝上：z - GROUND_H = 0。"""
    return np.array([0.0, 0.0, 1.0, -GROUND_H])


def _build(height_at_s, n_per_bin=90, seed=0):
    """按 s→底板高度 的函数合成车厢点云。

    s 为距前壁的距离（0 → 车厢长），返回的 H 是局部坐标的绝对高度。
    额外加两侧壁，检验内部区域裁剪确实生效。
    """
    rng = np.random.default_rng(seed)
    rows = []
    for s in np.arange(0.0, CARRIAGE_LENGTH, 0.1):
        local_l = FRONT_L - s
        z = height_at_s(s)
        w = rng.uniform(INTERIOR_W[0] + 0.05, INTERIOR_W[1] - 0.05, n_per_bin)
        rows.append(np.column_stack([
            w,
            np.full(n_per_bin, local_l),
            z + rng.normal(0.0, 0.005, n_per_bin),
        ]))
        # 两侧壁：高度远高于底板，必须被内部区域裁剪掉
        for wall_w in (INTERIOR_W[0] - 0.2, INTERIOR_W[1] + 0.2):
            rows.append(np.column_stack([
                np.full(20, wall_w),
                np.full(20, local_l),
                rng.uniform(z, z + 1.5, 20),
            ]))
    return np.vstack(rows)


def _analyze(points, cfg=None):
    cfg = cfg or Config()
    return FloorProfileAnalyzer(cfg).analyze(
        points, INTERIOR_W, FRONT_L, REAR_L, _plane_flat_ground()
    )


@pytest.mark.parametrize('height_at_s,front_cm', [
    (lambda s: .5 - .05 * s, 250.),
    (lambda s: .05 * s, 200.),
    (lambda s: .5 if s < 4. else 0., 250.),
    (lambda s: 0. if s < 4. else .5, 200.),
])
def test_bottom_height_is_front_floor_not_minimum_or_step_high_point(height_at_s, front_cm):
    from carpointcloud_analyzer.application.exporters import ResultExporter

    profile = _analyze(_build(height_at_s))
    profile.ground_plane_source = 'weighbridge_calibration'
    before = profile.to_dict()
    values, _ = ResultExporter._build_high_low_plate_payload(profile)
    # 沿用本文件底板测高的2cm精度，下降坡/下降台阶不能再取后部低值；
    # 上升台阶的P1是后方高板，也不能拿P1冒充前端底板。
    assert values['trailerBottomToGroundHeight'] == pytest.approx(front_cm, abs=2.)
    assert profile.to_dict() == before


def test_front_floor_keeps_existing_profile_endpoint_reading_when_front_is_occluded():
    from carpointcloud_analyzer.application.exporters import ResultExporter

    points = _build(lambda s: .5 - .05 * s)
    profile = _analyze(points[points[:, 1] <= FRONT_L - 1.])
    profile.ground_plane_source = 'weighbridge_calibration'
    values, _ = ResultExporter._build_high_low_plate_payload(profile)
    # 仍用既有s=0读数规则：前部有缺口时取最靠前的有效分箱，不新增缺测门禁。
    assert profile.profile_s[0] > 1.
    assert values['trailerBottomToGroundHeight'] == round(profile.profile_h[0] * 100., 1)


@pytest.mark.parametrize('shape,case', [
    (lambda s: 0. if s < 4. else .5, CASE_STEP_90),
    (lambda s: .5 * np.clip((s - 3.) / 2., 0., 1.), CASE_STEP_RAMP),
])
def test_ascending_platforms_keep_the_low_side_in_the_exported_drop(shape, case):
    from carpointcloud_analyzer.application.exporters import ResultExporter

    profile = _analyze(_build(shape))
    profile.ground_plane_source = 'weighbridge_calibration'
    values, plates = ResultExporter._build_high_low_plate_payload(profile)

    assert profile.case == case
    assert values['trailerBottomToGroundHeight'] == pytest.approx(200., abs=2.)
    assert values['highLowPlateHeight'] == pytest.approx(50., abs=2.)
    # 图中的三个点位都在高侧，业务高差仍必须反映前低后高。
    assert [p['lowestToGround'] for p in plates] == pytest.approx([250.] * 3, abs=2.)


def test_flat_floor_reports_single_segment():
    profile = _analyze(_build(lambda s: 0.0))
    assert profile.case == CASE_FLAT
    assert profile.case_name == "flat"
    assert len(profile.segments) == 1
    assert profile.segments[0].point_type == "flat"
    # 底板在 z=0，地面在 z=-2 → 离地 2.0m
    assert profile.segments[0].height_to_ground == pytest.approx(2.0, abs=0.02)
    assert profile.segments[0].length_to_prev == pytest.approx(CARRIAGE_LENGTH, abs=0.01)
    assert profile.measured_height_range is None
    payload = profile.to_dict()
    assert payload["heightReference"]["type"] == "scene_ground"
    assert payload["segments"][0]["heightToReference"] == pytest.approx(2.0)
    assert "heightToGround" not in payload["segments"][0]


def test_vertical_step_splits_into_three_segments():
    """前 4m 抬高 0.5m，之后垂直落下。"""
    profile = _analyze(_build(lambda s: 0.5 if s < 4.0 else 0.0))
    assert profile.case == CASE_STEP_90
    assert profile.step_angle_deg >= 75.0
    assert [s.point_type for s in profile.segments] == [
        "step_top", "mid_rear", "rear_wall"
    ]
    h1, h2, h3 = [s.height_to_ground for s in profile.segments]
    l1, l2, l3 = [s.length_to_prev for s in profile.segments]
    # H1 取抬高面（0.5 + 2.0 离地），H2/H3 在下层底板（2.0）
    assert h1 == pytest.approx(2.5, abs=0.05)
    assert h2 == pytest.approx(2.0, abs=0.05)
    assert h3 == pytest.approx(2.0, abs=0.05)
    # L1 到台阶，剩余 8m 平均分成两段
    assert l1 == pytest.approx(4.0, abs=0.25)
    assert l2 == pytest.approx(l3, abs=0.01)
    assert l1 + l2 + l3 == pytest.approx(CARRIAGE_LENGTH, abs=0.01)


def test_ascending_vertical_step_p1_uses_high_plateau():
    """前段低、后段高时，P1 仍应表示台阶高点而不是低板。"""
    profile = _analyze(_build(lambda s: 0.0 if s < 4.0 else 0.5))

    assert profile.case == CASE_STEP_90
    assert profile.segments[0].point_type == "step_top"
    assert profile.segments[0].length_to_prev == pytest.approx(4.0, abs=0.25)
    assert profile.segments[0].height_to_ground == pytest.approx(2.5, abs=0.05)


def test_ramp_step_measures_to_ramp_end_at_plateau_height():
    """前 3m 抬高 0.5m，3~5m 为坡道降到 0。

    按约定：垂直边落在坡道末端，L1 量到 s=5，H1 取坡顶平台高度。
    """
    def height(s):
        if s < 3.0:
            return 0.5
        if s < 5.0:
            return 0.5 * (1.0 - (s - 3.0) / 2.0)
        return 0.0

    profile = _analyze(_build(height))
    assert profile.case == CASE_STEP_RAMP
    assert profile.step_angle_deg < 75.0
    assert profile.segments[0].point_type == "ramp_end"
    # L1 含坡道水平投影，量到坡道末端而非坡道起点
    assert profile.segments[0].length_to_prev == pytest.approx(5.0, abs=0.35)
    # H1 是坡顶平台高度，不是坡道末端的实际高度
    assert profile.segments[0].height_to_ground == pytest.approx(2.5, abs=0.06)
    assert profile.segments[2].height_to_ground == pytest.approx(2.0, abs=0.05)


def test_full_slope_builds_three_right_triangles():
    """整体从 0.6m 均匀降到 0：三段顶点取各段前边界的高度。"""
    profile = _analyze(_build(lambda s: 0.6 * (1.0 - s / CARRIAGE_LENGTH)))
    assert profile.case == CASE_FULL_SLOPE
    assert [s.point_type for s in profile.segments] == ["slope_vertex"] * 3
    l1, l2, l3 = [s.length_to_prev for s in profile.segments]
    assert l1 == pytest.approx(4.0, abs=0.01)
    assert l2 == pytest.approx(4.0, abs=0.01)
    assert l3 == pytest.approx(4.0, abs=0.01)
    h1, h2, h3 = [s.height_to_ground for s in profile.segments]
    # 顶点高度 = 各段前边界（s=0/4/8）处的坡面高度 + 离地 2.0
    assert h1 == pytest.approx(2.6, abs=0.05)
    assert h2 == pytest.approx(2.4, abs=0.05)
    assert h3 == pytest.approx(2.2, abs=0.05)
    assert h1 > h2 > h3
    # 整体最低高度另取所有底板分箱，不能遗漏 P3 后方仍然下降的坡面。
    assert profile.measured_height_range == pytest.approx((2.0, 2.6), abs=0.02)


def test_ascending_slope_uses_same_formula():
    """上坡车型（后高前低）无需镜像逻辑，统一规则方向无关。"""
    profile = _analyze(_build(lambda s: 0.6 * s / CARRIAGE_LENGTH))
    assert profile.case == CASE_FULL_SLOPE
    h1, h2, h3 = [s.height_to_ground for s in profile.segments]
    # 前边界高度递增 → 顶点高度递增，与下坡完全对称
    assert h1 == pytest.approx(2.0, abs=0.05)
    assert h2 == pytest.approx(2.2, abs=0.05)
    assert h3 == pytest.approx(2.4, abs=0.05)
    assert profile.measured_height_range == pytest.approx((2.0, 2.6), abs=0.02)


def test_broad_nonlinear_floor_is_not_mislabeled_flat():
    """宽幅 U 形底板虽非线性，也不是物理意义上的平底板。"""
    profile = _analyze(
        _build(lambda s: 0.35 * ((s - CARRIAGE_LENGTH / 2.0) / 6.0) ** 2)
    )

    assert profile.case == CASE_FULL_SLOPE
    assert profile.case_name == "full_slope"
    assert profile.measured_height_range[0] == pytest.approx(2.0, abs=0.02)


def test_step_near_rear_still_splits_remaining_evenly():
    """台阶靠后时，剩余部分仍平均分两段，三段长度之和等于车厢长。"""
    profile = _analyze(_build(lambda s: 0.5 if s < 9.0 else 0.0))
    assert profile.case == CASE_STEP_90
    l1, l2, l3 = [s.length_to_prev for s in profile.segments]
    assert l1 == pytest.approx(9.0, abs=0.3)
    assert l2 == pytest.approx(l3, abs=0.01)
    assert l1 + l2 + l3 == pytest.approx(CARRIAGE_LENGTH, abs=0.01)


def test_rear_height_extrapolation_is_explicitly_marked():
    """后壁附近底板缺测时，P3 沿用边界值但必须带外推标记。"""
    points = _build(lambda s: 0.5 if s < 4.0 else 0.0)
    sample_s = FRONT_L - points[:, 1]
    profile = _analyze(points[sample_s <= 10.0])

    assert profile.case == CASE_STEP_90
    p3 = profile.segments[2]
    assert p3.point_type == "rear_wall"
    assert p3.height_extrapolated
    assert p3.to_dict()["heightExtrapolated"] is True
    assert any("P3" in warning and "外推" in warning for warning in profile.warnings)


def test_tilted_ground_plane_is_honoured():
    """地面带 1° 倾角时，高度必须按点到平面的垂直距离算，而不是 z 差。"""
    tilt = np.deg2rad(1.0)
    normal = np.array([0.0, np.sin(tilt), np.cos(tilt)])
    plane = np.concatenate([normal, [-normal @ np.array([0.0, 0.0, GROUND_H])]])
    points = _build(lambda s: 0.0)
    profile = FloorProfileAnalyzer(Config()).analyze(
        points, INTERIOR_W, FRONT_L, REAR_L, plane
    )
    # 平底板 + 倾斜地面 → 沿车长高度线性变化，极差 ≈ 12m × tan(1°) ≈ 0.21m
    assert profile.case == CASE_FULL_SLOPE
    span = float(profile.profile_h.max() - profile.profile_h.min())
    assert span == pytest.approx(CARRIAGE_LENGTH * np.tan(tilt), rel=0.15)


def test_missing_ground_plane_falls_back_with_warning():
    profile = FloorProfileAnalyzer(Config()).analyze(
        _build(lambda s: 0.0), INTERIOR_W, FRONT_L, REAR_L, None
    )
    assert profile.ground_plane_source == "floor_fallback"
    assert profile.warnings
    # 退化基准把最低底板面归零
    assert profile.segments[0].height_to_ground == pytest.approx(0.0, abs=0.02)


def test_returns_none_when_interior_too_sparse():
    sparse = np.column_stack([
        np.zeros(10), np.linspace(REAR_L, FRONT_L, 10), np.zeros(10)
    ])
    assert _analyze(sparse) is None
