"""情况 4（整体坡度）的真车回归：实测底板剖面驱动。

数据来源：2026-09-10 16:44 现场雷达 1256 采集的空车（用户确认空车）。原始点云
经地面平面调平后，由 ``FloorProfileAnalyzer._extract_profile`` 逐 0.2m 分箱提取出
下面这 60 个 (s, H) 实测对；s 为距车厢前壁的距离，H 为到地面的法向距离。

与 ``test_floor_profile.py`` 里的合成用例互补，覆盖两个真实世界特征：

* **坡度很缓**：全长 11.85m 只降 0.118m，约 10 mm/m。合成用例用的是 50 mm/m
  的匀坡，缓 5 倍的真实坡度会不会被 ``floor_flat_tolerance`` 误判成平板，
  只有真车数据能回答。
* **前平后斜**：前 4m 几乎水平（降 4mm），后 7.85m 才开始降（降 97mm）。
  合成用例都是匀坡或对称 U 形，没有这种形态；它的落差区间宽达 7.9m，
  必须落到情况 4 而不是情况 3 的坡道。

本用例固定的是判别结果与分段形状。**H1/L1 的绝对准确性仍未验证** —— 规格
``docs/floor_profile_spec.md`` §8 要求配套卷尺实测高差，目前没有，所以这里
只断言与实测剖面自洽，不断言与车辆真值一致。
"""

import numpy as np
import pytest

from carpointcloud_analyzer import Config
from carpointcloud_analyzer.domain.floor import CASE_FULL_SLOPE, FloorProfileAnalyzer

# 实测 (距前壁 s, 到地面高度 H)，单位米。见模块 docstring 的来源说明。
MEASURED_PROFILE = [
    (0.1, 2.2584), (0.3, 2.2522), (0.5, 2.2571), (0.7, 2.2642), (0.9, 2.2655),
    (1.1, 2.2618), (1.3, 2.2647), (1.5, 2.2609), (1.7, 2.2574), (1.9, 2.2594),
    (2.1, 2.2601), (2.3, 2.2733), (2.5, 2.2755), (2.7, 2.2643), (2.9, 2.2600),
    (3.1, 2.2586), (3.3, 2.2552), (3.5, 2.2591), (3.7, 2.2584), (3.9, 2.2532),
    (4.1, 2.2536), (4.3, 2.2496), (4.5, 2.2461), (4.7, 2.2499), (4.9, 2.2440),
    (5.1, 2.2369), (5.3, 2.2396), (5.5, 2.2371), (5.7, 2.2290), (5.9, 2.2239),
    (6.1, 2.2175), (6.3, 2.2175), (6.5, 2.2170), (6.7, 2.2168), (6.9, 2.2131),
    (7.1, 2.2132), (7.3, 2.2033), (7.5, 2.1955), (7.7, 2.1969), (7.9, 2.1970),
    (8.1, 2.1938), (8.3, 2.1951), (8.5, 2.1887), (8.7, 2.1878), (8.9, 2.1918),
    (9.1, 2.1870), (9.3, 2.1843), (9.5, 2.1828), (9.7, 2.1770), (9.9, 2.1777),
    (10.1, 2.1842), (10.3, 2.1780), (10.5, 2.1720), (10.7, 2.1698), (10.9, 2.1647),
    (11.1, 2.1623), (11.3, 2.1619), (11.5, 2.1586), (11.7, 2.1612), (11.9, 2.1571),
]

FRONT_L = 7.95          # 车厢前壁内表面（靠车头，L 大的一端）
REAR_L = -3.90          # 车厢后壁内表面
CARRIAGE_LENGTH = FRONT_L - REAR_L
INTERIOR_W = (-1.35, 1.02)
GROUND_H = 0.0          # 已按地面调平，地面即 H=0

_S = np.array([s for s, _ in MEASURED_PROFILE])
_H = np.array([h for _, h in MEASURED_PROFILE])


def _measured_height(s: float) -> float:
    """实测剖面在任意 s 处的高度（线性插值，越界取端点）。"""
    return float(np.interp(s, _S, _H))


def _build_cloud(seed: int = 0) -> np.ndarray:
    """按实测剖面合成车厢点云，另加两侧壁以检验内部裁剪生效。

    坐标约定与 geometry.py 一致：axis0=W 车宽，axis1=L 车长，axis2=H 高度。
    叠加 ±5mm 测距噪声，与合成用例同口径，避免只在无噪数据上成立。
    """
    rng = np.random.default_rng(seed)
    rows = []
    for s in np.arange(0.0, CARRIAGE_LENGTH, 0.1):
        local_l = FRONT_L - s
        z = _measured_height(s)
        width = rng.uniform(INTERIOR_W[0] + 0.05, INTERIOR_W[1] - 0.05, 90)
        rows.append(np.column_stack([
            width, np.full(90, local_l), z + rng.normal(0.0, 0.005, 90),
        ]))
        for wall_w in (INTERIOR_W[0] - 0.2, INTERIOR_W[1] + 0.2):
            rows.append(np.column_stack([
                np.full(20, wall_w), np.full(20, local_l),
                rng.uniform(z, z + 1.4, 20),
            ]))
    return np.vstack(rows)


def _analyze(points: np.ndarray):
    """按生产口径调用底板剖面分析。"""
    return FloorProfileAnalyzer(Config()).analyze(
        points, INTERIOR_W, FRONT_L, REAR_L,
        ground_plane_local=np.array([0.0, 0.0, 1.0, -GROUND_H]),
    )


def test_real_gentle_slope_is_not_mislabeled_flat():
    """10 mm/m 的真实缓坡必须判成整体坡度，不能因为坡缓而落回平板。

    回归背景：``floor_flat_tolerance`` 是 0.05m，而本车全长落差 0.118m，
    只有 2.4 倍余量。若今后放宽该容差或改动趋势判据，真实缓坡会被吞掉，
    高低板三段数据随之消失。
    """
    profile = _analyze(_build_cloud())

    assert profile is not None
    assert profile.case == CASE_FULL_SLOPE
    assert profile.case_name == "full_slope"
    assert profile.step_angle_deg is None       # 整体坡度没有落差角


def test_real_slope_yields_three_segments_covering_full_length():
    """情况 4 必须输出三段，且按长度等分、段长之和等于车厢内长。"""
    profile = _analyze(_build_cloud())
    segments = profile.segments

    assert len(segments) == 3
    assert [s.point_type for s in segments] == ["slope_vertex"] * 3
    lengths = [s.length_to_prev for s in segments]
    assert lengths == pytest.approx([CARRIAGE_LENGTH / 3.0] * 3, abs=0.01)
    assert sum(lengths) == pytest.approx(CARRIAGE_LENGTH, abs=0.01)
    assert not any(s.height_extrapolated for s in segments)


def test_real_slope_segment_heights_track_the_measured_profile():
    """每段的 H 取该段前边界处的实测高度，且整体单调递降。

    三段前边界分别是 s=0 / L/3 / 2L/3。这里只断言与实测剖面自洽；
    绝对准确性需要卷尺实测，见模块 docstring。
    """
    profile = _analyze(_build_cloud())
    heights = [s.height_to_ground for s in profile.segments]
    bounds = [CARRIAGE_LENGTH * i / 3.0 for i in range(3)]

    assert heights == pytest.approx([_measured_height(b) for b in bounds], abs=0.01)
    assert heights[0] >= heights[1] >= heights[2]
    # 前段近水平、后段才下降：前两段落差远小于后两段。这条形态断言防止
    # 今后改成"按等高差分段"时静默改变业务口径。
    assert heights[0] - heights[1] < 0.02
    assert heights[1] - heights[2] > 0.03


def test_real_slope_height_range_matches_measurement():
    """剖面高度区间应覆盖实测的 2.157~2.276，不能漏掉 P3 之后仍在下降的坡面。"""
    profile = _analyze(_build_cloud())

    low, high = profile.measured_height_range
    assert low == pytest.approx(_H.min(), abs=0.01)
    assert high == pytest.approx(_H.max(), abs=0.01)
    assert high - low == pytest.approx(_H.max() - _H.min(), abs=0.01)


def test_real_slope_is_not_mistaken_for_a_ramp_step():
    """落差区间宽达 7.9m，远超 step_span_max_ratio，必须落到情况 4 而非情况 3。

    本车前 4m 近水平、后 7.85m 下降，形似一条很长的坡道。若把
    ``floor_step_span_max_ratio`` 放宽到覆盖 0.67 倍车长，它会被误判成
    坡道台阶，第 1 段长度将由落差位置而非等分决定，业务口径随之改变。
    """
    config = Config()
    profile = _analyze(_build_cloud())

    assert profile.case == CASE_FULL_SLOPE
    drop_span_ratio = 7.9 / CARRIAGE_LENGTH
    assert drop_span_ratio > config.floor_step_span_max_ratio
