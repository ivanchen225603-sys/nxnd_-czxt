"""场景净化的两条纯净度守卫：磅坑深度判据与稳健底板找回足迹。

回归背景来自 0907 双雷达实测：场景里同时存在两层地面 —— 地磅台面（标定面 Z=0）
与低约 0.75m 的周边地坪 —— 以及一个 13.3m 长、低至 -6.5m 的磅坑/地沟结构。
该结构的 1~99 分位尺寸落在货车先验内，通过了尺寸门禁成为车辆候选，仅因点数
只有主车辆的 6.1% 才没被选成目标车辆。
"""

import numpy as np
import pytest

from carpointcloud_analyzer import Config
from carpointcloud_analyzer.application.dual_cloud_merge import _vehicle_candidate_report
from carpointcloud_analyzer.domain.preprocessing import (
    PreprocessParams,
    ScenePreprocessor,
    _trim_terminal_low_surface,
    filter_scene,
)


def _slab(rng, n, x0, x1, y0, y1, z0, z1):
    """均匀填充一个长方体。"""
    return np.column_stack([
        rng.uniform(x0, x1, n), rng.uniform(y0, y1, n), rng.uniform(z0, z1, n),
    ])


def _scene_with_pit(rng, bulk_low=-4.9):
    """磅台面 + 一台货车 + 一个磅坑结构。坐标即场地系，Z=0 是标定面。

    磅坑按 0907 实测那个结构的统计特征构造：1~99 分位尺寸 11.4 × 1.94 × 5.40
    **全部落在货车先验内**，Z 的 10% 分位却在 -3.18m。只有深度判据能区分它；
    尺寸先验用 1~99 分位，看不出 7.39m 的真实竖直跨度。
    """
    ground = _slab(rng, 40000, -25, 25, -9, 9, -0.02, 0.02)
    truck = np.vstack([
        _slab(rng, 9000, -6, 6, -1.25, -1.15, 0.9, 3.4),      # 左侧壁
        _slab(rng, 9000, -6, 6, 1.15, 1.25, 0.9, 3.4),        # 右侧壁
        _slab(rng, 6000, -6.1, -5.9, -1.25, 1.25, 0.9, 3.4),  # 后端壁
        _slab(rng, 6000, 5.9, 6.1, -1.25, 1.25, 0.9, 3.4),    # 前端壁
    ])
    pit = np.vstack([
        # 主体：竖向跨度 5.4m 落在高度先验 (0.5, 5.5) 之内
        _slab(rng, 9000, 12, 14.8, -5.7, 5.7, bulk_low, 0.5),
        # 上下细尾，把真实 ptp 拉到 7.4m 但不改变 1~99 分位
        _slab(rng, 150, 12, 14.8, -5.7, 5.7, -6.5, bulk_low),
        _slab(rng, 150, 12, 14.8, -5.7, 5.7, 0.5, 0.9),
    ])
    return np.vstack([ground, truck, pit]), len(truck)


def _params(**overrides) -> PreprocessParams:
    """生产口径参数，可覆盖单项。"""
    from dataclasses import replace
    return replace(ScenePreprocessor(Config()).params(), **overrides)


def test_two_vehicle_report_marks_only_selected_candidate():
    """双车同框诊断必须区分看见的车辆与真正进入融合的车辆。"""
    rng = np.random.default_rng(24)
    on_scale = _slab(rng, 300, 2, 17, -1.2, 1.2, 1, 3)
    outside = _slab(rng, 100, -15, -1, -1.2, 1.2, 1, 3)

    report = _vehicle_candidate_report([on_scale, outside], primary_index=0)

    assert [item["selectedForFusion"] for item in report] == [True, False]
    assert [item["pointCount"] for item in report] == [300, 100]


def test_pit_structure_is_not_a_vehicle_candidate_when_datum_is_trusted():
    """场地系下，显著低于地磅标定面的结构不得成为车辆候选。"""
    rng = np.random.default_rng(11)
    scene, _ = _scene_with_pit(rng)

    result = filter_scene(scene, _params(z_is_datum_height=True))

    assert len(result.vehicle_points_list) == 1, "深坑不应成为第二个车辆候选"
    picked = result.primary_vehicle_points
    assert float(np.percentile(picked[:, 2], 10)) > -1.0
    assert picked[:, 0].max() < 11.0, "深坑所在纵向区间不应进入车辆点"


def test_depth_guard_is_off_when_z_is_not_a_trusted_height():
    """单文件路径的 Z 只是输入轴向，不能据此判深度，守卫必须默认关闭。"""
    assert _params().z_is_datum_height is False


def test_vehicle_partly_on_lower_pavement_is_kept():
    """车头停在比磅台面低 0.75m 的地坪上时，整车必须保留。

    守卫的容差必须显著大于台面与地坪的落差，否则长车会被整段砍掉。
    """
    rng = np.random.default_rng(12)
    ground = np.vstack([
        _slab(rng, 25000, -25, 6, -9, 9, -0.02, 0.02),        # 磅台面
        _slab(rng, 20000, 6, 25, -9, 9, -0.77, -0.73),        # 低 0.75m 的地坪
    ])
    truck = np.vstack([
        _slab(rng, 9000, -6, 6, -1.25, -1.15, 0.9, 3.4),
        _slab(rng, 9000, -6, 6, 1.15, 1.25, 0.9, 3.4),
        _slab(rng, 6000, -6.1, -5.9, -1.25, 1.25, 0.9, 3.4),
        # 车头骑在低 0.75m 的地坪上：底部结构落到标定面以下
        _slab(rng, 7000, 6.2, 9.0, -1.2, 1.2, -0.60, 1.45),
    ])
    result = filter_scene(np.vstack([ground, truck]), _params(z_is_datum_height=True))

    picked = result.primary_vehicle_points
    assert picked[:, 0].max() > 8.0, "地坪上的车头段被误删"
    assert float(np.percentile(picked[:, 2], 10)) < 1.5


@pytest.mark.parametrize("angle", [0.0, np.pi / 3])
@pytest.mark.parametrize("reverse_tail", [False, True])
def test_terminal_road_surface_is_not_kept_as_vehicle(angle, reverse_tail):
    """任一端及任意停车方向下，低位宽薄坡面不能将车厢延长为假长车。"""
    road_x, road_y = np.meshgrid(np.arange(-7., 3., .10),
                                 np.arange(-1.6, 1.61, .10), indexing="ij")
    road = np.c_[road_x.ravel(), road_y.ravel(),
                 -0.5 + 0.05 * (road_x.ravel() + 7.)]
    body_x, body_y = np.meshgrid(np.arange(3., 18., .15),
                                 np.arange(-1.2, 1.21, .15), indexing="ij")
    body = np.c_[body_x.ravel(), body_y.ravel(),
                 np.full(body_x.size, 2.2)]
    # 路旁窄立柱的高点不能伪装成有整车宽度的连续车身。
    pole = _slab(np.random.default_rng(9), 300, -2.5, -2.0, 1.2, 1.4, 0., 2.5)
    scene = np.vstack((road, body, pole))
    if reverse_tail:
        scene[:, 0] *= -1
    rotation = np.array([[np.cos(angle), -np.sin(angle)],
                         [np.sin(angle), np.cos(angle)]])
    scene[:, :2] = scene[:, :2] @ rotation
    trimmed, removed = _trim_terminal_low_surface(scene, _params(z_is_datum_height=True))
    assert removed >= len(road) * 0.95
    assert len(trimmed) >= len(body)
    assert np.count_nonzero(trimmed[:, 2] == 2.2) == len(body)


def test_terminal_guard_keeps_low_truck_deck_and_requires_calibrated_height():
    """仅近地薄面可剔；0.8m 的真实低底板与未标定单云均保留。"""
    rng = np.random.default_rng(15)
    low_deck = _slab(rng, 5000, -8, 0, -1.2, 1.2, .78, .82)
    body = _slab(rng, 10000, 0, 12, -1.2, 1.2, 1.2, 2.8)
    points = np.vstack((low_deck, body))
    kept, removed = _trim_terminal_low_surface(points, _params(z_is_datum_height=True))
    assert removed == 0
    np.testing.assert_array_equal(kept, points)
    kept, removed = _trim_terminal_low_surface(points, _params(z_is_datum_height=False))
    assert removed == 0
    np.testing.assert_array_equal(kept, points)


def test_floor_restore_footprint_uses_robust_bounds():
    """底板找回的足迹用分位数而非极值，避免少量车外污染点撑大足迹。

    回归背景：极值足迹会形成正反馈 —— 先前步骤带进的路缘/护栏点把足迹撑大、
    把下沿压低，底板找回据此又补进更多车外地面点。实测一次补进 1112 点，
    其中 1005 点低于地磅标定面，而该车厢底板离台面 2.2m。
    """
    from carpointcloud_analyzer.domain.preprocessing import _restore_carriage_floor

    rng = np.random.default_rng(13)
    # 车体：宽 2.4m，底板在 +0.20m
    body = _slab(rng, 8000, -5, 5, -1.2, 1.2, 0.20, 2.6)
    # 一小撮车外污染点，把原始极值足迹横向撑到 ±2.2m
    contamination = _slab(rng, 60, -5, 5, 2.0, 2.2, -0.5, -0.3)
    vehicle = [np.vstack([body, contamination])]
    # 被剔除的地面：车体足迹内是底板高度，足迹外是真实地面
    removed = np.vstack([
        _slab(rng, 1500, -4, 4, -1.0, 1.0, 0.18, 0.22),       # 真底板
        _slab(rng, 1500, -4, 4, 1.5, 2.1, -0.55, -0.45),      # 车外地面
    ])

    n = _restore_carriage_floor(vehicle, removed, _params())

    restored = vehicle[0][len(body) + len(contamination):]
    assert n > 0, "真底板应当被找回"
    assert float(restored[:, 2].min()) > 0.0, "车外地面不得被当作底板补回"
    assert abs(float(np.median(restored[:, 1]))) < 1.2


def test_scene_preprocessing_fails_closed_by_default():
    """默认不得把提取失败的场景当作车辆点云返回。"""
    assert Config().scene_preprocessing_fail_open is False


@pytest.mark.parametrize("bulk_low", [-2.5, -4.0, -4.9])
def test_deeper_pit_is_always_rejected(bulk_low):
    """坑越深越该拒；容差之上不应出现"深到反而通过"的反常。"""
    rng = np.random.default_rng(14)
    scene, _ = _scene_with_pit(rng, bulk_low=bulk_low)

    result = filter_scene(scene, _params(z_is_datum_height=True))

    assert len(result.vehicle_points_list) == 1
