import numpy as np
import pytest

from carpointcloud_analyzer.domain.carriage import CarriageAnalyzer
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.dimensions import DimensionCalculator
from carpointcloud_analyzer.domain.geometry import (
    CoordinateTransform,
    LocalCarriageExtractor,
    LocalRibDetector,
    PointCloudGeometryNormalizer,
)
from carpointcloud_analyzer.domain.models import CarriageSplitResult
from carpointcloud_analyzer.domain.utils.planes import deterministic_plane_hypothesis


def test_plane_hypothesis_is_reproducible_and_input_order_independent():
    rng = np.random.default_rng(109)
    xy = rng.uniform(-3, 3, (1200, 2))
    ground = np.column_stack((xy, 0.01 * xy[:, 0] + rng.normal(0, 0.002, len(xy))))
    clutter = rng.uniform(-3, 3, (200, 3))
    points = np.vstack((ground, clutter))
    first, ids = deterministic_plane_hypothesis(points, 0.02, num_iterations=80)
    second, reversed_ids = deterministic_plane_hypothesis(points[::-1], 0.02, num_iterations=80)
    assert np.array_equal(first, second)
    assert set(ids.tolist()) == set((len(points) - 1 - reversed_ids).tolist())


def test_coordinate_transform_round_trip():
    angle = np.deg2rad(31)
    rotation = np.array([
        [np.cos(angle), -np.sin(angle), 0.0],
        [np.sin(angle), np.cos(angle), 0.0],
        [0.0, 0.0, 1.0],
    ])
    transform = CoordinateTransform(np.array([4.0, -2.0, 7.0]), rotation)
    source = np.array([[1.0, 2.0, 3.0], [-4.0, 5.0, 6.0]])

    assert np.allclose(transform.to_source(transform.to_local(source)), source)


def test_normalizer_finds_length_after_axis_rotation():
    rng = np.random.RandomState(7)
    local = np.column_stack([
        rng.uniform(-1.25, 1.25, 4000),
        rng.uniform(-6.0, 6.0, 4000),
        rng.choice([0.0, 1.9], 4000) + rng.normal(0, 0.005, 4000),
    ])
    angle = np.deg2rad(37)
    source_rotation = np.array([
        [np.cos(angle), -np.sin(angle), 0.0],
        [np.sin(angle), np.cos(angle), 0.0],
        [0.0, 0.0, 1.0],
    ])
    source = local @ source_rotation.T + np.array([10.0, -3.0, 2.0])
    cfg = Config(statistical_outlier_enabled=False)

    # 源坐标只绕 Z 旋转，竖直方向仍是 +Z；建系必须由外部基准约束高度轴。
    normalized, transform = PointCloudGeometryNormalizer(cfg).process(
        source, np.array([0.0, 0.0, 1.0]), "site_frame"
    )
    spans = np.percentile(normalized, 99, axis=0) - np.percentile(normalized, 1, axis=0)

    assert transform.confidence > 0.0
    assert transform.vertical_source == "site_frame"
    assert transform.vertical_alignment > 0.99
    assert spans[1] > 10.0
    assert 2.2 < spans[0] < 2.7
    assert 1.8 < spans[2] < 2.0


def test_rib_axis_model_uses_fitted_line_midpoint():
    rng = np.random.RandomState(11)
    width = np.linspace(-1.2, 1.2, 800)
    line = np.column_stack([
        width,
        2.0 + 0.04 * width,
        0.8 - 0.02 * width,
    ]) + rng.normal(0, 0.008, (800, 3))
    noise = rng.uniform([-1.2, 1.85, 0.2], [1.2, 2.15, 1.4], (200, 3))

    model = LocalRibDetector._fit_rib_axis(np.vstack((line, noise)))

    assert model is not None
    midpoint, direction, endpoints, inliers, rmse = model
    assert midpoint[1] == pytest.approx(2.0, abs=0.03)
    assert abs(direction[0]) > 0.95
    assert len(inliers) > 500
    assert rmse < 0.03


def test_rib_business_xyz_and_length_come_from_same_fitted_axis(monkeypatch):
    detector = LocalRibDetector(Config())
    points = np.array(
        [
            [-1.0, 4.0, 0.5],
            [0.8, 4.1, 1.5],
            [1.0, 4.2, 1.6],
        ]
        * 30
    )
    midpoint = np.array([0.25, 4.05, 1.2])
    endpoints = np.array([[-1.0, 4.0, 1.1], [1.0, 4.1, 1.3]])
    monkeypatch.setattr(
        detector,
        "_fit_rib_axis",
        lambda _: (
            midpoint,
            np.array([1.0, 0.05, 0.1]),
            endpoints,
            points,
            0.01,
        ),
    )

    rib = detector._build_rib(1, 4.0, points, (-1.0, 1.0))

    assert rib is not None
    expected_endpoints = np.vstack(
        [
            midpoint + ((wall_w - midpoint[0]) / 1.0) * np.array([1.0, 0.05, 0.1])
            for wall_w in (-1.0, 1.0)
        ]
    )
    expected_midpoint = expected_endpoints.mean(axis=0)
    assert (rib.x_center, rib.y_position, rib.z_center) == pytest.approx(
        expected_midpoint
    )
    assert rib.length == pytest.approx(
        np.linalg.norm(expected_endpoints[1] - expected_endpoints[0])
    )


def test_carriage_extractor_does_not_label_volume_density_as_inner_faces():
    """均匀厚体没有可定位表面，密度半高交点不能再冒充实测内壁。"""
    rng = np.random.default_rng(22)
    floor = np.column_stack([
        rng.uniform(-1.0, 1.0, 12000),
        rng.uniform(0.0, 10.0, 12000),
        rng.normal(0.0, 0.005, 12000),
    ])
    rear_wall = np.column_stack([
        rng.uniform(-1.2, 1.2, 6000),
        rng.uniform(0.0, 0.2, 6000),
        rng.uniform(0.0, 2.0, 6000),
    ])
    front_wall = np.column_stack([
        rng.uniform(-1.2, 1.2, 6000),
        rng.uniform(9.8, 10.0, 6000),
        rng.uniform(0.0, 2.0, 6000),
    ])
    cab = np.column_stack([
        rng.uniform(-0.8, 0.8, 2500),
        rng.uniform(10.5, 12.0, 2500),
        rng.uniform(0.2, 2.5, 2500),
    ])

    split = LocalCarriageExtractor(Config()).split(
        np.vstack([floor, rear_wall, front_wall, cab])
    )

    assert split.boundaries_are_inner_faces is False
    assert split.boundary_method == "wall_core_fallback"
    assert split.boundary_warning


def _volume_wall_trailer_with_cab(cab_count, floor_gap=False):
    """端壁为无表面的厚体，车头更密且与挂车之间有空隙。"""
    rng = np.random.default_rng(22)
    floor = np.column_stack([
        rng.uniform(-1.0, 1.0, 12000), rng.uniform(0.0, 10.0, 12000),
        rng.normal(0.0, 0.005, 12000),
    ])
    if floor_gap:
        floor = floor[(floor[:, 1] < 4.0) | (floor[:, 1] > 4.6)]
    walls = [np.column_stack([
        rng.uniform(-1.2, 1.2, 6000), rng.uniform(start, start + 0.2, 6000),
        rng.uniform(0.0, 2.0, 6000),
    ]) for start in (0.0, 9.8)]
    cab = [np.column_stack([
        rng.uniform(-0.8, 0.8, cab_count), rng.uniform(start, 12.0, cab_count),
        rng.uniform(0.2, 2.5, cab_count),
    ]) for start in (10.5, 11.85)]
    return np.vstack([floor, *walls, *cab])


def test_unverified_end_walls_do_not_span_gap_to_denser_cab():
    """内表面不可见时，车厢范围取车体连续的端壁对，不跨过车头空隙。"""
    split = LocalCarriageExtractor(Config()).split(_volume_wall_trailer_with_cab(8000))

    assert split.boundaries_are_inner_faces is False
    assert split.continuous_body_range is True
    assert split.carriage_start_y == pytest.approx(0.05, abs=0.05)
    assert split.split_y == pytest.approx(9.9, abs=0.05)
    assert split.carriage_points[:, 1].max() < 10.5
    assert "车头距" in split.boundary_warning


def test_unverified_end_walls_without_continuous_pair_keep_core_fallback():
    """端壁间车体本身断开时不能确认车厢范围，保持原壁芯回退且不开放测长。"""
    split = LocalCarriageExtractor(Config()).split(
        _volume_wall_trailer_with_cab(8000, floor_gap=True))

    assert split.boundaries_are_inner_faces is False
    assert split.continuous_body_range is False
    assert split.split_y == pytest.approx(11.93, abs=0.05)


def test_dimensions_report_observed_outer_width_and_visible_vehicle_height():
    """外沿宽与净宽分列，整车可见高度不混同车厢底板到壁沿高度。"""
    rng = np.random.default_rng(23)
    all_points = np.column_stack([
        rng.uniform(-1.5, 1.5, 5000),
        rng.uniform(0.0, 14.0, 5000),
        rng.uniform(0.5, 3.5, 5000),
    ])
    carriage_points = np.column_stack([
        rng.uniform(-1.3, 1.3, 3000),
        rng.uniform(0.2, 9.8, 3000),
        rng.uniform(1.0, 2.4, 3000),
    ])
    split = CarriageSplitResult(
        carriage_points=carriage_points,
        split_y=9.8,
        carriage_start_y=0.2,
        head_y=14.0,
        tail_y=0.0,
    )
    profile = CarriageAnalyzer(Config()).analyze(carriage_points)

    dimensions = DimensionCalculator(Config()).calculate(
        all_points, carriage_points, split, profile
    )

    expected_height = np.percentile(all_points[:, 2], 99) - np.percentile(
        all_points[:, 2], 1
    )
    expected_width = np.percentile(carriage_points[:, 0], 99) - np.percentile(
        carriage_points[:, 0], 1
    )
    assert dimensions.carriage_length == pytest.approx(9.6)
    assert np.isnan(dimensions.carriage_width)
    assert dimensions.carriage_outer_width == pytest.approx(expected_width)
    assert dimensions.width_measurement['adoptedSource'] == 'inner_width_unavailable'
    assert dimensions.vehicle_width >= dimensions.carriage_outer_width
    # 随机体积散点不提供连续侧壁，不能借 profile 轮廓补出内高。
    assert np.isnan(dimensions.carriage_height)
    assert dimensions.height_measurement['floorH'] == pytest.approx(profile.floor_z)
    assert dimensions.height_measurement['topH'] is None
    assert dimensions.height_measurement['reason'] == 'insufficient_supported_side_sections'
    assert dimensions.vehicle_height == pytest.approx(expected_height)
    assert dimensions.vehicle_height != pytest.approx(dimensions.carriage_height)


def test_cab_distance_uses_front_edge_at_business_ground_height():
    """全高度前突点不能替代离地 1.40m 截面的业务车头前沿。"""
    rng = np.random.default_rng(20260904)
    target_slice = np.column_stack([
        rng.uniform(-1.0, 1.0, 120),
        rng.uniform(7.90, 8.00, 120),
        rng.uniform(1.385, 1.415, 120),
    ])
    high_protrusion = np.column_stack([
        rng.uniform(-0.8, 0.8, 80),
        rng.uniform(8.08, 8.12, 80),
        rng.uniform(2.60, 2.80, 80),
    ])
    points = np.vstack([target_slice, high_protrusion])
    split = CarriageSplitResult(
        carriage_points=np.empty((0, 3)),
        split_y=3.36,
        carriage_start_y=-8.0,
        head_y=8.12,
        tail_y=-8.2,
        boundaries_are_inner_faces=True,
    )

    valid = DimensionCalculator(Config()).measure_cab_to_front_wall_at_height(
        points,
        split,
        np.array([0.0, 0.0, 1.0, 0.0]),
    )

    expected_head = np.percentile(target_slice[:, 1], 99)
    assert valid is True
    assert split.cab_measurement_valid is True
    assert split.cab_measurement_height == pytest.approx(1.4)
    assert split.cab_measurement_half_band == pytest.approx(0.02)
    assert split.cab_measurement_point_count == len(target_slice)
    assert split.cab_measurement_head_y == pytest.approx(expected_head)
    assert split.cab_to_front_wall == pytest.approx(expected_head - split.split_y)
    assert split.cab_measurement_head_y < split.head_y - 0.05


def test_cab_distance_fails_closed_without_ground_plane():
    split = CarriageSplitResult(np.empty((0, 3)), 3.0, -8.0, 8.0, -8.2)

    valid = DimensionCalculator(Config()).measure_cab_to_front_wall_at_height(
        np.ones((100, 3)), split, None
    )

    assert valid is False
    assert split.cab_measurement_valid is False
    assert split.cab_measurement_reason == "weighbridge_plane_unavailable"


def test_normalizer_fails_closed_when_points_are_too_few():
    """局部坐标是唯一路径，建系失败必须抛错而不是退回恒等变换。

    旧实现在点数不足时返回 applied=False 的恒等变换，交给固定 Z 轴兼容路径
    继续算；该路径已删除，若这里再降级，下游会拿"按输入轴向硬算"的假尺寸
    通过那些以局部 W/L/H 为前提的门禁。
    """
    cfg = Config()
    cfg.statistical_outlier_enabled = False
    tiny = np.random.default_rng(7).normal(0.0, 1.0, (cfg.pose_min_points - 1, 3))

    with pytest.raises(ValueError, match="拒绝继续测量"):
        PointCloudGeometryNormalizer(cfg).process(
            tiny, np.array([0.0, 0.0, 1.0]), "site_frame"
        )


def _trailer(rng, n_floor: int, n_wall: int) -> np.ndarray:
    """底板 + 两侧壁 + 前后端壁的合成空车厢（Z-up，底板离地 1.57m）。"""
    floor_z, wall_top, l0, l1, hw = 1.57, 3.57, -5.85, 5.85, 1.25

    def slab(n, x0, x1, y0, y1, z0, z1):
        return np.column_stack([
            rng.uniform(x0, x1, n), rng.uniform(y0, y1, n), rng.uniform(z0, z1, n)
        ])

    return np.vstack([
        slab(n_floor, l0, l1, -hw, hw, floor_z - 0.01, floor_z + 0.01),
        slab(n_wall, l0, l1, -hw - 0.01, -hw + 0.01, floor_z, wall_top),
        slab(n_wall, l0, l1, hw - 0.01, hw + 0.01, floor_z, wall_top),
        slab(3000, l0 - 0.01, l0 + 0.01, -hw, hw, floor_z, wall_top),
        slab(3000, l1 - 0.01, l1 + 0.01, -hw, hw, floor_z, wall_top),
    ])


def test_height_axis_is_not_taken_from_a_denser_side_wall():
    """侧壁比底板密时，高度轴必须仍来自底板，宽高不得互换。

    回归背景：历史实现直接取最大平面法向当高度轴。底板 12000 点 / 单侧壁
    40000 点时，H 轴偏离真实竖直 89.99°，量出的"车宽 1.99m / 车厢高 2.52m"
    正好是真值 2.50/2.00 互换，而姿态置信度仍然是 1.00 —— 下游没有任何判据
    能发现这次测量整体错了。现在借竖直基准跳过立面继续找底板，因此这类车不再
    被拒绝，而是量对。
    """
    cfg = Config(statistical_outlier_enabled=False)
    wall_heavy = _trailer(np.random.default_rng(20260905), 12000, 40000)

    local, transform = PointCloudGeometryNormalizer(cfg).process(
        wall_heavy, np.array([0.0, 0.0, 1.0]), "weighbridge"
    )
    spans = np.percentile(local, 99, axis=0) - np.percentile(local, 1, axis=0)

    assert transform.vertical_alignment > 0.95
    assert 2.4 < spans[0] < 2.6, f"车宽应约 2.50m，实际 {spans[0]:.2f}"
    assert 1.8 < spans[2] < 2.1, f"栏板深应约 2.00m，实际 {spans[2]:.2f}"


def test_height_axis_accepts_floor_dominant_plane_with_correct_spans():
    """同一构型下底板占多数时必须正常通过，且宽高不互换。"""
    cfg = Config(statistical_outlier_enabled=False)
    floor_heavy = _trailer(np.random.default_rng(20260905), 60000, 12000)

    local, transform = PointCloudGeometryNormalizer(cfg).process(
        floor_heavy, np.array([0.0, 0.0, 1.0]), "weighbridge"
    )
    spans = np.percentile(local, 99, axis=0) - np.percentile(local, 1, axis=0)

    assert 2.4 < spans[0] < 2.6      # 车宽 2.50
    assert 11.5 < spans[1] < 11.9    # 车长 11.70
    assert 1.8 < spans[2] < 2.1      # 栏板深 2.00
    assert transform.vertical_alignment > 0.99


def test_process_requires_vertical_reference_by_default():
    """默认必须有可信竖直基准，缺失即拒绝测量。"""
    cfg = Config(statistical_outlier_enabled=False)
    trailer = _trailer(np.random.default_rng(3), 60000, 12000)

    with pytest.raises(ValueError, match="缺少可信竖直基准"):
        PointCloudGeometryNormalizer(cfg).process(trailer)


def test_missing_vertical_reference_caps_confidence_when_explicitly_allowed():
    """显式允许降级时可以继续，但姿态置信度不得再报满分。"""
    cfg = Config(
        statistical_outlier_enabled=False,
        geometry_require_vertical_reference=False,
    )
    trailer = _trailer(np.random.default_rng(4), 60000, 12000)

    _, transform = PointCloudGeometryNormalizer(cfg).process(trailer)

    assert transform.vertical_source == "none"
    assert transform.vertical_alignment is None
    assert transform.confidence <= 0.5


def test_height_axis_is_oriented_up_by_the_vertical_reference():
    """Open3D 平面法向符号任意；H 轴必须由竖直基准定向，否则底板与车顶互换。"""
    cfg = Config(statistical_outlier_enabled=False)
    trailer = _trailer(np.random.default_rng(5), 60000, 12000)
    normalizer = PointCloudGeometryNormalizer(cfg)

    up = np.array([0.0, 0.0, 1.0])
    _, transform_up = normalizer.process(trailer, up, "weighbridge")
    # 传入朝下的基准即等价于"这份点云的上方在 -Z"，H 轴必须跟着翻转。
    _, transform_down = normalizer.process(trailer, -up, "weighbridge")

    assert float(transform_up.rotation[:, 2] @ up) > 0.99
    assert float(transform_down.rotation[:, 2] @ (-up)) > 0.99


def test_height_axis_searches_past_a_vertical_dominant_plane():
    """最大平面是立面时应继续找底板，而不是直接拒绝整单测量。

    回归背景：平板车/自卸车底盘的最大平面实测是一块偏离竖直 89.5° 的挡板，
    只否决不重找会让这一整类车全部无法测量。
    """
    rng = np.random.default_rng(20260907)

    def slab(n, x0, x1, y0, y1, z0, z1):
        return np.column_stack([
            rng.uniform(x0, x1, n), rng.uniform(y0, y1, n), rng.uniform(z0, z1, n)
        ])

    # 挡板（立面）点数刻意多于平板面，使它成为最大平面
    headboard = slab(40000, 5.0 - 0.01, 5.0 + 0.01, -1.2, 1.2, 1.7, 4.1)
    deck = slab(18000, -5, 5, -1.2, 1.2, 1.69, 1.71)
    cfg = Config(statistical_outlier_enabled=False)

    local, transform = PointCloudGeometryNormalizer(cfg).process(
        np.vstack([headboard, deck]), np.array([0.0, 0.0, 1.0]), "weighbridge"
    )

    assert transform.vertical_alignment > 0.95, "应当选中平板面而不是挡板"
    assert float(transform.rotation[:, 2] @ [0.0, 0.0, 1.0]) > 0.95


def test_height_axis_rejects_when_every_candidate_plane_is_vertical():
    """所有候选都是立面时仍必须拒绝，不能退回"随便挑一块"。"""
    rng = np.random.default_rng(3)

    def slab(n, x0, x1, y0, y1, z0, z1):
        return np.column_stack([
            rng.uniform(x0, x1, n), rng.uniform(y0, y1, n), rng.uniform(z0, z1, n)
        ])

    walls = np.vstack([
        slab(20000, x - 0.01, x + 0.01, -1.5, 1.5, 0.0, 3.0) for x in (-2.0, 0.0, 2.0)
    ] + [
        slab(20000, -2.0, 2.0, y - 0.01, y + 0.01, 0.0, 3.0) for y in (-1.5, 1.5)
    ])
    cfg = Config(statistical_outlier_enabled=False)

    with pytest.raises(ValueError, match="都不是水平面"):
        PointCloudGeometryNormalizer(cfg).process(
            walls, np.array([0.0, 0.0, 1.0]), "weighbridge"
        )
