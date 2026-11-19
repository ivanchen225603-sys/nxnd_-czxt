"""重构等价性基线测试。

这些用例把重构前的实际输出固化成断言，用于证明"重构不改变业务行为"。
数值容差取 1e-9（浮点重排级别），而不是宽松的业务容差 —— 目的是抓住任何
算法改动，而不只是抓住"结果还大致对"。

所有场景用合成点云，不依赖真实 PCD 文件，可在 CI 中稳定复现。
"""

import numpy as np
import pytest

from carpointcloud_analyzer import Config
from carpointcloud_analyzer.domain.carriage import (
    CarriageAnalyzer,
    floor_surface,
    occupancy_extent,
    repair_low_wall,
)
from carpointcloud_analyzer.domain.floor import FloorProfileAnalyzer
from carpointcloud_analyzer.domain.geometry import (
    LocalCarriageExtractor,
    LocalRibDetector,
    PointCloudGeometryNormalizer,
)
from carpointcloud_analyzer.domain.preprocessing import (
    ScenePreprocessor,
    cluster_dims,
    estimate_ground_plane,
    horizontal_pca_frame,
)

EXACT = 1e-9


# ---------------------------------------------------------------------------
# 合成场景：一台空车 + 地面 + 干扰物，覆盖整条流水线
# ---------------------------------------------------------------------------
def _slab(rng, n, x0, x1, y0, y1, z0, z1):
    return np.column_stack([
        rng.uniform(x0, x1, n),
        rng.uniform(y0, y1, n),
        rng.uniform(z0, z1, n),
    ])


@pytest.fixture(scope="module")
def scene():
    """40x20m 地面 + 底板离地 1.2m 的空车 + 护栏 + 立柱。

    车辆足迹内不铺地面点，模拟俯视雷达被车厢遮挡。
    """
    rng = np.random.default_rng(20260720)
    floor_z = 1.20
    ground = _slab(rng, 90000, -20, 20, -10, 10, -0.01, 0.01)
    ground = ground[~(
        (ground[:, 0] > -6.5) & (ground[:, 0] < 8.7) & (np.abs(ground[:, 1]) < 1.6)
    )]
    parts = [ground, _slab(rng, 12000, -6, 6, -1.25, 1.25, floor_z - 0.01, floor_z + 0.01)]
    for wall_y in (-1.25, 1.25):
        parts.append(_slab(rng, 7000, -6, 6, wall_y - 0.02, wall_y + 0.02, floor_z, 2.6))
    for wall_x in (-6, 6):
        parts.append(_slab(rng, 2500, wall_x - 0.02, wall_x + 0.02, -1.25, 1.25, floor_z, 2.6))
    parts.append(_slab(rng, 6000, 6.5, 8.5, -1.1, 1.1, 0.5, 3.0))     # 车头，比车厢窄
    parts.append(_slab(rng, 9000, -18, 18, 9.0, 9.05, 0.0, 1.1))      # 护栏
    parts.append(_slab(rng, 1600, 12.0, 12.1, 5.0, 5.1, 0.0, 6.0))    # 电线杆
    return np.vstack(parts)


@pytest.fixture(scope="module")
def pipeline_stages(scene):
    """跑到车厢剖面为止的中间结果，供多个用例共享（避免重复计算）。"""
    cfg = Config()
    pre = ScenePreprocessor(cfg).process(scene)
    local, transform = PointCloudGeometryNormalizer(cfg).process(
        pre.primary_vehicle_points, pre.ground_plane[:3], "scene_ground"
    )
    split = LocalCarriageExtractor(cfg).split(local)
    profile = CarriageAnalyzer(cfg).analyze(split.carriage_points)
    return {
        "cfg": cfg, "pre": pre, "local": local,
        "transform": transform, "split": split, "profile": profile,
    }


# ---------------------------------------------------------------------------
# 前处理
# ---------------------------------------------------------------------------
def test_preprocessing_isolates_single_vehicle(pipeline_stages):
    pre = pipeline_stages["pre"]
    assert pre.applied
    assert pre.stats["n_vehicle_clusters"] == 1
    # 护栏与立柱必须被剔除：保留点数应远小于输入
    assert pre.stats["n_vehicle"] < pre.stats["n_input"] * 0.5
    # 车辆点云的横向跨度应为车宽量级，而不是场景量级
    span = pre.primary_vehicle_points.max(axis=0) - pre.primary_vehicle_points.min(axis=0)
    assert max(span) < 20.0


def test_ground_plane_is_found_and_upward(pipeline_stages, scene):
    plane = pipeline_stages["pre"].ground_plane
    assert plane is not None
    assert plane[2] > 0                                  # 法向朝上
    assert abs(np.linalg.norm(plane[:3]) - 1.0) < 1e-6   # 已归一化
    # 合成地面在 z=0，故 d≈0；法向≈(0,0,1)
    assert abs(plane[3]) < 0.05
    assert plane[2] == pytest.approx(1.0, abs=1e-3)


def test_ground_plane_survives_removal_of_ground_points(pipeline_stages):
    """地面点被剔除，但平面系数仍在 —— 这是高度基准能用的前提。"""
    pre = pipeline_stages["pre"]
    assert pre.stats["ground"] > 0        # 确实剔除了地面点
    assert pre.ground_plane is not None   # 平面系数仍然可用


def test_local_frame_spans_match_vehicle_not_scene(pipeline_stages):
    local = pipeline_stages["local"]
    assert pipeline_stages["transform"].confidence > 0.0
    spans = np.percentile(local, 99, axis=0) - np.percentile(local, 1, axis=0)
    width, length, height = spans
    assert 2.0 < width < 3.5      # 车宽
    assert 12.0 < length < 17.0   # 整车长（含车头）
    assert 1.0 < height < 3.5


# ---------------------------------------------------------------------------
# 车厢剖面与尺寸
# ---------------------------------------------------------------------------
def test_carriage_profile_measures_symmetric_walls(pipeline_stages):
    profile = pipeline_stages["profile"]
    assert profile.valid
    # 合成车两侧壁等高，不应触发"单侧扫描不全"
    assert profile.wall_top_diff == pytest.approx(0.0, abs=0.05)
    assert profile.scan_incomplete_side is None
    # 壁深 = 底板(1.2) → 壁顶(2.6)
    assert profile.wall_depth == pytest.approx(1.4, abs=0.06)


def test_empty_carriage_fill_metrics_are_low(pipeline_stages):
    profile = pipeline_stages["profile"]
    cfg = pipeline_stages["cfg"]
    # 空车三指标都应远低于阈值，投票判空
    assert profile.median_fill_ratio < cfg.empty_fill_ratio_max
    assert profile.high_cell_fraction < cfg.empty_high_cell_frac_max
    assert profile.mid_band_density < cfg.empty_mid_density_max


def test_carriage_length_matches_end_planes(pipeline_stages):
    """车厢长必须等于端面之差 —— 拉筋间距序列以此为基准，两者必须同源。"""
    split = pipeline_stages["split"]
    cfg = pipeline_stages["cfg"]
    from carpointcloud_analyzer.domain.dimensions import DimensionCalculator
    dims = DimensionCalculator(cfg).calculate(
        pipeline_stages["local"], split.carriage_points, split, pipeline_stages["profile"]
    )
    assert dims.carriage_length == pytest.approx(
        abs(split.split_y - split.carriage_start_y), abs=EXACT
    )
    # 整车高是车辆局部 H 轴的稳健可见跨度，不能再复写栏板深度。
    expected_height = np.percentile(
        pipeline_stages["local"][:, 2], 99
    ) - np.percentile(pipeline_stages["local"][:, 2], 1)
    assert dims.vehicle_height == pytest.approx(expected_height, abs=EXACT)
    assert dims.vehicle_height > dims.carriage_height


# ---------------------------------------------------------------------------
# 底板剖面：真实地面基准
# ---------------------------------------------------------------------------
def test_floor_profile_uses_real_ground_datum(pipeline_stages):
    st = pipeline_stages
    world_plane = st["pre"].ground_plane
    local_plane = st["transform"].plane_to_local(world_plane)
    result = FloorProfileAnalyzer(st["cfg"]).analyze(
        st["split"].carriage_points,
        st["profile"].interior_w_range,
        front_l=st["split"].split_y,
        rear_l=st["split"].carriage_start_y,
        ground_plane_local=local_plane,
        ground_plane_world=world_plane,
    )
    assert result is not None
    assert result.ground_plane_source == "scene_ransac"
    assert result.ground_plane_frame == "source"
    assert result.case == 1                       # 合成底板平整
    # 底板离地真值 1.20m
    assert result.segments[0].height_to_ground == pytest.approx(1.20, abs=0.03)
    # 导出的是源坐标系平面，d≈0（局部系会是车辆中位数偏移，约 1.5）
    assert abs(result.ground_plane[3]) < 0.05


# ---------------------------------------------------------------------------
# 纯函数：重构最容易破坏的地方
# ---------------------------------------------------------------------------
def test_floor_surface_prefers_density_peak_over_low_percentile():
    """底板层有厚度 + 下方有穿透点时，应贴合密集层而非被穿透点拉低。"""
    rng = np.random.default_rng(1)
    layer = rng.normal(2.0, 0.01, 4000)          # 底板密集层
    penetration = rng.uniform(1.5, 1.9, 40)      # 少量穿透点
    z = np.concatenate([layer, penetration])
    assert floor_surface(z) == pytest.approx(2.0, abs=0.02)
    # 低分位会被穿透点带偏，密度峰法不会
    assert floor_surface(z) > float(np.percentile(z, 5.0))


def test_floor_surface_falls_back_on_tiny_sample():
    z = np.array([1.0, 1.1, 1.2])
    assert floor_surface(z) == pytest.approx(float(np.percentile(z, 5.0)), abs=EXACT)


def test_occupancy_extent_keeps_sparse_tail_drops_flyers():
    dense = np.linspace(0.0, 10.0, 500)
    sparse_tail = np.linspace(10.1, 12.0, 70)    # ≈7 点/bin，稀疏但过门槛
    flyer = np.array([25.0])                     # 孤立飞点
    values = np.concatenate([dense, sparse_tail, flyer])
    lo, hi = occupancy_extent(values, bin_width=0.2, min_points=5)
    assert lo == pytest.approx(0.0, abs=0.05)
    assert hi < 13.0            # 飞点被排除
    assert hi > 11.5            # 稀疏尾部被保留


def test_occupancy_extent_treats_subthreshold_trickle_as_noise():
    """门槛是"每 bin 绝对点数"，不随点云总量自适应。

    低于 min_points 的零散点按噪声丢弃。不能改成"按空隙连通性扩展"——
    整车长依赖本函数跨越车厢与车头之间的空隙（实测 3.22m），连通性判据
    会把车头判为不连通，整车长从 15.84m 掉到 11.3m。
    """
    dense = np.full(500, 1.0)
    trickle = np.linspace(5.0, 9.0, 8)           # 每 bin 不足 5 点
    lo, hi = occupancy_extent(np.concatenate([dense, trickle]), 0.2, 5)
    assert hi == pytest.approx(1.0, abs=0.05)


def test_occupancy_extent_returns_real_extrema_not_bin_edges():
    """返回值必须是区间内真实极值，取 bin 边界会让跨度虚高一个 bin 宽。"""
    values = np.concatenate([np.full(50, 1.03), np.full(50, 4.97)])
    lo, hi = occupancy_extent(values, bin_width=1.0, min_points=5)
    assert lo == pytest.approx(1.03, abs=EXACT)
    assert hi == pytest.approx(4.97, abs=EXACT)


def test_horizontal_pca_frame_is_orthonormal_and_reversible():
    rng = np.random.default_rng(2)
    points = np.column_stack([
        rng.uniform(-6, 6, 800), rng.uniform(-1, 1, 800), rng.uniform(0, 2, 800)
    ])
    local, rotation, center = horizontal_pca_frame(points)
    assert np.allclose(rotation.T @ rotation, np.eye(2), atol=1e-9)
    restored = local[:, :2] @ rotation.T + center
    assert np.allclose(restored, points[:, :2], atol=1e-9)
    assert np.allclose(local[:, 2], points[:, 2], atol=EXACT)
    # 主轴对齐最长方向
    assert np.ptp(local[:, 0]) > np.ptp(local[:, 1])


def test_cluster_dims_orders_length_width_height():
    rng = np.random.default_rng(3)
    cluster = np.column_stack([
        rng.uniform(-5, 5, 600), rng.uniform(-1, 1, 600), rng.uniform(0, 2.5, 600)
    ])
    length, width, height = cluster_dims(cluster)
    assert length > width and length > height
    assert length == pytest.approx(10.0, abs=0.4)
    assert width == pytest.approx(2.0, abs=0.2)
    assert height == pytest.approx(2.5, abs=0.2)


# ---------------------------------------------------------------------------
# 边界与异常
# ---------------------------------------------------------------------------
def test_preprocessing_degrades_only_when_explicitly_opted_in():
    """降级把整片环境当车辆返回，必须显式开启，不能是默认行为。"""
    rng = np.random.default_rng(4)
    ground_only = _slab(rng, 30000, -20, 20, -10, 10, -0.01, 0.01)
    cfg = Config(scene_preprocessing_fail_open=True)
    result = ScenePreprocessor(cfg).process(ground_only)
    assert not result.applied
    assert result.reason
    assert len(result.primary_vehicle_points) == len(ground_only)


def test_preprocessing_fails_closed_by_default():
    """默认必须 fail-closed：车辆提取失败时不得把地面/护栏交给尺寸算法。

    回归背景：默认 fail-open 时，任何忘记显式关掉它的入口都会把环境点静默
    写进正式结果，而调用方拿不到任何信号。
    """
    rng = np.random.default_rng(40)
    ground_only = _slab(rng, 30000, -20, 20, -10, 10, -0.01, 0.01)

    assert Config().scene_preprocessing_fail_open is False
    with pytest.raises(ValueError, match="场景前处理未能可靠提取车辆"):
        ScenePreprocessor(Config()).process(ground_only)


def test_preprocessing_skips_small_prepared_cloud():
    """已裁剪的小点云直接放行，不该被当场景硬跑一遍。"""
    rng = np.random.default_rng(5)
    small = _slab(rng, 500, -1, 1, -1, 1, 0, 1)
    result = ScenePreprocessor(Config()).process(small)
    assert not result.applied
    assert "点数" in result.reason


def test_estimate_ground_plane_returns_none_for_cropped_cloud():
    """裁剪到车身的点云没有地面，必须返回 None 而不是凑一个平面出来。"""
    rng = np.random.default_rng(6)
    vehicle_only = _slab(rng, 8000, -6, 6, -1.25, 1.25, 1.2, 2.6)
    params = ScenePreprocessor(Config()).params()
    assert estimate_ground_plane(vehicle_only, params) is None


def test_estimate_ground_plane_handles_degenerate_input():
    params = ScenePreprocessor(Config()).params()
    assert estimate_ground_plane(np.zeros((5, 3)), params) is None


def test_repair_low_wall_noop_when_walls_match(pipeline_stages):
    mirrored, info = repair_low_wall(
        pipeline_stages["split"].carriage_points,
        pipeline_stages["profile"],
        pipeline_stages["cfg"],
    )
    assert len(mirrored) == 0
    assert info["repaired"] is False


def test_rib_detector_suppresses_output_when_floor_invisible(pipeline_stages):
    """把车厢填满（模拟载货）后不应输出拉筋 —— 那些峰是货堆起伏。"""
    st = pipeline_stages
    rng = np.random.default_rng(7)
    carriage = st["split"].carriage_points
    filled = np.vstack([carriage, np.column_stack([
        rng.uniform(-1.0, 1.0, 20000),
        rng.uniform(carriage[:, 1].min(), carriage[:, 1].max(), 20000),
        rng.uniform(carriage[:, 2].min() + 0.2, carriage[:, 2].max(), 20000),
    ])])
    ribs = LocalRibDetector(st["cfg"]).detect(
        filled, st["split"].carriage_start_y, st["split"].split_y
    )
    assert ribs == []


# ---------------------------------------------------------------------------
# 车头必须完整计入（空车测量的硬要求）
# ---------------------------------------------------------------------------
def test_head_tail_cover_full_vehicle_extent(pipeline_stages):
    """headY/tailY 必须覆盖整车真实纵向范围，不能把稀疏的车头切掉。

    车头沿纵向的点密度远低于车厢，早期用 99.5 分位定 headY，实测把车头前端
    切掉 1.235m，导致 cabToFrontWall 偏小、且与 vehicle_length 相差 1.3m。
    """
    split = pipeline_stages["split"]
    length = pipeline_stages["local"][:, 1]
    # 允许 occupancy_extent 排除孤立飞点，但不该丢掉成片的车头结构
    assert split.head_y >= float(np.percentile(length, 99.9)) - 0.05
    assert split.tail_y <= float(np.percentile(length, 0.1)) + 0.05


def test_vehicle_length_matches_head_tail_span(pipeline_stages):
    """整车长与 headY-tailY 必须同源 —— 两者都描述整车纵向范围，不能各算各的。"""
    from carpointcloud_analyzer.domain.dimensions import DimensionCalculator
    split = pipeline_stages["split"]
    dims = DimensionCalculator(pipeline_stages["cfg"]).calculate(
        pipeline_stages["local"], split.carriage_points, split, pipeline_stages["profile"]
    )
    assert dims.vehicle_length == pytest.approx(split.head_y - split.tail_y, abs=1e-6)


def test_cab_segment_is_not_empty(pipeline_stages):
    """车厢前壁与车头前端之间必须有实质长度，否则说明车头没被识别出来。"""
    split = pipeline_stages["split"]
    assert split.cab_to_front_wall > 0.5


# ---------------------------------------------------------------------------
# 可复现性：同一份点云重复处理必须给出同样的测量值
# ---------------------------------------------------------------------------
def test_preprocessing_is_reproducible(scene):
    """同一场景重复前处理，车辆点集必须逐点一致。

    open3d 的 segment_plane 是随机 RANSAC 且不接受种子，早期实现里内点集合
    每次都不同，导致剔除后的车辆点集不同、所有尺寸在毫米级漂移。现在地面
    平面与内点都做了交替收敛，结果不再依赖初始随机假设。
    """
    cfg = Config()
    first = ScenePreprocessor(cfg).process(scene).primary_vehicle_points
    for _ in range(2):
        again = ScenePreprocessor(cfg).process(scene).primary_vehicle_points
        assert len(again) == len(first)
        assert np.array_equal(np.sort(again, axis=0), np.sort(first, axis=0))


def test_local_frame_is_reproducible(scene):
    """局部坐标系的三个轴向重复运行必须稳定 —— 它决定全部尺寸口径。"""
    cfg = Config()
    points = ScenePreprocessor(cfg).process(scene).primary_vehicle_points
    vertical = ScenePreprocessor(cfg).process(scene).ground_plane[:3]
    axes = [
        PointCloudGeometryNormalizer(cfg)
        .process(points, vertical, "scene_ground")[1]
        .rotation
        for _ in range(3)
    ]
    for rotation in axes[1:]:
        # 逐轴夹角，允许 0.01° 的数值噪声
        cosines = np.abs(np.sum(rotation * axes[0], axis=0))
        angles = np.degrees(np.arccos(np.clip(cosines, -1.0, 1.0)))
        assert angles.max() < 0.01, f"坐标轴未收敛，最大夹角 {angles.max():.4f}°"


def test_ground_plane_is_reproducible(scene):
    cfg = Config()
    params = ScenePreprocessor(cfg).params()
    planes = [estimate_ground_plane(scene, params) for _ in range(3)]
    for plane in planes[1:]:
        angle = np.degrees(np.arccos(np.clip(abs(plane[:3] @ planes[0][:3]), -1, 1)))
        assert angle < 0.01
        assert abs(plane[3] - planes[0][3]) < 1e-3


# ---------------------------------------------------------------------------
# 配置错误汇总的公开行为
# ---------------------------------------------------------------------------


def test_plane_to_local_preserves_point_plane_distance(pipeline_stages):
    """平面变换必须保距 —— 否则所有离地高度都是错的。"""
    st = pipeline_stages
    transform = st["transform"]
    world_plane = st["pre"].ground_plane
    local_plane = transform.plane_to_local(world_plane)
    sample = st["pre"].primary_vehicle_points[:500]
    d_world = sample @ world_plane[:3] + world_plane[3]
    d_local = transform.to_local(sample) @ local_plane[:3] + local_plane[3]
    assert np.allclose(d_world, d_local, atol=1e-9)
