import numpy as np
import pytest

from carpointcloud_analyzer import Config
from carpointcloud_analyzer.application.dual_cloud_merge import (
    DualCloudMergeError,
    site_frame_from_plane,
)
from carpointcloud_analyzer.application.pipeline import PointCloudAnalyzer


def test_estimated_site_frame_repeats_exactly_with_competing_surfaces():
    """场地系重复估计保持一致；地面与抬高平面同时存在。"""
    from carpointcloud_analyzer.domain.site_frame import estimate_site_frame

    rng = np.random.default_rng(23364)
    ground = np.c_[rng.uniform(-10., 10., 12000), rng.uniform(-5., 5., 12000),
                   rng.normal(0., .03, 12000)]
    deck = np.c_[rng.uniform(-8., 8., 6000), rng.uniform(-2., 2., 6000),
                 rng.normal(.7, .02, 6000)]
    scene = np.vstack((ground, deck))
    frames = [estimate_site_frame(scene) for _ in range(4)]
    for frame in frames[1:]:
        np.testing.assert_array_equal(frame.axes, frames[0].axes)
        np.testing.assert_array_equal(frame.origin, frames[0].origin)
        np.testing.assert_array_equal(frame.plane, frames[0].plane)


def _sideways_scene() -> tuple[np.ndarray, np.ndarray]:
    """构造“源 X 是竖直方向”的场景，验证算法不依赖 XYZ 固定含义。"""
    rng = np.random.default_rng(20260731)
    scale = np.column_stack(
        [
            np.full(2000, 2.0),
            rng.uniform(-8.0, 8.0, 2000),
            rng.uniform(-3.0, 3.0, 2000),
        ]
    )
    vehicle = np.column_stack(
        [
            rng.uniform(2.5, 5.0, 1500),
            rng.uniform(-5.0, 5.0, 1500),
            rng.uniform(-1.2, 1.2, 1500),
        ]
    )
    return np.vstack([scale, vehicle]), scale


def test_calibrated_weighbridge_plane_establishes_z_up_for_swapped_axes():
    points, scale = _sideways_scene()
    frame = site_frame_from_plane(points, np.array([2.0, 0.0, 0.0, -4.0]))

    leveled_scale = frame.transform(scale)
    leveled_all = frame.transform(points)

    assert np.max(np.abs(leveled_scale[:, 2])) < 1e-9
    assert np.percentile(leveled_all[:, 2], 90) > 1.0
    assert np.allclose(frame.plane, [1.0, 0.0, 0.0, -2.0])


def test_pipeline_uses_calibrated_plane_before_environment_preprocessing():
    points, _ = _sideways_scene()
    cfg = Config(
        scene_auto_level_enable=True,
        weighbridge_plane_source=(1.0, 0.0, 0.0, -2.0),
        scene_auto_level_required=True,
    )
    analyzer = PointCloudAnalyzer(cfg)

    leveled, analysis_plane, source_plane, calibrated = analyzer._level_scene(points)

    assert calibrated is True
    assert analyzer._last_site_frame is not None
    assert np.allclose(analysis_plane, [0.0, 0.0, 1.0, 0.0])
    assert np.allclose(source_plane, [1.0, 0.0, 0.0, -2.0])
    assert np.percentile(leveled[:, 2], 90) > 1.0


def test_required_scene_leveling_fails_closed_when_no_support_plane():
    cfg = Config(
        scene_auto_level_enable=True,
        scene_auto_level_required=True,
    )
    analyzer = PointCloudAnalyzer(cfg)

    with pytest.raises(ValueError, match="无法可靠确定场地竖直轴"):
        analyzer._level_scene(np.zeros((20, 3)))


def test_calibrated_plane_rejects_normal_pointing_away_from_vehicle():
    points, _ = _sideways_scene()

    with pytest.raises(DualCloudMergeError, match="法向疑似背向车辆"):
        site_frame_from_plane(points, np.array([-1.0, 0.0, 0.0, 2.0]))


def test_pipeline_reuses_plane_orientation_verified_in_the_combined_scene():
    """单雷达背景落在地磅下方时，不应推翻共同场景已验证的法向。"""
    points, _ = _sideways_scene()
    rng = np.random.default_rng(20260904)
    below_weighbridge_background = np.column_stack(
        [
            np.full(5000, -20.0),
            rng.uniform(-8.0, 8.0, 5000),
            rng.uniform(-3.0, 3.0, 5000),
        ]
    )
    single_radar_points = np.vstack([points, below_weighbridge_background])
    cfg = Config(
        scene_auto_level_enable=True,
        scene_auto_level_required=True,
        weighbridge_plane_source=(1.0, 0.0, 0.0, -2.0),
        weighbridge_plane_orientation_verified=True,
    )

    leveled, analysis_plane, source_plane, calibrated = (
        PointCloudAnalyzer(cfg)._level_scene(single_radar_points)
    )

    assert calibrated is True
    assert np.allclose(analysis_plane, [0.0, 0.0, 1.0, 0.0])
    assert np.allclose(source_plane, [1.0, 0.0, 0.0, -2.0])
    assert np.percentile(leveled[: len(points), 2], 90) > 1.0


def _weighbridge_scene(rng) -> np.ndarray:
    """地面 + 空车厢 + 车头的 Z-up 合成场景（俯视雷达看不到车底）。"""
    floor_z, wall_top, l0, l1, hw = 1.57, 3.57, -5.85, 5.85, 1.25

    def slab(n, x0, x1, y0, y1, z0, z1):
        return np.column_stack([
            rng.uniform(x0, x1, n), rng.uniform(y0, y1, n), rng.uniform(z0, z1, n)
        ])

    def lower_end_wall(wall_x):
        width, height = np.meshgrid(
            np.linspace(-hw, hw, 126), np.linspace(floor_z + 0.08, floor_z + 0.45, 20)
        )
        return np.column_stack([
            rng.uniform(wall_x - 0.01, wall_x + 0.01, width.size),
            width.ravel(), height.ravel(),
        ])

    return np.vstack([
        slab(60000, -25.0, 25.0, -12.0, 12.0, -0.02, 0.02),               # 地面
        slab(30000, l0, l1, -hw, hw, floor_z - 0.01, floor_z + 0.01),     # 底板
        slab(6000, l0, l1, -hw - 0.01, -hw + 0.01, floor_z, wall_top),    # 左侧壁
        slab(6000, l0, l1, hw - 0.01, hw + 0.01, floor_z, wall_top),      # 右侧壁
        slab(1500, l0 - 0.01, l0 + 0.01, -hw, hw, floor_z, wall_top),     # 后端壁
        slab(1500, l1 - 0.01, l1 + 0.01, -hw, hw, floor_z, wall_top),     # 前端壁
        slab(4500, l1 + 1.6, l1 + 4.0, -1.1, 1.1, 0.9, 2.9),              # 车头
        # 原全高端壁在任一侧4cm宽、20cm高的下部测线内仅有约2个点，
        # 无法承担新增的实测内缘口径。补足两端下部壁面的约2cm均匀网格采样，
        # 不改物理边界或只在算法取点位置塞点；旋转前后仍使用同一场景。
        lower_end_wall(l0),
        lower_end_wall(l1),
    ])


def _analyze_points(points: np.ndarray, tmp_path, name: str) -> dict:
    """把点云写成 PCD 并走完整的 analyze 入口。"""
    import open3d as o3d

    path = tmp_path / f"{name}.pcd"
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    assert o3d.io.write_point_cloud(str(path), cloud, write_ascii=False)
    cfg = Config()
    cfg.input_unit = "m"
    return PointCloudAnalyzer(cfg).analyze(str(path))


@pytest.mark.parametrize(
    "name, rotation",
    [
        ("rot_x_90", np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)),
        ("rot_y_270", np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]], dtype=float)),
    ],
)
def test_default_entry_is_invariant_to_source_axis_orientation(
    name, rotation, tmp_path
):
    """默认配置下，同一场景换个源轴向必须得到同一份尺寸。

    回归背景：``scene_auto_level_enable`` 曾默认关闭，只有 /analyze-pair 会显式
    打开。环境算法内部以 Z-up 为前提，因此把场景绕 X 轴转 90° 后地面法向判据
    直接失效 —— 提车整体失败并降级返回整幅场景（实测 230000 点 / 50m 跨度），
    地面基准同时丢失。单文件与批量入口因此隐含要求"输入 Z 必须朝上"。
    """
    scene = _weighbridge_scene(np.random.default_rng(20260905))

    upright = _analyze_points(scene, tmp_path, f"{name}_upright")
    rotated = _analyze_points(scene @ rotation.T, tmp_path, name)

    base = upright["integrationPayloads"]["vehicleDimensions"]["dimensions"]
    other = rotated["integrationPayloads"]["vehicleDimensions"]["dimensions"]

    assert upright["loadStatus"]["status"] == rotated["loadStatus"]["status"] == "Empty"
    # 估计场地系只证明轴向一致，不能充当地磅标定；车头距必须缺测。
    for result in (upright, rotated):
        assert result["measurementStatus"]["valid"] is False
        assert result["measurementStatus"]["cabMeasurement"]["reason"] == 'weighbridge_plane_unavailable'
    assert base['cabToTrailerFrontDist'] is other['cabToTrailerFrontDist'] is None
    for field in ("trailerLength", "trailerWidth", "trailerBodyHeight"):
        # 1cm 容差：RANSAC 平面拟合在不同轴向下有毫米级抖动，不是轴向依赖。
        assert abs(base[field] - other[field]) <= 1.0, (
            f"{field} 随源轴向变化：{base[field]} vs {other[field]}"
        )


def _slab(rng, n, x0, x1, y0, y1, z0, z1):
    return np.column_stack([
        rng.uniform(x0, x1, n), rng.uniform(y0, y1, n), rng.uniform(z0, z1, n)
    ])


def test_orientation_check_ignores_background_far_below_the_deck():
    """磅台下方的远处背景不得推翻一份正确的标定面。

    回归背景：判据原本比较 P99 上方跨度与 |P1| 下方跨度，而这两个量取决于当帧
    能看到多远的背景。实测同一份正确标定面，0903 共同场景 P99/|P1|=1.08（仅以
    0.27m 侥幸通过），0907 共同场景 =0.28 直接被判反向、整单失败。改为只在
    ±5m 有界带内比较点数后，远处背景不再参与投票。
    """
    from carpointcloud_analyzer.application.dual_cloud_merge import site_frame_from_plane

    rng = np.random.default_rng(20260907)
    plane = np.array([0.0, 0.0, 1.0, 0.0])          # z=0 即磅台面，法向朝上
    deck = _slab(rng, 20000, -6, 6, -2, 2, -0.01, 0.01)
    vehicle = _slab(rng, 30000, -5, 5, -1.2, 1.2, 0.3, 3.5)
    # 远处地坑/背景：数量与跨度都远超车辆，但都在 -5m 以外
    background = _slab(rng, 60000, -40, 40, -30, 30, -35.0, -6.0)
    scene = np.vstack([deck, vehicle, background])

    frame = site_frame_from_plane(scene, plane)
    assert np.allclose(frame.plane[:3], [0.0, 0.0, 1.0])


def test_orientation_check_still_rejects_a_genuinely_flipped_plane():
    """判据放宽后仍必须拦住真正写反符号的标定面。"""
    from carpointcloud_analyzer.application.dual_cloud_merge import (
        DualCloudMergeError, site_frame_from_plane,
    )

    rng = np.random.default_rng(11)
    deck = _slab(rng, 20000, -6, 6, -2, 2, -0.01, 0.01)
    vehicle = _slab(rng, 30000, -5, 5, -1.2, 1.2, 0.3, 3.5)
    scene = np.vstack([deck, vehicle])

    with pytest.raises(DualCloudMergeError, match="背向车辆"):
        site_frame_from_plane(scene, np.array([0.0, 0.0, -1.0, 0.0]))


@pytest.mark.parametrize('verified', [False, True])
def test_merge_keeps_verified_plane_when_current_returns_are_below_it(monkeypatch, verified):
    """同一标定和点云，已验收朝向继续融合，未验收朝向按当前点分布拒绝。"""
    from types import SimpleNamespace

    from carpointcloud_analyzer.application import dual_cloud_merge
    from carpointcloud_analyzer.domain.fusion import Extrinsic, MergeResult

    rng = np.random.default_rng(20261103)
    points_a = rng.uniform([-2., -6., -2.], [2., 6., -.5], (200, 3))
    points_b = rng.uniform([-2., -6., -2.], [2., 6., -.5], (200, 3))
    config = Config(weighbridge_plane_source=(0., 0., 1., 0.),
                    weighbridge_plane_orientation_verified=verified)
    # 隔离后续提车和体素融合；本用例实际执行共同场景的标定面建系。
    monkeypatch.setattr(dual_cloud_merge, 'filter_scene', lambda scene, _: SimpleNamespace(
        vehicle_points_list=[scene], primary_index=0, primary_vehicle_points=scene,
        stats={}, edge_truncated=False,
    ))
    monkeypatch.setattr(dual_cloud_merge, 'merge', lambda a, b, *_: MergeResult(
        points=a, n_from_a=len(a), n_from_b=len(b), n_after_uniform=len(a),
        overlap_rmse=None, quality_passed=True,
    ))
    if not verified:
        with pytest.raises(DualCloudMergeError, match='背向车辆'):
            dual_cloud_merge.merge_vehicle_clouds(points_a, points_b, Extrinsic(np.eye(4)), config=config)
        return

    result = dual_cloud_merge.merge_vehicle_clouds(
        points_a, points_b, Extrinsic(np.eye(4)), config=config)
    assert result.fusion.quality_passed
    np.testing.assert_allclose(result.site_frame.plane, [0., 0., 1., 0.])
    np.testing.assert_allclose(result.points, points_a, atol=1e-12)
