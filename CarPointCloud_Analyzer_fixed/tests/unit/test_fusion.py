"""双雷达标定与合并测试。

用合成的双雷达数据验证：外参能否从**已知真值**中恢复、退化场景能否被识别、
合并后密度是否均匀。真实数据到位前，这是唯一能证明算法正确的手段。

场景模拟现场布置：磅道地面 + 护栏 + 立柱 + 一辆停稳的车，两台雷达各看一部分
且有重叠。真值外参已知，标定结果与之比对。
"""

import numpy as np
import pytest

from carpointcloud_analyzer.domain import fusion
from carpointcloud_analyzer.domain.fusion import (
    Extrinsic,
    MergeParams,
    _overlap_stats,
    calibrate,
    merge,
)


def test_evaluate_returns_zero_only_for_expected_open3d_runtime_error(monkeypatch):
    def raise_runtime_error(*args, **kwargs):
        raise RuntimeError("degenerate correspondence")

    monkeypatch.setattr(
        fusion.o3d.pipelines.registration,
        "evaluate_registration",
        raise_runtime_error,
    )

    assert fusion._evaluate(object(), object(), np.eye(4), 0.1) == 0.0


def test_evaluate_does_not_swallow_programming_errors(monkeypatch):
    def raise_value_error(*args, **kwargs):
        raise ValueError("bad test input")

    monkeypatch.setattr(
        fusion.o3d.pipelines.registration,
        "evaluate_registration",
        raise_value_error,
    )

    with pytest.raises(ValueError, match="bad test input"):
        fusion._evaluate(object(), object(), np.eye(4), 0.1)


def _rotation_z(degrees: float) -> np.ndarray:
    angle = np.deg2rad(degrees)
    return np.array([
        [np.cos(angle), -np.sin(angle), 0.0],
        [np.sin(angle), np.cos(angle), 0.0],
        [0.0, 0.0, 1.0],
    ])


def _rotation_y(degrees: float) -> np.ndarray:
    angle = np.deg2rad(degrees)
    return np.array([
        [np.cos(angle), 0.0, np.sin(angle)],
        [0.0, 1.0, 0.0],
        [-np.sin(angle), 0.0, np.cos(angle)],
    ])


def _make_transform(yaw_deg, pitch_deg, translation) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = _rotation_z(yaw_deg) @ _rotation_y(pitch_deg)
    transform[:3, 3] = translation
    return transform


def _slab(rng, n, x0, x1, y0, y1, z0, z1):
    return np.column_stack([
        rng.uniform(x0, x1, n), rng.uniform(y0, y1, n), rng.uniform(z0, z1, n)
    ])


def _weighbridge_scene(rng, with_vehicle=True):
    """磅道场景：地面 + 护栏 + 立柱（+ 停稳的车）。"""
    parts = [_slab(rng, 40000, -15, 15, -8, 8, -0.01, 0.01)]      # 地面
    parts.append(_slab(rng, 6000, -12, 12, 6.0, 6.06, 0.0, 1.1))   # 护栏
    for x in (-8.0, 0.0, 8.0):                                     # 立柱
        parts.append(_slab(rng, 1500, x - 0.08, x + 0.08, -5.0, -4.84, 0.0, 2.5))
    # 磅道台面的横向边沿。真实磅道必有，且是重叠区里唯一能约束"沿行车方向"
    # 平移的结构 —— 护栏和车壁都沿行车方向延伸，平移时会顺着它们滑动
    for x in (-2.0, 2.0):
        parts.append(_slab(rng, 4000, x - 0.05, x + 0.05, -6, 6, 0.0, 0.30))
    if with_vehicle:
        parts.append(_slab(rng, 9000, -5, 5, -1.2, 1.2, 1.15, 1.2))     # 底板
        for wall_y in (-1.2, 1.2):                                       # 侧壁
            parts.append(_slab(rng, 6000, -5, 5, wall_y - 0.03, wall_y + 0.03, 1.2, 2.5))
        parts.append(_slab(rng, 3000, 5.0, 6.8, -1.05, 1.05, 0.6, 2.9))  # 车头
    return np.vstack(parts)


def _split_views(scene, truth, rng, noise=0.004):
    """按可见性把场景切成两台雷达的视角，B 侧再变换到 B 自身坐标系。

    A 看后半场景，B 看前半场景，中间一段共视 —— 与现场布置一致。
    """
    view_a = scene[scene[:, 0] < 3.0]
    view_b = scene[scene[:, 0] > -3.0]
    view_a = view_a + rng.normal(0, noise, view_a.shape)
    view_b = view_b + rng.normal(0, noise, view_b.shape)
    # scene 处于 A 坐标系；把共视部分反变换到 B 自身坐标系
    inverse = np.linalg.inv(truth)
    view_b = view_b @ inverse[:3, :3].T + inverse[:3, 3]
    return view_a, view_b


@pytest.fixture(scope="module")
def dual_view():
    rng = np.random.default_rng(20260720)
    truth = _make_transform(yaw_deg=12.0, pitch_deg=-4.0, translation=[2.5, -1.3, 0.6])
    scene = _weighbridge_scene(rng, with_vehicle=True)
    view_a, view_b = _split_views(scene, truth, rng)
    return {"truth": truth, "a": view_a, "b": view_b}


def _pose_error(estimated, truth):
    """返回 (旋转误差角度, 平移误差米)。"""
    delta = np.linalg.inv(truth) @ estimated
    cos_angle = (np.trace(delta[:3, :3]) - 1.0) / 2.0
    angle = np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0)))
    return float(angle), float(np.linalg.norm(delta[:3, 3]))


# ---------------------------------------------------------------------------
# 标定
# ---------------------------------------------------------------------------
def test_calibration_recovers_known_extrinsic(dual_view):
    result = calibrate(dual_view["a"], dual_view["b"], source="synthetic")
    angle, distance = _pose_error(result.matrix, dual_view["truth"])
    assert angle < 1.0, f"旋转误差 {angle:.3f}° 过大"
    assert distance < 0.05, f"平移误差 {distance*1000:.1f}mm 过大"


def test_calibration_reports_quality(dual_view):
    result = calibrate(dual_view["a"], dual_view["b"])
    assert result.quality is not None
    assert result.quality.fitness > 0.3
    assert result.quality.inlier_rmse < 0.08
    assert result.quality.trustworthy


def test_calibration_flags_degenerate_flat_only_overlap():
    """重叠区只有平地时必须报欠约束，而不是给一个看着可信的错误外参。"""
    rng = np.random.default_rng(7)
    truth = _make_transform(0.0, 0.0, [1.0, 0.5, 0.0])
    flat = _slab(rng, 30000, -15, 15, -8, 8, -0.005, 0.005)
    view_a = flat[flat[:, 0] < 3.0]
    view_b = flat[flat[:, 0] > -3.0]
    inverse = np.linalg.inv(truth)
    view_b = view_b @ inverse[:3, :3].T + inverse[:3, 3]
    result = calibrate(view_a, view_b)
    assert not result.quality.trustworthy
    assert result.quality.degenerate_axes or result.quality.warnings


def test_calibration_rejects_tiny_input():
    with pytest.raises(ValueError, match="点数过少"):
        calibrate(np.zeros((10, 3)), np.zeros((10, 3)))


# ---------------------------------------------------------------------------
# 外参文件
# ---------------------------------------------------------------------------
def test_extrinsic_roundtrip(tmp_path, dual_view):
    original = calibrate(
        dual_view["a"],
        dual_view["b"],
        source="roundtrip",
        camera_a="CAM-A",
        camera_b="CAM-B",
    )
    path = original.save(str(tmp_path / "extrinsic.json"))
    loaded = Extrinsic.load(path)
    assert np.allclose(loaded.matrix, original.matrix, atol=1e-8)
    assert loaded.source == "roundtrip"
    assert loaded.camera_a == "CAM-A"
    assert loaded.camera_b == "CAM-B"
    loaded.validate_camera_pair("cam-a", "cam-b")


def test_extrinsic_roundtrip_normalizes_weighbridge_plane(tmp_path):
    original = Extrinsic(
        matrix=np.eye(4),
        camera_a="CAM-A",
        camera_b="CAM-B",
        weighbridge_plane_a=np.array([0.0, 0.0, 2.0, -0.6]),
    )

    loaded = Extrinsic.load(original.save(str(tmp_path / "site.json")))

    assert np.allclose(loaded.weighbridge_plane_a, [0.0, 0.0, 1.0, -0.3])


def test_extrinsic_rejects_invalid_weighbridge_plane():
    with pytest.raises(ValueError, match="weighbridgePlaneA"):
        Extrinsic.from_dict(
            {
                "frame": "B_to_A",
                "matrix": np.eye(4).tolist(),
                "weighbridgePlaneA": [0.0, 0.0, 0.0, 1.0],
            }
        )

    with pytest.raises(ValueError, match="baselineResidual"):
        Extrinsic.from_dict(
            {
                "frame": "B_to_A",
                "matrix": np.eye(4).tolist(),
                "baselineResidual": 0.0,
            }
        )


def test_extrinsic_rejects_non_rigid_matrix():
    """含缩放/镜像的矩阵必须拒收 —— 静默接受会让所有尺寸系统性偏差。"""
    scaled = np.eye(4)
    scaled[0, 0] = 1.5
    with pytest.raises(ValueError, match="正交阵|刚体"):
        Extrinsic.from_dict({"matrix": scaled.tolist()})

    mirrored = np.eye(4)
    mirrored[2, 2] = -1.0
    with pytest.raises(ValueError, match="镜像|刚体"):
        Extrinsic.from_dict({"matrix": mirrored.tolist()})


def test_extrinsic_rejects_wrong_shape():
    with pytest.raises(ValueError, match="形状"):
        Extrinsic.from_dict({"matrix": np.eye(3).tolist()})


def test_extrinsic_rejects_wrong_frame_and_online_camera_order():
    with pytest.raises(ValueError, match="B_to_A"):
        Extrinsic.from_dict(
            {"frame": "A_to_B", "matrix": np.eye(4).tolist()}
        )

    extrinsic = Extrinsic.from_dict(
        {
            "frame": "B_to_A",
            "cameraA": "CAM-A",
            "cameraB": "CAM-B",
            "matrix": np.eye(4).tolist(),
        }
    )
    with pytest.raises(ValueError, match="与请求不一致"):
        extrinsic.validate_camera_pair("CAM-B", "CAM-A")


def test_online_camera_validation_requires_camera_metadata():
    extrinsic = Extrinsic.from_dict(
        {"frame": "B_to_A", "matrix": np.eye(4).tolist()}
    )
    with pytest.raises(ValueError, match="cameraA"):
        extrinsic.validate_camera_pair("CAM-A", "CAM-B")


def test_extrinsic_apply_matches_manual_transform(dual_view):
    extrinsic = Extrinsic(matrix=dual_view["truth"])
    points = dual_view["b"][:200]
    expected = points @ dual_view["truth"][:3, :3].T + dual_view["truth"][:3, 3]
    assert np.allclose(extrinsic.apply(points), expected, atol=1e-9)


def test_extrinsic_apply_handles_empty():
    extrinsic = Extrinsic(matrix=np.eye(4))
    assert len(extrinsic.apply(np.empty((0, 3)))) == 0


# ---------------------------------------------------------------------------
# 合并
# ---------------------------------------------------------------------------
def test_merge_uniformizes_density_in_overlap(dual_view):
    """重叠区不能是双倍密度 —— 底板面密度峰等判据全按密度工作。"""
    extrinsic = Extrinsic(matrix=dual_view["truth"])
    params = MergeParams(merge_voxel_size=0.05)
    result = merge(dual_view["a"], dual_view["b"], extrinsic, params)

    merged = result.points
    # 只比同一种结构（地面）的密度：重叠带里还有车体，直接比整体会把
    # "场景内容不同"误当成"密度翻倍"
    def ground_density(points, lo, hi):
        band = points[
            (points[:, 0] > lo) & (points[:, 0] < hi) & (np.abs(points[:, 2]) < 0.05)
        ]
        return len(band) / max(hi - lo, 1e-6)

    overlap = ground_density(merged, -1.5, 1.5)
    single = ground_density(merged, -12.0, -9.0)
    assert overlap < single * 1.4, (
        f"重叠区地面密度 {overlap:.0f} 显著高于单雷达区 {single:.0f}，体素去重未生效"
    )


def test_merge_preserves_full_extent(dual_view):
    """合并后的纵向范围必须覆盖两台雷达各自的范围 —— 这正是补车头的目的。"""
    extrinsic = Extrinsic(matrix=dual_view["truth"])
    result = merge(dual_view["a"], dual_view["b"], extrinsic)
    transformed_b = extrinsic.apply(dual_view["b"])
    expected_max = max(dual_view["a"][:, 0].max(), transformed_b[:, 0].max())
    expected_min = min(dual_view["a"][:, 0].min(), transformed_b[:, 0].min())
    assert result.points[:, 0].max() > expected_max - 0.1
    assert result.points[:, 0].min() < expected_min + 0.1


def test_merge_reports_overlap_residual(dual_view):
    extrinsic = Extrinsic(matrix=dual_view["truth"])
    result = merge(dual_view["a"], dual_view["b"], extrinsic)
    assert result.overlap_rmse is not None
    assert result.overlap_rmse < 0.05          # 真值外参下高分位残差应接近噪声水平
    assert not result.warnings
    assert result.quality_passed


def test_overlap_residual_uses_only_actual_matches_for_complementary_views():
    """互补视角在同一包围盒内的不可见表面不能污染已匹配点残差。"""
    rng = np.random.default_rng(20260903)
    shared = np.column_stack([
        np.full(120, -0.8),
        rng.uniform(-1.0, 1.0, 120),
        rng.uniform(0.2, 2.0, 120),
    ])
    points_a = np.vstack([
        shared,
        np.array([
            [-1.0, -1.0, 0.1],
            [-1.0, 1.0, 2.1],
            [1.0, -1.0, 0.1],
            [1.0, 1.0, 2.1],
        ]),
    ])
    matched_b = shared[:80] + rng.normal(0.0, 0.003, (80, 3))
    unseen_surface_b = np.column_stack([
        np.full(320, 0.5),
        rng.uniform(-0.9, 0.9, 320),
        rng.uniform(0.3, 1.9, 320),
    ])

    residual, match_ratio = _overlap_stats(
        points_a,
        np.vstack([matched_b, unseen_surface_b]),
        MergeParams(),
    )

    assert residual is not None and residual < 0.02
    assert match_ratio is not None and 0.12 < match_ratio < 0.30


def test_overlap_stats_skips_ground_estimation_for_vehicle_only_clouds(monkeypatch):
    """共同场景已提取车辆后不得再随机把车顶/侧壁当作地面。"""
    points_a = np.array(
        [[0.0, y, z] for y in np.linspace(-1.0, 1.0, 20)
         for z in np.linspace(0.2, 2.0, 20)]
    )
    points_b = points_a.copy()

    def unexpected_ground_estimation(points):
        raise AssertionError("vehicle-only overlap must not estimate ground")

    monkeypatch.setattr(fusion, "_dominant_ground", unexpected_ground_estimation)

    residual, match_ratio = _overlap_stats(
        points_a,
        points_b,
        MergeParams(filter_ground_for_overlap=False),
    )

    assert residual is not None and residual < 0.01
    assert match_ratio == pytest.approx(1.0)


@pytest.mark.parametrize("drift", [0.10, 0.20, 0.35])
def test_merge_warns_when_extrinsic_drifted(dual_view, drift):
    """外参漂移必须在每帧被发现，而不是等测量数据明显不对才察觉。

    门槛以标定基线为准而非绝对值：绝对阈值无法适配不同现场的噪声水平，
    实测 10cm 漂移残差仅 77mm，固定用 150mm 会静默放行。
    """
    truth = dual_view["truth"]
    baseline, _ = _overlap_stats(
        dual_view["a"], Extrinsic(matrix=truth).apply(dual_view["b"]), MergeParams()
    )
    drifted = truth.copy()
    drifted[:3, 3] += np.array([drift, 0.0, 0.0])
    result = merge(
        dual_view["a"], dual_view["b"],
        Extrinsic(matrix=drifted, baseline_residual=baseline),
    )
    assert result.warnings, f"{drift*100:.0f}cm 漂移未被发现"
    assert not result.quality_passed
    assert result.reason


def test_merge_accepts_frame_matching_calibration_baseline(dual_view):
    """无漂移时不能误报 —— 门槛必须容得下现场噪声。"""
    truth = dual_view["truth"]
    baseline, _ = _overlap_stats(
        dual_view["a"], Extrinsic(matrix=truth).apply(dual_view["b"]), MergeParams()
    )
    result = merge(
        dual_view["a"], dual_view["b"],
        Extrinsic(matrix=truth, baseline_residual=baseline),
    )
    assert not result.warnings


def test_calibration_records_baseline_residual(dual_view):
    result = calibrate(dual_view["a"], dual_view["b"])
    assert result.baseline_residual is not None
    assert result.baseline_residual < 0.05


def test_merge_warns_when_no_overlap():
    rng = np.random.default_rng(9)
    far_a = _slab(rng, 3000, -20, -15, -2, 2, 0, 1)
    far_b = _slab(rng, 3000, 30, 35, -2, 2, 0, 1)
    result = merge(far_a, far_b, Extrinsic(matrix=np.eye(4)))
    assert result.overlap_rmse is None
    assert any("重叠" in w for w in result.warnings)
    assert not result.quality_passed


def test_merge_result_serializes(dual_view):
    result = merge(dual_view["a"], dual_view["b"], Extrinsic(matrix=dual_view["truth"]))
    payload = result.to_dict()
    assert payload["pointsMerged"] > 0
    assert payload["pointsFromA"] == len(dual_view["a"])
    assert payload["qualityPassed"] is True
    assert payload["mode"] == "fused"
    assert payload["pointsCombined"] == (
        payload["pointsFilteredA"] + payload["pointsFilteredB"]
    )


def test_merge_light_filters_each_sensor_and_reports_stage_counts(dual_view):
    """有限点和明显孤立飞点在外参变换前按雷达分别清理。"""
    points_a = np.vstack([
        dual_view["a"],
        np.array([[np.nan, 0.0, 0.0], [1_000_000.0, 1_000_000.0, 1_000_000.0]]),
    ])
    points_b = np.vstack([
        dual_view["b"],
        np.array([[0.0, np.inf, 0.0], [-1_000_000.0, -1_000_000.0, -1_000_000.0]]),
    ])

    result = merge(points_a, points_b, Extrinsic(matrix=dual_view["truth"]))
    payload = result.to_dict()

    assert payload["pointsFromA"] == len(points_a)
    assert payload["pointsFromB"] == len(points_b)
    assert payload["pointsFiniteA"] == len(points_a) - 1
    assert payload["pointsFiniteB"] == len(points_b) - 1
    assert payload["pointsFilteredA"] < payload["pointsFiniteA"]
    assert payload["pointsFilteredB"] < payload["pointsFiniteB"]
    assert payload["pointsCombined"] == (
        payload["pointsFilteredA"] + payload["pointsFilteredB"]
    )


# ---------------------------------------------------------------------------
# 逐帧漂移门禁开关：支架永久固定时改由一次性空磅道验收确认
# ---------------------------------------------------------------------------
def _drifted_pair():
    """构造一对"外参明显对不上"的点云：立体结构整体错开 40cm。"""
    rng = np.random.default_rng(20260906)
    wall = np.column_stack([
        rng.uniform(-6.0, 6.0, 4000),
        np.full(4000, 1.2) + rng.normal(0, 0.004, 4000),
        rng.uniform(0.3, 2.5, 4000),
    ])
    ground = np.column_stack([
        rng.uniform(-12.0, 12.0, 20000),
        rng.uniform(-6.0, 6.0, 20000),
        rng.normal(0, 0.004, 20000),
    ])
    points_a = np.vstack([wall, ground])
    shifted = points_a.copy()
    shifted[:, 0] += 0.40
    return points_a, shifted


def test_overlap_gate_blocks_drift_when_enforced():
    """默认仍然拦截：离线标定与独立合并工具依赖这条门禁。"""
    points_a, points_b = _drifted_pair()
    extrinsic = Extrinsic(matrix=np.eye(4), baseline_residual=0.02)

    result = merge(points_a, points_b, extrinsic, MergeParams())

    assert result.quality_passed is False
    assert result.overlap_notes
    assert any(note in result.warnings for note in result.overlap_notes)


def test_overlap_gate_records_but_does_not_block_when_disabled():
    """生产双雷达链路：指标继续计算并留痕，但不再据此降级。

    回归背景：载车帧零漂移时匹配率仅 14.76%、门槛 12%，余量 23% 且随车形与
    停车位置浮动 —— 用它逐帧判漂移误杀正常车辆的概率高于发现真漂移。外参漂移
    改由一次性空磅道验收确认（两组独立空磅数据匹配率只差 0.15pp）。
    """
    points_a, points_b = _drifted_pair()
    extrinsic = Extrinsic(matrix=np.eye(4), baseline_residual=0.02)

    result = merge(
        points_a, points_b, extrinsic, MergeParams(enforce_overlap_gate=False)
    )

    assert result.quality_passed is True
    assert result.reason is None
    assert result.overlap_notes, "指标必须继续计算并留痕，只是不参与门禁"
    assert not result.warnings
    assert "overlapNotes" in result.to_dict()


def test_incomplete_data_still_blocks_even_with_gate_disabled():
    """关掉的只是漂移判定；点数不足这类"数据本身不完整"始终拦截。"""
    rng = np.random.default_rng(7)
    tiny = rng.normal(0, 1.0, (120, 3))
    extrinsic = Extrinsic(matrix=np.eye(4), baseline_residual=0.02)

    result = merge(
        tiny, tiny.copy(), extrinsic, MergeParams(enforce_overlap_gate=False)
    )

    assert result.quality_passed is False
    assert any("轻滤后仅" in w for w in result.warnings)


def test_overlap_stats_prefers_supplied_ground_plane():
    """重叠统计必须优先用标定面划分立体结构，而不是逐帧拟合最大平面。

    回归背景：真实地磅现场最大平面是磅台周边地坪，比磅台面低 0.76m。用它做
    基准后"离地 >0.15m 才算立体结构"把整个磅台面也留了下来 —— 126906 个
    "结构点"里 104545 个（82.4%）其实是磅台面，样本仍被平面主导，平面内漂移
    看不出来。
    """
    rng = np.random.default_rng(11)
    apron = np.column_stack([          # 周边地坪：最大平面，z=0
        rng.uniform(-20.0, 20.0, 40000),
        rng.uniform(-10.0, 10.0, 40000),
        rng.normal(0, 0.004, 40000),
    ])
    deck = np.column_stack([           # 磅台面：比地坪高 0.76m
        rng.uniform(-6.0, 6.0, 20000),
        rng.uniform(-1.5, 1.5, 20000),
        np.full(20000, 0.76) + rng.normal(0, 0.004, 20000),
    ])
    post = np.column_stack([           # 立柱：唯一真正的立体结构
        np.full(1500, 4.0) + rng.normal(0, 0.02, 1500),
        np.full(1500, 2.0) + rng.normal(0, 0.02, 1500),
        rng.uniform(0.76, 3.0, 1500),
    ])
    points = np.vstack([apron, deck, post])
    params = MergeParams(filter_ground_for_overlap=True)
    deck_plane = np.array([0.0, 0.0, 1.0, -0.76])   # 磅台面，法向朝上

    # 平面内平移：只有真正剔除了磅台面，匹配率才会掉下来
    shifted = points.copy()
    shifted[:, 0] += 0.30

    # 零漂移时两种口径都应判为对齐
    _, auto_base = _overlap_stats(points, points.copy(), params)
    _, deck_base = _overlap_stats(points, points.copy(), params, deck_plane)
    assert auto_base is not None and auto_base > 0.9
    assert deck_base is not None and deck_base > 0.9

    _, auto_ratio = _overlap_stats(points, shifted, params)
    _, deck_ratio = _overlap_stats(points, shifted, params, deck_plane)

    # 自动拟合口径：磅台面被当成"立体结构"留下，平面内漂移几乎看不见
    assert auto_ratio is not None and auto_ratio > 0.8, (
        f"自动口径本就对平面内漂移不敏感，实测 {auto_ratio}"
    )
    # 标定面口径：只剩立柱，30cm 漂移后要么彻底失配（None）要么匹配率大幅下降
    assert deck_ratio is None or deck_ratio < 0.5, (
        f"用标定面划分结构后应显著更敏感，实际 {deck_ratio}"
    )


def test_extrinsic_round_trips_empty_lane_baseline(tmp_path):
    """一次性空磅道验收结论必须能随外参文件持久化。"""
    baseline = {
        "structureMinHeightM": 0.5,
        "residualP90M": 0.075,
        "matchRatio": 0.77,
        "verifiedOn": ["09-03 11:14", "09-04 11:20/11:23"],
    }
    extrinsic = Extrinsic(
        matrix=np.eye(4),
        camera_a="CAM_004",
        camera_b="CAM_005",
        baseline_residual=0.04433,
        empty_lane_baseline=baseline,
    )
    path = tmp_path / "extrinsic.json"
    extrinsic.save(str(path))

    loaded = Extrinsic.load(str(path))

    assert loaded.empty_lane_baseline == baseline


def test_extrinsic_rejects_non_object_empty_lane_baseline():
    with pytest.raises(ValueError, match="emptyLaneBaseline"):
        Extrinsic.from_dict({
            "matrix": np.eye(4).tolist(),
            "frame": "B_to_A",
            "cameraA": "CAM_004",
            "cameraB": "CAM_005",
            "emptyLaneBaseline": "not-an-object",
        })
