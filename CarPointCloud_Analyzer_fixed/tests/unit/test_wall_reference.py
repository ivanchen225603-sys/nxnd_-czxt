"""四壁参考补全适用门禁及与正式测量的隔离。"""

from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from scipy.spatial import cKDTree

from carpointcloud_analyzer import Config
from carpointcloud_analyzer import PointCloudAnalyzer
from carpointcloud_analyzer.domain.box_geometry import rotate_wl
from carpointcloud_analyzer.domain.geometry import CoordinateTransform
from carpointcloud_analyzer.domain.wall_reference import complete_rectangular_wall_reference


def rectangular_shell(seed=71):
    rng = np.random.default_rng(seed)
    walls = []
    for l_value, count in [(0., 7000), (6., 5500)]:
        walls.append(np.column_stack([
            rng.uniform(-1.25, 1.25, count),
            rng.normal(l_value, 0.008, count),
            rng.uniform(1.5, 4.1, count),
        ]))
    # 中段只能看到一条上缘。模型必须补左右壁而不是靠镜像复制此细线。
    walls.append(np.column_stack([
        rng.normal(1.25, 0.005, 1200), rng.uniform(0, 6, 1200),
        rng.uniform(3.92, 4.1, 1200),
    ]))
    return np.vstack(walls)


def test_reference_fills_four_walls_only_and_preserves_observed_points():
    p = rectangular_shell()
    # 两端有遮挡，必须补端壁缺口。
    p = p[~((p[:, 0] < -0.5) & (p[:, 2] > 2.5))]
    original = p.copy()
    result = complete_rectangular_wall_reference(p, Config())
    assert result.metadata["completed"]
    assert result.metadata["referenceOnly"]
    assert not result.metadata["topGenerated"] and not result.metadata["bottomGenerated"]
    assert all(w["synthesizedPoints"] > 0 for w in result.metadata["walls"].values())
    box = result.box
    assert abs(box.length - 6) < 0.05
    assert abs(box.width - 2.5) < 0.05
    assert result.metadata['observedHeightM'] == pytest.approx(2.6, abs=.06)
    # 箱体高度就是观测跨度，不再被配置的标称高顶替。
    assert box.height == pytest.approx(result.metadata['observedHeightM'], abs=1e-12)
    aligned = rotate_wl(result.points, -np.deg2rad(result.metadata["yawDeg"]))
    on_wall = np.minimum.reduce([
        abs(aligned[:, 0] - box.w_range[0]), abs(aligned[:, 0] - box.w_range[1]),
        abs(aligned[:, 1] - box.rear_l), abs(aligned[:, 1] - box.front_l),
    ])
    assert np.max(on_wall) < 1e-10
    assert np.all(aligned[:, 2] >= box.h_range[0])
    assert np.all(aligned[:, 2] <= box.h_range[1])
    assert result.metadata['surfaceMode'] == 'complete_symmetric_grid'
    # 完整模型允许与实测点邻近，不再挖掉端壁网格。
    assert np.array_equal(original, p)
    assert len(result.points) == sum(w["synthesizedPoints"] for w in result.metadata["walls"].values())


@pytest.mark.parametrize("angle", [-4., 4.])
def test_reference_handles_both_yaw_signs_and_translation(angle):
    p = rotate_wl(rectangular_shell(), np.deg2rad(angle)) + [2., 7., -3.]
    result = complete_rectangular_wall_reference(p, Config())
    assert result.box is not None, result.metadata
    assert abs(result.box.width - 2.5) < 0.1
    assert abs(result.box.length - 6) < 0.1
    assert abs(result.metadata["yawDeg"] - angle) < 0.5


@pytest.mark.parametrize("height", [.5, 1.5, 2.8])
def test_visible_empty_floor_or_cargo_keeps_existing_pipeline(height):
    p = rectangular_shell()
    rng = np.random.default_rng(22)
    inside = np.column_stack([
        rng.uniform(-1., 1., 2000), rng.uniform(0.5, 5.5, 2000),
        np.full(2000, height),
    ])
    result = complete_rectangular_wall_reference(np.vstack([p, inside]), Config())
    assert result.box is None
    assert result.metadata["reason"] == "interior_observed_use_existing_pipeline"
    assert len(result.points) == 0


@pytest.mark.parametrize("case", ["one_wall", "low_board", "unequal_width", "sloping_end", "no_rim"])
def test_unsupported_vehicle_shapes_do_not_get_rectangular_walls(case):
    p = rectangular_shell()
    if case == "one_wall":
        p = p[p[:, 1] < 0.1]
    elif case == "low_board":
        p[:, 2] *= 0.4
    elif case == "unequal_width":
        p[p[:, 1] > 5.9, 0] *= 0.65
    elif case == "sloping_end":
        selected = p[:, 1] > 5.9
        p[selected, 1] += 0.3 * p[selected, 2]
    else:
        p = p[(p[:, 1] < 0.1) | (p[:, 1] > 5.9)]
    result = complete_rectangular_wall_reference(p, Config())
    assert result.box is None, result.metadata
    assert len(result.points) == 0


def test_chassis_below_end_wall_cannot_inflate_reference_width():
    p = rectangular_shell()
    rng = np.random.default_rng(33)
    chassis = np.column_stack([rng.uniform(-2., 2., 5000), rng.normal(0, .01, 5000),
                               rng.uniform(0., 1.1, 5000)])
    result = complete_rectangular_wall_reference(np.vstack([p, chassis]), Config())
    assert result.box is not None
    assert abs(result.box.width - 2.5) < 0.06
    assert abs(result.metadata['observedHRange'][0] - 1.5) < 0.05
    assert result.box.h_range[0] == pytest.approx(result.metadata['observedHRange'][0])


def test_reference_can_be_disabled_and_bounds_generation_resources():
    assert complete_rectangular_wall_reference(
        rectangular_shell(), Config(carriage_wall_reference_enable=False)
    ).metadata["reason"] == "wall_reference_disabled"
    assert complete_rectangular_wall_reference(
        rectangular_shell(), Config(carriage_box_surface_spacing=1e-9)
    ).metadata["reason"] == "reference_spacing_too_small"
    assert complete_rectangular_wall_reference(
        rectangular_shell() * 1e9, Config()
    ).metadata["reason"] == "unsupported_scene_extent"


def stub_preparation(monkeypatch, analyzer, p):
    transform = CoordinateTransform(np.zeros(3), np.eye(3))
    monkeypatch.setattr(analyzer, "_level_scene", lambda points: (points, None, None, False))
    monkeypatch.setattr(analyzer.preprocessor, "process", lambda points: SimpleNamespace(
        primary_vehicle_points=points, ground_plane=None,
    ))
    monkeypatch.setattr(analyzer.geometry, "process", lambda *args: (p, transform))
    monkeypatch.setattr(analyzer.geometry, "vertical_candidate", lambda *args: (p, transform))


def test_preview_reference_bypasses_business_classification_and_writes(monkeypatch):
    p = rectangular_shell()
    analyzer = PointCloudAnalyzer()
    stub_preparation(monkeypatch, analyzer, p)
    forbidden = Mock(side_effect=AssertionError("reference entered business pipeline"))
    monkeypatch.setattr(analyzer.local_splitter, "split", forbidden)
    monkeypatch.setattr(analyzer, "_assess_load", forbidden)
    monkeypatch.setattr(analyzer, "_save_clean_pointcloud", forbidden)
    result = analyzer.analyze_preview(p)
    assert result["loadStatus"]["status"] == "Uncertain"
    assert result["syntheticPointCount"] > 0
    assert result["referenceOnly"]
    assert len(result["coordinates"]) <= 60000
    assert "dimensions" not in result and "integrationPayloads" not in result
    forbidden.assert_not_called()


def test_formal_shell_without_floor_returns_diagnostics_without_empty_archive(monkeypatch):
    p = rectangular_shell()
    original = p.copy()
    analyzer = PointCloudAnalyzer()
    stub_preparation(monkeypatch, analyzer, p)
    forbidden = Mock(side_effect=AssertionError("shell entered empty measurement pipeline"))
    for obj, method in [(analyzer.local_splitter, "split"), (analyzer.geometry, "process"),
                        (analyzer, "_assess_load")]:
        monkeypatch.setattr(obj, method, forbidden)
    writer = Mock(return_value='clean-review.pcd')
    monkeypatch.setattr(analyzer, '_save_clean_pointcloud', writer)
    analyzer.config.save_clean_pointcloud = True
    analyzer.config.license_plate = "TEST"
    result = analyzer._analyze_points(p, "test.pcd")
    assert result['integrationPayloads']['vehicleDimensions']['detectStatus'] == 'success'
    assert result['dimensions']['carriage']['height'] is None
    assert result['carriageObservation']['visibleHeightM'] == pytest.approx(2.6, abs=.06)
    assert result['carriageObservation']['modelHeightM'] == pytest.approx(2.6, abs=.06)
    assert result['measurementStatus']['heightSource'] == 'completed_cloud_wall_height'
    assert result['carriageObservation']['syntheticPointsUsedForMeasurement'] > 0
    review = result["reviewMeasurements"]
    assert review['dimensions']['vehicle']['length'] == pytest.approx(6, abs=.1)
    assert review['dimensions']['vehicle']['height'] == pytest.approx(2.6, abs=.06)
    assert review['dimensions']['carriage']['length'] == pytest.approx(6, abs=.05)
    assert review['dimensions']['carriage']['width'] == pytest.approx(2.5, abs=.05)
    assert review['dimensions']['carriage']['height'] is None
    assert review['fields']['dimensions.carriage.height']['status'] == 'unavailable'
    assert review['carriageHeight']['innerHeightM'] is None
    assert review['carriageVisibleWallSpanM'] == pytest.approx(2.6, abs=.06)
    assert review["wireCount"] is None and review["floorProfile"] is None
    assert review["cabToTrailerFrontDist"] is None  # 夹具只有箱体，没有车头朝向证据。
    assert result["measurementStatus"]["valid"] is False
    assert result["measurementStatus"]["calculationSucceeded"] is True
    assert result['measurementStatus']['reason'] == 'load_status_uncertain'
    assert "reviewStatus" not in result["measurementStatus"]
    assert "requiresManualReview" not in result["measurementStatus"]
    assert result["loadStatus"]["status"] == "Uncertain"
    assert result["wallCompletion"]["completed"]
    assert "reviewStatus" not in result["wallCompletion"]
    assert result['pcd'] == '' and result['pcdPointCount'] == 0
    assert result['syntheticPointCount'] > 0
    writer.assert_not_called()
    np.testing.assert_array_equal(p, original)
    observation = result["carriageObservation"]
    assert observation["lengthM"] == pytest.approx(6, abs=.05)
    assert observation["referenceCompletion"]["synthesizedPoints"] > 0
    assert observation["interiorDimensions"] is None  # 本夹具未观测到顶部内缘。
    forbidden.assert_not_called()


@pytest.mark.parametrize("case", ["visible_floor", "disabled", "one_end"])
def test_other_shapes_keep_original_pipeline(monkeypatch, case):
    p = rectangular_shell()
    if case == "visible_floor":
        rng = np.random.default_rng(5)
        floor = np.column_stack([rng.uniform(-1, 1, 2000),
                                 rng.uniform(.5, 5.5, 2000), np.full(2000, 1.5)])
        p = np.vstack([p, floor])
    elif case == "one_end":
        p = p[p[:, 1] < .1]
    analyzer = PointCloudAnalyzer(Config(carriage_wall_reference_enable=case != "disabled"))
    stub_preparation(monkeypatch, analyzer, p)
    original = Mock(side_effect=ValueError("original splitter"))
    monkeypatch.setattr(analyzer.local_splitter, "split", original)
    with pytest.raises(ValueError, match="original splitter"):
        analyzer._analyze_points(p, "test.pcd")
    np.testing.assert_array_equal(original.call_args.args[0], p)


@pytest.mark.parametrize("angle", [0, 37, 90, 173])
def test_vertical_candidate_is_order_stable_and_axis_independent(angle):
    from scipy.spatial.transform import Rotation
    p = rectangular_shell()
    rotation = Rotation.from_euler("xyz", [angle, 23, -42], degrees=True).as_matrix()
    p = p @ rotation.T + [5, 12, -7]
    vertical = rotation[:, 2]
    analyzer = PointCloudAnalyzer()
    local, transform = analyzer.geometry.vertical_candidate(p, vertical, "weighbridge")
    shuffled, repeated = analyzer.geometry.vertical_candidate(p[::-1], vertical, "weighbridge")
    np.testing.assert_array_equal(local, shuffled)
    np.testing.assert_array_equal(transform.rotation, repeated.rotation)
    np.testing.assert_allclose(transform.rotation[:, 2], vertical, atol=1e-12)
    np.testing.assert_allclose(transform.to_local(transform.to_source(local)), local, atol=1e-12)
    result = complete_rectangular_wall_reference(local, Config())
    assert result.box is not None, result.metadata
    assert result.box.length == pytest.approx(6, abs=.05)
    assert result.box.width == pytest.approx(2.5, abs=.05)


def test_vertical_candidate_requires_independent_reference():
    analyzer = PointCloudAnalyzer()
    assert analyzer.geometry.vertical_candidate(rectangular_shell(), None, "none") is None


def test_service_preserves_shell_diagnostics_without_persisting_uncertain_dimensions(monkeypatch):
    from carpointcloud_analyzer.application.service import AnalysisService
    p = rectangular_shell()
    analyzer = PointCloudAnalyzer()
    stub_preparation(monkeypatch, analyzer, p)
    monkeypatch.setattr(analyzer.loader, "load", lambda path: p)
    store = Mock(enabled=True)
    profiles = Mock(enabled=True)
    service = AnalysisService(dimension_store=store, empty_profile_storage=profiles)
    try:
        result = service._run_single(analyzer, "test.pcd", empty_profile_source="test.pcd")
    finally:
        service.shutdown()
    assert result["outputs"]["carriageObservation"]["referenceOnly"]
    assert result['outputs']['dimensions']['carriage']['height'] is None
    assert result['outputs']['reviewMeasurements']['dimensions']['carriage']['length'] == pytest.approx(6, abs=.05)
    assert result['outputs']['reviewMeasurements']['dimensions']['carriage']['height'] is None
    assert result['outputs']['loadStatus']['status'] == 'Uncertain'
    assert "reviewStatus" not in result["outputs"]["measurementStatus"]
    assert result["outputs"]["reviewMeasurements"]["wireCount"] is None
    assert result['integrationPayloads']['vehicleDimensions']['detectStatus'] == 'success'
    store.persist_vehicle_dimensions.assert_called_once()
    profiles.stage.assert_not_called()


def test_offline_reference_is_full_resolution_and_has_no_identity(monkeypatch):
    p = np.repeat(rectangular_shell(), 3, axis=0)
    analyzer = PointCloudAnalyzer()
    stub_preparation(monkeypatch, analyzer, p)
    monkeypatch.setattr(analyzer.loader, "load", lambda path: p)
    result = analyzer.analyze_wall_reference("not-read.pcd")
    assert result["measuredPointCount"] == len(p)
    for config in [Config(license_plate="TEST"), Config(save_clean_pointcloud=True)]:
        with pytest.raises(ValueError, match="preview must disable"):
            PointCloudAnalyzer(config).analyze_wall_reference("not-read.pcd")


@pytest.mark.parametrize('angle', [-4, 0, 4])
def test_matched_model_has_complete_end_grids_and_exact_left_right_symmetry(angle):
    p = rectangular_shell()
    # 非对称遮挡：只有一侧上缘且前端左半面缺失一条竖带。
    p = p[~((p[:, 1] < .1) & (p[:, 0] < -.3) & (p[:, 2] < 3.3))]
    p = rotate_wl(p, np.deg2rad(angle)) + [2., 7., -3.]
    original = p.copy()
    cfg = Config()
    result = complete_rectangular_wall_reference(p, cfg)
    assert result.metadata['profileMatched']
    q = rotate_wl(result.points, -np.deg2rad(result.metadata['yawDeg']))
    box = result.box
    # 箱体高度等于观测跨度；不存在任何配置来源的高度。
    assert box.height == pytest.approx(result.metadata['observedHeightM'])
    assert box.h_range == pytest.approx(tuple(result.metadata['observedHRange']))
    assert result.metadata['heightSource'] == 'observed_wall_extent'
    assert len(np.unique(np.round(q, 10), axis=0)) == len(q)  # 共用棱线不重复。
    mirror = q.copy()
    mirror[:, 0] = sum(box.w_range) - mirror[:, 0]
    assert cKDTree(q).query(mirror)[0].max() < 1e-10
    axes = [np.linspace(lo, hi, int(np.ceil((hi - lo) / cfg.carriage_box_surface_spacing)) + 1)
            for lo, hi in [box.w_range, (box.rear_l, box.front_l), box.h_range]]
    for axis, value in [(0, box.w_range[0]), (0, box.w_range[1]), (1, box.rear_l), (1, box.front_l)]:
        free = [i for i in range(3) if i != axis]
        aa, bb = np.meshgrid(axes[free[0]], axes[free[1]], indexing='ij')
        expected = np.empty((aa.size, 3))
        expected[:, axis] = value
        expected[:, free[0]], expected[:, free[1]] = aa.ravel(), bb.ravel()
        assert cKDTree(q).query(expected)[0].max() < 1e-10  # 四面全部格点无缺口。
    np.testing.assert_array_equal(p, original)


@pytest.mark.parametrize('case', ['longer', 'wider'])
def test_out_of_window_vehicles_are_not_recognised_as_the_profile(case):
    """长宽落在识别窗口之外的矩形车不按该车型处理，补点退回覆盖式填充。"""
    p = rectangular_shell()
    if case == 'longer':
        p[:, 1] *= 1.5
    elif case == 'wider':
        p[:, 0] *= 1.2
    cfg = Config()
    result = complete_rectangular_wall_reference(p, cfg)
    assert result.box is not None
    assert not result.metadata['profileMatched']
    assert result.metadata['surfaceMode'] == 'observed_coverage_fill'
    assert np.all(cKDTree(p).query(result.points)[0] > cfg.carriage_completion_dedup_radius)


@pytest.mark.parametrize('scale', [1.0, 1.25, 0.85])
def test_box_height_always_equals_observed_extent(scale):
    """无论是否识别为该车型，箱体高度都等于本次观测的上下沿跨度。

    回归背景：历史实现在识别到车型后把箱底下移到配置的标称整体高 2.9m，
    那个高度不来自当前点云，却会进入导出点云与 modelLowerEdge 等字段。
    """
    p = rectangular_shell()
    p[:, 2] = 1.5 + (p[:, 2] - 1.5) * scale
    result = complete_rectangular_wall_reference(p, Config())
    assert result.box is not None
    observed = result.metadata['observedHeightM']
    assert result.box.height == pytest.approx(observed)
    assert result.box.height == pytest.approx(2.6 * scale, abs=.05)
    assert result.metadata['bounds']['modelHeightM'] == pytest.approx(observed)
    assert result.metadata['bounds']['visibleHeightM'] == pytest.approx(observed)


def shell_with_front_lip():
    rng = np.random.default_rng(123)
    p = rectangular_shell()
    distance = rng.uniform(.17, .28, 800)
    lip = np.column_stack([rng.uniform(-1.1, .45, len(distance)), 6 - distance,
                           4.1 - .04 * distance / .28 + rng.normal(0, .003, len(distance))])
    cab = np.column_stack([rng.uniform(-1, 1, 1500), rng.uniform(7.5, 10, 1500),
                           rng.uniform(.3, 2., 1500)])
    return np.vstack([p, lip, cab])


@pytest.mark.parametrize('flip,angle', [(False, 0), (True, 0), (False, 4), (True, -4)])
def test_front_lip_follows_front_direction_remains_symmetric_and_does_not_close_roof(flip, angle):
    p = shell_with_front_lip()
    if flip:
        p[:, :2] *= -1
    p = rotate_wl(p, np.deg2rad(angle)) + [2., 5., -3.]
    original = p.copy()
    result = complete_rectangular_wall_reference(p, Config())
    control = complete_rectangular_wall_reference(p, Config(carriage_wall_reference_front_lip_enable=False))
    assert result.box == control.box
    assert result.box.height == pytest.approx(result.metadata['observedHeightM'])
    lip = result.metadata['frontLip']
    assert lip['completed'], lip
    assert lip['depthM'] == pytest.approx(.28, abs=.03)
    assert lip['inwardSign'] == (1 if flip else -1)
    assert not result.metadata['topGenerated'] and not lip['roofClosed']
    assert len(result.points) == len(control.points) + lip['synthesizedPoints']
    assert len(result.points) == sum(item['synthesizedPoints'] for item in result.metadata['walls'].values())
    np.testing.assert_array_equal(result.points[:len(control.points)], control.points)
    q = rotate_wl(result.points, -np.deg2rad(result.metadata['yawDeg']))
    extra = q[len(control.points):]
    distance = (extra[:, 1] - lip['frontL']) * lip['inwardSign']
    assert np.all(distance > 0) and np.max(distance) == pytest.approx(lip['depthM'])
    assert np.min(extra[:, 2]) >= lip['innerDropLowerHeightM'] - 1e-10
    assert np.max(extra[:, 2]) <= result.box.h_range[1]
    mirror = q.copy()
    mirror[:, 0] = sum(result.box.w_range) - mirror[:, 0]
    assert cKDTree(q).query(mirror)[0].max() < 1e-10
    np.testing.assert_array_equal(p, original)


@pytest.mark.parametrize('case', ['absent', 'narrow', 'no_front', 'disabled', 'outside_profile', 'rear_only'])
def test_front_lip_requires_own_evidence_and_does_not_affect_other_vehicles(case):
    p = shell_with_front_lip()
    if case == 'absent':
        p = np.vstack([p[:13700], p[-1500:]])
    elif case == 'narrow':
        p[13700:14500, 0] *= .1
    elif case == 'no_front':
        p = p[:-1500]
    elif case == 'outside_profile':
        p[:, 1] *= 1.5
    elif case == 'rear_only':
        p[13700:14500, 1] = 6 - p[13700:14500, 1]
    cfg = Config(carriage_wall_reference_front_lip_enable=case != 'disabled')
    result = complete_rectangular_wall_reference(p, cfg)
    control = complete_rectangular_wall_reference(p, Config(carriage_wall_reference_front_lip_enable=False))
    assert result.box is not None
    assert not result.metadata['frontLip']['completed']
    np.testing.assert_array_equal(result.points, control.points)


def test_front_lip_reconstructs_supported_inner_downturn_without_full_height_inner_wall():
    from carpointcloud_analyzer.domain.front_lip import complete_front_lip
    from carpointcloud_analyzer.domain.models import BoxCarriageBox
    rng = np.random.default_rng(90)
    box = BoxCarriageBox(6., 0., (-1.25, 1.25), (1.2, 4.1), 1000, 1000)
    points = np.column_stack([rng.uniform(-1.2, 1.2, 1000),
                              6 - rng.uniform(.22, .28, 1000),
                              np.where(np.arange(1000) % 5 == 0, 4.02, 4.07)])
    generated, metadata = complete_front_lip(points, box, False, True, np.linspace(-1.25, 1.25, 65), .04)
    assert metadata['completed'] and metadata['innerDropGenerated']
    assert metadata['innerDropLowerHeightM'] == pytest.approx(4.02)
    assert generated[:, 2].min() == pytest.approx(4.02)
    assert generated[:, 2].max() == pytest.approx(4.1)
    assert not metadata['roofClosed']
