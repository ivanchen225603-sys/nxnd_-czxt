"""拉筋检测与尺寸读取同一份最终车厢云。"""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from carpointcloud_analyzer.application import pipeline
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.geometry import CoordinateTransform, LocalRibDetector
from carpointcloud_analyzer.domain.models import CarriageSplitResult, LoadStatusResult


@pytest.mark.parametrize('status,inner,preview', [
    ('Empty', True, False), ('Empty', False, False),
    ('Loaded', True, False), ('Uncertain', True, False), ('Empty', True, True),
])
def test_standard_pipeline_detects_ribs_from_final_cloud(monkeypatch, status, inner, preview):
    angle = .7
    rotation = np.array([[np.cos(angle), -np.sin(angle), 0.],
                         [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
    transform = CoordinateTransform(np.array([8., -2., 5.]), rotation)
    local = np.array([[0., -2., .3], [0., 1., .3], [.6, 3., .8], [0., 6., .3]])
    source = transform.to_source(local)
    before = source.copy()
    filtered = local[[1]].copy()  # 几何去噪删除的稀疏杆件点仍必须供拉筋检测使用。
    split = CarriageSplitResult(filtered, 5., 0., 6., -2., boundaries_are_inner_faces=inner)
    analyzer = pipeline.PointCloudAnalyzer(Config(save_clean_pointcloud=False))
    geometry = Mock(return_value=(filtered, transform))
    monkeypatch.setattr(analyzer.geometry, 'process', geometry)
    monkeypatch.setattr(analyzer.local_splitter, 'split', lambda pts: split)
    measure = Mock(return_value=SimpleNamespace())
    monkeypatch.setattr(analyzer.dim_calc, 'calculate', measure)
    monkeypatch.setattr(analyzer.dim_calc, 'measure_cab_to_front_wall_at_height', Mock())
    monkeypatch.setattr(analyzer.carriage_analyzer, 'analyze', lambda pts: SimpleNamespace(
        interior_w_range=(-1., 1.), floor_z=0., wall_top=2.))
    monkeypatch.setattr(analyzer, '_assess_load', lambda *args, **kwargs: LoadStatusResult(status, 1., 'test', {}))
    monkeypatch.setattr(analyzer, '_measure_current_floor', lambda *args, **kwargs: None)
    monkeypatch.setattr(pipeline, 'complete_empty_carriage_walls', lambda *args: SimpleNamespace(
        points=np.array([[999., 2., 999.]]), metadata={'completed': True}))
    detect = Mock(return_value=[])
    monkeypatch.setattr(analyzer.local_rib_detector, 'detect', detect)
    monkeypatch.setattr(pipeline.ResultExporter, 'build', lambda *args, **kwargs: dict(
        integrationPayloads={}, dimensions=None, measurementStatus={}, loadStatus={}, wallCompletion={}))
    scene = SimpleNamespace(preprocessed=SimpleNamespace(primary_vehicle_points=source),
                            vertical_reference=np.array([0., 0., 1.]), vertical_source='test',
                            site_frame=None, height_plane_analysis=None, height_plane_source=None,
                            height_plane_calibrated=False)
    mode = pipeline._AnalysisMode('test.pcd', preview, False, False)
    analyzer._analyze_standard(scene, mode)
    assert geometry.call_count == 1
    if inner:
        np.testing.assert_allclose(
            measure.call_args.args[0],
            np.vstack((filtered, local[[2]], [[999., 2., 999.]]))
            if not preview else np.vstack((filtered, [[999., 2., 999.]])), atol=1e-12)
    else:
        np.testing.assert_array_equal(measure.call_args.args[0], filtered)
    np.testing.assert_array_equal(source, before)
    if inner and not preview:
        assert detect.call_count == 2
        np.testing.assert_allclose(detect.call_args.args[0],
                                   np.vstack((filtered, [[999., 2., 999.]],
                                              local[[2]])), atol=1e-12)
        assert detect.call_args.args[1:] == (0., 5., (-1., 1.))
    else:
        detect.assert_not_called()


@pytest.mark.parametrize('count', [0, 2, 5])
def test_existing_rib_geometry_guards_count_measured_transverse_members(count):
    rng = np.random.default_rng(20260910)
    floor = np.column_stack([rng.uniform(-1.1, 1.1, 20000),
                             rng.uniform(0., 13., 20000), rng.normal(0., .004, 20000)])
    rods = [np.column_stack([np.linspace(-1.05, 1.05, 400),
                             np.full(400, position), np.full(400, 1.)])
            + rng.normal(0., .009, (400, 3)) for position in [2., 4., 6., 8., 10.][:count]]
    # 墙面和窄幅噪声团不能增加杆件数量；保留原检测阈值。
    wall = np.column_stack([np.full(1000, 1.2), rng.uniform(0., 13., 1000), rng.uniform(.2, 1.3, 1000)])
    noise = rng.uniform([-.1, 10.9, .3], [.1, 11.1, 1.2], (100, 3))
    points = np.vstack([floor, wall, noise, *rods])
    ribs = LocalRibDetector(Config()).detect(points, 0., 13., (-1.2, 1.2))
    assert len(ribs) == count
    assert [r.y_position for r in ribs] == pytest.approx([2., 4., 6., 8., 10.][:count], abs=.03)


@pytest.mark.parametrize('middle_present', [True, False])
def test_rib_count_uses_individual_geometry_not_other_rods_density(middle_present):
    """稀疏杆有实测支撑就保留；缺杆时不能根据相邻间距补出一根。"""
    rng = np.random.default_rng(20261104)
    floor = np.column_stack([rng.uniform(-1.1, 1.1, 20000),
                             rng.uniform(0., 13., 20000), rng.normal(0., .004, 20000)])
    positions = [1.8, 3.9, 6.2, 8.4, 10.7]
    counts = [400, 600, 210, 800, 400]
    rods = []
    expected = []
    for position, count in zip(positions, counts):
        if position == 6.2 and not middle_present:
            continue
        rods.append(np.column_stack([np.linspace(-1.05, 1.05, count),
                                     np.full(count, position), np.full(count, 1.)])
                    + rng.normal(0., .004, (count, 3)))
        expected.append(position)
    ribs = LocalRibDetector(Config()).detect(np.vstack([floor, *rods]), 0., 13., (-1.2, 1.2))
    assert len(ribs) == len(expected)
    assert [rib.y_position for rib in ribs] == pytest.approx(expected, abs=.03)


def test_transverse_planar_patch_is_not_a_rib():
    """切窗内横跨车宽的片面不能因为点多或能拟合直线就计作拉筋。"""
    rng = np.random.default_rng(20261104)
    floor = np.column_stack([rng.uniform(-1.1, 1.1, 20000),
                             rng.uniform(0., 13., 20000), rng.normal(0., .004, 20000)])
    width, height = np.meshgrid(np.arange(-1.05, 1.051, .02), np.arange(.7, 1.301, .02))
    plane = np.column_stack([width.ravel(), np.full(width.size, 6.), height.ravel()])
    ribs = LocalRibDetector(Config()).detect(np.vstack([floor, plane]), 0., 13., (-1.2, 1.2))
    assert ribs == []


def test_five_observed_ribs_survive_sparse_floor_sampling():
    """原失败点云原样冻结：底板以上占比51.93%，五根杆仍有各自几何支撑。"""
    with np.load(Path(__file__).parents[1] / 'fixtures' / 'five_ribs_floor_visibility.npz') as fixture:
        points = fixture['points']
    detector = LocalRibDetector(Config())
    rear, front = -7.800460045679999, 2.199540408661578
    width = (-1.0518718134911196, 1.048948130141754)
    ribs = detector.detect(points, rear, front, width)
    assert len(ribs) == 5
    assert [r.y_position for r in ribs] == pytest.approx(
        np.array([1.5, 3.1, 5.3, 7., 8.6]) + rear, abs=.03)


def test_extended_wall_bridge_keeps_sparse_rib_returns_in_final_cloud(monkeypatch):
    sparse = np.array([[0., 3., 1.]])
    filtered = np.array([[0., 1., 0.], [.2, 4., 0.]])
    raw = np.vstack((filtered, sparse))
    split = CarriageSplitResult(filtered, 5., 0., 7., 0., boundaries_are_inner_faces=True)
    analyzer = pipeline.PointCloudAnalyzer(Config(save_clean_pointcloud=False))
    transform = CoordinateTransform(np.zeros(3), np.eye(3))
    monkeypatch.setattr(analyzer.geometry, 'process', lambda *args: (filtered, transform))
    monkeypatch.setattr(analyzer.local_splitter, 'split', lambda *args: split)
    monkeypatch.setattr(pipeline, 'cab_requires_heading_reversal', lambda *args: False)
    monkeypatch.setattr(analyzer.dim_calc, 'measure_cab_to_front_wall_at_height', Mock())
    monkeypatch.setattr(analyzer.carriage_analyzer, 'analyze', lambda *args: SimpleNamespace(
        interior_w_range=(-1., 1.), floor_z=0., wall_top=2.))
    calls = []

    def complete(*args, **kwargs):
        calls.append(kwargs)
        return SimpleNamespace(points=np.array([[.9, 2., 1.]]), metadata={
            'completed': True,
            'rearWall': {'model': {'valid': True}},
            'frontWall': {'model': {'valid': True}},
            'cornerSeams': [{'longGapHeightRows': 1}] if kwargs.get('bridge_long_side_gaps') else [],
        })

    monkeypatch.setattr(pipeline, 'complete_empty_carriage_walls', complete)
    monkeypatch.setattr(pipeline, 'needs_side_end_bridge', lambda *args: True)
    monkeypatch.setattr(analyzer, '_assess_load', lambda *args, **kwargs: LoadStatusResult('Empty', 1., 'test', {}))
    monkeypatch.setattr(pipeline, 'measure_final_inner_end_faces', lambda *args, **kwargs: None)
    monkeypatch.setattr(analyzer.dim_calc, 'calculate', lambda *args, ground_plane_local=None, synthetic_point_count=0: None)
    monkeypatch.setattr(analyzer, '_measure_current_floor', lambda *args, **kwargs: None)
    detect = Mock(return_value=[])
    monkeypatch.setattr(analyzer.local_rib_detector, 'detect', detect)
    monkeypatch.setattr(pipeline.ResultExporter, 'build', lambda *args, **kwargs: dict(
        integrationPayloads={}, dimensions=None, measurementStatus={},
        loadStatus={}, wallCompletion={}))
    scene = SimpleNamespace(preprocessed=SimpleNamespace(primary_vehicle_points=raw),
                            vertical_reference=np.array([0., 0., 1.]), vertical_source='test',
                            site_frame=None, height_plane_analysis=None,
                            height_plane_source=None, height_plane_calibrated=False)
    analyzer._analyze_standard(scene, pipeline._AnalysisMode('test.pcd', False, False, False))
    assert len(calls) == 2
    assert calls[1]['bridge_long_side_gaps'] is True
    assert detect.call_count == 2
    assert np.any(np.all(np.isclose(detect.call_args.args[0], sparse), axis=1))
