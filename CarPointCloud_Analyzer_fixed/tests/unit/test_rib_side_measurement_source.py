"""真实普通管线中的最终云精定位契约。"""

from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from carpointcloud_analyzer.application import pipeline
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.geometry import CoordinateTransform
from carpointcloud_analyzer.domain.models import (
    CarriageSplitResult,
    LoadStatusResult,
    LongitudinalMeasurement,
    RibInfo,
    VehicleDimensions,
)


def _transform():
    yaw, tilt = .63, -.37
    rz = np.array([[np.cos(yaw), -np.sin(yaw), 0.],
                   [np.sin(yaw), np.cos(yaw), 0.], [0., 0., 1.]])
    ry = np.array([[np.cos(tilt), 0., np.sin(tilt)], [0., 1., 0.],
                   [-np.sin(tilt), 0., np.cos(tilt)]])
    return CoordinateTransform(np.array([8.3, -2.7, 5.1]), rz @ ry)


def _arrange(monkeypatch, *, raw_pair=True, status='Empty', inner=True,
             preview=False, measurement_valid=True):
    transform = _transform()
    local = np.array([
        [.1, -2., .3], [-.9, 1., .3], [.9, 2., .4],
        [.67, 3.1, 1.3], [-.41, 6.2, 1.7], [.2, 10., .3],
    ])
    source = transform.to_source(local)
    # 原始双云独有marker：一枚在车内，一枚超出测长的W裁剪带；精定位仍接收完整副本。
    raw_local = np.vstack([local, [.43, 2.34, 1.76], [2.21, 2.56, 1.31]])
    raw_source = transform.to_source(raw_local) if raw_pair else None
    filtered = local[[1, 2]].copy()
    synthetic = np.array([[999., 2., 999.], [888., 4., 888.]])
    split = CarriageSplitResult(filtered, 8., 0., 10., -2.,
                                boundaries_are_inner_faces=inner)
    profile = SimpleNamespace(interior_w_range=(-1., 1.), floor_z=0., wall_top=2.)
    source_name = 'current_split_carriage_points'
    measured = LongitudinalMeasurement(
        .1 if measurement_valid else None, 7.9, (0., 0., -1.12),
        # 此来源是管线原有注记，预设同值以单独检查后续replace不改变原对象。
        {'selectedSide': 'left', 'observedBaseSource': source_name,
         'inputSource': 'current_final_carriage_cloud'},
    )
    original_measurement = deepcopy(measured)
    detected = [
        RibInfo(1, 6.1, 0., 1.6, (-.5, .5), (1.5, 1.7), 80, 1., .2),
        RibInfo(2, 2.6, 0., 1.6, (-.5, .5), (1.5, 1.7), 70, 1., .2),
    ]
    original_ribs = deepcopy(detected)
    evidence = [
        {'ribId': 1, 'valid': True, 'positionM': 6.12, 'extrapolationM': .1},
        {'ribId': 2, 'valid': False, 'positionM': None, 'reason': 'test_missing'},
    ]
    events = []
    seen_measurements = []
    analyzer = pipeline.PointCloudAnalyzer(Config(save_clean_pointcloud=False))
    geometry = Mock(return_value=(filtered, transform))
    monkeypatch.setattr(analyzer.geometry, 'process', geometry)
    monkeypatch.setattr(analyzer.local_splitter, 'split', lambda pts: split)
    monkeypatch.setattr(analyzer.dim_calc, 'measure_cab_to_front_wall_at_height', Mock())
    monkeypatch.setattr(analyzer.carriage_analyzer, 'analyze', lambda pts: profile)
    length = Mock(return_value=measured)
    monkeypatch.setattr(pipeline, 'measure_final_inner_end_faces', length)

    def calculate(*args, ground_plane_local=None, synthetic_point_count=0):
        events.append('dimensions')
        seen_measurements.append(args[2].longitudinal_measurement)
        return VehicleDimensions(12., 2.4, 2.8, 7.8, 2.2, 1.9)

    dimensions = Mock(side_effect=calculate)
    monkeypatch.setattr(analyzer.dim_calc, 'calculate', dimensions)

    def assess(*args, **kwargs):
        events.append('assess_load')
        return LoadStatusResult(status, 1., 'test', {})

    monkeypatch.setattr(analyzer, '_assess_load', assess)
    monkeypatch.setattr(analyzer, '_measure_current_floor', lambda *args, **kwargs: None)

    def complete(*args):
        events.append('completion')
        return SimpleNamespace(points=synthetic, metadata={'completed': True})

    completion = Mock(side_effect=complete)
    monkeypatch.setattr(pipeline, 'complete_empty_carriage_walls', completion)

    def detect(*args):
        events.append('detect')
        return detected

    detector = Mock(side_effect=detect)
    monkeypatch.setattr(analyzer.local_rib_detector, 'detect', detector)

    def refine(*args, **kwargs):
        events.append('refine')
        assert args[1] is detected
        assert args[2] is measured
        return evidence

    refiner = Mock(side_effect=refine)
    monkeypatch.setattr(pipeline, 'refine_rib_side_positions', refiner)
    exporter = Mock(return_value=dict(integrationPayloads={}, dimensions=None,
                                     measurementStatus={}, loadStatus={}, wallCompletion={}))
    monkeypatch.setattr(pipeline.ResultExporter, 'build', exporter)
    scene = pipeline._SceneContext(
        SimpleNamespace(primary_vehicle_points=source), None, None, None, False,
        raw_source, np.array([0., 0., 1.]), 'test',
    )
    before_source = source.copy()
    before_raw = None if raw_source is None else raw_source.copy()
    result = analyzer._analyze_standard(scene, pipeline._AnalysisMode('test.pcd', preview, False, False))
    return SimpleNamespace(**locals())


@pytest.mark.parametrize('raw_pair', [True, False])
def test_refiner_uses_final_cloud_and_preserves_input(monkeypatch, raw_pair):
    state = _arrange(monkeypatch, raw_pair=raw_pair)
    state.refiner.assert_called_once()
    retained = state.local[[3, 4]]
    expected_source = np.vstack((state.filtered, state.synthetic, retained))
    np.testing.assert_allclose(state.refiner.call_args.args[0], expected_source, atol=1e-12)
    assert state.refiner.call_args.args[3:] == (0., 2.)
    assert state.refiner.call_args.kwargs == {'final_cloud': True}

    # 新局部精定位不能反向扩大发现点集、重新检测根数或改变旧杆的ID/轴。
    assert state.detector.call_count == 2
    np.testing.assert_allclose(state.detector.call_args.args[0], expected_source, atol=1e-12)
    assert state.detector.call_args.args[1:] == (0., 8., (-1., 1.))
    assert state.detected == state.original_ribs
    assert [item.rib_id for item in state.exporter.call_args.args[0]] == [1, 2]
    assert len(state.exporter.call_args.args[0]) == 2

    # 几何建系仍用场景点，后续检测与尺寸共用最终云。
    state.geometry.assert_called_once()
    np.testing.assert_allclose(state.geometry.call_args.args[0], state.source, atol=1e-12)
    np.testing.assert_allclose(
        state.dimensions.call_args.args[0],
        np.vstack((state.filtered, retained, state.synthetic)), atol=1e-12)
    assert state.events == ['completion', 'detect', 'detect', 'assess_load', 'dimensions', 'refine']
    assert state.result['syntheticPointCount'] == len(state.synthetic)

    # 把证据接到replace副本；原measurement对象及其元数据不能被副作用改写。
    exported = state.exporter.call_args.args[2].longitudinal_measurement
    assert state.seen_measurements == [state.measured]
    assert exported is not state.measured
    assert exported.rib_side_projections is state.evidence
    assert (exported.rear_l, exported.front_l, exported.side_plane) == (
        state.measured.rear_l, state.measured.front_l, state.measured.side_plane)
    assert state.measured == state.original_measurement
    assert state.measured.rib_side_projections is None
    np.testing.assert_array_equal(state.source, state.before_source)
    if raw_pair:
        np.testing.assert_array_equal(state.raw_source, state.before_raw)
        assert not np.any(np.isclose(state.refiner.call_args.args[0][:, 0], 2.21))
        assert not np.any(np.isclose(state.length.call_args.args[0][:, 0], 2.21))
    np.testing.assert_allclose(state.length.call_args.args[0], expected_source, atol=1e-12)
    np.testing.assert_array_equal(
        state.length.call_args.args[0][len(state.filtered):len(state.filtered) + len(state.synthetic)],
        state.synthetic)
    assert state.length.call_args.args[1:] == (0., 8., 0., 2., None)


@pytest.mark.parametrize('status, inner, preview', [
    ('Empty', True, True), ('Loaded', True, False),
    ('Uncertain', True, False), ('Empty', False, False),
])
def test_preview_and_nonempty_paths_do_not_publish_ribs(monkeypatch, status, inner, preview):
    state = _arrange(monkeypatch, status=status, inner=inner, preview=preview)
    state.refiner.assert_not_called()
    assert state.detector.call_count == (0 if preview or not inner else 2)
    assert state.detected == state.original_ribs
    assert state.measured == state.original_measurement
    if preview:
        state.length.assert_not_called()
        state.exporter.assert_not_called()
        assert state.result['referenceOnly'] is True
        assert state.result['measuredPointCount'] == len(state.filtered)
        assert state.result['syntheticPointCount'] == len(state.synthetic)
        np.testing.assert_allclose(state.result['coordinates'],
                                   state.transform.to_source(np.vstack([state.filtered, state.synthetic])),
                                   atol=1e-12)
    else:
        assert state.completion.call_count == int(inner)
        assert state.result['syntheticPointCount'] == 0
        assert state.exporter.call_args.args[0] == []


def test_missing_longitudinal_endpoints_do_not_trigger_refinement(monkeypatch):
    state = _arrange(monkeypatch, measurement_valid=False)
    assert state.detector.call_count == 2
    state.refiner.assert_not_called()
    assert state.exporter.call_args.args[2].longitudinal_measurement is state.measured
    assert state.measured == state.original_measurement
    assert state.detected == state.original_ribs
