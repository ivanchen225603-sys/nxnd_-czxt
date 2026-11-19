"""走真实 pipeline → exporter → service 包装，避免只用假响应掩盖漏字段。"""

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
    VehicleDimensions,
)
from carpointcloud_analyzer.application.service import AnalysisService


@pytest.mark.parametrize(
    "load_status,inner_faces,cab_valid,expected_valid",
    [
        ("Empty", True, True, True),
        ("Loaded", True, True, False),
        ("Uncertain", True, True, False),
        ("Empty", False, True, False),
        ("Empty", True, False, False),
        ("Loaded", False, True, False),
        ("Loaded", True, False, False),
    ],
)
def test_actual_pipeline_preserves_guarded_outputs(
    monkeypatch, load_status, inner_faces, cab_valid, expected_valid
):
    points = np.array([[0., 0., 0.], [1., 2., 3.]])
    candidate = VehicleDimensions(10., 2.5, 3., 6., 2.2, 1.2)
    split = CarriageSplitResult(
        points, 5., 0., 7., 0.,
        boundaries_are_inner_faces=inner_faces,
        cab_measurement_method="fixed_height",
        cab_measurement_head_y=7. if cab_valid else None,
        cab_measurement_height=1.4,
        cab_measurement_half_band=0.05,
        cab_measurement_point_count=10 if cab_valid else 0,
    )
    analyzer = pipeline.PointCloudAnalyzer(Config(save_clean_pointcloud=False))
    monkeypatch.setattr(analyzer.loader, "load", lambda path: points)
    monkeypatch.setattr(analyzer, "_level_scene", lambda p: (p, None, None, False))
    monkeypatch.setattr(analyzer.preprocessor, "process", lambda p: SimpleNamespace(
        primary_vehicle_points=p, ground_plane=None,
    ))
    transform = CoordinateTransform(np.zeros(3), np.eye(3))
    monkeypatch.setattr(analyzer.geometry, "process", lambda *args: (points, transform))
    monkeypatch.setattr(analyzer.local_splitter, "split", lambda p: split)
    monkeypatch.setattr(analyzer.dim_calc, "measure_cab_to_front_wall_at_height", lambda *args: None)
    monkeypatch.setattr(analyzer.dim_calc, "calculate", lambda *args, **kwargs: candidate)
    profile = SimpleNamespace(interior_w_range=(-1., 1.), floor_z=0., wall_top=2.,
                              measurement_confidence=1.)
    monkeypatch.setattr(analyzer.floor_analyzer, "analyze", lambda *args, **kwargs: None)
    monkeypatch.setattr(analyzer.carriage_analyzer, "analyze", lambda p: profile)
    monkeypatch.setattr(analyzer, "_assess_load", lambda *args, **kwargs:
                        LoadStatusResult(load_status, 0.9, "test", {}))
    # 这一测试无合成点；重点是 exporter 的二次防御不能被返回值绕过。
    completion = Mock(return_value=SimpleNamespace(
        points=np.empty((0, 3)), metadata={
            "completed": False,
            "rearWall": {"reason": "ambiguous_end_faces",
                         "model": {"reason": "ambiguous_end_faces"}},
        },
    ))
    monkeypatch.setattr(pipeline, "complete_empty_carriage_walls", completion)
    monkeypatch.setattr(analyzer.local_rib_detector, "detect", lambda *args: [])
    monkeypatch.setattr(pipeline, "measure_final_inner_end_faces", lambda *args, **kwargs: None)
    # 空车检测拉筋需要 interior_w_range；不引入无关的剖面伪造数据。
    if load_status == "Empty" and inner_faces:
        monkeypatch.setattr(pipeline.ResultExporter, "_build_carriage_profile", lambda *args: None)

    service = AnalysisService()
    try:
        response = service._run_single(analyzer, "contract.pcd")
    finally:
        service.shutdown()

    outputs = response["outputs"]
    assert outputs["measurementStatus"]["valid"] is expected_valid
    assert outputs["loadStatus"]["status"] == load_status
    if load_status == 'Uncertain':
        assert outputs['dimensions']['vehicle'] == candidate.to_dict()['vehicle']
        assert response['integrationPayloads']['vehicleDimensions']['detectStatus'] == 'success'
        assert response['integrationPayloads']['vehicleDimensions']['actualRibCount'] is None
    else:
        assert outputs["dimensions"] == (candidate.to_dict() if expected_valid else None)
        assert ("vehicleDimensions" in response["integrationPayloads"]) is expected_valid
    assert "wallCompletion" in outputs
    if not expected_valid:
        if load_status == 'Empty':
            assert outputs['wallCompletion']['completed'] is False
            if inner_faces:
                assert outputs['wallCompletion']['rearWall']['reason'] == 'ambiguous_end_faces'
            else:
                assert outputs['wallCompletion']['reason'] == 'boundaries_unverified'
        else:
            assert outputs["wallCompletion"] == {
                "completed": False, "reason": "measurement_not_valid",
            }
        if inner_faces:
            assert outputs["measurementStatus"]["wallCompletionDiagnostic"]["rearWall"]["reason"] == "ambiguous_end_faces"
    # 补点已移到判空之前，Loaded/Uncertain 也先生成独立候选；导出仍不得放行。
    assert completion.call_count == int(inner_faces)
    assert 'calculationPolicy' not in outputs['loadStatus']
    assert 'calculationPolicy' not in outputs['measurementStatus']
