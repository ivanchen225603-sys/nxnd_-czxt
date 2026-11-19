"""局部测量走真实 pipeline/exporter/service；已算项可入库但不建完整档案。"""

from datetime import datetime
import json
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
    VehicleDimensions,
)
from carpointcloud_analyzer.application.service import AnalysisService


@pytest.mark.parametrize(
    "status,inner_faces,cab_valid,length_missing,implausible,reason",
    [
        ("Loaded", True, True, False, False, "vehicle_not_empty"),
        ("Uncertain", True, True, False, False, "load_status_uncertain"),
        ("Empty", False, True, False, False, "inner_wall_boundaries_unverified"),
        ("Empty", True, False, False, False, "cab_front_at_measurement_height_unavailable"),
        ("Empty", True, True, True, False, "longitudinal_inner_edges_unavailable"),
        ("Empty", True, True, False, True, "ok"),
        ("Empty", True, True, False, False, "insufficient_supported_side_sections"),
    ],
)
def test_partial_pipeline_persists_only_empty_plausible_fields_without_archiving(
    monkeypatch, status, inner_faces, cab_valid, length_missing, implausible, reason,
):
    """空车逐项可空入库；载货/异常仍拦截；部分结果不生成永久档案。"""
    points = np.array([[0., 0., 0.], [1., 2., 3.]])
    candidate = VehicleDimensions(
        10., 2.5, 3., 6., 8. if implausible else 2.2, 1.2,
        carriage_outer_width=2.5,
        width_measurement={
            "valid": True, "widthM": 2.2,
            "adoptedSource": "observed_inner_side_faces",
        },
    )
    height_missing = reason == "insufficient_supported_side_sections"
    if height_missing:
        candidate.carriage_height = float('nan')
        candidate.height_measurement = {"valid": False, "heightM": None, "reason": reason}
    split = CarriageSplitResult(
        points, 5., 0., 7., 0.,
        boundaries_are_inner_faces=inner_faces,
        cab_measurement_method="fixed_height",
        cab_measurement_head_y=7. if cab_valid else None,
        cab_measurement_height=1.4,
        cab_measurement_half_band=.05,
        cab_measurement_point_count=10 if cab_valid else 0,
    )
    if length_missing:
        split.longitudinal_measurement = LongitudinalMeasurement(
            None, 5., None, {
                "valid": False, "reason": "lower_inner_edges_unavailable",
                "definition": "same_side_lower_band_longitudinal_projection",
            },
        )
    if height_missing:
        split.longitudinal_measurement = LongitudinalMeasurement(
            0., 5., (0., 0., -1.2), {"valid": True, "source": "observed_inner_faces",
                                    "definition": "main_side_wall_body_length"},
        )
    analyzer = pipeline.PointCloudAnalyzer(Config(
        save_clean_pointcloud=False, carriage_wall_reference_enable=False,
    ))
    monkeypatch.setattr(analyzer.loader, "load", lambda path: points)
    monkeypatch.setattr(analyzer, "_level_scene", lambda p: (p, None, None, False))
    monkeypatch.setattr(analyzer.preprocessor, "process", lambda p: SimpleNamespace(
        primary_vehicle_points=p, ground_plane=None,
    ))
    transform = CoordinateTransform(np.zeros(3), np.eye(3))
    monkeypatch.setattr(analyzer.geometry, "process", lambda *args: (points, transform))
    monkeypatch.setattr(analyzer.local_splitter, "split", lambda p: split)
    monkeypatch.setattr(analyzer.dim_calc, "measure_cab_to_front_wall_at_height", lambda *args: None)
    monkeypatch.setattr(analyzer.dim_calc, "calculate", lambda *args, ground_plane_local=None, synthetic_point_count=0: candidate)
    # 剖面量值是几何替身；编排、导出、局部结果和服务持久化判断均执行产品代码。
    profile = SimpleNamespace(interior_w_range=(-1., 1.), wall_depth=1.2,
                              floor_z=0., wall_top=1.2, valid=True)
    monkeypatch.setattr(analyzer.carriage_analyzer, "analyze", lambda p: profile)
    monkeypatch.setattr(pipeline.ResultExporter, "_build_carriage_profile", lambda *args: None)
    monkeypatch.setattr(pipeline, "measure_final_inner_end_faces",
                        lambda *args, **kwargs: split.longitudinal_measurement)
    monkeypatch.setattr(analyzer, "_assess_load", lambda *args, **kwargs: LoadStatusResult(status, .9, "fixture", {}))
    monkeypatch.setattr(analyzer, "_measure_current_floor", lambda *args, **kwargs: None)
    monkeypatch.setattr(pipeline, "complete_empty_carriage_walls", lambda *args: SimpleNamespace(
        points=np.empty((0, 3)), metadata={"completed": False},
    ))
    monkeypatch.setattr(analyzer.local_rib_detector, "detect", lambda *args: [])
    store = SimpleNamespace(enabled=True, get_workflow_plate=lambda workflow: None, persist_vehicle_dimensions=Mock(
        return_value=SimpleNamespace(to_dict=lambda: {"recordId": 1})))
    archive = SimpleNamespace(enabled=True, stage=Mock(), finalize=Mock())
    svc = AnalysisService(dimension_store=store, empty_profile_storage=archive)
    try:
        response = svc._run_single(
            analyzer, "partial-contract.pcd", workflow_id="partial-workflow",
            pair_id="partial-pair", captured_at=datetime(2026, 9, 26, 11, 26, 27),
            empty_profile_source="partial-contract.pcd",
        )
    finally:
        svc.shutdown()

    assert response["success"] is True
    assert response["workflowId"] == "partial-workflow"
    persists = status != "Loaded"
    if persists:
        payload = response["integrationPayloads"]["vehicleDimensions"]
        assert payload["detectStatus"] == "success"
        assert "persistence" in response
        store.persist_vehicle_dimensions.assert_called_once()
    else:
        assert response["integrationPayloads"] == {}
        assert "persistence" not in response
        store.persist_vehicle_dimensions.assert_not_called()
    archive.stage.assert_not_called()
    archive.finalize.assert_not_called()

    outputs = response["outputs"]
    assert (outputs["dimensions"] is not None) is persists
    assert outputs["loadStatus"]["status"] == status
    measurement = outputs["measurementStatus"]
    assert measurement["valid"] is implausible
    assert measurement["reason"] == reason
    assert measurement["calculationSucceeded"] is persists
    if implausible:
        # 经验范围不再使已算出的整条记录丢失，也不产生待审核状态。
        assert response["integrationPayloads"]["vehicleDimensions"]["dimensions"]["trailerWidth"] == 800.
        assert "reviewStatus" not in measurement
        assert "measurementConfidence" not in measurement
        return
    review = outputs["reviewMeasurements"]
    assert review["partial"] is True
    assert review["calculationSucceeded"] is True
    assert review["dimensions"]["vehicle"] == {"length": 10., "width": 2.5, "height": 3.}
    assert review["dimensions"]["carriage"]["length"] == (5. if height_missing else None)
    assert review["dimensions"]["carriage"]["width"] == 2.2
    assert review["trailerBottomToGroundHeight"] is None
    assert review["wireCount"] is None
    assert review["fields"]["wireCount"]["status"] == "unavailable"
    assert 'source' not in review["fields"]["dimensions.vehicle.height"]
    if height_missing:
        json.dumps(response, allow_nan=False)
        assert review["dimensions"]["carriage"]["height"] is None
        assert review["fields"]["dimensions.carriage.height"]["reason"] == reason
        dimensions = payload["dimensions"]
        assert dimensions["trailerBodyHeight"] is None
        assert dimensions["trailerWidth"] == 220.
        assert dimensions["trailerLength"] == 500.
    if not inner_faces or not cab_valid:
        assert review["cabToTrailerFrontDist"] is None
    if status != "Empty" or not inner_faces:
        assert review["ribCount"] is None
        assert review["ribs"] == []
