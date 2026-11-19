import asyncio
import json
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import pytest

from carpointcloud_analyzer.application import service
from carpointcloud_analyzer import composition
from carpointcloud_analyzer.infrastructure import cloud_io


def test_analysis_service_defaults_to_four_concurrent_workers():
    svc = service.AnalysisService(analyzer_factory=lambda cfg: None)

    try:
        assert svc.max_concurrency == 4
    finally:
        svc.shutdown()


def test_analyze_sync_does_not_require_result_json_file(tmp_path):
    pcd_path = tmp_path / "scan.pcd"
    pcd_path.write_text("placeholder", encoding="utf-8")

    class FakeAnalyzer:
        def __init__(self, config):
            pass

        def analyze(self, path, *, measurement_points=None):
            return {
                "png": "",
                "json": "",
                "integrationPayloads": {"vehicleDimensions": {"id": 7}},
            }

    svc = service.AnalysisService(analyzer_factory=FakeAnalyzer)
    try:
        result = svc.analyze_sync(service.AnalyzeRequest(str(pcd_path)))
    finally:
        svc.shutdown()

    assert result["outputs"]["json"] == ""
    assert result["outputs"]["png"] == ""
    assert result["integrationPayloads"] == {"vehicleDimensions": {"id": 7}}
    assert not list(tmp_path.glob("*.json"))
    assert not list(tmp_path.glob("*.png"))


def test_health_moves_blocking_store_check_off_event_loop(tmp_path):
    class SlowStore:
        enabled = True

        @staticmethod
        def health():
            time.sleep(0.15)
            return {"enabled": True, "status": "ok"}

    app = composition.create_app(dimension_store=SlowStore())
    endpoint = next(route.endpoint for route in app.routes if route.path == "/health")

    async def verify():
        task = asyncio.create_task(endpoint())
        started = time.perf_counter()
        await asyncio.sleep(0.01)
        event_loop_delay = time.perf_counter() - started
        result = await task
        return event_loop_delay, result

    try:
        delay, result = asyncio.run(verify())
    finally:
        app.state.analysis_service.shutdown()

    assert delay < 0.08
    assert result["persistence"]["status"] == "ok"


def test_analyze_sync_builds_isolated_config_without_png(tmp_path):
    pcd_path = tmp_path / "scan.pcd"
    pcd_path.write_text("placeholder", encoding="utf-8")
    created_configs = []

    class FakeAnalyzer:
        def __init__(self, config):
            created_configs.append(config)

        def analyze(self, path, *, measurement_points=None):
            return {"png": "", "json": "", "integrationPayloads": {}}

    svc = service.AnalysisService(analyzer_factory=FakeAnalyzer)

    try:
        result = svc.analyze_sync(
            service.AnalyzeRequest(
                pcd_path=str(pcd_path),
                camera="CAM-01",
                plate="PLATE-01",
            )
        )
    finally:
        svc.shutdown()

    assert result["success"] is True
    assert result["pcd_path"] == str(pcd_path)
    assert result["outputs"]["json"] == ""
    assert result["elapsed_ms"] >= 0
    assert len(created_configs) == 1
    cfg = created_configs[0]
    assert cfg.camera_identifier == "CAM-01"
    assert cfg.license_plate == "PLATE-01"


def test_analyze_sync_includes_in_memory_integration_payloads(tmp_path):
    pcd_path = tmp_path / "scan.pcd"
    pcd_path.write_text("placeholder", encoding="utf-8")
    class FakeAnalyzer:
        def __init__(self, config):
            pass

        def analyze(self, path, *, measurement_points=None):
            return {
                "png": "",
                "json": "",
                "integrationPayloads": {
                    "vehicleDimensions": {
                        "idempotencyKey": "scan",
                        "cameraIdentifier": "CAM-01",
                    },
                    "coalVolumes": None,
                },
            }

    svc = service.AnalysisService(analyzer_factory=FakeAnalyzer)

    try:
        result = svc.analyze_sync(
            service.AnalyzeRequest(
                pcd_path=str(pcd_path),
                camera="CAM-01",
            )
        )
    finally:
        svc.shutdown()

    assert result["integrationPayloads"]["vehicleDimensions"]["idempotencyKey"] == "scan"
    assert result["integrationPayloads"]["coalVolumes"] is None


def test_analyze_sync_applies_workflow_id_and_captured_at(tmp_path):
    """双雷达其中一台超时后的兜底分析走这条单文件路径，但它仍属于一次来车流程。

    workflowId 必须成为幂等键（否则记录无法回连 WebApi 来车记录），
    getTime 必须用实际采集时刻（否则退化成分析入库时刻，重试时会偏几小时）。
    """
    pcd_path = tmp_path / "scan.pcd"
    pcd_path.write_text("placeholder", encoding="utf-8")
    class FakeAnalyzer:
        def __init__(self, config):
            pass

        def analyze(self, path, *, measurement_points=None):
            return {
                "png": "",
                "json": "",
                "integrationPayloads": {
                    "vehicleDimensions": {
                        "idempotencyKey": "scan",
                        "cameraIdentifier": "CAM_004",
                        "getTime": None,
                    }
                },
            }

    captured_at = datetime(2026, 8, 13, 10, 30, 45)
    svc = service.AnalysisService(analyzer_factory=FakeAnalyzer)
    try:
        result = svc.analyze_sync(
            service.AnalyzeRequest(
                pcd_path=str(pcd_path),
                camera="CAM_004",
                workflow_id="wf_20260813_000001",
                captured_at=captured_at,
            )
        )
    finally:
        svc.shutdown()

    dimensions = result["integrationPayloads"]["vehicleDimensions"]
    assert dimensions["workflowId"] == "wf_20260813_000001"
    assert dimensions["idempotencyKey"] == "wf_20260813_000001"
    assert dimensions["getTime"] == captured_at.isoformat()
    # 回显让 Worker 能发现"分析服务是不认识 workflowId 的旧版本"。
    assert result["workflowId"] == "wf_20260813_000001"

    assert result["outputs"]["json"] == ""
    assert not list(tmp_path.glob("*.json"))


def test_analyze_sync_persists_vehicle_dimensions_when_direct_store_enabled(tmp_path):
    pcd_path = tmp_path / "scan.pcd"
    pcd_path.write_text("placeholder", encoding="utf-8")
    vehicle_payload = {
        "idempotencyKey": "scan",
        "cameraIdentifier": "CAM_004",
        "dimensions": {},
    }
    class FakeAnalyzer:
        def __init__(self, config):
            pass

        def analyze(self, path, *, measurement_points=None):
            return {
                "png": "",
                "json": "",
                "integrationPayloads": {"vehicleDimensions": vehicle_payload},
            }

    class FakeStore:
        enabled = True

        def __init__(self):
            self.payloads = []

        def persist_vehicle_dimensions(self, payload):
            self.payloads.append(payload)
            return type(
                "Result",
                (),
                {"to_dict": lambda self: {"status": "inserted", "recordId": 7}},
            )()

        def health(self):
            return {"enabled": True, "status": "ok"}

    store = FakeStore()
    svc = service.AnalysisService(
        analyzer_factory=FakeAnalyzer,
        dimension_store=store,
    )
    try:
        result = svc.analyze_sync(service.AnalyzeRequest(str(pcd_path)))
    finally:
        svc.shutdown()

    assert store.payloads == [vehicle_payload]
    assert result["persistence"] == {"status": "inserted", "recordId": 7}


@pytest.mark.parametrize(
    ('load_status', 'measurement_valid'),
    [('Empty', True), ('Empty', False), ('Uncertain', False), ('Loaded', False)],
)
def test_pair_result_archives_only_fused_completed_empty_cloud(
    tmp_path, load_status, measurement_valid,
):
    merged_pcd = tmp_path / "pair_merged.pcd"
    merged_pcd.write_bytes(b"fused-pcd")
    completed_pcd = tmp_path / "pair_completed.pcd"
    completed_pcd.write_bytes(b"completed-fused-pcd")
    class FakeAnalyzer:
        def analyze(self, path, *, measurement_points=None):
            return {
                "png": "",
                "json": "",
                "pcd": str(completed_pcd),
                "pcdPointCount": 99,
                "measurementStatus": {'valid': measurement_valid},
                "loadStatus": {'status': load_status},
                "integrationPayloads": {
                    "vehicleDimensions": {
                        "idempotencyKey": "temporary",
                        "cameraIdentifier": "CAM_004",
                        "licensePlate": "川A12047",
                        "detectStatus": "success",
                        "dimensions": {},
                    }
                },
            }

    class FakeStore:
        enabled = True

        def __init__(self):
            self.calls = []

        def persist_vehicle_dimensions(self, payload, empty_profile=None):
            self.calls.append((payload, empty_profile))
            return type(
                "Result",
                (),
                {
                    "to_dict": lambda self: {
                        "status": "inserted",
                        "recordId": 8,
                        "emptyVehicleProfile": {
                            "updated": True,
                            "pointCloudPath": empty_profile["pointCloudPath"],
                            "previousPointCloudPath": None,
                        } if empty_profile is not None else None,
                    }
                },
            )()

        def get_workflow_plate(self, workflow_id):
            return None

        def health(self):
            return {"enabled": True, "status": "ok"}

    captured_at = datetime(2026, 8, 13, 9, 30, 0)
    store = FakeStore()
    profile_storage = composition.EmptyVehicleProfileStorage(
        str(tmp_path / "empty-profiles")
    )
    svc = service.AnalysisService(
        dimension_store=store,
        empty_profile_storage=profile_storage,
    )
    try:
        result = svc._run_single(
            FakeAnalyzer(),
            str(merged_pcd),
            workflow_id="workflow-008",
            pair_id="pair-008",
            captured_at=captured_at,
            empty_profile_source=str(merged_pcd),
        )
    finally:
        svc.shutdown()

    payload, profile = store.calls[0]
    assert payload["idempotencyKey"] == "workflow-008"
    assert payload["pairId"] == "pair-008"
    assert payload["getTime"] == captured_at.isoformat()
    assert payload['detectStatus'] == 'success'
    if load_status != 'Empty':
        assert profile is None
        assert result["persistence"]["emptyVehicleProfile"] is None
    else:
        assert profile["pointCount"] == 99
        assert Path(profile["pointCloudPath"]).read_bytes() == b"completed-fused-pcd"
        assert merged_pcd.read_bytes() == b"fused-pcd"
        assert result["persistence"]["emptyVehicleProfile"]["updated"] is True


def test_batch_sync_reuses_one_analyzer_for_request(tmp_path):
    paths = []
    for name in ["a.pcd", "b.pcd"]:
        pcd_path = tmp_path / name
        pcd_path.write_text("placeholder", encoding="utf-8")
        paths.append(str(pcd_path))

    created = []
    analyzed = []

    class FakeAnalyzer:
        def __init__(self, config):
            created.append(config)

        def analyze(self, path, *, measurement_points=None):
            analyzed.append(path)
            return {"png": "", "json": "", "integrationPayloads": {}}

    svc = service.AnalysisService(analyzer_factory=FakeAnalyzer)

    try:
        result = svc.analyze_batch_sync(service.BatchAnalyzeRequest(pcd_paths=paths))
    finally:
        svc.shutdown()

    assert result["success"] is True
    assert len(created) == 1
    assert analyzed == paths
    assert [item["pcd_path"] for item in result["results"]] == paths


def test_analyze_pair_sync_keeps_fixed_ab_order_and_stable_merged_path(
    tmp_path, monkeypatch
):
    pcd_a = tmp_path / "camera-a.pcd"
    pcd_b = tmp_path / "camera-b.pcd"
    extrinsic_path = tmp_path / "b-to-a.json"
    for path in (pcd_a, pcd_b, extrinsic_path):
        path.write_text("placeholder", encoding="utf-8")

    loaded_paths = []
    merged_inputs = []
    created_configs = []
    analyzed_paths = []

    class FakeLoader:
        def __init__(self, config):
            created_configs.append(config)

        def load(self, path):
            loaded_paths.append(path)
            marker = 1.0 if Path(path) == pcd_a.resolve() else 2.0
            # 两台原始云都含目标车之外的点；融合替身只选择前三行。
            return np.vstack([np.full((3, 3), marker), np.full((2, 3), 99.)])

    class FakeMergeResult:
        points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        quality_passed = True
        reason = None

        @staticmethod
        def to_dict():
            return {
                "mode": "fused",
                "qualityPassed": True,
                "reason": None,
                "pointsFromA": 3,
                "pointsFromB": 3,
                "pointsFiniteA": 3,
                "pointsFiniteB": 3,
                "pointsFilteredA": 3,
                "pointsFilteredB": 3,
                "pointsCombined": 6,
                "pointsMerged": 2,
                "overlapRmse": 0.012,
                "overlapMatchRatio": 0.9,
                "warnings": [],
            }

    class FakeVehicleMergeResult:
        points = FakeMergeResult.points
        selected_observed_points = np.vstack([
            np.ones((3, 3)), np.tile([12., 2., 2.], (3, 1)),
        ])
        fusion = FakeMergeResult()

        @staticmethod
        def to_dict():
            return {
                "coordinateFrame": "cameraA",
                "points": {"mergedVehicleOnly": 2},
                "temporarySiteFrame": {},
                "environmentFilter": {},
                "fusion": FakeMergeResult.to_dict(),
            }

    class FakeExtrinsic:
        @staticmethod
        def apply(points):
            return points + np.array([10., 0., 0.])

        quality = type("Quality", (), {"trustworthy": True})()
        baseline_residual = 0.01
        weighbridge_plane_a = np.array([0.0, 0.0, 1.0, -0.25])
        empty_lane_baseline = None

        def validate_camera_pair(self, camera_a, camera_b):
            assert (camera_a, camera_b) == ("CAM-A", "CAM-B")

    fake_extrinsic = FakeExtrinsic()

    def fake_merge(points_a, points_b, extrinsic, **kwargs):
        merged_inputs.append((points_a.copy(), points_b.copy(), extrinsic))
        assert kwargs["camera_a"] == "CAM-A"
        assert kwargs["camera_b"] == "CAM-B"
        return FakeVehicleMergeResult()

    def fake_write(path, points):
        path.write_text(f"{len(points)} points", encoding="utf-8")

    class FakeAnalyzer:
        def __init__(self, config):
            created_configs.append(config)

        def analyze(self, path, *, measurement_points=None):
            analyzed_paths.append(path)
            # 内缘收到未体素化但已剔除 99 噪点的目标车回波，而非全场景或两点融合输出。
            np.testing.assert_array_equal(measurement_points[:3], np.ones((3, 3)))
            np.testing.assert_array_equal(measurement_points[3:], np.tile([12., 2., 2.], (3, 1)))
            return {
                "png": "",
                "json": "",
                "integrationPayloads": {
                    "vehicleDimensions": {
                        "idempotencyKey": Path(path).stem,
                        "cameraIdentifier": "CAM-A+CAM-B",
                    }
                },
            }

    monkeypatch.setattr(service, "PointCloudLoader", FakeLoader)
    monkeypatch.setattr(service.Extrinsic, "load", lambda path: fake_extrinsic)
    monkeypatch.setattr(service, "merge_vehicle_clouds", fake_merge)
    monkeypatch.setattr(
        service.AnalysisService, "_write_merged_pcd", staticmethod(fake_write)
    )

    svc = service.AnalysisService(analyzer_factory=FakeAnalyzer)
    request = service.PairAnalyzeRequest(
        pcd_path_a=str(pcd_a),
        camera_a="CAM-A",
        pcd_path_b=str(pcd_b),
        camera_b="CAM-B",
        pair_id="vehicle-pass-001",
        workflow_id="sess-workflow-001",
        extrinsic_path=str(extrinsic_path),
        plate="PLATE-01",
    )
    try:
        first = svc.analyze_pair_sync(request)
        second = svc.analyze_pair_sync(request)
    finally:
        svc.shutdown()

    assert loaded_paths[:2] == [str(pcd_a.resolve()), str(pcd_b.resolve())]
    assert np.all(merged_inputs[0][0][:3] == 1.0)
    assert np.all(merged_inputs[0][1][:3] == 2.0)
    assert np.all(merged_inputs[0][0][3:] == 99.0)
    assert np.all(merged_inputs[0][1][3:] == 99.0)
    assert merged_inputs[0][2] is fake_extrinsic
    assert analyzed_paths[0] == analyzed_paths[1]
    assert Path(analyzed_paths[0]).name.startswith("pair_")
    assert Path(analyzed_paths[0]).name.endswith("_merged.pcd")
    assert first["success"] is True
    assert first["pcd_path"] == second["pcd_path"]
    assert first["pairId"] == "vehicle-pass-001"
    assert first["workflowId"] == "sess-workflow-001"
    assert first["sourcePcdPaths"] == {
        "a": str(pcd_a.resolve()),
        "b": str(pcd_b.resolve()),
    }
    assert first["sourceCameras"]["combined"] == "CAM-A+CAM-B"
    assert first["analysisCamera"] == "CAM-A+CAM-B"
    assert first["fusion"]["pointsMerged"] == 2
    assert first["fusion"]["mode"] == "fused"
    assert first["fusion"]["qualityPassed"] is True
    assert first["outputs"]["json"] == ""
    assert first["outputs"]["png"] == ""
    assert first["integrationPayloads"]["vehicleDimensions"]["pairId"] == (
        "vehicle-pass-001"
    )
    assert first["integrationPayloads"]["vehicleDimensions"]["workflowId"] == (
        "sess-workflow-001"
    )
    assert first["elapsed_ms"] >= first["analysis_elapsed_ms"]
    assert (
        first["integrationPayloads"]["vehicleDimensions"]["idempotencyKey"]
        == "sess-workflow-001"
    )
    assert first["integrationPayloads"]["vehicleDimensions"]["workflowId"] == "sess-workflow-001"
    assert first["integrationPayloads"]["vehicleDimensions"]["pairId"] == "vehicle-pass-001"
    # 最终业务载荷归属固定基准 A，满足 WebApi cameraIdentifier 只能包含
    # 字母、数字和下划线的契约；组合来源保留在响应 pairMetadata 中。
    assert all(config.camera_identifier == "CAM-A" for config in created_configs)
    assert all(config.license_plate == "PLATE-01" for config in created_configs)
    assert all(config.input_unit == "m" for config in created_configs)
    assert all(
        config.weighbridge_plane_source == (0.0, 0.0, 1.0, -0.25)
        for config in created_configs
    )
    assert all(
        config.weighbridge_plane_orientation_verified
        for config in created_configs
    )
    assert all(config.scene_auto_level_required for config in created_configs)
    assert all(config.floor_require_weighbridge_plane for config in created_configs)
    assert all(
        config.scene_preprocessing_fail_open is False
        for config in created_configs
    )
    assert all(config.scene_preprocessing_enable is False for config in created_configs)
    # 未启用永久空车档案时，融合分析不额外生成补点 PCD。
    assert all(config.save_clean_pointcloud is False for config in created_configs)


def test_analyze_pair_extraction_failure_reports_zero_merged_points(
    tmp_path, monkeypatch
):
    pcd_a = tmp_path / "camera-a.pcd"
    pcd_b = tmp_path / "camera-b.pcd"
    extrinsic_path = tmp_path / "b-to-a.json"
    for path in (pcd_a, pcd_b, extrinsic_path):
        path.write_text("placeholder", encoding="utf-8")

    class FakeLoader:
        def __init__(self, config):
            pass

        def load(self, path):
            return np.ones((3, 3))

    class FakeExtrinsic:
        @staticmethod
        def apply(points):
            return points + np.array([10., 0., 0.])

        quality = type("Quality", (), {"trustworthy": True})()
        baseline_residual = 0.01
        weighbridge_plane_a = None
        empty_lane_baseline = None

        @staticmethod
        def validate_camera_pair(camera_a, camera_b):
            assert (camera_a, camera_b) == ("CAM-A", "CAM-B")

    class FakeAnalyzer:
        def __init__(self, config):
            pass

        def analyze(self, path, *, measurement_points=None):
            return {"png": "", "json": "", "integrationPayloads": {}}

    def fail_merge(*args, **kwargs):
        raise service.DualCloudMergeError("共同场景未提取到完整车辆")

    monkeypatch.setattr(service, "PointCloudLoader", FakeLoader)
    monkeypatch.setattr(service.Extrinsic, "load", lambda path: FakeExtrinsic())
    monkeypatch.setattr(service, "merge_vehicle_clouds", fail_merge)

    svc = service.AnalysisService(analyzer_factory=FakeAnalyzer)
    try:
        result = svc.analyze_pair_sync(
            service.PairAnalyzeRequest(
                pcd_path_a=str(pcd_a),
                camera_a="CAM-A",
                pcd_path_b=str(pcd_b),
                camera_b="CAM-B",
                pair_id="pair-extraction-failure",
                workflow_id="workflow-extraction-failure",
                extrinsic_path=str(extrinsic_path),
            )
        )
    finally:
        svc.shutdown()

    assert result["fusion"]["mode"] == "fallback_a"
    assert result["fusion"]["qualityPassed"] is False
    assert result["fusion"]["pointsFromA"] == 3
    assert result["fusion"]["pointsFromB"] == 3
    assert result["fusion"]["pointsCombined"] == 6
    assert result["fusion"]["pointsMerged"] == 0
    assert result["fusion"]["reason"] == "共同场景未提取到完整车辆"
    assert result["analysisCamera"] == "CAM-A"


def test_write_merged_pcd_is_readable_and_leaves_no_temp_file(tmp_path):
    target = tmp_path / "pair_merged.pcd"
    points = np.array(
        [
            [1.0, 2.0, 3.0],
            [-1.5, 0.25, 8.0],
        ]
    )

    service.AnalysisService._write_merged_pcd(target, points)

    loaded = np.asarray(cloud_io.o3d.io.read_point_cloud(str(target)).points)
    assert np.allclose(loaded, points)
    assert not list(tmp_path.glob(".*.tmp.pcd"))


def test_async_service_never_runs_more_than_configured_concurrency(tmp_path):
    paths = []
    for index in range(8):
        pcd_path = tmp_path / f"{index}.pcd"
        pcd_path.write_text("placeholder", encoding="utf-8")
        paths.append(str(pcd_path))

    lock = threading.Lock()
    active = 0
    peak = 0

    class SlowAnalyzer:
        def __init__(self, config):
            pass

        def analyze(self, path, *, measurement_points=None):
            nonlocal active, peak
            with lock:
                active += 1
                peak = max(peak, active)
            time.sleep(0.03)
            with lock:
                active -= 1
            return {"png": "", "json": "", "integrationPayloads": {}}

    async def run_all():
        svc = service.AnalysisService(max_concurrency=4, analyzer_factory=SlowAnalyzer)
        try:
            results = await asyncio.gather(
                *[
                    svc.analyze(service.AnalyzeRequest(path))
                    for path in paths
                ], return_exceptions=True
            )
            assert not any(isinstance(r, Exception) for r in results)
            assert sum(isinstance(r, dict) for r in results) == 8
        finally:
            svc.shutdown()

    asyncio.run(run_all())

    assert peak == 4


def test_fastapi_analyze_accepts_json_body(tmp_path):
    pytest.importorskip("fastapi.testclient")
    from fastapi.testclient import TestClient

    pcd_path = tmp_path / "scan.pcd"
    pcd_path.write_text("placeholder", encoding="utf-8")

    class FakeAnalyzer:
        def __init__(self, config):
            pass

        def analyze(self, path, *, measurement_points=None):
            return {"png": "", "json": "", "integrationPayloads": {}}

    app = composition.create_app(max_concurrency=4, analyzer_factory=FakeAnalyzer)
    try:
        client = TestClient(app)
        response = client.post(
            "/analyze",
            json={
                "pcdPath": str(pcd_path),
                "camera": "CAM-01",
                "plate": "PLATE-01",
                # Worker 仍在请求体里发这两个历史字段。可视化功能已删除，
                # 契约要求继续接受并忽略它们，而不是返回 422。
                "saveVisualization": False,
                "maxVizPoints": 15000,
            },
        )
    finally:
        app.state.analysis_service.shutdown()

    assert response.status_code == 200
    assert response.json()["outputs"]["json"] == ""
    assert not list(tmp_path.glob("*.json"))
    assert not list(tmp_path.glob("*.png"))


def test_fastapi_analyze_pair_accepts_fixed_ab_contract(tmp_path, monkeypatch):
    pytest.importorskip("fastapi.testclient")
    from fastapi.testclient import TestClient

    pcd_a = tmp_path / "a.pcd"
    pcd_b = tmp_path / "b.pcd"
    extrinsic_path = tmp_path / "extrinsic.json"
    for path in (pcd_a, pcd_b, extrinsic_path):
        path.write_text("placeholder", encoding="utf-8")

    class FakeLoader:
        def __init__(self, config):
            pass

        def load(self, path):
            return np.ones((3, 3))

    class FakeMergeResult:
        points = np.ones((2, 3))
        quality_passed = False
        reason = "quality warning"

        @staticmethod
        def to_dict():
            return {
                "mode": "rejected",
                "qualityPassed": False,
                "reason": "quality warning",
                "pointsFromA": 3,
                "pointsFromB": 3,
                "pointsFiniteA": 3,
                "pointsFiniteB": 3,
                "pointsFilteredA": 3,
                "pointsFilteredB": 3,
                "pointsCombined": 6,
                "pointsMerged": 2,
                "overlapRmse": None,
                "overlapMatchRatio": None,
                "warnings": ["quality warning"],
            }

    class FakeVehicleMergeResult:
        points = FakeMergeResult.points
        selected_observed_points = np.vstack([
            np.ones((3, 3)), np.tile([12., 2., 2.], (3, 1)),
        ])
        fusion = FakeMergeResult()

        @staticmethod
        def to_dict():
            return {
                "coordinateFrame": "cameraA",
                "points": {"mergedVehicleOnly": 2},
                "temporarySiteFrame": {},
                "environmentFilter": {},
                "fusion": FakeMergeResult.to_dict(),
            }

    class FakeAnalyzer:
        def __init__(self, config):
            pass

        def analyze(self, path, *, measurement_points=None):
            return {"png": "", "json": "", "integrationPayloads": {}}

    class FakeExtrinsic:
        @staticmethod
        def apply(points):
            return points + np.array([10., 0., 0.])

        quality = type("Quality", (), {"trustworthy": True})()
        baseline_residual = 0.01
        weighbridge_plane_a = None
        empty_lane_baseline = None

        @staticmethod
        def validate_camera_pair(camera_a, camera_b):
            assert (camera_a, camera_b) == ("CAM-A", "CAM-B")

    def fake_write(path, points):
        path.write_text("merged", encoding="utf-8")

    monkeypatch.setattr(service, "PointCloudLoader", FakeLoader)
    monkeypatch.setattr(service.Extrinsic, "load", lambda path: FakeExtrinsic())
    monkeypatch.setattr(
        service,
        "merge_vehicle_clouds",
        lambda a, b, ext, **kwargs: FakeVehicleMergeResult(),
    )
    monkeypatch.setattr(
        service.AnalysisService, "_write_merged_pcd", staticmethod(fake_write)
    )

    app = composition.create_app(max_concurrency=4, analyzer_factory=FakeAnalyzer)
    try:
        client = TestClient(app)
        response = client.post(
            "/analyze-pair",
            json={
                "pcdPathA": str(pcd_a),
                "cameraA": "CAM-A",
                "pcdPathB": str(pcd_b),
                "cameraB": "CAM-B",
                "pairId": "pair-001",
                "workflowId": "workflow-001",
                "extrinsicPath": str(extrinsic_path),
                "plate": "",
                "saveVisualization": False,
                "maxVizPoints": 15000,
            },
        )
    finally:
        app.state.analysis_service.shutdown()

    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["pairId"] == "pair-001"
    assert payload["workflowId"] == "workflow-001"
    assert payload["sourceCameras"] == {
        "a": "CAM-A",
        "b": "CAM-B",
        "combined": "CAM-A+CAM-B",
    }
    assert payload["fusion"]["warnings"] == ["quality warning"]
    assert payload["fusion"]["mode"] == "fallback_a"
    assert payload["fusion"]["qualityPassed"] is False
    assert payload["fusion"]["reason"] == "quality warning"
    assert payload["analysisCamera"] == "CAM-A"
    assert Path(payload["pcd_path"]) == pcd_a.resolve()


# ---------------------------------------------------------------------------
# 场地外参改由服务端配置：标定是场地属性，不是请求属性
# ---------------------------------------------------------------------------
def _site_extrinsic(tmp_path, name: str = "site.json"):
    """写一份通过启动期校验的外参 JSON。"""
    path = tmp_path / name
    path.write_text(json.dumps({
        "version": "v1",
        "frame": "B_to_A",
        "cameraA": "CAM_004",
        "cameraB": "CAM_005",
        "matrix": np.eye(4).tolist(),
        "baselineResidual": 0.04433,
        "weighbridgePlaneA": [0.0, 0.0, 1.0, -5.1],
        "quality": {
            "fitness": 0.31,
            "inlierRmse": 0.044,
            "correspondenceCount": 18628,
            "degenerateAxes": [],
            "conditionNumber": 194.1,
            "warnings": [],
        },
    }), encoding="utf-8")
    return path


def test_service_verifies_site_extrinsic_at_startup(tmp_path):
    """配置了场地外参就在启动期校验一次，坏标定不该等第一辆车来才发现。"""
    svc = service.AnalysisService(
        extrinsic_path=str(_site_extrinsic(tmp_path))
    )
    try:
        assert svc._extrinsic_path == str((tmp_path / "site.json").resolve())
        assert svc._site_extrinsic.camera_a
    finally:
        svc.shutdown()


def test_service_rejects_untrustworthy_site_extrinsic_at_startup(tmp_path):
    """空磅道单独标定的 degenerateAxes=['y','x'] 外参必须启动即拒。"""
    path = tmp_path / "bad.json"
    payload = json.loads(_site_extrinsic(tmp_path).read_text(encoding="utf-8"))
    payload["quality"]["degenerateAxes"] = ["y", "x"]
    path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match="trustworthy"):
        service.AnalysisService(extrinsic_path=str(path))


def test_pair_request_requires_extrinsic_without_service_config():
    """未配置服务端外参时请求必须自带 extrinsicPath。"""
    svc = service.AnalysisService()
    request = service.PairAnalyzeRequest(
        pcd_path_a="a.pcd", camera_a="CAM_004", pcd_path_b="b.pcd", camera_b="CAM_005",
        pair_id="pair", workflow_id="workflow")
    try:
        with pytest.raises(ValueError, match="必须提供 extrinsicPath"):
            svc.analyze_pair_sync(request)
    finally:
        svc.shutdown()


def test_main_accepts_omitted_root_without_starting_a_real_server(tmp_path, monkeypatch):
    import uvicorn
    (tmp_path / 'appsettings.json').write_text('{}', encoding='utf-8')
    monkeypatch.chdir(tmp_path)
    calls = []
    monkeypatch.setattr(uvicorn, 'run', lambda app, **options: calls.append((app, options)))
    assert composition.main(['--host', '127.0.0.1', '--port', '18010']) == 0
    app, options = calls[0]
    app.state.analysis_service.shutdown()
    assert options == {'host': '127.0.0.1', 'port': 18010}


def test_unknown_input_unit_is_rejected_instead_of_falling_back_to_auto(tmp_path):
    from fastapi.testclient import TestClient

    app = composition.create_app()
    try:
        response = TestClient(app).post(
            "/analyze", json={"pcdPath": str(tmp_path / "scan.pcd"), "inputUnit": "cm"})
    finally:
        app.state.analysis_service.shutdown()
    assert response.status_code == 400
    assert "inputUnit" in response.json()["detail"]
