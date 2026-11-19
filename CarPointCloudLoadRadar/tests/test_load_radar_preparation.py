"""首测前车次/基线准备；全部使用独占临时目录与无网络替身。"""
import hashlib
import io
import json
import time
import unittest
from datetime import datetime, timezone, timedelta
from unittest.mock import patch

from app.load_radar_bridge import LoadRadarBridge
from app.load_radar_transport import HttpError
from tests.test_load_radar_bridge import TestLoadRadarBridge as _ExistingBridgeTests
from tests.test_load_radar_contract import _npz
from tests import test_result_finalizer as _finalizer
from tests import test_volume_runtime as _volume
from tests.test_volume_provider import catalog_for, model_for
import numpy as np


class TestLoadRadarPreparation(unittest.TestCase):
    setUp = _ExistingBridgeTests.setUp
    enter = _ExistingBridgeTests.enter

    def configure(self, mode="live"):
        self.bridge.configure_channel("CH1", "a" * 64, mode)
        self.requests = []
        self.mode = "ready"
        self.mutate = lambda row: row
        self.client.prepare_passage = self.respond
        stamp = (datetime.now(timezone.utc) - timedelta(minutes=2)).isoformat()
        data = _npz()
        self.version = dict(versionId="v1", scopeKey="b" * 64, nodeId="jt-test", channelId="CH4",
            calibrationId="cal-1", measurementId="empty-baseline", vehicleIdentity="plate:宁A25771",
            licensePlate="宁A25771", actor="admin", reason="测试人工维护", createdAt=stamp,
            active=True, activatedAt=stamp, assetSha256=hashlib.sha256(data).hexdigest())
        self.client.assets["v1"] = data
        self.client.rows["empty-baseline"] = {"measurement": dict(measurementId="empty-baseline",
            nodeId="jt-test", channelId="CH4", calibrationId="cal-1", configurationHash="a" * 64,
            baselineCandidateSha256=self.version["assetSha256"], sourceMode="live", verdict="empty", frameDegraded=False)}

    def respond(self, payload):
        self.requests.append(dict(payload))
        key = hashlib.sha256(json.dumps([payload["nodeId"], payload["channelId"], payload["passageId"]],
            separators=(",", ":")).encode()).hexdigest()
        workflow = hashlib.sha256(json.dumps(["load-radar-arrival", key], separators=(",", ":")).encode()).hexdigest()
        binding = dict(passageKey=key, workflowId=workflow, licensePlate="宁A25771",
            vehicleIdentity="plate:宁A25771", cameraIdentifier="CAM_001")
        return self.mutate(dict(start=dict(payload), status=self.mode, workflowId=workflow, cameraIdentifier="CAM_001",
            binding=binding if self.mode in ("ready", "baseline_required") else None,
            baseline=dict(self.version) if self.mode == "ready" else None))

    def test_baseline_is_ready_without_first_measurement(self):
        self.configure()
        passage = self.enter()
        self.assertEqual(self.bridge.preparation_status("CH1"), "awaiting_plate")
        self.bridge.sync_once()
        self.assertEqual(self.bridge.preparation_status("CH1"), "ready")
        self.assertEqual(self.client.posts, [])
        self.assertIsNone(self.bridge._lane("CH1")["measurement"])
        base, version, plate = self.bridge.baseline("CH1", time.time())
        self.assertIsNotNone(base)
        self.assertEqual((version, plate), ("v1", "宁A25771"))
        self.bridge.sync_once()
        self.assertEqual(self.requests[0], self.requests[1])
        self.assertEqual(self.requests[0]["passageId"], passage)

    def test_waiting_plate_then_missing_baseline_never_fabricates_baseline(self):
        self.configure()
        self.enter()
        for status in ("awaiting_plate", "plate_unavailable", "baseline_required"):
            self.mode = status
            with patch("app.load_radar_bridge.make_candidate") as make:
                self.bridge.sync_once()
                make.assert_not_called()
            self.assertEqual(self.bridge.preparation_status("CH1"), status)
            self.assertEqual(self.bridge.baseline("CH1", time.time()), (None, None, ""))

    def test_delayed_response_cannot_prepare_next_vehicle(self):
        self.configure()
        old = self.enter()
        def change(row):
            self.enter()
            return row
        self.mutate = change
        self.bridge.sync_once()
        self.assertNotEqual(old, self.bridge.current_passage("CH1"))
        self.assertEqual(self.bridge.preparation_status("CH1"), "awaiting_plate")
        self.assertEqual(self.bridge.baseline("CH1", time.time()), (None, None, ""))

    def test_broken_identity_or_context_fails_closed(self):
        self.configure()
        self.enter()
        for field, value in (("channelId", "CH2"), ("configurationHash", "b" * 64), ("passageId", "other")):
            with self.subTest(field=field):
                self.mutate = lambda row, k=field, v=value: dict(row, start=dict(row["start"], **{k: v}))
                self.bridge.sync_once()
                self.assertEqual(self.bridge.preparation_status("CH1"), "awaiting_plate")
        self.mutate = lambda row: dict(row, binding=dict(row["binding"], workflowId="other"))
        self.bridge.sync_once()
        self.assertEqual(self.bridge.baseline("CH1", time.time()), (None, None, ""))

    def test_network_failure_retries_original_and_clears_prepared_lease(self):
        self.configure()
        self.enter()
        self.bridge.sync_once()
        original = self.requests[-1]
        with patch.object(self.client, "prepare_passage", side_effect=HttpError(503, True)):
            self.bridge.sync_once()
        self.assertEqual(self.bridge.preparation_status("CH1"), "awaiting_plate")
        self.bridge.sync_once()
        self.assertEqual(original, self.requests[-1])
        self.assertEqual(self.bridge.preparation_status("CH1"), "ready")

    def test_restart_cannot_reuse_current_vehicle_until_new_idle(self):
        self.configure()
        old = self.enter()
        self.bridge.sync_once()
        self.bridge.close()
        self.bridge = LoadRadarBridge(self.config, self.client)
        self.bridge.configure_channel("CH1", "a" * 64, "live")
        self.bridge.observe("CH1", True, True)
        self.assertIsNone(self.bridge.current_passage("CH1"))
        self.bridge.sync_once()
        self.assertEqual(len(self.requests), 1)
        self.assertNotEqual(old, self.enter())

    def test_demo_never_registers_real_vehicle(self):
        self.configure("simulate")
        self.enter()
        self.bridge.sync_once()
        self.assertEqual(self.requests, [])
        self.assertEqual(self.bridge.preparation_status("CH1"), "not_applicable")


# 不让 unittest 把导入的旧 TestCase 再运行一遍。
del _ExistingBridgeTests


class TestPreparationAtWorker(unittest.TestCase):
    setUp = _finalizer.TestResultFinalizer.setUp
    close = _finalizer.TestResultFinalizer.close
    bridge = _finalizer.TestResultFinalizer.bridge
    worker = _finalizer.TestResultFinalizer.worker

    def test_waiting_identity_does_not_consume_measurement_or_relax_stop_gate(self):
        bridge = self.bridge()
        worker = self.worker(bridge, save=False, live=True)
        ground, truck = _finalizer.TestResultFinalizer.ground(), _finalizer.TestResultFinalizer.truck()
        for _ in range(8):
            self.clock.tick(worker, ground)
        for _ in range(70):
            self.clock.tick(worker, truck)
        self.assertIsNotNone(bridge.current_passage("CH1"))
        self.assertEqual(worker._detected_this_session, 0)
        self.assertEqual(bridge.outbox.stats()["total"], 0)
        self.requests, self.mode, self.version = [], "baseline_required", {}
        self.mutate = lambda row: row
        bridge.client.prepare_passage = lambda payload: TestLoadRadarPreparation.respond(self, payload)
        bridge.sync_once()
        self.assertEqual(bridge.preparation_status("CH1"), "baseline_required")
        for _ in range(30):
            self.clock.tick(worker, truck)
            if worker._detected_this_session:
                break
        row = _finalizer.await_result(worker, exported=False)[0]
        payload = bridge.outbox.get(row["measurement_id"]).payload
        self.assertEqual(worker._detected_this_session, 1)
        self.assertIsNone(payload["volumeM3"])
        self.assertIsNone(payload["baselineVersionId"])
        self.assertEqual(payload["passageId"], self.requests[0]["passageId"])


class TestAutomaticVolumeFlow(unittest.TestCase):
    setUp = _finalizer.TestResultFinalizer.setUp
    close = _finalizer.TestResultFinalizer.close
    bridge = _finalizer.TestResultFinalizer.bridge
    worker = _volume.TestVolumeRuntime.worker

    def test_prepare_real_calculation_durable_retry_and_original_history(self):
        self.clock.wall = datetime.now(timezone.utc).timestamp()
        bridge = self.bridge()
        worker = self.worker(bridge, save=False, live=True)
        base, frame = _volume.TestVolumeRuntime.surfaces()
        # 已知半厢高出 0.4m 的阶梯散料；不是会被几何空车判定视作底面的整平面。
        frame.xyz[frame.xyz[:, 0] >= 0, 2] += 0.4
        stream = io.BytesIO()
        np.savez_compressed(stream, floor_z=base.floor_z, box=np.asarray(base.box),
            cloud=base.cloud.astype(np.float32), bed_plane=np.asarray(base.bed_plane), created="2024-01-01T00:00:00Z")
        data = stream.getvalue()
        from app.load_radar_contract import _read_baseline
        base = _read_baseline(data)
        stamp = (datetime.now(timezone.utc) - timedelta(minutes=2)).isoformat()
        self.requests, self.mode, self.mutate = [], "awaiting_plate", lambda row: row
        self.version = dict(versionId="version-1", scopeKey="b" * 64, nodeId="test", channelId="CH4",
            calibrationId="cal-1", measurementId="empty-baseline", vehicleIdentity="plate:宁A25771",
            licensePlate="宁A25771", actor="admin", reason="独立空车维护夹具", createdAt=stamp,
            active=True, activatedAt=stamp, assetSha256=hashlib.sha256(data).hexdigest())
        bridge.client.assets["version-1"] = data
        bridge.client.rows["empty-baseline"] = {"measurement": dict(measurementId="empty-baseline",
            nodeId="test", channelId="CH4", calibrationId="cal-1", configurationHash=worker._integration_hash,
            baselineCandidateSha256=self.version["assetSha256"], sourceMode="live", verdict="empty", frameDegraded=False)}
        bridge.client.prepare_passage = lambda payload: TestLoadRadarPreparation.respond(self, payload)
        model = model_for(base)
        model["binding"].update(node_id="test", configuration_sha256=worker._integration_hash,
            baseline_asset_sha256=self.version["assetSha256"])
        model.update(domain_xy=[-1.2, 1.2, -5., 5.], support_origin_xy=[-1.2, -5.],
            valid_from=self.clock.wall - 10, valid_until=self.clock.wall + 60)
        worker.volume_catalog = catalog_for([model])
        for _ in range(8):
            self.clock.tick(worker, _finalizer.TestResultFinalizer.ground())
        for _ in range(30):
            self.clock.tick(worker, frame)
            bridge.sync_once()
        self.assertEqual(worker._detected_this_session, 0)
        self.assertEqual(worker.snapshot()["preparation"], "awaiting_plate")
        original = dict(self.requests[0])
        self.mode = "ready"
        bridge.sync_once()
        self.assertIsNotNone(bridge.baseline("CH1", self.clock.wall)[0])
        self.assertEqual(bridge.outbox.stats()["total"], 0)
        for _ in range(50):
            self.clock.tick(worker, frame)
            if worker._detected_this_session:
                break
            bridge.sync_once()
        row = _finalizer.await_result(worker, exported=False)[0]
        payload = bridge.outbox.get(row["measurement_id"]).payload
        self.assertEqual(payload["passageId"], original["passageId"])
        self.assertEqual(payload["baselineVersionId"], "version-1")
        self.assertEqual(payload["volumeStatus"], "success", (payload["verdict"], row["empty"], row["step"], row["reason"], payload["frameDegraded"]))
        self.assertAlmostEqual(payload["volumeM3"], 14.4, places=5)
        self.assertEqual(payload["baselineUse"]["sourceChannelId"], "CH4")
        self.assertIsNone(row["volume_preparation"])
        bridge.client.failure = HttpError(503, True)
        # ★ M10（2026-09-23）：后台在线的 503 不再 raise 堵死整队 ——
        #   只把这一条记账留 pending，恢复后自动发出（原语义是无差别
        #   raise，一个被持续 5xx 的测量会连累发件箱和基线同步段）。
        bridge.sync_once()
        self.assertEqual(bridge.outbox.get(row["measurement_id"]).status, "pending",
                         "可重试 503 只该记账，不得误升级成终态 blocked")
        bridge.client.failure = None
        bridge.sync_once()
        self.assertEqual(bridge.outbox.get(row["measurement_id"]).status, "sent")
        posts = [p for p in bridge.client.posts if p["measurementId"] == row["measurement_id"]]
        self.assertEqual(posts, [payload, payload])
        self.assertEqual(len(worker.history), 1)
        self.assertTrue(all(request == original for request in self.requests))
