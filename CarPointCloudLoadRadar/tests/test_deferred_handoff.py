"""合成几何+真实SQLite/归档收尾联测；不启动雷达/HTTP服务，不连接真实业务库。"""
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from app.deferred_measurements import DeferredMeasurements, unpack_prepared
from app.load_radar_bridge import LoadRadarBridge
from app.load_radar_transport import BridgeConfig
from app.result_finalizer import PreparedBudgetExceeded, ResultFinalizer
from app.service import ChannelState, ChannelWorker, EventBus
from tests.test_frozen_detection import baseline_and_model, fixture
from tests.test_load_radar_bridge import MemoryClient


TEMP_ROOT = Path(tempfile.gettempdir()) / "load-radar-async-tests"


class HistoricalMemoryClient(MemoryClient):
    def prepare_passage(self, payload, *, measured_at=None):
        key = hashlib.sha256(json.dumps([payload["nodeId"], payload["channelId"], payload["passageId"]],
            separators=(",", ":")).encode()).hexdigest()
        workflow = hashlib.sha256(json.dumps(["load-radar-arrival", key], separators=(",", ":")).encode()).hexdigest()
        return dict(start=dict(payload), status="ready", workflowId=workflow, cameraIdentifier="CAM_004",
            baselineAsOf=datetime.fromtimestamp(measured_at, timezone.utc).isoformat(), baseline=dict(self.version),
            binding=dict(passageKey=key, workflowId=workflow, cameraIdentifier="CAM_004",
                licensePlate="宁A05922F", vehicleIdentity="plate:宁A05922F"))


class TestDeferredHandoff(unittest.TestCase):
    def setUp(self):
        TEMP_ROOT.mkdir(parents=True, exist_ok=True)
        self.tmp = tempfile.TemporaryDirectory(prefix="handoff-tests-", dir=TEMP_ROOT)
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.worker, self.frame, self.stab, identity, _start = fixture()
        snapshot = baseline_and_model(self.worker, identity)
        self.client = HistoricalMemoryClient()
        stamp = datetime.fromtimestamp(self.frame.ts - 10, timezone.utc).isoformat()
        self.client.version = dict(versionId="version-original", scopeKey="a" * 64, nodeId="test-node",
            channelId="CH4", calibrationId="cal-4", measurementId="empty-original",
            vehicleIdentity="plate:宁A05922F", licensePlate="宁A05922F", actor="test-admin",
            reason="合成几何离线验证", createdAt=stamp, active=False, activatedAt=stamp,
            assetSha256=hashlib.sha256(snapshot.asset_bytes).hexdigest())
        self.client.assets["version-original"] = snapshot.asset_bytes
        self.client.rows["empty-original"] = {"measurement": dict(measurementId="empty-original",
            nodeId="test-node", channelId="CH4", calibrationId="cal-4",
            configurationHash=self.worker._integration_hash,
            baselineCandidateSha256=self.client.version["assetSha256"], sourceMode="live", verdict="empty", frameDegraded=False)}
        self.config = BridgeConfig(node_id="test-node", base_url="http://127.0.0.1:1",
            agent_key="offline-test-only-not-a-real-key-123", storage_dir=self.root / "storage",
            algorithm_version="test-algorithm", channels={"CH4": {"calibration_id": "cal-4"}},
            retry_sec=1., timeout_sec=1.)
        self.network = patch("socket.create_connection", side_effect=AssertionError("离线验收不得联网"))
        self.network.start()
        self.addCleanup(self.network.stop)
        self.open_stack()
        self.enter()
        self.worker.integration = self.bridge
        self.worker.bus = EventBus()

    def open_stack(self, finalizer=None):
        self.bridge = LoadRadarBridge(self.config, self.client)
        self.addCleanup(self.bridge.close)
        self.finalizer = finalizer or ResultFinalizer({"CH4": (str(self.root / "archives"), True)}, self.bridge)
        if finalizer is None:
            self.addCleanup(lambda: self.finalizer.close(10.))
        self.engine = DeferredMeasurements(self.bridge, self.finalizer, ["CH4"])
        self.addCleanup(lambda: self.engine.close(10.))

    def enter(self):
        self.bridge.configure_channel("CH4", self.worker._integration_hash, "live")
        for _ in range(3):
            self.bridge.observe("CH4", False, True)
            self.bridge._idle["CH4"] = (3, time.monotonic() - 1.)
        self.bridge.observe("CH4", True, True)
        self.bridge._starts["CH4"]["arrivedAt"] = datetime.fromtimestamp(self.frame.ts - 1., timezone.utc).isoformat()

    def capture(self):
        mid = self.engine.capture(self.worker, self.frame, self.stab)
        self.assertIsNone(self.bridge.outbox.get(mid))
        self.assertTrue(self.engine.persist_once("CH4"))
        return mid

    def wait_outbox(self, mid):
        until = time.monotonic() + 10.
        while time.monotonic() < until:
            row = self.bridge.outbox.get(mid)
            if row is not None:
                return row
            time.sleep(.01)
        self.fail("纯本地收尾未在预算内提交")

    def ready(self, mid):
        self.engine.store.retry("CH4", mid, "test-clock", 0, now=0)

    def prepare_waiting(self):
        mid = self.capture()
        with patch.object(self.finalizer, "submit_prepared", return_value=False):
            self.assertTrue(self.engine.process_once("CH4"))
        row = self.engine.store.get("CH4", mid)
        self.assertEqual("prepared", row.state)
        self.assertIsNotNone(row.prepared_bytes)
        return mid, row, unpack_prepared(row.prepared_bytes)

    def test_exit_storage_failure_keeps_exact_original_frames_until_retry_is_durable(self):
        from app.cloud_observation import FrameSnapshot
        from app.recalculation import decode_exit
        mid = self.capture()
        frames = (FrameSnapshot.freeze(self.frame),)
        self.engine.offer_exit(mid, frames)
        with patch.object(self.engine.store, "freeze_exit", side_effect=OSError("test disk full")):
            self.engine.process_once("CH4")
        self.assertEqual(frames, self.engine._exit_volume[mid])
        self.ready(mid)
        with patch("app.deferred_measurements.calculate_frozen", side_effect=RuntimeError("stop after durable exit")):
            self.engine.process_once("CH4")
        self.assertEqual(frames, decode_exit(self.engine.store.exit_bytes("CH4", mid)))
        self.assertNotIn(mid, self.engine._exit_volume)

    def test_oversized_prepared_result_is_explicitly_rejected_and_original_task_retained(self):
        mid, original, (result_json, prepared) = self.prepare_waiting()
        self.finalizer.byte_budget = 1
        with patch.object(self.finalizer, "start") as start, \
                patch.object(self.bridge, "adopt_deferred") as adopt:
            with self.assertRaisesRegex(PreparedBudgetExceeded, "预算"):
                self.finalizer.submit_prepared(original.identity, result_json, prepared)
            self.ready(mid)
            self.assertTrue(self.engine.process_once("CH4"))
            start.assert_not_called()
            adopt.assert_not_called()
        blocked = self.engine.store.get("CH4", mid)
        self.assertEqual("blocked", blocked.state)
        self.assertEqual("历史最终结果超过本通道收尾内存预算，原任务保留待处理", blocked.error)
        for name in ("identity", "start_json", "input_bytes", "preparation_bytes", "prepared_bytes"):
            self.assertEqual(getattr(original, name), getattr(blocked, name), name)
        self.assertIsNone(self.bridge.outbox.get(mid))
        self.assertNotIn("CH4", self.finalizer._slots)
        self.assertNotIn(mid, self.engine._handed)
        self.assertFalse(self.engine.process_once("CH4"))

    def test_busy_slot_remains_retryable_without_adopting_or_starting(self):
        _mid, original, (result_json, prepared) = self.prepare_waiting()
        # 即使槽位忙，永久超预算仍应先明确拒绝，不得伪装成暂时拥塞。
        with patch.dict(self.finalizer._slots, {"CH4": object()}), \
                patch.object(self.finalizer, "start") as start, \
                patch.object(self.bridge, "adopt_deferred") as adopt:
            self.assertFalse(self.finalizer.submit_prepared(original.identity, result_json, prepared))
            self.finalizer.byte_budget = 1
            with self.assertRaisesRegex(PreparedBudgetExceeded, "预算"):
                self.finalizer.submit_prepared(original.identity, result_json, prepared)
            start.assert_not_called()
            adopt.assert_not_called()

    def assert_typed_payload_conflict(self, field, convert):
        mid, original, (_result_json, prepared) = self.prepare_waiting()
        payload = json.loads(prepared.payload_json)
        changed = dict(payload)
        changed[field] = convert(payload[field])
        self.assertEqual(payload, changed)  # Python 相等不能证明 JSON 数值类型或原字节相同。
        self.assertIsNot(type(payload[field]), type(changed[field]))
        existing = SimpleNamespace(payload=changed, asset=prepared.asset)
        self.ready(mid)
        with patch.object(self.bridge.outbox, "get", return_value=existing), \
                patch.object(self.bridge.outbox.archives, "get", return_value=prepared.archive), \
                patch.object(self.finalizer, "restore_durable") as restore:
            self.assertTrue(self.engine.process_once("CH4"))
            restore.assert_not_called()
        blocked = self.engine.store.get("CH4", mid)
        self.assertEqual("blocked", blocked.state)
        self.assertEqual(original.prepared_bytes, blocked.prepared_bytes)
        self.assertEqual(original.input_bytes, blocked.input_bytes)
        self.assertFalse(self.engine.process_once("CH4"))

    def test_restore_rejects_boolean_replaced_with_equal_integer(self):
        self.assert_typed_payload_conflict("frameDegraded", int)

    def test_restore_rejects_integer_replaced_with_equal_float(self):
        self.assert_typed_payload_conflict("pointCount", float)

    def test_departed_original_reaches_real_outbox_without_overwriting_next_vehicle(self):
        mid = self.capture()
        original = self.engine.store.get("CH4", mid)
        self.enter()
        next_passage = self.bridge.current_passage("CH4")
        self.assertNotEqual(original.identity.passage_id, next_passage)
        self.worker._session_id = 999
        self.engine.process_once("CH4")
        payload = self.wait_outbox(mid).payload
        self.assertAlmostEqual(9.6, payload["volumeM3"], places=4)
        self.assertEqual((mid, original.identity.passage_id), (payload["measurementId"], payload["passageId"]))
        self.assertIsNone(self.bridge._lane("CH4")["measurement"])
        self.ready(mid)
        self.engine.process_once("CH4")
        self.assertEqual("committed", self.engine.store.get("CH4", mid).state)
        self.finalizer.close(10.)
        events = []
        live = SimpleNamespace(_finalizer=self.finalizer, ch=self.worker.ch, integration=self.bridge,
            _current_result_token="next-vehicle-token", history=deque(), st=ChannelState("CH4", "下一车"),
            bus=SimpleNamespace(publish=lambda *args: events.append(args)), _saved_this_session=0)
        live.st.last_result = {"measurement_id": "next-vehicle-result"}
        live._runtime_record = ChannelWorker._runtime_record.__get__(live)
        ChannelWorker._drain_finalizer_locked(live)
        self.assertEqual("next-vehicle-result", live.st.last_result["measurement_id"])
        history = [body for kind, body in events if kind == "result_history"]
        self.assertEqual(1, len(history))
        self.assertEqual(19, history[0]["session"])
        self.assertEqual(mid, history[0]["measurement_id"])

    def test_shared_registered_archive_recomputes_from_original_asset(self):
        from app.archive_reader import load_archive
        from app.sample_paths import Sample
        from app.volume_replay import recompute
        from tests.test_sample_paths import publish
        mid = self.capture()
        self.engine.process_once("CH4")
        self.wait_outbox(mid)
        bundle = self.bridge.outbox.archives.get(mid)
        directory = self.root / "recompute-original"
        publish(directory, bundle)
        meta = bundle.metadata
        sample = Sample(str(directory / "observation.xyz"), "observation", "CH4", mid,
            meta["acquired_at"], hashlib.sha256(bundle.manifest).hexdigest())
        archive = load_archive(sample)
        report = recompute(archive)
        self.assertTrue(report["matched"], report)
        self.assertEqual("ch4-shared-v1", report["recomputed"]["baseline_registration"]["policy"])

    def test_prepared_bytes_restart_replays_without_plate_current_frame_or_recalculation(self):
        self.engine.close(10.)
        # 不运行收尾线程，先让独立任务保存完整最终字节，模拟提交前进程退出。
        waiting = SimpleNamespace(submit_prepared=lambda *args: False, restore_durable=lambda *args: True)
        self.engine = DeferredMeasurements(self.bridge, waiting, ["CH4"])
        self.addCleanup(lambda: self.engine.close(10.))
        mid = self.capture()
        self.engine.process_once("CH4")
        original = self.engine.store.get("CH4", mid)
        self.assertIsNotNone(original.prepared_bytes)
        original_payload = json.loads(unpack_prepared(original.prepared_bytes)[1].payload_json)
        self.assertEqual("CH4", original_payload["baselineUse"]["sourceChannelId"])
        self.client.version = dict(self.client.version, versionId="replacement", calibrationId="ch4-new-calibration")
        self.client.assets.clear()
        self.client.rows.clear()
        self.engine.close(10.)
        self.finalizer.close(10.)
        self.bridge.close()
        self.open_stack()
        self.assertIsNone(self.bridge.current_passage("CH4"))
        self.ready(mid)
        with patch("app.deferred_measurements.calculate_frozen", side_effect=AssertionError("不得重算")), \
                patch.object(self.client, "prepare_passage", side_effect=AssertionError("不得重取车牌")):
            self.engine.process_once("CH4")
            replayed = self.wait_outbox(mid)
            self.assertEqual(original_payload, replayed.payload)
            self.assertEqual("version-original", replayed.payload["baselineVersionId"])
            self.ready(mid)
            self.engine.process_once("CH4")
        restored = self.engine.store.get("CH4", mid)
        self.assertEqual("committed", restored.state)
        self.assertEqual(original.prepared_bytes, restored.prepared_bytes)
        self.assertEqual(original.input_bytes, restored.input_bytes)


if __name__ == "__main__":
    unittest.main()
