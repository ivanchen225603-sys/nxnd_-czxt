"""事后空车基线复算：解析货面、真实积分/SQLite持久化，所有HTTP均为内存替身。"""
from dataclasses import replace
from datetime import datetime, timezone, timedelta
import hashlib
import base64
import io
import json
import zipfile
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from app.frozen_detection import FrozenDetection, calculate_frozen
from app.load_radar_bridge import LoadRadarBridge, ResultReservation
from app.load_radar_contract import effective_configuration_bytes
from app.load_radar_transport import BridgeConfig, _json_bytes
from app.pending_measurements import PendingTaskStore
from app.recalculation import RecalculationProcessor, calculate_revision, encode_exit, decode_exit
from app.result_finalizer import prepare_result
from tests.test_frozen_detection import fixture, baseline_and_model, CAPTURE_TS
TEMP_ROOT = Path(tempfile.gettempdir()) / "load-radar-day-recalc-tests"


def loaded_label(pending):
    # 解析夹具为高于空底板0.4m的平面：只固定载重标签，体积积分与配准均运行真实代码。
    result = json.loads(pending.result_json)
    result.update(empty=False, reason="解析载重平面夹具", step=6)
    result["diagnostics"] = dict(result.get("diagnostics") or {}, step=6)
    return replace(pending, result_json=_json_bytes(result), archive_enabled=False, candidate=None)


class TestDayRecalculation(unittest.TestCase):
    def setUp(self):
        TEMP_ROOT.mkdir(parents=True, exist_ok=True)
        self.folder = tempfile.TemporaryDirectory(prefix="day-recalc-", dir=TEMP_ROOT)
        self.addCleanup(self.folder.cleanup)
        self.root = Path(self.folder.name)
        self.config = BridgeConfig(node_id="test-node", base_url="http://127.0.0.1:1",
            agent_key="isolated-recalculation-test-key-123456", storage_dir=self.root,
            algorithm_version="test-algorithm", channels={f"CH{i}": {"calibration_id": f"cal-{i}"} for i in range(1, 5)})
        self.targets, self.receipts, self.sent = {}, {}, []
        self.client = SimpleNamespace(download_baseline=lambda _version: self.asset,
            recalculations=self.poll, post_recalculation=self.post)
        self.bridge = LoadRadarBridge(self.config, self.client)
        self.addCleanup(self.bridge.close)
        self.store = PendingTaskStore(self.root, self.config.channels)
        self.addCleanup(self.store.close)
        self.processor = RecalculationProcessor(self.bridge, self.store)
        self.network = patch("socket.create_connection", side_effect=AssertionError("不得连接真实服务"))
        self.network.start()
        self.addCleanup(self.network.stop)
        self.math = patch("app.recalculation.calculate_frozen", side_effect=lambda *a, **k: loaded_label(calculate_frozen(*a, **k)))
        self.math.start()
        self.addCleanup(self.math.stop)

    def poll(self, channel, after=None, from_date=None):
        target = self.targets.get(channel)
        if target and target["id"] not in self.receipts and (after is None or target["original"]["measurement"]["measurementId"] > after):
            return [target]
        return []

    def post(self, payload):
        self.sent.append(_json_bytes(payload))
        revision = dict(calculation=payload["measurement"], measurementId=payload["measurementId"],
                        baselineVersionId=payload["baselineVersionId"])
        self.receipts[payload["id"]] = dict(id=payload["id"], status="applied", revision=revision)
        return self.receipts[payload["id"]]

    def source(self, channel="CH1", activation=17):
        worker, frame, stab, _identity, start = fixture(save=False)
        worker.ch.id = channel
        worker.integration.config = self.config
        worker._measurement_window = replace(worker._measurement_window,
            pairs=tuple(replace(p, channel=channel) for p in worker._measurement_window.pairs))
        worker._volume_observation_window = replace(worker._volume_observation_window,
            pairs=tuple(replace(p, channel=channel) for p in worker._volume_observation_window.pairs))
        worker._effective_configuration = effective_configuration_bytes(worker.calib, worker.dev, worker.ch, worker.angle)
        worker._integration_hash = hashlib.sha256(worker._effective_configuration).hexdigest()
        mid = "original-" + channel
        identity = ResultReservation(mid, channel, "passage-" + channel, "token-" + channel)
        start.update(channelId=channel, passageId=identity.passage_id, calibrationId=self.config.channels[channel].calibration_id,
                     configurationHash=worker._integration_hash)
        data = FrozenDetection.freeze(worker, frame, stab, identity, start)
        row = self.store.put(identity, _json_bytes(start), data)
        snapshot = baseline_and_model(worker, identity)
        self.asset = snapshot.asset_bytes
        original = loaded_label(calculate_frozen(data, identity, None, "宁A05922F", self.config))
        payload = json.loads(prepare_result(original, self.bridge).payload_json)
        self.store.freeze_exit(channel, mid, encode_exit(()))
        self.store.register_recalculation(channel, mid, payload)
        job_id = hashlib.sha256(json.dumps(["volume-revision", mid, str(activation)], separators=(",", ":")).encode()).hexdigest()
        measured = datetime.fromtimestamp(CAPTURE_TS + 3600, timezone.utc).isoformat()
        self.targets[channel] = dict(id=job_id, activationId=activation,
            original=dict(passageKey=hashlib.sha256(mid.encode()).hexdigest(), payloadHash="f" * 64,
                receivedAt=datetime.fromtimestamp(CAPTURE_TS + 1, timezone.utc).isoformat(), measurement=payload,
                binding=dict(passageKey=hashlib.sha256(mid.encode()).hexdigest(), licensePlate="宁A05922F",
                    workflowId="workflow-" + channel, cameraIdentifier="CAM_00" + channel[-1],
                    vehicleIdentity="plate:宁A05922F", actor="fixture", reason="解析夹具", boundAt=measured)),
            baseline=dict(versionId="afternoon-ch4", channelId="CH4", licensePlate="宁A05922F", nodeId="test-node",
                scopeKey="a" * 64, calibrationId="cal-4", vehicleIdentity="plate:宁A05922F", measurementId="empty-ch4",
                actor="fixture", reason="解析CH4空车", createdAt=measured, activatedAt=measured, active=True,
                assetSha256=hashlib.sha256(self.asset).hexdigest()),
            baselineSource=dict(measurementId="empty-ch4", nodeId="test-node", passageId="empty-passage", algorithmVersion="test-algorithm",
                pointCount=2400, points=[], channelId="CH4", calibrationId="cal-4", configurationHash="e" * 64,
                sourceMode="live", verdict="empty", frameDegraded=False,
                baselineCandidateSha256=hashlib.sha256(self.asset).hexdigest(), measuredAt=measured))
        return row

    def test_cross_project_fixture_matches_real_recalculation_outputs(self):
        path = Path(__file__).parents[2] / "CarPointCloudWebApi/tests/CarPointCloudWebApi.Tests/Fixtures/load-radar-day-recalculation-v1.json"
        expected = json.loads(path.read_text(encoding="utf-8"))
        asset = base64.b64decode(expected["assetBase64"])
        fixture = {}
        for channel in self.config.channels:
            row = self.source(channel)
            target = self.targets[channel]
            # NPZ的ZIP封装随平台变化；内部数组须逐字节一致，HTTP合同使用与C#相同的固定资产。
            with zipfile.ZipFile(io.BytesIO(self.asset)) as generated, zipfile.ZipFile(io.BytesIO(asset)) as recorded:
                self.assertEqual({name: generated.read(name) for name in generated.namelist()},
                                 {name: recorded.read(name) for name in recorded.namelist()})
            self.asset = asset
            target["baseline"]["assetSha256"] = hashlib.sha256(asset).hexdigest()
            target["baselineSource"]["baselineCandidateSha256"] = target["baseline"]["assetSha256"]
            fixture[channel] = dict(target=target, request=json.loads(calculate_revision(self.bridge, row, target, encode_exit(()))))
            # 合同夹具仅留3个预览点；真实积分仍使用冻结的完整解析点云。
            fixture[channel]["target"]["original"]["measurement"]["points"] = target["original"]["measurement"]["points"][:3]
            fixture[channel]["request"]["measurement"]["points"] = fixture[channel]["request"]["measurement"]["points"][:3]
            proof = fixture[channel]["request"]["provenance"]
            self.assertEqual(row.input_sha256, proof["inputSha256"])
            for value in proof.values():
                self.assertRegex(value, r"^[a-f0-9]{64}$")
        fixture["assetBase64"] = base64.b64encode(self.asset).decode("ascii")
        for channel in self.config.channels:
            # 代码指纹按本机实际代码字节计算，换行/构建版本变化不伪装为同一个指纹。
            expected[channel]["request"]["provenance"] = fixture[channel]["request"]["provenance"]
        self.assertEqual(fixture, expected)

    def test_all_four_lanes_use_later_ch4_asset_and_preserve_original_measurement(self):
        for channel in self.config.channels:
            with self.subTest(channel=channel):
                original = self.source(channel)
                target = self.targets[channel]
                payload = json.loads(calculate_revision(self.bridge, original, target, encode_exit(())))
                result = payload["measurement"]
                self.assertEqual("success", result["volumeStatus"], result["reason"])
                self.assertAlmostEqual(2.3 * 9.9 * .4, result["volumeM3"], delta=.01)
                self.assertEqual(channel, result["channelId"])
                self.assertEqual("CH4", result["baselineUse"]["sourceChannelId"])
                self.assertEqual(target["original"]["measurement"]["measuredAt"], result["measuredAt"])
                self.assertGreater(target["baselineSource"]["measuredAt"], result["measuredAt"])
                self.assertEqual(original.input_sha256, self.store.get(channel, original.identity.measurement_id).input_sha256)

    def test_durable_result_restart_retries_exact_bytes_after_commit_response_loss(self):
        self.source()
        self.assertTrue(self.processor.process_once("CH1"))
        pending = self.store.pending_recalculation("CH1")
        self.assertIsNotNone(pending)
        payload = pending[2]
        self.post(json.loads(payload))  # 服务器已提交，但源端还没收到确认。
        self.store.close()
        self.store = PendingTaskStore(self.root, self.config.channels)
        self.addCleanup(self.store.close)
        self.processor = RecalculationProcessor(self.bridge, self.store)
        with patch("app.recalculation.calculate_revision", side_effect=AssertionError("不得重新计算已持久结果")):
            self.assertTrue(self.processor.process_once("CH1"))
        self.assertEqual(payload, self.sent[-1])
        self.assertIsNone(self.store.pending_recalculation("CH1"))

    def test_failure_does_not_acknowledge_or_discard_calculation(self):
        self.source()
        self.processor.process_once("CH1")
        before = self.store.pending_recalculation("CH1")
        self.client.post_recalculation = lambda _payload: (_ for _ in ()).throw(OSError("offline"))
        with self.assertRaises(OSError):
            self.processor.process_once("CH1")
        self.assertEqual(before, self.store.pending_recalculation("CH1"))

    def test_superseded_result_releases_delivery_but_does_not_mark_source_complete(self):
        row = self.source()
        self.processor.process_once("CH1")
        self.client.post_recalculation = lambda payload: dict(id=payload["id"], status="superseded", revision=None)
        self.processor.process_once("CH1")
        cutoff = CAPTURE_TS + 86400
        self.assertIn(row.identity.measurement_id, self.store.protected_measurements(cutoff))

    def test_wrong_receipt_keeps_original_result_for_retry(self):
        self.source()
        self.processor.process_once("CH1")
        pending = self.store.pending_recalculation("CH1")
        self.client.post_recalculation = lambda payload: dict(id=payload["id"], status="applied", revision=dict(calculation={}))
        with self.assertRaises(ValueError):
            self.processor.process_once("CH1")
        self.assertEqual(pending, self.store.pending_recalculation("CH1"))

    def test_unselected_same_passage_result_is_acknowledged_without_a_fake_revision(self):
        row = self.source()
        self.processor.process_once("CH1")
        self.client.post_recalculation = lambda payload: dict(id=payload["id"], status="not-selected", revision=None)
        self.assertTrue(self.processor.process_once("CH1"))
        self.assertIsNone(self.store.pending_recalculation("CH1"))
        self.store.ensure_cleanup_schema()
        with self.store._transaction("CH1") as db:
            db.execute("UPDATE pending_tasks SET state='committed',committed_at=? WHERE measurement_id=?",
                       (CAPTURE_TS, row.identity.measurement_id))
        self.assertNotIn(row.identity.measurement_id, self.store.protected_measurements(CAPTURE_TS + 86400))

    def test_pending_and_current_day_inputs_remain_protected_until_next_cleanup(self):
        row = self.source()
        mid = row.identity.measurement_id
        # 即使原任务已完成，待基线/待复算仍是未完成业务。
        with self.store._transaction("CH1") as db:
            db.execute("UPDATE pending_tasks SET state='committed' WHERE measurement_id=?", (mid,))
        self.store.ensure_cleanup_schema()
        with self.store._transaction("CH1") as db:
            db.execute("UPDATE pending_tasks SET committed_at=? WHERE measurement_id=?", (CAPTURE_TS - 1, mid))
        today = datetime.fromtimestamp(CAPTURE_TS, timezone(timedelta(hours=8))).replace(hour=3, minute=0, second=0).timestamp()
        self.assertIn(mid, self.store.protected_measurements(today + 86400))
        self.processor.process_once("CH1")
        self.processor.process_once("CH1")
        self.assertIn(mid, self.store.protected_measurements(today))
        self.assertNotIn(mid, self.store.protected_measurements(today + 86400))

    def test_local_exit_frame_bytes_are_immutable_and_survive_restart(self):
        row = self.source()
        _, frame, _, _, _ = fixture()
        from app.cloud_observation import FrameSnapshot
        frames = (FrameSnapshot.freeze(frame),)
        # 另一个源记录验证首次写入后不被空重试替换。
        original = self.store.freeze_exit("CH1", "exit-source", encode_exit(frames))
        self.assertEqual(original, self.store.freeze_exit("CH1", "exit-source", encode_exit(())))
        self.store.close()
        self.store = PendingTaskStore(self.root, self.config.channels)
        self.addCleanup(self.store.close)
        self.assertEqual(frames, decode_exit(self.store.exit_bytes("CH1", "exit-source")))
        self.assertEqual(row.input_sha256, self.store.get("CH1", row.identity.measurement_id).input_sha256)

    def test_mismatched_target_and_missing_asset_fail_before_formal_delivery(self):
        row = self.source()
        target = json.loads(json.dumps(self.targets["CH1"]))
        target["original"]["measurement"]["channelId"] = "CH2"
        with self.assertRaises(ValueError):
            calculate_revision(self.bridge, row, target, encode_exit(()))
        self.assertFalse(self.sent)

    def test_other_plate_node_or_day_cannot_supply_retrospective_baseline(self):
        row = self.source()
        for field, value in (("licensePlate", "宁A52522S"), ("nodeId", "other-node")):
            target = json.loads(json.dumps(self.targets["CH1"]))
            target["baseline"][field] = value
            with self.assertRaises(ValueError):
                calculate_revision(self.bridge, row, target, encode_exit(()))
        target = json.loads(json.dumps(self.targets["CH1"]))
        target["baselineSource"]["measuredAt"] = "2026-10-22T08:00:00+08:00"
        with self.assertRaises(ValueError):
            calculate_revision(self.bridge, row, target, encode_exit(()))

    def test_missing_input_reports_failure_without_fake_volume(self):
        self.source()
        with self.store._transaction("CH1") as db:
            db.execute("DELETE FROM pending_tasks")
        with self.assertRaisesRegex(ValueError, "留样已不存在"):
            self.processor.process_once("CH1")
        self.assertFalse(self.sent)

    def test_failed_calculation_keeps_source_and_does_not_queue_invalid_formal_payload(self):
        row = self.source()
        def unavailable(*args, **kwargs):
            pending = loaded_label(calculate_frozen(*args, **kwargs))
            value = json.loads(pending.result_json)
            value["volume"].update(ok=False, volume_m3=None)
            return replace(pending, result_json=_json_bytes(value))
        with patch("app.recalculation.calculate_frozen", side_effect=unavailable):
            with self.assertRaisesRegex(ValueError, "未得到可发布体积"):
                self.processor.process_once("CH1")
        self.assertIsNone(self.store.pending_recalculation("CH1"))
        self.assertFalse(self.sent)
        self.assertIn(row.identity.measurement_id, self.store.protected_measurements(CAPTURE_TS + 86400))

    def test_used_enter_and_exit_frames_extend_actual_interval_without_forging_measurement_time(self):
        row = self.source()
        pending = loaded_label(calculate_frozen(row.input_bytes, row.identity, None, "宁A05922F", self.config))
        pending = replace(pending, entry_frame_times=(CAPTURE_TS - 2,), exit_frame_times=(CAPTURE_TS + 2,))
        payload = json.loads(prepare_result(pending, self.bridge).payload_json)
        parse = lambda value: datetime.fromisoformat(value.replace("Z", "+00:00")).timestamp()
        self.assertEqual(CAPTURE_TS - 2, parse(payload["measurementStartedAt"]))
        self.assertEqual(CAPTURE_TS + 2, parse(payload["measurementEndedAt"]))
        self.assertEqual(self.targets["CH1"]["original"]["measurement"]["measuredAt"], payload["measuredAt"])

    def test_old_source_hash_does_not_block_normal_resume_or_revision(self):
        row = self.source()
        value = json.loads(row.input_bytes)
        value["algorithm"]["service"] = "0" * 64
        original = _json_bytes(value)
        row = replace(row, input_bytes=original, input_sha256=hashlib.sha256(original).hexdigest())
        resumed = calculate_frozen(original, row.identity, None, "宁A05922F", self.config)
        self.assertEqual(row.identity, resumed.identity)
        self.assertEqual("宁A05922F", json.loads(resumed.result_json)["plate"])
        payload = json.loads(calculate_revision(self.bridge, row, self.targets["CH1"], encode_exit(())))
        proof = payload["provenance"]
        self.assertEqual(row.input_sha256, proof["inputSha256"])
        self.assertNotEqual(proof["originalAlgorithmSha256"], proof["calculationAlgorithmSha256"])

    def test_revision_uses_original_calibration_even_if_running_node_config_changed(self):
        row = self.source()
        from app.load_radar_transport import ChannelConfig
        self.bridge.config = replace(self.config, algorithm_version="new-running-build",
            channels=dict(self.config.channels, CH1=ChannelConfig("new-running-calibration")))
        payload = json.loads(calculate_revision(self.bridge, row, self.targets["CH1"], encode_exit(())))
        self.assertEqual("cal-1", payload["measurement"]["calibrationId"])
        self.assertEqual("test-algorithm", payload["measurement"]["algorithmVersion"])

    def test_legacy_completed_input_is_pinned_and_indexed_offline_before_cleanup(self):
        row = self.source()
        original = loaded_label(calculate_frozen(row.input_bytes, row.identity, None, "宁A05922F", self.config))
        prepared = prepare_result(original, self.bridge)
        from app.deferred_measurements import pack_prepared
        self.store.freeze_preparation("CH1", row.identity.measurement_id, b"original-proof")
        self.store.freeze_prepared("CH1", row.identity.measurement_id, pack_prepared(original.result_json, prepared))
        with self.store._transaction("CH1") as db:
            db.execute("DELETE FROM recalculation_sources")
        self.assertIn(row.identity.measurement_id, self.store.protected_measurements(CAPTURE_TS + 86400))
        with patch.object(self.client, "recalculations", side_effect=AssertionError("索引恢复不需要网络")):
            self.assertTrue(self.processor.process_once("CH1"))
        self.assertEqual("2026-10-21", self.store.recalculation_since("CH1"))
