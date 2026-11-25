# -*- coding: utf-8 -*-
"""冻结车次准备的无网络测试；不构造线程、SQLite 或磁盘基线缓存。"""
import base64
from copy import deepcopy
from dataclasses import replace
from datetime import datetime, timedelta, timezone
import hashlib
import json
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from app.load_radar_bridge import BridgeConflict, LoadRadarBridge
from app.load_radar_contract import BaselineCache
from app.load_radar_transport import _json_bytes
from tests.test_load_radar_bridge import MemoryClient
from tests import test_load_radar_preparation as preparation_fixture


class TestFrozenPreparation(unittest.TestCase):
    def setUp(self):
        # 只借正式方法；绕过构造器的队列、文件锁和投影线程。
        self.bridge = object.__new__(LoadRadarBridge)
        self.bridge.config = SimpleNamespace(node_id="jt-test", channels={
            "CH1": SimpleNamespace(calibration_id="cal-1")})
        self.bridge._lock = threading.RLock()
        self.bridge._frames = {"CH1": time.monotonic()}
        self.bridge._lanes = {"CH1": dict(channel="CH1", ready=0, passage=None, measurement=None)}
        for name in ("_starts", "_idle", "_loaded", "_prepared", "_reserved", "_uncommitted",
                     "_commands", "_channel_contexts", "_standstill_observed", "_standstill_pending",
                     "_standstill_done"):
            setattr(self.bridge, name, {})
        self.bridge.cache = BaselineCache
        self.bridge.outbox = Mock()
        self.client = MemoryClient()
        self.bridge.client = self.client
        self.reply_hook = lambda row: row
        self.query_calls = []
        preparation_fixture.TestLoadRadarPreparation.configure(self)
        self.version["active"] = False
        self.at = datetime.now(timezone.utc).replace(microsecond=123456)
        self.measured_at = self.at.timestamp()
        self.stamp = self.at.isoformat().replace("+00:00", "Z")
        self.set_current("passage-a")
        self.identity, self.start_json = self.bridge.capture_identity("CH1", "origin-a")
        self.start = json.loads(self.start_json)

    def respond(self, payload, *, measured_at=None):
        self.query_calls.append((_json_bytes(payload), measured_at))
        row = preparation_fixture.TestLoadRadarPreparation.respond(self, payload)
        row["baselineAsOf"] = (None if measured_at is None else
            datetime.fromtimestamp(measured_at, timezone.utc).isoformat().replace("+00:00", "Z"))
        return self.reply_hook(row)

    def set_current(self, passage):
        self.bridge._frames["CH1"] = time.monotonic()
        self.bridge._lanes["CH1"].update(ready=1, passage=passage, measurement=None)
        self.bridge._starts["CH1"] = dict(nodeId="jt-test", channelId="CH1", passageId=passage,
            calibrationId="cal-1", configurationHash="a" * 64, sourceMode="live",
            arrivedAt=(self.at - timedelta(seconds=10)).isoformat().replace("+00:00", "Z"))

    def prepare(self):
        return self.bridge.prepare_frozen(self.identity, self.start_json, self.measured_at)

    def restore(self, proof, *, identity=None, start=None, measured_at=None):
        return self.bridge.frozen_preparation(identity or self.identity,
            self.start_json if start is None else start,
            self.measured_at if measured_at is None else measured_at, proof)

    def assert_no_measurement(self):
        self.assertEqual(self.client.posts, [])
        self.bridge.outbox.save.assert_not_called()

    def test_capture_identity_owns_start_bytes_without_reserving_channel(self):
        self.assertEqual(self.identity.passage_id, "passage-a")
        self.assertEqual(self.identity.channel, "CH1")
        self.assertEqual(self.identity.origin_token, "origin-a")
        self.assertEqual(self.bridge._reserved, {})
        self.set_current("passage-b")
        self.assertEqual(json.loads(self.start_json)["passageId"], "passage-a")
        other, other_start = self.bridge.capture_identity("CH1", "origin-b")
        self.assertNotEqual(other.measurement_id, self.identity.measurement_id)
        self.assertEqual(json.loads(other_start)["passageId"], "passage-b")
        self.assert_no_measurement()

    def test_capture_identity_rejects_quarantined_or_mismatched_start(self):
        self.bridge._lanes["CH1"]["ready"] = 0
        with self.assertRaises(BridgeConflict):
            self.bridge.capture_identity("CH1", "blocked")
        self.bridge._lanes["CH1"]["ready"] = 1
        self.bridge._starts["CH1"]["passageId"] = "wrong"
        with self.assertRaises(BridgeConflict):
            self.bridge.capture_identity("CH1", "blocked")
        self.assertEqual(self.bridge._reserved, {})

    def test_old_a_preparation_preserves_b_and_does_not_write_active_cache(self):
        self.set_current("passage-b")
        prepared = ("passage-b", "ready", {"licensePlate": "宁A27066"}, time.monotonic())
        loaded = ("passage-b", "当前车基线")
        self.bridge._prepared["CH1"], self.bridge._loaded["CH1"] = prepared, loaded
        before_lane, before_start = deepcopy(self.bridge._lanes), deepcopy(self.bridge._starts)
        status, proof = self.prepare()
        snapshot, plate = self.restore(proof)
        self.assertEqual((status, plate), ("ready", "宁A25771"))
        self.assertEqual(snapshot.passage_id, "passage-a")
        self.assertEqual(snapshot.binding["configuration_sha256"], "a" * 64)
        self.assertEqual(snapshot.binding["baseline_version_id"], "v1")
        self.assertEqual(snapshot.asset_bytes, self.client.assets["v1"])
        self.assertEqual(self.bridge._lanes, before_lane)
        self.assertEqual(self.bridge._starts, before_start)
        self.assertIs(self.bridge._prepared["CH1"], prepared)
        self.assertIs(self.bridge._loaded["CH1"], loaded)
        self.assertEqual(self.bridge._reserved, {})
        self.assert_no_measurement()

    def test_inactive_history_is_accepted_without_weakening_current_selection(self):
        _, proof = self.prepare()
        receipt = json.loads(proof)["receipt"]
        self.assertIs(receipt["baseline"]["active"], False)
        self.assertIs(self.version["active"], False)
        self.assertIsNone(BaselineCache.select([receipt["baseline"]], receipt["binding"],
            "jt-test", "CH1", "cal-1"))
        self.assertIsNotNone(self.restore(proof)[0])

    def test_missing_or_wrong_as_of_never_produces_proof(self):
        for value in (None, "2026-01-01T00:00:00Z", self.stamp[:-1] + "1Z"):
            with self.subTest(value=value):
                self.reply_hook = lambda row, value=value: dict(row, baselineAsOf=value)
                with self.assertRaises(ValueError):
                    self.prepare()
        self.reply_hook = lambda row: {k: v for k, v in row.items() if k != "baselineAsOf"}
        with self.assertRaises(ValueError):
            self.prepare()
        self.assert_no_measurement()

    def test_wrong_workflow_camera_passage_or_vehicle_binding_is_rejected(self):
        hooks = (
            lambda row: dict(row, workflowId="wrong"),
            lambda row: dict(row, binding=dict(row["binding"], workflowId="wrong")),
            lambda row: dict(row, binding=dict(row["binding"], passageKey="f" * 64)),
            lambda row: dict(row, binding=dict(row["binding"], cameraIdentifier="CAM_002")),
            lambda row: dict(row, binding=dict(row["binding"], vehicleIdentity="plate:宁A27066")),
        )
        for index, hook in enumerate(hooks):
            with self.subTest(index=index):
                self.reply_hook = hook
                with self.assertRaises(ValueError):
                    self.prepare()
        self.assert_no_measurement()

    def test_wrong_source_configuration_or_identity_is_rejected(self):
        origin = self.client.rows["empty-baseline"]["measurement"]
        for key, value in (("nodeId", "other"),
                           ("channelId", "CH2"), ("calibrationId", "cal-2"),
                           ("baselineCandidateSha256", "c" * 64), ("sourceMode", "replay"),
                           ("verdict", "loaded"), ("frameDegraded", True)):
            with self.subTest(key=key):
                with patch.dict(origin, {key: value}):
                    with self.assertRaises(ValueError):
                        self.prepare()
        self.assert_no_measurement()

    def test_wrong_asset_sha_or_invalid_npz_is_rejected(self):
        self.client.assets["v1"] += b"damaged"
        with self.assertRaises(ValueError):
            self.prepare()
        invalid = b"invalid npz"
        digest = hashlib.sha256(invalid).hexdigest()
        self.client.assets["v1"] = invalid
        self.version["assetSha256"] = digest
        self.client.rows["empty-baseline"]["measurement"]["baselineCandidateSha256"] = digest
        with self.assertRaises(ValueError):
            self.prepare()
        self.assert_no_measurement()

    def test_origin_and_asset_proof_cannot_be_replaced_after_prepare(self):
        _, raw = self.prepare()
        for field, value in (("origin", {}), ("asset", base64.b64encode(b"wrong").decode("ascii")),
                              ("asset", "not base64")):
            with self.subTest(field=field, value=value):
                proof = json.loads(raw)
                proof[field] = value
                with self.assertRaises(ValueError):
                    self.restore(_json_bytes(proof))

    def test_as_of_and_activation_submicrosecond_boundaries_are_exact(self):
        self.reply_hook = lambda row: dict(row, baselineAsOf=self.stamp[:-1] + "0Z")
        self.assertEqual(self.prepare()[0], "ready")
        self.version["activatedAt"] = self.stamp[:-1] + "1Z"
        with self.assertRaises(ValueError):
            self.prepare()
        earlier = self.at - timedelta(microseconds=1)
        self.version["activatedAt"] = earlier.isoformat().replace("+00:00", "9Z")
        self.assertEqual(self.prepare()[0], "ready")

    def test_baseline_required_freezes_no_baseline_at_original_time(self):
        self.mode = "baseline_required"
        status, proof = self.prepare()
        self.assertEqual(status, "baseline_required")
        self.assertEqual(self.restore(proof), (None, "宁A25771"))
        self.client.get_measurement = Mock(side_effect=AssertionError("恢复不得查询新基线"))
        self.client.download_baseline = Mock(side_effect=AssertionError("恢复不得下载新基线"))
        self.mode = "ready"
        self.set_current("passage-b")
        self.assertEqual(self.restore(proof), (None, "宁A25771"))
        self.assert_no_measurement()

    def test_missing_baseline_cannot_smuggle_origin_or_asset(self):
        self.mode = "baseline_required"
        _, raw = self.prepare()
        for field, value in (("origin", {}), ("asset", "YQ==")):
            with self.subTest(field=field):
                proof = json.loads(raw)
                proof[field] = value
                with self.assertRaises(ValueError):
                    self.restore(_json_bytes(proof))

    def test_waiting_or_unavailable_plate_never_creates_proof_or_measurement(self):
        self.client.get_measurement = Mock(side_effect=AssertionError("等待车牌不能读基线"))
        self.client.download_baseline = Mock(side_effect=AssertionError("等待车牌不能下载资产"))
        for status in ("awaiting_plate", "plate_unavailable"):
            with self.subTest(status=status):
                self.mode = status
                self.assertEqual(self.prepare(), (status, None))
                self.assert_no_measurement()
        self.assertEqual(self.bridge._reserved, {})

    def test_pending_receipt_cannot_include_authority_or_become_frozen_proof(self):
        self.mode = "awaiting_plate"
        self.reply_hook = lambda row: dict(row, baseline=dict(self.version))
        with self.assertRaises(ValueError):
            self.prepare()
        self.reply_hook = lambda row: row
        receipt = self.respond(self.start, measured_at=self.measured_at)
        proof = dict(schema="jt128-historical-preparation-v1", receipt=receipt, origin=None, asset=None)
        with self.assertRaises(ValueError):
            self.restore(_json_bytes(proof))

    def test_retry_keeps_frozen_query_and_original_body(self):
        first = self.prepare()
        self.set_current("passage-b")
        second = self.prepare()
        self.assertEqual(first, second)
        self.assertEqual(self.query_calls, [(self.start_json, self.measured_at)] * 2)
        self.assertEqual(self.requests, [self.start, self.start])

    def test_proof_cannot_be_restored_with_changed_query_body_or_identity(self):
        _, proof = self.prepare()
        with self.assertRaises(ValueError):
            self.restore(proof, measured_at=self.measured_at + 1)
        for key, value in (("arrivedAt", "2026-01-01T00:00:00Z"), ("configurationHash", "b" * 64),
                           ("passageId", "passage-b"), ("channelId", "CH2"), ("nodeId", "other")):
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.restore(proof, start=_json_bytes(dict(self.start, **{key: value})))
        with self.assertRaises(ValueError):
            self.restore(proof, identity=replace(self.identity, passage_id="passage-b"))

    def test_restore_uses_only_original_bytes_when_remote_or_current_state_changes(self):
        _, proof = self.prepare()
        self.client.prepare_passage = Mock(side_effect=AssertionError("恢复不得重发 prepare"))
        self.client.get_measurement = Mock(side_effect=AssertionError("恢复不得刷新来源"))
        self.client.download_baseline = Mock(side_effect=AssertionError("恢复不得刷新资产"))
        self.client.assets.clear()
        self.client.rows.clear()
        self.set_current("passage-b")
        snapshot, plate = self.restore(proof)
        self.assertEqual((snapshot.passage_id, plate), ("passage-a", "宁A25771"))
        self.assert_no_measurement()

    def test_ch4_source_calibration_update_does_not_change_frozen_original_proof(self):
        self.version["calibrationId"] = "ch4-survey-v1"
        self.client.rows["empty-baseline"]["measurement"].update(calibrationId="ch4-survey-v1", configurationHash="d" * 64)
        _, first = self.prepare()
        snapshot, _ = self.restore(first)
        self.assertEqual("CH1", snapshot.binding["channel_id"])
        self.assertEqual("cal-1", snapshot.binding["calibration_id"])
        self.assertEqual("ch4-survey-v1", json.loads(snapshot.source_json)["sourceCalibrationId"])
        self.version.update(versionId="v2", calibrationId="ch4-survey-v2")
        self.client.rows["empty-baseline"]["measurement"].update(calibrationId="ch4-survey-v2", configurationHash="e" * 64)
        self.client.assets["v2"] = self.client.assets["v1"]
        _, second = self.prepare()
        self.assertEqual("v2", self.restore(second)[0].binding["baseline_version_id"])
        cold = object.__new__(LoadRadarBridge)
        cold.config, cold.cache = self.bridge.config, BaselineCache
        restored, _ = cold.frozen_preparation(self.identity, self.start_json, self.measured_at, first)
        self.assertEqual("v1", restored.binding["baseline_version_id"])
        self.assertEqual(snapshot.source_json, restored.source_json)

    def test_adopt_is_idempotent_and_does_not_replace_current_vehicle(self):
        self.set_current("passage-b")
        before = deepcopy(self.bridge._lanes)
        self.bridge.adopt_deferred(self.identity)
        self.bridge.adopt_deferred(self.identity)
        self.assertEqual(self.bridge._reserved, {"CH1": self.identity})
        self.assertEqual(self.bridge._lanes, before)
        self.assertEqual(self.bridge.current_passage("CH1"), "passage-b")
        self.assert_no_measurement()

    def test_adopt_rejects_conflicting_pending_results(self):
        other = replace(self.identity, measurement_id="m-other")
        self.bridge._reserved["CH1"] = other
        with self.assertRaises(BridgeConflict):
            self.bridge.adopt_deferred(self.identity)
        self.assertIs(self.bridge._reserved["CH1"], other)
        self.bridge._reserved.clear()
        self.bridge._uncommitted["CH1"] = object()
        with self.assertRaises(BridgeConflict):
            self.bridge.adopt_deferred(self.identity)
        self.assertEqual(self.bridge._reserved, {})

    def test_adopt_rejects_missing_or_unconfigured_identity(self):
        for identity in (replace(self.identity, passage_id=None), replace(self.identity, channel="CH2")):
            with self.subTest(identity=identity), self.assertRaises(BridgeConflict):
                self.bridge.adopt_deferred(identity)
        self.assertEqual(self.bridge._reserved, {})


if __name__ == "__main__":
    unittest.main()
