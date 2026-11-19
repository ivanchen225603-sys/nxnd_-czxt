"""冻结任务只读离线验证；不用实时源、HTTP、数据库或现场目录。"""
from dataclasses import replace
from datetime import datetime, timezone
import base64
import copy
import hashlib
import io
import json
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from app.bed_reference import baseline_digest
from app.cloud_observation import FrameSnapshot, ObservationPair, ObservationWindow
from app.detect import StabilityDetector, StabilityResult
from app.frozen_detection import FrozenDetection, calculate_frozen, inspect_frozen
from app.load_radar_bridge import BaselineSnapshot, ResultReservation
from app.load_radar_contract import _read_baseline, effective_configuration_bytes
from app.measurement_runtime import canonical
from app.pointcloud import AngleCache, Frame, apply_roi
from app.service import ChannelWorker
from app.settings import CalibCfg, ChannelCfg, DevicesCfg, config_fingerprint
from app.volume_provider import VolumeCatalog


CAPTURE_TS = 1792584000.


def fixture(*, merged=False, save=True):
    calib, dev, ch, angle = CalibCfg(), DevicesCfg(), ChannelCfg(id="CH4", name="离线冻结验证"), AngleCache()
    calib.roi = ((-2., -6., -5.5), (2., 6., .5))
    calib.detection.settle_sec = .5
    calib.detection.settle_accumulate = merged
    calib.detection.vehicle_threshold = 2000
    dev.min_save_points = 1
    config = SimpleNamespace(node_id="test-node", algorithm_version="test-algorithm",
                             channels={"CH4": SimpleNamespace(calibration_id="cal-4")})
    x, y = np.meshgrid((np.arange(24) + .5) * .1 - 1.2, (np.arange(100) + .5) * .1 - 5.)
    cloud = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, -3.4))).astype(np.float32)
    pairs, cropped = [], []
    for index in range(3):
        raw = Frame(np.vstack((cloud, [9., 9., 9.])).astype(np.float32),
                    np.arange(len(cloud) + 1, dtype=np.uint16) % 128, index + 1, CAPTURE_TS + index * .1, 4, 901)
        frame = apply_roi(raw, calib.roi)
        pairs.append(ObservationPair.freeze(channel="CH4", epoch=7, received_monotonic=1. + index * .1,
                                            complete=True, raw=raw, cropped=frame, roi=calib.roi))
        cropped.append(frame)
    primary = ObservationWindow(tuple(pairs) if merged else (pairs[-1],))
    frame = primary.input_frame() if merged else cropped[-1]
    worker = SimpleNamespace(calib=calib, dev=dev, ch=ch, angle=angle,
        integration=SimpleNamespace(config=config), volume_catalog=None, _capture_instance_id="capture-original",
        _observation_epoch=7, _session_id=19, _session_forced=True, _last_save_name="m-previous",
        _detected_this_session=0, _degraded_measure=False, save_enabled=save, eff_mode="live",
        _measurement_window=primary, _volume_observation_window=ObservationWindow(tuple(pairs)),
        _volume_extra_members=(0, 1), _vol_extra=[f.xyz for f in cropped[:2]], _settle_frames=cropped,
        _raw_shots=[(.7, pairs[0].raw.frame())], _last_frame_monotonic=pairs[-1].received_monotonic,
        stability=StabilityDetector(vehicle_threshold=2000), _config_fp=config_fingerprint(calib, dev))
    worker._effective_configuration = effective_configuration_bytes(calib, dev, ch, angle)
    worker._integration_hash = hashlib.sha256(worker._effective_configuration).hexdigest()
    stab = StabilityResult(state=2, stable=True, n_points=len(cloud), fresh=True, n_vehicle_points=len(cloud),
                           stationary_sec=3., speed_mps=0., motion_observable=True)
    decision = ChannelWorker._gate_decision(worker, cropped[-1], stab, True)
    worker._measurement_gate = dict(decision=decision, manual_three_frames=[],
        settle=dict(kind="automatic", applicable=True, started_monotonic=0., finished_monotonic=1.2,
                    waited_sec=1.2, required_sec=.5, accumulate=merged))
    identity = ResultReservation("m-original", "CH4", "p-original", "token-original")
    start = dict(nodeId=config.node_id, channelId="CH4", passageId=identity.passage_id, calibrationId="cal-4",
                 configurationHash=worker._integration_hash, sourceMode="live",
                 arrivedAt=datetime.fromtimestamp(CAPTURE_TS - 1, timezone.utc).isoformat().replace("+00:00", "Z"))
    return worker, frame, stab, identity, start


def baseline_and_model(worker, identity):
    cloud = worker._measurement_window.pairs[-1].cropped.frame().xyz.astype(np.float64).copy()
    cloud[:, 2] = -3.8
    stream = io.BytesIO()
    np.savez_compressed(stream, floor_z=-3.8, box=np.array([-1.2, 1.2, -5., 5., -3.8, -.8]),
                        cloud=cloud.astype(np.float32), bed_plane=np.array([0., 0., 1., 3.8]), created="test")
    asset = stream.getvalue()
    base = _read_baseline(asset)
    binding = dict(node_id="test-node", channel_id="CH4", calibration_id="cal-4",
        configuration_sha256=worker._integration_hash, baseline_version_id="version-original",
        baseline_asset_sha256=hashlib.sha256(asset).hexdigest(), baseline_sha256=baseline_digest(base))
    snapshot = BaselineSnapshot(identity.passage_id, canonical(binding), base.cloud.astype("<f8").tobytes(),
                                base.floor_z, tuple(base.box), base.bed_plane, asset)
    model = dict(model_id="model-original", binding=binding, reference_id="synthetic-survey",
        coordinate_frame="baseline-heading-aligned-m-v1", valid_from=CAPTURE_TS - 1, valid_until=CAPTURE_TS + 10,
        domain_xy=[-1.2, 1.2, -5., 5.], domain_source="synthetic_model", max_height_m=3.,
        height_source="synthetic_model", support_cell_m=.1, support_origin_xy=[-1.2, -5.],
        support_model="cell_mean_max_relative_height", error_source="synthetic_model", surface_error_m=0.,
        bed_error_m=0., roi_error_area_m2=0., conditions_status="declared_model",
        registration=dict(mode="fixed", dy_m=0.))
    worker.volume_catalog = VolumeCatalog.from_bytes(canonical(dict(schema="volume-provider-v1", models=[model])))
    return snapshot


class TestFrozenDetection(unittest.TestCase):
    def test_actual_numpy_stability_scalars_use_exact_json_primitives(self):
        worker, frame, stab, identity, start = fixture()
        stab.speed_mps = np.float64(0.)
        stab.drift_m = np.float64(0.)
        worker._measurement_gate["decision"]["stability"]["speed_mps"] = stab.speed_mps
        worker._measurement_gate["decision"]["stability"]["drift_m"] = stab.drift_m
        data = FrozenDetection.freeze(worker, frame, stab, identity, start)
        self.assertIs(type(FrozenDetection.restore(data).record["gate"]["decision"]["stability"]["speed_mps"]), float)

    def freeze(self, **kwargs):
        worker, frame, stab, identity, start = fixture(**kwargs)
        data = FrozenDetection.freeze(worker, frame, stab, identity, start)
        return data, worker, frame, stab, identity, start

    def test_round_trip_preserves_raw_roi_ring_id_and_configuration(self):
        data, worker, frame, _, identity, _ = self.freeze()
        frozen = FrozenDetection.restore(data)
        self.assertEqual(FrameSnapshot.freeze(frame), frozen.frame)
        self.assertEqual(worker._measurement_window, frozen.primary)
        self.assertEqual(worker._volume_observation_window, frozen.support)
        self.assertEqual(worker._effective_configuration, frozen.configuration)
        info = inspect_frozen(data)
        self.assertEqual(identity.measurement_id, info["identity"]["measurement_id"])
        self.assertEqual(frame.ts, info["measured_at"])
        self.assertEqual(worker._integration_hash, info["configuration_hash"])
        self.assertEqual(2401, frozen.primary.pairs[-1].raw.frame().n)
        self.assertEqual(2400, frozen.primary.pairs[-1].cropped.frame().n)
        self.assertFalse(frozen.frame.frame().xyz.flags.writeable)

    def test_two_stage_draft_owns_no_live_mutable_context(self):
        worker, frame, stab, identity, start = fixture()
        snapshot = baseline_and_model(worker, identity)
        draft = FrozenDetection.capture(worker, frame, stab, identity, start)
        old_config = copy.deepcopy(worker.integration.config)
        expected_frame = FrameSnapshot.freeze(frame)
        worker.calib.detection.ground_z = 99.
        worker._measurement_gate.clear()
        worker._settle_frames.clear()
        worker._raw_shots.clear()
        worker.volume_catalog = None
        frame.xyz[:] = 99.
        start["passageId"] = "next-car"
        self.assertIsInstance(draft.metadata_json, bytes)
        self.assertTrue(all(isinstance(f, FrameSnapshot) for f in draft.frames))
        self.assertTrue(all(type(item) is bytes for item in draft.indices + draft.models))
        data = draft.encode()
        self.assertGreater(draft.byte_size(), len(data))
        self.assertEqual(expected_frame, FrozenDetection.restore(data).frame)
        pending = calculate_frozen(data, identity, snapshot, "原车", old_config)
        self.assertEqual(identity, pending.identity)

    def test_real_algorithm_uses_frozen_context_after_live_worker_is_replaced(self):
        worker, frame, stab, identity, start = fixture()
        snapshot = baseline_and_model(worker, identity)
        data = FrozenDetection.freeze(worker, frame, stab, identity, start)
        old_config = copy.deepcopy(worker.integration.config)
        expected = calculate_frozen(data, identity, snapshot, "宁A05922F", old_config)
        worker._session_id = 999
        worker.ch.name = "下一车"
        worker.calib.detection.ground_z = 200.
        worker._measurement_gate.clear()
        worker._vol_extra.clear()
        worker.volume_catalog = None
        frame.xyz[:] = 99.
        with patch("app.application.channel_worker.load_calib", side_effect=AssertionError("读取现场配置")), \
             patch("app.application.channel_worker.load_devices", side_effect=AssertionError("读取现场配置")), \
             patch("app.application.channel_worker.build_source", side_effect=AssertionError("创建数据源")), \
             patch.object(AngleCache, "load", side_effect=AssertionError("读取角度文件")), \
             patch.object(VolumeCatalog, "load", side_effect=AssertionError("读取模型文件")), \
             patch("socket.socket", side_effect=AssertionError("网络调用")):
            actual = calculate_frozen(data, identity, snapshot, "宁A05922F", old_config)
        a, b = json.loads(actual.result_json), json.loads(expected.result_json)
        a.pop("elapsed_ms"), b.pop("elapsed_ms")
        self.assertEqual(a, b)
        self.assertEqual(19, a["session"])
        self.assertEqual("宁A05922F", a["plate"])
        self.assertEqual("m-previous", a["duplicate_of"])
        self.assertEqual("version-original", actual.baseline_version_id)
        self.assertEqual("supplied", a["volume_runtime"]["status"])
        self.assertAlmostEqual(9.6, a["volume"]["volume_m3"], places=4)
        self.assertEqual("m-original", actual.identity.measurement_id)
        self.assertIn("baseline.npz", dict(actual.runtime_files))

    def test_no_baseline_and_disabled_archive_still_returns_original_identity(self):
        data, worker, _, _, identity, _ = self.freeze(save=False)
        pending = calculate_frozen(data, identity, None, "冻结车牌", worker.integration.config)
        self.assertFalse(pending.archive_enabled)
        self.assertIsNone(pending.baseline_version_id)
        self.assertEqual(identity, pending.identity)
        self.assertEqual((), pending.runtime_files)

    def test_actual_payload_and_archive_construct_without_io(self):
        from app.result_finalizer import prepare_result
        worker, frame, stab, identity, start = fixture()
        snapshot = baseline_and_model(worker, identity)
        worker.integration.config.agent_key = "TEST-SECRET-MUST-NOT-ENTER-FROZEN-BYTES"
        data = FrozenDetection.freeze(worker, frame, stab, identity, start)
        self.assertNotIn(worker.integration.config.agent_key.encode(), data)
        pending = calculate_frozen(data, identity, snapshot, "宁A05922F", worker.integration.config)
        prepared = prepare_result(pending, SimpleNamespace(config=worker.integration.config))
        payload = json.loads(prepared.payload_json)
        self.assertEqual(identity.measurement_id, payload["measurementId"])
        self.assertEqual(identity.passage_id, payload["passageId"])
        self.assertEqual("version-original", payload["baselineVersionId"])
        self.assertIsNotNone(prepared.archive)
        files = dict(prepared.archive.files)
        gate = json.loads(files["gate-summary.json"])
        self.assertEqual(identity.measurement_id, gate["identity"]["measurement_id"])
        self.assertEqual(frame.ts, gate["decision"]["frame"]["ts"])
        self.assertEqual(snapshot.asset_bytes, files["baseline.npz"])
        self.assertEqual(worker._effective_configuration, files["effective-configuration.json"])

    def _enter(self, worker, n=3):
        from app.cloud_observation import FrameSnapshot
        cloud = worker._measurement_window.pairs[-1].cropped.frame().xyz
        return tuple(FrameSnapshot((cloud + [0., -.3 * (n - k), 0.]).astype("<f4").tobytes(), None,
                                   k + 1 - n - 1 + 1000, CAPTURE_TS - 5 + k, 4, 901) for k in range(n))

    def test_declared_model_keeps_frozen_input_but_omits_unused_archive_entry(self):
        """留样仍保全原输入；正式模型未用的进场帧不冒充归档积分证据。"""
        from app.result_finalizer import prepare_result
        worker, frame, stab, identity, start = fixture()
        snapshot = baseline_and_model(worker, identity)
        worker._enter_volume = self._enter(worker)
        data = FrozenDetection.freeze(worker, frame, stab, identity, start)
        frozen = FrozenDetection.restore(data)
        self.assertEqual(worker._enter_volume, frozen.enter)
        pending = calculate_frozen(data, identity, snapshot, "宁A05922F", worker.integration.config)
        files = dict(prepare_result(pending, SimpleNamespace(config=worker.integration.config)).archive.files)
        call = json.loads(files["volume-call.json"])
        self.assertEqual("volume-call-v3", call["schema"])
        self.assertIsNone(call["enter"])
        self.assertIsNone(call["exit"])
        self.assertNotIn("enter-frames.f32le", files)
        self.assertNotIn("exit-frames.f32le", files)
        self.assertEqual((), pending.entry_frame_times)

    def test_non_declared_entry_frames_extend_interval_even_without_archive(self):
        from app.result_finalizer import prepare_result
        worker, frame, stab, identity, start = fixture(save=False)
        snapshot = baseline_and_model(worker, identity)
        worker.volume_catalog = None
        worker._enter_volume = self._enter(worker)
        data = FrozenDetection.freeze(worker, frame, stab, identity, start)
        pending = calculate_frozen(data, identity, snapshot, "宁A05922F", worker.integration.config)
        self.assertEqual(tuple(item.ts for item in worker._enter_volume), pending.entry_frame_times)
        prepared = prepare_result(pending, SimpleNamespace(config=worker.integration.config))
        payload = json.loads(prepared.payload_json)
        started = datetime.fromisoformat(payload["measurementStartedAt"].replace("Z", "+00:00")).timestamp()
        ended = datetime.fromisoformat(payload["measurementEndedAt"].replace("Z", "+00:00")).timestamp()
        self.assertEqual(min(item.ts for item in worker._enter_volume), started)
        self.assertEqual(frame.ts, ended)
        self.assertIsNone(prepared.archive)

    def test_enter_frames_never_push_the_frozen_task_over_budget(self):
        """预算不够时从最早的帧开始舍，留样本身照常成功。"""
        from app import frozen_detection
        worker, frame, stab, identity, start = fixture()
        worker._enter_volume = self._enter(worker, 4)
        one = len(worker._enter_volume[0].xyz_bytes)
        with patch.object(frozen_detection, "ENTER_FROZEN_BUDGET", one * 2 + 1):
            frozen = FrozenDetection.restore(FrozenDetection.freeze(worker, frame, stab, identity, start))
        self.assertEqual(worker._enter_volume[-2:], frozen.enter)

    def test_enter_frames_after_the_standstill_window_are_refused(self):
        from dataclasses import replace
        worker, frame, stab, identity, start = fixture()
        late = self._enter(worker)
        worker._enter_volume = late[:-1] + (replace(late[-1], ts=CAPTURE_TS + 1),)
        with self.assertRaises(ValueError):
            FrozenDetection.restore(FrozenDetection.freeze(worker, frame, stab, identity, start))

    def test_merged_primary_keeps_original_members_and_diagnostic_frame_identity(self):
        data, worker, frame, _, identity, _ = self.freeze(merged=True)
        self.assertEqual(7200, frame.n)
        pending = calculate_frozen(data, identity, None, "冻结车牌", worker.integration.config)
        self.assertEqual(3, len(pending.observation_window.pairs))
        self.assertEqual((0, 1), pending.volume_extra_members)
        self.assertEqual(1, len(pending.extra_frames))

    def test_mismatched_identity_node_or_plate_refused(self):
        data, worker, _, _, identity, _ = self.freeze()
        for wrong in (replace(identity, passage_id="next-car"), replace(identity, measurement_id="next-result")):
            with self.assertRaises(ValueError):
                calculate_frozen(data, wrong, None, "原车", worker.integration.config)
        changed = copy.deepcopy(worker.integration.config)
        changed.node_id = "next-node"
        with self.assertRaises(ValueError):
            calculate_frozen(data, identity, None, "原车", changed)
        for plate in (None, "", "  "):
            with self.assertRaises(ValueError):
                calculate_frozen(data, identity, None, plate, worker.integration.config)

    def test_old_source_hashes_do_not_block_restored_frozen_measurement(self):
        worker, frame, stab, identity, start = fixture()
        snapshot = baseline_and_model(worker, identity)
        data = FrozenDetection.freeze(worker, frame, stab, identity, start)
        expected = calculate_frozen(data, identity, snapshot, "宁A05922F", worker.integration.config)
        record = json.loads(data)
        self.assertNotIn("fingerprint", record["algorithm"])
        record["algorithm"] = dict(volume={**record["algorithm"], "fingerprint": "f" * 64},
                                   service="0" * 64, adapter="1" * 64)
        # 重新解码旧格式留样，计算只依赖原输入，不依赖旧 Python 源文件。
        restored = FrozenDetection.restore(canonical(record))
        actual = calculate_frozen(canonical(record), identity, snapshot, "宁A05922F",
                                  worker.integration.config, frozen=restored)
        a, b = json.loads(actual.result_json), json.loads(expected.result_json)
        a.pop("elapsed_ms"), b.pop("elapsed_ms")
        self.assertEqual(a, b)
        self.assertEqual(identity, actual.identity)
        self.assertEqual("version-original", actual.baseline_version_id)

    def test_baseline_wrong_car_wrong_config_or_modified_asset_refused(self):
        worker, frame, stab, identity, start = fixture()
        snapshot = baseline_and_model(worker, identity)
        data = FrozenDetection.freeze(worker, frame, stab, identity, start)
        binding = snapshot.binding
        binding["configuration_sha256"] = "0" * 64
        for bad in (replace(snapshot, passage_id="next-car"), replace(snapshot, binding_json=canonical(binding)),
                    replace(snapshot, asset_bytes=b"broken"), replace(snapshot, asset_bytes=None)):
            with self.assertRaises(ValueError):
                calculate_frozen(data, identity, bad, "原车", worker.integration.config)

    def test_safe_codec_rejects_noncanonical_duplicate_unknown_and_binary_damage(self):
        data, *_ = self.freeze()
        for bad in (data + b" ", b'{"a":1,"a":2}', b'{"x":NaN}', b'\x80\x04pickle',
                    canonical(dict(json.loads(data), unknown="x"))):
            with self.assertRaises((ValueError, UnicodeError)):
                FrozenDetection.restore(bad)
        record = json.loads(data)
        record["frames"][0]["xyz"] = "*invalid*"
        with self.assertRaises(ValueError):
            FrozenDetection.restore(canonical(record))
        record = json.loads(data)
        record["frames"][0]["ring"] = base64.b64encode(b"xx").decode()
        with self.assertRaises(ValueError):
            FrozenDetection.restore(canonical(record))

    def test_original_gate_and_pair_evidence_cannot_be_replaced(self):
        data, *_ = self.freeze()
        changes = [lambda r: r["gate"]["decision"]["stability"].update(stable=False),
                   lambda r: r["gate"]["decision"]["stability"].update(motion_pending=True),
                   lambda r: r["gate"]["settle"].update(waited_sec=0.),
                   lambda r: r["support"][0].update(complete=False),
                   lambda r: r["support"][0].update(channel="CH3"),
                   lambda r: r["primary"][0].update(indices=""),
                   lambda r: r["frames"][0].update(dropped_packets=1)]
        for change in changes:
            record = json.loads(data)
            change(record)
            with self.assertRaises(ValueError):
                FrozenDetection.restore(canonical(record))

    def test_configuration_catalog_and_input_budget_refused_when_corrupted(self):
        data, *_ = self.freeze()
        record = json.loads(data)
        config = json.loads(base64.b64decode(record["configuration"]))
        config["calib"]["floor"]["extra"] = 1
        record["configuration"] = base64.b64encode(canonical(config)).decode()
        with self.assertRaises(ValueError):
            FrozenDetection.restore(canonical(record))
        with patch("app.frozen_detection.MAX_BINARY_BYTES", 16):
            with self.assertRaises(ValueError):
                FrozenDetection.restore(data)

    def test_manual_commands_and_unbound_passage_cannot_enter_async_automatic_queue(self):
        worker, frame, stab, identity, start = fixture()
        for wrong in (replace(identity, passage_id=None), replace(identity, command_json=b"{}")):
            with self.assertRaises(ValueError):
                FrozenDetection.freeze(worker, frame, stab, wrong, start)

    def test_new_task_requires_all_stability_fields_and_actual_configuration(self):
        data, worker, frame, stab, identity, start = self.freeze()
        record = json.loads(data)
        for name in ("motion_pending", "instant_speed_mps", "jitter_confirmed"):
            del record["gate"]["decision"]["stability"][name]
        with self.assertRaises(ValueError):
            FrozenDetection.restore(canonical(record))
        worker.calib.detection.ground_z = -8.
        with self.assertRaisesRegex(ValueError, "活动配置对象已变化"):
            FrozenDetection.freeze(worker, frame, stab, identity, start)


if __name__ == "__main__":
    unittest.main()
