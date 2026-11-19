"""真实测量原包的离线复算、预算、重哈希篡改和能力边界。"""
from dataclasses import replace
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

import numpy as np

from app.archive_io import ArchiveReadLimits, check_budget
from app.archive_reader import load_archive
from app.measurement_runtime import canonical
from app.sample_paths import Sample
from app.volume_replay import recompute
from tests.test_archive_inputs import mutate, make_inputs
from tests.test_sample_paths import publish
from tests import test_volume_runtime as runtime_fixture


class TestVolumeReplay(unittest.TestCase):
    setUp = runtime_fixture.TestVolumeRuntime.setUp
    close = runtime_fixture.TestVolumeRuntime.close
    bridge = runtime_fixture.TestVolumeRuntime.bridge
    feed = runtime_fixture.TestVolumeRuntime.feed
    worker = runtime_fixture.TestVolumeRuntime.worker
    ground = staticmethod(runtime_fixture.TestVolumeRuntime.ground)
    surfaces = staticmethod(runtime_fixture.TestVolumeRuntime.surfaces)
    arm = runtime_fixture.TestVolumeRuntime.arm
    finish = runtime_fixture.TestVolumeRuntime.finish

    def sample(self, bundle):
        serial = getattr(self, "serial", 0) + 1
        self.serial = serial
        directory = Path(self.tmp.name) / ("replay-" + str(serial))
        publish(directory, bundle)
        meta = bundle.metadata
        return Sample(str(directory / "observation.xyz"), "observation", meta["channel"], meta["measurement_id"],
                      meta["acquired_at"], hashlib.sha256(bundle.manifest).hexdigest())

    def bundle(self, **kwargs):
        worker, bridge, base, frame, model = self.arm(original_asset=True, **kwargs)
        row = self.finish(worker, frame)
        return bridge.outbox.archives.get(row["measurement_id"]), worker

    def test_actual_worker_archive_and_isolated_process_recompute(self):
        bundle, worker = self.bundle()
        archive = load_archive(self.sample(bundle))
        with patch("app.settings.load_calib", side_effect=AssertionError("禁止读取当前配置")), \
                patch("app.load_radar_contract.BaselineCache.load", side_effect=AssertionError("禁止读取当前缓存"), create=True), \
                patch("socket.socket", side_effect=AssertionError("禁止联网")):
            report = recompute(archive)
        self.assertTrue(report["matched"], report)
        self.assertAlmostEqual(9.6, report["recomputed"]["volume_m3"], places=5)
        self.assertTrue(archive.capabilities["original_volume_replay"])
        self.assertFalse(archive.capabilities["standstill_timeline"])
        sample = self.sample(bundle)
        directory = Path(sample.path).parent
        env = dict(os.environ, PYTHONIOENCODING="utf-8")
        run = subprocess.run([sys.executable, str(Path(__file__).resolve().parents[1] / "cli/volume_replay.py"),
                              "--archive", str(directory)], cwd=self.tmp.name, env=env,
                             capture_output=True, encoding="utf-8", timeout=30)
        self.assertEqual(0, run.returncode, "标准错误:\n" + run.stderr + "\n标准输出:\n" + run.stdout)
        self.assertTrue(json.loads(run.stdout)["matched"])

    def test_enter_frames_are_archived_and_reregistered_on_replay(self):
        """v3：实际交给体积的进场帧原字节进归档，复算按原帧重新配准并逐项核对记录；换一个字节就拒绝。"""
        from app.cloud_observation import FrameSnapshot
        worker, bridge, _, frame, _ = self.arm(original_asset=True)
        worker.volume_catalog = None  # 仅非声明路径会使用进场帧。
        points = np.asarray(frame.xyz, np.float32)
        worker._enter_volume = tuple(
            FrameSnapshot((points + [0., -.3 * (3 - k), 0.]).astype("<f4").tobytes(), None,
                          k + 1, self.clock.wall - .5 + k * .1, frame.gen, 0)
            for k in range(3))
        row = self.finish(worker, frame)
        bundle = bridge.outbox.archives.get(row["measurement_id"])
        files = dict(bundle.files)
        self.assertIn("enter-frames.f32le", list(files))
        self.assertEqual("measurement-runtime-v3", bundle.metadata["measurement_runtime"]["schema"])
        call = json.loads(files["volume-call.json"])
        self.assertEqual("volume-call-v3", call["schema"])
        self.assertIs(True, call["params"]["fill_bound"])
        self.assertEqual(3, len(call["enter"]["frames"]))
        self.assertIsNone(call["exit"])
        report = recompute(load_archive(self.sample(bundle)))
        self.assertTrue(report["matched"], report)
        tampered = bytearray(files["enter-frames.f32le"])
        tampered[40] ^= 1
        with self.assertRaises(ValueError):
            load_archive(self.sample(mutate(bundle, name="enter-frames.f32le", data=bytes(tampered))))
        changed = json.loads(canonical(call))
        changed["enter"]["frames"][0]["ts"] = call["enter"]["frames"][-1]["ts"] + 1.
        with self.assertRaises(ValueError):
            load_archive(self.sample(mutate(bundle, name="volume-call.json", data=canonical(changed))))

    def test_exit_frames_are_archived_and_reregistered_on_replay(self):
        """离场帧（采集序近端在前）进归档；复算 reverse 后重新配准。"""
        from app.cloud_observation import FrameSnapshot
        worker, bridge, _, frame, _ = self.arm(original_asset=True)
        worker.volume_catalog = None
        points = np.asarray(frame.xyz, np.float32)
        # 离场：时间必须严格晚于停稳观测末帧（restore 会核）。夹具里 frame.ts 常为 0，
        # 以 wall 为准并留余量，避免 finish 推进后末帧仍晚于 exit。
        exit_t0 = max(float(frame.ts), float(self.clock.wall)) + 1000.
        worker._exit_volume = tuple(
            FrameSnapshot((points + [0., .3 * k, 0.]).astype("<f4").tobytes(), None,
                          100 + k, exit_t0 + k * .1, frame.gen, 0)
            for k in range(3))
        row = self.finish(worker, frame)
        bundle = bridge.outbox.archives.get(row["measurement_id"])
        files = dict(bundle.files)
        self.assertIn("exit-frames.f32le", list(files))
        call = json.loads(files["volume-call.json"])
        self.assertEqual("volume-call-v3", call["schema"])
        self.assertEqual(3, len(call["exit"]["frames"]))
        report = recompute(load_archive(self.sample(bundle)))
        self.assertTrue(report["matched"], report)

    def test_declared_model_replay_truthfully_omits_unused_entry(self):
        from app.cloud_observation import FrameSnapshot
        worker, bridge, _, frame, _ = self.arm(original_asset=True)
        worker._enter_volume = (FrameSnapshot(np.asarray(frame.xyz, "<f4").tobytes(), None,
                                              1, self.clock.wall - .5, frame.gen, 900),)
        row = self.finish(worker, frame)
        bundle = bridge.outbox.archives.get(row["measurement_id"])
        files = dict(bundle.files)
        call = json.loads(files["volume-call.json"])
        self.assertIsNone(call["enter"])
        self.assertIsNone(call["exit"])
        self.assertFalse(call["params"]["fill_bound"])
        self.assertNotIn("enter-frames.f32le", list(files))
        self.assertNotIn("exit-frames.f32le", list(files))
        self.assertIn("进/离场帧和遮挡上限补洞未参与计算", row["volume"]["text"])
        report = recompute(load_archive(self.sample(bundle)))
        self.assertTrue(report["matched"], report["differences"])

    def test_merged_overlap_nonzero_dy_and_partial_range(self):
        bundle, _ = self.bundle(merged=True, settle=.25, dy=1.25, partial=True)
        report = recompute(load_archive(self.sample(bundle)))
        self.assertTrue(report["matched"], report["differences"])
        self.assertFalse(report["recomputed"]["low_confidence"])
        self.assertGreater(report["recomputed"]["unknown_area_m2"], 4.)
        self.assertLessEqual(report["recomputed"]["bound_lo"], 9.6)
        self.assertGreaterEqual(report["recomputed"]["bound_hi"], 9.6)

    def test_missing_original_asset_is_not_no_baseline(self):
        worker, bridge, _, frame, _ = self.arm()
        row = self.finish(worker, frame)
        archive = load_archive(self.sample(bridge.outbox.archives.get(row["measurement_id"])))
        self.assertFalse(archive.capabilities["original_volume_replay"])
        self.assertEqual("unavailable", recompute(archive)["status"])

    def test_no_model_retains_null_range(self):
        bundle, _ = self.bundle(change=lambda m: m["binding"].update(baseline_asset_sha256="c" * 64))
        report = recompute(load_archive(self.sample(bundle)))
        self.assertTrue(report["matched"], report)
        self.assertIsNone(report["recomputed"]["bound_lo"])

    def test_missing_inputs_and_unknown_call_format_remain_explicit(self):
        old, _ = make_inputs()
        archive = load_archive(self.sample(old))
        self.assertFalse(archive.capabilities["original_volume_replay"])
        self.assertEqual("unavailable", recompute(archive)["status"])
        bundle, _ = self.bundle()
        call = json.loads(dict(bundle.files)["volume-call.json"])
        call["algorithm"]["contract"] = "unknown-call-format"
        archive = load_archive(self.sample(mutate(bundle, name="volume-call.json", data=canonical(call))))
        self.assertEqual("unsupported_algorithm", recompute(archive)["status"])

    def test_old_source_fingerprint_does_not_block_replay_of_original_inputs(self):
        bundle, _ = self.bundle()
        call = json.loads(dict(bundle.files)["volume-call.json"])
        self.assertNotIn("fingerprint", call["algorithm"])
        call["algorithm"]["fingerprint"] = "f" * 64
        archive = load_archive(self.sample(mutate(bundle, name="volume-call.json", data=canonical(call))))
        report = recompute(archive)
        self.assertTrue(report["matched"], report)
        self.assertEqual(call["original_result"]["volume_m3"], report["recomputed"]["volume_m3"])

    def test_rehashed_parameter_and_asset_tampering_is_rejected(self):
        bundle, _ = self.bundle(merged=True, settle=.25)
        original = json.loads(dict(bundle.files)["volume-call.json"])
        changes = [lambda c: c["params"].update(dy=2.),
                   lambda c: c["baseline"].update(version_id="another"),
                   lambda c: c["loaded"].update(surface_sha256="f" * 64),
                   lambda c: c["loaded"]["surface_members"].reverse(),
                   lambda c: c["identity"].update(passage_id="another"),
                   lambda c: c["params"].update(bed_evidence={}),
                   lambda c: c["original_result"].pop("bound_lo"),
                   lambda c: c["params"].update(cells=[True]),
                   lambda c: c["params"].update(cells=[.1] * 17)]
        for change in changes:
            call = json.loads(canonical(original))
            change(call)
            with self.subTest(change=change), self.assertRaises(ValueError):
                load_archive(self.sample(mutate(bundle, name="volume-call.json", data=canonical(call))))
        config = json.loads(dict(bundle.files)["effective-configuration.json"])
        config["channel"]["id"] = "CH2"
        with self.assertRaises(ValueError):
            load_archive(self.sample(mutate(bundle, name="effective-configuration.json", data=canonical(config))))
        with self.assertRaises(ValueError):
            load_archive(self.sample(mutate(bundle, name="baseline.npz", data=b"invalid")))

    def test_stripped_extension_and_budget_rejected_before_member_reads(self):
        bundle, _ = self.bundle()
        bad = mutate(bundle, change=lambda meta: meta.pop("measurement_runtime"))
        with self.assertRaises(ValueError):
            load_archive(self.sample(bad))
        sample = self.sample(bundle)
        from app import archive_reader
        real = archive_reader._read_regular
        with patch.object(archive_reader, "_read_regular", wraps=real) as read:
            with self.assertRaises(ValueError):
                load_archive(sample, limits=replace(ArchiveReadLimits(), working_bytes=400 * 1024 * 1024))
        self.assertEqual(1, read.call_count)

    def test_gate_summary_raw_precision_and_actual_frame(self):
        bundle, worker = self.bundle(settle=.25)
        gate = json.loads(dict(bundle.files)["gate-summary.json"])
        self.assertTrue(gate["summary_only"])
        self.assertFalse(gate["event_stream_complete"])
        self.assertEqual(bundle.metadata["provenance"]["segments"][-1]["frame"], gate["decision"]["frame"])
        self.assertGreaterEqual(gate["settle"]["waited_sec"], .25)
        from app.detect import StabilityResult
        stab = StabilityResult(stationary_sec=1.9999996)
        decision = worker._gate_decision(worker.last_frame, stab, True)
        self.assertEqual(1.9999996, decision["stability"]["stationary_sec"])
        self.assertEqual(2., stab.to_dict()["stationary_sec"])

    def test_output_cannot_overwrite_input_or_existing_report(self):
        bundle, _ = self.bundle()
        directory = Path(self.sample(bundle).path).parent
        from cli.volume_replay import main
        self.assertEqual(2, main(["--archive", str(directory), "--json", str(directory / "report.json")]))
        self.assertFalse((directory / "report.json").exists())

    def test_actual_nonzero_heading_rotation_reconstructs_original_cloud(self):
        worker, bridge, _, frame, _ = self.arm(original_asset=True)
        angle = .12
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0.],
                             [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
        from app.pointcloud import Frame
        rotated = Frame((frame.xyz @ rotation.T).astype(np.float32))
        row = self.finish(worker, rotated)
        bundle = bridge.outbox.archives.get(row["measurement_id"])
        call = json.loads(dict(bundle.files)["volume-call.json"])
        self.assertFalse(np.allclose(call["loaded"]["source_rotation"], np.eye(3)))
        report = recompute(load_archive(self.sample(bundle)))
        self.assertTrue(report["matched"], report["differences"])

    def test_no_baseline_recomputes_unknown_not_zero(self):
        worker, bridge, _, frame, _ = self.arm(original_asset=True)
        bridge._loaded.clear()
        row = self.finish(worker, frame)
        bundle = bridge.outbox.archives.get(row["measurement_id"])
        call = json.loads(dict(bundle.files)["volume-call.json"])
        self.assertEqual("absent", call["baseline"]["state"])
        self.assertNotIn("baseline.npz", dict(bundle.files))
        report = recompute(load_archive(self.sample(bundle)))
        self.assertTrue(report["matched"], report)
        self.assertIsNone(report["recomputed"]["volume_m3"])

    def test_manual_original_three_frames_and_wrong_summary_rejected(self):
        bundle, worker = self.bundle(live=True)
        bridge = self.bridges[-1]
        _, frame = self.surfaces()
        self.feed(worker, frame, 3)
        passage = bridge.current_passage("CH1")
        bridge.queue_command(dict(requestId="replay-manual", channelId="CH1", passageId=passage,
                                  kind="remeasure", confirmedEmpty=False))
        self.feed(worker, frame, 1)
        from tests.test_result_finalizer import await_result
        row = await_result(worker, count=2)[-1]
        bundle = bridge.outbox.archives.get(row["measurement_id"])
        gate = json.loads(dict(bundle.files)["gate-summary.json"])
        self.assertEqual(3, len(gate["manual_three_frames"]))
        self.assertEqual(3, len({d["frame"]["seq"] for d in gate["manual_three_frames"]}))
        self.assertTrue(recompute(load_archive(self.sample(bundle)))["matched"])
        gate["manual_three_frames"][0] = gate["manual_three_frames"][-1]
        with self.assertRaises(ValueError):
            load_archive(self.sample(mutate(bundle, name="gate-summary.json", data=canonical(gate))))
        gate = json.loads(dict(bundle.files)["gate-summary.json"])
        gate["settle"]["baseline_quality"] = dict(ok=True, n=3, r_min=1., r_all=[1., 1., 1.], why="合成")
        with self.assertRaises(ValueError):
            load_archive(self.sample(mutate(bundle, name="gate-summary.json", data=canonical(gate))))

    def test_candidate_summary_requires_actual_three_correlations(self):
        from tests.test_result_finalizer import TestResultFinalizer, await_result
        harness = TestResultFinalizer()
        harness.setUp()
        try:
            bridge = harness.bridge()
            worker = harness.worker(bridge, live=True, channel="CH4")
            harness.feed(worker, harness.ground(), 8)
            frame = harness.empty_truck()
            harness.feed(worker, frame, 35)
            await_result(worker)
            bridge.queue_command(dict(requestId="quality-summary", channelId="CH4",
                passageId=bridge.current_passage("CH4"), kind="baseline-capture", confirmedEmpty=True))
            harness.feed(worker, frame, 1)
            row = await_result(worker, count=2)[-1]
            bundle = bridge.outbox.archives.get(row["measurement_id"])
            load_archive(self.sample(bundle))
            original = json.loads(dict(bundle.files)["gate-summary.json"])
            for quality in (dict(ok=True, n=3), dict(ok=True, n=3, r_min=-1., r_all=[-1.] * 3, why="异常")):
                gate = json.loads(canonical(original))
                gate["settle"]["baseline_quality"] = quality
                with self.assertRaises(ValueError):
                    load_archive(self.sample(mutate(bundle, name="gate-summary.json", data=canonical(gate))))
        finally:
            self.assertTrue(harness.doCleanups())

    def test_gate_rehashed_rounding_and_false_full_timeline_rejected(self):
        bundle, _ = self.bundle()
        original = json.loads(dict(bundle.files)["gate-summary.json"])
        for change in (lambda g: g.update(event_stream_complete=True),
                       lambda g: g["decision"]["stability"].update(stationary_sec=1.9999996),
                       lambda g: g["decision"]["stability"].update(n_points=-1),
                       lambda g: g["decision"]["stability"].update(n_vehicle_points={"invalid": True}),
                       lambda g: g["decision"]["stability"].update(n_points=True),
                       lambda g: g["decision"]["thresholds"].update(stationary_speed_window_sec=-1),
                       lambda g: g["decision"]["thresholds"].update(stationary_max_gap_sec=3),
                       lambda g: g["decision"]["thresholds"].update(stationary_jitter_m=.03),
                       lambda g: g["decision"]["thresholds"].update(stationary_jitter_m=.002),
                       lambda g: g["decision"]["thresholds"].pop("stationary_jitter_m"),
                       lambda g: g["decision"]["stability"].update(motion_pending=True),
                       lambda g: g["decision"]["stability"].update(instant_speed_mps=.03, jitter_confirmed=False),
                       lambda g: g["settle"].update(waited_sec=-1.),
                       lambda g: g["decision"]["frame_policy"].update(packet_count_known=True)):
            gate = json.loads(canonical(original))
            change(gate)
            with self.assertRaises(ValueError):
                load_archive(self.sample(mutate(bundle, name="gate-summary.json", data=canonical(gate))))

    def test_confirmation_fields_cannot_be_stripped_from_current_archive(self):
        bundle, _ = self.bundle()
        original = json.loads(dict(bundle.files)["gate-summary.json"])
        for remove_threshold in (False, True):
            with self.subTest(remove_threshold=remove_threshold):
                gate = json.loads(canonical(original))
                for key in ("motion_pending", "instant_speed_mps", "jitter_confirmed"):
                    gate["decision"]["stability"].pop(key)
                if remove_threshold:
                    gate["decision"]["thresholds"].pop("stationary_jitter_m")
                with self.assertRaises(ValueError):
                    load_archive(self.sample(mutate(bundle, name="gate-summary.json", data=canonical(gate))))

    def test_legacy_decision_requires_legacy_configuration_without_confirmation(self):
        from app.measurement_runtime import _validate_decision
        bundle, _ = self.bundle()
        decision = json.loads(dict(bundle.files)["gate-summary.json"])["decision"]
        detection = json.loads(dict(bundle.files)["effective-configuration.json"])["calib"]["detection"]
        _validate_decision(decision, detection)
        for key in ("motion_pending", "instant_speed_mps", "jitter_confirmed"):
            decision["stability"].pop(key)
        decision["thresholds"].pop("stationary_jitter_m")
        with self.assertRaises(ValueError):
            _validate_decision(decision, detection)
        detection.pop("stationary_jitter_m")
        _validate_decision(decision, detection)

    def test_shape_and_yaw_speeds_are_checked_against_their_own_thresholds(self):
        """★★ 归档校验必须与 StabilityDetector 的判据逐项一致（2026-09-23）。

        原来这里把 shape/yaw 速度也拿去比 stationary_speed_mps(0.02)，而检测器对它们
        按各自门槛（现场 0.10）放行 -> 许可给了、归档却拒收 -> 终局器按原 ID 无限重放
        -> 该道此后停测。9-21 实测静止帧 69.9% 落在 (0.02, 0.10]；真实整帧回放里
        CH4 第一辆车（yaw_speed 0.0267）就这样把整条道卡死了。见 docs/60 第 7.5 节。
        """
        from app.measurement_runtime import _validate_decision
        bundle, _ = self.bundle()
        original = json.loads(dict(bundle.files)["gate-summary.json"])["decision"]
        detection = json.loads(dict(bundle.files)["effective-configuration.json"])["calib"]["detection"]

        def decision(**stability):
            d = json.loads(json.dumps(original))
            d["thresholds"].update(stationary_speed_mps=0.02, stationary_shape_speed_mps=0.10,
                                   stationary_yaw_speed_mps=0.10)
            d["stability"].update(dict(speed_mps=0.0, shape_speed_mps=0.0, yaw_speed_mps=0.0), **stability)
            return d

        # 检测器会放行的：形变/旋转在 (0.02, 0.10]，含恰好等于门槛
        for shape, yaw in ((0.05, 0.0), (0.0, 0.0267), (0.0999, 0.0999), (0.10, 0.10)):
            with self.subTest(accept=(shape, yaw)):
                _validate_decision(decision(shape_speed_mps=shape, yaw_speed_mps=yaw), detection)
        # 检测器会拦的：超过各自门槛；回归速度仍按平移门槛 0.02
        for key, value in (("shape_speed_mps", 0.1001), ("yaw_speed_mps", 0.1001), ("speed_mps", 0.0201)):
            with self.subTest(reject=key):
                with self.assertRaisesRegex(ValueError, "停稳原值未达到原许可门槛"):
                    _validate_decision(decision(**{key: value}), detection)
        # 门槛本身非正也不许过（检测器实际生效的值恒为正）
        for key in ("stationary_shape_speed_mps", "stationary_yaw_speed_mps"):
            with self.subTest(nonpositive=key):
                bad = decision()
                bad["thresholds"][key] = 0.0
                with self.assertRaises(ValueError):
                    _validate_decision(bad, detection)

    def test_original_asset_survives_no_provider_version_switch_during_calculation(self):
        worker, bridge, _, frame, _ = self.arm(original_asset=True)
        worker.volume_catalog = None
        original_asset = bridge._loaded["CH1"][6]
        from app.application import channel_worker as service
        actual = service.compute_volume_auto
        def switch_version(*args, **kwargs):
            bridge._loaded.clear()
            return actual(*args, **kwargs)
        with patch.object(service, "compute_volume_auto", side_effect=switch_version):
            row = self.finish(worker, frame)
        bundle = bridge.outbox.archives.get(row["measurement_id"])
        self.assertEqual(original_asset, dict(bundle.files)["baseline.npz"])
        self.assertTrue(recompute(load_archive(self.sample(bundle)))["matched"])

    def test_semantic_baseline_digest_rejects_valid_repacked_asset(self):
        bundle, _ = self.bundle()
        from app.load_radar_contract import _read_baseline
        base = _read_baseline(dict(bundle.files)["baseline.npz"])
        stream = io.BytesIO()
        np.savez_compressed(stream, floor_z=base.floor_z + .1, box=np.asarray(base.box), cloud=base.cloud,
                            bed_plane=np.asarray(base.bed_plane), created="2024-01-01T00:00:00Z")
        asset = stream.getvalue()
        _read_baseline(asset)  # 替换资产本身合法，不靠ZIP损坏拒绝。
        bad = mutate(bundle, name="baseline.npz", data=asset)
        call = json.loads(dict(bundle.files)["volume-call.json"])
        digest = hashlib.sha256(asset).hexdigest()
        call["baseline"]["asset_sha256"] = digest
        call["runtime"]["binding"]["baseline_asset_sha256"] = digest
        call["runtime"]["model"]["binding"]["baseline_asset_sha256"] = digest
        call["runtime"]["model_sha256"] = hashlib.sha256(canonical(call["runtime"]["model"])).hexdigest()
        result = json.loads(dict(bundle.files)["detection.xyz.meta.json"])
        result["volume_runtime"] = call["runtime"]
        bad = mutate(mutate(bad, name="volume-call.json", data=canonical(call)),
                     name="detection.xyz.meta.json", data=canonical(result))
        with self.assertRaisesRegex(ValueError, "原资产解码内容"):
            load_archive(self.sample(bad))

    def test_legacy_union_grid_budget_checked_before_algorithm_allocation(self):
        bundle, worker = self.bundle()
        archive = load_archive(self.sample(bundle))
        from app import volume_replay
        from app.measurement_runtime import restore_inputs
        call, base, loaded, extras = restore_inputs(archive, ArchiveReadLimits())
        call["params"].update(volume_context=None, volume_scope=None)
        loaded.cloud = np.array([[-20., -490., -3.4], [20., 490., -3.4]])
        with patch.object(volume_replay, "restore_inputs", return_value=(call, base, loaded, extras)), \
                patch("app.detect.compute_volume_auto", side_effect=AssertionError("预算必须先拒绝")):
            with self.assertRaisesRegex(ValueError, "预算"):
                recompute(archive)
