"""显式参数经真实Worker/计算/收尾进入临时outbox和归档，不注入预期体积。"""
import copy
import errno
import hashlib
import io
from dataclasses import replace
import json
from pathlib import Path
import threading
import time
from unittest.mock import patch
import unittest

import numpy as np

from app.bed_reference import baseline_digest
from app.detect import estimate_dy, estimate_registration, LoadedResult
from app.load_radar_bridge import ResultReservation
from app.pointcloud import Frame
from app.service import ChannelWorker, EventBus, LidarService
from tests import _helpers as helpers
from tests import test_result_finalizer as fixture
from tests.test_volume_provider import catalog_for, model_for
from tests.test_volume_validity import scene


class TestVolumeRuntime(unittest.TestCase):
    setUp = fixture.TestResultFinalizer.setUp
    close = fixture.TestResultFinalizer.close
    bridge = fixture.TestResultFinalizer.bridge
    feed = fixture.TestResultFinalizer.feed
    ground = staticmethod(fixture.TestResultFinalizer.ground)

    def worker(self, bridge, save=True, merged=False, settle=.0, live=False):
        settings, ch, _, _, _ = helpers.ctx()
        ch = copy.deepcopy(ch)
        # ★ 通道号必须钉死 CH1：本夹具的 bridge 只登记了 CH1，下面的断言也全写死 CH1。
        #   不钉死的话它会跟着 config/channels.json 的"第一条启用通道"漂 ——
        #   CH1 一停用就变成 CH2，ChannelWorker 构造时 configure_channel 直接抛
        #   "到车上下文缺少实际配置摘要"（该异常同时覆盖"通道不认识"，看报错信息看不出来）。
        #   夹具不该依赖现场部署形态。参见 tests/test_result_finalizer.py 的同名方法。
        ch.id = "CH1"
        ch.mode = "live" if live else "synthetic"
        ch.abs_output_dir = str(Path(self.tmp.name) / ("CH1-" + str(len(self.workers))))
        ch.abs_baseline_dir = str(Path(self.tmp.name) / "unused-baselines")
        from app.radar_source import build_source
        def synthetic_source(*args, **kwargs):
            return build_source(*args, **dict(kwargs, mode="synthetic"))
        with patch("app.application.channel_worker.build_source", side_effect=synthetic_source):
            worker = ChannelWorker(ch, EventBus(), settings.plate, save_override=save, plate_mode="null",
                                   integration=bridge, raw_frames=0, volume_catalog=catalog_for([]))
        worker.settle_sec, worker.settle_accumulate = settle, merged
        self.workers.append(worker)
        return worker

    @staticmethod
    def surfaces(partial=False, dy=0., height=.4):
        base, loaded = scene(height)
        offset = np.array([-1.2, -5., -3.8])
        base.cloud = base.cloud + offset
        base.floor_z = -3.8
        base.bed_plane = (0., 0., 1., 3.8)
        base.box = (-1.2, 1.2, -5., 5., -3.8, -.8)
        cloud = loaded.cloud + offset + [0, dy, 0]
        if partial:
            cloud = cloud[cloud[:, 1] >= -3 + dy]
        return base, Frame(np.repeat(cloud, 20, axis=0).astype(np.float32))

    def arm(self, *, save=True, merged=False, settle=0., partial=False, dy=0., change=None, live=False, height=.4,
            original_asset=False):
        bridge = self.bridge()
        worker = self.worker(bridge, save, merged, settle, live)
        base, frame = self.surfaces(partial, dy, height)
        self.feed(worker, self.ground(), 8)
        self.feed(worker, frame, 1)
        passage = bridge.current_passage("CH1")
        self.assertIsNotNone(passage)
        version = dict(versionId="version-1", assetSha256="b" * 64, activatedAt="2023-12-31T00:00:00Z")
        asset = None
        if original_asset:
            from app.load_radar_contract import _read_baseline
            stream = io.BytesIO()
            np.savez_compressed(stream, floor_z=base.floor_z, box=np.asarray(base.box),
                                cloud=base.cloud.astype(np.float32), bed_plane=np.asarray(base.bed_plane),
                                created="2024-01-01T00:00:00Z")
            asset = stream.getvalue()
            version["assetSha256"] = hashlib.sha256(asset).hexdigest()
            base = _read_baseline(asset)
        bridge._loaded["CH1"] = (passage, version, base, {"licensePlate": "合成测试车"},
                                  self.clock.mono, worker._integration_hash, asset)
        model = model_for(base)
        model["binding"].update(node_id="test", configuration_sha256=worker._integration_hash)
        model["binding"]["baseline_asset_sha256"] = version["assetSha256"]
        model.update(domain_xy=[-1.2, 1.2, -5., 5.], support_origin_xy=[-1.2, -5.],
                     valid_from=self.clock.wall - 10, valid_until=self.clock.wall + 60)
        model["registration"]["dy_m"] = dy
        if change:
            change(model)
        worker.volume_catalog = catalog_for([model])
        return worker, bridge, base, frame, model

    def finish(self, worker, frame, exported=True):
        for _ in range(80):
            self.feed(worker, frame, 1)
            if worker._detected_this_session:
                break
        self.assertEqual(1, worker._detected_this_session)
        return fixture.await_result(worker, exported=exported)[0]

    def test_actual_standstill_calculation_outbox_and_archive_evidence(self):
        worker, bridge, base, frame, model = self.arm()
        row = self.finish(worker, frame)
        self.assertEqual("supplied", row["volume_runtime"]["status"], row["volume_runtime"])
        self.assertEqual("conditional", row["volume"]["range_status"])
        self.assertAlmostEqual(9.6, row["volume"]["volume_m3"], places=5)
        self.assertEqual(0., row["volume"]["bed_shift_applied_m"])
        payload = bridge.outbox.get(row["measurement_id"]).payload
        self.assertEqual(23, len(payload))
        self.assertEqual("version-1", payload["baselineVersionId"])
        bundle = bridge.outbox.archives.get(row["measurement_id"])
        stored = json.loads(dict(bundle.files)["detection.xyz.meta.json"])
        self.assertEqual(row["volume_runtime"], stored["volume_runtime"])
        self.assertEqual(model, stored["volume_runtime"]["model"])
        self.assertEqual(baseline_digest(base), stored["volume_runtime"]["binding"]["baseline_sha256"])

    def test_disabled_saving_still_has_real_scope_without_archives(self):
        worker, bridge, _, frame, _ = self.arm(save=False)
        self.assertTrue(worker.source.keep_raw)
        self.assertEqual(0, worker.raw_frames)
        row = self.finish(worker, frame, exported=False)
        self.assertEqual("supplied", row["volume_runtime"]["status"])
        self.assertAlmostEqual(9.6, row["volume"]["volume_m3"], places=5)
        self.assertIsNone(bridge.outbox.archives.get(row["measurement_id"]))

    def test_partial_coverage_retains_unknown_area_and_low_quality(self):
        worker, _, _, frame, _ = self.arm(partial=True)
        row = self.finish(worker, frame)
        vol = row["volume"]
        self.assertEqual("conditional", vol["range_status"])
        self.assertGreater(vol["unknown_area_m2"], 4.)
        self.assertFalse(vol["low_confidence"])
        self.assertLessEqual(vol["bound_lo"], 9.6)
        self.assertGreaterEqual(vol["bound_hi"], 9.6)

    def test_merged_and_extra_surface_mappings_preserve_real_frame_ids(self):
        worker, _, _, frame, _ = self.arm(merged=True, settle=.25)
        row = self.finish(worker, frame)
        runtime = row["volume_runtime"]
        self.assertEqual("supplied", runtime["status"], runtime)
        mapping = runtime["scope"]["surface_members"]
        self.assertGreater(len(mapping[0]), 1)
        self.assertEqual(list(range(len(runtime["scope"]["frame_ids"]))), mapping[0])
        self.assertEqual(3, len(mapping))
        self.assertAlmostEqual(9.6, row["volume"]["volume_m3"], places=5)

    def test_fixed_nonzero_registration_is_used_once(self):
        worker, _, _, frame, _ = self.arm(dy=1.25, settle=.25)
        row = self.finish(worker, frame)
        runtime = row["volume_runtime"]
        self.assertEqual("supplied", runtime["status"], runtime)
        self.assertEqual([-1.2, 1.2, -3.75, 6.25], runtime["context"]["domain_xy"])
        self.assertEqual(1.25, runtime["registration"]["dy_m"])
        self.assertAlmostEqual(9.6, row["volume"]["volume_m3"], places=5)

    def test_unresolved_estimation_cannot_authorize_zero_alignment(self):
        worker, _, _, frame, _ = self.arm(change=lambda m: m.update(registration=dict(mode="estimated")))
        row = self.finish(worker, frame)
        self.assertEqual("rejected", row["volume_runtime"]["status"])
        self.assertEqual("unresolved", row["volume_runtime"]["registration"]["status"])
        self.assertIsNone(row["volume"]["bound_lo"])
        self.assertFalse(row["volume"]["low_confidence"])

    def test_wrong_asset_binding_and_expiry_never_supply_conditions(self):
        for change, status in ((lambda m: m["binding"].update(baseline_asset_sha256="c" * 64), "binding_not_found"),
                (lambda m: m.update(valid_until=self.clock.wall - 1), "expired")):
            worker, _, _, frame, _ = self.arm(change=change)
            row = self.finish(worker, frame)
            self.assertEqual(status, row["volume_runtime"]["status"])
            self.assertIsNone(row["volume"]["bound_hi"])
            self.assertEqual("parameters_required", row["volume_preparation"])
            worker.st.online = True
            self.assertEqual("parameters_required", worker.snapshot()["volume_preparation"])
            worker.force_new_session()
            self.assertIsNone(worker.snapshot()["volume_preparation"])
            # 下一轮有独立临时队列，不与尚存测试bridge共用存储。
            worker.stop()
            worker.close_results(10.)
            self.bridges[-1].close()
            self.bridges.pop()

    def test_snapshot_never_switches_to_next_passage_and_owns_original_bytes(self):
        worker, bridge, base, _, _ = self.arm()
        reservation = bridge.reserve_result("CH1", "token")
        snapshot = bridge.baseline_snapshot(reservation, self.clock.wall, worker._integration_hash)
        self.assertIsNotNone(snapshot)
        original = snapshot.baseline().cloud.copy()
        base.cloud[:] = 99.
        np.testing.assert_array_equal(original, snapshot.baseline().cloud)
        with self.assertRaises(ValueError):
            snapshot.baseline().cloud.setflags(write=True)
        self.assertIsNone(bridge.baseline_snapshot(replace(reservation, passage_id="next"),
                                                self.clock.wall, worker._integration_hash))
        self.assertIsNone(bridge.baseline_snapshot(reservation, self.clock.wall, "f" * 64))
        self.clock.advance(11)
        self.assertIsNone(bridge.baseline_snapshot(reservation, self.clock.wall, worker._integration_hash))
        # 测试未生成测量，手动清除仅本测试的预留，避免cleanup误称有待交付结果。
        bridge._reserved.clear()

    def test_startup_invalid_model_has_no_capture_or_bridge_side_effect(self):
        path = Path(self.tmp.name) / "invalid-provider.json"
        path.write_text('{"schema":"volume-provider-v1","models":[],"unknown":1}', encoding="utf-8")
        with patch("app.application.channel_worker.build_source", side_effect=AssertionError("不得启动采集")), \
                patch("app.load_radar_bridge.LoadRadarBridge", side_effect=AssertionError("不得创建数据库")):
            with self.assertRaises(ValueError):
                LidarService(volume_provider_config=str(path), webapi_config="unused.json")
        self.assertEqual("models.json", __import__("cli.run", fromlist=["build_parser"]).build_parser().parse_args(
            ["--volume-provider-config", "models.json"]).volume_provider_config)

    def test_manual_three_frame_remeasure_has_actual_scope_and_idempotent_result(self):
        worker, bridge, _, frame, _ = self.arm(live=True)
        self.finish(worker, frame)
        self.feed(worker, frame, 3)
        passage = bridge.current_passage("CH1")
        command = dict(requestId="volume-remeasure", channelId="CH1", passageId=passage,
                       kind="remeasure", confirmedEmpty=False)
        bridge.queue_command(command)
        self.feed(worker, frame, 1)
        bridge._loaded.clear()
        row = fixture.await_result(worker, count=2)[-1]
        self.assertEqual("supplied", row["volume_runtime"]["status"], row["volume_runtime"])
        self.assertEqual([[2], [0], [1]], row["volume_runtime"]["scope"]["surface_members"])
        self.assertAlmostEqual(9.6, row["volume"]["volume_m3"], places=5)
        self.assertEqual(row["measurement_id"], bridge.queue_command(command)["measurementId"])

    def test_retry_after_reset_and_model_change_reuses_original_frozen_evidence(self):
        worker, bridge, base, frame, model = self.arm(original_asset=True)
        captured = []
        def fail(payload, **kwargs):
            captured.append((json.dumps(payload, sort_keys=True), kwargs["archive"]))
            raise OSError(errno.ENOSPC, "合成测试磁盘不足")
        with patch.object(bridge.outbox, "save", side_effect=fail):
            self.feed(worker, frame, 35)
            deadline = time.perf_counter() + 8
            while not captured and time.perf_counter() < deadline:
                threading.Event().wait(.01)
            self.assertTrue(captured)
            original = json.loads(dict(captured[0][1].files)["detection.xyz.meta.json"])["volume_runtime"]
            worker.force_new_session()
            worker.volume_catalog = catalog_for([])
            base.cloud[:] = 99.
            self.assertFalse(worker.close_results(0.))
        row = fixture.await_result(worker)[0]
        self.assertEqual(original, row["volume_runtime"])
        self.assertEqual(model, row["volume_runtime"]["model"])
        self.assertIn("baseline.npz", dict(captured[0][1].files))
        self.assertEqual(json.loads(captured[0][0])["measurementId"], row["measurement_id"])
        self.assertTrue(all(value == captured[0] for value in captured))

    def test_thin_coal_is_not_turned_into_an_independent_floor_reference(self):
        worker, _, _, frame, _ = self.arm(height=.04)
        row = self.finish(worker, frame)
        self.assertEqual("supplied", row["volume_runtime"]["status"])
        self.assertEqual(0., row["volume"]["bed_shift_applied_m"])
        self.assertNotEqual("applied", row["volume"]["bed_reference_status"])
        self.assertAlmostEqual(.96, row["volume"]["volume_m3"], places=5)

    def test_new_worker_instance_never_reuses_capture_epoch_namespace(self):
        worker, bridge, _, frame, _ = self.arm(save=False)
        row = self.finish(worker, frame, exported=False)
        replacement = self.worker(bridge, save=False)
        self.assertNotEqual(worker._capture_instance_id, replacement._capture_instance_id)
        self.assertTrue(row["volume_runtime"]["scope"]["capture_epoch"].startswith(worker._capture_instance_id + ":"))

    def test_registration_status_compatible_tuple_and_invalid_input_guard(self):
        base, loaded = scene(.4)
        self.assertEqual("unresolved", estimate_registration(base, loaded).status)
        self.assertEqual(estimate_dy(base, loaded), (0., "找不到车厢后沿"))
        bad = LoadedResult(cloud=np.array([[0., float("inf"), 0.]]))
        self.assertEqual("unresolved", estimate_registration(base, bad).status)


if __name__ == "__main__":
    unittest.main()
