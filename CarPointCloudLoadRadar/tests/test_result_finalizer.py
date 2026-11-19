"""真实停稳测量与后台成套归档；故障不能阻止采集下一车辆。"""
import copy
from datetime import datetime
import errno
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

from app.cloud_archive import ArchiveStore
from app.load_radar_bridge import LoadRadarBridge
from app.load_radar_transport import BridgeConfig
from app.pointcloud import Frame
from app.result_finalizer import ResultFinalizer
from app.service import ChannelWorker, EventBus
from tests import _helpers as helpers
from tests._observations import ObservationClock
from tests.test_load_radar_bridge import MemoryClient


def await_result(worker, count=1, exported=True, timeout=10.):
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        worker._drain_finalizer()
        rows = list(worker.history)
        if len(rows) >= count and (not exported or all(row.get("archive_status") == "complete" for row in rows[-count:])):
            return rows
        threading.Event().wait(.01)
    raise AssertionError("结果未完成: " + str(worker._finalizer.status(worker.ch.id)))


class TestResultFinalizer(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.clock = ObservationClock(self)
        self.clock.wall = 1704067200.
        self.workers = []
        self.bridges = []
        self.addCleanup(self.close)

    def close(self):
        for worker in self.workers:
            worker.stop()
            self.assertTrue(worker.close_results(10.), "测试不可遗留 RAM 待提交结果")
            worker.source.close()
        for bridge in self.bridges:
            bridge.close()

    def bridge(self):
        cfg = BridgeConfig(node_id="test", base_url="http://127.0.0.1:1", agent_key="k" * 40,
            storage_dir=str(Path(self.tmp.name) / "outbox"), algorithm_version="test-v1",
            channels={"CH1": {"calibration_id": "cal-1"}, "CH4": {"calibration_id": "cal-4"}}, enabled=True, timeout_sec=1, retry_sec=1)
        bridge = LoadRadarBridge(cfg, MemoryClient())
        self.bridges.append(bridge)
        return bridge

    def worker(self, bridge=None, save=True, channel="CH1", finalizer=None, live=False):
        settings, ch, _, _, _ = helpers.ctx()
        ch = copy.deepcopy(ch)
        ch.id, ch.mode = channel, "live" if live else "synthetic"
        ch.abs_output_dir = str(Path(self.tmp.name) / channel)
        ch.abs_baseline_dir = str(Path(self.tmp.name) / "baselines")
        from app.radar_source import build_source
        def fake_source(*args, **kwargs):
            kwargs["mode"] = "synthetic"
            return build_source(*args, **kwargs)
        with patch("app.application.channel_worker.build_source", side_effect=fake_source):
            worker = ChannelWorker(ch, EventBus(), settings.plate, save_override=save, plate_mode="null",
                                   integration=bridge, raw_frames=0, finalizer=finalizer)
        worker.settle_sec = 0.
        self.workers.append(worker)
        return worker

    @staticmethod
    def ground():
        rng = np.random.default_rng(0)
        return Frame(np.column_stack((rng.uniform(-1.8, 1.8, 8000), rng.uniform(-7, 7, 8000),
                                      rng.normal(-5., .01, 8000))).astype(np.float32))

    @classmethod
    def truck(cls):
        rng = np.random.default_rng(5)
        body = np.column_stack((rng.uniform(-1.2, 1.2, 40000), rng.uniform(-6, 6, 40000),
                                rng.uniform(-4.2, -2.5, 40000))).astype(np.float32)
        return Frame(np.concatenate((cls.ground().xyz, body)), np.arange(48000, dtype=np.uint16))

    def feed(self, worker, frame, count=30):
        for _ in range(count):
            self.clock.tick(worker, frame)
            # 本组专测归档/命令边界：明确提供已完成的内部识牌前置条件。
            # 真实到车准备握手及缺失门控由 test_load_radar_preparation 单独验证。
            bridge = worker.integration
            if bridge is not None and worker.eff_mode == "live":
                passage = bridge.current_passage(worker.ch.id)
                if passage:
                    bridge._prepared[worker.ch.id] = (passage,
                        "ready" if worker.ch.id in bridge._loaded else "baseline_required", None, time.monotonic())

    def enter(self, worker):
        self.feed(worker, self.ground(), 8)
        self.feed(worker, self.truck())

    def manifests(self, worker):
        return [json.loads(path.read_bytes()) for path in Path(worker.ch.abs_output_dir).rglob("manifest.json")]

    def test_real_stop_archives_all_roles_without_diagnostic_sampling(self):
        worker = self.worker()
        with patch("app.result_finalizer.make_candidate", side_effect=AssertionError("自动测量不能造候选")):
            self.enter(worker)
            rows = await_result(worker)
        meta = self.manifests(worker)[0]
        self.assertEqual(worker.raw_frames, 0)
        self.assertEqual(worker._detected_this_session, 1)
        self.assertEqual(worker.st.saved_count, 1)
        self.assertEqual(set(("observation.xyz", "vehicle.xyz", "carriage.xyz", "detection.xyz")) - set(meta["files"]), set())
        self.assertEqual(meta["files"]["observation.xyz"]["points"], 48000)
        self.assertIn("provenance", meta)
        self.assertIn("volume_support", meta)
        self.assertTrue(meta["passage_id"].startswith("local-"))
        self.assertFalse(meta["authoritative"])
        self.assertEqual(rows[0]["empty"], json.loads(Path(rows[0]["saved"] + ".meta.json").read_bytes())["empty"])

    def test_managed_archive_and_payload_have_identical_identity(self):
        bridge = self.bridge()
        worker = self.worker(bridge)
        self.enter(worker)
        row = await_result(worker)[0]
        payload = bridge.outbox.get(row["measurement_id"]).payload
        bundle = bridge.outbox.archives.get(row["measurement_id"])
        self.assertEqual(bundle.metadata["passage_id"], payload["passageId"])
        self.assertEqual(payload["sourceMode"], "demo")
        self.assertIsNone(payload["volumeM3"])
        source_times = [part["frame"]["ts"] for part in bundle.metadata["volume_support"]["segments"]]
        self.assertAlmostEqual(datetime.fromisoformat(payload["measurementStartedAt"].replace("Z", "+00:00")).timestamp(),
                               min(source_times), places=5)
        self.assertAlmostEqual(datetime.fromisoformat(payload["measurementEndedAt"].replace("Z", "+00:00")).timestamp(),
                               max(source_times), places=5)
        self.assertEqual(row["integration"]["status"], "queued")

    def test_quarantined_result_never_creates_business_passage(self):
        bridge = self.bridge()
        worker = self.worker(bridge)
        self.feed(worker, self.truck())
        row = await_result(worker)[0]
        self.assertIsNone(bridge.outbox.get(row["measurement_id"]))
        self.assertIsNone(self.manifests(worker)[0]["passage_id"])
        self.assertEqual(row["integration"]["status"], "quarantined")
        self.assertIsNone(worker._finalizer._store(worker.ch.abs_output_dir).journal.reference("CH1"))

    def test_failed_commit_keeps_original_and_observes_next_vehicle(self):
        bridge = self.bridge()
        worker = self.worker(bridge)
        captured = []
        original = bridge.outbox.save
        def fail(payload, **kwargs):
            captured.append((json.dumps(payload, sort_keys=True), kwargs["archive"]))
            raise OSError(errno.ENOSPC, "注入磁盘空间不足")
        with patch.object(bridge.outbox, "save", side_effect=fail):
            self.enter(worker)
            deadline = time.perf_counter() + 8
            while not captured and time.perf_counter() < deadline:
                threading.Event().wait(.01)
            self.assertTrue(captured)
            first = json.loads(captured[0][0])
            self.feed(worker, self.ground(), 8)
            self.feed(worker, self.truck(), 30)
            second_passage = bridge.current_passage("CH1")
            self.assertNotEqual(second_passage, first["passageId"])
            self.assertEqual(worker._detected_this_session, 0)
            self.assertEqual(worker.st.saved_count, 0)
            capacity = worker._finalizer.capacity_snapshot()["runtime"]
            self.assertEqual(1, capacity["slot_count"])
            self.assertGreater(capacity["estimated_pending_bytes"], 0)
            self.assertEqual({"commit_pending": 1}, capacity["states"])
            self.assertFalse(worker._finalizer.can_accept("CH1"))
            self.assertFalse(any(e.get("kind") == "durable" for e in worker._finalizer._events["CH1"]))
            self.assertFalse(worker.close_results(0.))
        # close_results 已禁止新测量，但会补交原结果，不能换 ID。
        rows = await_result(worker)
        self.assertEqual(rows[0]["measurement_id"], first["measurementId"])
        self.assertEqual(bridge.outbox.get(first["measurementId"]).payload, first)
        self.assertEqual(len(worker.history), 1)
        self.assertFalse(worker.st.last_result)
        self.assertEqual(worker._saved_this_session, 0)
        self.assertEqual(bridge.current_passage("CH1"), second_passage)
        self.assertTrue(all(item == captured[0] for item in captured))

    def test_slow_roi_does_not_block_new_reads_or_other_channel(self):
        import app.result_finalizer as module
        worker = self.worker()
        other = self.worker(channel="CH2")
        entered, release = threading.Event(), threading.Event()
        original = module.extract_observed_roi
        def slow(*args, **kwargs):
            if threading.current_thread().name.endswith("CH1"):
                entered.set()
                if not release.wait(10):
                    raise AssertionError("屏障超时")
            return original(*args, **kwargs)
        with patch.object(module, "extract_observed_roi", side_effect=slow):
            try:
                self.enter(worker)
                self.assertTrue(entered.wait(3))
                self.feed(worker, self.ground(), 8)
                self.assertEqual(worker._session_id, 0)
                self.enter(other)
                await_result(other)
                self.assertEqual(worker.st.saved_count, 0)
            finally:
                release.set()
            await_result(worker)

    def test_save_off_has_no_archive_io_but_managed_outbox_is_durable(self):
        bridge = self.bridge()
        worker = self.worker(bridge, save=False)
        with patch("app.result_finalizer.ArchiveStore", side_effect=AssertionError("禁止云图落库")):
            self.enter(worker)
            row = await_result(worker, exported=False)[0]
        self.assertIsNotNone(bridge.outbox.get(row["measurement_id"]))
        self.assertEqual(row["archive_status"], "disabled")
        self.assertFalse(Path(worker.ch.abs_output_dir).exists())

    def test_archive_recovery_does_not_construct_source_or_detector(self):
        worker = self.worker()
        with patch("app.cloud_archive._write_once", side_effect=OSError(errno.ENOSPC, "注入投影磁盘不足")):
            self.enter(worker)
            row = await_result(worker, exported=False)[0]
            runtime = worker._finalizer.capacity_snapshot()["runtime"]
            self.assertEqual(0, runtime["slot_count"])
            self.assertTrue(worker._finalizer.can_accept("CH1"))
            store = worker._finalizer._stores[worker.ch.abs_output_dir]
            self.assertIn(row["measurement_id"], store.journal.pending("CH1"))
            self.assertFalse(any(e.get("kind") == "exported" for e in worker._finalizer._events["CH1"]))
            self.assertTrue(worker.close_results(5.))
        with patch("app.application.channel_worker.build_source", side_effect=AssertionError("不应访问硬件")), \
                patch("app.result_finalizer.prepare_result", side_effect=AssertionError("不应重新计算")):
            recovery = ResultFinalizer({"CH1": (worker.ch.abs_output_dir, True)})
            recovery.start()
            try:
                deadline = time.perf_counter() + 8
                while not self.manifests(worker) and time.perf_counter() < deadline:
                    threading.Event().wait(.01)
                self.assertEqual(self.manifests(worker)[0]["measurement_id"], row["measurement_id"])
            finally:
                self.assertTrue(recovery.close(5.))

    def test_budget_refuses_new_reservation_without_dropping_original(self):
        finalizer = ResultFinalizer({"CH1": (self.tmp.name, False), "CH2": (self.tmp.name, False)}, byte_budget=100)
        ticket = finalizer.reserve("CH1", "origin", "local-p", 100)
        self.assertFalse(finalizer.can_accept("CH1"))            # 本道已有在途结果
        self.assertFalse(finalizer.can_accept("CH2", 101))       # 超过本道自己的预算
        # ★ 预算按车道隔离（2026-09-23）：CH1 占满它自己的预算，不影响 CH2。
        #   原来这里断言 CH2 也被拒 —— 那正是 4 车道同时来车时一条道挡住另一条道的根源。
        self.assertTrue(finalizer.can_accept("CH2", 100))
        self.assertFalse(finalizer.close(0))
        self.assertEqual(finalizer._slots["CH1"]["identity"], ticket)
        # 未启动线程、未创建库的纯 RAM 夹具，无生产结果被丢弃。

    def test_four_lanes_reserve_concurrently_without_interference(self):
        """★ 4 条车道同时出结果：每条道都能立刻拿到自己的槽位，互不挡、不串。

        按真实整帧量级给预留（每份约 196 MB，见 docs/64）：原来的全局预算下 4 份只放得下 2 份。
        并发预留用屏障对齐到同一时刻；再验证一条道卡住（槽位不释放）时，其它道照样能接新测量。
        """
        lanes = ("CH1", "CH2", "CH3", "CH4")
        finalizer = ResultFinalizer({lane: (self.tmp.name, False) for lane in lanes})
        estimate = 196 * 1024 * 1024
        barrier = threading.Barrier(len(lanes))
        tickets, errors = {}, []

        def reserve(lane):
            try:
                barrier.wait(5)
                tickets[lane] = finalizer.reserve(lane, "origin-" + lane, "passage-" + lane, estimate)
            except Exception as error:                               # noqa: BLE001
                errors.append((lane, repr(error)))

        threads = [threading.Thread(target=reserve, args=(lane,)) for lane in lanes]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(10)
        self.assertEqual([], errors)
        self.assertEqual(set(lanes), set(tickets))
        for lane in lanes:
            identity = finalizer._slots[lane]["identity"]
            self.assertEqual(identity, tickets[lane])
            self.assertEqual(lane, identity.channel)                 # 槽位不串道
            self.assertEqual("origin-" + lane, identity.origin_token)
        self.assertEqual(len(set(t.measurement_id for t in tickets.values())), len(lanes))
        # 一条道卡住不释放：别的道释放后照样能接下一份，卡住的那条道只挡它自己
        for lane in lanes[1:]:
            finalizer._slots.pop(lane)
            self.assertTrue(finalizer.can_accept(lane, estimate))
        self.assertFalse(finalizer.can_accept("CH1", estimate))
        self.assertFalse(finalizer.close(0))

    def test_missing_baseline_remeasure_is_rejected_without_poisoning_channel(self):
        bridge = self.bridge()
        worker = self.worker(bridge, live=True)
        self.enter(worker)
        await_result(worker)
        body = dict(requestId="missing-base", channelId="CH1", passageId=bridge.current_passage("CH1"),
                    kind="remeasure", confirmedEmpty=False)
        bridge.queue_command(body)
        self.feed(worker, self.truck(), 1)
        self.assertEqual(bridge.command("missing-base")["status"], "failed")
        self.assertTrue(worker._finalizer.can_accept("CH1"))
        self.assertNotIn("CH1", bridge._reserved)
        self.feed(worker, self.ground(), 8)
        self.feed(worker, self.truck())
        self.assertEqual(len(await_result(worker, count=2)), 2)

    def test_candidate_nonempty_rejection_releases_only_unsubmitted_command(self):
        bridge = self.bridge()
        worker = self.worker(bridge, live=True, channel="CH4")
        self.enter(worker)
        row = await_result(worker)[0]
        self.assertFalse(row["empty"], "夹具必须真实判定非空")
        self.assertTrue(row["fit_ok"], "拟合成功不能冒充空车")
        body = dict(requestId="not-empty", channelId="CH4", passageId=bridge.current_passage("CH4"),
                    kind="baseline-capture", confirmedEmpty=True)
        bridge.queue_command(body)
        with patch("app.result_finalizer.make_candidate", side_effect=AssertionError("不可生成载货候选")):
            self.feed(worker, self.truck(), 1)
        self.assertEqual(bridge.command("not-empty")["status"], "failed")
        self.assertTrue(worker._finalizer.can_accept("CH4"))
        self.assertEqual(len(worker.history), 1)

    def test_rejected_manual_request_does_not_consume_automatic_measurement(self):
        bridge = self.bridge()
        worker = self.worker(bridge, live=True, channel="CH4")
        worker.settle_sec = 5.
        self.feed(worker, self.ground(), 8)
        self.feed(worker, self.truck(), 26)
        self.assertEqual(worker._detected_this_session, 0)
        self.assertEqual(len(worker._integration_frames), 3)
        bridge.queue_command(dict(requestId="early-capture", channelId="CH4",
            passageId=bridge.current_passage("CH4"), kind="baseline-capture", confirmedEmpty=True))
        self.feed(worker, self.truck(), 1)
        self.assertEqual(bridge.command("early-capture")["status"], "failed")
        self.assertEqual(worker._detected_this_session, 0)
        self.feed(worker, self.truck(), 65)
        await_result(worker)
        self.assertEqual(worker._detected_this_session, 1)

    def test_database_lock_and_file_io_do_not_stall_reads(self):
        import app.cloud_archive as module
        bridge = self.bridge()
        worker = self.worker(bridge)
        entered, release = threading.Event(), threading.Event()
        original = bridge.outbox.save
        def slow_database(*args, **kwargs):
            with bridge.outbox._lock:
                entered.set()
                if not release.wait(10):
                    raise OSError("测试超时")
                return original(*args, **kwargs)
        with patch.object(bridge.outbox, "save", side_effect=slow_database):
            try:
                self.enter(worker)
                self.assertTrue(entered.wait(3))
                done = threading.Event()
                thread = threading.Thread(target=lambda: (self.feed(worker, self.ground(), 8), done.set()))
                thread.start()
                self.assertTrue(done.wait(1.))
                thread.join()
            finally:
                release.set()
        await_result(worker)
        # 第二次测量只阻塞文件叶子；outbox提交、采集仍正常。
        entered.clear()
        release.clear()
        write = module._write_once
        def slow_file(*args, **kwargs):
            entered.set()
            if not release.wait(10):
                raise OSError("测试超时")
            return write(*args, **kwargs)
        with patch.object(module, "_write_once", side_effect=slow_file):
            try:
                self.feed(worker, self.truck())
                self.assertTrue(entered.wait(3))
                self.feed(worker, self.ground(), 8)
                self.assertEqual(worker._session_id, 0)
                self.assertFalse(worker.close_results(0))
            finally:
                release.set()
        self.assertTrue(worker.close_results(5))
        self.assertEqual(len(worker.history), 2)

    def test_accumulated_detection_and_volume_members_are_all_retained(self):
        worker = self.worker()
        worker.settle_accumulate, worker.settle_sec, worker.settle_max_frames = True, .5, 8
        self.enter(worker)
        await_result(worker)
        meta = self.manifests(worker)[0]
        primary = meta["provenance"]["segments"]
        self.assertGreaterEqual(len(primary), 5)
        self.assertEqual(meta["volume_support"]["primary_members"], list(range(len(primary))))
        self.assertEqual(meta["volume_support"]["extra_members"], [len(primary) - 3, len(primary) - 2])

    def test_accumulation_respects_budget_without_permanent_self_block(self):
        finalizer = ResultFinalizer({"CH1": (str(Path(self.tmp.name) / "CH1"), True)}, byte_budget=60 * 1024 * 1024)
        worker = self.worker(finalizer=finalizer)
        worker.settle_accumulate, worker.settle_sec, worker.settle_max_frames = True, 3., 60
        self.feed(worker, self.ground(), 8)
        self.feed(worker, self.truck(), 65)
        await_result(worker)
        self.assertEqual(worker._detected_this_session, 1)
        self.assertLessEqual(len(self.manifests(worker)[0]["provenance"]["segments"]), 2)

    def test_durable_slot_releases_large_frozen_inputs_while_thread_idle(self):
        import gc
        import weakref
        import app.result_finalizer as module
        original = module.prepare_result
        references = []
        def observe_inputs(pending, bridge):
            prepared = original(pending, bridge)
            references.extend((weakref.ref(pending), weakref.ref(prepared)))
            return prepared
        worker = self.worker()
        with patch.object(module, "prepare_result", side_effect=observe_inputs):
            self.enter(worker)
            await_result(worker)
        # Mock 本身保留 call_args；离开作用域后先释放它，检查实际后台线程栈。
        gc.collect()
        self.assertEqual(len(references), 2)
        self.assertTrue(all(reference() is None for reference in references))

    def test_optional_raw_sampling_cannot_self_block_primary_archive(self):
        finalizer = ResultFinalizer({"CH1": (str(Path(self.tmp.name) / "CH1"), True)}, byte_budget=35 * 1024 * 1024)
        worker = self.worker(finalizer=finalizer)
        worker.settle_accumulate, worker.settle_sec, worker.raw_frames = True, 3., 3
        self.feed(worker, self.ground(), 8)
        self.feed(worker, self.truck(), 65)
        await_result(worker)
        self.assertEqual(worker._detected_this_session, 1)
        self.assertEqual(self.manifests(worker)[0]["files"]["observation.xyz"]["points"], 48000)

    def test_early_export_does_not_pin_completed_slot_in_exporter_stack(self):
        import gc
        import weakref
        import app.result_finalizer as module
        original_prepare, original_save = module.prepare_result, ArchiveStore.save
        references = []
        committed, release = threading.Event(), threading.Event()
        def observe_inputs(pending, bridge):
            prepared = original_prepare(pending, bridge)
            references.extend((weakref.ref(pending), weakref.ref(prepared)))
            return prepared
        def commit_then_hold(store, bundle):
            value = original_save(store, bundle)
            committed.set()
            if not release.wait(10):
                raise OSError("测试屏障超时")
            return value
        worker = self.worker()
        with patch.object(module, "prepare_result", side_effect=observe_inputs), \
                patch.object(ArchiveStore, "save", new=commit_then_hold):
            try:
                self.enter(worker)
                self.assertTrue(committed.wait(3.))
                deadline = time.perf_counter() + 5
                early = False
                while time.perf_counter() < deadline:
                    with worker._finalizer._lock:
                        early = bool(worker._finalizer._slots["CH1"].get("exported_path"))
                    if early:
                        break
                    threading.Event().wait(.01)
                self.assertTrue(early, "必须覆盖导出先于 durable 事件的实际竞争")
            finally:
                release.set()
            await_result(worker)
        gc.collect()
        self.assertTrue(all(reference() is None for reference in references))

    @staticmethod
    def empty_truck():
        from tests.test_cloud_roi import labelled_scene
        raw = labelled_scene()[0]
        rng = np.random.default_rng(5)
        floor = np.column_stack((rng.uniform(-1.2, 1.2, 50000), rng.uniform(-5, 5, 50000),
                                 rng.normal(1., .002, 50000)))
        return Frame((np.concatenate((raw.xyz[raw.xyz[:, 2] > .1], floor)) + [0, 0, -5]).astype(np.float32))

    def test_empty_reference_updates_without_automatic_formal_baseline(self):
        worker = self.worker()
        with patch("app.result_finalizer.make_candidate", side_effect=AssertionError("不自动创建正式基线")):
            self.feed(worker, self.ground(), 8)
            self.feed(worker, self.empty_truck())
            row = await_result(worker)[0]
        self.assertTrue(row["empty"])
        reference = worker._finalizer._store(worker.ch.abs_output_dir).journal.reference("CH1")
        self.assertEqual(reference["measurement_id"], row["measurement_id"])
        self.assertFalse(reference["authoritative"])
        self.assertIsNone(worker.baseline)
        self.assertFalse(Path(worker.ch.abs_baseline_dir).exists())

    def test_only_live_CH4_empty_capture_creates_automatic_candidate_without_a_command(self):
        from app.load_radar_contract import _read_baseline
        bridge = self.bridge()
        for channel in ("CH1", "CH4"):
            with self.subTest(channel=channel):
                worker = self.worker(bridge, live=True, channel=channel)
                worker.settle_sec = .5  # 自动基线沿用至少两帧沉降证据，不用零等待夹具绕过门槛。
                self.feed(worker, self.ground(), 8)
                self.feed(worker, self.empty_truck())
                row = await_result(worker)[0]
                item = bridge.outbox.get(row["measurement_id"])
                self.assertTrue(row["empty"])
                self.assertEqual(channel == "CH4", item.payload.get("automaticBaseline", False), str(row))
                if channel == "CH4":
                    self.assertGreater(_read_baseline(item.asset).n, 20000)
                else:
                    self.assertIsNone(item.asset)

    def test_manual_candidate_npz_archive_and_completion_commit_once(self):
        import app.result_finalizer as module
        from app.load_radar_contract import _read_baseline
        bridge = self.bridge()
        worker = self.worker(bridge, live=True, channel="CH4")
        empty = self.empty_truck()
        self.feed(worker, self.ground(), 8)
        self.feed(worker, empty)
        self.assertTrue(await_result(worker)[0]["empty"])
        body = dict(requestId="capture", channelId="CH4", passageId=bridge.current_passage("CH4"),
                    kind="baseline-capture", confirmedEmpty=True)
        bridge.queue_command(body)
        failed = threading.Event()
        original_save = bridge.outbox.save
        submitted = []
        def fail_once(payload, **kwargs):
            submitted.append((json.dumps(payload, sort_keys=True), kwargs["candidate_bytes"], kwargs["archive"]))
            if not failed.is_set():
                failed.set()
                raise OSError("模拟第一次联合提交失败")
            return original_save(payload, **kwargs)
        with patch.object(module, "make_candidate", wraps=module.make_candidate) as make, \
                patch.object(bridge.outbox, "save", side_effect=fail_once):
            self.feed(worker, empty, 1)
            self.assertTrue(failed.wait(3.))
            self.assertEqual(bridge.command("capture")["status"], "running")
            row = await_result(worker, count=2)[-1]
            make.assert_called_once()
            self.assertTrue(all(item == submitted[0] for item in submitted))
            accepted = bridge.queue_command(body)
            self.feed(worker, empty, 3)
            make.assert_called_once()
        self.assertEqual(accepted["measurementId"], row["measurement_id"])
        receipt = bridge.outbox.command_result("capture")
        self.assertEqual(receipt["measurement"].measurement_id, row["measurement_id"])
        self.assertGreater(_read_baseline(receipt["measurement"].asset).n, 20000)
        self.assertIsNotNone(bridge.outbox.archives.get(row["measurement_id"]))
        self.assertIsNone(worker.baseline)
        self.assertEqual(len(worker.history), 2)

    def test_manual_remeasure_freezes_the_actual_baseline_version(self):
        from app.detect import EmptyBaseline, detect_empty
        bridge = self.bridge()
        worker = self.worker(bridge, live=True)
        self.enter(worker)
        await_result(worker)
        passage = bridge.current_passage("CH1")
        empty = detect_empty(self.empty_truck(), worker.calib)
        baseline = EmptyBaseline.from_empty_result(empty, "2023-12-31T00:00:00Z")
        version = dict(versionId="manual-version", activatedAt="2023-12-31T00:00:00Z", assetSha256="b" * 64)
        bridge._loaded["CH1"] = (passage, version, baseline, {"licensePlate": "测试挂车"},
                                 self.clock.mono, worker._integration_hash)
        body = dict(requestId="remeasure", channelId="CH1", passageId=passage,
                    kind="remeasure", confirmedEmpty=False)
        bridge.queue_command(body)
        with patch("app.result_finalizer.make_candidate", side_effect=AssertionError("重测不能产生新基线")):
            self.feed(worker, self.truck(), 1)
            # 修改活动缓存不能影响已经冻结的本次实际版本。
            bridge._loaded.clear()
            row = await_result(worker, count=2)[-1]
        payload = bridge.outbox.get(row["measurement_id"]).payload
        archive = bridge.outbox.archives.get(row["measurement_id"])
        self.assertEqual(payload["baselineVersionId"], "manual-version")
        self.assertEqual(archive.metadata["baseline_version_id"], "manual-version")
        self.assertEqual(bridge.queue_command(body)["measurementId"], row["measurement_id"])
        self.assertEqual(len(worker.history), 2)


if __name__ == "__main__":
    unittest.main()

class TestPreparationFailure(unittest.TestCase):
    """★ 构造/冻结失败不能把通道永久钉死（2026-09-18 复现的缺陷）。

    原来的形态：`capture_failed` 把槽位置成 `preparation_failed`，而
    `_commit_loop` 的 ready 判据把这个状态排除、全项目又没有一处 pop
    掉它 —— 于是 `can_accept` 恒 False，`service._tick` 的沉降窗口门控
    恒不通过，**该通道此后每辆车都不测、不落盘**。

    而且是静默的：`health` 横幅只管"从不回到空闲"（这里会话照常结束），
    "停稳 30s 仍未稳定"要求 `stab.state != STABLE`（这里车恰恰判成
    STABLE），两个前端都不渲染 `finalizer.state`。除了开头那一条会被
    poll 掉的 error 事件之外，现场什么都看不到。

    对照：**提交**失败一直是有退避重放的
    （`test_failed_commit_keeps_original_and_observes_next_vehicle`），
    漏的正是**构造**失败这一条。
    """

    def finalizer(self, channel="CH1"):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        fin = ResultFinalizer({channel: (str(Path(tmp.name) / channel), False)}, bridge=None)
        self.addCleanup(fin.close, .5)
        return fin

    @staticmethod
    def pending(identity, archive_enabled=True):
        from app.cloud_observation import FrameSnapshot
        from app.result_finalizer import PendingResult
        frame = Frame(np.zeros((3, 3), np.float64), np.zeros(3, np.uint16), 1, 1704067200., 0, 1)
        return PendingResult(identity=identity, result_json=b'{"empty":true}',
            detection=FrameSnapshot.freeze(frame), detection_indices=np.arange(3, dtype="<i8").tobytes(),
            detection_rotation=np.eye(3, dtype="<f8").tobytes(), observation_window=None,
            volume_window=None, volume_extra_members=(), extra_frames=(), configuration_hash="h",
            algorithm_version="v", baseline_version_id=None, source_mode="live", lane_seed=None,
            ground_z=-5., archive_enabled=archive_enabled)

    @staticmethod
    def wait(predicate, timeout=15.):
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            if predicate():
                return True
            threading.Event().wait(.02)
        return False

    def test_capture_failure_releases_the_slot_for_the_next_vehicle(self):
        """★ 本次修复的目标之一：冻结从未完成 -> 下一辆车必须还能测。

        这一步没有任何结果字节存在（outbox 里没有这个 measurement_id、
        投影也没发生），所以释放它不会让谁留下两条记录。
        """
        fin = self.finalizer()
        first = fin.reserve("CH1", "token-1", None, 1000)
        fin.capture_failed(first, ValueError("测量缺少原始运行许可摘要，禁止事后补造"))
        self.assertEqual("idle", fin.status("CH1")["state"])
        self.assertTrue(fin.can_accept("CH1"), "冻结失败之后通道被永久钉死了")
        second = fin.reserve("CH1", "token-2", None, 1000)
        self.assertNotEqual(first.measurement_id, second.measurement_id)

    def test_capture_failure_leaves_a_standing_alert_not_only_one_event(self):
        """一次性事件会被 poll 掉；"这个通道刚丢了一次测量"必须一直挂着。"""
        fin = self.finalizer()
        identity = fin.reserve("CH1", "token-1", None, 1000)
        fin.capture_failed(identity, ValueError("boom"), detail="检测输入不是本次原观测的精确 ROI 行子集")
        self.assertTrue(fin.poll("CH1"), "应当同时发一次事件")
        self.assertFalse(fin.poll("CH1"), "事件被消费掉了")
        self.assertIn("ROI", fin.alert("CH1"), "常驻告警丢了上游原因 —— 现场无从查起")

    def test_transient_preparation_failure_is_replayed_under_the_same_id(self):
        """★ 本次修复的目标之二：构造失败要和提交失败一个待遇 —— 退避重放。

        `prepare_result` 是 `pending` 的纯函数，重试用同一份冻结字节，
        不存在换 ID 的问题。
        """
        import app.result_finalizer as module
        fin = self.finalizer()
        identity = fin.reserve("CH1", "token-1", None, 1000)
        calls = []
        def flaky(pending, bridge=None):
            calls.append(1)
            if len(calls) <= 3:
                raise RuntimeError("模拟瞬时失败")
            return module.PreparedResult(None, None, None)
        with patch.object(module, "prepare_result", side_effect=flaky):
            fin.submit(self.pending(identity))
            self.assertTrue(self.wait(lambda: fin.status("CH1")["state"] == "idle"),
                            "构造失败之后再也没有重试过: " + str(fin.status("CH1")))
        self.assertGreater(len(calls), 3, "根本没有重放")
        events = [event for event in fin.poll("CH1") if event["kind"] == "durable"]
        self.assertEqual(1, len(events))
        self.assertEqual(identity.measurement_id, events[0]["identity"].measurement_id,
                         "重放换了 ID —— 同一辆车会留下两条记录")
        self.assertEqual("", fin.alert("CH1"), "走通之后常驻告警没撤")
        self.assertTrue(fin.can_accept("CH1"))

    def test_permanent_preparation_failure_is_visible_instead_of_silent(self):
        """确定性失败修不好，但**不许静默** —— 槽位占着是有意的背压
        （结果还在内存里，丢掉就是丢数据），现场必须看得见它卡住了。"""
        import app.result_finalizer as module
        fin = self.finalizer()
        identity = fin.reserve("CH1", "token-1", None, 1000)
        def boom(pending, bridge=None):
            raise ValueError("归档缺少真实原观测，禁止用判定云替代")
        with patch.object(module, "prepare_result", side_effect=boom):
            fin.submit(self.pending(identity))
            self.assertTrue(self.wait(lambda: bool(fin.alert("CH1"))),
                            "卡死了却没有任何常驻告警 —— 这正是从前的形态")
            self.assertGreaterEqual(fin.status("CH1")["attempts"], fin.ALERT_AFTER_ATTEMPTS)
            self.assertIn("ValueError", fin.alert("CH1"))
            self.assertFalse(fin.can_accept("CH1"))

    def test_worker_health_banner_carries_the_finalizer_alert(self):
        """两个前端都只渲染 `health` 一个字段，所以告警必须并进去 ——
        只放在 `finalizer.state` 里等于没报。"""
        outer = TestResultFinalizer("test_real_stop_archives_all_roles_without_diagnostic_sampling")
        self.addCleanup(outer.doCleanups)
        outer.setUp()
        worker = outer.worker()
        worker._ensure_finalizer()
        identity = worker._finalizer.reserve(worker.ch.id, "token-1", None, 1000)
        worker._finalizer.capture_failed(identity, ValueError("boom"), detail="观测缺少有效 seq/ts/gen/packets 身份")
        text = worker._health_text()
        self.assertIn("冻结失败", text)
        self.assertIn("seq", text)
        self.assertEqual(text, worker.snapshot()["health"])

    def test_idle_transition_does_not_wipe_the_finalizer_alert(self):
        """`st.health`（有车判据失效）回到空闲就清掉是对的 —— 它量的就是
        "从不空闲"。但收尾告警是另一回事：车照常来去、只是测不出来。
        两条必须分开存，否则前一条的清除会把后一条一起抹掉。"""
        outer = TestResultFinalizer("test_real_stop_archives_all_roles_without_diagnostic_sampling")
        self.addCleanup(outer.doCleanups)
        outer.setUp()
        worker = outer.worker()
        worker._ensure_finalizer()
        identity = worker._finalizer.reserve(worker.ch.id, "token-1", None, 1000)
        worker._finalizer.capture_failed(identity, ValueError("boom"))
        worker.st.health = "有车判据可能已失效"
        self.assertIn("有车判据", worker._health_text())
        worker.st.health = ""                      # 车回到空闲时发生的事
        self.assertIn("冻结失败", worker._health_text(), "收尾告警被空闲清除连带抹掉了")


class TestExportFailureIsVisible(unittest.TestCase):
    """★ 归档导出失败不能像从前那样**完全无声**。

    2026-09-22 审查（docs/52 第一节）：`_export_loop` 的外层 except 是
    `pass`，捕的是 `_store()`（归档库初始化）与 `journal.pending_after()`
    （查询）—— 而本类的常驻告警设置点全在 `_export_one` 里面，
    这两条路径一个告警都不设。

    实测复现（让 `_store()` 持续抛 OSError）：

        失败 28 次 / 3 秒（约 9 次/秒）
        alert('CH1')  : ''          <- 看板健康横幅读的就是它
        status('CH1') : {'state': 'idle', 'error': ''}
        events        : []

    于是「归档库打不开 / 权限不足 / 磁盘满 / 库文件损坏」这一整类故障，
    表现是服务一切正常、看板一切正常，而测量一份都导不出去。
    现场已经吃过这一类亏：`cloud_archives` 整整 0 行，没有任何人察觉。
    """

    def _finalizer(self, boom):
        """起一个只会在导出外层失败的 finalizer；boom 为 None 表示正常。"""
        tmp = tempfile.mkdtemp()
        fin = ResultFinalizer({"CH1": (tmp, True)})
        self.addCleanup(fin._export_stop.set)
        self.calls = []

        def store(_output_dir):
            self.calls.append(1)
            if boom is not None:
                raise boom
            journal = type("J", (), {"pending_after": staticmethod(lambda *a, **k: [])})()
            return type("S", (), {"journal": journal})()

        fin._store = store
        fin.start()
        return fin

    def _wait_alert(self, fin, timeout=12.):
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            if fin.alert("CH1"):
                return fin.alert("CH1")
            time.sleep(0.05)
        return ""

    def test_persistent_failure_raises_a_standing_alert(self):
        fin = self._finalizer(OSError("归档库打不开"))
        text = self._wait_alert(fin)
        self.assertTrue(text, "连续失败却一个告警都没有 —— 又回到无声了")
        self.assertIn("导出", text)
        self.assertIn("OSError", text, "必须带异常类型，否则现场无从下手")
        self.assertIn("归档库打不开", text, "必须带异常消息本身")

    def test_the_alert_is_standing_not_one_shot(self):
        """★ 必须是常驻告警，不是一次性事件 —— 事件被 poll 一次就没了。"""
        fin = self._finalizer(OSError("磁盘满"))
        self.assertTrue(self._wait_alert(fin))
        fin.poll("CH1")                          # 看板轮询一次
        time.sleep(0.3)
        self.assertTrue(fin.alert("CH1"), "poll 一次之后告警就消失了")

    def test_it_emits_at_most_one_event_per_episode(self):
        """★ 每次失败都发事件会把通道堵死：can_accept 要求待发事件 < 8。"""
        fin = self._finalizer(OSError("库文件损坏"))
        self.assertTrue(self._wait_alert(fin))
        time.sleep(1.0)
        events = [e for e in fin.poll("CH1") if e["kind"] == "error"]
        self.assertEqual(len(events), 1, "一轮失败只该发一条事件，实得 %d" % len(events))

    def test_recovery_clears_the_alert(self):
        fin = self._finalizer(OSError("临时不可用"))
        self.assertTrue(self._wait_alert(fin))
        fin._store = lambda _d: type("S", (), {"journal": type(
            "J", (), {"pending_after": staticmethod(lambda *a, **k: [])})()})()
        deadline = time.perf_counter() + 40.
        while time.perf_counter() < deadline and fin.alert("CH1"):
            time.sleep(0.1)
        self.assertEqual(fin.alert("CH1"), "", "恢复之后告警该撤掉")

    def test_failures_back_off_instead_of_spinning(self):
        """从前是约 9 次/秒地空转抛异常；退避之后同样时长内要少得多。"""
        fin = self._finalizer(OSError("打不开"))
        time.sleep(3.0)
        self.assertLess(len(self.calls), 12,
                        "3 秒内失败 %d 次，退避没起作用" % len(self.calls))
        self.assertGreater(len(self.calls), 2, "完全不重试也不对")

    def test_export_alert_survives_a_successful_commit(self):
        """★★ 导出告警与提交告警必须**分开存**。

        `_commit_loop` 每成功提交一份就会把 `_alerts[channel]` 清成空串。
        两者共用一个槽的话，那次成功提交会顺手把还在持续的导出告警抹掉 ——
        而导出侧其实一份都没导出去。
        """
        fin = self._finalizer(OSError("归档库打不开"))
        self.assertTrue(self._wait_alert(fin))
        with fin._lock:                          # 模拟一次成功提交的收尾动作
            fin._alerts["CH1"] = ""
        self.assertTrue(fin.alert("CH1"), "提交侧的清除把导出告警一起抹掉了")

    def test_a_healthy_export_loop_stays_quiet(self):
        """反过来：正常时不能刷告警，否则横幅长期挂着等于没有。"""
        fin = self._finalizer(None)
        time.sleep(1.0)
        self.assertEqual(fin.alert("CH1"), "")
        self.assertEqual([e for e in fin.poll("CH1") if e["kind"] == "error"], [])
        self.assertGreater(len(self.calls), 3, "这条用例的前提没成立：循环根本没跑")


class TestPerFileExportFailureIsVisible(unittest.TestCase):
    """★ 每一份**落盘**失败也不能无声 —— 与上一个类是同一个洞的另一半。

    `_export_one` 原来是 `except Exception: return False`，异常整个丢掉。
    于是"输出目录只读 / 权限不足 / 磁盘满"这一类故障的表现是：
    每 2 秒重试一份、一份都导不出去，而三处全是空的。实测（2026-09-22）：

        export 被调用 : 108 次 / 4 秒
        alert('CH1')  : ''
        status('CH1') : {'state': 'idle', 'error': ''}
        事件          : []

    ▲ 但**单份毒包不该报警** —— `pending_after` 的游标会轮过去，其余照常导。
      判据因此是"这一轮一份都没导成"，不是"有失败"。
    """

    def _finalizer(self, export):
        tmp = tempfile.mkdtemp()
        fin = ResultFinalizer({"CH1": (tmp, True)})
        self.addCleanup(fin._export_stop.set)
        self.tries = []
        outer = self

        class Journal:
            @staticmethod
            def pending_after(_ch, _cursor, limit=8):
                return ["m-%03d" % i for i in range(3)]

            @staticmethod
            def export(measurement, _output_dir, channel=None):
                outer.tries.append(measurement)
                return export(measurement)

        fin._store = lambda _d: type("S", (), {"journal": Journal()})()
        fin.start()
        return fin

    def _wait_alert(self, fin, timeout=12.):
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            if fin.alert("CH1"):
                return fin.alert("CH1")
            time.sleep(0.05)
        return ""

    @staticmethod
    def _boom(_m):
        raise OSError("落盘失败：目录只读")

    def test_every_file_failing_raises_a_standing_alert(self):
        fin = self._finalizer(self._boom)
        text = self._wait_alert(fin)
        self.assertTrue(text, "一份都导不出去却没有任何告警")
        self.assertIn("OSError", text)
        self.assertIn("目录只读", text, "必须带异常消息，只报'失败了'现场无从下手")

    def test_the_alert_says_which_stage_failed(self):
        """★ 要能分开"库打不开"与"落盘失败"——两者的处置完全不同。"""
        fin = self._finalizer(self._boom)
        self.assertIn("导出落盘", self._wait_alert(fin))

    def test_a_single_poison_pill_does_not_alert(self):
        """★ 一份坏、其余照常导 = 有进展，不该惊动人（游标会轮过去）。"""
        def export(measurement):
            if measurement == "m-001":
                raise OSError("这一份坏了")
            return "/tmp/%s.xyz" % measurement
        fin = self._finalizer(export)
        time.sleep(3.0)
        self.assertEqual(fin.alert("CH1"), "",
                         "单份毒包不该报警，否则告警横幅会长期挂着等于没有")
        self.assertEqual([e for e in fin.poll("CH1") if e["kind"] == "error"], [])

    def test_failures_back_off_instead_of_spinning(self):
        fin = self._finalizer(self._boom)
        time.sleep(3.0)
        self.assertLess(len(self.tries), 40,
                        "3 秒内试了 %d 次，退避没起作用" % len(self.tries))

    def test_recovery_clears_the_alert(self):
        fin = self._finalizer(self._boom)
        self.assertTrue(self._wait_alert(fin))
        fin._store = lambda _d: type("S", (), {"journal": type("J", (), {
            "pending_after": staticmethod(lambda *a, **k: [])})()})()
        deadline = time.perf_counter() + 40.
        while time.perf_counter() < deadline and fin.alert("CH1"):
            time.sleep(0.1)
        self.assertEqual(fin.alert("CH1"), "", "恢复之后告警该撤掉")

    def test_export_one_returns_the_exception_not_a_bool(self):
        """成功返回 None、失败返回异常本身 —— 告警要靠它把原因带出去。"""
        fin = self._finalizer(self._boom)
        fin._export_stop.set()
        journal = fin._store(None).journal
        err = fin._export_one(journal, "m-000", tempfile.mkdtemp(), "CH1")
        self.assertIsInstance(err, OSError)
        ok = fin._export_one(type("J", (), {"export": staticmethod(
            lambda m, d, channel=None: "/tmp/x.xyz")})(), "m-000", ".", "CH1")
        self.assertIsNone(ok, "成功必须返回 None")
