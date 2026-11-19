"""离线编排替身测试：真实任务SQLite；几何/识牌/上传均为mock，不是实录回放。"""
import json
from pathlib import Path
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from app.deferred_measurements import DeferredMeasurements, pack_prepared, unpack_prepared
from app.load_radar_bridge import ResultReservation
from app.load_radar_transport import BridgeConfig, ChannelConfig, HttpError
from app.pending_measurements import CompletionBusy, PendingTaskStore
from app.result_finalizer import PreparedResult, result_bytes


CHANNELS = ("CH1", "CH2", "CH3", "CH4")
TEMP_ROOT = Path(tempfile.gettempdir()) / "load-radar-async-tests"


class MemoryOutbox:
    """仅模拟既有outbox查询；不会HTTP、写业务库或创建完整运行服务。"""
    def __init__(self):
        self.rows = {}
        self.archives = {}

    def get(self, measurement_id):
        return self.rows.get(measurement_id)

    def save(self, identity, prepared):
        self.rows[identity.measurement_id] = SimpleNamespace(
            payload=json.loads(prepared.payload_json), asset=prepared.asset)
        self.archives[identity.measurement_id] = prepared.archive


class MockFrozenDraft:
    """仅验证编排时序：输入为不可变JSON字节，不承担真实点云冻结验证。"""
    def __init__(self, data):
        self.data = data
        self.encode = Mock(return_value=data)

    def byte_size(self):
        return len(self.data)


class TestDeferredMeasurements(unittest.TestCase):
    def setUp(self):
        TEMP_ROOT.mkdir(parents=True, exist_ok=True)
        self.tmp = tempfile.TemporaryDirectory(prefix="deferred-tests-", dir=TEMP_ROOT)
        self.addCleanup(self.tmp.cleanup)
        self.storage = Path(self.tmp.name) / "store"
        config = BridgeConfig(node_id="mock-node", base_url="http://127.0.0.1:1",
            agent_key="offline-mock-not-a-real-secret-000", storage_dir=self.storage,
            algorithm_version="mock-only", channels={c: ChannelConfig("cal-" + c) for c in CHANNELS},
            enabled=False, retry_sec=1., timeout_sec=1.)
        self.bridge = SimpleNamespace(config=config, outbox=MemoryOutbox(),
            capture_identity=Mock(side_effect=self.capture_identity),
            prepare_frozen=Mock(return_value=("ready", result_bytes({"mock": "original-proof"}))),
            frozen_preparation=Mock(return_value=("original-baseline-snapshot", "mock-plate")),
            _prepare_channel=Mock())
        self.current = {channel: "passage-old-" + channel for channel in CHANNELS}
        self.finalizer = SimpleNamespace(submit_prepared=Mock(return_value=True),
                                         restore_durable=Mock(return_value=True))
        self.engine = self.open_engine()
        self.freeze = self.patch("FrozenDetection.capture", side_effect=self.freeze_input)
        self.inspect = self.patch("inspect_frozen", side_effect=lambda data: json.loads(data)["metadata"])
        self.calculate = self.patch("calculate_frozen", side_effect=self.calculate_input)
        self.prepare = self.patch("prepare_result", side_effect=self.prepare_output)

    def patch(self, name, **kw):
        context = patch("app.deferred_measurements." + name, **kw)
        value = context.start()
        self.addCleanup(context.stop)
        return value

    def open_engine(self):
        store = PendingTaskStore(self.storage, CHANNELS)
        self.addCleanup(store.close)
        return DeferredMeasurements(self.bridge, self.finalizer, CHANNELS, store=store)

    def capture_identity(self, channel, token):
        identity = ResultReservation("m-" + token, channel, self.current[channel], token)
        start = result_bytes({"nodeId": self.bridge.config.node_id, "channelId": channel,
                              "passageId": identity.passage_id, "source": "offline-mock"})
        return identity, start

    @staticmethod
    def freeze_input(worker, frame, stab, identity, start):
        return MockFrozenDraft(result_bytes({"metadata": {"measured_at": "2026-10-22T03:00:00Z",
            "measurementId": identity.measurement_id, "passageId": identity.passage_id,
            "channelId": identity.channel, "mock": True}, "inline_frame": frame}))

    @staticmethod
    def calculate_input(data, identity, snapshot, plate, config, *, exit_frames=()):
        return SimpleNamespace(identity=identity, result_json=result_bytes({"mock": True,
            "plate": plate, "measurement_id": identity.measurement_id, "volume": 12.5}))

    @staticmethod
    def prepare_output(pending, bridge):
        identity = pending.identity
        return PreparedResult(result_bytes({"measurementId": identity.measurement_id,
            "channelId": identity.channel, "passageId": identity.passage_id,
            "nodeId": bridge.config.node_id, "mock": True, "volumeM3": 12.5,
            "measuredAt": "2026-10-22T03:00:00Z", "sourceMode": "live", "verdict": "loaded"}), None, None)

    @staticmethod
    def worker(channel="CH1"):
        return SimpleNamespace(ch=SimpleNamespace(id=channel), _detected_this_session=0,
            _current_result_token=None, _last_save_name=None, bus=SimpleNamespace(publish=Mock()))

    def capture(self, channel="CH1", frame=None):
        worker = self.worker(channel)
        frame = frame or {"xyz": [[1., 2., 3.]], "intensity": [7], "sequence": 41}
        measurement_id = self.engine.capture(worker, frame, SimpleNamespace())
        return worker, measurement_id

    def captured(self, channel="CH1"):
        worker, measurement_id = self.capture(channel)
        self.assertTrue(self.engine.persist_once(channel))
        return worker, measurement_id

    def ready_again(self, channel, measurement_id):
        self.engine.store.retry(channel, measurement_id, "test-clock-only", 0, now=0)

    def test_runtime_log_only_records_new_capture_with_original_session_and_time(self):
        class TimedFrame(dict):
            ts = 1001.5
        worker = self.worker()
        worker._session_id = 7
        worker._runtime_record = Mock()
        frame = TimedFrame(xyz=[[1., 2., 3.]])
        measurement_id = self.engine.capture(worker, frame, SimpleNamespace())
        worker._runtime_record.assert_called_once_with("measurement_queued",
            "本车观测已进入留样队列，等待持久化与后台识牌计算",
            ts=1001.5, session=7, identity=measurement_id)
        worker._session_id = 8
        self.assertEqual(measurement_id, self.engine.capture(worker, frame, SimpleNamespace()))
        self.assertEqual(1, worker._runtime_record.call_count)
        self.assertTrue(self.engine.persist_once("CH1"))

    def test_runtime_log_error_does_not_change_capture_or_persistence(self):
        worker = self.worker()
        worker._runtime_record = Mock(side_effect=OSError("仅测试日志异常"))
        measurement_id = self.engine.capture(worker, {"xyz": [[1., 2., 3.]]}, SimpleNamespace())
        self.assertEqual(1, self.engine.snapshot("CH1")["ingress_jobs"])
        self.assertTrue(self.engine.persist_once("CH1"))
        self.assertIsNotNone(self.engine.store.get("CH1", measurement_id))

    def test_capture_stages_original_bytes_then_persist_once_makes_durable(self):
        frame = {"xyz": [[1., 2., 3.]], "intensity": [7], "sequence": 41}
        worker, measurement_id = self.capture(frame=frame)
        self.assertIsNone(self.engine.store.get("CH1", measurement_id))
        self.assertEqual((1, "persisting"), (self.engine.snapshot("CH1")["ingress_jobs"],
                                              self.engine.snapshot("CH1")["last_capture"]))
        self.assertEqual(1, worker._detected_this_session)
        draft = self.engine._ingress["CH1"][0][2]
        draft.encode.assert_not_called()
        frame["xyz"][0][0] = 999.
        self.assertTrue(self.engine.persist_once("CH1"))
        draft.encode.assert_called_once_with()
        row = self.engine.store.get("CH1", measurement_id)
        self.assertEqual(1., json.loads(row.input_bytes)["inline_frame"]["xyz"][0][0])
        self.assertEqual(0, self.engine.snapshot("CH1")["ingress_jobs"])
        self.assertEqual("captured", self.engine.snapshot("CH1")["last_capture"])
        self.assertFalse(self.engine.persist_once("CH1"))
        self.assertTrue(self.engine.close())
        self.engine = self.open_engine()
        self.assertEqual(row, self.engine.store.get("CH1", measurement_id))

    def test_failed_persistence_keeps_original_staged_item_for_retry(self):
        _, measurement_id = self.capture()
        original = self.engine._ingress["CH1"][0]
        with patch.object(self.engine.store, "put", side_effect=OSError("mock disk failure")):
            with self.assertRaises(OSError):
                self.engine.persist_once("CH1")
        retained = self.engine._ingress["CH1"][0]
        self.assertEqual(original[:2], retained[:2])
        self.assertEqual(original[2].data, retained[2])
        self.assertFalse(self.engine.close())
        self.assertTrue(self.engine.persist_once("CH1"))
        self.assertEqual(original[2].data, self.engine.store.get("CH1", measurement_id).input_bytes)
        original[2].encode.assert_called_once_with()
        self.freeze.assert_called_once()

    def test_encoding_failure_keeps_same_immutable_draft_for_retry(self):
        _, measurement_id = self.capture()
        original = self.engine._ingress["CH1"][0]
        draft = original[2]
        draft.encode.side_effect = [OSError("mock encode failure"), draft.data]
        with self.assertRaises(OSError):
            self.engine.persist_once("CH1")
        self.assertEqual(original, self.engine._ingress["CH1"][0])
        self.assertIsNone(self.engine.store.get("CH1", measurement_id))
        self.engine.persist_once("CH1")
        self.assertEqual(draft.data, self.engine.store.get("CH1", measurement_id).input_bytes)
        self.assertEqual(2, draft.encode.call_count)
        self.freeze.assert_called_once()

    def test_same_passage_in_one_process_is_not_recaptured(self):
        _, measurement_id = self.capture()
        worker, second_id = self.capture(frame={"xyz": [[9., 9., 9.]]})
        self.assertEqual(measurement_id, second_id)
        self.assertEqual(1, worker._detected_this_session)
        self.freeze.assert_called_once()
        self.assertEqual(1, self.engine.snapshot("CH1")["ingress_jobs"])
        self.engine.persist_once("CH1")

    def test_capture_rejects_manual_reservation_before_same_passage_shortcut(self):
        _, original_id = self.captured()
        original = self.engine.store.get("CH1", original_id)
        command = result_bytes({"requestId": "manual-r1", "channelId": "CH1",
            "passageId": original.identity.passage_id, "kind": "remeasure"})
        identity = ResultReservation("manual-m1", "CH1", original.identity.passage_id,
                                     "manual-token", command)
        self.bridge.capture_identity.side_effect = None
        self.bridge.capture_identity.return_value = (identity, original.start_json)
        with self.assertRaises(ValueError):
            self.capture()
        self.assertEqual(original, self.engine.store.get("CH1", original_id))
        self.assertEqual(0, self.engine.snapshot("CH1")["ingress_jobs"])

    def test_awaiting_plate_then_departure_and_new_car_keep_original_identity(self):
        _, old_id = self.captured()
        original = self.engine.store.get("CH1", old_id)
        self.bridge.prepare_frozen.side_effect = [("awaiting_plate", None),
            ("ready", result_bytes({"mock": "late-original-proof"}))]
        self.engine.process_once("CH1")
        waiting = self.engine.store.get("CH1", old_id)
        self.assertEqual(("awaiting_preparation", "awaiting_plate"), (waiting.state, waiting.error))
        self.calculate.assert_not_called()
        self.current["CH1"] = "passage-new-CH1"
        _, new_id = self.captured()
        self.ready_again("CH1", old_id)
        self.engine.process_once("CH1")
        identity, start, measured_at = self.bridge.prepare_frozen.call_args.args
        self.assertEqual((original.identity, original.start_json, "2026-10-22T03:00:00Z"),
                         (identity, start, measured_at))
        self.assertEqual(original.input_bytes, self.calculate.call_args.args[0])
        self.assertEqual(original.identity, self.calculate.call_args.args[1])
        self.assertEqual(original.identity, self.finalizer.submit_prepared.call_args.args[0])
        self.assertNotEqual(old_id, new_id)
        self.assertEqual("awaiting_preparation", self.engine.store.get("CH1", new_id).state)

    def test_repeated_awaiting_reuses_metadata_without_decoding_large_input(self):
        _, measurement_id = self.captured()
        self.bridge.prepare_frozen.return_value = ("awaiting_plate", None)
        for _ in range(3):
            self.ready_again("CH1", measurement_id)
            self.engine.process_once("CH1")
        self.inspect.assert_called_once()
        self.assertEqual(3, self.bridge.prepare_frozen.call_count)
        self.calculate.assert_not_called()

    def test_preparation_proof_survives_calculation_failure_and_restart(self):
        _, measurement_id = self.captured()
        self.calculate.side_effect = RuntimeError("mock calculation interrupted")
        self.engine.process_once("CH1")
        original = self.engine.store.get("CH1", measurement_id)
        self.assertEqual("ready_to_compute", original.state)
        self.assertIsNotNone(original.preparation_bytes)
        self.assertIsNone(original.prepared_bytes)
        self.assertTrue(self.engine.close())
        self.engine = self.open_engine()
        self.bridge.prepare_frozen.reset_mock(side_effect=True)
        self.bridge.prepare_frozen.side_effect = AssertionError("不应重新识牌")
        self.calculate.side_effect = self.calculate_input
        self.ready_again("CH1", measurement_id)
        self.engine.process_once("CH1")
        self.bridge.prepare_frozen.assert_not_called()
        self.assertEqual(original.preparation_bytes, self.bridge.frozen_preparation.call_args.args[3])
        self.assertEqual("prepared", self.engine.store.get("CH1", measurement_id).state)

    def test_prepared_bytes_survive_restart_and_are_submitted_without_recalculation(self):
        _, measurement_id = self.captured()
        self.finalizer.submit_prepared.return_value = False
        self.engine.process_once("CH1")
        original = self.engine.store.get("CH1", measurement_id)
        expected_result, expected_prepared = unpack_prepared(original.prepared_bytes)
        self.assertTrue(self.engine.close())
        self.engine = self.open_engine()
        for mocked in (self.inspect, self.calculate, self.prepare, self.bridge.prepare_frozen,
                       self.bridge.frozen_preparation):
            mocked.reset_mock()
            mocked.side_effect = AssertionError("已有最终原件不能再计算或查询")
        self.finalizer.submit_prepared.reset_mock()
        self.finalizer.submit_prepared.return_value = True
        self.ready_again("CH1", measurement_id)
        self.engine.process_once("CH1")
        self.finalizer.submit_prepared.assert_called_once_with(original.identity, expected_result, expected_prepared)
        for mocked in (self.inspect, self.calculate, self.prepare, self.bridge.prepare_frozen,
                       self.bridge.frozen_preparation):
            mocked.assert_not_called()
        self.assertEqual(original.prepared_bytes, self.engine.store.get("CH1", measurement_id).prepared_bytes)

    def test_finalizer_full_waits_and_never_recalculates(self):
        _, measurement_id = self.captured()
        self.finalizer.submit_prepared.side_effect = [False, False, True]
        for _ in range(3):
            self.ready_again("CH1", measurement_id)
            self.engine.process_once("CH1")
        self.calculate.assert_called_once()
        self.prepare.assert_called_once()
        self.assertEqual(3, self.finalizer.submit_prepared.call_count)
        calls = self.finalizer.submit_prepared.call_args_list
        self.assertEqual(calls[0], calls[1])
        self.assertEqual(calls[1], calls[2])
        self.ready_again("CH1", measurement_id)
        self.engine.process_once("CH1")
        self.assertEqual(3, self.finalizer.submit_prepared.call_count)
        self.assertEqual("prepared", self.engine.store.get("CH1", measurement_id).state)

    def test_matching_outbox_commits_only_after_original_handoff(self):
        _, measurement_id = self.captured()
        self.engine.process_once("CH1")
        identity, _, prepared = self.finalizer.submit_prepared.call_args.args
        self.bridge.outbox.save(identity, prepared)
        self.ready_again("CH1", measurement_id)
        self.engine.process_once("CH1")
        self.assertEqual("committed", self.engine.store.get("CH1", measurement_id).state)
        self.finalizer.restore_durable.assert_not_called()
        self.assertFalse(self.engine.process_once("CH1"))

    def test_outbox_recovery_waits_for_restore_capacity_without_recalculation(self):
        _, measurement_id = self.captured()
        self.engine.process_once("CH1")
        identity, result_json, prepared = self.finalizer.submit_prepared.call_args.args
        self.bridge.outbox.save(identity, prepared)
        self.assertTrue(self.engine.close())
        self.engine = self.open_engine()
        self.finalizer.restore_durable.side_effect = [False, True]
        for expected in ("prepared", "committed"):
            self.ready_again("CH1", measurement_id)
            self.engine.process_once("CH1")
            self.assertEqual(expected, self.engine.store.get("CH1", measurement_id).state)
        self.finalizer.restore_durable.assert_called_with(identity, result_json, prepared)
        self.calculate.assert_called_once()
        self.prepare.assert_called_once()

    def test_existing_outbox_without_frozen_result_blocks_before_compute(self):
        _, measurement_id = self.captured()
        self.bridge.outbox.rows[measurement_id] = SimpleNamespace(payload={}, asset=None)
        self.engine.process_once("CH1")
        self.assertEqual("blocked", self.engine.store.get("CH1", measurement_id).state)
        self.calculate.assert_not_called()
        self.bridge.prepare_frozen.assert_not_called()
        self.finalizer.submit_prepared.assert_not_called()

    def test_existing_outbox_changed_payload_or_asset_blocks_original_id(self):
        for channel, field in (("CH1", "payload"), ("CH2", "asset")):
            with self.subTest(field=field):
                _, measurement_id = self.captured(channel)
                self.engine.process_once(channel)
                identity, _, prepared = self.finalizer.submit_prepared.call_args.args
                self.bridge.outbox.save(identity, prepared)
                outbox_row = self.bridge.outbox.rows[measurement_id]
                if field == "payload":
                    outbox_row.payload["volumeM3"] = 99.
                else:
                    outbox_row.asset = b"other-bytes"
                before = self.engine.store.get(channel, measurement_id)
                self.ready_again(channel, measurement_id)
                self.engine.process_once(channel)
                after = self.engine.store.get(channel, measurement_id)
                self.assertEqual("blocked", after.state)
                self.assertEqual(before.prepared_bytes, after.prepared_bytes)
                self.assertEqual(before.identity, after.identity)

    def test_http_permanent_blocks_and_transient_retries_without_computing(self):
        for channel, status, expected in (("CH1", 401, "blocked"), ("CH2", 422, "blocked"),
                ("CH3", 503, "awaiting_preparation"), ("CH4", 429, "awaiting_preparation")):
            with self.subTest(status=status):
                _, measurement_id = self.captured(channel)
                self.bridge.prepare_frozen.side_effect = HttpError(status)
                self.engine.process_once(channel)
                row = self.engine.store.get(channel, measurement_id)
                self.assertEqual(expected, row.state)
                self.assertEqual(0 if expected == "blocked" else 1, row.attempts)
        self.calculate.assert_not_called()
        self.finalizer.submit_prepared.assert_not_called()

    def test_plate_unavailable_is_terminal_without_placeholder_result(self):
        _, measurement_id = self.captured()
        self.bridge.prepare_frozen.return_value = ("plate_unavailable", None)
        self.engine.process_once("CH1")
        row = self.engine.store.get("CH1", measurement_id)
        self.assertEqual(("blocked", "plate_unavailable"), (row.state, row.error))
        self.assertIsNone(row.prepared_bytes)
        self.calculate.assert_not_called()
        self.finalizer.submit_prepared.assert_not_called()

    def test_one_channel_blocked_in_preparation_does_not_delay_other_three(self):
        ids = {channel: self.captured(channel)[1] for channel in CHANNELS}
        entered, release = threading.Event(), threading.Event()
        done = {channel: threading.Event() for channel in CHANNELS}
        errors = []
        def prepare(identity, start, measured_at):
            if identity.channel == "CH1":
                entered.set()
                if not release.wait(5):
                    raise RuntimeError("测试同步超时")
            return "ready", result_bytes({"mock": identity.channel})
        def run(channel):
            try:
                self.engine.process_once(channel)
            except Exception as error:
                errors.append(error)
            finally:
                done[channel].set()
        self.bridge.prepare_frozen.side_effect = prepare
        threads = [threading.Thread(target=run, args=(channel,)) for channel in CHANNELS]
        try:
            threads[0].start()
            self.assertTrue(entered.wait(2))
            for thread in threads[1:]:
                thread.start()
            for channel in CHANNELS[1:]:
                self.assertTrue(done[channel].wait(2), channel + "不应等待CH1")
                self.assertEqual("prepared", self.engine.store.get(channel, ids[channel]).state)
            self.assertFalse(done["CH1"].is_set())
        finally:
            release.set()
            for thread in threads:
                if thread.ident is not None:
                    thread.join(3)
        self.assertEqual([], errors)
        self.assertTrue(all(event.is_set() for event in done.values()))

    def test_prepared_codec_roundtrip_keeps_exact_json_bytes_with_no_archive(self):
        result = b'{"mock":true,"volume":12.500}'
        payload = b'{"measurementId":"m-1","channelId":"CH1","passageId":"p-1"}'
        prepared = PreparedResult(payload, b"immutable-asset", None)
        encoded = pack_prepared(result, prepared)
        self.assertEqual((result, prepared), unpack_prepared(encoded))
        with self.assertRaises(ValueError):
            unpack_prepared(encoded + b" ")

    def test_durable_capture_is_not_published_before_database_commit(self):
        _, measurement_id = self.capture()
        self.assertIsNone(self.engine.durable_capture("CH1"))
        with patch.object(self.engine.store, "put", side_effect=OSError("模拟落盘失败")):
            with self.assertRaises(OSError):
                self.engine.persist_once("CH1")
        self.assertIsNone(self.engine.durable_capture("CH1"))
        self.engine.persist_once("CH1")
        self.assertEqual(measurement_id, self.engine.durable_capture("CH1").measurement_id)
        self.engine._refresh("CH1")
        self.assertEqual("captured", self.engine.snapshot("CH1")["last_capture"])

    def test_completion_budget_wait_does_not_run_geometry_again(self):
        _, measurement_id = self.captured()
        with patch.object(self.engine.store, "freeze_preparation", side_effect=CompletionBusy()):
            self.engine.process_once("CH1")
            self.ready_again("CH1", measurement_id)
            self.engine.process_once("CH1")
        self.calculate.assert_not_called()
        row = self.engine.store.get("CH1", measurement_id)
        self.assertEqual("awaiting_preparation", row.state)
        self.assertIn("完成预算", row.error)
        self.assertIsNone(row.prepared_bytes)

    def test_existing_payload_boolean_is_not_the_same_as_integer_bytes(self):
        _, measurement_id = self.captured()
        self.engine.process_once("CH1")
        row = self.engine.store.get("CH1", measurement_id)
        _, prepared = unpack_prepared(row.prepared_bytes)
        self.bridge.outbox.save(row.identity, prepared)
        self.bridge.outbox.rows[measurement_id].payload["mock"] = 1
        self.ready_again("CH1", measurement_id)
        self.engine.process_once("CH1")
        self.assertEqual("blocked", self.engine.store.get("CH1", measurement_id).state)
        self.finalizer.restore_durable.assert_not_called()


    # ---- 处理线程唤醒：新留样不等固定 2 秒轮询，WebApi 查询保持原节拍 ----

    def run_processor(self, channel="CH1"):
        thread = threading.Thread(target=self.engine._processor, args=(channel,), daemon=True)

        def stop():
            self.engine._halt.set()
            self.engine._process_wake[channel].set()
            thread.join(3.)
        self.addCleanup(stop)
        thread.start()
        return thread

    def test_persisted_capture_wakes_the_processor(self):
        self.assertFalse(self.engine._process_wake["CH1"].is_set())
        self.captured("CH1")
        self.assertTrue(self.engine._process_wake["CH1"].is_set())
        self.assertFalse(self.engine._process_wake["CH2"].is_set())

    def test_idle_wait_follows_the_next_due_task_but_never_busy_waits(self):
        import time
        _, measurement_id = self.captured("CH1")
        self.engine.store.retry("CH1", measurement_id, "short-retry", .3)
        wait = self.engine._idle_wait("CH1", 2., time.monotonic() + 5.)
        self.assertLessEqual(wait, .31)
        self.assertGreaterEqual(wait, .05)
        # 没有任务的车道按原节拍等；已到期却取不到时也至少等 0.05 秒。
        self.assertAlmostEqual(self.engine._idle_wait("CH2", 2., time.monotonic() + 5.), 2., delta=.01)
        self.engine.store.retry("CH1", measurement_id, "due-now", 0, now=0)
        self.assertEqual(self.engine._idle_wait("CH1", 2., time.monotonic() + 5.), .05)

    def test_processor_starts_a_new_capture_without_waiting_for_the_poll(self):
        import time
        self.engine.recalculation = SimpleNamespace(process_once=Mock(return_value=False))
        self.run_processor("CH1")
        time.sleep(.2)                      # 首轮已执行完，线程在 1 秒的空闲等待中
        prepare_calls = self.bridge._prepare_channel.call_count
        started = time.monotonic()
        self.captured("CH1")
        deadline = started + .6
        while not self.finalizer.submit_prepared.called and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertTrue(self.finalizer.submit_prepared.called, "新留样应在空闲等待结束前被处理")
        # 本地任务提前处理，不额外触发到车准备（WebApi）请求。
        self.assertEqual(self.bridge._prepare_channel.call_count, prepare_calls)

    def test_processor_error_falls_back_to_the_fixed_interval(self):
        import time
        self.engine.recalculation = SimpleNamespace(process_once=Mock(return_value=False))
        self.engine.process_once = Mock(side_effect=RuntimeError("boom"))
        self.run_processor("CH1")
        time.sleep(.5)
        self.engine._process_wake["CH1"].set()   # 唤醒只会多跑一次，不会忙等
        time.sleep(.3)
        self.assertLessEqual(self.engine.process_once.call_count, 2)
        self.assertEqual(self.engine._stats["CH1"]["processor_error"], "RuntimeError")


if __name__ == "__main__":
    unittest.main()
