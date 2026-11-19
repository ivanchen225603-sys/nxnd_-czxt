# -*- coding: utf-8 -*-
"""近期运行日志回归：纯内存观测与临时目录，不启动服务或接触设备、业务库。"""
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from app.runtime_events import RuntimeEvents
from app.service import ChannelWorker, ConsoleLogger, EventBus
from app.pointcloud import Frame
from tests import _helpers as H
from tests._observations import ObservationClock


def observed(state, **extra):
    return dict(state=state, fresh=True, degraded=False, stable=state == 2, **extra)


class TestRuntimeEvents(unittest.TestCase):
    def test_webapi_contract_fixture_matches_current_source_events(self):
        events = RuntimeEvents("CH1", "live")
        events.observe(1, observed(1), 1700000000.)
        events.observe(1, observed(2, settling=True), 1700000005.)
        events.record("preparation_wait", "车辆已停稳，等待本车车牌确认", 1, 1700000006.)
        fixture = (Path(__file__).resolve().parents[2] / "CarPointCloudWebApi" / "tests" /
                   "CarPointCloudWebApi.Tests" / "Fixtures" / "load-radar-runtime-python-v1.json")
        expected = json.loads(fixture.read_text(encoding="utf-8"))["channel"]["runtime_events"]
        actual = events.snapshot()
        # 每次运行的进程标识本应不同，仅归一化这一部分后逐字段检查真实源端契约。
        for rows in (expected, actual):
            for row in rows:
                row["event_id"] = row["event_id"].split(":")[-1]
        self.assertEqual(expected, actual)

    def test_empty_initial_idle_and_bad_frames_do_not_invent_vehicle_events(self):
        events = RuntimeEvents("CH1", "live")
        for _ in range(10):
            events.observe(0, observed(0), 1000.)
            events.observe(1, dict(state=2, stable=True, fresh=False, degraded=False), 1001.)
            events.observe(1, dict(state=2, stable=True, fresh=True, degraded=True), 1002.)
        self.assertEqual([], events.snapshot())

    def test_real_state_changes_are_bounded_and_do_not_spam_every_frame(self):
        events = RuntimeEvents("CH2", "live")
        for _ in range(20):
            events.observe(7, observed(1), 1001.)
        for _ in range(20):
            events.observe(7, observed(2, settling=True), 1002.)
        events.observe(7, observed(1), 1003.)
        events.observe(7, observed(2, settling=True), 1004.)
        events.observe(0, observed(0), 1005.)
        events.observe(0, observed(0), 1006.)
        self.assertEqual(["vehicle_detected", "vehicle_stable", "settling", "vehicle_moving",
                          "vehicle_stable", "settling", "vehicle_departed"],
                         [row["kind"] for row in events.snapshot()])
        self.assertTrue(all(row["session"] == 7 for row in events.snapshot()))

    def test_gap_is_not_departure_and_repeated_snapshot_has_no_side_effects(self):
        events = RuntimeEvents("CH2", "live")
        events.observe(3, observed(2), 1000.)
        self.assertIsNotNone(events.gap())
        self.assertIsNone(events.gap())
        for _ in range(10):
            events.observe(0, dict(state=0, fresh=False, degraded=True), 1001.)
        rows = events.snapshot()
        self.assertNotIn("vehicle_departed", [row["kind"] for row in rows])
        for _ in range(10):
            self.assertEqual(rows, events.snapshot())
        events.observe(3, observed(1), 1002.)
        self.assertIn("observation_resumed", [row["kind"] for row in events.snapshot()])
        self.assertNotIn("vehicle_moving", [row["kind"] for row in events.snapshot()])
        events.observe(3, observed(2), 1003.)
        events.gap()
        self.assertEqual(2, sum(row["kind"] == "observation_gap" for row in events.snapshot()))

    def test_event_ids_change_across_restart_and_output_is_exactly_allowlisted(self):
        first, restarted = RuntimeEvents("CH3", "live"), RuntimeEvents("CH3", "live")
        a = first.record("error", "采集异常", 0, 1000.)
        b = restarted.record("error", "采集异常", 0, 1000.)
        self.assertNotEqual(a["event_id"], b["event_id"])
        self.assertEqual({"event_id", "channel", "session", "ts", "kind", "message", "source"}, set(a))
        for i in range(130):
            first.record("preparation_wait", "等待车牌 " + str(i), 1, 1001. + i)
        rows = first.snapshot()
        self.assertEqual(100, len(rows))
        self.assertEqual("等待车牌 30", rows[0]["message"])
        rows[-1]["message"] = "浏览器副本已修改"
        self.assertNotEqual(rows[-1], first.snapshot()[-1])

    def test_forced_session_does_not_reuse_same_frame_stability(self):
        events = RuntimeEvents("CH2", "live")
        events.observe(7, observed(2), 1000.)
        count = len(events.snapshot())
        events.observe(8, observed(2, settling=True), 1001., forced=True)
        events.observe(8, observed(2, settling=True), 1001., forced=True)
        self.assertEqual(["observation_resumed"], [row["kind"] for row in events.snapshot()[count:]])
        events.observe(8, observed(1), 1001.1, forced=True)
        events.observe(8, observed(2), 1003., forced=True)
        self.assertEqual("vehicle_stable", events.snapshot()[-1]["kind"])


class TestChannelRuntimeEvents(unittest.TestCase):
    def worker(self):
        settings, channel, _, _, _ = H.ctx()
        channel = copy.deepcopy(channel)
        channel.id, channel.mode = "CH1", "synthetic"
        directory = tempfile.TemporaryDirectory(prefix="runtime-events-")
        self.addCleanup(directory.cleanup)
        channel.abs_output_dir = channel.abs_baseline_dir = directory.name
        worker = ChannelWorker(channel, EventBus(), settings.plate,
                               save_override=False, plate_mode="null")
        self.addCleanup(worker.source.close)
        self.addCleanup(worker.stop)
        worker.settle_sec = .6
        return worker

    @staticmethod
    def vehicle(worker):
        random = np.random.default_rng(20261029)
        n = worker.stability.vehicle_threshold + 1000
        return Frame(np.column_stack((random.uniform(-1., 1., n), random.uniform(-4., 4., n),
                                      random.uniform(-3.7, -2., n))).astype("f4"))

    def test_tick_emits_arrival_stable_settle_result_and_departure_without_saved_measurement(self):
        worker = self.worker()
        clock = ObservationClock(self)
        frame = self.vehicle(worker)
        empty = Frame(np.column_stack((np.zeros(100), np.zeros(100), np.full(100, -5.2))).astype("f4"))
        clock.tick(worker, empty)
        self.assertEqual([], worker.snapshot()["runtime_events"])
        clock.settle(worker, frame)
        for _ in range(10):
            clock.tick(worker, frame)
        rows = worker.snapshot()["runtime_events"]
        kinds = [row["kind"] for row in rows]
        for kind in ["vehicle_detected", "vehicle_stable", "settling", "measurement_completed"]:
            self.assertIn(kind, kinds)
        self.assertEqual(0, worker.st.saved_count)
        self.assertEqual(1, kinds.count("vehicle_detected"))
        self.assertEqual(1, kinds.count("vehicle_stable"))
        self.assertEqual(rows, worker.snapshot()["runtime_events"])
        clock.tick(worker, empty)
        self.assertEqual("vehicle_departed", worker.snapshot()["runtime_events"][-1]["kind"])

    def test_missing_plate_and_baseline_changes_are_visible_without_results(self):
        worker = self.worker()
        frame = self.vehicle(worker)
        worker._session_id = 4
        worker.st.stability = observed(1)
        status = ["awaiting_plate"]
        worker.integration = SimpleNamespace(preparation_status=lambda channel: status[0],
                                             current_passage=lambda channel: "current-passage")
        for item in ["awaiting_plate", "awaiting_plate", "baseline_required", "baseline_required", "ready"]:
            status[0] = item
            worker._runtime_observe(frame)
        rows = worker.snapshot()["runtime_events"]
        self.assertEqual(["vehicle_detected", "preparation_wait", "preparation_wait", "preparation_ready"],
                         [row["kind"] for row in rows])
        self.assertEqual([], list(worker.history))

    def test_runtime_bus_failure_does_not_change_tick_and_console_has_discrete_message(self):
        worker = self.worker()
        clock = ObservationClock(self)
        frame = self.vehicle(worker)
        original = worker.bus.publish

        def publish(kind, payload):
            if kind == "runtime_event":
                raise OSError("仅测试日志不可用")
            return original(kind, payload)

        with patch.object(worker.bus, "publish", side_effect=publish):
            clock.settle(worker, frame)
            for _ in range(10):
                self.assertTrue(clock.tick(worker, frame))
        self.assertEqual(1, len(worker.history))
        line = ConsoleLogger.format(dict(kind="runtime_event", channel="CH1",
                                         event=dict(session=3, message="车辆已停稳")))
        self.assertIn("车辆已停稳", line)
        self.assertIn("runtime_event", ConsoleLogger.KINDS)

    def test_late_finalizer_completion_and_error_do_not_attach_to_current_vehicle(self):
        worker = self.worker()
        worker._session_id = 9
        worker._current_result_token = "current-car-token"
        identity = SimpleNamespace(measurement_id="old-measurement")
        body = dict(origin_token="old-car-token", session=7, ts=1001.5, measurement_id="old-measurement")
        records = [dict(kind="durable", result_json=json.dumps(body), identity=identity,
                        archived=False, managed=False),
                   dict(kind="error", measurement_id="old-measurement", error="私有路径不得进入运行日志")]
        worker._finalizer = SimpleNamespace(poll=lambda channel: records)
        with patch("time.time", return_value=2001.5):
            worker._drain_finalizer()
        rows = worker._runtime_events.snapshot()
        self.assertEqual([7, 0], [row["session"] for row in rows])
        self.assertEqual(2001.5, rows[0]["ts"])
        self.assertNotEqual(body["ts"], rows[0]["ts"])
        self.assertIn("历史", rows[0]["message"])
        self.assertIn("历史", rows[1]["message"])
        self.assertNotIn("私有路径", json.dumps(rows, ensure_ascii=False))
        self.assertEqual(9, worker._session_id)
        worker._finalizer = None

    def test_runtime_snapshot_failure_keeps_channel_snapshot_available(self):
        worker = self.worker()
        with patch.object(worker._runtime_events, "snapshot", side_effect=OSError("仅测试日志不可读")):
            snapshot = worker.snapshot()
        self.assertEqual([], snapshot["runtime_events"])
        self.assertEqual("CH1", snapshot["id"])
