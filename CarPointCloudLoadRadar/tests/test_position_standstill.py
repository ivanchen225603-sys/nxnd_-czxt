# -*- coding: utf-8 -*-
"""CH4 位置停稳触发：独立计时器、检测器只读位姿、bridge 每车次一次的后台通知。"""
import dataclasses
import tempfile
import threading
import time
import unittest
from datetime import datetime

import numpy as np

from app.detect import StabilityDetector, StabilityResult
from app.load_radar_bridge import LoadRadarBridge
from app.load_radar_transport import BridgeConfig, HttpError
from app.pointcloud import Frame
from app.position_standstill import PositionStandstill


def pose(x=0.0, y=0.0, heading=0.0, radius=5.0):
    return np.array([x, y]), heading, radius


class TestPositionStandstillTimer(unittest.TestCase):
    def setUp(self):
        self.t = PositionStandstill(drift_m=0.03, min_sec=2.0)

    def feed(self, seconds, step=0.1, key="p1", **kw):
        out, ts = False, self.start
        while ts <= self.start + seconds + 1e-9:
            out = self.t.update(key, ts, pose(**kw), True)
            ts += step
        self.start = ts
        return out

    def test_still_for_two_seconds_triggers(self):
        self.start = 100.0
        self.assertFalse(self.feed(1.5))
        self.assertTrue(self.feed(0.6))

    def test_small_jitter_inside_drift_keeps_counting(self):
        ts, still = 100.0, False
        for i in range(25):
            still = self.t.update("p1", ts, pose(x=0.01 * (-1) ** i), True)
            ts += 0.1
        self.assertTrue(still)

    def test_slow_creep_never_triggers_even_when_each_frame_is_tiny(self):
        # 原判稳的锚点会被形变判据反复重置；这里起点只随自身位移重置，慢速蠕行累计后必然超限
        ts, still_seen = 100.0, False
        for i in range(60):
            still_seen |= self.t.update("p1", ts, pose(x=0.02 * i * 0.1), True)  # 0.02 m/s
            ts += 0.1
        self.assertFalse(still_seen)

    def test_heading_change_is_converted_to_metres_with_radius(self):
        ts = 100.0
        self.t.update("p1", ts, pose(heading=0.0, radius=5.0), True)
        # 0.01 rad × 5 m = 5 cm > 3 cm：重新起算
        for _ in range(25):
            ts += 0.1
            still = self.t.update("p1", ts, pose(heading=0.01, radius=5.0), True)
            if ts < 102.0:
                self.assertFalse(still)
        self.assertTrue(still)  # 新起点后又稳定 2 秒
        # 0.004 rad × 5 m = 2 cm < 3 cm：不算挪动
        t2 = PositionStandstill(0.03, 2.0)
        t2.update("p1", 0.0, pose(heading=0.0), True)
        self.assertTrue(t2.update("p1", 2.0, pose(heading=0.004), True))

    def test_unusable_frame_or_new_passage_restarts(self):
        self.t.update("p1", 0.0, pose(), True)
        self.t.update("p1", 1.5, pose(), False)
        self.assertFalse(self.t.update("p1", 2.5, pose(), True))
        self.assertFalse(self.t.update("p2", 5.0, pose(), True))
        self.assertTrue(self.t.update("p2", 7.0, pose(), True))
        self.assertFalse(self.t.update(None, 8.0, pose(), True))

    def test_unknown_and_discontinuous_frames_break_the_certified_period(self):
        for seq, ts, xy in ((12, 10.4, 0), (11, 11.1, 0), (11, 9.5, 0), (11, 10.1, float("nan"))):
            timer = PositionStandstill(.03, .2, .75)
            for i in range(3):
                timer.update("car", 9.8 + .1 * i, pose(), True, 8 + i)
            self.assertFalse(timer.update("car", ts, pose(x=xy), True, seq))
            self.assertIn(timer.break_reason, ("observation_unavailable", "observation_gap"))


class TestDetectorPose(unittest.TestCase):
    def test_stability_result_shape_is_unchanged(self):
        # StabilityResult 会被 asdict 持久化进冻结任务；位姿不得加进这里
        names = [f.name for f in dataclasses.fields(StabilityResult)]
        self.assertNotIn("position", " ".join(names))
        self.assertEqual(names, [
            "state", "deviation", "stable", "n_points", "elapsed", "n_vehicle_points", "degraded",
            "fresh", "stationary_sec", "speed_mps", "drift_m", "shape_speed_mps", "yaw_speed_mps",
            "motion_observable", "motion_pending", "instant_speed_mps", "jitter_confirmed",
            "triggers", "reason"])

    def test_last_pose_follows_each_detect(self):
        det = StabilityDetector(vehicle_threshold=10, ground_z=-5.0)
        rng = np.random.default_rng(0)
        box = np.column_stack([rng.uniform(-1.2, 1.2, 4000), rng.uniform(-5, 5, 4000),
                               rng.uniform(-4, -2, 4000)]).astype(np.float32)
        self.assertIsNone(det.last_pose())
        det.detect(Frame(box, None, 1, time.time(), 0, 900))
        xy, heading, radius = det.last_pose()
        self.assertEqual(xy.shape, (2,))
        self.assertGreater(radius, 1.0)
        self.assertIsNotNone(heading)
        empty = np.column_stack([np.zeros(50), np.zeros(50), np.full(50, -5.2)]).astype(np.float32)
        det.detect(Frame(empty, None, 2, time.time(), 0, 900))
        self.assertIsNone(det.last_pose())


class StandstillClient:
    def __init__(self):
        self.calls = []
        self.departures = []
        self.states = []
        self.failures = 0
        self.event = threading.Event()

    def report_standstill(self, payload):
        self.calls.append(payload)
        self.event.set()
        if self.failures:
            self.failures -= 1
            raise HttpError(None)
        return {"status": "dispatched"}

    def report_departure(self, payload):
        self.departures.append(payload)
        return {"status": "closed"}

    def report_standstill_state(self, payload):
        self.states.append(dict(payload))
        return {"status": "accepted"}


class TestBridgeStandstillReport(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        config = BridgeConfig(node_id="jt-test", base_url="http://127.0.0.1:1", agent_key="test-key-" * 5,
                              storage_dir=self.tmp.name, algorithm_version="test-v1",
                              channels={"CH4": {"calibration_id": "cal-4", "standstill_observation_age_bound_ms": 100}},
                              enabled=True, timeout_sec=1, retry_sec=1)
        self.client = StandstillClient()
        self.bridge = LoadRadarBridge(config, self.client)
        self.addCleanup(lambda: self.bridge.close())

    def enter(self):
        for _ in range(3):
            self.bridge.observe("CH4", False, True)
            self.bridge._idle["CH4"] = (3, time.monotonic() - 1)
        self.bridge.observe("CH4", True, True)
        return self.bridge.current_passage("CH4")

    def wait_calls(self, n, timeout=5.0):
        deadline = time.monotonic() + timeout
        while len(self.client.calls) < n and time.monotonic() < deadline:
            self.bridge.observe("CH4", True, True)   # 维持车次新鲜，避免断流隔离
            time.sleep(0.05)
        return len(self.client.calls)

    def test_waits_for_registration_then_reports_once(self):
        passage = self.enter()
        self.assertTrue(passage)
        # 未登记：不通知
        self.assertFalse(self.bridge.report_standstill("CH4", passage))
        original_stopped_at = self.bridge._standstill_observed["CH4"][1]
        # 登记回执到达（任何状态都算，含等车牌）后，下一次调用即通知
        self.bridge._prepared["CH4"] = (passage, "awaiting_plate", None, time.monotonic())
        self.assertTrue(self.bridge.report_standstill("CH4", passage))
        self.assertEqual(self.wait_calls(1), 1)
        self.assertEqual(self.client.calls[0], {"nodeId": "jt-test", "channelId": "CH4", "passageId": passage,
                                                "stoppedAt": original_stopped_at})
        self.assertIsNotNone(datetime.fromisoformat(original_stopped_at).tzinfo)
        # 收到答复后，同一车次不再通知
        time.sleep(0.2)
        self.assertFalse(self.bridge.report_standstill("CH4", passage))
        time.sleep(0.2)
        self.assertEqual(len(self.client.calls), 1)

    def test_network_failure_is_retried_while_passage_is_current(self):
        passage = self.enter()
        self.bridge._prepared["CH4"] = (passage, "ready", None, time.monotonic())
        self.client.failures = 1
        self.assertTrue(self.bridge.report_standstill("CH4", passage))
        self.assertEqual(self.wait_calls(2), 2)
        self.assertEqual(self.client.calls[0], self.client.calls[1])
        time.sleep(0.2)
        self.assertFalse(self.bridge.report_standstill("CH4", passage))

    def test_other_passage_is_not_reported(self):
        passage = self.enter()
        self.bridge._prepared["CH4"] = (passage, "ready", None, time.monotonic())
        self.assertFalse(self.bridge.report_standstill("CH4", "p-other"))
        self.assertFalse(self.bridge.report_standstill("CH4", None))

    def _static_scenario(self, broken_at, *, gap=False):
        # 生产 ChannelWorker 的逐帧入口，唯一替身是位置与检测结果。
        from types import SimpleNamespace
        from app.application.channel_worker import ChannelWorker
        self.bridge.configure_channel("CH4", "a" * 64, "live")
        passage = self.enter()
        self.bridge._prepared["CH4"] = (passage, "awaiting_plate", None, time.monotonic())
        position = [pose()]
        worker = SimpleNamespace(ch=SimpleNamespace(id="CH4"), integration=self.bridge,
                                 _position_still=PositionStandstill(.03, 2., .75),
                                 stability=SimpleNamespace(last_pose=lambda: position[0]))
        valid = SimpleNamespace(fresh=True, degraded=False)
        origin = int(time.time()) - 20
        for i in range(-20, 120):
            if i == broken_at * 10:
                if gap:
                    # 观察序列缺帧；随后原位停稳也不能续上旧段。
                    continue
                position[0] = pose(x=.2)
            frame = SimpleNamespace(ts=origin + i / 10, seq=i + 21)
            ChannelWorker._report_position_standstill(worker, frame, valid, True, True, passage)
        end = time.monotonic() + 4
        while not any(row["reason"] for row in self.client.states) and time.monotonic() < end:
            time.sleep(.01)
        self.assertEqual(self.wait_calls(1), 1)
        states = list(self.client.states)
        self.assertTrue(states)
        segment = self.client.calls[0]["segmentId"]
        self.assertTrue(all(row["segmentId"] == segment and row["passageId"] == passage for row in states))
        self.assertEqual([r["sequence"] for r in states], list(range(1, len(states) + 1)))
        self.assertEqual(self.client.calls[0]["stoppedAt"], states[-1]["stoppedAt"])
        return origin, states[-1]

    def test_both_scans_end_at_eight_then_move_at_ten(self):
        origin, state = self._static_scenario(10)
        self.assertEqual(datetime.fromisoformat(state["stoppedAt"]).timestamp(), origin)
        self.assertEqual(datetime.fromisoformat(state["lastStillAt"]).timestamp(), origin + 9.8)
        self.assertEqual(datetime.fromisoformat(state["brokenAt"]).timestamp(), origin + 10)
        self.assertEqual(state["reason"], "movement")

    def test_move_at_seven_never_certifies_scan_ending_at_eight(self):
        origin, state = self._static_scenario(7)
        self.assertEqual(datetime.fromisoformat(state["lastStillAt"]).timestamp(), origin + 6.8)
        self.assertEqual(state["reason"], "movement")

    def test_sequence_gap_cannot_be_bridged_by_later_heartbeats(self):
        origin, state = self._static_scenario(7, gap=True)
        self.assertEqual(datetime.fromisoformat(state["lastStillAt"]).timestamp(), origin + 6.8)
        self.assertEqual(state["reason"], "observation_gap")

    def test_silent_source_freezes_at_last_observation_not_time_of_timeout(self):
        from types import SimpleNamespace
        from app.application.channel_worker import ChannelWorker
        self.bridge.configure_channel("CH4", "a" * 64, "live")
        passage = self.enter()
        self.bridge._prepared["CH4"] = (passage, "awaiting_plate", None, time.monotonic())
        origin = int(time.time()) - 5
        self.bridge.observe_position("CH4", passage, origin, True, None)
        self.bridge.observe_position("CH4", passage, origin + 1.5, True, None)
        worker = SimpleNamespace(source=SimpleNamespace(read=lambda: None), last_frame=None,
                                 _discard_observation_window=lambda: None, _refresh_stats=lambda: None,
                                 st=SimpleNamespace(stability={}), stability=SimpleNamespace(expire=lambda: True, state=0),
                                 _position_still=PositionStandstill(.03, 2., .75), ch=SimpleNamespace(id="CH4"),
                                 integration=self.bridge)
        self.assertFalse(ChannelWorker._tick_observation(worker))
        deadline = time.monotonic() + 3
        while not any(row["reason"] for row in self.client.states) and time.monotonic() < deadline:
            time.sleep(.01)
        final = self.client.states[-1]
        self.assertEqual(final["reason"], "observation_gap")
        self.assertEqual(datetime.fromisoformat(final["lastStillAt"]).timestamp(), origin + 1.4)

    def test_uncommissioned_observation_time_still_triggers_without_certifying_scans(self):
        from app.adapters.webapi_config import ChannelConfig
        self.bridge.config.channels["CH4"] = ChannelConfig("cal-4")
        _, state = self._static_scenario(7)
        self.assertEqual(state["lastStillAt"], state["stoppedAt"])
        self.assertEqual(state["reason"], "observation_timing_unknown")

    def test_delayed_registration_retries_identical_sequence_then_sends_frozen_break(self):
        entered, release = threading.Event(), threading.Event()
        attempts = []
        def post(payload):
            attempts.append(dict(payload))
            if len(attempts) == 1:
                entered.set()
                release.wait(4)
                raise HttpError(409)
            return {"status": "accepted"}
        self.client.report_standstill_state = post
        self.bridge.configure_channel("CH4", "a" * 64, "live")
        passage = self.enter()
        self.bridge._prepared["CH4"] = (passage, "awaiting_plate", None, time.monotonic())
        origin = time.time()
        self.bridge.observe_position("CH4", passage, origin, True, None)
        try:
            self.assertTrue(entered.wait(2))
            self.bridge.observe_position("CH4", passage, origin + 2, True, None)
            self.bridge.observe_position("CH4", passage, origin + 2.1, False, "movement")
            self.assertEqual(self.wait_calls(1), 1)
        finally:
            release.set()
        deadline = time.monotonic() + 4
        while len(attempts) < 3 and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertEqual([item["sequence"] for item in attempts], [1, 1, 2])
        self.assertEqual(attempts[0], attempts[1])
        self.assertEqual(attempts[-1]["reason"], "movement")

    def test_departure_reports_old_passage_and_does_not_reuse_it(self):
        passage = self.enter()
        self.bridge._prepared["CH4"] = (passage, "awaiting_plate", None, time.monotonic())
        self.assertTrue(self.bridge.report_standstill("CH4", passage))
        self.assertEqual(self.wait_calls(1), 1)

        entered, release = threading.Event(), threading.Event()
        report = self.client.report_departure

        def slow_departure(payload):
            entered.set()
            release.wait(10)
            return report(payload)

        self.client.report_departure = slow_departure
        try:
            next_passage = self.enter()
            self.assertTrue(entered.wait(1))
            self.assertNotEqual(passage, next_passage)
            self.bridge._prepared["CH4"] = (next_passage, "awaiting_plate", None, time.monotonic())
            self.assertTrue(self.bridge.report_standstill("CH4", next_passage))
            self.assertEqual(self.wait_calls(2), 2)
            self.assertFalse(self.client.departures)
            self.assertEqual(self.client.calls[1]["passageId"], next_passage)
        finally:
            release.set()
        self.bridge.close()
        self.assertEqual(self.client.departures, [
            {"nodeId": "jt-test", "channelId": "CH4", "passageId": passage}])


if __name__ == "__main__":
    unittest.main()
