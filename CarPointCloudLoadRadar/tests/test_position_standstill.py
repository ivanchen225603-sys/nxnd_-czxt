# -*- coding: utf-8 -*-
"""CH4 位置停稳触发：独立计时器、检测器只读位姿、bridge 每车次一次的后台通知。"""
import dataclasses
import tempfile
import threading
import time
import unittest

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
        self.failures = 0
        self.event = threading.Event()

    def report_standstill(self, payload):
        self.calls.append(payload)
        self.event.set()
        if self.failures:
            self.failures -= 1
            raise HttpError(None)
        return {"status": "dispatched"}


class TestBridgeStandstillReport(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        config = BridgeConfig(node_id="jt-test", base_url="http://127.0.0.1:1", agent_key="test-key-" * 5,
                              storage_dir=self.tmp.name, algorithm_version="test-v1",
                              channels={"CH4": {"calibration_id": "cal-4"}},
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
        # 登记回执到达（任何状态都算，含等车牌）后，下一次调用即通知
        self.bridge._prepared["CH4"] = (passage, "awaiting_plate", None, time.monotonic())
        self.assertTrue(self.bridge.report_standstill("CH4", passage))
        self.assertEqual(self.wait_calls(1), 1)
        self.assertEqual(self.client.calls[0], {"nodeId": "jt-test", "channelId": "CH4", "passageId": passage})
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
        time.sleep(0.2)
        self.assertFalse(self.bridge.report_standstill("CH4", passage))

    def test_other_passage_is_not_reported(self):
        passage = self.enter()
        self.bridge._prepared["CH4"] = (passage, "ready", None, time.monotonic())
        self.assertFalse(self.bridge.report_standstill("CH4", "p-other"))
        self.assertFalse(self.bridge.report_standstill("CH4", None))


if __name__ == "__main__":
    unittest.main()
