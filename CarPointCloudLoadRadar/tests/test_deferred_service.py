"""异步留样接入回归：纯模拟源/内存依赖，无服务、HTTP 或业务数据库。"""
from collections import Counter, deque
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

from app.deferred_measurements import DeferredMeasurements
from app.load_radar_bridge import ResultReservation
from app.pointcloud import Frame
from app.service import ChannelWorker, StabilityState
from tests._observations import ObservationClock
from tests import test_service as service_cases


TEMP_ROOT = Path(tempfile.gettempdir()) / "load-radar-async-tests"


class TestDeferredService(unittest.TestCase):
    def test_manual_claim_uses_finalizer_capacity_not_automatic_pool(self):
        for finalizer_ready, automatic_ready in ((True, False), (False, True)):
            with self.subTest(finalizer=finalizer_ready):
                worker = ChannelWorker.__new__(ChannelWorker)
                frame = Frame(np.zeros((3, 3), dtype="f4"))
                stab = SimpleNamespace(state=StabilityState.IDLE, degraded=True, fresh=False,
                    stable=False, motion_pending=False, n_vehicle_points=0, triggers={}, to_dict=lambda: {})
                worker.ch = SimpleNamespace(id="CH1")
                worker._gen = frame.gen
                worker.source = SimpleNamespace(read=lambda: frame, stats=SimpleNamespace(frames=1, fps=1.))
                worker.st = SimpleNamespace(incomplete_frames=0)
                worker.stability = SimpleNamespace(detect=lambda *a, **kw: stab)
                worker.calib = SimpleNamespace(detection=SimpleNamespace(
                    vehicle_threshold=100, vehicle_release_ratio=.5))
                worker._complete_hist, worker._triggers = deque(), Counter()
                # 该替身跳过构造函数；补齐逐帧诊断所用的原始状态。
                worker._pkt_hist = deque(maxlen=ChannelWorker._COMPLETE_HISTORY)
                worker._frame_integrity_history = deque(maxlen=ChannelWorker._INCOMPLETE_WINDOW)
                worker._settle_t0, worker._session_id = None, 1
                worker._frame_is_complete = Mock(return_value=True)
                worker._gate_decision = Mock(return_value={})
                worker._discard_observation_window = Mock()
                worker._pair_observation = Mock()
                worker._publish_state = Mock()
                worker._refresh_deferred_exit_qualification = Mock()
                worker._finalize_estimate = Mock(return_value=123)
                worker._finalizer = SimpleNamespace(can_accept=Mock(return_value=finalizer_ready))
                worker._deferred = SimpleNamespace(can_accept=Mock(return_value=automatic_ready))
                worker.integration = SimpleNamespace(observe=Mock(), current_passage=lambda _: "passage",
                    claim_command_fast=Mock(return_value=None))
                self.assertTrue(worker._tick_observation())
                self.assertEqual(finalizer_ready, worker.integration.claim_command_fast.called)
                worker._finalizer.can_accept.assert_called_once_with("CH1", 123)
                worker._deferred.can_accept.assert_not_called()

    def _exit_worker(self):
        TEMP_ROOT.mkdir(parents=True, exist_ok=True)
        tmp = tempfile.TemporaryDirectory(prefix="deferred-exit-", dir=TEMP_ROOT)
        self.addCleanup(tmp.cleanup)
        with patch("tempfile.mkdtemp", return_value=tmp.name):
            worker = service_cases.TestExitFrames()._worker(exit_frames=10, exit_step_m=.10)
        self.addCleanup(worker.source.close)
        self.addCleanup(worker.stop)
        worker._publish_state = Mock()
        worker._flush_exit = Mock(side_effect=worker._exit_reset)
        clock = ObservationClock(self)
        rng = np.random.default_rng(7)
        n = worker.stability.vehicle_threshold * 2
        cloud = np.column_stack([rng.uniform(-1.2, 1.2, n), rng.uniform(-6., 6., n),
                                 rng.uniform(-4.2, -2.5, n)]).astype("f4")
        clock.tick(worker, Frame(cloud, packets=900))
        current = {"passage": "original-passage"}
        identity = ResultReservation("original-measurement", worker.ch.id, current["passage"], "original-token")
        bridge = SimpleNamespace(config=SimpleNamespace(), capture_identity=lambda *_: (identity, b"{}"),
            current_passage=lambda _: current["passage"], observe=Mock(), claim_command_fast=Mock(return_value=None))
        store = SimpleNamespace(snapshot=lambda _: dict(pending_jobs=0, pending_bytes=0,
            max_jobs_per_channel=10, max_bytes_per_channel=1024 * 1024),
            put=Mock(return_value=SimpleNamespace(identity=identity)), close=Mock())
        engine = DeferredMeasurements(bridge, SimpleNamespace(), [worker.ch.id], store=store)
        worker.integration, worker._deferred = bridge, engine
        draft = SimpleNamespace(byte_size=lambda: 2, encode=lambda: b"{}")
        with patch("app.deferred_measurements.FrozenDetection.capture", return_value=draft):
            worker._run_detection(Frame(cloud, packets=900), None)
        return worker, clock, cloud, current, identity, engine, store

    def test_durable_capture_enables_real_moving_exit_frames_only_after_success(self):
        worker, clock, cloud, _, identity, engine, store = self._exit_worker()
        clock.tick(worker, Frame((cloud + [0., .4, 0.]).astype("f4"), packets=900))
        self.assertIsNone(worker._exit_qualification)
        self.assertEqual([], worker._exit_shots)
        with patch("app.deferred_measurements.inspect_frozen", return_value={}):
            store.put.side_effect = OSError("模拟写盘失败")
            with self.assertRaises(OSError):
                engine.persist_once(worker.ch.id)
            clock.tick(worker, Frame((cloud + [0., .8, 0.]).astype("f4"), packets=900))
            self.assertIsNone(worker._exit_qualification)
            self.assertIsNone(engine.durable_capture(worker.ch.id))
            store.put.side_effect = None
            self.assertTrue(engine.persist_once(worker.ch.id))
        for step in range(3, 8):
            clock.tick(worker, Frame((cloud + [0., .4 * step, 0.]).astype("f4"), packets=900))
        self.assertEqual(worker._exit_scope(), worker._exit_qualification)
        self.assertEqual(identity.measurement_id, worker._exit_measurement_id)
        self.assertGreater(len(worker._exit_shots), 0)

    def test_late_persistence_does_not_enable_next_passage_or_cleared_token(self):
        worker, clock, cloud, current, _, engine, _ = self._exit_worker()
        with patch("app.deferred_measurements.inspect_frozen", return_value={}):
            engine.persist_once(worker.ch.id)
        current["passage"] = "next-passage"
        clock.tick(worker, Frame((cloud + [0., .4, 0.]).astype("f4"), packets=900))
        self.assertIsNone(worker._exit_qualification)
        self.assertEqual([], worker._exit_shots)
        current["passage"] = "original-passage"
        worker._current_result_token = ""
        clock.tick(worker, Frame((cloud + [0., .8, 0.]).astype("f4"), packets=900))
        self.assertIsNone(worker._exit_qualification)
        current["passage"] = None
        worker._current_result_token = "original-token"
        worker._refresh_deferred_exit_qualification()
        self.assertIsNone(worker._exit_qualification)

    def test_restart_without_reliable_passage_does_not_fall_back_to_another_measurement(self):
        worker, clock, cloud, current, _, _, _ = self._exit_worker()
        worker.eff_mode = "live"
        worker._detected_this_session = 0
        worker._run_detection = Mock()
        current["passage"] = None
        for _ in range(40):
            clock.tick(worker, Frame(cloud.copy(), packets=900))
        worker._run_detection.assert_not_called()
        self.assertIsNone(worker._settle_t0)
        self.assertEqual("awaiting_passage", worker.st.last_diag["preparation"])


if __name__ == "__main__":
    unittest.main()
