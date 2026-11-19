# -*- coding: utf-8 -*-
"""用真实状态机和线程屏障验证控制完成语义，不降低停稳门槛。"""
import copy
import inspect
import os
import sys
import tempfile
import threading
import unittest
import warnings
import time
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from app.demo_director import DemoDirector
from app.detect import EmptyBaseline, StabilityDetector, StabilityState
from app.pointcloud import Frame
from app.service import ChannelWorker, EventBus, LidarService
from app.settings import load_settings
from tests._observations import ObservationClock
from app.control_lock import FairRLock


class _Threads(unittest.TestCase):
    def start_call(self, callback):
        started, done = threading.Event(), threading.Event()
        values, errors = [], []

        def call():
            started.set()
            try:
                values.append(callback())
            except BaseException as error:
                errors.append(error)
            finally:
                done.set()

        thread = threading.Thread(target=call, daemon=True)
        thread.start()
        self.assertTrue(started.wait(2))
        return thread, done, values, errors

    def finish(self, job):
        thread, done, values, errors = job
        self.assertTrue(done.wait(5), "控制或计算未结束")
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        return values[0] if values else None

    def trace_barrier(self, method, marker, callback):
        lines, start = inspect.getsourcelines(method)
        target = next(start + n for n, line in enumerate(lines) if marker in line)
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)

        def traced():
            def trace(frame, event, arg):
                if event == "line" and frame.f_code is method.__func__.__code__ and frame.f_lineno == target:
                    entered.set()
                    if not release.wait(5):
                        raise AssertionError("测试屏障未释放")
                return trace
            sys.settrace(trace)
            try:
                return callback()
            finally:
                sys.settrace(None)

        job = self.start_call(traced)
        self.assertTrue(entered.wait(3), (marker, job[3]))
        return release, job


class TestFairControlLock(_Threads):
    def wait_queued(self, lock, count):
        deadline = time.perf_counter() + 3
        while time.perf_counter() < deadline:
            with lock._condition:
                if len(lock._queue) == count:
                    return
            threading.Event().wait(.001)
        self.fail("等待线程未排入控制锁队列")

    def test_existing_waiters_precede_reacquiring_producer_and_reentry_is_safe(self):
        lock, order, jobs = FairRLock(), [], []

        def record(index):
            with lock:
                order.append(index)
        with lock:
            with lock:
                self.assertTrue(lock.acquire(False))
                lock.release()
            for index in range(3):
                jobs.append(self.start_call(lambda n=index: record(n)))
                self.wait_queued(lock, index + 1)
        # 模拟采集线程立即再读下一帧；不得插队已等待的3个控制请求。
        record(3)
        for job in jobs:
            self.finish(job)
        self.assertEqual(order, [0, 1, 2, 3])

    def test_timeout_and_exception_do_not_leave_ticket_or_ownership(self):
        lock = FairRLock()
        with lock:
            attempt = self.start_call(lambda: lock.acquire(timeout=.02))
            self.assertFalse(self.finish(attempt))
            self.assertFalse(self.finish(self.start_call(lambda: lock.acquire(False))))
            job = self.start_call(lock.release)
            self.assertTrue(job[1].wait(2))
            self.assertIsInstance(job[3][0], RuntimeError)
            job[0].join()
            with lock._condition:
                self.assertEqual(len(lock._queue), 0)
        with self.assertRaises(ValueError):
            with lock:
                raise ValueError("测试调用者异常")
        self.assertTrue(lock.acquire(False))
        lock.release()

    def test_invalid_timeout_cannot_leave_an_ownerless_ticket(self):
        lock = FairRLock()
        for timeout in (float("nan"), float("inf"), -float("inf"), 10 ** 1000,
                        threading.TIMEOUT_MAX * 2, -2):
            with self.subTest(timeout_type=type(timeout).__name__):
                with self.assertRaises((ValueError, OverflowError)):
                    lock.acquire(timeout=timeout)
                self.assertEqual(len(lock._queue), 0)
                self.assertTrue(lock.acquire(False))
                lock.release()
        with lock:
            with patch.object(lock._condition, "wait", side_effect=RuntimeError("等待失败")):
                attempt = self.start_call(lock.acquire)
                self.assertTrue(attempt[1].wait(2))
                attempt[0].join()
                self.assertIsInstance(attempt[3][0], RuntimeError)
                self.assertEqual(len(lock._queue), 0)
            self.assertEqual(lock._owner, threading.get_ident())


class TestDetectorControlBoundary(_Threads):
    def setUp(self):
        self.clock = ObservationClock(self)
        rng = np.random.default_rng(37)
        self.frame = Frame(np.column_stack((rng.uniform(-1, 1, 4000),
                                            rng.uniform(-5, 5, 4000),
                                            rng.uniform(-3.7, -2, 4000))))

    def test_reset_waits_for_each_previously_racy_calculation_boundary(self):
        markers = ("t = np.array([x[index]", "r.stationary_sec =", "self._state = StabilityState.STABLE")
        for marker in markers:
            with self.subTest(marker=marker), warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter("always")
                detector = StabilityDetector(vehicle_threshold=3000, ground_z=-5.23, count_ratio=0)
                self.assertTrue(self.clock.hold(detector, self.frame, steps=30, dt=.1).stable)
                frame = self.clock.frame(self.frame, .1)
                release, computing = self.trace_barrier(detector._detect_locked, marker,
                                                       lambda: detector.detect(frame))
                resetting = self.start_call(detector.reset)
                try:
                    self.assertFalse(resetting[1].wait(.05), "reset不能穿过在途detect")
                finally:
                    release.set()
                self.assertTrue(self.finish(computing).stable)
                self.finish(resetting)
                self.assertEqual(detector.state, StabilityState.IDLE)
                self.assertFalse(detector._present)
                self.assertEqual(detector._last_seq, 0)
                self.assertEqual(len(detector._motion), 0)
                self.assertFalse(self.clock.detect(detector, self.frame, dt=.1).stable)
                self.assertEqual(caught, [])

    def test_invalidate_and_expire_cannot_be_overwritten_by_old_detect(self):
        for operation in ("invalidate", "expire"):
            with self.subTest(operation=operation):
                detector = StabilityDetector(vehicle_threshold=3000, ground_z=-5.23, count_ratio=0)
                self.assertTrue(self.clock.hold(detector, self.frame).stable)
                frame = self.clock.frame(self.frame, .1)
                release, computing = self.trace_barrier(detector._detect_locked,
                    "self._state = StabilityState.STABLE", lambda: detector.detect(frame))
                if operation == "expire":
                    self.clock.advance(1)
                changing = self.start_call(getattr(detector, operation))
                try:
                    self.assertFalse(changing[1].wait(.05))
                finally:
                    release.set()
                self.finish(computing)
                result = self.finish(changing)
                if operation == "expire":
                    self.assertTrue(result)
                self.assertNotEqual(detector.state, StabilityState.STABLE)
                self.assertEqual(len(detector._motion), 0)


# 空车基线按车牌存（2026-09-23）：标定必须知道是哪一辆车。
# 这些用例测的是并发/新鲜度不变量，与车牌无关，统一用一个固定牌。
_PLATE = "宁A5548Q"


class TestWorkerControlBoundary(_Threads):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.clock = ObservationClock(self)
        settings = load_settings("")
        self.settings = settings
        self.ch = copy.deepcopy(settings.channels[0])
        self.ch.id = "CH4"
        self.ch.mode = "synthetic"
        self.ch.abs_baseline_dir = self.tmp.name
        self.worker = self.make_worker()
        rng = np.random.default_rng(29)
        self.frame = Frame(np.column_stack((rng.uniform(-1, 1, 20000),
                                            rng.uniform(-5, 5, 20000),
                                            rng.uniform(-3.7, -2, 20000))))

    def make_worker(self):
        worker = ChannelWorker(self.ch, EventBus(), self.settings.plate, save_override=False, plate_mode="null")
        worker.settle_sec = 100
        self.addCleanup(worker.source.close)
        self.addCleanup(worker.stop)
        return worker

    def warm(self):
        self.clock.settle(self.worker, self.frame)
        for _ in range(3):
            self.clock.tick(self.worker, self.frame)

    def test_reset_returns_only_after_inflight_measurement_authorization(self):
        self.warm()
        worker = self.worker
        worker.settle_sec = 0
        order = []

        def calculate(frame, stab, *args):
            self.assertTrue(stab.stable)
            self.assertGreater(worker._session_id, 0)
            order.append("measurement")
        worker._calculate_detection = calculate
        release, computing = self.trace_barrier(worker._tick_observation,
            "self._run_detection(target, stab)", lambda: self.clock.tick(worker, self.frame))

        def reset():
            worker.force_new_session()
            order.append("reset_returned")
        resetting = self.start_call(reset)
        try:
            self.assertFalse(resetting[1].wait(.05))
        finally:
            release.set()
        self.finish(computing)
        self.finish(resetting)
        self.assertEqual(order, ["measurement", "reset_returned"])
        self.assertEqual(worker._session_id, 0)
        self.assertEqual(worker.stability.state, StabilityState.IDLE)
        self.assertFalse(worker.st.stability["stable"])
        self.assertFalse(worker.st.stability["fresh"])
        self.clock.tick(worker, self.frame)
        self.assertEqual(len(order), 2)

    def test_internal_generation_and_timeout_reset_are_reentrant(self):
        self.warm()
        f = copy.deepcopy(self.frame)
        f.gen = 1
        self.finish(self.start_call(lambda: self.clock.tick(self.worker, f)))
        self.assertFalse(self.worker.st.stability["stable"])
        self.worker.session_timeout = .05
        self.finish(self.start_call(lambda: self.clock.tick(self.worker, f)))
        self.assertFalse(self.worker.st.stability["stable"])

    def test_stop_signal_and_other_channel_do_not_wait_for_active_channel_lock(self):
        self.warm()
        release, computing = self.trace_barrier(self.worker._tick_observation,
            "stab = self.stability.detect", lambda: self.clock.tick(self.worker, self.frame))
        other = self.make_worker()
        try:
            self.finish(self.start_call(lambda: self.clock.tick(other, self.frame)))
            self.finish(self.start_call(self.worker.stop))
            self.assertTrue(self.worker._stop_evt.is_set())
        finally:
            release.set()
        self.finish(computing)
        self.assertFalse(self.worker._tick())
        self.assertTrue(self.worker.close_results())

    def test_snapshots_and_sse_do_not_retain_mutable_state_references(self):
        self.warm()
        worker = self.worker
        worker.history.append({"nested": {"items": [1]}})
        snap, hist, frame = worker.snapshot(), worker.history_snapshot(), worker.frame_snapshot()
        queue = worker.bus.subscribe()
        worker._publish_state(worker.last_frame, force=True)
        event = copy.deepcopy(queue[-1])
        worker.force_new_session()
        worker.history[-1]["nested"]["items"].append(2)
        self.assertTrue(snap["stability"]["stable"])
        self.assertEqual(hist[-1]["nested"]["items"], [1])
        self.assertEqual(snap["history"][-1]["nested"]["items"], [1])
        self.assertEqual(queue[-1], event)
        frame.xyz[:] = 0
        self.assertTrue(np.any(worker.last_frame.xyz != 0))

    def test_baseline_prepare_is_unlocked_and_reset_prevents_stale_commit(self):
        self.warm()
        worker = self.worker
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        baseline = EmptyBaseline(-4, (-1, -5, -4, 1, 5, -2), self.frame.xyz, self.frame.n)
        path = worker.baseline_path_for(_PLATE)
        baseline.save(path)
        with open(path, "rb") as stream:
            original = stream.read()

        def prepare(*args):
            entered.set()
            if not release.wait(5):
                raise AssertionError("基线准备屏障未释放")
            return baseline
        fitted = SimpleNamespace(plane=lambda _: SimpleNamespace(valid=True))
        with patch("app.application.channel_worker.detect_empty", return_value=fitted), \
                patch("app.application.channel_worker.EmptyBaseline.from_empty_result", side_effect=prepare):
            capturing = self.start_call(lambda: worker.capture_baseline(_PLATE))
            self.assertTrue(entered.wait(3))
            try:
                self.finish(self.start_call(worker.force_new_session))
            finally:
                release.set()
            ok, reason = self.finish(capturing)
        self.assertFalse(ok)
        self.assertIn("观测已变化", reason)
        self.assertIsNone(worker.baseline)
        with open(path, "rb") as stream:
            self.assertEqual(stream.read(), original)
        self.assertFalse(any(name.startswith(".baseline-") for name in os.listdir(self.tmp.name)))

    def test_fresh_manual_baseline_can_commit_and_preserve_quality_gate(self):
        self.warm()
        baseline = EmptyBaseline(-4, (-1, -5, -4, 1, 5, -2), self.frame.xyz, self.frame.n)
        fitted = SimpleNamespace(plane=lambda _: SimpleNamespace(valid=True))
        with patch("app.application.channel_worker.detect_empty", return_value=fitted), \
                patch("app.application.channel_worker.EmptyBaseline.from_empty_result", return_value=baseline):
            ok, _ = self.worker.capture_baseline(_PLATE)
        self.assertTrue(ok)
        self.assertEqual(EmptyBaseline.load(self.worker.baseline_path_for(_PLATE)).n, self.frame.n)
        self.assertIs(self.worker.baseline, baseline)

    def test_capture_without_a_plate_is_refused_and_writes_nothing(self):
        """★ 读不到车牌就拒绝标定（2026-09-23，基线按车牌存）。

        绝不能存成「未知车牌」：那样所有认不出牌的车会共用这一份基线，
        恰好重新造出"拿别的车的车厢盒算体积"那个错。
        这里 plate_mode="null"，读牌必然是「未知车牌」。
        """
        from app.plate import EMPTY_PLATE
        self.warm()
        baseline = EmptyBaseline(-4, (-1, -5, -4, 1, 5, -2), self.frame.xyz, self.frame.n)
        fitted = SimpleNamespace(plane=lambda _: SimpleNamespace(valid=True))
        with patch("app.application.channel_worker.detect_empty", return_value=fitted),                 patch("app.application.channel_worker.EmptyBaseline.from_empty_result", return_value=baseline):
            ok, reason = self.worker.capture_baseline()
        self.assertFalse(ok, "读不到车牌却标定成功了")
        self.assertIn("车牌", reason)
        self.assertIsNone(self.worker.baseline)
        self.assertFalse(os.path.exists(self.worker.baseline_path_for(EMPTY_PLATE)),
                         "存出了一份「未知车牌」的基线 —— 所有认不出牌的车都会共用它")
        self.assertEqual([], [n for n in os.listdir(self.tmp.name) if n.endswith(".npz")])

    def test_explicit_plate_capture_lands_under_that_plate(self):
        """操作员直接给车牌时，基线就存在那个车牌名下，且当前通道立刻可用。"""
        self.warm()
        baseline = EmptyBaseline(-4, (-1, -5, -4, 1, 5, -2), self.frame.xyz, self.frame.n)
        fitted = SimpleNamespace(plane=lambda _: SimpleNamespace(valid=True))
        with patch("app.application.channel_worker.detect_empty", return_value=fitted),                 patch("app.application.channel_worker.EmptyBaseline.from_empty_result", return_value=baseline):
            ok, msg = self.worker.capture_baseline(_PLATE)
        self.assertTrue(ok, msg)
        self.assertIn(_PLATE, msg)
        self.assertTrue(os.path.exists(self.worker.baseline_path_for(_PLATE)))
        self.assertIs(self.worker._resolve_baseline(_PLATE)[0], baseline,
                      "刚标定的基线应当进缓存、本通道立刻可用")

    def test_stopped_worker_rejects_control_and_service_does_not_report_success(self):
        self.warm()
        worker = self.worker
        worker.stop()
        epoch = worker._observation_epoch
        self.assertFalse(worker.force_new_session())
        self.assertFalse(worker.set_loaded_demo(True))
        self.assertFalse(worker.set_vehicle_demo(loaded=True))
        self.assertFalse(worker.capture_baseline(_PLATE)[0])
        service = LidarService.__new__(LidarService)
        service.workers, service.director = [worker], None
        self.assertEqual(service.force_new_session(worker.ch.id), 0)
        self.assertEqual(worker._observation_epoch, epoch)
        from app.web.server import DashboardHandler
        handler = DashboardHandler.__new__(DashboardHandler)
        handler.service = service
        handler.path = "/api/demo/force"
        handler._read_json = lambda: {"channel": worker.ch.id}
        handler._json = lambda payload, status=200: (status, payload)
        status, payload = handler.do_POST()
        self.assertEqual(status, 409)
        self.assertFalse(payload["ok"])
        self.assertEqual(payload["channels"], 0)

    def test_demo_change_revokes_old_permission_but_noop_preserves_window(self):
        self.worker = ChannelWorker(self.ch, EventBus(), self.settings.plate,
                                    save_override=False, plate_mode="null", demo=True)
        self.addCleanup(self.worker.source.close)
        self.addCleanup(self.worker.stop)
        self.worker.settle_sec = 100
        self.warm()
        epoch = self.worker._observation_epoch
        self.assertTrue(self.worker.set_vehicle_demo(loaded=False, y_offset=0))
        self.assertEqual(self.worker._observation_epoch, epoch)
        self.assertTrue(self.worker.st.stability["stable"])
        self.assertTrue(self.worker.set_vehicle_demo(loaded=True, y_offset=0))
        self.assertGreater(self.worker._observation_epoch, epoch)
        self.assertFalse(self.worker.st.stability["stable"])
        self.assertFalse(self.worker.capture_baseline(_PLATE)[0])
        new_frame = copy.deepcopy(self.frame)
        new_frame.gen = self.worker.source.generation
        self.finish(self.start_call(lambda: self.clock.tick(self.worker, new_frame)))
        self.assertFalse(self.worker.st.stability["stable"])


class TestDemoControlBoundary(_Threads):
    def test_pause_waits_for_inflight_auto_then_manual_value_wins(self):
        from tests.test_demo_director import _FakeWorker
        worker = _FakeWorker("CH1")
        director = DemoDirector([worker])
        release, applying = self.trace_barrier(director._apply, "if not w.set_vehicle_demo", director._apply)

        def manual():
            director.pause("人工")
            worker.set_vehicle_demo(loaded=True, y_offset=0)
        manual_job = self.start_call(manual)
        try:
            self.assertFalse(manual_job[1].wait(.05))
        finally:
            release.set()
        self.finish(applying)
        self.finish(manual_job)
        self.assertEqual(worker.source.calls[-1], {"loaded": True, "y_offset": 0})
        count = len(worker.source.calls)
        director._apply()
        self.assertEqual(len(worker.source.calls), count)

    def test_service_pauses_director_before_entering_worker_control(self):
        events = []
        worker = SimpleNamespace(ch=SimpleNamespace(id="CH1"),
            set_vehicle_demo=lambda **kw: events.append("vehicle") or True,
            force_new_session=lambda: events.append("reset") or True)
        service = LidarService.__new__(LidarService)
        service.workers = [worker]
        service.director = SimpleNamespace(pause=lambda reason: events.append("pause"))
        self.assertTrue(service.set_loaded_demo("CH1", True))
        self.assertEqual(events, ["pause", "vehicle"])
        events.clear()
        self.assertEqual(service.force_new_session("CH1"), 1)
        self.assertEqual(events, ["pause", "reset"])
