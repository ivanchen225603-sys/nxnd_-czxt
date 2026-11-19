# -*- coding: utf-8 -*-
"""持续停稳的独立轨迹真值、异常时序及正向测量能力。"""
import unittest

import numpy as np

from app.detect import StabilityDetector, StabilityState
from app.pointcloud import Frame
from tests._observations import ObservationClock


class TestContinuousStandstill(unittest.TestCase):
    def setUp(self):
        self.clock = ObservationClock(self)
        rng = np.random.default_rng(37)
        self.car = np.column_stack((rng.uniform(-1, 1, 4000),
                                    rng.uniform(-5, 5, 4000),
                                    rng.uniform(-3.7, -2, 4000))).astype(np.float32)
        self.ground = np.column_stack((rng.uniform(-2, 2, 80000),
                                       rng.uniform(-8, 8, 80000),
                                       np.full(80000, -5.23))).astype(np.float32)

    def detector(self, **kw):
        return StabilityDetector(vehicle_threshold=3000, ground_z=-5.23,
                                 count_ratio=0, **kw)

    def cloud(self, y=0, ground=False):
        car = self.car + np.array([0, y, 0], np.float32)
        return Frame(np.vstack((car, self.ground)) if ground else car)

    def test_creep_is_rejected_at_all_sampling_rates_and_background_ratios(self):
        for hz in (5, 10, 20):
            for background in (False, True):
                d = self.detector()
                for i in range(4 * hz + 1):
                    r = self.clock.detect(d, self.cloud(.4 * i / hz, background), dt=1 / hz)
                    self.assertFalse(r.stable, (hz, background, i, r.to_dict()))

    def test_brief_stop_not_stable_but_continuous_stop_is(self):
        d = self.detector()
        for _ in range(3):
            self.assertFalse(self.clock.detect(d, self.cloud(), dt=.1).stable)
        r = self.clock.hold(d, self.cloud(), steps=24, dt=.1)
        self.assertTrue(r.stable)
        self.assertGreaterEqual(r.stationary_sec, 2)

    def test_slow_motion_after_stop_invalidates_fixed_anchor(self):
        for speed in (.01, .04, .4):
            d = self.detector()
            self.assertTrue(self.clock.hold(d, self.cloud()).stable)
            results = [self.clock.detect(d, self.cloud(speed * i / 10), dt=.1)
                       for i in range(1, 41)]
            self.assertTrue(any(not r.stable for r in results), speed)
            if speed > d.stationary_speed_mps:
                self.assertFalse(results[-1].stable)

    def test_repeated_identity_and_repeated_timestamp_cannot_create_stability(self):
        for repeat_seq in (True, False):
            d = self.detector()
            ts = self.clock.wall
            for i in range(50):
                self.clock.advance(.1)
                r = d.detect(Frame(self.car, seq=1 if repeat_seq else i + 1, ts=ts))
                self.assertFalse(r.stable)

    def test_duplicate_identity_revokes_before_age_deadline(self):
        for repeat_seq in (True, False):
            d = self.detector()
            self.assertTrue(self.clock.hold(d, self.cloud()).stable)
            last_seq, last_ts = self.clock.seq, self.clock.wall
            self.clock.advance(.1)
            f = Frame(self.car, seq=last_seq if repeat_seq else last_seq + 1,
                      ts=self.clock.wall if repeat_seq else last_ts)
            self.assertLess(self.clock.wall - f.ts, d.stationary_max_gap_sec)
            r = d.detect(f)
            self.assertFalse(r.fresh)
            self.assertFalse(r.stable)
            self.clock.seq += 1
            self.assertTrue(self.clock.hold(d, self.cloud()).stable)

    def test_missing_identity_does_not_authorize_measurement(self):
        d = self.detector()
        for _ in range(40):
            self.clock.advance(.1)
            self.assertFalse(d.detect(Frame(self.car)).stable)

    def test_stale_and_future_captures_cannot_authorize_measurement(self):
        for age in (-1, 10, 1000):
            d = self.detector()
            for i in range(50):
                self.clock.advance(.1)
                r = d.detect(Frame(self.car, seq=i + 1, ts=self.clock.wall - age))
                self.assertFalse(r.fresh)
                self.assertFalse(r.stable)

    def test_processing_backlog_cannot_stretch_capture_duration(self):
        d = self.detector()
        started = self.clock.wall
        for i in range(30):
            self.clock.advance(.1)
            r = d.detect(Frame(self.cloud(.4 * i / 1000).xyz,
                               seq=i + 1, ts=started + i / 1000))
            self.assertFalse(r.stable, "20毫秒采集不能靠处理延迟变成2秒停稳")

    def test_starting_motion_revokes_permission_on_first_coherent_step(self):
        for speed in (.04, .4):
            d = self.detector()
            self.assertTrue(self.clock.hold(d, self.cloud()).stable)
            r = self.clock.detect(d, self.cloud(speed * .1), dt=.1)
            self.assertFalse(r.stable)

    def test_oscillation_cannot_cancel_into_low_regression_speed(self):
        d = self.detector()
        for i in range(50):
            r = self.clock.detect(d, self.cloud(.01 if i % 2 else -.01), dt=.1)
            self.assertFalse(r.stable)

    def test_rotation_about_fixed_center_is_not_standstill(self):
        d = self.detector()
        for i in range(101):
            angle = np.deg2rad(i / 10)
            rot = np.array([[np.cos(angle), -np.sin(angle)],
                            [np.sin(angle), np.cos(angle)]])
            xyz = self.car.copy()
            xyz[:, :2] = xyz[:, :2] @ rot.T
            r = self.clock.detect(d, Frame(xyz), dt=.1)
            self.assertFalse(r.stable, (i, r.to_dict()))

    def test_symmetric_shape_change_is_not_cancelled_as_translation(self):
        d = self.detector()
        for i in range(101):
            xyz = self.car.copy()
            xyz[:, 1] *= 1 + .01 * i / 10
            r = self.clock.detect(d, Frame(xyz), dt=.1)
            self.assertFalse(r.stable, (i, r.to_dict()))

    def test_shape_change_after_stop_revokes_on_first_frame(self):
        d = self.detector()
        self.assertTrue(self.clock.hold(d, self.cloud()).stable)
        xyz = self.car.copy()
        xyz[:, 1] *= 1.001
        r = self.clock.detect(d, Frame(xyz), dt=.1)
        self.assertFalse(r.stable)
        self.assertGreater(r.shape_speed_mps, d.stationary_speed_mps)

    def test_fixed_rays_with_independent_two_mm_noise_remain_measurable(self):
        rng = np.random.default_rng(44)
        for hz in (5, 10, 20):
            d = self.detector()
            results = []
            for _ in range(7 * hz):
                noisy = self.car + rng.normal(0, .002, self.car.shape)
                results.append(self.clock.detect(d, Frame(noisy), dt=1 / hz))
            # 中位位置会受局部采样噪声影响；允许既有禁测复核状态，但不能清零
            # 已有连续观察、无限等待，或把未确认帧当成已停稳帧。
            for i in range(3 * hz, 6 * hz):
                r = results[i]
                self.assertTrue(r.stable or (r.motion_pending and r.triggers == ("translation_pending",)),
                                (hz, i, r.to_dict()))
                self.assertGreater(r.stationary_sec, results[i - 1].stationary_sec)
                if r.motion_pending:
                    self.assertFalse(r.stable)
                    following = results[i + 1:i + 1 + int(np.ceil(d.stationary_speed_window_sec * hz))]
                    self.assertTrue(any(next_result.stable for next_result in following), (hz, i))
            first = next(i for i, r in enumerate(results) if r.stable)
            measurable = [i for i, r in enumerate(results) if r.stable and (i - first) / hz >= 3.]
            self.assertTrue(measurable, hz)
            self.assertLessEqual((measurable[0] + 1) / hz, 10.)

    def test_isotropic_view_cannot_prove_rotation_has_stopped(self):
        x, y = np.meshgrid(np.linspace(-1.2, 1.2, 80), np.linspace(-1.2, 1.2, 80))
        base = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, -3)))
        d = self.detector()
        for i in range(50):
            angle = np.deg2rad(i / 10)
            xy = base[:, :2] @ np.array([[np.cos(angle), -np.sin(angle)],
                                         [np.sin(angle), np.cos(angle)]]).T
            r = self.clock.detect(d, Frame(np.column_stack((xy, base[:, 2]))), dt=.1)
            self.assertFalse(r.motion_observable)
            self.assertFalse(r.stable)

    def test_receive_jitter_does_not_discard_needed_capture_span(self):
        d = self.detector()
        results = []
        for i in range(30):
            self.clock.advance(.5)
            f = Frame(self.car, seq=i + 1, ts=self.clock.wall - (i % 3) * .005)
            results.append(d.detect(f))
        self.assertTrue(all(r.stable for r in results[8:]))

    def test_gap_clears_stop_not_vehicle_presence(self):
        d = self.detector()
        self.assertTrue(self.clock.hold(d, self.cloud()).stable)
        self.clock.advance(1)
        self.assertTrue(d.expire())
        self.assertEqual(d.state, StabilityState.BASELINE)
        self.assertFalse(self.clock.detect(d, self.cloud()).stable)
        self.assertTrue(self.clock.hold(d, self.cloud()).stable)

    def test_partial_frame_and_nan_clear_stop_without_false_departure(self):
        d = self.detector()
        for kind in ("partial", "nan"):
            self.assertTrue(self.clock.hold(d, self.cloud()).stable)
            f = self.cloud()
            if kind == "nan":
                f.xyz[0, 0] = np.nan
            else:
                f = Frame(f.xyz[:10])
            r = self.clock.detect(d, f, trusted=(kind != "partial"))
            self.assertFalse(r.stable)
            self.assertTrue(r.degraded)
            self.assertNotEqual(r.state, StabilityState.IDLE)

    def test_wall_clock_jump_does_not_add_stop_time(self):
        d = self.detector()
        self.clock.detect(d, self.cloud(), dt=.1)
        self.clock.wall += 100
        r = self.clock.detect(d, self.cloud(), dt=.1)
        self.assertFalse(r.stable)
        self.assertAlmostEqual(r.stationary_sec, .1)
        self.clock.wall -= 150
        r = self.clock.detect(d, self.cloud(), dt=.1)
        self.assertFalse(r.fresh)
        self.assertFalse(r.stable)
        self.assertTrue(self.clock.hold(d, self.cloud()).stable)

    def test_real_departure_requires_fresh_complete_frame(self):
        d = self.detector()
        self.assertTrue(self.clock.hold(d, self.cloud()).stable)
        g = Frame(self.ground)
        self.assertNotEqual(self.clock.detect(d, g, trusted=False).state, StabilityState.IDLE)
        self.assertEqual(self.clock.detect(d, g).state, StabilityState.IDLE)

    def test_ground_change_does_not_indicate_vehicle_motion(self):
        d = self.detector()
        for i in range(30):
            g = self.ground.copy()
            g[:, 1] += .5 * i
            r = self.clock.detect(d, Frame(np.vstack((self.car, g))), dt=.1)
        self.assertTrue(r.stable)
        self.assertGreater(r.deviation, .4)

    def test_gate_cannot_be_disabled_by_zero_or_nonfinite(self):
        for name in ("stationary_min_sec", "stationary_speed_mps", "stationary_drift_m",
                     "stationary_speed_window_sec", "stationary_max_gap_sec"):
            for value in (0, -1, np.nan, np.inf):
                with self.assertRaises(ValueError, msg=(name, value)):
                    self.detector(**{name: value})

    def test_invalid_window_relationships_fail_fast(self):
        for kw in ({"stationary_max_gap_sec": 2}, {"stationary_speed_window_sec": 3}):
            with self.assertRaises(ValueError):
                self.detector(**kw)


class TestStandstillConfiguration(unittest.TestCase):
    def load(self, values):
        import json
        import os
        import tempfile
        from app.settings import load_calib
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "config.json")
            with open(path, "w", encoding="utf-8") as stream:
                json.dump({"detection": values}, stream)
            return load_calib(path).detection

    def test_defaults_are_enabled_without_changing_field_json(self):
        cfg = self.load({})
        self.assertEqual(cfg.stationary_min_sec, 2)
        self.assertEqual(cfg.stationary_speed_mps, .02)
        self.assertEqual(cfg.stationary_drift_m, .03)
        self.assertEqual(cfg.stationary_speed_window_sec, .5)
        self.assertEqual(cfg.stationary_max_gap_sec, .75)
        self.assertEqual(cfg.stationary_jitter_m, .0031)

    def test_loader_validates_and_applies_jitter_budget(self):
        self.assertEqual(self.load({"stationary_jitter_m": .002}).stationary_jitter_m, .002)
        for value in (0., float("nan"), .03):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.load({"stationary_jitter_m": value})

    def test_loader_rejects_nonfinite_or_disabled_values(self):
        for key in ("stationary_min_sec", "stationary_speed_mps", "stationary_drift_m",
                    "stationary_speed_window_sec", "stationary_max_gap_sec"):
            for value in (0, -1, float("nan"), float("inf")):
                with self.assertRaises(ValueError, msg=(key, value)):
                    self.load({key: value})

    def test_loader_rejects_inconsistent_windows(self):
        for values in ({"stationary_min_sec": .5}, {"stationary_speed_window_sec": 3}):
            with self.assertRaises(ValueError):
                self.load(values)

    def test_loader_rejects_unknown_spelling(self):
        with self.assertRaises(ValueError):
            self.load({"stationary_min_second": 2})


class TestWorkerContinuousStandstill(unittest.TestCase):
    def setUp(self):
        from unittest.mock import patch
        from app.service import ChannelWorker, EventBus
        from app.settings import load_settings
        self.clock = ObservationClock(self)
        st = load_settings("")
        ch = st.channels[0]
        ch.mode = "synthetic"
        with patch("app.application.channel_worker.EmptyBaseline.load", return_value=None):
            self.worker = ChannelWorker(ch, EventBus(), st.plate, save_override=False,
                                        plate_mode="null", raw_frames=3, state_push_hz=0)
        self.addCleanup(self.worker.stop)
        self.results = []

        def record(frame, stab):
            self.results.append((frame.seq, self.clock.mono, stab.to_dict()))
            self.worker._detected_this_session += 1

        self.worker._run_detection = record
        rng = np.random.default_rng(29)
        self.xyz = np.column_stack((rng.uniform(-1, 1, 40000),
                                    rng.uniform(-5, 5, 40000),
                                    rng.uniform(-3.7, -2, 40000))).astype(np.float32)

    def f(self, y=0, packets=900):
        return Frame(self.xyz + [0, y, 0], packets=packets)

    def test_zero_extra_settle_still_requires_continuous_standstill(self):
        w = self.worker
        w.settle_sec = 0
        for _ in range(3):
            self.clock.tick(w, self.f())
        self.assertEqual(self.results, [])
        for _ in range(70):
            self.clock.tick(w, self.f())
        self.assertEqual(len(self.results), 1)

    def test_departing_at_measurement_boundary_revokes_permission(self):
        w = self.worker
        self.clock.settle(w, self.f())
        while self.clock.mono - w._settle_t0 < w.settle_sec - .15:
            self.clock.tick(w, self.f())
        for i in range(1, 10):
            self.clock.tick(w, self.f(.004 * i))
        self.assertEqual(self.results, [])
        self.assertFalse(w.st.stability["stable"])
        self.assertEqual(w._raw_shots, [])

    def test_gap_updates_display_once_and_discards_active_frames(self):
        w = self.worker
        self.clock.settle(w, self.f())
        self.assertTrue(w._raw_shots)
        q = w.bus.subscribe()
        self.clock.tick(w, None, dt=.9)
        self.assertEqual(w.st.stability["state_cn"], "运动中")
        self.assertFalse(w.st.stability["stable"])
        self.assertIsNone(w.st.stability["speed_mps"])
        self.assertEqual(w.st.stability["stationary_sec"], 0)
        self.assertEqual(w._raw_shots, [])
        self.assertEqual(len(w._recent_frames), 0)
        self.assertEqual(len(w._integration_frames), 0)
        self.clock.tick(w, None, dt=.1)
        self.assertEqual(sum(e["kind"] == "state" for e in q), 1)
        self.clock.tick(w, self.f(), dt=3)
        self.assertEqual(self.results, [])

    def test_completeness_is_evaluated_once_and_partial_never_measures(self):
        from unittest.mock import patch
        w = self.worker
        self.clock.settle(w, self.f())
        with patch.object(w, "_frame_is_complete", wraps=w._frame_is_complete) as checked:
            for _ in range(8):
                self.clock.tick(w, self.f(packets=450))
            self.assertEqual(checked.call_count, 8)
        self.assertEqual(self.results, [])
        self.assertEqual(w._raw_shots, [])

    def test_explicit_reset_clears_sampling_not_just_timer(self):
        w = self.worker
        self.clock.settle(w, self.f())
        self.assertTrue(w._raw_shots)
        w.force_new_session()
        self.assertEqual(w._raw_shots, [])
        self.assertEqual(len(w._recent_frames), 0)
        self.assertIsNone(w._settle_t0)
        self.clock.tick(w, self.f())
        self.assertFalse(w.st.stability["stable"])

    def test_manual_baseline_rejects_stale_stop_without_writing(self):
        w = self.worker
        w.ch.id = "CH4"
        self.clock.settle(w, self.f())
        self.clock.advance(1)
        ok, reason = w.capture_baseline()
        self.assertFalse(ok)
        self.assertIn("连续完整", reason)


class TestMillimetreCreepCountsAsStopped(unittest.TestCase):
    """★ 毫米级爬行**故意判成停稳**（2026-09-19 实测定案）。

    曾经加过一条"净位移 + 直线度"的蠕行判据去堵
    `immediate_speed`（相邻帧 0.02 m/s = 每帧 2 mm）与 `drift`（30 mm）
    之间那段 0.002~0.015 m/s 的盲区。**撤掉了**，代价方向和当初设想的相反：

    · 漏判不是"错数"。测量用**单帧**（一圈约 100 ms），5 mm/s 帧内只糊 0.5 mm。
      帧内剪切实测：0.005 / 0.05 / 0.2 / 1.0 m/s 的**内宽偏差全是 0.00 mm**，
      厢长偏差 0 / 50 / 50 / 100 mm（50 mm 正好是精修步长一格）；
      而内宽实测标准差 **68 mm**、厢长帧间离散中位 **150 mm**。
    · 误判的代价是**整车漏测**：现场停稳后只有约 10 s 放行，
      2 s 连稳 + 3 s 沉降已占 5 s。整云刚体抖动下实测，
      10 Hz >= 1.5 mm、20 Hz >= 0.75 mm 时**真正停住的车从未判稳**。
    · 它也没拦住别的：关掉后 5 cm/s 起仍全部被既有判据拦死。

    这个类钉的就是"别再把它加回来"，以及"真正该拦的仍然拦得住"。
    """

    def setUp(self):
        self.clock = ObservationClock(self)
        rng = np.random.default_rng(37)
        self.car = np.column_stack((rng.uniform(-1, 1, 4000),
                                    rng.uniform(-5, 5, 4000),
                                    rng.uniform(-3.7, -2, 4000))).astype(np.float32)

    def detector(self, **kw):
        return StabilityDetector(vehicle_threshold=3000, ground_z=-5.23,
                                 count_ratio=0, **kw)

    def drive(self, detector, speed_mps, frames=80, dt=.1, jitter_m=0., rigid_m=0., seed=5):
        """匀速前进，可叠加逐点噪声或整云刚体横向抖动。返回每帧的 stable。"""
        rng = np.random.default_rng(seed)
        out = []
        for i in range(frames):
            xyz = self.car + np.array([0, speed_mps * dt * i, 0], np.float32)
            if rigid_m:
                xyz = xyz + np.array([rng.uniform(-rigid_m, rigid_m), 0, 0], np.float32)
            if jitter_m:
                xyz = xyz + rng.normal(0, jitter_m, xyz.shape).astype(np.float32)
            out.append(self.clock.detect(detector, Frame(xyz), dt=dt).stable)
        return out

    def test_millimetre_creep_is_allowed_to_measure(self):
        """毫米级爬行必须放行 —— 拦它换不来精度，只会吃光 10 s 放行预算。"""
        for speed in (.002, .005, .010):
            with self.subTest(mm_s=speed * 1000):
                self.assertIn(True, self.drive(self.detector(), speed),
                              "毫米级爬行被拦住了，会把车卡在 10 s 预算外")

    def test_driving_speed_is_still_rejected(self):
        """真正要拦的是**米/秒级驶入** —— 2026-09-14 CH4 那次点数
        16737 -> 24434 -> 83919，车一直在开却被判稳。这条不许放松。"""
        for speed in (.05, .3, 1.0, 3.0):
            with self.subTest(m_s=speed):
                self.assertNotIn(True, self.drive(self.detector(), speed),
                                 "行驶中的车被判稳了")

    def test_stationary_vehicle_fits_the_ten_second_budget(self):
        """停住的车必须在 10 s 内攒够 2 s 连稳 + 3 s 沉降。"""
        for rigid_mm in (0.5, 0.75, 1.0):
            with self.subTest(rigid_mm=rigid_mm):
                detector, rng, run, granted = self.detector(), np.random.default_rng(5), 0, None
                for i in range(100):
                    jitter = rng.uniform(-rigid_mm / 1000., rigid_mm / 1000.)
                    frame = Frame(self.car + np.array([jitter, 0, 0], np.float32))
                    run = run + 1 if self.clock.detect(detector, frame, dt=.1).stable else 0
                    if run * .1 >= 3.0 and granted is None:
                        granted = (i + 1) * .1
                self.assertIsNotNone(granted, "停住的车没能拿到测量许可")
                self.assertLessEqual(granted, 10.0, "超出现场 10 s 放行预算")

    def test_triggers_name_the_condition_that_fired(self):
        """归因必须指出是哪条判据拦的，而不是只说"运动中"。"""
        detector, seen = self.detector(), set()
        for i in range(20):
            frame = Frame(self.car + np.array([0, 1.0 * .1 * i, 0], np.float32))
            seen.update(self.clock.detect(detector, frame, dt=.1).triggers)
        self.assertTrue(seen & {"immediate", "regression", "shape_speed", "yaw_speed"},
                        "行驶中的车没有给出任何判据归因")


class TestFieldCountGateValue(unittest.TestCase):
    """★ 现场 `stability_count_ratio` 是 0.03，而全库没有一处按它构造检测器。

    `tests/test_detect.py::TestEnteringVehicleIsNotStable` 那 11 条跑的是
    `count_ratio=0.005`（`_det` 的默认值），本文件又全部写死 `count_ratio=0`
    —— 现场真正生效的那一档**从没进过回归**。

    判据本体（`StabilityDetector`）是"车辆点的近 3 帧均值与 8 帧前那 3 帧均值
    相对差超过该比例就不算停稳"。**因为比较跨 8 帧，比例门槛的等效逐帧灵敏度是
    `(1 + ratio) ** (1/8) - 1`**：0.03 对应 0.37%/帧，0.005 对应 0.062%/帧
    —— 现场值比被覆盖的那档**松 6 倍**。这一点只看配置里的 0.03 是看不出来的，
    所以下面把边界按现场值钉死，四件事缺一件现场就会以
    "判不出停稳"（过严）或"同一辆车测两遍"（过松）的方式失效。
    """

    def setUp(self):
        self.clock = ObservationClock(self)
        rng = np.random.default_rng(37)
        self.car = np.column_stack((rng.uniform(-1, 1, 4000),
                                    rng.uniform(-5, 5, 4000),
                                    rng.uniform(-3.7, -2, 4000))).astype(np.float32)

    def detector(self, ratio=0.03):
        return StabilityDetector(vehicle_threshold=3000, ground_z=-5.23,
                                 count_ratio=ratio)

    def cloud(self, n):
        """点数精确为 n 的一帧，且**空间分布几乎逐位不变**。

        `self.car[:n]` 那种切法会连带挪走 p10/p50/p90 与标准差 ——
        那条路径上先响的是 `shape_drift` / `spread_speed` / `immediate`，
        量出来的就不是点数判据了。这里按分位**均匀**增删（加的点跨整个分布取、
        删的点也跨整个分布删），实测 4000 点上下的 +-0.1% 波动只让
        各分位挪动 <= 4.4 mm、标准差挪动 <= 0.5 mm，落在 `immediate`
        单步预算（0.5 秒 10 mm）以内，于是只有 `n_veh` 真正改变。
        """
        if n > len(self.car):
            pick = np.linspace(0, len(self.car) - 1, n - len(self.car)).astype(int)
            return Frame(np.vstack((self.car, self.car[pick])).astype(np.float32))
        keep = np.unique(np.linspace(0, len(self.car) - 1, n).astype(int))
        return Frame(self.car[keep].astype(np.float32))

    def ramp(self, detector, per_frame, frames=24, start=4000):
        """按给定比例逐帧增减点数，返回出现过的触发标记与逐帧 stable。"""
        n, seen, stable = start, set(), []
        for _ in range(frames):
            n = int(round(n * (1 + per_frame)))
            r = self.clock.detect(detector, self.cloud(n), dt=.5)
            seen.update(r.triggers)
            stable.append(r.stable)
        return seen, stable

    def test_field_value_tolerates_parked_drift_that_the_old_value_rejected(self):
        """停稳车的点数**持续**漂移：0.03 放行，而被覆盖的 0.005 会把同一序列拦掉。

        实测停稳时逐帧点数波动最大 0.11%，这里按同一量级做成持续同向漂移
        （0.1%/帧，8 帧累计约 0.9%）。两次跑用的是**逐字节相同**的帧序列与时钟，
        唯一的差别就是 `count_ratio`，所以结论不同只可能来自这一档门槛。

        ▲ 为什么不用**来回抖动**的序列：3 对 3 的均值窗口会把振荡抹平，
        `ratio=0.005` 与 `0.03` 对它的结论一模一样（实测两档都放行）。
        拿振荡论证"必须松到 0.03"在现行判据下站不住 —— 两档真正分开的是
        **持续趋势**。所以这条用例对 `0.005` 断的是反面，确保钉的是现场值本身。
        """
        for ratio, expect in ((.03, True), (.005, False)):
            with self.subTest(ratio=ratio):
                detector, last = self.detector(ratio=ratio), None
                for i in range(12):
                    last = self.clock.detect(detector, self.cloud(4000 + 4 * i), dt=.5)
                self.assertEqual(expect, last.stable, (ratio, last.to_dict()))

    def test_field_value_still_blocks_entering_growth(self):
        """车还在进场（点数每帧 +7%）时，现场值也照样拦住，且归因里有 `count`。"""
        seen, stable = self.ramp(self.detector(), .07, frames=20)
        self.assertFalse(any(stable), seen)
        self.assertIn("count", seen, "进场没有归因到点数判据")

    def test_field_value_attributes_departure_to_the_count_gate(self):
        """车开走时点数骤降 —— 现场值必须把这次变化归因到 `count`。

        实测离场 80 141 -> 13 978，而质心偏差只有 0.0478，旧判据一路显示「已停稳」
        直到车都快出画面。骤降这么大时 `immediate` / `shape_speed` 也会一起响，
        所以这里钉的**不是"只有 count 拦得住"**，而是"现场值下 `count` 确实参与
        归因" —— 同一组帧在 `count_ratio=0` 的 `triggers` 里没有它，在 0.03 里有。
        （这一条盯的是"同一辆车落两份盘"：点数掉到退出阈值以下会走「无车」早退、
        结束会话，下一帧点数回来又开新会话；`count` 在**退出阈值之上**就先按住它。）
        """
        for ratio, expect in ((.03, True), (0.0, False)):
            with self.subTest(ratio=ratio):
                detector = self.detector(ratio=ratio)
                self.assertTrue(self.clock.hold(detector, self.cloud(4000), dt=.5).stable)
                r = self.clock.detect(detector, self.cloud(3200), dt=.5)
                self.assertFalse(r.stable, r.to_dict())
                self.assertEqual(expect, "count" in r.triggers, r.triggers)

    def test_field_value_sensitivity_is_the_eight_frame_gap(self):
        """0.03 的等效边界就是"8 帧累计 3%"，两侧各钉一条。

        低的一侧（0.2%/帧，8 帧累计 1.6%）**不响**，高的一侧（0.5%/帧，
        8 帧累计 4.1%）**响**。同一条 0.2%/帧的缓升在 0.005 下是会响的 ——
        这正是"现场值比被覆盖的 0.005 松 6 倍"的具体含义，
        别只看到 0.03 这个数字就以为它和用例里跑的一样严。
        """
        for per_frame, expect in ((.002, False), (.005, True)):
            with self.subTest(per_frame=per_frame):
                seen, _ = self.ramp(self.detector(), per_frame)
                self.assertEqual(expect, "count" in seen, sorted(seen))
        self.assertIn("count", self.ramp(self.detector(ratio=.005), .002)[0],
                      "0.005 档对 0.2%/帧 的缓升应当有反应")


class TestPermitRecordsEveryStandstillGate(unittest.TestCase):
    """许可记录必须写全「当时生效的停稳门槛」。

    `_gate_decision` 的门槛名是**写死的元组**，`measurement_runtime` 那边又
    单独写了一份字符串。加判据时漏掉任何一边，归档下来的许可就说不清
    「这次放行是在哪套门槛下做的」—— 而停稳门槛**不在 `config_fingerprint()`
    里**（指纹只盖判定链），许可记录是它唯一的留痕处。
    2026-09-19 加 `stationary_net_*` 时就漏过一次。
    """

    def test_every_stationary_gate_is_recorded(self):
        from types import SimpleNamespace
        from app.service import ChannelWorker
        detector = StabilityDetector(vehicle_threshold=3000, ground_z=-5.0)
        frame = SimpleNamespace(seq=1, ts=1.0, gen=0, packets=450)
        stab = detector.detect(Frame(np.zeros((1, 3), np.float32)))
        worker = SimpleNamespace(stability=detector, _last_frame_monotonic=1.0)
        recorded = ChannelWorker._gate_decision(worker, frame, stab, True)["thresholds"]
        missing = sorted(a for a in vars(detector)
                         if a.startswith("stationary_") and a not in recorded)
        self.assertEqual([], missing, "新加的停稳门槛没写进许可记录")


class TestFrameQualityIsAttributed(unittest.TestCase):
    """★ 「帧质量不行」必须与「判据拦的」分得开（2026-09-19）。

    残缺帧走**早退分支**，在此之前 `triggers` 是空的 —— 而残缺帧会
    `invalidate()` 清掉锚点，持续残缺就**永远攒不满 `stationary_min_sec`**，
    该通道所有车都测不出来。实测残缺帧占比 10% 时判稳率只剩 14%，30% 时为 0。
    这是本项目最容易静默失效的一条链，而它原本在告警里显示成「无触发记录」，
    **最严重的故障恰好没有归因**。
    """

    def setUp(self):
        self.clock = ObservationClock(self)
        rng = np.random.default_rng(37)
        self.car = np.column_stack((rng.uniform(-1, 1, 4000),
                                    rng.uniform(-5, 5, 4000),
                                    rng.uniform(-3.7, -2, 4000))).astype(np.float32)

    def detector(self):
        return StabilityDetector(vehicle_threshold=3000, ground_z=-5.23, count_ratio=0)

    def test_incomplete_frames_are_named(self):
        detector = self.detector()
        seen = set()
        for i in range(30):
            r = self.clock.detect(detector, Frame(self.car), trusted=i % 3 != 0, dt=.1)
            seen.update(r.triggers)
        self.assertIn("frame_incomplete", seen, "残缺帧必须能归因，否则故障时是一片空白")

    def test_non_finite_and_sparse_are_distinguishable(self):
        bad = self.car.copy()
        bad[0, 0] = np.nan
        self.assertEqual(("frame_nonfinite",),
                         self.clock.detect(self.detector(), Frame(bad), dt=.1).triggers)
        # 「车辆观测不足」这条分支只在 vehicle_threshold < 3 时够得到 ——
        # 否则点数掉到 3 以下早就被"无车"早退拦掉了。这里把前提摆出来，
        # 免得后人以为它在正常配置下也会触发。
        detector = StabilityDetector(vehicle_threshold=2, ground_z=-5.23, count_ratio=0)
        self.clock.detect(detector, Frame(self.car), dt=.1)
        two = self.car[:2].copy()
        self.assertEqual(("vehicle_sparse",),
                         self.clock.detect(detector, Frame(two), dt=.1).triggers)

    def test_empty_lane_is_not_tallied(self):
        """车道空着是正常工况，不许刷进触发统计。"""
        detector = self.detector()
        empty = self.car.copy()
        empty[:, 2] = -5.2
        self.assertEqual((), self.clock.detect(detector, Frame(empty), dt=.1).triggers)


class TestBoundedTranslationConfirmation(unittest.TestCase):
    """业务真值：允许微动与有界观测误差，疑似起步的当前帧始终禁测。"""
    setUp = TestWorkerContinuousStandstill.setUp
    f = TestWorkerContinuousStandstill.f

    def test_common_mode_noise_and_allowed_creep_fit_measurement_window(self):
        for hz, amplitude in ((10, .0015), (20, .00075)):
            for speed in (0., .005):
                with self.subTest(hz=hz, speed=speed):
                    self.worker.force_new_session()
                    self.results.clear()
                    started = self.clock.mono
                    rng = np.random.default_rng(73)
                    for i in range(10 * hz):
                        offset = [amplitude if i % 2 else -amplitude, speed * i / hz, 0.]
                        cloud = self.xyz + offset + rng.normal(0., .002, self.xyz.shape)
                        self.clock.tick(self.worker, Frame(cloud, packets=900), dt=1. / hz)
                        if self.results:
                            break
                    self.assertTrue(self.results, self.worker.st.stability)
                    self.assertLess(self.results[0][1] - started, 8.)

    def test_late_three_mm_spike_defers_only_unconfirmed_frame(self):
        started = self.clock.mono
        for _ in range(48):
            self.clock.tick(self.worker, self.f())
        previous_start = self.worker._settle_t0
        self.assertIsNotNone(previous_start)
        self.clock.tick(self.worker, Frame(self.xyz + [.003, 0, 0], packets=900))
        self.assertTrue(self.worker.st.stability["motion_pending"])
        self.assertFalse(self.worker.st.stability["stable"])
        self.assertFalse(self.results)
        self.assertEqual(previous_start, self.worker._settle_t0)
        self.assertFalse(self.worker._recent_frames)
        for _ in range(20):
            self.clock.tick(self.worker, self.f())
            if self.results:
                break
        self.assertTrue(self.results)
        self.assertLess(self.results[0][1] - started, 7.)

    def test_step_noise_with_allowed_creep_uses_net_speed_confirmation(self):
        # 每侧保持两帧；3毫米横跳叠加两帧1毫米纵移，净位移超过单步抖动预算。
        # 但实际净速度仍低于门槛，不能因此把允许的5毫米/秒微动永久挡住。
        started = self.clock.mono
        for i in range(100):
            offset = [.0015 if (i // 2) % 2 else -.0015, .005 * i * .1, 0.]
            self.clock.tick(self.worker, Frame(self.xyz + offset, packets=900), dt=.1)
            if self.results:
                break
        self.assertTrue(self.results, self.worker.st.stability)
        self.assertLess(self.results[0][1] - started, 8.)

    def test_small_starting_steps_cannot_measure_and_continued_motion_revokes(self):
        # 20Hz起步前两帧仍可解释为误差加允许微动，均禁测；第三帧超预算撤销。
        for _ in range(102):
            self.clock.tick(self.worker, self.f(), dt=.05)
            if self.worker._settle_t0 is not None and self.clock.mono - self.worker._settle_t0 >= 2.9:
                break
        self.assertFalse(self.results)
        self.clock.tick(self.worker, self.f(.0025), dt=.05)
        self.assertTrue(self.worker.st.stability["motion_pending"])
        self.assertFalse(self.results)
        self.clock.tick(self.worker, self.f(.005), dt=.05)
        self.assertTrue(self.worker.st.stability["motion_pending"])
        self.assertFalse(self.results)
        self.clock.tick(self.worker, self.f(.0075), dt=.05)
        self.assertFalse(self.worker.st.stability["motion_pending"])
        self.assertFalse(self.worker.st.stability["stable"])
        self.assertIsNone(self.worker._settle_t0)
        self.assertFalse(self.worker._raw_shots)
        self.assertFalse(self.results)

    def test_noise_parallel_to_allowed_creep_fits_measurement_window(self):
        for hz, amplitude in ((10, .0015), (20, .00075)):
            for hold in (1, 2):
                with self.subTest(hz=hz, hold=hold):
                    self.worker.force_new_session()
                    self.results.clear()
                    started = self.clock.mono
                    rng = np.random.default_rng(43)
                    for i in range(10 * hz):
                        noise = amplitude if (i // hold) % 2 else -amplitude
                        offset = [0., .005 * i / hz + noise, 0.]
                        cloud = self.xyz + offset + rng.normal(0., .002, self.xyz.shape)
                        self.clock.tick(self.worker, Frame(cloud, packets=900), dt=1. / hz)
                        if self.results:
                            break
                    self.assertTrue(self.results, self.worker.st.stability)
                    self.assertLess(self.results[0][1] - started, 8.)

    def test_pending_cannot_survive_invalid_observation_or_authorize_baseline(self):
        for fault in ("duplicate", "partial", "gap"):
            with self.subTest(fault=fault):
                self.worker.force_new_session()
                self.results.clear()
                self.clock.settle(self.worker, self.f())
                self.clock.tick(self.worker, Frame(self.xyz + [.003, 0, 0], packets=900))
                self.assertTrue(self.worker.st.stability["motion_pending"])
                # 显式给车牌：确保是因"观测无效"被拒，而不是因"读不到牌"空转通过
                self.assertFalse(self.worker.capture_baseline("宁A5548Q")[0])
                if fault == "gap":
                    self.clock.tick(self.worker, None, dt=.8)
                elif fault == "partial":
                    self.clock.tick(self.worker, self.f(packets=450))
                else:
                    duplicate = self.worker.last_frame
                    self.clock.advance(.1)
                    self.worker.source.read = lambda: duplicate
                    self.worker._tick()
                self.assertIsNone(self.worker._settle_t0)
                self.assertIsNone(self.worker.stability._translation_pending)
                self.assertFalse(self.worker._raw_shots)
                self.assertFalse(self.results)

    def test_pending_has_no_timeout_release_and_large_reversal_is_not_noise(self):
        detector = self.worker.stability
        self.clock.hold(detector, self.f(), dt=.1, steps=30)
        self.assertTrue(self.clock.detect(detector, self.f(.003), dt=.1).motion_pending)
        late = self.clock.detect(detector, self.f(.003), dt=.6)
        self.assertFalse(late.stable)
        self.assertFalse(late.jitter_confirmed)
        self.clock.hold(detector, self.f(), dt=.1, steps=30)
        self.assertTrue(self.clock.detect(detector, self.f(.003), dt=.1).motion_pending)
        jumped = self.clock.detect(detector, self.f(-.003), dt=.1)
        self.assertFalse(jumped.stable)
        self.assertFalse(jumped.jitter_confirmed)

    def test_jitter_configuration_rejects_invalid_budget(self):
        for value in (0., float("nan"), .03, .04):
            with self.subTest(value=value), self.assertRaises(ValueError):
                StabilityDetector(stationary_jitter_m=value)
