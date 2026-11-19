#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tests/test_field_fixes.py — 钉住「现场前审计」这一轮修掉的缺陷

每个用例对应一个**实测复现过**的现场故障。改回旧实现时这些用例必须红。
"""
from __future__ import annotations

import io
import os
import sys
import threading
import unittest

import numpy as np

from ._helpers import ctx                      # noqa: F401  （顺带把项目根加进 sys.path）
from . import _helpers as H
from app.settings import load_calib, load_devices, load_settings
from app.service import ChannelWorker, EventBus
from app.pointcloud import Frame, make_synthetic_box, normalize_roi, apply_roi
from app.geometry import fit_plane_pca, correct_heading
from app.detect import (detect_empty, detect_loaded, check_dims, compute_volume,
                        EmptyBaseline, StabilityDetector, StabilityState)


def _calib():
    return load_calib("config/config.json")


def _scene(cargo_h=0.0, cw=2.2, cl=11.0, n=60000, seed=0):
    """俯视雷达看到的车厢：地板 z=-5、两侧壁、可选货堆**顶面**（只有表面回波）。"""
    r = np.random.default_rng(seed)
    pts = [np.stack([r.uniform(-1.2, 1.2, n // 2), r.uniform(-6, 6, n // 2),
                     -5.0 + r.normal(0, 0.002, n // 2)], 1)]
    for xv in (-1.2, 1.2):
        k = n // 8
        pts.append(np.stack([np.full(k, xv) + r.normal(0, 0.002, k),
                             r.uniform(-6, 6, k), r.uniform(-5.0, -2.4, k)], 1))
    if cargo_h > 0:
        k = n // 3
        pts.append(np.stack([r.uniform(-cw / 2, cw / 2, k), r.uniform(-cl / 2, cl / 2, k),
                             -5.0 + cargo_h + r.normal(0, 0.002, k)], 1))
    return Frame(np.concatenate(pts).astype(np.float32))


class TestPlaneFitDegenerate(unittest.TestCase):
    """fit_plane_pca 曾把『完美平面』当成退化点集拒掉。"""

    def test_perfect_plane_is_not_degenerate(self):
        # 车厢地板扫得足够平时，float32 下平面残差会下溢到 0。
        # 旧实现 `if w[0] <= 0: return None` 于是返回 None →
        # fit_floor → detect_empty 全线返回 None → 误报「空车检测失败」。
        r = np.random.default_rng(0)
        flat = np.stack([r.uniform(-1.2, 1.2, 5000), r.uniform(-6, 6, 5000),
                         np.full(5000, -5.0)], 1).astype(np.float32)
        p = fit_plane_pca(flat)
        self.assertIsNotNone(p, "完美平面必须拟合成功（w[0]==0 是理想输入，不是退化）")
        self.assertAlmostEqual(abs(p.c), 1.0, places=5)
        self.assertAlmostEqual(p.intercept(2), -5.0, places=4)

    def test_collinear_and_coincident_still_rejected(self):
        line = np.stack([np.linspace(0, 1, 100), np.linspace(0, 2, 100),
                         np.linspace(0, 3, 100)], 1).astype(np.float32)
        self.assertIsNone(fit_plane_pca(line), "共线点集必须被拒")
        self.assertIsNone(fit_plane_pca(np.zeros((50, 3), np.float32)), "重合点集必须被拒")


class TestDimsOrder(unittest.TestCase):
    """dims 曾经是 (高,宽,长)，而 check_dims / to_dict 按 (宽,长,高) 读。"""

    def test_dims_are_x_y_z(self):
        calib = _calib()
        for dx, dy, dz in ((2.4, 12.0, 2.6), (2.2, 10.5, 2.4)):
            res = detect_empty(make_synthetic_box(dx=dx, dy=dy, dz=dz, n=60000), calib)
            self.assertIsNotNone(res)
            gx, gy, gz = res.dims
            self.assertAlmostEqual(gx, dx, delta=0.15, msg=f"dims[0] 应是宽 x={dx}")
            self.assertAlmostEqual(gy, dy, delta=0.30, msg=f"dims[1] 应是长 y={dy}")
            self.assertAlmostEqual(gz, dz, delta=0.30, msg=f"dims[2] 应是高 z={dz}")
            d = res.to_dict()["dims"]
            self.assertAlmostEqual(d["x"], gx, places=3)
            self.assertAlmostEqual(d["y"], gy, places=3)
            self.assertAlmostEqual(d["z"], gz, places=3)

    def test_normal_truck_passes_size_gate(self):
        # 旧实现拿"宽"去比"最小长度 10m"，任何车都报「尺寸不达标：长 2.4 < 10.0」
        calib, dev = _calib(), load_devices("config/devices.json")
        res = detect_empty(make_synthetic_box(dx=2.4, dy=12.0, dz=2.6, n=60000), calib)
        ok, msg = check_dims(res, dev)
        self.assertTrue(ok, f"标准 2.4×12×2.6 车厢不应判尺寸不达标，实际: {msg}")


class TestSideWallReach(unittest.TestCase):
    """配置窗口 ±1m 够不到实车 ±1.2m 的侧壁 → 车宽量小 30%。"""

    def test_wall_window_auto_widens(self):
        calib = _calib()
        res = detect_empty(make_synthetic_box(dx=2.4, dy=12.0, dz=2.6, n=60000), calib)
        self.assertGreater(res.dims[0], 2.2, "车宽不能被量成窗口边缘的 1.7m")
        self.assertIn("放宽", res.plane("left").note + res.plane("right").note,
                      "窗口够不到真实车壁时应标出『自动放宽』，提醒现场去校准")


class TestVehiclePresence(unittest.TestCase):
    """空车道的地面回波曾被判成『有车且已停稳』，把会话永久锁死。"""

    def setUp(self):
        from tests._observations import ObservationClock
        self.clock = ObservationClock(self)

    @staticmethod
    def _ground(i, n=8000):
        base = np.stack([np.random.default_rng(0).uniform(-2, 2, n),
                         np.random.default_rng(1).uniform(-11, 8, n),
                         -5.0 + np.random.default_rng(2).normal(0, 0.01, n)],
                        1).astype(np.float32)
        jit = np.random.default_rng(100 + i).normal(0, 0.003, base.shape).astype(np.float32)
        return Frame(base + jit)

    @staticmethod
    def _truck(n=40000):
        r = np.random.default_rng(7)
        return Frame(np.stack([r.uniform(-1.2, 1.2, n), r.uniform(-6, 6, n),
                               r.uniform(-4.2, -2.5, n)], 1).astype(np.float32))

    def test_empty_lane_stays_idle(self):
        d = StabilityDetector(0.05, 3000, 30.0, ground_z=-5.0, vehicle_min_height_m=0.30)
        for i in range(5):
            r = self.clock.detect(d, self._ground(i))
            self.assertEqual(r.state, StabilityState.IDLE,
                             "只有地面回波时必须是空闲 —— 否则会话永不结束、真车会被漏检")
            self.assertFalse(r.stable)
            self.assertEqual(r.n_vehicle_points, 0)

    def test_real_vehicle_still_detected(self):
        d = StabilityDetector(0.05, 3000, 30.0, ground_z=-5.0, vehicle_min_height_m=0.30)
        self.clock.detect(d, self._truck())
        r = self.clock.hold(d, self._truck())
        self.assertEqual(r.state, StabilityState.STABLE, "真车停稳必须还能测出来")
        self.assertTrue(r.stable)
        self.assertGreater(r.n_vehicle_points, 3000)

    def test_returns_to_idle_after_vehicle_leaves(self):
        d = StabilityDetector(0.05, 3000, 30.0, ground_z=-5.0, vehicle_min_height_m=0.30)
        self.clock.detect(d, self._truck())
        self.clock.detect(d, self._truck())
        self.assertEqual(self.clock.detect(d, self._ground(0)).state, StabilityState.IDLE)


class TestDegradedFrameKeepsSession(unittest.TestCase):
    """★ 一帧突发丢包曾能结束会话，于是同一辆车被测两遍。

    2026-09-15 现场实测（CH4 11:11:02）：

        11:09:47 会话 #1 开始，点数 71584
        11:09:59 存盘 20260915_110959.xyz   ROI 点数 129906
        11:11:02 会话 #1 结束
        11:11:02 会话 #2 开始，点数 66063      <- 同一秒
        11:11:07 存盘 20260915_111107.xyz   ROI 点数 129908
                                            <- 与上一份只差 2 个点，车一动没动

    紧接着 11:12:13 又开了一个会话，那一帧只有 30109 点
    （当天正常的 session_start 点数是 68206~74991）。

    原因：有车判据（n_veh < gate -> reset -> IDLE）排在 trusted 检查
    **前面**。残缺帧缺的是一整块连续扇形，车上的点也跟着少一块 ——
    点数偏少是丢包造成的，却被读成"车开走了"。
    老规矩本来就规定"残缺帧不参与判定"，只是当时只挡住了质心那一路，
    漏了点数这一路。

    后果不是"少测"，而是**样本数虚高**：下游按份数统计车次或算体积会双计。
    """

    def setUp(self):
        from tests._observations import ObservationClock
        self.clock = ObservationClock(self)

    @staticmethod
    def _truck(n=40000, seed=7, packets=450):
        r = np.random.default_rng(seed)
        return Frame(np.stack([r.uniform(-1.2, 1.2, n), r.uniform(-6, 6, n),
                               r.uniform(-4.2, -2.5, n)], 1).astype(np.float32),
                     packets=packets)

    @staticmethod
    def _ground(n=8000, packets=450):
        r = np.random.default_rng(0)
        return Frame(np.stack([r.uniform(-2, 2, n), r.uniform(-11, 8, n),
                               -5.0 + r.normal(0, 0.01, n)], 1).astype(np.float32),
                     packets=packets)

    # 现场值：vehicle_threshold=15000，回差 0.8 -> 退出阈值 12000。
    # 正常一帧车点约 40000，现场那一帧只剩 42%（30109/72000）——
    # 掉到退出阈值以下，判据就翻脸了。
    THRESHOLD = 15000
    GATE = 12000

    def _det(self):
        return StabilityDetector(0.05, self.THRESHOLD, 30.0, ground_z=-5.0,
                                 vehicle_min_height_m=0.30, count_ratio=0.0)

    def test_the_degraded_frame_really_is_below_the_gate(self):
        """先钉住前提：构造的残缺帧点数确实掉到了退出阈值以下。
        否则下面几条会空转（改坏了也照样绿）—— 这个坑当场踩过。"""
        d = self._det()
        self.assertGreater(d.vehicle_points(self._truck()), self.THRESHOLD)
        self.assertLess(d.vehicle_points(self._truck(n=10000)), self.GATE)

    def test_degraded_frame_does_not_end_the_session(self):
        d = self._det()
        self.clock.detect(d, self._truck())
        self.assertEqual(self.clock.hold(d, self._truck()).state, StabilityState.STABLE)
        # 残缺帧：车上的点少了一大块，掉到退出阈值以下
        r = self.clock.detect(d, self._truck(n=10000), trusted=False)
        self.assertNotEqual(r.state, StabilityState.IDLE,
                            "残缺帧的点数偏少是丢包，不是车走了 —— "
                            "判成 IDLE 会结束会话并把同一辆车重测一遍")
        self.assertTrue(r.degraded, "应当标成 degraded 让看板看得见")
        # 下一个完整帧照常接着判，状态没被打断
        self.assertFalse(self.clock.detect(d, self._truck()).stable)
        self.assertTrue(self.clock.hold(d, self._truck()).stable)

    def test_a_complete_frame_still_ends_the_session(self):
        """修的是"残缺帧"，**车真走了必须照旧回到空闲** —— 别把保险焊死。"""
        d = self._det()
        self.clock.detect(d, self._truck())
        self.clock.detect(d, self._truck())
        self.assertEqual(self.clock.detect(d, self._ground()).state, StabilityState.IDLE,
                         "完整帧显示没车，就是没车")

    def test_degraded_then_really_gone_still_returns_to_idle(self):
        """丢包期间车开走了：等第一个完整帧到，照样要回到空闲。"""
        d = self._det()
        self.clock.detect(d, self._truck())
        self.clock.detect(d, self._truck())
        self.clock.detect(d, self._truck(n=10000), trusted=False)
        self.clock.detect(d, self._ground(n=3000), trusted=False)
        self.assertEqual(self.clock.detect(d, self._ground()).state, StabilityState.IDLE)

    def test_idle_lane_is_unaffected(self):
        """还没进入"有车"时不走这条 —— 空车道上的残缺帧仍然是空闲。"""
        d = self._det()
        r = self.clock.detect(d, self._ground(n=3000), trusted=False)
        self.assertEqual(r.state, StabilityState.IDLE)
        self.assertTrue(r.degraded)


class TestStabilityReference(unittest.TestCase):
    """参考帧曾被冻结在『会话第一帧』，导致「车驶入 → 停住」永远判不出停稳。

    旧实现里 `self._ref = c` 只写在 stable 分支中，于是车一进 ROI 就把入场那帧
    的质心钉成参考点；车往前开几米停住后，dev 恒为那几米，状态永远「运动中」
    → 不进沉降窗口、不检测、不落盘，只能等 session_timeout_sec(180s) 翻篇。

    本组用例必须喂**先动后停**的序列 —— 这正是老用例漏掉的地方：
    它们都只喂同一帧两次，参考帧恰好吻合，看不出问题。
    """

    def setUp(self):
        from tests._observations import ObservationClock
        self.clock = ObservationClock(self)

    @staticmethod
    def _truck_at(y0, n=40000, seed=0):
        """停在 y=y0 的车：底板 z=-4.2（高于 ground_z），另加一片不动的地面回波。"""
        rng = np.random.default_rng(seed)
        body = np.stack([rng.uniform(-1.2, 1.2, n),
                         rng.uniform(y0 - 6, y0 + 6, n),
                         np.full(n, -4.2) + rng.normal(0, 0.01, n)], 1)
        ground = np.stack([rng.uniform(-2, 2, 8000), rng.uniform(-11, 8, 8000),
                           np.full(8000, -5.0) + rng.normal(0, 0.01, 8000)], 1)
        return Frame(np.concatenate([body, ground]).astype(np.float32))

    def _det(self):
        return StabilityDetector(0.05, 3000, 30.0, ground_z=-5.0,
                                 vehicle_min_height_m=0.30)

    def test_drive_in_then_stop_becomes_stable(self):
        """★ 核心用例：驶入 8 帧 + 停住若干帧，必须判稳。"""
        d = self._det()
        for k in range(8):                       # 每帧前进 1 m
            r = self.clock.detect(d, self._truck_at(10.0 - k))
            self.assertFalse(r.stable, f"驶入第 {k+1} 帧不该判稳")
        r = None
        for _ in range(8):                       # 连续新帧停住超过2秒
            r = self.clock.detect(d, self._truck_at(2.0))
        self.assertTrue(r.stable,
                        f"车停住后必须判稳，实测 dev={r.deviation:.3f} m —— "
                        f"参考帧被冻结在驶入第一帧了（app/detect.py StabilityDetector）")
        self.assertEqual(r.state, StabilityState.STABLE)
        self.assertLessEqual(r.deviation, 0.05)

    def test_deviation_is_relative_to_previous_frame(self):
        """dev 的语义是『相对上一帧的位移』，不是『相对会话第一帧』。"""
        d = self._det()
        self.clock.detect(d, self._truck_at(10.0))
        self.clock.detect(d, self._truck_at(9.0))            # 走了 1 m
        r = self.clock.detect(d, self._truck_at(8.0))        # 又走 1 m
        self.assertLess(r.deviation, 1.2,
                        "相对上一帧只走了 1 m；若接近 2 m 说明参考帧没跟随")
        self.assertGreater(r.deviation, 0.5)

    def test_moving_again_after_stable_is_detected(self):
        """停稳之后车又动了，必须立刻回到『运动中』（沉降窗口靠它作废本次）。"""
        d = self._det()
        self.clock.detect(d, self._truck_at(2.0))
        self.assertTrue(self.clock.hold(d, self._truck_at(2.0)).stable)
        r = self.clock.detect(d, self._truck_at(3.0))
        self.assertFalse(r.stable)
        self.assertEqual(r.state, StabilityState.BASELINE)

    def test_metric_still_equals_centroid_norm(self):
        """度量本身没变 —— 仍是 ‖质心差‖（受控实验验证到 1e-7 的那个量）。"""
        rng = np.random.default_rng(3)
        base = rng.normal(0, 1, (5000, 3)).astype(np.float32)
        d = StabilityDetector(0.05, 100)
        self.clock.detect(d, Frame(base))
        shift = np.array([0.3, -0.2, 0.1], np.float32)
        r = self.clock.detect(d, Frame(base + shift))
        self.assertAlmostEqual(r.deviation, float(np.linalg.norm(shift)), places=6)


class TestVolume(unittest.TestCase):
    """体积曾按『占据体素』算，对俯视雷达只算出一层壳（真值的 15%）。"""

    def setUp(self):
        self.calib = _calib()
        self.base = EmptyBaseline.from_empty_result(detect_empty(_scene(), self.calib), "t")
        self.assertIsNotNone(self.base)

    def test_volume_within_10_percent(self):
        for cw, cl, ch in ((2.2, 11.0, 1.4), (2.0, 9.0, 0.8), (2.2, 11.0, 2.0)):
            lr = detect_loaded(_scene(cargo_h=ch, cw=cw, cl=cl, seed=1), self.calib)
            v = compute_volume(self.base, lr)
            truth = cw * cl * ch
            self.assertTrue(v.ok)
            self.assertLess(abs(v.volume_m3 - truth) / truth, 0.10,
                            f"{cw}×{cl}×{ch}: 真值 {truth:.2f} 实测 {v.volume_m3:.2f}")

    def test_empty_vehicle_is_near_zero(self):
        v = compute_volume(self.base, detect_loaded(_scene(seed=2), self.calib))
        self.assertTrue(v.ok, v.text)
        self.assertTrue(np.isfinite(v.volume_m3))
        self.assertLess(v.volume_m3, 1.0, "空车体积应接近 0")

    def test_no_baseline_reports_clearly(self):
        v = compute_volume(None, detect_loaded(_scene(cargo_h=1.4), self.calib))
        self.assertFalse(v.ok)
        self.assertIn("基线", v.message)

    def test_baseline_from_empty_cloud_returns_none(self):
        # 旧实现直接 res.cloud.min(axis=0) → 空点云 ValueError 崩掉
        self.assertIsNone(EmptyBaseline.from_empty_result(None))


class TestRoiNormalization(unittest.TestCase):
    """轴区间写反时曾静默裁成空点云。"""

    def test_reversed_axis_range_is_normalized(self):
        lo, hi = normalize_roi(((-2, 2), (-11, 8), (-0.75, -6)))   # z 写反
        self.assertAlmostEqual(float(lo[2]), -6.0)
        self.assertAlmostEqual(float(hi[2]), -0.75)

    def test_reversed_roi_keeps_points(self):
        f = Frame(np.array([[0, 0, -5.0], [0, 0, -3.0], [0, 0, -20.0]], np.float32))
        self.assertEqual(apply_roi(f, ((-2, 2), (-11, 8), (-0.75, -6))).n, 2)


class TestHeadingDtype(unittest.TestCase):
    """correct_heading 的出口 dtype 曾随『这一帧转没转』变化。"""

    def test_dtype_is_stable(self):
        r = np.random.default_rng(0)
        straight = np.stack([r.uniform(-1, 1, 500), r.uniform(-6, 6, 500),
                             np.full(500, -5.0)], 1).astype(np.float32)
        rot = straight @ np.array([[0.9962, -0.0872, 0],
                                   [0.0872, 0.9962, 0], [0, 0, 1]]).T
        a, _, _ = correct_heading(straight)
        b, _, _ = correct_heading(rot.astype(np.float32))
        self.assertEqual(a.dtype, b.dtype, "转与不转必须给出同样的 dtype")
        self.assertEqual(a.dtype, np.float64)


class TestWorkerShutdown(unittest.TestCase):
    """ChannelWorker 曾用 self._stop 覆盖 Thread._stop()，join() 必抛 TypeError。"""

    def test_thread_stop_not_shadowed(self):
        from app.service import ChannelWorker
        self.assertNotIn("_stop", ChannelWorker.__dict__,
                         "不要在 ChannelWorker 上定义 _stop —— 会覆盖 Thread._stop()")
        t = threading.Thread(target=lambda: None)
        t.start()
        t.join(timeout=2.0)
        self.assertFalse(t.is_alive())


class TestCaptureBindIp(unittest.TestCase):
    """★ 多雷达部署下 bind_ip 留空时，抓包 socket 必须绑 0.0.0.0。

    历史坑（2026-09-13 现场踩到）：cli/capture.py 原写成
        args.bind or ch.lidar.bind_ip or ch.lidar.ip
    而本项目多雷达约定是「各通道 bind_ip 一律留空，由单一 socket 绑 0.0.0.0
    再按源 IP 分流」。于是它退到了 **雷达自己的** IP（如 CH1 的 198.51.100.1），
    本机并不持有该地址 → bind 必失败，现象是刚启动就报
    WinError 10049，极易误判成"端口被占"而白折腾。
    """

    @staticmethod
    def _capture_mod():
        import importlib.util
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cli_dir = os.path.join(root, "cli")
        if cli_dir not in sys.path:
            sys.path.insert(0, cli_dir)
        spec = importlib.util.spec_from_file_location(
            "capture_under_test", os.path.join(cli_dir, "capture.py"))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod

    @staticmethod
    def _ch(bind_ip, lidar_ip):
        class _Lidar:
            pass

        class _Chan:
            pass
        lidar = _Lidar()
        lidar.bind_ip = bind_ip
        lidar.ip = lidar_ip
        ch = _Chan()
        ch.lidar = lidar
        return ch

    def test_cli_bind_wins(self):
        m = self._capture_mod()
        self.assertEqual(m._pick_bind_ip("10.1.2.3", self._ch("", "198.51.100.1")),
                         "10.1.2.3")

    def test_channel_bind_used_when_no_cli(self):
        m = self._capture_mod()
        self.assertEqual(m._pick_bind_ip("", self._ch("198.51.100.15", "198.51.100.1")),
                         "198.51.100.15")

    def test_never_falls_back_to_lidar_ip(self):
        """★ 核心回归：bind_ip 留空时必须是 0.0.0.0，绝不能是雷达自己的 IP"""
        m = self._capture_mod()
        got = m._pick_bind_ip("", self._ch("", "198.51.100.1"))
        self.assertEqual(got, "0.0.0.0")
        self.assertNotEqual(got, "198.51.100.1")

    def test_real_channels_config_all_bind_all_interfaces(self):
        """跑真实 config/channels.json：4 条通道 bind_ip 全空 → 全部落到 0.0.0.0"""
        m = self._capture_mod()
        from app.settings import load_settings
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        st = load_settings(os.path.join(root, "config", "channels.json"))
        chans = st.channels
        self.assertTrue(chans, "channels.json 里应有通道")
        for c in chans:
            self.assertEqual(m._pick_bind_ip("", c), "0.0.0.0",
                             f"{c.id} 的 bind_ip 留空时应绑 0.0.0.0")


class TestSettleWindow(unittest.TestCase):
    """停稳后要再等 settle_sec 秒（沉降）才测量落盘，窗口内又动就作废。"""

    def setUp(self):
        from tests._observations import ObservationClock
        self.clock = ObservationClock(self)

    def _worker(self, settle=0.6, accumulate=False):
        from app.service import EventBus, ChannelWorker
        from app.settings import load_settings
        st = load_settings("")
        ch = [c for c in st.channels if c.id == "CH1"][0]
        ch.mode = "synthetic"
        bus = EventBus()
        w = ChannelWorker(ch, bus, st.plate, save_override=False)
        w.settle_sec = settle
        w.settle_accumulate = accumulate
        return w, bus, bus.subscribe()

    @staticmethod
    def _ground(n=8000):
        return np.stack([np.random.default_rng(0).uniform(-2, 2, n),
                         np.random.default_rng(1).uniform(-11, 8, n),
                         -5.0 + np.random.default_rng(2).normal(0, .01, n)], 1).astype(np.float32)

    def _truck(self, shift=0.0, seed=5, n=40000):
        r = np.random.default_rng(seed)
        car = np.stack([r.uniform(-1.2, 1.2, n), r.uniform(-6, 6, n),
                        r.uniform(-4.2, -2.5, n)], 1).astype(np.float32) + shift
        return Frame(np.concatenate([self._ground(), car]))

    def _tick(self, w, q, frame):
        self.clock.tick(w, frame)
        kinds = [e["kind"] for e in list(q)]
        q.clear()
        return kinds

    def test_no_result_before_window_elapses(self):
        import time
        w, _, q = self._worker(settle=0.6)
        self.clock.settle(w, self._truck())
        kinds = [e["kind"] for e in q]
        q.clear()
        self.assertNotIn("result", kinds, "沉降窗口没走完就落盘了")
        self.assertTrue(w.st.stability.get("settling"), "应标记为沉降中")
        for _ in range(6):
            self._tick(w, q, self._truck())
        kinds = self._tick(w, q, self._truck())       # 窗口到期
        self.assertIn("result", kinds, "沉降窗口结束后应产出一次 result")
        self.assertFalse(w.st.stability.get("settling"))

    def test_movement_during_window_aborts(self):
        w, _, q = self._worker(settle=5.0)
        self.clock.settle(w, self._truck())
        self.assertTrue(w.st.stability.get("settling"))
        kinds = self._tick(w, q, self._truck(shift=np.array([0.4, 0, 0], np.float32)))
        self.assertIn("warn", kinds, "窗口内车辆移动应告警并作废")
        self.assertNotIn("result", kinds, "余振未停不能落盘")
        self.assertFalse(w.st.stability.get("settling"))
        self.assertIsNone(w._settle_t0)

    def test_accumulate_merges_frames(self):
        import time
        w, _, q = self._worker(settle=0.5, accumulate=True)
        self.clock.settle(w, self._truck())
        n_one = self._truck().n
        for _ in range(7):
            self.clock.tick(w, self._truck())
        r = [e for e in list(q) if e["kind"] == "result"]
        self.assertTrue(r, "累积模式也要产出 result")
        self.assertGreater(r[-1]["n_points"], n_one,
                           "累积模式应合并出比单帧更密的点云")

    def test_settle_zero_is_old_behaviour(self):
        w, _, q = self._worker(settle=0.0)
        kinds = []
        for _ in range(30):
            kinds += self._tick(w, q, self._truck())
        self.assertIn("result", kinds, "settle_sec=0 仅取消额外沉降，连续停稳后应测量")


class TestCarriageYTrim(unittest.TestCase):
    """★ 短车厢会把地面带进 ROI —— 拟合前按车厢收窄 y 把它甩掉。

    2026-09-15 实测上午 53 辆：
      · 短厢（牵引车 + 短高栏挂，5.25~6.25 m）**6/6** 有地面点，
        min_z 到 -5.90（地面 -5.23）；长厢（7.25~11.75 m）**0/42**。
      · 判定链停在"未见厢底平面"的比例：短厢 50% vs 长厢 11%。
    原因：短厢只占 ROI 19 m 窗口的三成，车厢前方留出 6 m 以上空车道，
    雷达在天底附近直视地面 -> 底面种子（z 最低 30%）被地面点占掉。

    开关：`detect_empty.floor.y_trim_gap_m`，**0 = 关闭**（行为逐位不变）。
    实测（2026-09-16）：开关两种状态 `cli/verify.py` 都是 10/10；
    53 辆里只在那 6 辆短厢上触发，长厢 0/47 结论变化，车厢零切割。
    """

    P = dict(bin_m=0.5, gap_m=1.5, occupy=0.02, below_m=1.0)

    @staticmethod
    def _slab(y0, y1, z, n=4000, seed=0, x0=-1.2, x1=1.2):
        r = np.random.default_rng(seed)
        return np.column_stack([r.uniform(x0, x1, n), r.uniform(y0, y1, n),
                                np.full(n, z) + r.normal(0, 0.01, n)])

    def _win(self, a, **kw):
        from app.detect import carriage_y_window
        p = dict(self.P)
        p.update(kw)
        return carriage_y_window(np.asarray(a, np.float64), p["bin_m"], p["gap_m"],
                                 p["occupy"], p["below_m"])

    def test_disabled_by_default(self):
        """gap_m=0 -> 一律返回 None，功能整个关掉。"""
        a = np.concatenate([self._slab(-4, 2, -3.5), self._slab(4, 8, -5.3, n=300)])
        self.assertIsNone(self._win(a, gap_m=0.0))

    def test_drops_the_ground_blob(self):
        """车厢 + 前方一段明显更低的地面 -> 地面被甩掉。"""
        a = np.concatenate([self._slab(-4, 2, -3.5, n=40000),
                            self._slab(4.5, 8, -5.3, n=800, seed=1)])
        w = self._win(a)
        self.assertIsNotNone(w, "该收窄却没收")
        self.assertLessEqual(w[0], -4.0 + 0.5)   # 分格量化，允许半格
        self.assertLess(w[1], 4.5, "地面那一段应当被排除在窗口外")

    def test_keeps_a_gap_split_carriage(self):
        """★ 保命用例：车厢被遮挡断成两段，**不许**把后半段当地面丢掉。

        2026-09-16 实测：门槛设成 0.5 m 时，CH2_112025 的车厢后段
        （只比主体低 0.52 m）被整段丢掉，切掉车厢 0.76 m。
        真正的地面比主体低 1.73~3.62 m，所以门槛取 1.0。
        """
        # 断口 2.0 m（> gap_m=1.5）-> 确实断成两段；后段只比主体低 0.5 m
        a = np.concatenate([self._slab(-2, 3, -3.5, n=40000),
                            self._slab(-6, -4, -4.0, n=3000, seed=2)])
        self.assertIsNotNone(self._win(a, below_m=0.3),
                             "前提：门槛够低时确实会丢段，否则本用例空转")
        w = self._win(a)
        if w is not None:
            self.assertLessEqual(w[0], -6.0 + 0.5,
                                 "车厢后段被当成地面丢掉了 —— 车厢被切会直接少算体积")

    def test_single_run_is_left_alone(self):
        """整片车厢连成一段时不收窄 —— 没什么可丢的。"""
        self.assertIsNone(self._win(self._slab(-6, 6, -3.5, n=40000)))

    def test_internal_gap_does_not_split(self):
        """★ 10 份回归基线每一份都有 0.5~2.5 m 的内部断口，不能据此切短。"""
        a = np.concatenate([self._slab(-6, -1.5, -3.5, n=20000),
                            self._slab(0.2, 6, -3.5, n=20000, seed=3)])
        w = self._win(a)
        self.assertIsNone(w, "1.7 m 的内部断口不该把车厢切成两段")

    def test_a_higher_blob_is_kept(self):
        """比主体**高**的东西（门架/驾驶室）不是地面，不丢 —— 丢了等于白改判据。"""
        a = np.concatenate([self._slab(-4, 2, -3.5, n=40000),
                            self._slab(5, 7, -1.5, n=2000, seed=4)])
        w = self._win(a)
        if w is not None:
            self.assertGreaterEqual(w[1], 7.0 - 0.5)

    def test_off_is_bit_identical(self):
        """★ 关闭时判定结果必须与加这个功能之前**逐位一致**。"""
        import copy as _copy
        from app.detect import detect_empty, evaluate_empty
        from app.settings import load_devices
        calib = _calib()
        off = _copy.deepcopy(calib)
        off.floor.y_trim_gap_m = 0.0
        dev = load_devices("config/devices.json")
        f = _scene(cargo_h=1.0)
        for c in (calib, off):
            r = detect_empty(f, c)
            self.assertIsNotNone(r)
        r0 = detect_empty(f, off)
        self.assertEqual(len(r0.cloud), f.n,
                         "关闭时点云一个点都不该少")


class TestCarriageYWindowRejectsNonFinite(unittest.TestCase):
    """★ 非有限值要优雅返回 None，不能抛 "arange: cannot compute length"。

    2026-09-22 审查发现（docs/52 第三节）：`carriage_y_window` 少了这道守卫，
    而同模块做同类事情的 `carriage_box` 有。全 NaN 点云会崩在
    `np.arange(lo, hi + bin_m, bin_m)` 上，报错里**没有任何线索**指向
    "点云含非有限值"。

    服务侧走不到这里（停稳层对非有限帧早退并打 frame_nonfinite 标记），
    但 `cli/replay.py` `cli/inspect_cloud.py` `cli/baseline.py` `cli/verify.py`
    都是**直接对文件**调 `detect_empty` 的 —— 一份损坏的 .xyz 就会踩到。
    """

    P = dict(bin_m=0.5, gap_m=1.5, occupy=0.02, below_m=1.0)

    def _win(self, a):
        from app.detect import carriage_y_window
        return carriage_y_window(np.asarray(a, np.float64), self.P["bin_m"],
                                 self.P["gap_m"], self.P["occupy"], self.P["below_m"])

    def test_all_nan_returns_none_instead_of_raising(self):
        self.assertIsNone(self._win(np.full((200, 3), np.nan)))

    def test_infinite_and_partially_nan_are_rejected(self):
        r = np.random.default_rng(7)
        good = np.column_stack([r.uniform(-1.2, 1.2, 400), r.uniform(-5, 5, 400),
                                np.full(400, -4.0)])
        for name, bad in (
                ("含 +inf", np.vstack([good, [[0.0, np.inf, -4.0]]])),
                ("含 -inf", np.vstack([good, [[0.0, -np.inf, -4.0]]])),
                ("z 含 NaN", np.vstack([good, [[0.0, 1.0, np.nan]]])),
        ):
            with self.subTest(name):
                self.assertIsNone(self._win(bad), "%s 应当返回 None" % name)

    def test_carriage_box_already_behaved_this_way(self):
        """对照组：守卫是照着它补的，两个函数对同一份坏输入必须一致。"""
        from app.detect import carriage_box
        bad = np.full((400, 3), np.nan)
        self.assertIsNone(carriage_box(bad))
        self.assertIsNone(self._win(bad))

    def test_finite_input_is_untouched(self):
        """★ 守卫**只**拦非有限值 —— 10 份回归基线一份都不该因此改变行为。

        基线上 `carriage_y_window` 本来就全部返回 None（CLAUDE.md §7d），
        所以这里用一个会真正触发收窄的合成场景，确认它仍然收窄。
        """
        r = np.random.default_rng(3)
        def slab(y0, y1, z, n):
            return np.column_stack([r.uniform(-1.2, 1.2, n), r.uniform(y0, y1, n),
                                    np.full(n, z) + r.normal(0, .01, n)])
        # 车厢（高）+ 远处一段明显更低的地面（该被甩掉）
        scene = np.vstack([slab(-3.0, 3.0, -3.5, 6000), slab(6.0, 9.0, -5.2, 3000)])
        win = self._win(scene)
        self.assertIsNotNone(win, "这一场景本应触发收窄，前提没成立就证明不了什么")
        self.assertLess(win[1], 5.0, "远处那段地面应当被甩掉，实得 %r" % (win,))


class TestStationaryWarnOnlyBeforeResult(unittest.TestCase):
    """★ 「车辆停稳超过 30s 仍未稳定」曾 100% 是误报。

    timed_out() 量的是"车进来多久了"（_entered 在车出现那一刻置位），
    不是"停稳了多久"。现场车辆中位停留 37 s > 阈值 30 s，于是**车一开走
    就必然触发**：状态离开 STABLE + 计时早已超时。

    2026-09-15 全天 53 次告警逐条比对时刻：
        告警距 session_end 中位 2 s、最大 4 s —— 53/53 都在车走的前几秒。
    一次真正的"停着却判不稳"都没有。

    加的条件是"本会话还没测出结果"：车进来 30 s 还没测出来才值得报。
    """

    def setUp(self):
        from tests._observations import ObservationClock
        self.clock = ObservationClock(self)

    def _worker(self):
        from app.service import EventBus, ChannelWorker
        from app.settings import load_settings
        st = load_settings("")
        ch = [c for c in st.channels if c.id == "CH1"][0]
        ch.mode = "synthetic"
        bus = EventBus()
        w = ChannelWorker(ch, bus, st.plate, save_override=False)
        w.settle_sec = 0.0
        return w, bus.subscribe()

    @staticmethod
    def _age(w, sec=60.0):
        """把"车进来的时刻"往前推，模拟车已经在道上待了这么久。
        不这么做的话，告警会在**驶入那一拍**就先响掉，_warned 置位，
        后面测什么都测不出来（这个空转坑当场踩过）。"""
        import time as _t
        w.stability._entered = _t.monotonic() - sec
        w._warned = False

    @staticmethod
    def _ground(n=8000):
        r = np.random.default_rng(0)
        return np.stack([r.uniform(-2, 2, n), r.uniform(-11, 8, n),
                         -5.0 + r.normal(0, .01, n)], 1).astype(np.float32)

    def _truck(self, shift=0.0, n=40000):
        r = np.random.default_rng(5)
        car = np.stack([r.uniform(-1.2, 1.2, n), r.uniform(-6, 6, n),
                        r.uniform(-4.2, -2.5, n)], 1).astype(np.float32) + shift
        return Frame(np.concatenate([self._ground(), car]))

    def _tick(self, w, q, frame):
        self.clock.tick(w, frame)
        msgs = [str(e.get("message", "")) for e in list(q)
                if e["kind"] == "warn"]
        q.clear()
        return msgs

    def test_no_warning_once_the_vehicle_was_measured(self):
        w, q = self._worker()
        for _ in range(30):
            self._tick(w, q, self._truck())
        self.assertGreater(w._detected_this_session, 0, "前提：本会话已经测过了")
        self._age(w)                                     # 车已经待了 60 s
        # 车开始移动（还没少到判成无车）—— 这正是现场那 53 次的时刻
        msgs = self._tick(w, q, self._truck(shift=0.5))
        self.assertTrue(w.stability.timed_out(), "前提：计时确实已经超了")
        self.assertNotEqual(w.st.stability.get("state"), 2,
                            "前提：这一拍确实已经离开了 STABLE")
        self.assertFalse([m for m in msgs if "停稳超过" in m],
                         "测都测完了，车要走了才报「停稳超时」，53/53 是误报")

    def test_still_warns_when_nothing_was_measured(self):
        """★ 真正该报的那种：车进来一直不停稳，从来没测出结果。"""
        w, q = self._worker()
        self._tick(w, q, self._truck())                  # 驶入
        self._age(w)                                     # 已经待了 60 s
        msgs = self._tick(w, q, self._truck(shift=1.2))  # 还在动
        self.assertEqual(w._detected_this_session, 0, "前提：本会话一次都没测成")
        self.assertTrue([m for m in msgs if "停稳超过" in m],
                        "车进来 30s 还没测出结果 —— 这条必须照旧报")


class TestDuplicateOfMark(unittest.TestCase):
    """★ session_timeout 强开的会话若再落一次盘，那是**同一辆车**被重测。

    2026-09-15 下午实测（CH4）：

        15:28:30 会话#1 开始 -> 15:28:38 存盘 112386 点
        15:31:30 [warn] 会话 1 超过 180s，强制开新会话
        15:31:34          存盘 112770 点     <- 同一辆车又测了一遍（差 0.3%）

    4 份存盘其实只有 2 辆车，后果是**样本数虚高**，下游按份数统计车次
    或算体积会双计。

    判据不靠比几何量，而是**状态机的直接推论**：车道一旦回到空闲就会把
    _last_save_name 清掉；既然没清掉而会话又是强开的，车就没走过。

    ▲ 只打标、不拦截 —— 超时保险本身一个字没动，**阈值也别调**。
      上面那两次超时，现场确认是在单独测试其它功能时把车故意停住的，
      **超时触发得对**（2026-09-16 现场答复）；迄今没有一次误触发。
      而它是 2026-09-13"会话永不结束导致后面的车全部被静默跳过"那次事故的
      唯一兜底，调大 = 延长那类事故的静默期。
      要解决的只是"重测出来的那一份是重复数据"，与超时该不该触发是两回事。
    """

    def setUp(self):
        from tests._observations import ObservationClock
        self.clock = ObservationClock(self)

    def _worker(self, tmp):
        from app.service import EventBus, ChannelWorker
        from app.settings import load_settings
        st = load_settings("")
        ch = [c for c in st.channels if c.id == "CH1"][0]
        ch.mode = "synthetic"
        ch.abs_output_dir = tmp
        bus = EventBus()
        w = ChannelWorker(ch, bus, st.plate, save_override=True)
        w.settle_sec = 0.0
        w.raw_frames = 0
        return w

    @staticmethod
    def _ground(n=8000):
        r = np.random.default_rng(0)
        return np.stack([r.uniform(-2, 2, n), r.uniform(-11, 8, n),
                         -5.0 + r.normal(0, .01, n)], 1).astype(np.float32)

    def _truck(self, n=40000):
        r = np.random.default_rng(5)
        car = np.stack([r.uniform(-1.2, 1.2, n), r.uniform(-6, 6, n),
                        r.uniform(-4.2, -2.5, n)], 1).astype(np.float32)
        return Frame(np.concatenate([self._ground(), car]))

    def _tick(self, w, frame):
        self.clock.tick(w, frame)

    def _metas(self, tmp):
        import glob
        import json as _json
        out = []
        for f in sorted(glob.glob(os.path.join(tmp, "archives-v1", "**", "detection.xyz.meta.json"), recursive=True)):
            with io.open(f, encoding="utf-8") as fp:
                out.append(_json.load(fp))
        return sorted(out, key=lambda row: row["ts"])

    def test_forced_session_marks_the_second_save(self):
        import shutil
        import tempfile
        import time as _t
        tmp = tempfile.mkdtemp(prefix="dupmark_")
        try:
            w = self._worker(tmp)
            self._tick(w, self._truck())            # 驶入
            for _ in range(30):
                self._tick(w, self._truck())
            from tests.test_result_finalizer import await_result
            await_result(w)
            self.assertEqual(len(self._metas(tmp)), 1)
            self.assertNotIn("duplicate_of", self._metas(tmp)[0],
                             "第一份不该有 duplicate_of")
            # 让会话超时**一次**：车一直没走。
            # ★ 触发完必须把阈值调回去 —— 否则每一拍都会再超时一次，
            #   每次都 stability.reset()，车永远回不到"已停稳"，
            #   于是第二份盘永远落不下来。机器慢的时候才复现，
            #   跑单模块看不出来（当场踩过：单跑绿、全跑红）。
            w.session_timeout = 0.01
            w._session_t0 = _t.monotonic() - 10.0
            self._tick(w, self._truck())            # 这一拍强开新会话
            self.assertTrue(w._session_forced, "前提：会话确实是被超时强开的")
            w.session_timeout = 180.0
            w._session_t0 = _t.monotonic()
            self._tick(w, self._truck())            # 重建参考帧
            for _ in range(30):
                self._tick(w, self._truck())
            await_result(w, count=2)
            ms = self._metas(tmp)
            self.assertEqual(len(ms), 2, "超时之后应当又落了一份")
            self.assertIn("duplicate_of", ms[1],
                          "超时强开 + 车道从未回到空闲 = 同一辆车，必须打标")
            self.assertEqual(ms[1]["duplicate_of"], ms[0]["measurement_id"])
        finally:
            w.stop()
            self.assertTrue(w.close_results(10.))
            w.source.close()
            shutil.rmtree(tmp, ignore_errors=True)

    def test_incomplete_frame_right_after_a_forced_session_keeps_the_mark(self):
        """★ 强开会话后紧跟一帧残缺帧，标记也不许丢（2026-09-24 现场 CH4 10:07:12）。

        强开会话会 reset 检测器；它要等一帧可信的帧才重新确认"有车"。
        这之间来一帧残缺帧，检测器报 IDLE（degraded）—— 原来服务照单全收、
        当成车走了，清掉 _last_save_name / _session_forced，于是同一辆车的
        下一份测量没带 duplicate_of。当时现场正在大量丢包（残缺帧 38~45%）。
        """
        import shutil
        import tempfile
        import time as _t
        tmp = tempfile.mkdtemp(prefix="dupmark_")
        try:
            w = self._worker(tmp)
            self._tick(w, self._truck())
            for _ in range(30):
                self._tick(w, self._truck())
            from tests.test_result_finalizer import await_result
            await_result(w)
            w.session_timeout = 0.01
            w._session_t0 = _t.monotonic() - 10.0
            self._tick(w, self._truck())            # 这一拍强开新会话
            self.assertTrue(w._session_forced, "前提：会话确实是被超时强开的")
            w.session_timeout = 180.0
            w._session_t0 = _t.monotonic()
            self._tick(w, Frame(self._truck().xyz, dropped_packets=3))   # 紧跟一帧残缺帧
            self.assertTrue(w._session_forced, "残缺帧不能当成车走了")
            self.assertNotEqual(w._last_save_name, "", "残缺帧不能清掉上一份落盘名")
            for _ in range(30):
                self._tick(w, self._truck())
            await_result(w, count=2)
            ms = self._metas(tmp)
            self.assertEqual(len(ms), 2, "超时之后应当又落了一份")
            self.assertEqual(ms[1].get("duplicate_of"), ms[0]["measurement_id"],
                             "同一辆车，第二份必须标重复")
        finally:
            w.stop()
            self.assertTrue(w.close_results(10.))
            w.source.close()
            shutil.rmtree(tmp, ignore_errors=True)

    def test_a_new_vehicle_after_the_lane_cleared_is_not_marked(self):
        """★ 车真走了再来一辆 —— 绝不能打成重复，否则真车次会被去掉。"""
        import shutil
        import tempfile
        tmp = tempfile.mkdtemp(prefix="dupmark_")
        try:
            w = self._worker(tmp)
            self._tick(w, self._truck())
            for _ in range(30):
                self._tick(w, self._truck())
            from tests.test_result_finalizer import await_result
            await_result(w)
            self._tick(w, Frame(self._ground()))            # 车走了，车道空闲
            self.assertEqual(w._last_save_name, "", "回到空闲必须清掉上一次落盘")
            self._tick(w, self._truck())
            for _ in range(30):
                self._tick(w, self._truck())
            await_result(w, count=2)
            ms = self._metas(tmp)
            self.assertEqual(len(ms), 2)
            self.assertNotIn("duplicate_of", ms[1],
                             "车道空过了，这是新的一辆车")
        finally:
            w.stop()
            self.assertTrue(w.close_results(10.))
            w.source.close()
            shutil.rmtree(tmp, ignore_errors=True)

    def test_scene_change_after_a_forced_session_is_not_marked(self):
        """★ 超时强开残留 + 场景代次切换 —— 新场景第一辆绝不能打成重复（2026-09-23，M6）。

        车道从未回到空闲时，_session_forced / _last_save_name 会一直挂着 ——
        这是「同一辆车」的正确标记。但数据源场景代次变了（演示/回放切换），
        这辆车**必然**不是上一辆：若不清标记，新场景第一辆会被错标
        duplicate_of 指向旧场景那份，下游按此去重会错丢真实车次。
        人工翻篇则**不**清：点按钮的人可能就是想重测当前这辆车。
        """
        import shutil
        import tempfile
        import time as _t
        tmp = tempfile.mkdtemp(prefix="dupmark_")
        try:
            w = self._worker(tmp)
            self._tick(w, self._truck())            # 第一辆车驶入
            for _ in range(30):
                self._tick(w, self._truck())
            from tests.test_result_finalizer import await_result
            await_result(w)
            # 会话超时强开一次（车道从未空闲，标记保留 —— 这是正确的同车语义）
            w.session_timeout = 0.01
            w._session_t0 = _t.monotonic() - 10.0
            self._tick(w, self._truck())
            self.assertTrue(w._session_forced, "前提：会话确实是被超时强开的")
            self.assertNotEqual(w._last_save_name, "", "前提：上一份落盘名还在")
            w.session_timeout = 180.0
            w._session_t0 = _t.monotonic()
            # 人工翻篇不清标记 —— 重测同一辆车的重复语义必须保留。
            self.assertTrue(w.force_new_session())
            self.assertTrue(w._session_forced, "人工翻篇不证明车换了，标记必须保留")
            self.assertNotEqual(w._last_save_name, "")
            # 场景代次切换（gen+1）—— 车必然换了，标记必须一起清掉。
            self._tick(w, Frame(self._truck().xyz, gen=1))
            self.assertFalse(w._session_forced, "场景切换 = 新车，必须清掉强开标记")
            self.assertEqual(w._last_save_name, "", "场景切换必须清掉上一份落盘名")
            for _ in range(30):
                self._tick(w, Frame(self._truck().xyz, gen=1))
            await_result(w, count=2)
            ms = self._metas(tmp)
            self.assertEqual(len(ms), 2, "新场景的车应当正常落一份")
            self.assertNotIn("duplicate_of", ms[1],
                              "新场景的车与旧会话无连续性，绝不能错标重复")
        finally:
            w.stop()
            self.assertTrue(w.close_results(10.))
            w.source.close()
            shutil.rmtree(tmp, ignore_errors=True)


class TestRawFrameCapture(unittest.TestCase):
    """service.raw_frames：每次测量顺带存几帧**未裁 ROI** 的整帧。

    为什么需要：落盘的 .xyz 是「先裁 ROI、再做航向旋转」之后的点云，
    ROI 之外的东西根本不在文件里 —— 想验证"车厢有没有被 ROI 切掉"，
    拿它只能看"边缘离边界多远"做间接推断，而这在 y 方向不可靠
    （2026-09-14 实测：车厢远端因掠射角会看起来变窄，主体宽 2.67 m
    到 y=5.5 只剩 2.25 m）。有未裁的整帧才能**直接看到框外面**。
    """

    def _worker(self, n_raw):
        st = load_settings("")
        ch = [c for c in st.channels if c.id == "CH1"][0]
        ch.mode = "synthetic"
        return ChannelWorker(ch, EventBus(), st.plate, save_override=False,
                             plate_mode="null", state_push_hz=0.0, raw_frames=n_raw)

    @staticmethod
    def _read_pair(w):
        frame = w.source.read()
        w._pair_observation(frame, complete=True)

    def test_off_by_default_costs_nothing(self):
        """默认关闭时数据源不留未裁帧 —— 一分内存都不该占。"""
        w = self._worker(0)
        self.assertFalse(w.source.keep_raw)
        self.assertIsNone(w.source.take_raw())

    def test_enabling_makes_the_source_keep_raw(self):
        w = self._worker(3)
        self.assertTrue(w.source.keep_raw)

    def test_samples_are_spread_across_the_settle_window(self):
        """首 / 中 / 末 —— 末帧正是测量用的那一帧。"""
        w = self._worker(3)
        w.settle_sec = 3.0
        self._read_pair(w)                    # 走与主循环相同的一次性配对
        picked = []
        for waited in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.2):
            before = len(w._raw_shots)
            self._read_pair(w)
            w._grab_raw(waited)
            if len(w._raw_shots) > before:
                picked.append(waited)
        self.assertEqual(picked, [0.0, 1.5, 3.0],
                         f"取样时刻不对：{picked}（应当在 settle_sec 上均分）")

    def test_never_exceeds_the_configured_count(self):
        w = self._worker(2)
        w.settle_sec = 3.0
        for waited in np.arange(0, 6, 0.2):
            self._read_pair(w)
            w._grab_raw(float(waited))
        self.assertLessEqual(len(w._raw_shots), 2)

    def test_settle_zero_takes_one_shot(self):
        """settle_sec=0（停稳即测）时只取一帧，不能除零。"""
        w = self._worker(3)
        w.settle_sec = 0.0
        for _ in range(5):
            self._read_pair(w)
            w._grab_raw(0.0)
        self.assertEqual(len(w._raw_shots), 3)

    def test_saved_files_land_in_a_raw_subdir_with_json(self):
        import json as _json
        import tempfile, shutil
        w = self._worker(2)
        d = tempfile.mkdtemp(); self.addCleanup(shutil.rmtree, d, True)
        w.ch.abs_output_dir = d   # 加载时一次性算好的字段，不是属性
        w.settle_sec = 3.0
        for waited in (0.0, 3.0):
            self._read_pair(w)
            w._grab_raw(waited)
        n = w._save_raw("20260914_120000")
        self.assertEqual(n, 2)
        files = sorted(os.listdir(os.path.join(d, "raw", w.ch.id)))
        self.assertEqual(len([f for f in files if f.endswith(".xyz")]), 2)
        j = [f for f in files if f.endswith(".json")]
        self.assertEqual(len(j), 2)
        meta = _json.load(io.open(os.path.join(d, "raw", w.ch.id, j[0]),
                                  encoding="utf-8"))
        for k in ("roi", "yaw_offset_deg", "effective_yaw_offset_deg",
                  "ground_z", "paired_xyz", "settle_elapsed_sec"):
            self.assertIn(k, meta, f"旁注里少了 {k} —— 离线复算要靠它")

    def test_sidecar_name_lets_cleanup_remove_it(self):
        """★ 旁注必须叫 <点云名>.meta.json。

        storage.remove_sample 删的是 `xyz + XYZ_SUFFIXES`
        （.reason.txt / .meta.json）。叫别的名字，清理时点云被删、
        json 留成孤儿，日积月累一地垃圾。
        """
        import tempfile, shutil
        from app import storage
        w = self._worker(1)
        d = tempfile.mkdtemp(); self.addCleanup(shutil.rmtree, d, True)
        w.ch.abs_output_dir = d
        self._read_pair(w); w._grab_raw(0.0)
        w._save_raw("20260914_120000")
        raw = os.path.join(d, "raw", w.ch.id)
        xyz = [f for f in os.listdir(raw) if f.endswith(".xyz")][0]
        storage.remove_sample(os.path.join(raw, xyz))
        left = os.listdir(raw)
        self.assertEqual(left, [], f"清理之后还剩下孤儿文件：{left}")

    def test_named_and_foldered_by_lane(self):
        """整帧按车道分目录，文件名也以车道号打头 —— 现场翻看要靠这个。

        分目录：打开 CH3 就只看 3 号道。
        文件名再写一次：文件常被单独拷出来，离开目录后还得认得出是哪条道。
        """
        import tempfile, shutil
        w = self._worker(1)
        d = tempfile.mkdtemp(); self.addCleanup(shutil.rmtree, d, True)
        w.ch.abs_output_dir = d
        self._read_pair(w); w._grab_raw(0.0)
        w._save_raw("20260914_120000")
        sub = os.path.join(d, "raw", w.ch.id)
        self.assertTrue(os.path.isdir(sub), f"没有按车道建子目录：{w.ch.id}")
        for f in os.listdir(sub):
            self.assertTrue(f.startswith(w.ch.id + "_"),
                            f"文件名没有以车道号打头：{f}")

    def test_degraded_frames_are_never_captured(self):
        """★ 残缺帧一律跳过 —— 宁可少存一帧也不存带洞的。

        这些整帧的用途是"看车厢边缘之外是不是空的"。丢包造成的洞若正好
        落在车厢边缘，会被读成"车厢到这里就结束了"，得出完全相反的结论。
        """
        w = self._worker(3)
        w.settle_sec = 3.0
        for waited in (0.0, 1.5, 3.0):
            self._read_pair(w)
            w._grab_raw(waited, complete=False)
        self.assertEqual(w._raw_shots, [], "残缺帧被存进去了")
        # 跳过之后取样计划不推进：下一个完整帧补上
        self._read_pair(w)
        w._grab_raw(3.2, complete=True)
        self.assertEqual(len(w._raw_shots), 1)

    def test_saving_clears_the_buffer(self):
        """存完要清空，否则下一辆车会把上一辆的帧一起存进去。"""
        import tempfile, shutil
        w = self._worker(2)
        d = tempfile.mkdtemp(); self.addCleanup(shutil.rmtree, d, True)
        w.ch.abs_output_dir = d   # 加载时一次性算好的字段，不是属性
        self._read_pair(w); w._grab_raw(0.0)
        w._save_raw("a")
        self.assertEqual(w._raw_shots, [])
        self.assertEqual(w._save_raw("b"), 0)


if __name__ == "__main__":
    unittest.main()


class TestCaptureLossReport(unittest.TestCase):
    """★ 丢包率必须按「圈」量，不能看平均包率。

    2026-09-14 现场 CH1 丢包 28%，但平均包率看上去只是"偏低"——
    真实情况是少数圈缺掉一整块连续扇形（最差连续缺 31 包 ≈ 12.8 度）。
    那些帧会被 service._frame_is_complete 判为残缺 -> 不参与停稳判定、
    也不会被存成全量帧，直接导致「这一路采不到数据」。
    所以报告必须把**完整圈占比**和**最差一圈**摊开，而不是给个平均数。
    """

    def _cap(self, fn, *a):
        import io as _io
        from contextlib import redirect_stdout
        buf = _io.StringIO()
        with redirect_stdout(buf):
            fn(*a)
        return buf.getvalue()

    def _report(self):
        import os as _os
        import sys as _sys
        b = _os.path.join(_os.path.dirname(_os.path.dirname(
            _os.path.abspath(__file__))), "cli")
        if b not in _sys.path:
            _sys.path.insert(0, b)
        from capture import _report_loss
        return _report_loss

    def test_clean_stream_reports_ok(self):
        out = self._cap(self._report(), "x", [450] + [901] * 20 + [300], 0.4)
        self.assertIn("OK", out)
        self.assertIn("100.0%", out)
        self.assertNotIn("▲", out)

    def test_burst_loss_is_not_hidden_by_the_average(self):
        """18 圈满 + 2 圈缺 31% —— 平均只少 3%，但必须报出来。"""
        out = self._cap(self._report(),
                        "x", [450] + [901] * 18 + [620, 640] + [300], 12.8)
        self.assertIn("90.0%", out)
        self.assertIn("620", out, "最差一圈的绝对包数必须给出")
        self.assertIn("31%", out)
        self.assertIn("▲", out, "低于 99% 完整圈必须给排查提示")

    def test_first_and_last_revolutions_are_dropped(self):
        """抓包起止多半落在半圈上，算进去会误报成丢包。"""
        out = self._cap(self._report(), "x", [12] + [901] * 20 + [7], 0.4)
        self.assertIn("100.0%", out)

    def test_worst_gap_is_converted_to_packet_count(self):
        """给"缺了多少度"没用，现场要知道**缺了几个报文**。"""
        out = self._cap(self._report(),
                        "x", [450] + [901] * 18 + [620, 640] + [300], 12.8)
        self.assertIn("12.8", out)
        self.assertRegex(out, r"约 3[12] 个报文")

    def test_too_few_revolutions_says_so_instead_of_guessing(self):
        out = self._cap(self._report(), "x", [901, 901], 0.4)
        self.assertIn("圈数太少", out)
        self.assertNotIn("完整圈", out)

    def test_zero_median_does_not_divide_by_zero(self):
        out = self._cap(self._report(), "x", [0] * 8, 0.0)
        self.assertIn("方位角没推进", out)

class TestMetaEmptyIsTheVerdict(unittest.TestCase):
    """落盘 meta 的 `empty` 必须是**判定链的结论**，不是六面拟合成功与否。

    2026-09-17 查实的真 bug：`_save` 里写的是 `res.ok`，而
    `EmptyResult.ok` 的含义是"六面拟合成功了"，和空不空车没关系。
    现场 137 份 meta 里 **115 份**写着 empty=True 而实际是载货车。

    为什么这条要紧：按车牌绑定空车云图那套设计里，下游就是照 meta 的
    `empty` 去挑"哪一份是这辆车的空车基线"。写错 = 把载货点云存成基线，
    之后这辆车的体积会算成接近 0，而且**不会报错**。
    """

    def _save_once(self, chain_ok, fit_ok):
        import json as _json
        import os as _os
        import tempfile
        import types
        from app.service import ChannelWorker
        from app.detect import EmptyResult, Diagnostics
        from app.pointcloud import Frame

        d = tempfile.mkdtemp()
        res = EmptyResult(cloud=np.zeros((50, 3), np.float32))
        res.ok = fit_ok
        res.dims = (2.4, 11.0, 2.8)
        dg = Diagnostics()
        vol = types.SimpleNamespace(to_dict=lambda: {})
        pr = types.SimpleNamespace(error="", ok=True)
        from app.settings import (load_calib, load_devices, load_settings,
                                  config_fingerprint)
        st = load_settings("")
        ch0 = st.channels[0]
        w = types.SimpleNamespace(
            ch=types.SimpleNamespace(id="CH9", abs_output_dir=d),
            dev=load_devices(ch0.abs_devices_path),
            baseline=None,
            _config_fp=config_fingerprint(load_calib(ch0.abs_config_path),
                                          load_devices(ch0.abs_devices_path),
                                          getattr(ch0.lidar, "yaw_offset_deg", None)),
            _degraded_measure=False, _session_forced=False, _last_save_name="",
            _save_raw=lambda name: 0,
            _worth_labeling=lambda dg: False,
        )
        path = ChannelWorker._save(w, Frame(res.cloud), res, "宁A96523",
                                   "空车" if chain_ok else "底面内点占比过低",
                                   dg, vol, pr, None, is_empty=chain_ok)
        with io.open(path + ".meta.json", encoding="utf-8") as f:
            return _json.load(f)

    def test_loaded_vehicle_is_not_marked_empty(self):
        """拟合成功但判定链说非空 —— 这正是现场那 115 份的形态。"""
        m = self._save_once(chain_ok=False, fit_ok=True)
        self.assertFalse(m["empty"],
                         "载货车被写成 empty=True —— 下游会拿它当空车基线")
        self.assertTrue(m["fit_ok"], "六面拟合的结果应另存 fit_ok，别丢掉")

    def test_empty_vehicle_is_marked_empty(self):
        m = self._save_once(chain_ok=True, fit_ok=True)
        self.assertTrue(m["empty"])

    def test_meta_carries_the_config_fingerprint(self):
        """★ 端到端钉住 docs/14 §1.8：落盘的 meta 必须带配置指纹。

        没有它，存档结论会随配置变化悄悄过期而无从察觉 —— 实测 62 份
        现场样本里已有 7 份（11%）的 reason 与用当前配置重算的结果不一致。
        """
        m = self._save_once(chain_ok=True, fit_ok=True)
        self.assertIn("config", m)
        fp = m["config"]
        self.assertTrue(fp.get("sha"), "指纹里没有 sha")
        for k in ("z_above_mode", "yaw_offset_deg", "ground_z", "roi"):
            self.assertIn(k, fp, "指纹里缺人能直接看懂的 %s" % k)

    def test_empty_survives_a_failed_fit(self):
        """两个字段是独立的：拟合没成不代表判定链的结论要跟着翻。"""
        m = self._save_once(chain_ok=True, fit_ok=False)
        self.assertTrue(m["empty"])
        self.assertFalse(m["fit_ok"])

def _bed_and_cargo(nx=24, ny=110, cell=0.10, bed=1.2, cargo=1.0,
                   drop_ends=0, seed=3, gz=-5.23, wall=1.6):
    """造「空车基线 + 载货」一对：平底板 + 两侧厢壁 + 一层等高的货。

    · drop_ends > 0：把载货侧两端各 drop_ends 格的点**挖掉**，
      模拟掠射角下两端扫不到（docs/13 §14）。货其实在那儿，只是没回波。
    · wall：两侧厢壁高出底板多少。**厢壁必须有** —— 它是结构性盲区，
      每一车都存在，`filled_ratio` 恒在 25~40% 就是它造成的。
      造景漏了厢壁的话，`filled_ratio` 与 `hole_ratio` 会几乎相等，
      "别拿 filled_ratio 当门槛"那条用例就抓不住东西了。
    """
    rng = np.random.default_rng(seed)
    x = (np.arange(nx) + 0.5) * cell - nx * cell / 2
    y = (np.arange(ny) + 0.5) * cell - ny * cell / 2
    xx, yy = np.meshgrid(x, y)

    def pts(h, mask=None):
        m = np.ones_like(xx, bool) if mask is None else mask
        n = int(m.sum())
        return np.c_[xx[m] + rng.normal(0, 0.005, n),
                     yy[m] + rng.normal(0, 0.005, n),
                     np.full(n, gz + h) + rng.normal(0, 0.004, n)]

    empty = pts(bed)
    if wall:
        w = np.zeros_like(xx, bool)
        w[:, :2] = True
        w[:, -2:] = True                      # 最外两列 = 两侧厢壁
        empty = np.vstack([empty, pts(bed + wall, w)])
    keep = np.ones_like(xx, bool)
    if drop_ends:
        keep[:drop_ends, :] = False
        keep[-drop_ends:, :] = False
    loaded = pts(bed + cargo, keep)
    return empty.astype(np.float64), loaded.astype(np.float64), nx * cell * ny * cell


def _mk(empty, loaded):
    from app.detect import EmptyBaseline, LoadedResult
    b = EmptyBaseline(floor_z=float(np.median(empty[:, 2])),
                      box=(empty[:, 0].min(), empty[:, 0].max(),
                           empty[:, 1].min(), empty[:, 1].max(),
                           empty[:, 2].min() - 0.1, empty[:, 2].max() + 3.0),
                      cloud=empty.astype(np.float32), n=len(empty))
    l = LoadedResult()
    l.cloud = loaded.astype(np.float32)
    l.ok = True
    return b, l


class TestVolumeMultiFrame(unittest.TestCase):
    """货面多帧填 —— docs/13 §16.2。

    ▲ 铁律：多帧只喂**货面高度图**，盒子/配准仍只用主帧。
      把多帧揉成一片去定盒子会把车厢盒撑长（1 帧 11.5 -> 3 帧 14.35 m），
      实测体积从 55 冲到 69。
    """

    def test_extra_frames_fill_the_unscanned_ends(self):
        """两端缺数据时，补上另外两帧应当把体积拉回真值附近。"""
        from app.detect import compute_volume
        empty, full, area = _bed_and_cargo(drop_ends=0)
        # 主帧缺两端；另外两帧各缺一端 —— 合起来才凑齐
        _, f_main, _ = _bed_and_cargo(drop_ends=8, seed=4)
        _, f_a, _ = _bed_and_cargo(drop_ends=0, seed=5)
        b, l = _mk(empty, f_main)
        one = compute_volume(b, l, fill_holes=False)
        multi = compute_volume(b, l, fill_holes=False, loaded_extra=[f_a])
        self.assertTrue(one.ok and multi.ok)
        self.assertEqual(one.frames_used, 1)
        self.assertEqual(multi.frames_used, 2)
        self.assertGreater(multi.volume_m3, one.volume_m3 * 1.05,
                           "多帧没有把两端补回来 —— loaded_extra 可能没生效")

    def test_extra_frames_do_not_change_the_box(self):
        """★ 盒子必须只由主帧定：给一帧**范围更大**的额外点云，
        体积不许跟着暴涨（那正是"揉成一片"会犯的错）。"""
        from app.detect import compute_volume
        empty, loaded, _ = _bed_and_cargo()
        b, l = _mk(empty, loaded)
        base = compute_volume(b, l)
        # 额外帧在 y 上多伸出 2 m（模拟合并把盒撑长）
        _, longer, _ = _bed_and_cargo(ny=150, seed=9)
        ext = compute_volume(b, l, loaded_extra=[longer])
        self.assertTrue(base.ok and ext.ok)
        self.assertLess(abs(ext.volume_m3 - base.volume_m3) / base.volume_m3, 0.25,
                        "额外帧把积分区域撑大了 —— 盒子不该受 loaded_extra 影响")

    def test_none_and_empty_extra_are_noops(self):
        from app.detect import compute_volume
        empty, loaded, _ = _bed_and_cargo()
        b, l = _mk(empty, loaded)
        a = compute_volume(b, l)
        for ex in (None, [], [None]):
            v = compute_volume(b, l, loaded_extra=ex)
            self.assertAlmostEqual(v.volume_m3, a.volume_m3, places=6)
            self.assertEqual(v.frames_used, 1)


class TestVolumeConfidence(unittest.TestCase):
    """独立条件下的范围包含真值；不能豁免壁后未知区。"""

    @staticmethod
    def conditions(b, l):
        from tests._volume_context import conditions
        return conditions(b, l, domain=(-1.2, 1.2, -5.5, 5.5), origin=(-1.2, -5.5), surface=.03, bed=.01)

    def test_holes_raise_the_filled_ratio_and_spread(self):
        from app.detect import compute_volume
        empty, full, _ = _bed_and_cargo(drop_ends=0)
        _, holed, _ = _bed_and_cargo(drop_ends=10, seed=4)
        b1, l1 = _mk(empty, full)
        b2, l2 = _mk(empty, holed)
        good = compute_volume(b1, l1, dy=0., **self.conditions(b1, l1))
        bad = compute_volume(b2, l2, dy=0., **self.conditions(b2, l2))
        self.assertTrue(good.ok and bad.ok)
        self.assertGreater(bad.hole_ratio, good.hole_ratio + 0.05,
                           "挖掉两端之后空洞外推占比没升上去")
        # ▲ filled_ratio 含厢壁格、每车都有，不能拿它做门槛（见 confidence 的说明）
        self.assertGreater(bad.filled_ratio, good.filled_ratio)
        self.assertGreater(bad.bound_spread, good.bound_spread,
                           "挖掉两端之后上下界差没变大")
        self.assertGreater(bad.unknown_area_m2, good.unknown_area_m2)
        for v in (good, bad):
            self.assertLessEqual(v.bound_lo, 26.4)
            self.assertGreaterEqual(v.bound_hi, 26.4)

    def test_confidence_flags_a_bad_one(self):
        from app.detect import compute_volume
        empty, _, _ = _bed_and_cargo()
        _, holed, _ = _bed_and_cargo(drop_ends=14, seed=4)
        b, l = _mk(empty, holed)
        v = compute_volume(b, l)
        ok, why = v.confidence()
        self.assertFalse(ok)
        self.assertTrue(why)

    def test_complete_support_is_confident_but_wall_blindness_is_not(self):
        """保留带壁场景，配完整可信支持正例，禁止一律拒绝通过测试。"""
        from app.detect import compute_volume_auto
        from tests.test_volume_validity import scene
        from tests._volume_context import conditions
        empty, full, _ = _bed_and_cargo(drop_ends=0)
        b, l = _mk(empty, full)
        v = compute_volume_auto(b, l, dy=0., **self.conditions(b, l))
        self.assertTrue(v.ok)
        self.assertGreater(v.volume_m3, 0.)
        self.assertGreater(v.filled_ratio, 0.15,
                           "这个场景本就该有可观的壁格占比，否则用例没意义")
        self.assertLess(v.hole_ratio, 0.15,
                        "覆盖完好却算出很高的空洞占比")
        # 宽度门槛默认已关（2026-09-18 现场决定：质量筛选放到下游第三方系统）。
        # 显式给门槛来钉这条：要钉的是壁后未知区确实进了不确定性。
        ok, why = v.confidence(spread_max=0.03)
        self.assertFalse(ok, "壁后未知区不能从不确定性里豁免")
        self.assertGreater(v.unknown_area_m2, 0.)
        self.assertLessEqual(v.bound_lo, 26.4)
        self.assertGreaterEqual(v.bound_hi, 26.4)
        clean_b, clean_l = scene()
        clean = compute_volume_auto(clean_b, clean_l, dy=0., **conditions(clean_b, clean_l))
        self.assertTrue(clean.ok)
        self.assertAlmostEqual(24., clean.volume_m3)
        self.assertTrue(clean.confidence()[0], clean.confidence()[1])

    def test_bounds_bracket_the_answer(self):
        from app.detect import compute_volume
        empty, _, _ = _bed_and_cargo()
        _, holed, _ = _bed_and_cargo(drop_ends=8, seed=4)
        b, l = _mk(empty, holed)
        v = compute_volume(b, l, dy=0., **self.conditions(b, l))
        self.assertLessEqual(v.bound_lo, v.volume_m3 + 1e-6)
        self.assertGreaterEqual(v.bound_hi, v.volume_m3 - 1e-6)
        self.assertLessEqual(v.bound_lo, 26.4)
        self.assertGreaterEqual(v.bound_hi, 26.4)


class TestVolumeAutoCell(unittest.TestCase):
    """候选扫描不能靠粗化减少固定支持的未知面积。"""

    def test_auto_picks_a_cell_and_reports_it(self):
        from app.detect import compute_volume_auto
        empty, _, _ = _bed_and_cargo()
        _, holed, _ = _bed_and_cargo(drop_ends=6, seed=4)
        b, l = _mk(empty, holed)
        v = compute_volume_auto(b, l, dy=0., **TestVolumeConfidence.conditions(b, l))
        self.assertTrue(v.ok)
        self.assertTrue(v.cell_auto)
        self.assertEqual(v.bound_spread, v.bound_spread)      # 不是 nan

    def test_auto_scans_without_erasing_gaps_between_scan_lines(self):
        from app.detect import compute_volume, compute_volume_auto, VOLUME_CELLS
        from unittest.mock import patch
        rng = np.random.default_rng(7)
        empty, loaded, _ = _bed_and_cargo()
        # 按 0.10 m 的格子散布地挖掉一半的点 —— 模拟扫描线之间的缝
        keep = rng.random(len(loaded)) > 0.55
        b, l = _mk(empty, loaded[keep])
        opt = TestVolumeConfidence.conditions(b, l)
        fixed = compute_volume(b, l, voxel=0.10, dy=0., **opt)
        with patch("app.detect.compute_volume", wraps=compute_volume) as compute:
            auto = compute_volume_auto(b, l, dy=0., **opt)
        self.assertTrue(auto.ok)
        self.assertEqual(len(VOLUME_CELLS), compute.call_count)
        self.assertEqual(fixed.unknown_area_m2, auto.unknown_area_m2)
        self.assertEqual(fixed.bound_spread, auto.bound_spread)
        self.assertFalse(auto.confidence()[0])
        for cell in VOLUME_CELLS:
            value = compute_volume(b, l, voxel=cell, dy=0., **opt)
            self.assertTrue(value.ok)
            self.assertEqual(fixed.bound_lo, value.bound_lo)
            self.assertEqual(fixed.bound_hi, value.bound_hi)
            self.assertLessEqual(value.bound_lo, 26.4)
            self.assertGreaterEqual(value.bound_hi, 26.4)

    def test_auto_without_baseline(self):
        from app.detect import compute_volume_auto
        v = compute_volume_auto(None, None)
        self.assertFalse(v.ok)
        self.assertTrue(v.message)

class TestBaselineQuality(unittest.TestCase):
    """采空车基线前的重复性闸门 —— docs/13 §18。

    现场实证：2026-09-15 那两次**空车**标定，三帧互相关只有 0.799 / 0.865，
    而同一批**载货**是 0.986 / 1.000。拿前者建基线，同一辆车体积差 4%，
    且事后完全看不出来。这道闸门就是拦它的。
    """

    @staticmethod
    def _scene(shift=0.0, drop=0.0, seed=1, n=6000):
        rng = np.random.default_rng(seed)
        y = rng.uniform(-5.0, 5.0, n) + shift
        x = rng.uniform(-1.0, 1.0, n)
        z = np.full(n, -4.0) + rng.normal(0, 0.01, n)
        a = np.c_[x, y, z]
        if drop:                       # 砍掉一段 y，模拟"场景变了"
            a = a[~((a[:, 1] > 1.0) & (a[:, 1] < 1.0 + drop))]
        return a

    def test_identical_frames_pass(self):
        from app.detect import baseline_quality
        cl = [self._scene(seed=s) for s in (1, 2, 3)]
        q = baseline_quality(cl)
        self.assertTrue(q["ok"], q["why"])
        self.assertGreaterEqual(q["r_min"], 0.95)

    def test_a_shifted_frame_is_rejected(self):
        """车还在动 —— 必须拦住。"""
        from app.detect import baseline_quality
        cl = [self._scene(seed=1), self._scene(seed=2), self._scene(shift=1.5, seed=3)]
        q = baseline_quality(cl)
        self.assertFalse(q["ok"])
        self.assertIn("静止", q["why"])

    def test_a_changed_scene_is_rejected(self):
        """场景里有东西变了（不是平移）—— 同样要拦。"""
        from app.detect import baseline_quality
        cl = [self._scene(seed=1), self._scene(seed=2), self._scene(drop=3.0, seed=3)]
        q = baseline_quality(cl)
        self.assertFalse(q["ok"])

    def test_single_frame_cannot_judge(self):
        from app.detect import baseline_quality
        q = baseline_quality([self._scene()])
        self.assertFalse(q["ok"])
        self.assertIn("判不了", q["why"])

    def test_empty_input(self):
        from app.detect import baseline_quality
        q = baseline_quality([])
        self.assertFalse(q["ok"])
        self.assertEqual(q["n"], 0)

class TestRecentFramesThreadSafety(unittest.TestCase):
    """`_recent_frames` 是跨线程的：通道线程写、HTTP 线程（看板标定基线）读。

    2026-09-17 查实：不加锁直接迭代，**2 秒内必现**
    `RuntimeError: deque mutated during iteration` —— 而它炸的正好是
    现场点"标定空车基线"那一下。
    """

    def test_concurrent_append_and_snapshot(self):
        import threading
        import time
        from collections import deque
        lock = threading.Lock()
        d = deque(maxlen=4)
        for i in range(4):
            d.append(np.zeros((3, 3)))
        errs = []
        stop = threading.Event()

        def writer():
            while not stop.is_set():
                with lock:
                    d.append(np.zeros((3, 3)))

        def reader():
            while not stop.is_set():
                try:
                    with lock:
                        _ = [x for x in d]
                except Exception as e:                  # noqa: BLE001
                    errs.append(repr(e))
                    return

        tw = threading.Thread(target=writer, daemon=True)
        tr = threading.Thread(target=reader, daemon=True)
        tw.start(); tr.start()
        time.sleep(0.8)
        stop.set()
        tw.join(timeout=1); tr.join(timeout=1)
        self.assertEqual(errs, [], "加锁之后仍然炸了")

    def test_worker_uses_a_lock_on_both_sides(self):
        """★ 直接钉住 service.py 里两处都在锁内 —— 光测 deque 本身是空用例。"""
        import io as _io
        import os as _os
        from tests import _helpers as _H
        src = _io.open(_os.path.join(_H.ROOT, "app", "application", "channel_worker.py"),
                       encoding="utf-8").read()
        self.assertIn("self._recent_lock", src)
        # append 必须在 with 块里
        i = src.index("self._recent_frames.append(frame)")
        self.assertIn("with self._recent_lock:", src[max(0, i - 120):i],
                      "append 不在锁内")
        j = src.index("for x in self._recent_frames")
        self.assertIn("with self._recent_lock:", src[max(0, j - 200):j],
                      "读取不在锁内")

class TestVolExtraCap(unittest.TestCase):
    """喂给体积货面的额外帧数必须有**显式**上限。

    2026-09-17 查实两个问题：
      · 原来写成 `[-(settle_max_frames - 1):]`，而 `settle_max_frames` 默认 **60**
        —— 等于往货面塞最多 59 帧、约 650 万点，而实测只验证过 3 帧；
      · `settle_max_frames=1` 时退化成 `[-0:]` = **整个列表**（Python 的 -0 就是 0），
        与"只留 1 帧"的意图正好相反。
    """

    def test_negative_zero_slice_is_the_whole_list(self):
        """把那个陷阱本身钉下来，免得以后又有人写 [-(n-1):]。"""
        L = ["a", "b", "c", "d"]
        self.assertEqual(L[-(1 - 1):], L, "Python 的 -0 切片就是整个列表")

    def test_worker_uses_an_explicit_cap(self):
        import io as _io
        import os as _os
        from tests import _helpers as _H
        src = _io.open(_os.path.join(_H.ROOT, "app", "application", "channel_worker.py"),
                       encoding="utf-8").read()
        self.assertIn("_VOL_EXTRA_MAX", src)
        self.assertNotIn("[-(self.settle_max_frames - 1):]", src,
                         "又用回了会退化成整列表的切片")
        i = src.index("self._vol_extra = ")
        seg = src[i:i + 160]
        self.assertIn("_VOL_EXTRA_MAX", seg, "_vol_extra 没有按显式上限截断")

    def test_cap_matches_what_was_validated(self):
        """上限 = 2 额外帧（合计 3 帧）—— docs/13 §17.2 实测过的那个配置。"""
        from app.service import ChannelWorker
        self.assertEqual(ChannelWorker._VOL_EXTRA_MAX, 2)


class TestDemoCargoStaysInsideRoi(unittest.TestCase):
    """★ 合成点云的货堆不能整体掉到 ROI 外面（2026-09-22，见 docs/55 M1）。

    `SyntheticSource` 的 `cargo` 第四个分量是货堆在**箱体坐标**里的起高
    （`make_synthetic_box` 的地板 z=0），不是世界坐标。而 `read()` 之后
    还会整体 `z -= 5.0`。原值写的是 -5.0，于是：

        箱体坐标 z ∈ [-5.03, -3.86]  ->  平移后 z ∈ [-10.03, -8.86]

    而 ROI 的 z 窗口是 [-6, -0.75] —— **5 万个货堆点全被裁掉**，
    `demo_loaded=True` 与空车道产出的点数完全一样（都是 60 000）。

    ▲ 影响面要说准：`cli/run.py --demo` 走的是 `mode="simulate"`
      -> `RadarSimSource`，**不经过这条分支**，所以看板演示一直是对的。
      这条只在通道显式配 `mode: "synthetic"` 时才走到。
    """

    ROI_Z = (-6.0, -0.75)

    def _frame(self, z0):
        from app.radar_source import SyntheticSource
        from app.pointcloud import normalize_roi
        from app.settings import load_calib
        calib = load_calib(os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), "config", "config.json"))
        src = SyntheticSource(roi=normalize_roi(calib.roi), interval=0.0,
                              cargo=(2.28, 10.2, 1.17, z0))
        return src.read()

    def test_cargo_survives_the_roi(self):
        loaded = self._frame(0.0)
        self.assertGreater(loaded.n, 100000,
                           "货堆被 ROI 裁掉了，实得 %d 点" % loaded.n)
        z = loaded.xyz[:, 2]
        self.assertGreaterEqual(float(z.min()), self.ROI_Z[0])
        self.assertLessEqual(float(z.max()), self.ROI_Z[1])

    def test_the_old_value_really_did_clip_it(self):
        """反面对照：这条用例的前提没成立的话，上面那条证明不了什么。"""
        self.assertLess(self._frame(-5.0).n, self._frame(0.0).n,
                        "旧值本应把货堆裁掉，前提不成立")

    def test_the_shipped_demo_uses_the_fixed_value(self):
        """★ 钉住实际配线的那个值 —— 改回 -5.0 时这条要红。"""
        import inspect as _inspect
        from app import radar_source
        src = _inspect.getsource(radar_source.build_source)
        self.assertIn("cargo=(2.28, 10.2, 1.17, 0.0)", src,
                      "合成点云的货堆起高被改回箱体坐标以外的值了")


class TestAnalyzeLoadRejectsNonFinite(unittest.TestCase):
    """★ `analyze_load` 的非有限值守卫（2026-09-23，见 docs/55 M3）。

    与 `TestCarriageYWindowRejectsNonFinite` 是同一条纪律的第三处落点
    （`carriage_box` 本来就有、`carriage_y_window` 9-22 补上、这里 9-23 补上）。

    没有它时下面的取整会抛：
        全 NaN -> ValueError: cannot convert float NaN to integer
        含 inf -> OverflowError: cannot convert float infinity to integer
    报错里没有任何线索指向"点云含非有限值"。服务侧走不到（停稳层对非有限帧
    早退并打 frame_nonfinite），但 `cli/inspect_cloud.py` / `cli/replay.py`
    是**直接对文件**调的 —— 现场拿一份坏 .xyz 看某一帧就会踩到。
    """

    @staticmethod
    def _good(n=3000):
        r = np.random.default_rng(11)
        return np.column_stack([r.uniform(-1.2, 1.2, n), r.uniform(-5, 5, n),
                                np.full(n, -4.0)])

    def test_degenerate_inputs_return_a_message_instead_of_raising(self):
        from app.detect import analyze_load
        good = self._good()
        cases = {
            "全 NaN": np.full((500, 3), np.nan),
            "含 +inf": np.vstack([good, [[np.inf, 0., 0.]]]),
            "含 -inf": np.vstack([good, [[0., -np.inf, 0.]]]),
            "单点 z 为 NaN": np.vstack([good, [[0., 0., np.nan]]]),
            "只有两列": np.zeros((500, 2)),
        }
        for name, bad in cases.items():
            with self.subTest(name):
                out = analyze_load(np.asarray(bad, np.float64))
                self.assertTrue(out.message, "%s 应当带上说明" % name)
                self.assertIn("非有限值", out.message)

    def test_good_input_is_unaffected(self):
        """★ 守卫只拦坏输入 —— 正常点云必须照常算出结果。"""
        from app.detect import analyze_load
        out = analyze_load(self._good())
        self.assertNotIn("非有限值", out.message or "")

    def test_the_ten_baselines_still_analyse(self):
        """10 份回归基线一份都不许因为这道守卫改变行为。"""
        import glob
        from app.detect import analyze_load
        from app.pointcloud import load_xyz
        files = sorted(glob.glob(os.path.join(H.ROOT, "baseline", "**", "*.xyz"),
                                 recursive=True))
        self.assertEqual(10, len(files), "基线份数不对，这条用例的前提变了")
        for path in files:
            out = analyze_load(load_xyz(path).xyz)
            self.assertNotIn("非有限值", out.message or "",
                             "%s 被守卫误伤" % os.path.basename(path))


class TestConfigFilesTolerateBom(unittest.TestCase):
    """★ 配置 JSON 必须吃得下 BOM（2026-09-23，见 docs/55 M8）。

    Windows 记事本"另存为 UTF-8"会在文件头写 BOM（EF BB BF），而
    `json.load` 遇到 BOM 直接抛
        JSONDecodeError: Unexpected UTF-8 BOM (decode using utf-8-sig)
    —— **服务启动即死**，报错里没有一个字告诉现场"把 BOM 去掉"。
    而现场要改的正是 `config/channels.json`，记事本是最顺手的工具。

    同仓 `load_radar_transport.py` 早就用 `utf-8-sig` 读了，这里只是对齐。
    ▲ `utf-8-sig` 对**没有** BOM 的文件与 `utf-8` 行为完全一致。
    """

    def _with_bom(self, src):
        import tempfile
        with io.open(src, encoding="utf-8") as f:
            text = f.read()
        path = os.path.join(tempfile.mkdtemp(), os.path.basename(src))
        with io.open(path, "w", encoding="utf-8-sig") as f:
            f.write(text)
        with open(path, "rb") as f:
            self.assertEqual(b"\xef\xbb\xbf", f.read(3), "这条用例没真的写进 BOM")
        return path

    def test_all_three_configs_load_with_a_bom(self):
        from app.settings import load_calib, load_channels, load_devices
        root = H.ROOT
        for name, loader in (("channels.json", load_channels),
                             ("config.json", load_calib),
                             ("devices.json", load_devices)):
            with self.subTest(name):
                loader(self._with_bom(os.path.join(root, "config", name)))

    def test_the_channel_values_survive_the_bom(self):
        """不止"不报错"——BOM 被吃掉之后第一个键要能正常读出来。"""
        from app.settings import load_channels
        path = self._with_bom(os.path.join(H.ROOT, "config", "channels.json"))
        _sv, chans, _p, _s = load_channels(path)
        self.assertEqual(4, len(chans))
        self.assertEqual("CH1", chans[0].id, "BOM 粘在第一个键上了")

    def test_files_without_a_bom_are_unchanged(self):
        from app.settings import load_channels
        _sv, chans, _p, _s = load_channels(os.path.join(H.ROOT, "config", "channels.json"))
        self.assertEqual(4, len(chans))


class TestLowSeverityBatch20260923(unittest.TestCase):
    """★ 2026-09-23 低严重度修复批次：每条改动一个可钉的断言。"""

    def test_throttle_sleep_waits_the_full_interval(self):
        """节流必须睡满剩余时间（分 0.5s 段），不再提前出帧。"""
        import time as _t
        from app.radar_source import _throttle_sleep
        t0 = _t.time()
        _throttle_sleep(0.8, _t.time())            # interval > 0.5s：旧实现只睡 0.5s
        elapsed = _t.time() - t0
        self.assertGreaterEqual(elapsed, 0.75, "0.8s 的节流至少要睡满 0.75s")

    def test_packet_layout_rejects_out_of_range_angle_offsets(self):
        """角度偏移越界在构造解码器时即报错（配置错宁可启动即失败）。"""
        import copy
        from app.radar_packet import PacketDecoder, PacketError
        settings, ch, calib, angle, _ = H.ctx()
        bad = copy.deepcopy(calib)
        bad.packet.angle2_offset = bad.packet.size + 4     # 伸到报文外
        with self.assertRaises(PacketError):
            PacketDecoder(bad.packet, bad.processing, angle,
                          range_mode=ch.lidar.range_mode, z_down=True,
                          max_range_m=ch.lidar.max_range_m)

    def test_safe_dir_name_escapes_windows_reserved_device_names(self):
        from app.plate import safe_dir_name
        self.assertEqual("~CON", safe_dir_name("con"))
        self.assertEqual("~COM1", safe_dir_name("COM1"))
        self.assertEqual("~CON.TXT", safe_dir_name("con.txt"))
        self.assertEqual("宁A10333F", safe_dir_name("宁A10333F"),
                         "正常车牌不受保留名转义影响")

    def test_angle_cache_rejects_non_finite_corrections(self):
        """nan/inf 修正行被跳过，且以 channels_seen 不足的形式暴露。"""
        import hashlib
        import tempfile
        from app.pointcloud import AngleCache
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, "angle.csv")
            rows = ["1,0.5,0.1", "2,nan,0.2", "3,inf,-0.3"]
            digest = hashlib.sha256(("\n".join(["Channel,Elevation,Azimuth"] + rows) + "\n").encode()).hexdigest()
            with io.open(p, "w", encoding="utf-8", newline="\n") as fp:
                fp.write("Channel,Elevation,Azimuth\n" + "\n".join(rows) + "\n" + digest + "\n")
            ac = AngleCache(p).load()
            self.assertEqual(1, ac.channels_seen, "nan/inf 行必须被拒收")

    def test_check_range_rejects_infinity(self):
        from app.settings import check_range
        with self.assertRaises(ValueError):
            check_range("x.json", "devices.fusion.session_timeout_sec", float("inf"))

    def test_enter_counters_are_coerced_to_int(self):
        """service 段的三个 enter_* 键 JSON 写成 30.5 也要以 int 落进 dataclass。"""
        import json
        import tempfile
        from app.settings import load_channels
        with io.open(os.path.join(H.ROOT, "config", "channels.json"), encoding="utf-8") as fp:
            d = json.load(fp)
        d["service"]["enter_frames"] = 30.5
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, "channels.json")
            with io.open(p, "w", encoding="utf-8") as fp:
                json.dump(d, fp, ensure_ascii=False)
            sv, _chans, _p, _s = load_channels(p)
            self.assertIs(int, type(sv.enter_frames))
            self.assertEqual(30, sv.enter_frames)

    def test_ransac_inliers_match_the_refined_plane(self):
        """inliers 必须与重拟合后的最终平面严格对应，不再张冠李戴。"""
        from app.geometry import fit_plane_ransac, Plane
        rng = np.random.default_rng(5)
        # 主平面 6000 点 + 一片会赢得初始 RANSAC 的密集小簇（重拟合后内点会变）
        main = np.column_stack((rng.uniform(-3, 3, 6000), rng.uniform(-3, 3, 6000),
                                np.zeros(6000)))
        blob = np.column_stack((rng.uniform(0.8, 1.0, 900), rng.uniform(0.8, 1.0, 900),
                                rng.uniform(0.05, 0.1, 900)))
        pts = np.vstack((main, blob)).astype(np.float64)
        p = fit_plane_ransac(pts, thr=0.02, iters=80, prior=Plane(0, 0, 1, 0, True), seed=3)
        self.assertIsNotNone(p)
        recount = int((p.abs_distance(pts) <= 0.02).sum())
        self.assertEqual(recount, int(p.inliers),
                         "inliers 字段必须等于最终平面的实际内点数")

    def test_bad_packet_layout_fails_at_decoder_construction(self):
        """★ 布局错的两层防线（2026-09-23 低严重度批次）。

        `PacketDecoder._check_layout` 的启动校验（size 装不下 / 角度偏移
        越界 / block 跨度矛盾）已把畸形布局**结构性堵死在构造期** ——
        `RadarSimulator` 里的运行时切片检查因此成为纵深防御（正常路径
        不可达，保留它防的是将来 _check_layout 放宽或旁路构造）。
        这里钉住第一层：坏配置必须启动即报，且现有配置必须照常加载。
        """
        import copy
        from app.radar_packet import PacketDecoder, PacketError
        settings, ch, calib, angle, _ = H.ctx()
        PacketDecoder(calib.packet, calib.processing, angle,
                     range_mode=ch.lidar.range_mode, z_down=True,
                     max_range_m=ch.lidar.max_range_m)      # 现场配置不拦
        for mutate in (lambda p: setattr(p, "size", 64),
                       lambda p: setattr(p, "angle2_offset", p.size + 4),
                       lambda p: setattr(p, "block1_data_offset", p.angle2_offset + 4)):
            bad = copy.deepcopy(calib)
            mutate(bad.packet)
            with self.assertRaises(PacketError):
                PacketDecoder(bad.packet, bad.processing, angle,
                              range_mode=ch.lidar.range_mode, z_down=True,
                              max_range_m=ch.lidar.max_range_m)


class TestVolumeDistinguishesMissingBaselineFromFitFailure(unittest.TestCase):
    """★ 「没有基线」与「载货拟合失败」必须报不同的理由（2026-09-23）。

    `compute_volume` 原来对两种情况都报 `无空车基线`：

        if baseline is None or baseline.n == 0: -> 无空车基线   （对）
        if loaded is None:                      -> 无空车基线   （错）

    但走到第二条时 baseline 已经确认非空 —— `loaded=None` 只可能是
    载货四面拟合没成功。实测（跑通体积链时发现）：基线刚标定好、
    186 193 点躺在 data/in/CH2.baseline.npz 里，体积却报「无空车基线」。

    ▲ 危害是**把人支使到错误的方向**：现场会去重新标定一份根本没问题的
      基线（而标定还要等一辆空车进场、过重复性闸门），真正该查的
      "这一帧为什么拟不出货面"反而没人看。
    """

    def _baseline(self):
        from app.detect import EmptyBaseline
        import numpy as np
        r = np.random.default_rng(5)
        n = 4000
        cloud = np.column_stack([r.uniform(-1.2, 1.2, n), r.uniform(-6, 6, n),
                                 np.full(n, -5.0) + r.normal(0, .003, n)])
        b = EmptyBaseline(cloud=cloud, floor_z=-5.0, n=n,
                          box=(-1.2, 1.2, -6.0, 6.0, -5.0, -2.4),
                          created="用例合成")
        self.assertTrue(getattr(b, "n", 0) > 0, "这条用例的前提没成立：基线是空的")
        return b

    def test_no_baseline_still_says_so(self):
        from app.detect import compute_volume
        self.assertEqual("无空车基线", compute_volume(None, None).message)

    def test_fit_failure_does_not_blame_the_baseline(self):
        from app.detect import compute_volume
        vr = compute_volume(self._baseline(), None)
        self.assertNotEqual("无空车基线", vr.message,
                            "基线好好的却报无基线 —— 现场会去重标一份没问题的基线")
        self.assertIn("载货拟合", vr.text)

    def test_the_text_tells_the_field_not_to_recalibrate(self):
        """★ 光换个措辞不够，得直说"不用重标"，否则现场还是会去重标。"""
        from app.detect import compute_volume
        self.assertIn("不用重标", compute_volume(self._baseline(), None).text)


class TestBaselineKeyedByPlate(unittest.TestCase):
    """★★ 空车基线**按车牌存**，不按车道（2026-09-23 现场定案）。

    业务事实：**空车只会上一次车道**（进场时在某一条道过一次空车扫描），
    **载重之后走哪条道是不一定的**（4 条车道随机）。所以基线必须属于车，
    不能属于道 —— 否则空车在 CH4 标定、载重走 CH2，CH2 就拿不到这辆车的车厢。

    改造之前是 `<通道>.baseline.npz` 一条道一份，而内容却是某一辆车的车厢
    （`box` 直接进 compute_volume 的积分域）。实测辆辆车厢不同（长厢
    10.45~13.90 m 连续铺开），dy 在 ±2 m 内变动体积差 18%。
    """

    PLATE = "宁A5548Q"

    def setUp(self):
        import copy, tempfile
        from app.settings import load_settings
        from app.service import ChannelWorker, EventBus
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        st = load_settings("")
        self.workers = {}
        for cid in ("CH4", "CH2"):
            ch = copy.deepcopy([c for c in st.channels if c.id == cid][0])
            ch.mode = "synthetic"
            ch.abs_baseline_dir = self.tmp.name          # ★ 四条道共用一个基线目录
            w = ChannelWorker(ch, EventBus(), st.plate, save_override=False, plate_mode="null")
            self.addCleanup(w.source.close)
            self.addCleanup(w.stop)
            self.workers[cid] = w

    def _save(self, worker, plate, n=5000):
        from app.detect import EmptyBaseline
        r = np.random.default_rng(3)
        cloud = np.column_stack([r.uniform(-1.2, 1.2, n), r.uniform(-6, 6, n),
                                 np.full(n, -5.0)])
        EmptyBaseline(-5.0, (-1.2, 1.2, -6, 6, -5.0, -2.4), cloud, n, source_channel=worker.ch.id).save(
            worker.baseline_path_for(plate))

    def test_baseline_captured_on_one_lane_is_found_on_another(self):
        """★ 核心业务要求：CH4 标定的空车，载重走 CH2 也要查得到。"""
        self._save(self.workers["CH4"], self.PLATE)
        b, key = self.workers["CH2"]._resolve_baseline(self.PLATE)
        self.assertIsNotNone(b, "空车在 CH4 标定，载重走 CH2 却查不到 —— 按车道存的老毛病")
        self.assertEqual(5000, b.n)

    def test_another_vehicle_gets_no_baseline_not_someone_elses(self):
        """★★ 查不到就是 None —— 绝不能退回别的车的车厢盒。

        这是整个改造的要点：宁可"这辆车没有基线、不出体积"，
        也不能拿另一辆车的车厢算出一个**不报错的错数**。
        """
        self._save(self.workers["CH4"], self.PLATE)
        b, _ = self.workers["CH2"]._resolve_baseline("宁B08669")
        self.assertIsNone(b, "另一辆车拿到了宁A5548Q 的基线 —— 这就是要修掉的错")

    def test_unknown_plate_never_gets_a_baseline(self):
        from app.plate import EMPTY_PLATE
        self._save(self.workers["CH4"], self.PLATE)
        for plate in ("", EMPTY_PLATE):
            with self.subTest(plate=plate):
                self.assertIsNone(self.workers["CH4"]._resolve_baseline(plate)[0])

    def test_legacy_lane_baseline_is_never_used(self):
        """旧的 CH4.baseline.npz 就算还在，也**不许**拿来算任何一辆车的体积。"""
        import os
        from app.detect import EmptyBaseline
        w = self.workers["CH4"]
        r = np.random.default_rng(4)
        EmptyBaseline(-5.0, (-1, 1, -5, 5, -5, -2), r.normal(size=(800, 3)), 800).save(
            os.path.join(self.tmp.name, "CH4.baseline.npz"))
        self.assertIsNone(w._resolve_baseline(self.PLATE)[0])
        self.assertNotIn("CH4", w.known_baseline_plates(), "车道级旧文件被当成了车牌")
        self.assertTrue(w.snapshot()["legacy_lane_baseline"], "旧文件还在应当提醒")

    def test_same_plate_spelled_differently_is_one_file(self):
        """同一辆车的不同写法（大小写 / 首尾空白）只能对应**一份**基线。"""
        w = self.workers["CH4"]
        self.assertEqual(w.baseline_path_for("宁A5548Q"), w.baseline_path_for(" 宁a5548q "))

    def test_a_lane_that_looked_before_calibration_still_sees_it_after(self):
        """★★ 跨车道实测抓到的真缺陷（2026-09-23）：不许缓存"没有"。

        时序：CH2 先看见这辆车（此时还没标定，查到 None）→ CH4 标定写出基线
        → 这辆车装货走 CH2。第一版把 None 缓存了，CH2 永远查不到，不重启不恢复。
        单元用例都是"先存后查"，所以一直没暴露 —— 只有跨车道端到端才撞上。
        """
        ch4 = self.workers["CH2"]
        self.assertIsNone(ch4._resolve_baseline(self.PLATE)[0], "前提：此时还没标定")
        self._save(self.workers["CH4"], self.PLATE)      # 另一条道写出基线
        b, key = ch4._resolve_baseline(self.PLATE)
        self.assertIsNotNone(b, "CH2 之前查过一次，就再也看不见 CH4 刚写的基线了")
        self.assertEqual(self.PLATE, key)

    def test_recalibration_replaces_the_cached_baseline(self):
        """重新标定同一辆车后，已经缓存过的车道要拿到**新**的那份（按 mtime 校验）。"""
        import os, time
        ch4 = self.workers["CH2"]
        self._save(self.workers["CH4"], self.PLATE, n=5000)
        self.assertEqual(5000, ch4._resolve_baseline(self.PLATE)[0].n)
        path = ch4.baseline_path_for(self.PLATE)
        before = os.path.getmtime(path)
        self._save(self.workers["CH4"], self.PLATE, n=6000)
        os.utime(path, (before + 5, before + 5))          # 保证 mtime 确实变了
        self.assertEqual(6000, ch4._resolve_baseline(self.PLATE)[0].n,
                         "重标后仍拿着旧基线 —— 缓存没按修改时间校验")

    def test_the_reported_plate_is_empty_when_nothing_was_found(self):
        """看板的 baseline_plate 只在**真查到**时才填，不能把"查过这个牌"报成"有基线"。"""
        b, key = self.workers["CH2"]._resolve_baseline("宁B08669")
        self.assertIsNone(b)
        self.assertEqual("", key, "没查到却报出了车牌 —— 看板会误以为这辆车有基线")

    def test_known_plates_lists_what_has_been_calibrated(self):
        for plate in ("宁A5548Q", "蒙C88593X"):
            self._save(self.workers["CH4"], plate)
        self.assertEqual(["宁A5548Q", "蒙C88593X"], self.workers["CH2"].known_baseline_plates())
