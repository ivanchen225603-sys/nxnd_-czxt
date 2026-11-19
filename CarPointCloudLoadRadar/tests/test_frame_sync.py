#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tests/test_frame_sync.py — 钉住 2026-09 现场问题「点云刷新太快、状态每秒都在变」

现场现象（设备内网、4 台真雷达）
    1. 没有车、只有车道时，看板上的点云也在飞快变化，根本看不清；
    2. 车开过来以后同样看不清，而且"车辆行驶状态"每秒都在翻。

根因不在判定链，而在**切帧**：LiveSource 原来按 frame_ms 的时间窗聚合报文，
时间窗和雷达转一圈**不同步**，于是每一帧只是一片位置不断变化的扇面 ——
点集每帧都不一样 → 画面乱闪、质心乱跳（dev 忽大忽小）→ 停稳状态乱翻。

修法三条，本模块逐条钉死：
    ① LiveSource.frame_mode="rotation"：按方位角回绕切帧，一帧＝一整圈；
    ② StabilityDetector 有车判据加回差，点数在阈值附近抖动时状态不再乱翻；
    ③ ChannelWorker 对看板 state 推送做节流，但**状态跳变立刻推**。
"""
from __future__ import annotations

import time
import unittest

import numpy as np

from ._helpers import ctx                      # noqa: F401  （把项目根加进 sys.path）
from app.settings import load_calib, root
from app.pointcloud import AngleCache, Frame
from app.radar_packet import PacketDecoder, make_synthetic_packet
from app.radar_source import LiveSource, LiveHub
from app.detect import StabilityDetector, StabilityState
from app.settings import load_settings
from app.service import ChannelWorker, EventBus

_CALIB = None
_ANGLE = None


def _calib():
    global _CALIB, _ANGLE
    if _CALIB is None:
        _CALIB = load_calib(root("config", "config.json"))
        _ANGLE = AngleCache(root("config", "angle_cache.csv")).load()
    return _CALIB, _ANGLE


def _source(mode="rotation", **kw):
    """不开 socket 的 LiveSource：挂一个未启动的 LiveHub，报文直接塞进队列。"""
    calib, angle = _calib()
    dec = PacketDecoder(calib.packet, calib.processing, angle,
                        range_mode="raw_offset", z_down=True, max_range_m=60.0)
    hub = LiveHub(bind_ip="127.0.0.1", udp_port=0)
    return LiveSource(dec, frame_ms=100, roi=calib.roi, hub=hub,
                      source_ip="10.0.0.1", name="t", frame_mode=mode, **kw)


def _push_rev(src, packets=90, raw=1200, t0=None):
    """往队列里塞完整的一圈：方位角 0 → 356°，步进 360/packets。"""
    calib, _ = _calib()
    t = t0 if t0 is not None else time.time()
    for k in range(packets):
        az = (k * 360.0 / packets) % 360.0
        src.queue.push((make_synthetic_packet(calib.packet, azimuth_deg=az,
                                              raw_distance=raw, seed=k), t))
        t += 0.0005
    return t


def _drain(src):
    out = []
    f = src.read()
    while f is not None:
        out.append(f)
        f = src.read()
    return out


class TestRotationFraming(unittest.TestCase):
    """一帧必须是一整圈 —— 这是「点云看得清」和「质心不乱跳」的前提。"""

    def test_one_frame_per_revolution(self):
        src = _source()
        frames = []
        t = time.time()
        for _ in range(4):
            t = _push_rev(src, packets=90, t0=t)
            frames += _drain(src)
        # 推了 4 圈：前 3 圈的回绕各切出一帧，最后一圈还没等到下一次回绕
        self.assertEqual(len(frames), 3, f"4 圈应切出 3 帧，实得 {len(frames)}")
        self.assertEqual(src.info()["rev_packets"], 90, "一帧应当正好是一圈的报文数")
        self.assertEqual(src.info()["frames_by_timeout"], 0, "不该走兜底")

    def test_point_count_is_stable_across_frames(self):
        """★ 核心：静止场景下帧与帧的点数必须一致。

        按时间切帧时这个数会忽多忽少（每帧覆盖的方位角不同），质心也就跟着乱跳
        —— 现场「状态每秒都在变」的直接来源。
        """
        src = _source()
        ns = []
        t = time.time()
        for _ in range(4):
            t = _push_rev(src, packets=90, t0=t)
            ns += [f.n for f in _drain(src)]
        self.assertGreaterEqual(len(ns), 3)
        self.assertEqual(len(set(ns)), 1, f"整圈切帧的点数应完全一致，实得 {ns}")

    def test_centroid_barely_moves_between_frames(self):
        """同样的静止场景，帧间质心位移应远小于 stability_delta(0.05m)。"""
        src = _source()
        frames = []
        t = time.time()
        for _ in range(3):
            t = _push_rev(src, packets=90, t0=t)
            frames += _drain(src)
        self.assertGreaterEqual(len(frames), 2)
        dev = float(np.linalg.norm(frames[-1].centroid() - frames[-2].centroid()))
        self.assertLess(dev, 0.01, f"静止场景帧间质心位移 {dev:.4f} m 偏大")

    def test_backlog_keeps_only_newest_complete_revolution(self):
        """worker 被检测占住时队列会积压好几圈 —— 只能出**最后一整圈**，不能重影。"""
        src = _source()
        t = time.time()
        for raw in (1000, 1500, 2000):          # 三圈，各自半径不同
            t = _push_rev(src, packets=90, raw=raw, t0=t)
        frames = _drain(src)
        self.assertEqual(len(frames), 1, "积压三圈也只应出一帧")
        r = float(np.median(np.linalg.norm(frames[0].xyz.astype(np.float64), axis=1)))
        # 最后一圈(raw=2000)还没收完，完整的最后一圈是 raw=1500 那圈
        self.assertGreater(r, 1.2, f"半径 {r:.3f} 像是 raw=1000 那一圈（出了旧数据）")
        self.assertLess(r, 1.9, f"半径 {r:.3f} 像是 raw=2000 那一圈（切了没收完的圈）")

    def test_falls_back_to_time_window_when_no_wrap(self):
        """方位角解析不对 / 雷达没转时，必须兜底出帧并把原因摆到 info() 上。"""
        calib, _ = _calib()
        src = _source(rotation_timeout_ms=150)
        t = time.time()
        for k in range(60):
            src.queue.push((make_synthetic_packet(calib.packet, azimuth_deg=17.0,
                                                  raw_distance=1200, seed=k), t))
            t += 0.0005
        self.assertIsNone(src.read(), "还没到兜底时限就不该出帧")
        time.sleep(0.2)
        self.assertIsNotNone(src.read(), "过了 rotation_timeout_ms 必须兜底出帧")
        info = src.info()
        self.assertEqual(info["frames_by_rotation"], 0)
        self.assertGreaterEqual(info["frames_by_timeout"], 1)
        self.assertIn("回绕", info.get("error", ""), "兜底必须在看板上留下可见线索")

    def test_time_mode_still_works(self):
        """旧的按时间切帧保留着（排查切帧本身时要用）。"""
        src = _source("time")
        _push_rev(src, packets=30)
        self.assertIsNone(src.read(), "时间窗没到就不出帧")
        time.sleep(0.13)
        f = src.read()
        self.assertIsNotNone(f, "时间窗到了应当出帧")
        self.assertEqual(src.info()["frame_mode"], "time")

    def test_unknown_frame_mode_raises(self):
        """打错一个字母必须启动即报错 —— 与 range_mode 同一套规矩。"""
        with self.assertRaises(ValueError):
            _source("rotaton")


class TestFallbackWarningDecays(unittest.TestCase):
    """兜底切帧的告警必须会"退烧"，否则开机一次抖动就永远挂着红字。"""

    def test_warning_clears_after_window(self):
        calib, _ = _calib()
        src = _source(rotation_timeout_ms=120)
        t = time.time()
        for k in range(40):                       # 方位角恒定 → 等不到回绕
            src.queue.push((make_synthetic_packet(calib.packet, azimuth_deg=5.0,
                                                  raw_distance=1200, seed=k), t))
            t += 0.0005
        src.read()
        time.sleep(0.18)
        src.read()                                # 触发一次兜底
        self.assertGreaterEqual(src.info()["frames_by_timeout"], 1)
        self.assertIn("回绕", src.info().get("error", ""), "刚发生时必须报出来")
        # 把"最近一次兜底"挪到 31 秒前 —— 等价于现场已经恢复正常
        src._last_timeout_t -= 31.0
        self.assertEqual(src.info().get("error", ""), "",
                         "超过 30s 没再发生就不该继续挂红字（累计计数仍然保留）")
        self.assertGreaterEqual(src.info()["frames_by_timeout"], 1)


class TestStateLinkInfo(unittest.TestCase):
    """看板的链路实况（包率/转速/包每圈）必须跟着 state 推送走。

    之前它只在**全量快照**里，而快照只在看板刚连上时发一次 ——
    于是这些数字一挂就是一整班，雷达中途掉线在看板上根本看不出来。
    """

    def test_state_event_carries_link(self):
        from app.service import EventBus, ChannelWorker
        from app.settings import load_settings
        st = load_settings("")
        ch = [c for c in st.channels if c.id == "CH1"][0]
        ch.mode = "synthetic"
        bus = EventBus()
        w = ChannelWorker(ch, bus, st.plate, save_override=False, state_push_hz=0.0)
        q = bus.subscribe(tag="test")
        r = np.random.default_rng(0)
        f = Frame(np.stack([r.uniform(-2, 2, 5000), r.uniform(-11, 8, 5000),
                            np.full(5000, -5.0)], 1).astype(np.float32))
        w.source.read = lambda: f
        w.last_frame = f
        w._tick()
        ev = [e for e in q if e["kind"] == "state"]
        self.assertTrue(ev, "应当推一条 state")
        self.assertIn("link", ev[-1])
        self.assertIn("error", ev[-1])
        self.assertIsInstance(ev[-1]["link"], dict)


class TestSkipLoadedWithoutBaseline(unittest.TestCase):
    """没有空车基线时不该再跑 detect_loaded —— 它占整条检测链七成时间，结果却被丢掉。"""

    def setUp(self):
        from tests._observations import ObservationClock
        self.clock = ObservationClock(self)

    def _worker(self, plate: str = ""):
        """plate 给了就让这台 worker 识别到这个牌（仿真车牌只出这一个）。

        ★ 基线目录一律指向临时目录 —— 默认的 ch.abs_baseline_dir 是真实的
          data/in，用例绝不能往现场基线目录里写文件。
        """
        import tempfile
        from app.service import EventBus, ChannelWorker
        from app.settings import load_settings
        st = load_settings("")
        ch = [c for c in st.channels if c.id == "CH1"][0]
        ch.mode = "synthetic"
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        ch.abs_baseline_dir = tmp.name
        if plate:
            st.plate.sample_plates = [plate]
        w = ChannelWorker(ch, EventBus(), st.plate, save_override=False,
                          plate_mode="simulate" if plate else "null")
        w.settle_sec = 0.0
        if plate:
            w.plate.delay = 0.0          # 仿真相机默认要等 0.6 s，用例里不需要
        return w

    @staticmethod
    def _truck(n=40000, seed=5):
        r = np.random.default_rng(seed)
        g = np.stack([r.uniform(-2, 2, 8000), r.uniform(-11, 8, 8000),
                      np.full(8000, -5.0)], 1)
        car = np.stack([r.uniform(-1.2, 1.2, n), r.uniform(-6, 6, n),
                        r.uniform(-4.2, -2.5, n)], 1)
        return Frame(np.concatenate([g, car]).astype(np.float32))

    def _run(self, w, frame):
        import app.application.channel_worker as S
        calls = []
        orig = S.detect_loaded
        S.detect_loaded = lambda *a, **k: calls.append(1) or orig(*a, **k)
        try:
            for _ in range(30):
                self.clock.tick(w, frame)
        finally:
            S.detect_loaded = orig
        return calls

    def test_not_called_without_baseline(self):
        w = self._worker()
        w.baseline = None
        self.assertEqual(len(self._run(w, self._truck())), 0,
                         "没有基线还在跑 detect_loaded —— 白烧 CPU")
        self.assertEqual(w.st.last_result["volume"]["message"], "无空车基线",
                         "结论仍然应当是「无空车基线」")

    def test_still_called_with_baseline(self):
        """有基线时体积分支必须照跑。

        ★ 基线按车牌存之后（2026-09-23），"有基线"的含义是：**识别到的这个车牌**
          在共用基线目录里有一份。直接写 `w.baseline = base` 已经不算数 ——
          测量时会按车牌重新解析、把它覆盖掉（这正是改造要的效果：
          绝不能拿一份不属于这辆车的基线去算）。
        """
        from app.detect import detect_empty, EmptyBaseline
        plate = "宁A5548Q"
        w = self._worker(plate)
        f = self._truck(seed=9)
        res = detect_empty(f, w.calib)
        base = EmptyBaseline.from_empty_result(res, "test") if res else None
        self.assertIsNotNone(base, "确定性合成夹具必须能生成空车基线")
        base.source_channel = "CH4"
        base.save(w.baseline_path_for(plate))
        self.assertGreaterEqual(len(self._run(w, self._truck())), 1,
                                "有基线时体积分支必须照跑")

    def test_not_called_when_this_vehicle_has_no_baseline(self):
        """★ 别的车有基线、这辆车没有 -> 照样不跑 detect_loaded，也不借别人的。"""
        from app.detect import detect_empty, EmptyBaseline
        w = self._worker("宁A5548Q")
        f = self._truck(seed=9)
        res = detect_empty(f, w.calib)
        base = EmptyBaseline.from_empty_result(res, "test") if res else None
        self.assertIsNotNone(base, "确定性合成夹具必须能生成空车基线")
        base.source_channel = "CH4"
        base.save(w.baseline_path_for("宁B08669"))      # 存的是**另一辆车**
        self.assertEqual(0, len(self._run(w, self._truck())),
                         "这辆车没有基线却跑了体积分支 —— 借用了别的车的基线")


class TestVehicleGateHysteresis(unittest.TestCase):
    """有车判据的回差：点数在阈值附近抖动时，状态不能每帧翻一次。"""

    def setUp(self):
        from tests._observations import ObservationClock
        self.clock = ObservationClock(self)

    @staticmethod
    def _frame(n_veh, n_ground=8000, seed=0):
        r = np.random.default_rng(seed)
        g = np.stack([r.uniform(-2, 2, n_ground), r.uniform(-11, 8, n_ground),
                      np.full(n_ground, -5.0)], 1)
        v = np.stack([r.uniform(-1.2, 1.2, n_veh), r.uniform(-6, 6, n_veh),
                      np.full(n_veh, -4.2)], 1)
        return Frame(np.concatenate([g, v]).astype(np.float32))

    def test_dither_around_threshold_does_not_flap(self):
        d = StabilityDetector(0.05, 3000, 30.0, ground_z=-5.0,
                              vehicle_min_height_m=0.30, vehicle_release_ratio=0.8)
        self.clock.detect(d, self._frame(3200))                      # 进入"有车"
        states = [self.clock.detect(d, self._frame(n)).state
                  for n in (2900, 3100, 2800, 3050, 2950)]
        self.assertNotIn(StabilityState.IDLE, states,
                         f"点数在阈值附近抖动就掉回空闲了：{states} —— 回差没生效，"
                         f"每次掉回空闲都会 reset 参考帧、结束会话")

    def test_really_leaving_still_goes_idle(self):
        """真的走了（掉到回差线以下）必须回到空闲，否则会话永不结束。"""
        d = StabilityDetector(0.05, 3000, 30.0, ground_z=-5.0,
                              vehicle_min_height_m=0.30, vehicle_release_ratio=0.8)
        self.clock.detect(d, self._frame(4000))
        self.assertEqual(self.clock.detect(d, self._frame(100)).state, StabilityState.IDLE)

    def test_entry_still_needs_full_threshold(self):
        """回差只放宽**退出**，进入仍要够满阈值 —— 否则空车道更容易被误判成有车。"""
        d = StabilityDetector(0.05, 3000, 30.0, ground_z=-5.0,
                              vehicle_min_height_m=0.30, vehicle_release_ratio=0.8)
        self.assertEqual(self.clock.detect(d, self._frame(2600)).state, StabilityState.IDLE)

    def test_ratio_one_is_old_behaviour(self):
        d = StabilityDetector(0.05, 3000, 30.0, ground_z=-5.0,
                              vehicle_min_height_m=0.30, vehicle_release_ratio=1.0)
        self.clock.detect(d, self._frame(3200))
        self.assertEqual(self.clock.detect(d, self._frame(2900)).state, StabilityState.IDLE)


class TestStabilityWobbleSuppression(unittest.TestCase):
    """丢几个报文就能让整帧质心偏过阈值 —— 不能因此判成"车又动了"。

    实测（450 包/圈的真实场景，丢连续 N 个报文后整帧质心的位移）：
        丢 1 包 → 0.014 m      丢 2 包 → 0.028 m
        丢 4 包 → **0.057 m**（越过 stability_delta=0.05）
        丢 8 包 → 0.115 m      丢 16 包 → 0.231 m
    UDP 偶发丢几个包是常态。压不住的话现场表现是：看板状态闪，
    更要命的是**沉降窗口被作废**，本来 3 秒能出的结论要重等一轮。

    注意参考帧是**上一帧**：掉一帧会产生**两次**超阈值（偏出去 + 偏回来），
    所以确认帧数默认是 3 而不是 2。
    """

    def setUp(self):
        from tests._observations import ObservationClock
        self.clock = ObservationClock(self)

    @staticmethod
    def _scene(n=40000, seed=0):
        rng = np.random.default_rng(seed)
        body = np.stack([rng.uniform(-1.2, 1.2, n), rng.uniform(-6, 6, n),
                         np.full(n, -4.2)], 1)
        ground = np.stack([rng.uniform(-2, 2, 8000), rng.uniform(-11, 8, 8000),
                           np.full(8000, -5.0)], 1)
        return np.concatenate([body, ground]).astype(np.float32)

    @classmethod
    def _at(cls, dx):
        """整帧平移 dx —— 这样 dev 恰好等于 dx（受控实验验证过的那个量）。"""
        return Frame(cls._scene() + np.array([dx, 0, 0], np.float32))

    def _det(self, **kw):
        kw.setdefault("release_frames", 3)
        kw.setdefault("break_ratio", 6.0)
        return StabilityDetector(0.05, 3000, 30.0, ground_z=-5.0,
                                 vehicle_min_height_m=0.30, **kw)

    def _stabilize(self):
        d = self._det()
        self.clock.detect(d, self._at(0.0))
        self.assertTrue(self.clock.hold(d, self._at(0.0)).stable, "先得停稳")
        return d

    def test_dropped_frame_revokes_permission_without_false_departure(self):
        """标残缺后撤销测量许可，不把缺扇区当作车辆离场。"""
        d = self._stabilize()
        r1 = self.clock.detect(d, self._at(.08), trusted=False)
        r2 = self.clock.detect(d, self._at(0))
        self.assertAlmostEqual(r1.deviation, .08, places=3)
        self.assertAlmostEqual(r2.deviation, 0, places=3)
        self.assertFalse(r1.stable)
        self.assertFalse(r2.stable)
        self.assertNotEqual(r1.state, StabilityState.IDLE)
        self.assertTrue(self.clock.hold(d, self._at(0)).stable)

    def test_three_consecutive_frames_do_break_stable(self):
        d = self._stabilize()
        self.clock.detect(d, self._at(0.08))
        self.clock.detect(d, self._at(0.16))
        r = self.clock.detect(d, self._at(0.24))         # 连续三帧在走 → 真在动
        self.assertFalse(r.stable, "连续三帧超阈值必须判成又动了")
        self.assertEqual(r.state, StabilityState.BASELINE)

    def test_big_jump_breaks_immediately(self):
        """明显的大位移不等确认帧 —— 沉降窗口靠它立刻作废。"""
        d = self._stabilize()
        r = self.clock.detect(d, self._at(1.0))          # 1 m，远超 delta×6
        self.assertFalse(r.stable)
        self.assertEqual(r.state, StabilityState.BASELINE)

    def test_driving_vehicle_breaks_on_first_frame(self):
        """真开着的车每帧至少走 0.3m（3m/s × 0.1s），一帧就该判出来。"""
        d = self._stabilize()
        r = self.clock.detect(d, self._at(0.36))
        self.assertFalse(r.stable)

    def test_release_frames_one_is_old_behaviour(self):
        d = self._det(release_frames=1)
        self.clock.detect(d, self._at(0.0))
        self.assertTrue(self.clock.hold(d, self._at(0.0)).stable)
        r = self.clock.detect(d, self._at(0.08))
        self.assertFalse(r.stable, "release_frames=1 应退回旧行为")

    def test_not_yet_stable_is_unaffected(self):
        """还没停稳过的车，一帧超阈值就该是"运动中" —— 不受确认帧影响。"""
        d = self._det()
        self.clock.detect(d, self._at(0.0))              # 只建了参考帧，还没停稳
        r = self.clock.detect(d, self._at(0.08))
        self.assertFalse(r.stable)
        self.assertEqual(r.state, StabilityState.BASELINE)


class TestSessionTimeoutTick(unittest.TestCase):
    """会话超时翻篇的那一拍必须就地收尾，不能接着往下走。

    踩过的两个坑（都会在现场刷出莫名其妙的现象）：
      ① `stab` 是 reset **之前**算出来的，可能还写着"已停稳"。往下走会开一个
         沉降窗口，而 reset 后的下一帧只是在重建参考帧（stable=False），
         窗口立刻作废 → 刷出"沉降窗口内车辆再次移动（偏差 0.000 m）"。
         车根本没动，现场会以为雷达在抖。
      ② 上一个会话没走完的 `_settle_t0` 不清掉，新会话会拿旧窗口的起点算
         waited，**直接跳过沉降窗口**就测量 —— 等于 settle_sec 白配了。
    """

    def setUp(self):
        from tests._observations import ObservationClock
        self.clock = ObservationClock(self)

    def _run(self, ticks=120):
        from app.service import EventBus, ChannelWorker
        from app.settings import load_settings
        st = load_settings("")
        ch = [c for c in st.channels if c.id == "CH1"][0]
        ch.mode = "synthetic"
        bus = EventBus()
        q = bus.subscribe(tag="t")
        w = ChannelWorker(ch, bus, st.plate, save_override=False,
                          plate_mode="null", state_push_hz=0.0)
        w.settle_sec = 0.3
        w.session_timeout = 4.0          # 压短，几秒内就能翻好几次篇
        r = np.random.default_rng(0)
        truck = Frame(np.concatenate([
            np.stack([r.uniform(-1.2, 1.2, 30000), r.uniform(-6, 6, 30000),
                      np.full(30000, -4.2)], 1),
            np.stack([r.uniform(-2, 2, 8000), r.uniform(-11, 8, 8000),
                      np.full(8000, -5.0)], 1)]).astype(np.float32))
        for _ in range(ticks):
            self.clock.tick(w, truck)
        return w, [e for e in q]

    def test_no_phantom_settle_warning(self):
        _w, evs = self._run()
        spur = [e["message"] for e in evs
                if e["kind"] == "warn" and "沉降窗口内车辆再次移动" in e.get("message", "")
                and "0.000" in e["message"]]
        self.assertFalse(spur, f"翻篇那一拍刷出了假告警：{spur[:1]}")

    def test_still_remeasures_after_timeout(self):
        """修掉假告警不能把"超时后重测"一起修没了。"""
        _w, evs = self._run()
        n = len([e for e in evs if e["kind"] == "result"])
        self.assertGreaterEqual(n, 2, "会话超时后应当继续重测")

    def test_settle_window_is_not_skipped(self):
        """新会话必须重新等满 settle_sec，不能拿旧窗口的起点蒙混过关。"""
        from app.service import EventBus, ChannelWorker
        from app.settings import load_settings
        st = load_settings("")
        ch = [c for c in st.channels if c.id == "CH1"][0]
        ch.mode = "synthetic"
        w = ChannelWorker(ch, EventBus(), st.plate, save_override=False,
                          plate_mode="null", state_push_hz=0.0)
        w.settle_sec = 5.0               # 故意配得很长
        w.session_timeout = 0.3
        w._settle_t0 = time.monotonic() - 999  # 伪造一个"很久以前就开始"的旧窗口
        w._session_t0 = time.monotonic() - 10  # 让这一拍必定超时
        w._session_id = 1
        r = np.random.default_rng(1)
        truck = Frame(np.stack([r.uniform(-1.2, 1.2, 30000),
                                r.uniform(-6, 6, 30000),
                                np.full(30000, -4.2)], 1).astype(np.float32))
        self.clock.tick(w, truck)
        self.assertIsNone(w._settle_t0, "翻篇时必须把旧的沉降窗口清掉")
        self.assertEqual(w._detected_this_session, 0, "翻篇那一拍不该直接测量")


class TestChannelHealthWarning(unittest.TestCase):
    """通道一直判「有车」却从不回到空闲 = 有车判据失效，系统此时是"瞎"的。

    2026-09-13 现场：4 号车道连着 10 分钟显示有车，实际一辆车都没有。
    根子是 `vehicle_points()` 拿**配置里写死的** ground_z 当基准 ——
    现场几何只要对不上，地面整片跑到判据线之上，就会：
        每帧都判有车 → 判已停稳 → 测一次 → "车"永远不走 → 会话永不结束
        → 一个会话只测一次 → **之后真来的车全部被静默跳过**
    代码早就预判了这个失效（会话超时那段注释写着），但只发了一条日志事件，
    而当时既没有落盘日志、看板上那行也会被刷走 —— 等于没说。
    所以现在把它升级成**常驻**的通道健康告警。
    """

    def setUp(self):
        from tests._observations import ObservationClock
        self.clock = ObservationClock(self)

    @staticmethod
    def _cloud(n=30000, seed=1):
        """模拟"地面整片跑到判据线之上"：所有点都在 z=-1.4 附近。"""
        r = np.random.default_rng(seed)
        return Frame(np.stack([r.uniform(-2, 2, n), r.uniform(-11, 8, n),
                               np.full(n, -1.4) + r.normal(0, 0.01, n)],
                              1).astype(np.float32))

    def _worker(self):
        from app.service import EventBus, ChannelWorker
        from app.settings import load_settings
        st = load_settings("")
        ch = [c for c in st.channels if c.id == "CH1"][0]
        ch.mode = "synthetic"
        bus = EventBus()
        w = ChannelWorker(ch, bus, st.plate, save_override=False,
                          plate_mode="null", state_push_hz=0.0)
        w.settle_sec = 0.0
        w.session_timeout = 0.05          # 把 180s 压短，测试才跑得快
        return w, bus.subscribe(tag="t")

    def _tick(self, w, f):
        self.clock.tick(w, f)

    def test_health_warning_appears_and_names_the_numbers(self):
        w, _q = self._worker()
        f = self._cloud()
        for _ in range(40):
            self._tick(w, f)
        self.assertTrue(w.st.health, "一直判有车却从不空闲，必须挂出常驻告警")
        for must in ("从未回到空闲", "100%", "有车判据已失效", "range_mode"):
            self.assertIn(must, w.st.health, f"告警里少了「{must}」")
        self.assertIn("若车道上确实停着车，属正常", w.st.health,
                      "两种可能都要写，别替现场下结论")

    def test_health_clears_when_lane_goes_idle(self):
        w, _q = self._worker()
        for _ in range(40):
            self._tick(w, self._cloud())
        self.assertTrue(w.st.health)
        # 车道真的空了（点数掉到阈值以下）
        r = np.random.default_rng(2)
        empty = Frame(np.stack([r.uniform(-2, 2, 500), r.uniform(-11, 8, 500),
                                np.full(500, -5.0)], 1).astype(np.float32))
        for _ in range(3):
            self._tick(w, empty)
        self.assertEqual(w.st.stability.get("state"), 0)
        self.assertEqual(w.st.health, "", "回到空闲说明判据是好的，告警要自动清掉")

    def test_normal_traffic_never_triggers_it(self):
        """车来了又走（正常车流）不该挂告警 —— 误报会让现场学会无视它。"""
        w, _q = self._worker()
        r = np.random.default_rng(3)
        truck = Frame(np.concatenate([
            np.stack([r.uniform(-1.2, 1.2, 30000), r.uniform(-6, 6, 30000),
                      np.full(30000, -4.2)], 1),
            np.stack([r.uniform(-2, 2, 8000), r.uniform(-11, 8, 8000),
                      np.full(8000, -5.0)], 1)]).astype(np.float32))
        empty = Frame(np.stack([r.uniform(-2, 2, 500), r.uniform(-11, 8, 500),
                                np.full(500, -5.0)], 1).astype(np.float32))
        for _ in range(3):                       # 来三辆车，每辆都正常走掉
            for _ in range(4):
                self._tick(w, truck)
            for _ in range(3):
                self._tick(w, empty)
            self.assertEqual(w.st.health, "", "正常车流被误报成判据失效")

    def test_state_event_carries_health(self):
        w, q = self._worker()
        for _ in range(40):
            self._tick(w, self._cloud())
        ev = [e for e in q if e["kind"] == "state"]
        self.assertTrue(ev and "health" in ev[-1], "看板要靠 state 事件拿到它")
        self.assertTrue(ev[-1]["health"])


class TestStatePushThrottle(unittest.TestCase):
    """看板推送节流：看着一样的帧少推，但「事情发生了」必须立刻推。"""

    def _worker(self, hz=5.0):
        from app.service import EventBus, ChannelWorker
        from app.settings import load_settings
        st = load_settings("")
        ch = [c for c in st.channels if c.id == "CH1"][0]
        ch.mode = "synthetic"
        bus = EventBus()
        w = ChannelWorker(ch, bus, st.plate, save_override=False, state_push_hz=hz)
        return w, bus.subscribe()

    @staticmethod
    def _ground(n=8000):
        r = np.random.default_rng(0)
        return Frame(np.stack([r.uniform(-2, 2, n), r.uniform(-11, 8, n),
                               np.full(n, -5.0)], 1).astype(np.float32))

    @classmethod
    def _truck(cls, n=40000, seed=5):
        r = np.random.default_rng(seed)
        car = np.stack([r.uniform(-1.2, 1.2, n), r.uniform(-6, 6, n),
                        r.uniform(-4.2, -2.5, n)], 1).astype(np.float32)
        return Frame(np.concatenate([cls._ground().xyz, car]))

    def _tick(self, w, frame):
        w.source.read = lambda: frame
        w.last_frame = frame
        w._tick()

    def test_same_state_is_throttled(self):
        w, q = self._worker(hz=5.0)              # 200ms 一次
        f = self._ground()
        for _ in range(20):                      # 20 帧同一个"空闲"状态
            self._tick(w, f)
        n = len([e for e in q if e["kind"] == "state"])
        self.assertLessEqual(n, 3, f"同一状态连推了 {n} 次 —— 节流没生效")
        self.assertGreaterEqual(n, 1, "至少要推一次，否则看板永远是空的")

    def test_state_change_is_pushed_immediately(self):
        w, q = self._worker(hz=1.0)              # 1s 才允许推一次
        self._tick(w, self._ground())            # 空闲
        q.clear()
        self._tick(w, self._truck())             # 立刻来车 → 状态变了
        kinds = [e["kind"] for e in q]
        self.assertIn("state", kinds,
                      "状态跳变被节流压掉了 —— 看板会慢半拍甚至漏掉整个事件")

    def test_zero_disables_throttle(self):
        w, q = self._worker(hz=0.0)
        f = self._ground()
        for _ in range(5):
            self._tick(w, f)
        self.assertEqual(len([e for e in q if e["kind"] == "state"]), 5,
                         "state_push_hz=0 应当每帧都推（旧行为）")


class TestFrameCompleteness(unittest.TestCase):
    """ChannelWorker 怎么认出"这一圈缺了一块"。

    2026-09-14 现场实测：一圈 901 个报文，正常抖动 ±2 包，但突发丢包会一次
    缺掉 18~31 个连续报文。基准取最近若干帧的**中位数**，所以雷达调速、
    或现场换了转速，判据自己会跟上，不用改配置。
    """

    def _worker(self):
        st = load_settings("")
        ch = [c for c in st.channels if c.id == "CH1"][0]
        ch.mode = "synthetic"
        return ChannelWorker(ch, EventBus(), st.plate, save_override=False,
                             plate_mode="null", state_push_hz=0.0)

    def _f(self, packets):
        return Frame(np.zeros((10, 3), np.float32), packets=packets)

    def test_unknown_packet_count_is_treated_as_complete(self):
        """回放 / 仿真 / 合成源不给包数 —— 不能因为"不知道"就把帧全排除。"""
        w = self._worker()
        for _ in range(10):
            self.assertTrue(w._frame_is_complete(self._f(0)))

    def test_steady_stream_is_complete(self):
        w = self._worker()
        for n in [901, 900, 902, 901, 901, 899, 901, 901]:
            self.assertTrue(w._frame_is_complete(self._f(n)), f"{n} 包被误判成残缺")

    def test_burst_loss_is_caught(self):
        """实测的那几个值：901 圈里缺 18~31 包必须被拦下。"""
        w = self._worker()
        for _ in range(8):
            w._frame_is_complete(self._f(901))       # 先把基准建起来
        for n in (883, 878, 871, 843, 801):
            self.assertFalse(w._frame_is_complete(self._f(n)),
                             f"{n} 包（缺 {901-n} 个）没被认出来")

    def test_sustained_packet_loss_cannot_redefine_complete(self):
        """没有独立转速变更证据时，低包数不能把自身洗成完整帧。"""
        w = self._worker()
        for _ in range(20):
            self.assertTrue(w._frame_is_complete(self._f(901)))
        for _ in range(80):
            self.assertFalse(w._frame_is_complete(self._f(450)))
        self.assertTrue(w._frame_is_complete(self._f(901)))

    def test_first_few_frames_are_accepted(self):
        """样本还不够时不能乱拦 —— 启动头几帧必须放行。"""
        w = self._worker()
        self.assertTrue(w._frame_is_complete(self._f(400)))
        self.assertTrue(w._frame_is_complete(self._f(901)))


if __name__ == "__main__":
    unittest.main()
