# -*- coding: utf-8 -*-
"""
演示编排器测试（app/demo_director.py）。

这是"给领导看效果"那条链路的核心，出问题**不会报错**、只会表现成
"画面很假"（四路齐刷刷同步、或者车永远停在原地不动），所以必须钉死：

    ① 时间线：周期 30s、四段时长、相位边界、y 轨迹（驶入 20→0 / 驶离 0→20）
    ② 错开：四路各差 1/4 周期 → 任意时刻屏幕上同时有不同阶段
    ③ 交替：同一条车道一趟空车、一趟载货；相邻车道当场相反
    ④ 手动接管：pause() 之后绝不能再写数据源，否则看板按钮像坏了
    ⑤ 健壮性：某个数据源抛异常不能把编排线程带走（演示不能中途卡死）

全部用**假 worker**（ch.id + 受控set_vehicle_demo接口），不起服务、不收包，
所以毫秒级跑完。
"""
from __future__ import annotations

import time
import unittest
from types import SimpleNamespace

from tests import _helpers as H                       # noqa: F401 —— 加 sys.path
from app.demo_director import (CYCLE_SEC, OUT_Y, PHASES, DemoDirector)


# ═════════════════════════════════════════════ 假对象

class _FakeSource:
    """只记调用。set_vehicle 的语义细节在 radar_source 里单测，这里不重复。"""

    def __init__(self, boom: bool = False):
        self.calls = []
        self.boom = boom

    def set_vehicle(self, loaded=None, y_offset=None):
        if self.boom:
            raise RuntimeError("数据源炸了")
        self.calls.append({"loaded": loaded, "y_offset": y_offset})
        return True


class _NotSim:
    """没有 set_vehicle 的源（真雷达/回放）—— 编排器必须跳过它。"""


class _FakeWorker:

    def __init__(self, cid: str, source=None):
        self.ch = SimpleNamespace(id=cid)
        self.source = source if source is not None else _FakeSource()

    def set_vehicle_demo(self, loaded=None, y_offset=None):
        self.source.set_vehicle(loaded=loaded, y_offset=y_offset)
        return True


class _FakeBus:

    def __init__(self):
        self.events = []

    def publish(self, kind, payload=None):
        ev = dict(payload or {})
        ev["kind"] = kind
        self.events.append(ev)
        return ev

    def kinds(self):
        return [e["kind"] for e in self.events]

    def of(self, kind):
        return [e for e in self.events if e["kind"] == kind]


def _mk(n: int = 4, tick: float = 0.05):
    workers = [_FakeWorker("CH%d" % (i + 1)) for i in range(n)]
    bus = _FakeBus()
    return DemoDirector(workers, bus=bus, tick=tick), workers, bus


def _at(d: DemoDirector, t: float):
    """把编排器的时间指针拨到 t 秒并立刻推一帧（不启线程，测试才可控）。"""
    d._t0 = time.time() - t
    d._apply()


def _alive(d: DemoDirector) -> bool:
    t = d._thread
    return bool(t is not None and t.is_alive())


# ═════════════════════════════════════════════ ① 时间线

class TestTimeline(unittest.TestCase):

    def test_cycle_is_thirty_seconds(self):
        self.assertAlmostEqual(CYCLE_SEC, 30.0, places=6)
        self.assertAlmostEqual(sum(p.dur for p in PHASES), CYCLE_SEC, places=6)

    def test_phase_order_and_durations(self):
        self.assertEqual([p.name for p in PHASES],
                         ["空场等待", "车辆驶入", "停稳测量", "车辆驶离"])
        # 停稳测量必须最长：停稳检测要 2 帧确认 + settle 沉降窗口才出结论
        longest = max(PHASES, key=lambda p: p.dur)
        self.assertEqual(longest.name, "停稳测量")
        self.assertGreaterEqual(longest.dur, 10.0)

    def test_out_y_is_unambiguous(self):
        """OUT_Y 必须够远：实测 ≥8m 车点就掉到阈值以下，20m 留足余量。"""
        self.assertGreaterEqual(OUT_Y, 14.0)

    def test_phase_boundaries(self):
        d, _, _ = _mk(1)
        cases = [
            (0.0, "空场等待", 0.0),
            (4.9, "空场等待", 0.98),
            (5.0, "车辆驶入", 0.0),
            (10.9, "车辆驶入", 0.983),
            (11.0, "停稳测量", 0.0),
            (23.9, "停稳测量", 0.992),
            (24.0, "车辆驶离", 0.0),
            (29.9, "车辆驶离", 0.983),
            (30.1, "空场等待", 0.02),           # 绕回第二圈
        ]
        for t, name, frac in cases:
            p, f, _cycle, _left = d._phase_at(0.0, t)
            self.assertEqual(p.name, name, f"t={t}")
            self.assertAlmostEqual(f, frac, delta=0.01, msg=f"t={t} frac")

    def test_y_profile_drive_in_and_out(self):
        d, _, _ = _mk(1)
        # 空场：停在 OUT_Y
        self.assertAlmostEqual(d._phase_at(0.0, 2.0)[0].y_at(
            d._phase_at(0.0, 2.0)[1]), OUT_Y, places=6)
        # 驶入中段：介于 0 与 OUT_Y 之间
        y_mid = PHASES[1].y_at(0.5)
        self.assertAlmostEqual(y_mid, OUT_Y / 2.0, places=6)
        # 停稳：恒为 0（车停在检测位正中）
        self.assertAlmostEqual(PHASES[2].y_at(0.0), 0.0, places=6)
        self.assertAlmostEqual(PHASES[2].y_at(1.0), 0.0, places=6)
        # 驶离：0 → OUT_Y
        self.assertAlmostEqual(PHASES[3].y_at(0.0), 0.0, places=6)
        self.assertAlmostEqual(PHASES[3].y_at(1.0), OUT_Y, places=6)

    def test_y_at_clamps_frac(self):
        p = PHASES[1]
        self.assertAlmostEqual(p.y_at(-5.0), p.y_from, places=6)
        self.assertAlmostEqual(p.y_at(5.0), p.y_to, places=6)

    def test_cycle_counter_increments(self):
        d, _, _ = _mk(1)
        self.assertEqual(d._phase_at(0.0, 1.0)[2], 0)
        self.assertEqual(d._phase_at(0.0, 31.0)[2], 1)
        self.assertEqual(d._phase_at(0.0, 61.0)[2], 2)

    def test_new_vehicle_only_on_drive_in(self):
        """只有"驶入"这一段允许换车 —— 别的地方 bump generation 会清掉参考帧。"""
        self.assertEqual([p.name for p in PHASES if p.new_vehicle], ["车辆驶入"])


# ═════════════════════════════════════════════ ② 错开

class TestStagger(unittest.TestCase):

    def test_offsets_evenly_spaced(self):
        d, _, _ = _mk(4)
        offs = [off for _c, _w, off in d.targets]
        self.assertEqual(len(offs), 4)
        for i, off in enumerate(offs):
            self.assertAlmostEqual(off, CYCLE_SEC / 4 * i, places=6)

    def test_single_channel_offset_zero(self):
        d, _, _ = _mk(1)
        self.assertAlmostEqual(d.targets[0][2], 0.0, places=6)

    def test_non_sim_channels_are_skipped(self):
        """真雷达/回放没有 set_vehicle → 不能进编排目标，否则每 tick 静默失败。"""
        workers = [_FakeWorker("CH1"),
                   _FakeWorker("CH2", source=_NotSim()),
                   _FakeWorker("CH3"),
                   _FakeWorker("CH4", source=_NotSim())]
        d = DemoDirector(workers, bus=_FakeBus())
        cids = [cid for cid, _w, _o in d.targets]
        self.assertEqual(cids, ["CH1", "CH3"])
        # 只有 2 路可编排 → 错开按 1/2 周期
        self.assertAlmostEqual(d.targets[1][2], CYCLE_SEC / 2, places=6)

    def test_worker_without_source_is_skipped(self):
        """连 source 都没有（真雷达未接上）也不能把整件事带崩。"""
        w = _FakeWorker("CH9")
        w.source = None
        d = DemoDirector([w], bus=_FakeBus())
        self.assertEqual(d.targets, [])

    def test_zero_channels_is_noop(self):
        d = DemoDirector([_FakeWorker("CH1", source=_NotSim())], bus=_FakeBus())
        self.assertEqual(d.targets, [])
        d.start()                  # 不该起线程、不该炸
        self.assertIsNone(d._thread)
        d.stop()

    def test_at_any_instant_four_distinct_phases(self):
        """t=0 时四路应分别落在不同阶段（这是"四个车道"观感的前提）。"""
        d, _, _ = _mk(4)
        _at(d, 0.0)
        got = [d.snapshot_row("CH%d" % (i + 1))["phase"] for i in range(4)]
        self.assertEqual(got, ["空场等待", "车辆驶入", "停稳测量", "停稳测量"])
        # y 不能都一样（否则看起来像四张同步幻灯片）
        ys = {d.snapshot_row("CH%d" % (i + 1))["y_offset"] for i in range(4)}
        self.assertGreater(len(ys), 1)


# ═════════════════════════════════════════════ ③ 空车/载货交替

class TestLoadedAlternation(unittest.TestCase):

    def test_alternates_across_cycles(self):
        self.assertFalse(DemoDirector._loaded_for(0, 0))
        self.assertTrue(DemoDirector._loaded_for(1, 0))
        self.assertFalse(DemoDirector._loaded_for(2, 0))

    def test_neighbouring_channels_opposite(self):
        """同一时刻四路载货状态交替 → 画面混合（有的判空车、有的判载货），不像假的。"""
        for cycle in range(4):
            vals = [DemoDirector._loaded_for(cycle, i) for i in range(4)]
            for i in range(3):
                self.assertNotEqual(vals[i], vals[i + 1],
                                    f"cycle={cycle} 第 {i}/{i+1} 路状态相同：{vals}")

    def test_applied_to_source(self):
        d, workers, _ = _mk(4)
        _at(d, 0.0)
        # 每路都至少写了一次，且 loaded 是 bool
        for w in workers:
            self.assertTrue(w.source.calls)
            self.assertIsInstance(w.source.calls[-1]["loaded"], bool)


# ═════════════════════════════════════════════ ④ _apply / 暂停恢复

class TestApplyAndPause(unittest.TestCase):

    def test_apply_writes_position(self):
        d, workers, _ = _mk(4)
        _at(d, 0.0)
        self.assertTrue(all(w.source.calls for w in workers))
        row = d.snapshot_row("CH1")
        self.assertEqual(row["phase"], "空场等待")
        self.assertAlmostEqual(row["y_offset"], OUT_Y, places=3)
        self.assertIn("progress", row)
        self.assertIn("phase_left", row)

    def test_pause_stops_writing(self):
        d, workers, bus = _mk(4)
        _at(d, 0.0)
        n_before = len(workers[0].source.calls)
        d.pause("测试手动接管")
        _at(d, 20.0)                      # 时间拨到"停稳测量"，但已暂停
        self.assertEqual(len(workers[0].source.calls), n_before,
                         "暂停后仍在写数据源 —— 看板手动按钮会被瞬间覆盖")
        self.assertFalse(d.enabled)
        self.assertIn("warn", bus.kinds())

    def test_pause_is_idempotent(self):
        d, _, bus = _mk(2)
        d.pause("A")
        d.pause("B")
        self.assertEqual(len(bus.of("warn")), 1)
        self.assertEqual(d.status()["paused_reason"], "A")

    def test_resume_restores(self):
        d, workers, _ = _mk(4)
        d.pause("测试")
        _at(d, 0.0)
        n = len(workers[0].source.calls)
        d.resume()
        _at(d, 6.0)
        self.assertGreater(len(workers[0].source.calls), n)
        self.assertTrue(d.enabled)
        self.assertEqual(d.status()["paused_reason"], "")

    def test_set_enabled_false_then_true(self):
        d, workers, _ = _mk(4)
        d.set_enabled(False)
        _at(d, 0.0)
        self.assertEqual(len(workers[0].source.calls), 0)
        self.assertEqual(d.status()["switched_on"], False)
        d.set_enabled(True)
        _at(d, 0.0)
        self.assertTrue(workers[0].source.calls)
        self.assertEqual(d.status()["switched_on"], True)

    def test_enable_clears_pause(self):
        d, _, _ = _mk(2)
        d.pause("测试")
        d.set_enabled(True)
        self.assertTrue(d.enabled)

    def test_source_exception_does_not_kill_director(self):
        """某一路数据源炸了，其余通道必须照常演下去。"""
        workers = [_FakeWorker("CH1", source=_FakeSource(boom=True)),
                   _FakeWorker("CH2")]
        d = DemoDirector(workers, bus=_FakeBus())
        _at(d, 0.0)
        _at(d, 12.0)
        self.assertTrue(workers[1].source.calls, "健康通道被拖死了")
        # 坏通道不进状态表（continue 掉了），但整体没抛
        self.assertEqual(d.status()["n_channels"], 2)


# ═════════════════════════════════════════════ ⑤ 事件 / 状态 / 线程

class TestEventsAndStatus(unittest.TestCase):

    def test_phase_change_publishes_event(self):
        d, _, bus = _mk(4)
        _at(d, 0.0)                       # 首帧：四路各播一条
        self.assertEqual(len(bus.of("demo_phase")), 4)
        _at(d, 1.0)                       # 同相位内：不再播
        self.assertEqual(len(bus.of("demo_phase")), 4)
        _at(d, 5.2)                       # 跨过 5s 边界：CH1 进"驶入"、CH4 进"驶离"
        evs = bus.of("demo_phase")
        self.assertGreater(len(evs), 4, "相位切换没有播事件，看板事件日志会一直静止")
        for cid, phase in (("CH1", "车辆驶入"), ("CH4", "车辆驶离")):
            hit = [e for e in evs if e["channel"] == cid and e["phase"] == phase]
            self.assertTrue(hit, f"{cid} 没有播 {phase} 事件")
        # 同一路同一相位只播一次（不能每 tick 刷屏）
        keys = [(e["channel"], e["phase"], e["cycle"]) for e in evs]
        self.assertEqual(len(keys), len(set(keys)))

    def test_event_carries_channel_and_loaded(self):
        d, _, bus = _mk(4)
        _at(d, 0.0)
        for ev in bus.of("demo_phase"):
            self.assertIn(ev["channel"], ("CH1", "CH2", "CH3", "CH4"))
            self.assertIsInstance(ev["loaded"], bool)
            self.assertIn("y_offset", ev)

    def test_status_shape(self):
        d, _, _ = _mk(4)
        _at(d, 3.0)
        s = d.status()
        for k in ("auto", "switched_on", "paused_reason", "tick", "cycle_sec",
                  "phases", "out_y", "n_channels", "uptime", "channels"):
            self.assertIn(k, s)
        self.assertAlmostEqual(s["cycle_sec"], 30.0, places=1)
        self.assertEqual(s["n_channels"], 4)
        self.assertEqual(len(s["phases"]), 4)
        self.assertEqual(set(s["channels"]), {"CH1", "CH2", "CH3", "CH4"})

    def test_snapshot_row_missing_channel_is_empty(self):
        d, _, _ = _mk(2)
        self.assertEqual(d.snapshot_row("NOPE"), {})

    def test_snapshot_row_is_a_copy(self):
        d, _, _ = _mk(2)
        _at(d, 12.0)
        row = d.snapshot_row("CH1")
        row["phase"] = "被我改了"
        self.assertNotEqual(d.snapshot_row("CH1")["phase"], "被我改了")

    def test_start_applies_initial_pose_then_ticks(self):
        d, workers, _ = _mk(4, tick=0.05)
        d.start()
        try:
            # start() 里 force=True 先摆好姿态：不用等第一个 tick
            self.assertTrue(all(w.source.calls for w in workers))
            n = len(workers[0].source.calls)
            time.sleep(0.35)
            self.assertGreater(len(workers[0].source.calls), n,
                               "后台线程没有在 tick")
            self.assertIsNotNone(d._thread)
        finally:
            d.stop()
        self.assertIsNone(d._thread)
        self.assertFalse(_alive(d))

    def test_stop_is_idempotent(self):
        d, _, _ = _mk(2, tick=0.05)
        d.start()
        d.stop()
        d.stop()
        self.assertFalse(_alive(d))

    def test_stop_after_stop_does_not_hang(self):
        d, _, _ = _mk(2, tick=0.05)
        d.stop()                       # 没启动过也能安全停
        self.assertFalse(_alive(d))

    def test_no_bus_is_fine(self):
        d = DemoDirector([_FakeWorker("CH1")], bus=None)
        d.start()
        d.pause("无总线")
        d.resume()
        d.stop()

    def test_tick_floor(self):
        """tick 不能被设成 0（会变成忙等，饿死 GIL）。"""
        d = DemoDirector([_FakeWorker("CH1")], bus=None, tick=0.0)
        self.assertGreaterEqual(d.tick, 0.05)
        d2 = DemoDirector([_FakeWorker("CH1")], bus=None, tick=-3)
        self.assertGreaterEqual(d2.tick, 0.05)


if __name__ == "__main__":
    unittest.main()
