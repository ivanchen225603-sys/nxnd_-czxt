# -*- coding: utf-8 -*-
"""
服务编排测试：事件总线 + 会话状态机 + 场景代次切换。

这里最值得盯的是**场景切换那次的竞态**：
    仿真源切空车/载货时会自增 Frame.gen；服务必须在"拿到新场景的第一帧"时
    才重置停稳检测器。若在 set_loaded() 里立刻重置，旧场景在途的那一帧会被
    当成新参考帧，随后新场景的质心差 ~0.7m → 偏差永远大于阈值 → 卡在"运动中"。
    下面 test_scene_switch_recovers 就是钉这个行为。
"""
from __future__ import annotations

import os
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

from tests import _helpers as H
from app.service import EventBus, LidarService
from app.detect import StabilityState
from app.pointcloud import Frame


class TestEventBus(unittest.TestCase):

    def test_publish_subscribe(self):
        bus = EventBus()
        q1, q2 = bus.subscribe(), bus.subscribe()
        bus.publish("a", {"x": 1})
        bus.publish("b", {"y": 2})
        self.assertEqual(bus.subscribers, 2)
        self.assertEqual([e["kind"] for e in q1], ["a", "b"])
        self.assertEqual([e["kind"] for e in q2], ["a", "b"])
        # 序号单调递增
        self.assertLess(q1[0]["seq"], q1[1]["seq"])

    def test_unsubscribe(self):
        bus = EventBus()
        q = bus.subscribe()
        bus.unsubscribe(q)
        self.assertEqual(bus.subscribers, 0)
        bus.publish("a", {})
        self.assertEqual(len(q), 0)

    def test_bounded_queue_drops_oldest(self):
        bus = EventBus(capacity=3)
        q = bus.subscribe()
        for i in range(10):
            bus.publish("e", {"i": i})
        self.assertEqual(len(q), 3)
        self.assertEqual([e["i"] for e in q], [7, 8, 9])

    def test_payload_merged(self):
        bus = EventBus()
        q = bus.subscribe()
        ev = bus.publish("k", {"channel": "CH1"})
        self.assertEqual(ev["channel"], "CH1")
        self.assertIn("ts", ev)
        self.assertEqual(q[0]["channel"], "CH1")

    def test_four_lanes_publish_concurrently_without_loss_or_crosstalk(self):
        """★ 4 条车道线程同时发事件：不丢、不重号、不串道，发布后改原对象不影响已发出的事件。

        2026-09-23 把深拷贝挪到锁外（大结果不再让别的车道排队），这条钉住挪动没有破坏语义。
        """
        import threading
        bus = EventBus(capacity=100000)
        q = bus.subscribe()
        lanes, per_lane = ("CH1", "CH2", "CH3", "CH4"), 500
        barrier = threading.Barrier(len(lanes))

        def run(lane):
            payload = {"channel": lane, "i": 0, "cloud": [[0.0, 0.0, 0.0]] * 50}
            barrier.wait(5)
            for i in range(per_lane):
                payload["i"] = i
                payload["cloud"][0] = [float(i)] * 3
                bus.publish("state", payload)
                payload["cloud"][0] = [-1.0] * 3            # 发布之后改原对象，已发出的事件不许跟着变

        threads = [threading.Thread(target=run, args=(lane,)) for lane in lanes]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(30)
        events = list(q)
        self.assertEqual(len(events), len(lanes) * per_lane)
        self.assertEqual(sorted(e["seq"] for e in events), list(range(1, len(events) + 1)))
        self.assertEqual([e["seq"] for e in events], sorted(e["seq"] for e in events))   # 入队顺序 = 序号顺序
        for lane in lanes:
            mine = [e for e in events if e["channel"] == lane]
            self.assertEqual([e["i"] for e in mine], list(range(per_lane)))   # 每条道自己的顺序不乱
            self.assertTrue(all(e["cloud"][0] == [float(e["i"])] * 3 for e in mine))  # 保留发布那一刻的快照


class _DemoBase(unittest.TestCase):
    """起一个单通道仿真服务；用完必须停干净。

    注意：仿真场景是**静止**的，所以一个场景只会产出一次 result
    （停稳会话不结束就不会重复检测 —— 这正是原版 stable_save_max 的语义）。
    因此每个用例都要先 force_new_session() 开一个新会话再等结果，
    否则会互相把对方的结果吃掉，测试顺序一变就挂。
    """

    @classmethod
    def setUpClass(cls):
        st, ch, calib, dev, ac = H.ctx()
        cls.ch_id = ch.id
        cls.svc = LidarService(settings=st, save_override=False,
                               plate_mode="simulate", demo=True, only=[ch.id])
        cls.q = cls.svc.bus.subscribe()
        cls.svc.start()
        # 先跑通一次空车，确认链路可用（后面每个用例再各自开新会话）
        cls._first_result = cls._drain(timeout=25.0)

    @classmethod
    def tearDownClass(cls):
        cls.svc.stop()

    @classmethod
    def _drain(cls, timeout=25.0, want=("result",), source_generation=None, session=None):
        """收集事件直到出现 want 里的任一类型，或超时。返回 {kind: [events]}。"""
        got = {}
        t0 = time.time()
        while time.time() - t0 < timeout:
            while cls.q:
                ev = cls.q.popleft()
                # 控制可等待旧测量先完成；只验证指定新场景，不能把排队旧事件当新车。
                if (ev["kind"] == "result" and source_generation is not None
                        and ev.get("source_generation") != source_generation):
                    continue
                if ev["kind"] == "result" and session is not None and ev.get("session") != session:
                    continue
                got.setdefault(ev["kind"], []).append(ev)
                if ev["kind"] in want:
                    return got
            time.sleep(0.05)
        return got

    def fresh_result(self, timeout=25.0):
        """开一个新停稳会话并等一次 result。返回事件或 None。"""
        # 清掉积压，避免拿到上一轮的结果
        while self.q:
            self.q.popleft()
        w = self.svc.by_id(self.ch_id)
        with w._control_lock:
            self.assertTrue(w.force_new_session())
            expected_session = w.st.session + 1
        got = self._drain(timeout=timeout, want=("result",), session=expected_session)
        return got.get("result", [None])[-1]


class TestServiceDemo(_DemoBase):

    def test_01_demo_produces_result(self):
        r = self.fresh_result()
        self.assertIsNotNone(r, "25s 内没有 result 事件")
        self.assertEqual(r["channel"], self.ch_id)
        for k in ("reason", "empty", "n_points", "diagnostics", "planes"):
            self.assertIn(k, r)
        self.assertGreater(r["n_points"], 0)

    def test_02_result_has_six_planes(self):
        r = self.fresh_result()
        self.assertIsNotNone(r, "没拿到 result")
        for name in ("floor", "roof", "left", "right", "front", "back"):
            self.assertIn(name, r["planes"])

    def test_03_snapshot_shape(self):
        snap = self.svc.snapshot()
        self.assertIn("service", snap)
        self.assertIn("gate", snap)
        self.assertIn("channels", snap)
        self.assertEqual(len(snap["channels"]), 1)
        c = snap["channels"][0]
        for k in ("id", "online", "fps", "saved_count", "stability",
                  "calib_path", "angle_ok", "history"):
            self.assertIn(k, c)
        self.assertTrue(c["angle_ok"], "角度表校验未通过")

    def test_04_scene_switch_recovers(self):
        """
        切到载货场景后，偏差应当回落到阈值以下（即"新场景重新建立参考帧"），
        而不是卡在"运动中"。这是 Frame.gen 惰性重置要解决的问题。
        """
        while self.q:
            self.q.popleft()
        self.assertTrue(self.svc.set_loaded_demo(self.ch_id, True))
        generation = self.svc.by_id(self.ch_id).source.generation
        got = self._drain(timeout=25.0, want=("result",), source_generation=generation)
        self.assertIn("result", got, "切场景后 25s 内没有出新结果")
        r = got["result"][-1]
        self.assertLess(r["deviation"], 0.5,
                        f"切场景后偏差仍达 {r['deviation']} —— "
                        f"多半是场景代次重置没生效（见 service._tick）")
        # 收尾：切回空车，避免影响后续用例的语义
        self.svc.set_loaded_demo(self.ch_id, False)

    def test_05_loaded_scene_is_not_empty(self):
        """载货场景必须判为非空 —— 与空车场景形成对照。"""
        while self.q:
            self.q.popleft()
        self.svc.set_loaded_demo(self.ch_id, True)
        generation = self.svc.by_id(self.ch_id).source.generation
        got = self._drain(timeout=25.0, want=("result",), source_generation=generation)
        self.assertIn("result", got, "载货场景没拿到 result")
        r = got["result"][-1]
        self.assertFalse(r["empty"], f"载货场景竟然判成空车（理由 {r['reason']!r}）")
        self.svc.set_loaded_demo(self.ch_id, False)

    def test_06_force_new_session(self):
        w = self.svc.by_id(self.ch_id)
        self.assertIsNotNone(w)
        w.force_new_session()
        self.assertEqual(w._session_id, 0)
        self.assertEqual(w._detected_this_session, 0)

    def test_07_no_save_when_disabled(self):
        """save_override=False 时绝不能写盘（回放/演示不该污染 data/loaded）。"""
        w = self.svc.by_id(self.ch_id)
        self.assertFalse(w.save_enabled)
        self.assertEqual(w.st.saved_count, 0)

    def test_08_empty_scene_is_empty_or_reported(self):
        """空车场景必须给出一个明确理由（要么'空车'，要么有诊断的拒绝理由）。"""
        r = self.first_result()
        self.assertIsNotNone(r, "首次链路没有产出 result")
        self.assertTrue(r["reason"], "理由为空 —— 判定链没走到结论")

    def first_result(self):
        """setUpClass 里跑的那次空车结果。"""
        got = self._first_result
        return got.get("result", [None])[-1]


class TestServiceAutoDemo(unittest.TestCase):
    """
    auto_demo=True 时的**接线**测试。

    故意不做长时彩排（那是 tools/demo_rehearsal.py 的活），这里只钉：
        ① 编排器真被挂上、真在推进相位（接线断了看板就是四张静止卡片）
        ② 快照能如实报出"几路仿真 / 几路真雷达"（领导别把演示当已接硬件）
        ③ 手动点按钮后编排器让位；点"自动演示"能恢复

    用例带序号，因为共用同一个服务实例、且后面几个会改状态。
    每个改状态的用例都在收尾处把"自动演示"恢复成开，避免互相踩。
    """

    @classmethod
    def setUpClass(cls):
        st, ch, _, _, _ = H.ctx()
        cls.ch_id = ch.id
        cls.svc = LidarService(settings=st, save_override=False,
                               plate_mode="simulate", demo=True, only=[ch.id],
                               auto_demo=True)
        cls.svc.start()

    @classmethod
    def tearDownClass(cls):
        cls.svc.stop()

    def test_01_snapshot_has_director(self):
        s = self.svc.snapshot()
        d = s["director"]
        self.assertIsNotNone(d, "auto_demo=True 却没有编排器 —— 看板不会自己演")
        self.assertTrue(d["auto"])
        self.assertEqual(d["n_channels"], 1)
        self.assertAlmostEqual(d["cycle_sec"], 30.0, places=1)
        self.assertEqual([p["name"] for p in d["phases"]],
                         ["空场等待", "车辆驶入", "停稳测量", "车辆驶离"])

    def test_02_sim_and_live_channel_counting(self):
        s = self.svc.snapshot()
        self.assertEqual(s["service"]["sim_channels"], 1)
        self.assertEqual(s["service"]["live_channels"], 0,
                         "演示模式把通道算成了真雷达 —— 看板会谎报已接硬件")
        self.assertTrue(s["service"]["demo"])
        self.assertEqual(s["channels"][0]["source_kind"], "sim-radar")
        # 演示/回放不写盘，别污染 data/loaded
        self.assertEqual(s["service"]["saved_total"], 0)

    def test_03_channel_carries_demo_row(self):
        c = self.svc.snapshot()["channels"][0]
        self.assertIn("demo", c, "通道上没有演示剧本行 —— 看板显示不出阶段")
        for k in ("phase", "cycle", "progress", "y_offset", "loaded", "phase_left"):
            self.assertIn(k, c["demo"], f"演示剧本行缺字段 {k}")
        self.assertIn(c["demo"]["loaded"], (True, False))

    def test_04_vehicle_position_actually_moves(self):
        """编排器必须真的在写数据源位置 —— 否则车永远停在原地（"很假"）。"""
        svc = self.svc
        seen = set()
        t0 = time.time()
        while time.time() - t0 < 15.0:
            row = svc.snapshot()["channels"][0].get("demo") or {}
            seen.add(round(float(row.get("y_offset", -1)), 3))
            if len(seen) > 3:
                break
            time.sleep(0.4)
        self.assertGreater(len(seen), 3,
                           f"15s 内车辆 y 只有 {len(seen)} 个取值 —— 编排线程没在推：{seen}")

    def test_05_manual_switch_pauses_director(self):
        svc = self.svc
        self.assertTrue(svc.set_loaded_demo(self.ch_id, True))
        d = svc.snapshot()["director"]
        self.assertFalse(d["auto"], "手动切了场景，编排器却没让位 —— 按钮会被立刻覆盖")
        self.assertTrue(d["paused_reason"])
        svc.set_loaded_demo(self.ch_id, False)

    def test_06_auto_demo_toggle(self):
        svc = self.svc
        self.assertTrue(svc.set_auto_demo(False))
        self.assertFalse(svc.snapshot()["director"]["auto"])
        self.assertFalse(svc.snapshot()["director"]["switched_on"])
        self.assertTrue(svc.set_auto_demo(True))
        self.assertTrue(svc.snapshot()["director"]["auto"])

    def test_07_force_new_session_pauses(self):
        svc = self.svc
        n = svc.force_new_session(self.ch_id)
        self.assertEqual(n, 1)
        self.assertFalse(svc.snapshot()["director"]["auto"])
        svc.set_auto_demo(True)                       # 收尾：恢复自动演示
        self.assertTrue(svc.snapshot()["director"]["auto"])


class TestServiceNoAutoDemoByDefault(unittest.TestCase):
    """默认 demo=True 但 auto_demo=False —— 场景必须**静止**，否则用例之间互相踩。"""

    def test_no_director_unless_asked(self):
        st, ch, _, _, _ = H.ctx()
        svc = LidarService(settings=st, save_override=False, plate_mode="simulate",
                           demo=True, only=[ch.id])
        try:
            self.assertIsNone(svc.director, "demo=True 不该顺手起自动编排")
            self.assertIsNone(svc.snapshot()["director"])
            self.assertFalse(svc.set_auto_demo(True),
                             "没有编排器时应返回 False，而不是悄悄造一个")
        finally:
            svc.stop()


class TestChannelSnapshotCloud(unittest.TestCase):
    """轮询快照只展示当前在线帧，不更改检测云或读取历史帧充数。"""

    def setUp(self):
        st, ch, _, _, _ = H.ctx()
        self.svc = LidarService(settings=st, save_override=False, plate_mode="simulate",
                                demo=True, only=[ch.id])
        self.addCleanup(self.svc.stop)
        self.worker = self.svc.by_id(ch.id)

    def test_online_snapshot_contains_bounded_current_cloud(self):
        xyz = np.arange(9000, dtype=np.float32).reshape(3000, 3) / 1000.0
        self.worker.last_frame = Frame(xyz.copy())
        self.worker._last_frame_monotonic = time.monotonic()
        self.worker.st.online = True
        cloud = self.worker.snapshot()["cloud"]
        self.assertEqual(len(cloud), 2500)
        self.assertTrue(all(len(point) == 3 for point in cloud))
        np.testing.assert_allclose(cloud[0], xyz[0], atol=0.001)
        np.testing.assert_allclose(cloud[-1], xyz[-1], atol=0.001)
        np.testing.assert_array_equal(self.worker.last_frame.xyz, xyz)
        # 返回的是展示副本，调用方改它不能改掉正在检测的原始帧。
        cloud[0][0] = 999.0
        self.assertEqual(float(self.worker.last_frame.xyz[0, 0]), 0.0)

    def test_offline_snapshot_discards_last_cloud(self):
        self.worker.last_frame = Frame(np.ones((5, 3), np.float32))
        self.worker._last_frame_monotonic = time.monotonic()
        self.worker.st.online = True
        self.assertEqual(len(self.worker.snapshot()["cloud"]), 5)
        self.worker.st.online = False
        self.assertEqual(self.worker.snapshot()["cloud"], [])

    def test_online_without_a_frame_returns_no_cloud(self):
        self.worker.st.online = True
        self.worker.last_frame = None
        snapshot = self.worker.snapshot()
        self.assertEqual(snapshot["cloud"], [])
        self.assertIsNone(snapshot["frame_age_sec"])

    def test_online_but_no_new_frame_for_five_seconds_hides_cloud(self):
        self.worker.last_frame = Frame(np.ones((5, 3), np.float32))
        self.worker._last_frame_monotonic = 100.0
        self.worker.st.online = True
        with patch("app.service.time.monotonic", return_value=105.0):
            snapshot = self.worker.snapshot()
            self.assertEqual(snapshot["frame_age_sec"], 5.0)
            self.assertEqual(len(snapshot["cloud"]), 5)
        with patch("app.service.time.monotonic", return_value=105.01):
            snapshot = self.worker.snapshot()
            self.assertTrue(snapshot["online"])
            self.assertGreater(snapshot["frame_age_sec"], 5.0)
            self.assertEqual(snapshot["cloud"], [])

    def test_receiving_a_new_frame_resets_age(self):
        self.worker.st.online = True
        frame = Frame(np.zeros((3, 3), np.float32))
        with patch.object(self.worker.source, "read", return_value=frame), \
                patch("app.service.time.monotonic", return_value=123.0):
            self.assertTrue(self.worker._tick())
            snapshot = self.worker.snapshot()
        self.assertEqual(snapshot["frame_age_sec"], 0.0)
        self.assertEqual(len(snapshot["cloud"]), 3)

    def test_current_measurement_requires_fresh_same_session_and_origin(self):
        worker = self.worker
        worker.st.online = True
        worker.last_frame = Frame(np.ones((5, 3), np.float32))
        worker._last_frame_monotonic = time.monotonic()
        worker._session_id = 7
        worker._current_result_token = "origin-current"
        worker.st.last_result = dict(session=7, origin_token="origin-current",
                                     measurement_id="m-current")
        self.assertEqual(worker.snapshot()["current_result"], {"measurement_id": "m-current"})
        for field, value in (("session", 6), ("origin_token", "older")):
            original = worker.st.last_result[field]
            worker.st.last_result[field] = value
            self.assertIsNone(worker.snapshot()["current_result"])
            worker.st.last_result[field] = original
        worker._session_id = 0
        self.assertIsNone(worker.snapshot()["current_result"])
        worker._session_id = 7
        worker._last_frame_monotonic -= 6
        self.assertIsNone(worker.snapshot()["current_result"])
        self.assertEqual(worker.st.last_result["measurement_id"], "m-current")

    def test_current_measurement_requires_same_managed_passage(self):
        from unittest.mock import Mock
        worker = self.worker
        worker.st.online = True
        worker.last_frame = Frame(np.ones((5, 3), np.float32))
        worker._last_frame_monotonic = time.monotonic()
        worker._session_id = 7
        worker._current_result_token = "origin-current"
        worker.st.last_result = dict(session=7, origin_token="origin-current",
                                     measurement_id="m-current",
                                     integration=dict(status="queued", passageId="p-current"))
        worker.integration = Mock()
        try:
            for passage, expected in (("p-current", True), ("p-next", False), (None, False)):
                worker.integration.current_passage.return_value = passage
                self.assertEqual(worker.snapshot()["current_result"],
                                 {"measurement_id": "m-current"} if expected else None)
            worker.integration.current_passage.return_value = "p-current"
            worker.st.last_result["integration"]["status"] = "quarantined"
            self.assertIsNone(worker.snapshot()["current_result"])
            self.assertEqual(worker.st.last_result["measurement_id"], "m-current")
        finally:
            worker.integration = None


class TestCurrentResultSurvivesDriveOff(unittest.TestCase):
    """本车的结果不因"车起步"变成历史补交（2026-09-24 现场 CH4 实测）。

    现场时序：测完 -> 约 1 秒后车起步（没停稳的每一帧都走"作废观测窗口"）
    -> 约 3 秒后归档提交完成。原来作废观测窗口时无条件清"本车当前结果"标记，
    这份结果于是被当成历史补交：看板不出这辆车的结论，会话存盘数记成 0。
    只有换车（回到空闲 / 强开新会话）才该清。
    """

    def setUp(self):
        st, ch, _, _, _ = H.ctx()
        self.svc = LidarService(settings=st, save_override=False, plate_mode="simulate",
                                demo=True, only=[ch.id])
        self.addCleanup(self.svc.stop)
        self.worker = self.svc.by_id(ch.id)
        self.events = self.svc.bus.subscribe("test")
        self.addCleanup(self.svc.bus.unsubscribe, self.events)

    def _drain(self, *events):
        import json
        from types import SimpleNamespace
        pending = list(events)
        rows = []
        for kind, token, mid in pending:
            if kind == "durable":
                rows.append(dict(kind="durable", archived=True, managed=False,
                                 identity=SimpleNamespace(measurement_id=mid, passage_id="local-x"),
                                 result_json=json.dumps(dict(channel=self.worker.ch.id, measurement_id=mid,
                                                             origin_token=token, session=5))))
            else:
                rows.append(dict(kind="exported", measurement_id=mid, path="archive/" + mid))
        self.worker._finalizer = SimpleNamespace(poll=lambda channel: [rows.pop(0) for _ in list(rows)],
                                                 close=lambda timeout=None: True)
        self.worker._drain_finalizer_locked()
        return [ev["kind"] for ev in self.events]

    def test_result_stays_current_while_the_vehicle_drives_off(self):
        worker = self.worker
        worker._session_id = 5
        worker._current_result_token = "origin-A"
        for _ in range(3):
            worker._discard_observation_window()        # 车在动：没停稳的每一帧都走到这里
        self.assertEqual(worker._current_result_token, "origin-A")
        kinds = self._drain(("durable", "origin-A", "m-A"), ("exported", "origin-A", "m-A"))
        self.assertIn("result", kinds)
        self.assertNotIn("result_history", kinds)
        self.assertEqual(worker.st.last_result["measurement_id"], "m-A")
        self.assertEqual(worker._saved_this_session, 1)

    def test_vehicle_change_still_turns_a_late_result_into_history(self):
        worker = self.worker
        worker._session_id = 5
        worker._current_result_token = "origin-A"
        worker.st.last_result = dict(measurement_id="m-previous")
        worker._discard_observation_window(vehicle_changed=True)
        self.assertEqual(worker._current_result_token, "")
        kinds = self._drain(("durable", "origin-A", "m-A"), ("exported", "origin-A", "m-A"))
        self.assertIn("result_history", kinds)
        self.assertNotIn("result", kinds)
        self.assertEqual(worker.st.last_result["measurement_id"], "m-previous")
        self.assertEqual(worker._saved_this_session, 0)

    def test_forced_new_session_counts_as_a_vehicle_change(self):
        worker = self.worker
        worker._current_result_token = "origin-A"
        worker._force_new_session_locked()
        self.assertEqual(worker._current_result_token, "")


class TestServiceUnknownChannel(unittest.TestCase):

    def test_unknown_channel_returns_none(self):
        st, ch, _, _, _ = H.ctx()
        svc = LidarService(settings=st, save_override=False, plate_mode="simulate",
                           demo=True, only=[ch.id])
        self.assertIsNone(svc.by_id("NOPE"))
        self.assertFalse(svc.set_loaded_demo("NOPE", True))


class TestStandstillDiagnosis(unittest.TestCase):
    """★ 停稳归因必须出得来（2026-09-19）。

    现场报「行驶状态每秒都在变」时，日志与看板给的一直是旧的 `deviation`——
    而它按设计**早就不单独决定停稳**（见 docs/18），等于报了个不拦人的数，
    于是"卡在哪一条"始终查不出来。判据算出的 `triggers` 在此之前
    **在 app/ 里零消费者**：算完即丢。
    """

    def test_digest_ranks_and_caps(self):
        from app.service import trigger_digest
        self.assertEqual("", trigger_digest({}))
        self.assertEqual("", trigger_digest({"net": 0}), "零次的不该出现")
        self.assertEqual(
            "immediate x9 yaw_speed x5",
            trigger_digest({"yaw_speed": 5, "immediate": 9, "net": 1}, top=2),
            "应按次数降序并截断")

    def test_session_end_log_names_the_blocking_gates(self):
        from app.service import ConsoleLogger
        line = ConsoleLogger.format({"kind": "session_end", "channel": "CH1",
                                     "session": 7, "saved": 1,
                                     "triggers": {"immediate": 4, "net": 2}})
        self.assertIn("会话 #7", line)
        self.assertIn("immediate x4", line)
        self.assertIn("net x2", line)

    def test_session_end_without_triggers_is_unchanged(self):
        """没触发过就不该多打一截，日志要能看。"""
        from app.service import ConsoleLogger
        base = {"kind": "session_end", "channel": "CH1", "session": 7, "saved": 1}
        self.assertEqual(ConsoleLogger.format(base),
                         ConsoleLogger.format(dict(base, triggers={})))


class TestIncompleteFrameVisibility(unittest.TestCase):
    """★ 残缺帧占比必须可见（2026-09-19）。

    `_frame_is_complete` 的结果在此之前**只用不记**：残缺帧 -> `trusted=False`
    -> `StabilityDetector.invalidate()` -> 锚点清零 -> 攒不满连续停稳 ->
    **这辆车测不出来**，而看板上只表现为"一直运动中"。
    实测（车辆停留 40 s、10 Hz）：占比 <=3% 无影响、5% 掉到 84%、
    **10% 只剩 17% 的车测得成**。

    更麻烦的是它**不会自己恢复** —— `_frame_is_complete` 的包数基准
    只涨不降（残帧不进 `_pkt_hist`），所以转速变过或网络劣化之后，
    不重启该通道就永远测不出车。在此之前只有"车停够 30 s"才会报一次 warn，
    车道空着时一声不吭。
    """

    def worker(self, flags, base=450):
        from collections import deque
        from types import SimpleNamespace
        from app.service import ChannelWorker, ChannelState
        w = SimpleNamespace(
            st=ChannelState(id="CH1", name="一号道"),
            _complete_hist=deque(flags, maxlen=ChannelWorker._INCOMPLETE_WINDOW),
            _pkt_hist=deque([base] * 9),
            _INCOMPLETE_WINDOW=ChannelWorker._INCOMPLETE_WINDOW,
            _INCOMPLETE_WARN=ChannelWorker._INCOMPLETE_WARN,
            _INCOMPLETE_BAD=ChannelWorker._INCOMPLETE_BAD)
        w.st.incomplete_ratio = round(
            1.0 - sum(w._complete_hist) / float(len(w._complete_hist)), 4)
        return ChannelWorker._incomplete_alert(w), w.st.incomplete_ratio

    def flags(self, bad, total=200):
        return [i >= bad for i in range(total)]

    def test_clean_link_says_nothing(self):
        text, ratio = self.worker(self.flags(4))          # 2%
        self.assertEqual(0.02, ratio)
        self.assertEqual("", text, "干净链路不该刷横幅")

    def test_five_percent_raises_the_banner(self):
        text, ratio = self.worker(self.flags(12))         # 6%
        self.assertEqual(0.06, ratio)
        self.assertIn("残缺帧占比 6%", text)
        self.assertIn("450", text, "要把包数基准摊开写，否则现场无从下手")
        self.assertNotIn("需重启", text, "6% 还不到该让人重启的程度")

    def test_ten_percent_tells_the_operator_to_restart(self):
        """包数基准只涨不降，到这一档光等是等不回来的。"""
        text, _ = self.worker(self.flags(30))             # 15%
        self.assertIn("需重启本通道", text)

    def test_too_few_samples_draws_no_conclusion(self):
        text, _ = self.worker(self.flags(20, total=20))   # 全残但只有 20 帧
        self.assertEqual("", text, "样本太少不许下结论")

    def test_banner_uses_decision_median_not_middle_arrival(self):
        from collections import deque
        from types import SimpleNamespace
        from app.service import ChannelWorker
        worker = SimpleNamespace(
            _pkt_hist=deque([901] * 15 + [920] + [901] * 15, maxlen=31),
            _complete_hist=deque(self.flags(40), maxlen=200),
            st=SimpleNamespace(incomplete_ratio=.2),
            _INCOMPLETE_WINDOW=ChannelWorker._INCOMPLETE_WINDOW,
            _INCOMPLETE_WARN=ChannelWorker._INCOMPLETE_WARN,
            _INCOMPLETE_BAD=ChannelWorker._INCOMPLETE_BAD,
            _COMPLETE_RATIO=ChannelWorker._COMPLETE_RATIO)
        self.assertTrue(ChannelWorker._frame_is_complete(
            worker, SimpleNamespace(packets=901, dropped_packets=0)))
        # 重建同一份判定前历史，验证显示值与判据实际中位数一致。
        worker._pkt_hist = deque([901] * 15 + [920] + [901] * 15, maxlen=31)
        self.assertIn('包数基准 901', ChannelWorker._incomplete_alert(worker))


_FIELD_TRIPLETS = os.path.join(H.ROOT, "data", "raw-62", "CH2")
# 现场三连帧（约 489 MB）不在本仓库，见 AGENTS.md。数据不在就跳过，
# 但要说清楚缺的是什么，不要静悄悄绿过去。


@unittest.skipUnless(os.path.isdir(_FIELD_TRIPLETS),
                     "缺现场三连帧 data/raw-62/CH2（见 AGENTS.md）")
class TestWindowEvidence(unittest.TestCase):
    """★ 证据不足时，看看沉降窗口里其它帧（2026-09-19）。

    测量用的是窗口**末帧**。实测 62 组三连帧里 **8 组有证据不足的帧，
    其中 5 组恰好坏在末帧** —— 也就是这 5 次测量会报"证据不足"，
    而同一窗口里另外两帧其实都拿到了完整证据。**整组三帧全坏的是 0 组。**
    分不清这一点，现场就不知道该重测还是该拦截。
    """

    def worker(self, frames):
        from types import SimpleNamespace
        from app.service import ChannelWorker
        _, _, calib, dev, _ = H.ctx()
        return SimpleNamespace(_settle_frames=frames, calib=calib, dev=dev,
                               _VOL_EXTRA_MAX=ChannelWorker._VOL_EXTRA_MAX), ChannelWorker

    def load(self, name):
        from app.pointcloud import load_xyz, apply_roi
        _, _, calib, _, _ = H.ctx()
        return apply_roi(load_xyz(os.path.join(
            H.ROOT, "data/raw-62/CH2", name)), calib.roi)

    def test_costs_nothing_unless_evidence_is_missing(self):
        """只有 NO_EVIDENCE 才跑 —— 检测链 50~70 ms，平时一次都不许多花。"""
        from app.detect import Diagnostics
        w, cw = self.worker([object(), object()])
        for step in (6, 7, 10, 11):
            self.assertIsNone(cw._window_evidence(w, Diagnostics(step=step), None),
                              "第 %d 步不该触发窗口复核" % step)

    def test_names_how_many_other_frames_had_evidence(self):
        from app.detect import detect_empty, evaluate_empty
        _, _, calib, dev, _ = H.ctx()
        # 这一组实测：1of3 停在第 7 步（证据充分），2of3/3of3 停在第 3 步。
        first = self.load("CH2_20260915_111311_1of3.xyz")
        last = self.load("CH2_20260915_111311_3of3.xyz")
        res = detect_empty(last, calib)
        _ok, _reason, dg = evaluate_empty(res, last.n,
                                          res.cloud if res else last.xyz, dev)
        self.assertEqual("NO_EVIDENCE", dg.verdict, "末帧应当是证据不足")
        w, cw = self.worker([first, last])
        out = cw._window_evidence(w, dg, last)
        self.assertEqual(1, out["frames"])
        self.assertEqual(1, out["with_evidence"])
        self.assertIn("可重测而非拦截", out["hint"])

    def test_full_chain_reads_the_live_window_not_a_cleared_one(self):
        """★ 主链路必须读到**真实的**沉降窗口（2026-09-23，两轮审查 H1）。

        原实现把 _settle_clear() 放在 _run_detection **之前**，检测里的
        window_evidence / empty_evidence 恒读到空列表 —— 「同窗口其它帧
        有没有证据」修了但没接通。上面两条用例都伪造 _settle_frames 直调
        方法本体，守住了函数没守住链路；这条走 _tick_observation 全链路：
        真停稳、真开窗、真测量，不许再靠伪造状态过关。
        """
        import copy
        import shutil
        import tempfile
        from app.service import ChannelWorker, EventBus
        from tests._observations import ObservationClock

        settings, ch, _, _, _ = H.ctx()
        ch = copy.deepcopy(ch)
        ch.id, ch.mode = "CH1", "synthetic"
        tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, tmp, True)
        ch.abs_output_dir = tmp
        ch.abs_baseline_dir = tmp
        worker = ChannelWorker(ch, EventBus(), settings.plate,
                               save_override=False, plate_mode="null")
        self.addCleanup(worker.stop)
        self.addCleanup(worker.source.close)
        worker.settle_sec = 0.6
        queue = worker.bus.subscribe()
        # 18000 点静止"车厢"云：车点高于 vehicle_threshold(15000) 可判有车、
        # 完全静止可判稳；总点数低于 min_save_points(20000) -> 判定链停在第 1 步
        # -> NO_EVIDENCE，正好落进 window_evidence 的触发条件（只有证据不足才复核窗口）。
        rng = np.random.default_rng(20260923)
        frame = Frame(np.column_stack((rng.uniform(-1, 1, 18000),
                                       rng.uniform(-5, 5, 18000),
                                       rng.uniform(-3.7, -2, 18000))))
        clock = ObservationClock(self)
        clock.settle(worker, frame)
        self.assertIsNotNone(worker._settle_t0, "应当已进入沉降窗口")
        self.assertGreaterEqual(len(worker._settle_frames), 1)

        for _ in range(10):               # 假时钟 0.1s/帧 -> 窗口期满自动测量
            clock.tick(worker, frame)
        results = [e for e in queue if e["kind"] == "result"]
        self.assertTrue(results, "窗口期满后必须发布一次测量结果")
        event = results[-1]
        self.assertEqual("NO_EVIDENCE", event["verdict"])
        evidence = event["window_evidence"]
        self.assertIsNotNone(
            evidence, "主链路上的窗口证据必须读到同窗口的其它帧 —— "
                      "修复前 _settle_clear() 先于检测执行，这里恒为 None")
        self.assertGreaterEqual(evidence["frames"], 1)
        # 检测结束后窗口必须收口，不能留半开窗口影响后续会话。
        self.assertIsNone(worker._settle_t0)
        self.assertEqual(worker._settle_frames, [])


if __name__ == "__main__":
    unittest.main()

class TestEnterFrames(unittest.TestCase):
    """★ 进场帧采集（service.enter_frames）—— 只采不判，且默认关闭。

    为什么要这个功能：车停到位时车厢两端离天底 5 m 以上、俯角只剩 16~28 度，
    前端最外 1.5 m 覆盖只有 50%（2026-09-20 两辆磅单真值车实测）；而车往里开
    时前端会经过天底附近，回退 2.0~2.5 m 就能补到 96~99%。沉降窗口那 3 帧
    补不回来（车不动，落点几乎重合）。见 docs/41 第 0h 节。

    为什么必须默认关闭：它每车要写几十帧、上百 MB，而且配准精度还没实测过。
    """

    def _worker(self, **kw):
        import copy, tempfile
        from app.service import ChannelWorker, EventBus
        from unittest.mock import patch
        from app.radar_source import build_source
        from tests import _helpers as H
        settings, ch, _, _, _ = H.ctx()
        ch = copy.deepcopy(ch)
        ch.id, ch.mode = "CH1", "synthetic"
        self._tmp = tempfile.mkdtemp()
        ch.abs_output_dir = self._tmp

        def syn(*a, **k):
            return build_source(*a, **dict(k, mode="synthetic"))
        save_override = kw.pop("save_override", True)
        kw.setdefault("enter_step_m", 0.0)  # 旧采集用例显式走逐帧口径。
        kw.setdefault("enter_crop_roi", False)
        with patch("app.application.channel_worker.build_source", side_effect=syn):
            return ChannelWorker(ch, EventBus(), settings.plate, save_override=save_override,
                                 plate_mode="null", **kw)

    class _Stab:
        def __init__(self, n):
            self.n_vehicle_points = n

    def _pair(self, worker, n=500):
        import numpy as np
        from app.pointcloud import Frame

        class _Raw:
            def __init__(self, f):
                self._f = f

            def frame(self):
                return self._f
        cloud = np.zeros((n, 3), np.float32)
        cloud[:, 2] = -1.
        shot = _Raw(Frame(cloud))
        worker._current_pair = type("P", (), {"raw": shot, "cropped": shot})()

    def test_default_off_collects_nothing(self):
        """不给 enter_frames 就是关闭 —— 行为与从前逐位一致。"""
        w = self._worker()
        self.assertEqual(w.enter_frames, 0)
        self._pair(w)
        for _ in range(50):
            w._grab_enter(self._Stab(w.stability.vehicle_threshold + 1), True)
        self.assertEqual(len(w._enter_shots), 0)
        w._flush_enter()
        self.assertFalse(os.path.isdir(os.path.join(self._tmp, "enter")))

    def test_no_save_never_collects_or_writes(self):
        """落盘关闭时即使配置了进场帧，也不应生成仿真文件。"""
        w = self._worker(enter_frames=10, save_override=False)
        self.assertFalse(w.source.keep_raw)
        self._pair(w)
        for _ in range(3):
            w._grab_enter(self._Stab(w.stability.vehicle_threshold + 1), True)
        w._flush_enter()
        self.assertEqual(len(w._enter_shots), 0)
        self.assertFalse(os.path.isdir(os.path.join(self._tmp, "enter")))

    def test_step_keeps_moving_positions_and_skips_stationary_frames(self):
        """两种纵向位置证据任一挪够门槛才留；停住后的重复帧不挤走进场帧。"""
        from app.pointcloud import Frame
        from types import SimpleNamespace

        w = self._worker(enter_frames=10, enter_step_m=0.10, enter_crop_roi=True)
        stab = self._Stab(w.stability.vehicle_threshold + 1)
        for seq, dy in enumerate((0., .04, .11, .15, .23, .23, .23), 1):
            points = np.zeros((500, 3), np.float32)
            points[:, 1] = np.linspace(-1., 1., 500) + dy
            points[:, 2] = -1.
            shot = SimpleNamespace(frame=lambda points=points, seq=seq: Frame(points, seq=seq))
            w._current_pair = SimpleNamespace(raw=shot, cropped=shot)
            w._grab_enter(stab, True)
        self.assertEqual([shot.seq for shot in w._enter_shots], [1, 3, 5])
        self.assertEqual(w._enter_seen, 7)
        self.assertEqual(w._enter_selected, 3)

    def test_crop_uses_detection_roi_without_changing_raw_frame(self):
        """进场文件取同次 ROI 副本；未裁原帧仍留给独立诊断。"""
        from app.pointcloud import Frame
        from types import SimpleNamespace

        w = self._worker(enter_frames=3, enter_crop_roi=True)
        raw = Frame(np.array([[0., 0., -1.], [20., 0., -1.]], np.float32))
        cropped = Frame(np.array([[0., 0., -1.]], np.float32))
        w._current_pair = SimpleNamespace(
            raw=SimpleNamespace(frame=lambda: raw),
            cropped=SimpleNamespace(frame=lambda: cropped))
        w._grab_enter(self._Stab(w.stability.vehicle_threshold + 1), True)
        self.assertEqual(w._enter_shots[0].n, 1)
        self.assertEqual(raw.n, 2)

    def test_forced_session_and_single_frame_do_not_write(self):
        """超时强开视为同车重测；单帧也不足以形成进场轨迹。"""
        w = self._worker(enter_frames=3)
        self._pair(w)
        stab = self._Stab(w.stability.vehicle_threshold + 1)
        w._session_forced = True
        w._grab_enter(stab, True)
        self.assertEqual(len(w._enter_shots), 0)
        w._session_forced = False
        w._grab_enter(stab, True)
        w._flush_enter()
        self.assertTrue(w._enter_done)
        self.assertFalse(os.path.isdir(os.path.join(self._tmp, "enter")))

    def test_buffer_accumulates_while_the_vehicle_is_moving_through_the_real_tick(self):
        """★★ 走**真实的 _tick_observation**：车在动的每一帧都不许把进场缓冲清空。

        2026-09-23 查实的缺陷：`_settle_clear()` 里顺手 `_enter_shots.clear()`，
        而车没停稳的每一帧都会走 `elif not stab.stable: _discard_observation_window()`
        -> `_settle_clear()`，紧接着 `_grab_enter` 再追加 1 帧 —— 90 帧的环形缓冲
        **永远只剩最新 1 帧**。仿真实测：进场 8 个位置缓存一直是 1、落盘 2 帧且都在
        停稳位置；修复后落盘 24 帧、来自 9 个不同进场位置。

        本类其余用例都**直接调 _grab_enter**，从不经过那条清缓冲的链路，
        所以一直是绿的 —— 与 H1「守住了函数、没守住链路」同一个坑。
        这条必须让帧真正走 `_tick_observation`。
        """
        import numpy as np
        from app.pointcloud import Frame
        from tests._observations import ObservationClock
        w = self._worker(enter_frames=90, enter_min_points=500,
                         enter_step_m=0.10, enter_crop_roi=True)
        clock = ObservationClock(self)
        r = np.random.default_rng(3)
        # 一辆车沿 +y 一步步开进来：每帧整体平移 0.4 m，停稳判据不会放行
        # 点数必须超过有车阈值（vehicle_threshold），否则停稳层判「无车」、
        # 根本不开会话，帧也就到不了 _grab_enter —— 那样用例会空转通过
        n = w.stability.vehicle_threshold * 2
        base = np.column_stack([r.uniform(-1.2, 1.2, n), r.uniform(-6, 6, n),
                                r.uniform(-4.2, -2.5, n)])
        seen = []
        for step in range(12):
            # packets=900：与本仓库其它链路用例一致，表示"完整一圈" —— 不设的话
            # 完整性判定与 ObservationPair.freeze 都不放行，帧根本到不了 _grab_enter
            frame = Frame((base + [0.0, -6.0 + 0.4 * step, 0.0]).astype(np.float32), packets=900)
            clock.tick(w, frame)
            seen.append(len(w._enter_shots))
        self.assertFalse(w.st.stability.get("stable"), "前提：车一直在动，不该判成停稳")
        self.assertGreater(max(seen), 3,
                           "车在动的过程中进场缓冲没攒起来（缓存序列 %s）—— "
                           "又被每帧清空了" % seen)
        self.assertEqual(sorted(seen), seen, "进场过程中缓存数不该回落（被清空过）：%s" % seen)

    def test_backend_preparation_wait_does_not_discard_enter_frames(self):
        """后台准备未完成只阻止测量，不应丢失正在进场的完整原始帧。"""
        from types import SimpleNamespace
        from tests._observations import ObservationClock

        w = self._worker(enter_frames=20, enter_min_points=500)
        w.integration = SimpleNamespace(
            observe=lambda *_: None,
            current_passage=lambda *_: "passage-pending",
            claim_command_fast=lambda *_: None,
            preparation_status=lambda *_: "waiting_plate")
        clock = ObservationClock(self)
        rng = np.random.default_rng(31)
        n = w.stability.vehicle_threshold * 2
        base = np.column_stack([rng.uniform(-1.2, 1.2, n),
                                rng.uniform(-6., 6., n),
                                rng.uniform(-4.2, -2.5, n)])
        for step in range(12):
            frame = Frame((base + [0., -6. + .4 * step, 0.]).astype(np.float32),
                          packets=900)
            clock.tick(w, frame)
        self.assertEqual(w.st.last_diag.get("preparation"), "waiting_plate")
        self.assertGreater(len(w._enter_shots), 3)
        self.assertEqual(w._detected_this_session, 0)

    def test_preentry_frames_survive_session_start_and_clear_at_empty_lane(self):
        """2,000 点门槛的早期帧要跨过 15,000 点建车次时保留。"""
        from tests._observations import ObservationClock

        w = self._worker(enter_frames=20, enter_min_points=2000)
        self.assertGreater(w.stability.vehicle_threshold, w.enter_min_points)
        clock = ObservationClock(self)
        rng = np.random.default_rng(32)

        def frame(n, dy):
            points = np.column_stack([rng.uniform(-1., 1., n),
                                      rng.uniform(-4., 4., n) + dy,
                                      np.full(n, -3.)]).astype(np.float32)
            return Frame(points, packets=900)

        for step in range(5):
            clock.tick(w, frame(3000, step * .2))
        self.assertEqual(w._session_id, 0)
        self.assertEqual(len(w._enter_shots), 5)
        early_seq = [shot.seq for shot in w._enter_shots]
        clock.tick(w, frame(w.stability.vehicle_threshold * 2, 1.))
        self.assertNotEqual(w._session_id, 0)
        self.assertEqual([shot.seq for shot in list(w._enter_shots)[:5]], early_seq)
        clock.tick(w, frame(100, 0.))
        self.assertEqual(w._session_id, 0)
        self.assertEqual(len(w._enter_shots), 0)
        clock.tick(w, frame(3000, 0.))
        self.assertEqual(len(w._enter_shots), 1)
        stale_seq = w._enter_shots[0].seq
        clock.advance(w.calib.detection.stationary_max_gap_sec + 1.)
        clock.tick(w, frame(w.stability.vehicle_threshold * 2, 0.))
        self.assertNotIn(stale_seq, [shot.seq for shot in w._enter_shots],
                         "断流后的预进场帧不许归到下一车次")

    def test_a_new_vehicle_starts_with_an_empty_buffer(self):
        """进场缓冲属于**一辆车**：换车（开新会话）时必须清，不能把上一辆的帧混进来。"""
        w = self._worker(enter_frames=10)
        self._pair(w)
        stab = self._Stab(w.stability.vehicle_threshold + 1)
        for _ in range(5):
            w._grab_enter(stab, True)
        self.assertEqual(5, len(w._enter_shots))
        w._force_new_session_locked()
        self.assertEqual(0, len(w._enter_shots), "换车后仍留着上一辆车的进场帧")
        self.assertFalse(w._enter_done)

    def test_settle_clear_no_longer_wipes_the_enter_buffer(self):
        """沉降窗口作废（车又动了）只清沉降状态，不碰进场缓冲。"""
        w = self._worker(enter_frames=10)
        self._pair(w)
        stab = self._Stab(w.stability.vehicle_threshold + 1)
        for _ in range(4):
            w._grab_enter(stab, True)
        w._settle_clear()
        self.assertEqual(4, len(w._enter_shots), "沉降窗口作废把进场缓冲也清了")

    def test_ring_buffer_keeps_the_latest_and_honours_stride(self):
        """环形缓冲只留最近 N 帧；stride 控制稀疏度。"""
        w = self._worker(enter_frames=4, enter_stride=3)
        self._pair(w)
        stab = self._Stab(w.stability.vehicle_threshold + 1)
        for _ in range(30):
            w._grab_enter(stab, True)
        self.assertEqual(len(w._enter_shots), 4)          # 只留最近 4 帧
        self.assertEqual(w._enter_seen, 30)

    def test_trigger_threshold_is_configurable_and_much_lower(self):
        """★★ 采集起点必须能配得比 vehicle_threshold 低得多。

        2026-09-21 实测（43 个会话）：车进 ROI 之后**只走约 1.6 m 就停了**，
        判稳前最后 3 秒速度几乎为 0（车早停住了，在等连续停稳计时）。
        逐档实测判稳前能回退的距离：
            30 帧(3 s) 0.01 m   60 帧(6 s) 1.27 m   90 帧(9 s) 1.55 m
            120 帧(12 s) 1.57 m —— 到顶
        用 vehicle_threshold(15000) 当采集起点更晚，**整段进场都采不到**。

        ▲▲ 上面这组"回退距离"**对不上**（2026-09-23 复算）：它来自停稳日志
          `motion_20260921.csv` 的速度积分，与进场缓冲缺陷无关；但按时间窗重算，
          判稳前 3 s 0.03 m、6 s 0.07 m、9 s 1.00 m、12 s 3.31 m（中位），
          并不"到顶"。本用例要守的"采集起点可配得更低"仍然成立。见 docs/60 第 3.5 节。
        """
        w = self._worker(enter_frames=5, enter_min_points=2000)
        self.assertEqual(w.enter_min_points, 2000)
        self._pair(w)
        thr = w.stability.vehicle_threshold
        self.assertGreater(thr, 2000, "本用例假定 vehicle_threshold 远高于 2000")
        for _ in range(8):                      # 点数够不到 vehicle_threshold，但过了新门槛
            w._grab_enter(self._Stab(3000), True)
        self.assertEqual(len(w._enter_shots), 5, "低门槛下就该开始采了")
        # 0 = 回退到 vehicle_threshold（原行为）
        w2 = self._worker(enter_frames=5)
        self._pair(w2)
        for _ in range(8):
            w2._grab_enter(self._Stab(3000), True)
        self.assertEqual(len(w2._enter_shots), 0, "不给 min_points 时仍按 vehicle_threshold")

    def test_no_vehicle_and_degraded_frames_are_skipped(self):
        """没车不采；残缺帧一律跳过 —— 理由与 _grab_raw 相同。"""
        w = self._worker(enter_frames=5)
        self._pair(w)
        for _ in range(10):                                # 车辆点不足
            w._grab_enter(self._Stab(w.stability.vehicle_threshold - 1), True)
        self.assertEqual(len(w._enter_shots), 0)
        for _ in range(10):                                # 残缺帧
            w._grab_enter(self._Stab(w.stability.vehicle_threshold + 1), False)
        self.assertEqual(len(w._enter_shots), 0)

    def test_flush_writes_files_once_per_session(self):
        """判稳那一刻整批落盘；同一会话只落一次，不进归档。"""
        import glob
        import time as _t
        w = self._worker(enter_frames=3)
        self._pair(w)
        stab = self._Stab(w.stability.vehicle_threshold + 1)
        for _ in range(6):
            w._grab_enter(stab, True)
        self.assertEqual(len(w._enter_shots), 3)
        w._flush_enter()
        for _ in range(100):                               # 后台线程写盘
            xyz = glob.glob(os.path.join(self._tmp, "enter", "CH1", "*.xyz"))
            if len(xyz) == 3:
                break
            _t.sleep(.05)
        self.assertEqual(len(xyz), 3, "进场帧应落盘 3 帧")
        self.assertEqual(len(glob.glob(os.path.join(self._tmp, "enter", "CH1", "*.meta.json"))), 3)
        import json
        meta_path = glob.glob(os.path.join(self._tmp, "enter", "CH1", "*.meta.json"))[0]
        with open(meta_path, encoding="utf-8") as fh:
            meta = json.load(fh)
        self.assertEqual(meta["source_mode"], "synthetic")
        self.assertTrue(meta["complete_gate_passed"])
        self.assertEqual(meta["capture_instance_id"], w._capture_instance_id)
        self.assertEqual(meta["eligible_frames_seen"], 6)
        self.assertEqual(meta["selected_frames_total"], 6)
        self.assertTrue(meta["buffer_truncated"])
        self.assertIn("dropped_packets", meta)
        self.assertTrue(w._enter_done)
        w._flush_enter()                                   # 再调一次不应重复写
        self.assertEqual(len(glob.glob(os.path.join(self._tmp, "enter", "CH1", "*.xyz"))), 3)

    def test_flush_records_backend_passage_without_measuring(self):
        """进场旁注绑定后台车次，原始帧仍不进入测量载荷。"""
        import glob
        import json
        from types import SimpleNamespace

        w = self._worker(enter_frames=2, enter_min_points=500)
        w.integration = SimpleNamespace(current_passage=lambda *_: "p-current")
        w._session_id = 7
        self._pair(w)
        w._grab_enter(self._Stab(501), True)
        w._grab_enter(self._Stab(501), True)
        w._flush_enter()
        meta = None
        for _ in range(100):
            paths = glob.glob(os.path.join(self._tmp, "enter", "CH1", "*.meta.json"))
            if paths:
                try:
                    with open(paths[0], encoding="utf-8") as fh:
                        meta = json.load(fh)
                    break
                except json.JSONDecodeError:
                    pass
            time.sleep(.01)
        self.assertIsNotNone(meta)
        self.assertEqual(meta["passage_id"], "p-current")
        self.assertEqual(meta["session_id"], 7)
        self.assertFalse(meta["buffer_truncated"])


class TestExitFrames(unittest.TestCase):
    """★ 离场帧采集（service.exit_frames，2026-10-02）—— 只采不判，默认关闭。

    与进场帧对称：停稳时车厢后端也扫不全（9-28 两车停稳 3 帧后端 2 m 覆盖 54%），
    车测完往前开走时后端经过雷达附近。只在**本车测过之后**、车在动时按位移留，
    只留最先的 exit_frames 帧，车走（回到空闲）时整批落盘到 exit/<通道>/。
    """

    def _worker(self, **kw):
        import copy, tempfile
        from app.service import ChannelWorker, EventBus
        from unittest.mock import patch
        from app.radar_source import build_source
        from tests import _helpers as H
        settings, ch, _, _, _ = H.ctx()
        ch = copy.deepcopy(ch)
        ch.id, ch.mode = "CH1", "synthetic"
        self._tmp = tempfile.mkdtemp()
        ch.abs_output_dir = self._tmp

        def syn(*a, **k):
            return build_source(*a, **dict(k, mode="synthetic"))
        save_override = kw.pop("save_override", True)
        with patch("app.application.channel_worker.build_source", side_effect=syn):
            worker = ChannelWorker(ch, EventBus(), settings.plate, save_override=save_override,
                                   plate_mode="null", **kw)
        worker._local_passage = "test-exit-passage"
        return worker

    def _shot(self, w, dy, seq):
        from types import SimpleNamespace
        from app.pointcloud import Frame
        points = np.zeros((500, 3), np.float32)
        points[:, 1] = np.linspace(-1., 1., 500) + dy
        points[:, 2] = -1.
        shot = SimpleNamespace(frame=lambda: Frame(points, seq=seq))
        w._current_pair = SimpleNamespace(raw=shot, cropped=shot)

    def _stab(self, w):
        from types import SimpleNamespace
        return SimpleNamespace(n_vehicle_points=w.stability.vehicle_threshold + 1)

    def _wait_files(self, pattern, timeout=5.0):
        import glob
        import time
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            found = sorted(glob.glob(pattern))
            if found:
                return found
            time.sleep(.01)
        return []

    def test_default_off_and_not_before_the_measurement(self):
        w = self._worker()
        self.assertEqual(0, w.exit_frames)
        w._mark_exit_eligible("test-measurement")
        for i in range(5):
            self._shot(w, .2 * i, i)
            w._grab_exit(self._stab(w), True)
        self.assertEqual([], w._exit_shots)
        w = self._worker(exit_frames=5)
        for i in range(5):                      # 本车还没测过：进场阶段不算离场
            self._shot(w, .2 * i, i)
            w._grab_exit(self._stab(w), True)
        self.assertEqual([], w._exit_shots)

    def test_keeps_the_first_frames_by_displacement(self):
        """按位移留帧、只留最先的 exit_frames 帧；停住的重复帧与残缺帧不留。"""
        w = self._worker(exit_frames=3, exit_step_m=0.10)
        w._mark_exit_eligible("test-measurement")
        for seq, dy in enumerate((0., .03, .12, .12, .30, .45, .60), 1):
            self._shot(w, dy, seq)
            w._grab_exit(self._stab(w), complete=(seq != 5))
        self.assertEqual([1, 3, 6], [s.seq for s in w._exit_shots])

    def test_flushes_on_drive_off_and_resets(self):
        w = self._worker(exit_frames=5)
        w._mark_exit_eligible("test-measurement")
        w._session_id = 4
        for i in range(4):
            self._shot(w, .2 * i, i)
            w._grab_exit(self._stab(w), True)
        w._flush_exit()
        self.assertEqual([], w._exit_shots)
        metas = self._wait_files(os.path.join(self._tmp, "exit", "CH1", "*_1of4.xyz.meta.json"))
        self.assertEqual(1, len(metas))
        import json
        import time
        for _ in range(100):
            try:
                with open(metas[0], encoding="utf-8") as fh:
                    meta = json.load(fh)
                break
            except json.JSONDecodeError:
                time.sleep(.01)
        self.assertEqual((4, 4, False), (meta["frames"], meta["session_id"], meta["buffer_full"]))
        self.assertIsNone(w._exit_last_position)

    def test_manual_reset_and_no_save_write_nothing(self):
        w = self._worker(exit_frames=5)
        w._mark_exit_eligible("test-measurement")
        w.force_new_session()
        self._shot(w, 0., 1)
        w._grab_exit(self._stab(w), True)
        self.assertEqual([], w._exit_shots)
        w = self._worker(exit_frames=5, save_override=False)
        w._mark_exit_eligible("test-measurement")
        for i in range(3):
            self._shot(w, .2 * i, i)
            w._grab_exit(self._stab(w), True)
        w._flush_exit()
        self.assertFalse(os.path.isdir(os.path.join(self._tmp, "exit")))

    def test_drive_off_through_the_real_tick(self):
        """★ 走真实的 _tick_observation：测过的车往 +y 开走、车道回到空闲 -> 离场帧落盘。"""
        from app.pointcloud import Frame
        from tests._observations import ObservationClock
        w = self._worker(exit_frames=10, exit_step_m=0.10)
        clock = ObservationClock(self)
        rng = np.random.default_rng(7)
        n = w.stability.vehicle_threshold * 2
        base = np.column_stack([rng.uniform(-1.2, 1.2, n), rng.uniform(-6., 6., n),
                                rng.uniform(-4.2, -2.5, n)])
        clock.tick(w, Frame(base.astype(np.float32), packets=900))
        self.assertNotEqual(0, w._session_id)
        w._mark_exit_eligible("test-measurement")               # 本车已测过（测量链本身不在这条用例里）
        for step in range(1, 8):
            clock.tick(w, Frame((base + [0., .4 * step, 0.]).astype(np.float32), packets=900))
        self.assertGreater(len(w._exit_shots), 3)
        empty = np.column_stack([rng.uniform(-1., 1., 100), rng.uniform(-4., 4., 100), np.full(100, -5.2)])
        clock.tick(w, Frame(empty.astype(np.float32), packets=900))
        self.assertEqual(0, w._session_id)
        self.assertTrue(self._wait_files(os.path.join(self._tmp, "exit", "CH1", "*.xyz")))


class TestPlatePrefetch(unittest.TestCase):
    """★ 车牌预取（2026-09-23，两轮审查 M4）。

    原来测量在 `_calculate_detection` 里同步 read()：相机 HTTP 预算
    （f2+f3 默认 5 秒）**全程持通道锁** —— 后台停机时每测一辆车，看板的
    snapshot / capture_baseline / force_new_session 就卡 5 秒。
    现在：开窗/人工接管那一刻异步起跑（对齐原版 PlateMonitor），与
    3 秒沉降窗口并行；测量时刻只收结果，持锁等待以 2 秒封顶。
    """

    def _worker(self):
        import copy
        import tempfile
        from app.service import ChannelWorker, EventBus
        settings, ch, _, _, _ = H.ctx()
        ch = copy.deepcopy(ch)
        ch.id, ch.mode = "CH1", "synthetic"
        self._tmp = tempfile.mkdtemp()
        ch.abs_output_dir = self._tmp
        w = ChannelWorker(ch, EventBus(), settings.plate,
                          save_override=False, plate_mode="null")
        self.addCleanup(w.stop)
        self.addCleanup(w.source.close)
        return w

    def test_collect_caps_the_wait_and_consumes_the_prefetched_result(self):
        """预取后的收牌必须：等待有上限、消费真实结果、用完复位。"""
        from app.plate import PlateResult
        w = self._worker()
        seen = {}

        class _SlowFake:
            cfg = w.plate.cfg

            def wait(self, timeout):
                seen["wait"] = timeout

            def result(self):
                return PlateResult(ok=True, plate="宁B08669")

        w._plate_prefetched = True
        w.plate = _SlowFake()
        res = w._plate_collect()
        self.assertEqual("宁B08669", res.plate, "收到的必须是预取线程的识别结果")
        expected = min(2.0, settings_like_f3(w.plate.cfg))
        self.assertAlmostEqual(seen["wait"], expected, places=6,
                               msg="持锁等待必须以 _PLATE_LOCK_BUDGET_S 与 f3 预算取小封顶")
        self.assertFalse(w._plate_prefetched, "消费后必须复位，下一辆重新起跑")

    def test_collect_without_prefetch_stays_synchronous(self):
        """直调/回放路径（没经过开窗）保持同步读 —— 行为与从前一致。"""
        from app.plate import PlateResult
        w = self._worker()
        calls = []

        def _read(timeout=0.0):
            calls.append(timeout)
            return PlateResult(ok=False)
        w.plate.read = _read
        w._plate_collect()
        self.assertEqual(1, len(calls), "未预取时必须走同步 read()")

    def test_discarding_the_observation_invalidates_the_prefetch(self):
        """车次边界必须作废没被消费的预取 —— 否则下一车拿到上一车的牌。"""
        w = self._worker()
        w._plate_prefetched = True
        w._discard_observation_window()
        self.assertFalse(w._plate_prefetched)

    def test_full_chain_consumes_the_async_plate(self):
        """走 _tick_observation 全链路：开窗起跑 -> 与沉降并行 -> 测量收牌。"""
        from app.plate import SimulatedPlateReader
        from app.pointcloud import Frame
        from tests._observations import ObservationClock
        w = self._worker()
        w.settle_sec = 0.5
        slow = SimulatedPlateReader(w.plate.cfg, plates=["宁B47519"], delay=1.2)
        w.plate = slow
        self.addCleanup(slow.stop)
        queue = w.bus.subscribe()
        rng = np.random.default_rng(77)
        frame = Frame(np.column_stack((rng.uniform(-1, 1, 18000),
                                       rng.uniform(-5, 5, 18000),
                                       rng.uniform(-3.7, -2, 18000))))
        clock = ObservationClock(self)
        clock.settle(w, frame)
        self.assertTrue(w._plate_prefetched, "开窗那一刻必须异步起跑识别")
        self.assertTrue(slow.running(), "识别应与沉降窗口并行，而不是测量时才开始")
        for _ in range(12):
            clock.tick(w, frame)
        results = [e for e in queue if e["kind"] == "result"]
        self.assertTrue(results, "窗口期满后必须完成一次测量")
        self.assertEqual("宁B47519", results[-1]["plate"],
                         "异步预取的牌必须被测量消费 —— 拿到未知车牌说明 collect 没接到")


def settings_like_f3(cfg) -> float:
    """测试辅助：从 plate 配置读 f3 预算（毫秒转秒）。"""
    return float(getattr(cfg, "f3_timeout_ms", 0) or 0) / 1000.0
