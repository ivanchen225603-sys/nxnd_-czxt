#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tests/test_dashboard_contract.py — 后端推给看板的字段是一份**契约**

看板（app/web/static/index.html）是纯手写 JS，直接按字段名取值：
    e.link.packet_rate、c.roi、stb.n_vehicle_points、r.dim_message …
后端改个名字、少发一个字段，前端**不会报错**，只会安静地显示 "–" 或不再刷新
—— 而这正是现场最难发现的一类故障（"看板看着挺正常，就是数字不动"）。

所以这里把契约写死：
    ① state / result 事件、/api/state 快照必须包含这些键；
    ② 这些键必须真的在 index.html 里被用到（防止后端发了一堆没人看的字段，
       更防止改名时只改了一边）。

要加字段：先加到后端，再改 index.html，最后把名字登记到这里。
要删字段：先确认 index.html 不再用它。
"""
from __future__ import annotations

import json
import unittest

import numpy as np

from tests import _helpers as H                # noqa: F401  （把项目根加进 sys.path）
from app.settings import load_settings, root
from app.pointcloud import Frame
from app.service import EventBus, ChannelWorker, LidarService

# ── 契约：看板从每类载荷里取的字段 ──────────────────────────────
STATE_KEYS = {
    "channel", "channel_name", "session", "n_points", "stability", "plate",
    "reason", "fps", "frames", "source", "save_enabled", "link", "error", "cloud",
    "health",          # 常驻健康告警（有车判据失效时挂出来）
}
STABILITY_KEYS = {
    "state", "state_cn", "deviation", "stable", "n_points", "elapsed",
    "n_vehicle_points",
}
RESULT_KEYS = {
    "channel", "channel_name", "session", "plate", "plate_ok", "empty", "reason",
    "n_points", "deviation", "elapsed_ms", "dim_ok", "dim_message", "saved",
    "save_enabled", "dims", "diagnostics", "volume", "planes", "cloud",
}
DIAG_KEYS = {
    "n_points", "inliers", "ratio", "span_x", "span_y", "normal_z",
    "floor_z", "min_z", "z_above", "yaw_deg",
}
CHANNEL_SNAPSHOT_KEYS = {
    "id", "name", "mode", "online", "source_kind", "fps", "frames", "stability",
    "last_reason", "last_plate", "last_diag", "last_result", "saved_count",
    "session", "error", "source", "plate", "angle_ok", "baseline", "history",
    "roi", "state_push_hz", "save_enabled", "health",
}
SERVICE_SNAPSHOT_KEYS = {
    "uptime", "channels", "online", "saved_total", "subscribers",
    "sim_channels", "live_channels",
    # 看板算"这条结论是多久以前的"要用服务端时钟 + 原厂的 result_hold_sec
    "now", "result_hold_sec",
}
# live 通道的链路实况（看板顶部徽标"源IP · 包率 · 转速 · 整圈切帧 N 包/圈"）
LINK_KEYS_LIVE = {
    "source_ip", "packet_rate", "rpm", "frame_mode", "frame_ms",
    "rev_packets", "frames_by_rotation", "frames_by_timeout",
    "pending_packets", "error",
}
# ROI 保留率要等出过帧才有（info() 里按需加），单独列
LINK_KEYS_AFTER_FRAME = {"roi_keep", "points_raw"}


def _ground(n=9000, seed=0):
    r = np.random.default_rng(seed)
    return Frame(np.stack([r.uniform(-2, 2, n), r.uniform(-11, 8, n),
                           np.full(n, -5.0)], 1).astype(np.float32))


def _truck(n=40000, seed=5):
    r = np.random.default_rng(seed)
    car = np.stack([r.uniform(-1.2, 1.2, n), r.uniform(-6, 6, n),
                    r.uniform(-4.2, -2.5, n)], 1).astype(np.float32)
    return Frame(np.concatenate([_ground().xyz, car]))


class TestPayloadContract(unittest.TestCase):

    def setUp(self):
        from tests._observations import ObservationClock
        self.clock = ObservationClock(self)

    def _worker(self):
        st = load_settings("")
        ch = [c for c in st.channels if c.id == "CH1"][0]
        ch.mode = "synthetic"
        bus = EventBus()
        w = ChannelWorker(ch, bus, st.plate, save_override=False,
                          plate_mode="null", state_push_hz=0.0)
        w.settle_sec = 0.0
        return w, bus.subscribe(tag="test")

    def _tick(self, w, frame):
        self.clock.tick(w, frame)

    def test_state_event_contract(self):
        w, q = self._worker()
        self._tick(w, _ground())
        ev = [e for e in q if e["kind"] == "state"][-1]
        missing = STATE_KEYS - set(ev)
        self.assertFalse(missing, f"state 事件少了看板要用的字段：{sorted(missing)}")
        missing = STABILITY_KEYS - set(ev["stability"])
        self.assertFalse(missing, f"stability 少了字段：{sorted(missing)}")

    def test_result_event_contract(self):
        w, q = self._worker()
        for _ in range(30):
            self._tick(w, _truck())      # 连续真实新帧停稳后才触发
        res = [e for e in q if e["kind"] == "result"]
        self.assertTrue(res, "没有产出 result 事件")
        missing = RESULT_KEYS - set(res[-1])
        self.assertFalse(missing, f"result 事件少了字段：{sorted(missing)}")
        missing = DIAG_KEYS - set(res[-1]["diagnostics"])
        self.assertFalse(missing, f"diagnostics 少了字段：{sorted(missing)}")

    def test_light_snapshot_only_omits_history(self):
        """WebApi 用 include_history=False：只少最近结果，其余字段与全量快照一致。"""
        w, _ = self._worker()
        for _ in range(30):
            self._tick(w, _truck())
        self.assertTrue(w.history, "前提：要有最近结果")
        full, light = w.snapshot(), w.snapshot(include_history=False)
        self.assertIn("history", full)
        self.assertNotIn("history", light)
        self.assertEqual(set(full) - {"history"}, set(light))
        self.assertEqual(full["cloud"], light["cloud"])

    def test_snapshot_shares_no_state_with_the_worker(self):
        """去掉整份二次深拷贝后，返回值仍不能与工作线程的活动状态共享引用。"""
        w, _ = self._worker()
        for _ in range(30):
            self._tick(w, _truck())
        before = w.snapshot()
        snap = w.snapshot()
        snap["cloud"].append([9.0, 9.0, 9.0])
        snap["history"][-1]["reason"] = "被调用方改写"
        snap["stability"]["probe"] = 1
        snap["finalizer"]["probe"] = 1
        snap["source"]["probe"] = 1
        after = w.snapshot()
        for value in (before, after):
            value.pop("frame_age_sec", None)        # 按时钟实时计算，与引用共享无关
        self.assertEqual(before, after)

    def test_snapshot_contract(self):
        st = load_settings("")
        for c in st.channels:
            c.mode = "synthetic"
        svc = LidarService(settings=st, save_override=False, plate_mode="null")
        snap = svc.snapshot()
        missing = SERVICE_SNAPSHOT_KEYS - set(snap["service"])
        self.assertFalse(missing, f"snapshot.service 少了字段：{sorted(missing)}")
        for c in snap["channels"]:
            missing = CHANNEL_SNAPSHOT_KEYS - set(c)
            self.assertFalse(missing, f"{c.get('id')} 的快照少了字段：{sorted(missing)}")
        self.assertIn("gate", snap)
        self.assertIn("hub", snap)
        diagnostic = svc.snapshot(include_history=False, include_frame_integrity=True)
        for channel in diagnostic['channels']:
            self.assertIn('frame_integrity', channel)
            self.assertNotIn('history', channel)

    def test_snapshot_is_json_serialisable(self):
        """看板走 JSON —— 快照里混进 numpy 标量会让 /api/state 直接 500。"""
        st = load_settings("")
        for c in st.channels:
            c.mode = "synthetic"
        svc = LidarService(settings=st, save_override=False, plate_mode="null")
        from app.web.server import _json_default
        s = json.dumps(svc.snapshot(), ensure_ascii=False, default=_json_default)
        self.assertGreater(len(s), 100)

    def test_live_link_contract(self):
        """live 源的 link 载荷要带齐徽标要显示的东西。"""
        from app.radar_packet import PacketDecoder
        from app.radar_source import LiveSource, LiveHub
        from app.settings import load_calib
        from app.pointcloud import AngleCache
        calib = load_calib(root("config", "config.json"))
        angle = AngleCache(root("config", "angle_cache.csv")).load()
        dec = PacketDecoder(calib.packet, calib.processing, angle,
                            range_mode="raw_offset", z_down=True, max_range_m=60.0)
        hub = LiveHub(bind_ip="127.0.0.1", udp_port=0)
        src = LiveSource(dec, frame_ms=100, roi=calib.roi, hub=hub,
                         source_ip="10.0.0.9", name="t")
        info = src.info()
        missing = LINK_KEYS_LIVE - set(info)
        self.assertFalse(missing, f"live 源 info() 少了字段：{sorted(missing)}")


class TestFrontendUsesThem(unittest.TestCase):
    """反向检查：契约里的关键字段必须真的出现在 index.html 里。

    只查"名字有没有在前端出现"，不做 JS 解析 —— 够挡住"后端改名、前端忘改"
    这类事故，又不会因为前端换了写法就误报。
    """

    @classmethod
    def setUpClass(cls):
        with open(root("app", "web", "static", "index.html"),
                  encoding="utf-8") as f:
            cls.html = f.read()

    def _check(self, names, where):
        missing = [n for n in sorted(names) if n not in self.html]
        self.assertFalse(missing,
                         f"{where} 里这些字段在 index.html 中找不到 —— "
                         f"要么前端没跟上改名，要么后端在发没人看的字段：{missing}")

    def test_health_warning_is_shown(self):
        """判据失效的常驻告警必须能显示出来 —— 它意味着判定结果不可信。"""
        self._check({"health", "判定结果不可信"}, "健康告警")

    def test_roi_keep_is_shown(self):
        """ROI 保留率是"量纲对不对"的现场信号，必须出现在看板上。"""
        self._check({"roi_keep"}, "link 载荷")

    def test_state_and_link_fields_used(self):
        self._check({"link", "error", "cloud", "stability", "fps"}, "state 事件")
        self._check(LINK_KEYS_LIVE - {"frames_by_rotation", "pending_packets",
                                      "frame_ms"}, "link 载荷")

    def test_stability_fields_used(self):
        self._check({"state_cn", "deviation", "n_vehicle_points", "settle_remain"},
                    "stability")

    def test_idle_lane_display_inputs_used(self):
        """空车道是**正常工况**，看板必须能把它和"有车/没数据"分开显示。

        依赖三样东西：stability.state（=0 即车道无车）、channels[].frames
        （一帧都没有 = 根本没收到报文）、result.session（结论属于哪一次会话）。
        少任何一个，卡片就会把"车走了"显示成上一辆车的结论。
        """
        self._check({"车道空闲", "等待雷达数据", "上一辆"}, "空车道文案")
        for name in ("stb.state === 0", "!c.frames", "r.session === c.session"):
            self.assertIn(name, self.html,
                          f"空车道三态判断少了 `{name}`")

    def test_diag_fields_used(self):
        self._check(DIAG_KEYS - {"z_above"}, "diagnostics")   # z_above 前端是算出来的

    def test_snapshot_fields_used(self):
        self._check({"roi", "saved_total", "subscribers", "sim_channels",
                     "live_channels", "unknown", "pkt_rate",
                     "result_hold_sec"}, "快照")


if __name__ == "__main__":
    unittest.main()
