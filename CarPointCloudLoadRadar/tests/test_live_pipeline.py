#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tests/test_live_pipeline.py — 真·端到端：UDP 报文 → 判定结论

其它测试都是分层的（解码归解码、判定归判定），这一组把**整条现场链路**串起来跑：

    RadarSimulator 造合法报文
        → 真 UDP 发到 127.0.0.1:<随机端口>（源 IP 127.0.0.2 / 127.0.0.3）
        → LiveHub 一个 socket 收 + 按源 IP 分流
        → LiveSource 按**方位角整圈**切帧
        → PacketDecoder.decode_frame_batch 解码 + 裁 ROI
        → StabilityDetector 停稳门控 → 沉降窗口
        → detect_empty / evaluate_empty 出结论
        → EventBus 广播

也就是说：除了雷达本身和相机，现场跑的每一行代码都在这条路径上。

为什么要有它：2026-09 现场那次"点云刷新太快、状态每秒都在变"，
分层测试全绿也没能拦住 —— 因为问题出在**层与层之间**（切帧与转圈不同步）。
这组用例就是补这个洞：空车道必须稳定空闲，来车必须出结论，
而且**帧与帧的点数必须一致**（点数抖 = 切帧又没切在整圈上）。
"""
from __future__ import annotations

import os
import socket
import tempfile
import threading
import time
import unittest

import numpy as np

from tests import _helpers as H                # noqa: F401  （把项目根加进 sys.path）
from app.settings import load_settings, load_calib
from app.pointcloud import AngleCache
from app.radar_sim import RadarSimulator, box_ground, box_truck
from app.service import LidarService, ConsoleLogger


# 仿真场景的地面高度 = 配置里的 ground_z。
# 写死成 -5.0 的话，现场把 ground_z 标定成别的值（2026-09 实测 -5.23）之后，
# 仿真地面就会跑到"有车"判据线（ground_z + vehicle_min_height_m）之上 ——
# 空车道被判成有车，而这正是这组用例要防的那个事故。
GROUND_Z = load_calib(
    [c for c in load_settings("").channels][0].abs_config_path
).detection.ground_z


def _free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _sender(ip: str) -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((ip, 0))                 # 绑源地址 → 收端 recvfrom 才会看到这个源 IP
    return s


def _wait(pred, timeout=8.0, step=0.02) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        if pred():
            return True
        time.sleep(step)
    return pred()


class _Radar:
    """一台"假雷达"：按场景生成整圈报文，用自己的源 IP 发出去。"""

    # az_step=0.8：与现场一致 —— 一包 2 个方位角，间隔 0.4°（config 的 angle_bins=900），
    # 一圈 450 包。步长写大了一圈的点数就不够，判定链第一关就会挡掉。
    # range_mode / z_down 必须跟着**通道配置**走，不能写死。
    # 假雷达负责"编码"、被测的服务负责"解码"，两边用的换算不一样就等于
    # 现场把 range_mode 配错：距离整体差 distance_scale 倍（4×）→ 点全落在
    # ROI 外 → 帧是空的 → 一条结论都出不来，而且哪个断言都不会指向真正的原因。
    # （2026-09-14 现场把 raw_offset 改成 scale_offset 后，这里正是这么挂的。）
    def __init__(self, ip: str, port: int, calib, angle, lidar, az_step=0.8):
        self.ip = ip
        self.port = port
        self.sock = _sender(ip)
        self.sim = RadarSimulator(calib.packet, calib.processing, angle,
                                  range_mode=lidar.range_mode,
                                  z_down=lidar.z_down,
                                  az_step_deg=az_step, range_noise_m=0.002,
                                  dropout_prob=0.0, seed=3)
        self.boxes = [box_ground(z_floor=GROUND_Z)]
        self._packets = None
        self._stop = threading.Event()
        self._thread: threading.Thread = None       # type: ignore[assignment]
        self.sent = 0

    def set_scene(self, boxes) -> None:
        self.boxes = list(boxes)
        self._packets = None                        # 场景变了，重新生成报文

    def _revolution(self):
        if self._packets is None:
            self._packets = self.sim.scan(self.boxes)
        return self._packets

    def spin_once(self) -> int:
        """发一整圈。"""
        pkts = self._revolution()
        for b in pkts:
            self.sock.sendto(b, ("127.0.0.1", self.port))
            self.sent += 1
        return len(pkts)

    def start(self, period: float = 0.1) -> None:
        def run():
            while not self._stop.is_set():
                self.spin_once()
                self._stop.wait(period)
        self._thread = threading.Thread(target=run, daemon=True,
                                        name=f"fake-radar-{self.ip}")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self.sock.close()


class TestLivePipeline(unittest.TestCase):
    """两路真 UDP 雷达跑完整条链路。"""

    @classmethod
    def setUpClass(cls):
        cls.port = _free_udp_port()
        cls.tmp = tempfile.mkdtemp(prefix="lidar-live-")
        st = load_settings("")
        ips = ["127.0.0.2", "127.0.0.3"]
        chans = []
        for i, ch in enumerate(st.channels[:2]):
            ch.enabled = True
            ch.mode = "live"
            ch.lidar.source_ip = ips[i]
            ch.lidar.ip = ips[i]
            ch.lidar.udp_port = cls.port
            ch.lidar.bind_ip = ""
            ch.lidar.frame_mode = "rotation"
            ch.abs_output_dir = os.path.join(cls.tmp, ch.id)
            ch.save = False
            chans.append(ch)
        st.channels = chans
        st.service.host = "127.0.0.1"
        st.service.state_push_hz = 0.0          # 测试里不节流，事件条数好数

        cls.calib = load_calib(chans[0].abs_config_path)
        cls.angle = AngleCache(chans[0].abs_angle_cache).load()

        cls.svc = LidarService(settings=st, save_override=False, plate_mode="null")
        # hub 必须绑到回环，不然收不到 127.0.0.x 发来的包
        cls.svc.hub.bind_ip = "127.0.0.1"
        for w in cls.svc.workers:
            w.settle_sec = 0.3                  # 沉降窗口压短，测试才跑得快
        cls.q = cls.svc.bus.subscribe(tag="test")
        cls.radars = [_Radar(ip, cls.port, cls.calib, cls.angle,
                             chans[0].lidar) for ip in ips]
        cls.svc.start()
        for r in cls.radars:
            r.start(period=0.1)

    @classmethod
    def tearDownClass(cls):
        for r in cls.radars:
            r.stop()
        cls.svc.stop()
        import shutil
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def _events(self, kind=None):
        evs = list(self.q)
        return [e for e in evs if kind is None or e.get("kind") == kind]

    def _snap(self, cid):
        for c in self.svc.snapshot()["channels"]:
            if c["id"] == cid:
                return c
        raise AssertionError(f"快照里没有通道 {cid}")

    # ── ① 空车道：出帧、分流正确、状态稳定在"空闲" ──────────────
    def test_01_frames_flow_and_lane_is_idle(self):
        ok = _wait(lambda: all(self._snap(c.ch.id)["frames"] >= 3
                               for c in self.svc.workers), timeout=10.0)
        self.assertTrue(ok, f"两路都应当出帧，实际 "
                            f"{[self._snap(w.ch.id)['frames'] for w in self.svc.workers]}")
        for w in self.svc.workers:
            s = self._snap(w.ch.id)
            src = s["source"]
            self.assertEqual(src["frame_mode"], "rotation")
            self.assertGreaterEqual(src["frames_by_rotation"], 1,
                                    "必须是按整圈切出来的帧")
            self.assertEqual(src["frames_by_timeout"], 0,
                             f"不该走兜底切帧：{src.get('error', '')}")
            self.assertEqual(s["stability"].get("state"), 0,
                             "只有地面回波时必须是「空闲」——否则会话永不结束、真车会被漏检")

    # ── ② 每帧包数/点数必须稳定（切帧切在整圈上的直接证据）────────
    def test_02_frame_size_is_stable(self):
        w = self.svc.workers[0]
        ns, revs = [], []
        for _ in range(6):
            f = w.last_frame
            info = w.source.info()
            if f is not None:
                ns.append(f.n)
                revs.append(info["rev_packets"])
            time.sleep(0.12)
        self.assertGreaterEqual(len(ns), 4, "应当持续有帧")
        self.assertEqual(len(set(revs)), 1,
                         f"每帧的报文数必须一致（一圈），实得 {revs}")
        spread = (max(ns) - min(ns)) / max(1, max(ns))
        self.assertLess(spread, 0.02,
                        f"静止场景帧间点数波动 {spread:.1%} 太大，切帧没对齐整圈：{ns}")

    # ── ③ 来车 → 停稳 → 出结论 ─────────────────────────────────
    def test_03_vehicle_triggers_result(self):
        self.q.clear()
        for r in self.radars:
            r.set_scene([box_ground(z_floor=GROUND_Z),
                         box_truck(dx=2.4, dy=12.0, dz=2.6, z_floor=GROUND_Z)])
        ok = _wait(lambda: len(self._events("result")) >= 2, timeout=25.0)
        res = self._events("result")
        self.assertTrue(ok, f"两路都该出结论，实得 {len(res)} 条；"
                            f"最近状态 {[self._snap(w.ch.id)['stability'] for w in self.svc.workers]}")
        by_ch = {e["channel"]: e for e in res}
        self.assertEqual(set(by_ch), {w.ch.id for w in self.svc.workers},
                         "两条通道各自都要出一条（分流没串台）")
        for cid, e in by_ch.items():
            self.assertGreater(e["n_points"], 20000,
                               f"{cid} 一帧只有 {e['n_points']} 点 —— 切帧没切满一圈")
            self.assertTrue(e["reason"], "必须给出判定理由")
            self.assertEqual(e["plate"], "未知车牌")     # plate_mode=null
            self.assertFalse(e["saved"], "本测试不落盘")

    # ── ④ 车开走 → 回到空闲、会话结束 ───────────────────────────
    def test_04_vehicle_leaves_closes_session(self):
        self.q.clear()
        for r in self.radars:
            r.set_scene([box_ground(z_floor=GROUND_Z)])
        ok = _wait(lambda: all(self._snap(w.ch.id)["stability"].get("state") == 0
                               for w in self.svc.workers), timeout=15.0)
        self.assertTrue(ok, "车走了必须回到「空闲」，否则下一辆车会被静默跳过")
        self.assertTrue(_wait(lambda: len(self._events("session_end")) >= 1, timeout=5.0),
                        "应当广播 session_end")

    # ── ⑤ 控制台事件日志：现场唯一的历史记录 ────────────────────
    def test_05_console_logger_writes_lines(self):
        import io as _io
        buf = _io.StringIO()
        log = ConsoleLogger(self.svc.bus, stream=buf)
        log.start()
        try:
            self.svc.bus.publish("result", {
                "channel": "CH9", "plate": "宁A88635W", "empty": True,
                "reason": "空车", "n_points": 55000, "deviation": 0.0012,
                "elapsed_ms": 700, "saved": ""})
            self.svc.bus.publish("state", {"channel": "CH9"})     # 不该被打
            self.assertTrue(_wait(lambda: log.printed >= 1, timeout=3.0))
            time.sleep(0.3)
            out = buf.getvalue()
            self.assertIn("[result]", out)
            self.assertIn("宁A88635W", out)
            self.assertNotIn("[state]", out, "state 每秒好几条，不该进日志")
        finally:
            log.stop()

    # ── ⑥ 没有 stdout 也不能崩（pythonw / 无控制台 exe）──────────
    def test_05b_console_logger_survives_missing_stdout(self):
        """★ 打包成无控制台窗口的 exe 时 sys.stdout 可能是 None。

        日志线程必须"写不出去就丢掉"，绝不能带着 AttributeError 死掉 ——
        死了之后现场就再也没有历史记录，而且没有任何提示。
        """
        import sys as _sys
        log = ConsoleLogger(self.svc.bus)          # 不指定流 → 取 sys.stdout
        old_out = _sys.stdout
        _sys.stdout = None                          # 模拟无控制台
        log.start()
        try:
            for _ in range(3):
                self.svc.bus.publish("warn", {"channel": "CH9", "message": "x"})
            self.assertTrue(_wait(lambda: log.dropped >= 1, timeout=3.0),
                            "应当计入丢弃，而不是抛异常")
            self.assertTrue(log.is_alive(), "日志线程不能死")
            self.assertEqual(log.printed, 0)
            # 恢复 stdout 后必须能继续写
            _sys.stdout = old_out
            import io as _io
            log.stream = _io.StringIO()
            self.svc.bus.publish("warn", {"channel": "CH9", "message": "y"})
            self.assertTrue(_wait(lambda: log.printed >= 1, timeout=3.0),
                            "流恢复后必须继续写")
        finally:
            _sys.stdout = old_out
            log.stop()

    # ── ⑦ 内部订阅者不算"看板连接数" ────────────────────────────
    def test_07_console_subscriber_not_counted_as_dashboard(self):
        before = self.svc.bus.subscribers
        log = ConsoleLogger(self.svc.bus)
        try:
            self.assertEqual(self.svc.bus.subscribers, before,
                             "控制台日志被算成看板了 —— 会挤占 MAX_SSE 名额")
            self.assertGreater(self.svc.bus.all_subscribers, before)
        finally:
            log.stop()


if __name__ == "__main__":
    unittest.main()
