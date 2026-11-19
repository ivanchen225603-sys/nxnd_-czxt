# -*- coding: utf-8 -*-
"""
多雷达分流测试 —— 一台机器同时处理 4 台雷达的核心保证。

为什么需要 LiveHub（而不是每条通道各开一个 socket）？
    UDP 没有"多个 socket 绑同一地址、各自收一份"的语义。4 条通道各 bind
    0.0.0.0:2368 时，内核只会把每个包投递给其中一个 socket ——
    实测现象是"只有一条通道有数据，其余永远空"，而且不报错。

所以多雷达只有一条正路：**一个 socket 收全部 + 按 recvfrom 的源 IP 分流**。
这个模块用本机回环地址 127.0.0.2~5 模拟 4 台雷达来验证它，
并且**用点的半径**确认没有串台（每台雷达发不同的距离，串台了一定露馅）。
"""
from __future__ import annotations

import socket
import time
import unittest

import numpy as np

from tests import _helpers as H
from app.radar_source import LiveHub, ChannelQueue, LiveSource
from app.radar_packet import PacketDecoder, make_synthetic_packet


def _free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _sender(ip: str) -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((ip, 0))                     # 绑源地址 → recvfrom 才会看到这个源 IP
    return s


def _wait(pred, timeout=5.0, step=0.01) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        if pred():
            return True
        time.sleep(step)
    return pred()


# ═══════════════════════════════════════════════ 队列

class TestChannelQueue(unittest.TestCase):

    def test_push_drain_roundtrip(self):
        q = ChannelQueue(maxlen=8)
        for i in range(3):
            q.push((bytes([i]), float(i)))
        items = q.drain(0.0)
        self.assertEqual(len(items), 3)
        self.assertEqual([d[0] for d in items], [b"\x00", b"\x01", b"\x02"])
        self.assertEqual(q.qsize(), 0)

    def test_overflow_drops_oldest_keeps_newest(self):
        """积压时丢最旧的 —— 保留最新一段才有意义（旧数据会造成一帧重影）。"""
        q = ChannelQueue(maxlen=3)
        for i in range(5):
            q.push((str(i).encode(), 0.0))
        items = q.drain(0.0)
        self.assertEqual([d.decode() for d, _ in items], ["2", "3", "4"])
        self.assertEqual(q.dropped, 2)

    def test_drain_timeout_returns_empty(self):
        q = ChannelQueue()
        t0 = time.time()
        items = q.drain(0.02)
        self.assertEqual(items, [])
        self.assertGreaterEqual(time.time() - t0, 0.015)

    def test_drain_wakes_on_push(self):
        """消费者阻塞在 drain 上时，push 应当把它唤醒（而不是等满超时）。"""
        import threading
        q = ChannelQueue()
        out = {}

        def consumer():
            t0 = time.time()
            out["items"] = q.drain(2.0)
            out["dt"] = time.time() - t0

        th = threading.Thread(target=consumer)
        th.start()
        time.sleep(0.05)
        q.push((b"x", 0.0))
        th.join(timeout=3.0)
        self.assertEqual(len(out.get("items", [])), 1)
        self.assertLess(out["dt"], 0.5, "push 后应当立刻被唤醒")


# ═══════════════════════════════════════════════ 注册规则

class TestLiveHubRegister(unittest.TestCase):

    def test_duplicate_source_ip_rejected(self):
        hub = LiveHub(bind_ip="127.0.0.1", udp_port=_free_udp_port())
        hub.register("127.0.0.2", "CH1")
        with self.assertRaises(ValueError):
            hub.register("127.0.0.2", "CH2")

    def test_empty_source_ip_rejected(self):
        hub = LiveHub(bind_ip="127.0.0.1", udp_port=_free_udp_port())
        with self.assertRaises(ValueError):
            hub.register("", "CH1")
        with self.assertRaises(ValueError):
            hub.register("   ", "CH1")

    def test_ensure_is_idempotent(self):
        """ensure 允许 service 先登记、LiveSource 后认领，不会重复报错。"""
        hub = LiveHub(bind_ip="127.0.0.1", udp_port=_free_udp_port())
        a = hub.register("127.0.0.2", "CH1")
        b = hub.ensure("127.0.0.2", "CH1 live@127.0.0.2")
        self.assertIs(a, b)
        self.assertEqual(hub.source_ips(), ["127.0.0.2"])


# ═══════════════════════════════════════════════ 分流正确性

class TestLiveHubDemux(unittest.TestCase):

    def setUp(self):
        self.port = _free_udp_port()
        self.hub = LiveHub(bind_ip="127.0.0.1", udp_port=self.port, queue_max=256)
        self.ips = [f"127.0.0.{i}" for i in (2, 3, 4, 5)]
        self.socks = []

    def tearDown(self):
        for s in self.socks:
            s.close()
        self.hub.close()

    def _send(self, ip, payload: bytes):
        s = _sender(ip)
        self.socks.append(s)
        s.sendto(payload, ("127.0.0.1", self.port))

    def test_four_sources_are_demultiplexed(self):
        """4 个源的数据必须各回各家，且内容不能串台。"""
        for i, ip in enumerate(self.ips):
            self.hub.register(ip, f"CH{i + 1}")
        self.hub.start()

        n = 6
        for k in range(n):
            for ip in self.ips:
                self._send(ip, f"{ip}#{k}".encode())

        expect = n * len(self.ips)
        self.assertTrue(_wait(lambda: self.hub.recv_count >= expect),
                        f"只收到 {self.hub.recv_count}/{expect} 个包")
        self.assertEqual(self.hub.unknown, 0, "不该有未登记源的包")

        for i, ip in enumerate(self.ips):
            items = self.hub._queues[ip].drain(0.0)
            self.assertEqual(len(items), n, f"{ip} 收到 {len(items)} 个包（应为 {n}）")
            payloads = sorted(d.decode() for d, _ in items)
            self.assertEqual(
                payloads, sorted(f"{ip}#{k}" for k in range(n)),
                f"{ip} 的队列里混入了别的雷达的包 —— 分流串台了")

    def test_unknown_source_dropped_and_counted(self):
        """未登记的源在 strict 模式下被丢弃 + 计数（不串进任何通道）。"""
        self.hub.register(self.ips[0], "CH1")
        self.hub.start()

        for _ in range(5):
            self._send("127.0.0.9", b"intruder")
        for _ in range(3):
            self._send(self.ips[0], b"mine")

        self.assertTrue(_wait(lambda: self.hub.unknown >= 5 and self.hub.recv_count >= 3),
                        f"unknown={self.hub.unknown} recv={self.hub.recv_count}")
        self.assertIn("127.0.0.9", self.hub.unknown_ips)
        self.assertEqual(self.hub.recv_count, 3, "已登记源不该被误伤")
        self.assertEqual(len(self.hub._queues[self.ips[0]].drain(0.0)), 3)

    def test_non_strict_routes_unknown_to_default(self):
        hub = LiveHub(bind_ip="127.0.0.1", udp_port=_free_udp_port(), strict=False)
        q = hub.register(self.ips[0], "CH1", default=True)
        hub.start()
        try:
            s = _sender("127.0.0.9")
            self.socks.append(s)
            for _ in range(3):
                s.sendto(b"default", ("127.0.0.1", hub.udp_port))
            self.assertTrue(_wait(lambda: q.qsize() >= 3), f"qsize={q.qsize()}")
            self.assertEqual(len(q.drain(0.0)), 3)
        finally:
            hub.close()

    def test_stats_snapshot_shape(self):
        for i, ip in enumerate(self.ips):
            self.hub.register(ip, f"CH{i + 1}")
        self.hub.start()
        self._send(self.ips[0], b"a")
        self.assertTrue(_wait(lambda: self.hub.recv_count >= 1))
        d = self.hub.to_dict()
        self.assertIn("pkt_rate", d)
        self.assertIn("sources", d)
        # 4 台雷达都要出现在统计里（没上线的显示 0，便于现场发现接线问题）
        self.assertEqual(set(d["sources"]), set(self.ips))
        self.assertEqual(d["sources"][self.ips[0]]["channel"], "CH1")


class TestRateIsWindowed(unittest.TestCase):
    """包率 / 转速必须按**滑动窗口**算，不能用「总数 / 运行时长」。

    累计平均在现场排障时是有害的，而且恰恰在最需要它的时候骗人：
    一条通道掉线一阵子再恢复，那段空窗会永久压低平均值 ——
    已经满速的链路会持续显示成只有两三成包率，看起来就像在丢包。

    2026-09-14 现场真的据此误判过：改网段导致 CH2/3/4 断了十来分钟，
    恢复后看板报 2 096 包/秒、转速 139 rpm，裸接收实测其实是 8 980 / 600。
    """

    def test_recovers_immediately_after_an_outage(self):
        from app.radar_source import RateWindow
        w = RateWindow(window=5.0, min_span=0.5)
        t0 = 1000.0
        w.rate(0, t0, t0)                       # 开机
        # 掉线 100 秒：一个包都没有
        w.rate(0, t0, t0 + 100.0)
        # 恢复，满速 9000 包/秒
        r = 0.0
        for i in range(1, 12):
            r = w.rate(i * 900, t0, t0 + 100.0 + i * 0.1)
        self.assertAlmostEqual(r, 9000.0, delta=200.0,
                               msg=f"恢复后应当立刻报满速，实得 {r:.0f}")
        cumulative = 11 * 900 / (100.0 + 1.1)
        self.assertLess(cumulative, 200.0,
                        "这条用例的前提没成立：累计口径本应被空窗压得很低")

    def test_falls_back_to_average_before_the_window_fills(self):
        """样本还不够一个 min_span 时退回累计口径 —— 刚启动只有这个可用。"""
        from app.radar_source import RateWindow
        w = RateWindow(window=5.0, min_span=1.0)
        t0 = 500.0
        r = w.rate(1800, t0, t0 + 0.2)          # 才过 0.2 秒
        self.assertAlmostEqual(r, 1800 / 0.2, delta=1.0)

    def test_drops_when_the_stream_stops(self):
        """反过来也要灵：真断了就得掉下去，不能被历史撑着。"""
        from app.radar_source import RateWindow
        w = RateWindow(window=3.0, min_span=0.5)
        t0 = 0.0
        for i in range(0, 20):
            w.rate(i * 900, t0, t0 + i * 0.1)   # 满速跑 2 秒
        r = 0.0
        for i in range(1, 40):                  # 之后彻底不来包
            r = w.rate(19 * 900, t0, t0 + 2.0 + i * 0.2)
        self.assertLess(r, 100.0, f"流断了速率却还报 {r:.0f}")


# ═══════════════════════════════════════════════ 接收线程存活

class TestReceiveThreadLiveness(unittest.TestCase):
    """★ 接收线程"进程内部挂了"必须与"网络没数据"分开显示。

    `_recv_loop` 只兜 `socket.timeout` 与 `OSError`，其余任何异常
    （MemoryError、队列对象被换掉后的 AttributeError…）都会让线程**静默退出**。
    而这是全链路**唯一**的接收点 —— 它一死，四条通道同时不再收到任何包，
    看板上却只显示"等待雷达数据…"，与拔网线、与雷达断电完全无法区分。

    2026-09-22 审查（docs/52 第二节）：`app/` 里 plate / demo_director /
    result_finalizer / load_radar_bridge / service 的线程都有 `is_alive()` 检查，
    唯独 LiveHub 没有，对外统计里也没有任何线程状态字段。
    """

    class _BoomSock:
        """替换掉真 socket：recvfrom 抛一个 _recv_loop **不兜**的异常。"""

        def __init__(self, exc):
            self.exc = exc

        def recvfrom(self, _n):
            raise self.exc

        def close(self):
            pass

    def _crashed_hub(self, exc):
        hub = LiveHub(udp_port=_free_udp_port())
        hub.register("127.0.0.1", "CH1")
        hub.open()
        hub.sock.close()
        hub.sock = self._BoomSock(exc)
        hub.start()
        for _ in range(100):                    # 最多等 1 秒
            if not hub.alive():
                break
            time.sleep(0.01)
        return hub

    def test_a_crash_is_visible_instead_of_silent(self):
        hub = self._crashed_hub(MemoryError("模拟内存耗尽"))
        try:
            self.assertFalse(hub.alive(), "线程已经死了，alive() 却还报 True")
            self.assertTrue(hub.dead)
            self.assertIn("MemoryError", hub.last_error)
            self.assertIn("接收线程", hub.last_error,
                          "报错必须说清是**线程**挂了，不是网络没数据")
            self.assertIs(hub.to_dict()["alive"], False,
                          "快照里要能读到，否则看板/api 还是看不见")
        finally:
            hub.close()

    def test_any_unexpected_exception_is_caught(self):
        for exc in (AttributeError("队列没了"), RuntimeError("别的什么"),
                    ValueError("解包失败")):
            with self.subTest(type(exc).__name__):
                hub = self._crashed_hub(exc)
                try:
                    self.assertFalse(hub.alive())
                    self.assertIn(type(exc).__name__, hub.last_error)
                finally:
                    hub.close()

    def test_a_healthy_hub_reports_alive(self):
        """反过来也要对：正常跑着不能误报成挂了。"""
        hub = LiveHub(udp_port=_free_udp_port())
        hub.register("127.0.0.1", "CH1")
        hub.start()
        try:
            time.sleep(0.15)
            self.assertTrue(hub.alive(), "正常运行却报 alive=False")
            self.assertFalse(hub.dead)
            self.assertIs(hub.to_dict()["alive"], True)
        finally:
            hub.close()
        self.assertFalse(hub.alive(), "close() 之后不该再报 alive")

    def test_never_started_is_not_alive(self):
        hub = LiveHub(udp_port=_free_udp_port())
        self.assertFalse(hub.alive())
        self.assertFalse(hub.dead, "没起过不等于挂了，dead 只表示异常退出")

    def test_ordinary_oserror_does_not_kill_the_thread(self):
        """★ OSError 是**可恢复**的（recvfrom 偶发失败），不能被新守卫误杀。

        没有这条，很容易把外层 except 写得太宽、把原来 continue 的那条
        路径也一起收掉 —— 那会让一次偶发的 recvfrom 失败变成整机停摆。
        """
        hub = LiveHub(udp_port=_free_udp_port())
        hub.register("127.0.0.1", "CH1")
        hub.open()
        real = hub.sock
        calls = []

        class Flaky:
            def recvfrom(self, n):
                calls.append(1)
                if len(calls) < 3:
                    raise OSError("偶发失败")
                raise socket.timeout()          # 之后就当没数据，正常空转

            def close(self):
                real.close()

        hub.sock = Flaky()
        hub.start()
        try:
            time.sleep(0.5)
            self.assertTrue(hub.alive(), "OSError 之后线程不该退出")
            self.assertFalse(hub.dead)
            self.assertGreaterEqual(hub.errors, 2, "OSError 要计数")
            self.assertGreater(len(calls), 3, "线程应当继续在收")
        finally:
            hub.close()


# ═══════════════════════════════════════════════ 端到端：4 路各自成帧

class TestMultiRadarEndToEnd(unittest.TestCase):
    """
    4 个 LiveSource 共用一个 LiveHub，各自组帧 —— 复现现场拓扑。

    关键断言用**点的半径**：4 台雷达故意用不同的 raw_distance，
    于是各自点云的半径必然不同。若分流串台，半径就会对不上 ——
    这比"队列里包的数量对不对"强得多。
    """

    def test_each_channel_builds_frames_from_its_own_radar_only(self):
        _, ch, calib, _, ac = H.ctx()
        port = _free_udp_port()
        hub = LiveHub(bind_ip="127.0.0.1", udp_port=port, strict=True)
        ips = [f"127.0.0.{i}" for i in (2, 3, 4, 5)]
        raws = [5000, 6000, 7000, 8000]            # 每台雷达一个特征距离（mm）

        srcs, decs = [], []
        for i, ip in enumerate(ips):
            dec = PacketDecoder(calib.packet, calib.processing, ac,
                                range_mode=ch.lidar.range_mode,
                                z_down=ch.lidar.z_down,
                                max_range_m=ch.lidar.max_range_m)
            decs.append(dec)
            srcs.append(LiveSource(dec, frame_ms=200, roi=None,
                                   name=f"CH{i + 1}", hub=hub, source_ip=ip))
        # 4 条通道必须共用同一个 socket —— 自己不该有 socket
        self.assertTrue(all(s.sock is None for s in srcs),
                        "hub 模式下 LiveSource 不应自己开 socket")
        self.assertEqual(len({id(s.queue) for s in srcs}), 4, "每路应有独立队列")

        hub.start()
        senders = [_sender(ip) for ip in ips]
        try:
            frames, sent = {}, 0
            deadline = time.time() + 10.0
            while len(frames) < 4 and time.time() < deadline:
                for j, s in enumerate(senders):
                    s.sendto(make_synthetic_packet(
                        calib.packet, azimuth_deg=(sent * 9.0) % 360.0,
                        raw_distance=raws[j], seed=sent * 7 + j),
                        ("127.0.0.1", port))
                sent += 1
                for i, src in enumerate(srcs):
                    if i in frames:
                        continue
                    f = src.read()
                    if f is not None and f.n > 0:
                        frames[i] = f
                time.sleep(0.02)

            self.assertEqual(len(frames), 4, f"只有 {len(frames)}/4 路出帧")

            for i, f in frames.items():
                radius = np.linalg.norm(f.xyz.astype(np.float64), axis=1)
                # make_synthetic_packet 的第二块比第一块远 200 个原始单位，
                # 所以一包的平均落在两块之间 —— 拿 raws[i] 单点算期望会系统性偏低。
                # 偏多少取决于 range_mode：raw_offset 下 1 单位 = 1mm（偏 0.1m，
                # 恰好压在旧的 delta=0.15 边上），scale_offset 下 1 单位 = 4mm
                # （偏 0.4m，直接失败）。期望值必须跟着换算走，不能写死。
                expect = float(np.mean(
                    decs[i].to_range_m(np.array([raws[i], raws[i] + 200]))))
                # 每台雷达的距离都不同 → 半径必须落在自己那一档上
                self.assertAlmostEqual(
                    float(radius.mean()), expect, delta=0.15,
                    msg=f"CH{i + 1} 的半径 {radius.mean():.3f} 与预期 {expect:.3f} 不符"
                        f"（疑似接到了别的雷达的数据）")
                # 且与其它雷达的特征距离明显不同（进一步排除串台）
                for j, other in enumerate(raws):
                    if j == i:
                        continue
                    other_r = float(np.mean(
                        decs[j].to_range_m(np.array([other, other + 200]))))
                    self.assertGreater(abs(radius.mean() - other_r), 0.3)
                self.assertEqual(f.n % 256, 0, "每包应贡献 256 点，点数不是整数包")
        finally:
            for s in senders:
                s.close()
            for src in srcs:
                src.close()
            hub.close()


class TestQueueOverflowIsVisible(unittest.TestCase):
    """★ 队列持续溢出必须从 info() 看得见（2026-09-23，两轮审查 M7）。

    溢出的含义是「消费者（通道线程）停了」—— 解码层的丢包会写 last_error，
    队列溢出原来一个字不出，而它恰恰是最该被看见的卡死信号。
    红字只挂"最近 30 秒仍在发生"的（与兜底切帧同一条纪律）：瞬时积压
    （六面拟合几百 ms）不报警，持续顶着上限才报；队列排空后红字要撤。
    """

    def _source(self):
        from app.settings import load_settings
        from app.pointcloud import AngleCache
        settings, ch, calib, _, _ = H.ctx()
        angle = AngleCache(ch.abs_angle_cache).load()
        dec = PacketDecoder(calib.packet, calib.processing, angle,
                            range_mode=ch.lidar.range_mode, z_down=ch.lidar.z_down,
                            max_range_m=ch.lidar.max_range_m)
        hub = LiveHub(bind_ip="127.0.0.1", udp_port=_free_udp_port(), queue_max=64)
        src = LiveSource(dec, frame_ms=100, roi=None, hub=hub,
                         source_ip="192.168.1.10")
        return src, hub

    def test_persistent_overflow_raises_a_red_error_and_clears_when_drained(self):
        src, hub = self._source()
        try:
            q = src.queue
            self.assertIsNotNone(q)
            for _ in range(64 + 32):          # 超容量：丢 32 个最旧的、仍顶着满
                q.push((b"x", 0.0))
            self.assertGreater(q.dropped, 0)
            info = src.info()
            self.assertIn("queue", info, "队列实况必须进 info（看板/快照的出口）")
            self.assertEqual(info["queue"]["dropped"], q.dropped)
            self.assertIn("仍在溢出", info.get("error", ""),
                          "worker 卡住时红字必须看得见 —— 修复前这里恒为空")
            # 瞬时溢出不该一直挂红字：消费者恢复、队列排空后要撤
            q.drain(0.0)
            info2 = src.info()
            self.assertNotIn("仍在溢出", info2.get("error", ""),
                             "队列已排空，红字不撤现场会学会无视它")
        finally:
            hub.close()


if __name__ == "__main__":
    unittest.main()
