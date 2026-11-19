# -*- coding: utf-8 -*-
"""
整帧批量解码的一致性测试。

`PacketDecoder.decode_frame_batch()` 是"一台机器同时处理 4 台雷达"的性能前提：
它把一帧内的所有报文堆成一个大数组一次算完，从而绕开"逐包 25 次小 numpy 调用
把 GIL 攥死"的问题。

但**性能优化绝不能改变结果**。这个模块就是钉死这一点：
批量路径的输出必须与逐包 `decode()+to_cartesian()` 拼接的结果**逐位相同** ——
包括坐标数值、ring、强度，以及**点的先后顺序**。

一旦有人以后"顺手优化"批量路径，这里会立刻变红。
"""
from __future__ import annotations

import unittest

import numpy as np

from tests import _helpers as H
from app.radar_packet import PacketDecoder, make_synthetic_packet


def _per_packet(dec, bufs):
    """逐包路径：复刻改造前 LiveSource._flush() 的拼帧方式。"""
    xs, rs, its = [], [], []
    for b in bufs:
        pkt = dec.decode(b)
        xyz, ring, it = dec.to_cartesian(pkt)
        if len(xyz):
            xs.append(xyz)
            rs.append(ring)
            its.append(it)
    if not xs:
        return (np.zeros((0, 3), np.float32), np.zeros(0, np.uint16),
                np.zeros(0, np.uint8))
    return (np.concatenate(xs, axis=0), np.concatenate(rs, axis=0),
            np.concatenate(its, axis=0))


class TestBatchDecode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        _, ch, calib, _, ac = H.ctx()
        cls.calib = calib
        cls.dec = PacketDecoder(calib.packet, calib.processing, ac,
                                range_mode=ch.lidar.range_mode,
                                z_down=ch.lidar.z_down,
                                max_range_m=ch.lidar.max_range_m)

    def _packets(self, n=12, span_deg=360.0):
        bufs = []
        for i in range(n):
            az = (i * span_deg / n) % 360.0
            bufs.append(make_synthetic_packet(self.calib.packet, azimuth_deg=az,
                                              raw_distance=5000 + 37 * i, seed=i))
        return bufs

    def test_bitwise_identical_to_per_packet(self):
        """批量解码必须与逐包拼接逐位一致（数值 + ring + 强度 + 顺序）。"""
        bufs = self._packets(n=16)
        bx, br, bi = _per_packet(self.dec, bufs)
        gx, gr, gi, info = self.dec.decode_frame_batch(bufs)

        self.assertEqual(info["packets"], len(bufs))
        self.assertEqual(info["dropped"], 0)
        self.assertEqual(len(gx), info["points"])
        self.assertGreater(len(gx), 0, "合成报文应当解出点")

        self.assertEqual(bx.shape, gx.shape)
        # 坐标必须完全相等（不是近似）——证明只是换了算法，没有换结果
        self.assertTrue(np.array_equal(bx, gx),
                        f"坐标不一致：最大偏差 "
                        f"{float(np.max(np.abs(bx - gx))) if bx.size else 0}")
        self.assertTrue(np.array_equal(br, gr), "ring 不一致")
        self.assertTrue(np.array_equal(bi, gi), "强度不一致")

    def test_single_packet_matches(self):
        """只有 1 个报文时也要一致（退化情形）。"""
        bufs = self._packets(n=1)
        bx, br, bi = _per_packet(self.dec, bufs)
        gx, gr, gi, _ = self.dec.decode_frame_batch(bufs)
        self.assertTrue(np.array_equal(bx, gx))
        self.assertTrue(np.array_equal(br, gr))
        self.assertTrue(np.array_equal(bi, gi))

    def test_empty_input(self):
        gx, gr, gi, info = self.dec.decode_frame_batch([])
        self.assertEqual(len(gx), 0)
        self.assertEqual(info["packets"], 0)
        self.assertEqual(info["points"], 0)

    def test_short_packets_are_dropped_and_counted(self):
        """长度不足的报文要被丢掉并计数，而不是让整帧失败。"""
        good = self._packets(n=4)
        bufs = good + [b"\x00\x01\x02", b""]
        gx, _gr, _gi, info = self.dec.decode_frame_batch(bufs)
        self.assertEqual(info["dropped"], 2, "两个短包应被计入 dropped")
        self.assertEqual(info["packets"], 4)
        # 结果仍应与"只喂好包"完全一致
        bx, _, _ = _per_packet(self.dec, good)
        self.assertTrue(np.array_equal(bx, gx))

    def test_all_bad_input_returns_empty(self):
        gx, _gr, _gi, info = self.dec.decode_frame_batch([b"x", b"yy"])
        self.assertEqual(len(gx), 0)
        self.assertEqual(info["packets"], 0)
        self.assertEqual(info["dropped"], 2)

    def test_frame_from_batch_shape(self):
        bufs = self._packets(n=8)
        fr, info = self.dec.frame_from_batch(bufs, seq=3)
        self.assertIsNotNone(fr)
        self.assertEqual(fr.seq, 3)
        self.assertEqual(fr.n, info["points"])
        self.assertEqual(fr.ring.dtype, np.uint16)
        self.assertEqual(len(fr.ring), fr.n)

    def test_rev_deg_estimates_one_turn(self):
        """一圈 360° 的报文，估计前进量应当接近 360°（误差 < 5%）。"""
        n = 36
        bufs = [make_synthetic_packet(self.calib.packet,
                                      azimuth_deg=(i * 10.0) % 360.0,
                                      raw_distance=5000, seed=i)
                for i in range(n)]
        _x, _r, _i, info = self.dec.decode_frame_batch(bufs)
        self.assertGreater(info["rev_deg"], 360.0 * 0.95)
        self.assertLess(info["rev_deg"], 360.0 * 1.05)

    def test_rev_deg_handles_wrap(self):
        """跨 360° 回绕时要补成 +360，不能算成负增长。"""
        bufs = [make_synthetic_packet(self.calib.packet, azimuth_deg=az,
                                      raw_distance=5000, seed=i)
                for i, az in enumerate([350.0, 355.0, 0.0, 5.0, 10.0])]
        _x, _r, _i, info = self.dec.decode_frame_batch(bufs)
        self.assertAlmostEqual(info["rev_deg"], 20.0, delta=1e-6)


class TestFramePacketCountIsValidOnly(unittest.TestCase):
    """★ 帧声明的包数必须是**有效**报文数，不是收到数（2026-09-19 修）。

    `LiveSource` 原先写的是 `packets=len(bufs)` —— 收到多少写多少，
    把**解不出点的报文也算进去**。于是解码大面积失败的帧照样声明满包，
    过 `ChannelWorker._frame_is_complete`，拿到 `trusted=True`，
    最终取得测量许可。实测：送进 901 包、451 包因长度不足被丢，
    帧仍声明 901。

    正确值 `info["packets"]` 一直就在解码结果里，只是没被用。
    """

    def test_short_packets_are_not_counted(self):
        _, ch, calib, _, ac = H.ctx()
        dec = PacketDecoder(calib.packet, calib.processing, ac,
                            range_mode=ch.lidar.range_mode, z_down=ch.lidar.z_down,
                            max_range_m=ch.lidar.max_range_m)
        size, rng, bufs = calib.packet.size, np.random.default_rng(1), []
        for i in range(901):
            b = bytearray(rng.integers(0, 256, size, dtype=np.uint8).tobytes())
            ang = int((i * 900.0 / 901) * 100) % 36000
            for off in (calib.packet.angle1_offset, calib.packet.angle2_offset):
                b[off:off + 2] = int(ang).to_bytes(2, "little")
            bufs.append(bytes(b)[:size // 2] if i % 2 == 0 else bytes(b))
        _xyz, _ring, _it, info = dec.decode_frame_batch(bufs, 1, 1.0)
        self.assertEqual(451, info["dropped"], "夹具没造出预期的残包")
        self.assertEqual(450, info["packets"])
        self.assertNotEqual(len(bufs), info["packets"],
                            "有效数与收到数必须不同，否则这条用例证明不了什么")

    def test_live_source_preserves_decode_loss_through_roi(self):
        """真实源输出的 raw 与 ROI 帧都须保留已知损失，不能只检查解码统计。"""
        import time
        from app.radar_source import LiveHub, LiveSource
        _, ch, calib, _, ac = H.ctx()
        dec = PacketDecoder(calib.packet, calib.processing, ac,
                            range_mode=ch.lidar.range_mode, z_down=ch.lidar.z_down,
                            max_range_m=ch.lidar.max_range_m)
        source = LiveSource(dec, hub=LiveHub(bind_ip="127.0.0.1", udp_port=0),
                            source_ip="127.0.0.2", roi=calib.roi)
        self.addCleanup(source.close)
        source.keep_raw = True
        bufs = [make_synthetic_packet(calib.packet, azimuth_deg=i * .4,
                                       raw_distance=5000, seed=i) for i in range(900)]
        bufs[::2] = [b[:calib.packet.size // 2] for b in bufs[::2]]
        now = time.time()
        frame = source._emit_batch(bufs, [now] * len(bufs), now)
        raw = source.take_raw()
        self.assertIsNotNone(frame)
        self.assertIsNotNone(raw)
        self.assertGreater(raw.n, 0)
        self.assertLess(frame.n, raw.n, "夹具须实际经过 ROI 裁剪")
        for observed in (raw, frame):
            self.assertEqual(observed.packets, 450)
            self.assertEqual(observed.dropped_packets, 450)

    def test_live_source_emits_observable_empty_frame_when_all_packets_fail(self):
        """全坏批次仍有可见的观测故障，不能靠不出帧维持旧许可。"""
        import time
        from app.radar_source import LiveHub, LiveSource
        _, ch, calib, _, ac = H.ctx()
        dec = PacketDecoder(calib.packet, calib.processing, ac,
                            range_mode=ch.lidar.range_mode, z_down=ch.lidar.z_down,
                            max_range_m=ch.lidar.max_range_m)
        source = LiveSource(dec, hub=LiveHub(bind_ip="127.0.0.1", udp_port=0),
                            source_ip="127.0.0.2", roi=calib.roi)
        self.addCleanup(source.close)
        now = time.time()
        frame = source._emit_batch([b"\0"] * 900, [now] * 900, now)
        self.assertIsNotNone(frame)
        self.assertEqual((frame.n, frame.packets, frame.dropped_packets), (0, 0, 900))


if __name__ == "__main__":
    unittest.main()
