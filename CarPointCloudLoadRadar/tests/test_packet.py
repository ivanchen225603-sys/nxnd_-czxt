# -*- coding: utf-8 -*-
"""
报文编解码测试（jt128.dll 等价物）。

核心思路是**闭环**：用 RadarSimulator 按几何真值编码出原始报文，
再走真正的 PacketDecoder 解回来，跟真值比对。
这样一次就同时验证了「编码」和「解码」两侧——
任何一侧的偏移/量纲/符号搞错，bbox 立刻对不上。
"""
from __future__ import annotations

import unittest

import numpy as np

from tests import _helpers as H
from app.radar_packet import (PacketDecoder, PacketError, RANGE_MODES,
                              imu_magnitude, make_synthetic_packet)
from app.radar_sim import RadarSimulator, make_demo_scene, box_truck, Box
from app.pointcloud import Frame


def _ctx():
    _, ch, calib, _, ac = H.ctx()
    dec = PacketDecoder(calib.packet, calib.processing, ac,
                        range_mode=ch.lidar.range_mode, z_down=ch.lidar.z_down,
                        max_range_m=ch.lidar.max_range_m)
    return ch, calib, ac, dec


def _sim(calib, ac, ch, **kw):
    kw.setdefault("az_step_deg", 2.0)
    kw.setdefault("range_noise_m", 0.0)
    kw.setdefault("dropout_prob", 0.0)
    kw.setdefault("seed", 3)
    return RadarSimulator(calib.packet, calib.processing, ac,
                          range_mode=ch.lidar.range_mode,
                          z_down=ch.lidar.z_down, **kw)


def _frame(packets, dec):
    xs, rs = [], []
    for b in packets:
        pkt = dec.decode(b)
        xyz, ring, _it = dec.to_cartesian(pkt)
        if len(xyz):
            xs.append(xyz)
            rs.append(ring)
    if not xs:
        return None
    return Frame(np.concatenate(xs, 0), np.concatenate(rs, 0))


class TestRangeModes(unittest.TestCase):

    def test_all_modes_present(self):
        for m in ("raw_offset", "scale", "scale_offset", "scale_plus"):
            self.assertIn(m, RANGE_MODES)

    def test_decoder_rejects_unknown_mode(self):
        _, ch, calib, _, ac = H.ctx()
        with self.assertRaises(Exception):
            PacketDecoder(calib.packet, calib.processing, ac,
                          range_mode="no_such_mode")


class TestLayout(unittest.TestCase):

    def test_points_per_packet(self):
        _, calib, _, dec = _ctx()
        # 2 个 block，每 block 128 通道
        self.assertEqual(dec._npoints, calib.packet.channels * 2)

    def test_decode_rejects_short_packet(self):
        _, _, _, dec = _ctx()
        with self.assertRaises(PacketError):
            dec.decode(b"\x00" * 10)                  # 远短于 1100B

    def test_blocks_contiguous_flag(self):
        _, _, _, dec = _ctx()
        # 只是不能崩；两种布局都要能自检
        self.assertIsInstance(dec._blocks_contiguous(), bool)


class TestClosedLoop(unittest.TestCase):

    def test_geometry_matches_truth(self):
        """编码 → 解码 的 bbox 必须与几何真值一致（< 0.10 m）。"""
        ch, calib, ac, dec = _ctx()
        sim = _sim(calib, ac, ch)
        boxes = make_demo_scene(loaded=False, z_floor=-5.0)
        truth = sim.truth_cloud(boxes, az_step=2.0)
        fr = _frame(sim.scan(boxes), dec)
        self.assertIsNotNone(fr)
        self.assertGreater(fr.n, 10000)

        tlo, thi = truth.min(0), truth.max(0)
        flo, fhi = fr.xyz.min(0), fr.xyz.max(0)
        err = float(np.max(np.abs(np.array([flo - tlo, fhi - thi]))))
        self.assertLess(err, 0.10, f"bbox 偏差 {err:.4f} m")

    def test_z_points_down(self):
        """地板必须落在 z≈-5（z_down=true 的直接证据）。"""
        ch, calib, ac, dec = _ctx()
        sim = _sim(calib, ac, ch)
        fr = _frame(sim.scan(make_demo_scene(loaded=False, z_floor=-5.0)), dec)
        self.assertIsNotNone(fr)
        self.assertTrue(-6.0 < fr.xyz[:, 2].min() < -3.0,
                        f"最低点 z={fr.xyz[:, 2].min():.3f}，"
                        f"若为正值说明 z_down 应为 false")

    def test_flipping_z_down_mirrors_z(self):
        ch, calib, ac, _dec = _ctx()
        s_down = _sim(calib, ac, ch)
        f_down = _frame(s_down.scan(make_demo_scene(loaded=False)), _dec)
        dec_up = PacketDecoder(calib.packet, calib.processing, ac,
                               range_mode=ch.lidar.range_mode, z_down=False,
                               max_range_m=ch.lidar.max_range_m)
        s_up = _sim(calib, ac, ch)
        f_up = _frame(s_up.scan(make_demo_scene(loaded=False)), dec_up)
        self.assertIsNotNone(f_down)
        self.assertIsNotNone(f_up)
        # 仅 z 符号相反，x/y 不变
        np.testing.assert_allclose(f_down.xyz[:, :2], f_up.xyz[:, :2], atol=0.05)
        self.assertAlmostEqual(float(f_down.xyz[:, 2].min()),
                               -float(f_up.xyz[:, 2].max()), delta=0.15)

    def test_ring_encoding_range(self):
        """ring = block_index×128 + 通道号 → 取值范围必须落在 [0, 255]。"""
        ch, calib, ac, dec = _ctx()
        sim = _sim(calib, ac, ch)
        fr = _frame(sim.scan(make_demo_scene(loaded=False)), dec)
        self.assertIsNotNone(fr)
        self.assertIsNotNone(fr.ring)
        self.assertGreaterEqual(int(fr.ring.min()), 0)
        self.assertLessEqual(int(fr.ring.max()), 255)

    def test_loaded_scene_differs_from_empty(self):
        """
        载货场景与空车场景必须显著不同，否则演示/自检没有意义。

        实测（az_step=2°，无噪声）：
            空车  z min=-5.000  p50=-5.000   ← 回波几乎全落在车厢底板
            载货  z min=-4.503  p50=-3.830   ← 货堆顶面挡住了底板
        所以判据是「底板回波被抬高」，而不是"整体 bbox 变化"
        —— 车厢外壳尺寸不变，bbox 基本不动的。
        """
        ch, calib, ac, dec = _ctx()
        f_empty = _frame(_sim(calib, ac, ch).scan(make_demo_scene(loaded=False)), dec)
        f_load = _frame(_sim(calib, ac, ch).scan(make_demo_scene(loaded=True)), dec)
        self.assertIsNotNone(f_empty)
        self.assertIsNotNone(f_load)

        ze, zl = f_empty.xyz[:, 2], f_load.xyz[:, 2]
        # 空车地板应当精确落在 z_floor
        self.assertAlmostEqual(float(ze.min()), -5.0, delta=0.05)
        # 载货：底板回波被货堆顶挡住，最低点抬高
        self.assertGreater(float(zl.min()), float(ze.min()) + 0.3)
        # 载货：中位数显著上移（大部分回波来自货堆顶面而非底板）
        self.assertGreater(float(np.percentile(zl, 50)),
                           float(np.percentile(ze, 50)) + 0.8)
        # 两场场景的点数应当接近（同样一圈扫描）
        self.assertEqual(f_empty.n, f_load.n)

    def test_to_cartesian_returns_three_arrays(self):
        ch, calib, ac, dec = _ctx()
        pkt = dec.decode(make_synthetic_packet(calib.packet, azimuth_deg=30.0))
        out = dec.to_cartesian(pkt)
        self.assertEqual(len(out), 3)
        xyz, ring, intensity = out
        self.assertEqual(xyz.shape[1], 3)
        self.assertEqual(len(ring), len(xyz))
        self.assertEqual(len(intensity), len(xyz))


class TestBatchScanEquivalence(unittest.TestCase):
    """
    整圈批量生成（scan_packets / _dirs_many）必须与逐包版几何**等价**。

    为什么单独立一组：仿真源在 live 化的过程中被向量化过（4 路并发时逐包
    360 次小 numpy 调用会反复抢 GIL，帧率退化到 ~1.5fps），现在雷达源走
    `scan_packets`，而单测/探针仍在走 `_one_packet`。两条生成路径若不一致，
    就会出现"测试里空间位置好好的、看板上整体错位/镜像"这种最难查的问题。

    取证口径：无噪声、无丢点时 raw 完全由几何决定 → 逐包 raw 必须严格相等。
    """

    def test_dirs_many_matches_dirs(self):
        """批量方向向量必须与逐个算的严格一致（向量化最容易在这偏一位）。"""
        ch, calib, ac, _dec = _ctx()
        sim = _sim(calib, ac, ch)
        azs = np.array([0.0, 0.01, 17.5, 90.0, 179.99, 271.3, 359.99])
        many = sim._dirs_many(azs)
        self.assertEqual(many.shape, (len(azs), sim._pps, 3))
        for i, az in enumerate(azs):
            np.testing.assert_allclose(many[i], sim._dirs(float(az)), atol=1e-12,
                                       err_msg=f"az={az} 的批量方向与逐包不一致")

    def test_ranges_are_identical_per_packet(self):
        ch, calib, ac, dec = _ctx()
        sim = _sim(calib, ac, ch)                    # noise=0 dropout=0 az_step=2.0
        boxes = make_demo_scene(loaded=False, z_floor=-5.0)
        n, step = 180, 2.0
        batch = sim.scan_packets(boxes, 0.0, step, n)
        ones = [sim._one_packet_with(boxes, float(a)) for a in np.arange(n) * step]
        self.assertEqual(len(batch), n)
        self.assertEqual(len(ones), n)
        for i in (0, 1, 37, 90, n - 1):
            rb = dec.decode(batch[i]).ranges_raw
            ro = dec.decode(ones[i]).ranges_raw
            np.testing.assert_array_equal(
                rb, ro, err_msg=f"第 {i} 包 raw 与逐包版不一致（量纲/方位角/命中判定）")

    def test_ranges_identical_on_loaded_scene(self):
        """载货场景（有货堆遮挡）也要逐包一致 —— 遮挡判定最容易被向量化写错。"""
        ch, calib, ac, dec = _ctx()
        sim = _sim(calib, ac, ch)
        # ★ scan_packets 的 az_step 与 sim.az_step_deg 必须一致：
        #   第 2 块的方位角是 az + az_step/2，两者不一致就差了半步（下面有用例钉住）
        step = 4.0
        sim.az_step_deg = step
        boxes = make_demo_scene(loaded=True, z_floor=-5.0)
        n = 90
        batch = sim.scan_packets(boxes, 0.0, step, n)
        for i in (0, 22, n - 1):
            ro = dec.decode(sim._one_packet_with(boxes, float(i * step))).ranges_raw
            np.testing.assert_array_equal(dec.decode(batch[i]).ranges_raw, ro)

    def test_block2_azimuth_is_half_step_ahead(self):
        """第 2 块的方位角 = 第 1 块 + step/2（两条生成路径都必须如此）。"""
        ch, calib, ac, dec = _ctx()
        sim = _sim(calib, ac, ch)
        sim.az_step_deg = 4.0
        b = sim.scan_packets(make_demo_scene(loaded=False), 10.0, 4.0, 1)[0]
        az = dec.decode(b).azimuth_deg
        self.assertAlmostEqual(az[0], 10.0, places=2)
        self.assertAlmostEqual(az[1], 12.0, places=2)

    def test_ranges_differ_if_step_mismatched(self):
        """
        反证：step 与 az_step_deg 不一致时确实会算出不同的第 2 块 ——
        说明上面那些"逐包一致"的用例不是碰巧为真（它们真的在比对数据）。
        """
        ch, calib, ac, dec = _ctx()
        sim = _sim(calib, ac, ch)            # az_step_deg = 2.0
        boxes = make_demo_scene(loaded=False)
        b = sim.scan_packets(boxes, 0.0, 4.0, 1)[0]      # step=4 → block2 在 +2°
        o = sim._one_packet_with(boxes, 0.0)             # block2 在 +1°
        self.assertFalse(
            np.array_equal(dec.decode(b).ranges_raw, dec.decode(o).ranges_raw),
            "两条路径居然一致 —— 说明这个用例没在比第 2 块，逐包等价断言不可信")

    def test_azimuth_fields_match(self):
        ch, calib, ac, dec = _ctx()
        sim = _sim(calib, ac, ch)
        batch = sim.scan_packets(make_demo_scene(loaded=False), 0.0, 2.0, 5)
        for i, b in enumerate(batch):
            az = i * 2.0
            pkt = dec.decode(b)
            self.assertAlmostEqual(pkt.azimuth_deg[0], az, places=2)
            self.assertAlmostEqual(pkt.azimuth_deg[1], az + 1.0, places=2)

    def test_start_angle_wraps(self):
        """az_start 移到 358° 之后必须绕回 0.xx°，不能出现 400+ 的非法方位角。"""
        ch, calib, ac, dec = _ctx()
        sim = _sim(calib, ac, ch)
        batch = sim.scan_packets(make_demo_scene(loaded=False), 358.0, 2.0, 3)
        for b in batch:
            for az in dec.decode(b).azimuth_deg:
                self.assertGreaterEqual(az, 0.0)
                self.assertLess(az, 360.0)

    def test_frames_have_same_geometry(self):
        ch, calib, ac, dec = _ctx()
        sim = _sim(calib, ac, ch)
        step = 4.0
        sim.az_step_deg = step
        boxes = make_demo_scene(loaded=True, z_floor=-5.0)
        n = 90
        fb = _frame(sim.scan_packets(boxes, 0.0, step, n), dec)
        fo = _frame([sim._one_packet_with(boxes, float(a))
                     for a in np.arange(n) * step], dec)
        self.assertIsNotNone(fb)
        self.assertIsNotNone(fo)
        self.assertEqual(fb.n, fo.n)
        np.testing.assert_allclose(fb.xyz, fo.xyz, atol=1e-6)

    def test_scan_defers_to_scan_packets(self):
        """scan() 只是 scan_packets 的取整包装 —— 别在两处各写一套扫描范围逻辑。"""
        ch, calib, ac, dec = _ctx()
        a, b = _sim(calib, ac, ch), _sim(calib, ac, ch)
        boxes = make_demo_scene(loaded=False)
        fa = _frame(a.scan(boxes, 0.0, 360.0), dec)          # az_step=2 → 180 包
        fb = _frame(b.scan_packets(boxes, 0.0, 2.0, 180), dec)
        self.assertIsNotNone(fa)
        self.assertIsNotNone(fb)
        self.assertEqual(fa.n, fb.n)
        np.testing.assert_allclose(fa.xyz, fb.xyz, atol=1e-6)

    def test_zero_packets_floors_to_one(self):
        """
        n_packets<=0 时批量版**兜底生成 1 包**（不是空表也不是崩）。

        这是有意的防御：调用方（RadarSimSource）用 packets_per_frame 控制一帧，
        若配置被人改成 0，宁可出一包"几乎全是无效点"的帧让上层走"点云不足"
        的正常拒绝路径，也不要返回空列表让通道静默僵死。
        `scan()` 自己会先挡住跨度<=0 的情况，所以正常路径不会走到这里。
        """
        ch, calib, ac, _dec = _ctx()
        sim = _sim(calib, ac, ch)
        boxes = make_demo_scene(loaded=False)
        for n in (0, -5):
            self.assertEqual(len(sim.scan_packets(boxes, 0.0, 2.0, n)), 1,
                             f"n_packets={n} 的兜底行为变了")

    def test_scan_empty_span_returns_nothing(self):
        ch, calib, ac, _dec = _ctx()
        sim = _sim(calib, ac, ch)
        self.assertEqual(sim.scan(make_demo_scene(loaded=False), 10.0, 10.0), [])


class TestIMU(unittest.TestCase):

    def test_imu_magnitude_on_synthetic(self):
        _, calib, _, dec = _ctx()
        pkt = dec.decode(make_synthetic_packet(calib.packet, seed=2))
        mag = imu_magnitude(pkt)
        self.assertTrue(np.isfinite(mag))
        self.assertGreater(mag, 0.0)          # 合成包写入了 acc_z=2048


if __name__ == "__main__":
    unittest.main()
