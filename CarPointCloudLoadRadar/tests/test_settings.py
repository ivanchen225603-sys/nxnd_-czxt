# -*- coding: utf-8 -*-
"""配置与标定解析的测试 —— 这些是最该先过的，因为后面所有环节都依赖它们。"""
from __future__ import annotations

import os
import unittest

import numpy as np

from tests import _helpers as H
from app.settings import load_settings, load_calib, root


class TestSettings(unittest.TestCase):

    def test_channels_load(self):
        st, ch = H.first_channel()
        self.assertTrue(len(st.channels) >= 1)
        self.assertTrue(ch.id)
        # 绝对路径必须都已经解析出来
        for attr in ("abs_config_path", "abs_devices_path", "abs_output_dir",
                     "abs_baseline_dir", "abs_angle_cache"):
            p = getattr(ch, attr)
            self.assertTrue(p, f"{attr} 未解析")
            self.assertTrue(os.path.isabs(p), f"{attr} 不是绝对路径: {p}")

    def test_config_files_exist(self):
        _, ch, calib, dev, ac = H.ctx()
        self.assertTrue(os.path.exists(calib.path))
        self.assertTrue(os.path.exists(dev.path))
        self.assertTrue(os.path.exists(ch.abs_angle_cache))

    def test_packet_layout_consistent(self):
        """packet 段的偏移必须自洽，否则解码一定错位。"""
        _, _, calib, _, _ = H.ctx()
        p = calib.packet
        self.assertEqual(p.size, 1100)
        self.assertEqual(p.channels, 128)
        self.assertEqual(p.point_size, 4)
        # 每 block 数据段长度 = 128 通道 × 4 字节
        block_len = p.channels * p.point_size
        self.assertLessEqual(p.block1_data_offset + block_len, p.size)
        self.assertLessEqual(p.block2_data_offset + block_len, p.size)
        # 两块的数据段不能重叠（重叠说明偏移抄错了）
        self.assertGreaterEqual(p.block2_data_offset,
                                p.block1_data_offset + block_len)
        # IMU 段在数据之后
        self.assertGreaterEqual(p.imu_offset, p.block2_data_offset + 126 * 4)

    def test_roi_normalized_to_corner_pair(self):
        _, _, calib, _, _ = H.ctx()
        lo, hi = (np.asarray(calib.roi[0], np.float64),
                  np.asarray(calib.roi[1], np.float64))
        self.assertTrue((hi > lo).all(), f"ROI 上下界顺序不对: {calib.roi}")
        self.assertAlmostEqual(float(lo[2]), -6.0, places=6)
        self.assertAlmostEqual(float(hi[2]), -0.75, places=6)

    def test_gate_thresholds_sane(self):
        _, _, _, dev, _ = H.ctx()
        g = dev.gate
        self.assertGreater(g.min_inliers, 0)
        self.assertTrue(0 < g.min_inlier_ratio < 1)
        self.assertTrue(0 < g.normal_z_min <= 1)
        self.assertGreater(g.distance_m, 0)
        self.assertGreater(g.z_above_min_max_m, 0)
        self.assertGreater(dev.min_save_points, 0)

    def test_angle_cache_verified(self):
        """角度表的 SHA-256 校验必须通过 —— 这是"标定文件没被改坏"的硬保证。"""
        _, _, _, _, ac = H.ctx()
        self.assertTrue(ac.loaded, ac.error)
        self.assertGreaterEqual(ac.channels_seen, 128)
        self.assertTrue(ac.hash_ok, f"校验值不匹配: {ac.error}\n{ac.check()}")

    def test_range_mode_known(self):
        from app.radar_packet import RANGE_MODES
        _, ch, _, _, _ = H.ctx()
        self.assertIn(ch.lidar.range_mode, RANGE_MODES)


class TestGroupGuard(unittest.TestCase):
    """`group` 说的是"这几台雷达合扫同一辆车"，但本版本**没有实现聚合**。

    照着填下去，两台雷达会各判各的：同一辆车两份独立结论、分别落盘、不报错。
    使用者的意图和系统的行为完全相反 —— 必须启动即拦。
    """

    def _ch(self, cid, group="", enabled=True):
        from app.settings import ChannelCfg
        return ChannelCfg(id=cid, group=group, enabled=enabled)

    def test_same_group_on_two_channels_raises(self):
        from app.settings import check_groups
        with self.assertRaises(ValueError) as cm:
            check_groups([self._ch("CH1", "A"), self._ch("CH2", "A")], "t.json")
        msg = str(cm.exception)
        self.assertIn("CH1", msg)
        self.assertIn("CH2", msg)
        self.assertIn("没有实现聚合", msg, "报错必须说清楚为什么，否则现场只会一脸懵")

    def test_empty_groups_are_fine(self):
        from app.settings import check_groups
        check_groups([self._ch("CH1"), self._ch("CH2"), self._ch("CH3")], "t")

    def test_distinct_groups_are_fine(self):
        from app.settings import check_groups
        check_groups([self._ch("CH1", "A"), self._ch("CH2", "B")], "t")

    def test_disabled_channel_does_not_count(self):
        from app.settings import check_groups
        check_groups([self._ch("CH1", "A"), self._ch("CH2", "A", enabled=False)], "t")

    def test_shipped_config_passes(self):
        """现场那份配置必须能过 —— 4 路的 group 都是留空的。"""
        from app.settings import load_settings
        st = load_settings("")
        self.assertEqual(len(st.channels), 4)


class TestPerChannelYaw(unittest.TestCase):
    """逐通道方位角微调：四条车道并不平行，共用的 yaw 只能对齐其中一条。

    2026-09-14 实测 27 份实车点云，逐格量车厢左右沿对 y 的斜率
    （= 车相对 ROI 长轴停斜多少度）：

        CH1  -1.50°~+0.50°（14 份，std 0.6°）  与 ROI 对齐
        CH4  +1.00°~+2.50°（6 份，std 0.6°）
        CH3  +4.25°~+6.00°（6 份，std 0.7°）   ← 每一份都斜

    通道内散布不到 1°、通道间差 5.6° —— 不同的车、不同时间、同一条道
    停出同一个角度，这是系统性偏差而不是司机随机停斜。
    后果：CH3 的车厢最近只差 **0.01 m** 就被 ROI 的 x 边界切掉（172808），
    而车厢被切会直接少算体积。逐通道补偿后四路统一回到约 0.44 m 余量。

    ★ 符号约定（2026-09-16 实测钉死，别再推导错方向）
      量出的停车角 θ = atan(dx/dy)（车厢中心线对 +y 的斜率），
      而**要补偿的增量是 -θ**，不是 +θ：
          把 lidar.yaw_offset_deg 加 d，等价于点云绕 z 转 d，
          残余角变成 θ + d（已用真解码器逐点验证到 2.6e-06 m）。
      CH2 实测 θ = -3.06°（25 份，std 0.30），所以补 +3.0 → -183 + 3.0 = **-180.0**。
      反着补（-186.0）会把倾斜**加倍**：车厢 x 宽度 3.04 → 3.60 m、
      最小 x 余量 +0.05 → -0.47 m，等于主动把车厢切掉。
      自检：CH3/CH4 补过之后残余角 -0.46° / +0.30°，细扫最优点就落在 0 附近。
    """

    def test_absent_means_no_override(self):
        """不填就是不叠加 —— 老配置的行为必须一个字不变。"""
        from app.settings import LidarCfg
        self.assertIsNone(LidarCfg().yaw_offset_deg)

    def test_channels_json_values_are_parsed(self):
        st = load_settings("")
        got = {c.id: c.lidar.yaw_offset_deg for c in st.channels}
        self.assertIsNone(got.get("CH1"), "CH1 实测无偏差，不该设覆盖值")
        for cid in ("CH2", "CH3", "CH4"):
            self.assertIsNotNone(got.get(cid), f"{cid} 应当有逐通道 yaw")
        # CH2：2026-09-15 上午 25 份全量帧实测 θ = -3.06°（std 0.30），
        # 补偿量是 -θ = +3.06，取整到 +3.0 → -183 + 3.0 = -180.0。
        # 这里钉死**符号**：写成 -186 是把倾斜加倍，属于踩过的坑。
        self.assertAlmostEqual(got["CH2"], -180.0, places=3,
                               msg="CH2 应为 -180.0（= -183 + 3.0）；"
                                   "-186 方向反了，会把车厢倾斜加倍")

    def test_override_is_within_a_few_degrees_of_the_shared_value(self):
        """覆盖值只应当是**微调**。差出十几度说明搞错了对象（那是朝向问题，
        该去改共用的 yaw_offset_deg，见 docs/08 §3.0）。"""
        st = load_settings("")
        base = load_calib(root("config", "config.json")).processing.yaw_offset_deg
        for c in st.channels:
            if c.lidar.yaw_offset_deg is None:
                continue
            d = abs(c.lidar.yaw_offset_deg - base)
            self.assertLess(d, 10.0,
                            f"{c.id} 的逐通道 yaw 与共用值差了 {d:.1f}°，"
                            f"这不是微调 —— 先确认不是整体朝向搞错了")

    def test_build_source_applies_it_without_touching_shared_calib(self):
        """★ 只能改这一路解码用的副本，共用的 calib 一个字不能动 —— 否则四路互相踩。"""
        from app.radar_source import build_source
        from app.pointcloud import AngleCache
        st = load_settings("")
        ch = [c for c in st.channels if c.lidar.yaw_offset_deg is not None][0]
        calib = load_calib(ch.abs_config_path)
        before = calib.processing.yaw_offset_deg
        angle = AngleCache(ch.abs_angle_cache).load()
        src = build_source(ch, calib, angle, mode="synthetic")
        self.assertEqual(calib.processing.yaw_offset_deg, before,
                         "build_source 改到了共用的 calib 上")
        del src


if __name__ == "__main__":
    unittest.main()
