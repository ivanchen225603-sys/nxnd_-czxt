# -*- coding: utf-8 -*-
"""PLY 导出的测试。

最要紧的一条是**去重**：2026-09-15 实测每帧约 198 000 点里只有约 99 000 个
是不同的测量 —— 94% 的点成对出现，每一对的 ring 之差恰好是 128。
`ring = block_index x 128 + 通道号`，所以一个报文的两个 block 装的是
同一次测量（原厂存档 `return_mode = 2`，双回波；实心表面的最强回波与
最后回波落在同一点）。

去重必须**只去真重复**：把相邻但不同的两个点合掉会抹掉货面细节，
而漏去重则让文件白白大一倍、CloudCompare 里满是重影。
"""
from __future__ import annotations

import os
import sys
import tempfile
import unittest

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "cli")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from export_ply import colorize, dedup, write_ply      # noqa: E402


class TestDedup(unittest.TestCase):

    def test_removes_the_dual_return_twin(self):
        """同坐标、ring 差 128 的一对，应当只剩一个。"""
        xyz = np.array([[1.0, 2.0, 3.0], [1.0, 2.0, 3.0]])
        ring = np.array([7, 135], np.int32)
        out, _r, dropped = dedup(xyz, ring)
        self.assertEqual(dropped, 1)
        self.assertEqual(len(out), 1)

    def test_keeps_the_lower_ring(self):
        """保留 block 0（ring 较小的那个），口径固定才可复现。"""
        xyz = np.array([[1.0, 2.0, 3.0], [1.0, 2.0, 3.0]])
        _o, r, _d = dedup(xyz, np.array([135, 7], np.int32))
        self.assertEqual(int(r[0]), 7)

    def test_does_not_merge_nearby_but_distinct_points(self):
        """★ 相差 1 mm 的两个点是真实的不同测量，不能合掉。"""
        xyz = np.array([[1.0000, 2.0, 3.0], [1.0010, 2.0, 3.0]])
        out, _r, dropped = dedup(xyz, np.array([1, 2], np.int32))
        self.assertEqual(dropped, 0)
        self.assertEqual(len(out), 2)

    def test_empty_input(self):
        out, _r, dropped = dedup(np.zeros((0, 3)), np.zeros(0, np.int32))
        self.assertEqual(len(out), 0)
        self.assertEqual(dropped, 0)

    def test_no_duplicates_changes_nothing(self):
        xyz = np.arange(30, dtype=float).reshape(10, 3)
        out, _r, dropped = dedup(xyz, np.arange(10, dtype=np.int32))
        self.assertEqual(dropped, 0)
        self.assertEqual(len(out), 10)

    def test_halves_a_realistic_dual_return_frame(self):
        """整帧级别：每个点复制一份、ring 加 128，应当正好去掉一半。"""
        rng = np.random.default_rng(0)
        base = rng.normal(0, 5, (5000, 3))
        xyz = np.vstack([base, base])
        ring = np.concatenate([np.arange(5000) % 128,
                               np.arange(5000) % 128 + 128]).astype(np.int32)
        out, _r, dropped = dedup(xyz, ring)
        self.assertEqual(len(out), 5000)
        self.assertEqual(dropped, 5000)

    def test_order_is_preserved(self):
        """保持原始顺序，方便与 .xyz 逐点对照。"""
        xyz = np.array([[5.0, 0, 0], [1.0, 0, 0], [3.0, 0, 0]])
        out, _r, _d = dedup(xyz, np.arange(3, dtype=np.int32))
        np.testing.assert_allclose(out[:, 0], [5.0, 1.0, 3.0])


class TestColorize(unittest.TestCase):

    def test_height_mode_varies_with_height(self):
        h = np.linspace(0, 4, 100)
        c = colorize(h, "height", np.zeros(100, np.int32))
        self.assertEqual(c.shape, (100, 3))
        self.assertGreater(len(np.unique(c, axis=0)), 10)

    def test_none_mode_is_uniform(self):
        c = colorize(np.linspace(0, 4, 50), "none", np.zeros(50, np.int32))
        self.assertEqual(len(np.unique(c, axis=0)), 1)

    def test_constant_height_does_not_divide_by_zero(self):
        c = colorize(np.full(50, 2.0), "height", np.zeros(50, np.int32))
        self.assertTrue(np.isfinite(c).all())

    def test_output_is_uint8(self):
        c = colorize(np.linspace(0, 4, 20), "height", np.zeros(20, np.int32))
        self.assertEqual(c.dtype, np.uint8)


class TestWritePly(unittest.TestCase):
    """CloudCompare 只认 ASCII 头 —— 头里混进中文会直接写不出来（踩过一次）。"""

    def _round_trip(self, ascii_mode, ground_z=-5.23):
        xyz = np.array([[1.0, 2.0, -3.0], [4.0, 5.0, -6.0]])
        rgb = np.array([[255, 0, 0], [0, 255, 0]], np.uint8)
        h = np.array([2.23, -0.77], np.float32)
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "t.ply")
            write_ply(p, xyz, rgb, h, ascii_mode=ascii_mode, ground_z=ground_z)
            with open(p, "rb") as f:
                head = b""
                while b"end_header\n" not in head:
                    line = f.readline()
                    if not line:
                        break
                    head += line
                body = f.read()
            return head, body

    def test_binary_header_and_payload(self):
        head, body = self._round_trip(False)
        self.assertIn(b"binary_little_endian", head)
        self.assertIn(b"element vertex 2", head)
        self.assertIn(b"property float height", head)
        rec = np.frombuffer(body, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                         ("red", "u1"), ("green", "u1"), ("blue", "u1"),
                                         ("height", "<f4")])
        self.assertEqual(len(rec), 2)
        self.assertAlmostEqual(float(rec["x"][0]), 1.0, places=4)
        self.assertAlmostEqual(float(rec["height"][1]), -0.77, places=4)
        self.assertEqual(int(rec["red"][0]), 255)

    def test_ascii_mode(self):
        head, body = self._round_trip(True)
        self.assertIn(b"format ascii", head)
        self.assertEqual(len(body.strip().split(b"\n")), 2)

    def test_header_is_pure_ascii(self):
        """★ 中文注释会让 encode('ascii') 抛 UnicodeEncodeError。"""
        for mode in (True, False):
            head, _b = self._round_trip(mode)
            head.decode("ascii")            # 不是 ASCII 就在这里抛

    def test_ground_z_is_recorded(self):
        """离地高度的基准必须写进文件，否则 scalar field 没法解释。"""
        head, _b = self._round_trip(False, ground_z=-5.23)
        self.assertIn(b"-5.230", head)


if __name__ == "__main__":
    unittest.main()
