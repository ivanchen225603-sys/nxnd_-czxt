# -*- coding: utf-8 -*-
"""点云容器与角度标定表的测试。"""
from __future__ import annotations

import os
import tempfile
import unittest

import numpy as np

from tests import _helpers as H
from app.settings import BASELINE_SUBDIR
from app.pointcloud import (Frame, load_xyz, save_xyz, apply_roi, normalize_roi,
                            roi_as_ranges, export_ply, export_csv, AngleCache)


class TestFrame(unittest.TestCase):

    def test_basic_quantities(self):
        xyz = np.array([[0, 0, 0], [2, 0, 0], [0, 4, 0], [0, 0, 6]], np.float32)
        f = Frame(xyz)
        self.assertEqual(f.n, 4)
        np.testing.assert_allclose(f.centroid(), [0.5, 1.0, 1.5])
        np.testing.assert_allclose(f.bbox()[0], [0, 0, 0])
        np.testing.assert_allclose(f.bbox()[1], [2, 4, 6])
        np.testing.assert_allclose(f.extent(), [2, 4, 6])

    def test_ring_length_mismatch_dropped(self):
        # 设计如此：ring 长度对不上就静默置 None，绝不抛异常打断主循环
        # —— 车上掉包时 ring 偶有错位，宁可丢诊断信息也不能让服务崩。
        f = Frame(np.zeros((3, 3), np.float32), np.zeros(2, np.uint16))
        self.assertIsNone(f.ring)

    def test_empty_frame_quantities(self):
        f = Frame(np.zeros((0, 3), np.float32))
        self.assertEqual(f.n, 0)
        np.testing.assert_allclose(f.centroid(), [0, 0, 0])
        np.testing.assert_allclose(f.extent(), [0, 0, 0])

    def test_slice_and_mask(self):
        xyz = np.array([[0, 0, -1], [0, 0, 0], [0, 0, 1]], np.float32)
        f = Frame(xyz)
        s = f.slice_z(-0.5, 1.5)
        self.assertEqual(s.n, 2)

    def test_transform_translation(self):
        f = Frame(np.eye(3, dtype=np.float32))
        g = f.transformed(np.eye(3), np.array([1, 2, 3], np.float32))
        np.testing.assert_allclose(g.xyz, f.xyz + [1, 2, 3], atol=1e-6)


class TestXYZIO(unittest.TestCase):

    def test_roundtrip(self):
        rng = np.random.default_rng(0)
        f = Frame((rng.random((999, 3)) * 10 - 5).astype(np.float32),
                  rng.integers(0, 256, 999).astype(np.uint16))
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "a.xyz")
            save_xyz(p, f)
            g = load_xyz(p)
            self.assertEqual(g.n, f.n)
            np.testing.assert_allclose(g.xyz, f.xyz, atol=1e-6)
            np.testing.assert_array_equal(g.ring, f.ring)
            # 文件大小 = 4 字节点数头 + 每点 (3×float32 + uint16) = 14 字节
            self.assertEqual(os.path.getsize(p), 4 + f.n * 14)

    def test_broken_file_raises(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "bad.xyz")
            with open(p, "wb") as fp:                    # 声称 16 点却只有 4 字节
                fp.write((16).to_bytes(4, "little") + b"\x00" * 4)
            with self.assertRaises(Exception):
                load_xyz(p)

    def test_empty_cloud(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "e.xyz")
            save_xyz(p, Frame(np.zeros((0, 3), np.float32)))
            self.assertEqual(load_xyz(p).n, 0)


class TestROI(unittest.TestCase):

    def test_normalize_axis_wise(self):
        lo, hi = normalize_roi({"x": [-2, 2], "y": [-11, 8], "z": [-6, -1]})
        np.testing.assert_allclose(lo, [-2, -11, -6])
        np.testing.assert_allclose(hi, [2, 8, -1])

    def test_normalize_corner_pair(self):
        lo, hi = normalize_roi(((-2, -11, -6), (2, 8, -1)))
        np.testing.assert_allclose(lo, [-2, -11, -6])
        np.testing.assert_allclose(hi, [2, 8, -1])

    def test_normalize_ranges(self):
        lo, hi = normalize_roi(((-2, 2), (-11, 8), (-6, -1)))
        np.testing.assert_allclose(lo, [-2, -11, -6])

    def test_apply_roi_filters(self):
        xyz = np.array([[0, 0, 0], [5, 0, 0], [0, 0, 0.5]], np.float32)
        f = apply_roi(Frame(xyz), ((-1, -1, -1), (1, 1, 1)))
        self.assertEqual(f.n, 2)

    def test_roi_as_ranges_roundtrip(self):
        r = roi_as_ranges(((-2, -11, -6), (2, 8, -1)))
        lo, hi = normalize_roi(r)
        np.testing.assert_allclose(lo, [-2, -11, -6])
        np.testing.assert_allclose(hi, [2, 8, -1])


class TestAngleCache(unittest.TestCase):

    def test_hash_caliber(self):
        """
        回归那条"永远对不上"的坑：
        校验值必须是对「表头 + 128 行」按 LF 归一化后算的 SHA-256，
        而不是整个文件（文件末尾还有 0xFF 填充），也不是 CRLF。
        """
        _, ch, _, _, ac = H.ctx()
        self.assertTrue(ac.hash_ok, f"校验失败: {ac.error}")
        self.assertEqual(os.path.getsize(ch.abs_angle_cache), 4226)

    def test_elevation_azimuth_shape(self):
        _, _, _, _, ac = H.ctx()
        self.assertTrue(hasattr(ac, "elevation"))
        self.assertTrue(hasattr(ac, "azimuth"))


class TestRingEncoding(unittest.TestCase):
    """
    ring = block_index × 128 + 通道号，取值 0..255。

    第一轮逆向报告里写的是"实测 0~187" —— 那是**看得不够全**。
    实测 10 份历史样本共 575595 点：
        ring 范围 0..255
        落在 [0,128) 的唯一值恰好 **128** 个（= 第一个 block 的 128 个通道全用到）
        落在 [128,256) 的只有 27 个（第二个 block 只用到一部分）
    "低半区恰好 128 个"就是 block 编码的直接证据，不是巧合。
    """

    def test_history_ring_is_two_block_encoded(self):
        import glob
        from app.pointcloud import load_xyz
        files = sorted(glob.glob(os.path.join(H.ROOT, *BASELINE_SUBDIR,
                                              "**", "*.xyz"), recursive=True))
        if not files:
            self.skipTest("没有历史样本")
        uniq = set()
        for p in files:
            f = load_xyz(p)
            if f.ring is not None:
                uniq |= set(np.unique(f.ring).tolist())
        self.assertTrue(uniq)
        self.assertGreaterEqual(min(uniq), 0)
        self.assertLessEqual(max(uniq), 255)
        low = [v for v in uniq if v < 128]
        # 第一个 block 的 128 个通道应当全部出现
        self.assertEqual(len(low), 128,
                         f"[0,128) 内的唯一 ring 值有 {len(low)} 个，应为 128")


class TestExport(unittest.TestCase):

    def test_ply_and_csv(self):
        rng = np.random.default_rng(2)
        f = Frame((rng.random((300, 3)) * 4 - 2).astype(np.float32),
                  rng.integers(0, 256, 300).astype(np.uint16))
        with tempfile.TemporaryDirectory() as d:
            ply = os.path.join(d, "a.ply")
            csv = os.path.join(d, "a.csv")
            export_ply(f, ply)
            export_csv(f, csv)
            self.assertTrue(os.path.getsize(ply) > 0)
            with open(ply, "rb") as fp:
                head = fp.readline()
            self.assertTrue(head.startswith(b"ply"))
            with open(csv, encoding="utf-8") as fp:
                self.assertEqual(len(fp.readlines()), f.n + 1)


class TestFrameFieldsSurviveDerivation(unittest.TestCase):
    """派生新帧时身份、有效包数和已知解码损失不得丢失。

    这类"字段悄悄掉了"的 bug 不报错，只会让上层判据静默失效：
    2026-09-14 加 Frame.packets（一圈的报文数，停稳判据靠它识别残缺帧）时，
    `_mask` 忘了带 —— 而 ROI 裁剪走的正是 `_mask`，于是判定层拿到的
    packets 恒为 0，残缺帧识别整个不生效，且一个字的报错都没有。
    """

    def _f(self):
        return Frame(np.arange(30, dtype=np.float32).reshape(10, 3),
                     np.arange(10, dtype=np.uint16),
                     seq=7, ts=123.5, gen=2, packets=450, dropped_packets=451)

    def _check(self, g, how):
        for name in ("seq", "ts", "gen", "packets", "dropped_packets"):
            self.assertEqual(getattr(g, name), getattr(self._f(), name),
                             f"{how} 之后 {name} 丢了")

    def test_mask_keeps_them(self):
        f = self._f()
        self._check(f._mask(np.array([True, False] * 5)), "_mask")

    def test_apply_roi_keeps_them(self):
        f = self._f()
        self._check(apply_roi(f, ((-1e9, -1e9, -1e9), (1e9, 1e9, 1e9))), "apply_roi")

    def test_transformed_keeps_them(self):
        f = self._f()
        self._check(f.transformed(np.eye(3)), "transformed")

    def test_slice_z_keeps_them(self):
        self._check(self._f().slice_z(2, 16), "slice_z")

    def test_old_positional_construction_has_no_known_decode_loss(self):
        f = Frame(np.zeros((1, 3)), None, 7, 123.5, 2, 901)
        self.assertEqual(f.dropped_packets, 0)


if __name__ == "__main__":
    unittest.main()

class TestReasonEncoding(unittest.TestCase):
    """`load_reason` 的编码回退 —— `docs/14` §2.1。

    以前写死 `encoding="utf-8"`，喂一个 GBK 文件就是 `UnicodeDecodeError`，
    而 `cli/verify.py`（回归闸门）、`cli/replay.py`、`cli/inspect_cloud.py`
    三个都调它 —— 一份旧文件就能把闸门打挂。
    原厂那套是 GBK 系的，麒麟上 locale 不对时落的盘也可能不是 UTF-8。
    """

    TXT = "底面内点占比过低"

    def _read(self, data: bytes) -> str:
        import tempfile
        from app.pointcloud import load_reason
        d = tempfile.mkdtemp()
        x = os.path.join(d, "a.xyz")
        with open(x + ".reason.txt", "wb") as f:
            f.write(data)
        return load_reason(x)

    def test_utf8(self):
        self.assertEqual(self._read(self.TXT.encode("utf-8")), self.TXT)

    def test_utf8_with_bom(self):
        self.assertEqual(self._read(b"\xef\xbb\xbf" + self.TXT.encode("utf-8")),
                         self.TXT)

    def test_gbk(self):
        """★ 这条是重点：以前会抛 UnicodeDecodeError。"""
        self.assertEqual(self._read(self.TXT.encode("gbk")), self.TXT)

    def test_binary_garbage_does_not_raise(self):
        """读不干净也不该让回归/回放崩掉 —— 坏字符显示成 U+FFFD。"""
        got = self._read(b"\xff\xfe\x00\x01")
        self.assertIsInstance(got, str)

    def test_missing_file(self):
        import tempfile
        from app.pointcloud import load_reason
        self.assertEqual(load_reason(os.path.join(tempfile.mkdtemp(), "none.xyz")), "")

    def test_roundtrip_is_unchanged(self):
        """合法 UTF-8 的读出内容必须与从前逐字相同，否则 verify.py 会破。"""
        import tempfile
        from app.pointcloud import save_reason, load_reason
        d = tempfile.mkdtemp()
        x = os.path.join(d, "a.xyz")
        save_reason(x + ".reason.txt", self.TXT)
        self.assertEqual(load_reason(x), self.TXT)
