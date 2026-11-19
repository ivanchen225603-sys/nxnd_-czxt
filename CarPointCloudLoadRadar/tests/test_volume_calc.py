# -*- coding: utf-8 -*-
"""带配准的体积计算测试。

这套东西存在的理由只有一条：`detect.compute_volume` 用**绝对坐标**逐格相减，
而 2026-09-15 实测 CH4 21 辆车的纵向停车位置 std 0.86 m、极差 4.00 m
（横向只有 0.083 m）。空车比载货车靠前一米，逐格相减就整体错位一米。

所以最重要的两条测试是：
  · **错位会毁掉结果** —— 不配准时体积必须错得离谱（证明问题真实存在）
  · **配准能救回来**   —— 配准后必须回到真值附近
两条缺一不可：只证明后者，说明不了这套复杂度有没有必要。
"""
from __future__ import annotations

import os
import sys
import unittest

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "cli")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from volume_calc import (columnar_volume, height_map,      # noqa: E402
                         register, L_TOL, W_TOL)

G = -5.23
CELL = 0.10


def bed(x0, x1, y0, y1, z, n=60000, seed=0):
    """一块水平面（空车厢底，或平顶的货面）。"""
    rng = np.random.default_rng(seed)
    return np.column_stack([rng.uniform(x0, x1, n),
                            rng.uniform(y0, y1, n),
                            np.full(n, G + z) + rng.normal(0, 0.005, n)])


class TestRegister(unittest.TestCase):

    def test_pure_longitudinal_shift(self):
        """空车靠后 0.8 m，配准应当算出 dy=+0.8。"""
        dx, dy, warn = register((-1.3, 1.3, -6.0, 5.0), (-1.3, 1.3, -5.2, 5.8))
        self.assertAlmostEqual(dx, 0.0, places=6)
        self.assertAlmostEqual(dy, 0.8, places=6)
        self.assertEqual(warn, [])

    def test_lateral_shift_uses_centre(self):
        dx, _dy, _w = register((-1.3, 1.3, -6, 5), (-1.1, 1.5, -6, 5))
        self.assertAlmostEqual(dx, 0.2, places=6)

    def test_aligns_on_the_rear_edge_not_the_front(self):
        """★ 前端掠射角大、覆盖率只有 43~71%，y1 不可信，必须用 y0。

        这里空车与载货的 y0 一致、y1 差 2 m（模拟前端分段不稳），
        正确的 dy 是 0；若拿 y1 对齐会得到 -2.0。
        """
        _dx, dy, _w = register((-1.3, 1.3, -5.0, 7.0), (-1.3, 1.3, -5.0, 5.0))
        self.assertAlmostEqual(dy, 0.0, places=6)

    def test_width_mismatch_warns(self):
        """宽度对不上 = 不是同一辆车。现场至少有两种车型。"""
        _dx, _dy, warn = register((-1.3, 1.3, -6, 5), (-1.0, 1.0, -6, 5))
        self.assertTrue(any("宽度差" in w for w in warn))

    def test_length_mismatch_warns(self):
        _dx, _dy, warn = register((-1.3, 1.3, -6.0, 0.25), (-1.3, 1.3, -6.0, 5.0))
        self.assertTrue(any("长度差" in w for w in warn))

    def test_small_differences_do_not_warn(self):
        """分段本身在两端有几十厘米的不确定，不能动不动就报警。"""
        _dx, _dy, warn = register((-1.30, 1.30, -6.00, 5.00),
                                  (-1.25, 1.34, -5.75, 5.25))
        self.assertEqual(warn, [])

    def test_tolerances_are_not_absurdly_loose(self):
        self.assertLessEqual(W_TOL, 0.3)
        self.assertLessEqual(L_TOL, 2.0)


class TestColumnarVolume(unittest.TestCase):

    def test_flat_load_gives_area_times_height(self):
        z_base = np.zeros((20, 30))
        z_load = np.full((20, 30), 1.5)
        v, n, holes, hmax, _c = columnar_volume(z_load, z_base, CELL)
        self.assertAlmostEqual(v, 20 * 30 * CELL * CELL * 1.5, places=6)
        self.assertEqual(n, 600)
        self.assertEqual(holes, 0)
        self.assertAlmostEqual(hmax, 1.5, places=6)

    def test_load_below_floor_counts_as_zero(self):
        """比空车还低只可能是噪声或底板缝隙，不能算成负体积。"""
        z_base = np.zeros((10, 10))
        z_load = np.full((10, 10), -0.3)
        v, _n, _h, _m, _c = columnar_volume(z_load, z_base, CELL)
        self.assertEqual(v, 0.0)

    def test_missing_cells_are_counted_and_excluded(self):
        z_base = np.zeros((10, 10))
        z_load = np.full((10, 10), 1.0)
        z_load[0, :] = np.nan
        v, _n, holes, _m, _c = columnar_volume(z_load, z_base, CELL)
        self.assertEqual(holes, 10)
        self.assertAlmostEqual(v, 90 * CELL * CELL, places=6)

    def test_fill_holes_gives_a_larger_number(self):
        """补洞是上界、只算有效格是下界，真值在中间。"""
        z_base = np.zeros((10, 10))
        z_load = np.full((10, 10), 1.0)
        z_load[0, :] = np.nan
        lo, _n, _h, _m, _c = columnar_volume(z_load, z_base, CELL, fill_holes=False)
        hi, _n2, _h2, _m2, _c2 = columnar_volume(z_load, z_base, CELL, fill_holes=True)
        self.assertGreater(hi, lo)


class TestHeightMap(unittest.TestCase):

    def test_recovers_a_flat_surface(self):
        p = bed(-1.0, 1.0, -2.0, 2.0, 1.20, n=40000)
        Z = height_map(p, -1.0, -2.0, 20, 40, CELL, G)
        self.assertTrue(np.isfinite(Z).all())
        self.assertAlmostEqual(float(np.nanmedian(Z)), 1.20, places=2)

    def test_sparse_cells_are_left_empty(self):
        """点太少的格子必须留空，不能拿一两个点的噪声当高度。"""
        p = bed(-1.0, 1.0, -2.0, 2.0, 1.2, n=50)
        Z = height_map(p, -1.0, -2.0, 20, 40, CELL, G)
        self.assertLess(np.isfinite(Z).mean(), 0.2)

    def test_uses_median_not_max(self):
        """货面有毛刺，取最高点会被单点噪声带偏。"""
        p = bed(-0.5, 0.5, -0.5, 0.5, 1.0, n=20000)
        p = np.vstack([p, np.array([[0.0, 0.0, G + 4.0]])])
        Z = height_map(p, -0.5, -0.5, 10, 10, CELL, G)
        self.assertLess(float(np.nanmax(Z)), 1.5)


class TestEndToEnd(unittest.TestCase):
    """★ 核心：错位会毁结果，配准能救回来。"""

    W, L, H = 2.6, 10.0, 1.20

    def _pair(self, shift_y):
        """造一对云：空车厢底 + 平顶货面，空车整体平移 shift_y。"""
        empty = bed(-self.W / 2, self.W / 2,
                    -self.L / 2 + shift_y, self.L / 2 + shift_y, 1.00,
                    n=120000, seed=1)
        loaded = bed(-self.W / 2, self.W / 2, -self.L / 2, self.L / 2,
                     1.00 + self.H, n=120000, seed=2)
        return empty, loaded

    def _volume(self, empty, loaded, dx, dy):
        x0, y0 = -self.W / 2, -self.L / 2
        nx = int(np.ceil(self.W / CELL))
        ny = int(np.ceil(self.L / CELL))
        e = empty.copy()
        e[:, 0] += dx
        e[:, 1] += dy
        zb = height_map(e, x0, y0, nx, ny, CELL, G)
        zl = height_map(loaded, x0, y0, nx, ny, CELL, G)
        return columnar_volume(zl, zb, CELL)[0]

    def test_aligned_case_recovers_truth(self):
        truth = self.W * self.L * self.H
        empty, loaded = self._pair(0.0)
        v = self._volume(empty, loaded, 0.0, 0.0)
        self.assertAlmostEqual(v / truth, 1.0, delta=0.03)

    def test_misalignment_wrecks_the_result(self):
        """★ 不配准时必须错得离谱 —— 否则这整套复杂度就没必要。

        空车靠后 1.5 m：重叠区只剩 8.5/10，两端各有一块只有一朵云有数据。
        """
        truth = self.W * self.L * self.H
        empty, loaded = self._pair(1.5)
        v = self._volume(empty, loaded, 0.0, 0.0)
        self.assertLess(v / truth, 0.90,
                        "错位 1.5 m 居然没影响结果 —— 检查测试是否真的造出了错位")

    def test_registration_restores_it(self):
        """★ 配准后必须回到真值附近。"""
        truth = self.W * self.L * self.H
        empty, loaded = self._pair(1.5)
        ebox = (-self.W / 2, self.W / 2, -self.L / 2 + 1.5, self.L / 2 + 1.5)
        lbox = (-self.W / 2, self.W / 2, -self.L / 2, self.L / 2)
        dx, dy, warn = register(ebox, lbox)
        self.assertEqual(warn, [])
        self.assertAlmostEqual(dy, -1.5, places=6)
        v = self._volume(empty, loaded, dx, dy)
        self.assertAlmostEqual(v / truth, 1.0, delta=0.03)



class TestPhysicalCap(unittest.TestCase):
    """★ 物理上限：载货侧个别格子会打到车厢以外的东西。

    2026-09-15 实测 33 个点落在车厢尾部 y=-4.8 一带、高度 4.98 m，
    比厢壁顶还高 1.7 m。不截断就会虚增体积。
    """

    def test_cap_limits_the_height(self):
        z_base = np.zeros((10, 10))
        z_load = np.full((10, 10), 1.0)
        z_load[0, 0] = 5.0
        v, _n, _h, hmax, clipped = columnar_volume(z_load, z_base, CELL, h_max=2.0)
        self.assertEqual(clipped, 1)
        self.assertAlmostEqual(hmax, 2.0, places=6)
        self.assertAlmostEqual(v, (99 * 1.0 + 2.0) * CELL * CELL, places=6)

    def test_no_cap_when_none(self):
        z_base = np.zeros((4, 4))
        z_load = np.full((4, 4), 9.0)
        _v, _n, _h, hmax, clipped = columnar_volume(z_load, z_base, CELL, h_max=None)
        self.assertEqual(clipped, 0)
        self.assertAlmostEqual(hmax, 9.0, places=6)


class TestFloorPlane(unittest.TestCase):
    """厢底是**斜的**，必须用平面拟合，不能用固定高度带。"""

    def _tilted(self, slope=0.03, z0=1.50, n=60000):
        rng = np.random.default_rng(3)
        x = rng.uniform(-1.3, 1.3, n)
        y = rng.uniform(-5.0, 5.0, n)
        z = G + z0 + slope * y + rng.normal(0, 0.01, n)
        return np.column_stack([x, y, z])

    def test_recovers_the_tilt(self):
        from volume_calc import floor_plane
        pts = self._tilted(slope=0.03)
        a, b, c, n, _z0 = floor_plane(pts, G)
        self.assertAlmostEqual(b, 0.03, delta=0.005)
        self.assertAlmostEqual(a, 0.0, delta=0.01)
        self.assertAlmostEqual(c, 1.50, delta=0.02)
        self.assertGreater(n, 1000)

    def test_walls_do_not_drag_the_plane_up(self):
        """★ 两侧厢壁比底板高 1.7 m，拟合必须把它们剔掉。"""
        from volume_calc import floor_plane
        floor = self._tilted(slope=0.0, n=60000)
        rng = np.random.default_rng(4)
        wall = np.column_stack([
            rng.choice([-1.3, 1.3], 20000) + rng.normal(0, 0.01, 20000),
            rng.uniform(-5, 5, 20000),
            G + rng.uniform(1.6, 3.3, 20000)])
        _a, _b, c, _n, _z = floor_plane(np.vstack([floor, wall]), G)
        self.assertAlmostEqual(c, 1.50, delta=0.10)

    def test_too_few_points_degrades_gracefully(self):
        from volume_calc import floor_plane
        a, b, c, n, _z = floor_plane(np.zeros((10, 3)), G)
        self.assertEqual((a, b), (0.0, 0.0))
        self.assertEqual(n, 10)


class TestInterpFill(unittest.TestCase):
    """★ 空洞必须按**邻居插值**补，不能当成没有货。

    车厢前端的空洞是扫描线之间的缝（实测一处 0.84 m，连着 8 个 0.10 m 格子）。
    缝两边都有货，缝里当然也有 —— "不算"等于假设缝里是空的，这是明确错的：
    2026-09-15 实测把格子从 0.10 放大到 0.30，下界从 51.0 一路涨到 55.0，
    而上界纹丝不动。
    """

    def test_bridges_a_gap_linearly(self):
        """缝两边是 1.0 和 2.0，缝里应当落在两者之间。"""
        from volume_calc import interp_fill
        Z = np.full((5, 11), np.nan)
        Z[:, :3] = 1.0
        Z[:, 8:] = 2.0
        out = interp_fill(Z)
        self.assertTrue(np.isfinite(out).all())
        mid = out[2, 5]
        self.assertGreater(mid, 1.0)
        self.assertLess(mid, 2.0)

    def test_leaves_measured_cells_untouched(self):
        from volume_calc import interp_fill
        Z = np.arange(20, dtype=float).reshape(4, 5)
        out = interp_fill(Z.copy())
        np.testing.assert_allclose(out, Z)

    def test_all_nan_stays_nan(self):
        from volume_calc import interp_fill
        out = interp_fill(np.full((4, 4), np.nan))
        self.assertTrue(np.isnan(out).all())

    def test_makes_volume_nearly_cell_size_independent(self):
        """★ 这是这个函数存在的全部理由。

        造一个"有缝"的平顶货面：真值已知，缝按不同格子会掉不同数量的格。
        插值补之后，不同格子尺寸算出的体积应当彼此接近。
        """
        from volume_calc import interp_fill
        W, L, H = 2.4, 9.6, 1.5
        vols, truths = [], []
        for c in (0.10, 0.20, 0.30):
            # ★ 真值按**实际网格**算，不按 W*L*H —— int(2.4/0.10) 因浮点得 23
            #   而不是 24，网格会少覆盖 5%。那是取整问题，与 interp_fill 无关，
            #   混进来会让这条测试查的东西不纯。
            nx, ny = int(round(W / c)), int(round(L / c))
            truths.append(nx * ny * c * c * H)
            zl = np.full((nx, ny), H)
            # 每隔几列挖一条缝，模拟扫描线间隙
            step = max(2, int(round(0.6 / c)))
            # 从第 1 列起挖，别动首尾 —— 边缘只能单边插值，
            # 那是测试构造的边界效应，不是 interp_fill 的问题
            zl[:, 1:-1:step] = np.nan
            v = columnar_volume(interp_fill(zl), np.zeros((nx, ny)), c)[0]
            vols.append(v)
        for v, t in zip(vols, truths):
            self.assertAlmostEqual(v / t, 1.0, delta=0.02,
                                   msg="插值补没有还原出缝里的货")
        self.assertLess((max(vols) - min(vols)) / float(np.mean(vols)), 0.05,
                        "插值补之后体积仍随格子尺寸大幅变化")

if __name__ == "__main__":
    unittest.main()
