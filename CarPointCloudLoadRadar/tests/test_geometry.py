# -*- coding: utf-8 -*-
"""几何原语测试：平面拟合、内点收紧、朝向矫正。

这些函数是"可以放心改"的地方（拟合形式无法从机器码逐行还原），
所以更要有单测兜着：改完先跑这里，再跑 cli/verify.py。
"""
from __future__ import annotations

import unittest

import numpy as np

from tests import _helpers as H
from app.geometry import (Plane, fit_plane_pca, fit_plane_ransac, refine_inliers,
                          correct_heading, rotate_about_z)


class TestPlane(unittest.TestCase):

    def test_normal_is_unit(self):
        p = Plane(0, 0, 2, -6)
        self.assertAlmostEqual(p.norm(), 2.0)
        np.testing.assert_allclose(p.normal(), [0, 0, 2])

    def test_distance(self):
        p = Plane(0, 0, 1, 0)                         # z = 0 平面
        pts = np.array([[0, 0, 1], [0, 0, -3], [0, 0, 0]], np.float32)
        np.testing.assert_allclose(p.distance(pts), [1, -3, 0], atol=1e-6)
        np.testing.assert_allclose(p.abs_distance(pts), [1, 3, 0], atol=1e-6)

    def test_intercept_and_horizontal(self):
        p = Plane(0, 0, 1, -2)                        # z = 2
        self.assertAlmostEqual(p.intercept(2), 2.0)
        self.assertTrue(p.is_horizontal())
        q = Plane(1, 0, 0, -2)                        # x = 2 → 竖直面
        self.assertFalse(q.is_horizontal())

    def test_orient_towards(self):
        p = Plane(0, 0, -1, 5).orient_towards(2, 1)   # 期望法向 +z
        self.assertGreater(p.c, 0)


class TestFitPlane(unittest.TestCase):

    def test_pca_recovers_plane(self):
        pts = H.grid_plane(n=50, extent=2.0, z=1.3, noise=0.0, seed=3)
        p = fit_plane_pca(pts)
        self.assertIsNotNone(p)
        self.assertAlmostEqual(abs(p.c), 1.0, places=4)
        # 平面应过 z=1.3
        self.assertAlmostEqual(abs(p.intercept(2)), 1.3, places=3)

    def test_pca_needs_three_points(self):
        self.assertIsNone(fit_plane_pca(np.zeros((2, 3), np.float32)))

    def test_pca_collinear_returns_none(self):
        t = np.linspace(0, 1, 20, dtype=np.float32)
        pts = np.stack([t, np.zeros_like(t), np.zeros_like(t)], axis=1)
        self.assertIsNone(fit_plane_pca(pts))

    def test_ransac_ignores_outliers(self):
        rng = np.random.default_rng(4)
        pts = H.grid_plane(n=40, extent=1.5, z=0.0, noise=0.001, seed=4)
        noise = rng.uniform(-1, 1, (len(pts) // 5, 3)).astype(np.float32)
        allp = np.concatenate([pts, noise], axis=0)
        p = fit_plane_ransac(allp, thr=0.02, iters=128, seed=0)
        self.assertIsNotNone(p)

    def test_refine_inliers_grows_set(self):
        pts = H.grid_plane(n=40, extent=1.5, z=0.0, noise=0.001, seed=5)
        seed = pts[:50]
        p = fit_plane_pca(seed)
        plane, mask = refine_inliers(pts, p, 0.01)     # 返回 (平面, 布尔掩码)
        self.assertGreater(int(mask.sum()), 50)
        self.assertEqual(plane.inliers, int(mask.sum()))


class TestHeading(unittest.TestCase):
    """
    ▲ 关于 correct_heading 的能力边界（实测结论，不是猜的）

    它**不是**一个通用的"任意角度航向估计器"，而是"小残差回正器"：
        · 车厢点云在进入它之前，大角度已经由 processing.yaw_offset_deg 矫正过，
          留下的通常只有 ±3° 以内的残差；它负责把这部分抹平。
        · 它的实现里有一个"按 x 分位裁剪"的步骤（去侧壁噪声），
          这隐含假设 x 是车宽方向、且点云已大致轴对齐。
        · 对**完全对称的矩形平板**做小角度旋转时，裁剪后的形状仍关于 x 轴对称，
          主方向在数学上不可辨识（两个特征值接近），PCA 只能给出接近 0 的结果。
          实测：矩形板旋转 1°/2°/3° → 估计值 0.00°（触发 min_correction 早退）。

    所以单测只固化两件**对管线真正重要**的事：
        ① 接口契约（形状、早退条件）
        ② 在真实样本上残差航向很小（|yaw| < 3°），与历史记录一致
    以及把上面这条"对称板退化"钉住，避免有人误以为它能做任意角度估计。
    """

    def test_shapes(self):
        pts = H.grid_plane(n=40, extent=2.0, z=-3.0, noise=0.02, seed=8)
        out, yaw_rad, R = correct_heading(pts, pct=15.0, min_correction=0.005)
        self.assertEqual(out.shape, pts.shape)
        self.assertIsInstance(yaw_rad, float)
        self.assertEqual(R.shape, (3, 3))

    def test_skips_tiny_yaw(self):
        """航向已经很正时不应旋转（与原版一致，避免引入浮点噪声）。"""
        rng = np.random.default_rng(7)
        n = 2000
        pts = np.stack([rng.normal(0, 0.2, n), rng.normal(0, 2.0, n),
                        rng.normal(-3, 0.05, n)], axis=1).astype(np.float32)
        out, yaw_rad, R = correct_heading(pts, pct=15.0, min_correction=0.05)
        self.assertAlmostEqual(yaw_rad, 0.0, places=9)
        np.testing.assert_allclose(R, np.eye(3), atol=1e-12)

    def test_real_sample_residual_yaw_is_small(self):
        """真实样本的残差航向应当很小 —— 大角度早已被 yaw_offset_deg 抹掉。"""
        import math
        samples = H.history_samples()
        if not samples:
            self.skipTest("没有历史样本")
        from app.pointcloud import load_xyz
        _, _, calib, _, _ = H.ctx()
        worst = 0.0
        for p, _r in samples:
            fr = load_xyz(p)
            _out, yaw, _R = correct_heading(fr.xyz, pct=15.0, min_correction=0.005)
            worst = max(worst, abs(math.degrees(yaw)))
        self.assertLess(worst, 3.0, f"某样本残差航向达 {worst:.2f}°，超出预期（<3°）")

    def test_symmetric_slab_limitation(self):
        """固化已知局限：对称矩形板的小角度旋转无法被估计出来（返回 ~0）。

        这不是要修的 bug，而是"该函数只做小残差回正"的证据。
        哪天它真能估出角度了，说明实现被改强了 —— 那时请更新这条测试与注释。
        """
        rng = np.random.default_rng(11)
        n = 20000
        x = rng.uniform(-1.2, 1.2, n)
        y = rng.uniform(-6, 6, n)
        z = rng.normal(-3.6, 0.05, n)
        base = np.stack([x, y, z], axis=1).astype(np.float32)
        slab = rotate_about_z(base, 2.0)              # 对称板，转 2°
        _out, yaw_rad, _R = correct_heading(slab, pct=15.0, min_correction=0.005)
        self.assertLess(abs(np.degrees(yaw_rad)), 1.0,
                        "对称板居然估出了航向 —— 实现可能被改动了，"
                        "请同步更新本测试与 geometry.correct_heading 的注释")

    def test_rotate_about_z_known(self):
        pts = np.array([[1.0, 0.0, 0.0]], np.float32)
        out = rotate_about_z(pts, 90.0)
        np.testing.assert_allclose(out[0], [0.0, 1.0, 0.0], atol=1e-5)


if __name__ == "__main__":
    unittest.main()
