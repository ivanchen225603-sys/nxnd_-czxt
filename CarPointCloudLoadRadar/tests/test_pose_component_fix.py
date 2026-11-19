# -*- coding: utf-8 -*-
"""姿态逐分量显著性和进场零残差反例（2026-10-03）。"""
import math
import unittest

import numpy as np

from app import detect as D
from . import test_empty_audit as scenes


class TestPoseComponents(unittest.TestCase):
    @staticmethod
    def _profile(heave=0.0, roll=0.0, pitch=0.0, twist=0.0):
        """直接给两侧实测壁顶段：噪声与截距、斜率正交，已知真值不靠碰巧的随机偏差。"""
        y = np.arange(-4.75, 5.0, 0.5)
        A = np.column_stack([np.ones(len(y)), y])
        noise = np.resize(np.array([0.01, -0.013, 0.008, -0.005]), len(y))
        noise -= A @ np.linalg.lstsq(A, noise, rcond=None)[0]
        out = dict(ym=y, y_mid=0.0)
        for side, x, noise_factor in (("L", -1.25, 1.0), ("R", 1.25, 1.2)):
            slope = pitch + (-0.5 if side == "L" else 0.5) * twist
            baseline = np.full(len(y), -2.0)
            loaded = baseline + heave + roll * x + slope * y + noise_factor * noise
            out[side] = (baseline, loaded, np.full(len(y), x), 0, np.full(len(y), x))
        cloud = np.zeros((300, 3))
        base = D.EmptyBaseline(floor_z=0.0, cloud=cloud, n=len(cloud))
        return base, D.LoadedResult(cloud=cloud), out

    def test_each_component_can_be_significant_by_itself(self):
        """原逻辑只检查中心升降，纯横滚或俯仰会被整项误拒绝。"""
        for values, expected in (((0.04, 0.0, 0.0), (True, False, False)),
                                 ((0.0, 0.03, 0.0), (False, True, False)),
                                 ((0.0, 0.0, 0.02), (False, False, True))):
            with self.subTest(values=values):
                base, loaded, profiles = self._profile(*values)
                pose = D.estimate_pose_registration(base, loaded, profiles=profiles)
                self.assertEqual("resolved", pose.status, pose.reason)
                self.assertEqual(expected, pose.adopted_components)
                x = np.array([-1.0, 0.0, 1.0])
                y = np.array([2.0, 0.0, -2.0])
                np.testing.assert_allclose(pose.at(x, y), values[0] + values[1] * x + values[2] * y,
                                           atol=1e-12)

    def test_small_components_are_zeroed_and_reference_has_no_twist(self):
        """只保留显著升降，小横滚/小俯仰置零；两壁斜率差不应混成参考面扭转。"""
        base, loaded, profiles = self._profile(0.04, 0.00001, 0.00001, twist=0.008)
        pose = D.estimate_pose_registration(base, loaded, profiles=profiles)
        self.assertEqual("resolved", pose.status, pose.reason)
        np.testing.assert_allclose(pose.at(np.array([-1., 1.]), np.array([-4., 4.])), 0.04, atol=1e-12)
        self.assertEqual((True, False, False), pose.adopted_components)
        self.assertEqual(pose.left[1], pose.right[1])

    def test_all_insignificant_retains_conditional_error_without_applying_it(self):
        base, loaded, profiles = self._profile()
        pose = D.estimate_pose_registration(base, loaded, profiles=profiles)
        self.assertNotEqual("resolved", pose.status)
        self.assertTrue(pose.estimated)
        self.assertGreater(pose.sigma_m, 0.0)
        self.assertEqual((False, False, False), pose.adopted_components)
        np.testing.assert_array_equal(pose.at(np.array([-1., 1.]), np.array([-4., 4.])), 0.0)
        self.assertEqual((3, 3), np.asarray(pose.conditional_covariance).shape)

    def test_line_covariance_uses_observed_span_and_off_diagonal(self):
        """缺测后段中点不为零：截距和斜率相关，不能把残差标准差直接当斜率标准误。"""
        y = np.arange(0.0, 10.0, 0.5)
        d = 0.04 + 0.007 * y + np.resize([0.003, -0.004, 0.002, -0.001], len(y))
        line = D._robust_wall_line(y, d)
        A = np.column_stack([np.ones(len(y)), y])
        coef = np.linalg.lstsq(A, d, rcond=None)[0]
        s2 = float(np.sum((d - A @ coef) ** 2) / (len(y) - 2))
        cov = s2 * np.linalg.inv(A.T @ A)
        self.assertAlmostEqual(s2 / np.sum((y - y.mean()) ** 2), line["var_b"], places=16)
        self.assertAlmostEqual(cov[0, 1], line["cov_ab"], places=16)
        self.assertNotEqual(0.0, line["cov_ab"])

    def test_pose_retains_the_correlations_from_asymmetric_segment_support(self):
        """保留段偏向车厢一端时，高差/俯仰以及横滚/俯仰的条件误差相关，不能全当零。"""
        base, loaded, profiles = self._profile(0.04, 0.02, 0.01)
        profiles["ym"] = profiles["ym"] + 1.5
        pose = D.estimate_pose_registration(base, loaded, profiles=profiles)
        left = D._robust_wall_line(profiles["ym"], profiles["L"][1] - profiles["L"][0])
        right = D._robust_wall_line(profiles["ym"], profiles["R"][1] - profiles["R"][0])
        cov = np.asarray(pose.conditional_covariance)
        self.assertAlmostEqual((left["cov_ab"] + right["cov_ab"]) / 4, cov[0, 2], places=16)
        self.assertAlmostEqual((right["cov_ab"] - left["cov_ab"]) / (2 * 2.5), cov[1, 2], places=16)
        self.assertNotEqual(0.0, cov[0, 2])
        self.assertNotEqual(0.0, cov[1, 2])
        np.testing.assert_allclose(cov, cov.T, atol=1e-16)

    def test_hard_limits_and_twist_still_reject_the_whole_fit(self):
        for params in (dict(heave=0.20, roll=0.03), dict(heave=0.04, roll=0.20),
                       dict(heave=0.04, pitch=0.15), dict(heave=0.04, twist=0.03)):
            with self.subTest(params=params):
                base, loaded, profiles = self._profile(**params)
                pose = D.estimate_pose_registration(base, loaded, profiles=profiles)
                self.assertEqual("unresolved", pose.status)

    def test_pure_pitch_thin_coal_matches_known_pose(self):
        """原逻辑识别3度俯仰仍拒绝，薄煤一端截为零后体积比正确姿态高18.5%。"""
        scene = scenes.TestVolumePoseRegistration()
        scene.H = 0.10
        base, loaded = scene._baseline(), scene._shaped(dz=0.0, pitch=3.0)
        slope = math.tan(math.radians(3.0))
        known = D.PoseDecision("resolved", "已知合成俯仰", 0., 0., (0., slope), (0., slope), -1.25, 1.25)
        kw = dict(dx=0., dy=0., yaw=0., fill_check=False)
        auto = D.compute_volume(base, loaded, **kw)
        expected = D.compute_volume(base, loaded, pose=known, **kw)
        self.assertLess(abs(auto.volume_m3 / expected.volume_m3 - 1.0), 0.01, auto.text)

    @staticmethod
    def _analytic_thin_coal(heave, roll, pitch):
        """解析棱柱：2.4*10*0.04=0.96 m^3，坐标剪切的行列式为1，体积不变。

        每个5cm格心放一个底板/煤面点，无随机噪声。侧壁用无厚度实测面，显式给出
        解析内腔边界，避免把壁厚特征选择和自动定界误差混入本组姿态门槛测试。
        roll/pitch是坡度；z'=z+heave+roll*x+pitch*y，煤厚始终是0.04m。
        """
        width, length, depth = 2.4, 10.0, 0.04
        x = np.arange(-width / 2 + 0.025, width / 2, 0.05)
        y = np.arange(-length / 2 + 0.025, length / 2, 0.05)
        xx, yy = np.meshgrid(x, y)
        floor = np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, -3.7)])
        parts = [floor]
        for side in (-width / 2, width / 2):
            wy, wz = np.meshgrid(y, np.linspace(-3.7, -2.1, 33))
            parts.append(np.column_stack([np.full(wy.size, side), wy.ravel(), wz.ravel()]))
        cloud = np.concatenate(parts)
        base = D.EmptyBaseline(-3.7, (-width / 2, width / 2, -length / 2, length / 2, -3.7, -2.1),
                               cloud, len(cloud), "analytic", bed_plane=(0., 0., 1., 3.7))
        cargo = cloud.copy()
        cargo[:len(floor), 2] += depth
        cargo[:, 2] += heave + roll * cargo[:, 0] + pitch * cargo[:, 1]
        inner = D.InnerDomain("resolved", "已知解析矩形内腔", (-width / 2, 0.), (width / 2, 0.),
                              0., -length / 2, length / 2)
        return base, D.LoadedResult(cloud=cargo), inner, width * length * depth

    def _assert_analytic_thin_volume(self, heave, roll, pitch):
        base, loaded, inner, truth = self._analytic_thin_coal(heave, roll, pitch)
        kw = dict(dx=0., dy=0., yaw=0., inner=inner, voxel=0.05, fill_check=False)
        result = D.compute_volume(base, loaded, **kw)
        off = D.compute_volume(base, loaded, pose=False, **kw)
        self.assertTrue(result.ok, result.text)
        # 直接比解析体积，不把另一遍积分或已知姿态的同算法结果当成真值。
        self.assertAlmostEqual(truth, result.volume_m3, delta=truth * 0.001, msg=result.text)
        self.assertGreater(abs(off.volume_m3 / truth - 1.), 0.05,
                           "夹具应当暴露关闭姿态校正的薄煤偏差")

    def test_pure_roll_thin_coal_matches_analytic_volume(self):
        self._assert_analytic_thin_volume(0., 0.07, 0.)

    def test_pure_sag_thin_coal_matches_analytic_volume(self):
        self._assert_analytic_thin_volume(-0.06, 0., 0.)

    def test_pure_pitch_thin_coal_matches_analytic_volume(self):
        self._assert_analytic_thin_volume(0., 0., 0.05)

    def test_all_three_components_match_analytic_thin_coal_volume(self):
        self._assert_analytic_thin_volume(-0.025, 0.04, 0.02)

    def test_identical_cloud_no_correction_matches_pose_false(self):
        scene = scenes._EndWallScene()
        base = scene._baseline()
        loaded = D.LoadedResult(cloud=base.cloud.copy())
        pose = D.estimate_pose_registration(base, loaded)
        self.assertTrue(pose.estimated)
        self.assertEqual((False, False, False), pose.adopted_components)
        kw = dict(dx=0., dy=0., yaw=0., fill_check=False)
        auto = D.compute_volume(base, loaded, pose=pose, **kw)
        off = D.compute_volume(base, loaded, pose=False, **kw)
        self.assertEqual(off.volume_m3, auto.volume_m3)


class TestEnterZeroResidual(unittest.TestCase):
    @staticmethod
    def _cloud(flat=False):
        r = np.random.default_rng(19)
        x, y = r.uniform(-1.25, 1.25, 5000), r.uniform(-4.95, 4.95, 5000)
        z = np.full(x.shape, -3.) if flat else r.uniform(-3., -2., x.shape)
        return np.column_stack([x, y, z])

    def test_exact_textured_match_is_accepted(self):
        """同一有纹理点云最佳代价为零，其它位移为正，旧代码把尖锐度写成零而误拒绝。"""
        cloud = self._cloud()
        result = D.register_enter_frames(D.LoadedResult(cloud=cloud), [cloud.copy()])
        self.assertEqual("resolved", result.status, result.reason)
        self.assertEqual(1, result.used)
        self.assertLess(abs(result.shifts[0]), 0.06)

    def test_all_zero_costs_still_reject_an_ambiguous_flat_surface(self):
        cloud = self._cloud(flat=True)
        result = D.register_enter_frames(D.LoadedResult(cloud=cloud), [cloud.copy()])
        self.assertEqual("unresolved", result.status)
        self.assertIn("尖锐度 0.0", result.reason)


if __name__ == "__main__":
    unittest.main()
