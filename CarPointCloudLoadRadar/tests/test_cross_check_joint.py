"""交叉校核的联合行列拟合：用已知体积检验不均匀缺测，而非只比两种算法。"""
import unittest

import numpy as np

from app import detect as D
from tests.test_empty_audit import _CarriageScene


class TestCrossCheckJointCloud(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        scene = _CarriageScene()
        cls.baseline = scene._baseline()
        cls.truth = 0.8 * scene.W * scene.L
        rng = np.random.default_rng(135)
        x = rng.uniform(-scene.W / 2, scene.W / 2, 180000)
        y = rng.uniform(-scene.L / 2, scene.L / 2, 180000)
        h = 0.2 + 0.6 * (x >= 0) + 0.6 * (y >= 0)
        walls = scene._walls(np.random.default_rng(44), 0.0, 1.42)
        cls.loads = {}
        for name, keep in (("full", np.ones(len(x), bool)),
                           ("missing_high", ~((x >= 0) & (y >= 0))),
                           ("missing_low", ~((x < 0) & (y < 0)))):
            surface = np.column_stack((x[keep], y[keep], scene._z(y[keep]) + h[keep]))
            cls.loads[name] = D.LoadedResult(cloud=np.vstack([surface, *walls]).astype(np.float32), ok=True)

    def _check_truth(self, name):
        result = D.cross_check_volume(self.baseline, self.loads[name])
        self.assertEqual("done", result.status, result.reason)
        self.assertLess(abs(result.volume_m3 / self.truth - 1.0), 0.01,
                        "%s: %.6f / 真值 %.6f；%s" % (name, result.volume_m3, self.truth, result.reason))

    def test_complete_additive_cloud(self):
        self._check_truth("full")

    def test_missing_high_quadrant_keeps_the_true_volume(self):
        self._check_truth("missing_high")

    def test_missing_low_quadrant_keeps_the_true_volume(self):
        self._check_truth("missing_low")

    def test_diagnostics_do_not_change_the_measurement_schema(self):
        result = D.cross_check_volume(self.baseline, self.loads["full"])
        before = set(D.VolumeResult().to_dict(raw=True))
        volume = D.VolumeResult(cross_check=result)
        self.assertEqual(before, set(volume.to_dict(raw=True)))
        self.assertTrue(result.diagnostics["boundary_area_weighted"])
        self.assertEqual([0.2, 99.8], result.diagnostics["bed_endpoint_percentiles"])
        self.assertEqual(0.1, result.diagnostics["row_cell_m"])


class TestCrossCheckJointSupport(unittest.TestCase):
    """图、模型与留出各有独立反例；缺测不等于断连，拟合精确也不等于可验证。"""

    def _case(self, mask=None, heights=None):
        row, col = np.indices((20, 20))
        if heights is None:
            heights = 0.2 + 0.6 * (row >= 10) + 0.6 * (col >= 10)
        if mask is None:
            mask = np.ones(row.shape, bool)
        return D._cross_check_joint_model(row[mask], col[mask], heights[mask],
                                          np.full(20, 0.5), np.full(20, 0.12))

    def test_additive_model_recovers_all_three_sampling_patterns(self):
        row, col = np.indices((20, 20))
        for mask in (np.ones(row.shape, bool), ~((row >= 10) & (col >= 10)),
                     ~((row < 10) & (col < 10))):
            with self.subTest(observations=int(mask.sum())):
                result = self._case(mask)
                self.assertEqual("done", result["status"], result["reason"])
                self.assertAlmostEqual(19.2, result["mean_depth_m"] * 24, places=10)
                self.assertEqual(1, result["diagnostics"]["components"])

    def test_disconnected_quadrants_do_not_pass_via_pseudoinverse(self):
        row, col = np.indices((20, 20))
        result = self._case((row < 10) == (col < 10))
        self.assertEqual("unavailable", result["status"])
        self.assertIn("分成 2 个分量", result["reason"])
        self.assertEqual(0.0, result["diagnostics"]["unsupported_union_fraction"])

    def test_single_bridge_is_not_treated_as_reliable_connection(self):
        row, col = np.indices((20, 20))
        mask = (row < 10) == (col < 10)
        mask[2, 12] = True
        result = self._case(mask)
        self.assertEqual("unavailable", result["status"])
        self.assertIn("连接过薄", result["reason"])
        self.assertEqual(1, result["diagnostics"]["bridges"])

    def test_two_thin_bridges_are_detected_by_spectral_gap(self):
        row, col = np.indices((20, 20))
        mask = (row < 10) == (col < 10)
        mask[2, 12] = mask[3, 13] = True
        result = self._case(mask)
        self.assertEqual("unavailable", result["status"])
        self.assertEqual(0, result["diagnostics"]["bridges"])
        self.assertLess(result["diagnostics"]["spectral_gap"], D.CROSS_CHECK_MIN_GRAPH_GAP)

    def test_large_unobserved_row_area_is_not_silently_extrapolated(self):
        row, _col = np.indices((20, 20))
        result = self._case(row < 10)
        self.assertEqual("unavailable", result["status"])
        self.assertIn("超过 15%", result["reason"])
        self.assertAlmostEqual(0.5, result["diagnostics"]["unsupported_union_fraction"])

    def test_small_row_column_gap_uses_area_union_without_double_counting(self):
        row, col = np.indices((20, 20))
        height = 0.8 + (row - 9.5) * 0.02 + (col - 9.5) * 0.01
        result = self._case((row != 4) & (col != 7), height)
        self.assertEqual("done", result["status"], result["reason"])
        self.assertAlmostEqual(0.0975, result["diagnostics"]["unsupported_union_fraction"])
        self.assertAlmostEqual(19.2, result["mean_depth_m"] * 24, places=10)

    def test_non_additive_ridge_is_rejected_for_model_mismatch(self):
        row, col = np.indices((20, 20))
        height = 0.4 + 0.8 * ((row >= 10) & (col >= 10))
        result = self._case(heights=height)
        self.assertEqual("unavailable", result["status"])
        self.assertIn("模型残差", result["reason"])

    def test_local_interaction_can_fail_block_holdout_despite_small_global_residual(self):
        height = np.ones((20, 20))
        height[:3, :3] += 0.16
        result = self._case(heights=height)
        self.assertEqual("unavailable", result["status"])
        self.assertLess(result["diagnostics"]["model_relative_rmse"], D.CROSS_CHECK_RESIDUAL_LIMIT)
        self.assertIn("留出检查", result["reason"])

    def test_connected_cycle_with_no_valid_holdout_is_explicitly_unavailable(self):
        row = np.repeat(np.arange(12), 2)
        col = np.column_stack((np.arange(12), (np.arange(12) + 1) % 12)).ravel()
        result = D._cross_check_joint_model(np.repeat(row, 3), np.repeat(col, 3), np.ones(72),
                                            np.full(12, 0.5), np.full(12, 0.2))
        self.assertEqual("unavailable", result["status"])
        self.assertIn("无法检查拟合稳定性", result["reason"])

    def test_fixed_seed_keeps_validation_reproducible(self):
        a, b = self._case(), self._case()
        self.assertEqual(a, b)
        self.assertEqual(3, len(a["diagnostics"]["dispersed_holdouts"]))
        self.assertGreaterEqual(len(a["diagnostics"]["block_holdouts"]), 2)

    def test_fine_row_model_rejects_excessive_parameter_count_before_matrix_allocation(self):
        row, col = np.indices((400, 3))
        result = D._cross_check_joint_model(row.ravel(), col.ravel(), np.ones(row.size),
                                            np.full(400, 0.1), np.full(3, 0.1))
        self.assertEqual("unavailable", result["status"])
        self.assertIn("超过校核预算", result["reason"])


class TestCrossCheckNonAdditiveCloud(unittest.TestCase):
    """已知解析体积的脊形点云：横向脊高随纵向改变，覆盖三种真实扫描缺失形态。"""

    @classmethod
    def setUpClass(cls):
        scene = _CarriageScene()
        cls.baseline = scene._baseline()
        rng = np.random.default_rng(97531)
        x = rng.uniform(-scene.W / 2, scene.W / 2, 200000)
        y = rng.uniform(-scene.L / 2, scene.L / 2, 200000)
        h = 0.4 + 0.6 * (1 - (2 * x / scene.W) ** 2) * (1 - 0.7 * (2 * y / scene.L) ** 2)
        # 两个平方项在完整横/纵区间的均值都是1/3，直接解析积分，不借用被测拟合结果。
        cls.truth = scene.W * scene.L * (0.4 + 0.6 * (2 / 3) * (1 - 0.7 / 3))
        walls = scene._walls(np.random.default_rng(2468), 0.0, 1.02)
        masks = {
            "sparse_ends": (np.abs(y) < scene.L / 2 - 1.5) | (rng.random(len(y)) < 0.25),
            "missing_side": x >= -0.1,
            "missing_corner": ~((x < -scene.W / 4) & (y > scene.L / 4)),
        }
        cls.loads = {name: D.LoadedResult(cloud=np.vstack([
            np.column_stack((x[keep], y[keep], scene._z(y[keep]) + h[keep])), *walls
        ]).astype(np.float32), ok=True) for name, keep in masks.items()}

    def _check(self, name):
        result = D.cross_check_volume(self.baseline, self.loads[name])
        if result.status == "done":
            self.assertLessEqual(abs(result.volume_m3 / self.truth - 1.0), 0.03,
                                 "%s: %.6f / 解析真值 %.6f；%s" % (
                                     name, result.volume_m3, self.truth, result.reason))
        else:
            self.assertEqual("unavailable", result.status)
            self.assertTrue(any(reason in result.reason for reason in (
                "模型残差", "留出检查", "未获足够支持", "连接过薄", "缺测区不可辨识", "无法检查拟合稳定性")),
                            result.reason)
            self.assertTrue(np.isnan(result.volume_m3), "拒绝的模型不能留下可当有效校核的体积")

    def test_sparse_ends_have_known_truth_or_an_explicit_model_rejection(self):
        self._check("sparse_ends")

    def test_missing_side_is_not_hidden_by_interpolating_unobserved_columns(self):
        self._check("missing_side")

    def test_missing_corner_does_not_hide_non_additive_ridge_error(self):
        self._check("missing_corner")


if __name__ == "__main__":
    unittest.main()
