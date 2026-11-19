# -*- coding: utf-8 -*-
"""2026-10-03：误差估计保持主口径，结构性补点与未估项必须可见。"""
from dataclasses import replace
import math
import unittest
from unittest.mock import patch

import numpy as np

from app import detect
from tests.test_empty_audit import _EndWallScene


class TestUncertaintyOptions(_EndWallScene, unittest.TestCase):
    def _case(self, statistic="median", dz=0.0):
        base, loaded = self._baseline(), self._loaded(sd=0.03)
        if dz:
            cloud = np.asarray(loaded.cloud, np.float64).copy()
            cloud[:, 2] += dz
            loaded = replace(loaded, cloud=cloud.astype(np.float32))
        options = dict(dx=0.0, dy=0.0, yaw=0, pose=False, surface_stat=statistic)
        result = detect.compute_volume(base, loaded, **options)
        return base, loaded, options, result

    def test_median_perturbation_uses_the_main_statistic(self):
        """旧实现删除 surface_stat，误差重算由中位数悄悄切回最高点。"""
        base, loaded, options, result = self._case()
        expected = max(abs(detect.compute_volume(base, loaded, **dict(options, dy=e),
                                                 fill_check=False).volume_m3 - result.volume_m3)
                       for e in (-detect.VOLUME_ERR_DY_M, detect.VOLUME_ERR_DY_M))
        uncertainty = detect.estimate_volume_uncertainty(base, loaded, result, **options)
        self.assertAlmostEqual(expected, uncertainty.dy_m3, places=9)

    def test_explicitly_disabled_pose_and_yaw_stay_disabled(self):
        """旧实现前后扰动强制 pose/yaw=None，把调用方关掉的校正又打开。"""
        base, loaded, options, result = self._case("max", dz=-0.08)
        expected = max(abs(detect.compute_volume(base, loaded, **dict(options, dy=e),
                                                 fill_check=False).volume_m3 - result.volume_m3)
                       for e in (-detect.VOLUME_ERR_DY_M, detect.VOLUME_ERR_DY_M))
        uncertainty = detect.estimate_volume_uncertainty(base, loaded, result, **options)
        self.assertAlmostEqual(expected, uncertainty.dy_m3, places=9)

    def test_every_recalculation_preserves_explicit_options(self):
        base, loaded, options, result = self._case()
        calls = []
        original = detect.compute_volume

        def recorded(*args, **kwargs):
            calls.append(kwargs.copy())
            return original(*args, **kwargs)

        with patch.object(detect, "compute_volume", side_effect=recorded):
            detect.estimate_volume_uncertainty(base, loaded, result, **options)
        self.assertGreaterEqual(len(calls), 4)
        for kwargs in calls:
            self.assertEqual("median", kwargs["surface_stat"])
            self.assertIs(False, kwargs["pose"])
            self.assertEqual(0, kwargs["yaw"])

    def test_partial_perturbation_failure_is_not_reported_as_complete(self):
        """扰动两侧都成功才给出敏感度，不能只拿成功的一侧掩盖另一侧失败。"""
        base, loaded, options, result = self._case()
        original = detect.compute_volume

        def one_failure(*args, **kwargs):
            if kwargs.get("dy") == detect.VOLUME_ERR_DY_M:
                return detect.VolumeResult()
            return original(*args, **kwargs)

        with patch.object(detect, "compute_volume", side_effect=one_failure):
            uncertainty = detect.estimate_volume_uncertainty(base, loaded, result, **options)
        self.assertTrue(math.isnan(uncertainty.dy_m3))
        self.assertIn("前后", uncertainty.missing_parts)

    def test_stale_explicit_decision_matches_the_main_reestimated_decision(self):
        """主算法会重估过期偏移标签，误差估计必须先取得同一份现行几何。"""
        base, loaded, _, _ = self._case("max", dz=-0.05)
        for name, estimator in (("pose", detect.estimate_pose_registration),
                                ("yaw", detect.estimate_yaw_registration)):
            with self.subTest(component=name):
                fresh = estimator(base, loaded, 0.0, 0.0)
                stale = estimator(base, loaded, 0.0, 0.5)
                options = dict(dx=0.0, dy=0.0, yaw=0, pose=False)
                options[name] = fresh
                stale_options = dict(options, **{name: stale})
                fresh_result = detect.compute_volume_auto(base, loaded, **options)
                stale_result = detect.compute_volume_auto(base, loaded, **stale_options)
                self.assertAlmostEqual(fresh_result.volume_m3, stale_result.volume_m3, places=9)
                self.assertAlmostEqual(fresh_result.uncertainty.dy_m3,
                                       stale_result.uncertainty.dy_m3, places=9)


class TestFillErrorCompleteness(_EndWallScene, unittest.TestCase):
    def _side_slope(self, random, xc, n=100000):
        x = random.uniform(xc - self.W / 2, xc + self.W / 2, n)
        y = random.uniform(-self.L / 2, self.L / 2, n)
        h = 0.3 + 0.8 * (1 - (2 * (x - xc) / self.W) ** 2)
        return np.column_stack((x, y, self._z(y) + h))

    def test_structural_fill_has_separate_error_components(self):
        """旧实现无普通空洞就报外推0，但侧壁、端板补出的煤面仍有误差。"""
        result = detect.compute_volume_auto(self._baseline(), self._loaded(surface=self._side_slope),
                                            dy=0.0, dx=0.0, yaw=0, pose=False)
        self.assertGreater(result.wall_ratio, 0.1)
        parts = {part.kind: part for part in result.domain.fill_parts}
        self.assertEqual({"hole", "side", "end"}, set(parts))
        self.assertGreater(parts["side"].filled_volume_m3, 0)
        self.assertGreater(parts["end"].filled_volume_m3, 0)
        self.assertGreater(parts["side"].error_m3, 0)
        self.assertGreater(parts["end"].holdout_cells, 0)
        self.assertEqual(result.filled_cells, sum(part.filled_cells for part in parts.values()))
        self.assertAlmostEqual(result.volume_m3 * result.filled_ratio,
                               sum(part.filled_volume_m3 for part in parts.values()), places=9)

    def test_no_fill_is_a_known_zero_not_missing(self):
        result = detect.compute_volume_auto(self._baseline(), self._loaded(), dy=0.0,
                                            dx=0.0, yaw=0, pose=False, fill_holes=False)
        self.assertEqual(0.0, result.uncertainty.fill_m3)
        self.assertTrue(all(part.error_m3 == 0 for part in result.domain.fill_parts))

    def test_skipped_holdout_marks_the_missing_fill_terms(self):
        result = detect.compute_volume_auto(self._baseline(), self._loaded(), dy=0.0,
                                            dx=0.0, yaw=0, pose=False, fill_check=False)
        self.assertIn("贴壁", result.uncertainty.missing_parts)
        self.assertIn("端板", result.uncertainty.missing_parts)
        self.assertIn("已估分量", result.text)
        self.assertIn("缺：", result.text)

    def test_center_uncertainty_survives_an_insignificant_pose(self):
        """分量未采纳不代表测量误差为零；仍报告成功估计的中心高差标准误。"""
        base, loaded = self._baseline(), self._loaded()
        pose = detect.estimate_pose_registration(base, loaded, 0.0, 0.0)
        self.assertGreater(pose.sigma_m, 0)
        result = detect.compute_volume_auto(base, loaded, dx=0.0, dy=0.0, yaw=0, pose=pose)
        self.assertAlmostEqual(pose.sigma_m * result.domain.area_m2,
                               result.uncertainty.pose_m3, places=9)

    def test_missing_components_are_named_and_do_not_claim_total_precision(self):
        uncertainty = detect.VolumeUncertainty(0.2, 0.2, float("nan"), float("nan"), 0.0, 10.0)
        text = uncertainty.text(10.0)
        self.assertIn("已估分量", text)
        self.assertIn("左右", uncertainty.missing_parts)
        self.assertIn("高低", uncertainty.missing_parts)
        self.assertNotIn("完整精度", text)

    def test_unavailable_cross_check_reason_is_visible_in_the_result_text(self):
        """校核已尝试但不适用时，原因必须随现有说明字段保存，不能只留在内存属性。"""
        reason = "观测分成两个不连通分量"
        with patch.object(detect, "cross_check_volume", return_value=detect.CrossCheck("unavailable", reason=reason)):
            result = detect.compute_volume_auto(self._baseline(), self._loaded(), dy=0.0,
                                                dx=0.0, yaw=0, pose=False)
        self.assertIn("交叉校核不可用", result.text)
        self.assertIn(reason, result.to_dict(raw=True)["text"])


if __name__ == "__main__":
    unittest.main()
