# -*- coding: utf-8 -*-
"""合成真值：平移统一取车辆点 p50_xy，垂直形变仍由原形状门控限制。"""
import unittest

import numpy as np

from app.pointcloud import Frame
from tests import test_standstill as standstill_cases


class TestHorizontalTranslationStandstill(unittest.TestCase):
    # 只复用夹具方法，不继承测试类，避免重复运行原有整组测试。
    setUp = standstill_cases.TestContinuousStandstill.setUp
    detector = standstill_cases.TestContinuousStandstill.detector
    cloud = standstill_cases.TestContinuousStandstill.cloud

    def translated(self, x=0., y=0., z=0.):
        xyz = self.car + np.array([x, y, z], np.float32)
        self.assertGreater(float(xyz[:, 2].min()), -4.0,
                           "车辆点须远离高度切线，不能因点集变化混淆平移真值")
        return Frame(xyz)

    def assert_no_translation(self, result):
        self.assertAlmostEqual(result.instant_speed_mps, 0., delta=1e-6)
        if result.speed_mps is not None:
            self.assertAlmostEqual(result.speed_mps, 0., delta=1e-6)
        self.assertFalse(result.motion_pending)

    def test_vertical_common_mode_oscillation_does_not_revoke_standstill(self):
        detector = self.detector()
        for i in range(60):
            result = self.clock.detect(detector,
                self.translated(z=.18 if i % 2 else -.18), dt=.1)
            self.assert_no_translation(result)
            self.assertLess(result.drift_m, 1e-5)
            if i >= 25:
                self.assertTrue(result.stable, (i, result.to_dict()))

    def test_cumulative_vertical_lift_does_not_enter_fixed_horizontal_drift(self):
        detector = self.detector()
        for i in range(80):
            result = self.clock.detect(detector, self.translated(z=.005 * i), dt=.1)
            self.assert_no_translation(result)
            self.assertLess(result.drift_m, 1e-5)
            if i >= 25:
                self.assertTrue(result.stable, (i, result.to_dict()))
        self.assertGreater(.005 * 79, detector.stationary_drift_m * 10,
                           "累计垂直位移须明显超过固定漂移门槛，才能区分旧三维实现")

    def test_horizontal_pending_and_recovery_ignore_large_vertical_offsets(self):
        detector = self.detector()
        before = self.clock.hold(detector, self.cloud(), steps=30, dt=.1)
        self.assertTrue(before.stable)
        pending = self.clock.detect(detector, self.translated(x=.003, z=.25), dt=.1)
        self.assertTrue(pending.motion_pending, pending.to_dict())
        self.assertFalse(pending.stable)
        self.assertAlmostEqual(pending.instant_speed_mps, .03, delta=2e-6)
        self.assertAlmostEqual(pending.drift_m, .003, delta=2e-6)
        recovered = self.clock.detect(detector, self.translated(z=-.25), dt=.1)
        self.assertTrue(recovered.jitter_confirmed, recovered.to_dict())
        self.assertFalse(recovered.motion_pending)
        self.assertTrue(recovered.stable, recovered.to_dict())
        self.assertGreater(recovered.stationary_sec, before.stationary_sec)
        self.assertAlmostEqual(recovered.instant_speed_mps, .03, delta=2e-6)
        self.assertLess(recovered.drift_m, 1e-5)

    def test_symmetric_vertical_expansion_still_triggers_shape_gate(self):
        detector = self.detector()
        self.assertTrue(self.clock.hold(detector, self.cloud(), steps=30, dt=.1).stable)
        xyz = self.car.copy()
        center = float(np.median(xyz[:, 2]))
        xyz[:, 2] = center + (xyz[:, 2] - center) * 1.02
        self.assertGreater(float(xyz[:, 2].min()), -4.0)
        result = self.clock.detect(detector, Frame(xyz), dt=.1)
        self.assert_no_translation(result)
        self.assertFalse(result.stable)
        self.assertIn("shape_speed", result.triggers)
        self.assertGreater(result.shape_speed_mps, detector.stationary_speed_mps)

    @staticmethod
    def deterministic_grid():
        x, y = np.meshgrid(np.linspace(-1., 1., 3000), [-5., -3., -1., 1., 3., 5.])
        return np.column_stack((x.ravel(), y.ravel(), np.full(x.size, -3.))).astype(np.float32)

    def test_all_translation_metrics_follow_p50_instead_of_mean_or_three_quantiles(self):
        base = self.deterministic_grid()
        middle = np.abs(base[:, 0]) < .4
        self.assertEqual(int(middle.sum()), 7200)
        detector = self.detector()
        self.assertTrue(self.clock.hold(detector, Frame(base), steps=30, dt=.1).stable)
        first_quantiles = np.percentile(base[:, 0], [10, 50, 90])
        for i in range(1, 51):
            xyz = base.copy()
            xyz[middle, 0] += .005 * (.1 * i)
            result = self.clock.detect(detector, Frame(xyz), dt=.1)
            self.assertAlmostEqual(result.instant_speed_mps, .005, delta=2e-6)
            self.assertTrue(result.stable, (i, result.to_dict()))
        last_quantiles = np.percentile(xyz[:, 0], [10, 50, 90])
        np.testing.assert_allclose(last_quantiles[[0, 2]], first_quantiles[[0, 2]], atol=1e-7)
        self.assertAlmostEqual(last_quantiles[1] - first_quantiles[1], .025, delta=2e-6)
        self.assertAlmostEqual(float(np.mean(xyz[:, 0].astype(np.float64) - base[:, 0])),
                               .010, delta=2e-6)
        self.assertAlmostEqual(result.speed_mps, .005, delta=2e-6)
        self.assertAlmostEqual(result.drift_m, .025, delta=2e-6)
        self.assertLess(result.shape_speed_mps, detector.stationary_speed_mps)

    def test_tail_distribution_can_move_mean_without_creating_p50_translation(self):
        base = self.deterministic_grid()
        tail = base[:, 0] > .6
        detector = self.detector()
        self.assertTrue(self.clock.hold(detector, Frame(base), steps=30, dt=.1).stable)
        for i in range(1, 31):
            xyz = base.copy()
            xyz[tail, 0] += .002 * i
            result = self.clock.detect(detector, Frame(xyz), dt=.1)
            # 可见面变化仍交给原 shape 门控；此处只判定平移量没有混入均值。
            self.assert_no_translation(result)
        self.assertGreater(float(np.mean(xyz[:, 0].astype(np.float64) - base[:, 0])), .01)
        self.assertAlmostEqual(float(np.median(xyz[:, 0]) - np.median(base[:, 0])), 0., delta=1e-7)


if __name__ == "__main__":
    unittest.main()
