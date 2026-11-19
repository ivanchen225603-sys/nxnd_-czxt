"""倾斜底板的解析体积不能随空载两次格内采样位置而改变。"""

import unittest

import numpy as np

from app.detect import (EmptyBaseline, InnerDomain, LoadedResult, PoseDecision,
                        _relative_height_map, compute_volume)
from tests._volume_context import conditions


def scene(slope=.07, phase=(.01, .09), height=.1, *, heave=0., pitch=0., bump=0.):
    width, length, floor, cell = 2.4, 10., -3.7, .1
    xb, yb = np.meshgrid(np.arange(24) * cell - width / 2 + .05,
                          np.arange(100) * cell - length / 2 + phase[0])
    xl, yl = np.meshgrid(np.arange(24) * cell - width / 2 + .05,
                          np.arange(100) * cell - length / 2 + phase[1])
    residual = bump * (np.arange(100) % 3)[:, None] * (np.abs(xb) < .5)
    bottom = np.c_[xb.ravel(), yb.ravel(), (floor + slope * yb + residual).ravel()]
    wy, wz = np.meshgrid(np.arange(100) * cell - length / 2 + .05,
                         np.linspace(0., 2., 20))
    walls = [np.c_[np.full(wy.size, side), wy.ravel(),
                    (floor + slope * wy + wz).ravel()] for side in (-width / 2, width / 2)]
    cloud = np.vstack((bottom, *walls))
    baseline = EmptyBaseline(floor, (-width / 2, width / 2, -length / 2, length / 2,
                                    floor, float(cloud[:, 2].max())), cloud, len(cloud),
                             '解析真值', (0., -slope, 1., -floor))
    loaded = LoadedResult(cloud=np.c_[xl.ravel(), yl.ravel(),
        (floor + slope * yl + residual + height + heave + pitch * yl).ravel()], ok=True)
    inner = InnerDomain('resolved', '已知几何', (-width / 2, 0.), (width / 2, 0.),
                        0., -length / 2, length / 2, None)
    pose = (PoseDecision('resolved', '已知姿态', left=(heave, pitch), right=(heave, pitch),
                         x_left=-width / 2, x_right=width / 2) if heave or pitch else False)
    return baseline, loaded, dict(voxel=cell, dx=0., dy=0., yaw=0., pose=pose,
                                  inner=inner, fill_check=False)


class TestRelativeHeightVolume(unittest.TestCase):
    def test_tilt_and_opposite_sampling_phases_preserve_known_thin_coal_volume(self):
        for slope in (0., .07, .15):
            for phase in ((.01, .09), (.05, .05), (.09, .01)):
                with self.subTest(slope=slope, phase=phase):
                    baseline, loaded, options = scene(slope, phase)
                    before = baseline.cloud.copy(), loaded.cloud.copy()
                    result = compute_volume(baseline, loaded, **options)
                    self.assertTrue(result.ok, result.text)
                    self.assertAlmostEqual(2.4, result.volume_m3, places=10)
                    np.testing.assert_array_equal(baseline.cloud, before[0])
                    np.testing.assert_array_equal(loaded.cloud, before[1])

    def test_empty_thick_load_pose_and_observed_floor_residual(self):
        for changes in (dict(height=0.), dict(height=1.), dict(heave=-.05, pitch=.03),
                        dict(bump=.1)):
            with self.subTest(changes=changes):
                baseline, loaded, options = scene(**changes)
                result = compute_volume(baseline, loaded, **options)
                self.assertTrue(result.ok, result.text)
                self.assertAlmostEqual(24 * changes.get('height', .1), result.volume_m3, places=10)

    def test_declared_model_keeps_existing_relative_reference(self):
        baseline, loaded, options = scene()
        options.update(conditions(baseline, loaded, domain=(-1.2, 1.2, -5., 5.),
                                  origin=(-1.2, -5.)))
        result = compute_volume(baseline, loaded, **options)
        self.assertTrue(result.ok, result.text)
        self.assertAlmostEqual(2.4, result.volume_m3, places=10)

    def test_transverse_slope_and_roll_keep_volume_when_horizontal_sampling_moves(self):
        for slope_x in (0., .08):
            for roll in (0., .03):
                with self.subTest(slope_x=slope_x, roll=roll):
                    baseline, loaded, options = scene()
                    baseline.cloud[:, 2] += slope_x * baseline.cloud[:, 0]
                    baseline.bed_plane = (-slope_x, -.07, 1., 3.7)
                    loaded.cloud[:, 0] += .04
                    loaded.cloud[:, 2] += (slope_x + roll) * loaded.cloud[:, 0]
                    options['pose'] = PoseDecision(
                        'resolved', '已知横滚', left=(-1.2 * roll, 0.), right=(1.2 * roll, 0.),
                        x_left=-1.2, x_right=1.2)
                    result = compute_volume(baseline, loaded, **options)
                    self.assertTrue(result.ok, result.text)
                    self.assertAlmostEqual(2.4, result.volume_m3, places=10)

    def test_extreme_outside_point_is_discarded_before_reference_arithmetic(self):
        baseline, loaded, _ = scene()
        original = loaded.cloud.copy()
        stray = np.vstack((original, [1e308, 1e308, 1e308]))
        with np.errstate(all='raise'):
            expected = _relative_height_map(original, baseline, 0., -1.2, -5., 24, 100, .1, 'max')
            actual = _relative_height_map(stray, baseline, 0., -1.2, -5., 24, 100, .1, 'max')
        np.testing.assert_array_equal(actual[0], expected[0])
        np.testing.assert_array_equal(actual[1], expected[1])


if __name__ == '__main__':
    unittest.main()
