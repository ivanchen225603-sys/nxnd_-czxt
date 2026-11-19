"""非整数尺寸、斜边和网格相位不能改变已知平煤体积。"""

import unittest

import numpy as np

from app.detect import (EmptyBaseline, InnerDomain, LoadedResult, YawDecision,
                        _strip_grid_weights, compute_volume)
from tests._volume_context import conditions
from tests.test_volume_validity import scene


def rectangular_scene(stray=None, shear=0.):
    width, length, floor = 2.37, 10.07, -3.7
    x, y = np.meshgrid(np.linspace(-width / 2, width / 2, 121),
                       np.linspace(-length / 2, length / 2, 505))
    bottom = np.c_[x.ravel(), y.ravel(), np.full(x.size, floor)]
    baseline = EmptyBaseline(floor, (-width / 2, width / 2, -length / 2, length / 2,
                                     floor, floor + 2.), bottom, len(bottom), '解析几何')
    coal = bottom.copy()
    coal[:, 0] += shear * coal[:, 1]
    coal[:, 2] += 1.
    if stray is not None:
        coal = np.vstack((coal, stray))
    return baseline, LoadedResult(cloud=coal), width * length


class TestVolumeBoundaryArea(unittest.TestCase):
    def test_centres_without_observed_boundary_do_not_authorize_whole_roi(self):
        # 同样的中心点，ROI放宽或网格换档都不能凭空补出四周半格的物理域。
        truth = (2.35 - .05) * (9.95 - .05)
        for cell in (.05, .1, .2, .3):
            for bounds in ((0., 2.4, 0., 10., 0., 3.), (-1.011, 3.4, -1.111, 11., 0., 3.)):
                with self.subTest(cell=cell, bounds=bounds):
                    base, loaded = scene(observed_boundary=False)
                    base.box = bounds
                    result = compute_volume(base, loaded, voxel=cell, dx=0., dy=0.,
                                            yaw=0., pose=False, inner=False, fill_check=False)
                    self.assertTrue(result.ok, result.text)
                    self.assertAlmostEqual(truth, result.domain.area_m2, places=10)
                    self.assertAlmostEqual(truth, result.volume_m3, places=10)

    def test_observed_boundary_supports_full_analytic_area_without_roi_expansion(self):
        for height in (.4, 1., 1.2):
            for cell in (.05, .1, .2, .3):
                with self.subTest(height=height, cell=cell):
                    base, loaded = scene(height)
                    np.testing.assert_array_equal(base.cloud[:, :2].min(axis=0), [0., 0.])
                    np.testing.assert_array_equal(base.cloud[:, :2].max(axis=0), [2.4, 10.])
                    base.box = (-1.011, 3.4, -1.111, 11., 0., 3.)
                    result = compute_volume(base, loaded, voxel=cell, dx=0., dy=0.,
                                            yaw=0., pose=False, inner=False, fill_check=False)
                    self.assertTrue(result.ok, result.text)
                    self.assertAlmostEqual(24., result.domain.area_m2, places=10)
                    self.assertAlmostEqual(24. * height, result.volume_m3, places=10)

    def test_explicit_physical_domain_can_authorize_area_not_observed_at_edges(self):
        for height in (.4, 1., 1.2):
            with self.subTest(height=height):
                base, loaded = scene(height, observed_boundary=False)
                raw = compute_volume(base, loaded, dx=0., dy=0., yaw=0., pose=False,
                                     inner=False, fill_check=False)
                declared = compute_volume(base, loaded, dy=0., **conditions(base, loaded))
                self.assertAlmostEqual(22.77 * height, raw.volume_m3, places=10)
                self.assertAlmostEqual(24. * height, declared.volume_m3, places=10)
                self.assertAlmostEqual(24., declared.domain_area_m2, places=10)
                self.assertEqual("conditional", declared.range_status)

    def test_chunked_slanted_strip_keeps_analytic_area(self):
        gx, gy = (np.arange(1100) + .5) * .1, (np.arange(300) + .5) * .1
        low, high = .023, 29.94
        result = _strip_grid_weights(gx, gy, .1, low, high,
                                     lambda y: (.37 + .01 * y, 109.13 + .01 * y))
        self.assertTrue(np.all((result >= 0.) & (result <= 1.)))
        self.assertAlmostEqual((109.13 - .37) * (high - low), result.sum() * .01, places=9)

    def test_fallback_box_preserves_known_volume_across_grid_sizes_and_origins(self):
        for cell in (.05, .1, .2, .3):
            for stray in (None, [-2.011, -5.111, -2.7]):
                with self.subTest(cell=cell, stray=stray):
                    base, loaded, truth = rectangular_scene(stray)
                    result = compute_volume(base, loaded, voxel=cell, dx=0., dy=0.,
                                            yaw=0., pose=False, inner=False, fill_check=False)
                    self.assertTrue(result.ok, result.text)
                    self.assertAlmostEqual(truth, result.volume_m3, places=10)
                    self.assertAlmostEqual(truth, result.domain.area_m2, places=10)

    def test_outside_high_point_in_boundary_cell_is_not_coal(self):
        base, loaded, truth = rectangular_scene([1.19, 0., -1.8])
        result = compute_volume(base, loaded, voxel=.3, dx=0., dy=0., yaw=0.,
                                pose=False, inner=False, fill_check=False)
        self.assertTrue(result.ok, result.text)
        self.assertAlmostEqual(truth, result.volume_m3, places=10)

    def test_fallback_shear_and_longitudinal_shift_keep_physical_area(self):
        for shear in (-.03, .03):
            for cell in (.05, .1, .2, .3):
                with self.subTest(shear=shear, cell=cell):
                    base, loaded, truth = rectangular_scene(shear=shear)
                    loaded.cloud[:, 1] += .257
                    loaded.cloud[:, 0] += shear * .257
                    result = compute_volume(
                        base, loaded, voxel=cell, dx=0., dy=.257,
                        yaw=YawDecision('resolved', '解析偏航', shear, dy_m=.257),
                        pose=False, inner=False, fill_check=False)
                    self.assertTrue(result.ok, result.text)
                    self.assertAlmostEqual(truth, result.domain.area_m2, places=10)
                    self.assertAlmostEqual(truth, result.volume_m3, places=10)

    def test_inner_taper_and_partial_end_rows_preserve_analytic_area(self):
        y0, y1, slope = -4.97, 4.94, .02
        x, y = np.meshgrid(np.linspace(-1.19, 1.19, 121), np.linspace(y0, y1, 501))
        floor = -3.7
        cloud = np.c_[x.ravel(), y.ravel(), np.full(x.size, floor)]
        base = EmptyBaseline(floor, (-2., 2., -5.003, 5., floor, floor + 2.),
                             cloud, len(cloud), '解析几何')
        loaded = LoadedResult(cloud=cloud + [0., 0., 1.])
        inner = InnerDomain('resolved', '解析斜壁', (-1.2, -slope), (1.2, slope),
                            0., y0, y1)
        truth = 2.4 * (y1 - y0) + slope * (y1 ** 2 - y0 ** 2)
        for cell in (.05, .1, .2, .3):
            with self.subTest(cell=cell):
                result = compute_volume(base, loaded, voxel=cell, dx=0., dy=0., yaw=0.,
                                        pose=False, inner=inner, fill_check=False)
                self.assertTrue(result.ok, result.text)
                self.assertAlmostEqual(truth, result.domain.area_m2, places=10)
                self.assertAlmostEqual(truth, result.volume_m3, places=10)


if __name__ == '__main__':
    unittest.main()
