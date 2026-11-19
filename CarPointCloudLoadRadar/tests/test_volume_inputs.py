"""体积输入真实时序、合帧角色和共用实际旋转；不注入预期体积。"""
from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from app.bed_reference import BedReferenceScope, valid_surface_scope
from app.cloud_observation import ObservationWindow
from app.detect import compute_volume, detect_loaded
from app.pointcloud import Frame
from app.volume_inputs import align_extra_surfaces, surface_lineage
from tests.test_cloud_observation import pair
from tests.test_volume_validity import scene
from tests._volume_context import conditions


class TestVolumeInputs(unittest.TestCase):
    def scope(self, ids, members):
        return BedReferenceScope("version-1", "cal-1", "a" * 64, "CH1", "p-1", "process-1:2",
                                 ids, "loaded-heading-m", ids[-1][2], members)

    def test_latest_primary_and_older_extras_preserve_chronology(self):
        pairs = tuple(pair(i) for i in (1, 2, 3))
        ids, members = surface_lineage(ObservationWindow((pairs[-1],)), ObservationWindow(pairs), (0, 1))
        self.assertEqual(((3, 1, 1001.), (3, 2, 1002.), (3, 3, 1003.)), ids)
        self.assertEqual(((2,), (0,), (1,)), members)
        self.assertTrue(valid_surface_scope(self.scope(ids, members), 3))

    def test_merged_main_and_reused_extras_are_not_new_frames(self):
        window = ObservationWindow(tuple(pair(i) for i in (1, 2, 3)))
        ids, members = surface_lineage(window, window, (0, 1))
        self.assertEqual(((0, 1, 2), (0,), (1,)), members)
        self.assertTrue(valid_surface_scope(self.scope(ids, members), 3))
        self.assertEqual(3, len(ids))
        ids, members = surface_lineage(window, window, ())
        self.assertEqual(((0, 1, 2),), members)
        self.assertTrue(valid_surface_scope(self.scope(ids, members), 1))

    def test_scope_invalid_mapping_never_authorizes(self):
        ids = tuple((3, i, 1000. + i) for i in (1, 2, 3))
        for members in ([], ((),), ((2,),), ((0, 0, 1, 2),), ((0, 2, 1),),
                        ((True, 1, 2),), ((-1, 0, 1, 2),), ((0, 1, 2, 3),), ((0,), (1, 2))):
            with self.subTest(members=members):
                self.assertFalse(valid_surface_scope(self.scope(ids, members), 1))

    def test_scope_legacy_one_frame_per_surface_still_valid(self):
        ids = ((3, 1, 1001.), (3, 2, 1002.))
        scope = self.scope(ids, ())
        self.assertTrue(valid_surface_scope(scope, 2))
        self.assertFalse(valid_surface_scope(scope, 1))

    def test_actual_context_accepts_merged_frame_mapping(self):
        base, loaded = scene(.4)
        ids = tuple((3, i, 1000. + i) for i in (1, 2, 3))
        scope = self.scope(ids, ((0, 1, 2),))
        options = conditions(base, loaded, scope=scope)
        options["volume_context"] = replace(options["volume_context"], valid_until=1010.)
        result = compute_volume(base, loaded, dy=0., **options)
        self.assertEqual("conditional", result.range_status)
        self.assertAlmostEqual(9.6, result.volume_m3)
        altered = replace(scope, surface_members=((2,), (0,), (1,)))
        result = compute_volume(base, loaded, dy=0., **dict(options, volume_scope=altered))
        self.assertEqual("rejected", result.range_status)

    def test_lineage_cross_context_and_member_order_rejected(self):
        primary = ObservationWindow((pair(3),))
        support = ObservationWindow(tuple(pair(i) for i in (1, 2, 3)))
        for members in ((1, 0), (0, 0), (True,), (3,), (2,), [0]):
            with self.subTest(members=members), self.assertRaises(ValueError):
                surface_lineage(primary, support, members)
        other = ObservationWindow((pair(3, epoch=9),))
        with self.assertRaises(ValueError):
            surface_lineage(other, support, (0,))

    def test_same_id_different_bytes_rejected(self):
        original = pair(1)
        changed_raw = replace(original.raw, xyz_bytes=np.ones((4, 3), "<f4").tobytes())
        changed = replace(original, raw=changed_raw)
        primary = ObservationWindow((original, pair(2)))
        support = ObservationWindow((changed, pair(2)))
        with self.assertRaisesRegex(ValueError, "不同观测内容"):
            surface_lineage(primary, support, (0,))
        # 未作为extra选取的支持成员也不能与主窗口同身份异内容。
        primary = ObservationWindow((original, pair(3)))
        support = ObservationWindow((changed, pair(2), pair(3)))
        with self.assertRaisesRegex(ValueError, "不同观测内容"):
            surface_lineage(primary, support, (1,))

    def test_bed_reference_also_requires_actual_surface_mapping(self):
        from tests._bed_reference import reference
        base, loaded = scene(1.2)
        options = reference(base, loaded, .2, xy=(.25, .5))
        invalid = replace(options["bed_scope"], surface_members=((0,), (0,)))
        options.update(bed_scope=invalid, bed_evidence=replace(options["bed_evidence"], scope=invalid))
        result = compute_volume(base, loaded, dy=0., **options)
        self.assertEqual("rejected", result.bed_reference_status)
        self.assertEqual(0., result.bed_shift_applied_m)
        self.assertAlmostEqual(28.8, result.volume_m3)

    def test_rotation_shared_not_reestimated_and_immutable(self):
        angle = np.deg2rad(3.)
        matrix = np.array([[np.cos(angle), -np.sin(angle), 0.],
                           [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
        extras = [np.array([[1., 2., 3.], [4., 5., 6.]], np.float32)]
        aligned = align_extra_surfaces(matrix, extras)
        np.testing.assert_allclose(aligned[0], extras[0].astype(np.float64) @ matrix.T, rtol=0, atol=0)
        before = aligned[0].copy()
        extras[0][:] = 0
        np.testing.assert_array_equal(aligned[0], before)
        with self.assertRaises(ValueError):
            aligned[0].setflags(write=True)

    def test_invalid_rotation_and_cloud_rejected(self):
        cloud = np.ones((2, 3))
        for matrix in (None, np.eye(2), np.eye(3) * 2, np.diag([1., 1., -1.]),
                       np.eye(3) * np.nan, np.eye(3, dtype=complex)):
            with self.subTest(matrix=matrix), self.assertRaises(ValueError):
                align_extra_surfaces(matrix, (cloud,))
        for extra in (np.ones((2, 4)), np.ones((2, 3)) * np.inf, np.ones((2, 3), complex)):
            with self.assertRaises(ValueError):
                align_extra_surfaces(np.eye(3), (extra,))

    def test_loaded_retains_actual_rotation_even_before_plane_fit(self):
        cloud = np.array([[1., 2., 3.], [4., 5., 6.]], np.float32)
        matrix = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        cfg = SimpleNamespace(heading=SimpleNamespace(pct=15., min_correction=.005))
        with patch("app.detect.correct_heading", return_value=(cloud.astype(float) @ matrix.T, 1., matrix)):
            loaded = detect_loaded(Frame(cloud), cfg)
        np.testing.assert_array_equal(matrix, loaded.source_rotation)
        np.testing.assert_array_equal(loaded.cloud, align_extra_surfaces(loaded.source_rotation, (cloud,))[0])
