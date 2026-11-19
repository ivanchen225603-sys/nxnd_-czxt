"""体积有效性、同域非周期补点及机器合同；真值由解析场景给出。"""
import json
import unittest
from unittest.mock import patch

import numpy as np

from app.detect import EmptyBaseline, LoadedResult, VolumeResult, _fill_holes, compute_volume, compute_volume_auto
from app.height_fill import fill_height_holes
from cli.volume_calc import interp_fill
from tests._volume_context import conditions


def scene(height=1.0, *, observed_boundary=True):
    x, y = np.meshgrid((np.arange(24) + .5) * .1, (np.arange(100) + .5) * .1)
    floor = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    if observed_boundary:
        # 原夹具只有格心，实测范围为2.3 x 9.9m，不能由整帧box声明成24m2。
        # 解析真值24m2的场景补齐四角实测支持，保留原体积期望及中间采样。
        floor = np.vstack((floor, [[0., 0., 0.], [2.4, 0., 0.],
                                  [0., 10., 0.], [2.4, 10., 0.]]))
    base = EmptyBaseline(floor_z=0., box=(0., 2.4, 0., 10., 0., 3.), cloud=floor, n=len(floor),
                         bed_plane=(0., 0., 1., 0.))
    loaded = floor.copy()
    loaded[:, 2] = height
    return base, LoadedResult(cloud=loaded)


def valid_result(**changes):
    values = dict(ok=True, volume_m3=1., voxel_size=.1, coverage=1., bound_lo=.99,
                  bound_hi=1.01, bound_spread=.02, hole_ratio=.01,
                  range_kind="conditional_cell_mean", range_status="conditional")
    values.update(changes)
    return VolumeResult(**values)


class TestHeightFill(unittest.TestCase):
    def test_exact_linear_interior_both_entrypoints(self):
        source = np.tile([0., np.nan, np.nan, np.nan, np.nan, 10.], (3, 1))
        expected = np.tile([0., 2., 4., 6., 8., 10.], (3, 1))
        for function in (_fill_holes, interp_fill):
            np.testing.assert_allclose(function(source), expected, rtol=0, atol=1e-6)
        np.testing.assert_array_equal(_fill_holes(source), interp_fill(source))

    def test_far_edge_cannot_be_a_direct_neighbor(self):
        for value in (9., 19.):
            source = np.tile([np.nan, 1., 1., 1., value], (3, 1))
            for function in (_fill_holes, interp_fill):
                np.testing.assert_allclose(function(source)[:, 0], 1., rtol=0, atol=1e-12)

    def test_measured_anchors_and_input_are_unchanged(self):
        source = np.array([[1., 4., np.nan], [np.nan, 2., 9.]])
        original = source.copy()
        mask = np.isfinite(source)
        for function in (_fill_holes, interp_fill):
            output = function(source)
            np.testing.assert_array_equal(output[mask], source[mask])
            np.testing.assert_array_equal(source, original)
            self.assertFalse(np.shares_memory(output, source))

    def test_domain_barrier_and_anchorless_component_remain_unknown(self):
        source = np.full((3, 7), np.nan)
        source[:, 0], source[:, 6], source[:, 3] = 0., 10., 1e6
        domain = np.ones(source.shape, dtype=bool)
        domain[:, 3] = False
        for function in (_fill_holes, interp_fill):
            output = function(source, domain=domain)
            np.testing.assert_array_equal(output[:, :3], 0.)
            np.testing.assert_array_equal(output[:, 4:], 10.)
            self.assertTrue(np.isnan(output[:, 3]).all())
            no_anchor = source.copy()
            no_anchor[:, 6] = np.nan
            self.assertTrue(np.isnan(function(no_anchor, domain=domain)[:, 3:]).all())

    def test_single_row_column_empty_and_all_unknown(self):
        for shape in ((1, 5), (5, 1)):
            source = np.array([0., np.nan, np.nan, np.nan, 8.]).reshape(shape)
            expected = np.array([0., 2., 4., 6., 8.]).reshape(shape)
            np.testing.assert_allclose(fill_height_holes(source), expected, rtol=0, atol=1e-6)
        self.assertEqual((0, 0), fill_height_holes(np.empty((0, 0))).shape)
        self.assertTrue(np.isnan(fill_height_holes(np.full((4, 3), np.nan))).all())

    def test_zero_iterations_and_invalid_contract(self):
        source = np.array([[1., np.nan]])
        np.testing.assert_array_equal(source, fill_height_holes(source, iters=0))
        for iters in (-1, 1.5, True):
            with self.assertRaises(ValueError):
                fill_height_holes(source, iters=iters)
        for domain in (np.ones((1, 2)), np.ones((3, 3), dtype=bool)):
            with self.assertRaises(ValueError):
                fill_height_holes(source, domain=domain)
        with self.assertRaises(ValueError):
            fill_height_holes(np.ones(3))

    def test_finite_neighbor_mean_does_not_overflow_by_summing_first(self):
        for value in (1e308, np.finfo(float).max, -np.finfo(float).max):
            for index in ((1, 1), (0, 1)):
                source = np.full((3, 3), value)
                source[index] = np.nan
                with np.errstate(over="raise", invalid="raise"):
                    output = fill_height_holes(source)
                self.assertTrue(np.isfinite(output).all())
                self.assertAlmostEqual(1., output[index] / value)

    def test_separate_components_do_not_share_numeric_scale(self):
        source = np.array([[1e-200, np.nan, np.nan, np.nan, 1e308]])
        domain = np.array([[True, True, False, True, True]])
        output = fill_height_holes(source, domain=domain)
        self.assertEqual(1e-200, output[0, 1])
        self.assertEqual(1e308, output[0, 3])
        self.assertTrue(np.isnan(output[0, 2]))


class TestVolumeResultValidity(unittest.TestCase):
    def test_failure_default_is_unknown_not_zero_on_wire(self):
        result = VolumeResult()
        self.assertEqual(0., result.volume_m3)       # 内部旧接口兼容，输出语义不混淆。
        for raw in (False, True):
            data = result.to_dict(raw=raw)
            self.assertIsNone(data["volume_m3"])
            self.assertFalse(data["low_confidence"])
            json.dumps(data, allow_nan=False)
        self.assertFalse(result.confidence()[0])

    def test_nonfinite_values_are_json_null_and_quality_invalid(self):
        for name in ("volume_m3", "voxel_size", "coverage", "filled_ratio", "wall_ratio", "hole_ratio",
                     "bound_lo", "bound_hi", "bound_spread"):
            for value in (float("nan"), float("inf"), float("-inf")):
                with self.subTest(name=name, value=value):
                    result = valid_result(**{name: value})
                    self.assertFalse(result.confidence()[0])
                    for raw in (False, True):
                        data = result.to_dict(raw=raw)
                        self.assertIsNone(data[name])
                        json.dumps(data, allow_nan=False)

    def test_invalid_ratios_grid_coverage_and_bounds_cannot_be_confident(self):
        for changes in (dict(ok=False), dict(volume_m3=-1.), dict(voxel_size=0.), dict(coverage=0.),
                        dict(coverage=1.1), dict(hole_ratio=-.1), dict(filled_ratio=1.1),
                        dict(wall_ratio=-.2), dict(bound_spread=-.1), dict(bound_lo=-.1),
                        dict(bound_lo=2., bound_hi=1.), dict(bound_lo=1.1, bound_hi=1.2)):
            with self.subTest(changes=changes):
                self.assertFalse(valid_result(**changes).confidence()[0])
        self.assertFalse(valid_result().confidence(spread_max=float("nan"))[0])

    def test_legal_zero_and_exact_thresholds_are_distinct_from_missing(self):
        result = valid_result(volume_m3=0., bound_lo=0., bound_hi=0., bound_spread=0., hole_ratio=0.)
        self.assertTrue(result.confidence()[0])
        self.assertEqual(0., result.to_dict()["volume_m3"])
        self.assertTrue(valid_result(hole_ratio=.15, bound_spread=.03).confidence()[0])

    def test_low_quality_estimate_retained_and_raw_precision_preserved(self):
        # 低质量的杠杆改用 hole_ratio —— 2026-09-18 起范围宽度不再当闸门
        # （现场决定：质量筛选放到下游第三方系统），但**原精度仍要保住**，
        # 这才是本条真正要钉的东西。
        result = valid_result(volume_m3=1.001234567, bound_hi=1.2, bound_spread=.15000001,
                              hole_ratio=.15000001)
        self.assertFalse(result.confidence()[0])
        self.assertEqual(1.0012, result.to_dict()["volume_m3"])
        self.assertEqual(1.001234567, result.to_dict(raw=True)["volume_m3"])
        self.assertEqual(.15000001, result.to_dict(raw=True)["bound_spread"])


class TestComputeVolumeValidity(unittest.TestCase):
    def test_array_plane_is_accepted_without_ambiguous_truth_value(self):
        base, loaded = scene()
        base.bed_plane = np.asarray(base.bed_plane)
        result = compute_volume(base, loaded, dy=0.)
        self.assertTrue(result.ok, result.text)
        self.assertAlmostEqual(24., result.volume_m3)

    def test_full_empty_zero_and_unshifted_thin_layer_analytic_control(self):
        for height, expected in ((0., 0.), (.4, 9.6), (1., 24.)):
            base, loaded = scene(height)
            result = compute_volume(base, loaded, voxel=.1, dy=0.)
            self.assertTrue(result.ok, result.text)
            self.assertAlmostEqual(expected, result.volume_m3, places=6)
            self.assertEqual(1., result.coverage)
            self.assertFalse(result.confidence()[0])
            self.assertIsNone(result.to_dict()["bound_hi"])
            bounded = compute_volume(base, loaded, voxel=.1, dy=0., **conditions(base, loaded))
            self.assertAlmostEqual(expected, bounded.volume_m3, places=6)
            self.assertAlmostEqual(expected, bounded.bound_lo, places=6)
            self.assertAlmostEqual(expected, bounded.bound_hi, places=6)
            self.assertTrue(bounded.confidence()[0], bounded.confidence()[1])

    def test_partial_empty_is_not_a_confident_zero(self):
        base, loaded = scene(0.)
        loaded.cloud = loaded.cloud[loaded.cloud[:, 1] >= 2.]
        result = compute_volume(base, loaded, dy=0.)
        self.assertTrue(result.ok)
        self.assertEqual(0., result.volume_m3)
        self.assertLess(result.coverage, 1.)
        self.assertFalse(result.confidence()[0])

    def test_no_floor_region_sparse_baseline_or_no_interior_observation_fails(self):
        for kind in ("high_floor", "sparse", "outside"):
            base, loaded = scene()
            if kind == "high_floor":
                base.cloud[:, 2] = 2.
            elif kind == "sparse":
                base.cloud, base.n = base.cloud[:30], 30
            else:
                loaded.cloud[:, 0] += 20.
            with self.subTest(kind=kind):
                result = compute_volume(base, loaded, dy=0.)
                self.assertFalse(result.ok)
                self.assertIsNone(result.to_dict()["volume_m3"])
                self.assertFalse(result.confidence()[0])

    def test_invalid_inputs_fail_before_indexing_or_large_allocation(self):
        mutations = [lambda b, l: setattr(b, "floor_z", float("nan")),
                     lambda b, l: setattr(b, "box", (0., 2., 0., float("inf"), 0., 3.)),
                     lambda b, l: setattr(b, "bed_plane", (0., 0., 0., 1.)),
                     lambda b, l: setattr(b, "bed_plane", (0., float("nan"), 1., 0.)),
                     lambda b, l: setattr(l, "cloud", np.zeros((100, 2))),
                     lambda b, l: l.cloud.__setitem__((0, 2), float("nan")),
                     lambda b, l: b.cloud.__setitem__((0, 0), float("inf")),
                     lambda b, l: setattr(b, "n", 1)]
        for mutate in mutations:
            base, loaded = scene()
            mutate(base, loaded)
            with self.subTest(mutate=mutate), patch("app.detect._height_map", side_effect=AssertionError("非法输入不可进栅格")):
                result = compute_volume(base, loaded, dy=0.)
                self.assertFalse(result.ok)
                self.assertIsNone(result.to_dict()["volume_m3"])
        for kwargs in (dict(voxel=float("inf")), dict(voxel=0.), dict(voxel=1e-320),
                       dict(dy=float("nan")), dict(bed_z=float("inf")), dict(floor_band_m=-1.),
                       dict(loaded_extra=[np.full((3, 3), float("nan"))])):
            base, loaded = scene()
            with self.subTest(kwargs=kwargs):
                self.assertFalse(compute_volume(base, loaded, **kwargs).ok)

    def test_auto_does_not_accept_low_spread_invalid_candidate_early(self):
        bad = valid_result(coverage=0., bound_spread=0.)
        good = valid_result(voxel_size=.2)
        with patch("app.detect.compute_volume", side_effect=[bad, good]) as compute:
            result = compute_volume_auto(None, None, cells=(.1, .2))
        self.assertEqual(2, compute.call_count)
        self.assertIs(result, good)

    def test_auto_skips_invalid_even_when_legal_estimate_is_low_quality(self):
        bad = valid_result(coverage=0., bound_spread=0.)
        low = valid_result(voxel_size=.2, bound_spread=.1)
        with patch("app.detect.compute_volume", side_effect=[bad, low]):
            self.assertIs(low, compute_volume_auto(None, None, cells=(.1, .2)))

    def test_auto_retains_missing_range_but_not_invalid_estimate(self):
        missing = valid_result(bound_lo=float("nan"), bound_hi=float("nan"), bound_spread=float("nan"),
                               range_kind="unavailable", range_status="missing")
        bad = valid_result(coverage=0., bound_spread=0.)
        with patch("app.detect.compute_volume", side_effect=[missing, bad]) as run:
            result = compute_volume_auto(None, None, cells=(.1, .2))
        self.assertEqual(2, run.call_count)
        self.assertIs(missing, result)
        self.assertTrue(result.ok)
        self.assertEqual(1., result.to_dict()["volume_m3"])
        self.assertIsNone(result.to_dict()["bound_spread"])
        self.assertFalse(result.confidence()[0])

    def test_finite_inputs_with_overflowing_height_cannot_be_confident_zero(self):
        base, loaded = scene()
        base.cloud[:, 2], base.floor_z = -1e308, -1e308
        base.bed_plane, base.box = (0., 0., 1., 1e308), (0., 2.4, 0., 10., -1e308, 1e308)
        loaded.cloud[:, 2] = 1e308
        with np.errstate(over="raise", invalid="raise"):
            result = compute_volume(base, loaded, dy=0.)
        self.assertFalse(result.ok)
        self.assertIsNone(result.to_dict()["volume_m3"])
        self.assertFalse(result.confidence()[0])

    def test_auto_rejects_invalid_parameters_without_evaluating_candidates(self):
        for kwargs in (dict(spread_target=float("nan")), dict(spread_target=None),
                       dict(spread_target=True), dict(spread_target=-.1), dict(spread_target=1.1),
                       dict(cells=("bad",)), dict(cells=None), dict(cells=()),
                       dict(cells=(.1, False)), dict(cells=(float("inf"),))):
            with self.subTest(kwargs=kwargs), patch("app.detect.compute_volume") as compute:
                result = compute_volume_auto(*scene(), **kwargs)
                compute.assert_not_called()
                self.assertFalse(result.ok)
                self.assertIsNone(result.to_dict()["volume_m3"])

    def test_auto_registration_range_rejected_before_allocating_bins(self):
        for owner in ("baseline", "loaded"):
            base, loaded = scene()
            cloud = base.cloud if owner == "baseline" else loaded.cloud
            cloud[:, 1] = np.linspace(0., 1e7, len(cloud))
            with patch("app.detect.estimate_dy", side_effect=AssertionError("不能先分配巨型配准桶")):
                result = compute_volume(base, loaded)
                self.assertFalse(result.ok)

    def test_huge_outside_extra_points_do_not_overflow_grid_index(self):
        base, loaded = scene()
        with np.errstate(over="raise", invalid="raise"):
            result = compute_volume(base, loaded, dy=0., loaded_extra=[np.full((1, 3), 1e308)])
        self.assertTrue(result.ok, result.text)
        self.assertAlmostEqual(24., result.volume_m3)


class TestServiceVolumeWire(unittest.TestCase):
    @staticmethod
    def snapshot(worker, base, reservation):
        from app.bed_reference import baseline_digest
        from app.load_radar_bridge import BaselineSnapshot
        binding = dict(node_id="test", channel_id=worker.ch.id, calibration_id="cal-1",
                       configuration_sha256=worker._integration_hash, baseline_version_id="test-version",
                       baseline_asset_sha256="b" * 64, baseline_sha256=baseline_digest(base))
        return BaselineSnapshot(reservation.passage_id, json.dumps(binding).encode("utf-8"),
                                np.asarray(base.cloud, "<f8").tobytes(), base.floor_z, base.box, base.bed_plane)

    def test_actual_worker_keeps_known_version_estimate_with_null_range(self):
        from tests.test_result_finalizer import TestResultFinalizer, await_result
        harness = TestResultFinalizer()
        harness.setUp()
        try:
            bridge = harness.bridge()
            worker = harness.worker(bridge)
            volume = valid_result(volume_m3=1.001234567, bound_lo=float("nan"), bound_hi=float("nan"),
                                  bound_spread=float("nan"), range_kind="unavailable", range_status="missing")
            with patch.object(bridge, "baseline_snapshot", side_effect=lambda identity, *_:
                              self.snapshot(worker, scene()[0], identity)), \
                    patch("app.application.channel_worker.compute_volume_auto", return_value=volume):
                harness.enter(worker)
                result = await_result(worker)[0]
            payload = bridge.outbox.get(result["measurement_id"]).payload
            bundle = bridge.outbox.archives.get(result["measurement_id"])
            stored = json.loads(dict(bundle.files)["detection.xyz.meta.json"])
            for data in (result["volume"], stored["volume"]):
                self.assertEqual(1.001234567, data["volume_m3"])
                self.assertIsNone(data["bound_spread"])
                self.assertFalse(data["low_confidence"])
            self.assertEqual(1.001234567, payload["volumeM3"])
            self.assertEqual("success", payload["volumeStatus"])
            self.assertIsNone(payload["boundSpread"])
            self.assertEqual(23, len(payload))
            self.assertIn("measurementStartedAt", payload)
            self.assertIn("measurementEndedAt", payload)
            json.dumps(payload, allow_nan=False)
        finally:
            self.assertTrue(harness.doCleanups())

    def test_actual_worker_with_version_keeps_low_quality_estimate_in_outbox(self):
        from tests.test_result_finalizer import TestResultFinalizer, await_result
        harness = TestResultFinalizer()
        harness.setUp()
        try:
            bridge = harness.bridge()
            worker = harness.worker(bridge)
            volume = valid_result(volume_m3=1.001234567, bound_hi=1.2, bound_spread=.03000001,
                                  hole_ratio=.15000001)   # 低质量的杠杆现在是空洞占比
            # 仅替换体积生产者和基线解析，不连接后台或激活真实基线。
            with patch.object(bridge, "baseline_snapshot", side_effect=lambda identity, *_:
                              self.snapshot(worker, scene()[0], identity)), \
                    patch("app.application.channel_worker.compute_volume_auto", return_value=volume):
                harness.enter(worker)
                result = await_result(worker)[0]
            payload = bridge.outbox.get(result["measurement_id"]).payload
            bundle = bridge.outbox.archives.get(result["measurement_id"])
            stored = json.loads(dict(bundle.files)["detection.xyz.meta.json"])
            self.assertEqual("test-version", payload["baselineVersionId"])
            # 范围宽度已不当闸门；这一份的低质量来自 hole_ratio（见 volume 夹具）。
            self.assertEqual("success", payload["volumeStatus"])
            for value in (result["volume"], stored["volume"]):
                self.assertEqual(volume.volume_m3, value["volume_m3"])
                self.assertEqual(volume.bound_spread, value["bound_spread"])
                self.assertFalse(value["low_confidence"])
            self.assertEqual(volume.volume_m3, payload["volumeM3"])
            self.assertEqual(volume.bound_spread, payload["boundSpread"])
        finally:
            self.assertTrue(harness.doCleanups())

    def test_actual_worker_no_baseline_stays_null_in_archive_and_outbox(self):
        from tests.test_result_finalizer import TestResultFinalizer, await_result
        harness = TestResultFinalizer()
        harness.setUp()
        try:
            bridge = harness.bridge()
            worker = harness.worker(bridge)
            harness.enter(worker)
            result = await_result(worker)[0]
            self.assertFalse(result["volume"]["ok"])
            self.assertIsNone(result["volume"]["volume_m3"])
            self.assertFalse(result["volume"]["low_confidence"])
            row = bridge.outbox.get(result["measurement_id"])
            self.assertIsNone(row.payload["volumeM3"])
            bundle = bridge.outbox.archives.get(result["measurement_id"])
            stored = json.loads(dict(bundle.files)["detection.xyz.meta.json"])
            self.assertIsNone(stored["volume"]["volume_m3"])
            json.dumps(stored, allow_nan=False)
        finally:
            self.assertTrue(harness.doCleanups())

    def test_actual_worker_keeps_raw_estimate_and_quality_before_rounding(self):
        from tests.test_result_finalizer import TestResultFinalizer, await_result
        harness = TestResultFinalizer()
        harness.setUp()
        try:
            bridge = harness.bridge()
            worker = harness.worker(bridge)
            volume = valid_result(volume_m3=1.001234567, bound_hi=1.2, bound_spread=.03000001,
                                  hole_ratio=.15000001)   # 低质量的杠杆现在是空洞占比
            with patch("app.application.channel_worker.compute_volume_auto", return_value=volume):
                harness.enter(worker)
                result = await_result(worker)[0]
            self.assertEqual(1.001234567, result["volume"]["volume_m3"])
            self.assertEqual(.03000001, result["volume"]["bound_spread"])
            self.assertFalse(result["volume"]["low_confidence"])   # 由 hole_ratio 触发
            # 没有正式基线版本，机器合同仍拒绝发布这个注入估计。
            self.assertIsNone(bridge.outbox.get(result["measurement_id"]).payload["volumeM3"])
        finally:
            self.assertTrue(harness.doCleanups())
