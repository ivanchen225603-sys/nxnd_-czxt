"""独立解析真值验证条件范围，不以估值自包围冒充准确率。"""
from dataclasses import FrozenInstanceError, replace
import json
import unittest
from unittest.mock import patch

import numpy as np

from app.detect import EmptyBaseline, LoadedResult, VOLUME_CELLS, compute_volume, compute_volume_auto
from tests._bed_reference import reference
from tests._volume_context import conditions
from tests.test_volume_validity import scene


class TestVolumeContext(unittest.TestCase):
    def test_complete_analytic_zero_thin_and_loaded_are_useful_not_blanket_refused(self):
        for height in (0., .005, .4, 1., 2.5):
            b, l = scene(height)
            for cell in VOLUME_CELLS:
                v = compute_volume(b, l, voxel=cell, dy=0., **conditions(b, l))
                self.assertTrue(v.ok, v.text)
                self.assertAlmostEqual(height * 24, v.volume_m3, places=9)
                self.assertAlmostEqual(height * 24, v.bound_lo, places=9)
                self.assertAlmostEqual(height * 24, v.bound_hi, places=9)
                self.assertTrue(v.confidence()[0], v.confidence()[1])
                self.assertAlmostEqual(24., v.observed_area_m2)
                self.assertEqual(0., v.unknown_area_m2)

    def test_three_hidden_worlds_and_wall_share_honest_range(self):
        for wall in (False, True):
            b, l = scene(.5)
            hidden = b.cloud[:, 1] < 2.
            if wall:
                b.cloud[hidden, 2] = 2.
            l.cloud = l.cloud[~hidden]
            options = conditions(b, l)
            for cell in VOLUME_CELLS:
                for fill in (False, True):
                    v = compute_volume(b, l, voxel=cell, fill_holes=fill, dy=0., **options)
                    self.assertTrue(v.ok, v.text)
                    self.assertAlmostEqual(9.6, v.bound_lo)
                    self.assertAlmostEqual(24., v.bound_hi)
                    self.assertAlmostEqual(19.2, v.observed_area_m2)
                    self.assertAlmostEqual(4.8, v.unknown_area_m2)
                    self.assertAlmostEqual(12. if fill else 9.6, v.volume_m3)
                    # 宽度门槛默认已关（2026-09-18 现场决定，质量筛选放下游）。
                    # 这里显式给门槛，钉的是这个场景的范围确实宽。
                    self.assertFalse(v.confidence(spread_max=0.03)[0])
                    for truth in (12., 16.8, 21.6):
                        self.assertLessEqual(v.bound_lo, truth)
                        self.assertGreaterEqual(v.bound_hi, truth)
                    self.assertAlmostEqual(v.volume_m3, v.measured_volume_m3 +
                                           v.interpolated_volume_m3 + v.extrapolated_volume_m3)

    def test_missing_and_bad_conditions_keep_finite_estimate_without_pseudo_bounds(self):
        b, l = scene(observed_boundary=False)
        opt = conditions(b, l)
        bads = (None, {}, replace(opt["volume_context"], height_source="loaded_max"),
                replace(opt["volume_context"], error_source="sensor_noise_only"),
                replace(opt["volume_context"], baseline_sha256="0" * 64))
        for context in bads:
            for function in (compute_volume, compute_volume_auto):
                v = function(b, l, dy=0., volume_context=context, volume_scope=opt["volume_scope"])
                self.assertTrue(v.ok, v.text)
                # 无有效声明时只估实测底板点域：x=.05..2.35、y=.05..9.95。
                self.assertAlmostEqual(2.3 * 9.9, v.volume_m3)
                self.assertFalse(v.estimate_validity_errors())
                self.assertTrue(v.range_validity_errors())
                self.assertFalse(v.confidence()[0])
                for name in ("bound_lo", "bound_hi", "bound_spread"):
                    self.assertIsNone(v.to_dict(raw=True)[name])

    def test_scope_digest_time_registration_and_error_budget_are_checked(self):
        b, l = scene()
        opt = conditions(b, l)
        changes = (dict(domain_source="cloud_roi"), dict(max_height_m=None), dict(max_height_m=True),
            dict(max_height_m=-1.), dict(max_height_m=float("inf")), dict(support_cell_m=0.),
            dict(support_cell_m=1e-320), dict(support_origin_xy=[0., 0.]), dict(domain_xy=(0., 0., 0., 10.)),
            dict(domain_xy=(0., float("inf"), 0., 10.)), dict(surface_error_m=-.1), dict(bed_error_m=None),
            dict(roi_error_area_m2=float("nan")), dict(surface_sha256="0" * 64), dict(valid_from=1000.1),
            dict(valid_until=999.9), dict(registration_dy_m=.1), dict(support_model="point_is_proof"))
        for change in changes:
            with self.subTest(change=change):
                v = compute_volume(b, l, dy=0., **dict(opt, volume_context=replace(opt["volume_context"], **change)))
                self.assertTrue(v.ok, v.text)
                self.assertEqual("rejected", v.range_status)
                self.assertIsNone(v.to_dict()["bound_hi"])
        for scope in (None, {}, replace(opt["volume_scope"], passage_id="other"),
                      replace(opt["volume_scope"], configuration_sha256="b" * 64)):
            v = compute_volume(b, l, dy=0., **dict(opt, volume_scope=scope))
            self.assertTrue(v.ok)
            self.assertEqual("rejected", v.range_status)
        with self.assertRaises(FrozenInstanceError):
            opt["volume_context"].max_height_m = 4.

    def test_actual_surface_mutation_invalidates_conditions(self):
        b, l = scene()
        opt = conditions(b, l)
        l.cloud[0, 2] += .1
        v = compute_volume(b, l, dy=0., **opt)
        self.assertTrue(v.ok)
        self.assertEqual("rejected", v.range_status)
        self.assertIsNone(v.to_dict()["bound_spread"])

    def test_height_error_and_symmetric_roi_error_expand_both_ends(self):
        b, l = scene()
        v = compute_volume(b, l, dy=0., **conditions(b, l, surface=.02, bed=.1, roi=.4))
        self.assertAlmostEqual(21.12 - 1.2, v.bound_lo)
        self.assertAlmostEqual(26.88 + 1.2, v.bound_hi)
        self.assertAlmostEqual(24., v.volume_m3)
        # 同上：显式给门槛，钉误差把两端都撑开了，不是钉默认策略。
        self.assertFalse(v.confidence(spread_max=0.03)[0])

    def test_raw_height_interval_conflicts_not_silently_clipped(self):
        for height, error, conflict in ((3.05, .1, False), (3.05, 0., True),
                                         (-.05, .1, False), (-.05, 0., True)):
            b, l = scene(height)
            v = compute_volume(b, l, dy=0., **conditions(b, l, surface=error))
            self.assertTrue(v.ok, v.text)
            self.assertEqual("condition_conflict" if conflict else "conditional", v.range_status)
            if conflict:
                self.assertIsNone(v.to_dict()["bound_hi"])
                self.assertFalse(v.confidence()[0])
            else:
                self.assertAlmostEqual(max(0., height - error) * 24, v.bound_lo)
                self.assertAlmostEqual(min(3., height + error) * 24, v.bound_hi)

    def test_slope_is_subtracted_at_each_point_before_taking_max(self):
        b, l = scene()
        b.bed_plane = (-1., 0., 1., 0.)
        b.cloud[:, 2] = b.cloud[:, 0]
        l.cloud[:, 2] = l.cloud[:, 0] + 1.
        for cell in VOLUME_CELLS:
            v = compute_volume(b, l, voxel=cell, dy=0., **conditions(b, l, cell=.2))
            self.assertAlmostEqual(24., v.volume_m3)
            self.assertAlmostEqual(24., v.bound_lo)
            self.assertAlmostEqual(24., v.bound_hi)

    def test_independent_bed_reference_uncertainty_is_added_not_averaged_down(self):
        b, l = scene(1.2)
        ref = reference(b, l, .2, xy=(1., 5.), uncertainty=.02)
        opt = conditions(b, l, scope=ref["bed_scope"], bed=.1)
        v = compute_volume(b, l, dy=0., **ref, **opt)
        self.assertAlmostEqual(24., v.volume_m3)
        self.assertAlmostEqual(21.12, v.bound_lo)
        self.assertAlmostEqual(26.88, v.bound_hi)
        self.assertEqual(.02, v.range_conditions["applied_bed_shift_error_m"])

    def test_partial_edges_exact_area_and_rigid_translation(self):
        for translation in ((0., 0.), (100., -17.)):
            tx, ty = translation
            # 密集常高观测在任何支持格裁剪部分均满足零代表性误差。
            x, y = np.meshgrid(np.arange(.0025, 1.05, .005) + tx, np.arange(.0025, .95, .005) + ty)
            floor = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
            domain = (tx, tx + 1.05, ty, ty + .95)
            b = EmptyBaseline(floor_z=0., box=domain + (0., 3.), cloud=floor, n=len(floor))
            cargo = floor.copy()
            cargo[:, 2] = .4
            l = LoadedResult(cloud=cargo)
            for origin in ((tx, ty), (tx + .03, ty + .02)):
                for cell in VOLUME_CELLS:
                    v = compute_volume(b, l, dy=0., voxel=cell, **conditions(b, l, domain=domain, origin=origin))
                    self.assertTrue(v.ok, v.text)
                    self.assertAlmostEqual(.9975, v.domain_area_m2)
                    self.assertAlmostEqual(.9975, v.observed_area_m2)
                    self.assertAlmostEqual(.399, v.volume_m3)
                    self.assertAlmostEqual(.399, v.bound_lo)

    def test_outside_point_cannot_support_clipped_edge_cell(self):
        b, l = scene()
        opt = conditions(b, l, domain=(.08, 2.4, 0., 10.))
        v = compute_volume(b, l, dy=0., **opt)
        self.assertAlmostEqual(.02 * 10, v.unknown_area_m2)
        self.assertAlmostEqual(23., v.bound_lo)
        self.assertAlmostEqual(23.6, v.bound_hi)

    def test_complementary_frames_reduce_unknown_but_duplicates_do_not(self):
        b, full = scene()
        l = LoadedResult(cloud=full.cloud[full.cloud[:, 1] >= 2.])
        for extra, unknown in (([l.cloud], 4.8), ([full.cloud], 0.)):
            v = compute_volume(b, l, dy=0., loaded_extra=extra, **conditions(b, l, extra=extra))
            self.assertAlmostEqual(unknown, v.unknown_area_m2)
            self.assertAlmostEqual(24., v.volume_m3)
            self.assertEqual(2, v.frames_used)
            stale = conditions(b, l)
            rejected = compute_volume(b, l, dy=0., loaded_extra=extra, **stale)
            self.assertTrue(rejected.ok)
            self.assertEqual("rejected", rejected.range_status)

    def test_auto_scans_all_candidates_without_shrinking_fixed_unknown(self):
        b, l = scene()
        l.cloud = l.cloud[::2]
        opt = conditions(b, l)
        candidates = [compute_volume(b, l, voxel=c, dy=0., **opt) for c in VOLUME_CELLS]
        first = candidates[0]
        for v in candidates:
            self.assertTrue(v.ok)
            self.assertEqual(first.unknown_area_m2, v.unknown_area_m2)
            self.assertEqual(first.bound_lo, v.bound_lo)
            self.assertEqual(first.bound_hi, v.bound_hi)
            self.assertFalse(v.confidence()[0])
        with patch("app.detect.compute_volume", wraps=compute_volume) as run:
            v = compute_volume_auto(b, l, dy=0., **opt)
        self.assertEqual(len(VOLUME_CELLS), run.call_count)
        self.assertTrue(v.ok)
        self.assertAlmostEqual(first.bound_spread, v.bound_spread)

    def test_no_observed_domain_does_not_fall_back_to_outside_estimate(self):
        b, l = scene()
        v = compute_volume(b, l, dy=0., **conditions(b, l, domain=(3., 5., 0., 10.)))
        self.assertFalse(v.ok)
        self.assertEqual("no_observation", v.range_status)
        self.assertIsNone(v.to_dict()["volume_m3"])

    def test_condition_metadata_and_numeric_json_are_independent_snapshots(self):
        b, l = scene()
        v = compute_volume(b, l, dy=0., **conditions(b, l))
        data = v.to_dict(raw=True)
        json.dumps(data, allow_nan=False)
        data["range_conditions"]["domain_xy"][0] = -100.
        self.assertEqual(0., v.range_conditions["domain_xy"][0])

    def test_two_individually_valid_but_different_scopes_cannot_be_combined(self):
        b, l = scene(1.2)
        ref = reference(b, l, .2, xy=(1., 5.))
        scope = replace(ref["bed_scope"], passage_id="different-passage")
        v = compute_volume(b, l, dy=0., **ref, **conditions(b, l, scope=scope))
        self.assertTrue(v.ok)
        self.assertEqual("applied", v.bed_reference_status)
        self.assertEqual("rejected", v.range_status)
        self.assertFalse(v.confidence()[0])
        self.assertIsNone(v.to_dict()["bound_spread"])

    def test_numpy_numeric_conditions_serialize_recursively(self):
        b, l = scene()
        opt = conditions(b, l, surface=np.float32(.01), bed=np.float32(.02), roi=np.float32(.03))
        v = compute_volume(b, l, dy=0., **opt)
        self.assertTrue(v.ok)
        self.assertEqual("conditional", v.range_status)
        for raw in (False, True):
            json.dumps(v.to_dict(raw=raw), allow_nan=False)
        extended = compute_volume(b, l, dy=0., **conditions(b, l, surface=np.longdouble(.01)))
        self.assertTrue(extended.ok)
        json.dumps(extended.to_dict(raw=True), allow_nan=False)

    def test_numpy_large_finite_budget_uses_float64_math_not_scalar_overflow(self):
        b, l = scene()
        for height, roi in ((np.float32(3.), np.float32(2e38)),
                            (np.int64(2 ** 32), np.int64(2 ** 32))):
            v = compute_volume(b, l, dy=0., **conditions(b, l, height=height, roi=roi))
            self.assertTrue(v.ok, v.text)
            self.assertAlmostEqual(24., v.volume_m3)
            self.assertEqual("conditional", v.range_status)
            self.assertEqual(0., v.bound_lo)
            self.assertGreater(v.bound_hi, 1e18)
            json.dumps(v.to_dict(raw=True), allow_nan=False)

    def test_display_rounding_cannot_exclude_raw_truth(self):
        b, l = scene(.123456789)
        v = compute_volume(b, l, dy=0., **conditions(b, l))
        truth = .123456789 * 24.
        data = v.to_dict()
        self.assertLessEqual(data["bound_lo"], truth)
        self.assertGreaterEqual(data["bound_hi"], truth)
        self.assertLessEqual(data["bound_lo"], data["volume_m3"])
        self.assertGreaterEqual(data["bound_hi"], data["volume_m3"])

    def test_range_budget_overflow_does_not_erase_legal_estimate(self):
        b, l = scene(observed_boundary=False)
        for error in (dict(surface=1e308, bed=1e308), dict(roi=1e308)):
            for function in (compute_volume, compute_volume_auto):
                v = function(b, l, dy=0., **conditions(b, l, **error))
                self.assertTrue(v.ok, v.text)
                self.assertAlmostEqual(2.3 * 9.9, v.volume_m3)  # 拒绝模型后按实测点域估计。
                self.assertEqual("rejected", v.range_status)
                self.assertIsNone(v.to_dict()["bound_hi"])

    def test_applied_reference_error_is_part_of_derived_budget_validation(self):
        b, l = scene(observed_boundary=False)
        ref = reference(b, l, 0., xy=(.25, .5), uncertainty=1e308)
        opt = conditions(b, l, domain=(0., .5, 0., 1.), surface=1e308, scope=ref["bed_scope"])
        v = compute_volume(b, l, dy=0., bed_shift_max_m=1.5e308, **ref, **opt)
        self.assertTrue(v.ok, v.text)
        self.assertEqual("applied", v.bed_reference_status)
        self.assertEqual("rejected", v.range_status)
        self.assertAlmostEqual(2.3 * 9.9, v.volume_m3)  # 拒绝模型后按实测点域估计。
        self.assertIsNone(v.to_dict()["bound_hi"])

    def test_unreached_blind_area_does_not_erase_observation_or_physical_range(self):
        b, l = scene(.5)
        l.cloud = l.cloud[l.cloud[:, 1] >= 3.]
        values = [compute_volume(b, l, dy=0., voxel=cell, **conditions(b, l)) for cell in (.01, .1)]
        for v in values:
            self.assertTrue(v.ok, v.text)
            self.assertAlmostEqual(8.4, v.bound_lo)
            self.assertAlmostEqual(30., v.bound_hi)
            self.assertGreaterEqual(v.volume_m3 + 1e-9, v.bound_lo)
            self.assertLessEqual(v.volume_m3, v.bound_hi)
            self.assertFalse(v.confidence()[0])
        self.assertGreater(values[0].unfilled_area_m2, 0.)
        self.assertEqual(0., values[1].unfilled_area_m2)


class TestVolumeGateSeparation(unittest.TestCase):
    """★ 估值可用性与范围宽度是两件事，必须分得开（2026-09-18）。

    实测两辆有磅单真值的车（`data/field/未知车牌` 的 09-15 那两对）：
    估值本身很好 —— 密度 0.994 / 0.981，范围也确实罩住了磅单换算的区间；
    但 `bound_spread` 是 **54.8% / 60.7%**，远超 `VOLUME_SPREAD_MAX=3%`。

    原因不在车，在几何：`integrate_context` 的上界要把**没观测到的面积**
    按 `max_height_m` 整个算进去，而俯视单雷达在车厢两端掠射，
    未观测面积占 34%~39%。扫遍 5 档格子 × 4 档货高上限 × 误差预算减半，
    spread 最低只能到 46% —— **这道门槛在现有几何下永远关着**，
    于是 `volumeStatus` 不可能成为 "success"。

    3% 这个值来自 docs/13 §17，当时 `bound_lo/hi` 还是"格子扫描的上下界"
    （自适应格子能压到 1.6%/1.8%）。换成声明模型区间之后没跟着改。

    该定多少是业务决定，所以这里**不动那个数**；要做的是让下游看得见
    卡的是哪一条，别去查车。
    """

    def _result(self, hole=0.05, spread=0.5, ok=True):
        from app.detect import VolumeResult
        v = VolumeResult()
        v.ok = ok
        v.volume_m3, v.voxel_size, v.coverage = 50.0, 0.10, 0.65
        v.wall_ratio, v.hole_ratio = 0.20, hole
        v.filled_ratio = v.wall_ratio + v.hole_ratio
        v.range_kind, v.range_status = "conditional_cell_mean", "conditional"
        # 区间必须**罩住** volume_m3，否则 range_validity_errors 会先报
        # "估计体积不在声明范围内" —— 那是另一条，别把它混进来。
        v.bound_hi = v.volume_m3 / (1.0 - spread / 2.0)
        v.bound_lo = v.bound_hi * (1.0 - spread)
        v.bound_spread = spread
        return v

    def test_wide_range_does_not_make_the_estimate_unusable(self):
        """★ 拆分的目标：范围宽不代表这车没扫好。

        ▲ 默认已经**不设**宽度门槛了（`VOLUME_SPREAD_MAX = None`，
          2026-09-18 现场决定），所以这里显式传一个门槛来验拆分本身。
        """
        v = self._result(hole=0.05, spread=0.55)
        self.assertTrue(v.estimate_ok()[0], v.estimate_ok()[1])
        self.assertFalse(v.range_ok(spread_max=0.03)[0])
        self.assertIn("上下界差", v.range_ok(spread_max=0.03)[1])
        # 默认（不设门槛）时它应当通过 —— 范围照报，只是不拦
        self.assertTrue(v.range_ok()[0], v.range_ok()[1])

    def test_bad_estimate_is_reported_on_the_estimate_side(self):
        v = self._result(hole=0.40, spread=0.0)
        self.assertFalse(v.estimate_ok()[0])
        self.assertIn("空洞", v.estimate_ok()[1])
        self.assertTrue(v.range_ok()[0], v.range_ok()[1])

    def test_confidence_is_the_conjunction_of_the_two_gates(self):
        """`confidence()` = `estimate_ok()` 且 `range_ok()`，显式给门槛时逐条一致。"""
        for hole, spread, want in ((0.05, 0.01, True), (0.40, 0.01, False),
                                   (0.05, 0.55, False), (0.40, 0.55, False)):
            v = self._result(hole=hole, spread=spread)
            self.assertEqual(want, v.confidence(spread_max=0.03)[0], (hole, spread))
            self.assertEqual(want, v.estimate_ok()[0] and v.range_ok(spread_max=0.03)[0],
                             (hole, spread))

    def test_spread_gate_is_off_by_default(self):
        """★ 2026-09-18 现场决定：范围宽度不再拦。

        「用户没时间看这个数字，最终会进入第三方系统，他们会去处理」——
        质量筛选放到下游。`bound_spread` 照算照报，只是不参与判定。
        """
        from app.detect import VOLUME_SPREAD_MAX
        self.assertIsNone(VOLUME_SPREAD_MAX)
        for spread in (0.03, 0.55, 0.99):
            v = self._result(hole=0.05, spread=spread)
            self.assertTrue(v.confidence()[0], (spread, v.confidence()[1]))
            self.assertEqual(spread, v.to_dict()["bound_spread"], "范围还是要报出去")
        # 空洞占比仍然拦得住 —— 不是全放开
        self.assertFalse(self._result(hole=0.40, spread=0.01).confidence()[0])

    def test_missing_range_fails_range_not_estimate(self):
        """没有声明模型时，估值该照样可用 —— 这正是从前混在一起的那条。"""
        v = self._result()
        v.range_kind, v.range_status = "unavailable", "missing"
        v.bound_lo = v.bound_hi = v.bound_spread = float("nan")
        v.range_reason = "缺少独立内腔、货高上限和支持误差条件"
        self.assertTrue(v.estimate_ok()[0], v.estimate_ok()[1])
        self.assertFalse(v.range_ok()[0])
        self.assertFalse(v.confidence()[0])

    def test_to_dict_exposes_both_gates(self):
        d = self._result(spread=0.55).to_dict()
        for key in ("low_confidence", "estimate_ok", "estimate_reason", "range_ok", "range_gate_reason"):
            self.assertIn(key, d)
        self.assertTrue(d["estimate_ok"])
        # 宽度门槛已关，所以范围这条也过；两个口径仍然各自并排报出来
        self.assertTrue(d["range_ok"])
        self.assertFalse(d["low_confidence"])
        bad = self._result(hole=0.40, spread=0.55).to_dict()
        self.assertTrue(bad["estimate_ok"])
        self.assertFalse(bad["low_confidence"])

    def test_publication_does_not_import_diagnostic_thresholds(self):
        """发布契约不再依赖内部诊断/档位选择的质量参考值。"""
        from app import load_radar_contract as contract
        self.assertFalse(hasattr(contract, "VOLUME_HOLE_MAX"))
        self.assertFalse(hasattr(contract, "VOLUME_SPREAD_MAX"))
        with open(contract.__file__, encoding="utf-8") as stream:
            source = stream.read()
        body = source[source.index("def build_measurement"):]
        gate = body[body.index('status = "unavailable"'):body.index("return {")]
        # 注释里可以提这两个数（说明它们的来历），代码里不许再写死。
        code = "\n".join(line.split("#", 1)[0] for line in gate.splitlines())
        self.assertNotIn("0.15", code, "契约里又写死了一份门槛数字")
        self.assertNotIn("0.03", code, "契约里又写死了一份门槛数字")
