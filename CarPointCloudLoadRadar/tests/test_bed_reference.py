"""底板独立证据的绑定、条件修正及真实Worker接线；不证明现场参考真实性。"""
from dataclasses import FrozenInstanceError, replace
import json
import unittest
from unittest.mock import patch

import numpy as np

from app.bed_reference import baseline_digest, surface_digest
from app.detect import LoadedResult, analyze_load, compute_volume, compute_volume_auto
from tests._bed_reference import reference
from tests._volume_context import conditions
from tests.test_volume_validity import scene


class TestBedReference(unittest.TestCase):
    def case(self, height=1.2, shift=.2, **kwargs):
        base, loaded = scene(height)
        return base, loaded, reference(base, loaded, shift, xy=(1.2, 5.), **kwargs)

    def test_thin_layer_profile_cannot_authorize_its_own_floor(self):
        base, loaded = scene(.4)
        profile = analyze_load(loaded.cloud)
        self.assertTrue(profile.ok)
        self.assertAlmostEqual(.4, profile.bed_z)
        for function in (compute_volume, compute_volume_auto):
            result = function(base, loaded, dy=0., bed_z=profile.bed_z)
            self.assertTrue(result.ok, result.text)
            self.assertAlmostEqual(9.6, result.volume_m3)
            self.assertEqual(0., result.bed_shift_applied_m)
            self.assertEqual("unverified", result.bed_reference_status)

    def test_independent_reference_keeps_analytic_volume_and_reports_source(self):
        base, loaded, options = self.case()
        raw = compute_volume(base, loaded, dy=0.)
        result = compute_volume(base, loaded, dy=0., **options,
                                **conditions(base, loaded, scope=options["bed_scope"]))
        self.assertTrue(result.ok, result.text)
        self.assertAlmostEqual(28.8, raw.volume_m3)
        self.assertAlmostEqual(24., result.volume_m3)
        self.assertAlmostEqual(.2, result.bed_shift_applied_m)
        self.assertEqual("applied", result.bed_reference_status)
        self.assertEqual("analytic-truth", result.bed_reference_id)
        self.assertTrue(result.confidence()[0], result.confidence()[1])
        for raw in (True, False):
            data = result.to_dict(raw=raw)
            self.assertEqual("independent_exposed_bed", data["bed_reference_source"])
            self.assertEqual(0., data["bed_reference_uncertainty_m"])
            json.dumps(data, allow_nan=False)

    def test_reference_error_is_not_silently_treated_as_zero(self):
        base, loaded, options = self.case(uncertainty=.01)
        for function in (compute_volume, compute_volume_auto):
            result = function(base, loaded, dy=0., **options)
            self.assertTrue(result.ok)
            self.assertAlmostEqual(24., result.volume_m3)
            self.assertEqual(.01, result.bed_reference_uncertainty_m)
            self.assertFalse(result.confidence()[0])
            self.assertFalse(result.to_dict()["low_confidence"])

    def test_corrected_complete_empty_keeps_zero_and_reference_quality(self):
        for uncertainty in (0., .01):
            base, loaded, options = self.case(height=.2, uncertainty=uncertainty)
            for function in (compute_volume, compute_volume_auto):
                with self.subTest(uncertainty=uncertainty, function=function.__name__):
                    result = function(base, loaded, dy=0., **options,
                                      **conditions(base, loaded, scope=options["bed_scope"]))
                    self.assertTrue(result.ok, result.text)
                    self.assertEqual(0., result.volume_m3)
                    self.assertEqual(0., result.to_dict()["volume_m3"])
                    self.assertEqual("applied", result.bed_reference_status)
                    # 宽度门槛默认已关（2026-09-18 现场决定：质量筛选放到下游
                    # 第三方系统）。显式给门槛来钉本条真正要钉的东西 ——
                    # 底板参考的不确定度确实被算进了区间。
                    self.assertEqual(uncertainty == 0., result.confidence(spread_max=0.03)[0])
                    self.assertAlmostEqual(0., result.bound_lo)
                    self.assertAlmostEqual(24. * uncertainty, result.bound_hi)

    def test_shift_plus_uncertainty_respects_existing_limit(self):
        for shift, uncertainty, accepted in ((.49, .005, True), (.49, .02, False),
                                             (.51, 0., False), (-.49, .02, False)):
            base, loaded, options = self.case(shift=shift, uncertainty=uncertainty)
            result = compute_volume(base, loaded, dy=0., **options)
            self.assertEqual("applied" if accepted else "rejected", result.bed_reference_status)
            self.assertAlmostEqual(shift if accepted else 0., result.bed_shift_applied_m)

    def test_wrong_scope_cannot_reuse_reference(self):
        base, loaded, options = self.case()
        variants = dict(baseline_version_id="other", calibration_id="other", configuration_sha256="b" * 64,
            channel_id="CH2", passage_id="other", capture_epoch="new-epoch", frame_ids=((2, 10, 1000.),),
            coordinate_frame="sensor-raw-m", measured_at=1000.1)
        for key, value in variants.items():
            with self.subTest(key=key):
                result = compute_volume(base, loaded, dy=0., **dict(options,
                    bed_scope=replace(options["bed_scope"], **{key: value})))
                self.assertTrue(result.ok)
                self.assertEqual("rejected", result.bed_reference_status)
                self.assertEqual(0., result.bed_shift_applied_m)
                self.assertFalse(result.confidence()[0])

    def test_missing_expected_context_cannot_trust_evidence_context(self):
        base, loaded, options = self.case()
        for scope in (None, {}, options["bed_evidence"]):
            result = compute_volume(base, loaded, dy=0., **dict(options, bed_scope=scope))
            self.assertEqual("rejected", result.bed_reference_status)
            self.assertEqual(0., result.bed_shift_applied_m)

    def test_invalid_scope_even_if_both_copies_match_is_rejected(self):
        base, loaded, options = self.case()
        for change in (dict(configuration_sha256=""), dict(channel_id=""), dict(frame_ids=()),
                       dict(frame_ids=((1, 0, 1000.),)),
                       dict(frame_ids=((True, 10, 1000.),)), dict(frame_ids=((1, 10, True),)),
                       dict(frame_ids=((1, 10, 999.), (1, 10, 1000.))),
                       dict(frame_ids=((1, 9, 999.), (2, 10, 1000.))),
                       dict(frame_ids=((1, 10, 999.),)), dict(measured_at=float("nan"))):
            scope = replace(options["bed_scope"], **change)
            evidence = replace(options["bed_evidence"], scope=scope)
            result = compute_volume(base, loaded, dy=0., **dict(options, bed_scope=scope, bed_evidence=evidence))
            self.assertEqual("rejected", result.bed_reference_status)

    def test_bad_evidence_preserves_original_estimate_but_cannot_be_confident(self):
        base, loaded, options = self.case()
        bad = [dict(source="analyze_load"), dict(source="unknown"), dict(source=np.array([1, 2])),
            dict(reference_id=""), dict(reference_id="bad\nref"), dict(confirmed_exposed=1),
            dict(confirmed_exposed=False), dict(valid_from=1000.1), dict(valid_until=999.9),
            dict(valid_from=float("nan")), dict(valid_until=float("inf")), dict(bed_z=.201),
            dict(bed_z=True), dict(bed_z=float("nan")), dict(uncertainty_m=-.01),
            dict(uncertainty_m=True), dict(uncertainty_m=float("inf")),
            dict(reference_xy=(30., 5.)), dict(reference_xy=(1., float("inf"))),
            dict(reference_xy=[1., 5.]), dict(baseline_sha256="0" * 64), dict(surface_sha256="0" * 64)]
        expected = compute_volume(base, loaded, dy=0.).volume_m3
        for change in bad:
            with self.subTest(change=change):
                result = compute_volume(base, loaded, dy=0., **dict(options,
                    bed_evidence=replace(options["bed_evidence"], **change)))
                self.assertTrue(result.ok, result.text)
                self.assertEqual(expected, result.volume_m3)
                self.assertEqual("rejected", result.bed_reference_status)
                self.assertFalse(result.confidence()[0])
        self.assertEqual("rejected", compute_volume(base, loaded, dy=0., **dict(options, bed_evidence={})).bed_reference_status)

    def test_actual_baseline_and_surface_mutation_detected_not_just_ids(self):
        for owner in ("base_cloud", "loaded", "floor", "box", "plane", "extra"):
            base, loaded = scene(1.2)
            extra = loaded.cloud.copy()
            options = reference(base, loaded, .2, xy=(1.2, 5.), extra=[extra])
            if owner == "base_cloud":
                base.cloud[0, 2] += .01
            elif owner == "loaded":
                loaded.cloud[0, 2] += .01
            elif owner == "floor":
                base.floor_z += .01
            elif owner == "box":
                base.box = base.box[:5] + (3.1,)
            elif owner == "plane":
                base.bed_plane = (0., 0., 1., -.01)
            else:
                extra[0, 2] += .01
            with self.subTest(owner=owner):
                result = compute_volume(base, loaded, dy=0., loaded_extra=[extra], **options)
                self.assertTrue(result.ok, result.text)
                self.assertEqual("rejected", result.bed_reference_status)
                self.assertEqual(0., result.bed_shift_applied_m)

    def test_evidence_and_scope_are_immutable_and_hashes_are_canonical(self):
        base, loaded, options = self.case()
        with self.assertRaises(FrozenInstanceError):
            options["bed_evidence"].bed_z = .3
        with self.assertRaises(FrozenInstanceError):
            options["bed_scope"].passage_id = "other"
        self.assertEqual(surface_digest([loaded.cloud]), surface_digest([loaded.cloud.astype(">f8")]))
        self.assertNotEqual(surface_digest([loaded.cloud]), surface_digest([loaded.cloud, loaded.cloud]))
        old = baseline_digest(base)
        base.cloud[0, 2] += .001
        self.assertNotEqual(old, baseline_digest(base))

    def test_no_evidence_does_not_hash_large_inputs(self):
        with patch("app.bed_reference.baseline_digest", side_effect=AssertionError("无证据无需摘要")), \
                patch("app.bed_reference.surface_digest", side_effect=AssertionError("无证据无需摘要")):
            self.assertTrue(compute_volume(*scene(), dy=0., bed_z=.2).ok)

    def test_reference_point_uses_shifted_plane_not_grid_median(self):
        base, loaded = scene()
        base.cloud[:, 2] = .04 * base.cloud[:, 1]
        base.bed_plane = (0., -.04, 1., 0.)
        loaded.cloud[:, 1] += .7
        loaded.cloud[:, 2] = .04 * (loaded.cloud[:, 1] - .7) + 1.2
        height_at_reference = .04 * (3. - .7) + .2
        options = reference(base, loaded, height_at_reference, xy=(1.2, 3.))
        for cell in (.1, .15, .2, .25, .3):
            result = compute_volume(base, loaded, voxel=cell, dy=.7, **options)
            self.assertTrue(result.ok, result.text)
            self.assertEqual("applied", result.bed_reference_status)
            self.assertAlmostEqual(.2, result.bed_shift_applied_m, places=12)


class TestWorkerBedReference(unittest.TestCase):
    def test_profile_ok_does_not_generate_evidence_or_erase_thin_coal(self):
        from tests.test_volume_runtime import TestVolumeRuntime
        harness = TestVolumeRuntime()
        harness.setUp()
        try:
            worker, bridge, base, frame, _ = harness.arm(original_asset=True, height=.04)
            profile = analyze_load(frame.xyz)
            self.assertTrue(profile.ok)
            # 正式快照、真实货面和体积供应全链路；仅固定双平面诊断，不能自造底板授权。
            with patch("app.application.channel_worker.analyze_load", return_value=profile), \
                    patch("app.application.channel_worker.compute_volume_auto", wraps=compute_volume_auto) as compute:
                result = harness.finish(worker, frame)
            self.assertIsNone(compute.call_args.kwargs["bed_evidence"])
            self.assertIsNone(compute.call_args.kwargs["bed_scope"])
            self.assertEqual(profile.bed_z, compute.call_args.kwargs["bed_z"])
            self.assertTrue(result["volume"]["ok"])
            self.assertAlmostEqual(.96, result["volume"]["volume_m3"], places=5)
            self.assertEqual(0., result["volume"]["bed_shift_applied_m"])
            self.assertEqual("unverified", result["volume"]["bed_reference_status"])
            payload = bridge.outbox.get(result["measurement_id"]).payload
            self.assertAlmostEqual(.96, payload["volumeM3"], places=5)
            bundle = bridge.outbox.archives.get(result["measurement_id"])
            saved = json.loads(dict(bundle.files)["detection.xyz.meta.json"])
            self.assertEqual(result["volume"], saved["volume"])
        finally:
            self.assertTrue(harness.doCleanups())
