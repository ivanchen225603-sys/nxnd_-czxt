"""测试专用独立参考条件；不是运行时来源或现场真实性证明。"""
from app.bed_reference import BedReferenceEvidence, BedReferenceScope, baseline_digest, surface_digest


def reference(base, loaded, bed_z, xy=(0., 0.), extra=(), uncertainty=0.):
    scope = BedReferenceScope(baseline_version_id="test-baseline", calibration_id="cal-1",
        configuration_sha256="a" * 64, channel_id="CH1", passage_id="passage-1", capture_epoch="epoch-1",
        frame_ids=((1, 10, 1000.),), coordinate_frame="loaded-calibrated-m", measured_at=1000.)
    evidence = BedReferenceEvidence(source="independent_exposed_bed", reference_id="analytic-truth",
        scope=scope, baseline_sha256=baseline_digest(base), surface_sha256=surface_digest([loaded.cloud] + list(extra)),
        confirmed_exposed=True, reference_xy=xy, bed_z=bed_z, uncertainty_m=uncertainty,
        valid_from=999., valid_until=1001.)
    return dict(bed_z=bed_z, bed_evidence=evidence, bed_scope=scope)
