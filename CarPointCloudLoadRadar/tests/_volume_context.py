"""测试专用解析条件；不可作为运行时证据供应者。"""
from app.bed_reference import BedReferenceScope, baseline_digest, surface_digest
from app.volume_context import VolumeContext


def conditions(base, loaded, domain=(0., 2.4, 0., 10.), height=3., cell=.1,
               origin=(0., 0.), extra=(), dy=0., scope=None, **errors):
    clouds = [loaded.cloud] + list(extra)
    if scope is None:
        scope = BedReferenceScope("test-baseline", "cal-1", "a" * 64, "CH1", "passage-1", "epoch-1",
            tuple((1, 10 + i, 1000. + i) for i in range(len(clouds))), "loaded-calibrated-m", 999. + len(clouds))
    context = VolumeContext(scope=scope, reference_id="analytic-range", baseline_sha256=baseline_digest(base),
        surface_sha256=surface_digest(clouds), domain_xy=domain, domain_source="synthetic_model",
        max_height_m=height, height_source="synthetic_model", conditions_status="declared_model",
        support_cell_m=cell, support_origin_xy=origin, support_model="cell_mean_max_relative_height",
        error_source="synthetic_model", surface_error_m=errors.get("surface", 0.),
        bed_error_m=errors.get("bed", 0.), roi_error_area_m2=errors.get("roi", 0.),
        registration_dy_m=dy, valid_from=999., valid_until=1001. + len(clouds))
    return dict(volume_context=context, volume_scope=scope)
