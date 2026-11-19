"""原体积调用的只读复算；不加载当前配置、基线缓存或联网。"""
from dataclasses import asdict, replace
import json
import math

from .archive_io import ArchiveReadLimits, check_budget
from .measurement_runtime import (algorithm_identity, canonical, context_from_record,
                                  restore_inputs, scope_from_record)
from .volume_context import grid_geometry


def recompute(archive, *, limits=None):
    limits = limits or ArchiveReadLimits()
    meta = archive.metadata
    check_budget(meta, len(archive.bundle.manifest), limits)
    restored = restore_inputs(archive, limits)
    report = dict(schema="volume-replay-report-v1", measurement_id=meta["measurement_id"],
                  manifest_sha256=archive.manifest_sha256, status="unavailable",
                  matched=None, differences=[], standstill_timeline=False)
    if restored is None:
        return dict(report, reason="旧归档未保存原体积参数及基线资产")
    call, base, loaded, extras = restored
    if call["algorithm"]["contract"] != algorithm_identity()["contract"]:
        return dict(report, status="unsupported_algorithm", reason="原体积调用格式不受当前程序支持")
    if call["baseline"]["state"] == "asset_unavailable":
        return dict(report, reason="当时使用了基线，但归档缺少原资产；禁止读取当前缓存替代")
    params = dict(call["params"])
    context = params["volume_context"]
    # 在模型供应及算法分配网格前，另预留最大候选/固定支持的保守工作空间。
    grids = []
    registration_work = 0
    if base is not None:
        domain = context["domain_xy"] if context is not None else list(base.box[:4])
        if context is None and loaded is not None and len(loaded.cloud):
            from .detect import ALIGN_X_MARGIN_M
            cloud = loaded.cloud
            # 横向配准与偏航剪切会把基线盒沿 x 撑开至多 ALIGN_X_MARGIN_M，网格跟着扩。
            domain[0] = min(domain[0], float(cloud[:, 0].min()), base.box[0] - ALIGN_X_MARGIN_M)
            domain[1] = max(domain[1], float(cloud[:, 0].max()), base.box[1] + ALIGN_X_MARGIN_M)
            # dy=None 的当前受信合同限定估计偏移绝对值<=2.5m；不先分配直方图来预检。
            shifts = (-2.5, 2.5) if params["dy"] is None else (params["dy"], params["dy"])
            domain[2] = min(domain[2], float(cloud[:, 1].min()), base.box[2] + shifts[0])
            domain[3] = max(domain[3], float(cloud[:, 1].max()), base.box[3] + shifts[1])
            if params["dy"] is None:
                spans = [float(c[:, 1].max()) - float(c[:, 1].min()) for c in (base.cloud, cloud)]
                if any(not math.isfinite(span) or span / .25 > 4_000_000 for span in spans):
                    raise ValueError("自动配准范围超过复算预算")
                registration_work = 64 * (math.ceil(max(spans) / .25) + 1)
        origin = context["support_origin_xy"] if context is not None else (domain[0], domain[2])
        for cell in params["cells"] + ([context["support_cell_m"]] if context else []):
            axes = grid_geometry(domain, cell, origin)
            grids.append(axes[0][4] * axes[1][4])
    additional = max(grids, default=0) * 256 + registration_work
    if additional >= limits.working_bytes:
        raise ValueError("复算网格工作内存超过预算")
    check_budget(meta, len(archive.bundle.manifest), replace(limits, working_bytes=limits.working_bytes - additional))
    if context is not None:
        from .volume_provider import VolumeCatalog
        runtime = call["runtime"]
        catalog = VolumeCatalog.from_bytes(canonical(dict(schema="volume-provider-v1", models=[runtime["model"]])))
        model, _ = catalog.select(runtime["binding"], params["volume_scope"]["measured_at"])
        if model is None or model.sha256 != runtime["model_sha256"]:
            raise ValueError("原物理模型绑定、有效期或摘要不一致")
        supplied = model.supply(scope=scope_from_record(params["volume_scope"]), baseline=base,
            surfaces=(loaded.cloud,) + tuple(extras), dy=params["dy"],
            registration_status=runtime["registration"]["status"])
        if canonical(asdict(supplied)) != canonical(context):
            raise ValueError("原物理模型不能产生存档中的实际体积条件")
        params["volume_context"] = context_from_record(context)
        params["volume_scope"] = scope_from_record(params["volume_scope"])
    params.pop("fill_holes")  # compute_volume_auto 合同固定内部传 True。
    params["loaded_extra"] = extras or None
    # v2/v3：进/离场帧按原字节重新配准，记录必须逐项复现，才把配准结果交给积分。
    from .measurement_runtime import restore_enter_frames, restore_exit_frames
    extra = []
    enter = restore_enter_frames(archive, call)
    if enter:
        from .enter_volume import register_for_volume
        clouds, record = register_for_volume(loaded, enter)
        if canonical(record) != canonical(call["enter"]):
            return dict(report, status="recomputed", matched=False,
                        differences=[dict(field="/enter", original=call["enter"], recomputed=record)],
                        reason="进场帧重新配准与原记录不一致")
        extra.extend(clouds or ())
    exit_frames = restore_exit_frames(archive, call)
    if exit_frames:
        from .enter_volume import register_exit_for_volume
        clouds, record = register_exit_for_volume(loaded, exit_frames)
        if canonical(record) != canonical(call["exit"]):
            return dict(report, status="recomputed", matched=False,
                        differences=[dict(field="/exit", original=call["exit"], recomputed=record)],
                        reason="离场帧重新配准与原记录不一致")
        extra.extend(clouds or ())
    if extra:
        params["enter_extra"] = tuple(extra)
    from .detect import compute_volume_auto
    current = json.loads(canonical(compute_volume_auto(base, loaded, **params).to_dict(raw=True)))
    old = call["original_result"]
    differences = []
    def compare(a, b, path=""):
        if isinstance(a, dict) and isinstance(b, dict):
            for key in sorted(set(a) | set(b)):
                # 展示措辞不作为数值重现标准。
                if not path and key in ("text", "message"):
                    continue
                compare(a.get(key), b.get(key), path + "/" + key)
        elif type(a) in (int, float) and type(b) in (int, float):
            if not math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9):
                differences.append(dict(field=path, original=a, recomputed=b))
        elif type(a) is not type(b) or a != b:
            differences.append(dict(field=path, original=a, recomputed=b))
    compare(old, current)
    return dict(report, status="recomputed", matched=not differences, differences=differences,
                original=old, recomputed=current, numeric_tolerance=dict(relative=1e-9, absolute=1e-9),
                original_environment=call["algorithm"], current_environment=algorithm_identity(),
                reason="只复算原体积调用；物理条件真实性与完整停稳事件流不在此证明范围")
