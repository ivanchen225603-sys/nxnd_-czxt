"""固定支持块的条件体积范围；模型声明及输入绑定不等于现场来源认证。"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

from .bed_reference import (BedReferenceScope, BedShiftDecision, _finite, _label,
                            _valid_scope, baseline_digest, surface_digest, valid_surface_scope)
from .height_fill import fill_height_holes


@dataclass(frozen=True)
class VolumeContext:
    scope: BedReferenceScope
    reference_id: str
    baseline_sha256: str
    surface_sha256: str
    domain_xy: Tuple[float, float, float, float]
    domain_source: str
    max_height_m: float
    height_source: str
    conditions_status: str
    support_cell_m: float
    support_origin_xy: Tuple[float, float]
    support_model: str
    error_source: str
    surface_error_m: float
    bed_error_m: float
    roi_error_area_m2: float
    registration_dy_m: float
    valid_from: float
    valid_until: float


def context_error(context, expected_scope, baseline, surfaces, dy, bed_uncertainty_m=0.) -> Optional[str]:
    """调用方另供期望身份；不能让证据自己选择所适用的车次/输入。"""
    if context is None:
        return "缺少独立内腔、货高上限和支持误差条件"
    if not isinstance(context, VolumeContext):
        return "体积条件类型非法"
    if not _valid_scope(expected_scope) or not _valid_scope(context.scope) or context.scope != expected_scope:
        return "体积条件与当前版本、配置、车次、帧、坐标域或时间不匹配"
    if not valid_surface_scope(context.scope, len(surfaces)):
        return "支持帧身份数量与有序货面不匹配"
    if (not _label(context.reference_id) or
            not isinstance(context.domain_source, str) or
            context.domain_source not in ("synthetic_model", "surveyed_interior") or
            not isinstance(context.height_source, str) or
            context.height_source not in ("synthetic_model", "independent_vehicle_envelope") or
            not isinstance(context.conditions_status, str) or
            context.conditions_status not in ("declared_model", "externally_supplied") or
            not isinstance(context.support_model, str) or
            context.support_model != "cell_mean_max_relative_height" or
            not isinstance(context.error_source, str) or
            context.error_source not in ("synthetic_model", "independent_error_budget")):
        return "来源或固定支持模型未明确声明"
    if (not isinstance(context.domain_xy, tuple) or len(context.domain_xy) != 4 or
            not all(_finite(v) for v in context.domain_xy) or
            not context.domain_xy[0] < context.domain_xy[1] or
            not context.domain_xy[2] < context.domain_xy[3] or
            not isinstance(context.support_origin_xy, tuple) or len(context.support_origin_xy) != 2 or
            not all(_finite(v) for v in context.support_origin_xy)):
        return "内腔域或固定支持原点非法"
    if (not all(_finite(v) and v > 0 for v in (context.max_height_m, context.support_cell_m)) or
            not all(_finite(v) and v >= 0 for v in
                    (context.surface_error_m, context.bed_error_m, context.roi_error_area_m2))):
        return "货高上限、支持尺度或误差预算非法"
    # 派生预算也须有限；范围预算溢出不能连带抹掉可计算的诊断估值。
    x0, x1, y0, y1 = map(float, context.domain_xy)
    hmax = float(context.max_height_m)
    area = (x1 - x0) * (y1 - y0)
    budget = float(context.surface_error_m) + float(context.bed_error_m) + float(bed_uncertainty_m)
    if not all(math.isfinite(v) for v in (area, budget, area * hmax,
                area * budget, hmax * float(context.roi_error_area_m2),
                hmax * (area + float(context.roi_error_area_m2)))):
        return "条件域或派生误差预算非有限"
    if (not _finite(context.registration_dy_m) or
            not math.isclose(context.registration_dy_m, dy, rel_tol=0., abs_tol=1e-9)):
        return "配准偏移与条件不匹配"
    if (not all(_finite(v) for v in (context.valid_from, context.valid_until)) or
            not 0 < context.valid_from <= expected_scope.measured_at <= context.valid_until):
        return "体积条件不在有效时间内"
    if (not isinstance(context.baseline_sha256, str) or not isinstance(context.surface_sha256, str) or
            context.baseline_sha256 != baseline_digest(baseline) or
            context.surface_sha256 != surface_digest(surfaces)):
        return "实际基线或货面内容与体积条件不匹配"
    try:
        _grid(context.domain_xy, context.support_cell_m, context.support_origin_xy)
    except (ValueError, OverflowError, FloatingPointError):
        return "固定支持网格非法、数值不可分辨或过大"
    return None


def grid_geometry(domain, cell, origin):
    """无数组分配的网格预检，供运行时与显式配置共用同一边界。"""
    domain, cell, origin = tuple(map(float, domain)), float(cell), tuple(map(float, origin))
    area = (domain[1] - domain[0]) * (domain[3] - domain[2])
    if not math.isfinite(area) or area <= 0 or not math.isfinite(cell) or cell <= 0:
        raise ValueError("网格面积或尺度非有限、为空或下溢")
    # 非正规浮点面积无法稳定保持分格面积的可加性；不把下溢格当作无观测。
    if area < np.finfo(np.float64).tiny or cell * cell < np.finfo(np.float64).tiny:
        raise ValueError("网格面积或单格面积低于可计算尺度")
    axes = []
    for lo, hi, offset in ((domain[0], domain[1], origin[0]), (domain[2], domain[3], origin[1])):
        indices = ((lo - offset) / cell, (hi - offset) / cell)
        if (not all(math.isfinite(v) and abs(v) < 2 ** 40 for v in indices) or
                cell <= 8 * abs(float(np.spacing(max(abs(lo), abs(hi), abs(offset)))))):
            raise ValueError("坐标与尺度不可分辨")
        first, last = math.floor(indices[0]), math.ceil(indices[1])
        count = last - first
        if not 0 < count <= 4_000_000:
            raise ValueError("网格轴过大")
        axes.append((lo, hi, offset, first, count))
    if axes[0][4] * axes[1][4] > 4_000_000:
        raise ValueError("网格过大")
    return axes


def _grid(domain, cell, origin):
    """格与域的交集面积；先限制尺寸，不能让坏尺度触发巨型分配。"""
    axes = grid_geometry(domain, cell, origin)
    cell = float(cell)
    centers, lengths, starts = [], [], []
    for lo, hi, offset, first, count in axes:
        start = offset + first * cell
        edges = start + np.arange(count + 1) * cell
        left, right = np.maximum(edges[:-1], lo), np.minimum(edges[1:], hi)
        lengths.append(np.maximum(0., right - left))
        centers.append(left + (right - left) * .5)
        starts.append(start)
    area = lengths[1][:, None] * lengths[0][None, :]
    return np.meshgrid(*centers), area, starts


def _relative_map(cloud, baseline, domain, cell, starts, shape, dy, shift):
    # 域外点不能替跨界格的域内面积授信；半开域保证每点至多一个归属。
    x0, x1, y0, y1 = domain
    pts = cloud[(cloud[:, 0] >= x0) & (cloud[:, 0] < x1) &
                (cloud[:, 1] >= y0) & (cloud[:, 1] < y1)]
    ix = np.floor((pts[:, 0] - starts[0]) / cell).astype(np.int64)
    iy = np.floor((pts[:, 1] - starts[1]) / cell).astype(np.int64)
    keep = (ix >= 0) & (ix < shape[1]) & (iy >= 0) & (iy < shape[0])
    pts, ix, iy = pts[keep], ix[keep], iy[keep]
    # 斜底必须逐点配对，不能max(z)后减格心底板。
    height = pts[:, 2] - baseline.bed_z_at(pts[:, 0], pts[:, 1] - dy) - shift
    result = np.full(shape[0] * shape[1], -np.inf)
    np.maximum.at(result, iy * shape[1] + ix, height)
    return result.reshape(shape)


def integrate_context(context: VolumeContext, baseline, surfaces: Sequence[np.ndarray],
                      dy: float, decision: BedShiftDecision, voxel: float, fill: bool):
    """在显式模型下积分；来源真实性/误差预算真实性由外部供应者负责。

    surface_error必须界定格与域交集上的平均货高代表性误差，含融合/采样/
    表面变化。有限点与所谓传感器精度本身不能证明这个误差。
    """
    domain, cell = tuple(map(float, context.domain_xy)), float(context.support_cell_m)
    (xx, yy), area, starts = _grid(domain, cell, context.support_origin_xy)
    shape = area.shape
    bc = np.asarray(baseline.cloud, np.float64).copy()
    bc[:, 1] += dy
    wall = _relative_map(bc, baseline, domain, cell, starts, shape, dy, 0.) > .25
    heights = np.full(shape, -np.inf)
    for surface in surfaces:
        heights = np.maximum(heights, _relative_map(surface, baseline, domain, cell, starts,
                                                   shape, dy, decision.shift_m))
    observed = np.isfinite(heights) & ~wall & (area > 0)
    unknown = ~observed & (area > 0)
    domain_area = float(area.sum())
    observed_area = float(area[observed].sum())
    unknown_area = float(area[unknown].sum())
    data = dict(range_kind="unavailable", range_status="no_observation",
                range_reason="固定积分域没有有效货面观测", domain_area_m2=domain_area,
                observed_area_m2=observed_area, unknown_area_m2=unknown_area,
                support_cell_m=cell, range_reference_id=context.reference_id,
                range_conditions=dict(scope=asdict(context.scope), valid_from=context.valid_from,
                    valid_until=context.valid_until, domain_xy=list(domain), domain_source=context.domain_source,
                    max_height_m=context.max_height_m, height_source=context.height_source,
                    conditions_status=context.conditions_status, support_model=context.support_model,
                    error_source=context.error_source,
                    support_origin_xy=list(context.support_origin_xy), surface_error_m=context.surface_error_m,
                    bed_error_m=context.bed_error_m, roi_error_area_m2=context.roi_error_area_m2,
                    coordinate_frame=context.scope.coordinate_frame, registration_dy_m=dy,
                    baseline_sha256=context.baseline_sha256, surface_sha256=context.surface_sha256),
                range_assumptions=("实心散料高度柱模型", "独立内腔与货高上限条件成立",
                    "固定支持块与域交集的平均货高误差受声明预算约束",
                    "误差覆盖配准、底板模型、采样代表性及多帧时变",
                    "上限覆盖ROI对称差可能增减区域", "来源声明不是现场认证"))
    def json_numbers(value):
        if isinstance(value, dict):
            return {key: json_numbers(item) for key, item in value.items()}
        if isinstance(value, (tuple, list)):
            return [json_numbers(item) for item in value]
        if isinstance(value, np.floating):
            return float(value)
        if isinstance(value, np.integer):
            return int(value)
        return value
    data["range_conditions"] = json_numbers(data["range_conditions"])
    if not observed.any():
        return data
    hmax = float(context.max_height_m)
    shift_error = decision.uncertainty_m if decision.status == "applied" else 0.
    error = float(context.surface_error_m) + float(context.bed_error_m) + float(shift_error)
    h = heights[observed]
    conflict = (decision.status == "rejected" or
                np.any(h - error > hmax) or np.any(h + error < 0))
    # 仅诊断估值可截到非负；条件冲突不以clip消失，不给范围。
    values = np.zeros(shape)
    values[observed] = np.maximum(h, 0.) if conflict else np.clip(h, 0., hmax)
    unfilled = np.zeros(shape, dtype=bool)
    if fill and unknown.any():
        (_, _), coarse_area, coarse_start = _grid(domain, voxel, (domain[0], domain[2]))
        ix = np.floor((xx - coarse_start[0]) / voxel).astype(np.int64)
        iy = np.floor((yy - coarse_start[1]) / voxel).astype(np.int64)
        ix, iy = np.minimum(ix, coarse_area.shape[1] - 1), np.minimum(iy, coarse_area.shape[0] - 1)
        flat = iy * coarse_area.shape[1] + ix
        weights = np.bincount(flat[observed], weights=area[observed], minlength=coarse_area.size)
        sums = np.bincount(flat[observed], weights=(area * values)[observed], minlength=coarse_area.size)
        coarse = np.full(coarse_area.size, np.nan)
        known = weights > 0
        coarse[known] = sums[known] / weights[known]
        coarse = fill_height_holes(coarse.reshape(coarse_area.shape), domain=coarse_area > 0)
        # 格心归属只用于未知值的插值，不声称守恒重栅格，更不改变观测支持。
        values[unknown] = coarse.ravel()[flat[unknown]]
        # 有界迭代未抵达的盲区仍未知；显式以零作诊断估值，不抹掉独立范围。
        unfilled = unknown & ~np.isfinite(values)
        values[unfilled] = 0.
        if not conflict:
            values[unknown] = np.clip(values[unknown], 0., hmax)
    if not np.isfinite(values).all():
        raise FloatingPointError("插值未得到有限估值")
    volume = float((values * area).sum())
    fill_mask = unknown & ~unfilled if fill else np.zeros(shape, dtype=bool)
    # 两侧有锚点的轴向内插与外推仅作可解释诊断，不是置信证明。
    inside = (((np.cumsum(observed, axis=0) > 0) & (np.cumsum(observed[::-1], axis=0)[::-1] > 0)) |
              ((np.cumsum(observed, axis=1) > 0) & (np.cumsum(observed[:, ::-1], axis=1)[:, ::-1] > 0)))
    def contribution(mask):
        return float((values[mask] * area[mask]).sum())
    filled_volume = contribution(fill_mask)
    data.update(volume_m3=volume, occupied_voxels=int(np.count_nonzero(values > 0)),
                coverage=observed_area / domain_area, frames_used=len(surfaces),
                filled_cells=int(fill_mask.sum()), filled_ratio=filled_volume / volume if volume > 0 else 0.,
                hole_cells=int((fill_mask & ~wall).sum()),
                hole_ratio=contribution(fill_mask & ~wall) / volume if volume > 0 else 0.,
                wall_cells=int((fill_mask & wall).sum()),
                wall_ratio=contribution(fill_mask & wall) / volume if volume > 0 else 0.,
                measured_volume_m3=contribution(observed), interpolated_volume_m3=contribution(fill_mask & inside),
                extrapolated_volume_m3=contribution(fill_mask & ~inside),
                unfilled_area_m2=float(area[unfilled if fill else unknown].sum()))
    data["range_conditions"]["unfilled_estimate_policy"] = "zero_diagnostic_only_not_empty_observation"
    data["range_conditions"]["applied_bed_shift_error_m"] = shift_error
    if conflict:
        data.update(range_status="condition_conflict", range_reason="观测高度或底板参考与声明条件冲突")
    else:
        allowance = hmax * float(context.roi_error_area_m2)
        lower = max(0., float((area[observed] * np.maximum(0., h - error)).sum()) - allowance)
        upper = float((area[observed] * np.minimum(hmax, h + error)).sum()) + unknown_area * hmax + allowance
        if not all(math.isfinite(v) for v in (volume, lower, upper, error)):
            raise FloatingPointError("条件积分结果非有限")
        data.update(bound_lo=lower, bound_hi=upper, bound_spread=(upper - lower) / upper if upper > 0 else 0.,
                    range_kind="conditional_cell_mean", range_status="conditional",
                    range_reason="仅在归档声明的独立几何与支持误差条件成立时有效")
    return data
