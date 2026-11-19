"""短矩形车型的实测顶部点带与对称开口推算；不把折边解释成全高墙厚。

阈值与 :mod:`carpointcloud_analyzer.front_lip` 同理，收敛为本模块常量：它们定义的是
"什么样的点带才算观测到了内缘"，属于方法本身，不是现场可调参数。
"""

from copy import deepcopy
from dataclasses import replace

import numpy as np

from carpointcloud_analyzer.domain.box_geometry import rotate_wl
from carpointcloud_analyzer.domain.front_lip import complete_front_lip
from carpointcloud_analyzer.domain.models import BoxCompletionResult

# 重采样间距，与 ``Config.carriage_box_surface_spacing`` 默认值一致。
# 此处不穿 Config：本函数是参考建模的下游，改间距只影响展示副本的点密度。
SURFACE_SPACING_M = 0.04

# --- 车头朝向判定：端面外侧多出来的点（车头/底盘）作为朝向证据 ---
ORIENTATION_MARGIN_M = 1.0
MIN_ORIENTATION_POINTS = 500
ORIENTATION_DOMINANCE = 3

# --- 前后端内伸量一致性 ---
# 两端都观测到内缘时必须互相印证；缺测才使用前后等宽规则。
MAX_END_INSET_DISAGREEMENT_M = 0.05

# --- 侧壁内缘取点带 ---
# 沿长度均分的采样边界数；两端各留 0.6m 避开与端壁的转角。
SIDE_LENGTH_BIN_EDGES = 11
SIDE_LENGTH_MARGIN_M = 0.6
# 竖直带：上沿以下 0.18m 到上沿以上 0.05m。
SIDE_BAND_BELOW_TOP_M = 0.18
SIDE_BAND_ABOVE_TOP_M = 0.05
# 横向搜索范围，负值容许壁面外侧的测距噪声。
SIDE_DEPTH_MIN_M = -0.15
SIDE_DEPTH_MAX_M = 0.5
# 支撑充分性：总点数、有效分段数、每段最少点数。
MIN_SIDE_POINTS = 50
MIN_SIDE_COVERED_BINS = 7
MIN_POINTS_PER_BIN = 5
# 合法内伸量范围与分段间的最大离散度。
MIN_SIDE_INSET_M = 0.06
MAX_SIDE_INSET_M = 0.30
MAX_SIDE_INSET_SPREAD_M = 0.06
# 左右两侧都成立时的最大互差。
MAX_SIDE_INSET_DISAGREEMENT_M = 0.04


def infer_symmetric_opening(points: np.ndarray, reference: BoxCompletionResult) -> dict:
    """已识别车型后运行。数据不足/两侧矛盾不套对称值，不消费任何合成点。"""
    meta, box = reference.metadata, reference.box
    result = {"applied": False, "reason": "profile_not_matched", "syntheticPointsUsed": 0,
              "scope": "top_opening_not_full_height_inner_walls"}
    if box is None or not meta.get("profileMatched"):
        return result
    p = rotate_wl(points, -np.deg2rad(meta["yawDeg"]))
    if not np.isfinite(p).all():
        return {**result, "reason": "nonfinite_measurement_points"}
    low = int(np.sum(p[:, 1] < box.rear_l - ORIENTATION_MARGIN_M))
    high = int(np.sum(p[:, 1] > box.front_l + ORIENTATION_MARGIN_M))
    front_low = low > max(MIN_ORIENTATION_POINTS, high * ORIENTATION_DOMINANCE)
    front_high = high > max(MIN_ORIENTATION_POINTS, low * ORIENTATION_DOMINANCE)
    if "frontAtLowL" in meta:
        # 朝向来自已提取车辆，不让补充的原始全场景背景改变已确认的车头方向。
        front_low, front_high = meta["frontAtLowL"], meta["frontAtHighL"]
    width_axis = np.array(box.w_range)
    _, front = complete_front_lip(p, box, front_low, front_high, width_axis,
                                  SURFACE_SPACING_M)
    if not front["completed"]:
        return {**result, "reason": "front_edge_not_observed", "front": front}
    # 后壁若也观测到内缘，必须核对；缺测才使用用户确认的前后等宽规则。
    _, rear = complete_front_lip(p, box, front_high, front_low, width_axis,
                                 SURFACE_SPACING_M)
    if (rear["completed"]
            and abs(front["depthM"] - rear["depthM"]) > MAX_END_INSET_DISAGREEMENT_M):
        return {**result, "reason": "end_insets_disagree", "front": front, "rear": rear}
    sides = _measure_side_insets(p, box)
    valid = [s for s in sides if s["valid"]]
    if not valid or any(s["supported"] and not s["valid"] for s in sides):
        return {**result, "reason": "side_edge_not_consistent", "sides": sides}
    if (len(valid) == 2
            and abs(valid[0]["insetM"] - valid[1]["insetM"]) > MAX_SIDE_INSET_DISAGREEMENT_M):
        return {**result, "reason": "side_insets_disagree", "sides": sides}
    best = max(valid, key=lambda s: (s["coveredBins"], s["pointCount"]))
    end_depth = front["depthM"]
    side_depth = best["insetM"]
    return {**result, "applied": True, "reason": "observed_edges_with_symmetry",
            "method": "measured_top_edges_equal_opposite_insets_v1",
            "assumptions": ["equal_front_rear_insets", "equal_left_right_insets",
                            "reference_planes_used_as_outer_datums"],
            "fullHeightInnerWallsObserved": False,
            "lengthM": float(box.length - 2 * end_depth),
            "widthM": float(box.width - 2 * side_depth),
            "endInsetM": end_depth, "sideInsetM": side_depth,
            "referenceLengthM": box.length, "referenceWidthM": box.width,
            "innerLRange": [box.rear_l + end_depth, box.front_l - end_depth],
            "innerWRange": [box.w_range[0] + side_depth, box.w_range[1] - side_depth],
            "referenceSide": best["side"], "front": front, "rear": rear, "sides": sides,
            "measurementInputPointCount": len(p),
            "note": "点云内缘加前后等宽/左右对称推算；不是全高内壁实测，未使用人工标准长宽。"}


def _measure_side_insets(p: np.ndarray, box) -> list[dict]:
    """分别量两侧壁顶部内缘的内伸量，并逐段核对一致性。

    沿长度分段取中位数而不是整体统计：局部一坨噪声能把整体中位数拉偏，
    但撑不起 7 个分段都一致。
    """
    length_edges = np.linspace(box.rear_l + SIDE_LENGTH_MARGIN_M,
                               box.front_l - SIDE_LENGTH_MARGIN_M,
                               SIDE_LENGTH_BIN_EDGES)
    top = box.h_range[1]
    sides = []
    for name, boundary, sign in [("negativeW", box.w_range[0], 1),
                                 ("positiveW", box.w_range[1], -1)]:
        depth = (p[:, 0] - boundary) * sign
        mask = ((p[:, 1] > length_edges[0]) & (p[:, 1] < length_edges[-1])
                & (p[:, 2] > top - SIDE_BAND_BELOW_TOP_M)
                & (p[:, 2] < top + SIDE_BAND_ABOVE_TOP_M)
                & (depth > SIDE_DEPTH_MIN_M) & (depth < SIDE_DEPTH_MAX_M))
        q, values = p[mask], depth[mask]
        samples = [values[(q[:, 1] >= lo) & (q[:, 1] < hi)]
                   for lo, hi in zip(length_edges, length_edges[1:])]
        medians = [float(np.median(v)) for v in samples if len(v) >= MIN_POINTS_PER_BIN]
        supported = len(values) >= MIN_SIDE_POINTS and len(medians) >= MIN_SIDE_COVERED_BINS
        inset = float(np.median(medians)) if supported else None
        # 有足够观测但没有内伸边时，也不能当作缺测而强行镜像另一侧。
        valid = (supported and MIN_SIDE_INSET_M <= inset <= MAX_SIDE_INSET_M
                 and np.ptp(medians) <= MAX_SIDE_INSET_SPREAD_M)
        sides.append({"side": name, "pointCount": len(values), "coveredBins": len(medians),
                      "binMediansM": medians, "insetM": inset, "supported": supported,
                      "valid": bool(valid)})
    return sides


def opening_quads(reference: BoxCompletionResult) -> list[np.ndarray]:
    """仅四条顶部连接带及有实测依据的两端短下折面；不封顶、不补全高内壁。"""
    opening = reference.metadata.get("symmetricOpening", {})
    if not opening.get("applied"):
        return []
    box = reference.box
    w0, w1 = box.w_range
    l0, l1 = box.rear_l, box.front_l
    i0, i1 = opening["innerWRange"]
    j0, j1 = opening["innerLRange"]
    h = box.h_range[1]
    edge = opening["front"]["innerEdgeHeightM"]
    outer = np.array([[w0, l0, h], [w1, l0, h], [w1, l1, h], [w0, l1, h]])
    inner = np.array([[i0, j0, edge], [i1, j0, edge], [i1, j1, edge], [i0, j1, edge]])
    quads = [np.array([outer[k], outer[(k + 1) % 4], inner[(k + 1) % 4], inner[k]])
             for k in range(4)]
    if opening["front"].get("innerDropGenerated"):
        bottom = opening["front"]["innerDropLowerHeightM"]
        for j in [j0, j1]:
            quads.append(np.array([[i0, j, edge], [i1, j, edge],
                                   [i1, j, bottom], [i0, j, bottom]]))
    return quads


def sample_quad(corners: np.ndarray, spacing: float) -> np.ndarray:
    """双线性均匀采样梯形连接带，四角精确，不越过内缘。"""
    a, b, c, d = corners
    n = int(np.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - d)) / spacing)) + 1
    m = int(np.ceil(max(np.linalg.norm(d - a), np.linalg.norm(c - b)) / spacing)) + 1
    u, v = np.meshgrid(np.linspace(0, 1, n), np.linspace(0, 1, m), indexing="ij")
    return ((1 - u)[..., None] * (1 - v)[..., None] * a
            + u[..., None] * (1 - v)[..., None] * b
            + u[..., None] * v[..., None] * c
            + (1 - u)[..., None] * v[..., None] * d).reshape(-1, 3)


def attach_symmetric_opening(
    points: np.ndarray, reference: BoxCompletionResult
) -> BoxCompletionResult:
    """匹配后独立附加元数据和参考补点；原外形 box 与实测点不变。"""
    opening = infer_symmetric_opening(points, reference)
    reference = replace(reference, metadata=deepcopy(reference.metadata))
    reference.metadata["symmetricOpening"] = opening
    if opening["applied"]:
        quads = opening_quads(reference)
        extra = rotate_wl(
            np.vstack([sample_quad(q, SURFACE_SPACING_M) for q in quads]),
            np.deg2rad(reference.metadata["yawDeg"]))
        old_lip_count = reference.metadata.get("frontLip", {}).get("synthesizedPoints", 0)
        base = reference.points[:-old_lip_count] if old_lip_count else reference.points
        reference = replace(reference, points=np.unique(np.vstack([base, extra]), axis=0))
        reference.metadata["synthesizedPoints"] = len(reference.points)
        reference.metadata["walls"].pop("frontTopLip", None)
        reference.metadata["walls"]["symmetricTopRim"] = {
            "synthesizedPoints": len(reference.points) - len(base)}
        reference.metadata["frontLip"] = {**opening["front"], "synthesizedPoints": 0,
                                          "geometryIncludedIn": "symmetricTopRim"}
        reference.metadata["partialTopGenerated"] = True
        opening["generatedPointCountBeforeDedup"] = len(extra)
    return reference
