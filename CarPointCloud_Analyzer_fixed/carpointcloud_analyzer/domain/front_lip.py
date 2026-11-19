"""已识别短矩形车型的前壁顶部内折边；只生成局部连接带，不封顶。

阈值全部收敛为本模块常量。折边判据是"实测上沿点带"的形态判定，与站点标定无关，
因此不进 :class:`~carpointcloud_analyzer.config.Config`：改这些值等于改识别方法本身，
不是现场可调参数。取值来自 0907/0909 两台短矩形车的实测点带。
"""

import numpy as np

from carpointcloud_analyzer.domain.models import BoxCarriageBox

# --- 支撑点带的取点范围（车辆局部 W/L/H，单位 m）---
# 内伸量上限同时受绝对值和箱长比例约束：短车厢按比例，长车厢按绝对值封顶。
MAX_LIP_DEPTH_M = 0.4
MAX_LIP_DEPTH_LENGTH_RATIO = 0.08
# 下限剔除端壁面自身的回波：小于此深度的点仍属于前壁，不是折边。
MIN_LIP_DEPTH_M = 0.10
# 竖直取点带：上沿以下 0.18m 到上沿以上 0.02m。折边只可能出现在这一带。
LIP_BAND_BELOW_TOP_M = 0.18
LIP_BAND_ABOVE_TOP_M = 0.02
# 两侧各内缩，避开侧壁与前壁的转角，转角点会把内伸量估大。
SIDE_MARGIN_M = 0.05

# --- 支撑充分性门槛 ---
MIN_SUPPORT_POINTS = 120
WIDTH_PERCENTILES = (5, 95)
MIN_WIDTH_COVERAGE = 0.35
WIDTH_BIN_COUNT = 10
MIN_COVERED_WIDTH_BINS = 5

# --- 折边形态估计 ---
# 内伸量取深度的高分位而不是最大值：最大值会被单点噪声拉长。
EXTENSION_PERCENTILE = 95
# 内缘高度只用最深的四分之一点，避免混入斜面中段。
EDGE_PERCENTILE = 75
# 实测点与线性折边模型的 90 分位残差上限；超过说明这不是一条一致的折边。
MAX_BAND_RESIDUAL_M = 0.06

# --- 折边末端的下折面 ---
# 只用内半段回波估下折深度，避免把主体前壁高度当成折边厚度。
INNER_BAND_PERCENTILE = 50
DROP_BOTTOM_PERCENTILE = 5
MIN_DROP_M = 0.025
MAX_DROP_M = 0.15


def complete_front_lip(
    points: np.ndarray, box: BoxCarriageBox, front_low: bool, front_high: bool,
    width_axis: np.ndarray, step: float,
) -> tuple[np.ndarray, dict]:
    """从独立实测上沿点带估计内伸量，按左右对称先验连接前壁与两侧壁。"""
    empty = np.empty((0, 3), dtype=float)

    def skip(reason: str) -> tuple[np.ndarray, dict]:
        """支撑不足时返回空点，由调用方保留原有流程。"""
        return empty, {"completed": False, "reason": reason, "synthesizedPoints": 0}

    if front_low == front_high:
        return skip("front_direction_unknown")
    front = box.rear_l if front_low else box.front_l
    inward = 1.0 if front_low else -1.0
    top = box.h_range[1]
    depth = (points[:, 1] - front) * inward
    max_depth = min(MAX_LIP_DEPTH_M, box.length * MAX_LIP_DEPTH_LENGTH_RATIO)
    selected = points[(depth > MIN_LIP_DEPTH_M) & (depth < max_depth)
                      & (points[:, 2] > top - LIP_BAND_BELOW_TOP_M)
                      & (points[:, 2] <= top + LIP_BAND_ABOVE_TOP_M)
                      & (points[:, 0] >= box.w_range[0] + SIDE_MARGIN_M)
                      & (points[:, 0] <= box.w_range[1] - SIDE_MARGIN_M)]
    if len(selected) < MIN_SUPPORT_POINTS:
        return skip("insufficient_front_lip_support")
    width_lo, width_hi = np.percentile(selected[:, 0], WIDTH_PERCENTILES)
    coverage = float((width_hi - width_lo) / box.width)
    bin_index = (selected[:, 0] - box.w_range[0]) / box.width * WIDTH_BIN_COUNT
    bins = np.unique(np.clip(bin_index.astype(int), 0, WIDTH_BIN_COUNT - 1))
    if coverage < MIN_WIDTH_COVERAGE or len(bins) < MIN_COVERED_WIDTH_BINS:
        return skip("front_lip_support_too_local")
    depths = (selected[:, 1] - front) * inward
    extension = float(np.percentile(depths, EXTENSION_PERCENTILE))
    edge = selected[depths >= np.percentile(depths, EDGE_PERCENTILE)]
    edge_height = min(top, float(np.median(edge[:, 2])))
    # 观测边缘到前壁上沿形成短折边；只在宽度方向镜像，不将长向复制到后壁。
    predicted = top + (edge_height - top) * depths / extension
    residual = float(np.percentile(np.abs(selected[:, 2] - predicted), 90))
    if residual > MAX_BAND_RESIDUAL_M:
        return skip("front_lip_not_a_consistent_band")
    distance_axis = np.linspace(0, extension, int(np.ceil(extension / step)) + 1)
    widths, distances = np.meshgrid(width_axis, distance_axis, indexing="ij")
    generated = np.column_stack([widths.ravel(), front + inward * distances.ravel(),
                                 top + (edge_height - top) * distances.ravel() / extension])
    # 实测内缘若呈短竖向点带，补出折边末端的下折面；不能只铺一层薄连接面。
    inner_band = selected[depths >= np.percentile(depths, INNER_BAND_PERCENTILE)]
    drop_height = min(edge_height,
                      float(np.percentile(inner_band[:, 2], DROP_BOTTOM_PERCENTILE)))
    drop = edge_height - drop_height
    drop_generated = MIN_DROP_M <= drop <= MAX_DROP_M
    if drop_generated:
        heights = np.linspace(drop_height, edge_height, int(np.ceil(drop / step)) + 1)[:-1]
        ww, hh = np.meshgrid(width_axis, heights, indexing="ij")
        generated = np.vstack([generated, np.column_stack([
            ww.ravel(), np.full(ww.size, front + inward * extension), hh.ravel()])])
    return generated, {
        "completed": True, "method": "observed_front_edge_symmetric_connection",
        "referenceOnly": True, "depthM": extension, "frontL": front, "inwardSign": inward,
        "wallEdgeHeightM": top, "innerEdgeHeightM": edge_height,
        "innerDropGenerated": bool(drop_generated),
        "innerDropLowerHeightM": drop_height if drop_generated else edge_height,
        "widthRangeM": list(box.w_range), "supportPointCount": len(selected),
        "observedWidthCoverage": coverage, "supportResidualP90M": residual,
        "fullWidthInferred": True, "roofClosed": False,
        "coordinateFrame": "yaw_aligned_vehicle_local_WLH",
        "synthesizedPoints": len(generated),
    }
