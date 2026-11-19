from typing import Tuple

import numpy as np


def robust_span(values: np.ndarray, p_low: float, p_high: float) -> float:
    """返回按分位数计算的跨度，降低离群点的影响。"""
    if len(values) == 0:
        return 0.0
    lo, hi = np.percentile(values, [p_low, p_high])
    return float(hi - lo)


def occupancy_extent(
    values: np.ndarray, bin_width: float = 0.2, min_points: int = 5
) -> Tuple[float, float]:
    """按 bin 占用计算一维跨度（抗远端稀疏）。

    分位数跨度会把点数稀少的远端当噪声裁掉，远点缺失时尺寸偏短；改为凡是
    点数 >= ``min_points`` 的 bin 都算有效占用，稀疏但真实存在的结构被保留，
    孤立飞点仍被排除。返回值取占用区间内的**真实极值**而非 bin 边界 ——
    取边界会让跨度两端各虚高最多一个 bin 宽。

    注意 ``min_points`` 是"每 bin 绝对点数"，不随点云总量自适应：低于门槛的
    零散点一律按噪声丢弃。也不能改成"按空隙连通性扩展"—— 整车长依赖本函数
    跨越车厢与车头之间的空隙（实测 3.22m），连通性判据会把车头判为不连通。
    """
    v_min, v_max = float(values.min()), float(values.max())
    n_bins = max(1, int(np.ceil((v_max - v_min) / bin_width)))
    index = np.clip(((values - v_min) / bin_width).astype(int), 0, n_bins - 1)
    counts = np.bincount(index, minlength=n_bins)
    occupied = np.where(counts >= min_points)[0]
    if len(occupied) == 0:
        return v_min, v_max
    # 取占用区间内的真实极值，而不是 bin 边界 —— 后者两端各多算最多一个
    # bin 宽（实测使整车宽虚高 0.4m）。仍然排除了占用不足的孤立飞点 bin。
    inside = (index >= occupied[0]) & (index <= occupied[-1])
    return float(values[inside].min()), float(values[inside].max())
