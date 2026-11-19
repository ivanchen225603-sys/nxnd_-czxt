"""非周期四邻域高度估计；数值内插不是未观测区域的实测或物理上界。"""
from __future__ import annotations

from typing import Optional

import numpy as np


DEFAULT_FILL_ITERATIONS = 200


def fill_height_holes(values: np.ndarray, iters: int = DEFAULT_FILL_ITERATIONS,
                      domain: Optional[np.ndarray] = None) -> np.ndarray:
    """固定原实测锚点，持续松弛原未知格；域外和无锚点连通域保持 NaN。

    边缘只有实际存在的邻居，不首尾环绕。迭代次数有限，不承诺任意大孔洞
    都已收敛，也不把单侧常值延伸当作真实端部高度。输入始终不修改。
    """
    source = np.asarray(values, dtype=np.float64)
    if source.ndim != 2:
        raise ValueError("高度图必须为二维数组")
    if isinstance(iters, (bool, np.bool_)) or not isinstance(iters, (int, np.integer)) or iters < 0:
        raise ValueError("补点迭代次数必须为非负整数")
    inside = np.ones(source.shape, dtype=bool) if domain is None else np.asarray(domain)
    if inside.shape != source.shape or inside.dtype.kind != "b":
        raise ValueError("补点域必须为同形布尔掩码")
    fixed = inside & np.isfinite(source)
    unknown = inside & ~fixed
    present = fixed.copy()
    values_now = np.zeros_like(source)
    values_now[fixed] = source[fixed]
    for _ in range(int(iters)):
        if not unknown.any() or not present.any():
            break
        # 不让数组两端互邻；域外值和计数恒为零。
        count = np.zeros(source.shape, dtype=np.int8)
        scale = np.zeros_like(source)
        neighbors = ((np.s_[1:, :], np.s_[:-1, :]), (np.s_[:-1, :], np.s_[1:, :]),
                     (np.s_[:, 1:], np.s_[:, :-1]), (np.s_[:, :-1], np.s_[:, 1:]))
        for dst, src in neighbors:
            count[dst] += present[src]
            np.maximum(scale[dst], np.abs(values_now[src]), out=scale[dst])
        scale[scale == 0] = 1.0
        total = np.zeros_like(values_now)
        # 每格独立归一化，避免极值求和溢出，也不压没另一个连通域的小值。
        for dst, src in neighbors:
            total[dst] += (values_now[src] / scale[dst]) / np.maximum(count[dst], 1)
        update = unknown & (count > 0)
        if not update.any():
            break
        next_values = values_now.copy()
        next_values[update] = np.clip(total[update], -1.0, 1.0) * scale[update]
        unchanged = np.array_equal(next_values, values_now) and np.all(present[update])
        values_now = next_values
        present |= update
        if unchanged:
            break
    return np.where(present & inside, values_now, np.nan)
