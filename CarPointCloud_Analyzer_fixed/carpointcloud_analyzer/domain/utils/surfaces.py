"""规则壁面采样基础函数；不包含车型选择、去重或点数预算策略。"""

import numpy as np


def axis_aligned_face(axes: list[np.ndarray], axis: int, value: float) -> np.ndarray:
    """按原 ij 顺序采样一个定坐标平面，保留端点和浮点运算顺序。"""
    free = [i for i in range(3) if i != axis]
    aa, bb = np.meshgrid(axes[free[0]], axes[free[1]], indexing='ij')
    face = np.empty((aa.size, 3))
    face[:, axis], face[:, free[0]], face[:, free[1]] = value, aa.ravel(), bb.ravel()
    return face
