"""平面拟合工具。

open3d 的 ``segment_plane`` 是随机 RANSAC，且 0.19 版不接受随机种子，因此同一份
点云连续两次运行会给出略有差异的平面（实测整车宽/车厢高在毫米级抖动）。对车辆
档案这类测量输出，同一辆车两次测量给不同数字是不可接受的。

重拟合只能收敛到初始假设附近，不能保证多个平面竞争时总落到同一解。
场景前处理先用排序后的局部固定采样序列选假设，再全量重拟合；不设置全局随机种子。
"""

from typing import Tuple

import numpy as np

from carpointcloud_analyzer.domain.utils.parallel import WORKERS, map_blocks


def _lexicographic_order(points: np.ndarray) -> np.ndarray:
    """与 ``np.lexsort(points.T[::-1])`` 相同的行序。

    先按首列稳定排序；只有首列相等的行才需要比较后续列，这些行在原位置内再做完整字典序排序。
    """
    order = np.argsort(points[:, 0], kind='stable')
    first = points[order, 0]
    same = first[1:] == first[:-1]
    if same.any():
        tied = np.r_[False, same] | np.r_[same, False]
        rows = order[tied]
        order[tied] = rows[np.lexsort(points[rows].T[::-1])]
    return order


def deterministic_plane_hypothesis(points: np.ndarray, distance_threshold: float,
                                   num_iterations: int = 1000) -> Tuple[np.ndarray, np.ndarray]:
    """确定性共识初值，避免多层地面随机选到不同吸引域。

    仅作 RANSAC 初值，调用方仍须验证法向/跨度/高度并全量收敛。固定局部采样序列，
    不修改 Open3D/NumPy 全局随机状态；排序使相同点集不依赖输入顺序。最多 8192 点
    对假设评分，分块控制内存，最终内点回到全量点集且保持调用方原下标。
    """
    points = np.asarray(points, dtype=float)
    if len(points) < 3 or not np.isfinite(points).all() or distance_threshold <= 0:
        raise ValueError("plane hypothesis requires finite points and a positive threshold")
    ordered = points[_lexicographic_order(points)]
    sample = ordered[np.linspace(0, len(ordered) - 1, min(len(ordered), 8192), dtype=int)]
    rng = np.random.default_rng(0)
    triples = sample[rng.integers(0, len(sample), size=(max(1, num_iterations), 3))]
    normals = np.cross(triples[:, 1] - triples[:, 0], triples[:, 2] - triples[:, 0])
    norms = np.linalg.norm(normals, axis=1)
    valid = norms > 1e-10
    if not valid.any():
        raise ValueError("degenerate plane samples")
    normals = normals[valid] / norms[valid, None]
    offsets = -np.einsum('ij,ij->i', normals, triples[valid, 0])
    def score(start, stop):
        scores = []
        for begin in range(start, stop, 32):
            end = min(begin + 32, stop)
            distance = sample @ normals[begin:end].T
            distance += offsets[begin:end]
            np.abs(distance, out=distance)
            inside = distance <= distance_threshold
            counts = inside.sum(axis=0)
            distance[~inside] = 0
            scores.append((counts, distance.sum(axis=0) / np.maximum(counts, 1)))
        return scores

    # 仍按 32 个假设一块评分；各块互不相关，分给各线程后按原假设顺序比较，
    # 先出现的同分假设优先。
    best_key, best, index = (-1, -float('inf')), None, 0
    group = 32 * max(1, -(-len(normals) // (32 * WORKERS)))
    for scores in map_blocks(score, len(normals), group):
        for counts, errors in scores:
            for j in range(len(counts)):
                key = (int(counts[j]), -float(errors[j]))
                if key > best_key:
                    best_key, best = key, index + j
            index += len(counts)
    plane = np.r_[normals[best], offsets[best]]
    if plane[2] < 0:
        plane *= -1
    inliers = np.flatnonzero(np.abs(points @ plane[:3] + plane[3]) <= distance_threshold)
    return plane, inliers


def fit_plane(points: np.ndarray) -> np.ndarray:
    """对点集做最小二乘平面拟合（SVD），返回归一化的 ``[a, b, c, d]``。

    满足 ``ax + by + cz + d = 0``。法向方向由 SVD 决定，调用方若对朝向有要求
    需自行翻转。少于 3 点时无法定义平面，返回全零向量由调用方判断。
    """
    if len(points) < 3:
        return np.zeros(4)
    centroid = points.mean(axis=0)
    # 最小奇异值对应的右奇异向量即最佳拟合平面的法向
    _, _, vh = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vh[-1]
    norm = np.linalg.norm(normal)
    if norm < 1e-12:
        return np.zeros(4)
    normal = normal / norm
    return np.concatenate([normal, [-float(normal @ centroid)]])


def consolidate_plane(
    points: np.ndarray,
    ransac_plane: np.ndarray,
    ransac_inliers: np.ndarray,
    distance_threshold: float,
    iterations: int = 6,
) -> Tuple[np.ndarray, np.ndarray]:
    """交替「重拟合平面 / 重选内点」，收敛当前初始假设附近的平面。

    单纯重拟合平面还不够：真正让输出抖动的是**内点集合本身**每次都不同 ——
    落在距离阈值边界附近的点会在两次运行间进出，导致剔除后的车辆点集不同，
    进而让所有尺寸在毫米级漂移。这里每轮都用当前平面重新按阈值选一遍内点。
    多层/相交平面仍可能存在不同局部解，调用方需要确定性初值才能保证重复性。

    Args:
        points: 参与拟合的全部点。
        ransac_plane: RANSAC 给出的初始平面。
        ransac_inliers: RANSAC 给出的初始内点下标。
        distance_threshold: 判定内点的点到平面距离阈值。
        iterations: 交替轮数上限；内点不再变化即提前收敛，通常 2~3 轮即可。
    Returns:
        (稳定后的平面, 内点布尔掩码)
    """
    plane = refine_plane(ransac_plane, points[ransac_inliers])
    mask = np.zeros(len(points), dtype=bool)
    mask[ransac_inliers] = True
    for _ in range(max(0, iterations)):
        distance = np.abs(points @ plane[:3] + plane[3])
        new_mask = distance <= distance_threshold
        if new_mask.sum() < 3 or np.array_equal(new_mask, mask):
            break
        mask = new_mask
        plane = refine_plane(plane, points[mask])
    return plane, mask


def refine_plane(ransac_plane: np.ndarray, inlier_points: np.ndarray) -> np.ndarray:
    """用内点重拟合 RANSAC 平面，并保持与原平面一致的法向朝向。

    保持朝向是为了让下游依赖法向符号的逻辑（例如由此构建的坐标轴）行为不变，
    重拟合只收紧数值、不改变语义。拟合失败时原样返回 RANSAC 结果。
    """
    refined = fit_plane(inlier_points)
    if not refined.any():
        return np.asarray(ransac_plane, dtype=float)
    ransac_plane = np.asarray(ransac_plane, dtype=float)
    if refined[:3] @ ransac_plane[:3] < 0:
        refined = -refined
    return refined
