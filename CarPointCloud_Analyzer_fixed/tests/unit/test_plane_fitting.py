"""平面最小二乘拟合与 RANSAC 重拟合的测试。

重点覆盖：拟合精度、法向朝向保持、退化输入不崩、以及重拟合确实提升可复现性。
"""

import numpy as np
import pytest

from carpointcloud_analyzer.domain.utils.planes import (
    consolidate_plane,
    fit_plane,
    refine_plane,
)


def _plane_points(rng, normal, offset, n=800, noise=0.002):
    """在给定平面上采样带噪点。"""
    normal = np.asarray(normal, float)
    normal = normal / np.linalg.norm(normal)
    # 构造平面内的两个正交基
    helper = np.array([1.0, 0.0, 0.0])
    if abs(normal @ helper) > 0.9:
        helper = np.array([0.0, 1.0, 0.0])
    u = np.cross(normal, helper)
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    base = -offset * normal
    coords = rng.uniform(-5, 5, (n, 2))
    points = base + coords[:, :1] * u + coords[:, 1:] * v
    return points + rng.normal(0, noise, points.shape)


def test_fit_plane_recovers_horizontal_plane():
    rng = np.random.default_rng(0)
    points = _plane_points(rng, [0, 0, 1], -2.0)   # z = 2 平面
    plane = fit_plane(points)
    assert abs(abs(plane[2]) - 1.0) < 1e-3          # 法向近 ±z
    # 平面上任一点代入方程应≈0
    residual = points @ plane[:3] + plane[3]
    assert np.abs(residual).max() < 0.01


def test_fit_plane_recovers_tilted_plane():
    rng = np.random.default_rng(1)
    normal = np.array([0.1, -0.05, 1.0])
    normal /= np.linalg.norm(normal)
    points = _plane_points(rng, normal, 1.5)
    plane = fit_plane(points)
    # 法向平行（允许整体反号）
    assert abs(abs(plane[:3] @ normal) - 1.0) < 1e-3


def test_fit_plane_is_normalized():
    rng = np.random.default_rng(2)
    plane = fit_plane(_plane_points(rng, [0, 0, 1], 0.0))
    assert np.linalg.norm(plane[:3]) == pytest.approx(1.0, abs=1e-9)


def test_fit_plane_handles_degenerate_input():
    assert not fit_plane(np.zeros((2, 3))).any()      # 点数不足
    collinear = np.column_stack([np.linspace(0, 1, 10), np.zeros(10), np.zeros(10)])
    plane = fit_plane(collinear)                       # 共线：法向不唯一但不应崩
    assert plane.shape == (4,)


def test_refine_plane_keeps_normal_orientation():
    """重拟合只收紧数值，不能翻转法向 —— 下游按符号建坐标轴。"""
    rng = np.random.default_rng(3)
    points = _plane_points(rng, [0, 0, 1], -2.0)
    for ransac in (np.array([0.0, 0.0, 1.0, -2.0]), np.array([0.0, 0.0, -1.0, 2.0])):
        refined = refine_plane(ransac, points)
        assert refined[:3] @ ransac[:3] > 0


def test_refine_plane_falls_back_on_bad_inliers():
    ransac = np.array([0.0, 0.0, 1.0, -2.0])
    refined = refine_plane(ransac, np.zeros((1, 3)))
    assert np.allclose(refined, ransac)


def test_refine_plane_is_deterministic_unlike_ransac():
    """相同内点重复重拟合必须逐位一致 —— 这正是引入它的目的。"""
    rng = np.random.default_rng(4)
    points = _plane_points(rng, [0.02, 0.01, 1.0], -1.0)
    ransac = np.array([0.0, 0.0, 1.0, -1.0])
    first = refine_plane(ransac, points)
    for _ in range(5):
        assert np.array_equal(refine_plane(ransac, points), first)


def test_consolidate_plane_converges_from_different_seeds():
    """不同随机初始假设必须收敛到同一个解 —— 这是消除运行间抖动的关键。

    契约是"平面姿态收敛"，不是"内点掩码逐位相同"：恰好压在距离阈值上的点
    仍可能进出，但那已不影响下游几何。因此断言法向夹角与掩码一致率。
    """
    rng = np.random.default_rng(10)
    ground = _plane_points(rng, [0, 0, 1], 0.0, n=3000, noise=0.03)
    above = rng.uniform([-5, -5, 0.5], [5, 5, 3.0], (500, 3))
    points = np.vstack([ground, above])

    planes, masks = [], []
    for seed in range(4):
        sub = np.random.default_rng(seed).choice(len(ground), 3, replace=False)
        rough = fit_plane(ground[sub])
        if rough[2] < 0:
            rough = -rough
        plane, mask = consolidate_plane(points, rough, sub, distance_threshold=0.12)
        planes.append(plane)
        masks.append(mask)

    for plane, mask in zip(planes[1:], masks[1:]):
        angle = np.degrees(np.arccos(np.clip(abs(plane[:3] @ planes[0][:3]), -1, 1)))
        assert angle < 0.01, f"法向未收敛，夹角 {angle:.4f}°"
        assert (mask == masks[0]).mean() > 0.999


def test_consolidate_plane_recovers_true_inliers():
    rng = np.random.default_rng(11)
    ground = _plane_points(rng, [0, 0, 1], 0.0, n=2000, noise=0.02)
    above = rng.uniform([-5, -5, 1.0], [5, 5, 3.0], (400, 3))
    points = np.vstack([ground, above])
    seed_idx = np.arange(50)
    plane, mask = consolidate_plane(
        points, fit_plane(ground[seed_idx]), seed_idx, distance_threshold=0.12
    )
    # 地面点应几乎全部入选，上方点应全部排除
    assert mask[:len(ground)].mean() > 0.99
    assert mask[len(ground):].sum() == 0
    assert abs(abs(plane[2]) - 1.0) < 1e-3


def test_consolidate_plane_stops_when_inliers_collapse():
    """阈值过小导致内点不足时应停在上一轮，而不是抛异常。"""
    rng = np.random.default_rng(12)
    points = _plane_points(rng, [0, 0, 1], 0.0, n=200, noise=0.5)
    seed_idx = np.arange(10)
    plane, mask = consolidate_plane(
        points, fit_plane(points[seed_idx]), seed_idx, distance_threshold=1e-6
    )
    assert plane.shape == (4,)
    assert mask.shape == (len(points),)


def test_refine_plane_beats_noisy_ransac_hypothesis():
    """RANSAC 只用 3 点定平面，重拟合用全部内点，应更贴近真值。"""
    rng = np.random.default_rng(5)
    true_normal = np.array([0.0, 0.0, 1.0])
    points = _plane_points(rng, true_normal, -2.0, n=2000, noise=0.01)
    # 模拟一个由 3 个带噪点定出的粗糙 RANSAC 假设
    sample = points[rng.choice(len(points), 3, replace=False)]
    rough = fit_plane(sample)
    if rough[:3] @ true_normal < 0:
        rough = -rough
    refined = refine_plane(rough, points)
    rough_error = 1.0 - abs(rough[:3] @ true_normal)
    refined_error = 1.0 - abs(refined[:3] @ true_normal)
    assert refined_error <= rough_error
