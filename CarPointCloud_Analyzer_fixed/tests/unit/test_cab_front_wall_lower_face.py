"""车头距的前壁端点取离 1.40m 截面最近的下部实测墙面，不受顶部篷布带偏。

回归背景（2026-09-24 现场）：车 B 前壁顶部有篷布，整面前壁被拟合成一块倾斜
4.6° 的面，外推到离地 1.40m 时比下部实测墙面靠厢内约 9cm，车头距在 440 与
449.5cm 之间跳（取决于该倾斜面能否入选）。车 C 卷尺实测 450cm。
"""

import numpy as np
import pytest

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.dimensions import DimensionCalculator
from carpointcloud_analyzer.domain.models import CarriageSplitResult

GROUND = np.array([0.0, 0.0, 1.0, 0.0])      # 离地高度即 H
TILTED_FULL_FACE = (0.0, 0.08, 2.85)          # 被篷布拉斜的整面拟合：L = 2.85 + 0.08·H


def _vertical_wall(l_value, h_range=(1.55, 2.4), step=0.04, seed=0):
    rng = np.random.default_rng(seed)
    w, h = np.meshgrid(np.arange(-1.2, 1.2001, step), np.arange(*h_range, step), indexing="ij")
    return np.column_stack([w.ravel(), l_value + rng.normal(0, 0.003, w.size), h.ravel()])


def _head(seed=1):
    rng = np.random.default_rng(seed)
    return np.column_stack([rng.uniform(-1, 1, 200), rng.uniform(7.3, 7.4, 200),
                            rng.uniform(1.39, 1.41, 200)])


def _floor():
    w, l_value = np.meshgrid(np.arange(-1.2, 1.2001, 0.04), np.arange(0.0, 3.0, 0.04), indexing="ij")
    return np.column_stack([w.ravel(), l_value.ravel(), np.full(w.size, 1.55)])


def _tarp(seed=2):
    """前壁顶部向车头方向伸出的篷布：2.4~3.5m 高，L 从 3.0 斜到 3.25。"""
    rng = np.random.default_rng(seed)
    h = rng.uniform(2.4, 3.5, 3000)
    return np.column_stack([rng.uniform(-1.2, 1.2, 3000), 3.0 + (h - 2.4) * 0.23, h])


def _split():
    return CarriageSplitResult(np.empty((0, 3)), 3.0, -8.0, 7.5, -8.2,
                               front_wall_plane=TILTED_FULL_FACE)


def test_tarp_tilted_full_face_does_not_shift_cab_front_wall():
    """前壁端点取下部竖直实测墙面，而不是被篷布拉斜的整面投影。"""
    points = np.vstack([_head(), _vertical_wall(3.0), _floor(), _tarp()])
    split = _split()

    assert DimensionCalculator(Config()).measure_cab_to_front_wall_at_height(points, split, GROUND)

    assert split.cab_measurement_front_source == "lower_wall_face_near_measurement_height"
    assert split.cab_measurement_front_y == pytest.approx(3.0, abs=0.01)
    # 旧做法把倾斜整面投影到 1.40m，会落在 2.962m，比下部实测墙面靠厢内约 4cm
    assert abs(split.cab_measurement_front_y - (2.85 + 0.08 * 1.40)) > 0.03
    head = np.percentile(_head()[:, 1], 99)
    assert split.cab_to_front_wall == pytest.approx(head - 3.0, abs=0.01)


def test_lower_window_takes_the_face_toward_the_carriage():
    """下部窗口里同时看到前壁内外两面时，与端壁分割同规则取面向厢内（L 较小）的一面。"""
    points = np.vstack([_head(), _vertical_wall(3.0, seed=3), _vertical_wall(3.08, seed=4)])
    split = _split()

    assert DimensionCalculator(Config()).measure_cab_to_front_wall_at_height(points, split, GROUND)

    assert split.cab_measurement_front_source == "lower_wall_face_near_measurement_height"
    assert split.cab_measurement_front_y == pytest.approx(3.0, abs=0.01)


def test_unobserved_lower_wall_falls_back_to_full_face_projection():
    """截面以上看不到下部墙面时，退回原整面投影并标明来源，测距不因此失效。"""
    split = _split()

    assert DimensionCalculator(Config()).measure_cab_to_front_wall_at_height(_head(), split, GROUND)

    assert split.cab_measurement_front_source == "full_height_front_face_projection"
    assert split.cab_measurement_front_y == pytest.approx(2.85 + 0.08 * 1.40)


def test_split_front_is_used_when_no_front_face_plane():
    split = CarriageSplitResult(np.empty((0, 3)), 3.0, -8.0, 7.5, -8.2)

    assert DimensionCalculator(Config()).measure_cab_to_front_wall_at_height(_head(), split, GROUND)

    assert split.cab_measurement_front_source == "carriage_split_front_inner"
    assert split.cab_measurement_front_y == pytest.approx(3.0)
