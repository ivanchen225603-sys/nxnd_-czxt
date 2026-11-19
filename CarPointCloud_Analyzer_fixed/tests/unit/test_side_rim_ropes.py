"""沿用原 5cm 精度要求：纵向绳索不能替代实际侧壁顶沿。"""

from types import SimpleNamespace

import numpy as np
import pytest

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.dimensions import _measure_continuous_side_height
from carpointcloud_analyzer.domain.side_wall_body import side_body_mask


@pytest.mark.parametrize('layers,gap,sag', [(1, .08, 0.), (3, .08, 0.), (3, .14, .14)])
def test_nearby_longitudinal_ropes_do_not_raise_two_meter_wall(layers, gap, sag):
    # 与已冻结失败联合样本相同的墙/绳采样；不是把绳索移远或加密墙面。
    length, height = np.meshgrid(np.arange(0., 10.001, .06), np.arange(1.6, 3.601, .06))
    walls = np.vstack([np.c_[np.full(length.size, side), length.ravel(), height.ravel()]
                       for side in (-1.2, 1.2)])
    width, length = np.meshgrid(np.arange(-1.2, 1.201, .06), np.arange(0., 10.001, .06))
    floor = np.c_[width.ravel(), length.ravel(), np.full(width.size, 1.6)]
    longitudinal = np.arange(.5, 9.501, .05)
    curve = sag * (1. - ((longitudinal - 5.) / 4.5) ** 2)
    ropes = np.vstack([
        np.c_[np.full(len(longitudinal), side), longitudinal, 3.6 + gap + level * .08 - curve]
        for side in (-1.2, 1.2) for level in range(layers)])
    points = np.vstack((floor, walls, ropes))
    frozen = points.copy()
    profile = SimpleNamespace(floor_z=1.6, wall_top=3.6 + gap + (layers - 1) * .08,
                              interior_w_range=(-1.05, 1.05))
    measured, info = _measure_continuous_side_height(points, profile, Config())
    assert measured == pytest.approx(2., abs=.05)
    assert info['supportBinCount'] == {'left': 12, 'right': 12}
    np.testing.assert_array_equal(points, frozen)


def test_rope_removal_preserves_actual_top_row_and_folded_rim():
    # 外折上沿仍是墙体；上方三条独立绳索不能成为折沿的延长部分。
    length, height = np.meshgrid(np.linspace(0., 10., 168), np.linspace(1.6, 3.6, 34))
    width = 1.2 + np.maximum(height - 3.3, 0.) / 3.
    walls = np.vstack([np.c_[sign * width.ravel(), length.ravel(), height.ravel()]
                       for sign in (-1., 1.)])
    longitudinal = np.arange(.5, 9.501, .05)
    ropes = np.vstack([
        np.c_[np.full(len(longitudinal), side), longitudinal, np.full(len(longitudinal), 3.68 + level * .08)]
        for side in (-1.3, 1.3) for level in range(3)])
    points = np.vstack((walls, ropes))
    edges = np.linspace(1., 9., 13)
    for sign in (-1., 1.):
        side = sign * points[:, 0] > 0
        kept = side_body_mask(points, side, edges, 20)
        real_top = (np.arange(len(points)) < len(walls)) & side & (points[:, 2] == 3.6)
        real_top &= (points[:, 1] >= edges[0]) & (points[:, 1] < edges[-1])
        assert np.all(kept[real_top])
        assert not np.any(kept[len(walls):])
