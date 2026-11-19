"""密集驾驶室前脸不能替代车厢前壁；几何真值来自联合测试场景。"""

import numpy as np
import pytest

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.dimensions import DimensionCalculator
from carpointcloud_analyzer.domain.geometry import LocalCarriageExtractor
from carpointcloud_analyzer.domain.heading import cab_requires_heading_reversal


@pytest.mark.parametrize('cab_step,reverse', [(.04, False), (.03, True)])
def test_dense_cab_face_preserves_carriage_length_and_cab_distance(cab_step, reverse):
    parts = []

    def surface(axis, value, other_a, other_b, step=.06):
        a, b = np.meshgrid(np.arange(*other_a, step), np.arange(*other_b, step))
        points = np.empty((a.size, 3))
        points[:, axis] = value
        axes = [i for i in range(3) if i != axis]
        points[:, axes[0]], points[:, axes[1]] = a.ravel(), b.ravel()
        parts.append(points)

    for w in (-1.2, 1.2):
        surface(0, w, (0, 10.001), (1.6, 3.601))
    for end_l in (0, 10):
        surface(1, end_l, (-1.2, 1.201), (1.6, 3.601))
    surface(2, 1.6, (-1.2, 1.201), (0, 10.001))
    for w in (-1.1, 1.1):
        surface(0, w, (10.2, 14.501), (.8, 3.001))
    surface(1, 14.5, (-1.1, 1.101), (.8, 3.001), cab_step)
    surface(2, 3., (-1.1, 1.101), (10.2, 14.501))
    points = np.vstack(parts)
    if reverse:
        points[:, :2] *= -1
        points = points[::-1].copy()

    extractor = LocalCarriageExtractor(Config())
    split = extractor.split(points)
    assert split.boundaries_are_inner_faces
    assert split.split_y - split.carriage_start_y == pytest.approx(10., abs=.05)
    assert cab_requires_heading_reversal(points, split) == reverse
    if reverse:
        points[:, :2] *= -1
        split = extractor.split(points)
    assert DimensionCalculator(Config()).measure_cab_to_front_wall_at_height(
        points, split, np.array([0., 0., 1., 0.]))
    assert split.cab_to_front_wall == pytest.approx(4.5, abs=.05)
