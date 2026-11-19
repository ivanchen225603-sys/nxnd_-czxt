"""短车厢的实测端壁不能被整车长度比例排除。"""
import numpy as np
import pytest

from carpointcloud_analyzer import Config
from carpointcloud_analyzer.domain.geometry import LocalCarriageExtractor


@pytest.mark.parametrize('reverse', [False, True])
def test_supported_short_body_is_not_extended_to_cab_or_chassis(reverse):
    parts = []

    def face(axis, value, first, second):
        a, b = np.meshgrid(np.arange(*first, .06), np.arange(*second, .06))
        points = np.empty((a.size, 3))
        free = [i for i in range(3) if i != axis]
        points[:, axis] = value
        points[:, free[0]], points[:, free[1]] = a.ravel(), b.ravel()
        parts.append(points)

    for w in (-1.2, 1.2):
        face(0, w, (0., 6.001), (1.6, 3.601))
    for end in (0., 6.):
        face(1, end, (-1.2, 1.201), (1.6, 3.601))
    face(2, 1.6, (-1.2, 1.201), (0., 6.001))
    # 长底盘与驾驶室都是本车实测结构，不能纳入车厢底板或主体侧壁。
    face(2, 1., (-1., 1.001), (6., 12.001))
    for w in (-1.1, 1.1):
        face(0, w, (12., 14.501), (.8, 3.001))
    for end in (12., 14.5):
        face(1, end, (-1.1, 1.101), (.8, 3.001))
    face(2, 3., (-1.1, 1.101), (12., 14.501))
    points = np.vstack(parts)
    if reverse:
        points[:, :2] *= -1
    split = LocalCarriageExtractor(Config()).split(points)
    assert split.boundaries_are_inner_faces
    assert split.split_y - split.carriage_start_y == pytest.approx(6., abs=.05)
    assert (split.carriage_start_y, split.split_y) == pytest.approx(
        (-6., 0.) if reverse else (0., 6.), abs=.05)
