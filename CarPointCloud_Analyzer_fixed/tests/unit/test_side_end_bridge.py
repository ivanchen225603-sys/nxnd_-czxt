"""左右侧壁的同形端部缺口应以相同条件触发长接缝。"""

import numpy as np
import pytest

from carpointcloud_analyzer.domain.top_end_measurement import side_ends_needing_bridge


@pytest.mark.parametrize(('left_gaps', 'right_gaps', 'expected'), [
    ((2., 0.), (0., 0.), {('left', 'rearWall')}),
    ((0., 0.), (0., 2.), {('right', 'frontWall')}),
    ((2., 0.), (0., 2.), {('left', 'rearWall'), ('right', 'frontWall')}),
    ((0., 0.), (2., 2.), {('right', 'rearWall'), ('right', 'frontWall')}),
    ((0., 0.), (0., 0.), set()),
    ((.5, 0.), (0., .5), set()),
])
def test_end_gap_on_either_side(left_gaps, right_gaps, expected):
    length, height = np.meshgrid(np.linspace(0., 11., 241), np.linspace(1.4, 3.4, 101))
    walls = []
    for lateral, (rear, front) in ((-1.2, left_gaps), (1.2, right_gaps)):
        observed = ((length >= rear) & (length <= 11. - front)) | (height < 3.05)
        walls.append(np.column_stack((np.full(observed.sum(), lateral),
                                      length[observed], height[observed])))
    assert side_ends_needing_bridge(np.vstack(walls), 0., 11., 1.4, 3.4) == expected


def test_each_end_can_be_supported_by_a_different_top_height():
    length, height = np.meshgrid(np.linspace(0., 11., 241), np.linspace(1.4, 3.4, 101))
    left_observed = ((height < 3.05) | ((length >= 2.) & (length <= 9.))
                     | ((length < 2.) & (height <= 3.20)) | ((length > 9.) & (height >= 3.34)))
    left = np.column_stack((np.full(left_observed.sum(), -1.2),
                            length[left_observed], height[left_observed]))
    right = np.column_stack((np.full(length.size, 1.2), length.ravel(), height.ravel()))
    assert not side_ends_needing_bridge(np.vstack((left, right)), 0., 11., 1.4, 3.4)
