"""纵向连续性区分完整分箱内的缺口与不足一格的端面余量。"""
import numpy as np

from carpointcloud_analyzer.domain.heading import longitudinally_continuous


def test_continuity_ignores_only_incomplete_terminal_bin():
    width, length, height = np.meshgrid([-.5, .5], np.arange(0., 6., .05), [0., .5])
    points = np.c_[width.ravel(), length.ravel(), height.ravel()]
    assert longitudinally_continuous(points, 0., 6.000001)
    missing = points[(points[:, 1] < 2.) | (points[:, 1] >= 2.4)]
    assert not longitudinally_continuous(missing, 0., 6.000001)
