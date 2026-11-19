"""补点和最终云测位对斜杆、竞争杆段使用相同的几何约束。"""

import numpy as np
import pytest
from scipy.spatial import cKDTree

from carpointcloud_analyzer.domain.models import LongitudinalMeasurement, RibInfo
from carpointcloud_analyzer.domain.rib_completion import complete_rib_gaps
from carpointcloud_analyzer.domain.rib_side_measurement import refine_rib_side_positions
from carpointcloud_analyzer.domain.rib_support import validated_rib_support


@pytest.mark.parametrize('mirror', [-1, 1])
@pytest.mark.parametrize('slope', [.9, 1.5])
@pytest.mark.parametrize('axis_scale', [.1, 1., 10.])
def test_transverse_sloped_rod_is_completed_before_final_localization(mirror, slope, axis_scale):
    widths = np.arange(-.95, .951, .005)
    widths = widths[np.abs(widths) > .20]
    centers = np.c_[mirror * widths, 4. + .06 * widths, 2. + slope * widths]
    angles = np.arange(8) * np.pi / 4
    offsets = np.c_[np.zeros(8), .004 * np.cos(angles), .004 * np.sin(angles)]
    points = (centers[:, None, :] + offsets).reshape(-1, 3)
    direction = np.array([mirror, .06, slope])
    direction = direction / np.linalg.norm(direction) * axis_scale
    rib = RibInfo(1, 4., 0., 2., (-.9, .9), (0., 4.), len(points), 1.8, .03,
                  axis_midpoint=(0., 4., 2.), axis_direction=tuple(direction),
                  axis_observed_w_range=(-.9, .9))
    original = points.copy()
    assert validated_rib_support(points, cKDTree(points), rib, 0., 4.) is not None

    added, info = complete_rib_gaps(points, [rib], (-1.12, 1.12), 0., 9., 0., 4.)

    assert info['completed'] is True
    assert np.count_nonzero(np.abs(added[:, 0]) < .15) > 0
    final = np.vstack((points, added))
    measurement = LongitudinalMeasurement(0., 9., (0., 0., mirror * -1.12),
                                        {'selectedSide': 'left' if mirror == 1 else 'right'})
    row = refine_rib_side_positions(final, [rib], measurement, 0., 4., final_cloud=True)[0]
    assert row['valid'], row
    assert row['positionM'] == pytest.approx(4. - .06 * 1.12, abs=.003)
    np.testing.assert_array_equal(points, original)


@pytest.mark.parametrize('mirror', [-1, 1])
def test_final_cloud_competing_axes_take_precedence_over_sparse_local_window(mirror):
    widths = np.r_[np.arange(-1.025, -.20, .05), np.arange(-.195, 1.046, .005)]
    points = np.c_[mirror * widths, np.full(len(widths), 4.), np.full(len(widths), 1.5)]
    points = np.vstack((points + [0., .026, 0.], points - [0., .026, 0.]))
    rib = RibInfo(1, 4., 0., 1.5, (-.55, .55), (1., 2.), len(points), 1.1, .03,
                  axis_midpoint=(0., 4., 1.5), axis_direction=(mirror, 0., 0.),
                  axis_observed_w_range=(-.55, .55))
    measurement = LongitudinalMeasurement(0., 9., (0., 0., mirror * -1.12),
                                        {'selectedSide': 'left' if mirror == 1 else 'right'})

    added, info = complete_rib_gaps(points, [rib], (-1.12, 1.12), 0., 9., 0., 2.4)
    row = refine_rib_side_positions(np.vstack((points, added)), [rib], measurement,
                                    0., 2.4, final_cloud=True)[0]

    assert len(added) == 0
    assert info['ribs'][0]['supportValidation']['reason'] == 'multiple_whole_rib_axes'
    assert row['valid'] is False
    assert row['positionM'] is None
    assert row['reason'] == 'multiple_whole_rib_axes'
    assert row['localWindowReason'] in ('insufficient_local_rib_support', 'disconnected_local_rib_support')
    assert row['wholeRibSupport']['reason'] == row['reason']
