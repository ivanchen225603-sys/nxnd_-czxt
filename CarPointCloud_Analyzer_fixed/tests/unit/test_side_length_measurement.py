"""合成几何真值回归，不能把现场某辆车的目标长度硬编码到算法。"""

from dataclasses import replace

import numpy as np
import pytest

from carpointcloud_analyzer.domain.longitudinal_measurement import (
    rib_positions,
)
from carpointcloud_analyzer.domain.models import LongitudinalMeasurement, RibInfo


def test_rib_projection_rejects_invalid_axis_plane_and_out_of_length_support():
    m = LongitudinalMeasurement(0., 9., (0., 0., 1.), {})
    rib = RibInfo(1, 4., 0., 1.7, (-1., 1.), (1.65, 1.75), 500, 2., .1,
                  axis_midpoint=(0., 4., 1.7), axis_direction=(2., .2, 0.),
                  axis_observed_w_range=(-1.1, 1.1))
    assert rib_positions([rib], m) == pytest.approx([4.1])
    for changed in [replace(rib, axis_direction=(0., 1., 0.)),
                    replace(rib, axis_direction=(np.nan, 1., 0.)),
                    replace(rib, axis_midpoint=(0., 10., 1.7)),
                    replace(rib, axis_midpoint=(0.,)),
                    replace(rib, axis_observed_w_range=(-.1, .1))]:
        assert rib_positions([changed], m) is None
    assert rib_positions([rib], replace(m, side_plane=(np.nan, 0., 0.))) is None


def test_crossing_axes_do_not_silently_change_rib_order():
    m = LongitudinalMeasurement(0., 9., (0., 0., 1.), {})
    front = RibInfo(1, 4., 0., 1.7, (-1., 1.), (1.65, 1.75), 500, 2., .1,
                    axis_midpoint=(0., 4., 1.7), axis_direction=(1., -.4, 0.),
                    axis_observed_w_range=(-1.1, 1.1))
    rear = replace(front, rib_id=2, y_position=3.8, axis_midpoint=(0., 3.8, 1.7),
                   axis_direction=(1., .4, 0.))
    assert rib_positions([front, rear], m) is None
