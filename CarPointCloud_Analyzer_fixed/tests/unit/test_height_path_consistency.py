"""缺壁不借旧轮廓补高度；短矩形与普通车厢使用同一上沿口径。"""

import json
from types import SimpleNamespace

import numpy as np
import pytest

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.dimensions import _measure_continuous_side_height
from carpointcloud_analyzer.domain.models import BoxCarriageBox
from carpointcloud_analyzer.domain.rectangular_completion import completed_inner_dimensions


def test_missing_wall_sections_cannot_reintroduce_rope_height_through_profile():
    width, length = np.meshgrid(np.arange(-1.2, 1.201, .06), np.arange(0., 10.001, .06))
    floor = np.c_[width.ravel(), length.ravel(), np.full(width.size, 1.6)]
    length, height = np.meshgrid(np.arange(3., 4.001, .06), np.arange(1.6, 3.601, .06))
    walls = [np.c_[np.full(length.size, side), length.ravel(), height.ravel()] for side in (-1.2, 1.2)]
    longitudinal = np.arange(.5, 9.501, .05)
    ropes = [np.c_[np.full(len(longitudinal), side), longitudinal, np.full(len(longitudinal), 3.74)]
             for side in (-1.2, 1.2)]
    points = np.vstack((floor, *walls, *ropes))
    original = points.copy()
    profile = SimpleNamespace(floor_z=1.6, wall_top=3.74, interior_w_range=(-1.05, 1.05))
    measured, info = _measure_continuous_side_height(points, profile, Config())
    assert np.isnan(measured)
    assert info['heightM'] is None
    assert info['reason'] == 'insufficient_supported_side_sections'
    assert max(info['supportBinCount'].values()) < 6
    json.dumps(info, allow_nan=False)
    np.testing.assert_array_equal(points, original)


@pytest.mark.parametrize('right_top,ropes,expected', [(3.6, True, 2.), (2.6, False, 1.5)])
def test_short_box_height_excludes_ropes_and_pools_both_sides(right_top, ropes, expected):
    parts = []
    for side, top in [(-1.2, 3.6), (1.2, right_top)]:
        length, height = np.meshgrid(np.arange(.05, 5.951, .05), np.arange(1.6, top + .001, .05))
        parts.append(np.c_[np.full(length.size, side), length.ravel(), height.ravel()])
    width, height = np.meshgrid(np.arange(-1.2, 1.201, .05), np.arange(1.6, 3.601, .05))
    parts.extend([np.c_[width.ravel(), np.full(width.size, end), height.ravel()] for end in (0., 6.)])
    width, length = np.meshgrid(np.arange(-1.1, 1.101, .05), np.arange(.05, 5.951, .05))
    parts.append(np.c_[width.ravel(), length.ravel(), np.full(width.size, 1.6)])
    if ropes:
        length = np.arange(.5, 5.501, .05)
        parts.extend([np.c_[np.full(len(length), side), length, np.full(len(length), 3.68 + level * .08)]
                      for side in (-1.2, 1.2) for level in range(3)])
    observed = np.vstack(parts)
    synthetic = np.array([[-1.2, 3., 3.6]])
    original = observed.copy()
    box = BoxCarriageBox(6., 0., (-1.2, 1.2), (1.6, 3.6), 3000, 3000)
    result = completed_inner_dimensions(observed, synthetic, box, 1.6, 3.6)
    assert result is not None
    assert result['heightM'] == pytest.approx(expected, abs=.05)
    assert result['heightMeasurement']['aggregation'] == 'pooled_valid_side_sections_median'
    assert result['heightMeasurement']['selectedSide'] == 'both'
    json.dumps(result, allow_nan=False)
    np.testing.assert_array_equal(observed, original)
