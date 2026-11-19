"""净宽与原外沿、搜索带、上传尺寸的职责分离。"""

import numpy as np
import pytest

from carpointcloud_analyzer.domain.carriage import CarriageAnalyzer
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.dimensions import DimensionCalculator
from carpointcloud_analyzer.application.exporters import ResultExporter
from carpointcloud_analyzer.domain.models import CarriageSplitResult
from types import SimpleNamespace


def test_measured_inner_width_does_not_rename_outer_width_or_change_search_mask():
    ll, hh = np.meshgrid(np.linspace(0., 9., 240), np.linspace(0., 2., 70))
    sides = [np.column_stack((np.full(ll.size, w), ll.ravel(), hh.ravel()))
             for w in (-1.3, -1.2, 1.2, 1.3)]
    fl, fw = np.meshgrid(np.linspace(0., 9., 180), np.linspace(-1.2, 1.2, 60))
    points = np.vstack((*sides, np.column_stack((fw.ravel(), fl.ravel(), np.zeros(fl.size)))))
    before = points.copy()
    cfg = Config()
    profile = CarriageAnalyzer(cfg).analyze(points)
    bounds = profile.interior_w_range
    mask = profile.interior_mask.copy()
    split = CarriageSplitResult(points, 9., 0., 9., 0., boundaries_are_inner_faces=True)
    dimensions = DimensionCalculator(cfg).calculate(points, points, split, profile)
    assert dimensions.carriage_width == pytest.approx(2.4, abs=.01)
    assert dimensions.carriage_outer_width == pytest.approx(2.6, abs=.01)
    assert dimensions.vehicle_width >= dimensions.carriage_outer_width
    assert dimensions.width_measurement['adoptedSource'] == 'observed_inner_side_faces'
    assert profile.interior_w_range == bounds
    np.testing.assert_array_equal(profile.interior_mask, mask)
    np.testing.assert_array_equal(points, before)
    exported = ResultExporter._build_carriage_profile(profile, dimensions)
    assert exported['outerWidthMeasured'] == pytest.approx(2.6, abs=.01)
    assert exported['interiorSearchWidth'] == pytest.approx(bounds[1] - bounds[0], abs=.001)
    payload = ResultExporter._build_vehicle_dimension_payload(cfg, [], dimensions, split, '')
    assert payload['dimensions']['trailerWidth'] == pytest.approx(240., abs=1.)
    assert set(dimensions.to_dict()['carriage']) == {'length', 'width', 'height'}


def test_missing_side_width_is_measured_from_final_completed_cloud():
    ll, hh = np.meshgrid(np.linspace(0., 9., 240), np.linspace(0., 2., 70))
    left = np.column_stack((np.full(ll.size, -1.2), ll.ravel(), hh.ravel()))
    right = left.copy()
    right[:, 0] = 1.2
    observed = np.vstack((left, right[right[:, 1] < 1.]))
    completed = np.vstack((observed, right[right[:, 1] >= 1.]))
    split = CarriageSplitResult(observed, 9., 0., 9., 0.,
                                boundaries_are_inner_faces=True)
    profile = SimpleNamespace(floor_z=0., wall_top=2., wall_depth=2.)

    dimensions = DimensionCalculator(Config()).calculate(
        completed, completed, split, profile,
        synthetic_point_count=len(completed) - len(observed),
    )

    assert dimensions.carriage_width == pytest.approx(2.4, abs=.01)
    assert dimensions.width_measurement['adoptedSource'] == 'completed_cloud_inner_side_faces'
    assert dimensions.width_measurement['source'] == 'current_final_carriage_cloud'
    assert dimensions.width_measurement['completionUsed'] is True
    assert dimensions.width_measurement['syntheticPointCount'] == len(completed) - len(observed)
    np.testing.assert_array_equal(observed[:len(left)], left)
