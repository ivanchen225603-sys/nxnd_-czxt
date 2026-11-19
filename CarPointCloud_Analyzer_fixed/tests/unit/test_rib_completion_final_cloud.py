"""横杆补洞和结果去来源标记的最终云契约。"""

import numpy as np
import pytest

from carpointcloud_analyzer.domain.carriage import CarriageAnalyzer
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.application.exporters import ResultExporter
from carpointcloud_analyzer.domain.models import CarriageSplitResult, RibInfo
from carpointcloud_analyzer.application.pipeline import PointCloudAnalyzer
from carpointcloud_analyzer.domain.rib_completion import complete_rib_gaps


def _rib() -> RibInfo:
    return RibInfo(
        1, 2., 0., 1., (-.4, .4), (.98, 1.02), 80, 1.8, .04,
        axis_midpoint=(0., 2., 1.), axis_direction=(1., 0., 0.),
        axis_observed_w_range=(-.4, .4),
    )


def test_rib_completion_fills_only_missing_w_cells_without_moving_input():
    widths = np.r_[np.arange(-.8, -.1, .01), np.arange(.1, .81, .01)]
    width, offset = np.meshgrid(widths, [-.004, 0., .004])
    existing = np.c_[width.ravel(), 2. + offset.ravel(), np.ones(width.size)]
    before = existing.copy()
    added, info = complete_rib_gaps(existing, [_rib()], (-1., 1.), 0., 4., 0., 2.)
    np.testing.assert_array_equal(existing, before)
    assert info['completed'] is True
    assert info['pointCount'] == len(added)
    assert len(added) > 0
    assert np.all((added[:, 0] > -1.) & (added[:, 0] < 1.))
    assert np.all(np.abs(added[:, 1] - 2.) < .00801)
    assert np.all(np.abs(added[:, 2] - 1.) < .00801)
    assert np.any(np.abs(added[:, 0]) < .04)
    # 当前连续杆段距壁不到30cm：末端缺口也要补，仍保留壁边间隔。
    assert added[:, 0].min() >= -1. + .055 - 1e-9
    assert added[:, 0].max() <= 1. - .055 + 1e-9
    assert added[:, 0].min() < existing[:, 0].min()


def test_rib_completion_rejects_unanchored_or_out_of_box_axis():
    points = np.array([[0., 2., 1.]])
    bad = _rib()
    bad.axis_observed_w_range = (-.1, .1)
    added, info = complete_rib_gaps(points, [bad], (-1., 1.), 0., 4., 0., 2.)
    assert len(added) == 0
    assert info['completed'] is False


def test_result_removes_point_origin_labels_but_keeps_values_and_file_identity():
    value = {
        'sourceFile': 'scan.pcd',
        'dimensions': {'length': 11.2, 'source': 'current_observed_points'},
        'measurementStatus': {'lengthMeasurement': {
            'ribPositionSource': 'local_observed_axis', 'ribIntervalsM': [2., 3.]}},
        'wallCompletion': {'synthesizedPoints': 42,
                           'syntheticUsedForLoadEvidence': False},
    }
    cleaned = ResultExporter._without_point_origin_labels(value)
    assert cleaned['sourceFile'] == 'scan.pcd'
    assert cleaned['dimensions'] == {'length': 11.2}
    assert cleaned['measurementStatus']['lengthMeasurement'] == {'ribIntervalsM': [2., 3.]}
    assert cleaned['wallCompletion'] == {'synthesizedPoints': 42}


@pytest.mark.parametrize('surface_h, expected', [(0., 'Empty'), (1.4, 'Loaded')])
def test_final_cloud_load_uses_geometry_not_wall_density(surface_h, expected):
    cfg = Config()
    analyzer = PointCloudAnalyzer(cfg)
    width, longitudinal = np.meshgrid(np.linspace(-.9, .9, 41),
                                      np.linspace(.2, 5.8, 101))
    interior = np.c_[width.ravel(), longitudinal.ravel(),
                     np.full(width.size, surface_h)]
    length, height = np.meshgrid(np.linspace(0., 6., 101), np.linspace(0., 2., 25))
    walls = np.vstack([np.c_[np.full(length.size, side), length.ravel(), height.ravel()]
                       for side in (-1.2, 1.2)])
    front_w, front_h = np.meshgrid(np.linspace(-.9, .9, 31), np.linspace(0., 2., 31))
    front = np.c_[front_w.ravel(), np.zeros(front_w.size), front_h.ravel()]
    final = np.vstack((interior, walls, front))
    split = CarriageSplitResult(np.vstack((interior, walls)), 6., 0., 10., 0.,
                                boundaries_are_inner_faces=True)
    profile = CarriageAnalyzer(cfg).analyze(final)
    result = analyzer._assess_load(
        split, profile,
        final_carriage_points=final,
    )
    assert result.status == expected
