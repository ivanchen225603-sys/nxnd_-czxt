"""曲杆候选失败可有界重试，真实双轴/台阶不能通过换模型被掩盖。"""

import numpy as np
import pytest
from scipy.spatial import cKDTree

from carpointcloud_analyzer.domain import rib_support
from carpointcloud_analyzer.domain.models import RibInfo


@pytest.mark.parametrize('initial_reason', [
    'whole_rib_shape_unresolved', 'multiple_whole_rib_axes',
    'persistent_whole_rib_axis_step', 'insufficient_whole_rib_points',
])
@pytest.mark.parametrize('alternative_reason', [None, 'whole_rib_shape_unresolved',
                                               'multiple_whole_rib_axes',
                                               'persistent_whole_rib_axis_step'])
def test_only_shape_failure_retries_once_from_same_cloud_and_seed(
        monkeypatch, initial_reason, alternative_reason):
    values, tree, seed = object(), object(), object()
    candidate = object()
    calls = []

    def validate(*args, **kwargs):
        calls.append((args, kwargs))
        if len(calls) == 1:
            kwargs['diagnostics']['reason'] = initial_reason
            return None
        assert kwargs['height_degrees'] == (3,)
        if alternative_reason is None:
            return candidate
        kwargs['diagnostics']['reason'] = alternative_reason
        return None

    monkeypatch.setattr(rib_support, '_validated_rib_profile', validate)
    diagnostics = {}
    result = rib_support.validated_rib_support(values, tree, seed, 0., 4., diagnostics=diagnostics)
    retry = initial_reason == 'whole_rib_shape_unresolved'
    assert len(calls) == (2 if retry else 1)
    for args, kwargs in calls:
        assert args == (values, tree, seed, 0., 4.)
        assert kwargs['_profile'] is None
        assert kwargs['_passes_remaining'] == 2
    if retry and alternative_reason is None:
        assert result is candidate
        assert diagnostics == {}
    else:
        assert result is None
        assert diagnostics['reason'] == (alternative_reason if retry else initial_reason)


@pytest.mark.parametrize('mirror', [-1, 1])
def test_cubic_rod_accepts_local_fill_without_changing_existing_points(mirror):
    widths = np.arange(-1.05, 1.151, .005)
    centers = np.c_[mirror * widths, 4. + .02 * widths,
                    1.2 - .55 * widths + .4 * widths**2 - .25 * widths**3]
    angles = np.arange(8) * np.pi / 4
    offsets = np.c_[np.zeros(8), .004 * np.cos(angles), .004 * np.sin(angles)]
    points = (centers[:, None, :] + offsets).reshape(-1, 3)
    w = -.10
    fill = np.array([[mirror * w, 4. + .02 * w + dl,
                      1.2 - .55 * w + .4 * w**2 - .25 * w**3 + dh]
                     for dl in (-.008, 0., .008) for dh in (-.008, 0., .008)])
    seed = RibInfo(1, 4., 0., 1.22, (-.8, .8), (0., 4.), len(points), 1.6, .03,
                   axis_midpoint=(0., 4., 1.22), axis_direction=(mirror, .02, -.51),
                   axis_observed_w_range=(-.8, .8))
    original = points.copy()
    models = []
    for cloud in (points, np.vstack((points, fill))):
        model = rib_support.validated_rib_support(cloud, cKDTree(cloud), seed, 0., 4.)
        assert model is not None
        assert len(model.height_coefficients) == 4
        assert np.max(model.residual) <= .035
        models.append(model)
    assert models[0].evaluate(np.array([-1.]))[0, 1] == pytest.approx(
        models[1].evaluate(np.array([-1.]))[0, 1], abs=.003)
    np.testing.assert_array_equal(points, original)
