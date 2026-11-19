"""可见底板的短箱不因车头/车架主平面换掉独立高度轴。"""

from types import SimpleNamespace

import numpy as np
import pytest

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.application.pipeline import PointCloudAnalyzer
from carpointcloud_analyzer.application.pipeline import _AnalysisMode
from carpointcloud_analyzer.domain.geometry import CoordinateTransform
from carpointcloud_analyzer.domain.models import BoxCarriageBox
from test_observed_rectangular_body import _box_with_visible_floor_and_cab


def _scene(points, vertical):
    return SimpleNamespace(preprocessed=SimpleNamespace(primary_vehicle_points=points),
                           vertical_reference=vertical, vertical_source='weighbridge')


@pytest.mark.parametrize('tilt', [0., 17.])
def test_visible_box_retains_independent_vertical_despite_lower_exterior_plane(monkeypatch, tilt):
    points = _box_with_visible_floor_and_cab()
    w, length = np.meshgrid(np.linspace(-1., 1., 101), np.linspace(7., 14., 201))
    lower_chassis = np.c_[w.ravel(), length.ravel(), .04 * length.ravel() + .9]
    points = np.vstack((points, lower_chassis))
    angle = np.deg2rad(tilt)
    rotation = np.array([[1., 0., 0.], [0., np.cos(angle), -np.sin(angle)],
                         [0., np.sin(angle), np.cos(angle)]])
    source = points @ rotation.T + [3., 9., -2.]
    vertical = rotation[:, 2]
    analyzer = PointCloudAnalyzer(Config(statistical_outlier_enabled=False))

    def forbidden(*args):
        pytest.fail('verified short box must not be reoriented by a vehicle support plane')

    monkeypatch.setattr(analyzer.geometry, 'process', forbidden)
    local, transform, box = analyzer._standard_coordinates(_scene(source, vertical))
    assert box.length == pytest.approx(6., abs=.08)
    np.testing.assert_allclose(transform.rotation[:, 2], vertical, atol=1e-12)
    # 排序不是移动点；变换可逆，任何已有点不得被补点过程替换。
    reconstructed = transform.to_source(local)
    np.testing.assert_allclose(reconstructed, source[np.lexsort(source.T[::-1])], atol=1e-12)


@pytest.mark.parametrize('candidate_available', [False, True])
def test_unrecognized_body_keeps_original_standard_normalizer(monkeypatch, candidate_available):
    import carpointcloud_analyzer.application.pipeline as pipeline

    analyzer = PointCloudAnalyzer(Config())
    points = np.array([[0., 0., 1.]])
    scene = _scene(points, np.array([0., 0., 1.]))
    normalized, transform = np.array([[1., 2., 3.]]), object()
    candidate = (points.copy(), object()) if candidate_available else None
    monkeypatch.setattr(analyzer.geometry, 'vertical_candidate', lambda *args: candidate)
    monkeypatch.setattr(pipeline, 'observe_rectangular_body', lambda *args: None)
    monkeypatch.setattr(pipeline, 'align_observed_rectangular_body', lambda *args: None)
    calls = []

    def original(*args):
        calls.append(args)
        return normalized, transform

    monkeypatch.setattr(analyzer.geometry, 'process', original)
    result = analyzer._standard_coordinates(scene)
    assert result[0] is normalized
    assert result[1] is transform
    assert result[2] is None
    assert len(calls) == 1
    assert calls[0][0] is points
    assert calls[0][1] is scene.vertical_reference


def test_heading_flip_preserves_verified_box_instead_of_reidentifying(monkeypatch):
    import carpointcloud_analyzer.application.pipeline as pipeline

    analyzer = PointCloudAnalyzer(Config())
    points = np.array([[.4, -8., 1.4], [-.5, 6., 4.1]])
    box = BoxCarriageBox(6., 0., (-1.3, 1.2), (1.43, 4.1), 123, 456)
    transform = CoordinateTransform(np.zeros(3), np.eye(3))
    scene = _scene(points, np.array([0., 0., 1.]))
    scene.site_frame = scene.height_plane_analysis = scene.height_plane_source = None
    scene.height_plane_calibrated = False
    monkeypatch.setattr(analyzer, '_standard_coordinates', lambda _: (points, transform, box))

    def forbidden(*args):
        pytest.fail('a rigid heading flip must not re-detect or re-bin the verified box')

    monkeypatch.setattr(pipeline, 'observe_rectangular_body', forbidden)
    calls = []

    class SplitCaptured(Exception):
        pass

    def capture(local, current):
        calls.append((local, current))
        if len(calls) == 2:
            raise SplitCaptured
        return SimpleNamespace(tail_y=-8., head_y=6.2)

    monkeypatch.setattr(analyzer.local_splitter, 'split_observed_box', capture)
    with pytest.raises(SplitCaptured):
        analyzer._analyze_standard(scene, _AnalysisMode('memory.pcd', False, False, False))
    flipped_points, flipped_box = calls[1]
    np.testing.assert_array_equal(flipped_points, points * [-1., -1., 1.])
    assert flipped_box.h_range == box.h_range
    assert (flipped_box.rear_l, flipped_box.front_l) == (-6., 0.)
    assert flipped_box.w_range == (-1.2, 1.3)
    assert (flipped_box.front_points, flipped_box.rear_points) == (456, 123)
