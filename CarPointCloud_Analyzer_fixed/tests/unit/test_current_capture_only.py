"""本次点云判空与底板业务测量的执行顺序。"""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.geometry import CoordinateTransform
from carpointcloud_analyzer.domain.models import CarriageSplitResult, LoadStatusResult
from carpointcloud_analyzer.application.pipeline import PointCloudAnalyzer


@pytest.mark.parametrize('status', ['Empty', 'Loaded', 'Uncertain'])
def test_floor_measurement_runs_after_classification_only_for_empty(monkeypatch, status):
    a = PointCloudAnalyzer(Config())
    width, length = np.meshgrid(np.linspace(-.9, .9, 31), np.linspace(.2, 5.8, 61))
    bottom = np.c_[width.ravel(), length.ravel(), np.zeros(width.size)]
    length, height = np.meshgrid(np.linspace(.2, 5.8, 61), np.linspace(0., 2., 21))
    walls = [np.c_[np.full(length.size, side), length.ravel(), height.ravel()]
             for side in (-1.2, 1.2)]
    points = np.vstack((bottom, *walls))
    split = CarriageSplitResult(points, 6., 0., 8., 0., boundaries_are_inner_faces=True)
    profile = a.carriage_analyzer.analyze(points)
    classifier = Mock(return_value=LoadStatusResult(status, .8, 'current', {}))
    monkeypatch.setattr(a.load_classifier, 'classify', classifier)
    floor = Mock(return_value='current-measured-floor')
    monkeypatch.setattr(a, '_analyze_floor_profile', floor)
    result = a._assess_load(split, profile)
    assert len(classifier.call_args.args) == 2
    assert result.status == status
    # 判空只作当前证据分类，底板业务测量必须留到补点后的内长确定之后。
    assert floor.call_count == 0
    measured_floor = a._measure_current_floor(
        split, profile, result, SimpleNamespace(primary_vehicle_points=points),
        CoordinateTransform(np.zeros(3), np.eye(3)),
        np.array([0., 0., 1., 0.]), np.array([0., 0., 1., 0.]), True,
    )
    assert floor.call_count == int(status == 'Empty')
    assert measured_floor == ('current-measured-floor' if status == 'Empty' else None)


def test_insufficient_current_interior_is_uncertain_without_floor_measurement(monkeypatch):
    a = PointCloudAnalyzer(Config(license_plate='TEST001'))
    monkeypatch.setattr(a, '_analyze_floor_profile', Mock(side_effect=AssertionError('floor reuse')))
    split = CarriageSplitResult(np.zeros((2, 3)), 6., 0., 8., 0., boundaries_are_inner_faces=True)
    result = a._assess_load(split, SimpleNamespace(interior_w_range=(-1., 1.)))
    assert result.status == 'Uncertain'
    assert result.reason == 'carriage_interior_grid_insufficient'
