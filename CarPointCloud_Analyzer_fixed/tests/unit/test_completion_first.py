"""最终云判空：端壁结构带不能被当作货物填充率。"""

from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from carpointcloud_analyzer.application import pipeline
from carpointcloud_analyzer.domain.carriage import CarriageAnalyzer, spatially_balanced_cloud
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.geometry import CoordinateTransform
from carpointcloud_analyzer.domain.models import CarriageSplitResult, LongitudinalMeasurement, VehicleDimensions


def _current_cloud(surface_height, *, interior_visible=True):
    """可见平面可为底板或货面；墙顶均为本次实测 2 m，不借历史高度。"""
    lengths, heights = np.meshgrid(np.linspace(0., 6., 101), np.linspace(surface_height, 2., 25))
    walls = np.vstack([
        np.c_[np.full(lengths.size, side), lengths.ravel(), heights.ravel()]
        for side in (-1.2, 1.2)
    ])
    if not interior_visible:
        return walls
    width, length = np.meshgrid(np.linspace(-.9, .9, 41), np.linspace(.2, 5.8, 101))
    surface = np.c_[width.ravel(), length.ravel(), np.full(width.size, surface_height)]
    return np.vstack([surface, walls])


@pytest.mark.parametrize("surface_height,interior_visible,expected", [
    (0., True, "Empty"),
    (1.4, True, "Loaded"),
    (1.8, True, "Uncertain"),
    (0., False, "Uncertain"),
])
@pytest.mark.parametrize("synthetic_repeats", [0, 1, 250])
def test_completion_precedes_final_cloud_classification(
    monkeypatch, surface_height, interior_visible, expected, synthetic_repeats,
):
    measured = _current_cloud(surface_height, interior_visible=interior_visible)
    original = measured.copy()
    # 模拟极高密度的合成端壁；它们虽落在内部 W 带，也不属于内部实测支持。
    width, height = np.meshgrid(np.linspace(-.9, .9, 15), np.linspace(.1, 2., 15))
    synthetic = np.tile(np.c_[width.ravel(), np.zeros(width.size), height.ravel()],
                        (synthetic_repeats, 1))
    cfg = Config(save_clean_pointcloud=False)
    analyzer = pipeline.PointCloudAnalyzer(cfg)
    transform = CoordinateTransform(np.zeros(3), np.eye(3))
    split = CarriageSplitResult(measured, 6., 0., 10., 0., boundaries_are_inner_faces=True)
    profile = CarriageAnalyzer(cfg).analyze(measured)
    trace = []

    monkeypatch.setattr(analyzer.geometry, "process", lambda *args: (measured, transform))
    monkeypatch.setattr(analyzer.local_splitter, "split", lambda *args: split)
    monkeypatch.setattr(pipeline, "cab_requires_heading_reversal", lambda *args: False)
    monkeypatch.setattr(analyzer.dim_calc, "measure_cab_to_front_wall_at_height", Mock())
    monkeypatch.setattr(analyzer.carriage_analyzer, "analyze", lambda *args: profile)

    def complete(vehicle, carriage, *args):
        trace.append("completion")
        np.testing.assert_array_equal(vehicle, original)
        np.testing.assert_array_equal(carriage, original)
        return SimpleNamespace(points=synthetic, metadata={
            "completed": bool(len(synthetic)), "synthesizedPoints": len(synthetic),
        })

    monkeypatch.setattr(pipeline, "complete_empty_carriage_walls", complete)
    assess = analyzer._assess_load

    def assess_after_completion(*args, **kwargs):
        trace.append("classification")
        np.testing.assert_array_equal(args[0].carriage_points, original)
        np.testing.assert_array_equal(
            kwargs['final_carriage_points'],
            np.vstack((original, synthetic)) if len(synthetic) else original)
        return assess(*args, **kwargs)

    monkeypatch.setattr(analyzer, "_assess_load", assess_after_completion)
    classifier = Mock(wraps=analyzer.load_classifier.classify)
    monkeypatch.setattr(analyzer.load_classifier, "classify", classifier)
    measurement = LongitudinalMeasurement(0., 6., (0., 0., 1.), {"valid": True})

    def final_length(points, *args, **kwargs):
        trace.append("length")
        np.testing.assert_array_equal(
            points, np.vstack((original, synthetic)) if len(synthetic) else original)
        return measurement

    monkeypatch.setattr(pipeline, "measure_final_inner_end_faces", final_length)

    def dimensions(all_points, carriage_points, *args, **kwargs):
        trace.append("dimensions")
        # 最终云用于结构尺寸；实测点仍是外沿与原始覆盖诊断的独立来源。
        np.testing.assert_array_equal(
            all_points, np.vstack((original, synthetic)) if len(synthetic) else original)
        np.testing.assert_array_equal(
            carriage_points,
            np.vstack((original, synthetic)) if len(synthetic) else original)
        return VehicleDimensions(10., 2.4, 2., 6., 2.2, profile.wall_depth)

    monkeypatch.setattr(analyzer.dim_calc, "calculate", dimensions)
    floor = Mock(return_value=None)
    monkeypatch.setattr(analyzer, "_analyze_floor_profile", floor)
    ribs = Mock(return_value=[])
    monkeypatch.setattr(analyzer.local_rib_detector, "detect", ribs)
    monkeypatch.setattr(pipeline, "refine_rib_side_positions", lambda *args, **kwargs: {})

    def export(*args, **kwargs):
        return dict(integrationPayloads={}, dimensions=None, measurementStatus={},
                    loadStatus=args[3].to_dict(), wallCompletion=kwargs["wall_completion"])

    monkeypatch.setattr(pipeline.ResultExporter, "build", export)
    scene = pipeline._SceneContext(
        SimpleNamespace(primary_vehicle_points=measured, ground_plane=None),
        None, None, None, False, measured, np.array([0., 0., 1.]), "test",
    )
    result = analyzer._analyze_standard(scene, pipeline._AnalysisMode("test.pcd", False, False, False))

    assert trace[:3] == ["completion", "classification", "length"]
    assert trace[-1] == "dimensions"
    assert "completed_length" not in trace
    assert result["loadStatus"]["status"] == expected
    assert result["wallCompletion"]["stage"] == "before_load_classification"
    if interior_visible:
        classifier.assert_called_once()
        np.testing.assert_array_equal(
            classifier.call_args.args[0],
            spatially_balanced_cloud(
                original[(original[:, 1] > .12) & (original[:, 1] < 5.88)], cfg))
        assert classifier.call_args.args[1] is profile
    else:
        classifier.assert_not_called()
        assert result["loadStatus"]["reason"] == "carriage_interior_grid_insufficient"
    if expected == "Empty":
        assert result["syntheticPointCount"] == len(synthetic)
        floor.assert_called_once()
        np.testing.assert_array_equal(
            floor.call_args.args[0].carriage_points,
            np.vstack((original, synthetic)) if len(synthetic) else original)
    else:
        assert result["syntheticPointCount"] == 0
        assert result["wallCompletion"]["completed"] is False
        floor.assert_not_called()
    assert ribs.call_count == 2
    np.testing.assert_array_equal(
        ribs.call_args.args[0],
        np.vstack((original, synthetic)) if len(synthetic) else original)
    np.testing.assert_array_equal(measured, original)
    np.testing.assert_array_equal(split.carriage_points, original)
