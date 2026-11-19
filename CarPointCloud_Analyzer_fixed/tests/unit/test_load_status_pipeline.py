import numpy as np

from carpointcloud_analyzer import (
    CarriageSplitResult,
    Config,
    LoadStatusResult,
    PointCloudAnalyzer,
    VehicleDimensions,
)

from carpointcloud_analyzer.domain.geometry import CoordinateTransform
from carpointcloud_analyzer.domain.preprocessing import PreprocessResult


def test_load_grid_excludes_side_walls():
    floor = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.5, 1.0, 0.0],
        ]
    )
    walls = np.array(
        [
            [-1.2, 0.0, 2.0],
            [1.2, 1.0, 2.0],
        ]
    )

    selected = PointCloudAnalyzer._interior_grid_points(
        np.vstack([floor, walls]),
        (-1.0, 1.0),
    )

    assert np.array_equal(selected, floor)


def test_unobservable_interior_omits_dimensions_and_skips_empty_rib_detection(
    tmp_path
):
    cfg = Config()
    cfg.camera_identifier = "TEST-CAM"
    analyzer = PointCloudAnalyzer(cfg)
    points = np.column_stack([
        np.linspace(-1.0, 1.0, 100),
        np.linspace(0.0, 5.0, 100),
        np.linspace(2.0, 3.0, 100),
    ])
    split = CarriageSplitResult(
        carriage_points=points,
        split_y=5.0,
        carriage_start_y=0.0,
        head_y=6.0,
        tail_y=0.0,
    )
    dims = VehicleDimensions(6.0, 2.0, 3.0, 5.0, 2.0, 0.5)

    analyzer.loader.load = lambda pcd_path: points
    analyzer.preprocessor.process = lambda pts: PreprocessResult(
        vehicle_points=pts,
        primary_vehicle_points=pts,
        vehicle_points_list=[pts],
        primary_index=0,
        ground_plane=None,
        applied=False,
        stats={},
    )
    analyzer.geometry.process = (
        lambda pts, vertical=None, source="none":
        (pts, CoordinateTransform(np.zeros(3), np.eye(3)))
    )
    analyzer.local_splitter.split = lambda pts: split
    analyzer.dim_calc.calculate = (
        lambda all_pts, carriage_pts, split_result, profile=None, ground_plane_local=None, synthetic_point_count=0: dims
    )
    analyzer.load_classifier.classify = (
        lambda carriage_pts, profile=None:
        LoadStatusResult("Uncertain", 0.4, "fallback", {})
    )
    def fail_if_rib_detection_runs(*args, **kwargs):
        raise AssertionError("Uncertain 状态不得执行空车专用拉筋检测")

    analyzer.local_rib_detector.detect = fail_if_rib_detection_runs
    outputs = analyzer.analyze(str(tmp_path / "scan.pcd"))

    assert outputs["png"] == ""
    assert outputs["json"] == ""
    assert outputs["integrationPayloads"] == {}
