import os
from pathlib import Path

import numpy as np
import pytest

from carpointcloud_analyzer.api import cli
from carpointcloud_analyzer.infrastructure import loader as loader_module
from carpointcloud_analyzer.api.cli import parse_cli_args
from carpointcloud_analyzer.application.exporters import ResultExporter
from carpointcloud_analyzer.domain.floor import FloorProfile, FloorSegment
from carpointcloud_analyzer.infrastructure.loader import PointCloudLoader
from carpointcloud_analyzer.domain.utils.histogram import robust_span
from carpointcloud_analyzer import (
    CarriageSplitResult,
    Config,
    LoadStatusResult,
    RibInfo,
    VehicleDimensions,
)


def test_parse_cli_args_accepts_all_runtime_options():
    args = parse_cli_args(
        [
            "cli.py",
            "scan.pcd",
            "--camera",
            "CAM-01",
            "--plate",
            "A12345",
            "--baseline-dir",
            "D:/tmp/baselines",
        ]
    )

    assert args == {
        "pcd_path": "scan.pcd",
        "pcd_paths": ["scan.pcd"],
        "camera": "CAM-01",
        "plate": "A12345",
        "baseline_dir": "D:/tmp/baselines",
    }


def test_parse_cli_args_reads_environment_defaults(monkeypatch):
    monkeypatch.setenv("CARPOINT_CAMERA_IDENTIFIER", "ENV-CAM")
    monkeypatch.setenv("CARPOINT_LICENSE_PLATE", "ENV-PLATE")
    monkeypatch.setenv("CARPOINT_BASELINE_DIR", "D:/env/baselines")

    args = parse_cli_args(["cli.py", "scan.pcd"])

    assert args["camera"] == "ENV-CAM"
    assert args["plate"] == "ENV-PLATE"
    assert args["baseline_dir"] == "D:/env/baselines"
    assert args["pcd_paths"] == ["scan.pcd"]


def test_parse_cli_args_accepts_multiple_pcd_paths():
    args = parse_cli_args(
        [
            "cli.py",
            "scan1.pcd",
            "scan2.pcd",
        ]
    )

    assert args["pcd_path"] == "scan1.pcd"
    assert args["pcd_paths"] == ["scan1.pcd", "scan2.pcd"]


def test_main_reuses_one_analyzer_for_batch(monkeypatch):
    created_configs = []
    analyzed_paths = []

    class FakeAnalyzer:
        def __init__(self, config):
            created_configs.append(config)

        def analyze(self, pcd_path):
            analyzed_paths.append(pcd_path)
            return {"png": "", "json": f"{pcd_path}.json"}

    monkeypatch.setattr(cli.sys, "argv", ["cli.py", "a.pcd", "b.pcd"])
    monkeypatch.setattr(cli.os.path, "exists", lambda path: True)
    monkeypatch.setattr(cli, "PointCloudAnalyzer", FakeAnalyzer)
    monkeypatch.setattr(cli, "_wait_exit", lambda: None)

    assert cli.main() == 0
    assert len(created_configs) == 1
    assert analyzed_paths == ["a.pcd", "b.pcd"]


def test_robust_span_uses_percentiles_to_reduce_outlier_impact():
    values = np.array([*range(100), 1000.0])

    assert robust_span(values, 5, 95) < 1000.0
    assert robust_span(np.array([]), 5, 95) == 0.0


def test_pointcloud_loader_converts_millimeter_scale_points():
    cfg = Config()
    loader = PointCloudLoader(cfg)
    points = np.array(
        [
            [100.0, 0.0, 1000.0],
            [200.0, 2000.0, 1100.0],
            [300.0, 4000.0, 1200.0],
        ]
    )

    converted = loader._auto_convert_unit(points)

    assert converted[1, 1] == pytest.approx(2.0)
    assert converted[2, 2] == pytest.approx(1.2)


def test_pointcloud_loader_auto_unit_does_not_assume_y_is_vehicle_axis():
    """历史毫米点云即使长轴落在 X，也必须正确转换为米。"""
    loader = PointCloudLoader(Config(input_unit="auto"))
    points = np.array(
        [
            [0.0, 100.0, 1000.0],
            [2000.0, 200.0, 1100.0],
            [4000.0, 300.0, 1200.0],
        ]
    )

    converted = loader._auto_convert_unit(points)

    assert converted[1, 0] == pytest.approx(2.0)
    assert converted[2, 2] == pytest.approx(1.2)


def test_pointcloud_loader_honors_explicit_meter_unit():
    """显式米制是生产契约，不能再因坐标绝对值较大而二次缩放。"""
    loader = PointCloudLoader(Config(input_unit="m"))
    points = np.array([[1000.0, 0.0, 0.0], [1004.0, 0.0, 0.0]])

    converted = loader._auto_convert_unit(points)

    assert np.array_equal(converted, points)


def test_pointcloud_loader_removes_non_finite_points_before_unit_detection(
    tmp_path, monkeypatch
):
    class FakePointCloud:
        points = np.array(
            [
                [np.nan, 0.0, 0.0],
                [100.0, 0.0, 1000.0],
                [200.0, 2000.0, 1100.0],
                [300.0, 4000.0, 1200.0],
            ]
        )

    monkeypatch.setattr(
        loader_module.o3d.io,
        "read_point_cloud",
        lambda path: FakePointCloud(),
    )
    pcd_path = tmp_path / "scan.pcd"
    pcd_path.write_bytes(
        b"FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nWIDTH 4\nHEIGHT 1\nPOINTS 4\nDATA ascii\n"
        b"nan 0 0\n100 0 1000\n200 2000 1100\n300 4000 1200\n"
    )

    points = PointCloudLoader(Config()).load(str(pcd_path))

    assert len(points) == 3
    assert points[1, 1] == pytest.approx(2.0)


def test_result_builder_keeps_invalid_measurement_in_memory(tmp_path):
    cfg = Config(camera_identifier="CAM")
    dims = VehicleDimensions(10.0, 2.5, 3.0, 6.0, 2.2, 1.2)
    split = CarriageSplitResult(np.zeros((1, 3)), 5.0, 0.0, 7.0, 0.0)
    ribs = [
        RibInfo(1, 4.0, 0.0, 2.0, (-1.0, 1.0), (2.0, 3.0), 10, 2.0, 1.0)
    ]
    payload = ResultExporter.build(
        ribs,
        dims,
        split,
        LoadStatusResult("Uncertain", 0.4, "test", {}),
        "scan.pcd",
        cfg,
        wall_repair={"repaired": True, "synthesizedPoints": 99},
        floor_profile=type(
            "FakeFloorProfile",
            (),
            {"to_dict": lambda self: {"shouldNotLeak": True}},
        )(),
    )

    assert not list(tmp_path.iterdir())
    # Uncertain 不是已确认空车：即使调用方传了候选拉筋，导出边界也必须清空。
    assert payload["dimensions"] is None
    assert payload["carriageBoundaries"] is None
    assert payload["carriageProfile"] is None
    assert payload["integrationPayloads"] == {}
    assert payload["ribs_count"] == 0
    assert payload["ribs"] == []
    assert payload["floorProfile"] is None
    assert payload["wallRepair"] == {"repaired": False}


def test_empty_result_does_not_upload_when_inner_walls_are_unverified():
    cfg = Config(camera_identifier="CAM")
    dims = VehicleDimensions(10.0, 2.5, 3.0, 6.0, 2.2, 1.2)
    split = CarriageSplitResult(
        np.zeros((1, 3)),
        5.0,
        0.0,
        7.0,
        0.0,
        boundaries_are_inner_faces=False,
        boundary_method="wall_core_fallback",
        boundary_warning="内表面不可测",
    )
    payload = ResultExporter.build(
        [],
        dims,
        split,
        LoadStatusResult("Empty", 0.9, "test", {}),
        "scan.pcd",
        cfg,
    )

    assert payload["loadStatus"]["status"] == "Empty"
    assert payload["measurementStatus"]["valid"] is False
    assert payload["dimensions"] is None
    assert payload["integrationPayloads"] == {}


def test_vehicle_payload_segments_run_front_to_rear_and_close_exactly():
    """两根拉筋形成三段；业务方向固定为前内壁到后内壁，三段和等于内长。"""
    cfg = Config(camera_identifier="CAM")
    dimensions = VehicleDimensions(14.0, 2.5, 3.0, 10.0, 2.2, 1.2)
    split = CarriageSplitResult(
        carriage_points=np.zeros((1, 3)),
        split_y=10.0,
        carriage_start_y=0.0,
        head_y=14.0,
        tail_y=0.0,
    )
    ribs = [
        RibInfo(42, 3.333, 0.0, 2.0, (-1.0, 1.0), (2.0, 3.0), 10, 2.0, 1.0),
        RibInfo(99, 8.888, 0.0, 2.0, (-1.0, 1.0), (2.0, 3.0), 10, 2.0, 1.0),
    ]

    payload = ResultExporter._build_vehicle_dimension_payload(
        cfg,
        ribs,
        dimensions,
        split,
        "scan.pcd",
    )

    segments = payload["ribs"]
    assert [segment["ribId"] for segment in segments] == [1, 2, 3]
    assert [segment["distToPrev"] for segment in segments] == [
        111.2,
        555.5,
        333.3,
    ]
    assert round(sum(segment["distToPrev"] for segment in segments), 1) == (
        payload["dimensions"]["trailerLength"]
    )
    assert payload["resultJsonPath"] is None
    assert payload["resultImagePath"] is None


def test_vehicle_payload_contains_weighbridge_floor_profile():
    cfg = Config(camera_identifier="CAM")
    dimensions = VehicleDimensions(14.0, 2.5, 3.0, 10.0, 2.2, 1.2)
    split = CarriageSplitResult(
        np.zeros((1, 3)),
        10.0,
        0.0,
        14.0,
        0.0,
        boundaries_are_inner_faces=True,
    )
    floor = FloorProfile(
        case=1,
        ground_plane=np.array([0.0, 0.0, 1.0, -0.2]),
        ground_plane_source="weighbridge_calibration",
        segments=[FloorSegment(1, "flat", 1.25, 10.0)],
        front_height_to_ground=1.25,
    )

    payload = ResultExporter._build_vehicle_dimension_payload(
        cfg,
        [],
        dimensions,
        split,
        "scan.pcd",
        floor,
    )

    assert payload["floorProfile"]["heightReference"]["type"] == "weighbridge"
    assert (
        payload["floorProfile"]["segments"][0]["heightToReference"]
        == pytest.approx(1.25)
    )
    assert payload["dimensions"]["trailerBottomToGroundHeight"] == 125.0
    assert payload["dimensions"]["highLowPlateHeight"] == 0.0
    assert payload["highLowPlates"] == []


def test_vehicle_payload_maps_step_profile_to_high_low_plate_contract():
    cfg = Config(camera_identifier="CAM")
    dimensions = VehicleDimensions(14.0, 2.5, 3.0, 10.0, 2.2, 1.2)
    split = CarriageSplitResult(
        np.zeros((1, 3)),
        10.0,
        0.0,
        14.0,
        0.0,
        boundaries_are_inner_faces=True,
    )
    floor = FloorProfile(
        case=2,
        ground_plane=np.array([0.0, 0.0, 1.0, -0.2]),
        ground_plane_source="weighbridge_calibration",
        segments=[
            FloorSegment(1, "step_top", 1.421, 3.2),
            FloorSegment(2, "mid_rear", 0.954, 4.0),
            FloorSegment(3, "rear_wall", 0.951, 4.0),
        ],
        front_height_to_ground=1.421,
    )

    payload = ResultExporter._build_vehicle_dimension_payload(
        cfg, [], dimensions, split, "scan.pcd", floor
    )

    assert payload["dimensions"]["trailerBottomToGroundHeight"] == 142.1
    assert payload["dimensions"]["highLowPlateHeight"] == 47.0
    assert payload["highLowPlates"] == [
        {"plateId": 1, "lengthCm": 320.0, "highestToGround": 142.1, "lowestToGround": 142.1},
        {"plateId": 2, "lengthCm": 400.0, "highestToGround": 95.4, "lowestToGround": 95.4},
        {"plateId": 3, "lengthCm": 400.0, "highestToGround": 95.1, "lowestToGround": 95.1},
    ]


def test_vehicle_payload_does_not_publish_untrusted_floor_reference():
    floor = FloorProfile(
        case=2,
        ground_plane=np.array([0.0, 0.0, 1.0, 0.0]),
        ground_plane_source="scene_ransac",
        segments=[
            FloorSegment(1, "step_top", 1.4, 3.0),
            FloorSegment(2, "rear_wall", 0.9, 7.0),
        ],
    )

    dimensions, plates = ResultExporter._build_high_low_plate_payload(floor)

    assert dimensions == {
        "trailerBottomToGroundHeight": None,
        "highLowPlateHeight": None,
    }
    assert plates == []


def test_cli_module_is_runnable_with_dash_m():
    """`python -m carpointcloud_analyzer.api.cli` 必须真的执行 main()。

    根目录兼容 shim 删除后，这是唯一的单文件排障入口。缺少 __main__ 守卫时
    该命令会静默退出 0、什么都不做，从命令行看不出任何异常。
    """
    import subprocess
    import sys as _sys

    completed = subprocess.run(
        [_sys.executable, "-m", "carpointcloud_analyzer.api.cli"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        cwd=str(Path(__file__).resolve().parents[2]),
        env={**os.environ, "CARPOINT_PCD_PATH": "", "PYTHONIOENCODING": "utf-8"},
    )

    assert completed.returncode == 1
    assert "未提供 PCD 文件路径" in (completed.stdout + completed.stderr)
