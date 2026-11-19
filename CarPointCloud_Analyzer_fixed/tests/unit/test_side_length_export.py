"""完整侧壁测长的米制输出、厘米载荷及底板口径必须共用业务端点。"""

from copy import deepcopy
from dataclasses import asdict, replace
from decimal import Decimal
import json
from types import SimpleNamespace

import numpy as np
import pytest

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.carriage import CarriageAnalyzer, spatially_balanced_cloud
from carpointcloud_analyzer.domain.dimensions import DimensionCalculator
from carpointcloud_analyzer.application.exporters import ResultExporter
from carpointcloud_analyzer.domain.geometry import CoordinateTransform
from carpointcloud_analyzer.domain.models import (
    CarriageSplitResult,
    LoadStatusResult,
    LongitudinalMeasurement,
    RibInfo,
    VehicleDimensions,
)
from carpointcloud_analyzer.infrastructure.persistence import _normalize_payload
from carpointcloud_analyzer.application.pipeline import PointCloudAnalyzer


def _measurement(*, inferred=False):
    return LongitudinalMeasurement(
        rear_l=0.1,
        front_l=10.8,
        side_plane=(0.0, 0.0, 1.0),
        metadata={
            "definition": "same_side_lower_band_longitudinal_projection",
            "valid": True,
            "heightAboveFloorM": [0.15, 0.35],
            "inwardBandM": [0.06, 0.10],
            "completionUsed": inferred,
        },
    )


def _floor_points():
    w, length = np.meshgrid(np.linspace(-1.2, 1.2, 31), np.linspace(0.0, 11.0, 221))
    return np.c_[w.ravel(), length.ravel(), np.full(w.size, 1.4)]


def _split(measurement):
    return CarriageSplitResult(
        carriage_points=_floor_points(),
        split_y=11.0,
        carriage_start_y=0.0,
        head_y=15.0,
        tail_y=-0.1,
        boundaries_are_inner_faces=True,
        boundary_method="observed_wall_surface_fit",
        front_wall_plane=(0.0, 0.0, 11.0),
        cab_measurement_head_y=15.0,
        cab_measurement_front_y=10.98,
        cab_measurement_height=1.4,
        cab_measurement_half_band=0.02,
        cab_measurement_point_count=100,
        cab_measurement_method="ground_height_slice_percentile",
        longitudinal_measurement=measurement,
    )


def _dimensions(length=10.7):
    return VehicleDimensions(15.0, 2.5, 3.0, length, 2.3, 1.6)


def _rib(identifier, length, slope):
    direction = np.array([1.0, slope, 0.0])
    direction /= np.linalg.norm(direction)
    return RibInfo(
        identifier, length, 0.0, 2.7, (-1.1, 1.1), (2.68, 2.72), 500, 2.4, 0.04,
        model_type="ransac_pca_line",
        axis_midpoint=(0.0, length, 2.7),
        axis_direction=tuple(direction),
        axis_endpoints=((-1.2, length - 1.2 * slope, 2.7),
                        (1.2, length + 1.2 * slope, 2.7)),
        axis_observed_w_range=(-1.1, 1.1),
        model_inlier_count=400,
        model_rmse=0.01,
    )


def _export(split, ribs=(), dimensions=None, status="Empty"):
    return ResultExporter.build(
        list(ribs), dimensions or _dimensions(), split,
        LoadStatusResult(status, 0.95, "test", {}), "side_length.pcd",
        Config(camera_identifier="CAM_004", license_plate="TEST_SIDE_LENGTH"),
    )


def test_dimensions_and_payload_use_side_endpoints_without_changing_structure(monkeypatch):
    split = _split(_measurement())
    length, height = np.meshgrid(np.linspace(0.0, 11.0, 221), np.linspace(1.4, 3.0, 41))
    walls = [np.c_[np.full(length.size, side), length.ravel(), height.ravel()]
             for side in (-1.2, 1.2)]
    split.carriage_points = np.vstack([split.carriage_points, *walls])
    original_points = split.carriage_points.copy()
    # 使用真实剖面，覆盖高度测量所需的侧壁范围与掩码，避免替身漏掉新增字段。
    profile = CarriageAnalyzer(Config()).analyze(original_points)
    # 本测试隔离净宽拟合，只验证端点、正式高度和载荷的导出契约。
    monkeypatch.setattr("carpointcloud_analyzer.domain.dimensions.measure_inner_width",
                        lambda *args, **kwargs: SimpleNamespace(
                            width_m=2.3, metadata={"valid": True}))
    full_points = np.vstack([original_points, original_points + [0.0, 0.0, 1.6],
                             [[-1.2, 15.0, 1.4], [1.2, 15.0, 3.0]]])
    dimensions = DimensionCalculator(Config()).calculate(
        full_points, split.carriage_points, split, profile)
    result = _export(split, dimensions=dimensions)
    payload = result["integrationPayloads"]["vehicleDimensions"]

    assert dimensions.carriage_length == pytest.approx(10.7)
    assert result["dimensions"]["carriage"]["length"] == pytest.approx(10.7)
    assert payload["dimensions"]["trailerLength"] == 1070.0
    assert payload["ribs"] == [{"ribId": 1, "distToPrev": 1070.0}]
    assert payload["dimensions"]["cabToTrailerFrontDist"] == 402.0
    assert 1.5 < dimensions.carriage_height <= 1.6
    assert payload["dimensions"]["trailerBodyHeight"] == round(dimensions.carriage_height * 100, 1)
    assert result["measurementStatus"]["heightMeasurement"] == dimensions.height_measurement
    assert (split.split_y, split.carriage_start_y, split.front_wall_plane) == (
        11.0, 0.0, (0.0, 0.0, 11.0))
    np.testing.assert_array_equal(split.carriage_points, original_points)


def test_slanted_ribs_use_same_side_projection_and_close_after_cm_rounding():
    split = _split(_measurement())
    ribs = [_rib(42, 3.7, 0.1), _rib(99, 8.1, 0.2)]
    original_ribs = deepcopy([asdict(rib) for rib in ribs])
    result = _export(split, ribs)
    payload = result["integrationPayloads"]["vehicleDimensions"]

    # W=1 处轴线交点为 L=3.8 和 8.3；不能继续使用轴线中点 3.7 和 8.1。
    assert payload["ribs"] == [
        {"ribId": 1, "distToPrev": 250.0},
        {"ribId": 2, "distToPrev": 450.0},
        {"ribId": 3, "distToPrev": 370.0},
    ]
    normalized = _normalize_payload(payload)
    assert sum(row["value"] for row in normalized["ribs"]) == normalized["dimensions"]["trailerLength"]
    assert normalized["dimensions"]["trailerLength"] == Decimal("1070.0")
    assert [asdict(rib) for rib in ribs] == original_ribs
    assert [rib["y_position"] for rib in result["ribs"]] == [3.7, 8.1]
    projection = result["measurementStatus"]["lengthMeasurement"]["ribProjections"]
    assert [row["positionM"] for row in projection] == pytest.approx([3.8, 8.3])
    assert all(row["extrapolationM"] == 0.0 for row in projection)


def test_fractional_positions_quantize_cumulative_boundaries_not_each_gap():
    split = _split(_measurement())
    payload = _export(split, [_rib(1, 8.33336, 0.0), _rib(2, 4.66668, 0.0)])[
        "integrationPayloads"]["vehicleDimensions"]
    assert [row["distToPrev"] for row in payload["ribs"]] == [246.7, 366.6, 456.7]
    assert sum(Decimal(str(row["distToPrev"])) for row in payload["ribs"]) == Decimal("1070.0")


def test_missing_side_endpoint_does_not_fall_back_to_original_wall_spacing():
    measurement = replace(_measurement(), rear_l=None,
                          metadata={**_measurement().metadata, "valid": False})
    result = _export(_split(measurement), dimensions=_dimensions(11.0))
    assert result["dimensions"]["carriage"]["length"] is None
    assert result["integrationPayloads"]["vehicleDimensions"]["dimensions"]["trailerLength"] is None
    assert result["measurementStatus"]["valid"] is False
    assert result["measurementStatus"]["reason"] == "longitudinal_inner_edges_unavailable"
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("missing", ["axis", "observed_extent", "outside_extent"])
def test_missing_same_side_rib_position_keeps_length_and_unknown_gaps(missing):
    rib = _rib(1, 5.0, 0.1)
    if missing == "axis":
        rib.axis_midpoint = None
    elif missing == "observed_extent":
        rib.axis_observed_w_range = None
    else:
        rib.axis_observed_w_range = (-0.2, 0.2)
    result = _export(_split(_measurement()), [rib])
    assert result["dimensions"]["carriage"]["length"] == pytest.approx(10.7)
    assert result["measurementStatus"]["valid"] is True
    assert "reviewStatus" not in result["measurementStatus"]
    metadata = result["measurementStatus"]["lengthMeasurement"]
    assert metadata["ribPositionsValid"] is False
    assert metadata["ribProjections"][0]["valid"] is False
    assert metadata["ribProjections"][0]["reason"]
    payload = result["integrationPayloads"]["vehicleDimensions"]
    assert payload["detectStatus"] == "success"
    assert payload["dimensions"]["trailerLength"] == 1070.0
    assert payload["ribs"] == [{"ribId": 1, "distToPrev": None},
                               {"ribId": 2, "distToPrev": None}]
    assert all(row["value"] is None for row in _normalize_payload(payload)["ribs"])
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("missing_index,expected_gaps", [
    (0, [None, None, 370.0]),
    (1, [250.0, None, None]),
])
def test_one_missing_rib_only_nulls_its_adjacent_gaps(missing_index, expected_gaps):
    ribs = [_rib(1, 8.1, 0.2), _rib(2, 3.7, 0.1)]
    ribs[missing_index].axis_observed_w_range = None
    originals = deepcopy([asdict(rib) for rib in ribs])
    result = _export(_split(_measurement()), ribs)
    payload = result["integrationPayloads"]["vehicleDimensions"]

    assert payload["dimensions"]["trailerLength"] == 1070.0
    assert [row["ribId"] for row in payload["ribs"]] == [1, 2, 3]
    assert [row["distToPrev"] for row in payload["ribs"]] == expected_gaps
    assert result["ribs_count"] == 2
    assert "reviewStatus" not in result["measurementStatus"]
    normalized = _normalize_payload(payload)
    assert sum((row["value"] or Decimal("0")) for row in normalized["ribs"]) < Decimal("1070.0")
    assert [asdict(rib) for rib in ribs] == originals
    # 未知区间必须保持 null，不平均分摊剩余长度，也不借轴线中点填入数值。
    assert sum(row["value"] is None for row in normalized["ribs"]) == 2


def test_inferred_side_length_exports_success_payload_without_review_status():
    result = _export(_split(_measurement(inferred=True)), [_rib(1, 5.0, 0.0)])
    status = result["measurementStatus"]
    assert status["valid"] is True
    assert "reviewStatus" not in status
    assert status["calculationSucceeded"] is True
    assert "inferredGeometry" not in status["lengthMeasurement"]
    payload = result["integrationPayloads"]["vehicleDimensions"]
    assert payload["detectStatus"] == "success"
    assert _normalize_payload(payload)["detectStatus"] == "success"
    assert payload["dimensions"]["trailerLength"] == 1070.0


def test_completed_lower_edge_exports_estimate_and_preserves_missing_observation():
    measurement = _measurement(inferred=True)
    measurement.metadata.update(
        method="completed_lower_side_inner_edges",
        source="current_measured_points_and_symmetric_wall_completion",
        assumption="left_right_symmetric_end_walls",
        observedMeasurement={"valid": False, "reason": "lower_inner_edges_unavailable",
                             "lowerEndpoints": {"rear": {"valid": False, "pointCount": 11}}},
        lowerEndpoints={"rear": {"valid": True, "positionM": .1,
                                  "source": "measured_and_symmetric_wall_completion",
                                  "supportPointCount": 26, "observedPointCount": 11}},
        completionFaceSelection={"rear": {
            "faceSelection": "dominant_near_coincident_relaxed",
            "relaxation": {"originalReason": "ambiguous_end_faces"},
        }},
    )
    original_metadata = deepcopy(measurement.metadata)
    result = _export(_split(measurement))
    status = result["measurementStatus"]
    assert status["valid"] is True
    assert "reviewStatus" not in status
    assert status["calculationSucceeded"] is True
    assert "inferredGeometry" not in status["lengthMeasurement"]
    diagnosis = status["lengthMeasurement"]
    for field in ("method", "assumption", "lowerEndpoints",
                  "completionFaceSelection"):
        assert diagnosis[field] == ResultExporter._without_point_origin_labels(original_metadata[field])
    assert 'source' not in diagnosis
    assert 'observedMeasurement' not in diagnosis
    payload = result["integrationPayloads"]["vehicleDimensions"]
    assert payload["detectStatus"] == "success"
    assert result["dimensions"]["carriage"]["length"] == pytest.approx(10.7)
    assert _normalize_payload(payload)["dimensions"]["trailerLength"] == Decimal("1070.0")
    assert measurement.metadata == original_metadata
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("observed_high", [0.85, 0.75])
def test_bounded_rib_extrapolation_is_traced_not_claimed_fully_observed(observed_high):
    rib = _rib(1, 5.0, 0.1)
    rib.axis_observed_w_range = (-1.1, observed_high)
    result = _export(_split(_measurement()), [rib])
    assert "reviewStatus" not in result["measurementStatus"]
    assert result["integrationPayloads"]["vehicleDimensions"]["detectStatus"] == "success"
    metadata = result["measurementStatus"]["lengthMeasurement"]
    assert metadata["maximumRibExtrapolationM"] == 0.30
    assert metadata["ribProjections"][0]["extrapolationM"] == pytest.approx(1.0 - observed_high)
    assert metadata["ribProjections"][0]["observedWRangeM"] == [-1.1, observed_high]


@pytest.mark.parametrize("status", ["Loaded", "Uncertain"])
def test_inferred_measurement_never_relaxes_load_guard(status):
    result = _export(_split(_measurement(inferred=True)), [_rib(1, 5.0, 0.0)], status=status)
    if status == 'Loaded':
        assert result["dimensions"] is None
        assert result["integrationPayloads"] == {}
    else:
        assert result['integrationPayloads']['vehicleDimensions']['detectStatus'] == 'success'
        assert result['dimensions']['carriage']['length'] == pytest.approx(10.7)
        assert result['integrationPayloads']['vehicleDimensions']['actualRibCount'] is None
    assert result["measurementStatus"]["valid"] is False
    assert result["measurementStatus"].get("calculationSucceeded") is (status == 'Uncertain')


def test_inconsistent_formal_length_is_rejected_not_silently_overwritten():
    with pytest.raises(ValueError, match="内长.*端点不一致"):
        _export(_split(_measurement()), dimensions=_dimensions(11.0))


def test_legacy_model_without_measurement_keeps_original_contract():
    result = _export(_split(None), dimensions=_dimensions(11.0))
    assert result["dimensions"]["carriage"]["length"] == 11.0
    assert result["integrationPayloads"]["vehicleDimensions"]["dimensions"]["trailerLength"] == 1100.0


def test_business_floor_is_recomputed_with_new_endpoints_without_changing_load_points(monkeypatch):
    analyzer = PointCloudAnalyzer(Config())
    split = _split(_measurement())
    original_points = split.carriage_points.copy()
    profile = SimpleNamespace(interior_w_range=(-1.0, 1.0), measurement_confidence="high")
    classified = []

    def classify(points, *args):
        classified.append(points.copy())
        return LoadStatusResult("Empty", 0.95, "test", {})

    monkeypatch.setattr(analyzer.load_classifier, "classify", classify)
    plane = np.array([0.0, 0.0, 1.0, 0.0])
    load = analyzer._assess_load(
        split, profile,
    )

    floor = analyzer._measure_current_floor(
        split, profile, load, SimpleNamespace(ground_plane=plane),
        CoordinateTransform(np.zeros(3), np.eye(3)), plane, plane, True,
    )
    assert floor is not None
    assert sum(segment.length_to_prev for segment in floor.segments) == pytest.approx(10.7)
    assert floor.segments[0].height_to_ground == pytest.approx(1.4)
    np.testing.assert_array_equal(
        classified[0], spatially_balanced_cloud(original_points[
            (original_points[:, 1] > split.carriage_start_y + .12)
            & (original_points[:, 1] < split.split_y - .12)], analyzer.config))
    np.testing.assert_array_equal(split.carriage_points, original_points)
    assert (split.split_y, split.carriage_start_y) == (11.0, 0.0)
