from decimal import Decimal

import numpy as np
import pytest

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.application.exporters import ResultExporter
from carpointcloud_analyzer.domain.models import CarriageSplitResult, VehicleDimensions
from carpointcloud_analyzer.infrastructure.persistence import _normalize_payload


def payload(length, segments, field="ribs"):
    id_name = "ribId" if field == "ribs" else "wireId"
    return {"idempotencyKey": "test-longitudinal", "cameraIdentifier": "CAM_004",
            "dimensions": {"trailerLength": length},
            field: [{id_name: i, "distToPrev": value} for i, value in enumerate(segments, 1)]}


@pytest.mark.parametrize("field", ["ribs", "wires"])
@pytest.mark.parametrize("length,segments", [
    (1134.4, [1134.4]), (3000, [3000]), (1134.4, [500, 634.4]),
    (960.04, [960.04]), (100, [40, None]), (None, [2000]), (0, [0]),
])
def test_segments_follow_carriage_length_and_preserve_partial_unknowns(field, length, segments):
    result = _normalize_payload(payload(length, segments, field))
    assert len(result[field]) == len(segments)
    for expected, actual in zip(segments, result[field]):
        assert (actual["value"] is None) == (expected is None)


def test_no_detected_ribs_exports_one_full_length_segment_and_passes_storage_validation():
    length = 11.344
    split = CarriageSplitResult(carriage_points=np.zeros((1, 3)), split_y=length,
                                carriage_start_y=0, head_y=15, tail_y=0)
    dimensions = VehicleDimensions(15, 2.6, 3, length, 2.5, 2.7)
    data = ResultExporter._build_vehicle_dimension_payload(
        Config(camera_identifier="CAM_004"), [], dimensions, split, "test.pcd")
    assert data["ribs"] == [{"ribId": 1, "distToPrev": 1134.4}]
    normalized = _normalize_payload(data)
    assert normalized["ribs"][0]["value"] == normalized["dimensions"]["trailerLength"] == Decimal("1134.4")
