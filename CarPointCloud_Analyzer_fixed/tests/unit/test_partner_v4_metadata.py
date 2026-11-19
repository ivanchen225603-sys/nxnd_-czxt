"""对外合同附加标记不可改变内长闭合或把原高度当作板长。"""
from decimal import Decimal

import pytest

from carpointcloud_analyzer.infrastructure.persistence import _normalize_payload, _payload_hash
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.application.exporters import ResultExporter


def _payload():
    return {
        "idempotencyKey": "partner_v4", "cameraIdentifier": "CAM_004",
        "licensePlate": "宁A25771", "detectStatus": "success",
        "dimensions": {"trailerLength": 1000},
        "ribs": [{"ribId": 1, "distToPrev": 400}, {"ribId": 2, "distToPrev": 600}],
        "wires": [], "highLowPlates": [],
    }


def test_actual_count_preserves_internal_tail_and_participates_in_hash():
    payload = _payload()
    old = _normalize_payload(payload)
    payload["actualRibCount"] = 1
    new = _normalize_payload(payload)
    assert new["actualRibCount"] == 1
    assert new["ribs"] == old["ribs"]
    assert _payload_hash(new) != _payload_hash(old)


def test_length_is_independent_of_legacy_highest_height():
    payload = _payload()
    payload["highLowPlates"] = [{"plateId": 1, "highestToGround": 150,
                                 "lowestToGround": 100, "lengthCm": 350}]
    plate = _normalize_payload(payload)["highLowPlates"][0]
    assert plate["highestToGround"] == Decimal("150.0")
    assert plate["lengthCm"] == Decimal("350.0")
    del payload["highLowPlates"][0]["lengthCm"]
    assert "lengthCm" not in _normalize_payload(payload)["highLowPlates"][0]


def test_outer_width_is_separate_from_inner_width_and_changes_hash():
    payload = _payload()
    payload["dimensions"]["trailerWidth"] = 240
    old = _normalize_payload(payload)
    assert "maximumWidthCm" not in old
    payload["maximumWidthCm"] = 255.55
    new = _normalize_payload(payload)
    assert new["maximumWidthCm"] == Decimal("255.6")
    assert new["dimensions"]["trailerWidth"] == Decimal("240.0")
    assert _payload_hash(new) != _payload_hash(old)


def test_partial_export_keeps_observed_outer_width_separate_from_net_width():
    partial = {"dimensions": {"carriage": {"width": 2.4}},
               "referenceDimensions": {"carriage": {"width": 2.55}},
               "ribIntervals": [4, 6]}
    payload = ResultExporter._build_partial_vehicle_dimension_payload(Config(), partial, "scan.pcd")
    assert payload["dimensions"]["trailerWidth"] == 240
    assert payload["maximumWidthCm"] == 255
    assert payload["actualRibCount"] == 1
    assert len(payload["ribs"]) == 2


@pytest.mark.parametrize("count", [None, 0, 2])
def test_partial_missing_length_keeps_detected_count_without_inventing_distances(count):
    partial = {"dimensions": {"carriage": {"width": 2.4}},
               "ribCount": count, "ribIntervals": None}
    payload = ResultExporter._build_partial_vehicle_dimension_payload(
        Config(camera_identifier="CAM_004"), partial, "missing-length.pcd")
    assert payload["actualRibCount"] == count
    assert payload["dimensions"]["trailerLength"] is None
    assert payload["ribs"] == ([{"ribId": i, "distToPrev": None}
                                for i in range(1, count + 2)] if count else [])
    normalized = _normalize_payload(payload)
    assert normalized.get("actualRibCount") == count
