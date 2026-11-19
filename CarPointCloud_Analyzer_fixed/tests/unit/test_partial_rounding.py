"""部分尺寸经过厘米量化后仍满足真实持久化的分段闭合约束。"""

from decimal import Decimal

import numpy as np
import pytest

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.application.exporters import ResultExporter
from carpointcloud_analyzer.domain.models import (
    CarriageSplitResult, LoadStatusResult, LongitudinalMeasurement, RibInfo, VehicleDimensions,
)
from carpointcloud_analyzer.infrastructure.persistence import _normalize_payload


@pytest.mark.parametrize("missing,expected", [
    (None, [333.4, 333.3, 333.4]),
    (0, [None, None, 333.4]),
    (1, [333.4, None, None]),
])
def test_partial_export_uses_verified_endpoints_without_filling_unknown_gaps(missing, expected):
    positions = [6.6672, 3.3336]
    ribs = [RibInfo(i, p, 0., 1.6, (-1.1, 1.1), (1.58, 1.62), 50, 2.2, .04)
            for i, p in enumerate(positions, 1)]
    measurement = LongitudinalMeasurement(
        rear_l=0., front_l=10.0008, side_plane=(0., 0., 1.),
        metadata={"definition": "same_side_inner_end_faces", "selectedSide": "right"},
        rib_side_projections=[{
            "ribId": i, "valid": i - 1 != missing, "positionM": p if i - 1 != missing else None,
            "extrapolationM": 0., "selectedSide": "right", "referencePlaneWLH": [0., 0., 1.],
            "intersectionWLH": [1., p, 1.6],
        } for i, p in enumerate(positions, 1)],
    )
    split = CarriageSplitResult(
        carriage_points=np.zeros((1, 3)), split_y=10.0008, carriage_start_y=0.,
        head_y=15., tail_y=0., boundaries_are_inner_faces=True,
        cab_measurement_method="fixed_height", cab_measurement_reason="head_slice_unavailable",
        longitudinal_measurement=measurement,
    )
    dimensions = VehicleDimensions(
        15., 2.5, 3., 10.0008, 2.3, 1.6, carriage_outer_width=2.5,
        width_measurement={"valid": True, "widthM": 2.3, "adoptedSource": "completed_cloud_inner_side_faces"},
    )
    result = ResultExporter.build(ribs, dimensions, split, LoadStatusResult("Empty", 1., "test", {}),
                                  "partial.pcd", Config(camera_identifier="CAM_004"))
    payload = result["integrationPayloads"]["vehicleDimensions"]
    assert payload["dimensions"]["cabToTrailerFrontDist"] is None
    assert payload["dimensions"]["trailerLength"] == 1000.1
    assert [row["distToPrev"] for row in payload["ribs"]] == expected
    _normalize_payload(payload)
    if missing is None:
        assert sum(Decimal(str(value)) for value in expected) == Decimal("1000.1")
