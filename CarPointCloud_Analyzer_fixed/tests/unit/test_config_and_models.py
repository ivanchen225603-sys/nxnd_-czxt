
from carpointcloud_analyzer import (
    LoadStatusResult,
    RibInfo,
    VehicleDimensions,
)


def test_rib_info_to_dict_keeps_public_payload_fields():
    rib = RibInfo(
        rib_id=1,
        y_position=1.23456,
        x_center=0.12345,
        z_center=2.34567,
        x_range=(-1.234, 1.987),
        z_range=(2.111, 3.222),
        point_count=42,
        length=2.5001,
        height=1.2001,
    )

    assert rib.to_dict() == {
        "rib_id": 1,
        "y_position": 1.23456,
        "x_center": 0.12345,
        "z_center": 2.34567,
        "x_range": [-1.234, 1.987],
        "z_range": [2.111, 3.222],
        "point_count": 42,
        "length": 2.5001,
        "height": 1.2001,
    }


def test_vehicle_dimensions_to_dict_keeps_vehicle_and_carriage_groups():
    dimensions = VehicleDimensions(
        vehicle_length=12.0,
        vehicle_width=2.5,
        vehicle_height=3.1,
        carriage_length=8.2,
        carriage_width=2.4,
        carriage_height=2.2,
    )

    assert dimensions.to_dict() == {
        "vehicle": {"length": 12.0, "width": 2.5, "height": 3.1},
        "carriage": {"length": 8.2, "width": 2.4, "height": 2.2},
    }


def test_status_payload_confidence_is_rounded():
    status = LoadStatusResult("Loaded", 0.92345, "baseline", {"ratio": 0.12})

    assert status.to_dict()["confidence"] == 0.923


# ------------------------------------------------------------
# 多智能体评审提出问题的回归测试
# ------------------------------------------------------------
