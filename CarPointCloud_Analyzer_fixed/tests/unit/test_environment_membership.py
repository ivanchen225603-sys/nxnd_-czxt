"""原失败环境点云：完整尺寸、顶沿及五根拉筋必须同时保留。"""
from pathlib import Path

import numpy as np
import pytest

from carpointcloud_analyzer import Config, PointCloudAnalyzer


@pytest.mark.parametrize("case", ["scattered", "mixed"])
def test_environment_does_not_extend_vehicle_or_erase_rim_and_ribs(case):
    with np.load(Path(__file__).parents[1] / "fixtures/environment_membership.npz") as saved:
        points = saved[case]
    analyzer = PointCloudAnalyzer(Config(
        save_clean_pointcloud=False, weighbridge_plane_source=(0., 0., 1., 0.),
        weighbridge_plane_orientation_verified=True))
    result = analyzer._analyze_points(points, "")
    # 沿用原环境审计的 5cm 尺寸与 3cm 拉筋位置容差。
    assert result["dimensions"]["vehicle"]["length"] == pytest.approx(14.5, abs=.05)
    assert result["dimensions"]["carriage"] == pytest.approx(
        dict(length=10., width=2.4, height=2.), abs=.05)
    assert result["loadStatus"]["status"] == "Empty"
    payload = result["integrationPayloads"]["vehicleDimensions"]
    assert payload["dimensions"]["cabToTrailerFrontDist"] == pytest.approx(450., abs=5.)
    assert payload["dimensions"]["trailerBottomToGroundHeight"] == pytest.approx(160., abs=5.)
    assert payload["actualRibCount"] == 5
    segments = [rib["distToPrev"] for rib in payload["ribs"]]
    assert segments == pytest.approx([140., 160., 170., 220., 160., 150.], abs=3.)
    assert round(sum(segments), 1) == payload["dimensions"]["trailerLength"]
