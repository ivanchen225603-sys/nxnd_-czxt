"""整车高按车顶离地面平面高度量取，不受车厢俯仰或车底遮挡影响。"""

import numpy as np
import pytest
from pathlib import Path

from carpointcloud_analyzer.domain.dimensions import vehicle_height_above_ground
from carpointcloud_analyzer.domain.utils.histogram import robust_span


def _pitched_truck(pitch_deg):
    """16m 车：顶面离地 4.0m，可见最低点是离地 1.0m 的车底；局部系随底板俯仰。"""
    rng = np.random.default_rng(7)
    length = rng.uniform(-8.0, 8.0, 20000)
    width = rng.uniform(-1.25, 1.25, 20000)
    top = np.c_[width, length, np.full(20000, 4.0)]
    bottom = np.c_[width, length, np.full(20000, 1.0)]
    world = np.vstack((top, bottom))
    angle = np.deg2rad(pitch_deg)
    # 局部 L/H 轴绕 W 轴俯仰；地面 z=0 在局部系中随之倾斜。
    rotation = np.array([[1, 0, 0],
                         [0, np.cos(angle), -np.sin(angle)],
                         [0, np.sin(angle), np.cos(angle)]])
    local = world @ rotation
    ground = np.r_[rotation.T @ np.array([0., 0., 1.]), 0.]
    return local, ground


def test_height_is_top_above_ground_not_visible_span():
    local, ground = _pitched_truck(1.0)

    height, metadata = vehicle_height_above_ground(local, ground, 99.0)

    assert abs(height - 4.0) < 1e-6
    assert metadata['definition'] == 'top_above_ground_plane'
    # 旧口径：只看到车底到车顶 3m，且 1° 俯仰在 16m 上再拉长约 0.28m。
    assert robust_span(local[:, 2], 1.0, 99.0) < 3.4


def test_downward_plane_normal_is_flipped():
    local, ground = _pitched_truck(0.0)

    height, _ = vehicle_height_above_ground(local, -ground, 99.0)

    assert abs(height - 4.0) < 1e-6


def test_missing_ground_plane_reports_fallback_definition():
    local, _ = _pitched_truck(0.0)

    height, metadata = vehicle_height_above_ground(local, None, 99.0)

    assert height is None
    assert metadata['definition'] == 'visible_local_h_span_not_ground_height'


def test_floor_sampling_density_does_not_lower_the_observed_roof():
    width, length = np.meshgrid(np.linspace(-1.2, 1.2, 20), np.linspace(0., 10., 10))
    roof = np.c_[width.ravel(), length.ravel(), np.full(width.size, 4.)]
    bottom = roof.copy()
    bottom[:, 2] = 1.
    points = np.vstack((roof, np.repeat(bottom, 200, axis=0), [[0., 0., 8.], [1., 5., 9.]]))
    height, _ = vehicle_height_above_ground(points, np.array([0., 0., 1., 0.]), 99.)
    assert height == pytest.approx(4., abs=1e-6)


def test_original_complete_scene_meets_existing_vehicle_height_tolerance():
    from carpointcloud_analyzer import Config, PointCloudAnalyzer
    with np.load(Path(__file__).parents[1] / 'fixtures/environment_membership.npz') as saved:
        points = saved['base']
    result = PointCloudAnalyzer(Config(save_clean_pointcloud=False,
        weighbridge_plane_source=(0., 0., 1., 0.), weighbridge_plane_orientation_verified=True))._analyze_points(points, '')
    assert result['dimensions']['vehicle']['height'] == pytest.approx(3.6, abs=.05)
