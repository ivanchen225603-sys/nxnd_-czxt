"""评审缺陷回归：标定来源、实测点找回、水平整车长度。"""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from carpointcloud_analyzer import Config, PointCloudAnalyzer
from carpointcloud_analyzer.domain.dimensions import DimensionCalculator
from carpointcloud_analyzer.domain.models import CarriageSplitResult


@pytest.mark.parametrize('calibrated', [False, True])
def test_cab_height_slice_requires_weighbridge_calibration(calibrated):
    with np.load(Path(__file__).parents[1] / 'fixtures/environment_membership.npz') as saved:
        points = saved['base']
    config = Config(save_clean_pointcloud=False,
                    weighbridge_plane_source=(0., 0., 1., 0.) if calibrated else None,
                    weighbridge_plane_orientation_verified=calibrated)
    result = PointCloudAnalyzer(config)._analyze_points(points, '')
    payload = result['integrationPayloads']['vehicleDimensions']
    measured = payload['dimensions']
    assert measured['trailerLength'] == pytest.approx(1000., abs=5.)
    assert measured['trailerWidth'] == pytest.approx(240., abs=5.)
    if calibrated:
        assert measured['cabToTrailerFrontDist'] == pytest.approx(450., abs=5.)
    else:
        assert measured['cabToTrailerFrontDist'] is None
        assert result['measurementStatus']['cabMeasurement']['reason'] == 'weighbridge_plane_unavailable'


def test_uncalibrated_http_result_writes_null_cab_to_sql(tmp_path, monkeypatch):
    import open3d as o3d
    from fastapi.testclient import TestClient
    from test_persistence import FakeDatabase, _store
    from carpointcloud_analyzer.composition import create_app
    from carpointcloud_analyzer.infrastructure.profile_storage import DisabledEmptyVehicleProfileStorage

    monkeypatch.delenv('CARPOINT_EXTRINSIC_PATH', raising=False)
    with np.load(Path(__file__).parents[1] / 'fixtures/environment_membership.npz') as saved:
        points = saved['base']
    pcd = tmp_path / 'unmarked-ground.pcd'
    o3d.io.write_point_cloud(str(pcd), o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)))
    database = FakeDatabase()
    database.arrivals['uncalibrated-scan'] = ('TEST001', 'SUCCEEDED')
    app = create_app(dimension_store=_store(database),
                     empty_profile_storage=DisabledEmptyVehicleProfileStorage())
    with TestClient(app) as client:
        response = client.post('/analyze', json={'pcdPath': str(pcd), 'camera': 'CAM_004',
                               'workflowId': 'uncalibrated-scan', 'inputUnit': 'm',
                               'capturedAt': '2026-11-05T12:00:00+08:00'})
    assert response.status_code == 200, response.text
    result = response.json()
    assert result['integrationPayloads']['vehicleDimensions']['dimensions']['cabToTrailerFrontDist'] is None
    assert result['outputs']['measurementStatus']['cabMeasurement']['reason'] == 'weighbridge_plane_unavailable'
    records = [row for kind, row in database.details if kind == 'record']
    assert len(records) == 1
    assert records[0][8] is None
    assert records[0][9] == pytest.approx(1000., abs=5.)
    assert records[0][10] == pytest.approx(240., abs=5.)
    assert records[0][11] == pytest.approx(198., abs=5.)


def test_recovered_measured_points_do_not_enable_sparse_rectangular_inference():
    length, height = np.meshgrid(np.arange(0., 10.001, .05), np.arange(1.5, 3.501, .05))
    walls = np.vstack([np.c_[np.full(length.size, side), length.ravel(), height.ravel()]
                       for side in (-1.2, 1.2)])
    observed = walls[(walls[:, 0] < 0) | ((walls[:, 1] > 2.) & (walls[:, 1] < 4.2))]
    # 找回的是内部原始杆件回波，既不是镜像墙，也不能证明缺失侧壁为矩形。
    recovered = np.c_[np.linspace(-.8, .8, 41), np.full(41, 5.), np.full(41, 3.)]
    final = np.vstack((observed, recovered))
    split = CarriageSplitResult(observed, 10., 0., 10., 0., boundaries_are_inner_faces=True)
    profile = SimpleNamespace(floor_z=1.5, wall_top=3.5, interior_w_range=(-1., 1.))
    result = DimensionCalculator(Config()).calculate(final, final, split, profile)
    assert np.isnan(result.carriage_width)
    assert result.width_measurement['completionUsed'] is False
    assert result.width_measurement['syntheticPointCount'] == 0


@pytest.mark.parametrize('pitch', [-5., 5.])
def test_vehicle_length_is_horizontal_extent_with_calibrated_plane(pitch):
    width, length = np.meshgrid(np.linspace(-1.2, 1.2, 31), np.linspace(0., 10., 251))
    world = np.vstack([np.c_[width.ravel(), length.ravel(), np.full(width.size, h)]
                       for h in (1., 4.)])
    angle = np.deg2rad(pitch)
    rotation = np.array([[1., 0., 0.], [0., np.cos(angle), -np.sin(angle)],
                         [0., np.sin(angle), np.cos(angle)]])
    local = world @ rotation
    plane = np.r_[rotation.T @ np.array([0., 0., 1.]), 0.]
    split = CarriageSplitResult(local, float(local[:, 1].max()), float(local[:, 1].min()),
                                float(local[:, 1].max()), float(local[:, 1].min()))
    profile = SimpleNamespace(floor_z=1., wall_top=4., interior_w_range=(-1., 1.))
    measured = DimensionCalculator(Config()).calculate(local, local, split, profile, plane)
    assert measured.vehicle_length == pytest.approx(10., abs=1e-6)
    uncalibrated = DimensionCalculator(Config()).calculate(local, local, split, profile)
    assert uncalibrated.vehicle_length == pytest.approx(float(np.ptp(local[:, 1])), abs=1e-6)
