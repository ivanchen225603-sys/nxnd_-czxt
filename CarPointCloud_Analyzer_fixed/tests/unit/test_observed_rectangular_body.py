"""短矩形箱车与长车分割隔离，以及外包络尺寸门禁。"""

import json

import numpy as np
import pytest
from scipy.spatial import cKDTree

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.dimensions import DimensionCalculator
from carpointcloud_analyzer.domain.carriage import CarriageProfile
from carpointcloud_analyzer.application.exporters import ResultExporter
from carpointcloud_analyzer.domain.geometry import LocalCarriageExtractor
from carpointcloud_analyzer.domain.models import (
    CarriageSplitResult, LongitudinalMeasurement, VehicleDimensions,
)
from carpointcloud_analyzer.domain.models import BoxCarriageBox
from carpointcloud_analyzer.application.partial_measurements import build_partial_measurements
from carpointcloud_analyzer.domain.models import LoadStatusResult
from carpointcloud_analyzer.domain.rectangular_completion import (
    completed_cloud_dimensions, completed_inner_dimensions,
    complete_rectangular_interior_gaps,
)
from carpointcloud_analyzer.domain.wall_reference import observe_rectangular_body


def _box_with_visible_floor_and_cab() -> np.ndarray:
    rng = np.random.default_rng(913)
    walls = [np.column_stack((
        rng.uniform(-1.25, 1.25, count), rng.normal(wall_l, .008, count),
        rng.uniform(1.5, 4.1, count),
    )) for wall_l, count in ((0., 7000), (6., 6000))]
    floor = np.column_stack((rng.uniform(-1, 1, 3000),
                             rng.uniform(.3, 5.7, 3000), np.full(3000, 1.5)))
    cab = np.column_stack((rng.uniform(-1, 1, 5000),
                           rng.uniform(7, 15, 5000), rng.uniform(.4, 2.8, 5000)))
    return np.vstack((*walls, floor, cab))


def test_visible_interior_box_uses_outer_end_walls_instead_of_cab_peak():
    points = _box_with_visible_floor_and_cab()
    box = observe_rectangular_body(points, Config())
    assert box is not None
    assert box.length == pytest.approx(6., abs=.08)
    split = LocalCarriageExtractor.split_observed_box(points, box)
    assert split.boundary_method == 'observed_rectangular_outer_faces'
    assert not split.boundaries_are_inner_faces
    assert split.split_y - split.carriage_start_y == pytest.approx(6., abs=.08)
    assert split.head_y - split.tail_y > 14.


def test_observer_does_not_take_standalone_or_unsupported_length():
    points = _box_with_visible_floor_and_cab()
    assert observe_rectangular_body(points[points[:, 1] < 6.5], Config()) is None
    assert observe_rectangular_body(points, Config(
        carriage_wall_reference_profile_length_range=(10., 12.),
    )) is None


def test_inner_length_and_width_above_observed_outer_box_are_removed_not_clamped():
    points = _box_with_visible_floor_and_cab()
    box = observe_rectangular_body(points, Config())
    split = LocalCarriageExtractor.split_observed_box(points, box)
    split.longitudinal_measurement = LongitudinalMeasurement(
        0., 11.5, None, {'definition': 'same_side_lower_band_longitudinal_projection'},
    )
    dimensions = VehicleDimensions(15., 2.4, 4., 11.5, 3.2, 2.6)
    dimensions.width_measurement = {'valid': True, 'widthM': 3.2,
                                    'adoptedSource': 'observed_inner_side_faces'}
    DimensionCalculator.validate_observed_box_inner_dimensions(dimensions, split)
    assert dimensions.vehicle_width == pytest.approx(box.width)
    assert dimensions.carriage_outer_width == pytest.approx(box.width)
    assert np.isnan(dimensions.carriage_length)
    assert np.isnan(dimensions.carriage_width)
    assert not split.longitudinal_measurement.valid
    assert split.longitudinal_measurement.metadata['rejectedLengthM'] == 11.5
    assert dimensions.width_measurement['rejectedWidthM'] == 3.2
    partial = build_partial_measurements(
        dimensions, split, LoadStatusResult('Empty', .8, 'test', {}),
        None, None, {}, [], [], split.longitudinal_measurement.metadata,
        'longitudinal_inner_edges_unavailable',
    )
    assert partial['dimensions']['carriage']['length'] is None
    assert partial['dimensions']['carriage']['width'] is None
    assert partial['referenceDimensions']['carriage']['length'] == pytest.approx(box.length)
    assert partial['observedOuterBox']['widthM'] == pytest.approx(box.width, abs=.0001)


def test_completion_mirrors_full_walls_and_front_back_floor():
    rng = np.random.default_rng(281)
    parts = []
    for length_l in (0., 6.):
        face = np.column_stack((rng.uniform(-1.2, 1.2, 5500),
                                rng.normal(length_l, .006, 5500),
                                rng.uniform(.04, 2.5, 5500)))
        parts.append(face[~((face[:, 0] > .15) & (face[:, 2] < 1.25))])
    for width_w in (-1.2, 1.2):
        side = np.column_stack((rng.normal(width_w, .006, 4500),
                                rng.uniform(.1, 5.9, 4500),
                                rng.uniform(.04, 2.5, 4500)))
        if width_w > 0:
            side = side[~((side[:, 1] > 4.2) & (side[:, 2] < 1.5))]
        parts.append(side)
    parts.append(np.column_stack((rng.uniform(-.95, .95, 5500),
                                  rng.uniform(.2, 2.9, 5500),
                                  rng.normal(0., .006, 5500))))
    observed = np.vstack(parts)
    original = observed.copy()
    box = BoxCarriageBox(6., 0., (-1.2, 1.2), (0., 2.5), 5000, 5000)
    completion = complete_rectangular_interior_gaps(
        observed, box, (-1., 1.), 0., 2.5, Config())
    assert completion.metadata['symmetryPlane']['offsetW'] == 0.
    assert completion.metadata['observedPointsModified'] == 0
    assert completion.metadata['intensity']['reason'] == 'source_pcd_xyz_only'
    assert completion.metadata['regionPointCounts']['frontWall'] > 0
    assert completion.metadata['regionPointCounts']['rearWall'] > 0
    assert completion.metadata['regionPointCounts']['sideWalls'] > 0
    assert completion.metadata['regionPointCounts']['frontFloor'] > 0
    assert completion.metadata['frontWall']['sourceHalf'] == 'left'
    assert completion.metadata['frontFloor']['sourceHalf'] == 'rear'
    assert completion.metadata['frontFloor']['heightModel'] == 'continuous_floor_plane'
    assert completion.metadata['floorMirrorPlaneL'] == 3.
    # 前壁下部右半幅完全缺测；镜像后必须是连续面，不是几条稀疏线。
    front_added = completion.points[
        (np.abs(completion.points[:, 1] - 6.) < .12)
        & (completion.points[:, 0] > .15)
        & (completion.points[:, 2] < 1.25)]
    ww, hh = np.meshgrid(np.arange(.25, 1.15, .08),
                         np.arange(.15, 1.2, .08))
    check_grid = np.column_stack((ww.ravel(), hh.ravel()))
    assert np.mean(cKDTree(front_added[:, [0, 2]]).query(check_grid)[0] < .06) > .98
    assert len(completion.points) == completion.metadata['synthesizedPoints']
    assert np.all(completion.points[:, 1] >= -.12)
    assert np.all(completion.points[:, 1] <= 6.12)
    np.testing.assert_array_equal(observed, original)


def test_completion_never_fabricates_floor_without_current_floor_support():
    points = _box_with_visible_floor_and_cab()
    points = points[points[:, 2] > 1.8]
    box = BoxCarriageBox(6., 0., (-1.25, 1.25), (1.5, 4.1), 5000, 5000)
    completion = complete_rectangular_interior_gaps(
        points, box, (-1., 1.), 1.5, 4.1, Config())
    assert completion.metadata['frontFloor']['synthesizedPoints'] == 0


def test_completed_container_exports_dimensions_without_long_truck_endpoint_gate():
    box = BoxCarriageBox(6., 0., (-1.25, 1.25), (0., 2.9), 5000, 5000)
    split = CarriageSplitResult(np.zeros((1, 3)), 6., 0., 15., 0.,
                                observed_outer_box=box)
    split.longitudinal_measurement = LongitudinalMeasurement(
        0., 5.4176, (0., 0., -1.25), {'reason': 'ok',
                           'definition': 'main_side_wall_front_to_rear_length'})
    split.cab_measurement_method = 'container_front_gutter_clearance'
    split.cab_measurement_head_y = 15.
    split.cab_measurement_front_y = 6.
    split.cab_measurement_height = 1.4
    split.cab_measurement_point_count = 20
    profile = CarriageProfile(
        floor_z=0., wall_top_left=2.8, wall_top_right=2.8, wall_top=2.8,
        wall_depth=2.8, wall_top_diff=0., scan_incomplete_side=None,
        interior_w_range=(-1.1, 1.1), interior_mask=np.array([True]),
        median_fill_ratio=0., high_cell_fraction=0., mid_band_density=0.,
        n_grid_cells=1, sparse_bin_fraction=0., edge_sparse_length=(0., 0.),
        measurement_confidence='medium',
    )
    dims = VehicleDimensions(
        15.2409, 2.5585, 3.2626, 5.4176, 2.2265, 2.809,
        carriage_outer_width=2.5585,
        width_measurement={'valid': True, 'widthM': 2.2265,
                           'adoptedSource': 'completed_cloud_inner_side_faces'},
    )
    result = ResultExporter.build(
        [], dims, split, LoadStatusResult('Empty', .8, 'test', {}), 'container.pcd', Config(), profile=profile,
        wall_completion={'completed': True, 'completedInnerDimensions': {
            'lengthGutterClearanceM': 5.4176,
            'widthGutterClearanceM': 2.2265,
        }},
    )
    status = result['measurementStatus']
    assert status['valid'] is True
    assert status['reason'] == 'completed_container_cloud'
    assert status['lengthDefinition'] == 'main_side_wall_front_to_rear_length'
    assert status['partial'] is True  # 测试未提供底板剖面，与内长端点无关。
    assert 'source' not in result['reviewMeasurements']['fields'][
        'dimensions.carriage.height']
    assert 'source' not in result['reviewMeasurements']['fields'][
        'dimensions.vehicle.length']
    assert result['dimensions']['carriage'] == {
        'length': 5.4176, 'width': 2.2265, 'height': 2.809,
    }
    assert result['integrationPayloads']['vehicleDimensions']['dimensions'][
        'trailerLength'] == 541.8


@pytest.mark.parametrize('slope', [-.02, .02])
@pytest.mark.parametrize('source_half', ['rear', 'front'])
def test_floor_preserves_one_plane_across_mirrored_missing_half(slope, source_half):
    rng = np.random.default_rng(702)
    width = rng.uniform(-.95, .95, 7000)
    length = rng.uniform(.2, 2.9, 7000)
    if source_half == 'front':
        length = 6. - length
    height = slope * length + .003 * width + rng.normal(0., .002, 7000)
    original = np.column_stack((width, length, height))
    box = BoxCarriageBox(6., 0., (-1.2, 1.2), (0., 2.5), 5000, 5000)
    completion = complete_rectangular_interior_gaps(
        original, box, (-1., 1.), float(np.median(height)), 2.5, Config())
    target_l = 5.4 if source_half == 'rear' else .6
    mirrored = completion.floor_points[
        (np.abs(completion.floor_points[:, 1] - target_l) < .05)
        & (np.abs(completion.floor_points[:, 0]) < .2)]
    assert len(mirrored) > 20
    assert np.median(mirrored[:, 2]) == pytest.approx(slope * target_l, abs=.006)
    final_floor = np.vstack((original, completion.floor_points))
    residual = final_floor[:, 2] - slope * final_floor[:, 1] - .003 * final_floor[:, 0]
    assert np.percentile(np.abs(residual), 99) < .006
    assert completion.metadata['frontFloor']['mirroredHalfPoints'] > 0
    assert completion.metadata['frontFloor']['heightExtrapolated'] is True


def test_completed_cloud_dimensions_recomputed_from_completed_cloud():
    rng = np.random.default_rng(284)
    parts = []
    for length_l in (0., 6.):
        face = np.column_stack((rng.uniform(-1.2, 1.2, 5500),
                                rng.normal(length_l, .006, 5500),
                                rng.uniform(.04, 2.5, 5500)))
        parts.append(face[~((face[:, 0] > .15) & (face[:, 2] < 1.25))])
    for width_w in (-1.2, 1.2):
        side = np.column_stack((rng.normal(width_w, .006, 4500),
                                rng.uniform(.1, 5.9, 4500),
                                rng.uniform(.04, 2.5, 4500)))
        if width_w > 0:
            side = side[~((side[:, 1] > 4.2) & (side[:, 2] < 1.5))]
        parts.append(side)
    parts.append(np.column_stack((rng.uniform(-.95, .95, 5500),
                                  rng.uniform(.2, 2.9, 5500),
                                  rng.normal(0., .006, 5500))))
    observed = np.vstack(parts)
    box = BoxCarriageBox(6., 0., (-1.2, 1.2), (0., 2.5), 5000, 5000)
    completion = complete_rectangular_interior_gaps(
        observed, box, (-1., 1.), 0., 2.5, Config())
    assert completion.metadata['completed']
    dimensions = completed_cloud_dimensions(
        observed, completion.points, box, percentile_low=1., percentile_high=99.)
    assert dimensions is not None
    # 严格 JSON 序列化，供 Worker/WebApi 完整透传。
    json.dumps(dimensions, allow_nan=False)
    assert dimensions['source'] == 'observed_plus_synthetic_envelope'
    assert dimensions['synthesizedPointCount'] == len(completion.points)
    # 合成点全部落在实测点云包络加容差内：补点不得把外包络撑大。
    lower = observed.min(axis=0) - .15
    upper = observed.max(axis=0) + .15
    assert np.all((completion.points >= lower) & (completion.points <= upper))
    assert dimensions['maxVehicleEnvelopeDeltaM'] <= .05
    completed_box = dimensions['completedCloud']['carriageOuter']
    assert completed_box['lengthM'] == pytest.approx(box.length, abs=.3)
    assert completed_box['widthM'] == pytest.approx(box.width, abs=.3)
    # 异常输入显式缺测，不回退、不抛错。
    assert completed_cloud_dimensions(
        observed, np.empty((0, 3)), box, percentile_low=1., percentile_high=99.) is None
    assert completed_cloud_dimensions(
        np.vstack((observed, [[np.nan, 0., 1.]])), completion.points, box,
        percentile_low=1., percentile_high=99.) is None


def test_ordinary_long_truck_never_enters_box_completion_paths():
    """普通长车（11m 厢式车厢）不进入集装箱补点/扣水槽内尺寸路径。

    observe_rectangular_body 的现场长度窗口（默认 5.7~6.4m）即车型门禁：
    长车返回 None，pipeline 不会调用 complete_rectangular_interior_gaps
    与 completed_inner_dimensions，仍走原长车补点分支。
    """
    rng = np.random.default_rng(914)
    parts = []
    # 11m 长车厢：端壁 + 双侧壁 + 底板 + 车头，几何完整但长度超窗。
    for wall_l in (0., 11.):
        parts.append(np.column_stack((rng.uniform(-1.2, 1.2, 6000),
                                      rng.normal(wall_l, .006, 6000),
                                      rng.uniform(.1, 2.5, 6000))))
    for wall_w in (-1.2, 1.2):
        parts.append(np.column_stack((rng.normal(wall_w, .006, 6000),
                                      rng.uniform(.1, 10.9, 6000),
                                      rng.uniform(.1, 2.5, 6000))))
    parts.append(np.column_stack((rng.uniform(-.95, .95, 5000),
                                  rng.uniform(.3, 10.7, 5000),
                                  np.zeros(5000) + .1)))
    parts.append(np.column_stack((rng.uniform(-1., 1., 6000),
                                  rng.uniform(11.6, 15., 6000),
                                  rng.uniform(.3, 2.8, 6000))))
    points = np.vstack(parts)
    config = Config()
    assert observe_rectangular_body(points, config) is None
    # 车型门禁关闭后即使人为构造长箱体 Box，补点也不得产出（管线根本
    # 不会走到这里）；此处直接断言识别层已挡住，作为路径隔离的守卫。
    assert config.carriage_wall_reference_profile_length_range == (5.7, 6.4)


def test_completion_mirrors_top_gutter_shape_to_missing_half():
    """端壁顶部导水槽的凹形必须随镜像补到缺失半幅，而不是补成平壁。"""
    rng = np.random.default_rng(517)
    parts = []
    for length_l in (0., 6.):
        face = np.column_stack((rng.uniform(-1.2, 1.2, 9000),
                                rng.normal(length_l, .006, 9000),
                                rng.uniform(.04, 2.5, 9000)))
        # 顶部 15cm 水槽带整体向厢内凹 7cm，仅保留在左半幅。
        gutter = (face[:, 2] > 2.35) & (face[:, 0] < .0)
        face[gutter, 1] = face[gutter, 1] + .07 * np.sign(length_l - 3.)
        # 右半幅（含水槽位置）完全缺失。
        parts.append(face[face[:, 0] < .15])
    side = np.column_stack((rng.normal(-1.2, .006, 4500),
                            rng.uniform(.1, 5.9, 4500),
                            rng.uniform(.04, 2.5, 4500)))
    parts.append(side)
    parts.append(np.column_stack((rng.uniform(-.95, .95, 5500),
                                  rng.uniform(.2, 2.9, 5500),
                                  rng.normal(0., .006, 5500))))
    observed = np.vstack(parts)
    box = BoxCarriageBox(6., 0., (-1.2, 1.2), (0., 2.5), 5000, 5000)
    completion = complete_rectangular_interior_gaps(
        observed, box, (-1., 1.), 0., 2.5, Config())
    for boundary in (0., 6.):
        wall = completion.points[np.abs(completion.points[:, 1] - boundary) <= .12]
        gutter_added = wall[(wall[:, 0] > .15) & (wall[:, 2] > 2.35)]
        plain_added = wall[(wall[:, 0] > .15) & (wall[:, 2] < 2.2)]
        assert len(gutter_added) > 50, '缺失半幅顶部水槽位置必须有补点'
        # 水槽补点壁深相对主壁面凹进约 7cm；普通区补点仍在主壁面附近。
        expected = boundary + .07 * np.sign(boundary - 3.)
        assert abs(np.median(gutter_added[:, 1]) - expected) < .03
        assert abs(np.median(plain_added[:, 1]) - boundary) < .03
    # 侧壁同样校验：源侧形状直接传递到源面板自身空洞与对侧镜像。
    side_added = completion.points[
        (np.abs(completion.points[:, 0] + 1.2) <= .12)
        | (np.abs(completion.points[:, 0] - 1.2) <= .12)]
    assert len(side_added) > 50


def _gutter_box_observed_and_synthetic() -> tuple[np.ndarray, np.ndarray]:
    """构造带侧壁水槽（12cm）与端壁顶梁层（22cm）的完整箱体实测云。

    主体内面：左右壁 W=±1.1、前后壁 L=0.15/5.85；水槽带（顶部 35cm）
    内侧壁凹至 ±0.98、端壁顶梁凹至 0.37/5.63；底板 H=0.1。
    """
    rng = np.random.default_rng(611)
    parts = []
    for wall_w, gutter_w in ((-1.1, -.98), (1.1, .98)):
        side = np.column_stack((rng.normal(wall_w, .004, 6000),
                                rng.uniform(.6, 5.4, 6000),
                                rng.uniform(.15, 2.45, 6000)))
        gutter = np.column_stack((rng.normal(gutter_w, .004, 1200),
                                  rng.uniform(.6, 5.4, 1200),
                                  rng.uniform(2.16, 2.5, 1200)))
        parts.extend([side, gutter])
    for boundary, layer_l in ((.15, .37), (5.85, 5.63)):
        face = np.column_stack((rng.uniform(-.9, .9, 5000),
                                rng.normal(boundary, .004, 5000),
                                rng.uniform(.15, 2.45, 5000)))
        layer = np.column_stack((rng.uniform(-.9, .9, 900),
                                 rng.normal(layer_l, .004, 900),
                                 rng.uniform(2.16, 2.5, 900)))
        parts.extend([face, layer])
    parts.append(np.column_stack((rng.uniform(-.9, .9, 4000),
                                  rng.uniform(.6, 5.4, 4000),
                                  rng.normal(.1, .004, 4000))))
    observed = np.vstack(parts)
    synthetic = np.column_stack((rng.normal(.98, .004, 200),
                                 rng.uniform(.6, 5.4, 200),
                                 rng.uniform(2.16, 2.5, 200)))
    return observed, synthetic


def test_completed_inner_dimensions_deduct_gutter_clearance():
    observed, synthetic = _gutter_box_observed_and_synthetic()
    # 箱体边界与端壁主面一致（同真实识别：rear_l/front_l 即端壁位置）。
    box = BoxCarriageBox(5.85, .15, (-1.2, 1.2), (0., 2.5), 5000, 5000)
    dims = completed_inner_dimensions(observed, synthetic, box, .1, 2.4)
    assert dims is not None
    json.dumps(dims, allow_nan=False)
    assert dims['source'] == 'observed_plus_synthetic_completed_cloud'
    # 主体壁面口径：全高可用内尺寸。
    assert dims['widthMainM'] == pytest.approx(2.2, abs=.03)
    assert dims['lengthMainM'] == pytest.approx(5.7, abs=.03)
    # 侧壁水槽凹进约 12cm，扣减后净宽。
    assert dims['widthGutterClearanceM'] == pytest.approx(1.96, abs=.03)
    assert dims['perFace']['leftWall']['recessM'] == pytest.approx(.12, abs=.02)
    assert dims['perFace']['rightWall']['recessM'] == pytest.approx(.12, abs=.02)
    # 端壁顶梁层凹进约 22cm：最窄面与层中心双口径。
    assert dims['lengthGutterClearanceM'] == pytest.approx(5.26, abs=.05)
    assert dims['lengthGutterLayerM'] == pytest.approx(5.26, abs=.05)
    assert dims['perFace']['rearWall']['recessM'] == pytest.approx(.22, abs=.03)
    assert dims['perFace']['frontWall']['recessM'] == pytest.approx(.22, abs=.03)
    assert dims['perFace']['rearWall']['evidencePoints'] > 100
    # 内高：底板到侧壁顶沿。
    assert dims['heightM'] == pytest.approx(2.4, abs=.05)
    # 非法输入显式缺测。
    assert completed_inner_dimensions(
        observed, np.empty((0, 3)), box, .1, 2.4) is None
    assert completed_inner_dimensions(
        np.vstack((observed, [[np.nan, 0., 1.]])), synthetic, box, .1, 2.4) is None
    assert completed_inner_dimensions(
        observed, synthetic, box, 2.4, .1) is None
