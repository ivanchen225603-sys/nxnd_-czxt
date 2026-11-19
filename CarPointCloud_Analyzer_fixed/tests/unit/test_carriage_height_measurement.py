"""最终车厢云的连续壁顶测高，排除端部高物和只有底板的伪侧壁支持。"""

from types import SimpleNamespace

import numpy as np
import pytest

from carpointcloud_analyzer.domain.carriage import CarriageAnalyzer
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.dimensions import _measure_continuous_side_height
from carpointcloud_analyzer.domain.geometry import PointCloudGeometryNormalizer


def _floor():
    width, length = np.meshgrid(np.linspace(-1.2, 1.2, 61), np.linspace(0., 11., 241))
    return np.c_[width.ravel(), length.ravel(), np.full(width.size, 1.4)]


def _wall(side, top=3.0):
    length, height = np.meshgrid(np.linspace(0., 11., 241), np.linspace(1.4, top, 81))
    return np.c_[np.full(length.size, side), length.ravel(), height.ravel()]


def _profile(top=3.0):
    return SimpleNamespace(floor_z=1.4, wall_top=top, interior_w_range=(-1.05, 1.05))


def test_dense_floor_edges_do_not_replace_the_side_rim_with_zero_height():
    # 中部侧壁缺失，底板两侧边缘再密也不能作为竖壁上沿。
    height, info = _measure_continuous_side_height(_floor(), _profile(), Config())
    assert np.isnan(height)
    assert info['method'] == 'side_rim_unavailable'
    assert info['reason'] == 'insufficient_supported_side_sections'
    assert info['supportBinCount'] == {'left': 0, 'right': 0}


def test_local_end_panel_does_not_raise_continuous_side_height():
    wall = _wall(-1.2)
    end_panel = _wall(-1.2, top=4.2)
    end_panel = end_panel[(end_panel[:, 1] < .5) | (end_panel[:, 1] > 10.5)]
    points = np.vstack([_floor(), wall, _wall(1.2), end_panel])
    height, info = _measure_continuous_side_height(points, _profile(top=4.2), Config())
    assert height == pytest.approx(1.6, abs=.05)
    assert info['method'] == 'central_side_rim_longitudinal_bin_median'
    assert info['topH'] < 3.05


@pytest.mark.parametrize('complete_side', [-1.2, 1.2])
def test_both_sides_contribute_sections_to_the_height_median(complete_side):
    points = np.vstack([_floor(), _wall(complete_side), _wall(-complete_side, top=2.2)])
    height, info = _measure_continuous_side_height(points, _profile(), Config())
    # 用户确认合并两侧有效段：12段1.6m与12段0.8m的中位数是1.2m。
    # 保留原点云及5cm容差，只替换旧“取较高侧”的业务期望。
    assert height == pytest.approx(1.2, abs=.05)
    assert info['selectedSide'] == 'both'


def test_side_height_median_weights_sections_not_side_medians():
    high = _wall(1.2, top=3.8)
    high = high[high[:, 1] < 7.5]
    points = np.vstack([_floor(), _wall(-1.2), high])
    height, info = _measure_continuous_side_height(points, _profile(), Config())
    assert info['supportBinCount']['left'] == 12
    assert 6 <= info['supportBinCount']['right'] < 12
    # 低侧有效段占多数；不能把两侧先各压成一个数再平均成2.0m。
    assert height == pytest.approx(1.6, abs=.05)


def test_insufficient_longitudinal_wall_support_leaves_height_unavailable():
    wall = _wall(-1.2)
    points = np.vstack([_floor(), wall[(wall[:, 1] >= 3.) & (wall[:, 1] <= 4.)]])
    height, info = _measure_continuous_side_height(points, _profile(), Config())
    assert np.isnan(height)
    assert info['method'] == 'side_rim_unavailable'
    assert info['reason'] == 'insufficient_supported_side_sections'


@pytest.mark.parametrize('wall_top,minimum_height', [(1.85, .5), (3., 2.)])
def test_low_continuous_wall_is_not_replaced_by_high_end_panels(wall_top, minimum_height):
    config = Config(sanity_vehicle_height=(minimum_height, 5.))
    end_panel = _wall(-1.2, top=4.2)
    end_panel = end_panel[(end_panel[:, 1] < .5) | (end_panel[:, 1] > 10.5)]
    points = np.vstack([_floor(), _wall(-1.2, wall_top), _wall(1.2, wall_top), end_panel])
    profile = CarriageAnalyzer(config).analyze(points)
    height, info = _measure_continuous_side_height(points, profile, config)
    assert height == pytest.approx(wall_top - 1.4, abs=.05)
    assert height < minimum_height
    assert info['method'] == 'central_side_rim_longitudinal_bin_median'


@pytest.mark.parametrize('vertical_candidate', [False, True])
def test_pose_filter_does_not_erode_measured_continuous_rim(vertical_candidate):
    # 6cm左右的采样，顶沿有整行实测支持；SOR会因边缘邻域只有半面而删掉顶行。
    length, height = np.meshgrid(np.linspace(0., 10., 168), np.linspace(1.6, 3.6, 34))
    walls = [np.c_[np.full(length.size, side), length.ravel(), height.ravel()]
             for side in (-1.2, 1.2)]
    width, length = np.meshgrid(np.linspace(-1.2, 1.2, 41), np.linspace(0., 10., 168))
    floor = np.c_[width.ravel(), length.ravel(), np.full(width.size, 1.6)]
    # 四个孤立高回波不能改变建系法向，也不能抬高连续侧壁上沿。
    spikes = np.array([[-1.2, 2., 6.], [1.2, 4., 7.], [-1.2, 6., 8.], [1.2, 8., 9.]])
    points = np.vstack([floor, *walls, spikes])
    normalizer = PointCloudGeometryNormalizer(Config())
    normalize = normalizer.vertical_candidate if vertical_candidate else normalizer.process
    local, transform = normalize(points, np.array([0., 0., 1.]), 'weighbridge')
    np.testing.assert_allclose(transform.rotation[:, 2], [0., 0., 1.], atol=1e-10)
    profile = CarriageAnalyzer(Config()).analyze(local)
    measured, info = _measure_continuous_side_height(local, profile, Config())
    assert measured == pytest.approx(2., abs=.05)
    # 建系保留去噪，但不能把用于拟合姿态的删点掩码当作正式量测点集。
    reconstructed = transform.to_source(local)
    np.testing.assert_allclose(
        reconstructed[np.lexsort(np.round(reconstructed, 8).T[::-1])],
        points[np.lexsort(points.T[::-1])], atol=1e-10)
    assert info['supportBinCount'] == {'left': 12, 'right': 12}


@pytest.mark.parametrize('noisy_side', [-1., 1.])
def test_wall_exterior_returns_do_not_raise_the_measured_rim(noisy_side):
    # 与联合点云相同的 6cm 采样；壁外杂点分布在整段车长，不能靠纵向中位数排除。
    length, height = np.meshgrid(np.arange(0., 10.001, .06), np.arange(1.6, 3.601, .06))
    walls = [np.c_[np.full(length.size, side), length.ravel(), height.ravel()]
             for side in (-1.2, 1.2)]
    width, length = np.meshgrid(np.arange(-1.2, 1.201, .06), np.arange(0., 10.001, .06))
    floor = np.c_[width.ravel(), length.ravel(), np.full(width.size, 1.6)]
    random = np.random.default_rng(5103)
    halo = random.uniform([1.18, .5, 3.60], [1.30, 9.5, 3.90], (450, 3))
    halo[:, 0] *= noisy_side
    points = np.vstack([floor, *walls, halo])
    profile = CarriageAnalyzer(Config()).analyze(points)
    measured, info = _measure_continuous_side_height(points, profile, Config())
    assert measured == pytest.approx(2., abs=.05)
    assert info['supportBinCount'] == {'left': 12, 'right': 12}


def test_outward_folded_continuous_rim_is_not_clipped_to_the_main_wall():
    length, height = np.meshgrid(np.linspace(0., 10., 168), np.linspace(1.6, 3.6, 34))
    width = 1.2 + np.maximum(height - 3.3, 0.) / 3.
    walls = [np.c_[sign * width.ravel(), length.ravel(), height.ravel()] for sign in (-1., 1.)]
    width, length = np.meshgrid(np.linspace(-1.2, 1.2, 41), np.linspace(0., 10., 168))
    floor = np.c_[width.ravel(), length.ravel(), np.full(width.size, 1.6)]
    points = np.vstack([floor, *walls])
    profile = CarriageAnalyzer(Config()).analyze(points)
    measured, info = _measure_continuous_side_height(points, profile, Config())
    assert measured == pytest.approx(2., abs=.05)
    assert info['supportBinCount'] == {'left': 12, 'right': 12}


@pytest.mark.parametrize('gap', [.14, .4])
@pytest.mark.parametrize('accessory', ['side_rod', 'top_beam', 'canopy_crossbars'])
def test_detached_accessories_do_not_define_the_side_wall_rim(accessory, gap):
    points = np.vstack([_floor(), _wall(-1.2), _wall(1.2)])
    if accessory == 'side_rod':
        length = np.linspace(0., 11., 2201)
        attachments = np.vstack([
            np.c_[np.full(len(length), side), length, np.full(len(length), 3. + gap)]
            for side in (-1.2, 1.2)])
    elif accessory == 'top_beam':
        length, height = np.meshgrid(np.linspace(0., 11., 1101),
                                    np.linspace(3. + gap, 3. + gap + .06, 7))
        attachments = np.vstack([
            np.c_[np.full(length.size, side), length.ravel(), height.ravel()]
            for side in (-1.2, 1.2)])
    else:
        width, length = np.meshgrid(np.linspace(-1.3, 1.3, 1301), np.arange(.5, 11., .5))
        attachments = np.c_[width.ravel(), length.ravel(), np.full(width.size, 3. + gap)]
    measured, _ = _measure_continuous_side_height(np.vstack([points, attachments]), _profile(), Config())
    # 独立横杆、梁和篷杆不改变底板到真实侧壁上沿的1.6m，沿用已有5cm容差。
    assert measured == pytest.approx(1.6, abs=.05)


def test_sparse_detached_rod_cannot_bridge_to_wall_through_fitting_neighbors():
    # 融合后的采样约5cm：20点拟合邻域能跨过16cm空隙，不能视作结构相连。
    length, height = np.meshgrid(np.arange(0., 10.001, .06), np.arange(1.6, 3.601, .06))
    walls = np.vstack([np.c_[np.full(length.size, side), length.ravel(), height.ravel()]
                       for side in (-1.2, 1.2)])
    length = np.arange(.5, 9.501, .05)
    rods = np.vstack([np.c_[np.full(len(length), side), length, np.full(len(length), 3.74)]
                      for side in (-1.2, 1.2)])
    profile = SimpleNamespace(floor_z=1.6, wall_top=3.74, interior_w_range=(-1.05, 1.05))
    measured, _ = _measure_continuous_side_height(np.vstack((walls, rods)), profile, Config())
    assert measured == pytest.approx(2., abs=.05)


@pytest.mark.parametrize('high_at_front', [True, False])
def test_height_pairs_each_rim_section_with_its_own_step_floor(high_at_front):
    floor = _floor()
    high = floor[:, 1] >= 3.3 if high_at_front else floor[:, 1] <= 7.7
    floor[high, 2] = 2.0
    walls = np.vstack([_wall(-1.2), _wall(1.2)])
    high_wall = walls[:, 1] >= 3.3 if high_at_front else walls[:, 1] <= 7.7
    walls = walls[~(high_wall & (walls[:, 2] < 2.0))]
    # 低板扫描更密，原全车密度峰仍取1.4m；不能把它用于占车长多数的高板。
    points = np.vstack([floor, np.repeat(floor[~high], 20, axis=0), walls])
    profile = CarriageAnalyzer(Config()).analyze(points)
    measured, _ = _measure_continuous_side_height(points, profile, Config())
    assert measured == pytest.approx(1.0, abs=.05)


def test_height_pairs_rim_with_sloping_floor_in_each_section():
    floor = _floor()
    floor[:, 2] += .06 * floor[:, 1]
    walls = np.vstack([_wall(-1.2), _wall(1.2)])
    walls = walls[walls[:, 2] >= 1.4 + .06 * walls[:, 1]]
    measured, _ = _measure_continuous_side_height(np.vstack([floor, walls]), _profile(), Config())
    assert measured == pytest.approx(3.0 - (1.4 + .06 * 5.5), abs=.05)
