"""最终云的补底不穿过货面，重复补点不改变空间填充统计。"""

import numpy as np
import pytest

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.carriage import CarriageAnalyzer
from carpointcloud_analyzer.domain.floor_evidence import lower_wall_floor_conflict
from carpointcloud_analyzer.domain.models import BoxCarriageBox, CarriageSplitResult
from carpointcloud_analyzer.application.pipeline import PointCloudAnalyzer
from carpointcloud_analyzer.domain.rectangular_completion import _complete_floor


def _sample(cargo=True):
    rng = np.random.default_rng(20261022)
    floor = np.c_[rng.uniform(-.95, .95, 30000), rng.uniform(.2, 2.65, 30000),
                  np.zeros(30000)]
    length, height = np.meshgrid(np.linspace(.2, 5.8, 151), np.linspace(0, 2.5, 31))
    walls = np.vstack([np.c_[np.full(length.size, side), length.ravel(), height.ravel()]
                       for side in (-1.2, 1.2)])
    width, length = np.meshgrid(np.linspace(-.9, .9, 41), np.linspace(2.7, 5.8, 43))
    surface = (np.c_[width.ravel(), length.ravel(), np.full(width.size, 1.4)]
               if cargo else np.empty((0, 3)))
    return np.vstack((floor, surface, walls))


def _complete(cloud):
    box = BoxCarriageBox(6., 0., (-1.2, 1.2), (0., 2.5), 5000, 5000)
    return _complete_floor(cloud, cloud, box, (-1.05, 1.05), 0.)


def _assess(observed, final):
    analyzer = PointCloudAnalyzer(Config(save_clean_pointcloud=False))
    split = CarriageSplitResult(observed, 6., 0., 10., 0., boundaries_are_inner_faces=True)
    profile = analyzer.carriage_analyzer.analyze(final)
    return analyzer._assess_load(
        split, profile,
        final_carriage_points=final, ribs=[],
    )


def test_current_cargo_projection_blocks_synthetic_floor():
    cloud = _sample()
    added, info = _complete(cloud)
    assert len(added) > 0
    assert info['occludedFloorCandidates'] > 0
    assert not np.any((added[:, 0] >= -.9) & (added[:, 0] <= .9)
                      & (added[:, 1] >= 2.7) & (added[:, 1] <= 5.8))


def test_empty_visible_floor_can_still_mirror_without_moving_observations():
    cloud = _sample(cargo=False)
    original = cloud.copy()
    added, _ = _complete(cloud)
    assert np.count_nonzero(added[:, 1] > 3.) > 100
    np.testing.assert_array_equal(cloud, original)


def test_obstruction_height_uses_the_same_continuous_plane_as_generated_floor():
    cloud = _sample(cargo=False)
    floor = np.abs(cloud[:, 0]) < 1.
    cloud[floor, 2] = .02 * cloud[floor, 1] + .003 * cloud[floor, 0]
    width, length = np.meshgrid(np.linspace(-.9, .9, 41), np.linspace(4.5, 5.8, 43))
    cargo = np.c_[width.ravel(), length.ravel(), np.full(width.size, .27)]
    added, info = _complete(np.vstack((cloud, cargo)))
    assert info['occludedFloorCandidates'] > 0
    assert not np.any((added[:, 0] >= -.9) & (added[:, 0] <= .9)
                      & (added[:, 1] >= 4.5) & (added[:, 1] <= 5.8))


@pytest.mark.parametrize('repeats', [1, 50])
def test_synthetic_floor_density_cannot_change_load_evidence(repeats):
    cloud = _sample()
    # 模拟旧版本会生成的完整底板，确保即使旧候选被传入也不影响载荷判断。
    floor, _ = _complete(_sample(cargo=False))
    before = _assess(cloud, cloud)
    after = _assess(cloud, np.vstack((cloud, np.tile(floor, (repeats, 1)))))
    assert before.status != 'Empty'
    once = _assess(cloud, np.vstack((cloud, floor)))
    assert after.status != 'Empty'
    assert after.status == once.status
    assert after.metrics == once.metrics


def test_final_floor_geometry_is_used_consistently_with_completed_walls():
    cloud = _sample(cargo=False)
    walls = cloud[np.abs(cloud[:, 0]) > 1.]
    result = _assess(walls, cloud)
    assert result.status == 'Empty'


def _continuous_surface(surface_h):
    length, height = np.meshgrid(np.linspace(.2, 5.8, 151), np.linspace(0., 2.5, 31))
    walls = np.vstack([np.c_[np.full(length.size, side), length.ravel(), height.ravel()]
                       for side in (-1.2, 1.2)])
    width, length = np.meshgrid(np.linspace(-.9, .9, 41), np.linspace(.2, 5.8, 101))
    return np.vstack((walls, np.c_[width.ravel(), length.ravel(),
                                   np.full(width.size, surface_h)]))


@pytest.mark.parametrize('height_shift', [0., 1.233, -4.2])
def test_continuous_cargo_surface_is_not_floor_when_both_observed_walls_extend_below(height_shift):
    cloud = _continuous_surface(1.4)
    cloud[:, 2] += height_shift
    result = _assess(cloud, cloud)
    assert result.status == 'Uncertain'
    assert result.reason == 'observed_floor_contradicts_inner_walls'


@pytest.mark.parametrize('height_shift', [0., 1.233])
def test_current_empty_floor_is_not_rejected_by_absolute_ground_height(height_shift):
    cloud = _continuous_surface(0.)
    cloud[:, 2] += height_shift
    assert _assess(cloud, cloud).status == 'Empty'


def test_horizontal_footboards_and_external_skirts_do_not_prove_lower_inner_walls():
    cloud = _continuous_surface(0.)
    config = Config()
    profile = CarriageAnalyzer(config).analyze(cloud)
    width, length = np.meshgrid(np.linspace(1.15, 1.25, 11), np.linspace(.2, 5.8, 151))
    foot = np.c_[width.ravel(), length.ravel(), np.full(width.size, -.9)]
    length, height = np.meshgrid(np.linspace(.2, 5.8, 151), np.linspace(-1., -.3, 21))
    skirt = np.c_[np.full(length.size, 1.6), length.ravel(), height.ravel()]
    for attachment in (foot, skirt):
        mirror = attachment.copy()
        mirror[:, 0] *= -1
        evidence = lower_wall_floor_conflict(
            np.vstack((cloud, attachment, mirror)), profile, config)
        assert evidence['conflict'] is False
