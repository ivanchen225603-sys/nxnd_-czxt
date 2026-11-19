"""物理端壁定位回归，不使用现场车辆尺寸作为算法标准。"""

import numpy as np
import pytest

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.dimensions import DimensionCalculator
from carpointcloud_analyzer.domain.geometry import LocalCarriageExtractor
from carpointcloud_analyzer.domain.inner_faces import measure_end_faces
from carpointcloud_analyzer.domain.models import CarriageSplitResult


def _wall(rng, wall_l, n=4000, slope=0.0, h_range=(0.0, 2.0)):
    w = rng.uniform(-1.2, 1.2, n)
    h = rng.uniform(*h_range, n)
    return np.column_stack((w, wall_l + slope * h + rng.normal(0, 0.006, n), h))


def _vehicle(seed=17, slope=0.0):
    rng = np.random.default_rng(seed)
    walls = [_wall(rng, wall_l, slope=slope) for wall_l in (0.0, 0.18, 9.72, 9.90)]
    floor = np.column_stack((rng.uniform(-1.2, 1.2, 5000),
                             rng.uniform(0.18, 9.72, 5000), rng.normal(0, .005, 5000)))
    cab = np.column_stack((rng.uniform(-1, 1, 1200), rng.uniform(10.5, 12, 1200),
                           rng.uniform(0.4, 2.2, 1200)))
    return np.vstack((*walls, floor, cab))


@pytest.mark.parametrize('bin_width', [.03, .05, .08])
def test_surface_boundaries_do_not_inherit_histogram_smoothing_width(bin_width):
    split = LocalCarriageExtractor(Config(rib_bin_width=bin_width)).split(_vehicle())
    assert split.boundaries_are_inner_faces
    assert split.boundary_method == 'observed_wall_surface_fit'
    assert split.carriage_start_y == pytest.approx(.18, abs=.015)
    assert split.split_y == pytest.approx(9.72, abs=.015)
    assert split.split_y - split.carriage_start_y == pytest.approx(9.54, abs=.02)
    assert split.rear_wall_plane is not None
    assert split.rear_wall_plane[2] == pytest.approx(.18, abs=.015)


def test_surface_fit_is_point_order_independent():
    points = _vehicle()
    first = LocalCarriageExtractor(Config()).split(points)
    second = LocalCarriageExtractor(Config()).split(points[np.random.default_rng(1).permutation(len(points))])
    assert first.split_y == pytest.approx(second.split_y, abs=1e-9)
    assert first.carriage_start_y == pytest.approx(second.carriage_start_y, abs=1e-9)


def test_top_beam_is_not_a_full_inner_face():
    rng = np.random.default_rng(5)
    points = np.vstack((_wall(rng, 0), _wall(rng, 10),
                        _wall(rng, .22, n=10000, h_range=(1.93, 2.0)),
                        _wall(rng, 9.78, n=10000, h_range=(1.93, 2.0))))
    faces = measure_end_faces(points, .1, 9.9)
    assert faces is not None
    rear, front, w, h = faces
    assert rear.at(w, h) == pytest.approx(0, abs=.02)
    assert front.at(w, h) == pytest.approx(10, abs=.02)


def test_missing_end_face_is_not_replaced_by_symmetry_for_measurement():
    rng = np.random.default_rng(5)
    assert measure_end_faces(_wall(rng, 0), 0, 10) is None


def test_diagonal_ribs_with_large_marginal_spans_are_not_wall_surfaces():
    rng = np.random.default_rng(9)
    w = rng.uniform(-1.2, 1.2, 5000)
    rear = np.column_stack((w, rng.normal(0, .003, len(w)),
                            w + rng.normal(0, .008, len(w))))
    front = rear + [0., 10., 0.]
    assert measure_end_faces(np.vstack((rear, front)), 0., 10.) is None


def test_tilted_front_face_is_projected_to_business_ground_height():
    rng = np.random.default_rng(6)
    points = np.column_stack((rng.uniform(-.2, .2, 200),
                             rng.uniform(8.9, 9.0, 200), rng.uniform(1.39, 1.41, 200)))
    split = CarriageSplitResult(np.empty((0, 3)), 5.2, -5, 9.3, -5.1,
                                front_wall_plane=(0.0, .1, 5.0))
    assert DimensionCalculator(Config()).measure_cab_to_front_wall_at_height(
        points, split, np.array([0., 0., 1., 0.]))
    assert split.cab_measurement_front_y == pytest.approx(5.14)
    assert split.cab_to_front_wall == pytest.approx(np.percentile(points[:, 1], 99) - 5.14)


def test_sloped_wall_pair_uses_common_height_not_different_point_density():
    faces = measure_end_faces(_vehicle(slope=.06), .1, 9.8)
    assert faces is not None
    rear, front, w, h = faces
    assert front.at(w, h) - rear.at(w, h) == pytest.approx(9.54, abs=.02)
