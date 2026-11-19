"""整厢来源选择不按点密度/拉筋结果投票，不能裁掉另一视角独有货物。"""

import numpy as np
import pytest

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.geometry import CoordinateTransform
from carpointcloud_analyzer.domain.models import CarriageSplitResult
from carpointcloud_analyzer.domain.regional_fusion import (
    RegionalSelection, _cab_boundary, _coverage, _uncovered_interior, select_carriage_source,
)


def _surface(first, second, axis, value):
    grid = np.meshgrid(first, second)
    points = np.zeros((grid[0].size, 3))
    points[:, [i for i in range(3) if i != axis]] = np.c_[grid[0].ravel(), grid[1].ravel()]
    points[:, axis] = value
    return points


def _body():
    w = np.arange(-1.2, 1.21, .06)
    length = np.arange(0., 10.01, .06)
    h = np.arange(1.5, 3.01, .06)
    return np.vstack([_surface(w, h, 1, end) for end in (0., 10.)]
                     + [_surface(length, h, 0, side) for side in (-1.2, 1.2)]
                     + [_surface(w, length, 2, 1.5)])


@pytest.mark.parametrize('primary', [0, 1])
@pytest.mark.parametrize('sign', [-1, 1])
def test_masks_keep_whole_source_and_only_other_cab_in_any_frame(primary, sign):
    rotation = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])
    transform = CoordinateTransform(np.array([12., -4., 7.]), rotation)
    local = np.array([[0., -3., 1.5], [0., 0., 2.], [0., 5., 2.], [0., 13., 1.4]])
    sources = [transform.to_source(local), transform.to_source(local + [0., .06, .06])]
    original = [points.copy() for points in sources]
    boundary = -.35 if sign < 0 else 10.35
    selection = RegionalSelection(transform, sign, boundary, primary)
    masks = selection.masks(*sources)
    assert masks[primary].all()
    assert np.flatnonzero(masks[1-primary]).tolist() == ([0] if sign < 0 else [3])
    for points, before in zip(sources, original):
        np.testing.assert_array_equal(points, before)


def test_coverage_is_density_invariant_and_rejects_half_carriage():
    body = _body()
    first = _coverage(body, 0., 10., np.array([-1.2, 1.2]), np.array([1.5, 3.]))
    repeated = _coverage(np.repeat(body, 3, axis=0), 0., 10.,
                         np.array([-1.2, 1.2]), np.array([1.5, 3.]))
    assert first == repeated
    assert first['eligible']
    half = _coverage(body[body[:, 1] < 5.], 0., 10.,
                     np.array([-1.2, 1.2]), np.array([1.5, 3.]))
    assert not half['eligible']


@pytest.mark.parametrize('kind', ['cargo', 'thin_cargo', 'floor_patch', 'rib', 'shadow',
                                 'vertical_cargo', 'near_end_cargo', 'near_wall_cargo', 'diagonal_rib'])
def test_secondary_interior_surface_cannot_be_deleted(kind):
    primary = _body()
    if kind in ('cargo', 'thin_cargo'):
        height = 2.5 if kind == 'cargo' else 1.65
        secondary = _surface(np.arange(-.7, .71, .04), np.arange(3., 5., .04), 2, height)
    elif kind == 'floor_patch':
        secondary = _surface(np.arange(-.7, .71, .04), np.arange(3., 5., .04), 2, 1.5)
        primary = primary[(primary[:, 1] < 2.5) | (primary[:, 1] > 5.5)]
    elif kind == 'rib':
        secondary = np.c_[np.arange(-1., 1.01, .005),
                           np.full(402, 4.), np.full(402, 2.5)]
    elif kind == 'diagonal_rib':
        widths = np.arange(-1., 1.01, .005)
        secondary = np.c_[widths, 4. + .3 * widths, 2.5 + .1 * widths]
    elif kind == 'vertical_cargo':
        secondary = _surface(np.arange(2., 8., .04), np.arange(1.7, 2.81, .04), 0, .3)
    elif kind == 'near_end_cargo':
        secondary = _surface(np.arange(-.7, .71, .04), np.arange(.12, .34, .04), 2, 2.5)
    elif kind == 'near_wall_cargo':
        secondary = _surface(np.arange(-1.04, -.85, .04), np.arange(2., 8., .04), 2, 2.5)
    else:
        secondary = primary + [0., .06, .06]
    assert _uncovered_interior(primary, secondary, 0., 10., np.array([-1.05, 1.05]), .1) == (
        kind not in ('rib', 'diagonal_rib', 'shadow'))


@pytest.mark.parametrize('primary', [0, 1])
def test_real_partition_selects_complete_body_not_sparse_ghosts(primary):
    body = _body()
    cab = np.random.default_rng(4).uniform([-1.2, -4., .5], [1.2, -1., 3.], (6000, 3))
    sources = [np.vstack((body[::100] + [0., .06, .06], cab)), body]
    if primary == 0:
        sources.reverse()
    merged = np.vstack(sources)
    selection, metadata = select_carriage_source(merged, *sources, Config())
    assert selection is not None, metadata
    assert selection.primary == primary
    masks = selection.masks(*sources)
    assert masks[primary].all()
    assert masks[1-primary].sum() == len(cab)


@pytest.mark.parametrize('has_plane', [True, False])
@pytest.mark.parametrize('sign', [-1, 1])
def test_tilted_end_wall_is_wholly_owned_by_primary_including_short_box(has_plane, sign):
    face = _surface(np.arange(-1.2, 1.21, .04), np.arange(0., 2.61, .04), 1, 0.)
    face[:, 1] = .05 * face[:, 0] + .12 * face[:, 2]
    split = CarriageSplitResult(face, .15 if sign > 0 else 6.,
                                .15 if sign < 0 else -6., 9., -9.)
    if has_plane:
        if sign < 0:
            split.rear_wall_plane = (.05, .12, 0.)
        else:
            split.front_wall_plane = (.05, .12, 0.)
    boundary = _cab_boundary(face, split, sign, np.array([-1.2, 1.2]), np.array([0., 2.6]))
    assert boundary is not None
    assert np.max(sign * (face[:, 1] - boundary)) <= -.34
