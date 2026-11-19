"""顶部开口前沿：车头距与内长共用的半挂前端（2026-11-07 用户确认口径）。"""
import numpy as np
import pytest

from carpointcloud_analyzer.domain.top_opening import measure_top_opening_front_edge

STEP = .025


def _grid(w, length, h):
    a, b, c = np.meshgrid(np.arange(*w, STEP), np.arange(*length, STEP), np.arange(*h, STEP))
    return np.c_[a.ravel(), b.ravel(), c.ravel()]


def _carriage(front_top=True):
    """10m 车厢：底板 H=0、侧壁上沿 H=2、前端壁内表面 L=10。"""
    parts = [
        _grid((-1.2, 1.2), (0., 10.), (0., STEP)),
        _grid((-1.2, -1.2 + STEP), (0., 10.), (0., 2.)),
        _grid((1.2 - STEP, 1.2), (0., 10.), (0., 2.)),
        _grid((-1.2, 1.2), (0., STEP), (0., 2.)),
        _grid((-1.2, 1.2), (10. - STEP, 10.), (0., 2. if front_top else 1.2)),
    ]
    return np.vstack(parts)


def _measure(points):
    return measure_top_opening_front_edge(points, points, 0., 10., 0., 2.)


def test_flat_front_wall_keeps_the_front_wall_inner_face():
    edge, metadata = _measure(_carriage())
    assert metadata['valid'], metadata
    assert edge == pytest.approx(10. - STEP, abs=1e-6)
    assert metadata['longitudinalStripCount'] == 0


def test_top_corner_gussets_move_the_front_edge_to_the_width_mean():
    # 两前角 0.4m 宽、伸向厢内 0.4m 的顶部角板：等效开口前沿内移约 0.4×0.8/2.25。
    gussets = np.vstack([_grid((-1.175, -0.775), (9.6, 10.), (1.6, 1.95)),
                         _grid((0.775, 1.175), (9.6, 10.), (1.6, 1.95))])
    edge, metadata = _measure(np.vstack((_carriage(), gussets)))
    assert metadata['valid'], metadata
    strips = [value for value in metadata['stripEdgesL'] if value is not None]
    assert min(strips) == pytest.approx(9.6, abs=STEP)
    assert 10. - .2 < edge < 10. - .1
    assert edge == pytest.approx(np.mean(strips))


def test_structures_below_the_body_band_do_not_define_the_opening():
    # 主体高度带内的前壁内衬不属于顶部开口。
    liner = _grid((-1.1, 1.1), (9.5, 10.), (.2, 1.0))
    edge, metadata = _measure(np.vstack((_carriage(), liner)))
    assert edge == pytest.approx(10. - STEP, abs=1e-6), metadata


def test_front_top_canopy_at_rim_level_does_not_define_the_opening():
    # 前顶篷布/顶板在侧壁上沿处及以上：属于前后顶部附加部分，不定义开口前沿。
    canopy = _grid((-0.75, 0.75), (9.7, 10.), (1.99, 2.1))
    edge, metadata = _measure(np.vstack((_carriage(), canopy)))
    assert edge == pytest.approx(10. - STEP, abs=1e-6), metadata


def test_rim_lip_running_along_the_carriage_is_not_a_front_structure():
    # 侧壁上沿内折 15cm、沿全长贯通：这些条整条排除，不把前沿拉到车厢中部。
    lips = np.vstack([_grid((-1.175, -1.025), (0., 10.), (1.9, 2.)),
                      _grid((1.025, 1.175), (0., 10.), (1.9, 2.))])
    edge, metadata = _measure(np.vstack((_carriage(), lips)))
    assert metadata['valid'], metadata
    assert metadata['longitudinalStripCount'] >= 2
    assert edge == pytest.approx(10. - STEP, abs=1e-6)


def test_detached_structure_behind_a_gap_is_not_counted():
    # 与前壁之间有大于半个面格空档的顶部横杆不连到前壁。
    bar = _grid((-1.1, 1.1), (9.6, 9.7), (1.8, 2.))
    edge, _ = _measure(np.vstack((_carriage(), bar)))
    assert edge == pytest.approx(10. - STEP, abs=1e-6)


def test_unobserved_front_wall_top_leaves_the_edge_unavailable():
    edge, metadata = _measure(_carriage(front_top=False))
    assert edge is None
    assert metadata['reason'] == 'front_wall_top_unobserved'


def test_missing_side_inner_faces_leave_the_edge_unavailable():
    points = _carriage()
    points = points[points[:, 0] < 1.1]
    edge, metadata = _measure(points)
    assert edge is None
    assert metadata['reason'] == 'side_inner_faces_unavailable'
