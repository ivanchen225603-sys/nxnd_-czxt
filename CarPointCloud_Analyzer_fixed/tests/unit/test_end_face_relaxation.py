"""端壁歧义的有界放宽：不放开分层墙、相当支持或支撑域外推。"""

from itertools import permutations
from types import SimpleNamespace

import numpy as np
import pytest

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.inner_faces import FACE_TOLERANCE_M, ObservedFace
from carpointcloud_analyzer.domain.wall_patch import (
    _choose_face, _relaxed_dominant_face, complete_end_patch,
)


def _cells():
    w, h = np.meshgrid(np.linspace(-1, 1, 19), np.linspace(.1, 1.5, 6))
    return np.c_[w.ravel(), np.full(w.size, 3.), h.ravel()]


def _face(count, slope, offset):
    cells = _cells()[np.linspace(0, 113, count).astype(int)].copy()
    coefficients = np.array([0., slope, 3. + offset - .8 * slope])
    cells[:, 1] = np.c_[cells[:, [0, 2]], np.ones(count)] @ coefficients
    return ObservedFace(1, coefficients, cells)


def _candidates():
    # 复现 52/114/59 格且弱面更近；数值不是产品中的车型尺寸常量。
    return [_face(52, -.043849, .000084), _face(114, .004293, .002507),
            _face(59, .054750, -.007504)]


@pytest.mark.parametrize('order', list(permutations(range(3))))
def test_nearest_weak_face_does_not_veto_reviewable_dominant_face(monkeypatch, order):
    faces = _candidates()
    monkeypatch.setattr('carpointcloud_analyzer.domain.wall_patch.supported_faces',
                        lambda *args, **kwargs: [faces[i] for i in order])
    audit = {}
    chosen, reason = _choose_face(_cells(), 3., .2, diagnostics=audit)
    assert chosen is faces[1]
    assert reason == 'dominant_near_coincident_relaxed'
    assert audit['candidateCellCounts'] == [114, 59, 52]
    assert audit['centerSpreadM'] == pytest.approx(.010011)
    assert audit['originalReason'] == 'ambiguous_end_faces'
    assert 'requiresManualReview' not in audit
    assert audit['limits']['maxSurfaceSeparationM'] == .10
    assert FACE_TOLERANCE_M == .025


@pytest.mark.parametrize('failure', ['center_spread', 'boundary_offset', 'surface_gap',
                                    'comparable_support', 'competing_pair'])
def test_relaxation_rejects_each_out_of_bounds_case(monkeypatch, failure):
    faces = _candidates()
    if failure == 'center_spread':
        faces[2] = _face(59, .054750, -.02)
    elif failure == 'boundary_offset':
        faces[1] = _face(114, .004293, .011)
        faces[2] = _face(59, .054750, .006)
    elif failure == 'surface_gap':
        faces[2] = _face(59, .13, -.007504)
    elif failure == 'comparable_support':
        faces[2] = _face(80, .054750, -.007504)
    else:
        # 主面和两弱面各相差 <10cm，但两弱面之间 >10cm，不能漏检。
        faces = [_face(52, -.09, 0.), _face(114, 0., .003), _face(59, .09, .001)]
    monkeypatch.setattr('carpointcloud_analyzer.domain.wall_patch.supported_faces',
                        lambda *args, **kwargs: faces)
    audit = {}
    chosen, reason = _choose_face(_cells(), 3., .2, diagnostics=audit)
    assert chosen is None and reason == 'ambiguous_end_faces'
    assert not audit


@pytest.mark.parametrize('threshold,extra,allowed', [
    ('center', 0., True), ('center', .0001, False),
    ('boundary', 0., True), ('boundary', .0001, False),
    ('separation', 0., True), ('separation', .0001, False),
    ('support', 0., True), ('support', 1., False),
])
def test_relaxed_limits_have_explicit_inclusive_boundaries(threshold, extra, allowed):
    main, other = _face(114, 0., 0.), _face(59, .04, .001)
    if threshold == 'center':
        other = _face(59, .04, .015 + extra)
    elif threshold == 'boundary':
        main, other = _face(114, 0., .01 + extra), _face(59, .04, .01 + extra)
    elif threshold == 'separation':
        other = _face(59, (.10 + extra) / .7, 0.)
    else:
        other = _face(76 + int(extra), .04, .001)
    chosen, _ = _relaxed_dominant_face([main, other], np.array([0., .8]), 3.)
    assert (chosen is main) is allowed


def test_relaxed_model_reports_policy_but_stays_on_observed_surface(monkeypatch):
    faces = _candidates()
    monkeypatch.setattr('carpointcloud_analyzer.domain.wall_patch.supported_faces',
                        lambda *args, **kwargs: faces)
    w, h = np.meshgrid(np.linspace(-1., 1., 91), np.linspace(.1, 1.5, 61))
    observed = np.c_[w.ravel(), np.full(w.size, 3.), h.ravel()]
    profile = SimpleNamespace(floor_z=0., wall_top=1.5, interior_w_range=(-1., 1.))
    points, metadata = complete_end_patch(observed, 3., 0., profile, Config())
    assert metadata['valid'] and len(points)
    assert metadata['faceSelection'] == 'dominant_near_coincident_relaxed'
    assert 'requiresManualReview' not in metadata['faceSelectionRelaxation']
    assert metadata['observedCellCount'] == 114
    assert not metadata['heightExtrapolated']
    assert points[:, 2].min() >= .1 and points[:, 2].max() <= 1.5
    assert np.allclose(points[:, 1], np.c_[points[:, [0, 2]], np.ones(len(points))]
                       @ faces[1].coefficients)


def test_original_success_does_not_switch_to_larger_face(monkeypatch):
    near, larger = _face(52, 0., 0.), _face(114, 0., .01)
    monkeypatch.setattr('carpointcloud_analyzer.domain.wall_patch.supported_faces',
                        lambda *args, **kwargs: [larger, near])
    audit = {}
    chosen, reason = _choose_face(_cells(), 3., .2, diagnostics=audit)
    assert chosen is near and reason == 'ok'
    assert not audit


@pytest.mark.parametrize('order', [(0, 1), (1, 0)])
def test_front_vertical_inner_face_is_selected_beside_sloped_outer_cover(monkeypatch, order):
    inner = _face(114, -.01, .005)
    outer = _face(100, .085, .045)
    faces = [inner, outer]
    monkeypatch.setattr('carpointcloud_analyzer.domain.wall_patch.supported_faces',
                        lambda *args, **kwargs: [faces[index] for index in order])
    audit = {}
    chosen, reason = _choose_face(_cells(), 3., .2, diagnostics=audit, interior_sign=-1)
    assert chosen is inner
    assert reason == 'inner_face_beside_sloped_cover'
    assert audit['originalReason'] == 'ambiguous_end_faces'
    assert audit['centerGapM'] == pytest.approx(.04)
    assert audit['innerCellCount'] == 114
    # 没有前后壁身份时，不得按同样两个候选面自行猜内侧。
    assert _choose_face(_cells(), 3., .2)[0] is None


@pytest.mark.parametrize('inner_slope,outer_slope,inner_offset,outer_offset', [
    (.04, .085, .005, .045),
    (-.01, .03, .005, .045),
    (-.01, .085, .035, .075),
    (-.01, .085, .005, .12),
])
def test_double_wall_rule_keeps_other_ambiguities_closed(
    monkeypatch, inner_slope, outer_slope, inner_offset, outer_offset,
):
    faces = [_face(114, inner_slope, inner_offset),
             _face(100, outer_slope, outer_offset)]
    monkeypatch.setattr('carpointcloud_analyzer.domain.wall_patch.supported_faces',
                        lambda *args, **kwargs: faces)
    chosen, reason = _choose_face(_cells(), 3., .2, interior_sign=-1)
    assert chosen is None and reason == 'ambiguous_end_faces'


def _rear_faces():
    """09-15 后端双面形态：主面覆盖更多，外侧弱面在高度方向分叉。"""
    w, h = np.meshgrid(np.linspace(-1.1, 1.2, 17), np.linspace(-.45, 1.46, 11))
    wh = np.column_stack((w.ravel(), h.ravel()))
    primary = np.array([-.0011218998, .0223105687, -8.4213242408])
    secondary = np.array([.0012100439, -.0360867972, -8.4098337886])

    def make_face(coefficients, count):
        selected = wh[np.linspace(0, len(wh) - 1, count).astype(int)]
        length = np.c_[selected, np.ones(len(selected))] @ coefficients
        return ObservedFace(1, coefficients,
                            np.c_[selected[:, 0], length, selected[:, 1]])

    return make_face(primary, 119), make_face(secondary, 77)


@pytest.mark.parametrize('order', [(0, 1), (1, 0)])
def test_rear_inner_face_uses_verified_tail_plane_when_patch_fits_diverge(
    monkeypatch, order,
):
    faces = _rear_faces()
    monkeypatch.setattr('carpointcloud_analyzer.domain.wall_patch.supported_faces',
                        lambda *args, **kwargs: [faces[index] for index in order])
    points = np.vstack([face.cells for face in faces])
    reference = (-.0016676041, .0053184023, -8.4215134588)
    assert _choose_face(points, -8.4203, .12, interior_sign=1)[0] is None
    audit = {}
    chosen, reason = _choose_face(
        points, -8.4203, .12, diagnostics=audit,
        interior_sign=1, verified_inner_plane=reference,
    )
    assert chosen is faces[0]
    assert reason == 'tail_anchored_inner_face'
    assert audit['selectedReferenceP90M'] < FACE_TOLERANCE_M
    assert audit['otherReferenceP90M'] > FACE_TOLERANCE_M
    assert audit['candidateCellCounts'] == [119, 77]


def test_tail_reference_cannot_choose_weak_outer_or_front_face(monkeypatch):
    inner, outer = _rear_faces()
    monkeypatch.setattr('carpointcloud_analyzer.domain.wall_patch.supported_faces',
                        lambda *args, **kwargs: [inner, outer])
    points = np.vstack([inner.cells, outer.cells])
    outer_reference = tuple(float(value) for value in outer.coefficients)
    for sign, reference in ((1, outer_reference), (-1, tuple(inner.coefficients))):
        chosen, reason = _choose_face(
            points, -8.4203, .12, interior_sign=sign,
            verified_inner_plane=reference,
        )
        assert chosen is None and reason == 'ambiguous_end_faces'


def test_tail_reference_does_not_bypass_comparable_supported_layers(monkeypatch):
    inner, outer = _rear_faces()
    comparable = ObservedFace(1, outer.coefficients, _rear_faces()[0].cells.copy())
    comparable.cells[:, 1] = np.c_[comparable.cells[:, [0, 2]],
                                   np.ones(len(comparable.cells))] @ outer.coefficients
    monkeypatch.setattr('carpointcloud_analyzer.domain.wall_patch.supported_faces',
                        lambda *args, **kwargs: [inner, comparable])
    points = np.vstack([inner.cells, comparable.cells])
    chosen, reason = _choose_face(
        points, -8.4203, .12, interior_sign=1,
        verified_inner_plane=(-.0016676041, .0053184023, -8.4215134588),
    )
    assert chosen is None and reason == 'ambiguous_end_faces'
