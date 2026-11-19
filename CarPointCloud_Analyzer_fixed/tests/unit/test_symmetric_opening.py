"""点云内缘驱动的对称推算，不硬编码车型人工尺寸，不扩大到全高内壁。"""

from copy import deepcopy
from dataclasses import replace

import numpy as np
import pytest
from scipy.spatial import cKDTree

from carpointcloud_analyzer import Config
from carpointcloud_analyzer import PointCloudAnalyzer
from carpointcloud_analyzer.domain.box_geometry import rotate_wl
from carpointcloud_analyzer.domain.models import BoxCarriageBox, BoxCompletionResult
from carpointcloud_analyzer.domain.geometry import CoordinateTransform
from carpointcloud_analyzer.application.review_measurements import calculate_review_measurements, build_review_dimension_payload
from carpointcloud_analyzer.domain.shell_appearance import build_shell_appearance
from carpointcloud_analyzer.domain.symmetric_opening import infer_symmetric_opening, attach_symmetric_opening


def fixture(end=.27, side=.15, yaw=0., flip=False):
    box = BoxCarriageBox(5.96, 0., (-1.28, 1.28), (1.2, 4.1), 6000, 6000)
    w, h = np.meshgrid(np.linspace(-1.1, 1.1, 50), np.linspace(4.02, 4.09, 10))
    front = np.column_stack([w.ravel(), np.full(w.size, box.front_l-end), h.ravel()])
    length, h = np.meshgrid(np.linspace(.65, 5.3, 100), np.linspace(4., 4.09, 5))
    left = np.column_stack([np.full(length.size, -1.28+side), length.ravel(), h.ravel()])
    cab = np.tile([0., 8., 1.4], (600, 1))
    p = np.vstack([front, left, cab])
    if flip:
        p[:, 1] = box.front_l - p[:, 1]
    p = rotate_wl(p, np.deg2rad(yaw))
    reference = BoxCompletionResult(box=box, points=np.empty((0, 3)), metadata={
        'profileMatched': True, 'yawDeg': yaw, 'observedHeightM': 2.6,
        'heightSource': 'observed_wall_extent', 'walls': {},
        'frontLip': {'completed': False, 'synthesizedPoints': 0}})
    return p, reference


@pytest.mark.parametrize('end,side', [(.27, .15), (.22, .12), (.32, .19)])
@pytest.mark.parametrize('yaw,flip', [(0, False), (4, True)])
def test_dimensions_follow_points_and_not_fixed_numbers(end, side, yaw, flip):
    p, ref = fixture(end, side, yaw, flip)
    before = p.copy()
    result = infer_symmetric_opening(p, ref)
    assert result['applied'], result
    assert result['lengthM'] == pytest.approx(5.96-2*end)
    assert result['widthM'] == pytest.approx(2.56-2*side)
    assert result['syntheticPointsUsed'] == 0
    assert not result['fullHeightInnerWallsObserved']
    np.testing.assert_array_equal(p, before)
    shuffled = p[np.random.default_rng(18).permutation(len(p))]
    assert infer_symmetric_opening(shuffled, ref) == result
    ref = replace(ref, points=np.full((100, 3), 1e6))
    assert infer_symmetric_opening(p, ref) == result


def test_missing_or_unmatched_does_not_invent_measurement():
    p, ref = fixture()
    ref.metadata['profileMatched'] = False
    assert not infer_symmetric_opening(p, ref)['applied']
    ref.metadata['profileMatched'] = True
    assert not infer_symmetric_opening(p[:500], ref)['applied']
    assert not infer_symmetric_opening(np.vstack([p[:500], p[-600:]]), ref)['applied']


@pytest.mark.parametrize('other_inset', [0., .24])
def test_observed_other_side_contradiction_is_not_overridden(other_inset):
    p, ref = fixture()
    other = p[500:1000].copy()
    other[:, 0] = 1.28-other_inset
    result = infer_symmetric_opening(np.vstack([p, other]), ref)
    assert not result['applied']
    assert result['reason'] in ['side_edge_not_consistent', 'side_insets_disagree']


def test_local_corner_cluster_is_not_a_longitudinal_rim():
    p, ref = fixture()
    p[500:1000, 1] = .8
    assert not infer_symmetric_opening(p, ref)['applied']


def test_opposite_end_disagreement_is_not_mirrored():
    p, ref = fixture()
    rear = p[:500].copy()
    rear[:, 1] = .38
    result = infer_symmetric_opening(np.vstack([p, rear]), ref)
    assert not result['applied'] and result['reason'] == 'end_insets_disagree'


def test_completed_rim_is_symmetric_open_and_matches_saved_dimensions():
    p, ref = fixture()
    before = p.copy()
    original_meta = deepcopy(ref.metadata)
    completed = attach_symmetric_opening(p, ref)
    assert ref.metadata == original_meta and len(ref.points) == 0
    ref = completed
    # 单独的开口补点只含顶部连接带；正式测量还要有补全的四面外壁。
    axes = [np.linspace(-1.28, 1.28, 30), np.linspace(0., 5.96, 60),
            np.linspace(1.2, 4.1, 30)]
    from carpointcloud_analyzer.domain.utils.surfaces import axis_aligned_face
    walls = np.vstack([axis_aligned_face(axes, axis, value)
                       for axis, values in ((0, (-1.28, 1.28)), (1, (0., 5.96)))
                       for value in values])
    measured_ref = replace(ref, points=np.vstack((walls, ref.points)))
    result = calculate_review_measurements(p, measured_ref,
                                           CoordinateTransform(np.zeros(3), np.eye(3)),
                                           None, False, Config())
    payload = build_review_dimension_payload(result, Config(), 'test.pcd')
    # 主体侧壁长5.96m；前后顶部内折不再把长度扣成5.42m。
    assert payload['dimensions']['trailerLength'] == 596.
    assert payload['dimensions']['trailerWidth'] == 226.
    assert result['dimensions']['carriage']['height'] == pytest.approx(2.9)
    meta = deepcopy(ref.metadata)
    for budget in [1000, 29000, 550000]:
        appearance = build_shell_appearance(p, ref, max_points=budget)
        q = appearance.shell
        assert len(q) <= budget
        assert len(appearance.triangles) <= 60
        transported = PointCloudAnalyzer()._preview_points(
            appearance.context, appearance.shell, CoordinateTransform(np.zeros(3), np.eye(3)),
            None, ref.metadata, {}, True, appearance.triangles)
        assert len(transported['displayMesh']['coordinates']) == len(appearance.triangles)*3
        tree = cKDTree(q)
        for axis, center in [(0, 0.), (1, 2.98)]:
            mirror = q.copy()
            mirror[:, axis] = 2*center-q[:, axis]
            assert tree.query(mirror)[0].max() < 1e-8
        opening = meta['symmetricOpening']
        i0, i1 = opening['innerWRange']
        j0, j1 = opening['innerLRange']
        assert not ((q[:, 0] > i0+1e-8) & (q[:, 0] < i1-1e-8)
                    & (q[:, 1] > j0+1e-8) & (q[:, 1] < j1-1e-8)).any()
    assert meta == ref.metadata
    np.testing.assert_array_equal(p, before)
