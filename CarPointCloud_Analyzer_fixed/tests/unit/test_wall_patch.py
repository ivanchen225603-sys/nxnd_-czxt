"""端壁补洞只接受本次实测二维面，不能靠补点创造实测支撑。"""

from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial import ConvexHull

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.carriage import _tail_anchored_rear_plane
from carpointcloud_analyzer.domain.inner_faces import ObservedFace
from carpointcloud_analyzer.domain.models import CarriageSplitResult
from carpointcloud_analyzer.domain.wall_patch import (
    _choose_face, complete_end_patch, complete_side_end_strip,
)


def _profile():
    return SimpleNamespace(floor_z=0.0, wall_top=1.5, interior_w_range=(-1.0, 1.0))


def test_rear_face_prior_requires_confirmed_wall_near_vehicle_tail():
    split = CarriageSplitResult(
        np.empty((0, 3)), 2.42, -8.42, 7.13, -8.47,
        boundaries_are_inner_faces=True,
        rear_wall_plane=(0., .005, -8.42),
    )
    assert _tail_anchored_rear_plane(split) == split.rear_wall_plane
    split.tail_y = -8.90
    assert _tail_anchored_rear_plane(split) is None
    split.tail_y = -8.47
    split.boundaries_are_inner_faces = False
    assert _tail_anchored_rear_plane(split) is None


def _wall(w_bounds=(-1.2, 1.2), h_bounds=(0.1, 1.5), slope=0.0, offset=3.0):
    w, h = np.meshgrid(np.linspace(*w_bounds, 61), np.linspace(*h_bounds, 47))
    return np.column_stack((w.ravel(), offset + slope * w.ravel(), h.ravel()))


def _inside(points, polygon):
    hull = ConvexHull(polygon)
    return np.all(points @ hull.equations[:, :2].T + hull.equations[:, 2] <= 1e-8,
                  axis=1)


def test_complete_observed_plane_preserves_input_and_coefficients():
    observed = _wall()
    before = observed.copy()
    # 留一个内部空洞；补洞不能依赖原有点恰好占据相同位置。
    observed = observed[~((np.abs(observed[:, 0]) < 0.2)
                          & (np.abs(observed[:, 2] - 0.7) < 0.2))]
    snapshot = observed.copy()
    points, meta = complete_end_patch(observed, 3.0, 0.0, _profile(), Config())
    assert meta["valid"] and meta["source"] == "current_observed_end_wall"
    assert len(points) > 0 and meta["modelGeneratedPoints"] == len(points)
    assert np.allclose(meta["coefficients"], [0, 0, 3], atol=1e-9)
    assert np.any((np.abs(points[:, 0]) < 0.1) & (np.abs(points[:, 2] - 0.7) < 0.1))
    assert np.array_equal(observed, snapshot)
    assert len(observed) < len(before)
    assert np.allclose(points[:, 1], 3.0)
    assert not meta["heightExtrapolated"]


def test_half_side_only_mirrors_observed_height_support():
    observed = _wall(w_bounds=(0.2, 1.2), h_bounds=(0.2, 1.4))
    points, meta = complete_end_patch(observed, 3, 0, _profile(), Config())
    assert meta["valid"] and meta["symmetryApplied"]
    assert np.min(points[:, 0]) < -1.1
    assert np.min(points[:, 2]) >= 0.2 and np.max(points[:, 2]) <= 1.4
    assert not np.any(np.abs(points[:, 0]) < 0.19)
    assert len(meta["supportPolygonsWH"]) == 2
    wh = points[:, [0, 2]]
    assert np.all(np.logical_or.reduce([_inside(wh, polygon)
                                      for polygon in meta["supportPolygonsWH"]]))


def test_missing_low_edge_never_extrapolates_measurement_height():
    points, meta = complete_end_patch(_wall(h_bounds=(0.5, 1.5)), 3, 0, _profile(), Config())
    assert meta["valid"] and len(points)
    assert np.min(points[:, 2]) >= 0.5
    assert not any(_inside(np.array([[1.0, 0.25]]), polygon)[0]
                   for polygon in meta["supportPolygonsWH"])


def test_supported_side_end_seam_closes_small_corner_gap_without_changing_input():
    observed = _wall(w_bounds=(-1.11, 1.0))
    original = observed.copy()
    side = {"left": np.array([0., 0., -1.135])}
    points, meta = complete_end_patch(observed, 3., 0., _profile(), Config(),
                                      side_planes=side)
    assert np.array_equal(observed, original)
    assert meta["valid"] and meta["sideEndSeams"]
    assert max(item["maxGapM"] for item in meta["sideEndSeams"]) <= .03
    assert np.any((np.abs(points[:, 0] + 1.136) < 1e-6)
                  & (np.abs(points[:, 2] - .8) < .02))
    assert any(_inside(np.array([[-1.135, .8]]), polygon)[0]
               for polygon in meta["supportPolygonsWH"])


def test_side_end_seam_does_not_join_distant_or_unsupported_wall():
    observed = _wall(w_bounds=(-1.11, 1.0))
    points, meta = complete_end_patch(
        observed, 3., 0., _profile(), Config(),
        side_planes={"left": np.array([0., 0., -1.2])},
    )
    assert meta["valid"] and not meta.get("sideEndSeams")
    assert not np.any(np.abs(points[:, 0] + 1.2) < .02)
    assert not any(_inside(np.array([[-1.2, .8]]), polygon)[0]
                   for polygon in meta["supportPolygonsWH"])


def test_side_end_strip_joins_only_nearby_supported_end_to_side():
    _, model = complete_end_patch(_wall(), 3., 0., _profile(), Config())
    lengths, heights = np.meshgrid(np.linspace(3.27, 4., 30),
                                   np.linspace(.2, 1.4, 30))
    side = np.column_stack((np.full(lengths.size, -1.), lengths.ravel(),
                            heights.ravel()))
    strip, info = complete_side_end_strip(
        side, np.array([0., 0., -1.]), model, 3., 1, .12,
    )
    assert info["completed"] and len(strip)
    assert np.any(np.abs(strip[:, 1] - 3.) < 1e-6)
    assert np.any(np.abs(strip[:, 1] - 3.27) < .03)
    assert np.all(np.abs(strip[:, 0] + 1.) < 1e-9)
    assert np.max(strip[:, 1]) <= 3.35

    distant = side.copy()
    distant[:, 1] += .4
    rejected, reason = complete_side_end_strip(
        distant, np.array([0., 0., -1.]), model, 3., 1, .12,
    )
    assert not len(rejected) and reason["reason"] == "side_end_gap_unsupported"


def test_two_confirmed_ends_bridge_one_meter_side_scan_gap():
    _, model = complete_end_patch(_wall(), 3., 0., _profile(), Config())
    lengths, heights = np.meshgrid(np.linspace(4.35, 5., 42),
                                   np.linspace(.2, 1.4, 30))
    side = np.column_stack((np.full(lengths.size, -1.), lengths.ravel(),
                            heights.ravel()))
    strip, info = complete_side_end_strip(
        side, np.array([0., 0., -1.]), model, 3., 1, .12,
        opposite_boundary_l=13.,
    )
    assert info["completed"] and info["longGapHeightRows"] > 0
    assert 1.3 <= info["maximumGapM"] < 2.
    assert np.any(np.abs(strip[:, 1] - 3.) < 1e-6)
    assert strip[:, 1].max() < 5.

    # 只有孤立带或没有另一端壁边界时不能跨过大盲段。
    isolated = side[side[:, 1] < 4.42]
    refused, _ = complete_side_end_strip(
        isolated, np.array([0., 0., -1.]), model, 3., 1, .12,
        opposite_boundary_l=13.,
    )
    assert not len(refused)


def test_transverse_sloped_plane_is_not_mirrored_or_flattened():
    observed = _wall(w_bounds=(0.2, 1.2), slope=0.07)
    points, meta = complete_end_patch(observed, 3.0, 0.0, _profile(), Config())
    assert meta["valid"] and not meta["symmetryApplied"]
    assert meta["symmetryReason"] == "transversely_sloped_observed_face"
    assert np.min(points[:, 0]) >= 0.2
    assert np.allclose(points[:, 1], 3.0 + 0.07 * points[:, 0])


def test_volume_scatter_is_not_a_wall():
    rng = np.random.default_rng(129)
    observed = rng.uniform([-1.2, 2.88, 0.1], [1.2, 3.12, 1.5], size=(5000, 3))
    points, meta = complete_end_patch(observed, 3, 0, _profile(), Config())
    assert not meta["valid"] and not len(points)


@pytest.mark.parametrize("separation", [0.04, 0.10])
def test_parallel_supported_faces_are_ambiguous(separation):
    observed = np.vstack((_wall(offset=2.96), _wall(offset=2.96 + separation)))
    points, meta = complete_end_patch(observed, 3, 0, _profile(), Config())
    assert not meta["valid"] and not len(points)
    assert meta["reason"] == "ambiguous_end_faces"


def test_weak_near_coincident_face_does_not_veto_dominant_end_wall(monkeypatch):
    """弱的局部拟合和主端壁中心重合时，允许主面补洞并留下选面标记。"""
    w, h = np.meshgrid(np.linspace(-1, 1, 19), np.linspace(0.1, 1.5, 6))
    cells = np.column_stack((w.ravel(), np.full(w.size, 3.0), h.ravel()))
    main = ObservedFace(1, np.array([0., 0., 3.]), cells)
    weak = ObservedFace(1, np.array([0.03, 0., 3.003]), cells[:44])
    monkeypatch.setattr("carpointcloud_analyzer.domain.wall_patch.supported_faces",
                        lambda *args, **kwargs: [main, weak])
    chosen, reason = _choose_face(cells, 3.0, 0.2)
    assert chosen is main
    assert reason == "dominant_near_coincident_face"


def test_comparable_competing_face_remains_ambiguous(monkeypatch):
    w, h = np.meshgrid(np.linspace(-1, 1, 19), np.linspace(0.1, 1.5, 6))
    cells = np.column_stack((w.ravel(), np.full(w.size, 3.0), h.ravel()))
    main = ObservedFace(1, np.array([0., 0., 3.]), cells)
    competing = ObservedFace(1, np.array([0.03, 0., 3.003]), cells[:100])
    monkeypatch.setattr("carpointcloud_analyzer.domain.wall_patch.supported_faces",
                        lambda *args, **kwargs: [main, competing])
    chosen, reason = _choose_face(cells, 3.0, 0.2)
    assert chosen is None
    assert reason == "ambiguous_end_faces"


def test_grid_allocation_obeys_resource_limit(monkeypatch):
    monkeypatch.setattr("carpointcloud_analyzer.domain.wall_patch.MAX_END_PATCH_POINTS", 64)
    cfg = Config(carriage_box_surface_spacing=1e-9)
    points, meta = complete_end_patch(_wall(), 3, 0, _profile(), cfg)
    assert meta["valid"] and 0 < len(points) <= 64
    assert meta["maximumModelPoints"] == 64
    assert meta["surfaceSpacingM"] >= 0.01


@pytest.mark.parametrize("points", [np.empty((0, 3)), np.ones((100, 2)),
                                    np.full((100, 3), np.nan)])
def test_invalid_input_returns_finite_empty_metadata(points):
    completed, meta = complete_end_patch(points, 3, 0, _profile(), Config())
    assert not len(completed) and not meta["valid"]
    assert meta["coefficients"] is None
