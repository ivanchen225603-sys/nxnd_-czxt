"""车头磅台残片粘连主车辆簇的回归，不依赖现场 PCD。"""

import numpy as np
import pytest
from scipy.spatial import cKDTree

from carpointcloud_analyzer.domain import preprocessing as pp


def _plane(x_range, y_range, z, step=0.25):
    x, y = np.meshgrid(
        np.arange(*x_range, step), np.arange(*y_range, step), indexing="ij"
    )
    return np.column_stack([x.ravel(), y.ravel(), np.full(x.size, z)])


def _sort_rows(points):
    return points[np.lexsort(points.T[::-1])]


def test_residual_ground_is_found_below_a_dominant_vehicle_side():
    """大侧壁使旧平面循环提前退出，低位广域面仍能被补充识别。"""
    ground = _plane((-10, 10), (-6, 6), 0.7)
    side = _plane((-6, 6), (1.5, 4), 0.0, step=0.05)[:, [0, 2, 1]]
    apron = _plane((-15, 15), (-10, 10), 0.0, step=0.1)
    points = np.vstack([apron, side, ground])
    original = points.copy()
    datum = np.array([0.0, 0.0, 1.0, 0.0])

    result = pp._remove_ground(points, pp.PreprocessParams(), datum)

    assert result.n_removed == len(apron) + len(ground)
    np.testing.assert_array_equal(_sort_rows(result.remaining), _sort_rows(side))
    np.testing.assert_array_equal(_sort_rows(result.removed), _sort_rows(np.vstack([apron, ground])))
    np.testing.assert_array_equal(points, original)
    np.testing.assert_array_equal(datum, [0, 0, 1, 0])


def test_no_initial_ground_removal_preserves_the_existing_cluster_route(monkeypatch):
    """未剔过首层地面时不扩大提车范围，保留原车型的聚类/底板恢复路径。"""
    ground = _plane((-10, 10), (-6, 6), 0.7)
    side = _plane((-6, 6), (1.5, 4), 0.0, step=0.05)[:, [0, 2, 1]]
    points = np.vstack([side, ground])

    def unexpected(*args):
        pytest.fail("没有首层地面剔除证据，不应新增重新分簇的触发条件")

    monkeypatch.setattr(pp, "_residual_ground_mask", unexpected)
    result = pp._remove_ground(points, pp.PreprocessParams(), np.array([0, 0, 1, 0]))
    assert result.n_removed == 0
    np.testing.assert_array_equal(result.remaining, points)


def test_low_support_exit_does_not_trigger_residual_pass(monkeypatch):
    ground = _plane((-10, 10), (-6, 6), 0.0)
    side = _plane((-6, 6), (1.5, 4), 0.0, step=0.05)[:, [0, 2, 1]]
    points = np.vstack([ground, side])
    calls = iter([
        (np.array([0, 0, 1, 0]), np.arange(len(ground))),
        (np.array([1, 0, 0, 0]), np.arange(3)),
    ])
    monkeypatch.setattr(pp, "deterministic_plane_hypothesis", lambda *args: next(calls))

    def unexpected(*args):
        pytest.fail("低支持率退出不应触发残留面复检")

    monkeypatch.setattr(pp, "_residual_ground_mask", unexpected)
    result = pp._remove_ground(points, pp.PreprocessParams(), np.array([0, 0, 1, 0]))
    assert result.n_removed == len(ground)
    np.testing.assert_array_equal(result.remaining, side)


@pytest.mark.parametrize("angle", [0.0, np.pi / 4])
def test_residual_pass_keeps_a_long_narrow_low_floor(angle):
    """长窄底板斜放时不能用固定 X/Y 双跨度冒充广域地面。"""
    floor = _plane((-12, 12), (-1.25, 1.25), 0.7, step=0.1)
    rotation = np.array([[np.cos(angle), -np.sin(angle)],
                         [np.sin(angle), np.cos(angle)]])
    floor[:, :2] = floor[:, :2] @ rotation
    if angle:
        assert np.ptp(floor[:, :2], axis=0).min() > 6.0
    mask = pp._residual_ground_mask(floor, pp.PreprocessParams(), np.array([0, 0, 1, 0]))
    assert not mask.any()


@pytest.mark.parametrize("datum", [None, np.array([0, 0, 1, 0])])
def test_no_datum_or_high_vehicle_surface_is_not_removed(datum):
    points = _plane((-10, 10), (-6, 6), 0.7 if datum is None else 1.5)
    assert not pp._residual_ground_mask(points, pp.PreprocessParams(), datum).any()


def test_removal_cannot_expand_outside_the_original_low_height_candidates():
    ground = _plane((-10, 10), (-6, 6), 0.97)
    # 距拟合面只有 8cm，但高于既有 1m 地面候选上限，必须保留。
    above = _plane((-1, 1), (-1, 1), 1.05)
    points = np.vstack([ground, above])
    mask = pp._residual_ground_mask(points, pp.PreprocessParams(), np.array([0, 0, 1, 0]))
    assert mask[:len(ground)].all()
    assert not mask[len(ground):].any()


def test_deep_returns_below_ground_are_not_a_second_ground_datum():
    points = _plane((-10, 10), (-6, 6), -2.0)
    assert not pp._residual_ground_mask(
        points, pp.PreprocessParams(), np.array([0, 0, 1, 0])
    ).any()


@pytest.mark.parametrize("failure", ["exception", "vertical_after_refinement"])
def test_failed_or_nonhorizontal_final_fit_keeps_all_points(monkeypatch, failure):
    points = _plane((-10, 10), (-6, 6), 0.7)
    if failure == "exception":
        def fail(*args, **kwargs):
            raise ValueError("degenerate candidate")
        monkeypatch.setattr(pp, "deterministic_plane_hypothesis", fail)
    else:
        monkeypatch.setattr(pp, "consolidate_plane", lambda *args: (
            np.array([1, 0, 0, 0]), np.ones(len(points), dtype=bool)
        ))
    assert not pp._residual_ground_mask(
        points, pp.PreprocessParams(), np.array([0, 0, 1, 0])
    ).any()


def test_residual_removal_is_order_independent_and_translation_covariant():
    ground = _plane((-10, 10), (-6, 6), 0.7)
    body = _plane((-6, 6), (-1.25, 1.25), 2.0)
    points = np.vstack([ground, body])
    params = pp.PreprocessParams()
    first = pp._residual_ground_mask(points, params, np.array([0, 0, 1, 0]))
    order = np.random.default_rng(42).permutation(len(points))
    shift = np.array([20, -30, 7])
    moved = points[order] + shift
    again = pp._residual_ground_mask(moved, params, np.array([0, 0, 1, -7]))
    np.testing.assert_array_equal(again, first[order])
    assert not pp._residual_ground_mask(
        points[~first], params, np.array([0, 0, 1, 0])
    ).any()


def test_residual_ground_remains_available_to_low_floor_restoration():
    ground = _plane((-10, 10), (-6, 6), 0.0, step=0.1)
    walls = np.vstack([
        _plane((-5, 5), (0, 1.8), side, step=0.1)[:, [0, 2, 1]]
        for side in (-1.2, 1.2)
    ])
    params = pp.PreprocessParams()
    mask = pp._residual_ground_mask(ground, params, np.array([0, 0, 1, 0]))
    vehicles = [walls]
    restored = pp._restore_carriage_floor(vehicles, ground[mask], params)
    assert restored > 0
    assert len(vehicles[0]) == len(walls) + restored


def test_ground_bridge_is_cut_while_cab_and_outboard_mirrors_survive(monkeypatch):
    """镜片/细支撑明显外伸、驾驶室较宽；不能按厢体宽度裁掉它们。"""
    carriage = np.vstack([
        _plane((-6, 2), (-1.1, 1.1), 2.0, step=0.1),
        *[_plane((-6, 2), (2.0, 3.5), side, step=0.03)[:, [0, 2, 1]]
          for side in (-1.1, 1.1)],
    ])
    cab_roof = _plane((3, 5), (-1.4, 1.4), 2.8, step=0.07)
    cab_front = _plane((-1.4, 1.4), (1.2, 2.8), 5.0, step=0.07)[:, [2, 0, 1]]
    cab_back = _plane((-1.4, 1.4), (2.0, 2.8), 3.0, step=0.07)[:, [2, 0, 1]]
    chassis = _plane((1.9, 3.1), (-1.0, 1.0), 2.0, step=0.1)
    mirrors = np.vstack([
        _plane((4.3, 4.6), (1.45, 2.1), 2.7, step=0.07),
        _plane((4.3, 4.6), (-2.1, -1.45), 2.7, step=0.07),
    ])
    # 全场地面通过接近车头下沿的点带，把细柱连到车辆附近。
    ground = _plane((-10, 10), (-6, 6), 0.9, step=0.25)
    apron = _plane((-15, 15), (-10, 10), 0.0, step=0.1)
    pole = _plane((5.5, 5.75), (0.9, 2.6), 2.0, step=0.06)[:, [0, 2, 1]]
    scene = np.vstack([
        carriage, chassis, cab_roof, cab_back, cab_front, mirrors, pole, ground, apron
    ])
    params = pp.PreprocessParams(scene_edge_filter=False)
    # 独立地坪位于 z=0，本例磅台 z=.9；不从车辆猜高度基准。
    monkeypatch.setattr(pp, "estimate_ground_plane", lambda *args: np.array([0, 0, 1, 0]))
    # 保持非地面点，单独检验地面+聚类+找回，不让 SOR 掩盖误删。
    import open3d as o3d
    monkeypatch.setattr(pp, "_remove_outliers", lambda points, params: (
        o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)), points, np.empty((0, 3))
    ))
    residual_counts = []
    residual = pp._residual_ground_mask

    def record_residual(*args):
        mask = residual(*args)
        residual_counts.append(int(mask.sum()))
        return mask

    monkeypatch.setattr(pp, "_residual_ground_mask", record_residual)
    result = pp.filter_scene(scene, params)
    # 2026-09-24 起立面让位后主循环即剔掉这层磅台，残面复检仍作为兜底运行；
    # 地面是否剔净由下方 stats["ground"] 断言，不再绑定由哪一步剔除。
    assert residual_counts
    tree = cKDTree(result.primary_vehicle_points)
    for genuine in (carriage, cab_roof, cab_front, mirrors):
        assert (tree.query(genuine)[0] < 1e-7).mean() > 0.99
    assert (tree.query(pole)[0] < 1e-7).mean() < 0.1
    assert result.stats["ground"] >= len(ground) + len(apron)
    assert result.stats["n_removed"] == len(scene) - len(result.vehicle_points)


def test_adjacent_cluster_shape_guard_rejects_pole_but_keeps_cab():
    body = _plane((-6, 2), (-1.25, 1.25), 2.0, step=0.1)
    cab = _plane((3, 5), (-1.2, 1.2), 2.7, step=0.1)
    pole = _plane((3, 3.3), (0.5, 3.0), 1.65, step=0.03)[:, [0, 2, 1]]
    work = np.vstack([body, cab, pole])
    labels = np.repeat(np.arange(3), [len(body), len(cab), len(pole)])
    masks = [labels == 0]
    pp._attach_adjacent_clusters(work, labels, masks, [1, 2], pp.PreprocessParams())
    assert masks[0][labels == 1].all()
    assert not masks[0][labels == 2].any()
