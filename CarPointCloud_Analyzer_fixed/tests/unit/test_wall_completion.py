import numpy as np

from carpointcloud_analyzer.domain.carriage import (
    CarriageProfile,
    complete_empty_carriage_walls,
)
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.models import CarriageSplitResult


def _surface_grid(fixed_axis, fixed_value, axis_a, axis_b, step=0.1):
    aa, bb = np.meshgrid(axis_a, axis_b, indexing="ij")
    points = np.zeros((aa.size, 3), dtype=float)
    points[:, fixed_axis] = fixed_value
    free = [axis for axis in range(3) if axis != fixed_axis]
    points[:, free[0]] = aa.ravel()
    points[:, free[1]] = bb.ravel()
    return points


def _incomplete_empty_trailer():
    lengths = np.arange(0.2, 9.81, 0.1)
    full_height = np.arange(0.1, 2.01, 0.1)
    low_height = np.arange(0.1, 1.21, 0.1)
    left_side = _surface_grid(0, -1.0, lengths, full_height)
    right_side = _surface_grid(0, 1.0, lengths, low_height)

    # 前后壁只观测到中心面左侧，右半壁应由对称补点恢复。
    half_width = np.arange(-1.0, -0.09, 0.1)
    rear_wall = _surface_grid(1, 0.0, half_width, full_height)
    front_wall = _surface_grid(1, 10.0, half_width, full_height)
    floor = _surface_grid(
        2,
        0.0,
        np.arange(-0.8, 0.81, 0.2),
        np.arange(0.2, 9.81, 0.2),
        step=0.2,
    )
    carriage = np.vstack([left_side, right_side, rear_wall, front_wall, floor])
    cab = np.column_stack(
        [
            np.linspace(-0.8, 0.8, 200),
            np.linspace(12.0, 14.0, 200),
            np.linspace(0.4, 2.5, 200),
        ]
    )
    vehicle = np.vstack([carriage, cab])
    split = CarriageSplitResult(
        carriage_points=carriage,
        split_y=10.0,
        carriage_start_y=0.0,
        head_y=14.0,
        tail_y=0.0,
        boundaries_are_inner_faces=True,
        boundary_method="test",
    )
    profile = CarriageProfile(
        floor_z=0.0,
        wall_top_left=2.0,
        wall_top_right=1.2,
        wall_top=2.0,
        wall_depth=2.0,
        wall_top_diff=0.8,
        scan_incomplete_side="right",
        interior_w_range=(-0.85, 0.85),
        interior_mask=np.abs(carriage[:, 0]) < 0.85,
        median_fill_ratio=0.0,
        high_cell_fraction=0.0,
        mid_band_density=0.0,
        n_grid_cells=100,
        sparse_bin_fraction=0.0,
        edge_sparse_length=(0.0, 0.0),
        measurement_confidence="high",
        valid=True,
    )
    return vehicle, carriage, split, profile


def test_complete_empty_carriage_walls_fills_side_front_and_rear_only():
    vehicle, carriage, split, profile = _incomplete_empty_trailer()
    original = vehicle.copy()

    result = complete_empty_carriage_walls(
        vehicle,
        carriage,
        split,
        profile,
        Config(),
    )

    assert result.metadata["completed"] is True
    assert len(result.side_points) > 0
    assert len(result.front_points) > 0
    assert len(result.rear_points) > 0
    assert result.metadata["synthesizedPoints"] == len(result.points)
    assert np.array_equal(vehicle, original), "补点函数不得原地修改实测点"

    assert np.all(result.side_points[:, 0] > 0.85)
    assert np.all(result.side_points[:, 2] > 1.2)
    assert np.all(np.abs(result.front_points[:, 1] - 10.0) <= 0.12)
    assert np.all(np.abs(result.rear_points[:, 1]) <= 0.12)
    assert np.all(result.front_points[:, 2] >= 0.05)
    assert np.all(result.rear_points[:, 2] >= 0.05)
    assert result.points[:, 1].max() <= 10.12, "车头不得被镜像"


def test_long_bridge_is_enabled_only_at_the_requested_corner(monkeypatch):
    from carpointcloud_analyzer.domain import carriage as module

    vehicle, carriage, split, profile = _incomplete_empty_trailer()
    strip = module.complete_side_end_strip
    calls = {}

    def record(source, plane, model, boundary, direction, band, *, opposite_boundary_l):
        side = 'left' if plane[2] < 0 else 'right'
        calls[(side, 'rearWall' if direction == 1 else 'frontWall')] = opposite_boundary_l
        return strip(source, plane, model, boundary, direction, band, opposite_boundary_l=opposite_boundary_l)

    monkeypatch.setattr(module, 'complete_side_end_strip', record)
    complete_empty_carriage_walls(vehicle, carriage, split, profile, Config(),
                                  bridge_long_side_gaps=frozenset({('right', 'frontWall')}))
    assert calls == {('left', 'rearWall'): None, ('left', 'frontWall'): None,
                     ('right', 'rearWall'): None, ('right', 'frontWall'): split.carriage_start_y}


def test_complete_empty_carriage_walls_can_disable_end_wall_fill():
    vehicle, carriage, split, profile = _incomplete_empty_trailer()
    cfg = Config(carriage_complete_end_walls=False)

    result = complete_empty_carriage_walls(
        vehicle,
        carriage,
        split,
        profile,
        cfg,
    )

    assert len(result.side_points) > 0
    assert len(result.front_points) == 0
    assert len(result.rear_points) == 0
    assert result.metadata["frontWall"]["reason"] == "end_wall_completion_disabled"


def test_complete_empty_carriage_walls_rejects_unverified_boundaries():
    vehicle, carriage, split, profile = _incomplete_empty_trailer()
    split.boundaries_are_inner_faces = False

    result = complete_empty_carriage_walls(
        vehicle,
        carriage,
        split,
        profile,
        Config(),
    )

    assert len(result.points) == 0
    assert result.metadata == {
        "completed": False,
        "reason": "quality_gate_not_met",
    }


# ---------------------------------------------------------------------------
# 厢式车 / 集装箱车厢补全
# ---------------------------------------------------------------------------
def _observed_container(rng, *, length=6.0, width=2.44, height=2.59, deck=1.55):
    """模拟实测：两个端面密集，中段只剩一条掠射角细线。

    这正是 0907 那台 20ft 集装箱车的观测形态 —— 端面法向指向雷达所以完整，
    侧面和顶面几乎与光束平行，6.5m 里有 2.5m 只剩一条 0.17m 宽的细线。
    """
    l0, l1 = 0.0, length
    w0, w1 = -width / 2, width / 2
    h0, h1 = deck, deck + height

    def face(n, wr, lr, hr):
        return np.column_stack([
            rng.uniform(*wr, n), rng.uniform(*lr, n), rng.uniform(*hr, n)
        ])

    return np.vstack([
        face(24000, (w0, w1), (l0 - 0.02, l0 + 0.02), (h0, h1)),   # 后端面（密）
        face(12000, (w0, w1), (l1 - 0.02, l1 + 0.02), (h0, h1)),   # 前端面（密）
        face(900, (w1 - 0.02, w1 + 0.02), (l0, l1), (h1 - 0.3, h1)),  # 右上棱细线
    ])


def test_box_completion_reconstructs_the_container_from_its_two_end_walls():
    """只有两个端面 + 一条细线时，仍要还原出正确的箱体尺寸。"""
    from carpointcloud_analyzer.domain.carriage import complete_box_carriage

    rng = np.random.default_rng(20260907)
    observed = _observed_container(rng)

    result = complete_box_carriage(observed, Config())

    assert result.metadata["completed"] is True
    box = result.box
    assert abs(box.length - 6.0) < 0.25, f"箱长 {box.length:.2f}"
    assert abs(box.width - 2.44) < 0.10, f"箱宽 {box.width:.2f}"
    assert abs(box.height - 2.59) < 0.10, f"箱高 {box.height:.2f}"


def test_box_completion_fills_the_unobserved_middle_span():
    """补全后，中段原本无点的侧面必须被合成点覆盖。"""
    from carpointcloud_analyzer.domain.carriage import complete_box_carriage

    rng = np.random.default_rng(5)
    observed = _observed_container(rng)
    result = complete_box_carriage(observed, Config())

    middle = (result.points[:, 1] > 2.0) & (result.points[:, 1] < 4.0)
    assert int(middle.sum()) > 1000, "中段应当被补上"

    # 左侧面（实测完全没有）必须出现合成点
    left_wall = result.points[:, 0] < -1.0
    assert int((middle & left_wall).sum()) > 100


def test_box_completion_does_not_duplicate_observed_points():
    """端面已经扫得很密，那里不应再堆合成点。

    重复堆叠会破坏后续按点密度工作的算法（底板密度峰、拉筋找峰、
    occupancy_extent 门槛）。
    """
    from scipy.spatial import cKDTree

    from carpointcloud_analyzer.domain.carriage import complete_box_carriage

    rng = np.random.default_rng(9)
    observed = _observed_container(rng)
    cfg = Config()
    result = complete_box_carriage(observed, cfg)

    distance, _ = cKDTree(observed).query(
        result.points, distance_upper_bound=cfg.carriage_completion_dedup_radius
    )
    assert not np.any(np.isfinite(distance)), "合成点不得落在实测点的去重半径内"


def test_box_completion_needs_two_separated_end_walls():
    """只有一个端面时不得凭空造箱体。"""
    from carpointcloud_analyzer.domain.carriage import complete_box_carriage

    rng = np.random.default_rng(13)
    single_wall = np.column_stack([
        rng.uniform(-1.2, 1.2, 20000),
        rng.uniform(-0.02, 0.02, 20000),
        rng.uniform(1.55, 4.14, 20000),
    ])

    result = complete_box_carriage(single_wall, Config())

    assert result.metadata["completed"] is False
    assert result.box is None
    assert len(result.points) == 0


def test_box_completion_can_be_disabled():
    from carpointcloud_analyzer.domain.carriage import complete_box_carriage

    cfg = Config()
    cfg.carriage_box_completion_enable = False
    result = complete_box_carriage(
        _observed_container(np.random.default_rng(1)), cfg
    )

    assert result.metadata == {
        "completed": False, "reason": "box_completion_disabled"
    }


def test_box_completion_does_not_lay_a_second_skin_over_an_observed_wall():
    """真墙偏离重建平面时，只补没扫到的那块，不能在真墙旁边再糊一层合成墙。

    重建出的平面是整面的折中位置，真实墙面（波纹板、门锁件、板厚）会系统性地
    偏离它 —— 实测该偏移中位 5.6~13.4cm，远大于三维去重半径 2cm。按球半径去重
    一个点也删不掉，两层皮就并排留在导出点云里，人工复核时看到的是"补的墙和真
    墙对不上"。判定必须落在面自身的二维平面内：这块墙扫到过，就不再合成。
    """
    from scipy.spatial import cKDTree

    from carpointcloud_analyzer.domain.carriage import complete_box_carriage

    rng = np.random.default_rng(20260908)
    observed = _observed_container(rng)

    # 后端面只扫到下半截，且整体外推 9cm —— 真墙偏离重建平面的典型形态
    rear = np.abs(observed[:, 1]) < 0.05
    keep = ~(rear & (observed[:, 2] > 1.55 + 2.59 * 0.6))
    observed, rear = observed[keep], rear[keep]
    observed[rear, 1] -= 0.09

    cfg = Config()
    result = complete_box_carriage(observed, cfg)
    assert result.metadata["completed"] is True

    rear_l = result.box.rear_l
    rear_synthetic = result.points[np.abs(result.points[:, 1] - rear_l) < 0.02]
    rear_observed = observed[rear]          # 只取真正属于后壁的实测点
    assert len(rear_observed) > 500

    # 没扫到的上半截必须补上
    upper = rear_synthetic[rear_synthetic[:, 2] > 1.55 + 2.59 * 0.7]
    assert len(upper) > 200, f"后壁缺失部分应当补上，实得 {len(upper)}"

    # 扫到的下半截不得再补
    distance, _ = cKDTree(rear_observed[:, [0, 2]]).query(
        rear_synthetic[:, [0, 2]],
        distance_upper_bound=cfg.carriage_box_surface_dedup_radius,
    )
    assert not np.any(np.isfinite(distance)), (
        "后壁已经扫到的位置不得再补点，否则与真墙叠成两层皮"
    )
