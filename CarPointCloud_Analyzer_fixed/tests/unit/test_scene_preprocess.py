"""场景前处理两处缺陷的回归用例。

两处缺陷都是在 2026-09-03 的双雷达合并帧上实测暴露的：
* 车头与残留地面、路肩立柱聚成同一个尺寸合格的大簇，主车辆按点数取最大后
  只剩车厢，``cabToFrontWall`` 从卷尺实测的 4.6m 塌成 0.15m；
* 磅台面比周边地坪只高 0.70m、宽仅 4m，被窄跨度判据当"车辆表面"保护下来，
  主车辆簇 L 被拉到 25.9m。

用合成点云复现，不依赖真实 PCD。
"""

import numpy as np
import open3d as o3d

from carpointcloud_analyzer.domain.preprocessing import (
    PreprocessParams,
    _attach_body_run,
    _body_cluster_labels,
    _one_density_cluster,
    _remove_ground,
)


def test_dense_block_and_ambiguous_border_match_open3d():
    """独立大块走快速路径，小块共享一个边界点时保持原分配顺序，另有孤立噪声。"""
    centers = np.indices((7, 7, 7)).reshape(3, -1).T * 0.1
    offsets = np.linspace(0., 0.001, 12)
    dense = (centers[:, None, :] + offsets[None, :, None]).reshape(-1, 3)
    # 两个密集团由各自的核心点连接到中间的非核心点。
    x = np.repeat([8.3, 8.16, 7.84, 7.7, 8.0, 12.0], [15, 1, 1, 15, 1, 1])
    small = np.column_stack((x, np.zeros_like(x), np.zeros_like(x)))
    points = np.concatenate((dense, small))
    radius, min_points = 0.215, 10
    assert _one_density_cluster(dense, radius, min_points)
    assert not _one_density_cluster(small[:-1], radius, min_points)
    expected = np.asarray(o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)).cluster_dbscan(
        eps=radius, min_points=min_points, print_progress=False))
    labels = _body_cluster_labels(points, radius, min_points)
    np.testing.assert_array_equal(labels < 0, expected < 0)
    # 数字编号可随分块顺序改变，逐簇核对每个原始点的归属。
    for label in np.unique(expected[expected >= 0]):
        mask = expected == label
        np.testing.assert_array_equal(labels == labels[np.flatnonzero(mask)[0]], mask)


def _box(x_range, y_range, z_range, step=0.1):
    """生成一个实心长方体点阵。"""
    grids = [np.arange(lo, hi + step / 2, step) for lo, hi in (x_range, y_range, z_range)]
    return np.stack(np.meshgrid(*grids, indexing="ij"), axis=-1).reshape(-1, 3)


def _plane(x_range, y_range, z, step=0.2):
    """生成一层水平面点阵。"""
    xs = np.arange(x_range[0], x_range[1] + step / 2, step)
    ys = np.arange(y_range[0], y_range[1] + step / 2, step)
    xx, yy = np.meshgrid(xs, ys, indexing="ij")
    return np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, float(z))])


def _scene_with_cab():
    """车厢 + 间隙 + 车头 + 车头前方一根细立柱 + 地面。"""
    carriage = _box((0.0, 12.0), (-1.25, 1.25), (1.5, 3.5), step=0.25)
    cab = _box((13.8, 16.2), (-1.2, 1.2), (0.6, 3.4), step=0.1)
    post = _box((16.4, 16.7), (-1.7, -1.4), (0.3, 2.5), step=0.05)
    ground = _plane((-4.0, 24.0), (-6.0, 6.0), 0.0)
    work = np.vstack([carriage, cab, post, ground])
    base = np.zeros(len(work), dtype=bool)
    base[: len(carriage)] = True
    spans = {
        "cab": (len(carriage), len(carriage) + len(cab)),
        "post": (len(carriage) + len(cab), len(carriage) + len(cab) + len(post)),
    }
    return work, base, spans


def test_body_run_attaches_cab_but_not_roadside_post():
    """车头并入主车辆，车头前方的细立柱不并入。

    立柱刻意放在横向走廊之内（y∈[-1.7,-1.4]，走廊为 ±1.75m），
    确保它是被"分箱横向跨度"判据挡住的，而不是被走廊挡住的。
    """
    work, base, spans = _scene_with_cab()
    masks = [base.copy()]

    attached = _attach_body_run(work, masks, PreprocessParams())

    assert attached > 0
    merged = masks[0]
    cab_lo, cab_hi = spans["cab"]
    post_lo, post_hi = spans["post"]
    assert merged[cab_lo:cab_hi].mean() > 0.9, "车头应基本整体并入"
    assert not merged[post_lo:post_hi].any(), "路肩立柱不得并入"
    assert merged[cab_hi + (post_hi - post_lo):].sum() == 0, "地面不得并入"
    assert work[merged][:, 0].max() > 16.0, "并入后主车辆应覆盖到车头前端"


def test_body_run_disabled_keeps_primary_untouched():
    """关闭开关时行为与改动前一致：主车辆掩码原样不动。"""
    work, base, _ = _scene_with_cab()
    masks = [base.copy()]

    attached = _attach_body_run(work, masks, PreprocessParams(body_run_enable=False))

    assert attached == 0
    assert np.array_equal(masks[0], base)


def _scene_multilevel_ground():
    """周边地坪 z=0、磅台面 z=0.7（窄）、车厢底板 z=2.3（窄）。"""
    apron = _plane((-20.0, 20.0), (-8.0, 8.0), 0.0, step=0.2)
    deck = _plane((-10.0, 10.0), (-2.0, 2.0), 0.7, step=0.1)
    floor = _plane((-5.5, 5.5), (-1.25, 1.25), 2.3, step=0.08)
    return np.vstack([apron, deck, floor]), apron, deck, floor


def test_weighbridge_deck_removed_as_ground_when_datum_known():
    """有地面基准时，窄而低的磅台面按地面剔除，只有抬高的车厢底板受保护。"""
    points, apron, deck, floor = _scene_multilevel_ground()
    params = PreprocessParams()

    result = _remove_ground(points, params, ground_datum=np.array([0.0, 0.0, 1.0, 0.0]))

    remaining_z = result.remaining[:, 2]
    assert result.n_removed >= len(apron) + len(deck) * 0.9
    # 底板被保护后原样放回，磅台面和地坪则不应残留
    assert np.count_nonzero(remaining_z > 2.0) >= len(floor) * 0.9
    assert np.count_nonzero((remaining_z > 0.4) & (remaining_z < 1.2)) < len(deck) * 0.1


def test_narrow_deck_is_protected_without_datum():
    """无地面基准时退化为旧判据：窄跨度的磅台面仍被当作车辆表面保护。

    该用例固化降级路径的行为，避免以后误以为基准是可选的。
    """
    points, _, deck, _ = _scene_multilevel_ground()

    result = _remove_ground(points, PreprocessParams(), ground_datum=None)

    remaining_z = result.remaining[:, 2]
    assert np.count_nonzero((remaining_z > 0.4) & (remaining_z < 1.2)) > len(deck) * 0.9


def test_floor_restore_skips_a_chassis_with_open_underside():
    """悬空底盘：车底下的真实地面不得被当成车厢底板补回来。

    回归背景：底板找回的物理前提是"俯视雷达照不到车底下的地面"。该前提只对低
    底板封闭车厢成立；高平板底盘从轮间空隙就能看到车底地面。实测一辆空平板车
    因此被补进 73077 个地面点，底板面落到地面高度上，栏板深算成 2.64m、填充率
    0.626，最终被判成 Loaded 而拿不到任何尺寸。
    """
    from carpointcloud_analyzer import Config
    from carpointcloud_analyzer.domain.preprocessing import (
        ScenePreprocessor, _restore_carriage_floor,
    )

    rng = np.random.default_rng(20260907)

    def slab(n, x0, x1, y0, y1, z0, z1):
        return np.column_stack([
            rng.uniform(x0, x1, n), rng.uniform(y0, y1, n), rng.uniform(z0, z1, n)
        ])

    # 平板底盘：平板面在 1.7m，只有轮子触地
    deck = slab(20000, -5, 5, -1.2, 1.2, 1.65, 1.75)
    wheels = np.vstack([
        slab(1200, cx - 0.4, cx + 0.4, sy - 0.15, sy + 0.15, 0.0, 1.6)
        for cx in (-3.0, 3.0) for sy in (-1.1, 1.1)
    ])
    chassis = [np.vstack([deck, wheels])]
    ground = slab(30000, -6, 6, -1.6, 1.6, -0.01, 0.01)

    params = ScenePreprocessor(Config()).params()
    restored = _restore_carriage_floor(chassis, ground, params)

    assert restored == 0, "悬空底盘不该触发底板找回"
    assert len(chassis[0]) == len(deck) + len(wheels)


def test_floor_restore_still_recovers_a_low_carriage_floor():
    """低底板车厢：底板与地面共面，仍必须找回，否则车厢内部无点。"""
    from carpointcloud_analyzer import Config
    from carpointcloud_analyzer.domain.preprocessing import (
        ScenePreprocessor, _restore_carriage_floor,
    )

    rng = np.random.default_rng(7)

    def slab(n, x0, x1, y0, y1, z0, z1):
        return np.column_stack([
            rng.uniform(x0, x1, n), rng.uniform(y0, y1, n), rng.uniform(z0, z1, n)
        ])

    # 侧壁自底板（z≈0）立起，车辆最低结构就在地面高度上 → 净空≈0
    walls = np.vstack([
        slab(9000, -5, 5, sy - 0.02, sy + 0.02, 0.0, 1.8) for sy in (-1.2, 1.2)
    ])
    carriage = [walls]
    ground = slab(30000, -6, 6, -1.6, 1.6, -0.01, 0.01)

    params = ScenePreprocessor(Config()).params()
    restored = _restore_carriage_floor(carriage, ground, params)

    assert restored > 0, "低底板车厢的底板必须找回"
    assert len(carriage[0]) == len(walls) + restored
