"""厢体长方体的几何原语：偏航对齐、端面定位、六面归属与覆盖判定。

从 ``carriage`` 拆出。这些函数原本是 ``carriage`` 的模块私有实现，却被
``wall_reference``、``symmetric_opening``、``shell_appearance``、
``review_measurements`` 跨模块导入——私有符号成了事实上的公共 API，改动
``carriage`` 内部会静默破坏下游。独立成模块后依赖方向变为单向：
``config``/``models`` -> ``box_geometry`` -> ``carriage`` 及各补全模块。

坐标约定与 ``carriage`` 一致：axis 0 = W 车宽，axis 1 = L 车长，axis 2 = H 高度。
"""

from typing import Optional, Tuple

import numpy as np
from scipy.spatial import cKDTree

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.models import BoxCarriageBox
from carpointcloud_analyzer.domain.utils import occupancy_extent


# 六个面的编号与法向轴：0/1 左右(W)，2/3 后前(L)，4/5 底顶(H)。
BOX_FACE_AXES = (0, 0, 1, 1, 2, 2)

def rotate_wl(points: np.ndarray, yaw: float) -> np.ndarray:
    """在 W–L 平面内绕 H 轴旋转 ``yaw`` 弧度。"""
    cos_a, sin_a = float(np.cos(yaw)), float(np.sin(yaw))
    rotated = points.copy()
    rotated[:, 0] = points[:, 0] * cos_a - points[:, 1] * sin_a
    rotated[:, 1] = points[:, 0] * sin_a + points[:, 1] * cos_a
    return rotated

def find_end_wall_centers(
    points: np.ndarray, config: Config
) -> Optional[Tuple[float, float]]:
    """沿 L 找两个最强的横向密集面中心；两者必须相隔足够远。

    第二个端面必须与第一个相隔至少 ``carriage_box_min_length``，否则同一面的
    相邻两个 bin 会被当成两个端面，箱长塌成一个 bin 宽。
    """
    length = points[:, 1]
    band = config.carriage_box_end_wall_band
    lo, hi = float(length.min()), float(length.max())
    if hi - lo < band * 3:
        return None
    edges = np.arange(lo, hi + band, band)
    hist, _ = np.histogram(length, bins=edges)
    centers = (edges[:-1] + edges[1:]) / 2.0
    if len(hist) < 2:
        return None

    order = np.argsort(hist)[::-1]
    first = int(order[0])
    for index in order[1:]:
        if abs(centers[index] - centers[first]) >= config.carriage_box_min_length:
            coarse_a, coarse_b = float(centers[first]), float(centers[int(index)])
            rear_coarse, front_coarse = sorted((coarse_a, coarse_b))
            rear = _refine_wall_position(
                length, rear_coarse, band, "min",
                config.carriage_box_wall_face_fraction,
            )
            front = _refine_wall_position(
                length, front_coarse, band, "max",
                config.carriage_box_wall_face_fraction,
            )
            # 保持与粗定位相同的先后顺序，调用方据此对应端面点数。
            if coarse_a <= coarse_b:
                return rear, front
            return front, rear
    return None

def _refine_wall_position(
    length: np.ndarray, coarse: float, band: float, outer: str,
    face_fraction: float,
) -> float:
    """把端面位置从 bin 中心细化到该端面的**外表面**。

    两个理由：

    * 直接用 bin 中心会让箱长精度受 bin 宽（0.20m）限制，而且依赖分箱相位 ——
      实测同一份点云整体旋转 2° 后分箱起点改变，端面跳了一格，箱长从 −6cm
      恶化到 −26cm。
    * 箱长是两个**外表面**之间的距离，不是两个端面点云重心之间的距离。实测
      取带内中位数得 −13cm、取带内密度峰得 −20cm，都因为把厚度和内部横梁算了
      进去；取外侧极值得 −6cm。

    ``outer`` 为 ``"min"`` 时取靠后一侧的外表面，``"max"`` 时取靠前一侧。
    极值用占用跨度取，避免个别飞点把箱体撑长。
    """
    inside = np.abs(length - coarse) <= band
    if inside.sum() < 20:
        return coarse
    # 门槛按带内点数取比例而非绝对值：门把手、铰链、保险杠这类小凸起点数很少，
    # 绝对门槛（min_points=5）会把它们当成端面而使箱长虚长（实测 +11cm）。
    values = length[inside]
    threshold = max(20, int(round(len(values) * face_fraction)))
    lo, hi = occupancy_extent(values, bin_width=0.02, min_points=threshold)
    return float(lo if outer == "min" else hi)

def box_face_planes(box: BoxCarriageBox) -> Tuple[float, ...]:
    """按 :data:`BOX_FACE_AXES` 的顺序给出六个面在各自法向上的坐标。"""
    return (
        box.w_range[0], box.w_range[1],
        box.rear_l, box.front_l,
        box.h_range[0], box.h_range[1],
    )

def mask_points_away_from(
    candidates: np.ndarray, observed: np.ndarray, radius: float
) -> np.ndarray:
    """标出离任何实测点都超过 ``radius`` 的候选点。"""
    if len(observed) == 0 or radius <= 0:
        return np.ones(len(candidates), dtype=bool)
    distance, _ = cKDTree(observed).query(candidates, distance_upper_bound=radius)
    return ~np.isfinite(distance)

def assign_box_faces(
    aligned: np.ndarray, box: BoxCarriageBox, band: float
) -> np.ndarray:
    """把每个实测点归给离它最近的那个箱面；超出 ``band`` 的记 ``-1``。

    必须"最近面独占"而不是"每个面各自划一条带"：顶面的点在横向上离侧壁也只有
    十几厘米，若按带划分会同时算进侧壁，让侧壁误以为自己被扫到过。
    """
    planes = box_face_planes(box)
    signed = np.column_stack([
        planes[0] - aligned[:, 0], aligned[:, 0] - planes[1],
        planes[2] - aligned[:, 1], aligned[:, 1] - planes[3],
        planes[4] - aligned[:, 2], aligned[:, 2] - planes[5],
    ])
    owner = np.argmin(np.abs(signed), axis=1)
    within = np.abs(signed[np.arange(len(aligned)), owner]) <= band
    return np.where(within, owner, -1)

def mask_uncovered_on_faces(
    synthetic: np.ndarray,
    face_ids: np.ndarray,
    aligned: np.ndarray,
    owners: np.ndarray,
    radius: float,
) -> np.ndarray:
    """标出所在墙面位置尚未被扫到的合成点。

    判定要在**面自身的二维平面内**做，把法向偏移丢掉。重建出来的平面是整面的
    折中位置，真实墙面（波纹板、门锁件、板厚）会系统性地偏离它 —— 实测该偏移
    中位 5.6~13.4cm，远大于三维去重半径 2cm，因此按球半径去重一个点也删不掉，
    合成面与真墙叠成相距十几厘米的两层皮。改在面内判定，等价于问"这块墙被扫到
    过没有"，扫到过就不再合成。
    """
    keep = np.ones(len(synthetic), dtype=bool)
    if radius <= 0:
        return keep
    for face_id, normal_axis in enumerate(BOX_FACE_AXES):
        target = face_ids == face_id
        source = owners == face_id
        if not target.any() or not source.any():
            continue
        plane_axes = [axis for axis in range(3) if axis != normal_axis]
        tree = cKDTree(aligned[source][:, plane_axes])
        distance, _ = tree.query(
            synthetic[target][:, plane_axes], distance_upper_bound=radius
        )
        keep[target] = ~np.isfinite(distance)
    return keep
