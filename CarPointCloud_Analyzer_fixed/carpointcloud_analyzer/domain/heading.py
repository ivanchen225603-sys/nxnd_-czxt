"""用车厢端壁以外的实测车体支持校正普通车型头尾，不使用车型标准尺寸。"""

import numpy as np

from carpointcloud_analyzer.domain.inner_faces import FACE_CELL_M, FACE_MIN_POINTS, FACE_SEARCH_HALF_BAND_M
from carpointcloud_analyzer.domain.models import CarriageSplitResult
from carpointcloud_analyzer.domain.runtime import logger
from carpointcloud_analyzer.domain.utils.cells import unique_cells


# 几何支持门槛，不是车头长度先验：排除单壁、薄附件、飞点和窄立柱。
_MIN_POINTS_PER_CELL = 4
_MIN_LONGITUDINAL_RUN = 4
_MIN_WIDTH_M = 0.8
_MIN_HEIGHT_M = 0.4
_MIN_CARRIAGE_WIDTH_RATIO = 0.5
_MIN_SECONDARY_SPAN_M = 2 * FACE_CELL_M


def cab_requires_heading_reversal(points: np.ndarray, split: CarriageSplitResult) -> bool:
    """仅当负 L 端有独立车体、正 L 端没有时，否决初始“窄端为头”方向。

    两端都支持、都缺测或内壁未验证时不强行选边，保留初始方向。调用方必须
    同时翻转 W/L 坐标和刚体变换，并重新分割，再执行任何前后相关的测量。
    本函数只读取去噪后的实测点，不能传入补点来证明车头存在。
    """
    if not split.boundaries_are_inner_faces:
        return False
    support = cab_end_support(points, split)
    reverse = support[0] and not support[1]
    if reverse:
        logger.info("负L端存在独立实测车体、正L端仅壁面/附件：统一反转头尾坐标")
    return reverse


def cab_end_support(points: np.ndarray, split: CarriageSplitResult) -> tuple[bool, bool]:
    """返回两端搜索带外的独立车体支持；双端/无端支持均不能确认车头。"""
    if len(split.carriage_points) < FACE_MIN_POINTS:
        return False, False
    w_lo, w_hi = np.percentile(split.carriage_points[:, 0], [5, 95])
    width = float(w_hi - w_lo)
    if width <= 0 or split.split_y <= split.carriage_start_y:
        return False, False
    return tuple(
        _has_external_body(points, wall, sign, float(w_lo), width)
        for wall, sign in ((split.carriage_start_y, -1), (split.split_y, 1))
    )


def longitudinally_continuous(points: np.ndarray, rear: float, front: float) -> bool:
    """两端壁之间的实测车体须为一段连续纵向占用，不能跨过车头与车厢间的空隙。

    沿用独立车体判定的 FACE_CELL_M 三维格与每格最少点数：中间出现整格空段，
    说明两道端壁分属两个车体（如挂车前壁与牵引车车头），不是同一车厢。
    """
    inside = points[(points[:, 1] >= rear) & (points[:, 1] <= front)]
    count = max(1, int(np.ceil((front - rear) / FACE_CELL_M)))
    if len(inside) < FACE_MIN_POINTS:
        return False
    cells = np.column_stack((
        np.floor((inside[:, 0] - np.min(inside[:, 0])) / FACE_CELL_M),
        np.clip(np.floor((inside[:, 1] - rear) / FACE_CELL_M), 0, count - 1),
        np.floor((inside[:, 2] - np.min(inside[:, 2])) / FACE_CELL_M),
    )).astype(np.int64)
    cells, counts = unique_cells(cells, return_counts=True)
    occupied = np.unique(cells[counts >= _MIN_POINTS_PER_CELL][:, 1])
    # 拟合端面会比最后一列实测点多出毫米级距离；末格只有这段余量，
    # 不是车体中间的断裂。只检查端面之间的完整分箱，内部空格仍不可跳过。
    full_count = int(np.floor((front - rear) / FACE_CELL_M))
    return bool(full_count and np.array_equal(
        occupied[occupied < full_count], np.arange(full_count)))


def _has_external_body(
    points: np.ndarray, wall: float, sign: int, w_lo: float, width: float,
) -> bool:
    """端壁搜索带外须有连续纵向占用及横向/高度覆盖，点数不能代替体量。"""
    distance = sign * (points[:, 1] - wall)
    # 允许车头比厢体宽，但不让远离车辆横向范围的环境点决定头尾。
    mask = ((distance > FACE_SEARCH_HALF_BAND_M)
            & (points[:, 0] >= w_lo - width * 0.25)
            & (points[:, 0] <= w_lo + width * 1.25))
    outside = points[mask]
    if len(outside) < FACE_MIN_POINTS:
        return False
    local = np.column_stack((outside[:, 0] - w_lo, distance[mask],
                             outside[:, 2] - np.min(outside[:, 2])))
    cells, counts = unique_cells(np.floor(local / FACE_CELL_M).astype(np.int64), return_counts=True)
    cells = cells[counts >= _MIN_POINTS_PER_CELL]
    if len(cells) < _MIN_LONGITUDINAL_RUN:
        return False
    occupied_l = np.unique(cells[:, 1])
    runs = np.split(occupied_l, np.flatnonzero(np.diff(occupied_l) > 1) + 1)
    for run in runs:
        if len(run) < _MIN_LONGITUDINAL_RUN:
            continue
        # 每一段必须独立有车体覆盖，不能把远处宽壁与另一段窄梁拼成车头。
        region = cells[(cells[:, 1] >= run[0]) & (cells[:, 1] <= run[-1])] * FACE_CELL_M
        span = np.ptp(region, axis=0)
        if (span[0] < max(_MIN_WIDTH_M, width * _MIN_CARRIAGE_WIDTH_RATIO)
                or span[2] < _MIN_HEIGHT_M):
            continue
        centered = region - np.mean(region, axis=0)
        _, _, axes = np.linalg.svd(centered, full_matrices=False)
        second_span = np.diff(np.percentile(centered @ axes[1], [5, 95]))[0]
        if second_span >= _MIN_SECONDARY_SPAN_M:
            # 对角细杆也能同时跨 W/L/H，但不能提供第二个独立面内跨度。
            return True
    return False
