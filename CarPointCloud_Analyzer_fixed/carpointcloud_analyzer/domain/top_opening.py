"""车厢顶部开口前沿：车头距与内长共用的半挂前端（2026-11-07 用户确认口径）。

俯视看车厢开口，前端由前壁顶部及与之相连的前角构件（角板、斜撑、内折沿）决定。
宁A43204M 前壁主体内表面 L≈3.07，而顶部两前角构件伸向厢内到 2.68～2.86，
甲方系统车头距 465 / 内长 1058 对应的前端落在这一区域。

规则：
* 高度带：主体高度带以上（底板 + BODY_TOP_FRACTION × 内深）至侧壁上沿以下一个面容差。
  上沿处及以上的前顶篷布/顶板属于已确认排除的“前后顶部附加部分”（-23063 前端约 30cm 深的
  顶板在上沿 1.59m 处及以上，计入会使内长短 24cm），不定义开口。
* 横向：两侧壁内表面之间、各内缩一个侧壁点带（3 × 面容差），按约 0.2m 等宽分条。
* 顶部带中段沿车长贯通的条（侧壁内折沿等）整条排除，不属于前端结构。
* 每条从前壁开始向厢内追踪连续相连的结构（相邻实测点纵向间隙不超过半个面格），
  取连续结构最靠厢内的位置；该条前壁顶部未观测到时该条不计。
* 有效条数不少于一半时，开口前沿取各条等权平均，即顶部开口面积除以内宽的
  等效长度；不取分位数，避免按单车选统计量。

只用本次实测点（不含补壁/补杆合成点）；前壁顶部无内伸构件时退化为前壁顶部最内实测面。
"""

from typing import Optional

import numpy as np

from carpointcloud_analyzer.domain.inner_faces import FACE_CELL_M, FACE_TOLERANCE_M
from carpointcloud_analyzer.domain.inner_length_measurement import BODY_TOP_FRACTION
from carpointcloud_analyzer.domain.longitudinal_measurement import _side_candidates

# 同一构件内相邻实测点的最大纵向间隙；更大的空档视为与前壁不相连。
OPENING_MAX_GAP_M = FACE_CELL_M / 2
# 侧壁点带宽度，与内长测线取侧壁点的残差带一致。
OPENING_SIDE_MARGIN_M = 3 * FACE_TOLERANCE_M
OPENING_MIN_STRIP_POINTS = 5


def measure_top_opening_front_edge(
    observed_points: np.ndarray, side_points: np.ndarray,
    rear: float, front: float, floor: float, top: float,
) -> tuple[Optional[float], dict]:
    """返回顶部开口前沿的局部 L 及诊断；无法可靠量取时 L 为 None。

    Args:
        observed_points: 本次实测车厢点（W/L/H）。
        side_points: 定侧壁内表面所用的最终车厢云（与内长测线同一份）。
        rear, front: 分割实测的后、前端壁内表面 L。
        floor, top: 最终剖面的底板高与侧壁上沿高。
    """
    metadata = {
        'definition': 'top_opening_front_edge_width_mean',
        'source': 'observed_carriage_points',
        'frontWallInnerFaceL': float(front),
        'maxGapM': OPENING_MAX_GAP_M,
        'sideMarginM': OPENING_SIDE_MARGIN_M,
        'valid': False,
    }

    def unavailable(reason):
        return None, {**metadata, 'reason': reason}

    values = np.asarray(observed_points, dtype=float)
    if (values.ndim != 2 or values.shape[1:] != (3,) or len(values) == 0
            or not np.isfinite([rear, front, floor, top]).all() or top - floor <= .5):
        return unavailable('invalid_input')
    band_low = floor + BODY_TOP_FRACTION * (top - floor)
    sides = _side_candidates(np.asarray(side_points, dtype=float), rear, front, floor, top,
                             min_coverage=.20, min_length_coverage=.50)
    faces = {sign: face for _, _, sign, face in sides}
    if set(faces) != {-1, 1}:
        return unavailable('side_inner_faces_unavailable')
    h_ref = (band_low + top) / 2
    left = faces[-1].at(front, h_ref) + OPENING_SIDE_MARGIN_M
    right = faces[1].at(front, h_ref) - OPENING_SIDE_MARGIN_M
    if right - left < 2 * FACE_CELL_M:
        return unavailable('insufficient_inner_width')
    count = max(1, int(round((right - left) / FACE_CELL_M)))
    bounds = np.linspace(left, right, count + 1)
    band_high = top - FACE_TOLERANCE_M
    zone = values[(values[:, 2] >= band_low) & (values[:, 2] <= band_high)
                  & (values[:, 1] <= front + FACE_TOLERANCE_M)
                  & (values[:, 1] >= rear)]
    # 沿车长贯通的顶部构件（侧壁内折沿等）不属于前端结构：沿用侧壁选面的
    # 中段 0.25m 纵向分箱，顶部带中段覆盖过半的条整条排除，不计入开口宽度。
    n_l = max(3, int(np.ceil((front - rear - 1.) / .25)))
    mid_step = (front - rear - 1.) / n_l
    edges = []
    longitudinal = 0
    for lo, hi in zip(bounds[:-1], bounds[1:]):
        in_strip = zone[(zone[:, 0] >= lo) & (zone[:, 0] < hi)]
        mid = in_strip[(in_strip[:, 1] > rear + .5) & (in_strip[:, 1] < front - .5), 1]
        _, counts = np.unique(np.floor((mid - rear - .5) / mid_step).astype(int),
                                 return_counts=True)
        if np.count_nonzero(counts >= 3) * 2 >= n_l:
            longitudinal += 1
            continue
        strip = np.sort(in_strip[:, 1])[::-1]
        # 该条前壁顶部须有实测，才能从前壁开始追踪相连结构。
        if len(strip) < OPENING_MIN_STRIP_POINTS or front - strip[0] > OPENING_MAX_GAP_M:
            edges.append(None)
            continue
        breaks = np.nonzero(-np.diff(strip) > OPENING_MAX_GAP_M)[0]
        edges.append(float(strip[breaks[0]] if len(breaks) else strip[-1]))
    valid = [edge for edge in edges if edge is not None]
    metadata.update(
        heightBandM=[float(band_low), float(band_high)], innerWidthRangeM=[float(left), float(right)],
        stripWidthM=float((right - left) / count), stripEdgesL=edges,
        validStripCount=len(valid), stripCount=count, longitudinalStripCount=longitudinal,
    )
    if not valid or len(valid) * 2 < count - longitudinal:
        return unavailable('front_wall_top_unobserved')
    edge = float(np.mean(valid))
    metadata.update(valid=True, reason='ok', frontEdgeL=edge,
                    insetFromInnerFaceM=float(front - edge))
    return edge, metadata
