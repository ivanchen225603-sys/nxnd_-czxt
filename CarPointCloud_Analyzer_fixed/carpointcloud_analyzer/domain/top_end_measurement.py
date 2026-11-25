"""识别当前侧壁端部的扫描盲段，供补壁阶段选择接缝。"""


import numpy as np

from carpointcloud_analyzer.domain.longitudinal_measurement import _side_candidates


TOP_OFFSETS_M = (.10, .08, .12, .06, .14, .16, .18, .20)
TOP_HALF_BAND_M = .05
SIDE_FACE_DISTANCE_M = .035
TOP_BIN_M = .25
TOP_MIN_COVERAGE = .65
TOP_MAX_EMPTY_BINS = 3


def needs_side_end_bridge(
    points: np.ndarray, rear: float, front: float, floor: float, top: float,
) -> bool:
    """判断是否有侧壁需要长桥接。"""
    return bool(side_ends_needing_bridge(points, rear, front, floor, top))


def side_ends_needing_bridge(
    points: np.ndarray, rear: float, front: float, floor: float, top: float,
) -> frozenset[tuple[str, str]]:
    """返回常规补壁后顶部仍有长盲段的（侧壁，端壁）。"""
    values = np.asarray(points, dtype=float)
    if (values.ndim != 2 or values.shape[1:] != (3,) or len(values) < 200
            or not np.isfinite(values).all() or not np.isfinite([rear, front, floor, top]).all()
            or front - rear <= 2.0):
        return frozenset()
    sides = _side_candidates(
        values, rear, front, floor, top,
        min_coverage=.20, min_length_coverage=.50,
    )
    bin_count = max(1, int(np.ceil((front - rear - .60) / TOP_BIN_M)))
    interior = values[(values[:, 1] >= rear + .30) & (values[:, 1] <= front - .30)]
    selected = set()
    for _, _, sign, face in sides:
        sa, sb, sc = face.coefficients
        distance = np.abs(interior[:, 0] - (sa * interior[:, 1] + sb * interior[:, 2] + sc))
        side = 'left' if sign < 0 else 'right'
        selected.update((side, end) for end in _long_end_gaps(
            interior[distance <= SIDE_FACE_DISTANCE_M], rear, floor, top, bin_count))
    return frozenset(selected)


def _long_end_gaps(side: np.ndarray, rear: float, floor: float, top: float, bin_count: int) -> set[str]:
    gaps, reached = set(), set()
    for offset in TOP_OFFSETS_M:
        height = top - offset
        if height <= floor + .10:
            continue
        side_top = side[np.abs(side[:, 2] - height) <= TOP_HALF_BAND_M]
        if len(side_top) < 100:
            continue
        bins = np.unique(np.floor((side_top[:, 1] - rear - .30) / TOP_BIN_M).astype(int))
        bins = bins[(bins >= 0) & (bins < bin_count)]
        if not len(bins):
            continue
        covered = len(bins) / bin_count >= TOP_MIN_COVERAGE
        for end, gap in (('rearWall', bins[0]), ('frontWall', bin_count - 1 - bins[-1])):
            if gap > TOP_MAX_EMPTY_BINS:
                gaps.add(end)
            elif covered:
                reached.add(end)
        # 每个端点各自检查，能到达前后端的两条测线可以来自不同高度。
        if len(reached) == 2:
            break
    return gaps - reached
