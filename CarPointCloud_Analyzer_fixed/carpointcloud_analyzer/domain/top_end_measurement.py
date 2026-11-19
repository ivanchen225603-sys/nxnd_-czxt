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
    """判断最终云左侧顶部是否仍有端点扫描盲段，供补壁阶段选择长接缝。"""
    values = np.asarray(points, dtype=float)
    if (values.ndim != 2 or values.shape[1:] != (3,) or len(values) < 200
            or not np.isfinite(values).all() or not np.isfinite([rear, front, floor, top]).all()
            or front - rear <= 2.0):
        return False
    left = next((item for item in _side_candidates(
        values, rear, front, floor, top,
        min_coverage=.20, min_length_coverage=.50,
    ) if item[2] < 0), None)
    if left is None:
        return False
    sa, sb, sc = left[3].coefficients
    distance = np.abs(values[:, 0] - (sa * values[:, 1] + sb * values[:, 2] + sc))
    bin_count = max(1, int(np.ceil((front - rear - .60) / TOP_BIN_M)))
    has_long_end_gap = False
    for offset in TOP_OFFSETS_M:
        height = top - offset
        if height <= floor + .10:
            continue
        side_top = values[(distance <= SIDE_FACE_DISTANCE_M)
                          & (np.abs(values[:, 2] - height) <= TOP_HALF_BAND_M)
                          & (values[:, 1] >= rear + .30)
                          & (values[:, 1] <= front - .30)]
        if len(side_top) < 100:
            continue
        bins = np.unique(np.floor((side_top[:, 1] - rear - .30) / TOP_BIN_M).astype(int))
        bins = bins[(bins >= 0) & (bins < bin_count)]
        if not len(bins):
            continue
        end_gap_bins = max(int(bins[0]), bin_count - 1 - int(bins[-1]))
        has_long_end_gap |= end_gap_bins > TOP_MAX_EMPTY_BINS
        if (len(bins) / bin_count >= TOP_MIN_COVERAGE
                and end_gap_bins <= TOP_MAX_EMPTY_BINS):
            # 两端均已接到顶部测线的车辆不改变原补点结果。
            return False
    return has_long_end_gap
