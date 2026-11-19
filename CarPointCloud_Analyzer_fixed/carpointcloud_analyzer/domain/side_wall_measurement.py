"""最终车厢云同一纵向位置两侧主体上沿的间距。

接受实测与补片组成的局部 W/L/H 点。先检查局部壁面的二维支持，再按同 L/H
窗口确认主体侧壁；本模块不自行镜像，不从拉筋排除带扣出所谓壁厚。
宽度只量对应纵向小格的两侧上沿，各窗口等权取中位数，不混入下部内衬净空。
"""

from dataclasses import dataclass
from typing import Any, Optional

import numpy as np

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.side_wall_body import side_body_mask
from carpointcloud_analyzer.domain.utils.cells import unique_cells


LONGITUDINAL_WINDOW_M = 1.0
HEIGHT_WINDOW_M = 0.6
FACE_CELL_M = 0.15
FACE_TOLERANCE_M = 0.025
FACE_SEARCH_BAND_M = 0.09
FACE_MIN_POINTS = 24
FACE_MIN_CELLS = 8
FACE_MAX_SLOPE = 0.25
FACE_MIN_CONTRAST = 2.5
MIN_LONGITUDINAL_COVERAGE = 0.30
MIN_PAIRED_WINDOWS = 6
# 仅当双端实测壁面和部分侧壁已支持矩形补全时启用；不能作为普通长车全局门槛。
SPARSE_RECTANGLE_MIN_PAIRED_WINDOWS = 4
SPARSE_RECTANGLE_MAX_WIDTH_SPREAD_M = 0.015
SPARSE_RECTANGLE_MIN_LONGITUDINAL_SPAN_M = 0.8
MAX_LONGITUDINAL_WINDOWS = 80
MAX_HEIGHT_WINDOWS = 12


@dataclass(frozen=True)
class InnerWidthMeasurement:
    """可空净宽及窗口覆盖、来源、缺测原因；单位为米。"""

    width_m: Optional[float]
    metadata: dict[str, Any]


@dataclass(frozen=True)
class _SideFace:
    coefficients: np.ndarray
    cells: np.ndarray
    point_count: int
    residual_p90: float


def _cell_medians(points: np.ndarray, ids: np.ndarray) -> np.ndarray:
    """每个纵向/高度小格等权，消除近端密集点的额外权重。"""
    counts = np.bincount(ids)
    ends = np.cumsum(counts)
    valid = counts >= 3
    counts, ends = counts[valid], ends[valid]
    mid = ends - 1 - (counts - 1) // 2
    even = counts % 2 == 0
    cells = np.empty((len(mid), 3))
    for axis in range(3):
        values = points[np.lexsort((points[:, axis], ids)), axis]
        cells[:, axis] = values[mid]
        cells[even, axis] = (values[mid[even] - 1] + values[mid[even]]) / 2
    return cells


def _supported_side_faces(points: np.ndarray, center: np.ndarray) -> list[_SideFace]:
    """提取局部近纵向竖直面，拒绝斜筋、窄梁和没有表面峰的体点。"""
    if len(points) < FACE_MIN_POINTS:
        return []
    width = points[:, 0]
    # 固定格宽只用于寻找面的位置，不直接把格边界当成物理壁面。
    step = FACE_TOLERANCE_M / 2
    edges = np.arange(np.min(width) - step, np.max(width) + 2 * step, step)
    counts, _ = np.histogram(width, edges)
    # 精确/量化平面可能只占两格；same 在短数组时会返回卷积核长度，越过 edges。
    density = np.convolve(np.pad(counts, (1, 1)), np.ones(3), mode="valid")
    order = np.argsort(-density, kind="stable")
    candidates: list[float] = []
    for index in order:
        mode = float((edges[index] + edges[index + 1]) / 2)
        if all(abs(mode - old) > 2 * FACE_TOLERANCE_M for old in candidates):
            candidates.append(mode)
        if len(candidates) == 4:
            break

    design = np.column_stack((points[:, 1:3] - center, np.ones(len(points))))
    origin = center - [LONGITUDINAL_WINDOW_M / 2, HEIGHT_WINDOW_M / 2]
    grid = np.floor((points[:, 1:3] - origin) / FACE_CELL_M).astype(int)
    _, cell_ids = unique_cells(grid, return_inverse=True)
    results = []
    for mode in candidates:
        mask = np.abs(width - mode) <= FACE_SEARCH_BAND_M
        if np.count_nonzero(mask) < FACE_MIN_POINTS:
            continue
        coefficients = np.array([0.0, 0.0, mode])
        cells = np.empty((0, 3))
        # 与原来一样最多修订五次；若第五次仍改变掩码，第六次只拟合最终内点。
        for iteration in range(6):
            cells = _cell_medians(points[mask], cell_ids[mask])
            if len(cells) < FACE_MIN_CELLS:
                break
            cell_design = np.column_stack((cells[:, 1:3] - center, np.ones(len(cells))))
            coefficients = np.linalg.lstsq(cell_design, cells[:, 0], rcond=None)[0]
            if iteration == 5:
                break
            residual = np.abs(design @ coefficients - width)
            revised = residual <= FACE_TOLERANCE_M
            if np.array_equal(mask, revised):
                break
            mask = revised
        if len(cells) < FACE_MIN_CELLS or np.count_nonzero(mask) < FACE_MIN_POINTS:
            continue
        if np.linalg.norm(coefficients[:2]) > FACE_MAX_SLOPE:
            continue
        span = np.ptp(cells[:, 1:3], axis=0)
        if span[0] < 0.45 or span[1] < 0.25:
            continue
        grid = np.floor((cells[:, [1, 2]] - origin) / FACE_CELL_M).astype(int)
        if any(len(np.unique(grid[:, axis])) < 3 for axis in range(2)):
            continue
        # L/H 包围框有跨度不等于有面：一条斜线也能跨越两轴，需独立第二方向支持。
        normalized = (cells[:, 1:3] - np.mean(cells[:, 1:3], axis=0)) / [
            LONGITUDINAL_WINDOW_M, HEIGHT_WINDOW_M,
        ]
        singular = np.linalg.svd(normalized, compute_uv=False)
        if singular[1] < singular[0] * 0.15:
            continue
        # 不能只在窗口一端有点，然后把平面外推到未观察的中心处。
        if np.any(center < np.min(cells[:, 1:3], axis=0)) or np.any(
            center > np.max(cells[:, 1:3], axis=0)
        ):
            continue
        residual = np.abs(design @ coefficients - width)
        near = np.count_nonzero(residual <= FACE_TOLERANCE_M)
        flank = np.count_nonzero(
            (residual > FACE_TOLERANCE_M) & (residual <= 3 * FACE_TOLERANCE_M)
        )
        if 2 * near < FACE_MIN_CONTRAST * max(flank, 1):
            continue
        cell_error = np.abs(cell_design @ coefficients - cells[:, 0])
        error = float(np.percentile(cell_error, 90))
        if error > FACE_TOLERANCE_M:
            continue
        if any(abs(coefficients[-1] - face.coefficients[-1]) < FACE_TOLERANCE_M
               for face in results):
            continue
        results.append(_SideFace(coefficients, cells, int(near), error))
    return results


def measure_inner_width(
    points: np.ndarray, rear_l: float, front_l: float, floor_h: float, top_h: float,
    *, allow_sparse_rectangular: bool = False, config: Optional[Config] = None,
) -> InnerWidthMeasurement:
    """自动量取同纵向/高度窗口中的双壁净宽，并返回采用范围和缺测原因。

    ``points`` 必须是未补点、已分割车厢的 W/L/H 实测点。``floor_h`` 与
    ``top_h`` 仅限定主体侧壁搜索范围，不能把其差值当成已确认内高。
    测量失败时 ``width_m=None``，调用方可明确标识兼容外沿回退，不能将
    外沿或算法内部搜索区重命名成净内宽。
    """
    metadata: dict[str, Any] = {
        "method": "paired_observed_side_faces",
        "definition": "median_paired_side_rim_spacing",
    "source": "measurement_input_carriage_cloud",
        "coordinateFrame": "vehicle_local_WLH",
        "pairedWindowCount": 0,
        "reason": "insufficient_supported_side_faces",
        "valid": False,
    }

    def unavailable(reason: str) -> InnerWidthMeasurement:
        return InnerWidthMeasurement(None, {**metadata, "reason": reason})

    values = np.asarray(points, dtype=float)
    if values.ndim != 2 or values.shape[1:] != (3,) or len(values) < 2 * FACE_MIN_POINTS:
        return unavailable("insufficient_points")
    if not np.all(np.isfinite(values)) or not np.all(
        np.isfinite([rear_l, front_l, floor_h, top_h])
    ):
        return unavailable("non_finite_input")
    length, depth = front_l - rear_l, top_h - floor_h
    if length <= 0 or depth <= 0:
        return unavailable("invalid_carriage_bounds")
    end_margin = min(LONGITUDINAL_WINDOW_M / 2, length * 0.05)
    h_margin = max(FACE_CELL_M, depth * 0.12)
    l_lo, l_hi = rear_l + end_margin, front_l - end_margin
    h_lo, h_hi = floor_h + h_margin, top_h - h_margin
    if (l_hi - l_lo < 2 * LONGITUDINAL_WINDOW_M
            or h_hi - h_lo < HEIGHT_WINDOW_M + FACE_CELL_M):
        return unavailable("insufficient_body_span")

    n_l = min(MAX_LONGITUDINAL_WINDOWS,
              max(3, int(np.ceil((l_hi - l_lo) / LONGITUDINAL_WINDOW_M))))
    n_h = min(MAX_HEIGHT_WINDOWS, max(2, int(np.ceil((h_hi - h_lo) / HEIGHT_WINDOW_M))))
    l_centers = np.linspace(l_lo + LONGITUDINAL_WINDOW_M / 2,
                            l_hi - LONGITUDINAL_WINDOW_M / 2, n_l)
    h_centers = np.linspace(h_lo + HEIGHT_WINDOW_M / 2, h_hi - HEIGHT_WINDOW_M / 2, n_h)
    body = values[(values[:, 1] >= l_lo) & (values[:, 1] <= l_hi)
                  & (values[:, 2] >= h_lo) & (values[:, 2] <= h_hi)]
    if len(body) < 2 * FACE_MIN_POINTS:
        return unavailable("insufficient_body_points")
    w_lo, w_hi = np.percentile(body[:, 0], [1, 99])
    center_w = (w_lo + w_hi) / 2
    if w_hi - w_lo <= 4 * FACE_SEARCH_BAND_M:
        return unavailable("insufficient_lateral_separation")
    # 稳健外沿只限定候选搜索，不是净宽；极少数远处飞点不能撑大直方图内存。
    body = body[(body[:, 0] >= w_lo - FACE_SEARCH_BAND_M)
                & (body[:, 0] <= w_hi + FACE_SEARCH_BAND_M)]
    # 两侧搜索范围留出横向中央一半，避免把底板/拉筋中部当侧壁；此带不是壁厚。
    center_margin = (w_hi - w_lo) / 4
    windows: list[dict[str, Any]] = []
    for i, l_center in enumerate(l_centers):
        slab = body[np.abs(body[:, 1] - l_center) <= LONGITUDINAL_WINDOW_M / 2]
        for j, h_center in enumerate(h_centers):
            patch = slab[np.abs(slab[:, 2] - h_center) <= HEIGHT_WINDOW_M / 2]
            center = np.array([l_center, h_center])
            left_points = patch[patch[:, 0] < center_w - center_margin]
            right_points = patch[patch[:, 0] > center_w + center_margin]
            left = _supported_side_faces(left_points, center)
            right = _supported_side_faces(right_points, center)
            if not left or not right:
                continue
            selected = []
            for faces, sign in ((left, 1), (right, -1)):
                support = max(len(face.cells) for face in faces)
                eligible = [face for face in faces if len(face.cells) >= support * 0.6]
                selected.append(max(eligible, key=lambda face: sign * face.coefficients[-1]))
            a, b = selected
            # 必须有共同二维小格，不能用左右各自不相交的残片外推“同截面”宽度。
            origin = center - [LONGITUDINAL_WINDOW_M / 2, HEIGHT_WINDOW_M / 2]
            grids = [np.floor((face.cells[:, 1:3] - origin) / FACE_CELL_M).astype(int)
                     for face in (a, b)]
            shared = set(map(tuple, grids[0])) & set(map(tuple, grids[1]))
            if (len(shared) < 6 or any(len({key[axis] for key in shared}) < 2
                                      for axis in range(2))):
                continue
            yaw = (a.coefficients[0] + b.coefficients[0]) / 2
            width_m = float((b.coefficients[-1] - a.coefficients[-1])
                            / np.sqrt(1 + yaw * yaw))
            if not np.isfinite(width_m) or width_m <= 0:
                continue
            windows.append({
                "longitudinalIndex": i, "heightIndex": j,
                "lM": round(float(l_center), 6), "hM": round(float(h_center), 6),
                "widthM": width_m,
                "leftCoefficients": a.coefficients, "rightCoefficients": b.coefficients,
                "leftSupportPoints": a.point_count, "rightSupportPoints": b.point_count,
                "leftCells": len(a.cells), "rightCells": len(b.cells),
                "sharedCells": len(shared),
                "residualP90M": max(a.residual_p90, b.residual_p90),
            })

    observed_l = sorted({window["longitudinalIndex"] for window in windows})
    observed_h = {window["heightIndex"] for window in windows}
    coverage = len(observed_l) / n_l
    metadata.update({
        "pairedWindowCount": len(windows), "candidateWindowCount": n_l * n_h,
        "longitudinalCoverage": round(coverage, 4),
        "observedHeightBandCount": len(observed_h),
        "searchLongitudinalRangeM": [float(l_lo), float(l_hi)],
        "searchHeightRangeM": [float(h_lo), float(h_hi)],
    })
    widths = [window["widthM"] for window in windows]
    dense_supported = (len(windows) >= MIN_PAIRED_WINDOWS and len(observed_l) >= 3
                       and len(observed_h) >= 2 and coverage >= MIN_LONGITUDINAL_COVERAGE)
    sparse_rectangle = (
        allow_sparse_rectangular and not dense_supported
        and len(windows) >= SPARSE_RECTANGLE_MIN_PAIRED_WINDOWS
        and len(observed_l) >= 2 and len(observed_h) >= 2
        and max(row["lM"] for row in windows) - min(row["lM"] for row in windows)
        >= SPARSE_RECTANGLE_MIN_LONGITUDINAL_SPAN_M
        and max(widths) - min(widths) <= SPARSE_RECTANGLE_MAX_WIDTH_SPREAD_M
    )
    if not dense_supported and not sparse_rectangle:
        if len(windows) < MIN_PAIRED_WINDOWS or len(observed_l) < 3 or len(observed_h) < 2:
            return unavailable("insufficient_paired_windows")
        if coverage < MIN_LONGITUDINAL_COVERAGE:
            return unavailable("insufficient_longitudinal_coverage")
    # 二维侧壁支持仍按原窗口/门槛核验；正式宽度只量同一纵向位置的两侧上沿。
    # 侧壁归属与内高共用，防止独立篷杆或端部附加件冒充上沿。
    cfg = config if config is not None else Config()
    edges = np.linspace(l_lo, l_hi, n_l + 1)
    side_masks = [side_body_mask(values, mask, edges, cfg.statistical_outlier_neighbors,
                                 inner_direction=direction)
                  for mask, direction in ((values[:, 0] < center_w - center_margin, 1),
                                          (values[:, 0] > center_w + center_margin, -1))]
    rims = []
    for i in observed_l:
        support = max((row for row in windows if row['longitudinalIndex'] == i),
                      key=lambda row: row['hM'])
        center_l = l_centers[i]
        pair = []
        for mask in side_masks:
            wall = values[mask & (np.abs(values[:, 1] - center_l) <= LONGITUDINAL_WINDOW_M / 2)]
            if not len(wall):
                break
            # 每个纵向小格分别找上沿，倾斜上沿不能只剩窗口较高的一端。
            ids = np.floor((wall[:, 1] - l_lo) / FACE_CELL_M).astype(int)
            upper = {}
            for key in np.unique(ids):
                local = wall[ids == key]
                rim_h = np.percentile(local[:, 2], 98.)
                rim = local[np.abs(local[:, 2] - rim_h) <= FACE_TOLERANCE_M]
                if len(rim):
                    upper[int(key)] = rim
            pair.append(upper)
        if len(pair) != 2:
            continue
        common = sorted(pair[0].keys() & pair[1].keys())
        if not common:
            continue
        a, b = support['leftCoefficients'], support['rightCoefficients']
        yaw = (a[0] + b[0]) / 2
        gaps, heights = [], []
        for key in common:
            left, right = pair[0][key], pair[1][key]
            # 两层平行侧壁同时可见时，上沿仍属于已选主体面，不能把内外两层
            # 平均成一条不存在的边。外折上沿整体偏离主体面时保留其实际位置。
            selected_rims = []
            for rim, coefficients in ((left, a), (right, b)):
                design = np.c_[rim[:, 1] - center_l, rim[:, 2] - support['hM'], np.ones(len(rim))]
                distance = np.abs(rim[:, 0] - design @ coefficients)
                selected_rims.append(rim[distance <= np.min(distance) + FACE_TOLERANCE_M])
            left, right = selected_rims
            # 将两侧同格实测上沿投影到同一L位置；不斜连不同纵向位置的最高点。
            location = l_lo + (key + .5) * FACE_CELL_M
            w_left = np.median(left[:, 0] - a[0] * (left[:, 1] - location))
            w_right = np.median(right[:, 0] - b[0] * (right[:, 1] - location))
            gaps.append(float((w_right - w_left) / np.sqrt(1 + yaw * yaw)))
            heights.append([float(np.median(left[:, 2])), float(np.median(right[:, 2]))])
        rims.append({'lM': float(center_l), 'widthM': float(np.median(gaps)),
                     'leftRimH': float(np.median(heights, axis=0)[0]),
                     'rightRimH': float(np.median(heights, axis=0)[1]),
                     'pairedRimCellCount': len(common)})
    if not rims:
        return unavailable('paired_side_rims_unavailable')
    widths = [row['widthM'] for row in rims]
    width_m = float(np.median(widths))
    narrowest = min(rims, key=lambda row: row['widthM'])
    metadata.update({
        "valid": True, "reason": "ok", "widthM": width_m,
        "method": ("sparse_rectangular_paired_faces" if sparse_rectangle
                   else "paired_observed_side_faces"),
        "sparseRectangularInference": sparse_rectangle,
        "widthRangeP10P90M": np.percentile(widths, [10, 90]).tolist(),
        "narrowestWindow": narrowest,
        "rimWindows": rims,
        "observedLongitudinalRangeM": [min(row["lM"] for row in windows),
                                      max(row["lM"] for row in windows)],
        "observedHeightRangeM": [min(row["hM"] for row in windows),
                                max(row["hM"] for row in windows)],
        "windowSizeM": [LONGITUDINAL_WINDOW_M, HEIGHT_WINDOW_M],
        "maximumFaceResidualP90M": max(row["residualP90M"] for row in windows),
        "fullHeightMinimumClearance": False,
        "note": "主体侧壁同位置两侧上沿间距的中位数，不是下部最小净空。",
    })
    return InnerWidthMeasurement(width_m, metadata)
