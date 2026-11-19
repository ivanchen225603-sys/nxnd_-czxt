"""独立实测车辆/车厢提取，功能等价实现；不替换原厂判定链输入。

只输出原始行号。观测终止不是物理边界，缺端板保持 unsupported。
TODO(现场): 用带独立边界真值的实车数据标定采样/结构支持参数。
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Optional

import numpy as np

from .geometry import fit_plane_pca
from .pointcloud import Frame, normalize_roi


@dataclass(frozen=True)
class RoiLimits:
    cell_m: float = .20
    plane_band_m: float = .045
    plane_rms_m: float = .02
    slice_m: float = .25
    min_height_m: float = .30
    max_height_m: float = 4.8
    min_width_m: float = 1.6
    max_width_m: float = 3.2
    min_length_m: float = 3.0
    max_length_m: float = 22.0
    min_vertical_m: float = .40
    top_jump_m: float = .35
    max_yaw_deg: float = 6.0
    min_points: int = 200

    def validate(self) -> None:
        for key, value in asdict(self).items():
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError("ROI 参数须为有限正值: " + key)
        if (self.min_height_m >= self.max_height_m or self.min_width_m >= self.max_width_m
                or self.min_length_m >= self.max_length_m or self.max_yaw_deg > 15
                or type(self.min_points) is not int):
            raise ValueError("ROI 参数范围矛盾或超出小偏航分析边界")


@dataclass
class ObservedRoi:
    vehicle_indices: np.ndarray
    carriage_indices: np.ndarray
    report: dict


def _components(points: np.ndarray, cell: float):
    """所有 XY 连通簇均保留，不按点数选择最大簇。"""
    cells, inverse = np.unique(np.floor(points[:, :2] / cell).astype(np.int64), axis=0, return_inverse=True)
    lookup = {tuple(value): index for index, value in enumerate(cells)}
    labels = np.full(len(cells), -1, np.int64)
    label = 0
    for start in range(len(cells)):
        if labels[start] >= 0:
            continue
        labels[start] = label
        queue = [start]
        while queue:
            current = queue.pop()
            x, y = cells[current]
            for dx, dy in ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)):
                neighbor = lookup.get((x + dx, y + dy))
                if neighbor is not None and labels[neighbor] < 0:
                    labels[neighbor] = label
                    queue.append(neighbor)
        label += 1
    return labels[inverse], label


def _vertical_support(points: np.ndarray, horizontal: int, limits: RoiLimits) -> dict:
    """面须在多个横向片内有连续垂向展布；两侧竖线或水平货面不能冒充端板。"""
    empty = dict(ok=False, points=int(len(points)), span=0., vertical=0., strips=0, coverage=0.)
    if len(points) < 24:
        return empty
    h0, h1 = np.percentile(points[:, horizontal], [2, 98])
    z0, z1 = np.percentile(points[:, 2], [2, 98])
    span, vertical = h1 - h0, z1 - z0
    if span < .5 or vertical < limits.min_vertical_m:
        return {**empty, "span": float(span), "vertical": float(vertical)}
    hi = np.clip(((points[:, horizontal] - h0) / span * 8).astype(int), 0, 7)
    zi = np.clip(((points[:, 2] - z0) / vertical * 6).astype(int), 0, 5)
    occupied = np.zeros((8, 6), bool)
    occupied[hi, zi] = True
    strips = 0
    # 去掉两端横向片，不能把两侧壁拼出的 U 型截面当成端板。
    for column in range(1, 7):
        section = points[hi == column, 2]
        if (len(section) >= 4 and np.ptp(section) >= limits.min_vertical_m
                and int(occupied[column].sum()) >= 4):
            strips += 1
    coverage = float(occupied[1:7, 1:5].mean())
    return dict(ok=bool(strips >= 4 and coverage >= .5), points=int(len(points)),
                span=float(span), vertical=float(vertical), strips=strips, coverage=coverage)


def _rotation(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, s, 0.], [-s, c, 0.], [0., 0., 1.]])


def _side_pair(points: np.ndarray, limits: RoiLimits):
    # 仅分析采样减量，输出仍从完整原数组按索引获取；不影响归档分辨率。
    # 按空间单元排序后再取样，避免仅重排输入行就把样本全部选到底板上。
    cells, first = np.unique(np.floor(points / limits.plane_band_m).astype(np.int64),
                             axis=0, return_index=True)
    sample = points[first[np.linspace(0, len(cells) - 1, min(len(cells), 60000)).astype(int)]]
    step = limits.plane_band_m
    best = None
    for angle in np.linspace(-limits.max_yaw_deg, limits.max_yaw_deg, max(2, int(8 * limits.max_yaw_deg) + 1)):
        matrix = _rotation(math.radians(float(angle)))
        x = sample @ matrix[0]
        cells = np.floor(x / step).astype(int)
        origin = int(cells.min())
        counts = np.bincount(cells - origin)
        smooth = np.convolve(counts, np.ones(3), mode="same")
        candidates = np.argsort(smooth)[-min(len(smooth), 24):]
        for left in candidates:
            for right in candidates:
                width = (right - left) * step
                if not limits.min_width_m <= width <= limits.max_width_m:
                    continue
                score = min(smooth[left], smooth[right])
                if best is None or score > best[0]:
                    best = (score, matrix, (left + origin + .5) * step, (right + origin + .5) * step)
    if best is None:
        return None
    _, matrix, x0, x1 = best
    q = points @ matrix.T
    normals = []
    masks = []
    for coordinate in (x0, x1):
        mask = np.abs(q[:, 0] - coordinate) <= step * 1.8
        if not _vertical_support(q[mask], 1, limits)["ok"]:
            return None
        plane = fit_plane_pca(points[mask])
        if plane is None:
            return None
        # 由较宽搜索带收紧平面，避免把贴近侧壁的水平底板拉偏法向。
        for _ in range(3):
            refined = mask & (plane.abs_distance(points) <= limits.plane_rms_m)
            if refined.sum() < 24:
                return None
            plane = fit_plane_pca(points[refined])
            if plane is None:
                return None
        normal = np.array([plane.a, plane.b, plane.c])
        normal /= np.linalg.norm(normal)
        if normal[0] < 0:
            normal = -normal
        if abs(normal[2]) > .08 or abs(math.degrees(math.atan2(normal[1], normal[0]))) > limits.max_yaw_deg + 1e-6:
            return None
        normals.append(normal)
        masks.append(refined)
    if np.dot(normals[0], normals[1]) < .998:
        return None
    normal = np.mean(normals, axis=0)
    yaw = math.atan2(normal[1], normal[0])
    matrix = _rotation(yaw)
    q = points @ matrix.T
    coordinates = [float(np.median(q[mask, 0])) for mask in masks]
    if not limits.min_width_m <= coordinates[1] - coordinates[0] <= limits.max_width_m:
        return None
    return matrix, q, coordinates


def _wall_runs(q: np.ndarray, sides, limits: RoiLimits):
    wall = [np.abs(q[:, 0] - side) <= limits.plane_band_m for side in sides]
    bins = np.floor(q[:, 1] / limits.slice_m).astype(np.int64)
    common = np.intersect1d(np.unique(bins[wall[0]]), np.unique(bins[wall[1]]))
    slices = []
    for index in common:
        sections = [q[mask & (bins == index)] for mask in wall]
        if any(len(section) < 6 or np.ptp(section[:, 2]) < limits.min_vertical_m for section in sections):
            continue
        top = [float(np.percentile(section[:, 2], 95)) for section in sections]
        if abs(top[0] - top[1]) > limits.top_jump_m:
            continue
        slices.append((index, float(np.mean(top))))
    runs, current = [], []
    for item in slices:
        if current and (item[0] - current[-1][0] > 2 or abs(item[1] - current[-1][1]) > limits.top_jump_m):
            runs.append(current)
            current = []
        current.append(item)
    if current:
        runs.append(current)
    out = []
    for run in runs:
        mask = (wall[0] | wall[1]) & (bins >= run[0][0]) & (bins <= run[-1][0])
        region = q[mask]
        if len(region) and np.ptp(region[:, 1]) >= limits.min_length_m:
            out.append((mask, float(region[:, 1].min()), float(region[:, 1].max())))
    return out


def _boundary(q: np.ndarray, axis: int, coordinate: float, limits: RoiLimits) -> dict:
    mask = np.abs(q[:, axis] - coordinate) <= limits.plane_band_m
    plane = fit_plane_pca(q[mask]) if mask.sum() >= 24 else None
    support = _vertical_support(q[mask], 1 - axis, limits)
    fitted = None
    if plane is not None:
        normal = np.array([plane.a, plane.b, plane.c])
        normal /= np.linalg.norm(normal)
        residual = plane.abs_distance(q[mask])
        rms = float(np.sqrt(np.mean(residual ** 2)))
        support.update(normal=normal.tolist(), rms=rms)
        if support["ok"] and abs(normal[axis]) > .99 and rms <= limits.plane_rms_m:
            fitted = float(np.median(q[mask, axis]))
    return {"status": "supported" if fitted is not None else "unsupported", "value_m": fitted,
            "observed_limit_m": float(coordinate), "evidence": support}


def _structural_planes(q: np.ndarray, axis: int, limits: RoiLimits, min_span: float):
    """枚举竞争结构面；不能仅挑点数最多的两面就声称目标唯一。"""
    bins = np.floor(q[:, axis] / limits.plane_band_m).astype(np.int64)
    values, counts = np.unique(bins, return_counts=True)
    planes = []
    for position in np.argsort(-counts):
        if counts[position] < 24:
            break
        coordinate = (values[position] + .5) * limits.plane_band_m
        if any(abs(coordinate - prior["value_m"]) <= limits.plane_band_m * 2 for prior in planes):
            continue
        boundary = _boundary(q, axis, coordinate, limits)
        if boundary["status"] == "supported" and boundary["evidence"]["span"] >= min_span:
            planes.append(boundary)
    return sorted(planes, key=lambda value: value["value_m"])


def _end_connected(q: np.ndarray, coordinate: float, sides, top: float, limits: RoiLimits) -> bool:
    """端板须与两侧壁相接并达到同一厢顶；低矮货物断面不是物理端板。"""
    section = q[np.abs(q[:, 1] - coordinate) <= limits.plane_band_m]
    if len(section) < 24:
        return False
    margin = limits.plane_band_m * 2
    width = sides[1] - sides[0]
    interior = section[(section[:, 0] > sides[0] + .15 * width)
                       & (section[:, 0] < sides[1] - .15 * width)]
    # 两侧壁在端切片中的密集高点不属于端板到顶的证据。
    if len(interior) < 24:
        return False
    return bool(section[:, 0].min() <= sides[0] + margin
                and section[:, 0].max() >= sides[1] - margin
                and np.percentile(interior[:, 2], 98) >= top - margin)


def extract_observed_roi(frame: Frame, lane_seed, ground_z: float,
                         limits: Optional[RoiLimits] = None, capture_bounds=None) -> ObservedRoi:
    """车道框只选择候选，不用其边界裁断主车。多解不按最大点数猜测。

    capture_bounds 仅在调用者明确知道输入已被裁切时提供，不能拿车道种子冒充。
    该函数一次处理一个冻结观测，不能逐帧回正后送入运动判据。
    """
    limits = limits or RoiLimits()
    limits.validate()
    seed = normalize_roi(lane_seed)
    capture = normalize_roi(capture_bounds)
    if seed is None or not math.isfinite(ground_z):
        raise ValueError("独立 ROI 需要明确车道种子与有限地面高度")
    empty = np.empty(0, np.int64)
    report = {"algorithm": "observed-roi-v1", "parameters": asdict(limits),
              "vehicle_status": "unavailable", "carriage_status": "unavailable",
              "candidate_count": 0, "reason": "没有足够车辆结构观测", "boundaries": {},
              "analysis_transform": {"R": np.eye(3).tolist(), "t": [0., 0., 0.], "source": "source_frame"},
              "frame": {"seq": frame.seq, "ts": frame.ts, "gen": frame.gen},
              "ground_z": float(ground_z), "authoritative": False,
              "boundary_semantics": "geometric_candidate_not_verified_physical"}
    xyz = np.asarray(frame.xyz, np.float64)
    finite = np.isfinite(xyz).all(axis=1)
    selected = np.flatnonzero(finite & (xyz[:, 2] > ground_z + limits.min_height_m)
                             & (xyz[:, 2] < ground_z + limits.max_height_m))
    report["nonfinite_points"] = int((~finite).sum())
    if len(selected) < limits.min_points:
        return ObservedRoi(empty, empty.copy(), report)
    points = xyz[selected]
    labels, count = _components(points, limits.cell_m)
    candidates = []
    for label in range(count):
        index = np.flatnonzero(labels == label)
        if len(index) < limits.min_points:
            continue
        p = points[index]
        lo, hi = p.min(axis=0), p.max(axis=0)
        center = (lo + hi) / 2
        # 局部种子只确认属于本车道；邻车即使回波很多也不按最大簇取胜。
        if not np.all((center[:2] >= seed[0][:2]) & (center[:2] <= seed[1][:2])):
            continue
        if hi[1] - lo[1] < limits.min_length_m or hi[0] - lo[0] < limits.min_width_m:
            continue
        candidates.append(index)
    report["candidate_count"] = len(candidates)
    if len(candidates) != 1:
        if candidates:
            report.update(vehicle_status="ambiguous", carriage_status="ambiguous", reason="多个候选位于本车道种子内")
        return ObservedRoi(empty, empty.copy(), report)
    indices = selected[candidates[0]]
    p = xyz[indices]
    extent = np.ptp(p, axis=0)
    if extent[0] > limits.max_width_m + limits.max_length_m * math.sin(math.radians(limits.max_yaw_deg)) or extent[1] > limits.max_length_m:
        report.update(vehicle_status="ambiguous", carriage_status="ambiguous", reason="候选可能与邻车或固定结构相连")
        return ObservedRoi(empty, empty.copy(), report)
    # 连通只说明采样格相邻，不能证明是同一辆车。先在允许偏航内检查
    # 整个候选能否容纳于单车宽度；保留离散角度误差的上界，不缩窄正常偏停车。
    angles = np.linspace(-limits.max_yaw_deg, limits.max_yaw_deg, max(2, int(8 * limits.max_yaw_deg) + 1))
    min_width = min(float(np.ptp(p @ _rotation(math.radians(float(angle)))[0])) for angle in angles)
    angular_error = float(np.hypot(extent[0], extent[1])) * math.radians(float(angles[1] - angles[0])) / 2
    report["minimum_candidate_width_m"] = min_width
    if min_width > limits.max_width_m + angular_error + limits.plane_band_m * 2:
        report.update(vehicle_status="ambiguous", carriage_status="ambiguous",
                      reason="允许偏航内仍超过单车宽度，连通簇不能作为单车归档")
        return ObservedRoi(empty, empty.copy(), report)
    report.update(vehicle_status="available", observed_vehicle_extent=[p.min(axis=0).tolist(), p.max(axis=0).tolist()])
    pair = _side_pair(p, limits)
    if pair is None:
        report.update(vehicle_status="unavailable", reason="缺少唯一平行侧壁，仅保留全观测，不猜车辆或车厢")
        return ObservedRoi(empty, empty.copy(), report)
    matrix, q, sides = pair
    report["analysis_transform"] = {"R": matrix.tolist(), "t": [0., 0., 0.], "source": "observed_parallel_sidewalls"}
    competing = _structural_planes(q, 0, limits, limits.min_length_m)
    report["side_plane_candidates"] = [plane["value_m"] for plane in competing]
    outside = (q[:, 0] < sides[0] - limits.plane_band_m * 2) | (q[:, 0] > sides[1] + limits.plane_band_m * 2)
    external = _structural_planes(q[outside], 0, limits, .5) if outside.any() else []
    if len(competing) != 2 or external or np.ptp(q[:, 0]) > limits.max_width_m + limits.plane_band_m * 2:
        report.update(vehicle_status="ambiguous", carriage_status="ambiguous",
                      reason="存在竞争侧壁或连通邻车，不能按点数猜测本车外边界")
        return ObservedRoi(empty, empty.copy(), report)
    runs = _wall_runs(q, sides, limits)
    if len(runs) != 1:
        report.update(carriage_status="ambiguous" if runs else "unavailable",
                      reason="侧壁有多个长度合格结构段" if runs else "侧壁连续垂向支持不足")
        return ObservedRoi(indices, empty.copy(), report)
    wall, y0, y1 = runs[0]
    z0 = float(np.percentile(q[wall, 2], 1))
    z1 = float(np.percentile(q[wall, 2], 99))
    between = ((q[:, 0] >= sides[0] - limits.plane_band_m)
               & (q[:, 0] <= sides[1] + limits.plane_band_m)
               & (q[:, 1] >= y0 - limits.plane_band_m) & (q[:, 1] <= y1 + limits.plane_band_m))
    end_planes = [item["value_m"] for item in _structural_planes(q[between], 1, limits,
                                                               .75 * (sides[1] - sides[0]))
                  if _end_connected(q[between], item["value_m"], sides, z1, limits)]
    report["end_plane_candidates"] = end_planes
    interior = [value for value in end_planes if y0 + limits.slice_m < value < y1 - limits.slice_m]
    if interior:
        # 内部端板也可能是车厢隔板。只有端板外侧存在两侧壁共同的真实断口，
        # 才可作为驾驶室与车厢的分隔证据；不能仅凭最长段替代语义判断。
        for value in interior:
            disconnected = []
            for sign in (-1, 1):
                distances = []
                for side in sides:
                    edge = q[(np.abs(q[:, 0] - side) <= limits.plane_band_m)
                             & (q[:, 2] > z0 + limits.min_vertical_m)]
                    dy = (edge[:, 1] - value) * sign
                    dy = dy[dy > limits.plane_band_m * 2]
                    distances.append(float(dy.min()) if len(dy) else 0.)
                disconnected.append(min(distances) >= limits.slice_m)
            if not any(disconnected):
                report.update(carriage_status="ambiguous", reason="内部端板两侧结构连续，无法区分隔板与车厢端板")
                return ObservedRoi(indices, empty.copy(), report)
        divisions = [y0] + interior + [y1]
        segments = [(lo, hi) for lo, hi in zip(divisions, divisions[1:]) if hi - lo >= limits.min_length_m]
        if len(segments) != 1:
            report.update(carriage_status="ambiguous", reason="内部端板形成多个可用厢段，不能按最长段猜车厢")
            return ObservedRoi(indices, empty.copy(), report)
        y0, y1 = segments[0]
        segment_wall = wall & (q[:, 1] >= y0 - limits.plane_band_m) & (q[:, 1] <= y1 + limits.plane_band_m)
        z0 = float(np.percentile(q[segment_wall, 2], 1))
        z1 = float(np.percentile(q[segment_wall, 2], 99))
    band = limits.plane_band_m
    # 侧壁可能仅扫到上半段，不能把侧壁低分位当成厢底而丢掉可见底板。
    # 内部低点只是观测下界，不是独立底板真值；避开端外低底盘连接区。
    interior_points = q[(q[:, 0] > sides[0] + band * 2) & (q[:, 0] < sides[1] - band * 2)
                        & (q[:, 1] > y0 + limits.slice_m) & (q[:, 1] < y1 - limits.slice_m)]
    if len(interior_points):
        z0 = min(z0, float(interior_points[:, 2].min()))
    report["observed_lower_support_m"] = z0
    mask = ((q[:, 0] >= sides[0] - band) & (q[:, 0] <= sides[1] + band)
            & (q[:, 1] >= y0 - band) & (q[:, 1] <= y1 + band)
            & (q[:, 2] >= z0 - band))
    carriage = q[mask]
    bounds = {name: _boundary(carriage, axis, value, limits) for name, axis, value in
              (("left", 0, sides[0]), ("right", 0, sides[1]), ("rear", 1, y0), ("front", 1, y1))}
    for name, coordinate in (("rear", y0), ("front", y1)):
        if not _end_connected(carriage, coordinate, sides, z1, limits):
            bounds[name].update(status="unsupported", value_m=None)
        elif bounds[name]["status"] == "supported":
            # 等高煤堆切面与端板可能有完全相同的点云，几何拟合不能证明材质/物理边界。
            bounds[name].update(status="surface_supported", candidate_m=bounds[name]["value_m"], value_m=None)
    if capture is not None:
        for name, axis, value in (("left", 0, sides[0]), ("right", 0, sides[1]), ("rear", 1, y0), ("front", 1, y1)):
            near = p[mask & (np.abs(q[:, axis] - value) <= band * 2)]
            normal = matrix[axis]
            if len(near) and any(abs(normal[j]) > .9 and
                    (np.any(np.abs(near[:, j] - capture[0][j]) <= band)
                     or np.any(np.abs(near[:, j] - capture[1][j]) <= band)) for j in (0, 1)):
                bounds[name].update(status="capture_clipped", value_m=None)
    report["boundaries"] = bounds
    complete = all(item["status"] == "supported" for item in bounds.values())
    report.update(carriage_status="available" if complete else "partial",
                  observed_carriage_extent=[carriage.min(axis=0).tolist(), carriage.max(axis=0).tolist()],
                  reason="四侧结构边界有实测支持" if complete else "仅保存已观测车厢片段，缺失边界不推定为真实厢长")
    return ObservedRoi(indices, indices[mask], report)
