"""体积补洞的遮挡上限与坡度包络（2026-10-24）。

俯视单雷达在车厢两端是掠射：停稳帧打不到的格子，原来只用四邻域松弛（height_fill）补，
它没有任何物理约束 —— 阴影区补出来的高度可以高过"视线已经穿过去"的位置，
贴前壁上升的煤面又会被拉平。这里补两条由观测和散料性质给出的约束：

1. **视线上限**（观测给的，硬约束）：雷达打到某个更远的点，说明从雷达到那个点的
   整条视线都穿过了空气。视线越过的每一格，煤面都必须低于视线在那一格的高度，
   否则这条激光会先打在煤上。逐点射线采样，每格取经过它的最低视线。
2. **坡度包络**（假设给的，软约束）：静止散料表面坡度不超过堆积角。离已测格 d 远的
   未测格，高度在 [h_k - tan(theta)*d, h_k + tan(theta)*d] 之内（对所有已测格取交）。
   theta 取 45 度，比煤的自然堆积角（约 35~40 度）陡，包络因此偏宽、不会把真值挡在外面。
   TODO(现场)：没有本矿煤的实测堆积角，45 度只是保守取值。

估值 = 原松弛结果夹到 [下包络, min(上包络, 视线上限)] 之间；只动未测格，实测格一个不改。
另报"未测格全取下界 / 全取上界"的体积包络，让现场看得见这一车有多少体积是推出来的。

▲ 它**不是**测量：上限只排除"不可能"的高度，下限依赖堆积角假设。包络宽说明这车两端
  确实没看清，不能拿包络中点当真值，也不能用它反推密度。
▲ 只用停稳帧（雷达在坐标原点）做视线；进场帧的雷达位置经配准平移、高度另扣过，
  不拿来做视线约束。
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple

import numpy as np


OCCLUSION_COLUMN_M = 0.05      # 射线终点按 5 cm 竖列去重，每列取最高点（偏松，不会把真货面压低）
OCCLUSION_STOP_CELLS = 1.5     # 终点所在格及相邻半格不采样：终点处视线就贴着表面
OCCLUSION_MARGIN_M = 0.03      # 测距噪声与格内起伏的余量，上限再抬这么多
OCCLUSION_STEP_FRACTION = 0.5  # 沿视线采样步长 = 格宽 x 这个比例
OCCLUSION_MAX_SAMPLES = 3_000_000  # 采样总数上限；超了按比例放大步长（约束只会变松）
OCCLUSION_ENDPOINT_MARGIN_M = 1.0  # 射线终点只取积分域外包再往外这么远以内的（端壁、紧邻地面）
REPOSE_TAN = 1.0               # 坡度包络用的 tan(45 度)，见模块说明
REPOSE_EST_TAN = math.tan(math.radians(38.0))  # 阴影格估值用的堆积角；TODO(现场)：本矿煤实测后改
SHADOW_MIN_DEPTH_M = 0.20      # 空洞最深处离最近回波不到这么远的，按扫描线窄缝处理（不当阴影）
_ENVELOPE_MAX_ITERS = 4096


@dataclass(frozen=True)
class OcclusionFillInfo:
    """仅内存诊断，不进 VolumeResult.to_dict 的归档合同（同 cross_check）。"""
    status: str                    # applied / skipped
    reason: str
    unknown_cells: int = 0         # 积分域内补洞前没有实测的格
    ceiling_cells: int = 0         # 其中有视线上限的
    shadow_cells: int = 0          # 其中在实测货面阴影里、按堆积角背坡估的
    lowered_cells: int = 0         # 估值被视线上限或上包络压低的
    raised_cells: int = 0          # 估值被下包络抬高的
    volume_lo_m3: float = float("nan")   # 未测格全取下界
    volume_hi_m3: float = float("nan")   # 未测格全取上界

    def text(self) -> str:
        if self.status != "applied":
            return "（遮挡上限补洞未用：%s）" % self.reason
        return ("（遮挡上限补洞：未测 %d 格，%d 格有视线上限，%d 格在货堆阴影里按堆积角估，"
                "压低 %d 格、抬高 %d 格；未测区按上下界的体积包络 %.2f~%.2f m^3）" % (
                    self.unknown_cells, self.ceiling_cells, self.shadow_cells, self.lowered_cells,
                    self.raised_cells, self.volume_lo_m3, self.volume_hi_m3))


def ray_ceiling(points: np.ndarray, x0: float, y0: float, nx: int, ny: int, cell: float,
                floor_grid: np.ndarray, origin: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                domain: Optional[np.ndarray] = None) -> np.ndarray:
    """逐格视线上限（相对 floor_grid 的高度，已含余量）；没有视线经过的格为 nan。

    points 与 origin 在同一坐标系（载货坐标，雷达在原点、z 向下为负）。
    floor_grid (ny, nx) 是每格中心的绝对底板高度，用它把视线高度换成与高度图同口径的相对量。
    domain：只关心的格（积分域）。给了就只取终点落在它外包 + OCCLUSION_ENDPOINT_MARGIN_M 内的射线 ——
    ROI 两头的地面、驾驶室回波的视线大多不经过车厢，算了也白算（每车省约 1 s）。
    """
    pts = np.asarray(points, np.float64)
    floor_grid = np.asarray(floor_grid, np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3 or floor_grid.shape != (ny, nx) or nx <= 0 or ny <= 0:
        raise ValueError("视线上限输入形状非法")
    ceiling = np.full(ny * nx, np.inf)
    ox, oy, oz = (float(v) for v in origin)
    lo_x, hi_x, lo_y, hi_y = x0, x0 + nx * cell, y0, y0 + ny * cell
    if domain is not None and np.any(domain):
        rows, cols = np.nonzero(domain)
        lo_x = max(lo_x, x0 + cols.min() * cell - OCCLUSION_ENDPOINT_MARGIN_M)
        hi_x = min(hi_x, x0 + (cols.max() + 1) * cell + OCCLUSION_ENDPOINT_MARGIN_M)
        lo_y = max(lo_y, y0 + rows.min() * cell - OCCLUSION_ENDPOINT_MARGIN_M)
        hi_y = min(hi_y, y0 + (rows.max() + 1) * cell + OCCLUSION_ENDPOINT_MARGIN_M)
    pts = pts[np.isfinite(pts).all(axis=1) & (pts[:, 2] < oz)
              & (pts[:, 0] >= lo_x) & (pts[:, 0] < hi_x)
              & (pts[:, 1] >= lo_y) & (pts[:, 1] < hi_y)]
    if len(pts):
        # 竖列去重，每列只留最高点：低于真实表面的离群回波会把上限压得过紧，最高点只会让它变松。
        cx = np.floor((pts[:, 0] - x0) / OCCLUSION_COLUMN_M).astype(np.int64)
        cy = np.floor((pts[:, 1] - y0) / OCCLUSION_COLUMN_M).astype(np.int64)
        key = cy * (int(math.ceil(nx * cell / OCCLUSION_COLUMN_M)) + 1) + cx
        order = np.lexsort((pts[:, 2], key))
        key, pts = key[order], pts[order]
        last = np.ones(len(key), bool)
        last[:-1] = key[1:] != key[:-1]
        pts = pts[last]
    if len(pts):
        dx, dy, dz = pts[:, 0] - ox, pts[:, 1] - oy, pts[:, 2] - oz
        reach = np.hypot(dx, dy)
        step = cell * OCCLUSION_STEP_FRACTION
        usable = reach - OCCLUSION_STOP_CELLS * cell
        counts = np.where(usable > step, np.floor(usable / step), 0).astype(np.int64)
        total = int(counts.sum())
        if total > OCCLUSION_MAX_SAMPLES:
            scale = total / OCCLUSION_MAX_SAMPLES
            step *= scale
            counts = np.where(usable > step, np.floor(usable / step), 0).astype(np.int64)
            total = int(counts.sum())
        if total:
            ray = np.repeat(np.arange(len(pts)), counts)
            k = np.arange(total) - np.repeat(np.cumsum(counts) - counts, counts) + 1
            t = k * step / reach[ray]
            sx = ox + dx[ray] * t
            sy = oy + dy[ray] * t
            sz = oz + dz[ray] * t
            ix = np.floor((sx - x0) / cell).astype(np.int64)
            iy = np.floor((sy - y0) / cell).astype(np.int64)
            inside = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
            flat = iy[inside] * nx + ix[inside]
            value = sz[inside] - floor_grid.reshape(-1)[flat]
            if len(flat):
                order = np.argsort(flat, kind="stable")
                flat, value = flat[order], value[order]
                start = np.flatnonzero(np.r_[True, flat[1:] != flat[:-1]])
                ceiling[flat[start]] = np.minimum.reduceat(value, start)
    ceiling = ceiling.reshape(ny, nx)
    # 视线只约束它经过的那一点；格内按"取最高点"统计，坡面上格内另一侧可以再高半格的起伏
    # （射线合成场景 35 度坡、0.10 m 格实测最多低 3.4 cm）。按假设的最陡坡补半格。
    margin = OCCLUSION_MARGIN_M + 0.5 * cell * REPOSE_TAN
    return np.where(np.isfinite(ceiling), ceiling + margin, np.nan)


def _min_plus_envelope(seed: np.ndarray, domain: np.ndarray, cell: float, slope: float) -> np.ndarray:
    """U = min_k(seed_k + slope * 路径长)，只在 domain 内传播（8 邻域倒角距离，偏长不超过 8%）。"""
    value = np.where(domain & np.isfinite(seed), seed, np.inf)
    ny, nx = value.shape
    moves = ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1))
    for _ in range(_ENVELOPE_MAX_ITERS):
        best = value.copy()
        for di, dj in moves:
            cost = slope * cell * (math.sqrt(2.0) if di and dj else 1.0)
            src = value[max(0, -di):ny - max(0, di), max(0, -dj):nx - max(0, dj)]
            dst = best[max(0, di):ny - max(0, -di), max(0, dj):nx - max(0, -dj)]
            np.minimum(dst, src + cost, out=dst)
        best = np.where(domain, best, np.inf)
        if np.array_equal(best, value):
            break
        value = best
    return value


def _deep_hole_components(holes: np.ndarray, cell: float) -> np.ndarray:
    """只留"够深"的连通空洞（8 邻接）：洞内最远一格离洞外至少 SHADOW_MIN_DEPTH_M。

    为什么要这一条：背坡试探点紧挨着任何一个掠射命中点时都会被它挡住（堆积角比掠射仰角陡），
    所以"试探点被挡"区分不了阴影和扫描线之间的窄缝。窄缝两侧都有回波、松弛插值本来就准；
    阴影是成片的（货堆背面、车厢最外一段），洞深明显大于扫描线间距。
    """
    holes = np.asarray(holes, bool)
    if not holes.any():
        return holes
    depth = _min_plus_envelope(np.where(holes, np.nan, 0.0), np.ones_like(holes), cell, 1.0)
    ny, nx = holes.shape
    label = np.where(holes, np.arange(ny * nx).reshape(ny, nx), ny * nx)
    moves = ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1))
    for _ in range(_ENVELOPE_MAX_ITERS):
        best = label.copy()
        for di, dj in moves:
            src = label[max(0, -di):ny - max(0, di), max(0, -dj):nx - max(0, dj)]
            dst = best[max(0, di):ny - max(0, -di), max(0, dj):nx - max(0, -dj)]
            np.minimum(dst, src, out=dst)
        best = np.where(holes, best, ny * nx)
        if np.array_equal(best, label):
            break
        label = best
    ids = label[holes]
    deepest = np.zeros(ny * nx + 1)
    np.maximum.at(deepest, ids, depth[holes])
    keep = np.zeros_like(holes)
    keep[holes] = deepest[ids] >= SHADOW_MIN_DEPTH_M - 1e-9
    return keep


def repose_descent(measured: np.ndarray, domain: np.ndarray, cell: float,
                   repose_tan: float = REPOSE_EST_TAN) -> np.ndarray:
    """从各实测格按堆积角往外下降的面（取各格下降面的最高者，不低于 0）；域内无实测格可达为 nan。"""
    known = domain & np.isfinite(measured)
    value = -_min_plus_envelope(np.where(known, -measured, np.nan), domain, cell, repose_tan)
    return np.where(np.isfinite(value), np.maximum(value, 0.0), np.nan)


def shadowed_cells(trial: np.ndarray, measured: np.ndarray, candidates: np.ndarray,
                   x0: float, y0: float, cell: float, floor_grid: np.ndarray,
                   origin: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    """候选格里哪些在实测货面的阴影里：雷达看向它的试探表面点时，途中先碰到实测格。

    试探高度要用**堆积角背坡**（repose_descent），不能用松弛值：松弛值在阴影里正好贴着
    掠过脊线的视线，测出来恒为"看得见"。背坡点被挡住 = 那里本来就不该有回波，按背坡估；
    背坡点看得见却没回波 = 采样稀疏（掠射处扫描线间距大于格宽），保留松弛值。
    只拿**实测格**当遮挡物（它们是真表面）。
    """
    estimate = trial
    ny, nx = estimate.shape
    candidates = candidates & _deep_hole_components(candidates, cell)
    target = np.flatnonzero(candidates & np.isfinite(estimate))
    shadow = np.zeros(ny * nx, bool)
    if not len(target):
        return shadow.reshape(ny, nx)
    ox, oy, oz = (float(v) for v in origin)
    ty, tx = np.divmod(target, nx)
    px = x0 + (tx + 0.5) * cell
    py = y0 + (ty + 0.5) * cell
    pz = floor_grid.reshape(-1)[target] + estimate.reshape(-1)[target]
    reach = np.hypot(px - ox, py - oy)
    step = cell * OCCLUSION_STEP_FRACTION
    counts = np.where(reach - OCCLUSION_STOP_CELLS * cell > step,
                      np.floor((reach - OCCLUSION_STOP_CELLS * cell) / step), 0).astype(np.int64)
    total = int(counts.sum())
    if not total:
        return shadow.reshape(ny, nx)
    ray = np.repeat(np.arange(len(target)), counts)
    k = np.arange(total) - np.repeat(np.cumsum(counts) - counts, counts) + 1
    t = k * step / reach[ray]
    ix = np.floor((ox + (px[ray] - ox) * t - x0) / cell).astype(np.int64)
    iy = np.floor((oy + (py[ray] - oy) * t - y0) / cell).astype(np.int64)
    sz = oz + (pz[ray] - oz) * t
    inside = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    ray, ix, iy, sz = ray[inside], ix[inside], iy[inside], sz[inside]
    flat = iy * nx + ix
    surface = floor_grid.reshape(-1)[flat] + measured.reshape(-1)[flat]
    blocked = np.isfinite(surface) & (surface > sz + OCCLUSION_MARGIN_M)
    shadow[target[np.unique(ray[blocked])]] = True
    return shadow.reshape(ny, nx)


def bound_unknown_cells(estimate: np.ndarray, measured: np.ndarray, domain: np.ndarray,
                        ceiling: Optional[np.ndarray], cell: float, slope: float = REPOSE_TAN,
                        shadow: Optional[np.ndarray] = None, repose_tan: float = REPOSE_EST_TAN
                        ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
    """把未测格的估值夹进 [下包络, min(上包络, 视线上限)]；实测格原样返回。

    shadow 给出的阴影格不用松弛值：松弛把遮挡脊线和远处可见面直接连起来，正好贴着掠过
    脊线的那条视线，也就是取了上限。阴影里的面是货堆背坡，散料背坡按堆积角下降，
    所以估值取"从各实测格按 repose_tan 下降"的包络（再夹进上下界）。

    返回 (夹过的估值, 下界图, 上界图, 计数)。下界、上界只在未测格上有意义，其余格等于估值。
    视线上限与下包络冲突时（陡于假设坡度的真实地形、或堆积角假设不成立）以观测为准：
    下界让到上限。
    """
    est = np.asarray(estimate, np.float64)
    known = domain & np.isfinite(measured)
    unknown = domain & ~known & np.isfinite(est)
    hi = _min_plus_envelope(np.where(known, measured, np.nan), domain, cell, slope)
    lo = -_min_plus_envelope(np.where(known, -measured, np.nan), domain, cell, slope)
    has_ceiling = np.zeros_like(unknown)
    if ceiling is not None:
        has_ceiling = unknown & np.isfinite(ceiling)
        hi = np.where(has_ceiling, np.minimum(hi, ceiling), hi)
    lo = np.minimum(lo, hi)
    shadow = unknown & (np.zeros_like(unknown) if shadow is None else np.asarray(shadow, bool))
    if np.any(shadow):
        descent = repose_descent(measured, domain, cell, repose_tan)
        prior = est.copy()
        prior[shadow] = np.where(np.isfinite(descent[shadow]), np.maximum(descent[shadow], 0.0), est[shadow])
    else:
        prior = est
    lowered = unknown & (prior > hi)
    raised = unknown & (prior < lo)
    out = est.copy()
    out[unknown] = np.clip(prior[unknown], lo[unknown], hi[unknown])
    lo_map = est.copy()
    hi_map = est.copy()
    lo_map[unknown] = np.where(np.isfinite(lo[unknown]), lo[unknown], out[unknown])
    hi_map[unknown] = np.where(np.isfinite(hi[unknown]), hi[unknown], out[unknown])
    counts = dict(unknown_cells=int(np.count_nonzero(unknown)),
                  ceiling_cells=int(np.count_nonzero(has_ceiling)),
                  shadow_cells=int(np.count_nonzero(shadow)),
                  lowered_cells=int(np.count_nonzero(lowered)),
                  raised_cells=int(np.count_nonzero(raised)))
    return out, lo_map, hi_map, counts
