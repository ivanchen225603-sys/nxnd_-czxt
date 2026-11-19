"""观测到矩形端壁时的四壁参考补全，不是空车结构测量。

供正式分析的独立外壁诊断、只读预览和显式离线导出。不能据此判断内部是否装货，不能补顶/底，
也不能把重建包围尺寸当作业务内尺寸。普通空车仍走原有对称补点流程。
"""

import numpy as np
from scipy.spatial import cKDTree

from carpointcloud_analyzer.domain.box_geometry import (
    assign_box_faces,
    find_end_wall_centers,
    mask_points_away_from,
    mask_uncovered_on_faces,
    rotate_wl,
)
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.models import BoxCarriageBox, BoxCompletionResult
from carpointcloud_analyzer.domain.utils import occupancy_extent
from carpointcloud_analyzer.domain.front_lip import complete_front_lip
from carpointcloud_analyzer.domain.utils.surfaces import axis_aligned_face

# --- 端壁平面拟合（``_fit_end``）---
# 稳健回归的重加权轮数；固定轮数保证确定性，不用收敛判据以免不同机器结果不同。
END_FIT_REWEIGHT_ROUNDS = 6
MIN_END_FIT_POINTS = 50
# 内点带取"绝对下限"与"中位残差 3 倍"的较大者：面很平时不至于把波纹板全剔掉。
END_FIT_MIN_TOLERANCE_M = 0.035
END_FIT_MEDIAN_MULTIPLIER = 3
# 判定平面度时使用的固定内点带，与上面的自适应带分开，口径才可比。
END_PLANARITY_TOLERANCE_M = 0.08
MIN_END_PLANARITY_RATIO = 0.85

# --- 输入合法性 ---
MIN_SCENE_POINTS = 1000
MAX_SCENE_EXTENT_M = 100
# 端面中心间距的可支持范围，超出说明不是单节厢体。
MIN_END_WALL_GAP_M = 2
MAX_END_WALL_GAP_M = 15

# --- 端壁竖直范围 ---
OCCUPANCY_BIN_M = 0.05
OCCUPANCY_MIN_POINTS = 5
MIN_WALL_HEIGHT_M = 1.8
MAX_WALL_HEIGHT_M = 4

# --- 两端壁的平行性与竖直性 ---
MAX_SLOPE_DISAGREEMENT = 0.08
MAX_HEIGHT_SLOPE = 0.12

# --- 箱宽 ---
MIN_BOX_WIDTH_M = 1.4
MAX_BOX_WIDTH_M = 3.5
MAX_WIDTH_DISAGREEMENT_M = 0.25

# --- 对齐后的箱长 ---
MIN_ALIGNED_LENGTH_M = 2
MAX_ALIGNED_LENGTH_M = 15
# 仅在箱体一端紧靠车尾、另一端有独立车头时接管普通分割。
BOX_TAIL_MAX_GAP_M = 0.50
BOX_CAB_MIN_OVERHANG_M = 1.20

# --- 内部可观测性判定 ---
# 从两侧壁各内缩这么多再看内部，避免把壁面自身的回波当成内部点。
INTERIOR_SIDE_INSET_M = 0.20
INTERIOR_TOP_MARGIN_M = 0.15
# 低于上沿这个深度仍有成片点，说明内部（底板/货物）可见，应走原有流程。
INTERIOR_BELOW_RIM_M = 0.25
MAX_INTERIOR_POINTS = 50

# --- 纵向上沿支撑 ---
RIM_BAND_BELOW_TOP_M = 0.25
RIM_BAND_ABOVE_TOP_M = 0.15
RIM_SIDE_MARGIN_M = 0.15
RIM_BIN_COUNT = 10
MIN_RIM_POINTS = 50
MIN_RIM_COVERED_BINS = 7

# --- 表面重采样 ---
MIN_SURFACE_SPACING_M = 0.02
# 折边与四壁共有的格点按此容差判重，仅去掉完全重合的点。
COINCIDENT_POINT_TOLERANCE_M = 1e-9

# --- 车头朝向证据 ---
ORIENTATION_MARGIN_M = 1.0
MIN_ORIENTATION_POINTS = 500
ORIENTATION_DOMINANCE = 3



def _fit_end(wall: np.ndarray) -> tuple[np.ndarray, float] | None:
    """确定性稳健回归 L=aW+bH+c，避免不完整端面的质心差被误认成偏航。"""
    design = np.column_stack((wall[:, 0], wall[:, 2], np.ones(len(wall))))
    keep = np.ones(len(wall), dtype=bool)
    for _ in range(END_FIT_REWEIGHT_ROUNDS):
        if (keep.sum() < MIN_END_FIT_POINTS
                or np.linalg.matrix_rank(design[keep]) < 3):
            return None
        fit = np.linalg.lstsq(design[keep], wall[keep, 1], rcond=None)[0]
        residual = np.abs(design @ fit - wall[:, 1])
        next_keep = residual <= max(
            END_FIT_MIN_TOLERANCE_M,
            END_FIT_MEDIAN_MULTIPLIER * float(np.median(residual[keep])),
        )
        if np.array_equal(keep, next_keep):
            break
        keep = next_keep
    return fit, float(np.mean(residual <= END_PLANARITY_TOLERANCE_M))


class _Rejected(Exception):
    """点云不在参考重建的适用范围内；调用方保持原车型流程。

    各阶段用抛异常代替层层 ``return``：原实现 12 处守卫全部堆在一个函数里，
    圈复杂度 42。拆开后每个阶段只关心自己的判据，入口统一转成空结果。
    """

    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = reason


def _crop_end_walls(
    points: np.ndarray, config: Config
) -> tuple[list[np.ndarray], float, float]:
    """定位两块端壁并裁到共同的竖直范围。

    Returns:
        ``(裁剪后的两块端壁, 下沿 h0, 上沿 h1)``。

    Raises:
        _Rejected: 端壁数量、间距、支撑点数或高度不在适用范围内。
    """
    if np.max(np.ptp(points, axis=0)) > MAX_SCENE_EXTENT_M:
        raise _Rejected("unsupported_scene_extent")
    centers = find_end_wall_centers(points, config)
    if centers is None:
        raise _Rejected("two_end_walls_required")
    centers = sorted(centers)
    if not MIN_END_WALL_GAP_M <= centers[1] - centers[0] <= MAX_END_WALL_GAP_M:
        raise _Rejected("unsupported_length")
    band = config.carriage_box_end_wall_band
    walls = [points[np.abs(points[:, 1] - center) <= band] for center in centers]
    if min(map(len, walls)) < config.carriage_box_min_end_wall_points:
        raise _Rejected("end_wall_support_insufficient")

    ranges = [occupancy_extent(w[:, 2], OCCUPANCY_BIN_M, OCCUPANCY_MIN_POINTS)
              for w in walls]
    # 先剔除车头一端的车架/轮胎，再估宽，不能让下方底盘把箱宽撑大。
    h0, h1 = max(r[0] for r in ranges), max(r[1] for r in ranges)
    if not MIN_WALL_HEIGHT_M <= h1 - h0 <= MAX_WALL_HEIGHT_M:
        raise _Rejected("unsupported_wall_height")
    cropped = [w[(w[:, 2] >= h0) & (w[:, 2] <= h1)] for w in walls]
    if min(map(len, cropped)) < config.carriage_box_min_end_wall_points:
        raise _Rejected("cropped_end_wall_support_insufficient")
    return cropped, h0, h1


def _fit_box(
    points: np.ndarray, cropped: list[np.ndarray], h0: float, h1: float, config: Config
) -> tuple[BoxCarriageBox, float, np.ndarray]:
    """由两块端壁的主平面求偏航、箱宽与两端位置。

    Returns:
        ``(厢体长方体, 偏航弧度, 已转正的全部点)``。

    Raises:
        _Rejected: 端壁不平、不平行、偏航超限、两端宽度不一致或箱长超范围。
    """
    fits = [_fit_end(w) for w in cropped]
    if any(f is None or f[1] < MIN_END_PLANARITY_RATIO for f in fits):
        raise _Rejected("end_walls_not_planar")
    slopes = np.array([f[0][:2] for f in fits])
    if (np.max(np.abs(slopes[0] - slopes[1])) > MAX_SLOPE_DISAGREEMENT
            or np.max(np.abs(slopes[:, 1])) > MAX_HEIGHT_SLOPE):
        raise _Rejected("end_walls_not_parallel_vertical")
    yaw = float(np.arctan(np.mean(slopes[:, 0])))
    if abs(np.degrees(yaw)) > config.carriage_box_max_yaw_deg:
        raise _Rejected("end_wall_yaw_out_of_range")
    # L'=sin(-yaw)W+cos(-yaw)L；偏航符号不能由端点密度顺序决定。
    aligned = rotate_wl(points, -yaw)
    aligned_walls = [rotate_wl(w, -yaw) for w in cropped]
    wr = [occupancy_extent(w[:, 0], OCCUPANCY_BIN_M, OCCUPANCY_MIN_POINTS)
          for w in aligned_walls]
    if (any(not MIN_BOX_WIDTH_M <= r[1] - r[0] <= MAX_BOX_WIDTH_M for r in wr)
            or np.max(np.abs(np.array(wr[0]) - wr[1])) > MAX_WIDTH_DISAGREEMENT_M):
        raise _Rejected("end_wall_widths_disagree")
    w0, w1 = np.mean(wr, axis=0)
    # 用端面主平面而不是取点带外沿，保留波纹板/附件为实测点，不造第二层皮。
    mid_h = (h0 + h1) / 2
    end_l = [float((fit[0][1] * mid_h + fit[0][2]) * np.cos(yaw)) for fit in fits]
    rear, front = sorted(end_l)
    box = BoxCarriageBox(front, rear, (float(w0), float(w1)), (h0, h1),
                         len(cropped[1]), len(cropped[0]))
    if not MIN_ALIGNED_LENGTH_M <= box.length <= MAX_ALIGNED_LENGTH_M:
        raise _Rejected("unsupported_aligned_length")
    return box, yaw, aligned


def _require_closed_shell(aligned: np.ndarray, box: BoxCarriageBox, band: float) -> int:
    """确认内部不可观测且纵向上沿有连续支撑。

    Returns:
        上沿以下的内部点数，写入元数据供人工复核。

    Raises:
        _Rejected: 已看到内部底板/货物，或纵向上沿支撑不足。
    """
    rear, front = box.rear_l, box.front_l
    w0, w1 = box.w_range
    h1 = box.h_range[1]
    interior = aligned[
        (aligned[:, 1] > rear + band * 2) & (aligned[:, 1] < front - band * 2)
        & (aligned[:, 0] > w0 + INTERIOR_SIDE_INSET_M)
        & (aligned[:, 0] < w1 - INTERIOR_SIDE_INSET_M)
        & (aligned[:, 2] <= h1 + INTERIOR_TOP_MARGIN_M)
    ]
    # 不以下沿限制搜索：外壁下沿可能被遮挡，真实底板可能比它更低。
    # 已看到内部底板/货物的车型不走外壳推断，避免空车或载货车被套成箱子。
    below_rim = interior[interior[:, 2] < h1 - INTERIOR_BELOW_RIM_M]
    if len(below_rim) >= MAX_INTERIOR_POINTS:
        raise _Rejected("interior_observed_use_existing_pipeline")
    middle = aligned[(aligned[:, 1] > rear + band) & (aligned[:, 1] < front - band)]
    rim = middle[(middle[:, 2] > h1 - RIM_BAND_BELOW_TOP_M)
                 & (middle[:, 2] < h1 + RIM_BAND_ABOVE_TOP_M)
                 & (middle[:, 0] > w0 - RIM_SIDE_MARGIN_M)
                 & (middle[:, 0] < w1 + RIM_SIDE_MARGIN_M)]
    rim_index = (rim[:, 1] - rear) / box.length * RIM_BIN_COUNT
    rim_bins = np.unique(np.clip(rim_index.astype(int), 0, RIM_BIN_COUNT - 1))
    if len(rim) < MIN_RIM_POINTS or len(rim_bins) < MIN_RIM_COVERED_BINS:
        raise _Rejected("longitudinal_rim_support_insufficient")
    return len(below_rim)


def _matches_configured_profile(box: BoxCarriageBox, config: Config) -> bool:
    """观测到的长宽是否落在配置的车型窗口内。

    只做车型识别，不引入任何配置尺寸：箱体高度一律来自本次观测。历史实现还会
    比对配置的标称整体高 2.9m 并据此下移箱底，那个高度不来自当前点云，已移除。
    """
    length_range = config.carriage_wall_reference_profile_length_range
    width_range = config.carriage_wall_reference_profile_width_range
    return bool(
        length_range[0] <= box.length <= length_range[1]
        and width_range[0] <= box.width <= width_range[1]
    )


def _build_wall_surfaces(
    points: np.ndarray, aligned: np.ndarray, box: BoxCarriageBox, yaw: float,
    profile_matched: bool, config: Config,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[np.ndarray]]:
    """在四个外壁上重采样，并按是否匹配车型选择去重方式。

    Returns:
        ``(转回原姿态的合成点, 面编号, 转正坐标下的合成点, 三条轴)``。

    Raises:
        _Rejected: 配置的重采样间距过小。
    """
    # linspace 保证端点准确，不用 arange(stop+step) 将壁面延伸到箱体外。
    step = config.carriage_box_surface_spacing
    if step < MIN_SURFACE_SPACING_M:
        raise _Rejected("reference_spacing_too_small")
    w0, w1 = box.w_range
    rear, front = box.rear_l, box.front_l
    axes = [np.linspace(lo, hi, int(np.ceil((hi - lo) / step)) + 1)
            for lo, hi in [(w0, w1), (rear, front), box.h_range]]
    faces, labels = [], []
    for face_id, (axis, value) in enumerate([(0, w0), (0, w1), (1, rear), (1, front)]):
        face = axis_aligned_face(axes, axis, value)
        faces.append(face)
        labels.append(np.full(len(face), face_id, dtype=int))
    synthetic, face_ids = np.vstack(faces), np.concatenate(labels)
    if profile_matched:
        # 完整规则壳体与实测云是两层不同来源的数据。端壁附近的波纹板点不能通过
        # 投影/大半径去重挖掉模型；否则前壁出现缺口、左右面密度不对称。
        # 仅去掉四面共用棱线的重复网格点，不移动/裁剪/镜像改写任何实测点。
        synthetic, indices = np.unique(synthetic, axis=0, return_index=True)
        face_ids = face_ids[indices]
        restored = rotate_wl(synthetic, yaw)
    else:
        owners = assign_box_faces(aligned, box, config.carriage_box_face_assign_band)
        keep = mask_uncovered_on_faces(synthetic, face_ids, aligned, owners,
                                       config.carriage_box_surface_dedup_radius)
        synthetic, face_ids = synthetic[keep], face_ids[keep]
        restored = rotate_wl(synthetic, yaw)
        keep = mask_points_away_from(restored, points,
                                     config.carriage_completion_dedup_radius)
        restored, face_ids = restored[keep], face_ids[keep]
    return restored, face_ids, synthetic, axes


def _infer_front_orientation(
    aligned: np.ndarray, rear: float, front: float
) -> tuple[bool, bool]:
    """按端壁外侧多出来的点判断车头在哪一端；证据不足时两个都为 False。"""
    low_support = int(np.sum(aligned[:, 1] < rear - ORIENTATION_MARGIN_M))
    high_support = int(np.sum(aligned[:, 1] > front + ORIENTATION_MARGIN_M))
    front_low = low_support > max(MIN_ORIENTATION_POINTS,
                                  high_support * ORIENTATION_DOMINANCE)
    front_high = high_support > max(MIN_ORIENTATION_POINTS,
                                    low_support * ORIENTATION_DOMINANCE)
    return front_low, front_high


def complete_rectangular_wall_reference(
    points: np.ndarray, config: Config
) -> BoxCompletionResult:
    """为有两块可核对端壁、内部不可观测的矩形壳体补四面外壁。

    拒绝单面、明显不平行/异形、可见底板和大面积内部点；失败返回空点，
    由调用方保持原车型流程。所有阈值是参考重建的适用范围，不放宽业务门禁。
    """
    if not config.carriage_wall_reference_enable:
        return _skip("wall_reference_disabled")
    points = np.asarray(points, dtype=float)
    if (points.ndim != 2 or points.shape[1:] != (3,) or len(points) < MIN_SCENE_POINTS
            or not np.isfinite(points).all()):
        return _skip("insufficient_finite_points")
    try:
        return _build_wall_reference(points, config)
    except _Rejected as rejected:
        return _skip(rejected.reason)


def _observed_box_fit(
    points: np.ndarray, config: Config,
) -> tuple[BoxCarriageBox, float, np.ndarray] | None:
    """核验短矩形双端面，返回同一去偏航坐标系的箱体与点集。"""
    values = np.asarray(points, dtype=float)
    if (not config.carriage_wall_reference_enable or values.ndim != 2
            or values.shape[1:] != (3,) or len(values) < MIN_SCENE_POINTS
            or not np.isfinite(values).all()):
        return None
    try:
        cropped, h0, h1 = _crop_end_walls(values, config)
        fitted = _fit_box(values, cropped, h0, h1, config)
    except _Rejected:
        return None
    if not _matches_configured_profile(fitted[0], config):
        return None
    return fitted


def _has_cab_overhang(points: np.ndarray, rear: float, front: float) -> bool:
    tail, head = occupancy_extent(points[:, 1])
    outside = (max(0.0, rear - tail), max(0.0, head - front))
    return min(outside) <= BOX_TAIL_MAX_GAP_M and max(outside) >= BOX_CAB_MIN_OVERHANG_M


def align_observed_rectangular_body(
    points: np.ndarray, config: Config,
) -> tuple[np.ndarray, BoxCarriageBox, float] | None:
    """仅已核验短箱返回去偏航后的点、箱体和角度；高度轴保持不变。"""
    fitted = _observed_box_fit(points, config)
    if fitted is None:
        return None
    box, yaw, aligned = fitted
    if not _has_cab_overhang(aligned, box.rear_l, box.front_l):
        return None
    return aligned, box, yaw


def observe_rectangular_body(points: np.ndarray, config: Config) -> BoxCarriageBox | None:
    """核验短箱外轮廓，返回当前 L 轴上的两端，不代表内壁或空车。"""
    fitted = _observed_box_fit(points, config)
    if fitted is None:
        return None
    box, yaw, _ = fitted
    # BoxCarriageBox 的 L 是去偏航后的坐标，转回当前局部坐标的箱宽中线。
    midpoint_w = sum(box.w_range) / 2
    rear = (box.rear_l + np.sin(yaw) * midpoint_w) / np.cos(yaw)
    front = (box.front_l + np.sin(yaw) * midpoint_w) / np.cos(yaw)
    if not _has_cab_overhang(np.asarray(points, dtype=float), rear, front):
        return None
    return BoxCarriageBox(float(front), float(rear), box.w_range, box.h_range,
                           box.front_points, box.rear_points)


def _skip(reason: str) -> BoxCompletionResult:
    """不适用时的统一空结果。"""
    return BoxCompletionResult(np.empty((0, 3), dtype=float), None, {
        "completed": False, "reason": reason, "referenceOnly": True,
    })


def _build_wall_reference(points: np.ndarray, config: Config) -> BoxCompletionResult:
    """已通过输入校验后的重建主流程；任一阶段不适用即抛 ``_Rejected``。"""
    band = config.carriage_box_end_wall_band
    cropped, h0, h1 = _crop_end_walls(points, config)
    box, yaw, aligned = _fit_box(points, cropped, h0, h1, config)
    interior_below_rim = _require_closed_shell(aligned, box, band)

    # 箱体高度只由本次观测决定；识别车型不改变任何尺寸。
    observed_h0, observed_h1 = h0, h1
    profile_matched = _matches_configured_profile(box, config)

    restored, face_ids, synthetic, axes = _build_wall_surfaces(
        points, aligned, box, yaw, profile_matched, config)
    # 前后业务朝向可能被原局部建系判反：这里只记录有无车头侧证据，不猜未知朝向。
    front_low, front_high = _infer_front_orientation(aligned, box.rear_l, box.front_l)
    names = ["negativeWSide", "positiveWSide", "lowLEnd", "highLEnd"]
    if front_low or front_high:
        names[2:] = ["frontWall", "rearWall"] if front_low else ["rearWall", "frontWall"]
    lip_metadata = {"completed": False, "reason": "profile_not_enabled",
                    "synthesizedPoints": 0}
    if profile_matched and config.carriage_wall_reference_front_lip_enable:
        step = config.carriage_box_surface_spacing
        lip, lip_metadata = complete_front_lip(
            aligned, box, front_low, front_high, axes[0], step)
        if len(lip):
            # 与四壁共有的前沿格点只保存一次，避免叠点；新折边不改变四壁边界。
            lip = lip[
                cKDTree(synthetic).query(lip)[0] > COINCIDENT_POINT_TOLERANCE_M]
            lip_metadata["synthesizedPoints"] = len(lip)
            restored = np.vstack([restored, rotate_wl(lip, yaw)])
            face_ids = np.concatenate([face_ids, np.full(len(lip), 4, dtype=int)])
            names.append("frontTopLip")
    metadata = _reference_metadata(
        box, yaw, restored, face_ids, names, lip_metadata, profile_matched,
        observed_h0, observed_h1, interior_below_rim, front_low, front_high)
    return BoxCompletionResult(restored, box, metadata)


def _reference_metadata(
    box: BoxCarriageBox, yaw: float, restored: np.ndarray, face_ids: np.ndarray,
    names: list[str], lip_metadata: dict, profile_matched: bool,
    observed_h0: float, observed_h1: float, interior_below_rim: int,
    front_low: bool, front_high: bool,
) -> dict:
    """组装参考重建的审计元数据；口径与警示语随是否套用标称高度而变。"""
    w0, w1 = box.w_range
    h0, h1 = box.h_range
    return {
        "completed": bool(len(restored)),
        "method": "observed_rectangular_four_wall_reference",
        "referenceOnly": True, "interiorObservable": False,
        "interiorBelowRimPoints": interior_below_rim,
        "topGenerated": False, "bottomGenerated": False,
        "partialTopGenerated": bool(lip_metadata["completed"]), "roofClosed": False,
        "frontLip": lip_metadata,
        "profileMatched": profile_matched,
        "heightSource": "observed_wall_extent",
        "observedHeightM": observed_h1 - observed_h0,
        "observedHRange": [observed_h0, observed_h1],
        "heightAnchor": "observed_top",
        "surfaceMode": ("complete_symmetric_grid" if profile_matched
                        else "observed_coverage_fill"),
        "symmetricAboutW": float((w0 + w1) / 2),
        "frontRearInferredFromOverhang": bool(front_low or front_high),
        "frontAtLowL": bool(front_low), "frontAtHighL": bool(front_high),
        "bounds": {
            "lowLEnd": box.rear_l, "highLEnd": box.front_l,
            "lengthM": box.length, "widthM": box.width,
            "visibleHeightM": observed_h1 - observed_h0, "modelHeightM": box.height,
            "wRange": [float(w0), float(w1)], "hRange": [h0, h1],
        },
        "yawDeg": float(np.degrees(yaw)),
        "synthesizedPoints": len(restored),
        "walls": {name: {"synthesizedPoints": int(np.sum(face_ids == i))}
                  for i, name in enumerate(names)},
        "warning": ("四壁按现场整体高度及左右对称先验完整重建；合成面可能与实测波纹板有偏移。"
                    "不代表内高、实测底板或空载证明。" if profile_matched else
                    "四壁按矩形先验推断；不可观测下沿可能偏高。不是空载证明或业务尺寸。"),
    }
