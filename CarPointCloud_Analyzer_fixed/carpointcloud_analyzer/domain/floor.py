"""车厢底板剖面：四种情况判别 + 三段 (H, L) 提取。

完整规格见 ``docs/floor_profile_spec.md``（含四种情况的示意图）。本模块只实现规格，
所有取值口径以该文档为准。要点复述：

* 高度 H = 测量点到**地磅标定面**的法向距离（兼容入口也可传场景地平面，但会在
  ``heightReference`` 中明确标识）。不能直接做局部 z 差 —— 局部 H 轴与参考面
  法向不一定平行，实测倾角 1.05° 时，11m 车长上会累积约 0.2m 偏差。
* 长度 L = 沿车长方向的水平投影；L1 从车厢前壁起算，L2/L3 从前一个测量点起算。
* **统一测量点规则**：测量点的 L 取该段（或落差特征）的**后边界**，H 取其**前边界**
  处的高度。这条规则来自"垂直边放在车厢里面"的约定，一条式子覆盖情况 2/3/4，且
  方向无关 —— 上坡车型（后高前低）不需要镜像逻辑，公式原样成立。
* 只在空车时有意义：载货时俯视雷达看到的是货面而非底板。

内部一律在 ``s`` 坐标下工作：``s = 车厢前壁 → 当前位置`` 的距离，前壁为 0、向车厢
后方增大。局部坐标里车头在 L 大的一端，故 ``s = split_l - L``，与 L 轴方向解耦。
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.spatial import cKDTree

from carpointcloud_analyzer.domain.carriage import floor_surface
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.inner_faces import FACE_TOLERANCE_M
from carpointcloud_analyzer.domain.runtime import logger
from carpointcloud_analyzer.domain.utils.cells import unique_cells

CASE_FLAT = 1
CASE_STEP_90 = 2
CASE_STEP_RAMP = 3
CASE_FULL_SLOPE = 4

_CASE_NAMES = {
    CASE_FLAT: "flat",
    CASE_STEP_90: "step_90",
    CASE_STEP_RAMP: "step_ramp",
    CASE_FULL_SLOPE: "full_slope",
}


@dataclass
class _StepFeature:
    """落差特征的前后边界（s 坐标）。

    ``front_s`` 是读取 H 的一侧，``rear_s`` 是测量点所在位置 —— 对应规格里
    "L 取后边界、H 取前边界处的高度"。
    """

    front_s: float
    rear_s: float
    platform_height_range: Tuple[float, float]
    # True 表示台阶/坡道之后（s 较大一侧）才是高板。P1 的 H 必须始终取
    # 高板，而不能固定取“前段”，否则上升台阶会把 P1 错记为低板高度。
    high_after: bool = False


@dataclass
class FloorSegment:
    """一个测量点。``length_to_prev`` 对第 1 段即"到车厢前壁的距离"。"""

    index: int
    point_type: str
    height_to_ground: float
    length_to_prev: float
    # 测量点超出有效底板剖面覆盖范围时，H 只能沿用最近边界值。
    # 显式标记，避免 P3 看起来像真实后壁实测值。
    height_extrapolated: bool = False

    def to_dict(self) -> Dict[str, Any]:
        """转换为对外结果中的单个底板测量点。

        Returns:
            包含点序号、类型、参考面高度、前段长度及外推标记的字典。
        """
        return {
            "index": self.index,
            "pointType": self.point_type,
            "heightToReference": round(self.height_to_ground, 3),
            "lengthToPrev": round(self.length_to_prev, 3),
            "heightExtrapolated": self.height_extrapolated,
        }


@dataclass
class FloorProfile:
    """底板四种结构分类、测量点及高度参考面的完整结果。"""

    case: int
    # 导出用的地平面，**源坐标系**（与输入 PCD 同系，可回溯）。
    # 计算时用的是变换到局部坐标后的平面，但那个系的原点是车辆点云中位数，
    # 单独导出没有意义 —— 曾误导出局部系平面，d 系数与源坐标不符。
    ground_plane: Optional[np.ndarray]
    ground_plane_source: str  # 地平面来源：weighbridge_calibration / scene_ransac / floor_fallback
    ground_plane_frame: str = "source"   # 坐标系：source / local（直接调用未传源系平面）
    segments: List[FloorSegment] = field(default_factory=list)
    step_angle_deg: Optional[float] = None
    confidence: str = "high"
    warnings: List[str] = field(default_factory=list)
    # 调试/可视化用：沿车长的底板高度剖面
    profile_s: Optional[np.ndarray] = None
    profile_h: Optional[np.ndarray] = None
    # 前内壁处底板到参考面的高度；与台阶高点P1及全剖面最低值分别计算。
    front_height_to_ground: Optional[float] = None
    # 台阶或斜接两侧实测平台的代表高度范围。三段测量点可能全位于高侧。
    step_platform_height_range: Optional[Tuple[float, float]] = None

    @property
    def case_name(self) -> str:
        """返回底板情况的稳定英文标识。"""
        return _CASE_NAMES.get(self.case, "unknown")

    @property
    def measured_height_range(self) -> Optional[Tuple[float, float]]:
        """整体坡面有效观测分箱的高度极值，不是三段等效阶梯的极值。

        三段坡面测量点按既定规格读取 s=0、L/3、2L/3，不能代表后端或
        非线性坡面的中间极值。独立使用全部有效底板分箱，既保留三段 H/L
        的业务含义，也不外推未观测的后端。旧对象没有剖面时返回 None，
        平板和台阶仍采用原有代表高度口径。
        """
        if (
            self.case != CASE_FULL_SLOPE
            or self.profile_s is None
            or self.profile_h is None
        ):
            return None
        samples_s = np.asarray(self.profile_s, dtype=float)
        samples_h = np.asarray(self.profile_h, dtype=float)
        if (
            samples_s.ndim != 1
            or samples_h.ndim != 1
            or len(samples_s) < 4
            or samples_s.shape != samples_h.shape
            or not np.isfinite(samples_s).all()
            or not np.isfinite(samples_h).all()
            or not np.all(np.diff(samples_s) > 0.0)
        ):
            return None
        return float(samples_h.min()), float(samples_h.max())

    def to_dict(self) -> Dict[str, Any]:
        """导出可回溯参考面与三段测量结果。

        Returns:
            供 JSON 结果和空车档案持久化使用的字典。
        """
        return {
            "case": self.case,
            "caseName": self.case_name,
            "heightReference": {
                "type": {
                    "weighbridge_calibration": "weighbridge",
                    "scene_ransac": "scene_ground",
                    "floor_fallback": "relative_floor",
                }.get(self.ground_plane_source, "unknown"),
                "source": self.ground_plane_source,
                "frame": self.ground_plane_frame,
            },
            "referencePlane": (
                [round(float(v), 6) for v in self.ground_plane]
                if self.ground_plane is not None else None
            ),
            "stepAngleDeg": (
                round(self.step_angle_deg, 2) if self.step_angle_deg is not None else None
            ),
            "segments": [s.to_dict() for s in self.segments],
            "confidence": self.confidence,
            "warnings": list(self.warnings),
        }


class FloorProfileAnalyzer:
    """从车厢点云提取底板剖面并判别四种情况。"""

    def __init__(self, config: Config):
        """初始化底板剖面分箱、平滑和情况判别阈值。

        Args:
            config: 底板轮廓算法与车厢最少点数配置。
        """
        self.config = config

    def analyze(
        self,
        carriage_points: np.ndarray,
        interior_w_range: Tuple[float, float],
        front_l: float,
        rear_l: float,
        ground_plane_local: Optional[np.ndarray] = None,
        confidence: str = "high",
        ground_plane_world: Optional[np.ndarray] = None,
        ground_plane_source: Optional[str] = None,
        longitudinal_axis: Optional[Tuple[float, float, float]] = None,
    ) -> Optional[FloorProfile]:
        """
        Args:
            carriage_points: (N,3) 车厢点云（局部 W/L/H 坐标）。
            interior_w_range: 内部区域的 W 区间，用于剔除两侧壁。
            front_l: 车厢前壁的 L 坐标。
            rear_l: 车厢后壁的 L 坐标。
            ground_plane_local: 局部坐标下的地平面 [a,b,c,d]，法向朝上，用于算高度；
                None 时退化为以最低底板面为基准。
            confidence: 车厢剖面的质量评级，直接透传。
            ground_plane_world: 同一平面在源坐标系下的系数，仅用于导出回溯。
            ground_plane_source: 平面来源；生产链应传 ``weighbridge_calibration``。
            longitudinal_axis: 与内长、拉筋共用的纵向投影轴；None 保持局部 L。
        Returns:
            FloorProfile；车厢内部点不足以构建剖面时返回 None。
        """
        cfg = self.config
        w_lo, w_hi = interior_w_range
        width = carriage_points[:, 0]
        interior = carriage_points[(width > w_lo) & (width < w_hi)]
        if len(interior) < cfg.carriage_interior_min_points:
            logger.warning("车厢内部点不足，跳过底板剖面分析")
            return None

        # s 轴：前壁为 0，向车厢后方增大。前壁在 L 大的一端（车头侧）。
        sign = 1.0 if rear_l < front_l else -1.0
        longitudinal = (interior[:, 1] if longitudinal_axis is None
                        else interior @ np.asarray(longitudinal_axis))
        s_all = (front_l - longitudinal) * sign
        carriage_length = abs(front_l - rear_l)
        if carriage_length < 1.0:
            logger.warning("车厢长度仅 %.2fm，跳过底板剖面分析", carriage_length)
            return None

        plane = ground_plane_local
        source = (
            ground_plane_source
            if plane is not None and ground_plane_source
            else ("scene_ransac" if plane is not None else "floor_fallback")
        )
        warnings: List[str] = []
        if plane is None:
            warnings.append("未取得真实地平面，底板高度以车厢最低底板面为基准，仅供参考")

        profile_s, profile_h = self.extract_profile(
            interior, s_all, carriage_length, plane
        )
        if len(profile_s) < 4:
            logger.warning("底板剖面有效分箱不足，跳过分析")
            return None

        case, feature, angle = self._classify(profile_s, profile_h, carriage_length)
        segments = self._build_segments(
            case, profile_s, profile_h, carriage_length, feature
        )
        if any(
            segment.index == 3 and segment.height_extrapolated
            for segment in segments
        ):
            warnings.append(
                "P3 后壁超出有效底板剖面覆盖范围，"
                "heightToReference 使用最近边界值外推"
            )
        logger.info(
            "底板剖面：%s（极差 %.3fm%s）",
            _CASE_NAMES[case],
            float(profile_h.max() - profile_h.min()),
            f"，落差角 {angle:.1f}°" if angle is not None else "",
        )
        return FloorProfile(
            case=case,
            ground_plane=ground_plane_world if ground_plane_world is not None else plane,
            ground_plane_source=source,
            ground_plane_frame="source" if ground_plane_world is not None else "local",
            segments=segments,
            step_angle_deg=angle,
            confidence=confidence,
            warnings=warnings,
            profile_s=profile_s,
            profile_h=profile_h,
            front_height_to_ground=self._height_at(profile_s, profile_h, 0.0),
            step_platform_height_range=feature.platform_height_range if feature else None,
        )

    # ------------------------------------------------------------------
    def extract_profile(
        self,
        interior: np.ndarray,
        s_all: np.ndarray,
        carriage_length: float,
        plane: Optional[np.ndarray],
    ) -> Tuple[np.ndarray, np.ndarray]:
        """沿车长逐 bin 取底板面高度。

        先按局部面朝向排除竖直壁面，再在每个 bin 内用密度峰层定位底板面，
        避免端墙上的密集横带被当成底板（低分位会咬在点层下沿使高度偏大），
        再取贴近该层的点算它们到地平面的平均有符号距离作为该 bin 的 H。
        """
        cfg = self.config
        bin_width = cfg.floor_profile_bin
        # 宽度裁切仍会留下端墙和台阶立面。仅有高度密度峰不能证明它是底板；
        # 在既有剖面分箱尺度内判断面朝向，保留法向以 H 分量为主的底面/坡面。
        # 端墙的密集波纹带是有厚度的点团；只看协方差最小轴会把它误认成
        # 水平底面。沿用判空细格等权拟合局部面，再以既有面残差确认二维支持。
        grid = np.floor(interior / (cfg.carriage_grid_cell / 4)).astype(np.int64)
        _, representatives, inverse = unique_cells(grid, return_index=True, return_inverse=True)
        balanced = interior[representatives]
        neighborhoods = cKDTree(balanced).query_ball_point(balanced, bin_width, workers=-1)
        surface = np.zeros(len(balanced), dtype=bool)
        normals = np.zeros_like(balanced)
        centers = np.zeros_like(balanced)
        # 同长度邻域批量拟合，无须补齐或截断邻点。批次只限制临时矩阵的大小。
        by_size: dict[int, list[int]] = {}
        for i, neighbors in enumerate(neighborhoods):
            if len(neighbors) >= 3:
                by_size.setdefault(len(neighbors), []).append(i)
        for size, indices in by_size.items():
            batch_size = max(1, 65536 // size)
            for start in range(0, len(indices), batch_size):
                rows = np.asarray(indices[start:start + batch_size])
                chunks = balanced[np.stack(neighborhoods[rows])]
                centers[rows] = chunks.mean(axis=1)
                centered = chunks - centers[rows, None, :]
                _, singular, axes = np.linalg.svd(centered, full_matrices=False)
                normal = axes[:, -1, :]
                normals[rows] = normal
                horizontal = (singular[:, 1] != 0) & (np.argmax(np.abs(normal), axis=1) == 2)
                if horizontal.any():
                    errors = np.abs(centered[horizontal] @ normal[horizontal, :, None])
                    surface[rows[horizontal]] = np.percentile(errors[:, :, 0], 90, axis=1) <= FACE_TOLERANCE_M
        # 几何判定等权，测高仍使用原始点；不通过抽稀改变原分箱点数和密度峰。
        residual = np.abs(np.einsum('ij,ij->i', interior - centers[inverse], normals[inverse]))
        floor_mask = surface[inverse] & (residual <= FACE_TOLERANCE_M)
        interior = interior[floor_mask]
        s_all = s_all[floor_mask]
        n_bins = max(4, int(np.ceil(carriage_length / bin_width)))
        index = np.clip((s_all / bin_width).astype(int), 0, n_bins - 1)

        s_list: List[float] = []
        h_list: List[float] = []
        for b in range(n_bins):
            mask = index == b
            if mask.sum() < cfg.floor_profile_min_points:
                continue
            chunk = interior[mask]
            level = floor_surface(chunk[:, 2])
            near = chunk[np.abs(chunk[:, 2] - level) <= cfg.floor_layer_tolerance]
            if len(near) < cfg.floor_profile_min_points:
                near = chunk[chunk[:, 2] <= level + cfg.floor_layer_tolerance]
            if len(near) == 0:
                continue
            if plane is not None:
                height = float(np.mean(near @ plane[:3] + plane[3]))
            else:
                height = float(np.mean(near[:, 2]))
            s_list.append((b + 0.5) * bin_width)
            h_list.append(height)

        s_arr = np.asarray(s_list)
        h_arr = np.asarray(h_list)
        if len(h_arr) and plane is None:
            # 无地平面时统一平移，使最低底板面为 0，避免输出带上任意的坐标原点
            h_arr = h_arr - h_arr.min()
        return s_arr, h_arr

    def _classify(
        self, s: np.ndarray, h: np.ndarray, carriage_length: float
    ) -> Tuple[int, Optional[_StepFeature], Optional[float]]:
        """判别四种情况。

        Returns:
            (case, 落差特征, 落差角度)
        """
        cfg = self.config
        h_high = float(np.percentile(h, 95))
        h_low = float(np.percentile(h, 5))
        drop = h_high - h_low
        if drop < cfg.floor_flat_tolerance:
            return CASE_FLAT, None, None

        # 极差超阈值不代表就是坡：局部不平和测距噪声同样会把极差顶上去。
        # 实测两份平底板空车极差 5~6cm 但剖面非单调（0.038→0.008→0.027），
        # 若只看极差会误判成整体坡度。用线性趋势区分"系统性倾斜"与"起伏"。
        slope_coef, intercept = np.polyfit(s, h, 1)
        trend = abs(float(slope_coef)) * carriage_length
        # 还要求剖面确实"像一条直线"：下沉/起伏的底板同样能拟合出非零斜率，
        # 但残差很大。实测一份平底板空车 H 呈 0.052→0.001→0.035 的 V 形，
        # 只看趋势仍会误判成坡度，用决定系数 R² 把它挡掉。
        residual = h - (slope_coef * s + intercept)
        variance = float(np.var(h))
        r_squared = (
            1.0 - float(np.var(residual)) / variance if variance > 1e-12 else 0.0
        )

        # 落差区间：剖面从"接近高位"跌到"接近低位"所跨的 s 范围。
        # 用 10%/90% 分位带定界，对单点噪声不敏感。
        hi_level = h_low + 0.9 * drop
        lo_level = h_low + 0.1 * drop
        hi_idx = np.flatnonzero(h >= hi_level)
        lo_idx = np.flatnonzero(h <= lo_level)
        if len(hi_idx) == 0 or len(lo_idx) == 0:
            return self._non_step_case(trend, r_squared, h), None, None

        # 前段高、后段低（下坡）时 hi 在小 s 侧；反之亦然。两种都支持。
        high_after = float(np.mean(s[hi_idx])) > float(np.mean(s[lo_idx]))
        if not high_after:
            feature_front = float(s[hi_idx].max())
            feature_rear = float(s[lo_idx].min())
        else:
            feature_front = float(s[lo_idx].max())
            feature_rear = float(s[hi_idx].min())
        span = abs(feature_rear - feature_front)

        # 落差区间过宽 → 不是台阶而是整体坡度
        if span > cfg.floor_step_span_max_ratio * carriage_length:
            return self._non_step_case(trend, r_squared, h), None, None

        # 落差区间之外的两段各自要近水平，否则仍属整体坡度
        lo_s, hi_s = min(feature_front, feature_rear), max(feature_front, feature_rear)
        before = h[s <= lo_s]
        after = h[s >= hi_s]
        for part in (before, after):
            if len(part) >= 2 and float(part.max() - part.min()) > cfg.floor_plateau_tolerance:
                return self._non_step_case(trend, r_squared, h), None, None

        # 落差区间内应当集中了绝大部分高度变化
        inside = h[(s >= lo_s) & (s <= hi_s)]
        if len(inside) >= 2:
            captured = float(inside.max() - inside.min())
            if captured < cfg.floor_step_drop_min_ratio * drop:
                return self._non_step_case(trend, r_squared, h), None, None

        # 分箱量化下限：真正垂直的台阶最多只落在一个 bin 内，此时 span 就等于
        # bin 宽，直接按 atan(drop/span) 算会得到 60~70° 而误判成坡道（实测
        # 0.5m 落差 + 0.2m 分箱 → 68°）。扣掉一个 bin 宽还原真实倾角。
        span_eff = max(span - cfg.floor_profile_bin, 0.0)
        if span_eff <= 1e-9:
            # 垂直落差：真实台阶位于两个分箱之间，取中点作为测量点
            angle = 90.0
            front_s = lo_s
            rear_s = (feature_front + feature_rear) / 2.0
        else:
            angle = float(np.degrees(np.arctan2(drop, span_eff)))
            # 10%/90% 分位带只覆盖 80% 的落差，两端各外推 12.5% 还原坡道真实起止
            extension = span * 0.125
            front_s = lo_s - extension
            rear_s = hi_s + extension

        # 两个平台都由分类时已观测的分箱支撑，取各自中位数。
        front_height = float(np.median(before))
        rear_height = float(np.median(after))
        feature = _StepFeature(
            front_s, rear_s,
            (min(front_height, rear_height), max(front_height, rear_height)),
            high_after=high_after,
        )
        case = CASE_STEP_90 if angle >= cfg.floor_step_angle_min else CASE_STEP_RAMP
        return case, feature, angle

    def _non_step_case(
        self, trend: float, r_squared: float, heights: np.ndarray
    ) -> int:
        """不构成台阶时，区分整体坡度与"平整但有起伏"。

        线性坡仍用趋势 + R² 判定；弧形、折线形等非线性但连续的整体变化不能
        因 R² 偏低就回落成 flat。对剖面平滑后检查“宽幅变化 + 很少转向”：
        U/V 形或单调曲面会保留为整体坡度，锯齿测距噪声则因转向过多仍判平整。
        """
        cfg = self.config
        if trend >= cfg.floor_flat_tolerance and r_squared >= cfg.floor_slope_r2_min:
            return CASE_FULL_SLOPE

        smooth = gaussian_filter1d(np.asarray(heights, dtype=float), sigma=1.5)
        smooth_span = float(np.percentile(smooth, 95) - np.percentile(smooth, 5))
        delta = np.diff(smooth)
        # 忽略单 bin 小于总平整容差 1/10 的抖动，避免噪声制造大量虚假转向。
        meaningful = delta[np.abs(delta) >= cfg.floor_flat_tolerance / 10.0]
        if len(meaningful) <= 1:
            direction_changes = 0
        else:
            direction_changes = int(
                np.count_nonzero(np.sign(meaningful[1:]) != np.sign(meaningful[:-1]))
            )
        if smooth_span >= cfg.floor_flat_tolerance and direction_changes <= 2:
            return CASE_FULL_SLOPE
        return CASE_FLAT

    def _height_at(self, s: np.ndarray, h: np.ndarray, target: float) -> float:
        """剖面在给定 s 处的高度（线性插值，越界取端点）。"""
        return float(np.interp(target, s, h))

    def _height_at_with_extrapolation(
        self, s: np.ndarray, h: np.ndarray, target: float
    ) -> Tuple[float, bool]:
        """读取 H，并区分正常的半个分箱边界与真正缺测外推。"""
        covered_lo = float(s[0]) - self.config.floor_profile_bin / 2.0
        covered_hi = float(s[-1]) + self.config.floor_profile_bin / 2.0
        tolerance = 1e-9
        extrapolated = (
            target < covered_lo - tolerance or target > covered_hi + tolerance
        )
        return self._height_at(s, h, target), extrapolated

    def _build_segments(
        self,
        case: int,
        s: np.ndarray,
        h: np.ndarray,
        carriage_length: float,
        feature: Optional[_StepFeature],
    ) -> List[FloorSegment]:
        """按识别情况生成统一“三段 H/L”业务测量点。

        Args:
            case: 四种底板情况之一。
            s: 从车厢前壁向后的剖面距离。
            h: 对应参考面高度。
            carriage_length: 前后内壁间水平长度。
            feature: 台阶或坡道的边界；平板和全坡时为空。

        Returns:
            按前到后排列的一个或三个底板测量点。
        """
        if case == CASE_FLAT:
            return [FloorSegment(1, "flat", float(np.median(h)), carriage_length)]

        if case == CASE_FULL_SLOPE:
            # 按长度等分 3 段；每段以坡面为斜边构建直角三角形，水平边取该段前
            # 边界的高度向后延伸、垂直边落在该段后边界，90° 顶点即测量点。
            bounds = [carriage_length * i / 3.0 for i in range(4)]
            segments: List[FloorSegment] = []
            for i in range(3):
                segments.append(FloorSegment(
                    index=i + 1,
                    point_type="slope_vertex",
                    height_to_ground=self._height_at(s, h, bounds[i]),
                    length_to_prev=bounds[i + 1] - bounds[i],
                ))
            return segments

        # 情况 2 / 3：第 1 段由落差位置决定，剩余部分平均分成 2 段
        step_s = float(np.clip(feature.rear_s, 0.0, carriage_length))
        # P1 是“台阶高点”：下降台阶取前侧高板，上升台阶取后侧高板。固定取
        # 前段会把上升台阶的 H1 错写成低板高度。
        high_region = (
            h[s >= feature.rear_s]
            if feature.high_after
            else h[s <= feature.front_s]
        )
        high_target = feature.rear_s if feature.high_after else feature.front_s
        h1 = (
            float(np.median(high_region))
            if len(high_region)
            else self._height_at(s, h, high_target)
        )
        mid_s = (step_s + carriage_length) / 2.0
        point_type = "step_top" if case == CASE_STEP_90 else "ramp_end"
        h3, h3_extrapolated = self._height_at_with_extrapolation(
            s, h, carriage_length
        )
        return [
            FloorSegment(1, point_type, h1, step_s),
            FloorSegment(2, "mid_rear", self._height_at(s, h, mid_s), mid_s - step_s),
            FloorSegment(
                3,
                "rear_wall",
                h3,
                carriage_length - mid_s,
                height_extrapolated=h3_extrapolated,
            ),
        ]
