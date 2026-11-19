from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import open3d as o3d
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks
from scipy.spatial import cKDTree

from carpointcloud_analyzer.domain.carriage import CarriageAnalyzer, floor_surface
from carpointcloud_analyzer.domain.utils import occupancy_extent
from carpointcloud_analyzer.domain.utils.planes import consolidate_plane, deterministic_plane_hypothesis
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.heading import cab_end_support, longitudinally_continuous
from carpointcloud_analyzer.domain.runtime import logger
from carpointcloud_analyzer.domain.models import BoxCarriageBox, CarriageSplitResult, RibInfo
from carpointcloud_analyzer.domain.inner_faces import measure_end_faces, supported_faces
from carpointcloud_analyzer.domain.rib_support import validated_rib_support


@dataclass
class _InteriorView:
    """底板以上的拉筋搜索点与同一次估计的底板参考。"""

    above_floor: np.ndarray   # 底板面以上的点，拉筋候选只能来自这里
    floor_level: float


@dataclass
class CoordinateTransform:
    """Rigid transform from source XYZ to vehicle-local W/L/H coordinates.

    建系是唯一的算法路径，因此这个变换要么成立、要么 ``process()`` 直接抛错，
    不存在“未应用”的中间状态。
    """

    origin: np.ndarray
    rotation: np.ndarray
    confidence: float = 1.0
    # H 轴与外部竖直基准的余弦（1.0 表示完全对齐）；无基准时为 None。
    vertical_alignment: Optional[float] = None
    # 竖直基准来源：weighbridge / site_frame / scene_ground / none。
    vertical_source: str = "none"

    def to_local(self, points: np.ndarray) -> np.ndarray:
        """把源 XYZ 点转换到车辆 W/L/H 局部坐标。

        Args:
            points: 源坐标点数组。

        Returns:
            应用平移和正交旋转后的局部坐标。
        """
        return (points - self.origin) @ self.rotation

    def to_source(self, points: np.ndarray) -> np.ndarray:
        """把车辆局部点恢复到输入 PCD 的源坐标。

        Args:
            points: 局部 W/L/H 点数组。

        Returns:
            源 XYZ 坐标数组。
        """
        return points @ self.rotation.T + self.origin

    def plane_to_local(self, plane: np.ndarray) -> np.ndarray:
        """把源坐标系的平面 [a,b,c,d] 变换到局部坐标。

        局部坐标定义为 ``p_local = Rᵀ(p − origin)``，故 ``p = R·p_local + origin``。
        代入 ``nᵀp + d = 0`` 得 ``(Rᵀn)ᵀ·p_local + (nᵀ·origin + d) = 0``。
        R 正交，因此点到平面的有符号距离在两个坐标系下相同。
        """
        plane = np.asarray(plane, dtype=float)
        normal_local = self.rotation.T @ plane[:3]
        offset_local = float(plane[:3] @ self.origin + plane[3])
        return np.concatenate([normal_local, [offset_local]])

    def to_dict(self) -> Dict[str, Any]:
        """导出坐标轴在源坐标系中的方向和变换置信度。

        Returns:
            可写入结果 JSON 的坐标变换说明。
        """
        return {
            "confidence": round(float(self.confidence), 3),
            "verticalSource": self.vertical_source,
            "verticalAlignment": (
                round(float(self.vertical_alignment), 6)
                if self.vertical_alignment is not None else None
            ),
            "originSourceXYZ": self.origin.tolist(),
            "widthAxisSourceXYZ": self.rotation[:, 0].tolist(),
            "lengthAxisSourceXYZ": self.rotation[:, 1].tolist(),
            "heightAxisSourceXYZ": self.rotation[:, 2].tolist(),
        }


class PointCloudGeometryNormalizer:
    """在去噪副本上拟合 W/L/H，再转换输入的车辆点。

    PCA 提供与坐标轴无关的初始坐标系：最长的稳健跨度即车长，其余两轴中
    跨度较小者为车高；较窄的一端只作为车头方向的初始假设。正式链路在任何
    依赖前/后的计算之前，都会用已确认车厢端壁以外的实测车体支撑纠正该方向。
    """

    def __init__(self, config: Config):
        """初始化统计滤波、平面拟合和车辆轴向识别配置。

        Args:
            config: 坐标标准化算法参数。
        """
        self.config = config

    def vertical_candidate(
        self, points: np.ndarray, vertical_reference: Optional[np.ndarray], vertical_source: str,
    ) -> Optional[Tuple[np.ndarray, CoordinateTransform]]:
        """仅供矩形端壁验证的确定性候选系，不代替普通车型的底板建系。

        高度直接来自独立竖直基准，不能用车身最大平面猜高度。候选必须再通过
        双端面、等宽、平行和上缘连续性验证才可使用；本方法不确认内壁或空载。
        排序消除输入点序对协方差累加及邻域滤波的影响，不设置全局随机种子。
        """
        vertical = self._unit_vertical(vertical_reference)
        if vertical is None:
            return None
        points = np.asarray(points, dtype=float)
        if (points.ndim != 2 or points.shape[1:] != (3,)
                or len(points) < self.config.pose_min_points
                or not np.isfinite(points).all()):
            return None
        ordered = points[np.lexsort(points.T[::-1])]
        clean = self.remove_outliers(ordered)
        if len(clean) < self.config.pose_min_points:
            return None
        origin = np.median(clean, axis=0)
        centered = clean - origin
        horizontal = centered - np.outer(centered @ vertical, vertical)
        values, vectors = np.linalg.eigh(np.cov(horizontal, rowvar=False))
        if values[-1] <= max(1e-8, values[-2] * 1.5):
            return None
        length = vectors[:, -1]
        length -= vertical * np.dot(length, vertical)
        length /= np.linalg.norm(length)
        if length[np.argmax(np.abs(length))] < 0:
            length *= -1
        width = np.cross(vertical, length)
        transform = CoordinateTransform(origin, np.column_stack((width, length, vertical)),
                                        0.5, 1.0, vertical_source)
        # 去噪副本只用于建系。场景已提取的车辆点是量测输入，不能因边缘
        # 邻域只有半面而再次删掉连续壁顶；保留原点供后续实测面/分箱量测。
        return transform.to_local(ordered), transform

    def process(
        self,
        points: np.ndarray,
        vertical_reference: Optional[np.ndarray] = None,
        vertical_source: str = "none",
    ) -> Tuple[np.ndarray, CoordinateTransform]:
        """用统计去噪副本建立 W/L/H 坐标系，再变换全部车辆量测点。

        Args:
            points: 输入坐标系下形状为 ``(N, 3)`` 的车辆点云。
            vertical_reference: 同一坐标系下朝上的竖直单位向量，来自地磅标定面
                或场地支撑面。它必须独立于车辆自身点云。
            vertical_source: 该基准的来源标识，仅用于审计与日志。

        Returns:
            输入车辆点的局部坐标及可逆变换；建系删点不改变量测点集。

        Raises:
            ValueError: 点数不足、缺少必需的竖直基准，或主平面法向偏离竖直基准
                超过 ``geometry_vertical_max_deviation_deg``。
            RuntimeError: Open3D 平面拟合无法建立有效车辆高度轴时向上传播。

        Notes:
            高度轴**不能**只取"内点最多的那个平面"的法向。侧壁比底板密时最大
            平面就是侧壁：实测底板 12000 点 / 单侧壁 40000 点时，H 轴偏离真实
            竖直 89.99°，车宽与车厢高整体互换，而姿态置信度仍是 1.00 —— 下游
            没有任何判据能发现这次测量是错的。因此这里用外部竖直基准同时做两件
            事：卡住法向偏差，并统一 H 轴朝上（Open3D 返回的平面法向符号是任意
            的，靠它决定上下等于赌一半概率）。
        """
        cfg = self.config
        clean = self.remove_outliers(points)
        if len(clean) < cfg.pose_min_points:
            raise ValueError(
                f"去噪后仅剩 {len(clean)} 点，少于建立车辆局部坐标系所需的 "
                f"{cfg.pose_min_points} 点，拒绝继续测量"
            )

        vertical = self._unit_vertical(vertical_reference)
        if vertical is None:
            if cfg.geometry_require_vertical_reference:
                raise ValueError(
                    "缺少可信竖直基准（地磅标定面或场地支撑面），无法确认高度轴；"
                    "拒绝继续测量。请开启场地调平，或显式关闭 "
                    "geometry_require_vertical_reference 承担宽高互换风险"
                )
            logger.warning(
                "无竖直基准，高度轴仅由最大平面法向决定；侧壁比底板密时会导致"
                "宽高互换，本次姿态置信度按未验证处理"
            )

        origin = np.median(clean, axis=0)
        centered = clean - origin
        plane, inliers, alignment = self._dominant_horizontal_plane(
            clean, vertical, vertical_source
        )
        e_h = np.asarray(plane[:3], dtype=float)
        e_h /= np.linalg.norm(e_h)

        # 只在平面内做 PCA，得到纵向（L 轴）的初始方向。
        horizontal = centered - np.outer(centered @ e_h, e_h)
        covariance = np.cov(horizontal, rowvar=False)
        values, vectors = np.linalg.eigh(covariance)
        candidates = vectors[:, np.argsort(values)[::-1][:2]]
        candidate_spans = []
        for i in range(2):
            projected = centered @ candidates[:, i]
            candidate_spans.append(np.percentile(projected, 99) - np.percentile(projected, 1))
        e_l = candidates[:, int(np.argmax(candidate_spans))]
        e_l -= e_h * np.dot(e_l, e_h)
        e_l /= np.linalg.norm(e_l)

        # 保持既有 W/L/H 正交轴约定，列序的行列式为 -1。
        # 该矩阵用于坐标投影，不能直接当作 SO(3) 姿态输出四元数。
        e_w = np.cross(e_h, e_l)
        e_w /= np.linalg.norm(e_w)
        e_h = np.cross(e_l, e_w)
        e_h /= np.linalg.norm(e_h)

        rotation = np.column_stack((e_w, e_l, e_h))
        local = centered @ rotation

        # 用最强横向平面（通常是挂车前壁）的法向修正 L 轴：PCA 残留的
        # 微小偏航误差会把窄拉筋峰在几厘米范围内抹平，修正后峰才立得住。
        l_values = local[:, 1]
        bin_width = 0.05
        bins = np.arange(l_values.min(), l_values.max() + bin_width * 1.5, bin_width)
        l_hist, l_edges = np.histogram(l_values, bins)
        if len(l_hist):
            strongest = int(np.argmax(gaussian_filter1d(l_hist.astype(float), 1.0)))
            plane_l = (l_edges[strongest] + l_edges[strongest + 1]) / 2
            h_low, h_high = np.percentile(local[:, 2], [10, 90])
            plane_mask = (
                (np.abs(l_values - plane_l) <= 0.08)
                & (local[:, 2] > h_low)
                & (local[:, 2] < h_high)
            )
            plane_points = centered[plane_mask]
            if len(plane_points) >= 100:
                plane_cov = np.cov(plane_points, rowvar=False)
                _, plane_vectors = np.linalg.eigh(plane_cov)
                refined_l = plane_vectors[:, 0]
                refined_l -= e_h * np.dot(refined_l, e_h)
                if np.linalg.norm(refined_l) > 1e-6:
                    refined_l /= np.linalg.norm(refined_l)
                    if np.dot(refined_l, e_l) < 0:
                        refined_l *= -1
                    e_l = refined_l
                    e_w = np.cross(e_h, e_l)
                    e_w /= np.linalg.norm(e_w)
                    e_h = np.cross(e_l, e_w)
                    e_h /= np.linalg.norm(e_h)
                    rotation = np.column_stack((e_w, e_l, e_h))
                    local = centered @ rotation

        # 初始方向使用端点宽度而非截面积，避免高度抵消宽度差异。
        # 这不是“车头必比车尾窄”的保证；宽车头/残缺尾壁会使其失效，
        # pipeline 分割后还须用实测厢外车体校正，再固定 +L 指向车头。
        # 端点范围必须用 0.1/99.9 分位而非 1/99：俯视雷达下车头常只剩一块
        # 低矮顶面（实测仅占全车点数的 1.3%），1% 分位会把整个车头切掉，
        # 使两端判据变成"车厢 vs 车厢"的无效比较，车头端永远判不出来。
        ratio = self.config.pose_endpoint_ratio
        l_lo, l_hi = np.percentile(local[:, 1], [0.1, 99.9])
        band = max((l_hi - l_lo) * ratio, 1e-6)
        low = local[local[:, 1] <= l_lo + band]
        high = local[local[:, 1] >= l_hi - band]
        low_width = self._cross_section_width(low)
        high_width = self._cross_section_width(high)
        if low_width < high_width:
            rotation[:, 1] *= -1
            rotation[:, 0] *= -1  # 保持右手系
            local = centered @ rotation

        local_spans = np.percentile(local, 99, axis=0) - np.percentile(local, 1, axis=0)
        confidence = self._confidence(local_spans)
        plane_support = len(inliers) / len(clean)
        confidence = float(np.clip(0.7 * confidence + 0.3 * min(1.0, plane_support / 0.15), 0, 1))
        if vertical is None:
            # 姿态未经独立基准验证时不允许报满分，否则下游无法把"轴向可能整体
            # 搞错"和"轴向已核对"区分开。
            confidence = min(confidence, 0.5)
        transform = CoordinateTransform(
            origin,
            rotation,
            confidence,
            vertical_alignment=alignment,
            vertical_source=vertical_source if vertical is not None else "none",
        )
        logger.info(
            "自动坐标标准化完成：W/L/H跨度≈%.2f/%.2f/%.2fm，置信度=%.2f",
            np.ptp(np.percentile(local, [1, 99], axis=0), axis=0)[0],
            np.ptp(np.percentile(local, [1, 99], axis=0), axis=0)[1],
            np.ptp(np.percentile(local, [1, 99], axis=0), axis=0)[2],
            confidence,
        )
        return transform.to_local(points), transform

    def remove_outliers(self, points: np.ndarray) -> np.ndarray:
        """按配置执行 Open3D 统计离群点过滤；建系与整车外包络共用同一份去噪口径。

        Args:
            points: 原始点云。

        Returns:
            过滤后的独立坐标数组；禁用或点数不足时原样返回。
        """
        if not self.config.statistical_outlier_enabled or len(points) < 3:
            return points
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        filtered, _ = cloud.remove_statistical_outlier(
            nb_neighbors=min(self.config.statistical_outlier_neighbors, len(points) - 1),
            std_ratio=self.config.statistical_outlier_std_ratio,
        )
        clean = np.asarray(filtered.points).copy()
        logger.info("统计离群点滤波：保留 %s / %s 点", f"{len(clean):,}", f"{len(points):,}")
        return clean

    def _dominant_horizontal_plane(
        self,
        clean: np.ndarray,
        vertical: Optional[np.ndarray],
        vertical_source: str,
    ) -> Tuple[np.ndarray, np.ndarray, Optional[float]]:
        """挑一块可以当"底板/车顶"用的主平面，并按竖直基准定向。

        无竖直基准时退化为"取最大平面"（历史行为）。有基准时逐块考察：最大平面
        如果是立面就剔除其内点继续找。平板车/自卸车底盘的最大平面实测是一块偏离
        竖直 89.5° 的挡板，只否决不重找会让这类车全部无法测量。

        Returns:
            ``(平面, 内点下标, 与竖直基准的对齐余弦)``；对齐余弦在无基准时为 None。

        Raises:
            ValueError: 所有候选平面都不是水平面。
        """
        cfg = self.config
        limit = float(np.cos(np.deg2rad(cfg.geometry_vertical_max_deviation_deg)))
        attempts = max(1, cfg.geometry_plane_search_max_attempts) if vertical is not None else 1
        remaining = np.arange(len(clean))
        deviations: List[float] = []

        for attempt in range(attempts):
            if len(remaining) < 3:
                break
            plane, raw_inliers = deterministic_plane_hypothesis(
                clean[remaining], distance_threshold=0.035, num_iterations=600
            )
            # RANSAC 的最小采样假设带随机性，直接拿它建系会让整套局部坐标（进而
            # 所有尺寸）在毫米级抖动。仅重拟合平面还不够 —— 内点集合本身每次也不同，
            # 实测 H 轴仍会抖 0.08°，在 2.58m 车宽上折算成约 1.3mm。交替收敛内点后
            # 才与初始随机假设无关。内点始终相对完整点云收敛，保证第一轮就命中
            # 底板时与历史行为逐位一致。
            plane, inlier_mask = consolidate_plane(
                clean,
                plane,
                remaining[np.asarray(raw_inliers)],
                distance_threshold=0.035,
            )
            inliers = np.flatnonzero(inlier_mask)
            normal = np.asarray(plane[:3], dtype=float)
            normal = normal / np.linalg.norm(normal)

            if vertical is None:
                return np.asarray(plane, dtype=float), inliers, None

            signed = float(normal @ vertical)
            if abs(signed) >= limit:
                if signed < 0:
                    # Open3D 平面法向符号任意。统一朝上，否则底板与车顶互换，
                    # floor_surface 取到的"最低密集层"会变成壁顶。
                    plane = -np.asarray(plane, dtype=float)
                return np.asarray(plane, dtype=float), inliers, abs(signed)

            deviation = float(np.degrees(np.arccos(min(1.0, abs(signed)))))
            deviations.append(deviation)
            logger.info(
                "第 %d 块主平面偏离%s竖直基准 %.1f°（%d 内点），判为立面，继续寻找底板",
                attempt + 1, vertical_source, deviation, len(inliers),
            )
            remaining = remaining[~inlier_mask[remaining]]

        raise ValueError(
            f"连续 {len(deviations)} 块主平面都不是水平面（偏离{vertical_source}竖直基准 "
            + "、".join(f"{d:.1f}°" for d in deviations)
            + "），无法确定车辆高度轴；继续测量会把车宽当成车厢高，拒绝继续测量"
        )

    @staticmethod
    def _unit_vertical(reference: Optional[np.ndarray]) -> Optional[np.ndarray]:
        """把外部竖直基准规范成单位向量；非法或退化时返回 None。"""
        if reference is None:
            return None
        vector = np.asarray(reference, dtype=float).reshape(-1)
        if vector.size == 4:      # 允许直接传平面 [a,b,c,d]
            vector = vector[:3]
        if vector.size != 3 or not np.all(np.isfinite(vector)):
            return None
        norm = float(np.linalg.norm(vector))
        if norm < 1e-9:
            return None
        return vector / norm

    @staticmethod
    def _cross_section_width(points: np.ndarray) -> float:
        """端面在车宽方向的跨度；点数不足时返回 inf（不判定为车头端）。"""
        if len(points) < 10:
            return float("inf")
        low, high = np.percentile(points[:, 0], [5, 95])
        return float(max(high - low, 0.0))

    @staticmethod
    def _confidence(spans: np.ndarray) -> float:
        """根据最长轴相对次长轴的分离程度估计姿态置信度。

        Args:
            spans: 三个候选轴的稳健跨度。

        Returns:
            限制在 ``[0, 1]`` 的置信度。
        """
        ordered = np.sort(spans)[::-1]
        if ordered[1] <= 0:
            return 0.0
        return float(np.clip((ordered[0] / ordered[1] - 1.0) / 3.0, 0.0, 1.0))


class LocalCarriageExtractor:
    """在局部 W/L/H 坐标中识别挂车端面。

    与旧的切分器不同，这里不假设源坐标系的哪个符号方向指向车头：优先选取
    一端之外还有独立车头的实测端壁，再用横向密度对其余端面对排序。
    """

    def __init__(self, config: Config):
        """初始化局部纵向密度端壁提取器。

        Args:
            config: 端壁峰显著性、最小跨度和边缘窗口配置。
        """
        self.config = config

    @staticmethod
    def split_observed_box(points: np.ndarray, box: BoxCarriageBox) -> CarriageSplitResult:
        """优先用双端面实测外包络裁短矩形箱，内壁保持未验证。"""
        rear, front = sorted((box.rear_l, box.front_l))
        tail, head = occupancy_extent(points[:, 1])
        carriage = points[(points[:, 1] >= rear) & (points[:, 1] <= front)]
        if not len(carriage):
            raise ValueError("矩形箱体端面之间没有实测车辆点")
        return CarriageSplitResult(
            carriage_points=carriage, split_y=front, carriage_start_y=rear,
            head_y=head, tail_y=tail, boundaries_are_inner_faces=False,
            boundary_method="observed_rectangular_outer_faces",
            boundary_warning=("短矩形箱体外壁已定位；下部内缘点不足时只输出已测字段，"
                              "外壁跨度不得冒充内长"),
            observed_outer_box=box,
        )

    def split(self, points: np.ndarray) -> CarriageSplitResult:
        """识别车厢前后端壁内表面并提取其间点云。

        Args:
            points: 已标准化到 W/L/H 的整车点云。

        Returns:
            车厢点云、内壁边界、整车端点及边界可信标记。

        Raises:
            ValueError: 无法找到满足车厢几何约束的两块横向端面。
        """
        cfg = self.config
        length = points[:, 1]
        lo, hi = np.percentile(length, [0.1, 99.9])
        step = cfg.rib_bin_width
        edges = np.arange(lo, hi + step * 1.5, step)
        hist, edges = np.histogram(length, edges)
        centers = (edges[:-1] + edges[1:]) / 2
        smooth = gaussian_filter1d(hist.astype(float), sigma=1.5)
        peaks, _ = find_peaks(
            smooth,
            prominence=max(np.std(smooth) * cfg.carriage_wall_prominence_ratio, 1.0),
            distance=max(1, int(cfg.carriage_wall_min_distance / step)),
        )
        # scipy 不会把首/末采样点判为峰，而端壁常常正好压在扫描边界上，
        # 因此两端窗口各自补入最强 bin。
        edge_count = max(1, int(len(smooth) * cfg.carriage_edge_window_ratio))
        edge_peaks = np.array([
            int(np.argmax(smooth[:edge_count])),
            int(len(smooth) - edge_count + np.argmax(smooth[-edge_count:])),
        ])
        peaks = np.unique(np.concatenate((peaks, edge_peaks)))
        if len(peaks) < 2:
            raise ValueError("无法在车辆局部坐标中识别两个车厢端面")

        total = hi - lo
        candidates = []
        for i, left in enumerate(peaks[:-1]):
            for right in peaks[i + 1:]:
                span = centers[right] - centers[left]
                # 至少有一道端壁要贴近整体端点；另一道可能因车头存在而落在车辆内部。
                end_gap = min(centers[left] - lo, hi - centers[right])
                if end_gap <= total * cfg.carriage_end_gap_ratio:
                    strength = smooth[left] + smooth[right]
                    candidates.append((span, strength, left, right))
        if not candidates:
            raise ValueError("局部坐标密度峰不满足车厢端面几何约束")
        # 驾驶室前脸也可能比车厢端壁更密。先用实测端面及单端外部车体区分
        # 车厢与整车包络；没有独立车头证据时，保留原密度排序和缺测处理。
        min_span = total * cfg.carriage_min_span_ratio
        # 端壁内表面都未验证时，按原排序取第一对之间车体连续的壁芯端壁，
        # 不让跨过车头空隙的端壁对决定车厢范围。
        continuous_cores = None
        density_choice = None
        short_body_verified = False
        for span, _, left, right in sorted(
                candidates, key=lambda item: (item[1], item[0]), reverse=True):
            rear_core = self._wall_core_center(hist, centers, left)
            front_core = self._wall_core_center(hist, centers, right)
            faces = measure_end_faces(points, rear_core, front_core)
            if span >= min_span and density_choice is None:
                density_choice = rear_core, front_core, faces
            if span >= min_span and continuous_cores is None and longitudinally_continuous(
                    points, rear_core, front_core):
                continuous_cores = rear_core, front_core
            if faces is None:
                continue
            rear_face, front_face, w_ref, h_ref = faces
            rear, front = rear_face.at(w_ref, h_ref), front_face.at(w_ref, h_ref)
            candidate = CarriageSplitResult(
                carriage_points=points[(length >= rear) & (length <= front)],
                split_y=front, carriage_start_y=rear, head_y=hi, tail_y=lo,
            )
            negative_cab, positive_cab = cab_end_support(points, candidate)
            if negative_cab != positive_cab:
                # 底盘的水平连接不能代替两端之间的车厢壁体，车头顶也不是厢底。
                # 复用同一二维面支持和连续占用规则，识别有底、壁体连贯的开口车厢。
                body = candidate.carriage_points
                if not longitudinally_continuous(body[body[:, 2] >= h_ref], rear, front):
                    continue
                # 长底盘不能把已实测的短车厢拉到驾驶室。比例仅用于缺少几何证据
                # 的密度回退；短车厢须由两端二维壁面、连续车体及单端外部车体证明。
                if front - rear < min_span:
                    lower = body[body[:, 2] < h_ref]
                    if not supported_faces(lower, 2):
                        continue
                    profile = CarriageAnalyzer(cfg).analyze(body)
                    # 台阶立面只封住底部，不能作为短车厢端壁；端壁的上半部
                    # 也须具备原二维面支持，与侧壁主体高度相对应。
                    middle = (profile.floor_z + profile.wall_top) / 2
                    if min(np.max(face.cells[:, 2]) for face in (rear_face, front_face)) <= middle:
                        continue
                    short_body_verified = True
                break
        else:
            if density_choice is None:
                raise ValueError("局部坐标密度峰不满足车厢端面几何约束")
            rear_core, front_core, faces = density_choice
        # 局部 +L 指向车头：低 L 为车厢后壁，高 L 为车厢前壁。车厢可用长度、
        # 拉筋首尾段和底板剖面必须共用两块壁的“内表面”，不能一端取壁芯、
        # 另一端取内表面，否则会把一侧壁厚稳定地算进长度。
        rear_inner = front_inner = None
        if faces is not None:
            rear_face, front_face, w_ref, h_ref = faces
            rear_inner = rear_face.at(w_ref, h_ref)
            front_inner = front_face.at(w_ref, h_ref)
        # 峰仅用于搜索。没有实测二维面支持时保持未验证，不再把平滑展宽的
        # 半高交点当作内壁；不得将体积散点/横梁截出的薄片伪装成内壁。
        inner_faces_valid = bool(
            rear_inner is not None
            and front_inner is not None
            and (front_inner - rear_inner >= min_span or short_body_verified)
        )
        boundary_warning = None
        continuous_body_range = False
        if not inner_faces_valid:
            measured_span = (
                front_inner - rear_inner
                if rear_inner is not None and front_inner is not None
                else float("nan")
            )
            boundary_warning = (
                "端壁内表面不可可靠定位，当前边界仅用于空载判断；"
                "不得上传内部长度或拉筋间距"
            )
            if continuous_cores is not None:
                # 内长取主体侧壁长度，不依赖端壁内表面；车头距与拉筋间距仍不可用。
                rear_core, front_core = continuous_cores
                continuous_body_range = True
                boundary_warning = (
                    "端壁内表面不可可靠定位，车厢范围取车体连续的端壁壁芯；"
                    "内长按主体侧壁量取，不得上传车头距或拉筋间距"
                )
            logger.warning(
                "两端内表面跨度=%s（要求 ≥ %.2fm），"
                "临时退回壁芯中心 [%.2f, %.2f]m",
                f"{measured_span:.2f}m" if np.isfinite(measured_span) else "不可测",
                min_span,
                rear_core,
                front_core,
            )
            rear_inner, front_inner = rear_core, front_core
        mask = (length >= rear_inner) & (length <= front_inner)
        carriage = points[mask]
        # 车头/车尾端点必须用占用跨度，不能用分位数：车头沿纵向的点密度远低于
        # 车厢，99.5 分位会切进车头内部（实测切掉 1.235m），使 cabToFrontWall
        # 偏小、且与同样描述整车纵向范围的 vehicle_length 相差 1.3m。
        # 与 DimensionCalculator 的整车长同源，保证两处口径一致。
        tail_y, head_y = occupancy_extent(length)
        logger.info(
            "局部坐标车厢内壁：后壁=%.3fm，前壁=%.3fm，可用长度=%.3fm，"
            "车厢点数=%s",
            rear_inner,
            front_inner,
            front_inner - rear_inner,
            f"{len(carriage):,}",
        )
        # 保留既有字段名以兼容下游：carriage_start_y=后内壁，
        # split_y=前内壁。
        return CarriageSplitResult(
            carriage_points=carriage,
            split_y=front_inner,
            carriage_start_y=rear_inner,
            head_y=head_y,
            tail_y=tail_y,
            boundaries_are_inner_faces=inner_faces_valid,
            boundary_method=(
                "observed_wall_surface_fit"
                if inner_faces_valid else "wall_core_fallback"
            ),
            boundary_warning=boundary_warning,
            continuous_body_range=continuous_body_range,
            front_wall_plane=(
                tuple(float(v) for v in front_face.coefficients)
                if inner_faces_valid else None
            ),
            rear_wall_plane=(
                tuple(float(v) for v in rear_face.coefficients)
                if inner_faces_valid else None
            ),
        )

    @staticmethod
    def _wall_core_center(hist: np.ndarray, centers: np.ndarray, peak: int) -> float:
        """壁面稠密核心的加权中心；不受平滑宽度引入的偏移影响。"""
        lo = max(0, peak - 1)
        hi = min(len(hist), peak + 2)
        weights = hist[lo:hi].astype(float)
        if weights.sum() <= 0:
            return float(centers[peak])
        return float(np.average(centers[lo:hi], weights=weights))

class LocalRibDetector:
    """面向归一化 W/L/H 点的轴无关横向拉筋检测器。"""

    def __init__(self, config: Config):
        """初始化局部坐标拉筋检测与几何拟合参数。

        Args:
            config: 内部点提取、底板剔除和拉筋候选配置。
        """
        self.config = config

    def detect(
        self,
        points: np.ndarray,
        rear_inner: float,
        front_inner: float,
        interior_w_range: Optional[Tuple[float, float]] = None,
    ) -> List[RibInfo]:
        """在车厢点云中检测横向拉筋。

        Args:
            points: 车厢点云（局部 W/L/H 坐标）。
            rear_inner: 车厢后内壁 L 坐标（``carriage_start_y``，低 L）。
            front_inner: 车厢前内壁 L 坐标（``split_y``，高 L）。
        Returns:
            按位置排序的 RibInfo 列表；无有效几何候选时返回空列表。
        """
        cfg = self.config

        # 侧壁与底板都会污染检测：侧壁沿整个车长连续存在，其密度起伏会产生假峰，
        # 且开窗统计会把整面墙算进拉筋几何（实测 length=车厢外宽、height=整个
        # 侧壁高）；底板则是大面积密集片层，点数是拉筋的百倍量级，会把真实峰淹没。
        interior, physical_w_range = self._extract_interior(
            points, interior_w_range
        )
        inner_width = physical_w_range[1] - physical_w_range[0]
        if len(interior) < cfg.rib_interior_min_points:
            logger.warning(
                "车厢内部点数 %d < %d，无法进行局部坐标拉筋检测",
                len(interior), cfg.rib_interior_min_points,
            )
            return []

        # 点数占比受采样密度影响，不能据此整体否决实测杆；货面与杆件由自身几何区分。
        view = self._split_at_floor(interior)

        candidates = self._find_candidates(
            view.above_floor, rear_inner, front_inner, inner_width
        )
        if not candidates:
            return []
        selected = self._merge_shoulder_peaks(candidates)

        tree = cKDTree(points)
        top_h = float(np.max(points[:, 2]))
        ribs = []
        for position, local in selected:
            rib = self._build_rib(len(ribs) + 1, position, local, physical_w_range)
            # 每根用自身完整邻域的杆形证据，不与其他拉筋的采样点数比较。
            if rib is not None and validated_rib_support(
                points, tree, rib, view.floor_level, top_h
            ) is not None:
                ribs.append(rib)
        logger.info("局部坐标检测到 %d 根拉筋", len(ribs))
        return ribs

    def _find_candidates(
        self,
        search_points: np.ndarray,
        rear_inner: float,
        front_inner: float,
        inner_width: float,
    ) -> List[Tuple[float, np.ndarray]]:
        """在底板以上的点里找拉筋候选峰，并逐条做几何校验。

        依次过滤：突出度不足的弱峰、落在两端壁排除带内的峰、开窗点数不足的峰、
        横向覆盖率不足的峰（货堆尖或噪声团）。候选随后按自身轴线与完整杆形验证。
        """
        cfg = self.config
        step = cfg.rib_bin_width
        edges = np.arange(rear_inner, front_inner + step * 1.5, step)
        hist, edges = np.histogram(search_points[:, 1], edges)
        centers = (edges[:-1] + edges[1:]) / 2
        smooth = gaussian_filter1d(hist.astype(float), cfg.rib_smooth_sigma)
        peaks, props = find_peaks(
            smooth,
            prominence=max(
                cfg.rib_local_min_prominence,
                np.std(smooth) * cfg.rib_local_prominence_ratio,
            ),
            distance=max(1, int(cfg.rib_min_distance / step)),
        )
        if len(peaks) == 0:
            logger.info("局部坐标：车厢内部未检测到拉筋峰值")
            return []

        prominence_floor = (
            float(np.max(props["prominences"])) * cfg.rib_local_prominence_floor_ratio
        )
        span = front_inner - rear_inner
        lower = rear_inner + span * cfg.carriage_tail_exclude_ratio
        upper = front_inner - span * cfg.carriage_head_exclude_ratio

        candidates: List[Tuple[float, np.ndarray]] = []
        for index, peak in enumerate(peaks):
            if props["prominences"][index] < prominence_floor:
                continue
            position = float(centers[peak])
            if not lower < position < upper:
                continue          # 两端壁本身也是强峰，必须排除
            # 取点同样限定在底板以上：切片若含底板，几何统计会退化成"底板到
            # 拉筋的距离"（实测 height 报 1.4m 而非杆件厚度），且 _fit_rib_axis
            # 会把轴线拟合到底板点上而非拉筋本身。
            local = search_points[
                np.abs(search_points[:, 1] - position) <= cfg.rib_extract_window
            ]
            if len(local) < cfg.rib_min_points:
                continue
            coverage = self._x_coverage(local, inner_width)
            if coverage < cfg.rib_x_coverage_min:
                logger.debug(
                    "  [拒] L=%.3f: X 覆盖率 %.2f < %.2f",
                    position, coverage, cfg.rib_x_coverage_min,
                )
                continue
            candidates.append((position, local))
        return candidates

    def _merge_shoulder_peaks(
        self, candidates: List[Tuple[float, np.ndarray]]
    ) -> List[Tuple[float, np.ndarray]]:
        """同一根拉筋的宽结构可能产生多个肩峰，按间距聚簇后每簇留最强的一个。"""
        merge_distance = self.config.rib_local_cluster_merge_distance
        ordered = sorted(candidates, key=lambda item: item[0])
        clusters: List[List[Tuple[float, np.ndarray]]] = []
        for item in ordered:
            if not clusters or item[0] - clusters[-1][-1][0] >= merge_distance:
                clusters.append([item])
            else:
                clusters[-1].append(item)
        return [max(cluster, key=lambda item: len(item[1])) for cluster in clusters]

    def _build_rib(
        self,
        rib_id: int,
        position: float,
        local: np.ndarray,
        physical_w_range: Tuple[float, float],
    ) -> Optional[RibInfo]:
        """拟合轴线并组装 RibInfo；轴线不稳定时返回 None。

        业务坐标取拟合轴线的中点，而不是原始点的密度加权质心 —— 后者会被
        单侧采样偏斜带偏。
        """
        model = self._fit_rib_axis(local)
        if model is None:
            logger.warning("拉筋候选 L=%.3f 无法拟合稳定轴线，跳过", position)
            return None
        observed_midpoint, direction, _, inliers, rmse = model
        if abs(direction[0]) < 1e-6:
            logger.warning("拉筋候选 L=%.3f 的拟合轴线不横跨车宽，跳过", position)
            return None
        # 无限拟合轴分别与左右车厢内壁 W=const 相交。业务端点和中点由这两个
        # 物理面定义，而不是由“本帧恰好看到了多少杆件点”定义；即使远侧只扫描到
        # 半根，中心也不会向近侧高密度区域偏移。
        endpoints = np.vstack(
            [
                observed_midpoint
                + (
                    (wall_w - observed_midpoint[0]) / direction[0]
                )
                * direction
                for wall_w in sorted(physical_w_range)
            ]
        )
        midpoint = endpoints.mean(axis=0)
        x_lo, x_hi = np.percentile(local[:, 0], [1, 99])
        z_lo, z_hi = np.percentile(local[:, 2], [1, 99])
        return RibInfo(
            rib_id=rib_id,
            y_position=float(midpoint[1]),
            # 业务坐标的 X/Y/Z 必须来自同一个几何定义：拟合轴线中点。
            # 不能只把 Y 换成中点、X/Z 仍取点云中位数，否则单侧密度偏斜时三个
            # 字段并不表示同一个物理点。
            x_center=float(midpoint[0]),
            z_center=float(midpoint[2]),
            x_range=(float(x_lo), float(x_hi)),
            z_range=(float(z_lo), float(z_hi)),
            point_count=len(local),
            length=float(np.linalg.norm(endpoints[1] - endpoints[0])),
            height=float(z_hi - z_lo),
            model_type="ransac_pca_line",
            axis_midpoint=tuple(float(v) for v in midpoint),
            axis_direction=tuple(float(v) for v in direction),
            axis_endpoints=(
                tuple(float(v) for v in endpoints[0]),
                tuple(float(v) for v in endpoints[1]),
            ),
            model_inlier_count=int(len(inliers)),
            model_rmse=float(rmse),
            axis_observed_w_range=tuple(float(v) for v in np.percentile(inliers[:, 0], [1, 99])),
        )

    def _extract_interior(
        self,
        points: np.ndarray,
        physical_w_range: Optional[Tuple[float, float]] = None,
    ) -> Tuple[np.ndarray, Tuple[float, float]]:
        """剔除两侧壁带，返回搜索点和用于轴线截断的物理内壁 W 范围。

        局部坐标下车宽是 W 轴（第 0 列）。用 2%/98% 分位定位壁面（比直方图峰
        更稳，且不受单侧扫描不全影响），各向内收缩 ``rib_wall_band``。
        正式调用传入的profile.interior_w_range已经内缩；这里保持既有发现搜索带，
        不将返回范围视为认证物理内壁。贴壁测位另由rib_side_measurement回实测源定位，
        不能把本搜索带或旧axis_endpoints当成近壁支持证据。
        """
        width = points[:, 0]
        if (
            physical_w_range is not None
            and len(physical_w_range) == 2
            and np.all(np.isfinite(physical_w_range))
            and physical_w_range[1] > physical_w_range[0]
        ):
            w_lo, w_hi = (float(value) for value in physical_w_range)
        else:
            w_lo, w_hi = np.percentile(width, [2.0, 98.0])
        band = self.config.rib_wall_band
        inner_lo, inner_hi = w_lo + band, w_hi - band
        if inner_hi <= inner_lo:
            return points, (float(w_lo), float(w_hi))
        interior = points[(width > inner_lo) & (width < inner_hi)]
        return interior, (float(w_lo), float(w_hi))

    def _split_at_floor(self, interior: np.ndarray) -> "_InteriorView":
        """剔除底板面及以下点，搜索点与后续杆形验证共用同一底板参考。

        拉筋是架在底板上方的横向杆件，因此候选点只能取底板面以上的部分。
        不能用高度分位带来"抑制底板"：空车车厢内部的点绝大多数是底板，
        分位带的上下界会双双落在底板层内，等于只切了底板中的一片，真正的
        拉筋点（实测每根仅约 1000 点，而底板约 15 万点）反被丢弃 —— 实测该
        写法使 2 根真实拉筋全部漏检，同时把底板扫描线的密度起伏误检为 3 根。

        底板面用 ``carriage.floor_surface``（最低密集层中位数）估计，比低分位
        稳、不受底板下方穿透点影响，再上抬 ``rib_floor_clearance`` 作为门槛。
        """
        cfg = self.config
        z = interior[:, 2]
        floor = floor_surface(z)
        above_mask = z > floor + cfg.rib_floor_clearance
        above = interior[above_mask]
        return _InteriorView(
            above_floor=above,
            floor_level=floor,
        )

    @staticmethod
    def _x_coverage(local: np.ndarray, inner_width: float) -> float:
        """拉筋候选在车宽方向的覆盖率（1%~99% 分位跨度 / 车厢内宽）。"""
        if inner_width <= 0:
            return 0.0
        span = float(
            np.percentile(local[:, 0], 99.0) - np.percentile(local[:, 0], 1.0)
        )
        return span / inner_width

    @staticmethod
    def _fit_rib_axis(points: np.ndarray):
        """用带约束的 RANSAC 拟合横向拉筋轴线，再做 PCA 精修。"""
        if len(points) < 50:
            return None
        # 抑制底板/顶面薄片，只保留横向杆件。
        h_lo, h_hi = np.percentile(points[:, 2], [8, 92])
        work = points[(points[:, 2] > h_lo) & (points[:, 2] < h_hi)]
        if len(work) < 50:
            work = points
        rng = np.random.RandomState(20260715)
        sample = work if len(work) <= 5000 else work[rng.choice(len(work), 5000, replace=False)]
        width = float(np.percentile(sample[:, 0], 99) - np.percentile(sample[:, 0], 1))
        best = None
        threshold = 0.055
        for _ in range(350):
            i, j = rng.randint(0, len(sample), 2)
            if i == j:
                continue
            vector = sample[j] - sample[i]
            norm = np.linalg.norm(vector)
            if norm < max(0.5, width * 0.35):
                continue
            direction = vector / norm
            # 挂车拉筋主要沿局部宽度 W 方向延伸。
            if abs(direction[0]) < 0.75:
                continue
            distance = np.linalg.norm(np.cross(sample - sample[i], direction), axis=1)
            mask = distance <= threshold
            score = int(mask.sum())
            if best is None or score > best[0]:
                best = (score, sample[i], direction)
        if best is None or best[0] < 30:
            return None

        origin, direction = best[1], best[2]
        for _ in range(2):
            distance = np.linalg.norm(np.cross(work - origin, direction), axis=1)
            inliers = work[distance <= threshold]
            if len(inliers) < 30:
                return None
            origin = inliers.mean(axis=0)
            _, _, vh = np.linalg.svd(inliers - origin, full_matrices=False)
            direction = vh[0]
            if direction[0] < 0:
                direction *= -1
        distance = np.linalg.norm(np.cross(work - origin, direction), axis=1)
        inliers = work[distance <= threshold]
        projection = (inliers - origin) @ direction
        low, high = np.percentile(projection, [1, 99])
        endpoints = np.vstack((origin + low * direction, origin + high * direction))
        midpoint = endpoints.mean(axis=0)
        rmse = float(np.sqrt(np.mean(distance[distance <= threshold] ** 2)))
        return midpoint, direction, endpoints, inliers, rmse
