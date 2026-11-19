"""场地坐标与支撑平面算法；不依赖命令行、业务编排或文件存储。"""

from dataclasses import dataclass
from typing import Any, Dict, List, Tuple

import numpy as np
import open3d as o3d

from carpointcloud_analyzer.domain.errors import DualCloudMergeError
from carpointcloud_analyzer.domain.runtime import logger
from carpointcloud_analyzer.domain.utils.planes import deterministic_plane_hypothesis


@dataclass(frozen=True)
class SiteFrame:
    """CAM_004 原始坐标到临时场地坐标的刚体变换。

    ``axes`` 的三列依次是临时 X/Y/Z 在 CAM_004 中的单位向量；其中 Z 是估计的
    地面法向。只在环境过滤期间使用，不改变最终 PCD 的 CAM_004 坐标。
    """

    origin: np.ndarray
    axes: np.ndarray
    plane: np.ndarray
    inlier_count: int
    sampled_points: int
    spans: Tuple[float, float]
    one_sided_ratio: float

    def transform(self, points: np.ndarray) -> np.ndarray:
        """把 CAM_004 原始点变换到临时场地坐标。"""
        return (np.asarray(points, dtype=float) - self.origin) @ self.axes

    def to_dict(self) -> Dict[str, Any]:
        """导出场地坐标原点、轴向和支撑平面质量指标。"""
        return {
            "originInCameraA": np.round(self.origin, 8).tolist(),
            "axesInCameraA": np.round(self.axes, 8).tolist(),
            "groundPlaneInCameraA": np.round(self.plane, 8).tolist(),
            "inlierCount": self.inlier_count,
            "sampledPoints": self.sampled_points,
            "planeSpans": [round(value, 4) for value in self.spans],
            "oneSidedRatio": round(self.one_sided_ratio, 4),
        }


@dataclass(frozen=True)
class _PlaneCandidate:
    """场地支撑平面的内部候选及其几何评分。"""
    centroid: np.ndarray
    normal: np.ndarray
    axes: np.ndarray
    inlier_count: int
    spans: Tuple[float, float]
    one_sided_ratio: float
    score: float


def _validate_points(points: np.ndarray, name: str) -> np.ndarray:
    """校验点云形状、非空性和数值有限性。"""
    values = np.asarray(points, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3:
        raise DualCloudMergeError(
            f"{name} 点云形状必须为 (N, 3)，实际为 {values.shape}"
        )
    if len(values) == 0:
        raise DualCloudMergeError(f"{name} 点云为空")
    if not np.all(np.isfinite(values)):
        raise DualCloudMergeError(f"{name} 点云包含 NaN 或 Inf")
    return values


def _plane_axes(normal: np.ndarray, points: np.ndarray) -> np.ndarray:
    """建立以 ``normal`` 为 Z 的正交基，平面内 X 取点集最大延伸方向。"""
    normal = np.asarray(normal, dtype=float)
    normal /= np.linalg.norm(normal) + 1e-12
    centered = points - points.mean(axis=0)
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    axis_x = vh[0] - float(vh[0] @ normal) * normal
    if np.linalg.norm(axis_x) < 1e-8:
        basis = np.eye(3)[int(np.argmin(np.abs(normal)))]
        axis_x = basis - float(basis @ normal) * normal
    axis_x /= np.linalg.norm(axis_x) + 1e-12
    axis_y = np.cross(normal, axis_x)
    axis_y /= np.linalg.norm(axis_y) + 1e-12
    axis_x = np.cross(axis_y, normal)
    return np.column_stack([axis_x, axis_y, normal])


def _refine_plane(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """通过 SVD 精化平面中心和单位法向。"""
    centroid = points.mean(axis=0)
    _, _, vh = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vh[-1]
    normal /= np.linalg.norm(normal) + 1e-12
    return centroid, normal


def estimate_site_frame(
    points_in_camera_a: np.ndarray,
    *,
    voxel_size: float = 0.12,
    distance_threshold: float = 0.10,
    max_planes: int = 6,
    min_plane_span: float = 2.5,
    min_plane_area: float = 18.0,
) -> SiteFrame:
    """从任意轴向的共同场景估计地面，并建立临时 Z-up 场地坐标。

    只依赖几何，不假定 CAM_004 的 X/Y/Z 各自指向。候选平面必须同时满足：

    * 面积和两个平面内方向的跨度足够大，避免把车顶或车侧壁当地面；
    * 大多数非平面点位于同一侧，符合地面作为支撑面的特征；
    * 多个候选中优先选择内点最多、覆盖面积最大的平面。

    如果无法可靠找到支撑平面会安全失败，而不是在错误轴向下删除车辆。
    """
    points = _validate_points(points_in_camera_a, "共同场景")
    if len(points) < 500:
        raise DualCloudMergeError("共同场景点数不足 500，无法可靠估计磅秤/地面")

    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    sampled_cloud = cloud.voxel_down_sample(voxel_size)
    sampled = np.asarray(sampled_cloud.points)
    if len(sampled) < 300:
        sampled = points.copy()

    work = sampled.copy()
    candidates: List[_PlaneCandidate] = []
    for _ in range(max_planes):
        if len(work) < 150:
            break
        try:
            # 全局种子不能固定 Open3D 多线程 RANSAC 的候选选择。
            # 复用按点集排序的局部采样；距离阈值、预算及后续选面规则不变。
            _, inlier_indices = deterministic_plane_hypothesis(
                work, distance_threshold, num_iterations=1200)
        except Exception as exc:
            logger.warning("支撑平面 RANSAC 失败：%s", exc)
            break

        inlier_indices = np.asarray(inlier_indices, dtype=int)
        if len(inlier_indices) < 100:
            break
        inliers = work[inlier_indices]
        centroid, normal = _refine_plane(inliers)

        signed = (sampled - centroid) @ normal
        away = np.abs(signed) > distance_threshold * 1.5
        positive = int(np.sum(signed[away] > 0))
        negative = int(np.sum(signed[away] < 0))
        if negative > positive:
            normal = -normal
            signed = -signed
            positive, negative = negative, positive
        sided_total = positive + negative
        one_sided = positive / sided_total if sided_total else 0.0

        axes = _plane_axes(normal, inliers)
        projected = (inliers - centroid) @ axes[:, :2]
        spans_array = np.percentile(projected, 98, axis=0) - np.percentile(
            projected, 2, axis=0
        )
        spans = tuple(sorted((float(spans_array[0]), float(spans_array[1])), reverse=True))
        area = spans[0] * spans[1]

        # 支撑面之上应容纳绝大多数现场点。允许少量 RANSAC 噪声落在面下。
        support_fraction = float(np.mean((signed >= -0.30) & (signed <= 8.0)))
        if (
            spans[1] >= min_plane_span
            and area >= min_plane_area
            and one_sided >= 0.55
            and support_fraction >= 0.70
        ):
            score = (
                len(inliers)
                * (0.5 + one_sided)
                * (0.5 + support_fraction)
                * (1.0 + np.log1p(area))
            )
            candidates.append(
                _PlaneCandidate(
                    centroid=centroid,
                    normal=normal,
                    axes=axes,
                    inlier_count=len(inliers),
                    spans=spans,
                    one_sided_ratio=one_sided,
                    score=float(score),
                )
            )

        keep = np.ones(len(work), dtype=bool)
        keep[inlier_indices] = False
        work = work[keep]

    if not candidates:
        raise DualCloudMergeError(
            "未找到可靠的大面积支撑平面。请确认两台雷达都采集到磅秤/地面，"
            "或检查点云单位是否正确"
        )

    best = max(candidates, key=lambda candidate: candidate.score)
    plane = np.r_[best.normal, -float(best.normal @ best.centroid)]
    logger.info(
        "临时场地坐标：地面跨度 %.2fm × %.2fm，内点 %d，单侧率 %.1f%%",
        best.spans[0],
        best.spans[1],
        best.inlier_count,
        best.one_sided_ratio * 100,
    )
    return SiteFrame(
        origin=best.centroid,
        axes=best.axes,
        plane=plane,
        inlier_count=best.inlier_count,
        sampled_points=len(sampled),
        spans=best.spans,
        one_sided_ratio=best.one_sided_ratio,
    )


# 朝向判据只在 ±ORIENTATION_BAND_M 内统计。超出这个范围的是远处地坪和地坑，
# 它们与"车辆停在磅台哪一侧"无关，却足以主导无界的跨度比较。
ORIENTATION_BAND_M = 5.0
ORIENTATION_DEAD_ZONE_M = 0.3


def _assert_normal_faces_vehicle(distance: np.ndarray) -> None:
    """校验标定面法向指向车辆一侧。

    历史实现比较 ``P99`` 上方跨度与 ``|P1|`` 下方跨度，但这两个量由当帧能看到
    多远的背景决定，而不是由车辆位置决定。实测同一份正确标定面：

    ========================  ==========  ==========
    场景                       P99/|P1|    结论
    ========================  ==========  ==========
    0903 共同场景              1.08        侥幸通过（余量仅 0.27m）
    0907 共同场景              0.28        误判为反向
    0903 / 0907 的 CAM_004     0.16/0.18   永远误判为反向
    ========================  ==========  ==========

    改为只在 ±5m 有界带内比较点数：车辆是停在磅台上的实体，带内上方点必然多于
    下方；远处地坪与地坑被排除在投票外。
    """
    inside = np.abs(distance) <= ORIENTATION_BAND_M
    above = int(np.count_nonzero((distance > ORIENTATION_DEAD_ZONE_M) & inside))
    below = int(np.count_nonzero((distance < -ORIENTATION_DEAD_ZONE_M) & inside))
    if above <= below:
        raise DualCloudMergeError(
            f"地磅标定面法向疑似背向车辆：标定面上方 ±{ORIENTATION_BAND_M:.0f}m 内 "
            f"{above} 点、下方 {below} 点。若标定确实正确，请确认该帧车辆已在磅台上；"
            "否则请翻转 [a,b,c,d] 的全部符号"
        )


def site_frame_from_plane(
    points_in_camera_a: np.ndarray,
    plane_in_camera_a: np.ndarray,
    *,
    inlier_tolerance: float = 0.10,
    validate_orientation: bool = True,
) -> SiteFrame:
    """用已标定地磅平面建立临时 Z-up 坐标。

    与 :func:`estimate_site_frame` 不同，本函数不从当前车辆帧“猜”高度基准。
    固定地磅可能比真实地面高，若每帧选择点数最多的水平面，很容易选到地面并
    产生一个稳定但业务含义错误的高度。这里把标定平面法向作为临时 Z 轴，平面
    内方向只用于数值建系，不参与高度定义。

    ``plane_in_camera_a`` 的法向方向由标定文件负责，必须朝向车辆；函数不会根据
    单帧点分布静默翻转法向，以免同一场地不同车辆得到相反的高度符号。
    """
    points = _validate_points(points_in_camera_a, "共同场景")
    plane = np.asarray(plane_in_camera_a, dtype=float)
    if plane.shape != (4,) or not np.all(np.isfinite(plane)):
        raise DualCloudMergeError("地磅标定面必须是四个有限数 [a,b,c,d]")
    normal_norm = float(np.linalg.norm(plane[:3]))
    if normal_norm <= 1e-9:
        raise DualCloudMergeError("地磅标定面的法向量不能为零")
    plane = plane / normal_norm

    normal = plane[:3]
    # 原点取源坐标原点在地磅面上的正交投影，保证变换后该平面严格为 z=0。
    origin = -plane[3] * normal
    distance = points @ normal + plane[3]
    if validate_orientation:
        _assert_normal_faces_vehicle(distance)
    inliers = points[np.abs(distance) <= inlier_tolerance]
    axes_points = inliers if len(inliers) >= 20 else points
    axes = _plane_axes(normal, axes_points)
    projected = (axes_points - origin) @ axes[:, :2]
    if len(projected) >= 2:
        spans_array = np.percentile(projected, 98, axis=0) - np.percentile(
            projected, 2, axis=0
        )
        spans = tuple(
            sorted(
                (float(spans_array[0]), float(spans_array[1])),
                reverse=True,
            )
        )
    else:
        spans = (0.0, 0.0)

    away = np.abs(distance) > inlier_tolerance * 1.5
    away_count = int(np.count_nonzero(away))
    one_sided_ratio = (
        float(np.count_nonzero(distance[away] > 0) / away_count)
        if away_count else 1.0
    )
    return SiteFrame(
        origin=origin,
        axes=axes,
        plane=plane,
        inlier_count=int(len(inliers)),
        sampled_points=int(len(points)),
        spans=spans,
        one_sided_ratio=one_sided_ratio,
    )
