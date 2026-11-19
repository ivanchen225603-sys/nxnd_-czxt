"""双雷达点云合并：外参标定 + 在线合并。

场景约定
--------
两台雷达固定安装、采集时车辆停稳，因此：

* **两台雷达之间的位姿是常量**，标定一次即可，不需要每帧配准。每帧跑 ICP 反而
  危险 —— 重叠区往往只覆盖车头一带，车头表面偏光滑，ICP 在这类面上会"滑动"，
  收敛到错误位姿且不自知，失败是静默的。
* **不存在时间同步问题**。车静止、雷达固定，两帧无论差多久都对应同一个刚体。
  若将来改成行进中采集，本模块的前提即失效（5 km/h 下 100ms ≈ 14cm 错位）。

标定数据的两个来源互为交叉验证：
    * 空磅道（无车）—— 静态基础设施，无任何运动干扰，可多帧平均降噪；
    * 同一辆车同一位置 —— 车体有三个互相垂直方向的平面（侧壁 / 车头前脸 /
      顶面），对 6 个自由度的约束比平坦地面强得多。

密度均匀化是必需步骤，不是可选优化
----------------------------------
本项目多处算法按点密度判定：``carriage.floor_surface`` 取密度峰层、拉筋检测靠
找峰、``occupancy_extent`` 用每 bin 点数门槛、DBSCAN 用 ``min_points``。两片点云
直接 vstack 会让重叠区密度翻倍，上述判据全部系统性偏移。因此合并后必须做体素
去重，把密度拉回均匀。
"""

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import open3d as o3d

from carpointcloud_analyzer.domain.runtime import logger
from carpointcloud_analyzer.domain.utils.planes import fit_plane

# 退化判据：信息矩阵最小特征值低于此值，说明该自由度几乎不受约束
_DEGENERACY_EIGENVALUE_MIN = 1e-3
_AXIS_NAMES = ("roll", "pitch", "yaw", "x", "y", "z")


@dataclass
class CalibrationQuality:
    """标定质量报告。``degenerate_axes`` 非空即表示结果不可信。"""

    fitness: float                 # 重叠点占比，越高越好
    inlier_rmse: float             # 内点残差（m）
    correspondence_count: int
    degenerate_axes: List[str] = field(default_factory=list)
    condition_number: float = 0.0
    warnings: List[str] = field(default_factory=list)

    @property
    def trustworthy(self) -> bool:
        """是否不存在退化自由度和质量告警。"""
        return not self.degenerate_axes and not self.warnings

    def to_dict(self) -> Dict[str, Any]:
        """导出可审计的标定重叠、残差和退化指标。

        Returns:
            适合写入外参 JSON 的质量字典。
        """
        return {
            "fitness": round(self.fitness, 4),
            "inlierRmse": round(self.inlier_rmse, 5),
            "correspondenceCount": self.correspondence_count,
            "degenerateAxes": list(self.degenerate_axes),
            "conditionNumber": round(self.condition_number, 1),
            "trustworthy": self.trustworthy,
            "warnings": list(self.warnings),
        }


def _parse_rigid_matrix(data: Dict[str, Any]) -> np.ndarray:
    """校验 4x4 刚体变换矩阵。

    Raises:
        ValueError: 缺失、形状不对、含非有限值、旋转部分非正交或含镜像缩放。
    """
    matrix = np.asarray(data["matrix"], dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError(f"外参矩阵形状应为 (4,4)，实际 {matrix.shape}")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("外参矩阵包含 NaN 或 Inf")
    rotation = matrix[:3, :3]
    # 标定文件可能被手工编辑或跨版本损坏，落地前校验旋转部分仍是正交阵
    if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-4):
        raise ValueError("外参矩阵的旋转部分不是正交阵，文件可能已损坏")
    if abs(np.linalg.det(rotation) - 1.0) > 1e-3:
        raise ValueError("外参矩阵含镜像或缩放，不是纯刚体变换")
    if not np.allclose(matrix[3], np.array([0.0, 0.0, 0.0, 1.0]), atol=1e-4):
        raise ValueError("外参矩阵最后一行必须为 [0,0,0,1]")
    return matrix


def _parse_camera_pair(data: Dict[str, Any]) -> Tuple[str, str]:
    """校验雷达身份与方向标记。

    Raises:
        ValueError: frame 不是 B_to_A，或 A/B 只填一个、填成同一台。
    """
    if data.get("frame") != "B_to_A":
        raise ValueError("外参 frame 必须为 B_to_A，禁止静默使用反向矩阵")
    camera_a = str(data.get("cameraA") or "").strip()
    camera_b = str(data.get("cameraB") or "").strip()
    if bool(camera_a) != bool(camera_b):
        raise ValueError("外参 cameraA/cameraB 必须同时填写或同时留空")
    if camera_a and camera_a.casefold() == camera_b.casefold():
        raise ValueError("外参 cameraA/cameraB 必须是不同雷达")
    return camera_a, camera_b


def _parse_baseline_residual(data: Dict[str, Any]) -> Optional[float]:
    """校验基线残差；``None`` 表示未提供，不能当作 0。

    Raises:
        ValueError: 不能转成数值，或不是有限正数。
    """
    baseline_residual = data.get("baselineResidual")
    if baseline_residual is None:
        return None
    baseline_residual = float(baseline_residual)
    if not np.isfinite(baseline_residual) or baseline_residual <= 0.0:
        raise ValueError("baselineResidual 必须是有限正数")
    return baseline_residual


def _parse_weighbridge_plane(data: Dict[str, Any]) -> Optional[np.ndarray]:
    """校验地磅标定平面并归一化法向。

    Raises:
        ValueError: 不是四个数值、含非有限值或法向量为零。
    """
    weighbridge_plane = data.get("weighbridgePlaneA")
    if weighbridge_plane is None:
        return None
    weighbridge_plane = np.asarray(weighbridge_plane, dtype=float)
    if weighbridge_plane.shape != (4,):
        raise ValueError("weighbridgePlaneA 必须是 [a,b,c,d]")
    if not np.all(np.isfinite(weighbridge_plane)):
        raise ValueError("weighbridgePlaneA 包含 NaN 或 Inf")
    normal_norm = float(np.linalg.norm(weighbridge_plane[:3]))
    if normal_norm <= 1e-9:
        raise ValueError("weighbridgePlaneA 的法向量不能为零")
    # 统一为单位法向，后续点到平面的代数值才能直接解释为米。
    return weighbridge_plane / normal_norm


def _parse_empty_lane_baseline(data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """校验空车道基线快照的容器类型。

    Raises:
        ValueError: 既不是对象也不是 null。
    """
    empty_lane_baseline = data.get("emptyLaneBaseline")
    if empty_lane_baseline is not None and not isinstance(empty_lane_baseline, dict):
        raise ValueError("emptyLaneBaseline 必须是对象或 null")
    return empty_lane_baseline


def _parse_quality(data: Dict[str, Any]) -> Optional["CalibrationQuality"]:
    """校验标定质量快照。

    Raises:
        ValueError: 容器类型、字段完整性或数值范围不合法。
    """
    quality_data = data.get("quality")
    if quality_data is None:
        return None
    if not isinstance(quality_data, dict):
        raise ValueError("外参 quality 必须是对象或 null")
    degenerate_axes = quality_data.get("degenerateAxes", [])
    quality_warnings = quality_data.get("warnings", [])
    if not isinstance(degenerate_axes, list) or not isinstance(quality_warnings, list):
        raise ValueError("外参 quality 的 degenerateAxes/warnings 必须是数组")
    try:
        quality = CalibrationQuality(
            fitness=float(quality_data["fitness"]),
            inlier_rmse=float(quality_data["inlierRmse"]),
            correspondence_count=int(quality_data["correspondenceCount"]),
            degenerate_axes=[str(value) for value in degenerate_axes],
            condition_number=float(quality_data.get("conditionNumber", 0.0)),
            warnings=[str(value) for value in quality_warnings],
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("外参 quality 字段不完整或类型错误") from exc
    if (
        not 0.0 <= quality.fitness <= 1.0
        or not np.isfinite(quality.inlier_rmse)
        or quality.inlier_rmse < 0.0
        or quality.correspondence_count < 0
        or not np.isfinite(quality.condition_number)
        or quality.condition_number < 0.0
    ):
        raise ValueError("外参 quality 数值超出合法范围")
    return quality


@dataclass
class Extrinsic:
    """雷达 B → 雷达 A 的刚体变换。

    以 A 为基准坐标系，这样已建立的空车基准档案与历史数据继续有效。
    """

    matrix: np.ndarray                       # 4x4 齐次变换
    quality: Optional[CalibrationQuality] = None
    source: str = ""                         # 标定数据来源说明
    camera_a: str = ""                       # 基准雷达 A 标识
    camera_b: str = ""                       # 被变换雷达 B 标识
    # 标定当时测得的重叠区残差，作为每帧漂移判定的基线。绝对阈值无法适配不同
    # 现场的噪声水平：实测 10cm 漂移的残差是 77mm，若固定用 150mm 就会放行。
    baseline_residual: Optional[float] = None
    # 固定地磅台面在 CAM_004/A 坐标系中的平面系数，单位为米，法向朝向车辆。
    # 它与雷达外参属于同一份场地标定：既用于任意原始轴向的 Z-up 对齐，也作为
    # “车厢底板到底磅”高度的唯一业务基准。
    weighbridge_plane_a: Optional[np.ndarray] = None
    # 一次性空磅道验收留下的锚点：支架永久固定，验收通过后不再逐帧判定漂移。
    # 它**不参与运行时门禁**，只在将来怀疑雷达被动过时提供可比对的基准值。
    empty_lane_baseline: Optional[Dict[str, Any]] = None

    def apply(self, points: np.ndarray) -> np.ndarray:
        """把 B 坐标系的点变换到 A 坐标系。"""
        if len(points) == 0:
            return points
        return points @ self.matrix[:3, :3].T + self.matrix[:3, 3]

    def to_dict(self) -> Dict[str, Any]:
        """导出带方向、雷达身份、参考平面和质量的外参结构。

        Returns:
            可 JSON 序列化的 B 到 A 刚体变换。
        """
        return {
            "version": "v1",
            "frame": "B_to_A",
            "cameraA": self.camera_a,
            "cameraB": self.camera_b,
            "matrix": [[round(float(v), 8) for v in row] for row in self.matrix],
            "source": self.source,
            "baselineResidual": (
                round(self.baseline_residual, 5)
                if self.baseline_residual is not None else None
            ),
            "weighbridgePlaneA": (
                [round(float(value), 8) for value in self.weighbridge_plane_a]
                if self.weighbridge_plane_a is not None else None
            ),
            "emptyLaneBaseline": (
                dict(self.empty_lane_baseline)
                if self.empty_lane_baseline is not None else None
            ),
            "quality": self.quality.to_dict() if self.quality else None,
        }

    @staticmethod
    def from_dict(data: Dict[str, Any]) -> "Extrinsic":
        """从不可信 JSON 数据恢复并严格校验刚体外参。

        逐组字段的校验拆到模块级 ``_parse_*``：原实现把六组无关校验堆在一个
        103 行的方法里，圈复杂度 35，任何一项改动都要通读全部分支。

        Args:
            data: 外参 JSON 对象。

        Returns:
            旋转正交、方向明确且字段范围合法的外参。

        Raises:
            ValueError: 矩阵、雷达身份、质量或地磅平面字段缺失或非法。
        """
        # 校验顺序即报错优先级，必须与拆分前一致：矩阵 -> 雷达身份 -> 基线残差
        # -> 地磅平面 -> 空车道基线 -> 质量。同时缺多项时调用方看到的仍是同一条。
        matrix = _parse_rigid_matrix(data)
        camera_a, camera_b = _parse_camera_pair(data)
        baseline_residual = _parse_baseline_residual(data)
        weighbridge_plane = _parse_weighbridge_plane(data)
        empty_lane_baseline = _parse_empty_lane_baseline(data)
        quality = _parse_quality(data)
        return Extrinsic(
            matrix=matrix,
            quality=quality,
            source=data.get("source", ""),
            camera_a=camera_a,
            camera_b=camera_b,
            baseline_residual=baseline_residual,
            weighbridge_plane_a=weighbridge_plane,
            empty_lane_baseline=empty_lane_baseline,
        )

    def validate_camera_pair(self, camera_a: str, camera_b: str) -> None:
        """确认外参确实属于本次请求的固定 A/B，避免错文件静默融合。"""
        if not self.camera_a or not self.camera_b:
            raise ValueError(
                "双雷达在线分析要求外参 JSON 包含 cameraA 和 cameraB"
            )
        if (
            self.camera_a.casefold() != camera_a.strip().casefold()
            or self.camera_b.casefold() != camera_b.strip().casefold()
        ):
            raise ValueError(
                "外参 cameraA/cameraB 与请求不一致："
                f"文件={self.camera_a}/{self.camera_b}，"
                f"请求={camera_a}/{camera_b}"
            )

    def save(self, path: str) -> str:
        """把外参以 UTF-8 JSON 写入指定路径。

        Args:
            path: 外参文件目标路径；父目录不存在时自动创建。

        Returns:
            实际写入的文件路径。

        Raises:
            OSError: 目录创建或文件写入失败。
        """
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(
            json.dumps(self.to_dict(), indent=2, ensure_ascii=False), encoding="utf-8"
        )
        logger.info("外参已保存：%s", target)
        return str(target)

    @staticmethod
    def load(path: str) -> "Extrinsic":
        """读取并校验外参 JSON。

        Args:
            path: 待加载的外参文件路径。

        Returns:
            通过刚体和业务字段校验的外参。

        Raises:
            OSError: 文件不可读取。
            ValueError: JSON 或外参内容非法。
        """
        return Extrinsic.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


@dataclass
class MergeParams:
    """标定与合并的阈值。"""

    # 在线融合只做很轻的单雷达清洗：NaN/Inf 必须去掉，SOR 阈值则刻意放宽，
    # 仅压掉明显孤立飞点。车头在单台雷达里本来就可能稀疏，不能在合并前做
    # DBSCAN/车辆簇裁剪，否则第二台雷达用于补齐的车头会先被误删。
    prefilter_neighbors: int = 12
    prefilter_std_ratio: float = 3.0
    min_prefilter_retention: float = 0.90
    min_filtered_points: int = 500
    voxel_size: float = 0.05          # 配准用降采样尺寸（m）
    normal_radius_factor: float = 2.0  # 法向估计半径 = voxel * 该系数
    feature_radius_factor: float = 5.0
    ransac_distance_factor: float = 1.5
    ransac_iterations: int = 4_000_000
    ransac_confidence: float = 0.999
    icp_distance_factor: float = 2.0
    icp_max_iterations: int = 60
    merge_voxel_size: float = 0.02    # 合并后密度均匀化尺寸（m）
    min_fitness: float = 0.30         # 低于此值判定重叠不足
    max_inlier_rmse: float = 0.08     # 高于此值判定配准不可信（m）
    min_overlap_match_ratio: float = 0.60  # 每帧重叠区匹配率下限
    structure_height_min: float = 0.15     # 高于地面此值才算立体结构（m）
    # 原始场景质量检查需要先排除地面；已经由共同场景提取出的 vehicle-only
    # 点云本身没有地面，再做 RANSAC 会随机把车顶/侧壁误当地面，导致同一对 PCD
    # 的匹配率在门槛两侧波动。
    filter_ground_for_overlap: bool = True
    residual_percentile: float = 90.0      # 重叠残差取的分位（对少数严重错位敏感）
    max_overlap_residual: float = 0.15     # 高分位残差绝对上限（m）
    drift_residual_factor: float = 2.5     # 相对标定基线的倍数门槛
    # 重叠残差/匹配率是否参与"融合是否合格"的判定。
    #
    # 关闭的场景：支架永久固定、外参漂移已由一次性空磅道验收确认，且逐帧参照物
    # （车辆）本身在变。实测载车帧零漂移时匹配率只有 14.76%、门槛 12%，余量 23%，
    # 而余量还会随车形和停车位置波动 —— 这种配置下它更容易误杀正常车辆，而不是
    # 发现真漂移。关闭后指标继续计算并记录在 overlapNotes 里，只是不再拦截。
    #
    # 点数不足、轻滤保留率过低这类"数据本身不完整"的告警不受此开关影响，始终拦截。
    enforce_overlap_gate: bool = True


# ---------------------------------------------------------------------------
# 内部工具
# ---------------------------------------------------------------------------
def _to_cloud(points: np.ndarray) -> o3d.geometry.PointCloud:
    """把 ``(N, 3)`` 数组包装为 Open3D 点云。"""
    return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(points, float)))


def _light_filter(
    points: np.ndarray, params: MergeParams
) -> Tuple[np.ndarray, int]:
    """单雷达有限点清洗 + 宽松统计离群滤波。

    Returns:
        (滤波后点云, 有限点数量)。原始数量由调用方记录，二者结合即可审计
        NaN/Inf 与 SOR 各自删掉了多少点。
    """
    values = np.asarray(points, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError(f"point cloud must have shape (N, 3), got {values.shape}")

    finite = values[np.all(np.isfinite(values), axis=1)]
    n_finite = int(len(finite))
    if n_finite == 0:
        raise ValueError("point cloud has no finite points")
    if n_finite < 3:
        return finite.copy(), n_finite

    cloud = _to_cloud(finite)
    filtered, _ = cloud.remove_statistical_outlier(
        nb_neighbors=min(params.prefilter_neighbors, n_finite - 1),
        std_ratio=params.prefilter_std_ratio,
    )
    clean = np.asarray(filtered.points).copy()
    if len(clean) == 0:
        # 极小或退化点集上 Open3D 的统计量可能没有意义；保留有限点比把整台
        # 雷达清空更保守，后续重叠质量门禁仍会拒绝不可信融合。
        clean = finite.copy()
    return clean, n_finite


def _prepare(
    points: np.ndarray, params: MergeParams
) -> Tuple[o3d.geometry.PointCloud, o3d.pipelines.registration.Feature]:
    """降采样 + 法向估计 + FPFH 特征，供全局配准使用。"""
    cloud = _to_cloud(points).voxel_down_sample(params.voxel_size)
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=params.voxel_size * params.normal_radius_factor, max_nn=30
    ))
    feature = o3d.pipelines.registration.compute_fpfh_feature(
        cloud,
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=params.voxel_size * params.feature_radius_factor, max_nn=100
        ),
    )
    return cloud, feature


def _ground_alignment(points_a: np.ndarray, points_b: np.ndarray) -> np.ndarray:
    """用两侧地平面把 roll / pitch / z 对齐，作为全局配准的初值。

    地平面是两台雷达必然共视的结构，先用它压掉 3 个自由度，能显著提高后续
    特征配准的成功率。剩余的 yaw / x / y 由 FPFH + RANSAC 解决。
    """
    plane_a, plane_b = _dominant_ground(points_a), _dominant_ground(points_b)
    if plane_a is None or plane_b is None:
        return np.eye(4)

    normal_a, normal_b = plane_a[:3], plane_b[:3]
    axis = np.cross(normal_b, normal_a)
    axis_norm = np.linalg.norm(axis)
    rotation = np.eye(3)
    if axis_norm > 1e-8:
        angle = float(np.arctan2(axis_norm, float(normal_b @ normal_a)))
        rotation = _rodrigues(axis / axis_norm, angle)

    transform = np.eye(4)
    transform[:3, :3] = rotation
    # 旋转后再把两个平面的高度对齐
    rotated_offset = float(plane_b[3] - plane_a[3])
    transform[:3, 3] = normal_a * rotated_offset
    return transform


def _dominant_ground(points: np.ndarray) -> Optional[np.ndarray]:
    """取大面积支撑面，法向统一朝向场景主体。

    原始 PCD 的 XYZ 方向不固定，不能再用 ``plane[2]`` 判断“向上”。固定场景中
    车辆、护栏等主体应位于支撑面的同一侧，因此用非平面点数量较多的一侧确定
    法向。该平面在融合阶段只用于重叠质量检查，不作为地磅高度业务基准。
    """
    if len(points) < 200:
        return None
    cloud = _to_cloud(points).voxel_down_sample(0.1)
    work = np.asarray(cloud.points)
    if len(work) < 100:
        return None
    try:
        plane, inliers = _to_cloud(work).segment_plane(0.08, 3, 800)
    except Exception:
        # 拿不到地磅平面时 floorProfile 会退化为 null，高低板明细随之为空。
        # 这是设计内的降级，但必须能从日志区分"没有地磅"和"分割失败"。
        logger.warning(
            "地磅平面分割失败，本次不提供地磅参考面：降采样后点数 %d",
            len(work),
            exc_info=True,
        )
        return None
    inliers = np.asarray(inliers)
    if len(inliers) < 50:
        return None
    plane = fit_plane(work[inliers])
    if not plane.any():
        return None
    signed = points @ plane[:3] + plane[3]
    away = np.abs(signed) > 0.12
    if np.count_nonzero(signed[away] > 0) < np.count_nonzero(signed[away] < 0):
        plane = -plane
    return plane


def _rodrigues(axis: np.ndarray, angle: float) -> np.ndarray:
    """轴角 → 旋转矩阵。"""
    cross = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return (
        np.eye(3) + np.sin(angle) * cross + (1.0 - np.cos(angle)) * (cross @ cross)
    )


def _check_degeneracy(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    transform: np.ndarray,
    max_distance: float,
) -> Tuple[List[str], float]:
    """用 point-to-plane 的 Hessian 判断哪些自由度欠约束。

    重叠区若几乎只有平坦地面，水平面内的 yaw/x/y 是自由的：配准会滑动到一个
    残差很小、实际错位几十厘米的解，且不会报错。必须能识别出这种情况。

    不能用 open3d 的 ``get_information_matrix_from_point_clouds``：它基于点对点
    对应，纯平面场景下最小特征值仍有 0.041（实测），完全捕捉不到滑动自由度。
    point-to-plane 的 Hessian 才是对的判据 —— 残差 ``rᵢ = nᵢ·(R pᵢ + t − qᵢ)``
    对位姿的雅可比为 ``[pᵢ × nᵢ, nᵢ]``，纯平面时 ``nᵢ=(0,0,1)`` 使 x、y、yaw
    三列恒为零，对应特征值精确趋零。
    """
    correspondence = _correspondence_normals(source, target, transform, max_distance)
    if correspondence is None:
        return list(_AXIS_NAMES), 0.0
    points, normals = correspondence

    # 平移到质心只去掉原点影响；再用均方根半径消除旋转项的长度量纲，
    # 避免同一几何结构因使用米/毫米或场景尺寸不同而改变退化判定。
    centered = points - points.mean(axis=0)
    radius = float(np.sqrt(np.mean(np.sum(centered ** 2, axis=1))))
    if radius > 0:
        centered = centered / radius
    jacobian = np.hstack([np.cross(centered, normals), normals])
    hessian = jacobian.T @ jacobian

    eigenvalues, eigenvectors = np.linalg.eigh(hessian)
    largest = float(eigenvalues.max())
    if largest <= 0:
        return list(_AXIS_NAMES), 0.0
    degenerate: List[str] = []
    for index, value in enumerate(eigenvalues):
        if value / largest >= _DEGENERACY_EIGENVALUE_MIN:
            continue
        # 用零空间方向上占比最大的分量给这个自由度命名，便于现场排查
        axis = int(np.argmax(np.abs(eigenvectors[:, index])))
        name = _AXIS_NAMES[axis]
        if name not in degenerate:
            degenerate.append(name)
    condition = float(largest / max(float(eigenvalues.min()), 1e-12))
    return degenerate, condition


def _correspondence_normals(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    transform: np.ndarray,
    max_distance: float,
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """取 source 变换后与 target 的最近邻对应，返回 (对应点, target 法向)。"""
    moved = np.asarray(source.points) @ transform[:3, :3].T + transform[:3, 3]
    target_points = np.asarray(target.points)
    if len(moved) == 0 or len(target_points) == 0:
        return None

    # 可观测性需要表面的法向，不能复用配准小半径内仅一至三个点算出的方向。
    # 沿用三十邻点的局部拟合，用独立副本避免修改 ICP 的法向。
    normal_cloud = o3d.geometry.PointCloud(target)
    normal_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(knn=30))
    target_normals = np.asarray(normal_cloud.normals)

    tree = o3d.geometry.KDTreeFlann(target)
    step = max(1, len(moved) // 8000)
    matched_points: List[np.ndarray] = []
    matched_normals: List[np.ndarray] = []
    for point in moved[::step]:
        found, index, squared = tree.search_knn_vector_3d(point, 1)
        if found and squared[0] <= max_distance ** 2:
            matched_points.append(point)
            matched_normals.append(target_normals[index[0]])
    if len(matched_points) < 100:
        return None
    return np.asarray(matched_points), np.asarray(matched_normals)


# ---------------------------------------------------------------------------
# 标定
# ---------------------------------------------------------------------------
def calibrate(
    points_a: np.ndarray,
    points_b: np.ndarray,
    params: Optional[MergeParams] = None,
    source: str = "",
    camera_a: str = "",
    camera_b: str = "",
) -> Extrinsic:
    """求雷达 B → A 的外参。

    三步：地平面对齐定初值 → FPFH + RANSAC 全局配准 → point-to-plane ICP 精配。
    最后做退化性检测并把结论写进质量报告，欠约束时**不静默通过**。

    Args:
        points_a: 雷达 A 的点云（基准坐标系）。
        points_b: 雷达 B 的点云。
        params: 阈值集合。
        source: 标定数据来源说明，写入标定文件便于追溯。
        camera_a: 基准雷达 A 标识；投用于在线双雷达分析时必须填写。
        camera_b: 被变换雷达 B 标识；投用于在线双雷达分析时必须填写。
    Returns:
        Extrinsic，其 ``quality.trustworthy`` 为 False 时不应直接投用。
    Raises:
        ValueError: 任一侧点数过少，无法配准。
    """
    params = params or MergeParams()
    if len(points_a) < 500 or len(points_b) < 500:
        raise ValueError(
            f"标定点数过少（A={len(points_a)}, B={len(points_b)}），至少各需 500 点"
        )

    cloud_a, feature_a = _prepare(points_a, params)
    cloud_b, feature_b = _prepare(points_b, params)

    init = _ground_alignment(points_a, points_b)
    ransac_distance = params.voxel_size * params.ransac_distance_factor
    global_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        cloud_b, cloud_a, feature_b, feature_a, True, ransac_distance,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
        [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(ransac_distance),
        ],
        o3d.pipelines.registration.RANSACConvergenceCriteria(
            params.ransac_iterations, params.ransac_confidence
        ),
    )
    # 全局配准偶尔会跑偏；地平面初值本身已定住 3 个自由度，取更优者继续
    coarse = global_result.transformation if global_result.fitness > 0 else init
    if _evaluate(cloud_b, cloud_a, init, ransac_distance) > global_result.fitness:
        coarse = init

    icp_distance = params.voxel_size * params.icp_distance_factor
    icp = o3d.pipelines.registration.registration_icp(
        cloud_b, cloud_a, icp_distance, coarse,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=params.icp_max_iterations
        ),
    )

    degenerate, condition = _check_degeneracy(
        cloud_b, cloud_a, icp.transformation, icp_distance
    )
    warnings: List[str] = []
    if icp.fitness < params.min_fitness:
        warnings.append(
            f"重叠比例仅 {icp.fitness:.1%}（阈值 {params.min_fitness:.0%}），"
            "两台雷达共视区可能不足"
        )
    if icp.inlier_rmse > params.max_inlier_rmse:
        warnings.append(
            f"配准残差 {icp.inlier_rmse*1000:.0f}mm 超过阈值 "
            f"{params.max_inlier_rmse*1000:.0f}mm"
        )
    if degenerate:
        warnings.append(
            f"自由度 {', '.join(degenerate)} 欠约束 —— 重叠区可能只有平坦地面，"
            "配准会在该方向自由滑动，请在共视区放置立体物体后重新标定"
        )

    quality = CalibrationQuality(
        fitness=float(icp.fitness),
        inlier_rmse=float(icp.inlier_rmse),
        correspondence_count=int(np.asarray(icp.correspondence_set).shape[0]),
        degenerate_axes=degenerate,
        condition_number=condition,
        warnings=warnings,
    )
    for message in warnings:
        logger.warning("标定：%s", message)
    logger.info(
        "标定完成：重叠 %.1f%%，残差 %.1fmm，%s",
        quality.fitness * 100, quality.inlier_rmse * 1000,
        "可信" if quality.trustworthy else "不可信（见告警）",
    )
    extrinsic = Extrinsic(
        matrix=np.asarray(icp.transformation, dtype=float),
        quality=quality,
        source=source,
        camera_a=camera_a,
        camera_b=camera_b,
    )
    # 记录标定当时的重叠残差，供每帧漂移判定用作自适应基线
    baseline, _ = _overlap_stats(
        points_a, extrinsic.apply(points_b), params, extrinsic.weighbridge_plane_a
    )
    extrinsic.baseline_residual = baseline
    if baseline is not None:
        logger.info("标定基线残差：%.1fmm（每帧漂移判定以此为准）", baseline * 1000)
    return extrinsic


def _evaluate(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    transform: np.ndarray,
    max_distance: float,
) -> float:
    """计算给定变换下的配准重叠率，评估失败时返回零。"""
    try:
        result = o3d.pipelines.registration.evaluate_registration(
            source, target, max_distance, transform
        )
        return float(result.fitness)
    except RuntimeError as exc:
        # Open3D 对退化/空对应场景可能抛 RuntimeError，可按零重叠降级；
        # 编程错误等其它异常必须继续抛出，不能伪装成一次正常的低质量标定。
        logger.warning("配准重叠率评估失败，按 0 处理: %s", exc, exc_info=True)
        return 0.0


# ---------------------------------------------------------------------------
# 合并
# ---------------------------------------------------------------------------
@dataclass
class MergeResult:
    """双雷达融合结果及上线质量门禁的审计指标。"""
    points: np.ndarray
    n_from_a: int
    n_from_b: int
    n_after_uniform: int
    overlap_rmse: Optional[float]
    overlap_match_ratio: Optional[float] = None
    warnings: List[str] = field(default_factory=list)
    n_finite_a: Optional[int] = None
    n_finite_b: Optional[int] = None
    n_filtered_a: Optional[int] = None
    n_filtered_b: Optional[int] = None
    n_combined: Optional[int] = None
    quality_passed: Optional[bool] = None
    reason: Optional[str] = None
    # 重叠残差/匹配率导出的观测项。``enforce_overlap_gate=False`` 时它们不进
    # ``warnings``、不影响 ``quality_passed``，仅作为审计记录随结果返回。
    overlap_notes: List[str] = field(default_factory=list)
    # 仅供共同场景包装层在体素化前选择整厢来源；不进入 JSON/持久化。
    filtered_points_a: Optional[np.ndarray] = field(default=None, repr=False, compare=False)
    aligned_filtered_points_b: Optional[np.ndarray] = field(default=None, repr=False, compare=False)

    def __post_init__(self) -> None:
        """兼容历史构造方式并补齐拒绝原因。"""
        # 保持对历史调用方直接构造 MergeResult 的兼容：未显式传质量状态时，
        # 仍由已有 warnings 推导，而不是把带严重告警的结果默认为通过。
        if self.quality_passed is None:
            self.quality_passed = not self.warnings
        if not self.quality_passed and not self.reason:
            self.reason = "；".join(self.warnings) or "融合质量门禁未通过"

    def to_dict(self) -> Dict[str, Any]:
        """导出融合点数、重叠质量与门禁结论。

        Returns:
            可 JSON 序列化的融合质量字典。
        """
        return {
            "mode": "fused" if self.quality_passed else "rejected",
            "qualityPassed": self.quality_passed,
            "reason": self.reason,
            "pointsFromA": self.n_from_a,
            "pointsFromB": self.n_from_b,
            "pointsFiniteA": self.n_finite_a,
            "pointsFiniteB": self.n_finite_b,
            "pointsFilteredA": self.n_filtered_a,
            "pointsFilteredB": self.n_filtered_b,
            "pointsCombined": self.n_combined,
            "pointsMerged": self.n_after_uniform,
            "overlapRmse": (
                round(self.overlap_rmse, 5) if self.overlap_rmse is not None else None
            ),
            "overlapMatchRatio": (
                round(self.overlap_match_ratio, 4)
                if self.overlap_match_ratio is not None else None
            ),
            "warnings": list(self.warnings),
            "overlapNotes": list(self.overlap_notes),
        }


def merge(
    points_a: np.ndarray,
    points_b: np.ndarray,
    extrinsic: Extrinsic,
    params: Optional[MergeParams] = None,
) -> MergeResult:
    """把 B 变换到 A 坐标系后合并，并做密度均匀化与质量检查。

    体素去重不是可选项：重叠区双倍密度会让底板面密度峰、拉筋找峰、
    ``occupancy_extent`` 门槛和 DBSCAN 全部系统性偏移（详见模块头部注释）。

    Returns:
        MergeResult。``overlap_rmse`` 显著大于标定残差时说明外参已漂移，
        或采集时车辆并非静止。
    """
    params = params or MergeParams()
    raw_a = np.asarray(points_a, dtype=float)
    raw_b = np.asarray(points_b, dtype=float)
    filtered_a, n_finite_a = _light_filter(raw_a, params)
    filtered_b, n_finite_b = _light_filter(raw_b, params)

    # 固定安装场景只应用离线标定的 B→A 外参。这里绝不能做逐帧 ICP：
    # 车头共视区通常光滑且欠约束，ICP 很容易沿平面滑动到低残差的错误位姿。
    transformed_b = extrinsic.apply(filtered_b)
    combined = np.vstack([filtered_a, transformed_b])

    # 密度均匀化：每个体素只保留一个代表点，重叠与非重叠区密度自然拉平
    uniform = np.asarray(
        _to_cloud(combined).voxel_down_sample(params.merge_voxel_size).points
    )

    overlap_rmse, match_ratio = _overlap_stats(
        filtered_a, transformed_b, params, extrinsic.weighbridge_plane_a
    )
    warnings: List[str] = []
    overlap_notes: List[str] = []
    for camera, filtered, finite in (
        ("A", filtered_a, n_finite_a),
        ("B", filtered_b, n_finite_b),
    ):
        retention = len(filtered) / finite if finite else 0.0
        if len(filtered) < params.min_filtered_points:
            warnings.append(
                f"雷达 {camera} 轻滤后仅 {len(filtered)} 点（至少 "
                f"{params.min_filtered_points} 点），无法证明车辆结构完整"
            )
        elif retention < params.min_prefilter_retention:
            warnings.append(
                f"雷达 {camera} 轻滤仅保留 {retention:.1%}（阈值 "
                f"{params.min_prefilter_retention:.0%}），稀疏车头可能被误删"
            )
    if overlap_rmse is None:
        overlap_notes.append("两片点云没有可测的重叠区，无法校验外参是否仍然有效")
    else:
        # 门槛取"标定基线的若干倍"与绝对上限中的较小者：前者适配现场噪声，
        # 后者兜底防止基线本身就很差时永远不报警
        limit = params.max_overlap_residual
        if extrinsic.baseline_residual:
            limit = min(
                limit, extrinsic.baseline_residual * params.drift_residual_factor
            )
        if overlap_rmse > limit:
            overlap_notes.append(
                f"重叠区 P{params.residual_percentile:.0f} 残差 {overlap_rmse*1000:.0f}mm "
                f"超过门槛 {limit*1000:.0f}mm：外参可能已漂移，"
                "或采集时车辆并非静止"
            )
        # 匹配率是识别"沿平面平移"的关键：地面点错开后仍能就近匹配，残差不涨，
        # 但立柱/护栏/车壁这些立体结构会彻底失配
        if match_ratio is not None and match_ratio < params.min_overlap_match_ratio:
            overlap_notes.append(
                f"重叠区匹配率仅 {match_ratio:.0%}（阈值 "
                f"{params.min_overlap_match_ratio:.0%}）：外参可能已沿地面方向漂移"
            )
    if params.enforce_overlap_gate:
        warnings.extend(overlap_notes)
    for message in warnings:
        logger.warning("合并：%s", message)
    if not params.enforce_overlap_gate:
        for message in overlap_notes:
            logger.info("合并观测（不参与门禁）：%s", message)

    quality_passed = not warnings
    reason = None if quality_passed else "；".join(warnings)
    logger.info(
        "点云合并：A %d→%d + B %d→%d → 去重后 %d 点%s",
        len(raw_a), len(filtered_a), len(raw_b), len(filtered_b), len(uniform),
        f"，重叠残差 {overlap_rmse*1000:.1f}mm" if overlap_rmse is not None else "",
    )
    return MergeResult(
        points=uniform,
        n_from_a=len(raw_a),
        n_from_b=len(raw_b),
        n_after_uniform=len(uniform),
        overlap_rmse=overlap_rmse,
        overlap_match_ratio=match_ratio,
        warnings=warnings,
        n_finite_a=n_finite_a,
        n_finite_b=n_finite_b,
        n_filtered_a=len(filtered_a),
        n_filtered_b=len(filtered_b),
        n_combined=len(combined),
        quality_passed=quality_passed,
        reason=reason,
        overlap_notes=overlap_notes,
        filtered_points_a=filtered_a,
        aligned_filtered_points_b=transformed_b,
    )


def _overlap_stats(
    points_a: np.ndarray,
    transformed_b: np.ndarray,
    params: MergeParams,
    ground_plane: Optional[np.ndarray] = None,
) -> Tuple[Optional[float], Optional[float]]:
    """重叠区的最近邻残差与匹配率。

    **只看残差会漏掉沿平面的平移**：地面是场景里最大的结构，水平错开 35cm 后
    每个点仍能在地面上找到很近的邻居，残差几乎不涨（实测 35cm 漂移只测出
    44mm）。匹配率能补上这一点 —— 立柱、护栏、车壁这些立体结构一旦错开就
    彻底失配，匹配率会明显下降。两个指标必须一起看。

    Args:
        points_a: 基准坐标系下的 A 点云。
        transformed_b: 已变换到 A 坐标系的 B 点云。
        params: 阈值集合。
        ground_plane: 可选的可信地面基准 ``[a,b,c,d]``（法向朝上、已单位化）。
            给定时用它划分"立体结构"，不再逐帧拟合最大平面。

    Returns:
        (残差 RMSE, 匹配率)；重叠样本不足时返回 (None, None)。
    """
    if len(points_a) == 0 or len(transformed_b) == 0:
        return None, None

    # 只统计落在 A 覆盖范围内的 B 点：B 大部分点本就在 A 视野之外、永远匹配不上，
    # 把它们算进来会让匹配率恒偏低（实测真值外参下也只有 47%），失去判别力
    lower = points_a.min(axis=0)
    upper = points_a.max(axis=0)
    inside = np.all((transformed_b >= lower) & (transformed_b <= upper), axis=1)
    candidates = transformed_b[inside]
    if len(candidates) < 100:
        return None, None

    # 再只保留立体结构：地面对平面内平移不敏感（错开 35cm 仍能就近匹配到地面，
    # 残差几乎不涨），必须靠护栏/立柱/车壁这类竖直面才能发现漂移。
    #
    # 地面基准优先用调用方给的标定面。逐帧拟合"最大平面"在真实地磅现场会命中
    # 磅台周边地坪而不是磅台面：实测两者相差 0.76m，之后"离地 >0.15m 才算立体
    # 结构"就把整个磅台面也留了下来 —— 保留的 126906 个"结构点"里有 104545 个
    # （82.4%）其实是磅台面，样本仍被平面主导，平面内漂移照样看不见。
    ground = ground_plane
    if ground is None and params.filter_ground_for_overlap:
        ground = _dominant_ground(points_a)
    if ground is not None:
        height = candidates @ ground[:3] + ground[3]
        structure = candidates[height > params.structure_height_min]
        if len(structure) >= 100:
            candidates = structure

    tree = o3d.geometry.KDTreeFlann(_to_cloud(points_a))
    threshold = params.voxel_size * params.icp_distance_factor
    step = max(1, len(candidates) // 5000)   # 采样即可，重叠残差是统计量
    sample = candidates[::step]
    all_distances: List[float] = []
    matched: List[float] = []
    for point in sample:
        found, _, squared = tree.search_knn_vector_3d(point, 1)
        if not found:
            continue
        distance = float(np.sqrt(squared[0]))
        all_distances.append(distance)
        if distance <= threshold:
            matched.append(distance)
    if len(all_distances) < 50 or len(matched) < 50:
        return None, None
    match_ratio = len(matched) / len(all_distances)
    # “残差”只描述实际重叠对应点。互补视角会在同一三维包围盒内看到彼此不可见
    # 的车厢内外表面；把这些非重叠点的最近邻距离算进 P90，会把正确外参误报成
    # 数米漂移。非重叠或沿平面漂移仍由 match_ratio 单独约束，不能把匹配率删掉。
    return float(np.percentile(matched, params.residual_percentile)), match_ratio
