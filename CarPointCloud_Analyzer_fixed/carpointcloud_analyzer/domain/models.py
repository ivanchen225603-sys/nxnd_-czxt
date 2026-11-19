from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple, TypedDict

import numpy as np


class IntegrationPayloads(TypedDict, total=False):
    """内存业务载荷；只在兼容载荷进入持久化边界时规范化。"""

    vehicleDimensions: dict[str, Any] | None


class AnalysisOutput(TypedDict, total=False):
    """算法交给应用层的结果；缺测字段允许为空，内部不重复探测对象类型。"""

    png: str
    json: str
    pcd: str
    pcdPointCount: int
    syntheticPointCount: int
    integrationPayloads: IntegrationPayloads
    dimensions: dict[str, Any] | None
    measurementStatus: dict[str, Any]
    loadStatus: dict[str, Any]
    wallCompletion: dict[str, Any]
    reviewMeasurements: dict[str, Any]


@dataclass
class RibInfo:
    """一根车厢拉筋的位置、跨度和可选三维轴线拟合质量。"""

    rib_id: int
    y_position: float
    x_center: float
    z_center: float
    x_range: Tuple[float, float]
    z_range: Tuple[float, float]
    point_count: int
    length: float
    height: float
    model_type: Optional[str] = None
    axis_midpoint: Optional[Tuple[float, float, float]] = None
    axis_direction: Optional[Tuple[float, float, float]] = None
    axis_endpoints: Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float]]] = None
    model_inlier_count: Optional[int] = None
    model_rmse: Optional[float] = None
    # 真实轴线内点的横向范围；axis_endpoints 是外推到两壁后的点，不能作为实测证据。
    axis_observed_w_range: Optional[Tuple[float, float]] = None

    def to_dict(self) -> Dict[str, Any]:
        """转换为分析 JSON 使用的可序列化拉筋结构。

        Returns:
            基础位置尺寸，以及存在时的三维几何模型字段。
        """
        result = {
            "rib_id": self.rib_id,
            "y_position": self.y_position,
            "x_center": self.x_center,
            "z_center": self.z_center,
            "x_range": list(self.x_range),
            "z_range": list(self.z_range),
            "point_count": self.point_count,
            "length": self.length,
            "height": self.height,
        }
        if self.model_type is not None:
            result["geometry_model"] = {
                "type": self.model_type,
                "axis_midpoint": list(self.axis_midpoint) if self.axis_midpoint else None,
                "axis_direction": list(self.axis_direction) if self.axis_direction else None,
                "axis_endpoints": (
                    [list(self.axis_endpoints[0]), list(self.axis_endpoints[1])]
                    if self.axis_endpoints else None
                ),
                "inlier_count": self.model_inlier_count,
                "rmse": self.model_rmse,
                "observed_w_range": (list(self.axis_observed_w_range)
                                     if self.axis_observed_w_range is not None else None),
            }
        return result


@dataclass
class VehicleDimensions:
    """以米为单位的整车和车厢长、宽、高测量结果。"""

    vehicle_length: float
    vehicle_width: float
    vehicle_height: float
    carriage_length: float
    carriage_width: float
    carriage_height: float
    carriage_outer_width: Optional[float] = None
    width_measurement: Optional[Dict[str, Any]] = None
    height_measurement: Optional[Dict[str, Any]] = None
    vehicle_height_measurement: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Dict[str, float]]:
        """按 vehicle/carriage 两层结构导出尺寸。

        Returns:
            可直接写入结果 JSON 的尺寸字典，所有数值单位为米。
        """
        return {
            "vehicle": {
                "length": self.vehicle_length,
                "width": self.vehicle_width,
                "height": self.vehicle_height,
            },
            "carriage": {
                "length": self.carriage_length,
                "width": self.carriage_width,
                "height": self.carriage_height,
            },
        }


@dataclass
class CarriageSplitResult:
    """车厢点云子集、纵向边界及边界可信性说明。"""

    carriage_points: np.ndarray
    split_y: float
    carriage_start_y: float
    head_y: float
    tail_y: float
    # True 仅表示 split_y / carriage_start_y 都是两块端壁朝向车厢内部的物理面。
    # 旧轴向兼容分割或端壁扫描不清时为 False，下游不得据此上传内部尺寸/拉筋。
    boundaries_are_inner_faces: bool = False
    boundary_method: str = "unverified"
    boundary_warning: Optional[str] = None
    # 端壁内表面未验证时，True 表示所取壁芯端壁对之间是一段连续实测车体；
    # 只允许按主体侧壁量内长，不代表端壁内表面、车头距或拉筋间距可用。
    continuous_body_range: bool = False
    # 全高度 head_y 继续用于整车包络；以下字段只承载业务要求的离地固定高度测距。
    cab_measurement_head_y: Optional[float] = None
    cab_measurement_height: Optional[float] = None
    cab_measurement_half_band: Optional[float] = None
    cab_measurement_point_count: int = 0
    cab_measurement_method: Optional[str] = None
    cab_measurement_reason: Optional[str] = None
    # 内部拟合模型 L=aW+bH+c；不改变既有 JSON/厘米载荷结构。
    front_wall_plane: Optional[Tuple[float, float, float]] = None
    cab_measurement_front_y: Optional[float] = None
    # 测距截面处前壁位置的来源：下部实测面 / 整面投影 / 分割端点。
    cab_measurement_front_source: Optional[str] = None
    # 分割阶段在较宽搜索带拟合的后内壁实测面；仅供尾端补点歧义时交叉核对。
    rear_wall_plane: Optional[Tuple[float, float, float]] = None
    # 业务测长与结构分割分离；不以缩短后的端点裁车、判空或补全整幅壁面。
    longitudinal_measurement: Optional["LongitudinalMeasurement"] = None
    # 当前点云验证的短矩形箱体外包络；不能直接当作车厢内尺寸。
    observed_outer_box: Optional["BoxCarriageBox"] = None

    @property
    def cab_to_front_wall(self) -> float:
        """业务高度截面的车头前沿到车厢前内壁距离（m）。

        ``head_y`` 保留全高度整车端点语义。正常流水线使用
        ``cab_measurement_head_y``（默认离地 1.40m）；仅旧调用方没有提供新测量时
        兼容退回 ``head_y``。
        """
        measured_head = (
            self.cab_measurement_head_y
            if self.cab_measurement_head_y is not None
            else self.head_y
        )
        front = self.cab_measurement_front_y
        return float(measured_head - (front if front is not None else self.split_y))

    @property
    def cab_measurement_valid(self) -> bool:
        """固定业务高度的车头前沿是否已可靠定位。"""
        return bool(
            self.cab_measurement_head_y is not None
            and self.cab_measurement_height is not None
            and self.cab_measurement_point_count > 0
        )


@dataclass
class LongitudinalMeasurement:
    """当前实测侧壁测线的端点；None 端点表示已尝试但无法可靠测量。"""

    rear_l: Optional[float]
    front_l: Optional[float]
    # W = aL + bH + c；拉筋轴线与同侧同偏移测量面相交，再投影到纵向。
    side_plane: Optional[Tuple[float, float, float]]
    metadata: Dict[str, Any]
    # None表示旧调用方未执行；空/失败记录表示已精定位但缺测，不能回旧轴补值。
    rib_side_projections: Optional[List[Dict[str, Any]]] = None
    # 点和侧壁面仍在原 W/L/H 系；端点及分段统一使用此纵向投影轴。
    longitudinal_axis: Optional[Tuple[float, float, float]] = None

    def project_longitudinal(self, points):
        """将原 W/L/H 点投影到测长轴；无标定时保持原局部 L。"""
        values = np.asarray(points)
        return (values[..., 1] if self.longitudinal_axis is None
                else values @ np.asarray(self.longitudinal_axis))

    @property
    def valid(self) -> bool:
        return bool(self.rear_l is not None and self.front_l is not None
                    and np.isfinite([self.rear_l, self.front_l]).all()
                    and self.front_l > self.rear_l)

    @property
    def length(self) -> Optional[float]:
        return float(self.front_l - self.rear_l) if self.valid else None


@dataclass
class LoadStatusResult:
    """车辆空载、载货或不确定分类及其置信度和诊断指标。"""

    status: str
    confidence: float
    reason: str
    metrics: Dict[str, float]

    @property
    def calculation_allowed(self) -> bool:
        """仅本次判定 Empty 可进入普通空车计算；旧容差元数据不能放行。"""
        return self.status == 'Empty'

    def to_dict(self) -> Dict[str, Any]:
        """转换为稳定的载荷状态 JSON 字段。

        Returns:
            状态、三位小数置信度、原因和判别指标。
        """
        return {
            "status": self.status,
            "confidence": round(self.confidence, 3),
            "reason": self.reason,
            "metrics": self.metrics,
        }


@dataclass(frozen=True)
class BoxCarriageBox:
    """由两个端面重建出的厢体外轮廓（车辆局部 W/L/H 坐标）。"""

    front_l: float          # 靠车头一端的端面 L
    rear_l: float           # 远离车头一端的端面 L
    w_range: Tuple[float, float]
    h_range: Tuple[float, float]
    front_points: int
    rear_points: int

    @property
    def length(self) -> float:
        """观测端面之间的外轮廓长度；不可替代业务内长（m）。"""
        return abs(self.front_l - self.rear_l)

    @property
    def width(self) -> float:
        """厢体宽度（m）。"""
        return self.w_range[1] - self.w_range[0]

    @property
    def height(self) -> float:
        """厢体高度（m）。"""
        return self.h_range[1] - self.h_range[0]

    def to_dict(self) -> Dict[str, Any]:
        """导出重建出的厢体尺寸，供审计与人工复核。"""
        return {
            "frontL": round(self.front_l, 4),
            "rearL": round(self.rear_l, 4),
            "lengthM": round(self.length, 4),
            "widthM": round(self.width, 4),
            "heightM": round(self.height, 4),
            "wRange": [round(v, 4) for v in self.w_range],
            "hRange": [round(v, 4) for v in self.h_range],
            "frontWallPoints": self.front_points,
            "rearWallPoints": self.rear_points,
        }


@dataclass(frozen=True)
class BoxCompletionResult:
    """厢式车厢补全结果。``points`` 全部是合成点，不得参与测量。"""

    points: np.ndarray
    box: Optional[BoxCarriageBox]
    metadata: Dict[str, Any]
