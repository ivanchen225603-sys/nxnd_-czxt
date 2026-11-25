"""应用层持久化抽象和结果；不依赖 SQL 驱动、配置或文件系统。"""

from datetime import datetime
from pathlib import Path
from dataclasses import dataclass
from typing import Any, Iterable, Literal, Mapping, Protocol


class DimensionStore(Protocol):
    """车辆尺寸及空车档案的持久化端口。"""

    enabled: bool

    def get_workflow_plate(self, workflow_id: str) -> str | None:
        """分析前读取已确认车牌，保证空车基准与归档使用同一车辆身份。"""
        ...

    def persist_vehicle_dimensions(
        self,
        payload: Mapping[str, Any],
        empty_profile: Mapping[str, Any] | None = None,
    ) -> "PersistResult":
        """幂等写入一次车辆尺寸，可同时更新同车空车档案。"""
        ...

    def health(self) -> dict[str, Any]:
        """探测持久化后端是否可用。"""
        ...

    def complete_pending_empty_profile(
        self, payload: Mapping[str, Any], empty_profile: Mapping[str, Any],
    ) -> "PersistResult | Literal['pending', 'unidentified']":
        """仅在本车次车牌已确认时保存延迟归档。"""
        ...


@dataclass(frozen=True)
class PersistResult:
    """尺寸持久化结果，区分新插入和幂等重复。"""
    record_id: int
    inserted: bool
    idempotency_key: str
    empty_vehicle_profile: Mapping[str, Any] | None = None

    def to_dict(self) -> dict[str, Any]:
        """导出供服务响应使用的持久化结果。"""
        result = {
            "status": "inserted" if self.inserted else "duplicate",
            "recordId": self.record_id,
            "idempotencyKey": self.idempotency_key,
        }
        if self.empty_vehicle_profile is not None:
            result["emptyVehicleProfile"] = dict(self.empty_vehicle_profile)
        return result


@dataclass(frozen=True)
class StagedEmptyVehicleProfile:
    """一份完整且不可变的 PCD 候选，等待登记到 DM8。"""

    point_cloud_path: str
    point_cloud_sha256: str
    point_cloud_size: int
    point_count: int
    scan_time: datetime

    def to_mapping(self) -> dict[str, Any]:
        """转换为数据库持久化层接受的空车档案映射。"""
        return {
            "pointCloudPath": self.point_cloud_path,
            "pointCloudSha256": self.point_cloud_sha256,
            "pointCloudSize": self.point_cloud_size,
            "pointCount": self.point_count,
            "scanTime": self.scan_time,
        }


@dataclass(frozen=True)
class PendingEmptyVehicleProfile:
    """待归档的尺寸载荷和已保存的本次净化点云。"""

    receipt_path: Path
    payload: Mapping[str, Any]
    profile: StagedEmptyVehicleProfile


class DisabledEmptyVehicleProfileStorage:
    """空车点云归档关闭时使用的无操作标记实现。"""

    enabled = False
    root: Path | None = None


class DisabledDimensionStore:
    """数据库关闭时使用的显式禁用实现。"""
    enabled = False

    def get_workflow_plate(self, workflow_id: str) -> str | None:
        """关闭持久化时不访问数据库。"""
        return None

    def persist_vehicle_dimensions(
        self,
        payload: Mapping[str, Any],
        empty_profile: Mapping[str, Any] | None = None,
    ) -> PersistResult:
        """拒绝写入，防止配置关闭时误认为已经持久化。"""
        raise RuntimeError("数据库直写未启用")

    def health(self) -> dict[str, Any]:
        """返回无需探测外部数据库的禁用状态。"""
        return {"enabled": False, "status": "disabled"}


class EmptyProfileStorage(Protocol):
    """应用仅依赖档案操作，不依赖目录布局或复制实现。"""
    enabled: bool
    root: Path | None

    def stage(self, source_pcd: str | Path, license_plate: str, stable_scan_id: str,
              point_count: int, scan_time: datetime) -> StagedEmptyVehicleProfile:
        """准备本次最终点云档案。"""
        ...

    def finalize(self, staged_path: str, persistence: Mapping[str, Any]) -> None:
        """完成档案替换并清理旧文件。"""
        ...

    def defer(self, source_pcd: str | Path, payload: Mapping[str, Any],
              point_count: int, scan_time: datetime) -> None:
        """保存迟到车牌对应的档案候选，独立于原始点云保留期。"""
        ...

    def pending(self) -> Iterable[PendingEmptyVehicleProfile]:
        """读取尚未办结的档案候选。"""
        ...

    def finish_pending(self, job: PendingEmptyVehicleProfile, result: PersistResult | None) -> None:
        """按数据库回执办结，或删去已确认无法识别的点云候选。"""
        ...
