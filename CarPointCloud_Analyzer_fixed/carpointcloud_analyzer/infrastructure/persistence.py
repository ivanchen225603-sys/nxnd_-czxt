"""Analyzer 直接写入达梦 DM8 的车辆尺寸持久化层。

本模块只依赖达梦官方 ``dmPython`` DB-API 驱动。数据库开关关闭时不会导入驱动，
因此开发机和纯算法测试不需要安装 DM 客户端。每一条车辆记录及其拉筋、钢丝绳、
高低板明细在同一个事务中提交；同一幂等键重试不会重复写入。
"""

from __future__ import annotations

import hashlib
import importlib
import json
import logging
import os
from pathlib import Path

from carpointcloud_analyzer.domain import stage_timing
from carpointcloud_analyzer.domain.utils.environment import env_bool as _env_bool
from carpointcloud_analyzer.application.ports import (
    DimensionStore as DimensionStore, PersistResult as PersistResult,
    DisabledDimensionStore as DisabledDimensionStore,
)
from dataclasses import dataclass
from datetime import datetime
from decimal import Decimal, ROUND_HALF_UP
from typing import Any, Callable, Literal, Mapping


_ONE_DECIMAL = Decimal("0.1")
logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class Dm8Settings:
    """Analyzer 直连达梦 DM8 所需的环境配置。"""
    enabled: bool = False
    host: str = "127.0.0.1"
    port: int = 5236
    user: str = ""
    password: str = ""
    schema: str = ""
    connect_timeout_seconds: int = 10

    @classmethod
    def from_env(cls) -> "Dm8Settings":
        """从 ``CARPOINT_DB_*`` 环境变量读取设置。"""
        return cls(
            enabled=_env_bool("CARPOINT_DB_ENABLED", False),
            host=os.getenv("CARPOINT_DB_HOST", "127.0.0.1").strip(),
            port=int(os.getenv("CARPOINT_DB_PORT", "5236")),
            user=os.getenv("CARPOINT_DB_USER", "").strip(),
            password=os.getenv("CARPOINT_DB_PASSWORD", ""),
            schema=os.getenv("CARPOINT_DB_SCHEMA", "").strip(),
            connect_timeout_seconds=int(os.getenv("CARPOINT_DB_CONNECT_TIMEOUT_SECONDS", "10")),
        )


class Dm8DimensionStore:
    """通过达梦官方驱动事务性写入车辆尺寸和空车档案。"""
    enabled = True

    def __init__(
        self,
        settings: Dm8Settings,
        connect: Callable[..., Any] | None = None,
    ) -> None:
        """创建 DM8 存储并延迟到实际操作时建立连接；未注入连接工厂时使用 dmPython。"""
        if connect is None:
            driver = importlib.import_module("dmPython")
            logger.info("DM8驱动 module=%s version=%s threadsafety=%s",
                        getattr(driver, "__file__", None), getattr(driver, "version", None),
                        getattr(driver, "threadsafety", None))
            connect = driver.connect
        self._settings = settings
        self._connect_fn = connect
        prefix = f"{settings.schema.upper()}." if settings.schema else ""
        self._record_table = prefix + "VehicleDimensionRecord"
        self._rib_table = prefix + "VehicleDimensionRib"
        self._wire_table = prefix + "VehicleDimensionWire"
        self._plate_table = prefix + "VehicleDimensionHighLowPlate"
        self._empty_profile_table = prefix + "EmptyVehicleProfile"
        self._arrival_table = prefix + "VehicleArrival"

    @classmethod
    def from_env(cls) -> DimensionStore:
        """按环境开关返回 DM8 实现或禁用实现。"""
        settings = Dm8Settings.from_env()
        return cls(settings) if settings.enabled else DisabledDimensionStore()

    def health(self) -> dict[str, Any]:
        """连接 DM8 并读取空车档案表以验证模式可用。"""
        connection = self._connect()
        try:
            cursor = connection.cursor()
            try:
                cursor.execute(
                    f"SELECT TOP 1 LicensePlate FROM {self._empty_profile_table}"
                )
                cursor.fetchone()
            finally:
                cursor.close()
        finally:
            connection.close()
        return {"enabled": True, "status": "ok"}


    def persist_vehicle_dimensions(
        self, payload: Mapping[str, Any], empty_profile: Mapping[str, Any] | None = None,
    ) -> PersistResult:
        """一次事务写主记录、明细和空车档案；数据库异常直接回滚并上抛。"""
        return self._persist(payload, empty_profile, wait_for_plate=False)

    def complete_pending_empty_profile(
        self, payload: Mapping[str, Any], empty_profile: Mapping[str, Any],
    ) -> PersistResult | Literal["pending", "unidentified"]:
        return self._persist(payload, empty_profile, wait_for_plate=True)

    def _persist(
        self, payload: Mapping[str, Any], empty_profile: Mapping[str, Any] | None,
        *, wait_for_plate: bool,
    ) -> PersistResult | Literal["pending", "unidentified"]:
        record = _normalize_payload(payload)
        connection = self._connect()
        try:
            with stage_timing.stage("db-write-record"):
                arrival = self._read_workflow_plate(connection, record["workflowId"], lock=True)
                plate = self._confirmed_plate(arrival)
                if wait_for_plate and not plate:
                    connection.rollback()
                    if arrival and str(arrival[1] or "").upper() not in ("FAILED", "SUCCEEDED"):
                        return "pending"
                    return "unidentified"
                if plate:
                    record["licensePlate"] = plate
                payload_hash = _payload_hash(record)
                existing = self._find_existing(connection, record)
                if existing is not None:
                    persisted = self._resolve_existing(existing, payload_hash, record)
                    record_id = persisted.record_id
                    if plate:
                        self._backfill_dimension_plate(connection, record_id, plate)
                else:
                    record_id = self._insert_transaction(connection, record, payload_hash)
            profile_result = None
            if empty_profile is not None:
                with stage_timing.stage("db-write-profile"):
                    profile_result = self._upsert_empty_profile(
                        connection, record_id, record, empty_profile, require_file=wait_for_plate)
            with stage_timing.stage("db-commit"):
                connection.commit()
            return PersistResult(record_id, existing is None, record['idempotencyKey'], profile_result)
        except Exception:
            with stage_timing.stage("db-rollback"):
                connection.rollback()
            raise
        finally:
            with stage_timing.stage("db-close"):
                connection.close()

    def _connect(self):
        """创建关闭自动提交的 DM8 连接。"""
        with stage_timing.stage("db-connect"):
            return self._connect_fn(
                user=self._settings.user,
                password=self._settings.password,
                server=self._settings.host,
                port=self._settings.port,
                autoCommit=False,
                connection_timeout=self._settings.connect_timeout_seconds,
            )

    def _find_existing(
        self,
        connection,
        record: Mapping[str, Any],
    ) -> tuple[int, str | None] | None:
        """按幂等键、工作流或配对标识查找既有记录。"""
        identities = [("IdempotencyKey", record["idempotencyKey"])]
        if record.get("workflowId"):
            identities.append(("WorkflowId", record["workflowId"]))
        if record.get("pairId"):
            identities.append(("PairId", record["pairId"]))

        cursor = connection.cursor()
        try:
            for column, value in identities:
                cursor.execute(
                    f"SELECT Id, PayloadHash FROM {self._record_table} "
                    f"WHERE {column} = ?",
                    value,
                )
                row = cursor.fetchone()
                if row:
                    return int(row[0]), str(row[1]) if row[1] is not None else None
            return None
        finally:
            cursor.close()

    def get_workflow_plate(self, workflow_id: str) -> str | None:
        """分析前只读查询；入库事务仍再次核实，不把此读当作写入凭据。"""
        connection = self._connect()
        try:
            with stage_timing.stage("db-query-plate"):
                return self._confirmed_plate(self._read_workflow_plate(connection, workflow_id))
        finally:
            with stage_timing.stage("db-close"):
                connection.close()

    def _read_workflow_plate(self, connection, workflow_id: str | None, *, lock: bool = False):
        """写入与 WebApi 识牌使用同一来车行锁，分析前的只读查询不拿锁。"""
        if not workflow_id:
            return None

        cursor = connection.cursor()
        try:
            if lock:
                cursor.execute(
                    f"UPDATE {self._arrival_table} SET UpdatedAt = UpdatedAt WHERE SessionId = ?",
                    workflow_id,
                )
            cursor.execute(
                f"SELECT LicensePlate, PlateStatus FROM {self._arrival_table} "
                "WHERE SessionId = ?",
                workflow_id,
            )
            return cursor.fetchone()
        finally:
            cursor.close()

    @staticmethod
    def _confirmed_plate(row) -> str | None:
        if row and str(row[1] or "").upper() == "SUCCEEDED":
            plate = str(row[0] or "").strip().upper()
            if plate and plate != "UNKNOWN":
                return plate
        return None

    def _backfill_dimension_plate(self, connection, record_id: int, plate: str) -> None:
        cursor = connection.cursor()
        try:
            cursor.execute(
                f"UPDATE {self._record_table} SET LicensePlate = ?, UpdatedAt = ? WHERE Id = ? "
                "AND (LicensePlate IS NULL OR TRIM(LicensePlate) = '' OR UPPER(LicensePlate) = 'UNKNOWN')",
                plate, datetime.now(), record_id,
            )
        finally:
            cursor.close()


    @staticmethod
    def _resolve_existing(
        existing: tuple[int, str | None],
        payload_hash: str,
        record: Mapping[str, Any],
    ) -> PersistResult:
        """同一业务标识只接受相同载荷，返回幂等重复结果。"""
        record_id, stored_hash = existing
        if stored_hash != payload_hash:
            raise ValueError(
                "幂等键/workflowId/pairId 已被不同车辆尺寸载荷使用："
                f"{record['idempotencyKey']}"
            )
        return PersistResult(record_id, False, str(record["idempotencyKey"]))

    def _insert_transaction(
        self,
        connection,
        record: Mapping[str, Any],
        payload_hash: str,
    ) -> int:
        """插入车辆主记录及全部结构明细并返回主键。"""
        cursor = connection.cursor()
        now = datetime.now()
        dimensions = record["dimensions"]
        try:
            cursor.execute(
                f"INSERT INTO {self._record_table} "
                "(IdempotencyKey, WorkflowId, PairId, PayloadHash, LicensePlate, CameraIdentifier, "
                "GetTime, DetectStatus, CabToTrailerFrontDist, TrailerLength, "
                "TrailerWidth, TrailerBodyHeight, TrailerBottomToGroundHeight, "
                "HighLowPlateHeight, CreatedAt, UpdatedAt, SourceFile, "
                "ResultJsonPath, ResultImagePath, ActualRibCount, MaximumWidthCm) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                record["idempotencyKey"], record["workflowId"], record["pairId"],
                payload_hash, record["licensePlate"],
                record["cameraIdentifier"], record["getTime"],
                record["detectStatus"], dimensions["cabToTrailerFrontDist"],
                dimensions["trailerLength"], dimensions["trailerWidth"],
                dimensions["trailerBodyHeight"],
                dimensions["trailerBottomToGroundHeight"],
                dimensions["highLowPlateHeight"], now, now,
                record["sourceFile"], record["resultJsonPath"],
                record["resultImagePath"],
                record.get("actualRibCount"),
                record.get("maximumWidthCm"),
            )
        finally:
            cursor.close()

        record_id = self._find_existing(connection, record)[0]
        self._insert_details(connection, self._rib_table, "RibId", "DistToPrev", record_id, record["ribs"])
        self._insert_details(connection, self._wire_table, "WireId", "DistToPrev", record_id, record["wires"])

        cursor = connection.cursor()
        try:
            for plate in record["highLowPlates"]:
                cursor.execute(
                    f"INSERT INTO {self._plate_table} "
                    "(DimensionRecordId, PlateId, HighestToGround, LowestToGround, LengthCm) "
                    "VALUES (?, ?, ?, ?, ?)",
                    record_id, plate["plateId"], plate["highestToGround"],
                    plate["lowestToGround"],
                    plate.get("lengthCm"),
                )
        finally:
            cursor.close()
        return record_id

    def _upsert_empty_profile(
        self,
        connection,
        dimension_record_id: int,
        record: Mapping[str, Any],
        profile: Mapping[str, Any],
        *, require_file: bool = False,
    ) -> dict[str, Any]:
        """仅以较新的成功空车扫描覆盖同车牌档案。"""
        cursor = connection.cursor()
        try:
            cursor.execute(
                f"UPDATE {self._empty_profile_table} SET UpdatedAt = UpdatedAt WHERE LicensePlate = ?",
                record["licensePlate"],
            )
            cursor.execute(
                f"SELECT Id, PointCloudPath, ScanTime, WorkflowId FROM {self._empty_profile_table} "
                "WHERE LicensePlate = ?",
                record["licensePlate"],
            )
            row = cursor.fetchone()
        finally:
            cursor.close()

        previous_path = str(row[1]) if row and row[1] is not None else None
        previous_scan_time = row[2] if row and row[2] is not None else None
        previous_workflow = str(row[3]) if row and row[3] is not None else None
        scan_time = _business_datetime(profile["scanTime"])
        should_update = previous_path != str(profile["pointCloudPath"]) and (
            row is None or previous_scan_time is None or scan_time > previous_scan_time
            or (scan_time == previous_scan_time and previous_workflow == record["workflowId"]))
        if should_update and require_file and not Path(str(profile["pointCloudPath"])).is_file():
            raise FileNotFoundError("待归档空车点云缺失")
        dimensions = record["dimensions"]
        now = datetime.now()

        if row is None:
            cursor = connection.cursor()
            try:
                cursor.execute(
                    f"INSERT INTO {self._empty_profile_table} "
                    "(LicensePlate, DimensionRecordId, WorkflowId, PairId, CameraIdentifier, "
                    "PointCloudPath, PointCloudSha256, PointCloudSize, PointCount, ScanTime, "
                    "CabToTrailerFrontDist, TrailerLength, TrailerWidth, TrailerBodyHeight, "
                    "TrailerBottomToGroundHeight, HighLowPlateHeight, CreatedAt, UpdatedAt) "
                    "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                    record["licensePlate"], dimension_record_id, record["workflowId"],
                    record["pairId"], record["cameraIdentifier"],
                    profile["pointCloudPath"], profile["pointCloudSha256"],
                    profile["pointCloudSize"], profile["pointCount"],
                    scan_time, dimensions["cabToTrailerFrontDist"],
                    dimensions["trailerLength"], dimensions["trailerWidth"],
                    dimensions["trailerBodyHeight"],
                    dimensions["trailerBottomToGroundHeight"],
                    dimensions["highLowPlateHeight"], now, now,
                )
            finally:
                cursor.close()
        elif should_update:
            cursor = connection.cursor()
            try:
                cursor.execute(
                    f"UPDATE {self._empty_profile_table} SET "
                    "DimensionRecordId = ?, WorkflowId = ?, PairId = ?, CameraIdentifier = ?, "
                    "PointCloudPath = ?, PointCloudSha256 = ?, PointCloudSize = ?, PointCount = ?, "
                    "ScanTime = ?, CabToTrailerFrontDist = ?, TrailerLength = ?, TrailerWidth = ?, "
                    "TrailerBodyHeight = ?, TrailerBottomToGroundHeight = ?, HighLowPlateHeight = ?, "
                    "UpdatedAt = ? WHERE LicensePlate = ? AND "
                    "(ScanTime IS NULL OR ScanTime < ? OR (ScanTime = ? AND WorkflowId = ?))",
                    dimension_record_id, record["workflowId"], record["pairId"],
                    record["cameraIdentifier"], profile["pointCloudPath"],
                    profile["pointCloudSha256"], profile["pointCloudSize"],
                    profile["pointCount"], scan_time,
                    dimensions["cabToTrailerFrontDist"], dimensions["trailerLength"],
                    dimensions["trailerWidth"], dimensions["trailerBodyHeight"],
                    dimensions["trailerBottomToGroundHeight"],
                    dimensions["highLowPlateHeight"], now, record["licensePlate"],
                    scan_time, scan_time, record["workflowId"],
                )
            finally:
                cursor.close()

        current_path = (
            str(profile["pointCloudPath"])
            if should_update
            else str(previous_path or "")
        )
        return {
            "updated": should_update,
            "pointCloudPath": current_path,
            "previousPointCloudPath": previous_path,
            "scanTime": (
                scan_time if should_update else previous_scan_time
            ).isoformat(timespec="milliseconds"),
        }

    @staticmethod
    def _insert_details(
        connection,
        table: str,
        id_column: str,
        value_column: str,
        record_id: int,
        details: list[dict[str, Any]],
    ) -> None:
        """批量插入拉筋或钢丝绳明细。"""
        cursor = connection.cursor()
        try:
            for item in details:
                cursor.execute(
                    f"INSERT INTO {table} "
                    f"(DimensionRecordId, {id_column}, {value_column}) "
                    "VALUES (?, ?, ?)",
                    record_id, item["id"], item["value"],
                )
        finally:
            cursor.close()


_DIMENSION_FIELDS = (
    "cabToTrailerFrontDist", "trailerLength", "trailerWidth",
    "trailerBodyHeight", "trailerBottomToGroundHeight", "highLowPlateHeight",
)


def _normalize_payload(payload: Mapping[str, Any]) -> dict[str, Any]:
    """把导出器生成的厘米载荷转换为 DM8 写入值：一位小数 Decimal、本地无时区时间。"""
    raw_get_time = payload.get("getTime")
    get_time = _business_datetime(raw_get_time)
    plates = sorted(payload.get("highLowPlates", []), key=lambda item: int(item["plateId"]))
    actual_rib_count = payload.get("actualRibCount")
    return {
        "idempotencyKey": str(payload["idempotencyKey"]).strip(),
        "workflowId": _optional_text(payload.get("workflowId")),
        "pairId": _optional_text(payload.get("pairId")),
        "cameraIdentifier": str(payload["cameraIdentifier"]).strip(),
        "licensePlate": str(payload.get("licensePlate") or "").strip().upper() or "UNKNOWN",
        "getTime": get_time,
        "getTimeForHash": get_time if raw_get_time is not None else None,
        "detectStatus": payload.get("detectStatus") or "success",
        "dimensions": {name: _distance(payload["dimensions"].get(name)) for name in _DIMENSION_FIELDS},
        "ribs": _details(payload.get("ribs", []), "ribId", "distToPrev"),
        **({"actualRibCount": actual_rib_count} if actual_rib_count is not None else {}),
        **({"maximumWidthCm": _distance(payload["maximumWidthCm"])}
           if payload.get("maximumWidthCm") is not None else {}),
        "wires": _details(payload.get("wires", []), "wireId", "distToPrev"),
        "highLowPlates": [
            {
                "plateId": int(item["plateId"]),
                "highestToGround": _distance(item.get("highestToGround")),
                "lowestToGround": _distance(item.get("lowestToGround")),
                **({"lengthCm": _distance(item["lengthCm"])} if item.get("lengthCm") is not None else {}),
            }
            for item in plates
        ],
        "sourceFile": _optional_text(payload.get("sourceFile")),
        "resultJsonPath": _optional_text(payload.get("resultJsonPath")),
        "resultImagePath": _optional_text(payload.get("resultImagePath")),
    }


def _details(raw: list[Mapping[str, Any]], id_name: str, value_name: str) -> list[dict[str, Any]]:
    """拉筋/钢丝绳明细按编号排序并量化距离。"""
    return sorted(
        ({"id": int(item[id_name]), "value": _distance(item.get(value_name))} for item in raw),
        key=lambda item: item["id"],
    )


def _distance(value: Any) -> Decimal | None:
    """厘米距离按入库精度量化为一位小数。"""
    if value is None:
        return None
    return Decimal(str(value)).quantize(_ONE_DECIMAL, rounding=ROUND_HALF_UP)


def _business_datetime(value: Any) -> datetime:
    """解析 ISO-8601 业务时间并转换成本地无时区时间；缺省取当前时间。"""
    if value is None or not str(value).strip():
        return datetime.now()
    parsed = datetime.fromisoformat(str(value).replace("Z", "+00:00"))
    if parsed.tzinfo is not None:
        parsed = parsed.astimezone().replace(tzinfo=None)
    return parsed


def _optional_text(value: Any) -> str | None:
    """去掉首尾空白，空值统一为 None。"""
    if value is None or not str(value).strip():
        return None
    return str(value).strip()


def _payload_hash(record: Mapping[str, Any]) -> str:
    """生成排除易变诊断路径和 workflow 异步车牌的稳定业务载荷 SHA-256。"""

    def convert(value: Any) -> Any:
        """递归转换 Decimal、时间和容器为稳定 JSON 值。"""
        if isinstance(value, Decimal):
            return format(value, "f")
        if isinstance(value, datetime):
            return value.isoformat(timespec="seconds")
        if isinstance(value, Mapping):
            return {key: convert(item) for key, item in value.items()}
        if isinstance(value, list):
            return [convert(item) for item in value]
        return value

    fingerprint_record = dict(record)
    fingerprint_record.pop("idempotencyKey", None)
    if fingerprint_record.get("workflowId"):
        # workflow 车牌允许从 UNKNOWN 原子回填为真实值，不属于尺寸载荷冲突字段。
        fingerprint_record.pop("licensePlate", None)
    fingerprint_record["getTime"] = fingerprint_record.pop("getTimeForHash", None)
    # 同一 PCD 重试分析会生成新的带时间戳/UUID 的 JSON、PNG 文件名；它们只用于
    # 诊断，不能让已经成功提交的业务记录在响应丢失后变成“幂等键冲突”。
    fingerprint_record.pop("resultJsonPath", None)
    fingerprint_record.pop("resultImagePath", None)
    encoded = json.dumps(
        convert(fingerprint_record),
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
