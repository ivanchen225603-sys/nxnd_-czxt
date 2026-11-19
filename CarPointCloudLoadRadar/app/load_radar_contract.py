# -*- coding: utf-8 -*-
"""JT128 与后台的上层合同；不改变判定算法，不加载或激活旧通道基线。"""
from __future__ import annotations

import hashlib
import io
import json
import math
import os
import re
import stat
import tempfile
import unicodedata
import zipfile
from dataclasses import fields, is_dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

import numpy as np

from .detect import (EmptyBaseline, EmptyResult, R_INSUFFICIENT_POINTS,
                     R_EMPTY_FAILED, R_NO_FLOOR, R_FLOOR_NOT_LEVEL, R_TOO_FEW_CORRECTION)
from .pointcloud import AngleCache, Frame
from .settings import BASELINE_SUBDIR, CalibCfg, ChannelCfg, DevicesCfg, PROJECT_ROOT

MAX_ASSET_BYTES = 32 * 1024 * 1024
MAX_EXPANDED_BYTES = 128 * 1024 * 1024
MAX_BASELINE_POINTS = 2_000_000
_IDENTIFIER = re.compile(r"[A-Za-z0-9_.:-]{1,64}\Z")
_SHA256 = re.compile(r"[a-f0-9]{64}\Z")
_REQUIRED_ARRAYS = {"floor_z.npy", "box.npy", "cloud.npy", "created.npy"}


def _identifier(value: Any, name: str) -> str:
    if not isinstance(value, str) or not _IDENTIFIER.fullmatch(value):
        raise ValueError(name + " 必须为 1-64 位合法标识符")
    return value


def _text(value: Any, name: str, limit: int) -> str:
    if (not isinstance(value, str) or not value or value != value.strip() or
            len(value) > limit or any(unicodedata.category(c) == "Cc" for c in value)):
        raise ValueError(name + " 为空、超长或含首尾空白及控制字符")
    return value


def _hash(value: Any, name: str) -> str:
    if not isinstance(value, str) or not _SHA256.fullmatch(value):
        raise ValueError(name + " 必须为完整小写 SHA-256")
    return value


def _number(value: Any, name: str, maximum: Optional[float] = None) -> Optional[float]:
    # 未知数值只能传 null，禁止把 NaN/Infinity 或默认 0 当成成功结果。
    if value is None:
        return None
    if isinstance(value, (bool, str)) or not isinstance(value, (int, float, np.number)):
        raise ValueError(name + " 必须为数字或 null")
    number = float(value)
    if not math.isfinite(number):
        return None
    if number < 0 or maximum is not None and number > maximum:
        raise ValueError(name + " 超出允许范围")
    return number


def _points(value: Any, minimum: int = 0, maximum: int = 10000) -> np.ndarray:
    try:
        array = np.asarray(value)
        if array.size == 0 and array.shape == (0,):
            array = np.empty((0, 3), dtype=np.float32)
        if (array.ndim != 2 or array.shape[1] != 3 or not minimum <= len(array) <= maximum or
                array.dtype.kind not in "fiu" or not np.isfinite(array).all() or
                np.any(np.abs(array.astype(np.float64)) > np.finfo(np.float32).max)):
            raise ValueError
        return array
    except (TypeError, ValueError, OverflowError):
        raise ValueError("点云必须是规定数量内的有限 XYZ 数值，且可由 float32 表示") from None


def _timestamp(value: Any) -> datetime:
    try:
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError
        stamp = datetime.fromtimestamp(value, timezone.utc)
        if stamp.year < 2020 or stamp > datetime.now(timezone.utc) + timedelta(minutes=5):
            raise ValueError
        return stamp
    except (ValueError, OverflowError, OSError):
        raise ValueError("测量时间缺失或超出后台允许范围") from None


def build_measurement(result: dict, *, measurement_id: str, node_id: str,
                      passage_id: str, calibration_id: str, algorithm_version: str,
                      configuration_hash: str, source_mode: str,
                      baseline_version_id: Optional[str] = None,
                      baseline_candidate_sha256: Optional[str] = None) -> dict:
    """将 service 的一次正式 result 转为不可变机器上报载荷，不读取旧 meta.empty。"""
    if not isinstance(result, dict):
        raise ValueError("测量结果必须为对象")
    # 与 radar_source.build_source 的显式分支对齐；未知值绝不猜为真采集。
    modes = {"live": "live", "udp": "live", "real": "live", "replay": "replay", "demo": "demo",
             "simulate": "demo", "sim": "demo", "sim-radar": "demo", "synthetic": "demo", "syn": "demo"}
    source_mode = modes.get(source_mode.lower()) if isinstance(source_mode, str) else None
    if source_mode not in ("live", "demo", "replay"):
        raise ValueError("数据源模式无效")
    if baseline_version_id is not None:
        _identifier(baseline_version_id, "baselineVersionId")
    if baseline_candidate_sha256 is not None:
        _hash(baseline_candidate_sha256, "baselineCandidateSha256")
    cloud = _points(result.get("cloud", []))
    count = result.get("n_points")
    if isinstance(count, bool) or not isinstance(count, int) or not len(cloud) <= count <= 2147483647:
        raise ValueError("原始点数不能小于预览点数或超出 int32")
    degraded = result.get("frame_degraded")
    if not isinstance(degraded, bool):
        raise ValueError("必须显式声明 frame_degraded")
    empty = result.get("empty")
    if empty is not None and not isinstance(empty, bool):
        raise ValueError("empty 必须是正式布尔判定或 null")
    volume = result.get("volume") or {}
    if not isinstance(volume, dict):
        raise ValueError("体积结果必须为对象")
    amount = _number(volume.get("volume_m3"), "volumeM3", 1000)
    hole = _number(volume.get("hole_ratio"), "holeRatio", 1)
    spread = _number(volume.get("bound_spread"), "boundSpread")
    filled = _number(volume.get("filled_ratio"), "filledRatio", 1)
    usable = (volume.get("ok") is True and amount is not None and baseline_version_id is not None and
              volume.get("status") not in ("failed", "unavailable"))
    # 1-5 步是无足够证据；6-10 步保留原检测的非空车语义，不添加双平面诊断门槛。
    diagnostics = result.get("diagnostics") or {}
    if not isinstance(diagnostics, dict):
        raise ValueError("诊断结果必须为对象")
    step = result.get("step", diagnostics.get("step"))
    early_failure = (step in (1, 2, 3, 4, 5) or result.get("reason") in
                     (R_INSUFFICIENT_POINTS, R_EMPTY_FAILED, R_NO_FLOOR,
                      R_FLOOR_NOT_LEVEL, R_TOO_FEW_CORRECTION))
    verdict = "unknown"
    if empty is True:
        verdict = "empty"
    elif empty is False and not early_failure and (step in (6, 7, 8, 9, 10) or usable):
        verdict = "loaded"
    status = "unavailable"
    if usable:
        # 已算出的载货体积直接交付；空洞、范围和残帧只保留原诊断值。
        # 旧归档中的低置信标记也不阻断按当前规则重新计算后的发布。
        status = "success" if verdict == "loaded" else "unavailable"
    else:
        amount = None
    interval = {}
    start = result.get("measurement_started_at")
    end = result.get("measurement_ended_at")
    if start is not None or end is not None:
        started, ended = _timestamp(start), _timestamp(end)
        if ended < started:
            raise ValueError("实际测量结束时间不能早于开始时间")
        interval = {
            "measurementStartedAt": started.isoformat().replace("+00:00", "Z"),
            "measurementEndedAt": ended.isoformat().replace("+00:00", "Z"),
        }
    return {
        "measurementId": _identifier(measurement_id, "measurementId"),
        "nodeId": _identifier(node_id, "nodeId"),
        "channelId": _identifier(result.get("channel"), "channelId"),
        "passageId": _identifier(passage_id, "passageId"),
        "calibrationId": _identifier(calibration_id, "calibrationId"),
        "sourceMode": source_mode,
        "algorithmVersion": _text(algorithm_version, "algorithmVersion", 64),
        "configurationHash": _hash(configuration_hash, "configurationHash"),
        "baselineCandidateSha256": baseline_candidate_sha256,
        "measuredAt": _timestamp(result.get("ts")).isoformat().replace("+00:00", "Z"),
        **interval,
        **({"automaticBaseline": True} if result.get("automatic_baseline") else {}),
        **({"baselineUse": result["baseline_use"]} if result.get("baseline_use") is not None else {}),
        "verdict": verdict, "frameDegraded": degraded,
        "volumeStatus": status, "volumeM3": amount,
        "baselineVersionId": baseline_version_id,
        "filledRatio": filled, "boundSpread": spread, "holeRatio": hole,
        "pointCount": count, "reason": _text(result.get("reason"), "reason", 2000),
        # ▲▲ **自动基线的闸门证据（`empty_evidence`）没有放进这个载荷**，
        #    2026-09-18 查实原因：`LoadRadarBridge._same_measurement()` 是
        #    **遍历 payload 的每一个键**要求后台回执逐一相等的
        #    （`load_radar_bridge.py` 里 `all(actual.get(k) == v for k, v in payload.items())`）。
        #    后台的 `LoadRadarMeasurement` 是强类型记录，不认识的字段**不会回显** ——
        #    于是 `actual.get("baselineCandidate")` 是 None 而这边是 dict，
        #    比对必然失败 -> `mark_blocked("后台回执标识不匹配")` ->
        #    **每一次测量上报都被阻塞**。加一个字段就能打挂整条上报链。
        #    => 要往这里加字段，**后台必须先加上对应属性并回显**，两边同时上线。
        #    在那之前证据走源端这三条路：看板 SSE、落盘 `.xyz.meta.json`、成套归档包。
        #    见 `detect.empty_evidence` 与 `docs/34`。
        "points": cloud.astype(np.float32).tolist(),
    }


_GATE_KEYS = ("name", "passed", "value", "limit", "note")


def baseline_candidate_payload(evidence: Any) -> Optional[Dict[str, Any]]:
    """把闸门证据整理成可上报的纯 JSON；算不出来时明确为 null，不编。

    ▲▲ **现在还没有接进 `build_measurement()`** —— 加一个字段就会让
       `_same_measurement()` 的回执比对失败、把整条上报链打挂（原因见
       `build_measurement` 里 `points` 上方那段说明）。
       等后台的 `LoadRadarMeasurement` 加上对应属性并回显之后，
       在那边加一行 `"baselineCandidate": baseline_candidate_payload(...)`
       即可，**两边必须同时上线**。
       在那之前证据走源端三条路：看板 SSE、落盘 meta、成套归档包。

    ▲ 逐条保留**实测值与门槛**，不只报过/不过 —— 后台要能看出"差多少"，
      也要能在攒够空车真值之后回头复核这些常数（现在只有 2 份正样本）。
    """
    if not isinstance(evidence, dict):
        return None
    gates = evidence.get("gates")
    if not isinstance(gates, list):
        return None
    rows = []
    for gate in gates[:32]:
        if not isinstance(gate, dict):
            continue
        rows.append({key: _finite_json(gate.get(key)) for key in _GATE_KEYS})
    profile = evidence.get("profile")
    return {"ok": evidence.get("ok") is True,
            "reason": _text(evidence.get("reason") or "", "baselineCandidateReason", 500),
            "gates": rows,
            "carriage": _finite_json(profile) if isinstance(profile, dict) else None}


def _finite_json(value: Any) -> Any:
    """非有限量显式记为 null；不把 NaN 混进 JSON。"""
    if isinstance(value, dict):
        return {str(key): _finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite_json(item) for item in value]
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(float(value)) else None
    if isinstance(value, (int, np.integer)):
        return int(value)
    if isinstance(value, str):
        return value[:500]
    return None if value is not None else None


def _configuration_value(value: Any) -> Any:
    if is_dataclass(value) and not isinstance(value, type):
        return {item.name: _configuration_value(getattr(value, item.name)) for item in fields(value)
                if item.name not in ("path", "raw", "angle_cache", "abs_angle_cache") and
                not item.name.endswith(("_path", "_dir"))}
    if isinstance(value, dict):
        return {str(key): _configuration_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_configuration_value(item) for item in value]
    if isinstance(value, np.ndarray):
        if value.dtype.kind not in "fiu" or not np.isfinite(value).all():
            raise ValueError("有效配置数组必须为有限数值")
        return value.tolist()
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, (float, np.number)) and math.isfinite(float(value)):
        return float(value)
    raise ValueError("有效配置含不支持或非有限字段")


def effective_configuration_bytes(calib: CalibCfg, dev: DevicesCfg, ch: ChannelCfg,
                                  angle_cache: Optional[AngleCache] = None) -> bytes:
    """冻结原有效配置的规范字节，不含Bridge/Plate凭据或原配置路径。"""
    if not isinstance(calib, CalibCfg) or not isinstance(dev, DevicesCfg) or not isinstance(ch, ChannelCfg):
        raise ValueError("必须传入实际生效的配置对象")
    if not isinstance(angle_cache, AngleCache):
        raise ValueError("必须显式传入实际使用的 AngleCache，不能省略角度标定")
    payload = {"schema": "jt128-effective-configuration-v1",
               "calib": _configuration_value(calib), "devices": _configuration_value(dev),
               "channel": _configuration_value(ch),
               "angle": {"elevation": _configuration_value(angle_cache.elevation),
                         "azimuth": _configuration_value(angle_cache.azimuth),
                         "loaded": angle_cache.loaded, "channels_seen": angle_cache.channels_seen}}
    return json.dumps(payload, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"), allow_nan=False).encode("utf-8")


def effective_configuration_hash(calib: CalibCfg, dev: DevicesCfg, ch: ChannelCfg,
                                 angle_cache: Optional[AngleCache] = None) -> str:
    """与历史合同完全相同的规范字节和摘要。"""
    return hashlib.sha256(effective_configuration_bytes(calib, dev, ch, angle_cache)).hexdigest()


def _iso_timestamp(value: Any, name: str) -> datetime:
    _text(value, name, 64)
    try:
        result = datetime.fromisoformat(value.replace("Z", "+00:00"))
        if result.tzinfo is None or result.year < 2020:
            raise ValueError
        return result.astimezone(timezone.utc)
    except ValueError:
        raise ValueError(name + " 必须为带时区的有效时间") from None


def _iso_timestamp_parts(value: Any, name: str) -> tuple[datetime, bool]:
    """保留被 Python 截断的非零精度，用于和微秒主帧精确比较。"""
    _text(value, name, 64)
    match = re.fullmatch(
        r"[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}"
        r"(?:\.([0-9]+))?(?:Z|[+-][0-9]{2}:[0-9]{2})", value)
    if match is None:
        raise ValueError(name + " 必须为带时区 ISO 时间")
    return _iso_timestamp(value, name), any(digit != "0" for digit in (match.group(1) or "")[6:])


def validate_baseline_as_of(measured_at: Any, baseline_as_of: Any) -> datetime:
    """校验历史选择回执与冻结主帧；UTC 转换与正式 measuredAt 完全一致。"""
    expected = _timestamp(measured_at)
    actual, remainder = _iso_timestamp_parts(baseline_as_of, "baselineAsOf")
    if remainder or actual != expected:
        raise ValueError("基线历史选择时刻与冻结主帧不一致")
    return expected


def _version(version: dict) -> dict:
    if not isinstance(version, dict):
        raise ValueError("基线版本元数据必须为对象")
    for key in ("versionId", "nodeId", "channelId", "calibrationId", "measurementId"):
        _identifier(version.get(key), key)
    for key in ("scopeKey", "assetSha256"):
        _hash(version.get(key), key)
    for key, limit in (("vehicleIdentity", 128), ("licensePlate", 32), ("actor", 128), ("reason", 500)):
        _text(version.get(key), key, limit)
    if version["licensePlate"].upper() == "UNKNOWN":
        raise ValueError("基线不能使用未知车牌")
    created = _iso_timestamp(version.get("createdAt"), "createdAt")
    if not isinstance(version.get("active"), bool):
        raise ValueError("基线必须显式声明 active")
    activated = version.get("activatedAt")
    if activated is not None:
        if _iso_timestamp(activated, "activatedAt") < created:
            raise ValueError("基线激活不能早于创建")
    elif version["active"]:
        raise ValueError("激活基线缺少激活时间")
    return dict(version)


def _read_baseline(data: bytes) -> EmptyBaseline:
    """在 NumPy 分配数组前验证 ZIP/NPY 长度、类型与维度，不允许 pickle。"""
    if not isinstance(data, bytes) or not 0 < len(data) <= MAX_ASSET_BYTES:
        raise ValueError("基线压缩文件大小无效")
    arrays: Dict[str, np.ndarray] = {}
    try:
        with zipfile.ZipFile(io.BytesIO(data)) as archive:
            entries = archive.infolist()
            names = [entry.filename for entry in entries]
            if (len(names) not in (4, 5, 6) or len(set(names)) != len(names) or
                    not _REQUIRED_ARRAYS <= set(names) or
                    set(names) - _REQUIRED_ARRAYS - {"bed_plane.npy", "source_channel.npy"} or
                    sum(entry.file_size for entry in entries) > MAX_EXPANDED_BYTES):
                raise ValueError
            for entry in entries:
                if (entry.flag_bits & 1 or entry.compress_type not in (zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED) or
                        not 10 <= entry.file_size <= MAX_EXPANDED_BYTES):
                    raise ValueError
                blob = archive.read(entry)
                stream = io.BytesIO(blob)
                version = np.lib.format.read_magic(stream)
                if version not in ((1, 0), (2, 0)):
                    raise ValueError
                length_size = 2 if version == (1, 0) else 4
                header_size = int.from_bytes(stream.read(length_size), "little")
                if not 0 < header_size <= 16 * 1024:
                    raise ValueError
                stream.seek(8)
                read_header = (np.lib.format.read_array_header_1_0 if version == (1, 0)
                               else np.lib.format.read_array_header_2_0)
                # NumPy 1.22 没有 max_header_size 参数；上面已在读取头部前显式限长。
                shape, _, dtype = read_header(stream)
                name = entry.filename[:-4]
                good_shape = ((name in ("floor_z", "created", "source_channel") and shape == ()) or
                              (name == "box" and shape == (6,)) or
                              (name == "bed_plane" and shape == (4,)) or
                              (name == "cloud" and len(shape) == 2 and shape[1] == 3 and
                               50 <= shape[0] <= MAX_BASELINE_POINTS))
                if not good_shape or dtype.hasobject or dtype.fields is not None:
                    raise ValueError
                if name in ("created", "source_channel"):
                    if dtype.kind not in "SU" or not 0 < dtype.itemsize <= (16384 if dtype.kind == "U" else 4096):
                        raise ValueError
                elif dtype.kind != "f" or dtype.itemsize not in (4, 8):
                    raise ValueError
                if stream.tell() + math.prod(shape) * dtype.itemsize != len(blob):
                    raise ValueError
                value = np.load(io.BytesIO(blob), allow_pickle=False)
                if name not in ("created", "source_channel") and not np.isfinite(value).all():
                    raise ValueError
                arrays[name] = value
        box = arrays["box"]
        if any(not math.isfinite(float(box[i + 1] - box[i])) or box[i + 1] <= box[i] for i in (0, 2, 4)):
            raise ValueError
        plane = arrays.get("bed_plane")
        source = arrays.get("source_channel")
        source = (source.item().decode("utf-8") if source.dtype.kind == "S" else str(source)) if source is not None else ""
        if source and source != "CH4":
            raise ValueError("基线来源必须为CH4")
        if plane is not None and abs(float(plane[2])) < 1e-9:
            raise ValueError
        created = arrays["created"].item()
        if isinstance(created, bytes):
            created = created.decode("utf-8")
        _text(created, "created", 4096)
        _points(arrays["cloud"], 50, MAX_BASELINE_POINTS)
        return EmptyBaseline(float(arrays["floor_z"]), tuple(float(v) for v in box),
                             np.asarray(arrays["cloud"], dtype=np.float32), len(arrays["cloud"]), created,
                             bed_plane=None if plane is None else tuple(float(v) for v in plane),
                             source_channel=source)
    except (ValueError, TypeError, KeyError, EOFError, OverflowError, OSError, UnicodeError,
            zipfile.BadZipFile, NotImplementedError):
        raise ValueError("空车基线 NPZ 数组、元数据或压缩内容无效") from None


def make_candidate(frame: Frame, res: EmptyResult, confirmed_empty: bool) -> bytes:
    """仅供当前人工确认操作生成候选；不写旧目录，不把 res.ok 解释为空车结论。"""
    if confirmed_empty is not True:
        raise ValueError("生成基线候选前必须显式人工确认空车")
    if not isinstance(frame, Frame) or not isinstance(res, EmptyResult):
        raise ValueError("缺少对应帧或六面拟合结果")
    _points(frame.xyz, 50, MAX_BASELINE_POINTS)
    _points(res.cloud, 50, MAX_BASELINE_POINTS)
    floor = res.plane("floor")
    if not floor.valid or not all(math.isfinite(v) for v in floor.as_tuple()) or abs(floor.c) < 1e-9:
        raise ValueError("缺少有效底面，不能生成基线候选")
    baseline = EmptyBaseline.from_empty_result(res, _timestamp(frame.ts).isoformat())
    if baseline is None:
        raise ValueError("当前结果不能构造空车基线")
    extra = {} if baseline.bed_plane is None else {"bed_plane": np.asarray(baseline.bed_plane, np.float64)}
    output = io.BytesIO()
    np.savez_compressed(output, floor_z=float(baseline.floor_z), box=np.asarray(baseline.box, np.float64),
                        cloud=baseline.cloud, created=baseline.created, **extra)
    data = output.getvalue()
    _read_baseline(data)
    return data


class BaselineCache:
    """后台资产的独立内容寻址缓存；选择与安装都不会自动修改测量线程基线。"""

    def __init__(self, storage_dir: str):
        self.storage_dir = Path(os.path.abspath(os.fspath(storage_dir)))
        for protected in (Path(PROJECT_ROOT).joinpath(*BASELINE_SUBDIR),
                          Path(PROJECT_ROOT) / "data" / "in",
                          Path(PROJECT_ROOT) / "re"):
            if self.storage_dir == protected or protected in self.storage_dir.parents:
                raise ValueError("后台基线缓存不能写入旧基线或只读材料目录")
        self._check_path(self.storage_dir)

    @staticmethod
    def _check_path(path: Path) -> None:
        for part in (path, *path.parents):
            try:
                info = part.lstat()
            except FileNotFoundError:
                continue
            if stat.S_ISLNK(info.st_mode) or getattr(info, "st_file_attributes", 0) & 0x400:
                raise ValueError("基线缓存路径不能经过符号链接或目录联接")

    @staticmethod
    def select(versions: Iterable[dict], binding: Optional[dict], node_id: str,
               channel_id: str, calibration_id: str) -> Optional[dict]:
        """只按后台明确身份选择唯一激活版本；无绑定返回 None，身份冲突直接报错。"""
        for name, value in (("nodeId", node_id), ("channelId", channel_id), ("calibrationId", calibration_id)):
            _identifier(value, name)
        if binding is None:
            return None
        if not isinstance(binding, dict):
            raise ValueError("车辆绑定必须为后台对象")
        _text(binding.get("vehicleIdentity"), "vehicleIdentity", 128)
        _text(binding.get("licensePlate"), "licensePlate", 32)
        if binding["licensePlate"].upper() == "UNKNOWN":
            return None
        matches = []
        for value in versions:
            version = _version(value)
            if (version["active"] and version["nodeId"] == node_id and version["channelId"] == "CH4" and
                    version["licensePlate"] == binding["licensePlate"]):
                matches.append(version)
        if len(matches) > 1:
            raise ValueError("同一车辆作用域出现多个激活基线，禁止猜测")
        return matches[0] if matches else None

    @staticmethod
    def select_historical(versions: Iterable[dict], binding: Optional[dict], node_id: str,
                          channel_id: str, calibration_id: str, *, measured_at: Any,
                          baseline_as_of: Any) -> Optional[dict]:
        """只接受后台已证明的原时刻版本；active 保留当前事实，不冒充历史证据。"""
        expected = validate_baseline_as_of(measured_at, baseline_as_of)
        for name, value in (("nodeId", node_id), ("channelId", channel_id), ("calibrationId", calibration_id)):
            _identifier(value, name)
        if binding is None:
            return None
        if not isinstance(binding, dict):
            raise ValueError("车辆绑定必须为后台对象")
        _text(binding.get("vehicleIdentity"), "vehicleIdentity", 128)
        _text(binding.get("licensePlate"), "licensePlate", 32)
        if binding["licensePlate"].upper() == "UNKNOWN":
            return None
        matches = []
        for value in versions:
            version = _version(value)
            if (version["nodeId"] != node_id or version["channelId"] != "CH4" or
                    version["licensePlate"] != binding["licensePlate"]):
                continue
            # 完整激活区间由 WebApi 审计历史证明；首次激活时间只做额外反证检查。
            activated, remainder = _iso_timestamp_parts(version.get("activatedAt"), "activatedAt")
            if activated > expected or activated == expected and remainder:
                raise ValueError("基线首次激活晚于冻结主帧")
            matches.append(version)
        if len(matches) > 1:
            raise ValueError("同一车辆原时刻存在多个基线，禁止猜测")
        return matches[0] if matches else None

    def install(self, version: dict, data: bytes) -> EmptyBaseline:
        version = _version(version)
        if (not isinstance(data, bytes) or not 0 < len(data) <= MAX_ASSET_BYTES or
                hashlib.sha256(data).hexdigest() != version["assetSha256"]):
            raise ValueError("基线资产 SHA-256 不匹配")
        baseline = _read_baseline(data)
        self._check_path(self.storage_dir)
        self.storage_dir.mkdir(parents=True, exist_ok=True)
        destination = self.storage_dir / (version["assetSha256"] + ".npz")
        self._check_path(destination)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(dir=str(self.storage_dir), prefix=".baseline-", delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
            self._check_path(destination)
            os.replace(str(temporary), str(destination))
            temporary = None
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
        return baseline
