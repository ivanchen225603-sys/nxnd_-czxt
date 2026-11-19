"""进场离线输入的身份、坐标配置和内容校验；历史缺证据必须显式确认。"""
from __future__ import annotations

import hashlib
import json
import math
import re
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .pointcloud import XYZ_DTYPE, XYZ_HEADER, XYZ_STRIDE
from .plate import safe_dir_name


MOTION_SCHEMA = "jt128-motion-frame-v2"
HISTORY_SCHEMA = "enter-history-confirmation-v1"


def file_sha256(path) -> str:
    """按块读取，写完的点云才计算摘要，避免再占一份整帧内存。"""
    value = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def coordinate_evidence(configuration: bytes) -> dict:
    """与正式归档相同的配置字节；角度表身份含实际表值，不依赖文件名。"""
    cfg = json.loads(configuration)
    angle = json.dumps(cfg["angle"], ensure_ascii=False, sort_keys=True,
                       separators=(",", ":"), allow_nan=False).encode("utf-8")
    return dict(configuration_sha256=hashlib.sha256(configuration).hexdigest(),
                coordinate_config=dict(processing=cfg["calib"]["processing"],
                    range_mode=cfg["channel"]["lidar"]["range_mode"],
                    z_down=cfg["channel"]["lidar"]["z_down"],
                    angle_calibration_sha256=hashlib.sha256(angle).hexdigest()))


@dataclass(frozen=True)
class ValidatedEnter:
    clouds: tuple
    historical: bool
    notes: tuple


def _require(ok, message):
    if not ok:
        raise ValueError(message)


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _same_numbers(a, b):
    try:
        aa, bb = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        return aa.shape == bb.shape and np.isfinite(aa).all() and np.allclose(aa, bb, atol=1e-8, rtol=0)
    except (TypeError, ValueError):
        return False


def validate_enter_frames(archive, paths, baseline_path, actual_plate,
                          history_confirmation=None) -> ValidatedEnter:
    """先核证据再交给几何配准。确认文件仅承认旧旁注缺项，不放过字段冲突。"""
    meta, assets = archive.metadata, dict(archive.bundle.files)
    cfg_bytes = assets["effective-configuration.json"]
    cfg = json.loads(cfg_bytes)
    gate = json.loads(assets["gate-summary.json"])
    result = json.loads(assets["detection.xyz.meta.json"])
    expected = coordinate_evidence(cfg_bytes)
    _require(expected["configuration_sha256"] == meta["configuration_hash"], "归档配置摘要不一致")
    paths = tuple(Path(p) for p in paths)
    _require(len(paths) >= 2, "进场帧不足两帧")
    notes = []
    confirmation = history_confirmation
    if confirmation is not None:
        _require(isinstance(confirmation, dict), "历史确认文件必须是对象")
        checks = dict(schema=HISTORY_SCHEMA, archive_manifest_sha256=archive.manifest_sha256,
                      channel=meta["channel"], passage_id=meta["passage_id"],
                      capture_instance_id=gate["capture_instance"], archive_plate=result.get("plate"),
                      baseline_sha256=file_sha256(baseline_path))
        for name, value in checks.items():
            _require(confirmation.get(name) == value, "历史确认与本次输入不符：" + name)
        _require(confirmation.get("actual_plate") == actual_plate, "历史确认的实际车牌不符")
        _require(isinstance(confirmation.get("reason"), str) and confirmation["reason"].strip(), "历史确认缺少对应依据")
        _require(isinstance(confirmation.get("enter_files_sha256"), dict), "历史确认缺少进场文件摘要清单")
    else:
        _require(result.get("plate_ok") is True and isinstance(actual_plate, str) and actual_plate
                 and safe_dir_name(result.get("plate", "")) == safe_dir_name(actual_plate),
                 "空车基线车牌与归档车牌不能确认；旧接口牌对应真实牌须提供 --history-confirmation")

    channel = cfg["channel"]["lidar"]
    proc = cfg["calib"]["processing"]
    effective_yaw = channel.get("yaw_offset_deg")
    if effective_yaw is None:
        effective_yaw = proc["yaw_offset_deg"]
    roi = np.asarray(cfg["calib"]["roi"], dtype=float)
    frames = meta.get("volume_support", meta["provenance"])["segments"]
    first_ts = min(s["frame"]["ts"] for s in frames)
    first_seq = min(s["frame"]["seq"] for s in frames)
    previous = None
    clouds, legacy = [], False
    indices = []
    for path in paths:
        _require(path.stat().st_size <= 128 * 1024 * 1024, "进场帧超过读取上限")
        sidecar = Path(str(path) + ".meta.json")
        _require(sidecar.stat().st_size <= 1024 * 1024, "进场旁注超过读取上限")
        item = json.loads(sidecar.read_text(encoding="utf-8"))
        _require(isinstance(item, dict), "进场旁注必须是对象")
        for name, value in dict(channel=meta["channel"], passage_id=meta["passage_id"],
                capture_instance_id=gate["capture_instance"], gen=meta["frame"]["gen"],
                range_mode=channel["range_mode"]).items():
            _require(item.get(name) == value, "进场旁注与归档不符：" + name)
        _require(item.get("source_mode") in ("live", "udp", "real"), "进场旁注与归档不符：source_mode")
        _require(type(item.get("gen")) is int and type(item.get("session_id")) is int
                 and item["session_id"] > 0, "进场源代次或会话编号非法")
        if item["session_id"] != result["session"]:
            note = "同一物理车次跨测量会话：进场 %s，测量 %s" % (item["session_id"], result["session"])
            if note not in notes:
                notes.append(note)
        _require(meta["source_mode"] == "live", "进场补点只接受实车来源归档")
        _require(item.get("complete_gate_passed") is True and type(item.get("dropped_packets")) is int
                 and item["dropped_packets"] == 0 and type(item.get("packets")) is int and item["packets"] > 0,
                 "进场帧缺少完整且无丢包的证据")
        _require(type(item.get("cropped_to_roi")) is bool, "进场旁注缺少裁剪状态")
        try:
            item_roi = np.asarray([item["roi"][axis] for axis in "xyz"], dtype=float).T
        except (KeyError, TypeError, ValueError):
            raise ValueError("进场旁注缺少合法裁剪框") from None
        _require(_same_numbers(item_roi, roi), "进场裁剪框与归档输入裁剪框不符")
        for name, value in dict(ground_z=cfg["calib"]["detection"]["ground_z"],
                yaw_offset_deg=proc["yaw_offset_deg"], effective_yaw_offset_deg=effective_yaw).items():
            _require(_finite(item.get(name)) and abs(item[name] - value) < 1e-8, "进场坐标配置不符：" + name)
        _require(item.get("channel_yaw_offset_deg") == channel.get("yaw_offset_deg"), "进场通道偏航配置不符")
        seq, ts = item.get("seq"), item.get("ts")
        _require(type(seq) is int and seq >= 0 and _finite(ts) and ts > 0, "进场帧身份非法")
        _require(seq < first_seq and ts < first_ts, "进场帧必须早于停稳支持窗口")
        _require(previous is None or (seq > previous[0] and ts > previous[1]), "进场帧序或时间重复、倒序")
        previous = (seq, ts)
        _require(type(item.get("index")) is int and type(item.get("frames")) is int
                 and item["frames"] == len(paths), "进场批次缺帧或总数不一致")
        indices.append(item["index"])
        match = re.search(r"_(\d+)of(\d+)\.xyz$", path.name)
        _require(match is not None and (int(match[1]), int(match[2])) == (item["index"], item["frames"]),
                 "进场文件名与旁注编号不符")

        data = path.read_bytes()
        _require(len(data) >= XYZ_HEADER, "进场点云头缺失")
        n = struct.unpack("<I", data[:XYZ_HEADER])[0]
        _require(type(item.get("points")) is int and n == item["points"]
                 and len(data) == XYZ_HEADER + n * XYZ_STRIDE, "进场实际点数与旁注或文件长度不符")
        digest = hashlib.sha256(data).hexdigest()
        if item.get("metadata_schema") == MOTION_SCHEMA:
            _require(item.get("motion_kind") == "enter", "该文件不是进场帧")
            for name, value in expected.items():
                _require(item.get(name) == value, "进场配置证据不符或缺失：" + name)
            _require(item.get("file_sha256") == digest, "进场点云摘要不符")
        else:
            _require("metadata_schema" not in item, "进场旁注版本不支持")
            _require(confirmation is not None, "历史旁注缺少配置/内容证据，须显式提供 --history-confirmation")
            for name, value in expected.items():
                _require(name not in item or item[name] == value, "历史旁注已有配置证据冲突：" + name)
            _require("file_sha256" not in item or item["file_sha256"] == digest, "历史旁注已有点云摘要冲突")
            _require(confirmation["enter_files_sha256"].get(path.name) == digest, "进场内容与历史确认清单不符")
            legacy = True
        raw = np.frombuffer(data, dtype=XYZ_DTYPE, offset=XYZ_HEADER)
        xyz = np.column_stack([raw[a] for a in "xyz"])
        _require(np.isfinite(xyz).all(), "进场点云有非有限坐标")
        inside = ((xyz >= roi[0] - 1e-5) & (xyz <= roi[1] + 1e-5)).all(axis=1)
        _require(not item["cropped_to_roi"] or bool(inside.all()), "已裁剪进场帧包含框外点")
        clouds.append(xyz if item["cropped_to_roi"] else xyz[inside])
    _require(indices == list(range(1, len(paths) + 1)), "进场批次编号缺失、重复或乱序")
    if legacy:
        notes.append("历史弱证据回放：身份、已记坐标配置及确认时文件摘要已核验；采集时完整配置指纹、"
                     "距离比例/偏置、角度标定身份、z_down及原始内容摘要未验证，不能作为正式数据验收")
    if confirmation is not None:
        notes.append("已按显式确认绑定旧接口牌 %s -> 实际牌 %s" % (result.get("plate"), actual_plate))
    return ValidatedEnter(tuple(clouds), legacy, tuple(notes))
