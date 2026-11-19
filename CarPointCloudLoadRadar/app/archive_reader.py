"""有界只读加载归档，恢复真实输入行；不补造历史配置、基线或停稳时序。"""
from __future__ import annotations

import ast
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import struct
from typing import Optional, Tuple

import numpy as np

from .cloud_archive import FrozenBundle
from .cloud_observation import FrameSnapshot, ObservationPair, ObservationWindow
from .pointcloud import Frame, XYZ_DTYPE
from .sample_paths import Sample, ROLES
from .archive_io import ArchiveReadLimits, _read_regular, bounded_json, check_budget, runtime_members


INPUT_REPLAY_NOTE = ("按原成员与行索引恢复主 ROI 输入，使用当前配置诊断；"
                     "不代表原参数/原体积结果或连续停稳过程复现")




def _integer(value, minimum=0):
    if type(value) is not int or value < minimum:
        raise ValueError("归档整数元数据无效")
    return value


def _number(value, minimum=0., positive=False):
    if (type(value) not in (int, float) or not math.isfinite(value)
            or value < minimum or positive and value <= minimum):
        raise ValueError("归档时间元数据无效")


def _identity(value, packets=True):
    _integer(value["seq"], 1)
    _number(value["ts"], positive=True)
    _integer(value["gen"])
    if packets:
        _integer(value["packets"])




def _npy(data: bytes, lineage: bool, limit: int) -> np.ndarray:
    """先验证有限 header、dtype、shape 与精确长度；不让 np.load 按恶意声明分配。"""
    if data[:6] != b"\x93NUMPY" or data[6:8] not in (b"\x01\x00", b"\x02\x00"):
        raise ValueError("归档索引不是支持的 NPY")
    version1 = data[6:8] == b"\x01\x00"
    start = 10 if version1 else 12
    if len(data) < start:
        raise ValueError("归档索引头截断")
    length = int.from_bytes(data[8:start], "little")
    if length > 4096 or start + length > len(data):
        raise ValueError("归档索引头过大或截断")
    header = ast.literal_eval(data[start:start + length].decode("ascii").strip())
    if not isinstance(header, dict) or set(header) != {"descr", "fortran_order", "shape"}:
        raise ValueError("归档索引头字段无效")
    descriptor, shape = header["descr"], header["shape"]
    if (not isinstance(descriptor, str) or not re.fullmatch(r"[<>=|][iu][1248]", descriptor)
            or header["fortran_order"] is not False or type(shape) is not tuple
            or len(shape) != (2 if lineage else 1)
            or any(type(n) is not int or n < 0 for n in shape)
            or lineage and shape[1] != 2):
        raise ValueError("归档索引必须为有限形状的整数行映射")
    count = math.prod(shape)
    dtype = np.dtype(descriptor)
    if count > limit or start + length + count * dtype.itemsize != len(data):
        raise ValueError("归档索引大小超预算、截断或存在尾随字节")
    return np.frombuffer(data, dtype=dtype, offset=start + length).reshape(shape)


def _snapshot(data: bytes, record: dict, identity: dict) -> FrameSnapshot:
    values = np.frombuffer(data, dtype=XYZ_DTYPE, offset=4)
    xyz = np.column_stack([values[key] for key in ("x", "y", "z")])
    return FrameSnapshot.freeze(Frame(xyz, values["ring"] if record["ring_present"] else None,
                                      identity["seq"], identity["ts"], identity["gen"],
                                      identity.get("packets", 0)))


def _preflight(meta: dict, limits: ArchiveReadLimits):
    entries = meta["files"]
    if not isinstance(entries, dict) or len(entries) > limits.members:
        raise ValueError("归档成员数超过预算")
    _identity(meta["frame"])
    _identity(meta["detection_frame"], packets=False)
    _number(meta["acquired_at"], positive=True)
    if type(meta["empty_observation"]) is not bool or not isinstance(meta["roi"], dict):
        raise ValueError("归档状态或 ROI 元数据无效")
    expected = {role + ".xyz": role for role in ROLES}
    expected.update({"vehicle.indices.npy": "vehicle_indices", "carriage.indices.npy": "carriage_indices",
                     "detection.xyz.meta.json": "result", "detection.xyz.reason.txt": "reason"})
    for key, prefix, role in (("provenance", "source", "measurement"),
                              ("volume_support", "volume-source", "volume_surface")):
        if key not in meta:
            continue
        segments = meta[key]["segments"]
        if not isinstance(segments, list) or not 0 < len(segments) <= limits.segments:
            raise ValueError("归档来源窗口大小无效或超过预算")
        _integer(meta[key]["input_points"])
        for index, segment in enumerate(segments):
            _identity(segment["frame"])
            _integer(segment["member"])
            _integer(segment["epoch"])
            _number(segment["received_monotonic"])
            for field in ("raw_rows", "input_rows"):
                if type(segment[field]) is not list or len(segment[field]) != 2:
                    raise ValueError("归档来源行区间无效")
                for value in segment[field]:
                    _integer(value)
            for field in ("complete", "raw_ring_present", "input_ring_present"):
                if type(segment[field]) is not bool:
                    raise ValueError("归档来源状态必须为布尔值")
            expected["%s-%03d.xyz" % (prefix, index)] = role + "_source"
            expected["%s-%03d.input-indices.npy" % (prefix, index)] = role + "_input_indices"
        if key == "provenance":
            expected.update({"detection.input-indices.npy": "detection_input_indices",
                             "detection.lineage.npy": "detection_lineage"})
    raw_names = sorted(name for name in entries if re.fullmatch(r"raw-[0-9]{3}\.xyz", name))
    if raw_names != ["raw-%03d.xyz" % i for i in range(len(raw_names))]:
        raise ValueError("额外诊断成员编号不连续")
    expected.update({name: "diagnostic_observation" for name in raw_names})
    expected.update(runtime_members(meta))
    if set(expected) != set(entries):
        raise ValueError("归档角色缺失或存在未声明来源成员")
    total, points = 0, 0
    for name, record in entries.items():
        size = _integer(record["bytes"])
        if size > limits.file_bytes or record["role"] != expected[name]:
            raise ValueError("归档成员角色不符或大小超过预算")
        if not isinstance(record["sha256"], str) or not re.fullmatch("[a-f0-9]{64}", record["sha256"]):
            raise ValueError("归档成员摘要无效")
        total += size
        if name.endswith(".xyz"):
            count = _integer(record["points"])
            points += count
            if (type(record["ring_present"]) is not bool or record["origin"] != "observed"
                    or record["coordinates"] != ("detection" if name == "detection.xyz" else "source")
                    or size != 4 + count * XYZ_DTYPE.itemsize):
                raise ValueError("归档点云的 ring、坐标、来源或大小无效")
            if "frame" in record:
                _identity(record["frame"])
        # 派生预算为保守估算，不声称等于进程实际 RSS 上限。
        if total > limits.total_bytes or points > limits.points or total * 2 + points * 128 > limits.working_bytes:
            raise ValueError("归档总字节、点数或派生工作内存超过预算")


def _semantic_checks(bundle: FrozenBundle, meta: dict, limits: ArchiveReadLimits):
    files, arrays = dict(bundle.files), {}
    result = bounded_json(files["detection.xyz.meta.json"], limits)
    if "empty" in result and type(result["empty"]) is not bool:
        raise ValueError("归档判定状态必须为布尔值")
    for key in ("channel", "measurement_id", "passage_id"):
        if key in result and result[key] != meta[key]:
            raise ValueError("归档结果身份与清单不一致")
    for key in ("seq", "ts", "gen"):
        if key in result and (type(result[key]) not in (int, float) or result[key] != meta["frame"][key]):
            raise ValueError("归档结果帧身份与清单不一致")
    if "source_generation" in result and (type(result["source_generation"]) is not int
            or result["source_generation"] != meta["frame"]["gen"]):
        raise ValueError("归档结果数据源代次与清单不一致")
    if ("reason" in result and (not isinstance(result["reason"], str)
            or result["reason"].encode("utf-8") != files["detection.xyz.reason.txt"])):
        raise ValueError("归档理由与结果不一致")
    files["detection.xyz.reason.txt"].decode("utf-8")
    index_values = 0
    for name, data in files.items():
        record = meta["files"][name]
        if name.endswith(".npy"):
            arrays[name] = _npy(data, name == "detection.lineage.npy", limits.index_values - index_values)
            index_values += arrays[name].size
        elif name.endswith(".xyz"):
            if struct.unpack("<I", data[:4])[0] != record["points"]:
                raise ValueError("归档点数头与清单不一致")
            value = np.frombuffer(data, dtype=XYZ_DTYPE, offset=4)
            if (any(not np.isfinite(value[key]).all() for key in ("x", "y", "z"))
                    or not record["ring_present"] and np.any(value["ring"] != 0)):
                raise ValueError("归档点云非有限或缺失 ring 占位无效")
    raw = np.frombuffer(files["observation.xyz"], dtype=XYZ_DTYPE, offset=4)
    for role in ("vehicle", "carriage"):
        selected = arrays[role + ".indices.npy"]
        if len(selected) and (selected.min() < 0 or selected.max() >= len(raw)
                              or np.any(selected[1:] <= selected[:-1])):
            raise ValueError("车辆或车厢原行索引越界、重复或乱序")
        actual = np.frombuffer(files[role + ".xyz"], dtype=XYZ_DTYPE, offset=4)
        if (not np.array_equal(raw[selected], actual)
                or meta["files"][role + ".xyz"]["ring_present"] is not meta["files"]["observation.xyz"]["ring_present"]):
            raise ValueError("车辆或车厢点云不是声明的原观测子集")
    if not np.isin(arrays["carriage.indices.npy"], arrays["vehicle.indices.npy"]).all():
        raise ValueError("车厢原行不属于车辆原行")
    # 所有 NPY 在进入既有 np.load/追溯检查之前已经通过有界长度核验。
    bundle.validate()
    for key, prefix in (("provenance", "source"), ("volume_support", "volume-source")):
        if key in meta:
            last = "%s-%03d.xyz" % (prefix, len(meta[key]["segments"]) - 1)
            if meta["files"][last]["ring_present"] is not meta["files"]["observation.xyz"]["ring_present"]:
                raise ValueError("末帧原观测的 ring 缺失状态不一致")
    if "provenance" in meta and "volume_support" in meta:
        for index, support in enumerate(meta["volume_support"]["primary_members"]):
            if (meta["files"]["source-%03d.xyz" % index]["ring_present"] is not
                    meta["files"]["volume-source-%03d.xyz" % support]["ring_present"]):
                raise ValueError("重复用途成员的 ring 缺失状态不一致")


@dataclass(frozen=True)
class VerifiedArchive:
    """只持有已校验不可变字节；每次访问元数据或 Frame 返回独立对象。"""
    bundle: FrozenBundle
    manifest_sha256: str

    @property
    def metadata(self) -> dict:
        return self.bundle.metadata

    @property
    def capabilities(self) -> dict:
        meta = self.metadata
        runtime = "measurement_runtime" in meta
        original_replay = False
        if runtime:
            from .measurement_runtime import algorithm_identity
            call = json.loads(dict(self.bundle.files)["volume-call.json"])
            local = algorithm_identity()
            original_replay = (call["baseline"]["state"] != "asset_unavailable" and
                call["algorithm"]["contract"] == local["contract"])
        return dict(primary_input="provenance" in meta, volume_extra_inputs="volume_support" in meta,
                    original_parameters=runtime, original_loaded_cloud=runtime and call["loaded"] is not None,
                    original_volume_replay=original_replay, gate_summary=runtime, standstill_timeline=False,
                    limitation="停稳记录仅为许可摘要，不是完整事件流；旧包或缺原基线资产不可原参数复算")

    def role(self, role: str) -> FrameSnapshot:
        if role not in ROLES:
            raise ValueError("未知归档角色")
        meta = self.metadata
        identity = meta["detection_frame" if role == "detection" else "frame"]
        name = role + ".xyz"
        return _snapshot(dict(self.bundle.files)[name], meta["files"][name], identity)

    def window(self, volume: bool = False) -> ObservationWindow:
        meta, files = self.metadata, dict(self.bundle.files)
        key, prefix = ("volume_support", "volume-source") if volume else ("provenance", "source")
        if key not in meta:
            raise ValueError("归档缺少 %s，不能精确恢复该窗口" % key)
        pairs = []
        for segment in meta[key]["segments"]:
            name = "%s-%03d" % (prefix, segment["member"])
            raw = _snapshot(files[name + ".xyz"], meta["files"][name + ".xyz"], segment["frame"])
            selected = _npy(files[name + ".input-indices.npy"], False, len(files[name + ".input-indices.npy"]))
            cropped = FrameSnapshot.freeze(raw.frame()._mask(selected))
            pairs.append(ObservationPair(segment["channel"], segment["epoch"], segment["received_monotonic"],
                                         segment["complete"], raw, cropped, selected.astype("<i8").tobytes()))
        return ObservationWindow(tuple(pairs))

    def primary_input(self) -> FrameSnapshot:
        return FrameSnapshot.freeze(self.window().input_frame())

    def volume_extra_inputs(self) -> Tuple[FrameSnapshot, ...]:
        window = self.window(volume=True)
        return tuple(window.pairs[i].cropped for i in self.metadata["volume_support"]["extra_members"])


def load_archive(sample: Sample, *, limits: Optional[ArchiveReadLimits] = None) -> VerifiedArchive:
    """从发现摘要绑定的包读取；损坏、不支持或资源不足一律报错，不回退裸 XYZ。"""
    limits = limits or ArchiveReadLimits()
    if (not isinstance(sample, Sample) or not sample.archived or sample.role not in ROLES
            or not re.fullmatch("[a-f0-9]{64}", sample.manifest_sha256)):
        raise ValueError("归档读取需要发现阶段的角色、身份及清单摘要")
    path = Path(os.path.abspath(sample.path))
    if path.name != sample.role + ".xyz":
        raise ValueError("归档角色路径与发现记录不符")
    manifest_path = path.parent / "manifest.json"
    manifest = _read_regular(manifest_path, limits.manifest_bytes)
    digest = hashlib.sha256(manifest).hexdigest()
    if digest != sample.manifest_sha256:
        raise ValueError("归档清单在发现后被替换")
    try:
        meta = bounded_json(manifest, limits, manifest=True)
        if (meta["channel"] != sample.channel or meta["measurement_id"] != sample.measurement_id
                or meta["acquired_at"] != sample.acquired_at):
            raise ValueError("归档身份与发现记录不一致")
        check_budget(meta, len(manifest), limits)
        _preflight(meta, limits)
        files = []
        for name, record in sorted(meta["files"].items()):
            data = _read_regular(path.parent / name, limits.file_bytes, record["bytes"])
            if hashlib.sha256(data).hexdigest() != record["sha256"]:
                raise ValueError("归档成员字节与清单摘要不一致")
            files.append((name, data))
        if _read_regular(manifest_path, limits.manifest_bytes) != manifest:
            raise ValueError("归档清单在读取期间发生变化")
        bundle = FrozenBundle(manifest, tuple(files))
        _semantic_checks(bundle, meta, limits)
        archive = VerifiedArchive(bundle, digest)
        if runtime_members(meta):
            from .measurement_runtime import restore_enter_frames, restore_exit_frames, restore_inputs
            call = restore_inputs(archive, limits)[0]
            # 进/离场帧逐帧摘要、点数、时序在读入时就核，不等到复算才发现被换过。
            restore_enter_frames(archive, call)
            restore_exit_frames(archive, call)
        return archive
    except (KeyError, TypeError, IndexError, OverflowError, SyntaxError, UnicodeError, RecursionError) as exc:
        raise ValueError("归档结构或索引无效: " + type(exc).__name__) from exc
