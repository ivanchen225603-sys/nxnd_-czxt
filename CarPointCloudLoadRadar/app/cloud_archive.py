"""不可变点云包与可恢复文件投影；不创建或激活正式空车基线。"""
from __future__ import annotations

import hashlib
import errno
import io
import json
import math
import os
import shutil
from pathlib import Path
import re
import sqlite3
import stat
import struct
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Any, Optional, Tuple

import numpy as np

from .pointcloud import Frame, XYZ_DTYPE
from .cloud_observation import ObservationWindow


ARCHIVE_DIR = "archives-v1"
_ROLES = ("observation", "vehicle", "carriage", "detection")


class ArchiveConflict(ValueError):
    """同一标识出现不同内容，保留现场，不覆盖也不换 ID 重试。"""


def _identifier(value: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,64}", value) or value in (".", ".."):
        raise ValueError("归档标识必须与接口的安全字符合同一致")
    return value


def _component(value: str) -> str:
    # 原标识留在 manifest；编码避免冒号、保留名、大小写折叠和过长路径。
    return "id-" + _sha(_identifier(value).encode("ascii"))[:32]


def _json(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"),
                      allow_nan=False).encode("utf-8")


def _sha(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def xyz_bytes(frame: Frame) -> bytes:
    """原厂二进制格式；ring 缺失状态另记在 manifest，不伪称零号回波。"""
    out = np.empty(frame.n, dtype=XYZ_DTYPE)
    for index, key in enumerate(("x", "y", "z")):
        out[key] = frame.xyz[:, index]
    out["ring"] = frame.ring if frame.ring is not None else 0
    return struct.pack("<I", frame.n) + out.tobytes()


def _indices_bytes(indices: np.ndarray) -> bytes:
    stream = io.BytesIO()
    np.save(stream, indices.astype("<i8", copy=False), allow_pickle=False)
    return stream.getvalue()


@dataclass(frozen=True)
class FrozenBundle:
    """只含不可变字节；原数组、结果字典或配置后续变化不影响重放。"""
    manifest: bytes
    files: Tuple[Tuple[str, bytes], ...]

    @property
    def metadata(self) -> dict:
        return json.loads(self.manifest.decode("utf-8"))

    @property
    def measurement_id(self) -> str:
        return self.metadata["measurement_id"]

    def validate(self) -> dict:
        meta = self.metadata
        if meta.get("schema") != "cloud-archive-v1":
            raise ValueError("不支持的点云归档版本")
        for key in ("measurement_id", "channel"):
            _identifier(meta[key])
        if meta["passage_id"] is not None:
            _identifier(meta["passage_id"])
        if not isinstance(meta["acquired_at"], (float, int)) or not math.isfinite(meta["acquired_at"]) or meta["acquired_at"] <= 0:
            raise ValueError("归档必须携带有限、正值采集时间")
        if _json(meta) != self.manifest:
            raise ValueError("manifest 必须为唯一规范 JSON")
        if (meta.get("authoritative") is not False or
                not re.fullmatch(r"[a-f0-9]{64}", meta.get("configuration_hash", "")) or
                not isinstance(meta.get("algorithm_version"), str) or not meta["algorithm_version"].strip()):
            raise ValueError("归档必须记录有效配置摘要、算法版本及非权威状态")
        if meta["source_mode"] not in ("live", "demo", "replay"):
            raise ValueError("归档必须使用规范的数据来源模式")
        if type(meta["frame"]["complete"]) is not bool or type(meta["frame"]["measurable"]) is not bool:
            raise ValueError("归档完整性与可测性必须为明确布尔值")
        if any(meta["frame"][key] != meta["detection_frame"][key] for key in ("seq", "ts", "gen")):
            raise ValueError("主观测与判定云的帧身份不一致")
        if meta["frame"]["ts"] != meta["acquired_at"]:
            raise ValueError("归档时间必须来自主观测帧")
        entries = meta["files"]
        if (not isinstance(self.manifest, bytes) or not isinstance(self.files, tuple)
                or tuple(sorted(self.files)) != self.files
                or len(entries) != len(self.files) or len(dict(self.files)) != len(self.files)):
            raise ValueError("归档文件清单重复或不完整")
        names = set()
        for name, data in self.files:
            if (not isinstance(data, bytes) or not re.fullmatch(r"[a-z0-9][a-z0-9_.-]{0,95}", name)
                    or name == "manifest.json" or name not in entries):
                raise ValueError("归档文件名或原字节无效")
            record = entries[name]
            if record["sha256"] != _sha(data) or record["bytes"] != len(data):
                raise ArchiveConflict("归档字节与 manifest 摘要不一致")
            if name.endswith(".xyz"):
                if len(data) < 4 or struct.unpack("<I", data[:4])[0] != record["points"] or len(data) != 4 + record["points"] * XYZ_DTYPE.itemsize:
                    raise ValueError("点云字节与点数元数据不一致")
            names.add(name)
        if not {role + ".xyz" for role in _ROLES}.issubset(names):
            raise ValueError("缺少主观测、车辆、车厢或判定云角色")
        if "provenance" in meta:
            _validate_provenance(meta, dict(self.files))
        if "volume_support" in meta:
            _validate_provenance(meta, dict(self.files), volume_support=True)
        if "provenance" in meta and "volume_support" in meta:
            primary, support = meta["provenance"]["segments"], meta["volume_support"]["segments"]
            if (primary[-1]["epoch"] != support[-1]["epoch"]
                    or primary[-1]["received_monotonic"] != support[-1]["received_monotonic"]):
                raise ValueError("检测与体积支持不属于同一活动观测窗口")
            if [support[index]["frame"] for index in meta["volume_support"]["primary_members"]] != [item["frame"] for item in primary]:
                raise ValueError("体积主输入成员与检测主输入不一致")
            source_members = {tuple(segment["frame"][key] for key in ("seq", "ts", "gen", "packets")): segment
                              for segment in primary}
            contents = dict(self.files)
            for segment in support:
                member = source_members.get(tuple(segment["frame"][key] for key in ("seq", "ts", "gen", "packets")))
                if member is not None:
                    if (member["received_monotonic"] != segment["received_monotonic"]
                            or any(contents["source-%03d.%s" % (member["member"], suffix)] !=
                                   contents["volume-source-%03d.%s" % (segment["member"], suffix)]
                                   for suffix in ("xyz", "input-indices.npy"))):
                        raise ValueError("检测与体积重复成员的原观测或输入行不一致")
        if meta["empty_observation"] and (not meta["frame"]["complete"] or not meta["frame"]["measurable"]
                or meta["passage_id"] is None or entries["carriage.xyz"]["points"] <= 0
                or meta["roi"].get("carriage_status") not in ("available", "partial")
                or json.loads(dict(self.files)["detection.xyz.meta.json"])["empty"] is not True):
            raise ValueError("空车观测引用缺少完整可测车次或独立车厢")
        from .archive_io import runtime_members, check_budget, ArchiveReadLimits
        if runtime_members(meta):
            check_budget(meta, len(self.manifest), ArchiveReadLimits())
            from .measurement_runtime import validate_runtime_records
            validate_runtime_records(meta, dict(self.files), ArchiveReadLimits())
        return meta


def freeze_bundle(*, measurement_id: str, channel: str, passage_id: Optional[str],
                  observation: Frame, detection: Frame, vehicle_indices: np.ndarray,
                  carriage_indices: np.ndarray, result: dict, roi: dict,
                  configuration_hash: str, algorithm_version: str,
                  complete: bool, measurable: bool, baseline_version_id=None,
                  source_mode: str = "live", transforms: Optional[dict] = None,
                  extra_frames: Tuple[Frame, ...] = (),
                  observation_window: Optional[ObservationWindow] = None,
                  detection_indices: Optional[np.ndarray] = None,
                  detection_rotation: Optional[np.ndarray] = None,
                  volume_window: Optional[ObservationWindow] = None,
                  volume_extra_members: Optional[Tuple[int, ...]] = None,
                  runtime_files: Tuple[Tuple[str, bytes], ...] = ()) -> FrozenBundle:
    """冻结实测点及其原索引。全观测包括环境，不表示隐藏表面已扫描完整。"""
    for value in (measurement_id, channel):
        _identifier(value)
    if passage_id is not None:
        _identifier(passage_id)
    if type(complete) is not bool or type(measurable) is not bool:
        raise ValueError("完整性和可测性不能使用隐式真值转换")
    if not observation.n or not np.isfinite(observation.xyz).all():
        raise ValueError("全观测必须包含有限实测点")
    selected = {}
    for role, indices in (("vehicle", vehicle_indices), ("carriage", carriage_indices)):
        indices = np.asarray(indices)
        if indices.ndim != 1 or indices.dtype.kind not in "iu":
            raise ValueError("点云索引必须为一维整数")
        if len(indices) and (indices.min() < 0 or indices.max() >= observation.n or
                             np.any(indices[1:] <= indices[:-1])):
            raise ValueError("点云索引必须有序、唯一并位于原观测范围")
        selected[role] = indices.astype(np.int64, copy=True)
    if not np.isin(selected["carriage"], selected["vehicle"]).all():
        raise ValueError("车厢实测点必须属于车辆实测点")
    if not np.isfinite(detection.xyz).all():
        raise ValueError("判定云含非有限点")
    frames = {"observation": observation, "detection": detection}
    frames.update({role: observation._mask(indices) for role, indices in selected.items()})
    files = []
    records = {}

    def add(name: str, data: bytes, **metadata: Any) -> None:
        files.append((name, data))
        records[name] = {"sha256": _sha(data), "bytes": len(data), **metadata}

    for role in _ROLES:
        frame = frames[role]
        add(role + ".xyz", xyz_bytes(frame), role=role, points=frame.n,
            ring_present=frame.ring is not None,
            coordinates="detection" if role == "detection" else "source",
            origin="observed")
    for role, indices in selected.items():
        add(role + ".indices.npy", _indices_bytes(indices), role=role + "_indices")
    add("detection.xyz.reason.txt", str(result.get("reason", "")).encode("utf-8"), role="reason")
    add("detection.xyz.meta.json", _json(result), role="result")
    for index, frame in enumerate(extra_frames):
        if not np.isfinite(frame.xyz).all():
            raise ValueError("额外观测含非有限点")
        add("raw-%03d.xyz" % index, xyz_bytes(frame), role="diagnostic_observation",
            points=frame.n, ring_present=frame.ring is not None, coordinates="source",
            origin="observed", frame={"seq": frame.seq, "ts": frame.ts,
                                      "gen": frame.gen, "packets": frame.packets})
    meta = {
        "schema": "cloud-archive-v1", "measurement_id": measurement_id, "channel": channel,
        "passage_id": passage_id, "acquired_at": observation.ts,
        "configuration_hash": configuration_hash, "algorithm_version": algorithm_version,
        "baseline_version_id": baseline_version_id, "source_mode": source_mode,
        "frame": {"seq": observation.seq, "ts": observation.ts, "gen": observation.gen,
                  "packets": observation.packets, "complete": bool(complete),
                  "measurable": bool(measurable)},
        "detection_frame": {"seq": detection.seq, "ts": detection.ts, "gen": detection.gen},
        "roi": roi, "transforms": transforms or {}, "files": records,
        "empty_observation": (result.get("empty") is True and bool(complete) and bool(measurable)
                              and passage_id is not None and len(selected["carriage"]) > 0
                              and roi.get("carriage_status") in ("available", "partial")),
        "authoritative": False,
        "scope": "全部已解码观测点；未扫描、遮挡及推断点不属于实测归档",
    }
    if observation_window is not None:
        if detection_indices is None or detection_rotation is None:
            raise ValueError("显式来源窗口必须同时提供检测行索引和实际旋转矩阵")
        lineage = observation_window.detection_lineage(detection_indices)
        for index, pair in enumerate(observation_window.pairs):
            raw = pair.raw.frame()
            add("source-%03d.xyz" % index, xyz_bytes(raw), role="measurement_source",
                points=raw.n, ring_present=raw.ring is not None, coordinates="source",
                origin="observed", frame=pair.raw.identity())
            add("source-%03d.input-indices.npy" % index, _indices_bytes(pair.indices()),
                role="measurement_input_indices")
        add("detection.input-indices.npy", _indices_bytes(np.asarray(detection_indices)), role="detection_input_indices")
        add("detection.lineage.npy", _indices_bytes(lineage), role="detection_lineage")
        meta["provenance"] = dict(schema="observation-lineage-v1",
                                  aggregation="concatenate" if len(observation_window.pairs) > 1 else "single",
                                  segments=observation_window.segments(),
                                  detection_rotation=np.asarray(detection_rotation).tolist(),
                                  input_points=sum(len(pair.indices()) for pair in observation_window.pairs))
    elif detection_indices is not None or detection_rotation is not None:
        raise ValueError("检测追溯缺少来源窗口")
    if volume_window is not None:
        if observation_window is None or volume_extra_members is None:
            raise ValueError("体积来源必须显式说明检测主输入及额外货面成员")
        volume_ids = [(p.raw.seq, p.raw.ts, p.raw.gen, p.raw.packets) for p in volume_window.pairs]
        primary_members = []
        for pair in observation_window.pairs:
            identity = (pair.raw.seq, pair.raw.ts, pair.raw.gen, pair.raw.packets)
            if identity not in volume_ids:
                raise ValueError("体积来源遗漏检测主输入成员")
            primary_members.append(volume_ids.index(identity))
        for index, pair in enumerate(volume_window.pairs):
            raw = pair.raw.frame()
            add("volume-source-%03d.xyz" % index, xyz_bytes(raw), role="volume_surface_source",
                points=raw.n, ring_present=raw.ring is not None, coordinates="source",
                origin="observed", frame=pair.raw.identity())
            add("volume-source-%03d.input-indices.npy" % index, _indices_bytes(pair.indices()),
                role="volume_surface_input_indices")
        meta["volume_support"] = dict(schema="observation-lineage-v1",
                                       aggregation="concatenate" if len(volume_window.pairs) > 1 else "single",
                                       segments=volume_window.segments(),
                                       input_points=sum(len(pair.indices()) for pair in volume_window.pairs),
                                       primary_members=primary_members, extra_members=list(volume_extra_members),
                                       usage="主输入与额外货面分别列成员；同一成员角色复用不等于新帧")
    elif volume_extra_members is not None:
        raise ValueError("体积额外成员缺少来源窗口")
    if runtime_files:
        from .archive_io import RUNTIME_ROLES
        for name, content in runtime_files:
            if name not in RUNTIME_ROLES or name in records:
                raise ValueError("运行扩展有未知或重复成员")
            add(name, content, role=RUNTIME_ROLES[name])
        meta["measurement_runtime"] = dict(schema="measurement-runtime-v3",
            baseline_asset="baseline.npz" if "baseline.npz" in records else None,
            enter_frames="enter-frames.f32le" if "enter-frames.f32le" in records else None,
            exit_frames="exit-frames.f32le" if "exit-frames.f32le" in records else None)
    bundle = FrozenBundle(_json(meta), tuple(sorted(files)))
    bundle.validate()
    return bundle


def _validate_provenance(meta: dict, files: dict, volume_support: bool = False) -> None:
    """读回时核验各行的来源及真实变换；不能仅检查一个末帧身份。"""
    provenance = meta["volume_support" if volume_support else "provenance"]
    prefix = "volume-source" if volume_support else "source"
    segments = provenance.get("segments")
    if provenance.get("schema") != "observation-lineage-v1" or not isinstance(segments, list) or not segments:
        raise ValueError("来源窗口无效")
    if provenance["aggregation"] != ("concatenate" if len(segments) > 1 else "single"):
        raise ValueError("来源聚合模式不一致")
    if volume_support:
        if "provenance" not in meta:
            raise ValueError("体积来源缺少主检测来源")
        members = []
        for key in ("primary_members", "extra_members"):
            selected = provenance.get(key)
            if (not isinstance(selected, list) or any(type(index) is not int or not 0 <= index < len(segments) for index in selected)
                    or selected != sorted(set(selected))):
                raise ValueError("体积用途成员无效")
            members.extend(selected)
        if not provenance["primary_members"] or set(members) != set(range(len(segments))):
            raise ValueError("体积用途未覆盖全部来源或缺少主输入")
    matrix = np.eye(3) if volume_support else np.asarray(provenance["detection_rotation"], np.float64)
    if (matrix.shape != (3, 3) or not np.isfinite(matrix).all()
            or not np.allclose(matrix @ matrix.T, np.eye(3), rtol=0, atol=1e-10)
            or abs(np.linalg.det(matrix) - 1.) > 1e-10):
        raise ValueError("检测实际旋转不是有效刚体旋转")

    def cloud(name):
        record = meta["files"][name]
        data = np.frombuffer(files[name], dtype=XYZ_DTYPE, offset=4)
        xyz = np.column_stack([data[key] for key in ("x", "y", "z")])
        if not np.isfinite(xyz).all():
            raise ValueError("来源点云含非有限坐标")
        return xyz, data["ring"] if record["ring_present"] else None

    def indices(name, count):
        value = np.load(io.BytesIO(files[name]), allow_pickle=False)
        if (value.ndim != 1 or value.dtype.kind not in "iu" or len(value) and
                (value.min() < 0 or value.max() >= count or np.any(value[1:] <= value[:-1]))):
            raise ValueError("来源行索引无效")
        return value

    points, rings, expected_lineage = [], [], []
    raw_offset, input_offset, previous = 0, 0, None
    for index, segment in enumerate(segments):
        identity = segment["frame"]
        source_name = "%s-%03d.xyz" % (prefix, index)
        record = meta["files"][source_name]
        if (segment["member"] != index or segment["channel"] != meta["channel"]
                or segment["epoch"] != segments[0]["epoch"] or segment["complete"] is not True
                or identity["gen"] != meta["frame"]["gen"] or record["frame"] != identity
                or type(identity["seq"]) is not int or identity["seq"] <= 0
                or type(identity["packets"]) is not int or identity["packets"] < 0
                or not math.isfinite(identity["ts"]) or identity["ts"] <= 0
                or not math.isfinite(segment["received_monotonic"]) or segment["received_monotonic"] < 0):
            raise ValueError("来源成员身份或完整性不一致")
        if previous is not None and (identity["seq"] <= previous["frame"]["seq"]
                or identity["ts"] <= previous["frame"]["ts"]
                or segment["received_monotonic"] <= previous["received_monotonic"]):
            raise ValueError("来源成员重复或乱序")
        xyz, ring = cloud(source_name)
        selected = indices("%s-%03d.input-indices.npy" % (prefix, index), len(xyz))
        if (segment["raw_rows"] != [raw_offset, raw_offset + len(xyz)]
                or segment["input_rows"] != [input_offset, input_offset + len(selected)]
                or segment["raw_ring_present"] is not (ring is not None)
                or segment["input_ring_present"] is not (ring is not None)):
            raise ValueError("来源分段行区间或 ring 元数据不一致")
        points.append(xyz[selected])
        rings.append(ring[selected] if ring is not None else None)
        expected_lineage.append(np.column_stack((np.full(len(selected), index, np.int64), selected)))
        raw_offset += len(xyz)
        input_offset += len(selected)
        previous = segment
    if (provenance["input_points"] != input_offset
            or any(previous["frame"][key] != meta["frame"][key] for key in ("seq", "ts", "gen", "packets"))
            or files["%s-%03d.xyz" % (prefix, len(segments) - 1)] != files["observation.xyz"]):
        raise ValueError("主观测不是来源窗口末帧或输入点数不一致")
    if volume_support:
        return
    selected = indices("detection.input-indices.npy", input_offset)
    lineage = np.load(io.BytesIO(files["detection.lineage.npy"]), allow_pickle=False)
    if lineage.dtype.kind not in "iu" or not np.array_equal(lineage, np.concatenate(expected_lineage)[selected]):
        raise ValueError("检测原始行关系不一致")
    expected = (np.concatenate(points).astype(np.float64)[selected] @ matrix.T).astype(np.float32)
    actual, actual_ring = cloud("detection.xyz")
    expected_ring = np.concatenate(rings)[selected] if all(ring is not None for ring in rings) else None
    if (not np.array_equal(expected, actual) or (actual_ring is None) != (expected_ring is None)
            or expected_ring is not None and not np.array_equal(expected_ring, actual_ring)):
        raise ValueError("检测云与来源索引、ring 或实际变换不一致")


def _safe_path(path: Path) -> Path:
    """拒绝符号链接及 Windows junction；不允许归档穿过链接写向其它目录。"""
    path = Path(os.path.abspath(str(path)))
    for part in (path, *path.parents):
        try:
            info = part.lstat()
        except FileNotFoundError:
            continue
        if stat.S_ISLNK(info.st_mode) or getattr(info, "st_file_attributes", 0) & 0x400:
            raise ValueError("归档路径不能含符号链接或重解析点")
    from .storage import is_baseline, root
    if is_baseline(str(path)):
        raise ValueError("禁止将归档写入回归样本目录")
    evidence = Path(os.path.abspath(root("re")))
    if evidence == path or evidence in path.parents:
        raise ValueError("禁止将归档写入只读取证目录")
    return path


def _sync_directory(path: Path) -> None:
    if os.name != "nt":
        descriptor = os.open(str(path), os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def _mkdir(path: Path) -> None:
    missing = []
    cursor = path
    while not cursor.exists():
        missing.append(cursor)
        cursor = cursor.parent
    for directory in reversed(missing):
        _safe_path(directory)
        directory.mkdir(exist_ok=True)
        _sync_directory(directory.parent)


def _write_once(path: Path, data: bytes) -> None:
    """只提交完整文件；冲突不覆盖。跨进程同内容重放也安全。"""
    _safe_path(path)
    if path.exists():
        if path.read_bytes() != data:
            raise ArchiveConflict("归档目标已有不同内容: " + path.name)
        return
    temporary = path.with_name(".pending-" + uuid.uuid4().hex)
    try:
        with open(str(temporary), "xb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            # link 为原子“不存在才创建”，避免 replace 覆盖并发写入的冲突文件。
            os.link(str(temporary), str(path))
        except FileExistsError:
            if path.read_bytes() != data:
                raise ArchiveConflict("归档目标已有不同内容: " + path.name)
        _sync_directory(path.parent)
    finally:
        if temporary.exists():
            temporary.unlink()


class ArchiveJournal:
    """使用调用者的 SQLite 连接/锁；put 不自行提交，可与 outbox 同一事务。"""

    def __init__(self, connection: sqlite3.Connection, lock: threading.RLock, *, owner_kind="archive"):
        self.connection = connection
        self.lock = lock
        # 文件投影单独串行化，不能用 outbox 的数据库锁包住磁盘 I/O。
        self._export_lock = threading.Lock()
        self._closed = False
        self._cleanup_ready = False
        from .archive_capacity import CapacityProbe
        self._capacity = CapacityProbe(connection, lock, lambda: self._closed, owner_kind)
        with lock:
            connection.execute("""CREATE TABLE IF NOT EXISTS cloud_archives (
                seq INTEGER PRIMARY KEY AUTOINCREMENT, measurement_id TEXT NOT NULL UNIQUE,
                channel TEXT NOT NULL, passage_id TEXT, acquired_at REAL NOT NULL,
                manifest BLOB NOT NULL, manifest_sha256 TEXT NOT NULL,
                status TEXT NOT NULL DEFAULT 'pending' CHECK(status IN ('pending','complete')),
                attempts INTEGER NOT NULL DEFAULT 0, last_error TEXT)""")
            connection.execute("""CREATE TABLE IF NOT EXISTS cloud_archive_files (
                measurement_id TEXT NOT NULL, name TEXT NOT NULL, content BLOB NOT NULL,
                PRIMARY KEY(measurement_id,name))""")
            connection.execute("""CREATE TABLE IF NOT EXISTS empty_observations (
                channel TEXT NOT NULL, passage_id TEXT NOT NULL, measurement_id TEXT NOT NULL,
                acquired_at REAL NOT NULL, PRIMARY KEY(channel,passage_id))""")
            connection.execute("""CREATE TABLE IF NOT EXISTS empty_observation_latest (
                channel TEXT PRIMARY KEY, measurement_id TEXT NOT NULL,
                acquired_at REAL NOT NULL, archive_seq INTEGER NOT NULL)""")
    def ensure_cleanup_schema(self):
        """清理专属迁移在后台执行；失败不增加采集启动的硬依赖。"""
        with self.lock:
            if self._cleanup_ready:
                return
            connection = self.connection
            connection.execute("BEGIN IMMEDIATE")
            try:
                connection.execute("""CREATE TABLE IF NOT EXISTS cloud_archive_cleanup (
                measurement_id TEXT PRIMARY KEY, cutoff REAL NOT NULL,
                state TEXT NOT NULL CHECK(state IN ('deleting','purged')))""")
                columns = {row[1] for row in connection.execute("PRAGMA table_info(cloud_archives)")}
                predicate = "status='complete'" + (" AND completed_at IS NULL" if "completed_at" in columns else "")
                size = connection.execute("SELECT coalesce(sum(length(manifest)),0) FROM cloud_archives WHERE " + predicate).fetchone()[0]
                database = connection.execute("PRAGMA database_list").fetchone()[2]
                if size and database and shutil.disk_usage(Path(database).parent).free < int(size) * 2 + 16 * 1024 * 1024:
                    raise OSError(errno.ENOSPC, "归档清理迁移缺少WAL安全空间，保留原件并稍后重试")
                if "completed_at" not in columns:
                    connection.execute("ALTER TABLE cloud_archives ADD COLUMN completed_at REAL")
                # 旧包保守从观察时刻起算；不把未知完成时刻倒推为采集时间。
                connection.execute("UPDATE cloud_archives SET completed_at=? WHERE status='complete' AND completed_at IS NULL", (time.time(),))
                connection.commit()
                self._cleanup_ready = True
            except Exception:
                connection.rollback()
                raise

    def receipt(self, measurement_id: str) -> Optional[bytes]:
        """归档凭据永久保留，即使普通点云已经清理也能验证同MID幂等。"""
        with self.lock:
            row = self.connection.execute("SELECT manifest,manifest_sha256 FROM cloud_archives WHERE measurement_id=?",
                                          (_identifier(measurement_id),)).fetchone()
            if row is None:
                return None
            value = bytes(row[0])
            if _sha(value) != row[1]:
                raise ArchiveConflict("持久归档凭据摘要损坏")
            return value

    def is_pruned(self, measurement_id: str) -> bool:
        with self.lock:
            if self.connection.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='cloud_archive_cleanup'").fetchone() is None:
                return False
            return self.connection.execute("SELECT 1 FROM cloud_archive_cleanup WHERE measurement_id=?",
                                           (_identifier(measurement_id),)).fetchone() is not None

    def put(self, bundle: FrozenBundle) -> str:
        """须在持有 lock 的显式事务中调用；同 ID 只能重放完全相同的包。"""
        if not self.connection.in_transaction:
            raise RuntimeError("归档必须与调用者测量事务一起提交")
        meta = bundle.validate()
        old = self.connection.execute("SELECT manifest FROM cloud_archives WHERE measurement_id=?",
                                      (meta["measurement_id"],)).fetchone()
        if old is not None:
            if (bytes(old[0]) != bundle.manifest or
                    not self.is_pruned(meta["measurement_id"]) and self.get(meta["measurement_id"]) != bundle):
                raise ArchiveConflict("同一测量归档已经固定，禁止覆盖")
            return meta["measurement_id"]
        self.connection.execute("""INSERT INTO cloud_archives
            (measurement_id,channel,passage_id,acquired_at,manifest,manifest_sha256)
            VALUES (?,?,?,?,?,?)""", (meta["measurement_id"], meta["channel"], meta["passage_id"],
                                    meta["acquired_at"], bundle.manifest, _sha(bundle.manifest)))
        self.connection.executemany("INSERT INTO cloud_archive_files VALUES (?,?,?)",
                                    [(meta["measurement_id"], name, data) for name, data in bundle.files])
        return meta["measurement_id"]

    def get(self, measurement_id: str) -> Optional[FrozenBundle]:
        with self.lock:
            if self.is_pruned(measurement_id):
                return None
            row = self.connection.execute("SELECT manifest,manifest_sha256,channel,passage_id,acquired_at FROM cloud_archives WHERE measurement_id=?",
                                          (_identifier(measurement_id),)).fetchone()
            if row is None:
                return None
            manifest = bytes(row[0])
            rows = self.connection.execute("SELECT name,content FROM cloud_archive_files WHERE measurement_id=? ORDER BY name",
                                           (measurement_id,)).fetchall()
            bundle = FrozenBundle(manifest, tuple((name, bytes(data)) for name, data in rows))
        if _sha(manifest) != row[1]:
            raise ArchiveConflict("持久归档 manifest 摘要损坏")
        meta = bundle.validate()
        if (meta["measurement_id"] != measurement_id or meta["channel"] != row[2]
                or meta["passage_id"] != row[3] or meta["acquired_at"] != row[4]):
            raise ArchiveConflict("持久归档索引与 manifest 身份不一致")
        return bundle

    def pending(self, channel: str, limit: int = 8) -> list:
        if type(limit) is not int or not 1 <= limit <= 100:
            raise ValueError("归档批次必须为 1 至 100")
        with self.lock:
            return [row[0] for row in self.connection.execute(
                "SELECT measurement_id FROM cloud_archives WHERE channel=? AND status='pending' ORDER BY seq LIMIT ?",
                (_identifier(channel), limit))]

    def pending_after(self, channel: str, after: str = "", limit: int = 8) -> list:
        """有限批次轮转；坏包仍保留，但不能永久饿死队尾。"""
        if type(limit) is not int or not 1 <= limit <= 100:
            raise ValueError("归档批次必须为 1 至 100")
        with self.lock:
            row = self.connection.execute("SELECT seq FROM cloud_archives WHERE measurement_id=?", (after,)).fetchone()
            values = [item[0] for item in self.connection.execute(
                "SELECT measurement_id FROM cloud_archives WHERE channel=? AND status='pending' AND seq>? ORDER BY seq LIMIT ?",
                (_identifier(channel), row[0] if row else 0, limit))]
            return values or self.pending(channel, limit)

    def _complete(self, measurement_id: str, meta: dict) -> None:
        """manifest 已落盘后才发布观测引用；状态与引用同事务。"""
        self.connection.execute("BEGIN IMMEDIATE")
        try:
            if meta["empty_observation"]:
                sequence = self.connection.execute("SELECT seq FROM cloud_archives WHERE measurement_id=?",
                                                   (measurement_id,)).fetchone()[0]
                self.connection.execute("""INSERT INTO empty_observations VALUES (?,?,?,?)
                    ON CONFLICT(channel,passage_id) DO UPDATE SET
                    measurement_id=excluded.measurement_id,acquired_at=excluded.acquired_at
                    WHERE excluded.acquired_at > empty_observations.acquired_at
                    AND (SELECT seq FROM cloud_archives WHERE measurement_id=excluded.measurement_id)
                    > (SELECT seq FROM cloud_archives WHERE measurement_id=empty_observations.measurement_id)""",
                    (meta["channel"], meta["passage_id"], measurement_id, meta["acquired_at"]))
                # 持久顺序与采集时间都向前才替换当前引用。旧包迟交、墙钟回拨均不能抢占。
                self.connection.execute("""INSERT INTO empty_observation_latest VALUES (?,?,?,?)
                    ON CONFLICT(channel) DO UPDATE SET measurement_id=excluded.measurement_id,
                    acquired_at=excluded.acquired_at,archive_seq=excluded.archive_seq
                    WHERE excluded.archive_seq > empty_observation_latest.archive_seq
                    AND excluded.acquired_at > empty_observation_latest.acquired_at""",
                    (meta["channel"], measurement_id, meta["acquired_at"], sequence))
            if "completed_at" in {row[1] for row in self.connection.execute("PRAGMA table_info(cloud_archives)")}:
                self.connection.execute("UPDATE cloud_archives SET status='complete',last_error=NULL,"
                                        "completed_at=coalesce(completed_at,?) WHERE measurement_id=?",
                                        (time.time(), measurement_id))
            else:
                self.connection.execute("UPDATE cloud_archives SET status='complete',last_error=NULL WHERE measurement_id=?", (measurement_id,))
            self.connection.commit()
        except Exception:
            self.connection.rollback()
            raise

    def export(self, measurement_id: str, output_dir: str, *, channel: str) -> str:
        """无硬件恢复：只从持久字节投影，manifest 最后写入，绝不重新检测。"""
        with self._export_lock:
            if self._closed:
                raise RuntimeError("归档库已关闭，禁止新导出")
            if self.is_pruned(measurement_id):
                raise ValueError("普通点云已按日清理，不能重新导出")
            # get 内部仅在读出不可变快照期间持数据库锁。
            bundle = self.get(measurement_id)
            if bundle is None:
                raise ValueError("归档尚未持久提交")
            meta = bundle.metadata
            if meta["channel"] != _identifier(channel):
                raise ValueError("禁止把归档输出到另一通道")
            directory = _safe_path(Path(output_dir) / ARCHIVE_DIR / _component(channel) /
                                   (_component(meta["passage_id"]) if meta["passage_id"] is not None else "unbound") /
                                   _component(measurement_id))
            try:
                _mkdir(directory)
                _safe_path(directory)
                # 不能以 manifest 存在替代逐文件核验；缺失文件重建，损坏内容报冲突。
                for name, data in bundle.files:
                    _write_once(directory / name, data)
                _write_once(directory / "manifest.json", bundle.manifest)
                with self.lock:
                    self._complete(measurement_id, meta)
            except Exception as error:
                # ★ 记账失败绝不能吞掉/替换原始的导出错误（2026-09-23，M13）：
                #   原来 BEGIN IMMEDIATE 写在 try 外面 —— 清理进程（cleanup
                #   --apply）持写锁超时时的 OperationalError 会**替换**磁盘满/
                #   权限这类真正的根因向外抛，且 attempts/last_error 不落库。
                #   现在 BEGIN 也进 try；记账失败时原错误进消息并挂 __cause__。
                bookkeeping = None
                with self.lock:
                    try:
                        self.connection.execute("BEGIN IMMEDIATE")
                        try:
                            self.connection.execute("UPDATE cloud_archives SET status='pending',attempts=attempts+1,last_error=? WHERE measurement_id=?",
                                                    (type(error).__name__, measurement_id))
                            # 保留引用水位；reference 对 pending 返回未知，不能让旧包趁故障回退抢占。
                            self.connection.commit()
                        except Exception as inner:
                            try:
                                self.connection.rollback()
                            except Exception:                     # noqa: BLE001
                                pass                             # 回滚失败也不该盖过真正的失败原因
                            raise inner
                    except Exception as outer:                   # noqa: BLE001
                        bookkeeping = outer
                if bookkeeping is not None:
                    raise RuntimeError(
                        "导出失败后记账也失败：%s: %s（原始导出错误：%s）—— 包仍为 pending 安全，"
                        "但 attempts/last_error 未落库"
                        % (type(bookkeeping).__name__, bookkeeping, error)
                    ) from error
                raise
            return str(directory / "detection.xyz")

    def purge_completed(self, measurement_ids, before: float, output_dirs: dict, *, protected_roots=()) -> dict:
        """有业务完成证据的一小批普通云；墓碑阻止重启后再次投影。

        调用者还须证明 outbox 已发送、对应待办已提交。这里独立复验归档完成时间，
        空车观测不是正式基线，同样清理。保留幂等manifest与所有NPZ资产。
        """
        if type(before) not in (int, float) or not math.isfinite(before) or before <= 0:
            raise ValueError("日清截止时间必须为有限正数")
        ids = tuple(dict.fromkeys(_identifier(mid) for mid in measurement_ids))
        if len(ids) > 128:
            raise ValueError("日清每批最多128份归档")
        self.ensure_cleanup_schema()
        report = dict(purged=[], bytes_reusable=0)
        with self._export_lock:
            for mid in ids:
                with self.lock:
                    row = self.connection.execute("SELECT channel,passage_id,acquired_at,status,completed_at "
                        "FROM cloud_archives WHERE measurement_id=?", (mid,)).fetchone()
                    intent = self.connection.execute("SELECT state FROM cloud_archive_cleanup WHERE measurement_id=?", (mid,)).fetchone()
                    if intent is not None and intent[0] == "purged":
                        report["purged"].append(mid)
                        continue
                    if (row is None or row[3] != "complete" or row[4] is None
                            or row[2] >= before or row[4] >= before or row[0] not in output_dirs):
                        continue
                    target = Path(os.path.abspath(Path(output_dirs[row[0]]) / ARCHIVE_DIR / _component(row[0]) /
                        (_component(row[1]) if row[1] is not None else "unbound") / _component(mid)))
                    for protected in protected_roots:
                        permanent = Path(protected).resolve()
                        if target.is_relative_to(permanent) or permanent.is_relative_to(target):
                            raise ValueError("归档删除范围与永久基线目录重叠，保留原件")
                    receipt = self.receipt(mid)
                    metadata = json.loads(receipt) if receipt is not None else {}
                    if (metadata.get("measurement_id") != mid or metadata.get("channel") != row[0]
                            or metadata.get("passage_id") != row[1] or metadata.get("acquired_at") != row[2]):
                        raise ArchiveConflict("日清归档凭据与索引不一致，保留原件")
                    size = self.connection.execute("SELECT coalesce(sum(length(content)),0) FROM cloud_archive_files WHERE measurement_id=?", (mid,)).fetchone()[0]
                    database = self.connection.execute("PRAGMA database_list").fetchone()[2]
                    if database and shutil.disk_usage(Path(database).parent).free < int(size) * 2 + 16 * 1024 * 1024:
                        raise OSError(errno.ENOSPC, "归档维护缺少WAL安全空间，请先迁到数据盘或腾出空间")
                    self.connection.execute("BEGIN IMMEDIATE")
                    try:
                        self.connection.execute("INSERT OR IGNORE INTO cloud_archive_cleanup VALUES (?,?,'deleting')", (mid, before))
                        self.connection.commit()
                    except Exception:
                        self.connection.rollback()
                        raise
                directory = _safe_path(Path(output_dirs[row[0]]) / ARCHIVE_DIR / _component(row[0]) /
                                       (_component(row[1]) if row[1] is not None else "unbound") / _component(mid))
                # 意图已持久化；删除失败仍保留原BLOB，下次只续删，不重新导出。
                if directory.exists():
                    shutil.rmtree(directory)
                with self.lock:
                    self.connection.execute("BEGIN IMMEDIATE")
                    try:
                        size = self.connection.execute("SELECT coalesce(sum(length(content)),0) FROM cloud_archive_files WHERE measurement_id=?", (mid,)).fetchone()[0]
                        self.connection.execute("DELETE FROM cloud_archive_files WHERE measurement_id=?", (mid,))
                        self.connection.execute("DELETE FROM empty_observations WHERE measurement_id=?", (mid,))
                        self.connection.execute("DELETE FROM empty_observation_latest WHERE measurement_id=?", (mid,))
                        self.connection.execute("UPDATE cloud_archive_cleanup SET state='purged' WHERE measurement_id=?", (mid,))
                        self.connection.commit()
                    except Exception:
                        self.connection.rollback()
                        raise
                report["purged"].append(mid)
                report["bytes_reusable"] += int(size)
        return report

    def prune(self, before: float, *, apply: bool = False, output_dir: str = "") -> dict:
        """按采集时间清理成套归档；**空车与未落定的包一律不删**。

        为什么归档必须自带清理（2026-09-18）：`cli/cleanup.py` 走的是
        `storage.scan` / `remove_sample`，而这两个函数看到 `archives-v1`
        **一律跳过**（`storage.is_archive`）—— 那是对的，成套包不能被逐个
        XYZ 地拆。结果是归档库**没有任何删除路径，只涨不删**。

        四条保留规则，任何一条命中就留下：
          ① `empty_observation` —— 空车观测是基线的原始材料，**永久保留**；
          ② `empty_observation_latest` 引用到的 —— 本通道最新那份空车；
          ③ `status='pending'` —— 还没导出/上报，删了就是丢数据；
          ④ 采集时间不早于 `before`。
        其余按 `acquired_at` 清理。

        ▲ 默认 `apply=False` 只报不删，与 `cli/cleanup.py` 同一套规矩：
          "检测记录该留多久是业务决定，不该由程序替人删"。

        ▲▲ `output_dir` 非空时，apply 会**先删磁盘投影、后删数据库行**
        （2026-09-23，两轮审查 H4）：`export()` 投影出的
        `<out>/archives-v1/<通道>/<车次>/<测量>/` 目录原来没有任何删除
        路径 —— DB 删了文件还在，孤儿投影仍会被 `discover_samples` 加载，
        磁盘只涨不降。顺序上"先文件后行"：文件删失败就跳过该行不删，
        保持「目录在 = 行在」的对账关系，重跑清理即可续删。
        """
        if not isinstance(before, (int, float)) or before != before or before in (float("inf"), float("-inf")):
            raise ValueError("清理时间点必须为有限实数")
        if isinstance(apply, (int, float)) and not isinstance(apply, bool):
            raise ValueError("apply 必须是布尔值")
        with self.lock:
            # 空车判定读 manifest 里的 empty_observation，与 reference() 同一口径，
            # 不去解包点云，也不依赖落盘文件是否还在。
            rows = self.connection.execute(
                "SELECT measurement_id,channel,passage_id,acquired_at,status,manifest FROM cloud_archives "
                "ORDER BY seq").fetchall()
            latest = {row[0] for row in self.connection.execute(
                "SELECT measurement_id FROM empty_observation_latest").fetchall()}
            report = dict(scanned=len(rows), kept_empty=0, kept_pending=0, kept_recent=0,
                          removable=[], removed=0, removed_bytes=0, applied=bool(apply),
                          removed_dirs=0, dir_failures=[])
            victims = []
            for measurement_id, channel, passage_id, acquired_at, status, manifest in rows:
                if self.is_pruned(measurement_id):
                    continue  # 日清凭据不可被旧按天清理器删掉，否则旧MID会失去幂等保护。
                try:
                    empty = bool(json.loads(bytes(manifest)).get("empty_observation"))
                except (ValueError, TypeError):
                    empty = True                  # 读不懂的包按"可能是空车"处理，宁留勿删
                if empty or measurement_id in latest:
                    report["kept_empty"] += 1
                    continue
                if status != "complete":
                    report["kept_pending"] += 1
                    continue
                if not (float(acquired_at) < before):
                    report["kept_recent"] += 1
                    continue
                victims.append((measurement_id, channel, float(acquired_at), passage_id))
            for measurement_id, channel, acquired_at, _passage in victims:
                size = self.connection.execute(
                    "SELECT COALESCE(SUM(LENGTH(content)),0) FROM cloud_archive_files WHERE measurement_id=?",
                    (measurement_id,)).fetchone()[0]
                report["removable"].append(dict(measurement_id=measurement_id, channel=channel,
                                                acquired_at=acquired_at, bytes=int(size)))
                report["removed_bytes"] += int(size)
            if apply and victims:
                # 先清磁盘投影（H4）：目录在才删、删不动就跳过该行（行留给重跑）。
                deletable = []
                for measurement_id, channel, _acquired_at, passage_id in victims:
                    if output_dir:
                        directory = _safe_path(Path(output_dir) / ARCHIVE_DIR / _component(channel) /
                                               (_component(passage_id) if passage_id is not None else "unbound") /
                                               _component(measurement_id))
                        try:
                            if directory.exists():
                                shutil.rmtree(directory)
                                report["removed_dirs"] += 1
                        except Exception as error:            # noqa: BLE001
                            report["dir_failures"].append(
                                dict(measurement_id=measurement_id, channel=channel,
                                     error="%s: %s" % (type(error).__name__, error)))
                            continue
                    deletable.append(measurement_id)
                self.connection.execute("BEGIN IMMEDIATE")
                try:
                    for measurement_id in deletable:
                        self.connection.execute("DELETE FROM cloud_archive_files WHERE measurement_id=?",
                                                (measurement_id,))
                        # empty_observations 里不该有它们（空车已被①挡下），
                        # 但历史库可能留有孤行，一并清掉免得悬空引用。
                        self.connection.execute("DELETE FROM empty_observations WHERE measurement_id=?",
                                                (measurement_id,))
                        self.connection.execute("DELETE FROM cloud_archives WHERE measurement_id=?",
                                                (measurement_id,))
                    self.connection.commit()
                except Exception:
                    self.connection.rollback()
                    raise
                report["removed"] = len(deletable)
                return report
            return report

    def capacity_snapshot(self, *, refresh=False) -> dict:
        """维护用途的异步缓存，默认路径不调用，不读取归档内容。"""
        return self._capacity.snapshot(refresh=refresh)

    def close_connection(self) -> None:
        """先等已开始的导出结束，再关连接；锁序始终为导出锁、数据库锁。"""
        with self._export_lock, self.lock:
            if not self._closed:
                self.connection.close()
                self._closed = True

    def reference(self, channel: str, passage_id: Optional[str] = None) -> Optional[dict]:
        """非权威观测引用，不按相机车牌跨车复用，不参与正式基线选择。"""
        with self.lock:
            where = "channel=?"
            params = [_identifier(channel)]
            if passage_id is not None:
                where += " AND passage_id=?"
                params.append(_identifier(passage_id))
            table = "empty_observations" if passage_id is not None else "empty_observation_latest"
            rows = self.connection.execute("SELECT measurement_id,acquired_at FROM " + table + " WHERE " + where +
                                           " ORDER BY acquired_at DESC,measurement_id LIMIT 2", params).fetchall()
            if not rows:
                return None
            if self.is_pruned(rows[0][0]):
                return None
            status = self.connection.execute("SELECT status FROM cloud_archives WHERE measurement_id=?",
                                             (rows[0][0],)).fetchone()
            if status is None:
                raise ArchiveConflict("观测引用缺少归档状态")
            if status[0] != "complete":
                return None
            # ★ 防御性断言（2026-09-23 标注定性）：上面两张表的主键分别是
            #   (channel, passage_id) 与 (channel)，WHERE 条件全部命中主键
            #   前缀 —— 当前查询结构下 LIMIT 2 最多返回 1 行，这条分支
            #   **结构性不触发**。留着它防的是将来查询条件被放宽（比如
            #   按 acquired_at 区间取）时"同刻双车次"静默任选一条 ——
            #   那种歧义宁可报冲突，不猜。别当死代码删。
            if len(rows) > 1 and rows[0][1] == rows[1][1]:
                raise ArchiveConflict("同一采集时刻存在不同车次观测，不能猜测最新车")
            bundle = self.get(rows[0][0])
            if bundle is None:
                raise ArchiveConflict("观测引用缺少持久归档")
            meta = bundle.metadata
            if (meta["channel"] != channel or not meta["empty_observation"]
                    or meta["acquired_at"] != rows[0][1]
                    or passage_id is not None and meta["passage_id"] != passage_id):
                raise ArchiveConflict("观测引用与原归档身份或资格不一致")
            return {"authoritative": False, "measurement_id": rows[0][0], "channel": channel,
                    "passage_id": meta["passage_id"], "acquired_at": meta["acquired_at"],
                    "configuration_hash": meta["configuration_hash"], "roi": meta["roi"],
                    "manifest_sha256": _sha(bundle.manifest),
                    "carriage_available": meta["files"]["carriage.xyz"]["points"] > 0}


class ArchiveStore:
    """无 WebApi 时的独立归档库；不伪造任何业务 outbox 测量。"""

    def __init__(self, output_dir: str):
        # 记下原始 output_dir（构造投影路径用；prune 删投影时也走它，见 H4）。
        self.output_dir = str(output_dir)
        directory = _safe_path(Path(output_dir) / ARCHIVE_DIR)
        _mkdir(directory)
        database = _safe_path(directory / "archive.sqlite3")
        for suffix in ("-wal", "-shm"):
            _safe_path(Path(str(database) + suffix))
        self.connection = sqlite3.connect(str(database), timeout=10, isolation_level=None, check_same_thread=False)
        self.lock = threading.RLock()
        try:
            if self.connection.execute("PRAGMA journal_mode=WAL").fetchone()[0].lower() != "wal":
                raise RuntimeError("归档库必须支持 WAL")
            self.connection.execute("PRAGMA synchronous=FULL")
            self.journal = ArchiveJournal(self.connection, self.lock)
        except Exception:
            self.connection.close()
            raise

    def save(self, bundle: FrozenBundle) -> str:
        with self.lock:
            self.connection.execute("BEGIN IMMEDIATE")
            try:
                value = self.journal.put(bundle)
                self.connection.commit()
                return value
            except Exception:
                self.connection.rollback()
                raise

    def capacity_snapshot(self, *, refresh=False) -> dict:
        return self.journal.capacity_snapshot(refresh=refresh)

    def prune(self, before: float, *, apply: bool = False) -> dict:
        """按采集时间清理成套归档；空车与未落定的包不删。见 ArchiveJournal.prune。

        顺带清磁盘投影（两轮审查 H4）：本类的 output_dir 就是投影根目录。
        """
        return self.journal.prune(before, apply=apply, output_dir=self.output_dir)

    def close(self) -> None:
        self.journal.close_connection()
