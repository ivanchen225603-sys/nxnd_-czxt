"""按逻辑测量发现样本；只用标准库，不把归档角色误当连续帧。"""
from __future__ import annotations

from dataclasses import dataclass, field
import fnmatch
from functools import lru_cache
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
import struct
import sys
from typing import List, Optional
from .archive_io import ArchiveReadLimits, _read_regular, bounded_json, check_budget


ARCHIVE_DIR = "archives-v1"
ROLES = ("observation", "vehicle", "carriage", "detection")
REPLAY_NOTE = "新归档仅重跑主窗口末帧原观测；不是原多帧检测或原连续停稳过程的精确复现"


@dataclass(frozen=True)
class Sample:
    path: str
    role: str = "legacy"
    channel: str = ""
    measurement_id: str = ""
    acquired_at: float = 0.0
    manifest_sha256: str = ""
    window_members: int = 0

    @property
    def archived(self) -> bool:
        return bool(self.manifest_sha256)

    @property
    def output_key(self) -> str:
        """固定长度、安全字符，不同包/旧文件不能因同 basename 覆盖。"""
        identity = (self.channel + "\0" + self.measurement_id if self.archived
                    else os.path.normcase(os.path.abspath(self.path)))
        return hashlib.sha256(identity.encode("utf-8")).hexdigest()


@dataclass
class Discovery:
    samples: List[Sample] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)

    def report(self) -> None:
        for error in self.errors:
            print("样本发现: " + error, file=sys.stderr)


def _not_link(path: Path) -> None:
    info = path.lstat()
    if stat.S_ISLNK(info.st_mode) or getattr(info, "st_file_attributes", 0) & 0x400:
        raise ValueError("不读取符号链接或重解析点")


def _identifier(value) -> bool:
    return (isinstance(value, str) and value not in (".", "..") and
            re.fullmatch(r"[A-Za-z0-9_.:-]{1,64}", value) is not None)


def _matches(path: Path, root: Path, pattern: str) -> bool:
    """在已安全发现的路径上匹配 glob，不再通过 glob 递归链接目录。"""
    parts, patterns = path.relative_to(root).parts, Path(pattern).parts

    @lru_cache(None)
    def match(index: int, current: int) -> bool:
        if current == len(patterns):
            return index == len(parts)
        word = patterns[current]
        if word == "**":
            return (match(index, current + 1) or index < len(parts) and
                    not parts[index].startswith(".") and match(index + 1, current))
        return (index < len(parts) and (not parts[index].startswith(".") or word.startswith(".")) and
                fnmatch.fnmatchcase(os.path.normcase(parts[index]), os.path.normcase(word)) and
                match(index + 1, current + 1))

    return match(0, 0)


def _read_package(directory: Path, role: str, limits: ArchiveReadLimits) -> Sample:
    """校验发布清单及所有成员字节；不重新证明点云的物理含义。"""
    for parent in (directory, *directory.parents):
        _not_link(parent)
    manifest = directory / "manifest.json"
    _not_link(manifest)
    # 清单不是点云；异常大小不允许无界占用内存。
    raw = _read_regular(manifest, limits.manifest_bytes)
    meta = bounded_json(raw, limits, manifest=True)
    if (not isinstance(meta, dict) or meta.get("schema") != "cloud-archive-v1" or
            meta.get("authoritative") is not False):
        raise ValueError("不支持或不合法的归档声明")
    if json.dumps(meta, ensure_ascii=False, sort_keys=True, separators=(",", ":"),
                  allow_nan=False).encode("utf-8") != raw:
        raise ValueError("manifest 不是规范 JSON")
    if not all(_identifier(meta.get(key)) for key in ("channel", "measurement_id")):
        raise ValueError("归档身份无效")
    if meta.get("passage_id") is not None and not _identifier(meta["passage_id"]):
        raise ValueError("归档车次无效")
    ts = meta.get("acquired_at")
    if type(ts) not in (int, float) or not math.isfinite(ts) or ts <= 0:
        raise ValueError("采集时间无效")
    frame, detection = meta.get("frame", {}), meta.get("detection_frame", {})
    if (not isinstance(frame, dict) or not isinstance(detection, dict) or
            frame.get("ts") != ts or
            any(key not in frame or frame[key] != detection.get(key) for key in ("seq", "ts", "gen")) or
            any(type(frame.get(key)) is not bool for key in ("complete", "measurable"))):
        raise ValueError("归档帧身份或质量标记不完整")
    if (not re.fullmatch(r"[a-f0-9]{64}", str(meta.get("configuration_hash", ""))) or
            not isinstance(meta.get("algorithm_version"), str) or not meta["algorithm_version"].strip() or
            meta.get("source_mode") not in ("live", "demo", "replay")):
        raise ValueError("归档版本、来源或配置摘要无效")
    entries = meta.get("files")
    required = {item + ".xyz" for item in ROLES} | {
        "vehicle.indices.npy", "carriage.indices.npy", "detection.xyz.meta.json", "detection.xyz.reason.txt"}
    if not isinstance(entries, dict) or not required.issubset(entries):
        raise ValueError("归档角色或配套文件不完整")
    check_budget(meta, len(raw), limits)
    for name, record in entries.items():
        if (not re.fullmatch(r"[a-z0-9][a-z0-9_.-]{0,95}", name) or name == "manifest.json" or
                not isinstance(record, dict) or type(record.get("bytes")) is not int or record["bytes"] < 0 or
                not re.fullmatch(r"[a-f0-9]{64}", str(record.get("sha256", "")))):
            raise ValueError("归档文件名或摘要记录无效")
        path = directory / name
        _not_link(path)
        if not path.is_file() or path.stat().st_size != record["bytes"]:
            raise ValueError("成员缺失或长度不符: " + name)
        content = _read_regular(path, limits.file_bytes, record["bytes"])
        header = content[:4]
        if hashlib.sha256(content).hexdigest() != record["sha256"]:
            raise ValueError("成员摘要不符: " + name)
        if name.endswith(".xyz"):
            count = record.get("points")
            if (type(count) is not int or count < 0 or len(header) != 4 or
                    struct.unpack("<I", header)[0] != count or record["bytes"] != 4 + 14 * count):
                raise ValueError("XYZ 点数或记录长度无效: " + name)
    for item in ROLES:
        record = entries[item + ".xyz"]
        if (record.get("role") != item or record.get("origin") != "observed" or
                record.get("coordinates") != ("detection" if item == "detection" else "source")):
            raise ValueError("云图角色或坐标系无效: " + item)
    provenance = meta.get("provenance", {})
    if _read_regular(manifest, limits.manifest_bytes) != raw:
        raise ValueError("归档清单在发现期间发生变化")
    segments = provenance.get("segments", []) if isinstance(provenance, dict) else []
    return Sample(str(directory / (role + ".xyz")), role, meta["channel"], meta["measurement_id"],
                  float(ts), hashlib.sha256(raw).hexdigest(), len(segments))


def discover_samples(directory: str, role: str = "detection", limit: int = 0,
                     exclude_raw: bool = False, pattern: str = "**/*.xyz",
                     archive_channel: Optional[str] = None,
                     archive_limits: Optional[ArchiveReadLimits] = None) -> Discovery:
    """自动发现完整包中的一个角色；旧样本保持字典序，新包按采集时间排序。"""
    if role not in ROLES:
        raise ValueError("不支持的云图角色: " + role)
    if limit < 0:
        raise ValueError("样本数量限制不能为负数")
    root = Path(os.path.abspath(directory))
    archive_limits = archive_limits or ArchiveReadLimits()
    result = Discovery()
    if not root.is_dir():
        result.errors.append(str(root) + ": 目录不存在")
        return result
    for parent in (root, *root.parents):
        try:
            _not_link(parent)
        except (OSError, ValueError) as exc:
            result.errors.append(str(root) + ": " + str(exc))
            return result
    if os.path.isabs(pattern) or ".." in Path(pattern).parts:
        raise ValueError("样本匹配模式必须位于输入目录内")
    identities = {}
    conflicts = set()
    legacy = []
    for base, dirs, files in os.walk(str(root), followlinks=False,
                                      onerror=lambda exc: result.errors.append(str(exc))):
        folder = Path(base)
        dirs.sort()
        if exclude_raw and any(os.path.normcase(part) == "raw" for part in folder.parts):
            dirs[:] = []
            continue
        for name in list(dirs):
            try:
                _not_link(folder / name)
            except (OSError, ValueError) as exc:
                dirs.remove(name)
                result.errors.append(str(folder / name) + ": " + str(exc))
        if "manifest.json" in files:
            dirs[:] = []
            try:
                sample = _read_package(folder, role, archive_limits)
                key = (sample.channel, sample.measurement_id)
                old = identities.get(key)
                if old is not None and old.manifest_sha256 != sample.manifest_sha256:
                    conflicts.add(key)
                    result.errors.append(str(folder) + ": 同测量身份存在冲突归档，全部排除")
                elif old is None or (not _matches(Path(old.path), root, pattern) and
                                     _matches(Path(sample.path), root, pattern)):
                    identities[key] = sample
            except (OSError, ValueError, TypeError, KeyError, OverflowError, RecursionError) as exc:
                result.errors.append(str(folder) + ": 归档不可用: " + str(exc))
            continue
        xyz = [name for name in files if name.lower().endswith(".xyz")]
        # Windows 同一目录可用不同大小写进入；保护不能取决于调用者拼写。
        in_archive = any(part.casefold() == ARCHIVE_DIR for part in folder.parts)
        names = {name.casefold() for name in files}
        markers = {"vehicle.indices.npy", "carriage.indices.npy", "detection.input-indices.npy",
                   "detection.lineage.npy", "detection.xyz.meta.json", "detection.xyz.reason.txt"}
        looks_package = (len(names & {item + ".xyz" for item in ROLES}) >= 2 or bool(names & markers) or
                         any(re.fullmatch(r"(?:volume-)?source-\d+\.(?:xyz|input-indices.npy)", name)
                             for name in names))
        if in_archive or looks_package:
            if xyz or looks_package:
                result.errors.append(str(folder) + ": 未发布 manifest，跳过包内云图")
            if looks_package:
                dirs[:] = []
            continue
        for name in xyz:
            path = folder / name
            try:
                _not_link(path)
                legacy.append(Sample(str(path)))
            except (OSError, ValueError) as exc:
                result.errors.append(str(path) + ": " + str(exc))
    archives = sorted((sample for key, sample in identities.items() if key not in conflicts and
                       (archive_channel is None or sample.channel == archive_channel)),
                      key=lambda sample: (sample.acquired_at, sample.channel, sample.measurement_id, sample.path))
    samples = sorted(legacy, key=lambda sample: sample.path) + archives
    samples = [sample for sample in samples if _matches(Path(sample.path), root, pattern)]
    result.samples = samples[:limit] if limit else samples
    return result
