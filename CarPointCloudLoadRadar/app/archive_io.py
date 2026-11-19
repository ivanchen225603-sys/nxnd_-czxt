"""归档发现和加载共用的标准库预算与有限读取，不依赖 NumPy。"""
from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import stat
from typing import Optional

@dataclass(frozen=True)
class ArchiveReadLimits:
    manifest_bytes: int = 8 * 1024 * 1024
    file_bytes: int = 128 * 1024 * 1024
    total_bytes: int = 256 * 1024 * 1024
    json_bytes: int = 2 * 1024 * 1024
    json_depth: int = 64
    json_nodes: int = 200_000
    json_numeric_chars: int = 128
    members: int = 2048
    segments: int = 256
    points: int = 8_000_000
    index_values: int = 16_000_000
    working_bytes: int = 1024 * 1024 * 1024

    def __post_init__(self):
        if any(type(value) is not int or value <= 0 for value in vars(self).values()):
            raise ValueError("归档读取预算必须为正整数")


def _stamp(info):
    return (info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns)


def _path_guard(path: Path):
    for part in (path, *path.parents):
        info = part.lstat()
        if stat.S_ISLNK(info.st_mode) or getattr(info, "st_file_attributes", 0) & 0x400:
            raise ValueError("归档读取拒绝符号链接或重解析点")
        if part != path and not stat.S_ISDIR(info.st_mode):
            raise ValueError("归档祖先不是目录")
    return path.lstat()


def _read_regular(path: Path, limit: int, expected: Optional[int] = None) -> bytes:
    """前后核验文件身份，只使用本次打开的字节；不是跨平台目录句柄沙箱。"""
    before = _path_guard(path)
    if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1:
        raise ValueError("归档成员必须为普通单链接文件")
    if before.st_size > limit or expected is not None and before.st_size != expected:
        raise ValueError("归档成员大小不符或超过读取预算")
    flags = os.O_RDONLY | getattr(os, "O_BINARY", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(str(path), flags)
    with os.fdopen(descriptor, "rb") as stream:
        opened = os.fstat(stream.fileno())
        if _stamp(opened) != _stamp(before) or opened.st_nlink != 1:
            raise ValueError("归档成员在打开时发生替换")
        data = stream.read(before.st_size + 1)
        after = os.fstat(stream.fileno())
    final = _path_guard(path)
    if (len(data) != before.st_size or _stamp(after) != _stamp(before)
            or _stamp(final) != _stamp(before) or after.st_nlink != 1 or final.st_nlink != 1):
        raise ValueError("归档成员在读取期间发生变化")
    return data


def bounded_json(data: bytes, limits: ArchiveReadLimits, *, manifest=False) -> dict:
    """在 JSON 解码前检查字节、结构深度和节点预算，拒绝非规范对象。"""
    cap = limits.manifest_bytes if manifest else limits.json_bytes
    if len(data) > cap or len(data) * 128 > limits.working_bytes:
        raise ValueError("归档 JSON 字节或派生工作内存超过预算")
    text = data.decode("utf-8", "strict")
    if b"\x00" in data or data.startswith(b"\xef\xbb\xbf"):
        raise ValueError("归档 JSON 必须为无 BOM/NUL 的 UTF-8")
    depth, nodes, numeric, quoted, escaped = 0, 0, 0, False, False
    for char in data:
        numeric = numeric + 1 if not quoted and char in b"0123456789+-.eE" else 0
        if numeric > limits.json_numeric_chars:
            raise ValueError("归档 JSON 数字长度超过预算")
        if quoted:
            if escaped:
                escaped = False
            elif char == 92:
                escaped = True
            elif char == 34:
                quoted = False
        elif char == 34:
            quoted = True
        elif char in (91, 123):
            depth += 1
            nodes += 1
        elif char in (93, 125):
            depth -= 1
        elif char in (44, 58):
            nodes += 1
        if depth > limits.json_depth or nodes > limits.json_nodes:
            raise ValueError("归档 JSON 深度或节点超过预算")
    # 不传 bytes，禁止解析器自动探测 UTF-16/32 绕过以上单字节 token 预算。
    value = json.loads(text)
    if not isinstance(value, dict):
        raise ValueError("归档 JSON 必须为对象")
    if json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8") != data:
        raise ValueError("归档 JSON 必须为唯一规范形式")
    return value


RUNTIME_ROLES = {"effective-configuration.json": "effective_configuration",
                 "volume-call.json": "volume_call", "gate-summary.json": "gate_summary",
                 "baseline.npz": "original_baseline_asset",
                 # v2：进场帧；v3：再加离场帧。见 app/enter_volume.py。
                 "enter-frames.f32le": "enter_frames",
                 "exit-frames.f32le": "exit_frames"}
# 4M 点 x 12 字节；与 enter_volume.ENTER_MAX_POINTS 同口径。
ENTER_FRAMES_MAX_BYTES = 4_000_000 * 12


def runtime_members(meta: dict) -> dict:
    """扩展只允许固定成员，不接受路径、别名或剥掉声明后降为旧包。"""
    entries = meta["files"]
    if "measurement_runtime" not in meta:
        if set(entries) & set(RUNTIME_ROLES):
            raise ValueError("运行扩展成员缺少声明")
        return {}
    extension = meta["measurement_runtime"]
    schema = extension.get("schema") if type(extension) is dict else None
    v2 = schema == "measurement-runtime-v2"
    v3 = schema == "measurement-runtime-v3"
    keys = {"schema", "baseline_asset"}
    if v2 or v3:
        keys |= {"enter_frames"}
    if v3:
        keys |= {"exit_frames"}
    if (type(extension) is not dict or set(extension) != keys or
            schema not in ("measurement-runtime-v1", "measurement-runtime-v2", "measurement-runtime-v3") or
            extension["baseline_asset"] not in (None, "baseline.npz") or
            ((v2 or v3) and extension["enter_frames"] not in (None, "enter-frames.f32le")) or
            (v3 and extension["exit_frames"] not in (None, "exit-frames.f32le")) or
            "provenance" not in meta or "volume_support" not in meta):
        raise ValueError("运行扩展版本、固定成员或真实输入窗口无效")
    expected = dict(RUNTIME_ROLES)
    if extension["baseline_asset"] is None:
        expected.pop("baseline.npz")
    if not (v2 or v3) or extension["enter_frames"] is None:
        expected.pop("enter-frames.f32le")
    elif (type(entries.get("enter-frames.f32le", {}).get("bytes")) is not int or
          not 0 < entries["enter-frames.f32le"]["bytes"] <= ENTER_FRAMES_MAX_BYTES):
        raise ValueError("进场帧成员字节超过预算")
    if not v3 or extension["exit_frames"] is None:
        expected.pop("exit-frames.f32le")
    elif (type(entries.get("exit-frames.f32le", {}).get("bytes")) is not int or
          not 0 < entries["exit-frames.f32le"]["bytes"] <= ENTER_FRAMES_MAX_BYTES):
        raise ValueError("离场帧成员字节超过预算")
    if set(entries) & set(RUNTIME_ROLES) != set(expected) or any(
            not isinstance(entries[name], dict) or entries[name].get("role") != role for name, role in expected.items()):
        raise ValueError("运行扩展成员缺失、角色错误或基线资产声明不符")
    if "baseline.npz" in expected and (type(entries["baseline.npz"].get("bytes")) is not int or
                                      not 0 < entries["baseline.npz"]["bytes"] <= 32 * 1024 * 1024):
        raise ValueError("原基线压缩字节超过预算")
    return expected


def check_budget(meta: dict, manifest_size: int, limits: ArchiveReadLimits):
    """全清单先检查，禁止先读几个巨大成员再发现包整体超预算。"""
    entries = meta["files"]
    if not isinstance(entries, dict) or len(entries) > limits.members:
        raise ValueError("归档成员数超过预算")
    runtime = runtime_members(meta)
    total, points, json_bytes = manifest_size, 0, manifest_size
    for name, record in entries.items():
        if (not re.fullmatch(r"[a-z0-9][a-z0-9_.-]{0,95}", name) or name == "manifest.json"
                or not isinstance(record, dict) or type(record.get("bytes")) is not int or record["bytes"] < 0):
            raise ValueError("归档成员文件名或字节数无效")
        size = record["bytes"]
        if size > limits.file_bytes:
            raise ValueError("归档成员大小超过预算")
        total += size
        if name.endswith(".json"):
            if size > limits.json_bytes:
                raise ValueError("归档 JSON 成员大小超过预算")
            json_bytes += size
        if name.endswith(".xyz"):
            count = record.get("points")
            if type(count) is not int or count < 0 or size != 4 + 14 * count:
                raise ValueError("XYZ 点数或声明大小无效")
            points += count
    for key in ("provenance", "volume_support"):
        if key in meta:
            segments = meta[key]["segments"]
            if not isinstance(segments, list) or not 0 < len(segments) <= limits.segments:
                raise ValueError("归档来源窗口大小无效或超过预算")
    # 不信任清单自报解压量；新基线预留现有NPZ上限的最坏展开/转换预算。
    baseline_work = 3 * 128 * 1024 * 1024 + 48 * 2_000_000 if "baseline.npz" in runtime else 0
    runtime_work = points * 48 if runtime else 0
    if "enter-frames.f32le" in runtime:
        # 复算要重新配准：每帧转 float64 + 平移副本 + 高度图，按原字节的 16 倍保守预留。
        runtime_work += entries["enter-frames.f32le"]["bytes"] * 16
    if (total > limits.total_bytes or points > limits.points
            or total * 2 + points * 128 + json_bytes * 128 + baseline_work + runtime_work > limits.working_bytes):
        raise ValueError("归档总字节、点数或派生工作内存超过预算")
