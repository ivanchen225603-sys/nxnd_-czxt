"""只读容量诊断；文件表观大小、数据库逻辑字节与内存估值不能相加或授权删除。"""
from __future__ import annotations

from dataclasses import dataclass
import copy
import os
from pathlib import Path
import re
import stat
import threading
import time
from typing import Iterable, Optional

from .archive_io import _path_guard


@dataclass(frozen=True)
class CapacityScanLimits:
    roots: int = 32
    entries: int = 100_000
    depth: int = 64
    errors: int = 100
    seconds: float = 5.

    def __post_init__(self):
        import math
        if (any(type(value) is not int or value <= 0 for value in (self.roots, self.entries, self.depth, self.errors))
                or type(self.seconds) not in (int, float) or not math.isfinite(self.seconds) or self.seconds <= 0):
            raise ValueError("容量扫描预算必须为有限正值")


def _file_stamp(info):
    return (info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns, info.st_nlink)


def _category(path: Path) -> str:
    name = path.name.casefold()
    if name.endswith((".sqlite3-wal", ".sqlite-wal", ".db-wal")):
        return "database_wal"
    if name.endswith((".sqlite3-shm", ".sqlite-shm", ".db-shm")):
        return "database_shm"
    if name.endswith((".sqlite3", ".sqlite", ".db")):
        return "database"
    if any(part.startswith((".pending-", ".baseline-")) for part in path.parts):
        return "temporary_unclassified"
    if "archives-v1" in [part.casefold() for part in path.parts]:
        return "archive_projection_unverified"
    if re.fullmatch(r"[a-f0-9]{64}\.npz", name):
        return "baseline_cache_unverified"
    if name.endswith(".xyz"):
        return "legacy_xyz"
    if name.endswith((".xyz.meta.json", ".xyz.reason.txt")):
        return "legacy_sidecar"
    return "other"


def scan_capacity_files(roots: Iterable[str], *, limits: Optional[CapacityScanLimits] = None) -> dict:
    """仅 stat 显式目录，不读取云图/数据库内容；时限是协作式而非系统调用硬超时。"""
    limits = limits or CapacityScanLimits()
    if isinstance(roots, (str, bytes)):
        raise ValueError("容量盘点需要显式目录列表")
    normalized = {}
    for index, item in enumerate(roots):
        if index >= limits.roots:
            raise ValueError("容量扫描根目录数超过预算")
        if not isinstance(item, (str, os.PathLike)) or not str(item).strip():
            raise ValueError("容量扫描根目录无效")
        path = Path(os.path.abspath(item))
        normalized.setdefault(os.path.normcase(str(path)), path)
    if not normalized:
        raise ValueError("必须显式提供至少一个容量扫描目录")
    start_wall, start = time.time(), time.monotonic()
    result = dict(schema="file-capacity-v1", status="complete", roots=[str(p) for p in normalized.values()],
                  sampled_at=start_wall, entries=0, directories=0, files=0, apparent_bytes=0,
                  hardlink_aliases=0, overlapping_directories=0, categories={}, errors=[], error_count=0,
                  integrity="not_checked", database_references="unknown", runtime_protection="unknown",
                  consistency="sampled_non_atomic", time_bound="cooperative_not_hard",
                  size_semantics="按文件身份去重的表观st_size，不是实际分配块或可回收空间")
    files, directories = {}, set()
    stopped = False

    def error(path, code):
        result["error_count"] += 1
        if len(result["errors"]) < limits.errors:
            result["errors"].append(dict(path=str(path), code=code))

    def budget(path):
        nonlocal stopped
        if stopped:
            return False
        if result["entries"] >= limits.entries or time.monotonic() - start >= limits.seconds:
            error(path, "scan_budget_exceeded")
            stopped = True
            return False
        return True

    def visit(directory, depth):
        if not budget(directory):
            return
        if depth > limits.depth:
            error(directory, "depth_budget_exceeded")
            return
        try:
            before = _path_guard(directory)
            if not stat.S_ISDIR(before.st_mode):
                error(directory, "root_not_directory")
                return
            identity = (before.st_dev, before.st_ino) if before.st_ino else None
            if identity is None:
                error(directory, "directory_identity_unavailable")
                return
            if identity in directories:
                result["overlapping_directories"] += 1
                return
            directories.add(identity)
            result["directories"] += 1
            with os.scandir(directory) as entries:
                for entry in entries:
                    path = directory / entry.name
                    if not budget(path):
                        break
                    result["entries"] += 1
                    try:
                        info = _path_guard(path)
                        if stat.S_ISDIR(info.st_mode):
                            visit(path, depth + 1)
                            continue
                        if not stat.S_ISREG(info.st_mode):
                            error(path, "not_regular_file")
                            continue
                        after = _path_guard(path)
                        if _file_stamp(info) != _file_stamp(after):
                            error(path, "file_changed_during_stat")
                            continue
                        identity = (info.st_dev, info.st_ino) if info.st_ino else None
                        if identity is None:
                            error(path, "file_identity_unavailable")
                            continue
                        if identity in files:
                            if files[identity][0] != _file_stamp(info):
                                error(path, "file_changed_between_aliases")
                            if files[identity][1] != _category(path):
                                error(path, "hardlink_category_ambiguous")
                            result["hardlink_aliases"] += 1
                            continue
                        files[identity] = (_file_stamp(info), _category(path))
                        result["files"] += 1
                        result["apparent_bytes"] += info.st_size
                        category = result["categories"].setdefault(_category(path), dict(files=0, apparent_bytes=0))
                        category["files"] += 1
                        category["apparent_bytes"] += info.st_size
                    except (OSError, ValueError) as exc:
                        error(path, "path_unavailable:" + type(exc).__name__)
            if _file_stamp(_path_guard(directory)) != _file_stamp(before):
                error(directory, "directory_changed_during_scan")
        except (OSError, ValueError) as exc:
            error(directory, "directory_unavailable:" + type(exc).__name__)

    for path in normalized.values():
        visit(path, 0)
    if result["error_count"]:
        result["status"] = "partial" if result["directories"] else "unavailable"
    result["finished_at"] = time.time()
    result["seconds"] = time.monotonic() - start
    result["errors_truncated"] = result["error_count"] > len(result["errors"])
    return result


# 每一分支先按 rowid 截取，再只返回标量长度/有限状态。引用以唯一键核对，
# 不能把父表样本之外的正常引用误称悬挂；不解析 manifest 或业务 JSON。
_ARCHIVE_QUERIES = (
    ("cloud_archives", "CASE WHEN typeof(status)='text' AND status COLLATE BINARY IN ('pending','complete') "
     "THEN status ELSE 'invalid' END", "length(manifest)", "0", "'not_applicable'"),
    ("cloud_archive_files", "'stored'", "length(content)", "0",
     "CASE WHEN EXISTS(SELECT 1 FROM cloud_archives p WHERE p.measurement_id=s.measurement_id) "
     "THEN 'present' ELSE 'missing_parent' END"),
    ("empty_observations", "'reference'", "0", "0",
     "CASE WHEN EXISTS(SELECT 1 FROM cloud_archives p WHERE p.measurement_id=s.measurement_id "
     "AND p.channel=s.channel AND p.passage_id=s.passage_id AND p.acquired_at=s.acquired_at) "
     "THEN 'matching_metadata' ELSE 'missing_or_mismatched' END"),
    ("empty_observation_latest", "'reference'", "0", "0",
     "CASE WHEN EXISTS(SELECT 1 FROM cloud_archives p WHERE p.measurement_id=s.measurement_id "
     "AND p.channel=s.channel AND p.seq=s.archive_seq AND p.acquired_at=s.acquired_at) "
     "THEN 'matching_metadata' ELSE 'missing_or_mismatched' END"),
)
_OUTBOX_QUERIES = (
    ("outbox", "CASE WHEN typeof(status)='text' AND status COLLATE BINARY IN ('pending','sent','blocked') "
     "THEN status ELSE 'invalid' END", "length(CAST(payload_json AS BLOB))", "coalesce(length(asset),0)",
     "coalesce((SELECT CASE p.status WHEN 'pending' THEN 'projection_pending' "
     "WHEN 'complete' THEN 'projection_complete' ELSE 'projection_unknown' END "
     "FROM cloud_archives p WHERE p.measurement_id=s.measurement_id), 'no_archive')"),
    ("command_results", "'receipt'", "length(CAST(command_json AS BLOB))", "0",
     "CASE WHEN EXISTS(SELECT 1 FROM outbox p WHERE p.measurement_id=s.measurement_id) "
     "THEN 'present' ELSE 'missing_parent' END"),
) + _ARCHIVE_QUERIES
_PASSAGE_QUERIES = (
    ("lanes", "CASE ready WHEN 0 THEN 'not_ready' WHEN 1 THEN 'ready' ELSE 'unknown' END", "0", "0", "'not_checked'"),
    ("commands", "CASE WHEN typeof(status)='text' AND status COLLATE BINARY IN ('pending','running','succeeded','failed') "
     "THEN status ELSE 'invalid' END", "length(CAST(body AS BLOB))", "0", "'cross_database_unknown'"),
)
_QUERIES = dict(archive=_ARCHIVE_QUERIES, outbox=_OUTBOX_QUERIES, passages=_PASSAGE_QUERIES)
_STATES = dict(cloud_archives={"pending", "complete"}, cloud_archive_files={"stored"},
               empty_observations={"reference"}, empty_observation_latest={"reference"},
               outbox={"pending", "sent", "blocked"}, command_results={"receipt"},
               lanes={"not_ready", "ready"}, commands={"pending", "running", "succeeded", "failed"})


def _database_sample(connection, lock, closed, owner_kind, row_budget, lock_timeout):
    start = time.time()
    report = dict(schema="database-capacity-v1", owner_kind=owner_kind, status="unavailable",
                  started_at=start, integrity="not_checked", reference_scope="database_metadata_only",
                  consistency="single_select_snapshot", time_bound="not_hard",
                  byte_semantics="样本选定TEXT的UTF-8字节及BLOB长度；非数据库文件大小，不可相加推算回收空间")
    if not lock.acquire(timeout=lock_timeout):
        report.update(reason="owner_lock_busy", finished_at=time.time())
        return report
    try:
        if closed():
            report["reason"] = "closed"
            return report
        if connection.in_transaction:
            report["reason"] = "transaction_active"
            return report
        branches = _QUERIES[owner_kind]
        per_table = row_budget // len(branches)
        sql = " UNION ALL ".join(
            "SELECT * FROM (SELECT '%s',%s,%s,%s,%s "
            "FROM %s s NOT INDEXED ORDER BY rowid LIMIT ?)" % (table, state, a, b, ref, table)
            for table, state, a, b, ref in branches)
        # CAST(TEXT AS BLOB) 使用数据库编码，不可将预存UTF-16库误报为UTF-8字节。
        # 在同一只读语句内取得编码，不改PRAGMA或连接级自定义函数。
        sql = "SELECT '__encoding__',encoding,0,0,'not_applicable' FROM pragma_encoding UNION ALL " + sql
        cursor = connection.cursor()
        try:
            # 仅改本次游标，不覆盖连接现有 row_factory、trace/progress/authorizer。
            cursor.row_factory = None
            rows = cursor.execute(sql, [per_table + 1] * len(branches)).fetchall()
        finally:
            cursor.close()
    except Exception as exc:
        # 不包含 SQL、私有路径、凭据或业务内容。
        report["reason"] = "query_failed:" + type(exc).__name__
        return report
    finally:
        report["finished_at"] = time.time()
        lock.release()
    if not rows or rows[0] != ("__encoding__", "UTF-8", 0, 0, "not_applicable"):
        report["reason"] = "unsupported_text_encoding"
        return report
    rows = rows[1:]
    for table, state, a, b, ref in rows:
        if (type(table) is not str or table not in _STATES or type(state) is not str
                or state not in _STATES[table] or type(a) is not int or a < 0
                or type(b) is not int or b < 0 or type(ref) is not str):
            report["reason"] = "invalid_sample_metadata"
            return report
    tables = {table: dict(observed_rows=0, logical_bytes=0, auxiliary_bytes=0, states={}, references={},
                         ack_projection_pairs={}, truncated=False) for table, *_ in branches}
    for table, state, a, b, ref in rows:
        entry = tables[table]
        if entry["observed_rows"] >= per_table:
            entry["truncated"] = True
            continue
        entry["observed_rows"] += 1
        entry["logical_bytes"] += a
        entry["auxiliary_bytes"] += b
        entry["states"][state] = entry["states"].get(state, 0) + 1
        entry["references"][ref] = entry["references"].get(ref, 0) + 1
        if table == "outbox":
            pair = str(state) + "/" + ref
            entry["ack_projection_pairs"][pair] = entry["ack_projection_pairs"].get(pair, 0) + 1
    for entry in tables.values():
        entry["extent"] = "observed_lower_bound" if entry["truncated"] else "all_rows_at_sample"
    report.update(status="partial" if any(t["truncated"] for t in tables.values()) else "complete",
                  tables=tables, per_table_budget=per_table, returned_rows=len(rows))
    return report


class CapacityProbe:
    """显式维护调用才启动的单飞缓存；SQL 不在调用线程或通道控制锁内运行。

    owner 必须在同一数据库锁内设置 closed 并关闭连接。查询只取该锁，绝不
    等待导出锁；close 等在途查询归还连接，排队查询见 closed 后不执行 SQL。
    LIMIT 不提供硬超时，慢查询期间 close 也可能等待；不强行中断共享连接。
    """
    def __init__(self, connection, lock, closed, owner_kind, *, row_budget=6000, lock_timeout=.05):
        import math
        if owner_kind not in _QUERIES:
            raise ValueError("未知容量数据库所有者")
        if (type(row_budget) is not int or not len(_QUERIES[owner_kind]) <= row_budget <= 100000
                or type(lock_timeout) not in (int, float) or not math.isfinite(lock_timeout)
                or not 0 <= lock_timeout <= 1):
            raise ValueError("数据库容量预算无效")
        self._args = (connection, lock, closed, owner_kind, row_budget, lock_timeout)
        self._lock = threading.Lock()
        self._running = False
        self._cached = None

    def snapshot(self, *, refresh=False):
        with self._lock:
            if self._args[2]():
                return dict(status="closed", query_running=self._running, cached=None)
            if not self._running and (self._cached is None or refresh):
                self._running = True
                try:
                    threading.Thread(target=self._run, daemon=True, name="capacity-" + self._args[3]).start()
                except Exception:
                    self._running = False
                    raise
            return dict(status="pending" if self._cached is None else "cached",
                        query_running=self._running, cached=copy.deepcopy(self._cached))

    def _run(self):
        try:
            result = _database_sample(*self._args)
        except Exception as exc:
            result = dict(status="unavailable", reason="sample_failed:" + type(exc).__name__)
        with self._lock:
            self._cached = result
            self._running = False
