"""按车道持久化原测量任务；等待车牌不创建占位测量，不改写已冻结字节。"""
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass, field as dataclass_field, replace
import hashlib
import errno
import json
import math
from pathlib import Path
import sqlite3
import shutil
import threading
import time
from datetime import datetime, timezone, timedelta
from typing import Optional, TYPE_CHECKING

from .load_radar_transport import (_identifier, _private_permissions, _reject_constant,
                                   _reject_link, _safe_storage, _unique_object)

if TYPE_CHECKING:
    from .load_radar_bridge import ResultReservation


MAX_PREPARED_TASK_BYTES = 384 * 1024 * 1024
_INPUT_SIZE_SQL = ("length(identity_json)+coalesce(length(command_json),0)+"
                   "length(start_json)+length(input_bytes)")


class CompletionBusy(RuntimeError):
    """完成保留池不足；任务仍只保留原件，不开始或重复计算。"""


@dataclass(frozen=True)
class TaskRow:
    identity: ResultReservation
    start_json: bytes
    input_bytes: bytes
    preparation_bytes: Optional[bytes]
    prepared_bytes: Optional[bytes]
    state: str
    attempts: int
    next_at: float
    error: str
    compacted: bool = False
    input_sha256: str = dataclass_field(default="", compare=False)


def _blob(value, name):
    if not isinstance(value, bytes) or not value:
        raise ValueError(name + "必须为非空不可变字节")
    return value


def _object(value, name):
    try:
        result = json.loads(value, object_pairs_hook=_unique_object, parse_constant=_reject_constant)
    except (ValueError, TypeError, UnicodeError, RecursionError):
        raise ValueError(name + "必须为有效JSON对象") from None
    if not isinstance(result, dict):
        raise ValueError(name + "必须为JSON对象")
    return result


def _digest(*parts):
    value = hashlib.sha256()
    for part in parts:
        # 带长度编码，避免不同字段边界的串接歧义。
        value.update(len(part).to_bytes(8, "little"))
        value.update(part)
    return value.hexdigest()


def _clock(value):
    value = time.time() if value is None else value
    if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
        raise ValueError("任务时刻必须为有限非负墙上时间")
    return float(value)


class PendingTaskStore:
    """独立SQLite/锁，容量按通道计算；进程唯一性由外层bridge lease保证。

    原输入池和完成保留池独立计算，不包括SQLite页/WAL开销；已提交任务仍保留，
    本组件不清理历史。每通道只有一个执行者，due不建立跨线程/跨进程工作租约。
    """
    def __init__(self, storage_dir, channels, max_bytes_per_channel=512 * 1024 * 1024,
                 max_jobs_per_channel=32, max_completion_bytes_per_channel=512 * 1024 * 1024):
        for value in (max_bytes_per_channel, max_jobs_per_channel, max_completion_bytes_per_channel):
            if type(value) is not int or value <= 0:
                raise ValueError("任务容量必须为正整数")
        if isinstance(channels, str):
            raise ValueError("通道必须为非空集合")
        names = tuple(_identifier(channel) for channel in channels)
        if not names or len(set(names)) != len(names):
            raise ValueError("通道为空或重复")
        self.max_bytes_per_channel = max_bytes_per_channel
        self.max_jobs_per_channel = max_jobs_per_channel
        self.max_completion_bytes_per_channel = max_completion_bytes_per_channel
        self.storage_dir = _safe_storage(storage_dir)
        self.pending_dir = _safe_storage(self.storage_dir / "pending")
        self.pending_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
        _safe_storage(self.pending_dir)
        _private_permissions(self.pending_dir, 0o700)
        self._locks = {channel: threading.RLock() for channel in names}
        self._connections = {}
        self._closed = False
        self._cleanup_ready = False
        self._close_lock = threading.Lock()
        try:
            for channel in names:
                # 摘要文件名避免Windows大小写路径碰撞及通道名路径穿越。
                path = self.pending_dir / ("channel-" + hashlib.sha256(channel.encode()).hexdigest() + ".sqlite3")
                for suffix in ("", "-wal", "-shm"):
                    _reject_link(Path(str(path) + suffix))
                connection = sqlite3.connect(str(path), timeout=10., isolation_level=None,
                                             check_same_thread=False)
                self._connections[channel] = connection
                connection.row_factory = sqlite3.Row
                if connection.execute("PRAGMA journal_mode=WAL").fetchone()[0].lower() != "wal":
                    raise RuntimeError("待计算任务库无法启用WAL")
                connection.execute("PRAGMA synchronous=FULL")
                connection.executescript("""
                    CREATE TABLE IF NOT EXISTS pending_tasks (
                        measurement_id TEXT PRIMARY KEY, passage_id TEXT NOT NULL,
                        command_request_id TEXT, identity_json BLOB NOT NULL, command_json BLOB,
                        start_json BLOB NOT NULL, input_bytes BLOB NOT NULL, input_sha256 TEXT NOT NULL,
                        preparation_bytes BLOB, preparation_sha256 TEXT,
                        prepared_bytes BLOB, prepared_sha256 TEXT,
                        state TEXT NOT NULL CHECK(state IN
                          ('awaiting_preparation','ready_to_compute','prepared','blocked','committed')),
                        attempts INTEGER NOT NULL DEFAULT 0, next_at REAL NOT NULL DEFAULT 0,
                        error TEXT NOT NULL DEFAULT '', stored_bytes INTEGER NOT NULL,
                        completion_reserved_bytes INTEGER NOT NULL DEFAULT 0,
                        created_at REAL NOT NULL);
                    CREATE UNIQUE INDEX IF NOT EXISTS pending_auto_passage
                        ON pending_tasks(passage_id) WHERE command_request_id IS NULL;
                    CREATE UNIQUE INDEX IF NOT EXISTS pending_command
                        ON pending_tasks(command_request_id) WHERE command_request_id IS NOT NULL;
                    CREATE INDEX IF NOT EXISTS pending_due ON pending_tasks(state,next_at,created_at);
                    CREATE TABLE IF NOT EXISTS recalculation_sources (
                        measurement_id TEXT PRIMARY KEY, exit_bytes BLOB NOT NULL,
                        plate TEXT NOT NULL DEFAULT '', day TEXT NOT NULL DEFAULT '',
                        eligible INTEGER NOT NULL DEFAULT 0, acknowledged_id TEXT);
                    CREATE TABLE IF NOT EXISTS recalculation_results (
                        id TEXT PRIMARY KEY, measurement_id TEXT NOT NULL,
                        payload BLOB NOT NULL, acknowledged INTEGER NOT NULL DEFAULT 0);
                """)
                if "completion_reserved_bytes" not in {
                        row[1] for row in connection.execute("PRAGMA table_info(pending_tasks)")}:
                    with self._transaction(channel) as migration:
                        migration.execute("ALTER TABLE pending_tasks ADD COLUMN completion_reserved_bytes "
                                          "INTEGER NOT NULL DEFAULT 0")
                        # 旧任务已冻结的证据同样占完成预留；不丢弃超额的历史原件。
                        migration.execute("UPDATE pending_tasks SET completion_reserved_bytes="
                            "length(preparation_bytes)+? WHERE preparation_bytes IS NOT NULL "
                            "AND state NOT IN ('blocked','committed')", (MAX_PREPARED_TASK_BYTES,))
                # 终态保留实际证据，释放不再执行的未来计算预留；也修复旧版本留下的预留。
                connection.execute("UPDATE pending_tasks SET completion_reserved_bytes="
                    "coalesce(length(preparation_bytes),0)+coalesce(length(prepared_bytes),0) "
                    "WHERE state='blocked'")
                for suffix in ("", "-wal", "-shm"):
                    _private_permissions(Path(str(path) + suffix), 0o600)
        except Exception:
            self.close()
            raise

    def ensure_cleanup_schema(self):
        """按车道独立迁移，失败可重试；不阻塞构造时原采集路径。"""
        if self._cleanup_ready:
            return
        for channel in self._connections:
            with self._transaction(channel) as connection:
                columns = {row[1] for row in connection.execute("PRAGMA table_info(pending_tasks)")}
                predicate = "state='committed'" + (" AND committed_at IS NULL" if "committed_at" in columns else "")
                size = connection.execute("SELECT coalesce(sum(length(input_bytes)+coalesce(length(preparation_bytes),0)+coalesce(length(prepared_bytes),0)),0) FROM pending_tasks WHERE " + predicate).fetchone()[0]
                database = connection.execute("PRAGMA database_list").fetchone()[2]
                if size and database and shutil.disk_usage(Path(database).parent).free < int(size) * 2 + 16 * 1024 * 1024:
                    raise OSError(errno.ENOSPC, "旧任务清理迁移缺少WAL安全空间，保留原任务并稍后重试")
                for column, declaration in (("committed_at", "REAL"), ("compacted_at", "REAL"), ("metadata_sha256", "TEXT")):
                    if column not in columns:
                        connection.execute("ALTER TABLE pending_tasks ADD COLUMN " + column + " " + declaration)
                connection.execute("UPDATE pending_tasks SET committed_at=? WHERE state='committed' AND committed_at IS NULL", (time.time(),))
        self._cleanup_ready = True

    def _lane(self, channel):
        if channel not in self._locks:
            raise ValueError("未配置的任务通道")
        if self._closed:
            raise RuntimeError("待计算任务库已关闭")
        return self._locks[channel], self._connections[channel]

    @contextmanager
    def _transaction(self, channel):
        lock, connection = self._lane(channel)
        with lock:
            self._lane(channel)
            connection.execute("BEGIN IMMEDIATE")
            try:
                yield connection
                connection.commit()
            except Exception:
                connection.rollback()
                raise

    @staticmethod
    def _row(row, channel):
        if row is None:
            return None
        # bridge会导入本组件；仅在实际取行时解析身份，避免模块循环导入。
        from .load_radar_bridge import ResultReservation
        identity_data = _object(row["identity_json"], "任务身份")
        if (identity_data.get("channel") != channel
                or identity_data.get("measurement_id") != row["measurement_id"]
                or identity_data.get("passage_id") != row["passage_id"]):
            raise ValueError("任务身份与索引不一致")
        command = None if row["command_json"] is None else bytes(row["command_json"])
        original = tuple(bytes(row[key]) for key in ("identity_json", "start_json", "input_bytes"))
        if "compacted_at" in row.keys() and row["compacted_at"] is not None:
            if (row["state"] != "committed" or original[2] or row["preparation_bytes"] is not None
                    or row["prepared_bytes"] is not None or
                    PendingTaskStore._metadata_digest(row) != row["metadata_sha256"]):
                raise ValueError("已清理任务凭据损坏，禁止重算或重采")
            return TaskRow(ResultReservation(**identity_data, command_json=command), original[1], b"",
                           None, None, row["state"], row["attempts"], row["next_at"], row["error"],
                           compacted=True, input_sha256=row["input_sha256"])
        if _digest(original[0], command or b"", original[1], original[2]) != row["input_sha256"]:
            raise ValueError("冻结任务输入摘要不符")
        frozen = []
        for field in ("preparation", "prepared"):
            value = row[field + "_bytes"]
            value = bytes(value) if value is not None else None
            if (None if value is None else _digest(value)) != row[field + "_sha256"]:
                raise ValueError("冻结任务结果摘要不符")
            frozen.append(value)
        return TaskRow(ResultReservation(**identity_data, command_json=command), original[1], original[2],
                       frozen[0], frozen[1], row["state"], row["attempts"], row["next_at"], row["error"],
                       input_sha256=row["input_sha256"])

    @staticmethod
    def _metadata_digest(row):
        return _digest(bytes(row["identity_json"]), bytes(row["command_json"] or b""),
                       bytes(row["start_json"]), row["input_sha256"].encode("ascii"),
                       (row["preparation_sha256"] or "").encode("ascii"),
                       (row["prepared_sha256"] or "").encode("ascii"))

    @staticmethod
    def _find(connection, measurement_id):
        return connection.execute("SELECT * FROM pending_tasks WHERE measurement_id=?",
                                  (_identifier(measurement_id),)).fetchone()

    def _required(self, connection, channel, measurement_id):
        row = self._row(self._find(connection, measurement_id), channel)
        if row is None:
            raise ValueError("待计算任务不存在")
        return row

    def _capacity(self, connection, extra_bytes, extra_jobs=0):
        count, size = connection.execute("SELECT count(*),coalesce(sum(" + _INPUT_SIZE_SQL + "),0) "
                                         "FROM pending_tasks WHERE state!='committed'").fetchone()
        if count + extra_jobs > self.max_jobs_per_channel or size + extra_bytes > self.max_bytes_per_channel:
            raise RuntimeError("本通道待计算任务容量已满，原任务保留")

    def put(self, identity, start_json: bytes, input_bytes: bytes) -> TaskRow:
        fields = {key: _identifier(getattr(identity, key)) for key in
                  ("measurement_id", "channel", "passage_id", "origin_token")}
        start_json, input_bytes = _blob(start_json, "到车请求"), _blob(input_bytes, "原始输入")
        if len(start_json) > 1024 * 1024:
            raise ValueError("到车请求超过1MiB")
        _object(start_json, "到车请求")
        command = identity.command_json
        request_id = None
        if command is not None:
            command = _blob(command, "原命令")
            body = _object(command, "原命令")
            request_id = _identifier(body.get("requestId"))
            if body.get("channelId") != fields["channel"] or body.get("passageId") != fields["passage_id"]:
                raise ValueError("命令与任务车次不一致")
        identity_json = json.dumps(fields, sort_keys=True, separators=(",", ":")).encode()
        size = sum(map(len, (identity_json, command or b"", start_json, input_bytes)))
        channel = fields["channel"]
        with self._transaction(channel) as connection:
            existing = self._row(self._find(connection, fields["measurement_id"]), channel)
            if existing is not None:
                if (existing.identity != identity or existing.start_json != start_json
                        or (existing.compacted and _digest(identity_json, command or b"", start_json, input_bytes) != existing.input_sha256)
                        or (not existing.compacted and existing.input_bytes != input_bytes)):
                    raise ValueError("同一任务ID的原始身份和字节已固定，禁止覆盖")
                return existing
            if request_id is None:
                old = connection.execute("SELECT * FROM pending_tasks WHERE passage_id=? "
                                         "AND command_request_id IS NULL", (fields["passage_id"],)).fetchone()
            else:
                old = connection.execute("SELECT * FROM pending_tasks WHERE command_request_id=?", (request_id,)).fetchone()
            existing = self._row(old, channel)
            if existing is not None:
                if request_id is not None and existing.identity.command_json != command:
                    raise ValueError("同一命令的原始内容已固定，禁止覆盖")
                return existing
            self._capacity(connection, size, 1)
            connection.execute("""INSERT INTO pending_tasks
                (measurement_id,passage_id,command_request_id,identity_json,command_json,start_json,
                 input_bytes,input_sha256,state,stored_bytes,created_at) VALUES (?,?,?,?,?,?,?,?,?,?,?)""",
                (fields["measurement_id"], fields["passage_id"], request_id, identity_json, command,
                 start_json, input_bytes, _digest(identity_json, command or b"", start_json, input_bytes),
                 "awaiting_preparation", size, time.time()))
            return self._required(connection, channel, fields["measurement_id"])

    def get(self, channel, measurement_id):
        lock, connection = self._lane(channel)
        with lock:
            self._lane(channel)
            return self._row(self._find(connection, measurement_id), channel)

    def due(self, channel, limit=1, now=None):
        if type(limit) is not int or not 1 <= limit <= self.max_jobs_per_channel:
            raise ValueError("任务读取批次超出通道上限")
        now = _clock(now)
        lock, connection = self._lane(channel)
        with lock:
            self._lane(channel)
            rows = connection.execute("SELECT * FROM pending_tasks WHERE state NOT IN ('blocked','committed') "
                "AND next_at<=? ORDER BY next_at,created_at,measurement_id LIMIT ?", (now, limit)).fetchall()
            return [self._row(row, channel) for row in rows]

    def next_due_at(self, channel):
        """未终局任务里最早的到期墙上时间；没有时为 None。只供后台线程决定何时醒来。"""
        lock, connection = self._lane(channel)
        with lock:
            row = connection.execute("SELECT MIN(next_at) FROM pending_tasks "
                "WHERE state NOT IN ('blocked','committed')").fetchone()
            return None if row is None or row[0] is None else float(row[0])

    def _freeze(self, channel, measurement_id, value, field, state):
        value = _blob(value, "冻结内容")
        with self._transaction(channel) as connection:
            row = self._required(connection, channel, measurement_id)
            old = getattr(row, field + "_bytes")
            if old is not None:
                if old != value:
                    raise ValueError("任务的冻结内容已固定，禁止覆盖")
                return row
            if row.state in ("blocked", "committed"):
                raise ValueError("终局任务不能添加新的冻结内容")
            if field == "prepared" and row.preparation_bytes is None:
                raise ValueError("任务尚未冻结原车次准备证据")
            if field == "preparation":
                reserved = len(value) + MAX_PREPARED_TASK_BYTES
                used = connection.execute("SELECT coalesce(sum(completion_reserved_bytes),0) "
                    "FROM pending_tasks WHERE state!='committed'").fetchone()[0]
                if used + reserved > self.max_completion_bytes_per_channel:
                    raise CompletionBusy("本通道完成保留池已满，原任务保留等待，尚未开始计算")
            else:
                reserved = connection.execute("SELECT completion_reserved_bytes FROM pending_tasks "
                    "WHERE measurement_id=?", (measurement_id,)).fetchone()[0]
                if len(value) > MAX_PREPARED_TASK_BYTES or len(row.preparation_bytes) + len(value) > reserved:
                    raise ValueError("最终任务字节超过已持久预留的完成容量")
            # field只由两个固定入口给定，不来自用户/归档字段。
            connection.execute("UPDATE pending_tasks SET " + field + "_bytes=?," + field + "_sha256=?,"
                               "state=?,next_at=0,error='',stored_bytes=stored_bytes+?,"
                               "completion_reserved_bytes=? WHERE measurement_id=?",
                               (value, _digest(value), state, len(value), reserved, measurement_id))
            return replace(row, **{field + "_bytes": value}, state=state, next_at=0., error="")

    def freeze_preparation(self, channel, measurement_id, proof_bytes):
        return self._freeze(channel, measurement_id, proof_bytes, "preparation", "ready_to_compute")

    def freeze_prepared(self, channel, measurement_id, prepared_bytes):
        return self._freeze(channel, measurement_id, prepared_bytes, "prepared", "prepared")

    def retry(self, channel, measurement_id, error, delay, now=None):
        if delay is None:
            raise ValueError("任务重试间隔必须为有限非负秒数")
        delay = _clock(delay)
        next_at = _clock(now) + delay
        if not math.isfinite(next_at):
            raise ValueError("任务重试时间超出范围")
        with self._transaction(channel) as connection:
            row = self._required(connection, channel, measurement_id)
            if row.state in ("blocked", "committed"):
                raise ValueError("终局任务不能自动重试")
            error = str(error)[:1000]
            connection.execute("UPDATE pending_tasks SET attempts=attempts+1,next_at=?,error=? WHERE measurement_id=?",
                               (next_at, error, measurement_id))
            return replace(row, attempts=row.attempts + 1, next_at=next_at, error=error)

    def block(self, channel, measurement_id, error):
        with self._transaction(channel) as connection:
            row = self._required(connection, channel, measurement_id)
            if row.state == "committed":
                raise ValueError("已提交任务不能改为阻断")
            error = str(error)[:1000]
            if row.state == "blocked" and row.error == error:
                return row
            connection.execute("UPDATE pending_tasks SET state='blocked',error=?,completion_reserved_bytes="
                               "coalesce(length(preparation_bytes),0)+coalesce(length(prepared_bytes),0) WHERE measurement_id=?",
                               (error, measurement_id))
            return replace(row, state="blocked", error=error)

    def mark_committed(self, channel, measurement_id):
        with self._transaction(channel) as connection:
            row = self._required(connection, channel, measurement_id)
            if row.state == "committed":
                return row
            if row.state not in ("prepared", "committed") or row.prepared_bytes is None:
                raise ValueError("只有最终结果已冻结的任务才可标记提交")
            if "committed_at" in {
                    column[1] for column in connection.execute("PRAGMA table_info(pending_tasks)")}:
                connection.execute("UPDATE pending_tasks SET state='committed',next_at=0,error='',"
                                   "completion_reserved_bytes=0,committed_at=? WHERE measurement_id=?",
                                   (time.time(), measurement_id))
            else:
                connection.execute("UPDATE pending_tasks SET state='committed',next_at=0,error='',"
                                   "completion_reserved_bytes=0 WHERE measurement_id=?", (measurement_id,))
            return replace(row, state="committed", next_at=0., error="")

    def protected_measurements(self, before):
        """清理先取保护集合；状态只会由未完成向终局推进，不反向复活。"""
        return self.cleanup_protection(before)[0]

    def freeze_exit(self, channel, measurement_id, encoded):
        with self._transaction(channel) as connection:
            connection.execute("INSERT OR IGNORE INTO recalculation_sources(measurement_id,exit_bytes) VALUES (?,?)",
                               (measurement_id, encoded))
            return bytes(connection.execute("SELECT exit_bytes FROM recalculation_sources WHERE measurement_id=?",
                                            (measurement_id,)).fetchone()[0])

    def register_recalculation(self, channel, measurement_id, payload):
        day = datetime.fromisoformat(payload["measuredAt"].replace("Z", "+00:00")).astimezone(
            timezone(timedelta(hours=8))).date().isoformat()
        # 内部载荷已由measurement边界校验，不再重复验证车牌/身份。
        with self._transaction(channel) as connection:
            connection.execute("INSERT OR IGNORE INTO recalculation_sources(measurement_id,exit_bytes) VALUES (?,?)",
                (measurement_id, b'{"frames":[]}'))
            connection.execute("UPDATE recalculation_sources SET plate=?,day=?,eligible=? WHERE measurement_id=?",
                (payload.get("licensePlate", ""), day,
                 int(payload["sourceMode"] == "live" and payload["verdict"] == "loaded"), measurement_id))

    def unindexed_recalculation(self, channel):
        lock, connection = self._lane(channel)
        with lock:
            row = connection.execute("SELECT p.* FROM pending_tasks p LEFT JOIN recalculation_sources s "
                "ON s.measurement_id=p.measurement_id WHERE s.measurement_id IS NULL AND p.prepared_bytes IS NOT NULL LIMIT 1").fetchone()
            return self._row(row, channel)

    def recalculation_since(self, channel):
        lock, connection = self._lane(channel)
        with lock:
            return connection.execute("SELECT MIN(s.day) FROM recalculation_sources s JOIN pending_tasks p "
                "ON p.measurement_id=s.measurement_id WHERE s.eligible=1 AND length(p.input_bytes)>0").fetchone()[0]

    def save_recalculation(self, channel, job_id, measurement_id, payload):
        with self._transaction(channel) as connection:
            connection.execute("INSERT OR IGNORE INTO recalculation_results(id,measurement_id,payload) VALUES (?,?,?)",
                               (job_id, measurement_id, payload))
            return bytes(connection.execute("SELECT payload FROM recalculation_results WHERE id=?", (job_id,)).fetchone()[0])

    def pending_recalculation(self, channel):
        lock, connection = self._lane(channel)
        with lock:
            row = connection.execute("SELECT id,measurement_id,payload FROM recalculation_results WHERE acknowledged=0 ORDER BY rowid LIMIT 1").fetchone()
            return (row[0], row[1], bytes(row[2])) if row else None

    def acknowledge_recalculation(self, channel, job_id, *, superseded=False):
        with self._transaction(channel) as connection:
            connection.execute("UPDATE recalculation_results SET acknowledged=1 WHERE id=?", (job_id,))
            if not superseded:
                connection.execute("UPDATE recalculation_sources SET acknowledged_id=? WHERE measurement_id="
                    "(SELECT measurement_id FROM recalculation_results WHERE id=?)", (job_id, job_id))

    def exit_bytes(self, channel, measurement_id):
        lock, connection = self._lane(channel)
        with lock:
            row = connection.execute("SELECT exit_bytes FROM recalculation_sources WHERE measurement_id=?", (measurement_id,)).fetchone()
            return bytes(row[0]) if row else b'{"frames":[]}'

    def cleanup_protection(self, before):
        """停用/--only未打开的车道旧任务也保护；只读旧库，不启动任务。"""
        before = _clock(before)
        self.ensure_cleanup_schema()
        protected, passages, opened = set(), set(), set()
        def collect(connection):
            columns = {row[1] for row in connection.execute("PRAGMA table_info(pending_tasks)")}
            if not {"measurement_id", "passage_id", "state"}.issubset(columns):
                raise ValueError("历史车道任务库结构未知，暂不清理")
            predicate = "state!='committed' OR committed_at IS NULL OR committed_at>=?" if "committed_at" in columns else "1=1"
            parameters = (before,) if "committed_at" in columns else ()
            for mid, passage in connection.execute("SELECT measurement_id,passage_id FROM pending_tasks WHERE " + predicate, parameters):
                protected.add(mid)
                passages.add(passage)
            if connection.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='recalculation_sources'").fetchone():
                for mid, passage in connection.execute("SELECT p.measurement_id,p.passage_id FROM pending_tasks p "
                    "LEFT JOIN recalculation_sources s ON s.measurement_id=p.measurement_id "
                    "WHERE s.measurement_id IS NULL AND p.prepared_bytes IS NOT NULL AND length(p.input_bytes)>0"):
                    protected.add(mid)
                    passages.add(passage)
                day = datetime.fromtimestamp(before, timezone(timedelta(hours=8))).date().isoformat()
                for mid, passage in connection.execute("SELECT p.measurement_id,p.passage_id FROM pending_tasks p "
                    "JOIN recalculation_sources s ON s.measurement_id=p.measurement_id WHERE s.eligible=1 AND "
                    "(s.acknowledged_id IS NULL OR s.day>=? OR EXISTS "
                    "(SELECT 1 FROM recalculation_results r WHERE r.measurement_id=p.measurement_id AND r.acknowledged=0))", (day,)):
                    protected.add(mid)
                    passages.add(passage)
        for channel in self._connections:
            lock, connection = self._lane(channel)
            with lock:
                opened.add(Path(connection.execute("PRAGMA database_list").fetchone()[2]))
                collect(connection)
        _safe_storage(self.pending_dir)
        for path in self.pending_dir.iterdir():
            if not (path.name.startswith("channel-") and path.name.endswith(".sqlite3")) or path in opened:
                continue
            for suffix in ("", "-wal", "-shm"):
                _reject_link(Path(str(path) + suffix))
            connection = sqlite3.connect(path.absolute().as_uri() + "?mode=ro", uri=True, timeout=1.)
            try:
                collect(connection)
            finally:
                connection.close()
        return protected, passages

    def compact_committed(self, measurement_ids, before):
        """只回收已经清除归档的已完成任务大BLOB；保留原输入SHA和车次唯一索引。"""
        before = _clock(before)
        ids = tuple(dict.fromkeys(_identifier(mid) for mid in measurement_ids))
        if len(ids) > 128:
            raise ValueError("任务清理每批最多128条")
        self.ensure_cleanup_schema()
        removed = 0
        for channel in self._connections:
            with self._transaction(channel) as connection:
                for mid in ids:
                    row = self._find(connection, mid)
                    if (row is None or row["state"] != "committed" or row["compacted_at"] is not None
                            or row["committed_at"] is None or row["committed_at"] >= before):
                        continue
                    self._row(row, channel)  # 压缩前确认原字节未损坏，不能把损坏洗成有效墓碑。
                    size = len(row["input_bytes"]) + len(row["preparation_bytes"] or b"") + len(row["prepared_bytes"] or b"")
                    size += connection.execute("SELECT coalesce((SELECT length(exit_bytes) FROM recalculation_sources "
                        "WHERE measurement_id=?),0)+coalesce((SELECT sum(length(payload)) FROM recalculation_results "
                        "WHERE measurement_id=? AND acknowledged=1),0)", (mid, mid)).fetchone()[0]
                    database = connection.execute("PRAGMA database_list").fetchone()[2]
                    if database and shutil.disk_usage(Path(database).parent).free < size * 2 + 16 * 1024 * 1024:
                        raise OSError(errno.ENOSPC, "任务维护缺少WAL安全空间，请先迁到数据盘或腾出空间")
                    removed += size
                    digest = self._metadata_digest(row)
                    connection.execute("UPDATE pending_tasks SET input_bytes=X'',preparation_bytes=NULL,prepared_bytes=NULL,"
                        "stored_bytes=length(identity_json)+coalesce(length(command_json),0)+length(start_json),"
                        "completion_reserved_bytes=0,compacted_at=?,metadata_sha256=? WHERE measurement_id=?",
                        (time.time(), digest, mid))
                    connection.execute("UPDATE recalculation_sources SET exit_bytes=X'' WHERE measurement_id=?", (mid,))
                    connection.execute("UPDATE recalculation_results SET payload=X'' WHERE measurement_id=? AND acknowledged=1", (mid,))
        return removed

    def snapshot(self, channel):
        lock, connection = self._lane(channel)
        with lock:
            self._lane(channel)
            rows = connection.execute("SELECT state,count(*),coalesce(sum(" + _INPUT_SIZE_SQL + "),0),"
                "min(created_at),coalesce(sum(completion_reserved_bytes),0),"
                "coalesce(sum(coalesce(length(preparation_bytes),0)+coalesce(length(prepared_bytes),0)),0) "
                "FROM pending_tasks GROUP BY state").fetchall()
            active = [row for row in rows if row[0] != "committed"]
            return dict(channel=channel, states={row[0]: row[1] for row in rows},
                        pending_jobs=sum(row[1] for row in active), pending_bytes=sum(row[2] for row in active),
                        max_jobs_per_channel=self.max_jobs_per_channel,
                        max_bytes_per_channel=self.max_bytes_per_channel,
                        completion_reserved_bytes=sum(row[4] for row in active),
                        completion_bytes=sum(row[5] for row in active),
                        max_completion_bytes_per_channel=self.max_completion_bytes_per_channel,
                        oldest_pending_at=min((row[3] for row in active), default=None))

    def close(self):
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            for channel, connection in self._connections.items():
                with self._locks[channel]:
                    connection.close()
