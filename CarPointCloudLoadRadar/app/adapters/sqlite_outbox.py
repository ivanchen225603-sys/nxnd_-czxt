"""测量待发队列适配器：SQLite 事务、归档和发送状态。"""
from __future__ import annotations
import json
import sqlite3
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Union
from .bridge_boundary import (_safe_storage, _private_permissions, _reject_link, _json_bytes,
                              _identifier, _validate_asset, command_body, HttpError)

@dataclass(frozen=True)
class OutboxItem:
    measurement_id: str
    payload: Dict[str, Any]
    asset: Optional[bytes]
    status: str
    attempts: int
    last_error: Optional[str]


class SQLiteOutbox:
    """不可变测量及候选文件原子落盘；状态变更不删除 sent/blocked 记录。"""
    def __init__(self, storage_dir: Union[str, Path]) -> None:
        self.storage_dir = _safe_storage(storage_dir)
        self.storage_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
        _safe_storage(self.storage_dir)
        _private_permissions(self.storage_dir, 0o700)
        self.db_path = self.storage_dir / "load-radar-outbox.sqlite3"
        self._lock = threading.RLock()
        self._closed = False
        for path in (self.db_path, Path(str(self.db_path) + "-wal"), Path(str(self.db_path) + "-shm")):
            _reject_link(path)
        self._connection = sqlite3.connect(str(self.db_path), timeout=10.0, isolation_level=None, check_same_thread=False)
        self._connection.row_factory = sqlite3.Row
        try:
            if self._connection.execute("PRAGMA journal_mode=WAL").fetchone()[0].lower() != "wal":
                raise RuntimeError("发件箱无法启用 WAL 持久日志")
            self._connection.execute("PRAGMA synchronous=FULL")
            self._connection.execute("""CREATE TABLE IF NOT EXISTS outbox (
                seq INTEGER PRIMARY KEY AUTOINCREMENT,
                measurement_id TEXT NOT NULL UNIQUE,
                channel_id TEXT NOT NULL,
                payload_json TEXT NOT NULL,
                asset BLOB,
                status TEXT NOT NULL DEFAULT 'pending' CHECK(status IN ('pending','sent','blocked')),
                attempts INTEGER NOT NULL DEFAULT 0,
                last_error TEXT,
                created_at_ns INTEGER NOT NULL,
                sent_at_ns INTEGER
            )""")
            self._connection.execute("CREATE INDEX IF NOT EXISTS outbox_pending ON outbox(status,seq)")
            self._connection.execute("CREATE INDEX IF NOT EXISTS outbox_sent ON outbox(channel_id,status,sent_at_ns)")
            self._connection.execute("""CREATE TABLE IF NOT EXISTS command_results (
                request_id TEXT PRIMARY KEY,
                command_json TEXT NOT NULL,
                measurement_id TEXT NOT NULL UNIQUE
            )""")
            from ..cloud_archive import ArchiveJournal
            self.archives = ArchiveJournal(self._connection, self._lock, owner_kind="outbox")
            self._permissions()
        except Exception:
            self._connection.close()
            raise

    def _permissions(self) -> None:
        for suffix in ("", "-wal", "-shm"):
            _private_permissions(Path(str(self.db_path) + suffix), 0o600)

    def save(self, payload: Dict[str, Any], candidate_bytes: Optional[bytes] = None,
             command: Optional[Dict[str, Any]] = None, archive=None) -> str:
        if not isinstance(payload, dict):
            raise ValueError("发件箱测量必须为 JSON 对象")
        measurement_id = _identifier(payload.get("measurementId"))
        channel_id = _identifier(payload.get("channelId"), 64)
        encoded = _json_bytes(payload).decode("utf-8")
        if candidate_bytes is not None:
            _validate_asset(payload.get("baselineCandidateSha256"), candidate_bytes)
        command_json = None
        if command is not None:
            command = command_body(command)
            self._validate_command_measurement(command, payload, candidate_bytes)
            command_json = _json_bytes(command).decode("utf-8")
        if archive is not None:
            from datetime import datetime, timezone
            meta = archive.validate()
            for local, remote in (("measurement_id", "measurementId"), ("channel", "channelId"),
                                  ("passage_id", "passageId"), ("configuration_hash", "configurationHash"),
                                  ("algorithm_version", "algorithmVersion"),
                                  ("source_mode", "sourceMode"),
                                  ("baseline_version_id", "baselineVersionId")):
                if meta[local] != payload.get(remote):
                    raise ValueError("归档与 outbox 测量身份或版本不一致")
            result = json.loads(dict(archive.files)["detection.xyz.meta.json"])
            verdict = payload.get("verdict")
            if ((result.get("empty") is True and verdict != "empty") or
                    (result.get("empty") is False and verdict not in ("loaded", "unknown")) or
                    (result.get("empty") is None and verdict != "unknown") or
                    type(payload.get("frameDegraded")) is not bool or
                    payload["frameDegraded"] == meta["frame"]["complete"]):
                raise ValueError("归档与 outbox 的判定或完整性不一致")
            acquired = datetime.fromtimestamp(meta["acquired_at"], timezone.utc).isoformat().replace("+00:00", "Z")
            if acquired != payload.get("measuredAt") or result.get("reason") != payload.get("reason"):
                raise ValueError("归档与 outbox 的采集时间或理由不一致")
        with self._lock:
            self._connection.execute("BEGIN IMMEDIATE")
            try:
                existing = self._connection.execute("SELECT payload_json,asset FROM outbox WHERE measurement_id=?", (measurement_id,)).fetchone()
                if command is not None:
                    receipt = self._connection.execute("SELECT * FROM command_results WHERE request_id=?",
                                                       (command["requestId"],)).fetchone()
                    if receipt is not None:
                        if receipt["command_json"] != command_json or receipt["measurement_id"] != measurement_id:
                            raise ValueError("同一命令的原始载荷与测量已经固定，禁止覆盖")
                        if existing is None:
                            raise ValueError("命令完成凭据缺少原测量")
                    elif existing is not None:
                        raise ValueError("禁止把既有测量事后挂到新命令")
                if existing is not None:
                    if existing["payload_json"] != encoded or candidate_bytes is not None and existing["asset"] != candidate_bytes:
                        raise ValueError("同一测量 ID 的载荷或候选资产已经固定，禁止覆盖")
                    if archive is not None and self.archives.receipt(measurement_id) is None:
                        raise ValueError("禁止把既有测量事后挂到新归档")
                else:
                    self._connection.execute("""INSERT INTO outbox
                        (measurement_id,channel_id,payload_json,asset,created_at_ns) VALUES (?,?,?,?,?)""",
                        (measurement_id, channel_id, encoded, candidate_bytes, time.time_ns()))
                if command is not None and receipt is None:
                    self._connection.execute("INSERT INTO command_results VALUES (?,?,?)",
                                             (command["requestId"], command_json, measurement_id))
                if archive is not None:
                    self.archives.put(archive)
                self._connection.commit()
                self._permissions()
                return measurement_id
            except Exception:
                self._connection.rollback()
                raise

    @staticmethod
    def _validate_command_measurement(command: Dict[str, Any], payload: Dict[str, Any],
                                      asset: Optional[bytes]) -> None:
        if any(command[key] != payload.get(key) for key in ("channelId", "passageId")):
            raise ValueError("命令与测量的通道或车次不一致")
        if command["kind"] == "baseline-capture":
            if (asset is None or payload.get("sourceMode") != "live" or payload.get("verdict") != "empty"
                    or payload.get("frameDegraded") is not False):
                raise ValueError("空车候选完成凭据缺少完整实车测量或资产")
            _validate_asset(payload.get("baselineCandidateSha256"), asset)

    def command_result(self, request_id: str) -> Optional[Dict[str, Any]]:
        """持久凭据为权威；损坏或读盘故障抛错，不能当成未执行。"""
        with self._lock:
            row = self._connection.execute("SELECT * FROM command_results WHERE request_id=?",
                                           (_identifier(request_id, 64),)).fetchone()
            if row is None:
                return None
            try:
                body = command_body(json.loads(row["command_json"]))
                item = self.get(row["measurement_id"])
                if item is not None:
                    self._validate_command_measurement(body, item.payload, item.asset)
            except (ValueError, TypeError, KeyError):
                raise RuntimeError("命令完成凭据损坏，结果暂时无法确认") from None
            if body["requestId"] != request_id or item is None:
                raise RuntimeError("命令完成凭据不完整，须核对历史")
            return {"command": body, "measurement": item}

    @staticmethod
    def _item(row: sqlite3.Row) -> OutboxItem:
        return OutboxItem(row["measurement_id"], json.loads(row["payload_json"]), row["asset"],
                          row["status"], row["attempts"], row["last_error"])

    def get(self, measurement_id: str) -> Optional[OutboxItem]:
        with self._lock:
            row = self._connection.execute("SELECT * FROM outbox WHERE measurement_id=?", (_identifier(measurement_id),)).fetchone()
            return self._item(row) if row is not None else None

    def pending(self, limit: int = 100) -> List[OutboxItem]:
        with self._lock:
            rows = self._connection.execute("SELECT * FROM outbox WHERE status='pending' ORDER BY seq LIMIT ?", (self._limit(limit),)).fetchall()
            return [self._item(row) for row in rows]

    def latest_sent(self, channel_id: str, limit: int = 20) -> List[Dict[str, Any]]:
        """返回该通道最近发送成功的原始 payload，新到旧；业务应以 get(id) 精确重查绑定。"""
        with self._lock:
            rows = self._connection.execute("""SELECT payload_json FROM outbox WHERE channel_id=? AND status='sent'
                ORDER BY sent_at_ns DESC,seq DESC LIMIT ?""", (_identifier(channel_id, 64), self._limit(limit))).fetchall()
            return [json.loads(row[0]) for row in rows]

    @staticmethod
    def _limit(limit: int) -> int:
        if type(limit) is not int or not 1 <= limit <= 1000:
            raise ValueError("发件箱读取数量须为 1 至 1000")
        return limit

    def mark_sent(self, measurement_id: str) -> None:
        self._transition(measurement_id, "sent", None)

    def mark_blocked(self, measurement_id: str, error: Any) -> None:
        self._transition(measurement_id, "blocked", error)

    def note_retry(self, measurement_id: str, error: Any) -> None:
        self._transition(measurement_id, "pending", error)

    def _transition(self, measurement_id: str, target: str, failure: Any) -> None:
        # 不持久化调用者任意异常文本：其中可能含完整 URL、响应正文或节点凭据。
        safe_error = str(failure) if isinstance(failure, HttpError) else "发送失败，敏感详情已隐藏"
        with self._lock:
            self._connection.execute("BEGIN IMMEDIATE")
            try:
                row = self._connection.execute("SELECT status FROM outbox WHERE measurement_id=?", (_identifier(measurement_id),)).fetchone()
                if row is None:
                    raise KeyError("发件箱测量不存在")
                if row["status"] == target and target != "pending":
                    self._connection.commit()
                    return
                if row["status"] != "pending":
                    raise ValueError("已发送或阻塞的测量不能被后台重试改写状态")
                self._connection.execute("""UPDATE outbox SET status=?,attempts=attempts+1,last_error=?,sent_at_ns=?
                    WHERE measurement_id=?""", (target, None if target == "sent" else safe_error,
                    time.time_ns() if target == "sent" else None, measurement_id))
                self._connection.commit()
                self._permissions()
            except Exception:
                self._connection.rollback()
                raise

    def capacity_snapshot(self, *, refresh=False) -> dict:
        """仅已有所有者可调用；不打开第二连接或执行全库 COUNT。"""
        return self.archives.capacity_snapshot(refresh=refresh)

    def stats(self) -> Dict[str, int]:
        with self._lock:
            result = {"pending": 0, "sent": 0, "blocked": 0}
            for row in self._connection.execute("SELECT status,COUNT(*) AS amount FROM outbox GROUP BY status"):
                result[row["status"]] = row["amount"]
            result["total"] = sum(result.values())
            return result

    def close(self) -> None:
        # 不能先持数据库锁再等导出锁，否则和导出完成事务形成死锁。
        self.archives.close_connection()
        self._closed = True

    def __enter__(self) -> "SQLiteOutbox":
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()
