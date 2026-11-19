"""本地03点普通点云维护；不接触DM8、正式NPZ或未完成的恢复材料。"""
from __future__ import annotations

from datetime import datetime, timedelta
import hashlib
import json
import os
from pathlib import Path
import stat
import threading
import time
import uuid

from .cloud_archive import ARCHIVE_DIR, _safe_path
from .settings import root


class StorageMaintenance:
    def __init__(self, service, daily_at, *, clock=None, retry_interval=60.):
        self.service = service
        self.daily_at = daily_at
        self.hour, self.minute = map(int, daily_at.split(":"))
        self.data_root = Path(root(service.st.service.data_root)).absolute()
        self.state_path = self.data_root / "storage-cleanup-state.json"
        self.outputs = {ch.id: Path(ch.abs_output_dir).absolute() for ch in service.st.channels}
        self.protected_roots = {Path(ch.abs_baseline_dir).resolve() for ch in service.st.channels}
        self.protected_roots.add(Path(root("baseline")).absolute())
        self.signature = hashlib.sha256(json.dumps([daily_at, sorted(map(str, self.outputs.values())),
            sorted(map(str, self.protected_roots))], ensure_ascii=False).encode()).hexdigest()
        for path in self.outputs.values():
            if path == self.data_root or not path.is_relative_to(self.data_root):
                raise ValueError("启用日清时output_dir必须是data_root下的独立子目录")
            archive = path / ARCHIVE_DIR
            if any(archive.is_relative_to(p) or p.is_relative_to(archive) for p in self.protected_roots):
                raise ValueError("永久基线目录不能与归档清理树重叠")
        self._stop = threading.Event()
        self._thread = None
        self.last_report = None
        self.last_error = ""
        self._clock = clock or time.time
        self._retry_interval = retry_interval

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="pointcloud-daily-cleanup", daemon=True)
            self._thread.start()

    def close(self, timeout=3.):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)
        return self._thread is None or not self._thread.is_alive()

    def _check_stop(self):
        if self._stop.is_set():
            raise InterruptedError("服务停止，日清不推进水位")

    def _run(self):
        while not self._stop.is_set():
            try:
                self.run_once()
                self.last_error = ""
            except Exception as error:  # 首次目录/读锁/磁盘异常不能拖垮采集启动。
                if isinstance(error, InterruptedError) and self._stop.is_set():
                    break
                self.last_error = type(error).__name__
                self.service.bus.publish("error", dict(channel="", error="每日点云清理暂未完成（%s），一分钟后重试" % self.last_error))
            self._stop.wait(self._retry_interval)

    def due_cutoff(self, now):
        local = datetime.fromtimestamp(now)
        due = local.replace(hour=self.hour, minute=self.minute, second=0, microsecond=0)
        if due > local:
            due -= timedelta(days=1)
        cutoff = due.timestamp()
        _safe_path(self.state_path)
        if self.state_path.exists():
            state = json.loads(self.state_path.read_text(encoding="utf-8"))
            if (not isinstance(state, dict) or state.get("schema") != 1
                    or type(state.get("completed_cutoff")) not in (int, float)
                    or not 0 < state["completed_cutoff"] < float("inf")):
                raise ValueError("日清水位损坏，停止删除")
            if state.get("signature") == self.signature and state["completed_cutoff"] >= cutoff:
                return None
        return cutoff

    def _complete(self, cutoff):
        _safe_path(self.state_path)
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        _safe_path(self.state_path)
        temporary = self.state_path.with_name(self.state_path.name + "." + uuid.uuid4().hex + ".tmp")
        try:
            with temporary.open("xb") as stream:
                stream.write(json.dumps(dict(schema=1, signature=self.signature,
                    completed_cutoff=cutoff), allow_nan=False).encode())
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.state_path)
        finally:
            temporary.unlink(missing_ok=True)

    def _protected_passages(self, cutoff):
        passages = set()
        for worker in self.service.workers:
            with worker._control_lock:
                value = (worker.integration.current_passage(worker.ch.id) if worker.integration is not None
                         else worker._local_passage)
                if value:
                    passages.add(value)
        bridge = self.service.integration
        if bridge is not None:
            with bridge.outbox._lock:
                for row in bridge.outbox._connection.execute("SELECT payload_json FROM outbox "
                        "WHERE status!='sent' OR sent_at_ns IS NULL OR sent_at_ns>=?", (int(cutoff * 1e9),)):
                    value = json.loads(row[0]).get("passageId")
                    if value:
                        passages.add(value)
        return passages

    def _clean_loose(self, cutoff, protected_passages):
        deleted = freed = 0
        def enumeration_failed(error):
            raise error
        for output in set(self.outputs.values()):
            self._check_stop()
            _safe_path(output)
            if not output.exists():
                continue
            for directory, directories, files in os.walk(output, followlinks=False, onerror=enumeration_failed):
                safe_directories = []
                for name in directories:
                    child = Path(directory) / name
                    info = child.lstat()
                    if (name != ARCHIVE_DIR and not stat.S_ISLNK(info.st_mode)
                            and not getattr(info, "st_file_attributes", 0) & 0x400
                            and not any(child.absolute().is_relative_to(p) for p in self.protected_roots)):
                        safe_directories.append(name)
                directories[:] = safe_directories
                if any(Path(directory).absolute().is_relative_to(p) for p in self.protected_roots):
                    directories[:] = []
                    continue
                for name in files:
                    self._check_stop()
                    if not name.endswith(".xyz"):
                        continue
                    cloud = Path(directory) / name
                    metadata = Path(str(cloud) + ".meta.json")
                    _safe_path(cloud)
                    _safe_path(metadata)
                    # 旁注由原写入者在XYZ完成后保存；缺失/损坏时不猜文件已完成。
                    if not metadata.is_file():
                        continue
                    before = cloud.stat()
                    if before.st_mtime >= cutoff or metadata.stat().st_mtime >= cutoff:
                        continue
                    try:
                        note = json.loads(metadata.read_text(encoding="utf-8-sig"))
                    except (ValueError, UnicodeError):
                        continue
                    if not isinstance(note, dict) or note.get("passage_id") in protected_passages:
                        continue
                    _safe_path(cloud)
                    after = cloud.stat()
                    if (after.st_mtime_ns, after.st_size, after.st_ino) != (before.st_mtime_ns, before.st_size, before.st_ino):
                        continue
                    cloud.unlink()
                    for suffix in (".meta.json", ".reason.txt"):
                        sidecar = Path(str(cloud) + suffix)
                        _safe_path(sidecar)
                        sidecar.unlink(missing_ok=True)
                    deleted += 1
                    freed += before.st_size
        return deleted, freed

    def _clean_journal(self, journal, connection, lock, cutoff, protected, outbox=False):
        cursor = 0
        purged = reusable = compacted = 0
        while True:
            self._check_stop()
            with lock:
                if outbox:
                    rows = connection.execute("SELECT o.seq,o.measurement_id FROM outbox o "
                        "LEFT JOIN cloud_archives a ON a.measurement_id=o.measurement_id "
                        "WHERE o.seq>? AND o.status='sent' AND o.sent_at_ns<? AND "
                        "(a.measurement_id IS NULL OR (a.status='complete' AND a.acquired_at<? AND a.completed_at<?)) "
                        "ORDER BY o.seq LIMIT 64", (cursor, int(cutoff * 1e9), cutoff, cutoff)).fetchall()
                else:
                    rows = connection.execute("SELECT seq,measurement_id FROM cloud_archives "
                        "WHERE seq>? AND status='complete' AND acquired_at<? AND completed_at<? "
                        "ORDER BY seq LIMIT 64", (cursor, cutoff, cutoff)).fetchall()
            if not rows:
                break
            cursor = rows[-1][0]
            ids = [row[1] for row in rows if row[1] not in protected]
            report = journal.purge_completed(ids, cutoff, {key: str(value) for key, value in self.outputs.items()}, protected_roots=self.protected_roots)
            purged += len(report["purged"])
            reusable += report["bytes_reusable"]
            if outbox and self.service.deferred is not None:
                compacted += self.service.deferred.store.compact_committed(ids, cutoff)
        return purged, reusable, compacted

    def run_once(self, now=None):
        cutoff = self.due_cutoff(self._clock() if now is None else now)
        if cutoff is None:
            return None
        self._check_stop()
        # 所有清理专属DDL先成功，再做任何删除；失败留给后台下一轮，不影响采集启动。
        bridge = self.service.integration
        if bridge is not None:
            bridge.outbox.archives.ensure_cleanup_schema()
        if self.service.deferred is not None:
            self.service.deferred.store.ensure_cleanup_schema()
        for output in set(self.outputs.values()):
            database = output / ARCHIVE_DIR / "archive.sqlite3"
            _safe_path(database)
            if database.is_file():
                self.service.finalizer._store(str(output))
        with self.service.finalizer._store_lock:
            stores = list(self.service.finalizer._stores.values())
        for store in stores:
            store.journal.ensure_cleanup_schema()
        protected, pending_passages = (self.service.deferred.store.cleanup_protection(cutoff)
                     if self.service.deferred is not None else (set(), set()))
        passages = self._protected_passages(cutoff) | pending_passages
        files, file_bytes = self._clean_loose(cutoff, passages)
        total = [0, 0, 0]
        if bridge is not None:
            outbox = bridge.outbox
            total = list(self._clean_journal(outbox.archives, outbox._connection, outbox._lock,
                                            cutoff, protected, outbox=True))
        for store in stores:
            journal = store.journal
            values = self._clean_journal(journal, journal.connection, journal.lock, cutoff, protected)
            total = [left + right for left, right in zip(total, values)]
        self._check_stop()
        self._complete(cutoff)
        self.last_report = dict(cutoff=cutoff, loose_files_deleted=files, file_bytes_deleted=file_bytes,
            archives_purged=total[0], sqlite_bytes_reusable=total[1] + total[2])
        self.service.bus.publish("warn", dict(channel="", message="每日普通点云清理完成；SQLite已删除内容转为可复用空间，不代表数据库文件立即缩小",
                                             **self.last_report))
        return self.last_report
