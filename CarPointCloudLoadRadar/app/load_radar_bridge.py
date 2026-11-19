"""JT128 与业务后台之间的可靠适配；不参与几何判定。"""
from __future__ import annotations

import base64
import hashlib
import hmac
import json
import os
import sqlite3
import threading
import time
import uuid
from datetime import datetime, timezone
from dataclasses import dataclass
from typing import Dict, Optional

from .load_radar_contract import BaselineCache, build_measurement, make_candidate, _hash
from .load_radar_transport import BridgeConfig, HttpClient, HttpError, SQLiteOutbox, command_body


class BridgeConflict(ValueError):
    """请求不适用于当前车辆，必须人工核对。"""


@dataclass(frozen=True)
class _ArchivePreparationFailure:
    """冻结失败尚无可重放字节；保留原身份并关闭自动重采，不能猜成未执行。"""
    error_type: str


@dataclass(frozen=True)
class ResultReservation:
    """采集时固定业务身份；后台构造不得改读下一辆车。"""
    measurement_id: str
    channel: str
    passage_id: Optional[str]
    origin_token: str
    command_json: Optional[bytes] = None

    @property
    def command(self):
        return json.loads(self.command_json) if self.command_json is not None else None


@dataclass(frozen=True)
class BaselineSnapshot:
    """预留车次对应的实际基线；全部拥有不可变字节，不别名活动缓存。"""
    passage_id: str
    binding_json: bytes
    cloud_bytes: bytes
    floor_z: float
    box: tuple
    bed_plane: Optional[tuple]
    asset_bytes: Optional[bytes] = None
    source_json: Optional[bytes] = None

    @property
    def binding(self):
        return json.loads(self.binding_json)

    def baseline(self):
        import numpy as np
        from .detect import EmptyBaseline
        cloud = np.frombuffer(self.cloud_bytes, "<f8").reshape(-1, 3)
        return EmptyBaseline(self.floor_z, self.box, cloud, len(cloud), bed_plane=self.bed_plane)


class LoadRadarBridge(threading.Thread):
    """落盘后再发送；进程重启或断流后必须重新确认车道空闲。"""

    def __init__(self, config: BridgeConfig, client=None):
        super().__init__(daemon=True, name="load-radar-sync")
        self.config = config
        self.client = client or HttpClient(config)
        self.outbox = SQLiteOutbox(config.storage_dir)
        self.cache = BaselineCache(config.storage_dir)
        self._lock = threading.RLock()
        self._db_lock = threading.RLock()
        self._command_submit_lock = threading.Lock()
        self._quit = threading.Event()
        self._wake = threading.Event()
        self._frames: Dict[str, float] = {}
        self._idle: Dict[str, tuple] = {}
        self._loaded: Dict[str, tuple] = {}
        self._channel_contexts: Dict[str, tuple] = {}
        self._starts: Dict[str, dict] = {}
        self._prepared: Dict[str, tuple] = {}
        self._uncommitted: Dict[str, tuple] = {}
        self._reserved: Dict[str, ResultReservation] = {}
        self._error = ""
        self._closed = False
        self._async_preparation = False
        # 位置停稳通知（CH4 双雷达触发）：每车次一次，独立线程发送，不排在测量上传后面。
        self._standstill_pending: Dict[str, str] = {}
        self._standstill_done: Dict[tuple, None] = {}
        self._standstill_wake = threading.Event()
        self._standstill_thread: Optional[threading.Thread] = None
        # 同一队列只能由一个进程承载，避免两套通道共用业务 ID。
        from pathlib import Path
        for name in ("bridge.lock", "passages.sqlite3", "passages.sqlite3-wal", "passages.sqlite3-shm"):
            BaselineCache._check_path(Path(config.storage_dir) / name)
        self._lease = open(os.path.join(config.storage_dir, "bridge.lock"), "a+b")
        try:
            if os.name == "nt":
                import msvcrt
                self._lease.seek(0)
                msvcrt.locking(self._lease.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(self._lease.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self._lease.close()
            self.outbox.close()
            raise ValueError("载货接口队列已被另一进程使用") from None
        self._db = sqlite3.connect(os.path.join(config.storage_dir, "passages.sqlite3"),
                                   check_same_thread=False)
        self._db.row_factory = sqlite3.Row
        self._db.execute("PRAGMA journal_mode=WAL")
        self._db.execute("PRAGMA synchronous=FULL")
        self._db.executescript("""
            CREATE TABLE IF NOT EXISTS lanes (
              channel TEXT PRIMARY KEY, passage TEXT, ready INTEGER NOT NULL DEFAULT 0,
              measurement TEXT);
            CREATE TABLE IF NOT EXISTS commands (
              id TEXT PRIMARY KEY, body TEXT NOT NULL, channel TEXT NOT NULL,
              passage TEXT NOT NULL, status TEXT NOT NULL, measurement TEXT, error TEXT);
        """)
        with self._db:
            for channel in config.channels:
                self._db.execute("INSERT OR IGNORE INTO lanes(channel) VALUES (?)", (channel,))
            self._db.execute("UPDATE lanes SET ready=0")
            self._db.execute("UPDATE commands SET status='failed', error=? "
                             "WHERE status IN ('pending','running')",
                             ("进程已重启，命令未确认完成，可能已有文件或测量；请先核对历史，不会自动重新采集",))
        # 过车状态仅对本次进程有效；重启必须重新观察空闲，不回读旧车次。
        self._lanes = {channel: dict(channel=channel, ready=0, passage=None, measurement=None)
                       for channel in config.channels}
        self._commands = {}
        self._command_dirty = {}
        from .archive_capacity import CapacityProbe
        self._capacity_db_closed = False
        self._capacity = CapacityProbe(self._db, self._db_lock, lambda: self._capacity_db_closed, "passages")
        self._projection_wake = threading.Event()
        self._projection_quit = threading.Event()
        self._projection_thread = threading.Thread(target=self._projection_loop, daemon=True,
                                                   name="load-radar-projection")
        self._projection_thread.start()

    def authorized(self, node: str, key: str) -> bool:
        return (hmac.compare_digest(node.encode(), self.config.node_id.encode()) and
                hmac.compare_digest(key.encode(), self.config.agent_key.encode()))

    def _lane(self, channel: str):
        return self._lanes.get(channel)

    def _set_command(self, request_id, **changes):
        """调用者持状态锁；持久投影不在采集线程执行。"""
        row = self._commands[request_id]
        row.update(changes)
        self._command_dirty[request_id] = dict(row)
        self._projection_wake.set()

    def _fail_pending(self, channel, error):
        for request_id, row in list(self._commands.items()):
            if row["channel"] == channel and row["status"] == "pending":
                self._set_command(request_id, status="failed", error=error)

    def _projection_loop(self):
        while not self._projection_quit.is_set():
            self._projection_wake.wait(.1)
            self._projection_wake.clear()
            with self._lock:
                rows = list(self._command_dirty.items())
            for request_id, row in rows:
                if self._projection_quit.is_set():
                    return
                try:
                    with self._db_lock, self._db:
                        self._db.execute("UPDATE commands SET status=?,measurement=?,error=? WHERE id=?",
                            (row["status"], row["measurement"], row["error"], request_id))
                    with self._lock:
                        if self._command_dirty.get(request_id) == row:
                            self._command_dirty.pop(request_id)
                except Exception:
                    # pending 受理早已持久化；重启隔离和 outbox 完成凭据保证不重采。
                    self._projection_quit.wait(.2)

    def _quarantine(self, channel: str) -> None:
        self._loaded.pop(channel, None)
        self._prepared.pop(channel, None)
        self._starts.pop(channel, None)
        self._idle.pop(channel, None)
        self._lanes[channel]["ready"] = 0
        self._fail_pending(channel, "采集连续性中断，请核对车辆")

    def configure_channel(self, channel: str, configuration_hash: str, source_mode: str) -> None:
        """由实际 Worker 配置冻结；不触发网络，重新配置须再次观察完整空闲。"""
        if (channel not in self.config.channels or not isinstance(configuration_hash, str) or
                len(configuration_hash) != 64 or any(c not in "0123456789abcdef" for c in configuration_hash)):
            raise ValueError("到车上下文缺少实际配置摘要")
        source_mode = {"udp": "live", "real": "live", "simulate": "demo", "sim": "demo",
                       "sim-radar": "demo", "synthetic": "demo", "syn": "demo"}.get(source_mode, source_mode)
        if source_mode not in ("live", "demo", "replay"):
            raise ValueError("未知到车数据源")
        with self._lock:
            self._quarantine(channel)
            self._channel_contexts[channel] = (configuration_hash, source_mode)

    def preparation_status(self, channel: str) -> str:
        with self._lock:
            context = self._channel_contexts.get(channel)
            if context is None or context[1] != "live":
                return "not_applicable"
            passage = self.current_passage(channel)
            prepared = self._prepared.get(channel)
            if not prepared or prepared[0] != passage or time.monotonic() - prepared[3] > 10:
                return "awaiting_plate"
            return prepared[1]

    def prepared_plate(self, channel: str) -> str:
        """只返回当前车次、仍有效的 WebApi 权威绑定车牌。"""
        with self._lock:
            context = self._channel_contexts.get(channel)
            if context is None or context[1] != "live":
                return ""
            passage = self.current_passage(channel)
            prepared = self._prepared.get(channel)
            if (not prepared or prepared[0] != passage or
                    prepared[1] not in ("ready", "baseline_required") or
                    time.monotonic() - prepared[3] > 10):
                return ""
            binding = prepared[2]
            return str(binding.get("licensePlate") or "") if isinstance(binding, dict) else ""

    def observe(self, channel: str, occupied: Optional[bool], trusted: bool) -> None:
        """只接受完整新帧的明确占位/离场信息，不以算法会话号判车次。"""
        now = time.monotonic()
        with self._lock:
            if channel not in self.config.channels:
                raise ValueError("未配置的接口通道")
            if now - self._frames.get(channel, 0) > 5:
                self._quarantine(channel)
            if not trusted:
                self._idle.pop(channel, None)
                return
            self._frames[channel] = now
            lane = self._lane(channel)
            if occupied is False:
                count, since = self._idle.get(channel, (0, now))
                self._idle[channel] = (count + 1, since)
                if count + 1 >= 3 and now - since >= .5:
                    self._loaded.pop(channel, None)
                    self._prepared.pop(channel, None)
                    self._starts.pop(channel, None)
                    lane.update(ready=1, passage=None, measurement=None)
                    self._fail_pending(channel, "车辆已经离场")
            else:
                self._idle.pop(channel, None)
                if occupied is True and lane["ready"] and not lane["passage"]:
                    lane.update(passage="p-" + uuid.uuid4().hex, measurement=None)
                    context = self._channel_contexts.get(channel)
                    if context and context[1] == "live":
                        self._starts[channel] = dict(nodeId=self.config.node_id, channelId=channel,
                            passageId=lane["passage"], calibrationId=self.config.channels[channel].calibration_id,
                            configurationHash=context[0], sourceMode="live",
                            arrivedAt=datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"))
                        self._wake.set()

    def report_standstill(self, channel: str, passage: Optional[str]) -> bool:
        """位置停稳时由通道线程调用：本车次已登记且尚未通知过才排队，网络在后台线程。"""
        with self._lock:
            prepared = self._prepared.get(channel)
            if (self._closed or not passage or prepared is None or prepared[0] != passage
                    or (channel, passage) in self._standstill_done
                    or self._standstill_pending.get(channel) == passage):
                return False
            self._standstill_pending[channel] = passage
            if self._standstill_thread is None:
                self._standstill_thread = threading.Thread(target=self._standstill_loop, daemon=True,
                                                           name="load-radar-standstill")
                self._standstill_thread.start()
        self._standstill_wake.set()
        return True

    def _standstill_loop(self) -> None:
        while not self._quit.is_set():
            self._standstill_wake.clear()
            with self._lock:
                jobs = list(self._standstill_pending.items())
            for channel, passage in jobs:
                if self._quit.is_set():
                    return
                if self.current_passage(channel) != passage:
                    with self._lock:              # 车辆已离场或换车，不再通知
                        if self._standstill_pending.get(channel) == passage:
                            del self._standstill_pending[channel]
                    continue
                try:
                    self.client.report_standstill(dict(nodeId=self.config.node_id, channelId=channel,
                                                       passageId=passage))
                except HttpError as exc:
                    if exc.retryable:
                        continue                  # 网络失败：车次仍在时下一轮重发
                except Exception:                 # noqa: BLE001
                    continue
                with self._lock:                  # 收到答复即结束本车次
                    if self._standstill_pending.get(channel) == passage:
                        del self._standstill_pending[channel]
                    self._standstill_done[(channel, passage)] = None
                    while len(self._standstill_done) > 256:
                        self._standstill_done.pop(next(iter(self._standstill_done)))
            with self._lock:
                waiting = bool(self._standstill_pending)
            self._standstill_wake.wait(max(1.0, float(self.config.retry_sec)) if waiting else 1.0)

    def current_passage(self, channel: str) -> Optional[str]:
        with self._lock:
            if time.monotonic() - self._frames.get(channel, 0) > 5:
                self._quarantine(channel)
            lane = self._lane(channel)
            return lane["passage"] if lane and lane["ready"] else None

    def baseline(self, channel: str, measured_at: float):
        """返回本次实际使用的版本快照；后台不可达时租约到期关闭体积计算。"""
        with self._lock:
            passage = self.current_passage(channel)
            row = self._loaded.get(channel)
            if not row or row[0] != passage or time.monotonic() - row[4] > 10:
                return None, None, ""
            activated = datetime.fromisoformat(row[1]["activatedAt"].replace("Z", "+00:00"))
            if activated.timestamp() > measured_at:
                return None, None, ""
            return row[2], row[1]["versionId"], row[3]["licensePlate"]

    def reserve_result(self, channel: str, origin_token: str, command=None) -> ResultReservation:
        """短锁预留一次采集；不读 outbox，不构造资产，不进行大文件 I/O。"""
        body = (command_body({key: command[key] for key in
                ("requestId", "channelId", "passageId", "kind", "confirmedEmpty")}) if command else None)
        with self._lock:
            if channel in self._uncommitted or channel in self._reserved:
                raise BridgeConflict("本通道原测量尚未完成，禁止重新预留")
            if channel not in self.config.channels:
                raise BridgeConflict("未配置的接口通道")
            passage = self.current_passage(channel)
            if body:
                row = self._commands.get(body["requestId"])
                if (body["channelId"] != channel or body["passageId"] != passage or row is None
                        or row["status"] != "running" or json.loads(row["body"]) != body):
                    raise BridgeConflict("命令没有当前车次的有效运行态受理记录")
            reservation = ResultReservation("m-" + uuid.uuid4().hex, channel, passage, origin_token,
                json.dumps(body, sort_keys=True, separators=(",", ":")).encode("utf-8") if body else None)
            self._reserved[channel] = reservation
            return reservation

    def baseline_snapshot(self, reservation: ResultReservation, measured_at: float,
                          configuration_hash: str) -> Optional[BaselineSnapshot]:
        """预留后在同一身份锁内匹配；不访问数据库、不再选择下一车基线。"""
        import numpy as np
        from .bed_reference import baseline_digest
        with self._lock:
            channel = reservation.channel
            if (self._reserved.get(channel) != reservation or not reservation.passage_id or
                    self.current_passage(channel) != reservation.passage_id):
                return None
            row = self._loaded.get(channel)
            if (not row or row[0] != reservation.passage_id or row[5] != configuration_hash or
                    time.monotonic() - row[4] > 10):
                return None
            activated = datetime.fromisoformat(row[1]["activatedAt"].replace("Z", "+00:00"))
            if not np.isfinite(measured_at) or activated.timestamp() > measured_at:
                return None
            base = row[2]
            binding = dict(node_id=self.config.node_id, channel_id=channel,
                calibration_id=self.config.channels[channel].calibration_id,
                configuration_sha256=configuration_hash, baseline_version_id=row[1]["versionId"],
                baseline_asset_sha256=row[1]["assetSha256"], baseline_sha256=baseline_digest(base))
            return BaselineSnapshot(reservation.passage_id,
                json.dumps(binding, sort_keys=True, separators=(",", ":")).encode("utf-8"),
                np.asarray(base.cloud, "<f8").tobytes(), float(base.floor_z), tuple(base.box),
                tuple(base.bed_plane) if base.bed_plane is not None else None,
                row[6] if len(row) > 6 else None,
                row[7] if len(row) > 7 else None)

    def commit_reserved(self, reservation: ResultReservation, payload: dict, *, asset=None, archive=None) -> str:
        """锁外提交原字节；CAS 投影即使晚于下一辆车也不覆盖下一车。"""
        if (payload.get("measurementId") != reservation.measurement_id
                or payload.get("channelId") != reservation.channel
                or payload.get("passageId") != reservation.passage_id or reservation.passage_id is None):
            raise BridgeConflict("冻结结果与预留身份不一致")
        with self._lock:
            if self._reserved.get(reservation.channel) != reservation:
                raise BridgeConflict("原结果预留已失效，禁止替换")
        self.outbox.save(payload, candidate_bytes=asset, command=reservation.command, archive=archive)
        with self._lock:
            self._project_measurement(payload, reservation.command)
            if self._reserved.get(reservation.channel) == reservation:
                self._reserved.pop(reservation.channel)
        self._wake.set()
        return reservation.measurement_id

    def capture_identity(self, channel: str, origin_token: str):
        """短锁取得原车次和到车正文；不占用等待结果的通道槽。"""
        from .load_radar_transport import _json_bytes
        with self._lock:
            passage = self.current_passage(channel)
            start = self._starts.get(channel)
            if not passage or start is None or start["passageId"] != passage:
                raise BridgeConflict("没有可信物理车次，不能冻结正式待计算任务")
            return (ResultReservation("m-" + uuid.uuid4().hex, channel, passage, origin_token),
                    _json_bytes(start))

    def adopt_deferred(self, identity: ResultReservation):
        """只供已持久任务完整计算后的短期提交；不改变当前车次选择规则。"""
        with self._lock:
            if identity.channel not in self.config.channels or not identity.passage_id:
                raise BridgeConflict("历史提交身份无效")
            prior = self._reserved.get(identity.channel)
            if prior is not None and prior != identity or identity.channel in self._uncommitted:
                raise BridgeConflict("本通道已有完整结果等待提交")
            self._reserved[identity.channel] = identity

    def _frozen_receipt(self, identity, start, receipt, measured_at):
        from .load_radar_contract import validate_baseline_as_of
        channel = self.config.channels.get(identity.channel)
        if (channel is None or identity.passage_id != start.get("passageId") or
                start.get("nodeId") != self.config.node_id or start.get("channelId") != identity.channel or
                start.get("calibrationId") != channel.calibration_id):
            raise ValueError("冻结原车次与当前节点授权作用域不一致")
        actual = receipt.get("start")
        if (not isinstance(actual, dict) or set(actual) != set(start) or
                any(actual.get(k) != v for k, v in start.items() if k != "arrivedAt") or
                datetime.fromisoformat(actual.get("arrivedAt", "").replace("Z", "+00:00")) !=
                datetime.fromisoformat(start["arrivedAt"].replace("Z", "+00:00"))):
            raise ValueError("冻结到车正文回显不一致")
        status, binding, version = receipt.get("status"), receipt.get("binding"), receipt.get("baseline")
        if status not in ("awaiting_plate", "plate_unavailable", "baseline_required", "ready"):
            raise ValueError("未知历史准备状态")
        if status in ("awaiting_plate", "plate_unavailable"):
            if binding is not None or version is not None:
                raise ValueError("未取到车牌的历史回执不能携带绑定或基线")
            return status, None, None
        validate_baseline_as_of(measured_at, receipt.get("baselineAsOf"))
        key = hashlib.sha256(json.dumps([self.config.node_id, identity.channel, identity.passage_id],
            separators=(",", ":")).encode("utf-8")).hexdigest()
        workflow = hashlib.sha256(json.dumps(["load-radar-arrival", key],
            separators=(",", ":")).encode("utf-8")).hexdigest()
        if (not isinstance(binding, dict) or binding.get("passageKey") != key or
                receipt.get("workflowId") != workflow or binding.get("workflowId") != workflow or
                not binding.get("licensePlate") or binding.get("licensePlate") == "UNKNOWN" or
                binding.get("vehicleIdentity") != "plate:" + binding["licensePlate"] or
                not receipt.get("cameraIdentifier") or binding.get("cameraIdentifier") != receipt["cameraIdentifier"]):
            raise ValueError("冻结车次权威身份回显不一致")
        if status == "baseline_required":
            if version is not None:
                raise ValueError("历史无基线回执不能夹带版本")
            return status, binding, None
        selected = self.cache.select_historical([version], binding, self.config.node_id,
            identity.channel, channel.calibration_id, measured_at=measured_at,
            baseline_as_of=receipt.get("baselineAsOf"))
        if selected is None:
            raise ValueError("历史基线不匹配冻结原车次")
        return status, binding, selected

    def prepare_frozen(self, identity, start_json: bytes, measured_at: float):
        """网络在调用者独立车道线程执行，迟到结果不写活动通道缓存。"""
        start = json.loads(start_json)
        receipt = self.client.prepare_passage(start, measured_at=measured_at)
        status, _binding, version = self._frozen_receipt(identity, start, receipt, measured_at)
        if status in ("awaiting_plate", "plate_unavailable"):
            return status, None
        origin, asset = None, None
        if version is not None:
            origin = self.client.get_measurement(version["measurementId"]).get("measurement", {})
            asset = self.client.download_baseline(version["versionId"])
        proof = dict(schema="jt128-historical-preparation-v1", receipt=receipt, origin=origin,
                     asset=base64.b64encode(asset).decode("ascii") if asset is not None else None)
        encoded = json.dumps(proof, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
        self.frozen_preparation(identity, start_json, measured_at, encoded)
        return status, encoded

    def frozen_preparation(self, identity, start_json: bytes, measured_at: float, proof_bytes: bytes):
        """只从持久原回执和资产恢复；不读当前车辆、不刷新租约、不把inactive改active。"""
        import numpy as np
        from .bed_reference import baseline_digest
        from .load_radar_contract import MAX_ASSET_BYTES, _read_baseline
        if not isinstance(proof_bytes, bytes) or len(proof_bytes) > MAX_ASSET_BYTES * 2:
            raise ValueError("历史准备证据超过预算")
        proof, start = json.loads(proof_bytes), json.loads(start_json)
        if set(proof) != {"schema", "receipt", "origin", "asset"} or proof["schema"] != "jt128-historical-preparation-v1":
            raise ValueError("历史准备证据格式不符")
        status, binding, version = self._frozen_receipt(identity, start, proof["receipt"], measured_at)
        if status not in ("ready", "baseline_required"):
            raise ValueError("持久证据尚无权威车牌")
        if version is None:
            if proof["asset"] is not None or proof["origin"] is not None:
                raise ValueError("无基线证据不得包含资产")
            return None, binding["licensePlate"]
        expected = dict(measurementId=version["measurementId"], nodeId=self.config.node_id,
            channelId="CH4", calibrationId=version["calibrationId"],
            baselineCandidateSha256=version["assetSha256"],
            sourceMode="live", verdict="empty", frameDegraded=False)
        if not isinstance(proof["origin"], dict) or any(proof["origin"].get(k) != v for k, v in expected.items()):
            raise ValueError("历史基线来源测量不匹配冻结配置")
        asset = base64.b64decode(proof["asset"], validate=True)
        if not 0 < len(asset) <= MAX_ASSET_BYTES or hashlib.sha256(asset).hexdigest() != version["assetSha256"]:
            raise ValueError("历史基线资产SHA或大小不符")
        baseline = _read_baseline(asset)
        scope = dict(node_id=self.config.node_id, channel_id=identity.channel,
            calibration_id=start["calibrationId"], configuration_sha256=start["configurationHash"],
            baseline_version_id=version["versionId"], baseline_asset_sha256=version["assetSha256"],
            baseline_sha256=baseline_digest(baseline))
        return BaselineSnapshot(identity.passage_id,
            json.dumps(scope, sort_keys=True, separators=(",", ":")).encode("utf-8"),
            np.asarray(baseline.cloud, "<f8").tobytes(), float(baseline.floor_z), tuple(baseline.box),
            tuple(baseline.bed_plane) if baseline.bed_plane is not None else None, asset,
            json.dumps(dict(sourceChannelId="CH4", sourceCalibrationId=version["calibrationId"],
                sourceConfigurationHash=_hash(proof["origin"].get("configurationHash"), "CH4 configurationHash")), sort_keys=True).encode("utf-8")), binding["licensePlate"]

    def release_quarantined(self, reservation: ResultReservation) -> None:
        """隔离结果仅本地归档后释放；不创建后台 passage 或伪上报。"""
        with self._lock:
            if reservation.passage_id is not None or self._reserved.get(reservation.channel) != reservation:
                raise BridgeConflict("仅可释放本次隔离观测预留")
            self._reserved.pop(reservation.channel)

    def abandon_reservation(self, reservation: ResultReservation) -> None:
        """结果**从未冻结出来**时放弃预留；不放弃已冻结的结果。

        与另外两条释放路径严格分开：
          · `cancel_rejected` —— 明确的人工业务拒绝，要求 command 非空；
          · `release_quarantined` —— 隔离观测本地归档完成后释放；
          · 本条 —— 冻结阶段就抛了异常，`outbox` 里根本没有这个
            measurement_id，投影也没发生。放弃它不会让任何一辆车拿到
            第二条记录：本会话的检测计数由 `service` 侧保留，这辆车
            不会被重测；下一辆车才拿新 ID。

        ▲ **绝不供提交失败使用**。那种情况结果已经冻结成字节，必须原样
          重放（`ResultFinalizer._commit_loop` 的 `commit_pending` 分支），
          换 ID 会让同一辆车留下两条记录。
        ▲ 不放弃的后果实测过：`_reserved` 一直占着 -> `reserve_result`
          恒抛 BridgeConflict -> 该通道此后每辆车都不测。
        """
        if self.outbox.get(reservation.measurement_id) is not None:
            raise BridgeConflict("该测量已经冻结并入库，只能原字节重放")
        with self._lock:
            if self._reserved.get(reservation.channel) != reservation:
                raise BridgeConflict("只能放弃本通道当前的预留")
            self._reserved.pop(reservation.channel)
            if reservation.command is not None:
                self._set_command(reservation.command["requestId"], status="failed",
                                  error="结果冻结失败，本次未产生测量")

    def claim_command_fast(self, channel: str) -> Optional[dict]:
        """采集线程只认领已受理的新命令；受理/查询的 outbox 校验由非采集线程执行。"""
        with self._lock:
            if channel in self._reserved or channel in self._uncommitted:
                return None
            passage = self.current_passage(channel)
            row = next((row for row in self._commands.values() if row["channel"] == channel
                        and row["passage"] == passage and row["status"] == "pending"), None)
            if row is None:
                return None
            body = command_body(json.loads(row["body"]))
            self._set_command(row["id"], status="running")
            return dict(body, ok=True, status="running", measurementId=None, error=None)

    def reject_command_fast(self, request_id: str, error: str) -> None:
        """仅拒绝尚无冻结结果的命令；已有结果仍保持 running 等待原包提交。"""
        with self._lock:
            if any(item.command and item.command["requestId"] == request_id for item in self._reserved.values()):
                return
            row = self._commands.get(request_id)
            if row is not None and row["status"] != "succeeded":
                self._set_command(request_id, status="failed", error=error)

    def cancel_rejected(self, reservation: ResultReservation) -> None:
        """仅供明确业务拒绝、尚未提交的人工请求，不供保存失败换 ID。"""
        with self._lock:
            if reservation.command is None or self._reserved.get(reservation.channel) != reservation:
                raise BridgeConflict("只能取消本次尚未构造结果的人工请求")
            self._reserved.pop(reservation.channel)

    def record(self, result: dict, *, configuration_hash: str, source_mode: str,
               baseline_version_id=None, candidate_frame=None, candidate_result=None,
               command=None, archive_factory=None) -> Optional[str]:
        channel = result["channel"]
        replay = False
        replay_id = None
        with self._lock:
            original_command = None
            if command is not None:
                original_command = command_body({key: command[key] for key in
                    ("requestId", "channelId", "passageId", "kind", "confirmedEmpty")})
                accepted = self.command(original_command["requestId"])
                if not accepted or any(accepted[key] != value for key, value in original_command.items()):
                    raise BridgeConflict("命令没有受理记录或载荷已变化")
                if original_command["channelId"] != channel:
                    raise BridgeConflict("命令不属于本通道")
                if accepted["status"] == "succeeded":
                    return accepted["measurementId"]
                if accepted["status"] != "running":
                    raise BridgeConflict("命令尚未认领或已经终止")
            pending = self._uncommitted.get(channel)
            if pending is not None:
                if original_command is not None and pending[2] == original_command:
                    replay_id = pending[0]["measurementId"]
                    replay = True
                else:
                    raise BridgeConflict("本通道仍有原测量待补提交，禁止覆盖")
            else:
                passage = self.current_passage(channel)
                if not passage:
                    if command:
                        raise BridgeConflict("车辆连续性未确认")
                    result["integration"] = {"status": "quarantined", "error": "等待完整空闲帧确认车道"}
                    return None
                if command and command["passageId"] != passage:
                    raise BridgeConflict("车辆已变化，拒绝迟到命令")
                asset = None
                if command and command["kind"] == "baseline-capture":
                    if source_mode != "live" or not result["empty"] or result["frame_degraded"]:
                        raise BridgeConflict("候选基线只接受完整实车且本次判定为空车的结果")
                    asset = make_candidate(candidate_frame, candidate_result, confirmed_empty=True)
                measurement = "m-" + uuid.uuid4().hex
                payload = build_measurement(
                    result, measurement_id=measurement, node_id=self.config.node_id,
                    passage_id=passage, calibration_id=self.config.channels[channel].calibration_id,
                    algorithm_version=self.config.algorithm_version, configuration_hash=configuration_hash,
                    source_mode=source_mode, baseline_version_id=baseline_version_id,
                    baseline_candidate_sha256=hashlib.sha256(asset).hexdigest() if asset else None)
                # 身份由本次后台车次绑定；构造器只运行一次，失败重试不能重新采样。
                # 构造器只接收载荷副本，不能改动已确定的机器上报字段。
                archive = None
                if archive_factory is not None:
                    try:
                        from .cloud_archive import FrozenBundle
                        archive = archive_factory(json.loads(json.dumps(payload)))
                        if not isinstance(archive, FrozenBundle):
                            raise ValueError("显式要求归档时必须返回不可变完整包")
                        archive.validate()
                    except Exception as error:
                        # 冻结失败不能重跑构造器读到下一车；这类错误不按磁盘重试处理。
                        # 尚未持久提交的对象仍仅在 RAM，进程同时崩溃不承诺零丢失。
                        self._uncommitted[channel] = (payload, asset, original_command,
                                                       _ArchivePreparationFailure(type(error).__name__))
                        raise
                # 磁盘故障时保留本次原 ID/字节并暂停该通道采集；恢复后优先补提交。
                self._uncommitted[channel] = (payload, asset, original_command, archive)
        # ★ 补提交（outbox.save 的 fsync + 归档资产落库）挪到观测锁**外**
        #   （2026-09-23，两轮审查 M9）：原来 record -> flush_pending 全程持
        #   _lock，磁盘慢/满时四条通道的 observe / current_passage 一起被拖住
        #   —— 而 commit_reserved 早已确立"写盘不占车辆观测锁"的分工。
        #   save 对同一 ID 幂等，flush_pending 内部按 _uncommitted 身份收尾，
        #   锁外重放不会双写坏。
        self.flush_pending(channel)
        if replay:
            return replay_id
        with self._lock:
            result["integration"] = {"status": "queued", "measurementId": measurement,
                                     "passageId": passage, "baselineVersionId": baseline_version_id}
        self._wake.set()
        return measurement

    def flush_pending(self, channel: str) -> None:
        with self._lock:
            pending = self._uncommitted.get(channel)
            if pending is None:
                return
            if isinstance(pending[3], _ArchivePreparationFailure):
                raise BridgeConflict("原测量归档冻结失败，须核对原观测；禁止自动重采或更换测量 ID")
            payload, asset, command, archive = pending
        # 写盘在锁外（M9，模板同 commit_reserved）；同 ID 载荷幂等，重放安全。
        self.outbox.save(payload, candidate_bytes=asset, command=command, archive=archive)
        with self._lock:
            if self._uncommitted.get(channel) is not pending:
                return                  # 另一线程已完成同一份的收尾，幂等退出
            self._project_measurement(payload, command)
            self._uncommitted.pop(channel, None)
        self._wake.set()

    def _project_measurement(self, payload: dict, command: Optional[dict]) -> None:
        """结果索引可重建，绝不把旧车次补交结果更新到下一辆车。"""
        lane = self._lane(payload["channelId"])
        if lane["passage"] == payload["passageId"]:
            lane["measurement"] = payload["measurementId"]
        if command is not None:
            self._set_command(command["requestId"], status="succeeded", measurement=payload["measurementId"], error=None)

    def queue_command(self, body: dict) -> dict:
        body = command_body(body)
        encoded = json.dumps(body, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        # 持久完成凭据查询可能等 outbox 写盘，不能连带占住车辆观测锁。
        old = self.command(body["requestId"])
        if old:
            if any(old[key] != value for key, value in body.items()):
                raise BridgeConflict("相同 requestId 载荷不同")
            return old
        self._repair_completed_commands(body["channelId"])
        with self._command_submit_lock:
            old = self.command(body["requestId"])
            if old:
                if any(old[key] != value for key, value in body.items()):
                    raise BridgeConflict("相同 requestId 载荷不同")
                return old
            with self._lock:
                if body["channelId"] not in self.config.channels or not self.current_passage(body["channelId"]):
                    raise BridgeConflict("当前车道没有可确认的车辆")
                if self.current_passage(body["channelId"]) != body["passageId"]:
                    raise BridgeConflict("车辆已变化，拒绝迟到命令")
                if body["channelId"] in self._uncommitted or body["channelId"] in self._reserved:
                    raise BridgeConflict("该通道仍有原测量待补提交")
                if any(row["channel"] == body["channelId"] and row["status"] in ("pending", "running")
                       for row in self._commands.values()):
                    raise BridgeConflict("该通道已有待执行命令")
            with self._db_lock, self._db:
                self._db.execute("INSERT INTO commands(id,body,channel,passage,status) VALUES (?,?,?,?,'pending')",
                                 (body["requestId"], encoded, body["channelId"], body["passageId"]))
            with self._lock:
                row = dict(id=body["requestId"], body=encoded, channel=body["channelId"],
                           passage=body["passageId"], status="pending", measurement=None, error=None)
                self._commands[body["requestId"]] = row
                if (self.current_passage(body["channelId"]) != body["passageId"]
                        or body["channelId"] in self._reserved or body["channelId"] in self._uncommitted):
                    self._set_command(body["requestId"], status="failed", error="受理期间车辆或测量状态已变化，请核对")
        return self.command(body["requestId"])

    def command(self, request_id: str) -> Optional[dict]:
        receipt = self.outbox.command_result(request_id)
        with self._lock:
            cached = self._commands.get(request_id)
            row = dict(cached) if cached else None
        if row is None:
            with self._db_lock:
                row = self._db.execute("SELECT * FROM commands WHERE id=?", (request_id,)).fetchone()
        stored_body = None
        if row is not None:
            try:
                stored_body = command_body(json.loads(row["body"]))
            except (ValueError, TypeError, KeyError):
                raise RuntimeError("命令受理记录损坏，结果暂时无法确认") from None
        if receipt is not None:
            body, item = receipt["command"], receipt["measurement"]
            if (item.payload.get("nodeId") != self.config.node_id or
                    stored_body is not None and stored_body != body):
                raise RuntimeError("命令完成凭据与当前节点或受理记录不一致")
            return dict(body, ok=True, status="succeeded", measurementId=item.measurement_id, error=None)
        if not row:
            return None
        # 升级前明确保存了测量 ID 的成功记录可兼容；无关联记录不得猜测回填。
        if row["status"] == "succeeded":
            try:
                item = self.outbox.get(row["measurement"]) if row["measurement"] else None
                if item is not None:
                    self.outbox._validate_command_measurement(stored_body, item.payload, item.asset)
            except (ValueError, TypeError, KeyError):
                raise RuntimeError("旧命令成功记录损坏，结果暂时无法确认") from None
            if item is None or item.payload.get("nodeId") != self.config.node_id:
                raise RuntimeError("旧命令成功记录缺少有效测量，须核对历史")
        return dict(stored_body, ok=True, status=row["status"],
                    measurementId=row["measurement"], error=row["error"])

    def _repair_completed_commands(self, channel: str) -> None:
        with self._lock:
            rows = [dict(row) for row in self._commands.values() if row["channel"] == channel
                    and row["status"] in ("pending", "running")]
        for row in rows:
            result = self.command(row["id"])
            if result["status"] == "succeeded":
                with self._lock:
                    self._set_command(row["id"], status="succeeded", measurement=result["measurementId"], error=None)

    def claim_command(self, channel: str) -> Optional[dict]:
        self._repair_completed_commands(channel)
        return self.claim_command_fast(channel)

    def finish_command(self, request_id: str, measurement_id=None, error=None) -> None:
        result = self.command(request_id)
        with self._lock:
            if result is None:
                raise KeyError("命令不存在")
            if result["status"] == "succeeded":
                status, measurement_id, error = "succeeded", result["measurementId"], None
            elif any(pending[2] is not None and pending[2]["requestId"] == request_id
                     for pending in self._uncommitted.values()) or any(
                     reservation.command is not None and reservation.command["requestId"] == request_id
                     for reservation in self._reserved.values()):
                status, measurement_id, error = "running", None, "原测量待补提交，不会重新采集"
            else:
                status, measurement_id = "failed", None
                error = error or "命令缺少持久完成凭据，请核对历史"
            row = self._commands.get(request_id)
            if row is not None and row["status"] != "succeeded":
                self._set_command(request_id, status=status, measurement=measurement_id, error=error)

    @staticmethod
    def _same_measurement(record: dict, payload: dict) -> bool:
        actual = record.get("measurement", {}) if isinstance(record, dict) else {}
        if not isinstance(actual, dict):
            return False
        # .NET float 的最短 JSON 表示与 Python 展开的 float32 小数不同，按实际单精度比较。
        try:
            import numpy as np
            same_points = np.array_equal(np.asarray(actual.get("points"), dtype=np.float32),
                                         np.asarray(payload.get("points"), dtype=np.float32))
            time_keys = {"measuredAt", "measurementStartedAt", "measurementEndedAt"}
            same_time = all(datetime.fromisoformat(actual[key].replace("Z", "+00:00")) ==
                            datetime.fromisoformat(value.replace("Z", "+00:00"))
                            for key, value in payload.items() if key in time_keys)
            return same_points and same_time and all(actual.get(key) == value for key, value in payload.items()
                                                    if key not in time_keys | {"points"})
        except (ValueError, TypeError, KeyError):
            return False

    def sync_once(self) -> None:
        """网络失败保留原始 ID；409 进入人工处理，不换 ID 重发。"""
        for item in self.outbox.pending(limit=20):
            if self._quit.is_set():
                return
            try:
                receipt = self.client.post_measurement(item.payload)
                if not self._same_measurement(receipt, item.payload):
                    self.outbox.mark_blocked(item.measurement_id, "后台回执标识不匹配")
                    continue
                if item.asset is not None:
                    if self._quit.is_set():
                        return
                    self.client.upload_baseline(item.measurement_id,
                                                item.payload["baselineCandidateSha256"], item.asset)
                self.outbox.mark_sent(item.measurement_id)
            except HttpError as exc:
                if exc.retryable:
                    self.outbox.note_retry(item.measurement_id, exc)
                    # ★ 队头隔离（2026-09-23，两轮审查 M10）：
                    #   · 连接失败（status=None）→ 后台不可达，后续项大概率
                    #     同因，提前收手省掉一轮白打（原语义保留）；
                    #   · 后台**在线**却拒绝这一条（HTTP 429/5xx）→ 只把这一条
                    #     记账留在 pending，本轮继续发后面的项。原来无差别
                    #     raise：一个被后台持续 5xx 的"毒"测量会堵死整个发件箱
                    #     和基线同步段（_loaded 10 秒过期 -> 全通道 volume
                    #     不可用）。▲ 不升级成 blocked：那是终态，过载/瞬断
                    #     恢复后这条仍要自动发出，误升级等于丢数据。
                    if exc.status is None:
                        raise
                    continue
                else:
                    self.outbox.mark_blocked(item.measurement_id, exc)
        if self._quit.is_set():
            return
        versions = self.client.list_baselines() if any(c not in self._channel_contexts for c in self.config.channels) else []
        if not isinstance(versions, list):
            raise ValueError("后台基线清单格式错误")
        errors = []
        for channel, channel_config in self.config.channels.items():
            if self._quit.is_set():
                return
            if self._async_preparation and channel in self._channel_contexts:
                continue
            try:
                self._sync_channel(channel, channel_config, versions)
            except Exception as exc:
                with self._lock:
                    self._loaded.pop(channel, None)
                    self._prepared.pop(channel, None)
                errors.append(channel + ": " + (str(exc) if isinstance(exc, HttpError) else type(exc).__name__))
        self._error = "; ".join(errors)

    def _sync_channel(self, channel, channel_config, versions) -> None:
        if channel in self._channel_contexts:
            self._prepare_channel(channel, channel_config)
            return
        with self._lock:
            passage = self.current_passage(channel)
            lane = self._lane(channel)
            measurement = lane["measurement"] if lane else None
        if not passage or not measurement:
            return
        item = self.outbox.get(measurement)
        if not item or item.status != "sent":
            return
        receipt = self.client.get_measurement(measurement)
        if not self._same_measurement(receipt, item.payload):
            raise ValueError("后台关联回执标识不匹配")
        binding = receipt.get("binding")
        version = self.cache.select(versions, binding, self.config.node_id, channel,
                                    channel_config.calibration_id)
        if not version:
            with self._lock:
                self._loaded.pop(channel, None)
            return
        if self._quit.is_set():
            return
        with self._lock:
            previous = self._loaded.get(channel)
        if previous and previous[1] == version and previous[5] == item.payload["configurationHash"] and len(previous) > 7:
            baseline = previous[2]
            asset_bytes = previous[6] if len(previous) > 6 else None
            source = previous[7]
        else:
            origin = self.client.get_measurement(version["measurementId"])
            original = origin.get("measurement", {})
            expected = {"measurementId": version["measurementId"], "nodeId": self.config.node_id,
                        "channelId": "CH4", "calibrationId": version["calibrationId"],
                        "baselineCandidateSha256": version["assetSha256"],
                        "sourceMode": "live", "verdict": "empty", "frameDegraded": False}
            if any(original.get(key) != value for key, value in expected.items()):
                raise ValueError("基线来源测量或有效配置与当前通道不匹配")
            if self._quit.is_set():
                return
            asset_bytes = self.client.download_baseline(version["versionId"])
            baseline = self.cache.install(version, asset_bytes)
            source = json.dumps(dict(sourceChannelId="CH4", sourceCalibrationId=version["calibrationId"],
                sourceConfigurationHash=_hash(original.get("configurationHash"), "CH4 configurationHash")), sort_keys=True).encode("utf-8")
        with self._lock:
            if self.current_passage(channel) == passage:
                self._loaded[channel] = (passage, version, baseline, binding, time.monotonic(),
                                        item.payload["configurationHash"], asset_bytes, source)

    def _prepare_channel(self, channel, channel_config) -> None:
        """后台线程首测前查询；迟到响应只可刷新原 passage，不能附到下一车。"""
        with self._lock:
            passage = self.current_passage(channel)
            payload = self._starts.get(channel)
            if not passage or not payload or payload["passageId"] != passage:
                return
            payload = dict(payload)
        receipt = self.client.prepare_passage(payload)
        actual = receipt.get("start") if isinstance(receipt, dict) else None
        if (not isinstance(actual, dict) or
                any(actual.get(k) != v for k, v in payload.items() if k != "arrivedAt") or
                datetime.fromisoformat(actual.get("arrivedAt", "").replace("Z", "+00:00")) !=
                datetime.fromisoformat(payload["arrivedAt"].replace("Z", "+00:00"))):
            raise ValueError("到车回执上下文不一致")
        status, binding, version = receipt.get("status"), receipt.get("binding"), receipt.get("baseline")
        if status not in ("awaiting_plate", "plate_unavailable", "baseline_required", "ready"):
            raise ValueError("未知到车准备状态")
        if status in ("baseline_required", "ready"):
            key = hashlib.sha256(json.dumps([self.config.node_id, channel, passage],
                separators=(",", ":")).encode("utf-8")).hexdigest()
            workflow = hashlib.sha256(json.dumps(["load-radar-arrival", key],
                separators=(",", ":")).encode("utf-8")).hexdigest()
            if (not isinstance(binding, dict) or binding.get("passageKey") != key or
                    receipt.get("workflowId") != workflow or binding.get("workflowId") != workflow or
                    not binding.get("licensePlate") or binding.get("licensePlate") == "UNKNOWN" or
                    binding.get("vehicleIdentity") != "plate:" + binding["licensePlate"] or
                    not receipt.get("cameraIdentifier") or binding.get("cameraIdentifier") != receipt["cameraIdentifier"]):
                raise ValueError("到车回执权威身份不一致")
        elif binding is not None or version is not None:
            raise ValueError("未就绪回执不能携带可用基线或身份")
        loaded = None
        if status == "ready":
            selected = self.cache.select([version], binding, self.config.node_id, channel, channel_config.calibration_id)
            if selected is None:
                raise ValueError("准备回执基线不匹配本车")
            origin = self.client.get_measurement(selected["measurementId"]).get("measurement", {})
            expected = dict(measurementId=selected["measurementId"], nodeId=self.config.node_id,
                channelId="CH4", calibrationId=selected["calibrationId"],
                baselineCandidateSha256=selected["assetSha256"],
                sourceMode="live", verdict="empty", frameDegraded=False)
            if any(origin.get(k) != v for k, v in expected.items()):
                raise ValueError("计算前基线原测量与实际配置不一致")
            with self._lock:
                previous = self._loaded.get(channel)
            if previous and previous[1] == selected and previous[5] == payload["configurationHash"]:
                baseline, asset_bytes = previous[2], previous[6]
            else:
                asset_bytes = self.client.download_baseline(selected["versionId"])
                baseline = self.cache.install(selected, asset_bytes)
            source = json.dumps(dict(sourceChannelId="CH4", sourceCalibrationId=selected["calibrationId"],
                sourceConfigurationHash=_hash(origin.get("configurationHash"), "CH4 configurationHash")), sort_keys=True).encode("utf-8")
            loaded = (passage, selected, baseline, binding, time.monotonic(), payload["configurationHash"], asset_bytes, source)
        elif version is not None:
            raise ValueError("未准备好的基线不得用于计算")
        with self._lock:
            if self.current_passage(channel) == passage and self._starts.get(channel) == payload:
                self._prepared[channel] = (passage, status, binding, time.monotonic())
                if loaded is None:
                    self._loaded.pop(channel, None)
                else:
                    self._loaded[channel] = loaded

    def run(self) -> None:
        while not self._quit.is_set():
            try:
                self.sync_once()
            except Exception as exc:
                # 未知异常只显示类型，避免网络库和路径含密钥。
                self._error = str(exc) if isinstance(exc, HttpError) else "接口同步失败: " + type(exc).__name__
            self._wake.wait(self.config.retry_sec)
            self._wake.clear()

    def snapshot(self) -> dict:
        with self._lock:
            lanes = []
            for channel in self.config.channels:
                passage = self.current_passage(channel)
                row = self._lane(channel)
                loaded = self._loaded.get(channel)
                lanes.append({"channelId": channel, "passageId": passage,
                              "status": "occupied" if passage else ("idle" if row["ready"] else "quarantined"),
                              "lastMeasurementId": row["measurement"],
                              "baselineVersionId": loaded[1]["versionId"] if loaded and time.monotonic() - loaded[4] <= 10 else None})
            result = {"enabled": True, "nodeId": self.config.node_id,
                      "uncommittedChannels": sorted(set(self._uncommitted) | set(self._reserved)),
                      "error": self._error, "lanes": lanes}
        result["outbox"] = self.outbox.stats()
        return result

    def capacity_snapshot(self, *, refresh=False) -> dict:
        """独立维护入口；不调用 current_passage，不消费事件或修复车道。"""
        runtime = dict(status="unavailable", reason="state_lock_busy")
        if self._lock.acquire(timeout=.05):
            try:
                runtime = dict(status="sampled", sampled_at=time.time(),
                               reserved_count=len(self._reserved), uncommitted_count=len(self._uncommitted),
                               command_dirty_count=len(self._command_dirty), loaded_count=len(self._loaded),
                               closed=self._closed, baseline_authority="not_checked")
            finally:
                self._lock.release()
        return dict(runtime=runtime, passages=self._capacity.snapshot(refresh=refresh),
                    outbox=self.outbox.capacity_snapshot(refresh=refresh), consistency="separate_samples")

    def close(self) -> None:
        if self._closed:
            return
        self._quit.set()
        self._wake.set()
        self._standstill_wake.set()
        if self._standstill_thread is not None:
            self._standstill_thread.join(timeout=self.config.timeout_sec + 2)
        if self.is_alive():
            self.join(timeout=min(60, self.config.timeout_sec * 2 + 2))
        if self.is_alive():
            # 网络线程尚未退出时不得关闭正在使用的 SQLite 句柄。
            raise RuntimeError("接口同步线程未退出，待发队列已保留")
        self._projection_quit.set()
        self._projection_wake.set()
        self._projection_thread.join()
        with self._db_lock:
            self._db.close()
            self._capacity_db_closed = True
        self.outbox.close()
        self._lease.close()
        self._closed = True
