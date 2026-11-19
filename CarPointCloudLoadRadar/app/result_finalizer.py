"""冻结测量的后台收尾；采集不等待 ROI、SQLite 或文件投影。"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, fields
import hashlib
import json
import math
import threading
import time
import uuid
from typing import Optional, Tuple

import numpy as np

from .cloud_archive import ArchiveStore, FrozenBundle, freeze_bundle
from .cloud_observation import FrameSnapshot, ObservationWindow
from .cloud_roi import extract_observed_roi
from .detect import EmptyResult
from .geometry import Plane
from .load_radar_bridge import ResultReservation
from .load_radar_contract import build_measurement, make_candidate


class MeasurementRejected(ValueError):
    """可明确解释的人工业务拒绝；不同于已生成结果的归档失败。"""


class PreparedBudgetExceeded(ValueError):
    """持久最终结果永久超出收尾预算，原任务应保留并明确阻断。"""


def result_bytes(value) -> bytes:
    """诊断中的非有限量显式记为未知；点云仍由快照严格校验。"""
    def clean(item):
        if isinstance(item, dict):
            return {key: clean(value) for key, value in item.items()}
        if isinstance(item, (list, tuple)):
            return [clean(value) for value in item]
        if isinstance(item, (float, np.floating)):
            return float(item) if math.isfinite(item) else None
        if isinstance(item, np.integer):
            return int(item)
        return item
    return json.dumps(clean(value), ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"), allow_nan=False).encode("utf-8")


@dataclass(frozen=True)
class CandidateInput:
    """只冻结人工候选的输入；压缩和二次拟合在后台且只执行一次。"""
    frame: FrameSnapshot
    cloud: bytes
    planes: tuple
    dims: tuple
    yaw: float
    fit_ok: bool

    @classmethod
    def freeze(cls, frame, result):
        if result is None:
            raise ValueError("候选缺少六面拟合")
        return cls(FrameSnapshot.freeze(frame), np.asarray(result.cloud, "<f8").tobytes(),
                   tuple(tuple(getattr(plane, field.name) for field in fields(Plane))
                         for plane in result.planes), tuple(result.dims), float(result.yaw), bool(result.fit_ok))

    def build(self) -> bytes:
        result = EmptyResult(planes=[Plane(*row) for row in self.planes], dims=self.dims,
                             cloud=np.frombuffer(self.cloud, "<f8").reshape(-1, 3),
                             yaw=self.yaw, ok=self.fit_ok)
        return make_candidate(self.frame.frame(), result, confirmed_empty=True)


@dataclass(frozen=True)
class PendingResult:
    identity: ResultReservation
    result_json: bytes
    detection: FrameSnapshot
    detection_indices: bytes
    detection_rotation: bytes
    observation_window: Optional[ObservationWindow]
    volume_window: Optional[ObservationWindow]
    volume_extra_members: tuple
    extra_frames: Tuple[FrameSnapshot, ...]
    configuration_hash: str
    algorithm_version: str
    baseline_version_id: Optional[str]
    source_mode: str
    lane_seed: Optional[tuple]
    ground_z: float
    archive_enabled: bool
    candidate: Optional[CandidateInput] = None
    runtime_files: Tuple[Tuple[str, bytes], ...] = ()
    entry_frame_times: Tuple[float, ...] = ()
    exit_frame_times: Tuple[float, ...] = ()

    def byte_size(self) -> int:
        # 对共享 bytes 去重，但不把尚未构造的 bundle 当成免费内存。
        blobs = {}
        def visit(value):
            if isinstance(value, bytes):
                blobs[id(value)] = len(value)
            elif isinstance(value, tuple):
                for child in value:
                    visit(child)
            elif hasattr(value, "__dataclass_fields__"):
                for item in fields(value):
                    visit(getattr(value, item.name))
        visit(self)
        return sum(blobs.values()) * 4


@dataclass(frozen=True)
class PreparedResult:
    payload_json: Optional[bytes]
    asset: Optional[bytes]
    archive: Optional[FrozenBundle]


@dataclass(frozen=True)
class CompletedInput:
    """历史待计算任务已经持久构造最终字节；不再持有活动观测窗口。"""
    identity: ResultReservation
    result_json: bytes


def prepare_result(pending: PendingResult, bridge=None) -> PreparedResult:
    """纯构造一次；重试只接受返回的原字节，不重新读取通道状态。"""
    result = json.loads(pending.result_json)
    identity = pending.identity
    asset = None
    command = identity.command
    automatic = command is None and identity.channel == "CH4" and pending.candidate is not None
    if automatic or command and command["kind"] == "baseline-capture":
        if (pending.source_mode != "live" or result["empty"] is not True
                or result["frame_degraded"] or pending.candidate is None):
            raise ValueError("候选基线只接受人工确认的完整实车空车结果")
        asset = pending.candidate.build()
        if automatic:
            result["automatic_baseline"] = True
    payload = None
    if bridge is not None and identity.passage_id is not None:
        # 使用本次冻结观测窗的真实帧时间，重试不能改成当前时间。
        window = pending.observation_window
        if window is not None:
            frame_times = [pair.raw.ts for pair in window.pairs]
            if pending.volume_window is not None:
                frame_times.extend(pair.raw.ts for pair in pending.volume_window.pairs)
            frame_times.extend(frame.ts for frame in pending.extra_frames)
            frame_times.extend(pending.entry_frame_times)
            frame_times.extend(pending.exit_frame_times)
            result["measurement_started_at"] = min(frame_times)
            result["measurement_ended_at"] = max(frame_times)
        payload = build_measurement(result, measurement_id=identity.measurement_id,
            node_id=bridge.config.node_id, passage_id=identity.passage_id,
            calibration_id=bridge.config.channels[identity.channel].calibration_id,
            algorithm_version=pending.algorithm_version, configuration_hash=pending.configuration_hash,
            source_mode=pending.source_mode, baseline_version_id=pending.baseline_version_id,
            baseline_candidate_sha256=hashlib.sha256(asset).hexdigest() if asset else None)
    archive = None
    if pending.archive_enabled:
        window = pending.observation_window
        if window is None:
            raise ValueError("归档缺少真实原观测，禁止用判定云替代")
        raw = window.pairs[-1].raw.frame()
        roi = extract_observed_roi(raw, pending.lane_seed, pending.ground_z)
        archive = freeze_bundle(measurement_id=identity.measurement_id, channel=identity.channel,
            passage_id=identity.passage_id, observation=raw, detection=pending.detection.frame(),
            vehicle_indices=roi.vehicle_indices, carriage_indices=roi.carriage_indices,
            result=result, roi=roi.report, configuration_hash=pending.configuration_hash,
            algorithm_version=pending.algorithm_version, complete=True,
            measurable=not result["frame_degraded"], baseline_version_id=pending.baseline_version_id,
            source_mode=pending.source_mode, extra_frames=tuple(f.frame() for f in pending.extra_frames),
            observation_window=window,
            detection_indices=np.frombuffer(pending.detection_indices, "<i8"),
            detection_rotation=np.frombuffer(pending.detection_rotation, "<f8").reshape(3, 3),
            volume_window=pending.volume_window,
            volume_extra_members=pending.volume_extra_members if pending.volume_window else None,
            runtime_files=pending.runtime_files)
    return PreparedResult(result_bytes(payload) if payload is not None else None, asset, archive)


class ResultFinalizer:
    """每通道一个未持久槽位，共享内存预算；提交与导出分线程。"""

    # 连续失败多少次开始挂常驻告警（约 0.1+0.2+...≈3 秒之后）。
    ALERT_AFTER_ATTEMPTS = 5
    # 导出外层连续失败多少次开始挂常驻告警。导出循环失败后同样退避，
    # 5 次也是约 3 秒 —— 与提交侧保持同一个时间尺度。
    EXPORT_ALERT_AFTER = 5

    def __init__(self, channels: dict, bridge=None, byte_budget: int = 512 * 1024 * 1024):
        # channels: channel -> (显式输出目录, 是否允许归档恢复)
        # ★ byte_budget 是**每条车道**的在途结果预算，不是全局的（2026-09-23）。
        #   每条道同一时刻最多一份在途结果（_slots 以车道为键），总占用天然 <= 车道数 x 预算，
        #   内存照样有界。原来按全局求和：真实整帧每份测量预留约 196 MB，4 条道同时出结果时
        #   第 3、4 条道被别的车道的在途结果挡住 —— 推迟测量、丢掉核对 ROI 用的整帧。
        #   车道之间不许因为共享预算互相干扰。钉住：
        #   tests/test_result_finalizer.py::TestResultFinalizer::test_four_lanes_reserve_concurrently_without_interference
        self.channels = dict(channels)
        self.bridge = bridge
        self.byte_budget = byte_budget
        self._lock = threading.RLock()
        self._slots = {}
        self._events = {channel: deque() for channel in channels}
        # ★ 常驻告警，不是一次性事件。事件被 poll 一次就没了，而"这个通道
        #   现在测不了"必须一直挂着 —— 否则现场只会看到卡片停在"判定中"。
        self._alerts = {channel: "" for channel in channels}
        # ★ 导出侧**另开一个槽**，不能与上面共用：`_commit_loop` 每成功提交
        #   一份就会把 `_alerts[channel]` 清成空串，而导出可能还在持续失败 ——
        #   共用的话那次成功提交会顺手把导出告警一起抹掉。两边分开存、
        #   在 alert() 里合并显示，与 service._health_text 同一做法。
        self._export_alerts = {channel: "" for channel in channels}
        self._tracked = {channel: deque(maxlen=500) for channel in channels}
        self._stores = {}
        self._store_lock = threading.Lock()
        self._wake = threading.Event()
        self._export_stop = threading.Event()
        self._closing = False
        self._closed = False
        self._threads = []

    def start(self):
        with self._lock:
            if self._threads or self._closed:
                return
            for channel in self.channels:
                for target in (self._commit_loop, self._export_loop):
                    thread = threading.Thread(target=target, args=(channel,), daemon=True,
                                              name="result-%s-%s" % (target.__name__, channel))
                    self._threads.append(thread)
                    thread.start()

    def can_accept(self, channel: str, estimate: int = 0) -> bool:
        """这条车道能不能再接一份测量。**只看本车道自己**：别的车道有没有在途结果、
        占了多少预算，都不影响这里的答案（车道隔离，见 __init__ 的说明）。"""
        with self._lock:
            return (not self._closing and channel not in self._slots
                    and len(self._events[channel]) < 8
                    and estimate <= self.byte_budget)

    def reserve(self, channel: str, token: str, passage: Optional[str], estimate: int,
                command=None) -> ResultReservation:
        with self._lock:
            if not self.can_accept(channel, estimate):
                raise RuntimeError("原测量未持久化或内存预算已满，暂停新测量但继续采集")
            identity = (self.bridge.reserve_result(channel, token, command) if self.bridge is not None else
                        ResultReservation("m-" + uuid.uuid4().hex, channel, passage, token))
            self._slots[channel] = dict(identity=identity, pending=None, prepared=None,
                                        bytes=estimate, state="captured", error="", attempts=0, retry_at=0.)
            self._tracked[channel].append(identity.measurement_id)
            return identity

    def submit(self, pending: PendingResult):
        with self._lock:
            row = self._slots[pending.identity.channel]
            if row["identity"] != pending.identity or row["pending"] is not None:
                raise ValueError("冻结结果不属于原预留槽位")
            row.update(pending=pending, bytes=max(row["bytes"], pending.byte_size()), state="ready_to_prepare")
            row["budget_overrun"] = row["bytes"] > self.byte_budget
        self.start()
        self._wake.set()

    def submit_prepared(self, identity: ResultReservation, result_json: bytes, prepared: PreparedResult):
        """接收独立任务已经持久保存的最终字节，提交失败只重放这些字节。"""
        size = len(result_json) + len(prepared.payload_json or b"") + len(prepared.asset or b"")
        if prepared.archive is not None:
            size += len(prepared.archive.manifest) + sum(len(data) for _, data in prepared.archive.files)
        estimate = size * 4
        if estimate > self.byte_budget:
            raise PreparedBudgetExceeded("历史最终结果超过本通道收尾内存预算，原任务保留待处理")
        with self._lock:
            if not self.can_accept(identity.channel, estimate):
                return False
            if self.bridge is None or prepared.payload_json is None:
                raise ValueError("历史任务必须有正式上报身份及完整载荷")
            self.bridge.adopt_deferred(identity)
            self._slots[identity.channel] = dict(identity=identity,
                pending=CompletedInput(identity, result_json), prepared=prepared, bytes=estimate,
                state="committing", error="", attempts=0, retry_at=0.)
            self._tracked[identity.channel].append(identity.measurement_id)
        self.start()
        self._wake.set()
        return True

    def restore_durable(self, identity, result_json, prepared):
        """调用方已逐字核对原outbox；恢复通知不重新计算或重新发送。"""
        with self._lock:
            if len(self._events[identity.channel]) >= 8:
                return False
            self._tracked[identity.channel].append(identity.measurement_id)
            self._event(identity.channel, dict(kind="durable", identity=identity,
                result_json=result_json, archived=prepared.archive is not None, managed=True))
        return True

    def reject(self, identity: ResultReservation, error: MeasurementRejected):
        with self._lock:
            row = self._slots[identity.channel]
            if row["identity"] != identity or row["pending"] is not None or identity.command is None:
                raise ValueError("仅允许取消未提交结果的人工业务拒绝")
            self.bridge.cancel_rejected(identity)
            self._slots.pop(identity.channel)
        self.bridge.reject_command_fast(identity.command["requestId"], str(error))

    def capture_failed(self, identity: ResultReservation, error: Exception, detail: str = ""):
        """冻结阶段就失败了 —— 一个结果字节都不存在，**释放槽位**。

        ▲ 必须与提交失败分开。提交失败时结果已经冻结，要原字节重放、
          绝不能换 ID（下面 `_commit_loop` 的 `commit_pending` 分支）；
          这里 outbox 里根本没有这个 measurement_id，放弃它不会让任何
          一辆车留下两条记录，而本会话的检测计数由 `service` 侧保留，
          这辆车也不会被重测。
        ▲ 不释放的后果实测过（2026-09-18）：槽位卡在 preparation_failed，
          全项目没有一处会 pop 掉它 -> `can_accept` 恒 False ->
          `service._tick` 的沉降窗口门控恒不通过 -> **该通道此后每辆车
          都不测、不落盘**，而 health 横幅与"停稳 30s"告警都不触发
          （车是判成 STABLE 的），两个前端也都不渲染 finalizer.state ——
          除了开头那一条会被刷走的 error 事件之外完全静默。
          钉住：`tests/test_result_finalizer.py::TestPreparationFailure`。
        """
        reason = "原结果冻结失败: " + type(error).__name__ + (("（%s）" % detail) if detail else "")
        with self._lock:
            row = self._slots.get(identity.channel)
            if row is None or row["identity"] != identity:
                return
            if row["pending"] is not None:
                # 已经冻结出结果了，不走这条路 —— 交给重放分支，不许换 ID。
                row.update(state="commit_pending", error=type(error).__name__)
                return
            self._slots.pop(identity.channel)
            self._alerts[identity.channel] = reason
            self._event(identity.channel, dict(kind="error", measurement_id=identity.measurement_id,
                                               error=reason))
        if self.bridge is not None:
            try:
                self.bridge.abandon_reservation(identity)
            except Exception as exc:                          # noqa: BLE001
                # 放不掉就只能让后台那边的预留继续占着，但**必须留下痕迹**，
                # 不能像从前那样静默。此时 reserve 会抛 BridgeConflict。
                with self._lock:
                    self._alerts[identity.channel] = (
                        reason + "；接口预留未能释放（%s），该通道需人工介入" % type(exc).__name__)

    def alert(self, channel: str) -> str:
        """常驻告警文本；空串表示这个通道没有待人工处理的事。

        提交侧与导出侧两个槽合并 —— 分开存的理由见 `_export_alerts` 的注释。
        """
        with self._lock:
            parts = (self._alerts.get(channel, ""), self._export_alerts.get(channel, ""))
        return "；".join(part for part in parts if part)

    def _event(self, channel, event):
        with self._lock:
            self._events[channel].append(event)

    def poll(self, channel):
        with self._lock:
            events = list(self._events[channel])
            self._events[channel].clear()
            return events

    def status(self, channel):
        with self._lock:
            row = self._slots.get(channel)
            return ({key: row[key] for key in ("state", "error", "attempts", "bytes")}
                    if row else {"state": "idle", "error": ""})

    def capacity_snapshot(self, *, refresh=False) -> dict:
        """短锁只复制状态；不消费事件，不懒建库，不持状态锁请求数据库摘要。"""
        runtime = dict(status="unavailable", reason="state_lock_busy")
        if self._lock.acquire(timeout=.05):
            try:
                states = {}
                for row in self._slots.values():
                    states[row["state"]] = states.get(row["state"], 0) + 1
                runtime = dict(status="sampled", sampled_at=time.time(), slot_count=len(self._slots),
                               estimated_pending_bytes=sum(row["bytes"] for row in self._slots.values()),
                               byte_budget=self.byte_budget, budget_scope="per_channel",
                               channel_count=len(self.channels), states=states,
                               event_count=sum(len(events) for events in self._events.values()),
                               closing=self._closing, closed=self._closed)
            finally:
                self._lock.release()
        stores = None
        if self._store_lock.acquire(timeout=.05):
            try:
                stores = list(self._stores.items())
            finally:
                self._store_lock.release()
        databases = (dict(status="unavailable", reason="store_lock_busy") if stores is None
                     else dict(status="opened" if stores else "not_opened",
                               stores={path: store.capacity_snapshot(refresh=refresh) for path, store in stores}))
        return dict(runtime=runtime, archive_databases=databases, consistency="separate_samples")

    def _store(self, output_dir):
        with self._store_lock:
            if output_dir not in self._stores:
                self._stores[output_dir] = ArchiveStore(output_dir)
            return self._stores[output_dir]

    def _commit_loop(self, channel):
        while not self._export_stop.is_set():
            with self._lock:
                row = self._slots.get(channel)
                # 槽位的状态字段只在持锁时读写；status()/capacity_snapshot()
                # 也持锁读同一批键，两边必须是同一套纪律。
                ready = bool(row) and row["pending"] is not None
                waiting = ready and time.perf_counter() < row["retry_at"]
                if ready and not waiting:
                    pending, prepared = row["pending"], row["prepared"]
                    row["state"] = "preparing" if prepared is None else "committing"
            if not ready or waiting:
                self._wake.wait(.05)
                self._wake.clear()
                continue
            if prepared is None:
                try:
                    prepared = prepare_result(pending, self.bridge)
                except Exception as error:
                    # ★ 构造失败**照样要退避重放**，和提交失败一个待遇。
                    #   prepare_result 是 pending 的纯函数，重试用的是同一份
                    #   冻结字节、同一个 measurement_id，不存在换 ID 的问题。
                    #   从前这里调 capture_failed 把槽位钉成 preparation_failed
                    #   而没有任何一处会 pop 掉它 —— 该通道之后每辆车都不再测
                    #   （2026-09-18 复现）。瞬时原因（内存、磁盘、ROI 抢不到）
                    #   重试就能过；确定性原因则由 attempts 与常驻告警暴露出来，
                    #   不再是静默的。
                    self._retry_later(channel, pending.identity, error, "构造")
                    continue
                with self._lock:
                    row = self._slots.get(channel)
                    if row is None or row["identity"] != pending.identity:
                        continue                      # 槽位已经换人，丢弃这次构造
                    row["prepared"] = prepared
                    row["state"] = "committing"
                # ★ 这里**不能**留下第二个指向槽位的局部名。槽位里挂着整份
                #   冻结点云，线程空转时栈上多一个引用就是多留一份内存 ——
                #   `test_durable_slot_releases_large_frozen_inputs_while_thread_idle`
                #   用弱引用盯着这件事。所以复用 `row`，由末尾的 del 统一清。
            try:
                if prepared.payload_json is not None:
                    self.bridge.commit_reserved(pending.identity, json.loads(prepared.payload_json),
                                                asset=prepared.asset, archive=prepared.archive)
                else:
                    if prepared.archive is not None:
                        self._store(self.channels[channel][0]).save(prepared.archive)
                    if self.bridge is not None:
                        self.bridge.release_quarantined(pending.identity)
                with self._lock:
                    # 先发布事件再释放槽位；调用者不能在此间丢失已持久结果。
                    self._event(channel, dict(kind="durable", identity=pending.identity,
                        result_json=pending.result_json, archived=prepared.archive is not None,
                        managed=prepared.payload_json is not None))
                    done = self._slots.get(channel)
                    if done is not None and done.get("exported_path"):
                        self._event(channel, dict(kind="exported", measurement_id=pending.identity.measurement_id,
                                                  path=done["exported_path"]))
                    self._slots.pop(channel, None)
                    # 这一份走通了，通道恢复正常 —— 常驻告警该撤掉。
                    self._alerts[channel] = ""
                    del done
                # 成功后的 idle wait 不能继续用线程栈保留上一份完整点云/包。
                del pending, prepared, row
            except Exception as error:
                # 状态可轮询；不每次失败追加无限队列。
                self._retry_later(channel, pending.identity, error, "提交")

    def _retry_later(self, channel, identity, error, stage: str):
        """构造/提交失败统一退避重放；**始终原 ID、原字节**，不换测量。

        连续失败到一定次数就挂常驻告警 —— 槽位占着不放是**有意的背压**
        （结果还在内存里，丢掉就是丢数据），但现场必须看得见它卡住了。
        """
        with self._lock:
            row = self._slots.get(channel)
            if row is None or row["identity"] != identity:
                return
            row["attempts"] += 1
            # ★ 必须把**异常消息**也带出来（2026-09-21 现场）。
            #   在此之前这里只留 type(error).__name__，于是现场看到的是
            #   「构造失败（ValueError）」—— 而 freeze_bundle 一条路上有 20 多处
            #   `raise ValueError(...)`，每一条对应完全不同的处置。实测卡住时
            #   除了重启没有别的办法定位，日志/看板/API 三处都拿不到原因。
            detail = " ".join(str(error).split())[:200]
            row.update(state="commit_pending", error=type(error).__name__, detail=detail,
                       retry_at=time.perf_counter() + min(30., .1 * 2 ** min(row["attempts"], 8)))
            if row["attempts"] >= self.ALERT_AFTER_ATTEMPTS:
                self._alerts[channel] = (
                    "%s结果%s失败已重试 %d 次（%s：%s），该通道暂停新测量但仍在采集。"
                    "结果仍在内存中按原 ID 重放，不会换 ID；请查磁盘空间、"
                    "接口连通性与归档库状态。" % (identity.measurement_id, stage,
                                                 row["attempts"], type(error).__name__,
                                                 detail or "无消息"))

    def _export_one(self, journal, measurement, output_dir, channel):
        """导出一份并把路径回写/发事件。**成功返回 None，失败返回异常本身。**

        从 `_export_loop` 里抽出来的（2026-09-18 重构）：那个循环原来嵌套
        **8 层**（while > try > for > for > if > try > with > if），
        最里面那两层才是真正干活的。逻辑逐条未变。

        ★ 2026-09-22：原来是 `except Exception: return False`，异常被整个丢掉。
          于是"输出目录只读 / 权限不足 / 磁盘满"这一类故障的表现是：
          每 2 秒重试一份、一份都导不出去，而 alert/status/事件三处全是空的
          （实测 108 次 / 4 秒，一点痕迹都没有）。这与 `_export_loop` 外层
          那条路径是**同一个签名**，所以两条一起修。返回异常而不是 False，
          是为了让告警能把原因带出去 —— 只报"失败了"现场照样无从下手。
        """
        try:
            path = journal.export(measurement, output_dir, channel=channel)
        except Exception as error:                           # noqa: BLE001
            return error
        with self._lock:
            row = self._slots.get(channel)
            if row and row["identity"].measurement_id == measurement:
                row["exported_path"] = path
            elif measurement in self._tracked[channel]:
                self._event(channel, dict(kind="exported", measurement_id=measurement, path=path))
        return None

    def _export_journal(self, journal, output_dir, channel, delays):
        """把一本日志里待导出的批次走一遍。`delays` 同时兼作游标与退避表。

        返回 `(成功几份, 最后一次失败的异常或 None)` —— 调用方据此判断
        "这一轮到底有没有进展"。只看失败不看成功是不行的：毒包与
        "全都导不出去"是两回事，前者不该报警（游标会轮过去，其余照常导）。
        """
        # 游标轮转见 pending_after：毒包不会永久占据队首。
        cursor = delays.get(id(journal), "")
        exported, failure = 0, None
        for measurement in journal.pending_after(channel, cursor, limit=8):
            delays[id(journal)] = measurement
            if self._export_stop.is_set():
                break
            key = (id(journal), measurement)
            if time.perf_counter() < delays.get(key, 0):
                continue                      # 还在退避窗口里
            error = self._export_one(journal, measurement, output_dir, channel)
            if error is None:
                delays.pop(key, None)
                exported += 1
            else:
                delays[key] = time.perf_counter() + 2.
                failure = error
        return exported, failure

    def _export_loop(self, channel):
        output_dir, enabled = self.channels[channel]
        if not enabled:
            return
        delays = {}
        fails, backoff = 0, 0.
        while not self._export_stop.is_set():
            # ★ 两条失败路径合到一个计数器上：
            #   ① 外层 —— 归档库打不开 / 查询失败（原来是 `except: pass`）；
            #   ② 每一份 —— journal.export 落盘失败（原来是 `except: return False`）。
            #   两条**都曾完全无声**（实测 28 次/3 秒 与 108 次/4 秒，
            #   而 alert 空、status 报 idle、事件队列也空）。见 docs/54 §1。
            exported, failure, stage = 0, None, ""
            try:
                journals = [self._store(output_dir).journal]
                if self.bridge is not None:
                    journals.append(self.bridge.outbox.archives)
                for journal in journals:
                    done, error = self._export_journal(journal, output_dir, channel, delays)
                    exported += done
                    if error is not None and failure is None:
                        failure, stage = error, "导出落盘"
            except Exception as error:                       # noqa: BLE001
                # 初始化/查询失败与文件失败都不影响采集；后续重试持久记录。
                failure, stage, exported = error, "打开归档库或查询待导出", 0
            # ★ 只要这一轮**导成了一份**就不算"一份都导不出去" —— 单份毒包由
            #   pending_after 的游标轮过去，其余照常导，那不是要惊动人的事。
            if exported:
                failure = None
            if failure is None:
                if fails:
                    fails, backoff = 0, 0.
                    with self._lock:
                        self._export_alerts[channel] = ""
            else:
                fails += 1
                backoff = min(30., .1 * 2 ** min(fails, 8))
                if fails >= self.EXPORT_ALERT_AFTER:
                    detail = " ".join(str(failure).split())[:200]
                    text = ("归档导出连续失败 %d 次（%s时 %s：%s）。测量仍在采集与提交，"
                            "但一份都导不出去；请查磁盘空间、输出目录权限与归档库文件。"
                            % (fails, stage, type(failure).__name__, detail or "无消息"))
                    with self._lock:
                        first = not self._export_alerts[channel]
                        self._export_alerts[channel] = text
                        # 事件**只在这一轮失败的开头发一次**，之后靠常驻告警挂着。
                        # 每次失败都发的话，can_accept 的 len(events) < 8 会把整条
                        # 通道堵死 —— 那是比无声更糟的后果。
                        if first:
                            self._event(channel, dict(kind="error", measurement_id="", error=text))
            # 失败时退避，有进展时保持原来的 0.1 秒节奏（导出延迟一个字没变）。
            self._export_stop.wait(backoff or .1)

    def close(self, timeout: Optional[float] = 3.) -> bool:
        """RAM 尚未持久时不关闭连接；调用者必须保留本对象和进程所有权。"""
        deadline = None if timeout is None else time.perf_counter() + max(0., timeout)
        with self._lock:
            self._closing = True
        while True:
            with self._lock:
                pending = bool(self._slots)
            if not pending:
                break
            if deadline is not None and time.perf_counter() >= deadline:
                return False
            self._wake.wait(.05)
            self._wake.clear()
        self._export_stop.set()
        self._wake.set()
        for thread in self._threads:
            thread.join(None if deadline is None else max(0., deadline - time.perf_counter()))
        if any(thread.is_alive() for thread in self._threads):
            return False
        with self._store_lock:
            for store in self._stores.values():
                store.close()
        self._closed = True
        return True
