"""停稳留样与迟到识牌分离；每车道独立落盘/准备线程，最终结果只交接原字节。"""
from __future__ import annotations

import base64
from collections import deque
import json
import sys
import threading
import time
import uuid

from .cloud_archive import FrozenBundle
from .frozen_detection import FrozenDetection, calculate_frozen, inspect_frozen
from .load_radar_bridge import BridgeConflict
from .load_radar_transport import HttpError, _json_bytes
from .pending_measurements import CompletionBusy, MAX_PREPARED_TASK_BYTES, PendingTaskStore
from .result_finalizer import PreparedBudgetExceeded, PreparedResult, prepare_result, result_bytes

MAX_STAGED_BYTES = MAX_PREPARED_TASK_BYTES


def _stage_timing(channel, measurement_id, stage, info, **extra):
    """分阶段耗时只写 stderr（桌面模式进 logs/desktop.log），不改变留样、计算或提交。"""
    try:
        since = time.time() - float(info["measured_at"])
        details = "".join(f" {key}={value}" for key, value in extra.items())
        print(f"[stage-timing] {time.strftime('%Y-%m-%d %H:%M:%S')} channel={channel} "
              f"measurementId={measurement_id} stage={stage} sinceFrozen={since:.2f}s{details}",
              file=sys.stderr, flush=True)
    except Exception:                                        # noqa: BLE001
        pass                                                # 计时失败不影响业务


def _encode(value):
    return None if value is None else base64.b64encode(value).decode("ascii")


def _decode(value):
    return None if value is None else base64.b64decode(value, validate=True)


def pack_prepared(result_json, prepared):
    archive = prepared.archive
    body = dict(schema="jt128-prepared-job-v1", result=_encode(result_json),
        payload=_encode(prepared.payload_json), asset=_encode(prepared.asset),
        archive=None if archive is None else dict(manifest=_encode(archive.manifest),
            files=[[name, _encode(data)] for name, data in archive.files]))
    encoded = result_bytes(body)
    if len(encoded) > MAX_STAGED_BYTES:
        raise ValueError("最终任务字节超过持久化预算")
    return encoded


def unpack_prepared(data):
    if not isinstance(data, bytes) or not 0 < len(data) <= MAX_STAGED_BYTES:
        raise ValueError("最终任务字节无效或超过预算")
    body = json.loads(data)
    if (set(body) != {"schema", "result", "payload", "asset", "archive"} or
            body["schema"] != "jt128-prepared-job-v1" or result_bytes(body) != data):
        raise ValueError("最终任务格式或规范字节不符")
    archive = body["archive"]
    if archive is not None:
        if set(archive) != {"manifest", "files"}:
            raise ValueError("最终归档格式不符")
        archive = FrozenBundle(_decode(archive["manifest"]),
            tuple((name, _decode(value)) for name, value in archive["files"]))
        archive.validate()
    result, payload = _decode(body["result"]), _decode(body["payload"])
    if result is None or payload is None:
        raise ValueError("完整任务缺少结果或上报原字节")
    json.loads(result)
    json.loads(payload)
    return result, PreparedResult(payload, _decode(body["asset"]), archive)


class DeferredMeasurements:
    """网络/计算不持采集锁；每通道最多一项准备计算，上传仍在原bridge线程。"""

    def __init__(self, bridge, finalizer, channels, *, store=None):
        self.bridge, self.finalizer = bridge, finalizer
        self.channels = tuple(channels)
        self.store = store or PendingTaskStore(bridge.config.storage_dir, self.channels)
        self._lock = threading.RLock()
        self._halt = threading.Event()
        self._ingress_stop = threading.Event()
        self._wake = {channel: threading.Event() for channel in self.channels}
        # 处理线程的唤醒：新留样落盘即可开始准备/计算，不等固定轮询间隔。
        self._process_wake = {channel: threading.Event() for channel in self.channels}
        self._ingress = {channel: deque() for channel in self.channels}
        self._queued_bytes = {channel: 0 for channel in self.channels}
        self._passages = {channel: {} for channel in self.channels}
        self._stats = {channel: self.store.snapshot(channel) for channel in self.channels}
        self._durable_captures = {channel: None for channel in self.channels}
        self._handed = set()
        self._infos = {}
        # 离场帧迟到附着：测后采集、识牌前交给体积；未采到则空元组，不拦计算。
        self._exit_volume = {}
        from .recalculation import RecalculationProcessor
        self.recalculation = RecalculationProcessor(bridge, self.store)
        self._threads = []
        self._closed = False
        self.bridge._async_preparation = True

    def offer_exit(self, measurement_id, frames):
        """通道线程在离场缓冲满/落盘时调用；覆盖同 MID 的旧副本。"""
        if not measurement_id or not frames:
            return
        with self._lock:
            self._exit_volume[measurement_id] = tuple(frames)

    def take_exit(self, measurement_id):
        with self._lock:
            return self._exit_volume.pop(measurement_id, ())

    def can_accept(self, channel, estimate=0):
        with self._lock:
            stats = self._stats[channel]
            return (not self._ingress_stop.is_set() and not self._closed and
                stats["pending_jobs"] + len(self._ingress[channel]) < stats["max_jobs_per_channel"] and
                stats["pending_bytes"] + self._queued_bytes[channel] + estimate <= stats["max_bytes_per_channel"])

    def capture(self, worker, frame, stab):
        identity, start = self.bridge.capture_identity(worker.ch.id, uuid.uuid4().hex)
        if identity.command_json is not None:
            raise ValueError("异步自动采集入口不接收人工命令")
        channel = identity.channel
        with self._lock:
            prior = self._passages[channel].get(identity.passage_id)
        if prior is not None:
            worker._detected_this_session += 1
            worker._current_result_token = prior.origin_token
            worker._last_save_name = prior.measurement_id
            return prior.measurement_id
        data = FrozenDetection.capture(worker, frame, stab, identity, start)
        size = data.byte_size() + len(start)
        with self._lock:
            if not self.can_accept(channel, size):
                raise RuntimeError("本通道待计算留样容量已满，不能替换原任务")
            self._ingress[channel].append((identity, start, data, size))
            self._queued_bytes[channel] += size
            self._passages[channel][identity.passage_id] = identity
            # 历史passage不会复用；只限制内存去重索引，不清任何持久任务。
            if len(self._passages[channel]) > 512:
                self._passages[channel].pop(next(iter(self._passages[channel])))
            self._stats[channel]["last_capture"] = "persisting"
        worker._detected_this_session += 1
        worker._current_result_token = identity.origin_token
        worker._last_save_name = identity.measurement_id
        worker.bus.publish("capture_pending", dict(channel=channel, measurement_id=identity.measurement_id,
            passage_id=identity.passage_id, state="persisting"))
        try:
            logger = getattr(worker, "_runtime_record", None)
            if callable(logger):
                logger("measurement_queued", "本车观测已进入留样队列，等待持久化与后台识牌计算",
                       ts=getattr(frame, "ts", None), session=getattr(worker, "_session_id", 0),
                       identity=identity.measurement_id)
        except Exception:                                    # noqa: BLE001
            pass                                            # 日志不能改变原留样队列的交接
        self._wake[channel].set()
        return identity.measurement_id

    def _refresh(self, channel, **extra):
        stats = self.store.snapshot(channel)
        with self._lock:
            # 落盘与计算是两个阶段，刷新队列计数不能抹掉最近已留样状态。
            self._stats[channel] = dict(self._stats[channel], **stats, **extra)

    def persist_once(self, channel):
        """独立writer的单步；磁盘失败原项仍在内存，不能换ID或读下一帧替代。"""
        with self._lock:
            item = self._ingress[channel][0] if self._ingress[channel] else None
        if item is None:
            return False
        identity, start, data, size = item
        if not isinstance(data, bytes):
            data = data.encode()
            with self._lock:
                self._ingress[channel][0] = (identity, start, data, size)
        with self._lock:
            info = self._infos.get(identity.measurement_id)
        if info is None:
            info = inspect_frozen(data)
            with self._lock:
                self._infos[identity.measurement_id] = info
        original = self.store.put(identity, start, data)
        if original.identity != identity:
            raise ValueError("原车次已有不同测量身份，候选保留待核实")
        with self._lock:
            self._ingress[channel].popleft()
            self._queued_bytes[channel] -= size
            self._durable_captures[channel] = identity
        self._refresh(channel, last_capture="captured", last_measurement_id=identity.measurement_id,
                      capture_error="")
        _stage_timing(channel, identity.measurement_id, "persisted", info)
        self._process_wake[channel].set()
        return True

    def process_once(self, channel):
        rows = self.store.due(channel, limit=1)
        if not rows:
            return False
        row = rows[0]
        identity = row.identity
        try:
            existing = self.bridge.outbox.get(identity.measurement_id)
            if existing is not None and row.prepared_bytes is None:
                raise ValueError("原ID已有outbox但任务缺最终字节，禁止重算")
            if row.prepared_bytes is None:
                with self._lock:
                    info = self._infos.get(identity.measurement_id)
                if info is None:
                    info = inspect_frozen(row.input_bytes)
                    with self._lock:
                        self._infos[identity.measurement_id] = info
                if row.preparation_bytes is None:
                    status, proof = self.bridge.prepare_frozen(identity, row.start_json, info["measured_at"])
                    if status == "awaiting_plate":
                        self.store.retry(channel, identity.measurement_id, "awaiting_plate", self.bridge.config.retry_sec)
                        return True
                    if status == "plate_unavailable":
                        self.store.block(channel, identity.measurement_id, "plate_unavailable")
                        return True
                    row = self.store.freeze_preparation(channel, identity.measurement_id, proof)
                    _stage_timing(channel, identity.measurement_id, "plate-ready", info,
                                  attempts=row.attempts)
                snapshot, plate = self.bridge.frozen_preparation(identity, row.start_json,
                    info["measured_at"], row.preparation_bytes)
                # 离场帧若已采到则并入体积；未到齐不硬等（与进场 fail-open 同口径）。
                from .recalculation import encode_exit, decode_exit
                with self._lock:
                    exits = self._exit_volume.get(identity.measurement_id, ())
                exit_data = self.store.freeze_exit(channel, identity.measurement_id,
                    encode_exit(exits))
                self.take_exit(identity.measurement_id)
                calculation_started = time.perf_counter()
                pending = calculate_frozen(row.input_bytes, identity, snapshot, plate, self.bridge.config,
                                          exit_frames=decode_exit(exit_data))
                prepared = prepare_result(pending, self.bridge)
                calculation_ms = round((time.perf_counter() - calculation_started) * 1000, 1)
                row = self.store.freeze_prepared(channel, identity.measurement_id,
                    pack_prepared(pending.result_json, prepared))
                _stage_timing(channel, identity.measurement_id, "calculated", info,
                              calculationMs=calculation_ms, exitFrames=len(exits))
            result_json, prepared = unpack_prepared(row.prepared_bytes)
            payload = json.loads(prepared.payload_json)
            self.store.register_recalculation(channel, identity.measurement_id, payload)
            if (payload["measurementId"], payload["channelId"], payload["passageId"]) != (
                    identity.measurement_id, channel, identity.passage_id):
                raise ValueError("最终字节身份与原任务不一致")
            if existing is not None:
                archived = self.bridge.outbox.archives.get(identity.measurement_id)
                if (_json_bytes(existing.payload) != prepared.payload_json or
                        existing.asset != prepared.asset or archived != prepared.archive):
                    raise ValueError("原ID已存在不同最终字节，禁止覆盖或换ID")
                if identity.measurement_id not in self._handed:
                    if not self.finalizer.restore_durable(identity, result_json, prepared):
                        self.store.retry(channel, identity.measurement_id, "等待历史结果通知容量", .2)
                        return True
                self._handed.discard(identity.measurement_id)
                self.store.mark_committed(channel, identity.measurement_id)
                with self._lock:
                    info = self._infos.pop(identity.measurement_id, None)
                if info is not None:
                    _stage_timing(channel, identity.measurement_id, "committed", info)
            elif identity.measurement_id not in self._handed:
                if self.finalizer.submit_prepared(identity, result_json, prepared):
                    self._handed.add(identity.measurement_id)
                self.store.retry(channel, identity.measurement_id, "等待原最终字节提交", .2)
            else:
                self.store.retry(channel, identity.measurement_id, "等待原最终字节提交", .2)
            return True
        except HttpError as error:
            if error.retryable:
                self.store.retry(channel, identity.measurement_id, str(error),
                    min(30., self.bridge.config.retry_sec * 2 ** min(row.attempts, 4)))
            else:
                self.store.block(channel, identity.measurement_id, str(error))
        except CompletionBusy:
            self.store.retry(channel, identity.measurement_id, "本通道完成预算已占用，保留原留样等待", 2.)
        except PreparedBudgetExceeded as error:
            self.store.block(channel, identity.measurement_id, str(error))
        except BridgeConflict:
            self.store.retry(channel, identity.measurement_id, "原通道结果仍在提交，保留最终字节等待", 1.)
        except (ValueError, TypeError, KeyError) as error:
            self.store.block(channel, identity.measurement_id, "冻结任务校验失败: " + type(error).__name__)
        except Exception as error:
            self.store.retry(channel, identity.measurement_id, "后台处理异常: " + type(error).__name__, 2.)
        finally:
            self._refresh(channel)
        return True

    def _writer(self, channel):
        while not self._ingress_stop.is_set() or self._ingress[channel]:
            try:
                if self.persist_once(channel):
                    continue
            except Exception as error:
                with self._lock:
                    self._stats[channel]["capture_error"] = type(error).__name__
            self._wake[channel].wait(.2)
            self._wake[channel].clear()

    def _processor(self, channel):
        idle = min(2., self.bridge.config.retry_sec)
        wake = self._process_wake[channel]
        # 到车准备与复算查询都要访问 WebApi，保持原节拍；只有本地留样任务提前处理。
        poll_after = 0.
        while not self._halt.is_set():
            wake.clear()
            polled = time.monotonic() >= poll_after
            if polled:
                # 活动车次的首报和历史任务准备由同一车道线程串行，最多4路网络准备。
                try:
                    self.bridge._prepare_channel(channel, self.bridge.config.channels[channel])
                except Exception as error:
                    with self._lock:
                        self._stats[channel]["current_preparation_error"] = type(error).__name__
                if self._halt.is_set():
                    break
                poll_after = time.monotonic() + idle
            worked = False
            try:
                worked = bool(self.process_once(channel))
                if polled and self.recalculation.process_once(channel):
                    # 复算按条推进：有进展就立刻取下一条，不等下一拍。
                    worked, poll_after = True, 0.
            except Exception as error:
                with self._lock:
                    self._stats[channel]["processor_error"] = type(error).__name__
                # 异常不能变成忙等：退回原固定间隔。
                wake.wait(idle)
                continue
            if worked:
                continue
            wake.wait(self._idle_wait(channel, idle, poll_after))

    def _idle_wait(self, channel, idle, poll_after):
        """无事可做时等到下一项到期、下一次轮询或新留样落盘，最长 idle 秒。"""
        wait = min(idle, max(0., poll_after - time.monotonic()))
        try:
            next_at = self.store.next_due_at(channel)
        except Exception:                                    # noqa: BLE001
            next_at = None                                  # 读不到就按原节拍
        if next_at is not None:
            wait = min(wait, max(0., next_at - time.time()))
        # 下限防止"已到期却取不到"之类的不一致把线程变成忙等。
        return max(.05, wait)

    def start(self):
        with self._lock:
            if self._threads or self._closed:
                return
            for channel in self.channels:
                for target in (self._writer, self._processor):
                    thread = threading.Thread(target=target, args=(channel,), daemon=True,
                        name="deferred-%s-%s" % (target.__name__, channel))
                    self._threads.append(thread)
                    thread.start()

    def durable_capture(self, channel):
        """只返回已提交到独立任务库的身份，不把内存入队视作已留样。"""
        with self._lock:
            return self._durable_captures[channel]

    def snapshot(self, channel):
        with self._lock:
            return dict(self._stats[channel], ingress_jobs=len(self._ingress[channel]),
                        ingress_bytes=self._queued_bytes[channel])

    def close(self, timeout=3.):
        self._ingress_stop.set()
        self._halt.set()
        for wake in (*self._wake.values(), *self._process_wake.values()):
            wake.set()
        deadline = None if timeout is None else time.monotonic() + timeout
        for thread in self._threads:
            thread.join(None if deadline is None else max(0., deadline - time.monotonic()))
        if any(thread.is_alive() for thread in self._threads):
            return False
        # 未启动时不会有异步捕获；不允许关掉仍只在RAM的输入。
        if any(self._ingress.values()):
            return False
        if not self._closed:
            self.store.close()
            self._closed = True
        return True
