"""先留样后识牌的独立计算上下文；安全字节编解码，不启动线程或读取现场配置。"""
from __future__ import annotations

import base64
from dataclasses import asdict, dataclass, fields, is_dataclass
import hashlib
from types import SimpleNamespace

import numpy as np

from .archive_io import ArchiveReadLimits, bounded_json
from .cloud_observation import FrameSnapshot, ObservationPair, ObservationWindow
from .detect import StabilityResult
from .load_radar_bridge import ResultReservation
from .load_radar_contract import effective_configuration_bytes
from .measurement_runtime import _validate_decision, algorithm_identity, canonical
from .pointcloud import AngleCache
from .result_finalizer import PendingResult, result_bytes
from .settings import CalibCfg, ChannelCfg, DevicesCfg
from .volume_provider import VolumeCatalog


MAX_INPUT_BYTES = 96 * 1024 * 1024
MAX_BINARY_BYTES = 64 * 1024 * 1024
MAX_FRAMES = 192
MAX_FRAME_POINTS = 2_000_000
_STATE_FIELDS = ("_capture_instance_id", "_observation_epoch", "_session_id", "_session_forced",
                 "_last_save_name", "_detected_this_session", "_degraded_measure", "_config_fp",
                 "save_enabled", "eff_mode")
_ROOT_FIELDS = {"schema", "identity", "start", "configuration", "algorithm", "integration", "catalog",
                "frames", "primary", "support", "frame", "settle", "raw", "extra_members", "gate", "state"}
# v2（2026-10-24）：另冻结判稳时的进场帧（给体积补两端，见 app/enter_volume.py）。
_ROOT_FIELDS_V2 = _ROOT_FIELDS | {"enter"}
# 进场帧最多占冻结二进制预算的这么多；超了从最早的帧开始舍（锚点是最后一帧，链从后往前配）。
ENTER_FROZEN_BUDGET = 24 * 1024 * 1024


def _keys(value, expected, name):
    if type(value) is not dict or set(value) != set(expected):
        raise ValueError(name + "字段缺失或含未知字段")


def _json(data, cap=MAX_INPUT_BYTES):
    if type(data) is not bytes or not data or len(data) > cap:
        raise ValueError("冻结字节为空或超过预算")
    # Base64 占据绝大多数文本；结构另设小预算，解码后的点云另限总字节。
    return bounded_json(data, ArchiveReadLimits(json_bytes=cap, working_bytes=cap * 128,
                                               json_depth=24, json_nodes=50_000))


def _b64(data):
    return base64.b64encode(data).decode("ascii")


def _unb64(value, cap):
    if type(value) is not str or len(value) > (cap + 2) // 3 * 4:
        raise ValueError("二进制成员超过预算")
    try:
        data = base64.b64decode(value, validate=True)
    except (ValueError, UnicodeError) as error:
        raise ValueError("二进制成员不是规范 Base64") from error
    if len(data) > cap or _b64(data) != value:
        raise ValueError("二进制成员长度或编码不规范")
    return data


def _excluded(name):
    return name in ("path", "raw", "angle_cache", "abs_angle_cache") or name.endswith(("_path", "_dir"))


def _configuration_object(default, record):
    """固定本机 dataclass 白名单；包中的类名或路径永远不参与加载。"""
    expected = {field.name for field in fields(default) if not _excluded(field.name)}
    _keys(record, expected, "有效配置对象")
    for field in fields(default):
        name, old = field.name, getattr(default, field.name)
        if _excluded(name):
            setattr(default, name, {} if name == "raw" else "")
            continue
        value = record[name]
        if is_dataclass(old):
            value = _configuration_object(old, value)
        elif type(old) in (str, bool, int) and type(value) is not type(old):
            raise ValueError("有效配置字段类型不符: " + name)
        elif type(old) is float and type(value) not in (int, float):
            raise ValueError("有效配置数字类型不符: " + name)
        elif isinstance(old, tuple):
            if type(value) is not list or len(value) != len(old):
                raise ValueError("有效配置向量长度不符: " + name)
            value = tuple(value)
        setattr(default, name, value)
    return default


def _configuration(data):
    value = _json(data, 1024 * 1024)
    _keys(value, {"schema", "calib", "devices", "channel", "angle"}, "有效配置")
    if value["schema"] != "jt128-effective-configuration-v1":
        raise ValueError("不支持的有效配置版本")
    calib = _configuration_object(CalibCfg(), value["calib"])
    dev = _configuration_object(DevicesCfg(), value["devices"])
    ch = _configuration_object(ChannelCfg(), value["channel"])
    angle = AngleCache()
    _keys(value["angle"], {"elevation", "azimuth", "loaded", "channels_seen"}, "角度表")
    for name in ("elevation", "azimuth"):
        array = np.asarray(value["angle"][name], dtype=np.float32)
        if array.shape != (256,) or not np.isfinite(array).all():
            raise ValueError("角度表必须为 256 个有限数值")
        setattr(angle, name, array)
    angle.loaded, angle.channels_seen = value["angle"]["loaded"], value["angle"]["channels_seen"]
    if (type(angle.loaded) is not bool or type(angle.channels_seen) is not int
            or not 0 <= angle.channels_seen <= 256):
        raise ValueError("角度标定状态非法")
    if effective_configuration_bytes(calib, dev, ch, angle) != data:
        raise ValueError("恢复配置不能逐字重建原配置")
    return calib, dev, ch, angle


def _integration(config, channel):
    return dict(node_id=config.node_id, calibration_id=config.channels[channel].calibration_id,
                algorithm_version=config.algorithm_version)


def _identity(identity):
    if not isinstance(identity, ResultReservation) or identity.command_json is not None:
        raise ValueError("异步留样只接自动测量，人工命令继续原许可链")
    values = dict(measurement_id=identity.measurement_id, channel=identity.channel,
                  passage_id=identity.passage_id, origin_token=identity.origin_token)
    if any(type(v) is not str or not v or len(v) > 256 for v in values.values()):
        raise ValueError("自动留样必须有完整的原车次及测量身份")
    return values


class _FramePool:
    def __init__(self):
        self.frames, self.refs, self.size = [], {}, 0
        self.indices = []

    def add(self, frame):
        if not isinstance(frame, FrameSnapshot):
            # 优先复用原窗口字节；常见单帧路径不再复制约数 MB 的 XYZ/ring。
            for index, existing in enumerate(self.frames):
                if (frame.seq, frame.ts, frame.gen, frame.packets, frame.dropped_packets) != (
                        existing.seq, existing.ts, existing.gen, existing.packets, existing.dropped_packets):
                    continue
                original = existing.frame()
                if (np.array_equal(frame.xyz, original.xyz) and (frame.ring is None) == (original.ring is None)
                        and (frame.ring is None or np.array_equal(frame.ring, original.ring))):
                    return index
        snap = frame if isinstance(frame, FrameSnapshot) else FrameSnapshot.freeze(frame)
        if snap in self.refs:
            return self.refs[snap]
        self.size += len(snap.xyz_bytes) + len(snap.ring_bytes or b"")
        if (self.size > MAX_BINARY_BYTES or len(self.frames) >= MAX_FRAMES
                or len(snap.xyz_bytes) // 12 > MAX_FRAME_POINTS):
            raise ValueError("冻结帧数量或点云内存超过预算")
        self.refs[snap] = len(self.frames)
        self.frames.append(snap)
        return self.refs[snap]

    def window(self, window):
        if not isinstance(window, ObservationWindow):
            raise ValueError("留样必须包含实际主观测和体积支持窗口")
        rows = []
        for pair in window.pairs:
            rows.append(dict(channel=pair.channel, epoch=pair.epoch, received_monotonic=pair.received_monotonic,
                complete=pair.complete, raw=self.add(pair.raw), cropped=self.add(pair.cropped), indices=len(self.indices)))
            self.indices.append(pair.index_bytes)
        return rows


@dataclass(frozen=True)
class FrozenDraft:
    """采集锁内的小元数据加不可变原字节；大 JSON/Base64 只在 writer 编码。"""
    metadata_json: bytes
    frames: tuple
    indices: tuple
    models: tuple

    def byte_size(self):
        blobs = {id(data): data for frame in self.frames for data in (frame.xyz_bytes, frame.ring_bytes) if data is not None}
        blobs.update({id(data): data for data in self.indices + self.models + (self.metadata_json,)})
        return sum(map(len, blobs.values())) * 4 + 64 * 1024

    def encode(self) -> bytes:
        value = _json(self.metadata_json, 1024 * 1024)
        value["frames"] = [dict(f.identity(), dropped_packets=f.dropped_packets, xyz=_b64(f.xyz_bytes),
            ring=_b64(f.ring_bytes) if f.ring_bytes is not None else None) for f in self.frames]
        for name in ("primary", "support"):
            for pair in value[name]:
                pair["indices"] = _b64(self.indices[pair["indices"]])
        if value["catalog"] is not None:
            value["catalog"]["models"] = [_b64(model) for model in self.models]
        data = canonical(value)
        if len(data) > MAX_INPUT_BYTES:
            raise ValueError("冻结编码超过字节预算")
        return data


def _context(value, configuration):
    _keys(value["identity"], {"measurement_id", "channel", "passage_id", "origin_token"}, "冻结身份")
    identity = ResultReservation(**value["identity"])
    _identity(identity)
    calib, _, ch, _ = _configuration(configuration)
    start, public = value["start"], value["integration"]
    _keys(public, {"node_id", "calibration_id", "algorithm_version"}, "节点身份")
    _keys(start, {"nodeId", "channelId", "passageId", "calibrationId", "configurationHash", "sourceMode", "arrivedAt"}, "原到车请求")
    if (start["nodeId"] != public["node_id"] or start["channelId"] != identity.channel
            or ch.id != identity.channel or start["passageId"] != identity.passage_id
            or start["calibrationId"] != public["calibration_id"] or start["sourceMode"] != "live"
            or start["configurationHash"] != hashlib.sha256(configuration).hexdigest()):
        raise ValueError("原到车请求与冻结通道/车次/配置不一致")
    state = value["state"]
    _keys(state, _STATE_FIELDS, "计算状态")
    if (state["eff_mode"] not in ("live", "udp", "real") or state["_degraded_measure"] is not False
            or type(state["save_enabled"]) is not bool or type(state["_session_forced"]) is not bool
            or any(type(state[k]) is not int or state[k] < 0 for k in
                   ("_session_id", "_observation_epoch", "_detected_this_session"))
            or type(state["_capture_instance_id"]) is not str or not state["_capture_instance_id"]
            or type(state["_last_save_name"]) is not str):
        raise ValueError("冻结测量状态不完整或不是正式完整源")
    return identity, calib, state


def _gate(gate, primary, calib):
    _keys(gate, {"decision", "settle", "manual_three_frames"}, "采集许可")
    decision = gate["decision"]
    _keys(decision["stability"], {f.name for f in fields(StabilityResult)}, "本版停稳原值")
    _validate_decision(decision, asdict(calib.detection))
    last = primary.pairs[-1]
    if (decision["frame"] != last.raw.identity() or decision["received_monotonic"] != last.received_monotonic
            or decision["stability"]["n_points"] != last.cropped.frame().n):
        raise ValueError("冻结许可不是原末帧观测")
    settle = gate["settle"]
    _keys(settle, {"kind", "applicable", "started_monotonic", "finished_monotonic", "waited_sec", "required_sec", "accumulate"}, "沉降许可")
    if (settle["kind"] != "automatic" or settle["applicable"] is not True or gate["manual_three_frames"] != []
            or type(settle["accumulate"]) is not bool or settle["required_sec"] < 0
            or settle["waited_sec"] < settle["required_sec"]
            or settle["finished_monotonic"] - settle["started_monotonic"] != settle["waited_sec"]
            or settle["finished_monotonic"] < last.received_monotonic):
        raise ValueError("原沉降窗口尚未达到许可")


def _frames(records):
    if type(records) is not list or not 0 < len(records) <= MAX_FRAMES:
        raise ValueError("冻结帧数量非法")
    frames, remaining = [], MAX_BINARY_BYTES
    for row in records:
        _keys(row, {"seq", "ts", "gen", "packets", "dropped_packets", "xyz", "ring"}, "冻结帧")
        xyz = _unb64(row["xyz"], min(remaining, MAX_FRAME_POINTS * 12))
        remaining -= len(xyz)
        ring = None if row["ring"] is None else _unb64(row["ring"], min(remaining, MAX_FRAME_POINTS * 2))
        remaining -= len(ring or b"")
        if len(xyz) % 12 or ring is not None and len(ring) != len(xyz) // 6:
            raise ValueError("冻结点云 XYZ/ring 行数不一致")
        snap = FrameSnapshot(xyz, ring, row["seq"], row["ts"], row["gen"], row["packets"], row["dropped_packets"])
        if FrameSnapshot.freeze(snap.frame()) != snap:
            raise ValueError("冻结帧内容无法精确恢复")
        frames.append(snap)
    return frames


def _ref(frames, index):
    if type(index) is not int or not 0 <= index < len(frames):
        raise ValueError("冻结帧引用越界")
    return frames[index]


def _window(records, frames, roi):
    if type(records) is not list or not 0 < len(records) <= MAX_FRAMES:
        raise ValueError("观测窗口为空或超过预算")
    pairs = []
    for row in records:
        _keys(row, {"channel", "epoch", "received_monotonic", "complete", "raw", "cropped", "indices"}, "观测配对")
        pair = ObservationPair.freeze(channel=row["channel"], epoch=row["epoch"],
            received_monotonic=row["received_monotonic"], complete=row["complete"],
            raw=_ref(frames, row["raw"]).frame(), cropped=_ref(frames, row["cropped"]).frame(), roi=roi)
        if _unb64(row["indices"], MAX_FRAME_POINTS * 8) != pair.index_bytes:
            raise ValueError("原始观测 ROI 行索引已变化")
        pairs.append(pair)
    return ObservationWindow(tuple(pairs))


@dataclass(frozen=True)
class FrozenDetection:
    """恢复结果拥有自己的对象/字节；不保存活动 worker 或配置路径引用。"""
    record: dict
    configuration: bytes
    primary: ObservationWindow
    support: ObservationWindow
    frame: FrameSnapshot
    settle: tuple
    raw: tuple
    catalog: object
    enter: tuple = ()

    @staticmethod
    def capture(worker, frame, stab, identity, start_json) -> FrozenDraft:
        if worker.integration is None:
            raise ValueError("异步车牌留样要求已配置正式后台绑定")
        if effective_configuration_bytes(worker.calib, worker.dev, worker.ch, worker.angle) != worker._effective_configuration:
            raise ValueError("活动配置对象已变化，不能冒充原有效配置")
        writer = _FramePool()
        catalog = worker.volume_catalog
        models = None if catalog is None else dict(source_sha256=catalog.source_sha256,
            # VolumeCatalog 只持有各模型规范字节及原目录摘要，不虚构原目录文件字节。
            models=[])
        start = _json(start_json, 16384) if isinstance(start_json, bytes) else _json(canonical(start_json), 16384)
        body = dict(schema="jt128-frozen-detection-v2", identity=_identity(identity), start=start,
            configuration=_b64(worker._effective_configuration), algorithm=algorithm_identity(),
            integration=_integration(worker.integration.config, worker.ch.id), catalog=models,
            primary=writer.window(worker._measurement_window), support=writer.window(worker._volume_observation_window),
            frame=writer.add(frame), settle=[writer.add(f) for f in worker._settle_frames],
            raw=[dict(waited=waited, frame=writer.add(f)) for waited, f in worker._raw_shots],
            extra_members=list(worker._volume_extra_members), gate=worker._measurement_gate,
            state={name: getattr(worker, name) for name in _STATE_FIELDS}, frames=[])
        # 进场帧不能让整份留样超预算而失败：预算不够时只留最近的若干帧，再不够就不带。
        enter = list(getattr(worker, "_enter_volume", ()) or ())
        room = min(ENTER_FROZEN_BUDGET, MAX_BINARY_BYTES - writer.size - 1024 * 1024)
        kept = []
        for snap in reversed(enter):
            if len(snap.xyz_bytes) > room or len(writer.frames) + len(kept) + 1 >= MAX_FRAMES:
                break
            room -= len(snap.xyz_bytes)
            kept.append(snap)
        body["enter"] = [writer.add(snap) for snap in reversed(kept)]
        # 与正式归档出口一致：NumPy 标量/状态枚举编码成等值 JSON 原语，再做严格许可校验。
        # 不允许用 None/0 修补无穷值；下面的许可校验仍会拒绝缺失或非有限指标。
        body["gate"] = _json(result_bytes(body["gate"]), 1024 * 1024)
        if result_bytes(body["gate"]["decision"]["stability"]) != result_bytes(asdict(stab)):
            raise ValueError("传入停稳原值与本次许可不一致")
        if hashlib.sha256(worker._effective_configuration).hexdigest() != worker._integration_hash:
            raise ValueError("活动有效配置摘要不一致")
        expected = tuple(worker._volume_observation_window.pairs[i].cropped.frame().xyz
                         for i in worker._volume_extra_members)
        if len(expected) != len(worker._vol_extra) or any(not np.array_equal(a, b) for a, b in zip(expected, worker._vol_extra)):
            raise ValueError("额外货面不属于已冻结支持成员")
        frozen_identity, calib, state = _context(body, worker._effective_configuration)
        primary, support = worker._measurement_window, worker._volume_observation_window
        _gate(body["gate"], primary, calib)
        from .volume_inputs import surface_lineage
        surface_lineage(primary, support, tuple(worker._volume_extra_members))
        if (len(worker._volume_extra_members) > 2 or any(
                p.channel != frozen_identity.channel or p.epoch != state["_observation_epoch"]
                for window in (primary, support) for p in window.pairs)):
            raise ValueError("采集窗口的通道、活动代次或额外货面数量不符")
        actual = writer.frames[body["frame"]]
        if actual != FrameSnapshot.freeze(primary.input_frame()):
            raise ValueError("采集主帧不属于实际窗口")
        metadata = canonical(body)
        if len(metadata) > 1024 * 1024:
            raise ValueError("采集元数据超过预算")
        return FrozenDraft(metadata, tuple(writer.frames), tuple(writer.indices),
                           tuple(model.canonical_json for model in catalog.models) if catalog else ())

    @staticmethod
    def freeze(worker, frame, stab, identity, start_json) -> bytes:
        """离线兼容入口；采集线程使用 capture，writer 再 encode/inspect。"""
        return FrozenDetection.capture(worker, frame, stab, identity, start_json).encode()

    @classmethod
    def restore(cls, data):
        value = _json(data)
        v2 = type(value) is dict and value.get("schema") == "jt128-frozen-detection-v2"
        _keys(value, _ROOT_FIELDS_V2 if v2 else _ROOT_FIELDS, "冻结任务")
        if value["schema"] not in ("jt128-frozen-detection-v1", "jt128-frozen-detection-v2"):
            raise ValueError("不支持的冻结任务版本")
        configuration = _unb64(value["configuration"], 1024 * 1024)
        identity, calib, state = _context(value, configuration)
        frames = _frames(value["frames"])
        primary, support = (_window(value[k], frames, calib.roi) for k in ("primary", "support"))
        frame = _ref(frames, value["frame"])
        if frame != FrameSnapshot.freeze(primary.input_frame()):
            raise ValueError("计算主帧不是冻结主窗口输入")
        if any(p.channel != identity.channel or p.epoch != state["_observation_epoch"]
               for window in (primary, support) for p in window.pairs):
            raise ValueError("冻结观测通道或活动代次不一致")
        from .volume_inputs import surface_lineage
        extra = value["extra_members"]
        if type(extra) is not list or len(extra) > 2 or any(type(i) is not int for i in extra):
            raise ValueError("体积只允许最多两份真实额外货面")
        surface_lineage(primary, support, tuple(extra))
        gate = value["gate"]
        _gate(gate, primary, calib)
        if type(value["settle"]) is not list or len(value["settle"]) > MAX_FRAMES:
            raise ValueError("沉降帧数量超过预算")
        settled = tuple(_ref(frames, i) for i in value["settle"])
        known = {p.cropped for p in support.pairs} | {p.cropped for p in primary.pairs}
        if any(f not in known for f in settled):
            raise ValueError("诊断沉降帧不属于原观测窗口")
        if type(value["raw"]) is not list or len(value["raw"]) > MAX_FRAMES:
            raise ValueError("原始留帧数量超过预算")
        raw = []
        for item in value["raw"]:
            _keys(item, {"waited", "frame"}, "原始留帧")
            shot = _ref(frames, item["frame"])
            if (type(item["waited"]) not in (int, float) or item["waited"] < 0 or shot.gen != frame.gen
                    or shot.ts > frame.ts or shot.dropped_packets):
                raise ValueError("原始留帧时序或完整性不符")
            raw.append((item["waited"], shot))
        catalog = None
        if value["catalog"] is not None:
            item = value["catalog"]
            _keys(item, {"source_sha256", "models"}, "冻结模型目录")
            if type(item["models"]) is not list or len(item["models"]) > 128:
                raise ValueError("冻结模型数量超过预算")
            models = [_json(_unb64(m, 1024 * 1024), 1024 * 1024) for m in item["models"]]
            checked = VolumeCatalog.from_bytes(canonical(dict(schema="volume-provider-v1", models=models)))
            digest = item["source_sha256"]
            if type(digest) is not str or len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
                raise ValueError("原模型目录摘要非法")
            catalog = VolumeCatalog(checked.models, digest)
        enter = ()
        if v2:
            if type(value["enter"]) is not list or len(value["enter"]) > MAX_FRAMES:
                raise ValueError("进场帧数量超过预算")
            from .enter_volume import check_frames, volume_snapshot
            enter = check_frames(volume_snapshot(_ref(frames, i)) for i in value["enter"])
            first = primary.pairs[0].raw
            if any(f.gen != first.gen or f.ts >= first.ts for f in enter):
                raise ValueError("进场帧不在本次停稳观测之前或跨源代次")
        return cls(value, configuration, primary, support, frame, settled, tuple(raw), catalog, enter)


class _Collector:
    def __init__(self):
        self.pending = None
        self.events = []

    def submit(self, pending):
        if not isinstance(pending, PendingResult) or self.pending is not None:
            raise ValueError("独立计算必须且只能生成一份完整结果")
        self.pending = pending

    def publish(self, kind, value):
        self.events.append((kind, value))


def inspect_frozen(input_bytes) -> dict:
    """严格校验后返回原任务的公开绑定信息，不返回凭据或活动对象。"""
    frozen = FrozenDetection.restore(input_bytes)
    value = frozen.record
    return dict(measured_at=frozen.frame.ts, configuration_hash=value["start"]["configurationHash"],
                algorithm_version=value["integration"]["algorithm_version"],
                node_id=value["integration"]["node_id"], calibration_id=value["integration"]["calibration_id"],
                identity=dict(value["identity"]))


def calculate_frozen(input_bytes, identity, baseline_snapshot, prepared_plate, integration_config,
                     exit_frames=(), *, recalculation=False, frozen=None) -> PendingResult:
    """基线/车牌由原车次授权绑定层传入；此函数无 HTTP、缓存选择或落盘。

    exit_frames：测后迟到附着的离场帧快照（采集时间序）；没有则 ()，体积照旧只用进场/停稳。
    recalculation 保留旧调用签名；恢复与复算均不再按源码指纹决定是否执行。
    """
    frozen = frozen or FrozenDetection.restore(input_bytes)
    value = frozen.record
    if value["identity"] != _identity(identity) or value["integration"] != _integration(integration_config, identity.channel):
        raise ValueError("计算身份或节点配置不是原冻结任务")
    if type(prepared_plate) is not str or not prepared_plate.strip() or len(prepared_plate) > 64:
        raise ValueError("计算必须传入原车次的权威车牌")
    baseline, version = None, None
    if baseline_snapshot is not None:
        from .bed_reference import baseline_digest
        from .load_radar_bridge import BaselineSnapshot
        from .load_radar_contract import _read_baseline
        from .volume_provider import _binding
        if not isinstance(baseline_snapshot, BaselineSnapshot):
            raise ValueError("必须传入不可变的原车次基线快照")
        binding = baseline_snapshot.binding
        _binding(binding)
        expected = dict(node_id=value["integration"]["node_id"], channel_id=identity.channel,
                        calibration_id=value["integration"]["calibration_id"],
                        configuration_sha256=value["start"]["configurationHash"])
        baseline = baseline_snapshot.baseline()
        if (baseline_snapshot.passage_id != identity.passage_id or any(binding.get(k) != v for k, v in expected.items())
                or binding.get("baseline_sha256") != baseline_digest(baseline)):
            raise ValueError("基线快照不属于原车次/配置/实际基线")
        asset = baseline_snapshot.asset_bytes
        if (type(asset) is not bytes or len(asset) > 32 * 1024 * 1024
                or hashlib.sha256(asset).hexdigest() != binding.get("baseline_asset_sha256")
                or baseline_digest(_read_baseline(asset)) != baseline_digest(baseline)):
            raise ValueError("正式基线缺少内容一致的原资产")
        version = binding["baseline_version_id"]
    from .plate import PlateResult
    from .service import ChannelWorker, ChannelState
    # 不运行 ChannelWorker.__init__：它会读取配置并创建源。这里只有独立白名单上下文。
    class Adapter:
        _calculate_detection = ChannelWorker._calculate_detection
        _build_result = ChannelWorker._build_result
        _window_evidence = ChannelWorker._window_evidence
        _empty_evidence = ChannelWorker._empty_evidence
        _worth_labeling = ChannelWorker._worth_labeling
        _runtime_record = ChannelWorker._runtime_record
        _VOL_EXTRA_MAX = ChannelWorker._VOL_EXTRA_MAX
        _LABEL_NEAR_MISS = ChannelWorker._LABEL_NEAR_MISS

        def _plate_collect(self):
            return PlateResult(ok=True, plate=prepared_plate, confidence=1., raw="原车次权威绑定", attempts=1)

    adapter = Adapter()
    adapter.calib, adapter.dev, adapter.ch, adapter.angle = _configuration(frozen.configuration)
    for name, setting in value["state"].items():
        setattr(adapter, name, setting)
    adapter.integration = SimpleNamespace(config=SimpleNamespace(**value["integration"]))
    adapter.baseline = baseline
    adapter.volume_catalog = frozen.catalog
    adapter._effective_configuration = frozen.configuration
    adapter._integration_hash = value["start"]["configurationHash"]
    adapter._measurement_window, adapter._volume_observation_window = frozen.primary, frozen.support
    adapter._volume_extra_members = tuple(value["extra_members"])
    adapter._vol_extra = [frozen.support.pairs[i].cropped.frame().xyz for i in adapter._volume_extra_members]
    frame = frozen.frame.frame()
    # 保持原窗口“主帧对象本身”的排除语义，避免诊断把主帧又算一次。
    adapter._settle_frames = [frame if f == frozen.frame else f.frame() for f in frozen.settle]
    adapter._raw_shots = [(waited, f.frame()) for waited, f in frozen.raw]
    adapter._enter_volume = frozen.enter
    adapter._exit_volume = tuple(exit_frames or ())
    adapter._measurement_gate = value["gate"]
    adapter.st = ChannelState(adapter.ch.id, adapter.ch.name)
    collector = _Collector()
    adapter._finalizer = adapter.bus = collector
    stab = StabilityResult(**dict(value["gate"]["decision"]["stability"],
                                  triggers=tuple(value["gate"]["decision"]["stability"]["triggers"])))
    adapter._calculate_detection(frame, stab, identity=identity, baseline_version_id=version,
                                 baseline_snapshot=baseline_snapshot)
    if collector.pending is None:
        raise ValueError("独立计算没有提交完整结果")
    return collector.pending
