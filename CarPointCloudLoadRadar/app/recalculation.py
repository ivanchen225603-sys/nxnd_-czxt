"""CH4空车基线生效后的历史载重复算；沿用原车次留样，不采集或改变活动车辆。"""
import hashlib
import json
from dataclasses import replace
from types import SimpleNamespace
from datetime import datetime, timezone, timedelta

from .frozen_detection import FrozenDetection, _FramePool, _b64, _frames, _json, calculate_frozen
from .load_radar_bridge import BaselineSnapshot
from .load_radar_contract import _read_baseline
from .load_radar_transport import _json_bytes
from .bed_reference import baseline_digest
from .result_finalizer import prepare_result
from .measurement_runtime import canonical, algorithm_identity


def encode_exit(frames):
    pool = _FramePool()
    for frame in frames:
        pool.add(frame)
    return canonical({"frames": [dict(f.identity(), dropped_packets=f.dropped_packets, xyz=_b64(f.xyz_bytes),
        ring=_b64(f.ring_bytes) if f.ring_bytes is not None else None) for f in pool.frames]})


def decode_exit(data):
    records = _json(data)["frames"]
    return tuple(_frames(records)) if records else ()


def calculate_revision(bridge, row, target, exit_data):
    original = target["original"]["measurement"]
    version, source = target["baseline"], target["baselineSource"]
    frozen = FrozenDetection.restore(row.input_bytes)
    saved = frozen.record
    local = timezone(timedelta(hours=8))
    day = datetime.fromisoformat(original["measuredAt"].replace("Z", "+00:00")).astimezone(local).date()
    empty_day = datetime.fromisoformat(source["measuredAt"].replace("Z", "+00:00")).astimezone(local).date()
    # 这里只校验外部复算任务的来源，后续使用原冻结任务的类型与身份。
    if (original["measurementId"] != row.identity.measurement_id or original["nodeId"] != bridge.config.node_id
            or original["channelId"] != row.identity.channel or original["passageId"] != row.identity.passage_id
            or original["configurationHash"] != saved["start"]["configurationHash"]
            or version["channelId"] != "CH4" or source["channelId"] != "CH4"
            or version["nodeId"] != bridge.config.node_id or source["nodeId"] != bridge.config.node_id
            or version["measurementId"] != source["measurementId"] or version["calibrationId"] != source["calibrationId"]
            or day != empty_day
            or source["verdict"] != "empty" or source["sourceMode"] != "live" or source["frameDegraded"]
            or source["baselineCandidateSha256"] != version["assetSha256"]
            or version["licensePlate"] != target["original"]["binding"]["licensePlate"]):
        raise ValueError("事后复算目标、原观测或CH4空车来源不一致")
    asset = bridge.client.download_baseline(version["versionId"])
    if hashlib.sha256(asset).hexdigest() != version["assetSha256"]:
        raise ValueError("事后复算基线资产摘要不符")
    base = _read_baseline(asset)
    binding = dict(node_id=bridge.config.node_id, channel_id=row.identity.channel,
        calibration_id=saved["integration"]["calibration_id"], configuration_sha256=saved["start"]["configurationHash"],
        baseline_version_id=version["versionId"], baseline_asset_sha256=version["assetSha256"],
        baseline_sha256=baseline_digest(base))
    snapshot = BaselineSnapshot(row.identity.passage_id, _json_bytes(binding),
        base.cloud.astype("<f8").tobytes(), float(base.floor_z), tuple(base.box),
        tuple(base.bed_plane) if base.bed_plane is not None else None, asset,
        _json_bytes(dict(sourceChannelId="CH4", sourceCalibrationId=source["calibrationId"],
                        sourceConfigurationHash=source["configurationHash"])))
    plate = target["original"]["binding"]["licensePlate"]
    original_config = SimpleNamespace(node_id=bridge.config.node_id, algorithm_version=saved["integration"]["algorithm_version"],
        channels={row.identity.channel: SimpleNamespace(calibration_id=saved["integration"]["calibration_id"])})
    pending = calculate_frozen(row.input_bytes, row.identity, snapshot, plate, original_config,
        exit_frames=decode_exit(exit_data), recalculation=True, frozen=frozen)
    prepared = prepare_result(replace(pending, archive_enabled=False, candidate=None), SimpleNamespace(config=original_config))
    measurement = json.loads(prepared.payload_json)
    if measurement["volumeStatus"] != "success":
        raise ValueError("本次复算未得到可发布体积，保留原正式值和留样：" + measurement["reason"])
    return _json_bytes(dict(id=target["id"], activationId=target["activationId"],
        measurementId=row.identity.measurement_id, baselineVersionId=version["versionId"], measurement=measurement,
        provenance=dict(inputSha256=row.input_sha256,
            exitFramesSha256=hashlib.sha256(exit_data).hexdigest(),
            originalAlgorithmSha256=hashlib.sha256(_json_bytes(saved["algorithm"])).hexdigest(),
            calculationAlgorithmSha256=hashlib.sha256(_json_bytes(dict(detection=algorithm_identity(),
                revision="ch4-day-recalculation-v1", algorithmVersion=bridge.config.algorithm_version))).hexdigest())))


class RecalculationProcessor:
    def __init__(self, bridge, store):
        self.bridge, self.store = bridge, store
        self.after = {}

    def process_once(self, channel):
        old = self.store.unindexed_recalculation(channel)
        if old is not None:
            from .deferred_measurements import unpack_prepared
            from .enter_volume import restore_frames
            _, prepared = unpack_prepared(old.prepared_bytes)
            files = dict(prepared.archive.files) if prepared.archive is not None else {}
            call = json.loads(files["volume-call.json"]) if "volume-call.json" in files else {}
            frames = restore_frames(call["exit"], files["exit-frames.f32le"]) if call.get("exit") else ()
            self.store.freeze_exit(channel, old.identity.measurement_id, encode_exit(frames))
            self.store.register_recalculation(channel, old.identity.measurement_id, json.loads(prepared.payload_json))
            return True
        pending = self.store.pending_recalculation(channel)
        if pending is not None:
            job_id, _mid, payload = pending
            receipt = self.bridge.client.post_recalculation(json.loads(payload))
            if receipt.get("id") != job_id:
                raise ValueError("复算回执编号不符")
            status = receipt.get("status")
            if status == "applied":
                calculation = receipt.get("revision", {}).get("calculation")
                if not self.bridge._same_measurement({"measurement": calculation}, json.loads(payload)["measurement"]):
                    raise ValueError("复算回执未保存本次原结果")
            elif status not in ("superseded", "not-selected"):
                raise ValueError("复算回执状态不符")
            self.store.acknowledge_recalculation(channel, job_id, superseded=status == "superseded")
            return True
        targets = self.bridge.client.recalculations(channel, self.after.get(channel), self.store.recalculation_since(channel))
        if not targets:
            self.after[channel] = None
            return False
        target = targets[0]
        mid = target["original"]["measurement"]["measurementId"]
        self.after[channel] = mid
        row = self.store.get(channel, mid)
        if row is None or row.compacted:
            raise ValueError("原载重留样已不存在，不能凭旧体积或预览点伪造复算")
        payload = calculate_revision(self.bridge, row, target, self.store.exit_bytes(channel, mid))
        self.store.save_recalculation(channel, target["id"], mid, payload)
        return True
