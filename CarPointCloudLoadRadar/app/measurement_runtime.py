"""测量时冻结原参数、原资产和许可摘要；不读取活动配置或生产数据库。"""
from __future__ import annotations

from dataclasses import asdict, fields
import hashlib
import json
import math
import platform

import numpy as np

from .archive_io import bounded_json, runtime_members
from .bed_reference import BedReferenceScope, baseline_digest, surface_digest, valid_surface_scope
from .volume_context import VolumeContext, grid_geometry
from .volume_inputs import align_extra_surfaces, surface_lineage


# v2（2026-10-24）：进场帧 + 遮挡上限补洞 → volume-call-v2。
# v3：对称接入离场帧补后端 → volume-call-v3；v1/v2 旧档照读，复算合同不同报 unsupported_algorithm。
ALGORITHM_CONTRACT = "service-volume-auto-v3"
_CALL_V1_KEYS = "schema identity configuration_sha256 algorithm baseline loaded params runtime original_result"
_PARAMS_V1_KEYS = "cells spread_target bed_z bed_shift_max_m fill_holes floor_band_m dy bed_evidence bed_scope volume_context volume_scope"


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"), allow_nan=False).encode("utf-8")


def algorithm_identity():
    """记录调用格式及运行环境；源码变化不限制旧留样按当前程序计算。"""
    return dict(contract=ALGORITHM_CONTRACT, python=platform.python_version(), numpy=np.__version__)


def freeze_runtime(*, identity, configuration, baseline, baseline_version, asset, loaded,
                   params, runtime, volume, gate, primary, support, extra_members,
                   enter_frames=(), enter_record=None, exit_frames=(), exit_record=None):
    """参数来自实际调用，资产必须是原下载字节；缺资产明确保留不可复算状态。

    enter/exit_frames 与对应 record：本次实际交给体积的进/离场帧快照与配准记录。
    有记录就同时存原帧字节，复算时重新配准；没有就记 None。
    """
    from .result_finalizer import result_bytes
    from .enter_volume import (ENTER_FRAMES_MEMBER, EXIT_FRAMES_MEMBER, frame_record, frames_bytes,
                               validate_record)
    enter_frames = tuple(enter_frames or ())
    exit_frames = tuple(exit_frames or ())
    if enter_record is not None:
        validate_record(enter_record)
        if [frame_record(f) for f in enter_frames] != enter_record["frames"]:
            raise ValueError("进场帧记录不属于实际冻结的进场帧")
    elif enter_frames:
        raise ValueError("进场帧缺少配准记录")
    if exit_record is not None:
        validate_record(exit_record)
        if [frame_record(f) for f in exit_frames] != exit_record["frames"]:
            raise ValueError("离场帧记录不属于实际冻结的离场帧")
    elif exit_frames:
        raise ValueError("离场帧缺少配准记录")
    ids, members = surface_lineage(primary, support, extra_members)
    actual_identity = dict(measurement_id=identity.measurement_id, channel=identity.channel,
                           passage_id=identity.passage_id, origin_token=identity.origin_token)
    has_base = baseline is not None and baseline.n > 0
    if not has_base:
        asset = None
    record = dict(state="available" if asset is not None else "asset_unavailable" if has_base else "absent",
                  version_id=baseline_version, semantic_sha256=baseline_digest(baseline) if has_base else None,
                  asset_sha256=hashlib.sha256(asset).hexdigest() if asset is not None else None)
    extras = params.get("loaded_extra") or ()
    loaded_record = None if loaded is None else dict(
        source_rotation=np.asarray(loaded.source_rotation).tolist(),
        surface_sha256=surface_digest((loaded.cloud,) + tuple(extras)),
        frame_ids=ids, surface_members=members)
    call_params = {key: (asdict(value) if hasattr(value, "__dataclass_fields__") else value)
                   for key, value in params.items() if key not in ("loaded_extra", "enter_extra")}
    call = dict(schema="volume-call-v3", identity=actual_identity,
                configuration_sha256=hashlib.sha256(configuration).hexdigest(),
                algorithm=algorithm_identity(), baseline=record, loaded=loaded_record,
                params=call_params, runtime=runtime, original_result=volume.to_dict(raw=True),
                enter=enter_record, exit=exit_record)
    summary = dict(gate, schema="gate-summary-v1", summary_only=True, event_stream_complete=False,
                   identity=actual_identity, configuration_sha256=call["configuration_sha256"],
                   input_refs=dict(primary=primary.segments(), support=support.segments(),
                                   extra_members=list(extra_members)))
    files = [("effective-configuration.json", configuration), ("volume-call.json", result_bytes(call)),
             ("gate-summary.json", result_bytes(summary))]
    if asset is not None:
        files.append(("baseline.npz", asset))
    if enter_record is not None:
        files.append((ENTER_FRAMES_MEMBER, frames_bytes(enter_frames)))
    if exit_record is not None:
        files.append((EXIT_FRAMES_MEMBER, frames_bytes(exit_frames)))
    return tuple(files)


def _keys(value, names, title):
    if type(value) is not dict or set(value) != set(names.split()):
        raise ValueError(title + "字段无效")


def _number(value, nullable=False):
    if value is None and nullable:
        return
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("运行参数必须为有限数值")


def _sha(value):
    if not isinstance(value, str) or len(value) != 64 or any(c not in "0123456789abcdef" for c in value):
        raise ValueError("运行摘要非法")


def scope_from_record(value):
    _keys(value, " ".join(f.name for f in fields(BedReferenceScope)), "体积scope")
    converted = dict(value)
    converted["frame_ids"] = tuple(tuple(row) for row in value["frame_ids"])
    converted["surface_members"] = tuple(tuple(row) for row in value["surface_members"])
    scope = BedReferenceScope(**converted)
    if not valid_surface_scope(scope, len(scope.surface_members)):
        raise ValueError("体积scope格式非法")
    return scope


def context_from_record(value):
    _keys(value, " ".join(f.name for f in fields(VolumeContext)), "体积context")
    converted = dict(value, scope=scope_from_record(value["scope"]))
    for name in ("domain_xy", "support_origin_xy"):
        converted[name] = tuple(value[name])
    return VolumeContext(**converted)


def _validate_decision(decision, detection=None):
    from .detect import StabilityResult
    _keys(decision, "frame received_monotonic stability thresholds frame_policy", "单帧许可")
    _keys(decision["frame"], "seq ts gen packets", "许可帧")
    frame = decision["frame"]
    if any(type(frame[key]) is not int or frame[key] < minimum for key, minimum in
           (("seq", 1), ("gen", 0), ("packets", 0))):
        raise ValueError("许可帧身份必须为合法整数")
    _number(frame["ts"])
    if frame["ts"] <= 0:
        raise ValueError("许可采集时间非法")
    _number(decision["received_monotonic"])
    state = decision["stability"]
    # 旧归档没有微位移复核诊断；只兼容整组缺省，不补造旧观测的确认记录。
    confirmation_fields = {"motion_pending", "instant_speed_mps", "jitter_confirmed"}
    expected_fields = {f.name for f in fields(StabilityResult)}
    if not (confirmation_fields & set(state)):
        expected_fields -= confirmation_fields
    _keys(state, " ".join(expected_fields), "停稳原值")
    if state["state"] != 2 or type(state["state"]) is not int:
        raise ValueError("许可必须对应已停稳状态")
    if (any(type(state[key]) is not int or state[key] < 0 for key in ("n_points", "n_vehicle_points"))
            or state["n_vehicle_points"] > state["n_points"] or not isinstance(state["reason"], str)):
        raise ValueError("停稳点数、计数或理由类型非法")
    if any(state[key] is not expected for key, expected in (
            ("stable", True), ("fresh", True), ("degraded", False), ("motion_observable", True))):
        raise ValueError("测量许可缺少完整连续新帧")
    if state.get("motion_pending", False) is not False:
        raise ValueError("待复核的微小位移不能取得测量许可")
    if type(state.get("jitter_confirmed", False)) is not bool:
        raise ValueError("抖动复核标记必须为布尔值")
    _number(state.get("instant_speed_mps", 0.))
    if state.get("instant_speed_mps", 0.) < 0:
        raise ValueError("相邻帧速度不能为负")
    for key in ("deviation", "elapsed", "stationary_sec", "speed_mps", "drift_m", "shape_speed_mps", "yaw_speed_mps"):
        _number(state[key])
        if state[key] < 0:
            raise ValueError("停稳指标不能为负")
    thresholds = decision["thresholds"]
    if (confirmation_fields <= set(state)) != ("stationary_jitter_m" in thresholds):
        raise ValueError("抖动复核诊断与原门槛必须同时存在")
    if detection is not None:
        if (("stationary_jitter_m" in detection) != ("stationary_jitter_m" in thresholds)
                or detection.get("stationary_jitter_m") != thresholds.get("stationary_jitter_m")):
            raise ValueError("抖动复核门槛与原有效配置不一致")
    # ▲ 这张表必须与 `service._gate_decision` 的 names 逐字一致。
    #   2026-09-21 现场踩过：只给产生端加了
    #   stationary_shape_speed_mps / stationary_yaw_speed_mps 两个门槛，
    #   这边没跟上 -> 每一份归档都抛「停稳门槛字段无效」，
    #   而终局器只报 ValueError 类型名，整天 41 车次一份没落盘。
    names = ("delta vehicle_threshold ground_z vehicle_min_height_m count_ratio "
             "stationary_min_sec stationary_speed_mps stationary_drift_m "
             "stationary_shape_speed_mps stationary_yaw_speed_mps "
             "stationary_speed_window_sec stationary_max_gap_sec")
    if "stationary_jitter_m" in thresholds:
        names += " stationary_jitter_m"
    _keys(thresholds, names, "停稳门槛")
    for value in thresholds.values():
        _number(value)
    if "stationary_jitter_m" in thresholds:
        if not 0 < thresholds["stationary_jitter_m"] < thresholds["stationary_drift_m"]:
            raise ValueError("抖动复核预算不在固定锚点范围内")
        if (state.get("instant_speed_mps", 0.) > thresholds["stationary_speed_mps"]
                and state.get("jitter_confirmed") is not True):
            raise ValueError("瞬时超速没有后续新帧复核记录")
    # ▲ 每个速度量都要和**它自己的**门槛比，与 StabilityDetector 的判据逐项一致：
    #     回归速度 speed -> stationary_speed_mps
    #     形变 shape_speed -> stationary_shape_speed_mps
    #     旋转 yaw_speed   -> stationary_yaw_speed_mps
    #   2026-09-23 用当前代码回放 63 车次真实整帧查实：这里原来把三者都拿去比
    #   stationary_speed_mps（0.02），而检测器对形变/旋转按各自门槛（现场 0.10）放行
    #   -> 检测器给了测量许可、归档却拒收（本函数的「停稳原值未达到原许可门槛」）
    #   -> 终局器按原 ID 无限重放 -> **该道此后停测**。9-21 实测静止帧 69.9% 落在
    #   (0.02, 0.10] 之间，每条道前 3 辆车内卡死的概率 97%。
    #   9-21 当晚只把两个键加进了上面的键名表，这里的数值比较没跟上。
    #   门槛取的是检测器当时**实际生效**的值（service._gate_decision 用 getattr 取），
    #   配置填 0 时检测器已回退成 stationary_speed_mps，所以这里不用再做回退。
    #   复现与验证见 docs/60 第 7.5 节。
    if (thresholds["stationary_min_sec"] <= 0 or thresholds["stationary_speed_mps"] <= 0 or
            thresholds["stationary_shape_speed_mps"] <= 0 or thresholds["stationary_yaw_speed_mps"] <= 0 or
            thresholds["stationary_drift_m"] <= 0 or state["stationary_sec"] + 1e-9 < thresholds["stationary_min_sec"] or
            not 0 < thresholds["stationary_speed_window_sec"] <= thresholds["stationary_min_sec"] or
            not 0 < thresholds["stationary_max_gap_sec"] < thresholds["stationary_min_sec"] or
            state["speed_mps"] > thresholds["stationary_speed_mps"] or
            state["shape_speed_mps"] > thresholds["stationary_shape_speed_mps"] or
            state["yaw_speed_mps"] > thresholds["stationary_yaw_speed_mps"] or
            state["drift_m"] > thresholds["stationary_drift_m"]):
        raise ValueError("停稳原值未达到原许可门槛")
    policy = decision["frame_policy"]
    _keys(policy, "accepted packet_count_known", "完整帧策略")
    if policy["accepted"] is not True or policy["packet_count_known"] is not (decision["frame"]["packets"] > 0):
        raise ValueError("完整帧策略声明不一致")


def validate_runtime_records(meta, files, limits):
    """事务/导出使用的有界轻量核验；不解压NPZ或重新执行体积算法。"""
    if not runtime_members(meta):
        return None
    config = bounded_json(files["effective-configuration.json"], limits)
    _keys(config, "schema calib devices channel angle", "有效配置")
    if config["schema"] != "jt128-effective-configuration-v1":
        raise ValueError("有效配置schema不支持")
    for key in ("calib", "devices", "channel", "angle"):
        if type(config[key]) is not dict:
            raise ValueError("有效配置结构非法")
    if config["channel"].get("id") != meta["channel"]:
        raise ValueError("原有效配置不属于测量通道")
    config_hash = hashlib.sha256(files["effective-configuration.json"]).hexdigest()
    if config_hash != meta["configuration_hash"]:
        raise ValueError("原配置字节与测量摘要不一致")
    call = bounded_json(files["volume-call.json"], limits)
    schema = call.get("schema") if type(call) is dict else None
    v2 = schema == "volume-call-v2"
    v3 = schema == "volume-call-v3"
    extra = (" enter exit" if v3 else " enter" if v2 else "")
    _keys(call, _CALL_V1_KEYS + extra, "原体积调用")
    if schema not in ("volume-call-v1", "volume-call-v2", "volume-call-v3") or call["configuration_sha256"] != config_hash:
        raise ValueError("原体积调用schema或配置不一致")
    from .enter_volume import ENTER_FRAMES_MEMBER, EXIT_FRAMES_MEMBER, validate_record
    if (v2 or v3) and call["enter"] is not None:
        validate_record(call["enter"])
    if ((v2 or v3) and call["enter"] is not None) != (ENTER_FRAMES_MEMBER in files):
        raise ValueError("进场帧记录与原帧成员存在性不一致")
    if v3 and call["exit"] is not None:
        validate_record(call["exit"])
    if (v3 and call["exit"] is not None) != (EXIT_FRAMES_MEMBER in files):
        raise ValueError("离场帧记录与原帧成员存在性不一致")
    result = bounded_json(files["detection.xyz.meta.json"], limits)
    _keys(call["identity"], "measurement_id channel passage_id origin_token", "测量身份")
    for key in ("measurement_id", "channel", "passage_id"):
        if call["identity"][key] != meta[key]:
            raise ValueError("运行参数与测量身份不一致")
    if call["identity"]["origin_token"] != result.get("origin_token"):
        raise ValueError("运行参数与原结果token不一致")
    environment = call["algorithm"]
    fields = "contract python numpy"
    if isinstance(environment, dict) and "fingerprint" in environment:
        fields += " fingerprint"  # 旧归档字段只读兼容，不参与恢复决策。
    _keys(environment, fields, "算法绑定")
    if any(not isinstance(environment[k], str) or not 0 < len(environment[k]) <= 256
           for k in ("contract", "python", "numpy")):
        raise ValueError("算法身份非法")
    base = call["baseline"]
    _keys(base, "state version_id semantic_sha256 asset_sha256", "原基线")
    if base["version_id"] != meta["baseline_version_id"]:
        raise ValueError("原基线版本与测量不一致")
    if base["state"] not in ("available", "absent", "asset_unavailable"):
        raise ValueError("未知原基线状态")
    if (base["state"] == "available") != ("baseline.npz" in files):
        raise ValueError("原基线状态与资产存在性不一致")
    if base["state"] == "absent":
        if any(base[k] is not None for k in ("version_id", "semantic_sha256", "asset_sha256")):
            raise ValueError("无基线不能含基线绑定")
    else:
        _sha(base["semantic_sha256"])
    if base["state"] == "available":
        if hashlib.sha256(files["baseline.npz"]).hexdigest() != base["asset_sha256"]:
            raise ValueError("原基线资产与调用摘要不一致")
    elif base["asset_sha256"] is not None:
        raise ValueError("缺少原资产不能声明资产摘要")
    params = call["params"]
    _keys(params, _PARAMS_V1_KEYS + (" fill_bound" if (v2 or v3) else "") +
          (" shared_baseline" if "shared_baseline" in params else ""), "计算实参")
    if "shared_baseline" in params and params["shared_baseline"] is not True:
        raise ValueError("共享基线开关必须为true")
    if (v2 or v3) and type(params["fill_bound"]) is not bool:
        raise ValueError("遮挡上限补洞开关必须为布尔值")
    if (type(params["cells"]) is not list or not 1 <= len(params["cells"]) <= 16
            or params["fill_holes"] is not True or params["bed_evidence"] is not None or params["bed_scope"] is not None):
        raise ValueError("计算候选或底板授权不符合当前合同")
    for cell in params["cells"]:
        _number(cell)
        if cell <= 0:
            raise ValueError("网格尺度必须为正")
    for key in ("spread_target", "bed_shift_max_m", "floor_band_m", "bed_z", "dy"):
        _number(params[key], nullable=key in ("bed_z", "dy"))
    if not 0 <= params["spread_target"] <= 1 or params["bed_shift_max_m"] < 0 or params["floor_band_m"] <= 0:
        raise ValueError("计算实参超出范围")
    loaded = call["loaded"]
    if (loaded is None) != (base["state"] == "absent"):
        raise ValueError("载货输入与基线状态不一致")
    if loaded is not None:
        _keys(loaded, "source_rotation surface_sha256 frame_ids surface_members", "载货输入")
        _sha(loaded["surface_sha256"])
        align_extra_surfaces(loaded["source_rotation"], ())
    runtime = call["runtime"]
    if runtime is not None:
        required = {"schema", "status", "reason", "catalog_sha256", "registration", "range_status"}
        optional = {"source_rotation", "scope", "binding", "model", "model_sha256", "context"}
        if (type(runtime) is not dict or not required <= set(runtime) or set(runtime) - required - optional or
                runtime["schema"] != "volume-runtime-v1" or runtime["status"] not in
                ("unavailable", "rejected", "binding_not_found", "expired", "supplied")):
            raise ValueError("体积运行记录schema或状态非法")
        _sha(runtime["catalog_sha256"])
        _keys(runtime["registration"], "dy_m status reason", "配准记录")
        _number(runtime["registration"]["dy_m"])
        if runtime["registration"]["status"] not in ("unresolved", "resolved", "declared_fixed"):
            raise ValueError("未知配准状态")
        if "model" in runtime:
            from .volume_provider import VolumeCatalog
            catalog = VolumeCatalog.from_bytes(canonical(dict(schema="volume-provider-v1", models=[runtime["model"]])))
            if catalog.models[0].sha256 != runtime.get("model_sha256") or runtime["model"]["binding"] != runtime.get("binding"):
                raise ValueError("物理模型摘要或正式基线绑定不一致")
    if runtime != result.get("volume_runtime"):
        raise ValueError("体积参数与原结果运行证据不一致")
    context, scope = params["volume_context"], params["volume_scope"]
    if context is not None:
        context_from_record(context)
        scope_from_record(scope)
        if runtime is None or runtime.get("status") != "supplied" or runtime.get("context") != context or runtime.get("scope") != scope:
            raise ValueError("体积条件与实际供应记录不一致")
        if context["scope"] != scope:
            raise ValueError("体积条件scope不一致")
        grid_geometry(context["domain_xy"], context["support_cell_m"], context["support_origin_xy"])
    elif scope is not None:
        raise ValueError("无体积条件不能含授权scope")
    if runtime is not None:
        if loaded is not None and runtime.get("source_rotation") != loaded["source_rotation"]:
            raise ValueError("货面旋转与运行证据不一致")
        if "binding" in runtime:
            binding = runtime["binding"]
            for k, expected in (("configuration_sha256", config_hash), ("baseline_version_id", base["version_id"]),
                                ("baseline_sha256", base["semantic_sha256"]), ("channel_id", meta["channel"])):
                if binding[k] != expected:
                    raise ValueError("正式基线绑定不一致")
            if base["state"] == "available" and binding["baseline_asset_sha256"] != base["asset_sha256"]:
                raise ValueError("正式基线原资产绑定不一致")
        if "registration" in runtime and params["dy"] != runtime["registration"]["dy_m"] and not (
                params.get("shared_baseline") and params["dy"] is None and runtime["status"] != "supplied"):
            raise ValueError("实际配准实参与运行记录不一致")
    from .detect import VolumeResult
    _keys(call["original_result"], " ".join(VolumeResult().to_dict(raw=True)) +
          (" baseline_registration" if "baseline_registration" in call["original_result"] else ""), "原精度体积结果")
    if call["original_result"] != result.get("volume"):
        raise ValueError("原精度体积结果与测量结果不一致")
    gate = bounded_json(files["gate-summary.json"], limits)
    _keys(gate, "schema summary_only event_stream_complete identity configuration_sha256 input_refs decision manual_three_frames settle capture_instance epoch", "停稳摘要")
    if (gate["schema"] != "gate-summary-v1" or gate["summary_only"] is not True
            or gate["event_stream_complete"] is not False or gate["identity"] != call["identity"]
            or gate["configuration_sha256"] != config_hash):
        raise ValueError("停稳摘要身份或能力声明不一致")
    refs = gate["input_refs"]
    _keys(refs, "primary support extra_members", "摘要来源")
    if (refs["primary"] != meta["provenance"]["segments"] or refs["support"] != meta["volume_support"]["segments"]
            or refs["extra_members"] != meta["volume_support"]["extra_members"]):
        raise ValueError("停稳摘要与实际窗口不一致")
    if type(refs["primary"]) is not list or not refs["primary"]:
        # ★ primary 为空时 320 行的 [-1] 会抛 IndexError —— 终局器按类型名
        #   报错（文件内注释自述），现场会被"IndexError"误导。空窗口本身
        #   就是数据不完整，给出可读的拒绝理由（2026-09-23，低严重度批次）。
        raise ValueError("停稳摘要的主观测窗口为空，禁止凭空补造测量")
    last = refs["primary"][-1]
    if (type(gate["epoch"]) is not int or gate["epoch"] != last["epoch"] or
            not isinstance(gate["capture_instance"], str) or not gate["capture_instance"]):
        raise ValueError("停稳摘要活动代次不一致")
    decision = gate["decision"]
    detection = config["calib"].get("detection")
    if type(detection) is not dict:
        raise ValueError("原有效配置缺少判定参数")
    _validate_decision(decision, detection)
    if decision["frame"] != last["frame"] or decision["received_monotonic"] != last["received_monotonic"]:
        raise ValueError("停稳许可不是末帧真实观测")
    if decision["stability"]["n_points"] != last["input_rows"][1] - last["input_rows"][0]:
        raise ValueError("许可点数与末帧输入行不一致")
    settle = gate["settle"]
    if type(gate["manual_three_frames"]) is not list or type(settle) is not dict:
        raise ValueError("人工帧或沉降摘要类型非法")
    if settle.get("kind") == "automatic":
        _keys(settle, "kind applicable started_monotonic finished_monotonic waited_sec required_sec accumulate", "沉降摘要")
        for key in ("started_monotonic", "finished_monotonic", "waited_sec", "required_sec"):
            _number(settle[key])
        if (settle["applicable"] is not True or type(settle["accumulate"]) is not bool or
                settle["required_sec"] < 0 or settle["waited_sec"] < settle["required_sec"] or
                settle["finished_monotonic"] - settle["started_monotonic"] != settle["waited_sec"] or
                settle["finished_monotonic"] < decision["received_monotonic"] or gate["manual_three_frames"]):
            raise ValueError("沉降许可与实际等待不一致")
    elif settle.get("kind") == "manual":
        _keys(settle, "kind applicable command baseline_quality", "人工许可")
        command = settle["command"]
        _keys(command, "requestId channelId passageId kind confirmedEmpty", "人工命令")
        frames = gate["manual_three_frames"]
        if (settle["applicable"] is not False or len(frames) != 3 or len(refs["support"]) != 3 or
                command["channelId"] != meta["channel"] or command["passageId"] != meta["passage_id"] or
                command["kind"] not in ("remeasure", "baseline-capture") or type(command["confirmedEmpty"]) is not bool or
                not isinstance(command["requestId"], str) or not command["requestId"]):
            raise ValueError("人工许可与车次或三帧来源不一致")
        for member, segment in zip(frames, refs["support"]):
            _validate_decision(member, detection)
            if member["frame"] != segment["frame"] or member["received_monotonic"] != segment["received_monotonic"]:
                raise ValueError("人工三帧许可被替换或错序")
        if frames[-1] != decision:
            raise ValueError("人工末帧许可不一致")
        quality = settle["baseline_quality"]
        if command["kind"] == "baseline-capture":
            _keys(quality, "ok r_min r_all n why", "人工候选一致性")
            if (command["confirmedEmpty"] is not True or quality["ok"] is not True or
                    type(quality["n"]) is not int or quality["n"] != 3 or type(quality["r_all"]) is not list or
                    len(quality["r_all"]) != 3 or not isinstance(quality["why"], str)):
                raise ValueError("人工候选缺少实际三帧一致性许可")
            for value in quality["r_all"] + [quality["r_min"]]:
                _number(value)
            if quality["r_min"] != min(quality["r_all"]) or quality["r_min"] < .95:
                raise ValueError("人工候选原精度一致性未达到门槛")
        elif quality is not None:
            raise ValueError("重测命令不能包含候选一致性许可")
    else:
        raise ValueError("未记录自动或人工运行许可，不能形成可复算新归档")
    return call


def restore_inputs(archive, limits):
    """完成预算和包内校验后恢复输入；不读取当前配置/缓存。"""
    files, meta = dict(archive.bundle.files), archive.metadata
    call = validate_runtime_records(meta, files, limits)
    if call is None:
        return None
    from .load_radar_contract import _read_baseline
    from .detect import LoadedResult
    base = _read_baseline(files["baseline.npz"]) if call["baseline"]["state"] == "available" else None
    if base is not None and baseline_digest(base) != call["baseline"]["semantic_sha256"]:
        raise ValueError("原资产解码内容与实际基线不一致")
    primary, support = archive.window(), archive.window(volume=True)
    extra_members = tuple(meta["volume_support"]["extra_members"])
    ids, members = surface_lineage(primary, support, extra_members)
    loaded, extras = None, ()
    if call["loaded"] is not None:
        item = call["loaded"]
        if canonical([ids, members]) != canonical([item["frame_ids"], item["surface_members"]]):
            raise ValueError("货面用途与实际原帧不一致")
        surfaces = align_extra_surfaces(item["source_rotation"], (primary.input_frame().xyz,) +
            tuple(support.pairs[i].cropped.frame().xyz for i in extra_members))
        if surface_digest(surfaces) != item["surface_sha256"]:
            raise ValueError("重建货面摘要不一致，不能声称原输入复现")
        loaded, extras = LoadedResult(cloud=surfaces[0], source_rotation=np.asarray(item["source_rotation"])), surfaces[1:]
    context, scope = call["params"]["volume_context"], call["params"]["volume_scope"]
    if context is not None:
        gate = bounded_json(files["gate-summary.json"], limits)
        expected = dict(scope, channel_id=meta["channel"], passage_id=meta["passage_id"],
                        configuration_sha256=meta["configuration_hash"], baseline_version_id=meta["baseline_version_id"],
                        measured_at=primary.pairs[-1].raw.ts, frame_ids=ids, surface_members=members,
                        capture_epoch=gate["capture_instance"] + ":" + str(gate["epoch"]))
        if canonical(expected) != canonical(scope) or context["baseline_sha256"] != call["baseline"]["semantic_sha256"] or context["surface_sha256"] != call["loaded"]["surface_sha256"]:
            raise ValueError("体积条件不属于实际输入或测量身份")
    return call, base, loaded, extras


def restore_enter_frames(archive, call):
    """v2/v3 调用实际用过的进场帧（原字节）；没有记录返回 ()。不读当前目录里的进场帧文件。"""
    record = call.get("enter") if call.get("schema") in ("volume-call-v2", "volume-call-v3") else None
    if record is None:
        return ()
    from .enter_volume import ENTER_FRAMES_MEMBER, restore_frames
    frames = restore_frames(record, dict(archive.bundle.files)[ENTER_FRAMES_MEMBER])
    first = archive.window().pairs[0].raw
    if any(f.gen != first.gen or f.ts >= first.ts for f in frames):
        raise ValueError("进场帧不在本次停稳观测之前或跨源代次")
    return frames


def restore_exit_frames(archive, call):
    """v3 调用实际用过的离场帧（原字节）；没有记录返回 ()。"""
    record = call.get("exit") if call.get("schema") == "volume-call-v3" else None
    if record is None:
        return ()
    from .enter_volume import EXIT_FRAMES_MEMBER, restore_frames
    frames = restore_frames(record, dict(archive.bundle.files)[EXIT_FRAMES_MEMBER])
    last = archive.window().pairs[-1].raw
    if any(f.gen != last.gen or f.ts <= last.ts for f in frames):
        raise ValueError("离场帧不在本次停稳观测之后或跨源代次")
    return frames
