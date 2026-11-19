# -*- coding: utf-8 -*-
"""双雷达采集管理器 —— 载货雷达 CH4 判到停稳、测出结果后，自动驱动双雷达采集。

依据：`docs/68-CH4双雷达采集触发与进度监控方案.md`（v2）。
接口契约：`../CarPointCloudWebApi/docs/manual-capture-integration-20260924.md`。
进度呈现规则照抄后台页面 `CarPointCloudWebApi/wwwroot/pointcloud-viewer/js/radar-live.js`。

它在整条链里的位置
------------------

    停稳记录器主线程（每 2 s 读 /api/state）          本模块的后台线程（CH4 专用）
      ├─ 判进场 / 停稳 / 测完 / 开始移动 / 离场  ──事件──>  ├─ 读台账（去重）-> 前置检查 -> 负载闸门
      ├─ 打印车辆事件行                                    ├─ 固定 Token 查询状态
      ├─ 写台账（进场 / 离场）                             ├─ 受理（同键重试规则见方案 §5）
      └─ say()（加锁）  <────────打印─────────────────────  ├─ 轮询进度（2 s，只在变化时打印）
                                                           └─ 写终局流水 / 更新台账（方案 §8）

四条必须守住的红线
------------------

1. **主线程不做任何 HTTP**。WebApi 慢或挂了，停稳记录照常。
   本模块的每一次网络调用都在后台线程里（`CaptureSession` 被线程驱动）。
2. **一车只触发一次**（方案 D2）。去重判据落在**持久化台账**上（方案 D7）：
   停在原地的车每 180 秒被服务超时重测一次，会产生新的测量号；
   记录器本身也会被看门狗重启（9-24 当天拉起 5 次）。
   只靠进程内存会把同一辆车采两次 —— 那会往 DM8 写两条记录。
3. **载货链优先**（方案 D9）。任何一步发现载货链负载异常（方案 §4.4 两道门），
   本功能主动让步：不发起、不重试、不报警，只留一行 `跳过`。
4. **绝不换幂等键**。超时 / 502 / 504 之后重试必须用同一个键（方案 §5、接口文档 §6）。
   换键 = 重复采集，且难以发现。

符号约定（GBK 控制台，见 `tests/test_encoding.py`）
---------------------------------------------------

    ● 已完成   ◎ 进行中   ○ 未开始   × 失败   √ 成功/可以放车   ▲ 警告   -> 发起
    ▲ 后台页面用的几个符号（U+25CD / U+2715 / U+26A0）**编不进 GBK**，这里一律换成上面的等价符号。

测试
----

`tests/test_dual_radar_capture.py` 用假 WebApi 起在 `http.server` 上，
时钟与响铃都可注入，**不真 sleep、不起线程** —— 全部走 `CaptureSession` 的同步接口。
"""
from __future__ import annotations

import collections
import json
import math
import os
import queue
import re
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

# ═══════════════════════════════════════════════════════════ 阶段表（照 radar-live.js）

STAGE_FLOW = (
    ("waiting", "等待安全完整场"),
    ("captured", "双雷达点云已捕获"),
    ("saved", "两份 PCD 已落盘"),
    ("queued", "已配对，排队分析"),
    ("analyzing", "Analyzer 正在融合计算"),
    ("completed", "计算完成并写入数据库"),
)
STAGE_LABELS = dict(STAGE_FLOW)
STAGE_ORDER = {key: index for index, (key, _) in enumerate(STAGE_FLOW)}

# 失败 / 特殊终局。`completed_without_dimensions` 不是故障，但同样是终局。
FAILURE_STAGES = {
    "capture_failed": "采集失败",
    "persistence_failed": "PCD 落盘失败",
    "analysis_failed": "分析失败",
    "completed_without_dimensions": "分析完成，本次不产出尺寸",
}
# "失败前已完成几步"（对应 radar-live.js 的 failedBoundary）。
FAILED_BOUNDARY = {
    "capture_failed": 1,
    "persistence_failed": 2,
    "analysis_failed": 4,
    "completed_without_dimensions": 5,
}

SAVED_ORDER = STAGE_ORDER["saved"]

# ═══════════════════════════════════════════════════════════ 约束与默认值

PLACEHOLDER_PLATES = frozenset(
    ("UNKNOWN", "NULL", "N/A", "NA", "NONE", "无车牌", "未知", "未知车牌"))
PLATE_MAX_LEN = 32                       # 与 DM8 VARCHAR(32) 对齐（接口文档 §3）
PLATE_KEY_RE = re.compile(r"^[A-Za-z0-9_\-:.]{1,64}$")
KEY_PREFIX = "lr-"

RESUME_UNKNOWN_SEC = 35.0                # 自检一：unknown 持续多久同键重投（radar-live.js）
AWAITING_DB_WARN_SEC = 45.0              # 自检二：等数据库记录多久后警告（radar-live.js）
RESPONSE_WARN_BYTES = 64 * 1024          # 响应体首次超过它就警告一次（方案 §6）
POLL_FAIL_WARN_EVERY = 10                # 连续轮询失败这么多次打印一次警告（方案 §6）
LEAVE_CANCEL_GRACE_SEC = 30.0            # 离场取消后最多再轮询多久拿终局（方案 §7.2）
MOVING_REPEAT_SEC = 5.0                  # 车在动、还没采到：每 5 秒重复报警（方案 D6）
UNSURE_RETRY_MAX = 2                     # 502/504/连接失败：同键重试上限（方案 §5）
UNSURE_RETRY_SEC = 3.0
BUSY_RETRY_SEC = 3.0                     # Worker busy：同键重试间隔（方案 §5）
DEFAULT_TOTAL_TIMEOUT = 180.0            # 受理后仍未终局就停（方案 §6）
DEFAULT_POLL_SEC = 2.0                   # 方案 D10：不用页面的 1.5 秒
DEFAULT_LEDGER_TTL = 1800.0              # 台账条目最长开着多久（方案 §4.2，30 分钟）
DEFAULT_GUARD_FPS = 7.5                  # 方案 §4.4 门 2
DEFAULT_GUARD_INCOMPLETE = 0.10

DEFAULT_CAPTURE_WEBAPI = "http://127.0.0.1:5031"
DEFAULT_TOKEN_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                 "config", "dual-radar-recorder.json")
CAPTURE_CHANNEL = "CH4"
RECORD_DIR = os.path.join("data", "field-log")
LEDGER_NAME = "双雷达采集-台账.json"


# ═══════════════════════════════════════════════════════════ 纯函数


def normalize_plate(text):
    """车牌规范化。返回规范化后的车牌，不合法返回 None（方案 §4.3）。

    与接口文档 §3 一致：去掉**全部**空白（含中间空格）后转大写、长度 <= 32、不含控制字符。
    另加占位值拦截（`未知车牌` 等）—— 本项目不判断车牌是模拟值还是真实值（方案 D3）。
    """
    if text is None:
        return None
    compact = "".join(ch for ch in str(text) if not ch.isspace())
    if not compact or len(compact) > PLATE_MAX_LEN:
        return None
    if any(ord(ch) < 32 or ord(ch) == 127 for ch in compact):
        return None
    if compact.upper() in PLACEHOLDER_PLATES:
        return None
    return compact.upper()


def idempotency_key(measurement_id):
    """`lr-<测量号>`。同一辆车的所有重试都用这一个键（方案 §4.5）。"""
    return KEY_PREFIX + str(measurement_id or "").strip()


def key_is_valid(key):
    """幂等键格式：`^[A-Za-z0-9_\\-:.]{1,64}$`（接口文档 §3）。"""
    return bool(PLATE_KEY_RE.match(key or ""))


def _as_number(value):
    """能当数字用的值；bool 不算（JSON 里 true 不是 1）。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


# ── 进度渲染 ────────────────────────────────────────────────


def render_steps(progress):
    """阶段步骤条。格式照 `renderProgress`，步骤名用方案 §2.4 的名称。

    方案 §6.1 的示例把步骤名写短了（"两份PCD已落盘"），这里按方案自己的
    要求（"步骤名用 §2.4 的名称"）用完整名称。
    """
    progress = progress or {}
    stage = progress.get("stage")

    if stage == "pending_review":
        parts = []
        for key, label in STAGE_FLOW[:-1]:
            parts.append("●" + ("融合与计算完成" if key == "analyzing" else label))
        parts.append("◎待人工审核（未入库）")
        return "│ " + "".join(parts)

    failure = FAILURE_STAGES.get(stage) if stage else None
    active = STAGE_ORDER.get(stage, -1)
    boundary = FAILED_BOUNDARY.get(stage, 0)
    cameras = [str(c) for c in (progress.get("capturedCameras") or []) if c]

    parts = []
    for index, (key, label) in enumerate(STAGE_FLOW):
        if failure:
            icon = "●" if index < boundary else "○"
        elif active >= 0 and index < active:
            icon = "●"
        elif index == active:
            icon = "●" if stage == "completed" else "◎"
        else:
            icon = "○"
        # 只在"已捕获"这一步正进行时标注相机（与方案 §6.1 的示例一致）。
        if key == "captured" and icon == "◎" and cameras:
            label = label + "（" + "、".join(cameras) + "）"
        parts.append(icon + label)

    if failure:
        parts.append("×" + failure)
    dimensions = progress.get("dimensions") or {}
    if stage == "completed" and dimensions.get("reviewStatus") == "pending":
        parts.append("◎已入库，待人工审核")
    return "│ " + "".join(parts)


def format_cm(value):
    """数据库口径：厘米（`formatCm`）。"""
    number = _as_number(value)
    return "—" if number is None else "%.1f cm" % number


def format_meters(value):
    """未入库计算值口径：米（`formatMeters`）。"""
    number = _as_number(value)
    if number is None or not math.isfinite(number):
        return "未算出"
    return "%.3f m" % number


def describe_rib_count(count):
    """现场车厢拉筋为 2~6 根；超出范围标出来（页面上是个警告符号，GBK 编不进，用 `▲`）。"""
    if count is None:
        return "未确认"
    return ("%d 根" % count) if 2 <= count <= 6 else ("%d 根 ▲" % count)


def format_rib_gap(value, formatter):
    """`null` 是未能量取的间距，不能经数值化变成零（页面 `formatRibGap`）。"""
    number = _as_number(value)
    if number is None or not math.isfinite(number) or number < 0:
        return "缺测"
    return formatter(number)


def measurement_range_cm(values):
    """把 `[a, b]`（米）压成 `"a～b"`（厘米、一位小数）；不合法返回 ""。"""
    if not isinstance(values, (list, tuple)) or len(values) != 2:
        return ""
    low, high = _as_number(values[0]), _as_number(values[1])
    if low is None or high is None:
        return ""
    if low < 0 or high <= low or high > 10:
        return ""
    return "～".join(str(round(x * 1000) / 10) for x in (low, high))


def length_measurement_note(progress):
    """内长测线口径说明（页面 `lengthMeasurementNote`）。没有就返回 ""。

    ▲ 这条**不能省**：本项目"车厢内长"有多个口径（底板分位 / 硬边界 / 物理内长 /
      `dims[1]` / `box`），页面上显示的 `trailerLength` 与卷尺量出来的值不一样。
      不把口径说明带上，现场会拿它直接和卷尺比，然后认为系统算错了。
    """
    analysis = (progress or {}).get("analysis") or {}
    if analysis.get("stage") != "completed":
        return ""
    measurement = (analysis.get("measurementStatus") or {}).get("lengthMeasurement") or {}
    if measurement.get("valid") is not True:
        return ""
    if measurement.get("definition") != "same_side_lower_band_longitudinal_projection":
        return ""
    height = measurement_range_cm(measurement.get("heightAboveFloorM"))
    inward = measurement_range_cm(measurement.get("inwardBandM"))
    if not height or not inward:
        return ""
    return ("内长测线：同侧下部，底板以上 %s cm，侧壁参考面向内 %s cm（纵向投影）。"
            % (height, inward))


def collect_result_fields(progress):
    """结果字段归一（页面 `collectResultFields`）。本次没有任何尺寸时返回 None。

    ▲ 待审核计算值与明细必须**同源**：`showReview` 是复合条件，不是"`dimensions` 为空"。
      简化它会在分析失败时把上一轮数据渲染成有尺寸结果（页面源码专门防的就是这个）。
    """
    progress = progress or {}
    stage = progress.get("stage")
    analysis = progress.get("analysis") or {}
    dimensions = progress.get("dimensions")
    stored = (dimensions or {}).get("dimensions")
    stored_review = bool(dimensions and dimensions.get("reviewStatus") == "pending")

    if isinstance(stored, dict) and "trailerLength" in stored:
        # Analyzer 的约定：拉筋按前壁→后壁排序，N 根拉筋输出 N+1 段，段数不是根数。
        gaps = [format_rib_gap(item.get("distToPrev") if isinstance(item, dict) else None,
                              format_cm)
                for item in (stored.get("ribs") or [])]
        count = len(gaps) - 1 if gaps else 0
        return {
            "pending": stored_review,
            "cabToFront": format_cm(stored.get("cabToTrailerFrontDist")),
            "innerLength": format_cm(stored.get("trailerLength")),
            "width": format_cm(stored.get("trailerWidth")),
            "innerHeight": format_cm(stored.get("trailerBodyHeight")),
            "floorHeight": format_cm(stored.get("trailerBottomToGroundHeight")),
            "ribCount": count,
            "ribCountText": describe_rib_count(count),
            "ribGaps": gaps,
            "plates": [{"id": (plate or {}).get("plateId"),
                        "high": format_cm((plate or {}).get("highestToGround")),
                        "low": format_cm((plate or {}).get("lowestToGround"))}
                       for plate in (stored.get("highLowPlates") or [])],
            "plateDrop": format_cm(stored.get("highLowPlateHeight")),
            "recordId": dimensions.get("recordId"),
        }

    show_review = (stage in ("pending_review", "completed") and (stage == "pending_review" or stored_review)) \
        and analysis.get("stage") == "completed" \
        and (analysis.get("measurementStatus") or {}).get("reviewStatus") == "pending"
    if not show_review:
        return None

    review = analysis.get("reviewMeasurements") or {}
    carriage = (review.get("dimensions") or {}).get("carriage") or {}
    gaps_raw = review.get("ribIntervals")
    gaps = list(gaps_raw) if isinstance(gaps_raw, list) else []
    # ribCount 已经是根数；ribIntervals 是 N+1 段，兜底时要减 1 才是根数。
    if review.get("ribCount") is None:
        count = len(gaps) - 1 if gaps else 0
    else:
        count = review.get("ribCount")
    plates = review.get("highLowPlates") or []
    return {
        "pending": True,
        "cabToFront": format_meters(review.get("cabToTrailerFrontDist")),
        "innerLength": format_meters(carriage.get("length")),
        "width": format_meters(carriage.get("width")),
        "innerHeight": format_meters(carriage.get("height")),
        "floorHeight": format_meters(review.get("trailerBottomToGroundHeight")),
        "ribCount": count,
        "ribCountText": describe_rib_count(count),
        "ribGaps": [format_rib_gap(value, format_meters) for value in gaps],
        "plates": [{"id": (plate or {}).get("plateId", index + 1),
                    "high": format_meters((plate or {}).get("highestToGround")),
                    "low": format_meters((plate or {}).get("lowestToGround"))}
                   for index, plate in enumerate(plates)],
        "plateDrop": format_meters(review.get("highLowPlateHeight")),
        "recordId": (progress.get("dimensions") or {}).get("recordId"),
    }


def result_line(fields):
    """方案 §6.1 的结果行（七项，单位随来源）。"""
    return ("车头到车厢前内壁 %s｜车厢内长 %s｜车厢宽（外沿）%s｜车厢内高 %s｜"
            "底板离地 %s｜拉筋 %s｜高低板差 %s"
            % (fields["cabToFront"], fields["innerLength"], fields["width"],
               fields["innerHeight"], fields["floorHeight"], fields["ribCountText"],
               fields["plateDrop"]))


# ── 受理分支 ────────────────────────────────────────────────


# kind 取值：accepted / duplicate / processing / manual / rejected / busy / uncertain / locked
AcceptDecision = collections.namedtuple("AcceptDecision", "kind workflow_id message text")


def classify_accept(http_status, body):
    """受理响应 -> 该做什么（方案 §5 表，逐行照接口文档）。

    `http_status == 0` 表示连接失败 / 超时（结果不确定），由客户端这样上报。
    """
    body = body if isinstance(body, dict) else {}
    data = body.get("data") if isinstance(body.get("data"), dict) else {}
    message = str(body.get("message") or "")
    workflow_id = str(data.get("workflowId") or "")
    status = str(data.get("status") or "")

    if http_status == 200:
        if bool(data.get("duplicate")) or status == "already_accepted":
            return AcceptDecision("duplicate", workflow_id, message, "")
        return AcceptDecision("accepted", workflow_id, message, "")

    if http_status == 202:
        return AcceptDecision("processing", workflow_id, message, "")

    if http_status == 400:
        return AcceptDecision("rejected", "", message, message)

    if http_status == 401:
        return AcceptDecision("rejected", "", message,
                              "WebApi 未接受 CH4 采集标记，请确认服务版本")

    if http_status == 403:
        return AcceptDecision("rejected", "", message, "WebApi 拒绝 CH4 采集请求")

    if http_status == 409:
        if status == "needs_manual_confirmation":
            return AcceptDecision("manual", workflow_id, message, "")
        if "相同 Idempotency-Key 已用于车牌" in message:
            return AcceptDecision("rejected", "", message, message)
        if "已存在空车档案" in message:
            return AcceptDecision("rejected", "", message, message)
        return AcceptDecision("busy", "", message, "")

    if http_status == 423:
        return AcceptDecision("locked", "", message,
                              "采集服务拒绝受理，本班次停止双雷达采集，请核对 WebApi 状态")

    if http_status == 503:
        if message.startswith("采集安全门禁未满足：acquisition_unverified"):
            return AcceptDecision("rejected", "", message,
                                  "采集未启动：完整场采集条件尚未确认")
        return AcceptDecision("rejected", "", message, message)

    if http_status in (0, 502, 504):
        return AcceptDecision("uncertain", "", message, "")

    return AcceptDecision("rejected", "", message, "HTTP %s：%s" % (http_status, message))


def _reject_reason(http_status, message):
    """把"放弃"的原因压成短标签，拼在 `not_accepted:` 后面（方案 §8.2）。"""
    if http_status == 400:
        return "请求不合法"
    if http_status == 401:
        return "认证失败"
    if http_status == 403:
        return "采集请求被拒绝"
    if http_status == 409:
        if "相同 Idempotency-Key 已用于车牌" in (message or ""):
            return "幂等键车牌不一致"
        if "已存在空车档案" in (message or ""):
            return "空车档案未确认覆盖"
        return "业务拒绝"
    if http_status == 503:
        if (message or "").startswith("采集安全门禁未满足：acquisition_unverified"):
            return "acquisition_unverified"
        return "设备状态"
    return "HTTP %s" % http_status


# Worker `DualRadarScanOnceState`（`CarPointCloudLadarWorker/Models/DualRadarScanOnceModels.cs`）
# 序列化成小写。▲ "可以受理新请求"有**两个**状态：持续扫描模式下是 `idle`（现场配置
# `ScanOnceOperatingMode = ContinuousSoftwareGate`），硬件待机模式下是 `ready`。
# 原来只认 `idle`，换成待机模式后每次都被当成"忙"，窗口里一直打错误的"正忙"提示。
SCAN_ACCEPTING = frozenset(("idle", "ready"))
SCAN_BUSY = frozenset(("opening", "scanning", "stopping"))
SCAN_UNAVAILABLE = {"disabled": "双雷达单次扫描功能未启用",
                    "initializing": "双雷达还在初始化"}


def _dual_status_detail(body):
    """`(判定, 原因)`。判定取值见 `dual_status`；原因给窗口那一行用。"""
    if not isinstance(body, dict):
        return "unavailable", "状态接口没有返回有效内容"
    data = body.get("data") if isinstance(body.get("data"), dict) else body
    scan = data.get("scan") if isinstance(data.get("scan"), dict) else {}
    if scan.get("enabled") is False:
        return "unavailable", SCAN_UNAVAILABLE["disabled"]
    state = str(scan.get("state") or "").lower()
    if state in SCAN_UNAVAILABLE:
        return "unavailable", SCAN_UNAVAILABLE[state]
    if state in SCAN_BUSY:
        return "busy", "双雷达正在别的采集会话里（%s）" % state
    preview = data.get("preview") if isinstance(data.get("preview"), dict) else {}
    if preview.get("available") is False:
        reason = str(preview.get("unavailableReason") or "")
        return "unavailable", "双雷达实时预览不可用" + ("：" + reason if reason else "")
    return "ok", ""


def dual_status(body):
    """`GET /radar-live/status` 的判定（方案 §4.4 门 1）。

    返回 `"ok"` / `"busy"`（Worker 正在别的会话里，照发、交给 §5 的 busy 逻辑）/
    `"unavailable"`（功能未启用、初始化中、预览不可用、读不到 -> 跳过本次）。
    读不到 `scan.state` 时放行（别自己吓自己）。
    """
    return _dual_status_detail(body)[0]


def load_guard(channel_state, cfg):
    """载货链负载闸门（方案 §4.4 门 2）。放行返回 None，否则返回跳过原因。

    取值缺失 / 不是数字时**不拦** —— 宁可放行也不要因为读不到指标而永远不采。
    """
    if cfg is not None and not getattr(cfg, "guard_enabled", True):
        return None
    state = channel_state or {}
    # ★ 帧率用记录器按相邻两次采样的 `frames` 差算出的 `recent_fps`（近 2 秒）。
    #   `/api/state` 里的 `fps` 是 `frames / 数据源启动以来的秒数` —— **开机以来的平均值**
    #   （`radar_source.SourceStats.fps`），服务跑了几个小时后帧率掉到一半它也几乎不动，
    #   拿它当闸门等于没有闸门（2026-09-25 审查查实）。只在没有近期值时才退回它。
    fps = _as_number(state.get("recent_fps"))
    if fps is None:
        fps = _as_number(state.get("fps"))
    if fps is not None and fps < cfg.guard_fps:
        return "fps %.1f < %.1f" % (fps, cfg.guard_fps)
    incomplete = _as_number(state.get("incomplete_ratio"))
    if incomplete is not None and incomplete > cfg.guard_incomplete:
        return "残缺帧 %.1f%% > %.1f%%" % (100 * incomplete, 100 * cfg.guard_incomplete)
    rate = _as_number((state.get("source") or {}).get("packet_rate"))
    if rate is not None and rate == 0:
        return "packet_rate 0（雷达断流）"
    return None


def leave_action(progress, accepted):
    """车辆离场时该做什么（方案 §7.2 / D5）。返回 `(action, stage)`。

    action:
        `"never"`  离场时还没受理 -> 放弃
        `"cancel"` 还在 waiting/unknown -> 取消本次采集
        `"done"`   已经终局 -> 不用管
        `"watch"`  已捕获/已落盘 -> 不取消，继续盯到落盘（数据有效，取消反而丢数据）
    """
    if not accepted:
        return ("never", None)
    progress = progress or {}
    stage = progress.get("stage")
    if progress.get("terminal"):
        return ("done", stage)
    if stage is None or stage in ("unknown", "waiting"):
        return ("cancel", stage)
    return ("watch", stage)


def needs_save_selfcheck(last_result):
    """服务没开落盘时 `measurement_id` 永远不产生（方案 §4.6）。

    这个失败模式是**静默**的 —— 触发会永不发生，所以必须显式喊出来。
    """
    if not isinstance(last_result, dict) or not last_result:
        return False
    return not last_result.get("measurement_id")


def matched_result(channel_state, measurement_id):
    """`/api/state` 里**属于这一份测量**的 `last_result`；对不上号返回 None。

    ▲ `last_result` 在新车进场后仍是**上一辆车**的（服务只在下一份结果提交时才换），
      所以取车牌 / `duplicate_of` 之前必须先核对测量号，否则会把上一辆的牌安到这辆车上。
    """
    last = (channel_state or {}).get("last_result")
    mid = str(measurement_id or "")
    if isinstance(last, dict) and mid and str(last.get("measurement_id") or "") == mid:
        return last
    return None


def duplicate_source(channel_state, measurement_id):
    """服务标给这份测量的 `duplicate_of`（同一辆车被超时强开会话重测，指向上一份）；没有返回 ""。"""
    result = matched_result(channel_state, measurement_id)
    return str((result or {}).get("duplicate_of") or "")


def short_id(value, keep=8):
    """窗口里打印用的截断编号。"""
    text = str(value or "")
    return text if len(text) <= keep else text[:keep]


# ═══════════════════════════════════════════════════════════ 配置


class CaptureConfig:
    """把方案 §4.6 的命令行项收在一处，默认值就是方案里的默认值。"""

    def __init__(self, **kw):
        self.base_url = DEFAULT_CAPTURE_WEBAPI
        self.token_file = DEFAULT_TOKEN_FILE
        self.channel = CAPTURE_CHANNEL
        self.poll_sec = DEFAULT_POLL_SEC
        self.total_timeout = DEFAULT_TOTAL_TIMEOUT
        self.ledger_ttl = DEFAULT_LEDGER_TTL
        self.guard_fps = DEFAULT_GUARD_FPS
        self.guard_incomplete = DEFAULT_GUARD_INCOMPLETE
        self.guard_enabled = True
        self.bell = True
        self.record_dir = RECORD_DIR
        self.capture_timeout = 20.0
        self.progress_timeout = 10.0
        self.cancel_timeout = 10.0
        self.status_timeout = 5.0
        self.unsure_retries = UNSURE_RETRY_MAX
        self.unsure_interval = UNSURE_RETRY_SEC
        self.busy_retry_sec = BUSY_RETRY_SEC
        self.leave_cancel_grace = LEAVE_CANCEL_GRACE_SEC
        self.resume_unknown_sec = RESUME_UNKNOWN_SEC
        self.awaiting_db_warn_sec = AWAITING_DB_WARN_SEC
        self.response_warn_bytes = RESPONSE_WARN_BYTES
        for name, value in kw.items():
            if not hasattr(self, name):
                raise AttributeError("CaptureConfig 不认识的参数：%s" % name)
            setattr(self, name, value)

    def ledger_path(self):
        return os.path.join(self.record_dir, LEDGER_NAME)

    def record_path(self, date_text):
        return os.path.join(self.record_dir, "双雷达采集-%s.jsonl" % date_text)


# ═══════════════════════════════════════════════════════════ 车辆台账（去重）

_EMPTY_ENTRY = {"state": "open", "opened_at": 0.0, "closed_at": None,
                "measurement_id": "", "idempotency_key": "", "workflow_id": "",
                "last_stage": None, "terminal": False}

# "已经处理过的测量号"留多久、最多留几条（`duplicate_of` 链去重用，见 `VehicleLedger.mark_handled`）
HANDLED_KEEP_SEC = 24 * 3600.0
HANDLED_KEEP_MAX = 500


def _as_time(value):
    """台账里的时刻 -> 秒；缺失 / 坏值一律当 0（不许因为一个坏字段把后台线程打死）。"""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    return number if math.isfinite(number) else 0.0


class VehicleLedger:
    """`data/field-log/双雷达采集-台账.json`：以车牌为键的"车次是否还开着"（方案 §8.1）。

    ▲ 读写与 `say()` **共用同一把锁**（同一个文件，不允许两处并发写）。
      整份覆写：条目少（CH4 一天几十条），覆写成本可忽略。
    """

    def __init__(self, path, lock, ttl=DEFAULT_LEDGER_TTL):
        self.path = path
        self._lock = lock
        self.ttl = float(ttl)

    # ── 内部 ──
    def _read_doc_locked(self):
        try:
            with open(self.path, encoding="utf-8") as fh:
                data = json.load(fh)
        except (OSError, ValueError):
            data = {}
        data = data if isinstance(data, dict) else {}
        entries = data.get("entries")
        handled = data.get("handled")
        return {"entries": entries if isinstance(entries, dict) else {},
                "handled": handled if isinstance(handled, dict) else {}}

    def _read_locked(self):
        return self._read_doc_locked()["entries"]

    def _write_locked(self, entries, handled=None):
        if handled is None:                 # 只改条目时原样保留"已处理测量号"
            handled = self._read_doc_locked()["handled"]
        folder = os.path.dirname(self.path)
        if folder:
            os.makedirs(folder, exist_ok=True)
        temp = self.path + ".tmp"
        with open(temp, "w", encoding="utf-8") as fh:
            json.dump({"version": 1, "entries": entries, "handled": handled}, fh,
                      ensure_ascii=False, indent=1, sort_keys=True)
        os.replace(temp, self.path)

    def _entry(self, plate, mutate):
        with self._lock:
            entries = self._read_locked()
            entry = entries.get(plate)
            if not isinstance(entry, dict):
                entry = dict(_EMPTY_ENTRY)
            mutate(entry)
            entries[plate] = entry
            self._write_locked(entries)

    # ── 对外 ──
    def open_vehicle(self, plate, t):
        """进场（主线程调用）。

        ▲ 已存在且仍开着的条目**只刷新时间，不清字段** —— 记录器重启后会把停在
          原地的同一辆车重新记一次"进场"，清掉测量号就等于把去重判据丢了。
        """
        def mutate(entry):
            if entry.get("state") == "open":
                entry["opened_at"] = t
                return
            entry.update(state="open", opened_at=t, closed_at=None,
                         measurement_id="", idempotency_key="", workflow_id="",
                         last_stage=None, terminal=False, triggered_at=None)
        self._entry(plate, mutate)

    def close_vehicle(self, plate, t):
        """离场（主线程调用）。"""
        self._entry(plate, lambda entry: entry.update(state="closed", closed_at=t))

    def mark_triggered(self, plate, measurement_id, key, t):
        """触发（管理器调用）。**必须在发出受理请求之前落盘**（方案 §8.1）。

        ▲ 顺手兜住 `opened_at`：正常路径由记录器"进场/读到车牌"时 `open_vehicle` 写，
          但万一没写（例如只有管理器在跑），条目会带着 opened_at=0.0 被 TTL 扫描
          当成"1970 年开的"立刻作废 —— 触发过的车次就接力不上了，下一班还会重复触发。
        """
        def mutate(entry):
            entry.update(state="open", measurement_id=measurement_id,
                         idempotency_key=key, triggered_at=t)
            if not entry.get("opened_at"):
                entry["opened_at"] = t
        self._entry(plate, mutate)

    def mark_accepted(self, plate, workflow_id):
        self._entry(plate, lambda entry: entry.update(workflow_id=workflow_id or ""))

    def mark_progress(self, plate, stage, terminal):
        self._entry(plate, lambda entry: entry.update(
            last_stage=stage, terminal=bool(terminal)))

    def open_entry(self, plate, now):
        """未关闭且未过 TTL 的条目（过期的顺手关掉）；没有则返回 None。"""
        with self._lock:
            entries = self._read_locked()
            entry = entries.get(plate)
            if not isinstance(entry, dict) or entry.get("state") != "open":
                return None
            if now - _as_time(entry.get("opened_at")) > self.ttl:
                entry["state"] = "closed"
                entry["closed_at"] = now
                entries[plate] = entry
                self._write_locked(entries)
                return None
            return dict(entry)

    def resumable(self, now):
        """启动时用于接力监控（方案 §8.1）：还开着、未终局、未过 TTL 的条目。"""
        out = []
        with self._lock:
            entries = self._read_locked()
            changed = False
            for plate, entry in entries.items():
                if not isinstance(entry, dict) or entry.get("state") != "open":
                    continue
                if now - _as_time(entry.get("opened_at")) > self.ttl or entry.get("terminal"):
                    entry["state"] = "closed"
                    entry["closed_at"] = now
                    entries[plate] = entry
                    changed = True
                    continue
                out.append(dict(entry, plate=plate))
            if changed:
                self._write_locked(entries)
        return out

    # ── 已处理过的测量号（与车牌无关的去重，方案 §4.2 第一层） ──
    def mark_handled(self, measurement_id, t):
        """这份测量已经得到过"一次机会"（触发了，或按规则跳过了）。

        ▲ 为什么不能只靠车牌：现场车牌还是模拟的，**同一点位每 60 秒换一个随机号**
          （WebApi `Radar:LicensePlateMockHoldSeconds`）。停着不走的车每 180 秒被服务
          超时重测一次，重测那一份的车牌已经换了 —— 以车牌为键的条目认不出它是同一辆。
          服务在重测份里带 `duplicate_of`（指向上一份），这里记住处理过哪些测量号，
          `duplicate_of` 命中就是"这辆车本车次已经处理过"。
        """
        mid = str(measurement_id or "")
        if not mid:
            return
        with self._lock:
            doc = self._read_doc_locked()
            handled = doc["handled"]
            handled[mid] = t
            keep = {key: value for key, value in handled.items()
                    if t - _as_time(value) <= HANDLED_KEEP_SEC}
            if len(keep) > HANDLED_KEEP_MAX:
                keep = dict(sorted(keep.items(), key=lambda kv: _as_time(kv[1]))[-HANDLED_KEEP_MAX:])
            self._write_locked(doc["entries"], keep)

    def was_handled(self, measurement_id):
        mid = str(measurement_id or "")
        if not mid:
            return False
        with self._lock:
            return mid in self._read_doc_locked()["handled"]


# ═══════════════════════════════════════════════════════════ WebApi 客户端


# WebApi 未接受固定 Token 时，避免整个班次反复发起无效采集。
AUTH_FAIL_DISABLE_AFTER = 3
# 一份作业在后台线程里连续抛错这么多次就放弃它（否则永不收尾，排队的车一辆也发不出去）
JOB_ERROR_LIMIT = 5


def load_recorder_token(path):
    """仅从本地私有文件取令牌；错误不含文件正文或凭据值。"""
    try:
        with open(path, encoding="utf-8-sig") as stream:
            raw = stream.read(4097)
        if len(raw) > 4096:
            raise ValueError
        config = json.loads(raw)
        token = config.get("recorderToken") if isinstance(config, dict) else None
        if not isinstance(token, str) or re.fullmatch(r"[A-Za-z0-9_-]{32,256}", token) is None:
            raise ValueError
        return token
    except (OSError, ValueError, TypeError):
        raise ValueError("CH4 采集令牌配置缺失或无效，请检查私有配置的 recorderToken") from None


class _NoRecorderRedirect(urllib.request.HTTPRedirectHandler):
    """凭据只交给配置的接口，不能随重定向泄露到其它地址。"""

    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


class WebApiClient:
    """受理、查进度、读状态、取消；使用私有配置令牌，绕开系统代理。

    ▲ 本机配了代理：访问 127.0.0.1 会被代理返回 502（`CLAUDE.md`）。
      与 `tools/field_patrol.py` 一致，用空 `ProxyHandler` 的 opener。
    ▲ 不登录后台账号；缺失令牌时禁止创建采集客户端，不使用历史默认值。
    """

    def __init__(self, base_url, cfg, opener=None):
        self.base_url = str(base_url).rstrip("/")
        self.cfg = cfg
        self._recorder_token = load_recorder_token(cfg.token_file)
        self.last_size = 0
        self._opener = opener or urllib.request.build_opener(
            urllib.request.ProxyHandler({}), _NoRecorderRedirect())

    # ── 底层 ──
    def _send(self, method, path, body=None, headers=None, timeout=10.0):
        """返回 `(HTTP 状态码, 响应体)`；连接失败 / 超时返回 `(0, None)`。"""
        payload = json.dumps(body).encode("utf-8") if body is not None else None
        request = urllib.request.Request(self.base_url + path, data=payload, method=method)
        request.add_header("Accept", "application/json")
        if payload is not None:
            request.add_header("Content-Type", "application/json")
        for name, value in (headers or {}).items():
            request.add_header(name, value)
        try:
            with self._opener.open(request, timeout=timeout) as response:
                raw = response.read()
                self.last_size = len(raw)
                return int(response.status), self._parse(raw)
        except urllib.error.HTTPError as error:
            try:
                raw = error.read()
            except Exception:                               # noqa: BLE001
                raw = b""
            self.last_size = len(raw)
            return int(error.code), self._parse(raw)
        except Exception:                                   # noqa: BLE001
            self.last_size = 0
            return 0, None

    @staticmethod
    def _parse(raw):
        if not raw:
            return {}
        try:
            return json.loads(raw.decode("utf-8"))
        except Exception:                                   # noqa: BLE001
            return {"message": raw.decode("utf-8", errors="replace")[:200]}

    def _request(self, method, path, body=None, headers=None, timeout=10.0):
        merged = dict(headers or {})
        merged["X-Radar-Live-Token"] = self._recorder_token
        return self._send(method, path, body=body, headers=merged, timeout=timeout)

    # ── 四个接口 ──
    def post_capture(self, plate, key, note):
        body = {"licensePlate": plate, "idempotencyKey": key,
                "confirmOverwriteEmptyProfile": True, "note": note}
        return self._request("POST", "/api/v1/radar-live/captures", body=body,
                             headers={"Idempotency-Key": key},
                             timeout=self.cfg.capture_timeout)

    def get_progress(self, workflow_id):
        path = "/api/v1/radar-live/captures/" + urllib.parse.quote(str(workflow_id), safe="")
        return self._request("GET", path, timeout=self.cfg.progress_timeout)

    def get_status(self):
        return self._request("GET", "/api/v1/radar-live/status", timeout=self.cfg.status_timeout)

    def cancel(self, workflow_id):
        path = "/api/v1/radar-live/captures/%s/cancel" % urllib.parse.quote(str(workflow_id), safe="")
        return self._request("POST", path, timeout=self.cfg.cancel_timeout)


# ═══════════════════════════════════════════════════════════ 一辆车的一次作业


VEHICLE_TIME_KEYS = ("entered", "stable", "measured", "moving", "left")


def vehicle_times(times, left_at=None):
    """把车辆事件的时刻整成终局流水里那五个键（方案 §8.2）。

    ▲ **键必须齐全**（没有的写 `None`，不是省略）：现场拿 jsonl 做统计时，
      "这一列在不在"和"这一列是 null"是两件事，缺键会让 `pandas` 之类直接报错。
    """
    times = times or {}
    out = {key: times.get(key) for key in VEHICLE_TIME_KEYS}
    if left_at is not None:
        out["left"] = left_at
    return out


def _new_job(plate, measurement_id, key, now):
    return {
        "plate": plate, "mid": measurement_id, "key": key,
        "status": "posting", "workflow_id": "", "accepted": False,
        "posted_attempts": 0, "busy_retries": 0,
        "next_at": now, "accepted_at": None, "started_at": now,
        "poll_fail": 0, "unknown_since": None, "unknown_resumed": False,
        "awaiting_db_since": None, "awaiting_db_warned": False,
        "big_payload_warned": False, "stages_seen": {}, "last_stage": None,
        "last_progress": None, "left": False, "cancel_deadline": None,
        "moving_warned_at": None, "last_alarm_at": 0.0,
        "terminal": False, "outcome": None, "guards": {}, "alerts": [],
        "finished": False,
        # ↓ 方案 §8.2 的 vehicle_times 与 accept_status
        "times": {}, "accept_status": None,
        # ↓ 属于第几辆车（`CaptureSession.visit`）。上一辆车的作业可能在它离场后还要盯
        #   几十秒到分析终局，这期间下一辆车的 移动/离场 事件**不许**算到它头上。
        "visit": 0,
        "cancel_requested": False,      # 离场时发过取消：终局 capture_failed 按"已取消"记
        "unreachable_since": None,      # WebApi 连不上的起始时刻（按 busy 一样在车在场时重试）
        "errors": 0,                    # 后台线程处理这份作业连续出错的次数
    }


class CaptureSession:
    """一辆车的一次采集作业 —— **纯逻辑，同步可测**（时钟 / 客户端 / 响铃都可注入）。

    线程由 `CaptureManager` 驱动；测试直接调 `measured()` / `moving()` / `tick()`，
    不真 sleep、不起线程（方案 §10）。
    """

    def __init__(self, client, ledger, say, cfg, clock=time.monotonic, wall=time.time,
                 stamp=None, ring=None):
        self.client = client
        self.ledger = ledger
        self.say = say
        self.cfg = cfg
        self.clock = clock
        self.wall = wall
        self.stamp = stamp or (lambda: time.strftime("%H:%M:%S"))
        self.ring = ring or (lambda: print("\a", end="", flush=True))
        self.job = None
        self.pending = None             # (t, 测量号, 车牌原文, 通道快照, 车次)
        self.disabled = ""              # 非空 = 本班次停用采集（423 / 连续 Token 被拒之后）
        self.silent_skip = set()        # 同一测量号只回执一次"跳过"
        # 当前这辆车的事件时刻（方案 §7.1 / §8.2）。进场时重置，测完时快照进作业，
        # 之后 moving/stopped/left 两边一起更新 —— 作业跨车次时不会串。
        self.times = {}
        # 车次号：每次进场 +1。作业与排队项都记下自己属于哪一车次，
        # 移动 / 重新停住 / 离场事件只作用于**当前车次**（见 `_new_job` 的 visit）。
        self.visit = 0
        self.visit_outcome = (0, None)  # (车次, 本车次双雷达的结论)；供"车在动"那一行判断
        self.auth_failures = 0          # 连续固定 Token 被拒次数

    # ── 输出 ──
    def _line(self, text):
        self.say("%s %-4s %s" % (self.stamp(), self.cfg.channel, text))

    def _block(self, text):
        """结果块的续行：不带时间前缀，与方案 §6.1 的排版一致。"""
        self.say(" " * 15 + text)

    def _bell(self, text):
        self._line(text)
        if self.cfg.bell:
            try:
                self.ring()
            except Exception:                                # noqa: BLE001
                pass

    # ═════════════════════════════ 事件（主线程调用）

    def _current(self, job):
        """这份作业还在跑、且属于**当前车次**（上一辆车离场后还在盯分析的作业不算）。"""
        return job is not None and not job["finished"] and job.get("visit", 0) == self.visit

    def _was_handled(self, measurement_id):
        try:
            return self.ledger.was_handled(measurement_id)
        except Exception:                                    # noqa: BLE001
            return False

    def _mark_handled(self, measurement_id, t):
        try:
            self.ledger.mark_handled(measurement_id, t)
        except Exception as error:                           # noqa: BLE001
            self._line("▲ 台账写不进去（已处理测量号）：%s: %s" % (type(error).__name__, error))

    def entered(self, t):
        """车进场（方案 §7.1）。▲ 不触发采集 —— 只是开一个新车次、把本车的事件时刻归零重记。"""
        self.visit += 1
        self.times = {"entered": t}
        self._line("-> 双雷达采集：本车已入场，等第一份测量出来再触发")

    def stable(self, t):
        """判到【已停稳】（方案 §7.1）。只为 §8.2 的 `vehicle_times.stable` 留时刻。"""
        self.times["stable"] = t

    def measured(self, t, measurement_id, plate_raw, channel_state=None):
        """本车的第一份测量出来了（方案 §4.1 的触发事件）。"""
        self.times["measured"] = t
        mid = str(measurement_id or "")
        if not mid:
            return
        if self.disabled:
            self._line("◎ 双雷达采集已停用（%s），本次不触发  测量 %s" % (self.disabled, short_id(mid)))
            return
        # §4.2 第一层：服务把这份标成 `duplicate_of`（超时强开会话、期间车道从没空过 = 同一辆车），
        # 且它指向的那一份已经处理过 -> 本车次已经有过一次机会，不再触发。
        # ★ 这一层**与车牌无关**：模拟车牌每 60 秒换一个，记录器被看门狗重启后，
        #   停着不走的空车在下一次超时重测时车牌已经变了，只靠车牌台账会再采一遍。
        source = duplicate_source(channel_state, mid)
        if source and self._was_handled(source):
            self._mark_handled(mid, t)          # 链接下去：下一次重测指向的就是这一份
            if mid not in self.silent_skip:
                self.silent_skip.add(mid)
                self._line("▲ 同一辆车停着没走、被超时重测（接测量 %s），本车次已处理过，不重复采集  测量 %s"
                           % (short_id(source), short_id(mid)))
            return
        plate = normalize_plate(plate_raw)
        # 同一辆车的重复测量：停着不走的车每 180 秒被服务超时重测一次，会产生新测量号。
        # 本车次已经触发过了，不再触发（方案 D2 / §4.1）。
        if (plate and self.job is not None and not self.job["finished"]
                and plate == self.job.get("plate")):
            self._mark_handled(mid, t)
            if mid not in self.silent_skip:
                self.silent_skip.add(mid)
                self._line("▲ 同一辆车的重复测量，本车次已触发过，不重复采集  测量 %s" % short_id(mid))
            return
        if self.job is not None and not self.job["finished"]:
            if self.pending is not None:
                # 排队位只有一个：再来一辆说明排着的那辆早就走了（或者车次没接上），别让它冒名顶替
                old_t, old_mid, old_plate, _old_state, old_visit = self.pending
                self._line("▲ 排队中的上一辆（测量 %s）还没开始采集就来了新车，放弃它" % short_id(old_mid))
                self._record_skip(old_t, normalize_plate(old_plate) or "", old_mid,
                                  "not_accepted:排队时被新车替换", visit=old_visit)
            self.pending = (t, mid, plate_raw, channel_state, self.visit)
            self._line("-> 上一辆车的采集还没收尾，本车等它结束再发  测量 %s" % short_id(mid))
            return
        self._begin(t, mid, plate_raw, channel_state, self.visit)

    def moving(self, t):
        """车开始移动（方案 §7.1 / D6）。这是唯一还来得及拦车的时刻。"""
        self.times["moving"] = t
        job = self.job
        if not self._current(job):
            if self.pending is not None and self.pending[4] == self.visit:
                self._line("▲ CH4 车在动，本车的双雷达采集还在排队（等上一辆收尾），尚未开始")
                return
            visit, outcome = self.visit_outcome
            if visit == self.visit and outcome:
                # 本车次已有结论（完成 / 跳过 / 失败各自都单独打印过）。
                # ▲ 原来这里一律打"本车未发起双雷达采集" —— 采集成功后车倒出去时也打，误导现场。
                return
            self._line("▲ CH4 车在动，本车未发起双雷达采集")
            return
        job["times"]["moving"] = t
        stage = (job.get("last_progress") or {}).get("stage")
        if not job["accepted"] or stage in (None, "unknown", "waiting"):
            job["moving_warned_at"] = t
            job["last_alarm_at"] = t
            job["alerts"].append({"t": t, "text": "车在动，双雷达还没采到完整场"})
            self._bell("▲▲ CH4 车在动，双雷达还没采到完整场，请让司机停车！")
        elif stage == "captured":
            self._line("▲ 车在动，点云已捕获，等待落盘")

    def stopped(self, t):
        """车重新停住 —— 停掉重复报警（方案 D6）。"""
        self.times["stopped"] = t
        job = self.job
        if not self._current(job):
            return
        job["times"]["stopped"] = t
        if job.get("moving_warned_at"):
            job["moving_warned_at"] = None
            self._line("-> 车重新停住，停止报警")

    def left(self, t):
        """车辆离场（方案 §7.2）。"""
        self.times["left"] = t
        if self.pending is not None and self.pending[4] == self.visit:
            # 本车还在排队（等上一辆的作业收尾）就走了：它的采集永远不该再发出去。
            # ▲ 原来排队项不认车次，上一辆收尾后照样给这辆**已经离场**的车发受理。
            pend_t, pend_mid, pend_plate, _state, pend_visit = self.pending
            self.pending = None
            self._line("▲ 车辆离场，双雷达未受理（排队等上一辆收尾时就走了）  测量 %s" % short_id(pend_mid))
            self._mark_handled(pend_mid, t)
            self._record_skip(pend_t, normalize_plate(pend_plate) or "", pend_mid,
                              "not_accepted:离场前未受理", visit=pend_visit, left_at=t)
        job = self.job
        if not self._current(job):
            return
        job["left"] = True
        job["times"]["left"] = t
        action, stage = leave_action(job.get("last_progress"), job["accepted"])
        if action == "never":
            self._line("▲ 车辆离场，双雷达未受理（离场前未受理）")
            self._finish(t, "not_accepted:离场前未受理")
            return
        if action == "done":
            return
        if action == "cancel":
            self._cancel(t, job, stage)
            return
        if stage == "captured":
            self._line("▲ 车已离开，点云已捕获，继续监控落盘")
        else:
            self._line("▲ 车已离开，继续监控落盘")

    # ═════════════════════════════ 时钟推进（管理器调用）

    def tick(self, now=None):
        now = self.wall() if now is None else now
        job = self.job
        if job is not None and not job["finished"]:
            try:
                self._repeat_alarm(now, job)
                if not job["finished"] and now >= job.get("next_at", 0):
                    self._step(now, job)
                job["errors"] = 0
            except Exception as error:                       # noqa: BLE001
                # ★ 一份作业反复抛错就放弃它 —— 否则它永远不收尾，后面排队的车一辆也发不出去
                job["errors"] = job.get("errors", 0) + 1
                job["next_at"] = now + self.cfg.poll_sec
                self._line("▲ 双雷达采集处理出错（第 %d 次）：%s: %s"
                           % (job["errors"], type(error).__name__, error))
                if job["errors"] >= JOB_ERROR_LIMIT and not job["finished"]:
                    self._line("× 连续出错 %d 次，放弃这份作业。流程编号 %s 请到后台核查"
                               % (job["errors"], job.get("workflow_id") or "-"))
                    try:
                        self._finish(now, "error:%s" % type(error).__name__)
                    except Exception:                        # noqa: BLE001
                        job["finished"] = True
        if self.job is not None and self.job["finished"]:
            self.job = None
        if self.job is None and self.pending is not None:
            t, mid, plate_raw, state, visit = self.pending
            self.pending = None
            self._begin(t, mid, plate_raw, state, visit)

    def idle(self):
        return self.job is None and self.pending is None

    def resume(self, now=None):
        """启动时接力监控上一班次没结束的流程（方案 §8.1）。"""
        now = self.wall() if now is None else now
        waiting = [entry for entry in self.ledger.resumable(now)
                   if entry.get("workflow_id")]
        if not waiting:
            return False
        entry = waiting[0]
        if len(waiting) > 1:
            self._line("▲ 台账里有 %d 个未结束的流程，先接力第一个（%s），其余等 TTL 解封"
                       % (len(waiting), entry["workflow_id"]))
        self._line("-> 接着监控上次未结束的流程 %s  车牌 %s"
                   % (entry["workflow_id"], entry.get("plate") or "-"))
        self._adopt(now, entry.get("plate") or "", entry)
        return True

    # ═════════════════════════════ 触发

    def _begin(self, t, mid, plate_raw, channel_state, visit=None):
        visit = self.visit if visit is None else visit
        plate = normalize_plate(plate_raw)
        if not plate:
            self._line("CH4 未取到有效车牌（%s），未触发双雷达采集  测量 %s"
                       % (str(plate_raw or "空")[:16], short_id(mid)))
            self.visit_outcome = (visit, "no_plate")
            return

        key = idempotency_key(mid)
        if not key_is_valid(key):
            self._line("× 测量号不能做成合法幂等键（%s），未触发" % short_id(mid, 16))
            self.visit_outcome = (visit, "bad_key")
            return

        # 从这里起，这份测量都算"得到过一次机会"（触发、接力或按规则跳过）——
        # 之后带 duplicate_of 指向它的重测份不再触发（§4.2 第一层）。
        self._mark_handled(mid, t)

        # §4.2 去重：台账里同牌且未关闭的条目
        entry = self.ledger.open_entry(plate, t)
        if entry:
            if entry.get("workflow_id") and not entry.get("terminal"):
                self._line("-> 该车牌本车次已触发过，改为接力监控流程 %s" % entry["workflow_id"])
                self._adopt(t, plate, entry, visit)
                return
            if entry.get("measurement_id") == mid:
                # 同一个测量号、不同进程 = 上次"触发已落盘但请求没发出去就被杀"。
                # **必须用同一个幂等键**重新受理（换键 = 重复采集，方案 §5 / §8.1）。
                key = entry.get("idempotency_key") or key
                self._line("-> 上次触发后进程被中断，用同一个幂等键重新受理  测量 %s" % short_id(mid))
            elif entry.get("measurement_id"):
                # 本车次已触发过（停着不走被 180 秒超时重测，或记录器重启后重新判到）
                if mid not in self.silent_skip:
                    self.silent_skip.add(mid)
                    self._line("▲ 该车牌本车次已触发过（台账在册），本次不重复采集  测量 %s"
                               % short_id(mid))
                    self._record_skip(t, plate, mid, "duplicate_skipped", visit=visit)
                return
            # 测量号为空 = 本车自己的"进场"记录：允许触发

        # 门 1：双雷达侧健康检查（只读）
        try:
            status, body = self.client.get_status()
        except Exception as error:                           # noqa: BLE001
            self._skip(t, plate, mid, "双雷达状态不可用（%s）" % error,
                       "skipped_dual_unavailable", {"status": "error"}, visit=visit)
            return
        if status == 401:
            self._auth_rejected(t, plate, mid, "状态接口 401", visit)
            return
        if status == 403:
            self._line("× WebApi 拒绝 CH4 状态请求  测量 %s" % short_id(mid))
            self._record_skip(t, plate, mid, "not_accepted:采集请求被拒绝", visit=visit)
            return
        if status == 200:
            self.auth_failures = 0
        health, why = (("unavailable", "HTTP %s" % status) if status != 200
                       else _dual_status_detail(body))
        if health == "unavailable":
            self._skip(t, plate, mid, "双雷达状态不可用" + ("：" + why if why else ""),
                       "skipped_dual_unavailable", {"status": status}, visit=visit)
            return

        # 门 2：载货链负载闸门
        reason = load_guard(channel_state, self.cfg)
        if reason:
            guards = self._guard_signals(channel_state)
            self._skip(t, plate, mid, "载货链负载：" + reason,
                       "skipped_load_guard", guards, visit=visit)
            return

        # 触发先落盘，再发请求：请求途中被杀，重启也不会重复触发
        self.ledger.mark_triggered(plate, mid, key, t)
        job = _new_job(plate, mid, key, self.wall())
        job["visit"] = visit
        job["guards"] = dict(self._guard_signals(channel_state), dual_health=health)
        # 本车的事件时刻快照进作业（方案 §8.2）：之后再进场的车不会串改这一份
        job["times"] = dict(self.times, measured=t)
        self.job = job
        if health == "busy":
            # ▲ 状态接口说"忙" = Worker 正在**别的**会话里（本工具同一时刻只跑一份作业）。
            #   原来这里的提示写成"等本工具自己的作业"，把原因说反了。
            job["busy_note"] = why or "双雷达正被别的流程占用"
        self._post(self.wall(), job)

    def _auth_rejected(self, t, plate, mid, error, visit):
        """WebApi 拒绝 CH4 请求；只放弃这一辆，连续几次才停用。"""
        self.auth_failures += 1
        self._line("× WebApi 未接受 CH4 采集标记，请确认服务版本（%s）  测量 %s"
                   % (error, short_id(mid)))
        self._record_skip(t, plate, mid, "not_accepted:认证失败", visit=visit)
        self._maybe_disable_after_auth_failures()

    def _maybe_disable_after_auth_failures(self):
        if self.auth_failures >= AUTH_FAIL_DISABLE_AFTER and not self.disabled:
            self.disabled = "连续 %d 次认证失败" % self.auth_failures
            self._line("▲ 连续 %d 次 CH4 请求被拒，本班次停止双雷达采集。"
                       "确认 WebApi 已更新后重启停稳记录器" % self.auth_failures)

    def _guard_signals(self, channel_state):
        state = channel_state or {}
        return {"fps": _as_number(state.get("fps")),
                "recent_fps": _as_number(state.get("recent_fps")),
                "incomplete_ratio": _as_number(state.get("incomplete_ratio")),
                "packet_rate": _as_number((state.get("source") or {}).get("packet_rate"))}

    def _adopt(self, t, plate, entry, visit=None):
        """接力监控上一班次没结束的流程（方案 §8.1）。"""
        self.ledger.mark_progress(plate, entry.get("last_stage"), False)
        job = _new_job(plate, entry.get("measurement_id") or "", entry.get("idempotency_key") or "",
                       self.wall())
        job.update(status="polling", workflow_id=entry.get("workflow_id") or "",
                   accepted=True, accepted_at=self.wall())
        job["rescued"] = True
        job["times"] = dict(self.times)         # 车辆事件已对不上，留空就写 None（§8.1）
        # 启动时接力（visit=None）：记录器刚起、马上会把在场的车记成新一次进场，
        # 所以这份作业**不认**之后的移动 / 离场事件（§8.1："只盯阶段，不做离场判定"）。
        job["visit"] = -1 if visit is None else visit
        self.job = job
        self._poll(self.wall(), job)

    # ═════════════════════════════ 受理

    def _post(self, now, job):
        job["status"] = "posting"
        job["posted_attempts"] += 1
        note = "载货雷达%s停稳自动触发 测量 %s" % (self.cfg.channel, job["mid"])
        try:
            status, body = self.client.post_capture(job["plate"], job["key"], note)
        except Exception as error:                           # noqa: BLE001
            status, body = 0, {"message": str(error)}
        job["unreachable_since"] = None
        if status not in (0, 401, 403, 423):
            self.auth_failures = 0

        decision = classify_accept(status, body)
        job["http_status"] = status
        job["accept_message"] = decision.message
        job["duplicate"] = decision.kind == "duplicate"
        # 响应体里的 status（accepted / already_accepted / processing…），方案 §8.2 要求入库
        data = body.get("data") if isinstance(body, dict) and isinstance(body.get("data"), dict) else {}
        if data.get("status"):
            job["accept_status"] = str(data["status"])

        if decision.kind in ("accepted", "duplicate", "processing"):
            workflow_id = decision.workflow_id
            if not workflow_id:
                self._finish(now, "not_accepted:受理成功但没拿到流程号")
                self._line("× 受理返回里没有流程编号，无法轮询（%s）" % decision.message)
                return
            job.update(status="polling", workflow_id=workflow_id, accepted=True,
                       accepted_at=now, next_at=now)
            self.ledger.mark_accepted(job["plate"], workflow_id)
            head = ("-> 该请求已在进行中（未重复采集）" if decision.kind == "duplicate"
                    else "-> 正在派发，按流程号轮询" if decision.kind == "processing"
                    else "-> 双雷达采集已受理")
            self._line("%s  车牌 %s  流程 %s  测量 %s"
                       % (head, job["plate"], workflow_id, short_id(job["mid"])))
            return

        if decision.kind == "busy":
            job["busy_retries"] += 1
            job["status"] = "posting"
            job["next_at"] = now + self.cfg.busy_retry_sec
            self._line("◎ 双雷达正忙，重试中（第 %d 次）%s  测量 %s"
                       % (job["busy_retries"],
                          ("（%s）" % job["busy_note"]) if job.get("busy_note") else "",
                          short_id(job["mid"])))
            if job["left"]:
                self._line("▲ 车辆已离场，放弃受理重试")
                self._finish(now, "not_accepted:双雷达正忙")
            elif now - job["started_at"] > self.cfg.total_timeout:
                self._finish(now, "not_accepted:双雷达正忙")
            return

        if decision.kind == "uncertain":
            if job["posted_attempts"] <= self.cfg.unsure_retries:
                job["status"] = "posting"
                job["next_at"] = now + self.cfg.unsure_interval
                self._line("▲ 结果不确定（HTTP %s），同键重试（第 %d 次）  测量 %s"
                           % (status, job["posted_attempts"], short_id(job["mid"])))
                return
            self._line("▲ 结果不确定：保留幂等键，勿换键重发（HTTP %s，已重试 %d 次）"
                       % (status, job["posted_attempts"] - 1))
            self._finish(now, "not_accepted:结果不确定")
            return

        if decision.kind == "manual":
            self._line("▲ 需人工确认：%s（流程 %s）请到后台页面（雷达实况）人工核实后处理"
                       % (decision.message or "无法核实上次派发结果",
                          decision.workflow_id or "-"))
            self._finish(now, "manual_confirmation")
            return

        if decision.kind == "locked":
            self._disable(now, job, decision.text or "采集服务拒绝受理")
            return

        # rejected：一律记成方案 §8.2 的 `not_accepted:<原因>`，原文放进 accept_message
        self._line("× %s  测量 %s" % (decision.text or decision.message or "受理被拒绝",
                                    short_id(job["mid"])))
        self._finish(now, "not_accepted:" + _reject_reason(status, decision.message))

    def _disable(self, now, job, text):
        self.disabled = text
        self._line("▲ %s" % text)
        self._finish(now, "not_accepted:" + text)

    # ═════════════════════════════ 进度

    def _step(self, now, job):
        if job["status"] in ("posting",):
            self._post(now, job)
            return
        if job["status"] == "polling":
            self._poll(now, job)
            return

    def _poll(self, now, job):
        job["next_at"] = now + self.cfg.poll_sec
        try:
            status, body = self.client.get_progress(job["workflow_id"])
        except Exception:                                    # noqa: BLE001
            status, body = 0, None

        if self.client.last_size > self.cfg.response_warn_bytes and not job["big_payload_warned"]:
            job["big_payload_warned"] = True
            self._line("▲ 进度响应体 %.0f KB（超过 %.0f KB），注意别让它被反复拉取"
                       % (self.client.last_size / 1024.0, self.cfg.response_warn_bytes / 1024.0))

        progress = (body.get("data") if isinstance(body, dict) and isinstance(body.get("data"), dict)
                    else {})
        if status != 200 or not progress:
            job["poll_fail"] += 1
            if job["poll_fail"] % POLL_FAIL_WARN_EVERY == 1:
                self._line("▲ 进度查询连续失败 %d 次（HTTP %s），继续重试  流程 %s"
                           % (job["poll_fail"], status, job["workflow_id"]))
            # ★ 查不到进度时也要看总时限 / 取消时限。原来这里直接 return：
            #   WebApi 一挂，这份作业就**永远**轮询下去，后面排队的车一辆也发不出去。
            self._check_deadlines(now, job)
            return
        job["poll_fail"] = 0
        job["last_progress"] = progress
        stage = str(progress.get("stage") or "")
        self._note_stage(now, job, stage, progress)

        # 自检一：unknown 持续 -> 用同一个幂等键重投一次
        if not progress.get("terminal") and stage == "unknown":
            if job["unknown_since"] is None:
                job["unknown_since"] = now
            elif not job["unknown_resumed"] and now - job["unknown_since"] >= self.cfg.resume_unknown_sec:
                job["unknown_resumed"] = True
                self._line("▲ 本次流程长时间无进展（unknown %.0f 秒），用同一个幂等键重新提交一次"
                           % self.cfg.resume_unknown_sec)
                job["status"] = "posting"
                job["next_at"] = now
                self._post(now, job)                 # 立刻重投，不等下一个采样点
                return
        elif stage != "unknown":
            job["unknown_since"] = None

        # 自检二：分析已完成但数据库记录迟迟不出现
        if stage == "analyzing" and "确认数据库记录" in str(progress.get("message") or ""):
            if job["awaiting_db_since"] is None:
                job["awaiting_db_since"] = now
            elif not job["awaiting_db_warned"] and now - job["awaiting_db_since"] >= self.cfg.awaiting_db_warn_sec:
                job["awaiting_db_warned"] = True
                self._line("▲ 分析已完成，但数据库记录超过 %.0f 秒仍未出现。请保留流程编号 %s，"
                           "联系管理员核查 Analyzer 入库状态；不要重新发起采集"
                           % (self.cfg.awaiting_db_warn_sec, job["workflow_id"]))
        else:
            job["awaiting_db_since"] = None

        if progress.get("terminal"):
            self._finish_terminal(now, job, progress)
            return
        self._check_deadlines(now, job)

    def _check_deadlines(self, now, job):
        """取消后的收尾时限、受理后的总时限。到了就收尾并返回 True。"""
        if job["finished"] or job["status"] != "polling":
            return job["finished"]
        # 取消后仍没终局（方案 §7.2：取消后最多再轮询 30 秒拿终局）
        if job["cancel_deadline"] is not None and now >= job["cancel_deadline"]:
            self.ledger.mark_progress(job["plate"], "cancelled", True)
            self._finish(now, "cancelled_left_before_capture")
            return True
        # 总超时（方案 §6）
        if job["accepted_at"] is not None and now - job["accepted_at"] > self.cfg.total_timeout:
            self._line("▲ 双雷达采集超过 %.0f 秒仍未终局，停止轮询。流程编号 %s 请到后台核查"
                       % (self.cfg.total_timeout, job["workflow_id"]))
            self.ledger.mark_progress(job["plate"], job["last_stage"], False)
            self._finish(now, "timeout")
            return True
        return False

    def _note_stage(self, now, job, stage, progress):
        """只在阶段变化时打印步骤条，并补"车可以离开"。"""
        if not stage or stage == job["last_stage"]:
            return
        job["last_stage"] = stage
        job["stages_seen"].setdefault(stage, now)
        if job.get("cancel_requested") and stage == "capture_failed":
            # 离场取消后的终局：WebApi 把"采集已取消"也映射成 capture_failed。
            # 这一行交给 `_finish_terminal` 按"已取消"打，这里不画一条"×采集失败"去吓人。
            return
        self.ledger.mark_progress(job["plate"], stage, bool(progress.get("terminal")))
        line = render_steps(progress)
        if stage == "saved":
            # ★ 现场据此放车，这一行必须带这句。
            line += "   √ 双雷达数据已落盘，车可以离开"
        self._line(line)

    # ═════════════════════════════ 终局

    def _finish_terminal(self, now, job, progress):
        stage = str(progress.get("stage") or "")
        message = str(progress.get("message") or "")
        error = str(progress.get("error") or "")
        job["terminal"] = True

        if job.get("cancel_requested") and stage == "capture_failed":
            # ★ 离场取消后，WebApi 把 Worker 的 cancelled 也映射成 capture_failed（"采集已取消"，
            #   见 `CaptureProgressBuilder`）。这是方案 §7.2 预期的收尾，按"已取消"记，
            #   不按"采集失败"记 —— 原来这里会记成 `failed:capture_failed`，
            #   方案 §8.2 的 `cancelled_left_before_capture` 在真实环境里永远出不来。
            self.ledger.mark_progress(job["plate"], "cancelled", True)
            self._line("√ 双雷达：本次采集已取消（车辆离场前未采到完整场）  流程 %s%s"
                       % (job["workflow_id"], ("：" + message) if message else ""))
            self._finish(now, "cancelled_left_before_capture")
            return
        self.ledger.mark_progress(job["plate"], stage, True)

        if stage == "completed":
            dimensions = progress.get("dimensions") or {}
            pending = dimensions.get("reviewStatus") == "pending"
            tail = ""
            if pending:
                tail = "  ◎已入库，待人工审核（记录 %s）" % (dimensions.get("recordId") or "-")
            self._line("√ 双雷达：计算完成并写入数据库%s" % tail)
        elif stage == "pending_review":
            self._line("▲ 双雷达：计算值已返回，待人工审核（未入库）：%s" % message)
        elif stage == "completed_without_dimensions":
            self._line("▲ 双雷达：分析完成，本次不产出尺寸：%s" % message)
        elif stage == "persistence_failed":
            # 数据丢了，且车多半已经走了 —— 必须响铃让人知道（方案 §7.2）
            self._bell("×× 落盘失败，本车双雷达数据丢失，需要让车回来重新采")
            if message:
                self._block(message)
        elif stage in FAILURE_STAGES:
            self._line("× 双雷达：%s：%s%s"
                       % (FAILURE_STAGES[stage], message, ("（%s）" % error) if error else ""))
        else:
            self._line("▲ 双雷达：终局 %s：%s" % (stage or "-", message))

        fields = collect_result_fields(progress)
        if fields:
            if fields.get("pending"):
                self._block("模型/计算值，尚未经人工核验")
            self._block(result_line(fields))
            note = length_measurement_note(progress)
            if note:
                self._block(note)

        outcome = {
            "completed": "completed",
            "pending_review": "pending_review",
            "completed_without_dimensions": "no_dimensions",
        }.get(stage) or ("failed:%s" % stage)
        if stage == "completed" and (progress.get("dimensions") or {}).get("reviewStatus") == "pending":
            outcome = "completed_pending_review"
        if job["left"]:
            if stage in ("completed", "pending_review", "completed_without_dimensions"):
                outcome = "saved_after_leave"
            elif stage == "persistence_failed":
                outcome = "persistence_failed_after_leave"
        self._finish(now, outcome)

    def _cancel(self, now, job, stage):
        self._line("-> 车辆离场且未采到完整场（%s），请求取消本次采集  流程 %s"
                   % (stage or "unknown", job["workflow_id"]))
        job["cancel_requested"] = True
        try:
            status, body = self.client.cancel(job["workflow_id"])
        except Exception as error:                           # noqa: BLE001
            status, body = 0, {"message": str(error)}
        if status == 200:
            self._line("√ 已请求取消  流程 %s" % job["workflow_id"])
        elif status == 409:
            # ▲ 409 = Worker 当前会话已不属于本流程 —— 多半是在上一次查进度之后
            #   这次采集**已经收完**（进度落后 2 秒）。不能就此记成"已取消"：
            #   方案 §7.2 要求 200/409 都"再最多轮询 30 秒拿终局"，以终局为准。
            self._line("▲ 取消未生效（409 会话已不属于本流程，可能刚好已采完），继续查终局  流程 %s"
                       % job["workflow_id"])
        else:
            self._line("▲ 取消失败（HTTP %s %s），继续查终局  流程 %s"
                       % (status, (body or {}).get("message") or "", job["workflow_id"]))
        # 再最多轮询一段时间拿终局
        job["cancel_deadline"] = now + self.cfg.leave_cancel_grace
        job["next_at"] = now + self.cfg.poll_sec

    # ═════════════════════════════ 跳过 / 收尾

    def _skip(self, t, plate, mid, reason, outcome, guards, visit=None):
        self._line("-> 双雷达采集已跳过（%s）  测量 %s" % (reason, short_id(mid)))
        self._record_skip(t, plate, mid, outcome, guards, visit=visit)

    def _record_skip(self, t, plate, mid, outcome, guards=None, visit=None, left_at=None):
        self.visit_outcome = (self.visit if visit is None else visit, outcome)
        self._write_record({
            "channel": self.cfg.channel, "plate": plate, "measurement_id": mid,
            "idempotency_key": idempotency_key(mid),
            "outcome": outcome, "guards": guards or {},
            "vehicle_times": vehicle_times(dict(self.times, measured=t), left_at),
            "accept_http": None, "accept_status": None, "accept_message": "",
            "duplicate": False, "workflow_id": "", "stage_times": {},
            "captured_cameras": [], "persisted_cameras": [], "persistence_state": None,
            "safety_block_reason": "", "error": "", "alerts": [],
            "dimensions": None, "left_before_terminal": left_at is not None,
        })

    def _finish(self, now, outcome):
        job = self.job
        job["outcome"] = outcome
        job["finished"] = True
        job["finished_at"] = now
        self.visit_outcome = (job.get("visit", 0), outcome)
        job["terminal"] = job.get("terminal") or outcome in (
            "completed", "completed_pending_review", "pending_review", "no_dimensions",
            "saved_after_leave")
        # 离场后终局、或接力监控跑到终局 —— 这两种情况下这辆车都不会再来了，关掉台账条目。
        # ▲ 超时（timeout）**不能**关：方案 §6 要求保留流程编号，条目也给 TTL 兜底。
        if job.get("rescued") or (job["left"] and job["terminal"]):
            try:
                self.ledger.close_vehicle(job["plate"], now)
            except Exception:                                # noqa: BLE001
                pass
        self._write_record({
            "channel": self.cfg.channel,
            "plate": job["plate"],
            "measurement_id": job["mid"],
            "idempotency_key": job["key"],
            "workflow_id": job.get("workflow_id") or "",
            "accept_http": job.get("http_status"),
            "accept_status": job.get("accept_status"),
            "accept_message": job.get("accept_message") or "",
            "duplicate": bool(job.get("duplicate")),
            "outcome": outcome,
            "vehicle_times": vehicle_times(job.get("times"), now if job["left"] else None),
            "stage_times": {k: v for k, v in job["stages_seen"].items()},
            "captured_cameras": list((job.get("last_progress") or {}).get("capturedCameras") or []),
            "persisted_cameras": list((job.get("last_progress") or {}).get("persistedCameras") or []),
            "persistence_state": (job.get("last_progress") or {}).get("persistenceState"),
            "safety_block_reason": (job.get("last_progress") or {}).get("safetyBlockReason"),
            "error": (job.get("last_progress") or {}).get("error") or "",
            "guards": job.get("guards") or {},
            "alerts": job.get("alerts") or [],
            "dimensions": collect_result_fields(job.get("last_progress")),
            "left_before_terminal": bool(job["left"]),
        })

    def _write_record(self, record):
        path = self.cfg.record_path(time.strftime("%Y%m%d"))
        folder = os.path.dirname(path)
        try:
            if folder:
                os.makedirs(folder, exist_ok=True)
            with open(path, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
        except OSError as error:
            self._line("▲ 终局流水写不进去（%s）：%s" % (path, error))

    # ═════════════════════════════ 报警重复

    def _repeat_alarm(self, now, job):
        """车还在动、还没采到完整场：每 5 秒重复一次，直到捕获 / 重新停住 / 离场（方案 D6）。"""
        if not job.get("moving_warned_at"):
            return
        stage = (job.get("last_progress") or {}).get("stage")
        if job["left"] or stage not in (None, "unknown", "waiting"):
            job["moving_warned_at"] = None
            return
        if now - job.get("last_alarm_at", 0.0) >= MOVING_REPEAT_SEC:
            job["last_alarm_at"] = now
            self._bell("▲▲ CH4 车在动，双雷达还没采到完整场，请让司机停车！")


# ═══════════════════════════════════════════════════════════ 后台线程


class CaptureManager(threading.Thread):
    """CH4 专用后台线程。主线程只调 `on_*` 入队，不等待、不做 HTTP（方案 §3）。"""

    def __init__(self, client, ledger, say, cfg, clock=time.monotonic, wall=time.time,
                 stamp=None, ring=None):
        super().__init__(name="dual-radar-capture", daemon=True)
        self.session = CaptureSession(client, ledger, say, cfg, clock=clock, wall=wall,
                                      stamp=stamp, ring=ring)
        self._queue = queue.Queue()
        # ▲ 不能叫 _stop：threading.Thread 内部有同名方法 _stop()，被属性遮蔽后
        #   join() 内部调用它会抛 TypeError: 'Event' object is not callable。
        self._halt = threading.Event()
        self._wake = threading.Event()
        self._resume_pending = False
        self._last_error = None
        self._last_error_at = 0.0

    # ── 主线程调用：只入队 ──
    def on_entered(self, t=None):
        self._queue.put(("entered", t, None))
        self._wake.set()

    def on_measured(self, t, measurement_id, plate, channel_state=None):
        self._queue.put(("measured", t, (measurement_id, plate, channel_state)))
        self._wake.set()

    def on_moving(self, t=None):
        self._queue.put(("moving", t, None))
        self._wake.set()

    def on_stable(self, t=None):
        self._queue.put(("stable", t, None))
        self._wake.set()

    def on_stopped(self, t=None):
        self._queue.put(("stopped", t, None))
        self._wake.set()

    def on_left(self, t=None):
        self._queue.put(("left", t, None))
        self._wake.set()

    def close(self, timeout=10.0):
        """Ctrl-C 退出：进行中的作业**不取消**，台账条目保留，下次启动接着盯（方案 §9.1）。"""
        self._halt.set()
        self._wake.set()
        if self.is_alive():
            self.join(timeout)

    def resume(self, now=None):
        """启动时要求接力监控上一班次没结束的流程。

        ▲ 这里**只置标志**，真正的 HTTP 由后台线程在 `run()` 里做 ——
          主线程一次网络调用都不发（方案 §3）。
        """
        self._resume_pending = True
        self._wake.set()
        return None

    # ── 线程 ──
    def run(self):
        """★ 每一步都有兜底：原来任何一处抛错（台账写不进去、坏字段、没想到的分支）
        都会让这条线程**悄无声息地死掉** —— 之后记录器照样入队，但再也没人处理，
        双雷达采集从此一次都不触发，屏幕上一个字都没有（2026-09-25 审查查实）。"""
        if self._resume_pending:
            self._resume_pending = False
            self._guarded("接力监控", self.session.resume)
        while not self._halt.is_set():
            drained = self._drain()
            if not drained:
                self._wake.wait(0.2)
                self._wake.clear()
                self._drain()
            if self._halt.is_set():
                break
            self._guarded("推进作业", self.session.tick, self.session.wall())

    def _guarded(self, what, func, *args):
        try:
            return func(*args)
        except Exception as error:                           # noqa: BLE001
            text = "%s: %s" % (type(error).__name__, error)
            now = time.monotonic()
            # 同一个错每分钟最多报一次，别刷屏
            if self._last_error != (what, text) or now - self._last_error_at >= 60.0:
                self._last_error = (what, text)
                self._last_error_at = now
                try:
                    self.session._line("▲ 双雷达采集后台出错（%s，已跳过这一步，线程继续）：%s" % (what, text))
                except Exception:                            # noqa: BLE001
                    pass
            return None

    def _drain(self):
        got = False
        while True:
            try:
                kind, t, payload = self._queue.get_nowait()
            except queue.Empty:
                return got
            got = True
            self._guarded("处理事件 " + str(kind), self._dispatch, kind, t, payload)

    def _dispatch(self, kind, t, payload):
        now = self.session.wall() if t is None else t
        if kind == "entered":
            # 台账的"进场"由主线程写；这里只开新车次、把本车的事件时刻归零（§8.2 的 vehicle_times）
            self.session.entered(now)
        elif kind == "stable":
            self.session.stable(now)
        elif kind == "measured":
            mid, plate, state = payload
            self.session.measured(now, mid, plate, state)
        elif kind == "moving":
            self.session.moving(now)
        elif kind == "stopped":
            self.session.stopped(now)
        elif kind == "left":
            self.session.left(now)


# ═══════════════════════════════════════════════════════════ 组装


def build_manager(cfg, say, lock=None, ledger=None, client=None):
    """按配置把台账、客户端、后台线程装起来。停稳记录器用这个入口。"""
    lock = lock or threading.RLock()        # 与 say() 共用时必须可重入（CLAUDE.md）
    if ledger is None:
        ledger = VehicleLedger(cfg.ledger_path(), lock, ttl=cfg.ledger_ttl)
    if client is None:
        client = WebApiClient(cfg.base_url, cfg)
    return CaptureManager(client, ledger, say, cfg), ledger, client
