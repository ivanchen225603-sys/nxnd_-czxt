#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
settings.py — 配置加载

设计要点
  · 所有路径都相对「项目根目录」解析，因此可以在任意工作目录下启动。
  · 完整兼容原厂的两份配置：
        config/config.json   → DLL 内部拟合参数（roi / detect_empty / packet / processing ...）
        config/devices.json  → 主程序判定门槛（calibrate.floor_gate_* ...）
    原版文件的键名、层级、默认值都尽量原样保留，方便与现场交接的旧文件直接互换。
  · config/channels.json 是本项目新增的多通道编排（原版只支持单台雷达）。
"""
from __future__ import annotations

import os
import hashlib
import json
import copy
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

# ───────────────────────────────────────────────────────── 项目根

_HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(_HERE)


def root(*parts: str) -> str:
    """把相对路径拼成项目内的绝对路径；已是绝对路径则原样返回。"""
    if not parts:
        return PROJECT_ROOT
    p = parts[0]
    if os.path.isabs(p):
        return os.path.normpath(os.path.join(p, *parts[1:]))
    return os.path.normpath(os.path.join(PROJECT_ROOT, *parts))


# ★★ 回归基线目录 —— 全项目**只在这里定义一次**。
#
#   `cli/verify.py` 的全部效力都来自这个目录下的 10 份原版点云 + 同名
#   `.reason.txt` + `MANIFEST.json`。在此之前它被写死在 14 处代码里
#   （storage / verify / replay / pc_tool / fix_plate_dirs / 若干用例），
#   改名或迁移时漏掉任意一处，闸门就会**静默地变成"自己跟自己比"**
#   并照样报"全部通过" —— 这正是本文件顶上那条"落盘目录与回归基线必须分开"
#   想防的事故，只是那时没有收口点。
#
#   ▲ 存的是**相对路径分段**，不是绝对路径：打包成 exe 后
#     `desktop/app.py` 会在启动过程中改写 `PROJECT_ROOT`，import 时算成
#     绝对路径会停在 _MEIPASS 上（详见 `storage.baseline_dir()` 的说明）。
#
#   ▲ 不要把 `ProcessingCfg.loaded_output_dir` 和 `ChannelCfg.output_dir`
#     的默认值改成用它 —— 那两个是**落盘目录的默认值**（前者还是原厂遗留键），
#     只是历史上恰好与基线同名，语义完全不同。混成一个常量，
#     基线改名那天现场落盘目录会跟着一起变。
BASELINE_SUBDIR: Tuple[str, ...] = ("baseline",)


def baseline_rel() -> str:
    """回归基线目录的相对路径，用于命令行默认值与提示文案。"""
    return "/".join(BASELINE_SUBDIR)


def ensure_dir(p: str) -> str:
    os.makedirs(p, exist_ok=True)
    return p


# ───────────────────────────────────────────────────────── JSON 读取

def _load_json(path: str) -> dict:
    """读一份配置 JSON。**用 utf-8-sig，不是 utf-8**（2026-09-22，见 docs/55 M8）。

    为什么：Windows 记事本"另存为 UTF-8"会在文件头写 BOM（EF BB BF），
    而 `json.load` 遇到 BOM 直接抛
        JSONDecodeError: Unexpected UTF-8 BOM (decode using utf-8-sig)
    —— 服务启动即死，且报错里没有一个字告诉现场"把 BOM 去掉"。
    现场要改的正是 `config/channels.json`，记事本是最顺手的工具。

    `utf-8-sig` 对**没有** BOM 的文件与 `utf-8` 行为完全一致，只是多吃一个
    可选前缀，所以这是纯增益。同仓的 `load_radar_transport.py` 早就这么读了
    （那边的 `config_path.open(encoding="utf-8-sig")`），这里只是对齐。
    """
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8-sig") as f:
        return json.load(f)


# ────────────────────────────────────────────── config.json（拟合参数）

@dataclass
class NetCfg:
    udp_port: int = 2368
    tcp_port: int = 9347
    bind_ip: str = "0.0.0.0"


@dataclass
class PacketCfg:
    """128 线雷达 UDP 报文布局。字段名沿用原 config.json。"""
    size: int = 1100
    channels: int = 128
    angle_bins: int = 900
    imu_warmup: int = 1000
    angle1_offset: int = 12
    angle2_offset: int = 526
    block1_data_offset: int = 14
    block2_data_offset: int = 528
    point_size: int = 4
    imu_offset: int = 1054
    imu_acc_x_offset: int = 30
    imu_acc_y_offset: int = 32
    imu_acc_z_offset: int = 34
    angle_bin_size: int = 40


@dataclass
class ProcessingCfg:
    offset_r: float = 15.640617
    distance_scale: float = 4.0
    angle_scale: float = 0.01
    deg_to_rad: float = 0.017453292519943295
    yaw_offset_deg: float = -93.0
    min_distance_mm: float = 100.0


@dataclass
class DetectionCfg:
    detect_interval: float = 0.5
    ground_z: float = -5.0          # 地面高度（z 向下为负）—— 有车判据的基准线
    vehicle_threshold: int = 3000   # 高于地面的点数达到它才算"有车"
    stability_delta: float = 0.05
    # 连续停稳门控始终开启；TODO(现场): 用独立运动真值标定这些工程缺省值。
    stationary_min_sec: float = 2.0
    stationary_speed_mps: float = 0.02
    stationary_drift_m: float = 0.03
    stationary_speed_window_sec: float = 0.5
    stationary_max_gap_sec: float = 0.75
    # ★ 形变/旋转侧的独立门槛（2026-09-21 现场实测）。0 = 回退到 stationary_speed_mps，
    #   行为与从前逐位一致。为什么需要它：见 detect.StabilityDetector.__init__ 的长注释。
    stationary_shape_speed_mps: float = 0.0
    stationary_yaw_speed_mps: float = 0.0
    # 微小快速平移先禁测、待新帧复核；TODO(现场): 3.1毫米为工程复核预算。
    stationary_jitter_m: float = 0.0031
    stable_save_max: int = 1
    # 车厢底板离地的最小高度。低于 ground_z + 它的点一律算地面，不计入有车判据。
    # 货车底板离地普遍 > 0.8 m，取 0.30 m 已经很保守。
    vehicle_min_height_m: float = 0.30
    # 有车判据的回差：进入"有车"要 vehicle_threshold 点，退回"空闲"要掉到
    # vehicle_threshold × 它以下。防止点数在阈值附近抖动时状态每帧乱翻。
    # 1.0 = 关闭回差（旧行为）。见 detect.StabilityDetector。
    vehicle_release_ratio: float = 0.8
    # 旧回差字段保留读取兼容，不能绕过连续停稳门控取得测量许可。
    stability_release_frames: int = 3
    stability_break_ratio: float = 6.0
    # 车点数均值趋势的附加门控；0仅关闭这一条，不关闭速度/时间/位移门控。
    stability_count_ratio: float = 0.0
    # ★ 沉降窗口：判定"已停稳"之后**再等这么多秒**才真正测量并落盘。
    #   连续停稳门控已经通过，但此时车辆和货物仍可能有余振
    #   （悬挂回弹、货物沉降、扬尘未落）。立刻测会把这些抖动一起拍进点云。
    #   窗口内若车辆再次移动，本次窗口作废、重新等停稳。
    #   设为 0 仅取消额外沉降，基础连续停稳条件仍必须满足。
    settle_sec: float = 3.0
    # 沉降窗口内是否把多帧**合并**成一份更密的点云再存。
    # 默认关：合并会改变判定链里按**绝对点数**算的门槛（min_save_points、
    # floor_gate_min_inliers），让它们更容易通过——方向可预期但确实改行为。
    # 只想要"等一会儿再测"就保持 false；想要更密的点云存档再打开。
    settle_accumulate: bool = False
    # 合并时最多保留多少帧，防止 settle_sec 配大了把内存吃光。
    settle_max_frames: int = 60


@dataclass
class FloorCfg:
    distance_threshold: float = 0.015
    z_percentile: float = 30.0
    normal_z_threshold: float = 0.99
    max_iterations: int = 10
    expand_ratio: float = 2.0
    # ★ 底面拟合前"按车厢收窄 y"（2026-09-16 加，**默认关闭**）
    #
    #   短车厢（牵引车 + 短高栏挂，厢长 5.25~6.25 m）只占 ROI 19 m 窗口的三成，
    #   车厢前方留出 6 m 以上空车道，雷达在天底附近直视地面 -> 地面进 ROI ->
    #   底面种子（z 最低 z_percentile%）被地面污染。
    #   实测上午 53 辆：短厢 6/6 有地面点（min_z 到 -5.90，地面 -5.23），
    #   长厢 0/42；判定链停在"未见厢底平面"的比例 50% vs 11%。
    #   详见 docs/08 §12.4 / §13。
    #
    #   y_trim_gap_m = 0 即关闭，行为与从前**逐位一致**。
    y_trim_gap_m: float = 0.0       # 段内允许的断口（米）；0 = 关闭整个功能
    y_trim_bin_m: float = 0.5       # 分格宽度
    y_trim_occupy: float = 0.02     # 格内点数 >= 这个比例 x 最大格，才算"被占据"
    y_trim_below_m: float = 1.0     # 旁支段要比主体低这么多，才认定是地面而丢掉


@dataclass
class HeadingCfg:
    pct: float = 15.0
    min_correction: float = 0.005


@dataclass
class RoofCfg:
    distance_threshold: float = 0.01
    z_percentile_hi: float = 10.0
    y_percentile_rear: float = 80.0


@dataclass
class WallCfg:
    distance_threshold: float = 0.015
    percentile: float = 30.0
    left_x_range: Tuple[float, float] = (-1.0, 0.0)
    right_x_range: Tuple[float, float] = (0.0, 1.0)
    # 边界面（壁/顶）的种子窄带厚度：只把"极值往内 seed_band_m"的点当种子，
    # 否则地板会混进侧壁的种子里，把面拽歪。见 detect.fit_boundary_plane。
    seed_band_m: float = 0.20


@dataclass
class OverhangCfg:
    z_margin: float = 0.10
    gap: float = 0.10
    min_points: int = 20


@dataclass
class CalibCfg:
    """config.json —— DLL 内部算法与雷达参数。"""
    path: str = ""
    network: NetCfg = field(default_factory=NetCfg)
    packet: PacketCfg = field(default_factory=PacketCfg)
    processing: ProcessingCfg = field(default_factory=ProcessingCfg)
    detection: DetectionCfg = field(default_factory=DetectionCfg)
    heading: HeadingCfg = field(default_factory=HeadingCfg)
    floor: FloorCfg = field(default_factory=FloorCfg)
    roof: RoofCfg = field(default_factory=RoofCfg)
    wall: WallCfg = field(default_factory=WallCfg)
    overhang: OverhangCfg = field(default_factory=OverhangCfg)
    roi: Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float]]] = None
    raw: Dict[str, Any] = field(default_factory=dict)


# ══════════════════════════════════════════════════════ 配置校验
#
# 2026-09-17 审查（docs/14 §1.4~§1.7）查实：**物理上不可能的门槛值被静默接受**，
# 后果是每辆车都误判、而且任何地方都不报错。实跑 10 份回归基线：
#
#   floor_gate_min_inlier_ratio = 5.0（占比 > 1）  -> 10/10「底面内点占比过低」
#   floor_gate_normal_z_min     = 2.0（|cos| > 1） -> 10/10「底面不够水平」
#   normal_z_threshold          = 2.0             -> 10/10「未见厢底平面」
#   roi 少写一个轴                                 -> roi=None，ROI **完全失效**
#
# 最后那条尤其致命：ROI 不生效 = 地面全进来，正是 2026-09-13 那次
#   「空车道判成有车 -> 会话永不结束 -> 之后的车全被静默跳过」的入口条件。
#
# 所以这里把每个数值门槛的**物理取值范围**写死成一张表，启动即校验。
# 表里只写**物理/数学上不可能**的边界，不写"看起来不合理"的经验值 ——
# 后者属于现场调参的自由，拦了会挡住正常工作。
#
# 格式：完整键路径 -> (下界, 上界, 为什么)。None 表示该侧不限。

RANGES: Dict[str, Tuple[Optional[float], Optional[float], str]] = {
    # ── config.json ──────────────────────────────────────────────
    "config.processing.angle_scale": (0.0, None, "角度标度必须为正"),
    "config.processing.distance_scale": (0.0, None, "距离标度必须为正"),
    "config.processing.min_distance_mm": (0.0, None, "距离不能为负"),
    "config.detection.detect_interval": (0.0, None, "间隔不能为负"),
    "config.detection.vehicle_threshold": (0.0, None, "点数阈值不能为负"),
    "config.detection.stability_delta": (0.0, None, "偏差阈值不能为负"),
    "config.detection.stationary_min_sec": (0.001, None, "停稳持续时间必须为正"),
    "config.detection.stationary_speed_mps": (0.000001, None, "停稳速度必须为正"),
    "config.detection.stationary_shape_speed_mps": (0.0, None, "形变速度门槛不能为负（0=回退到停稳速度）"),
    "config.detection.stationary_yaw_speed_mps": (0.0, None, "旋转速度门槛不能为负（0=回退到停稳速度）"),
    "config.detection.stationary_drift_m": (0.000001, None, "停稳位移必须为正"),
    "config.detection.stationary_speed_window_sec": (0.001, None, "速度窗口必须为正"),
    "config.detection.stationary_max_gap_sec": (0.001, None, "观测间隔必须为正"),
    "config.detection.stationary_jitter_m": (0.000001, None, "抖动复核位移必须为正"),
    "config.detection.stable_save_max": (1.0, None, "每会话至少允许存 1 次"),
    "config.detection.vehicle_min_height_m": (0.0, None, "高度不能为负"),
    "config.detection.vehicle_release_ratio": (0.0, 1.0, "回差是比例，必须在 0~1"),
    "config.detection.stability_release_frames": (1.0, None, "至少 1 帧"),
    "config.detection.stability_break_ratio": (1.0, None, "打断倍数不应小于 1"),
    "config.detection.stability_count_ratio": (0.0, 1.0, "点数变化是比例"),
    "config.detection.settle_sec": (0.0, None, "沉降窗口不能为负"),
    "config.detection.settle_max_frames": (1.0, None, "至少留 1 帧"),
    "config.detect_empty.heading.pct": (0.0, 100.0, "百分位"),
    "config.detect_empty.floor.distance_threshold": (0.0, None, "距离阈值必须为正"),
    "config.detect_empty.floor.z_percentile": (0.0, 100.0,
                                               "百分位；越界会让 np.percentile 直接抛异常"),
    "config.detect_empty.floor.normal_z_threshold": (0.0, 1.0,
                                                     "它是 |法向 z| = |cos|，不可能 > 1"),
    "config.detect_empty.floor.max_iterations": (1.0, None, "至少迭代 1 次"),
    "config.detect_empty.floor.expand_ratio": (1.0, None, "放宽倍数不应小于 1"),
    "config.detect_empty.floor.y_trim_gap_m": (0.0, None, "断口不能为负；0 = 关闭"),
    "config.detect_empty.floor.y_trim_bin_m": (0.0, None, "分格宽度必须为正"),
    "config.detect_empty.floor.y_trim_occupy": (0.0, 1.0, "占据比是比例"),
    "config.detect_empty.floor.y_trim_below_m": (0.0, None, "高度差不能为负"),
    "config.detect_empty.roof.distance_threshold": (0.0, None, "距离阈值必须为正"),
    "config.detect_empty.roof.z_percentile_hi": (0.0, 100.0, "百分位"),
    "config.detect_empty.roof.y_percentile_rear": (0.0, 100.0, "百分位"),
    "config.detect_empty.wall.distance_threshold": (0.0, None, "距离阈值必须为正"),
    "config.detect_empty.wall.percentile": (0.0, 100.0, "百分位"),
    "config.detect_empty.overhang.min_points": (0.0, None, "点数不能为负"),
    "config.packet.size": (1.0, None, "报文长度必须为正"),
    "config.packet.channels": (1.0, None, "通道数必须为正"),
    "config.packet.angle_bins": (1.0, None, "角度分箱必须为正"),
    "config.packet.point_size": (1.0, None, "每点字节数必须为正"),
    "config.network.udp_port": (1.0, 65535.0, "端口号"),
    "config.network.tcp_port": (1.0, 65535.0, "端口号"),
    # ── devices.json ─────────────────────────────────────────────
    "devices.calibrate.floor_gate_distance_m": (0.0, None, "距离阈值必须为正"),
    "devices.calibrate.floor_gate_normal_z_min": (0.0, 1.0,
                                                  "它是 |法向 z| = |cos|，不可能 > 1"),
    "devices.calibrate.floor_gate_z_above_min_max_m": (0.0, None,
                                                       "它是一个高度门槛，负数没有意义"),
    "devices.calibrate.floor_gate_min_inliers": (0.0, None, "点数不能为负"),
    "devices.calibrate.floor_gate_min_inlier_ratio": (0.0, 1.0, "它是占比，不可能 > 1"),
    "devices.calibrate.floor_gate_min_span_x_m": (0.0, None, "跨度不能为负"),
    "devices.calibrate.floor_gate_min_span_y_m": (0.0, None, "跨度不能为负"),
    "devices.calibrate.floor_gate_bed_exposure_min": (0.0, 1.0, "底板露出门槛是占比，必须在 0~1（0=关闭）"),
    "devices.calibrate.floor_gate_bed_flatness_max": (0.0, None, "平整度门槛不能为负（0=关闭）"),
    "devices.calibrate.floor_gate_bed_thickness_min": (0.0, None, "厚度门槛不能为负（0=不要求）"),
    "devices.calibrate.floor_gate_floor_height_max_m": (0.0, None, "离地高度上限不能为负（0=关闭）"),
    "devices.calibrate.min_save_points": (0.0, None, "点数不能为负"),
    "devices.calibrate.stationary_warn_sec": (0.0, None, "秒数不能为负"),
    "devices.calibrate.result_hold_sec": (0.0, None, "秒数不能为负"),
    "devices.calibrate.dims_min_length_m": (0.0, None, "尺寸不能为负"),
    "devices.calibrate.dims_min_width_m": (0.0, None, "尺寸不能为负"),
    "devices.calibrate.dims_min_height_m": (0.0, None, "尺寸不能为负"),
    "devices.fusion.imu_warmup_sec": (0.0, None, "秒数不能为负"),
    "devices.fusion.session_timeout_sec": (0.0, None, "秒数不能为负"),
}


def check_range(path: str, full_key: str, value) -> None:
    """按 RANGES 校验一个值。越界就抛 ValueError（**启动即失败**，不静默）。"""
    rule = RANGES.get(full_key)
    if rule is None:
        return
    lo, hi, why = rule
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise ValueError("%s: %s 必须是数字，当前是 %r"
                         % (path, full_key.split(".", 1)[1], value)) from None
    if v != v:
        raise ValueError("%s: %s 不能是 NaN" % (path, full_key.split(".", 1)[1]))
    # ★ ±inf 与 NaN 同罪（2026-09-23，低严重度批次）：JSON 里写 1e400
    # 会解析成 inf，一切 "(0.0, None)" 型的"无上界"规则都拦不住它 ——
    # settle_sec=inf 等于永久等待沉降，detect_interval=inf 等于帧率归零。
    # 物理量在数学上就不可能是无穷，按本表"只写物理/数学不可能边界"的
    # 口径，非有限值一律拒。
    if v in (float("inf"), float("-inf")):
        raise ValueError("%s: %s 不能是无穷大（当前 %g）" %
                         (path, full_key.split(".", 1)[1], v))
    if (lo is not None and v < lo) or (hi is not None and v > hi):
        rng = "[%s, %s]" % ("-inf" if lo is None else ("%g" % lo),
                            "+inf" if hi is None else ("%g" % hi))
        raise ValueError("%s: %s = %g 超出允许范围 %s —— %s"
                         % (path, full_key.split(".", 1)[1], v, rng, why))


def check_unknown(path: str, section: str, data, allowed, comments: bool = False) -> None:
    """段里出现了不认识的键就报错。

    为什么要拦：配置项打错名字（`floor_gate_min_inliers` 少个 s 之类）时，
    **旧的默认值会继续生效而没有任何提示** —— 现场以为改了，其实没改。
    2026-09-17 实测：config.json / devices.json 的所有层级都静默接受未知键。

    `comments=True` 时跳过以 `_` 开头的键。只有 `channels.json` 用得上 ——
    它是本项目自己的配置（不是原厂文件），里面按约定写了 `_说明` 这类注释键。
    原厂两份配置没有这种键，所以默认关着，那三处调用行为逐字不变。
    """
    if not isinstance(data, dict):
        return
    if comments:
        data = {k: v for k, v in data.items() if not str(k).startswith("_")}
    bad = sorted(set(data) - set(allowed))
    if bad:
        raise ValueError(
            "%s: %s 段里有不认识的配置项 %s。\n"
            "  认识的是：%s\n"
            "  （打错名字时旧默认值会继续生效且不报错，所以这里直接拦下来）"
            % (path, section or "顶层", "、".join(repr(b) for b in bad),
               "、".join(sorted(allowed))))


def _fill(obj, data: dict, mapping: Dict[str, str], ints=(), tuples=(), strs=(),
          bools=(), path: str = "", section: str = ""):
    """把 dict 里存在的键填进 dataclass 实例（缺键保持默认值）。

    字段类型由调用方通过 ints / tuples / strs 声明；未声明的按 float 处理。
    strs 必须先于 float 处理，否则 bind_ip 这类字符串会被强转抛错。

    ★ 传了 path + section 就同时做两道校验（docs/14 §1.4~§1.7）：
      · **未知键** —— mapping 本身就是"这一段认哪些键"的权威，直接拿它比；
      · **取值范围** —— 查 RANGES 表。
      校验挂在这里而不是另写一张表，是为了**不会漂移**：
      以后给某一段加字段，只要改 mapping，两道校验自动跟上。
    """
    if not isinstance(data, dict):
        return
    if path and section is not None:
        check_unknown(path, section, data, set(mapping.values()))
    for attr, key in mapping.items():
        if key not in data or data[key] is None:
            continue
        if path:
            check_range(path, "%s.%s" % (section, key) if section else key, data[key])
        v = data[key]
        if attr in strs:
            v = str(v)
        elif attr in bools:
            v = bool(v)
        elif attr in ints:
            v = int(v)
        elif attr in tuples:
            v = tuple(float(x) for x in v)
        else:
            v = float(v)
        setattr(obj, attr, v)


# ══════════════════════════════════════════════════════ 配置指纹
#
# 2026-09-17 审查（docs/14 §1.8）查实：**落盘的测量 meta 里没有任何配置信息**，
# 于是存档结论会随配置变化而悄悄过期 —— 62 份现场样本里已经有 **7 份（11%）**
# 的 meta reason 与用当前配置重算的结果不一致，其中 2 份现在是「空车」
# 而 meta 记的是「底面过高，疑似载物」。
#
# 这件事要紧，是因为 docs/13 那套「按车牌存空车基线」的设计**正是照 meta 的
# `empty` 去挑基线的** —— 拿过期的结论挑，会把载货点云存成空车基线，
# 之后那辆车的体积恒为接近 0，而且不报错。
#
# 对照：`raw/` 的旁注 json 一直记着 effective_yaw_offset_deg / ground_z /
# roi / range_mode（所以 roi_audit 能复算），只有测量 meta 什么都没记。
#
# ▲ 指纹覆盖的是**会改变判定结论**的那些项。加新的此类配置项时，
#   `_FINGERPRINT_CONFIG_KEYS` / `_FINGERPRINT_DEVICES_KEYS` 要同步 ——
#   漏了的话指纹不会变，过期就又察觉不到了。
#   `tests/test_config_validation.py::TestConfigFingerprint::
#    test_every_judgement_gate_changes_the_sha` 按**这张表本身**自动取键，
#   表里漏了谁、多了谁，那条用例当场就红。
#   （这里曾写过 `TestFingerprintCoversTheChain` —— 注释点了名、测试根本
#    不存在，两轮审查 H3 查实后改成本句，见 docs/55。）

# config.json 里影响判定的：整段 detect_empty（六面拟合的全部参数）、
# roi、航向、地面高度。不含 network / packet（只影响解码，不影响已解码点云的判定）。
_FINGERPRINT_CONFIG_KEYS = (
    "roi",
    "detect_empty",
    "processing.yaw_offset_deg",
    "detection.ground_z",
)
# devices.json 里影响判定的：7 个 floor_gate_* + 度量模式 + 落盘门槛。
# 不含 loaded_output_dir / lidar_ip 这类与结论无关的。
_FINGERPRINT_DEVICES_KEYS = (
    "calibrate.floor_gate_distance_m",
    "calibrate.floor_gate_normal_z_min",
    "calibrate.floor_gate_z_above_min_max_m",
    "calibrate.floor_gate_z_above_mode",
    "calibrate.floor_gate_min_inliers",
    "calibrate.floor_gate_min_inlier_ratio",
    "calibrate.floor_gate_min_span_x_m",
    "calibrate.floor_gate_min_span_y_m",
    "calibrate.floor_gate_bed_exposure_min", "calibrate.floor_gate_bed_flatness_max",
    "calibrate.floor_gate_bed_thickness_min", "calibrate.floor_gate_floor_height_max_m",
    "calibrate.min_save_points",
)


def _dig(d, dotted: str):
    cur = d
    for k in dotted.split("."):
        if not isinstance(cur, dict) or k not in cur:
            return None
        cur = cur[k]
    return cur


def config_fingerprint(calib, dev, channel_yaw_deg=None) -> Dict[str, Any]:
    """算这一次判定所用配置的指纹，写进落盘 meta。

    返回的 dict 有两部分：
      · `sha` —— 上面两张表覆盖的全部配置项的 SHA-256 前 12 位。
        比对它就知道"是不是同一套配置"，不用逐项比。
      · 几个**人能直接看懂**的字段（`z_above_mode` / `yaw_offset_deg` /
        `ground_z` / `roi`）—— 指纹对不上时，光看这几个往往就知道差在哪。

    `channel_yaw_deg` 是该通道 `channels.json` 里的 `lidar.yaw_offset_deg`
    （叠加在共用值上；没设就传 None）。它同样进指纹 —— 逐通道 yaw 改了
    会实打实改变判定结果（2026-09-16 改过一次，正是那 7 份不一致的成因之一）。
    """
    payload: List[Any] = []
    craw = getattr(calib, "raw", None) or {}
    draw = getattr(dev, "raw", None) or {}
    for k in _FINGERPRINT_CONFIG_KEYS:
        payload.append((("config." + k), _dig(craw, k)))
    for k in _FINGERPRINT_DEVICES_KEYS:
        payload.append((("devices." + k), _dig(draw, k)))
    payload.append(("channel.lidar.yaw_offset_deg", channel_yaw_deg))
    blob = json.dumps(payload, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"))
    sha = hashlib.sha256(blob.encode("utf-8")).hexdigest()[:12]
    yaw = channel_yaw_deg
    if yaw is None:
        yaw = _dig(craw, "processing.yaw_offset_deg")
    return {
        "sha": sha,
        "z_above_mode": _dig(draw, "calibrate.floor_gate_z_above_mode") or "origin",
        "yaw_offset_deg": None if yaw is None else float(yaw),
        "ground_z": _dig(craw, "detection.ground_z"),
        "roi": _dig(craw, "roi"),
    }


def config_mismatch(saved, current) -> List[str]:
    """比对落盘 meta 里的指纹与当前配置，返回人话的差异清单（一致就是空表）。

    `saved` 是 meta 里的 `config` 字段（老样本没有这个字段 -> 返回一条说明）。
    """
    if not isinstance(saved, dict) or not saved.get("sha"):
        return ["这份样本落盘时还没有记配置指纹 —— 无从判断它是哪套配置产出的"]
    if saved.get("sha") == current.get("sha"):
        return []
    out = ["配置指纹不一致：落盘 %s -> 现在 %s" % (saved.get("sha"), current.get("sha"))]
    for k, cn in (("z_above_mode", "第 10 步的度量"), ("yaw_offset_deg", "航向角"),
                  ("ground_z", "地面高度"), ("roi", "ROI")):
        a, b = saved.get(k), current.get(k)
        if a != b:
            out.append("  %s：%r -> %r" % (cn, a, b))
    if len(out) == 1:
        out.append("  （上面这几项没变，差异在 detect_empty / floor_gate 的其它参数里）")
    return out


# config.json 顶层与 detect_empty 下允许出现的段名。加新段时这里要同步。
_CONFIG_SECTIONS = {"network", "packet", "processing", "roi", "detection", "detect_empty"}
_DETECT_EMPTY_SECTIONS = {"heading", "floor", "roof", "wall",
                          "left_wall", "right_wall", "overhang"}


def load_calib(path: str) -> CalibCfg:
    d = _load_json(path)
    c = CalibCfg(path=path, raw=d)
    if not d:
        return c

    check_unknown(path, "", d, _CONFIG_SECTIONS)
    _fill(c.network, d.get("network", {}),
          {"udp_port": "udp_port", "tcp_port": "tcp_port", "bind_ip": "bind_ip"},
          ints=("udp_port", "tcp_port"), strs=("bind_ip",),
          path=path, section="config.network")

    pk = PacketCfg()
    _fill(pk, d.get("packet", {}), {k: k for k in
          ("size", "channels", "angle_bins", "imu_warmup", "angle1_offset", "angle2_offset",
           "block1_data_offset", "block2_data_offset", "point_size", "imu_offset",
           "imu_acc_x_offset", "imu_acc_y_offset", "imu_acc_z_offset", "angle_bin_size")},
          ints=("size", "channels", "angle_bins", "imu_warmup", "angle1_offset", "angle2_offset",
                "block1_data_offset", "block2_data_offset", "point_size", "imu_offset",
                "imu_acc_x_offset", "imu_acc_y_offset", "imu_acc_z_offset", "angle_bin_size"),
          path=path, section="config.packet")
    c.packet = pk

    _fill(c.processing, d.get("processing", {}),
          {k: k for k in ("offset_r", "distance_scale", "angle_scale", "deg_to_rad",
                          "yaw_offset_deg", "min_distance_mm")},
          path=path, section="config.processing")
    _fill(c.detection, d.get("detection", {}),
          {k: k for k in ("detect_interval", "ground_z", "vehicle_threshold",
                          "stability_delta", "stable_save_max", "vehicle_min_height_m",
                          "vehicle_release_ratio", "stability_release_frames",
                          "stability_break_ratio", "stability_count_ratio",
                          "stationary_min_sec", "stationary_speed_mps", "stationary_drift_m",
                          "stationary_speed_window_sec", "stationary_max_gap_sec", "stationary_jitter_m",
                          "stationary_shape_speed_mps", "stationary_yaw_speed_mps",
                          "settle_sec", "settle_accumulate", "settle_max_frames")},
          ints=("vehicle_threshold", "stable_save_max", "settle_max_frames",
                "stability_release_frames"),
          bools=("settle_accumulate",),
          path=path, section="config.detection")
    for key in ("stationary_min_sec", "stationary_speed_mps", "stationary_drift_m",
                "stationary_speed_window_sec", "stationary_max_gap_sec", "stationary_jitter_m",
                "stationary_shape_speed_mps", "stationary_yaw_speed_mps"):
        if not math.isfinite(getattr(c.detection, key)):
            raise ValueError("%s: detection.%s 必须为有限正数" % (path, key))
    if c.detection.stationary_speed_window_sec > c.detection.stationary_min_sec:
        raise ValueError("速度窗口不能长于连续停稳时间")
    if c.detection.stationary_max_gap_sec >= c.detection.stationary_min_sec:
        raise ValueError("最大观测间隔必须短于连续停稳时间")
    if c.detection.stationary_jitter_m >= c.detection.stationary_drift_m:
        raise ValueError("抖动复核位移必须小于固定锚点位移门槛")

    de = d.get("detect_empty", {})
    check_unknown(path, "detect_empty", de, _DETECT_EMPTY_SECTIONS)
    _fill(c.heading, de.get("heading", {}),
          {"pct": "pct", "min_correction": "min_correction"},
          path=path, section="config.detect_empty.heading")
    _fill(c.floor, de.get("floor", {}),
          {k: k for k in ("distance_threshold", "z_percentile", "normal_z_threshold",
                          "max_iterations", "expand_ratio",
                          "y_trim_gap_m", "y_trim_bin_m", "y_trim_occupy",
                          "y_trim_below_m")},
          ints=("max_iterations",),
          path=path, section="config.detect_empty.floor")
    _fill(c.roof, de.get("roof", {}),
          {k: k for k in ("distance_threshold", "z_percentile_hi", "y_percentile_rear")},
          path=path, section="config.detect_empty.roof")
    _fill(c.wall, de.get("wall", {}),
          {"distance_threshold": "distance_threshold", "percentile": "percentile",
           "seed_band_m": "seed_band_m"},
          path=path, section="config.detect_empty.wall")
    if de.get("left_wall", {}).get("x_range"):
        c.wall.left_x_range = tuple(float(x) for x in de["left_wall"]["x_range"])
    if de.get("right_wall", {}).get("x_range"):
        c.wall.right_x_range = tuple(float(x) for x in de["right_wall"]["x_range"])
    _fill(c.overhang, de.get("overhang", {}),
          {"z_margin": "z_margin", "gap": "gap", "min_points": "min_points"},
          ints=("min_points",),
          path=path, section="config.detect_empty.overhang")

    r = d.get("roi")
    if r is not None:
        # ★ 少写一个轴会让 roi 变成 None，而 apply_roi 对 None 是**完全空操作** ——
        #   ROI 静默失效 = 地面全进来，正是 2026-09-13 那次事故的入口条件
        #   （docs/14 §1.5）。所以这里宁可启动就崩，也不能放过去。
        check_unknown(path, "roi", r, {"x", "y", "z"})
        miss = [k for k in ("x", "y", "z") if k not in r]
        if miss:
            raise ValueError(
                "%s: roi 少了 %s 轴。三个轴必须都写全 —— 少一个的话 roi 会变成 None，"
                "而 apply_roi 对 None 是完全不裁剪的（ROI 静默失效）。" % (path, "、".join(miss)))
    if r and all(k in r for k in ("x", "y", "z")):
        for ax in ("x", "y", "z"):
            v = r[ax]
            if not (isinstance(v, (list, tuple)) and len(v) == 2):
                raise ValueError(f"{path}: roi.{ax} 必须是 [下界, 上界] 两个数，当前是 {v!r}")
            if float(v[0]) >= float(v[1]):
                raise ValueError(
                    f"{path}: roi.{ax} = {list(v)} 写反了（下界 >= 上界）。"
                    f"这样裁出来会是**空点云且不报错**，必须改成 [小, 大]。"
                    f"注意 z 向下为负，所以 z 应该写成 [-6, -0.75] 而不是 [-0.75, -6]。")
        # 统一成 (下界 xyz, 上界 xyz) —— 与 pointcloud.normalize_roi 一致。
        # config.json 原文是按轴写的 ((x0,x1),(y0,y1),(z0,z1))，这里做一次转换，
        # 之后全项目只认这一种形式，避免"两种 ROI 写法"互相踩。
        c.roi = ((float(r["x"][0]), float(r["y"][0]), float(r["z"][0])),
                 (float(r["x"][1]), float(r["y"][1]), float(r["z"][1])))
    return c


# ─────────────────────────────────────────── devices.json（判定门槛）

@dataclass
class GateCfg:
    """
    空车判定门槛。这 7 个键与反汇编出的参数结构体偏移一一对应：

        +0x40  floor_gate_distance_m          点到拟合底面的归一化距离
        +0x44  floor_gate_normal_z_min        |底面法向.z| 下限
        +0x48  floor_gate_z_above_min_max_m   「底面过高，疑似载物」的容差
        +0x50  floor_gate_min_inliers         最少内点数
        +0x58  floor_gate_min_inlier_ratio    最少内点占比
        +0x5c  floor_gate_min_span_x_m        内点 X 跨度下限
        +0x60  floor_gate_min_span_y_m        内点 Y 跨度下限
    """
    distance_m: float = 0.015
    normal_z_min: float = 0.99
    z_above_min_max_m: float = 0.08
    min_inliers: int = 600
    min_inlier_ratio: float = 0.15
    min_span_x_m: float = 1.8
    min_span_y_m: float = 5.0
    # ★ 第 10 步的**底板露出兜底**（2026-09-21 加，原厂没有这个键）
    #
    #   0.0（默认）= 关闭，行为逐位不变。
    #
    #   为什么需要：第 10 步的 z_above 在空车与载货之间**完全穿插**，
    #   4 份空车真值实测 0.0409 / 0.0607 / 0.3308 / 0.3617，
    #   而载货是 0.3024 / 0.9422 —— 载货那个 0.3024 正夹在空车中间。
    #   **调门槛救不了任何一辆空车**，只会同时放行载货。
    #
    #   而"底板在整个车厢内露出多少"分得干干净净：
    #       空车 100.0% / 100.0% / 99.5% / 98.2%   载货 0.0% / 0.0%
    #   间隙 98 个百分点（124 份全量实测也是强双峰，模糊带只有 3.2%）。
    #   半装载也拦得住 —— 它的剖面集中在一头（100/19/0/0/0/0/0/0），
    #   总露出只有约 12%，够不到 0.9 的门槛。
    #
    #   ▲ 高度必须用**离地绝对值**，所以 evaluate_empty 要拿到 ground_z；
    #     拿不到时这条兜底自动失效，回到原行为。
    #
    #   ▲▲ **2026-09-21 实测：单靠它不安全，所以现场值设回 0（关闭）。**
    #      薄载会骗过它：底板在离地 1.2 m 的车装 0.3 m 薄煤，货面 1.5 m
    #      **正好落在底板高度带 [0.9, 1.9] 内**，于是每格最低回波都算成
    #      "底板露出"，exposure 冲到 100% -> 判成空车。
    #      `tests/test_service.py::TestServiceDemo::test_05_loaded_scene_is_not_empty`
    #      抓到了这件事（合成载货场景地板 0.23 m、货顶 1.40 m，正是这个形态）。
    #      而"把载货判成空车"是最危险的方向 —— 它会被存成空车基线，
    #      之后那辆车的体积恒为接近 0，**且不报错**。
    #
    #      要启用必须再加一条独立证据，候选（都还没验）：
    #        · 货面离**壁顶**够远（参考项目 CarPointCloud_Analyzer 的
    #          `min_visible_wall_depth=0.80`：可见表面贴近栏板顶 = 货面或苫布）；
    #        · 车厢带内 z 厚度（4 份真值实测 空车 2.42~2.96 vs 载货 0.64~0.83）；
    #        · 中部高度带（归一化 0.2~0.8）的点密度 —— 空车点集中在底板。
    #      样本只有 4 份空车真值，定不了新门槛，见 docs/41 第 0f 节。
    bed_exposure_min: float = 0.0
    # ★★ 第 10 步的**自参考**兜底（2026-09-21 定案）—— 不依赖任何绝对高度。
    #
    #   为什么不用底板露出：它靠"最低回波落不落在离地 [0.9, 1.9]"判，
    #   而**每辆车的车厢高度不固定**，固定带必然两头为难 ——
    #   62 份实测里已经有 7 份载货的最低面落进带内（1.828~1.9），
    #   上界压在载货群里；而带太窄又会漏掉高底盘的空车。
    #
    #   改用两条**这辆车自己量出来**的量，都与车厢离地多高无关：
    #
    #   ① 底板平整度（拟合残差 RMS）—— 底板是制造出来的平面，煤面是糙的。
    #      62 份实测：空车 0.0189~0.0244，其余 58 份最小 0.0421，**零重叠**。
    #   ② 车厢带内 z 厚度（壁顶 - 最低可见面）—— 空车看到的是底板，
    #      厚度就是整个厢深；装了货，看到的是货面，厚度被吃掉。
    #      62 份实测：空车 2.423~2.957，其余 58 份最大 1.692，**零重叠**。
    #
    #   ▲ **两条必须同时满足**，单用任一条都有洞：
    #      · 只看平整度 -> 铺平的非煤货物（砂石/板材）货顶也很平；
    #        合成载货场景的货顶是完美平面，残差约 0，光看平整度必放行。
    #      · 只看厚度   -> 124 份全量实测厚度是连续谱，1.2~2.4 之间压着
    #        43 份（35% 模糊带，见 docs/13 §12.1 的作废说明）。
    #      合起来：合成载货被厚度拦（1.43 < 2.2），糙煤被平整度拦。
    #
    #   ▲ 前提是**车厢切对了**。2026-09-13 那片"比底板还平"的地面（残差 0.013）
    #     是 yaw 配错把整片地面当成车厢切进来的 —— 输入错了，任何基于
    #     车厢内点云的判据都不成立，不止平整度。切车厢质量由厢长/内宽
    #     在量程内保证（那片地面宽 4 m，内宽闸门直接拦）。
    #
    #   0.0 = 关闭（默认），行为逐位不变。
    bed_flatness_max: float = 0.0
    bed_thickness_min: float = 0.0
    # ★ 第 10 步用哪种度量（2026-09-16 加，原厂没有这个键）
    #
    #   "origin"（原厂行为）：z_above = 平面在**原点**处的 z - **整片**点云最低点。
    #       两个量取的不是同一个地方，底板一斜就差出落差的一半 ——
    #       空载自卸车底板向车头翘 3.8~4.1 度，于是 z_above=0.32/0.35 远超 0.08，
    #       **空车被判成载货**。2026-09-15 两辆现场确认的空车都栽在这里。
    #   "plane"（倾斜无关）：z_above = 点在拟合平面**下方**深度的 p99.9。
    #       语义才是这一步的本意 ——「底面下面还有东西 = 拟合到的不是真底板」。
    #       用 p99.9 不用 max 是为了躲开离群点（基线 190520 有 4 个地面点，
    #       max 冲到 1.738，p99.9 是 0.033）。
    #
    #   ▲ 换成 "plane" **会与原版在 4 份回归基线上分道扬镳** —— 那 4 份本身就是
    #     空车（拟合面落在底板高度 1.51~1.64 m），原版把它们误判成了载货。
    #     详见 docs/12 §4.1 / §6。
    z_above_mode: str = "origin"
    # ★★ 第 10 步的**平面离地上限**（2026-09-24 现场加，原厂没有这个键）。
    #
    #   拟合出的"底面"平面在原点处离地面（detection.ground_z）超过这个高度，
    #   就判「底面过高，疑似载物」——那不是底板，是**被刮平的煤面**。
    #   为什么需要：2026-09-24 上午 CH2 一辆满载车，煤面被刮得很平
    #   （内点占比 0.205 过了第 7 步），平面离地 3.03 m、底板一处都没露出，
    #   第 10 步的 z_above 只有 0.047（平面下面确实没东西 —— 下面是煤）。
    #   最后只因内点 X 跨度 1.795 < 1.8 **差 4.8 mm** 被第 8 步拦下；
    #   宽 5 mm 就会把满载车判成空车。第 10 步两种度量都量"平面下方"，
    #   对"平面本身就是货面"天生失明 —— 俯视单视角的固有极限（CLAUDE.md 已知限制）。
    #   门槛取值：空车底板平面实测 1.30~1.64 m（4 份基线 + 2 辆现场真值 + B 型短厢推断），
    #   载货拟合面 2.93~3.75 m（历史 186 帧里走到第 8~10 步的 7 帧 + 09-24 当天 13 份），
    #   取 2.3。▲ 历史 186 帧全量平面离地 p10 是 2.39，载货低端离它不远，别往上调。
    #   ▲ 只在 z_above_mode="plane" 且拿得到 ground_z 时生效（与第 10 步兜底同一规矩：
    #     origin 档是原厂行为的定义，必须与 calibrate_test.exe 逐字一致）。
    #   ▲ 它是**绝对高度**判据，而车厢离地高度因车而异 —— 所以只用来拦
    #     "明显高于任何底板"的面，门槛两侧各留 0.6 m 以上，别往 1.9 附近调。
    #   0.0 = 关闭（默认），行为逐位不变。
    floor_height_max_m: float = 0.0


@dataclass
class DevicesCfg:
    path: str = ""
    gate: GateCfg = field(default_factory=GateCfg)
    # fusion / calibrate 段的其它键
    imu_warmup_sec: float = 5.0
    session_timeout_sec: float = 180.0
    entry_baseline_dir: str = "data/in"
    stationary_warn_sec: float = 30.0
    min_save_points: int = 20000
    dims_min_length_m: float = 10.0
    dims_min_width_m: float = 2.0
    dims_min_height_m: float = 1.5
    result_hold_sec: float = 8.0
    loaded_output_dir: str = "data/loaded"
    lidar_config_path: str = "config/config.json"
    lidar_ip: str = "192.168.1.201"
    ip_override: str = ""
    raw: Dict[str, Any] = field(default_factory=dict)


# devices.json 允许出现的段与键。加新项时这里要同步 —— 不同步的话
# 新项会被当成"不认识的配置项"拦下来，而不是被静默忽略，这正是我们要的。
_DEVICES_SECTIONS = {"lidar", "fusion", "calibrate"}
_DEVICES_LIDAR_KEYS = {"config_path", "lidar_ip", "ip_override"}
_DEVICES_FUSION_KEYS = {"imu_warmup_sec", "session_timeout_sec", "entry_baseline_dir"}
_DEVICES_CALIBRATE_KEYS = {
    "stationary_warn_sec", "min_save_points", "dims_min_length_m", "dims_min_width_m",
    "dims_min_height_m", "result_hold_sec", "loaded_output_dir",
    "floor_gate_distance_m", "floor_gate_normal_z_min", "floor_gate_z_above_min_max_m",
    "floor_gate_z_above_mode", "floor_gate_min_inliers", "floor_gate_min_inlier_ratio",
    "floor_gate_min_span_x_m", "floor_gate_min_span_y_m",
    "floor_gate_bed_exposure_min", "floor_gate_bed_flatness_max",
    "floor_gate_bed_thickness_min", "floor_gate_floor_height_max_m",
}


def load_devices(path: str) -> DevicesCfg:
    d = _load_json(path)
    c = DevicesCfg(path=path, raw=d)
    if not d:
        return c

    # ★ 三道校验，理由见 RANGES 上面那段说明与 docs/14 §1.4~§1.7。
    check_unknown(path, "", d, _DEVICES_SECTIONS)
    lid = d.get("lidar", {})
    check_unknown(path, "lidar", lid, _DEVICES_LIDAR_KEYS)
    c.lidar_config_path = lid.get("config_path", c.lidar_config_path)
    c.lidar_ip = lid.get("lidar_ip", c.lidar_ip)
    c.ip_override = lid.get("ip_override", "") or ""

    fu = d.get("fusion", {})
    check_unknown(path, "fusion", fu, _DEVICES_FUSION_KEYS)
    for _k in _DEVICES_FUSION_KEYS:
        if _k in fu:
            check_range(path, "devices.fusion." + _k, fu[_k])
    c.imu_warmup_sec = float(fu.get("imu_warmup_sec", c.imu_warmup_sec))
    c.session_timeout_sec = float(fu.get("session_timeout_sec", c.session_timeout_sec))
    c.entry_baseline_dir = fu.get("entry_baseline_dir", c.entry_baseline_dir)

    ca = d.get("calibrate", {})
    check_unknown(path, "calibrate", ca, _DEVICES_CALIBRATE_KEYS)
    for _k in ca:
        check_range(path, "devices.calibrate." + _k, ca[_k])
    g = c.gate
    for attr, key, is_int in (
        ("distance_m", "floor_gate_distance_m", False),
        ("normal_z_min", "floor_gate_normal_z_min", False),
        ("z_above_min_max_m", "floor_gate_z_above_min_max_m", False),
        ("min_inliers", "floor_gate_min_inliers", True),
        ("min_inlier_ratio", "floor_gate_min_inlier_ratio", False),
        ("min_span_x_m", "floor_gate_min_span_x_m", False),
        ("min_span_y_m", "floor_gate_min_span_y_m", False),
        ("bed_exposure_min", "floor_gate_bed_exposure_min", False),
        ("bed_flatness_max", "floor_gate_bed_flatness_max", False),
        ("bed_thickness_min", "floor_gate_bed_thickness_min", False),
        ("floor_height_max_m", "floor_gate_floor_height_max_m", False),
    ):
        if key in ca:
            setattr(g, attr, int(ca[key]) if is_int else float(ca[key]))
    if "floor_gate_z_above_mode" in ca:
        m = str(ca["floor_gate_z_above_mode"]).strip().lower()
        if m not in ("origin", "plane"):
            raise ValueError(
                f"{path}: calibrate.floor_gate_z_above_mode 只能是 "
                f"'origin'（原厂行为）或 'plane'（倾斜无关），当前是 {m!r}")
        g.z_above_mode = m
    for attr, key, is_int in (
        ("stationary_warn_sec", "stationary_warn_sec", False),
        ("min_save_points", "min_save_points", True),
        ("dims_min_length_m", "dims_min_length_m", False),
        ("dims_min_width_m", "dims_min_width_m", False),
        ("dims_min_height_m", "dims_min_height_m", False),
        ("result_hold_sec", "result_hold_sec", False),
    ):
        if key in ca:
            setattr(c, attr, int(ca[key]) if is_int else float(ca[key]))
    if "loaded_output_dir" in ca:
        c.loaded_output_dir = str(ca["loaded_output_dir"])
    return c


# ────────────────────────────────────────────── channels.json（编排）

@dataclass
class LidarCfg:
    ip: str = "192.168.1.201"           # 该通道雷达自身地址（多雷达时=源 IP）
    source_ip: str = ""                 # 多雷达共用端口时按此分流；留空则用 ip
    udp_port: int = 2368
    tcp_port: int = 9347
    angle_cache: str = "config/angle_cache.csv"
    bind_ip: str = ""                  # 留空 = 跟随 config.json 的 network.bind_ip
    range_mode: str = "raw_offset"     # 距离换算方式，见 radar_packet.RANGE_MODES
    z_down: bool = True                # z 向下为负（雷达吊装俯视），已交叉验证
    # 切帧方式，见 radar_source.LiveSource：
    #   rotation —— 按方位角回绕切帧（一帧＝一整圈），与转速无关。**现场用这个**
    #   time     —— 按 frame_ms 时间窗切帧（旧行为）。时间窗与转圈不同步时，
    #                每帧只是一片来回转的扇面：看板点云乱跳、质心乱跳、
    #                停稳状态每帧乱翻。只在排查切帧本身时才临时切回来。
    frame_mode: str = "rotation"
    frame_ms: int = 100                # time 模式的时间窗；rotation 模式下只作兜底
    max_range_m: float = 60.0          # 超远回波直接丢弃（车厢场景 60m 足够）
    # ★ 逐通道的方位角微调，叠加在 config.json 的 processing.yaw_offset_deg 上。
    #   None / 缺省 = 不叠加，完全沿用共用配置（默认行为不变）。
    #
    #   为什么需要它：`yaw_offset_deg` 在四路共用的 config.json 里，但
    #   **四条车道彼此并不平行**。2026-09-14 实测 27 份实车点云，逐格量车厢
    #   左右沿对 y 的斜率（= 车相对 ROI 长轴停斜多少度）：
    #       CH1  -1.50° ~ +0.50°（14 份）  车道与 ROI 对齐
    #       CH4  +1.00° ~ +2.50°（6 份）
    #       CH3  +4.25° ~ +6.00°（6 份）   ← 每一份都斜，散布只有 1.75°
    #   这种一致性说明是**车道朝向**的系统性偏差，不是司机随机停斜。
    #   后果：CH3 的车厢最近只差 **0.01 m** 就要被 ROI 的 x 边界切掉
    #   （172808），而车厢被切会直接少算体积。
    #   逐通道补偿后四路统一回到约 0.44 m 余量。
    #
    #   ▲ 这是「10/10 证明不了」的解码层参数（见 CLAUDE.md）。
    #     改它必须拿现场实车点云验证，不能照抄。
    yaw_offset_deg: Optional[float] = None


@dataclass
class ChannelCfg:
    id: str = "CH1"
    name: str = "通道1"
    enabled: bool = True
    mode: str = "live"                 # live | replay
    group: str = ""                    # 同 group 的通道可聚合成一辆车（预留）
    note: str = ""
    lidar: LidarCfg = field(default_factory=LidarCfg)
    config_path: str = "config/config.json"
    devices_path: str = "config/devices.json"
    # 本通道对应的相机/点位编码（CH1~CH4 -> CAM_001~CAM_004）。
    # 留空 = 沿用 plate 段里的全局值。车牌接口按点位取牌，
    # 所以这个值**必须逐通道不同**，全局一份会让三条车道拿到同一辆车的牌。
    camera_identifier: str = ""
    output_dir: str = "data/loaded"
    baseline_dir: str = "data/in"
    replay_dir: str = ""
    replay_interval: float = 0.5
    replay_loop: bool = True
    # None = 自动（live 存盘 / replay 不存盘，避免污染历史样本）
    save: Optional[bool] = None

    # 解析后的绝对路径（load_channels 填充）
    abs_config_path: str = ""
    abs_devices_path: str = ""
    abs_output_dir: str = ""
    abs_baseline_dir: str = ""
    abs_angle_cache: str = ""
    abs_replay_dir: str = ""


@dataclass
class ServiceCfg:
    host: str = "0.0.0.0"
    port: int = 8080
    data_root: str = "data"
    log_dir: str = "logs"
    log_level: str = "INFO"
    keep_days: int = 90
    daily_cleanup_at: str = ""
    # 看板实时推送频率（Hz）。检测仍按帧走，这里只管**推给浏览器**的节奏。
    # 默认 2Hz：与原版 detect_interval=0.5s 同拍，人眼看得清；
    # 状态跳变（空闲/运动中/已停稳/换会话）不受它限制，一律立刻推。
    # 设 0 = 不节流（每帧都推，就是 2026-09 现场"点云闪得看不清"的那种行为）。
    state_push_hz: float = 2.0
    # ★ 每次测量顺带存几帧**未裁 ROI** 的整帧（0 = 关闭，默认）。
    #
    #   为什么需要：落盘的 .xyz 是「先裁 ROI、再做航向旋转」之后的点云，
    #   ROI 之外的东西根本不在文件里 —— 想验证"车厢有没有被 ROI 切掉"，
    #   拿它只能看"边缘离边界多远"做间接推断，而这在 y 方向不可靠
    #   （车厢远端因掠射角会看起来变窄：2026-09-14 实测主体宽 2.67 m，
    #   到 y=5.5 只剩 2.25 m）。有未裁的整帧才能**直接看到框外面**。
    #
    #   取几帧：在沉降窗口内均匀取（首 / 中 / 末），末帧正是测量用的那一帧，
    #   与同次落盘的 .xyz 一一对应。3 帧还能看出停稳期的帧间抖动。
    #   体积：一帧约 2.9 MB（20 万点 × 14 B）。每车 3 帧 ≈ 8.7 MB，
    #   一上午 50 车约 0.4 GB。
    #   只存**有车那一路** —— 空车道的整帧没有信息量。
    raw_frames: int = 0
    # ★ 车辆**进场过程**的连续整帧（0 = 关闭，默认）。见 docs/41 第 0h 节。
    #
    #   为什么需要：车停到位时，车厢两端离雷达天底 5 m 以上、俯角只剩 16~28 度，
    #   前端最外 1.5 m 覆盖只有 50%（2026-09-20 实测两辆磅单真值车）。而车
    #   **往里开**的时候，车厢前端会经过天底附近（俯角 45 度以上）——
    #   回退 2.0~2.5 m 时前端带预测覆盖就到 96~99%。
    #   沉降窗口那 3 帧补不回来：车停着不动，落点几乎重合（宁A42264Z 三帧
    #   互相关 r=1.000，覆盖只从 85.1% 涨到 87.4%）。要拿新覆盖只能换位置。
    #
    #   ▲ 这里**只负责把数据采下来**，不参与任何判定、不进归档、不改体积。
    #     配准精度能不能做到 0.10 m 以内还没有实测过，做不到就别合并
    #     （合并多帧定车厢盒实测 1 帧 11.5 m -> 3 帧 14.35 m，体积 55 冲到 69）。
    #
    #   ▲ 旁路抓包（cli/capture.py）在 Windows 上与服务**不能共存**：
    #     SO_REUSEADDR 让两个 bind 都成功，但包只落到其中一个 socket
    #     （radar_source.py 第 639 行）。所以只能由服务自己采。
    #
    #   体积：未裁整帧约 2.6 MB，ROI 内约 1.1~1.6 MB。只在**判到停稳那一刻**
    #   落盘一次，写在后台线程上，不占通道线程。
    #   ▲ 它是缓冲上限，不是每车必存的帧数：按位移留帧（enter_step_m）之后，
    #     停着的车只留 1 帧，一般进场 50~60 帧就够（2026-09-27 实测）。
    enter_frames: int = 0
    # 每隔几帧留一帧（1 = 每帧都留）。10 Hz 下 stride=2 约等于每 0.2 秒一帧。
    enter_stride: int = 1
    # ★ 从"车辆点达到多少"开始滚缓冲。0 = 用 detection.vehicle_threshold（原行为）。
    #
    #   ▲▲ **必须比 vehicle_threshold 低得多**，否则采不到进场段。
    #      2026-09-21 实测（43 个会话）：车进 ROI 之后**只走约 1.6 m 就停了**，
    #      而且判稳前最后 3 秒速度几乎为 0（车早就物理停住，在等连续停稳计时）。
    #      逐档实测判稳前能回退的距离：
    #          30 帧(3 s) 0.01 m   60 帧(6 s) 1.27 m   90 帧(9 s) 1.55 m
    #          120 帧(12 s) 1.57 m —— 到顶了，再多也没用
    #      而 §0h 想要的回退量是 2.0~2.5 m，**在车进 ROI 之前**。
    #      用 vehicle_threshold(15000) 当起点更晚，等于整段都采不到。
    #
    #   ▲ 也不能设得太低：空车道本身就有噪声点。2026-09-21 实测 CH4 因为
    #     地面沿 y 上坡 2.33 度，最前端地面抬到判据线之上，空车道恒有约
    #     560 个"车辆点"。所以门槛要在这个之上，2000 是安全值。
    enter_min_points: int = 0
    # ★ 进场帧按**车辆位移**留帧（米；0 = 每帧都留，旧行为）。2026-09-27 实测定。
    #
    #   车辆前沿（车辆点 y 的 99 分位）或中位 y 比上一张留下的帧挪够这么多才留。
    #   以前是每帧都留：全部 61 批 8.14 GB 里 36% 是车已经停住后拍的
    #   （车停下到判稳之间一直在收），其中 1.67 GB 整批车根本没动
    #   （停着的车每 180 秒超时强开一次会话，每次再存约 20 帧）。
    #   这些帧与沉降窗口、归档里的停稳帧重复，对"补车厢前端"没有新信息。
    #   22 辆真实进场车模拟：0.10 m 时每车 107 -> 56 帧；车正常驶入约每帧
    #   挪 8~12 cm，所以在动的帧几乎全留，丢的只是停住和蠕行的帧。
    #   ▲ 位置用 ROI 内车辆点算（与停稳层"只数高于地面的点"同一口径），
    #     只决定留不留，不进任何判定。
    enter_step_m: float = 0.10
    # ★ 进场帧只存 ROI 内的点（False = 存未裁整帧，旧行为）。2026-09-27 实测定。
    #
    #   未裁整帧里 35~58% 的点在 ROI 外（墙、屋顶、邻道），补车厢前端用不上；
    #   车往里开时前端经过的天底（y≈0）与停车位置（前沿 y≈4~6.6）都在 ROI 内。
    #   存的是数据源已经裁好的那一份（与检测输入同一坐标系，**没有**航向旋转），
    #   旁注的 roi / cropped_to_roi 记着裁剪框。
    #   ▲ 与 raw_frames 不同：raw_frames 存未裁整帧是为了看"车厢有没有被 ROI
    #     切掉"，那个用途必须留框外；进场帧没有这个用途。
    enter_crop_roi: bool = True
    # ★ 进场帧接入体积（2026-10-24，docs/92）：判稳时冻结的进场帧（只认 ROI 裁剪的，enter_crop_roi=true）
    #   配准后补停稳帧没测到的格，并把未测格夹进视线上限与堆积角包络（遮挡上限补洞）。
    #   false = 只关进场帧这一半，遮挡上限补洞照开；进场帧照旧只采不用。
    enter_volume: bool = True
    # ★ 离场帧（2026-10-02，0 = 关闭）：本车测完、车往外开时按位移留的帧，只留最先的这么多帧，
    #   车走（回到空闲）时落盘到 <output_dir>/exit/<通道>/。用途与进场帧对称 —— 补停稳时扫不全的
    #   车厢**后端**（9-28 两车停稳 3 帧后端 2 m 覆盖只有 54%），车往前开走时后端经过雷达附近。
    #   ▲ 采集与体积分开关：exit_frames>0 才采；exit_volume 才接入体积（见下）。
    #   ▲ 现场车测完往哪个方向走还没确认：倒出时后端不经天底，配不上就不补、不拦测。
    exit_frames: int = 0
    exit_step_m: float = 0.10       # 同 enter_step_m（0 = 每帧都留）
    exit_crop_roi: bool = True      # 同 enter_crop_roi
    # ★ 离场帧接入体积（对称 enter_volume）：缓冲满/落盘时冻结 ROI 裁剪帧，异步识牌计算前
    #   若已采到则 reverse 后走同一 register_for_volume，只补停稳没测到的格；配不上不补不拦测。
    #   false = 只关体积这一半，exit_frames 照旧只采落盘。默认 true；现场还需 exit_frames>0 才有料。
    exit_volume: bool = True


@dataclass
class PlateCfg:
    """
    车牌（对应 stopdetect.dll 里的 PlateMonitor）。

    ▲ 相机接口路径**无法从二进制还原**（DLL 里没有任何 URL 片段），
    必须按相机厂商手册在 config/channels.json 的 plate 段补
    snap_url / result_url / plate_field。详见 app/plate.py 的模块说明。
    """
    enabled: bool = False
    mode: str = ""                     # "" 自动 | simulate 仿真 | null 关闭 | http
    base_url: str = ""
    device_id: str = "1"
    camera_identifier: str = ""
    # 接口模板：支持 {base_url} {device_id} {camera_identifier} 占位
    snap_url: str = ""
    result_url: str = ""
    method: str = "GET"
    body: str = ""
    headers: Dict[str, str] = field(default_factory=dict)
    plate_field: str = ""              # JSON 取值路径，如 data.plate
    plate_regex: str = ""              # 留空用内置的中国车牌正则
    sample_plates: List[str] = field(default_factory=list)   # 仿真模式出牌序列
    # 计时（原版键名）
    f2_timeout_ms: int = 2000
    f2_retry_delay_1_ms: int = 300
    f2_retry_delay_2_ms: int = 800
    f2_retry_delay_3_ms: int = 1500
    f3_poll_interval_ms: int = 200
    f3_timeout_ms: int = 5000


@dataclass
class StabilityCfg:
    stationary_warn_sec: float = 30.0
    result_hold_sec: float = 8.0
    min_save_points: int = 20000


# ── channels.json 各段认哪些键（2026-09-22 补，见 docs/55 H2）────────────
#
# ★ 为什么非补不可：`load_calib` / `load_devices` 每一层都接了 check_unknown
#   （2026-09-17），唯独 `channels.json` 全程裸奔 —— 而按 CLAUDE.md 的说法，
#   它才是「**现场主要改这个**」的那一份。实测打错一个字母全部静默接受：
#       service.state_push_hz -> state_push_hzz   静默用默认值
#       plate.f2_timeout_ms   -> f2_timeout_mss   静默用默认值
#       lidar.yaw_offset_deg  -> yaw_offset_degg  静默退回共用值
#   最后一条尤其要命：逐通道 yaw 静默失效 = 车厢按错误角度切，
#   而 CLAUDE.md 用整整一节讲反向补偿会把车厢切掉。
#
# ▲ 这几张表就是"这一段认哪些键"的权威，**加字段时必须同步改**，
#   否则新键会被当成打错名字拦下来 —— 这是故意的，与原厂两份配置同一纪律。
#   自检用例：tests/test_config_validation.py::TestChannelsUnknownKeys。
_CHANNELS_SECTIONS = {"service", "channels", "plate", "stability"}
_CHANNELS_SERVICE_KEYS = {
    "host", "port", "data_root", "log_dir", "log_level", "keep_days", "daily_cleanup_at",
    "state_push_hz", "raw_frames", "enter_frames", "enter_stride", "enter_min_points",
    "enter_step_m", "enter_crop_roi", "enter_volume",
    "exit_frames", "exit_step_m", "exit_crop_roi", "exit_volume"}
_CHANNELS_CHANNEL_KEYS = {
    "id", "name", "enabled", "mode", "group", "note", "lidar",
    "config_path", "devices_path", "camera_identifier", "output_dir",
    "baseline_dir", "replay_dir", "replay_interval", "replay_loop", "save"}
# ▲ lidar 段里的 replay_dir 是**真会被读**的（通道级 replay_dir 缺失时的回退），
#   别因为"看着不属于雷达参数"就删掉。
_CHANNELS_LIDAR_KEYS = {
    "ip", "source_ip", "udp_port", "tcp_port", "angle_cache", "bind_ip",
    "range_mode", "z_down", "frame_mode", "frame_ms", "max_range_m",
    "yaw_offset_deg", "replay_dir"}
_CHANNELS_PLATE_KEYS = {
    "enabled", "base_url", "device_id", "camera_identifier", "method", "body",
    "snap_url", "result_url", "plate_field", "plate_regex", "mode",
    "headers", "sample_plates",
    "f2_timeout_ms", "f2_retry_delay_1_ms", "f2_retry_delay_2_ms",
    "f2_retry_delay_3_ms", "f3_poll_interval_ms", "f3_timeout_ms"}
_CHANNELS_STABILITY_KEYS = {"stationary_warn_sec", "result_hold_sec", "min_save_points"}


def load_channels(path: str) -> Tuple[ServiceCfg, List[ChannelCfg], PlateCfg, StabilityCfg]:
    d = _load_json(path)
    check_unknown(path, "", d, _CHANNELS_SECTIONS, comments=True)
    sv = ServiceCfg()
    s = d.get("service", {})
    check_unknown(path, "service", s, _CHANNELS_SERVICE_KEYS, comments=True)
    daily_at = s.get("daily_cleanup_at", "")
    if (not isinstance(daily_at, str) or daily_at and
            (len(daily_at) != 5 or daily_at[2] != ":"
             or any(c not in "0123456789" for c in daily_at[:2] + daily_at[3:])
             or not 0 <= int(daily_at[:2]) <= 23 or not 0 <= int(daily_at[3:]) <= 59)):
        raise ValueError("service.daily_cleanup_at 必须为空或HH:mm本地时间")
    sv.daily_cleanup_at = daily_at
    for k in ("host", "port", "data_root", "log_dir", "log_level", "keep_days",
              "state_push_hz", "raw_frames", "enter_frames", "enter_stride",
              "enter_min_points", "exit_frames"):
        if k in s:
            # ★ 整数字段一律 int() 强转（2026-09-23，低严重度批次）：原来
            #   enter_frames / enter_stride / enter_min_points 原样落进
            #   dataclass，JSON 里写成 30.5 就会以 float 进入切片/计数。
            if k in ("port", "keep_days", "raw_frames", "enter_frames",
                     "enter_stride", "enter_min_points", "exit_frames"):
                setattr(sv, k, int(s[k]))
            elif k == "state_push_hz":
                setattr(sv, k, max(0.0, float(s[k])))
            else:
                setattr(sv, k, s[k])
    # 进场帧留帧位移：负数/非有限值在物理上不成立，启动即报错（0 = 每帧都留）。
    # ▲ 不进 RANGES：那张表的自检只认 config.json / devices.json 的键。
    for key in ("enter_step_m", "exit_step_m"):
        if key in s:
            step = s[key]
            if (isinstance(step, bool) or not isinstance(step, (int, float))
                    or not math.isfinite(step) or step < 0):
                raise ValueError("%s: service.%s 必须是 >= 0 的有限数（0 = 每帧都留），当前 %r"
                                 % (path, key, step))
            setattr(sv, key, float(step))
    # 必须是 JSON 布尔：bool("false") 是 True，写成字符串会静默反义。
    for key in ("enter_crop_roi", "enter_volume", "exit_crop_roi", "exit_volume"):
        if key in s:
            if not isinstance(s[key], bool):
                raise ValueError("%s: service.%s 必须是 true/false，当前 %r" % (path, key, s[key]))
            setattr(sv, key, s[key])
    if sv.exit_frames < 0:
        raise ValueError("%s: service.exit_frames 必须是 >= 0 的整数（0 = 关闭），当前 %r"
                         % (path, sv.exit_frames))

    chans: List[ChannelCfg] = []
    for i, c in enumerate(d.get("channels", [])):
        # 段名带上是第几个通道 —— 4 路配置里只报"channels 段"找起来很费劲
        where = "channels[%d]%s" % (i, ("（%s）" % c["id"]) if isinstance(c, dict) and c.get("id") else "")
        check_unknown(path, where, c, _CHANNELS_CHANNEL_KEYS, comments=True)
        check_unknown(path, where + ".lidar", (c.get("lidar") if isinstance(c, dict) else None),
                      _CHANNELS_LIDAR_KEYS, comments=True)
        ch = ChannelCfg(
            id=c.get("id", f"CH{i+1}"),
            name=c.get("name", f"通道{i+1}"),
            enabled=bool(c.get("enabled", True)),
            mode=c.get("mode", "live"),
            group=c.get("group", "") or "",
            note=c.get("note", ""),
        )
        lid = c.get("lidar", {})
        ch.lidar = LidarCfg(
            ip=lid.get("ip", ch.lidar.ip),
            source_ip=str(lid.get("source_ip", "") or ""),
            udp_port=int(lid.get("udp_port", 2368)),
            tcp_port=int(lid.get("tcp_port", 9347)),
            angle_cache=lid.get("angle_cache", ch.lidar.angle_cache),
            bind_ip=str(lid.get("bind_ip", "") or ""),
            range_mode=str(lid.get("range_mode", ch.lidar.range_mode)),
            z_down=bool(lid.get("z_down", True)),
            frame_mode=str(lid.get("frame_mode", ch.lidar.frame_mode)),
            frame_ms=int(lid.get("frame_ms", ch.lidar.frame_ms)),
            max_range_m=float(lid.get("max_range_m", ch.lidar.max_range_m)),
            yaw_offset_deg=(None if lid.get("yaw_offset_deg") is None
                            else float(lid["yaw_offset_deg"])),
        )
        ch.config_path = c.get("config_path", ch.config_path)
        ch.devices_path = c.get("devices_path", ch.devices_path)
        ch.camera_identifier = str(c.get("camera_identifier", "") or "")
        ch.output_dir = c.get("output_dir", ch.output_dir)
        ch.baseline_dir = c.get("baseline_dir", ch.baseline_dir)
        ch.replay_dir = (c.get("replay_dir", "") or
                         (c.get("lidar", {}) or {}).get("replay_dir", "") or "")
        ch.replay_interval = float(c.get("replay_interval", 0.5))
        ch.replay_loop = bool(c.get("replay_loop", True))
        ch.save = None if c.get("save") is None else bool(c.get("save"))
        chans.append(ch)

    pl = PlateCfg()
    p = d.get("plate", {})
    check_unknown(path, "plate", p, _CHANNELS_PLATE_KEYS, comments=True)
    if "enabled" in p:
        pl.enabled = bool(p["enabled"])
    for k in ("base_url", "device_id", "camera_identifier", "method", "body",
              "snap_url", "result_url", "plate_field", "plate_regex", "mode"):
        if k in p and p[k] is not None:
            setattr(pl, k, str(p[k]))
    if isinstance(p.get("headers"), dict):
        pl.headers = {str(k): str(v) for k, v in p["headers"].items()}
    if isinstance(p.get("sample_plates"), list):
        pl.sample_plates = [str(x) for x in p["sample_plates"]]
    for k in ("f2_timeout_ms", "f2_retry_delay_1_ms", "f2_retry_delay_2_ms",
              "f2_retry_delay_3_ms", "f3_poll_interval_ms", "f3_timeout_ms"):
        if k in p:
            setattr(pl, k, int(p[k]))

    st = StabilityCfg()
    t = d.get("stability", {})
    check_unknown(path, "stability", t, _CHANNELS_STABILITY_KEYS, comments=True)
    for k in ("stationary_warn_sec", "result_hold_sec"):
        if k in t:
            setattr(st, k, float(t[k]))
    if "min_save_points" in t:
        st.min_save_points = int(t["min_save_points"])

    return sv, chans, pl, st


def resolve_channel(ch: ChannelCfg) -> ChannelCfg:
    """把所有相对路径解析成项目内绝对路径。"""
    ch.abs_config_path = root(ch.config_path)
    ch.abs_devices_path = root(ch.devices_path)
    ch.abs_output_dir = root(ch.output_dir)
    ch.abs_baseline_dir = root(ch.baseline_dir)
    ch.abs_angle_cache = root(ch.lidar.angle_cache)
    ch.abs_replay_dir = root(ch.replay_dir) if ch.replay_dir else ""
    return ch


# ───────────────────────────────────────────────────── 汇总加载

@dataclass
class Settings:
    service: ServiceCfg = field(default_factory=ServiceCfg)
    channels: List[ChannelCfg] = field(default_factory=list)
    plate: PlateCfg = field(default_factory=PlateCfg)
    stability: StabilityCfg = field(default_factory=StabilityCfg)
    channels_path: str = ""

    def enabled(self) -> List[ChannelCfg]:
        return [c for c in self.channels if c.enabled]

    def by_id(self, cid: str) -> Optional[ChannelCfg]:
        for c in self.channels:
            if c.id == cid:
                return c
        return None


def check_groups(chans: List[ChannelCfg], path: str = "") -> None:
    """★ `group` 目前**没有实现**，两条通道填成同一组必须启动即报错。

    `group` 的语义是"这几台雷达合扫同一辆车，结论要聚合"。而本版本的
    `service.py` 里根本没有聚合分支（一个 group 字都没有）——
    照着填下去，两台雷达会**各判各的**：同一辆车产出两份互相独立、
    可能互相矛盾的结论，各自落盘，而且不报任何错。

    这正是本项目明令禁止的"配置项静默无效"：使用者的**意图**（合扫）
    与系统的**行为**（各判各的）完全相反，且没有任何迹象。
    所以这里直接拦住 —— 留空即可，需要聚合就得先把那条分支做出来并验证。
    """
    seen: Dict[str, List[str]] = {}
    for c in chans:
        g = (c.group or "").strip()
        if c.enabled and g:
            seen.setdefault(g, []).append(c.id)
    bad = {g: ids for g, ids in seen.items() if len(ids) > 1}
    if bad:
        detail = "；".join(f"group={g!r} 被 {', '.join(ids)} 共用" for g, ids in bad.items())
        raise ValueError(
            f"{path or 'channels.json'}: {detail}。\n"
            f"  `group` 的意思是「这几台雷达合扫同一辆车、结论要聚合」，"
            f"但**本版本没有实现聚合**，填了也只会各判各的 ——\n"
            f"  同一辆车会产出多份独立结论并分别落盘，且不报错。\n"
            f"  现场是 4 个独立检测位，请把这些通道的 group 留空（\"\"）。")


def load_settings(channels_path: str = "") -> Settings:
    p = channels_path or root("config", "channels.json")
    sv, chans, plate, stab = load_channels(p)
    check_groups(chans, p)
    st = Settings(service=sv, channels=[resolve_channel(c) for c in chans],
                  plate=plate, stability=stab, channels_path=p)
    ensure_dir(root(sv.data_root))
    ensure_dir(root(sv.log_dir))
    return st


def dump_effective(cfg: CalibCfg, dev: DevicesCfg) -> str:
    """打印当前生效的关键参数 —— 现场调试第一件事就是核对这里。"""
    lines = [
        f"config.json   : {cfg.path}",
        f"  雷达         : udp={cfg.network.udp_port} tcp={cfg.network.tcp_port} bind={cfg.network.bind_ip}",
        f"  报文         : {cfg.packet.size}B  {cfg.packet.channels}通道  {cfg.packet.angle_bins}bin",
        f"  距离/角度     : scale={cfg.processing.distance_scale} offset_r={cfg.processing.offset_r} "
        f"yaw_offset={cfg.processing.yaw_offset_deg}°",
        f"  ROI          : {cfg.roi}",
        f"  底面拟合      : thr={cfg.floor.distance_threshold} zpct={cfg.floor.z_percentile} "
        f"nz>={cfg.floor.normal_z_threshold} iters={cfg.floor.max_iterations}",
        f"  航向矫正      : pct={cfg.heading.pct} min_corr={cfg.heading.min_correction}",
        f"  停稳         : delta={cfg.detection.stability_delta} 有车阈值={cfg.detection.vehicle_threshold}点 "
        f"回差×{cfg.detection.vehicle_release_ratio} 确认{cfg.detection.stability_release_frames}帧",
        # ★ detect_interval 只对 replay / simulate / synthetic 生效：接真雷达时
        #   节奏由"攒满一圈"决定（frame_mode=rotation）。不标清楚的话，现场会
        #   以为调它能改真雷达的判定频率 —— 又是一个"调了没反应"。
        f"  取帧间隔      : {cfg.detection.detect_interval}s（仅回放/仿真生效；"
        f"接真雷达时按方位角整圈出帧）",
        f"devices.json  : {dev.path}",
        f"  判定门槛      : dist={dev.gate.distance_m} nz>={dev.gate.normal_z_min} "
        f"z_above<={dev.gate.z_above_min_max_m}",
        f"                inliers>={dev.gate.min_inliers} ratio>={dev.gate.min_inlier_ratio} "
        f"spanX>={dev.gate.min_span_x_m} spanY>={dev.gate.min_span_y_m}",
        f"  其它         : 最少点数={dev.min_save_points} 停稳告警={dev.stationary_warn_sec}s "
        f"结果保持={dev.result_hold_sec}s",
    ]
    return "\n".join(lines)
