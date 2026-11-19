"""单车道采集与测量用例；多车道组装和控制台日志由外层负责。"""
from __future__ import annotations
import io
import os
import re
import json
import copy
import tempfile
import time
import threading
import traceback
import uuid
from collections import Counter, deque
from dataclasses import asdict, replace
from typing import Deque, Dict, List, Optional, Tuple
import numpy as np
from ..settings import load_calib, load_devices, ensure_dir, root, ChannelCfg, config_fingerprint
from ..pointcloud import AngleCache, Frame, save_xyz, save_reason
from ..radar_source import build_source, SourceBase, LiveHub
from ..detect import analyze_load, detect_empty, gate_report, reason_detail, detect_loaded, evaluate_empty, check_dims, StabilityDetector, StabilityState, EmptyBaseline, compute_volume_auto, baseline_quality, empty_evidence, EmptyResult
from ..plate import build_plate_reader, safe_dir_name, PlateReaderBase, PlateResult, EMPTY_PLATE, BASELINE_SUFFIX, baseline_file
from ..cloud_observation import FrameSnapshot, ObservationPair, ObservationWindow
from ..control_lock import FairRLock
from ..result_finalizer import CandidateInput, MeasurementRejected, PendingResult, ResultFinalizer, result_bytes
from ..runtime_events import RuntimeEvents
from .events import EventBus
from .channel_state import ChannelState, trigger_digest

_LEGACY_CHANNEL_BASELINE = re.compile(r"^CH\d+$", re.IGNORECASE)
ENTER_THIN_MAX_STEP_M = 0.5


class ChannelWorker(threading.Thread):
    def __init__(self, ch: ChannelCfg, bus: EventBus, plate_cfg,
                 save_override: Optional[bool] = None,
                 plate_mode: str = "", demo: bool = False,
                 hub: Optional[LiveHub] = None,
                 plate_rotate: int = 0,
                 state_push_hz: float = 2.0, raw_frames: int = 0,
                 enter_frames: int = 0, enter_stride: int = 1, enter_min_points: int = 0,
                 enter_step_m: float = 0.10, enter_crop_roi: bool = True, enter_volume: bool = True,
                 exit_frames: int = 0, exit_step_m: float = 0.10, exit_crop_roi: bool = True,
                 exit_volume: bool = True,
                 integration=None, finalizer=None, volume_catalog=None):
        super().__init__(daemon=True, name=f"ch-{ch.id}")
        # 锁序：Director -> 本通道 -> Detector/recent/finalizer；后台不反向回调。
        # 必须覆盖读帧到测量冻结及回执更新，单锁 Detector 不能撤销旧测量许可。
        self._control_lock = FairRLock()
        self.ch = ch
        self.bus = bus
        self.calib = load_calib(ch.abs_config_path)
        self.dev = load_devices(ch.abs_devices_path)
        self.angle = AngleCache(ch.abs_angle_cache).load()
        self.integration = integration
        if volume_catalog is not None and integration is None:
            raise ValueError("体积参数接入必须使用WebApi正式基线")
        self.volume_catalog = volume_catalog
        self._capture_instance_id = uuid.uuid4().hex
        self._deferred = None
        from ..load_radar_contract import effective_configuration_bytes
        import hashlib
        self._effective_configuration = effective_configuration_bytes(self.calib, self.dev, ch, self.angle)
        self._integration_hash = hashlib.sha256(self._effective_configuration).hexdigest()
        from ..enter_validation import coordinate_evidence
        self._motion_configuration = coordinate_evidence(self._effective_configuration)
        self._motion_yaw = (ch.lidar.yaw_offset_deg if ch.lidar.yaw_offset_deg is not None
                            else self.calib.processing.yaw_offset_deg)
        # ★ 本次实际生效的工作模式。演示（demo=True）会覆盖配置里的 live ——
        #   不覆盖的话看板/快照里会一直写着 live，让人误以为在接真雷达。
        self.eff_mode = ("simulate" if demo else (ch.mode or "replay"))
        if integration is not None and hasattr(integration, "configure_channel"):
            integration.configure_channel(ch.id, self._integration_hash, self.eff_mode)
        # ★ 必须在 build_source **之前**赋值 —— 下面要用它决定数据源留不留未裁的帧。
        #   未裁 ROI 的整帧取样（service.raw_frames，0 = 关闭）。见 _grab_raw。
        self.raw_frames = max(0, int(raw_frames))
        # ★ 进场过程的连续整帧（service.enter_frames，0 = 关闭）。见 _grab_enter。
        self.enter_frames = max(0, int(enter_frames))
        self.enter_stride = max(1, int(enter_stride))
        self.enter_step_m = float(enter_step_m)
        self.enter_crop_roi = bool(enter_crop_roi)
        self.enter_volume = bool(enter_volume)
        # 0 = 用 vehicle_threshold（原行为）。见 settings.ServiceCfg.enter_min_points：
        # 实测车进 ROI 后只走约 1.6 m 就停，用 vehicle_threshold 当起点整段都采不到。
        self.enter_min_points = max(0, int(enter_min_points))
        # ★ 离场过程的连续整帧（service.exit_frames，0 = 关闭）。见 _grab_exit。
        self.exit_frames = max(0, int(exit_frames))
        self.exit_step_m = float(exit_step_m)
        self.exit_crop_roi = bool(exit_crop_roi)
        self.exit_volume = bool(exit_volume)
        self.source: SourceBase = build_source(
            ch, self.calib, self.angle,
            mode=("simulate" if demo else None),
            demo_loaded=False,          # 演示一律先从"空车"开始，载货由编排器/界面切换
            hub=hub)                    # 多雷达：共用一个 LiveHub（见 service.LidarService）
        self.stability = StabilityDetector(self.calib.detection.stability_delta,
                                           self.calib.detection.vehicle_threshold,
                                           self.dev.stationary_warn_sec,
                                           ground_z=self.calib.detection.ground_z,
                                           vehicle_min_height_m=self.calib.detection.vehicle_min_height_m,
                                           vehicle_release_ratio=self.calib.detection.vehicle_release_ratio,
                                           release_frames=self.calib.detection.stability_release_frames,
                                           break_ratio=self.calib.detection.stability_break_ratio,
                                           count_ratio=self.calib.detection.stability_count_ratio,
                                           stationary_min_sec=self.calib.detection.stationary_min_sec,
                                           stationary_speed_mps=self.calib.detection.stationary_speed_mps,
                                           stationary_drift_m=self.calib.detection.stationary_drift_m,
                                           stationary_speed_window_sec=self.calib.detection.stationary_speed_window_sec,
                                           stationary_max_gap_sec=self.calib.detection.stationary_max_gap_sec,
                                           stationary_jitter_m=self.calib.detection.stationary_jitter_m,
                                           stationary_shape_speed_mps=self.calib.detection.stationary_shape_speed_mps,
                                           stationary_yaw_speed_mps=self.calib.detection.stationary_yaw_speed_mps)
        # ★ 车牌接口是**按点位取牌**的（CH1~CH4 -> CAM_001~CAM_004），
        #   而 plate 段是全局一份。这里按通道覆盖 camera_identifier ——
        #   不覆盖的话三条车道会拿同一个点位编码去查，取回同一辆车的牌，
        #   而且不报错：落盘目录、空车/载货配对全部串到一起。
        if ch.camera_identifier:
            plate_cfg = replace(plate_cfg, camera_identifier=ch.camera_identifier)
        plate_kind = "null" if integration is not None else plate_mode
        # ★★ 非 live 通道默认**不读牌**。
        #   仿真 / 回放 / 合成通道里没有真车，去调真实相机或后台既没有意义，
        #   又会让**离线运行反过来依赖一个在线服务**：车牌配成 http 之后，
        #   后台一停，每次 read() 都要空等 f3_timeout_ms（默认 5 秒）才失败。
        #   实测后果是整套用例卡死在某一条 —— 表现为"测试挂起"，
        #   而真正的原因（后台没起）在任何日志里都看不出来。
        #   要在仿真里也读牌就显式给：`--plate http` / `--plate simulate`
        #   （`--demo` 本来就等价于 `--plate simulate`，不受这条影响）。
        if not plate_kind and ch.mode != "live":
            plate_kind = "null"
        self.plate: PlateReaderBase = build_plate_reader(
            plate_cfg, mode=plate_kind,
            save_dir=root("logs", "plate_fail"),
            rotate=plate_rotate)
        # ★ 车牌预取状态（两轮审查 M4）：开窗/命令接管时异步起跑，
        #   测量时刻只收结果 —— 见 _plate_prefetch / _plate_collect。
        self._plate_prefetched = False

        # 是否落盘：显式 override > 通道配置 > 按模式推断
        if save_override is not None:
            self.save_enabled = bool(save_override)
        elif ch.save is not None:
            self.save_enabled = bool(ch.save)
        else:
            self.save_enabled = (self.eff_mode == "live")
        # raw_frames 只控制额外诊断取样；开启保存必须保留主测量的全观测。
        # 进场帧要求实际落盘；关闭保存时不为它保留额外原始观测。
        self.source.keep_raw = (self.save_enabled or self.raw_frames > 0
                                or volume_catalog is not None)
        self._finalizer = finalizer
        self._owns_finalizer = finalizer is None
        self._current_result_token = ""
        self._local_passage = None
        self._observation_epoch = 0
        self._current_pair: Optional[ObservationPair] = None
        self._pair_error = ""
        self._settle_pairs: List[Optional[ObservationPair]] = []
        self._measurement_window: Optional[ObservationWindow] = None
        self._volume_observation_window: Optional[ObservationWindow] = None
        self._volume_extra_members: Tuple[int, ...] = ()
        self._measurement_gate = None

        self.st = ChannelState(id=ch.id, name=ch.name, mode=self.eff_mode,
                               source_kind=self.source.kind,
                               backend="cpu")
        self.history: Deque[Dict] = deque(maxlen=500)
        self._runtime_events = RuntimeEvents(ch.id, self.source.kind)
        # ★★ 空车基线**按车牌存**，不按车道（2026-09-23 现场定案）。
        #
        #   业务事实：**空车只会上一次车道**（进场时在某一条道过一次空车扫描），
        #   而**载重之后走哪条道是不一定的**（4 条车道随机）。所以"车道 A 的基线"
        #   与"载重时车在车道 C"根本对不上，必须用车牌把同一辆车的两趟串起来。
        #
        #   改造之前是 `<通道>.baseline.npz` 一条道一份，而文件内容其实是标定那一刻
        #   停在那条道上的**某一辆车**的车厢（`box` 直接进 compute_volume 的积分域）。
        #   即便同一条道也必错：实测长厢 53 辆厢长 10.45~13.90 m 连续铺开、辆辆不同，
        #   车厢后沿 y0 极差 4.00 m，而 dy 在 ±2 m 内变动**体积差 18%**。
        #
        #   现在 `self.baseline` 不在构造时预加载 —— 它是**这一辆车**的基线，
        #   只有识别出车牌之后才知道是哪一份，见 `_resolve_baseline()`。
        self.baseline: Optional[EmptyBaseline] = None
        self._baseline_plate = ""                    # 当前 self.baseline 属于哪个车牌
        self._baseline_dir = ch.abs_baseline_dir
        # 车牌 -> (文件 mtime, 基线)。只缓存**查到的**，见 _resolve_baseline。
        self._baseline_cache: Dict[str, Tuple[float, Optional[EmptyBaseline]]] = {}
        # 旧的车道级基线：**只用来提醒，绝不拿来算体积**（那正是要修掉的错）。
        self._legacy_base_path = os.path.join(ch.abs_baseline_dir, f"{ch.id}.baseline.npz")
        # ★ 千万不要叫 self._stop：ChannelWorker 继承自 threading.Thread，
        #   而 Thread 内部有一个**方法** _stop()，join() 成功回收线程时会调它。
        #   用同名属性覆盖掉它，join() 就会抛 TypeError: 'Event' object is not callable，
        #   导致 LidarService.stop() 在第一条通道上就崩 —— 后面的通道不会被 join、
        #   LiveHub 的 UDP socket 也永远不会被 close()。
        self._stop_evt = threading.Event()
        # ★ 两个计数器要分开，别用同一个：
        #   _detected_this_session  节流用 —— 一个"停稳会话"只跑一次完整检测，
        #                           与是否真的写盘无关。
        #   _saved_this_session     统计用 —— 实际写盘次数。
        #   踩过的坑：一开始只用 _saved_this_session 做节流，结果在
        #   save_enabled=False（回放模式）下它永远不增长，于是每一帧
        #   （0.5s 一次）都跑一遍完整检测，单帧 600+ms，CPU 白烧。
        self._detected_this_session = 0
        self._saved_this_session = 0
        self._session_id = 0
        self._session_t0 = 0.0
        # 沉降窗口：停稳后再等 settle_sec 秒才测量落盘（见 _tick / _merge_settled）
        self.settle_sec = max(0.0, float(self.calib.detection.settle_sec))
        self.settle_accumulate = bool(self.calib.detection.settle_accumulate)
        self.settle_max_frames = max(1, int(self.calib.detection.settle_max_frames))
        self._settle_t0: Optional[float] = None
        self._settle_frames: List[Frame] = []
        # 沉降窗口里除 target 之外的帧的点云，只喂给体积的货面（见 _tick）
        self._vol_extra: List[np.ndarray] = []
        self._warned = False
        # 本会话每条停稳判据各拦了多少帧；只在真正的会话边界清零。
        self._triggers: Counter = Counter()
        # 连续几次会话超时都没等到"车走"。车真的停在那儿也会超时，所以第一次
        # 不下结论；连着两次（≈6 分钟从不空闲）就该让人看一眼了。
        self._timeouts_in_a_row = 0
        self._gen = 0
        # 会话超时保险，见 _tick()
        self.session_timeout = float(getattr(self.dev, "session_timeout_sec", 180.0) or 0.0)
        # CH4 双雷达尺寸采集触发：独立的位置停稳，不改原判稳（见 app/position_standstill.py）
        from ..position_standstill import PositionStandstill
        self._position_still = PositionStandstill(self.calib.detection.stationary_drift_m,
                                                  self.calib.detection.stationary_min_sec)
        self.max_save = max(1, int(self.calib.detection.stable_save_max))
        # 最近一帧（已裁 ROI），供 /api/export 导出 PLY/XYZ 现场排查用。
        # 只保留一帧，约 0.6MB，不值得为它做更复杂的缓存。
        self.last_frame: Optional[Frame] = None
        self._last_frame_monotonic = 0.0
        # 最近几帧，**只**给"标定空车基线"做重复性闸门用（见 capture_baseline）。
        # 保留 4 帧约 2.4MB，不参与判定，也不参与体积。
        # ▲ 它是**跨线程**的：通道线程写、HTTP 线程（看板点"标定基线"）读。
        #   不加锁直接迭代会 `RuntimeError: deque mutated during iteration` ——
        #   实测 2 秒内必现。所以读写都走 _recent_lock。
        self._recent_frames: Deque[Frame] = deque(maxlen=4)
        self._integration_frames: Deque[tuple] = deque(maxlen=3)
        self._recent_lock = threading.Lock()
        # 本通道这一套配置的指纹，落盘时写进 meta（见 _save）。
        # 配置在进程生命周期内不变，所以只算一次。
        self._config_fp = config_fingerprint(
            self.calib, self.dev, getattr(self.ch.lidar, "yaw_offset_deg", None))
        # ★ 看板推送节流（2026-09 现场问题：点云刷新太快看不清）
        #   检测按帧走（10Hz 一路），但**看板没必要每帧都收一次**：
        #     · 每条 state 事件带 2500 个点，4 路 ×10Hz ≈ 3MB/s，
        #       浏览器端每秒重画 40 次，人眼只看到一片乱闪；
        #     · 序列化这些点还要占检测线程的 CPU。
        #   于是按 state_push_hz（默认 2Hz，与原版 detect_interval=0.5s 同拍）推送，
        #   **但状态跳变（空闲/运动中/已停稳/沉降/会话号）一律立刻推** ——
        #   节流的是"看着一样的帧"，不是"事情发生了"。
        self._state_min_dt = (1.0 / float(state_push_hz)) if state_push_hz > 0 else 0.0
        self._state_push_t = 0.0
        self._state_key: Tuple = ()
        self._state_skipped = 0
        self._settle_warn_t = 0.0          # "沉降窗口内又动了"告警的限流时刻
        self._degraded_warn_t = 0.0        # "推迟测量等完整帧"告警的限流时刻
        # ★ 未裁 ROI 的整帧被内存预算挤掉的次数（2026-09-19 加）。
        #   在此之前只写 `st.stability["raw_budget_limited"]`，而那个键
        #   **全项目没有任何地方读** —— 于是"整帧一份都没存下来"
        #   可以整天无人察觉，而现场排查 ROI 有没有切到车厢**只能靠它**。
        self._raw_dropped = 0
        self._raw_warn_t = 0.0
        # ▲▲ **现在这个标志结构性地恒为 False**（2026-09-19 查实：全库无 `= True`）。
        #   它原本表示"等不到完整帧硬测"，但那条路径已经关掉 —— 沉降窗口只在
        #   `stab.stable` 时推进，而 `stable` 要求 `trusted=True`，所以进到判定链的
        #   帧一定是完整帧（见 _tick 里"仅连续完整新帧可到此处"那句）。
        #   **不能删**：`load_radar_contract.build_measurement` 强制要求
        #   `frame_degraded` 是显式 bool（缺了直接抛错），它还进上报载荷的
        #   `frameDegraded`，而 `_same_measurement` 逐键比对 —— 删了会打挂整条上报链。
        #   所以它现在的含义是**不变量断言**，不是测量值。
        self._degraded_measure = False
        # ★ 超时强开的会话如果又落一次盘，那必定是**同一辆车**被重测（见 _tick）。
        #   这两个字段就是为了把这件事写进 .meta.json，让下游能去重。
        self._session_forced = False       # 本会话是被 session_timeout 强开的
        self._last_save_name = ""          # 本车道上一次落盘的文件名（车走了就清空）
        # 最近若干帧的报文数，用来识别"残缺帧"（见 _frame_is_complete）
        self._pkt_hist: Deque[int] = deque(maxlen=self._COMPLETE_HISTORY)
        # 最近 _INCOMPLETE_WINDOW 帧的完整性判定，用来算残缺帧占比。
        # 只统计不参与判定 —— 门槛一个字没动。
        self._complete_hist: Deque[bool] = deque(maxlen=self._INCOMPLETE_WINDOW)
        # 逐帧诊断复用同一窗口，仅采证请求导出，不参与判定或写盘。
        self._frame_integrity_history: Deque[Dict] = deque(maxlen=self._INCOMPLETE_WINDOW)
        # 未裁 ROI 的整帧取样容器（raw_frames 在 build_source 之前就赋好了）
        self._raw_shots: List[Tuple[float, Frame]] = []
        # 进场帧环形缓冲：车在场但还没判稳时一直滚动，判稳那一刻整批落盘
        self._enter_shots: Deque[Frame] = deque(maxlen=self.enter_frames or 1)
        self._enter_positions: Deque = deque(maxlen=self.enter_frames or 1)   # 与 _enter_shots 一一对应
        self._enter_seen = 0
        self._enter_selected = 0
        self._enter_done = False
        self._enter_last_identity = None
        self._enter_last_position = None
        # 按位移抽稀后的有效留帧步长（见 _grab_enter）；车次边界恢复成 enter_step_m。
        self._enter_step_eff = self.enter_step_m
        # 判稳那一刻冻结给体积用的进场帧（ROI 裁剪后、去 ring 的快照，时间顺序）；换车时清。
        self._enter_volume: Tuple[FrameSnapshot, ...] = ()
        # 离场帧：本车测过之后车在动时按位移留（只留最先的 exit_frames 帧），车走（回到空闲）时整批落盘
        self._exit_shots: List[Frame] = []
        self._exit_seen = 0
        self._exit_selected = 0
        self._exit_last_position = None
        self._exit_passage = None
        # 离场资格属于物理车次；180 秒只是重测节流会话，不能把已经测过的车变成没测过。
        self._exit_qualification = None
        self._exit_measurement_id = None
        # 体积用的离场帧快照（ROI 裁剪、去 ring；采集时间顺序）；缓冲满或落盘时冻结。
        self._exit_volume: Tuple[FrameSnapshot, ...] = ()

    # ── 主循环 ──────────────────────────────────────────────
    def run(self) -> None:
        with self._control_lock:
            self.st.online = True
        self.bus.publish("channel_online", {"channel": self.ch.id,
                                            "source": self.source.kind})
        try:
            while not self._stop_evt.is_set():
                try:
                    # ★ 没有新帧就小睡 2ms，别忙等。
                    #   数据源的 read() 在"还没攒够一帧"时会立刻返回 None，而报文是
                    #   一包一包到的（一圈 450 包），于是这个循环会被唤醒上千次/秒，
                    #   每次都白跑一遍 drain / 回绕判定 / info() —— 实测 4 路时
                    #   光这份空转就占掉约一个核。一帧本来就要 100ms 左右，
                    #   2ms 的检查间隔既不耽误出帧，也把空转砍掉一个数量级。
                    #   用 _stop_evt.wait 而不是 time.sleep：stop() 能立刻打断。
                    if not self._tick():
                        self._stop_evt.wait(0.002)
                except Exception as e:                       # noqa: BLE001
                    with self._control_lock:
                        self.st.error = f"{type(e).__name__}: {e}"
                        self.source.stats.errors += 1
                        self._runtime_record("error", "本通道处理异常，已保留诊断日志并继续等待新帧")
                        self.bus.publish("error", {
                            "channel": self.ch.id, "error": self.st.error,
                            "trace": traceback.format_exc()[-1200:]})
                    time.sleep(0.5)
        finally:
            with self._control_lock:
                self.st.online = False
            try:
                self.source.close()
            except Exception:                                # noqa: BLE001
                pass
            self.bus.publish("channel_offline", {"channel": self.ch.id})

    # 最近合格帧的包数中位数；低包数不能自行降低基准。
    # TODO(现场): 人工确认转速/切帧改变后重启重建，不能仅凭低包数区分调速和丢包。
    _COMPLETE_HISTORY = 31
    # 少于基准的这个比例就算"残缺帧"。2026-09-14 实测（一圈 901 包）：
    #   · 正常帧之间只差 ±2 包（0.2%），dev p95 = 0.013、最大 0.051；
    #   · 突发丢包的帧是 886 / 883 / 878 / 873 / 868 / 843 / 801 包
    #     （缺 15~100 个，1.7%~11%），dev 冲到 0.13~0.22。
    # 界划在 1%：把 ±2 包的正常抖动放过，把缺 15 个以上的拦下。
    # 宁可多拦几帧也不能漏 —— 漏一帧就是一次"车停了却说在动"；
    # 而多拦一帧只是让下一个完整帧去和上一个完整帧比，没有代价。
    _COMPLETE_RATIO = 0.99
    # 残缺帧占比的统计窗口。10 Hz 下约 20 秒，够看出持续丢包，又不至于
    # 让一次突发把占比长时间钉住。
    _INCOMPLETE_WINDOW = 200

    # 喂给体积货面的**额外**帧数上限。3 帧（主帧 + 2）是 2026-09-17
    # 在两辆有磅单真值的车上实测验证过的配置（docs/13 §17.2）：
    # 覆盖 78%->92%，上下界差 13.9%->1.6%。再多没验证过，别放开。
    _VOL_EXTRA_MAX = 2

    # ★ 车牌识别在通道锁内的等待上限（秒，两轮审查 M4）。相机 HTTP 的
    #   f2+f3 预算（默认 5 秒）在**锁外**与沉降窗口并行跑；测量时刻只等
    #   这么多。取 2.0：沉降 3 秒 + 2 秒预算 = 5 秒，恰好罩住默认相机预算
    #   —— 慢相机不丢取牌机会，看板快照最多卡 2 秒而不是 5 秒。
    _PLATE_LOCK_BUDGET_S = 2.0

    def _plate_prefetch(self) -> None:
        """车牌识别在**开窗/命令接管那一刻**异步起跑（M4，对齐原版 PlateMonitor）。

        原来 _calculate_detection 里同步 read()：相机预算最多 5 秒**全程
        持通道锁** —— 后台停机时每测一辆车，看板的 snapshot /
        capture_baseline / force_new_session 就卡 5 秒。现在与 3 秒沉降
        窗口并行，测量时刻通常早已就绪。每会话只起跑一次；车辆再动
        作废重开时，同车几秒内的旧识别结果仍然可信，不重启。"""
        if self._plate_prefetched:
            return
        self._plate_prefetched = True
        self.plate.start()

    def _plate_collect(self) -> PlateResult:
        """测量时刻收车牌（M4）。预取未起跑（直调/回放路径）时退回同步读，
        行为与从前逐位一致 —— 那些路径本来就没人等锁。"""
        if not self._plate_prefetched:
            return self.plate.read()
        self._plate_prefetched = False
        budget = max(0.0, min(self._PLATE_LOCK_BUDGET_S,
                              (getattr(self.plate.cfg, "f3_timeout_ms", 0) or 0) / 1000.0))
        self.plate.wait(budget)
        return self.plate.result()

    def _note_raw_dropped(self) -> None:
        """整帧被内存预算挤掉了 —— 记账并（限流地）说一声。

        为什么必须吭声：未裁 ROI 的整帧是**唯一**能直接看到"框外面还有没有车厢"
        的东西（落盘的 .xyz 已经裁过 ROI，拿它只能做不可靠的间接推断）。
        丢了它，"车厢有没有被 ROI 切掉"这个问题当天就查不了，而且补不回来。
        在此之前这里只写一个**没人读**的 `raw_budget_limited`。
        """
        self._raw_dropped += 1
        self.st.stability["raw_budget_limited"] = True
        self.st.raw_dropped = self._raw_dropped
        now = time.monotonic()
        if now - self._raw_warn_t > 30.:
            self._raw_warn_t = now
            self.bus.publish("warn", {
                "channel": self.ch.id,
                "message": ("未裁 ROI 的整帧被内存预算挤掉（累计 %d 帧）——"
                            "本次无法核对车厢是否被 ROI 切掉；"
                            "多为上一次测量尚未提交完就来了下一次" % self._raw_dropped)})

    def _grab_enter(self, stab, complete: bool) -> None:
        """★ 车还在进场时按位移留帧；仅用于后续离线前端补点研究。

        为什么要在停稳之前采：车停到位时车厢两端离天底 5 m 以上、俯角只剩
        16~28 度，前端最外 1.5 m 覆盖只有 50%（2026-09-20 两辆磅单真值车实测）。
        而车往里开时前端会经过天底附近，回退 2.0~2.5 m 就能补到 96~99%。
        沉降窗口那 3 帧补不回来 —— 车不动，落点几乎重合。见 docs/41 第 0h 节。

        ▲ **只采不判**：这些帧不进判定链、不进停稳、不进归档、不改体积。
          配准精度能不能做到 0.10 m 以内还没实测过，做不到就不该合并。
        ▲ 残缺帧跳过，理由与 `_grab_raw` 相同（丢包造成的洞会被读成车厢边界）。
        """
        if (self.enter_frames <= 0 or not self.save_enabled or self._session_forced
                or self._enter_done or not complete):
            return
        floor = self.enter_min_points or self.stability.vehicle_threshold
        if stab.n_vehicle_points < floor:
            return
        pair = self._current_pair
        if pair is None:
            return
        cropped = pair.cropped.frame()
        if not cropped.n:
            return
        self._enter_last_identity = (cropped.gen, cropped.seq, cropped.ts)
        self._enter_seen += 1
        if self._enter_seen % self.enter_stride:
            return
        if self.enter_step_m > 0:
            cut = self.calib.detection.ground_z + self.calib.detection.vehicle_min_height_m
            points = cropped.xyz
            vehicle = points[np.isfinite(points).all(axis=1) & (points[:, 2] > cut)]
            if not len(vehicle):
                return
            position = (float(np.median(vehicle[:, 1])), float(np.percentile(vehicle[:, 1], 99)))
            previous = self._enter_last_position
            if previous is not None and max(abs(a - b) for a, b in zip(position, previous)) < self._enter_step_eff:
                return
            self._enter_last_position = position
        source = pair.cropped if self.enter_crop_roi else pair.raw
        shot = source.frame()
        if shot.n:
            # ★ 缓冲满了先按位移抽稀，不直接丢最早的帧（2026-10-24）：最早的帧是离停车点最远、
            #   正好看清车厢前端的那几帧，环形缓冲只留最后 N 帧时它们最先被覆盖（docs/91 §8）。
            #   从最早一帧起隔一帧留一帧（最早那帧总保留），有效步长翻倍。
            #   抽完后相邻留帧（含本帧）的**实测位移**都不超过 ENTER_THIN_MAX_STEP_M 才抽，否则退回丢最早帧 ——
            #   相邻帧链式配准要求两帧位移不超过 0.6 m（detect.ENTER_STEP_MAX_M）。按实测位移判，
            #   不按名义步长：留帧只保证"至少挪了一步"，两步加起来可能超过两倍步长。
            if len(self._enter_shots) >= self.enter_frames >= 2 and self.enter_step_m > 0:
                kept_positions = list(self._enter_positions)[::2] + [position]
                if all(max(abs(a - b) for a, b in zip(p, q)) <= ENTER_THIN_MAX_STEP_M
                       for p, q in zip(kept_positions, kept_positions[1:])):
                    kept = list(self._enter_shots)[::2]
                    self._enter_shots.clear()
                    self._enter_shots.extend(kept)
                    kept_positions.pop()
                    self._enter_positions.clear()
                    self._enter_positions.extend(kept_positions)
                    self._enter_step_eff *= 2
            self._enter_shots.append(shot)
            self._enter_positions.append(position if self.enter_step_m > 0 else None)
            self._enter_selected += 1

    def _flush_enter(self) -> None:
        """判到停稳那一刻，把缓冲里的进场帧整批落盘。

        ▲ **走后台线程**：一车最多几十帧、上百 MB，在通道线程里写会拖住出帧，
          而沉降窗口只有 3 秒。帧是不可变快照，交给别的线程是安全的。
        ▲ **不经过归档**：直接写 .xyz + 旁注 json，与测量链完全解耦 ——
          归档链路出问题时它照样留得下数据。
        """
        if self.enter_frames <= 0 or not self.save_enabled or self._enter_done or self._session_forced:
            return
        if len(self._enter_shots) < 2:
            self._enter_shots.clear()
            self._enter_positions.clear()
            self._enter_done = True
            return
        self._enter_done = True
        shots = list(self._enter_shots)
        self._enter_shots.clear()
        self._enter_positions.clear()
        # 体积只接 ROI 裁剪后的帧（与停稳货面同一坐标域）；残缺帧在 _grab_enter 已拦。
        if self.enter_volume and self.enter_crop_roi:
            from ..enter_volume import ENTER_MAX_FRAMES, volume_snapshot
            try:
                self._enter_volume = tuple(volume_snapshot(f) for f in shots[-ENTER_MAX_FRAMES:])
            except ValueError:
                self._enter_volume = ()
        selected_total = self._enter_selected
        stamp = time.strftime("%Y%m%d_%H%M%S")
        out = os.path.join(self.ch.abs_output_dir, "enter", self.ch.id)
        passage_id = (self.integration.current_passage(self.ch.id)
                      if self.integration is not None else self._local_passage)
        note = dict(channel=self.ch.id, frames=len(shots), stride=self.enter_stride,
                    metadata_schema="jt128-motion-frame-v2", motion_kind="enter",
                    step_m=self.enter_step_m, effective_step_m=self._enter_step_eff,
                    cropped_to_roi=self.enter_crop_roi,
                    note="车辆进场过程按位移留帧（只采不判，见 docs/41 第 0h 节）",
                    source_mode=self.eff_mode, passage_id=passage_id,
                    session_id=self._session_id, capture_instance_id=self._capture_instance_id,
                    complete_gate_passed=True,
                    eligible_frames_seen=self._enter_seen,
                    selected_frames_total=selected_total,
                    buffer_truncated=selected_total > len(shots),
                    max_seq_gap=max((b.seq - a.seq for a, b in zip(shots, shots[1:])), default=0),
                    max_time_gap_s=max((b.ts - a.ts for a, b in zip(shots, shots[1:])), default=0.),
                    roi=dict(zip("xyz", zip(*self.calib.roi))) if self.calib.roi else None,
                    ground_z=self.calib.detection.ground_z,
                    yaw_offset_deg=self.calib.processing.yaw_offset_deg,
                    channel_yaw_offset_deg=getattr(self.ch.lidar, "yaw_offset_deg", None),
                    effective_yaw_offset_deg=self._motion_yaw,
                    range_mode=getattr(self.ch.lidar, "range_mode", ""))
        note.update(self._motion_configuration)

        def write():
            try:
                os.makedirs(out, exist_ok=True)
                n = len(shots)
                for i, fr in enumerate(shots, 1):
                    stem = os.path.join(out, "%s_%dof%d" % (stamp, i, n))
                    save_xyz(stem + ".xyz", fr)
                    from ..enter_validation import file_sha256
                    with open(stem + ".xyz.meta.json", "w", encoding="utf-8") as fh:
                        json.dump(dict(note, index=i, points=fr.n, seq=fr.seq, ts=fr.ts,
                                       gen=fr.gen, packets=fr.packets,
                                       file_sha256=file_sha256(stem + ".xyz"),
                                       dropped_packets=fr.dropped_packets),
                                  fh, ensure_ascii=False, indent=1)
                self.bus.publish("warn", dict(channel=self.ch.id,
                    message="进场帧已存 %d 帧 -> %s" % (n, out)))
            except Exception as exc:                        # noqa: BLE001
                self.bus.publish("error", dict(channel=self.ch.id,
                    message="进场帧落盘失败（不影响测量）：%s" % type(exc).__name__))

        threading.Thread(target=write, daemon=True, name="enter-%s" % self.ch.id).start()

    def _grab_exit(self, stab, complete: bool) -> None:
        """★ 本车测完、车往外开时按位移留帧（2026-10-02）；exit_volume 时接入体积补后端。

        停稳时车厢后端也扫不全（9-28 两车停稳 3 帧后端 2 m 覆盖只有 54%），车往前开走时
        后端会经过雷达附近。与进场帧对称：按车辆点纵向中位/后沿位移留帧，残缺帧跳过。
        ▲ 只留**最先**的 exit_frames 帧（不是环形缓冲）：后端经过天底是在离场的前几米，
          再往后车已出 ROI，留最后几帧没用。
        ▲ 现场车测完是往前开走还是倒出去不确定（空车只能倒出，见 CLAUDE.md），倒出时后端
          不经过天底 —— 配不上就不补、不拦测。TODO(现场)：确认载货车测完的离场方向。
        ▲ 不进判定链、不进停稳。
        """
        if (self.exit_frames <= 0 or not self.save_enabled or not complete
                or len(self._exit_shots) >= self.exit_frames):
            return
        if self._exit_qualification != self._exit_scope() or self._exit_qualification is None:
            # 后台车次已变或手动重开/换场景已撤销资格，不能借上一车的测量采这一车。
            self._exit_reset()
            return
        floor = self.enter_min_points or self.stability.vehicle_threshold
        if stab.n_vehicle_points < floor:
            return
        pair = self._current_pair
        if pair is None:
            return
        cropped = pair.cropped.frame()
        if not cropped.n:
            return
        self._exit_seen += 1
        if self.exit_step_m > 0:
            cut = self.calib.detection.ground_z + self.calib.detection.vehicle_min_height_m
            points = cropped.xyz
            vehicle = points[np.isfinite(points).all(axis=1) & (points[:, 2] > cut)]
            if not len(vehicle):
                return
            position = (float(np.median(vehicle[:, 1])), float(np.percentile(vehicle[:, 1], 1)))
            previous = self._exit_last_position
            if previous is not None and max(abs(a - b) for a, b in zip(position, previous)) < self.exit_step_m:
                return
            self._exit_last_position = position
        shot = (pair.cropped if self.exit_crop_roi else pair.raw).frame()
        if shot.n:
            if not self._exit_shots:
                self._exit_passage = (self.integration.current_passage(self.ch.id)
                                      if self.integration is not None else self._local_passage)
            self._exit_shots.append(shot)
            self._exit_selected += 1
            if len(self._exit_shots) >= self.exit_frames:
                self._freeze_exit_volume(self._exit_shots)

    def _freeze_exit_volume(self, shots) -> None:
        """缓冲满或落盘时冻结体积用离场帧；已交给 deferred 的副本不因本地 reset 丢失。"""
        if not (self.exit_volume and self.exit_crop_roi and len(shots) >= 2):
            return
        from ..enter_volume import ENTER_MAX_FRAMES, volume_snapshot
        try:
            self._exit_volume = tuple(volume_snapshot(f) for f in shots[:ENTER_MAX_FRAMES])
        except ValueError:
            self._exit_volume = ()
            return
        deferred = getattr(self, "_deferred", None)
        mid = self._exit_measurement_id
        if deferred is not None and mid and self._exit_volume:
            deferred.offer_exit(mid, self._exit_volume)

    def _flush_exit(self) -> None:
        """车走了（回到空闲）那一刻，把离场帧整批落盘到 <output_dir>/exit/<通道>/；后台线程写。"""
        shots, self._exit_shots = self._exit_shots, []
        if self.exit_frames <= 0 or not self.save_enabled or len(shots) < 2:
            self._exit_reset()
            return
        # 缓冲未满也尽量冻给异步体积（识牌常与离场重叠）；满缓冲时 _grab_exit 已冻过。
        if not self._exit_volume:
            self._freeze_exit_volume(shots)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        out = os.path.join(self.ch.abs_output_dir, "exit", self.ch.id)
        note = dict(channel=self.ch.id, frames=len(shots), step_m=self.exit_step_m,
                    metadata_schema="jt128-motion-frame-v2", motion_kind="exit",
                    cropped_to_roi=self.exit_crop_roi,
                    note="本车测完后车辆离场过程按位移留帧（可接入体积，见 service.exit_volume）",
                    source_mode=self.eff_mode, passage_id=self._exit_passage,
                    session_id=self._session_id, capture_instance_id=self._capture_instance_id,
                    measurement_id=self._exit_measurement_id, complete_gate_passed=True,
                    eligible_frames_seen=self._exit_seen, selected_frames_total=self._exit_selected,
                    buffer_full=len(shots) >= self.exit_frames,
                    roi=dict(zip("xyz", zip(*self.calib.roi))) if self.calib.roi else None,
                    ground_z=self.calib.detection.ground_z,
                    yaw_offset_deg=self.calib.processing.yaw_offset_deg,
                    channel_yaw_offset_deg=getattr(self.ch.lidar, "yaw_offset_deg", None),
                    effective_yaw_offset_deg=self._motion_yaw,
                    range_mode=getattr(self.ch.lidar, "range_mode", ""))
        note.update(self._motion_configuration)
        self._exit_reset()

        def write():
            try:
                os.makedirs(out, exist_ok=True)
                n = len(shots)
                for i, fr in enumerate(shots, 1):
                    stem = os.path.join(out, "%s_%dof%d" % (stamp, i, n))
                    save_xyz(stem + ".xyz", fr)
                    from ..enter_validation import file_sha256
                    with open(stem + ".xyz.meta.json", "w", encoding="utf-8") as fh:
                        json.dump(dict(note, index=i, points=fr.n, seq=fr.seq, ts=fr.ts,
                                       gen=fr.gen, packets=fr.packets,
                                       file_sha256=file_sha256(stem + ".xyz"),
                                       dropped_packets=fr.dropped_packets),
                                  fh, ensure_ascii=False, indent=1)
                self.bus.publish("warn", dict(channel=self.ch.id,
                    message="离场帧已存 %d 帧 -> %s" % (n, out)))
            except Exception as exc:                        # noqa: BLE001
                self.bus.publish("error", dict(channel=self.ch.id,
                    message="离场帧落盘失败（不影响测量）：%s" % type(exc).__name__))

        threading.Thread(target=write, daemon=True, name="exit-%s" % self.ch.id).start()

    def _exit_reset(self) -> None:
        """空场、换车或人工撤销时清；自动超时重测必须保留本车资格与已采帧。"""
        self._exit_shots = []
        self._exit_seen = 0
        self._exit_selected = 0
        self._exit_last_position = None
        self._exit_passage = None
        self._exit_qualification = None
        self._exit_measurement_id = None
        self._exit_volume = ()

    def _exit_scope(self):
        passage = (self.integration.current_passage(self.ch.id)
                   if self.integration is not None else self._local_passage)
        return (self._capture_instance_id, self.ch.id, self._gen, passage) if passage else None

    def _mark_exit_eligible(self, measurement_id=None, passage_id=None) -> None:
        """仅成功生成本车测量后授予资格，业务拒绝/取消不会走到这里。"""
        scope = self._exit_scope()
        if scope is None or (passage_id is not None and scope[-1] != passage_id):
            return
        if self._exit_qualification != scope:
            self._exit_reset()
            self._exit_qualification = scope
            self._exit_measurement_id = measurement_id

    def _refresh_deferred_exit_qualification(self) -> None:
        """原留样已落盘才允许采离场帧；迟到通知不能授予下一辆车。"""
        deferred = getattr(self, "_deferred", None)
        if deferred is None or not self._current_result_token:
            return
        identity = deferred.durable_capture(self.ch.id)
        if (identity is not None and identity.channel == self.ch.id and identity.passage_id
                and identity.origin_token == self._current_result_token):
            self._mark_exit_eligible(identity.measurement_id, identity.passage_id)

    def _grab_raw(self, waited: float, complete: bool = True) -> None:
        """在沉降窗口内均匀取 raw_frames 帧**未裁 ROI** 的整帧。

        取样时刻：settle_sec 上均分。raw_frames=3、settle_sec=3.0 时
        就是 0s / 1.5s / 3.0s —— **末帧正是测量用的那一帧**，
        与同次落盘的 .xyz 一一对应；首帧和中帧还能看出停稳期的帧间抖动。

        settle_sec=0（停稳即测）时只取一帧。

        ★ 残缺帧一律跳过（complete=False），**宁可少存一帧也不存带洞的**。
          突发丢包会让一圈缺掉一整块连续扇形（2026-09-14 实测最差连续缺 31 包
          ≈ 12.8°）。这些整帧的用途正是"看车厢边缘之外是不是空的"来判断
          有没有被 ROI 切掉 —— 一个丢包造成的洞正好落在车厢边缘，
          就会被读成"车厢到这里就结束了"，得出**完全相反**的结论。
          跳过时不推进取样计划：下一个完整帧补上，只是时刻稍晚。
        """
        if self.raw_frames <= 0 or len(self._raw_shots) >= self.raw_frames:
            return
        if not complete:
            return
        n = self.raw_frames
        if n > 1 and self.settle_sec > 0:
            target = self.settle_sec * len(self._raw_shots) / (n - 1)
        else:
            target = 0.0
        if waited + 1e-6 < target:
            return
        raw = self._current_pair.raw.frame() if self._current_pair is not None else None
        if raw is not None and raw.n:
            self._raw_shots.append((waited, raw))

    def _pair_observation(self, frame: Frame, complete: bool, received_monotonic=None) -> None:
        """一次 read 对应一次 take_raw，主归档和诊断共享同一不可变观测。"""
        raw = self.source.take_raw()
        self._current_pair = None
        self._pair_error = ""
        if not self.source.keep_raw:
            return
        try:
            self._current_pair = ObservationPair.freeze(
                channel=self.ch.id, epoch=self._observation_epoch,
                received_monotonic=(time.monotonic() if received_monotonic is None else received_monotonic),
                complete=complete,
                raw=raw, cropped=frame, roi=self.source.roi)
        except ValueError as error:
            self._pair_error = str(error)

    def _save_raw(self, base_name: str) -> int:
        """把取到的整帧写进 <output_dir>/raw/。

        单独放一个子目录，而不是和判定用的 .xyz 混在一起 —— `cli/verify.py`
        与 tests 都是递归扫「.xyz + 同名 .reason.txt」的组合，整帧没有
        reason.txt 本来就会被跳过，但分开放更不容易出事。
        旁边写一份 .json 记下当时的 ROI / yaw / ground_z，离线复算不用猜配置。
        """
        if not self._raw_shots:
            return 0
        # ★ 按车道分目录，文件名再带一次车道号。
        #   分目录是为了**现场翻看方便** —— 打开 CH3 就只看 3 号道；
        #   文件名里再写一次是因为文件常被单独拷出来给别人看，
        #   离开目录之后还得认得出是哪条道。
        d = os.path.join(self.ch.abs_output_dir, "raw", self.ch.id)
        ensure_dir(d)
        roi = self.calib.roi
        try:
            lo, hi = roi[0], roi[1]
            roi_out = {"x": [float(lo[0]), float(hi[0])],
                       "y": [float(lo[1]), float(hi[1])],
                       "z": [float(lo[2]), float(hi[2])]}
        except Exception:
            roi_out = None
        n = 0
        for i, (waited, f) in enumerate(self._raw_shots, 1):
            stem = os.path.join(d, "%s_%s_%dof%d" % (self.ch.id, base_name,
                                                     i, len(self._raw_shots)))
            save_xyz(stem + ".xyz", f)
            # 旁注必须叫 <点云名>.meta.json —— storage.remove_sample 删的是
            # `xyz + XYZ_SUFFIXES`，叫别的名字会在清理时留下孤儿 json。
            json.dump({
                "channel": self.ch.id, "points": int(f.n),
                "settle_elapsed_sec": round(waited, 2),
                "note": "未裁 ROI 的整帧 —— 用来直接验证车厢有没有被 ROI 切掉",
                "roi": roi_out,
                # 三个都记：共用值 / 逐通道覆盖 / **实际生效**的那个。
                # 明天离线复算时直接读 effective，不用再推导。
                "yaw_offset_deg": float(self.calib.processing.yaw_offset_deg),
                "channel_yaw_offset_deg": getattr(self.ch.lidar, "yaw_offset_deg", None),
                "effective_yaw_offset_deg": float(
                    getattr(self.ch.lidar, "yaw_offset_deg", None)
                    if getattr(self.ch.lidar, "yaw_offset_deg", None) is not None
                    else self.calib.processing.yaw_offset_deg),
                "ground_z": float(self.calib.detection.ground_z),
                "range_mode": getattr(self.ch.lidar, "range_mode", ""),
                "paired_xyz": base_name + ".xyz",
            }, io.open(stem + ".xyz.meta.json", "w", encoding="utf-8"),
                ensure_ascii=False, indent=1)
            n += 1
        self._raw_shots = []
        return n

    def _frame_is_complete(self, frame) -> bool:
        """这一帧是不是完整的一圈。

        突发丢包会让一圈缺掉一整块连续的扇形（2026-09-14 实测最差连续缺
        31 个报文 ≈ 12.8°），而 dev 是**整帧质心**的位移 —— 缺一块扇形就让
        dev 冲到 0.13~0.22 m，车一动没动也会被判成"又动了"，沉降窗口反复作废。
        所以残缺帧不参与停稳判定（见 StabilityDetector.detect 的 trusted 参数）。

        `packets=0` 表示数据源没给这个信息（回放 / 仿真 / 合成源）—— 一律当完整，
        不能因为"不知道"就把所有帧都排除掉。
        """
        # 解码器已经确认的损失不能参与启动学习，也不能被低包数中位数洗白。
        if getattr(frame, "dropped_packets", 0) > 0:
            return False
        n = int(getattr(frame, "packets", 0) or 0)
        if n <= 0:
            return True
        if len(self._pkt_hist) < 5:
            self._pkt_hist.append(n)
            return True
        ref = sorted(self._pkt_hist)[len(self._pkt_hist) // 2]      # 中位数
        complete = n >= ref * self._COMPLETE_RATIO
        # 残帧不能把自己的低包数逐步洗成正常基准。
        if complete:
            self._pkt_hist.append(n)
        return complete

    def _tick(self) -> bool:
        """跑一轮。返回**这一轮有没有拿到新帧** —— run() 据此决定要不要小睡。"""
        with self._control_lock:
            if self._stop_evt.is_set():
                return False
            try:
                return self._tick_observation()
            finally:
                # 先更新本帧所属车辆/活动代次，再判断迟到结果是否仍属当前车。
                self._drain_finalizer()

    def _tick_observation(self) -> bool:
        frame = self.source.read()
        if frame is None:
            self._current_pair = None
            if self.stability.expire():
                self._discard_observation_window()
                self.st.stability.update(stable=False, degraded=True, fresh=False,
                                         state=self.stability.state,
                                         state_cn=StabilityState.CN[self.stability.state],
                                         stationary_sec=0.0, speed_mps=None, drift_m=0.0,
                                         shape_speed_mps=None, yaw_speed_mps=None,
                                         motion_observable=False, motion_pending=False,
                                         instant_speed_mps=0., jitter_confirmed=False, reason="新帧超时")
                if self.integration is not None:
                    self.integration.observe(self.ch.id, None, False)
                if self.last_frame is not None:
                    try:
                        self._runtime_publish(self._runtime_events.gap())
                    except Exception:                        # noqa: BLE001
                        pass                                # 过程日志失败不能阻断采集
                    self._publish_state(self.last_frame)
            self._refresh_stats()
            return False
        self.last_frame = frame
        self._last_frame_monotonic = time.monotonic()

        # ★ 数据源场景代次变了 → 这一帧属于新场景，此刻才重置停稳检测器。
        #   放在"拿到帧之后"是关键：切换瞬间可能还有一帧旧场景在路上，
        #   若在 set_loaded 时就重置，旧场景的质心会被当成新参考帧，
        #   紧接着新场景的质心差 ~0.7m → 偏差恒大于阈值 → 永远"运动中"。
        gen = int(getattr(frame, "gen", 0))
        if gen != self._gen:
            self._gen = gen
            # 场景代次变了 = 这辆车必然不是上一辆 —— 连 duplicate_of 的
            # 标记一起清（M6：超时强开残留 + 场景切换 = 新车被错标重复）。
            self.force_new_session(scene_changed=True)

        history_count = len(self._pkt_hist)
        reference_before = (sorted(self._pkt_hist)[history_count // 2]
                            if history_count else None)
        complete = self._frame_is_complete(frame)
        reference_after = (sorted(self._pkt_hist)[len(self._pkt_hist) // 2]
                           if self._pkt_hist else None)
        # 只解释已完成的判定，完整性函数和传给停稳检测器的布尔值不变。
        comparison_applied = (frame.dropped_packets <= 0 and frame.packets > 0
                              and history_count >= 5)
        reason = ('decode_dropped' if frame.dropped_packets > 0 else
                  'packet_count_unavailable' if frame.packets <= 0 else
                  'learning' if history_count < 5 else
                  'packet_count_sufficient' if complete else 'packet_count_below_reference')
        queue = getattr(self.source, 'queue', None)
        self._frame_integrity_history.append({
            'seq': frame.seq, 'frame_ts': frame.ts, 'decision_at': time.time(),
            'received_packets': frame.packets + frame.dropped_packets,
            'valid_packets': frame.packets, 'decode_dropped': frame.dropped_packets,
            'reference_before': reference_before, 'reference_after': reference_after,
            'history_count_before': history_count, 'comparison_applied': comparison_applied,
            'required_packets': reference_before * self._COMPLETE_RATIO if comparison_applied else None,
            'complete': complete, 'reason': reason,
            'queue_dropped_total': queue.dropped if queue is not None else None,
            'buffer_trimmed_total': getattr(self.source, '_trimmed', None),
            'timeout_frames_total': getattr(self.source, '_by_timeout', None),
        })
        self._complete_hist.append(bool(complete))
        if not complete:
            self.st.incomplete_frames += 1
        self.st.incomplete_ratio = round(
            1.0 - sum(self._complete_hist) / float(len(self._complete_hist)), 4)
        stab = self.stability.detect(frame, trusted=complete)
        self._triggers.update(stab.triggers)
        decision_gate = self._gate_decision(frame, stab, complete)
        self.st.stability = stab.to_dict()
        if stab.motion_pending:
            # 当前帧尚未确认，不采样、不测量。保留此前完整观测的窗口，
            # 后续确认往返或低净速度才可续用；真实运动/残帧/断流仍完整作废。
            with self._recent_lock:
                self._recent_frames.clear()
            self._integration_frames.clear()
        elif not stab.stable:
            if self._settle_t0 is not None and time.monotonic() - self._settle_warn_t > 5.0:
                self._settle_warn_t = time.monotonic()
                self.bus.publish("warn", {
                    "channel": self.ch.id,
                    "message": "沉降窗口已作废，重新等待连续停稳: " + stab.reason})
            self._discard_observation_window()
        else:
            with self._recent_lock:
                self._recent_frames.append(frame)
        self._pair_observation(frame, complete, self._last_frame_monotonic)
        self.st.frames = self.source.stats.frames
        self.st.fps = round(self.source.stats.fps, 2)
        self.st.last_diag = {"n_points": frame.n}
        if self.integration is not None:
            threshold = self.calib.detection.vehicle_threshold
            absent = (stab.state == StabilityState.IDLE and
                      stab.n_vehicle_points < threshold * self.calib.detection.vehicle_release_ratio)
            occupied = False if absent else (True if stab.n_vehicle_points >= threshold else None)
            self.integration.observe(self.ch.id, occupied, complete and stab.fresh and not stab.degraded)
            passage = self.integration.current_passage(self.ch.id)
            self._report_position_standstill(frame, stab, complete, occupied, passage)
            self._refresh_deferred_exit_qualification()
            if complete and stab.fresh and stab.stable:
                self._integration_frames.append((passage, frame, True, self._current_pair, decision_gate))
            command = (self.integration.claim_command_fast(self.ch.id)
                       if self._finalizer is None or self._finalizer.can_accept(
                           self.ch.id, self._finalize_estimate()) else None)
            if command is not None:
                try:
                    if not complete or not stab.stable or self.eff_mode != "live":
                        raise ValueError("命令只可用于已停稳的完整实车新帧")
                    recent = [item[1] for item in self._integration_frames
                              if item[0] == passage and item[2]]
                    if len(recent) != 3:
                        raise ValueError("人工命令需要当前车次连续三帧完整且停稳")
                    if command["kind"] == "baseline-capture":
                        quality = baseline_quality([x.xyz for x in recent])
                        if quality["n"] < 3 or not quality["ok"]:
                            raise ValueError("空车候选需要至少三帧通过静止场景一致性检查")
                    self._degraded_measure = False
                    # ★ 人工命令接管测量的这一刻，必须清掉自动沉降窗口的残留
                    #   （2026-09-23，两轮审查 M5）。原来 finally 只清 5 个装配
                    #   字段，_settle_t0/_settle_frames/_settle_pairs 一直挂着：
                    #   ① 余下会话 891 行的 _grab_enter 因 _settle_t0 非空被
                    #      跳过 —— 进场帧从此采不到；
                    #   ② _run_detection 里的 _window_evidence/_empty_evidence
                    #      会读到**上一辆车**自动窗口的旧帧，交叉污染。
                    # 放这里（闸门全过、确认要测）而不是 try 外：命令被拒
                    # 时自动窗口是正常状态机的一部分，不许动。
                    self._settle_clear()
                    # 人工路径没有沉降窗口可并行 —— 接管那一刻就异步起跑
                    # 车牌，测量时收结果（M4，同自动路径）。
                    self._plate_prefetch()
                    # 仅用本次车次最新三帧，绝不混入旧沉降窗口的货面。
                    extra = [x.xyz for x in recent[:-1]]
                    self._vol_extra = extra[-self._VOL_EXTRA_MAX:] if self._VOL_EXTRA_MAX else []
                    pairs = [item[3] for item in self._integration_frames if item[0] == passage and item[2]]
                    if self.save_enabled and any(pair is None for pair in pairs):
                        raise ValueError("人工命令缺少当前车次连续三帧原始观测配对")
                    self._measurement_window = ObservationWindow((pairs[-1],)) if pairs[-1] is not None else None
                    support = pairs[-(self._VOL_EXTRA_MAX + 1):]
                    self._volume_observation_window = (ObservationWindow(tuple(support))
                                                       if all(pair is not None for pair in support) else None)
                    self._volume_extra_members = tuple(range(len(support) - 1))
                    self._measurement_gate = dict(decision=decision_gate,
                        manual_three_frames=[item[4] for item in self._integration_frames if item[0] == passage and item[2]],
                        settle=dict(kind="manual", applicable=False,
                                    command={key: command[key] for key in
                                             ("requestId", "channelId", "passageId", "kind", "confirmedEmpty")},
                                    baseline_quality=quality if command["kind"] == "baseline-capture" else None))
                    self._run_detection(frame, stab, command=command)
                except Exception as exc:
                    # 后台只获得固定诊断，异常原文可能包含文件路径。
                    self.integration.reject_command_fast(command["requestId"],
                        error="命令未完成，请核对停稳、空车及接口状态: " + type(exc).__name__)
                finally:
                    self._vol_extra = []
                    self._measurement_window = None
                    self._volume_observation_window = None
                    self._volume_extra_members = ()
                    self._measurement_gate = None
                self._publish_state(frame)
                return True

        # ★ 残缺/不可信帧说明不了"车走了"（与第 7b 条同理，2026-09-24 现场查实）。
        #   超时强开会话会 reset 检测器，它要等下一帧**可信**的帧才重新确认"有车"；
        #   这之间来一帧残缺帧，检测器就报 IDLE（带 degraded）。原来照单全收、当成车走了 ——
        #   把 _last_save_name / _session_forced 清掉，同一辆车的下一份测量就丢了
        #   duplicate_of（9-24 CH4 10:07:12 那份，当时正在大量丢包）。
        #   会话进行中遇到这种 IDLE 只跳过这一帧，由下一帧可信的帧来裁决车在不在。
        if stab.state == StabilityState.IDLE and stab.degraded and self._session_id:
            self._publish_state(frame)
            return True

        if stab.state == StabilityState.IDLE:
            # 状态机到 vehicle_threshold 才建车次；进场采集的低门槛可以更早。
            # 仅在尚无车次且软件判完整/新鲜的帧上预存，真正空场即清空，
            # 后续跨过有车门槛时再绑定到新车次。
            if self._session_id:
                self._enter_reset()
            elif (self.enter_frames > 0 and self.enter_min_points > 0 and
                  self.enter_min_points < self.stability.vehicle_threshold and
                  complete and stab.fresh and not stab.degraded and
                  stab.n_vehicle_points >= self.enter_min_points):
                self._grab_enter(stab, complete)
            elif self._enter_shots or self._enter_done:
                self._enter_reset()
            if self._session_id:
                self._flush_exit()           # 车走了 -> 离场帧整批落盘（没开或不足 2 帧时只清空）
            if self._session_id and self._detected_this_session:
                self.bus.publish("session_end", {"channel": self.ch.id,
                                                 "session": self._session_id,
                                                 "saved": self._saved_this_session,
                                                 "triggers": dict(self._triggers)})
            self._session_id = 0
            self._detected_this_session = 0
            self._saved_this_session = 0
            self._warned = False
            self._triggers.clear()
            # 能回到空闲，说明有车判据是工作的 —— 清掉健康告警
            self._timeouts_in_a_row = 0
            if self.st.health:
                self.st.health = ""
            # 车真的离开了 —— 下一次落盘是新的一辆车，不能再挂 duplicate_of
            self._last_save_name = ""
            self._local_passage = None
            self._session_forced = False
            self._discard_observation_window(vehicle_changed=True)
            self._publish_state(frame)
            return True

        if not self._session_id:
            self._session_id = self.st.session + 1
            self.st.session = self._session_id
            self._detected_this_session = 0
            self._saved_this_session = 0
            # 低门槛时已经采到的同一段连续进场帧继续留给本车。
            # 若中间断流/换代，则丢弃，避免把旧场景误绑给新车次。
            preentry = bool(self._enter_shots)
            if preentry:
                last = self._enter_last_identity
                preentry = (last is not None and last[0] == frame.gen and
                            last[1] < frame.seq and
                            0. < frame.ts - last[2] <= self.calib.detection.stationary_max_gap_sec)
            if not preentry:
                self._enter_reset()
            self._exit_reset()
            self._session_t0 = time.monotonic()
            self._session_forced = False
            if self._local_passage is None:
                self._local_passage = "local-" + uuid.uuid4().hex
            self.bus.publish("session_start", {"channel": self.ch.id,
                                              "session": self._session_id,
                                              "n_points": frame.n})

        self._runtime_observe(frame)

        # ★ 会话超时保险（devices.json 的 fusion.session_timeout_sec，默认 180s）
        #   一个会话只跑一次完整检测（对齐原版 stable_save_max）。万一某种没预料到的
        #   点云让通道一直判成"有车"而回不到 IDLE，会话就永远不结束，
        #   后面真来的车会被静默跳过。这里给一条硬性下限：超时就强制翻篇，
        #   下一辆车至少还有机会被测到。宁可多测一次，不可漏测。
        if (self.session_timeout > 0
                and time.monotonic() - self._session_t0 > self.session_timeout):
            self._timeouts_in_a_row += 1
            self.bus.publish("warn", {
                "channel": self.ch.id,
                "message": f"会话 {self._session_id} 超过 {self.session_timeout:.0f}s 仍未结束，"
                           f"已强制开新会话（检查 ROI / vehicle_threshold 是否把静止物体当成车）"})
            # ★ 连着两次都没等到"车走"（≈6 分钟从不空闲）→ 挂常驻告警。
            #   只发日志是不够的：2026-09-13 现场就是这条日志发了 3 次没人看见，
            #   因为当时既没有落盘日志，看板上也只是一行会被刷走的记录。
            #   两种可能都写进去，让现场自己一眼分辨 —— 别替他下结论。
            if self._timeouts_in_a_row >= 2:
                n_all = int(self.st.stability.get("n_points") or 0)
                n_veh = int(self.st.stability.get("n_vehicle_points") or 0)
                pct = (100.0 * n_veh / n_all) if n_all else 0.0
                held = self._timeouts_in_a_row * self.session_timeout
                mins = (f"{held/60.0:.0f} 分钟" if held >= 60 else f"{held:.0f} 秒")
                cut = self.calib.detection.ground_z + self.calib.detection.vehicle_min_height_m
                self.st.health = (
                    f"已连续 {mins}判定为「有车」且从未回到空闲。"
                    f"当前 ROI 内 {n_all} 点里有 {n_veh} 点（{pct:.0f}%）被算作车辆点"
                    f"（判据：z > {cut:.1f}）。"
                    f"若车道上确实停着车，属正常；"
                    f"**若车道是空的，说明有车判据已失效** —— 地面没落在 "
                    f"ground_z={self.calib.detection.ground_z} 附近，"
                    f"此时真来的车会被静默跳过。"
                    f"查：cli/capture.py + cli/probe.py 定 range_mode，或核对雷达安装高度。")
                self.bus.publish("warn", {"channel": self.ch.id,
                                          "message": self.st.health})
            self._enter_reset()              # 测量会话重开清进场缓存，物理车次仍是同一辆车
            # 本车未离场：保留离场资格和已采帧，测量次数仍按新会话重置。
            self._session_id = self.st.session + 1
            self.st.session = self._session_id
            self._detected_this_session = 0
            self._saved_this_session = 0
            self._session_t0 = time.monotonic()
            # ★ 打标：这个会话是**强开**的，不是"车走了又来一辆"。
            #   车道从来没回到过空闲（回到空闲会走上面的 IDLE 分支，
            #   那里把 _last_save_name 清掉），所以这个会话里再落的盘
            #   必定是同一辆车的第二份 —— 写进 meta 的 duplicate_of 让下游去重。
            #   这是**状态机的直接推论**，不是靠比几何量猜的。
            self._session_forced = True
            self.stability.reset()
            # ★ 翻篇这一拍必须**就地收尾**，不能往下走：
            #   ① stab 是 reset 之前算出来的，已经过期 —— 它可能还写着"已停稳"，
            #      于是下面会开一个沉降窗口，而 reset 后的下一帧只是在重建参考帧
            #      （stable=False），窗口立刻被作废，刷出一条
            #      "沉降窗口内车辆再次移动（偏差 0.000 m）" —— 车根本没动。
            #   ② 上一个会话没走完的 _settle_t0 必须清掉，否则新会话会拿着
            #      旧窗口的起点算 waited，直接**跳过**沉降窗口就测量。
            self._discard_observation_window(vehicle_changed=True)
            self._warned = False
            self._publish_state(frame)
            return True

        # 进场帧是只读原始观测；后台车牌/基线尚未就绪也必须继续采，
        # 否则最早经过雷达下方的车厢前端会永久缺失。测量仍由下面的准备闸门保护。
        if not stab.stable and self._settle_t0 is None:
            self._grab_enter(stab, complete)
            self._grab_exit(stab, complete)

        # 重启/断帧后尚未重新确认完整空闲，不能给仍在场的旧车新建测量身份。
        if (getattr(self, "_deferred", None) is not None and self.eff_mode in ("live", "udp", "real")
                and not self.integration.current_passage(self.ch.id)):
            self._discard_observation_window()
            self.st.last_diag["preparation"] = "awaiting_passage"
            self._publish_state(frame)
            return True

        # 兼容未启用异步留样的路径；异步路径只等待可靠车次，不等待车牌或基线。
        if getattr(self, "_deferred", None) is None and self.integration is not None and hasattr(self.integration, "preparation_status"):
            preparation = self.integration.preparation_status(self.ch.id)
            if preparation not in ("ready", "baseline_required", "not_applicable"):
                self._discard_observation_window()
                self.st.last_diag["preparation"] = preparation
                self._publish_state(frame)
                return True

        # 停稳 + 本会话尚未检测过 → 进入**沉降窗口**，窗口走完才测这一次
        if (stab.stable and self._detected_this_session < self.max_save
                and (self._deferred.can_accept(self.ch.id) if getattr(self, "_deferred", None) is not None else
                     self._finalizer is None or self._finalizer.can_accept(self.ch.id))):
            now = time.monotonic()
            if self._settle_t0 is None:
                self._settle_t0 = now
                self._settle_frames = []
                self._settle_pairs = []
                self._flush_enter()          # 判到停稳 -> 把进场帧整批落盘
                # ★ 车牌识别在开窗那一刻异步起跑（M4，见 _plate_prefetch）：
                #   与 3 秒沉降窗口并行，测量时只收结果，不再持锁空等 5 秒。
                self._plate_prefetch()
            # ★ 沉降窗口的帧**一律留下**（2026-09-17）。
            #   它有两个完全不同的用途，别混：
            #     · `settle_accumulate=True` 时合并成**测量/落盘用的那一帧**
            #       —— 这条会改判定链看到的点云，默认关闭，行为不变；
            #     · **体积的货面**用多帧填（`compute_volume(loaded_extra=...)`）
            #       —— 这条不碰判定，也不碰车厢盒，所以一律开。
            #   为什么要开：2026-09-17 在两辆有磅单真值的车上实测，
            #   货面用 3 帧后车厢盒内覆盖 78.4%->91.5% / 79.8%->85.1%，
            #   "换一帧就换一个数"的敏感性从 **20% 掉到 0%**，
            #   两车密度一致性从差 4.8% 收到 1.3%（docs/13 §16.2）。
            #   ▲ 但**绝不能拿合并后的点云去定车厢盒** —— 合并会把盒撑长
            #     （1 帧 11.5 m -> 3 帧 14.35 m），实测体积会算成 69 而非 55。
            self._settle_frames.append(frame)
            self._settle_pairs.append(self._current_pair)
            keep = self.settle_max_frames if self.settle_accumulate else min(self.settle_max_frames, self._VOL_EXTRA_MAX + 1)
            if len(self._settle_frames) > keep:
                self._settle_frames.pop(0)          # 丢最旧，保留最靠近窗口末尾的
                self._settle_pairs.pop(0)
            # 预算限制发生在测量前；合帧可保留最新有限窗口，不改变已产生结果。
            while self._raw_shots and not self._can_finalize():
                self._raw_shots.pop(0)
                self._note_raw_dropped()
            while len(self._settle_frames) > 1 and not self._can_finalize():
                self._settle_frames.pop(0)
                self._settle_pairs.pop(0)
            if not self._can_finalize():
                if time.monotonic() - self._settle_warn_t > 5.:
                    self._settle_warn_t = time.monotonic()
                    self.bus.publish("warn", dict(channel=self.ch.id,
                        message="当前单帧不能取得结果内存预算，继续观测，暂不测量"))
                self._settle_clear()
                self._raw_shots = []
                self.st.stability["waiting_result_capacity"] = True
                self._publish_state(frame)
                return True
            self.st.stability["waiting_result_capacity"] = False
            waited = now - self._settle_t0
            # 未裁 ROI 的整帧：首 / 中 / 末（残缺帧跳过，见 _grab_raw）
            self._grab_raw(waited, complete=complete)
            # 额外诊断采样不能挤掉开启保存所必需的主观测与检测结果。
            while self._raw_shots and not self._can_finalize():
                self._raw_shots.pop(0)
                self._note_raw_dropped()
            self.st.stability["settling"] = True
            self.st.stability["settle_remain"] = round(max(0.0, self.settle_sec - waited), 2)
            if waited < self.settle_sec:
                self._publish_state(frame)          # 还在沉降，先不测
                return True

            # 仅连续完整新帧可到此处；残缺帧不能靠超时取得测量许可。
            self._degraded_measure = False
            self.st.stability["waiting_complete"] = False
            target = self._merge_settled(frame)
            pairs = self._settle_pairs if self.settle_accumulate else self._settle_pairs[-1:]
            self._measurement_window = (ObservationWindow(tuple(pairs))
                                        if pairs and all(pair is not None for pair in pairs) else None)
            support = self._settle_pairs if self.settle_accumulate else self._settle_pairs[-(self._VOL_EXTRA_MAX + 1):]
            self._volume_observation_window = (ObservationWindow(tuple(support))
                                               if support and all(pair is not None for pair in support) else None)
            extra_count = min(self._VOL_EXTRA_MAX, len(self._settle_frames) - 1)
            self._volume_extra_members = tuple(range(len(support) - 1 - extra_count, len(support) - 1))
            # 体积的货面用窗口里的其它帧补（不含 target 自己，避免重复计权）
            #
            # ★ 帧数**必须显式设上限**，不能跟着 settle_max_frames 走：
            #   · 它默认 60 —— 那就是往货面里塞最多 59 帧、约 650 万点，
            #     而实测验证过的只有 **3 帧**（docs/13 §17.2）；
            #   · 写成 `[-(settle_max_frames - 1):]` 时，settle_max_frames=1
            #     会退化成 `[-0:]` = **整个列表**（Python 的 -0 就是 0），
            #     正好和"只留 1 帧"的意图相反。
            extra = [f.xyz for f in self._settle_frames if f is not frame]
            self._vol_extra = extra[-self._VOL_EXTRA_MAX:] if self._VOL_EXTRA_MAX else []
            self._measurement_gate = dict(decision=decision_gate, manual_three_frames=[],
                settle=dict(kind="automatic", applicable=True, started_monotonic=self._settle_t0,
                            finished_monotonic=now, waited_sec=waited, required_sec=self.settle_sec,
                            accumulate=self.settle_accumulate))
            # ★ 窗口清空必须放在检测**之后**（2026-09-23，两轮审查 H1）：
            #   _run_detection -> _build_result -> _window_evidence / _empty_evidence
            #   读的正是 self._settle_frames —— 原来清在检测前，生产链路上
            #   这两个诊断恒读到空列表：「同窗口其它帧有没有证据」修了但
            #   没接通（TestWindowEvidence 伪造 _settle_frames 直调方法本体，
            #   守住了函数没守住链路）。放 finally 还能保证异常路径不留半开窗口。
            try:
                self._run_detection(target, stab)
            finally:
                self._vol_extra = []
                self._measurement_window = None
                self._volume_observation_window = None
                self._volume_extra_members = ()
                self._measurement_gate = None
                self._settle_clear()
        else:
            self._publish_state(frame)

        # 停稳超时告警（原版 stationary_warn_sec）
        #
        # ★ 本会话**已经测出结果**之后就不再报（2026-09-15 现场统计后加的条件）
        #
        #   timed_out() 量的是"车进来多久了"（_entered 在车出现那一刻置位），
        #   不是"停稳了多久"。而现场车辆中位停留 37 s > 阈值 30 s，于是
        #   **车一开走就必然触发**：状态离开 STABLE + 计时早已超时 = 报警。
        #   2026-09-15 全天 53 次告警，逐条比对时刻：
        #       告警距会话结束 中位 2 s、最大 4 s，53/53 都在车走的前几秒。
        #   一次真正的"停着却判不稳"都没有 —— 这条告警 100% 是误报，
        #   而它把日志刷满之后，真正有用的 result / 超时告警全被顶掉。
        #
        #   加的条件是"本会话还没测出结果"：
        #     · 车进来 30 s 还没测出来 -> 照旧报，这正是原版要提醒的事；
        #     · 已经测完了再动 -> 那是车在走，不是异常，闭嘴。
        #   判据本身一个字没动，改的只是**什么时候值得吭声**。
        if (stab.state != StabilityState.STABLE and self.stability.timed_out()
                and not self._warned and self._detected_this_session == 0):
            self._warned = True
            self.bus.publish("warn", {
                "channel": self.ch.id,
                "message": f"车辆停稳超过 {self.dev.stationary_warn_sec:.0f}s 仍未稳定"
                           f"（拦在 {trigger_digest(self._triggers) or '无触发记录'}；"
                           f"旧偏差 {stab.deviation:.4f} m 仅供参考，不决定停稳）"})
        return True

    # ── 沉降窗口 ────────────────────────────────────────────
    def _settle_clear(self) -> None:
        # ★★ 这里**不许**再清进场缓冲（2026-09-23 修）。
        #   原来 _settle_clear 顺手做了 _enter_shots.clear()。而车在动的每一帧都会走
        #   `elif not stab.stable: self._discard_observation_window()` -> 这里，
        #   紧接着 _grab_enter 又追加 1 帧 —— 于是 90 帧的环形缓冲**永远只剩最新 1 帧**，
        #   "车往里开时把车厢前端补上"要的进场序列从来没攒起来过，落盘的只是停稳前
        #   最后一两帧（仿真实测：进场 8 个位置，缓存一直是 1，落盘 2 帧点数几乎相同）。
        #   两种状态性质相反，不能一起清：
        #     · 沉降窗口是给**停住的车**用的，车一动就该作废；
        #     · 进场缓冲的全部意义就是**车在动的时候**攒帧，只该在换车时重置。
        #   进场缓冲改由 _enter_reset() 在新会话开始时单独清。
        self._settle_t0 = None
        self._settle_frames = []
        self._settle_pairs = []
        self.st.stability["settling"] = False
        self.st.stability["settle_remain"] = 0.0

    def _enter_reset(self) -> None:
        """进场缓冲属于一辆车：确认空场、换车或源代次切换时清。"""
        self._enter_shots.clear()
        self._enter_positions.clear()
        self._enter_seen = 0
        self._enter_selected = 0
        self._enter_done = False
        self._enter_last_identity = None
        self._enter_last_position = None
        self._enter_step_eff = self.enter_step_m
        self._enter_volume = ()

    def _discard_observation_window(self, vehicle_changed: bool = False) -> None:
        """活动观测失效；不清待提交的不可变测量，归档重试与本窗口分离。

        ★ `vehicle_changed` 只在**换车**时给 True（回到空闲 / 强开新会话 / 演示换场景），
        只有那时才清"本车当前结果"的标记 `_current_result_token`。
        2026-09-24 现场查实：原来这里无条件清，而车没停稳的**每一帧**都走到这里 ——
        车测完 1 秒就起步、归档还没提交完（约 3 秒），结果就被当成「历史补交」：
        看板不出这辆车的结论、「上一辆」还挂着再早一辆的，会话存盘数记成 0。
        车在动、断流都不是换车，本车的结果仍是本车的。
        """
        self._settle_clear()
        # 车牌预取随观测一起作废（M4）：任何没被测量消费掉的预取 ——
        # 比如 reserve 抛"预算不足"后翻篇 —— 不许带到下一辆车去，
        # 否则下一车会收到上一车的牌。
        self._plate_prefetched = False
        self._observation_epoch += 1
        if vehicle_changed:
            self._current_result_token = ""
        self._current_pair = None
        self._raw_shots = []
        self._vol_extra = []
        self._integration_frames.clear()
        with self._recent_lock:
            self._recent_frames.clear()
        self.st.stability["waiting_complete"] = False
        self.st.stability["stable"] = False
        self.st.stability["stationary_sec"] = 0.0

    def _merge_settled(self, last: Frame) -> Frame:
        """窗口结束时给出要测量/落盘的那一帧。

        · settle_accumulate=False（默认）→ 直接用窗口末尾那一帧。
          车是静止的，末帧就是余振最小的一帧。
        · True → 把窗口内所有帧拼成一份更密的点云。
          注意：雷达在固定位置扫固定车辆，每一圈打到的方向几乎相同，
          所以合并**主要是补掉零星丢点、平均噪声**，并不会显著扩大覆盖；
          代价是点数成倍增长（.xyz 体积 = 4 + 点数×14 字节）。
        """
        if not self.settle_accumulate or len(self._settle_frames) < 2:
            return last
        import numpy as _np
        xs = [f.xyz for f in self._settle_frames]
        rings = [f.ring for f in self._settle_frames]
        xyz = _np.concatenate(xs, axis=0)
        ring = (_np.concatenate(rings, axis=0)
                if all(r is not None and len(r) == len(x) for r, x in zip(rings, xs))
                else None)
        return Frame(xyz, ring, seq=last.seq, ts=last.ts, gen=last.gen)

    def _refresh_stats(self) -> None:
        self.st.frames = self.source.stats.frames
        self.st.fps = round(self.source.stats.fps, 2)
        info = self.source.info()
        if info.get("error"):
            self.st.error = str(info["error"])
        elif self.st.error:
            self.st.error = ""

    # ── 完整检测 ────────────────────────────────────────────
    def _gate_decision(self, frame, stab, complete):
        """只记录本次原精度许可，不从UI舍入值或末帧补造完整事件流。"""
        names = ("delta", "vehicle_threshold", "ground_z", "vehicle_min_height_m", "count_ratio",
                 "stationary_min_sec", "stationary_speed_mps", "stationary_drift_m",
                 "stationary_shape_speed_mps", "stationary_yaw_speed_mps",
                 "stationary_speed_window_sec", "stationary_max_gap_sec", "stationary_jitter_m")
        return dict(frame=dict(seq=frame.seq, ts=frame.ts, gen=frame.gen, packets=frame.packets),
                    received_monotonic=self._last_frame_monotonic, stability=asdict(stab),
                    thresholds={name: getattr(self.stability, name) for name in names},
                    frame_policy=dict(accepted=bool(complete), packet_count_known=frame.packets > 0))

    def _run_detection(self, frame: Frame, stab, command=None):
        """自动测量冻结后交后台计算；人工命令保持原许可和计算路径。"""
        deferred = getattr(self, "_deferred", None)
        if (command is None and deferred is not None and
                self.integration.current_passage(self.ch.id) is not None):
            return deferred.capture(self, frame, stab)
        identity = None
        previous_count = self._detected_this_session
        previous_token = self._current_result_token
        baseline_version_id = None
        baseline_snapshot = None
        if self.save_enabled or self.integration is not None:
            self._ensure_finalizer()
            identity = self._finalizer.reserve(self.ch.id, uuid.uuid4().hex,
                self._local_passage, self._finalize_estimate(frame), command)
            self._current_result_token = identity.origin_token
        try:
            if self.integration is not None:
                baseline_snapshot = self.integration.baseline_snapshot(identity, frame.ts, self._integration_hash)
                self.baseline = baseline_snapshot.baseline() if baseline_snapshot else None
                baseline_version_id = baseline_snapshot.binding["baseline_version_id"] if baseline_snapshot else None
                if command and command["kind"] == "remeasure" and not baseline_version_id:
                    raise MeasurementRejected("预留车次没有本次配置适用的正式基线")
            measurement = self._calculate_detection(frame, stab, command, identity, baseline_version_id, baseline_snapshot)
            self._mark_exit_eligible(measurement, identity.passage_id if identity is not None else None)
            return measurement
        except MeasurementRejected as error:
            if identity is not None:
                self._finalizer.reject(identity, error)
            # 正常业务拒绝没有产生待保存结果，不占用本车自动测量名额。
            # 真正冻结/提交失败走下方分支，仍保留原计数，不能重采换 ID。
            self._detected_this_session = previous_count
            self._current_result_token = previous_token
            raise
        except Exception as error:
            if identity is not None:
                # ★ `_pair_error` 写了从不读的时候，这条路径丢的正是唯一线索：
                #   `ObservationPair.freeze` 为什么没冻出观测对 —— 而它恰好是
                #   `prepare_result` 报「归档缺少真实原观测」的上游原因。
                self._finalizer.capture_failed(identity, error, detail=self._pair_error)
            raise

    def _build_result(self, *, frame, stab, plate, plate_res, ok, reason, dg, res, load,
                      vol, dim_ok, dim_msg, n_input, elapsed, saved_path) -> dict:
        """组装一次测量对外的结果字典。

        从 `_calculate_detection` 里搬出来的（2026-09-18 重构）——
        那个函数原来 197 行，其中 50 行只是在填这张表，读的人得先翻过
        车牌、判定、体积三段才看得到"最后到底报了什么"。
        **字段、顺序、取值表达式一个字没动**，纯位置移动。

        ▲ 依赖的局部量全部**显式传参**，不从 self 上偷读 ——
          否则搬走之后就看不出它到底依赖了调用处的哪些中间结果。
        """
        return {
            "channel": self.ch.id,
            "channel_name": self.ch.name,
            "session": self._session_id,
            "seq": frame.seq,
            "source_generation": frame.gen,
            "ts": frame.ts or time.time(),
            "plate": plate,
            "plate_ok": bool(plate_res.ok),
            "plate_error": plate_res.error,
            "empty": bool(ok),
            "reason": reason,
            "n_points": int(n_input),
            "deviation": round(float(stab.deviation), 5),
            "elapsed_ms": int(elapsed * 1000),
            "dim_ok": bool(dim_ok),
            "dim_message": dim_msg,
            # ★ 判定链走到第几步被拦 + 逐步的实测/门槛/余量（2026-09-16 加）
            #   现场原来只看得到一句理由，看不出"离通过差多少"。
            "step": int(getattr(dg, "step", 0) or 0),
            # ★ 三态（2026-09-19）：判定链十步全部 return False，于是
            #   「没看清」与「看清了、有货」压成同一个答案，现场分不清
            #   该**重测**还是该**拦截**。实测 196 帧里判 False 的 186 帧中，
            #   真正证据不足只有 12 帧（6.5%）。纯诊断，从 step 派生。
            "verdict": dg.verdict,
            # 证据不足时，沉降窗口里其它帧的结论（只在需要时才算，见方法注释）
            "window_evidence": self._window_evidence(dg, frame),
            # ★ 一句人话说清卡在哪、差多少；第 6/7 步还会带上余高与 z_above
            #   —— 实测第 7 步 161 帧**结论全对、理由全错**（真实原因是
            #   拟合面落在货堆顶），不摆物理量现场会去调错的那个门槛。
            "reason_detail": reason_detail(dg, self.dev),
            "gates": gate_report(dg, self.dev),
            # 这一份是在等不到完整帧的情况下硬测的 —— 内点/占比可能偏低，要打折看
            "frame_degraded": bool(self._degraded_measure),
            # 双平面（底板 + 货面）：纯诊断，判定链不看
            "load_profile": load.to_dict() if load is not None else None,
            "saved": saved_path,
            "save_enabled": self.save_enabled,
            "dims": list(res.dims) if res is not None else [0.0, 0.0, 0.0],
            "diagnostics": dg.to_dict(),
            "volume": vol.to_dict(),
            "planes": (res.to_dict()["planes"] if res is not None else {}),
            # 点云只发降采样后的坐标，够画三维散点图但不至于把 SSE 撑爆
            "cloud": _downsample_xyz(res.cloud if res is not None else frame.xyz, 4000),
            "fit_ok": bool(res.fit_ok) if res is not None else False,
            "config": self._config_fp,
            "needs_truth_label": self._worth_labeling(dg),
            # ★ 自动挑空车基线的闸门证据（2026-09-18）。
            #
            #   为什么随每次测量都算：现场车太多，"每辆车人工点一次激活"
            #   不可行，基线必须能自动挑。而源端**不自我授权** —— 它只把
            #   七道闸门的实测值、门槛和余量算齐报上去，由授权方（后台）
            #   据此决定要不要采候选、要不要激活。激活权限在业务侧。
            #
            #   ▲ 失败是**静默且永久**的：把载货点云存成空车基线，这辆车
            #     之后的体积恒为接近 0 且不报错。所以闸门取**保守**方向 ——
            #     124 份现场样本里只有 5.6% 通过，低于预期的空车比例（约 22%）。
            #   详见 `detect.empty_evidence` 的说明与 `docs/34`。
            "empty_evidence": self._empty_evidence(res, frame),
        }

    def _calculate_detection(self, frame: Frame, stab, command=None, identity=None, baseline_version_id=None,
                             baseline_snapshot=None):
        t0 = time.time()
        n_input = frame.n
        self._detected_this_session += 1
        # ① 车牌 —— 收**开窗时就已异步起跑**的识别结果（M4）。
        #    相机 HTTP 预算（f2+f3 默认 5 秒）大部分在锁外的沉降窗口里
        #    并行耗掉；这里最多等 _PLATE_LOCK_BUDGET_S，持锁空等从 5 秒
        #    压到 2 秒封顶，四路同时出结果时看板不再卡死。
        plate_res = self._plate_collect()
        if self.integration is not None and hasattr(self.integration, "prepared_plate"):
            bound_plate = self.integration.prepared_plate(self.ch.id)
            if bound_plate:
                plate_res = PlateResult(ok=True, plate=bound_plate, confidence=1.0,
                                        raw="WebApi 已绑定本车车牌", attempts=1)
        plate = plate_res.plate or EMPTY_PLATE
        if not plate_res.ok and plate_res.error:
            self.bus.publish("plate_miss", {"channel": self.ch.id,
                                            "error": plate_res.error})
        self.st.last_plate = plate

        # ② 空车检测 + 判定链
        res = detect_empty(frame, self.calib)
        ok, reason, dg = evaluate_empty(res, n_input, res.cloud if res else frame.xyz,
                                       self.dev, ground_z=self.calib.detection.ground_z)
        self.st.last_reason = reason
        self.st.last_verdict_ok = bool(ok)
        self.st.last_diag = dg.to_dict()

        # ②b 双平面（底板 + 货面）—— **纯诊断，不改变判定结果**。
        #    它回答的是"装了多大一片、装多高"，判定链一个字都不看它。
        #    实测约 30 ms（检测链本身 50~70 ms），而检测每辆车只跑一次，可以忽略。
        #    ★ 用 try 兜住：诊断再有用也不能拖垮一次测量。
        try:
            load = analyze_load(np.asarray(res.cloud, np.float64)) if res is not None else None
        except Exception as e:                              # noqa: BLE001
            load = None
            self.bus.publish("warn", {"channel": self.ch.id,
                                      "message": f"双平面诊断失败（不影响判定）：{e}"})

        # ③ 尺寸达标（诊断信息，不改变判定结果 —— 与原版一致）
        dim_ok, dim_msg = (True, "")
        if res is not None:
            dim_ok, dim_msg = check_dims(res, self.dev)

        # ④ 载货车检测 + 体积（原版走的是另一条分支；现场没有空车基线，
        #    所以这段在原系统里从未真正跑过。见 compute_volume 的 TODO(现场)）
        #
        #    ★ 没有空车基线时**不要**跑 detect_loaded：它要做四面拟合，
        #      6 万点实测 65ms，占整条检测链的七成，而结果会被 compute_volume
        #      在第一行就丢掉（没有基线 → 直接返回"无空车基线"）。
        #      现场至今没标定过基线，等于每辆车白烧这 65ms。
        # ★★ 到这里才知道是哪一辆车 —— 按车牌取**这辆车自己的**空车基线。
        #   后台接管时不走这条（上面 _run_detection 已按车次绑定版本取过）。
        if self.integration is None:
            self.baseline, self._baseline_plate = self._resolve_baseline(plate)
        has_base = bool(self.baseline is not None and self.baseline.n)
        loaded = detect_loaded(frame, self.calib) if has_base else None
        # 双平面仅提供诊断高度，不是裸露底板证据；薄煤也可能形成完整平面。
        # TODO(现场): 尚无独立参考供应路径，不能由load.ok生成授权修正的证据。
        # 只有显式模型能提供物理条件，旧box/拟合煤面不能充当内腔或误差来源。
        from ..volume_inputs import align_extra_surfaces
        volume_options = dict(volume_context=None, volume_scope=None,
            loaded_extra=(align_extra_surfaces(loaded.source_rotation, self._vol_extra) or None)
                         if loaded is not None and self._vol_extra else None)
        volume_runtime = None
        if self.volume_catalog is not None:
            from ..volume_runtime import runtime_volume_options
            volume_options, volume_runtime = runtime_volume_options(catalog=self.volume_catalog,
                snapshot=baseline_snapshot, baseline=self.baseline, loaded=loaded, frame=frame,
                primary=self._measurement_window, support=self._volume_observation_window,
                extra_members=self._volume_extra_members, extras=self._vol_extra,
                channel=self.ch.id, capture_instance=self._capture_instance_id, epoch=self._observation_epoch,
                identity=identity, configuration_hash=self._integration_hash)
        from ..detect import VOLUME_CELLS
        actual_params = dict(cells=VOLUME_CELLS, spread_target=.03, floor_band_m=.5, bed_shift_max_m=.5,
                             dy=None, bed_z=load.bed_z if load is not None and load.ok else None,
                             bed_evidence=None, bed_scope=None, **{k: v for k, v in volume_options.items() if k != "dy"})
        actual_params["dy"] = volume_options.get("dy")
        shared_baseline = baseline_snapshot is not None and baseline_snapshot.source_json is not None
        if shared_baseline:
            actual_params["shared_baseline"] = True
            if (volume_runtime or {}).get("status") != "supplied":
                actual_params["dy"] = None
        # ★ 进场帧补两端 + 离场帧补后端 + 遮挡上限补洞（docs/92；离场对称接入）。
        #   配不上只是不补，不拦测量；配准记录随归档走，复算时按原帧重新配准核对。
        enter_frames, enter_record = (), None
        exit_frames, exit_record = (), None
        extra_clouds = []
        if has_base and loaded is not None and self._enter_volume:
            from ..enter_volume import register_for_volume
            enter_frames = self._enter_volume
            enter_clouds, enter_record = register_for_volume(loaded, enter_frames)
            if enter_clouds is not None:
                extra_clouds.extend(enter_clouds)
        if has_base and loaded is not None and self._exit_volume:
            from ..enter_volume import register_exit_for_volume
            exit_frames = self._exit_volume
            exit_clouds, exit_record = register_exit_for_volume(loaded, exit_frames)
            if exit_clouds is not None:
                extra_clouds.extend(exit_clouds)
        if extra_clouds:
            actual_params["enter_extra"] = tuple(extra_clouds)
        actual_params["fill_bound"] = True
        vol = compute_volume_auto(self.baseline if has_base else None, loaded, **actual_params)
        if vol.declared_model_used:
            # 声明模型只绑定主观测与停稳支持帧；归档不能把进/离场帧冒充积分输入。
            enter_frames, enter_record = (), None
            exit_frames, exit_record = (), None
            actual_params.pop("enter_extra", None)
            actual_params["fill_bound"] = False
        if enter_record is not None:
            vol.text += ("（进场帧 %d 帧：%s）" % (len(enter_frames), enter_record["reason"])
                         if enter_record["status"] != "resolved" else "")
        if exit_record is not None:
            vol.text += ("（离场帧 %d 帧：%s）" % (len(exit_frames), exit_record["reason"])
                         if exit_record["status"] != "resolved" else "")
        if volume_runtime is not None:
            volume_runtime["range_status"] = vol.range_status

        elapsed = time.time() - t0

        # ⑤ 文件保存由后台原包投影；这里尚未持久化，不能先增加成功计数。
        saved_path = ""
        archive_enabled = self.save_enabled and n_input >= self.dev.min_save_points
        if self.save_enabled and not archive_enabled:
            self.bus.publish("skip_save", {
                "channel": self.ch.id, "n_points": n_input,
                "need": self.dev.min_save_points})

        # ⑥ 组装结果 + 广播（字典本体见 _build_result）
        result = self._build_result(
            frame=frame, stab=stab, plate=plate, plate_res=plate_res, ok=ok, reason=reason,
            dg=dg, res=res, load=load, vol=vol, dim_ok=dim_ok, dim_msg=dim_msg,
            n_input=n_input, elapsed=elapsed, saved_path=saved_path)
        measurement = None
        if volume_runtime is not None:
            result["volume_runtime"] = volume_runtime
        if shared_baseline and vol.baseline_registration is not None:
            result["baseline_use"] = dict(json.loads(baseline_snapshot.source_json), **vol.baseline_registration)
            if vol.baseline_registration["registrationStatus"] == "baseline_fallback":
                result["reason"] += "；几何配准未完整通过，按CH4基准计算并发布正式体积"
        result["volume_preparation"] = ("parameters_required" if self.integration is not None and has_base and not shared_baseline
            and (volume_runtime or {}).get("status") != "supplied" else None)
        if self.integration is not None or archive_enabled:
            # 保存原精度质量指标，防止 to_dict 的四位小数把超门槛抹平。
            # 使用同一有效性出口；不能把内部失败默认零重新覆盖回对外结果。
            result["volume"] = vol.to_dict(raw=True)
        if identity is not None:
            if command and command["kind"] == "baseline-capture" and (
                    self.eff_mode != "live" or not ok or self._degraded_measure or res is None
                    or not res.plane("floor").valid):
                raise MeasurementRejected("候选只接受本次完整实车空车结果及有效底面")
            result["measurement_id"] = identity.measurement_id
            result["origin_token"] = identity.origin_token
            if self._session_forced and self._last_save_name:
                result["duplicate_of"] = self._last_save_name
                result["duplicate_reason"] = "会话强制开启，期间未观察到完整离场"
            modes = {"udp": "live", "real": "live", "simulate": "demo", "sim": "demo",
                     "sim-radar": "demo", "synthetic": "demo", "syn": "demo"}
            source_mode = modes.get(self.eff_mode, self.eff_mode)
            indices = (np.asarray(res.source_indices, "<i8") if res is not None
                       else np.arange(frame.n, dtype="<i8"))
            rotation = np.asarray(res.source_rotation if res is not None else np.eye(3), "<f8")
            cloud = res.cloud if res is not None else frame.xyz
            ring = frame.ring[indices] if frame.ring is not None else None
            candidate = (CandidateInput.freeze(frame, res)
                         if (command and command["kind"] == "baseline-capture" or
                             command is None and self.ch.id == "CH4" and source_mode == "live" and ok
                             and not self._degraded_measure and res is not None and res.plane("floor").valid
                             and result["empty_evidence"].get("ok") is True
                             and float(res.cloud[:, 2].max()) > float(res.plane("floor").intercept(2))) else None)
            runtime_files = ()
            if archive_enabled and self._measurement_window is not None and self._volume_observation_window is not None:
                from ..measurement_runtime import freeze_runtime
                if self._measurement_gate is None:
                    raise ValueError("测量缺少原始运行许可摘要，禁止事后补造")
                gate = dict(self._measurement_gate)
                gate.update(capture_instance=self._capture_instance_id, epoch=self._observation_epoch)
                asset = baseline_snapshot.asset_bytes if baseline_snapshot else None
                runtime_files = freeze_runtime(identity=identity, configuration=self._effective_configuration,
                    baseline=self.baseline if has_base else None, baseline_version=baseline_version_id,
                    asset=asset, loaded=loaded, params=dict(actual_params, fill_holes=True), runtime=volume_runtime,
                    volume=vol, gate=gate, primary=self._measurement_window, support=self._volume_observation_window,
                    extra_members=self._volume_extra_members, enter_frames=enter_frames, enter_record=enter_record,
                    exit_frames=exit_frames, exit_record=exit_record)
            pending = PendingResult(identity=identity, result_json=result_bytes(result),
                detection=FrameSnapshot.freeze(Frame(cloud, ring, frame.seq, frame.ts, frame.gen, frame.packets)),
                detection_indices=indices.tobytes(), detection_rotation=rotation.tobytes(),
                observation_window=self._measurement_window,
                volume_window=self._volume_observation_window, volume_extra_members=self._volume_extra_members,
                extra_frames=tuple(FrameSnapshot.freeze(item[1]) for item in self._raw_shots),
                configuration_hash=self._integration_hash,
                algorithm_version=self.integration.config.algorithm_version if self.integration else "clean-room-v1",
                baseline_version_id=baseline_version_id, source_mode=source_mode,
                lane_seed=tuple(tuple(row) for row in self.calib.roi) if self.calib.roi else None,
                ground_z=float(self.calib.detection.ground_z), archive_enabled=archive_enabled,
                candidate=candidate, runtime_files=runtime_files,
                entry_frame_times=tuple(item.ts for item in enter_frames),
                exit_frame_times=tuple(item.ts for item in exit_frames))
            self._finalizer.submit(pending)
            self._raw_shots = []
            self._last_save_name = identity.measurement_id
            self.bus.publish("result_pending", {"channel": self.ch.id, "measurement_id": identity.measurement_id})
            self._runtime_record("measurement_calculated", "检测计算已完成，等待结果持久化",
                                 identity=identity.measurement_id)
            self._degraded_measure = False
            return identity.measurement_id
        self.st.last_result = {k: v for k, v in result.items() if k != "cloud"}
        self.st.updated = time.time()
        self.history.append(self.st.last_result)
        self._runtime_record("measurement_completed", "本次检测已完成，详情见测量结果",
                             identity=result.get("ts"))
        self.bus.publish("result", result)
        self._degraded_measure = False        # 只对这一次落盘有效，用完即清
        return measurement

    def _finalize_estimate(self, frame=None):
        pairs = (self._volume_observation_window.pairs if self._volume_observation_window else
                 tuple(pair for pair in self._settle_pairs if pair is not None))
        size = sum(len(pair.raw.xyz_bytes) + len(pair.cropped.xyz_bytes) + len(pair.index_bytes)
                   + len(pair.raw.ring_bytes or b"") + len(pair.cropped.ring_bytes or b"") for pair in pairs)
        frame = frame if frame is not None else self.last_frame
        # 即使关闭云图保存仍需冻结候选输入和判定云；不能仅按 raw 窗口估算。
        points = (frame.n if frame is not None else 0)
        if self.settle_accumulate:
            points = max(points, sum(item.n for item in self._settle_frames))
        size += points * 64 + sum(item[1].n * 16 for item in self._raw_shots)
        # 原资产可能在估算后换版本；按下载合同上限预留，不能用可竞态缓存大小作承诺。
        runtime_size = ((32 * 1024 * 1024 if self.integration else 0) +
                        len(self._effective_configuration) + 128 * 1024 + len(pairs) * 4096) * 4 if self.save_enabled else 0
        return max(1024 * 1024, size * 6 + 1024 * 1024 + runtime_size)

    def _can_finalize(self):
        if getattr(self, "_deferred", None) is not None:
            return self._deferred.can_accept(self.ch.id, self._finalize_estimate())
        return self._finalizer is None or self._finalizer.can_accept(self.ch.id, self._finalize_estimate())

    def _ensure_finalizer(self):
        if self._finalizer is None:
            self._finalizer = ResultFinalizer({self.ch.id: (self.ch.abs_output_dir, self.save_enabled)}, self.integration)

    def _drain_finalizer(self):
        with self._control_lock:
            self._drain_finalizer_locked()

    def _drain_finalizer_locked(self):
        if self._finalizer is None:
            return
        for event in self._finalizer.poll(self.ch.id):
            if event["kind"] == "error":
                self._runtime_record("error", "测量归档或历史结果收尾异常，原结果保留并等待重试",
                                     session=0, identity=event.get("measurement_id"))
                self.bus.publish("error", dict(channel=self.ch.id, error=event["error"]))
                continue
            if event["kind"] == "durable":
                result = json.loads(event["result_json"])
                identity = event["identity"]
                result["archive_status"] = "export_pending" if event["archived"] else "disabled"
                if self.integration is not None:
                    result["integration"] = dict(status="queued" if event["managed"] else "quarantined",
                        measurementId=identity.measurement_id, passageId=identity.passage_id)
                current = result["origin_token"] == self._current_result_token
                result["current"] = current
                summary = {key: value for key, value in result.items() if key != "cloud"}
                if not any(row.get("measurement_id") == identity.measurement_id for row in self.history):
                    self.history.append(summary)
                if current:
                    self.st.last_result = summary
                    self.st.updated = time.time()
                self._runtime_record("measurement_completed",
                                     "本次检测结果已持久化" if current else "历史留样检测结果已持久化",
                                     session=result.get("session", 0), identity=identity.measurement_id)
                self.bus.publish("result" if current else "result_history", result)
            elif event["kind"] == "exported":
                result = next((row for row in self.history if row.get("measurement_id") == event["measurement_id"]), None)
                if result is not None and result.get("archive_status") != "complete":
                    result.update(archive_status="complete", saved=event["path"])
                    self.st.saved_count += 1
                    if result["origin_token"] == self._current_result_token:
                        self.st.last_saved = event["path"]
                        self._saved_this_session += 1
                    self.bus.publish("result_archive", dict(channel=self.ch.id, measurement_id=event["measurement_id"],
                        saved=event["path"], archive_status="complete"))

    def _save(self, frame: Frame, res: Optional[EmptyResult], plate: str,
              reason: str, dg, vol, plate_res, load=None, is_empty=False) -> str:
        """落盘：<output>/<车牌>/<时间>.xyz + .reason.txt（与原版目录结构一致）。

        ▲▲ **这个方法当前没有任何调用点（2026-09-23 查实，全仓零引用）。**
        测量的持久化已经整体改走成套归档：`ResultFinalizer` -> `ArchiveJournal.export`
        -> `<output>/archives-v1/<通道>/<车次>/<测量>/detection.xyz`（见上面
        第 ⑤ 步「文件保存由后台原包投影」那句注释）。

        所以**现场不会再看到 `<车牌>/<时间>.xyz` 这个目录布局** ——
        `cli/label_truth.py` / `cli/replay.py` 改用 `app.sample_paths.discover_samples`
        发现归档样本（实测能找到 detection.xyz 及其 meta/reason），功能没有断；
        断的只是"照旧目录名去翻文件"这个习惯。

        留着不删是因为它是**原版目录结构的唯一存档**：真要回退到原厂布局，
        这段是现成的。要动它之前先确认没有人按旧布局写脚本。
        """
        d = os.path.join(self.ch.abs_output_dir, safe_dir_name(plate))
        ensure_dir(d)
        name = time.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(d, name)
        # 同一秒内多次保存时加后缀，避免覆盖
        n = 1
        while os.path.exists(base + ".xyz"):
            base = os.path.join(d, f"{name}_{n}")
            n += 1
        cloud = res.cloud if res is not None else frame.xyz
        rng = frame.ring if res is None else None
        if res is not None and res.source_indices is not None and frame.ring is not None:
            rng = frame.ring[res.source_indices]
        save_xyz(base + ".xyz", Frame(cloud, rng, seq=frame.seq, ts=frame.ts,
                                       gen=frame.gen, packets=frame.packets))
        save_reason(base + ".xyz.reason.txt", reason)
        # 未裁 ROI 的整帧（service.raw_frames > 0 时）——与这次落盘的 .xyz 配对，
        # 文件名前缀相同，.json 里记了 paired_xyz，下午复算时一一对得上。
        n_raw = self._save_raw(os.path.basename(base))
        meta = {
            "channel": self.ch.id, "plate": plate, "reason": reason,
            # ★ 配置指纹（2026-09-17，docs/14 §1.8）。
            #   没有它的话，存档结论会随配置变化悄悄过期而**无从察觉** ——
            #   实测 62 份现场样本里已有 7 份（11%）的 reason 与用当前配置
            #   重算的结果不一致，其中 2 份现在是「空车」而 meta 记的是
            #   「底面过高，疑似载物」。而 docs/13 那套按车牌存基线的设计
            #   正是照 meta 的 `empty` 去挑基线的。
            "config": self._config_fp,
            # ★ 这里必须写**判定链的结论**，不是 res.ok。
            #   `EmptyResult.ok` 是"六面拟合成功了"，与空不空车没关系 ——
            #   2026-09-17 查实：现场 137 份 meta 里有 **115 份**写着 empty=True
            #   而实际是载货车（reason 是"底面内点占比过低"之类）。
            #   广播那条（result["empty"]）一直是对的，只有落盘这条写错了。
            #   下游按车牌存空车基线时会直接照着它把载货点云存成基线，
            #   所以这条在"按车牌绑定空车云图"那套设计里是致命的。
            "empty": bool(is_empty),
            "fit_ok": bool(res.ok) if res is not None else False,
            "n_points": int(len(cloud)),
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "diagnostics": dg.to_dict() if hasattr(dg, "to_dict") else {},
            "dims": list(res.dims) if res is not None else [0, 0, 0],
            "volume": vol.to_dict(),
            "has_baseline": bool(self.baseline and self.baseline.n),
            "plate_error": plate_res.error,
            "raw_frames_saved": n_raw,
            # ★ 判定链走到第几步被拦，以及逐步的实测值/门槛/余量。
            #   离线复算时不用再跑一遍就知道"卡在哪、差多少"。
            "step": int(getattr(dg, "step", 0) or 0),
            "gates": gate_report(dg, self.dev) if hasattr(dg, "step") else [],
            "frame_degraded": bool(self._degraded_measure),
            "load_profile": load.to_dict() if load is not None else None,
            # ★ 值不值得人工打标（2026-09-16 加，见 docs/12 §5 的 1.3 条）
            #
            #   现在全项目只有 2 份"空车"真值，而门槛正是靠它标定的 ——
            #   样本荒是空车判定最大的短板。这个标记把**信息量最大**的那些
            #   样本挑出来，让现场用 `cli/label_truth.py` 顺手打个标：
            #     · 判成空车的  —— 正样本，多多益善；
            #     · 第 10 步差一点点被拦的 —— 恰恰是门槛定得准不准的关键区。
            #   只是打标记，不改判定，也不多存一份点云（样本本来就落了盘）。
            "needs_truth_label": self._worth_labeling(dg),
        }
        # ★ 重复测量标记：会话是 session_timeout 强开的，而车道从上一次落盘
        #   到现在**一次都没回到空闲**（回到空闲会把 _last_save_name 清掉）——
        #   那么这一份和上一份就是同一辆车。下游按份数统计车次 / 算体积时
        #   照这个字段去重，别双计。
        #   2026-09-15 现场实测：下午 9 份落盘里有 2 份是这么来的
        #   （CH4 152838->153134、153206->153502，间隔都是 176 s），
        #   两份的 ROI 点数只差 0.3%，车一动没动。
        #   ▲ 只打标、不拦截，**阈值也别调**。那两次超时现场确认是在单独测试
        #     其它功能时把车故意停住的，超时触发得对（2026-09-16 现场答复），
        #     迄今没有一次误触发。而它是 2026-09-13"会话永不结束导致后面的车
        #     全部被静默跳过"那次事故的唯一兜底，调大 = 延长那类事故的静默期。
        #     要解决的只是"重测出来的那一份是重复数据"，与超时该不该触发是两回事。
        if self._session_forced and self._last_save_name:
            meta["duplicate_of"] = self._last_save_name
            meta["duplicate_reason"] = (
                "会话由 session_timeout 强制开启，且车道期间从未回到空闲 —— "
                "与 duplicate_of 指向的那一份是同一辆车")
        self._last_save_name = os.path.basename(base) + ".xyz"
        # 文件名与 README / docs 一致：<时间>.xyz.meta.json（挂在点云名后面，
        # 和 .xyz.reason.txt 一样），这样一个目录里三个文件是同一前缀，好排序好核对。
        with open(base + ".xyz.meta.json", "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
        return base + ".xyz"

    # 第 10 步的实测值在门槛的这个倍数以内，就算"差一点点"，值得人工确认
    _LABEL_NEAR_MISS = 3.0

    def _worth_labeling(self, dg) -> bool:
        """这一份值不值得拉去人工打标。见 _save 里的说明。"""
        step = int(getattr(dg, "step", 0) or 0)
        if step == 11:
            return True                       # 走完 11 步 = 判成空车，正样本一份都不能漏
        if step != 10:
            return False                      # 不是卡在第 10 步，信息量不大
        v = getattr(dg, "z_above_used", float("nan"))
        lim = self.dev.gate.z_above_min_max_m
        return v == v and lim > 0 and v <= lim * self._LABEL_NEAR_MISS

    def _report_position_standstill(self, frame, stab, complete, occupied, passage) -> None:
        """位置停稳且本车次已登记即通知 WebApi 触发双雷达；只读，不影响原判稳、留样或体积。"""
        try:
            usable = bool(complete and stab.fresh and not stab.degraded and occupied is not False)
            still = self._position_still.update(passage, float(frame.ts), self.stability.last_pose(), usable)
            report = getattr(self.integration, "report_standstill", None)
            if still and callable(report):
                report(self.ch.id, passage)
        except Exception:                                    # noqa: BLE001
            pass                                            # 触发通知失败不改变本车道的检测与测量

    def _runtime_publish(self, event) -> None:
        if event is not None:
            self.bus.publish("runtime_event", {"channel": self.ch.id, "ts": event["ts"], "event": event})

    def _runtime_record(self, kind, message, *, ts=None, session=None, identity=None, category=None) -> None:
        try:
            events = getattr(self, "_runtime_events", None)
            if events is not None:
                self._runtime_publish(events.record(kind, message,
                    self._session_id if session is None else session, ts, category, identity))
        except Exception:                                    # noqa: BLE001
            pass                                            # 日志失败不改变检测及保存流程

    def _runtime_observe(self, frame: Frame) -> None:
        try:
            events = getattr(self, "_runtime_events", None)
            if events is None:
                return
            stability = self.st.stability or {}
            for event in events.observe(self._session_id, stability, frame.ts, self._session_forced):
                self._runtime_publish(event)
            if (not self._session_id or stability.get("fresh") is not True
                    or stability.get("degraded") is True or self.integration is None):
                return
            status = (self.integration.preparation_status(self.ch.id)
                      if hasattr(self.integration, "preparation_status") else None)
            if (getattr(self, "_deferred", None) is not None
                    and not self.integration.current_passage(self.ch.id)):
                status = "awaiting_passage"
            messages = {"awaiting_passage": "等待重新确认完整空闲与本次车辆身份",
                        "awaiting_plate": "正在等待本次车辆识牌，采集继续进行",
                        "plate_unavailable": "未取得本次车辆车牌，保留观测等待处理",
                        "baseline_required": "缺少本车适用的正式空车基线，暂不能计算载货体积",
                        "ready": "本次车辆测量准备已就绪"}
            if status in messages:
                self._runtime_record("preparation_wait" if status != "ready" else "preparation_ready",
                                     messages[status], ts=frame.ts, category="preparation")
        except Exception:                                    # noqa: BLE001
            pass                                            # 只读准备提示不能反向影响采集

    def _publish_state(self, frame: Frame, force: bool = False) -> None:
        self._runtime_observe(frame)
        now = time.time()
        st = self.st.stability or {}
        # 只有"离散状态"进 key —— 偏差/点数每帧都在变，拿它们比等于没节流
        key = (st.get("state"), bool(st.get("stable")), bool(st.get("settling")),
               self._session_id)
        throttled = (self._state_min_dt > 0 and key == self._state_key
                     and (now - self._state_push_t) < self._state_min_dt)
        if throttled and not force:
            self._state_skipped += 1
            return
        self._state_key = key
        self._state_push_t = now
        self.bus.publish("state", {
            "channel": self.ch.id, "channel_name": self.ch.name,
            "session": self._session_id,
            "n_points": int(frame.n),
            "stability": self.st.stability,
            "plate": self.st.last_plate,
            "reason": self.st.last_reason,
            "fps": self.st.fps,
            "frames": self.st.frames,
            "source": self.source.kind,
            "save_enabled": self.save_enabled,
            # ★ 链路实况必须跟着每次推送走。
            #   之前包率/转速/包每圈/通道错误只在**全量快照**里有，而快照只在
            #   看板刚连上时发一次 —— 于是这几个数字在看板上一挂就是一整班，
            #   雷达中途掉线也看不出来（这正是它们存在的理由）。
            "link": self._link_info(),
            "error": self.st.error,
            "health": self._health_text(),
            "cloud": _downsample_xyz(frame.xyz, 2500),
        })

    def _window_evidence(self, dg, frame) -> Optional[dict]:
        """主帧证据不足时，看看沉降窗口里**其它帧**有没有拿到证据。

        为什么值得做：测量用的是窗口**末帧**，而实测 62 组三连帧里
        **8 组有证据不足的帧、其中 5 组恰好坏在末帧** ——
        也就是说这 5 次测量会报"证据不足"，而同一个窗口里另外两帧
        其实都拿到了完整证据。**整组三帧全坏的是 0 组。**
        分不清这一点，现场就不知道该**重测**还是该**拦截**。

        ▲ **只在主帧判成 `NO_EVIDENCE` 时才跑** —— 那是唯一需要它的场景，
          实测只占约 8%。检测链本身 50~70 ms，平时一次都不多花。
        ▲ 纯诊断：判定结论、落盘内容、`cli/verify.py` 一律不受影响。
        """
        if dg.verdict != "NO_EVIDENCE":
            return None
        others = [f for f in self._settle_frames if f is not frame]
        if not others:
            return None
        steps = []
        for item in others[-self._VOL_EXTRA_MAX:]:
            try:
                other = detect_empty(item, self.calib)
                _ok, _reason, other_dg = evaluate_empty(
                    other, item.n, other.cloud if other else item.xyz, self.dev,
                    ground_z=self.calib.detection.ground_z)
                steps.append({"step": other_dg.step, "verdict": other_dg.verdict})
            except Exception as error:                        # noqa: BLE001
                # 诊断绝不能让一次正常测量失败。
                steps.append({"step": 0, "verdict": "", "error": type(error).__name__})
        evidenced = sum(1 for x in steps if x["verdict"] == "REJECT_LOADED")
        return {"frames": len(steps), "with_evidence": evidenced, "steps": steps,
                "hint": ("同窗口另有 %d 帧拿到了完整证据 —— 多半是瞬时遮挡，"
                         "可重测而非拦截" % evidenced) if evidenced else
                        "同窗口所有帧都证据不足 —— 这一车确实没看清"}

    def _empty_evidence(self, res, frame) -> dict:
        """算"这一份能不能自动当空车基线"的闸门证据。**只出证据，不自我授权。**

        用的点云与判定链**同一份**（`res.cloud`，已裁 ROI、已航向矫正），
        多帧用沉降窗口里那几帧 —— 与 `baseline_quality` 在
        `capture_baseline` 里用的是同一个口径。

        ▲ 任何异常都吞掉并记进 reason：这是**诊断证据**，
          绝不能因为它出错而让一次正常测量失败。
        """
        try:
            cloud = res.cloud if res is not None and res.cloud is not None else frame.xyz
            clouds = [np.asarray(f.xyz, np.float64) for f in self._settle_frames] or None
            return empty_evidence(
                np.asarray(cloud, np.float64), float(self.calib.detection.ground_z),
                clouds=clouds, baseline=self.baseline,
                degraded=bool(self._degraded_measure),
                duplicate=bool(self._session_forced and self._last_save_name))
        except Exception as error:                           # noqa: BLE001
            return {"ok": False, "gates": [], "profile": None,
                    "reason": "闸门证据未算出：" + type(error).__name__}

    def _health_text(self) -> str:
        """给看板的常驻横幅。

        `st.health`（有车判据失效）在车回到空闲时会被清掉 —— 那是对的，
        它量的就是"从不空闲"。但**收尾卡住**是另一回事：车照常来去、
        会话照常结束，只是一辆都测不出来。所以两条分开存、显示时合并，
        别让前一条的清除把后一条也抹掉。
        ▲ 两个前端都只渲染 `health` 这一个字段（`app/web/` 与
          CarPointCloudWebApi 的 `load-radar.js`），所以必须并进来，
          不能只放在 `finalizer.state` 里等人去翻 JSON。
        """
        parts = [self.st.health]
        if self._finalizer is not None:
            parts.append(self._finalizer.alert(self.ch.id))
        parts.append(self._incomplete_alert())
        return "；".join(part for part in parts if part)

    # 残缺帧占比到这一档就开始明显漏车（实测车辆停留 40 s、10 Hz：
    # <=3% 无影响、5% 掉到 84%、10% 只剩 17%）。取 5% 报警，10% 升级措辞。
    _INCOMPLETE_WARN = 0.05
    _INCOMPLETE_BAD = 0.10

    def _incomplete_alert(self) -> str:
        """残缺帧太多时的常驻横幅。

        这是**静默失效**的那一类：残缺帧不参与停稳判定，于是车照常来、
        看板照常显示"运动中"，只是**一辆都测不出来**，而且不重启不恢复
        （`_frame_is_complete` 的包数基准只涨不降，见 `service.py` 该函数
        与 `docs/41` §0b）。没有这条横幅，只有车停够 30 s 才会报一次 warn；
        车道空着的时候一声不吭。
        """
        if len(self._complete_hist) < self._INCOMPLETE_WINDOW // 4:
            return ""                      # 样本太少，不下结论
        ratio = self.st.incomplete_ratio
        if ratio < self._INCOMPLETE_WARN:
            return ""
        base = sorted(self._pkt_hist)[len(self._pkt_hist) // 2] if self._pkt_hist else 0
        tail = ("；持续到这个程度多半是雷达转速变过或网络劣化，"
                "包数基准不会自己降，需重启本通道"
                if ratio >= self._INCOMPLETE_BAD else "")
        return ("残缺帧占比 %.0f%%（最近 %d 帧，包数基准 %d），"
                "这些帧不参与停稳判定，车可能测不出来%s"
                % (ratio * 100, len(self._complete_hist), base, tail))

    def _link_info(self) -> Dict:
        """给看板的链路实况：几个数字，够判断"这台雷达还正常吗"。"""
        info = self.source.info()
        keys = ("source_ip", "packet_rate", "rpm", "frame_mode", "frame_ms",
                "rev_packets", "frames_by_rotation", "frames_by_timeout",
                "pending_packets", "roi_keep", "points_raw", "queue", "error")
        return {k: info[k] for k in keys if k in info}

    # ── 控制 ────────────────────────────────────────────────
    def stop(self) -> None:
        self._stop_evt.set()
        try:
            self.plate.stop()
        except Exception:                                    # noqa: BLE001
            pass

    def close_results(self, timeout=3.) -> bool:
        """须先结束生产者；超时仍持有原结果和连接，不假报成功。"""
        if self.is_alive():
            return False
        closed = self._finalizer is None or self._finalizer.close(timeout)
        self._drain_finalizer()
        return closed

    def snapshot(self, include_history: bool = True, include_frame_integrity: bool = False) -> Dict:
        """通道快照。include_history=False 时不带最近 60 条结果（WebApi 只读白名单字段，用不到）。

        include_frame_integrity=True 才附带最近逐帧决策，普通预览不携带该列表。

        ▲ 整段持有 _control_lock —— 与 _tick 处理每一帧是同一把锁。这里多花的每一毫秒，
          这条车道就晚处理一毫秒的新帧（见 CLAUDE.md「/api/state 很重」）。
        """
        with self._control_lock:
            return self._snapshot_locked(include_history, include_frame_integrity)

    def _snapshot_locked(self, include_history: bool = True, include_frame_integrity: bool = False) -> Dict:
        d = self.st.to_dict()
        # 平台轮询展示当前帧；线程在线不代表雷达仍出帧，断流超过 5 秒须清空。
        frame = self.last_frame
        age = (max(0.0, time.monotonic() - self._last_frame_monotonic)
               if frame is not None else None)
        d["frame_age_sec"] = age
        d["cloud"] = (_downsample_xyz(frame.xyz, 2500)
                      if d["online"] and frame is not None and age <= 5.0 else [])
        # 客户页只按当前过车的持久测量编号取结果，不能把 last_result 当作本车。
        result = self.st.last_result
        current = (d["online"] and frame is not None and age <= 5.0
                   and self._session_id != 0 and result.get("session") == self._session_id
                   and bool(self._current_result_token)
                   and result.get("origin_token") == self._current_result_token)
        if current and self.integration is not None:
            association = result.get("integration", {})
            passage = self.integration.current_passage(self.ch.id)
            current = (bool(passage) and association.get("status") != "quarantined"
                       and association.get("passageId") == passage)
        d["current_result"] = ({"measurement_id": result["measurement_id"]}
                               if current and result.get("measurement_id") else None)
        d["preparation"] = (self.integration.preparation_status(self.ch.id)
                            if self.integration is not None and hasattr(self.integration, "preparation_status")
                            and self.integration.current_passage(self.ch.id) else None)
        d["volume_preparation"] = result.get("volume_preparation") if current else None
        d["pending_compute"] = self._deferred.snapshot(self.ch.id) if getattr(self, "_deferred", None) is not None else None
        d["source"] = self.source.info()
        d["plate"] = self.plate.info()
        d["calib_path"] = self.calib.path
        d["devices_path"] = self.dev.path
        d["angle_ok"] = bool(self.angle.loaded and self.angle.hash_ok)
        d["angle_note"] = self.angle.error
        # ★ 按车牌存之后，"这条道有没有基线"不再有意义 —— 基线属于车，不属于道。
        #   这里报：刚测的那辆车有没有基线、是哪个车牌的，以及共用目录里一共
        #   标定过几辆车。旧的车道级文件还在的话单独提醒（它不会再被使用）。
        d["baseline"] = bool(self.baseline and self.baseline.n)
        d["baseline_plate"] = self._baseline_plate or ""
        d["baseline_plates"] = len(self.known_baseline_plates())
        d["legacy_lane_baseline"] = os.path.exists(self._legacy_base_path)
        # 最近结果每条约 6 KB，60 条的深拷贝是整份快照里最贵的一步（基准约 9 ms/道）。
        history = copy.deepcopy(list(self.history)[-60:]) if include_history else None
        events = getattr(self, "_runtime_events", None)
        try:
            d["runtime_events"] = events.snapshot() if events is not None else []
        except Exception:                                    # noqa: BLE001
            d["runtime_events"] = []                         # 日志故障不影响点云快照
        # ★ ROI 发给看板：点云视图用它做**固定**的取景框。
        #   之前看板每帧按当前点云的包围盒自适应缩放，于是多一个噪点、少一排点，
        #   整片点云就整体平移/缩放一下 —— 静止的场景看着也在"抖"。
        d["roi"] = ([list(self.calib.roi[0]), list(self.calib.roi[1])]
                    if self.calib.roi else None)
        d["state_push_hz"] = round(1.0 / self._state_min_dt, 2) if self._state_min_dt else 0.0
        d["state_skipped"] = self._state_skipped
        # ★ 看板的"存盘开/关"徽标读这个字段。它之前只在 state 事件里有，
        #   而 SSE 断了退回轮询时，applySnapshot 会用快照**整体覆盖**通道对象 ——
        #   于是徽标会一直显示"存盘关"，现场会以为数据没在存。
        d["save_enabled"] = bool(self.save_enabled)
        d["health"] = self._health_text()
        if include_frame_integrity:
            d['frame_integrity'] = {
                'capture_instance_id': self._capture_instance_id,
                'complete_ratio': self._COMPLETE_RATIO,
                'reference_history': list(self._pkt_hist),
                'reference_packets': (sorted(self._pkt_hist)[len(self._pkt_hist) // 2]
                                      if self._pkt_hist else None),
                'frames': list(self._frame_integrity_history),
            }
        d["finalizer"] = self._finalizer.status(self.ch.id) if self._finalizer else {"state": "idle"}
        # 只深拷贝可能与活动状态共享引用的小字段。点云是 _downsample_xyz 新建的列表、
        # history 上面已拷贝过 —— 原来对整份 d 再 deepcopy 一次，等于把这两块大数据再复制一遍
        # （基准约 21 ms/道，且全在帧锁内）。返回值仍与工作线程的状态完全不共享引用。
        cloud = d.pop("cloud")
        d = copy.deepcopy(d)
        d["cloud"] = cloud
        if history is not None:
            d["history"] = history
        return d

    def history_snapshot(self, limit: int = 100) -> List[Dict]:
        with self._control_lock:
            return copy.deepcopy(list(self.history)[-limit:])

    def frame_snapshot(self) -> Optional[Frame]:
        with self._control_lock:
            return copy.deepcopy(self.last_frame)

    def set_loaded_demo(self, loaded: bool) -> bool:
        """
        仿真源专用：切换空车/载货场景。

        **不要在这里重置检测器** —— 交给 _tick 按帧的 gen 惰性重置，
        否则会把"旧场景的最后一帧"误当成新参考帧。原因见 _tick 的注释。
        """
        with self._control_lock:
            fn = getattr(self.source, "set_loaded", None)
            if not callable(fn) or self._stop_evt.is_set():
                return False
            fn(loaded)
            self._demo_observation_changed()
            return True

    def set_vehicle_demo(self, loaded: Optional[bool] = None,
                         y_offset: Optional[float] = None) -> bool:
        """
        仿真源专用：手动摆一次车辆场景（换车 / 挪位置）。

        与 set_loaded_demo 的区别：这个走 RadarSimSource.set_vehicle，
        可以同时把车挪到指定 y（演示/排查时想看"车只进来一半"之类）。
        参数语义见 radar_source.RadarSimSource.set_vehicle。
        """
        with self._control_lock:
            fn = getattr(self.source, "set_vehicle", None)
            if not callable(fn) or self._stop_evt.is_set():
                return False
            if fn(loaded=loaded, y_offset=y_offset) is not False:
                self._demo_observation_changed()
            return True

    def _demo_observation_changed(self) -> None:
        # 不在此建立新场景参考帧；但控制完成后旧许可立即失效，不能人工取旧基线。
        self.stability.invalidate("演示场景变化，等待新观测")
        self._discard_observation_window(vehicle_changed=True)
        self.st.stability.update(fresh=False, state=self.stability.state,
                                 state_cn=StabilityState.CN[self.stability.state],
                                 reason="演示场景变化，等待新观测")

    def capture_baseline(self, plate: str = "") -> Tuple[bool, str]:
        """
        把**当前这一帧**当作**这辆车**的空车基线存下来（按车牌存，体积计算必须有它）。

        现场用法：空车停进检测位 → 看板点"标定空车基线" → 之后这辆车装货回来，
        **无论走哪条车道**都能按车牌查到这份基线出体积。
        故意不自动触发：基线一旦存错（例如把载货车当空车），之后所有体积都错，
        而且不会报错。必须由人确认"这是一辆空车"。

        plate：操作员明确给出的车牌。不给就**现读一次**（车正停在检测位上）。
        ▲ 识别不到车牌就拒绝标定 —— 绝不能存成「未知车牌」：那样所有认不出牌的
          车会共用这一份基线，恰好重新造出"拿别的车的车厢盒算体积"那个错。
        """
        if self.ch.id != "CH4":
            return False, "空车基线只能在CH4建立"
        with self._control_lock:
            if self.integration is not None:
                return False, "已由业务后台接管，请在后台发起人工空车候选采集"
            f = copy.deepcopy(self.last_frame)
            if f is None or f.n < 50:
                return False, "当前没有可用的点云帧（雷达在出数吗？）"
            if not self._baseline_capture_current(self._observation_epoch):
                return False, "需要当前连续完整的新帧证明已停稳，才能人工标定"
            epoch = self._observation_epoch
            with self._recent_lock:
                recent = [x.xyz.copy() for x in self._recent_frames]
        # ★ 先确认这几帧**确实是同一个静止场景**（2026-09-17，docs/13 §18）。
        #   2026-09-15 现场那两次空车标定，三帧互相关只有 0.799 / 0.865
        #   （同批载货是 0.986 / 1.000）—— 车其实还在动或场景在变，
        #   拿它建基线会让同一辆车的体积差 4%，而且**事后完全看不出来**。
        #   必须至少三帧完整、停稳且仍新鲜；没有证据时人工操作也不能跳过。
        if len(recent) < 3:
            return False, "人工标定至少需要连续三帧完整且停稳的新观测"
        q = baseline_quality(recent)
        if q["n"] >= 2 and not q["ok"]:
            return False, ("这几帧不是同一个静止场景（帧间互相关 %.3f < 0.95）——"
                           "等车彻底停稳、场地清空后再标定。"
                           "硬存下来的话，之后这辆车的体积会一直偏，而且不报错。"
                           % q["r_min"])
        if not plate:
            # 现读一次：车正停在检测位上，这时候的识别结果就是这辆车。
            # 不持通道锁 —— 读牌可能要等相机/后台最多 f3_timeout_ms。
            try:
                got = self.plate.read()
                plate = got.plate if got.ok else ""
            except Exception:                                 # noqa: BLE001
                plate = ""
        plate = (plate or "").strip()
        if not plate or plate == EMPTY_PLATE:
            return False, ("识别不到车牌，不能标定 —— 空车基线按车牌存，存成「%s」"
                           "会让所有认不出牌的车共用这一份，正是要修掉的那个错。"
                           "请确认相机能读到牌，或在标定时直接指定车牌。" % EMPTY_PLATE)
        base_path = self.baseline_path_for(plate)
        res = detect_empty(f, self.calib)
        if res is None:
            return False, "六面拟合失败，这一帧不能当基线"
        fl = res.plane("floor")
        if not fl.valid:
            return False, (f"底面不够水平（|法向z|={abs(fl.c):.4f} < "
                           f"{self.calib.floor.normal_z_threshold}），这一帧不能当基线")
        b = EmptyBaseline.from_empty_result(
            res, f"{time.strftime('%Y-%m-%d %H:%M:%S')} 看板标定 {self.ch.id}")
        if b is None:
            return False, "点云过少，无法建基线"
        b.source_channel = "CH4"
        staged_path = None
        try:
            # 拟合/压缩/写临时文件不持通道锁；最后短锁内核对原观测并原子发布。
            ensure_dir(os.path.dirname(base_path))
            fd, staged_path = tempfile.mkstemp(prefix=".baseline-", suffix=".npz",
                                             dir=os.path.dirname(base_path))
            os.close(fd)
            b.save(staged_path)
            with self._control_lock:
                if not self._baseline_capture_current(epoch):
                    return False, "标定期间观测已变化，请重新确认当前空车"
                os.replace(staged_path, base_path)
                key = safe_dir_name(plate)
                self._baseline_cache[key] = (os.path.getmtime(base_path), b)
                self.baseline, self._baseline_plate = b, key
                self.bus.publish("warn", {"channel": self.ch.id,
                    "message": f"已标定空车基线：底板 z={b.floor_z:.3f} m，{b.n} 点"})
        except OSError as e:
            return False, f"写入失败 {base_path}: {e}"
        finally:
            if staged_path is not None and os.path.exists(staged_path):
                os.unlink(staged_path)
        return True, (f"已保存 {plate} 的空车基线 {base_path}（底板 z={b.floor_z:.3f} m，{b.n} 点）")

    # ── 按车牌取空车基线（2026-09-23）──────────────────────────
    def baseline_path_for(self, plate: str) -> str:
        """这辆车的空车基线文件路径。车牌走 `safe_dir_name` 规范化 ——
        它已经处理了非法字符 / 长度 / NFC / 大小写折叠，做文件名正合适，
        顺带保证"同一辆车的不同写法"不会各存一份。"""
        return baseline_file(self._baseline_dir, plate)

    def _resolve_baseline(self, plate: str):
        """按车牌取这辆车自己的基线。**取不到就是 None，绝不退回别的车的。**

        ▲ 这一条是整个改造的要点：宁可"这辆车没有基线、不出体积"，
          也不能拿另一辆车的车厢盒算出一个**不报错的错数**。
          实测证据见构造函数里的说明（辆辆车厢不同，dy 差 2 m 体积差 18%）。
        """
        key = safe_dir_name(plate)
        if not plate or plate == EMPTY_PLATE:
            return None, ""
        # ★★ 缓存按**文件修改时间**校验，且**绝不缓存"没有"**（2026-09-23 跨车道实测）。
        #   第一版把"这个车牌查不到"也缓存成 None，结果：CH4 在 CH2 标定**之前**
        #   就看见过这辆车（查一次、缓存 None），CH2 随后写出基线，CH4 却永远
        #   返回缓存里的 None —— 不重启就查不到。这恰好就是现场的样子：
        #   基线目录四条道共用，谁写的、什么时候写的，别的道都得立刻看得见。
        #   查不到时只花一次 os.stat，没必要缓存；查到了才按 mtime 缓存点云。
        path = self.baseline_path_for(plate)
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            self._baseline_cache.pop(key, None)
            return None, ""
        hit = self._baseline_cache.get(key)
        if hit is not None and hit[0] == mtime:
            return hit[1], (key if hit[1] is not None else "")
        b = EmptyBaseline.load(path)
        if b is not None and b.source_channel != "CH4":
            b = None
        if b is not None and not b.n:
            b = None
        self._baseline_cache[key] = (mtime, b)
        return b, (key if b is not None else "")

    def known_baseline_plates(self) -> List[str]:
        """这个车道能看到的、已标定过空车基线的车牌（按文件名列，不加载点云）。

        ▲ 基线目录是**所有车道共用**的（`data/in`）——这正是按车牌存的意义：
          空车在 CH2 标定，载重走 CH4 也查得到。
        """
        try:
            names = os.listdir(self._baseline_dir)
        except OSError:
            return []
        out = []
        for name in names:
            if not name.endswith(BASELINE_SUFFIX):
                continue
            stem = name[: -len(BASELINE_SUFFIX)]
            if _LEGACY_CHANNEL_BASELINE.match(stem):
                continue                     # 旧的车道级文件（CH1.baseline.npz），不是车牌
            out.append(stem)
        return sorted(out)

    def _baseline_capture_current(self, epoch: int) -> bool:
        """仅持通道锁调用，计算期间断流、运动、重置或停止均撤销人工许可。"""
        return (not self._stop_evt.is_set() and epoch == self._observation_epoch
                and bool(self.st.stability.get("stable"))
                and time.monotonic() - self._last_frame_monotonic <= self.stability.stationary_max_gap_sec)

    def force_new_session(self, scene_changed: bool = False) -> bool:
        """
        结束当前会话、开一个新的。

        用途：
          · 演示时切换场景（否则"车一直没走"，会话不结束，不会再检测）
          · 现场排查时手动重测一次
        会同时重置停稳检测器 —— 让参考帧重新建立，等价于"来了一辆新车"。

        scene_changed：本次翻篇是因为**数据源场景代次变了**（演示/回放切换，
        车必然不是上一辆）—— 此时必须同时清掉 `_session_forced` /
        `_last_save_name`，否则新场景第一辆车会被错标 `duplicate_of`
        指向旧场景那份，下游按此去重会错丢真实车次（两轮审查 M6）。
        人工翻篇**不**传它：点按钮的人可能只是想重测当前这辆车，
        那种情况下重复标记恰恰是对的 —— 歧义留给人工，机器不猜。
        """
        with self._control_lock:
            if self._stop_evt.is_set():
                return False
            self._force_new_session_locked(scene_changed=scene_changed)
            return True

    def _force_new_session_locked(self, scene_changed: bool = False) -> None:
        self._enter_reset()
        self._exit_reset()
        self._session_id = 0
        self._detected_this_session = 0
        self._saved_this_session = 0
        self._warned = False
        self._triggers.clear()
        if scene_changed:
            # 只有「回到过空闲」或「场景切换」才能证明车换了（M6，见上）。
            self._session_forced = False
            self._last_save_name = ""
            self._local_passage = None
        self._discard_observation_window(vehicle_changed=True)
        self.stability.reset()
        self.st.stability.update(state=StabilityState.IDLE, state_cn=StabilityState.CN[StabilityState.IDLE],
                                 fresh=False, speed_mps=None, drift_m=0.0,
                                 shape_speed_mps=None, yaw_speed_mps=None, motion_observable=False,
                                  motion_pending=False, instant_speed_mps=0., jitter_confirmed=False,
                                  reason="重新等待连续观测")


def _downsample_xyz(xyz: np.ndarray, limit: int) -> List[List[float]]:
    """降采样点云给前端。保留原始量级（mm 级精度足够画图）。

    等间隔取点（不是随机采样）：随机采样会让**静止场景**每帧取到不同的子集，
    看板上就成了一片乱闪 —— 现场反馈过的"点云看不清"里也有这一份。
    """
    if xyz is None or len(xyz) == 0:
        return []
    n = len(xyz)
    if n > limit:
        idx = np.linspace(0, n - 1, limit).astype(np.int64)
        xyz = xyz[idx]
    # 向量化取整再一次性转成 Python 列表：逐点 round(float(v),3) 在 2500 点上
    # 要做 7500 次 Python 级调用，而这段代码跑在**检测线程**里。
    return np.round(np.asarray(xyz, np.float64), 3).tolist()
