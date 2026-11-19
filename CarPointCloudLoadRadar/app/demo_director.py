#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
demo_director.py — 无硬件演示编排器（多路雷达"车队流水"）

────────────────────────────────────────────────────────────────────────
它解决什么问题
────────────────────────────────────────────────────────────────────────

接真雷达之前要给领导/客户看效果，需要四路通道**持续地**有车进、有车出、
有结论。手动点"演示：载货"按钮只能做一次：
    切场景 → 停稳 → 检测一次 → 会话不结束 → 之后再没有任何动静。
领导坐下来看 30 秒，屏幕上就是四张一动不动的卡片。

本模块让每一条仿真通道按一条**时间线**自己演下去：

    空场等待(5s) → 车辆驶入(6s) → 停稳测量(13s) → 车辆驶离(6s) → 循环
                                                      周期 30s

四路通道的时间线**互相错开**（各差 1/4 周期），所以任何时刻屏幕上都同时
存在"正在驶入 / 正在停稳测量 / 刚刚出结果 / 正在驶离"的不同状态 ——
看起来就是四个互不相干的车道各自的真实节奏，而不是四张同步的幻灯片。

每一圈还会**交替**空车 / 载货（见 _loaded_for），所以领导能看到同一条车道
上一趟判「空车」、下一趟判「底面过高，疑似载物」。

────────────────────────────────────────────────────────────────────────
依赖的数据源能力（只有仿真源具备，真雷达自动跳过）
────────────────────────────────────────────────────────────────────────

    source.set_vehicle(loaded=bool, y_offset=float)

    语义（详见 radar_source.RadarSimSource.set_vehicle）：
        loaded    变了 = 换了一辆车 → 自增 generation → 上层重建停稳参考帧
        y_offset  变了 = 同一辆车在移动 → 不动 generation，
                          由停稳检测按质心差自己判「运动中」/「已停稳」

"驶入/驶离"就是让 y_offset 从 OUT_Y 线性走到 0（或反向）。
OUT_Y = 20 m 不是拍脑袋定的 —— logs/_offset_calib.txt 实测：
    空车在 y_offset ≥ 8 m 时"高于地面的点数"掉到 2407 < vehicle_threshold(3000)
    → 停稳检测正确回到 IDLE（车不在场）；
    y_offset ≥ 14 m 时该计数精确为 0。
取 20 m 留足余量（含 ±10 cm 的定位/噪声）。

────────────────────────────────────────────────────────────────────────
与手动按钮的关系
────────────────────────────────────────────────────────────────────────

看板上的「演示：空车/载货」「强制重测」是**手动接管**用的。
一旦点了它们，编排器会**自动暂停**（并在事件日志里说明原因），
否则编排器下一秒就会把场景覆盖回去，让人以为按钮坏了。
要恢复自动演示，点看板上的「自动演示」按钮即可。
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

# 车辆"不在场"的 y 偏移（米）。实测依据见模块头部说明。
OUT_Y = 20.0


@dataclass(frozen=True)
class Phase:
    """时间线的一段。y 在该段内从 y_from 线性走到 y_to。"""
    name: str
    dur: float
    y_from: float
    y_to: float
    new_vehicle: bool = False      # 本段开始处是否"换车"（唯一写 loaded 的地方）

    def y_at(self, frac: float) -> float:
        f = 0.0 if frac < 0.0 else (1.0 if frac > 1.0 else frac)
        return self.y_from + (self.y_to - self.y_from) * f


# 一个完整周期 = 30 s。刻意让"停稳测量"最长（13s）：
# 停稳检测要 2 帧确认 + settle_sec(3s) 沉降窗口才出结论，得留够时间。
PHASES: Tuple[Phase, ...] = (
    Phase("空场等待", 5.0, OUT_Y, OUT_Y),
    Phase("车辆驶入", 6.0, OUT_Y, 0.0, new_vehicle=True),
    Phase("停稳测量", 13.0, 0.0, 0.0),
    Phase("车辆驶离", 6.0, 0.0, OUT_Y),
)

CYCLE_SEC = sum(p.dur for p in PHASES)


class DemoDirector:
    """
    驱动所有"可编排"通道按时间线自动演下去。

    用法：
        d = DemoDirector(svc.workers, bus=svc.bus)
        d.start()          # 起一个后台线程，每 tick 秒推一次车辆位置
        ...
        d.stop()

    线程安全：set_enabled()/pause()/resume() 随时可调；
    status() 是纯读，给看板用。
    """

    name = "demo-director"

    def __init__(self, workers: Sequence, bus=None, tick: float = 0.25,
                 enabled: bool = True):
        self.bus = bus
        self.tick = max(0.05, float(tick))
        self._enabled = bool(enabled)
        self._paused_reason = ""
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._t0 = 0.0
        self._state: Dict[str, Dict] = {}
        self._last_phase: Dict[str, str] = {}

        # 只挑"能编排"的通道（仿真源才有 set_vehicle；真雷达/回放自动跳过）
        self.targets: List[Tuple[str, object, float]] = []
        cands = []
        for w in workers:
            fn = getattr(getattr(w, "source", None), "set_vehicle", None)
            if callable(fn):
                cands.append(w)
        n = max(1, len(cands))
        for i, w in enumerate(cands):
            # 互相错开 1/n 个周期：屏幕上同时呈现各个阶段
            off = CYCLE_SEC / n * i
            self.targets.append((w.ch.id, w, off))
            self._state[w.ch.id] = {
                "phase": PHASES[0].name, "cycle": 0, "progress": 0.0,
                "y_offset": OUT_Y, "loaded": False, "phase_left": 0.0,
            }

    # ── 对外状态 ────────────────────────────────────────────
    @property
    def enabled(self) -> bool:
        with self._lock:
            return self._enabled and not self._paused_reason

    def set_enabled(self, on: bool) -> None:
        with self._lock:
            self._enabled = bool(on)
            if self._enabled:
                self._paused_reason = ""
        self._announce("自动演示已" + ("开启" if on else "关闭"))

    def pause(self, reason: str = "") -> None:
        """手动接管时调用 —— 让编排器立刻停手，别把用户刚设的场景覆盖掉。"""
        with self._lock:
            if self._paused_reason:
                return
            self._paused_reason = reason or "手动接管"
        self._announce("自动演示已暂停（" + (reason or "手动接管") +
                       "）。要恢复请点「自动演示」。")

    def resume(self) -> None:
        with self._lock:
            self._paused_reason = ""
        self._announce("自动演示已恢复")

    def _announce(self, msg: str) -> None:
        if self.bus is None:
            return
        self.bus.publish("warn", {"channel": "", "message": msg})

    def status(self) -> Dict:
        with self._lock:
            en = self._enabled and not self._paused_reason
            d = {
                "auto": en,
                "switched_on": self._enabled,
                "paused_reason": self._paused_reason,
                "tick": self.tick,
                "cycle_sec": round(CYCLE_SEC, 1),
                "phases": [{"name": p.name, "dur": p.dur} for p in PHASES],
                "out_y": OUT_Y,
                "n_channels": len(self.targets),
                "uptime": round(time.time() - self._t0, 1) if self._t0 else 0.0,
                "channels": {k: dict(v) for k, v in self._state.items()},
            }
        return d

    # ── 生命周期 ────────────────────────────────────────────
    def start(self) -> None:
        if not self.targets:
            return
        self._t0 = time.time()
        self._stop_evt.clear()
        self._apply(force=True)          # 先摆好初始姿态，别等第一个 tick
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name=self.name)
        self._thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=1.5)
        self._thread = None

    def _run(self) -> None:
        while not self._stop_evt.wait(self.tick):
            try:
                self._apply()
            except Exception:                    # noqa: BLE001 —— 演示编排绝不能拖垮服务
                pass

    # ── 核心 ────────────────────────────────────────────────
    @staticmethod
    def _loaded_for(cycle: int, idx: int) -> bool:
        """
        第 cycle 圈、第 idx 路通道：空车 / 载货交替。

        用 (cycle + idx) 的奇偶而不是只看 cycle ——
        这样同一时刻 CH1/CH3 与 CH2/CH4 的载货状态**相反**，
        看板上天然是"有的判空车、有的判载货"的混合画面，
        而不是四路齐刷刷一起空、一起满（那看起来像假的）。
        """
        return ((cycle + idx) % 2) == 1

    def _phase_at(self, off: float, t: float):
        pos = (t + off) % CYCLE_SEC
        cycle = int((t + off) // CYCLE_SEC)
        acc = 0.0
        for p in PHASES:
            if pos < acc + p.dur or p is PHASES[-1]:
                frac = (pos - acc) / p.dur if p.dur > 0 else 1.0
                frac = min(1.0, max(0.0, frac))
                return p, frac, cycle, (acc + p.dur - pos)
            acc += p.dur
        p = PHASES[-1]
        return p, 1.0, cycle, 0.0

    def _apply(self, force: bool = False) -> None:
        with self._lock:
            if not (self._enabled and not self._paused_reason) and not force:
                return
        t = time.time() - self._t0
        for idx, (cid, w, off) in enumerate(self.targets):
            p, frac, cycle, left = self._phase_at(off, t)
            y = p.y_at(frac)
            loaded = self._loaded_for(cycle, idx)
            with self._lock:
                # pause 返回前先收完在途自动写；锁序固定 Director -> Worker。
                if self._stop_evt.is_set() or (not force and not (self._enabled and not self._paused_reason)):
                    return
                try:
                    if not w.set_vehicle_demo(loaded=loaded, y_offset=round(y, 3)):
                        continue
                except Exception:                    # noqa: BLE001
                    continue
                st = self._state.setdefault(cid, {})
                st.update({"phase": p.name, "cycle": cycle,
                           "progress": round(frac, 3),
                           "y_offset": round(y, 3), "loaded": bool(loaded),
                           "phase_left": round(left, 1)})
            # 阶段切换时播一条事件，看板事件日志会跟着"演"
            if self._last_phase.get(cid) != p.name:
                self._last_phase[cid] = p.name
                if self.bus is not None:
                    self.bus.publish("demo_phase", {
                        "channel": cid, "phase": p.name,
                        "cycle": cycle, "loaded": bool(loaded),
                        "y_offset": round(y, 2)})

    def snapshot_row(self, cid: str) -> Dict:
        with self._lock:
            return dict(self._state.get(cid, {}))
