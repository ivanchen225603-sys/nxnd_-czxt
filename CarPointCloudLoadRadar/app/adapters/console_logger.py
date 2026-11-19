"""事件到控制台的日志适配器。"""
from __future__ import annotations
import sys
import time
import threading
from typing import Dict, TYPE_CHECKING
from ..application.events import EventBus
from ..application.channel_state import trigger_digest

if TYPE_CHECKING:
    from ..service import LidarService

class ConsoleLogger(threading.Thread):
    """把事件总线上的关键事件打到 stdout。

    为什么需要它：systemd 把 stdout 重定向到 logs/service.log，桌面版重定向到
    logs/desktop.log —— 这是现场**唯一**的历史记录。在此之前服务起来后除了启动横幅
    什么都不打，日志文件里永远只有开头那几行，出了事只能靠看板（而看板只存内存）。

    刻意不打 state（默认 2Hz × 4 路）和 demo_phase：日志要能看，不是流水账。
    """

    KINDS = ("result", "result_pending", "result_history", "result_archive", "runtime_event", "warn", "error", "plate_miss", "skip_save",
             "session_start", "session_end",
             "channel_online", "channel_offline", "service_start", "service_stop")

    def __init__(self, bus: EventBus, stream=None, kinds=None):
        super().__init__(daemon=True, name="console-log")
        self.bus = bus
        # ★ 不要在这里把 sys.stdout 定死。
        #   用 pythonw.exe / 打包成无控制台窗口的 exe 启动时，sys.stdout 可能是
        #   **None**；桌面版又会在启动过程中把 sys.stdout 换成落日志的 _Tee。
        #   所以只记住"调用方指定的流"，真正写的时候再取一次当前的 sys.stdout。
        self.stream = stream
        self.kinds = set(kinds or self.KINDS)
        self.q = bus.subscribe(tag="console")
        self._stop_evt = threading.Event()
        self.printed = 0
        self.dropped = 0            # 没有可用输出流而丢掉的条数

    def _out(self):
        """当前可用的输出流；没有就返回 None（写日志绝不能把服务拖垮）。"""
        st = self.stream if self.stream is not None else sys.stdout
        return st if (st is not None and hasattr(st, "write")) else None

    @staticmethod
    def format(ev: Dict) -> str:
        k = ev.get("kind", "")
        ch = ev.get("channel", "")
        if k == "runtime_event":
            event = ev.get("event") or {}
            return f"{ch} 会话 #{event.get('session', 0)} {event.get('message', '')}"
        if k == "result":
            return (f"{ch} {ev.get('plate', '')} "
                    f"{'空车' if ev.get('empty') else '非空'} "
                    f"{ev.get('reason', '')} 点数={ev.get('n_points', 0)} "
                    f"偏差={ev.get('deviation', 0):.4f} 用时={ev.get('elapsed_ms', 0)}ms"
                    + (f" 存盘={ev['saved']}" if ev.get("saved") else
                       " 已持久化，文件待导出" if ev.get("archive_status") == "export_pending" else " 未启用保存"))
        if k == "result_pending":
            return f"{ch} 测量 {ev.get('measurement_id', '')} 已计算，等待持久化"
        if k == "result_history":
            return f"{ch} 历史测量 {ev.get('measurement_id', '')} 补交完成，不替换当前车辆"
        if k == "result_archive":
            return f"{ch} 测量 {ev.get('measurement_id', '')} 成套存盘={ev.get('saved', '')}"
        if k in ("warn", "service_stop"):
            return f"{ch} {ev.get('message', '')}".strip()
        if k == "error":
            return f"{ch} {ev.get('error', '')}"
        if k == "plate_miss":
            return f"{ch} 未取到车牌: {ev.get('error', '')}"
        if k == "skip_save":
            return f"{ch} 点数不足未存盘: {ev.get('n_points')} < {ev.get('need')}"
        if k == "session_start":
            return f"{ch} 会话 #{ev.get('session')} 开始，点数 {ev.get('n_points')}"
        if k == "session_end":
            digest = trigger_digest(ev.get("triggers") or {})
            return (f"{ch} 会话 #{ev.get('session')} 结束，存盘 {ev.get('saved')} 次"
                    + (f"，拦在 {digest}" if digest else ""))
        if k in ("channel_online", "channel_offline"):
            return f"{ch} {ev.get('source', '')}"
        if k == "service_start":
            return f"通道 {ev.get('channels')} 接收点 {ev.get('hub') or '(无)'}"
        return ""

    def run(self) -> None:
        while not self._stop_evt.is_set():
            try:
                ev = self.q.popleft()
            except IndexError:
                self._stop_evt.wait(0.2)
                continue
            if ev.get("kind") not in self.kinds:
                continue
            out = self._out()
            if out is None:
                self.dropped += 1
                continue
            try:
                t = time.strftime("%m-%d %H:%M:%S", time.localtime(ev.get("ts") or None))
                out.write(f"{t} [{ev['kind']}] {self.format(ev)}" + "\n")
                out.flush()
                self.printed += 1
            except Exception:                     # noqa: BLE001
                # 流被关掉、编码报错、_Tee 少个方法…… 打日志绝不能拖垮检测，
                # 也绝不能让这条线程带着异常死掉（死了就再没有现场记录）。
                self.dropped += 1

    def stop(self) -> None:
        self._stop_evt.set()
        self.bus.unsubscribe(self.q)


def attach_console_logger(svc: "LidarService", stream=None) -> ConsoleLogger:
    """给服务挂一条控制台事件日志，返回它（调用方负责 stop()）。"""
    log = ConsoleLogger(svc.bus, stream=stream)
    log.start()
    return log
