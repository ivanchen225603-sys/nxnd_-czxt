"""车道状态和停稳摘要；不访问设备、网络或持久化。"""
import copy
from collections import Counter
from dataclasses import dataclass, field
from typing import Dict

@dataclass
class ChannelState:
    id: str
    name: str
    mode: str = ""
    online: bool = False
    source_kind: str = ""
    fps: float = 0.0
    frames: int = 0
    stability: Dict = field(default_factory=dict)
    last_reason: str = ""
    last_verdict_ok: bool = False
    last_plate: str = ""
    last_diag: Dict = field(default_factory=dict)
    last_result: Dict = field(default_factory=dict)
    last_saved: str = ""
    saved_count: int = 0
    session: int = 0
    updated: float = 0.0
    error: str = ""
    # ★ 常驻健康告警（不是一次性日志）。典型场景：通道一直判「有车」却从不回到
    #   空闲 —— 说明有车判据失效了，系统此时是"瞎"的：真来的车会被静默跳过。
    #   2026-09-13 现场就这样跑了 10 分钟，一声不吭。
    health: str = ""
    # ★ 残缺帧占比（最近若干帧里被 `_frame_is_complete` 判为残缺的比例）。
    #   在此之前这个判定**只用不记**：残缺帧 -> trusted=False ->
    #   StabilityDetector.invalidate() -> 锚点清零 -> 攒不满连续停稳 ->
    #   这辆车测不出来，而看板上只表现为"一直运动中"。
    #   实测（车辆停留 40 s、10 Hz）：占比 <=3% 无影响、5% 掉到 84%、
    #   **10% 只剩 17% 的车测得成**。所以它必须能被直接看到，
    #   不能只靠抓包离线算（docs/41 §0b）。
    incomplete_ratio: float = 0.0
    incomplete_frames: int = 0
    # 被内存预算挤掉的整帧累计数（见 ChannelWorker._note_raw_dropped）
    raw_dropped: int = 0
    backend: str = "cpu"

    def to_dict(self) -> Dict:
        d = copy.deepcopy(self.__dict__)
        return d


def trigger_digest(tally, top: int = 4) -> str:
    """把停稳触发直方图压成一行。

    现场报「行驶状态每秒都在变」时，看板和日志给的一直是旧的 `deviation`——
    而它按设计**早就不单独决定停稳**了，等于报了个不拦人的数。
    这个摘要给的是真正拦住它的那几条（yaw_speed / immediate / net ...）。
    """
    return " ".join("%s x%d" % (name, n)
                    for name, n in Counter(tally).most_common(top) if n)
