"""车道事件发布订阅；与 HTTP、日志和设备实现独立。"""
import copy
import time
import threading
from collections import deque
from typing import Deque, Dict, List, Tuple

class EventBus:
    """
    极简发布/订阅。看板（SSE）与日志都挂在这上面。

    刻意不用线程安全的队列库：订阅者自己保证消费速度，
    服务端只负责把事件丢进每个订阅者的有限队列，满了就丢最旧的 ——
    看板卡住绝不能反过来拖慢检测主循环。
    """

    def __init__(self, capacity: int = 200):
        self.capacity = capacity
        # (用途标签, 队列)。标签只影响统计：看板要显示"几个看板连着"，
        # 而控制台日志、自检脚本这类订阅者不该被算成看板。
        self._subs: List[Tuple[str, Deque]] = []
        self._lock = threading.Lock()
        self._seq = 0

    def subscribe(self, tag: str = "sse") -> Deque:
        q: Deque = deque(maxlen=self.capacity)
        with self._lock:
            self._subs.append((tag, q))
        return q

    def unsubscribe(self, q: Deque) -> None:
        with self._lock:
            self._subs = [(t, x) for t, x in self._subs if x is not q]

    def publish(self, kind: str, payload: Dict) -> Dict:
        # 发布后生产者仍会更新状态/归档状态，事件不得保留其可变引用。
        # ★ 深拷贝放在**锁外**（2026-09-23）：4 条车道共用这一把锁，而结果事件带着点云，
        #   在锁里拷贝会让别的车道发状态时排队、拖慢它们的检测线程。拷贝发生在发布调用之内，
        #   仍是"发布那一刻"的快照；序号、时间戳与入队仍在锁内，事件顺序不变。
        body = copy.deepcopy(payload)
        with self._lock:
            self._seq += 1
            ev = {"seq": self._seq, "kind": kind, "ts": time.time(), **body}
            for _tag, q in self._subs:
                q.append(ev)
        return ev

    @property
    def subscribers(self) -> int:
        """看板（SSE）连接数 —— 控制台日志之类的内部订阅者不计入。"""
        with self._lock:
            return sum(1 for t, _q in self._subs if t == "sse")

    @property
    def all_subscribers(self) -> int:
        with self._lock:
            return len(self._subs)
