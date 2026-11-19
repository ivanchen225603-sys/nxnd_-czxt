# -*- coding: utf-8 -*-
"""通道控制的FIFO可重入锁：防止读帧循环连续抢锁饿死已排队的请求。"""
from collections import deque
import math
import threading
from time import monotonic


class FairRLock:
    """保持重入语义，非重入申请按入队顺序取得所有权。"""

    def __init__(self) -> None:
        self._condition = threading.Condition(threading.Lock())
        self._queue = deque()
        self._owner = None
        self._depth = 0

    def acquire(self, blocking: bool = True, timeout: float = -1) -> bool:
        if not blocking and timeout != -1:
            raise ValueError("非阻塞获取不能指定超时")
        if timeout < 0 and timeout != -1:
            raise ValueError("锁超时不能为负数")
        if not math.isfinite(timeout):
            raise ValueError("锁超时必须有限")
        if timeout > threading.TIMEOUT_MAX:
            raise OverflowError("锁超时超过线程等待上限")
        # 所有可能失败的截止计算必须早于入队，避免留下永不释放的失主票据。
        deadline = monotonic() + timeout if timeout >= 0 else None
        ident = threading.get_ident()
        with self._condition:
            if self._owner == ident:
                self._depth += 1
                return True
            if not blocking and (self._owner is not None or self._queue):
                return False
            ticket = object()
            self._queue.append(ticket)
            try:
                while self._owner is not None or self._queue[0] is not ticket:
                    remaining = None if deadline is None else deadline - monotonic()
                    if remaining is not None and remaining <= 0:
                        return False
                    self._condition.wait(remaining)
                self._owner, self._depth = ident, 1
                return True
            finally:
                # 超时/异常也必须移除票据并唤醒队尾，不能堵住后续控制。
                self._queue.remove(ticket)
                self._condition.notify_all()

    def release(self) -> None:
        with self._condition:
            if self._owner != threading.get_ident():
                raise RuntimeError("不能释放其它线程持有的控制锁")
            self._depth -= 1
            if self._depth == 0:
                self._owner = None
                self._condition.notify_all()

    def __enter__(self) -> "FairRLock":
        self.acquire()
        return self

    def __exit__(self, *args) -> None:
        self.release()
