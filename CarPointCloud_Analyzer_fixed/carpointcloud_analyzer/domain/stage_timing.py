"""分阶段耗时采样：只写日志，不进入响应、业务载荷或算法判定。

``collect()`` 在一次请求内开启采样；``stage()`` 包住一段调用并记录总耗时，
其内部的 ``lap()`` 记录相对上一个打点的耗时，名称带外层阶段前缀。
未开启采样时两者都是空操作，算法单测与离线工具不受影响。
"""
import json
import logging
import time
from contextlib import contextmanager
from contextvars import ContextVar

from carpointcloud_analyzer.domain.runtime import logger

_current = ContextVar("carpointcloud_stage_timer", default=None)


class StageTimer:
    def __init__(self, workflow_id=None, pair_id=None, snapshot=None):
        self.started = time.perf_counter()
        self.workflow_id = workflow_id
        self.pair_id = pair_id
        self.entries = []
        self._snapshot = snapshot
        self.diagnostic_ms = 0.0
        self._scopes = [["", (self.started, time.thread_time(), time.process_time())]]

    def capture_state(self, name):
        if self._snapshot is None or not logger.isEnabledFor(logging.INFO):
            return
        started = time.perf_counter()
        try:
            state = self._snapshot(name)
            logger.info("运行状态 workflowId=%s pairId=%s boundary=%s state=%s",
                        self.workflow_id, self.pair_id, name,
                        json.dumps(state, ensure_ascii=False, separators=(",", ":")))
        except Exception:
            # 只隔离观测失败；诊断不能改变本次计算结果或吞掉算法异常。
            logger.warning("运行状态采样失败 workflowId=%s boundary=%s", self.workflow_id, name, exc_info=True)
        finally:
            self.diagnostic_ms += (time.perf_counter() - started) * 1000

    def _record(self, name, since):
        now = (time.perf_counter(), time.thread_time(), time.process_time())
        elapsed = [round((end - start) * 1000, 1) for end, start in zip(now, since)]
        self.entries.append((name, elapsed[0]))
        # thread_cpu不含数值库的其他线程；process_cpu包含本进程其他并发请求，不能当作单请求CPU。
        logger.info("分析阶段 workflowId=%s pairId=%s stage=%s wall_ms=%s thread_cpu_ms=%s process_cpu_ms=%s",
                    self.workflow_id, self.pair_id, name, *elapsed)
        if "/" not in name or name.startswith(("persist/", "resolve-plate/")):
            self.capture_state(name)
        return now

    def lap(self, name):
        scope = self._scopes[-1]
        scope[1] = self._record(scope[0] + name, scope[1])

    def total_ms(self):
        return round((time.perf_counter() - self.started) * 1000, 1)

    def describe(self):
        return " ".join(f"{name}={ms}" for name, ms in self.entries)


@contextmanager
def collect(*, workflow_id=None, pair_id=None, snapshot=None):
    timer = StageTimer(workflow_id, pair_id, snapshot)
    token = _current.set(timer)
    timer.capture_state("request-start")
    try:
        yield timer
    finally:
        timer.capture_state("request-end")
        if snapshot is not None:
            logger.info("状态采样总耗时 workflowId=%s pairId=%s diagnostic_ms=%.1f",
                        workflow_id, pair_id, timer.diagnostic_ms)
        _current.reset(token)


@contextmanager
def stage(name):
    timer = _current.get()
    if timer is None:
        yield
        return
    name = timer._scopes[-1][0] + name
    started = (time.perf_counter(), time.thread_time(), time.process_time())
    logger.info("分析阶段开始 workflowId=%s pairId=%s stage=%s",
                timer.workflow_id, timer.pair_id, name)
    timer._scopes.append([name + "/", started])
    try:
        yield
    finally:
        timer._scopes.pop()
        # 外层后续打点从本阶段结束算起，避免把本阶段重复计入下一个 lap。
        timer._scopes[-1][1] = timer._record(name, started)


def lap(name):
    timer = _current.get()
    if timer is not None:
        timer.lap(name)
