"""客户端超时不重复启动同一组采集，也不丢失已完成的结果。"""
import asyncio
from dataclasses import replace
import threading

import pytest

from carpointcloud_analyzer.application.service import AnalysisService, PairAnalyzeRequest


def test_timeout_retry_joins_running_job_then_reuses_completed_result():
    entered = threading.Event()
    release = threading.Event()
    calls = []
    service = AnalysisService()
    request = PairAnalyzeRequest('a.pcd', 'A', 'b.pcd', 'B', 'pair', 'workflow')
    result = {'success': True, 'outputs': {'height': 1.95}}

    def calculate(value):
        calls.append(value)
        entered.set()
        assert release.wait(5)
        return result

    service.analyze_pair_sync = calculate

    async def run():
        first = asyncio.create_task(service.analyze_pair(request))
        assert await asyncio.to_thread(entered.wait, 2)
        # HTTP超时取消等待者；后台线程仍在计算。
        first.cancel()
        with pytest.raises(asyncio.CancelledError):
            await first
        retry = asyncio.create_task(service.analyze_pair(request))
        await asyncio.sleep(0.02)
        release.set()
        assert await retry == result
        # 模拟首个HTTP响应遗失，完成后再次取同一任务结果。
        assert await service.analyze_pair(request) == result
        assert calls == [request]

    try:
        asyncio.run(run())
    finally:
        release.set()
        service.shutdown()


def test_actual_failure_can_retry_and_different_inputs_are_not_reused():
    service = AnalysisService()
    request = PairAnalyzeRequest('a.pcd', 'A', 'b.pcd', 'B', 'pair', 'workflow')
    calls = []

    def calculate(value):
        calls.append(value)
        if len(calls) == 1:
            raise RuntimeError('database failed')
        return {'success': True, 'path': value.pcd_path_b}

    service.analyze_pair_sync = calculate

    async def run():
        with pytest.raises(RuntimeError, match='database failed'):
            await service.analyze_pair(request)
        assert (await service.analyze_pair(request))['path'] == 'b.pcd'
        changed = replace(request, pcd_path_b='different.pcd')
        assert (await service.analyze_pair(changed))['path'] == 'different.pcd'
        assert calls == [request, request, changed]

    try:
        asyncio.run(run())
    finally:
        service.shutdown()
