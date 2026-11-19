"""诊断保持数值库与GC配置不变。"""
import gc
import os
import sys
import threading

import numpy as np
from threadpoolctl import threadpool_info

from carpointcloud_analyzer.infrastructure.runtime_diagnostics import snapshot


def test_snapshot_is_read_only_and_reports_actual_process():
    before = (np.geterr(), gc.isenabled(), gc.get_threshold(), sys.getswitchinterval(), threadpool_info())
    result = snapshot()
    after = (np.geterr(), gc.isenabled(), gc.get_threshold(), sys.getswitchinterval(), threadpool_info())
    assert before == after
    assert result['pid'] == os.getpid()
    assert result['nativeThreadId'] == threading.get_native_id()
    assert result['rssBytes'] > 0
    assert result['nativeThreads'] >= result['pythonThreads']
    assert result['numericPools'] == before[-1]
