"""进程内共享的数值线程池：只并行互不相关的同一批计算，结果与串行逐位相同。

各块调用的函数与串行时完全一样，只是同时执行，再按原顺序拼回；NumPy/SciPy 的
批量运算在执行期间释放 GIL。池内任务再次调用本模块时就地串行执行，不会互相等待。
"""
import os
import threading
from concurrent.futures import ThreadPoolExecutor

import numpy as np

WORKERS = max(1, min(8, os.cpu_count() or 1))
_THREAD_PREFIX = "carpointcloud-numeric"
_pool = ThreadPoolExecutor(max_workers=WORKERS, thread_name_prefix=_THREAD_PREFIX)


def map_parallel(function, items) -> list:
    """并行执行互不相关的 ``function(item)``，按输入顺序返回；靠前一项的异常先抛出。"""
    items = list(items)
    if (len(items) <= 1 or WORKERS == 1
            or threading.current_thread().name.startswith(_THREAD_PREFIX)):
        return [function(item) for item in items]
    return list(_pool.map(function, items))


def map_blocks(function, count: int, block: int) -> list:
    """把 ``[0, count)`` 按 ``block`` 切段，并行调用 ``function(start, stop)``，按段序返回。"""
    ranges = [(start, min(start + block, count)) for start in range(0, count, block)]
    return map_parallel(lambda item: function(*item), ranges)


def stacked_svd(matrices: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """等价于 ``np.linalg.svd(matrices, full_matrices=False)``。

    堆叠矩阵逐个独立分解，分段并行后按原顺序拼接，数值与一次调用相同。
    """
    block = max(1024, -(-len(matrices) // WORKERS))
    if len(matrices) <= block:
        return tuple(np.linalg.svd(matrices, full_matrices=False))
    parts = map_blocks(lambda start, stop: np.linalg.svd(matrices[start:stop], full_matrices=False),
                       len(matrices), block)
    return tuple(np.concatenate(part) for part in zip(*parts))
