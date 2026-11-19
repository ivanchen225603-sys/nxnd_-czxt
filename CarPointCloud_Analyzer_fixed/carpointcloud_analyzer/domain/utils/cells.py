"""整数格坐标的按行去重。"""
import numpy as np


def unique_cells(cells: np.ndarray, *, return_index: bool = False,
                 return_inverse: bool = False, return_counts: bool = False):
    """等价于 ``np.unique(cells, axis=0, ...)``，仅用于二维整数格坐标。

    各列减去最小值后并成一个整数键再排序，键的大小顺序就是行的字典序，
    去重行、首次出现下标、反向下标和计数都与逐行比较相同。键可能溢出或
    输入不是非空二维整数数组时，直接按行去重。
    """
    cells = np.asarray(cells)
    options = dict(return_index=return_index, return_inverse=return_inverse, return_counts=return_counts)
    if cells.ndim != 2 or cells.dtype.kind not in "iu" or 0 in cells.shape:
        return np.unique(cells, axis=0, **options)
    low = cells.min(axis=0).astype(np.int64)
    spans = [int(high) - int(base) + 1 for high, base in zip(cells.max(axis=0), low)]
    capacity = 1
    for span in spans:
        capacity *= span
    if capacity >= 2 ** 62:
        return np.unique(cells, axis=0, **options)
    keys = np.zeros(len(cells), dtype=np.int64)
    for column, span in enumerate(spans):
        keys = keys * span + (cells[:, column] - low[column])
    result = np.unique(keys, **options)
    unique_keys = result[0] if isinstance(result, tuple) else result
    rows = np.empty((len(unique_keys), cells.shape[1]), dtype=cells.dtype)
    for column in range(cells.shape[1] - 1, -1, -1):
        unique_keys, remainder = np.divmod(unique_keys, spans[column])
        rows[:, column] = remainder + low[column]
    return (rows, *result[1:]) if isinstance(result, tuple) else rows
