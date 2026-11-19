"""提速用的数值辅助函数必须与原写法逐位相同（2026-11-13 第一档提速）。"""
import numpy as np
import pytest

from carpointcloud_analyzer.domain.utils.cells import unique_cells
from carpointcloud_analyzer.domain.utils.parallel import map_blocks, map_parallel, stacked_svd
from carpointcloud_analyzer.domain.utils.planes import _lexicographic_order, deterministic_plane_hypothesis


def _same(expected, actual):
    expected, actual = (expected, actual) if isinstance(expected, tuple) else ((expected,), (actual,))
    assert len(expected) == len(actual)
    for left, right in zip(expected, actual):
        assert left.dtype == right.dtype and left.shape == right.shape
        assert left.tobytes() == right.tobytes()


@pytest.mark.parametrize("options", [
    {}, {"return_index": True}, {"return_inverse": True}, {"return_counts": True},
    {"return_index": True, "return_inverse": True, "return_counts": True},
])
@pytest.mark.parametrize("dtype", [np.int64, np.int32])
def test_unique_cells_matches_row_unique(options, dtype):
    rng = np.random.default_rng(7)
    for columns in (1, 2, 3):
        for spread in (2, 40, 100000):
            cells = rng.integers(-spread, spread, size=(1500, columns)).astype(dtype)
            _same(np.unique(cells, axis=0, **options), unique_cells(cells, **options))


def test_unique_cells_falls_back_when_key_would_overflow_or_input_is_not_a_grid():
    wide = np.array([[0, 0, 0], [2 ** 40, 2 ** 40, 2 ** 40], [0, 0, 0]])
    _same(np.unique(wide, axis=0, return_inverse=True), unique_cells(wide, return_inverse=True))
    empty = np.zeros((0, 2), dtype=int)
    _same(np.unique(empty, axis=0), unique_cells(empty))
    points = np.array([[.5, 1.], [.5, 1.], [-.25, 3.]])
    _same(np.unique(points, axis=0, return_counts=True), unique_cells(points, return_counts=True))


def test_stacked_svd_matches_single_call_for_small_and_split_batches():
    rng = np.random.default_rng(11)
    for count in (0, 1, 5, 9000):
        matrices = rng.normal(size=(count, 20, 3))
        _same(tuple(np.linalg.svd(matrices, full_matrices=False)), stacked_svd(matrices))


def test_parallel_maps_keep_input_order_and_raise_the_first_error():
    assert map_parallel(lambda value: value * 2, range(50)) == [value * 2 for value in range(50)]
    assert map_blocks(lambda start, stop: (start, stop), 10, 4) == [(0, 4), (4, 8), (8, 10)]
    assert map_blocks(lambda start, stop: (start, stop), 0, 4) == []

    def fail(value):
        raise ValueError(str(value))

    with pytest.raises(ValueError, match="^0$"):
        map_parallel(fail, range(8))
    # 池内任务再次并行时就地执行，占满线程池也不会互相等待。
    nested = map_parallel(lambda value: map_parallel(lambda inner: inner + value, range(4)), range(64))
    assert nested == [[inner + value for inner in range(4)] for value in range(64)]


def test_lexicographic_order_matches_lexsort_with_ties_and_signed_zero():
    rng = np.random.default_rng(13)
    for _ in range(50):
        points = rng.integers(-2, 3, size=(int(rng.integers(3, 300)), 3)).astype(float)
        points[rng.random(len(points)) < .1, 0] = -0.0
        assert np.array_equal(_lexicographic_order(points), np.lexsort(points.T[::-1]))
    points = rng.normal(size=(5000, 3))
    assert np.array_equal(_lexicographic_order(points), np.lexsort(points.T[::-1]))


def test_plane_hypothesis_keeps_the_first_best_hypothesis_across_parallel_groups():
    rng = np.random.default_rng(17)
    plane_points = np.c_[rng.uniform(-5, 5, size=(6000, 2)), rng.normal(0., .005, size=6000)]
    points = np.vstack([plane_points, rng.uniform(-5, 5, size=(3000, 3))])
    threshold, iterations = .03, 1000
    plane, inliers = deterministic_plane_hypothesis(points, threshold, iterations)

    # 原写法：全量字典序排序，32 个假设一块依次评分，严格大于才替换。
    ordered = points[np.lexsort(points.T[::-1])]
    sample = ordered[np.linspace(0, len(ordered) - 1, min(len(ordered), 8192), dtype=int)]
    triples = sample[np.random.default_rng(0).integers(0, len(sample), size=(iterations, 3))]
    normals = np.cross(triples[:, 1] - triples[:, 0], triples[:, 2] - triples[:, 0])
    norms = np.linalg.norm(normals, axis=1)
    valid = norms > 1e-10
    normals = normals[valid] / norms[valid, None]
    offsets = -np.einsum('ij,ij->i', normals, triples[valid, 0])
    best_key, best = (-1, -float('inf')), None
    for start in range(0, len(normals), 32):
        distance = np.abs(sample @ normals[start:start + 32].T + offsets[start:start + 32])
        inside = distance <= threshold
        counts = inside.sum(axis=0)
        distance[~inside] = 0
        errors = distance.sum(axis=0) / np.maximum(counts, 1)
        for j in range(len(counts)):
            key = (int(counts[j]), -float(errors[j]))
            if key > best_key:
                best_key, best = key, start + j
    expected = np.r_[normals[best], offsets[best]]
    if expected[2] < 0:
        expected *= -1
    assert plane.tobytes() == expected.tobytes()
    assert np.array_equal(inliers, np.flatnonzero(np.abs(points @ expected[:3] + expected[3]) <= threshold))
