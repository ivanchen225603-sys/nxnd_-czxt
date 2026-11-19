"""多层地面假设应可复现，且不得污染并发任务的随机状态。"""

from concurrent.futures import ThreadPoolExecutor

import numpy as np
import pytest

from carpointcloud_analyzer.domain.utils.planes import deterministic_plane_hypothesis


def test_multilevel_plane_is_reproducible_across_order_and_threads():
    rng = np.random.default_rng(72)
    p = rng.uniform(-10, 10, (6000, 3))
    p[:3000, 2] = rng.normal(-.75, .008, 3000)
    p[3000:4800, 2] = rng.normal(0, .008, 1800)
    p[4800:, 0] = rng.normal(0, .008, 1200)
    state = np.random.get_state()
    expected, selected = deterministic_plane_hypothesis(p, .03)
    assert abs(expected[2]) > .999 and len(selected) >= 2990
    with ThreadPoolExecutor(max_workers=3) as executor:
        results = list(executor.map(lambda points: deterministic_plane_hypothesis(points, .03),
                                    [p.copy(), p[::-1], p[rng.permutation(len(p))]]))
    for plane, indices in results:
        np.testing.assert_array_equal(plane, expected)
        assert len(indices) == len(selected)
    after = np.random.get_state()
    assert state[0] == after[0] and state[2:] == after[2:]
    np.testing.assert_array_equal(state[1], after[1])


@pytest.mark.parametrize("points", [np.zeros((2, 3)), np.zeros((30, 3)), np.full((30, 3), np.nan)])
def test_invalid_plane_input_rejected(points):
    with pytest.raises(ValueError):
        deterministic_plane_hypothesis(points, .03)
