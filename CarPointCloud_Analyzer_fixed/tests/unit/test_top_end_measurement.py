"""端部扫描盲段触发补缝，已有连续侧壁保持原补点行为。"""

import numpy as np

from carpointcloud_analyzer.domain.inner_faces import ObservedFace
from carpointcloud_analyzer.domain.top_end_measurement import (
    needs_side_end_bridge,
)


def test_long_end_scan_gap_requests_completion_without_changing_supported_top(monkeypatch):
    length, height = np.meshgrid(np.arange(.3, 9.71, .1),
                                 np.arange(.1, 2.01, .05))
    observed = np.c_[np.full(length.size, -1.), length.ravel(), height.ravel()]
    width, wall_height = np.meshgrid(np.arange(-1.1, 1.11, .05),
                                     np.arange(.1, 2.01, .05))
    rear = np.c_[width.ravel(), np.zeros(width.size), wall_height.ravel()]
    front = rear.copy()
    front[:, 1] = 10.
    synthetic = np.vstack([rear, front])
    side = ObservedFace(0, np.array([0., 0., -1.]), observed)
    monkeypatch.setattr('carpointcloud_analyzer.domain.top_end_measurement._side_candidates',
                        lambda *args, **kwargs: [(.9, .95, -1, side)])
    assert not needs_side_end_bridge(np.vstack((observed, synthetic)), 0., 10., 0., 2.)

    # 顶部端点超过原 3 格连续性门槛时，门槛只触发几何补点，
    # 不再作为允许补点的最长距离。
    end_gap = observed[(observed[:, 1] >= 1.5) & (observed[:, 1] <= 8.5)]
    assert needs_side_end_bridge(np.vstack((end_gap, synthetic)), 0., 10., 0., 2.)
