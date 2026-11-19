"""退化判定必须与长度单位无关，并继续识别真实的平面滑动。"""
import numpy as np
import open3d as o3d
import pytest
from carpointcloud_analyzer.domain.fusion import _check_degeneracy

@pytest.mark.parametrize('scale', [0.001, 1.0, 1000.0])
@pytest.mark.parametrize('planes', [1, 3])
def test_observability_is_independent_of_length_units(scale, planes):
    rng = np.random.default_rng(20261104)
    points, normals = [], []
    for axis in range(planes):
        p = rng.uniform(-5, 5, (1200, 3))
        p[:, axis] = 0
        n = np.zeros_like(p)
        n[:, axis] = 1
        points.append(p)
        normals.append(n)
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.vstack(points) * scale))
    cloud.normals = o3d.utility.Vector3dVector(np.vstack(normals))
    axes, _ = _check_degeneracy(cloud, cloud, np.eye(4), 0.1 * scale)
    if planes == 1:
        assert axes, '单平面无法约束全部六个自由度'
    else:
        assert axes == [], '三个正交平面的约束不应随长度单位改变'
