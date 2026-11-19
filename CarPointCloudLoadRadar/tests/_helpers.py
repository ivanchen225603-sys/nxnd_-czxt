# -*- coding: utf-8 -*-
"""测试公共夹具：把项目根加进 sys.path，并提供一份共享的标定/门槛/角度表。"""
from __future__ import annotations

import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import numpy as np                                    # noqa: E402

from app.settings import (BASELINE_SUBDIR, load_settings, load_calib,   # noqa: E402
                          load_devices)
from app.pointcloud import AngleCache                               # noqa: E402

ROOT = _ROOT


def first_channel():
    st = load_settings("")
    chans = st.enabled() or st.channels
    assert chans, "config/channels.json 里没有任何通道"
    return st, chans[0]


_CACHE = {}


def ctx():
    """共享的 (settings, channel, calib, devices, angle_cache)，只加载一次。"""
    if "ctx" not in _CACHE:
        st, ch = first_channel()
        _CACHE["ctx"] = (st, ch, load_calib(ch.abs_config_path),
                         load_devices(ch.abs_devices_path),
                         AngleCache(ch.abs_angle_cache).load())
    return _CACHE["ctx"]


def history_samples():
    """返回 [(xyz_path, reason)] —— data/loaded 下所有带 reason.txt 的样本。"""
    import glob
    d = os.path.join(ROOT, *BASELINE_SUBDIR)
    out = []
    for p in sorted(glob.glob(os.path.join(d, "**", "*.xyz"), recursive=True)):
        rp = p + ".reason.txt"
        if os.path.exists(rp):
            with open(rp, encoding="utf-8") as f:
                out.append((p, f.read().strip()))
    return out


def grid_plane(n=60, extent=2.0, z=0.0, noise=0.0005, seed=1):
    """造一片水平平面点云（法向 ±z），用于几何单测。"""
    rng = np.random.default_rng(seed)
    x = rng.uniform(-extent, extent, n * n)
    y = rng.uniform(-extent, extent, n * n)
    zz = np.full_like(x, z) + rng.normal(0, noise, x.size)
    return np.stack([x, y, zz], axis=1).astype(np.float32)
