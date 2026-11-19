#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cli/_common.py — 命令行工具的公共部分

放在这里的三件事，每个 cli/ 脚本都要用，所以只写一遍：
    ensure_root()   把项目根加进 sys.path，让脚本能从任意目录启动
    add_common_args() 统一的公共参数（--channels / --channel / -v）
    load_ctx()      一次把 settings / calib / devices / angle 都加载好
"""
from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass
from typing import List, Optional

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

# 提示里写的解释器名按平台取：Windows 只有 python，麒麟/Linux 才是 python3。
# 现场照着提示抄命令很常见，写错了会白绕一圈。
PYEXE = "python" if os.name == "nt" else "python3"


def ensure_root() -> str:
    return _ROOT


def setup_console() -> None:
    """
    Windows 控制台默认不是 UTF-8，中文输出会变问号。
    麒麟/Linux 本来就是 UTF-8，这里只是把 Windows 拉齐。
    """
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")  # py3.7+
        except Exception:                                        # noqa: BLE001
            pass


def add_common_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--channels", default="", metavar="PATH",
                   help="多通道配置文件（默认 config/channels.json）")
    p.add_argument("--channel", action="append", default=[], metavar="ID",
                   help="只处理指定通道，可多次使用（默认全部启用的通道）")
    p.add_argument("-q", "--quiet", action="store_true", help="少说话")


@dataclass
class Ctx:
    settings: object
    channels: List[object]           # 解析过路径的 ChannelCfg
    calib: object = None
    devices: object = None
    angle: object = None

    def describe(self) -> str:
        from app.pointcloud import AngleCache
        s = self.settings
        lines = [f"配置文件   : {s.channels_path}",
                 f"通道       : " + ", ".join(
                     f"{c.id}({c.mode}{'' if c.enabled else ',停用'})" for c in s.channels)]
        if self.calib is not None:
            lines.append(f"拟合参数   : {self.calib.path}")
            lines.append(f"ROI        : {self.calib.roi}")
        if self.devices is not None:
            lines.append(f"判定门槛   : {self.devices.path}")
        return "\n".join(lines)


def load_ctx(args, need_calib: bool = True) -> Ctx:
    from app.settings import load_settings, load_calib, load_devices
    from app.pointcloud import AngleCache

    st = load_settings(args.channels or "")
    ids = set(args.channel or [])
    chans = [c for c in st.channels if c.enabled and (not ids or c.id in ids)]
    if ids:
        missing = ids - {c.id for c in st.channels}
        if missing:
            raise SystemExit(f"× 配置里没有这些通道: {', '.join(sorted(missing))}")
    if not chans:
        raise SystemExit("× 没有可用的通道（检查 config/channels.json 的 enabled）")

    ctx = Ctx(settings=st, channels=chans)
    if need_calib:
        ctx.calib = load_calib(chans[0].abs_config_path)
        ctx.devices = load_devices(chans[0].abs_devices_path)
        ctx.angle = AngleCache(chans[0].abs_angle_cache).load()
    return ctx
