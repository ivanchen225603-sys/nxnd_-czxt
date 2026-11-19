#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cli/run.py — 启动检测服务；后台接管时只提供采集接口

用法示例

    # ① 无雷达，先跑通全链路（推荐第一次就这么干）
    python3 cli/run.py --demo

    # ② 接真雷达
    python3 cli/run.py --mode live

    # ③ 回放历史样本，验证算法/回归，不写盘
    python3 cli/run.py --mode replay

    # ④ 只看数据不落盘（现场"跑给你看"又不污染数据目录）
    python3 cli/run.py --mode live --no-save

    # ⑤ 只用第 1 通道，且不启看板（塞进 systemd 跑）
    python3 cli/run.py --channel CH1 --no-web

参数说明
    --mode      覆盖 config/channels.json 里每条通道的 mode
                live | replay | simulate | synthetic
    --demo      等价于 --mode simulate --plate simulate（无硬件完整演示）
    --plate     车牌模式：auto | simulate | null | http
    --save/--no-save   覆盖落盘开关（默认：live 存，replay/simulate 不存）
    --open      独立模式启动后自动打开浏览器；后台接管时忽略
    -q/--quiet  不在控制台打事件日志（默认打 result/warn/error/会话起止；
                systemd 与桌面版把 stdout 重定向到 logs/*.log，那就是现场
                唯一的历史记录 —— 看板日志只在内存里，重启即失）
"""
from __future__ import annotations

import argparse
import os
import signal
import sys
import time

import _common
_common.setup_console()

from app.service import LidarService, attach_console_logger
from app.settings import dump_effective


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="run.py", description="载重雷达检测服务（麒麟平台）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("参数说明")[0])
    _common.add_common_args(p)
    p.add_argument("--mode", default="", choices=["", "live", "replay", "simulate", "synthetic"],
                   help="覆盖通道的工作模式")
    p.add_argument("--demo", action="store_true",
                   help="无硬件完整演示（= --mode simulate --plate simulate）")
    p.add_argument("--plate", default="", choices=["", "auto", "simulate", "null", "http"],
                   help="车牌读取模式")
    p.add_argument("--save", dest="save", action="store_true", default=None,
                   help="强制开启落盘")
    p.add_argument("--no-save", dest="save", action="store_false", default=None,
                   help="强制关闭落盘")
    p.add_argument("--host", default="", help="HTTP 监听地址（默认取配置）")
    p.add_argument("--port", type=int, default=0, help="HTTP 端口（默认取配置）")
    p.add_argument("--no-web", action="store_true", help="不启动 HTTP 服务（包括后台状态和命令接口）")
    p.add_argument("--open", action="store_true", help="独立模式自动打开看板；后台接管时忽略")
    p.add_argument("--webapi-config", default="", help="显式启用 CarPointCloudWebApi 接口配置文件")
    p.add_argument("--volume-provider-config", default="", help="显式载入绑定正式基线的体积物理参数文件")
    # 注意：-q/--quiet 由 _common.add_common_args 统一提供，这里**不能再加一次**
    #（argparse 会直接抛 conflicting option string，脚本起都起不来）。
    # 在本脚本里它的含义是"不打控制台事件日志"。
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    ctx = _common.load_ctx(args)

    # --demo 覆盖
    if args.demo:
        args.mode = "simulate"
        args.plate = args.plate or "simulate"

    # 把命令行覆盖写回通道配置
    for c in ctx.channels:
        if args.mode:
            c.mode = args.mode

    print("=" * 78)
    print("载重雷达检测服务")
    print("=" * 78)
    print(ctx.describe())
    print("-" * 78)
    if ctx.calib is not None:
        print(dump_effective(ctx.calib, ctx.devices))
    print(f"角度标定   : {ctx.angle.check()}")
    if args.mode or args.demo:
        print(f"模式覆盖   : {args.mode or '(无)'}   车牌: {args.plate or 'auto'}")
    print("=" * 78)

    svc = LidarService(settings=ctx.settings, save_override=args.save,
                       plate_mode=args.plate or "",
                       demo=bool(args.demo),
                       only=args.channel or None, webapi_config=args.webapi_config,
                       volume_provider_config=args.volume_provider_config)

    httpd = None
    if not args.no_web:
        from app.web.server import serve
        httpd = serve(svc, host=args.host, port=args.port, open_browser=args.open)
        host = args.host or svc.st.service.host
        port = svc.st.service.port if not args.port else args.port
        shown = "127.0.0.1" if host in ("0.0.0.0", "::", "") else host
        if svc.integration is not None:
            print(f"采集接口   : http://{shown}:{port}")
            print("操作页面   : 使用 CarPointCloudWebApi 后台的载货雷达页面")
        else:
            print(f"看板地址   : http://{shown}:{port}/")
            print("             （现场用另一台机器看时，用本机 IP 替换 127.0.0.1）")
        import threading
        threading.Thread(target=httpd.serve_forever, daemon=True,
                         name="http").start()
    else:
        print("HTTP 服务  : 已禁用（--no-web），后台状态和命令接口不可用")

    print("-" * 78)
    # ★ 控制台事件日志：服务跑起来后，关键事件（判定结果/告警/错误/会话）会打到
    #   stdout。systemd 与桌面版都把 stdout 重定向到 logs/*.log —— 这是现场唯一的
    #   历史记录（看板的日志只存在内存里，重启就没了）。--quiet 可关掉。
    clog = None if args.quiet else attach_console_logger(svc)
    print("事件日志   : " + ("关闭（--quiet）" if args.quiet else "已打开 → 标准输出"))
    print("Ctrl-C 退出")
    svc.start()

    stop = {"n": 0}

    def _sig(_s, _f):
        stop["n"] += 1
        if stop["n"] > 1:
            print("\n强制退出")
            sys.exit(1)
        print("\n正在停止…（再按一次 Ctrl-C 强制退出）")

    try:
        signal.signal(signal.SIGINT, _sig)
        signal.signal(signal.SIGTERM, _sig)
    except (ValueError, AttributeError):
        pass                       # 某些平台/线程里注册不了，忽略

    try:
        while stop["n"] == 0:
            time.sleep(0.3)
    except KeyboardInterrupt:
        pass
    finally:
        stopped = svc.stop()
        if clog is not None:
            clog.stop()
        if httpd is not None:
            httpd.shutdown()
    print("已停止" if stopped else "等待最后一次测量写盘，队列仍保持占用")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
