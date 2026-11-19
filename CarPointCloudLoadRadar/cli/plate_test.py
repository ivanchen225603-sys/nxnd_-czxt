#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cli/plate_test.py — 车牌相机连通性测试（现场校准第三步）

为什么要有这个单独的脚本？
    车牌不走雷达，走**外部相机**。相机接口路径没法从二进制还原
    （stopdetect.dll 里一个 URL 字符串都没有，见 app/plate.py 的说明），
    必须现场按相机厂商手册填进 config/channels.json 的 plate 段。
    而"填得对不对"这件事，最好在命令行里单独验证 ——
    不要在服务里调：一旦不通，你分不清是相机的问题还是雷达服务的问题。

    所以顺序是：plate_test.py 打通 → 再 run.py 接入。
    （和 capture.py → probe.py → run.py 是同一种思路：先分环节验证，再合起来跑。）

────────────────────────────────────────────────────────────────────────
用法
────────────────────────────────────────────────────────────────────────

    # 按配置打一次（最常用）
    python3 cli/plate_test.py

    # 连打 5 次，看稳定性/耗时
    python3 cli/plate_test.py --sample 5

    # 临时试一套接口，不写回配置
    python3 cli/plate_test.py \
        --base-url http://192.168.1.100 --device-id 1 \
        --snap-url   "/api/gate/snap?device={device_id}" \
        --result-url "/api/gate/result?device={device_id}" \
        --field data.plate

    # 试通了，把参数写回 config/channels.json 的 plate 段
    python3 cli/plate_test.py --base-url ... --snap-url ... --result-url ... \\
        --field data.plate --save-config

    # 排查用：把相机返回的原文打出来，方便对着手册抠 plate_field
    python3 cli/plate_test.py --dump-raw

    # 没相机时验证"仿真出牌"模式（服务链路演示用）
    python3 cli/plate_test.py --mode simulate

退出码
    0 = 成功取到车牌    1 = 没取到（会打印原因）    2 = 参数/配置有问题
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import _common
_common.setup_console()

from app.settings import load_settings
from app.plate import build_plate_reader, safe_dir_name, DEFAULT_PLATE_REGEX


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="plate_test.py", description="车牌相机连通性测试",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("用法")[0])
    p.add_argument("--channels", default="", help="多通道配置路径")
    p.add_argument("--mode", default="", choices=["", "auto", "simulate", "null", "http"],
                   help="读取器模式（默认按配置）")
    p.add_argument("--sample", type=int, default=1, help="连续读取次数（默认 1）")
    p.add_argument("--timeout", type=float, default=8.0, help="单次等待上限（秒）")
    p.add_argument("--base-url", default="", help="覆盖 plate.base_url")
    p.add_argument("--device-id", default="", help="覆盖 plate.device_id")
    p.add_argument("--snap-url", default="", help="覆盖 plate.snap_url")
    p.add_argument("--result-url", default="", help="覆盖 plate.result_url")
    p.add_argument("--method", default="", choices=["", "GET", "POST"],
                   help="覆盖 plate.method")
    p.add_argument("--field", default="", help="覆盖 plate.plate_field（如 data.plate）")
    p.add_argument("--regex", default="", help="覆盖 plate.plate_regex")
    p.add_argument("--header", action="append", default=[], metavar="K:V",
                   help="附加请求头，可多次（如 --header Authorization:abc）")
    p.add_argument("--enable", action="store_true", help="顺便把 plate.enabled 置 true")
    p.add_argument("--save-config", action="store_true",
                   help="把本次覆盖写回 config/channels.json（会先备份）")
    p.add_argument("--dump-raw", action="store_true",
                   help="打印相机返回的原文（排查 plate_field 用）")
    return p


def apply_overrides(cfg, args) -> None:
    if args.base_url:
        cfg.base_url = args.base_url
    if args.device_id:
        cfg.device_id = args.device_id
    if args.snap_url:
        cfg.snap_url = args.snap_url
    if args.result_url:
        cfg.result_url = args.result_url
    if args.method:
        cfg.method = args.method
    if args.field:
        cfg.plate_field = args.field
    if args.regex:
        cfg.plate_regex = args.regex
    if args.enable:
        cfg.enabled = True
    for h in args.header:
        if ":" not in h:
            raise SystemExit(f"× --header 格式应为 K:V，收到: {h!r}")
        k, v = h.split(":", 1)
        cfg.headers[k.strip()] = v.strip()


def save_back(channels_path: str, cfg) -> None:
    """把 plate 段的当前值写回 channels.json（先备份 .bak）。"""
    with open(channels_path, encoding="utf-8") as f:
        obj = json.load(f)
    plate = obj.setdefault("plate", {})
    plate["enabled"] = bool(cfg.enabled)
    plate["base_url"] = cfg.base_url
    plate["device_id"] = cfg.device_id
    plate["camera_identifier"] = cfg.camera_identifier
    plate["snap_url"] = cfg.snap_url
    plate["result_url"] = cfg.result_url
    plate["method"] = cfg.method
    plate["body"] = cfg.body
    plate["headers"] = cfg.headers
    plate["plate_field"] = cfg.plate_field
    plate["plate_regex"] = cfg.plate_regex

    bak = channels_path + ".bak"
    if not os.path.exists(bak):
        with open(channels_path, encoding="utf-8") as src:
            old_obj = json.load(src)
        with open(bak, "w", encoding="utf-8") as f:
            json.dump(old_obj, f, ensure_ascii=False, indent=2)
    with open(channels_path, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)
    print(f"\n√ 已写回 : {channels_path}")
    print(f"   （原文件备份在 {os.path.basename(bak)}）")


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    st = load_settings(args.channels or "")
    cfg = st.plate
    apply_overrides(cfg, args)

    print("=" * 78)
    print("车牌相机连通性测试")
    print("=" * 78)
    print(f"配置文件   : {st.channels_path}")
    print(f"启用       : {cfg.enabled}    模式: {args.mode or cfg.mode or 'auto'}")
    print(f"相机基址   : {cfg.base_url or '(未配置)'}   设备号: {cfg.device_id}")
    print(f"抓拍接口   : {cfg.snap_url or '(未配置)'}")
    print(f"结果接口   : {cfg.result_url or '(未配置)'}")
    print(f"取牌字段   : {cfg.plate_field or '(未配置 → 退回全文正则)'}")
    print(f"车牌正则   : {cfg.plate_regex or DEFAULT_PLATE_REGEX}")
    print("-" * 78)

    reader = build_plate_reader(cfg, mode=args.mode,
                                save_dir=os.path.join(_common.ensure_root(),
                                                      "logs", "plate_fail"))
    print(f"读取器     : {reader.kind}")

    if reader.kind == "null":
        print("\n▲  当前是 Null 读取器 —— 不会真的取车牌。")
        if not cfg.snap_url and not cfg.result_url:
            print("    原因：plate.snap_url / result_url 都是空的。")
            print("    现场请按相机手册填这两个地址，或先用 --mode simulate 演示链路。")
        if not cfg.enabled:
            print("    另：plate.enabled=false。填好地址后加 --enable 或手动改配置。")
        return 2

    n = max(1, args.sample)
    print(f"\n开始读取（{n} 次，单次上限 {args.timeout:g}s）…")
    print("-" * 78)
    print(f"{'#':<4}{'车牌':<16}{'置信':>6}{'耗时':>9}{'重试':>6}  说明")

    n_ok = 0
    last_raw = ""
    last_plate = ""
    for i in range(1, n + 1):
        r = reader.read(timeout=args.timeout)
        if r.ok:
            n_ok += 1
            last_plate = r.plate
            print(f"{i:<4}{r.plate:<16}{r.confidence:>6.2f}{r.elapsed:>8.2f}s"
                  f"{r.attempts:>6}  √")
        else:
            print(f"{i:<4}{'—':<16}{'—':>6}{r.elapsed:>8.2f}s{r.attempts:>6}  "
                  f"× {r.error or '未取到车牌'}")
        if args.dump_raw and r.raw:
            last_raw = r.raw

    print("-" * 78)
    print(f"成功 {n_ok}/{n}")

    if args.dump_raw:
        print("-" * 78)
        print("相机返回原文（最后一次非空）：")
        print("-" * 78)
        print(last_raw[:2000] if last_raw else "（没有可显示的内容 —— "
              "说明连请求都没成功，先确认网络/地址）")
        if last_raw:
            print("\n→ 对照上面的原文，把 JSON 取值路径填进 plate.plate_field，"
                  "例如 {\"data\":{\"plate\":\"宁A88635W\"}} → data.plate")

    if n_ok:
        # 顺手验证目录名规范化（GBK 乱码问题的修复点）。
        # ★ 必须拿**刚读到的**车牌来演示（2026-09-23）：原来写死
        #   safe_dir_name('宁A88635W')，而上面逐行打印的是真实读到的牌 ——
        #   两者对不上时现场看到的是"读到 宁A10333F、目录名却是 宁A88635W"，
        #   像是 safe_dir_name 把车牌改写了。实际没有，但排查这种**假故障**
        #   要花的时间比这几行代码多得多。
        print(f"\n车牌目录名 : {safe_dir_name(last_plate)}  "
              f"（由上面最后一次成功识别的 {last_plate} 规范化而来；safe_dir_name 会把盘符/空格/分隔符处理掉，见 app/plate.py）")

    if args.save_config:
        save_back(st.channels_path, cfg)

    if n_ok:
        print("\n√ 相机通了。可以把 channels.json 的 plate.enabled 打开，"
              "然后 " + _common.PYEXE + " cli/run.py --mode live")
    else:
        print("\n排查顺序：")
        print("  1) ping 相机 IP；curl 一下 snap_url（去掉占位符的真实地址）看返回什么")
        print("  2) 加 --dump-raw 看原文，确认返回里到底有没有车牌号")
        print("  3) 有车牌号但没取到 → 多半是 plate_field 路径或车牌正则不对")
        print("  4) 全都没有 → 接口方法/请求头不对（试 --method POST --header ...）")
    print("=" * 78)
    return 0 if n_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
