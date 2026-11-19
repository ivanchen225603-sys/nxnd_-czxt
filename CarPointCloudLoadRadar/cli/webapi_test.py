#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cli/webapi_test.py -- 后台接口连通性自检（接通车牌号的第一步）

为什么要有这个单独的脚本？
    车牌**不走相机**，走后台。链路是：

        乙方识牌服务  GET {PartnerBaseUrl}/api/v1/licensePlateRecords?cameraIdentifier=CAM_00x
              |   后台 PartnerLicensePlateClient 轮询重试（10s -> 5s -> 5s）
              v
        VehicleArrivalRecord.PlateStatus = Succeeded + LicensePlate
              |   我们 POST /api/v1/load-radar-agent/passages/prepare
              v
        回执 binding.licensePlate，身份 = "plate:<规范化车牌>"

    也就是说我们**不主动查车牌**，车牌是到车登记接口的返回值之一。
    `config/channels.json` 的 plate 段（相机直连）是**另一条路**，
    两条互斥 —— 开了后台桥之后 service.py 会把本地相机读牌强制关成 null。

    这个脚本只做**只读**探测（GET /baselines），目的是在把
    `config/webapi.json` 的 enabled 打开**之前**，先证明这几件事是通的：
        · 网络到得了后台            · node_id 认得
        · agent_key 对得上          · 后台 LoadRadar:Enabled = true
        · 后台 Database:Enabled     · 本节点通道已登记

    ★ 刻意**不**做 passages/prepare 探测：那个接口有副作用 ——
      它会在后台真的建一条到车记录并触发一次真实识牌任务。
      车牌本身能不能取到，由**第一辆真车**在看板上证明，不该拿假数据试。

★★ 为什么必须先跑这个，而不是直接把 enabled 打开：
    开了桥之后 `service.py` 会用到车准备状态**门控测量** ——
    状态不是 ready / baseline_required / not_applicable 就
    `_discard_observation_window()` 直接返回。而后台不可达时
    状态恒为 awaiting_plate，于是**一辆车都测不出来，本地也不落盘**。
    先跑通这个脚本，再打开开关。

--------------------------------------------------------------------
用法
--------------------------------------------------------------------

    # 按 config/webapi.json 探一次（最常用）
    python3 cli/webapi_test.py

    # 临时试另一个后台地址，不写回配置
    python3 cli/webapi_test.py --base-url http://10.0.0.5:5031

    # 后台在别的机器上、且只有 http（没有 https）时
    python3 cli/webapi_test.py --base-url http://10.0.0.5:5031 --insecure

退出码：0 = 通；1 = 不通（原因已打印）；2 = 配置本身有问题。
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import _common
_common.setup_console()

from app.load_radar_transport import BridgeConfig, HttpClient, HttpError


# HTTP 状态 -> 现场该去查哪里。不认识的状态不瞎猜，原样报。
CAUSES = {
    401: ("鉴权失败", [
        "config/webapi.json 的 agent_key 与后台 LoadRadar:Nodes[].AgentKey 不一致",
        "两边 node_id 不一致（请求头 X-Load-Radar-Node）",
    ]),
    403: ("节点身份不符", [
        "node_id 不在后台 LoadRadar:Nodes 里",
    ]),
    404: ("路径不存在", [
        "base_url 指到了别的服务（这里要的是 CarPointCloudWebApi，不是我们自己的看板 8080）",
        "后台版本过旧，没有 load-radar-agent 路由",
    ]),
    400: ("节点或通道未登记", [
        "后台 LoadRadar:Nodes[].Channels 里没有本节点要用的通道",
    ]),
    503: ("后台模块未启用", [
        "后台 appsettings.json 缺 LoadRadar 段，或 LoadRadar:Enabled = false",
        "后台 Database:Enabled = false（LoadRadar 启用时必须开数据库）",
        "三个 SQL 迁移脚本没执行：load_radar_20260917 / _publication_20260918 / _passage_start_20260918",
    ]),
}


def describe(status, retryable):
    if status is None:
        return "连不上后台", [
            "base_url 写错，或后台没起来",
            "防火墙 / 路由不通（麒麟机与后台之间）",
            "base_url 指向了 https 而后台只开了 http",
        ]
    if status in CAUSES:
        return CAUSES[status]
    return "后台返回 HTTP %d%s" % (status, "（可重试）" if retryable else ""), []


def main() -> int:
    root = _common.ensure_root()

    p = argparse.ArgumentParser(description="后台接口连通性自检（只读）")
    p.add_argument("--config", default=os.path.join(root, "config", "webapi.json"),
                   help="接口配置（默认 config/webapi.json）")
    p.add_argument("--base-url", default="", help="临时覆盖后台地址，不写回配置")
    p.add_argument("--insecure", action="store_true",
                   help="允许非回环的明文 http（后台不在本机且没有 https 时才加）")
    args = p.parse_args()

    # ---- 1. 配置 ----
    print("== 1. 配置 ==")
    if not os.path.isfile(args.config):
        print("  找不到配置文件：%s" % args.config)
        print("  从模板复制一份：config/webapi.example.json -> config/webapi.json")
        return 2
    try:
        cfg = BridgeConfig.load(args.config)
    except ValueError as exc:
        print("  配置无法加载：%s" % exc)
        return 2

    if args.base_url or args.insecure:
        raw = json.load(open(args.config, "r", encoding="utf-8-sig"))
        if args.base_url:
            raw["base_url"] = args.base_url
        if args.insecure:
            raw["allow_insecure_http"] = True
        raw["storage_dir"] = str(cfg.storage_dir)
        try:
            cfg = BridgeConfig(**raw)
        except ValueError as exc:
            print("  覆盖后的配置无效：%s" % exc)
            return 2

    print("  文件      : %s" % args.config)
    print("  node_id   : %s" % cfg.node_id)
    print("  base_url  : %s" % cfg.base_url)
    print("  agent_key : %d 字符（不打印内容）" % len(cfg.agent_key))
    print("  通道      : %s" % ", ".join(
        "%s->%s" % (k, v.calibration_id) for k, v in sorted(cfg.channels.items())))
    print("  enabled   : %s%s" % (cfg.enabled,
          "" if cfg.enabled else "   <- 还没打开；先让本脚本通过，再改成 true"))

    # ---- 2. 只读探测 ----
    print("\n== 2. 只读探测 GET /api/v1/load-radar-agent/baselines ==")
    try:
        rows = HttpClient(cfg).list_baselines()
    except HttpError as exc:
        title, hints = describe(exc.status, exc.retryable)
        print("  不通：%s" % title)
        for h in hints:
            print("    - %s" % h)
        return 1
    except Exception as exc:                     # 契约外的异常照样要能看懂
        print("  不通：%s: %s" % (type(exc).__name__, exc))
        return 1

    print("  通。本节点当前活动基线 %d 份。" % len(rows))
    for row in rows[:8]:
        print("    %s / %s  车牌=%s  标定=%s" % (
            row.get("channelId"), row.get("versionId"),
            row.get("licensePlate"), row.get("calibrationId")))
    if not rows:
        print("    （0 份是正常的：还没标过空车基线。只要 prepare 这一侧也通，")
        print("      到车准备会返回 baseline_required —— 判空车照常，体积为 null。）")

    # ---- 3. 下一步 ----
    print("\n== 3. 下一步 ==")
    print("  管路已通 -- 但**这不等于车牌能取到**，两件事被不同的开关卡着：")
    print("    · 本脚本探的是 GET /baselines，只证明 LoadRadar 段开着、库连得上；")
    print("    · 车牌走的是 POST /passages/prepare，它另外要求后台")
    print("      Integration:ExternalPartnerEnabled = true。")
    print("      这个开关关着时，新车**每次都** 503「正式识牌服务未启用」，")
    print("      且重试不会好（SavePassageStart 在抛异常之后，见 docs/49 第零节）。")
    print("  确认后台那个开关也开着，再把 config/webapi.json 的 enabled 改成 true，")
    print("  并且启动时显式带上参数（光改 enabled 没用，没有自动发现）：")
    print("    %s cli/run.py --mode live --webapi-config config/webapi.json" % _common.PYEXE)
    print("  之后车牌由第一辆真车证明，看板卡片会显示到车准备状态：")
    print("    awaiting_plate    识牌中（到车 30 秒内）；也可能是 prepare 一直失败")
    print("    plate_unavailable 识牌失败或超窗 -> 查后台 Integration:PartnerBaseUrl")
    print("    baseline_required 车牌已拿到，缺空车基线（体积 null，判定照常）")
    print("    ready             车牌 + 基线都齐")
    return 0


if __name__ == "__main__":
    sys.exit(main())
