#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/make_volume_model.py — 从一份空车基线生成「体积物理参数」文件

    python tools/make_volume_model.py --baseline data/in/CH4.baseline.npz \\
        --node-id node-1 --channel CH4 --calibration-id cal-1 \\
        --configuration-sha256 <64位> --baseline-version-id bv-1 \\
        --baseline-asset-sha256 <64位> \\
        --surface-error 0.05 --bed-error 0.03 --roi-error-area 0.5 \\
        --valid-days 365 --out config/volume-provider.json

为什么必须有这个工具
────────────────────────────────────────────────────────────────────────
体积要给出**范围**（不只是一个数），`app/volume_context.py` 要求调用方
显式声明：车厢内腔在哪、货堆最高能到多少、支持误差预算是多少 —— 并且
这些声明要用 sha256 **绑死在这一份基线和这一次测量上**，不许拿别的车的
参数来套。这条设计是对的。

但在此之前**没有任何办法把这份文件造出来**：
`VolumeCatalog.load(path)` 收的是一份手写 JSON，里面有 7 个 sha256 绑定、
`domain_xy`、`support_origin_xy`、三项误差预算、有效期 —— 现场没人能手写
对。于是 `volume_provider_config` 永远是空的，`volume_context` 永远是
None，`range_status` 永远是 `missing`，**每一次测量都被标成低置信**，
煤量永远进不了正式入库。算得对不对根本轮不上。

这个工具做的事
────────────────────────────────────────────────────────────────────────
· **内腔域**（`domain_xy`）—— 用 `detect.carriage_box()` 从这份基线的点云
  里**量出来**，不是猜。它按横截面宽度切，实测内宽 72 份标准差 0.067 m。
· **货高上限**（`max_height_m`）—— 厢内深（厢壁顶 - 底板）＋ 冒尖余量。
· 其余绑定标识、误差预算 —— **必须由人给**，工具不编。

▲ 生成出来的文件里 `conditions_status` 恒为 `declared_model`：
  它是**声明**，不是现场计量认证。误差预算写多少，范围就多宽；
  写小了范围会假窄，而范围罩不罩得住真值是没人替你验的。
  这一点 `app/volume_provider.py` 的枚举里写死了，本工具也改不了。

▲ 基线内容一变（重新标定、换车厢），`baseline_sha256` 就变，这份文件
  立刻失配、体积退回无范围 —— 这是**故意的**，重标定之后要重新生成。

★★ `--channel` 填**装货车道**，不是标定车道（2026-09-23 定案）
────────────────────────────────────────────────────────────────────────
体积按**载重车辆真实走的车道**计算：空车在 CH2 标定、装货走 CH4，就用
CH4 的标定、配置与车道号，从 CH2 借来的只有"这辆车的空车车厢"。
运行时 `volume_runtime` 比对 `binding["channel_id"] != channel`（测量车道），
所以 `--channel` / `--calibration-id` / `--configuration-sha256` 都要填
**装货那条道**的值。填成标定车道 -> 永远 `binding_not_found`，而且不报错。

装货车道是四选一随机的，所以**一辆车要为每条可能装货的道各生成一份**：

    for CH in CH2 CH3 CH4; do
        python tools/make_volume_model.py --baseline data/in/<车牌>.baseline.npz             --channel $CH --calibration-id <该道的> --configuration-sha256 <该道的>             ... --out config/volume-provider.json
    done

每次运行都**合并**进 `--out`（同一绑定的旧模型会被替换），不会抹掉别的
车辆或别的车道。▲ 目录上限 `MAX_MODELS = 128`，即约 32 辆车 x 4 条道。
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np                                          # noqa: E402

from app.bed_reference import baseline_digest               # noqa: E402
from app.detect import EmptyBaseline, carriage_box          # noqa: E402
from app.volume_provider import VolumeCatalog               # noqa: E402

_SHA = re.compile(r"^[0-9a-f]{64}$")


def measure(baseline: EmptyBaseline, crown: float) -> dict:
    """从基线点云量出内腔域与货高上限。量不出来就报错，不给默认值。"""
    cloud = np.asarray(baseline.cloud, np.float64)
    box = carriage_box(cloud)
    if box is None:
        raise SystemExit(
            "× 这份基线切不出车厢（carriage_box 返回 None）。\n"
            "  多半是：基线不是一辆真空车、点太少、或者整帧没裁 ROI。\n"
            "  先用 cli/carriage_cut.py 看一眼这份点云长什么样。")
    depth = float(baseline.box[5]) - float(baseline.floor_z)
    if not (0.5 <= depth <= 4.0):
        raise SystemExit("× 量出的厢内深 %.2f m 不合常理（应在 0.5~4.0），"
                         "基线可能不是空车。" % depth)
    return {"domain_xy": [box["x0"], box["x1"], box["y0"], box["y1"]],
            "inner_depth_m": depth, "max_height_m": depth + crown,
            "carriage_length_m": box["length"], "carriage_width_m": box["width"],
            "points": box["n_points"]}


def build(args, baseline: EmptyBaseline, defaults_used: bool = True) -> dict:
    m = measure(baseline, args.crown)
    now = time.time()
    x0, x1, y0, y1 = m["domain_xy"]
    model = {
        # ★ 默认 ID 带基线摘要前缀：基线按车牌存之后，同一条道上会有**多辆车**
        #   各一份模型。原来只用"车道 + 秒级时间戳"，同一秒生成两辆车就撞 ID，
        #   合并进目录时整份文件被 VolumeCatalog 以"标识重复"拒绝。
        "model_id": args.model_id or ("vm-%s-%s" % (args.channel, baseline_digest(baseline)[:12])),
        "binding": {
            "node_id": args.node_id, "channel_id": args.channel,
            "calibration_id": args.calibration_id,
            "configuration_sha256": args.configuration_sha256,
            "baseline_version_id": args.baseline_version_id,
            "baseline_asset_sha256": args.baseline_asset_sha256,
            "baseline_sha256": baseline_digest(baseline),
        },
        "reference_id": args.reference_id or ("baseline:" + args.baseline_version_id),
        "coordinate_frame": "baseline-heading-aligned-m-v1",
        "valid_from": now,
        "valid_until": now + args.valid_days * 86400.0,
        # 内腔域是量出来的；`synthetic_model` 表示"按模型声明"，
        # 换成 surveyed_interior 只有在真拿卷尺量过之后才成立。
        "domain_xy": [x0, x1, y0, y1],
        "domain_source": "surveyed_interior" if args.surveyed else "synthetic_model",
        "max_height_m": m["max_height_m"],
        "height_source": "independent_vehicle_envelope" if args.surveyed else "synthetic_model",
        "support_cell_m": args.support_cell,
        # 支持格锚在世界坐标整数倍上 —— 与 detect.carriage_box 的分格同一个
        # 道理：锚在点云自身的 min 上，每帧相位都不同，格子会跟着漂。
        "support_origin_xy": [0.0, 0.0],
        "support_model": "cell_mean_max_relative_height",
        "error_source": "independent_error_budget" if args.surveyed else "synthetic_model",
        "surface_error_m": args.surface_error,
        "bed_error_m": args.bed_error,
        "roi_error_area_m2": args.roi_error_area,
        # ▲ 这里**不能**加自定义字段记来源：VolumeCatalog 的模型是白名单校验，
        #   多一个键就整份文件作废（试过 error_basis，直接被拒）。
        #   这三个数的来源记在 docs/48 §8.3，并由本工具运行时打印出来。
        "conditions_status": "declared_model",
        "registration": ({"mode": "fixed", "dy_m": args.dy} if args.dy is not None
                         else {"mode": "estimated"}),
    }
    return {"schema": "volume-provider-v1", "models": [model]}, m


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="从空车基线生成体积物理参数文件")
    p.add_argument("--baseline", required=True, help="空车基线 npz（data/in/<CH>.baseline.npz）")
    p.add_argument("--out", default="", help="写到哪；留空只打印不落盘")
    p.add_argument("--node-id", required=True)
    p.add_argument("--channel", required=True,
                   help="**装货车辆实际测量的那条道**，不是空车标定的那条道。"
                        "体积按载重车辆真实走的车道计算（2026-09-23 定案）；"
                        "一辆车可能在哪几条道装货，就对每条道各跑一次（结果合并进 --out）")
    p.add_argument("--calibration-id", required=True)
    p.add_argument("--configuration-sha256", required=True, help="判定配置指纹的完整 64 位 sha256")
    p.add_argument("--baseline-version-id", required=True, help="后台正式基线版本号")
    p.add_argument("--baseline-asset-sha256", required=True, help="后台那份基线资产的 sha256")
    p.add_argument("--model-id", default="")
    p.add_argument("--reference-id", default="")
    p.add_argument("--crown", type=float, default=0.8,
                   help="冒尖余量（米）：货高上限 = 厢内深 + 它，默认 0.8")
    p.add_argument("--support-cell", type=float, default=0.10, help="固定支持块尺度（米）")
    # ★★ 这三个默认值是 2026-09-22 从两辆有磅单真值的车**实测**出来的，
    #    不是拍脑袋的。依据见 docs/48 §8.3：
    #      · bed-error      空车基线底板平面残差 0.021 / 0.025 m
    #      · surface-error  同一辆车三帧逐格货面高度极差 p90 = 0.241 / 0.016 m
    #      · roi-error-area 厢长帧间极差中位 1.275 m x 内宽 2.4 m
    #    以前这三个是 required —— 因为当时**没有任何实测依据**，
    #    宁可让人卡住也不能让工具编一个数出来。现在有依据了，就给默认值，
    #    但两件事必须同时做到：① 打印出来说明它们是哪来的；
    #    ② 记进生成文件的 error_basis，事后能追溯是哪一批数据定的。
    #    ▲ 只有 2 辆车撑着，现场第一天跑起来后要用同样方法重测再定稿。
    p.add_argument("--surface-error", type=float, default=0.25,
                   help="货面平均高度的代表性误差（米），含融合/采样/表面变化。"
                        "默认 0.25 = 两辆真值车实测 p90 里差的那辆")
    p.add_argument("--bed-error", type=float, default=0.03,
                   help="底板参考误差（米）。默认 0.03 = 实测底板平面残差 0.021/0.025 取整")
    p.add_argument("--roi-error-area", type=float, default=3.0,
                   help="ROI 对称差可能增减的面积（m^2）。默认 3.0 = 厢长帧间极差 x 内宽")
    p.add_argument("--valid-days", type=float, default=365.0)
    p.add_argument("--dy", type=float, default=None,
                   help="显式固定纵向配准偏移；不给则声明为 estimated（运行时估）")
    p.add_argument("--surveyed", action="store_true",
                   help="内腔与误差预算确实来自现场实测/独立预算（而不是按模型声明）")
    args = p.parse_args(argv)

    for name in ("configuration_sha256", "baseline_asset_sha256"):
        if not _SHA.match(getattr(args, name) or ""):
            print("× --%s 必须是 64 位小写十六进制" % name.replace("_", "-"))
            return 2
    if not all(v >= 0 for v in (args.surface_error, args.bed_error, args.roi_error_area)):
        print("× 误差预算不能为负")
        return 2
    if not 0 < args.support_cell <= 1.0:
        print("× --support-cell 应在 (0, 1] 米之间")
        return 2

    # 三个都没被显式给过 = 用的是实测默认值
    defaults_used = not any(flag in (argv if argv is not None else sys.argv[1:])
                            for flag in ("--surface-error", "--bed-error", "--roi-error-area"))

    baseline = EmptyBaseline.load(args.baseline)
    if baseline is None or not baseline.n:
        print("× 读不出基线：" + args.baseline)
        return 2

    catalog, m = build(args, baseline, defaults_used)

    print("=" * 78)
    print("体积物理参数 —— 从基线量出来的部分")
    print("=" * 78)
    print("  基线        : %s（%d 点）" % (args.baseline, baseline.n))
    print("  基线摘要    : %s" % catalog["models"][0]["binding"]["baseline_sha256"])
    print("  车厢        : 长 %.2f m  内宽 %.2f m（%d 点参与）"
          % (m["carriage_length_m"], m["carriage_width_m"], m["points"]))
    print("  内腔域      : x[%.2f, %.2f]  y[%.2f, %.2f]" % tuple(m["domain_xy"]))
    print("  厢内深      : %.2f m" % m["inner_depth_m"])
    print("  货高上限    : %.2f m（内深 + 冒尖 %.2f）" % (m["max_height_m"], args.crown))
    print()
    print("误差预算（只决定区间有多宽，**不参与体积计算**）")
    print("  货面误差    : %.3f m%s" % (args.surface_error,
          "   <- 默认值" if args.surface_error == 0.25 else ""))
    print("  底板误差    : %.3f m%s" % (args.bed_error,
          "   <- 默认值" if args.bed_error == 0.03 else ""))
    print("  ROI 面积误差: %.3f m^2%s" % (args.roi_error_area,
          "   <- 默认值" if args.roi_error_area == 3.0 else ""))
    if defaults_used:
        print()
        print("  ▲ 用的是 2026-09-22 两辆磅单真值车的实测默认值（docs/48 §8.3）：")
        print("      底板误差   实测底板平面残差 0.021 / 0.025 m")
        print("      货面误差   三帧逐格货面高度极差 p90 = 0.241 / 0.016 m")
        print("      ROI 面积   厢长帧间极差中位 1.275 m x 内宽 2.4 m")
        print("    只有 2 辆车撑着，现场第一天跑起来后用同样方法重测再定稿。")
    print("  来源声明    : %s" % catalog["models"][0]["conditions_status"])

    # ★★ 合并进已有目录，**不再整份覆盖**（2026-09-23）。
    #   原来每跑一次就把 --out 整个换成"只有这一个模型"的文件。而按现场定案：
    #     · 基线按车牌存 -> 目录里要有**多辆车**；
    #     · 体积按装货车辆**实际走的车道**算，binding 绑车道，而装货车道四选一
    #       随机 -> 一辆车要为**每条可能装货的道**各备一份模型。
    #   覆盖写的后果是：任何时刻最多只有一个"车辆 x 车道"组合拿得到范围，
    #   给 B 车生成就把 A 车的抹掉 —— 其余全部静默走 binding_not_found。
    #   现在：读已有目录 -> 去掉**同一绑定**的旧模型（否则有效期重叠会被拒）
    #   -> 追加这一份 -> 整份过一遍 VolumeCatalog 校验再落盘。
    replaced = 0
    if args.out and os.path.exists(os.path.abspath(args.out)):
        with open(os.path.abspath(args.out), "rb") as handle:
            existing = json.loads(handle.read().decode("utf-8"))
        new_model = catalog["models"][0]
        kept = [x for x in existing.get("models", []) if x.get("binding") != new_model["binding"]]
        replaced = len(existing.get("models", [])) - len(kept)
        catalog = {"schema": "volume-provider-v1", "models": kept + [new_model]}
        print()
        print("  合并进已有目录：原有 %d 个模型，替换同绑定 %d 个，合并后 %d 个"
              % (len(existing.get("models", [])), replaced, len(catalog["models"])))

    data = json.dumps(catalog, ensure_ascii=False, indent=2).encode("utf-8")
    try:
        VolumeCatalog.from_bytes(data)                       # 自己先过一遍校验
    except ValueError as exc:
        print()
        print("× 生成的文件过不了 VolumeCatalog 校验：%s" % exc)
        return 1
    print()
    print("√ 已通过 VolumeCatalog 校验")

    if not args.out:
        print()
        print("（没给 --out，只打印不落盘）")
        print(data.decode("utf-8"))
        return 0
    path = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "wb") as handle:
        handle.write(data)
    print("√ 已写入 " + path)
    print()
    print("接下来：")
    print("  python cli/run.py --mode live --webapi-config <...> \\")
    print("      --volume-provider-config %s" % args.out)
    print()
    print("▲ 重新标定基线之后这份文件会立刻失配（baseline_sha256 变了），")
    print("  体积退回无范围。那是**故意的** —— 重标定后要重新生成一份。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
