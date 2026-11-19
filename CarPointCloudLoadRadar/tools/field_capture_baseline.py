# -*- coding: utf-8 -*-
"""现场标定空车基线：服务运行中调用看板接口，**必须显式给真实车牌**。

用法（本仓库目录下）：
    python tools/field_capture_baseline.py --channel CH2 --plate 宁A25771

什么时候用：
    · 车是**空车**，已经在车道里停稳（看板这条道显示"已停稳"或已出结论），司机别动；
    · 车牌填**真实车牌** —— 现在的车牌接口是模拟的，服务自己读到的不是真牌，
      不填真牌的话基线会存到一个随机号下，同一条道 60 秒内的下一辆车会拿它算体积且不报错。

结果：
    · 成功：生成 data/in/<车牌>.baseline.npz；
    · 不论成败都在 data/field-log/基线标定记录-<日期>.csv 追加一行；
    · 退出码 0 = 成功，1 = 服务拒绝（按提示处理），2 = 连不上服务。

只调现有的 POST /api/baseline/capture，不改服务的任何行为。
"""
import argparse
import csv
import json
import os
import sys
import time
import urllib.error
import urllib.request

try:                                    # tools/ 下的脚本没有 _common.setup_console，这里自己保证中文能打出来
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:                       # noqa: BLE001
    pass

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 服务拒绝时，按消息里的关键字给现场一句能照做的话
ADVICE = (
    ("帧间互相关", "车还没完全静止或场景在变：让司机保持不动再等 10 秒，重新执行本命令（最多再试 2 次）。"),
    ("连续完整的新帧", "这条道现在不算停稳：等看板这条道显示「已停稳」后再执行。"),
    ("至少需要连续三帧", "停稳的帧还不够：再等 3 秒重新执行。"),
    ("当前没有可用的点云帧", "服务收不到这条道的雷达数据：看板这张卡片是不是「等待雷达数据」？车道号填对了吗？"),
    ("底面不够水平", "这次扫描不适合当基线：让司机把车停正（车道中间、停到位）后再试一次；还不行就记下来，回去离线处理。"),
    ("六面拟合失败", "这次扫描不适合当基线：让司机把车停正后再试一次；还不行就记下来，回去离线处理。"),
    ("点云过少", "点云太少：确认车在这条道的检测区里，再试一次。"),
    ("观测已变化", "标定过程中车动了：等重新停稳后再执行。"),
    ("识别不到车牌", "车牌没传进去：检查命令里的 --plate。"),
    ("已由业务后台接管", "现在是接后台模式，基线要在后台发起，本命令不适用。"),
)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="现场标定空车基线（必须给真实车牌）")
    ap.add_argument("--channel", required=True, choices=["CH1", "CH2", "CH3", "CH4"], help="空车停在哪条道")
    ap.add_argument("--plate", required=True, help="真实车牌，如 宁A25771")
    ap.add_argument("--url", default="http://127.0.0.1:8080", help="服务看板地址")
    ap.add_argument("--record", default="", help="记录文件（默认 data/field-log/基线标定记录-<日期>.csv）")
    a = ap.parse_args(argv)

    plate = a.plate.strip()
    if not plate or plate == "未知车牌" or len(plate) < 7:
        print("车牌看起来不对：%r。请填完整的真实车牌（如 宁A25771）。" % plate)
        return 1
    record = a.record or os.path.join(ROOT, "data", "field-log", "基线标定记录-%s.csv" % time.strftime("%Y%m%d"))

    body = json.dumps({"channel": a.channel, "plate": plate}).encode("utf-8")
    req = urllib.request.Request(a.url.rstrip("/") + "/api/baseline/capture", data=body,
                                 headers={"Content-Type": "application/json"})
    # 回环地址不走系统代理（本机配了代理时，普通请求会被代理返回 502）
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(req, timeout=60) as resp:
            reply = json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        try:
            reply = json.loads(error.read().decode("utf-8"))
        except Exception:                                       # noqa: BLE001
            reply = {"ok": False, "message": "HTTP %s" % error.code}
    except Exception as error:                                  # noqa: BLE001
        print("连不上服务（%s）：%s" % (a.url, error))
        print("服务启动了吗？（python -u cli/run.py --mode live）")
        return 2

    ok = bool(reply.get("ok"))
    message = str(reply.get("message") or reply.get("error") or "")
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    print("=" * 70)
    print("%s  %s  %s" % (stamp, a.channel, plate))
    print(("√ 标定成功：" if ok else "× 没有标定：") + message)
    if not ok:
        for key, text in ADVICE:
            if key in message:
                print("  怎么办：" + text)
                break
        else:
            print("  怎么办：把上面这句原样记下来，回去离线处理（空车这次的归档里有整帧，可以离线重建基线）。")
    print("=" * 70)

    try:
        os.makedirs(os.path.dirname(record), exist_ok=True)
        new = not os.path.exists(record)
        with open(record, "a", newline="", encoding="utf-8-sig") as fh:
            writer = csv.writer(fh)
            if new:
                writer.writerow(["时间(电脑)", "车道", "真实车牌", "结果", "说明"])
            writer.writerow([stamp, a.channel, plate, "成功" if ok else "失败", message])
        print("已记录到 " + record)
    except OSError as error:
        print("记录文件写不进去（不影响标定结果）：%s" % error)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
