# -*- coding: utf-8 -*-
"""现场一键巡检（只读）：每条道收没收到数据、测了几辆、入库几条、有没有告警横幅、磁盘够不够。

用法（本仓库目录下，服务运行中）：
    python tools/field_patrol.py
    python tools/field_patrol.py --log logs/run_20260924.log

看什么：
    · 每条道"测了"（日志里的 [result_pending]）应当和实际过车数对得上 —— 测没测只认这个；
    · "入库"应当等于"测了"（差 1 以内是正在提交）；
    · 横幅不为空、残缺帧 > 5%、帧龄 > 2 秒（收不到数据）都会列在最后的"需要处理"里。
不改任何东西，也不需要外网。
"""
import argparse
import glob
import json
import os
import re
import shutil
import sqlite3
import sys
import time
import urllib.request

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:                       # noqa: BLE001
    pass

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main(argv=None) -> int:
    day = time.strftime("%Y%m%d")
    ap = argparse.ArgumentParser(description="现场一键巡检（只读）")
    ap.add_argument("--url", default="http://127.0.0.1:8080")
    ap.add_argument("--log", default=os.path.join(ROOT, "logs", "run_%s.log" % day), help="服务日志（tee 出来的那份）")
    ap.add_argument("--field", default=os.path.join(ROOT, "data", "field"), help="落盘目录")
    a = ap.parse_args(argv)
    todo = []

    # ① 看板状态（回环地址不走系统代理）
    state = None
    try:
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(a.url.rstrip("/") + "/api/state", timeout=5) as resp:
            state = json.loads(resp.read().decode("utf-8"))
    except Exception as error:                                  # noqa: BLE001
        todo.append("看板接口连不上（%s）：服务还在跑吗？" % error)

    # ② 服务日志：每条道测了几辆 / 开了几个会话 / 关键告警
    measured, sessions, warn_hits = {}, {}, {}
    keys = {"内存预算挤掉": "丢整帧", "暂停新测量": "终局器卡住", "停稳超过": "停不稳告警", "沉降窗口已作废": "沉降作废"}
    if os.path.exists(a.log):
        pat = re.compile(r"\[(result_pending|session_start|warn|error)\]\s+(CH\d)")
        with open(a.log, encoding="utf-8", errors="replace") as fh:
            for line in fh:
                m = pat.search(line)
                if not m:
                    continue
                kind, ch = m.group(1), m.group(2)
                if kind == "result_pending":
                    measured[ch] = measured.get(ch, 0) + 1
                elif kind == "session_start":
                    sessions[ch] = sessions.get(ch, 0) + 1
                for k, name in keys.items():
                    if k in line:
                        warn_hits[(ch, name)] = warn_hits.get((ch, name), 0) + 1
    else:
        todo.append("找不到服务日志 %s：服务是不是没用 tee 落日志？" % a.log)

    # ③ 归档库：每条道入库几条
    archived, pending = {}, {}
    db = os.path.join(a.field, "archives-v1", "archive.sqlite3")
    if os.path.exists(db):
        try:
            con = sqlite3.connect("file:%s?mode=ro" % db, uri=True, timeout=5)
            for ch, status, n in con.execute("select channel, status, count(*) from cloud_archives group by channel, status"):
                if status == "complete":
                    archived[ch] = archived.get(ch, 0) + n
                else:
                    pending[ch] = pending.get(ch, 0) + n
            con.close()
        except Exception as error:                              # noqa: BLE001
            todo.append("归档库读不了：%s" % error)

    # ④ 进场帧与磁盘
    enter = {}
    for p in glob.glob(os.path.join(a.field, "enter", "CH*", day + "_*.xyz")):
        ch = os.path.basename(os.path.dirname(p))
        n, size = enter.get(ch, (0, 0))
        enter[ch] = (n + 1, size + os.path.getsize(p))
    free_gb = shutil.disk_usage(ROOT).free / 2 ** 30

    print("=" * 78)
    print("巡检 %s    磁盘剩余 %.0f GB" % (time.strftime("%Y-%m-%d %H:%M:%S"), free_gb))
    print("-" * 78)
    print("%-4s %-4s %6s %8s %7s %6s %6s %6s %8s %11s" % ("车道", "在线", "帧/秒", "包/秒", "残缺%", "会话", "测了", "入库", "终局器", "进场帧"))
    lanes = [c for c in (state or {}).get("channels", [])] or [{"id": ch} for ch in sorted(set(measured) | set(archived))]
    for c in lanes:
        ch = c.get("id")
        src = c.get("source") or {}
        fin = (c.get("finalizer") or {}).get("state", "-")
        inc = c.get("incomplete_ratio")
        en = enter.get(ch, (0, 0))
        print("%-4s %-4s %6s %8s %7s %6s %6d %6d %8s %11s" % (
            ch, "是" if c.get("online") else "否", "%.1f" % c["fps"] if isinstance(c.get("fps"), (int, float)) else "-",
            "%.0f" % src["packet_rate"] if isinstance(src.get("packet_rate"), (int, float)) else "-",
            "%.1f" % (100 * inc) if isinstance(inc, (int, float)) else "-", sessions.get(ch, 0),
            measured.get(ch, 0), archived.get(ch, 0), fin, "%d 帧/%.1fG" % (en[0], en[1] / 2 ** 30)))
        if c.get("health"):
            todo.append("%s 横幅：%s" % (ch, str(c["health"])[:120]))
        if isinstance(inc, (int, float)) and inc > 0.05:
            todo.append("%s 残缺帧 %.1f%%（>5%%）：网络丢包，查交换机/网线；>10%% 要重启服务" % (ch, 100 * inc))
        age = c.get("frame_age_sec")
        if state and c.get("online") and not (src.get("packet_rate") or 0) > 0:
            todo.append("%s 收不到雷达报文（包/秒 = 0）：查这台雷达的网线、供电和目标 IP" % ch)
        elif state and c.get("online") and isinstance(age, (int, float)) and age > 2:
            todo.append("%s 已 %.0f 秒没收到新帧：查这台雷达的网线/供电" % (ch, age))
        if measured.get(ch, 0) - archived.get(ch, 0) > 1:
            todo.append("%s 测了 %d 辆但只入库 %d 条：看这条道的横幅" % (ch, measured.get(ch, 0), archived.get(ch, 0)))
        if pending.get(ch):
            todo.append("%s 有 %d 条归档还没完成" % (ch, pending[ch]))
    for (ch, name), n in sorted(warn_hits.items()):
        if name in ("丢整帧", "终局器卡住"):
            todo.append("%s 日志里出现「%s」%d 次" % (ch, name, n))
    if free_gb < 20:
        todo.append("磁盘只剩 %.0f GB：考虑把 enter_stride 调成 2 或暂停进场帧" % free_gb)
    print("-" * 78)
    if todo:
        print("需要处理：")
        for t in todo:
            print("  ▲ " + t)
    else:
        print("√ 没有需要处理的问题")
    print("=" * 78)
    return 0


if __name__ == "__main__":
    sys.exit(main())
