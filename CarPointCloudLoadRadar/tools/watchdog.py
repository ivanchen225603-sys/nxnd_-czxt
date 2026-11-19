# -*- coding: utf-8 -*-
"""看门狗：把 cli/run.py 托管起来，崩了自动拉起 —— 无人值守采集用。

为什么需要它：麒麟/Linux 上用 systemd 的 `Restart=always` 就够了（见 docs/02 §八），
**Windows 上没有对应机制**。手工在控制台起的服务，进程一崩、机器一重启就没了，
而现场采集常常要跑一整天没人看。

判据只有两条，刻意做得很窄：

  1. **进程退出** -> 拉起
  2. **HTTP 接口连续无响应** -> 判定卡死，杀掉重启

★ 通道离线**不重启**。雷达断电、网线松了、CH1 根本没接 —— 这些是设备问题，
  重启服务一点用没有，反而会陷入"重启 -> 还是离线 -> 再重启"的死循环。
  这类情况只记日志。这条边界是这个工具能不能安心放着跑的关键。

退避：连续失败时逐次拉长重启间隔（5s -> 10s -> ... -> 300s 封顶），
避免配置写错时每秒重启一次把日志刷爆、把 CPU 吃满。
连续成功运行超过 `--healthy-sec` 就把退避清零。

用法：
    python tools/watchdog.py                     # 托管 cli/run.py --mode live
    python tools/watchdog.py --max-restarts 20   # 重启够这么多次就放弃并退出
    python tools/watchdog.py --dry-run           # 只打印将要执行的命令，不真跑

日志：logs/watchdog.log（服务自身的输出仍在 logs/run_<日期>.log）

▲ 本工具**不注册开机自启** —— 那要改系统设置，得由人来做。
  注册方法见文件末尾的说明，或 docs/02 §八（Linux）。
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

BACKOFF_START = 5.0
BACKOFF_MAX = 300.0
BACKOFF_FACTOR = 2.0


def next_backoff(cur: float) -> float:
    """退避时长递增，封顶 BACKOFF_MAX。抽成纯函数是为了能单测。"""
    return min(BACKOFF_MAX, max(BACKOFF_START, cur * BACKOFF_FACTOR))


def should_restart(proc_alive: bool, api_ok: bool, api_bad_sec: float,
                   api_grace_sec: float) -> tuple:
    """要不要重启，返回 (要不要, 原因)。

    ★ 只认两条：进程没了、接口卡死。**通道离线不在其列** ——
      那是设备问题（雷达断电/网线松/压根没接），重启服务修不了，
      反而会变成无限重启。这条边界写死在这里，别放宽。
    """
    if not proc_alive:
        return True, "进程已退出"
    if not api_ok and api_bad_sec >= api_grace_sec:
        return True, "HTTP 接口连续 %.0f 秒无响应，判定卡死" % api_bad_sec
    return False, ""


def probe(url: str, timeout: float = 4.0):
    """探一次 /api/state。返回 (通不通, 在线通道数, 总通道数)。

    只读，不改任何状态。
    """
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            d = json.loads(r.read().decode("utf-8"))
    except (urllib.error.URLError, OSError, ValueError, TimeoutError):
        return False, 0, 0
    chs = d.get("channels") or []
    return True, sum(1 for c in chs if c.get("online")), len(chs)


def _log(fp, msg: str) -> None:
    line = "%s %s" % (time.strftime("%m-%d %H:%M:%S"), msg)
    print(line, flush=True)
    if fp:
        fp.write(line + "\n")
        fp.flush()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="托管 cli/run.py，崩了自动拉起")
    ap.add_argument("--url", default="http://127.0.0.1:8080/api/state")
    ap.add_argument("--interval", type=float, default=10.0, help="健康检查间隔秒（默认 10）")
    ap.add_argument("--api-grace", type=float, default=60.0,
                    help="接口连续无响应多少秒才判定卡死（默认 60）")
    ap.add_argument("--startup-grace", type=float, default=45.0,
                    help="刚拉起后多少秒内不做接口判定（默认 45）")
    ap.add_argument("--max-restarts", type=int, default=0,
                    help="重启上限，0 = 不限")
    ap.add_argument("--log", default="", help="日志文件（默认 logs/watchdog.log）")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args(argv)

    cmd = [sys.executable, os.path.join(ROOT, "cli", "run.py"), "--mode", "live"]
    if args.dry_run:
        print("将要执行：", " ".join(cmd))
        print("健康检查：", args.url, "每", args.interval, "秒")
        return 0

    log_path = args.log or os.path.join(ROOT, "logs", "watchdog.log")
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    fp = open(log_path, "a", encoding="utf-8")

    env = dict(os.environ)
    env.setdefault("PYTHONIOENCODING", "utf-8")

    restarts, backoff = 0, BACKOFF_START
    proc = None
    try:
        while True:
            stamp = time.strftime("%Y%m%d")
            out_path = os.path.join(ROOT, "logs", "run_%s.log" % stamp)
            out = open(out_path, "a", encoding="utf-8", errors="replace")
            _log(fp, "启动服务 -> %s" % out_path)
            proc = subprocess.Popen(cmd, cwd=ROOT, stdout=out, stderr=subprocess.STDOUT,
                                    env=env)
            started = time.time()
            api_bad_since = 0.0
            last_online = -1

            while True:
                time.sleep(args.interval)
                alive = (proc.poll() is None)
                age = time.time() - started

                api_ok, online, total = (True, -1, -1)
                if alive and age >= args.startup_grace:
                    api_ok, online, total = probe(args.url)
                    if api_ok:
                        api_bad_since = 0.0
                        if online != last_online:
                            # 通道数变了只记录，**不重启** —— 设备问题重启修不了
                            _log(fp, "通道在线 %d/%d" % (online, total))
                            last_online = online
                            if online == 0:
                                _log(fp, "  ! 全部通道离线 —— 查雷达供电/网线/交换机。"
                                         "看门狗不会因此重启（重启修不了设备问题）")
                    elif api_bad_since == 0.0:
                        api_bad_since = time.time()

                bad_sec = (time.time() - api_bad_since) if api_bad_since else 0.0
                need, why = should_restart(alive, api_ok, bad_sec, args.api_grace)
                if not need:
                    if age > 300 and backoff > BACKOFF_START:
                        backoff = BACKOFF_START   # 稳定跑够 5 分钟，退避清零
                    continue

                _log(fp, "需要重启：%s" % why)
                if alive:
                    proc.terminate()
                    try:
                        proc.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        _log(fp, "  未能优雅退出，强杀")
                        proc.kill()
                out.close()
                restarts += 1
                if args.max_restarts and restarts >= args.max_restarts:
                    _log(fp, "已达重启上限 %d 次，看门狗退出" % args.max_restarts)
                    return 1
                _log(fp, "第 %d 次重启，%.0f 秒后拉起" % (restarts, backoff))
                time.sleep(backoff)
                backoff = next_backoff(backoff)
                break
    except KeyboardInterrupt:
        _log(fp, "收到 Ctrl-C，停止看门狗")
        if proc and proc.poll() is None:
            proc.terminate()
        return 0
    finally:
        fp.close()


if __name__ == "__main__":
    sys.exit(main())

# ---------------------------------------------------------------------------
# 开机自启（需要人工执行，本工具不代劳 —— 那是改系统设置）
#
# Windows：以管理员身份跑一次，登录后自动起：
#   schtasks /create /tn "LidarWatchdog" /tr "python D:\...\tools\watchdog.py" ^
#            /sc onlogon /rl highest
#   取消：schtasks /delete /tn "LidarWatchdog" /f
#
# 麒麟 / Linux：用 systemd，见 docs/02 §八（Restart=always，比本工具更可靠）。
# ---------------------------------------------------------------------------
