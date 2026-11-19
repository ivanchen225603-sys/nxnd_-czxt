# -*- coding: utf-8 -*-
"""现场停稳记录器 —— 每辆车一组三行：进场 / 判到停稳 / 离场；并驱动 CH4 双雷达采集。

为什么单独有它（2026-09-21 现场）：
    看板给的是瞬时状态，日志给的是 `拦在 xxx x173` 这样的计数，
    两者都回答不了"今天来了几辆车、有几辆判到了停稳"。这个脚本
    只盯这一件事，一辆车三行，可以直接数。

★ 2026-11-02 起双雷达采集**默认关闭**：CH4 改由载货雷达服务在"位置停稳"时经
    `passages/standstill` 通知 WebApi 触发（根目录 `docs/ch4-standstill-dimension-trigger-remediation-20261102.md`）。
    下面的采集驱动只在显式 `--capture-channel CH4` 时启用，两条触发不能同时开。

★ 2026-09-24 起它多了一项职责：**CH4 车辆测完之后自动驱动双雷达采集**
    （`docs/68-CH4双雷达采集触发与进度监控方案.md`）。
    主线程只产生车辆事件、入队、不做任何 HTTP；真正的采集与进度轮询在
    `tools/dual_radar_capture.py` 的后台线程里。WebApi 慢或挂了，停稳记录照常。

★ 进出场判据不能只看车点数 —— 残缺帧会骗过它

    2026-09-21 实测：CH3 一辆车停得好好的，15:21:36 起连续 16 帧残缺
    （丢包），ROI 点数在 700~116000 之间乱跳。按"车点 < 阈值就算离场"
    的写法，这**一辆**车被记成了**三个**车次（3.4s + 0.0s + 0.0s），
    判稳率被拉低。而车实际一直在，15:21:40 点数还回到 116395。

    所以这里做两件事：
      1. **残缺帧完全跳过** —— 不参与进出场判断，也不刷新峰值。
         残缺帧缺的是一整块连续扇形，点数没有意义。
         这与 `StabilityDetector` 里"已经判成有车时残缺帧一律跳过"
         是同一个道理（见 CLAUDE.md 第 7b 条，防的是同一辆车被测两遍）。
      2. **离场要连续确认** —— 连续 `--leave-frames` 次采样（默认 2，
         按 2 秒一次即 4 秒）可信且点数低于门槛，才算走了。

▲▲ 采样间隔默认 2 秒，**别调回 0.1**（2026-09-24 现场实测）：
   `/api/state` 每次要在服务里现拼整份快照（当时 235 KB、约 300 ms，
   且随当天测量数增长），按 0.1 秒轮询等于让服务一直忙着拼快照、
   收包线程抢不到解释器 —— 系统层 UDP 缓冲溢出丢包，
   三条道残缺帧从 0% 涨到 38~45%（残缺帧 >10% 时绝大多数车测不出来）。
   停掉本工具 5 秒内回到 0%；改成 2 秒一次后 30 秒实测仍为 0%。

▲ 单通道处理有异常兜底：新增的事件判定一旦抛错，只打一行并跳过本通道本次采样，
  **不允许让整个进程退出**（进程重启会把内存里的车辆状态清零，而
  "记录器被看门狗重启"是常态 —— 9-24 当天被拉起 5 次）。跨重启的状态延续靠
  `data/field-log/双雷达采集-台账.json`（方案 §4.2 / §8.1）。

用法：
    python tools/field_standstill_log.py                  # 打到屏幕
    python tools/field_standstill_log.py --out 某文件      # 同时写文件
    python tools/field_standstill_log.py --enter 15000    # 进场门槛
    python tools/field_standstill_log.py --no-capture     # 只要停稳记录，不驱动双雷达

双雷达采集使用 WebApi 的 CH4 固定请求头，不需要后台账号密码。
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
import urllib.request

try:                                    # GBK 控制台里中文会崩（见 tests/test_encoding.py）
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:                       # noqa: BLE001
    pass

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:                # 脚本以 `python tools/xxx.py` 跑时 sys.path[0] 是 tools/
    sys.path.insert(0, ROOT)

from tools import dual_radar_capture as drc   # noqa: E402

# 这些触发标记表示"这一帧本身不可信"，点数不能拿来判进出场。
BAD_FRAME = ("frame_incomplete", "frame_nonfinite", "frame_identity")

# 离场行末尾追加的双雷达结论（按台账里最后一次已知阶段）
_DUAL_TAIL = {
    "waiting": "未采到完整场",
    "captured": "已捕获，等待落盘",
    "saved": "√ 已落盘",
    "queued": "已落盘，排队分析",
    "analyzing": "已落盘，分析中",
    "completed": "√ 已落盘、已入库",
    "pending_review": "已计算，待人工审核",
    "completed_without_dimensions": "已结束，本次不产出尺寸",
    "capture_failed": "× 采集失败",
    "persistence_failed": "× 落盘失败",
    "analysis_failed": "× 分析失败",
    "cancelled": "离场前未采到，已取消",
    "unknown": "未确认",
}

# 采样间隔下限：/api/state 很重（CLAUDE.md），轮询太密会让服务丢包
MIN_INTERVAL = 1.0
# 读不到 /api/state 时多久提示一次
READ_FAIL_WARN_SEC = 60.0
_LOCAL_OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def _triggers(stab) -> str:
    value = stab.get("triggers") or []
    if isinstance(value, dict):
        return " ".join(str(k) for k in value)
    return " ".join(str(x) for x in value)


def _new_row() -> dict:
    return {"in": False, "peak": 0., "stable": False, "low": 0, "skipped": 0,
            # ↓ 双雷达采集用（方案 §9.2 第 3 条）
            "mid0": "", "measured": False, "moving": False,
            "session_prev": None, "state_prev": None, "plate": "",
            # ↓ 近期帧率（负载闸门用）：相邻两次采样的 frames 差 / 时间差
            "frames_prev": None, "t_prev": None, "recent_fps": None}


def _current_measurement_id(ch) -> str:
    """服务自己认定的"本车当前结果"编号（方案 §2.2，比 `last_result` 严）。"""
    current = ch.get("current_result")
    if isinstance(current, dict):
        return str(current.get("measurement_id") or "")
    return ""


def _measured_plate(ch, mid) -> str:
    """**这一份测量**读到的车牌；`last_result` 对不上这份测量号时返回 ""。

    ▲ 原来先取 `last_result.plate`、没有再取 `last_plate`，**不核对测量号** ——
      新车进场时 `last_result` 还是上一辆车的（服务到下一份结果提交才换），
      于是一进场就把**上一辆车**的车牌台账条目重新打开、字段清空。
    """
    result = drc.matched_result(ch, mid)
    return str((result or {}).get("plate") or "")


def _update_recent_fps(ch, row, mono) -> None:
    """按相邻两次采样的 `frames` 差算近期帧率，存进 `row["recent_fps"]`。

    ▲ `/api/state` 里的 `fps` 是数据源启动以来的平均值，跑久了几乎不动，不能当负载闸门。
    """
    frames = ch.get("frames")
    if isinstance(frames, bool) or not isinstance(frames, (int, float)):
        return
    if row["frames_prev"] is not None and row["t_prev"] is not None:
        dt = mono - row["t_prev"]
        if dt >= 0.5 and frames >= row["frames_prev"]:
            row["recent_fps"] = (frames - row["frames_prev"]) / dt
        elif frames < row["frames_prev"]:
            row["recent_fps"] = None            # 服务重启过，计数从头来
    row["frames_prev"] = frames
    row["t_prev"] = mono


def _dual_tail(ledger, plate, now) -> str:
    """离场行末尾的双雷达结论。台账没记录时返回"未触发"。"""
    if not plate:
        return "双雷达：未触发（未取到车牌）"
    entry = ledger.open_entry(plate, now) if ledger is not None else None
    if not entry or not (entry.get("last_stage") or entry.get("workflow_id")):
        return "双雷达：未触发"
    stage = entry.get("last_stage")
    return "双雷达：%s" % _DUAL_TAIL.get(stage, str(stage or "未确认"))


def _handle_channel(ch, row, now, args, ledger, say, session, seen) -> None:
    """单通道一次采样的全部处理（由 main 的循环在 try/except 里调用）。"""
    stab = ch.get("stability") or {}
    cid = ch.get("id", "?")
    n = int(stab.get("n_vehicle_points") or 0)
    sec = float(stab.get("stationary_sec") or 0.)
    session_id = ch.get("session")
    watching = cid == args.capture_channel and session is not None
    _update_recent_fps(ch, row, time.monotonic())     # 残缺帧也要算：帧率与这一帧好坏无关

    # ★ 残缺帧：整块跳过，点数无意义
    if stab.get("degraded") or any(bad in _triggers(stab) for bad in BAD_FRAME):
        row["skipped"] += 1
        return

    # 启动自检（方案 §4.6）：服务没开落盘时测量号永远不产生，这个失败模式是静默的
    if watching and not seen["save_check"]:
        if drc.needs_save_selfcheck(ch.get("last_result")):
            seen["save_check"] = True
            say("%s %-4s ▲▲ 测量服务未产生测量号（未开落盘？），双雷达采集不会触发"
                % (now, cid))

    if not row["in"]:
        if n >= args.enter:
            row.update({"in": True, "peak": 0., "stable": False, "low": 0, "skipped": 0,
                        "mid0": _current_measurement_id(ch), "measured": False,
                        "moving": False, "session_prev": session_id,
                        "state_prev": stab.get("state"), "plate": ""})
            say("%s %-4s 车进场" % (now, cid))
            if watching:
                # ★ 调管理器的 on_*（方案 §9.2 第 4 条）：管理器的方法名带 on_ 前缀，
                #   写成 session.entered(...) 会 AttributeError，被本通道兜底吞掉
                #   -> 双雷达采集**一次都不会触发**，而且只在屏幕上一行小字里露出来。
                session.on_entered(time.time())
        else:
            row["session_prev"] = session_id
            row["state_prev"] = stab.get("state")
        return

    row["peak"] = max(row["peak"], sec)
    if stab.get("state") == 2 and not row["stable"]:
        row["stable"] = True
        say("%s %-4s ★ 判到【已停稳】 停稳 %.1f s" % (now, cid, sec))
        if watching:
            session.on_stable(time.time())          # §8.2 的 vehicle_times.stable

    if watching:
        # 测完：本车"当前结果编号"从进场时记下的值变成了新值（方案 §4.1）
        mid = _current_measurement_id(ch)
        if mid and not row["measured"] and mid != row["mid0"]:
            row["measured"] = True
            plate_raw = _measured_plate(ch, mid)
            plate_now = drc.normalize_plate(plate_raw)
            if plate_now and ledger is not None:
                # 台账的"进场"在**车牌确定属于这辆车**时才写（原来一进场就写，
                # 拿到的是上一辆车的牌）。离场时按同一个车牌关掉。
                ledger.open_vehicle(plate_now, time.time())
                row["plate"] = plate_now
            state = dict(ch)
            state["recent_fps"] = row["recent_fps"]
            session.on_measured(time.time(), mid, plate_raw, state)
        # 开始移动 / 重新停住（方案 §7.1）
        if row["measured"]:
            if stab.get("state") == 2:
                if row["moving"]:
                    row["moving"] = False
                    session.on_stopped(time.time())
            elif (row["state_prev"] == 2 and not row["moving"]
                  and session_id == row["session_prev"] and n >= args.enter):
                # ▲ 判据是"从 2 变成非 2"，且会话号未变 —— 停着不走的车每 180 秒被
                #   服务强开新会话，state 会短暂从 2 掉到 1，不排除就会每 3 分钟误报一次。
                row["moving"] = True
                session.on_moving(time.time())
    row["session_prev"] = session_id
    row["state_prev"] = stab.get("state")

    row["low"] = row["low"] + 1 if n < args.enter * args.leave_ratio else 0
    if row["low"] >= args.leave_frames:
        tail = _dual_tail(ledger, row["plate"], time.time()) if watching else ""
        say("%s %-4s 车离场  最大停稳 %.1f s  %s  判定「%s」 saved=%d%s%s"
            % (now, cid, row["peak"],
               "√ 判到停稳" if row["stable"] else "x 全程未判稳",
               ch.get("last_reason") or "-", ch.get("saved_count") or 0,
               ("  （本次跳过 %d 个残缺帧）" % row["skipped"]) if row["skipped"] else "",
               ("  " + tail) if tail else ""))
        if watching:
            if ledger is not None and row["plate"]:
                ledger.close_vehicle(row["plate"], time.time())
            if session is not None:
                session.on_left(time.time())
        row["in"] = False


def _poll_once(url, state, args, ledger, say, session, seen, opener=None) -> dict:
    """读一次 `/api/state`，处理每条道。

    ▲ 单独抽出来是为了**可测**（`tests/test_dual_radar_capture.py` 直接喂一份快照，
      不起服务、不起线程、不真等待）；主循环只负责按时调它。
    """
    # 现场电脑有系统代理，默认 urlopen 会把 127.0.0.1 送往代理并收到 502。
    # 本接口只访问本机看板，显式禁用代理；测试仍可注入替身 opener。
    opener = opener or _LOCAL_OPENER.open
    with opener(url, timeout=3) as response:
        data = json.load(response)
    now = time.strftime("%H:%M:%S")
    for ch in data.get("channels", []):
        cid = ch.get("id", "?")
        row = state.setdefault(cid, _new_row())
        try:
            # ★ 单通道异常兜底：任何抛错只跳过本通道本次采样，
            #   绝不让整个进程退出（进程重启 = 内存状态清零，见文件头）。
            _handle_channel(ch, row, now, args, ledger, say, session, seen)
        except Exception as error:                                  # noqa: BLE001
            say("%s %-4s ▲ 本通道本次采样处理出错，已跳过：%s: %s"
                % (now, cid, type(error).__name__, error))
    return data


def _build_capture(args, say, lock):
    """按参数把采集管理器装起来。返回 `(manager, ledger)`。

    管理器起不来时返回 `(None, None)` —— 停稳记录照常。
    """
    if not args.capture_channel:
        return None, None
    cfg = drc.CaptureConfig(
        base_url=args.capture_webapi, channel=args.capture_channel,
        token_file=args.capture_token_file,
        poll_sec=args.capture_poll, ledger_ttl=args.ledger_ttl,
        guard_fps=args.guard_fps, guard_incomplete=args.guard_incomplete,
        guard_enabled=not args.no_load_guard, bell=not args.no_bell)
    try:
        manager, ledger, _client = drc.build_manager(cfg, say, lock=lock)
        return manager, ledger
    except Exception as error:                                      # noqa: BLE001
        say("▲ 双雷达采集管理器起不来（%s），只做停稳记录" % error)
        return None, None


def build_parser() -> argparse.ArgumentParser:
    """记录器的命令行。

    ▲ 单独抽出来是为了**可测**：看门狗会把这套参数拼成 argv 透传进来（方案 §9.3 / D8），
      两边一旦对不上就会"启动即报错退出"，那是只有到现场才会发现的失败。
      `tests/test_dual_radar_capture.py::TestWatchdogPassthrough` 直接拿看门狗的输出去喂它。
    """
    ap = argparse.ArgumentParser(description="现场停稳记录器（含 CH4 双雷达采集触发）")
    # ?history=0：只用各通道状态字段，不要最近结果 —— 服务省掉帧锁内对 60 条结果的深拷贝
    # （约 8 ms/道/次）和约八成的传输量。旧版服务忽略该参数、照常返回全量，记录器不受影响。
    ap.add_argument("--url", default="http://127.0.0.1:8080/api/state?history=0")
    ap.add_argument("--out", default="", help="除屏幕外再追加写这个文件")
    ap.add_argument("--enter", type=int, default=15000, help="车辆点达到它算进场")
    ap.add_argument("--leave-ratio", type=float, default=0.5, help="低于 enter x 它才考虑离场")
    ap.add_argument("--leave-frames", type=int, default=2, help="连续这么多次可信采样低于门槛才算离场")
    ap.add_argument("--interval", type=float, default=2.0,
                    help="采样间隔（秒）。别低于 1：/api/state 很重，轮询太密会让服务丢包")
    # ── 双雷达采集（方案 §4.6）；注意基址参数不叫 --webapi，见 D8
    ap.add_argument("--capture-webapi", default=drc.DEFAULT_CAPTURE_WEBAPI,
                    help="车牌/采集 WebApi 基址（默认 %s）" % drc.DEFAULT_CAPTURE_WEBAPI)
    ap.add_argument("--capture-token-file", default=drc.DEFAULT_TOKEN_FILE,
                    help="CH4 采集私有令牌文件，只传路径，不在命令行传令牌")
    ap.add_argument("--capture-channel", default="",
                    help="触发双雷达采集的车道，只接受 CH4；默认空串 = 关闭（已由服务的位置停稳通知取代）")
    ap.add_argument("--no-capture", action="store_true", help="关闭双雷达采集触发")
    ap.add_argument("--no-bell", action="store_true", help="关掉报警响铃")
    ap.add_argument("--ledger-ttl", type=float, default=drc.DEFAULT_LEDGER_TTL,
                    help="台账条目最长开着多少秒（默认 1800）")
    ap.add_argument("--guard-fps", type=float, default=drc.DEFAULT_GUARD_FPS,
                    help="载货链负载闸门：本道帧率低于它就跳过（默认 7.5）")
    ap.add_argument("--guard-incomplete", type=float, default=drc.DEFAULT_GUARD_INCOMPLETE,
                    help="载货链负载闸门：残缺帧占比高于它就跳过（默认 0.10）")
    ap.add_argument("--no-load-guard", action="store_true", help="关掉载货链负载闸门")
    ap.add_argument("--capture-poll", type=float, default=drc.DEFAULT_POLL_SEC,
                    help="采集进度轮询间隔（默认 2.0 秒）")
    return ap


def main(argv=None) -> int:
    a = build_parser().parse_args(argv)

    if a.interval < MIN_INTERVAL:
        # /api/state 很重：9-24 按 0.1 秒轮询时三条道残缺帧涨到 38~45%（文件头）
        print("▲ --interval %.2f 太密，按 %.1f 秒采样（/api/state 很重，轮询太密会让服务丢包）"
              % (a.interval, MIN_INTERVAL))
        a.interval = MIN_INTERVAL
    if a.no_capture:
        a.capture_channel = ""
    if a.capture_channel and a.capture_channel != drc.CAPTURE_CHANNEL:
        # fail-fast：误配后静默无效比启动失败更难查
        print("▲ --capture-channel 只接受 %s（或空串 / --no-capture 关闭），收到 %r"
              % (drc.CAPTURE_CHANNEL, a.capture_channel))
        return 2

    sink = open(a.out, "a", encoding="utf-8") if a.out else None
    lock = threading.RLock()          # ★ 台账与本函数共用**同一把**锁；用可重入锁防自锁

    def say(text: str) -> None:
        with lock:
            print(text, flush=True)
            if sink:
                sink.write(text + "\n")
                sink.flush()

    # ── 双雷达采集管理器（方案 §3：主线程只入队，不做 HTTP）
    session, ledger = _build_capture(a, say, lock)
    if session is not None:
        try:
            session.resume()        # 接力监控上次没结束的流程（方案 §8.1）
            session.start()         # ▲ 先置标志再起线程，否则会漏掉接力
        except Exception as error:                              # noqa: BLE001
            say("▲ 双雷达采集管理器起不来（%s），只做停稳记录" % error)
            session, ledger = None, None

    seen = {"save_check": False}

    say("时间     通道 事件")
    state = {}
    failing_since = None
    warned_at = 0.0
    try:
        while True:
            try:
                _poll_once(a.url, state, a, ledger, say, session, seen)
            except Exception as error:                          # noqa: BLE001
                # ★ 读不到 /api/state 要说出来（原来一声不吭地每秒重试）：
                #   服务在重启时短暂失败正常，一直失败就得有人去看（服务挂了 / 端口不对）。
                now = time.monotonic()
                if failing_since is None:
                    failing_since = now
                if now - warned_at >= READ_FAIL_WARN_SEC:
                    warned_at = now
                    say("%s ▲ 读不到 %s（%s: %s），已持续 %.0f 秒，继续重试"
                        % (time.strftime("%H:%M:%S"), a.url, type(error).__name__, error,
                           now - failing_since))
                time.sleep(1.)
                continue
            if failing_since is not None:
                say("%s √ /api/state 恢复（中断 %.0f 秒）"
                    % (time.strftime("%H:%M:%S"), time.monotonic() - failing_since))
                failing_since = None
                warned_at = 0.0
            time.sleep(a.interval)
    except KeyboardInterrupt:
        say("收到 Ctrl-C，退出")
    finally:
        if session is not None:
            try:
                session.close()          # 进行中的作业不取消，台账条目保留，下次启动接着盯
            except Exception:            # noqa: BLE001
                pass
        if sink:
            sink.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
