# -*- coding: utf-8 -*-
"""桌面版「双击启动」端到端验证。

和 desktop_smoketest.py 的区别：
  · smoketest 直接起 python desktop/app.py，验证"程序能开窗"；
  · 本脚本用 os.startfile 打开 **启动桌面版.vbs**，等价于用户双击，
    验证完整链路：VBS → 隐藏 cmd → python → Edge 应用窗口，
    并且验证「点 X 关窗 → 程序自动退出 → 锁文件清理」这一桌面行为。

跑法：
    python tools/desktop_launch_test.py                     # 正式版启动器
    python tools/desktop_launch_test.py --demo              # 演示版启动器（4路仿真）
    python tools/desktop_launch_test.py --launcher xxx.vbs  # 指定启动器

--demo 会额外校验「这次真的是演示态」：/api/state 里 director.auto 为真、
4 路通道全部是仿真源、并且车辆位置真的在变 —— 否则看板看起来正常，
实际却是四张静止卡片（演示时才发现就来不及了）。
"""
import ctypes
import os
import socket
import subprocess
import sys
import time
from ctypes import wintypes

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TITLE_CANDS = ("载重雷达检测看板", "载重雷达检测系统")
DENY_CLASSES = ("CabinetWClass", "ExploreWClass", "Progman", "Shell_TrayWnd")
LOCK = os.path.join(ROOT, "logs", ".desktop.lock")

_u = ctypes.windll.user32


def list_visible():
    out = []

    def cb(hwnd, _):
        if not _u.IsWindowVisible(hwnd):
            return True
        n = _u.GetWindowTextLengthW(hwnd)
        if n <= 0:
            return True
        b = ctypes.create_unicode_buffer(n + 1)
        _u.GetWindowTextW(hwnd, b, n + 1)
        c = ctypes.create_unicode_buffer(256)
        _u.GetClassNameW(hwnd, c, 256)
        r = wintypes.RECT()
        _u.GetWindowRect(hwnd, ctypes.byref(r))
        out.append((hwnd, b.value, c.value, r.right - r.left, r.bottom - r.top))
        return True

    _u.EnumWindows(ctypes.WINFUNCTYPE(ctypes.c_bool, wintypes.HWND,
                                      wintypes.LPARAM)(cb), 0)
    return out


def find_win():
    for hwnd, t, c, w, h in list_visible():
        if c in DENY_CLASSES or w < 300 or h < 200:
            continue
        if t.strip() in TITLE_CANDS:
            return hwnd, t, c, w, h
    return None


def read_lock():
    try:
        with open(LOCK, encoding="utf-8") as f:
            return f.read().strip()
    except OSError:
        return None


def health(port):
    try:
        import http.client
        c = http.client.HTTPConnection("127.0.0.1", port, timeout=4)
        c.request("GET", "/api/health")
        r = c.getresponse()
        body = r.read().decode()
        c.close()
        return r.status, body
    except Exception as e:                                    # noqa: BLE001
        return None, f"{type(e).__name__}: {e}"


def state(port):
    """取一份 /api/state 快照（不加代理 —— 本机代理会拦 urllib）。"""
    try:
        import http.client
        import json
        c = http.client.HTTPConnection("127.0.0.1", port, timeout=6)
        c.request("GET", "/api/state")
        r = c.getresponse()
        body = json.loads(r.read().decode("utf-8"))
        c.close()
        return body
    except Exception as e:                                    # noqa: BLE001
        return {"__error__": f"{type(e).__name__}: {e}"}


def read_lock_pair():
    """锁文件格式是 'pid port'，返回 (pid, port)，读不到返回 (0, 0)。"""
    raw = read_lock()
    if not raw:
        return 0, 0
    try:
        p = raw.split()
        return int(p[0]), int(p[1])
    except (IndexError, ValueError):
        return 0, 0


def pid_alive(pid):
    """与 desktop/launcher.pid_alive 同样的实现：OpenProcess，避免 tasklist 的编码坑。"""
    if pid <= 0:
        return False
    try:
        import ctypes
        k32 = ctypes.windll.kernel32
        h = k32.OpenProcess(0x1000, False, int(pid))       # PROCESS_QUERY_LIMITED_INFORMATION
        if not h:
            return False
        try:
            code = ctypes.c_ulong(0)
            if not k32.GetExitCodeProcess(h, ctypes.byref(code)):
                return False
            return code.value == 259                        # STILL_ACTIVE
        finally:
            k32.CloseHandle(h)
    except Exception:                                       # noqa: BLE001
        return False


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(prog="desktop_launch_test.py",
                                 description="桌面版双击启动端到端验证")
    ap.add_argument("--launcher", default="",
                    help="要模拟双击的启动器（省略时：--demo 用演示版，否则用正式版）")
    ap.add_argument("--demo", action="store_true",
                    help="按演示版校验：额外要求 director 自动编排在跑、4 路全是仿真源")
    ap.add_argument("--no-screenshot", action="store_true", help="跳过截图")
    ap.add_argument("--keep-existing", action="store_true",
                    help="不去清残留实例（默认会先清，否则量到的是旧进程）")
    args = ap.parse_args(argv)

    if not args.launcher:
        args.launcher = ("启动演示版（4路仿真）.vbs" if args.demo
                         else "启动桌面版.vbs")

    vbs = os.path.join(ROOT, args.launcher)
    if not os.path.isfile(vbs):
        print(f"× 找不到 {vbs}")
        return 2

    print("=" * 62)
    print(" 桌面版「双击启动」端到端验证")
    print("=" * 62)
    print(f"启动器: {os.path.basename(vbs)}")
    print(f"模式  : {'演示版（4 路仿真）' if args.demo else '正式版'}")
    print(f"锁文件: {LOCK}  (启动前: {read_lock() or '不存在'})")
    print()

    # ── 0) 前置清场（★ 这一步不能省）
    #  单实例保护：新进程发现锁文件里的实例还活着时，只会"开个窗口指向旧进程"
    #  然后自己退出。于是我们随后量到的 director / y_offset 全是**旧进程**的，
    #  验证结论完全不成立（而且旧进程可能是几分钟前跑过的、状态早已变了）。
    if not args.keep_existing:
        old_pid, old_port = read_lock_pair()
        if old_pid and pid_alive(old_pid):
            print(f"[0] 发现残留实例 pid={old_pid} port={old_port}，先清理 …")
            subprocess.run(["taskkill", "/F", "/T", "/PID", str(old_pid)],
                           capture_output=True)
            for _ in range(20):
                time.sleep(0.5)
                if not pid_alive(old_pid):
                    break
            print(f"    已清理: {not pid_alive(old_pid)}")
            try:
                os.remove(LOCK)
                print("    锁文件已删除")
            except OSError:
                pass
            # 关掉旧实例留下的浏览器窗口，免得下面 find_win() 认错窗口
            for hwnd0, t0, c0, w0, h0 in list_visible():
                if c0 in DENY_CLASSES or w0 < 300 or h0 < 200:
                    continue
                if t0.strip() in TITLE_CANDS:
                    print(f"    关闭遗留窗口 {t0!r} hwnd={hwnd0}")
                    _u.PostMessageW(hwnd0, 0x0010, 0, 0)      # WM_CLOSE
            time.sleep(1.5)
        elif old_pid:
            print(f"[0] 锁文件里有死进程 pid={old_pid}，顺手清掉锁")
            try:
                os.remove(LOCK)
            except OSError:
                pass

    ok = {"win": False, "health": False, "clean_exit": False, "lock": False}
    if args.demo:
        ok["demo_auto"] = False
        ok["demo_all_sim"] = False
        ok["demo_moving"] = False
    hwnd = None
    app_pid = 0

    # ── 1) 模拟双击
    print("[1] 模拟双击启动 …")
    os.startfile(vbs)                                          # noqa: S606

    # ── 2) 等窗口
    print("[2] 等待应用窗口（最长 75s）…")
    deadline = time.time() + 75
    while time.time() < deadline:
        got = find_win()
        if got:
            hwnd, t, c, w, h = got
            print(f"    [OK] 窗口已出现")
            print(f"         title = {t!r}")
            print(f"         class = {c!r}")
            print(f"         size  = {w}x{h}   hwnd = {hwnd}")
            ok["win"] = True
            break
        time.sleep(1.0)
    if not ok["win"]:
        print("    [FAIL] 75s 内未见窗口")
        print("    当前可见窗口：")
        for hwnd2, t2, c2, w2, h2 in list_visible()[:20]:
            print(f"      - {t2!r} [{c2}] {w2}x{h2}")
        return 1

    # ── 3) 锁文件 + 健康检查
    time.sleep(2.0)
    app_pid, port = read_lock_pair()
    print(f"[3] 锁文件内容: {read_lock()!r}")
    print(f"    → 服务进程 pid={app_pid}  端口={port}")
    ok["lock"] = bool(port and app_pid)
    if port:
        st, body = health(port)
        print(f"    GET /api/health -> {st} {body}")
        ok["health"] = (st == 200)
    else:
        print("    [FAIL] 锁文件里没解析出端口")

    # ── 3b) 演示态校验（--demo）
    if args.demo and port:
        print("[3b] 演示态校验 …")
        s = state(port)
        if "__error__" in s:
            print(f"    [FAIL] /api/state 取不到: {s['__error__']}")
        else:
            svc = s.get("service") or {}
            d = s.get("director")
            chans = s.get("channels") or []
            print(f"    demo={svc.get('demo')}  "
                  f"仿真路数={svc.get('sim_channels')}  "
                  f"真雷达路数={svc.get('live_channels')}")
            ok["demo_all_sim"] = (svc.get("sim_channels") == len(chans)
                                  and len(chans) >= 1)
            if d is None:
                print("    [FAIL] director 为 None —— 看板不会自己演（四张静止卡片）")
            else:
                print(f"    director.auto={d.get('auto')}  周期={d.get('cycle_sec')}s  "
                      f"通道数={d.get('n_channels')}")
                print(f"    各通道阶段: " + ", ".join(
                    f"{c.get('id')}={((c.get('demo') or {}).get('phase') or '?')}"
                    for c in chans))
                ok["demo_auto"] = bool(d.get("auto"))

            # 盯 12 秒：车辆 y 必须真的在变（编排线程在推）
            seen = set()
            t0 = time.time()
            while time.time() - t0 < 12.0:
                s2 = state(port)
                for c in (s2.get("channels") or []):
                    row = c.get("demo") or {}
                    if "y_offset" in row:
                        seen.add(round(float(row["y_offset"]), 3))
                if len(seen) > 3:
                    break
                time.sleep(0.5)
            print(f"    12s 内车辆 y 取值数: {len(seen)}  {sorted(seen)[:6]}"
                  f"{' …' if len(seen) > 6 else ''}")
            ok["demo_moving"] = len(seen) > 3

    # ── 4) 等页面首帧渲染完再截图
    #  ★ 注意：窗口 hwnd 出现 ≠ 页面已绘制。Chromium 先建窗口再加载内核，
    #    立刻截图（尤其窗口还在后台被遮挡时）只会得到一张白图。
    if args.no_screenshot:
        print("[4] 跳过截图（--no-screenshot）")
    else:
        print("[4] 等待页面首帧后截图 …")
        time.sleep(9.0)
        fresh = find_win()
        if fresh:
            hwnd = fresh[0]
            SW_RESTORE, SW_SHOW = 9, 5
            _u.ShowWindow(hwnd, SW_SHOW)
            _u.SetForegroundWindow(hwnd)
            time.sleep(1.5)
        try:
            from PIL import ImageGrab
            tag = "demo" if args.demo else "launch"
            ImageGrab.grab().save(os.path.join(ROOT, "logs", f"{tag}_test_full.png"))
            try:
                sys.path.insert(0, os.path.join(ROOT, "tools"))
                from desktop_smoketest import capture_window
                out = os.path.join(ROOT, "logs", f"{tag}_test_window.png")
                # ★ Chromium 会在加载过程中**重建 hwnd**，拿旧句柄抓会
                #   "GetWindowRect 失败"。每次重试都重新枚举一遍。
                done = False
                for attempt in range(1, 6):
                    fresh = find_win()
                    if fresh:
                        hwnd = fresh[0]
                    o, i = capture_window(hwnd, out)
                    if o:
                        print(f"    窗口截图: logs/{tag}_test_window.png "
                              f"({i}, 第 {attempt} 次)")
                        done = True
                        break
                    time.sleep(1.5)
                if not done:
                    print(f"    窗口截图失败（已重试 5 次）: {i}")
            except Exception as e:                                 # noqa: BLE001
                print(f"    窗口截图跳过: {e}")
        except Exception as e:                                     # noqa: BLE001
            print(f"    全屏截图失败: {e}")

    # ── 5) 关窗 → 应自动退出并清锁
    print("[5] 向窗口发 WM_CLOSE（等价于点右上角 X）…")
    WM_CLOSE = 0x0010
    _u.PostMessageW(hwnd, WM_CLOSE, 0, 0)

    for _ in range(30):
        time.sleep(1.0)
        if find_win() is None:
            break
    time.sleep(3.0)                       # 等服务收尾 + 清锁

    gone = find_win() is None
    proc_gone = not pid_alive(app_pid) if app_pid else False
    lock_now = read_lock()
    print(f"    窗口已关闭     : {gone}")
    print(f"    服务进程已退出 : {proc_gone}  (pid={app_pid})")
    print(f"    锁文件         : {lock_now or '已清理'}")
    ok["clean_exit"] = gone and proc_gone and not lock_now

    # ── 6) 结果
    print()
    print("=" * 62)
    labels = [("win", "窗口以桌面程序形式打开（无地址栏独立窗口）"),
              ("lock", "写入实例锁（pid + 端口）"),
              ("health", "看板 HTTP 服务可访问")]
    if args.demo:
        labels += [
            ("demo_auto", "自动演示编排在跑（四路会自己驶入/驶离）"),
            ("demo_all_sim", "全部通道都是仿真源（明确标注为演示数据）"),
            ("demo_moving", "车辆位置真的在变（不是四张静止卡片）"),
        ]
    labels += [("clean_exit", "关窗后服务进程自动退出 + 锁文件清理")]
    for k, label in labels:
        print(f"  {'√' if ok[k] else '×'} {label}")
    print("=" * 62)

    # 兜底清理：按锁文件里的 pid 精确杀进程树（不会误伤用户的其它 Edge 窗口）
    if not ok["clean_exit"] and app_pid:
        print(f"▲ 仍有残留，按 pid={app_pid} 清理进程树 …")
        subprocess.run(["taskkill", "/F", "/T", "/PID", str(app_pid)],
                       capture_output=True)

    return 0 if all(ok.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
