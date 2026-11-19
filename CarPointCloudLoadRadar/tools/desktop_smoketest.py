# -*- coding: utf-8 -*-
"""桌面版启动验证：拉起桌面程序 → 定位窗口 → 截图 → 健康检查 → 收尾。

跑法（非交互环境）：
    python tools/desktop_smoketest.py

★ 注意窗口标题：Edge/Chrome 用 `--app=<url>` 时，窗口标题取自**网页 <title>**，
  即 app/web/static/index.html 里的「载重雷达检测看板」，**不是**我们传给
  pywebview 的 APP_TITLE。所以这里要按网页标题去找。
"""
import ctypes
import os
import socket
import subprocess
import sys
import time
from ctypes import wintypes

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PY = sys.executable

# 精确标题：网页 <title> / pywebview 窗口名。
# ★ 不要用「载重雷达」这种模糊子串——资源管理器打开工程目录时标题是
#   「载重雷达逆向方案 - 文件资源管理器」，会被误判成应用窗口。
TITLE_CANDS = ("载重雷达检测看板", "载重雷达检测系统")

# Chromium 系（Edge/Chrome --app）与 WebView2 的顶层窗口类名
APP_CLASSES = ("Chrome_WidgetWin_1", "Chrome_WidgetWin_0", "WinForms10")
# 明确排除的系统外壳窗口类
DENY_CLASSES = ("CabinetWClass", "ExploreWClass", "Progman", "Shell_TrayWnd")

_u = ctypes.windll.user32


def free_port(start=18080):
    for p in range(start, start + 30):
        s = socket.socket()
        try:
            s.bind(("127.0.0.1", p))
            return p
        except OSError:
            continue
        finally:
            s.close()
    return 0


def list_visible_windows():
    """返回 [(hwnd, title, class, w, h)]，只含可见且有标题的顶层窗口。"""
    out = []

    def cb(hwnd, _l):
        if not _u.IsWindowVisible(hwnd):
            return True
        n = _u.GetWindowTextLengthW(hwnd)
        if n <= 0:
            return True
        buf = ctypes.create_unicode_buffer(n + 1)
        _u.GetWindowTextW(hwnd, buf, n + 1)
        cls = ctypes.create_unicode_buffer(256)
        _u.GetClassNameW(hwnd, cls, 256)
        r = wintypes.RECT()
        _u.GetWindowRect(hwnd, ctypes.byref(r))
        out.append((hwnd, buf.value, cls.value,
                    r.right - r.left, r.bottom - r.top))
        return True

    _u.EnumWindows(ctypes.WINFUNCTYPE(ctypes.c_bool, wintypes.HWND,
                                      wintypes.LPARAM)(cb), 0)
    return out


def find_app_windows():
    """精确匹配应用窗口：标题完全相等，且类名不在拒绝名单里。"""
    hits = []
    for hwnd, title, cls, w, h in list_visible_windows():
        if cls in DENY_CLASSES:
            continue
        if w < 300 or h < 200:      # 排除托盘/浮标之类的小窗
            continue
        if title.strip() in TITLE_CANDS:
            hits.append((hwnd, title, cls, w, h))
    return hits


def kill_tree(pid):
    subprocess.run(["taskkill", "/F", "/T", "/PID", str(pid)],
                   capture_output=True)


def capture_window(hwnd, out_path):
    """用 PrintWindow 直接抓窗口位图，不受其它窗口遮挡影响。

    PW_RENDERFULLCONTENT(0x2) 是 DirectComposition/Chromium 窗口的关键，
    否则抓到的是一片空白。
    """
    from ctypes import wintypes as wt
    gdi = ctypes.windll.gdi32
    user = ctypes.windll.user32
    r = wt.RECT()
    if not user.GetWindowRect(hwnd, ctypes.byref(r)):
        return False, "GetWindowRect 失败"
    w, h = r.right - r.left, r.bottom - r.top
    if w <= 0 or h <= 0:
        return False, f"窗口尺寸异常 {w}x{h}"

    hdc = user.GetWindowDC(hwnd)
    if not hdc:
        return False, "GetWindowDC 失败"
    mdc = gdi.CreateCompatibleDC(hdc)
    bmp = gdi.CreateCompatibleBitmap(hdc, w, h)
    gdi.SelectObject(mdc, bmp)
    try:
        PW_RENDERFULLCONTENT = 0x2
        ok = user.PrintWindow(hwnd, mdc, PW_RENDERFULLCONTENT)
        if not ok:
            user.PrintWindow(hwnd, mdc, 0)      # 退化成老版抓法

        class BITMAPINFOHEADER(ctypes.Structure):
            _fields_ = [("biSize", wt.DWORD), ("biWidth", wt.LONG),
                        ("biHeight", wt.LONG), ("biPlanes", wt.WORD),
                        ("biBitCount", wt.WORD), ("biCompression", wt.DWORD),
                        ("biSizeImage", wt.DWORD),
                        ("biXPelsPerMeter", wt.LONG),
                        ("biYPelsPerMeter", wt.LONG),
                        ("biClrUsed", wt.DWORD), ("biClrImportant", wt.DWORD)]

        bi = BITMAPINFOHEADER()
        bi.biSize = ctypes.sizeof(BITMAPINFOHEADER)
        bi.biWidth, bi.biHeight = w, -h     # 负高度 = 自顶向下
        bi.biPlanes, bi.biBitCount = 1, 32
        stride = w * 4
        buf = ctypes.create_string_buffer(stride * h)
        gdi.GetDIBits(mdc, bmp, 0, h, buf, ctypes.byref(bi), 0)

        from PIL import Image
        img = Image.frombuffer("RGBA", (w, h), buf,
                               "raw", "BGRA", stride, 1).convert("RGB")
        img.save(out_path)
        return True, f"{w}x{h}"
    finally:
        gdi.DeleteObject(bmp)
        gdi.DeleteDC(mdc)
        user.ReleaseDC(hwnd, hdc)


def main():
    import argparse
    ap = argparse.ArgumentParser(description="桌面版启动端到端验证")
    ap.add_argument("--wait", type=float, default=8.0,
                    help="窗口出现后再等几秒截图（等点云/首帧检测出结果）")
    ap.add_argument("--port", type=int, default=0)
    args = ap.parse_args()

    port = args.port or free_port()
    print(f"选用端口 {port}")

    env = dict(os.environ)
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUNBUFFERED"] = "1"          # 免得被 kill 时丢输出

    logp = os.path.join(ROOT, "logs", "_smoketest.log")
    os.makedirs(os.path.dirname(logp), exist_ok=True)
    logf = open(logp, "wb")

    p = subprocess.Popen(
        [PY, "-u", os.path.join(ROOT, "desktop", "app.py"),
         "--demo", "--port", str(port), "--no-single-instance"],
        cwd=ROOT, env=env, stdout=logf, stderr=subprocess.STDOUT)

    win, health = [], None
    try:
        deadline = time.time() + 75
        while time.time() < deadline:
            if p.poll() is not None:
                print("!! 进程提前退出，returncode =", p.returncode)
                break
            win = find_app_windows()
            if win:
                break
            time.sleep(1.0)

        print("\n--- 窗口检测 ---")
        if win:
            for hwnd, title, cls, w, h in win:
                print(f"  [OK] title={title!r}")
                print(f"       class={cls!r}  尺寸={w}x{h}  hwnd={hwnd}")
            if args.wait > 0:
                print(f"  等待 {args.wait:g}s 让看板出首帧…")
                time.sleep(args.wait)
                # 句柄可能已被 Chromium 重建，截图前重新枚举一次
                fresh = find_app_windows()
                if fresh:
                    win = fresh
                else:
                    print("  ！等待期间未再匹配到窗口，全部可见窗口如下：")
                    for h_, t_, c_, w_, h_2 in list_visible_windows():
                        mark = " <<< 类名/尺寸被过滤" if (
                            c_ in DENY_CLASSES or w_ < 300 or h_2 < 200) else ""
                        print(f"    - title={t_!r} [{c_}] {w_}x{h_2}{mark}")
        else:
            print("  [FAIL] 未找到应用窗口")
            print("  当前所有可见窗口（前 25 个）：")
            for hwnd, title, cls, w, h in list_visible_windows()[:25]:
                print(f"    - {title!r}  [{cls}] {w}x{h}")

        # 健康检查
        try:
            import urllib.request
            with urllib.request.urlopen(
                    f"http://127.0.0.1:{port}/api/health", timeout=5) as r:
                health = r.read().decode()
            print(f"  /api/health -> {health}")
        except Exception as e:
            print("  /api/health 失败:", e)

        # 首页可访问
        try:
            import urllib.request
            with urllib.request.urlopen(
                    f"http://127.0.0.1:{port}/", timeout=5) as r:
                body = r.read().decode("utf-8", "replace")
            print(f"  / -> HTTP {r.status}, {len(body)} 字节, "
                  f"含标题={'载重雷达检测看板' in body}")
        except Exception as e:
            print("  / 失败:", e)

        # 截图（PrintWindow 抓窗口自身，不受遮挡影响）
        if win:
            out = os.path.join(ROOT, "logs", "desktop_window.png")
            ok, info = capture_window(win[0][0], out)
            if ok:
                print(f"  截图 -> logs/desktop_window.png（{info}，PrintWindow）")
            else:
                print("  窗口截图失败:", info)
                try:
                    from PIL import ImageGrab
                    ImageGrab.grab().save(
                        os.path.join(ROOT, "logs", "desktop_smoketest.png"))
                    print("  退化为全屏截图 -> logs/desktop_smoketest.png")
                except Exception as e:
                    print("  全屏截图也失败:", e)
    finally:
        kill_tree(p.pid)
        try:
            p.wait(timeout=8)
        except Exception:
            pass
        logf.close()

    print(f"\n--- 程序输出（{logp}，末 30 行）---")
    try:
        with open(logp, encoding="utf-8", errors="replace") as f:
            for ln in f.read().splitlines()[-30:]:
                print("  |", ln)
    except OSError:
        pass

    return 0 if (win and health) else 1


if __name__ == "__main__":
    raise SystemExit(main())
