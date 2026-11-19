# -*- coding: utf-8 -*-
"""一键打包桌面版 exe（PyInstaller），并在打包后自动验证产物能独立启动。

用法：
    python tools/build_desktop.py              # 目录模式（推荐，启动快）
    python tools/build_desktop.py --onefile    # 单文件 exe
    python tools/build_desktop.py --console    # 保留控制台（排查用）
    python tools/build_desktop.py --no-verify  # 跳过打包后验证

流程：
    1. 检查 PyInstaller 是否可用（没有就提示怎么装，不自动装）
    2. 清理旧的 build/ dist/ 产物目录
    3. 调 PyInstaller 按 desktop/loadradar.spec 打包
    4. 把 config/ 复制到产物**外面**（现场可改，不用重打包）
    5. 启动产物做冒烟验证：开窗 + /api/health，然后关掉

★ 为什么用 --windowed（spec 里 console=False）：
  桌面程序不该有黑框；出错信息由 desktop/app.py 用 MessageBox 弹出来。
"""

import argparse
import os
import shutil
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SPEC = os.path.join(ROOT, "desktop", "loadradar.spec")
NAME = "载重雷达检测系统"
DIST = os.path.join(ROOT, "dist")
BUILD = os.path.join(ROOT, "build")


def log(msg):
    print(msg, flush=True)


def check_pyinstaller():
    try:
        import PyInstaller                                    # noqa: F401
    except ImportError:
        return None
    return getattr(PyInstaller, "__version__", "?")


def find_python_with_pyinstaller():
    """本脚本可能跑在没装 PyInstaller 的解释器上 —— 那样就直接告诉用户换一个。

    常见情况：本机 3.13 是主力（有 numpy），但 PyInstaller 只装在 3.11 上。
    """
    try:
        import PyInstaller                                    # noqa: F401
        return sys.executable, getattr(PyInstaller, "__version__", "?")
    except ImportError:
        pass
    # 探一下 py -3.11 / py -3.12 之类
    for tag in ("-3.11", "-3.12", "-3.10", "-3.9"):
        try:
            out = subprocess.run(
                ["py", tag, "-c",
                 "import PyInstaller,numpy,sys;print(sys.executable);print(PyInstaller.__version__)"],
                capture_output=True, text=True, timeout=40)
            if out.returncode == 0:
                lines = out.stdout.strip().splitlines()
                if len(lines) >= 2:
                    return lines[0], lines[1]
        except (OSError, subprocess.SubprocessError):
            continue
    return None, None


def rmtree(p):
    if os.path.isdir(p):
        log(f"  清理 {os.path.relpath(p, ROOT)}")
        shutil.rmtree(p, ignore_errors=True)


def build(py_exe, onefile, console, workpath):
    env = dict(os.environ)
    if onefile:
        env["LOADRADAR_ONEFILE"] = "1"
    if console:
        env["LOADRADAR_CONSOLE"] = "1"

    # ★ 刻意不加 --clean：
    #   --clean 会让 PyInstaller 去删它自己的缓存与上一次的分析产物（几十个文件），
    #   在受管控/沙箱环境里这类批量删除会被安全策略直接拦掉，打包以 exit=1 失败，
    #   而且报错信息跟"代码有问题"长得一样，极易误判。
    #   改用"换一个全新的 workpath"来等效实现干净重建 —— 不需要删任何东西。
    cmd = [py_exe, "-m", "PyInstaller", "--noconfirm",
           "--distpath", DIST, "--workpath", workpath, SPEC]
    log("  执行: " + " ".join(f'"{c}"' if " " in c else c for c in cmd[2:]))
    p = subprocess.run(cmd, cwd=ROOT, env=env)
    return p.returncode


def clear_target(onefile):
    """给 PyInstaller 腾出目标路径 —— 用**改名**而不是删除。

    ★ 为什么不用 rmtree：
      PyInstaller 的 COLLECT 阶段遇到已存在的产物目录会先把它删掉（几百个文件），
      在受管控/沙箱环境里这类批量删除会被安全策略拦下，打包以 exit=1 失败，
      而且报错只写一行 safe-delete 提示，看起来像代码问题，很难联想到"删不掉"。

      改名（rename）在同一个卷上是原子操作、不需要删除权限，也不会有
      "删到一半失败"导致产物残缺的风险。旧产物保留在 .old 目录里，
      有问题时还能回去对照。
    """
    target = artifact_path(onefile)
    tree = target if os.path.isdir(target) else (
        DIST if onefile else os.path.join(DIST, NAME))
    if not os.path.isdir(tree):
        return

    stamp = time.strftime("%Y%m%d-%H%M%S")
    old = f"{tree}.old-{stamp}"
    try:
        os.rename(tree, old)
        log(f"  旧产物已让位 → {os.path.relpath(old, ROOT)}")
    except OSError as e:
        log(f"  ▲ 无法让位旧产物（{type(e).__name__}: {e}），继续尝试打包")


def purge_old_leftovers(keep=2):
    """清掉更早的 .old-* 残留，只留最近 keep 个。

    删除失败（权限/策略拦截）不算错误 —— 只是少回收点磁盘。
    """
    try:
        entries = sorted(
            [d for d in os.listdir(DIST) if ".old-" in d],
            reverse=True)
    except OSError:
        return
    for name in entries[keep:]:
        p = os.path.join(DIST, name)
        try:
            shutil.rmtree(p)
            log(f"  回收更早的残留 {name}")
        except OSError:
            log(f"  （残留 {name} 无法自动删除，可手工清理）")


def fresh_workpath():
    """挑一个空的 workpath 用，避免依赖删除权限。

    优先 build/loadradar（若为空或不存在），否则退到带序号的新目录。
    """
    base = os.path.join(BUILD, "loadradar")
    if not os.path.isdir(base) or not os.listdir(base):
        return base
    for i in range(2, 12):
        alt = os.path.join(BUILD, f"loadradar{i}")
        if not os.path.isdir(alt) or not os.listdir(alt):
            log(f"  build/loadradar 已有内容，改用 {os.path.relpath(alt, ROOT)}")
            return alt
    import tempfile
    return tempfile.mkdtemp(prefix="loadradar-")


def artifact_path(onefile):
    if onefile:
        return os.path.join(DIST, NAME + ".exe")
    return os.path.join(DIST, NAME, NAME + ".exe")


def stage_config(onefile):
    """把 config/ 放到 exe 旁边（而不是只留在 exe 内部）。

    ★ 现场改 IP/阈值的正确姿势：改 exe 旁边的 config/channels.json，重启程序。
      不用重打包。resolve_app_root() 会优先命中这个目录。
    """
    src = os.path.join(ROOT, "config")
    dst = os.path.join(DIST, "config") if onefile else os.path.join(DIST, NAME, "config")
    if not os.path.isdir(src):
        log(f"  ▲ 源 config/ 不存在，跳过")
        return None
    os.makedirs(dst, exist_ok=True)
    for fn in os.listdir(src):
        sp = os.path.join(src, fn)
        if os.path.isfile(sp):
            shutil.copy2(sp, os.path.join(dst, fn))
    log(f"  已复制 config/ → {os.path.relpath(dst, ROOT)}")
    return dst


def copy_site_docs(onefile):
    """顺手把启动脚本和现场调试文档放到产物旁边。"""
    dst_root = DIST if onefile else os.path.join(DIST, NAME)
    for fn in ("启动桌面版.vbs", "启动桌面版（带日志）.bat", "环境自检.bat",
               "清理过期样本.bat"):
        sp = os.path.join(ROOT, fn)
        if os.path.isfile(sp):
            shutil.copy2(sp, os.path.join(dst_root, fn))
            log(f"  已复制 {fn}")
    doc = os.path.join(ROOT, "docs", "05-桌面版部署与打包.md")
    if os.path.isfile(doc):
        os.makedirs(os.path.join(dst_root, "docs"), exist_ok=True)
        shutil.copy2(doc, os.path.join(dst_root, "docs"))
        log("  已复制 docs/05-桌面版部署与打包.md")


def _decode(b: bytes) -> str:
    """exe 输出可能仍是 GBK（老 console 代码页），逐级降级解码。"""
    for enc in ("utf-8", "gbk", "cp936", "latin-1"):
        try:
            return b.decode(enc)
        except UnicodeDecodeError:
            continue
    return b.decode("utf-8", "replace")


def _free_port(start=18500):
    import socket as _s
    for p in range(start, start + 60):
        s = _s.socket()
        try:
            s.bind(("127.0.0.1", p))
            return p
        except OSError:
            continue
        finally:
            s.close()
    return 0


def _pid_alive(pid):
    """OpenProcess 判活 —— 不用 tasklist（输出是本地代码页，会把 text=True 解码搞崩）。"""
    if pid <= 0:
        return False
    try:
        import ctypes
        k32 = ctypes.windll.kernel32
        h = k32.OpenProcess(0x1000, False, int(pid))
        if not h:
            return False
        try:
            code = ctypes.c_ulong(0)
            if not k32.GetExitCodeProcess(h, ctypes.byref(code)):
                return False
            return code.value == 259
        finally:
            k32.CloseHandle(h)
    except Exception:                                         # noqa: BLE001
        return False


def verify(exe, onefile):
    """打包产物的可信验证，分三步：

      1. `--check`  —— 配置、静态资源、窗口后端是否都就位（不开窗，快）
      2. 真开一次窗 —— 用 --demo 起真实进程，确认窗口 + /api/health 都活
      3. 关窗即退   —— 确认 GUI 进程能干净收场（不残留、不孤儿）

    ★ "打包成功" ≠ "能运行"。最常见的翻车是漏了 static/index.html 或 config/，
      只跑 --check 会漏掉"窗口起不来"，所以必须真开一次窗。
    """
    if not os.path.isfile(exe):
        log(f"× 产物不存在: {exe}")
        return False

    exe_dir = os.path.dirname(exe)
    size_mb = os.path.getsize(exe) / 1048576
    log(f"  产物: {os.path.relpath(exe, ROOT)}  ({size_mb:.1f} MB)")

    # ── 1) --check ──
    p = subprocess.run([exe, "--check"], capture_output=True, timeout=240, cwd=exe_dir)
    out = _decode(p.stdout or b"") + _decode(p.stderr or b"")
    log("  --- [1/3] exe --check ---")
    for ln in out.splitlines():
        log("    | " + ln)
    if p.returncode != 0:
        log(f"× --check 失败，exit={p.returncode}")
        return False
    if "index.html 就位" not in out:
        log("× 静态资源没随包带上（index.html 缺失）→ 打开窗口会白屏")
        return False
    log("  √ 配置 / 静态资源 / 窗口后端 均就位")

    # ── 2) 真开一次窗 ──
    port = _free_port()
    log(f"  --- [2/3] 启动产物开窗（--demo --port {port}）---")
    proc = subprocess.Popen([exe, "--demo", "--port", str(port),
                             "--no-single-instance"],
                            cwd=exe_dir)
    try:
        sys.path.insert(0, os.path.join(ROOT, "tools"))
        from desktop_smoketest import find_app_windows, list_visible_windows

        win = []
        deadline = time.time() + 90
        while time.time() < deadline:
            if proc.poll() is not None:
                log(f"  × 进程提前退出，exit={proc.returncode}")
                break
            win = find_app_windows()
            if win:
                break
            time.sleep(1.0)

        if not win:
            log("  × 90s 内未见应用窗口")
            for hwnd, t, c, w, h in list_visible_windows()[:20]:
                log(f"      - {t!r} [{c}] {w}x{h}")
            return False

        hwnd, title, cls, w, h = win[0]
        log(f"  √ 窗口已出现: title={title!r} class={cls!r} {w}x{h}")

        # ── 3) HTTP 健康 ──
        import http.client
        ok_http = False
        for _ in range(20):
            try:
                c = http.client.HTTPConnection("127.0.0.1", port, timeout=4)
                c.request("GET", "/api/health")
                r = c.getresponse()
                body = r.read().decode("utf-8", "replace")
                c.close()
                if r.status == 200 and '"ok"' in body:
                    log(f"  √ /api/health -> {body}")
                    ok_http = True
                    break
            except Exception:                                 # noqa: BLE001
                pass
            time.sleep(0.5)
        if not ok_http:
            log("  × /api/health 不通过")
            return False

        # ── 3) 关窗 → 进程应自行退出 ──
        log("  --- [3/3] 关窗，验证进程自行退出 ---")
        import ctypes
        ctypes.windll.user32.PostMessageW(hwnd, 0x0010, 0, 0)   # WM_CLOSE
        for _ in range(30):
            if proc.poll() is not None:
                break
            time.sleep(1.0)
        if proc.poll() is None:
            log("  ▲ 关窗后 30s 内进程未退出")
            return False
        log(f"  √ 关窗后进程已退出（exit={proc.returncode}）")
        return True
    finally:
        if proc.poll() is None:
            subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                           capture_output=True)


def main():
    ap = argparse.ArgumentParser(description="打包桌面版 exe")
    ap.add_argument("--onefile", action="store_true", help="打单文件 exe")
    ap.add_argument("--console", action="store_true", help="保留控制台窗口")
    ap.add_argument("--no-verify", action="store_true", help="跳过打包后验证")
    ap.add_argument("--clean", action="store_true",
                    help="连 dist/ 一起删干净（默认只删 build/ 中间产物；"
                         "PyInstaller 的 --noconfirm 会覆盖 dist/，"
                         "只有在产物结构变化明显时才需要 --clean）")
    args = ap.parse_args()

    log("=" * 66)
    log(" 载重雷达检测系统 · 桌面版打包")
    log("=" * 66)

    py_exe, ver = find_python_with_pyinstaller()
    if not py_exe:
        log("× 找不到装有 PyInstaller 的 Python。")
        log("")
        log("   先安装（任选一个解释器）：")
        log(f'     "{sys.executable}" -m pip install pyinstaller')
        log("   或：")
        log("     py -3.11 -m pip install pyinstaller")
        log("")
        log("   注意：该解释器还必须能 import numpy（打包时会一起带上）。")
        return 2

    log(f"  打包解释器 : {py_exe}")
    log(f"  PyInstaller: {ver}")
    log(f"  模式       : {'单文件 exe' if args.onefile else '目录模式'} / "
        f"{'带控制台' if args.console else '无控制台（windowed）'}")
    log("")

    if not os.path.isfile(SPEC):
        log(f"× 找不到 spec: {SPEC}")
        return 2

    log("[1/4] 检查中间产物")
    # ★ 刻意不做批量删除：PyInstaller 的 --noconfirm --clean 本身会覆盖重建，
    #   手工 rmtree 既没必要，又容易误删（还常常被安全策略拦下）。
    #   需要彻底重来时用 --clean 显式声明。
    if args.clean:
        log("  ▲ --clean：将删除 build/ 与 dist/（全量重建）")
        rmtree(BUILD)
        rmtree(DIST)
    else:
        stale = os.path.join(BUILD, "loadradar")
        if os.path.isdir(stale):
            log(f"  检测到 {os.path.relpath(stale, ROOT)}（将由 PyInstaller 覆盖重建）")
        else:
            log("  无历史中间产物")
        log("  提示：需要彻底重建时加 --clean")

    log("[2/4] 给 PyInstaller 腾出产物路径")
    clear_target(args.onefile)
    purge_old_leftovers()

    log("[3/4] PyInstaller 打包")
    rc = build(py_exe, args.onefile, args.console, fresh_workpath())
    if rc != 0:
        log(f"× 打包失败，exit={rc}")
        return rc

    exe = artifact_path(args.onefile)
    if not os.path.isfile(exe):
        log(f"× 打包结束但找不到产物 {exe}")
        return 3

    log("[4/4] 布置现场可改的配置与启动脚本")
    stage_config(args.onefile)
    copy_site_docs(args.onefile)

    if args.no_verify:
        log("[5/5] 已跳过验证")
    else:
        log("[5/5] 验证产物")
        if not verify(exe, args.onefile):
            return 4

    log("")
    log("=" * 66)
    log(" √ 打包完成")
    log("=" * 66)
    if args.onefile:
        log(f"  分发：dist/{NAME}.exe + dist/config/ + 启动脚本")
        log(f"  运行：双击 dist/{NAME}.exe")
    else:
        log(f"  分发：整个 dist/{NAME}/ 目录")
        log(f"  运行：dist/{NAME}/{NAME}.exe")
    log("")
    log("  现场改配置：改 exe 旁边 config/channels.json，重启程序即可，无需重打包。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
