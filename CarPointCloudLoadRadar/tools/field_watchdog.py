# -*- coding: utf-8 -*-
"""现场看门狗：载货雷达服务停了（进程退出、或接口没响应）就自动重启；顺带防电脑空闲睡眠。

为什么要它（2026-09-24 现场）：10:49 电脑空闲睡眠 -> 外接 USB 盘（D 盘）断电 ->
唤醒后服务崩溃，而服务是在命令行窗口里手动起的，**4 个多小时没人发现**。

用法（本仓库目录下；用它**代替**手动起服务，别两个一起开）：
    python tools/field_watchdog.py                      # 只管载货雷达服务
    python tools/field_watchdog.py --webapi --logger    # 连车牌接口 WebApi、停稳记录器一起管

▲ 双雷达采集（`docs/68`）：2026-11-02 起记录器**默认不再触发** CH4 双雷达采集，
  改由服务在位置停稳时通知 WebApi（根目录 `docs/ch4-standstill-dimension-trigger-remediation-20261102.md`）。
  旧版记录器默认是"启用 + CH4 + http://127.0.0.1:5031"，现场若仍是旧版，必须加 `--no-capture`。
  本工具的采集开关只是**覆盖**它：
    · `--capture-webapi` / `--capture-channel` / `--guard-fps` / `--guard-incomplete` /
      `--ledger-ttl` / `--capture-poll` —— 给了才透传，没给就用记录器的默认值；
    · `--no-capture` / `--no-bell` / `--no-load-guard` —— 给了就透传（关掉）。
  ★★ **本工具既有的 `--webapi` 是"也看管 WebApi 进程"，与记录器的 WebApi 基址
     完全是两件事。** 记录器那边那个参数因此叫 `--capture-webapi`（方案 D8），
     这里也**只能用 `--capture-webapi`**，禁止复用 `--webapi`。
  ★ CH4 记录器使用固定请求头调用采集流程，不读取后台账号密码。
  ★ 记录器的控制台输出**不会被本工具落盘**（该 Job 的 `log_name=None`），
    采集的每一行由记录器自己同时写进它的 `--out` 文件
    （`data/field-log/standstill_<日期>.log`）。

它做什么：
    · 起服务（python -u cli/run.py --mode live），输出照旧追加到 logs/run_<日期>.log；
    · 服务进程退出 -> 等一会儿重启（5、10、20、40、60 秒逐次加长，稳定跑 10 分钟后复位）；
    · 服务还在但 /api/health 连续 3 次没响应（每 10 秒查一次，刚启动的 30 秒不查）-> 强制重启；
    · 每 60 秒看一次三条道的收包率，全是 0 就报警（多半是以太网断了，重启服务没用，得查网线）；
    · Windows 上运行期间向系统申请"别空闲睡眠"（退出即撤销，不改任何电源设置；合盖照样会睡）；
    · 自己的事件记到 logs/watchdog_<日期>.log。
Ctrl-C：子进程和它在同一个窗口，会一起收到 Ctrl-C 自行收尾；看门狗等它们退出后再退出。

不做的事：服务已经在跑（不是它起的）、或 UDP 2368 被别的程序占着时，**拒绝启动** ——
Windows 上两个程序同时绑 2368 都不报错，但报文只会落到其中一个，另一个静默收不到数据。

▲ 对停稳记录器**只看进程在不在**（没有接口探活）。它现在多了锁与后台线程，
  一旦死锁，进程仍是"活着"，本工具不会重启它，而车辆事件与双雷达采集会**一起停摆且没有信号**。
  这一点本轮不改（见 `docs/68` §14.3），但**现场必须知道**：发现记录器窗口长时间没有新车行，
  要主动去查，别等看门狗报。
"""
import argparse
import datetime
import json
import os
import subprocess
import sys
import threading
import time
import urllib.request

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:                       # noqa: BLE001
    pass

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEBAPI_DIR = os.path.join(os.path.dirname(ROOT), "CarPointCloudWebApi")
_OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))   # 回环地址不走系统代理
_LOG_LOCK = threading.Lock()


def _today() -> str:
    return datetime.datetime.now().strftime("%Y%m%d")


def _append(path, text):
    """写日志；D 盘暂时掉了（睡眠唤醒）也不能把看门狗自己弄崩。"""
    if not path:
        return
    try:
        with _LOG_LOCK, open(path, "a", encoding="utf-8") as fh:
            fh.write(text + "\n")
    except OSError:
        pass


def event(text):
    line = "%s [看门狗] %s" % (datetime.datetime.now().strftime("%m-%d %H:%M:%S"), text)
    print(line, flush=True)
    _append(os.path.join(ROOT, "logs", "watchdog_%s.log" % _today()), line)


def http_ok(url, timeout=5.0):
    try:
        with _OPENER.open(url, timeout=timeout) as resp:
            return 200 <= resp.status < 300
    except Exception:                   # noqa: BLE001
        return False


def http_json(url, timeout=10.0):
    try:
        with _OPENER.open(url, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except Exception:                   # noqa: BLE001
        return None


def udp_port_owners(port):
    """谁绑着这个 UDP 端口（只查 Windows；别的平台返回空，不拦）。"""
    if os.name != "nt":
        return []
    try:
        out = subprocess.run(["netstat", "-ano", "-p", "UDP"], capture_output=True, timeout=15).stdout
        text = out.decode("gbk", errors="replace")
    except Exception:                   # noqa: BLE001
        return []
    owners = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[0].upper() == "UDP" and parts[1].endswith(":%d" % port):
            owners.append(parts[-1])
    return owners


class KeepAwake:
    """Windows：运行期间申请"系统别空闲睡眠"（与播放器同一种申请），退出即撤销。"""
    ES_CONTINUOUS, ES_SYSTEM_REQUIRED = 0x80000000, 0x00000001

    def __init__(self, enabled):
        self.on = False
        if enabled and os.name == "nt":
            try:
                import ctypes
                self.on = bool(ctypes.windll.kernel32.SetThreadExecutionState(
                    self.ES_CONTINUOUS | self.ES_SYSTEM_REQUIRED))
            except Exception:           # noqa: BLE001
                self.on = False

    def release(self):
        if self.on:
            try:
                import ctypes
                ctypes.windll.kernel32.SetThreadExecutionState(self.ES_CONTINUOUS)
            except Exception:           # noqa: BLE001
                pass
            self.on = False


class Job:
    """一个被看管的子进程。health=None 时只看进程在不在。"""

    def __init__(self, name, argv, cwd, log_name=None, health=None, grace=30.0, env=None):
        self.name, self.argv, self.cwd, self.log_name = name, argv, cwd, log_name
        self.health, self.grace, self.env = health, grace, env
        self.proc = None
        self.started = 0.0
        self.next_start = 0.0
        self.fails = 0
        self.backoff = 5.0
        self.restarts = 0
        self.adopted = False                # 别人起的、我们只监看（仅 WebApi）

    def log_path(self):
        return os.path.join(ROOT, "logs", self.log_name % _today()) if self.log_name else None

    def start(self):
        try:
            self.proc = subprocess.Popen(self.argv, cwd=self.cwd, env=self.env,
                                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        except Exception as error:          # noqa: BLE001  （D 盘刚唤醒还没回来时会走到这里）
            self.proc = None
            self.schedule("启动失败：%s" % error)
            return
        self.started, self.fails = time.monotonic(), 0
        event("%s 已启动（PID %d）%s" % (self.name, self.proc.pid,
                                        "，第 %d 次重启" % self.restarts if self.restarts else ""))
        threading.Thread(target=self._pump, args=(self.proc, self.log_path()), daemon=True).start()

    def _pump(self, proc, path):
        prefix = "[%s] " % self.name
        for raw in iter(proc.stdout.readline, b""):
            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            print(prefix + line, flush=True)
            _append(path, line)

    def schedule(self, why):
        self.restarts += 1
        self.next_start = time.monotonic() + self.backoff
        event("%s %s，%.0f 秒后重启" % (self.name, why, self.backoff))
        self.backoff = min(self.backoff * 2, 60.0)

    def kill(self):
        if self.proc is None:
            return
        try:
            self.proc.kill()
            self.proc.wait(10)
        except Exception:                   # noqa: BLE001
            pass
        self.proc = None

    def tick(self, now, check_due):
        if self.adopted:
            if check_due and not http_ok(self.health):
                self.fails += 1
                if self.fails >= 3:
                    event("%s（不是看门狗起的）已连续 %d 次没响应，改由看门狗接管" % (self.name, self.fails))
                    self.adopted, self.fails, self.next_start = False, 0, now
            elif check_due:
                self.fails = 0
            return
        if self.proc is None:
            if now >= self.next_start:
                self.start()
            return
        code = self.proc.poll()
        if code is not None:
            self.proc = None
            self.schedule("进程退出（退出码 %s）" % code)
            return
        if now - self.started > 600:
            self.backoff = 5.0                                  # 稳定跑满 10 分钟，退避复位
        if self.health and check_due and now - self.started > self.grace:
            if http_ok(self.health):
                self.fails = 0
            else:
                self.fails += 1
                if self.fails >= 3:
                    event("%s 进程还在，但接口已连续 %d 次没响应" % (self.name, self.fails))
                    self.kill()
                    self.schedule("强制结束")

    def stop(self, timeout=20.0):
        """Ctrl-C 已同时送到子进程（同一窗口），这里等它们自己收尾，超时才强杀。"""
        if self.proc is None:
            return
        try:
            self.proc.wait(timeout)
        except subprocess.TimeoutExpired:
            event("%s %.0f 秒内没退出，强制结束" % (self.name, timeout))
            self.kill()


def _capture_argv(a):
    """把采集参数拼成透传给停稳记录器的一段 argv（方案 §9.3 / D8）。

    **没给的参数一律不拼** —— 记录器自己的默认值就是"启用 CH4 双雷达采集"，
    本工具不改行为时功能照样是开的。
    """
    out = []
    if a.no_capture:
        out.append("--no-capture")
    if getattr(a, "capture_webapi", None):
        out += ["--capture-webapi", a.capture_webapi]
    if getattr(a, "capture_token_file", None):
        out += ["--capture-token-file", a.capture_token_file]
    if getattr(a, "capture_channel", None):
        out += ["--capture-channel", a.capture_channel]
    if a.no_bell:
        out.append("--no-bell")
    if a.no_load_guard:
        out.append("--no-load-guard")
    if a.guard_fps is not None:
        out += ["--guard-fps", str(a.guard_fps)]
    if a.guard_incomplete is not None:
        out += ["--guard-incomplete", str(a.guard_incomplete)]
    if a.ledger_ttl is not None:
        out += ["--ledger-ttl", str(a.ledger_ttl)]
    if a.capture_poll is not None:
        out += ["--capture-poll", str(a.capture_poll)]
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="现场看门狗：服务停了自动重启，顺带防空闲睡眠")
    ap.add_argument("--webapi", action="store_true", help="也看管车牌接口 WebApi（5031）")
    ap.add_argument("--webapi-runtime-dir", default="", help="WebApi 私有运行目录，内含 appsettings.json")
    ap.add_argument("--webapi-dll", default="", help="WebApi 已编译 DLL；与私有运行目录一起使用")
    ap.add_argument("--webapi-config", default="", help="传给载货雷达服务的 WebApi 节点配置文件")
    ap.add_argument("--capture-credentials-file", default="", help="旧参数，现已忽略；CH4 采集不需要后台账号")
    ap.add_argument("--logger", action="store_true", help="也看管停稳记录器")
    ap.add_argument("--mode", default="live", help="服务的 --mode（默认 live）")
    ap.add_argument("--port", type=int, default=8080, help="服务看板端口（默认 8080）")
    ap.add_argument("--grace", type=float, default=30.0, help="服务启动后多少秒内不查接口")
    ap.add_argument("--allow-sleep", action="store_true", help="不申请防睡眠")
    # ── 透传给停稳记录器的双雷达采集开关（方案 §9.3 / D8）
    #    ★ 不叫 --webapi：那个名字已经是"也看管 WebApi 进程"，同名不同义
    ap.add_argument("--capture-webapi", default="", help="停稳记录器的 WebApi 基址（默认用记录器的默认值）")
    ap.add_argument("--capture-token-file", default="", help="停稳记录器的私有令牌文件路径")
    ap.add_argument("--capture-channel", default="", help="触发采集的车道（默认用记录器的默认值 CH4）")
    ap.add_argument("--no-capture", action="store_true", help="关掉停稳记录器的双雷达采集触发")
    ap.add_argument("--no-bell", action="store_true", help="关掉停稳记录器的报警响铃")
    ap.add_argument("--no-load-guard", action="store_true", help="关掉停稳记录器的载货链负载闸门")
    ap.add_argument("--guard-fps", type=float, default=None, help="负载闸门：帧率下限")
    ap.add_argument("--guard-incomplete", type=float, default=None, help="负载闸门：残缺帧占比上限")
    ap.add_argument("--ledger-ttl", type=float, default=None, help="台账条目最长开着多少秒")
    ap.add_argument("--capture-poll", type=float, default=None, help="采集进度轮询间隔（秒）")
    ap.add_argument("--health-url", default="", help=argparse.SUPPRESS)      # 自测用
    a = ap.parse_args(argv)

    if bool(a.webapi_runtime_dir) != bool(a.webapi_dll):
        ap.error("--webapi-runtime-dir 和 --webapi-dll 必须同时指定")
    if a.webapi_runtime_dir and not a.webapi:
        ap.error("使用 WebApi 私有运行目录时必须同时指定 --webapi")
    if a.webapi_runtime_dir and not a.webapi_config:
        ap.error("使用 WebApi 私有运行目录时必须指定 --webapi-config")
    if a.webapi_runtime_dir and not os.path.isfile(os.path.join(a.webapi_runtime_dir, "appsettings.json")):
        ap.error("WebApi 私有运行目录缺少 appsettings.json")
    if a.webapi_dll and not os.path.isfile(a.webapi_dll):
        ap.error("指定的 WebApi DLL 不存在")
    if a.webapi_config and not os.path.isfile(a.webapi_config):
        ap.error("指定的载货雷达 WebApi 节点配置不存在")
    health = a.health_url or "http://127.0.0.1:%d/api/health" % a.port
    if http_ok("http://127.0.0.1:%d/api/health" % a.port):
        print("已经有载货雷达服务在跑（手动起的，或另一个看门狗起的）。先在那个窗口里按 Ctrl-C 关掉，再启动看门狗。")
        return 2
    owners = udp_port_owners(2368)
    if owners:
        print("UDP 2368 正被 PID %s 占着（抓包、空场标定，或另一个服务？）。先关掉它再启动看门狗 —— "
              "两个程序同时绑 2368 不报错，但报文只落到其中一个。" % "、".join(owners))
        return 2
    os.makedirs(os.path.join(ROOT, "logs"), exist_ok=True)

    py_env = dict(os.environ, PYTHONIOENCODING="utf-8")
    jobs = []
    if a.webapi:
        web_argv = (["dotnet", os.path.abspath(a.webapi_dll), "--webroot",
                     os.path.join(WEBAPI_DIR, "wwwroot")]
                    if a.webapi_runtime_dir else
                    ["dotnet", "run", "--no-restore", "--", "--urls", "http://127.0.0.1:5031"])
        web_cwd = os.path.abspath(a.webapi_runtime_dir) if a.webapi_runtime_dir else WEBAPI_DIR
        web = Job("WebApi", web_argv, web_cwd, "webapi_%s.log",
                  health="http://127.0.0.1:5031/healthz", grace=120.0)
        if a.webapi_runtime_dir and http_ok(web.health):
            print("5031 已有 WebApi 在运行，无法确认它使用私有配置；请先正常关闭该实例。")
            return 2
        if http_ok(web.health):
            web.adopted = True
            event("WebApi 已经在跑（不是看门狗起的），只监看；它停了再由看门狗接管")
        jobs.append(web)
    service_argv = [sys.executable, "-u", os.path.join("cli", "run.py"), "--mode", a.mode]
    if a.webapi_config:
        service_argv += ["--webapi-config", os.path.abspath(a.webapi_config)]
    jobs.append(Job("服务", service_argv,
                    ROOT, "run_%s.log", health=health, grace=a.grace, env=py_env))
    if a.logger:
        # ★ 采集参数由这里一并传入（方案 D8）。不传 = 用记录器自己的默认值。
        capture_argv = _capture_argv(a)
        recorder_env = dict(py_env)
        jobs.append(Job("停稳记录器", [sys.executable, "-u", os.path.join("tools", "field_standstill_log.py"),
                                    "--out", os.path.join("data", "field-log", "standstill_%s.log" % _today())]
                                   + capture_argv,
                        ROOT, None, env=recorder_env))
        if capture_argv:
            event("停稳记录器将带上采集参数：%s" % " ".join(capture_argv))

    awake = KeepAwake(not a.allow_sleep)
    event("开始看管：%s；防空闲睡眠：%s；Ctrl-C 全部退出" % (
        "、".join(j.name for j in jobs),
        "已申请（合盖仍会睡）" if awake.on else ("已关闭" if a.allow_sleep else "非 Windows，不适用")))

    last_check = last_radar = time.monotonic()
    silent = 0
    try:
        while True:
            now = time.monotonic()
            check_due = now - last_check >= 10.0
            if check_due:
                last_check = now
            for job in jobs:
                job.tick(now, check_due)
            svc = jobs[1] if a.webapi else jobs[0]
            if now - last_radar >= 60.0 and svc.proc is not None and now - svc.started > 60.0:
                last_radar = now
                # 只读各道 packet_rate；history=0 免得服务为看门狗在帧锁内复制最近结果。
                state = http_json("http://127.0.0.1:%d/api/state?history=0" % a.port)
                rates = [(c.get("source") or {}).get("packet_rate") for c in (state or {}).get("channels", [])]
                if rates and all(not r for r in rates):
                    silent += 1
                    if silent == 2 or silent % 10 == 0:
                        event("三条道都收不到雷达报文（持续约 %d 分钟）：查以太网是否连上、网线、交换机 —— 重启服务没用"
                              % silent)
                        print("\a", end="", flush=True)
                elif rates:
                    if silent >= 2:
                        event("雷达报文恢复")
                    silent = 0
            time.sleep(2.0)
    except KeyboardInterrupt:
        event("收到 Ctrl-C，等子进程收尾")
        for job in jobs:
            job.stop()
        event("已退出")
        return 0
    finally:
        awake.release()


if __name__ == "__main__":
    sys.exit(main())
