#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plate.py — 车牌识别（对应 stopdetect.dll 里的 PlateMonitor）

────────────────────────────────────────────────────────────────────────
关于这个模块的"确定性"边界，先说清楚：
────────────────────────────────────────────────────────────────────────

√ 已确认（来自 stopdetect.dll 的导出符号与字符串）
    class PlateMonitor 由 stopdetect.dll 导出，接口为：

        void        PlateMonitor::start()
        void        PlateMonitor::stop()
        bool        PlateMonitor::running() const
        bool        PlateMonitor::wait() const          ← 阻塞等一次识别完成
        bool        PlateMonitor::done() const
        std::string PlateMonitor::error() const
        ...         PlateMonitor::save_frame(...)        ← 识别到车牌时保存点云帧

    相关日志串：
        [PlateMonitor] save_frame failed: ...
        PlateMonitor config load failed: ...
        ~PlateMonitor() stop() failed: ...
        ": stable, waiting for vehicle to leave"

    → 工作模式是「后台线程 + 等待」：车停稳后启动识别，拿到车牌就 save_frame，
       等车离开再重置。

× 无法从二进制还原（**必须现场按相机手册补**）
    stopdetect.dll 里没有任何 URL / HTTP 方法 / 报文模板字符串。
    车牌的取图与识别完全走外部相机，接口地址来自配置，
    而现场部署用的那份配置我们没拿到（手上的 devices.json 是精简版）。
    所以本模块把「怎么调相机」做成**完全可配置**，而不是猜一个协议。

现场怎么补（10 分钟）
    1. 打开相机厂商（海康/大华/宇视…）的 SDK 或 HTTP 接口文档，
       找到「抓拍并返回车牌号」的那个接口。
    2. 在 config/channels.json 的 plate 段填：
           "snap_url":   "http://{base_url}/api/gate/snap?device={device_id}",
           "result_url": "http://{base_url}/api/gate/result?device={device_id}",
           "method":     "GET",
           "plate_field":"data.plate",          // JSON 取值路径
           "plate_regex":"[京津沪渝冀豫云辽黑湘皖鲁新苏浙赣鄂桂甘晋蒙陕吉闽贵粤青藏川宁琼][A-Z][A-Z0-9]{4,6}"
    3. 用 cli/plate_test.py 打一次真机，看到车牌就说明填对了。
       —— 这个"先在命令行打通，再接入服务"的顺序很重要，
          不要一上来就在服务里调，否则分不清是相机的问题还是服务的问题。

没有相机时
    enabled=false → NullPlateReader，永远返回空车牌（服务照常跑）。
    或者 mode="simulate" → SimulatedPlateReader，用 config 里的 sample_plates
    轮流出牌，方便演示与联调。
"""
from __future__ import annotations

import hashlib
import json
import os
import re
import threading
import time
import unicodedata
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

# 相机"这次没识别出车牌"的各种写法。它们**不是车牌**，必须落回 EMPTY_PLATE，
# 否则 data/<车牌>/ 下会长出 `未识别/`、`无牌车/`、`-/` 这种目录，
# 而且看板会显示"车牌已取到"。各家相机的写法不统一，这里按小写归一后比对。
NO_PLATE_TOKENS = {
    "", "-", "--", "0", "00", "000000", "null", "none", "nil", "n/a", "na",
    "unknown", "unrecognized", "fail", "failed", "error", "false",
    "无", "无牌", "无车牌", "无牌车", "未知", "未知车牌", "未识别", "识别失败",
    "无结果", "空", "暂无",
}


def _looks_like_no_plate(v: str) -> bool:
    """判断相机返回的这个字段是不是"没识别出来"的占位值。"""
    t = unicodedata.normalize("NFKC", (v or "").strip()).strip("　")
    if t.lower() in NO_PLATE_TOKENS:
        return True
    # 全是标点/空白，或短到不可能是车牌（最短的车牌也有 7 位）
    if len(t) < 3:
        return True
    return not any(ch.isalnum() for ch in t)


# 中国大陆车牌号的正则（含新能源 8 位、使/领/警/挂/学 等后缀）
DEFAULT_PLATE_REGEX = (
    r"[京津沪渝冀豫云辽黑湘皖鲁新苏浙赣鄂桂甘晋蒙陕吉闽贵粤青藏川宁琼]"
    r"[A-Z][A-HJ-NP-Z0-9]{4,5}[A-HJ-NP-Z0-9挂学警港澳]"
)


# ═════════════════════════════════════════════ 数据

@dataclass
class PlateResult:
    ok: bool = False
    plate: str = ""
    confidence: float = 0.0
    raw: str = ""
    elapsed: float = 0.0
    error: str = ""
    attempts: int = 0

    def to_dict(self) -> Dict:
        return {"ok": self.ok, "plate": self.plate,
                "confidence": round(self.confidence, 3),
                "elapsed": round(self.elapsed, 3), "attempts": self.attempts,
                "error": self.error}


EMPTY_PLATE = "未知车牌"


# ═════════════════════════════════════════════ 抽象

class PlateReaderBase:
    """车牌读取器接口。start/wait/done/error 对齐原版 PlateMonitor。"""
    kind = "base"

    def __init__(self, cfg):
        self.cfg = cfg
        self._lock = threading.Lock()
        self._running = False
        self._done = False
        self._result = PlateResult()
        self._error = ""
        self._thread: Optional[threading.Thread] = None

    # ── 同步接口（简单场景直接用这个）──────────────────────
    def read(self, timeout: float = 0.0) -> PlateResult:
        raise NotImplementedError

    # ── 异步接口（对齐原版 PlateMonitor）────────────────────
    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            self._running = True
            self._done = False
            self._error = ""
            self._result = PlateResult()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name=f"plate-{self.kind}")
        self._thread.start()

    def _run(self) -> None:
        """默认实现：跑一次 read() 就结束。原版也是"识别一次"的语义。"""
        try:
            r = self.read()
            with self._lock:
                self._result = r
                self._done = r.ok
                if r.error:
                    self._error = r.error
        except Exception as e:                       # noqa: BLE001
            with self._lock:
                self._error = f"{type(e).__name__}: {e}"
        finally:
            with self._lock:
                self._running = False

    def stop(self) -> None:
        with self._lock:
            self._running = False
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=2.0)

    def wait(self, timeout: float = 0.0) -> bool:
        """阻塞直到识别结束（或超时）。返回是否成功拿到车牌。"""
        t = self._thread
        if t is not None:
            t.join(timeout=timeout if timeout > 0 else None)
        with self._lock:
            return self._done and bool(self._result.plate)

    def running(self) -> bool:
        with self._lock:
            return self._running

    def done(self) -> bool:
        with self._lock:
            return self._done

    def error(self) -> str:
        with self._lock:
            return self._error

    def result(self) -> PlateResult:
        with self._lock:
            return self._result

    def info(self) -> Dict:
        return {"kind": self.kind, "enabled": bool(getattr(self.cfg, "enabled", False)),
                "running": self.running(), "done": self.done(),
                "error": self.error()}


# ═════════════════════════════════════════════ 空实现

class NullPlateReader(PlateReaderBase):
    """未接相机时的占位实现。服务照常运行，车牌一律记为「未知车牌」。"""

    kind = "null"

    def read(self, timeout: float = 0.0) -> PlateResult:
        return PlateResult(ok=False, plate=EMPTY_PLATE, error="未启用车牌识别")


# ═════════════════════════════════════════════ 仿真

class SimulatedPlateReader(PlateReaderBase):
    """
    离线演示用：按 sample_plates 列表轮流出牌（模拟"排队进场的车队"）。
    也让 cli/plate_test.py 在没有真机时能自检整条链路。
    """

    kind = "simulate"

    def __init__(self, cfg, plates: Optional[List[str]] = None,
                 delay: float = 0.6, fail_rate: float = 0.0, rotate: int = 0):
        super().__init__(cfg)
        self.plates = list(plates) if plates else \
            ["宁A88635W", "宁AV3808", "蒙C88593X", "宁A9H5D5"]
        self.delay = delay
        self.fail_rate = fail_rate
        # ★ rotate：多路通道各给一个不同的起点，否则四路会**同时**出同一张车牌。
        #   现场看板上四张卡片齐刷刷"宁A88635W"一眼就穿帮了；
        #   错开之后它们看起来就是四个车道各自拍到不同的车。
        self._i = int(rotate) % max(1, len(self.plates))
        self._rng = __import__("random").Random(20260721)

    def read(self, timeout: float = 0.0) -> PlateResult:
        t0 = time.time()
        time.sleep(self.delay)                     # 模拟相机响应
        p = self.plates[self._i % len(self.plates)]
        self._i += 1
        if self._rng.random() < self.fail_rate:
            return PlateResult(ok=False, plate=EMPTY_PLATE, elapsed=time.time() - t0,
                               attempts=1, error="仿真：本轮未识别到车牌")
        return PlateResult(ok=True, plate=p, confidence=0.96,
                           raw=json.dumps({"plate": p, "confidence": 0.96},
                                          ensure_ascii=False),
                           elapsed=time.time() - t0, attempts=1)


# ═════════════════════════════════════════════ HTTP 实现

class HttpPlateReader(PlateReaderBase):
    """
    配置驱动的 HTTP 车牌读取器（对应原版 PlateMonitor 的真实职责）。

    时序完全按 config/channels.json 的 plate 段：
        f2_timeout_ms         单次抓拍请求超时
        f2_retry_delay_1/2/3  抓拍失败后的三次重试间隔（原版就是 3 次）
        f3_poll_interval_ms   轮询识别结果的间隔
        f3_timeout_ms         轮询总超时

    流程（与原版语义一致）
        1. 请求 snap_url  → 触发相机抓拍（原版的 "F2"）
        2. 按 f3_poll_interval_ms 轮询 result_url（原版的 "F3"）直到拿到车牌
        3. 拿不到车牌就把抓拍图存下来（save_frame 的语义），便于事后查
    """

    kind = "http"

    def __init__(self, cfg, save_dir: str = ""):
        super().__init__(cfg)
        self.save_dir = save_dir
        self.plate_re = re.compile(cfg.plate_regex or DEFAULT_PLATE_REGEX)
        # ★ 相机地址是现场内网/本机服务，必须**禁用系统代理**（2026-09-23）。
        #   urlopen 默认吃 getproxies()：Windows 上读 IE/WinINET 代理、
        #   类 Unix 上读 HTTP_PROXY 等环境变量。磅房机器一旦挂了代理，
        #   车牌请求会被劫持到代理上白等超时 —— 实测本机（代理 127.0.0.1:29290）
        #   4 条 plate 用例全挂：本地测试后端零请求、耗时烧满重试预算。
        #   上报链 load_radar_transport 早已用 ProxyHandler({}) 清了代理
        #   （同一个坑，那边修过），这里对齐。
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    # ── URL 拼装 ────────────────────────────────────────────
    def _url(self, tmpl: str) -> str:
        c = self.cfg
        base = (c.base_url or "").rstrip("/")
        if not tmpl:
            return ""
        if not tmpl.startswith("http"):
            tmpl = base + "/" + tmpl.lstrip("/")
        return (tmpl.replace("{base_url}", base)
                    .replace("{device_id}", str(c.device_id))
                    .replace("{camera_identifier}", str(c.camera_identifier or "")))

    def _request(self, url: str, method: str = "GET", body: bytes = b"",
                 timeout_ms: int = 2000) -> Tuple[bool, bytes, str]:
        if not url:
            return False, b"", "URL 未配置"
        # ★ Request() 本身也会抛（base_url 少写 http:// 就是 ValueError: unknown url type）。
        #   以前它在 try 外面，异常会一路冒到 ChannelWorker._tick 的兜底 except，
        #   表现成"整条通道报错、每 0.5s 刷一次"，而不是"车牌读取失败"这条本该有的信息。
        try:
            req = urllib.request.Request(url, data=(body or None),
                                         method=method.upper())
            req.add_header("User-Agent", "kylin-lidar/1.0")
            for k, v in (self.cfg.headers or {}).items():
                req.add_header(k, str(v))
        except (ValueError, TypeError) as e:
            return False, b"", f"URL 非法（检查 plate.base_url 是否带 http://）: {e}"
        try:
            # 走禁用代理的 opener（见 __init__ 的注释），绝不吃系统代理。
            with self._opener.open(req, timeout=max(0.05, timeout_ms / 1000.0)) as r:
                return True, r.read(), ""
        except urllib.error.HTTPError as e:
            return False, b"", f"HTTP {e.code}"
        except urllib.error.URLError as e:
            return False, b"", f"连接失败: {e.reason}"
        except (TimeoutError, OSError) as e:
            return False, b"", f"网络错误: {e}"
        except Exception as e:                       # noqa: BLE001
            # 相机固件五花八门，urllib 还可能抛 http.client.* / ssl.* 等等。
            # 车牌读不到绝不能拖垮雷达检测 —— 这里必须兜住所有异常。
            return False, b"", f"{type(e).__name__}: {e}"

    # ── 从响应里抠车牌 ──────────────────────────────────────
    def _extract(self, text: str) -> Tuple[str, float]:
        """先按 plate_field 取 JSON 字段，失败再按正则全文扫。"""
        field_path = (self.cfg.plate_field or "").strip()
        if field_path:
            try:
                obj: Any = json.loads(text)
                for part in field_path.split("."):
                    if part == "":
                        continue
                    if isinstance(obj, list):
                        obj = obj[int(part)]
                    else:
                        obj = obj[part]
                if isinstance(obj, (str, int)):
                    m = self.plate_re.search(str(obj))
                    if m:
                        return m.group(0), 1.0
                    # 正则没匹配上也认这个字段 —— 军牌/港澳/教练车等格式不在默认正则里，
                    # 硬卡正则会把真车牌丢掉。但要先把"没识别出来"的占位值挡掉，
                    # 否则会建出 `未识别/` 这种目录并显示成"车牌已取到"。
                    if str(obj).strip() and not _looks_like_no_plate(str(obj)):
                        return str(obj).strip(), 1.0
            except Exception:                        # noqa: BLE001 —— 格式不符就退回正则
                pass
        m = self.plate_re.search(text)
        if m:
            return m.group(0), 0.9
        return "", 0.0

    def read(self, timeout: float = 0.0) -> PlateResult:
        c = self.cfg
        t0 = time.time()
        attempts = 0
        snap_url = self._url(c.snap_url)
        res_url = self._url(c.result_url)
        body = (c.body or "").encode("utf-8")

        # ① 触发抓拍，最多 1 + 3 次（重试间隔取 f2_retry_delay_*）
        #
        # ★ snap_url 留空 = 这个接口**没有"先抓拍"这个动作**，直接取牌就行
        #   （后台 CarPointCloudWebApi 的 GET /api/v1/radar/licensePlate 就是这种）。
        #   以前不管配没配都硬走一遍：_url("") 返回空串 -> _request 立刻报
        #   「URL 未配置」-> 四次重试白等 0+300+800+1500 = 2.6 秒 -> 最后返回
        #   「抓拍请求失败」。**车牌其实取得到，却永远走不到第②步**，
        #   而报错指向抓拍，和真正的接口毫无关系。
        last_err = ""
        if snap_url:
            delays = [0.0, c.f2_retry_delay_1_ms, c.f2_retry_delay_2_ms,
                      c.f2_retry_delay_3_ms]
            for d in delays:
                if d:
                    time.sleep(d / 1000.0)
                attempts += 1
                ok, _data, err = self._request(snap_url, c.method, body, c.f2_timeout_ms)
                if ok:
                    last_err = ""
                    break
                last_err = err
            if last_err:
                return PlateResult(ok=False, plate=EMPTY_PLATE, attempts=attempts,
                                   elapsed=time.time() - t0,
                                   error=f"抓拍请求失败（{attempts} 次）: {last_err}")

        # ② 轮询识别结果
        deadline = time.time() + c.f3_timeout_ms / 1000.0
        last_text = ""
        while time.time() < deadline:
            ok, data, err = self._request(res_url, "GET", b"", c.f2_timeout_ms)
            attempts += 1
            if ok:
                last_text = data.decode("utf-8", errors="replace")
                plate, conf = self._extract(last_text)
                if plate:
                    return PlateResult(ok=True, plate=plate, confidence=conf,
                                       raw=last_text, attempts=attempts,
                                       elapsed=time.time() - t0)
            else:
                last_err = err
            time.sleep(max(0.02, c.f3_poll_interval_ms / 1000.0))

        # ③ 没拿到车牌 → 落盘现场信息（对应原版 save_frame 的用途）
        self._dump_failure(last_text or last_err, t0)
        return PlateResult(ok=False, plate=EMPTY_PLATE, attempts=attempts,
                           elapsed=time.time() - t0,
                           error=f"轮询 {c.f3_timeout_ms}ms 未取到车牌"
                                 + (f"（最后响应: {last_text[:120]}）" if last_text else ""))

    def _dump_failure(self, text: str, t0: float) -> None:
        if not self.save_dir:
            return
        import os
        try:
            os.makedirs(self.save_dir, exist_ok=True)
            p = os.path.join(self.save_dir,
                             time.strftime("plate_fail_%Y%m%d_%H%M%S", time.localtime(t0)) + ".txt")
            with open(p, "w", encoding="utf-8") as f:
                f.write(f"time    : {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t0))}\n")
                f.write(f"snap_url: {self._url(self.cfg.snap_url)}\n")
                f.write(f"res_url : {self._url(self.cfg.result_url)}\n")
                f.write(f"response:\n{text}\n")
        except OSError:
            pass


# ═════════════════════════════════════════════ 工厂

def build_plate_reader(cfg, mode: str = "", save_dir: str = "",
                       rotate: int = 0) -> PlateReaderBase:
    """
    按配置造车牌读取器。

    mode 空字符串时按配置推断：
        enabled=false            → Null（服务照常跑）
        mode/simulate=true       → 仿真出牌
        配了 snap_url/result_url → HTTP 真机

    rotate：仅仿真模式有意义 —— 让多路通道从 sample_plates 的不同位置起牌，
            避免四路看板同时显示同一张车牌（详见 SimulatedPlateReader）。
    """
    m = (mode or getattr(cfg, "mode", "") or "").lower()
    if m in ("null", "off", "none"):
        return NullPlateReader(cfg)
    if m in ("simulate", "sim", "demo"):
        return SimulatedPlateReader(cfg, plates=getattr(cfg, "sample_plates", None),
                                    rotate=rotate)
    if not cfg.enabled:
        return NullPlateReader(cfg)
    if cfg.snap_url or cfg.result_url:
        return HttpPlateReader(cfg, save_dir=save_dir)
    return NullPlateReader(cfg)


# ═════════════════════════════════════════════ 目录名安全化

# Windows 保留设备名（含数字 1~9 的变体）：作为路径分量时 makedirs 直接失败。
_WINDOWS_RESERVED = frozenset(
    {"CON", "PRN", "AUX", "NUL"}
    | {"COM%d" % i for i in range(1, 10)}
    | {"LPT%d" % i for i in range(1, 10)})


# ── 空车基线按车牌存（2026-09-23）───────────────────────────
BASELINE_SUFFIX = ".baseline.npz"


def baseline_file(baseline_dir: str, plate: str) -> str:
    """这辆车的空车基线文件路径：`<基线目录>/<规范化车牌>.baseline.npz`。

    ★ 服务（`ChannelWorker.baseline_path_for`）与 `cli/baseline.py` **必须走同一个
      函数**。两边各拼各的，CLI 标定出来的基线服务就查不到 —— 而且不报错，
      只会表现为"这辆车没有基线"。车牌先过 `safe_dir_name`：非法字符、长度、
      NFC、大小写折叠都在那里处理，同一辆车的不同写法落到同一个文件。
    """
    return os.path.join(baseline_dir, safe_dir_name(plate) + BASELINE_SUFFIX)


def safe_dir_name(plate: str) -> str:
    """
    把车牌转成安全的目录名。

    ★ 这里顺带修掉原版的一个真实缺陷
        原版直接用相机返回的车牌建目录。相机给的是 **GBK** 字节，
        而程序把 std::string 原样当 UTF-8 存 → 中文省份简称被写坏，
        现场 baseline 下才会出现 `瀹丄09692D`（"宁A"）、
        `钂檆02002F`（"蒙C"）这种乱码目录，且同一辆车会分裂成两个目录
        （`钂機02002F` 与 `钂檆02002F` —— "C" 的两种坏法）。

    本实现统一走 UTF-8，并对文件系统非法字符做替换，
    再叠一层"同一辆车只可能有一个目录"的保证。
    tools/fix_plate_dirs.py 可以把历史乱码目录就地修好。

    另外还兜住两个文件系统层面的坑（相机返回垃圾串时才会踩到）：
        · 长度：单个路径分量在多数文件系统上是 255 **字节**上限，
          而一个汉字占 3 字节 → 超过 85 个汉字就会 makedirs 失败。
          这里按字节数截断，并追加内容短哈希，保证截断后不同车牌不会撞车。
        · 等价写法：做 NFC 归一化 + 英文字母统一大写，避免"同一个车牌的不同写法"
          各自建一个目录（这正是原版"一辆车两个目录"的另一半原因 ——
          现场同时存在 `钂機02002F` 与 `钂檆02002F`，修回来都是"蒙C88593X"）。
          车牌字母本来就是大写，所以这里折叠大小写不会误合并不同车牌。
    """
    if not plate:
        return EMPTY_PLATE
    bad = '\\/:*?"<>|\r\n\t'
    s = unicodedata.normalize("NFC", plate.strip()).upper()
    out = "".join("_" if ch in bad else ch for ch in s)
    out = out.strip(". ")
    if not out:
        return EMPTY_PLATE

    # ★ Windows 保留设备名（2026-09-23，低严重度批次）：CON/NUL/COM1~9/
    #   LPT1~9/AUX/PRN（含带扩展名形式，如 "CON.txt"）在 Windows 上
    #   os.makedirs 直接失败，该车无法落盘。命中时加 "~" 前缀转义；
    #   正常车牌必有省份汉字，只有相机吐垃圾串时才会踩到。
    stem = out.split(".", 1)[0]
    if stem in _WINDOWS_RESERVED:
        out = "~" + out

    # 单分量最多 255 字节；留出后缀余量，按字节截断且不切碎多字节字符
    limit = 200
    if len(out.encode("utf-8")) > limit:
        digest = hashlib.sha1(out.encode("utf-8")).hexdigest()[:8]
        keep: List[str] = []
        used = 0
        for ch in out:
            n = len(ch.encode("utf-8"))
            if used + n > limit - 9:                 # 9 = "_" + 8 位哈希
                break
            keep.append(ch)
            used += n
        out = "".join(keep) + "_" + digest
    return out
