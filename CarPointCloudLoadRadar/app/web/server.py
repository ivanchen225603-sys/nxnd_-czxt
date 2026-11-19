#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
web/server.py — 看板 HTTP 服务（只用标准库，零第三方依赖）

为什么不用 Flask/FastAPI？
    现场是**离线内网**，麒麟机器上装 pip 包很麻烦，还可能根本没有外网源。
    http.server 完全够用，而且少一层依赖就少一个现场装不上的理由。

路由
    GET  /                       独立模式看板页面（后台接管时关闭）
    GET  /static/<file>          独立模式静态资源（后台接管时关闭）
    GET  /api/health             存活探测（给 systemd / 巡检脚本用）
    GET  /api/state              全量快照；history=0省略最近结果，frame_integrity=1附带逐帧采证
    GET  /api/channel/<id>       单通道快照
    GET  /api/history            历史（?channel=CH1&limit=100）
    GET  /api/events             SSE 实时事件流
    GET  /api/config             生效配置（脱敏）
    POST /api/demo/loaded        {channel, loaded} 仿真场景切换
    POST /api/demo/force         {channel} 强制开新会话（重测一次）
    GET  /api/export/ply         导出某通道最新点云为 PLY（用 CloudCompare 看）
    GET  /api/export/xyz         导出某通道最新点云为 .xyz（原厂格式）

线程模型
    ThreadingHTTPServer：每个请求一个线程。SSE 连接会长期占用一个线程，
    所以加了连接数上限，避免看板开多了把服务拖死。
"""
from __future__ import annotations

import io
import json
import os
import threading
import time
import urllib.parse
import math
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Optional

from ..settings import root
from ..pointcloud import export_ply, Frame, save_xyz, load_xyz
from ..service import LidarService

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")

_MIME = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".svg": "image/svg+xml",
    ".png": "image/png",
    ".ico": "image/x-icon",
    ".woff2": "font/woff2",
}

MAX_SSE = 8                       # 同时最多 8 个看板连接


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "LidarDashboard/1.0"
    protocol_version = "HTTP/1.1"
    service: LidarService = None            # 由 serve() 注入

    # ── 基础输出 ────────────────────────────────────────────
    def log_message(self, fmt: str, *args) -> None:
        # 静默；需要排查时把下面这行注释掉
        if os.environ.get("LIDAR_WEB_VERBOSE"):
            super().log_message(fmt, *args)

    def handle(self) -> None:
        """吞掉"客户端在读写中途断开连接"这类噪音异常。

        关窗时浏览器会立刻掐断它所有连接（尤其是 SSE 长连接），
        Windows 上表现为 ConnectionAbortedError(WinError 10053)，
        位置在 http.server 读取请求行/写响应体时。

        这**不是错误** —— 用户关了窗口本来就会这样。但如果不拦，
        异常会冒到 socketserver.handle_error，往 stderr 打一整段 traceback；
        桌面模式下 stderr 会进 logs/desktop.log，现场工程师一看堆栈
        就以为程序崩了，专门来问。只吞这三类连接层异常，真实错误照旧暴露。
        """
        try:
            super().handle()
        except (ConnectionAbortedError, ConnectionResetError, BrokenPipeError):
            pass

    def _send(self, code: int, body: bytes, ctype: str,
              extra: Optional[Dict[str, str]] = None) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        if self.close_connection:
            self.send_header("Connection", "close")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _json(self, obj: Any, code: int = 200) -> None:
        self._send(code, _json_dumps(obj).encode("utf-8"),
                   "application/json; charset=utf-8")

    def _close_rejected_request(self) -> None:
        """拒绝小型 POST 前有界丢弃正文，避免 Windows 未读字节触发 RST 丢失错误回执。"""
        self.close_connection = True
        if self.command != "POST" or self.headers.get("Transfer-Encoding"):
            return
        lengths = self.headers.get_all("Content-Length", [])
        if len(lengths) != 1 or len(lengths[0]) > 16 or not lengths[0].isascii() or not lengths[0].isdigit():
            return
        remaining = int(lengths[0])
        if not 0 < remaining <= 4096:
            return
        timeout = self.connection.gettimeout()
        deadline = time.monotonic() + 1.0
        try:
            while remaining and time.monotonic() < deadline:
                self.connection.settimeout(max(0.001, deadline - time.monotonic()))
                data = self.rfile.read1(remaining)
                if not data:
                    break
                remaining -= len(data)
        except OSError:
            pass
        finally:
            self.connection.settimeout(timeout)

    def _read_json(self) -> Dict:
        try:
            n = int(self.headers.get("Content-Length") or 0)
            # ★ 上限 1MB（2026-09-23，低严重度批次）：通用 POST 分支
            #   （demo / baseline）原来对声明长度不设限，恶意或畸形的
            #   超大声明会让 rfile.read(n) 长期占住线程吃内存。
            #   现场合法载荷只有几百字节，1MB 绰绰有余。
            if n <= 0 or n > (1 << 20):
                return {}
            return json.loads(self.rfile.read(n).decode("utf-8"))
        except (ValueError, OSError):
            return {}

    # ── 路由 ────────────────────────────────────────────────
    def do_GET(self) -> None:                       # noqa: N802
        u = urllib.parse.urlparse(self.path)
        path = u.path
        q = urllib.parse.parse_qs(u.query)
        if (getattr(self.service, "integration", None) is not None
                and (path in ("/", "/index.html") or path.startswith("/static/"))):
            return self._json({"ok": False, "error": "已由 CarPointCloudWebApi 接管，请使用后台载货雷达页面"}, 403)
        if path.startswith("/api/integration/commands/"):
            bridge = self._integration_auth()
            if bridge is None:
                return
            request_id = urllib.parse.unquote(path[len("/api/integration/commands/"):])
            try:
                result = bridge.command(request_id)
            except ValueError:
                return self._json({"ok": False, "error": "命令标识或完成凭据无效，请核对历史"}, 400)
            except Exception:
                return self._json({"ok": False, "error": "命令结果暂时无法确认，请保留原编号查询"}, 503)
            return self._json(result if result else {"ok": False, "error": "命令不存在"},
                              200 if result else 404)

        if path in ("/", "/index.html"):
            return self._serve_file(os.path.join(STATIC_DIR, "index.html"))
        if path.startswith("/static/"):
            rel = urllib.parse.unquote(path[len("/static/"):])
            return self._serve_static(rel)
        if path == "/api/health":
            return self._json({"ok": True, "t": time.time(),
                               "channels": len(self.service.workers)})
        if path == "/api/state":
            if q.get("frame_integrity", ["0"])[0] == "1":
                return self._json(self.service.snapshot(
                    include_history=q.get("history", ["1"])[0] != "0",
                    include_frame_integrity=True))
            # ?history=0：不带各通道最近结果（WebApi 用；它只读白名单字段）。默认仍为全量，看板与记录器不变。
            # 默认调用保持原签名，只有显式要求省略时才传新参数。
            if q.get("history", ["1"])[0] == "0":
                return self._json(self.service.snapshot(include_history=False))
            return self._json(self.service.snapshot())
        if path.startswith("/api/channel/"):
            cid = urllib.parse.unquote(path[len("/api/channel/"):])
            w = self.service.by_id(cid)
            if w is None:
                return self._json({"error": f"通道不存在: {cid}"}, 404)
            return self._json(w.snapshot())
        if path == "/api/history":
            cid = (q.get("channel") or [""])[0]
            # ★ 畸形参数给 400 而不是打穿 handler（低严重度批次）：
            #   ?limit=abc 原来直接 ValueError 冒到 handle_error 往
            #   stderr 刷整段 traceback —— 现场会以为服务崩了。
            try:
                limit = int((q.get("limit") or ["100"])[0])
            except ValueError:
                return self._json({"error": "limit 必须是整数"}, 400)
            if not 1 <= limit <= 1000:
                return self._json({"error": "limit 必须在 1~1000"}, 400)
            if cid:
                w = self.service.by_id(cid)
                rows = w.history_snapshot(limit) if w else []
            else:
                rows = []
                for w in self.service.workers:
                    rows.extend(w.history_snapshot(limit))
                rows.sort(key=lambda r: r.get("ts", 0))
                rows = rows[-limit:]
            return self._json({"rows": rows, "count": len(rows)})
        if path == "/api/config":
            return self._json(self._effective_config())
        if path == "/api/events":
            return self._sse()
        if path in ("/api/export/ply", "/api/export/xyz"):
            cid = (q.get("channel") or [""])[0]
            w = self.service.by_id(cid) or (self.service.workers[0]
                                            if self.service.workers else None)
            if w is None:
                return self._json({"error": "没有可用通道"}, 404)
            return self._export(w, "ply" if path.endswith("ply") else "xyz")

        self._json({"error": "未知路径", "path": path}, 404)

    def do_POST(self) -> None:                      # noqa: N802
        u = urllib.parse.urlparse(self.path)
        bridge = getattr(self.service, "integration", None)
        if u.path == "/api/integration/commands":
            bridge = self._integration_auth()
            if bridge is None:
                return
            try:
                length = int(self.headers.get("Content-Length") or 0)
                if self.headers.get("Transfer-Encoding") or not 0 < length <= 4096:
                    self.close_connection = True
                    return self._json({"ok": False, "error": "请求体长度错误"}, 400)
                body = json.loads(self.rfile.read(length).decode("utf-8"))
                result = bridge.queue_command(body)
                return self._json(result)
            except ValueError as exc:
                from ..load_radar_bridge import BridgeConflict
                return self._json({"ok": False, "error": "命令格式错误或车辆状态不匹配"},
                                 409 if isinstance(exc, BridgeConflict) else 400)
            except Exception:
                return self._json({"ok": False, "error": "命令结果暂时无法确认，请保留原编号查询"}, 503)
        if bridge is not None:
            self._close_rejected_request()
            return self._json({"ok": False, "error": "已由业务后台接管，禁止裸调试写入"}, 403)
        body = self._read_json()
        if u.path == "/api/demo/loaded":
            cid = str(body.get("channel") or "CH1")
            loaded = bool(body.get("loaded"))
            ok = self.service.set_loaded_demo(cid, loaded)
            if not ok:
                return self._json({"ok": False,
                                   "error": "该通道不是仿真源，无法切换场景"}, 400)
            return self._json({"ok": True, "channel": cid, "loaded": loaded})
        if u.path == "/api/demo/force":
            cid = str(body.get("channel") or "")
            n = self.service.force_new_session(cid)
            if n == 0:
                return self._json({"ok": False, "channels": 0,
                                   "error": "没有可重置的活动通道，通道不存在或已停止"}, 409)
            return self._json({"ok": True, "channels": n})
        if u.path == "/api/demo/auto":
            # 自动演示编排：开着时四路仿真通道会按时间线自己演
            # （车驶入 → 停稳 → 出结论 → 驶离 → 循环）。手动点过按钮后会被自动暂停，
            # 用这个接口恢复。详见 app/demo_director.py。
            want = body.get("enabled")
            if want is None:
                want = not bool((self.service.director.status().get("auto")
                                 if self.service.director else False))
            ok = self.service.set_auto_demo(bool(want))
            if not ok:
                return self._json({"ok": False,
                                   "error": "本次不是演示启动，没有自动编排器"}, 400)
            return self._json({"ok": True, "auto": bool(want),
                               "director": self.service.director.status()})
        if u.path == "/api/baseline/capture":
            # 空车基线标定：必须由现场的人确认"这确实是一辆空车"才点，
            # 所以做成显式接口而不是自动触发（存错基线之后所有体积都错且不报错）。
            # ★ 基线按车牌存（2026-09-23）：可选的 plate 由操作员明确给出；
            #   不给就由通道现读一次（车正停在检测位上）。读不到牌会被拒绝 ——
            #   存成「未知车牌」等于所有认不出牌的车共用一份基线。
            cid = str(body.get("channel") or "")
            if not cid:
                return self._json({"ok": False, "error": "缺少 channel"}, 400)
            plate = str(body.get("plate") or "").strip()
            ok, msg = self.service.capture_baseline(cid, plate)
            return self._json({"ok": ok, "channel": cid, "plate": plate, "message": msg},
                              200 if ok else 400)
        self._json({"error": "未知路径"}, 404)

    def _integration_auth(self):
        bridge = getattr(self.service, "integration", None)
        if bridge is None:
            self._close_rejected_request()
            self._json({"ok": False, "error": "后台接口未启用"}, 503)
            return None
        if not bridge.authorized(self.headers.get("X-Load-Radar-Node", ""),
                                 self.headers.get("X-Load-Radar-Key", "")):
            self._close_rejected_request()
            self._json({"ok": False, "error": "节点认证失败"}, 401)
            return None
        return bridge

    # ── 静态资源 ────────────────────────────────────────────
    def _serve_static(self, rel: str) -> None:
        safe = os.path.normpath(rel).replace("\\", "/")
        if safe.startswith("..") or os.path.isabs(safe):
            return self._json({"error": "非法路径"}, 400)
        return self._serve_file(os.path.join(STATIC_DIR, safe))

    def _serve_file(self, fp: str) -> None:
        if not os.path.isfile(fp):
            return self._json({"error": f"文件不存在: {os.path.basename(fp)}"}, 404)
        ctype = _MIME.get(os.path.splitext(fp)[1].lower(), "application/octet-stream")
        with open(fp, "rb") as f:
            data = f.read()
        self._send(200, data, ctype)

    # ── SSE ─────────────────────────────────────────────────
    def _sse(self) -> None:
        if self.service.bus.subscribers >= MAX_SSE:
            return self._json({"error": f"实时连接数已达上限 {MAX_SSE}，"
                                        f"多余的看板请改用轮询 /api/state"}, 503)
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache, no-transform")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()

        # ★ SSE 是长连接，必须给写设超时（2026-09-23，两轮审查 M12）：
        #   客户端半断开（拔网线 / 休眠，收不到 RST）时 wfile.write 会永久
        #   阻塞在 send —— finally 的退订永远执行不到，8 个 SSE 名额被僵尸
        #   连接占满后新看板整体 503，直到重启。20 秒比心跳（15 秒）长：
        #   正常连接从不会超时；卡死的连接由下一次心跳写触发 socket.timeout
        #   （OSError 子类，下面的 except 接得住）-> 退订收尾，名额释放。
        prev_timeout = self.connection.gettimeout()
        self.connection.settimeout(20.0)
        q = self.service.bus.subscribe()
        try:
            # 先补一个全量快照，让刚打开/刷新的页面立刻有内容
            self._sse_write("snapshot", self.service.snapshot())
            last_beat = time.time()
            while True:
                try:
                    ev = q.popleft()
                except IndexError:
                    if time.time() - last_beat > 15.0:
                        last_beat = time.time()
                        self.wfile.write(b": ping\n\n")
                        self.wfile.flush()
                    time.sleep(0.05)
                    continue
                self._sse_write(ev.get("kind", "event"), ev)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            self.service.bus.unsubscribe(q)
            self.connection.settimeout(prev_timeout)

    def _sse_write(self, kind: str, obj: Any) -> None:
        payload = _json_dumps(obj)
        self.wfile.write(f"event: {kind}\ndata: {payload}\n\n".encode("utf-8"))
        self.wfile.flush()

    # ── 配置 / 导出 ─────────────────────────────────────────
    def _effective_config(self) -> Dict:
        st = self.service.st
        chans = []
        for c in st.channels:
            chans.append({
                "id": c.id, "name": c.name, "enabled": c.enabled, "mode": c.mode,
                "group": c.group, "note": c.note,
                "lidar": {"ip": c.lidar.ip, "udp_port": c.lidar.udp_port,
                          "bind_ip": c.lidar.bind_ip,
                          "range_mode": c.lidar.range_mode,
                          "z_down": c.lidar.z_down,
                          "frame_mode": c.lidar.frame_mode,
                          "frame_ms": c.lidar.frame_ms,
                          "max_range_m": c.lidar.max_range_m},
                "output_dir": c.output_dir, "baseline_dir": c.baseline_dir,
                "replay_dir": c.replay_dir, "save": c.save,
            })
        p = st.plate
        return {
            "service": {"host": st.service.host, "port": st.service.port,
                        "data_root": st.service.data_root,
                        "log_dir": st.service.log_dir,
                        "state_push_hz": st.service.state_push_hz},
            "channels_path": st.channels_path,
            "channels": chans,
            # 车牌：不返回 headers（可能含鉴权信息）
            "plate": {"enabled": p.enabled, "mode": p.mode,
                      "base_url": p.base_url, "device_id": p.device_id,
                      "snap_url": p.snap_url, "result_url": p.result_url,
                      "plate_field": p.plate_field,
                      "f2_timeout_ms": p.f2_timeout_ms, "f3_timeout_ms": p.f3_timeout_ms,
                      "headers": sorted((p.headers or {}).keys())},
            "stability": {"stationary_warn_sec": st.stability.stationary_warn_sec,
                          "result_hold_sec": st.stability.result_hold_sec,
                          "min_save_points": st.stability.min_save_points},
        }

    def _export(self, w, fmt: str) -> None:
        """
        导出"当前通道最新一帧"的点云。
        现场排查的标准动作：导成 PLY 用 CloudCompare 打开，
        比在网页上看散点图有用一百倍。
        """
        f = w.frame_snapshot()
        if f is None or f.n == 0:
            return self._json({"error": "该通道还没有可用的一帧"}, 404)
        ts = time.strftime("%Y%m%d_%H%M%S")
        # ★ 同秒并发导出原来写同名文件互相覆盖（读到半截）——
        #   加 4 位随机后缀区分（2026-09-23，低严重度批次）。
        uniq = uuid.uuid4().hex[:4]
        if fmt == "ply":
            out = os.path.join(root("logs", "export"), f"{w.ch.id}_{ts}_{uniq}.ply")
            os.makedirs(os.path.dirname(out), exist_ok=True)
            export_ply(f, out)
            ctype, name = "application/octet-stream", os.path.basename(out)
        else:
            out = os.path.join(root("logs", "export"), f"{w.ch.id}_{ts}_{uniq}.xyz")
            os.makedirs(os.path.dirname(out), exist_ok=True)
            save_xyz(out, f)
            ctype, name = "application/octet-stream", os.path.basename(out)
        with open(out, "rb") as fp:
            data = fp.read()
        self._send(200, data, ctype,
                   {"Content-Disposition": f'attachment; filename="{name}"'})


def _finite_json(value):
    """诊断缺测为 null；不让非有限浮点使整个节点快照成为无效 JSON。"""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: _finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite_json(item) for item in value]
    return value


def _json_dumps(value):
    """HTTP 与 SSE 共用严格 JSON 出口，保留有限数值与既有 numpy 转换。"""
    return json.dumps(_finite_json(value), ensure_ascii=False, allow_nan=False,
                      default=lambda item: _finite_json(_json_default(item)))


def _json_default(o):
    """numpy 标量/数组没法直接 json，统一降级成 Python 原生类型。"""
    try:
        import numpy as np
        if isinstance(o, (np.floating,)):
            return float(o)
        if isinstance(o, (np.integer,)):
            return int(o)
        if isinstance(o, np.ndarray):
            return o.tolist()
    except Exception:                                    # noqa: BLE001
        pass
    return str(o)


# ═════════════════════════════════════════════ 启动


class _ExclusiveHTTPServer(ThreadingHTTPServer):
    """看板 HTTP 服务，端口绑定策略与 desktop/launcher.port_free 保持一致。

    ★ 为什么不能沿用 stdlib 默认的 allow_reuse_address=1：
      HTTPServer 默认打开 SO_REUSEADDR。Unix 上它只影响 TIME_WAIT 残留，
      但 **Windows 上它允许另一个进程抢占一个正在被监听的端口** ——
      后果是两个实例都以为自己绑上了 8080，请求却只被其中一个收到，
      现场表现为"看板有时能打开、有时打不开"，且不报任何错。

      改成：Windows 上关掉 SO_REUSEADDR 并加 SO_EXCLUSIVEADDRUSE，
      端口真被占时 bind 直接抛错 → desktop/app.py 会顺延到下一个空闲端口，
      行为可见、可排查。已实测：监听套接字关闭后可立即重绑，不会因
      TIME_WAIT 导致"重启就换端口"。
    """

    def server_bind(self):
        if os.name == "nt":
            self.allow_reuse_address = False          # 先禁用再 bind
            try:
                import socket as _socket
                self.socket.setsockopt(
                    _socket.SOL_SOCKET, _socket.SO_EXCLUSIVEADDRUSE, 1)
            except (AttributeError, OSError):
                pass
        super().server_bind()


def serve(service: LidarService, host: str = "", port: int = 0,
          open_browser: bool = False) -> ThreadingHTTPServer:
    """起 HTTP 服务；后台接管时只保留接口，不打开独立看板。"""
    h = host or service.st.service.host or "0.0.0.0"
    p = int(port or service.st.service.port or 8080)

    handler = type("BoundHandler", (DashboardHandler,), {"service": service})
    httpd = _ExclusiveHTTPServer((h, p), handler)
    httpd.daemon_threads = True
    if open_browser and getattr(service, "integration", None) is None:
        threading.Timer(0.8, lambda: _open(h, p)).start()
    return httpd


def _open(host: str, port: int) -> None:
    import webbrowser
    url_host = "127.0.0.1" if host in ("0.0.0.0", "::", "") else host
    try:
        webbrowser.open(f"http://{url_host}:{port}/")
    except Exception:                                    # noqa: BLE001
        pass
