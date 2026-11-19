#!/usr/bin/env python3
"""在本机测试库上联调四系统；默认使用隔离端口，也可显式替换 5031。"""
from __future__ import annotations

import argparse
import copy
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import secrets
import socket
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request


PROJECT = Path(__file__).resolve().parents[1]
REPO = PROJECT.parent
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def choose_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def save(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


def call(url: str, method: str = "GET", body: object | None = None,
         token: str = "", timeout: int = 12,
         headers: dict[str, str] | None = None) -> tuple[int, object]:
    data = json.dumps(body, ensure_ascii=False).encode("utf-8") if body is not None else None
    request = urllib.request.Request(url, data=data, method=method)
    if data is not None:
        request.add_header("Content-Type", "application/json")
    if token:
        request.add_header("Authorization", "Bearer " + token)
    for name, value in (headers or {}).items():
        request.add_header(name, value)
    try:
        with OPENER.open(request, timeout=timeout) as response:
            status, raw = response.status, response.read()
    except urllib.error.HTTPError as error:
        status, raw = error.code, error.read()
    try:
        return status, json.loads(raw)
    except ValueError:
        return status, raw.decode("utf-8", errors="replace")[:500]


def launch(name: str, command: list[str], cwd: Path, env: dict[str, str],
           health: str, registry: list, root: Path) -> None:
    stream = (root / (name + ".log")).open("w", encoding="utf-8")
    flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
    process = subprocess.Popen(command, cwd=cwd, env=env, stdout=stream,
                               stderr=subprocess.STDOUT, creationflags=flags)
    registry.append((name, process, stream))
    deadline = time.monotonic() + 50
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"{name} 提前退出，代码 {process.returncode}")
        try:
            status, _ = call(health, timeout=3)
            if status == 200:
                print(name, "已启动", process.pid, flush=True)
                return
        except OSError:
            pass
        time.sleep(0.3)
    raise RuntimeError(name + " 健康检查超时")


def wait_port_free(port: int, timeout: float = 20) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with socket.socket() as sock:
            sock.settimeout(0.3)
            if sock.connect_ex(("127.0.0.1", port)) != 0:
                return
        time.sleep(0.2)
    raise RuntimeError(f"端口 {port} 未释放")


def stop_existing(pid: int) -> None:
    if os.name != "nt":
        raise RuntimeError("本工具只用于当前 Windows 联调机")
    subprocess.run(["powershell", "-NoProfile", "-Command",
                    f"Stop-Process -Id {pid} -Force"], check=True,
                   stdout=subprocess.DEVNULL)
    wait_port_free(5031)


def start_plate_stub(port: int, plate: str):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if not self.path.startswith("/api/v1/licensePlateRecords?cameraIdentifier=CAM_004"):
                self.send_error(404)
                return
            captured = datetime.now(timezone.utc).astimezone().strftime("%Y-%m-%d %H:%M:%S")
            body = json.dumps({"code": 200, "message": "success", "data": [
                {"cameraId": "CAM_004", "licensePlate": plate, "captureTime": captured}
            ]}, ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format, *args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def send_simulated_jt_packets(root: Path, udp_port: int, stop: threading.Event):
    """按正式报文布局生成数据，并从实际 UDP 接收入口注入。"""
    sys.path.insert(0, str(PROJECT))
    from app.pointcloud import AngleCache
    from app.radar_sim import Box, RadarSimulator, box_ground, make_demo_scene
    from app.settings import load_calib

    calib = load_calib(str(PROJECT / "config" / "config.json"))
    angles = AngleCache(str(PROJECT / "config" / "angle_cache.csv")).load()
    sim = RadarSimulator(calib.packet, calib.processing, angles,
                         range_mode="scale_offset", z_down=True, az_step_deg=0.4,
                         range_noise_m=0.001, dropout_prob=0.001, seed=20260925)
    ground = box_ground(z_floor=-5.0)
    truck = make_demo_scene(loaded=False, z_floor=-5.0)
    manifest = {"sourceIp": "127.0.0.4", "destination": f"127.0.0.1:{udp_port}",
                "packetBytes": calib.packet.size, "azimuthStepDegrees": 0.4,
                "intensity": "uint8 field in each return", "distance": "uint16 scaled-offset range in each return",
                "rotations": []}
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.4", 0))
    try:
        index = 0
        while not stop.is_set() and index < 360:
            if index < 8:
                phase, offset = "idle", None
            elif index < 14:
                phase, offset = "entering", float(14 - index) * 1.5
            else:
                phase, offset = "stationary_empty_truck", 0.0
            boxes = [ground]
            if offset is not None:
                boxes = [Box((b.lo[0], b.lo[1] + offset, b.lo[2]),
                             (b.hi[0], b.hi[1] + offset, b.hi[2]), b.hollow)
                         for b in truck] + boxes
            packets = sim.scan_packets(boxes, 0.0, 0.4, 900)
            sent_at = datetime.now(timezone.utc).isoformat()
            for start in range(0, len(packets), 30):
                if stop.is_set():
                    break
                for packet in packets[start:start + 30]:
                    sock.sendto(packet, ("127.0.0.1", udp_port))
                time.sleep(0.003)
            if index in (0, 8, 14):
                first = packets[0]
                manifest["rotations"].append({"index": index, "phase": phase,
                    "sentAtUtc": sent_at, "packets": len(packets),
                    "firstAzimuthDegrees": struct.unpack_from("<H", first, calib.packet.angle1_offset)[0] / 100,
                    "firstDistanceRaw": struct.unpack_from("<H", first, calib.packet.block1_data_offset)[0],
                    "firstIntensity": first[calib.packet.block1_data_offset + 2]})
                save(root / "jt-packet-manifest.json", manifest)
            index += 1
            stop.wait(0.18)
    finally:
        sock.close()
        save(root / "jt-packet-manifest.json", manifest)


def prepare(root: Path, webapi_port: int = 5031) -> dict[str, str]:
    data = root / "data"
    data.mkdir(parents=True, exist_ok=True)
    for name in ("Analyzer", "Worker", "WebApi", "JT128"):
        (root / name).mkdir(exist_ok=True)
    ports = {name: choose_port() for name in ("Analyzer", "Worker", "JT128", "JTUDP", "PlateStub")}
    urls = {name: f"http://127.0.0.1:{ports[name]}"
            for name in ("Analyzer", "Worker", "JT128")}
    urls["WebApi"] = f"http://127.0.0.1:{webapi_port}"
    extrinsic = data / "extrinsic.json"
    save(extrinsic, {
        "version": "v1", "frame": "B_to_A", "cameraA": "CAM_004", "cameraB": "CAM_005",
        "matrix": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
        "baselineResidual": 0.018, "weighbridgePlaneA": [0, 0, 1, 0],
        "quality": {"fitness": 0.82, "inlierRmse": 0.018,
                    "correspondenceCount": 42000, "degenerateAxes": [],
                    "conditionNumber": 12.4, "trustworthy": True, "warnings": []},
    })
    analyzer = json.loads((REPO / "CarPointCloud_Analyzer_fixed" / "appsettings.json")
                          .read_text(encoding="utf-8-sig"))
    analyzer.update(CARPOINT_SERVICE_HOST="127.0.0.1",
                    CARPOINT_SERVICE_PORT=ports["Analyzer"],
                    CARPOINT_SERVICE_ALLOWED_ROOT=str(data),
                    CARPOINT_EXTRINSIC_PATH=str(extrinsic),
                    CARPOINT_DB_ENABLED=True,
                    CARPOINT_EMPTY_PROFILE_ENABLED=True,
                    CARPOINT_EMPTY_PROFILE_ROOT=str(data / "empty-profiles"))
    save(root / "Analyzer" / "appsettings.json", analyzer)
    worker = copy.deepcopy(json.loads((REPO / "CarPointCloudLadarWorker" / "appsettings.json")
                                      .read_text(encoding="utf-8-sig"))["LadarCapture"])
    worker.update(EnableSyntheticRadar=True, EnableApiRequests=False,
                  EnableResultUploadToWebApi=False, EnablePythonAnalyzer=True,
                  FetchLicensePlateOnCapture=False, EnableLivePreview=True,
                  EnableApiTriggeredScanOnce=True, EnableHttpApi=True,
                  HttpApiHost="127.0.0.1", HttpApiPort=ports["Worker"],
                  AnalyzerServiceUrl=urls["Analyzer"], WebApiUrl=urls["WebApi"],
                  DualRadarExtrinsicPath=str(extrinsic),
                  AnalyzerBaselineDirectory=str(data / "analyzer-baselines"),
                  SyntheticFrameIntervalSeconds=4.04, SyntheticPointCount=140000)
    for key in ("CaptureDirectory", "PairManifestDirectory", "AnalysisQueueDirectory",
                "UploadQueueDirectory", "HealthSnapshotDirectory"):
        worker[key] = str(data / key)
    for index, device in enumerate(worker["Devices"]):
        device["Address"] = f"192.0.2.{index + 11}"
    worker["Address"] = "192.0.2.11"
    save(root / "Worker" / "appsettings.json", {"LadarCapture": worker})
    web = json.loads((REPO / "CarPointCloudWebApi" / "appsettings.json")
                     .read_text(encoding="utf-8-sig"))
    web["Kestrel"] = {"Endpoints": {"Http": {"Url": urls["WebApi"]}}}
    web["Radar"]["BaseUrl"] = urls["Worker"]
    web["Radar"]["LicensePlateMockEnabled"] = True
    web["Radar"]["LicensePlateMockRandom"] = True
    web["Radar"]["LicensePlateMockHoldSeconds"] = 3600
    web["Radar"]["LicensePlateMockPrefix"] = "宁A"
    web["Integration"].update(ExternalPartnerEnabled=True,
        PartnerBaseUrl=f"http://127.0.0.1:{ports['PlateStub']}",
        PartnerFirstCallDelayMs=0, PartnerRetryDelayMs=500,
        EnableMockLicensePlate=False)
    web["EmptyVehicleProfile"]["StorageRoot"] = str(data / "empty-profiles")
    recorder_token = secrets.token_urlsafe(32)
    web.setdefault("RadarLive", {})["RecorderToken"] = recorder_token
    save(root / "JT128" / "recorder.json", {"recorderToken": recorder_token})
    node_key = secrets.token_urlsafe(36)
    web["LoadRadar"] = {
        "Enabled": True, "StorageRoot": str(data / "load-radar-assets"),
        "Nodes": [{"NodeId": "jt128-codex-e2e", "AgentKey": node_key,
                   "BaseUrl": urls["JT128"],
                   "Channels": [{"ChannelId": "CH4", "CameraIdentifier": "CAM_004",
                                 "Name": "四号车道模拟"}]}],
    }
    save(root / "WebApi" / "appsettings.json", web)
    channels = json.loads((PROJECT / "config" / "channels.json").read_text(encoding="utf-8-sig"))
    channels["service"].update(host="127.0.0.1", port=ports["JT128"],
                               data_root=str(data / "jt-data"),
                               log_dir=str(data / "jt-logs"))
    channels["plate"]["base_url"] = urls["WebApi"]
    for channel in channels["channels"]:
        channel["enabled"] = channel["id"] == "CH4"
        if channel["id"] == "CH4":
            channel["lidar"].update(ip="127.0.0.4", source_ip="127.0.0.4",
                                    bind_ip="127.0.0.1", udp_port=ports["JTUDP"])
        for key in ("output_dir", "baseline_dir", "replay_dir"):
            channel[key] = str(data / "jt" / key / channel["id"])
    save(root / "JT128" / "channels.json", channels)
    bridge = {
        "enabled": True, "node_id": "jt128-codex-e2e", "base_url": urls["WebApi"],
        "agent_key": node_key, "storage_dir": str(data / "jt-outbox"),
        "algorithm_version": "jt128-clean-room-20260917", "timeout_sec": 5,
        "retry_sec": 2, "allow_insecure_http": True,
        "channels": {"CH4": {"calibration_id": "CH4-codex-e2e"}},
    }
    save(root / "JT128" / "webapi.json", bridge)
    save(root / "ports.json", ports)
    return urls


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--worker-dll", required=True)
    parser.add_argument("--webapi-dll", required=True)
    parser.add_argument("--webapi-port", type=int, default=5031,
                        help="隔离 WebApi 端口；默认使用 5031 并替换现有服务")
    parser.add_argument("--replace-webapi-pid", type=int, default=0)
    parser.add_argument("--auto-capture", action="store_true",
                        help="启动停稳记录器，由它自动受理并显示双雷达流程")
    args = parser.parse_args()
    root = Path(args.output_dir).resolve()
    root.mkdir(parents=True, exist_ok=True)
    urls = prepare(root, args.webapi_port)
    node = json.loads((root / "JT128" / "webapi.json").read_text(encoding="utf-8"))
    agent_headers = {"X-Load-Radar-Node": node["node_id"],
                     "X-Load-Radar-Key": node["agent_key"]}
    recorder = json.loads((root / "JT128" / "recorder.json").read_text(encoding="utf-8"))
    capture_headers = {"X-Radar-Live-Token": recorder["recorderToken"]}
    env = {key: value for key, value in os.environ.items()
           if not key.upper().startswith(("CARPOINT_", "LADARCAPTURE__", "CONNECTIONSTRINGS__",
                                         "AUTH__", "INTEGRATION__", "ASPNETCORE_", "DOTNET_",
                                         "DATABASE__", "RADAR__", "LOAD_RADAR_CAPTURE_"))
           and "PROXY" not in key.upper()}
    env.update(DOTNET_ENVIRONMENT="OfflineSmoke", ASPNETCORE_ENVIRONMENT="OfflineSmoke",
               PYTHONIOENCODING="utf-8", NO_PROXY="*", no_proxy="*")
    registry = []
    web_stopped = False
    plate_stub = None
    sender_stop = threading.Event()
    sender = None
    sender_error = []
    flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
    report: dict = {"urls": urls, "checks": [], "realRadar": False,
                    "testDatabase": True, "syntheticGuardFps": 2.0, "flow": []}
    try:
        analyzer = REPO / "CarPointCloud_Analyzer_fixed"
        launch("Analyzer", [str(analyzer / ".venv" / "Scripts" / "python.exe"),
                            "-m", "carpointcloud_analyzer.service"],
               root / "Analyzer", dict(env, PYTHONPATH=str(analyzer)),
               urls["Analyzer"] + "/health", registry, root)
        launch("Worker", ["dotnet", str(Path(args.worker_dll).resolve())],
               root / "Worker", env, urls["Worker"] + "/api/health", registry, root)
        plate_stub = start_plate_stub(json.loads((root / "ports.json").read_text())["PlateStub"],
                                      "宁A85Q4P")
        if args.replace_webapi_pid:
            if args.webapi_port != 5031:
                raise RuntimeError("仅 5031 模式允许替换现有 WebApi")
            stop_existing(args.replace_webapi_pid)
            web_stopped = True
        launch("WebApi", ["dotnet", str(Path(args.webapi_dll).resolve()),
                          "--webroot", str(REPO / "CarPointCloudWebApi" / "wwwroot")],
               root / "WebApi", env, urls["WebApi"] + "/readyz", registry, root)
        auth_url = urls["WebApi"] + "/api/v1/radar-live/status"
        missing_status, _ = call(auth_url)
        wrong_status, _ = call(auth_url, headers={"X-Radar-Live-Token": "wrong-token"})
        valid_status, _ = call(auth_url, headers=capture_headers)
        outside_status, _ = call(urls["WebApi"] + "/api/v1/radar-live/fused-frame",
                                 headers=capture_headers)
        auth_check = {"node": "capture_auth", "mode": "fixed_token",
                      "webapiAccountUsed": False, "missingStatus": missing_status,
                      "wrongStatus": wrong_status, "validStatus": valid_status,
                      "outsideScopeStatus": outside_status}
        report["checks"].append(auth_check)
        if (missing_status, wrong_status, valid_status, outside_status) != (401, 401, 200, 401):
            raise RuntimeError("CH4 固定 Token 鉴权边界未通过: " + str(auth_check))
        plate_status, plate = call(urls["WebApi"] +
                                   "/api/v1/radar/licensePlate?cameraIdentifier=CAM_004")
        report["flow"].append({"node": "plate", "http": plate_status, "response": plate})
        launch("JT128", [sys.executable, str(PROJECT / "cli" / "run.py"),
                          "--mode", "live", "--save", "--channels",
                          str(root / "JT128" / "channels.json"), "--channel", "CH4",
                          "--webapi-config", str(root / "JT128" / "webapi.json"),
                          "--host", "127.0.0.1", "--port",
                          urls["JT128"].rsplit(":", 1)[1]],
               PROJECT, env, urls["JT128"] + "/api/health", registry, root)
        if args.auto_capture:
            recorder_stream = (root / "Recorder.log").open("w", encoding="utf-8")
            recorder_cwd = root / "Recorder"
            recorder_cwd.mkdir(exist_ok=True)
            # 本机 UDP 合成发送约 2.9 fps，2.0 只用于离线闸门；现场默认值不变。
            recorder = subprocess.Popen([
                sys.executable, str(PROJECT / "tools" / "field_standstill_log.py"),
                "--url", urls["JT128"] + "/api/state",
                "--capture-webapi", urls["WebApi"],
                "--capture-token-file", str(root / "JT128" / "recorder.json"),
                "--out", str(root / "Recorder-console.log"),
                "--no-bell", "--guard-fps", "2.0"],
                cwd=recorder_cwd, env=env,
                stdout=recorder_stream,
                stderr=subprocess.STDOUT, creationflags=flags)
            registry.append(("Recorder", recorder, recorder_stream))
        def run_sender():
            try:
                send_simulated_jt_packets(root,
                    json.loads((root / "ports.json").read_text())["JTUDP"], sender_stop)
            except Exception as error:
                sender_error.append(str(error))

        sender = threading.Thread(target=run_sender, daemon=True)
        sender.start()
        measurement_id = None
        workflow_id = None
        stages = []
        deadline = time.monotonic() + 210
        next_jt = 0.0
        while time.monotonic() < deadline:
            if sender_error:
                raise RuntimeError("JT128 UDP 模拟发送失败: " + sender_error[0])
            if (root / "stop.txt").exists():
                break
            now = time.monotonic()
            if now >= next_jt:
                status, state = call(urls["JT128"] + "/api/state", timeout=10)
                next_jt = now + 2.0
                if status == 200 and isinstance(state, dict):
                    lane = next((row for row in state.get("channels", [])
                                 if row.get("id") == "CH4"), {})
                    current = lane.get("current_result") or {}
                    measurement_id = measurement_id or current.get("measurement_id")
                    if measurement_id:
                        report.setdefault("jt", {}).update({
                            "measurementId": measurement_id,
                            "plate": (lane.get("last_result") or {}).get("plate"),
                            "savedCount": lane.get("saved_count"),
                            "source": lane.get("source_kind"),
                        })
            # CAM_004 同时用于两条雷达链；先确认 JT128 测量和车牌绑定，
            # 再为同一摄像头点位建立双雷达来车流程。
            if measurement_id and not workflow_id:
                receipt_status, receipt = call(urls["WebApi"] +
                    "/api/v1/load-radar-agent/measurements/" + measurement_id,
                    headers=agent_headers)
                if receipt_status == 200:
                    plate_value = (receipt.get("data") or {}).get("binding", {}).get("licensePlate")
                    if plate_value != "宁A85Q4P":
                        raise RuntimeError("JT128 测量缺少本次车牌绑定")
                    report["jt"]["bindingPlate"] = plate_value
                    if args.auto_capture:
                        import re
                        recorder_log = root / "Recorder-console.log"
                        content = (recorder_log.read_text(encoding="utf-8")
                                   if recorder_log.exists() else "")
                        match = re.search(r"sess_[0-9]+_[0-9a-f]+", content)
                        workflow_id = match.group(0) if match else None
                        if workflow_id:
                            report["flow"].append({"node": "automatic_capture",
                                                   "workflowId": workflow_id})
                    else:
                        capture_status, capture = call(urls["WebApi"] +
                            "/api/v1/radar-live/captures", "POST",
                            {"licensePlate": plate_value, "confirmOverwriteEmptyProfile": True,
                             "idempotencyKey": "codex_" + secrets.token_hex(12),
                             "note": "模拟雷达联合流程"}, headers=capture_headers)
                        workflow_id = (capture.get("data") or {}).get("workflowId")
                        report["flow"].append({"node": "manual_capture", "http": capture_status,
                                               "workflowId": workflow_id,
                                               "status": (capture.get("data") or {}).get("status")})
                        if capture_status != 200 or not workflow_id:
                            raise RuntimeError("WebApi 未受理双雷达模拟采集")
            if workflow_id:
                status, progress = call(urls["WebApi"] + "/api/v1/radar-live/captures/" +
                                        workflow_id, headers=capture_headers)
                payload = (progress.get("data") or {}) if isinstance(progress, dict) else {}
                stage = payload.get("stage")
                row = {"http": status, "stage": stage, "message": payload.get("message")}
                if not stages or stages[-1] != row:
                    stages.append(row)
                    print("进度", row, flush=True)
                if stage in ("completed", "completed_without_dimensions", "failed",
                             "analysis_failed", "capture_failed", "persistence_failed"):
                    break
            time.sleep(2)
        report["workflowId"] = workflow_id
        report["progress"] = stages
        if args.auto_capture:
            recorder_log = root / "Recorder-console.log"
            report["recorderConsole"] = (recorder_log.read_text(encoding="utf-8")
                                         if recorder_log.exists() else "")[-6000:]
        pcds = list((root / "data" / "CaptureDirectory").rglob("*.pcd"))
        report["pcdCount"] = len(pcds)
        report["originalPcdCount"] = sum(path.name.startswith(("CAM_004_", "CAM_005_"))
                                         for path in pcds)
        if measurement_id:
            history_status, history = 404, {}
            for _ in range(20):
                history_status, history = call(urls["WebApi"] +
                                               "/api/v1/load-radar-agent/measurements/" + measurement_id,
                                               headers=agent_headers)
                if history_status == 200:
                    break
                time.sleep(2)
            data = (history.get("data") or {}) if isinstance(history, dict) else {}
            report["flow"].append({"node": "jt_measurement_in_webapi", "http": history_status,
                                   "measurementId": (data.get("measurement") or {}).get("measurementId"),
                                   "channelId": (data.get("measurement") or {}).get("channelId"),
                                   "verdict": (data.get("measurement") or {}).get("verdict"),
                                   "volumeStatus": (data.get("measurement") or {}).get("volumeStatus"),
                                   "measuredAt": (data.get("measurement") or {}).get("measuredAt")})
        report["passed"] = bool(measurement_id and workflow_id and
                                report.get("jt", {}).get("bindingPlate") == "宁A85Q4P" and
                                stages and stages[-1]["stage"] in ("completed", "completed_without_dimensions")
                                and any(row.get("node") == ("automatic_capture" if args.auto_capture
                                                            else "manual_capture") and
                                        (args.auto_capture or row.get("http") == 200)
                                        for row in report["flow"])
                                and any(row.get("node") == "jt_measurement_in_webapi" and
                                        row.get("http") == 200 and row.get("measurementId") == measurement_id
                                        for row in report["flow"])
                                and report["originalPcdCount"] == 2)
        save(root / "report.json", report)
        print("报告", root / "report.json", flush=True)
    except Exception as error:
        report["error"] = str(error)
        save(root / "report.json", report)
        raise
    finally:
        sender_stop.set()
        if sender is not None:
            sender.join(timeout=3)
        if plate_stub is not None:
            plate_stub.shutdown()
            plate_stub.server_close()
        for name, process, stream in reversed(registry):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(5)
            if stream is not None:
                stream.close()
        if web_stopped:
            wait_port_free(5031)
            original = REPO / "CarPointCloudWebApi"
            restored_log = (root / "WebApi-restored.log").open("w", encoding="utf-8")
            subprocess.Popen(["dotnet", str(original / "bin" / "Release" / "net8.0" /
                                              "CarPointCloudWebApi.dll")],
                             cwd=original, env=env, stdout=restored_log,
                             stderr=subprocess.STDOUT, creationflags=flags)
            restored_log.close()
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                try:
                    if call("http://127.0.0.1:5031/healthz")[0] == 200:
                        print("原 5031 WebApi 已恢复", flush=True)
                        break
                except OSError:
                    pass
                time.sleep(0.4)
            else:
                print("原 5031 WebApi 恢复健康检查失败，请查看恢复日志", flush=True)
        # 联调证据保留；复制到临时目录的数据库密码和节点密钥立即删除。
        for relative in ("Analyzer/appsettings.json",
                         "WebApi/appsettings.json", "JT128/webapi.json"):
            path = (root / relative).resolve()
            if path.is_relative_to(root) and path.is_file():
                path.unlink()
    return 0 if report.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
