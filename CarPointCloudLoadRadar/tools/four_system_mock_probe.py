#!/usr/bin/env python3
"""隔离启动四个实际入口，记录模拟链路证据；不连接真实雷达或数据库。"""
from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path
import secrets
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request


PROJECT = Path(__file__).resolve().parents[1]
REPO = PROJECT.parent
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


def request(url: str, method: str = "GET", body: object | None = None,
            headers: dict[str, str] | None = None):
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(url, data=data, method=method)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    for name, value in (headers or {}).items():
        req.add_header(name, value)
    try:
        with OPENER.open(req, timeout=8) as response:
            raw, status = response.read(), response.status
    except urllib.error.HTTPError as error:
        raw, status = error.read(), error.code
    try:
        result = json.loads(raw)
    except ValueError:
        result = raw.decode("utf-8", errors="replace")[:300]
    return status, result


def start(name: str, command: list[str], directory: Path, env: dict[str, str],
          health_url: str, processes: list, checks: list, log_dir: Path | None = None) -> None:
    stream = ((log_dir or directory) / "process.log").open("w", encoding="utf-8")
    process = subprocess.Popen(command, cwd=directory, env=env,
                               stdout=stream, stderr=subprocess.STDOUT)
    processes.append((name, process, stream))
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"{name} 提前退出：{process.returncode}")
        try:
            status, _ = request(health_url)
            if status == 200:
                checks.append({"program": name, "pid": process.pid, "health": 200})
                return
        except (OSError, TimeoutError):
            pass
        time.sleep(0.25)
    raise RuntimeError(f"{name} 健康检查超时")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--worker-dll", required=True)
    parser.add_argument("--webapi-dll", required=True)
    args = parser.parse_args()
    root = Path(args.output_dir).resolve()
    root.mkdir(parents=True, exist_ok=True)
    dirs = {name: root / name for name in ("Analyzer", "Worker", "WebApi", "JT128")}
    for directory in dirs.values():
        directory.mkdir(exist_ok=True)
    ports = {name: port() for name in dirs}
    if len(set(ports.values())) != 4:
        raise RuntimeError("端口分配冲突")
    urls = {name: f"http://127.0.0.1:{value}" for name, value in ports.items()}
    data = root / "data"
    data.mkdir(exist_ok=True)
    extrinsic = data / "extrinsic.json"
    write_json(extrinsic, {"frame": "B_to_A", "cameraA": "CAM_004", "cameraB": "CAM_005",
                           "matrix": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
                           "baselineResidual": 0.018,
                           "quality": {"fitness": 0.82, "inlierRmse": 0.018,
                                       "correspondenceCount": 42000, "degenerateAxes": [],
                                       "conditionNumber": 12.4, "trustworthy": True,
                                       "warnings": []}})
    write_json(dirs["Analyzer"] / "appsettings.json", {
        "CARPOINT_SERVICE_HOST": "127.0.0.1", "CARPOINT_SERVICE_PORT": ports["Analyzer"],
        "CARPOINT_SERVICE_ALLOWED_ROOT": str(data),
        "CARPOINT_SERVICE_BASELINE_DIR": str(data / "analyzer-baselines"),
        "CARPOINT_EXTRINSIC_PATH": str(extrinsic), "CARPOINT_DB_ENABLED": False,
        "CARPOINT_EMPTY_PROFILE_ENABLED": False})
    worker_project = REPO / "CarPointCloudLadarWorker"
    capture = copy.deepcopy(json.loads((worker_project / "appsettings.json").read_text(encoding="utf-8-sig"))["LadarCapture"])
    capture.update(EnableSyntheticRadar=True, EnableApiRequests=False,
                   EnableResultUploadToWebApi=False, EnablePythonAnalyzer=True,
                   FetchLicensePlateOnCapture=False, EnableLivePreview=True,
                   EnableApiTriggeredScanOnce=True, EnableHttpApi=True,
                   HttpApiHost="127.0.0.1", HttpApiPort=ports["Worker"],
                   AnalyzerServiceUrl=urls["Analyzer"], WebApiUrl=urls["WebApi"],
                   DualRadarExtrinsicPath=str(extrinsic),
                   AnalyzerBaselineDirectory=str(data / "analyzer-baselines"),
                   SyntheticFrameIntervalSeconds=4.04, SyntheticPointCount=6000)
    for key in ("CaptureDirectory", "PairManifestDirectory", "AnalysisQueueDirectory",
                "UploadQueueDirectory", "HealthSnapshotDirectory"):
        capture[key] = str(data / key)
    for index, device in enumerate(capture["Devices"]):
        device["Address"] = f"192.0.2.{index + 11}"
    capture["Address"] = "192.0.2.11"
    write_json(dirs["Worker"] / "appsettings.json", {"LadarCapture": capture})
    recorder_token = secrets.token_urlsafe(32)
    write_json(dirs["WebApi"] / "appsettings.json", {
        "RadarLive": {"RecorderToken": recorder_token},
        "Kestrel": {"Endpoints": {"Http": {"Url": urls["WebApi"]}}},
        "Database": {"Enabled": False, "InitializeOnStartup": False},
        "Integration": {"ExternalPartnerEnabled": False, "EnableMockLicensePlate": False,
                        "AppSecret": "mock-integration-secret-0123456789abcdef"},
        "Auth": {"JwtSecret": "mock-jwt-secret-0123456789abcdef"},
        "Radar": {"BaseUrl": urls["Worker"], "LicensePlateMockEnabled": True,
                  "LicensePlateMockRandom": False,
                  "AllowedDeviceAddresses": ["127.0.0.1", "::1"]},
        "AllowedHosts": "127.0.0.1;localhost"})
    channels = json.loads((PROJECT / "config" / "channels.json").read_text(encoding="utf-8-sig"))
    channels["service"]["host"] = "127.0.0.1"
    channels["service"]["port"] = ports["JT128"]
    channels["service"]["data_root"] = str(data / "jt-data")
    channels["service"]["log_dir"] = str(data / "jt-logs")
    for channel in channels["channels"]:
        channel["enabled"] = channel["id"] == "CH4"
        channel["output_dir"] = str(data / "jt-output" / channel["id"])
        channel["baseline_dir"] = str(data / "jt-baseline" / channel["id"])
        channel["replay_dir"] = str(data / "jt-replay" / channel["id"])
    channels["plate"]["base_url"] = urls["WebApi"]
    write_json(dirs["JT128"] / "channels.json", channels)
    env = {k: v for k, v in os.environ.items() if not k.upper().startswith(
        ("CARPOINT_", "LADARCAPTURE__", "CONNECTIONSTRINGS__", "AUTH__", "INTEGRATION__",
         "ASPNETCORE_", "DOTNET_", "DATABASE__", "RADAR__", "HTTP_PROXY", "HTTPS_PROXY"))}
    env.update(DOTNET_ENVIRONMENT="OfflineSmoke", ASPNETCORE_ENVIRONMENT="OfflineSmoke",
               PYTHONIOENCODING="utf-8", NO_PROXY="127.0.0.1,localhost")
    processes: list = []
    report: dict = {"ports": ports, "realRadar": False, "realDatabase": False,
                    "checks": [], "flow": []}
    try:
        analyzer = REPO / "CarPointCloud_Analyzer_fixed"
        analyzer_env = dict(env, PYTHONPATH=str(analyzer))
        start("Analyzer", [str(analyzer / ".venv" / "Scripts" / "python.exe"),
                           "-m", "carpointcloud_analyzer.service"],
              dirs["Analyzer"], analyzer_env, urls["Analyzer"] + "/health", processes, report["checks"])
        start("Worker", ["dotnet", str(Path(args.worker_dll).resolve())], dirs["Worker"], env,
              urls["Worker"] + "/api/health", processes, report["checks"])
        start("WebApi", ["dotnet", str(Path(args.webapi_dll).resolve()),
                         "--webroot", str(REPO / "CarPointCloudWebApi" / "wwwroot")],
              dirs["WebApi"], env, urls["WebApi"] + "/healthz", processes, report["checks"])
        start("JT128", [sys.executable, str(PROJECT / "cli" / "run.py"), "--demo", "--plate", "http",
                        "--save", "--channels", str(dirs["JT128"] / "channels.json"),
                        "--channel", "CH4", "--host", "127.0.0.1", "--port", str(ports["JT128"])],
              PROJECT, env, urls["JT128"] + "/api/health", processes, report["checks"],
              dirs["JT128"])
        for label, url in (("plate", urls["WebApi"] + "/api/v1/radar/licensePlate?cameraIdentifier=CAM_004"),
                           ("load_radar_page", urls["WebApi"] + "/pointcloud-viewer/load-radar.html"),
                           ("jt_state", urls["JT128"] + "/api/state"),
                           ("worker_status", urls["Worker"] + "/api/radars/status"),
                           ("webapi_capture_status", urls["WebApi"] + "/api/v1/radar-live/status")):
            try:
                headers = ({"X-Radar-Live-Token": recorder_token}
                           if label == "webapi_capture_status" else None)
                status, value = request(url, headers=headers)
                if label == "jt_state" and isinstance(value, dict):
                    lane = next((item for item in value.get("channels", []) if item.get("id") == "CH4"), {})
                    value = {"channel": lane.get("id"), "source_kind": lane.get("source_kind"),
                             "last_plate": lane.get("last_plate"), "current_result": lane.get("current_result"),
                             "stability": lane.get("stability"), "saved_count": lane.get("saved_count")}
                if label == "worker_status" and isinstance(value, dict):
                    value = {"success": value.get("success"), "scan": value.get("scan")}
                if label == "load_radar_page" and isinstance(value, str):
                    value = {"html": "载货雷达" in value}
                if label == "webapi_capture_status" and isinstance(value, dict):
                    value = {"code": value.get("code"), "message": value.get("message")}
                report["flow"].append({"node": label, "status": status, "response": value})
            except Exception as error:
                report["flow"].append({"node": label, "error": str(error)[:200]})
        # JT128 独立模式的完整模拟测量：从停稳、取牌到本地测量号。
        measured = None
        deadline = time.monotonic() + 35
        while time.monotonic() < deadline:
            status, snapshot = request(urls["JT128"] + "/api/state")
            if status == 200 and isinstance(snapshot, dict):
                lane = next((item for item in snapshot.get("channels", []) if item.get("id") == "CH4"), {})
                current = lane.get("current_result") or {}
                if isinstance(current, dict) and current.get("measurement_id"):
                    measured = {"measurement_id": current["measurement_id"],
                                "last_plate": lane.get("last_plate"),
                                "last_result_plate": (lane.get("last_result") or {}).get("plate"),
                                "saved_count": lane.get("saved_count"),
                                "stability": (lane.get("stability") or {}).get("state_cn")}
                    break
            time.sleep(2)
        report["flow"].append({"node": "jt_measurement", "result": measured})
        # 直接驱动隔离 Worker，以便独立验证合成双雷达 -> PCD -> Analyzer。
        # WebApi 的采集路由另行报告，绝不把本次直接请求冒充 WebApi 派发。
        workflow = "four-system-mock-001"
        accepted = None
        deadline = time.monotonic() + 35
        while time.monotonic() < deadline:
            status, value = request(urls["Worker"] + "/api/radars/scan-once?requestId=" + workflow,
                                    "POST")
            accepted = {"http": status, "body": value}
            if status in (200, 202):
                break
            time.sleep(4)
        report["flow"].append({"node": "worker_direct_scan", "result": accepted})
        if accepted and accepted["http"] in (200, 202):
            stages = []
            deadline = time.monotonic() + 70
            while time.monotonic() < deadline:
                status, value = request(urls["Worker"] + "/api/radars/analysis?workflowId=" + workflow)
                if isinstance(value, dict):
                    point = {"http": status, "stage": value.get("stage"),
                             "success": value.get("success"), "message": value.get("message"),
                             "error": value.get("error"), "analysis": value.get("analysis")}
                    if not stages or point != stages[-1]:
                        stages.append(point)
                    if value.get("stage") in ("completed", "failed") or value.get("permanentFailure"):
                        break
                time.sleep(2)
            report["flow"].append({"node": "worker_analyzer_progress", "stages": stages})
        report["flow"].append({"node": "worker_pcd_files",
                               "count": len(list((data / "CaptureDirectory").rglob("*.pcd")))})
        status, value = request(urls["WebApi"] + "/api/v1/radar-live/captures", "POST",
                                {"licensePlate": "宁A10333F", "idempotencyKey": "mock-capture-001"})
        report["flow"].append({"node": "webapi_capture_without_db", "status": status,
                               "response": value if isinstance(value, dict) else str(value)[:200]})
        write_json(root / "report.json", report)
        print(json.dumps({"report": str(root / "report.json"), "ports": ports,
                          "checks": len(report["checks"]),
                          "nodes": [item["node"] for item in report["flow"]]},
                         ensure_ascii=False), flush=True)
    finally:
        for _, process, _ in reversed(processes):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(5)
        for _, _, stream in processes:
            stream.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
