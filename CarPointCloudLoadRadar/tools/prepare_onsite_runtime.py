#!/usr/bin/env python3
"""在仓库外准备明日现场用的 WebApi 与载货雷达私有配置。"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
REPO = PROJECT.parent
WEBAPI = REPO / "CarPointCloudWebApi"


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8-sig"))


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def prepare(output: Path, webapi_port: int = 5031) -> dict:
    output = output.resolve()
    if output.is_relative_to(REPO.resolve()):
        raise ValueError("私有运行目录必须放在仓库外，避免把密钥加入版本控制")
    if not 1 <= webapi_port <= 65535:
        raise ValueError("WebApi 端口无效")
    web = read_json(WEBAPI / "appsettings.json")
    fragment = read_json(WEBAPI / "Config" / "load-radar.json")
    bridge = read_json(PROJECT / "config" / "webapi.json")
    channels = read_json(PROJECT / "config" / "channels.json")
    radar = copy.deepcopy(fragment["LoadRadar"])
    nodes = radar["Nodes"]
    if len(nodes) != 1 or nodes[0]["NodeId"] != bridge["node_id"]:
        raise ValueError("WebApi 与载货雷达节点编号不一致")
    if nodes[0]["AgentKey"] != bridge["agent_key"]:
        raise ValueError("WebApi 与载货雷达节点密钥不一致")
    mapped = {c["id"]: c["camera_identifier"] for c in channels["channels"]}
    declared = {c["ChannelId"]: c["CameraIdentifier"] for c in nodes[0]["Channels"]}
    if mapped != declared or mapped != {f"CH{i}": f"CAM_00{i}" for i in range(1, 5)}:
        raise ValueError("通道与车牌点位映射不一致")
    if channels["channels"][0]["enabled"] or not all(
            row["enabled"] for row in channels["channels"][1:]):
        raise ValueError("预期 CH1 停用、CH2/CH3/CH4 启用")
    if set(bridge["channels"]) != {"CH2", "CH3", "CH4"}:
        raise ValueError("上报配置的启用通道必须为 CH2/CH3/CH4")
    web["Kestrel"] = {"Endpoints": {"Http": {"Url": f"http://127.0.0.1:{webapi_port}"}}}
    radar["StorageRoot"] = str(output / "data" / "load-radar-assets")
    radar["Enabled"] = True
    nodes[0]["BaseUrl"] = "http://127.0.0.1:8080"
    web["LoadRadar"] = radar
    bridge["enabled"] = True
    bridge["base_url"] = f"http://127.0.0.1:{webapi_port}"
    bridge["storage_dir"] = str(output / "data" / "webapi-outbox")
    bridge["channels"] = {name: bridge["channels"][name] for name in ("CH2", "CH3", "CH4")}
    write_json(output / "WebApi" / "appsettings.json", web)
    write_json(output / "JT128" / "webapi.json", bridge)
    (output / "WebApi" / "logs").mkdir(exist_ok=True)
    result = {"webapiRuntimeDir": str(output / "WebApi"),
              "jt128WebapiConfig": str(output / "JT128" / "webapi.json"),
              "webapiUrl": f"http://127.0.0.1:{webapi_port}",
              "nodeId": bridge["node_id"], "enabledChannels": ["CH2", "CH3", "CH4"],
              "ch1Enabled": False, "keyMatches": True}
    write_json(output / "manifest.json", result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--webapi-port", type=int, default=5031)
    args = parser.parse_args()
    try:
        result = prepare(Path(args.output_dir), args.webapi_port)
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as error:
        print("准备失败：" + str(error))
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
