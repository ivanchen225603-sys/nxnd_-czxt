"""逐字节验证四项目交付，不启动现场服务或连接数据库。"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
import tarfile

from tools.build_four_project_release import PROJECTS, ROOT, sha


def current_secrets() -> list[bytes]:
    """仅在内存用于防泄漏比对，不返回、打印或写入秘密值。"""
    values = set()
    def collect(value, path=""):
        if isinstance(value, dict):
            for key, item in value.items():
                collect(item, path + "." + key)
        elif isinstance(value, list):
            for item in value:
                collect(item, path)
        elif isinstance(value, str) and len(value) >= 12:
            normalized = re.sub(r"[^a-z0-9]", "", path.lower())
            if any(key in normalized for key in ("secret", "password", "agentkey", "token", "connectionstring")):
                values.add(value.encode("utf-8"))
    for file in ("CarPointCloudWebApi/appsettings.json", "CarPointCloudLadarWorker/appsettings.json",
                 "CarPointCloudWebApi/config/load-radar.json", "CarPointCloudLoadRadar/config/webapi.json",
                 "CarPointCloudLoadRadar/config/dual-radar-recorder.json"):
        path = ROOT / file
        if path.exists():
            collect(json.loads(path.read_text(encoding="utf-8-sig")))
    return list(values)


def verify(output: Path) -> dict:
    delivery = json.loads((output / "DELIVERY.json").read_text(encoding="utf-8"))
    assert len(delivery["packages"]) == 4
    assert {next(p for p in PROJECTS if entry["file"].startswith(p + "-")) for entry in delivery["packages"]} == set(PROJECTS)
    secrets = current_secrets()
    snapshot = output / "source-snapshot"
    manifest = json.loads((output / "SOURCE-MANIFEST.json").read_text(encoding="utf-8"))
    for row in manifest["files"]:
        assert sha(ROOT / row["path"]) == row["sha256"], "工作区在冻结后改变：" + row["path"]
    source_manifest_hash = sha(output / "SOURCE-MANIFEST.json")
    checks = []
    for entry in delivery["packages"]:
        path = output / entry["file"]
        assert sha(path) == entry["sha256"], "归档总哈希错误：" + path.name
        prefix = path.name.removesuffix(".tar.gz")
        actual = {}
        scripts = matched = 0
        with tarfile.open(path) as bundle:
            for member in bundle.getmembers():
                posix = PurePosixPath(member.name)
                assert not posix.is_absolute() and ".." not in posix.parts and posix.parts[0] == prefix
                assert member.isdir() or member.isfile(), "不允许设备、符号链接或硬链接：" + member.name
                if member.isdir():
                    continue
                relative = PurePosixPath(*posix.parts[1:])
                assert relative.as_posix() not in actual, "重复归档成员"
                assert not any(part in {"AGENTS.md", "CLAUDE.md", ".ai-lifecycle", "logs", "TestResults", "__pycache__"} for part in relative.parts)
                assert relative.suffix.lower() not in {".pcd", ".pdb", ".log", ".out", ".pyc"}
                if relative.suffix == ".xyz":
                    assert relative.parts[:2] == ("app", "baseline")
                content = bundle.extractfile(member).read()
                assert not any(secret in content for secret in secrets), "检测到现场秘密：" + member.name
                actual[relative.as_posix()] = hashlib.sha256(content).hexdigest()
                if relative.name == "appsettings.json":
                    raise AssertionError("包内不允许现场 appsettings.json：" + member.name)
                # 只对本工具生成的包根启动脚本要求 Bash；Python 自带脚本可以是 POSIX sh。
                if relative.suffix == ".sh" and len(relative.parts) == 1:
                    assert b"\r" not in content and content.startswith(b"#!/usr/bin/env bash\n"), member.name
                    assert member.mode & 0o111 == 0o111
                    subprocess.run(["wsl.exe", "-d", "Ubuntu", "--exec", "bash", "-n"], input=content, check=True, capture_output=True)
                    scripts += 1
                if content.startswith(b"\x7fELF"):
                    assert member.mode & 0o111 == 0o111
                source = None
                if prefix.startswith(PROJECTS[2]) and relative.parts[:2] == ("app", "carpointcloud_analyzer"):
                    source = snapshot / PROJECTS[2] / str(relative.relative_to("app"))
                elif prefix.startswith(PROJECTS[3]) and relative.parts[0] == "app" and relative.parts[1] in {"app", "cli", "tools", "baseline"}:
                    source = snapshot / PROJECTS[3] / str(relative.relative_to("app"))
                elif prefix.startswith(PROJECTS[1]) and relative.parts[:2] == ("app", "wwwroot"):
                    source = snapshot / PROJECTS[1] / str(relative.relative_to("app"))
                elif prefix.startswith(PROJECTS[0]) and relative.as_posix() in {"native/ladar_bridge.cpp", "native/ladar_bridge.h"}:
                    source = snapshot / PROJECTS[0] / "native/linux" / relative.name
                if source is not None:
                    assert sha(source) == actual[relative.as_posix()], "归档源码不等于冻结版本：" + member.name
                    matched += 1
            expected = dict(line.split("  ", 1)[::-1] for line in bundle.extractfile(prefix + "/SHA256SUMS").read().decode().splitlines())
            assert expected == {k: v for k, v in actual.items() if k != "SHA256SUMS"}
            build = json.load(bundle.extractfile(prefix + "/BUILD.json"))
            assert build["sourceManifestSha256"] == source_manifest_hash
            if prefix.startswith(PROJECTS[0]):
                assert "compat-runtime/lib64/ld-linux-x86-64.so.2" in actual
                assert "app/libladar_bridge.so" in actual and "app/libladarsdk_gd.so" in actual
                assert actual["app/CarPointCloudLadarWorker.dll"] == sha(output / "build" / PROJECTS[0] / "CarPointCloudLadarWorker.dll")
                cfg = json.load(bundle.extractfile(prefix + "/config-templates/appsettings.example.json"))["LadarCapture"]
                assert cfg["CaptureCleanupDailyAt"] == "03:00"
                assert cfg["PermanentPointCloudDirectories"] == ["/data/carpointcloud/empty-vehicle-profiles"]
                assert cfg["CaptureDirectory"] == "/data/carpointcloud/captures"
            if prefix.startswith(PROJECTS[1]):
                assert actual["app/CarPointCloudWebApi.dll"] == sha(output / "build" / PROJECTS[1] / "CarPointCloudWebApi.dll")
                assert "app/wwwroot/pointcloud-viewer/vendor/three-r128/three.min.js" in actual
                assert "app/wwwroot/pointcloud-viewer/volume-query.html" in actual
                assert "app/wwwroot/pointcloud-viewer/js/volume-query.js" in actual
                assert "database/load_radar_volume_revision_20261102.sql" in actual
            if prefix.startswith(PROJECTS[3]):
                cfg = json.load(bundle.extractfile(prefix + "/config-templates/channels.example.json"))
                assert cfg["service"]["daily_cleanup_at"] == "03:00"
                assert "app/app/recalculation.py" in actual and "app/app/storage_maintenance.py" in actual
                for lane in cfg["channels"]:
                    assert lane["baseline_dir"].startswith("/data/carpointcloud/load-radar/baselines/")
                    assert lane["output_dir"].startswith("/data/carpointcloud/load-radar/field/")
                baseline = snapshot / PROJECTS[3] / "baseline"
                expected_baseline = {"app/baseline/" + p.relative_to(baseline).as_posix()
                                     for p in baseline.rglob("*") if p.is_file()}
                assert {name for name in actual if name.startswith("app/baseline/")} == expected_baseline
                assert sum(name.endswith(".xyz") for name in expected_baseline) == 10
        checks.append({"file": path.name, "files": len(actual), "currentSourceFilesMatched": matched,
                       "scriptsSyntaxChecked": scripts, "sha256": entry["sha256"]})
    return {"packageIntegrityPassed": True, "sourceMatchesFrozenWorkingTree": True,
            "currentSecretsScanPassed": True, "productionDeploymentVerified": False, "packages": checks}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    args = parser.parse_args()
    result = verify(args.output_root.resolve())
    (args.output_root / "PACKAGE-VERIFICATION.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
