"""冻结当前四项目源码并组装离线包；不启动服务、不连接数据库、不复制现场秘密。"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import struct
import subprocess
import tarfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROJECTS = ("CarPointCloudLadarWorker", "CarPointCloudWebApi",
            "CarPointCloud_Analyzer_fixed", "CarPointCloudLoadRadar")
PRIVATE = {
    "CarPointCloudLadarWorker/appsettings.json", "CarPointCloudWebApi/appsettings.json",
    "CarPointCloudWebApi/appsettings.Development.json", "CarPointCloud_Analyzer_fixed/appsettings.json",
    "CarPointCloudWebApi/config/load-radar.json", "CarPointCloudLoadRadar/config/webapi.json",
    "CarPointCloudLoadRadar/config/volume-provider.json", "CarPointCloudLoadRadar/config/dual-radar-recorder.json",
    "CarPointCloudLoadRadar/config/channels.json",
}
EXCLUDED_PARTS = {"bin", "obj", "__pycache__", ".venv", "data", "logs", "TestResults",
                  ".ai-lifecycle", "AGENTS.md", "CLAUDE.md", "artifacts", "desktop", "re"}


def sha(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content.replace("\r\n", "\n"), encoding="utf-8", newline="\n")


def json_write(path: Path, value: object) -> None:
    write(path, json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def sanitize_template(value: object) -> object:
    """模板不携带有效秘密或 Windows 绝对路径；不用于覆盖现场配置。"""
    if isinstance(value, dict):
        result = {}
        for key, item in value.items():
            normalized = re.sub(r"[^a-z0-9]", "", key.lower())
            sensitive = any(term in normalized for term in
                            ("password", "secret", "agentkey", "token", "apikey", "authorization", "credential", "connectionstring"))
            result[key] = "" if sensitive and isinstance(item, str) else sanitize_template(item)
        return result
    if isinstance(value, list):
        return [sanitize_template(item) for item in value]
    if isinstance(value, str) and re.match(r"^[A-Za-z]:[\\/]", value):
        return "/data/carpointcloud/REVIEW-RETAIN-SITE-PATH"
    return value


def source_allowed(name: str, tracked: bool) -> bool:
    path = Path(name)
    if name in PRIVATE or any(part in EXCLUDED_PARTS for part in path.parts):
        return False
    if path.parts[0] not in PROJECTS and path.parts[0] not in {"tools", "scripts"}:
        return False
    if path.suffix.lower() in {".pcd", ".pdb", ".log", ".out", ".dll", ".exe", ".so", ".whl", ".zip", ".gz"}:
        return False
    # 用户新加的现场报告不混入冻结源码；新产品分文件、测试和本工具必须进入。
    return (tracked or path.suffix in {".cs", ".py", ".csproj"}
            or path.parts[0] == PROJECTS[1] and len(path.parts) > 2 and
                (path.parts[1] == "wwwroot" and path.suffix in {".html", ".js", ".css", ".svg", ".json"}
                 or path.parts[1] == "Sql" and path.suffix == ".sql"
                 or path.parts[1:4] == ("tests", "CarPointCloudWebApi.Tests", "Fixtures") and path.suffix == ".json"))


def freeze(output: Path) -> dict:
    destination = output / "source-snapshot"
    destination.mkdir(parents=True, exist_ok=False)
    def names(*args: str) -> list[str]:
        return subprocess.check_output(["git", "ls-files", "-z", *args], cwd=ROOT).decode("utf-8").strip("\0").split("\0")
    tracked = set(names())
    candidates = tracked | set(names("--others", "--exclude-standard"))
    files = []
    for name in sorted(candidates):
        if not name or not source_allowed(name, name in tracked):
            continue
        source = ROOT / name
        target = destination / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        files.append({"path": name, "sha256": sha(target), "tracked": name in tracked})
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    branch = subprocess.check_output(["git", "branch", "--show-current"], cwd=ROOT, text=True).strip()
    manifest = {"baseCommit": head, "branch": branch, "usesWorkingTree": True,
                "privateConfigurationsExcluded": sorted(PRIVATE), "files": files}
    channels = sanitize_template(json.loads((ROOT / PROJECTS[3] / "config/channels.json").read_text(encoding="utf-8-sig")))
    channels.pop("_说明", None)
    channels["service"].update(host="127.0.0.1", data_root="/data/carpointcloud/load-radar",
                               log_dir="/data/carpointcloud/load-radar/logs", daily_cleanup_at="03:00")
    for channel in channels["channels"]:
        channel.update(enabled=False, mode="live", output_dir=f"/data/carpointcloud/load-radar/field/{channel['id']}",
                       baseline_dir=f"/data/carpointcloud/load-radar/baselines/{channel['id']}", replay_dir="")
        channel["lidar"]["ip"] = ""
        channel["lidar"]["source_ip"] = ""
    template = destination / PROJECTS[3] / "config/channels.example.json"
    json_write(template, channels)
    manifest["generatedTemplates"] = [{"path": template.relative_to(destination).as_posix(), "sha256": sha(template)}]
    json_write(output / "SOURCE-MANIFEST.json", manifest)
    return manifest


def check_elf(path: Path) -> None:
    with path.open("rb") as stream:
        header = stream.read(20)
    if len(header) < 20 or header[:6] != b"\x7fELF\x02\x01" or struct.unpack("<H", header[18:20])[0] != 62:
        raise ValueError(f"不是 Linux x86_64 ELF：{path}")


def prepare_compat_mountpoints(runtime: Path) -> None:
    """只读根中预建挂载点；bwrap 无法在挂载之后创建缺少的目录或文件。"""
    for name in ("dev", "proc", "sys", "tmp", "app", "data/carpointcloud", "usr/share/zoneinfo", "etc"):
        (runtime / name).mkdir(parents=True, exist_ok=True)
    for name in ("passwd", "group", "localtime", "timezone", "resolv.conf"):
        path = runtime / "etc" / name
        if not path.exists():
            path.touch()


def copy_publish(source: Path, target: Path, executable: str) -> None:
    check_elf(source / executable)
    for path in sorted(source.rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(source)
        if path.name.lower().startswith("appsettings") or any(p in EXCLUDED_PARTS for p in relative.parts):
            continue
        if path.suffix.lower() in {".pdb", ".pcd", ".log", ".config"}:
            continue
        if relative.parts[0] == "wwwroot" or (len(relative.parts) == 1 and
                (path.name == executable or path.suffix.lower() in {".dll", ".so", ".json", ".dat", ".txt"})):
            target_file = target / relative
            target_file.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target_file)


PREFLIGHT = '''#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$(readlink -f -- "$0")")"
[[ "$(uname -m)" == x86_64 ]] || { echo '需要 x86_64'; exit 1; }
[[ -r /usr/share/zoneinfo/Asia/Shanghai ]] || { echo '系统缺少 tzdata/Asia/Shanghai，业务时区不可用'; exit 1; }
sha256sum --check SHA256SUMS
echo '文件完整性通过；未验证设备、数据库、现场配置。'
'''
RUN_HEADER = '''#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$(readlink -f -- "$0")")"
root="$PWD"
[[ -f CONFIGURATION_CONFIRMED ]] || { echo '先承接并核对现场配置，再建立 CONFIGURATION_CONFIRMED 文件。'; exit 78; }
'''
PREPARE_PYTHON = '''#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$(readlink -f -- "$0")")"
bash preflight.sh
[[ ! -e venv ]] || { echo 'venv 已存在；请在最终新版本目录准备，不搬动 venv。'; exit 1; }
runtime/python/bin/python3 -m venv --copies venv
venv/bin/python -m pip install --no-index --find-links wheelhouse -r requirements-linux.lock
venv/bin/python -m pip check
'''
COMPAT_RUN = RUN_HEADER + '''[[ -f app/appsettings.json ]] || exit 78
runtime="$root/compat-runtime"
[[ -x "$runtime/lib64/ld-linux-x86-64.so.2" ]] || { echo '缺少配套兼容运行库'; exit 1; }
command -v bwrap >/dev/null || { echo '缺少 bwrap，不能回退到宿主直接启动'; exit 1; }
[[ -d /data/carpointcloud && -d app/logs ]] || { echo '先核对共享数据和日志目录权限'; exit 78; }
exec bwrap --die-with-parent --new-session --ro-bind "$runtime" / \\
  --dev /dev --proc /proc --ro-bind /sys /sys --tmpfs /tmp --ro-bind "$root/app" /app \\
  --ro-bind-try /etc/localtime /etc/localtime --ro-bind-try /etc/timezone /etc/timezone \\
  --ro-bind-try /etc/resolv.conf /etc/resolv.conf \\
  --ro-bind-try /usr/share/zoneinfo /usr/share/zoneinfo \\
  --ro-bind-try /etc/passwd /etc/passwd --ro-bind-try /etc/group /etc/group \\
  --bind /data/carpointcloud /data/carpointcloud --bind "$root/app/logs" /app/logs \\
  --chdir /app /app/CarPointCloudLadarWorker "$@"
'''


def common(package: Path, run: str) -> None:
    write(package / "preflight.sh", PREFLIGHT)
    write(package / "run.sh", RUN_HEADER + run)
    write(package / "CONFIGURATION_REQUIRED.txt",
          "不要覆盖服务器现有配置/外参/队列/基线。模板不含可用秘密；按配置差异承接，核对后建立 CONFIGURATION_CONFIRMED。\n")


def copy_tree(source: Path, target: Path) -> None:
    shutil.copytree(source, target, ignore=shutil.ignore_patterns("__pycache__", "*.pyc", ".pytest_cache"))


def templates(snapshot: Path, package: Path, project: str) -> None:
    # 仅复制无秘密示例，运行入口要求现场文件另行承接。
    if project == PROJECTS[0]:
        cfg = json.loads((snapshot / project / "appsettings.Production.example.json").read_text(encoding="utf-8-sig"))
        opts = cfg["LadarCapture"]
        opts.update(EnableApiRequests=False, EnableResultUploadToWebApi=False, AppSecret="",
                    CollectPcdOnly=False, EnablePythonAnalyzer=True, EnableSyntheticRadar=False,
                    HttpApiHost="127.0.0.1", AllowNonLoopbackControlApi=False,
                    ContinuousFrameContractEvidence=None, MaximumFrameDeliveryDelayMilliseconds=None,
                    CoordinateScale=1, LivePreviewMaxPointsPerCamera=12000,
                    CaptureCleanupDailyAt="03:00",
                    PermanentPointCloudDirectories=["/data/carpointcloud/empty-vehicle-profiles"])
        for key in ("CaptureDirectory", "PairManifestDirectory", "AnalysisQueueDirectory",
                    "UploadQueueDirectory", "HealthSnapshotDirectory", "AnalyzerBaselineDirectory"):
            opts[key] = "/data/carpointcloud/" + {"CaptureDirectory": "captures", "AnalyzerBaselineDirectory": "baselines"}.get(key, "worker-state/" + key)
        opts["DualRadarExtrinsicPath"] = "/data/carpointcloud/extrinsics/RETAIN-VERIFIED-EXTRINSIC.json"
        for device in opts["Devices"]:
            device["Address"] = ""
        cfg["Serilog"]["WriteTo"] = [{"Name": "Console"}]
    elif project == PROJECTS[1]:
        cfg = json.loads((snapshot / project / "appsettings.example.json").read_text(encoding="utf-8-sig"))
        cfg["ConnectionStrings"]["CarPointCloudDb"] = ""
        cfg["Auth"]["JwtSecret"] = ""
        cfg["Integration"].update(ExternalQueriesEnabled=True, ExternalPartnerEnabled=False,
                                    PartnerBaseUrl="", AppSecret="", EnableMockLicensePlate=False)
        cfg["Database"].update(Enabled=True, InitializeOnStartup=False)
        cfg["AuditLog"].update(CaptureBodies=False, CapturePartnerQueryBodies=True)
        cfg["Radar"]["BaseUrl"] = "http://127.0.0.1:8000"
        cfg["EmptyVehicleProfile"]["StorageRoot"] = "/data/carpointcloud/empty-vehicle-profiles"
        cfg["Urls"] = "http://127.0.0.1:5031"
        cfg.pop("Kestrel", None)
        cfg["AllowedHosts"] = "localhost;127.0.0.1"
    else:
        cfg = json.loads((snapshot / project / "appsettings.example.json").read_text(encoding="utf-8-sig"))
        cfg.update(CARPOINT_SERVICE_ALLOWED_ROOT="/data/carpointcloud", CARPOINT_DB_USER="", CARPOINT_DB_PASSWORD="",
                   CARPOINT_EMPTY_PROFILE_ROOT="/data/carpointcloud/empty-vehicle-profiles",
                   CARPOINT_EXTRINSIC_PATH="/data/carpointcloud/extrinsics/RETAIN-VERIFIED-EXTRINSIC.json")
    json_write(package / "config-templates/appsettings.example.json", sanitize_template(cfg))


def archive(package: Path, output: Path) -> dict:
    path = output / (package.name + ".tar.gz")
    if path.exists():
        raise FileExistsError(path)
    files = sorted(p for p in package.rglob("*") if p.is_file() and p != package / "SHA256SUMS")
    write(package / "SHA256SUMS", "".join(f"{sha(p)}  {p.relative_to(package).as_posix()}\n" for p in files))
    def modes(info: tarfile.TarInfo) -> tarfile.TarInfo:
        info.uid = info.gid = 0
        info.uname = info.gname = ""
        executable = info.isdir() or info.name.endswith(".sh")
        if info.isfile():
            with (package.parent / info.name).open("rb") as stream:
                executable |= stream.read(4) == b"\x7fELF"
        info.mode = 0o755 if executable else 0o644
        return info
    with tarfile.open(path, "w:gz", compresslevel=1, dereference=True) as bundle:
        bundle.add(package, arcname=package.name, filter=modes)
    return {"file": path.name, "sha256": sha(path), "bytes": path.stat().st_size}


def assemble(output: Path, version: str, assets: Path, sdk: Path) -> dict:
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}", version):
        raise ValueError("版本号只允许安全的字母数字和 ._-，且不超过 64 字符")
    snapshot = output / "source-snapshot"
    source_manifest = json.loads((output / "SOURCE-MANIFEST.json").read_text(encoding="utf-8"))
    for row in source_manifest["files"] + source_manifest.get("generatedTemplates", []):
        if sha(snapshot / row["path"]) != row["sha256"]:
            raise ValueError("冻结源码已改变：" + row["path"])
    check_elf(output / "compat-runtime/lib64/ld-linux-x86-64.so.2")
    check_elf(output / "native-build/libladar_bridge.so")
    prepare_compat_mountpoints(output / "compat-runtime")
    if not (output / "RELEASE-GATES.json").is_file():
        raise ValueError("缺少本轮回归结果 RELEASE-GATES.json，不能生成不标明状态的交付包")
    stage = output / ("packages-" + version)
    stage.mkdir(exist_ok=False)
    packages = []
    for project in PROJECTS:
        package = stage / f"{project}-kylin-x86_64-{version}"
        package.mkdir()
        if project in PROJECTS[:2]:
            copy_publish(output / "build" / project, package / "app", project)
            templates(snapshot, package, project)
            if project == PROJECTS[0]:
                common(package, "echo '此站点必须使用经核对的 run-compatible.sh'; exit 78\n")
                write(package / "run-compatible.sh", COMPAT_RUN)
                for name in ("ladar_bridge.cpp", "ladar_bridge.h"):
                    target = package / "native" / name
                    target.parent.mkdir(exist_ok=True)
                    shutil.copy2(snapshot / project / "native/linux" / name, target)
                shutil.copy2(sdk / "linux/libladarsdk_gd.so", package / "app/libladarsdk_gd.so")
                shutil.copy2(output / "native-build/libladar_bridge.so", package / "app/libladar_bridge.so")
                (package / "native/vendor").mkdir()
                shutil.copy2(sdk / "include/ladar_api.h", package / "native/vendor/ladar_api.h")
                copy_tree(output / "compat-runtime", package / "compat-runtime")
            else:
                common(package, '[[ -f app/appsettings.json ]] || exit 78\ncd app\nexec ./CarPointCloudWebApi "$@"\n')
                cfg = package / "config-templates"
                shutil.copy2(snapshot / project / "config/load-radar.example.json", cfg / "load-radar.example.json")
                database = package / "database"
                database.mkdir()
                for name in ("load_radar_20260917.sql", "load_radar_passage_start_20260918.sql",
                             "load_radar_publication_20260918.sql", "partner_contract_v4_20261012.sql",
                             "async_plate_checkpoint_20261022.sql", "load_radar_volume_revision_20261102.sql"):
                    shutil.copy2(snapshot / project / "Sql" / name, database / name)
                write(database / "README.txt", "仅供人工核对增量顺序；先备份和核对已执行迁移，再执行缺失项。\n"
                      "本次CH4当天复算新增load_radar_volume_revision_20261102.sql；没有自动执行入口。\n")
        else:
            copy_tree(assets / "runtime", package / "runtime")
            (package / "wheelhouse").mkdir()
            wheel_files = sorted((assets / "wheelhouse").glob("*.whl"))
            if project == PROJECTS[3]:
                wheel_files = [p for p in wheel_files if p.name.startswith("numpy-")]
            if not wheel_files:
                raise ValueError("缺少 Linux wheel")
            pins = []
            for wheel in wheel_files:
                shutil.copy2(wheel, package / "wheelhouse" / wheel.name)
                name, pin = wheel.name.split("-")[:2]
                pins.append(f"{name}=={pin}")
            write(package / "requirements-linux.lock", "\n".join(pins) + "\n")
            write(package / "prepare-python.sh", PREPARE_PYTHON)
            if project == PROJECTS[2]:
                copy_tree(snapshot / project / "carpointcloud_analyzer", package / "app/carpointcloud_analyzer")
                templates(snapshot, package, project)
                common(package, '[[ -f app/appsettings.json ]] || exit 78\ncd app\nexec "$root/venv/bin/python" -m carpointcloud_analyzer.service "$@"\n')
            else:
                for name in ("app", "cli", "baseline", "tools"):
                    copy_tree(snapshot / project / name, package / "app" / name)
                (package / "app/config").mkdir(parents=True)
                shutil.copy2(snapshot / project / "config/angle_cache.csv", package / "app/config/angle_cache.csv")
                for config in (snapshot / project / "config").glob("*.json"):
                    # 保留运行参数作合并参考，实际 site 配置不自动安装。
                    target = package / "config-templates" / (config.stem + ".example.json" if ".example" not in config.stem else config.name)
                    target.parent.mkdir(exist_ok=True)
                    json_write(target, sanitize_template(json.loads(config.read_text(encoding="utf-8-sig"))))
                common(package, '[[ -f app/config/channels.json ]] || exit 78\ncd app\nexec "$root/venv/bin/python" cli/run.py "$@"\n')
        json_write(package / "BUILD.json", {"version": version, "project": project,
                   "sourceManifestSha256": sha(output / "SOURCE-MANIFEST.json"),
                   "target": "kylin-x86_64", "deploymentPerformed": False})
        shutil.copy2(output / "RELEASE-GATES.json", package / "RELEASE-GATES.json")
        packages.append(archive(package, output))
    result = {"version": version, "baseCommit": json.loads((output / "SOURCE-MANIFEST.json").read_text(encoding="utf-8"))["baseCommit"],
              "packages": packages, "deploymentPerformed": False, "databaseTouched": False}
    json_write(output / "DELIVERY.json", result)
    write(output / "SHA256SUMS", "".join(f"{r['sha256']}  {r['file']}\n" for r in packages))
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["freeze", "assemble"])
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--version", default="20261029-r1")
    parser.add_argument("--analyzer-assets", type=Path)
    parser.add_argument("--sdk-root", type=Path)
    args = parser.parse_args()
    output = args.output_root.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if args.action == "freeze":
        result = freeze(output)
        print(json.dumps({"files": len(result["files"]), "baseCommit": result["baseCommit"]}))
    else:
        if args.analyzer_assets is None or args.sdk_root is None:
            parser.error("assemble 必须指定离线资产和 SDK 来源")
        print(json.dumps(assemble(output, args.version, args.analyzer_assets.resolve(), args.sdk_root.resolve()), indent=2))


if __name__ == "__main__":
    main()
