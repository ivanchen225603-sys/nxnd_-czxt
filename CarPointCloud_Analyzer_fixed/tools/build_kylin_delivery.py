"""组装三服务麒麟 x86_64 离线交付包；不复制现场配置或修改运行服务。"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import struct
import subprocess
import sys
import tarfile
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
VERSION = "20260910-measurement-fix"


def write(path: Path, content: str) -> None:
    """生成 LF/UTF-8 部署资产，不覆盖源码或现场配置。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content.replace('\r\n', '\n'), encoding='utf-8', newline='\n')


def write_json(path: Path, value: dict) -> None:
    write(path, json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def sha(path: Path) -> str:
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def check_elf(path: Path) -> None:
    """静态校验为 Linux ELF64 x86_64；不冒充目标机执行验证。"""
    with path.open('rb') as stream:
        header = stream.read(20)
    if header[:6] != b'\x7fELF\x02\x01' or struct.unpack('<H', header[18:20])[0] != 62:
        raise ValueError(f'Not ELF64 x86_64: {path.name}')


def copy_publish(source: Path, target: Path, executable: str) -> None:
    """只接收发布二进制和网页资源；绝不带走发布目录内的真实配置。"""
    check_elf(source / executable)
    for path in source.rglob('*'):
        if not path.is_file():
            continue
        relative = path.relative_to(source)
        if path.name.lower().startswith('appsettings'):
            continue
        if any(part in {'logs', 'tests', 'TestResults', '.uploads'} for part in relative.parts):
            continue
        if path.suffix.lower() in {'.pcd', '.pdb', '.log', '.pyc'}:
            continue
        allowed = relative.parts[0] == 'wwwroot' or (
            len(relative.parts) == 1 and (path.name == executable or
                                        path.suffix in {'.dll', '.so', '.json', '.dat', '.txt'})
        )
        if allowed:
            destination = target / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, destination)


PREFLIGHT = '''#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$(readlink -f -- "$0")")"
[[ "$(uname -m)" == x86_64 ]] || { echo 'ERROR: x86_64 required'; exit 1; }
version="$(getconf GNU_LIBC_VERSION)"
version="${version#glibc }"
[[ "$(printf '%s\\n' 2.31 "$version" | sort -V | head -n1)" == 2.31 ]] || {
  echo 'ERROR: glibc >= 2.31 required'; exit 1;
}
sha256sum --check SHA256SUMS
echo 'Archive/architecture checks passed. This does not test radar or database.'
'''

NATIVE = '''#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$(readlink -f -- "$0")")"
[[ "$(uname -m)" == x86_64 ]] || exit 1
command -v g++ >/dev/null || { echo 'Install g++ using the Kylin offline OS repository'; exit 1; }
g++ -std=gnu++14 -O2 -fPIC -shared -pthread -I native/vendor -I native \\
    native/ladar_bridge.cpp -L app -lladarsdk_gd -Wl,-rpath,'$ORIGIN' \\
    -o app/libladar_bridge.so
dependencies="$(ldd app/libladar_bridge.so 2>&1)" || {
    printf '%s\\n' "$dependencies"; exit 1;
}
printf '%s\\n' "$dependencies"
if grep -q 'not found' <<< "$dependencies"; then exit 1; fi
echo 'Native bridge compiled. Radar connection has NOT been tested or started.'
'''

PREPARE_ANALYZER = '''#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$(readlink -f -- "$0")")"
bash preflight.sh
[[ ! -e venv ]] || { echo 'venv exists; use a fresh extracted release directory'; exit 1; }
runtime/python/bin/python3 -m venv --copies venv
venv/bin/python -m pip install --no-index --find-links wheelhouse -r requirements-linux.lock
venv/bin/python -m pip check
venv/bin/python -c 'import numpy, scipy, open3d, fastapi, uvicorn, dmPython; print("Linux Python dependencies imported")'
echo 'Dependencies ready. Fill app/appsettings.json before starting.'
'''


def common(package: Path, service: str, run_body: str) -> None:
    write(package / 'preflight.sh', PREFLIGHT)
    write(package / 'run.sh', '#!/usr/bin/env bash\nset -euo pipefail\n'
          'cd -- "$(dirname -- "$(readlink -f -- "$0")")"\n'
          'root="$PWD"\ncd app\n' + run_body)
    write(package / (service + '.service.example'), f'''[Unit]
Description={service}
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
# Replace paths with the FINAL extracted directory; prepare Python there (venv is not relocatable).
User=carpointcloud
Group=carpointcloud-shared
UMask=0007
WorkingDirectory=/opt/{package.name}/app
ExecStart=/opt/{package.name}/run.sh
Restart=on-failure
RestartSec=5
TimeoutStopSec=60

[Install]
WantedBy=multi-user.target
''')


def worker_config() -> dict:
    # 仅从无凭据示例衍生；不读取用户 appsettings.json。
    source = ROOT / 'CarPointCloudLadarWorker/appsettings.Production.example.json'
    result = json.loads(source.read_text(encoding='utf-8-sig'))
    cfg = result['LadarCapture']
    for key, value in list(cfg.items()):
        if isinstance(value, str) and re.match(r'^[A-Za-z]:[\\/]', value):
            cfg[key] = '/data/carpointcloud/' + re.split(r'[\\/]', value)[-1]
    cfg.update({
        'EnableApiRequests': False,
        'EnablePythonAnalyzer': True, 'CollectPcdOnly': False, 'FetchLicensePlateOnCapture': False,
        'AppSecret': '', 'EnableSyntheticRadar': False, 'HttpApiHost': '127.0.0.1',
        'CaptureDirectory': '/data/carpointcloud/captures',
        'PermanentPointCloudDirectories': ['/data/carpointcloud/empty-vehicle-profiles'],
        'HealthSnapshotDirectory': '/data/carpointcloud/worker-state/health',
        'AnalysisTaskDirectory': '/data/carpointcloud/worker-state/analysis-tasks',
        'AnalyzerBaselineDirectory': '/data/carpointcloud/baselines',
        'DualRadarExtrinsicPath': '/data/carpointcloud/extrinsics/dual-radar-extrinsic.json',
        'SyntheticFrameIntervalSeconds': 7.0,
        'MaximumFrameDeliveryDelayMilliseconds': None,
    })
    for device in cfg['Devices']:
        device['Address'] = ''
    result['Serilog']['WriteTo'] = [{'Name': 'Console'}]
    return result


def analyzer_config() -> dict:
    result = json.loads((ROOT / 'CarPointCloud_Analyzer_fixed/appsettings.example.json').read_text())
    result.update({
        'CARPOINT_SERVICE_ALLOWED_ROOT': '/data/carpointcloud',
        'CARPOINT_EXTRINSIC_PATH': '/data/carpointcloud/extrinsics/dual-radar-extrinsic.json',
        'CARPOINT_EMPTY_PROFILE_ROOT': '/data/carpointcloud/empty-vehicle-profiles',
        'CARPOINT_BASELINE_DIR': '/data/carpointcloud/baselines',
        'CARPOINT_DB_USER': '', 'CARPOINT_DB_PASSWORD': '',
    })
    return result


def web_config() -> dict:
    return {
        'Urls': 'http://127.0.0.1:5031', 'AllowedHosts': '*',
        'ConnectionStrings': {'CarPointCloudDb': ''},
        'Database': {'Enabled': True, 'InitializeOnStartup': False},
        'Auth': {'JwtSecret': '', 'TokenExpireMinutes': 480},
        'Integration': {'ExternalPartnerEnabled': False, 'AppSecret': '', 'EnableMockLicensePlate': False},
        'Radar': {'BaseUrl': 'http://127.0.0.1:8000', 'AllowedDeviceAddresses': []},
        'Analyzer': {'BaseUrl': 'http://127.0.0.1:8010'},
        'EmptyVehicleProfile': {'StorageRoot': '/data/carpointcloud/empty-vehicle-profiles'},
        'Cors': {'AllowedOrigins': []}, 'AuditLog': {'CaptureBodies': False},
        'Serilog': {'MinimumLevel': {'Default': 'Information'}, 'WriteTo': [{'Name': 'Console'}]},
    }


def archive(package: Path, output: Path) -> dict:
    files = sorted(path for path in package.rglob('*') if path.is_file())
    # 配置部署时必改，归档哈希仍保留用于部署前验证。
    write(package / 'SHA256SUMS', ''.join(f'{sha(p)}  {p.relative_to(package).as_posix()}\n' for p in files))
    destination = output / (package.name + '.tar.gz')
    if destination.exists():
        raise FileExistsError(destination)

    def modes(info: tarfile.TarInfo) -> tarfile.TarInfo:
        info.uid = info.gid = 0
        info.uname = info.gname = ''
        info.mode = 0o755 if info.isdir() or info.name.endswith('.sh') or (
            '/runtime/python/bin/' in info.name or info.name.endswith('/CarPointCloudLadarWorker')
            or info.name.endswith('/CarPointCloudWebApi')) else 0o644
        if info.name.endswith('/appsettings.json'):
            info.mode = 0o640
        return info

    with tarfile.open(destination, 'w:gz', compresslevel=3) as tar:
        tar.add(package, arcname=package.name, filter=modes)
    return {'file': destination.name, 'bytes': destination.stat().st_size, 'sha256': sha(destination)}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--analyzer-assets', type=Path, required=True)
    parser.add_argument('--sdk-root', type=Path, required=True)
    args = parser.parse_args()
    output = args.output_root.resolve()
    stage = output / 'packages'
    stage.mkdir(exist_ok=False)
    assets = args.analyzer_assets.resolve()
    sdk = args.sdk_root.resolve()
    check_elf(assets / 'runtime/python/bin/python3')
    check_elf(sdk / 'linux/libladarsdk_gd.so')
    worker = stage / f'CarPointCloudLadarWorker-kylin-x86_64-{VERSION}'
    web = stage / f'CarPointCloudWebApi-kylin-x86_64-{VERSION}'
    analyzer = stage / f'CarPointCloud-Analyzer-kylin-x86_64-{VERSION}'
    copy_publish(output / 'build/worker', worker / 'app', 'CarPointCloudLadarWorker')
    copy_publish(output / 'build/webapi', web / 'app', 'CarPointCloudWebApi')
    for file in ('ladar_bridge.cpp', 'ladar_bridge.h'):
        (worker / 'native').mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / 'CarPointCloudLadarWorker/native/linux' / file, worker / 'native' / file)
    (worker / 'native/vendor').mkdir()
    shutil.copy2(sdk / 'include/ladar_api.h', worker / 'native/vendor/ladar_api.h')
    shutil.copy2(sdk / 'linux/libladarsdk_gd.so', worker / 'app/libladarsdk_gd.so')
    write(worker / 'prepare-native.sh', NATIVE)
    write_json(worker / 'app/appsettings.json', worker_config())
    write_json(web / 'app/appsettings.json', web_config())
    common(worker, 'car-pointcloud-ladar-worker',
           '[[ -f libladar_bridge.so ]] || { echo "Run prepare-native.sh first"; exit 1; }\n'
           'export LD_LIBRARY_PATH="$PWD${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"\n'
           'exec ./CarPointCloudLadarWorker "$@"\n')
    common(web, 'carpointcloudwebapi', 'exec ./CarPointCloudWebApi "$@"\n')
    shutil.copytree(ROOT / 'CarPointCloud_Analyzer_fixed/carpointcloud_analyzer',
                    analyzer / 'app/carpointcloud_analyzer',
                    ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    shutil.copytree(assets / 'runtime', analyzer / 'runtime')
    shutil.copytree(assets / 'wheelhouse', analyzer / 'wheelhouse')
    for path in (output / 'dm-wheels').glob('*.whl'):
        shutil.copy2(path, analyzer / 'wheelhouse' / path.name)
    if not list((analyzer / 'wheelhouse').glob('dmpython-2.5.38-*.whl')):
        raise ValueError('Missing Linux dmPython 2.5.38 wheel')
    requirements = []
    for path in sorted((analyzer / 'wheelhouse').glob('*.whl')):
        name, version = path.name.split('-')[:2]
        requirements.append(f'{name}=={version}')
    write(analyzer / 'requirements-linux.lock', '\n'.join(requirements) + '\n')
    # 解析所有已锁定传递依赖；必须从随包wheelhouse完全离线解析成功。
    subprocess.run([sys.executable, '-m', 'pip', 'download', '--no-index',
                    '--find-links', str(analyzer / 'wheelhouse'), '--dest', str(output / 'dependency-check'),
                    '--only-binary=:all:', '--platform', 'manylinux_2_31_x86_64',
                    '--platform', 'manylinux2014_x86_64', '--platform', 'manylinux2010_x86_64',
                    '--python-version', '311', '--implementation', 'cp', '--abi', 'cp311',
                    '-r', str(analyzer / 'requirements-linux.lock')], check=True)
    write_json(analyzer / 'app/appsettings.json', analyzer_config())
    write(analyzer / 'prepare-python.sh', PREPARE_ANALYZER)
    common(analyzer, 'carpointcloud-analyzer',
           '[[ -x "$root/venv/bin/python" ]] || { echo "Run prepare-python.sh first"; exit 1; }\n'
           'exec "$root/venv/bin/python" -m carpointcloud_analyzer.service "$@"\n')
    readme = (Path(__file__).parent / 'kylin_delivery_README.md').read_text(encoding='utf-8')
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    results = []
    for package in (worker, web, analyzer):
        write(package / 'README.md', readme)
        write_json(package / 'BUILD.json', {
            'target': 'Kylin x86_64 glibc>=2.31', 'version': VERSION, 'gitBaseCommit': commit,
            'source': 'current working tree including uncommitted fixes',
            'builtAtUtc': datetime.now(timezone.utc).isoformat(),
            'linuxExecutionVerified': False, 'realRadarVerified': False, 'realDatabaseVerified': False,
            'nativeBridge': 'compile on target with included source/SDK' if package == worker else None,
        })
        results.append(archive(package, output))
    write_json(output / 'DELIVERY.json', {'packages': results})
    write(output / 'SHA256SUMS', ''.join(f"{p['sha256']}  {p['file']}\n" for p in results))
    write(output / 'README.md', readme)
    print(json.dumps(results, indent=2), flush=True)


if __name__ == '__main__':
    main()
