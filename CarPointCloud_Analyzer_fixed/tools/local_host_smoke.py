"""在一次性本地配置中启动真实的 .NET 入口，不接 DM8，也不接真实 SDK。"""

import argparse
import json
import os
from pathlib import Path
import secrets
import socket
import subprocess
import time
import urllib.error
import urllib.request


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    root = Path(args.output_dir).resolve()
    root.mkdir(parents=True, exist_ok=False)
    repo = Path(__file__).resolve().parents[2]
    worker = repo / 'CarPointCloudLadarWorker'
    webapi = repo / 'CarPointCloudWebApi'
    ports = [free_port(), free_port()]
    assert ports[0] != ports[1]
    env = {k: v for k, v in os.environ.items() if not k.upper().startswith(
        ('CARPOINT_', 'LADARCAPTURE__', 'CONNECTIONSTRINGS__', 'AUTH__', 'INTEGRATION__',
         'ASPNETCORE_', 'DOTNET_', 'DATABASE__', 'RADAR__'))}
    # 使用自定义环境，避免加载开发机的 user-secrets 或现场 Production 配置。
    env.update(DOTNET_ENVIRONMENT='OfflineSmoke', ASPNETCORE_ENVIRONMENT='OfflineSmoke')
    processes, streams = [], []
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    report = dict(realDatabase=False, realRadar=False, productionConfiguration=False, checks=[])
    try:
        for name, project, port in [('Worker', worker, ports[0]), ('WebApi', webapi, ports[1])]:
            directory = root / name
            directory.mkdir()
            settings = env.copy()
            if name == 'Worker':
                # 只复制基础采集配置，绝不复制凭据或现场覆盖文件。
                base = json.loads((project / 'appsettings.json').read_text(encoding='utf-8-sig'))
                capture = base['LadarCapture']
                for key in list(capture):
                    if any(word in key.lower() for word in ('secret', 'password', 'token', 'connectionstring')):
                        del capture[key]
                capture.update(EnableSyntheticRadar=True, EnablePythonAnalyzer=False,
                               EnableResultUploadToWebApi=False, EnableApiRequests=False,
                               FetchLicensePlateOnCapture=False, EnableApiTriggeredScanOnce=True,
                               EnableDualRadarPairing=True, EnableLivePreview=False,
                               ScanOnceOperatingMode='ContinuousSoftwareGate', IdleFrameParseMode='Always',
                               EnableHttpApi=True, HttpApiHost='127.0.0.1', HttpApiPort=port,
                               AllowNonLoopbackControlApi=False, DualRadarExtrinsicPath=None)
                for key in ('CaptureDirectory', 'PairManifestDirectory', 'AnalysisQueueDirectory',
                            'UploadQueueDirectory', 'HealthSnapshotDirectory'):
                    capture[key] = str(directory / key)
                # 设备地址仅作示例：合成模式不会调用原生 SDK。
                for i, device in enumerate(capture.get('Devices', [])):
                    device['Address'] = f'192.0.2.{11 + i}'
                capture['Address'] = '192.0.2.11'
                (directory / 'appsettings.json').write_text(json.dumps({'LadarCapture': capture}), encoding='utf-8')
                extra = []
            else:
                # 文件驱动启动：隔离配置只写测试值，不复制现场凭据。
                (directory / 'appsettings.json').write_text(json.dumps({
                    'Kestrel': {'Endpoints': {'Http': {'Url': f'http://127.0.0.1:{port}'}}},
                    'Database': {'Enabled': False, 'InitializeOnStartup': False},
                    'Integration': {'ExternalPartnerEnabled': False, 'EnableMockLicensePlate': False,
                                    'AppSecret': secrets.token_urlsafe(48)},
                    'Auth': {'JwtSecret': secrets.token_urlsafe(48)},
                    'Radar': {'BaseUrl': f'http://127.0.0.1:{ports[0]}'},
                    'AllowedHosts': '127.0.0.1;localhost',
                }), encoding='utf-8')
                extra = ['--webroot', str(project / 'wwwroot')]
            stream = (directory / 'host.log').open('w', encoding='utf-8')
            streams.append(stream)
            dll = project / 'bin' / 'Release' / 'net8.0' / f'{project.name}.dll'
            process = subprocess.Popen(['dotnet', str(dll), *extra], cwd=directory, env=settings,
                                       stdout=stream, stderr=subprocess.STDOUT)
            processes.append(process)
            health = '/api/health' if name == 'Worker' else '/healthz'
            deadline = time.monotonic() + 35
            while True:
                if process.poll() is not None:
                    raise RuntimeError(f'{name} exited ({process.returncode}); inspect isolated host.log')
                try:
                    with opener.open(f'http://127.0.0.1:{port}{health}', timeout=2) as response:
                        assert response.status == 200
                        break
                except (OSError, urllib.error.URLError):
                    if time.monotonic() > deadline:
                        raise RuntimeError(f'{name} health timeout') from None
                    time.sleep(.2)
            report['checks'].append(dict(component=name, health=200))
            print(f'{name} real entry point: health 200 (test configuration)', flush=True)
        with opener.open(f'http://127.0.0.1:{ports[1]}/pointcloud-viewer/live.html') as response:
            assert response.status == 200 and b'radar-live' in response.read()
            report['checks'].append(dict(page='live.html', status=200))
        try:
            opener.open(f'http://127.0.0.1:{ports[1]}/api/v1/radar-live/status')
            raise AssertionError('anonymous live API must require authentication')
        except urllib.error.HTTPError as error:
            assert error.code == 401
            report['checks'].append(dict(anonymousLiveApi=401))
        report['passed'] = True
        (root / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report), flush=True)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(5)
        for stream in streams:
            stream.close()


if __name__ == '__main__':
    main()
