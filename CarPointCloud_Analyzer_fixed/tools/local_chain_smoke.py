"""离线跨项目冒烟：真实 PCD、真实 HTTP/SQL 代码路径、假的数据库驱动。

生产代码不得导入这个仅用于测试的宿主。只使用临时的回环套接字；
传入的输出目录必须不存在。全程不建立真实的 dmPython 连接。
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import threading
import time

import numpy as np
import uvicorn

from carpointcloud_analyzer.infrastructure.profile_storage import EmptyVehicleProfileStorage
from carpointcloud_analyzer.composition import create_app


CASE_EXPECTATIONS = {
    'new': {'terminal': 'completed', 'lengthCm': [538, 546], 'review': 'pending'},
    'original': {'terminal': 'completed', 'lengthCm': [1160, 1190], 'review': 'pending'},
    'd0924-1002': {'terminal': 'completed_without_dimensions'},
    # 当前算法回归范围，不是卷尺真值；测长来自本次实测面支撑的补壁推算。
    'd0924-1004': {'terminal': 'completed', 'lengthCm': [1135, 1138],
                   'lengthMethod': 'completed_lower_side_inner_edges', 'review': 'pending'},
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', nargs=3, action='append', required=True)
    parser.add_argument('--extrinsic', required=True)
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(project / 'tests' / 'unit'))
    from test_persistence import FakeDatabase, _store

    root = Path(args.output_dir).resolve()
    root.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(args.extrinsic, root / 'extrinsic.json')
    database = FakeDatabase()
    cases, hashes = [], {}
    for i, (name, a, b) in enumerate(args.case):
        workflow = f'local-smoke-{i}'
        plate = f'川A{i:05d}'
        database.arrivals[workflow] = (plate, 'SUCCEEDED')
        paths = []
        for camera, source in [('A', a), ('B', b)]:
            source = Path(source).resolve()
            hashes[str(source)] = hashlib.sha256(source.read_bytes()).hexdigest()
            target = root / f'{i}_{camera}.pcd'
            shutil.copyfile(source, target)
            paths.append(str(target))
        if name not in CASE_EXPECTATIONS:
            raise ValueError(f'未定义用例预期: {name}')
        cases.append(dict(name=name, workflow=workflow, plate=plate, a=paths[0], b=paths[1],
                          expected=CASE_EXPECTATIONS[name]))
    storage = EmptyVehicleProfileStorage(str(root / 'profiles'))
    app = create_app(max_concurrency=1,
                     dimension_store=_store(database),
                     empty_profile_storage=storage, extrinsic_path=str(root / 'extrinsic.json'))

    @app.get('/_test/persisted/{workflow}')
    def persisted(workflow: str):
        # 读 SQL 驱动实际记录的 INSERT 参数，不构造期望结果。
        records = [p for kind, p in database.details if kind == 'record' and p[1] == workflow]
        if not records:
            return None
        p = records[-1]
        fields = ['idempotencyKey', 'workflowId', 'pairId', 'payloadHash', 'licensePlate',
                  'cameraIdentifier', 'getTime', 'detectStatus', 'cabToTrailerFrontDist',
                  'trailerLength', 'trailerWidth', 'trailerBodyHeight',
                  'trailerBottomToGroundHeight', 'highLowPlateHeight']
        record_id = database.records[p[0]][0]
        return dict(zip(fields, p[:len(fields)], strict=True),
                    id=record_id, actualRibCount=p[19], maximumWidthCm=p[20],
                    highLowPlates=[dict(plateId=row[1], highestToGround=row[2],
                                        lowestToGround=row[3], lengthCm=row[4])
                                   for kind, row in database.details
                                   if kind == 'plate' and row[0] == record_id],
                    wires=[dict(wireId=row[1], distToPrev=row[2])
                           for kind, row in database.details
                           if kind == 'wire' and row[0] == record_id],
                    ribs=[dict(ribId=row[1], distToPrev=row[2])
                          for kind, row in database.details
                          if kind == 'rib' and row[0] == record_id])

    @app.get('/_test/profile/{plate}')
    def profile_record(plate: str):
        profile = database.profiles.get(plate)
        if profile is None:
            return None
        rows = [p for kind, p in database.details if kind == 'profile_insert' and p[0] == plate]
        p = rows[-1]
        return dict(licensePlate=plate, workflowId=p[2], pointCloudPath=profile[1])

    sock = socket.socket()
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()[1]
    config = uvicorn.Config(app, host='127.0.0.1', port=port, log_level='warning')
    server = uvicorn.Server(config)
    thread = threading.Thread(target=server.run, kwargs={'sockets': [sock]}, daemon=True)
    thread.start()
    env = {k: v for k, v in os.environ.items() if not k.upper().startswith(
        ('CARPOINT_', 'LADARCAPTURE__', 'CONNECTIONSTRINGS__', 'AUTH__', 'INTEGRATION__'))}
    try:
        deadline = time.monotonic() + 30
        while not server.started and thread.is_alive() and time.monotonic() < deadline:
            time.sleep(.1)
        if not server.started:
            raise RuntimeError('isolated Analyzer did not start')
        manifest = root / 'cases.json'
        manifest.write_text(json.dumps(cases, ensure_ascii=False), encoding='utf-8')
        # 这里刻意用 Debug：现场的 Worker/WebApi 直接从 <project>/bin/Release/net8.0
        # 运行，而被引用项目的 Release 构建会写同一个目录（它们运行时文件被占用会
        # 构建失败；它们停止时会静默替换已部署的二进制）。
        command = ['dotnet', 'run', '--project', str(project.parent / 'tools' / 'LocalChainSmoke'),
                   '-c', 'Debug', '--', f'http://127.0.0.1:{port}', str(manifest)]
        subprocess.run(command, check=True, env=env, timeout=1200)
        archives = []
        for plate, profile in database.profiles.items():
            path = Path(profile[1])
            blob = path.read_bytes()
            header, binary = blob.split(b'DATA binary\n', 1)
            assert b'FIELDS x y z\n' in header
            points = np.frombuffer(binary, dtype='<f4').reshape(-1, 3)
            assert len(points) > 1000 and np.isfinite(points).all()
            archives.append(dict(plate=plate, points=len(points), path=str(path),
                                 sha256=hashlib.sha256(blob).hexdigest()))
        expected_records = sum(case['expected']['terminal'] == 'completed' for case in cases)
        assert len(database.records) == expected_records == len(archives)
        for source, digest in hashes.items():
            assert hashlib.sha256(Path(source).read_bytes()).hexdigest() == digest
        report = dict(passed=True, realDatabase=False, realRadar=False,
                      acquisition='copied recorded PCD; simulated pairing timestamps, not field evidence',
                      databaseAdapter='FakeDatabase; real Dm8DimensionStore and profile storage',
                      cases=[{'name': case['name'], 'expected': case['expected']} for case in cases],
                      records=len(database.records), archives=archives, sourceSha256=hashes,
                      webapi='real query/controller; repository populated from actual SQL-driver records')
        (root / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        print(json.dumps(report, ensure_ascii=False), flush=True)
    finally:
        server.should_exit = True
        thread.join(30)
        sock.close()


if __name__ == '__main__':
    main()
