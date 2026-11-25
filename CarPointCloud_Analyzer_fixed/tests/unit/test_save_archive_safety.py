"""保存闭环的离线故障注入，不接真实数据库或雷达。"""

from datetime import datetime
from pathlib import Path
import time
from unittest.mock import Mock

import numpy as np
import pytest

from carpointcloud_analyzer import PointCloudAnalyzer
from carpointcloud_analyzer.application import service
from carpointcloud_analyzer.infrastructure.profile_storage import EmptyVehicleProfileStorage




@pytest.mark.parametrize('clean', [False, True])
def test_no_raw_fallback_and_no_deletion_after_uncertain_commit(tmp_path, clean):
    source = tmp_path / 'merged.pcd'
    source.write_bytes(b'raw-fused')
    completed = tmp_path / 'clean.pcd'
    completed.write_bytes(b'clean-vehicle')
    store = Mock(enabled=True)
    store.get_workflow_plate.return_value = None
    store.persist_vehicle_dimensions.side_effect = RuntimeError('commit acknowledgement lost')
    storage = EmptyVehicleProfileStorage(str(tmp_path / 'profiles'))
    outputs = {'pcd': str(completed) if clean else '', 'pcdPointCount': 20 if clean else 0,
               'measurementStatus': {'valid': True}, 'loadStatus': {'status': 'Empty'},
               'integrationPayloads': {'vehicleDimensions': {
                   'licensePlate': '川A12047', 'idempotencyKey': 'scan', 'dimensions': {}}}}
    analyzer = Mock()
    analyzer.analyze.return_value = outputs
    svc = service.AnalysisService(dimension_store=store, empty_profile_storage=storage)
    try:
        with pytest.raises((RuntimeError, ValueError), match='acknowledgement|clean PCD'):
            svc._run_single(analyzer, str(source), workflow_id='same-workflow', pair_id='pair',
                            captured_at=datetime.now(), empty_profile_source=str(source),)
    finally:
        svc.shutdown()
    candidates = list(storage.root.rglob('*.pcd'))
    if clean:
        assert len(candidates) == 1
        assert candidates[0].read_bytes() == b'clean-vehicle'
        assert Path(store.persist_vehicle_dimensions.call_args.args[1]['pointCloudPath']).exists()
    else:
        store.persist_vehicle_dimensions.assert_not_called()
        assert not candidates
    assert source.read_bytes() == b'raw-fused'


def test_clean_write_failure_is_not_success_and_removes_partial_temp(tmp_path, monkeypatch):
    import open3d as o3d

    def failed_write(path, *args, **kwargs):
        Path(path).write_bytes(b'partial')
        return False

    monkeypatch.setattr(o3d.io, 'write_point_cloud', failed_write)
    with pytest.raises(IOError):
        PointCloudAnalyzer()._save_clean_pointcloud(np.ones((4, 3)), str(tmp_path / 'source.pcd'))
    assert not list(tmp_path.iterdir())


def test_same_plate_replaces_pcd_and_profile_but_keeps_dimension_history(tmp_path):
    from test_persistence import FakeDatabase, _empty_scan, _store

    database = FakeDatabase()
    storage = EmptyVehicleProfileStorage(str(tmp_path / 'profiles'))
    svc = service.AnalysisService(dimension_store=_store(database), empty_profile_storage=storage)
    source = tmp_path / 'clean.pcd'

    def save(key, hour, content):
        captured = datetime(2026, 9, 8, hour)
        source.write_bytes(content)
        analyzer = Mock()
        analyzer.analyze.return_value = {'pcd': str(source), 'pcdPointCount': 20,
            'measurementStatus': {'valid': True}, 'loadStatus': {'status': 'Empty'},
            'integrationPayloads': {'vehicleDimensions': _empty_scan(key, captured)}}
        return svc._run_single(analyzer, str(source), workflow_id=key, pair_id='pair-' + key,
                               captured_at=captured, empty_profile_source=str(source))

    try:
        first = save('first', 9, b'clean-first')
        second = save('second', 11, b'clean-second')
        # 较旧扫描晚完成不影响最新文件；同 workflow 重试不增加历史记录。
        save('late', 10, b'clean-late')
        save('second', 11, b'clean-second')
    finally:
        svc.shutdown()
    assert first['persistence']['recordId'] != second['persistence']['recordId']
    assert len(database.records) == 3
    assert len(database.profiles) == 1
    current = Path(database.profiles['川A12047'][1])
    assert current.read_bytes() == b'clean-second'
    assert list(storage.root.rglob('*.pcd')) == [current]
    assert len([kind for kind, _ in database.details if kind == 'rib']) == 3
    assert len([kind for kind, _ in database.details if kind == 'plate']) == 3


def test_final_clean_pcd_can_be_read_back_in_source_coordinates(tmp_path):
    import open3d as o3d

    points = np.array([[1., 2., 3.], [2., 3., 4.], [-1., 0., 2.]])
    path = PointCloudAnalyzer()._save_clean_pointcloud(points, str(tmp_path / 'source.pcd'))
    np.testing.assert_allclose(np.asarray(o3d.io.read_point_cloud(path).points), points)
    assert not list(tmp_path.glob('.*.tmp.pcd'))


@pytest.mark.parametrize('visible_floor', [False, True])
@pytest.mark.parametrize('late_plate', [False, True])
def test_rectangular_pipeline_persists_only_with_resolved_final_floor(
    tmp_path, monkeypatch, visible_floor, late_plate,
):
    from decimal import Decimal

    from carpointcloud_analyzer import Config
    from test_persistence import FakeDatabase, _store
    from test_wall_reference import rectangular_shell, stub_preparation

    points = rectangular_shell()
    if visible_floor:
        width, length = np.meshgrid(np.linspace(-.95, .95, 49), np.linspace(.2, 5.8, 141))
        points = np.vstack((points, np.c_[width.ravel(), length.ravel(),
                                          np.full(width.size, 1.5)]))
    source = tmp_path / 'paired-source.pcd'
    source.write_bytes(b'raw-source-preserved')
    # 模拟HTTP按原workflow取牌后的本次配置；迟到识牌尚未得到车牌。
    analyzer = PointCloudAnalyzer(Config(license_plate='UNKNOWN' if late_plate else '川A12047',
                                       camera_identifier='CAM_004',
                                       save_clean_pointcloud=True))
    stub_preparation(monkeypatch, analyzer, points)
    monkeypatch.setattr(analyzer.loader, 'load', lambda _: points.copy())
    database = FakeDatabase()
    store = _store(database)
    storage = EmptyVehicleProfileStorage(str(tmp_path / 'profiles'))
    svc = service.AnalysisService(dimension_store=store, empty_profile_storage=storage)
    try:
        for index in [1, 2]:
            workflow = f'review-{index}'
            database.arrivals[workflow] = ((None, 'PENDING') if late_plate
                                           else ('川A12047', 'SUCCEEDED'))
            result = svc._run_single(analyzer, str(source), workflow_id=workflow, pair_id=f'pair-{index}',
                                     captured_at=datetime(2026, 9, 8, 10 + index), empty_profile_source=str(source))
            if visible_floor and late_plate:
                assert result['persistence']['status'] == 'inserted'
                assert len(database.records) == index
                assert not database.profiles
    finally:
        svc.shutdown()
    if not visible_floor:
        assert result['persistence']['status'] == 'inserted'
        assert result['integrationPayloads']['vehicleDimensions']['detectStatus'] == 'success'
        assert result['outputs']['dimensions']['carriage']['height'] is None
        assert result['outputs']['loadStatus']['status'] == 'Uncertain'
        assert database.records and not database.profiles
        assert not list(storage.root.rglob('*.pcd'))
        assert source.read_bytes() == b'raw-source-preserved'
        return
    assert result['persistence']['status'] == 'inserted'
    payload = result['integrationPayloads']['vehicleDimensions']
    assert payload['detectStatus'] == 'success'
    inner_cm = payload['dimensions']['trailerBodyHeight']
    assert 255 < inner_cm < 265
    assert payload['dimensions']['trailerBodyHeight'] == inner_cm
    assert payload['dimensions']['trailerBottomToGroundHeight'] is None
    assert payload['dimensions']['cabToTrailerFrontDist'] is None
    assert result['outputs']['loadStatus']['status'] == 'Empty'
    assert 'reviewStatus' not in result['outputs']['measurementStatus']
    assert len(database.records) == 2
    if late_plate:
        assert not database.profiles
        jobs = {job.payload["workflowId"]: job for job in storage.pending()}
        assert set(jobs) == {"review-1", "review-2"}
        expected_path = jobs["review-2"].profile.point_cloud_path
        assert Path(expected_path).read_bytes() == Path(result['outputs']['pcd']).read_bytes()
        Path(result['outputs']['pcd']).unlink()
        database.arrivals["review-1"] = (None, "FAILED")
        database.arrivals["review-2"] = ("川A12047", "SUCCEEDED")
        # 重启 Analyzer，原计算输出已移除，依然可由待办补登。
        restarted = service.AnalysisService(dimension_store=store, empty_profile_storage=storage)
        try:
            restarted.startup()
            deadline = time.monotonic() + 3
            while list(storage.pending()) and time.monotonic() < deadline:
                time.sleep(.02)
            assert not list(storage.pending())
            assert list(storage.root.rglob('*.pcd')) == [Path(expected_path)]
            assert database.profiles["川A12047"][1] == expected_path
            assert database.record_plates[result['persistence']['recordId']] == "川A12047"
            assert len(database.records) == 2
        finally:
            restarted.shutdown()
        return
    assert len(database.profiles) == 1
    records = [values for kind, values in database.details if kind == 'record']
    assert all(values[7] == 'success' for values in records)
    assert records[-1][11] == Decimal(str(inner_cm))
    assert records[-1][12] is None
    assert len(list(storage.root.rglob('*.pcd'))) == 1
    # 后台/浏览器按字节读取中文车牌路径，不依赖 Open3D 原生路径编码。
    # 独立验证实际生成的 binary XYZ PCD，而非用目录名称冒充读回成功。
    blob = Path(result['outputs']['pcd']).read_bytes()
    header, binary = blob.split(b'DATA binary\n', 1)
    assert b'FIELDS x y z\n' in header and b'SIZE 4 4 4\n' in header
    saved = np.frombuffer(binary, dtype='<f4').reshape(-1, 3)
    assert np.isfinite(saved).all()
    # 档案外形高度就是本次观测跨度；不再有配置来源的标称外高。
    assert np.ptp(saved[:, 2]) == pytest.approx(2.6, abs=.06)
    assert len(saved) == result['outputs']['pcdPointCount']
    assert len(saved) > len(points)  # 有补点，但密度不应绑定任意固定下限。
    assert source.read_bytes() == b'raw-source-preserved'


def test_successful_commit_keeps_unconfirmed_candidates_from_failed_calls(tmp_path):
    """失败两次再成功，旧候选缺乏未引用证明，不能凭新回执回收。"""
    from types import SimpleNamespace

    source = tmp_path / 'clean.pcd'
    source.write_bytes(b'clean-vehicle')
    storage = EmptyVehicleProfileStorage(str(tmp_path / 'profiles'))
    referenced = {}

    class Store:
        enabled = True
        fail = True

        def persist_vehicle_dimensions(self, payload, staged):
            if self.fail:
                raise RuntimeError('injected failure before database commit')
            referenced['path'] = staged['pointCloudPath']
            return SimpleNamespace(to_dict=lambda: {'emptyVehicleProfile': {
                'updated': True, 'pointCloudPath': staged['pointCloudPath'],
                'previousPointCloudPath': ''}})

    store = Store()
    svc = service.AnalysisService(dimension_store=store, empty_profile_storage=storage)
    payload = {'licensePlate': 'TEST001', 'idempotencyKey': 'same-scan'}
    outputs = {'loadStatus': {'status': 'Empty'}, 'pcd': str(source), 'pcdPointCount': 1}
    counts = []
    try:
        for _ in range(2):
            with pytest.raises(RuntimeError, match='injected failure'):
                svc._archive_and_persist(payload, outputs, 'same-scan', 'same-pair',
                                         datetime(2026, 11, 6), str(source))
            counts.append(len(list(storage.root.rglob('*.pcd'))))
        store.fail = False
        svc._archive_and_persist(payload, outputs, 'same-scan', 'same-pair',
                                 datetime(2026, 11, 6), str(source))
        counts.append(len(list(storage.root.rglob('*.pcd'))))
    finally:
        svc.shutdown()

    assert counts == [1, 2, 3]
    remaining = list(storage.root.rglob('*.pcd'))
    assert len(remaining) == 3
    assert Path(referenced['path']).read_bytes() == b'clean-vehicle'
    assert all(path.read_bytes() == b'clean-vehicle' for path in remaining)


def test_delayed_receipt_cannot_delete_later_committed_candidate(tmp_path):
    """同扫描后一次已提交但关闭连接失败，旧回执不得删除当前数据库引用。"""
    from concurrent.futures import ThreadPoolExecutor
    from threading import Event

    from test_persistence import FakeDatabase, _empty_scan, _store

    first_committed, release_first = Event(), Event()

    class Database(FakeDatabase):
        connections = 0

        def connect(self, **kwargs):
            connection = super().connect(**kwargs)
            self.connections += 1
            if self.connections == 2:
                def close_with_error():
                    assert connection.committed
                    raise OSError('connection close failed after commit')
                connection.close = close_with_error
            return connection

    database = Database()
    actual_store = _store(database)

    class Store:
        enabled = True

        def persist_vehicle_dimensions(self, payload, staged):
            receipt = actual_store.persist_vehicle_dimensions(payload, staged)
            first_committed.set()
            assert release_first.wait(10)
            return receipt

    storage = EmptyVehicleProfileStorage(str(tmp_path / 'profiles'))
    source = tmp_path / 'clean.pcd'
    source.write_bytes(b'clean-vehicle')
    at = datetime(2026, 11, 6, 12)
    payload = _empty_scan('same-workflow', at)
    database.arrivals['same-workflow'] = ('川A12047', 'SUCCEEDED')
    outputs = {'loadStatus': {'status': 'Empty'}, 'pcd': str(source), 'pcdPointCount': 1}
    svc = service.AnalysisService(dimension_store=Store(), empty_profile_storage=storage)

    def save():
        return svc._archive_and_persist(payload, outputs, 'same-workflow',
                                        'pair-same-workflow', at, str(source))

    try:
        with ThreadPoolExecutor(max_workers=1) as executor:
            first = executor.submit(save)
            try:
                assert first_committed.wait(10)
                with pytest.raises(OSError, match='after commit'):
                    save()
                current = Path(database.profiles['川A12047'][1])
                assert current.read_bytes() == b'clean-vehicle'
            finally:
                release_first.set()
            first.result(timeout=10)
        assert len(database.records) == len(database.profiles) == 1
        assert current.read_bytes() == b'clean-vehicle'
    finally:
        svc.shutdown()
