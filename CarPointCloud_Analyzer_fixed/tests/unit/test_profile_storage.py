from datetime import datetime
from pathlib import Path

import pytest

from carpointcloud_analyzer.infrastructure.profile_storage import EmptyVehicleProfileStorage


def test_profile_storage_stages_immutable_generations_even_for_same_workflow(tmp_path):
    allowed = tmp_path / "data"
    root = allowed / "empty-profiles"
    source = allowed / "captures" / "pair.pcd"
    source.parent.mkdir(parents=True)
    source.write_bytes(b"PCD-DATA")
    storage = EmptyVehicleProfileStorage(str(root))
    scan_time = datetime(2026, 8, 13, 9, 30, 0)

    first = storage.stage(source, "川A/..123", "workflow-001", 123, scan_time)
    source.write_bytes(b"PCD-RETRY")
    second = storage.stage(source, "川A/..123", "workflow-001", 123, scan_time)

    assert first.point_cloud_path != second.point_cloud_path
    assert Path(first.point_cloud_path).read_bytes() == b"PCD-DATA"
    assert Path(second.point_cloud_path).read_bytes() == b"PCD-RETRY"
    assert first.point_cloud_size == 8
    assert first.point_count == 123
    assert ".." not in Path(first.point_cloud_path).parent.name
    assert not list(root.rglob("*.tmp"))


@pytest.mark.parametrize('receipt', [{}, {'emptyVehicleProfile': {}},
                                    {'emptyVehicleProfile': {'updated': True}}])
def test_missing_commit_receipt_never_deletes_candidate(tmp_path, receipt):
    storage = EmptyVehicleProfileStorage(str(tmp_path / 'profiles'))
    source = tmp_path / 'source.pcd'
    source.write_bytes(b'PCD')
    candidate = storage.stage(source, '川A12047', 'same', 10, datetime.now())
    storage.finalize(candidate.point_cloud_path, receipt)
    assert Path(candidate.point_cloud_path).read_bytes() == b'PCD'


def test_profile_storage_finalize_removes_superseded_generation(tmp_path):
    root = tmp_path / "profiles"
    storage = EmptyVehicleProfileStorage(str(root))
    source = tmp_path / "source.pcd"
    source.write_bytes(b"PCD")
    old = storage.stage(source, "川A12047", "old", 10, datetime(2026, 8, 13, 9))
    current = storage.stage(source, "川A12047", "new", 10, datetime(2026, 8, 13, 10))

    storage.finalize(
        current.point_cloud_path,
        {
            "emptyVehicleProfile": {
                "updated": True,
                "pointCloudPath": current.point_cloud_path,
                "previousPointCloudPath": old.point_cloud_path,
            }
        },
    )

    assert Path(current.point_cloud_path).is_file()
    assert not Path(old.point_cloud_path).exists()


def test_finalize_keeps_other_candidates_even_for_same_scan(tmp_path):
    storage = EmptyVehicleProfileStorage(str(tmp_path / "profiles"))
    source = tmp_path / "source.pcd"
    source.write_bytes(b"PCD")

    other_scan = storage.stage(source, "川A12047", "other-workflow", 10, datetime(2026, 8, 13, 9))
    failed = storage.stage(source, "川A12047", "same-workflow", 10, datetime(2026, 8, 13, 9))
    current = storage.stage(source, "川A12047", "same-workflow", 10, datetime(2026, 8, 13, 10))

    storage.finalize(current.point_cloud_path, {
        "emptyVehicleProfile": {
            "updated": True,
            "pointCloudPath": current.point_cloud_path,
            "previousPointCloudPath": "",
        }
    })

    assert Path(current.point_cloud_path).is_file()
    # 同扫描和不同扫描的其他候选，都不能凭本次回执推断未被引用。
    assert Path(failed.point_cloud_path).is_file()
    assert Path(other_scan.point_cloud_path).is_file()


def test_currently_referenced_candidate_survives_finalize(tmp_path):
    """数据库可能已提交、只是客户端没收到结果；被回执引用的文件不能删。"""
    storage = EmptyVehicleProfileStorage(str(tmp_path / "profiles"))
    source = tmp_path / "source.pcd"
    source.write_bytes(b"PCD")
    current = storage.stage(source, "川A12047", "same-workflow", 10, datetime(2026, 8, 13, 9))

    storage.finalize(current.point_cloud_path, {
        "emptyVehicleProfile": {
            "updated": True,
            "pointCloudPath": current.point_cloud_path,
            "previousPointCloudPath": "",
        }
    })

    assert Path(current.point_cloud_path).read_bytes() == b"PCD"


def test_missing_commit_receipt_keeps_all_candidates(tmp_path):
    storage = EmptyVehicleProfileStorage(str(tmp_path / "profiles"))
    source = tmp_path / "source.pcd"
    source.write_bytes(b"PCD")
    failed = storage.stage(source, "川A12047", "same", 10, datetime(2026, 8, 13, 9))
    current = storage.stage(source, "川A12047", "same", 10, datetime(2026, 8, 13, 10))

    storage.finalize(current.point_cloud_path, {})

    assert Path(failed.point_cloud_path).is_file()
    assert Path(current.point_cloud_path).is_file()
