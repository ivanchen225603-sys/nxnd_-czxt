"""永久空车点云文件存储。"""

from carpointcloud_analyzer.application.ports import (
    StagedEmptyVehicleProfile as StagedEmptyVehicleProfile,
    DisabledEmptyVehicleProfileStorage as DisabledEmptyVehicleProfileStorage,
    PendingEmptyVehicleProfile, PersistResult,
)


import hashlib
import json
import logging
import os
import shutil

from carpointcloud_analyzer.domain.utils.environment import env_bool as _env_bool
import re
from datetime import datetime
from pathlib import Path
from typing import Any, Mapping
from uuid import uuid4


_INVALID_FILE_CHARS = re.compile(r"[^0-9A-Za-z\u4e00-\u9fff_-]+")
logger = logging.getLogger(__name__)


class EmptyVehicleProfileStorage:
    """保存本次最终空车云，数据库提交后清理被替换的旧档案。"""

    enabled = True

    def __init__(self, root: str) -> None:
        """创建永久档案目录。"""
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self._pending_dir = self.root / ".pending-plates"

    @classmethod
    def from_env(cls) -> "EmptyVehicleProfileStorage | DisabledEmptyVehicleProfileStorage":
        """按环境开关创建永久存储或禁用实现。"""
        enabled = _env_bool("CARPOINT_EMPTY_PROFILE_ENABLED", False)
        if not enabled:
            return DisabledEmptyVehicleProfileStorage()
        root = os.getenv(
            "CARPOINT_EMPTY_PROFILE_ROOT",
            "/data/carpointcloud/empty-vehicle-profiles",
        )
        return cls(root)

    def stage(
        self,
        source_pcd: str | Path,
        license_plate: str,
        stable_scan_id: str,
        point_count: int,
        scan_time: datetime,
    ) -> StagedEmptyVehicleProfile:
        """把本次最终空车 PCD 复制为新一代档案候选；车牌与点数已由调用方确认。"""
        plate = license_plate.strip().upper()
        safe_plate = _INVALID_FILE_CHARS.sub("_", plate).strip("_")[:24] or "plate"
        plate_digest = hashlib.sha256(plate.encode("utf-8")).hexdigest()[:16]
        return self._stage_candidate(source_pcd, self.root / f"{safe_plate}_{plate_digest}",
                                     stable_scan_id, point_count, scan_time)

    @staticmethod
    def _stage_candidate(source_pcd, target_dir, stable_scan_id, point_count, scan_time):
        scan_digest = hashlib.sha256(stable_scan_id.encode("utf-8")).hexdigest()[:24]
        target_dir.mkdir(parents=True, exist_ok=True)
        target = target_dir / f"{scan_digest}_{uuid4().hex}_fused_empty.pcd"
        shutil.copyfile(source_pcd, target)
        digest = hashlib.sha256(target.read_bytes())

        return StagedEmptyVehicleProfile(
            point_cloud_path=str(target),
            point_cloud_sha256=digest.hexdigest(),
            point_cloud_size=target.stat().st_size,
            point_count=int(point_count),
            scan_time=scan_time,
        )

    def defer(self, source_pcd: str | Path, payload: Mapping[str, Any],
              point_count: int, scan_time: datetime) -> None:
        """先保存完整净化点云，再原子发布同一 workflow 的待办。"""
        workflow_id = str(payload["workflowId"])
        key = hashlib.sha256(workflow_id.encode("utf-8")).hexdigest()
        self._pending_dir.mkdir(exist_ok=True)
        receipt = self._pending_dir / f"{key}.json"
        if receipt.exists():
            return
        profile = self._stage_candidate(source_pcd, self.root / "late-plates" / key,
                                        workflow_id, point_count, scan_time)
        mapping = profile.to_mapping()
        mapping["scanTime"] = scan_time.isoformat()
        temp = receipt.with_suffix(f".{uuid4().hex}.tmp")
        try:
            temp.write_text(json.dumps({"payload": dict(payload), "profile": mapping},
                                       ensure_ascii=False, allow_nan=False), encoding="utf-8")
            os.link(temp, receipt)
        except FileExistsError:
            self._delete_if_managed(profile.point_cloud_path)
        except Exception:
            self._delete_if_managed(profile.point_cloud_path)
            raise
        finally:
            temp.unlink(missing_ok=True)

    def pending(self):
        for receipt in sorted(self._pending_dir.glob("*.json")):
            try:
                data = json.loads(receipt.read_text(encoding="utf-8"))
                profile = data["profile"]
                yield PendingEmptyVehicleProfile(receipt, data["payload"], StagedEmptyVehicleProfile(
                    point_cloud_path=profile["pointCloudPath"],
                    point_cloud_sha256=profile["pointCloudSha256"],
                    point_cloud_size=profile["pointCloudSize"],
                    point_count=profile["pointCount"],
                    scan_time=datetime.fromisoformat(profile["scanTime"]),
                ))
            except FileNotFoundError:
                continue
            except (OSError, ValueError, KeyError, TypeError):
                logger.exception("无法读取迟到车牌的空车归档待办 path=%s", receipt)

    def finish_pending(self, job: PendingEmptyVehicleProfile, result: PersistResult | None) -> None:
        if result is None:
            self._delete_if_managed(job.profile.point_cloud_path)
        else:
            self.finalize(job.profile.point_cloud_path, result.to_dict())
        job.receipt_path.unlink(missing_ok=True)

    def finalize(
        self,
        staged_path: str,
        persistence: Mapping[str, Any],
    ) -> None:
        """仅清理本次竞败候选与回执明确替换的上一代，不推断其他候选的引用状态。"""
        profile = persistence.get("emptyVehicleProfile")
        if not isinstance(profile, Mapping):
            # 缺少提交回执不能证明未引用；保留候选，本服务不自动回收。
            return

        current = str(profile.get("pointCloudPath") or "")
        if not current:
            return
        updated = bool(profile.get("updated"))
        previous = str(profile.get("previousPointCloudPath") or "")
        if not _same_path(staged_path, current):
            self._delete_if_managed(staged_path)
        if updated and previous and not _same_path(previous, current):
            self._delete_if_managed(previous)

    def _delete_if_managed(self, path: str) -> None:
        """尽力删除受管文件，绝不触碰归档根目录外的路径。"""
        if not path:
            return
        candidate = Path(path).resolve()
        try:
            candidate.relative_to(self.root)
        except ValueError:
            return
        try:
            candidate.unlink(missing_ok=True)
            parent = candidate.parent
            if parent != self.root and not any(parent.iterdir()):
                parent.rmdir()
        except OSError:
            # 文件清理只是尽力而为：数据库始终指向已完整提交的文件，
            # 留下孤儿文件比删掉当前在用的文件更安全。
            return


def _same_path(left: str, right: str) -> bool:
    """按当前操作系统路径规则比较两个非空路径。"""
    if not left or not right:
        return False
    return os.path.normcase(os.path.abspath(left)) == os.path.normcase(os.path.abspath(right))
