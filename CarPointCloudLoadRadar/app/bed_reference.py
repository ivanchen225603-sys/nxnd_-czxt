"""独立裸露底板参考的内部合同；输入绑定不等于来源真实性验证。"""
from __future__ import annotations

import hashlib
import math
import struct
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class BedReferenceScope:
    baseline_version_id: str
    calibration_id: str
    configuration_sha256: str
    channel_id: str
    passage_id: str
    capture_epoch: str
    frame_ids: Tuple[Tuple[int, int, float], ...]  # 有序(gen, seq, ts)，含合帧来源。
    coordinate_frame: str
    measured_at: float
    # surface顺序与真实时间顺序不同，主合帧可对应多帧，extra可重用原帧。
    # 空值兼容旧的一帧一surface合同；非空映射由运行时单独构造并参与scope相等检查。
    surface_members: Tuple[Tuple[int, ...], ...] = ()


@dataclass(frozen=True)
class BedReferenceEvidence:
    source: str
    reference_id: str
    scope: BedReferenceScope
    baseline_sha256: str
    surface_sha256: str
    confirmed_exposed: bool
    reference_xy: Tuple[float, float]
    bed_z: float
    uncertainty_m: float
    valid_from: float
    valid_until: float


@dataclass(frozen=True)
class BedShiftDecision:
    status: str = "not_requested"
    reason: str = "使用原基线，未请求底板修正"
    shift_m: float = 0.0
    source: Optional[str] = None
    reference_id: Optional[str] = None
    uncertainty_m: Optional[float] = None


def _finite(value) -> bool:
    try:
        return (not isinstance(value, (bool, np.bool_)) and
                isinstance(value, (int, float, np.integer, np.floating)) and math.isfinite(value))
    except OverflowError:
        return False


def _label(value) -> bool:
    return (isinstance(value, str) and 0 < len(value) <= 256 and value == value.strip()
            and all(ord(c) >= 32 and ord(c) != 127 for c in value))


def _digest(value) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def _array_bytes(values, shape=None) -> bytes:
    a = np.asarray(values, dtype="<f8")
    if (shape is not None and a.shape != shape) or not np.isfinite(a).all():
        raise ValueError("参考摘要只接受预期形状的有限数值")
    return struct.pack("<Q", a.ndim) + b"".join(struct.pack("<Q", s) for s in a.shape) + a.tobytes(order="C")


def baseline_digest(baseline) -> str:
    """绑定实际底板、盒、平面、点数和云；不以文件名或版本号代替内容。"""
    cloud = np.asarray(baseline.cloud, dtype="<f8")
    if cloud.ndim != 2 or cloud.shape[1] != 3 or baseline.n != len(cloud):
        raise ValueError("基线云格式或点数不一致")
    digest = hashlib.sha256(b"bed-baseline-v1\0")
    digest.update(_array_bytes(cloud))
    digest.update(_array_bytes(baseline.box, (6,)))
    digest.update(_array_bytes([baseline.floor_z], (1,)))
    digest.update(b"none" if baseline.bed_plane is None else _array_bytes(baseline.bed_plane, (4,)))
    return digest.hexdigest()


def surface_digest(clouds: Sequence[np.ndarray]) -> str:
    """主货面及全部额外货面按原顺序绑定；不保存可变数组引用。"""
    digest = hashlib.sha256(b"bed-surface-v1\0" + struct.pack("<Q", len(clouds)))
    for cloud in clouds:
        a = np.asarray(cloud, dtype="<f8")
        if a.ndim != 2 or a.shape[1] != 3:
            raise ValueError("货面云格式非法")
        digest.update(_array_bytes(a))
    return digest.hexdigest()


def _valid_scope(scope) -> bool:
    if not (isinstance(scope, BedReferenceScope) and
            all(_label(getattr(scope, name)) for name in (
                "baseline_version_id", "calibration_id", "channel_id", "passage_id", "capture_epoch", "coordinate_frame")) and
            _digest(scope.configuration_sha256) and _finite(scope.measured_at) and scope.measured_at > 0):
        return False
    if not isinstance(scope.frame_ids, tuple) or not scope.frame_ids:
        return False
    for frame in scope.frame_ids:
        if (not isinstance(frame, tuple) or len(frame) != 3 or
                any(type(v) is not int or v < 0 for v in frame[:2]) or frame[1] == 0 or
                not _finite(frame[2]) or frame[2] <= 0):
            return False
    frames = scope.frame_ids
    if not (frames[-1][2] == scope.measured_at and
            all(a[0] == b[0] and a[1] < b[1] and a[2] < b[2] for a, b in zip(frames, frames[1:]))):
        return False
    members = scope.surface_members
    if not isinstance(members, tuple):
        return False
    if members:
        for group in members:
            if (not isinstance(group, tuple) or not group or
                    any(type(i) is not int or not 0 <= i < len(frames) for i in group) or
                    any(a >= b for a, b in zip(group, group[1:]))):
                return False
        if set(i for group in members for i in group) != set(range(len(frames))):
            return False
        if members[0][-1] != len(frames) - 1:
            return False
    return True


def valid_surface_scope(scope, surface_count: int) -> bool:
    """真实时间线不重排；明确映射当前有序主/extra货面到其原始观测。"""
    return (_valid_scope(scope) and type(surface_count) is int and surface_count > 0 and
            (len(scope.surface_members) if scope.surface_members else len(scope.frame_ids)) == surface_count)


def resolve_bed_shift(baseline, surfaces: Sequence[np.ndarray], bed_z: Optional[float],
                      evidence: Optional[BedReferenceEvidence], expected_scope: Optional[BedReferenceScope],
                      dy: float, max_shift_m: float) -> BedShiftDecision:
    """缺证据保持基线；只有独立参考与当前实际输入完全绑定才应用竖向平移。

    TODO(现场): 独立参考须由可信调用方验证来源；当前Service无此供应路径。
    不接受analyze_load输出自证裸露，亦不接受来自HTTP/配置的未验证对象。
    """
    def reject(reason):
        return BedShiftDecision(status="rejected", reason="底板修正未采纳：" + reason)

    if evidence is None:
        return (BedShiftDecision() if bed_z is None else
                BedShiftDecision(status="unverified", reason="底板修正未采纳：缺少独立裸露底板证据"))
    if not isinstance(evidence, BedReferenceEvidence):
        return reject("证据类型非法")
    if (not isinstance(evidence.source, str) or
            evidence.source not in ("independent_exposed_bed", "surveyed_exposed_bed") or
            not _label(evidence.reference_id) or evidence.confirmed_exposed is not True):
        return reject("未声明独立裸露参考")
    if (not valid_surface_scope(expected_scope, len(surfaces)) or
            not valid_surface_scope(evidence.scope, len(surfaces))):
        return reject("缺少当前测量身份或参考身份非法")
    if evidence.scope != expected_scope:
        return reject("参考与当前版本、标定、配置、车次、帧、坐标域或时间不匹配")
    if (not all(_finite(v) for v in (evidence.valid_from, evidence.valid_until)) or
            not 0 < evidence.valid_from <= expected_scope.measured_at <= evidence.valid_until):
        return reject("参考不在有效时间内")
    if (not all(_finite(v) for v in (bed_z, evidence.bed_z, evidence.uncertainty_m)) or
            evidence.uncertainty_m < 0 or not math.isclose(bed_z, evidence.bed_z, rel_tol=0., abs_tol=1e-9)):
        return reject("参考高度不一致或误差非法")
    if (not isinstance(evidence.reference_xy, tuple) or len(evidence.reference_xy) != 2 or
            not all(_finite(v) for v in evidence.reference_xy)):
        return reject("参考位置非法")
    if (not _digest(evidence.baseline_sha256) or not _digest(evidence.surface_sha256) or
            evidence.baseline_sha256 != baseline_digest(baseline) or
            evidence.surface_sha256 != surface_digest(surfaces)):
        return reject("实际基线或货面内容与参考不匹配")
    x, y = map(float, evidence.reference_xy)
    x0, x1, y0, y1 = map(float, baseline.box[:4])
    if not (x0 <= x <= x1 and y0 + dy <= y <= y1 + dy):
        return reject("参考位置不在配准后的基线区域")
    # bed_z是指定参考点高度，不是随积分网格变化的全图中位高度。
    base_level = float(baseline.bed_z_at(x, y - dy))
    shift = float(bed_z) - base_level
    if not math.isfinite(shift) or abs(shift) + evidence.uncertainty_m > max_shift_m:
        return reject("修正量及误差超过允许范围")
    return BedShiftDecision("applied", "已按独立裸露底板参考校正", shift,
                            evidence.source, evidence.reference_id, float(evidence.uncertainty_m))
