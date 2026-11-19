"""进场/离场帧接入服务体积：冻结、配准、记录与复算共用这一处。

进场（2026-10-24，docs/92）：车往里开时车厢前端经过雷达附近，停稳后扫不全的前端那时看得见。
离场（对称接入）：本车测完往外开时后端经过天底；采集顺序最早在前、近端在前，配准前须 reverse，
让最后一帧贴近停稳（`register_enter_frames` 锚的是 bands[-1]）。

`detect.register_enter_frames` + `compute_volume(enter_extra=...)` 是 10-02 写好的离线路径：

* 输入是冻结的 ROI 裁剪 `FrameSnapshot`（进场：判稳时；离场：缓冲满/落盘时），
  用主货面同一个航向旋转对齐，再做相邻帧链式配准（不改配准算法与门槛）。
* 配不上就**不补**，体积照旧只用停稳帧；绝不因为进/离场帧失败而拒绝测量。
* 归档保存原帧字节与配准记录；复算时重新配准，对不上判不能复现。

▲ 只补停稳帧**没测到**的格，实测格一个不改；不进判定链、不进停稳、不改车厢盒。
▲ 只认 ROI 裁剪帧（`enter_crop_roi` / `exit_crop_roi`）。存整帧时坐标域不同，不接。
"""
from __future__ import annotations

import hashlib
from typing import Optional, Sequence, Tuple

import numpy as np

from .cloud_observation import FrameSnapshot
from .volume_inputs import align_extra_surfaces


ENTER_VOLUME_SCHEMA = "enter-volume-v1"
ENTER_FRAMES_MEMBER = "enter-frames.f32le"
EXIT_FRAMES_MEMBER = "exit-frames.f32le"
ENTER_MAX_FRAMES = 64              # 与 service.enter_frames / exit_frames 上限无关的硬预算
ENTER_MAX_POINTS = 4_000_000       # 全部进场（或离场）帧合计点数上限
_RECORD_KEYS = {"schema", "status", "reason", "used", "shifts", "frames"}
_FRAME_KEYS = {"seq", "ts", "gen", "packets", "dropped_packets", "points", "sha256"}


def volume_snapshot(frame) -> FrameSnapshot:
    """体积只用 XYZ；去掉 ring，让服务里的快照与归档恢复出来的逐字节相同。"""
    snap = frame if isinstance(frame, FrameSnapshot) else FrameSnapshot.freeze(frame)
    return FrameSnapshot(snap.xyz_bytes, None, snap.seq, snap.ts, snap.gen, snap.packets, snap.dropped_packets)


def frame_record(snapshot: FrameSnapshot) -> dict:
    return dict(snapshot.identity(), dropped_packets=int(snapshot.dropped_packets),
                points=len(snapshot.xyz_bytes) // 12,
                sha256=hashlib.sha256(snapshot.xyz_bytes).hexdigest())


def check_frames(frames: Sequence[FrameSnapshot]) -> Tuple[FrameSnapshot, ...]:
    frames = tuple(frames)
    if len(frames) > ENTER_MAX_FRAMES or any(not isinstance(f, FrameSnapshot) for f in frames):
        raise ValueError("进场帧数量超过预算或不是冻结快照")
    if sum(len(f.xyz_bytes) // 12 for f in frames) > ENTER_MAX_POINTS:
        raise ValueError("进场帧点数超过预算")
    if any(f.dropped_packets for f in frames):
        raise ValueError("进场帧含残缺帧")
    if any(a.seq >= b.seq or a.ts >= b.ts or a.gen != b.gen for a, b in zip(frames, frames[1:])):
        raise ValueError("进场帧乱序、重复或跨源代次")
    return frames


def register_for_volume(loaded, frames: Sequence[FrameSnapshot]) -> Tuple[Optional[tuple], Optional[dict]]:
    """返回 (给 compute_volume 的 enter_extra 或 None, 记录或 None)。没有帧时两者都是 None。

    frames 必须是「时间序最早在前、最后一帧贴近停稳」——进场采集本身如此；
    离场见 register_exit_for_volume（先校验采集序，再 reverse 配准）。
    """
    return _register_checked(loaded, check_frames(frames))


def register_exit_for_volume(loaded, frames: Sequence[FrameSnapshot]) -> Tuple[Optional[tuple], Optional[dict]]:
    """离场帧：采集序最早/近端在前；配准前 reverse。记录 frames/shifts 仍按采集序，便于归档往返。"""
    frames = check_frames(frames)
    if not frames:
        return None, None
    # reverse 后时序倒序，不能再走 check_frames；配准链只要求「末帧贴停稳」。
    # 车向前离场（2026-11-02 现场确认）：离场帧在停稳位置的 +y 一侧，配准位移为负。
    clouds, record = _register_checked(loaded, tuple(reversed(frames)), direction=-1)
    if record is None:
        return None, None
    shifts = list(reversed(record["shifts"])) if record["shifts"] else []
    return clouds, dict(record, frames=[frame_record(f) for f in frames], shifts=shifts)


def _register_checked(loaded, frames: Tuple[FrameSnapshot, ...],
                      direction: int = 1) -> Tuple[Optional[tuple], Optional[dict]]:
    if not frames:
        return None, None
    record = dict(schema=ENTER_VOLUME_SCHEMA, status="unresolved", reason="", used=0, shifts=[],
                  frames=[frame_record(f) for f in frames])
    if len(frames) < 2:
        return None, dict(record, reason="进场帧不足 2 帧")
    if loaded is None or getattr(loaded, "source_rotation", None) is None:
        return None, dict(record, reason="载货拟合未成功，没有主货面旋转")
    from .detect import register_enter_frames
    surfaces = align_extra_surfaces(loaded.source_rotation, [f.frame().xyz for f in frames])
    reg = register_enter_frames(loaded, surfaces, direction=direction)
    record.update(status=reg.status, reason=reg.reason, used=int(reg.used),
                  shifts=[float(s) for s in reg.shifts])
    return (tuple(reg.clouds) if reg.status == "resolved" and reg.clouds else None), record


def frames_bytes(frames: Sequence[FrameSnapshot]) -> bytes:
    """全部进场帧按时间顺序首尾相接的原 XYZ 字节（每点 3 个 <f4）；分界由记录里的 points 给出。

    不用 .npy：归档读取器把所有 .npy 成员都按行索引数组解析。
    """
    return b"".join(f.xyz_bytes for f in check_frames(frames))


def validate_record(record) -> dict:
    if type(record) is not dict or set(record) != _RECORD_KEYS or record["schema"] != ENTER_VOLUME_SCHEMA:
        raise ValueError("进场帧体积记录字段无效")
    if record["status"] not in ("resolved", "unresolved") or type(record["reason"]) is not str:
        raise ValueError("进场帧配准状态非法")
    if type(record["used"]) is not int or record["used"] < 0 or type(record["shifts"]) is not list:
        raise ValueError("进场帧配准结果非法")
    if any(type(s) not in (int, float) for s in record["shifts"]):
        raise ValueError("进场帧位移非法")
    frames = record["frames"]
    if type(frames) is not list or not 0 < len(frames) <= ENTER_MAX_FRAMES:
        raise ValueError("进场帧数量非法")
    for item in frames:
        if type(item) is not dict or set(item) != _FRAME_KEYS:
            raise ValueError("进场帧身份字段无效")
        if (type(item["points"]) is not int or item["points"] < 0 or type(item["sha256"]) is not str
                or len(item["sha256"]) != 64 or any(c not in "0123456789abcdef" for c in item["sha256"])):
            raise ValueError("进场帧点数或摘要非法")
    if sum(item["points"] for item in frames) > ENTER_MAX_POINTS:
        raise ValueError("进场帧点数超过预算")
    return record


def restore_frames(record: dict, data: bytes) -> Tuple[FrameSnapshot, ...]:
    """从归档成员恢复原进场帧；任何一帧摘要、点数或身份对不上都拒绝。"""
    validate_record(record)
    if type(data) is not bytes or len(data) != 12 * sum(item["points"] for item in record["frames"]):
        raise ValueError("进场帧成员字节数与记录点数不符")
    out, start = [], 0
    for item in record["frames"]:
        chunk = data[start:start + 12 * item["points"]]
        start += 12 * item["points"]
        if not np.isfinite(np.frombuffer(chunk, dtype="<f4")).all():
            raise ValueError("进场帧含非有限坐标")
        if hashlib.sha256(chunk).hexdigest() != item["sha256"]:
            raise ValueError("进场帧内容摘要不一致")
        out.append(FrameSnapshot(chunk, None, item["seq"], item["ts"], item["gen"], item["packets"],
                                 item["dropped_packets"]))
    return check_frames(out)
