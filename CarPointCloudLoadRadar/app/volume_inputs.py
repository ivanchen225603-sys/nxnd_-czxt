"""体积主/额外货面的统一坐标和真实观测映射，不产生物理条件或基线授权。"""
from __future__ import annotations

from typing import Sequence, Tuple
import numpy as np

from .cloud_observation import ObservationWindow


def align_extra_surfaces(rotation, extras: Sequence[np.ndarray]) -> Tuple[np.ndarray, ...]:
    """只用主货面实际应用的旋转；不能从yaw猜矩阵或对每个extra独立拟合。"""
    source = np.asarray(rotation)
    if source.dtype.kind not in "fiu":
        raise ValueError("体积旋转必须为实数")
    matrix = np.asarray(source, dtype=np.float64)
    if (matrix.shape != (3, 3) or not np.isfinite(matrix).all() or
            not np.allclose(matrix @ matrix.T, np.eye(3), rtol=0., atol=1e-9) or
            not np.isclose(np.linalg.det(matrix), 1., rtol=0., atol=1e-9)):
        raise ValueError("体积缺少主货面的实际正交旋转")
    values = []
    with np.errstate(over="raise", invalid="raise"):
        for extra in extras:
            source = np.asarray(extra)
            if source.dtype.kind not in "fiu":
                raise ValueError("额外货面必须为实数XYZ")
            cloud = np.asarray(source, dtype=np.float64)
            if cloud.ndim != 2 or cloud.shape[1] != 3 or not np.isfinite(cloud).all():
                raise ValueError("额外货面必须为有限XYZ")
            value = cloud @ matrix.T
            # 不可变拥有字节，不保留可被采集线程复用的源数组。
            values.append(np.frombuffer(value.astype("<f8").tobytes(), dtype="<f8").reshape(-1, 3))
    return tuple(values)


def surface_lineage(primary: ObservationWindow, support: ObservationWindow, extra_members: tuple):
    """主合帧为第一个surface，其后按extra用途顺序；frame_ids独立保持真实时间顺序。"""
    if not isinstance(primary, ObservationWindow) or not isinstance(support, ObservationWindow):
        raise ValueError("体积缺少实际主窗口或支持窗口")
    if (not isinstance(extra_members, tuple) or
            any(type(i) is not int or not 0 <= i < len(support.pairs) - 1 for i in extra_members) or
            any(a >= b for a, b in zip(extra_members, extra_members[1:]))):
        raise ValueError("体积额外成员映射非法")
    reference = primary.pairs[-1]
    context = (reference.channel, reference.epoch, reference.raw.gen)
    frames = {}
    all_frames = {}
    groups = (primary.pairs,) + tuple((support.pairs[i],) for i in extra_members)
    for pairs in (primary.pairs, support.pairs):
        for pair in pairs:
            if (pair.channel, pair.epoch, pair.raw.gen) != context:
                raise ValueError("体积来源跨通道、活动代次或源代次")
            identity = (pair.raw.gen, pair.raw.seq, pair.raw.ts)
            if identity in all_frames and all_frames[identity] != pair:
                raise ValueError("同一体积帧身份对应不同观测内容")
            all_frames[identity] = pair
    if support.pairs[-1] != reference:
        raise ValueError("体积主/支持窗口末帧不一致")
    for group in groups:
        for pair in group:
            identity = (pair.raw.gen, pair.raw.seq, pair.raw.ts)
            if identity in frames and frames[identity] != pair:
                raise ValueError("同一体积帧身份对应不同观测内容")
            frames[identity] = pair
    identities = tuple(sorted(frames, key=lambda item: item[1]))
    if any(a[1] >= b[1] or a[2] >= b[2] for a, b in zip(identities, identities[1:])):
        raise ValueError("体积帧身份乱序或时间冲突")
    if any(frames[a].received_monotonic >= frames[b].received_monotonic for a, b in zip(identities, identities[1:])):
        raise ValueError("体积帧接收时间乱序")
    index = {identity: i for i, identity in enumerate(identities)}
    mapping = tuple(tuple(index[(p.raw.gen, p.raw.seq, p.raw.ts)] for p in group) for group in groups)
    return identities, mapping
