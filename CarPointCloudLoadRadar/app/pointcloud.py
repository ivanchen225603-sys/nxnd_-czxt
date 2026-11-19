#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pointcloud.py — 点云数据结构与磁盘格式

.xyz 磁盘格式（二进制，逆向确认，10/10 历史样本吻合）：

    ┌──────────────┬──────────────────────────────────────────────┐
    │ uint32  N    │ N × { float32 x, float32 y, float32 z,        │
    │ (4 字节)      │       uint16 ring }  = 14 字节/点            │
    └──────────────┴──────────────────────────────────────────────┘

    总长度必须满足  4 + N×14 == 文件大小
    ring = 128 线激光的通道号（0..127；跨 block 时实测可达 187）

本模块只做「读写 + 裁剪 + 导出」，不含任何算法，方便单独测试与替换。
"""
from __future__ import annotations

import math
import os
import struct
import hashlib
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

XYZ_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("ring", "<u2")])
XYZ_HEADER = 4
XYZ_STRIDE = 14

# 原厂默认 ROI（来自 config.json，保留为兜底）
DEFAULT_ROI = ((-2.0, -11.0, -6.0), (2.0, 8.0, -0.75))


class XYZFormatError(ValueError):
    """点云文件格式不符 —— 现场最常见的原因是拷进来的其实是文本 xyz。"""


@dataclass
class Frame:
    """
    一帧点云。

    xyz   : (N,3) float32  —— 车体坐标系，单位米
    ring  : (N,)  uint16   —— 激光通道号，仅用于诊断与重投影
    seq   : int            —— 帧序号（回放/接收时自增）
    ts    : float          —— 采集时刻（unix 秒），0 表示未知
    gen   : int            —— 数据源"场景代次"。仿真源在切换场景时自增，
                             用来让上层识别"这一帧属于新场景"。
                             真雷达永远是 0，不受影响。
    packets: int           —— 本帧有效 UDP 报文数（0 = 未知/非实时源）。
                             整圈切帧时它应当稳定在"一圈的包数"附近；明显偏少
                             说明这一圈**缺了一整块扇形**（突发丢包），
                             此时整帧质心必然偏移 —— 停稳判据要据此把它排除，
                             否则车停着也会被判成"又动了"。见 service.ChannelWorker。
    dropped_packets: int   —— 本批解码已知丢弃的报文数；非零时不得认作完整帧，
                             即使刚启动、尚未建立每圈包数参考也不能放行。
    """
    xyz: np.ndarray
    ring: Optional[np.ndarray] = None
    seq: int = 0
    ts: float = 0.0
    gen: int = 0
    packets: int = 0
    dropped_packets: int = 0

    def __post_init__(self):
        self.xyz = np.ascontiguousarray(self.xyz, dtype=np.float32).reshape(-1, 3)
        if self.ring is not None:
            self.ring = np.ascontiguousarray(self.ring, dtype=np.uint16).reshape(-1)
            if len(self.ring) != len(self.xyz):
                self.ring = None

    # ── 基本属性 ─────────────────────────────────────────────
    @property
    def n(self) -> int:
        return int(self.xyz.shape[0])

    def centroid(self) -> np.ndarray:
        """质心。停稳检测的核心量（已验证：dev = ‖质心差‖）。"""
        return self.xyz.mean(axis=0) if self.n else np.zeros(3, np.float32)

    def bbox(self) -> Tuple[np.ndarray, np.ndarray]:
        if not self.n:
            z = np.zeros(3, np.float32)
            return z, z
        return self.xyz.min(axis=0), self.xyz.max(axis=0)

    def extent(self) -> np.ndarray:
        lo, hi = self.bbox()
        return hi - lo

    def slice_z(self, lo: float, hi: float) -> "Frame":
        m = (self.xyz[:, 2] >= lo) & (self.xyz[:, 2] <= hi)
        return self._mask(m)

    # ★ 派生新帧时身份、有效包数和已知解码损失必须一起保留。
    #   漏掉 packets 的后果是静默的：ROI 裁剪之后停稳判据就再也看不到
    #   "这一圈有多少报文"，残缺帧识别直接失效（而且不报任何错）。
    #   钉住这条：tests/test_pointcloud.py::TestFrameFieldsSurviveDerivation
    def _mask(self, m: np.ndarray) -> "Frame":
        return Frame(self.xyz[m], self.ring[m] if self.ring is not None else None,
                     self.seq, self.ts, self.gen, self.packets, self.dropped_packets)

    def transformed(self, R: np.ndarray, t: Optional[np.ndarray] = None) -> "Frame":
        p = self.xyz @ R.T
        if t is not None:
            p = p + t
        return Frame(p, self.ring, self.seq, self.ts, self.gen, self.packets,
                     self.dropped_packets)

    def summary(self) -> str:
        lo, hi = self.bbox()
        d = hi - lo
        c = self.centroid()
        return (f"{self.n} 点  bbox x[{lo[0]:.2f},{hi[0]:.2f}] y[{lo[1]:.2f},{hi[1]:.2f}] "
                f"z[{lo[2]:.2f},{hi[2]:.2f}]  尺寸 {d[0]:.2f}×{d[1]:.2f}×{d[2]:.2f}  "
                f"质心 ({c[0]:.3f},{c[1]:.3f},{c[2]:.3f})")


# ─────────────────────────────────────────────────── 读写

def load_xyz(path: str) -> Frame:
    """读取原厂 .xyz（二进制）。格式不符时抛 XYZFormatError 并给出可操作提示。"""
    size = os.path.getsize(path)
    if size < XYZ_HEADER:
        raise XYZFormatError(f"{path}: 文件仅 {size} 字节，不是有效点云")
    with open(path, "rb") as f:
        head = f.read(XYZ_HEADER)
        n = struct.unpack("<I", head)[0]
        body = f.read()
    if len(body) != n * XYZ_STRIDE:
        raise XYZFormatError(
            f"{path}: 格式不符 —— 头部声明 {n} 点（应为 {XYZ_HEADER + n*XYZ_STRIDE} 字节），"
            f"实际 {size} 字节。若这是文本 xyz，请确认现场导出的是二进制格式。")
    arr = np.frombuffer(body, dtype=XYZ_DTYPE, count=n)
    xyz = np.empty((n, 3), np.float32)
    xyz[:, 0] = arr["x"]
    xyz[:, 1] = arr["y"]
    xyz[:, 2] = arr["z"]
    return Frame(xyz, np.ascontiguousarray(arr["ring"]),
                 ts=os.path.getmtime(path))


def save_xyz(path: str, frame: Frame) -> None:
    """写回同格式 .xyz（先写临时文件再改名，避免采集过程中被读到半截文件）。"""
    out = np.empty(frame.n, dtype=XYZ_DTYPE)
    out["x"] = frame.xyz[:, 0]
    out["y"] = frame.xyz[:, 1]
    out["z"] = frame.xyz[:, 2]
    out["ring"] = frame.ring if frame.ring is not None else 0
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(struct.pack("<I", frame.n))
        f.write(out.tobytes())
    os.replace(tmp, path)


def save_reason(path: str, reason: str) -> None:
    """写 reason.txt —— 与点云同名同目录，保持与原版一致。"""
    with open(path, "w", encoding="utf-8") as f:
        f.write(reason)


# reason.txt 的解码顺序。本项目自己写的是 UTF-8（见 save_reason），
# 但原厂那套是 GBK 系的，现场交接过来的旧文件、以及麒麟上 locale 不对时
# 落的盘，都可能不是 UTF-8。
#
# 为什么必须容错：`cli/verify.py`（回归闸门）、`cli/replay.py`、
# `cli/inspect_cloud.py` 三个都调 load_reason，以前写死 utf-8 ——
# 喂一个 GBK 文件就是 `UnicodeDecodeError` 直接把闸门打挂
# （2026-09-17 审查 docs/14 §2.1 复现过）。
_REASON_ENCODINGS = ("utf-8-sig", "utf-8", "gbk")


def load_reason(xyz_path: str) -> str:
    """读 reason.txt。按 utf-8 -> gbk 依次试，都不行就用替换字符兜底。

    ▲ **合法 UTF-8 文件的读出内容与从前逐字相同** —— 第一顺位仍是 utf-8，
      所以 `verify.py` 的逐字比对不受影响（utf-8-sig 排在最前只是为了
      顺手吃掉 BOM，无 BOM 时它与 utf-8 完全等价）。
    ▲ 兜底用 errors="replace" 而不是抛异常：reason 只是给人看的说明，
      读不干净也不该让整个回归/回放崩掉；坏字符会显示成 U+FFFD，一眼看得出。
    """
    p = xyz_path + ".reason.txt"
    if not os.path.exists(p):
        return ""
    with open(p, "rb") as f:
        raw = f.read()
    for enc in _REASON_ENCODINGS:
        try:
            return raw.decode(enc).strip()
        except UnicodeDecodeError:
            continue
    return raw.decode("utf-8", errors="replace").strip()


def file_sha256(path: str, chunk: int = 1 << 20) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


# ─────────────────────────────────────────────────── 角度标定表

class AngleCache:
    """
    逐通道角度修正表（原 jt128.dll 的 load_angle_csv）。

    ★ 文件真实结构（本轮新确认，原厂「固定长度记录」格式，以现场文件实测）
        偏移 0      [表头]            Channel,Elevation,Azimuth
        偏移 …      [128 行通道数据]   1,-5.452,0.130 … 128,88.973,-322.707
        偏移 2321   [SHA-256 校验行]   64 位十六进制
        偏移 2387   [0xFF 填充]       一直填到 4226 字节（固定长度）
        注：文件行尾为 CRLF，但**校验值按 LF 口径计算** —— 即把
            「表头 + 128 行数据 + 末尾换行」的行尾统一成 \\n 后再做 SHA-256。
            这是最容易踩的坑：按整个文件算、或按 CRLF 算，都永远对不上。

    退化行为：文件缺失或行数不足时，返回全零修正，并在日志里明确告警
    （宁可点云偏一点，也不要让服务起不来）。
    """

    def __init__(self, path: str = ""):
        self.path = path
        self.elevation = np.zeros(256, np.float32)
        self.azimuth = np.zeros(256, np.float32)
        self.channels_seen = 0
        self.loaded = False
        self.expected_hash = ""
        self.actual_hash = ""
        self.error = ""

    def load(self, path: Optional[str] = None) -> "AngleCache":
        p = path or self.path
        self.path = p
        if not p or not os.path.exists(p):
            self.error = f"角度标定表不存在: {p}"
            return self

        with open(p, "rb") as f:
            raw = f.read()

        # 尾部 0xFF 填充不属于逻辑内容，先切掉
        ff = raw.find(b"\xff")
        body_bytes = raw[:ff] if ff >= 0 else raw
        text = body_bytes.decode("utf-8", errors="replace")

        logical: List[str] = []      # 逻辑内容：表头 + 通道行 + 校验行
        for line in text.split("\n"):
            s = line.rstrip("\r").strip()
            if s:
                logical.append(s)
        if not logical:
            self.error = f"角度标定表为空: {p}"
            return self
        if "Channel" not in logical[0]:
            self.error = f"角度标定表缺表头（首行: {logical[0][:32]}）"

        hash_line = ""
        rows: List[str] = []
        for s in logical[1:]:
            parts = s.split(",")
            ok = False
            if len(parts) == 3:
                try:
                    ch, e, a = int(parts[0]), float(parts[1]), float(parts[2])
                    # ★ 非有限值必须挡在加载这一关（2026-09-23，低严重度批次）：
                    #   nan/inf 的修正会一路穿透成 NaN 点云 —— 质心变 NaN、
                    #   停稳判据全废，而且 valid_mask 只查距离不查有限值，
                    #   现场只能看到"车永远判不稳"却查不出原因。
                    ok = math.isfinite(e) and math.isfinite(a)
                except ValueError:
                    ok = False
            if ok:
                if 0 < ch <= len(self.elevation):
                    self.elevation[ch - 1] = e
                    self.azimuth[ch - 1] = a
                    self.channels_seen += 1
                rows.append(s)
            elif len(s) >= 64 and all(c in "0123456789abcdefABCDEF" for c in s[:64]):
                hash_line = s[:64].lower()
                break
            else:
                break

        # ★ 校验口径：表头 + 通道行，LF 归一化 + 末尾换行
        payload = ("\n".join([logical[0]] + rows) + "\n").encode("utf-8")
        self.actual_hash = hashlib.sha256(payload).hexdigest()
        self.expected_hash = hash_line

        self.loaded = True
        if self.expected_hash and self.expected_hash != self.actual_hash:
            self.error = (f"角度标定表校验值不符（文件内 {self.expected_hash[:16]}…，"
                          f"按表头+{self.channels_seen} 行数据实测 {self.actual_hash[:16]}…）"
                          f"—— 疑似被改动，请核对")
        elif not self.expected_hash and self.channels_seen:
            self.actual_hash = ""
        if self.channels_seen < 128 and not self.error:
            self.error = f"角度标定表只解析到 {self.channels_seen} 个通道（期望 128）"
        return self

    @property
    def hash_ok(self) -> bool:
        return (not self.expected_hash) or self.expected_hash == self.actual_hash

    def check(self) -> str:
        if not self.loaded:
            return f"× {self.error or '未加载'}"
        base = (f"√ 已加载 {self.channels_seen} 个通道修正"
                f"（俯仰 {self.elevation.min():.2f}°..{self.elevation.max():.2f}°，"
                f"校验值 {'匹配' if self.hash_ok else '不匹配'}）")
        if self.error:
            base += f"\n▲  {self.error}"
        return base


# ─────────────────────────────────────────────────── ROI

def normalize_roi(roi) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """
    把 ROI 统一成 (下界 xyz, 上界 xyz)。

    兼容三种写法（历史上都出现过，必须都吃下）：
        按角点：((x0,y0,z0), (x1,y1,z1))        ← 本项目规范形式
        按坐标轴：((x0,x1), (y0,y1), (z0,z1))   ← 顺序无关的轴区间写法
        按轴名：{"x": [x0,x1], "y": [...], "z": [...]}  ← config.json 里的原始写法
    """
    if not roi:
        return None
    # config.json 的原始写法：{"x": [x0,x1], "y": [y0,y1], "z": [z0,z1]}
    # settings.load_calib 已经会转，但这里也认，方便直接把原始 dict 传进来。
    if isinstance(roi, dict):
        try:
            roi = (tuple(roi["x"]), tuple(roi["y"]), tuple(roi["z"]))
        except KeyError as e:
            raise ValueError(f"ROI 字典缺少轴 {e}：{roi!r}") from None
    a, b = roi[0], roi[1]
    if len(roi) == 2 and len(a) == 3 and len(b) == 3:
        lo = np.minimum(np.asarray(a, np.float32), np.asarray(b, np.float32))
        hi = np.maximum(np.asarray(a, np.float32), np.asarray(b, np.float32))
        return lo, hi
    if len(roi) == 3 and all(len(r) == 2 for r in roi):
        # ★ 必须 min/max 归一化：轴区间写反（例如 z 写成 [-0.75, -6]）时，
        #   lo 会大于 hi，apply_roi 的掩码恒为 False → **一个点都不剩，且不报错**。
        #   现场表现是"雷达在出数但点云永远是空的"，极难定位。
        a = np.array([roi[0][0], roi[1][0], roi[2][0]], np.float32)
        b = np.array([roi[0][1], roi[1][1], roi[2][1]], np.float32)
        return np.minimum(a, b), np.maximum(a, b)
    raise ValueError(f"无法识别的 ROI 写法: {roi!r}")


def roi_arrays(roi) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    return normalize_roi(roi)


def roi_as_ranges(roi):
    """转回 config.json 的按轴写法 ((x0,x1),(y0,y1),(z0,z1))，便于写回配置。"""
    n = normalize_roi(roi)
    if n is None:
        return None
    lo, hi = n
    return ((float(lo[0]), float(hi[0])), (float(lo[1]), float(hi[1])),
            (float(lo[2]), float(hi[2])))


def apply_roi(frame: Frame, roi) -> Frame:
    """
    按 ROI 裁剪。原版是在雷达取帧后、送检测前做的。
    ROI 之外的点直接丢弃 —— 这是第一道也是最有效的降噪手段。
    """
    a = roi_arrays(roi)
    if a is None:
        return frame
    lo, hi = a
    m = np.all((frame.xyz >= lo) & (frame.xyz <= hi), axis=1)
    return frame._mask(m)


# ─────────────────────────────────────────────────── 调试导出

def export_ply(frame: Frame, path: str, color_by_ring: bool = True) -> None:
    """
    导出 ASCII PLY —— 用 MeshLab / CloudCompare 打开看，是现场排查的第一手段。
    颜色按 ring 通道上色，一眼就能看出是哪几线在起作用。
    """
    n = frame.n
    lines = [
        "ply", "format ascii 1.0", f"element vertex {n}",
        "property float x", "property float y", "property float z",
        "property uchar red", "property uchar green", "property uchar blue",
        "end_header",
    ]
    if color_by_ring and frame.ring is not None and n:
        r = frame.ring.astype(np.float32)
        rng = max(float(r.max() - r.min()), 1.0)
        t = (r - r.min()) / rng
        col = np.stack([(t * 255), ((1 - t) * 255), np.full(n, 128.0)], axis=1).astype(int)
    else:
        col = np.tile(np.array([[200, 200, 200]]), (n, 1))
    # ★ 逐点 f-string 拼字符串在 6 万点上要一秒左右，而这个函数是 /api/export/ply
    #   的处理线程在跑（现场点一下导出，看板会卡住）。改用 np.savetxt 一次写完。
    with open(path, "w", encoding="ascii", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
        if n:
            np.savetxt(f, np.column_stack([frame.xyz.astype(np.float64), col]),
                       fmt="%.4f %.4f %.4f %d %d %d")


def export_csv(frame: Frame, path: str, limit: int = 0) -> None:
    n = frame.n if not limit else min(limit, frame.n)
    with open(path, "w", encoding="utf-8") as f:
        f.write("x,y,z,ring\n")
        for i in range(n):
            x, y, z = frame.xyz[i]
            r = int(frame.ring[i]) if frame.ring is not None else 0
            f.write(f"{x:.4f},{y:.4f},{z:.4f},{r}\n")


def make_synthetic_box(dx=2.4, dy=12.0, dz=2.6, n=60000, seed=0,
                       cargo=None, noise=0.01) -> Frame:
    """
    合成一个「空车厢」点云，用于离线自测与阈值演练。

    参数
        dx, dy, dz : 车厢内空尺寸（米）
        cargo      : None 或 (宽, 长, 高, 高起位置z)，在车厢底部放一个货堆
        noise      : 墙面厚度方向的噪声（米）

    这是现场没雷达时唯一能做端到端演练的手段，也能用来给客户演示。
    """
    rng = np.random.default_rng(seed)
    half = np.array([dx / 2, dy / 2, 0.0])
    z_floor, z_roof = 0.0, dz
    faces = []

    def rect(npts, axis, val):
        """在一个面上均匀撒点。axis: 0=x墙, 1=y墙, 2=地面/顶面"""
        if axis == 2:
            p = np.stack([rng.uniform(-half[0], half[0], npts),
                          rng.uniform(-half[1], half[1], npts),
                          np.full(npts, val)], axis=1)
        elif axis == 0:
            p = np.stack([np.full(npts, val),
                          rng.uniform(-half[1], half[1], npts),
                          rng.uniform(z_floor, z_roof, npts)], axis=1)
        else:
            p = np.stack([rng.uniform(-half[0], half[0], npts),
                          np.full(npts, val),
                          rng.uniform(z_floor, z_roof, npts)], axis=1)
        faces.append(p)

    per = n // 6
    rect(per, 2, z_floor)            # 底
    rect(per, 2, z_roof)             # 顶
    rect(per, 0, -half[0])           # 左壁
    rect(per, 0, half[0])            # 右壁
    rect(per, 1, -half[1])           # 前壁
    rect(per, 1, half[1])            # 后壁

    if cargo is not None:
        cw, cl, ch, cz = cargo
        nc = per
        # 货堆：顶面 + 四个侧面
        faces.append(np.stack([rng.uniform(-cw / 2, cw / 2, nc),
                               rng.uniform(-cl / 2, cl / 2, nc),
                               np.full(nc, cz + ch)], axis=1))
        for axis, val in ((0, -cw / 2), (0, cw / 2), (1, -cl / 2), (1, cl / 2)):
            if axis == 0:
                faces.append(np.stack([np.full(nc, val),
                                       rng.uniform(-cl / 2, cl / 2, nc),
                                       rng.uniform(cz, cz + ch, nc)], axis=1))
            else:
                faces.append(np.stack([rng.uniform(-cw / 2, cw / 2, nc),
                                       np.full(nc, val),
                                       rng.uniform(cz, cz + ch, nc)], axis=1))

    pts = np.concatenate(faces, axis=0).astype(np.float32)
    pts += rng.normal(0, noise, pts.shape).astype(np.float32)
    ring = rng.integers(0, 128, len(pts)).astype(np.uint16)
    return Frame(pts, ring)
