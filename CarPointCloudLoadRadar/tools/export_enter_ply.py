#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
export_enter_ply.py — 将 JT128 车辆进场点云（.xyz 分片序列）导出为 PLY
================================================================================
用途：把 data/field/enter/<CH>/<批次>_<i>of<N>.xyz 这批"车辆进场连续整帧"
      合并（或逐帧）导出为二进制 PLY，供 CloudCompare / MeshLab / Open3D 打开。

数据格式（与 app/pointcloud.py、tools/pc_tool.py 一致）
------------------------------------------------------
  .xyz = [uint32 count] + count × [float32 x, float32 y, float32 z, uint16 ring]
         头 4 字节 + 每点 14 字节。

输出
----
  --mode merged  ：全部帧合并为一个 PLY（默认）
  --mode per-frame：每帧一个 PLY
  --mode raw | detection：
        raw       = 原始坐标（与 .xyz 逐位一致）—— 默认
        detection = 施加该批次的 yaw 旋转 + ground_z 平移（与系统"检测坐标系"一致）

PLY 字段：x y z (float32) + ring (uint16)。用 binary_little_endian，体积最小、加载最快。

用法
----
  python tools/export_enter_ply.py data/field/enter/CH4/20260926_114850 --out <dir>
"""

import argparse
import glob
import json
import os
import re
import struct
import sys

import numpy as np

# Windows 控制台默认非 UTF-8，中文输出会抛 UnicodeEncodeError 打断脚本（AGENTS.md 硬约束）
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:  # noqa: BLE001
        pass

XYZ_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("ring", "<u2")])
assert XYZ_DTYPE.itemsize == 14, XYZ_DTYPE.itemsize

# PLY 二进制头：x y z 用 float32，ring 用 uint16
PLY_STRUCT = "<fffH"
PLY_REC = struct.calcsize(PLY_STRUCT)
assert PLY_REC == 14, PLY_REC


def read_xyz(path):
    """读取单个 .xyz，返回结构化 numpy 数组（含 ring）。"""
    with open(path, "rb") as f:
        raw = f.read()
    if len(raw) < 4:
        raise ValueError(f"文件过短，不是合法 .xyz: {path}")
    (n,) = struct.unpack_from("<I", raw, 0)
    if len(raw) != 4 + n * XYZ_DTYPE.itemsize:
        n = (len(raw) - 4) // XYZ_DTYPE.itemsize  # 容错：以实际长度为准
    return np.frombuffer(raw, dtype=XYZ_DTYPE, count=n, offset=4)


def transform(arr, meta):
    """把原始坐标变换到"检测坐标系"。

    依据 app/pointcloud.py 的既有约定：绕 Z 轴旋转 effective_yaw_offset_deg，
    再按 ground_z 平移。这里只做刚体变换，不做任何裁剪/抽稀。
    """
    yaw = float(meta.get("effective_yaw_offset_deg",
                         meta.get("channel_yaw_offset_deg", 0.0)))
    gz = float(meta.get("ground_z", 0.0))
    th = np.deg2rad(yaw)
    c, s = np.cos(th), np.sin(th)
    x = arr["x"].astype(np.float64)
    y = arr["y"].astype(np.float64)
    z = arr["z"].astype(np.float64) - gz
    out = np.empty(len(arr), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("ring", "<u2")])
    out["x"] = (x * c - y * s).astype("<f4")
    out["y"] = (x * s + y * c).astype("<f4")
    out["z"] = z.astype("<f4")
    out["ring"] = arr["ring"]
    return out


def write_ply_binary(path, arr, comment=""):
    """写 binary_little_endian PLY（x y z ring）。"""
    n = len(arr)
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        + (f"comment {comment}\n" if comment else "")
        + f"element vertex {n}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property ushort ring\n"
        "end_header\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        # 复用同一个 14B 记录布局，整块写出（比逐点快几十倍）
        buf = bytearray(n * PLY_REC)
        view = np.frombuffer(buf, dtype=XYZ_DTYPE)
        view[:] = arr
        f.write(buf)


def batch_key(filename):
    """从 '20260926_114850_150of150.xyz' 提取批次键 '20260926_114850'。"""
    m = re.match(r"^(\d{8}_\d{6})_\d+of\d+\.xyz$", os.path.basename(filename))
    return m.group(1) if m else None


def frame_index(filename):
    m = re.search(r"_(\d+)of(\d+)\.xyz$", os.path.basename(filename))
    return int(m.group(1)) if m else -1


def main():
    ap = argparse.ArgumentParser(description="车辆进场点云 .xyz -> PLY 导出")
    ap.add_argument("batch_dir", help="分片所在目录（含 <批次>_<i>of<N>.xyz）")
    ap.add_argument("--batch", default=None, help="批次键，如 20260926_114850；默认取目录内最新批次")
    ap.add_argument("--out", default=None, help="输出目录，默认 <batch_dir>/../export_ply/")
    ap.add_argument("--mode", choices=["merged", "per-frame"], default="merged")
    ap.add_argument("--coord", choices=["raw", "detection"], default="raw",
                    help="raw=原始坐标(默认)；detection=施加 yaw+ground_z 变换")
    ap.add_argument("--max-points", type=int, default=0,
                    help=">0 时对合并结果做等间隔抽稀（便于快速预览）")
    args = ap.parse_args()

    d = os.path.abspath(args.batch_dir)
    files = sorted(glob.glob(os.path.join(d, "*_*of*.xyz")), key=frame_index)
    if not files:
        print(f"[错误] 目录内没有 *_<i>of<N>.xyz 分片：{d}")
        return 2

    all_keys = sorted({batch_key(p) for p in files if batch_key(p)})
    key = args.batch or all_keys[-1]
    files = [p for p in files if batch_key(p) == key]
    if not files:
        print(f"[错误] 批次 {key} 无分片。可用批次：{all_keys}")
        return 2

    out_dir = args.out or os.path.join(os.path.dirname(d), "export_ply")
    os.makedirs(out_dir, exist_ok=True)

    print(f"[批次] {key}  通道 {os.path.basename(d)}")
    print(f"[分片] {len(files)} 个，范围 {os.path.basename(files[0])} .. {os.path.basename(files[-1])}")
    print(f"[坐标] {args.coord}    [模式] {args.mode}")

    metas = []
    for p in files:
        mp = p + ".meta.json"
        metas.append(json.load(open(mp, encoding="utf-8")) if os.path.exists(mp) else {})

    total = 0
    if args.mode == "per-frame":
        for p, meta in zip(files, metas):
            arr = read_xyz(p)
            if args.coord == "detection":
                arr = transform(arr, meta)
            op = os.path.join(out_dir, f"{key}_{frame_index(p):03d}of{len(files):03d}.ply")
            write_ply_binary(op, arr, comment=f"{key} frame {frame_index(p)}/{len(files)} {args.coord}")
            total += len(arr)
        print(f"[完成] 写出 {len(files)} 个 PLY -> {out_dir}  共 {total:,} 点")
        return 0

    # merged
    arrs = []
    for p, meta in zip(files, metas):
        a = read_xyz(p)
        if args.coord == "detection":
            a = transform(a, meta)
        arrs.append(a)
    merged = np.concatenate(arrs)
    n_all = len(merged)
    if args.max_points and n_all > args.max_points:
        idx = np.linspace(0, n_all - 1, args.max_points).astype(np.int64)
        merged = merged[idx]
        print(f"[抽稀] {n_all:,} -> {len(merged):,} 点")

    suffix = "" if args.coord == "raw" else "_detection"
    if args.max_points and n_all > args.max_points:
        suffix += f"_sub{args.max_points}"
    op = os.path.join(out_dir, f"{key}_{os.path.basename(d)}{suffix}_merged.ply")
    write_ply_binary(
        op, merged,
        comment=f"CH4/JT128 enter batch {key} frames={len(files)} coord={args.coord}")
    sz = os.path.getsize(op) / 1024 / 1024

    # 几何统计（原始坐标系下更有参照意义）
    xs, ys, zs = merged["x"], merged["y"], merged["z"]
    print(f"[完成] {op}")
    print(f"       点数 {len(merged):,}   文件 {sz:.2f} MB   字段 x,y,z(float32)+ring(uint16)")
    print(f"       x [{xs.min():.3f}, {xs.max():.3f}]  y [{ys.min():.3f}, {ys.max():.3f}]  z [{zs.min():.3f}, {zs.max():.3f}]")

    # 时间跨度
    def ts(i):
        import datetime
        return datetime.datetime.utcfromtimestamp(metas[i]["ts"]) + datetime.timedelta(hours=8)
    if metas and metas[0].get("ts") and metas[-1].get("ts"):
        print(f"       时间 {ts(0):%Y-%m-%d %H:%M:%S.%f}".rstrip("0")
              + " -> " + f"{ts(-1):%Y-%m-%d %H:%M:%S.%f}".rstrip("0"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
