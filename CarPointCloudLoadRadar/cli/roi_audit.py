# -*- coding: utf-8 -*-
"""逐份审查原观测中的可见车厢候选与 ROI 的关系，不证明物理车厢完整性。

用途：落盘的 `.xyz` 是**裁过 ROI 的**，拿它只能间接推断"边缘离边界多远"，
而这个推断在 y 方向并不可靠（车厢远端因掠射角看起来会变窄）。
新归档优先使用主来源窗口的未裁原观测，兼容旧 `service.raw_frames` 整帧。
遮挡或未扫描端面不能从正余量推断为完整；候选始终非权威。

怎么找车厢
----------
沿 y 每 `SLICE` 米切一片，在片内取**最长的连续 x 簇**当作物体的横截面：

  · 簇宽落在 `BOX_W_LO..BOX_W_HI` 之间  -> 这一片像车厢
  · 连续的"像车厢"片段拼成一段，长度够 `MIN_LEN` 米  -> 算一个车厢
  · 中间允许 `MAX_GAP` 米的断口；横向中心跳动超过 `MAX_SHIFT` 就断开

★ **不能用 x 的百分位跨度当宽度。** 一片里若有两个物体（左边一块碎片、
  右边一道门架），百分位跨的是两者之和，会算出 4.4 m 宽的"车厢"。
  必须取**单个连续簇**。2026-09-15 第二版就是这么把门架误判成车厢的。

★ **不能按点数密度找车。** 俯视雷达在天底附近角分辨率极高 ——
  正下方一片能有 2 万点，6 米外同样大小的一片只剩几百。
  按点数找到的是"雷达看得最密的地方"，不是车在哪。
  2026-09-15 第一版就是这么错的：量出车厢只有 5 m 长（实际 11.5 m）。
  所以判据用**几何宽度**，不用点数。

★ y >= 8 一带有**固定结构**（门架/墙）：宽度突跳到 4.4~5.3 m、顶高恒定在 4.00 m，
  而且与车体之间隔着约 1 m 的空隙。宽度上限 + 最短长度 + 断口上限三条一起把它挡掉。
  详见 docs/08 §10.10。

★ **按当前配置复算**（默认开）
  整帧是**用当时那一刻的 yaw 解码之后**存下来的，文件里是笛卡尔坐标，
  不是原始报文 —— 所以改了 `channels.json` 的 `lidar.yaw_offset_deg` 之后
  直接重跑本工具，读到的还是旧角度下的点云，余量一个数都不会变。
  于是本工具改配置之后**根本验证不了配置**，这正是它最主要的用途。

  做法：读同名 `.xyz.meta.json` 里的 `effective_yaw_offset_deg`（存盘时生效的角度），
  与当前配置算出来的生效角度相比，差多少就把点云绕 z 转多少 ——
  这与「当时就按新配置解码」逐点等价（真解码器验证到 2.6e-06 m，
  见 tests/test_roi_audit.py::TestYawRecompute）。
  `--as-captured` 关掉它，审查文件原样。

用法
----
    python3 cli/roi_audit.py                      # 审查 data/field/raw 下全部
    python3 cli/roi_audit.py --date 20260915      # 只看某一天
    python3 cli/roi_audit.py --channel CH2        # 只看某一路
    python3 cli/roi_audit.py --csv out.csv        # 另存 CSV
    python3 cli/roi_audit.py --verbose            # 连每个车厢的包围盒一起打印
    python3 cli/roi_audit.py --as-captured        # 不按当前 yaw 复算，看存盘原样
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import io
import json
import math
import os
from pathlib import Path
import re
import sys
from typing import Optional

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cli import _common                      # noqa: E402
from app.pointcloud import load_xyz          # noqa: E402
from app.settings import load_calib, load_settings, root   # noqa: E402
from app.archive_reader import load_archive               # noqa: E402
from app.sample_paths import Sample, discover_samples     # noqa: E402

SLICE = 0.25        # 纵向切片厚度（米）
BOX_W_LO = 1.6      # 像车厢的最小宽度（车宽 2.55，掠射角下会显窄）
BOX_W_HI = 2.8      # 上限：实测真车厢宽 2.19~2.47 m；
                    # 超过 2.8 说明横向簇里并进了墙（实测并墙后会到 3.2~3.5）
MIN_LEN = 3.0       # 一个车厢至少这么长
MAX_GAP = 1.0       # 段内允许的断口
MIN_PTS = 20        # 一片至少几个点才参与判断
CELL = 0.25         # 横向聚簇的格子
CELL_PTS = 2        # 一格至少几个点算被占据
MAX_SHIFT = 0.4     # 相邻切片横向中心允许的跳动（超过 = 换了个物体）
H_LO = 0.5          # 车的高度带下界（相对地面）：排除地面与低矮杂物
H_HI = 4.2          # 上界：排除天花板（实测有 z 比雷达还高的点）
X_WIN = 3.0         # 横向取样窗口（比 ROI 宽，好看见框外）
SPAN_TOL = 1.0      # 3 帧之间车厢范围允许的差异（超过 = 分段不可信）


def rotate_yaw(a, ddeg):
    """把点云绕 z 转 ddeg 度 —— 等价于「解码时把 yaw_offset_deg 加 ddeg」。

    解码里 x = r*sin(az)、y = r*cos(az)（app/radar_packet.py），az 加 ddeg 即：
        x' =  x*cos(d) + y*sin(d)
        y' = -x*sin(d) + y*cos(d)
    z 不变（俯仰与距离都没动）。
    """
    if abs(ddeg) < 1e-9:
        return a
    d = math.radians(ddeg)
    c, s = math.cos(d), math.sin(d)
    b = a.copy()
    b[:, 0] = a[:, 0] * c + a[:, 1] * s
    b[:, 1] = -a[:, 0] * s + a[:, 1] * c
    return b


def captured_yaw(path):
    """整帧存盘时**生效**的 yaw；没有旁注 json 就返回 None。"""
    try:
        with io.open(path + ".meta.json", encoding="utf-8") as fp:
            v = json.load(fp).get("effective_yaw_offset_deg")
        return float(v) if type(v) in (int, float) and math.isfinite(v) else None
    except Exception:                                    # noqa: BLE001
        return None


def load_frame(path, cur_yaw, recompute=True):
    """读一帧整帧点云，必要时按当前配置的 yaw 复算。

    返回 (点云, 实际补了多少度)。补 0 度表示存盘角度与当前配置一致，
    或者没有旁注 json（这时**不猜**，原样返回）。
    """
    a = np.asarray(load_xyz(path).xyz, dtype=np.float64)
    if not recompute or cur_yaw is None:
        return a, 0.0
    old = captured_yaw(path)
    if old is None or abs(cur_yaw - old) < 1e-9:
        return a, 0.0
    d = cur_yaw - old
    return rotate_yaw(a, d), d


def find_boxes(a, ground_z):
    """在一帧里找出所有车厢，返回 [dict(y0,y1,x0,x1,z_top,slices), ...]。"""
    z = a[:, 2] - ground_z
    m = (z > H_LO) & (z < H_HI) & (np.abs(a[:, 0]) < X_WIN)
    p = a[m]
    if len(p) < 200:
        return []

    y_lo = float(np.floor(p[:, 1].min()))
    y_hi = float(np.ceil(p[:, 1].max()))
    xe = np.arange(-X_WIN, X_WIN + CELL, CELL)
    slices = []
    y = y_lo
    while y < y_hi:
        t = p[(p[:, 1] >= y) & (p[:, 1] < y + SLICE)]
        y += SLICE
        if len(t) < MIN_PTS:
            continue
        # ★ 取这一片里**最长的连续 x 簇**，不要用百分位跨度。
        #   百分位会把"左边一个窄碎片 + 右边一个门架"算成一个 4.4 m 宽的东西，
        #   2026-09-15 第二版就是这么把门架误判成车厢的。
        h, _ = np.histogram(t[:, 0], bins=xe)
        occ = h >= CELL_PTS
        best, cur_i = None, None
        for i, o in enumerate(list(occ) + [False]):
            if o and cur_i is None:
                cur_i = i
            elif not o and cur_i is not None:
                if best is None or (i - cur_i) > (best[1] - best[0]):
                    best = (cur_i, i)
                cur_i = None
        if best is None:
            continue
        # ★ 用簇内**实际点**的跨度，不要用格子边界 ——
        #   格子会向外多算最多 1 格（0.25 m），两边合计 0.5 m。
        #   实测真车厢 2.19~2.47 m，按格子边界量会变成 2.75，
        #   跟"并进了墙"的 3.25 挤在一起，宽度上限就没法定了。
        sel = t[(t[:, 0] >= xe[best[0]]) & (t[:, 0] < xe[best[1]])]
        if len(sel) < MIN_PTS:
            continue
        lo, hi = float(sel[:, 0].min()), float(sel[:, 0].max())
        if BOX_W_LO <= hi - lo <= BOX_W_HI:
            slices.append((y - SLICE, lo, hi, float((t[:, 2] - ground_z).max())))

    # 分段：断口过大、或**横向中心跳变**都要断开。
    # 中心跳变这一条挡掉"车旁边的墙恰好也有车厢宽度"的情况 ——
    # 真车厢从一片到下一片中心只挪几厘米，墙与车之间会差一两米。
    runs, cur = [], []
    for s in slices:
        if cur:
            gap = s[0] - cur[-1][0]
            jump = abs((s[1] + s[2]) / 2 - (cur[-1][1] + cur[-1][2]) / 2)
            if gap > MAX_GAP or jump > MAX_SHIFT:
                runs.append(cur)
                cur = []
        cur.append(s)
    if cur:
        runs.append(cur)

    out = []
    for r in runs:
        y0, y1 = r[0][0], r[-1][0] + SLICE
        if y1 - y0 < MIN_LEN:
            continue
        out.append(dict(y0=float(y0), y1=float(y1),
                        x0=float(min(s[1] for s in r)),
                        x1=float(max(s[2] for s in r)),
                        z_top=float(max(s[3] for s in r)),
                        slices=len(r)))
    return out


def main_box(boxes):
    """哪个才是**被测**的那台车。

    磅秤在雷达正下方，被测车必然跨过 y=0；其它段是过路车辆或结构。
    实测依据：同一辆车的 3 帧里，被测车厢的包围盒几乎不动，
    而"第二个车厢"会在帧间平移半米、甚至整个消失 —— 那是在开的车。
    """
    if not boxes:
        return None
    for i, b in enumerate(boxes):
        if b["y0"] <= 0.0 <= b["y1"]:
            return i
    return min(range(len(boxes)),
               key=lambda i: min(abs(boxes[i]["y0"]), abs(boxes[i]["y1"])))


def check(boxes, roi):
    """逐个车厢比 ROI，返回 (是否全部在内, [越界描述], 最小余量)。"""
    (rx0, rx1), (ry0, ry1) = roi
    bad, margin = [], None
    for i, b in enumerate(boxes, 1):
        ms = [b["x0"] - rx0, rx1 - b["x1"], b["y0"] - ry0, ry1 - b["y1"]]
        margin = min(ms) if margin is None else min(margin, min(ms))
        why = []
        if b["x0"] < rx0:
            why.append("左 %+.2f" % (b["x0"] - rx0))
        if b["x1"] > rx1:
            why.append("右 %+.2f" % (rx1 - b["x1"]))
        if b["y0"] < ry0:
            why.append("后 %+.2f" % (b["y0"] - ry0))
        if b["y1"] > ry1:
            why.append("前 %+.2f" % (ry1 - b["y1"]))
        if why:
            bad.append("车厢%d: %s" % (i, "、".join(why)))
    return (not bad), bad, margin


@dataclass(frozen=True)
class AuditGroup:
    channel: str
    key: str
    sample: Optional[Sample] = None
    files: tuple = ()


def discover_audit_groups(settings, directory="", date="", channel=""):
    """输出根去重，归档按逻辑测量发现，旧 raw 仅接受明确命名与目录。"""
    selected = [c for c in settings.channels if not channel or c.id == channel]
    roots = sorted({os.path.normcase(os.path.abspath(directory or c.abs_output_dir)):
                    os.path.abspath(directory or c.abs_output_dir) for c in selected}.values())
    known = {c.id for c in selected}
    packages, conflicts, legacy, errors = {}, set(), {}, []
    allowed = {}
    for c in selected:
        base = os.path.abspath(directory or c.abs_output_dir)
        allowed[c.id] = {os.path.normcase(os.path.abspath(p)) for p in
                         ((base, os.path.join(base, c.id), os.path.join(base, "raw", c.id))
                          if directory else (os.path.join(base, "raw", c.id),))}
    for directory_root in roots:
        if not os.path.exists(directory_root) and not directory:
            continue
        found = discover_samples(directory_root, role="observation")
        errors.extend(found.errors)
        for sample in found.samples:
            if sample.archived:
                if channel and sample.channel != channel:
                    continue
                if sample.channel not in known:
                    errors.append("归档通道没有当前配置: " + sample.channel)
                    continue
                try:
                    captured_date = datetime.fromtimestamp(sample.acquired_at).strftime("%Y%m%d")
                except (ValueError, OSError, OverflowError):
                    errors.append("归档采集日期无法转换: " + sample.path)
                    continue
                identity = sample.channel, sample.measurement_id
                previous = packages.get(identity)
                if previous is not None and previous.manifest_sha256 != sample.manifest_sha256:
                    conflicts.add(identity)
                    errors.append("跨目录归档身份冲突: %s/%s" % identity)
                packages.setdefault(identity, sample)
                # 日期在去重冲突检查后统一过滤，不能用日期藏掉同身份不同内容。
            else:
                parent = os.path.normcase(os.path.abspath(os.path.dirname(sample.path)))
                name = os.path.basename(sample.path)
                for cid in known:
                    match = re.fullmatch(re.escape(cid) + r"_(\d{8})_(.+)_(\d+)of(\d+)\.xyz", name)
                    if parent not in allowed[cid] or match is None or date and match[1] != date:
                        continue
                    key = name.rsplit("_", 1)[0]
                    legacy.setdefault((cid, parent, key), {})[os.path.normcase(os.path.abspath(sample.path))] = sample.path
    groups = []
    for identity, sample in sorted(packages.items()):
        if identity in conflicts:
            continue
        if date and datetime.fromtimestamp(sample.acquired_at).strftime("%Y%m%d") != date:
            continue
        groups.append(AuditGroup(sample.channel, sample.measurement_id, sample))
    for (cid, parent, key), paths in sorted(legacy.items()):
        files = tuple(sorted(paths.values(), key=lambda path: int(Path(path).stem.rsplit("_", 1)[1].split("of")[0])))
        slots = [tuple(int(n) for n in Path(path).stem.rsplit("_", 1)[1].split("of")) for path in files]
        if not slots or any(total != len(slots) for _, total in slots) or [i for i, _ in slots] != list(range(1, len(slots) + 1)):
            errors.append("旧 raw 分组成员不完整或重复: " + os.path.join(parent, key))
            continue
        groups.append(AuditGroup(cid, os.path.join(parent, key), files=files))
    return groups, errors


def audit_group(group, info, *, as_captured=False, window="primary"):
    """逐原帧审计；不拿其它角色副本凑帧，不把候选边界当物理真值。"""
    members, limitations = [], []
    if group.sample is not None:
        archive = load_archive(group.sample)
        if window == "volume":
            source = archive.window(volume=True)
            roles = archive.metadata["volume_support"]
            window_kind = "volume_support"
        elif archive.capabilities["primary_input"]:
            source, roles, window_kind = archive.window(), None, "primary"
        else:
            source, roles, window_kind = None, None, "single_observation"
            limitations.append("缺 provenance，仅审计末帧原观测")
        raw = [p.raw for p in source.pairs] if source is not None else [archive.role("observation")]
        for index, snapshot in enumerate(raw):
            usage = (["primary"] if roles is None else
                     [role for role, key in (("primary", "primary_members"), ("extra", "extra_members")) if index in roles[key]])
            members.append((snapshot.frame().xyz, str(index), "yaw_unknown/current_yaw_not_applied", usage))
        limitations.append("未记录实际解码 yaw，保留原坐标；使用当前 ROI/地面配置，不代表验证当前 yaw")
    else:
        if window != "primary":
            raise ValueError("旧 raw 不包含体积用途清单，不能冒充 volume_support")
        window_kind = "legacy_raw"
        for path in group.files:
            captured = captured_yaw(path)
            points, delta = load_frame(path, info["yaw"], not as_captured)
            status = ("yaw_unknown/current_yaw_not_applied" if captured is None else
                      "as_captured" if as_captured else "current_yaw_delta=%+.6f" % delta)
            members.append((points, os.path.basename(path), status, ["legacy_raw"]))
    if as_captured:
        limitations.append("--as-captured 只禁止 yaw 旋转，不恢复历史 ROI 或其它配置")
    best, spans, details, unknown = None, [], [], 0
    for points, name, yaw_status, usage in members:
        boxes = find_boxes(points, info["ground_z"])
        index = main_box(boxes)
        selected = [boxes[index]] if index is not None else []
        if selected:
            spans.append((selected[0]["y0"], selected[0]["y1"]))
        else:
            unknown += 1
        ok, bad, margin = check(selected, info["roi"])
        details.append(dict(member=name, yaw_status=yaw_status, usage=usage, boxes=boxes,
                            ok=bool(ok and selected), margin=margin))
        if selected:
            candidate = (len(boxes), ok, bad, margin)
            if best is None or (candidate[1], candidate[3]) < (best[1], best[3]):
                best = candidate
    shaky = bool(len(spans) >= 2 and
                 (max(s[1] for s in spans) - min(s[1] for s in spans) > SPAN_TOL
                  or max(s[0] for s in spans) - min(s[0] for s in spans) > SPAN_TOL))
    count, ok, bad, margin = best if best is not None else (0, False, [], None)
    if not count:
        verdict, status = "未识别候选，未知", "unknown"
    elif unknown:
        verdict, status = "部分帧无候选，存疑", "uncertain"
    elif shaky:
        verdict, status = "候选范围帧间不一致，存疑", "uncertain"
    elif ok:
        verdict, status = "可见候选在 ROI 内", "inside_visible"
    else:
        verdict, status = "可见候选越界", "outside_visible"
    return dict(channel=group.channel, key=group.key, window_kind=window_kind, frames=len(members),
                boxes=count, ok=status == "inside_visible", status=status, verdict=verdict,
                margin=margin, shaky=shaky, unknown_frames=unknown, detail="；".join(bad),
                physical_boundary_verified=False, limitations=limitations, members=details)


def main(argv=None):
    _common.setup_console()
    ap = argparse.ArgumentParser(description="审计原观测中可见候选与 ROI 的关系（非物理边界验证）")
    ap.add_argument("--dir", default="", help="输出根、archives-v1、单包或旧 raw 目录；默认各通道输出根")
    ap.add_argument("--date", default="", help="YYYYMMDD；新包按采集时间的本机本地日期，旧 raw 按文件名")
    ap.add_argument("--channel", default="", help="只看某一路，如 CH2")
    ap.add_argument("--window", choices=("primary", "volume"), default="primary", help="原主窗口或独立体积支持窗口")
    ap.add_argument("--csv", default="", help="另存 CSV")
    ap.add_argument("--json", default="", help="另存完整审计报告")
    ap.add_argument("--verbose", action="store_true", help="打印每成员候选包围盒")
    ap.add_argument("--as-captured", action="store_true", help="不旋转旧 raw；不等于恢复全部历史配置")
    args = ap.parse_args(argv)
    if args.date:
        try:
            if not re.fullmatch(r"\d{8}", args.date):
                raise ValueError()
            datetime.strptime(args.date, "%Y%m%d")
        except ValueError:
            ap.error("--date 必须为有效 YYYYMMDD")
    _common.ensure_root()
    settings = load_settings("")
    per_ch = {}
    for channel in settings.channels:
        if args.channel and channel.id != args.channel:
            continue
        calib = load_calib(channel.abs_config_path)
        lo, hi = np.asarray(calib.roi[0]), np.asarray(calib.roi[1])
        yaw = getattr(channel.lidar, "yaw_offset_deg", None)
        per_ch[channel.id] = dict(roi=((float(lo[0]), float(hi[0])), (float(lo[1]), float(hi[1]))),
                                  ground_z=float(calib.detection.ground_z),
                                  yaw=float(yaw if yaw is not None else calib.processing.yaw_offset_deg))
    if args.channel and args.channel not in per_ch:
        ap.error("没有该通道的配置: " + args.channel)
    groups, errors = discover_audit_groups(settings, args.dir, args.date, args.channel)
    print("边界：只审计可见几何候选，未扫描/遮挡端面不能证明完整；当前 ROI/地面配置。")
    print("新包日期按采集时间转本机本地日期；本机当前时区: " + str(datetime.now().astimezone().tzinfo))
    rows = []
    for group in groups:
        try:
            row = audit_group(group, per_ch[group.channel], as_captured=args.as_captured, window=args.window)
        except (OSError, ValueError) as exc:
            errors.append("%s/%s 读取失败: %s" % (group.channel, group.key, exc))
            continue
        rows.append(row)
        margin = "--" if row["margin"] is None else "%+.2f" % row["margin"]
        print("%s/%s  %s  %d 帧  余量 %s  %s  %s" %
              (row["channel"], row["key"], row["window_kind"], row["frames"], margin, row["verdict"], row["detail"]))
        for limitation in row["limitations"]:
            print("  " + limitation)
        for member in row["members"]:
            print("  成员 %s  %s  用途 %s" % (member["member"], member["yaw_status"], ",".join(member["usage"])))
            if args.verbose:
                for box in member["boxes"]:
                    print("    候选 x %.2f..%.2f  y %.2f..%.2f" % (box["x0"], box["x1"], box["y0"], box["y1"]))
    summary = {status: sum(row["status"] == status for row in rows)
               for status in ("inside_visible", "outside_visible", "uncertain", "unknown")}
    print("合计 %d 份：可见候选在内 %d，候选越界 %d，存疑 %d，未知 %d，读取/发现失败 %d。" %
          (len(rows), summary["inside_visible"], summary["outside_visible"], summary["uncertain"], summary["unknown"], len(errors)))
    for error in errors:
        print("审计错误: " + error, file=sys.stderr)
    if not rows:
        print("没有可审查的原观测：核对归档保存及目录；新包不要求 raw_frames > 0。")
    if args.json:
        with open(args.json, "w", encoding="utf-8") as stream:
            json.dump(dict(rows=rows, summary=summary, errors=errors, physical_boundary_verified=False),
                      stream, ensure_ascii=False, indent=2, allow_nan=False)
    if args.csv:
        import csv
        with open(args.csv, "w", newline="", encoding="utf-8-sig") as stream:
            writer = csv.writer(stream)
            writer.writerow(["通道", "测量", "窗口", "原帧数", "可见候选结论", "最小余量m", "明细", "物理边界已验证"])
            for row in rows:
                writer.writerow([row["channel"], row["key"], row["window_kind"], row["frames"],
                                 row["verdict"], row["margin"], row["detail"], "否"])
    return 2 if errors or not rows else 0


if __name__ == "__main__":
    sys.exit(main())
