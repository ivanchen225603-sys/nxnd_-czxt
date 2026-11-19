# -*- coding: utf-8 -*-
"""把每一辆车的整帧点云画成图，让人**用眼睛**核对车厢有没有完全落在 ROI 里。

为什么要画图
------------
`cli/roi_audit.py` 给的是数字（余量 +0.42 m、"存疑"），但它靠的是自动分段，
而分段本身会出错 —— 实测就有"前方另有一辆车首尾相接被并成一段"、
"门架宽度恰好像车厢"这类情况（docs/08 §11.2）。
**数字不可信的时候，人看一眼就知道**。所以这个工具只负责如实画，不下结论。

画什么
------
每辆车一张图，两个视图（都以 y 为横轴 —— 车厢是沿 y 的长条）：

    上：俯视 (y, x)   看车厢有没有超出 ROI 的**左右**和**前后**边界
    下：侧视 (y, z)   看货顶有没有顶到 ROI 的**顶盖**（短车厢实测只剩 0.18 m）

红框 = ROI（`config.json` 的 roi），绿框 = 自动识别出的被测车厢。
点按离地高度着色：低（地面/底板）偏蓝，高（货顶/厢壁）偏黄。

▲ **画的是未裁 ROI 的整帧**（`service.raw_frames` 存的那些），所以框外面的
  东西看得见 —— 这正是"车厢有没有被切"唯一能直接看的证据。
  落盘的判定帧是裁过 ROI 的，拿它画只能看到框里面，没有意义。

▲ 会按**当前配置**的 yaw 复算（与 roi_audit 同一套逻辑）。整帧存的是
  已解码的笛卡尔坐标，改完 `channels.json` 不复算的话画出来还是旧角度。

为什么不用 matplotlib
---------------------
项目的设计原则是**只依赖 numpy**（目标平台是麒麟 OS，见 requirements.txt）。
现场机器上多半没有 matplotlib，而这个工具恰恰是现场要用的。
所以 PNG 用 numpy 光栅化 + 标准库 zlib 自己写 —— 与项目自己写 HTTP 服务、
Canvas2D 前端是同一个路数。

用法
----
    python3 cli/roi_plot.py                          # 画 data/field/raw 下全部
    python3 cli/roi_plot.py --date 20260915          # 只画某一天
    python3 cli/roi_plot.py --channel CH2            # 只画某一路
    python3 cli/roi_plot.py --out data/diag/roi_png  # 指定输出目录
    python3 cli/roi_plot.py --no-sheet               # 不生成汇总长图
    python3 cli/roi_plot.py --as-captured            # 不按当前 yaw 复算
"""
from __future__ import annotations

import argparse
import glob
import os
import struct
import sys
import zlib

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cli import _common                                   # noqa: E402
from app.settings import load_calib, load_settings        # noqa: E402
from app.detect import carriage_agreement                 # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import roi_audit as RA                                      # noqa: E402
from roi_audit import (captured_yaw, check, find_boxes,    # noqa: E402
                       load_frame, main_box)

PX_PER_M = 34                 # 每米多少像素
PAD = 26                      # 四周留白（放坐标标注）
GAP = 10                      # 两个视图之间的间隔
BG = (250, 251, 253)
FG = (30, 36, 48)
GRID = (222, 228, 236)
ROI_C = (214, 52, 40)         # ROI 红
BOX_C = (16, 140, 74)         # 车厢绿
WARN_C = (232, 140, 20)       # 越界/存疑 橙

# ── 5x7 点阵字（只有 ASCII —— 中文说明放在文件名和索引里）──────────
def _glyphs():
    """5x7 点阵，每个字符 5 列、每列低 7 位。够写编号与余量就行。"""
    raw = {
        " ": (0, 0, 0, 0, 0),
        "0": (0x3E, 0x51, 0x49, 0x45, 0x3E), "1": (0x00, 0x42, 0x7F, 0x40, 0x00),
        "2": (0x62, 0x51, 0x49, 0x49, 0x46), "3": (0x22, 0x41, 0x49, 0x49, 0x36),
        "4": (0x18, 0x14, 0x12, 0x7F, 0x10), "5": (0x27, 0x45, 0x45, 0x45, 0x39),
        "6": (0x3C, 0x4A, 0x49, 0x49, 0x30), "7": (0x01, 0x71, 0x09, 0x05, 0x03),
        "8": (0x36, 0x49, 0x49, 0x49, 0x36), "9": (0x06, 0x49, 0x49, 0x29, 0x1E),
        "A": (0x7E, 0x11, 0x11, 0x11, 0x7E), "B": (0x7F, 0x49, 0x49, 0x49, 0x36),
        "C": (0x3E, 0x41, 0x41, 0x41, 0x22), "D": (0x7F, 0x41, 0x41, 0x22, 0x1C),
        "E": (0x7F, 0x49, 0x49, 0x49, 0x41), "F": (0x7F, 0x09, 0x09, 0x09, 0x01),
        "G": (0x3E, 0x41, 0x49, 0x49, 0x7A), "H": (0x7F, 0x08, 0x08, 0x08, 0x7F),
        "I": (0x00, 0x41, 0x7F, 0x41, 0x00), "J": (0x20, 0x40, 0x41, 0x3F, 0x01),
        "K": (0x7F, 0x08, 0x14, 0x22, 0x41), "L": (0x7F, 0x40, 0x40, 0x40, 0x40),
        "M": (0x7F, 0x02, 0x0C, 0x02, 0x7F), "N": (0x7F, 0x04, 0x08, 0x10, 0x7F),
        "O": (0x3E, 0x41, 0x41, 0x41, 0x3E), "P": (0x7F, 0x09, 0x09, 0x09, 0x06),
        "Q": (0x3E, 0x41, 0x51, 0x21, 0x5E), "R": (0x7F, 0x09, 0x19, 0x29, 0x46),
        "S": (0x46, 0x49, 0x49, 0x49, 0x31), "T": (0x01, 0x01, 0x7F, 0x01, 0x01),
        "U": (0x3F, 0x40, 0x40, 0x40, 0x3F), "V": (0x1F, 0x20, 0x40, 0x20, 0x1F),
        "W": (0x3F, 0x40, 0x38, 0x40, 0x3F), "X": (0x63, 0x14, 0x08, 0x14, 0x63),
        "Y": (0x07, 0x08, 0x70, 0x08, 0x07), "Z": (0x61, 0x51, 0x49, 0x45, 0x43),
        ".": (0x00, 0x60, 0x60, 0x00, 0x00), "-": (0x08, 0x08, 0x08, 0x08, 0x08),
        "+": (0x08, 0x08, 0x3E, 0x08, 0x08), "_": (0x40, 0x40, 0x40, 0x40, 0x40),
        "/": (0x20, 0x10, 0x08, 0x04, 0x02), ":": (0x00, 0x36, 0x36, 0x00, 0x00),
        "%": (0x23, 0x13, 0x08, 0x64, 0x62), "(": (0x00, 0x1C, 0x22, 0x41, 0x00),
        ")": (0x00, 0x41, 0x22, 0x1C, 0x00), "?": (0x02, 0x01, 0x51, 0x09, 0x06),
        "!": (0x00, 0x00, 0x5F, 0x00, 0x00), "=": (0x14, 0x14, 0x14, 0x14, 0x14),
        "*": (0x14, 0x08, 0x3E, 0x08, 0x14), ",": (0x00, 0x50, 0x30, 0x00, 0x00),
        "x": (0x22, 0x14, 0x08, 0x14, 0x22),
    }
    return raw


FONT = _glyphs()


def text(img, x, y, s, color=FG, scale=1):
    """在 (x, y) 画一行 ASCII。越界自动裁掉，不抛异常。"""
    h, w, _ = img.shape
    cx = x
    for ch in str(s):
        g = FONT.get(ch) or FONT.get(ch.upper()) or FONT[" "]
        for col in range(5):
            bits = g[col]
            for row in range(7):
                if bits & (1 << row):
                    x0, y0 = cx + col * scale, y + row * scale
                    x1, y1 = min(w, x0 + scale), min(h, y0 + scale)
                    if x0 < w and y0 < h and x1 > 0 and y1 > 0:
                        img[max(0, y0):y1, max(0, x0):x1] = color
        cx += 6 * scale
    return cx


def rect(img, x0, y0, x1, y1, color, width=1):
    h, w, _ = img.shape
    x0, x1 = int(min(x0, x1)), int(max(x0, x1))
    y0, y1 = int(min(y0, y1)), int(max(y0, y1))
    for t in range(width):
        for xx in (x0 + t, x1 - t):
            if 0 <= xx < w:
                img[max(0, y0):min(h, y1 + 1), xx] = color
        for yy in (y0 + t, y1 - t):
            if 0 <= yy < h:
                img[yy, max(0, x0):min(w, x1 + 1)] = color


def write_png(path, img):
    """纯标准库写 PNG（8 位 RGB）。不引 PIL —— 见文件头的说明。"""
    h, w, _ = img.shape
    raw = b"".join(b"\x00" + img[y].tobytes() for y in range(h))

    def chunk(tag, data):
        body = tag + data
        return (struct.pack(">I", len(data)) + body
                + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF))

    out = b"\x89PNG\r\n\x1a\n"
    out += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    out += chunk(b"IDAT", zlib.compress(raw, 6))
    out += chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(out)


def _color_by_height(h, lo, hi):
    """离地高度 -> 颜色。低处偏蓝（地面/底板），高处偏黄（货顶/厢壁）。"""
    t = np.clip((h - lo) / max(1e-6, hi - lo), 0.0, 1.0)
    r = (40 + 215 * t).astype(np.uint8)
    g = (90 + 130 * t).astype(np.uint8)
    b = (190 - 150 * t).astype(np.uint8)
    return np.stack([r, g, b], 1)


OUT_C = (232, 40, 40)         # ROI 之外的高处点 —— 标红让人一眼看见


def carriage_x_band(a, ground_z, half_win=2.0, pad=0.20):
    """车厢所在的 x 带。**只用天底附近那几片定** —— 那里点最密、掠射角最小。

    x 方向是这套数据里最可靠的量：2026-09-16 实测同一辆停着的车 3 帧之间
    车厢宽度差中位 0.04 m、最大 0.29 m（而 y 方向长度差中位 1.50 m、最大 5.75 m）。
    所以拿 x 带当锚，比拿"自动识别的车厢框"当锚稳得多。
    """
    h = a[:, 2] - ground_z
    p = a[(h > RA.H_LO) & (h < RA.H_HI) & (np.abs(a[:, 0]) < RA.X_WIN)
          & (np.abs(a[:, 1]) < half_win)]
    if len(p) < 200:
        p = a[(h > RA.H_LO) & (h < RA.H_HI) & (np.abs(a[:, 0]) < RA.X_WIN)]
    if len(p) < 200:
        return None
    xe = np.arange(-RA.X_WIN, RA.X_WIN + RA.CELL, RA.CELL)
    hh, _ = np.histogram(p[:, 0], bins=xe)
    occ = hh >= max(3, int(0.02 * hh.max()))
    best, cur = None, None
    for i, o in enumerate(list(occ) + [False]):
        if o and cur is None:
            cur = i
        elif not o and cur is not None:
            if best is None or (i - cur) > (best[1] - best[0]):
                best = (cur, i)
            cur = None
    if best is None:
        return None
    sel = p[(p[:, 0] >= xe[best[0]]) & (p[:, 0] < xe[best[1]])]
    if len(sel) < 50:
        return None
    return float(sel[:, 0].min()) - pad, float(sel[:, 0].max()) + pad


def outside_mask(a, roi, ground_z, min_h=0.5, max_h=4.8, band=None):
    """**车厢那条带上、高于地面 min_h、却落在 ROI 之外**的点。

    ▲ 一定要限定在车厢的 x 带里。不限的话，厂房墙、门架、屋顶全被算成
      "框外的点" —— 实测一帧能标出 3 万个红点，图糊成一片，什么也看不出来。
    ▲ 还要**按高度封顶**（max_h）。车厢正上方就是厂房顶和雷达支架，
      它们离地 5.1~5.5 m，高过 ROI 顶盖（4.48 m），不封顶的话每一帧都会
      在车厢中央标出一片红 —— 那不是车厢被切，是屋顶。
      货顶实测最高 4.30 m，屋顶最低 5.10 m，中间那段空隙里取 4.8。

    ★ 这才是"车厢有没有被切"的直接证据 —— 不依赖任何分割。
      自动找车厢（roi_audit.find_boxes）在 y 方向根本不可信：
      2026-09-16 实测同一辆停着的车，3 帧之间车厢**长度**差中位 1.50 m、
      最大 5.75 m，60% 的样本差 1 m 以上（x 方向倒是很稳，差 <0.3 m）。
      所以图上不要让人去信那个框 —— 直接把框外的东西标出来，
      是车厢还是门架/驾驶室，人一眼就分得出。
    """
    (rx0, rx1), (ry0, ry1), (rz0, rz1) = roi
    h = a[:, 2] - ground_z
    m = (h > min_h) & (h < max_h)
    if band is not None:
        m &= (a[:, 0] >= band[0]) & (a[:, 0] <= band[1])
    return m & ((a[:, 0] < rx0) | (a[:, 0] > rx1)
                | (a[:, 1] < ry0) | (a[:, 1] > ry1)
                | (a[:, 2] < rz0) | (a[:, 2] > rz1))


def render(a, roi, ground_z, title, sub, box=None, bad=False):
    """画一辆车：上俯视 (y,x)、下侧视 (y,z)。返回 (H,W,3) uint8。"""
    (rx0, rx1), (ry0, ry1), (rz0, rz1) = roi
    # 视野比 ROI 外扩，才看得见框外面的东西
    vy0, vy1 = ry0 - 2.0, ry1 + 3.0
    vx0, vx1 = rx0 - 1.5, rx1 + 1.5
    vz0, vz1 = rz0 - 0.5, rz1 + 0.6
    W = int((vy1 - vy0) * PX_PER_M)
    H1 = int((vx1 - vx0) * PX_PER_M)
    H2 = int((vz1 - vz0) * PX_PER_M)
    img = np.empty((PAD * 2 + H1 + GAP + H2 + 44, W + PAD * 2, 3), np.uint8)
    img[:] = BG

    def put(px, py, cols, top):
        m = (px >= 0) & (px < W) & (py >= 0) & (py < top[1])
        img[top[0] + py[m], PAD + px[m]] = cols[m]

    # ★ 按高度排序，**高点后画** —— 否则地面回波（每帧上万点）会把车厢盖住，
    #   俯视图上就只剩一片蓝，看不出车厢轮廓。
    a = a[np.argsort(a[:, 2], kind="stable")]
    hgt = a[:, 2] - ground_z
    cols = _color_by_height(hgt, 0.0, 4.5)
    # ★ ROI 之外的高处点标红并**最后画**（盖在别的点上面），一眼看得见
    out = outside_mask(a, roi, ground_z, band=carriage_x_band(a, ground_z))
    cols[out] = OUT_C
    order = np.argsort(out.astype(np.int8), kind="stable")
    a, cols, hgt, out = a[order], cols[order], hgt[order], out[order]
    py0 = PAD + 10
    py1 = PAD + H1 + GAP
    # 俯视：y 横、x 纵
    px = ((a[:, 1] - vy0) * PX_PER_M).astype(np.int64)
    py = ((vx1 - a[:, 0]) * PX_PER_M).astype(np.int64)
    put(px, py, cols, (py0, H1))
    # 侧视：y 横、z 纵
    pz = ((vz1 - a[:, 2]) * PX_PER_M).astype(np.int64)
    put(px, pz, cols, (py1, H2))

    def wx(y):
        return PAD + (y - vy0) * PX_PER_M

    # ROI 框
    rect(img, wx(ry0), py0 + (vx1 - rx1) * PX_PER_M,
         wx(ry1), py0 + (vx1 - rx0) * PX_PER_M, ROI_C, 2)
    rect(img, wx(ry0), py1 + (vz1 - rz1) * PX_PER_M,
         wx(ry1), py1 + (vz1 - rz0) * PX_PER_M, ROI_C, 2)
    # 车厢框（只画俯视，**仅供参考**）——
    # 它在 y 方向不可信（见 outside_mask 的说明），所以画得淡一些，
    # 不要让看图的人把它当结论。真正的结论看红点。
    if box:
        c = WARN_C if bad else BOX_C
        rect(img, wx(box["y0"]), py0 + (vx1 - box["x1"]) * PX_PER_M,
             wx(box["y1"]), py0 + (vx1 - box["x0"]) * PX_PER_M, c, 1)

    text(img, PAD, 6, title, FG)
    # 图例：红框就是 ROI 本身，把它的数值写出来，免得看的人还要去翻配置
    lx = text(img, PAD, 16, "RED=ROI X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f]"
              % (rx0, rx1, ry0, ry1, rz0, rz1), ROI_C)
    lx = text(img, lx + 12, 16, "GREEN=BOX(REF ONLY)", WARN_C if bad else BOX_C)
    text(img, lx + 12, 16, "RED DOTS=CARRIAGE BAND OUTSIDE ROI", OUT_C)
    text(img, 3, py0 + H1 // 2, "TOP", GRID)
    text(img, 3, py1 + H2 // 2, "SIDE", GRID)
    text(img, PAD, PAD * 2 + H1 + GAP + H2 + 14, sub, WARN_C if bad else FG)
    return img


def main(argv=None):
    _common.setup_console()
    ap = argparse.ArgumentParser(description="把整帧点云画成 ROI 核对图")
    ap.add_argument("--dir", default="", help="整帧根目录（默认各通道 output_dir/raw）")
    ap.add_argument("--date", default="", help="只画某天，如 20260915")
    ap.add_argument("--channel", default="", help="只画某一路")
    ap.add_argument("--out", default="data/diag/roi_png", help="输出目录")
    ap.add_argument("--as-captured", action="store_true", help="不按当前 yaw 复算")
    ap.add_argument("--no-sheet", action="store_true", help="不生成每路的汇总长图")
    args = ap.parse_args(argv)
    _common.ensure_root()

    st = load_settings("")
    out_root = args.out
    os.makedirs(out_root, exist_ok=True)
    index = []
    n_all = 0

    for c in st.channels:
        if args.channel and c.id != args.channel:
            continue
        calib = load_calib(c.abs_config_path)
        lo, hi = np.asarray(calib.roi[0], float), np.asarray(calib.roi[1], float)
        roi = ((float(lo[0]), float(hi[0])), (float(lo[1]), float(hi[1])),
               (float(lo[2]), float(hi[2])))
        gz = float(calib.detection.ground_z)
        dyaw = getattr(c.lidar, "yaw_offset_deg", None)
        cur_yaw = float(dyaw if dyaw is not None else calib.processing.yaw_offset_deg)
        d = args.dir or os.path.join(c.abs_output_dir, "raw", c.id)
        files = sorted(glob.glob(os.path.join(d, "%s_%s*.xyz" % (c.id, args.date or ""))))
        if not files:
            continue
        groups = {}
        for f in files:
            groups.setdefault(os.path.basename(f).rsplit("_", 1)[0], []).append(f)

        print("=" * 72)
        print("%s  %d 份  ->  %s" % (c.id, len(groups), out_root))
        sheet_rows = []
        for key in sorted(groups):
            f = sorted(groups[key])[-1]            # 末帧 = 测量用的那一帧
            try:
                a, dd = load_frame(f, cur_yaw, not args.as_captured)
            except Exception as e:                  # noqa: BLE001
                print("  x %s 读取失败 %s" % (key, e))
                continue
            # ★ 绿框改用 detect.carriage_box（按横截面宽度切），并带自举置信度。
            #   旧的 find_boxes 沿 y 数点数，远端掠射角下点密度骤降 ->
            #   段被切断 -> 框在 y 上乱跳。新法拿的是「内宽 2.4 m」这个
            #   现场最稳的量（72 份标准差 0.070 m），见 detect.carriage_box。
            # 整帧（未裁 ROI）必须给 z 带：厂房顶在离地 5.1~5.5 m，
            # 也落在 |x| < 1.6 里，不剔掉会把每格的截面宽度顶满。
            #
            # ★ 用**同一组的全部帧**量一致性，不是单帧抖参数 —— 后者预测不了
            #   帧间稳不稳（实测判"可信"的 15 组里真稳的只有 6 组，见
            #   detect.carriage_confidence 的说明与 docs/08 §17）。
            zb = (gz + 0.30, gz + 4.80)
            grp = []
            for gf in sorted(groups[key]):
                try:
                    ga, _ = load_frame(gf, cur_yaw, not args.as_captured)
                    grp.append(ga)
                except Exception:                    # noqa: BLE001
                    pass
            conf = carriage_agreement(grp, z_band=zb)
            cb = conf["box"]
            b = None
            if cb is not None:
                b = {"x0": cb["x0"], "x1": cb["x1"], "y0": cb["y0"], "y1": cb["y1"],
                     "z_top": float(a[:, 2].max() - gz), "n": cb["n_points"]}
            ok, bad_list, margin = check([b] if b else [], (roi[0], roi[1]))
            tag = key.replace(c.id + "_", "")
            # ★ 不依赖分割的硬指标：框外有多少高处点、最远伸出多少
            om = outside_mask(a, roi, gz, band=carriage_x_band(a, gz))
            n_out = int(om.sum())
            reach = n_near = 0
            reach = 0.0
            if n_out:
                q = a[om]
                # 每个红点伸出 ROI 多远（取它超出得最多的那一面）
                over = np.maximum.reduce([
                    roi[1][0] - q[:, 1], q[:, 1] - roi[1][1],
                    roi[0][0] - q[:, 0], q[:, 0] - roi[0][1],
                    roi[2][0] - q[:, 2], q[:, 2] - roi[2][1]])
                reach = float(over.max())
                # ★ **贴边**的红点才可能是"车厢被切"：伸出 2 m 以上的
                #   几乎一定是门架/旁边的车这类独立结构（实测门架在 y=13 一带）。
                n_near = int((over <= 2.0).sum())
            sub = ("MARGIN %+.2f M" % margin) if margin is not None else "NO BOX"
            sub += "   OUT %d PT (NEAR %d, MAX %.2f M)" % (n_out, n_near, reach)
            if b:
                sub += "   BOX %.2f X %.2f M" % (b["x1"] - b["x0"], b["y1"] - b["y0"])
                sub += "   TOP %.2f M" % (b["z_top"])
                sub += "   %s(%dF SWAY %.2f)" % (
                    "SOLID" if conf["ok"] else "SHAKY", conf["n"], conf["sway"])
            img = render(a, roi, gz, "%s  %s  (YAW %+.1f)" % (c.id, tag, cur_yaw),
                         sub, b, bad=not ok)
            p = os.path.join(out_root, "%s_%s.png" % (c.id, tag))
            write_png(p, img)
            sheet_rows.append(img)
            index.append((c.id, tag, margin, ok, n_out, n_near, reach,
                          os.path.basename(p), conf["ok"], conf["sway"],
                          None if cb is None else cb["length"],
                          None if cb is None else cb["width"]))
            n_all += 1
            print("  %s %s  %s" % ("√" if ok else "X", tag, sub))

        if sheet_rows and not args.no_sheet:
            w = max(r.shape[1] for r in sheet_rows)
            hh = sum(r.shape[0] for r in sheet_rows)
            sheet = np.full((hh, w, 3), 255, np.uint8)
            y = 0
            for r in sheet_rows:
                sheet[y:y + r.shape[0], :r.shape[1]] = r
                y += r.shape[0]
            sp = os.path.join(out_root, "_%s_ALL.png" % c.id)
            write_png(sp, sheet)
            print("  汇总长图 -> %s（%d x %d）" % (sp, w, hh))

    if index:
        ip = os.path.join(out_root, "index.csv")
        import csv
        with open(ip, "w", newline="", encoding="utf-8-sig") as fp:
            wr = csv.writer(fp)
            wr.writerow(["车道", "数据编号", "★贴边红点数(<=2m)", "框外红点总数",
                         "最远伸出m", "车厢框最小余量m", "框判定",
                         "★车厢内宽m(可信)", "车厢长m(存疑)",
                         "切割可信", "端点摆幅m", "图片"])
            for (cid, tag, m, ok, n_out, n_near, reach, png,
                 cok, sway, ln, wd) in index:
                wr.writerow([cid, tag, n_near, n_out, "%.2f" % reach,
                             "" if m is None else "%.2f" % m,
                             "完整在内" if ok else "需人工看",
                             "" if wd is None else "%.2f" % wd,
                             "" if ln is None else "%.2f" % ln,
                             "是" if cok else "否", "%.2f" % sway, png])
        print()
        print("共 %d 张，索引 %s" % (n_all, ip))
        print("红框 = ROI；**红点 = 高于地面 0.5 m 却落在 ROI 之外的点**。")
        print("看红点就够了：是车厢被切，还是门架/驾驶室/旁边的车，一眼分得出。")
        print("绿框 = detect.carriage_box 切出来的车厢。**x 可信、y 存疑**：")
        print("  内宽（x）62 份实测 2.26~2.53、标准差 0.067 m —— 可以当准确值用；")
        print("  两端（y）不可信：58 组真-停稳三连帧实测，帧间端点离散")
        print("  中位 1.30 m、最大 4.50 m。index.csv 的「切割可信」是拿同一组")
        print("  多帧一致性判的，=否 的那些必须人工看图定夺。")
        print("上半俯视看左右/前后，下半侧视看货顶有没有顶到 ROI 顶盖。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
