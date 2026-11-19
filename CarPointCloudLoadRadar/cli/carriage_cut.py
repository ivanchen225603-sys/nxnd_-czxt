# -*- coding: utf-8 -*-
"""把 `detect.carriage_box` 切出来的车厢画成图，供人逐辆核对。

和 `cli/roi_plot.py` 的分工
--------------------------
* `roi_plot.py` 回答"**车厢有没有被 ROI 切掉**" —— 看红点；
* 本工具回答"**算法切出来的那块到底是不是车厢**" —— 看灰/彩的分界。

每张图三栏：
  TOP   (y,x) 整帧俯视：**灰 = 被丢掉的**，彩 = 切出来当车厢的
  SIDE  (y,z) 整帧侧视：同上
  CROSS (x,z) **只画切出来的那块**的横截面

CROSS 这一栏是最省事的核对方式：切对了它长这样 ——
  · 空车：一个"U" —— 两侧竖直厢壁 + 中间底板；
  · 满载：一块贴着厢壁顶的实心板；
切进驾驶室 / 并进地面 / 切到旁边那辆车，形状立刻就不对了。

▲ 看图之前先记住结论（docs/08 §17）：
  **x（内宽）准，y（两端）不准。** 58 组真-停稳三连帧实测，帧间端点离散
  中位 1.30 m、最大 4.50 m。所以核对的重点是"有没有把不该要的东西切进来"，
  而不是"端点差几厘米"。表头里 AGREE 那一项是多帧一致性，
  **不一致的那些优先看**。

用法
----
    python3 cli/carriage_cut.py                      # 全部通道
    python3 cli/carriage_cut.py --channel CH4        # 只看一路
    python3 cli/carriage_cut.py --only-shaky         # 只出多帧不一致的那些
    python3 cli/carriage_cut.py --dir data/field/未知车牌 --cropped
       # 用已裁 ROI 的落盘样本（不传 z_band），而不是 raw 整帧
"""
from __future__ import annotations

import argparse
import collections
import csv
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from cli import _common                                       # noqa: E402
from app.settings import load_calib, load_settings            # noqa: E402
from app.detect import carriage_box, carriage_agreement       # noqa: E402
import roi_plot as RP                                         # noqa: E402
import roi_audit as RA                                        # noqa: E402

DROP_C = (176, 184, 196)      # 被丢掉的点 —— 灰（要看得见，才判得出「丢对没有」）
CUT_C = (16, 140, 74)         # 切割框 绿
SHAKY_C = RP.WARN_C           # 多帧不一致 橙


def render_cut(a, keep, box, ground_z, title, sub, shaky, roi_y=(-11.0, 8.0)):
    """三栏图：TOP(y,x) 与 SIDE(y,z) 上下排，CROSS(x,z) 放在 SIDE 右边。

    ★ 视野**必须收到 ROI 附近**，不能按整帧的 y 范围来。
      raw 整帧的 y 跨度有 60 m 开外（门架、隔壁车道、厂房全在里面），
      按它铺开的话车厢只占图上一小条，等于没法看。
    """
    P, G = RP.PAD, RP.GAP
    ppm = RP.PX_PER_M
    vy0, vy1 = roi_y[0] - 2.0, roi_y[1] + 2.0
    if box:                       # 车厢万一超出 ROI，也要看得见
        vy0 = min(vy0, box["y0"] - 1.0)
        vy1 = max(vy1, box["y1"] + 1.0)
    vx0, vx1 = -2.6, 2.6
    vz0, vz1 = ground_z - 0.4, ground_z + 5.2
    W = int((vy1 - vy0) * ppm)
    H1 = int((vx1 - vx0) * ppm)
    H2 = int((vz1 - vz0) * ppm)
    H3 = H2
    WC = int((vx1 - vx0) * ppm)
    cx = P + W + G * 3            # CROSS 的左边界
    top_w = W + G * 3 + WC
    img = np.empty((P * 2 + H1 + G + H2 + 46, top_w + P * 2, 3), np.uint8)
    img[:] = RP.BG

    def put(px, py, cols, oy, hh, ox=P):
        m = (px >= 0) & (px + ox - P < top_w) & (py >= 0) & (py < hh)
        img[oy + py[m], ox + px[m]] = cols[m]

    # 丢掉的先画、保留的后画（盖在上面），否则地面上万点会把车厢糊住
    order = np.argsort(keep.astype(np.int8), kind="stable")
    a, keep = a[order], keep[order]
    cols = np.empty((len(a), 3), np.uint8)
    cols[~keep] = DROP_C
    if keep.any():
        cols[keep] = RP._color_by_height(a[keep, 2] - ground_z, 0.0, 4.5)

    y0p = P + 12
    y1p = y0p + H1 + G
    px = ((a[:, 1] - vy0) * ppm).astype(np.int64)
    put(px, ((vx1 - a[:, 0]) * ppm).astype(np.int64), cols, y0p, H1)
    put(px, ((vz1 - a[:, 2]) * ppm).astype(np.int64), cols, y1p, H2)
    # CROSS：只画切出来的那块，放在 SIDE 右边
    if keep.any():
        s, sc = a[keep], cols[keep]
        put(((s[:, 0] - vx0) * ppm).astype(np.int64),
            ((vz1 - s[:, 2]) * ppm).astype(np.int64), sc, y1p, H3, ox=cx)

    def wx(y):
        return P + (y - vy0) * ppm

    c = SHAKY_C if shaky else CUT_C
    # ROI 的 y 边界 —— 淡红竖线，好看出车厢离边界多远
    for ry in roi_y:
        RP.rect(img, wx(ry), y0p, wx(ry), y1p + H2 - 1, RP.ROI_C, 1)
    if box:
        RP.rect(img, wx(box["y0"]), y0p + (vx1 - box["x1"]) * ppm,
                wx(box["y1"]), y0p + (vx1 - box["x0"]) * ppm, c, 2)
        RP.rect(img, wx(box["y0"]), y1p, wx(box["y1"]), y1p + H2 - 1, c, 2)
        RP.rect(img, cx + (box["x0"] - vx0) * ppm, y1p,
                cx + (box["x1"] - vx0) * ppm, y1p + H3 - 1, c, 2)
    # 地面参考线
    gy = y1p + (vz1 - ground_z) * ppm
    RP.rect(img, P, gy, P + W - 1, gy, RP.GRID, 1)
    RP.rect(img, cx, gy, cx + WC - 1, gy, RP.GRID, 1)

    RP.text(img, P, 4, title, RP.FG)
    RP.text(img, 3, y0p + H1 // 2, "TOP", RP.GRID)
    RP.text(img, 3, y1p + H2 // 2, "SIDE", RP.GRID)
    RP.text(img, cx, y1p - 10, "CROSS (KEPT ONLY)", RP.GRID)
    RP.text(img, P, y1p + H2 + 12, sub, SHAKY_C if shaky else RP.FG)
    RP.text(img, P, y1p + H2 + 24,
            "GREY=DROPPED   COLOUR=KEPT AS CARRIAGE   THIN RED=ROI Y EDGE", RP.GRID)
    return img


def main(argv=None):
    _common.setup_console()
    ap = argparse.ArgumentParser(description="把算法切出来的车厢画成图，供逐辆核对")
    ap.add_argument("--dir", default="", help="样本目录（默认各通道 output_dir/raw）")
    ap.add_argument("--date", default="", help="只看某天，如 20260915")
    ap.add_argument("--channel", default="", help="只看某一路")
    ap.add_argument("--out", default="data/diag/carriage_png", help="输出目录")
    ap.add_argument("--cropped", action="store_true",
                    help="输入是已裁 ROI 的样本（不传 z_band）")
    ap.add_argument("--only-shaky", action="store_true", help="只出多帧不一致的")
    ap.add_argument("--as-captured", action="store_true", help="不按当前 yaw 复算")
    ap.add_argument("--no-sheet", action="store_true", help="不生成汇总长图")
    args = ap.parse_args(argv)
    _common.ensure_root()

    st = load_settings("")
    out_root = args.out
    os.makedirs(out_root, exist_ok=True)
    index, n_all = [], 0

    for c in st.channels:
        if args.channel and c.id != args.channel:
            continue
        calib = load_calib(c.abs_config_path)
        gz = float(calib.detection.ground_z)
        dyaw = getattr(c.lidar, "yaw_offset_deg", None)
        cur_yaw = float(dyaw if dyaw is not None else calib.processing.yaw_offset_deg)
        zb = None if args.cropped else (gz + 0.30, gz + 4.80)
        if args.dir:
            pat = os.path.join(args.dir, "*%s*.xyz" % (args.date or ""))
        else:
            pat = os.path.join(c.abs_output_dir, "raw", c.id,
                               "%s_%s*.xyz" % (c.id, args.date or ""))
        files = sorted(glob.glob(pat))
        if not files:
            continue
        groups = collections.defaultdict(list)
        for f in files:
            key = (os.path.basename(f)[:-4] if args.dir
                   else os.path.basename(f).rsplit("_", 1)[0])
            groups[key].append(f)

        print("=" * 74)
        print("%s  %d 辆  ->  %s" % (c.id, len(groups), out_root))
        sheet = []
        for key in sorted(groups):
            clouds = []
            for f in sorted(groups[key]):
                try:
                    ga, _ = RA.load_frame(f, cur_yaw, not args.as_captured)
                    clouds.append(np.asarray(ga, dtype=np.float64))
                except Exception as e:                        # noqa: BLE001
                    print("  x %s 读取失败 %s" % (key, e))
            if not clouds:
                continue
            ag = carriage_agreement(clouds, z_band=zb) if len(clouds) > 1 else None
            a = clouds[len(clouds) // 2]
            box = carriage_box(a, z_band=zb)
            shaky = (ag is None) or (not ag["ok"])
            moved = bool(ag and ag.get("moved"))
            if args.only_shaky and not shaky:
                continue
            if box is None:
                keep = np.zeros(len(a), bool)
                sub = "NO CARRIAGE CUT"
            else:
                keep = ((a[:, 1] >= box["y0"]) & (a[:, 1] <= box["y1"])
                        & (a[:, 0] >= box["x0"] - 0.15)
                        & (a[:, 0] <= box["x1"] + 0.15))
                if zb is not None:
                    keep &= (a[:, 2] > zb[0]) & (a[:, 2] < zb[1])
                sub = ("Y %+.2f..%+.2f   LEN %.2f M   WIDTH %.2f M   KEPT %d PT"
                       % (box["y0"], box["y1"], box["length"], box["width"],
                          int(keep.sum())))
            if ag is None:
                sub += "   AGREE: 1 FRAME ONLY"
            else:
                if ag["ok"]:
                    st_ = "OK"
                elif ag.get("moved"):
                    st_ = "MOVED"          # 车在动，端点对不上不怪算法
                else:
                    st_ = "**SHAKY**"
                sub += "   AGREE: %s (%dF POS %.2f / LEN %.2f M)" % (
                    st_, ag["n"], ag["sway"], ag["length_sway"])
            tag = key.replace(c.id + "_", "")
            n_all += 1
            title = "#%03d  %s  %s  (YAW %+.1f)" % (n_all, c.id, tag, cur_yaw)
            img = render_cut(a, keep, box, gz, title, sub, shaky,
                             roi_y=(float(calib.roi[0][1]), float(calib.roi[1][1])))
            p = os.path.join(out_root, "%s_%s.png" % (c.id, tag))
            RP.write_png(p, img)
            sheet.append(img)
            index.append((n_all, c.id, tag,
                          None if box is None else box["y0"],
                          None if box is None else box["y1"],
                          None if box is None else box["length"],
                          None if box is None else box["width"],
                          None if ag is None else ag["ok"],
                          None if ag is None else ag["sway"],
                          None if ag is None else ag["length_sway"], moved,
                          os.path.basename(p)))
            print("  %s #%03d %s  %s" % ("!" if shaky else " ", n_all, tag, sub))

        if sheet and not args.no_sheet:
            w = max(r.shape[1] for r in sheet)
            hh = sum(r.shape[0] for r in sheet)
            big = np.full((hh, w, 3), 255, np.uint8)
            y = 0
            for r in sheet:
                big[y:y + r.shape[0], :r.shape[1]] = r
                y += r.shape[0]
            sp = os.path.join(out_root, "_%s_ALL.png" % c.id)
            RP.write_png(sp, big)
            print("  汇总长图 -> %s（%d x %d）" % (sp, w, hh))

    if index:
        ip = os.path.join(out_root, "index.csv")
        with open(ip, "w", newline="", encoding="utf-8-sig") as fp:
            wr = csv.writer(fp)
            wr.writerow(["检查顺序", "序号", "车道", "数据编号", "y0", "y1",
                         "厢长m", "内宽m(可信)", "位置一致", "位置摆幅m",
                         "★厢长摆幅m", "厢长一致", "车在动", "图片"])

            def f2(v):
                return "" if v is None else "%.2f" % v

            # 检查顺序：多帧不一致的排前面，摆幅大的优先 ——
            # 那些正是最可能切错的，先看它们最省时间。
            def prio(r):
                ok, sw, mv = r[7], r[8], r[10]
                # 车在动的排到最后 —— 那不是算法的问题，没必要优先看
                bad = 0 if (ok is False or ok is None) and not mv else (2 if mv else 1)
                return (bad, -(sw if sw is not None else 0.0), r[0])

            for i, row in enumerate(sorted(index, key=prio), 1):
                n, cid, tag, y0, y1, L, W, ok, sw, lsw, mv, png = row
                # 厢长与位置要分开报：实测厢长已经稳了（帧间离散中位 0.15 m），
                # 而位置还在摆 2.9 m。混成一个"不一致"会把能用的厢长也埋掉。
                lok = lsw is not None and lsw <= 0.50
                wr.writerow([i, n, cid, tag, f2(y0), f2(y1), f2(L), f2(W),
                             "" if ok is None else ("是" if ok else "否"),
                             f2(sw), f2(lsw), "是" if lok else "否",
                             "是" if mv else "", png])
        print()
        print("共 %d 辆，索引 %s" % (n_all, ip))
        print("怎么看：**灰 = 算法丢掉的，彩 = 算法当成车厢留下的**。")
        print("  CROSS 那一栏只画留下的那块 —— 空车应该是个 U 形（两侧厢壁+底板），")
        print("  满载是一块贴着壁顶的实心板。形状不对就是切错了。")
        print("▲ x（内宽）准、y（两端）不准（docs/08 §17）：")
        print("  58 组真-停稳三连帧实测帧间端点离散中位 1.30 m、最大 4.50 m。")
        print("  所以重点看有没有把驾驶室/地面/旁边的车切进来，别纠结端点几厘米。")
        print("  表里「多帧一致=否」的优先看。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
