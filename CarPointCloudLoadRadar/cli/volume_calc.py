# -*- coding: utf-8 -*-
"""带配准的载货体积计算 —— 空车基线与载货点云先对齐，再逐格相减。

为什么需要配准
--------------
`detect.compute_volume` 用的是**绝对坐标网格**：拿基线盒划格，逐格做
`z_load - z_base`。这隐含一个前提 —— 空车与载货车停在同一位置。

2026-09-15 实测 CH4 21 辆车的车厢包围盒，这个前提**不成立**：

    横向 x0 / x1 : std 0.083 m，极差 0.35 m   <- 很稳，车道约束住了
    纵向 y0      : std 0.864 m，极差 4.00 m   <- 差得多
    纵向 y1      : std 1.444 m，极差 5.50 m

空车若比载货车靠前一米，逐格相减就整体错位一米，结果毫无意义
（车厢中段的货高会被减到厢底以外去）。所以必须先把两朵云对齐。

怎么配准
--------
用**车厢包围盒**作基准，只做平移（车道约束住了朝向，逐通道 yaw 已补偿）：

    dx = 载货车厢 x 中心 - 空车厢 x 中心
    dy = 载货车厢 y0     - 空车厢 y0      <- 用**后沿**，不是前沿

为什么用后沿：俯视雷达在天底附近点最密，越往前掠射角越大。实测车厢
前端（+y）的栅格覆盖率只有 43~71%，后端有 77~88% —— 后沿的位置量得更准。

▲ **配准精度的下限是 0.25 m** —— 车厢盒由 `roi_audit.find_boxes` 给出，
  它沿 y 按 0.25 m 切片，y0 只能定位到这个粒度。
  合成数据实测：真实平移 0.90 m，配准算出 1.00 m，差 0.10 m。
  折合体积约 1 m3 上下的出入。要更准只能细化切片，
  或改用厢壁做互相关配准。

▲ **配准不能盲信**。若两个盒的宽度差 > `W_TOL` 或长度差 > `L_TOL`，
  多半根本不是同一辆车（现场至少有两种车型：车厢 6.25 m 与 10~12 m），
  此时报警并拒绝给结论 —— 宁可不算，也不能给一个错的体积。

空洞怎么办
----------
掠射角下车厢两端总有格子取不到点。两个口径都给出来，不替人选：

    仅有效格  —— 只对两朵云都有数据的格子积分（**偏小**）
    补洞后    —— 载货侧的空洞按该车货面中位高度补（**偏大**）

实测这两者差 10%~16%，是这套方法当前的精度上限。

用法
----
    python3 cli/volume_calc.py --empty 空车.xyz --loaded 载货.xyz
    python3 cli/volume_calc.py --empty CH4.baseline.npz --loaded 载货.xyz
    python3 cli/volume_calc.py --empty 空车.xyz --loaded prep/CH4_152838.npz
    python3 cli/volume_calc.py ... --no-register      # 不配准（与原实现一致）
    python3 cli/volume_calc.py ... --dy 0.35          # 手工指定纵向偏移
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cli import _common                                   # noqa: E402
from app.pointcloud import load_xyz                       # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))
from roi_audit import find_boxes, main_box                # noqa: E402

CELL = 0.10         # 柱状积分的格子（与 detect.compute_volume 默认一致）
MIN_PTS = 3         # 一格至少几个点才算有数据
W_TOL = 0.20        # 两个车厢的宽度差超过它 = 多半不是同一辆车
L_TOL = 1.50        # 长度差同上（分段本身在两端有 ~0.5 m 的不确定）
H_LO, H_HI = 0.3, 5.0   # 取点的高度带（相对地面）
FLOOR_BAND = 0.50       # 厢底带宽：空车云只取这个带内的点建基准面


def register(empty_box, loaded_box, tol_w=W_TOL, tol_l=L_TOL):
    """由两个车厢包围盒算平移量，返回 (dx, dy, [告警...])。

    盒的格式是 (x0, x1, y0, y1)。只做平移 —— 朝向由车道约束，
    逐通道 yaw 已在解码层补过（见 docs/08 §10.9）。

    ★ 纵向用**后沿** y0 对齐，不用前沿：前端掠射角大、覆盖率低（实测 43~71%），
      量出来的 y1 不可靠。
    """
    ex0, ex1, ey0, _ey1 = empty_box
    lx0, lx1, ly0, _ly1 = loaded_box
    ew, lw = ex1 - ex0, lx1 - lx0
    el, ll = empty_box[3] - ey0, loaded_box[3] - ly0

    warn = []
    if abs(ew - lw) > tol_w:
        warn.append("车厢宽度差 %.2f m（空车 %.2f / 载货 %.2f），超过 %.2f m —— "
                    "多半不是同一辆车，不要用这份基线"
                    % (abs(ew - lw), ew, lw, tol_w))
    if abs(el - ll) > tol_l:
        warn.append("车厢长度差 %.2f m（空车 %.2f / 载货 %.2f），超过 %.2f m —— "
                    "可能不是同一辆车，或有一侧的分段不可靠"
                    % (abs(el - ll), el, ll, tol_l))

    dx = (lx0 + lx1) / 2.0 - (ex0 + ex1) / 2.0
    dy = ly0 - ey0
    return float(dx), float(dy), warn


def floor_plane(pts, ground_z, band=FLOOR_BAND):
    """从空车点云里拟合**厢底平面**，返回 (a, b, c, 参与点数, 众数高度)。

    平面写成 h = a*x + b*y + c，h 是离地高度。

    ★ 为什么要平面，而不是逐格取实测值，也不是一条固定高度带：

      1. **厢壁会冒充底板**。`height_map` 每格取中位数，而车厢两侧的厢壁也在盒子里。
         2026-09-15 实测空车 宁A81530M：
             x = -1.23 列        2.42 m   <- 厢壁
             x = -1.03 .. 0.97   1.50~1.59 m   <- 真底板
             x =  1.37 列        3.27 m   <- 厢壁顶沿
         拿厢壁当基准，那几列的货高会少算 0.4~1.7 m，整车少算约 2 m3。

      2. **厢底是斜的**，所以不能用固定高度带去筛。同一辆车实测
         y=-5 处底板 1.35 m、y=+3 处 1.72 m —— 8 米抬了 0.37 m。
         按众数 1.55 开一条 [1.40, 2.05] 的带，会把后半段底板整片切掉：
         实测空车覆盖率从 93% 掉到 71%，体积从 42.8 少算到 35.4 m3。

      平面同时解决这两件事：跟着斜，且能外推到厢壁那几列。
      车厢底板本来就是平的（实测横向起伏 < 0.1 m），这个模型贴合实际。

    做法：先用高度直方图的众数找到底板大致高度，取附近 band 内的点做最小二乘，
    再按残差迭代收紧一次，把混进来的厢壁/残留物剔掉。
    """
    h = pts[:, 2] - ground_z
    if len(h) < 200:
        z0 = float(np.median(h)) if len(h) else 0.0
        return 0.0, 0.0, z0, len(h), z0
    edges = np.arange(max(0.0, float(h.min())), float(h.max()) + 0.05, 0.05)
    cnt, _ = np.histogram(h, bins=edges)
    z0 = float(edges[int(cnt.argmax())])
    m = (h >= z0 - band) & (h <= z0 + band)
    coef = np.array([0.0, 0.0, z0])
    for _ in range(2):
        if m.sum() < 100:
            break
        A = np.column_stack([pts[m, 0], pts[m, 1], np.ones(int(m.sum()))])
        coef, *_ = np.linalg.lstsq(A, h[m], rcond=None)
        resid = h - (coef[0] * pts[:, 0] + coef[1] * pts[:, 1] + coef[2])
        m = np.abs(resid) <= 0.20
    return float(coef[0]), float(coef[1]), float(coef[2]), int(m.sum()), z0


def height_map(pts, x0, y0, nx, ny, cell, ground_z, min_pts=MIN_PTS):
    """把点云打成 (nx, ny) 的高度场（相对地面），没数据的格子是 nan。

    每格取**中位数**而不是最高点：货面有毛刺（散煤、扬尘），最高点会被单点噪声
    带偏；中位数对 min_pts 以上的格子稳得多。
    """
    Z = np.full(nx * ny, np.nan)
    if len(pts) == 0:
        return Z.reshape(nx, ny)
    ix = np.floor((pts[:, 0] - x0) / cell).astype(np.int64)
    iy = np.floor((pts[:, 1] - y0) / cell).astype(np.int64)
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    if not ok.any():
        return Z.reshape(nx, ny)
    ix, iy, h = ix[ok], iy[ok], pts[ok, 2] - ground_z
    k = ix * ny + iy
    order = np.argsort(k, kind="stable")
    k, h = k[order], h[order]
    edges = np.flatnonzero(np.diff(k)) + 1
    for a, b in zip(np.r_[0, edges], np.r_[edges, len(k)]):
        if b - a >= min_pts:
            Z[k[a]] = np.median(h[a:b])
    return Z.reshape(nx, ny)


def interp_fill(Z, iters=200, domain=None):
    """与实时链共用非周期四邻域内插；未知格持续更新，不修改原始观测。

    单侧延伸只是估计，不等于实测或物理上界，也不等同于三角剖分积分。
    domain 可限定传播区域，域外及无实测锚点的连通区域保持未知。
    """
    from app.height_fill import fill_height_holes
    return fill_height_holes(Z, iters=iters, domain=domain)


def columnar_volume(z_load, z_base, cell, fill_holes=False, h_max=None):
    """柱状积分。返回 (体积 m3, 参与格数, 空洞格数, 最大货高, 被截断的格数)。

    高度取 max(0, z_load - z_base)：负值说明比空车还低（噪声或厢底缝隙），按 0 计。

    ★ `h_max` 是**物理上限**。厢内深约 1.7 m，煤冒尖最多再高半米出头；
      实测有约 0.6% 的格子算出 3.7 m 的"货高"，那是载货侧个别格子打到了
      车厢以外的东西（2026-09-15 实测：33 个点落在车厢尾部 y=-4.8 一带，
      高度 4.98 m，比厢壁顶还高 1.7 m）。不截断的话这些格子会虚增体积。
      与 `detect.compute_volume` 把 h 截到基线盒上沿是同一个道理。
    """
    both = np.isfinite(z_load) & np.isfinite(z_base)
    h = np.where(both, z_load - z_base, np.nan)
    h = np.clip(h, 0.0, None)
    clipped = 0
    if h_max is not None and h_max > 0:
        over = np.isfinite(h) & (h > h_max)
        clipped = int(over.sum())
        h = np.where(over, h_max, h)
    holes = int((~both).sum())
    if fill_holes and np.isfinite(h).any():
        h = np.where(np.isfinite(h), h, np.nanmedian(h))
    v = float(np.nansum(h)) * cell * cell
    n = int(np.count_nonzero(np.nan_to_num(h) > 0))
    hmax = float(np.nanmax(h)) if np.isfinite(h).any() else 0.0
    return v, n, holes, hmax, clipped


def _load_cloud(path, ground_z):
    """读一朵云，返回 (点集 Nx3, 车厢盒, 说明)。支持三种来源。"""
    ext = os.path.splitext(path)[1].lower()
    if ext == ".npz":
        z = np.load(path, allow_pickle=True)
        if "Z" in z:                       # 本工具备好的 prep 文件
            box = tuple(float(v) for v in z["box"])
            return None, box, ("prep 网格", z)
        if "cloud" in z:                   # cli/baseline.py 的 EmptyBaseline
            c = np.asarray(z["cloud"], np.float64)
            b = np.asarray(z["box"], np.float64)
            return c, (float(b[0]), float(b[1]), float(b[2]), float(b[3])), ("基线 npz", z)
        raise SystemExit("认不出这个 npz：%s" % path)
    a = np.asarray(load_xyz(path).xyz, dtype=np.float64)
    bs = find_boxes(a, ground_z)
    mi = main_box(bs)
    if mi is None:
        raise SystemExit("在 %s 里没找到车厢 —— 确认这是车停稳时的整帧" % path)
    b = bs[mi]
    return a, (b["x0"], b["x1"], b["y0"], b["y1"]), ("点云", None)


def main(argv=None):
    _common.setup_console()
    ap = argparse.ArgumentParser(description="带配准的载货体积计算")
    ap.add_argument("--empty", required=True, help="空车：整帧 .xyz 或基线 npz")
    ap.add_argument("--loaded", required=True, help="载货：整帧 .xyz 或 prep npz")
    ap.add_argument("--loaded-extra", action="append", default=[],
                    help="载货的其它帧（多帧合并可把覆盖率从约 79%% 提到 85~92%%）")
    ap.add_argument("--ground-z", type=float, default=-5.23)
    ap.add_argument("--cell", type=float, default=CELL)
    ap.add_argument("--sweep", action="store_true",
                    help="扫一遍格子尺寸，看两个口径收敛到哪里（建议常带）")
    ap.add_argument("--no-register", action="store_true", help="不配准（与原实现一致）")
    ap.add_argument("--dy", type=float, default=None, help="手工指定纵向偏移，覆盖自动配准")
    ap.add_argument("--floor-band", type=float, default=FLOOR_BAND,
                    help="厢底带宽（米，默认 %.2f）" % FLOOR_BAND)
    ap.add_argument("--crown", type=float, default=0.8,
                    help="冒尖余量（米）：货高上限 = 厢内深 + 它，默认 0.8")
    ap.add_argument("--no-floor-band", action="store_true",
                    help="不筛厢底带（会把厢壁当成底板，体积偏小）")
    args = ap.parse_args(argv)
    _common.ensure_root()

    G, C = args.ground_z, args.cell
    ec, ebox, (ekind, _ez) = _load_cloud(args.empty, G)
    lc, lbox, (lkind, lz) = _load_cloud(args.loaded, G)
    if ec is None:
        raise SystemExit("空车不能用 prep 网格，需要原始点云或基线 npz")

    print("=" * 74)
    print("空车 : %-8s %s" % (ekind, args.empty))
    print("       车厢盒 x %.2f..%.2f  y %.2f..%.2f  (宽 %.2f 长 %.2f)"
          % (*ebox, ebox[1] - ebox[0], ebox[3] - ebox[2]))
    print("载货 : %-8s %s" % (lkind, args.loaded))
    print("       车厢盒 x %.2f..%.2f  y %.2f..%.2f  (宽 %.2f 长 %.2f)"
          % (*lbox, lbox[1] - lbox[0], lbox[3] - lbox[2]))
    print("=" * 74)

    dx, dy, warn = register(ebox, lbox)
    if args.no_register:
        dx = dy = 0.0
        print("配准 : 已关闭（--no-register）")
    else:
        if args.dy is not None:
            dy = float(args.dy)
            print("配准 : dx %+.3f m   dy %+.3f m（纵向由 --dy 指定）" % (dx, dy))
        else:
            print("配准 : dx %+.3f m   dy %+.3f m（纵向按车厢后沿对齐）" % (dx, dy))
            print("       注意纵向精度下限约 0.25 m（车厢分段的切片粒度），"
                  "折合体积约 1 m3 的出入")
    for w in warn:
        print("  ! %s" % w)
    if any("宽度差" in w for w in warn):
        print("\n拒绝给出体积 —— 宽度对不上说明不是同一辆车，算出来的数没有意义。")
        return 2

    # ---- 网格以**载货**的车厢盒为准（货在哪就算到哪）
    x0, x1, y0, y1 = lbox
    nx = max(1, int(np.ceil((x1 - x0) / C)))
    ny = max(1, int(np.ceil((y1 - y0) / C)))

    def _sel(c, shift):
        h = c[:, 2] - G
        m = (h > H_LO) & (h < H_HI)
        q = c[m].copy()
        q[:, 0] += shift[0]
        q[:, 1] += shift[1]
        return q

    lp = [_sel(lc, (0.0, 0.0))] if lc is not None else []
    for extra in args.loaded_extra:
        q, _b, _k = _load_cloud(extra, G)
        if q is not None:
            lp.append(_sel(q, (0.0, 0.0)))
    if lp:
        z_load = height_map(np.vstack(lp), x0, y0, nx, ny, C, G)
    else:                                   # prep 网格直接用
        z_load = np.asarray(lz["Z"], np.float64)
        if z_load.shape != (nx, ny):
            nx, ny = z_load.shape

    eb_pts = _sel(ec, (dx, dy))
    z_base = height_map(eb_pts, x0, y0, nx, ny, C, G)
    plane_coef = None
    if not args.no_floor_band:
        pa, pb, pc, nfit, z0 = floor_plane(eb_pts, G, args.floor_band)
        plane_coef = (pa, pb, pc)
        gx = (x0 + (np.arange(nx) + 0.5) * C)[:, None]
        gy = (y0 + (np.arange(ny) + 0.5) * C)[None, :]
        plane = pa * gx + pb * gy + pc
        print("\n厢底 : 拟合平面 h = %+.4f*x %+.4f*y %+.3f  （%d 点参与，众数高度 %.2f m）"
              % (pa, pb, pc, nfit, z0))
        print("       两端 %.2f -> %.2f m（离地），%.1f m 长度上抬了 %.2f m"
              % (float(plane[:, 0].mean()), float(plane[:, -1].mean()),
                 y1 - y0, float(plane[:, -1].mean() - plane[:, 0].mean())))
        bad = np.isfinite(z_base) & (z_base > plane + 0.25)
        print("       %d 格实测值比平面高 0.25 m 以上（厢壁/残留物），改用平面值；"
              "另有 %d 格没数据，同样用平面补"
              % (int(bad.sum()), int((~np.isfinite(z_base)).sum())))
        z_base = np.where(np.isfinite(z_base) & ~bad, z_base, plane)

    print("\n%-14s %8s %8s" % ("", "有效格", "覆盖率"))
    for lab, Z in (("空车厢底", z_base), ("载货货面", z_load)):
        f = np.isfinite(Z)
        print("%-14s %8d %7.1f%%" % (lab, f.sum(), 100 * f.mean()))

    # 物理上限：厢内深（厢壁顶 - 底板）再加冒尖余量。
    # ★ 必须先裁到**车厢范围内**再取分位数 —— 整帧里还有远处的墙和门架，
    #   不裁的话 98% 分位会落到那些结构上（实测 4.42 m，而真实厢壁顶只有 3.27 m），
    #   算出的"厢内深"虚高一米多，上限就形同虚设。
    inbox = eb_pts[(eb_pts[:, 0] >= x0) & (eb_pts[:, 0] <= x1) &
                   (eb_pts[:, 1] >= y0) & (eb_pts[:, 1] <= y1)]
    eh = inbox[:, 2] - G
    # ★ 用 95% 分位，不是 98%。2026-09-15 现场给出车厢实际内深 1.8~1.85 m，
    #   拿它反查：95% 分位算出的内深两台车都是 1.85 m（吻合），
    #   98% 分位算出 2.04 m、99% 算出 2.25 m —— 那已经取到上沿以上的东西
    #   （爬梯、挡板、货堆尖）。分位选高了，物理上限就形同虚设。
    rim = float(np.percentile(eh, 95)) if len(eh) else 0.0
    depth = max(0.5, rim - float(np.nanmedian(z_base)))
    h_max = depth + args.crown
    print("  厢内深 %.2f m（厢壁顶 %.2f - 底板中位 %.2f），"
          "货高上限按 内深+%.1f = %.2f m"
          % (depth, rim, float(np.nanmedian(z_base)), args.crown, h_max))

    v1, n1, holes, hmax, clip1 = columnar_volume(z_load, z_base, C,
                                                 fill_holes=False, h_max=h_max)
    v2, _n2, _h2, _m2, _c2 = columnar_volume(z_load, z_base, C,
                                             fill_holes=True, h_max=h_max)
    v3 = columnar_volume(interp_fill(z_load), z_base, C, False, h_max)[0]
    print("\n" + "-" * 74)
    print("载货体积")
    print("-" * 74)
    print("  仅有效格 : %8.2f m3   （%d 格参与，%d 格缺数据）" % (v1, n1, holes))
    print("  补洞后   : %8.2f m3   （空洞按中位货高补，差 %+.1f%%）"
          % (v2, 100 * (v2 - v1) / v1 if v1 > 0 else 0.0))
    print("  ★ 插值补 : %8.2f m3   （空洞按邻居插值 —— 推荐用这个）" % v3)
    print("  最大货高 : %8.2f m" % hmax)
    if clip1:
        print("  （%d 格超过物理上限被截断 —— 多半是打到了车厢以外的东西）" % clip1)
    cov_l = float(np.isfinite(z_load).mean())
    print("\n  真值应落在这两个数之间。")
    if cov_l < 0.95:
        print("  ▲ 载货侧覆盖只有 %.0f%%。这不是点不够密 —— 而是 %.2f m 的格子比"
              "车厢两端的**扫描线间距**还细。" % (100 * cov_l, C))
        print("    2026-09-15 实测：车厢中段有 78 条 ring 够得着、相邻点间隔 2 mm；"
              "前端只剩 26 条，中间有一处 0.84 m 的空档 —— 格子整个掉进缝里。")
        print("    用 --sweep 看放大格子后两个口径收敛到哪，那才是可用的结论。")

    if args.sweep:
        print("\n" + "=" * 74)
        print("格子扫描 —— 下界随格子变大往上追、上界基本不动，就说明空格里确实有货")
        print("=" * 74)
        print("  %6s %8s %9s %10s %10s %8s"
              % ("格子", "覆盖率", "有效格", "仅有效格", "补洞后", "区间"))
        lp_all = np.vstack(lp) if lp else None
        for c in (0.10, 0.15, 0.20, 0.25, 0.30):
            mx = max(1, int(np.ceil((x1 - x0) / c)))
            my = max(1, int(np.ceil((y1 - y0) / c)))
            zl2 = (height_map(lp_all, x0, y0, mx, my, c, G)
                   if lp_all is not None else z_load)
            zb2 = height_map(eb_pts, x0, y0, mx, my, c, G)
            if plane_coef is not None:
                pa, pb, pc = plane_coef
                g2x = (x0 + (np.arange(mx) + 0.5) * c)[:, None]
                g2y = (y0 + (np.arange(my) + 0.5) * c)[None, :]
                pl2 = pa * g2x + pb * g2y + pc
                bad2 = np.isfinite(zb2) & (zb2 > pl2 + 0.25)
                zb2 = np.where(np.isfinite(zb2) & ~bad2, zb2, pl2)
            a1 = columnar_volume(zl2, zb2, c, False, h_max)[0]
            a2 = columnar_volume(zl2, zb2, c, True, h_max)[0]
            print("  %6.2f %7.1f%% %9d %10.2f %10.2f %7.1f%%"
                  % (c, 100 * np.isfinite(zl2).mean(), int(np.isfinite(zl2).sum()),
                     a1, a2, 100 * (a2 - a1) / a1 if a1 > 0 else 0.0))
        print("\n  取两个口径差 < 3% 的那一档 —— 那一档的数才是可用的结论。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
