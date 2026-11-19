"""新车型展示副本：单层、等密度四壁及折边，不参与识别、尺寸或空载判定。"""

from dataclasses import dataclass

import numpy as np

from carpointcloud_analyzer.domain.box_geometry import rotate_wl
from carpointcloud_analyzer.domain.models import BoxCompletionResult
from carpointcloud_analyzer.domain.symmetric_opening import opening_quads, sample_quad
from carpointcloud_analyzer.domain.utils.surfaces import axis_aligned_face

# 展示副本的取值范围。间距下限来自可交互渲染的点预算，上限保证面仍连续；
# 点预算上限对应前端一次加载的可接受体量。都是展示参数，不影响任何测量。
MIN_SPACING_M = 0.01
DEFAULT_SPACING_M = 0.01
DEFAULT_POINT_BUDGET = 550000
MAX_SPACING_M = 0.1
MIN_POINT_BUDGET = 1000
MAX_POINT_BUDGET = 600000
# 单辆车厢体任一方向的合理上限，超过说明传入的不是厢体包围盒。
MAX_BOX_EXTENT_M = 20
# 预算超限时的间距放大下限，保证循环必然收敛。
MIN_SPACING_GROWTH = 1.05
# 替换实测皮层的包络外扩量：横向/纵向留出波纹板厚度，
# 竖直方向下沿只留很小余量（避免吃掉底盘），上沿留出上边梁。
SHELL_REPLACE_MARGIN_M = 0.20
SHELL_REPLACE_MARGIN_BELOW_M = 0.04
SHELL_REPLACE_MARGIN_ABOVE_M = 0.15


@dataclass
class ShellAppearance:
    """展示上下文、合成点和连续网格，始终与测量输入分开。"""

    context: np.ndarray
    shell: np.ndarray
    metadata: dict
    triangles: np.ndarray


def build_shell_appearance(
    measured: np.ndarray, reference: BoxCompletionResult,
    spacing: float = DEFAULT_SPACING_M, max_points: int = DEFAULT_POINT_BUDGET,
) -> ShellAppearance | None:
    """仅消费已识别模型；删点仅作用于展示副本，实测输入和参考模型均不修改。

    去除车厢包络附近的旧实测皮层，保留车头/底盘等上下文。单独的 shell 是
    统一密度的重建表面，不伪造波纹、板厚、底板或顶盖。预算不足时整面降采样，
    不按数组步长抽点（按步长会把有序网格抽成明显条带）。
    """
    if (isinstance(spacing, bool) or not np.isfinite(spacing)
            or not MIN_SPACING_M <= spacing <= MAX_SPACING_M
            or isinstance(max_points, bool) or not isinstance(max_points, int)
            or not MIN_POINT_BUDGET <= max_points <= MAX_POINT_BUDGET):
        raise ValueError("invalid shell appearance spacing or point budget")
    meta = reference.metadata
    if reference.box is None or not meta.get('profileMatched'):
        return None
    box = reference.box
    ranges = [box.w_range, (box.rear_l, box.front_l), box.h_range]
    if not np.isfinite(ranges).all() or any(not 0 < hi - lo <= MAX_BOX_EXTENT_M for lo, hi in ranges):
        raise ValueError("invalid shell appearance bounds")
    lip = meta.get('frontLip', {})
    quads = opening_quads(reference)
    step = float(spacing)
    # 分配 meshgrid 前先约束资源；含公共棱线的上界比实际唯一点数更保守。
    while True:
        counts = [int(np.ceil((hi - lo) / step)) + 1 for lo, hi in ranges]
        nw, nl, nh = counts
        estimate = 2 * (nl + nw) * nh
        if quads:
            for a, b, c, d in quads:
                n = int(np.ceil(max(np.linalg.norm(b-a), np.linalg.norm(c-d)) / step)) + 1
                m = int(np.ceil(max(np.linalg.norm(d-a), np.linalg.norm(c-b)) / step)) + 1
                estimate += n * m
        elif lip.get('completed'):
            depth = lip['depthM']
            slope_height = lip['innerEdgeHeightM'] - lip['wallEdgeHeightM']
            lip_length = float(np.hypot(depth, slope_height))
            drop = lip['innerEdgeHeightM'] - lip['innerDropLowerHeightM']
            estimate += nw * (int(np.ceil(lip_length / step)) + 1)
            if lip.get('innerDropGenerated'):
                estimate += nw * (int(np.ceil(drop / step)) + 1)
        if estimate <= max_points:
            break
        step *= max(MIN_SPACING_GROWTH, np.sqrt(estimate / max_points))
    axes = [np.linspace(lo, hi, n) for (lo, hi), n in zip(ranges, counts)]
    faces = []
    triangles = []

    def add_quad(corners):
        """把一个四边形加入连续显示网格。"""
        # 连续显示面只有两个三角形，不以增加点数换取屏幕覆盖率。
        triangles.extend(np.asarray(corners)[[0, 1, 2, 0, 2, 3]])

    for axis, value in [(0, axes[0][0]), (0, axes[0][-1]), (1, axes[1][0]), (1, axes[1][-1])]:
        free = [i for i in range(3) if i != axis]
        face = axis_aligned_face(axes, axis, value)
        faces.append(face)
        grid = face.reshape(len(axes[free[0]]), len(axes[free[1]]), 3)
        add_quad([grid[0, 0], grid[-1, 0], grid[-1, -1], grid[0, -1]])
    if quads:
        for quad in quads:
            faces.append(sample_quad(quad, step))
            add_quad(quad)
    elif lip.get('completed'):
        distances = np.linspace(0, depth, int(np.ceil(lip_length / step)) + 1)
        ww, dd = np.meshgrid(axes[0], distances, indexing='ij')
        faces.append(np.column_stack([
            ww.ravel(), lip['frontL'] + lip['inwardSign'] * dd.ravel(),
            lip['wallEdgeHeightM'] + slope_height * dd.ravel() / depth,
        ]))
        grid = faces[-1].reshape(len(axes[0]), len(distances), 3)
        add_quad([grid[0, 0], grid[-1, 0], grid[-1, -1], grid[0, -1]])
        if lip.get('innerDropGenerated'):
            hh = np.linspace(lip['innerDropLowerHeightM'], lip['innerEdgeHeightM'],
                             int(np.ceil(drop / step)) + 1)
            ww, hh = np.meshgrid(axes[0], hh, indexing='ij')
            faces.append(np.column_stack([ww.ravel(),
                np.full(ww.size, lip['frontL'] + lip['inwardSign'] * depth), hh.ravel()]))
            grid = faces[-1].reshape(len(axes[0]), -1, 3)
            add_quad([grid[0, 0], grid[-1, 0], grid[-1, -1], grid[0, -1]])
    shell = np.unique(np.vstack(faces), axis=0)
    yaw = np.deg2rad(meta['yawDeg'])
    aligned = rotate_wl(measured, -yaw)
    # 只对副本中的厢体及紧邻波纹/上沿带作替换，底盘和远离车厢的车头保留。
    replace = ((aligned[:, 0] >= box.w_range[0] - SHELL_REPLACE_MARGIN_M)
               & (aligned[:, 0] <= box.w_range[1] + SHELL_REPLACE_MARGIN_M)
               & (aligned[:, 1] >= box.rear_l - SHELL_REPLACE_MARGIN_M)
               & (aligned[:, 1] <= box.front_l + SHELL_REPLACE_MARGIN_M)
               & (aligned[:, 2] >= box.h_range[0] - SHELL_REPLACE_MARGIN_BELOW_M)
               & (aligned[:, 2] <= box.h_range[1] + SHELL_REPLACE_MARGIN_ABOVE_M))
    return ShellAppearance(measured[~replace].copy(), rotate_wl(shell, yaw), {
        'applied': True, 'method': 'uniform_single_shell_v1', 'referenceOnly': True,
        'displayOnly': True, 'measurementInputChanged': False,
        'requestedSpacingM': float(spacing), 'effectiveMaxSpacingM': step,
        'shellPointCount': len(shell), 'retainedContextPointCount': int((~replace).sum()),
        'replacedDisplayPointCount': int(replace.sum()),
        'originalMeasuredPointCount': len(measured), 'maxShellPoints': max_points,
        'roofClosed': False, 'bottomGenerated': False,
        'frontLipIncluded': bool(lip.get('completed')),
        'symmetricOpeningIncluded': bool(quads),
        'note': '展示副本统一重建四壁及已识别折边；非原始回波，不用于重新测量。',
    }, rotate_wl(np.asarray(triangles), yaw))
