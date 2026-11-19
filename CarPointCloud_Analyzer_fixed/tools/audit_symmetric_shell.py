"""离线导出完整对称四壁及实测融合云，验证 PCD 回读覆盖并绘制前壁修复对比。"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from carpointcloud_analyzer import Config
from carpointcloud_analyzer import PointCloudAnalyzer
from carpointcloud_analyzer.domain.box_geometry import rotate_wl
from carpointcloud_analyzer.domain.fusion import Extrinsic
from carpointcloud_analyzer.domain.wall_reference import complete_rectangular_wall_reference


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', type=Path, required=True)
    parser.add_argument('--previous', type=Path, required=True)
    parser.add_argument('--extrinsic', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    hashes = {str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in [args.pcd, args.previous, args.extrinsic]}
    ext = Extrinsic.load(str(args.extrinsic))
    cfg = Config(input_unit='m', scene_preprocessing_enable=False,
                 weighbridge_plane_source=tuple(ext.weighbridge_plane_a),
                 weighbridge_plane_orientation_verified=True, scene_auto_level_required=True)
    analyzer = PointCloudAnalyzer(cfg)
    raw = analyzer.loader.load(str(args.pcd))
    leveled, plane, _, _ = analyzer._level_scene(raw)
    measured, transform = analyzer.geometry.vertical_candidate(leveled, plane[:3], 'weighbridge')
    reference = complete_rectangular_wall_reference(measured, cfg)
    if reference.box is None or not reference.metadata['profileMatched']:
        raise ValueError('当前数据没有匹配新车型')
    yaw = np.deg2rad(reference.metadata['yawDeg'])
    box = reference.box
    synthetic = analyzer._restore_source_coordinates(reference.points, transform, analyzer._last_site_frame)
    lip = reference.metadata.get('frontLip', {})
    lip_count = lip.get('synthesizedPoints', 0)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    for name, points, original_count in [
        ('completed_vehicle_height_2_9m.pcd', np.vstack([raw, synthetic]), len(raw)),
        ('symmetric_four_walls_only.pcd', synthetic, 0),
    ]:
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        colors = np.tile([1., .42, .05], (len(points), 1))
        colors[:original_count] = [.4, .55, .65]
        if lip_count:
            colors[-lip_count:] = [.1, .75, .3]
        cloud.colors = o3d.utility.Vector3dVector(colors)
        path = args.output_dir / name
        if not o3d.io.write_point_cloud(str(path), cloud):
            raise IOError(path)
        reread = np.asarray(o3d.io.read_point_cloud(str(path)).points)
        np.testing.assert_allclose(reread, points, atol=2e-6, rtol=0)
    # 对写出的 float32 PCD 回读再检验；不只核查内存中的生成模型。
    def aligned(source):
        return rotate_wl(transform.to_local(analyzer._last_site_frame.transform(source)), -yaw)
    q = aligned(reread)
    tree = cKDTree(q)
    mirrored = q.copy()
    mirrored[:, 0] = sum(box.w_range) - mirrored[:, 0]
    mirror_error = float(tree.query(mirrored)[0].max())
    axes = [np.linspace(lo, hi, int(np.ceil((hi - lo) / cfg.carriage_box_surface_spacing)) + 1)
            for lo, hi in [box.w_range, (box.rear_l, box.front_l), box.h_range]]
    coverage = {}
    for name, axis, value in [('left', 0, axes[0][0]), ('right', 0, axes[0][-1]),
                              ('lowEnd', 1, axes[1][0]), ('highEnd', 1, axes[1][-1])]:
        free = [i for i in range(3) if i != axis]
        aa, bb = np.meshgrid(axes[free[0]], axes[free[1]], indexing='ij')
        expected = np.empty((aa.size, 3))
        expected[:, axis] = value
        expected[:, free[0]], expected[:, free[1]] = aa.ravel(), bb.ravel()
        error = float(tree.query(expected)[0].max())
        coverage[name] = {'expectedGridPointsIncludingEdges': len(expected), 'maxGapM': error}
        assert error < 3e-6, coverage
    assert mirror_error < 3e-6
    assert abs(np.ptp(q[:, 2]) - 2.9) < 3e-6
    if lip_count:
        # 折边格点全部覆盖，包含与前壁相连的一整条共用边及两侧边。
        distance_axis = np.linspace(0, lip['depthM'], int(np.ceil(lip['depthM'] / cfg.carriage_box_surface_spacing)) + 1)
        ww, dd = np.meshgrid(axes[0], distance_axis, indexing='ij')
        expected = np.column_stack([ww.ravel(), lip['frontL'] + lip['inwardSign'] * dd.ravel(),
                                    lip['wallEdgeHeightM'] + (lip['innerEdgeHeightM'] - lip['wallEdgeHeightM'])
                                    * dd.ravel() / lip['depthM']])
        if lip.get('innerDropGenerated'):
            height_axis = np.linspace(lip['innerDropLowerHeightM'], lip['innerEdgeHeightM'],
                int(np.ceil((lip['innerEdgeHeightM'] - lip['innerDropLowerHeightM']) / cfg.carriage_box_surface_spacing)) + 1)
            ww, hh = np.meshgrid(axes[0], height_axis, indexing='ij')
            expected = np.vstack([expected, np.column_stack([ww.ravel(),
                np.full(ww.size, lip['frontL'] + lip['inwardSign'] * lip['depthM']), hh.ravel()])])
        coverage['frontTopLip'] = {'expectedGridPointsIncludingEdges': len(expected),
                                   'maxGapM': float(tree.query(expected)[0].max())}
        assert coverage['frontTopLip']['maxGapM'] < 3e-6
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(synthetic[-lip_count:]))
        cloud.paint_uniform_color([.1, .75, .3])
        if not o3d.io.write_point_cloud(str(args.output_dir / 'front_top_lip_only.pcd'), cloud):
            raise IOError('折边单独 PCD 写入失败')
    for path, digest in hashes.items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == digest
    report = {'sourceSha256': hashes, 'wallCompletion': reference.metadata,
              'originalPointCount': len(raw), 'combinedPointCount': len(raw) + len(synthetic),
              'pcdReadbackMirrorMaxErrorM': mirror_error, 'pcdReadbackSurfaceCoverage': coverage,
              'coordinateFrame': 'CAM_004', 'syntheticPointsUsedForMeasuredMetrics': 0}
    (args.output_dir / 'audit.json').write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
    previous_cloud = o3d.io.read_point_cloud(str(args.previous))
    previous_colors = np.asarray(previous_cloud.colors)
    if len(previous_colors) != len(previous_cloud.points):
        raise ValueError('对照 PCD 需包含区分实测和补点的颜色')
    previous_synthetic = np.asarray(previous_cloud.points)[previous_colors[:, 0] > .65]
    plot(aligned(raw), aligned(previous_synthetic), q, box, args.output_dir / 'front_wall_comparison.png')
    if lip_count:
        plot_lip(aligned(raw), aligned(previous_synthetic), q, lip, args.output_dir / 'front_lip_comparison.png')
    print(json.dumps(report, indent=2, ensure_ascii=True))


def plot(measured, previous, current, box, destination):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.family'] = ['Microsoft YaHei']
    plt.rcParams['axes.unicode_minus'] = False
    center_w = sum(box.w_range) / 2
    low_support = np.sum(measured[:, 1] < box.rear_l - 1)
    high_support = np.sum(measured[:, 1] > box.front_l + 1)
    if max(low_support, high_support) <= max(500, min(low_support, high_support) * 3):
        raise ValueError('不能确认车头方向，不能把端壁对比图标为前壁')
    front = box.rear_l if low_support > high_support else box.front_l
    def crop(p):
        return p[(p[:, 1] > box.rear_l - .25) & (p[:, 1] < box.front_l + .25)
                 & (p[:, 0] > box.w_range[0] - .25) & (p[:, 0] < box.w_range[1] + .25)
                 & (p[:, 2] > box.h_range[0] - .15) & (p[:, 2] < box.h_range[1] + .15)]
    measured = crop(measured)[::4]
    fig = plt.figure(figsize=(14, 10), facecolor='white')
    for index, (points, title) in enumerate([(previous, '对照模型'),
                                           (current, '当前模型：整体高 2.900 m')]):
        ax = fig.add_subplot(2, 2, index + 1, projection='3d')
        for p, color, alpha in [(measured, '#6f8c9f', .35), (points, '#ef7a24', .65)]:
            ax.scatter(p[:, 1] - box.rear_l, p[:, 0] - center_w, p[:, 2] - box.h_range[0],
                       c=color, s=.6, alpha=alpha, depthshade=False)
        ax.set_box_aspect([box.length, box.width, 2.9])
        ax.view_init(elev=24, azim=-52)
        ax.set(xlabel='长向（m）', ylabel='宽向（m）', zlabel='模型高度（m）', title=title,
               xlim=(-.2,box.length+.2), ylim=(-box.width/2-.2,box.width/2+.2), zlim=(-.1,3.05))
        ax = fig.add_subplot(2, 2, index + 3)
        selected = points[np.abs(points[:, 1] - front) < .015]
        ax.scatter(selected[:, 0] - center_w, selected[:, 2] - box.h_range[0],
                   s=2, c='#ef7a24', label='四壁模型网格')
        ax.set(xlabel='左右宽向（m）', ylabel='模型高度（m）', title='前壁补点单独检查（不以实测点遮盖缺口）',
               xlim=(-box.width/2-.1,box.width/2+.1), ylim=(-.05,3.0))
        ax.set_aspect('equal')
        ax.grid(alpha=.15)
    fig.suptitle('新车型：实测点保留；重建模型左右对称；不封闭车顶、不补底板', fontsize=15)
    fig.tight_layout(rect=(0,0,1,.96))
    fig.savefig(destination, dpi=130)
    plt.close(fig)


def plot_lip(measured, previous, current, lip, destination):
    """只看圈出的前壁顶部，按实际坐标展示内折边，而非再次展示整张竖壁。"""
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    top, front, sign = lip['wallEdgeHeightM'], lip['frontL'], lip['inwardSign']
    def roi(p):
        distance = (p[:, 1] - front) * sign
        return p[(distance > -.04) & (distance < .4) & (p[:, 2] > top - .2)
                 & (p[:, 2] < top + .05) & (p[:, 0] > lip['widthRangeM'][0])
                 & (p[:, 0] < lip['widthRangeM'][1])]
    raw = roi(measured)[::2]
    for ax, data, title in zip(axes[:2], [previous, current], ['修复前：只有竖直前壁', '修复后：绿色为内伸折边']):
        q = roi(data)
        distance = (q[:, 1] - front) * sign
        wall = (distance < .01) | (np.abs(q[:, 0] - lip['widthRangeM'][0]) < .01) | (np.abs(q[:, 0] - lip['widthRangeM'][1]) < .01)
        ax.scatter(raw[:, 0], (raw[:, 1] - front) * sign, s=1, c='#6f8c9f', alpha=.4)
        ax.scatter(q[wall, 0], distance[wall], s=3, c='#ef7a24')
        ax.scatter(q[~wall, 0], distance[~wall], s=5, c='#1aaf5d')
        ax.set(xlabel='左右宽向（m）', ylabel='向厢内伸出（m）', title=title,
               xlim=(lip['widthRangeM'][0]-.05,lip['widthRangeM'][1]+.05), ylim=(-.04,.36))
        ax.grid(alpha=.2)
    ax = axes[2]
    ax.scatter((raw[:, 1] - front) * sign, raw[:, 2] - top, s=2, c='#6f8c9f', alpha=.5, label='实测点')
    ax.plot([0,0], [-.2,0], c='#ef7a24', label='前壁模型')
    ax.plot([0,lip['depthM']], [0,lip['innerEdgeHeightM']-top], c='#1aaf5d', label='折边连接面')
    if lip.get('innerDropGenerated'):
        ax.plot([lip['depthM'],lip['depthM']], [lip['innerEdgeHeightM']-top,lip['innerDropLowerHeightM']-top],
                c='#1aaf5d', label='内缘下折面')
    ax.set(xlabel='向厢内伸出（m）', ylabel='相对上沿高度（m）', title='侧向剖面：连接前壁与实测内缘',
           xlim=(-.04,.36), ylim=(-.2,.05))
    ax.legend()
    ax.grid(alpha=.2)
    fig.suptitle('前壁顶部折边补全：实测点带支持，宽度方向对称延伸；中部仍为开口', fontsize=14)
    fig.tight_layout()
    fig.savefig(destination, dpi=130)
    plt.close(fig)


if __name__ == '__main__':
    main()
