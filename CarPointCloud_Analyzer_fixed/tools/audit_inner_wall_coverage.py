"""离线核对残缺原始点云的分侧覆盖与同位置壁间距，不改业务结果或数据库。"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from carpointcloud_analyzer import Config
from carpointcloud_analyzer import PointCloudAnalyzer
from carpointcloud_analyzer.domain.box_geometry import rotate_wl
from carpointcloud_analyzer.domain.fusion import Extrinsic
from carpointcloud_analyzer.domain.wall_reference import complete_rectangular_wall_reference


def cell_samples(points, length_edges, height_edges, minimum=5):
    """同一长向/高度网格内分别取侧壁坐标中位数，避免密集一侧淹没稀疏一侧。"""
    cells = {}
    for i in range(len(length_edges) - 1):
        for j in range(len(height_edges) - 1):
            q = points[(points[:, 1] >= length_edges[i]) & (points[:, 1] < length_edges[i + 1])
                       & (points[:, 2] >= height_edges[j]) & (points[:, 2] < height_edges[j + 1])]
            if len(q) >= minimum:
                cells[i, j] = {'count': len(q), 'medianW': float(np.median(q[:, 0]))}
    return cells


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ['a', 'b', 'fused', 'extrinsic', 'output-dir']:
        parser.add_argument('--' + key, type=Path, required=True)
    args = parser.parse_args()
    sources = [args.a, args.b, args.fused, args.extrinsic]
    hashes = {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    ext = Extrinsic.load(str(args.extrinsic))
    analyzer = PointCloudAnalyzer(Config(input_unit='m', scene_preprocessing_enable=False,
        weighbridge_plane_source=tuple(ext.weighbridge_plane_a),
        weighbridge_plane_orientation_verified=True, scene_auto_level_required=True))
    fused = analyzer.loader.load(str(args.fused))
    leveled, plane, _, _ = analyzer._level_scene(fused)
    measured, transform = analyzer.geometry.vertical_candidate(leveled, plane[:3], 'weighbridge')
    ref = complete_rectangular_wall_reference(measured, analyzer.config)
    if ref.box is None:
        raise ValueError('没有可用于定位检查区域的矩形模型')
    box, meta = ref.box, ref.metadata
    def local(p):
        return rotate_wl(transform.to_local(analyzer._last_site_frame.transform(p)), -np.deg2rad(meta['yawDeg']))
    clouds = {'CAM_004': local(analyzer.loader.load(str(args.a))),
              'CAM_005': local(ext.apply(analyzer.loader.load(str(args.b))))}
    raw = np.vstack(list(clouds.values()))
    clouds['rawCombined'] = raw
    bottom, top = meta['observedHRange']
    length_edges = np.linspace(box.rear_l + .3, box.front_l - .3, 21)
    height_edges = np.linspace(bottom, top, 11)
    # 内宽排除上沿/折边高度带；只比较两侧都有实测点的同一网格。
    inner_height_edges = np.linspace(bottom, top - .25, 11)
    report = {'sourceSha256': hashes, 'syntheticPointsUsed': 0, 'unit': 'm',
              'sideConvention': 'negativeW=code_left; positiveW=code_right; not photo left/right',
              'modelBoundsForROIOnly': meta['bounds'], 'clouds': {}}
    region_points = {}
    for name, p in clouds.items():
        row = {}
        for side, w in zip(['negativeW', 'positiveW'], box.w_range):
            q = p[(abs(p[:, 0] - w) < .3) & (p[:, 1] > length_edges[0])
                  & (p[:, 1] < length_edges[-1]) & (p[:, 2] > bottom - .1) & (p[:, 2] < top + .1)]
            grid, _, _ = np.histogram2d(q[:, 1], q[:, 2], bins=[length_edges, height_edges])
            row[side] = {'regionPointCount': len(q), 'occupiedCellsAtLeast2': int((grid >= 2).sum()),
                         'totalCells': grid.size, 'occupiedRatio': float(np.mean(grid >= 2))}
            if name == 'rawCombined':
                region_points[side] = q
        report['clouds'][name] = row
    samples = [cell_samples(region_points[side], length_edges, inner_height_edges)
               for side in ['negativeW', 'positiveW']]
    shared = sorted(set(samples[0]) & set(samples[1]))
    widths = [samples[1][cell]['medianW'] - samples[0][cell]['medianW'] for cell in shared]
    if len(shared) < 3:
        raise ValueError('左右同时可见区域不足，不从单侧镜像猜测内宽')
    report['width'] = {'method': 'median_of_equal_weight_shared_LH_cell_gaps',
        'referenceSide': 'negativeW', 'sharedCells': len(shared), 'minPointsPerSidePerCell': 5,
        'medianM': float(np.median(widths)), 'minM': float(min(widths)), 'maxM': float(max(widths)),
        'p10p90M': np.percentile(widths, [10, 90]).tolist(),
        'note': '共同可见区域的局部内壁间距参考，不证明缺失区全长净宽或最小净宽。',
        'samples': [{'lengthBin': i, 'heightBin': j, 'widthM': width,
                     'negativeCount': samples[0][i,j]['count'], 'positiveCount': samples[1][i,j]['count']}
                    for (i, j), width in zip(shared, widths)]}
    origins = local(np.vstack([np.zeros((1, 3)), ext.apply(np.zeros((1, 3)))]))
    report['sensorOriginsLocalWLH'] = origins.tolist()
    report['length'] = {'wallReferenceSpanM': box.length, 'innerLengthM': None,
        'note': '前后参考壁间距不等于净内长。雷达原点位于前壁外侧上方，不能将可见前壁回波直接认作内面；缺少前壁厚度/内面定位。'}
    for source, digest in hashes.items():
        assert hashlib.sha256(Path(source).read_bytes()).hexdigest() == digest
    args.output_dir.mkdir(parents=True, exist_ok=False)
    (args.output_dir / 'wall_coverage_audit.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    plot(region_points, box, meta, length_edges, inner_height_edges, shared,
         args.output_dir / 'wall_coverage.png')
    print(json.dumps({k: v for k, v in report.items() if k != 'sourceSha256'}, ensure_ascii=True, indent=2))


def plot(regions, box, meta, length_edges, height_edges, shared, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    plt.rcParams['font.family'] = ['Microsoft YaHei']
    plt.rcParams['axes.unicode_minus'] = False
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True, sharey=True)
    for ax, side, title in zip(axes, ['negativeW', 'positiveW'],
                              ['W负侧（代码 left）：相对完整', 'W正侧（代码 right）：明显残缺']):
        q = regions[side]
        ax.scatter(q[:, 1] - box.rear_l, q[:, 2] - meta['observedHRange'][0], s=1, c='#267b9e')
        for i, j in shared:
            ax.add_patch(Rectangle((length_edges[i] - box.rear_l, height_edges[j] - meta['observedHRange'][0]),
                                  np.diff(length_edges)[i], np.diff(height_edges)[j], fill=False,
                                  edgecolor='#dd791c', linewidth=.8))
        ax.set(title=title, ylabel='相对观测下沿高度（m）', xlim=(0, box.length), ylim=(-.1,2.75))
        ax.set_aspect('equal')
        ax.grid(alpha=.15)
    axes[-1].set_xlabel('由后端向前端的长向位置（m）')
    fig.suptitle('原始回波分侧核查：橙框为两侧同位置同高度均有点的测宽区域；无任何补点', fontsize=13)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
