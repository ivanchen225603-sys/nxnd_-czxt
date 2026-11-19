"""对已融合车辆 PCD 自动识别新车型并导出整洁展示副本；不写数据库或覆盖源文件。"""

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
from carpointcloud_analyzer.domain.geometry import CoordinateTransform


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', type=Path, required=True, help='CAM_004 坐标系、米制已融合车辆云')
    parser.add_argument('--extrinsic', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--raw-a', type=Path, help='可选：与融合云对应的原始 CAM_004')
    parser.add_argument('--raw-b', type=Path, help='可选：与融合云对应的原始 CAM_005')
    args = parser.parse_args()
    if bool(args.raw_a) != bool(args.raw_b):
        parser.error('--raw-a 与 --raw-b 必须一起指定')
    sources = [args.pcd, args.extrinsic] + ([args.raw_a, args.raw_b] if args.raw_a else [])
    hashes = {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sources}
    ext = Extrinsic.load(str(args.extrinsic))
    analyzer = PointCloudAnalyzer(Config(
        input_unit='m', scene_preprocessing_enable=False,
        weighbridge_plane_source=tuple(ext.weighbridge_plane_a),
        weighbridge_plane_orientation_verified=True, scene_auto_level_required=True,
    ))
    observations = (np.vstack([analyzer.loader.load(str(args.raw_a)),
                               ext.apply(analyzer.loader.load(str(args.raw_b)))]) if args.raw_a else None)
    result = analyzer.analyze_appearance(str(args.pcd), measurement_points=observations)
    completion = result['wallCompletion']
    appearance = completion.get('appearance', {})
    if not appearance.get('applied'):
        raise ValueError('未匹配新车型，不输出强制矩形展示文件')
    count = result['measuredPointCount']
    combined = result.pop('coordinates')
    shell = combined[count:]
    args.output_dir.mkdir(parents=True, exist_ok=False)
    outputs = {}
    for name, points in [('carriage_clean_reconstructed.pcd', shell),
                         ('vehicle_clean_reconstructed.pcd', combined)]:
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        cloud.paint_uniform_color([.38, .68, .78])
        path = args.output_dir / name
        if not o3d.io.write_point_cloud(str(path), cloud):
            raise IOError(f'PCD 写入失败：{path}')
        actual = np.asarray(o3d.io.read_point_cloud(str(path)).points)
        np.testing.assert_allclose(actual, points, atol=3e-6, rtol=0)
        outputs[name] = {'pointCount': len(actual), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
    # 用独立的原参考导出取得姿态，供同坐标、同视角绘图和回读几何检验。
    # 不从展示模型重新估计姿态/尺寸，也不把模型用作新一轮测量输入。
    raw = analyzer.loader.load(str(args.pcd))
    leveled, plane, _, _ = analyzer._level_scene(raw)
    local, transform = analyzer.geometry.vertical_candidate(leveled, plane[:3], 'weighbridge')
    assert isinstance(transform, CoordinateTransform)
    def aligned(p):
        return rotate_wl(transform.to_local(analyzer._last_site_frame.transform(p)),
                          -np.deg2rad(completion['yawDeg']))
    reread_shell = np.asarray(o3d.io.read_point_cloud(str(args.output_dir / 'carriage_clean_reconstructed.pcd')).points)
    q = aligned(reread_shell)
    mirrored = q.copy()
    mirrored[:, 0] = sum(completion['bounds']['wRange']) - mirrored[:, 0]
    mirror_error = float(cKDTree(q).query(mirrored)[0].max())
    assert mirror_error < 5e-6, mirror_error
    assert abs(np.ptp(q[:, 2]) - completion['bounds']['modelHeightM']) < 5e-6
    for path, digest in hashes.items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == digest
    report = {**result, 'sourceSha256': hashes, 'outputs': outputs,
              'coordinateFrame': 'CAM_004', 'pcdUnit': 'm',
              'combinedPointOrder': 'retained_measured_context_then_reconstructed_shell',
              'pcdReadbackMirrorMaxErrorM': mirror_error,
              'warning': '单色展示文件仍为重建副本，不是原始扫描，不可用来重新测量或替换永久空车档案。'}
    (args.output_dir / 'appearance_audit.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    plot(q, completion, args.output_dir / 'carriage_clean_preview.png')
    print(json.dumps(report, ensure_ascii=True, indent=2))


def plot(points, completion, destination):
    """绘制实际导出 PCD 全部点，不以填充面或 AI 图片替代计算结果。"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.family'] = ['Microsoft YaHei']
    plt.rcParams['axes.unicode_minus'] = False
    bounds = completion['bounds']
    fig = plt.figure(figsize=(12, 8), facecolor='#f4f7fa')
    ax = fig.add_subplot(111, projection='3d', facecolor='#f4f7fa')
    ax.scatter(points[:, 1] - bounds['lowLEnd'], points[:, 0] - np.mean(bounds['wRange']),
               points[:, 2] - bounds['hRange'][0], c='#61adc7', s=.7, linewidths=0,
               depthshade=True, rasterized=True)
    ax.set_box_aspect([bounds['lengthM'], bounds['widthM'], bounds['modelHeightM']])
    ax.view_init(elev=32, azim=-125)
    ax.set_proj_type('ortho')
    ax.set(xlabel='长向（m）', ylabel='宽向（m）', zlabel='模型高度（m）',
           title='新车型完整四壁展示副本 · 整体高度 2.900 m · 开顶\n'
                 '统一点密度 / 顶部内缘按观测及对称规则补全；无虚构底板')
    fig.tight_layout()
    fig.savefig(destination, dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
