"""显式离线四壁补全：原始 PCD 不改动，不连接数据库，不生成业务尺寸。

运行：python -m tools.complete_wall_reference --pcd scan.pcd --output-dir new-directory --plot
PNG/JSON 是本工具的人工复核附件，不是生产分析流水线产物。
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import open3d as o3d

from carpointcloud_analyzer import Config
from carpointcloud_analyzer import PointCloudAnalyzer


def export_reference(pcd_path: Path, output_dir: Path, plot: bool = False):
    """输出原始全场景+合成点、合成点单独 PCD、审计信息；不覆盖既有结果。"""
    analyzer = PointCloudAnalyzer(Config(input_unit="m"))
    source_hash = hashlib.sha256(pcd_path.read_bytes()).hexdigest()
    result = analyzer.analyze_wall_reference(str(pcd_path))
    measured_count = result["measuredPointCount"]
    measured = result["coordinates"][:measured_count]
    synthetic = result.pop("coordinates")[measured_count:]
    if not len(synthetic):
        raise ValueError(f"没有通过补点适用门禁：{result['wallCompletion']}")
    raw = analyzer.loader.load(str(pcd_path))
    if hashlib.sha256(pcd_path.read_bytes()).hexdigest() != source_hash:
        raise ValueError("计算过程中输入文件发生变化，拒绝导出")
    # 强制新建专用目录，避免静默覆盖上次人工复核结果。
    output_dir.mkdir(parents=True, exist_ok=False)
    combined = np.vstack([raw, synthetic])
    for filename, points, original_count in [
        ("completed_reference.pcd", combined, len(raw)),
        ("synthetic_walls_only.pcd", synthetic, 0),
    ]:
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        colors = np.tile([1.0, 0.45, 0.08], (len(points), 1))
        colors[:original_count] = [0.40, 0.55, 0.65]
        cloud.colors = o3d.utility.Vector3dVector(colors)
        if not o3d.io.write_point_cloud(str(output_dir / filename), cloud):
            raise IOError(f"写入失败：{filename}")
    report = {
        **result, "source": str(pcd_path.resolve()), "sourceSha256": source_hash,
        "originalPointCount": len(raw), "completedPointCount": len(combined),
        "coordinateFrame": "input_pcd", "unit": "m",
        "note": "蓝灰为实测，橙色为推断；只供参考，不是尺寸或空载证明。",
    }
    (output_dir / "reference_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    if plot:
        _plot_reference(measured, synthetic, analyzer, output_dir / "comparison.png")
    return report


def _plot_reference(measured, synthetic, analyzer, path):
    """用同一场地坐标绘制前后对比，不重新拟合、不改变点云数据。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    frame = analyzer._last_site_frame
    if frame is not None:
        measured, synthetic = frame.transform(measured), frame.transform(synthetic)
    fig = plt.figure(figsize=(15, 9))
    for index, title in enumerate(["Measured vehicle", "Four-wall reference (orange = inferred)"]):
        ax = fig.add_subplot(2, 2, index + 1, projection="3d")
        q = measured[::max(1, len(measured) // 25000)]
        ax.scatter(q[:, 0], q[:, 1], q[:, 2], s=0.3, c="#607d8b", alpha=0.6)
        if index:
            q = synthetic[::2]
            ax.scatter(q[:, 0], q[:, 1], q[:, 2], s=0.5, c="#ff7314")
        extent = np.maximum(np.ptp(measured, axis=0), 0.1)
        ax.set_box_aspect(extent)
        ax.view_init(elev=23, azim=-60)
        ax.set_title(title)
        ax.set_xlabel("site X (m)")
        ax.set_ylabel("site Y (m)")
        ax.set_zlabel("site Z (m)")
    for index, (x, y, title) in enumerate([(0, 1, "Top: open, no lid"), (0, 2, "Side")]):
        ax = fig.add_subplot(2, 2, index + 3)
        q = measured[::4]
        ax.scatter(q[:, x], q[:, y], s=0.3, c="#607d8b")
        ax.scatter(synthetic[:, x], synthetic[:, y], s=0.3, c="#ff7314")
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.set_title(title)
    fig.suptitle("Reference only: no inferred floor / roof, no business measurements")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--plot", action="store_true", help="附加对比图（需 matplotlib）")
    args = parser.parse_args()
    report = export_reference(args.pcd, args.output_dir, args.plot)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
