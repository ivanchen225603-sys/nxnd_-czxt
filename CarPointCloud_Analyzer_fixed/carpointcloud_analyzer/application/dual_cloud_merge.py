r"""独立的双雷达车辆点云清理与合并工具。

提供内存车辆融合及兼容的独立文件/命令行入口；正式服务也调用内存融合函数。
场地坐标算法位于 ``site_frame``。现场约定如下：

* A 固定为 CAM_004（最终输出坐标系），B 固定为 CAM_005；
* 车辆和两台雷达在两次采集之间完全静止；
* 必须提供已经标定并通过质量检查的 ``CAM_005 -> CAM_004`` 外参；
* 原始 XYZ 轴方向可以变化。本工具先从共同场景估计地面，建立临时 Z-up 坐标系
  进行环境过滤，最终输出仍保持 CAM_004 的原始坐标系。

为什么不是分别过滤 A/B
------------------------
CAM_005 很可能只看到局部车头。如果分别套用“完整货车尺寸”先验，车头会被当作
小簇删除。这里先用固定外参把 B *临时对齐*到 A，在共同坐标系中识别完整车辆，
再把车辆掩码分别应用回 A/B，最后才执行正式体素合并。这样既能先去掉环境点，
又不会因为单台雷达视野不完整而误删车头或车厢。

命令行示例::

    python -m carpointcloud_analyzer.dual_cloud_merge ^
      --pcd-a D:\data\CAM_004.pcd ^
      --pcd-b D:\data\CAM_005.pcd ^
      --extrinsic D:\data\CAM_005_to_CAM_004.json ^
      --output D:\data\vehicle_merged_CAM_004.pcd

缺少外参时本工具会直接失败，绝不使用质心、PCA 或逐帧 ICP 猜测两片点云的位置。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.fusion import Extrinsic, MergeParams, MergeResult, merge
from carpointcloud_analyzer.infrastructure.loader import read_xyz_pcd
from carpointcloud_analyzer.domain.preprocessing import ScenePreprocessor, cluster_dims, filter_scene
from carpointcloud_analyzer.domain.runtime import logger
from carpointcloud_analyzer.domain.regional_fusion import select_carriage_source


from carpointcloud_analyzer.domain.errors import DualCloudMergeError as DualCloudMergeError
from carpointcloud_analyzer.domain.site_frame import (
    SiteFrame as SiteFrame,
    estimate_site_frame as estimate_site_frame,
    site_frame_from_plane as site_frame_from_plane,
    ORIENTATION_BAND_M as ORIENTATION_BAND_M,
    ORIENTATION_DEAD_ZONE_M as ORIENTATION_DEAD_ZONE_M,
)


@dataclass
class VehicleOnlyMergeResult:
    """内存中的双雷达车辆合并结果。"""

    points: np.ndarray
    selected_observed_points: np.ndarray
    fusion: MergeResult
    site_frame: SiteFrame
    points_from_a_before_filter: int
    points_from_b_before_filter: int
    points_from_a_after_filter: int
    points_from_b_after_filter: int
    preprocess_stats: Dict[str, Any] = field(default_factory=dict)
    candidate_sizes: List[Dict[str, Any]] = field(default_factory=list)
    edge_truncated: bool = False

    @property
    def safe_to_publish(self) -> bool:
        """融合是否没有任何阻止正式发布的质量告警。"""
        return not self.fusion.warnings

    def to_dict(self) -> Dict[str, Any]:
        """导出环境过滤、点数和融合质量报告。"""
        return {
            "status": "pass" if self.safe_to_publish else "quality_warning",
            "coordinateFrame": "cameraA",
            "points": {
                "cameraABeforeFilter": self.points_from_a_before_filter,
                "cameraBBeforeFilter": self.points_from_b_before_filter,
                "cameraAAfterFilter": self.points_from_a_after_filter,
                "cameraBAfterFilter": self.points_from_b_after_filter,
                "mergedVehicleOnly": int(len(self.points)),
            },
            "temporarySiteFrame": self.site_frame.to_dict(),
            "environmentFilter": {
                "stats": _json_safe(self.preprocess_stats),
                "vehicleCandidates": _json_safe(self.candidate_sizes),
                "multiVehicleScene": len(self.candidate_sizes) > 1,
                "excludedCandidatePoints": sum(
                    int(item["pointCount"]) for item in self.candidate_sizes
                    if not item.get("selectedForFusion", False)
                ),
                "edgeTruncated": self.edge_truncated,
                # 单台雷达的保留率。两台严重不均时，一台故障就会让覆盖塌掉，
                # 而点数本身看不出这一点。
                "cameraARetention": round(
                    self.points_from_a_after_filter
                    / max(self.points_from_a_before_filter, 1), 4),
                "cameraBRetention": round(
                    self.points_from_b_after_filter
                    / max(self.points_from_b_before_filter, 1), 4),
            },
            "fusion": self.fusion.to_dict(),
        }


def _membership_mask(
    all_points: np.ndarray,
    selected_points: np.ndarray,
    tolerance: float,
) -> np.ndarray:
    """把严格前处理选中的坐标映射回 A/B 原始行号。"""
    if len(selected_points) == 0:
        return np.zeros(len(all_points), dtype=bool)
    tree = cKDTree(selected_points)
    distances, _ = tree.query(
        all_points,
        k=1,
        distance_upper_bound=tolerance,
        workers=-1,
    )
    return np.isfinite(distances) & (distances <= tolerance)


def _vehicle_candidate_report(
    vehicle_points_list: Sequence[np.ndarray], primary_index: int,
) -> List[Dict[str, Any]]:
    """汇总候选并显式标记真正进入融合的那辆车。"""
    report: List[Dict[str, Any]] = []
    for index, candidate in enumerate(vehicle_points_list):
        length, width, height = cluster_dims(candidate)
        report.append(
            {
                "index": index,
                "selectedForFusion": index == primary_index,
                "pointCount": int(len(candidate)),
                "length": round(length, 4),
                "width": round(width, 4),
                "height": round(height, 4),
            }
        )
    return report


def merge_vehicle_clouds(
    points_a: np.ndarray,
    points_b: np.ndarray,
    extrinsic_b_to_a: Extrinsic,
    *,
    camera_a: str = "CAM_004",
    camera_b: str = "CAM_005",
    config: Optional[Config] = None,
    merge_params: Optional[MergeParams] = None,
) -> VehicleOnlyMergeResult:
    """清除环境后合并两台固定雷达的车辆点云。

    返回点保持在 CAM_004/A 原始坐标系。函数不会根据点云内容重新估计 B→A 位姿；
    ``extrinsic_b_to_a`` 是唯一允许的雷达间变换。
    """
    cfg = config or Config()
    # 两台雷达从车头/车尾相向观察时，大部分车厢表面天然互补，重叠度天生就低
    # （实测零漂移时匹配率仅 14.76%）。用它逐帧判定外参漂移是选错了参照物：
    # 匹配率随车形和停车位置浮动，余量只有 23%，误杀正常车辆的概率高于发现真
    # 漂移。外参漂移改由一次性空磅道验收确认 —— 空磅场景两台雷达共视度达 77%，
    # 相隔一天的两组独立数据匹配率只差 0.15pp，而 10cm 漂移会让它掉 12.25pp。
    # 因此这里关闭逐帧门禁，指标继续计算并记录在 overlapNotes 中供审计。
    params = (
        replace(
            merge_params,
            filter_ground_for_overlap=False,
            enforce_overlap_gate=False,
        )
        if merge_params is not None
        else MergeParams(
            min_overlap_match_ratio=0.12,
            filter_ground_for_overlap=False,
            enforce_overlap_gate=False,
        )
    )
    # 质量检查输入已经是共同场景中提取出的 vehicle-only 点云，地面已被剔除。
    # 若再次用 RANSAC 猜“地面”，会随机命中车顶或侧壁，让同一输入的匹配率
    # 在通过/拒绝之间漂移。显式关闭二次地面过滤，同时保留残差和匹配率门禁。

    # 这里只做坐标对齐，不做体素去重，也不把这份临时共同场景写出。
    # 完整车辆识别结束、环境掩码应用回 A/B 后，才调用正式 merge()。
    aligned_b = extrinsic_b_to_a.apply(points_b)
    aligned_scene = np.vstack([points_a, aligned_b])
    if cfg.weighbridge_plane_source is not None:
        # 朝向已由一次性验收确认时不再逐帧反推；否则这里会用当帧看到多少磅台
        # 下方背景来否决一份正确的标定面。
        site_frame = site_frame_from_plane(
            aligned_scene,
            np.asarray(cfg.weighbridge_plane_source, dtype=float),
            validate_orientation=(
                not cfg.weighbridge_plane_orientation_verified
            ),
        )
    else:
        site_frame = estimate_site_frame(aligned_scene)
    site_scene = site_frame.transform(aligned_scene)

    # 必须使用会抛错的严格入口。ScenePreprocessor.process() 会在失败时返回原场景，
    # 对“vehicle_only 输出”来说会把环境静默写入正式文件。
    # site_scene 的 Z 就是"离地磅标定面的高度"，据此可以排除磅坑/地沟类结构。
    # 单文件路径没有这个保证，因此该开关只在这里打开。
    preprocess_params = replace(
        ScenePreprocessor(cfg).params(), z_is_datum_height=True
    )
    try:
        filtered = filter_scene(site_scene, preprocess_params)
    except Exception as exc:
        raise DualCloudMergeError(f"共同场景未能可靠提取车辆：{exc}") from exc

    candidate_sizes = _vehicle_candidate_report(
        filtered.vehicle_points_list, filtered.primary_index,
    )
    selected = _membership_mask(site_scene, filtered.primary_vehicle_points, tolerance=1e-7)

    split = len(points_a)
    keep_a = selected[:split]
    keep_b = selected[split:]
    vehicle_a = points_a[keep_a]
    vehicle_b = points_b[keep_b]

    # fusion._overlap_stats 会利用“离地高度”排除地面；因此质量检查也必须在临时
    # Z-up 场地坐标中执行，不能假定 CAM_004 原始 Z 就是竖直方向。把 B→A 与
    # A→site 组合后只调用一次正式 merge，再把体素中心刚体变换回 A。
    a_to_site = np.eye(4)
    a_to_site[:3, :3] = site_frame.axes.T
    a_to_site[:3, 3] = -site_frame.axes.T @ site_frame.origin
    b_to_site = a_to_site @ extrinsic_b_to_a.matrix
    site_extrinsic = Extrinsic(
        matrix=b_to_site,
        baseline_residual=extrinsic_b_to_a.baseline_residual,
        camera_a=camera_a,
        camera_b=camera_b,
    )
    vehicle_a_site = site_frame.transform(vehicle_a)
    site_fusion = merge(vehicle_a_site, vehicle_b, site_extrinsic, params)
    selected_observed_a = vehicle_a
    selected_observed_b = aligned_b[keep_b]
    # 完整 A/B 先完成原质量检查；车厢按整段覆盖选一个来源，再形成最终体素。
    # 不得先混合体素再删双影，否则代表点坐标已被另一雷达拉偏。
    regional = {"applied": False, "reason": "fusion_quality_not_passed"}
    if (site_fusion.quality_passed and site_fusion.filtered_points_a is not None
            and site_fusion.aligned_filtered_points_b is not None):
        sources = (site_fusion.filtered_points_a, site_fusion.aligned_filtered_points_b)
        selection, regional = select_carriage_source(site_fusion.points, *sources, cfg)
        if selection is not None:
            masks = selection.masks(*sources)
            selected_points = np.vstack([points[mask] for points, mask in zip(sources, masks)])
            cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(selected_points))
            uniform = np.asarray(cloud.voxel_down_sample(params.merge_voxel_size).points)
            site_fusion = replace(site_fusion, points=uniform, n_after_uniform=len(uniform))
            raw_masks = selection.masks(vehicle_a_site, site_frame.transform(selected_observed_b))
            selected_observed_a = selected_observed_a[raw_masks[0]]
            selected_observed_b = selected_observed_b[raw_masks[1]]
    filtered.stats = {**filtered.stats, "regionalFusion": regional}
    merged_in_a = site_fusion.points @ site_frame.axes.T + site_frame.origin
    fusion_result = MergeResult(
        points=merged_in_a,
        n_from_a=site_fusion.n_from_a,
        n_from_b=site_fusion.n_from_b,
        n_after_uniform=site_fusion.n_after_uniform,
        overlap_rmse=site_fusion.overlap_rmse,
        overlap_match_ratio=site_fusion.overlap_match_ratio,
        warnings=list(site_fusion.warnings),
        n_finite_a=site_fusion.n_finite_a,
        n_finite_b=site_fusion.n_finite_b,
        n_filtered_a=site_fusion.n_filtered_a,
        n_filtered_b=site_fusion.n_filtered_b,
        n_combined=site_fusion.n_combined,
        quality_passed=site_fusion.quality_passed,
        reason=site_fusion.reason,
        overlap_notes=list(site_fusion.overlap_notes),
    )
    return VehicleOnlyMergeResult(
        points=fusion_result.points,
        # 下游需要未体素化的实测回波，但它必须与正式融合使用同一车辆掩码。
        # 不能把原始 A+B 全场景再次传给局部内缘/拉筋测量。
        selected_observed_points=np.vstack([selected_observed_a, selected_observed_b]),
        fusion=fusion_result,
        site_frame=site_frame,
        points_from_a_before_filter=len(points_a),
        points_from_b_before_filter=len(points_b),
        points_from_a_after_filter=len(vehicle_a),
        points_from_b_after_filter=len(vehicle_b),
        preprocess_stats=filtered.stats,
        candidate_sizes=candidate_sizes,
        edge_truncated=filtered.edge_truncated,
    )


def _read_xyz_pcd(path: Path, unit: str) -> Tuple[np.ndarray, Dict[str, Any]]:
    """读取 PCD 的 XYZ 点并按显式或推断单位换算为米。"""
    points, invalid_count = read_xyz_pcd(path)

    robust_spans = np.percentile(points, 99, axis=0) - np.percentile(
        points, 1, axis=0
    )
    detected_unit = unit
    if unit == "auto":
        # 不依赖固定 Y 轴：任意轴的 98% 跨度超过 100，才判断为毫米。
        detected_unit = "mm" if float(np.max(robust_spans)) > 100.0 else "m"
    if detected_unit == "mm":
        points = points / 1000.0

    return points, {
        "path": str(path.resolve()),
        "points": int(len(points)),
        "invalidPointsRemoved": invalid_count,
        "requestedUnit": unit,
        "usedUnit": detected_unit,
        "robustSpansMeters": np.round(
            robust_spans / (1000.0 if detected_unit == "mm" else 1.0), 4
        ).tolist(),
    }


def _load_verified_extrinsic(
    path: Path,
    camera_a: str,
    camera_b: str,
    *,
    require_verified_quality: bool,
) -> Tuple[Extrinsic, Dict[str, Any]]:
    """加载与指定雷达对匹配、并可选要求质量证明的外参。"""
    raw = path.read_bytes()
    document = json.loads(raw.decode("utf-8"))
    extrinsic = Extrinsic.from_dict(document)
    extrinsic.validate_camera_pair(camera_a, camera_b)

    quality = document.get("quality")
    quality_verified = extrinsic.quality is not None and extrinsic.quality.trustworthy
    if require_verified_quality and not quality_verified:
        raise DualCloudMergeError(
            "外参没有可信的标定质量证明（quality.trustworthy != true），"
            "拒绝正式合并"
        )

    return extrinsic, {
        "path": str(path.resolve()),
        "sha256": hashlib.sha256(raw).hexdigest(),
        "frame": document.get("frame"),
        "cameraA": document.get("cameraA"),
        "cameraB": document.get("cameraB"),
        "source": document.get("source", ""),
        "qualityVerified": quality_verified,
        "quality": quality,
        "baselineResidual": document.get("baselineResidual"),
    }


def _write_pcd(path: Path, points: np.ndarray) -> None:
    """写出二进制压缩 PCD。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    o3d.io.write_point_cloud(str(path), cloud, write_ascii=False, compressed=True)


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    """写出 JSON 报告。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_json_safe(payload), ensure_ascii=False, indent=2), encoding="utf-8")


def _json_safe(value: Any) -> Any:
    """递归把 NumPy 值转换为标准 JSON 类型。"""
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def merge_vehicle_pcd_files(
    pcd_path_a: str,
    pcd_path_b: str,
    extrinsic_path: str,
    output_path: str,
    *,
    camera_a: str = "CAM_004",
    camera_b: str = "CAM_005",
    unit_a: str = "auto",
    unit_b: str = "auto",
    require_verified_extrinsic: bool = True,
    reject_fusion_warnings: bool = True,
    report_path: Optional[str] = None,
    merge_params: Optional[MergeParams] = None,
) -> Dict[str, Any]:
    """文件级入口；融合质量通过（或显式接受告警）后才写出正式 PCD。"""
    path_a = Path(pcd_path_a).expanduser().resolve()
    path_b = Path(pcd_path_b).expanduser().resolve()
    output = Path(output_path).expanduser().resolve()

    points_a, input_a = _read_xyz_pcd(path_a, unit_a)
    points_b, input_b = _read_xyz_pcd(path_b, unit_b)
    extrinsic, extrinsic_info = _load_verified_extrinsic(
        Path(extrinsic_path).expanduser().resolve(),
        camera_a,
        camera_b,
        require_verified_quality=require_verified_extrinsic,
    )

    result = merge_vehicle_clouds(
        points_a,
        points_b,
        extrinsic,
        camera_a=camera_a,
        camera_b=camera_b,
        merge_params=merge_params,
    )
    report = {
        **result.to_dict(),
        "cameras": {"a": camera_a, "b": camera_b, "outputFrame": camera_a},
        "inputs": {"a": input_a, "b": input_b},
        "extrinsic": extrinsic_info,
        "outputPath": str(output),
    }
    report_target = (
        Path(report_path).expanduser().resolve()
        if report_path
        else output.with_suffix(".merge_report.json")
    )

    if reject_fusion_warnings and result.fusion.warnings:
        report["status"] = "failed"
        report["failureReasons"] = list(result.fusion.warnings)
        _write_json(report_target, report)
        raise DualCloudMergeError(
            "融合质量检查未通过，未生成正式 PCD；详见报告："
            f"{report_target}"
        )

    _write_pcd(output, result.points)
    report["status"] = "pass" if not result.fusion.warnings else "warning_accepted"
    _write_json(report_target, report)
    logger.info("双雷达车辆点云已写入：%s", output)
    logger.info("合并质量报告已写入：%s", report_target)
    return report


def _build_parser() -> argparse.ArgumentParser:
    """创建独立双雷达合并命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        description=(
            "清除环境后，把固定 CAM_005 点云变换并合并到 CAM_004 坐标系。"
            "必须提供经过标定的 B_to_A 外参。"
        )
    )
    parser.add_argument("--pcd-a", required=True, help="CAM_004/A 的 PCD 路径")
    parser.add_argument("--pcd-b", required=True, help="CAM_005/B 的 PCD 路径")
    parser.add_argument(
        "--extrinsic",
        required=True,
        help="CAM_005 -> CAM_004 的 B_to_A 外参 JSON",
    )
    parser.add_argument("--output", required=True, help="输出 vehicle-only PCD")
    parser.add_argument("--report", help="质量报告 JSON；默认与输出 PCD 同目录")
    parser.add_argument("--camera-a", default="CAM_004")
    parser.add_argument("--camera-b", default="CAM_005")
    parser.add_argument("--unit-a", choices=("auto", "m", "mm"), default="auto")
    parser.add_argument("--unit-b", choices=("auto", "m", "mm"), default="auto")
    parser.add_argument(
        "--merge-voxel",
        type=float,
        default=0.02,
        help="最终密度均匀化体素尺寸，单位 m（默认 0.02）",
    )
    parser.add_argument(
        "--allow-unverified-extrinsic",
        action="store_true",
        help="仅供开发调试：允许没有 trustworthy 质量证明的外参",
    )
    parser.add_argument(
        "--allow-fusion-warning",
        action="store_true",
        help="仅供人工调试：质量告警时仍写出 PCD，报告会标记 warning_accepted",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """执行命令行合并并返回进程退出码。

    Args:
        argv: 可选参数序列；为空时读取当前进程参数。

    Returns:
        成功返回 0，业务或文件错误返回 1。
    """
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.merge_voxel <= 0:
        parser.error("--merge-voxel 必须大于 0")

    report_path = (
        Path(args.report).expanduser().resolve()
        if args.report
        else Path(args.output).expanduser().resolve().with_suffix(".merge_report.json")
    )
    try:
        report = merge_vehicle_pcd_files(
            args.pcd_a,
            args.pcd_b,
            args.extrinsic,
            args.output,
            camera_a=args.camera_a,
            camera_b=args.camera_b,
            unit_a=args.unit_a,
            unit_b=args.unit_b,
            require_verified_extrinsic=not args.allow_unverified_extrinsic,
            reject_fusion_warnings=not args.allow_fusion_warning,
            report_path=str(report_path),
            merge_params=MergeParams(merge_voxel_size=args.merge_voxel),
        )
    except DualCloudMergeError as exc:
        print(f"双雷达合并失败：{exc}", file=sys.stderr)
        return 2

    print(
        "双雷达合并完成："
        f"{report['points']['cameraAAfterFilter']} + "
        f"{report['points']['cameraBAfterFilter']} -> "
        f"{report['points']['mergedVehicleOnly']} 点"
    )
    print(f"输出：{report['outputPath']}")
    print(f"报告：{report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
