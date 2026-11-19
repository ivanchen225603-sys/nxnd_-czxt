"""点云文件输出；应用层传入已计算点集，不在这里重做算法。"""
import os
from datetime import datetime
from pathlib import Path
from uuid import uuid4
import numpy as np
import open3d as o3d
from carpointcloud_analyzer.domain.runtime import logger

def write_clean_pointcloud(points: np.ndarray, pcd_path: str) -> str:
    """原子发布最终车辆 PCD；失败向上传播，不能把缺文件报作保存成功。"""
    source = Path(pcd_path)
    out = source.parent / (
        f"{source.stem}_vehicle_clean_"
        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid4().hex[:8]}.pcd"
    )
    cloud = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(points.astype(np.float64))
    )
    temp = out.with_name(f'.{out.stem}.tmp.pcd')
    try:
        if not o3d.io.write_point_cloud(str(temp), cloud, write_ascii=False):
            raise IOError("open3d 写入返回 False")
        temp.replace(out)
        logger.info("纯净车辆点云已保存：%s（%d 点）", out, len(points))
        return str(out)
    finally:
        temp.unlink(missing_ok=True)


def write_merged_pcd(path: Path, points: np.ndarray) -> None:
    """写出已完成融合的内部 XYZ 点集。"""
    values = np.asarray(points, dtype=float)

    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_name(f".{path.stem}.{uuid4().hex}.tmp.pcd")
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(values))
    try:
        if not o3d.io.write_point_cloud(
            str(temp_path), cloud, write_ascii=False
        ):
            raise IOError(f"could not write merged PCD: {temp_path}")
        os.replace(temp_path, path)
    finally:
        try:
            temp_path.unlink(missing_ok=True)
        except OSError:
            logger.warning("could not remove temporary merged PCD: %s", temp_path)
