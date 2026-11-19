from pathlib import Path

import numpy as np
import open3d as o3d

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.runtime import logger


def read_xyz_pcd(file_path: str | Path) -> tuple[np.ndarray, int]:
    """PCD 外部文件边界：读取一次 XYZ，剔除非有限点，返回坐标及剔除数量。"""
    path = Path(file_path)
    if not path.is_file():
        raise FileNotFoundError(f"文件不存在：{file_path}")
    points = np.asarray(o3d.io.read_point_cloud(str(path)).points)
    finite_mask = np.all(np.isfinite(points), axis=1)
    points = points[finite_mask]
    if len(points) == 0:
        raise ValueError(f"点云为空或全部无效：{file_path}")
    return points, int(np.sum(~finite_mask))


class PointCloudLoader:
    """点云加载器。

    内部算法统一使用米。生产链路应通过 ``Config.input_unit`` 明确声明
    输入单位；自动判断仅用于兼容无法补充元数据的历史 PCD。
    """

    def __init__(self, config: Config):
        """初始化统一以米为内部单位的点云加载器。

        Args:
            config: 输入单位和历史自动单位识别配置。
        """
        self.config = config

    def load(self, file_path: str) -> np.ndarray:
        """读取 PCD、剔除非有限点并转换为米制坐标。

        Args:
            file_path: 待分析的本地 PCD 文件路径。

        Returns:
            形状为 ``(N, 3)``、单位为米的有限浮点坐标数组。

        Raises:
            FileNotFoundError: 文件不存在。
            ValueError: 点云为空或全部为非有限点。
        """
        logger.info(f"加载点云：{file_path}")
        points, invalid_count = read_xyz_pcd(file_path)
        if invalid_count:
            logger.warning(f"剔除 NaN/Inf 点 {invalid_count:,} 个")

        # 进入任何几何计算前统一为米，确保外参平移、体素尺寸和业务尺寸同量纲。
        points = self._auto_convert_unit(points)

        logger.info(f"加载完成，有效点数：{len(points):,}")
        return points

    def _auto_convert_unit(self, points: np.ndarray) -> np.ndarray:
        """按配置把输入转换为米。

        ``auto`` 模式不再假定车辆长轴一定是源坐标 Y。三轴方向可能因雷达
        安装姿态不同而互换，因此以三轴最大跨度参与毫米判定。
        """
        cfg = self.config
        if cfg.input_unit == "m":
            return points
        if cfg.input_unit == "mm":
            logger.info("输入单位已声明为 mm，统一转换为 m")
            return points / 1000.0

        abs_max = float(np.max(np.abs(points)))
        max_span = float(np.max(np.ptp(points, axis=0)))

        is_mm = abs_max > cfg.mm_unit_threshold and max_span > cfg.mm_unit_min_span

        if is_mm:
            logger.info(
                "自动识别为 mm（绝对值最大 %.0f，三轴最大跨度 %.0f），转换为 m",
                abs_max,
                max_span,
            )
            return points / 1000.0

        # 坐标量级很大但空间跨度不足时无法可靠猜测；兼容模式保持原值并明确告警。
        if abs_max > cfg.mm_unit_threshold and not is_mm:
            logger.warning(
                "坐标量级偏大（%.1f），但三轴最大跨度仅 %.1f，"
                "auto 模式无法确认是否为毫米，暂按米处理",
                abs_max,
                max_span,
            )
        return points
