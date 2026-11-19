from typing import Optional

import numpy as np

from carpointcloud_analyzer.domain.carriage import CarriageAnalyzer, CarriageProfile
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.models import (
    LoadStatusResult,
)

class LoadStatusClassifier:
    """按本次点云的内部填充率判定 Empty / Loaded / Uncertain。"""

    def __init__(self, config: Config):
        """使用业务阈值配置创建装载状态分类器。"""
        self.config = config

    def classify(
        self,
        carriage_points: np.ndarray,
        profile: Optional[CarriageProfile] = None,
    ) -> LoadStatusResult:
        """按车厢内部装载填充率判别空车/载货。

        三指标投票（≥2 票判空车）：
            A 内部俯视网格填充率中位数；
            B 填充率 > 0.5 的网格占比；
            C 中部高度带（20%~80%）的点密度 —— 空车点集中在底板，
              载货时物料表面处于中部/上部。
        """
        cfg = self.config
        if profile is None or not profile.valid:
            profile = CarriageAnalyzer(cfg).analyze(carriage_points)

        metrics = {
            'medianFillRatio': round(profile.median_fill_ratio, 3),
            'highCellFraction': round(profile.high_cell_fraction, 3),
            'midBandDensity': round(profile.mid_band_density, 3),
            'wallDepth': round(profile.wall_depth, 3),
            'gridCells': float(profile.n_grid_cells),
            'baselineAvailable': 0.0,
        }

        if not profile.valid:
            return LoadStatusResult(
                'Uncertain', 0.40, 'carriage_interior_not_observable', metrics
            )

        # 栏板深小到物理上不可能是车厢时，说明壁顶根本没测在栏板上。
        # 2026-09-09 现场实测过一次：车辆簇混入台面构筑物，车厢宽被撑到 4.54m，
        # 两侧壁带落到车外地面，wall_top 取成地面高度，wall_depth=0.04m，
        # 而从原始点直接量的实际内深是 2.72m。
        # 这种剖面不是"装满"的证据，而是"没测到车厢"的证据；必须先于满载保护
        # 返回，否则空车会被以 0.85 的置信度报成载货。
        if profile.wall_depth < cfg.sanity_vehicle_height[0]:
            return LoadStatusResult(
                'Uncertain', 0.20, 'carriage_profile_implausible', metrics
            )

        # 满载/苫盖保护：可见表面贴近栏板顶时，看到的"底"其实是货面或苫布，
        # 填充率会被归一化掉，此时直接判载货
        if profile.wall_depth < cfg.min_visible_wall_depth:
            return LoadStatusResult(
                'Loaded', 0.85, 'visible_surface_near_wall_top', metrics
            )

        votes = [
            profile.median_fill_ratio < cfg.empty_fill_ratio_max,
            profile.high_cell_fraction < cfg.empty_high_cell_frac_max,
            profile.mid_band_density < cfg.empty_mid_density_max,
        ]
        n_votes = sum(votes)
        metrics['emptyVotes'] = float(n_votes)
        if n_votes >= 2:
            return LoadStatusResult(
                'Empty', round(n_votes / 3, 2), 'interior_fill_ratio_low', metrics
            )
        if n_votes == 0:
            return LoadStatusResult(
                'Loaded', 0.85, 'interior_fill_ratio_high', metrics
            )
        return LoadStatusResult(
            'Uncertain', 0.45, 'interior_fill_ratio_between_thresholds', metrics
        )
