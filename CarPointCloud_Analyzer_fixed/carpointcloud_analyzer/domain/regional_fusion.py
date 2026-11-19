"""车头双源补全、车厢单源：在最终体素化之前消除跨雷达重复厢体。

只选择已有点，不估计雷达外参、不移动点、不按拉筋成败选源。未能证明分区或
完整来源时保留原融合，并输出未应用原因；后续仍由同一最终补点云计算。
"""

from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.geometry import CoordinateTransform, LocalCarriageExtractor, PointCloudGeometryNormalizer
from carpointcloud_analyzer.domain.heading import cab_end_support
from carpointcloud_analyzer.domain.inner_faces import (
    FACE_CELL_M, FACE_MIN_CELLS, FACE_MIN_POINTS, FACE_SEARCH_HALF_BAND_M, supported_faces,
)
from carpointcloud_analyzer.domain.models import CarriageSplitResult
from carpointcloud_analyzer.domain.utils.cells import unique_cells
from carpointcloud_analyzer.domain.wall_reference import observe_rectangular_body


# 空间覆盖而非点数投票；一侧完整即可供既有对称补点，禁止每根筋分别换来源。
MIN_LONGITUDINAL_COVERAGE = .8
MIN_END_WIDTH_RATIO = .5
MIN_END_HEIGHT_RATIO = .4
INTERIOR_CELL_M = FACE_CELL_M / 2


@dataclass(frozen=True)
class RegionalSelection:
    """一个不可变分区，同时用于轻滤点和未体素化车辆回波。"""

    transform: CoordinateTransform
    cab_sign: int
    cab_boundary: float
    primary: int

    def masks(self, points_a: np.ndarray, points_b: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """主雷达全车保留，另一雷达仅保留已验证前壁搜索带外的车头。"""
        return tuple(
            np.ones(len(points), dtype=bool) if index == self.primary else
            self.cab_sign * (self.transform.to_local(points)[:, 1] - self.cab_boundary) > 0
            for index, points in enumerate((points_a, points_b))
        )


def _coverage(points: np.ndarray, rear: float, front: float,
              w_range: np.ndarray, h_range: np.ndarray) -> dict[str, Any]:
    """以占用格及两端二维跨度验证一台雷达是否看到整段车厢。"""
    margin = FACE_SEARCH_HALF_BAND_M
    body = points[(points[:, 1] >= rear - margin) & (points[:, 1] <= front + margin)]
    if len(body) < FACE_MIN_POINTS:
        return {"eligible": False, "reason": "too_few_body_points", "cells": 0}
    cells = unique_cells(np.floor(body / FACE_CELL_M).astype(np.int64))
    expected = max(1, int(np.ceil((front - rear) / FACE_CELL_M)))
    l_keys = np.floor((body[:, 1] - rear) / FACE_CELL_M).astype(int)
    l_coverage = len(np.unique(l_keys[(l_keys >= 0) & (l_keys < expected)])) / expected
    end_support = []
    for end in (rear, front):
        patch = body[np.abs(body[:, 1] - end) <= margin]
        wh = unique_cells(np.floor(patch[:, [0, 2]] / FACE_CELL_M).astype(int))
        spans = np.ptp(wh, axis=0) * FACE_CELL_M if len(wh) else np.zeros(2)
        end_support.append(bool(
            len(patch) >= FACE_MIN_POINTS and len(wh) >= FACE_MIN_CELLS
            and spans[0] >= max(.8, np.ptp(w_range) * MIN_END_WIDTH_RATIO)
            and spans[1] >= max(.4, np.ptp(h_range) * MIN_END_HEIGHT_RATIO)
        ))
    side_coverage = []
    for w in w_range:
        side = body[np.abs(body[:, 0] - w) <= margin]
        keys = np.floor((side[:, 1] - rear) / FACE_CELL_M).astype(int)
        side_coverage.append(len(np.unique(keys[(keys >= 0) & (keys < expected)])) / expected)
    eligible = (all(end_support) and l_coverage >= MIN_LONGITUDINAL_COVERAGE
                and max(side_coverage) >= MIN_LONGITUDINAL_COVERAGE)
    return {"eligible": eligible, "cells": len(cells), "endSupport": end_support,
            "longitudinalCoverage": l_coverage, "sideCoverage": side_coverage}


def _uncovered_interior(primary: np.ndarray, secondary: np.ndarray, rear: float,
                        front: float, w_range: np.ndarray, tolerance: float) -> bool:
    """阻止丢掉另一视角独有的二维内部结构（货面/底板）；细杆双影不作面。"""
    inside = secondary[(secondary[:, 1] > rear) & (secondary[:, 1] < front)
                       & (secondary[:, 0] > w_range[0]) & (secondary[:, 0] < w_range[1])]
    if len(inside) < FACE_MIN_CELLS:
        return False
    distance, _ = cKDTree(primary).query(inside, distance_upper_bound=tolerance, workers=-1)
    missing = inside[~np.isfinite(distance)]
    cells, inverse = unique_cells(np.floor(missing / INTERIOR_CELL_M).astype(int), return_inverse=True)
    if len(cells) < FACE_MIN_CELLS:
        return False
    pairs = cKDTree(cells).query_pairs(1, p=np.inf, output_type="ndarray")
    graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])),
                       shape=(len(cells), len(cells))).tocsr()
    _, labels = connected_components(graph, directed=False)
    groups = np.split(missing[np.argsort(inverse)], np.cumsum(np.bincount(inverse))[:-1])
    centers = np.asarray([np.median(group, axis=0) for group in groups])
    patches = np.split(centers[np.argsort(labels)], np.cumsum(np.bincount(labels))[:-1])
    for patch in patches:
        if len(patch) < FACE_MIN_CELLS:
            continue
        # 包含竖直/倾斜货面；轴向包围盒会把一根斜横筋误当二维货面。
        centered = patch - np.mean(patch, axis=0)
        axes = np.linalg.svd(centered, full_matrices=False)[2]
        spans = np.ptp(centered @ axes.T, axis=0)
        if spans[0] >= 2 * FACE_CELL_M and spans[1] >= INTERIOR_CELL_M:
            return True
    return False


def _cab_boundary(local: np.ndarray, split: CarriageSplitResult, sign: int,
                  w_range: np.ndarray, h_range: np.ndarray) -> float | None:
    """切口覆盖整个斜端面；短箱的外端面仅用于分区，不冒充正式内壁。"""
    end = split.carriage_start_y if sign < 0 else split.split_y
    plane = split.rear_wall_plane if sign < 0 else split.front_wall_plane
    planes = [plane] if plane is not None else [
        face.coefficients for face in supported_faces(
            local[np.abs(local[:, 1] - end) <= FACE_SEARCH_HALF_BAND_M], axis=1)
    ]
    if not planes:
        return None
    outer = max(sign * (plane[0] * w + plane[1] * h + plane[2])
                for plane in planes for w in w_range for h in h_range)
    return sign * (outer + FACE_SEARCH_HALF_BAND_M)


def select_carriage_source(uniform: np.ndarray, points_a: np.ndarray, points_b: np.ndarray,
                           config: Config) -> tuple[RegionalSelection | None, dict[str, Any]]:
    """输入已在独立场地竖直系中；选择整厢单源，不产生任何正式尺寸。"""
    metadata: dict[str, Any] = {"applied": False, "policy": "single_carriage_dual_cab"}
    geometry = PointCloudGeometryNormalizer(config)
    try:
        candidate = geometry.vertical_candidate(uniform, np.array([0., 0., 1.]), "site_frame")
        box = observe_rectangular_body(candidate[0], config) if candidate is not None else None
        if box is not None:
            local, transform = candidate
            split = LocalCarriageExtractor.split_observed_box(local, box)
        else:
            local, transform = geometry.process(uniform, np.array([0., 0., 1.]), "site_frame")
            split = LocalCarriageExtractor(config).split(local)
            if not split.boundaries_are_inner_faces:
                return None, {**metadata, "reason": "carriage_end_faces_unverified"}
    except ValueError as exc:
        return None, {**metadata, "reason": "carriage_partition_unavailable", "detail": str(exc)}
    cab_ends = cab_end_support(local, split)
    if sum(cab_ends) != 1:
        return None, {**metadata, "reason": "cab_direction_ambiguous"}
    rear, front = split.carriage_start_y, split.split_y
    w_range = np.percentile(split.carriage_points[:, 0], [2, 98])
    h_range = np.percentile(split.carriage_points[:, 2], [2, 98])
    sources = [transform.to_local(points) for points in (points_a, points_b)]
    coverage = [_coverage(points, rear, front, w_range, h_range) for points in sources]
    metadata["coverage"] = {"a": coverage[0], "b": coverage[1]}
    eligible = [index for index, item in enumerate(coverage) if item["eligible"]]
    if not eligible:
        return None, {**metadata, "reason": "no_complete_carriage_source"}
    primary = max(eligible, key=lambda index: (coverage[index]["cells"], -index))
    interior_w = w_range + np.array([config.rib_wall_band, -config.rib_wall_band])
    if _uncovered_interior(sources[primary], sources[1-primary], rear, front, interior_w,
                           config.load_height_threshold):
        return None, {**metadata, "reason": "secondary_unique_interior_surface"}
    sign = -1 if cab_ends[0] else 1
    boundary = _cab_boundary(local, split, sign, w_range, h_range)
    if boundary is None:
        return None, {**metadata, "reason": "front_partition_surface_unverified"}
    selection = RegionalSelection(transform, sign, boundary, primary)
    masks = selection.masks(points_a, points_b)
    metadata.update(applied=True, reason="complete_carriage_source_selected",
                    primarySource="a" if primary == 0 else "b",
                    boundaryLocalL=boundary, cabDirection=sign,
                    localFrame=transform.to_dict(), coordinateFrame="temporarySiteFrame",
                    selectedPoints={"a": int(masks[0].sum()), "b": int(masks[1].sum())})
    return selection, metadata
