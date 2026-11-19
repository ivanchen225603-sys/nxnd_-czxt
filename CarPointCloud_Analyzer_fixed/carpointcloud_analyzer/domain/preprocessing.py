"""场景前处理：从完整场景点云中提取单台目标车辆的纯净点云。

背景
----
局部坐标路径（``geometry.py``）此前不做任何环境剔除，直接在完整场景
（地面 / 护栏 / 电线杆 / 碎石 / 杂物）上建立 PCA 坐标系。实测在一份
59m 跨度的采集数据上，识别出的"车辆"局部 L 跨度为 58.74m（即地面跨度），
"车厢"区间长 33m —— 坐标系、车厢端面、拉筋位置全部错误，且置信度仍报 1.00。
本模块补上这一级，输出单台目标车辆的纯净点云。

算法链（各步均已在真实采集数据上验证：空车 / 载煤 / 苫盖 / 单侧扫描不全）
-------------------------------------------------------------------
1. 迭代 RANSAC 地面剔除 —— 法向、高度、水平跨度三重约束，避免把车厢底板、
   货面、苫布顶面误当地面剔除；已去地面却被立面截断时再复检一次低位广域残面；
2. 统计离群滤波 —— 被删点暂存，供第 6 步找回（远端稀疏的真实车辆点会被误删）；
3. 体素降采样 + DBSCAN 聚类 + KDTree 标签回映射（大点云聚类提速 1~2 个数量级）；
4. 货车尺寸先验筛选 + 场景边缘截断剔除（视场边缘的半截目标不是待测车辆）；
5. 车头 / 附属结构找回（俯视雷达下车头常只剩低矮顶面，被单独聚类且不满足尺寸先验）；
5b. 车身段并入（车头与残留地面、路肩立柱聚成同一个尺寸合格的大簇时，第 5 步够不着，
    见 ``_attach_body_run``）；
6. 车辆范围内稀疏点找回（邻接约束区域生长，与车分离的异物连不上）；
7. 车厢底板找回（低底板车辆的底板与地面共面，会被 RANSAC 一并剔除，
   导致车厢内部无点、空车误判为载货）。
8. 场地基准可信时，剔除粘在车辆端部的连续低位薄路面；不能凭一个 DBSCAN
   簇就认定它全是车辆，也不能按固定场地坐标裁车。

``PreprocessParams`` 承载全部阈值，``ScenePreprocessor`` 从 ``Config`` 构造它。
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import open3d as o3d
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from carpointcloud_analyzer.domain.runtime import logger
from carpointcloud_analyzer.domain.utils.planes import consolidate_plane, deterministic_plane_hypothesis


TERMINAL_SURFACE_BIN_M = 0.5
TERMINAL_SURFACE_MIN_LENGTH_M = 2.0
TERMINAL_SURFACE_MAX_HEIGHT_M = 0.25
TERMINAL_SURFACE_MIN_HEIGHT_M = -1.2
TERMINAL_SURFACE_MAX_THICKNESS_M = 0.18
TERMINAL_SURFACE_MIN_WIDTH_M = 1.2


@dataclass
class PreprocessParams:
    """场景前处理阈值集合（默认值均来自真实采集数据上的实测标定）。"""

    enable: bool = True
    min_points: int = 2000  # 低于该点数视为已裁剪好的单车点云，跳过前处理

    # 地面剔除
    ground_dist_threshold: float = 0.12   # RANSAC 平面内点距离阈值（m）
    ground_normal_min_dot: float = 0.80   # 平面法向与 Z 轴夹角余弦下限
    ground_min_inlier_ratio: float = 0.08 # 内点占比低于该值停止迭代
    ground_max_iterations: int = 4        # 最多剔除几层地面（坡面/多级地面）
    ground_z_tolerance: float = 0.6       # 后续平面允许高出首个地面的最大值（m）
    ground_min_extent: float = 6.0        # 地面平面内点最小水平跨度（m）：地面横跨
                                          # 全场景，车顶/苫布/货面只有车宽量级
    ground_multilevel_max: float = 1.0    # 多级地面相对地面基准的最大抬高（m）
    # 地面基准估计（与剔除解耦，仅用于给底板剖面提供高度基准）
    ground_datum_voxel: float = 0.15      # 拟合前的降采样尺寸（m）
    ground_datum_max_planes: int = 5      # 最多考察几个候选平面

    # 离群滤波
    sor_neighbors: int = 20
    sor_std_ratio: float = 2.0

    # 聚类
    dbscan_voxel: float = 0.08
    dbscan_eps: float = 0.45
    dbscan_min_points: int = 10

    # 车辆尺寸先验
    veh_length_range: Tuple[float, float] = (3.5, 30.0)
    veh_width_range: Tuple[float, float] = (1.2, 4.5)
    veh_height_range: Tuple[float, float] = (0.5, 5.5)  # 下限需容纳苫盖车
                                          # （俯视只见苫布面+栏板沿，可见高 ~0.6-0.8m）
    veh_min_points: int = 1500

    # 车头 / 附属结构找回
    attach_min_points: int = 200
    # 车头找回的形状下限：候选簇宽度至少要有车辆宽度的这个比例。
    # 车头与车厢同宽量级，场地立柱只有 0.5m —— 距离判据分不开，形状可以。
    attach_min_width_ratio: float = 0.5
    attach_gap_length: float = 3.0        # 沿车长外扩（车头长度 + 车头车厢间隙）
    attach_gap_width: float = 0.5
    attach_gap_z: float = 0.8
    attach_inside_frac: float = 0.6

    # 车身段并入（步骤 5b）
    body_run_enable: bool = True
    body_run_bin: float = 0.20
    body_run_gap: float = 2.0
    body_run_width_frac: float = 0.40
    body_run_min_points: int = 20

    # 场景边缘截断剔除
    scene_edge_filter: bool = True
    scene_edge_margin: float = 1.5

    # 远端稀疏点找回
    rescue_nn_radius: float = 0.6
    rescue_gap_length: float = 6.0
    rescue_max_iterations: int = 30

    # 车厢底板找回
    floor_restore_enable: bool = True
    floor_restore_max_clearance: float = 0.5

    # 可信高度基准。仅当场景已转到以地磅标定面为 Z=0 的场地系时为 True
    # （双雷达合并路径）；单文件路径的 Z 只是输入轴向，不能据此判深度。
    z_is_datum_height: bool = False
    # 车辆候选允许低于标定面的最大深度。磅台面比周边地坪高约 0.75m，车头停在
    # 地坪上时整段车架都在标定面以下，容差必须显著大于这个落差。而磅坑、地沟、
    # 多路径伪影会低到数米：实测一个 13.3m 长、低至 -6.5m 的结构通过了尺寸先验，
    # 只因点数不及主车辆才没被选成目标车辆。
    max_depth_below_datum: float = 1.5


def estimate_ground_plane(
    points: np.ndarray, params: PreprocessParams
) -> Optional[np.ndarray]:
    """估计真实地平面，仅作高度基准，不参与点的取舍。

    必须与 ``filter_scene`` 的地面剔除解耦：剔除逻辑里，与地面共面的车厢底板会被
    "车辆表面保护"判据保住，真实地面则可能在后续按非车辆簇丢弃 —— 实测这批数据
    出现过 ``地面剔除：0 点``，整条链路拿不到地平面系数。

    判据：法向近垂直（近水平面）+ 水平跨度横跨全场景（车顶/底板/货面只有车宽量级）。
    多个候选时取**位置最低**的那个，避免把抬高的站台或车厢底板当地面。

    Args:
        points: (N,3) 完整场景点云。
    Returns:
        (4,) 平面系数 [a,b,c,d]，满足 ax+by+cz+d=0 且法向朝上（c>0）；
        场景中没有地面（例如输入已裁剪到车身）时返回 None。
    """
    if len(points) < 500:
        return None
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    # 地面基准只需平面姿态，降采样后拟合足够准且快得多
    sampled = cloud.voxel_down_sample(params.ground_datum_voxel)
    work = np.asarray(sampled.points)
    if len(work) < 200:
        work = points.copy()

    candidates: List[Tuple[float, np.ndarray]] = []
    for _ in range(params.ground_datum_max_planes):
        if len(work) < 100:
            break
        try:
            plane, inliers = deterministic_plane_hypothesis(work, params.ground_dist_threshold)
        except Exception:
            # 地面基准决定所有离地高度。这里静默退出会让尺寸整体偏移却查不到原因，
            # 因此即使降级路径本身是设计内的，也必须留下痕迹。
            logger.warning(
                "地面基准平面分割失败，已用现有 %d 个候选面继续：剩余点 %d",
                len(candidates),
                len(work),
                exc_info=True,
            )
            break
        inliers = np.asarray(inliers)
        if len(inliers) < 50:
            break
        # 地面基准直接决定所有离地高度，必须可复现：交替重拟合并重选内点，
        # 消除初始内点集合在阈值边缘的偏差；初值由局部确定性采样产生。
        plane, inlier_mask = consolidate_plane(
            work,
            plane,
            inliers,
            params.ground_dist_threshold,
        )
        inlier_points = work[inlier_mask]
        normal_dot_z = abs(plane[2]) / (np.linalg.norm(plane[:3]) + 1e-12)
        span_x, span_y = _horizontal_spans(inlier_points)
        if (normal_dot_z >= params.ground_normal_min_dot
                and min(span_x, span_y) >= params.ground_min_extent):
            candidates.append((float(np.median(inlier_points[:, 2])), plane))
        work = work[~inlier_mask]

    if not candidates:
        return None
    plane = min(candidates, key=lambda item: item[0])[1]
    normal_norm = np.linalg.norm(plane[:3])
    if normal_norm < 1e-9:
        return None
    plane = plane / normal_norm
    if plane[2] < 0:      # 统一法向朝上，使"地面之上"的有符号距离为正
        plane = -plane
    logger.info(
        "地面基准平面：法向 %s，倾角 %.2f°",
        np.round(plane[:3], 4).tolist(),
        float(np.degrees(np.arccos(min(1.0, abs(plane[2]))))),
    )
    return plane


@dataclass
class PreprocessResult:
    """前处理结果。``applied=False`` 表示原样返回输入点云。"""

    vehicle_points: np.ndarray            # 全部车辆有效点云
    primary_vehicle_points: np.ndarray    # 主车辆（点数最多的簇）
    vehicle_points_list: List[np.ndarray] = field(default_factory=list)  # 逐台车辆
    primary_index: int = 0                # 主车辆在 vehicle_points_list 中的下标
    # 真实地平面 [a,b,c,d]（源坐标系，法向朝上）；场景中没有地面时为 None
    ground_plane: Optional[np.ndarray] = None
    applied: bool = True
    reason: str = ""
    # 主车辆贴近场景边界：点云可能被视场截断，尺寸偏小。按设计仍然保留该车辆，
    # 但必须让调用方看到，不能只留在日志里。
    edge_truncated: bool = False
    stats: Dict[str, Any] = field(default_factory=dict)


# ---------------------------------------------------------------------
# 几何工具
# ---------------------------------------------------------------------
def horizontal_pca_frame(
    points: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """在水平面(XY)做 2D PCA，返回局部坐标 local(N,3)、2x2 旋转矩阵、XY 中心。

    local[:,0]=车长方向 L，local[:,1]=车宽方向 W，local[:,2]=高度（保持原 Z）。
    """
    xy = points[:, :2]
    center = xy.mean(axis=0)
    covariance = np.cov((xy - center).T)
    _, vectors = np.linalg.eigh(covariance)
    rotation = np.column_stack([vectors[:, 1], vectors[:, 0]])  # 主轴在前
    return np.column_stack([(xy - center) @ rotation, points[:, 2]]), rotation, center


def _horizontal_spans(points: np.ndarray) -> Tuple[float, float]:
    """点集在 X/Y 上的稳健跨度（2%~98% 分位），用于区分地面与车辆表面。"""
    lo, hi = np.percentile(points[:, :2], [2, 98], axis=0)
    return float(hi[0] - lo[0]), float(hi[1] - lo[1])


def cluster_dims(cluster: np.ndarray) -> Tuple[float, float, float]:
    """按水平 PCA 主轴计算簇的 长/宽/高（1%~99% 分位抑制噪点拉伸）。"""
    local, _, _ = horizontal_pca_frame(cluster)
    low, high = np.percentile(local, [1.0, 99.0], axis=0)
    return float(high[0] - low[0]), float(high[1] - low[1]), float(high[2] - low[2])


# ---------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------
@dataclass
class _GroundSplit:
    """地面剔除的结果。``removed`` 留着给步骤 7 找回与地面共面的车厢底板。"""

    remaining: np.ndarray
    removed: np.ndarray
    n_removed: int


def _residual_ground_mask(
    points: np.ndarray,
    params: PreprocessParams,
    ground_datum: Optional[np.ndarray],
) -> np.ndarray:
    """在独立地面基准附近复检一次广域地面，返回原点行号掩码。

    全场平面循环遇到大侧壁后可能结束（低位带已不足一层地面、立面让位用尽等），
    少量磅台残面因而把立柱和车头连成一个 DBSCAN 簇。只在既有多级地面高度带内
    拟合，避免侧壁密度压过地面；内点比例的分母也仅为该低位候选集。深于地坪
    一个距离容差的回波不作地面证据。

    最终拟合必须近水平，且水平 PCA 两个方向的稳健跨度都达到广域地面下限。
    长窄底板即使斜放也不能满足次轴跨度。无独立基准或证据不足时原样保留，
    不裁车宽、不扩展到候选高度带以外，也不改写地面基准或真实点坐标。
    """
    remove = np.zeros(len(points), dtype=bool)
    if ground_datum is None or len(points) < 100:
        return remove
    height = points @ ground_datum[:3] + ground_datum[3]
    indices = np.flatnonzero(
        (height >= -params.ground_dist_threshold)
        & (height <= params.ground_multilevel_max)
    )
    if len(indices) < 100:
        return remove
    candidates = points[indices]
    try:
        plane, inliers = deterministic_plane_hypothesis(
            candidates, params.ground_dist_threshold
        )
        plane, mask = consolidate_plane(
            candidates, plane, inliers, params.ground_dist_threshold
        )
        normal_dot_z = abs(plane[2]) / (np.linalg.norm(plane[:3]) + 1e-12)
        if (int(mask.sum()) < 100
                or float(mask.mean()) < params.ground_min_inlier_ratio
                or normal_dot_z < params.ground_normal_min_dot):
            logger.debug("低位残留地面复检：最终法向或支持点不足，保留候选点")
            return remove
        local, _, _ = horizontal_pca_frame(candidates[mask])
        low, high = np.percentile(local[:, :2], [2.0, 98.0], axis=0)
        spans = high - low
        if float(spans.min()) < params.ground_min_extent:
            logger.debug("低位残留地面复检：非广域面（%.2f×%.2fm），保留候选点", *spans)
            return remove
    except Exception:
        logger.warning("低位残留地面拟合失败，保留候选点", exc_info=True)
        return remove
    remove[indices[mask]] = True
    logger.info(
        "低位广域残留地面剔除：%d 点（水平主/次轴跨度 %.2f×%.2fm）",
        int(remove.sum()), *spans,
    )
    return remove


# 立面让位次数上限。让位的立面不计入地面层数预算，但每次让位都多一次全场平面
# 拟合，必须有界。09-24 问题帧在第 1 个立面让位后的下一轮即拿到磅台面。
MAX_SET_ASIDE_FACADES = 4


def _ground_band_share(
    points: np.ndarray, params: PreprocessParams, ground_datum: np.ndarray
) -> float:
    """地面基准以上多级地面高度带（与残面复检同一高度带）内的点占比。"""
    height = points @ ground_datum[:3] + ground_datum[3]
    band = ((height >= -params.ground_dist_threshold)
            & (height <= params.ground_multilevel_max))
    return float(band.mean()) if len(points) else 0.0


def _is_level_ground_layer(
    inlier_points: np.ndarray, params: PreprocessParams, ground_datum: np.ndarray
) -> bool:
    """内点 2~98 分位整层落在多级地面高度带内（漏剔的磅台面这类整层地面）。"""
    height = inlier_points @ ground_datum[:3] + ground_datum[3]
    low, high = np.percentile(height, [2.0, 98.0])
    return bool(low >= -params.ground_dist_threshold
                and high <= params.ground_multilevel_max)


def _remove_ground(
    points: np.ndarray,
    params: PreprocessParams,
    ground_datum: Optional[np.ndarray] = None,
) -> _GroundSplit:
    """步骤 1：迭代 RANSAC 剔除地面。

    三重约束，缺一不可：
        * 法向近垂直 —— 否则车厢侧壁这类大平面会被当地面删掉；
        * 水平跨度横跨全场景 —— 车顶/苫布/货面只有车宽量级，跨度不足者
          先"保护"移出待选（避免 RANSAC 反复命中同一平面），循环后原样放回；
        * 高度不显著高于地面基准 —— 挡住车厢底板/货面这类抬高的水平面
          （无基准时退化为"不显著高于首个地面"）。

    窄跨度保护还须叠加"离地够高"这一条，否则**磅台面**会被保护成车辆表面：
    它比周边地坪只高 0.65~0.70m、宽仅 4m，实测因此和车辆聚成一簇，主车辆簇
    L 被拉到 25.9m、``cabToFrontWall`` 塌成 0.17m。车厢底板离地 2.2~2.3m，
    两者之间有 1.5m 余量。把低底板误判成地面反而安全 —— 步骤 7
    ``_restore_carriage_floor`` 会把车辆足迹内的地面共面点找回。

    立面让位：平面按内点数排序，立面（车厢侧壁、墙体）不是地面，但也不能让它
    结束整轮剔除。2026-09-24 实测：车停得靠前时露出的磅台面变小，一面 27m 的
    立面排到它前面，旧实现在此停止，磅台面 4.6 万点一个未剔，和车连成 28m 的簇，
    车厢底板基准被取到台面上，空车被判 Loaded。现在立面暂时移出待选、继续找低位
    地面，循环结束后按原行号放回。只在已剔过地面、有独立基准、低位地面带仍有
    一层地面量级（``ground_min_inlier_ratio``）的点时让位；其余情形仍在立面处停止。
    让位后只剔内点 2~98 分位整层落在地面带内的水平层，遇到别的面即停：按内点数
    排在立面之后的"地面"常是跨几十米、略倾斜的拟合，只看中位数会切进车身。
    让位后若再没剔到地面，恢复首次遇到立面时的状态。两种情况都与改动前逐点一致，
    只有真的多剔出地面时结果才改变。

    Args:
        points: (N,3) 场景点云。
        params: 阈值集合。
        ground_datum: 步骤 0 估出的地平面系数 ``[a,b,c,d]``（法向朝上、模为 1）。
            为空时退化为只用跨度判据，即改动前的行为。
    """
    work = points.copy()
    # work 各点在输入中的行号：让位的立面放回时按行号合并，残面复检与后续聚类
    # 看到的点集和点序都与"遇立面即停"时一致。
    rows = np.arange(len(points))
    n_ground = 0
    met_nonhorizontal_plane = False
    ground_z_ref: Optional[float] = None
    removed_parts: List[np.ndarray] = []
    protected_parts: List[np.ndarray] = []
    facade_rows: List[np.ndarray] = []
    set_aside_count = 0
    # 首次让位前的状态。让位后若再没剔到地面，回到这里 —— 结果与遇立面即停
    # 逐点一致，连让位期间新保护的车辆表面也不挪动点序。
    before_set_aside: Optional[Tuple[np.ndarray, np.ndarray, int, int]] = None

    budget = params.ground_max_iterations + 3
    while budget > 0:
        if len(work) < 100:
            break
        try:
            plane, inliers = deterministic_plane_hypothesis(work, params.ground_dist_threshold)
        except Exception:
            # 地面剔除提前结束会让残留地面点混进车厢测量，同样必须留痕。
            logger.warning(
                "地面剔除平面分割失败，已剔除 %d 点后提前结束：剩余点 %d",
                sum(len(part) for part in removed_parts),
                len(work),
                exc_info=True,
            )
            break
        inliers = np.asarray(inliers)
        if len(inliers) == 0:
            break

        ratio = len(inliers) / len(work)
        normal_dot_z = abs(plane[2]) / (np.linalg.norm(plane[:3]) + 1e-12)
        if ratio < params.ground_min_inlier_ratio:
            break
        if normal_dot_z < params.ground_normal_min_dot:
            met_nonhorizontal_plane = True
            may_set_aside = (
                n_ground > 0
                and ground_datum is not None
                and set_aside_count < MAX_SET_ASIDE_FACADES
                and _ground_band_share(work, params, ground_datum)
                >= params.ground_min_inlier_ratio
            )
            if not may_set_aside:
                break
            _, facade_mask = consolidate_plane(
                work, plane, inliers, params.ground_dist_threshold
            )
            if not facade_mask.any():
                break
            if before_set_aside is None:
                before_set_aside = (work, rows, len(protected_parts), n_ground)
            logger.info(
                "  立面让位：%d 点（低位地面带仍占 %.1f%%，继续寻找地面）",
                int(facade_mask.sum()),
                100 * _ground_band_share(work, params, ground_datum),
            )
            facade_rows.append(rows[facade_mask])
            set_aside_count += 1
            work, rows = work[~facade_mask], rows[~facade_mask]
            continue
        budget -= 1

        # 内点集合本身每次运行都不同（边界点进出），会让剔除后的车辆点集、
        # 进而所有尺寸在毫米级漂移。确定性初值配合收敛，避免换到另一局部解。
        refined, inlier_mask = consolidate_plane(
            work, plane, inliers, params.ground_dist_threshold
        )
        inlier_points = work[inlier_mask]
        keep_mask = ~inlier_mask
        span_x, span_y = _horizontal_spans(inlier_points)
        above_datum = (
            float(np.median(inlier_points @ ground_datum[:3] + ground_datum[3]))
            if ground_datum is not None else float("inf")
        )
        if (min(span_x, span_y) < params.ground_min_extent
                and above_datum > params.ground_multilevel_max):
            logger.info(
                "  保护疑似车辆表面平面：%d 点（跨度 %.1f×%.1fm，离地 %.2fm）",
                len(inliers), span_x, span_y, above_datum,
            )
            # 倾斜车辆面可能延伸到低位地面。中位高度只能判面，不能把
            # 同面低点也保护起来；地面带内点留下，继续参加原有地面搜索。
            protected_mask = inlier_mask.copy()
            if ground_datum is not None:
                protected_mask[inlier_mask] = (
                    inlier_points @ ground_datum[:3] + ground_datum[3]
                    > params.ground_multilevel_max
                )
            protected_parts.append(work[protected_mask])
            work, rows = work[~protected_mask], rows[~protected_mask]
            continue

        if ground_datum is not None:
            # 有地面基准时以它为准：磅台面、引坡这些多级地面都在 multilevel_max
            # 之内，用首层地面 + ground_z_tolerance 反而会在 0.70m 的磅台处
            # 提前 break，把磅台面留给聚类（实测主车辆簇 L=25.9m）。
            if above_datum > params.ground_multilevel_max:
                break
        elif ground_z_ref is None:
            # 无基准时退化为旧判据：首个大平面还需位于场景低处，否则不认作地面
            inlier_z = float(np.median(inlier_points[:, 2]))
            if inlier_z > float(np.percentile(work[:, 2], 60.0)):
                break
            ground_z_ref = inlier_z
        elif float(np.median(inlier_points[:, 2])) > ground_z_ref + params.ground_z_tolerance:
            break
        if (before_set_aside is not None
                and not _is_level_ground_layer(inlier_points, params, ground_datum)):
            # 让位后只追查整层落在地面带内的漏剔层。排在立面之后的"地面"常是跨
            # 几十米、略倾斜的拟合，中位数落在带内、两端却切进车身（09-24 10:46
            # 实测内点离基准 -2.9~+2.6m），旧的中位数判据会把车身点当地面剔掉。
            break

        n_ground += int(inlier_mask.sum())
        removed_parts.append(inlier_points)
        work, rows = work[keep_mask], rows[keep_mask]
        if facade_rows:
            # 立面按无限大平面取内点，会连带移走它与地面交线 ±阈值 内的一条地面窄带
            # （车侧壁正下方的台面点）。立面若没让位，这些点本会随这层地面收敛进
            # 内点；按同一平面、同一阈值补判，免得窄带把车和地面重新连起来。
            aside = np.concatenate(facade_rows)
            coplanar = (np.abs(points[aside] @ refined[:3] + refined[3])
                        <= params.ground_dist_threshold)
            if coplanar.any():
                n_ground += int(coplanar.sum())
                removed_parts.append(points[aside[coplanar]])
                facade_rows = [aside[~coplanar]]

    if before_set_aside is not None and n_ground == before_set_aside[3]:
        logger.debug("立面让位后未再剔到地面，恢复遇立面即停时的状态")
        work, rows, n_protected, _ = before_set_aside
        del protected_parts[n_protected:]
        facade_rows.clear()
    if facade_rows:
        # 让位的立面原样放回，按输入行号恢复原相对点序。
        rows = np.sort(np.concatenate([rows, *facade_rows]))
        work = points[rows]

    # 仅修复“已剔地面，随后被立面截断”的残面遗漏。不扩大初始地面识别范围：
    # 未剔过任何地面时，保留原车辆表面保护/聚类/低底板找回分支，避免重新分簇。
    # 新剔除点仍进入同一个池，供后续低底板找回；不修改独立高度基准。
    if n_ground and met_nonhorizontal_plane:
        residual_mask = _residual_ground_mask(work, params, ground_datum)
        if residual_mask.any():
            n_ground += int(residual_mask.sum())
            removed_parts.append(work[residual_mask])
            work = work[~residual_mask]
    if protected_parts:
        work = np.vstack([work] + protected_parts)
    logger.info("地面剔除：%d 点", n_ground)
    return _GroundSplit(
        remaining=work,
        removed=np.vstack(removed_parts) if removed_parts else np.empty((0, 3)),
        n_removed=n_ground,
    )


def _remove_outliers(
    work: np.ndarray, params: PreprocessParams
) -> Tuple[o3d.geometry.PointCloud, np.ndarray, np.ndarray]:
    """步骤 2：统计离群滤波。

    被删点必须留下：远端稀疏的真实车辆点也会被 SOR 误删，步骤 6 要找回。

    Returns:
        (滤波后的 open3d 点云, 保留点, 被删点)
    """
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(work))
    filtered, keep_idx = cloud.remove_statistical_outlier(
        nb_neighbors=min(params.sor_neighbors, len(work) - 1),
        std_ratio=params.sor_std_ratio,
    )
    removed_mask = np.ones(len(work), dtype=bool)
    removed_mask[np.asarray(keep_idx)] = False
    removed = work[removed_mask]
    logger.info("离群噪点剔除：%d 点", len(removed))
    return filtered, np.asarray(filtered.points), removed


def _cluster_points(
    filtered: o3d.geometry.PointCloud, work: np.ndarray, params: PreprocessParams
) -> Tuple[np.ndarray, int]:
    """步骤 3：体素降采样 + DBSCAN + KDTree 标签回映射。

    直接对几十万点跑 DBSCAN 要数分钟；降采样后聚类再把标签按最近邻映射回
    全分辨率点云，速度提升 1~2 个数量级，聚类结果实测一致。
    """
    downsampled = filtered.voxel_down_sample(params.dbscan_voxel)
    ds_points = np.asarray(downsampled.points)
    if len(ds_points) == 0:
        raise ValueError("降采样后点云为空")
    labels_ds = np.asarray(
        downsampled.cluster_dbscan(
            eps=params.dbscan_eps,
            min_points=params.dbscan_min_points,
            print_progress=False,
        )
    )
    _, nn_index = cKDTree(ds_points).query(work, k=1, workers=-1)
    labels = labels_ds[nn_index]
    n_clusters = int(labels_ds.max()) + 1 if labels_ds.size else 0
    logger.info(
        "聚类完成：%d 个簇（降采样 %d 点，噪声点 %d）",
        n_clusters, len(ds_points), int((labels < 0).sum()),
    )
    return labels, n_clusters


def _select_vehicle_clusters(
    work: np.ndarray,
    labels: np.ndarray,
    n_clusters: int,
    scene_points: np.ndarray,
    params: PreprocessParams,
) -> Tuple[List[np.ndarray], List[int], bool]:
    """步骤 4：货车尺寸先验筛选 + 场景边缘截断剔除。

    边缘判据用**整幅场景**的边界（不是剔除后的残留），贴边者视为视场边缘被
    截断的远处目标。点数最多的主车辆即使贴边也保留，仅告警 —— 待测车辆可能
    尚未完全驶入视场，整台丢掉比测不准更糟。

    Returns:
        (车辆簇掩码列表, 被拒簇的 cid 列表, 主车辆是否贴边)
    """
    scene_lo = scene_points[:, :2].min(axis=0)
    scene_hi = scene_points[:, :2].max(axis=0)
    margin = params.scene_edge_margin

    def touches_edge(cluster_points: np.ndarray) -> bool:
        """判断候选簇是否接触场景 XY 边缘安全带。"""
        return bool(
            np.any(cluster_points[:, :2].min(axis=0) < scene_lo + margin)
            or np.any(cluster_points[:, :2].max(axis=0) > scene_hi - margin)
        )

    l_range, w_range, h_range = (
        params.veh_length_range, params.veh_width_range, params.veh_height_range
    )
    candidates: List[Tuple[np.ndarray, int, bool]] = []
    rejected_cids: List[int] = []
    counts = np.bincount(labels[labels >= 0], minlength=n_clusters)
    for cid in range(n_clusters):
        n_pts = int(counts[cid])
        if n_pts < params.attach_min_points:
            continue          # 碎片小簇：既不是车辆也不参与车头找回
        mask = labels == cid
        cluster_points = work[mask]
        length, width, height = cluster_dims(cluster_points)
        size_ok = (
            n_pts >= params.veh_min_points
            and l_range[0] <= length <= l_range[1]
            and w_range[0] <= width <= w_range[1]
            and h_range[0] <= height <= h_range[1]
        )
        # 尺寸先验用 1~99 分位抑制噪点，因此极端跨度不会体现在 length/width/height
        # 里。磅坑、地沟、多路径伪影正是这种形态：分位跨度像车，真实跨度深入地下。
        too_deep = False
        if size_ok and params.z_is_datum_height:
            depth = -float(np.percentile(cluster_points[:, 2], 10))
            too_deep = depth > params.max_depth_below_datum
            if too_deep:
                size_ok = False
                logger.info(
                    "  簇 %d: 10%% 分位低于地磅标定面 %.2fm（上限 %.2fm），"
                    "判定为磅坑/地沟类结构，不作车辆候选",
                    cid, depth, params.max_depth_below_datum,
                )
        at_edge = params.scene_edge_filter and touches_edge(cluster_points)
        verdict = "车辆" if size_ok and not at_edge else (
            "深度超限剔除" if too_deep else
            "边缘截断剔除" if size_ok else "待定/剔除"
        )
        logger.info(
            "  簇 %d: %d 点, L=%.2f W=%.2f H=%.2f 中心(%.1f, %.1f, %.1f) → %s",
            cid, n_pts, length, width, height, *cluster_points.mean(axis=0), verdict,
        )
        (candidates if size_ok else rejected_cids).append(
            (mask, n_pts, at_edge) if size_ok else cid
        )

    if not candidates:
        raise ValueError("未找到符合货车尺寸先验的点云簇（场景中无有效车辆）")

    max_pts = max(n for _, n, _ in candidates)
    vehicle_masks: List[np.ndarray] = []
    primary_at_edge = False
    for mask, n_pts, at_edge in candidates:
        if at_edge and n_pts < max_pts:
            continue
        if at_edge:
            primary_at_edge = True
            logger.warning("主车辆贴近场景边缘，点云可能被截断，测量结果需人工核对")
        vehicle_masks.append(mask)
    return vehicle_masks, rejected_cids, primary_at_edge


def _attach_adjacent_clusters(
    work: np.ndarray,
    labels: np.ndarray,
    vehicle_masks: List[np.ndarray],
    rejected_cids: List[int],
    params: PreprocessParams,
) -> None:
    """步骤 5：车头 / 附属结构找回（原地修改 ``vehicle_masks``）。

    俯视雷达下车头常只剩一块低矮顶面，与车厢之间还有空隙，会被 DBSCAN 分成
    独立小簇并被尺寸先验拒掉。落在车辆扩展包围盒内足够比例的被拒簇归还车辆。

    包围盒按合并前的车辆姿态计算一次，合并后不重算 —— 避免逐次合并让坐标系
    漂移，把越来越远的簇卷进来。

    仅靠距离无法把车头和场地立柱分开：纵向外扩必须有 3m 才能跨过车厢与车头之间
    的空隙（实测 3.22m），而磅秤旁的立柱离车尾只有 0.5m；后视镜又把包围盒横向
    撑到 ±2.4m，立柱正好落进容差里。2026-09-10 现场实测：两根 0.5×0.5×2.9m 的
    立柱（2091/943 点）被当成车头合并，只占全车点数 1%，却把车头前沿从 L≈7.0
    推到 L≈10.2，整车长和车头到车厢前内壁各虚高 3.1m。
    因此这里追加形状判据 —— 车头与车厢同宽量级，立柱不是。
    """
    for vi, base_mask in enumerate(vehicle_masks):
        vehicle_points = work[base_mask]
        local, rotation, center = horizontal_pca_frame(vehicle_points)
        l_lo, l_hi = local[:, 0].min(), local[:, 0].max()
        w_lo, w_hi = local[:, 1].min(), local[:, 1].max()
        z_lo, z_hi = vehicle_points[:, 2].min(), vehicle_points[:, 2].max()
        vehicle_width = cluster_dims(vehicle_points)[1]
        for cid in list(rejected_cids):
            cluster_mask = labels == cid
            cluster_points = work[cluster_mask]
            cluster_width = cluster_dims(cluster_points)[1]
            if cluster_width < vehicle_width * params.attach_min_width_ratio:
                logger.info(
                    "  簇 %d(%d 点) 宽 %.2fm 不足车辆宽 %.2fm 的 %.0f%%，"
                    "不是车头形状 → 不合并",
                    cid, int(cluster_mask.sum()), cluster_width, vehicle_width,
                    params.attach_min_width_ratio * 100,
                )
                continue
            projected = (cluster_points[:, :2] - center) @ rotation
            inside = (
                (projected[:, 0] > l_lo - params.attach_gap_length)
                & (projected[:, 0] < l_hi + params.attach_gap_length)
                & (projected[:, 1] > w_lo - params.attach_gap_width)
                & (projected[:, 1] < w_hi + params.attach_gap_width)
                & (cluster_points[:, 2] > z_lo - params.attach_gap_z)
                & (cluster_points[:, 2] < z_hi + params.attach_gap_z)
            )
            if inside.mean() >= params.attach_inside_frac:
                logger.info(
                    "  簇 %d(%d 点) 紧邻车辆簇，判定为车头/附属结构 → 合并",
                    cid, int(cluster_mask.sum()),
                )
                vehicle_masks[vi] = vehicle_masks[vi] | cluster_mask
                rejected_cids.remove(cid)


def _one_density_cluster(points: np.ndarray, radius: float, min_points: int) -> bool:
    """用少量实测核心点证明整块属于同一 DBSCAN 簇，证据不足时交回原算法。"""
    cells = np.floor((points - points.min(axis=0)) / (radius / 2))
    _, first, counts = np.unique(cells, axis=0, return_index=True, return_counts=True)
    anchors = points[first[counts >= min_points]]
    if not len(anchors):
        return False
    # 代表点须在原始点集中有足够邻点。距离略向内取，半径边界留给 Open3D。
    interior = radius * (1 - 32 * np.finfo(float).eps)
    distances, _ = cKDTree(points).query(anchors, k=[min_points])
    if np.any(distances >= interior):
        return False
    tree = cKDTree(anchors)
    links = tree.sparse_distance_matrix(tree, interior, output_type='coo_matrix')
    if connected_components(links, directed=False, return_labels=False) != 1:
        return False
    # 核心点相连且其邻域覆盖全部实测点，每个点必属于这个簇（含边界点）。
    nearest, _ = tree.query(points)
    return bool(np.all(nearest < interior))


def _body_cluster_labels(points: np.ndarray, radius: float, min_points: int) -> np.ndarray:
    """只按大于聚类半径的空隙分块；调用方仅比较同簇关系与噪声标记。"""
    def original(block):
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(block))
        return np.asarray(cloud.cluster_dbscan(eps=radius, min_points=min_points, print_progress=False))

    # 仅控制是否值得走快速路径，小点集直接调用开销更低。
    min_batch = 4096
    if len(points) < min_batch or not (np.isfinite(radius) and radius > 0 and min_points > 0):
        return original(points)
    # 任一坐标轴上的间隔都可证明两块没有半径邻接；选择最宽的间隔所在轴。
    gaps = np.diff(np.sort(points, axis=0), axis=0)
    axis = int(np.argmax(gaps.max(axis=0)))
    cuts = np.flatnonzero(gaps[:, axis] > radius * (1 + 32 * np.finfo(float).eps)) + 1
    if len(cuts) == 0:
        if _one_density_cluster(points, radius, min_points):
            return np.zeros(len(points), dtype=np.int32)
        return original(points)
    labels = np.full(len(points), -1, dtype=np.int32)
    offset = 0
    for indices in np.split(np.argsort(points[:, axis], kind='stable'), cuts):
        if len(indices) < min_points:
            continue
        # DBSCAN 的边界点可能在两簇间有歧义，保留原输入遍历顺序。
        indices.sort()
        block = points[indices]
        if len(block) >= min_batch and _one_density_cluster(block, radius, min_points):
            labels[indices] = offset
            offset += 1
        else:
            local = original(block)
            keep = local >= 0
            labels[indices[keep]] = local[keep] + offset
            offset += int(local.max()) + 1
    return labels


def _attach_body_run(
    work: np.ndarray,
    vehicle_masks: List[np.ndarray],
    params: PreprocessParams,
) -> int:
    """步骤 5b：沿车轴把与主车辆共线的车身段（车头）并入主车辆。

    步骤 5 只回收"被尺寸先验拒掉的整簇"。但俯视合并点云里车头往往和残留地面、
    路肩立柱聚成同一个**尺寸合格**的大簇：实测双雷达合并帧里该簇 10.7 万点、
    L=13.8m，自己就被判为"车辆"，从不进 ``rejected_cids``；主车辆按点数取最大，
    于是只剩车厢，``cabToFrontWall`` 从实测 4.6m 塌成 0.15m。

    这里改为按**点**并入，三重判据把车头与干扰分开：

    * 横向走廊 —— 偏离主车中心线超过 半宽 + ``attach_gap_width`` 的点不要；
    * 高度下限 —— 低于主车最低点 ``attach_gap_z`` 的点不要（残留地面在此被滤掉，
      实测该簇 10.7 万点里有 6.5 万点在磅台面以下）；
    * 分箱宽度 —— 同一三维连通实体在纵向分箱内的横向跨度须达到主车宽度的 ``body_run_width_frac``。
      车头有整车宽，路肩立柱只有 0.3m，这一条是区分两者的主判据。

    再从主车两端向外连通生长，空档超过 ``body_run_gap`` 即停止，保证不会越过车头
    去捞更远的立柱。外扩上限由车长先验 ``veh_length_range[1]`` 决定，不另设阈值。

    Args:
        work: (N,3) 参与聚类的场景点。
        vehicle_masks: 车辆簇掩码列表，**原地修改**：并入的点转入主车辆掩码，
            并从其它掩码里移除，避免 ``np.vstack`` 时重复计数。
        params: 阈值集合。

    Returns:
        并入主车辆的点数。
    """
    if not params.body_run_enable or not vehicle_masks:
        return 0
    primary = int(np.argmax([int(m.sum()) for m in vehicle_masks]))
    base_mask = vehicle_masks[primary]
    base = work[base_mask]
    if len(base) < params.veh_min_points:
        return 0

    local, rotation, center = horizontal_pca_frame(base)
    l_lo, l_hi = float(local[:, 0].min()), float(local[:, 0].max())
    w_lo, w_hi = np.percentile(local[:, 1], [0.5, 99.5])
    w_center, w_half = (w_lo + w_hi) / 2.0, (w_hi - w_lo) / 2.0
    width = 2.0 * w_half
    if width <= 0:
        return 0
    # 外扩上限：并入后整车长不得超过车长先验上界
    reach = max(params.veh_length_range[1] - (l_hi - l_lo), 0.0)
    if reach <= 0:
        return 0

    projected = (work[:, :2] - center) @ rotation
    corridor = (
        ~base_mask
        & (np.abs(projected[:, 1] - w_center) <= w_half + params.attach_gap_width)
        & (work[:, 2] >= base[:, 2].min() - params.attach_gap_z)
        & (projected[:, 0] > l_lo - reach)
        & (projected[:, 0] < l_hi + reach)
    )
    if not corridor.any():
        return 0

    origin = l_lo - reach
    n_bins = int(np.ceil((l_hi + reach - origin) / params.body_run_bin)) + 1

    def bin_of(values: np.ndarray) -> np.ndarray:
        """把沿车轴坐标映射到分箱下标（越界钳到两端）。"""
        return np.clip(
            ((values - origin) / params.body_run_bin).astype(int), 0, n_bins - 1
        )

    occupied = np.zeros(n_bins, dtype=bool)
    occupied[bin_of(local[:, 0])] = True

    cand_idx = np.nonzero(corridor)[0]
    # 同一纵向格的散点不能拼成车宽；先按既有三维聚类规则分出实体。
    candidate_labels = _body_cluster_labels(work[cand_idx], params.dbscan_eps, params.dbscan_min_points)
    cand_bin = bin_of(projected[cand_idx, 0])
    order = np.argsort(cand_bin, kind="stable")
    cand_bin_sorted = cand_bin[order]
    cand_w_sorted = projected[cand_idx[order], 1]
    edges = np.searchsorted(cand_bin_sorted, np.arange(n_bins + 1))
    body_like = np.zeros(n_bins, dtype=bool)
    body_points = np.zeros(len(cand_idx), dtype=bool)
    for b in range(n_bins):
        lo, hi = edges[b], edges[b + 1]
        if hi - lo < params.body_run_min_points:
            continue
        segment_labels = candidate_labels[order[lo:hi]]
        for label in np.unique(segment_labels[segment_labels >= 0]):
            segment = cand_w_sorted[lo:hi][segment_labels == label]
            if (len(segment) >= params.body_run_min_points
                    and float(np.ptp(segment)) >= params.body_run_width_frac * width):
                body_like[b] = True
                body_points[order[lo:hi][segment_labels == label]] = True

    max_gap_bins = max(int(round(params.body_run_gap / params.body_run_bin)), 1)
    accepted = np.zeros(n_bins, dtype=bool)
    occupied_bins = np.nonzero(occupied)[0]
    for start, step in ((int(occupied_bins.max()), 1), (int(occupied_bins.min()), -1)):
        gap, b = 0, start + step
        while 0 <= b < n_bins and gap <= max_gap_bins:
            if body_like[b]:
                accepted[b] = True
                gap = 0
            elif not occupied[b]:
                gap += 1
            b += step
    if not accepted.any():
        return 0

    attach_idx = cand_idx[accepted[cand_bin] & body_points]
    attach_mask = np.zeros(len(work), dtype=bool)
    attach_mask[attach_idx] = True
    grown = projected[attach_mask, 0]
    logger.info(
        "  车身段并入主车辆：%d 点，沿车轴 [%.2f, %.2f]m（主车原 [%.2f, %.2f]m）",
        int(attach_mask.sum()), float(grown.min()), float(grown.max()), l_lo, l_hi,
    )
    vehicle_masks[primary] = base_mask | attach_mask
    for i, mask in enumerate(vehicle_masks):
        if i != primary:
            vehicle_masks[i] = mask & ~attach_mask
    return int(attach_mask.sum())


def _trim_terminal_low_surface(
    vehicle_points: np.ndarray, params: PreprocessParams,
) -> Tuple[np.ndarray, int]:
    """剔除与车连簇、但端部没有车体支撑的低位连续路面。

    只在已标定的场地高度系启用。先找相邻两个具有车身宽度及高度的纵向分箱，
    再检查其外侧是否为连续、薄且横向铺满的近地表面。车轮、保险杠等局部低点
    不具备横向铺满的薄面；单个立柱也不能把道路误作车体起点。
    """
    if not params.z_is_datum_height or len(vehicle_points) < params.veh_min_points:
        return vehicle_points, 0
    local, _, _ = horizontal_pca_frame(vehicle_points)
    low_l = float(local[:, 0].min())
    bin_count = int(np.ceil((float(local[:, 0].max()) - low_l)
                            / TERMINAL_SURFACE_BIN_M)) + 1
    if bin_count < 2 * int(TERMINAL_SURFACE_MIN_LENGTH_M / TERMINAL_SURFACE_BIN_M) + 2:
        return vehicle_points, 0
    index = np.clip(((local[:, 0] - low_l) / TERMINAL_SURFACE_BIN_M).astype(int),
                    0, bin_count - 1)
    counts = np.bincount(index, minlength=bin_count)
    ground_like = np.zeros(bin_count, dtype=bool)
    body_like = np.zeros(bin_count, dtype=bool)
    elevated_min = max(params.ground_multilevel_max, 0.8)
    for cell in np.flatnonzero(counts >= 30):
        section = local[index == cell]
        elevated = section[section[:, 2] > elevated_min]
        if len(elevated) >= 30 and len(elevated) / len(section) >= 0.30:
            elevated_width = np.ptp(np.percentile(elevated[:, 1], [5, 95]))
            body_like[cell] = elevated_width >= 0.8

        low = section[(section[:, 2] >= TERMINAL_SURFACE_MIN_HEIGHT_M)
                      & (section[:, 2] <= TERMINAL_SURFACE_MAX_HEIGHT_M)]
        if len(low) < 100 or len(low) / len(section) < 0.70:
            continue
        height_span = np.ptp(np.percentile(low[:, 2], [10, 90]))
        width_lo, width_hi = np.percentile(low[:, 1], [5, 95])
        width = float(width_hi - width_lo)
        if (height_span > TERMINAL_SURFACE_MAX_THICKNESS_M
                or width < TERMINAL_SURFACE_MIN_WIDTH_M):
            continue
        lateral_bins = max(1, int(np.ceil(width / 0.25)))
        lateral_index = np.clip(((low[:, 1] - width_lo) / 0.25).astype(int),
                                0, lateral_bins - 1)
        occupied = np.bincount(lateral_index, minlength=lateral_bins) >= 5
        ground_like[cell] = float(np.mean(occupied)) >= 0.70

    body_pairs = np.flatnonzero(body_like[:-1] & body_like[1:])
    if not len(body_pairs):
        return vehicle_points, 0

    def terminal_cut(cells: np.ndarray) -> int | None:
        """从一个端部找首个宽高连续的车体段，并验证外侧是路面。"""
        pair = next((i for i in range(len(cells) - 1)
                     if body_like[cells[i]] and body_like[cells[i + 1]]), None)
        if pair is None:
            return None
        extension = cells[:pair]
        supported = extension[counts[extension] >= 100]
        if (len(extension) * TERMINAL_SURFACE_BIN_M < TERMINAL_SURFACE_MIN_LENGTH_M
                or len(supported) < 3
                or int(np.sum(counts[supported])) < 2000
                or float(np.mean(ground_like[supported])) < 0.75):
            return None
        return int(cells[pair])

    left_cut = terminal_cut(np.arange(bin_count))
    right_cut = terminal_cut(np.arange(bin_count - 1, -1, -1))
    keep = np.ones(len(vehicle_points), dtype=bool)
    if left_cut is not None:
        keep &= index >= left_cut
    if right_cut is not None:
        keep &= index <= right_cut
    removed = int(np.count_nonzero(~keep))
    if removed:
        logger.info("已剔除车体端部粘连的低位连续路面 %d 点", removed)
    return vehicle_points[keep], removed


def _rescue_sparse_points(
    vehicle_points_list: List[np.ndarray],
    rescue_pool: np.ndarray,
    n_from_outliers: int,
    params: PreprocessParams,
) -> Tuple[int, int]:
    """步骤 6：车辆范围内稀疏点找回（原地修改 ``vehicle_points_list``）。

    种子取紧贴现存车辆点的候选，再以新接受点继续连通生长。连接同时受既有半径
    和两端各自的近邻采样距离约束，避免密集车身向稀疏环境单向扩散。

    Args:
        n_from_outliers: ``rescue_pool`` 前多少个点来自 SOR 误删（用于统计核算）。
    Returns:
        (找回总点数, 其中来自 SOR 误删的点数)
    """
    if not len(rescue_pool):
        return 0, 0

    radius = params.rescue_nn_radius
    claimed = np.zeros(len(rescue_pool), dtype=bool)
    n_rescued = 0
    n_rescued_from_outliers = 0
    for vi, vehicle_points in enumerate(vehicle_points_list):
        local, rotation, center = horizontal_pca_frame(vehicle_points)
        projected = (rescue_pool[:, :2] - center) @ rotation
        inside = (
            (projected[:, 0] > local[:, 0].min() - params.rescue_gap_length)
            & (projected[:, 0] < local[:, 0].max() + params.rescue_gap_length)
            & (projected[:, 1] > local[:, 1].min() - 0.3)
            & (projected[:, 1] < local[:, 1].max() + 0.3)
            & (rescue_pool[:, 2] > vehicle_points[:, 2].min() - 0.3)
            & (rescue_pool[:, 2] < vehicle_points[:, 2].max() + 0.3)
            & ~claimed
        )
        if not inside.any():
            continue

        # 连接必须得到两端各自的采样邻域支持。只用固定半径会让离散环境点
        # 从密集车身单向挂接，并沿新接入点不断向车外扩散。
        pool_indices = np.flatnonzero(inside)
        graph_points = np.vstack([vehicle_points, rescue_pool[pool_indices]])
        tree = cKDTree(vehicle_points)
        candidates = rescue_pool[pool_indices]
        distance, nearest = tree.query(candidates, k=1, workers=-1)
        # 邻居仍在完整点集中搜索。车辆端只读取实际用到的端点半径，重复端点只查一次。
        endpoints, inverse = np.unique(nearest, return_inverse=True)
        queries = np.vstack([vehicle_points[endpoints], candidates])
        rank = min(params.sor_neighbors + 1, len(graph_points))
        radii = cKDTree(graph_points).query(queries, k=[rank], workers=-1)[0][:, 0]
        candidate_radii = radii[len(endpoints):]
        accepted = np.zeros(len(rescue_pool), dtype=bool)
        accepted[pool_indices[(distance <= radius)
                              & (distance <= radii[inverse])
                              & (distance <= candidate_radii)]] = True
        pool_radius = np.zeros(len(rescue_pool))
        pool_radius[pool_indices] = candidate_radii
        frontier = accepted.copy()
        for _ in range(params.rescue_max_iterations):
            pending = inside & ~accepted
            if not pending.any() or not frontier.any():
                break
            grown, nearest = cKDTree(rescue_pool[frontier]).query(rescue_pool[pending], k=1, workers=-1)
            new_index = np.where(pending)[0][
                (grown <= radius)
                & (grown <= pool_radius[np.flatnonzero(frontier)[nearest]])
                & (grown <= pool_radius[pending])]
            if len(new_index) == 0:
                break
            frontier = np.zeros(len(rescue_pool), dtype=bool)
            frontier[new_index] = True
            accepted |= frontier

        if accepted.any():
            vehicle_points_list[vi] = np.vstack([vehicle_points, rescue_pool[accepted]])
            claimed |= accepted
            n_rescued += int(accepted.sum())
            n_rescued_from_outliers += int(accepted[:n_from_outliers].sum())

    if n_rescued:
        logger.info("车辆范围内稀疏点找回：%d 点（SOR 误删/聚类噪声/碎片簇）", n_rescued)
    return n_rescued, n_rescued_from_outliers


def _underside_clearance(
    local: np.ndarray,
    vehicle_points: np.ndarray,
    candidate_ground: np.ndarray,
    cell: float = 0.30,
    min_cell_points: int = 3,
) -> Optional[float]:
    """车辆自身底面相对候选地面的净空中位数（m）。

    把足迹按 ``cell`` 分格，取每格内车辆点的最低高度，再对各格取中位数，减去候选
    地面的高度中位数。用中位数而非最小值，是为了不被轮胎、悬挂这类少数触地结构
    带偏 —— 悬空底盘只有轮子落地，绝大多数格子的最低点仍在底盘高度上。

    Returns:
        净空；格子太少无法判断时返回 ``None``（此时按原行为放行）。
    """
    if len(candidate_ground) == 0 or len(vehicle_points) == 0:
        return None
    key = np.floor(local[:, :2] / cell).astype(np.int64)
    order = np.lexsort((key[:, 1], key[:, 0]))
    sorted_key = key[order]
    sorted_z = vehicle_points[order][:, 2]
    starts = np.flatnonzero(
        np.r_[True, (np.diff(sorted_key, axis=0) != 0).any(axis=1)]
    )
    cell_min = np.minimum.reduceat(sorted_z, starts)
    cell_count = np.diff(np.r_[starts, len(sorted_z)])
    cell_min = cell_min[cell_count >= min_cell_points]
    if len(cell_min) < 10:
        return None
    return float(np.median(cell_min)) - float(np.median(candidate_ground[:, 2]))


def _restore_carriage_floor(
    vehicle_points_list: List[np.ndarray],
    ground_removed: np.ndarray,
    params: PreprocessParams,
) -> int:
    """步骤 7：车厢底板找回（原地修改 ``vehicle_points_list``）。

    低底板车辆的车厢底板与周围地面共面，会被同一个 RANSAC 平面一次性剔除，
    导致车厢内部无点、空车误判成载货（实测该场景填充率虚高到 0.80）。

    物理依据：俯视雷达照不到车底下的真实地面（被车厢遮挡），因此落在车辆
    足迹内部、且不低于车辆结构底部的"被删地面点"只可能是车厢底板。足迹向内
    收缩后再判定，排除车旁与车两端的真实地面。

    该前提**只对低底板封闭车厢成立**。高平板底盘（自卸车、平板车）的车底是敞开
    的，雷达从轮间空隙就能看到车下真实地面；把它当底板补回来会让底板面落到地面
    高度上。实测一辆空平板车因此得到 floorZ=0、栏板深 2.64m、填充率 0.626，被判
    成 Loaded 而拿不到任何尺寸。因此先测车辆自身底面相对候选地面的净空：低底板
    车厢的底板就是车辆最低结构，净空≈0；悬空底盘实测净空 +1.11m。
    """
    if not params.floor_restore_enable or not len(ground_removed):
        return 0

    claimed = np.zeros(len(ground_removed), dtype=bool)
    n_restored = 0
    for vi, vehicle_points in enumerate(vehicle_points_list):
        local, rotation, center = horizontal_pca_frame(vehicle_points)
        projected = (ground_removed[:, :2] - center) @ rotation
        # 足迹与下沿都用分位数而非极值。用极值会形成正反馈：先前步骤带进来的
        # 少量车外污染点（路缘、护栏根部）把足迹撑大、把下沿压低，于是这里又
        # 放进更多车外地面点。实测新车型该环节一次补进 1112 点，其中 1005 点
        # 低于地磅标定面，明显不是离地 2.2m 的车厢底板。
        low_l, high_l = np.percentile(local[:, 0], [1, 99])
        low_w, high_w = np.percentile(local[:, 1], [1, 99])
        floor_low = float(np.percentile(vehicle_points[:, 2], 1))
        inside = (
            (projected[:, 0] > low_l + 0.3)
            & (projected[:, 0] < high_l - 0.3)
            & (projected[:, 1] > low_w + 0.15)
            & (projected[:, 1] < high_w - 0.15)
            & (ground_removed[:, 2] > floor_low - 0.3)
            & ~claimed
        )
        if not inside.any():
            continue
        clearance = _underside_clearance(
            local, vehicle_points, ground_removed[inside]
        )
        if clearance is not None and clearance > params.floor_restore_max_clearance:
            logger.info(
                "跳过车厢底板找回：车辆底面高出候选地面 %.2fm（上限 %.2fm），"
                "判定为悬空底盘，足迹内的 %d 个点是车下真实地面而非底板",
                clearance,
                params.floor_restore_max_clearance,
                int(inside.sum()),
            )
            continue
        vehicle_points_list[vi] = np.vstack([vehicle_points, ground_removed[inside]])
        claimed |= inside
        n_restored += int(inside.sum())

    if n_restored:
        logger.info("车厢底板找回：%d 点（车辆足迹内被误删的地面共面点）", n_restored)
    return n_restored


def _build_rescue_pool(
    work: np.ndarray,
    labels: np.ndarray,
    n_clusters: int,
    outlier_points: np.ndarray,
    params: PreprocessParams,
) -> Tuple[np.ndarray, int]:
    """汇总"疑似误删"的点：SOR 误删 + DBSCAN 噪声 + 未达候选门槛的碎片小簇。

    尺寸不合格的大簇（护栏、立柱等已明确判为干扰）不入池。

    Returns:
        (候选点池, 其中来自 SOR 误删的点数 —— 池内排在最前，便于统计核算)
    """
    fragment_mask = labels < 0
    clustered = ~fragment_mask
    counts = np.bincount(labels[clustered], minlength=n_clusters)
    fragment_mask[clustered] = counts[labels[clustered]] < params.attach_min_points

    parts = [p for p in (outlier_points, work[fragment_mask]) if len(p)]
    if not parts:
        return np.empty((0, 3)), 0
    return np.vstack(parts), len(outlier_points)


def filter_scene(points: np.ndarray, params: PreprocessParams) -> PreprocessResult:
    """从完整场景点云中剔除环境干扰，提取目标车辆点云。

    编排模块头部注释里的七步链，各步实现见对应的 ``_*`` 函数。

    Args:
        points: (N,3) 场景点云（米制）。
        params: 阈值集合。
    Returns:
        PreprocessResult
    Raises:
        ValueError: 地面剔除后无剩余点，或找不到符合货车尺寸先验的簇。
    """
    n_input = len(points)

    # 步骤 0：地面基准。与剔除解耦，见 estimate_ground_plane 的注释。
    ground_plane = estimate_ground_plane(points, params)
    if ground_plane is None:
        logger.warning("场景中未找到地平面，底板高度将退化为相对基准")

    ground = _remove_ground(points, params, ground_datum=ground_plane)
    if len(ground.remaining) == 0:
        raise ValueError("地面剔除后无剩余点云，场景中可能不存在车辆")

    filtered, work, outlier_points = _remove_outliers(ground.remaining, params)
    labels, n_clusters = _cluster_points(filtered, work, params)

    vehicle_masks, rejected_cids, edge_truncated = _select_vehicle_clusters(
        work, labels, n_clusters, points, params
    )
    _attach_adjacent_clusters(work, labels, vehicle_masks, rejected_cids, params)
    _attach_body_run(work, vehicle_masks, params)
    vehicle_masks = [m for m in vehicle_masks if m.any()]

    vehicle_points_list: List[np.ndarray] = [work[mask] for mask in vehicle_masks]
    rescue_pool, n_pool_from_outliers = _build_rescue_pool(
        work, labels, n_clusters, outlier_points, params
    )
    n_rescued, n_rescued_from_outliers = _rescue_sparse_points(
        vehicle_points_list, rescue_pool, n_pool_from_outliers, params
    )
    n_restored_floor = _restore_carriage_floor(
        vehicle_points_list, ground.removed, params
    )
    terminal_surface_removed = 0
    for index, candidate in enumerate(vehicle_points_list):
        vehicle_points_list[index], removed = _trim_terminal_low_surface(candidate, params)
        terminal_surface_removed += removed

    vehicle_points = np.vstack(vehicle_points_list)
    primary_index = int(np.argmax([len(v) for v in vehicle_points_list]))
    logger.info(
        "前处理完成：%d → %d 点（主车辆 %d 点，车辆簇 %d 个）",
        n_input, len(vehicle_points),
        len(vehicle_points_list[primary_index]), len(vehicle_points_list),
    )
    return PreprocessResult(
        vehicle_points=vehicle_points,
        primary_vehicle_points=vehicle_points_list[primary_index],
        vehicle_points_list=vehicle_points_list,
        primary_index=primary_index,
        ground_plane=ground_plane,
        applied=True,
        edge_truncated=edge_truncated,
        stats={
            "n_input": n_input,
            "n_vehicle": int(len(vehicle_points)),
            "n_removed": int(n_input - len(vehicle_points)),
            # 找回的点要从对应的剔除计数里扣回，否则各项之和与 n_removed 不符
            "ground": int(ground.n_removed - n_restored_floor),
            "outlier": int(len(outlier_points) - n_rescued_from_outliers),
            "rescued_sparse": int(n_rescued),
            "restored_floor": int(n_restored_floor),
            "terminal_low_surface": terminal_surface_removed,
            "n_vehicle_clusters": len(vehicle_points_list),
        },
    )


class ScenePreprocessor:
    """``Config`` 驱动的场景前处理器，供 ``PointCloudAnalyzer`` 调用。

    小型、已裁剪的单车点云会直接放行。对于完整场景提取失败，是否继续使用
    原点云由 ``scene_preprocessing_fail_open`` 控制：离线兼容模式可以降级，
    生产双雷达链路必须失败关闭，避免把护栏和地面量成车辆尺寸。
    """

    def __init__(self, config):
        """保存用于生成前处理阈值的全局配置。"""
        self.config = config

    def params(self) -> PreprocessParams:
        """把全局配置投影为场景前处理专用参数。"""
        cfg = self.config
        return PreprocessParams(
            enable=cfg.scene_preprocessing_enable,
            min_points=cfg.scene_preprocess_min_points,
            ground_dist_threshold=cfg.scene_ground_dist_threshold,
            ground_normal_min_dot=cfg.scene_ground_normal_min_dot,
            ground_min_inlier_ratio=cfg.scene_ground_min_inlier_ratio,
            ground_max_iterations=cfg.scene_ground_max_iterations,
            ground_z_tolerance=cfg.scene_ground_z_tolerance,
            ground_min_extent=cfg.scene_ground_min_extent,
            ground_multilevel_max=cfg.scene_ground_multilevel_max,
            ground_datum_voxel=cfg.scene_ground_datum_voxel,
            ground_datum_max_planes=cfg.scene_ground_datum_max_planes,
            sor_neighbors=cfg.statistical_outlier_neighbors,
            sor_std_ratio=cfg.statistical_outlier_std_ratio,
            dbscan_voxel=cfg.scene_dbscan_voxel,
            dbscan_eps=cfg.scene_dbscan_eps,
            dbscan_min_points=cfg.scene_dbscan_min_points,
            veh_length_range=cfg.scene_vehicle_length_range,
            veh_width_range=cfg.scene_vehicle_width_range,
            veh_height_range=cfg.scene_vehicle_height_range,
            veh_min_points=cfg.scene_vehicle_min_points,
            attach_min_points=cfg.scene_attach_min_points,
            attach_min_width_ratio=cfg.scene_attach_min_width_ratio,
            attach_gap_length=cfg.scene_attach_gap_length,
            attach_gap_width=cfg.scene_attach_gap_width,
            attach_gap_z=cfg.scene_attach_gap_z,
            attach_inside_frac=cfg.scene_attach_inside_frac,
            body_run_enable=cfg.scene_body_run_enable,
            body_run_bin=cfg.scene_body_run_bin,
            body_run_gap=cfg.scene_body_run_gap,
            body_run_width_frac=cfg.scene_body_run_width_frac,
            body_run_min_points=cfg.scene_body_run_min_points,
            scene_edge_filter=cfg.scene_edge_filter,
            scene_edge_margin=cfg.scene_edge_margin,
            rescue_nn_radius=cfg.scene_rescue_nn_radius,
            rescue_gap_length=cfg.scene_rescue_gap_length,
            rescue_max_iterations=cfg.scene_rescue_max_iterations,
            floor_restore_enable=cfg.scene_floor_restore_enable,
            floor_restore_max_clearance=cfg.scene_floor_restore_max_clearance,
        )

    def process(self, points: np.ndarray) -> PreprocessResult:
        """提取主车辆点云，按配置决定失败关闭或兼容降级。

        Args:
            points: 已统一为米制的场景点云。

        Returns:
            主车辆、多车辆候选、地面基准和剔除统计。

        Raises:
            ValueError: 生产失败关闭模式下无法可靠提取车辆。
        """
        params = self.params()
        skip = None
        if not params.enable:
            skip = "配置关闭场景前处理"
        elif len(points) < params.min_points:
            skip = f"点数 {len(points)} < {params.min_points}，视为已裁剪的单车点云"
        if skip:
            logger.info("跳过场景前处理：%s", skip)
            # 跳过剔除不等于放弃地面基准：输入若仍含地面，底板剖面照样能用真实地面
            return PreprocessResult(
                points, points, [points], 0,
                ground_plane=estimate_ground_plane(points, params),
                applied=False, reason=skip,
            )

        try:
            return filter_scene(points, params)
        except Exception as exc:
            if not self.config.scene_preprocessing_fail_open:
                raise ValueError(f"场景前处理未能可靠提取车辆：{exc}") from exc
            # 兼容历史单文件：保留原行为，但通过 applied/reason 明确标记降级。
            logger.warning("场景前处理未能提取车辆（%s），按原始点云继续，结果需人工核对", exc)
            return PreprocessResult(
                points, points, [points], 0,
                ground_plane=estimate_ground_plane(points, params),
                applied=False, reason=str(exc),
            )
