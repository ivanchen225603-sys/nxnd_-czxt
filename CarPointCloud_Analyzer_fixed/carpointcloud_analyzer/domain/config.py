import os
from dataclasses import dataclass, field
from typing import Optional, Tuple


def _env_flag(name: str, default: bool) -> bool:
    """从环境变量读取布尔开关。

    Args:
        name: 环境变量名。
        default: 未设置时的取值。

    Returns:
        环境变量指定的布尔值；未设置或无法识别时返回 ``default``。

    Notes:
        现场排障时改一行 Python 源码风险高于设一个环境变量，
        所以这类"要不要启用某条兜底"的开关留出免改码的切换入口。
    """
    raw = os.getenv(name)
    if raw is None:
        return default
    value = raw.strip().lower()
    if value in ("1", "true", "yes", "on"):
        return True
    if value in ("0", "false", "no", "off"):
        return False
    return default


@dataclass
class Config:
    """点云分析全局配置。"""

    # Scene preprocessing (环境干扰剔除 → 单台目标车辆点云)
    # 未启用时，局部坐标路径会直接在完整场景（地面/护栏/杂物）上建立 PCA
    # 坐标系，导致坐标系、车厢端面、拉筋位置全部错误。详见 preprocessing.py。
    scene_preprocessing_enable: bool = True
    # 历史单文件工具允许环境提取失败后原样继续；双雷达生产链路应设为 False，
    # 防止把整片场景当作车辆并仍然输出“成功”尺寸。
    # 失败时是否退回原始场景点云。默认关闭：降级意味着把地面、护栏、行人
    # 当作车辆点写进正式结果，而调用方拿不到任何信号。需要降级的离线排查
    # 显式打开。
    scene_preprocessing_fail_open: bool = False
    # 环境算法内部仍以 Z-up 为前提，而现场 PCD 的 XYZ 方向可能变化。启用后先
    # 根据地磅标定面（优先）或场景大面积支撑面建立临时 Z-up 坐标，再做环境剔除。
    # 默认开启：关闭时所有入口都隐含假定输入 Z 朝上。实测把同一场景绕 X 轴转
    # 90° 后，关闭调平会让地面法向判据失效、提车整体失败并降级返回整幅场景
    # （230000 点 / 50m 跨度），地面基准同时丢失。
    scene_auto_level_enable: bool = True
    # 双雷达生产接口必须能确定竖直方向，识别失败即拒绝继续测量。单文件入口保持
    # False：允许"已裁剪到车身、看不到支撑面"的历史 PCD 走到下一步，由
    # geometry_require_vertical_reference 决定是否放行。
    scene_auto_level_required: bool = False
    # CAM_004/源坐标系中的地磅台面 ax+by+cz+d=0，法向必须朝向车辆。
    # 该值来自固定场地标定，不能用每车一帧临时拟合代替。
    weighbridge_plane_source: Optional[Tuple[float, float, float, float]] = None
    # 双雷达服务已在 A+B 共同场景中验证过标定面方向后设为 True。降级到单台 A
    # 时，A 的大面积背景可能位于地磅下方，不能再用整幅点云的两侧跨度否定该标定。
    weighbridge_plane_orientation_verified: bool = False
    # 底板高度的业务基准必须是地磅台面。生产接口设为 True 后，缺少上述标定面
    # 仍可做空载判断和可见尺寸，但不会输出容易误解的“到底磅高度”。
    floor_require_weighbridge_plane: bool = False
    scene_preprocess_min_points: int = 2000   # 低于该点数视为已裁剪的单车点云
    scene_ground_dist_threshold: float = 0.12
    scene_ground_normal_min_dot: float = 0.80
    scene_ground_min_inlier_ratio: float = 0.08
    scene_ground_max_iterations: int = 4
    scene_ground_z_tolerance: float = 0.6
    scene_ground_min_extent: float = 6.0      # 地面平面最小水平跨度（保护车顶/苫布）
    # 多级地面（磅台面、引坡）相对地面基准的最大抬高。磅台面比周边地坪高 0.65~0.70m
    # 且只有 4m 宽，会被 scene_ground_min_extent 的窄跨度判据当"车辆表面"保护下来；
    # 车厢底板离地 2.2~2.3m，两者之间有 1.5m 余量。
    scene_ground_multilevel_max: float = 1.0
    scene_ground_datum_voxel: float = 0.15    # 地面基准拟合前的降采样尺寸
    scene_ground_datum_max_planes: int = 5    # 地面基准最多考察的候选平面数
    scene_dbscan_voxel: float = 0.08
    scene_dbscan_eps: float = 0.45
    scene_dbscan_min_points: int = 10
    scene_vehicle_length_range: Tuple[float, float] = (3.5, 30.0)
    scene_vehicle_width_range: Tuple[float, float] = (1.2, 4.5)
    scene_vehicle_height_range: Tuple[float, float] = (0.5, 5.5)
    scene_vehicle_min_points: int = 1500
    scene_attach_min_points: int = 200
    # 见 preprocessing._attach_adjacent_clusters：车头找回的形状下限。
    scene_attach_min_width_ratio: float = 0.5
    scene_attach_gap_length: float = 3.0      # 车头找回沿车长外扩
    scene_attach_gap_width: float = 0.5
    scene_attach_gap_z: float = 0.8
    scene_attach_inside_frac: float = 0.6
    # 车身段并入（步骤 5b）：车头常与残留地面、路肩立柱聚成同一个尺寸合格的大簇，
    # 整簇既进不了 rejected_cids，也不该整簇并入。按纵向分箱逐段判定并生长。
    scene_body_run_enable: bool = True
    scene_body_run_bin: float = 0.20          # 沿车轴分箱宽度
    scene_body_run_gap: float = 2.0           # 允许桥接的空档：车头与车厢间实测 1.6m
    scene_body_run_width_frac: float = 0.40   # 分箱横向跨度下限（占主车宽）：
                                              # 车头有整车宽，路肩立柱只有 0.3m
    scene_body_run_min_points: int = 20       # 分箱最小点数
    scene_edge_filter: bool = True
    scene_edge_margin: float = 1.5            # 贴场景边界者视为视场截断异物
    scene_rescue_nn_radius: float = 0.6       # 稀疏点找回的邻接半径
    scene_rescue_gap_length: float = 6.0
    scene_rescue_max_iterations: int = 30
    scene_floor_restore_enable: bool = True   # 与地面共面的车厢底板找回
    # 底板找回的物理前提是"俯视雷达照不到车底下的地面"。该前提只对低底板封闭
    # 车厢成立；高平板底盘从轮间空隙就能看到车底真实地面，此时把它当底板补回来
    # 会让底板面落到地面高度上，车厢高与填充率全部失真（实测空平板车因此被判
    # 成 Loaded）。车辆自身底面高出候选地面超过此值即视为底盘悬空，跳过找回。
    scene_floor_restore_max_clearance: float = 0.5

    # 去噪与车辆局部坐标系（PointCloudGeometryNormalizer）
    # 高度轴不能只取"点最多的那个平面"的法向：侧壁比底板密时（实测底板 12000
    # 点 / 单侧壁 40000 点），最大平面就是侧壁，H 轴偏离真实竖直 89.99°，车宽和
    # 车厢高整体互换，而姿态置信度仍报 1.00。必须用独立的竖直基准（地磅标定面
    # 优先，其次场地支撑面）约束并定向 H 轴。
    geometry_require_vertical_reference: bool = True
    geometry_vertical_max_deviation_deg: float = 25.0
    # 有竖直基准时，最大平面若是立面（挡板、侧板）就继续找下一块，而不是直接
    # 拒绝：平板车/自卸车底盘的最大平面实测是一块偏离竖直 89.5° 的挡板，而真正
    # 的底板要到第二、三块才出现。全部候选都不合格时才拒绝测量。
    geometry_plane_search_max_attempts: int = 4
    statistical_outlier_enabled: bool = True
    statistical_outlier_neighbors: int = 20
    statistical_outlier_std_ratio: float = 2.0
    pose_min_points: int = 200      # 低于此点数拒绝建系，不再降级继续算
    pose_endpoint_ratio: float = 0.12
    carriage_head_exclude_ratio: float = 0.12
    carriage_tail_exclude_ratio: float = 0.08

    # 拉筋检测（分箱与取点共用于 LocalRibDetector）
    rib_bin_width: float = 0.05
    rib_smooth_sigma: float = 2.0
    rib_min_distance: float = 0.3
    rib_extract_window: float = 0.10
    rib_min_points: int = 10
    rib_wall_band: float = 0.15
    rib_x_coverage_min: float = 0.5
    rib_interior_min_points: int = 100

    # 局部坐标拉筋检测（LocalRibDetector）
    # 这些值此前硬编码在 geometry.py 中，导致局部路径无法通过 Config 调参。
    rib_local_min_prominence: float = 20.0        # 峰突出度绝对下限
    rib_local_prominence_ratio: float = 0.02      # 峰突出度相对 std 的系数
    rib_local_prominence_floor_ratio: float = 0.04  # 相对最强峰的突出度门槛
    rib_local_cluster_merge_distance: float = 1.0   # 同一结构的肩峰合并距离（m）
    # 拉筋是架在底板上方的横向杆件：找峰前先剔除底板面及其以下的点。
    # 底板点数是拉筋的百倍量级，不剔除会把拉筋峰完全淹没。
    rib_floor_clearance: float = 0.10   # 底板面以上多少米起算（m）

    # 车厢内部剖面（carriage.py）
    carriage_interior_min_points: int = 200   # 内部点少于此值时剖面降级
    carriage_grid_cell: float = 0.20          # 内部俯视网格尺寸（m）
    carriage_quality_bin: float = 0.20        # 沿车长的质量统计 bin 宽（m）
    carriage_wall_diff_tolerance: float = 0.15  # 两侧壁高差超过此值判定单侧扫描不全
    carriage_repair_low_wall: bool = True     # 空车时镜像修补扫描不全的低侧壁
    # 侧壁锚定：不再假设"车厢点集的横向分位数就是两侧栏板"。车辆簇一旦混入
    # 台面构筑物，分位数会落到车外地面，壁顶被测成地面高度（2026-09-09 现场
    # 实测 wall_depth=0.04m，而实际内深 2.72m）。改为按物理特征锚定：抬离地面
    # 后横向最稠密的两列即两道侧壁，间距须落在 carriage_anchor_width_range；
    # 再由壁上点"既稠密又高"的连续段界定车厢纵向范围，把车头与台面构筑物排除。
    # 默认关闭：兜底路径只在一份"车位偏了 4m、簇被污染"的样本上验证过，
    # 尚未在正常过磅工况下跑过。先按既有行为上线，取到正常样本再决定是否打开。
    # 关闭时壁深依旧会被 load_status 的物理下限拦住（判 Uncertain 而非 Loaded），
    # 尺寸也仍受 exporters 的越界门禁保护 —— 这两层不受本开关影响。
    # 现场可用 CARPOINT_CARRIAGE_ANCHOR=1 打开，不必改源码。
    carriage_anchor_enable: bool = field(
        default_factory=lambda: _env_flag("CARPOINT_CARRIAGE_ANCHOR", False)
    )
    carriage_anchor_deck_clearance: float = 0.60      # 高出最低水平面多少才算车体（m）
    carriage_anchor_bin: float = 0.05                 # 锚定直方图 bin 宽（m）
    carriage_anchor_wall_density_ratio: float = 0.15  # 稠密列门槛（占峰值比例）
    carriage_anchor_wall_height_ratio: float = 0.60   # 车厢段壁顶门槛（占最高壁顶比例）
    carriage_anchor_min_length: float = 2.0           # 车厢纵向最小跨度（m）
    carriage_anchor_width_range: Tuple[float, float] = (1.8, 3.2)  # 合理车厢外沿宽（m）
    # 空车壁面补全：前/后端壁也按车厢纵向中心面对称补点。只在端壁内表面
    # 已通过质量门禁时执行；与实测点 2cm 内重合的镜像点不会重复写入。
    carriage_complete_end_walls: bool = True
    carriage_completion_end_wall_band: float = 0.12
    carriage_completion_dedup_radius: float = 0.02
    carriage_completion_voxel: float = 0.005
    carriage_completion_floor_clearance: float = 0.05
    carriage_completion_width_margin: float = 0.30
    carriage_completion_height_margin: float = 0.12
    # 厢式/集装箱车厢补全（complete_box_carriage）。左右对称补点只适用于开顶栏板
    # 车 —— 它靠"把观测到的一侧镜像到另一侧"工作。厢式车两侧壁和顶面可能同时缺失
    # （实测一台 20ft 集装箱车：后端面 2605 点/m²、顶面 686、右侧 471，而中段
    # X 13.5~16.0 整段只剩一条 0.17m 宽的细线），此时没有可镜像的对象。
    # 改由两个端面重建箱体：端面正对雷达、观测最完整，它们的横断面直接给出箱宽
    # 和箱高，间距给出箱长，再按此重采样四个长面。
    carriage_box_completion_enable: bool = True
    # 矩形外壳诊断及预览/离线参考补全；不放宽正式内尺寸和空车档案门禁。
    carriage_wall_reference_enable: bool = True
    # 短矩形开顶车型的识别窗口。只用来判断"是不是这类车"，不提供任何尺寸：
    # 箱体高度一律取本次观测的上下沿。历史上这里还有一个标称整体高 2.9m，会把
    # 箱底下移到未观测的位置并进入导出点云，已移除 —— 车辆数据只能来自当前点云。
    # 仅新车型且前壁上缘有内折边实测支持时补局部连接带；不能用它生成整个顶盖。
    carriage_wall_reference_front_lip_enable: bool = True
    carriage_wall_reference_profile_length_range: Tuple[float, float] = (5.7, 6.4)
    carriage_wall_reference_profile_width_range: Tuple[float, float] = (2.4, 2.7)
    carriage_box_end_wall_band: float = 0.20   # 端面取点带宽（m）
    carriage_box_min_end_wall_points: int = 500
    carriage_box_surface_spacing: float = 0.04  # 重采样点间距（m）
    carriage_box_min_length: float = 2.0        # 两端面最小间距，防止把同一面拆成两个
    carriage_box_side_wall_band: float = 0.15   # 取侧壁顶高时贴壁带宽（m）
    carriage_box_max_yaw_deg: float = 15.0      # 超过即认为端面配对有误，不做旋转
    # 端面外表面定位时，一个 2cm 薄片要占端面带内多少比例才算"这是端面"。
    # 门把手、铰链、保险杠点数很少，绝对门槛会把它们当端面而使箱长虚长。
    # 实测（真值 6.06m）：门槛 0.002→+10.2cm，0.02→+8.0cm，0.10→−1.7cm。
    carriage_box_wall_face_fraction: float = 0.10
    # 面内覆盖判定：某个墙面位置周围这个半径内已经有实测点，就不再往那里补点。
    # 必须比三维去重半径大一个量级 —— 重建平面是整面的折中位置，真实墙面（波纹板、
    # 门锁件、板厚）会系统性偏离它，实测偏移中位 5.6~13.4cm，按 2cm 球半径去重
    # 一个点也删不掉，合成面就和真墙叠成相距十几厘米的两层皮。
    carriage_box_surface_dedup_radius: float = 0.12
    # 实测点归属到最近箱面的最大法向距离；超出的算箱体以外（底盘、车轮、后视镜）。
    carriage_box_face_assign_band: float = 0.35

    # 空车判别（无基准时）：车厢内部填充率三指标投票，≥2 票判空车。
    # 不能用栏板深度比“货物高度阈值”——真实货车栏板深 1~2m，这会让所有
    # 空车都被误判，历史空车参考也永远无法建立。
    empty_fill_ratio_max: float = 0.35        # 内部填充率中位数低于此值 → 空车票
    empty_high_cell_frac_max: float = 0.25    # 填充率>0.5 的网格占比上限
    empty_mid_density_max: float = 0.30       # 中部高度带点密度上限
    min_visible_wall_depth: float = 0.80      # 可见表面距栏板顶不足此值 → 满载/苫盖

    # 底板剖面（floor.py），规格见 docs/floor_profile_spec.md 第 5 节阈值表
    floor_profile_enable: bool = True
    floor_profile_bin: float = 0.20           # 沿车长的剖面分箱宽度（m）
    floor_profile_min_points: int = 20        # 每个分箱的最小点数
    floor_layer_tolerance: float = 0.05       # 贴近底板面的点层厚度（m）
    floor_flat_tolerance: float = 0.05        # 全长极差小于此值判平整
    floor_step_angle_min: float = 75.0        # 台阶 / 坡道分界角（度）
    floor_step_span_max_ratio: float = 0.25   # 落差区间占车厢长的比例上限
    floor_step_drop_min_ratio: float = 0.70   # 落差区间内高度变化占总极差的比例下限
    floor_plateau_tolerance: float = 0.08     # 落差前后各自"近水平"的容差（m）
    floor_slope_r2_min: float = 0.50          # 判整体坡度所需的剖面线性度（R²）

    # 局部坐标车厢提取（LocalCarriageExtractor）
    carriage_wall_prominence_ratio: float = 0.12
    carriage_wall_min_distance: float = 0.5       # 端面峰最小间距（m）
    carriage_edge_window_ratio: float = 0.12      # 端点窗口占比（补首尾峰）
    carriage_min_span_ratio: float = 0.5          # 车厢长度至少占整车比例
    carriage_end_gap_ratio: float = 0.15          # 至少一端贴近整体端点的容差比例

    # 输入坐标单位。生产采集链路应显式使用 ``m``，避免根据点云形状猜测单位；
    # ``auto`` 仅用于兼容历史离线 PCD，``mm`` 用于明确标注为毫米的旧数据。
    input_unit: str = "auto"
    mm_unit_threshold: float = 100.0
    mm_unit_min_span: float = 1000.0

    # 尺寸
    dimension_percentile_low: float = 1.0
    dimension_percentile_high: float = 99.0
    # 业务定义：车头到车厢前内壁的距离，必须在标定地面以上 1.40m 的
    # 水平截面量取车头前沿。默认先用 ±2cm 薄层，点数不足才扩到 ±5cm。
    cab_distance_measurement_height: float = 1.40
    cab_distance_height_half_band: float = 0.02
    cab_distance_max_height_half_band: float = 0.05
    cab_distance_min_slice_points: int = 30
    cab_distance_head_percentile: float = 99.0

    # 物理合理范围
    sanity_vehicle_length: Tuple[float, float] = (3.0, 30.0)
    sanity_vehicle_width: Tuple[float, float] = (1.0, 4.0)
    sanity_vehicle_height: Tuple[float, float] = (0.5, 5.0)
    sanity_carriage_min_ratio: float = 0.3

    # 输出
    # 生产链路只把计算结果直写 DM8，不生成 PNG 或结果 JSON。
    # 导出前处理后的纯净车辆点云（空车且触发补点时为补点后点云）。
    # 单份可达数十 MB，默认关闭，需要人工复核或建立永久空车档案时再开。
    save_clean_pointcloud: bool = False

    # 集成元数据
    camera_identifier: str = ""
    license_plate: str = ""

    # 区域融合检测货面变化的高度阈值（米）。
    load_height_threshold: float = 0.10
