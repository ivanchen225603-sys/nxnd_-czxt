"""侧壁锚定回归：车辆簇混入台面构筑物时，剖面不得取到车外地面。

2026-09-09 电厂现场，一辆空车（内宽 2.3m、内深 2.7m）被判成 Loaded，
置信度 0.85。原因是车厢宽取自 ``np.percentile(width, [2, 98])``，
而车辆簇里混进了磅秤台面构筑物，分位数落到车外，两侧"壁顶"取成地面高度，
``wall_depth`` 算出 0.044m（实际 2.72m），触发"满载/苫盖"分支。
"""
import numpy as np

from carpointcloud_analyzer.domain.carriage import CarriageAnalyzer, anchor_carriage_walls
from carpointcloud_analyzer.domain.config import Config
from carpointcloud_analyzer.domain.load_status import LoadStatusClassifier

RNG = np.random.RandomState(20260909)

# 现场实测的空车：内宽 2.3m、栏板深 2.7m、底板离地 1.45m、厢长 5.4m。
FLOOR_Z = 1.45
WALL_TOP = FLOOR_Z + 2.7
HALF_W = 1.15
L_LO, L_HI = 0.0, 5.4


def _empty_carriage() -> np.ndarray:
    """一节看得见底板的空车厢：两侧栏板 + 平整底板 + 两道端壁。"""
    parts = []
    for sign in (-1.0, 1.0):
        n = 6000
        parts.append(np.column_stack([
            np.full(n, sign * HALF_W) + RNG.normal(0, 0.01, n),
            RNG.uniform(L_LO, L_HI, n),
            RNG.uniform(FLOOR_Z, WALL_TOP, n),
        ]))
    n = 9000
    parts.append(np.column_stack([
        RNG.uniform(-HALF_W, HALF_W, n),
        RNG.uniform(L_LO, L_HI, n),
        FLOOR_Z + RNG.normal(0, 0.01, n),
    ]))
    for end in (L_LO, L_HI):
        n = 2500
        parts.append(np.column_stack([
            RNG.uniform(-HALF_W, HALF_W, n),
            np.full(n, end) + RNG.normal(0, 0.01, n),
            RNG.uniform(FLOOR_Z, WALL_TOP, n),
        ]))
    return np.vstack(parts)


def _weighbridge_deck() -> np.ndarray:
    """车厢两侧和前后延伸出去的磅秤台面 —— 就是它把簇撑宽的。

    车厢底板会遮住它下面的台面，所以车厢足迹内不留台面点：俯视雷达
    看不穿底板，留了反而让底板面变成两层，不是真实工况。
    """
    n = 24000
    deck = np.column_stack([
        RNG.uniform(-2.6, 2.4, n),
        RNG.uniform(L_LO - 6.0, L_HI + 3.0, n),
        RNG.normal(0.0, 0.01, n),
    ])
    shadowed = (
        (np.abs(deck[:, 0]) <= HALF_W)
        & (deck[:, 1] >= L_LO) & (deck[:, 1] <= L_HI)
    )
    return deck[~shadowed]


def test_anchor_locks_onto_carriage_walls_not_the_cluster_extent():
    """锚定必须落在两道栏板上，而不是被台面撑开的簇边界上。"""
    cfg = Config()
    scene = np.vstack([_empty_carriage(), _weighbridge_deck()])

    anchor = anchor_carriage_walls(scene, cfg)

    assert anchor is not None, "两道侧壁清晰可见时不应锚定失败"
    w_lo, w_hi, l_lo, l_hi = anchor
    # 台面横向到 ±2.5m；锚定必须收在栏板（±1.15m）上。
    assert abs(w_lo + HALF_W) < 0.15, f"左壁锚偏了：{w_lo}"
    assert abs(w_hi - HALF_W) < 0.15, f"右壁锚偏了：{w_hi}"
    assert abs((w_hi - w_lo) - 2 * HALF_W) < 0.20
    # 台面纵向比车厢长 9m；锚定不得把它算进车厢。
    assert l_hi - l_lo < (L_HI - L_LO) + 1.0, f"纵向锚定被台面带长：{l_lo}~{l_hi}"


def test_profile_recovers_wall_depth_despite_deck_contamination():
    """壁深必须是真实内深，而不是被台面拉成接近零。"""
    cfg = Config(carriage_anchor_enable=True)
    scene = np.vstack([_empty_carriage(), _weighbridge_deck()])

    profile = CarriageAnalyzer(cfg).analyze(scene)

    assert profile.valid
    assert abs(profile.wall_depth - 2.7) < 0.25, (
        f"壁深 {profile.wall_depth:.3f}m 偏离实测 2.7m —— "
        "壁带可能又取到了车外地面"
    )
    assert profile.wall_depth >= cfg.min_visible_wall_depth, (
        "壁深低于满载阈值会让空车走进 visible_surface_near_wall_top 分支"
    )


def test_empty_carriage_is_not_reported_as_loaded():
    """端到端：混入台面的空车必须判成 Empty，而不是 Loaded。"""
    cfg = Config(carriage_anchor_enable=True)
    scene = np.vstack([_empty_carriage(), _weighbridge_deck()])

    status = LoadStatusClassifier(cfg).classify(scene)

    assert status.status == "Empty", (
        f"空车被判成 {status.status}（{status.reason}），"
        f"wallDepth={status.metrics.get('wallDepth')}"
    )


def test_clean_carriage_never_reaches_the_anchor_fallback(monkeypatch):
    """默认口径本来就成立时，锚定不得被调用 —— 既有车型的行为必须一字不改。

    锚定是给"壁带落到车外地面"这种失效兜底的。只要横向分位数仍然等于壁位，
    就必须原样走老路径；否则等于借修复之名改动一条一直稳定的链路。
    """
    cfg = Config(carriage_anchor_enable=True)
    clean = _empty_carriage()

    called = []
    monkeypatch.setattr(
        "carpointcloud_analyzer.domain.carriage.anchor_carriage_walls",
        lambda *a, **k: called.append(1),
    )
    profile = CarriageAnalyzer(cfg).analyze(clean)

    assert not called, "默认剖面成立时不应触发侧壁锚定"
    assert abs(profile.wall_depth - 2.7) < 0.20


def test_disabled_anchor_still_refuses_to_call_a_broken_profile_loaded():
    """开关关闭时（明天现场的实际配置）也不得把空车报成载货。

    锚定默认关闭，所以污染复现时壁深仍会是 0.04m 量级。此时必须落到
    ``carriage_profile_implausible`` 的 Uncertain，而不是 0.85 置信度的 Loaded ——
    这一层防护不受 ``carriage_anchor_enable`` 影响，是关闭状态下的最后兜底。
    """
    cfg = Config(carriage_anchor_enable=False)
    scene = np.vstack([_empty_carriage(), _weighbridge_deck()])

    status = LoadStatusClassifier(cfg).classify(scene)

    assert status.status != "Loaded", (
        f"剖面不可信时报成 Loaded 会让空车被拒测：{status.reason}"
    )
    assert status.status == "Uncertain"
    assert status.reason == "carriage_profile_implausible"


def test_anchor_declines_when_no_wall_pair_exists():
    """只有一片地面时必须锚定失败，而不是硬凑一对"侧壁"。"""
    cfg = Config()
    flat = _weighbridge_deck()

    assert anchor_carriage_walls(flat, cfg) is None


def test_anchor_defaults_off_and_follows_the_field_env_switch(monkeypatch):
    """默认关闭；现场用 CARPOINT_CARRIAGE_ANCHOR 切换，不必改源码。"""
    monkeypatch.delenv("CARPOINT_CARRIAGE_ANCHOR", raising=False)
    assert Config().carriage_anchor_enable is False

    monkeypatch.setenv("CARPOINT_CARRIAGE_ANCHOR", "1")
    assert Config().carriage_anchor_enable is True

    monkeypatch.setenv("CARPOINT_CARRIAGE_ANCHOR", "off")
    assert Config().carriage_anchor_enable is False
