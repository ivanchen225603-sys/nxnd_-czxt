"""地面剔除遇到立面时让位继续找地面的回归，不依赖现场 PCD。

回归背景（2026-09-24 现场）：平面按内点数排序，车停得靠前时露出的磅台面变小，
一面 27m 的立面排到它前面。旧实现遇立面即停，磅台面 4.6 万点一个未剔；残面复检
又要求双向跨度 ≥6m，而磅台面只有约 4m 宽，也没兜住。磅台面和车连成 28m 的簇，
车厢底板基准被取到台面上，同一辆空车连续三次被判 Loaded。
"""

import numpy as np
import pytest

from carpointcloud_analyzer.domain import preprocessing as pp

DATUM = np.array([0.0, 0.0, 1.0, 0.0])


def test_protected_sloping_surface_does_not_hide_its_ground_intersection(monkeypatch):
    """高位斜面与地面相交的低点仍须参加地面搜索，不能随整面保护后放回。"""
    from scipy.spatial import cKDTree

    x, y = np.meshgrid(np.linspace(0., 12., 61), np.linspace(-4., 4., 41))
    ground = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]
    x, y = np.meshgrid(np.linspace(0., 12., 61), np.linspace(-1., 1., 11))
    slope = np.c_[x.ravel(), y.ravel(), .5 * x.ravel()]
    # 固定候选次序，隔离“斜面先被保护”的分类缺陷；拟合收敛与门槛仍用真实实现。
    hypotheses = iter((np.array([-.5, 0., 1., 0.]) / np.sqrt(1.25), DATUM))

    def hypothesis(points, distance_threshold):
        plane = next(hypotheses, np.array([0., 1., 0., 0.]))
        return plane, np.flatnonzero(np.abs(points @ plane[:3] + plane[3]) <= distance_threshold)

    monkeypatch.setattr(pp, 'deterministic_plane_hypothesis', hypothesis)
    result = pp._remove_ground(np.vstack((ground, slope)), pp.PreprocessParams(), DATUM)
    assert np.all(cKDTree(result.removed).query(ground)[0] < 1e-9)
    high_slope = slope[slope[:, 2] > pp.PreprocessParams().ground_multilevel_max]
    assert np.all(cKDTree(result.remaining).query(high_slope)[0] < 1e-9)


def _plane(x_range, y_range, z, step):
    x, y = np.meshgrid(
        np.arange(*x_range, step), np.arange(*y_range, step), indexing="ij"
    )
    return np.column_stack([x.ravel(), y.ravel(), np.full(x.size, z)])


def _facade(x_range, y, z_range, step):
    """y=常数 的竖直面。"""
    x, z = np.meshgrid(
        np.arange(*x_range, step), np.arange(*z_range, step), indexing="ij"
    )
    return np.column_stack([x.ravel(), np.full(x.size, y), z.ravel()])


def _sorted(points):
    return points[np.lexsort(points.T[::-1])]


class _LogRecorder:
    """记录 info 日志，用来确认让位路径确实被走到，避免用例空转。"""

    def __init__(self):
        self.messages = []

    def info(self, message, *args, **kwargs):
        self.messages.append(message % args if args else message)

    def debug(self, *args, **kwargs):
        pass

    def warning(self, *args, **kwargs):
        pass

    def count(self, prefix):
        return sum(message.strip().startswith(prefix) for message in self.messages)


def _scene_deck_behind_facade():
    """周边地坪 > 立面 > 磅台面 > 车厢底板（按点数），地坪 z=0 为基准。"""
    apron = _plane((-20, 20), (-8, 8), 0.0, step=0.15)
    facade = _facade((-10, 10), 3.0, (1.2, 4.2), step=0.06)
    deck = _plane((-10, 10), (-2, 2), 0.7, step=0.1)      # 窄：只有 4m 宽
    floor = _plane((-5.5, 5.5), (-1.25, 1.25), 2.3, step=0.08)
    assert len(apron) > len(facade) > len(deck) > len(floor)
    return np.vstack([apron, facade, deck, floor]), apron, facade, deck, floor


def test_deck_ranked_behind_a_facade_is_still_removed(monkeypatch):
    """立面排在磅台面之前时，磅台面仍按地面剔除，立面与车厢底板保留。"""
    points, apron, facade, deck, floor = _scene_deck_behind_facade()
    original = points.copy()
    log = _LogRecorder()
    monkeypatch.setattr(pp, "logger", log)

    result = pp._remove_ground(points, pp.PreprocessParams(), DATUM)

    assert log.count("立面让位") == 1
    assert result.n_removed == len(apron) + len(deck)
    np.testing.assert_array_equal(_sorted(result.remaining), _sorted(np.vstack([facade, floor])))
    np.testing.assert_array_equal(_sorted(result.removed), _sorted(np.vstack([apron, deck])))
    np.testing.assert_array_equal(points, original)


def test_without_set_aside_the_deck_survives(monkeypatch):
    """同一场景禁用让位即复现现场缺陷：磅台面未剔，残面复检也因宽度不足不兜底。

    证明上一条用例确实覆盖了修复点，而不是被别的路径碰巧剔掉。
    """
    points, apron, _, deck, _ = _scene_deck_behind_facade()
    monkeypatch.setattr(pp, "MAX_SET_ASIDE_FACADES", 0)

    result = pp._remove_ground(points, pp.PreprocessParams(), DATUM)

    assert result.n_removed == len(apron)
    low = (result.remaining[:, 2] > 0.6) & (result.remaining[:, 2] < 0.8)
    assert np.count_nonzero(low) == len(deck)


def test_set_aside_without_new_ground_is_identical_to_stopping(monkeypatch):
    """让位后没再剔到地面：结果与遇立面即停逐点一致，点序也不变。

    让位期间还会新保护一块车厢底板；若不回退，它会被挪到数组末尾，点序改变。
    """
    apron = _plane((-20, 20), (-8, 8), 0.0, step=0.15)
    facade = _facade((-10, 10), 3.0, (1.2, 4.2), step=0.06)
    floor = _plane((-5.5, 5.5), (-1.25, 1.25), 2.3, step=0.08)
    # 低位带里的竖直件：让低位带占比达标以触发让位，但它本身不是地面
    low_wall = _facade((-5, 5), -3.0, (0.2, 0.9), step=0.05)
    points = np.vstack([apron, facade, floor, low_wall])
    params = pp.PreprocessParams()
    log = _LogRecorder()
    monkeypatch.setattr(pp, "logger", log)

    result = pp._remove_ground(points, params, DATUM)
    assert log.count("立面让位") >= 1, "场景须真的走到让位路径"
    assert log.count("保护疑似车辆表面平面") >= 1, "让位期间须新保护一块底板"
    monkeypatch.setattr(pp, "MAX_SET_ASIDE_FACADES", 0)
    stopped = pp._remove_ground(points, params, DATUM)

    assert result.n_removed == stopped.n_removed == len(apron)
    np.testing.assert_array_equal(result.remaining, stopped.remaining)
    np.testing.assert_array_equal(result.removed, stopped.removed)


def test_tilted_long_fit_after_set_aside_is_not_ground(monkeypatch):
    """让位后拿到的倾斜长面：中位数在地面带内，两端却切进车身，不得当地面剔除。

    09-24 10:46 实测：磅台面早已剔掉，让位后拿到一个跨 62m、倾斜约 5° 的拟合，
    内点离基准 -2.9~+2.6m、中位数 +0.31m。按旧的中位数判据它被剔掉，车身点随之
    丢失，这辆本已正常入库的空车变成内长缺测。
    """
    apron = _plane((-20, 20), (-8, 8), 0.0, step=0.15)
    facade = _facade((-10, 10), 3.0, (1.2, 4.2), step=0.06)
    x, y = np.meshgrid(np.arange(-30, 30, 0.1), np.arange(10, 14, 0.15), indexing="ij")
    tilted = np.column_stack([x.ravel(), y.ravel(), 0.5 + np.tan(np.radians(5)) * x.ravel()])
    assert len(apron) > len(facade) > len(tilted)
    points = np.vstack([apron, facade, tilted])
    params = pp.PreprocessParams()
    log = _LogRecorder()
    monkeypatch.setattr(pp, "logger", log)

    result = pp._remove_ground(points, params, DATUM)
    assert log.count("立面让位") >= 1, "场景须真的走到让位路径"
    monkeypatch.setattr(pp, "MAX_SET_ASIDE_FACADES", 0)
    stopped = pp._remove_ground(points, params, DATUM)

    np.testing.assert_array_equal(result.remaining, stopped.remaining)
    np.testing.assert_array_equal(result.removed, stopped.removed)

    # 去掉整层判据即复现误剔：证明拦住它的正是这道判据
    monkeypatch.setattr(pp, "MAX_SET_ASIDE_FACADES", 4)
    monkeypatch.setattr(pp, "_is_level_ground_layer", lambda *args: True)
    loose = pp._remove_ground(points, params, DATUM)
    assert loose.n_removed > stopped.n_removed + len(tilted) * 0.9


def _scene_many_facades():
    """地坪 > 6 面平行立面（各略多于磅台面）> 磅台面。"""
    apron = _plane((-20, 20), (-8, 8), 0.0, step=0.15)
    facades = [_facade((-10, 10), y, (1.2, 5.0), step=0.08) for y in (3, 4, 5, 6, 7, 8)]
    deck = _plane((-10, 10), (-2, 2), 0.7, step=0.1)
    assert all(len(apron) > len(f) > len(deck) for f in facades)
    return np.vstack([apron, *facades, deck]), deck


def test_facade_set_aside_is_bounded(monkeypatch):
    """每次让位都多一次全场拟合，次数必须有上限；用尽即按旧行为停止。"""
    points, deck = _scene_many_facades()
    log = _LogRecorder()
    monkeypatch.setattr(pp, "logger", log)

    bounded = pp._remove_ground(points, pp.PreprocessParams(), DATUM)

    assert log.count("立面让位") == pp.MAX_SET_ASIDE_FACADES
    low = (bounded.remaining[:, 2] > 0.6) & (bounded.remaining[:, 2] < 0.8)
    assert np.count_nonzero(low) == len(deck), "上限之后的磅台面不再追查"

    # 放宽上限即可拿到磅台面，说明停下来的正是上限本身
    monkeypatch.setattr(pp, "MAX_SET_ASIDE_FACADES", 6)
    unbounded = pp._remove_ground(points, pp.PreprocessParams(), DATUM)
    low = (unbounded.remaining[:, 2] > 0.6) & (unbounded.remaining[:, 2] < 0.8)
    assert np.count_nonzero(low) == 0


@pytest.mark.parametrize("datum", [None, DATUM])
def test_facade_before_any_ground_still_stops(datum, monkeypatch):
    """首个平面就是立面（尚未剔过地面）时不让位，保留原聚类/底板找回路径。"""
    facade = _facade((-10, 10), 3.0, (0.0, 3.5), step=0.03)
    deck = _plane((-10, 10), (-2, 2), 0.7, step=0.25)
    points = np.vstack([facade, deck])

    def unexpected(*args):
        pytest.fail("未剔过地面时不应进入残面复检")

    monkeypatch.setattr(pp, "_residual_ground_mask", unexpected)
    result = pp._remove_ground(points, pp.PreprocessParams(), datum)

    assert result.n_removed == 0
    np.testing.assert_array_equal(result.remaining, points)
