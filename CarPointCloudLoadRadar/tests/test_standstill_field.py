# -*- coding: utf-8 -*-
"""62组原始沉降采样的水平位置回归；不把稀疏采样冒充连续停稳回放。"""
import json
from pathlib import Path
import unittest

import numpy as np

from app.detect import StabilityDetector
from app.pointcloud import apply_roi, load_xyz, roi_as_ranges
from tests import _helpers as H


def _shape_drift(feature_a, feature_b):
    """与 `StabilityDetector` 同口径的形变位移：三档各自减去三档均值后取最大行范数。"""
    a = feature_a - feature_a.mean(axis=0)
    b = feature_b - feature_b.mean(axis=0)
    return float(np.max(np.linalg.norm(b - a, axis=1)))


def field_translation_statistics():
    """按原ROI和高度筛选读取三连帧，实际平移位置调用检测器的共用入口。"""
    _, _, calib, _, _ = H.ctx()
    config = calib.detection
    groups = {}
    counts = {}
    for channel in ("CH2", "CH3", "CH4"):
        paths = sorted((Path(H.ROOT) / "data/raw-62" / channel).glob("*of3.xyz"))
        counts[channel] = len(paths) // 3
        for path in paths:
            group, index = path.stem.rsplit("_", 1)
            if not group.startswith(channel + "_") or index not in ("1of3", "2of3", "3of3"):
                raise ValueError("现场三连帧命名不符: " + str(path))
            groups.setdefault(group, {})[index] = path

    current, legacy_xyz, legacy_xy, intervals, rows = [], [], [], [], []
    # 形变分量单独统计：它与平移共用 stationary_drift_m，但两者噪声底差一个
    # 数量级。近间隔/远间隔分开收，用来判断它是不是白噪声（见用例说明）。
    shape_near, shape_far, trans_far = [], [], []
    unequal = 0
    for group, members in sorted(groups.items()):
        if set(members) != {"1of3", "2of3", "3of3"}:
            raise ValueError("现场三连帧缺失: " + group)
        features, positions, elapsed = [], [], []
        for index in ("1of3", "2of3", "3of3"):
            path = members[index]
            meta = json.loads(Path(str(path) + ".meta.json").read_text(encoding="utf-8"))
            raw = load_xyz(str(path))
            if (meta["channel"] != group[:3] or meta["points"] != raw.n
                    or meta["ground_z"] != config.ground_z
                    or roi_as_ranges(meta["roi"]) != roi_as_ranges(calib.roi)):
                raise ValueError("现场三连帧身份、点数或原筛选参数不一致: " + str(path))
            # 保存坐标不再施加当前通道yaw；历史坐标参数与当前参数可能不同。
            cloud = apply_roi(raw, meta["roi"]).xyz
            vehicle = cloud[cloud[:, 2] > meta["ground_z"] + config.vehicle_min_height_m]
            if len(vehicle) < 3 or not np.isfinite(vehicle).all():
                raise ValueError("现场车辆点不足或含非有限值: " + str(path))
            feature = np.percentile(vehicle, [10., 50., 90.], axis=0)
            features.append(feature)
            positions.append(StabilityDetector._translation_position(feature))
            elapsed.append(float(meta["settle_elapsed_sec"]))
        if elapsed[0] != 0. or not elapsed[0] < elapsed[1] < elapsed[2]:
            raise ValueError("现场三连帧沉降采样顺序不符: " + group)
        for a, b in ((0, 1), (1, 2)):
            old_delta = np.median(features[b] - features[a], axis=0)
            distance = float(np.linalg.norm(positions[b] - positions[a]))
            current.append(distance)
            legacy_xyz.append(float(np.linalg.norm(old_delta)))
            legacy_xy.append(float(np.linalg.norm(old_delta[:2])))
            intervals.append(elapsed[b] - elapsed[a])
            unequal += int(not np.allclose(old_delta, features[b][1] - features[a][1],
                                            atol=1e-12, rtol=0.))
            rows.append(dict(group=group, pair=[a + 1, b + 1], distance_m=distance))
            shape_near.append(_shape_drift(features[a], features[b]))
        shape_far.append(_shape_drift(features[0], features[2]))
        trans_far.append(float(np.linalg.norm(positions[2] - positions[0])))

    def summarize(values):
        array = np.asarray(values)
        over = int(np.count_nonzero(array > config.stationary_drift_m))
        return dict(median_mm=float(np.median(array) * 1000.),
                    p90_mm=float(np.percentile(array, 90) * 1000.),
                    max_mm=float(array.max() * 1000.), over_limit=over,
                    over_ratio=over / len(array))

    if not current:
        raise ValueError("缺少62组现场三连帧，不能把空数据视为验收通过")
    return dict(groups=len(groups), channels=counts, pairs=len(current),
                shape_near=summarize(shape_near), shape_far=summarize(shape_far),
                shape_growth=float(np.median(shape_far) / np.median(shape_near)),
                translation_growth=float(np.median(trans_far) / np.median(current)),
                coordinate_basis="保存时原始坐标，不施加当前通道yaw",
                stationary_drift_m=config.stationary_drift_m,
                stationary_speed_mps=config.stationary_speed_mps,
                median_interval_sec=float(np.median(intervals)),
                unequal_to_p50_pairs=unequal, translation=summarize(current),
                legacy_xyz=summarize(legacy_xyz), legacy_xy=summarize(legacy_xy), rows=rows)


FIELD_DATA = Path(H.ROOT) / "data" / "raw-62"
# **62 组现场三连帧（约 489 MB）不在本仓库**：根 .gitignore 的 `**/data/`
#   把它排除了，而且那是现场原始数据，本来也不该进 git。
#
#   下面两条用例钉的是 CLAUDE.md「改动红线」第 6 条 —— 平移度量必须是
#   「车辆点的水平中位位置」。数据不在就跳过是**有代价的**：那条红线在这台
#   机器上没有守卫。所以跳过信息要写明去哪儿取数据，而不是静悄悄绿过去。
DATA_HINT = """缺现场三连帧数据：%s
    这两条用例守的是 CLAUDE.md 红线第 6 条（平移度量）。
    数据在归档仓库 D:\载重雷达逆向方案\dataaw-62\，
    做停稳算法相关改动前请先放到上面这个路径再跑。""" % FIELD_DATA

@unittest.skipUnless(FIELD_DATA.is_dir(), DATA_HINT)
class TestFieldTranslation(unittest.TestCase):
    def test_all_62_triplets_meet_horizontal_translation_budget(self):
        result = field_translation_statistics()
        self.assertEqual({"CH2": 25, "CH3": 13, "CH4": 24}, result["channels"])
        self.assertEqual(62, result["groups"])
        self.assertEqual(124, result["pairs"])
        self.assertEqual(.030, result["stationary_drift_m"])
        self.assertEqual(.020, result["stationary_speed_mps"])
        self.assertLessEqual(result["translation"]["p90_mm"], 15., result["translation"])
        self.assertLessEqual(result["translation"]["over_ratio"], .05, result["translation"])
        # 独立核实旧公式与p50差不等价，防止后续又用错误恒等式解释收益。
        self.assertEqual(113, result["unequal_to_p50_pairs"])

    def test_shape_channel_is_not_covered_by_the_translation_fix(self):
        """★ 形变分量仍含 z 三档，且与平移共用同一个 `stationary_drift_m`。

        平移已改成水平中位位置（p90 13 mm 量级），但
        `r.drift_m = max(drift, shape_drift, yaw_drift)` 里的 `shape_drift`
        用的还是完整三档三轴 —— 而车厢可见面在货面/厢壁/底板之间切换时，
        z 的三档间距就变，会被记成"形变"。两者噪声底差一个数量级，
        **一个门槛服务不了两个量纲不同的东西**。

        ▲ 这里**不断言形变超标是 bug**：本数据间隔 1.5 s，而生产是 100 ms
          且 `shape_drift` 比的是 2 秒窗均值，有平滑；三帧一组复现不了那个窗。
          另外这批是 2026-09-15 的数据，而连续停稳门控 09-17 才落地，
          "它们当时都测出来了"证明不了现行门控。
        ▲ 但**间隔翻倍时形变明显增长**（不是白噪声），说明它在 100 ms 上
          也不会自动消失，离线定不了门槛 -> 现场项，见 `docs/41` §0d。

        这条用例钉住上面两个事实，任一侧发生实质变化都该重新看一眼。
        """
        result = field_translation_statistics()
        self.assertGreater(result["shape_near"]["p90_mm"],
                           5. * result["translation"]["p90_mm"],
                           "形变与平移的噪声底不再相差一个数量级了，"
                           "共用 stationary_drift_m 的结论要重新评估")
        self.assertGreater(result["shape_growth"], 1.3,
                           "形变位移不再随间隔增长 = 变成白噪声了，"
                           "那就可以离线定门槛，docs/41 §0d 那条现场项可以撤")


if __name__ == "__main__":
    unittest.main()
