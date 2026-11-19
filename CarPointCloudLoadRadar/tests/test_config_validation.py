# -*- coding: utf-8 -*-
"""配置校验 —— `docs/14` §1.4~§1.7 查出来的那一类缺陷的回归。

背景：2026-09-17 审查实测，**物理上不可能的门槛值被静默接受**，
后果是每辆车都误判、而且任何地方都不报错。在 10 份回归基线上跑出来的：

    floor_gate_min_inlier_ratio = 5.0（占比 > 1）  -> 10/10「底面内点占比过低」
    floor_gate_normal_z_min     = 2.0（|cos| > 1） -> 10/10「底面不够水平」
    normal_z_threshold          = 2.0             -> 10/10「未见厢底平面」
    roi 少写一个轴                                 -> roi=None，ROI **完全失效**

最后那条尤其致命：ROI 不生效 = 地面全进来，正是 2026-09-13
「空车道判成有车 -> 会话永不结束 -> 之后的车全被静默跳过」的入口条件。

本套件钉两件事：
  ① 坏配置**必须**在 load_* 就抛异常（不是运行时才崩，更不是静默）；
  ② **现有的真实配置必须照常加载** —— 校验表写得过严的话，
     现场一升级就全线起不来，那比不校验还糟。
"""
from __future__ import annotations

import io
import json
import os
import tempfile
import unittest

from tests import _helpers as H
from app.settings import (RANGES, check_range, check_unknown,
                          load_calib, load_channels, load_devices, load_settings)

CONFIG = os.path.join(H.ROOT, "config", "config.json")
DEVICES = os.path.join(H.ROOT, "config", "devices.json")
CHANNELS = os.path.join(H.ROOT, "config", "channels.json")


def _write(base_path: str, mutate) -> str:
    """复制一份配置、改一处、写到临时目录，返回路径。"""
    with io.open(base_path, encoding="utf-8") as f:
        d = json.loads(json.dumps(json.load(f)))
    mutate(d)
    p = os.path.join(tempfile.mkdtemp(), os.path.basename(base_path))
    with io.open(p, "w", encoding="utf-8") as f:
        f.write(json.dumps(d, ensure_ascii=False))
    return p


class TestRealConfigsStillLoad(unittest.TestCase):
    """★ 最重要的一条：校验表不许把现有配置拦下来。

    这条要是红了，说明表写过严 —— 现场升级会全线起不来。
    """

    def test_all_channels_load(self):
        st = load_settings("")
        self.assertTrue(st.channels)
        for c in st.channels:
            cal = load_calib(c.abs_config_path)
            dev = load_devices(c.abs_devices_path)
            self.assertIsNotNone(cal.roi, "%s 的 roi 解析成 None 了" % c.id)
            self.assertIsNotNone(dev.gate)


class TestRangeTableIsSane(unittest.TestCase):
    """校验表自身的体检。"""

    def test_every_rule_points_at_a_real_key(self):
        """★ 表里的键名打错的话，那条规则**永远不会触发** —— 和没写一样。

        所以逐条比对：RANGES 的键必须能在真实 config.json / devices.json 里找到
        （或者是本项目加的、有默认值但配置里没写的可选项）。
        """
        def flat(d, pre=""):
            out = {}
            for k, v in d.items():
                if isinstance(v, dict):
                    out.update(flat(v, pre + k + "."))
                else:
                    out[pre + k] = v
            return out

        with io.open(CONFIG, encoding="utf-8") as f:
            real = {"config." + k for k in flat(json.load(f))}
        with io.open(DEVICES, encoding="utf-8") as f:
            real |= {"devices." + k for k in flat(json.load(f))}
        # 本项目新增、有默认值但真实配置里没写的可选项
        optional = {
            "config.detection.stationary_min_sec",
            "config.detection.stationary_speed_mps",
            "config.detection.stationary_drift_m",
            "config.detection.stationary_speed_window_sec",
            "config.detection.stationary_max_gap_sec",
            "config.detection.stationary_jitter_m",
            "config.detect_empty.floor.y_trim_bin_m",
            "config.detect_empty.floor.y_trim_occupy",
            "config.detect_empty.floor.y_trim_below_m",
        }
        missing = sorted(set(RANGES) - real - optional)
        self.assertEqual(missing, [],
                         "RANGES 里这些键在真实配置里不存在，多半是打错了："
                         "%s" % missing)

    def test_bounds_are_ordered(self):
        for k, (lo, hi, why) in RANGES.items():
            if lo is not None and hi is not None:
                self.assertLess(lo, hi, "%s 的上下界写反了" % k)
            self.assertTrue(why, "%s 没写理由" % k)

    def test_check_range_accepts_the_middle(self):
        check_range("x", "devices.calibrate.floor_gate_min_inlier_ratio", 0.15)
        check_range("x", "config.detect_empty.floor.z_percentile", 30)

    def test_check_range_ignores_unknown_key(self):
        """表里没有的键不做限制 —— 不能因为漏写一条就把配置拦下来。"""
        check_range("x", "config.whatever.not_in_table", 1e9)

    def test_nan_is_rejected(self):
        with self.assertRaises(ValueError):
            check_range("x", "config.detection.settle_sec", float("nan"))


class TestImpossibleValuesAreRejected(unittest.TestCase):
    """物理上不可能的门槛 —— 每一条都是实测会让 10/10 全员误判的。"""

    def _bad_calib(self, mutate):
        with self.assertRaises(ValueError):
            load_calib(_write(CONFIG, mutate))

    def _bad_devices(self, mutate):
        with self.assertRaises(ValueError):
            load_devices(_write(DEVICES, mutate))

    def test_inlier_ratio_above_one(self):
        """占比 > 1：实测 10/10 全变「底面内点占比过低」。"""
        self._bad_devices(lambda d: d["calibrate"].__setitem__(
            "floor_gate_min_inlier_ratio", 5.0))

    def test_gate_normal_z_above_one(self):
        """|cos| > 1：实测 10/10 全变「底面不够水平」。"""
        self._bad_devices(lambda d: d["calibrate"].__setitem__(
            "floor_gate_normal_z_min", 2.0))

    def test_floor_normal_z_threshold_above_one(self):
        """实测 10/10 全变「未见厢底平面」。"""
        self._bad_calib(lambda d: d["detect_empty"]["floor"].__setitem__(
            "normal_z_threshold", 2.0))

    def test_percentile_out_of_range(self):
        """越界会让 np.percentile 在 fit_floor 里直接抛异常（运行时才崩）。"""
        self._bad_calib(lambda d: d["detect_empty"]["floor"].__setitem__(
            "z_percentile", 300))

    def test_negative_height_gate(self):
        self._bad_devices(lambda d: d["calibrate"].__setitem__(
            "floor_gate_z_above_min_max_m", -1.0))

    def test_negative_settle_sec(self):
        self._bad_calib(lambda d: d["detection"].__setitem__("settle_sec", -5))

    def test_release_ratio_above_one(self):
        self._bad_calib(lambda d: d["detection"].__setitem__(
            "vehicle_release_ratio", 1.5))

    def test_bad_port(self):
        self._bad_calib(lambda d: d["network"].__setitem__("udp_port", 99999))


class TestRoiMustBeComplete(unittest.TestCase):
    """roi 少一个轴 -> roi=None -> apply_roi 完全空操作 -> ROI 静默失效。"""

    def test_missing_axis_is_rejected(self):
        with self.assertRaises(ValueError) as cm:
            load_calib(_write(CONFIG, lambda d: d.__setitem__(
                "roi", {"x": [-2, 2], "y": [-11, 8]})))
        self.assertIn("roi", str(cm.exception))

    def test_unknown_axis_is_rejected(self):
        with self.assertRaises(ValueError):
            load_calib(_write(CONFIG, lambda d: d["roi"].__setitem__("w", [0, 1])))

    def test_reversed_bounds_still_rejected(self):
        """这条本来就有校验，一并钉住别回退。"""
        with self.assertRaises(ValueError):
            load_calib(_write(CONFIG, lambda d: d["roi"].__setitem__("x", [2, -2])))


class TestUnknownKeysAreRejected(unittest.TestCase):
    """打错配置项名字时，旧默认值会继续生效且不报错 —— 必须拦下来。"""

    def test_config_unknown_section(self):
        with self.assertRaises(ValueError):
            load_calib(_write(CONFIG, lambda d: d.__setitem__("zzz_section", {})))

    def test_config_unknown_key_in_detection(self):
        with self.assertRaises(ValueError):
            load_calib(_write(CONFIG, lambda d: d["detection"].__setitem__("zzz", 1)))

    def test_config_unknown_key_in_floor(self):
        with self.assertRaises(ValueError):
            load_calib(_write(CONFIG, lambda d: d["detect_empty"]["floor"]
                              .__setitem__("zzz", 1)))

    def test_config_unknown_subsection_in_detect_empty(self):
        with self.assertRaises(ValueError):
            load_calib(_write(CONFIG, lambda d: d["detect_empty"]
                              .__setitem__("zzz_wall", {})))

    def test_devices_unknown_section(self):
        with self.assertRaises(ValueError):
            load_devices(_write(DEVICES, lambda d: d.__setitem__("zzz", {})))

    def test_devices_unknown_key_in_calibrate(self):
        with self.assertRaises(ValueError):
            load_devices(_write(DEVICES, lambda d: d["calibrate"]
                                .__setitem__("zzz", 1)))

    def test_typo_in_a_real_key_is_caught(self):
        """现实里最常见的形态：少打一个字母。

        `floor_gate_min_inliers` 写成 `floor_gate_min_inlier` 时，
        以前是**旧默认值 600 继续生效、现场以为改了其实没改**。
        """
        def mutate(d):
            v = d["calibrate"].pop("floor_gate_min_inliers")
            d["calibrate"]["floor_gate_min_inlier"] = v
        with self.assertRaises(ValueError) as cm:
            load_devices(_write(DEVICES, mutate))
        self.assertIn("floor_gate_min_inlier", str(cm.exception))


class TestCheckUnknownMessage(unittest.TestCase):
    def test_message_lists_what_is_allowed(self):
        with self.assertRaises(ValueError) as cm:
            check_unknown("x.json", "calibrate", {"a": 1, "zzz": 2}, {"a", "b"})
        msg = str(cm.exception)
        self.assertIn("zzz", msg)
        self.assertIn("认识的是", msg)

    def test_non_dict_is_ignored(self):
        check_unknown("x.json", "calibrate", None, {"a"})
        check_unknown("x.json", "calibrate", [1, 2], {"a"})


if __name__ == "__main__":
    unittest.main()

class TestConfigFingerprint(unittest.TestCase):
    """落盘 meta 的配置指纹 —— `docs/14` §1.8。

    没有它的话，存档结论会随配置变化悄悄过期而无从察觉：
    2026-09-17 实测 62 份现场样本里已有 **7 份（11%）** 的 reason
    与用当前配置重算的结果不一致，其中 2 份现在是「空车」
    而 meta 记的是「底面过高，疑似载物」。
    而 `docs/13` 那套按车牌存基线的设计**正是照 meta 的 `empty` 挑基线的**。
    """

    def _fp(self, calib_mut=None, dev_mut=None, yaw=None):
        from app.settings import config_fingerprint
        cal = load_calib(_write(CONFIG, calib_mut) if calib_mut else CONFIG)
        dev = load_devices(_write(DEVICES, dev_mut) if dev_mut else DEVICES)
        return config_fingerprint(cal, dev, yaw)

    def test_same_config_same_sha(self):
        self.assertEqual(self._fp()["sha"], self._fp()["sha"])

    def test_readable_fields_are_there(self):
        fp = self._fp()
        for k in ("sha", "z_above_mode", "yaw_offset_deg", "ground_z", "roi"):
            self.assertIn(k, fp)

    def test_channel_yaw_changes_the_sha(self):
        """逐通道 yaw 改了会实打实改判定结果 —— 必须进指纹。

        2026-09-16 改过一次（CH2/CH3/CH4），正是那 7 份不一致的成因之一。
        """
        self.assertNotEqual(self._fp(yaw=-181.25)["sha"], self._fp(yaw=-180.0)["sha"])

    def test_every_judgement_gate_changes_the_sha(self):
        """★ 自检：指纹表里的每一个门槛，改了都必须让指纹变（按表自动取键）。

        以前这里是一份**手抄的**键清单 —— 指纹表后来加了 bed_exposure 等
        3 个键它不知道，守卫自己先漏了（两轮审查 H3；settings.py 的注释
        还点名过一个根本不存在的测试类）。现在直接按
        `_FINGERPRINT_DEVICES_KEYS` 全表迭代：将来往表里加任何判定门槛，
        这条用例自动跟着守，不用再记得回来抄一遍。
        """
        from app.settings import _FINGERPRINT_DEVICES_KEYS
        base = self._fp()["sha"]
        seen = set()
        for dotted in _FINGERPRINT_DEVICES_KEYS:
            section, _, key = dotted.partition(".")
            self.assertEqual(section, "calibrate", "指纹 devices 键都应在 calibrate 段")
            self.assertNotIn(key, seen, "指纹表里有重复键: %s" % key)
            seen.add(key)
            if key == "floor_gate_z_above_mode":
                # 枚举键不能乘 0.5，它有专门测试（test_z_above_mode_changes_the_sha）。
                continue

            def mut(d, key=key):
                # 不能只乘 0.5：现场配的 bed_exposure_min 是 0.0，乘完还是 0.0，
                # 指纹当然不变 —— 那是测试没改到值，不是指纹漏了（当场踩过）。
                # +0.037 保证任何起点都变；int 键经 load_devices 强转后仍必然异于原值。
                d["calibrate"][key] = float(d["calibrate"][key]) * 0.5 + 0.037
            self.assertNotEqual(self._fp(dev_mut=mut)["sha"], base,
                                "改 %s 之后指纹没变 —— 它没进指纹表" % key)
        self.assertGreaterEqual(len(seen), 12,
                                "指纹表至少 12 个门槛 —— 少了先查表是不是被人删了")

    def test_z_above_mode_changes_the_sha(self):
        """第 10 步的度量换了，4 份回归基线的结论会翻 —— 必须进指纹。"""
        base = self._fp()["sha"]
        fp = self._fp(dev_mut=lambda d: d["calibrate"].__setitem__(
            "floor_gate_z_above_mode", "origin"))
        self.assertNotEqual(fp["sha"], base)
        self.assertEqual(fp["z_above_mode"], "origin")

    def test_roi_and_ground_z_change_the_sha(self):
        base = self._fp()["sha"]
        self.assertNotEqual(
            self._fp(calib_mut=lambda d: d["roi"].__setitem__("y", [-10, 8]))["sha"],
            base)
        self.assertNotEqual(
            self._fp(calib_mut=lambda d: d["detection"]
                     .__setitem__("ground_z", -5.0))["sha"], base)

    def test_floor_fit_params_change_the_sha(self):
        base = self._fp()["sha"]
        self.assertNotEqual(
            self._fp(calib_mut=lambda d: d["detect_empty"]["floor"]
                     .__setitem__("z_percentile", 25))["sha"], base)

    def test_irrelevant_change_keeps_the_sha(self):
        """与结论无关的项（落盘目录）改了不该报警 —— 否则天天误报。"""
        self.assertEqual(
            self._fp(dev_mut=lambda d: d["calibrate"]
                     .__setitem__("loaded_output_dir", "data/zzz"))["sha"],
            self._fp()["sha"])


class TestConfigMismatch(unittest.TestCase):
    def test_same_is_empty(self):
        from app.settings import config_fingerprint, config_mismatch
        fp = config_fingerprint(load_calib(CONFIG), load_devices(DEVICES), -181.25)
        self.assertEqual(config_mismatch(fp, fp), [])

    def test_old_sample_without_fingerprint(self):
        from app.settings import config_fingerprint, config_mismatch
        fp = config_fingerprint(load_calib(CONFIG), load_devices(DEVICES), None)
        for saved in (None, {}, {"sha": ""}):
            why = config_mismatch(saved, fp)
            self.assertTrue(why)
            self.assertIn("还没有记配置指纹", why[0])

    def test_difference_is_spelled_out(self):
        from app.settings import config_fingerprint, config_mismatch
        a = config_fingerprint(load_calib(CONFIG), load_devices(DEVICES), -181.25)
        b = config_fingerprint(load_calib(CONFIG), load_devices(DEVICES), -180.0)
        why = config_mismatch(a, b)
        self.assertTrue(why)
        self.assertTrue(any("航向角" in x for x in why),
                        "差异清单没点出是航向角变了：%s" % why)


class TestChannelsUnknownKeys(unittest.TestCase):
    """★ `channels.json` 的未知键校验（2026-09-22 补，见 docs/55 H2）。

    在此之前，`load_calib` / `load_devices` 每一层都接了 `check_unknown`，
    **唯独 `channels.json` 全程裸奔** —— 而按 `CLAUDE.md` 的说法，
    它才是「现场主要改这个」的那一份。实测打错一个字母全部静默接受：

        service.state_push_hz -> state_push_hzz   静默用默认值
        plate.f2_timeout_ms   -> f2_timeout_mss   静默用默认值
        lidar.yaw_offset_deg  -> yaw_offset_degg  静默退回共用值

    最后一条尤其要命：逐通道 yaw 静默失效 = 车厢按错误角度切，
    而 `CLAUDE.md` 用整整一节讲反向补偿会把车厢切掉。

    ▲ 这个类的名字被 `app/settings.py` 的注释点名了 —— 改名要同步改那里。
      （`TestFingerprintCoversTheChain` 就是反面教材：注释点了名，
      测试根本不存在，见 docs/55 H3。）
    """

    def _load(self, mutate):
        return load_channels(_write(CHANNELS, mutate))

    def test_the_shipped_config_still_loads(self):
        """★ 最重要的一条：新校验不许把现有配置拦下来（现场升级会全线起不来）。"""
        sv, chans, plate, stab = load_channels(CHANNELS)
        self.assertTrue(chans, "现有 channels.json 解析不出通道")
        self.assertEqual(4, len(chans))
        ch2 = [c for c in chans if c.id == "CH2"][0]
        # 逐通道 yaw 与量纲是现场最要命的两项，顺带钉住它们确实被读进来了
        self.assertEqual(-180.0, ch2.lidar.yaw_offset_deg)
        self.assertTrue(ch2.lidar.source_ip)
        self.assertTrue(sv.host)
        self.assertEqual(sv.enter_step_m, 0.10)
        self.assertTrue(sv.enter_crop_roi)
        self.assertEqual(sv.enter_frames, 20)

    def test_enter_sampling_options_reject_invalid_values(self):
        """位移必须是非负有限数，裁剪开关必须是 JSON 布尔。"""
        cases = (("enter_step_m", -0.1), ("enter_step_m", True),
                 ("enter_step_m", "0.1"), ("enter_crop_roi", "false"),
                 ("enter_crop_roi", 1))
        for key, value in cases:
            with self.subTest(key=key, value=value):
                with self.assertRaises(ValueError):
                    self._load(lambda d: d["service"].__setitem__(key, value))
        for value in (float("nan"), float("inf")):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    self._load(lambda d: d["service"].__setitem__("enter_step_m", value))

    def test_exit_sampling_options(self):
        """离场帧：采集默认关；exit_volume 默认开；非法值启动即报错。"""
        sv, _, _, _ = load_channels(CHANNELS)
        self.assertEqual(0, sv.exit_frames)
        self.assertTrue(sv.exit_volume)
        sv, _, _, _ = self._load(lambda d: d["service"].update(exit_frames=12, exit_step_m=0.2,
                                                                exit_crop_roi=False, exit_volume=False))
        self.assertEqual((12, 0.2, False, False),
                         (sv.exit_frames, sv.exit_step_m, sv.exit_crop_roi, sv.exit_volume))
        cases = (("exit_step_m", -0.1), ("exit_step_m", "0.1"), ("exit_step_m", float("nan")),
                 ("exit_crop_roi", "false"), ("exit_frames", -3), ("exit_volume", "true"))
        for key, value in cases:
            with self.subTest(key=key, value=value):
                with self.assertRaises(ValueError):
                    self._load(lambda d: d["service"].__setitem__(key, value))

    def test_a_typo_in_every_section_is_rejected(self):
        cases = [
            ("service", lambda d: d["service"].__setitem__("state_push_hzz", 5)),
            ("plate", lambda d: d["plate"].__setitem__("f2_timeout_mss", 1)),
            ("stability", lambda d: d["stability"].__setitem__("result_hold_secs", 1)),
            ("channel", lambda d: d["channels"][1].__setitem__("output_dirr", "x")),
            ("lidar", lambda d: d["channels"][1]["lidar"].__setitem__("yaw_offset_degg", -180.)),
            ("顶层", lambda d: d.__setitem__("不存在的段", {})),
        ]
        for name, mutate in cases:
            with self.subTest(name):
                with self.assertRaises(ValueError, msg="%s 段的打错名字没被拦下" % name):
                    self._load(mutate)

    def test_the_message_names_the_channel_and_the_key(self):
        """4 路配置里只说"channels 段"找起来很费劲，必须点名是第几路。"""
        with self.assertRaises(ValueError) as cm:
            self._load(lambda d: d["channels"][1]["lidar"].__setitem__("yaw_offset_degg", -180.))
        text = str(cm.exception)
        self.assertIn("CH2", text)
        self.assertIn("yaw_offset_degg", text)
        self.assertIn("yaw_offset_deg", text, "要把认识的键列出来，否则不知道正确拼法")

    def test_comment_keys_are_still_allowed(self):
        """`channels.json` 里按约定用 `_说明` 写注释 —— 不能把它们当错误。"""
        for name, mutate in (
                ("顶层", lambda d: d.__setitem__("_说明2", "注释")),
                ("service", lambda d: d["service"].__setitem__("_说明", "注释")),
                ("通道", lambda d: d["channels"][1].__setitem__("_备注", "注释")),
                ("lidar", lambda d: d["channels"][1]["lidar"].__setitem__("_备注", "注释")),
        ):
            with self.subTest(name):
                self.assertTrue(self._load(mutate)[1], "%s 的注释键被当成错误了" % name)

    def test_every_key_the_loader_reads_is_in_the_allow_list(self):
        """★ 自检：白名单漏了某个**真会被读**的键，等于把好配置拦下来。

        反面教材是 lidar 段的 `replay_dir` —— 它看着不像雷达参数，
        但通道级 `replay_dir` 缺失时确实会回退到它。
        """
        from app import settings as S
        for section, allowed, probes in (
                ("lidar", S._CHANNELS_LIDAR_KEYS, ["replay_dir", "yaw_offset_deg", "source_ip"]),
                ("channel", S._CHANNELS_CHANNEL_KEYS, ["replay_interval", "replay_loop", "save"]),
                ("service", S._CHANNELS_SERVICE_KEYS, ["enter_min_points", "state_push_hz"]),
                ("plate", S._CHANNELS_PLATE_KEYS, ["headers", "sample_plates", "f3_timeout_ms"]),
        ):
            for key in probes:
                with self.subTest("%s.%s" % (section, key)):
                    self.assertIn(key, allowed)

    def test_lidar_replay_dir_actually_round_trips(self):
        """接着上一条：证明 lidar.replay_dir 真的被读，不是白名单里的死键。"""
        _sv, chans, _p, _s = self._load(
            lambda d: (d["channels"][1].pop("replay_dir", None),
                       d["channels"][1]["lidar"].__setitem__("replay_dir", "data/somewhere")))
        self.assertEqual("data/somewhere", [c for c in chans if c.id == "CH2"][0].replay_dir)
