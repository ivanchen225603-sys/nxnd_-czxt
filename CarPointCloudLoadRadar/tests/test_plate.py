# -*- coding: utf-8 -*-
"""
车牌模块测试。

相机接口无法从二进制还原（stopdetect.dll 里没有任何 URL），
所以这里的重点不是"调通真相机"，而是：
    ① _extract() 能从各种 JSON 形状里正确抠出车牌（plate_field 是现场最容易填错的地方）
    ② safe_dir_name() 把 GBK 乱码的车牌目录名修好（这是现场真实踩到的坑）
    ③ 没配相机时必须安全降级为 Null 读取器，服务照常跑
"""
from __future__ import annotations

import unittest

from tests import _helpers as H
from app.settings import PlateCfg
from app.plate import (HttpPlateReader, NullPlateReader, SimulatedPlateReader,
                       build_plate_reader, safe_dir_name, EMPTY_PLATE,
                       DEFAULT_PLATE_REGEX, _looks_like_no_plate)


def _http_cfg(**kw) -> PlateCfg:
    cfg = PlateCfg()
    cfg.enabled = True
    cfg.base_url = "http://10.0.0.9"
    for k, v in kw.items():
        setattr(cfg, k, v)
    return cfg


class TestPlateExtract(unittest.TestCase):

    def test_plain_json_field(self):
        r = HttpPlateReader(_http_cfg(plate_field="data.plate"))
        plate, conf = r._extract('{"data": {"plate": "\u5b81A09692D"}}')
        self.assertEqual(plate, "宁A88635W")
        self.assertGreater(conf, 0)

    def test_nested_and_list_path(self):
        r = HttpPlateReader(_http_cfg(plate_field="result.0.plateNo"))
        plate, _ = r._extract('{"result": [{"plateNo": "\u8499C02002F"}]}')
        self.assertEqual(plate, "蒙C88593X")

    def test_top_level_field(self):
        r = HttpPlateReader(_http_cfg(plate_field="plate"))
        plate, _ = r._extract('{"plate": "\u5b81AG3661"}')
        self.assertEqual(plate, "宁AV3808")

    def test_field_missing_falls_back_to_regex(self):
        """plate_field 填错了也要靠全文正则兜住 —— 现场最常见的情况。"""
        r = HttpPlateReader(_http_cfg(plate_field="wrong.path"))
        plate, conf = r._extract('{"msg": "ok", "text": "\u5b81A3F7K2"}')
        self.assertEqual(plate, "宁A9H5D5")
        self.assertLess(conf, 1.0)                    # 正则兜底的置信度低一档

    def test_no_plate_at_all(self):
        r = HttpPlateReader(_http_cfg(plate_field="data.plate"))
        plate, conf = r._extract('{"data": {"plate": ""}}')
        self.assertEqual(plate, "")
        self.assertEqual(conf, 0.0)

    def test_non_json_response(self):
        r = HttpPlateReader(_http_cfg())
        plate, _ = r._extract("plain text \u5b81A09692D end")
        self.assertEqual(plate, "宁A88635W")

    def test_default_regex_matches_real_plates(self):
        import re
        rx = re.compile(DEFAULT_PLATE_REGEX)
        for p in ("宁A88635W", "宁AV3808", "蒙C88593X", "宁A9H5D5"):
            self.assertTrue(rx.search(p), f"默认正则应匹配 {p}")

    def test_url_template_fill(self):
        r = HttpPlateReader(_http_cfg(
            device_id="7", camera_identifier="cam1",
            snap_url="/api/snap?dev={device_id}&cam={camera_identifier}"))
        self.assertEqual(r._url(r.cfg.snap_url),
                         "http://10.0.0.9/api/snap?dev=7&cam=cam1")


class TestSafeDirName(unittest.TestCase):
    """
    修 GBK→UTF-8 的目录名污染。

    现场真实事故：车牌"宁A88635W"按 GBK 编码后被当成 UTF-8 解释，
    变成"瀹丄09692D"这种乱码目录名 —— 样本还能存，但按车牌检索就废了。
    历史样本里那 4 个乱码目录就是证据。
    """

    def test_normal_plate_untouched(self):
        self.assertEqual(safe_dir_name("宁A88635W"), "宁A88635W")

    def test_case_folded_so_one_car_one_dir(self):
        """现场同时出现过 蒙C88593X 与 蒙c88593X 两个目录 —— 必须折叠成同一个。"""
        self.assertEqual(safe_dir_name("蒙C88593X"), safe_dir_name("蒙c88593x"))
        self.assertEqual(safe_dir_name("宁a88635w"), "宁A88635W")

    def test_truncated_names_stay_unique(self):
        """超长车牌截断后必须仍能区分，否则两辆车会共用目录。"""
        a = safe_dir_name("宁" * 400 + "A")
        b = safe_dir_name("宁" * 400 + "B")
        self.assertNotEqual(a, b)

    def test_strips_windows_illegal_chars(self):
        out = safe_dir_name('宁A<09>6:9"2D|?*')
        for ch in '<>:"/\\|?*':
            self.assertNotIn(ch, out)

    def test_strips_drive_and_separators(self):
        out = safe_dir_name("C:/a\\b/宁A88635W")
        self.assertNotIn("/", out)
        self.assertNotIn("\\", out)
        self.assertNotIn(":", out)

    def test_empty_becomes_placeholder(self):
        self.assertEqual(safe_dir_name(""), EMPTY_PLATE)
        self.assertEqual(safe_dir_name("   "), EMPTY_PLATE)

    def test_length_capped(self):
        self.assertLessEqual(len(safe_dir_name("宁" * 500)),
                              len("宁" * 500))
        self.assertTrue(len(safe_dir_name("宁" * 500)) <= 200)


class TestBuildReader(unittest.TestCase):

    def test_disabled_gives_null(self):
        cfg = PlateCfg()
        cfg.enabled = False
        self.assertIsInstance(build_plate_reader(cfg), NullPlateReader)

    def test_explicit_mode_wins(self):
        self.assertIsInstance(build_plate_reader(PlateCfg(), mode="simulate"),
                              SimulatedPlateReader)
        self.assertIsInstance(build_plate_reader(PlateCfg(), mode="null"),
                              NullPlateReader)

    def test_configured_urls_give_http(self):
        cfg = _http_cfg(snap_url="/snap", result_url="/result")
        self.assertIsInstance(build_plate_reader(cfg), HttpPlateReader)

    def test_enabled_but_no_url_still_null(self):
        """开了开关却没填地址 → 宁可 Null（不崩），也不能瞎猜协议。"""
        cfg = PlateCfg()
        cfg.enabled = True
        self.assertIsInstance(build_plate_reader(cfg), NullPlateReader)


class TestReaders(unittest.TestCase):

    def test_simulated_reader_returns_sample_plate(self):
        cfg = PlateCfg()
        cfg.sample_plates = ["宁A88635W"]
        r = SimulatedPlateReader(cfg, plates=cfg.sample_plates)
        res = r.read()
        self.assertTrue(res.ok)
        self.assertEqual(res.plate, "宁A88635W")

    def test_null_reader_always_empty(self):
        r = NullPlateReader(PlateCfg())
        res = r.read()
        self.assertFalse(res.ok)
        self.assertEqual(res.plate, EMPTY_PLATE)

    def test_reader_info_shape(self):
        for r in (NullPlateReader(PlateCfg()),
                  SimulatedPlateReader(PlateCfg(), plates=["宁A88635W"])):
            info = r.info()
            self.assertIn("kind", info)


class TestSimulatedPlateHolds(unittest.TestCase):
    """
    仿真出牌的"防穿帮"测试。

    四路仿真看板同时开着时，如果每路都从 sample_plates 的第一张起牌，
    屏幕上就是四张卡片齐刷刷"宁A88635W" —— 一眼就看出是假的。
    故 build_plate_reader(rotate=i) 让第 i 路从第 i 张起牌
    （见 plate.SimulatedPlateReader 与 service.ChannelWorker(plate_rotate=i)）。
    """

    PLATES = ["宁A88635W", "宁AV3808", "蒙C88593X", "宁A9H5D5"]

    def _cfg(self):
        cfg = PlateCfg()
        cfg.sample_plates = list(self.PLATES)
        return cfg

    def test_rotate_shifts_starting_plate(self):
        got = []
        for i in range(4):
            r = SimulatedPlateReader(self._cfg(), plates=self.PLATES,
                                     delay=0.0, rotate=i)
            got.append(r.read().plate)
        self.assertEqual(got, self.PLATES)
        self.assertEqual(len(set(got)), 4, f"四路首发车牌有重复：{got}")

    def test_default_starts_from_first(self):
        r = SimulatedPlateReader(self._cfg(), plates=self.PLATES, delay=0.0)
        self.assertEqual(r.read().plate, self.PLATES[0])

    def test_rotate_wraps_around(self):
        r = SimulatedPlateReader(self._cfg(), plates=self.PLATES,
                                 delay=0.0, rotate=6)      # 6 % 4 == 2
        self.assertEqual(r.read().plate, self.PLATES[2])

    def test_rotate_never_index_errors(self):
        for rot in (-9, 0, 1, 3, 4, 99):
            r = SimulatedPlateReader(self._cfg(), plates=self.PLATES,
                                     delay=0.0, rotate=rot)
            self.assertTrue(r.read().ok, f"rotate={rot} 出牌失败")

    def test_cycles_through_all_plates(self):
        """一轮下来四张牌都要出过（"车队"观感）。"""
        r = SimulatedPlateReader(self._cfg(), plates=self.PLATES,
                                 delay=0.0, rotate=0)
        seen = [r.read().plate for _ in range(4)]
        self.assertEqual(sorted(seen), sorted(self.PLATES))

    def test_build_passes_rotate_through(self):
        r = build_plate_reader(self._cfg(), mode="simulate", rotate=2)
        self.assertIsInstance(r, SimulatedPlateReader)
        self.assertEqual(r.read().plate, self.PLATES[2])

    def test_build_default_rotate_is_zero(self):
        r = build_plate_reader(self._cfg(), mode="simulate")
        self.assertEqual(r.read().plate, self.PLATES[0])


class TestNoPlatePlaceholder(unittest.TestCase):
    """相机"这次没识别出来"的各种写法不能被当成车牌。

    plate_field 指到的字段即使正则没匹配上也照收（军牌/港澳/教练车不在默认正则里），
    但如果收的是 "未识别"/"-"/"null" 这类占位值，就会：
        · 在输出目录下建出 `未识别/`、`-/` 这种目录，同一堆车全混在一起；
        · 看板显示"车牌已取到"，现场以为相机是好的。
    必须落回 EMPTY_PLATE（未知车牌）。
    """

    PLACEHOLDERS = ("", " ", "-", "--", "null", "NULL", "None", "n/a", "N/A",
                    "0", "000000", "unknown", "无", "无牌", "无车牌", "无牌车",
                    "未识别", "识别失败", "未知车牌", "???", "...")

    REAL = ("宁A88635W", "蒙C88593X", "WJ11234", "使123456", "粤Z1234港",
            "京AS66928", "沪A68532学")

    def test_placeholders_are_rejected(self):
        for v in self.PLACEHOLDERS:
            self.assertTrue(_looks_like_no_plate(v), f"{v!r} 应当被当成占位值")

    def test_real_plates_pass(self):
        for v in self.REAL:
            self.assertFalse(_looks_like_no_plate(v), f"{v!r} 是真车牌，不能被挡掉")

    def test_extract_drops_placeholder_field(self):
        r = HttpPlateReader(_http_cfg(plate_field="data.plate"))
        for v in ("未识别", "-", "null", "无牌车"):
            plate, conf = r._extract('{"data": {"plate": "%s"}}' % v)
            self.assertEqual(plate, "", f"{v!r} 被当成车牌了")
            self.assertEqual(conf, 0.0)

    def test_extract_keeps_unusual_but_real_plate(self):
        """正则覆盖不到的真车牌（军牌）仍要收下 —— 别为了挡占位值把真车牌卡掉。"""
        r = HttpPlateReader(_http_cfg(plate_field="data.plate"))
        plate, conf = r._extract('{"data": {"plate": "WJ11234"}}')
        self.assertEqual(plate, "WJ11234")
        self.assertEqual(conf, 1.0)


if __name__ == "__main__":
    unittest.main()
