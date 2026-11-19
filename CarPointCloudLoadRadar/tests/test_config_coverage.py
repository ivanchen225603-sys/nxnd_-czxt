#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tests/test_config_coverage.py — 配置项不许"配了没人用"

本项目的一条硬规矩是：**配置项不认识就启动即报错，不静默忽略**
（曾有现场打错 `range_mode` 一个字母导致产出错量纲点云且不报错）。
反过来的那一半同样要守：**不能存在"配置里有、代码里没人读"的键** ——
现场照着调半天没有任何效果，比报错难查得多。

这组用例把"哪些配置字段在代码里找不到引用"锁成一份清单：
    · 冒出新的未使用字段 → 红。要么去用它，要么在下面显式登记并写明原因；
    · 清单里的字段被用起来了 → 也红。说明清单过期了，删掉那一行。

登记不是豁免，是**留痕**：每一条都得说清楚"为什么它不生效"，
这样交接的人一眼知道调它没用，而不是对着配置文件瞎试。

（`group` 是个反例：它曾经在文档里被描述成"填上就会聚合"，而 service.py 里
 根本没有聚合分支 —— 现在由 `settings.check_groups()` 启动即报错拦住，
 见 tests/test_settings.py。）
"""
from __future__ import annotations

import ast
import io
import os
import re
import unittest

from tests import _helpers as H                # noqa: F401  （把项目根加进 sys.path）
from app.settings import root

# 字段名 → 为什么它现在不生效（登记即留痕，必须写人话）
KNOWN_INERT = {
    # ── 原厂 devices.json 的键：刻意原样保留，方便与现场旧文件互换；
    #    本项目的同类信息一律走 config/channels.json ──
    "lidar_ip": "原厂 devices.json 遗留；本项目按 channels.json 的 lidar.ip/source_ip 走",
    "ip_override": "原厂遗留；本项目没有这一层覆盖",
    "lidar_config_path": "原厂遗留；本项目按 channels.json 的 config_path 走",
    "loaded_output_dir": "原厂遗留；本项目按 channels.json 的 output_dir 走",
    "entry_baseline_dir": "原厂遗留；本项目按 channels.json 的 baseline_dir 走",

    # ── 被「功能等价实现」取代的拟合参数：调它们**没有任何效果** ──
    "expand_ratio": "底面拟合改成『z 最低 30% 做种子 → PCA → 迭代收紧』，不再按比例外扩",
    "z_percentile_hi": "顶面拟合改成『极值往内 seed_band_m 的窄带做种子』，不再取高分位",
    "y_percentile_rear": "后壁拟合同上，改走窄带种子",
    "overhang": "原版的『探出货物』检查本项目没有实现（10 份历史样本里没有触发它的）",
    "z_margin": "同 overhang：整段参数没有对应实现",

    # ── 报文布局里的说明性字段 ──
    "angle_bins": "说明性字段（一圈 900 个方位角）；解码按 angle1/2_offset 取角度，不用它",

    # ── IMU：报文里解得出来，但整条链路没有使用者 ──
    "imu_warmup": "IMU 数据本项目不参与任何判定（radar_packet.imu_magnitude 只有测试在调）",
    "imu_acc_x_offset": "同上，IMU 未接入判定",
    "imu_acc_y_offset": "同上，IMU 未接入判定",
    "imu_acc_z_offset": "同上，IMU 未接入判定",

    # ── 没有实现的功能 ──
    "log_level": "本项目没有日志框架；关键事件由 service.ConsoleLogger 打到 stdout",
}

# desktop/ 不在本项目：目标平台是麒麟，不做 Windows 桌面打包。
SCAN_DIRS = ("app", "cli", "tools")
SKIP_FILE = os.path.normcase(root("app", "settings.py"))


def _declared_fields():
    """settings.py 里所有 dataclass 的字段名 → 所属类。"""
    tree = ast.parse(io.open(root("app", "settings.py"), encoding="utf-8").read())
    out = {}
    for node in tree.body:
        if not isinstance(node, ast.ClassDef):
            continue
        for st in node.body:
            if isinstance(st, ast.AnnAssign) and isinstance(st.target, ast.Name):
                name = st.target.id
                if not name.startswith("_") and name not in ("path", "raw"):
                    out.setdefault(name, node.name)
    return out


def _code_blob():
    """除 settings.py 之外的全部代码（含看板前端 —— 它也直接读字段名）。"""
    parts = []
    for d in SCAN_DIRS:
        for r, _dirs, files in os.walk(root(d)):
            if "__pycache__" in r:
                continue
            for f in files:
                if not (f.endswith(".py") or f.endswith(".html")):
                    continue
                p = os.path.join(r, f)
                if os.path.normcase(p) == SKIP_FILE:
                    continue
                parts.append(io.open(p, encoding="utf-8", errors="replace").read())
    return "\n".join(parts)


class TestConfigCoverage(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.fields = _declared_fields()
        cls.blob = _code_blob()
        cls.unused = {n for n in cls.fields
                      if not re.search(r"\b" + re.escape(n) + r"\b", cls.blob)}

    def test_no_unregistered_dead_config(self):
        extra = sorted(self.unused - set(KNOWN_INERT))
        self.assertFalse(extra, (
            "这些配置字段在代码里找不到任何引用 —— 现场调它们不会有任何效果。\n"
            "要么去用它，要么在 KNOWN_INERT 里登记并写明原因：\n  "
            + "\n  ".join(f"{n}（{self.fields[n]}）" for n in extra)))

    def test_registry_is_not_stale(self):
        stale = sorted(set(KNOWN_INERT) - self.unused)
        self.assertFalse(stale, (
            "这些字段已经被用起来了，但还挂在 KNOWN_INERT 里 —— 把它们删掉：\n  "
            + "\n  ".join(stale)))

    def test_every_registration_has_a_reason(self):
        for name, why in KNOWN_INERT.items():
            self.assertGreaterEqual(
                len(why), 10, f"{name} 的登记理由太短，写清楚为什么它不生效")

    def test_sanity_the_scanner_actually_finds_used_fields(self):
        """自检扫描口径：这几个一定在用的字段必须被认出来，否则清单没意义。"""
        for name in ("stability_delta", "vehicle_threshold", "frame_mode",
                     "settle_sec", "state_push_hz", "range_mode"):
            self.assertIn(name, self.fields, f"{name} 应当是个配置字段")
            self.assertNotIn(name, self.unused, f"{name} 明明在用，却被判成没人用")


class TestSimulatorFactoryWiring(unittest.TestCase):
    def test_factory_keeps_dense_full_rotation_and_noise(self):
        """仿真源不能用构造器默认值覆盖工厂选定的射线步长。"""
        from app.radar_source import build_source

        _, channel, calib, _, angle_cache = H.ctx()
        source = build_source(channel, calib, angle_cache, mode="simulate")
        self.addCleanup(source.close)
        self.assertEqual(source.sim.az_step_deg, .2)
        self.assertEqual(source.packets_per_frame, 1800)
        self.assertAlmostEqual(source.sim.az_step_deg * source.packets_per_frame, 360.)
        self.assertEqual(source.interval, calib.detection.detect_interval)
        self.assertEqual(source.sim.range_noise_m, .004)
        self.assertEqual(source.sim.dropout_prob, .01)


if __name__ == "__main__":
    unittest.main()
