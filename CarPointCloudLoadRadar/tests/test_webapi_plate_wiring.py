# -*- coding: utf-8 -*-
"""车牌接通：与后台 CarPointCloudWebApi 的跨实现契约。

车牌不走相机，走后台 —— 我们 POST /api/v1/load-radar-agent/passages/prepare，
车牌是它的**返回值**。回执里的两个摘要（passageKey / workflowId）由
`load_radar_bridge._prepare_channel` **本地重算**再比对，对不上就抛
「到车回执权威身份不一致」，那一车直接测不出来。

所以这两边的哈希必须逐位一致：
    后台  LoadRadarService.Key(params object[] values)
          = sha256(JsonSerializer.Serialize(values)) 的小写十六进制
    我们  sha256(json.dumps(values, separators=(",", ":")))

下面的期望值**不是我们自己算完抄回来的**，是 2026-09-22 用后台真实程序集
（CarPointCloudWebApi.dll，net8.0）跑 `LoadRadarService.Key` 打出来的。
改我们这边的摘要写法时，这几条会先红。
"""
from __future__ import annotations

import hashlib
import io
import json
import os
import unittest

from app.load_radar_transport import BridgeConfig

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEBAPI = os.path.join(ROOT, "config", "webapi.json")
CHANNELS = os.path.join(ROOT, "config", "channels.json")

# 用后台程序集实测得到的 (通道, passageKey, workflowId)，node_id = jt128-01，
# passageId = passage-20260922-0001。
CSHARP_VECTORS = (
    ("CH2", "dc9e50969ce2d1ad0bcd8c2498946bdbe5354d99d4e991615decdaafd85a8d5c",
            "729f482dd75168f3e2432397fea9e24801cc40b765c499a1fd5ff45508466d22"),
    ("CH3", "3c6472a668edaa2aef86dc6b9b22d3968390471fc27e9e32a3ba3d04f8af6ee2",
            "359920ff325e93d29196e7afa3b44b66a0d4e581ee9c9ec8468b86d7053a9157"),
    ("CH4", "d3e0ff41c232640d6874ebddf22fed0be49388c2e0ba5c83566456fef44c7402",
            "3695e3c2e57a7f2c04b8de101c2f18aa51ec52b4ba96552429e4b9448da5bc06"),
)


def _digest(values):
    """与 load_radar_bridge._prepare_channel 里重算摘要的写法保持一致。"""
    return hashlib.sha256(json.dumps(values, separators=(",", ":")).encode("utf-8")).hexdigest()


class TestPassageDigestMatchesBackend(unittest.TestCase):
    """两边算 passageKey / workflowId 必须逐位一致。"""

    def test_vectors_from_the_real_backend_assembly(self):
        for channel, key, workflow in CSHARP_VECTORS:
            with self.subTest(channel=channel):
                mine = _digest(["jt128-01", channel, "passage-20260922-0001"])
                self.assertEqual(mine, key, "passageKey 与后台不一致")
                self.assertEqual(_digest(["load-radar-arrival", mine]), workflow,
                                 "workflowId 与后台不一致")

    def test_separator_style_is_what_the_backend_produces(self):
        """带空格的默认 json.dumps 会算出完全不同的摘要 —— 钉住这个坑。"""
        loose = hashlib.sha256(
            json.dumps(["jt128-01", "CH2", "passage-20260922-0001"]).encode("utf-8")).hexdigest()
        self.assertNotEqual(loose, CSHARP_VECTORS[0][1])


class TestWebapiConfig(unittest.TestCase):
    """现场那份 config/webapi.json 必须能用，且覆盖所有启用的雷达通道。"""

    def setUp(self):
        if not os.path.isfile(WEBAPI):
            self.skipTest("本机没有 config/webapi.json（尚未接后台）")
        self.cfg = BridgeConfig.load(WEBAPI)

    def test_loads_and_key_is_strong(self):
        self.assertTrue(32 <= len(self.cfg.agent_key) <= 4096)
        self.assertNotIn("REPLACE", self.cfg.agent_key.upper(),
                         "agent_key 还是模板占位符")

    def test_channel_set_equals_the_enabled_radar_channels(self):
        """★ 要求的是**完全相等**，不是覆盖。

        `service.LidarService.__init__` 里写死了
        `set(bridge_config.channels) != {c.id for c in chans}` 就抛
        「WebApi 接口通道必须与本次启动通道完全一致」-> **服务启动即失败**。
        所以多写一个停用的通道（比如返厂维修的 CH1）也是错的，
        不是"留着以后用"。CH1 接回来时，这两份配置必须同时改。
        """
        with io.open(CHANNELS, encoding="utf-8") as stream:
            raw = json.load(stream)
        enabled = {c["id"] for c in raw["channels"] if c.get("enabled")}
        self.assertEqual(set(self.cfg.channels), enabled,
                         "接口通道必须与 channels.json 里启用的通道完全一致")

    def test_outbox_is_outside_the_sample_output_dir(self):
        """发件箱装的是还没发出去的测量；放进 output_dir 会被清理任务扫到。"""
        with io.open(CHANNELS, encoding="utf-8") as stream:
            raw = json.load(stream)
        outbox = os.path.normcase(os.path.abspath(str(self.cfg.storage_dir)))
        for ch in raw["channels"]:
            out = os.path.normcase(os.path.abspath(os.path.join(ROOT, ch["output_dir"])))
            self.assertFalse(outbox == out or outbox.startswith(out + os.sep),
                             "发件箱不能放在通道落盘目录 %s 下面" % ch["output_dir"])


if __name__ == "__main__":
    unittest.main()
