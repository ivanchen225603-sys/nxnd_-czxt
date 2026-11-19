# -*- coding: utf-8 -*-
"""停车到真实体积及本地持久结果的单一 10 秒预算，不证明后台煤量发布。

观测用显式递增 seq/ts 和 10 Hz 受控时钟，实际计算与 SQLite 提交不替换。
预算为观测跨度加 perf_counter 实际耗时；后者从夹具准备前计起，连初帧、
基线准备、计算、轮询和独立连接读回成本也保守计入，进入测量后不重开时限。
解析平面场景的体积为 9.6 m^3，空车判定及机器 volumeStatus 保留真实输出；
这些合成条件只验证本地数值和持久交付，不证明实车准确率或后台发布成功。
"""
import json
import sqlite3
import threading
import time
import unittest
from contextlib import closing

import numpy as np

from app.pointcloud import Frame
from tests import test_volume_runtime as runtime_fixture


class TestStandstillDelivery(unittest.TestCase):
    # 只复用夹具方法，避免继承并重复执行整个体积测试类。
    setUp = runtime_fixture.TestVolumeRuntime.setUp
    close = runtime_fixture.TestVolumeRuntime.close
    bridge = runtime_fixture.TestVolumeRuntime.bridge
    worker = runtime_fixture.TestVolumeRuntime.worker
    feed = runtime_fixture.TestVolumeRuntime.feed
    ground = staticmethod(runtime_fixture.TestVolumeRuntime.ground)
    surfaces = staticmethod(runtime_fixture.TestVolumeRuntime.surfaces)
    arm = runtime_fixture.TestVolumeRuntime.arm

    def _deliver(self, jump_frame: int = 0) -> None:
        started = time.perf_counter()
        worker, bridge, baseline, frame, _ = self.arm(settle=3.0, original_asset=True)
        stopped_at = self.clock.mono       # arm 的最后一帧是首个车辆观测。
        self.assertEqual(3.0, worker.settle_sec)
        self.assertGreater(baseline.n, 0)

        def consumed() -> float:
            return self.clock.mono - stopped_at + time.perf_counter() - started

        def budget_message() -> str:
            return "停车后未在 10 秒内得到本地持久体积: " + repr({
                "observation_sec": self.clock.mono - stopped_at,
                "processing_sec": time.perf_counter() - started,
                "detected": worker._detected_this_session,
                "stability": worker.st.stability,
                "finalizer": worker._finalizer.status(worker.ch.id) if worker._finalizer else None,
            })

        step = 0
        while True:
            self.assertLess(consumed(), 10.0, budget_message())
            worker._drain_finalizer()
            if worker.history:
                result = worker.history[-1]
                measurement_id = result["measurement_id"]
                # 独立只读连接只能看到已提交数据，不能以 RAM 结果或同连接未提交行通过。
                uri = bridge.outbox.db_path.resolve().as_uri() + "?mode=ro"
                with closing(sqlite3.connect(uri, uri=True, timeout=0.1)) as persisted:
                    saved = persisted.execute(
                        "SELECT payload_json FROM outbox WHERE measurement_id=?",
                        (measurement_id,),
                    ).fetchone()
                self.assertIsNotNone(saved, "已展示结果尚未提交到临时 outbox")
                payload = json.loads(saved[0])
                elapsed = consumed()
                self.assertLessEqual(elapsed, 10.0, budget_message())
                self.assertEqual(measurement_id, payload["measurementId"])
                self.assertEqual("version-1", payload["baselineVersionId"])
                self.assertAlmostEqual(9.6, payload["volumeM3"], places=5)
                self.assertTrue(result["volume"]["ok"])
                self.assertFalse(result["volume"]["low_confidence"])
                self.assertEqual("conditional", result["volume"]["range_status"])
                self.assertEqual("supplied", result["volume_runtime"]["status"])
                self.assertAlmostEqual(9.6, result["volume"]["volume_m3"], places=5)
                self.assertEqual(1, worker._detected_this_session)
                if jump_frame:
                    self.assertGreaterEqual(step, jump_frame + 1,
                                            "必须实际经历抖动及原位恢复才允许测量")
                self.delivery_evidence = dict(
                    budget_sec=elapsed, observation_sec=self.clock.mono - stopped_at,
                    processing_sec=time.perf_counter() - started,
                    volume_status=payload["volumeStatus"], measurement_id=measurement_id,
                )
                return

            if worker._detected_this_session:
                # 与采样、真实计算共用剩余预算，绝不调用另给 10 秒的 await_result。
                threading.Event().wait(min(0.01, max(0.0, 10.0 - consumed())))
                continue

            step += 1
            # 停车 4.9 秒时只有一个 3 mm 横向共模跳点，下一帧恢复原位。
            # 最终主帧和额外货面仍与原基线/显式物理域匹配，不移动模型或改答案。
            current = (Frame(frame.xyz + np.array([0.003, 0.0, 0.0], np.float32))
                       if step == jump_frame else frame)
            self.feed(worker, current, 1)

    def test_stationary_vehicle_has_durable_volume_within_ten_seconds(self) -> None:
        self._deliver()

    def test_one_three_mm_jitter_at_4_9_seconds_still_delivers_within_budget(self) -> None:
        self._deliver(jump_frame=49)
