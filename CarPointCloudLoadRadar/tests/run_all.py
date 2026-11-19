#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tests/run_all.py — 跑全部单元测试（不依赖 pytest）

    python3 tests/run_all.py               # 全部
    python3 tests/run_all.py -v            # 详细
    python3 tests/run_all.py -k packet     # 只跑名字含 packet 的模块

装了 pytest 也可以直接：pytest tests/ -v
"""
from __future__ import annotations

import argparse
import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

MODULES = [
    "tests.test_architecture_boundaries",
    "tests.test_settings",
    "tests.test_pointcloud",
    "tests.test_geometry",
    "tests.test_packet",
    "tests.test_batch_decode",
    "tests.test_detect",
    "tests.test_volume_relative_height",
    "tests.test_volume_boundary_area",
    "tests.test_standstill",
    "tests.test_horizontal_standstill",
    "tests.test_standstill_field",
    "tests.test_frame_integrity",
    "tests.test_standstill_delivery",
    "tests.test_control_boundary",
    "tests.test_cloud_archive",
    "tests.test_storage_maintenance",
    "tests.test_cloud_roi",
    "tests.test_cloud_observation",
    "tests.test_archive_inputs",
    "tests.test_archive_capacity",
    "tests.test_windows_junction_boundaries",
    "tests.test_result_finalizer",
    "tests.test_async_plate_contract",
    "tests.test_pending_measurements",
    "tests.test_frozen_detection",
    "tests.test_frozen_preparation",
    "tests.test_deferred_measurements",
    "tests.test_deferred_handoff",
    "tests.test_day_recalculation",
    "tests.test_deferred_service",
    "tests.test_stage_timing",
    "tests.test_position_standstill",
    "tests.test_sample_paths",
    "tests.test_plate",
    "tests.test_hub",
    "tests.test_hr40_protocol",
    "tests.test_demo_director",
    "tests.test_demo_api",
    "tests.test_service",
    "tests.test_runtime_events",
    "tests.test_enter_validation",
    "tests.test_exit_qualification",
    "tests.test_load_radar_transport",
    "tests.test_load_radar_contract",
    "tests.test_load_radar_bridge",
    "tests.test_ch4_shared_baseline",
    "tests.test_load_radar_preparation",
    "tests.test_webapi_plate_wiring",
    "tests.test_plate_webapi",
    "tests.test_baseline_constant",
    "tests.test_baseline_maintenance",
    "tests.test_field_fixes",
    "tests.test_frame_sync",
    "tests.test_live_pipeline",
    "tests.test_cli_entrypoints",
    "tests.test_verify",
    "tests.test_dashboard_contract",
    "tests.test_web_json",
    "tests.test_cleanup",
    "tests.test_config_coverage",
    "tests.test_config_validation",
    "tests.test_lane_setup",
    "tests.test_configure_radars",
    "tests.test_net_diag",
    "tests.test_roi_audit",
    "tests.test_export_ply",
    "tests.test_volume_calc",
    "tests.test_volume_validity",
    "tests.test_volume_context",
    "tests.test_volume_inputs",
    "tests.test_volume_provider",
    "tests.test_volume_runtime",
    "tests.test_volume_replay",
    "tests.test_cross_check_joint",
    "tests.test_bed_reference",
    "tests.test_watchdog",
    "tests.test_empty_audit",
    "tests.test_volume_error_fix",
    "tests.test_pose_component_fix",
    "tests.test_enter_occlusion_volume",
    "tests.test_dual_radar_capture",
    "tests.test_encoding",
]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="run_all.py", description="跑全部单元测试")
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("-k", "--filter", default="", help="只跑名字含该串的测试模块")
    ap.add_argument("-f", "--failfast", action="store_true", help="首个失败就停")
    args = ap.parse_args(argv)

    mods = [m for m in MODULES if not args.filter or args.filter in m]
    if not mods:
        print(f"没有匹配 {args.filter!r} 的测试模块")
        return 1

    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    for m in mods:
        try:
            suite.addTests(loader.loadTestsFromName(m))
        except Exception as e:                              # noqa: BLE001
            print(f"▲  无法加载 {m}: {type(e).__name__}: {e}")
            return 1

    print(f"共 {suite.countTestCases()} 个用例，来自 {len(mods)} 个模块\n")
    runner = unittest.TextTestRunner(verbosity=2 if args.verbose else 1,
                                     failfast=args.failfast)
    res = runner.run(suite)
    return 0 if res.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
