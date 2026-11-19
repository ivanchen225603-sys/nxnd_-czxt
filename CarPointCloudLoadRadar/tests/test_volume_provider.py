"""显式声明模型的严格目录、不可变匹配及真实积分；数值均为合成测试条件。"""
import contextlib
import copy
from dataclasses import FrozenInstanceError, replace
import io
import json
import os
from pathlib import Path
import tempfile
import time
import unittest

import numpy as np

from app.bed_reference import BedReferenceScope, baseline_digest
from app.detect import compute_volume
from app.volume_provider import MAX_PROVIDER_BYTES, VolumeCatalog
from tests.test_volume_validity import scene


def model_for(base):
    return dict(model_id="test-model", binding=dict(node_id="test-node", channel_id="CH1", calibration_id="cal-1",
        configuration_sha256="a" * 64, baseline_version_id="version-1", baseline_asset_sha256="b" * 64,
        baseline_sha256=baseline_digest(base)), reference_id="test-survey",
        coordinate_frame="baseline-heading-aligned-m-v1", valid_from=999., valid_until=1010.,
        domain_xy=[0., 2.4, 0., 10.], domain_source="synthetic_model", max_height_m=3.,
        height_source="synthetic_model", support_cell_m=.1, support_origin_xy=[0., 0.],
        support_model="cell_mean_max_relative_height", error_source="synthetic_model", surface_error_m=0.,
        bed_error_m=0., roi_error_area_m2=0., conditions_status="declared_model",
        registration=dict(mode="fixed", dy_m=0.))


def catalog_for(models):
    return VolumeCatalog.from_bytes(json.dumps(dict(schema="volume-provider-v1", models=models),
                                              ensure_ascii=False).encode("utf-8"))


def runtime_scope(**kwargs):
    scope = BedReferenceScope("version-1", "cal-1", "a" * 64, "CH1", "passage-1", "process-1:0",
                              ((1, 1, 1000.),), "baseline-heading-aligned-m-v1", 1000., ((0,),))
    return replace(scope, **kwargs)


class TestVolumeProvider(unittest.TestCase):
    def setUp(self):
        self.base, self.loaded = scene(.4)
        self.value = model_for(self.base)

    def test_declared_model_supplies_real_calculation_not_expected_answer(self):
        catalog = catalog_for([self.value])
        model, status = catalog.select(self.value["binding"], 1000.)
        self.assertEqual("matched", status)
        context = model.supply(scope=runtime_scope(), baseline=self.base, surfaces=[self.loaded.cloud],
                               dy=0., registration_status="declared_fixed")
        result = compute_volume(self.base, self.loaded, dy=0., volume_context=context, volume_scope=runtime_scope())
        self.assertEqual("conditional", result.range_status)
        self.assertAlmostEqual(9.6, result.volume_m3)
        self.assertAlmostEqual(9.6, result.bound_lo)
        self.assertAlmostEqual(9.6, result.bound_hi)

    def test_nonzero_registration_translates_domain_and_origin_once(self):
        self.value["registration"]["dy_m"] = 1.25
        self.loaded.cloud[:, 1] += 1.25
        model, _ = catalog_for([self.value]).select(self.value["binding"], 1000.)
        context = model.supply(scope=runtime_scope(), baseline=self.base, surfaces=[self.loaded.cloud],
                               dy=1.25, registration_status="declared_fixed")
        self.assertEqual((0., 2.4, 1.25, 11.25), context.domain_xy)
        self.assertEqual((0., 1.25), context.support_origin_xy)
        result = compute_volume(self.base, self.loaded, dy=1.25, volume_context=context, volume_scope=runtime_scope())
        self.assertAlmostEqual(9.6, result.volume_m3)
        self.assertEqual("conditional", result.range_status)

    def test_every_binding_field_and_expiry_are_exact(self):
        catalog = catalog_for([self.value])
        for key in self.value["binding"]:
            binding = dict(self.value["binding"])
            binding[key] = "c" * 64 if key.endswith("sha256") else "other"
            self.assertEqual((None, "binding_not_found"), catalog.select(binding, 1000.))
        self.assertEqual((None, "expired"), catalog.select(self.value["binding"], 1011.))

    def test_catalog_and_frozen_model_independent_of_file_and_caller(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "models.json"
            path.write_text(json.dumps(dict(schema="volume-provider-v1", models=[self.value])), encoding="utf-8")
            catalog = VolumeCatalog.load(str(path))
            path.write_text("{}", encoding="utf-8")
        model, _ = catalog.select(self.value["binding"], 1000.)
        digest = model.sha256
        altered = model.data()
        altered["max_height_m"] = 100
        self.assertEqual(digest, model.sha256)
        self.assertEqual(3., model.data()["max_height_m"])
        with self.assertRaises(FrozenInstanceError):
            model.canonical_json = b"{}"

    def test_overlapping_models_rejected_including_touching_endpoints(self):
        other = copy.deepcopy(self.value)
        other["model_id"] = "second"
        other["valid_from"] = 1010.
        with self.assertRaisesRegex(ValueError, "重叠"):
            catalog_for([self.value, other])
        other["valid_from"] = 1011.
        other["valid_until"] = 1020.
        catalog = catalog_for([self.value, other])
        model, _ = catalog.select(self.value["binding"], 1011.)
        self.assertEqual("second", model.data()["model_id"])

    def test_unknown_duplicate_missing_and_nonfinite_json_rejected(self):
        for data in (b'{"schema":"volume-provider-v1","schema":"volume-provider-v1","models":[]}',
                     b'{"schema":"volume-provider-v1","models":[],"extra":0}',
                     b'{"schema":"volume-provider-v1","models":NaN}', b'{}',
                     b'\xef\xbb\xbf{}', b'\x00{}', b' ' * (MAX_PROVIDER_BYTES + 1),
                     b'{"a":' + b'[' * 17 + b'0' + b']' * 17 + b'}'):
            with self.subTest(data=data[:50]), self.assertRaises(ValueError):
                VolumeCatalog.from_bytes(data)
        altered = copy.deepcopy(self.value)
        del altered["bed_error_m"]
        with self.assertRaises(ValueError):
            catalog_for([altered])

    def test_bad_geometry_errors_sources_and_registration_rejected(self):
        changes = (dict(domain_xy=[0, 0, 0, 10]), dict(domain_xy=[0, True, 0, 10]),
                   dict(support_cell_m=1e-320), dict(max_height_m=float("inf")), dict(surface_error_m=-1),
                   dict(bed_error_m=True), dict(height_source="observed_max"), dict(domain_source="cloud_roi"),
                   dict(conditions_status="externally_supplied"), dict(registration=dict(mode="auto_guess")),
                   dict(registration=dict(mode="fixed", dy_m=True)), dict(valid_until=998.),
                   dict(support_origin_xy=[0, "1"]), dict(unexpected=1))
        for change in changes:
            with self.subTest(change=change), self.assertRaises(ValueError):
                catalog_for([{**self.value, **change}])

    def test_no_implicit_registration_success_or_scope_rebinding(self):
        model, _ = catalog_for([self.value]).select(self.value["binding"], 1000.)
        for dy, status in ((None, "declared_fixed"), (True, "declared_fixed"), (.5, "declared_fixed"),
                           (0., "unresolved"), (0., "resolved")):
            with self.subTest(dy=dy, status=status), self.assertRaises(ValueError):
                model.supply(scope=runtime_scope(), baseline=self.base, surfaces=[self.loaded.cloud],
                             dy=dy, registration_status=status)
        for scope in (runtime_scope(channel_id="CH2"), runtime_scope(baseline_version_id="other"),
                      runtime_scope(configuration_sha256="f" * 64), runtime_scope(coordinate_frame="wrong")):
            with self.assertRaises(ValueError):
                model.supply(scope=scope, baseline=self.base, surfaces=[self.loaded.cloud],
                             dy=0., registration_status="declared_fixed")
        self.value["registration"] = dict(mode="estimated")
        model, _ = catalog_for([self.value]).select(self.value["binding"], 1000.)
        with self.assertRaises(ValueError):
            model.supply(scope=runtime_scope(), baseline=self.base, surfaces=[self.loaded.cloud], dy=0., registration_status="unresolved")
        model.supply(scope=runtime_scope(), baseline=self.base, surfaces=[self.loaded.cloud], dy=0., registration_status="resolved")

    def test_startup_rejects_unrepresentable_geometry_and_derived_budget(self):
        changes = (dict(support_origin_xy=[1e300, 0]),
            dict(domain_xy=[0, 1, 0, 1], max_height_m=1e308, roi_error_area_m2=1),
            dict(registration=dict(mode="fixed", dy_m=1e300)),
            dict(domain_xy=[0, 1e-200, 0, 1e-200], support_cell_m=1e-201),
            dict(domain_xy=[0, 1e-160, 0, 1e-160], support_cell_m=1e-162),
            dict(domain_xy=[0, 1, .5, 1.5], max_height_m=1e308, support_cell_m=32,
                 registration=dict(mode="fixed", dy_m=2 ** 53)))
        for change in changes:
            with self.subTest(change=change), self.assertRaises(ValueError):
                catalog_for([{**self.value, **change}])

    def test_raw_json_underflow_never_silently_erases_error_budget(self):
        source = json.dumps(dict(schema="volume-provider-v1", models=[self.value])).encode("utf-8")
        for number in (b"-1e-400", b"-1e-324", b"1e-400", b"1e-324", b"1e400"):
            with self.subTest(number=number), self.assertRaises(ValueError):
                VolumeCatalog.from_bytes(source.replace(b'"surface_error_m": 0.0', b'"surface_error_m": ' + number))
        VolumeCatalog.from_bytes(source.replace(b'"surface_error_m": 0.0', b'"surface_error_m": -0e-400'))

    def test_changed_baseline_and_partial_coverage_do_not_fake_complete(self):
        model, _ = catalog_for([self.value]).select(self.value["binding"], 1000.)
        self.loaded.cloud = self.loaded.cloud[self.loaded.cloud[:, 1] >= 2.]
        context = model.supply(scope=runtime_scope(), baseline=self.base, surfaces=[self.loaded.cloud],
                               dy=0., registration_status="declared_fixed")
        result = compute_volume(self.base, self.loaded, dy=0., volume_context=context, volume_scope=runtime_scope())
        self.assertEqual("conditional", result.range_status)
        self.assertFalse(result.confidence()[0])
        self.assertGreater(result.unknown_area_m2, 0)
        self.assertLessEqual(result.bound_lo, 9.6)
        self.assertGreaterEqual(result.bound_hi, 9.6)
        self.base.cloud[0, 2] += .01
        with self.assertRaises(ValueError):
            model.supply(scope=runtime_scope(), baseline=self.base, surfaces=[self.loaded.cloud],
                         dy=0., registration_status="declared_fixed")


class TestMakeVolumeModelTool(unittest.TestCase):
    """★ 没有这个工具，整条体积链根本跑不起来（2026-09-18）。

    `volume_context` 要求调用方显式声明内腔、货高上限和误差预算，并用
    sha256 绑死在这一份基线上 —— 设计是对的。但在此之前**没有任何办法
    把那份文件造出来**：7 个 sha256 绑定 + domain_xy + support_origin_xy
    + 三项误差预算，现场没人能手写对。于是 `volume_provider_config`
    永远是空的、`range_status` 永远是 `missing`、每次测量都被标低置信，
    **煤量永远进不了正式入库**。
    """

    def setUp(self):
        import sys as _sys
        from app.settings import root as _root
        tools = os.path.join(_root(), "tools")
        if tools not in _sys.path:
            _sys.path.insert(0, tools)
            self.addCleanup(_sys.path.remove, tools)
        import make_volume_model
        self.mod = make_volume_model
        self.tmp = tempfile.TemporaryDirectory(prefix="volume-model-")
        self.addCleanup(self.tmp.cleanup)

    def _baseline(self, path):
        """造一辆合成空车：底板 + 两侧厢壁，宽度落在 carriage_box 的窗口里。"""
        from app.detect import EmptyBaseline
        rng = np.random.default_rng(7)
        xs = rng.uniform(-1.20, 1.20, 60000)
        ys = rng.uniform(-5.0, 5.0, 60000)
        bed = np.column_stack((xs, ys, np.full(60000, -3.7) + 0.02 * ys))
        wy = rng.uniform(-5.0, 5.0, 20000)
        wx = np.where(rng.random(20000) < .5, -1.24, 1.24)
        wall = np.column_stack((wx, wy, rng.uniform(-3.7, -1.9, 20000)))
        cloud = np.concatenate((bed, wall)).astype(np.float32)
        base = EmptyBaseline(floor_z=-3.7,
                             box=(-1.30, 1.30, -5.0, 5.0, -3.7, -1.9),
                             cloud=cloud, n=len(cloud), created="t",
                             bed_plane=(0.0, -0.02, 1.0, 3.7))
        base.save(path)
        return base

    def _argv(self, npz, out, **over):
        args = {"--baseline": npz, "--out": out, "--node-id": "node-1", "--channel": "CH1",
                "--calibration-id": "cal-1", "--configuration-sha256": "a" * 64,
                "--baseline-version-id": "bv-1", "--baseline-asset-sha256": "b" * 64,
                "--surface-error": "0.05", "--bed-error": "0.03", "--roi-error-area": "0.5"}
        args.update(over)
        argv = []
        for key, value in args.items():
            argv += [key, str(value)]
        return argv

    def _run(self, argv):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = self.mod.main(argv)
        return code, buf.getvalue()

    def _second_vehicle(self, path):
        """另一辆车：车厢更长，基线摘要与 _baseline 不同。"""
        from app.detect import EmptyBaseline
        rng = np.random.default_rng(11)
        xs = rng.uniform(-1.20, 1.20, 60000)
        ys = rng.uniform(-6.0, 6.0, 60000)
        bed = np.column_stack((xs, ys, np.full(60000, -3.7) + 0.02 * ys))
        wy = rng.uniform(-6.0, 6.0, 20000)
        wx = np.where(rng.random(20000) < .5, -1.24, 1.24)
        wall = np.column_stack((wx, wy, rng.uniform(-3.7, -1.9, 20000)))
        cloud = np.concatenate((bed, wall)).astype(np.float32)
        base = EmptyBaseline(floor_z=-3.7, box=(-1.30, 1.30, -6.0, 6.0, -3.7, -1.9),
                             cloud=cloud, n=len(cloud), created="t2",
                             bed_plane=(0.0, -0.02, 1.0, 3.7))
        base.save(path)
        return base

    def _models(self, out):
        with open(out, encoding="utf-8") as handle:
            return json.load(handle)["models"]

    def test_runs_merge_instead_of_overwriting(self):
        """★★ 同一辆车为多条装货车道各跑一次，目录里要有多份，而不是只剩最后一份。

        体积按**装货车辆实际走的车道**算（2026-09-23 定案），binding 绑车道，
        而装货车道四选一随机 -> 一辆车要为每条可能装货的道各备一份模型。
        原来每跑一次就把 --out 整份覆盖成单个模型，结果任何时刻最多只有
        一个"车辆 x 车道"组合拿得到范围，其余静默走 binding_not_found。
        """
        npz = os.path.join(self.tmp.name, "a.npz")
        out = os.path.join(self.tmp.name, "vp.json")
        self._baseline(npz)
        for ch in ("CH2", "CH4"):
            code, text = self._run(self._argv(npz, out, **{"--channel": ch,
                                                            "--calibration-id": ch + "-cal"}))
            self.assertEqual(0, code, text)
        self.assertEqual(["CH2", "CH4"], sorted(m["binding"]["channel_id"] for m in self._models(out)),
                         "第二次运行把第一次的模型覆盖掉了")

    def test_another_vehicle_does_not_wipe_the_first(self):
        """基线按车牌存 -> 目录里有多辆车。给 B 车生成不许抹掉 A 车的。"""
        from app.bed_reference import baseline_digest
        a_npz, b_npz = (os.path.join(self.tmp.name, n) for n in ("a.npz", "b.npz"))
        out = os.path.join(self.tmp.name, "vp.json")
        a, b = self._baseline(a_npz), self._second_vehicle(b_npz)
        self.assertNotEqual(baseline_digest(a), baseline_digest(b), "前提：两辆车的基线得不同")
        self.assertEqual(0, self._run(self._argv(a_npz, out))[0])
        self.assertEqual(0, self._run(self._argv(b_npz, out, **{"--baseline-version-id": "bv-2"}))[0])
        digests = {m["binding"]["baseline_sha256"] for m in self._models(out)}
        self.assertEqual({baseline_digest(a), baseline_digest(b)}, digests, "A 车的模型被 B 车抹掉了")
        ids = [m["model_id"] for m in self._models(out)]
        self.assertEqual(len(ids), len(set(ids)), "同一条道上两辆车撞了 model_id")

    def test_rerunning_the_same_binding_replaces_it(self):
        """重跑同一个"车辆 x 车道"只替换，不追加 —— 否则有效期重叠整份文件被拒。"""
        npz = os.path.join(self.tmp.name, "a.npz")
        out = os.path.join(self.tmp.name, "vp.json")
        self._baseline(npz)
        for _ in range(3):
            self.assertEqual(0, self._run(self._argv(npz, out))[0])
        self.assertEqual(1, len(self._models(out)), "同一绑定重跑出了重复模型")

    def test_each_lane_model_is_what_runtime_selects(self):
        """★ 运行时按 binding 挑：同一辆车在哪条道装货，就命中那条道的那份。"""
        import time as _time
        from app.volume_provider import VolumeCatalog
        npz = os.path.join(self.tmp.name, "a.npz")
        out = os.path.join(self.tmp.name, "vp.json")
        self._baseline(npz)
        for ch in ("CH2", "CH3", "CH4"):
            self._run(self._argv(npz, out, **{"--channel": ch, "--calibration-id": ch + "-cal"}))
        catalog = VolumeCatalog.load(out)
        for model in self._models(out):
            with self.subTest(channel=model["binding"]["channel_id"]):
                chosen, status = catalog.select(model["binding"], _time.time())
                self.assertEqual("matched", status)
                self.assertEqual(model["binding"]["channel_id"],
                                 chosen.data()["binding"]["channel_id"])

    def test_generated_file_loads_and_binds_to_the_actual_baseline(self):
        """★ 目标：生成出来的文件必须能过 VolumeCatalog 校验并选得中。"""
        from app.bed_reference import baseline_digest
        from app.volume_provider import VolumeCatalog
        npz = os.path.join(self.tmp.name, "b.npz")
        out = os.path.join(self.tmp.name, "vp.json")
        base = self._baseline(npz)
        code, text = self._run(self._argv(npz, out))
        self.assertEqual(0, code, text)
        catalog = VolumeCatalog.load(out)
        binding = dict(node_id="node-1", channel_id="CH1", calibration_id="cal-1",
                       configuration_sha256="a" * 64, baseline_version_id="bv-1",
                       baseline_asset_sha256="b" * 64, baseline_sha256=baseline_digest(base))
        model, status = catalog.select(binding, time.time())
        self.assertEqual("matched", status)
        self.assertIsNotNone(model)

    def test_domain_is_measured_from_the_cloud_not_the_bounding_box(self):
        """内腔域必须来自 carriage_box（按截面宽度切），不是点云包围盒。"""
        from app.detect import carriage_box
        from app.volume_provider import VolumeCatalog
        npz = os.path.join(self.tmp.name, "b.npz")
        out = os.path.join(self.tmp.name, "vp.json")
        base = self._baseline(npz)
        self._run(self._argv(npz, out))
        model = VolumeCatalog.load(out).models[0].data()
        box = carriage_box(np.asarray(base.cloud, np.float64))
        self.assertIsNotNone(box, "合成空车切不出车厢，用例本身有问题")
        for got, want in zip(model["domain_xy"], (box["x0"], box["x1"], box["y0"], box["y1"])):
            self.assertAlmostEqual(got, want, places=6)

    def test_rebuilt_baseline_invalidates_the_model(self):
        """★ 重新标定之后旧模型必须失配 —— 拿别的车的内腔算体积是致命的。"""
        from app.bed_reference import baseline_digest
        from app.volume_provider import VolumeCatalog
        npz = os.path.join(self.tmp.name, "b.npz")
        out = os.path.join(self.tmp.name, "vp.json")
        self._baseline(npz)
        self._run(self._argv(npz, out))
        catalog = VolumeCatalog.load(out)
        other = self._baseline(os.path.join(self.tmp.name, "b2.npz"))
        other.cloud[:, 2] += 0.13                       # 内容变了
        binding = dict(node_id="node-1", channel_id="CH1", calibration_id="cal-1",
                       configuration_sha256="a" * 64, baseline_version_id="bv-1",
                       baseline_asset_sha256="b" * 64, baseline_sha256=baseline_digest(other))
        model, status = catalog.select(binding, time.time())
        self.assertIsNone(model)
        self.assertEqual("binding_not_found", status)

    def test_rejects_bad_identifiers_and_budgets(self):
        npz = os.path.join(self.tmp.name, "b.npz")
        self._baseline(npz)
        for over in ({"--configuration-sha256": "短"}, {"--baseline-asset-sha256": "z" * 64},
                     {"--surface-error": "-0.1"}, {"--support-cell": "0"}):
            code, _ = self._run(self._argv(npz, "", **over))
            self.assertEqual(2, code, over)

    def test_conditions_status_is_always_declared_not_certified(self):
        """声明 ≠ 现场计量认证。哪怕带 --surveyed 也只能是 declared_model。"""
        from app.volume_provider import VolumeCatalog
        npz = os.path.join(self.tmp.name, "b.npz")
        out = os.path.join(self.tmp.name, "vp.json")
        self._baseline(npz)
        self._run(self._argv(npz, out) + ["--surveyed"])
        self.assertEqual("declared_model",
                         VolumeCatalog.load(out).models[0].data()["conditions_status"])

    def test_refuses_a_baseline_without_a_carriage(self):
        """切不出车厢就报错，不许编一个域出来。"""
        from app.detect import EmptyBaseline
        rng = np.random.default_rng(3)
        flat = np.column_stack((rng.uniform(-6, 6, 5000), rng.uniform(-6, 6, 5000),
                                np.full(5000, -5.2))).astype(np.float32)
        npz = os.path.join(self.tmp.name, "flat.npz")
        EmptyBaseline(floor_z=-5.2, box=(-6, 6, -6, 6, -5.2, -5.0), cloud=flat,
                      n=len(flat), created="t").save(npz)
        with self.assertRaises(SystemExit):
            self._run(self._argv(npz, ""))
