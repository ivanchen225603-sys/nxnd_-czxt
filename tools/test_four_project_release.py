"""四项目发布过滤、机密隔离和归档完整性回归，不运行业务服务。"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import struct
import tarfile
import tempfile
import unittest

from tools.build_four_project_release import ROOT, PROJECTS, archive, check_elf, copy_publish, prepare_compat_mountpoints, sanitize_template, source_allowed, templates, write


class 发布包回归(unittest.TestCase):
    def test_现场配置和过程文件不混入源码(self):
        for path in ("CarPointCloudWebApi/appsettings.json", "CarPointCloudLadarWorker/appsettings.json",
                     "CarPointCloudLoadRadar/config/webapi.json", "CarPointCloudLoadRadar/config/channels.json",
                     "CarPointCloudLoadRadar/AGENTS.md", "CarPointCloudLoadRadar/data/cloud.xyz"):
            self.assertFalse(source_allowed(path, True), path)

    def test_必要的新产品分文件会被纳入(self):
        for name in ("Repositories/IntegrationRepository.PartnerQueries.cs", "Services/PartnerQueryExecutor.cs",
                     "Services/IntegrationQueryService.PartnerQueries.cs"):
            self.assertTrue(source_allowed("CarPointCloudWebApi/" + name, False))
        self.assertTrue(source_allowed("CarPointCloudLoadRadar/cli/run.py", True))
        self.assertTrue(source_allowed("CarPointCloudLoadRadar/baseline/car.xyz", True))
        self.assertFalse(source_allowed("CarPointCloudLoadRadar/docs/local-report.md", False))
        for name in ("wwwroot/pointcloud-viewer/volume-query.html", "wwwroot/pointcloud-viewer/js/volume-query.js",
                     "wwwroot/pointcloud-viewer/css/volume-query.css", "wwwroot/pointcloud-viewer/css/page-shell.css",
                     "Sql/load_radar_volume_revision_20261102.sql",
                     "tests/CarPointCloudWebApi.Tests/Fixtures/load-radar-day-recalculation-v1.json"):
            self.assertTrue(source_allowed("CarPointCloudWebApi/" + name, False), name)
        self.assertFalse(source_allowed("CarPointCloudLoadRadar/p.out", False))

    def test_Worker发布模板开启三点日清并隔离永久目录(self):
        with tempfile.TemporaryDirectory() as folder:
            package = Path(folder)
            templates(ROOT, package, PROJECTS[0])
            cfg = json.loads((package / "config-templates/appsettings.example.json").read_text(encoding="utf-8"))
            opts = cfg["LadarCapture"]
            self.assertEqual("03:00", opts["CaptureCleanupDailyAt"])
            self.assertEqual(["/data/carpointcloud/empty-vehicle-profiles"], opts["PermanentPointCloudDirectories"])
            self.assertEqual("/data/carpointcloud/captures", opts["CaptureDirectory"])

    def test_递归清除凭据但保留业务开关(self):
        value = sanitize_template({"Auth": {"JwtSecret": "must-not-leak"}, "AgentKey": "secret",
            "Password": "private", "CapturePartnerQueryBodies": True, "root": "D:\\data\\field",
            "nodes": [{"RecorderToken": "private"}], "base_url": "http://127.0.0.1:5031"})
        self.assertNotIn("private", json.dumps(value))
        self.assertNotIn("must-not-leak", json.dumps(value))
        self.assertTrue(value["CapturePartnerQueryBodies"])
        self.assertEqual("", value["nodes"][0]["RecorderToken"])
        self.assertEqual("http://127.0.0.1:5031", value["base_url"])
        self.assertTrue(value["root"].startswith("/data/"))

    def test_非目标架构立即拒绝(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "app"
            path.write_bytes(b"\x7fELF\x02\x01" + bytes(12) + struct.pack("<H", 183))
            with self.assertRaises(ValueError):
                check_elf(path)

    def test_只读兼容根预建挂载点且不覆盖原文件(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory)
            write(runtime / "etc/passwd", "已有测试内容\n")
            prepare_compat_mountpoints(runtime)
            self.assertTrue((runtime / "data/carpointcloud").is_dir())
            self.assertTrue((runtime / "usr/share/zoneinfo").is_dir())
            self.assertTrue((runtime / "etc/localtime").is_file())
            self.assertEqual("已有测试内容\n", (runtime / "etc/passwd").read_text(encoding="utf-8"))

    def test_发布输出过滤保留网页且排除秘密和运行数据(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "raw"
            source.mkdir()
            (source / "Service").write_bytes(b"\x7fELF\x02\x01" + bytes(12) + struct.pack("<H", 62))
            for name in ("Service.dll", "Service.pdb", "appsettings.json", "appsettings.Production.json", "capture.pcd"):
                (source / name).write_text("fixture")
            write(source / "wwwroot/page.html", "page")
            write(source / "wwwroot/field.pcd", "private")
            copy_publish(source, root / "safe", "Service")
            self.assertEqual({p.relative_to(root / "safe").as_posix() for p in (root / "safe").rglob("*") if p.is_file()},
                             {"Service", "Service.dll", "wwwroot/page.html"})

    def test_归档校验覆盖全部文件且脚本和加载器可执行(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            package = root / "package"
            write(package / "run.sh", "#!/usr/bin/env bash\nexit 78\n")
            loader = package / "runtime/loader.so"
            loader.parent.mkdir()
            loader.write_bytes(b"\x7fELF" + bytes(16))
            write(package / "config-templates/example.json", "{}\n")
            result = archive(package, root)
            self.assertEqual(hashlib.sha256((root / result["file"]).read_bytes()).hexdigest(), result["sha256"])
            with tarfile.open(root / result["file"]) as bundle:
                manifest = bundle.extractfile("package/SHA256SUMS").read().decode()
                self.assertIn("  runtime/loader.so", manifest)
                self.assertEqual(0o755, bundle.getmember("package/runtime/loader.so").mode)
                self.assertEqual(0o755, bundle.getmember("package/run.sh").mode)
                self.assertEqual(0o644, bundle.getmember("package/config-templates/example.json").mode)
            before = (package / "SHA256SUMS").read_bytes()
            with self.assertRaises(FileExistsError):
                archive(package, root)
            self.assertEqual(before, (package / "SHA256SUMS").read_bytes())


if __name__ == "__main__":
    unittest.main()
