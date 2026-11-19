"""载货页真实浏览器布局回归；请求全部由只读界面夹具提供，不连接设备或数据库。"""
import json
import os
from pathlib import Path
import re
import shutil
import unittest
from urllib.parse import urlsplit

try:
    from playwright.sync_api import sync_playwright
except ImportError:
    sync_playwright = None


VIEWER = Path(__file__).resolve().parents[2] / "wwwroot" / "pointcloud-viewer"
ORIGIN = "http://load-radar-layout.test"


@unittest.skipIf(sync_playwright is None, "布局回归需要 Playwright 和 Chromium 浏览器")
class LoadRadarLayoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runtime = sync_playwright().start()
        candidates = [os.environ.get("PLAYWRIGHT_CHROMIUM_EXECUTABLE"),
                      cls.runtime.chromium.executable_path,
                      shutil.which("chromium"), shutil.which("google-chrome"),
                      r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"]
        executable = next((str(p) for p in candidates if p and Path(p).is_file()), None)
        if executable is None:
            cls.runtime.stop()
            raise unittest.SkipTest("本机没有可用的 Chromium 浏览器")
        cls.browser = cls.runtime.chromium.launch(headless=True, executable_path=executable)

    @classmethod
    def tearDownClass(cls):
        cls.browser.close()
        cls.runtime.stop()

    def page(self, width, scale):
        context = self.browser.new_context(viewport={"width": width, "height": 1000}, device_scale_factor=scale)
        self.addCleanup(context.close)
        html = (VIEWER / "load-radar.html").read_text(encoding="utf-8")
        html = re.sub(r"<script\b[^>]*>.*?</script>", "", html, flags=re.S)
        # 使用生产布局和生产页面脚本，只替换网络读取；夹具不启动任何业务服务。
        bootstrap = """<script>
        const fixtures = Array.from({length:4}, (_, i) => ({
            nodeId:'LAYOUT', channelId:'CH'+(i+1), name:'载货雷达通道'+(i+1), online:false,
            error:'采集服务离线或返回无效快照', state:{online:false, runtime_events:[]}
        }));
        window.API = {request:async path => ({data:path.includes('/state') ? {enabled:true,lanes:fixtures} : []})};
        </script><script src="js/load-radar-cloud.js"></script><script src="js/load-radar.js"></script>"""
        html = html.replace("</body>", bootstrap + "</body>")

        def serve(route):
            url = urlsplit(route.request.url)
            if url.netloc != "load-radar-layout.test" or route.request.method != "GET":
                route.abort()
                return
            relative = url.path.removeprefix("/pointcloud-viewer/")
            if relative == "load-radar.html":
                route.fulfill(status=200, content_type="text/html; charset=utf-8", body=html)
                return
            target = (VIEWER / relative).resolve()
            if not target.is_relative_to(VIEWER.resolve()) or not target.is_file():
                route.fulfill(status=404, body="")
                return
            mime = {".css": "text/css", ".js": "application/javascript", ".svg": "image/svg+xml"}
            route.fulfill(status=200, content_type=mime.get(target.suffix, "application/octet-stream"), body=target.read_bytes())

        context.route("**/*", serve)
        page = context.new_page()
        page.set_default_timeout(3000)
        page.goto(ORIGIN + "/pointcloud-viewer/load-radar.html")
        page.wait_for_selector("#loadLanes .card")
        return page

    def assert_layout(self, page, single):
        page.evaluate("""() => {
            const tbody = document.querySelector('.lane-log-rows');
            if (tbody) tbody.innerHTML='<tr><td>2026/10/29 20:17:49</td><td>载货雷达通道 CH1</td><td>未关联车牌</td><td>等待车辆停稳</td><td>--</td><td>'+'连续等待车牌及基线确认'.repeat(35)+'</td></tr>';
            const runtime = document.querySelector('.runtime-log-rows');
            if (runtime) runtime.innerHTML=Array.from({length:20},()=>'<tr><td>2026/10/29 20:17:49</td><td>载货雷达通道 CH1</td><td>车辆已停稳</td><td>'+'本车已留样等待车牌确认'.repeat(20)+'</td></tr>').join('');
        }""")
        page.wait_for_timeout(120)
        metrics = page.evaluate("""() => {
            const rect = selector => {
                const r=document.querySelector(selector).getBoundingClientRect();
                return {left:r.left,right:r.right,width:r.width,top:r.top,bottom:r.bottom};
            };
            return {viewport:document.documentElement.clientWidth, pageWidth:document.documentElement.scrollWidth,
                lanes:rect('#loadLanes'),logs:rect('#runLogs'),header:rect('.radar-header'),nav:rect('#global-nav'),
                card:rect('#loadLanes .card:not([hidden])'),runtimeScroll:rect('.runtime-log-scroll'),
                savedColumns:document.querySelectorAll('.lane-log-rows tr:first-child td').length,
                runtimeColumns:document.querySelectorAll('.runtime-log-rows tr:first-child td').length};
        }""")
        self.assertLessEqual(metrics["pageWidth"], metrics["viewport"] + 1, json.dumps(metrics))
        self.assertGreaterEqual(metrics["header"]["top"], metrics["nav"]["bottom"] - 1, json.dumps(metrics))
        for name in ["lanes", "logs", "card"]:
            self.assertGreater(metrics[name]["width"], 0, json.dumps(metrics))
            self.assertLessEqual(metrics[name]["right"], metrics["viewport"] + 1, json.dumps(metrics))
        self.assertAlmostEqual(metrics["lanes"]["left"], metrics["logs"]["left"], delta=1)
        self.assertAlmostEqual(metrics["lanes"]["right"], metrics["logs"]["right"], delta=1)
        self.assertEqual(metrics["savedColumns"], 6)
        self.assertEqual(metrics["runtimeColumns"], 4)
        self.assertLessEqual(metrics["runtimeScroll"]["bottom"] - metrics["runtimeScroll"]["top"], 281)
        self.assertAlmostEqual(metrics["runtimeScroll"]["width"], metrics["logs"]["width"] - 2, delta=1)
        if single:
            self.assertAlmostEqual(metrics["card"]["width"], metrics["logs"]["width"], delta=1)

    def test_channel_switches_keep_cards_and_logs_aligned(self):
        for width in [1920, 1366, 800, 390]:
            for scale in [1, 2]:
                with self.subTest(width=width, scale=scale):
                    page = self.page(width, scale)
                    self.assert_layout(page, False)
                    for label in ["载货雷达通道1 CH1", "载货雷达通道2 CH2", "全部", "载货雷达通道1 CH1", "全部"]:
                        page.get_by_role("button", name=label, exact=True).click()
                        self.assert_layout(page, label != "全部")


if __name__ == "__main__":
    unittest.main()
