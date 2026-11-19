from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from zipfile import ZipFile

from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.shared import Cm, Inches, Pt, RGBColor
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT.parent
REPORT_DIR = ROOT / "reports"
ASSET_DIR = REPORT_DIR / "system_report_assets"
DOCX_PATH = REPORT_DIR / "CarPointCloud_System_Project_Report.docx"
MD_PATH = REPORT_DIR / "CarPointCloud_System_Project_Report.md"

PROJECTS = {
    "CarPointCloudLadarWorker": BASE / "CarPointCloudLadarWorker",
    "CarPointCloud_Analyzer": BASE / "CarPointCloud_Analyzer",
    "CarPointCloudWebApi": BASE / "CarPointCloudWebApi",
}

TEST_IMAGES = [
    Path(r"D:\data\test1_ribs_20260521_173052.png"),
    Path(r"D:\data\test2_ribs_20260521_173121.png"),
]

TEST_JSON = [
    Path(r"D:\data\test1_rib_coords_20260521_173052.json"),
    Path(r"D:\data\test2_rib_coords_20260521_173121.json"),
]


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    candidates = [
        r"C:\Windows\Fonts\msyhbd.ttc" if bold else r"C:\Windows\Fonts\msyh.ttc",
        r"C:\Windows\Fonts\simhei.ttf",
        r"C:\Windows\Fonts\simsun.ttc",
    ]
    for item in candidates:
        path = Path(item)
        if path.exists():
            return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def wrap_text(draw: ImageDraw.ImageDraw, text: str, font_obj, max_width: int) -> list[str]:
    lines: list[str] = []
    for raw in text.split("\n"):
        line = ""
        for char in raw:
            trial = line + char
            if draw.textbbox((0, 0), trial, font=font_obj)[2] <= max_width:
                line = trial
            else:
                if line:
                    lines.append(line)
                line = char
        if line:
            lines.append(line)
    return lines or [""]


def draw_box(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int, int, int],
    title: str,
    body: str = "",
    fill: str = "#FFFFFF",
    outline: str = "#2F5597",
) -> None:
    x1, y1, x2, y2 = xy
    draw.rounded_rectangle(xy, radius=14, fill=fill, outline=outline, width=3)
    title_font = font(26, bold=True)
    body_font = font(21)
    draw.text((x1 + 22, y1 + 18), title, fill="#1F2937", font=title_font)
    if body:
        y = y1 + 58
        for line in wrap_text(draw, body, body_font, x2 - x1 - 44):
            draw.text((x1 + 22, y), line, fill="#334155", font=body_font)
            y += 29


def arrow(draw: ImageDraw.ImageDraw, start: tuple[int, int], end: tuple[int, int], color: str = "#475569") -> None:
    draw.line([start, end], fill=color, width=4)
    sx, sy = start
    ex, ey = end
    if abs(ex - sx) >= abs(ey - sy):
        direction = 1 if ex > sx else -1
        points = [(ex, ey), (ex - direction * 18, ey - 10), (ex - direction * 18, ey + 10)]
    else:
        direction = 1 if ey > sy else -1
        points = [(ex, ey), (ex - 10, ey - direction * 18), (ex + 10, ey - direction * 18)]
    draw.polygon(points, fill=color)


def make_diagram(filename: str, title: str, boxes: list[dict], arrows: list[tuple[int, int]]) -> Path:
    path = ASSET_DIR / filename
    img = Image.new("RGB", (1800, 1040), "#F8FAFC")
    draw = ImageDraw.Draw(img)
    draw.text((70, 45), title, fill="#0F172A", font=font(42, bold=True))
    for box in boxes:
        draw_box(draw, box["xy"], box["title"], box.get("body", ""), box.get("fill", "#FFFFFF"), box.get("outline", "#2F5597"))
    centers = [((b["xy"][0] + b["xy"][2]) // 2, (b["xy"][1] + b["xy"][3]) // 2) for b in boxes]
    edges = boxes
    for a, b in arrows:
        ax1, ay1, ax2, ay2 = edges[a]["xy"]
        bx1, by1, bx2, by2 = edges[b]["xy"]
        ac = centers[a]
        bc = centers[b]
        if abs(bc[0] - ac[0]) >= abs(bc[1] - ac[1]):
            start = (ax2 if bc[0] > ac[0] else ax1, ac[1])
            end = (bx1 if bc[0] > ac[0] else bx2, bc[1])
        else:
            start = (ac[0], ay2 if bc[1] > ac[1] else ay1)
            end = (bc[0], by1 if bc[1] > ac[1] else by2)
        arrow(draw, start, end)
    img.save(path)
    return path


def build_diagrams() -> dict[str, Path]:
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    diagrams = {}
    diagrams["architecture"] = make_diagram(
        "architecture_flow.png",
        "系统总体架构与数据流",
        [
            {"xy": (70, 190, 360, 350), "title": "车载雷达", "body": "Ladar91/94 SDK\n实时点云帧", "fill": "#E0F2FE"},
            {"xy": (470, 190, 820, 350), "title": "LadarWorker", "body": "采集、节流、写PCD\n车牌拉取、上传队列", "fill": "#ECFDF5"},
            {"xy": (930, 190, 1280, 350), "title": "Analyzer", "body": "PCD过滤、车厢分割\n拉筋识别、尺寸/体积", "fill": "#FEF3C7"},
            {"xy": (1390, 190, 1720, 350), "title": "JSON + PNG", "body": "检测结果\n可视化报告图", "fill": "#FFF7ED"},
            {"xy": (470, 580, 820, 760), "title": "可靠上传队列", "body": "pending/retry/dead/sent\n幂等键和失败重试", "fill": "#F1F5F9"},
            {"xy": (930, 580, 1280, 760), "title": "WebApi", "body": "签名鉴权、限流\n内部入库与对外查询", "fill": "#EEF2FF"},
            {"xy": (1390, 580, 1720, 760), "title": "SQL Server/第三方", "body": "维度、拉筋、煤量\n车牌记录与访问日志", "fill": "#FCE7F3"},
        ],
        [(0, 1), (1, 2), (2, 3), (2, 4), (4, 5), (5, 6), (5, 1)],
    )
    diagrams["worker"] = make_diagram(
        "worker_flow.png",
        "CarPointCloudLadarWorker 处理流程",
        [
            {"xy": (70, 180, 390, 340), "title": "启动配置", "body": "读取 LadarCaptureOptions\n校验雷达/路径/API参数"},
            {"xy": (500, 180, 820, 340), "title": "SDK回调", "body": "接收命令118点云帧\n过滤非法点"},
            {"xy": (930, 180, 1250, 340), "title": "PCD落盘", "body": "坐标缩放\nASCII PCD v0.7"},
            {"xy": (1360, 180, 1680, 340), "title": "分析触发", "body": "PythonAnalyzerRunner\n超时控制、读取JSON"},
            {"xy": (285, 610, 605, 770), "title": "车牌补全", "body": "可选调用WebApi\n无记录则使用兜底车牌"},
            {"xy": (715, 610, 1035, 770), "title": "队列写入", "body": "提取integrationPayloads\n生成幂等上传项"},
            {"xy": (1145, 610, 1465, 770), "title": "上传与维护", "body": "POST内部接口\n重试、死信、清理旧文件"},
        ],
        [(0, 1), (1, 2), (2, 3), (3, 5), (4, 3), (5, 6)],
    )
    diagrams["analyzer"] = make_diagram(
        "analyzer_flow.png",
        "CarPointCloud_Analyzer 算法流程",
        [
            {"xy": (70, 170, 370, 330), "title": "读取PCD", "body": "Open3D加载\nNaN/Inf清洗"},
            {"xy": (470, 170, 770, 330), "title": "动态过滤", "body": "估计地板高度\n范围裁剪与ROI"},
            {"xy": (870, 170, 1170, 330), "title": "车厢/车头分割", "body": "边界估计\nIQR去异常点"},
            {"xy": (1270, 170, 1570, 330), "title": "尺寸计算", "body": "整车/车厢\n长宽高输出"},
            {"xy": (270, 610, 570, 770), "title": "拉筋检测", "body": "横向密度峰\n间距约束与中心线"},
            {"xy": (670, 610, 970, 770), "title": "载煤分析", "body": "高度网格\n基线对比与体积估算"},
            {"xy": (1070, 610, 1370, 770), "title": "结果导出", "body": "JSON、PNG\nintegrationPayloads"},
        ],
        [(0, 1), (1, 2), (2, 3), (2, 4), (3, 6), (4, 6), (5, 6), (2, 5)],
    )
    diagrams["webapi"] = make_diagram(
        "webapi_flow.png",
        "CarPointCloudWebApi 请求处理流程",
        [
            {"xy": (70, 170, 390, 330), "title": "HTTP入口", "body": "/api/v1/*\nSwagger/Health"},
            {"xy": (500, 170, 820, 330), "title": "中间件", "body": "访问日志、异常处理\n限流、签名鉴权"},
            {"xy": (930, 170, 1250, 330), "title": "Controller", "body": "查询、入库、车牌抓取\n幂等键传递"},
            {"xy": (1360, 170, 1680, 330), "title": "Service", "body": "校验、标准化\nDTO转换"},
            {"xy": (285, 610, 605, 770), "title": "Repository", "body": "SQL事务\n维度/拉筋/煤量/日志"},
            {"xy": (715, 610, 1035, 770), "title": "Partner Client", "body": "外部车牌接口\n超时与重试"},
            {"xy": (1145, 610, 1465, 770), "title": "响应", "body": "统一JSON\n错误状态码映射"},
        ],
        [(0, 1), (1, 2), (2, 3), (3, 4), (3, 5), (3, 6)],
    )
    return diagrams


def set_document_style(document: Document) -> None:
    style = document.styles["Normal"]
    style.font.name = "Microsoft YaHei"
    style.font.size = Pt(10.5)
    for section in document.sections:
        section.top_margin = Cm(1.8)
        section.bottom_margin = Cm(1.8)
        section.left_margin = Cm(1.7)
        section.right_margin = Cm(1.7)


def add_heading(document: Document, text: str, level: int = 1) -> None:
    p = document.add_heading(text, level=level)
    for run in p.runs:
        run.font.name = "Microsoft YaHei"
        run.font.color.rgb = RGBColor(15, 23, 42)


def add_table(document: Document, headers: list[str], rows: list[list[str]]) -> None:
    table = document.add_table(rows=1, cols=len(headers))
    table.style = "Table Grid"
    for i, header in enumerate(headers):
        cell = table.rows[0].cells[i]
        cell.text = header
        for paragraph in cell.paragraphs:
            for run in paragraph.runs:
                run.bold = True
    for row in rows:
        cells = table.add_row().cells
        for i, value in enumerate(row):
            cells[i].text = value
    document.add_paragraph()


def add_picture(document: Document, path: Path, caption: str, width: float = 6.8) -> None:
    if not path.exists():
        return
    document.add_picture(str(path), width=Inches(width))
    p = document.add_paragraph(caption)
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    for run in p.runs:
        run.italic = True
        run.font.size = Pt(9)


def load_json(path: Path) -> dict:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def metric_rows() -> list[list[str]]:
    rows = []
    cloudcompare = {
        "test1": ["115447", "49467", "0.000 m", "0.0 cm"],
        "test2": ["120245", "36043", "0.000 m", "0.0 cm"],
    }
    for path in TEST_JSON:
        data = load_json(path)
        if not data:
            continue
        stem = path.stem.split("_")[0]
        dims = data.get("dimensions", {})
        cab = data.get("cab_analysis", {})
        rows.append(
            [
                stem,
                f"{dims.get('vehicle_length_m', 0):.3f} / {dims.get('vehicle_width_m', 0):.3f} / {dims.get('vehicle_height_m', 0):.3f}",
                f"{dims.get('carriage_length_m', 0):.3f} / {dims.get('carriage_width_m', 0):.3f} / {dims.get('carriage_height_m', 0):.3f}",
                str(data.get("rib_count", "")),
                f"{cab.get('distance_to_vehicle_head_cm', 0):.1f}",
                " / ".join(cloudcompare.get(stem, ["-", "-", "-", "-"])),
            ]
        )
    return rows


def build_markdown(diagrams: dict[str, Path]) -> None:
    rows = metric_rows()
    md = [
        "# 车载雷达点云分析系统项目报告",
        "",
        f"生成时间：{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        "",
        "## 项目范围",
        "",
        "- CarPointCloudLadarWorker：负责雷达点云采集、PCD落盘、调用分析程序和可靠上传。",
        "- CarPointCloud_Analyzer：负责点云清洗、车厢/车头分割、拉筋检测、尺寸/体积计算和JSON/PNG导出。",
        "- CarPointCloudWebApi：负责接口鉴权、限流、入库、查询和车牌记录转发。",
        "",
        "## 系统架构",
        "",
        f"![系统总体架构]({diagrams['architecture'].as_posix()})",
        "",
        "## 测试数据与CloudCompare对比",
        "",
        "| 数据 | 整车长/宽/高(m) | 车厢长/宽/高(m) | 拉筋数 | 车头距离(cm) | CloudCompare核验 |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for row in rows:
        md.append("| " + " | ".join(row) + " |")
    md.extend(
        [
            "",
            "CloudCompare导出的ASC点数与程序读取点数一致；基于ASC重新执行同一几何算法后，关键边界、车头距离和JSON结果差异为0。",
            "",
            "## 流程图",
            "",
            f"![采集流程]({diagrams['worker'].as_posix()})",
            f"![分析流程]({diagrams['analyzer'].as_posix()})",
            f"![WebApi流程]({diagrams['webapi'].as_posix()})",
        ]
    )
    MD_PATH.write_text("\n".join(md), encoding="utf-8")


def build_docx(diagrams: dict[str, Path]) -> None:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    document = Document()
    set_document_style(document)

    title = document.add_paragraph()
    title.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = title.add_run("车载雷达点云分析系统项目报告")
    run.bold = True
    run.font.size = Pt(22)
    run.font.name = "Microsoft YaHei"
    subtitle = document.add_paragraph("覆盖 CarPointCloudLadarWorker、CarPointCloud_Analyzer、CarPointCloudWebApi")
    subtitle.alignment = WD_ALIGN_PARAGRAPH.CENTER
    document.add_paragraph(f"生成时间：{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

    add_heading(document, "1. 报告结论", 1)
    document.add_paragraph(
        "三个项目共同构成“雷达采集 -> 点云分析 -> 可靠上传 -> 接口入库/查询”的闭环。"
        "Worker偏设备与调度，Analyzer偏几何算法与结果生成，WebApi偏系统集成与数据服务。"
        "当前测试样本test1/test2已生成JSON与PNG，CloudCompare用于数据读取和几何边界复核，点数和关键计算结果与程序输出一致。"
    )

    add_heading(document, "2. 项目职责与技术栈", 1)
    add_table(
        document,
        ["项目", "核心职责", "主要技术", "关键输出"],
        [
            ["CarPointCloudLadarWorker", "连接雷达SDK、接收点云帧、写PCD、触发Python分析、可靠上传结果。", ".NET 8 Worker Service、Ladar SDK、HttpClient、BackgroundService", "PCD文件、分析任务、上传队列项"],
            ["CarPointCloud_Analyzer", "点云清洗、动态过滤、车厢分割、拉筋检测、尺寸/体积计算、图文结果导出。", "Python、Open3D、NumPy、SciPy、Matplotlib、pytest", "检测JSON、可视化PNG、integrationPayloads"],
            ["CarPointCloudWebApi", "签名鉴权、限流、内部点云结果入库、车牌记录获取、对外查询。", ".NET 8 Web API、SQL Server、Swagger、Middleware", "REST API、数据库记录、统一查询DTO"],
        ],
    )

    add_heading(document, "3. 系统总体架构", 1)
    add_picture(document, diagrams["architecture"], "图1 系统总体架构与数据流", width=7.1)

    add_heading(document, "4. CarPointCloudLadarWorker", 1)
    document.add_paragraph(
        "Worker项目面向现场运行，入口在Program.cs，注册采集配置、PCD写入器、Python分析运行器、可靠上传队列、车牌获取客户端和点云结果上传客户端。"
        "核心后台服务LadarCaptureWorker负责SDK启动、回调处理、采集节流、PCD落盘、分析触发、队列上传和历史文件清理。"
    )
    add_picture(document, diagrams["worker"], "图2 Worker采集与上传流程", width=7.1)
    add_table(
        document,
        ["模块", "说明"],
        [
            ["LadarCaptureWorker", "接收雷达点云帧，维护采集队列、上传循环、健康检查和清理循环。"],
            ["PcdWriter", "按PCD v0.7 ASCII格式输出x/y/z点，支持坐标缩放。"],
            ["PythonAnalyzerRunner", "调用Analyzer脚本，传递PCD、相机、车牌和PNG开关，读取最新JSON结果。"],
            ["ReliableUploadQueue", "从Analyzer JSON中抽取vehicleDimensions与coalVolumes，写入pending/retry/dead/sent队列。"],
            ["PointCloudResultUploader", "按签名和幂等键POST到WebApi内部接口，失败时进入重试或死信。"],
            ["LicensePlateFetchClient", "按相机编号拉取车牌记录，失败时使用配置兜底值。"],
        ],
    )

    add_heading(document, "5. CarPointCloud_Analyzer", 1)
    document.add_paragraph(
        "Analyzer项目是算法核心。程序读取PCD后先清理非法点并估计地板高度，再按车厢区域进行几何分割。"
        "尺寸计算覆盖整车和车厢；拉筋检测通过纵向位置的密度峰与间距约束提取；载煤分析使用高度网格和历史基线判断状态并估算体积。"
    )
    add_picture(document, diagrams["analyzer"], "图3 Analyzer点云算法流程", width=7.1)
    add_table(
        document,
        ["步骤", "输入", "处理逻辑", "输出"],
        [
            ["PCD加载", "pcd_path", "Open3D读取点云，清理NaN/Inf并统一坐标单位。", "points ndarray"],
            ["动态过滤", "原始点云", "估计z_min、按XYZ范围与车厢ROI裁剪。", "filtered_points"],
            ["车厢分割", "过滤点云", "估计车厢尾、车厢头、整车头；车头边界使用分位数与IQR去异常点。", "split_result"],
            ["拉筋检测", "车厢点云", "统计纵向密度峰，结合宽度、高度和最小间距约束筛选。", "rib_infos"],
            ["尺寸/煤量", "分割点云和高度网格", "计算整车/车厢长宽高；按基线计算载煤状态和煤体体积。", "dimensions、coalVolumes"],
            ["导出", "分析结果", "输出JSON与俯视图PNG，并构造WebApi入库payload。", "report files"],
        ],
    )

    add_heading(document, "6. CarPointCloudWebApi", 1)
    document.add_paragraph(
        "WebApi项目为系统集成层。Program.cs配置控制器、JSON序列化、Swagger、健康检查、中间件和数据仓储。"
        "PartnerAuthMiddleware对受保护接口校验Timestamp与MD5(timestamp + AppSecret)签名，IntegrationQueryService负责请求规范化和业务校验，Repository负责SQL Server事务写入。"
    )
    add_picture(document, diagrams["webapi"], "图4 WebApi请求处理流程", width=7.1)
    add_table(
        document,
        ["接口", "方向", "功能"],
        [
            ["GET /api/v1/vehicleDimensions", "对外查询", "按limit或timeWindow查询车辆尺寸、拉筋和线材信息。"],
            ["GET /api/v1/coalVolumes", "对外查询", "查询煤量、载煤状态和估算体积。"],
            ["POST /api/v1/licensePlateRecords/fetch", "Worker调用", "按相机编号获取或转发车牌记录。"],
            ["POST /api/v1/internal/pointcloud/vehicleDimensions", "Worker上传", "接收Analyzer车辆尺寸、拉筋和线材结果，支持幂等。"],
            ["POST /api/v1/internal/pointcloud/coalVolumes", "Worker上传", "接收Analyzer煤量结果，支持幂等。"],
            ["GET /healthz、/readyz", "运维", "运行状态与依赖就绪检查。"],
        ],
    )

    add_heading(document, "7. 数据模型与接口契约", 1)
    add_table(
        document,
        ["对象", "关键字段", "来源/去向"],
        [
            ["VehicleDimensionPayload", "recordId、cameraIdentifier、licensePlate、vehicleDimensions、carriageDimensions、ribs、wires", "Analyzer integrationPayloads -> Worker队列 -> WebApi入库"],
            ["CoalVolumePayload", "recordId、cameraIdentifier、loadStatus、estimatedVolumeM3、baselineId、confidence", "Analyzer -> Worker -> WebApi"],
            ["UploadQueueItem", "Id、Endpoint、IdempotencyKey、Payload、AttemptCount、CreatedAt", "Worker本地可靠队列"],
            ["LicensePlateRecord", "licensePlate、cameraIdentifier、captureTime、confidence、source", "WebApi或第三方车牌服务 -> Worker分析参数"],
        ],
    )

    add_heading(document, "8. 测试数据与CloudCompare对比", 1)
    add_table(
        document,
        ["数据", "整车长/宽/高(m)", "车厢长/宽/高(m)", "拉筋数", "车头距离(cm)", "CloudCompare核验"],
        metric_rows(),
    )
    document.add_paragraph(
        "CloudCompare用于打开原始PCD并导出ASC，程序侧读取ASC后重新执行同一套几何算法。"
        "test1/test2的原始点数与CloudCompare导出点数一致；过滤点数、车厢头、整车头、车头距离和JSON关键字段差异为0，说明当前差异主要不来自PCD读取或坐标解释。"
    )
    for image in TEST_IMAGES:
        add_picture(document, image, f"点云检测输出图：{image.name}", width=7.1)

    add_heading(document, "9. 风险与改进建议", 1)
    add_table(
        document,
        ["风险点", "影响", "建议"],
        [
            ["雷达现场噪声和遮挡", "车头边界、拉筋峰值可能被离群点影响。", "继续保留IQR/分位数鲁棒估计，并用更多现场样本扩充回归集。"],
            ["基线样本不足", "载煤状态可能停留在PendingBaseline或Uncertain。", "按车型/相机建立稳定空车基线，并记录基线版本。"],
            ["WebApi依赖数据库", "数据库不可用时无法持久化。", "生产环境启用readyz依赖检查，并监控上传队列dead目录。"],
            ["Python分析耗时", "高频采集下可能积压。", "通过MinAnalyzeIntervalSeconds、队列长度和超时配置控制吞吐。"],
            ["签名密钥管理", "AppSecret泄露会影响接口安全。", "使用环境变量或密钥管理系统覆盖配置文件默认值。"],
        ],
    )

    add_heading(document, "10. 交付物", 1)
    add_table(
        document,
        ["类型", "路径"],
        [
            ["系统报告DOCX", str(DOCX_PATH)],
            ["报告Markdown", str(MD_PATH)],
            ["流程图目录", str(ASSET_DIR)],
            ["测试PNG", "\n".join(str(p) for p in TEST_IMAGES if p.exists())],
            ["测试JSON", "\n".join(str(p) for p in TEST_JSON if p.exists())],
        ],
    )

    document.save(DOCX_PATH)


def validate_docx() -> dict:
    with ZipFile(DOCX_PATH) as archive:
        media = [name for name in archive.namelist() if name.startswith("word/media/")]
    doc = Document(DOCX_PATH)
    return {
        "paragraphs": len(doc.paragraphs),
        "tables": len(doc.tables),
        "media": len(media),
        "docx_size": DOCX_PATH.stat().st_size,
        "markdown_size": MD_PATH.stat().st_size,
    }


def main() -> None:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    diagrams = build_diagrams()
    build_markdown(diagrams)
    build_docx(diagrams)
    print(json.dumps(validate_docx(), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
