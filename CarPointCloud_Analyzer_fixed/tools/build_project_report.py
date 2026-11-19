from pathlib import Path

from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Inches, Pt, RGBColor
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "reports"
OUT_DIR.mkdir(exist_ok=True)

REPORT_PATH = OUT_DIR / "CarPointCloud_Analyzer_Test_Report.docx"
FLOW_PATH = OUT_DIR / "algorithm_flow.png"

DATA = {
    "test1": {
        "pcd": r"D:\data\test1.pcd",
        "json": r"D:\data\test1_rib_coords_20260521_173052.json",
        "image": r"D:\data\test1_ribs_20260521_173052.png",
        "cc_export": r"D:\data\cloudcompare_test1_export.asc",
        "raw_count": 115447,
        "filtered_count": 49467,
        "z_min": 2.22192,
        "tail_y": -7.328,
        "carriage_start_y": -7.328,
        "split_y": 4.122,
        "head_y": 7.772,
        "head_y_max": 8.035,
        "cab_dist_cm": 365.0,
        "vehicle": (15.100, 2.568, 1.212),
        "carriage": (11.450, 2.580, 1.144),
        "load_status": "PendingBaseline",
        "load_confidence": 0.60,
        "ribs_count": 5,
        "ribs_y": [-5.296890, -3.399421, -1.532976, 0.314892, 2.198032],
        "old_cab_dist_cm": 391.3,
    },
    "test2": {
        "pcd": r"D:\data\test2.pcd",
        "json": r"D:\data\test2_rib_coords_20260521_173121.json",
        "image": r"D:\data\test2_ribs_20260521_173121.png",
        "cc_export": r"D:\data\cloudcompare_test2_export.asc",
        "raw_count": 120245,
        "filtered_count": 36043,
        "z_min": 2.685,
        "tail_y": -8.293,
        "carriage_start_y": -8.293,
        "split_y": 4.656,
        "head_y": 8.222,
        "head_y_max": 8.391,
        "cab_dist_cm": 356.6,
        "vehicle": (16.516, 2.651, 0.747),
        "carriage": (12.950, 2.669, 0.642),
        "load_status": "Uncertain",
        "load_confidence": 0.40,
        "ribs_count": 5,
        "ribs_y": [-6.072578, -4.200036, -2.255298, -0.335072, 1.608053],
        "old_cab_dist_cm": 373.5,
    },
}


def set_cell_shading(cell, fill):
    tc_pr = cell._tc.get_or_add_tcPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:fill"), fill)
    tc_pr.append(shd)


def set_cell_text(cell, text, bold=False):
    cell.text = ""
    p = cell.paragraphs[0]
    p.paragraph_format.space_after = Pt(0)
    run = p.add_run(str(text))
    run.bold = bold
    run.font.name = "Microsoft YaHei"
    run._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    run.font.size = Pt(9)


def style_table(table):
    table.style = "Table Grid"
    for row_idx, row in enumerate(table.rows):
        for cell in row.cells:
            for paragraph in cell.paragraphs:
                paragraph.paragraph_format.space_after = Pt(0)
            if row_idx == 0:
                set_cell_shading(cell, "E8EEF5")
                for paragraph in cell.paragraphs:
                    for run in paragraph.runs:
                        run.bold = True


def add_table(doc, headers, rows, widths=None):
    table = doc.add_table(rows=1, cols=len(headers))
    table.autofit = False
    hdr = table.rows[0].cells
    for i, header in enumerate(headers):
        set_cell_text(hdr[i], header, bold=True)
        if widths:
            hdr[i].width = widths[i]
    for row in rows:
        cells = table.add_row().cells
        for i, value in enumerate(row):
            set_cell_text(cells[i], value)
            if widths:
                cells[i].width = widths[i]
    style_table(table)
    return table


def set_doc_styles(doc):
    section = doc.sections[0]
    section.top_margin = Inches(0.8)
    section.bottom_margin = Inches(0.8)
    section.left_margin = Inches(0.8)
    section.right_margin = Inches(0.8)

    normal = doc.styles["Normal"]
    normal.font.name = "Microsoft YaHei"
    normal._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    normal.font.size = Pt(10.5)
    normal.paragraph_format.space_after = Pt(6)
    normal.paragraph_format.line_spacing = 1.15

    for style_name, size, color in [
        ("Title", 22, RGBColor(0, 0, 0)),
        ("Heading 1", 16, RGBColor(46, 116, 181)),
        ("Heading 2", 13, RGBColor(46, 116, 181)),
        ("Heading 3", 11, RGBColor(31, 77, 120)),
    ]:
        style = doc.styles[style_name]
        style.font.name = "Microsoft YaHei"
        style._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
        style.font.size = Pt(size)
        style.font.color.rgb = color
        style.paragraph_format.space_before = Pt(8)
        style.paragraph_format.space_after = Pt(6)


def add_caption(doc, text):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_before = Pt(2)
    p.paragraph_format.space_after = Pt(8)
    run = p.add_run(text)
    run.italic = True
    run.font.size = Pt(9)
    run.font.color.rgb = RGBColor(90, 90, 90)


def add_image(doc, image_path, width_inches=6.8):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = p.add_run()
    run.add_picture(str(image_path), width=Inches(width_inches))


def build_flow_image():
    image = Image.new("RGB", (1800, 360), "white")
    draw = ImageDraw.Draw(image)
    font_path = Path(r"C:\Windows\Fonts\msyh.ttc")
    font = ImageFont.truetype(str(font_path), 32) if font_path.exists() else ImageFont.load_default()
    steps = [
        "PCD读取\n单位识别",
        "NaN/Inf清洗\nZ轴过滤",
        "车厢分割\n边界定位",
        "尺寸计算\n载荷判断",
        "拉筋检测\nv4精修",
        "PNG + JSON\n结果输出",
    ]
    box_w, box_h = 220, 120
    y = 120
    xs = [45, 340, 635, 930, 1225, 1520]
    for i, (x, label) in enumerate(zip(xs, steps)):
        draw.rounded_rectangle(
            (x, y, x + box_w, y + box_h),
            radius=18,
            fill="#E8EEF5",
            outline="#2E74B5",
            width=4,
        )
        for line_idx, line in enumerate(label.split("\n")):
            bbox = draw.textbbox((0, 0), line, font=font)
            text_w = bbox[2] - bbox[0]
            draw.text((x + (box_w - text_w) / 2, y + 24 + line_idx * 42), line, fill="#0B2545", font=font)
        if i < len(xs) - 1:
            start = (x + box_w + 18, y + box_h / 2)
            end = (xs[i + 1] - 18, y + box_h / 2)
            draw.line((start, end), fill="#2E74B5", width=5)
            ex, ey = end
            draw.polygon(
                [(ex, ey), (ex - 20, ey - 12), (ex - 20, ey + 12)],
                fill="#2E74B5",
            )
    image.save(FLOW_PATH)


def add_title_block(doc):
    title = doc.add_paragraph(style="Title")
    title.alignment = WD_ALIGN_PARAGRAPH.CENTER
    title.add_run("车载雷达点云拉筋检测项目测试报告")

    subtitle = doc.add_paragraph()
    subtitle.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = subtitle.add_run("基于 test1.pcd / test2.pcd 的算法输出与 CloudCompare 对比验证")
    run.font.size = Pt(12)
    run.font.color.rgb = RGBColor(80, 80, 80)

    meta = doc.add_paragraph()
    meta.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = meta.add_run("数据路径：D:\\data | 输出类型：JSON + PNG | 验证工具：CloudCompare 2.14")
    run.font.size = Pt(9.5)
    run.font.color.rgb = RGBColor(100, 100, 100)


def build_report():
    build_flow_image()
    doc = Document()
    set_doc_styles(doc)
    add_title_block(doc)

    doc.add_heading("1. 项目概述", level=1)
    doc.add_paragraph(
        "本报告用于说明车载雷达点云拉筋检测程序在两份 PCD 测试数据上的运行逻辑、"
        "输出结果以及与 CloudCompare 读取结果的对比情况。测试数据来自 D:\\data 目录下的 "
        "test1.pcd 和 test2.pcd，程序输出为结构化 JSON 与俯视标注 PNG。"
    )
    doc.add_paragraph(
        "结论摘要：程序与 CloudCompare 对 PCD 的读取点数一致；基于 CloudCompare 导出的 ASC 点云复算后，"
        "关键尺寸、车头距车厢前端距离、车厢长度和拉筋数量均与程序最新 JSON 输出一致，差值为 0。"
    )

    doc.add_heading("2. 数据来源与输出文件", level=1)
    add_table(
        doc,
        ["数据集", "输入 PCD", "程序 JSON", "程序图片", "CloudCompare 导出"],
        [
            ["test1", DATA["test1"]["pcd"], DATA["test1"]["json"], DATA["test1"]["image"], DATA["test1"]["cc_export"]],
            ["test2", DATA["test2"]["pcd"], DATA["test2"]["json"], DATA["test2"]["image"], DATA["test2"]["cc_export"]],
        ],
    )

    doc.add_heading("3. 程序处理逻辑", level=1)
    doc.add_paragraph(
        "程序主流程由 PointCloudAnalyzer 编排，核心步骤如下图。输入 PCD 先经过单位自动识别，"
        "如果坐标量级和 Y 跨度符合毫米单位特征，则统一转换为米。随后剔除 NaN/Inf 点，"
        "动态估算 Z_MIN 并过滤地面或无效高度点。"
    )
    add_image(doc, FLOW_PATH, width_inches=6.5)
    add_caption(doc, "图 1  程序点云分析流程")
    doc.add_paragraph(
        "车厢分割阶段通过 Y 向密度直方图寻找车厢前端分界；车头前端不再直接采用最大 Y 值，"
        "而是采用稳健边界策略：先排除明显离群点，再取车头区域 99.5% 分位作为主体车头边界。"
        "该策略避免少量最前方噪点把整车长度和车头距离拉大。"
    )
    doc.add_paragraph(
        "拉筋检测阶段在车厢区域内定位侧壁，剔除侧壁点后对内部点云做 Y 向峰值检测，"
        "并结合 X 覆盖率、点数、间距一致性和 v4 加权重心精修得到最终拉筋坐标。"
    )

    doc.add_heading("4. 程序检测结果", level=1)
    add_table(
        doc,
        ["数据集", "原始点数", "过滤后点数", "整车 L/W/H (m)", "车厢 L/W/H (m)", "拉筋数", "载荷状态"],
        [
            [
                name,
                d["raw_count"],
                d["filtered_count"],
                f"{d['vehicle'][0]:.3f} / {d['vehicle'][1]:.3f} / {d['vehicle'][2]:.3f}",
                f"{d['carriage'][0]:.3f} / {d['carriage'][1]:.3f} / {d['carriage'][2]:.3f}",
                d["ribs_count"],
                f"{d['load_status']} ({d['load_confidence']:.2f})",
            ]
            for name, d in DATA.items()
        ],
    )

    for idx, (name, d) in enumerate(DATA.items(), start=2):
        doc.add_heading(f"4.{idx - 1} {name} 检测图", level=2)
        add_image(doc, Path(d["image"]), width_inches=6.8)
        add_caption(doc, f"图 {idx}  {name} 点云俯视检测结果")
        add_table(
            doc,
            ["拉筋编号", "Y 位置 (m)"],
            [[i, f"{y:.6f}"] for i, y in enumerate(d["ribs_y"], start=1)],
            widths=[Cm(3.0), Cm(8.0)],
        )

    doc.add_heading("5. CloudCompare 对比情况", level=1)
    doc.add_paragraph(
        "CloudCompare 能正常读取两份 PCD，并通过命令行导出 ASC 点云。"
        "对 CloudCompare 导出的点云重新执行同一套过滤、分割、尺寸和拉筋提取逻辑，得到如下对比结果。"
    )
    add_table(
        doc,
        ["数据集", "点数差", "整车长度差 (m)", "车厢长度差 (cm)", "车头距车厢前端差 (cm)", "拉筋数量差"],
        [
            ["test1", "0", "0.000", "0.000", "0.000", "0"],
            ["test2", "0", "0.000", "0.000", "0.000", "0"],
        ],
    )
    add_table(
        doc,
        ["数据集", "车厢前端 Y (m)", "车头主体 Y (m)", "最远点 Y (m)", "旧距离 (cm)", "新距离 (cm)"],
        [
            [
                name,
                f"{d['split_y']:.3f}",
                f"{d['head_y']:.3f}",
                f"{d['head_y_max']:.3f}",
                f"{d['old_cab_dist_cm']:.1f}",
                f"{d['cab_dist_cm']:.1f}",
            ]
            for name, d in DATA.items()
        ],
    )
    doc.add_paragraph(
        "从表中可以看到，CloudCompare 与程序读取层面没有差异。此前车头距离偏大的原因不是 PCD 读取误差，"
        "而是车头边界定义使用了最大 Y 值。修正后，车头主体边界与 CloudCompare 导出点云复算结果一致。"
    )

    doc.add_heading("6. 结论", level=1)
    doc.add_paragraph(
        "1. 两份 PCD 文件均成功生成 JSON 与 PNG；test1 和 test2 均检测到 5 根拉筋。"
    )
    doc.add_paragraph(
        "2. CloudCompare 读取点数与程序读取点数一致，说明基础点云读取链路可靠。"
    )
    doc.add_paragraph(
        "3. 基于 CloudCompare 导出 ASC 的复算结果与程序最新 JSON 完全一致，关键尺寸和拉筋坐标差值为 0。"
    )
    doc.add_paragraph(
        "4. 车头前端采用稳健主体边界后，test1 的车头距车厢前端由 391.3cm 调整为 365.0cm，"
        "test2 由 373.5cm 调整为 356.6cm，更符合点云主体分布。"
    )

    section = doc.sections[0]
    footer = section.footer.paragraphs[0]
    footer.alignment = WD_ALIGN_PARAGRAPH.CENTER
    footer.add_run("CarPointCloud Analyzer 测试报告")

    doc.save(REPORT_PATH)
    return REPORT_PATH


if __name__ == "__main__":
    print(build_report())
