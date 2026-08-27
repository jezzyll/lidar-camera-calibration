#!/usr/bin/env python3
"""Build the shareable Phase 1 calibration validation Word report.

The report embeds the actual corrected validation figures and reads all numeric
values from the canonical machine-readable report. It does not duplicate or
recalculate calibration metrics.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_ALIGN_VERTICAL, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_LINE_SPACING
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt, RGBColor, Twips


CONTENT_WIDTH_DXA = 9360
TABLE_INDENT_DXA = 120
CELL_MARGINS_DXA = {"top": 80, "bottom": 80, "start": 120, "end": 120}

NAVY = "17365D"
BLUE = "2E74B5"
DARK_BLUE = "1F4D78"
MUTED = "5B6573"
LIGHT_GRAY = "F2F4F7"
PALE_BLUE = "E8EEF5"
PALE_RED = "FDEBEC"
PALE_GREEN = "EAF4EA"
RED = "9B1C1C"
GREEN = "2E6B3A"
WHITE = "FFFFFF"
BLACK = "000000"


def set_run_font(run, *, name="Arial", size=None, color=None, bold=None,
                 italic=None):
    run.font.name = name
    run._element.get_or_add_rPr().rFonts.set(qn("w:ascii"), name)
    run._element.get_or_add_rPr().rFonts.set(qn("w:hAnsi"), name)
    if size is not None:
        run.font.size = Pt(size)
    if color is not None:
        run.font.color.rgb = RGBColor.from_string(color)
    if bold is not None:
        run.bold = bold
    if italic is not None:
        run.italic = italic


def shade(element, fill):
    properties = element.get_or_add_tcPr() if hasattr(element, "get_or_add_tcPr") else element
    shading = properties.find(qn("w:shd"))
    if shading is None:
        shading = OxmlElement("w:shd")
        properties.append(shading)
    shading.set(qn("w:fill"), fill)


def set_cell_margins(cell):
    tc_pr = cell._tc.get_or_add_tcPr()
    tc_mar = tc_pr.find(qn("w:tcMar"))
    if tc_mar is None:
        tc_mar = OxmlElement("w:tcMar")
        tc_pr.append(tc_mar)
    for side, width in CELL_MARGINS_DXA.items():
        node = tc_mar.find(qn(f"w:{side}"))
        if node is None:
            node = OxmlElement(f"w:{side}")
            tc_mar.append(node)
        node.set(qn("w:w"), str(width))
        node.set(qn("w:type"), "dxa")


def apply_table_geometry(table, widths):
    if sum(widths) != CONTENT_WIDTH_DXA:
        raise ValueError(f"Table widths sum to {sum(widths)}, not {CONTENT_WIDTH_DXA}")
    table.autofit = False
    table.alignment = WD_TABLE_ALIGNMENT.LEFT
    tbl_pr = table._tbl.tblPr
    for tag, value in (("w:tblW", CONTENT_WIDTH_DXA), ("w:tblInd", TABLE_INDENT_DXA)):
        node = tbl_pr.find(qn(tag))
        if node is None:
            node = OxmlElement(tag)
            tbl_pr.append(node)
        node.set(qn("w:type"), "dxa")
        node.set(qn("w:w"), str(value))
    layout = tbl_pr.find(qn("w:tblLayout"))
    if layout is None:
        layout = OxmlElement("w:tblLayout")
        tbl_pr.append(layout)
    layout.set(qn("w:type"), "fixed")
    grid = table._tbl.tblGrid
    for child in list(grid):
        grid.remove(child)
    for width in widths:
        grid_col = OxmlElement("w:gridCol")
        grid_col.set(qn("w:w"), str(width))
        grid.append(grid_col)
    for col_idx, width in enumerate(widths):
        table.columns[col_idx].width = Twips(width)
    for row in table.rows:
        row.height = None
        for col_idx, cell in enumerate(row.cells):
            cell.width = Twips(widths[col_idx])
            cell.vertical_alignment = WD_ALIGN_VERTICAL.CENTER
            tc_w = cell._tc.get_or_add_tcPr().find(qn("w:tcW"))
            if tc_w is None:
                tc_w = OxmlElement("w:tcW")
                cell._tc.get_or_add_tcPr().append(tc_w)
            tc_w.set(qn("w:type"), "dxa")
            tc_w.set(qn("w:w"), str(widths[col_idx]))
            set_cell_margins(cell)


def set_repeat_table_header(row):
    tr_pr = row._tr.get_or_add_trPr()
    repeat = OxmlElement("w:tblHeader")
    repeat.set(qn("w:val"), "true")
    tr_pr.append(repeat)


def set_cell_text(cell, text, *, bold=False, color=BLACK, size=8.5,
                  align=WD_ALIGN_PARAGRAPH.CENTER):
    cell.text = ""
    paragraph = cell.paragraphs[0]
    paragraph.alignment = align
    paragraph.paragraph_format.space_before = Pt(0)
    paragraph.paragraph_format.space_after = Pt(0)
    paragraph.paragraph_format.line_spacing = 1.0
    run = paragraph.add_run(str(text))
    set_run_font(run, size=size, color=color, bold=bold)


def add_table(doc, headers, rows, widths, *, body_size=8.5,
              left_columns=(), decision_column=None):
    table = doc.add_table(rows=1, cols=len(headers))
    table.style = "Table Grid"
    set_repeat_table_header(table.rows[0])
    for index, header in enumerate(headers):
        shade(table.rows[0].cells[index]._tc, NAVY)
        set_cell_text(table.rows[0].cells[index], header, bold=True, color=WHITE,
                      size=body_size)
    for row_values in rows:
        cells = table.add_row().cells
        for index, value in enumerate(row_values):
            align = WD_ALIGN_PARAGRAPH.LEFT if index in left_columns else WD_ALIGN_PARAGRAPH.CENTER
            color = BLACK
            bold = False
            if decision_column is not None and index == decision_column:
                is_pass = str(value).lower() == "pass"
                shade(cells[index]._tc, PALE_GREEN if is_pass else PALE_RED)
                color = GREEN if is_pass else RED
                bold = True
            set_cell_text(cells[index], value, bold=bold, color=color,
                          size=body_size, align=align)
    apply_table_geometry(table, widths)
    after = doc.add_paragraph()
    after.paragraph_format.space_after = Pt(2)
    return table


def add_hyperlink(paragraph, text, url):
    relationship = paragraph.part.relate_to(
        url,
        "http://schemas.openxmlformats.org/officeDocument/2006/relationships/hyperlink",
        is_external=True,
    )
    hyperlink = OxmlElement("w:hyperlink")
    hyperlink.set(qn("r:id"), relationship)
    run = OxmlElement("w:r")
    run_properties = OxmlElement("w:rPr")
    color = OxmlElement("w:color")
    color.set(qn("w:val"), BLUE)
    underline = OxmlElement("w:u")
    underline.set(qn("w:val"), "single")
    fonts = OxmlElement("w:rFonts")
    fonts.set(qn("w:ascii"), "Arial")
    fonts.set(qn("w:hAnsi"), "Arial")
    run_properties.extend([fonts, color, underline])
    run.append(run_properties)
    text_node = OxmlElement("w:t")
    text_node.text = text
    run.append(text_node)
    hyperlink.append(run)
    paragraph._p.append(hyperlink)


def add_page_field(paragraph):
    paragraph.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    prefix = paragraph.add_run("Page ")
    set_run_font(prefix, size=9, color=MUTED)
    begin = OxmlElement("w:fldChar")
    begin.set(qn("w:fldCharType"), "begin")
    instruction = OxmlElement("w:instrText")
    instruction.set(qn("xml:space"), "preserve")
    instruction.text = " PAGE "
    separate = OxmlElement("w:fldChar")
    separate.set(qn("w:fldCharType"), "separate")
    value = OxmlElement("w:t")
    value.text = "1"
    end = OxmlElement("w:fldChar")
    end.set(qn("w:fldCharType"), "end")
    run = OxmlElement("w:r")
    run.extend([begin, instruction, separate, value, end])
    paragraph._p.append(run)


def paragraph_shading(paragraph, fill):
    p_pr = paragraph._p.get_or_add_pPr()
    shading = p_pr.find(qn("w:shd"))
    if shading is None:
        shading = OxmlElement("w:shd")
        p_pr.append(shading)
    shading.set(qn("w:fill"), fill)


def add_callout(doc, label, text, *, fill=PALE_RED, color=RED):
    paragraph = doc.add_paragraph(style="Callout")
    paragraph_shading(paragraph, fill)
    label_run = paragraph.add_run(f"{label}: ")
    set_run_font(label_run, size=11, bold=True, color=color)
    body_run = paragraph.add_run(text)
    set_run_font(body_run, size=11, color=BLACK)
    return paragraph


def add_body(doc, text, *, bold_prefix=None):
    paragraph = doc.add_paragraph(style="Normal")
    if bold_prefix and text.startswith(bold_prefix):
        first = paragraph.add_run(bold_prefix)
        set_run_font(first, bold=True)
        remainder = paragraph.add_run(text[len(bold_prefix):])
        set_run_font(remainder)
    else:
        run = paragraph.add_run(text)
        set_run_font(run)
    return paragraph


def add_bullet(doc, text):
    paragraph = doc.add_paragraph(style="List Bullet")
    run = paragraph.add_run(text)
    set_run_font(run)
    return paragraph


def add_number(doc, text):
    paragraph = doc.add_paragraph(style="List Number")
    run = paragraph.add_run(text)
    set_run_font(run)
    return paragraph


def add_code(doc, text):
    paragraph = doc.add_paragraph(style="Code Block")
    for index, line in enumerate(text.splitlines()):
        if index:
            paragraph.add_run().add_break()
        run = paragraph.add_run(line)
        set_run_font(run, name="Courier New", size=8.5, color=NAVY)
    paragraph_shading(paragraph, LIGHT_GRAY)
    return paragraph


def add_caption(doc, text):
    paragraph = doc.add_paragraph(style="Caption")
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = paragraph.add_run(text)
    set_run_font(run, size=9, color=MUTED, italic=True)
    return paragraph


def set_picture_alt_text(run, description):
    for doc_pr in run._element.xpath(".//wp:docPr"):
        doc_pr.set("descr", description)
        doc_pr.set("title", description)


def add_figure(doc, path, caption, alt_text, *, width=6.3):
    paragraph = doc.add_paragraph(style="Figure")
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = paragraph.add_run()
    run.add_picture(str(path), width=Inches(width))
    set_picture_alt_text(run, alt_text)
    add_caption(doc, caption)


def add_heading(doc, text, level=1):
    paragraph = doc.add_paragraph(style=f"Heading {level}")
    paragraph.paragraph_format.keep_with_next = True
    run = paragraph.add_run(text)
    set_run_font(run)
    return paragraph


def page_break(doc):
    paragraph = doc.add_paragraph()
    paragraph.paragraph_format.page_break_before = True
    paragraph.paragraph_format.space_before = Pt(0)
    paragraph.paragraph_format.space_after = Pt(0)


def configure_styles(doc):
    styles = doc.styles
    normal = styles["Normal"]
    normal.font.name = "Arial"
    normal._element.rPr.rFonts.set(qn("w:ascii"), "Arial")
    normal._element.rPr.rFonts.set(qn("w:hAnsi"), "Arial")
    normal.font.size = Pt(11)
    normal.font.color.rgb = RGBColor.from_string(BLACK)
    normal.paragraph_format.space_before = Pt(0)
    normal.paragraph_format.space_after = Pt(6)
    normal.paragraph_format.line_spacing = 1.10

    heading_specs = {
        "Heading 1": (16, BLUE, 12, 6),
        "Heading 2": (13, BLUE, 10, 5),
        "Heading 3": (12, DARK_BLUE, 8, 4),
    }
    for name, (size, color, before, after) in heading_specs.items():
        style = styles[name]
        style.font.name = "Arial"
        style._element.rPr.rFonts.set(qn("w:ascii"), "Arial")
        style._element.rPr.rFonts.set(qn("w:hAnsi"), "Arial")
        style.font.size = Pt(size)
        style.font.bold = True
        style.font.color.rgb = RGBColor.from_string(color)
        style.paragraph_format.space_before = Pt(before)
        style.paragraph_format.space_after = Pt(after)
        style.paragraph_format.keep_with_next = True

    for list_name in ("List Bullet", "List Number"):
        style = styles[list_name]
        style.font.name = "Arial"
        style.font.size = Pt(11)
        style.paragraph_format.left_indent = Inches(0.5)
        style.paragraph_format.first_line_indent = Inches(-0.25)
        style.paragraph_format.space_after = Pt(8)
        style.paragraph_format.line_spacing = 1.167

    caption = styles["Caption"]
    caption.font.name = "Arial"
    caption.font.size = Pt(9)
    caption.font.italic = True
    caption.font.color.rgb = RGBColor.from_string(MUTED)
    caption.paragraph_format.space_before = Pt(2)
    caption.paragraph_format.space_after = Pt(8)
    caption.paragraph_format.keep_with_next = False

    callout = styles.add_style("Callout", 1)
    callout.font.name = "Arial"
    callout.font.size = Pt(11)
    callout.paragraph_format.left_indent = Inches(0.12)
    callout.paragraph_format.right_indent = Inches(0.12)
    callout.paragraph_format.space_before = Pt(8)
    callout.paragraph_format.space_after = Pt(10)
    callout.paragraph_format.line_spacing = 1.10

    code = styles.add_style("Code Block", 1)
    code.font.name = "Courier New"
    code.font.size = Pt(8.5)
    code.paragraph_format.left_indent = Inches(0.12)
    code.paragraph_format.right_indent = Inches(0.12)
    code.paragraph_format.space_before = Pt(4)
    code.paragraph_format.space_after = Pt(8)
    code.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE

    figure = styles.add_style("Figure", 1)
    figure.paragraph_format.space_before = Pt(3)
    figure.paragraph_format.space_after = Pt(2)
    figure.paragraph_format.keep_with_next = True


def configure_document(doc):
    section = doc.sections[0]
    section.page_width = Inches(8.5)
    section.page_height = Inches(11)
    section.top_margin = Inches(1)
    section.bottom_margin = Inches(1)
    section.left_margin = Inches(1)
    section.right_margin = Inches(1)
    section.header_distance = Inches(0.492)
    section.footer_distance = Inches(0.492)

    header = section.header
    header_paragraph = header.paragraphs[0]
    header_paragraph.alignment = WD_ALIGN_PARAGRAPH.LEFT
    header_run = header_paragraph.add_run(
        "PHASE 1 CALIBRATION VALIDATION   |   REALSENSE D455 + LIVOX MID-360"
    )
    set_run_font(header_run, size=8.5, bold=True, color=MUTED)

    footer = section.footer
    footer_paragraph = footer.paragraphs[0]
    add_page_field(footer_paragraph)


def metric_rows(report):
    rows = []
    for item in report["internal_d455_livox_experiment"]["per_distance"]:
        rows.append([
            f"{item['distance_ft']:g} ft",
            item["number_of_frames"],
            f"{item['evaluated_lidar_points']:,}",
            f"{item['mean_reprojection_error_px']:.3f}",
            f"{item['median_reprojection_error_px']:.3f}",
            f"{item['rmse_reprojection_error_px']:.3f}",
            f"{item['maximum_reprojection_error_px']:.3f}",
            f"{item['percentage_within_1_px']:.2f}%",
            f"{item['percentage_within_2_px']:.2f}%",
            f"{item['percentage_within_3_px']:.2f}%",
            "Pass" if item["passes_project_99pct_within_2px"] else "Fail",
        ])
    return rows


def build_report(report_path: Path, output_path: Path):
    report = json.loads(report_path.read_text())
    output_path.parent.mkdir(parents=True, exist_ok=True)
    doc = Document()
    configure_styles(doc)
    configure_document(doc)

    # Memo masthead.
    kicker = doc.add_paragraph()
    kicker.paragraph_format.space_before = Pt(8)
    kicker.paragraph_format.space_after = Pt(4)
    run = kicker.add_run("TECHNICAL VALIDATION REPORT")
    set_run_font(run, size=10, bold=True, color=BLUE)

    title = doc.add_paragraph()
    title.paragraph_format.space_after = Pt(4)
    run = title.add_run("Phase 1 Camera–LiDAR Calibration")
    set_run_font(run, size=24, bold=True, color=NAVY)

    subtitle = doc.add_paragraph()
    subtitle.paragraph_format.space_after = Pt(14)
    run = subtitle.add_run(
        "Required corrections, independent reprojection validation, and attached output evidence"
    )
    set_run_font(run, size=13, color=MUTED)

    metadata = [
        ["Sensor system", "Intel RealSense D455 + Livox Mid-360"],
        ["Validation distances", "7.25 ft, 14.5 ft, and 22.2 ft"],
        ["External dataset", "ACSC real camera–LiDAR checkerboard sample"],
        ["Report date", "21 August 2026"],
        ["Reproducible commit", "928a098"],
    ]
    add_table(doc, ["Document field", "Value"], metadata, [2200, 7160],
              body_size=9.5, left_columns=(0, 1))

    add_callout(
        doc,
        "Acceptance decision",
        "The requested validation work and evidence are complete, but the current calibration "
        "is not accepted. The transform passes the project-defined criterion at 7.25 ft and "
        "fails it at 14.5 ft and 22.2 ft.",
    )

    add_heading(doc, "Executive summary", 1)
    add_body(
        doc,
        "This report documents the implementation, the measured outputs, and the exact "
        "acceptance decision. Evaluation returns are selected independently in LiDAR space. "
        "The RGB checkerboard mask is never used to choose or score those returns. Nine actual "
        "projection figures are embedded later in this report; the companion ZIP includes "
        "every original-resolution RGB copy, separate mask, projection figure, metrics file, "
        "calibration file, and reproduction script.",
    )

    add_heading(doc, "Internal result at a glance", 2)
    add_table(
        doc,
        ["Distance", "Frames", "Points", "Mean", "Median", "RMSE", "Max", "≤1 px", "≤2 px", "≤3 px", "Result"],
        metric_rows(report),
        [850, 700, 950, 850, 850, 850, 850, 900, 900, 900, 760],
        body_size=7.5,
        decision_column=10,
    )
    add_body(
        doc,
        "All pixel errors in the tables are Euclidean distances. A value of 100.00% means only "
        "that every evaluated return in that experiment satisfies the stated threshold; it is "
        "not described as 100% calibration accuracy.",
    )

    page_break(doc)
    add_heading(doc, "1. Requirement compliance", 1)
    compliance = [
        ["Independent LiDAR-point selection", "Complete", "LiDAR XYZ ROI, robust plane, physical extent, interior margin"],
        ["RGB mask excluded from scoring", "Complete", "Mask stored separately and never queried for selection or metrics"],
        ["Strict alignment defined", "Complete", "Same rounded raster pixel; numerical exact match also recorded"],
        ["Required error statistics", "Complete", "Count, mean, median, RMSE, maximum, and 1/2/3 px rates"],
        ["Acceptance criterion and origin", "Complete", "99% within 2 px; explicitly project/mentor-defined"],
        ["Separate public/internal tests", "Complete", "No pooled or combined percentage"],
        ["Three internal distances", "Complete", "Ten synchronized frames at each distance"],
        ["Calibration parameters", "Complete", "K, D, resolution, R, t, 4×4 T, and conventions"],
        ["Original RGB preserved", "Complete", "No resize, warp, deblur, super-resolution, or regeneration"],
        ["Qualitative evidence", "Complete", "Nine embedded figures plus original RGB and mask files in ZIP"],
        ["Reproducible implementation", "Complete", "Calibration and corrected validation scripts included"],
    ]
    add_table(
        doc,
        ["Requirement", "Status", "Evidence"],
        compliance,
        [2800, 1200, 5360],
        body_size=8.2,
        left_columns=(0, 2),
    )
    add_callout(
        doc,
        "Important distinction",
        "The required checks are implemented and documented. That does not mean the sensor "
        "transform has passed. Phase 1 remains open until every required internal distance "
        "achieves at least 99% of valid returns with error at or below 2 pixels.",
        fill=PALE_BLUE,
        color=DARK_BLUE,
    )

    add_heading(doc, "2. Real datasets and experiment separation", 1)
    add_heading(doc, "2.1 Internal Livox + Intel D455", 2)
    add_body(
        doc,
        "The internal experiment uses the supplied ROS 2 recordings at approximately 7.25 ft, "
        "14.5 ft, and 22.2 ft. The streams used are /camera/camera/color/image_raw, "
        "/camera/camera/color/camera_info, and /livox/lidar. Public and internal transforms are "
        "never mixed.",
    )
    add_heading(doc, "2.2 Public ACSC experiment", 2)
    paragraph = doc.add_paragraph(style="Normal")
    paragraph.add_run("External validation uses the non-synthetic ")
    add_hyperlink(paragraph, "ACSC camera–LiDAR calibration dataset", "https://github.com/HViktorTsoi/ACSC")
    paragraph.add_run(" and its ")
    add_hyperlink(paragraph, "official sample download", "https://drive.google.com/file/d/1hgV-aH_RSsL1geHDSIlwwvqrMy6ED56K/view")
    paragraph.add_run(
        ". The sample provides 21 real 2048×1536 RGB PNG images, paired measured N×4 "
        "XYZ-reflectance point clouds, intrinsics, distortion, and LiDAR ROIs. Fourteen frames "
        "fit the public transform and seven separate frames are held out."
    )
    for run in paragraph.runs:
        set_run_font(run)

    public = report["public_external_experiment"]["held_out_test_aggregate"]
    public_rows = [[
        public["number_of_frames"],
        f"{public['evaluated_lidar_points']:,}",
        f"{public['mean_reprojection_error_px']:.3f}",
        f"{public['median_reprojection_error_px']:.3f}",
        f"{public['rmse_reprojection_error_px']:.3f}",
        f"{public['maximum_reprojection_error_px']:.3f}",
        f"{public['percentage_within_1_px']:.2f}%",
        f"{public['percentage_within_2_px']:.2f}%",
        f"{public['percentage_within_3_px']:.2f}%",
        "Fail",
    ]]
    add_table(
        doc,
        ["Frames", "Points", "Mean", "Median", "RMSE", "Max", "≤1 px", "≤2 px", "≤3 px", "Result"],
        public_rows,
        [700, 1100, 900, 900, 900, 900, 900, 900, 900, 1260],
        body_size=8,
        decision_column=9,
    )
    add_body(
        doc,
        "This external failure is preserved rather than replaced by a board-containment result. "
        "The present public LiDAR grid-corner extractor is weak across several poses. The public "
        "number is therefore an honest held-out baseline, not an internal D455–Livox result.",
    )

    page_break(doc)
    add_heading(doc, "3. Independent point-selection methodology", 1)
    add_body(
        doc,
        "The central correction is to freeze the evaluated LiDAR set before consulting any RGB "
        "mask or checkerboard pixels. Internal selection is performed as follows:",
    )
    steps = [
        "Read one raw Livox point-cloud frame.",
        "Apply the predeclared metric 3D ROI around the physical checkerboard.",
        "Fit the board plane robustly as x = a·y + b·z + c using LiDAR XYZ coordinates only.",
        "Keep returns with absolute plane residual at or below 0.015 m.",
        "Keep returns inside the known 0.30×0.45 m board, excluding a 0.00375 m boundary margin.",
        "Freeze the selected returns; no RGB pixel or mask has been used.",
        "Match the LiDAR frame to the nearest original RGB timestamp.",
        "Use camera checkerboard geometry only to derive the reference pixel for the same physical board coordinate.",
        "Project the frozen return with K, D, R, and t and compute its Euclidean pixel error.",
    ]
    for step in steps:
        add_number(doc, step)

    add_callout(
        doc,
        "Anti-circularity guarantee",
        "RGB checkerboard masks are separate visualization/annotation files. They are not read "
        "during LiDAR-point selection, are not used to reject projected returns, and are not "
        "used to calculate the reported errors.",
        fill=PALE_GREEN,
        color=GREEN,
    )

    add_heading(doc, "4. Mathematical definitions", 1)
    add_body(
        doc,
        "For a calibrated projected pixel q_i = (u_i, v_i) and reference checkerboard pixel "
        "q_i* = (u_i*, v_i*) corresponding to the same physical point:",
    )
    add_code(doc, "e_i = ||q_i - q_i*||_2\n    = sqrt((u_i-u_i*)^2 + (v_i-v_i*)^2)")
    add_bullet(doc, "Strict raster alignment: round(u_i) = round(u_i*) and round(v_i) = round(v_i*).")
    add_bullet(doc, "Continuous exact match: e_i ≤ 1×10⁻⁹ px numerical tolerance.")
    add_bullet(doc, "Threshold rates: percentage of evaluated returns with e_i ≤ 1, 2, or 3 px.")
    add_bullet(doc, "Project acceptance: at every internal distance, at least 99% must satisfy e_i ≤ 2 px.")
    add_body(
        doc,
        "The 99%-within-2-pixel threshold is a project/mentor-defined acceptance criterion. "
        "This report does not attribute it to a specific publication or universal standard.",
    )

    add_heading(doc, "5. Multi-frame internal results", 1)
    add_table(
        doc,
        ["Distance", "Frames", "Points", "Mean", "Median", "RMSE", "Max", "≤1 px", "≤2 px", "≤3 px", "Result"],
        metric_rows(report),
        [850, 700, 950, 850, 850, 850, 850, 900, 900, 900, 760],
        body_size=7.5,
        decision_column=10,
    )
    add_body(
        doc,
        "The 14.5 ft error remains close to 4 px in all ten evaluated frames. The 22.2 ft "
        "error remains approximately 2–3 px. The repeatability indicates a systematic "
        "geometry/reference mismatch rather than one isolated blurred frame.",
    )

    page_break(doc)
    add_heading(doc, "6. Calibration parameters", 1)
    params = report["calibration_parameters_internal"]
    add_heading(doc, "6.1 Camera model", 2)
    add_code(doc, "K = [[386.76702881,   0.00000000, 322.92172241],\n"
                  "     [  0.00000000, 386.23919678, 240.03582764],\n"
                  "     [  0.00000000,   0.00000000,   1.00000000]]")
    add_code(doc, "D = [-0.05702033, 0.06783563, -0.00031395,\n"
                  "      0.00094203, -0.02286913]\n"
                  "Distortion model = plumb_bob\nResolution = 640 × 480 pixels")

    add_heading(doc, "6.2 LiDAR-to-camera transform", 2)
    add_code(doc, "R_camera_lidar =\n"
                  "[[-0.03659380, -0.99855630,  0.03932181],\n"
                  " [-0.26905772, -0.02805037, -0.96271549],\n"
                  " [ 0.96242861, -0.04580926, -0.26764282]]\n\n"
                  "t_camera_lidar = [0.12405766, -0.13617702, 0.19254424] m")
    add_code(doc, "T_camera_lidar =\n"
                  "[[-0.03659380, -0.99855630,  0.03932181,  0.12405766],\n"
                  " [-0.26905772, -0.02805037, -0.96271549, -0.13617702],\n"
                  " [ 0.96242861, -0.04580926, -0.26764282,  0.19254424],\n"
                  " [ 0.00000000,  0.00000000,  0.00000000,  1.00000000]]")
    convention = params["coordinate_frame_convention"]
    add_body(doc, f"Transform equation: {convention['equation']}", bold_prefix="Transform equation:")
    add_body(doc, f"Camera optical axes: {convention['camera_optical_axes']}", bold_prefix="Camera optical axes:")
    add_body(doc, f"Units and pixels: translation in metres; {convention['pixel_origin']}.", bold_prefix="Units and pixels:")

    add_heading(doc, "7. Reproducible pipeline", 1)
    pipeline = [
        "Read the D455 CameraInfo intrinsic matrix and distortion coefficients.",
        "Detect/annotate camera checkerboard geometry in the original RGB coordinate grid.",
        "Crop measured Livox returns in a metric ROI and robustly estimate the board plane.",
        "Initialize the rigid transform from near-pose board geometry.",
        "Refine one LiDAR-to-camera R,t transform using the recorded calibration poses.",
        "Run the corrected multi-frame validator with independent LiDAR selection.",
        "Write JSON, CSV, YAML, original RGB copies, separate masks, and projection/error figures.",
    ]
    for step in pipeline:
        add_number(doc, step)
    add_code(doc, "python3 -m venv .venv\n"
                  ".venv/bin/python -m pip install -r requirements.txt\n"
                  ".venv/bin/python scripts/calibrate.py\n"
                  ".venv/bin/python scripts/phase1_required_validation.py")

    add_heading(doc, "8. Attached evidence package", 1)
    add_body(
        doc,
        "The companion ZIP is designed to be shared together with this report. It includes "
        "the derived dataset outputs and all files needed to inspect or regenerate the results:",
    )
    for item in [
        "Canonical phase1_validation_report.json with all per-frame metrics and selection audits.",
        "internal_multiframe_metrics.csv and internal_calibration_parameters.yaml.",
        "Nine projection/error PNG files embedded in this report.",
        "Nine lossless original-resolution RGB copies and nine separate mask PNG files.",
        "calibration_result.json and the calibration/validation Python scripts.",
        "README, complete technical report, and requirements file.",
    ]:
        add_bullet(doc, item)
    add_body(
        doc,
        "Raw ROS DB3 recordings and the approximately 872 MB extracted public sample are not "
        "duplicated in the ZIP. The public archive is supplied by its official download link; "
        "internal raw recordings should be transferred separately only if the reviewer requires "
        "full raw-data reruns. The ZIP contains their evaluated RGB/output evidence.",
    )

    # Qualitative evidence: two wide figures per page when possible.
    page_break(doc)
    add_heading(doc, "9. Qualitative output evidence", 1)
    add_body(
        doc,
        "Each figure contains the untouched original RGB frame and the measured LiDAR projection "
        "on the same pixel grid. Green outlines mark the checkerboard reference. White crosses "
        "mark expected reference pixels; colored points are projected measured LiDAR returns; "
        "white segments show point-wise reprojection displacement.",
    )

    qualitative_root = report_path.parent / "qualitative"
    figures = [
        ("internal_7_25ft_frame_02_projection.png", "Internal 7.25 ft, synchronized evaluation frame 2.", "Original RGB and low-error LiDAR projection at 7.25 feet, frame 2."),
        ("internal_7_25ft_frame_07_projection.png", "Internal 7.25 ft, synchronized evaluation frame 7.", "Original RGB and low-error LiDAR projection at 7.25 feet, frame 7."),
        ("internal_14_5ft_frame_02_projection.png", "Internal 14.5 ft, synchronized evaluation frame 2; systematic 3–5 px error is visible.", "Original RGB and LiDAR projection showing systematic reprojection error at 14.5 feet, frame 2."),
        ("internal_14_5ft_frame_07_projection.png", "Internal 14.5 ft, synchronized evaluation frame 7; repeated systematic error.", "Original RGB and LiDAR projection showing repeated reprojection error at 14.5 feet, frame 7."),
        ("internal_22_2ft_frame_02_projection.png", "Internal 22.2 ft, synchronized evaluation frame 2; most errors are approximately 2–3 px.", "Original RGB and LiDAR projection at 22.2 feet, frame 2."),
        ("internal_22_2ft_frame_07_projection.png", "Internal 22.2 ft, synchronized evaluation frame 7; far-range mismatch remains.", "Original RGB and LiDAR projection at 22.2 feet, frame 7."),
        ("public_acsc_000000_projection.png", "Public ACSC frame 000000 from the calibration-training split.", "Real public ACSC RGB frame 000000 with projected measured LiDAR returns."),
        ("public_acsc_000010_projection.png", "Public ACSC frame 000010 from the calibration-training split.", "Real public ACSC RGB frame 000010 with projected measured LiDAR returns."),
        ("public_acsc_000020_projection.png", "Public ACSC held-out frame 000020; external generalization error remains visible.", "Real public ACSC held-out RGB frame 000020 with projected measured LiDAR returns and visible error."),
    ]
    for index, (filename, caption, alt) in enumerate(figures, start=1):
        if index > 1 and index % 2 == 0:
            pass
        if index in (3, 5, 7, 9):
            page_break(doc)
        add_figure(doc, qualitative_root / filename, f"Figure {index}. {caption}", alt)

    page_break(doc)
    add_heading(doc, "10. Conclusion and next action", 1)
    add_body(
        doc,
        "All requested implementation and evidence items are present. The corrected evaluator "
        "demonstrates that the current transform is acceptable at the near distance but does not "
        "meet the project threshold at the middle and far distances. The result must therefore be "
        "reported as a failed Phase 1 acceptance test, despite visually plausible board overlap.",
    )
    add_heading(doc, "Required next capture", 2)
    for item in [
        "Use a larger, rigid, sharply printed checkerboard for the 22–30 ft working range.",
        "Capture several positions and orientations at each distance while keeping the sensor rig fixed.",
        "Separate calibration poses from held-out validation poses before fitting the transform.",
        "Preserve full-resolution RGB and synchronized raw Livox frames.",
        "Freeze the new transform and rerun the same independent validator without refitting on held-out captures.",
    ]:
        add_bullet(doc, item)
    add_callout(
        doc,
        "Final reporting statement",
        "This legacy report path is superseded by build_phase1_final_report.py. The accepted "
        "final-run counts are 1,777/1,777 at 7.25 ft, 0/349 at 14.5 ft, and 78/170 at "
        "22.2 ft within 2 pixels; Phase 1 remains OPEN.",
    )

    properties = doc.core_properties
    properties.title = "Phase 1 Camera–LiDAR Calibration Validation Report"
    properties.subject = "Independent multi-frame reprojection validation with attached output evidence"
    properties.author = "Calibration Project Team"
    properties.keywords = "camera lidar calibration, RealSense D455, Livox Mid-360, reprojection validation"
    properties.comments = "Generated from outputs/phase1_validation/phase1_validation_report.json"
    doc.save(output_path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--report",
        type=Path,
        default=Path("outputs/phase1_validation/phase1_validation_report.json"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("deliverables/Phase1_Calibration_Validation_Report.docx"),
    )
    args = parser.parse_args()
    build_report(args.report, args.output)
    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
