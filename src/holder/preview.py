"""PNG previews: hidden-line views drawn by ExportSVG, rasterized by rsvg-convert.

Each view is an orthographic projection (visible lines solid black, hidden lines dashed
grey). Views are laid out on one sheet at a common scale, third-angle style: top above
front, right beside front, isometric top right.
"""

import subprocess
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from io import BytesIO
from pathlib import Path

from build123d import Color, Compound, ExportSVG, LineType, Shape, Vector

SVG_NS = "http://www.w3.org/2000/svg"
ET.register_namespace("", SVG_NS)

SHEET_W = 1800  # px, max sheet width
SHEET_H = 1300  # px, max sheet height (both limits keep the same scale for all views)
PAD = 30  # px around and between views
TITLE_H = 80  # px for the title block
LABEL_H = 28  # px above each view for its label
VISIBLE_PX = 2.2  # stroke widths on the PNG
HIDDEN_PX = 1.3
HIDDEN_GREY = Color(0.55, 0.55, 0.55)
FONT = "DejaVu Sans"

# name: (direction from the target toward the viewer, screen up)
VIEWS = {
    "top": ((0, 0, 1), (0, 1, 0)),
    "iso": ((1, -1, 1), (0, 0, 1)),
    "front": ((0, -1, 0), (0, 0, 1)),
    "right": ((1, 0, 0), (0, 0, 1)),
}
VIEW_LABELS = {
    "top": "top (from +Z, back up)",
    "iso": "iso (front right)",
    "front": "front (from -Y)",
    "right": "right (from +X)",
}


@dataclass(frozen=True)
class View:
    label: str
    shape: Shape
    eye: tuple[float, float, float]  # direction from the shape toward the viewer
    up: tuple[float, float, float]


def view(shape: Shape, name: str, label: str | None = None) -> View:
    eye, up = VIEWS[name]
    return View(label or VIEW_LABELS[name], shape, eye, up)


def standard_sheet(shape: Shape, extra: View | None = None) -> list[list[View | None]]:
    """[[top, iso, extra], [front, right]]: top over front, right beside front."""
    top_row: list[View | None] = [view(shape, "top"), view(shape, "iso")]
    if extra is not None:
        top_row.append(extra)
    return [top_row, [view(shape, "front"), view(shape, "right")]]


def _project(v: View):
    box = v.shape.bounding_box()  # only for the camera target: a padded box is fine here
    target = box.center()
    eye = target + Vector(*v.eye).normalized() * (box.diagonal * 2 + 100)
    return v.shape.project_to_viewport(eye, v.up, target)


def _extent(edges) -> tuple[float, float]:
    box = Compound(edges).bounding_box()
    return box.size.X, box.size.Y


def _view_svg(visible, hidden, ppm: float) -> ET.Element:
    """One projected view as an <svg> element in mm (ExportSVG output)."""
    svg = ExportSVG(margin=0, fit_to_stroke=False)
    svg.add_layer(
        "hidden", line_color=HIDDEN_GREY, line_type=LineType.ISO_DASH, line_weight=HIDDEN_PX / ppm
    )
    svg.add_layer("visible", line_weight=VISIBLE_PX / ppm)
    if hidden:
        svg.add_shape(hidden, layer="hidden")
    svg.add_shape(visible, layer="visible")
    buf = BytesIO()
    svg.write(buf)
    return ET.fromstring(buf.getvalue())


def _text(parent, x, y, text, size, weight="normal", anchor="start"):
    el = ET.SubElement(
        parent,
        f"{{{SVG_NS}}}text",
        {
            "x": f"{x:.1f}",
            "y": f"{y:.1f}",
            "font-family": FONT,
            "font-size": str(size),
            "font-weight": weight,
            "text-anchor": anchor,
            "fill": "rgb(20,20,20)",
        },
    )
    el.text = text


def render_sheet(grid: list[list[View | None]], png_path: Path, title: str, subtitle: str = ""):
    """Render a grid of views onto one PNG with a white background."""
    projected = [[_project(v) if v else None for v in row] for row in grid]
    n_cols = max(len(row) for row in grid)
    col_w = [0.0] * n_cols
    row_h = [0.0] * len(grid)
    for r, row in enumerate(projected):
        for c, edges in enumerate(row):
            if edges:
                w, h = _extent(edges[0] + edges[1])
                col_w[c] = max(col_w[c], w)
                row_h[r] = max(row_h[r], h)

    fixed_w = PAD * (n_cols + 1)
    fixed_h = TITLE_H + (PAD + LABEL_H) * len(grid) + PAD
    ppm = min((SHEET_W - fixed_w) / sum(col_w), (SHEET_H - fixed_h) / sum(row_h))
    width = round(fixed_w + sum(col_w) * ppm)
    height = round(fixed_h + sum(row_h) * ppm)

    sheet = ET.Element(
        f"{{{SVG_NS}}}svg",
        {
            "width": str(width),
            "height": str(height),
            "viewBox": f"0 0 {width} {height}",
        },
    )
    ET.SubElement(sheet, f"{{{SVG_NS}}}rect", {"width": "100%", "height": "100%", "fill": "white"})
    _text(sheet, PAD, 38, title, 26, "bold")
    if subtitle:
        _text(sheet, PAD, 66, subtitle, 17)

    y = TITLE_H
    for r, row in enumerate(projected):
        x = PAD
        for c in range(n_cols):
            edges = row[c] if c < len(row) else None
            if edges:
                cell_w, cell_h = col_w[c] * ppm, row_h[r] * ppm
                _text(sheet, x + cell_w / 2, y + LABEL_H - 8, grid[r][c].label, 17, anchor="middle")
                child = _view_svg(edges[0], edges[1], ppm)
                _, _, vb_w, vb_h = (float(s) for s in child.get("viewBox").split())
                w, h = vb_w * ppm, vb_h * ppm
                child.set("x", f"{x + (cell_w - w) / 2:.2f}")
                child.set("y", f"{y + LABEL_H + (cell_h - h) / 2:.2f}")
                child.set("width", f"{w:.2f}")
                child.set("height", f"{h:.2f}")
                child.set("overflow", "visible")  # do not clip strokes on the view outline
                sheet.append(child)
            x += col_w[c] * ppm + PAD
        y += LABEL_H + row_h[r] * ppm + PAD

    png_path.parent.mkdir(parents=True, exist_ok=True)
    svg_bytes = ET.tostring(sheet, encoding="utf-8", xml_declaration=True)
    subprocess.run(
        [
            "rsvg-convert",
            "--background-color",
            "white",
            "--format",
            "png",
            "--output",
            str(png_path),
        ],
        input=svg_bytes,
        check=True,
    )
    return png_path
