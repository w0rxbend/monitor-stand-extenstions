"""PNG previews via ExportSVG and rsvg-convert.

Copied from Tigerbee 7inch accessories/_export.py.
"""

import subprocess
from pathlib import Path

from build123d import Compound, ExportSVG, Part, Vector

RSVG = "/usr/bin/rsvg-convert"
ISO = Vector(1, -1, 0.75)


def _project(shape: Part, view: str):
    center = shape.center()
    if view == "top":
        eye, up = Vector(0, 0, 1000), (0, 1, 0)
    elif view == "front":
        eye, up = Vector(0, 1000, 0), (0, 0, 1)
    else:
        eye, up = ISO * 1000, (0, 0, 1)
    return shape.project_to_viewport(center + eye, viewport_up=up, look_at=center)[0]


def render(
    layers: list[tuple[list, tuple[int, int, int], float]],
    png: Path,
    width: int = 1200,
) -> None:
    """layers: [(edges, rgb, line_weight)] already projected into one common viewport."""
    edges = [edge for lay in layers for edge in lay[0]]
    if not edges:
        raise RuntimeError(f"preview has no edges: {png}")
    bounds = Compound(edges).bounding_box()
    scale = 240 / max(bounds.size.X, bounds.size.Y, 1e-3)
    svg = ExportSVG(scale=scale, margin=4, line_weight=0.4)
    for index, (lay, rgb, weight) in enumerate(layers):
        if lay:
            svg.add_layer(f"l{index}", line_color=rgb, line_weight=weight)
            svg.add_shape(lay, f"l{index}")
    svg_path = png.with_suffix(".svg")
    svg.write(svg_path)
    subprocess.run(
        [RSVG, "-w", str(width), "-o", str(png), str(svg_path)],
        check=True,
        capture_output=True,
    )
    svg_path.unlink()


def preview_part(part: Part, png: Path) -> None:
    png.parent.mkdir(parents=True, exist_ok=True)
    render([(_project(part, "iso"), (30, 30, 30), 0.4)], png)
