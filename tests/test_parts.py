"""Part checks and export pipeline."""

from pathlib import Path

from build123d import Box, import_step

from holder.__main__ import placeholder_box
from holder.common import hex_pocket
from holder.export import export_part
from holder.preview import preview_part


def test_placeholder_box_exports_step_stl_and_png(tmp_path: Path) -> None:
    part = placeholder_box()
    step, stl = export_part(part, tmp_path / "placeholder")
    png = tmp_path / "placeholder.png"
    preview_part(part, png)

    assert step.is_file() and step.stat().st_size > 0
    assert stl.is_file() and stl.stat().st_size > 0
    assert png.is_file() and png.stat().st_size > 0
    brought_back = import_step(step)
    assert abs(brought_back.volume - part.volume) / part.volume < 0.005


def test_hex_pocket_is_a_void() -> None:
    # Box is centered on the origin, so its top face is at Z = 5.
    body = Box(20, 20, 10)
    pocket = hex_pocket(5.7, 2.6, at=(0, 0, 5 - 2.6))
    cut = body - pocket
    assert cut.volume < body.volume - 1.0
    left = cut & pocket
    assert left is None or left.volume <= 1e-6
