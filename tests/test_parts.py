"""Part checks and export pipeline."""

from pathlib import Path

import pytest
from build123d import Box, import_step

from holder import band, block
from holder.__main__ import placeholder_box
from holder.common import box_from, hex_pocket, print_orientation, y_cylinder
from holder.coupon import (
    COUPON_BOLT_Z,
    build_back,
    build_front,
    build_strip,
    checks_back,
    checks_front,
    checks_pair,
    checks_strip,
    m3_hex_tool,
    quarter_hex_tool,
)
from holder.export import export_part
from holder.params import Params
from holder.preview import preview_part


def test_placeholder_box_exports_step_stl_3mf_and_png(tmp_path: Path) -> None:
    part = placeholder_box()
    step, stl, mesh = export_part(part, tmp_path / "placeholder")
    png = tmp_path / "placeholder.png"
    preview_part(part, png)

    assert step.is_file() and step.stat().st_size > 0
    assert stl.is_file() and stl.stat().st_size > 0
    assert mesh.suffix == ".3mf" and mesh.read_bytes()[:2] == b"PK"
    assert png.is_file() and png.stat().st_size > 0
    brought_back = import_step(step)
    assert abs(brought_back.volume - part.volume) / part.volume < 0.005


def test_params_reject_non_finite_non_positive_and_thin_wall() -> None:
    with pytest.raises(ValueError):
        Params(column_w=float("nan"))
    with pytest.raises(ValueError):
        Params(band_h=-1)
    with pytest.raises(ValueError):
        Params(band_wall=1.0)


def test_coupon_checks_pass(tmp_path: Path) -> None:
    spec = Params()
    front, back, strip = build_front(spec), build_back(spec), build_strip(spec)
    for part, check in (
        (front, checks_front),
        (back, checks_back),
        (strip, checks_strip),
    ):
        failed = [(name, detail) for name, ok, detail in check(part, spec) if not ok]
        assert not failed, failed
        step, _stl, _mesh = export_part(part, tmp_path / part.label)
        brought_back = import_step(step)
        drift = abs(brought_back.volume - part.volume) / part.volume
        assert drift < 0.005
    failed = [(name, detail) for name, ok, detail in checks_pair(front, back, spec) if not ok]
    assert not failed, failed


def _failed(results: list[tuple[str, bool, str]], needle: str) -> bool:
    return any(not ok and needle in name for name, ok, _detail in results)


def test_shifted_seam_hole_fails_checks() -> None:
    spec = Params()
    front = build_front(spec)
    x, z = band.seam_axes(spec, (COUPON_BOLT_Z,))[0]
    y0, y1 = band.ear_outer_ys(spec)
    plug = band.seam_hole(spec, x, z, y0 - 1, y1 + 1)
    moved = band.seam_hole(spec, x, z + 15, y0 - 1, y1 + 1)
    shifted = front + plug - moved
    assert _failed(checks_front(shifted, spec), "seam through-hole open")


def test_plugged_m3_pocket_fails_checks() -> None:
    spec = Params()
    plugged = build_strip(spec) + m3_hex_tool(spec)
    assert _failed(checks_strip(plugged, spec), "m3 pocket open")


def test_oversize_quarter_pocket_fails_checks() -> None:
    spec = Params()
    bigger = quarter_hex_tool(spec, af=spec.quarter_hex_af + 1.5)
    opened = build_strip(spec) - bigger
    assert _failed(checks_strip(opened, spec), "quarter pocket wall")


@pytest.fixture(scope="module")
def band_halves() -> tuple[Params, object, object]:
    spec = Params()
    front, back = band.build_halves(spec)
    return spec, front, back


def test_band_checks_pass(band_halves: tuple, tmp_path: Path) -> None:
    spec, front, back = band_halves
    for part, check in ((front, band.checks_front), (back, band.checks_back)):
        failed = [(name, detail) for name, ok, detail in check(part, spec) if not ok]
        assert not failed, failed
        step, _stl, _mesh = export_part(part, tmp_path / part.label)
        brought_back = import_step(step)
        drift = abs(brought_back.volume - part.volume) / part.volume
        assert drift < 0.005
    pair = band.checks_pair(front, back, spec)
    failed = [(name, detail) for name, ok, detail in pair if not ok]
    assert not failed, failed


def test_plugged_boss_hole_fails_checks(band_halves: tuple) -> None:
    spec, _front, back = band_halves
    x, z = band.boss_hole_centers(spec)[0]
    plug = y_cylinder(x, z, spec.column_d / 2 - 0.4, band.boss_face_y(spec) + 0.2, spec.m3_through)
    assert _failed(band.checks_back(back + plug, spec), "boss hole open")


def test_block_checks_pass(band_halves: tuple, tmp_path: Path) -> None:
    spec, front, back = band_halves
    part = block.build(spec)
    failed = [(name, detail) for name, ok, detail in block.checks(part, spec) if not ok]
    assert not failed, failed
    failed = [
        (name, detail)
        for name, ok, detail in block.checks_vs_band(part, front, back, spec)
        if not ok
    ]
    assert not failed, failed
    assert block.PRINT["block"] == (-1, 0, 0)
    printed = print_orientation(part, block.PRINT["block"])
    assert printed.label == part.label
    assert abs(printed.bounding_box().min.Z) < 1e-4
    assert printed.bounding_box().size.Z == pytest.approx(spec.block_size, abs=0.05)
    step, _stl, _mesh = export_part(printed, tmp_path / part.label)
    brought_back = import_step(step)
    assert abs(brought_back.volume - part.volume) / part.volume < 0.005


def test_bridged_top_pockets_fail_the_wall_check() -> None:
    spec = Params()
    part = block.build(spec)
    m3_xy, quarter_xy = block.top_hole_xy(spec)
    x, y = m3_xy[0]
    qx, qy = quarter_xy
    top = block.riser_top(spec)
    bridge = box_from(
        min(qx, x) - 1, min(qy, y) - 1, top - 18, max(qx, x) + 1, max(qy, y) + 1, top - 6
    )
    assert _failed(block.checks(part - bridge, spec), "wall between pockets")


def test_hex_pocket_is_a_void() -> None:
    # Box is centered on the origin, so its top face is at Z = 5.
    body = Box(20, 20, 10)
    pocket = hex_pocket(5.7, 2.6, at=(0, 0, 5 - 2.6))
    cut = body - pocket
    assert cut.volume < body.volume - 1.0
    left = cut & pocket
    assert left is None or left.volume <= 1e-6
