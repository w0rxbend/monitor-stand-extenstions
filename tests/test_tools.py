"""Shared tools, each with a case that must fail."""

from math import sqrt

import pytest
from build123d import Box, CenterArc, Line, Plane, Pos, Vector, Wire, chamfer

from holder.common import (
    CaptiveNut,
    Stack,
    bearing_row,
    box_from,
    clash_rows,
    first_hit,
    hex_prism,
    insert_shear_row,
    overhang_rows,
    section_stress,
    skin_rows,
)
from holder.form import (
    BedLead,
    Frame,
    Roll,
    curvature_jumps,
    edge_curvature,
    g2_blend,
    g2_skin,
    skin_profile,
    soft_outline,
    soft_rect,
)
from holder.params import M3_INSERT, M3_SHCS, QUARTER_DROP, QUARTER_NUT, Params
from holder.stacks import rosette_preload_need


def test_soft_outline_is_g2() -> None:
    wire = soft_outline([(20, -20), (20, 20), (-20, 20), (-20, -20)], [12, 12, 12, 12])
    assert max(curvature_jumps(wire)) <= 1e-6


def test_tangent_arc_and_line_are_not_g2() -> None:
    arc = CenterArc((0, 0), 10, 0, 90)
    wire = Wire([arc, Line(arc @ 1, (-20, 10))])
    assert max(curvature_jumps(wire)) == pytest.approx(0.1, abs=1e-6)


def test_g2_blend_matches_circle_curvature() -> None:
    radius = 14.5
    start = Frame(Vector(20, 10), Vector(-1, 0), Vector(0, 1), 0.0)
    end = Frame(Vector(radius * 0.6, radius * 0.8), Vector(-0.8, 0.6), Vector(-0.6, -0.8),
                1 / radius)
    blend = g2_blend(start, end, 3.0, 3.0)
    assert edge_curvature(blend, end.point) == pytest.approx(1 / radius, rel=1e-6)
    assert edge_curvature(blend, start.point) == pytest.approx(0.0, abs=1e-9)


def _nut_block(rotate_corner: bool, drop: float = QUARTER_DROP) -> tuple[CaptiveNut, Box]:
    nut = CaptiveNut("test", QUARTER_NUT, Vector(0, 0, 16), Vector(0, 0, -1), Vector(0, 1, 0),
                     20, drop=drop)
    block = box_from(-20, -20, 0, 20, 19, 22)
    if rotate_corner:
        # The old hex_pocket: corners on +-X, an AF-wide slot along Y.
        old = Plane(origin=(0, 0, 16), x_dir=(1, 0, 0), z_dir=(0, 0, -1))
        af, height = QUARTER_NUT.pocket_af, QUARTER_NUT.pocket_m
        block -= hex_prism(af, height + drop, old)
        block -= box_from(-af / 2, 0, 16 - height, af / 2, 20, 16)
    else:
        for cutter in nut.cutters():
            block -= cutter
    return nut, block


def test_nut_travels_in_a_corner_on_travel_channel() -> None:
    nut, block = _nut_block(rotate_corner=False)
    assert all(row.ok for row in nut.checks(block))


def test_nut_without_a_drop_slides_back_to_the_mouth() -> None:
    nut, block = _nut_block(rotate_corner=False, drop=0.0)
    play = [row for row in nut.checks(block) if "play" in row.name]
    assert play and not play[0].ok


def test_old_corner_on_x_pocket_fails_nut_travel() -> None:
    nut, block = _nut_block(rotate_corner=True)
    travel = nut.checks(block)[0]
    assert not travel.ok


def test_stack_rejects_a_tip_past_the_bore() -> None:
    stack = Stack("test", M3_SHCS, 6.7, 6.7, 3.4, 4.0)
    assert stack.length == 12
    assert not all(row.ok for row in stack.rows())


def test_clash_rows_flag_overlap_unless_intended() -> None:
    solids = {"a": Box(10, 10, 10), "b": Pos(5, 0, 0) * Box(10, 10, 10)}
    assert not clash_rows(solids, set(), "t")[0].ok
    assert clash_rows(solids, {frozenset(("a", "b"))}, "t")[0].ok


def _ceiling(width: float):
    return box_from(-20, -20, 0, 20, 20, 10) - box_from(-width / 2, -20, 0, width / 2, 20, 6)


def test_short_bridge_passes_and_wide_ceiling_fails() -> None:
    assert overhang_rows(_ceiling(8.0), "t")[0].ok
    assert not overhang_rows(_ceiling(12.0), "t")[0].ok


def test_45_degree_chamfer_passes_and_flatter_slope_fails() -> None:
    block = Pos(0, 0, 10) * Box(20, 20, 20)
    under = block.edges().group_by()[0]
    assert overhang_rows(chamfer(under, 3.0), "t")[0].ok
    # 5 mm along the bed face, 3 mm up the side: 31 degrees from horizontal.
    flat = chamfer(under, 5.0, 3.0, reference=block.faces().sort_by()[0])
    assert not overhang_rows(flat, "t")[0].ok


def test_section_stress_of_a_rectangle_bends_about_both_axes() -> None:
    bar = box_from(-5, -10, 0, 5, 10, 50)
    plane = Plane.XY.offset(25)
    about_x = section_stress(bar, plane, Vector(1000, 0, 0))
    assert about_x.bending == pytest.approx(1000 / (10 * 20**2 / 6), rel=1e-6)
    about_y = section_stress(bar, plane, Vector(0, 1000, 0))
    assert about_y.bending == pytest.approx(1000 / (20 * 10**2 / 6), rel=1e-6)
    both = section_stress(bar, plane, Vector(1000, 1000, 0))
    assert both.bending == pytest.approx(about_x.bending + about_y.bending, rel=1e-6)
    twisted = section_stress(bar, plane, Vector(0, 0, 1000))
    assert twisted.torsion == pytest.approx(1000 / (0.208 * 20 * 10**2), rel=1e-6)


def test_g2_skin_is_g2_and_a_chamfered_box_is_not() -> None:
    outline = soft_rect(40, 40, 12).outer_wire()
    skin = g2_skin(outline, 10, BedLead(0.6, 2.0), Roll(2.5), 0.3)
    assert all(row.ok for row in skin_rows(skin, "skin", lambda c: abs(c.Z) < 1e-6))
    block = Pos(0, 0, 5) * Box(40, 40, 10)
    chamfered = chamfer(block.edges(), 1.0)
    assert not all(row.ok for row in skin_rows(chamfered, "box", lambda _c: False))


def test_skin_profile_joins_are_g2_and_a_roll_too_big_is_refused() -> None:
    profile = skin_profile(20, Roll(4), BedLead(1.0, 3.0), 0.6)
    assert max(curvature_jumps(profile)) <= 1e-6
    with pytest.raises(ValueError):
        g2_skin(soft_rect(40, 40, 3).outer_wire(), 10, Roll(4), Roll(4))
    with pytest.raises(ValueError):
        skin_profile(6, Roll(4), Roll(4))


def test_bearing_and_insert_rows_fail_when_overloaded() -> None:
    assert bearing_row("t", 80, 29.4).ok
    assert not bearing_row("t", 200, 29.4).ok
    assert insert_shear_row("t", M3_INSERT, 80).ok
    assert not insert_shear_row("t", M3_INSERT, 200).ok


def test_rosette_preload_need_grows_with_the_moment() -> None:
    spec = Params()
    assert rosette_preload_need(spec, 3700) == pytest.approx(274, rel=0.01)
    assert rosette_preload_need(spec, 7400) == pytest.approx(548, rel=0.01)


def test_first_hit_reads_the_face() -> None:
    assert first_hit(Box(10, 10, 10), Vector(-20, 0.3, 0.2), Vector(1, 0, 0)) == pytest.approx(15)


@pytest.mark.parametrize("field, value", [("column_w", float("nan")), ("collar_h", -1.0),
                                          ("liner_t", 1.0), ("hero_angles", (70.0, 120.0, 0.0))])
def test_params_reject_bad_values(field: str, value: object) -> None:
    with pytest.raises(ValueError):
        Params(**{field: value})


def test_hex_prism_has_a_corner_on_local_x() -> None:
    hexagon = hex_prism(5.7, 1.0, Plane.XY)
    assert hexagon.bounding_box().size.X == pytest.approx(2 * 5.7 / sqrt(3))
    assert hexagon.bounding_box().size.Y == pytest.approx(5.7)
