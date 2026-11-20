"""Shared tools, each with a case that must fail."""

from math import sqrt

import pytest
from build123d import Box, CenterArc, Line, Plane, Pos, Vector, Wire, chamfer

from holder.common import (
    SeatedInsert,
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
    Turtle,
    curvature_jumps,
    edge_curvature,
    g2_blend,
    g2_skin,
    skin_profile,
    soft_outline,
    soft_rect,
)
from holder.params import INSERT_M6, SHOULDER_M6, SPRING_B16, WASHER_M8, Params
from holder.stops import Placed, StopSpec


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


def _turtle() -> Turtle:
    path = Turtle((0, 0), 0.0)
    path.line(10).ellipse(8, 4, 90).arc(6, 90).line(10)
    return path


def test_turtle_soft_is_g2_and_keeps_its_conics() -> None:
    path = _turtle()
    hard = Wire([segment.edge() for segment in path.segments])
    assert max(curvature_jumps(hard)) > 0.1  # the raw G1 path is not G2
    soft = Wire(path.soft(1.0))
    assert max(curvature_jumps(soft)) <= 1e-6
    kinds = {edge.geom_type.name for edge in soft.edges()}
    assert {"ELLIPSE", "CIRCLE", "LINE"} <= kinds


def test_turtle_ellipse_vertex_curvature_and_closure() -> None:
    path = Turtle((0, 0), 0.0)
    path.ellipse(8, 4, 90)
    segment = path.segments[0]
    assert segment.frame(0.0).curvature == pytest.approx(4 / 8 ** 2, rel=1e-9)
    assert segment.frame(1.0).curvature == pytest.approx(8 / 4 ** 2, rel=1e-9)
    with pytest.raises(ValueError):
        _turtle().soft(1.0, closed=True)  # it does not end where it started
    with pytest.raises(ValueError):
        Turtle((0, 0), 0.0).ellipse(8, 4, 120)


def _nut_block() -> Box:
    return box_from(-20, -20, 0, 20, 20, 30)


def test_insert_hole_must_be_blind_and_walled() -> None:
    insert = SeatedInsert("t", INSERT_M6, Vector(0, 0, 30), Vector(0, 0, -1))
    block = _nut_block()
    for cutter in insert.cutters():
        block -= cutter
    assert all(row.ok for row in insert.checks(block))
    through = block - box_from(-1, -1, -1, 1, 1, 31)  # opens the floor
    assert not insert.checks(through)[1].ok
    thin = block - box_from(4.5, -2, 10, 9, 2, 25)  # into the 3.3 mm maker wall
    assert not insert.checks(thin)[0].ok


def _stop(l1: float) -> StopSpec:
    return StopSpec("t", SHOULDER_M6, SPRING_B16, 5, WASHER_M8, WASHER_M8, INSERT_M6, 0.6, l1)


def test_spring_force_follows_the_published_table() -> None:
    assert SPRING_B16.force(0.75 * SPRING_B16.h0) == pytest.approx(410.0)
    assert SPRING_B16.force(0.625 * SPRING_B16.h0) == pytest.approx(357.0)
    with pytest.raises(ValueError):
        SPRING_B16.force(1.2 * SPRING_B16.h0)  # past flat


def test_stop_force_is_set_by_geometry_and_a_short_shoulder_goes_flat() -> None:
    stop = _stop(20.0)
    low, nominal, high = stop.band(stop.clamped, 0.15)
    assert low < nominal < high
    assert nominal == pytest.approx(SPRING_B16.force(0.6 * SPRING_B16.h0))
    with pytest.raises(ValueError):
        stop.force(stop.clamped + 2.0, stop.l1)  # 2 mm more PETG: the springs go flat


def test_shoulder_must_stop_on_brass() -> None:
    stop = Placed(_stop(20.0), Vector(0, 0, 40), Vector(0, 0, -1))
    seated = stop.seated_insert()
    far = _nut_block()
    for cutter in seated.cutters():
        far -= cutter
    assert stop.shoulder_on_brass(far).ok
    plugged = far + Pos(0, 0, seated.top.Z + 0.1) * Box(20, 20, 0.2)  # PETG over the insert
    assert not stop.shoulder_on_brass(plugged).ok


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


def test_finished_part_rows_catch_a_fused_crease_and_skip_cut_features() -> None:
    base = g2_skin(soft_rect(40, 40, 12).outer_wire(), 10, Roll(2.0), Roll(2.0))
    post = Pos(0, 0, 8) * g2_skin(soft_rect(16, 16, 6).outer_wire(), 20, Roll(2.0), Roll(2.0))
    fused = base + post
    on_bed = lambda _c: False  # noqa: E731
    assert not all(row.ok for row in skin_rows(fused, "fused", on_bed, fused))
    drilled = base - Pos(0, 0, -1) * Box(6, 6, 30)
    assert all(row.ok for row in skin_rows(drilled, "drilled", on_bed, base))


def test_skin_profile_joins_are_g2_and_a_roll_too_big_is_refused() -> None:
    profile = skin_profile(20, Roll(4), BedLead(1.0, 3.0), 0.6)
    assert max(curvature_jumps(profile)) <= 1e-6
    with pytest.raises(ValueError):
        g2_skin(soft_rect(40, 40, 3).outer_wire(), 10, Roll(4), Roll(4))
    with pytest.raises(ValueError):
        skin_profile(6, Roll(4), Roll(4))


def test_bearing_and_insert_rows_fail_when_overloaded() -> None:
    assert bearing_row("t", 300, 145.6).ok
    assert not bearing_row("t", 700, 145.6).ok
    assert insert_shear_row("t", INSERT_M6, 300).ok
    assert not insert_shear_row("t", INSERT_M6, 450).ok


def test_first_hit_reads_the_face() -> None:
    assert first_hit(Box(10, 10, 10), Vector(-20, 0.3, 0.2), Vector(1, 0, 0)) == pytest.approx(15)


@pytest.mark.parametrize("field, value", [("column_w", float("nan")), ("collar_h", -1.0),
                                          ("liner_t", 1.0), ("hero_angles", (70.0, 120.0, 0.0)),
                                          ("rosette_r_in", 6.0)])
def test_params_reject_bad_values(field: str, value: object) -> None:
    with pytest.raises(ValueError):
        Params(**{field: value})


def test_hex_prism_has_a_corner_on_local_x() -> None:
    hexagon = hex_prism(5.7, 1.0, Plane.XY)
    assert hexagon.bounding_box().size.X == pytest.approx(2 * 5.7 / sqrt(3))
    assert hexagon.bounding_box().size.Y == pytest.approx(5.7)
