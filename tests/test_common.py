"""The shared helpers measure what they claim, on small known shapes."""

import math

import pytest
from build123d import Align, Box, Cylinder, Edge, Pos, Vector

from holder.common import (
    captive_nut,
    cavity_check,
    cavity_pieces,
    circles,
    clash_checks,
    coaxial_check,
    contact_faces,
    cut,
    hex_flat_widths,
    hex_pocket,
    hex_pocket_checks,
    hole_axes,
    intersection_volume,
    material_spans,
    planar_faces,
    rod,
    wall_around,
)
from holder.export import (
    STL_VOLUME_TOL,
    bed_area,
    print_rotation,
    stl_volume,
    to_print,
    write_stl,
)

AF, DEPTH = 5.7, 2.6
HEX_AREA = math.sqrt(3) / 2 * AF**2
BOTTOM = (Align.CENTER, Align.CENTER, Align.MIN)


def ok(checks):
    return all(c.ok for c in checks)


def test_hex_pocket_size_and_placement():
    pocket = hex_pocket(AF, DEPTH, origin=(5, 0, 0), axis=(1, 0, 0))
    assert pocket.volume == pytest.approx(HEX_AREA * DEPTH)
    center = pocket.center()
    assert (center.X, center.Y, center.Z) == pytest.approx((5 + DEPTH / 2, 0, 0), abs=1e-9)


@pytest.mark.parametrize(
    "axis, slot_dir", [((0, 0, 1), (0, 1, 0)), ((1, 0, 0), (0, 0, -1)), ((0, 1, 0), (1, 0, 0))]
)
def test_hex_pocket_slot_runs_along_the_flats(axis, slot_dir):
    # Half hex + an AF-wide channel: this volume only holds if two flats run along the slot.
    pocket = hex_pocket(AF, DEPTH, axis=axis, slot_dir=slot_dir, slot_len=10)
    assert len(pocket.solids()) == 1
    assert pocket.volume == pytest.approx(DEPTH * (HEX_AREA / 2 + AF * 10))
    tip = Vector(*slot_dir) * 9.9 + Vector(*axis) * (DEPTH / 2)
    assert pocket.solids()[0].is_inside(tip)


def test_hex_pocket_rejects_bad_input():
    with pytest.raises(ValueError):
        hex_pocket(AF, DEPTH, slot_len=5)  # slot without a direction
    with pytest.raises(ValueError):
        hex_pocket(AF, DEPTH, axis=(0, 0, 1), slot_dir=(0, 1, 1), slot_len=5)


def plate_with_hole(x: float):
    return Box(20, 20, 4, align=BOTTOM) - rod(3.4, 6, origin=(x, 0, -1))


def test_hole_axes_and_coaxial_check():
    plate = plate_with_hole(5.0)
    axes = hole_axes(plate, 3.4, direction=(0, 0, 1))
    assert len(axes) == 1
    assert hole_axes(plate, 3.4, direction=(1, 0, 0)) == []
    assert hole_axes(plate, 3.0) == []
    assert len(circles(plate, 1.7, direction=(0, 0, 1))) == 2  # top and bottom rim

    above = hole_axes(Pos(0, 0, 10) * plate_with_hole(5.0), 3.4)
    shifted = hole_axes(plate_with_hole(5.2), 3.4)
    assert coaxial_check("same", axes, above, expected=1).ok
    bad = coaxial_check("shifted", axes, shifted)
    assert not bad.ok and "0.2000" in bad.measured
    assert not coaxial_check("none", [], []).ok


def test_wall_around_measures_the_nearest_face():
    box = Box(10, 10, 10)
    void = rod(2, 12, origin=(2, 0, -6)) & box
    assert wall_around(box - void, void) == pytest.approx(2.0)  # to x = 5: 5 - 2 - 1


def nut_block(hole_dx=0.0, af=AF):
    """Block with an M3 hole and a side-slot nut pocket; returns (part, envelope, voids)."""
    envelope = Box(30, 20, 10, align=BOTTOM)
    hole = rod(3.4, 12, origin=(7, 0, -1)) & envelope
    nut = hex_pocket(AF, DEPTH, origin=(7, 0, 3.5), slot_dir=(1, 0, 0), slot_len=9) & envelope
    cut_hole = Pos(hole_dx, 0, 0) * hole
    cut_nut = hex_pocket(af, DEPTH, origin=(7, 0, 3.5), slot_dir=(1, 0, 0), slot_len=9)
    return envelope - cut_hole - cut_nut, envelope, [hole, nut]


def test_wall_around_skips_the_space_it_opens_into():
    # A bore opening into a recess with a ridge 0.5 mm beside the mouth: air, not a wall.
    box = Box(20, 20, 10, align=BOTTOM)
    ridge = Pos(0, 4, 8) * Box(20, 1, 1, align=BOTTOM)
    recess = Pos(0, 0, 8) * Box(20, 20, 2, align=BOTTOM) - ridge
    bore = rod(6, 6, origin=(0, 0, 5)) & box
    part = box - recess - bore
    assert wall_around(part, bore) == pytest.approx(0.5)  # ridge side, measured through air
    assert wall_around(part, bore, open_to=recess) == pytest.approx(5.0)  # to the bed face


def test_material_spans_along_a_line():
    part = Box(10, 10, 10) - rod(3.4, 12, origin=(0, 0, -6))
    spans = material_spans(part, (-10, 0, 0), (1, 0, 0), 20)
    assert [t for span in spans for t in span] == pytest.approx([5, 8.3, 11.7, 15])


def test_contact_faces_need_an_area_contact():
    a = Box(10, 10, 10)
    assert len(contact_faces(a, Pos(10, 0, 0) * Box(10, 10, 10))) == 1
    assert contact_faces(a, Pos(10, 10, 0) * Box(10, 10, 10)) == []  # edge to edge only
    assert len(planar_faces(a, (0, 0, -1))) == 2


def test_wall_around_a_connected_cavity():
    part, _, voids = nut_block()
    # The hole opens into the bed face; the nut pocket floor still counts: 3.5 to Z=0.
    assert wall_around(part, voids) == pytest.approx(3.5)


def test_cavity_check_catches_shifted_plugged_and_oversize():
    part, envelope, voids = nut_block()
    assert cavity_check("good", part, voids, envelope).ok
    shifted, _, _ = nut_block(hole_dx=0.2)
    oversize, _, _ = nut_block(af=AF + 0.2)
    plugged = part + voids[1]
    for bad in (shifted, oversize, plugged):
        assert not cavity_check("bad", bad, voids, envelope).ok


def test_hex_flat_widths_closed_and_slotted():
    block = Box(30, 20, 10, align=BOTTOM)
    closed = block - hex_pocket(AF, DEPTH, origin=(-7, 0, 3.5)) - rod(3.4, 12, origin=(-7, 0, -1))
    part, _, _ = nut_block()
    axis_closed = hole_axes(closed, 3.4)[0]
    axis_slot = hole_axes(part, 3.4)[0]
    assert hex_flat_widths(closed, axis_closed, AF) == pytest.approx([AF] * 3)
    assert hex_flat_widths(part, axis_slot, AF) == pytest.approx([AF])
    oversize, _, _ = nut_block(af=AF + 0.2)
    assert hex_flat_widths(oversize, axis_slot, AF) == pytest.approx([AF + 0.2])
    assert hex_flat_widths(plate_with_hole(5.0), hole_axes(plate_with_hole(5.0), 3.4)[0], AF) == []


def test_hex_pocket_checks_count_and_af():
    part, _, _ = nut_block()
    axes = hole_axes(part, 3.4)
    assert ok(hex_pocket_checks("M3", part, axes, AF, 1))
    assert not ok(hex_pocket_checks("M3", part, axes, AF, 0))
    oversize, _, _ = nut_block(af=AF + 0.2)
    names = {c.name for c in hex_pocket_checks("M3", oversize, axes, AF, 1) if not c.ok}
    assert names == {"M3 nut pocket AF"}


def test_captive_nut_and_cavity_pieces():
    envelope = Box(30, 20, 10, align=BOTTOM)
    hole, pocket = captive_nut(
        (7, 0, 10), (0, 0, -1), 3.4, 10, AF, DEPTH, 3.0, slot_dir=(1, 0, 0), slot_len=9
    )
    assert pocket.solids()[0].is_inside(Vector(7, 0, 10 - 3.0 - DEPTH / 2))  # nut 3 below top
    assert not pocket.solids()[0].is_inside(Vector(7, 0, 10 - 2.9))
    hole_piece, pocket_piece = cavity_pieces(envelope, [hole, pocket])
    assert intersection_volume(hole_piece, pocket_piece) == pytest.approx(0, abs=1e-6)
    assert len(hole_piece.solids()) == 2  # above and below the pocket
    part = cut(envelope, [hole, pocket])
    assert part.volume == pytest.approx(
        envelope.volume - hole_piece.volume - pocket_piece.volume, rel=1e-9
    )


def test_clash_checks():
    a = Box(10, 10, 10)
    touching = Pos(10, 0, 0) * Box(10, 10, 10)
    overlapping = Pos(9, 0, 0) * Box(10, 10, 10)
    apart = Pos(12, 0, 0) * Box(10, 10, 10)
    inside = Box(2, 2, 2)  # boundary distance 4 mm, but fully overlapping
    assert ok(clash_checks({"a": a, "b": touching}, [("a", "b")]))
    assert not ok(clash_checks({"a": a, "b": touching}))  # contact not declared
    assert not ok(clash_checks({"a": a, "b": overlapping}, [("a", "b")]))
    assert ok(clash_checks({"a": a, "b": apart}))
    assert not ok(clash_checks({"a": a, "b": apart}, [("a", "b")]))  # declared, but a gap
    assert not ok(clash_checks({"a": a, "b": inside}))
    assert not ok(clash_checks({"a": a}, [("a", "x")]))  # unknown part


def test_stl_volume_matches_the_solid(tmp_path):
    part = Cylinder(12, 30) - Cylinder(1.7, 40)  # curved inside and out
    path = write_stl(part, tmp_path / "ring.stl")
    assert stl_volume(path) == pytest.approx(part.volume, rel=STL_VOLUME_TOL)


@pytest.mark.parametrize(
    "normal", [(0, 0, -1), (0, 0, 1), (1, 0, 0), (0, -1, 0), (1, 1, 1), (-1, 0.2, 0.3)]
)
def test_print_rotation_turns_the_normal_down(normal):
    line = Edge.make_line((0, 0, 0), Vector(*normal).normalized())
    tip = line.moved(print_rotation(normal)).position_at(1)
    assert (tip.X, tip.Y, tip.Z) == pytest.approx((0, 0, -1), abs=1e-9)


@pytest.mark.parametrize(
    "normal, area",
    [((0, 0, -1), 30 * 20), ((0, 0, 1), 30 * 20), ((1, 0, 0), 20 * 10), ((0, -1, 0), 30 * 10)],
)
def test_to_print_puts_the_bed_face_on_z0(normal, area):
    box = Box(30, 20, 10)
    printed = to_print(box, normal)
    assert bed_area(printed) == pytest.approx(area)
    assert printed.bounding_box(optimal=True).min.Z == pytest.approx(0, abs=1e-6)
    assert printed.volume == pytest.approx(box.volume)
