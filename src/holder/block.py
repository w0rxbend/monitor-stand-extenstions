"""PETG L-block: plate, arm and riser (PLAN.md "Design" 3, Task 4; sketches 1-5).

Assembly coordinates (Z up, +Y back, origin on the column axis at the band bottom). The block
is one L profile in the Y-Z plane, block_size wide in X, centered on x = 0 like the boss:

    arm      block_size x block_size (X x Z) bar from the boss face back (+Y), z level with the
             boss (0 ... 32). Its front end is the plate: the face bolted to the boss.
    riser    block_size x block_size (X x Y) column at the back end of the arm. The arm runs
             arm_length from the boss face to the riser's front face; the riser stands
             riser_height above the arm top. Inner L corner rounded with fillet_r.
    top face the riser top = extension point (sketch 4): 1/4"-20 in the center, 4 x M3 on an
             m3_spacing square around it.

Every bolt threads into a captive nut in a side-slot pocket (common.captive_nut): the hole
enters a face, the nut sits a set depth below that face, and a slot as wide as the nut's
across-flats runs to a side face so the nut slides in. Two flats run along the slot.

    plate    4 x M3 coaxial with the boss holes (band.boss_bolts). Nut plate_nut_wall behind
             the boss face (the PETG wall that takes the clamp load), blind hole
             plate_hole_extra past the nut.
    top      1/4" nut top_quarter_nut_depth (2.5) below the top face: a thin top wall, so a
             short 1/4"-20 accessory screw (about 6 mm of thread) reaches the nut. The screw
             pulls the nut up against that wall, which the accessory presses down on: the
             wall only carries compression. M3 nuts top_m3_nut_depth (10) below the top
             face, under the 1/4" pocket: two depths, so the pockets never cut into each
             other. An M3 accessory bolt needs 10 + 2.4 mm (to the nut's far face) plus the
             accessory's thickness under its head, e.g. M3x16 for up to 3.6 mm.

Loads (the user's side view): the monitor hangs in front (-Y) of the column on a carriage;
the band clamps the column just under its top end, above that carriage; the block runs back
(+Y) and the riser stands behind the column. The top face will carry a 2-DOF arm (a yaw ring
plus a pitch hinge) leaning forward over the monitor, so it sees a forward bending moment
(about X) and a yaw torque (about Z).

Print: on its -X side (PRINT = (-1, 0, 0)), PLAN.md "Print". Why this side:
- The L profile lies flat on the bed, so every layer is a full L: the arm-to-riser corner and
  its fillet are inside each layer, and the forward bending moment from the arm on the top
  face (about X) acts in the plane of the layers instead of peeling them apart.
- Assembly +X is up on the bed. The 1/4" slot (the widest pocket) opens on the +X face, as do
  the slots of every nut at x > 0: each is an open channel from the top, no roof at all.
  The nuts at x < 0 slot out of the -X face, the bed: their channel opens onto the bed and
  the closed end is a hex corner pointing up (a 60 degree peak over a 2.85 mm half-span,
  self-supporting like the seam nut pockets of the band). Every slot is 7 mm long (6 mm to
  the face + 1 mm overshoot); no slot crosses another pocket.
- All holes (plate holes along Y, top holes along Z) lie horizontal on the bed. They print
  about 0.15 mm small; the 3.4 / 6.6 through sizes cover that (PLAN.md).

Hardware (user decision after Task 4): 4 x M3x16 ISO 7380 + nut, boss to block. The stack
is measured on the solids (boss_hardware_checks): head on the recess floor in the band,
through the TPU boss to its face, through the PETG wall to the nut seat, and the bolt must
reach through the whole nut. The top face takes 4 x M3 nut + 1 x 1/4"-20 nut; a short
1/4"-20 screw must engage its nut (quarter_screw_check).
"""

from functools import cache

from build123d import Align, Axis, Box, Compound, Part, Pos, fillet

from holder import band
from holder.common import (
    TOUCH_EPS,
    Check,
    PartSpec,
    at_least,
    captive_nut,
    cavity_check,
    cavity_pieces,
    circle_axes,
    clash_checks,
    coaxial_check,
    count_is,
    cut,
    hex_pocket_checks,
    hole_axes,
    material_spans,
    min_distance,
    single_solid,
    wall_around,
)
from holder.params import MATERIALS, PETG, Params

NAME = "block"
MATERIAL = PETG
PRINT = (-1, 0, 0)  # on its -X side: the L profile flat on the bed (see the docstring)
ASSEMBLY = "holder"

PLATE_AXIS = (0, 1, 0)  # plate bolts run +Y: from the band's head recesses into the plate
TOP_AXIS = (0, 0, -1)  # top-face bolts run down from the top face


# --- Envelope ------------------------------------------------------------------------------------


def front_y(p: Params) -> float:
    """y of the plate face: it sits on the boss face."""
    return band.boss_face_y(p)


def bottom_z(p: Params) -> float:
    """z of the arm bottom: the arm is centered on the boss face (flush with the band bottom)."""
    return band.boss_center_z(p) - p.block_size / 2


def riser_front_y(p: Params) -> float:
    return front_y(p) + p.arm_length


def arm_top_z(p: Params) -> float:
    return bottom_z(p) + p.block_size


def top_z(p: Params) -> float:
    """z of the top face (the extension point)."""
    return arm_top_z(p) + p.riser_height


def top_center(p: Params) -> tuple[float, float, float]:
    return (0.0, riser_front_y(p) + p.block_size / 2, top_z(p))


def envelope(p: Params) -> Part:
    """The L before any hole: arm + riser, inner corner rounded with fillet_r."""
    s, y0, z0 = p.block_size, front_y(p), bottom_z(p)
    low = (Align.MIN,) * 3
    arm = Pos(-s / 2, y0, z0) * Box(s, p.arm_length + s, s, align=low)
    riser = Pos(-s / 2, riser_front_y(p), z0) * Box(s, s, s + p.riser_height, align=low)
    body = arm + riser
    corner = [
        e
        for e in body.edges().filter_by(Axis.X)
        if abs(e.center().Y - riser_front_y(p)) <= TOUCH_EPS
        and abs(e.center().Z - arm_top_z(p)) <= TOUCH_EPS
    ]
    if len(corner) != 1:
        raise RuntimeError(f"block: expected 1 inner corner edge, found {len(corner)}")
    return fillet(corner, p.fillet_r)


# --- Bolts and cutters ---------------------------------------------------------------------------


def slot(p: Params, x: float) -> tuple[tuple[float, float, float], float]:
    """(slot_dir, slot_len) for a nut at `x`: to the nearer X face, 1 mm past it.
    A nut on the axis (the 1/4") slots out of +X, the side that is up when printed."""
    side = 1.0 if x >= 0 else -1.0
    return (side, 0.0, 0.0), p.block_size / 2 - abs(x) + 1


def plate_bolts(p: Params) -> list[tuple[float, float]]:
    """(x, z) of the plate bolts: the boss bolts, so the axes are coaxial by design."""
    return band.boss_bolts(p)


def plate_axis(x: float, z: float) -> Axis:
    return Axis((x, 0, z), PLATE_AXIS)


def top_m3_points(p: Params) -> list[tuple[float, float]]:
    """(x, y) of the 4 top-face M3: an m3_spacing square around the top center."""
    s = p.m3_spacing / 2
    _, yc, _ = top_center(p)
    return [(x, yc + dy) for dy in (-s, s) for x in (-s, s)]


def top_axis(x: float, y: float, p: Params) -> Axis:
    return Axis((x, y, top_z(p)), TOP_AXIS)


def plate_cutters(p: Params, x: float, z: float) -> list[Part]:
    """[blind M3 hole, nut pocket] for one plate bolt."""
    slot_dir, slot_len = slot(p, x)
    depth = p.plate_nut_wall + p.m3_hex_h + p.plate_hole_extra
    return captive_nut(
        (x, front_y(p), z),
        PLATE_AXIS,
        p.m3_through,
        depth - 1,  # captive_nut overshoots 1 mm at both ends: the hole ends `depth` deep
        p.m3_hex_af,
        p.m3_hex_h,
        p.plate_nut_wall,
        slot_dir,
        slot_len,
    )


def top_cutters(p: Params, x: float, y: float, quarter: bool) -> list[Part]:
    """[blind hole, nut pocket] for one top-face bolt (M3, or the 1/4"-20 if `quarter`)."""
    if quarter:
        hole, af, nut_h, nut_depth = (
            p.quarter_through,
            p.quarter_hex_af,
            p.quarter_hex_h,
            p.top_quarter_nut_depth,
        )
        extra = p.top_quarter_hole_extra
    else:
        hole, af, nut_h, nut_depth = p.m3_through, p.m3_hex_af, p.m3_hex_h, p.top_m3_nut_depth
        extra = p.top_m3_hole_extra
    slot_dir, slot_len = slot(p, x)
    depth = nut_depth + nut_h + extra
    return captive_nut(
        (x, y, top_z(p)), TOP_AXIS, hole, depth - 1, af, nut_h, nut_depth, slot_dir, slot_len
    )


def cutter_groups(p: Params) -> list[tuple[str, list[Part]]]:
    """[(label, [hole, nut pocket])] for every bolt: plate M3, top M3, top 1/4"."""
    groups = [(f"plate M3 x={x:g} z={z:g}", plate_cutters(p, x, z)) for x, z in plate_bolts(p)]
    groups += [
        (f"top M3 x={x:g} y={y:g}", top_cutters(p, x, y, quarter=False))
        for x, y in top_m3_points(p)
    ]
    _, yc, _ = top_center(p)
    groups.append(("top 1/4", top_cutters(p, 0.0, yc, quarter=True)))
    return groups


def build(p: Params) -> Part:
    """The block in assembly coordinates."""
    return cut(envelope(p), [c for _, cs in cutter_groups(p) for c in cs])


# --- Checks --------------------------------------------------------------------------------------


@cache
def band_parts(p: Params) -> dict[str, Part]:
    """The band halves and the column, which the block is checked against."""
    return {
        band.FRONT_NAME: band.build_front(p),
        band.BACK_NAME: band.build_back(p),
        band.COLUMN_NAME: band.column(p, p.band_h),
    }


def boss_holes(back: Part, p: Params) -> list[Axis]:
    """Axes of the M3 boss holes in band_back, from their circular edges."""
    axes = circle_axes(back, p.m3_through, direction=PLATE_AXIS)
    return [a for a in axes if abs(a.position.X) <= p.boss_w / 2]


def boss_hardware_checks(part: Part, back: Part, p: Params) -> list[Check]:
    """Hardware for each boss bolt: M3 x boss_bolt_len + nut, head in the band's recess.

    Measured along a probe line parallel to the bolt, between the hole (r 1.7) and the nut
    pocket flats (2.85): in band_back the material from the head recess floor to the boss
    face, in the block the wall from its face to the nut seat. The bolt must reach through
    the whole nut (tip at least flush with the nut's far face), and the blind hole must be
    deep enough for the tip.
    """
    probe = (p.m3_through / 2 + p.m3_hex_af / 2) / 2
    y0, reach = front_y(p), 2 * (front_y(p) + p.block_size)
    checks = []
    for x, z in plate_bolts(p):
        name = f"boss bolt x={x:g} z={z:g}"
        band_spans = material_spans(back, (x, 0, z + probe), PLATE_AXIS, y0 + 1)
        block_spans = material_spans(part, (x, y0 - 1, z + probe), PLATE_AXIS, reach)
        hole_spans = material_spans(part, (x, y0 - 1, z), PLATE_AXIS, reach)
        if not band_spans or not block_spans or not hole_spans:
            checks.append(Check(f"{name}: hardware fits", False, "probe missed a part"))
            continue
        head_floor = band_spans[-1][0]
        boss_face = band_spans[-1][1]
        seat = y0 - 1 + block_spans[0][1]  # end of the wall in front of the nut
        hole_end = y0 - 1 + hole_spans[0][0]  # bottom of the blind hole
        tip = head_floor + p.boss_bolt_len
        need = seat + p.m3_nut_h - head_floor
        spare = tip - (seat + p.m3_nut_h)
        checks.append(
            Check(
                f"{name}: M3x{p.boss_bolt_len:g} reaches through the plate nut",
                spare >= 0 and abs(boss_face - y0) <= TOUCH_EPS,
                f"needs {need:.2f} mm (TPU boss {boss_face - head_floor:.2f} + PETG wall "
                f"{seat - boss_face:.2f} + nut {p.m3_nut_h:g}), "
                + (f"{spare:.2f} mm past the nut" if spare >= 0 else f"{-spare:.2f} mm short"),
            )
        )
        checks.append(at_least(f"{name}: blind hole room past the bolt tip", hole_end - tip, 0.0))
    return checks


def quarter_screw_check(part: Part, p: Params) -> Check:
    """A short 1/4"-20 accessory screw (quarter_screw_thread) engages the top-face nut.

    Measured on the solid: the top wall over the nut along a probe line beside the hole
    (between the hole and the pocket flats, offset across the slot to -Y), and the depth of
    the blind hole along the axis. The thread past the top wall must give
    quarter_engage_threads full threads in the nut, and the screw must not bottom out.
    """
    wall_min = MATERIALS[MATERIAL].wall_min
    _, yc, zt = top_center(p)
    probe = (p.quarter_through / 2 + p.quarter_hex_af / 2) / 2
    reach = top_z(p) - bottom_z(p) + 2
    wall_spans = material_spans(part, (0, yc - probe, zt + 1), TOP_AXIS, reach)
    hole_spans = material_spans(part, (0, yc, zt + 1), TOP_AXIS, reach)
    name = f'1/4"-20 screw with {p.quarter_screw_thread:g} mm of thread engages the top nut'
    if not wall_spans or not hole_spans or abs(wall_spans[0][0] - 1) > TOUCH_EPS:
        return Check(name, False, "probe missed the top wall")
    wall = wall_spans[0][1] - wall_spans[0][0]
    engage = p.quarter_screw_thread - wall
    need = p.quarter_engage_threads * p.quarter_pitch
    depth = hole_spans[0][0] - 1
    ok = wall >= wall_min and engage >= need and depth >= p.quarter_screw_thread
    return Check(
        name,
        ok,
        f"top wall {wall:.2f} mm (min {wall_min:g}), engagement {engage:.2f} mm = "
        f"{engage / p.quarter_pitch:.1f} threads (min {p.quarter_engage_threads:g}), "
        f"hole {depth:.1f} mm deep",
    )


def clash_with_band(part: Part, p: Params) -> list[Check]:
    """Zero-volume intersection with each band half and the column. The plate face sits on
    the boss face (intended contact); the front half and the column keep a gap."""
    checks = []
    for name, other in band_parts(p).items():
        contacts = [(NAME, name)] if name == band.BACK_NAME else []
        checks += clash_checks({NAME: part, name: other}, contacts)
    return checks


def checks(part: Part, p: Params) -> list[Check]:
    wall_min = MATERIALS[MATERIAL].wall_min
    env = envelope(p)
    groups = cutter_groups(p)
    voids = [(label, cavity_pieces(env, cutters)) for label, cutters in groups]
    back = band_parts(p)[band.BACK_NAME]

    plate = hole_axes(part, p.m3_through, direction=PLATE_AXIS)
    top_m3 = hole_axes(part, p.m3_through, direction=TOP_AXIS)
    quarter = hole_axes(part, p.quarter_through, direction=TOP_AXIS)
    _, yc, _ = top_center(p)
    n_plate, n_top = len(plate_bolts(p)), len(top_m3_points(p))
    result = [
        single_solid(part),
        count_is("plate M3 holes", len(plate), n_plate),
        coaxial_check(
            "plate M3 holes on design axes",
            plate,
            [plate_axis(x, z) for x, z in plate_bolts(p)],
            expected=n_plate,
        ),
        *hex_pocket_checks("plate M3", part, plate, p.m3_hex_af, n_plate),
        count_is("top M3 holes", len(top_m3), n_top),
        coaxial_check(
            "top M3 holes on design axes",
            top_m3,
            [top_axis(x, y, p) for x, y in top_m3_points(p)],
            expected=n_top,
        ),
        *hex_pocket_checks("top M3", part, top_m3, p.m3_hex_af, n_top),
        count_is("top 1/4 holes", len(quarter), 1),
        coaxial_check("top 1/4 hole on its design axis", quarter, [top_axis(0, yc, p)], 1),
        *hex_pocket_checks("top 1/4", part, quarter, p.quarter_hex_af, 1),
        coaxial_check(
            "plate nut holes coaxial with the boss holes (circular edges)",
            circle_axes(part, p.m3_through, direction=PLATE_AXIS),
            boss_holes(back, p),
            expected=n_plate,
        ),
    ]
    all_pieces = [piece for _, pieces in voids for piece in pieces]
    result.append(cavity_check("holes and nut pockets as designed", part, all_pieces, env))
    for label, pieces in voids:
        result.append(at_least(f"wall around {label}", wall_around(part, pieces), wall_min))
    result.append(pocket_gap_check(voids, wall_min))
    result.append(quarter_screw_check(part, p))
    result += boss_hardware_checks(part, back, p)
    result += clash_with_band(part, p)
    return result


def pocket_gap_check(voids: list[tuple[str, list[Part]]], wall_min: float) -> Check:
    """Solid distance between every pair of hole + pocket cavities: the thinnest is reported."""
    worst, pair = float("inf"), ""
    for i, (label_a, a) in enumerate(voids):
        for label_b, b in voids[i + 1 :]:
            gap = min_distance(Compound(a), Compound(b))
            if gap < worst:
                worst, pair = gap, f"{label_a} / {label_b}"
    ok = worst >= wall_min
    return Check(
        "wall between every pair of pockets", ok, f"{worst:.3f} mm (min {wall_min:g}), {pair}"
    )


PARTS = [PartSpec(NAME, MATERIAL, PRINT, build, checks, ASSEMBLY)]
