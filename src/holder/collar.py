"""Collar: one barrel-walled PETG body round the column, split at the seam, and two TPU
95A grip liners.

Form. The plan is one G1 turtle path of exact conics, made G2 at every join: a straight
front, quarter-ellipse front corners, straight sides, circular back shoulders, a concave
waist and the circular tail that carries the pan pivot. One edge profile is swept round
it: a G2 roll at the bottom, a 45 degree lead on the top face (the print bed), and a wall
that is an exact parabola bulging 2 mm. The seam is a straight split through that one
body, so the two shells read as one turned-looking object with a single parting line.

Load. The PETG shell carries the clamp. TPU 95A loses 30-38 % of its compression to set
(Essentium, BASF TDS), so the liner is only a 2 mm grip layer with 45 degree V ribs, and
the seam stops (stops.py) push with six flat DIN 2093 C springs each: the clamp force
stays near constant while the ribs creep or the column runs a little large or small.
The seam shoulder screws enter from the back (+Y, away from the monitor) and stop on M5
inserts in the front shell's seam face. Nothing shows on the front.
"""

from functools import cache

from build123d import Face, Part, Pos, RectangleRounded, Vector, Wire, extrude, loft

from holder.arm import SPOT, knob_d, pivot_stop
from holder.common import (
    MODEL_GAP,
    Check,
    SeatedInsert,
    at_least,
    box_from,
    flat_teardrop_rod,
    named,
    one_solid,
    overhang_rows,
    print_orientation,
    rod,
    skin_rows,
    teardrop_rod,
)
from holder.form import BedLead, Roll, Turtle, curvature_jumps, g2_skin
from holder.params import (
    INSERT_M5,
    INSERT_SINK,
    MATERIALS,
    PETG_WALL,
    SHOULDER_CLEAR,
    SHOULDER_M5,
    SPRING_C14,
    WASHER_M8,
    Params,
)
from holder.stops import Placed, StopSpec

PRINT = {
    "collar_front": (0, 0, 1),
    "collar_back": (0, 0, 1),
    "liner_front": (0, 0, -1),
    "liner_back": (0, 0, -1),
    **{f"seam_cap_{k}": (0, -1, 0) for k in range(1, 5)},
}
MATERIAL = {"collar_front": "PETG", "collar_back": "PETG",
            "liner_front": "TPU95A", "liner_back": "TPU95A",
            **{f"seam_cap_{k}": "TPU95A" for k in range(1, 5)}}
TOP = BedLead(1.0, 3.0)  # the top face prints on the bed
BOTTOM = Roll(6.0)
FRONT_CORNER = (16.0, 14.0)  # quarter ellipse: along the front, up the side
BACK_START = 36.0  # the half-ellipse back starts here: behind the seam counterbore mouths
PLAN_TRIM = 3.0
SEAM_SPRINGS = 6
SEAM_TARGET = 0.75  # at liner contact, nominal column
SEAM_L1 = 35.0
LINER_END_GAP = 0.15
LINER_OUTER_GAP = 0.05
RIB_OVERLAP = 0.05
CAP_PRESS = 0.1  # the TPU cap is this much over the counterbore, a side: a press fit
CAP_ROOM = 1.4  # cap to screw head: the head moves 1.05 toward it over the seam travel
KNOB_ROOM = 1.2  # the pivot knob turns in a recess this much wider than the knob
UP = Vector(0, 0, -1)  # print-up in the use frame: the shells print top face down
CLAMP_SHIFT = 0.25 - 0.02  # each shell moves this far toward the other at liner contact


def column_profile(spec: Params, grow: float, z: float = 0.0) -> Face:
    """Column outline grown by `grow`. Arcs offset to arcs, so this offset is exact."""
    face = RectangleRounded(spec.column_w + 2 * grow, spec.column_d + 2 * grow,
                            spec.column_r + grow)
    return Pos(0, 0, z) * face


# --- Plan --------------------------------------------------------------------------------------


def plan_turtle(spec: Params) -> Turtle:
    """The whole plan, counter-clockwise from the front-right corner: a straight front, quarter
    ellipse corners, straight sides, and a half-ellipse back that carries the pan pivot."""
    hw, hd = spec.outer_half_w, spec.outer_half_d
    along, up = FRONT_CORNER
    back = tail_y(spec) - BACK_START
    path = Turtle((hw - along, -hd), 0.0)
    path.ellipse(along, up, 90)
    path.line(BACK_START - (-hd + up))
    path.ellipse(back, hw, 90)
    path.ellipse(hw, back, 90)
    path.line(BACK_START - (-hd + up))
    path.ellipse(up, along, 90)
    path.line(2 * (hw - along))
    return path


def tail_y(spec: Params) -> float:
    return spec.pivot_y + spec.pivot_r


def plan(spec: Params) -> Wire:
    return Wire(plan_turtle(spec).soft(PLAN_TRIM, closed=True))


@cache
def skin(spec: Params) -> Part:
    """The uncut body: one profile swept round the plan."""
    return g2_skin(plan(spec), spec.collar_h, BOTTOM, TOP, spec.collar_bulge)


def cavity(spec: Params) -> Part:
    """Bore with a 45 degree lip at each end. The liner snaps in behind the lips."""
    lip = spec.bore_offset - spec.lip_reach
    height = spec.collar_h
    through = extrude(column_profile(spec, lip, -1), amount=height + 2)
    sections = [
        column_profile(spec, lip, spec.lip_h),
        column_profile(spec, spec.bore_offset, spec.lip_h + spec.lip_reach),
        column_profile(spec, spec.bore_offset, height - spec.lip_h - spec.lip_reach),
        column_profile(spec, lip, height - spec.lip_h),
    ]
    return through + loft(sections, ruled=True)


def _half(body: Part, sign: float, spec: Params) -> Part:
    """The front (sign -1) or back half of a body split by the seam gap."""
    gap, reach = spec.seam_gap / 2, 200
    return body & box_from(-reach, sign * gap, -1, reach, sign * reach, spec.collar_h + 1)


# --- Seam stops --------------------------------------------------------------------------------


def seam_stop() -> StopSpec:
    return StopSpec("seam", SHOULDER_M5, SPRING_C14, SEAM_SPRINGS, None, WASHER_M8,
                    INSERT_M5, SEAM_TARGET, SEAM_L1)


def seam_points(spec: Params) -> list[tuple[float, float]]:
    return [(sign * spec.seam_x, z) for z in spec.seam_z for sign in (-1.0, 1.0)]


def contact_gap(spec: Params) -> float:
    """Seam gap when both liners just touch the column."""
    return spec.seam_gap - 2 * CLAMP_SHIFT


def seam_seat_y(spec: Params) -> float:
    """Back shell seat (part frame) that puts the stop at its design length at contact."""
    insert_top = -spec.seam_gap / 2 - 0.2 + CLAMP_SHIFT  # front shell moved toward the back
    return insert_top + seam_stop().clamped + CLAMP_SHIFT


def seam_stops(spec: Params) -> list[Placed]:
    """The four seam stops in the hero frame (shells closed to liner contact). The axis
    runs from the head (back) toward the insert (front)."""
    y = seam_seat_y(spec) - CLAMP_SHIFT
    return [Placed(seam_stop(), Vector(x, y, z), Vector(0, -1, 0)) for x, z in seam_points(spec)]


def seam_inserts(spec: Params) -> list[SeatedInsert]:
    """The inserts in the front shell's part frame, pressed from its seam face."""
    face_y = -spec.seam_gap / 2
    return [SeatedInsert(f"seam ({x:+.1f}, z {z:g})", INSERT_M5, Vector(x, face_y, z),
                         Vector(0, -1, 0), 0.8) for x, z in seam_points(spec)]


def seam_bore(spec: Params, x: float, z: float) -> Part:
    """Counterbore for the seat washer, springs and head, from the seat out the back."""
    return flat_teardrop_rod(spec.seam_bore_d, 40.0, Vector(x, seam_seat_y(spec), z),
                             (0, 1, 0), UP)


def seam_cap(spec: Params, x: float, z: float) -> Part:
    """TPU cover for a seam counterbore, cut from the collar's own skin: flush, so the back
    reads as one surface. It sits clear of the screw head over the whole seam travel."""
    stop = seam_stop()
    inner = seam_seat_y(spec) + stop.above_seat + stop.screw.head_k + CAP_ROOM
    plug = flat_teardrop_rod(spec.seam_bore_d + 2 * CAP_PRESS, 60.0, Vector(x, inner, z),
                             (0, 1, 0), UP)
    return plug & skin(spec)


def seam_caps(spec: Params) -> list[Part]:
    return [named(seam_cap(spec, x, z), f"seam_cap_{k + 1}")
            for k, (x, z) in enumerate(seam_points(spec))]


def checks_cap(part: Part, spec: Params) -> list[Check]:
    rows = [one_solid(part)]
    rows += overhang_rows(print_orientation(part, PRINT["seam_cap_1"]), part.label)
    depth = part.bounding_box().size.Y
    rows.append(at_least(f"{part.label} grip length in its counterbore", depth, 3.0))
    return rows


def seam_shank(spec: Params, x: float, z: float) -> Part:
    d = SHOULDER_M5.shoulder_d + SHOULDER_CLEAR
    start = Vector(x, spec.seam_gap / 2 - 1.0, z)
    return teardrop_rod(d, seam_seat_y(spec) - start.Y + 1.0, start, (0, 1, 0), UP)


# --- Pan pivot ---------------------------------------------------------------------------------


def pivot_cutters(spec: Params, seat_z: float, spot: float, recess_d: float) -> list[Part]:
    """Shoulder bore up through the tail, the seat washer's spot face, the knob recess."""
    x, y = 0.0, spec.pivot_y
    return [rod(8.4, spec.collar_h + 2, (x, y, -1), (0, 0, 1)),
            rod(WASHER_M8.outer + 0.4, spot + 0.01, (x, y, seat_z - spot), (0, 0, 1)),
            rod(recess_d, seat_z - spot + 1, (x, y, -1), (0, 0, 1))]


# --- Build ---------------------------------------------------------------------------------------


def pivot_seat_z(spec: Params) -> float:
    """Seat washer face under the tail: the pivot stop's clamped length below the insert
    top, which sits INSERT_SINK into the pedestal resting MODEL_GAP above the collar."""
    return spec.collar_h + MODEL_GAP + INSERT_SINK - pivot_stop().clamped


def build_shells(spec: Params) -> tuple[Part, Part]:
    body = skin(spec) - cavity(spec)
    front, back = _half(body, -1.0, spec), _half(body, 1.0, spec)
    for insert in seam_inserts(spec):
        for cutter in insert.cutters():
            front -= cutter
    for x, z in seam_points(spec):
        back -= seam_bore(spec, x, z)
        back -= seam_shank(spec, x, z)
    for cutter in pivot_cutters(spec, pivot_seat_z(spec), SPOT, knob_d(spec) + KNOB_ROOM):
        back -= cutter
    return named(front, "collar_front"), named(back, "collar_back")


def build_liners(spec: Params) -> tuple[Part, Part]:
    """Liner halves: shell cavity minus clearances, V ribs at rib_pitch."""
    lip = spec.bore_offset - spec.lip_reach
    outer = spec.bore_offset - LINER_OUTER_GAP
    z0 = spec.lip_h + LINER_END_GAP
    z1 = spec.collar_h - spec.lip_h - LINER_END_GAP
    cone = outer - lip
    sections = [column_profile(spec, lip, z0), column_profile(spec, outer, z0 + cone),
                column_profile(spec, outer, z1 - cone), column_profile(spec, lip, z1)]
    valley = spec.tpu_fit + spec.rib_h
    ring = loft(sections, ruled=True) - extrude(column_profile(spec, valley, z0 - 1),
                                                amount=z1 - z0 + 2)
    for z in rib_bases(spec, z0, z1):
        ring += rib(spec, z)
    front = _half(ring, -1.0, spec)
    back = _half(ring, 1.0, spec)
    return named(front, "liner_front"), named(back, "liner_back")


def rib(spec: Params, z: float) -> Part:
    """One 45 degree V rib ring, base sunk RIB_OVERLAP into the liner so the two fuse."""
    base = spec.tpu_fit + spec.rib_h + RIB_OVERLAP
    rise = spec.rib_h + RIB_OVERLAP
    bottom = z - RIB_OVERLAP
    band = extrude(column_profile(spec, base, bottom), amount=2 * rise)
    waist = loft([column_profile(spec, base, bottom),
                  column_profile(spec, spec.tpu_fit, bottom + rise),
                  column_profile(spec, base, bottom + 2 * rise)], ruled=True)
    return band - waist


def rib_bases(spec: Params, z0: float, z1: float) -> list[float]:
    """Rib bottoms, centred in the liner height, one per rib_pitch."""
    span = 2 * spec.rib_h
    count = int((z1 - z0 - span - 2.0) // spec.rib_pitch) + 1
    used = (count - 1) * spec.rib_pitch + span
    first = (z0 + z1 - used) / 2
    return [first + k * spec.rib_pitch for k in range(count)]


# --- Checks ------------------------------------------------------------------------------------


def _form_rows(part: Part, spec: Params, label: str) -> list[Check]:
    on_bed = lambda centre: abs(centre.Z - spec.collar_h) < 1e-4  # noqa: E731
    rows = skin_rows(part, label, on_bed, skin(spec))
    jump = max(curvature_jumps(plan(spec)))
    rows.append(Check(f"{label} plan curvature jump (G2)", jump <= 1e-3,
                      f"{jump:.2e} /mm (max 1e-3)"))
    hw, back = spec.outer_half_w, tail_y(spec) - BACK_START
    smallest = min(FRONT_CORNER[1] ** 2 / FRONT_CORNER[0], back ** 2 / hw, hw ** 2 / back)
    rows.append(at_least(f"{label} smallest convex plan radius over the bottom roll",
                         smallest, BOTTOM.setback + 0.5))
    return rows


def checks_front(part: Part, spec: Params) -> list[Check]:
    rows = [one_solid(part)]
    rows += overhang_rows(print_orientation(part, PRINT["collar_front"]), "collar_front")
    rows += _form_rows(part, spec, "collar_front")
    for insert in seam_inserts(spec):
        rows += insert.checks(part)
    return rows


def checks_back(part: Part, spec: Params) -> list[Check]:
    rows = [one_solid(part)]
    rows += overhang_rows(print_orientation(part, PRINT["collar_back"]), "collar_back")
    rows += _form_rows(part, spec, "collar_back")
    bore = cavity(spec)
    for x, z in seam_points(spec):
        where = f"({x:+.1f}, z {z:g})"
        rows.append(at_least(f"seam counterbore {where} wall to the bore",
                             seam_bore(spec, x, z).distance_to(bore), PETG_WALL))
        rows.append(at_least(f"seam shoulder bore {where} wall to the bore",
                             seam_shank(spec, x, z).distance_to(bore), PETG_WALL))
    low, high = spec.seam_z
    rows.append(at_least("PETG between the two seam counterbores",
                         high - low - spec.seam_bore_d, PETG_WALL))
    rows.append(at_least("PETG under the lower seam counterbore",
                         low - spec.seam_bore_d / 2, PETG_WALL))
    rows.append(at_least("PETG over the upper seam counterbore",
                         spec.collar_h - high - spec.seam_bore_d / 2, PETG_WALL))
    pivot = rod(8.4, spec.collar_h, (0, spec.pivot_y, 0), (0, 0, 1))
    rows.append(at_least("pivot bore wall to the column bore", pivot.distance_to(bore),
                         PETG_WALL))
    return rows


def checks_liner(part: Part, spec: Params) -> list[Check]:
    rows = [one_solid(part)]
    rows += overhang_rows(print_orientation(part, PRINT[part.label]), part.label)
    tpu = MATERIALS["TPU95A"]["wall_min"]
    rows.append(at_least(f"{part.label} wall", spec.liner_t, tpu))
    return rows


def checks_pair(front: Part, back: Part, liners: tuple[Part, Part], column: Part,
                spec: Params) -> list[Check]:
    """In the hero the shells are closed to liner contact: rib tips 0.02 off the column."""
    rows = [at_least("seam gap at liner contact", front.distance_to(back),
                     contact_gap(spec) - 0.05)]
    for liner in liners:
        gap = liner.distance_to(column)
        rows.append(Check(f"{liner.label} rib tips at the column (contact)",
                          abs(gap - 0.02) <= 0.01, f"{gap:.3f} mm (0.02 +- 0.01)"))
    return rows
