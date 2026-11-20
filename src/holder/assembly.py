"""The finished device on the column, every bought part a solid, clamped at liner contact.

Order of assembly (the access checks use these steps):
1. Press the four M5 inserts into collar_front's seam faces, the M6 inserts into arm_base
   (pivot face), arm_link (both ends) and arm_tip (end B). Melt each shoulder screw's
   knurled head into a knob.
2. Snap the liners into both shells.
3. Close the shells round the column. From the back, each seam stop: seat washer, six
   flat springs, M5 shoulder screw; turn with a 3 mm key until it stops. Press in the
   TPU caps (step 4 in the access checks, after the keys).
4. Pan pivot: pedestal on the collar's back; from below, seat washer, five springs, head
   washer and the knob with its screw; turn until it stops.
5. J1, J2, J3 the same way, with the TPU ring in J3. Knobs on -X, -X, +X.
6. Camera on the platform; 1/4"-20 screw and its washer from below.

Stops and stacks are measured here by rays on the placed solids. Loads use the payload at
the camera's centre of mass, in the worst pose (both links level).
"""

import multiprocessing
import os
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from functools import cache
from math import atan, degrees, radians, sin, tan

from build123d import Axis, Box, Cylinder, Location, Part, Plane, Pos, Rot, Vector, fillet

from holder import arm, collar
from holder.common import (
    MODEL_GAP,
    VOL_EPS,
    Check,
    annulus_footprint,
    at_least,
    at_most,
    bearing_row,
    boxes_touch,
    clash_rows,
    column_solid,
    contact_area,
    first_hit,
    intersection_volume,
    rod,
    socket_screw,
    stress_row,
    washer_solid,
    within,
)
from holder.params import (
    CAMERA_HAND_NM,
    INSERT_M6,
    MU_PETG,
    MU_TPU,
    PETG_UTS,
    PLATE_UNDER_HEAD,
    PRINT_TOL,
    QUARTER_SHCS,
    RING_T,
    RING_TOL,
    WASHER_M6_LARGE,
    ZVE10_BODY,
    ZVE10_DOOR,
    ZVE10_DOOR_SWING,
    ZVE10_SOCKET,
    ZVE10_THREAD_MAX,
    ZVE10_THREAD_MIN,
    Params,
    preload_n,
)
from holder.stops import Placed

X = Vector(1, 0, 0)
Y = Vector(0, 1, 0)
Z = Vector(0, 0, 1)
KEY_ROD = 120.0
SWEEP_STEP = 15
MAX_SWEEP_WORKERS = 12  # each worker holds its own copy of the hero (about 0.5 GB)
CHAINS = ("fixed", "base", "link", "tip", "head")  # a joint moves every chain after it
COLUMN_CLEARANCE = 3.0  # mm kept between the column and anything above the collar
PEAK = 2.0  # peak load factor (DESIGN 2.4)
FINGER = 10.0  # finger room round a knob, radially
GRIP_SHARE = 0.6  # fingers take the outer part of a knob, toward its crown
CAMERA_GAP = 0.05  # the platform's G2 end blend rises 0.02 mm over the flat top
CG_FORWARD = 5.0  # estimated: the kit lens moves the camera's mass this far toward -Y
J3_FINE_TILT = 15.0  # J3 holds the 2x load on friction within this tilt either way


@dataclass
class Item:
    part: Part
    finish: str
    step: int
    chain: str = "fixed"
    bom: tuple[str, str, str] | None = None  # (designation, standard, source)
    printed: bool = False


@dataclass
class Hero:
    items: dict[str, Item] = field(default_factory=dict)
    intended: set[frozenset[str]] = field(default_factory=set)
    rods: list[tuple[str, Part, int, set[str]]] = field(default_factory=list)
    hands: list[tuple[str, Part, set[str]]] = field(default_factory=list)
    stops: dict[str, tuple[Placed, str, str, str]] = field(default_factory=dict)
    locations: dict[str, Location] = field(default_factory=dict)

    def add(self, name: str, item: Item) -> None:
        if name in self.items:
            raise ValueError(f"duplicate assembly item {name}")
        item.part.label = name
        self.items[name] = item


def _turn(alpha: float) -> Vector:
    return Vector(0, 1, 0).rotate(Axis.X, alpha)


def pivot_world(spec: Params) -> Vector:
    """Pivot face centre in the hero: the back shell sits CLAMP_SHIFT toward the front."""
    return Vector(0, spec.pivot_y - collar.CLAMP_SHIFT, spec.collar_h + MODEL_GAP)


def joints(spec: Params) -> tuple[Vector, Vector, Vector]:
    j1 = pivot_world(spec) + arm.j1_local(spec)
    a1, a2, _ah = spec.hero_angles
    j2 = j1 + _turn(a1) * spec.link_pitch
    j3 = j2 + _turn(a2) * spec.link_pitch
    return j1, j2, j3


def _placed(part: Part, location: Location) -> Part:
    moved = location * part
    moved.label = part.label
    return moved


def _point(location: Location, local: Vector) -> Vector:
    return (location * Pos(local)).position


def _bought(designation: str, standard: str, source: str) -> tuple[str, str, str]:
    return (designation, standard, source)


# --- Building the hero ------------------------------------------------------------------------


@cache
def printed_parts(spec: Params) -> dict[str, Part]:
    front, back = collar.build_shells(spec)
    liner_front, liner_back = collar.build_liners(spec)
    return {
        "collar_front": front, "collar_back": back,
        "liner_front": liner_front, "liner_back": liner_back,
        "arm_base": arm.build_base(spec), "arm_link": arm.build_link(spec),
        "arm_tip": arm.build_tip(spec), "arm_head": arm.build_head(spec),
        "knob": arm.build_knob(spec), "j3_ring": arm.build_ring(spec),
        **{cap.label: cap for cap in collar.seam_caps(spec)},
    }


def column(spec: Params) -> Part:
    return column_solid(spec.column_w, spec.column_d, spec.column_r, -60, spec.column_top_z)


@cache
def hero(spec: Params) -> Hero:
    parts = printed_parts(spec)
    built = Hero()
    built.add("column", Item(column(spec), "column", 0))
    front_at, back_at = Pos(0, collar.CLAMP_SHIFT, 0), Pos(0, -collar.CLAMP_SHIFT, 0)
    built.locations.update({"collar_front": front_at, "collar_back": back_at})
    built.add("collar_front", Item(_placed(parts["collar_front"], front_at), "petg", 3,
                                   printed=True))
    built.add("collar_back", Item(_placed(parts["collar_back"], back_at), "petg", 3,
                                  printed=True))
    built.add("liner_front", Item(_placed(parts["liner_front"], front_at), "tpu", 2,
                                  printed=True))
    built.add("liner_back", Item(_placed(parts["liner_back"], back_at), "tpu", 2,
                                 printed=True))
    _seam(built, spec)
    for k in range(1, 5):
        name = f"seam_cap_{k}"
        built.add(name, Item(_placed(parts[name], back_at), "petg", 4, printed=True))
        built.intended.add(frozenset((name, "collar_back")))  # pressed in
    _arm(built, spec, parts)
    _camera(built, spec)
    return built


def _insert(built: Hero, name: str, seated, host: str, location: Location, chain: str,
            step: int) -> None:
    insert = seated.insert
    built.add(name, Item(location * seated.solid(), "brass", step, chain,
                         bom=_bought(insert.name, insert.standard, insert.source)))
    built.intended.add(frozenset((name, host)))  # the knurl melts into the hole wall


def _stop_hardware(built: Hero, spec: Params, name: str, stop: Placed, chain: str,
                   step: int, insert: str, knob: Part | None, grip_from: float = 0.0) -> None:
    """Screw, washers, springs, and the knob that holds the screw head (M6 stops).

    Finger room is checked round the outer GRIP_SHARE of the knob, or from `grip_from`
    past its rim where the knob sits in a recess.
    """
    parts = stop.spec
    screw = f"{name} screw"
    built.add(screw, Item(stop.screw_solid(), "black_steel", step, chain,
                          bom=_bought(f"{parts.screw.name}, l1 {parts.l1:g}",
                                      parts.screw.standard, parts.screw.source)))
    built.intended.add(frozenset((screw, insert)))
    for label, washer, solid in stop.washer_solids():
        built.add(f"{name} {label}", Item(solid, "steel", step, chain,
                                          bom=_bought(washer.name, washer.standard,
                                                      washer.source)))
    for k, solid in enumerate(stop.spring_solids()):
        built.add(f"{name} spring {k + 1}", Item(solid, "steel", step, chain, bom=_bought(
            parts.spring.name, parts.spring.standard, parts.spring.source)))
    if knob is None:
        return
    rim = stop.head_under + stop.axis * arm.knob_skirt()  # the knob holds the head
    turn = Plane(origin=rim, x_dir=_perpendicular(stop.axis), z_dir=-stop.axis).location
    knob_name = f"{name} knob"
    built.add(knob_name, Item(_placed(knob, turn), "petg", step, chain, printed=True))
    built.intended.add(frozenset((screw, knob_name)))  # the knurled head is melted in
    reach, height = arm.knob_d(spec) / 2, arm.knob_height()
    start = rim - stop.axis * max(grip_from, height * (1 - GRIP_SHARE))
    length = height - (start - rim).length
    hand = rod(2 * (reach + FINGER), length, start, -stop.axis)
    hand -= rod(2 * reach + 1.0, length + 2, start + stop.axis, -stop.axis)
    built.hands.append((knob_name, hand, {knob_name}))


def _perpendicular(axis: Vector) -> Vector:
    reference = Z if abs(axis.Z) < 0.9 else X
    return (reference - axis * reference.dot(axis)).normalized()


def _seam(built: Hero, spec: Params) -> None:
    front_at = built.locations["collar_front"]
    for index, (stop, seated) in enumerate(zip(collar.seam_stops(spec),
                                               collar.seam_inserts(spec))):
        name = f"seam {index + 1}"
        insert = f"{name} insert"
        _insert(built, insert, seated, "collar_front", front_at, "fixed", 1)
        _stop_hardware(built, spec, name, stop, "fixed", 3, insert, None)
        built.stops[name] = (stop, "collar_back", "collar_front", insert)
        key = rod(stop.spec.screw.key + 1.0, KEY_ROD, stop.head_under - stop.axis * 1.0,
                  -stop.axis)
        built.rods.append((f"{name} screw", key, 3, {insert}))


def link_location(start: Vector, alpha: float, flipped: bool) -> Location:
    turn = Rot(alpha, 0, 0) * (Rot(0, 180, 0) if flipped else Location())
    return Pos(start) * turn


def _arm(built: Hero, spec: Params, parts: dict[str, Part]) -> None:
    j1, j2, j3 = joints(spec)
    a1, a2, ah = spec.hero_angles
    gap, t, flat = arm.pitch_gap(spec), arm.link_t(spec), spec.ball_flat_x
    base_at = Pos(pivot_world(spec)) * Rot(0, 0, spec.hero_pan)
    link_at = link_location(Vector(flat + gap, j1.Y, j1.Z), a1, flipped=False)
    tip_at = link_location(Vector(flat, j2.Y, j2.Z), a2, flipped=True)
    face = flat + RING_T + 2 * arm.ring_gap()
    head_at = Pos(face, j3.Y, j3.Z) * Rot(ah, 0, 0)
    ring_at = Pos(flat + arm.ring_gap(), j3.Y, j3.Z) * Rot(0, 90, 0)
    built.locations.update({"arm_base": base_at, "arm_link": link_at, "arm_tip": tip_at,
                            "arm_head": head_at})
    for name, at, chain in (("arm_base", base_at, "base"), ("arm_link", link_at, "link"),
                            ("arm_tip", tip_at, "tip"), ("arm_head", head_at, "head")):
        built.add(name, Item(_placed(parts[name], at), "petg", 4 if chain == "base" else 5,
                             chain, printed=True))
    built.add("j3_ring", Item(_placed(parts["j3_ring"], ring_at), "tpu", 5, "tip",
                              printed=True))
    knob = parts["knob"]
    pivot = Placed(arm.pivot_stop(), Vector(0, pivot_world(spec).Y, collar.pivot_seat_z(spec)),
                   Z)
    _insert(built, "pivot insert", arm.pivot_insert(), "arm_base", base_at, "base", 1)
    recessed = pivot.head_under.Z + arm.knob_skirt() + 0.3  # rim to the collar bottom
    _stop_hardware(built, spec, "pivot", pivot, "fixed", 4, "pivot insert", knob, recessed)
    built.stops["pivot"] = (pivot, "collar_back", "arm_base", "pivot insert")
    a_insert, b_insert = arm.link_inserts(spec, ("A", "B"))
    _insert(built, "J1 insert", a_insert, "arm_link", link_at, "link", 1)
    _insert(built, "J2 insert", b_insert, "arm_link", link_at, "link", 1)
    _insert(built, "J3 insert", arm.link_inserts(spec, ("B",))[0], "arm_tip", tip_at, "tip", 1)
    stops = {
        "J1": (Placed(arm.joint_stop("J1"), Vector(arm.base_knob_seat_x(spec), j1.Y, j1.Z), X),
               "base", "arm_base", "arm_link"),
        "J2": (Placed(arm.joint_stop("J2"), Vector(flat - t + arm.SPOT, j2.Y, j2.Z), X),
               "tip", "arm_tip", "arm_link"),
        "J3": (Placed(arm.joint_stop("J3"),
                      Vector(face + arm.head_block_t() - arm.SPOT, j3.Y, j3.Z), -X),
               "head", "arm_head", "arm_tip"),
    }
    for label, (stop, chain, near, far) in stops.items():
        _stop_hardware(built, spec, label, stop, chain, 5, f"{label} insert", knob)
        built.stops[label] = (stop, near, far, f"{label} insert")


# --- Camera -----------------------------------------------------------------------------------


def camera_frame(spec: Params, built: Hero) -> tuple[Vector, Vector]:
    """(socket, body centre of the bottom face), world, hero pose."""
    socket = _point(built.locations["arm_head"], arm.camera_socket(spec)) + Z * CAMERA_GAP
    dx, dy = ZVE10_SOCKET
    return socket, socket - Vector(dx, dy, 0)


def camera_envelope(socket: Vector, centre: Vector) -> Part:
    """ZV-E10 body (Sony help guide) on the platform, lens toward -Y, socket at `socket`.

    The socket is a hole ZVE10_THREAD_MAX deep, so a screw that is too long clashes.
    The 3 mm edge rounds are for the picture only.
    """
    width, height, depth = ZVE10_BODY
    body = Pos(centre.X, centre.Y, centre.Z + height / 2) * Box(width, depth, height)
    body = fillet(body.edges(), 3.0)
    body -= rod(QUARTER_SHCS.d + 0.05, ZVE10_THREAD_MAX, socket - Z * 0.01, Z)
    # estimated: SELP1650 kit lens 64.7 x 29.9 mm (Sony spec sheet); lens axis over the socket.
    lens = Pos(socket.X, centre.Y - depth / 2 - 15, centre.Z + 34) * (
        Rot(90, 0, 0) * Cylinder(64.7 / 2, 30))
    return body + lens


def door_region(centre: Vector) -> Part:
    """The battery and card door on the grip end (-X) and the room it swings into."""
    width = ZVE10_BODY[0]
    door_w, door_d, inset = ZVE10_DOOR
    x0 = centre.X - width / 2 + inset
    return Pos(x0 + door_w / 2, centre.Y, centre.Z - ZVE10_DOOR_SWING / 2 - 0.05) * Box(
        door_w, door_d, ZVE10_DOOR_SWING)


def camera_cg(spec: Params, built: Hero) -> Vector:
    _socket, centre = camera_frame(spec, built)
    return centre + Vector(0, -CG_FORWARD, ZVE10_BODY[1] / 2)


def camera_screw_length() -> float:
    """Shortest stock that still engages ZVE10_THREAD_MIN on the short end of the tolerance."""
    grip = PLATE_UNDER_HEAD + WASHER_M6_LARGE.t
    return QUARTER_SHCS.stock_at_least(grip + ZVE10_THREAD_MIN + QUARTER_SHCS.length_tol)


def _camera(built: Hero, spec: Params) -> None:
    socket, centre = camera_frame(spec, built)
    head_at = built.locations["arm_head"]
    seat = _point(head_at, arm.camera_socket(spec) - Z * PLATE_UNDER_HEAD)
    washer_at = seat - Z * (MODEL_GAP + WASHER_M6_LARGE.t)
    head_under = washer_at - Z * MODEL_GAP
    length = camera_screw_length()
    built.add("camera washer", Item(washer_solid(WASHER_M6_LARGE, washer_at, Z), "steel", 6,
                                    "head", bom=_bought(WASHER_M6_LARGE.name,
                                                        WASHER_M6_LARGE.standard,
                                                        WASHER_M6_LARGE.source)))
    built.add("camera screw", Item(socket_screw(QUARTER_SHCS, length, head_under, Z),
                                   "black_steel", 6, "head",
                                   bom=_bought(f"1/4\"-20 x {length / 25.4:.3g}\"",
                                               QUARTER_SHCS.standard, QUARTER_SHCS.source)))
    built.add("zve10 camera", Item(camera_envelope(socket, centre), "camera", 6, "head"))
    key = rod(QUARTER_SHCS.key + 1.0, KEY_ROD, head_under - Z * (QUARTER_SHCS.head_k - 0.5),
              -Z)
    built.rods.append(("camera screw", key, 6, {"zve10 camera"}))


# --- Stops, measured ---------------------------------------------------------------------------


def payload_n(spec: Params) -> float:
    return spec.payload_kg * 9.81


def _reach(spec: Params, built: Hero) -> tuple[float, float]:
    """(J1 to the camera's mass along the arm with both links level, J3 to it)."""
    cg = camera_cg(spec, built)
    j3 = joints(spec)[2]
    j3_lever = abs(cg.Y - j3.Y) + CG_FORWARD
    return 2 * spec.link_pitch + j3_lever, j3_lever


def stop_needs(spec: Params, built: Hero) -> dict[str, tuple[float, str]]:
    """The least stop force each joint needs to hold the 2x load."""
    j1_lever, j3_lever = _reach(spec, built)
    w2 = PEAK * payload_n(spec)
    mean = (spec.rosette_r_in + spec.rosette_r_out) / 2
    phi = degrees(atan(MU_PETG))

    def rosette(lever: float) -> float:
        return w2 * lever / mean * tan(radians(45 - phi))

    cg = camera_cg(spec, built)
    j3 = joints(spec)[2]
    rise = cg.Z - j3.Z
    j3_moment = w2 * (j3_lever + rise * sin(radians(J3_FINE_TILT)))
    ring_mean = (arm.RING_OUTER + arm.RING_INNER) / 4
    pivot_moment = w2 * (j1_lever + abs(cg.X - pivot_world(spec).X))
    contact = arm.foot_r(spec) - arm.FOOT_LEAD
    couple = pivot_moment / (2 * spec.collar_h / 3)
    return {
        "J1": (rosette(j1_lever), "the rosette at the 2x moment"),
        "J2": (rosette(j1_lever - spec.link_pitch), "the rosette at the 2x moment"),
        "J3": (j3_moment / (MU_TPU * ring_mean), f"ring friction at 2x, tilted {J3_FINE_TILT:g}"),
        "pivot": (pivot_moment / contact, "no lift-off at the 2x tipping moment"),
        "seam": (couple / 4, "the 2x tilting couple on four screws"),
    }


def stop_rows(spec: Params) -> list[Check]:
    built = hero(spec)
    needs = stop_needs(spec, built)
    rows = []
    for name, (stop, near, far, insert) in built.stops.items():
        need, why = needs["seam" if name.startswith("seam") else name]
        near_part, far_part = built.items[near].part, built.items[far].part
        insert_part = built.items[insert].part
        if name.startswith("seam"):
            rows += _seam_rows(spec, name, stop, near_part, far_part, insert_part, need, why)
            continue
        spread = arm_spread(name)
        rows += stop.rows(near_part, far_part, insert_part, spread, need, why)
        lift = spec.index_lift if name in ("J1", "J2") else 1.0
        rows.append(stop.open_row(lift))
    return rows


def arm_spread(name: str) -> float:
    """Clamped-length tolerance: printed PETG, and for J3 the printed TPU ring too."""
    return PRINT_TOL + (RING_TOL if name == "J3" else 0.0)


def _seam_rows(spec: Params, name: str, stop: Placed, near: Part, far: Part, insert: Part,
               need: float, why: str) -> list[Check]:
    """The seam stop sits at liner contact. The column may run column_tol large or small,
    and the ribs crush up to seam_crush while they creep: the springs cover it all."""
    clamped = stop.measured_clamped(near, insert)
    design = stop.spec.clamped
    rows = [within(f"{name}: seat to insert top at liner contact", clamped, design - 0.05,
                   design + 0.05)]
    low = clamped - spec.column_tol - spec.seam_crush - PRINT_TOL
    high = clamped + spec.column_tol + PRINT_TOL
    rows += stop.force_rows(near, low, high, need, why)
    rows += stop.thread_rows(far)
    rows.append(stop.shoulder_on_brass(far))
    return rows


def camera_rows(spec: Params) -> list[Check]:
    """1/4"-20 camera screw: grip by rays, engagement in the socket, washer bearing."""
    built = hero(spec)
    head = built.items["arm_head"].part
    socket, centre = camera_frame(spec, built)
    side = Vector(0, WASHER_M6_LARGE.outer / 4 + 1.8, 0)
    below = Vector(socket.X, socket.Y, socket.Z - 40) + side
    ceiling = below.Z + first_hit(head, below, Z)
    top = ceiling + 0.01 + first_hit(head, Vector(below.X, below.Y, ceiling + 0.01), Z)
    grip = top - ceiling + WASHER_M6_LARGE.t
    length, tol = camera_screw_length(), QUARTER_SHCS.length_tol
    rows = [
        within("camera grip by rays is the design grip", grip,
               PLATE_UNDER_HEAD + WASHER_M6_LARGE.t - 0.05,
               PLATE_UNDER_HEAD + WASHER_M6_LARGE.t + 0.05),
        at_least("camera screw engagement, short screw", length - tol - grip, ZVE10_THREAD_MIN),
        at_most("camera screw tip in the socket, long screw", length + tol - grip,
                ZVE10_THREAD_MAX),
    ]
    force = preload_n(CAMERA_HAND_NM, QUARTER_SHCS.d)
    seat = annulus_footprint(Vector(socket.X, socket.Y, ceiling), Z, WASHER_M6_LARGE.outer,
                             WASHER_M6_LARGE.inner)
    rows.append(bearing_row(f"camera washer at {CAMERA_HAND_NM:g} Nm by hand", force,
                            contact_area(head, seat)))
    door = door_region(centre)
    hits = [name for name, item in built.items.items()
            if name != "zve10 camera" and intersection_volume(door, item.part) > VOL_EPS]
    rows.append(Check("ZV-E10 battery door and its swing room clear of every part", not hits,
                      "clear" if not hits else "hits " + ", ".join(hits)))
    return rows


# --- Loads: section stress -----------------------------------------------------------------------


def _level_moment(spec: Params, lever_y: float, lever_x: float) -> Vector:
    """Moment (N mm, world axes) of the payload about a point lever_y behind and lever_x
    beside the camera's mass, with the arm level: M = r x F, F = -w Z."""
    weight = payload_n(spec)
    return Vector(-weight * lever_y, weight * lever_x, 0)


@dataclass(frozen=True)
class Section:
    name: str
    part: str  # printed part, in its own frame
    plane: Plane  # in the part frame
    lever_y: float  # horizontal distance to the camera's mass, arm level
    lever_x: float  # camera mass x minus the section centre x, world
    frame: Rot  # part frame to world, arm level
    across: bool  # stress across the print layers


def sections(spec: Params) -> list[Section]:
    built = hero(spec)
    j1_lever, j3_lever = _reach(spec, built)
    cg_x = camera_cg(spec, built).X
    j1 = joints(spec)[0]
    t, flat = arm.link_t(spec), spec.ball_flat_x
    back_y = spec.bore_half_d + 2.0
    neck_z = arm.pedestal_height(spec) - spec.ball_r * 2 - arm.WAIST_R * 0.5
    head_x = flat + RING_T + 2 * arm.ring_gap()
    level = Rot(0, 0, 0)
    return [
        Section("collar back, bore to pivot", "collar_back",
                Plane(origin=(0, back_y, 0), z_dir=(0, 1, 0)),
                j1_lever + j1.Y - back_y, cg_x, level, False),
        Section("arm_base neck", "arm_base", Plane(origin=(0, 0, neck_z), z_dir=(0, 0, 1)),
                j1_lever, cg_x, level, True),
        Section("arm_link waist", "arm_link",
                Plane(origin=(0, spec.link_pitch / 2, 0), z_dir=(0, 1, 0)),
                j1_lever - spec.link_pitch / 2, cg_x - (flat + t / 2), level, False),
        Section("arm_tip waist", "arm_tip",
                Plane(origin=(0, spec.link_pitch / 2, 0), z_dir=(0, 1, 0)),
                j1_lever - 1.5 * spec.link_pitch, cg_x - (flat - t / 2), Rot(0, 180, 0), False),
        Section("arm_head block over J3", "arm_head",
                Plane(origin=(0, 0, 10.0), z_dir=(0, 0, 1)),
                j3_lever, cg_x - (head_x + arm.head_block_t() / 2), level, False),
        Section("arm_head platform root", "arm_head",
                Plane(origin=(-arm.HEAD_UNDER_SWEEP[0] - 1.0, 0, 0), z_dir=(1, 0, 0)),
                j3_lever, cg_x - (head_x - arm.HEAD_UNDER_SWEEP[0] - 1.0), level, False),
    ]


def stress_rows(spec: Params) -> list[Check]:
    """Sustained (0.1 x UTS) and 2x peak (0.25 x UTS) von Mises at each real section."""
    parts = printed_parts(spec)
    rows = []
    for section in sections(spec):
        world = _level_moment(spec, section.lever_y, section.lever_x)
        moment = (section.frame.inverse() * Pos(world)).position
        uts = PETG_UTS["across" if section.across else "along"]
        layers = "across layers" if section.across else "along layers"
        part = parts[section.part]
        rows.append(stress_row(part, section.plane, moment, 0.1 * uts,
                               f"{section.name} sustained, {layers}"))
        rows.append(stress_row(part, section.plane, moment * PEAK, 0.25 * uts,
                               f"{section.name} peak (2x), {layers}"))
    return rows


# --- Column and install ------------------------------------------------------------------------


def column_rows(spec: Params) -> list[Check]:
    """The hero reaches back over the column. Measure how high the column may run above the
    collar bottom before anything that swings over it would hit it."""
    built = hero(spec)
    footprint = column_solid(spec.column_w + 2 * COLUMN_CLEARANCE,
                             spec.column_d + 2 * COLUMN_CLEARANCE, spec.column_r
                             + COLUMN_CLEARANCE, spec.collar_h + 0.5, 1000.0)
    lowest = 1000.0
    for name, item in built.items.items():
        if item.chain == "fixed" or name == "column":
            continue
        overlap = item.part & footprint
        if overlap is not None and overlap.solids():
            lowest = min(lowest, overlap.bounding_box().min.Z)
    return [
        at_least("column top limit for the hero pose (from the collar bottom)", lowest,
                 spec.column_top_z),
        at_least("column top is above the collar", spec.column_top_z, spec.collar_h),
    ]


# --- Clash, joints, sweeps, access -------------------------------------------------------------


def clash_check_rows(spec: Params) -> list[Check]:
    built = hero(spec)
    solids = {name: item.part for name, item in built.items.items()}
    return clash_rows(solids, built.intended, "hero assembly")


def joint_rows(spec: Params) -> list[Check]:
    built = hero(spec)
    rows = []
    for label, left, right in (("J1", "arm_base", "arm_link"), ("J2", "arm_link", "arm_tip")):
        a, b = built.items[left].part, built.items[right].part
        volume = intersection_volume(a, b)
        rows.append(Check(f"{label} rosette engaged, no clash", volume <= VOL_EPS,
                          f"{volume:.2e} mm3"))
        rows.append(within(f"{label} rosette flank gap", a.distance_to(b), 0.05, 0.15))
    ring = built.items["j3_ring"].part
    for side in ("arm_tip", "arm_head"):
        gap = ring.distance_to(built.items[side].part)
        rows.append(within(f"J3 ring to {side} friction face", gap, 0.0, 0.03))
    pan = Vector(0, 0, 1)
    tilts = [X.rotate(Axis.Z, spec.hero_pan)] * 3
    worst = max(abs(axis.dot(pan)) for axis in tilts)
    rows.append(at_most("every tilt axis square to the pan axis (horizon stays level)",
                        worst, 1e-9, ""))
    return rows


def _sweep_bodies(built: Hero) -> dict[str, Item]:
    """Printed parts, the camera and the column: what can hit what in a sweep."""
    return {name: item for name, item in built.items.items()
            if item.printed or name in ("zve10 camera", "column")}


SKIPS = {
    "pan": {frozenset(("collar_back", "arm_base"))},
    "J1": {frozenset(("arm_base", "arm_link")), frozenset(("J1 knob", "arm_link"))},
    "J2": {frozenset(("arm_link", "arm_tip")), frozenset(("J2 knob", "arm_link"))},
    "J3": {frozenset(("arm_tip", "arm_head")), frozenset(("j3_ring", "arm_head")),
           frozenset(("J3 knob", "arm_tip"))},
}


def _sweep_setup(spec: Params, label: str):
    built = hero(spec)
    bodies = _sweep_bodies(built)
    j1, j2, j3 = joints(spec)
    axes = {"pan": Axis(pivot_world(spec), Z), "J1": Axis(j1, X), "J2": Axis(j2, X),
            "J3": Axis(j3, X)}
    index = ("pan", "J1", "J2", "J3").index(label)
    moving_chains = set(CHAINS[index + 1:])
    moving = {n: i.part for n, i in bodies.items() if i.chain in moving_chains}
    fixed = {n: i.part for n, i in bodies.items() if i.chain not in moving_chains}
    return axes[label], moving, fixed


def _sweep_step(job: tuple[Params, str, int]) -> tuple[str, int, bool]:
    spec, label, step = job
    axis, moving, fixed = _sweep_setup(spec, label)
    turned = {n: part.rotate(axis, step) for n, part in moving.items()}
    return label, step, _sweep_clear(turned, fixed, SKIPS[label])


def sweep_rows(spec: Params) -> list[Check]:
    """Pan and each joint alone through 360 degrees in 15 degree steps. Hero and folded
    must be clear.

    The swept joint's own faces (rosette, ring, or pivot face) are left out: the teeth
    repeat every 15 degrees and the friction faces are flat, so every step seats like the
    hero (checked in joint_rows). Steps run in parallel processes. They are spawned, not
    forked (OCCT's thread pools do not survive a fork), and each builds the hero once.
    """
    labels = ("pan", "J1", "J2", "J3")
    jobs = [(spec, label, step) for label in labels for step in range(0, 360, SWEEP_STEP)]
    workers = min(os.cpu_count() or 1, MAX_SWEEP_WORKERS)
    with ProcessPoolExecutor(max_workers=workers, mp_context=multiprocessing.get_context("spawn"),
                             initializer=hero, initargs=(spec,)) as pool:
        results = list(pool.map(_sweep_step, jobs))
    clear = {label: sorted(step for name, step, ok in results if name == label and ok)
             for label in labels}
    rows = [Check("pan sweep: hero pose clear", 0 in clear["pan"],
                  f"clear at {_ranges(clear['pan'], spec.hero_pan)} deg")]
    for index, label in enumerate(labels[1:]):
        hero_angle = spec.hero_angles[index]
        folded = spec.folded_angles[index] - hero_angle
        rows.append(Check(f"{label} sweep: hero pose clear", 0 in clear[label],
                          f"clear at {_ranges(clear[label], hero_angle)} deg (absolute)"))
        rows.append(Check(f"{label} sweep: folded pose clear", folded % 360 in clear[label],
                          f"folded at {spec.folded_angles[index]:g} deg"))
    return rows


def _sweep_clear(turned: dict[str, Part], fixed: dict[str, Part],
                 skip: set[frozenset[str]]) -> bool:
    for name, part in turned.items():
        for other, solid in fixed.items():
            if frozenset((name, other)) in skip:
                continue
            if not boxes_touch(part, solid):  # pruning only
                continue
            if intersection_volume(part, solid) > VOL_EPS:
                return False
    return True


def _ranges(steps: list[int], offset: float) -> str:
    if not steps:
        return "none"
    absolute = sorted({(s + offset) % 360 for s in steps})
    spans, start, last = [], absolute[0], absolute[0]
    for angle in absolute[1:]:
        if angle - last > SWEEP_STEP:
            spans.append((start, last))
            start = angle
        last = angle
    spans.append((start, last))
    return ", ".join(f"{a:g}..{b:g}" for a, b in spans)


def access_rows(spec: Params) -> list[Check]:
    """A key rod (key + 1 mm, 120 mm) from each keyed screw misses every part present then,
    and every knob has finger room all round it in the hero."""
    built = hero(spec)
    rows = []
    for screw, key, step, allowed in built.rods:
        present = [name for name, item in built.items.items()
                   if item.step <= step and name != screw and name not in allowed]
        hits = [name for name in present
                if boxes_touch(key, built.items[name].part)
                and intersection_volume(key, built.items[name].part) > VOL_EPS]
        rows.append(Check(f"key reaches {screw} at step {step}", not hits,
                          "clear" if not hits else "hits " + ", ".join(hits)))
    for knob, room, allowed in built.hands:
        hits = [name for name, item in built.items.items()
                if name not in allowed and boxes_touch(room, item.part)
                and intersection_volume(room, item.part) > VOL_EPS]
        rows.append(Check(f"finger room round {knob} ({FINGER:g} mm)", not hits,
                          "clear" if not hits else "hits " + ", ".join(hits)))
    return rows


def pan_drag_row(spec: Params) -> Check:
    """Torque to pan the arm with the pivot stopped: friction on the pivot face."""
    low, _nominal, _high = arm.pivot_stop().band(arm.pivot_stop().clamped,
                                                 arm_spread("pivot"))
    mean = (arm.foot_r(spec) + INSERT_M6.hole_d / 2) / 2
    return Check("pan drag with the pivot stopped (estimated mu 0.2)", True,
                 f"{MU_PETG * low * mean / 1000:.2f} Nm at the least stop force")


def bom(spec: Params) -> list[tuple[int, str, str, str]]:
    counts = Counter(item.bom for item in hero(spec).items.values() if item.bom)
    rows = [(qty, *key) for key, qty in counts.items()]
    return sorted(rows, key=lambda row: (row[2], row[1]))


def bom_text(spec: Params) -> str:
    lines = ["| Qty | Part | Standard | Source |", "| --- | --- | --- | --- |"]
    lines += [f"| {qty} | {name} | {standard} | {source} |"
              for qty, name, standard, source in bom(spec)]
    return "\n".join(lines)


def assembly_rows(spec: Params) -> list[Check]:
    built = hero(spec)
    rows = stop_rows(spec) + camera_rows(spec) + clash_check_rows(spec) + joint_rows(spec)
    rows += access_rows(spec)
    liners = (built.items["liner_front"].part, built.items["liner_back"].part)
    rows += collar.checks_pair(built.items["collar_front"].part,
                               built.items["collar_back"].part, liners,
                               built.items["column"].part, spec)
    rows += stress_rows(spec) + column_rows(spec) + [pan_drag_row(spec)]
    rows += sweep_rows(spec)
    return rows
