"""The finished device on the column, with every bought part as a solid.

Order of assembly (the access checks use these steps):
1. Press the eight M3 inserts (four in the seam faces, four in the MF) into collar_back.
   Hold collar_back MF face down, slide the 1/4" port nut in from the bore, turn it over:
   the nut drops into its pocket and cannot slide back.
2. Snap the liners in. They cover the port nut's slot mouth.
3. Close the halves round the column. 4 x M3 seam screws with washers from the -Y face.
4. Fix arm_base to the collar MF with 4 x M3 and washers from above.
5. Build J1, J2, J3: carriage bolt through, washer, knob. Head inserts and port nut first.
6. Fix zve10 to the head MF with 4 x M3 and washers. Then the camera, 1/4" screw from below.

Stacks are measured here by rays on the placed solids, not taken from the design numbers.
Loads: the payload weight at the camera, in the worst pose (both links level).
"""

from collections import Counter
from dataclasses import dataclass, field
from functools import cache

from build123d import Axis, Box, Cylinder, Location, Part, Plane, Pos, Rot, Vector, fillet

from holder import accessories, arm, collar
from holder.common import (
    MODEL_GAP,
    VOL_EPS,
    Check,
    Stack,
    at_least,
    at_most,
    boxes_touch,
    carriage_bolt,
    clash_rows,
    column_solid,
    first_hit,
    intersection_volume,
    nut_solid,
    rod,
    socket_screw,
    stress_row,
    thread_need,
    washer_solid,
    within,
)
from holder.mount import MountFace, foot_grip, foot_screw_length, m3_square
from holder.params import (
    INSERT_ENGAGE,
    INSERT_SINK,
    M3_INSERT,
    M3_SHCS,
    M3_WASHER,
    M3_WASHER_BORE,
    M6_CARRIAGE,
    M6_LARGE_WASHER,
    M6_NUT,
    PETG_UTS,
    PRELOAD_N,
    QUARTER_NUT,
    QUARTER_SHCS,
    QUARTER_WASHER_SLOT,
    ZVE10_BODY,
    ZVE10_THREAD_MAX,
    ZVE10_THREAD_MIN,
    Params,
    torque_nm,
)
from holder.stacks import (
    camera_grip,
    camera_screw_length,
    joint_screw_length,
    pitch_gap,
    rosette_preload_need,
)

X = Vector(1, 0, 0)
Y = Vector(0, 1, 0)
Z = Vector(0, 0, 1)
KEY_ROD = 120.0
SWEEP_STEP = 15
CHAINS = ("fixed", "link1", "link2", "head")  # a joint moves every chain after it
M3_MINOR = 2.459  # ISO 965-1 D1: the insert's thread bore, which the screw overlaps
COLUMN_CLEARANCE = 5.0  # mm kept between the column top and anything that swings over it
PEAK = 2.0  # peak load factor (DESIGN 2.4)


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
    head_location: Location = field(default_factory=Location)
    head_inserts: list[str] = field(default_factory=list)

    def add(self, name: str, item: Item) -> None:
        if name in self.items:
            raise ValueError(f"duplicate assembly item {name}")
        item.part.label = name
        self.items[name] = item


def _turn(alpha: float) -> Vector:
    return Vector(0, 1, 0).rotate(Axis.X, alpha)


def joints(spec: Params) -> tuple[Vector, Vector, Vector]:
    j1 = Vector(spec.arm_x0, spec.mf_y, spec.collar_h) + arm.j1_centre(spec) - Vector(
        spec.arm_x0, 0, 0)
    a1, a2, _ah = spec.hero_angles
    j2 = j1 + _turn(a1) * spec.link_pitch
    j3 = j2 + _turn(a2) * spec.link_pitch
    return j1, j2, j3


def _placed(part: Part, location: Location) -> Part:
    moved = location * part
    moved.label = part.label
    return moved


def link_location(spec: Params, start: Vector, alpha: float, flipped: bool) -> Location:
    turn = Rot(alpha, 0, 0) * (Rot(0, 180, 0) if flipped else Location())
    return Pos(start) * turn


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
        "arm_head": arm.build_head(spec), "knob": arm.build_knob(spec),
        **{name: builder(spec) for name, builder in accessories.BUILDERS.items()},
    }


def column(spec: Params) -> Part:
    return column_solid(spec.column_w, spec.column_d, spec.column_r, -60, spec.column_top_z)


@cache
def hero(spec: Params) -> Hero:
    parts = printed_parts(spec)
    built = Hero()
    built.add("column", Item(column(spec), "column", 0))
    built.add("collar_front", Item(parts["collar_front"], "petg", 3, printed=True))
    built.add("collar_back", Item(parts["collar_back"], "petg", 1, printed=True))
    built.add("liner_front", Item(parts["liner_front"], "tpu", 2, printed=True))
    built.add("liner_back", Item(parts["liner_back"], "tpu", 2, printed=True))
    _seam_hardware(built, spec)
    _arm(built, spec, parts)
    _zve10(built, spec, parts)
    return built


def _insert(built: Hero, name: str, seated, host: str, location: Location, chain: str,
            step: int) -> None:
    built.add(name, Item(location * seated.solid(M3_MINOR), "brass", step, chain,
                         bom=_bought(M3_INSERT.name, M3_INSERT.standard, M3_INSERT.source)))
    built.intended.add(frozenset((name, host)))  # the knurl melts into the hole wall


def _seam_hardware(built: Hero, spec: Params) -> None:
    length = collar.seam_screw_length(spec)
    for index, seated in enumerate(collar.seam_inserts(spec)):
        x, z = seated.face.X, seated.face.Z
        washer_at = Vector(x, spec.seam_seat_y - MODEL_GAP - M3_WASHER.t, z)
        head_at = washer_at - Y * MODEL_GAP
        screw, insert = f"seam screw {index + 1}", f"seam insert {index + 1}"
        _insert(built, insert, seated, "collar_back", Location(), "fixed", 1)
        built.add(screw, Item(socket_screw(M3_SHCS, length, head_at, Y), "black_steel", 3,
                              bom=_bought(f"M3x{length:g}", M3_SHCS.standard, M3_SHCS.source)))
        built.add(f"seam washer {index + 1}", Item(washer_solid(M3_WASHER, washer_at, Y),
                                                   "steel", 3, bom=_bought(
                                                       "M3 washer", M3_WASHER.standard,
                                                       M3_WASHER.source)))
        built.intended.add(frozenset((screw, insert)))
        built.rods.append((screw, _key_rod(head_at, Y, M3_SHCS), 3, {insert}))


def _key_rod(under_head: Vector, axis: Vector, screw) -> Part:
    """Key plus 1 mm, from the socket outward along the screw axis."""
    top = under_head - axis * (screw.head_k - 0.5)
    return rod(screw.key + 1.0, KEY_ROD, top, -axis)


def _mf_hardware(built: Hero, face: MountFace, spec: Params, location: Location, host: str,
                 chain: str, prefix: str, step: int) -> list[str]:
    """The MF's inserts, and its port nut resting in its pocket (no stud in the hero)."""
    names = []
    for index, seated in enumerate(face.inserts(spec)):
        name = f"{prefix} insert {index + 1}"
        _insert(built, name, seated, host, location, chain, step)
        names.append(name)
    nut = face.port_nut()
    built.add(f"{prefix} 1/4\" nut", Item(location * nut.resting(), "steel", step, chain,
                                          bom=_bought(QUARTER_NUT.name, QUARTER_NUT.standard,
                                                      QUARTER_NUT.source)))
    return names


def _foot_screws(built: Hero, spec: Params, insert_names: list[str], face: MountFace,
                 location: Location, chain: str, prefix: str, step: int) -> None:
    length = foot_screw_length(spec)
    for (dx, dy), insert in zip(m3_square(spec), insert_names):
        top = Vector(face.centre.X + dx, face.centre.Y + dy, face.centre.Z)
        washer_at = _point(location, top + Z * (spec.foot_under_head + M3_WASHER.t + MODEL_GAP))
        head_at = washer_at + Z * MODEL_GAP
        name = f"{prefix} screw ({dx:+g},{dy:+g})"
        built.add(name, Item(socket_screw(M3_SHCS, length, head_at, -Z), "black_steel", step,
                             chain, bom=_bought(f"M3x{length:g}", M3_SHCS.standard,
                                                M3_SHCS.source)))
        built.add(f"{prefix} washer ({dx:+g},{dy:+g})",
                  Item(washer_solid(M3_WASHER, washer_at, -Z), "steel", step, chain,
                       bom=_bought("M3 washer", M3_WASHER.standard, M3_WASHER.source)))
        built.intended.add(frozenset((name, insert)))
        built.rods.append((name, _key_rod(head_at, -Z, M3_SHCS), step, {insert}))


def _arm(built: Hero, spec: Params, parts: dict[str, Part]) -> None:
    j1, j2, j3 = joints(spec)
    a1, a2, ah = spec.hero_angles
    gap = pitch_gap(spec)
    mf = collar.mount_face(spec)
    collar_inserts = _mf_hardware(built, mf, spec, Location(), "collar_back", "fixed",
                                  "collar MF", 1)
    base_at = Pos(0, spec.mf_y, spec.collar_h)
    built.add("arm_base", Item(_placed(parts["arm_base"], base_at), "petg", 4, printed=True))
    _foot_screws(built, spec, collar_inserts, mf, Location(), "fixed", "arm_base", 4)
    l1 = link_location(spec, Vector(spec.arm_x0 + gap, j1.Y, j1.Z), a1, flipped=False)
    l2 = link_location(spec, Vector(spec.arm_x0, j2.Y, j2.Z), a2, flipped=True)
    hd = Pos(spec.arm_x0 + gap, j3.Y, j3.Z) * Rot(ah, 0, 0)
    built.add("arm_link 1", Item(_placed(parts["arm_link"], l1), "petg", 5, "link1",
                                 printed=True))
    built.add("arm_link 2", Item(_placed(parts["arm_link"], l2), "petg", 5, "link2",
                                 printed=True))
    built.add("arm_head", Item(_placed(parts["arm_head"], hd), "petg", 5, "head", printed=True))
    t, x0 = spec.knuckle_t, spec.arm_x0
    hardware = [
        JointHardware("J1", j1, x0 - t, X, x0 + gap + t, "fixed", "link1"),
        JointHardware("J2", j2, x0 + gap + t, -X, x0 - t, "link1", "link2"),
        JointHardware("J3", j3, x0 - t, X, x0 + gap + t, "link2", "head"),
    ]
    for joint in hardware:
        _joint_hardware(built, spec, joint, parts["knob"])
    built.head_location = hd
    built.head_inserts = _mf_hardware(built, arm.head_mount_face(spec), spec, hd, "arm_head",
                                      "head", "head MF", 5)


@dataclass(frozen=True)
class JointHardware:
    """Where one joint's bolt, washer, knob and nut go. The knob sits on +axis."""

    label: str
    centre: Vector
    head_x: float  # the knuckle face the dome head bears on
    axis: Vector  # bolt direction, head toward knob
    washer_x: float  # the knuckle face the washer bears on, on the knob side
    bolt_chain: str
    knob_chain: str


def _joint_hardware(built: Hero, spec: Params, joint: JointHardware, knob: Part) -> None:
    label, centre, axis = joint.label, joint.centre, joint.axis
    length = joint_screw_length(spec)
    under_head = Vector(joint.head_x, centre.Y, centre.Z) - axis * MODEL_GAP
    bolt, nut_name = f"{label} bolt", f"{label} nut"
    built.add(bolt, Item(carriage_bolt(M6_CARRIAGE, length, under_head, axis, Z), "steel", 5,
                         joint.bolt_chain, bom=_bought(f"M6x{length:g}", M6_CARRIAGE.standard,
                                                       M6_CARRIAGE.source)))
    washer_at = Vector(joint.washer_x, centre.Y, centre.Z) + axis * MODEL_GAP
    built.add(f"{label} washer", Item(washer_solid(M6_LARGE_WASHER, washer_at, axis), "steel",
                                      5, joint.knob_chain, bom=_bought(
                                          M6_LARGE_WASHER.name, M6_LARGE_WASHER.standard,
                                          M6_LARGE_WASHER.source)))
    base = washer_at + axis * (M6_LARGE_WASHER.t + MODEL_GAP)
    place = Pos(base) * Rot(0, 90 if axis.X > 0 else -90, 0)
    built.add(f"{label} knob", Item(_placed(knob, place), "petg", 5, joint.knob_chain,
                                    printed=True))
    nut_corner = Vector(0, 0, -1 if axis.X > 0 else 1)
    built.add(nut_name, Item(nut_solid(M6_NUT, base + axis * MODEL_GAP, axis, nut_corner),
                             "steel", 5, joint.knob_chain,
                             bom=_bought("M6 nut", M6_NUT.standard, M6_NUT.source)))
    built.intended.add(frozenset((bolt, nut_name)))


def camera_envelope(socket: Vector) -> Part:
    """ZV-E10 body (Sony help guide) on the plate, lens toward -Y, socket at `socket`.

    The socket is a hole ZVE10_THREAD_MAX deep, so a screw that is too long clashes.
    The 3 mm edge rounds are for the picture only.
    """
    width, height, depth = ZVE10_BODY
    body = Pos(socket.X, socket.Y, socket.Z + height / 2) * Box(width, depth, height)
    body = fillet(body.edges(), 3.0)
    body -= rod(QUARTER_SHCS.d + 0.05, ZVE10_THREAD_MAX, socket - Z * 0.01, Z)
    # estimated: SELP1650 kit lens 64.7 x 29.9 mm (Sony spec sheet); render context only.
    lens = Pos(socket.X - 12, socket.Y - depth / 2 - 15, socket.Z + 34) * (
        Rot(90, 0, 0) * Cylinder(64.7 / 2, 30))
    return body + lens


def _zve10(built: Hero, spec: Params, parts: dict[str, Part]) -> None:
    hd = built.head_location
    face = arm.head_mount_face(spec)
    plate_at = hd * Pos(face.centre.X, face.centre.Y, face.centre.Z)
    built.add("zve10", Item(_placed(parts["zve10"], plate_at), "petg", 6, "head", printed=True))
    _foot_screws(built, spec, built.head_inserts, face, hd, "head", "zve10", 6)
    length = camera_screw_length()
    thickness = accessories.camera_plate_t()
    ceiling = _point(plate_at, Vector(0, accessories.zve10_camera_y(), QUARTER_WASHER_SLOT[1]))
    washer_at = ceiling - Z * (MODEL_GAP + M6_LARGE_WASHER.t)
    head_at = washer_at - Z * MODEL_GAP
    built.add("camera washer", Item(washer_solid(M6_LARGE_WASHER, washer_at, Z), "steel", 6,
                                    "head", bom=_bought(M6_LARGE_WASHER.name,
                                                        M6_LARGE_WASHER.standard,
                                                        M6_LARGE_WASHER.source)))
    built.add("camera screw", Item(socket_screw(QUARTER_SHCS, length, head_at, Z),
                                   "black_steel", 6, "head",
                                   bom=_bought(f"1/4\"-20 x {length / 25.4:.3g}\"",
                                               QUARTER_SHCS.standard, QUARTER_SHCS.source)))
    socket = _point(plate_at, Vector(0, accessories.zve10_camera_y(), thickness)) + Z * MODEL_GAP
    built.add("zve10 camera", Item(camera_envelope(socket), "camera", 6, "head"))
    built.rods.append(("camera screw", _key_rod(head_at, Z, QUARTER_SHCS), 6, {"zve10 camera"}))


# --- Measured stacks ---------------------------------------------------------------------------


def _ray(part: Part, origin: Vector, direction: Vector) -> Vector:
    distance = first_hit(part, origin, direction)
    if distance == float("inf"):
        raise ValueError(f"ray from {origin} misses {part.label}")
    return origin + direction.normalized() * distance


def _length_matches(stack: Stack, built_length: float) -> list[Check]:
    ok = abs(stack.length - built_length) < 1e-9
    return [Check(f"{stack.joint} solid is the stack length", ok,
                  f"measured stack {stack.length:g}, modeled screw {built_length:g}")]


def _seam_stacks(built: Hero, spec: Params) -> list[Check]:
    front, back = built.items["collar_front"].part, built.items["collar_back"].part
    rows = []
    seam_gap = front.distance_to(back)
    head_bore = M3_WASHER_BORE[0] / 2
    for index, seated in enumerate(collar.seam_inserts(spec)):
        x, z = seated.face.X, seated.face.Z
        side = Vector(M3_WASHER.outer / 2 - 0.6, 0, 0)  # on the washer seat, past the hole
        seat = _ray(front, Vector(x, -80, z) + side, Y).Y
        load = _ray(back, Vector(x, 0.0, z) + Vector(M3_INSERT.hole_d / 2 + 0.6, 0, 0), Y).Y
        floor = _ray(back, Vector(x, load + 0.5, z), Y).Y
        rim = max(_ray(front, Vector(x, -80, z) + offset, Y).Y
                  for offset in (X * (head_bore + 0.3), -X * (head_bore + 0.3),
                                 Z * (head_bore + 0.3)))
        grip = load - seat + M3_WASHER.t
        sink = (seat - rim) - M3_WASHER.t - M3_SHCS.head_k
        stack = Stack(f"seam {index + 1}", M3_SHCS, grip, grip - seam_gap,
                      INSERT_SINK + INSERT_ENGAGE, floor - load - 0.5, sink)
        rows += _length_matches(stack, collar.seam_screw_length(spec)) + stack.rows()
    return rows


@dataclass(frozen=True)
class FootJoint:
    foot: str
    host: str
    face: MountFace
    location: Location
    label: str
    foot_top: float  # the foot's top face above the MF face


def _foot_stacks(built: Hero, spec: Params, joint: FootJoint) -> list[Check]:
    """Grip from the washer seat to the MF face; the tip room is the insert hole floor.

    The rim ray runs outward from the foot centre, so it meets the foot top, never the
    part standing on the foot. A row checks that it did.
    """
    foot_part, host_part = built.items[joint.foot].part, built.items[joint.host].part
    face, location, label, foot_top = joint.face, joint.location, joint.label, joint.foot_top
    top = _point(location, face.centre).Z
    rows = []
    for dx, dy in m3_square(spec):
        axis_top = _point(location, Vector(face.centre.X + dx, face.centre.Y + dy,
                                           face.centre.Z + 50))
        outward = Vector(dx, dy, 0).normalized()
        seat = _ray(foot_part, axis_top + outward * (M3_WASHER.outer / 2 - 0.6), -Z).Z
        rim = _ray(foot_part, axis_top + outward * (M3_WASHER_BORE[0] / 2 + 0.4), -Z).Z
        load = _ray(host_part, axis_top + outward * (M3_INSERT.hole_d / 2 + 0.6), -Z).Z
        floor = _ray(host_part, Vector(axis_top.X, axis_top.Y, load - 0.5), -Z).Z
        grip = seat + M3_WASHER.t - load
        stack = Stack(f"{label} ({dx:+g},{dy:+g})", M3_SHCS, grip, grip,
                      INSERT_SINK + INSERT_ENGAGE, load - floor - 0.5,
                      (rim - seat) - M3_WASHER.t - M3_SHCS.head_k)
        rows += _length_matches(stack, foot_screw_length(spec)) + stack.rows()
        rows.append(within(f"{label} ({dx:+g},{dy:+g}) rim ray is on the foot top",
                           rim - top, foot_top - 0.05, foot_top + 0.05))
        rows.append(within(f"{label} ({dx:+g},{dy:+g}) grip is the foot grip", grip,
                           foot_grip(spec) - 0.05, foot_grip(spec) + 0.05))
    return rows


def _joint_stacks(built: Hero, spec: Params) -> list[Check]:
    rows = []
    j1, j2, j3 = joints(spec)
    hosts = (("J1", j1, "arm_base", X), ("J2", j2, "arm_link 1", -X), ("J3", j3, "arm_link 2", X))
    for label, centre, host, axis in hosts:
        side = Vector(0, 3.5, 4.5)  # inside the dome footprint, outside the square neck hole
        face = _ray(built.items[host].part, centre - axis * 80 + side, axis)
        washer = _ray(built.items[f"{label} washer"].part, centre + axis * 80 + side, -axis)
        grip = (washer - face).dot(axis)
        knob = built.items[f"{label} knob"].part
        bore = _ray(knob, Vector(washer.X, centre.Y, centre.Z) + axis * 0.01, axis)
        room = (bore - washer).dot(axis)
        stack = Stack(f"{label} arm joint", M6_CARRIAGE, grip + spec.index_lift, grip,
                      thread_need(M6_CARRIAGE, M6_NUT), room - 0.5)
        rows += _length_matches(stack, joint_screw_length(spec)) + stack.rows()
    return rows


def _camera_stack(built: Hero) -> list[Check]:
    plate = built.items["zve10"].part
    screw_axis = built.items["camera screw"].part.bounding_box().center()
    side = Vector(0, QUARTER_WASHER_SLOT[0] / 4 + 1.6, 0)  # across the slot, onto the ceiling
    below = Vector(screw_axis.X, screw_axis.Y, plate.bounding_box().min.Z - 20)
    ceiling = _ray(plate, below + side, Z).Z
    top = _ray(plate, below + side + Z * 100, -Z).Z
    grip = top - ceiling + M6_LARGE_WASHER.t
    stack = Stack("camera screw", QUARTER_SHCS, grip, grip, ZVE10_THREAD_MIN, ZVE10_THREAD_MAX)
    rows = _length_matches(stack, camera_screw_length()) + stack.rows()
    rows.append(within("camera grip is the design grip", grip, camera_grip() - 0.05,
                       camera_grip() + 0.05))
    return rows


def stack_rows(spec: Params) -> list[Check]:
    built = hero(spec)
    rows = _seam_stacks(built, spec)
    rows += _foot_stacks(built, spec, FootJoint("arm_base", "collar_back",
                                                collar.mount_face(spec), Location(),
                                                "arm_base foot", spec.foot_t))
    rows += _foot_stacks(built, spec, FootJoint("zve10", "arm_head", arm.head_mount_face(spec),
                                                built.head_location, "zve10 foot",
                                                accessories.camera_plate_t()))
    rows += _joint_stacks(built, spec)
    rows += _camera_stack(built)
    return rows


# --- Loads: preload needs and section stress -----------------------------------------------------


def payload_n(spec: Params) -> float:
    return spec.payload_kg * 9.81


def camera_x(spec: Params) -> float:
    """World X of the camera's centre: on the head MF centre (all joints turn about X)."""
    return spec.arm_x0 + pitch_gap(spec) + spec.knuckle_t / 2


def payload_lever(spec: Params) -> float:
    """Worst lever from J1 to the camera: both links level, plus the deck offset."""
    return 2 * spec.link_pitch + abs(accessories.zve10_camera_y())


def _level_moment(spec: Params, lever_y: float, lever_x: float) -> Vector:
    """Moment (N mm, world axes) of the payload about a point lever_y behind and lever_x
    beside the camera, with the arm level: M = r x F, F = -w Z."""
    weight = payload_n(spec)
    return Vector(-weight * lever_y, weight * lever_x, 0)


def preload_rows(spec: Params) -> list[Check]:
    """Each joint's preload against what the 2x load needs from it."""
    rows = []
    j1 = joints(spec)[0]
    levers = {"J1": payload_lever(spec), "J2": payload_lever(spec) - spec.link_pitch,
              "J3": abs(accessories.zve10_camera_y())}
    for label, lever in levers.items():
        need = rosette_preload_need(spec, PEAK * payload_n(spec) * lever)
        rows.append(at_most(f"{label} rosette: preload the 2x moment needs", need,
                            PRELOAD_N["joint"], "N"))
    rows.append(Check("joint knob torque for the preload", True,
                      f"{torque_nm(PRELOAD_N['joint'], M6_CARRIAGE):.2f} Nm (estimated K 0.2)"))
    # Foot on the collar MF: it tips about the foot edge; the far screw pair carries it.
    half, foot = spec.mf_m3_spacing / 2, spec.mf_size / 2
    moment = PEAK * payload_n(spec) * (payload_lever(spec) + j1.Y - spec.mf_y)
    pull = moment / (2 * (foot + half))
    rows.append(at_most("arm_base foot: 2x bolt pull stays under the M3 preload", pull,
                        PRELOAD_N["m3"], "N"))
    # Collar: the clamp must hold the 2x tilting couple across its height, and the weight.
    couple = moment / (2 * spec.collar_h / 3)
    clamp = 2 * PRELOAD_N["m3"]  # two seam screws per side press each liner face
    rows.append(at_most("collar clamp: 2x tilting couple under the seam preload", couple,
                        2 * clamp, "N"))
    rows.append(Check("M3 torque for the preload", True,
                      f"{torque_nm(PRELOAD_N['m3'], M3_SHCS):.3f} Nm (estimated K 0.2)"))
    return rows


@dataclass(frozen=True)
class Section:
    name: str
    part: str  # printed part, in its own frame
    plane: Plane  # in the part frame
    lever_y: float  # horizontal distance to the camera, arm level
    lever_x: float  # camera x minus the section centre x, world
    frame: Rot  # part frame to world, arm level
    across: bool  # stress across the print layers


def sections(spec: Params) -> list[Section]:
    j1 = joints(spec)[0]
    cam_x = camera_x(spec)
    root_y = collar.spine_root_y(spec)
    base_root = spec.foot_t + 4.0
    upright_x = spec.arm_x0 - spec.knuckle_t / 2
    link1_x = spec.arm_x0 + pitch_gap(spec) + spec.knuckle_t / 2
    link2_x = spec.arm_x0 - spec.knuckle_t / 2
    level = Rot(0, 0, 0)
    return [
        Section("collar spine root", "collar_back",
                Plane(origin=(0, root_y, 0), z_dir=(0, 1, 0)),
                payload_lever(spec) + j1.Y - root_y, cam_x, level, False),
        Section("arm_base upright root", "arm_base",
                Plane(origin=(0, 0, base_root), z_dir=(0, 0, 1)),
                payload_lever(spec), cam_x - upright_x, level, True),
        Section("arm_link 1 waist", "arm_link",
                Plane(origin=(0, spec.link_pitch / 2, 0), z_dir=(0, 1, 0)),
                payload_lever(spec) - spec.link_pitch / 2, cam_x - link1_x, level, False),
        Section("arm_link 2 waist", "arm_link",
                Plane(origin=(0, spec.link_pitch / 2, 0), z_dir=(0, 1, 0)),
                payload_lever(spec) - 1.5 * spec.link_pitch, cam_x - link2_x,
                Rot(0, 180, 0), False),
        Section("arm_head neck", "arm_head",
                Plane(origin=(0, 0, spec.head_block_gap + 2.0), z_dir=(0, 0, 1)),
                abs(accessories.zve10_camera_y()), 0.0, level, True),
        Section("zve10 shoulder", "zve10",
                Plane(origin=(0, -20.5, 0), z_dir=(0, 1, 0)),
                abs(accessories.zve10_camera_y()) - 20.5, 0.0, level, False),
        Section("post column", "post", Plane(origin=(0, 0, spec.foot_t + 8.0), z_dir=(0, 0, 1)),
                payload_lever(spec), cam_x, level, True),
    ]


def stress_rows(spec: Params) -> list[Check]:
    """Sustained (0.1 x UTS) and 2x peak (0.25 x UTS) von Mises at each real section.

    The moment is the payload in the worst pose, as a vector, turned into the part frame:
    bending about both section axes and twist about its normal.
    """
    parts = printed_parts(spec)
    rows = []
    for section in sections(spec):
        world = _level_moment(spec, section.lever_y, section.lever_x)
        local = section.frame.inverse() * Pos(world)
        moment = local.position
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


# --- Clash, rosettes, sweeps, access -------------------------------------------------------------


def clash_check_rows(spec: Params) -> list[Check]:
    built = hero(spec)
    solids = {name: item.part for name, item in built.items.items()}
    return clash_rows(solids, built.intended, "hero assembly")


def rosette_rows(spec: Params) -> list[Check]:
    built = hero(spec)
    pairs = (("J1", "arm_base", "arm_link 1"), ("J2", "arm_link 1", "arm_link 2"),
             ("J3", "arm_link 2", "arm_head"))
    rows = []
    for label, left, right in pairs:
        a, b = built.items[left].part, built.items[right].part
        volume = intersection_volume(a, b)
        rows.append(Check(f"{label} rosette engaged, no clash", volume <= VOL_EPS,
                          f"{volume:.2e} mm3"))
        rows.append(within(f"{label} rosette flank gap", a.distance_to(b), 0.05, 0.15))
    return rows


def _rotated(item: Item, axis: Axis, angle: float) -> Part:
    return item.part.rotate(axis, angle)


def _sweep_bodies(built: Hero) -> dict[str, Item]:
    """Printed parts, knobs and the camera: the bodies that can hit each other in a sweep."""
    return {name: item for name, item in built.items.items()
            if item.printed or name in ("zve10 camera", "column")}


def sweep_rows(spec: Params) -> list[Check]:
    """Each joint alone through 360 degrees in 15 degree steps. Hero and folded must be clear.

    The engaged rosette pair of the swept joint is left out: its teeth repeat every 15
    degrees, so every step meshes like the hero (checked in rosette_rows).
    """
    built = hero(spec)
    bodies = _sweep_bodies(built)
    centres = joints(spec)
    pairs = {"J1": ("arm_base", "arm_link 1"), "J2": ("arm_link 1", "arm_link 2"),
             "J3": ("arm_link 2", "arm_head")}
    rows = []
    for index, label in enumerate(("J1", "J2", "J3")):
        moving_chains = set(CHAINS[index + 1:])
        axis = Axis(centres[index], X)
        moving = {n: i for n, i in bodies.items() if i.chain in moving_chains}
        fixed = {n: i.part for n, i in bodies.items() if i.chain not in moving_chains}
        clear = []
        for step in range(0, 360, SWEEP_STEP):
            turned = {n: _rotated(i, axis, step) for n, i in moving.items()}
            if _sweep_clear(turned, fixed, pairs[label]):
                clear.append(step)
        hero_angle = spec.hero_angles[index]
        folded = spec.folded_angles[index] - hero_angle
        rows.append(Check(f"{label} sweep: hero pose clear", 0 in clear,
                          f"clear at {_ranges(clear, hero_angle)} deg (absolute)"))
        rows.append(Check(f"{label} sweep: folded pose clear", folded % 360 in clear,
                          f"folded at {spec.folded_angles[index]:g} deg"))
    return rows


def _sweep_clear(turned: dict[str, Part], fixed: dict[str, Part], skip: tuple[str, str]) -> bool:
    for name, part in turned.items():
        for other, solid in fixed.items():
            if {name, other} == set(skip):
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
    """A key rod (key + 1 mm, 120 mm) from each screw head misses every part present then."""
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
    return rows


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
    rows = stack_rows(spec) + clash_check_rows(spec) + rosette_rows(spec) + access_rows(spec)
    built = hero(spec)
    liners = (built.items["liner_front"].part, built.items["liner_back"].part)
    rows += collar.checks_pair(built.items["collar_front"].part,
                               built.items["collar_back"].part, liners,
                               built.items["column"].part, spec)
    rows += preload_rows(spec) + stress_rows(spec) + column_rows(spec)
    rows += sweep_rows(spec)
    return rows
