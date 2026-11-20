"""The finished device on the column, with every bought part as a solid.

Order of assembly (the access checks use these steps):
1. Slide the MF nuts and the seam nuts into collar_back from the bore side.
2. Snap the liners in. They cover the slot mouths and keep the nuts in.
3. Close the halves round the column. 4 x M3 seam screws with washers from the -Y face.
4. Fix arm_base to the collar MF with 4 x M3 from above.
5. Build J1, J2, J3: hex bolt in its pocket, washer, knob. Head MF nuts go in first.
6. Fix zve10 to the head MF with 4 x M3. Then the camera, 1/4" screw from below.

Stacks are measured here by rays on the placed solids, not taken from the design numbers.
"""

from collections import Counter
from dataclasses import dataclass, field
from functools import cache

from build123d import Axis, Box, Cylinder, Location, Part, Pos, Rot, Vector, fillet

from holder import accessories, arm, collar
from holder.common import (
    MODEL_GAP,
    VOL_EPS,
    Check,
    Stack,
    boxes_touch,
    clash_rows,
    column_solid,
    first_hit,
    hex_bolt,
    intersection_volume,
    nut_solid,
    rod,
    socket_screw,
    thread_need,
    washer_solid,
    within,
)
from holder.mount import MountFace, foot_screw_length, m3_square
from holder.params import (
    M3_HEAD_BORE,
    M3_NUT,
    M3_SHCS,
    M3_WASHER,
    M3_WASHER_BORE,
    M6_HEAD_POCKET,
    M6_HEX_BOLT,
    M6_NUT,
    M6_WASHER,
    M6_WASHER_RECESS,
    QUARTER_HEAD_BORE,
    QUARTER_NUT,
    QUARTER_SHCS,
    ZVE10_BODY,
    ZVE10_THREAD_MAX,
    ZVE10_THREAD_MIN,
    Params,
)
from holder.stacks import camera_design, joint_design, pitch_gap, seam_design

X = Vector(1, 0, 0)
Y = Vector(0, 1, 0)
Z = Vector(0, 0, 1)
KEY_ROD = 120.0
SWEEP_STEP = 15
CHAINS = ("fixed", "link1", "link2", "head")  # a joint moves every chain after it


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
    head_nuts: list[str] = field(default_factory=list)

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


@cache
def hero(spec: Params) -> Hero:
    parts = printed_parts(spec)
    built = Hero()
    column = column_solid(spec.column_w, spec.column_d, spec.column_r, -60, 100)
    built.add("column", Item(column, "column", 0))
    built.add("collar_front", Item(parts["collar_front"], "petg", 3, printed=True))
    built.add("collar_back", Item(parts["collar_back"], "petg", 1, printed=True))
    built.add("liner_front", Item(parts["liner_front"], "tpu", 2, printed=True))
    built.add("liner_back", Item(parts["liner_back"], "tpu", 2, printed=True))
    _seam_hardware(built, spec)
    _arm(built, spec, parts)
    _zve10(built, spec, parts)
    return built


def _bought(screw_like: str, standard: str, source: str) -> tuple[str, str, str]:
    return (screw_like, standard, source)


def _seam_hardware(built: Hero, spec: Params) -> None:
    stack = seam_design(spec)
    for index, nut in enumerate(collar.seam_nuts(spec)):
        x, z = nut.seat.X, nut.seat.Z
        washer_at = Vector(x, spec.seam_seat_y - MODEL_GAP - M3_WASHER.t, z)
        head_at = washer_at - Y * MODEL_GAP
        screw, nut_name = f"seam screw {index + 1}", f"seam nut {index + 1}"
        built.add(screw, Item(socket_screw(M3_SHCS, stack.length, head_at, Y), "black_steel", 3,
                              bom=_bought(f"M3x{stack.length:g}", M3_SHCS.standard,
                                          M3_SHCS.source)))
        built.add(f"seam washer {index + 1}", Item(washer_solid(M3_WASHER, washer_at, Y), "steel",
                                                   3, bom=_bought("M3 washer", M3_WASHER.standard,
                                                                  M3_WASHER.source)))
        built.add(nut_name, Item(nut.nut_at_seat(), "steel", 1,
                                 bom=_bought("M3 nut", M3_NUT.standard, M3_NUT.source)))
        built.intended.add(frozenset((screw, nut_name)))
        built.rods.append((screw, _key_rod(head_at, Y, M3_SHCS), 3, {nut_name}))


def _key_rod(under_head: Vector, axis: Vector, screw) -> Part:
    """Key plus 1 mm, from the socket outward along the screw axis."""
    top = under_head - axis * (screw.head_k - 0.5)
    return rod(screw.key + 1.0, KEY_ROD, top, -axis)


def _mf_nuts(built: Hero, face: MountFace, spec: Params, location: Location, chain: str,
             prefix: str, step: int) -> list[str]:
    names = []
    for nut in face.nuts(spec):
        name = f"{prefix} {nut.nut.name} {len(names) + 1}"
        source = (M3_NUT if nut.nut is M3_NUT else QUARTER_NUT)
        built.add(name, Item(location * nut.nut_at_seat(), "steel", step, chain,
                             bom=_bought(source.name, source.standard, source.source)))
        names.append(name)
    return names


def _foot_screws(built: Hero, spec: Params, nut_names: list[str], face: MountFace,
                 location: Location, chain: str, prefix: str, step: int) -> None:
    length = foot_screw_length(spec)
    order = [(n.seat.X - face.centre.X, n.seat.Y - face.centre.Y) for n in face.nuts(spec)]
    for dx, dy in m3_square(spec):
        local = Vector(face.centre.X + dx, face.centre.Y + dy,
                       face.centre.Z + spec.foot_under_head + MODEL_GAP)
        head_at = _point(location, local)
        name = f"{prefix} screw ({dx:+g},{dy:+g})"
        nut_name = nut_names[order.index((dx, dy))]
        built.add(name, Item(socket_screw(M3_SHCS, length, head_at, -Z), "black_steel", step,
                             chain, bom=_bought(f"M3x{length:g}", M3_SHCS.standard,
                                                M3_SHCS.source)))
        built.intended.add(frozenset((name, nut_name)))
        built.rods.append((name, _key_rod(head_at, -Z, M3_SHCS), step, {nut_name}))


def _arm(built: Hero, spec: Params, parts: dict[str, Part]) -> None:
    j1, j2, j3 = joints(spec)
    a1, a2, ah = spec.hero_angles
    gap = pitch_gap(spec)
    mf = collar.mount_face(spec)
    collar_nuts = _mf_nuts(built, mf, spec, Location(), "fixed", "collar MF", 1)
    base_at = Pos(0, spec.mf_y, spec.collar_h)
    built.add("arm_base", Item(_placed(parts["arm_base"], base_at), "petg", 4, printed=True))
    _foot_screws(built, spec, collar_nuts[:4], mf, Location(), "fixed", "arm_base", 4)
    l1 = link_location(spec, Vector(spec.arm_x0 + gap, j1.Y, j1.Z), a1, flipped=False)
    l2 = link_location(spec, Vector(spec.arm_x0, j2.Y, j2.Z), a2, flipped=True)
    hd = Pos(spec.arm_x0 + gap, j3.Y, j3.Z) * Rot(ah, 0, 0)
    built.add("arm_link 1", Item(_placed(parts["arm_link"], l1), "petg", 5, "link1",
                                 printed=True))
    built.add("arm_link 2", Item(_placed(parts["arm_link"], l2), "petg", 5, "link2",
                                 printed=True))
    built.add("arm_head", Item(_placed(parts["arm_head"], hd), "petg", 5, "head", printed=True))
    t = spec.knuckle_t
    x0 = spec.arm_x0
    pocket, recess = M6_HEAD_POCKET[1], M6_WASHER_RECESS[1]
    hardware = [
        JointHardware("J1", j1, x0 - t + pocket, X, Y, x0 + gap + t - recess, "fixed", "link1"),
        JointHardware("J2", j2, x0 + gap + t - pocket, -X, _turn(a1), x0 - t + recess, "link1",
                      "link2"),
        JointHardware("J3", j3, x0 - t + pocket, X, _turn(a2), x0 + gap + t - recess, "link2",
                      "head"),
    ]
    for joint in hardware:
        _joint_hardware(built, spec, joint, parts["knob"])
    built.head_location = hd
    built.head_nuts = _mf_nuts(built, arm.head_mount_face(spec), spec, hd, "head", "head MF", 5)


@dataclass(frozen=True)
class JointHardware:
    """Where one joint's bolt, washer, knob and nut go. The knob sits on +axis."""

    label: str
    centre: Vector
    floor_x: float  # hex pocket floor, on the bolt-head part
    axis: Vector  # bolt direction, head toward knob
    corner: Vector  # hex corner direction of the head pocket
    recess_x: float  # washer recess floor, on the knob part
    bolt_chain: str
    knob_chain: str


def _joint_hardware(built: Hero, spec: Params, joint: JointHardware, knob: Part) -> None:
    label, centre, axis = joint.label, joint.centre, joint.axis
    floor_x, corner, recess_x = joint.floor_x, joint.corner, joint.recess_x
    bolt_chain, knob_chain = joint.bolt_chain, joint.knob_chain
    stack = joint_design(spec)
    under_head = Vector(floor_x, centre.Y, centre.Z) - axis * MODEL_GAP
    bolt, nut_name = f"{label} bolt", f"{label} nut"
    built.add(bolt, Item(hex_bolt(M6_HEX_BOLT, stack.length, under_head, axis, corner),
                         "steel", 5, bolt_chain,
                         bom=_bought(f"M6x{stack.length:g}", M6_HEX_BOLT.standard,
                                     M6_HEX_BOLT.source)))
    knob_dir = axis
    washer_at = Vector(recess_x, centre.Y, centre.Z) + knob_dir * MODEL_GAP
    built.add(f"{label} washer", Item(washer_solid(M6_WASHER, washer_at, knob_dir), "steel", 5,
                                      knob_chain, bom=_bought("M6 washer", M6_WASHER.standard,
                                                              M6_WASHER.source)))
    base = washer_at + knob_dir * (M6_WASHER.t + MODEL_GAP)
    turn = Rot(0, 90 if knob_dir.X > 0 else -90, 0)
    place = Pos(base) * turn
    built.add(f"{label} knob", Item(_placed(knob, place), "petg", 5, knob_chain, printed=True))
    nut_corner = Vector(0, 0, -1 if knob_dir.X > 0 else 1)
    built.add(nut_name, Item(nut_solid(M6_NUT, base + knob_dir * MODEL_GAP, knob_dir, nut_corner),
                             "steel", 5, knob_chain,
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
    _foot_screws(built, spec, built.head_nuts[:4], face, hd, "head", "zve10", 6)
    stack = camera_design()
    thickness = accessories.camera_plate_t()
    ceiling = _point(plate_at, Vector(0, accessories.zve10_camera_y(), QUARTER_HEAD_BORE[1]))
    head_at = ceiling - Z * MODEL_GAP
    built.add("camera screw", Item(socket_screw(QUARTER_SHCS, stack.length, head_at, Z),
                                   "black_steel", 6, "head",
                                   bom=_bought(f"1/4\"-20 x {stack.length / 25.4:.3g}\"",
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


def _seam_stacks(built: Hero, spec: Params) -> list[Check]:
    front, back = built.items["collar_front"].part, built.items["collar_back"].part
    rows = []
    seam_gap = front.distance_to(back)
    head_bore = M3_WASHER_BORE[0] / 2
    for index, nut in enumerate(collar.seam_nuts(spec)):
        x, z = nut.seat.X, nut.seat.Z
        side = Vector(M3_HEAD_BORE[0] / 4 + 0.8, 0, 0)
        seat = _ray(front, Vector(x, -80, z) + side, Y).Y
        load = _ray(back, Vector(x, nut.seat.Y + 1.0, z) + side, -Y).Y
        floor = _ray(back, Vector(x, nut.seat.Y + 1.0, z), Y).Y
        rim = max(_ray(front, Vector(x, -80, z) + offset, Y).Y
                  for offset in (X * (head_bore + 0.3), -X * (head_bore + 0.3),
                                 Z * (head_bore + 0.3)))
        grip = load - seat + M3_WASHER.t
        sink = (seat - rim) - M3_WASHER.t - M3_SHCS.head_k
        stack = Stack(f"seam {index + 1}", M3_SHCS, grip, grip - seam_gap,
                      thread_need(M3_SHCS, M3_NUT), floor - load - 0.5, sink)
        rows += _length_matches(stack, seam_design(spec).length) + stack.rows()
    return rows


def _length_matches(stack: Stack, built_length: float) -> list[Check]:
    ok = abs(stack.length - built_length) < 1e-9
    return [Check(f"{stack.joint} solid is the stack length", ok,
                  f"measured stack {stack.length:g}, modeled screw {built_length:g}")]


def _foot_stacks(built: Hero, spec: Params, foot: str, host: str, face: MountFace,
                 location: Location, label: str) -> list[Check]:
    foot_part, host_part = built.items[foot].part, built.items[host].part
    rows = []
    for dx, dy in m3_square(spec):
        axis_top = _point(location, Vector(face.centre.X + dx, face.centre.Y + dy,
                                           face.centre.Z + 50))
        side = X * (M3_HEAD_BORE[0] / 4 + 0.8)
        seat = _ray(foot_part, axis_top + side, -Z).Z
        rim = _ray(foot_part, axis_top + X * (M3_HEAD_BORE[0] / 2 + 0.4), -Z).Z
        inside = Vector(axis_top.X, axis_top.Y, face.centre.Z + location.position.Z
                        - spec.mf_m3_roof - 1.0)
        load = _ray(host_part, inside + side, Z).Z
        floor = _ray(host_part, inside, -Z).Z
        grip = seat - load
        stack = Stack(f"{label} ({dx:+g},{dy:+g})", M3_SHCS, grip, grip,
                      thread_need(M3_SHCS, M3_NUT), load - floor - 0.5,
                      (rim - seat) - M3_SHCS.head_k)
        rows += _length_matches(stack, foot_screw_length(spec)) + stack.rows()
    return rows


def _joint_stacks(built: Hero, spec: Params) -> list[Check]:
    rows = []
    j1, j2, j3 = joints(spec)
    hosts = (("J1", j1, "arm_base", X), ("J2", j2, "arm_link 1", -X), ("J3", j3, "arm_link 2", X))
    for label, centre, host, axis in hosts:
        side = Vector(0, 0.7, 4.0)  # inside the hex flats, off the symmetry plane OCCT misses
        floor = _ray(built.items[host].part, centre - axis * 80 + side, axis)
        washer = _ray(built.items[f"{label} washer"].part, centre + axis * 80 + side, -axis)
        grip = (washer - floor).dot(axis)
        knob = built.items[f"{label} knob"].part
        bore = _ray(knob, Vector(washer.X, centre.Y, centre.Z) + axis * 0.01, axis)
        room = (bore - washer).dot(axis)
        stack = Stack(f"{label} arm joint", M6_HEX_BOLT, grip + spec.index_lift, grip,
                      thread_need(M6_HEX_BOLT, M6_NUT), room - 0.5)
        rows += _length_matches(stack, joint_design(spec).length) + stack.rows()
    return rows


def _camera_stack(built: Hero) -> list[Check]:
    plate = built.items["zve10"].part
    screw_axis = built.items["camera screw"].part.bounding_box().center()
    side = Vector(0, QUARTER_HEAD_BORE[0] / 4 + 1.6, 0)  # across the slot, onto the ceiling
    below = Vector(screw_axis.X, screw_axis.Y, plate.bounding_box().min.Z - 20)
    ceiling = _ray(plate, below + side, Z).Z
    top = _ray(plate, below + side + Z * 100, -Z).Z
    grip = top - ceiling
    stack = Stack("camera screw", QUARTER_SHCS, grip, grip, ZVE10_THREAD_MIN, ZVE10_THREAD_MAX)
    return _length_matches(stack, camera_design().length) + stack.rows()


def stack_rows(spec: Params) -> list[Check]:
    built = hero(spec)
    rows = _seam_stacks(built, spec)
    rows += _foot_stacks(built, spec, "arm_base", "collar_back", collar.mount_face(spec),
                         Location(), "arm_base foot")
    rows += _foot_stacks(built, spec, "zve10", "arm_head", arm.head_mount_face(spec),
                         built.head_location, "zve10 foot")
    rows += _joint_stacks(built, spec)
    rows += _camera_stack(built)
    return rows


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
    rows += sweep_rows(spec)
    return rows
