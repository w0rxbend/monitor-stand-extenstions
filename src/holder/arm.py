"""Arm: base on the collar MF, two identical links, a head with an MF, three knobs.

Every joint is a lap joint with a 24 tooth Hirth rosette (90 degree flanks). The teeth
converge on the joint axis, so every flank is a plane. A tooth and a gap have the same
shape, so two rosettes mesh when their crests sit half a pitch apart. The phases below
make that true at every joint when each joint angle is a multiple of 15 degrees, with the
second link being the first one turned 180 degrees about its long axis.

Frames. A rosette is built in its joint frame: axis +X, pitch plane x = 0, angles measured
from +Y toward +Z. arm_base: foot bottom centre at the origin. arm_link: joint 1 at the
origin, joint 2 at +Y link_pitch, body x 0..knuckle_t. arm_head: J3 at the origin, body
x 0..knuckle_t, MF on top. knob: base on z = 0, axis +Z.
"""

from math import cos, radians, sin

from build123d import (
    Axis,
    CenterArc,
    Compound,
    EllipticalCenterArc,
    Face,
    Line,
    Part,
    Plane,
    Polygon,
    Pos,
    Vector,
    Vertex,
    Wire,
    chamfer,
    extrude,
    loft,
    make_face,
    revolve,
)

from holder.accessories import zve10_camera_y
from holder.common import (
    Check,
    at_least,
    hex_prism,
    named,
    one_solid,
    overhang_rows,
    print_orientation,
    rod,
    stress_row,
    teardrop_rod,
)
from holder.form import Frame, corner_curve, curvature_jumps, g2_blend
from holder.mount import MountFace, foot_checks, foot_cutters, foot_plan, foot_slab, m3_square
from holder.params import (
    M6_CLEAR,
    M6_HEAD_POCKET,
    M6_NUT,
    M6_WASHER_RECESS,
    PETG_WALL,
    Params,
)
from holder.stacks import knob_bore_depth

PRINT = {
    "arm_base": (0, 0, -1),
    "arm_link": (1, 0, 0),
    "arm_head": (0, 0, 1),
    "knob": (0, 0, -1),
}
MATERIAL = dict.fromkeys(PRINT, "PETG")
BASE_PHASE = 11.25  # crests toward +X
LINK_PHASE = 3.75  # crests toward -X, both ends
HEAD_PHASE = 3.75  # crests toward -X
ROOT_SETBACK = 4.0
EDGE = 1.0
BED_EDGE = 0.6
WAIST_START_DEG = 55.0
WAIST_RUN = 12.0
KNOB_GROOVES = (2.6, 4.2, 5.8)
KNOB_GROOVE_DEPTH = 0.5
KNOB_FLUTES = 24
KNOB_FLUTE_R = 1.1
KNOB_FLUTE_Z = 7.5
KNOB_DISH = 0.6
KNOB_DISH_R = 9.0
NUT_SINK = 5.5  # M6 nut pocket height in the knob
NECK_DROP = 12.0  # head ogee height, under the MF nuts and tip bores (8.6 deep)


# --- Rosette -----------------------------------------------------------------------------------


def _ring_point(radius: float, angle_deg: float, x: float = 0.0) -> Vector:
    angle = radians(angle_deg)
    return Vector(x, radius * cos(angle), radius * sin(angle))


def _pyramid(quad: list[Vector]) -> Part:
    face = Face(Wire.make_polygon(quad, close=True))
    return loft([face, Vertex(0, 0, 0)])


def _tooth(spec: Params, centre_deg: float, facing: float) -> Part:
    """Truncated pyramid from the axis point: a crest (facing) or a gap (-facing)."""
    half = 360 / spec.teeth / 4
    reach = spec.rosette_r_out + 1.0
    crest_h = reach * sin(radians(half))
    trunc = spec.tooth_trunc * reach / spec.rosette_r_out
    share = (crest_h - trunc) / crest_h
    peak = _ring_point(reach, centre_deg, facing * crest_h)
    low = _ring_point(reach, centre_deg - half)
    high = _ring_point(reach, centre_deg + half)
    quad = [low, low + (peak - low) * share, high + (peak - high) * share, high]
    return _pyramid(quad)


def rosette(spec: Params, facing: float, phase: float) -> tuple[Part, Part]:
    """(teeth to add, gaps to cut) in the joint frame, trimmed to r_in..r_out."""
    pitch = 360 / spec.teeth
    crests = [_tooth(spec, phase + k * pitch, facing) for k in range(spec.teeth)]
    gaps = [_tooth(spec, phase + pitch / 2 + k * pitch, -facing) for k in range(spec.teeth)]
    band = rosette_band(spec)
    return band & Compound(crests), band & Compound(gaps)


def rosette_band(spec: Params) -> Part:
    """r_in..r_out at the pitch plane, narrowing at 45 degrees away from it.

    A cylindrical trim would leave small flat ceilings on a vertical rosette. A 45 degree
    cone prints on any face.
    """
    low, high, depth = spec.rosette_r_in, spec.rosette_r_out, 1.0
    section = Polygon((-depth, low + depth), (0, low), (depth, low + depth),
                      (depth, high - depth), (0, high), (-depth, high - depth), align=None)
    return revolve(Plane.XY.location * section, Axis.X, 360)


def with_rosette(body: Part, spec: Params, at: Vector, facing: float, phase: float) -> Part:
    teeth, gaps = rosette(spec, facing, phase)
    return body + Pos(at) * teeth - Pos(at) * gaps


# --- Profiles ----------------------------------------------------------------------------------


def arch(spec: Params, centre_z: float, root_z: float, up: bool) -> Face:
    """Knuckle profile in (y, z): a half circle, straight sides, G2 roots into a flat.

    `up` puts the half circle above the flat (base); otherwise below (head). The circle
    meets the sides G1 on purpose: the sides are vertical, so nothing overhangs.
    """
    radius, run = spec.knuckle_r, ROOT_SETBACK
    beyond = root_z + (-1.0 if up else 1.0)
    arc = CenterArc((0, centre_z), radius, 0 if up else 180, 180)

    def root(side_x: float, leaving: bool) -> object:
        outward = (side_x + (9 if side_x > 0 else -9), root_z)
        if leaving:
            return corner_curve((side_x, centre_z), (side_x, root_z), outward, run)
        return corner_curve(outward, (side_x, root_z), (side_x, centre_z), run)

    first = root((arc @ 1).X, leaving=True)
    second = root((arc @ 0).X, leaving=False)
    a, b = first @ 1, second @ 0
    edges = [arc, Line(arc @ 1, first @ 0), first, Line(a, (a.X, beyond)),
             Line((a.X, beyond), (b.X, beyond)), Line((b.X, beyond), b), second,
             Line(second @ 1, arc @ 0)]
    return make_face(Wire(edges)).faces()[0]


def dog_bone(spec: Params) -> Face:
    """Link plan in (y, z): end circles joined to a straight waist by G2 quintic blends."""
    radius, pitch, half = spec.knuckle_r, spec.link_pitch, spec.link_waist / 2
    start = WAIST_START_DEG
    on_circle = Vector(radius * cos(radians(start)), radius * sin(radians(start)))
    waist_y = on_circle.X + WAIST_RUN
    reach = 0.25 * WAIST_RUN

    def blend(line_pt: Vector, line_dir: Vector, centre: Vector, angle: float,
              ccw: bool) -> object:
        point = centre + Vector(radius * cos(radians(angle)), radius * sin(radians(angle)))
        tangent = Vector(-sin(radians(angle)), cos(radians(angle)))
        tangent = tangent if ccw else -tangent
        inward = (centre - point).normalized()
        start_frame = Frame(line_pt, line_dir, Vector(0, 1), 0.0)
        end_frame = Frame(point, tangent, inward, 1 / radius)
        return g2_blend(start_frame, end_frame, reach, reach)

    left, right = Vector(0, 0), Vector(pitch, 0)
    top_left = blend(Vector(waist_y, half), Vector(-1, 0), left, start, True)
    bottom_left_rev = blend(Vector(waist_y, -half), Vector(-1, 0), left, -start, False)
    top_right_rev = blend(Vector(pitch - waist_y, half), Vector(1, 0), right, 180 - start, False)
    bottom_right = blend(Vector(pitch - waist_y, -half), Vector(1, 0), right, 180 + start, True)
    edges = [
        CenterArc((0, 0), radius, start, 360 - 2 * start),
        bottom_left_rev,
        Line((waist_y, -half), (pitch - waist_y, -half)),
        bottom_right,
        CenterArc((pitch, 0), radius, 180 + start, 360 - 2 * start),
        top_right_rev,
        Line((pitch - waist_y, half), (waist_y, half)),
        top_left,
    ]
    return make_face(Wire(edges)).faces()[0]


def _on_yz(face: Face, x: float) -> Face:
    return Plane.YZ.offset(x).location * face


def _chamfer_faces_at_x(part: Part, xs: tuple[float, ...], size: float,
                        z_range: tuple[float, float] = (-1e9, 1e9)) -> Part:
    """Chamfer the outline of the faces at each x, except edges buried in a mating body."""
    low, high = z_range
    edges = [edge for edge in part.edges()
             if any(abs(edge.center().X - x) < 1e-4 for x in xs)
             and abs(edge.bounding_box().size.X) < 1e-4
             and low + 1e-3 < edge.center().Z < high - 1e-3]
    return chamfer(edges, size)


def _loops(part: Part, top: float, top_size: float, bottom: float, bottom_size: float) -> Part:
    upper = [e for e in part.edges() if abs(e.center().Z - top) < 1e-4
             and e.bounding_box().size.Z < 1e-4]
    part = chamfer(upper, top_size)
    lower = [e for e in part.edges() if abs(e.center().Z - bottom) < 1e-4
             and e.bounding_box().size.Z < 1e-4]
    return chamfer(lower, bottom_size)


# --- Parts -------------------------------------------------------------------------------------


def j1_centre(spec: Params) -> Vector:
    return Vector(spec.arm_x0, 0, spec.foot_t + spec.j1_rise)


def build_base(spec: Params) -> Part:
    foot = _loops(foot_slab(spec, spec.foot_t), spec.foot_t, EDGE, 0.0, BED_EDGE)
    centre = j1_centre(spec)
    back_face = spec.arm_x0 - spec.knuckle_t
    profile = arch(spec, centre.Z, spec.foot_t, up=True)
    upright = extrude(_on_yz(profile, back_face), amount=spec.knuckle_t)
    upright = _chamfer_faces_at_x(upright, (back_face, spec.arm_x0), EDGE, (spec.foot_t + 0.5, 1e9))
    body = foot + upright
    body -= teardrop_rod(M6_CLEAR, spec.knuckle_t + 2, Vector(back_face - 1, 0, centre.Z),
                         Vector(1, 0, 0), Vector(0, 0, 1))
    pocket_af, pocket_depth = M6_HEAD_POCKET
    # A flat on top: the ceiling is a 5.9 mm bridge. A corner on top would leave 30 deg roofs.
    pocket = Plane(origin=(back_face - 1, 0, centre.Z), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    body -= hex_prism(pocket_af, pocket_depth + 1, pocket)
    body = with_rosette(body, spec, centre, 1.0, BASE_PHASE)
    for cutter in foot_cutters(spec, spec.foot_t):
        body -= cutter
    return named(body, "arm_base")


def build_link(spec: Params) -> Part:
    t = spec.knuckle_t
    body = extrude(_on_yz(dog_bone(spec), 0.0), amount=t)
    body = _chamfer_faces_at_x(body, (0.0, t), EDGE)
    ends = (Vector(0, 0, 0), Vector(0, spec.link_pitch, 0))
    for end in ends:
        body -= rod(M6_CLEAR, t + 2, end + Vector(-1, 0, 0), Vector(1, 0, 0))
        body = with_rosette(body, spec, end, -1.0, LINK_PHASE)
    recess_d, recess_h = M6_WASHER_RECESS
    body -= rod(recess_d, recess_h + 1, Vector(t - recess_h, 0, 0), Vector(1, 0, 0))
    pocket_af, pocket_depth = M6_HEAD_POCKET
    pocket = Plane(origin=(t - pocket_depth, spec.link_pitch, 0), x_dir=(0, 1, 0),
                   z_dir=(1, 0, 0))
    body -= hex_prism(pocket_af, pocket_depth + 1, pocket)
    return named(body, "arm_link")


def head_mf_centre(spec: Params) -> Vector:
    return Vector(spec.knuckle_t / 2, 0, spec.head_block_gap + spec.mf_depth)


def head_mount_face(spec: Params) -> MountFace:
    centre = head_mf_centre(spec)
    front = spec.mf_size / 2
    reach = {(dx, dy): front + dy + 1.5 for dx, dy in [*m3_square(spec), (0.0, 0.0)]}
    # Mouths on the -Y face: under the camera deck, out of sight.
    return MountFace("head MF", centre, Vector(0, -1, 0), reach)


def build_head(spec: Params) -> Part:
    t = spec.knuckle_t
    block_z = spec.head_block_gap
    top = block_z + spec.mf_depth
    knuckle = extrude(_on_yz(arch(spec, 0.0, block_z, up=False), 0.0), amount=t)
    knuckle = _chamfer_faces_at_x(knuckle, (0.0, t), EDGE, (-1e9, block_z - 0.5))
    block = Pos(0, 0, block_z) * extrude(foot_plan(spec, (t / 2, 0)), amount=spec.mf_depth)
    # OCCT 7.8 returns an empty common for block & neck here; this order is exact.
    block = neck_profile(spec) & block
    block = _loops_top(block, top, EDGE)
    body = block + knuckle
    tip = Vector(0, 0, -1)  # print-up: the head prints MF down
    body -= teardrop_rod(M6_CLEAR, t + 2, Vector(-1, 0, 0), Vector(1, 0, 0), tip)
    recess_d, recess_h = M6_WASHER_RECESS
    body -= teardrop_rod(recess_d, recess_h + 1, Vector(t - recess_h, 0, 0), Vector(1, 0, 0), tip)
    body = with_rosette(body, spec, Vector(0, 0, 0), -1.0, HEAD_PHASE)
    for cutter in head_mount_face(spec).cutters(spec):
        body -= cutter
    return named(body, "arm_head")


def neck_profile(spec: Params) -> Part:
    """Front view of the head: the 40 mm MF narrows to the knuckle by a G2 ogee.

    Each side is a quintic with vertical tangents and zero curvature at both ends, so it
    leaves the MF wall and lands on the knuckle face without a visible line. It runs
    inward as it drops, so nothing overhangs when the head prints MF down.
    """
    t, half = spec.knuckle_t, spec.mf_size / 2
    low, high = spec.head_block_gap, spec.head_block_gap + NECK_DROP
    top = spec.head_block_gap + spec.mf_depth + 1
    reach = 0.22 * NECK_DROP  # keeps both control polygons monotonic: no bump, no overhang
    down = Vector(0, -1)
    sides = []
    for outer, inner in ((t / 2 + half, t), (t / 2 - half, 0.0)):
        start = Frame(Vector(outer, high), down, Vector(1 if outer > t / 2 else -1, 0), 0.0)
        end = Frame(Vector(inner, low), down, Vector(1, 0), 0.0)
        sides.append(g2_blend(start, end, reach, reach))
    right, left = sides
    wide = 5.0  # past the block sides above the ogee, so no face is coplanar with them
    outline = Wire([
        Line((t / 2 - half - wide, top), (t / 2 + half + wide, top)),
        Line((t / 2 + half + wide, top), (t / 2 + half + wide, high)),
        Line((t / 2 + half + wide, high), (t / 2 + half, high)),
        right,
        Line((t, low), (t, low - 1)), Line((t, low - 1), (0, low - 1)),
        Line((0, low - 1), (0, low)),
        left,
        Line((t / 2 - half, high), (t / 2 - half - wide, high)),
        Line((t / 2 - half - wide, high), (t / 2 - half - wide, top)),
    ])
    face = Plane.XZ.offset(-spec.mf_size).location * make_face(outline).faces()[0]
    return extrude(face, amount=2 * spec.mf_size)


def _loops_top(part: Part, top: float, size: float) -> Part:
    upper = [e for e in part.edges() if abs(e.center().Z - top) < 1e-4
             and e.bounding_box().size.Z < 1e-4]
    return chamfer(upper, size)


def knob_height(spec: Params) -> float:
    return knob_bore_depth(spec) + spec.knob_roof + KNOB_DISH


def knob_profile(spec: Params) -> Face:
    radius, height = spec.knob_d / 2, knob_height(spec)
    points = [(0, 0), (radius - BED_EDGE, 0), (radius, BED_EDGE)]
    for z in KNOB_GROOVES:
        points += [(radius, z - KNOB_GROOVE_DEPTH), (radius - KNOB_GROOVE_DEPTH, z),
                   (radius, z + KNOB_GROOVE_DEPTH)]
    points += [(radius, height - EDGE), (radius - EDGE, height), (KNOB_DISH_R, height)]
    edges = [Line(a, b) for a, b in zip(points, points[1:])]
    dish = EllipticalCenterArc((0, height), KNOB_DISH_R, KNOB_DISH, start_angle=270, arc_size=90)
    edges += [dish, Line((0, height - KNOB_DISH), (0, 0))]
    return Plane.XZ.location * make_face(Wire(edges)).faces()[0]


def build_knob(spec: Params) -> Part:
    body = revolve(knob_profile(spec), Axis.Z, 360)
    radius, height = spec.knob_d / 2, knob_height(spec)
    for k in range(KNOB_FLUTES):
        angle = radians(360 / KNOB_FLUTES * (k + 0.5))
        centre = Vector((radius + 0.35) * cos(angle), (radius + 0.35) * sin(angle), KNOB_FLUTE_Z)
        body -= rod(2 * KNOB_FLUTE_R, height, centre, Vector(0, 0, 1))
    body -= hex_prism(M6_NUT.pocket_af, NUT_SINK + 1, Plane(origin=(0, 0, -1)))
    body -= rod(M6_CLEAR, knob_bore_depth(spec) + 1, Vector(0, 0, -1), Vector(0, 0, 1))
    return named(body, "knob")


# --- Checks ------------------------------------------------------------------------------------


def _printed(part: Part) -> list[Check]:
    return [one_solid(part), *overhang_rows(print_orientation(part, PRINT[part.label]),
                                            part.label)]


def checks_base(part: Part, spec: Params) -> list[Check]:
    rows = _printed(part) + foot_checks(part, spec)
    root = Plane(origin=(0, 0, spec.foot_t + ROOT_SETBACK + 0.5), z_dir=(0, 0, 1))
    rows += _stress(part, spec, root, "arm_base upright root", across=True)
    return rows


def checks_link(part: Part, spec: Params) -> list[Check]:
    rows = _printed(part)
    waist = Plane(origin=(0, spec.link_pitch / 2, 0), z_dir=(0, 1, 0))
    rows += _stress(part, spec, waist, "arm_link waist", across=False)
    jump = max(curvature_jumps(dog_bone(spec).outer_wire()))
    rows.append(Check("arm_link plan curvature jump (G2)", jump <= 1e-3, f"{jump:.2e} /mm"))
    rows.append(at_least("arm_link waist width", spec.link_waist, 20.0))
    return rows


def checks_head(part: Part, spec: Params) -> list[Check]:
    rows = _printed(part)
    neck = Plane(origin=(0, 0, spec.head_block_gap - ROOT_SETBACK - 0.5), z_dir=(0, 0, 1))
    rows += _stress(part, spec, neck, "arm_head neck", across=True)
    rows += head_mount_face(spec).checks(part, spec)
    return rows


def checks_knob(part: Part, spec: Params) -> list[Check]:
    rows = _printed(part)
    roof = knob_height(spec) - KNOB_DISH - knob_bore_depth(spec)
    rows.append(at_least("knob roof over the bolt tip", roof, PETG_WALL))
    nut_wall = spec.knob_d / 2 - KNOB_GROOVE_DEPTH - M6_NUT.pocket_af / 3 ** 0.5
    rows.append(at_least("knob wall round the nut corner", nut_wall, PETG_WALL))
    return rows


def payload_lever(spec: Params) -> float:
    """Worst lever from J1 to the camera: both links level, plus the deck offset."""
    return 2 * spec.link_pitch + abs(zve10_camera_y())


def _stress(part: Part, spec: Params, plane: Plane, name: str, across: bool) -> list[Check]:
    """Sustained and peak bending at a section, against 0.1 and 0.25 x UTS of PETG."""
    uts = 19.0 if across else 38.5  # Ultimaker PETG TDS: Z 19.0, XY 38.5 MPa
    moment = spec.payload_kg * 9.81 * payload_lever(spec)
    layers = "across layers" if across else "along layers"
    sustained = stress_row(part, plane, Vector(1, 0, 0), moment, 0.1 * uts,
                           f"{name} sustained stress, {layers}")
    peak = stress_row(part, plane, Vector(1, 0, 0), 2 * moment, 0.25 * uts,
                      f"{name} peak stress (2x), {layers}")
    return [sustained, peak]

