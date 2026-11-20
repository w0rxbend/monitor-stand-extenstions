"""Arm: base on the collar MF, two identical links, a head with an MF, three knobs.

Every joint is a lap joint with a 24 tooth Hirth rosette (90 degree flanks). The teeth
converge on the joint axis, so every flank is a plane. A tooth and a gap have the same
shape, so two rosettes mesh when their crests sit half a pitch apart. The phases below
make that true at every joint when each joint angle is a multiple of 15 degrees, with the
second link being the first one turned 180 degrees about its long axis.

Every knuckle and link is a G2 skin: its outline (an arch, or a dog-bone of circles and
G2 blends) with G2 rolls on both faces. The head rises from its knuckle into the MF as a
smooth loft whose width follows a quintic ogee, so it reads turned, not stacked. A DIN 603
carriage bolt's dome head bears on one face; its square neck in a square hole stops it
turning. The knob on the other face holds the nut and bears on an ISO 7093 washer.

Frames. A rosette is built in its joint frame: axis +X, pitch plane x = 0, angles measured
from +Y toward +Z. arm_base: foot bottom centre at the origin. arm_link: joint 1 at the
origin, joint 2 at +Y link_pitch, body x 0..knuckle_t. arm_head: J3 at the origin, body
x 0..knuckle_t, MF on top. knob: base on z = 0, axis +Z.
"""

from math import cos, radians, sin, sqrt

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
    extrude,
    loft,
    make_face,
    revolve,
)

from holder.common import (
    Check,
    annulus_footprint,
    at_least,
    bearing_row,
    contact_area,
    hex_prism,
    named,
    one_solid,
    overhang_rows,
    print_orientation,
    rod,
    skin_rows,
    teardrop_rod,
)
from holder.form import (
    BedLead,
    Frame,
    Roll,
    corner_curve,
    curvature_jumps,
    g2_blend,
    g2_skin,
    skin_profile,
    soft_outline,
)
from holder.mount import MountFace, foot_checks, foot_cutters, foot_slab
from holder.params import (
    M6_CARRIAGE,
    M6_CLEAR,
    M6_LARGE_WASHER,
    M6_NECK_HOLE,
    M6_NUT,
    PETG_WALL,
    PRELOAD_N,
    Params,
)

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
ROSETTE_ROLL = Roll(1.2)  # knuckle r 14.5 - rosette r 13 leaves 1.5 of flat face
FACE_ROLL = Roll(1.2)  # a buried corner of an upright outline must out-radius the roll
LINK_BED = BedLead(0.6, 1.5)  # the link prints on its +X face
BURY = 6.0  # an upright outline runs this far into the part it stands on
BURY_SETBACK = 2.8  # its buried corners: radius 1.3, over the 1.2 roll
WAIST_START_DEG = 55.0
WAIST_RUN = 12.0
TULIP_RISE = 12.0  # head ogee height, from the knuckle to the full MF plan
TULIP_PITCH = 1.0  # mm between loft sections
MF_BLOCK_TOP = BedLead(1.0, 3.0)  # the head prints MF down
KNOB_FLUTES = 24
KNOB_FLUTE_R = 1.1
KNOB_FLUTE_OFFSET = 0.35  # flute centres sit this far outside the knob wall
KNOB_FLUTE_TOP = 11.0
KNOB_BED = 0.6
KNOB_CROWN_H = 3.0  # elliptic crown: an exact conic, G2 into the wall
KNOB_CROWN_R = 11.0
KNOB_CROWN_START = 25.0  # degrees above the crown's equator where the blend lands
NUT_SINK = 5.5  # M6 nut pocket height in the knob


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

ARC_BLEND_DEG = 25.0  # where the G2 blend leaves the knuckle circle
SIDE_BLEND_DROP = 2.0  # how far below the circle centre the blend lands on the side


def knuckle_outline(spec: Params, centre_z: float, end_z: float) -> Wire:
    """Knuckle outline in (y, z), arc on top: a circle about the joint, quintic G2 blends to
    two straight sides, and G2 corners at `end_z`, which is buried in the part below."""
    radius, cz, run = spec.knuckle_r, centre_z, BURY_SETBACK
    lift = radians(ARC_BLEND_DEG)
    arc = CenterArc((0, cz), radius, ARC_BLEND_DEG, 180 - 2 * ARC_BLEND_DEG)

    def blend(side: float) -> object:
        angle = lift if side > 0 else radians(180) - lift
        on_circle = Vector(radius * cos(angle), cz + radius * sin(angle))
        inward = (Vector(0, cz) - on_circle).normalized()
        travel = Vector(-sin(angle), cos(angle)) * (1 if side > 0 else -1)
        circle = Frame(on_circle, travel, inward, 1 / radius)
        line = Frame(Vector(side * radius, cz - SIDE_BLEND_DROP), Vector(0, 1),
                     Vector(-side, 0), 0.0)
        reach = 0.3 * (on_circle - line.point).length
        if side > 0:
            return g2_blend(line, circle, reach, reach)
        return g2_blend(Frame(on_circle, -travel, inward, 1 / radius),
                        Frame(line.point, Vector(0, -1), Vector(1, 0), 0.0), reach, reach)

    right, left = blend(1.0), blend(-1.0)
    low = cz - SIDE_BLEND_DROP
    corner_left = corner_curve((-radius, low), (-radius, end_z), (radius, end_z), run)
    corner_right = corner_curve((-radius, end_z), (radius, end_z), (radius, low), run)
    edges = [arc, left, Line((-radius, low), corner_left @ 0), corner_left,
             Line(corner_left @ 1, corner_right @ 0), corner_right,
             Line(corner_right @ 1, (radius, low)), right]
    return Wire(edges)


def _on_yz(part: Part, x: float) -> Part:
    """A skin built in XY (outline in (y, z), thickness along local z) set on the YZ plane."""
    return Plane.YZ.offset(x).location * part


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




def neck_hole(origin: Vector, axis: Vector) -> list[Part]:
    """Square hole for the carriage bolt neck, a corner on +Z (45 degree roofs in any pose),
    then the round bolt hole through. `origin` is on the face the dome head bears on."""
    side, depth = M6_NECK_HOLE
    half = side / sqrt(2)
    plane = Plane(origin=origin - axis * 1.0, x_dir=Vector(0, 0, 1), z_dir=axis)
    diamond = Polygon((half, 0), (0, half), (-half, 0), (0, -half), align=None)
    return [plane.location * extrude(diamond, amount=depth + 1.0)]


def washer_footprint(floor: Vector, into: Vector) -> Part:
    return annulus_footprint(floor, into, M6_LARGE_WASHER.outer, M6_LARGE_WASHER.inner)


def dome_footprint(face: Vector, into: Vector) -> Part:
    return annulus_footprint(face, into, M6_CARRIAGE.bearing_d, 0.2)


# --- Parts -------------------------------------------------------------------------------------


def j1_centre(spec: Params) -> Vector:
    return Vector(spec.arm_x0, 0, spec.foot_t + spec.j1_rise)


def base_skins(spec: Params) -> tuple[Part, Part]:
    """(foot, upright) of arm_base, before they are fused and cut."""
    centre = j1_centre(spec)
    outline = knuckle_outline(spec, centre.Z, spec.foot_t - BURY)
    upright = _on_yz(g2_skin(outline, spec.knuckle_t, FACE_ROLL, ROSETTE_ROLL),
                     spec.arm_x0 - spec.knuckle_t)
    return foot_slab(spec, spec.foot_t), upright


def build_base(spec: Params) -> Part:
    foot, upright = base_skins(spec)
    centre = j1_centre(spec)
    back_face = spec.arm_x0 - spec.knuckle_t
    body = foot + upright
    body -= teardrop_rod(M6_CLEAR, spec.knuckle_t + 2, Vector(back_face - 1, 0, centre.Z),
                         Vector(1, 0, 0), Vector(0, 0, 1))
    for cutter in neck_hole(Vector(back_face, 0, centre.Z), Vector(1, 0, 0)):
        body -= cutter
    body = with_rosette(body, spec, centre, 1.0, BASE_PHASE)
    for cutter in foot_cutters(spec, spec.foot_t):
        body -= cutter
    return named(body, "arm_base")


def link_skin(spec: Params) -> Part:
    return _on_yz(g2_skin(dog_bone(spec).outer_wire(), spec.knuckle_t, ROSETTE_ROLL, LINK_BED),
                  0.0)


def build_link(spec: Params) -> Part:
    t = spec.knuckle_t
    body = link_skin(spec)
    for end in (Vector(0, 0, 0), Vector(0, spec.link_pitch, 0)):
        body -= rod(M6_CLEAR, t + 2, end + Vector(-1, 0, 0), Vector(1, 0, 0))
        body = with_rosette(body, spec, end, -1.0, LINK_PHASE)
    for cutter in neck_hole(Vector(t, spec.link_pitch, 0), Vector(-1, 0, 0)):
        body -= cutter
    return named(body, "arm_link")


def head_mf_centre(spec: Params) -> Vector:
    return Vector(spec.knuckle_t / 2, 0, spec.head_block_gap + spec.mf_depth)


def head_mount_face(spec: Params) -> MountFace:
    centre = head_mf_centre(spec)
    # The port nut mouth is on the -Y face: under the camera deck, out of sight.
    return MountFace("head MF", centre, Vector(0, -1, 0), spec.mf_size / 2 + 1.5)


SHOULDER = 0.6  # the tulip starts this far outside the knuckle: a turned shoulder line


def _smoothstep(u: float) -> float:
    """Quintic ogee: zero slope and zero curvature at both ends."""
    return u * u * u * (10 - 15 * u + 6 * u * u)


def _lead_inset(height: float, rule: BedLead) -> list[tuple[float, float]]:
    """(depth below the top, inset) along the top bed lead of a skin profile, densely."""
    profile = skin_profile(height, Roll(1.0), rule)
    points = [edge @ (k / 64) for edge in profile.edges() for k in range(65)]
    return sorted(((height - p.Y, p.X) for p in points if p.Y > height - rule.leg - rule.rise
                   - 1.0), key=lambda pair: pair[0])


def _inset_at(depth: float, table: list[tuple[float, float]]) -> float:
    for (d0, u0), (d1, u1) in zip(table, table[1:]):
        if d0 <= depth <= d1:
            return u0 if d1 - d0 < 1e-9 else u0 + (u1 - u0) * (depth - d0) / (d1 - d0)
    return 0.0


def tulip_sections(spec: Params) -> list[Face]:
    """Plan sections of the head, about 1 mm apart, from the knuckle shoulder to the MF face.

    The width follows one smooth law: a quintic ogee up to the full MF plan, straight walls,
    then the inset of a G2 bed lead (the head prints MF down). Dense, smooth sections keep
    the loft from overshooting between them.
    """
    t, base = spec.knuckle_t, spec.head_block_gap
    top = base + spec.mf_depth
    small = (t + 2 * SHOULDER, 2 * spec.knuckle_r + 2 * SHOULDER, FACE_ROLL.setback + SHOULDER)
    big = (spec.mf_size, spec.mf_size, spec.mf_setback)
    lead = _lead_inset(spec.mf_depth, MF_BLOCK_TOP)
    count = round(spec.mf_depth / TULIP_PITCH)
    faces = []
    for k in range(count + 1):
        z = base + spec.mf_depth * k / count
        share = _smoothstep(min((z - base) / TULIP_RISE, 1.0))
        inset = _inset_at(top - z, lead)
        w, d, corner = (a + (b - a) * share for a, b in zip(small, big))
        faces.append(Pos(0, 0, z) * soft_outline_rect(w - 2 * inset, d - 2 * inset,
                                                      corner - inset, (t / 2, 0)))
    return faces


def soft_outline_rect(width: float, depth: float, setback: float,
                      centre: tuple[float, float]) -> Face:
    cx, cy = centre
    hx, hy = width / 2, depth / 2
    corners = [(cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy), (cx - hx, cy - hy)]
    return make_face(soft_outline(corners, [setback] * 4)).faces()[0]


def head_skins(spec: Params) -> tuple[Part, Part]:
    """(knuckle, tulip) of arm_head. The knuckle arc points down; its top is buried."""
    outline = knuckle_outline(spec, 0.0, -(spec.head_block_gap + BURY)).mirror(Plane.XZ)
    knuckle = _on_yz(g2_skin(outline, spec.knuckle_t, ROSETTE_ROLL, FACE_ROLL), 0.0)
    return knuckle, loft(tulip_sections(spec), ruled=False)


def build_head(spec: Params) -> Part:
    t = spec.knuckle_t
    knuckle, tulip = head_skins(spec)
    body = knuckle + tulip
    tip = Vector(0, 0, -1)  # print-up: the head prints MF down
    body -= teardrop_rod(M6_CLEAR, t + 2, Vector(-1, 0, 0), Vector(1, 0, 0), tip)
    body = with_rosette(body, spec, Vector(0, 0, 0), -1.0, HEAD_PHASE)
    for cutter in head_mount_face(spec).cutters(spec):
        body -= cutter
    return named(body, "arm_head")


def knob_height(spec: Params) -> float:
    return spec.knob_tip_room + spec.knob_roof + 0.5


def knob_profile_wire(spec: Params) -> Wire:
    """(r, z): bed lead, straight wall, G2 blend, an elliptic crown (exact conic) to the axis."""
    radius, height = spec.knob_d / 2, knob_height(spec)
    a, b = KNOB_CROWN_R, KNOB_CROWN_H
    theta = radians(KNOB_CROWN_START)
    on_crown = Vector(a * cos(theta), height - b + b * sin(theta))
    travel = Vector(-a * sin(theta), b * cos(theta)).normalized()
    curvature = a * b / (a * a * sin(theta) ** 2 + b * b * cos(theta) ** 2) ** 1.5
    inward = Vector(-travel.Y, travel.X) if travel.X < 0 else Vector(travel.Y, -travel.X)
    if inward.Y > 0:
        inward = -inward
    wall_top = Vector(radius, on_crown.Y - 3.2)
    blend = g2_blend(Frame(wall_top, Vector(0, 1), Vector(-1, 0), 0.0),
                     Frame(on_crown, travel, inward, curvature), 1.4, 1.2)
    crown = EllipticalCenterArc((0, height - b), a, b, start_angle=KNOB_CROWN_START,
                                arc_size=90 - KNOB_CROWN_START)
    return Wire([Line((0, 0), (radius - KNOB_BED, 0)), Line((radius - KNOB_BED, 0),
                                                            (radius, KNOB_BED)),
                 Line((radius, KNOB_BED), wall_top), blend, crown,
                 Line((0, height), (0, 0))])


def build_knob(spec: Params) -> Part:
    # Two 180 degree halves: a single 360 degree revolve of the crown and its G2 blend
    # does not survive the STEP round trip (it reads back 12 % small and invalid).
    profile = Plane.XZ.location * make_face(knob_profile_wire(spec)).faces()[0]
    half = revolve(profile, Axis.Z, 180)
    body = half + half.rotate(Axis.Z, 180)
    radius = spec.knob_d / 2
    for k in range(KNOB_FLUTES):
        angle = radians(360 / KNOB_FLUTES * (k + 0.5))
        reach = radius + KNOB_FLUTE_OFFSET
        centre = Vector(reach * cos(angle), reach * sin(angle), -1)
        body -= rod(2 * KNOB_FLUTE_R, KNOB_FLUTE_TOP + 1, centre, Vector(0, 0, 1))
    body -= hex_prism(M6_NUT.pocket_af, NUT_SINK + 1, Plane(origin=(0, 0, -1)))
    body -= rod(M6_CLEAR, spec.knob_tip_room + 1, Vector(0, 0, -1), Vector(0, 0, 1))
    return named(body, "knob")


# --- Checks ------------------------------------------------------------------------------------


def _printed(part: Part) -> list[Check]:
    return [one_solid(part), *overhang_rows(print_orientation(part, PRINT[part.label]),
                                            part.label)]


def checks_base(part: Part, spec: Params) -> list[Check]:
    rows = _printed(part) + foot_checks(part, spec)
    foot, upright = base_skins(spec)
    rows += skin_rows(foot, "arm_base foot", lambda c: abs(c.Z) < 1e-4)
    rows += skin_rows(upright, "arm_base upright", lambda _c: False)
    face = Vector(spec.arm_x0 - spec.knuckle_t, 0, j1_centre(spec).Z)
    rows.append(bearing_row("J1 carriage dome on arm_base", PRELOAD_N["joint"],
                            contact_area(part, dome_footprint(face, Vector(1, 0, 0)))))
    return rows


def checks_link(part: Part, spec: Params) -> list[Check]:
    t = spec.knuckle_t
    rows = _printed(part)
    rows += skin_rows(link_skin(spec), "arm_link", lambda c: abs(c.X - t) < 1e-4)
    jump = max(curvature_jumps(dog_bone(spec).outer_wire()))
    rows.append(Check("arm_link plan curvature jump (G2)", jump <= 1e-3, f"{jump:.2e} /mm"))
    rows.append(at_least("arm_link waist width", spec.link_waist, 20.0))
    floor = Vector(t, 0, 0)
    rows.append(bearing_row("knob washer on arm_link", PRELOAD_N["joint"],
                            contact_area(part, washer_footprint(floor, Vector(-1, 0, 0)))))
    face = Vector(t, spec.link_pitch, 0)
    rows.append(bearing_row("carriage dome on arm_link", PRELOAD_N["joint"],
                            contact_area(part, dome_footprint(face, Vector(-1, 0, 0)))))
    return rows


def checks_head(part: Part, spec: Params) -> list[Check]:
    t = spec.knuckle_t
    rows = _printed(part)
    knuckle, tulip = head_skins(spec)
    top = spec.head_block_gap + spec.mf_depth
    rows += skin_rows(knuckle, "arm_head knuckle", lambda _c: False)
    rows += skin_rows(tulip, "arm_head tulip", lambda c: abs(c.Z - top) < 1e-4
                      or abs(c.Z - spec.head_block_gap) < 1e-4)
    floor = Vector(t, 0, 0)
    rows.append(bearing_row("J3 knob washer on arm_head", PRELOAD_N["joint"],
                            contact_area(part, washer_footprint(floor, Vector(-1, 0, 0)))))
    rows += head_mount_face(spec).checks(part, spec)
    return rows


def checks_knob(part: Part, spec: Params) -> list[Check]:
    rows = _printed(part)
    edge = M6_CLEAR / 2  # the crown is lowest over the bore at the bore's edge
    crown = knob_height(spec) - KNOB_CROWN_H * (1 - sqrt(1 - (edge / KNOB_CROWN_R) ** 2))
    rows.append(at_least("knob roof over the bolt bore", crown - spec.knob_tip_room,
                         PETG_WALL))
    flute_floor = spec.knob_d / 2 + KNOB_FLUTE_OFFSET - KNOB_FLUTE_R
    nut_wall = flute_floor - M6_NUT.pocket_af / sqrt(3)
    rows.append(at_least("knob wall round the nut corner, at a flute", nut_wall, PETG_WALL))
    outside = Wire(knob_profile_wire(spec).edges()[1:-1])  # not the base or the axis
    jump = max(curvature_jumps(outside))
    rows.append(Check("knob crown curvature jump (G2)", jump <= 1e-3, f"{jump:.2e} /mm"))
    return rows
