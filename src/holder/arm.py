"""Arm: a turned pedestal that pans on the collar, a link, a tip, a camera head, knobs.

Kinematics. The pedestal turns on the collar's tail about a vertical axis: smooth pan.
J1 and J2 are 24 tooth Hirth rosettes (15 degree index) that set reach and height. J3 is
a smooth friction joint on a TPU ring: fine tilt. Every tilt axis is horizontal for any
pan angle, so the horizon stays level without a roll joint.

Fasteners. Every joint is a spring stop (stops.py): an ISO 7379 shoulder screw whose
knurled head is melted into a knob, five DIN 2093 springs hidden in a 2.5 mm spot face
under the knob, and an M6 heat-set insert in the far part. Turn the knob until it stops.

Form. The pedestal is one turned profile of exact conics (a ball, a circular waist, a
quarter-ellipse flare, an elliptic foot shoulder) joined G2, with one milled J1 flat.
The link and the tip are G2 dog-bone skins. The head is one G2 skin whose side outline
sweeps from the J3 block into the camera platform through a quarter ellipse. The knob is
turned, with an elliptic crown.

Frames. A rosette is built in its joint frame: axis +X, pitch plane x = 0, angles from +Y
toward +Z. arm_base: pivot face centre at the origin, pan axis +Z, J1 on the flat at
x = ball_flat_x. arm_link / arm_tip: end A at the origin, end B at +Y link_pitch, body
x 0..link_t, inner (joint) face x = 0. arm_head: J3 at the origin, friction face x = 0,
the block toward +X, the platform toward -X. knob: rim on z = 0, axis +Z.
"""

from functools import cache
from math import cos, radians, sin

from build123d import (
    Align,
    Axis,
    CenterArc,
    Compound,
    Cone,
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
    loft,
    make_face,
    revolve,
)

from holder.common import (
    Check,
    SeatedInsert,
    at_least,
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
    Turtle,
    curvature_jumps,
    g2_blend,
    g2_skin,
    revolved,
)
from holder.params import (
    INSERT_M6,
    INSERT_SINK,
    PETG_WALL,
    PLATE_UNDER_HEAD,
    QUARTER_CLEAR,
    QUARTER_WASHER_SEAT,
    RING_T,
    SHOULDER_M6,
    SPRING_B16,
    WASHER_M8,
    ZVE10_BODY,
    ZVE10_SOCKET,
    Params,
)
from holder.stops import StopSpec

PRINT = {
    "arm_base": (0, 0, -1),
    "arm_link": (1, 0, 0),
    "arm_tip": (1, 0, 0),
    "arm_head": (0, 1, 0),
    "knob": (0, 0, -1),
    "j3_ring": (0, 0, -1),
}
MATERIAL = {**dict.fromkeys(PRINT, "PETG"), "j3_ring": "TPU95A"}
BASE_PHASE = 11.25  # crests toward +X
LINK_PHASE = 3.75  # crests toward -X, both ends
ROSETTE_ROLL = Roll(1.2)  # knuckle r 15.5 - rosette r 13 leaves 2.5 of flat face
LINK_BED = BedLead(0.6, 1.5)  # the link prints on its +X face
HEAD_BED = BedLead(0.8, 2.0)  # the head prints on its +Y face
HEAD_ROLL = Roll(2.0)
WAIST_START_DEG = 55.0
WAIST_RUN = 12.0
SPOT = 2.5  # every knob seat is a spot face this deep; the springs hide in it
SPOT_D = WASHER_M8.outer + 0.4
KNOB_RIM_GAP = 0.5  # knob rim above the face round its spot face
KNOB_FLUTES = 24
KNOB_FLUTE_R = 1.1
KNOB_FLUTE_OFFSET = 0.35  # flute centres sit this far outside the knob wall
KNOB_BED = 0.6
KNOB_CROWN_H = 3.0  # elliptic crown: an exact conic, G2 into the wall
KNOB_CROWN_R = 11.0
KNOB_CROWN_START = 25.0  # degrees above the crown's equator where the blend lands
KNOB_ROOF = 2.0
HEAD_PRESS = 0.6  # the knurled screw head is melted into a pocket this much under its size
RING_OUTER = 2 * 13.0
RING_INNER = SHOULDER_M6.shoulder_d + 0.8
J_SPRINGS = 5
J_TARGET = 0.6  # design deflection of the joint springs, share of h0
J3_TARGET = 0.5
J_L1 = 20.0
PIVOT_L1 = 40.0


# --- Stops -------------------------------------------------------------------------------------


def joint_stop(label: str) -> StopSpec:
    """J3 sits softer: its band also carries the printed ring, and friction needs less."""
    target = J3_TARGET if label == "J3" else J_TARGET
    return StopSpec(label, SHOULDER_M6, SPRING_B16, J_SPRINGS, WASHER_M8, WASHER_M8,
                    INSERT_M6, target, J_L1)


def pivot_stop() -> StopSpec:
    return StopSpec("pan pivot", SHOULDER_M6, SPRING_B16, J_SPRINGS, WASHER_M8, WASHER_M8,
                    INSERT_M6, J_TARGET, PIVOT_L1)


def pitch_gap(spec: Params) -> float:
    """Axial gap between engaged rosettes that leaves flank_fit normal to a 45 degree flank."""
    return spec.flank_fit / sin(radians(45))


def link_t(spec: Params) -> float:
    """Knuckle thickness: the near end's PETG under the spot face is the stop's clamped
    length less the rosette gap and the insert sink, plus the spot face."""
    return joint_stop("J").clamped - pitch_gap(spec) - INSERT_SINK + SPOT


def ring_gap() -> float:
    return 0.02  # the TPU ring is modeled this far off both faces


def head_block_t() -> float:
    """J3 block: clamped length less the ring, its gaps and the insert sink, plus the spot."""
    return joint_stop("J3").clamped - RING_T - 2 * ring_gap() - INSERT_SINK + SPOT


def knob_skirt() -> float:
    """Knob rim to the screw head's underside: the stack above the seat less the spot face
    and the rim gap."""
    return joint_stop("J").above_seat - SPOT - KNOB_RIM_GAP


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


def tooth_depth(spec: Params) -> float:
    """How far a gap sinks below the pitch plane: the insert wall is measured below it."""
    reach = spec.rosette_r_out + 1.0
    return reach * sin(radians(360 / spec.teeth / 4)) + 0.05


def with_rosette(body: Part, spec: Params, at: Vector, facing: float, phase: float) -> Part:
    teeth, gaps = rosette(spec, facing, phase)
    return body + Pos(at) * teeth - Pos(at) * gaps


# --- Pedestal ------------------------------------------------------------------------------------

BALL_LATITUDE = -40.0  # the ball starts here: 50 deg from down-facing, prints unsupported
WAIST_R = 4.0
FLARE = (13.0, 5.0)  # quarter ellipse: down, out
SHOULDER = (4.0, 3.0)  # quarter ellipse: out, down
FOOT_WALL = 2.0
FOOT_LEAD = 1.2  # lead on the bed edge (the crease)
FOOT_LEAD_TURN = 43.0  # the lead stands at 47 deg: the blend into it never dips under 45
TURTLE_TRIM = 1.0


def pedestal_turtle(spec: Params) -> Turtle:
    """From the ball's top pole outward and down to the bed edge (y is height, 0 at the pole)."""
    path = Turtle((0, 0), 0.0)
    path.arc(spec.ball_r, BALL_LATITUDE - 90)
    path.arc(WAIST_R, -BALL_LATITUDE)
    path.ellipse(*FLARE, 90)
    path.ellipse(*SHOULDER, -90)
    path.line(FOOT_WALL)
    path.arc(1.0, -FOOT_LEAD_TURN)
    path.line(FOOT_LEAD)
    return path


def pedestal_profile(spec: Params) -> Wire:
    path = pedestal_turtle(spec)
    edges = path.soft(TURTLE_TRIM)
    end = path.at
    edges += [Line(end, (0, end.Y)), Line((0, end.Y), (0, 0))]
    return Pos(0, -end.Y) * Wire(edges)


def pedestal_height(spec: Params) -> float:
    return -pedestal_turtle(spec).at.Y


def foot_r(spec: Params) -> float:
    return pedestal_turtle(spec).at.X


def j1_local(spec: Params) -> Vector:
    """J1 on the pedestal, in its frame: on the flat, at the ball centre's height."""
    return Vector(spec.ball_flat_x, 0, pedestal_height(spec) - spec.ball_r)


def neck_r(spec: Params) -> float:
    """Narrowest radius of the pedestal: where the waist arc turns vertical."""
    return spec.ball_r * cos(radians(BALL_LATITUDE)) - WAIST_R * (1 - cos(radians(
        BALL_LATITUDE)))


@cache
def pedestal_skin(spec: Params) -> Part:
    return revolved(pedestal_profile(spec))


def base_knob_seat_x(spec: Params) -> float:
    """J1's seat (the knob side, -X): the stop's clamped length back from link 1's insert."""
    return spec.ball_flat_x + pitch_gap(spec) + INSERT_SINK - joint_stop("J1").clamped


def build_base(spec: Params) -> Part:
    """The turned pedestal: rosette flat on +X, knob flat on -X, the J1 shoulder bore
    through the ball, and the pivot insert from the pivot face."""
    body = pedestal_skin(spec)
    j1 = j1_local(spec)
    up = Vector(0, 0, 1)  # print-up: the pedestal prints foot down
    clear = 2 * (spec.ball_r + 0.5)
    body -= rod(clear, spec.ball_r, j1, (1, 0, 0))  # milled J1 rosette flat
    face = base_knob_seat_x(spec) - SPOT
    body -= rod(clear, spec.ball_r, Vector(face, 0, j1.Z), (-1, 0, 0))  # milled knob flat
    body -= teardrop_rod(SPOT_D, SPOT + 1, Vector(face - 1, 0, j1.Z), (1, 0, 0), up)
    body -= teardrop_rod(SHOULDER_M6.shoulder_d + 0.4, 2 * spec.ball_r,
                         Vector(face - spec.ball_r, 0, j1.Z), (1, 0, 0), up)
    body = with_rosette(body, spec, j1, 1.0, BASE_PHASE)
    for cutter in pivot_insert().cutters():
        body -= cutter
    return named(body, "arm_base")


def pivot_insert() -> SeatedInsert:
    """The pan pivot's insert, pressed up into the pedestal from its pivot face."""
    return SeatedInsert("pan pivot", INSERT_M6, Vector(0, 0, 0), Vector(0, 0, 1))


# --- Link and tip --------------------------------------------------------------------------------


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


@cache
def link_skin(spec: Params) -> Part:
    return _on_yz(g2_skin(dog_bone(spec).outer_wire(), link_t(spec), ROSETTE_ROLL, LINK_BED),
                  0.0)


def link_inserts(spec: Params, ends: tuple[str, ...]) -> list[SeatedInsert]:
    """Inserts pressed from the inner face toward +X (vertical in print) at the far ends."""
    where = {"A": Vector(0, 0, 0), "B": Vector(0, spec.link_pitch, 0)}
    return [SeatedInsert(f"end {end}", INSERT_M6, where[end], Vector(1, 0, 0))
            for end in ends]


def build_link(spec: Params) -> Part:
    """J1 to J2: rosettes and inserts at both ends; the knobs sit on the parts beside it."""
    body = link_skin(spec)
    for end in (Vector(0, 0, 0), Vector(0, spec.link_pitch, 0)):
        body = with_rosette(body, spec, end, -1.0, LINK_PHASE)
    for insert in link_inserts(spec, ("A", "B")):
        for cutter in insert.cutters():
            body -= cutter
    return named(body, "arm_link")


def build_tip(spec: Params) -> Part:
    """J2 to J3: end A takes the J2 knob (spot face, shoulder bore, rosette); end B is a
    flat friction face for the TPU ring, with the J3 insert."""
    t = link_t(spec)
    body = link_skin(spec)
    body -= rod(SHOULDER_M6.shoulder_d + 0.4, t + 2, (-1, 0, 0), (1, 0, 0))
    body -= rod(SPOT_D, SPOT + 1, Vector(t - SPOT, 0, 0), (1, 0, 0))
    body = with_rosette(body, spec, Vector(0, 0, 0), -1.0, LINK_PHASE)
    for cutter in link_inserts(spec, ("B",))[0].cutters():
        body -= cutter
    return named(body, "arm_tip")


# --- Head ----------------------------------------------------------------------------------------

HEAD_CORNER = (4.5, 5.0)  # the J3 block's lower corners: min radius 4.05 over the 2.0 roll
HEAD_DROP = 4.0  # block bottom below the knuckle radius: the corner blends stay off the ring
HEAD_UNDER_SWEEP = (10.0, 20.0)  # the platform underside sweeps into the friction face
HEAD_KNOB_SWEEP = 8.0  # and on the knob side a narrower one: end radius 3.2 over the 2.0 roll
PLATFORM_T = 9.0
HEAD_TRIM = 1.5


def head_heights(spec: Params) -> tuple[float, float, float]:
    """(top of the flat friction face, platform underside, platform top) above J3."""
    face_top = spec.knuckle_r + 2.0
    under = face_top + HEAD_UNDER_SWEEP[1]
    return face_top, under, under + PLATFORM_T


def head_turtle(spec: Params) -> Turtle:
    """Side outline in (x, z), counter-clockwise from the top of the friction face: the J3
    block rises into a platform through a quarter ellipse on each side."""
    r, block, reach = spec.knuckle_r, head_block_t(), spec.platform_reach
    sweep_x, sweep_z = HEAD_UNDER_SWEEP
    face_top, under, top = head_heights(spec)
    bottom = -(r + HEAD_DROP)
    down, along = HEAD_CORNER
    end_r = PLATFORM_T / 2
    right = platform_right(spec)
    path = Turtle((0, face_top), 270.0)
    path.line(face_top - (bottom + down))
    path.ellipse(down, along, 90)
    path.line(block - 2 * along)
    path.ellipse(along, down, 90)
    path.line(face_top - (bottom + down))
    path.ellipse(sweep_z, HEAD_KNOB_SWEEP, -90)
    path.line(right - end_r - block - HEAD_KNOB_SWEEP)
    path.arc(end_r, 180)
    path.line(right - end_r + reach - end_r)
    path.arc(end_r, 180)
    path.line(reach - end_r - sweep_x)
    path.ellipse(sweep_x, sweep_z, -90)
    return path


def platform_right(spec: Params) -> float:
    """The platform's +X end: just past the camera's end over the knob side."""
    width, _height, _depth = ZVE10_BODY
    centre = -spec.camera_x - ZVE10_SOCKET[0]
    return centre + width / 2 + 2.0


@cache
def head_skin(spec: Params) -> Part:
    outline = Wire(head_turtle(spec).soft(HEAD_TRIM, closed=True))
    skin = g2_skin(outline, spec.head_t, HEAD_BED, HEAD_ROLL)
    plane = Plane(origin=(0, spec.head_t / 2, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0))
    return plane.location * skin


def camera_socket(spec: Params) -> Vector:
    """Where the camera's tripod socket sits: on the platform top, past arm_tip."""
    return Vector(-spec.camera_x, 0, head_heights(spec)[2])


def build_head(spec: Params) -> Part:
    block = head_block_t()
    up = Vector(0, -1, 0)  # print-up: the head prints on its +Y face
    body = head_skin(spec)
    body -= teardrop_rod(SHOULDER_M6.shoulder_d + 0.4, block + 2, (-1, 0, 0), (1, 0, 0), up)
    body -= teardrop_rod(SPOT_D, SPOT + 1, (block - SPOT, 0, 0), (1, 0, 0), up)
    socket = camera_socket(spec)
    _face_top, under, top = head_heights(spec)
    body -= teardrop_rod(QUARTER_CLEAR, PLATFORM_T + 2, socket - Vector(0, 0, PLATFORM_T + 1),
                         (0, 0, 1), up)
    seat = top - PLATE_UNDER_HEAD
    body -= teardrop_rod(QUARTER_WASHER_SEAT, seat - under + 1,
                         Vector(socket.X, 0, under - 1), (0, 0, 1), up)
    return named(body, "arm_head")


# --- Knob and ring -------------------------------------------------------------------------------


KNOB_CEILING = 1.4  # 45 degree ring over the head pocket: the flat ceiling bridges <= 10


def knob_height() -> float:
    return knob_skirt() + SHOULDER_M6.head_k + KNOB_CEILING + KNOB_ROOF + KNOB_CROWN_H


def knob_profile_wire(spec: Params) -> Wire:
    """(r, z): bed lead, straight wall, G2 blend, an elliptic crown (exact conic) to the axis."""
    radius, height = spec.knuckle_r * 0.9, knob_height()
    a, b = KNOB_CROWN_R, KNOB_CROWN_H
    theta = radians(KNOB_CROWN_START)
    on_crown = Vector(a * cos(theta), height - b + b * sin(theta))
    travel = Vector(-a * sin(theta), b * cos(theta)).normalized()
    curvature = a * b / (a * a * sin(theta) ** 2 + b * b * cos(theta) ** 2) ** 1.5
    inward = Vector(travel.Y, -travel.X)
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


def knob_d(spec: Params) -> float:
    return 2 * spec.knuckle_r * 0.9


def build_knob(spec: Params) -> Part:
    # Two 180 degree halves: a single 360 degree revolve of the crown and its G2 blend
    # does not survive the STEP round trip (it reads back 12 % small and invalid).
    profile = Plane.XZ.location * make_face(knob_profile_wire(spec)).faces()[0]
    half = revolve(profile, Axis.Z, 180)
    body = half + half.rotate(Axis.Z, 180)
    radius = knob_d(spec) / 2
    for k in range(KNOB_FLUTES):
        angle = radians(360 / KNOB_FLUTES * (k + 0.5))
        reach = radius + KNOB_FLUTE_OFFSET
        centre = Vector(reach * cos(angle), reach * sin(angle), -1)
        body -= rod(2 * KNOB_FLUTE_R, knob_skirt() + SHOULDER_M6.head_k + 1, centre, (0, 0, 1))
    skirt = knob_skirt()
    body -= rod(SPOT_D + 0.2, skirt + 1, (0, 0, -1), (0, 0, 1))
    pocket = SHOULDER_M6.head_d - HEAD_PRESS
    body -= rod(pocket, SHOULDER_M6.head_k + 0.01, (0, 0, skirt), (0, 0, 1))
    top = skirt + SHOULDER_M6.head_k
    body -= Pos(0, 0, top) * Cone(pocket / 2, pocket / 2 - KNOB_CEILING, KNOB_CEILING,
                                  align=(Align.CENTER, Align.CENTER, Align.MIN))
    return named(body, "knob")


def build_ring(spec: Params) -> Part:
    ring = rod(RING_OUTER, RING_T, (0, 0, 0), (0, 0, 1))
    ring -= rod(RING_INNER, RING_T + 2, (0, 0, -1), (0, 0, 1))
    return named(ring, "j3_ring")


# --- Checks --------------------------------------------------------------------------------------


def _printed(part: Part) -> list[Check]:
    return [one_solid(part), *overhang_rows(print_orientation(part, PRINT[part.label]),
                                            part.label)]


def checks_base(part: Part, spec: Params) -> list[Check]:
    rows = _printed(part)
    on_bed = lambda c: abs(c.Z) < 1e-4  # noqa: E731
    rows += skin_rows(part, "arm_base", on_bed, pedestal_skin(spec))
    profile = pedestal_profile(spec)
    jump = max(curvature_jumps(Wire(profile.edges()[:-3])))
    rows.append(Check("arm_base profile curvature jump (G2)", jump <= 1e-3,
                      f"{jump:.2e} /mm (max 1e-3)"))
    rows += pivot_insert().checks(part)
    rows.append(at_least("arm_base neck radius", neck_r(spec), 11.0))
    return rows


def _link_checks(part: Part, spec: Params, name: str) -> list[Check]:
    t = link_t(spec)
    rows = _printed(part)
    rows += skin_rows(part, name, lambda c: abs(c.X - t) < 1e-4, link_skin(spec))
    jump = max(curvature_jumps(dog_bone(spec).outer_wire()))
    rows.append(Check(f"{name} plan curvature jump (G2)", jump <= 1e-3, f"{jump:.2e} /mm"))
    ends = ("A", "B") if name == "arm_link" else ("B",)
    wall = tooth_depth(spec) if name == "arm_link" else 0.0
    for insert in link_inserts(spec, ends):
        rows += insert.checks(part, wall_from=wall)
        rows.append(at_least(f"{name} PETG under the {insert.label} insert hole",
                             t - insert.depth, 0.6))
    return rows


def checks_link(part: Part, spec: Params) -> list[Check]:
    return _link_checks(part, spec, "arm_link")


def checks_tip(part: Part, spec: Params) -> list[Check]:
    return _link_checks(part, spec, "arm_tip")


def checks_head(part: Part, spec: Params) -> list[Check]:
    rows = _printed(part)
    on_bed = lambda c: abs(c.Y - spec.head_t / 2) < 1e-4  # noqa: E731
    rows += skin_rows(part, "arm_head", on_bed, head_skin(spec))
    jump = max(curvature_jumps(Wire(head_turtle(spec).soft(HEAD_TRIM, closed=True))))
    rows.append(Check("arm_head side outline curvature jump (G2)", jump <= 1e-3,
                      f"{jump:.2e} /mm"))
    flat = spec.head_t / 2 - HEAD_ROLL.setback
    rows.append(at_least("arm_head J3 face flat half-width (ring needs 13)", flat,
                         RING_OUTER / 2))
    face_top, _under, _top = head_heights(spec)
    rows.append(at_least("arm_head friction face flat above J3 (ring needs 13)", face_top,
                         RING_OUTER / 2))
    rows.append(at_least("arm_head friction face flat below J3 (ring needs 13 + blend)",
                         spec.knuckle_r + HEAD_DROP - HEAD_CORNER[0] - HEAD_TRIM,
                         RING_OUTER / 2))
    return rows


def checks_knob(part: Part, spec: Params) -> list[Check]:
    rows = _printed(part)
    rows.append(at_least("knob roof over the screw head pocket",
                         knob_height() - KNOB_CROWN_H - knob_skirt() - SHOULDER_M6.head_k
                         - KNOB_CEILING, PETG_WALL))
    flute_floor = knob_d(spec) / 2 + KNOB_FLUTE_OFFSET - KNOB_FLUTE_R
    rows.append(at_least("knob wall round the skirt bore, at a flute",
                         flute_floor - (SPOT_D + 0.2) / 2, PETG_WALL))
    rows.append(at_least("knob skirt over the head washer", knob_skirt(), WASHER_M8.t))
    outside = Wire(knob_profile_wire(spec).edges()[1:-1])  # not the base or the axis
    jump = max(curvature_jumps(outside))
    rows.append(Check("knob crown curvature jump (G2)", jump <= 1e-3, f"{jump:.2e} /mm"))
    return rows


def checks_ring(part: Part, spec: Params) -> list[Check]:
    rows = _printed(part)
    rows.append(at_least("j3_ring radial wall", (RING_OUTER - RING_INNER) / 2, 1.2))
    rows.append(at_least("j3_ring outer edge inside the arm_tip knuckle face",
                         spec.knuckle_r - ROSETTE_ROLL.setback - RING_OUTER / 2, 0.3))
    return rows
