"""Accessories on the shared MF-40 foot: zve10 (hero), cam_plate, phone, gopro, post.

Every foot keeps 4.7 mm of PETG under the M3 heads, so one M3 x 12 fits every foot.
Camera screws come up from below in a counterbore that sits outside the MF footprint,
so the key reaches them with the plate mounted.

The ZV-E10 body is 115.2 x 64.2 x 44.8 mm (Sony help guide). The tripod socket offset is
not on that page, so the camera screw sits in a 24 mm slot. GoPro prongs are not printed:
a 1/4" bolt drops into a 13 x 5 pocket and the bought buckle threads onto it. The phone
ear was scaled from a photo on a 5 mm dot grid, not calipered.
"""

from build123d import (
    Axis,
    EllipticalCenterArc,
    Line,
    Part,
    Plane,
    Pos,
    Vector,
    Wire,
    chamfer,
    extrude,
    make_face,
    revolve,
)

from holder.common import (
    GABLE_HEAD,
    CaptiveNut,
    Check,
    box_from,
    encloses,
    lead_in,
    named,
    one_solid,
    overhang_rows,
    print_orientation,
    rod,
    stress_row,
    void_open,
)
from holder.form import soft_outline, soft_rect
from holder.mount import MountFace, foot_checks, foot_cutters, m3_square
from holder.params import (
    M3_CLEAR,
    M3_NUT,
    PETG_WALL,
    QUARTER_CLEAR,
    QUARTER_HEAD_BORE,
    ZVE10_BODY,
    Params,
)
from holder.stacks import camera_plate_t

EDGE = 1.0
BED_EDGE = 0.6
CAMERA_SLOT = 24.0
ZVE10_DECK_FRONT = -71.0
ZVE10_SHOULDER_Y = -26.0
CAM_PLATE_W = 300.0
CAM_STATION_Y = -32.0
CAM_SLOTS = ((-110.0, -70.0), (70.0, 110.0))
# Bought phone ear, from the photo on a 5 mm dot grid (32 px beside the ear).
# Not calipered. Width 104 px, hole 30 px. A 1/4" screw does not pass that hole.
PHONE_EAR_W = 16.0
PHONE_SLOT_W = PHONE_EAR_W + 3.0
PHONE_WALL_H = 14.0
PHONE_WALL_HALF_Y = 6.4  # walls stop short of the foot counterbores at y +-10
STUD_POCKET_D = 13.0
STUD_POCKET_H = 5.0
GOPRO_BOSS_D = 20.0
GOPRO_BOSS_TOP = 16.0
POST_COLUMN_D = 24.0
POST_BLOCK_Z = 64.0
POST_FLUTE_D = 7.0
# The post prints MF up: its 1/4" channel ceiling (11.45 wide) is the nut seat itself, so it
# gets 45 degree edge strips instead of a gable (common.GABLE_HEAD).
POST_QUARTER_ROOF = 6.1
POST_SHOULDER_R = 24.5  # past the soft-square corner (r 23.8), so the cone covers the block

PRINT = {name: (0, 0, -1) for name in ("zve10", "cam_plate", "phone", "gopro", "post")}
MATERIAL = dict.fromkeys(PRINT, "PETG")


def _loops(part: Part, top: float, bottom: float) -> Part:
    upper = [e for e in part.edges() if abs(e.center().Z - top) < 1e-4
             and e.bounding_box().size.Z < 1e-4]
    part = chamfer(upper, EDGE)
    lower = [e for e in part.edges() if abs(e.center().Z - bottom) < 1e-4
             and e.bounding_box().size.Z < 1e-4]
    return chamfer(lower, BED_EDGE)


def _cut(body: Part, cutters: list[Part]) -> Part:
    for cutter in cutters:
        body -= cutter
    return body


def _camera_slot(x0: float, x1: float, y: float, thickness: float) -> list[Part]:
    """Clearance slot through the plate, and the head counterbore slot from below."""
    head_d, head_h = QUARTER_HEAD_BORE
    cutters = []
    if x1 > x0:
        half = QUARTER_CLEAR / 2
        cutters += [box_from(x0, y - half, -1, x1, y + half, thickness + 1),
                    box_from(x0, y - head_d / 2, -1, x1, y + head_d / 2, head_h)]
    for x in (x0, x1):
        cutters += [rod(QUARTER_CLEAR, thickness + 2, Vector(x, y, -1), Vector(0, 0, 1)),
                    rod(head_d, head_h + 1, Vector(x, y, -1), Vector(0, 0, 1))]
    return cutters


def zve10_plan() -> object:
    half_w = ZVE10_BODY[0] / 2 + 0.4
    right = [(half_w, ZVE10_DECK_FRONT), (half_w, ZVE10_SHOULDER_Y), (20.0, -10.0), (20.0, 20.0)]
    vertices = right + [(-x, y) for x, y in reversed(right)]
    setbacks = [14.0, 10.0, 8.0, 12.0]
    return make_face(soft_outline(vertices, setbacks + setbacks[::-1])).faces()[0]


def zve10_camera_y() -> float:
    return (ZVE10_DECK_FRONT + ZVE10_SHOULDER_Y) / 2


def build_zve10(spec: Params) -> Part:
    thickness = camera_plate_t()
    body = _loops(extrude(zve10_plan(), amount=thickness), thickness, 0.0)
    y = zve10_camera_y()
    body = _cut(body, foot_cutters(spec, thickness)
                + _camera_slot(-CAMERA_SLOT / 2, CAMERA_SLOT / 2, y, thickness))
    return named(body, "zve10")


def build_cam_plate(spec: Params) -> Part:
    thickness = camera_plate_t()
    depth = 64.0
    plan = soft_rect(CAM_PLATE_W, depth, 20.0, (0, 20.0 - depth / 2))
    body = _loops(extrude(plan, amount=thickness), thickness, 0.0)
    cutters = foot_cutters(spec, thickness) + _camera_slot(0, 0, CAM_STATION_Y, thickness)
    for x0, x1 in CAM_SLOTS:
        cutters += _camera_slot(x0, x1, CAM_STATION_Y, thickness)
    return named(_cut(body, cutters), "cam_plate")


def phone_nut(spec: Params) -> CaptiveNut:
    seat = Vector(0, 0, spec.foot_t - spec.mf_m3_roof)
    return CaptiveNut("phone ear nut", M3_NUT, seat, Vector(0, 0, -1), Vector(1, 0, 0),
                      spec.mf_size / 2 + 1.5)


def _phone_wall(sign: float) -> Part:
    inner, outer = PHONE_SLOT_W / 2, 20.0
    radius = (outer - inner) / 2
    centre_x = sign * (inner + outer) / 2
    profile = make_face(Wire([
        Line((centre_x - radius, 0), (centre_x + radius, 0)),
        Line((centre_x + radius, 0), (centre_x + radius, PHONE_WALL_H - radius)),
        EllipticalCenterArc((centre_x, PHONE_WALL_H - radius), radius, radius, start_angle=0,
                            arc_size=180),
        Line((centre_x - radius, PHONE_WALL_H - radius), (centre_x - radius, 0)),
    ])).faces()[0]
    plane = Plane.XZ.offset(PHONE_WALL_HALF_Y)
    return extrude(plane.location * profile, amount=2 * PHONE_WALL_HALF_Y)


def build_phone(spec: Params) -> Part:
    foot = _loops(extrude(soft_rect(spec.mf_size, spec.mf_size, spec.mf_setback),
                          amount=spec.foot_t), spec.foot_t, 0.0)
    body = foot + _phone_wall(-1.0) + _phone_wall(1.0)
    nut = phone_nut(spec)
    cutters = foot_cutters(spec, spec.foot_t) + nut.cutters()
    cutters += [rod(M3_CLEAR, spec.foot_t + 2, Vector(0, 0, -1), Vector(0, 0, 1)),
                lead_in(M3_CLEAR, Vector(0, 0, spec.foot_t), Vector(0, 0, -1))]
    return named(_cut(body, cutters), "phone")


def _turned(profile_points: list[tuple[float, float]],
            fillet: tuple[float, float, float] | None = None) -> Part:
    """Revolve a closed (r, z) outline about Z.

    `profile_points` run from the axis out and down. `fillet` = (r, z, size) closes the
    outline through an elliptic concave root at (r, z); without it the last point drops
    straight to the axis.
    """
    edges = [Line(a, b) for a, b in zip(profile_points, profile_points[1:])]
    first, last = profile_points[0], profile_points[-1]
    if fillet is None:
        edges += [Line(last, (0, last[1])), Line((0, last[1]), first)]
    else:
        root_r, root_z, size = fillet
        arc = EllipticalCenterArc((root_r + size, root_z + size), size, size, start_angle=180,
                                  arc_size=90)
        edges += [Line(last, (root_r, root_z + size)), arc,
                  Line((root_r + size, root_z), (0, root_z)), Line((0, root_z), first)]
    face = Plane.XZ.location * make_face(Wire(edges)).faces()[0]
    return revolve(face, Axis.Z, 360)


def build_gopro(spec: Params) -> Part:
    foot = _loops(extrude(soft_rect(spec.mf_size, spec.mf_size, spec.mf_setback),
                          amount=spec.foot_t), spec.foot_t, 0.0)
    radius, top = GOPRO_BOSS_D / 2, GOPRO_BOSS_TOP
    boss = _turned([(0, top), (radius - EDGE, top), (radius, top - EDGE)],
                   (radius, spec.foot_t - 0.5, 2.5))
    body = foot + boss
    cutters = foot_cutters(spec, spec.foot_t) + [
        rod(STUD_POCKET_D, STUD_POCKET_H + 1, Vector(0, 0, -1), Vector(0, 0, 1)),
        rod(QUARTER_CLEAR, top + 2, Vector(0, 0, -1), Vector(0, 0, 1)),
        lead_in(QUARTER_CLEAR, Vector(0, 0, top), Vector(0, 0, -1)),
    ]
    return named(_cut(body, cutters), "gopro")


def post_mount_face(spec: Params) -> MountFace:
    top = POST_BLOCK_Z + spec.mf_depth
    reach = {(dx, dy): spec.mf_size / 2 + dy + 1.5 for dx, dy in [*m3_square(spec), (0.0, 0.0)]}
    return MountFace("post MF", Vector(0, 0, top), Vector(0, -1, 0), reach,
                     quarter_roof=POST_QUARTER_ROOF, quarter_gable=GABLE_HEAD)


def build_post(spec: Params) -> Part:
    """Foot, fluted column, MF block. The flutes give the long key a path to the foot screws."""
    foot = _loops(extrude(soft_rect(spec.mf_size, spec.mf_size, spec.mf_setback),
                          amount=spec.foot_t), spec.foot_t, 0.0)
    radius = POST_COLUMN_D / 2
    column = _turned([(0, POST_BLOCK_Z + 1), (radius, POST_BLOCK_Z + 1)],
                     (radius, spec.foot_t - 0.5, 4.0))
    top = POST_BLOCK_Z + spec.mf_depth
    block = Pos(0, 0, POST_BLOCK_Z) * extrude(
        soft_rect(spec.mf_size, spec.mf_size, spec.mf_setback), amount=spec.mf_depth)
    block = _loops(block, top, POST_BLOCK_Z)
    body = foot + column + block + post_shoulder(spec)
    cutters = foot_cutters(spec, spec.foot_t) + post_mount_face(spec).cutters(spec)
    for dx, dy in m3_square(spec):
        cutters.append(rod(POST_FLUTE_D, POST_BLOCK_Z - spec.foot_t + 0.5,
                           Vector(dx, dy, spec.foot_t - 0.5), Vector(0, 0, 1)))
        cutters.append(rod(M3_CLEAR, spec.mf_depth, Vector(dx, dy, POST_BLOCK_Z - 0.5),
                           Vector(0, 0, 1)))
    return named(_cut(body, cutters), "post")


def post_shoulder(spec: Params) -> Part:
    """45 degree turned cone from the column into the block, cut to the block plan.

    The block underside would otherwise be an unsupported ceiling in print.
    """
    radius = POST_COLUMN_D / 2
    reach = POST_SHOULDER_R
    low = POST_BLOCK_Z - (reach - radius)
    cone = _turned([(0, POST_BLOCK_Z + 0.5), (reach, POST_BLOCK_Z + 0.5),
                    (reach, POST_BLOCK_Z), (radius, low), (radius, low - 0.5)])
    plan = Pos(0, 0, low - 1) * extrude(soft_rect(spec.mf_size, spec.mf_size, spec.mf_setback),
                                        amount=POST_BLOCK_Z - low + 1)
    return cone & plan


BUILDERS = {
    "zve10": build_zve10,
    "cam_plate": build_cam_plate,
    "phone": build_phone,
    "gopro": build_gopro,
    "post": build_post,
}


# --- Checks --------------------------------------------------------------------------------


def _common(part: Part, spec: Params) -> list[Check]:
    return [one_solid(part), *overhang_rows(print_orientation(part, PRINT[part.label]),
                                            part.label), *foot_checks(part, spec)]


def _slot_open(part: Part, x0: float, x1: float, y: float, z0: float, z1: float,
               name: str) -> Check:
    probe = rod(5.0, z1 - z0, Vector(x0, y, z0), Vector(0, 0, 1))
    if x1 > x0:
        probe += box_from(x0, y - 2.5, z0, x1, y + 2.5, z1)
        probe += rod(5.0, z1 - z0, Vector(x1, y, z0), Vector(0, 0, 1))
    return void_open(part, probe, name)


def _deck_stress(part: Part, spec: Params, y: float, lever: float, name: str) -> Check:
    plane = Plane(origin=(0, y, 0), z_dir=(0, 1, 0))
    moment = spec.payload_kg * 9.81 * lever
    return stress_row(part, plane, Vector(1, 0, 0), moment, 0.1 * 38.5,
                      f"{name} sustained stress, along layers")


def checks_zve10(part: Part, spec: Params) -> list[Check]:
    thickness = camera_plate_t()
    y = zve10_camera_y()
    rows = _common(part, spec)
    rows.append(_slot_open(part, -CAMERA_SLOT / 2, CAMERA_SLOT / 2, y, -0.5, thickness + 0.5,
                           "camera slot open"))
    rows.append(_deck_stress(part, spec, -20.5, 20.5 - y, "zve10 shoulder"))
    return rows


def checks_cam_plate(part: Part, spec: Params) -> list[Check]:
    thickness = camera_plate_t()
    rows = _common(part, spec)
    for x0, x1 in ((0.0, 0.0), *CAM_SLOTS):
        rows.append(_slot_open(part, x0, x1, CAM_STATION_Y, -0.5, thickness + 0.5,
                               f"camera station x {x0:g}..{x1:g} open"))
    return rows


def checks_phone(part: Part, spec: Params) -> list[Check]:
    rows = _common(part, spec)
    nut = phone_nut(spec)
    rows += nut.checks(part)
    for name, probe in nut.wall_probes(PETG_WALL, M3_CLEAR):
        rows.append(encloses(part, probe, f"{name} >= {PETG_WALL}"))
    ear = box_from(-PHONE_EAR_W / 2, -PHONE_WALL_HALF_Y - 1, spec.foot_t + 0.2,
                   PHONE_EAR_W / 2, PHONE_WALL_HALF_Y + 1, PHONE_WALL_H + 2)
    rows.append(void_open(part, ear, "ear slot open"))
    return rows


def checks_gopro(part: Part, spec: Params) -> list[Check]:
    rows = _common(part, spec)
    probe = rod(QUARTER_CLEAR - 0.4, GOPRO_BOSS_TOP - STUD_POCKET_H - 0.6,
                Vector(0, 0, STUD_POCKET_H + 0.3), Vector(0, 0, 1))
    rows.append(void_open(part, probe, "stud bore open"))
    return rows


def checks_post(part: Part, spec: Params) -> list[Check]:
    rows = _common(part, spec)
    rows += post_mount_face(spec).checks(part, spec)
    plane = Plane(origin=(0, 0, spec.foot_t + 8.0), z_dir=(0, 0, 1))
    moment = spec.payload_kg * 9.81 * (2 * spec.link_pitch + abs(zve10_camera_y()))
    rows.append(stress_row(part, plane, Vector(1, 0, 0), moment, 0.1 * 19.0,
                           "post column sustained stress, across layers"))
    for dx, dy in m3_square(spec):
        path = rod(2.5 + 1.0, POST_BLOCK_Z + spec.mf_depth - spec.foot_t - 1,
                   Vector(dx, dy, spec.foot_t + 0.5), Vector(0, 0, 1))
        rows.append(void_open(part, path & box_from(-50, -50, 0, 50, 50, POST_BLOCK_Z - 0.5),
                              f"post key path ({dx:+g},{dy:+g}) up the flute"))
    return rows


CHECKS = {
    "zve10": checks_zve10,
    "cam_plate": checks_cam_plate,
    "phone": checks_phone,
    "gopro": checks_gopro,
    "post": checks_post,
}
