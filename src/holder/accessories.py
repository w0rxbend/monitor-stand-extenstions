"""Accessories on the shared MF-40 foot: zve10 (hero), cam_plate, phone, gopro, post.

Every body is a G2 skin, or a turned or lofted body joined to one. Every foot keeps
foot_under_head of PETG under the M3 washer, so one M3 length fits every foot.
Camera screws come up from below in a washer counterbore that sits outside the MF
footprint, so the key reaches them with the plate mounted.

The ZV-E10 body is 115.2 x 64.2 x 44.8 mm (Sony help guide). The tripod socket offset is
not on that page, so the camera screw sits in a 24 mm slot. GoPro prongs are not printed:
a 1/4" bolt drops into a 13 x 5 pocket and the bought buckle threads onto it. The phone
ear was scaled from a photo on a 5 mm dot grid, not calipered.
"""

from build123d import Axis, Face, Line, Part, Plane, Pos, Vector, Wire, loft, make_face, revolve

from holder.common import (
    GABLE_HEAD,
    Check,
    SeatedInsert,
    annulus_footprint,
    bearing_row,
    box_from,
    contact_area,
    insert_shear_row,
    named,
    one_solid,
    overhang_rows,
    print_orientation,
    rod,
    skin_rows,
    void_open,
)
from holder.form import BedLead, Roll, corner_curve, g2_skin, skin_profile, soft_outline, soft_rect
from holder.mount import MountFace, foot_checks, foot_cutters, foot_slab, m3_square
from holder.params import (
    M3_CLEAR,
    M3_INSERT,
    M6_LARGE_WASHER,
    PRELOAD_N,
    QUARTER_CLEAR,
    QUARTER_WASHER_SLOT,
    ZVE10_BODY,
    Params,
)
from holder.stacks import camera_plate_t

PLATE_TOP = BedLead(0.6, 2.0)  # camera plates print deck face down: the washer slots open up
PLATE_BOTTOM = Roll(3.0)
PLATE_BULGE = 0.25
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
PHONE_WALL_HALF_Y = 5.9  # walls stop 0.4 short of the 7.4 washer counterbores at y +-10
PHONE_WALL_OUTER = 17.0  # inside the foot's flat top, clear of its 2.5 roll
PHONE_WALL_TOP = 3.5  # G2 corner setback at the wall top (two fit the 7.5 wall)
BURY = 2.5  # a wall outline runs this far into the foot
BURY_SETBACK = 1.2  # its buried corners, radius 0.56, under the 0.5 wall-face roll
STUD_POCKET_D = 13.0
STUD_POCKET_H = 5.0
GOPRO_BOSS_D = 20.0
GOPRO_BOSS_TOP = 16.0
GOPRO_ROLL = 2.0
GOPRO_ROOT = 2.5
POST_COLUMN_W = 24.0
POST_BLOCK_Z = 64.0
POST_OGEE = 24.0  # the column flares into the MF plan; the corners stay under 45 deg
POST_FLUTE_D = 7.0
POST_TOP = Roll(3.0)
POST_PITCH = 1.0
# The post prints MF up: its 1/4" channel ceiling (11.45 wide) is the nut seat itself, so it
# gets 45 degree edge strips instead of a gable (common.GABLE_HEAD).
POST_QUARTER_ROOF = 6.1

PRINT = {"zve10": (0, 0, 1), "cam_plate": (0, 0, 1),
         **{name: (0, 0, -1) for name in ("phone", "gopro", "post")}}
MATERIAL = dict.fromkeys(PRINT, "PETG")


def _cut(body: Part, cutters: list[Part]) -> Part:
    for cutter in cutters:
        body -= cutter
    return body


def plate_skin(outline: Wire, thickness: float) -> Part:
    return g2_skin(outline, thickness, PLATE_BOTTOM, PLATE_TOP, PLATE_BULGE)


def _camera_slot(x0: float, x1: float, y: float, thickness: float) -> list[Part]:
    """Clearance slot through the plate, and the washer counterbore slot from below."""
    bore_d, bore_h = QUARTER_WASHER_SLOT
    cutters = []
    if x1 > x0:
        half = QUARTER_CLEAR / 2
        cutters += [box_from(x0, y - half, -1, x1, y + half, thickness + 1),
                    box_from(x0, y - bore_d / 2, -1, x1, y + bore_d / 2, bore_h)]
    for x in (x0, x1):
        cutters += [rod(QUARTER_CLEAR, thickness + 2, Vector(x, y, -1), Vector(0, 0, 1)),
                    rod(bore_d, bore_h + 1, Vector(x, y, -1), Vector(0, 0, 1))]
    return cutters


def zve10_plan() -> Wire:
    half_w = ZVE10_BODY[0] / 2 + 0.4
    right = [(half_w, ZVE10_DECK_FRONT), (half_w, ZVE10_SHOULDER_Y), (20.0, -10.0), (20.0, 20.0)]
    vertices = right + [(-x, y) for x, y in reversed(right)]
    setbacks = [14.0, 10.0, 8.0, 12.0]
    return soft_outline(vertices, setbacks + setbacks[::-1])


def zve10_camera_y() -> float:
    return (ZVE10_DECK_FRONT + ZVE10_SHOULDER_Y) / 2


def build_zve10(spec: Params) -> Part:
    thickness = camera_plate_t()
    body = plate_skin(zve10_plan(), thickness)
    y = zve10_camera_y()
    body = _cut(body, foot_cutters(spec, thickness)
                + _camera_slot(-CAMERA_SLOT / 2, CAMERA_SLOT / 2, y, thickness))
    return named(body, "zve10")


def cam_plate_plan() -> Wire:
    depth = 64.0
    return soft_rect(CAM_PLATE_W, depth, 20.0, (0, 20.0 - depth / 2)).outer_wire()


def build_cam_plate(spec: Params) -> Part:
    thickness = camera_plate_t()
    body = plate_skin(cam_plate_plan(), thickness)
    cutters = foot_cutters(spec, thickness) + _camera_slot(0, 0, CAM_STATION_Y, thickness)
    for x0, x1 in CAM_SLOTS:
        cutters += _camera_slot(x0, x1, CAM_STATION_Y, thickness)
    return named(_cut(body, cutters), "cam_plate")


def phone_insert(spec: Params) -> SeatedInsert:
    """The ear screw goes into an insert in the foot top; its tip runs on into the MF port."""
    return SeatedInsert("phone ear", M3_INSERT, Vector(0, 0, spec.foot_t), Vector(0, 0, -1),
                        spec.foot_t + 1.0, M3_CLEAR)


def phone_wall_outline(sign: float, foot_t: float) -> Wire:
    """Wall in (x, z): G2 corners at the top, buried corners in the foot."""
    inner, outer = PHONE_SLOT_W / 2, PHONE_WALL_OUTER
    low, high = sign * inner, sign * outer
    left, right = min(low, high), max(low, high)
    bottom, top = foot_t - BURY, PHONE_WALL_H
    vertices = [(left, bottom), (right, bottom), (right, top), (left, top)]
    setbacks = [BURY_SETBACK, BURY_SETBACK, PHONE_WALL_TOP, PHONE_WALL_TOP]
    return soft_outline(vertices, setbacks)


def phone_wall(sign: float, foot_t: float) -> Part:
    skin = g2_skin(phone_wall_outline(sign, foot_t), 2 * PHONE_WALL_HALF_Y, Roll(0.5),
                   Roll(0.5))
    return Plane.XZ.offset(PHONE_WALL_HALF_Y).location * skin


def phone_skins(spec: Params) -> list[Part]:
    return [foot_slab(spec, spec.foot_t), phone_wall(-1.0, spec.foot_t),
            phone_wall(1.0, spec.foot_t)]


def build_phone(spec: Params) -> Part:
    foot, left, right = phone_skins(spec)
    body = foot + left + right
    cutters = foot_cutters(spec, spec.foot_t) + phone_insert(spec).cutters()
    return named(_cut(body, cutters), "phone")


def gopro_profile(spec: Params) -> Wire:
    """Turned boss in (r, z): a G2 roll at the top, a G2 root into the foot top (buried)."""
    radius, top, foot = GOPRO_BOSS_D / 2, GOPRO_BOSS_TOP, spec.foot_t
    roll = corner_curve((0, top), (radius, top), (radius, foot), GOPRO_ROLL)
    root = corner_curve((radius, top), (radius, foot), (radius + 2 * GOPRO_ROOT, foot),
                        GOPRO_ROOT)
    out = radius + GOPRO_ROOT
    edges = [Line((0, top), roll @ 0), roll, Line(roll @ 1, root @ 0), root,
             Line(root @ 1, (out, foot - 2.0)), Line((out, foot - 2.0), (0, foot - 2.0)),
             Line((0, foot - 2.0), (0, top))]
    return Wire(edges)


def build_gopro(spec: Params) -> Part:
    boss = revolve(Plane.XZ.location * make_face(gopro_profile(spec)).faces()[0], Axis.Z, 360)
    body = foot_slab(spec, spec.foot_t) + boss
    cutters = foot_cutters(spec, spec.foot_t) + [
        rod(STUD_POCKET_D, STUD_POCKET_H + 1, Vector(0, 0, -1), Vector(0, 0, 1)),
        rod(QUARTER_CLEAR, GOPRO_BOSS_TOP + 2, Vector(0, 0, -1), Vector(0, 0, 1)),
    ]
    return named(_cut(body, cutters), "gopro")


def post_mount_face(spec: Params) -> MountFace:
    top = POST_BLOCK_Z + spec.mf_depth
    return MountFace("post MF", Vector(0, 0, top), Vector(0, -1, 0), spec.mf_size / 2 + 1.5,
                     quarter_roof=POST_QUARTER_ROOF, quarter_gable=GABLE_HEAD)


def _top_inset(height: float) -> list[tuple[float, float]]:
    profile = skin_profile(height, Roll(1.0), POST_TOP)
    points = [edge @ (k / 64) for edge in profile.edges() for k in range(65)]
    return sorted(((height - p.Y, p.X) for p in points
                   if p.Y > height - POST_TOP.setback - 1.0), key=lambda pair: pair[0])


def _inset_at(depth: float, table: list[tuple[float, float]]) -> float:
    for (d0, u0), (d1, u1) in zip(table, table[1:]):
        if d0 <= depth <= d1:
            return u0 if d1 - d0 < 1e-9 else u0 + (u1 - u0) * (depth - d0) / (d1 - d0)
    return 0.0


def post_sections(spec: Params) -> list[Face]:
    """Column sections: a squircle, an ogee into the MF plan, the MF block, a G2 top roll."""
    low, top = spec.foot_t - 2.0, POST_BLOCK_Z + spec.mf_depth
    start = POST_BLOCK_Z - POST_OGEE
    table = _top_inset(spec.mf_depth)
    small = (POST_COLUMN_W, POST_COLUMN_W / 2 - 0.2)
    big = (spec.mf_size, spec.mf_setback)
    count = round((top - low) / POST_PITCH)
    faces = []
    for k in range(count + 1):
        z = low + (top - low) * k / count
        u = min(max((z - start) / POST_OGEE, 0.0), 1.0)
        share = u * u * u * (10 - 15 * u + 6 * u * u)
        width, corner = (a + (b - a) * share for a, b in zip(small, big))
        inset = _inset_at(top - z, table)
        faces.append(Pos(0, 0, z) * soft_rect(width - 2 * inset, width - 2 * inset,
                                              corner - inset))
    return faces


def post_skins(spec: Params) -> tuple[Part, Part]:
    return foot_slab(spec, spec.foot_t), loft(post_sections(spec), ruled=False)


def build_post(spec: Params) -> Part:
    """Foot, fluted squircle column flaring into an MF. The flutes are the foot screws' key path."""
    foot, column = post_skins(spec)
    body = foot + column
    cutters = foot_cutters(spec, spec.foot_t) + post_mount_face(spec).cutters(spec)
    for dx, dy in m3_square(spec):
        cutters.append(rod(POST_FLUTE_D, POST_BLOCK_Z - spec.foot_t + 0.5,
                           Vector(dx, dy, spec.foot_t - 0.5), Vector(0, 0, 1)))
        cutters.append(rod(M3_CLEAR, spec.mf_depth, Vector(dx, dy, POST_BLOCK_Z - 0.5),
                           Vector(0, 0, 1)))
    return named(_cut(body, cutters), "post")


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


def _on_bed(centre: Vector) -> bool:
    return abs(centre.Z) < 1e-4


def _on_top(thickness: float):
    return lambda centre: abs(centre.Z - thickness) < 1e-4


def _slot_open(part: Part, x0: float, x1: float, y: float, z0: float, z1: float,
               name: str) -> Check:
    probe = rod(5.0, z1 - z0, Vector(x0, y, z0), Vector(0, 0, 1))
    if x1 > x0:
        probe += box_from(x0, y - 2.5, z0, x1, y + 2.5, z1)
        probe += rod(5.0, z1 - z0, Vector(x1, y, z0), Vector(0, 0, 1))
    return void_open(part, probe, name)


def camera_washer_row(part: Part, x: float, y: float, name: str) -> Check:
    """The camera screw's ISO 7093 washer on the slot ceiling, at the camera preload."""
    ceiling = Vector(x, y, QUARTER_WASHER_SLOT[1])
    footprint = annulus_footprint(ceiling, Vector(0, 0, 1), M6_LARGE_WASHER.outer,
                                  M6_LARGE_WASHER.inner)
    return bearing_row(name, PRELOAD_N["camera"], contact_area(part, footprint))


def checks_zve10(part: Part, spec: Params) -> list[Check]:
    thickness = camera_plate_t()
    y = zve10_camera_y()
    rows = _common(part, spec)
    rows += skin_rows(plate_skin(zve10_plan(), thickness), "zve10", _on_top(thickness))
    rows.append(_slot_open(part, -CAMERA_SLOT / 2, CAMERA_SLOT / 2, y, -0.5, thickness + 0.5,
                           "camera slot open"))
    for x in (-CAMERA_SLOT / 2, 0.0, CAMERA_SLOT / 2):
        rows.append(camera_washer_row(part, x, y, f"camera washer at x {x:g}"))
    return rows


def checks_cam_plate(part: Part, spec: Params) -> list[Check]:
    thickness = camera_plate_t()
    rows = _common(part, spec)
    rows += skin_rows(plate_skin(cam_plate_plan(), thickness), "cam_plate", _on_top(thickness))
    for x0, x1 in ((0.0, 0.0), *CAM_SLOTS):
        rows.append(_slot_open(part, x0, x1, CAM_STATION_Y, -0.5, thickness + 0.5,
                               f"camera station x {x0:g}..{x1:g} open"))
        rows.append(camera_washer_row(part, x0, CAM_STATION_Y, f"camera washer at x {x0:g}"))
    return rows


def checks_phone(part: Part, spec: Params) -> list[Check]:
    rows = _common(part, spec)
    foot, left, right = phone_skins(spec)
    rows += skin_rows(foot, "phone foot", _on_bed)
    rows += skin_rows(left, "phone wall", lambda _c: False)
    rows += phone_insert(spec).checks(part)
    rows.append(insert_shear_row("phone ear", M3_INSERT, PRELOAD_N["m3"]))
    ear = box_from(-PHONE_EAR_W / 2, -PHONE_WALL_HALF_Y - 1, spec.foot_t + 0.2,
                   PHONE_EAR_W / 2, PHONE_WALL_HALF_Y + 1, PHONE_WALL_H + 2)
    rows.append(void_open(part, ear, "ear slot open"))
    return rows


def checks_gopro(part: Part, spec: Params) -> list[Check]:
    rows = _common(part, spec)
    rows += skin_rows(foot_slab(spec, spec.foot_t), "gopro foot", _on_bed)
    probe = rod(QUARTER_CLEAR - 0.4, GOPRO_BOSS_TOP - STUD_POCKET_H - 0.6,
                Vector(0, 0, STUD_POCKET_H + 0.3), Vector(0, 0, 1))
    rows.append(void_open(part, probe, "stud bore open"))
    return rows


def checks_post(part: Part, spec: Params) -> list[Check]:
    rows = _common(part, spec)
    foot, column = post_skins(spec)
    rows += skin_rows(foot, "post foot", _on_bed)
    rows += skin_rows(column, "post column", lambda c: abs(c.Z - spec.foot_t + 2.0) < 1e-4
                      or abs(c.Z - POST_BLOCK_Z - spec.mf_depth) < 1e-4)
    rows += post_mount_face(spec).checks(part, spec)
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
