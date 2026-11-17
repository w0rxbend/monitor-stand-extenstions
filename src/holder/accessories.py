"""Parts that bolt to the block's 1/4"-20 nut.

Every foot has a 6.6 mm hole and a 12 mm counterbore. A mount face is the
other end: the same 32 mm pad with a captive 1/4" nut, so a foot also fits
an arm or the post.

The camera plate does not put a nut under the camera. The screw comes up
through the plate and threads into the camera. One screw cannot hold both
the plate and the camera.

GoPro prongs are not printed. A 1/4" screw sits in a downward pocket and
the bought buckle threads onto the end that sticks out. Pocket diameter and
depth are not checked against a specific screw.

The ZV-E10 body is 115.2 x 64.2 x 44.8 mm from grip to monitor (Sony help
guide). The tripod-socket offset is not on that page, so the camera screw
sits in a slot.

The phone cradle is not printed. The bought holder has one ear. `phone` is
the pocket for that ear. Width and hole size are from a photo, not calipers.
"""

from build123d import Cylinder, Part, Pos, Rot

from holder.band import build_halves
from holder.block import build as build_block
from holder.block import riser_top, top_hole_xy
from holder.common import (
    Check,
    box_from,
    hex_pocket,
    intersection_volume,
    one_solid,
    round_box,
    y_cylinder,
)
from holder.params import Params

PAD_T = 10.0
FOOT_T = 6.0
HEAD_D = 12.0
HEAD_H = 3.0
NUT_FLOOR = 2.0
M6 = 6.6
EAR_T = 6.0
GAP = 8.0
TANG = 7.2
LINK = 70.0
HINGE_Z = 24.0
JOINT_Y = 2.0
PLATE_T = 8.0
CAM_MOUNT_Y = -24.0
CAM_Y = 8.0
STUD_POCKET_D = 13.0
STUD_POCKET_H = 5.0
# Bought phone ear, from the photo on a 5 mm dot grid (32 px beside the ear).
# Not calipered. Width 104 px, hole 30 px. A 1/4" screw does not pass that hole.
PHONE_EAR_W = 16.0
PHONE_SLOT_W = PHONE_EAR_W + 3.0
PHONE_HOLE_Y = 10.0
PHONE_MOUNT_Y = -16.0
PHONE_WALL_Y0 = 0.0
PHONE_WALL_Y1 = 16.0
PHONE_WALL_Z = 14.0

# Base ears end at link-local Y = 12 once the tang hole (local Y = 0) sits on
# the base hole (Y = JOINT_Y). The beam starts past that, and the tang overlaps it.
BEAM_Y0 = 14.0
TANG_Y0 = -12.0
TANG_Y1 = 22.0
BEAM_Z0 = 18.0
BEAM_Z1 = 28.0


def _hole(x: float, y: float, z0: float, z1: float, diameter: float) -> Part:
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(diameter / 2, abs(z1 - z0) + 0.4)


def _x_hole(y: float, z: float, x0: float, x1: float, diameter: float) -> Part:
    rod = Rot(0, 90, 0) * Cylinder(diameter / 2, abs(x1 - x0))
    return Pos((x0 + x1) / 2, y, z) * rod


def _fuse(parts: list[Part]) -> Part:
    body = parts[0]
    for part in parts[1:]:
        body += part
    return body


def _subtract(body: Part, tools: list[Part]) -> Part:
    for tool in tools:
        body -= tool
    return body


def _label(part: Part, name: str) -> Part:
    part.label = name
    return part


def _foot_solid(x: float, y: float, z0: float) -> Part:
    return round_box(x - 16, y - 16, z0, x + 16, y + 16, z0 + FOOT_T, 6)


def _foot_tools(spec: Params, x: float, y: float, z0: float) -> list[Part]:
    top = z0 + FOOT_T
    bore = _hole(x, y, z0 - 0.2, top + 0.2, spec.quarter_through)
    head = _hole(x, y, top - HEAD_H, top + 0.2, HEAD_D)
    return [bore, head]


def _mount_solid(x: float, y: float, z0: float) -> Part:
    return round_box(x - 16, y - 16, z0, x + 16, y + 16, z0 + PAD_T, 6)


def _mount_tools(spec: Params, x: float, y: float, z0: float) -> list[Part]:
    """Through-hole plus a captive nut. The slot opens toward +X."""
    z1 = z0 + PAD_T
    bore = _hole(x, y, z0 - 0.2, z1 + 0.2, spec.quarter_through)
    nut_z = z0 + NUT_FLOOR
    pocket = hex_pocket(spec.quarter_hex_af, spec.quarter_hex_h, at=(x, y, nut_z))
    slot = box_from(
        x,
        y - spec.quarter_hex_af / 2,
        nut_z,
        x + 16.2,
        y + spec.quarter_hex_af / 2,
        nut_z + spec.quarter_hex_h,
    )
    return [bore, pocket, slot]


def _ears(y0: float, y1: float, z0: float, z1: float) -> list[Part]:
    ears = []
    for sign in (-1.0, 1.0):
        inner = sign * GAP / 2
        outer = inner + sign * EAR_T
        ears.append(
            round_box(min(inner, outer), y0, z0, max(inner, outer), y1, z1, 11, along="x")
        )
    return ears


def _tang(y_hole: float) -> Part:
    return round_box(
        -TANG / 2,
        y_hole + TANG_Y0,
        HINGE_Z - 12,
        TANG / 2,
        y_hole + TANG_Y1,
        HINGE_Z + 10,
        10,
        along="x",
    )


def _beam(y0: float, y1: float) -> Part:
    return round_box(-7, y0, BEAM_Z0, 7, y1, BEAM_Z1, 4.5, along="y")


def _bore_open(part: Part, probe: Part, name: str) -> Check:
    volume = intersection_volume(part, probe)
    return (name, volume <= 1e-3, f"{volume:.4f} mm3 of material in the bore")


def build_cam_plate(spec: Params) -> Part:
    """300 mm plate. Rear hole bolts down. Center camera is fixed. Sides slide."""
    plate = round_box(-150, -40, 0, 150, 24, PLATE_T, 32)
    tools = [
        *_foot_tools(spec, 0, CAM_MOUNT_Y, 0),
        _hole(0, CAM_Y, -0.2, PLATE_T + 0.2, spec.quarter_through),
        _hole(0, CAM_Y, -0.2, HEAD_H, HEAD_D),
    ]
    for x0, x1 in ((-110.0, -70.0), (70.0, 110.0)):
        tools.append(box_from(x0, CAM_Y - 3.3, -0.2, x1, CAM_Y + 3.3, PLATE_T + 0.2))
        tools.append(box_from(x0 - 3, CAM_Y - 6, -0.2, x1 + 3, CAM_Y + 6, HEAD_H))
    return _label(_subtract(plate, tools), "cam_plate")


def build_hinge_base(spec: Params) -> Part:
    """Foot plus a clevis. One M6 bolt is the hinge."""
    body = _fuse([_foot_solid(0, -16, 0), *_ears(-10, 14, 4, 36)])
    tools = [*_foot_tools(spec, 0, -16, 0), _x_hole(JOINT_Y, HINGE_Z, -22, 22, M6)]
    return _label(_subtract(body, tools), "hinge_base")


def build_hinge_link(spec: Params) -> Part:
    """Tang, beam, and a mount face. With hinge_base this is the 1 DOF arm."""
    face_z = HINGE_Z - PAD_T / 2
    body = _fuse([_tang(0), _beam(BEAM_Y0, LINK - 6), _mount_solid(0, LINK, face_z)])
    tools = [*_mount_tools(spec, 0, LINK, face_z), _x_hole(0, HINGE_Z, -12, 12, M6)]
    return _label(_subtract(body, tools), "hinge_link")


def build_hinge_mid(_spec: Params) -> Part:
    """Tang to clevis. Put it between hinge_base and hinge_link for 2 DOF."""
    body = _fuse(
        [
            _tang(0),
            _beam(BEAM_Y0, LINK - 16),
            *_ears(LINK - 22, LINK + 12, HINGE_Z - 14, HINGE_Z + 12),
        ]
    )
    tools = [
        _x_hole(0, HINGE_Z, -12, 12, M6),
        _x_hole(LINK, HINGE_Z, -22, 22, M6),
    ]
    return _label(_subtract(body, tools), "hinge_mid")


def build_gopro(spec: Params) -> Part:
    """Foot, plus a pocket a 1/4" screw drops into before the foot is bolted down."""
    pad = round_box(-16, -32, 0, 16, 28, FOOT_T, 16)
    boss = Pos(0, 14, 10) * Cylinder(10, 12)
    pocket = _hole(0, 14, -0.2, STUD_POCKET_H, STUD_POCKET_D)
    stud = _hole(0, 14, STUD_POCKET_H - 0.6, 16.2, spec.quarter_through)
    body = _subtract(_fuse([pad, boss]), [*_foot_tools(spec, 0, -16, 0), pocket, stud])
    return _label(body, "gopro")


def build_zve10(spec: Params) -> Part:
    """Plate for one ZV-E10. The camera screw slides. The socket offset is unverified."""
    plate = round_box(-70, -24, 0, 70, 80, FOOT_T, 20)
    lip = round_box(-61, 8, FOOT_T - 2, 61, 76, FOOT_T + 1.5, 8)
    opening = round_box(-58, 11, FOOT_T - 2.2, 58, 73, FOOT_T + 1.7, 6)
    slot = box_from(-12, 40, -0.2, 12, 46.6, FOOT_T + 0.2)
    head = box_from(-15, 37, -0.2, 15, 49.6, HEAD_H)
    body = _fuse([plate, lip]) - opening
    tools = [*_foot_tools(spec, 0, -8, 0), slot, head]
    return _label(_subtract(body, tools), "zve10")


def build_post(spec: Params) -> Part:
    cap_z = 74.0
    column = Pos(0, 0, (4 + cap_z + 8) / 2) * Cylinder(9, cap_z + 4)
    body = _fuse([_foot_solid(0, 0, 0), column, _mount_solid(0, 0, cap_z)])
    tools = [*_foot_tools(spec, 0, 0, 0), *_mount_tools(spec, 0, 0, cap_z)]
    return _label(_subtract(body, tools), "post")


def _phone_wall(x0: float, x1: float) -> Part:
    """Wall with a cylindrical top. The cap overlaps the web so the two fuse."""
    radius = abs(x1 - x0) / 2
    center_x = (x0 + x1) / 2
    web = box_from(x0, PHONE_WALL_Y0, 0, x1, PHONE_WALL_Y1, PHONE_WALL_Z - radius + 0.4)
    cap = y_cylinder(center_x, PHONE_WALL_Z - radius, PHONE_WALL_Y0, PHONE_WALL_Y1, 2 * radius)
    return web + cap


def build_phone(spec: Params) -> Part:
    """Pocket for the bought ear. Walls stop spin. An M3 screw clamps the ear down."""
    half = PHONE_SLOT_W / 2
    pad = round_box(-16, PHONE_MOUNT_Y - 16, 0, 16, PHONE_WALL_Y1, FOOT_T, 8)
    walls = [_phone_wall(-16, -half), _phone_wall(half, 16)]
    ear_hole = _hole(0, PHONE_HOLE_Y, -0.2, FOOT_T + 0.2, spec.m3_through)
    nut = hex_pocket(spec.m3_hex_af, spec.m3_hex_h, at=(0, PHONE_HOLE_Y, 0))
    nut_slot = box_from(
        0,
        PHONE_HOLE_Y - spec.m3_hex_af / 2,
        0,
        16.2,
        PHONE_HOLE_Y + spec.m3_hex_af / 2,
        spec.m3_hex_h,
    )
    body = _fuse([pad, *walls])
    tools = [*_foot_tools(spec, 0, PHONE_MOUNT_Y, 0), ear_hole, nut, nut_slot]
    return _label(_subtract(body, tools), "phone")


BUILDERS = (
    build_cam_plate,
    build_hinge_base,
    build_hinge_link,
    build_hinge_mid,
    build_gopro,
    build_zve10,
    build_post,
    build_phone,
)

PRINT = {name: (0, 0, -1) for name in (
    "cam_plate",
    "hinge_base",
    "hinge_link",
    "hinge_mid",
    "gopro",
    "zve10",
    "post",
    "phone",
)}


def place(part: Part, y: float, z: float = 0.0) -> Part:
    moved = part.moved(Pos(0, y, z))
    moved.label = part.label
    return moved


def on_link_face(plate: Part, link_shift_y: float, z: float = 0.0) -> Part:
    """Sit the plate's rear hole on the link mount face, 0.4 mm above the pad."""
    face_y = link_shift_y + LINK
    face_top = z + HINGE_Z + PAD_T / 2
    return place(plate, face_y - CAM_MOUNT_Y, face_top + 0.4)


def _riser_shift(spec: Params) -> tuple[float, float]:
    """Y and Z that put the hinge foot hole on the block's 1/4\" nut, 0.4 mm above the pad."""
    _x, quarter_y = top_hole_xy(spec)[1]
    return quarter_y + 16.0, riser_top(spec) + 0.4


def clamp_assembly(spec: Params) -> dict[str, Part]:
    """Band, block on boss_face_y, one hinge, and the camera plate. No hinge_mid."""
    front, back = build_halves(spec)
    brace = build_block(spec)
    shift_y, shift_z = _riser_shift(spec)
    base = place(build_hinge_base(spec), shift_y, shift_z)
    link = place(build_hinge_link(spec), shift_y + JOINT_Y, shift_z)
    plate = on_link_face(build_cam_plate(spec), shift_y + JOINT_Y, shift_z)
    return {
        "band_front": front,
        "band_back": back,
        "block": brace,
        "hinge_base": base,
        "hinge_link": link,
        "cam_plate": plate,
    }


def add_hinge_mid(stack: dict[str, Part], spec: Params) -> dict[str, Part]:
    """One-hinge stack plus hinge_mid. The link and the plate move out by one link."""
    shift_y, shift_z = _riser_shift(spec)
    placed = dict(stack)
    placed["hinge_mid"] = place(build_hinge_mid(spec), shift_y + JOINT_Y, shift_z)
    link_y = shift_y + JOINT_Y + LINK
    placed["hinge_link"] = place(build_hinge_link(spec), link_y, shift_z)
    placed["cam_plate"] = on_link_face(build_cam_plate(spec), link_y, shift_z)
    return placed


def _foot_probe(spec: Params, x: float, y: float, z0: float) -> Part:
    return _hole(x, y, z0 + 0.4, z0 + FOOT_T - HEAD_H - 0.2, spec.quarter_through - 0.4)


def _face_probe(spec: Params, x: float, y: float, z0: float) -> Part:
    return _hole(x, y, z0 + 0.4, z0 + PAD_T - 0.4, spec.quarter_through - 0.4)


def _m6_probe(y: float) -> Part:
    return _x_hole(y, HINGE_Z, -TANG / 2 + 0.3, TANG / 2 - 0.3, M6 - 0.4)


def _bores(label: str, spec: Params) -> list[tuple[str, Part]]:
    face_z = HINGE_Z - PAD_T / 2
    bores = {
        "cam_plate": [
            ("mount bore open", _foot_probe(spec, 0, CAM_MOUNT_Y, 0)),
            ("camera bore open", _hole(0, CAM_Y, HEAD_H + 0.3, PLATE_T - 0.3, 4)),
            ("camera bore open", _hole(-90, CAM_Y, HEAD_H + 0.3, PLATE_T - 0.3, 4)),
            ("camera bore open", _hole(90, CAM_Y, HEAD_H + 0.3, PLATE_T - 0.3, 4)),
        ],
        "hinge_base": [
            ("mount bore open", _foot_probe(spec, 0, -16, 0)),
            ("hinge bore open", _x_hole(JOINT_Y, HINGE_Z, -8, 8, M6 - 0.4)),
        ],
        "hinge_link": [
            ("mount bore open", _face_probe(spec, 0, LINK, face_z)),
            ("hinge bore open", _m6_probe(0)),
        ],
        "hinge_mid": [
            ("hinge bore open", _m6_probe(0)),
            ("hinge bore open", _x_hole(LINK, HINGE_Z, -8, 8, M6 - 0.4)),
        ],
        "gopro": [
            ("mount bore open", _foot_probe(spec, 0, -16, 0)),
            ("stud bore open", _hole(0, 14, STUD_POCKET_H + 0.4, 14, spec.quarter_through - 0.4)),
        ],
        "zve10": [
            ("mount bore open", _foot_probe(spec, 0, -8, 0)),
            ("camera bore open", _hole(0, 43.3, HEAD_H + 0.3, FOOT_T - 0.3, 4)),
        ],
        "post": [
            ("mount bore open", _foot_probe(spec, 0, 0, 0)),
            ("mount bore open", _face_probe(spec, 0, 0, 74)),
        ],
        "phone": [
            ("mount bore open", _foot_probe(spec, 0, PHONE_MOUNT_Y, 0)),
            (
                "ear bore open",
                _hole(
                    0,
                    PHONE_HOLE_Y,
                    spec.m3_hex_h + 0.3,
                    FOOT_T - 0.3,
                    spec.m3_through - 0.4,
                ),
            ),
            (
                "ear slot open",
                box_from(-8, 2, FOOT_T + 0.4, 8, 14, PHONE_WALL_Z - 0.4),
            ),
        ],
    }
    return bores[label]


def checks(part: Part, spec: Params) -> list[Check]:
    """One solid, and each bolt bore is open."""
    results = [one_solid(part)]
    for name, probe in _bores(part.label, spec):
        results.append(_bore_open(part, probe, name))
    return results
