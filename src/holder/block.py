"""PETG L block: plate on the boss, arm, riser with the extension face."""

from build123d import Cylinder, GeomType, Part, Pos

from holder.band import boss_face_y, boss_hole_centers, boss_z0
from holder.common import (
    VOL_EPS,
    Check,
    box_from,
    circles_on_yz_axis,
    hex_from_y_face,
    hex_pocket,
    hex_wall_ring,
    intersection_volume,
    one_solid,
    round_box,
    void_open,
    wall_encloses,
    y_cylinder,
)
from holder.params import MATERIALS, Params

NAME = "block"
MATERIAL = "PETG"
PRINT = {"block": (-1, 0, 0)}  # on its side; -X is the bed, so nut slots stay on ±Y

# Nut is 2.6 thick. 12 mm leaves a wall on both sides and matches boss_depth.
PLATE_DEPTH = 12.0
# Void probe starts here: below the 1/4" pocket, clear of the plate screws.
ARM_SECTION = 16.0
# Nut slots need a flat mouth, so the plate and riser stay at R=3.
# R=5 is the largest arm corner that stays inside the blank after the seam fillet.
PLATE_CORNER_R = 3.0
ARM_CORNER_R = 5.0
RISER_CORNER_R = 3.0
HOLE_PAST = 3.0
M3_BELOW_TOP = 6.0
QUARTER_BELOW_TOP = 14.0


def riser_top(spec: Params) -> float:
    return boss_z0(spec) + spec.riser_height


def _plate_y1(spec: Params) -> float:
    return boss_face_y(spec) + PLATE_DEPTH


def _arm_y1(spec: Params) -> float:
    return _plate_y1(spec) + spec.arm_length


def _bar_top(spec: Params) -> float:
    """Arm top. Same plane as the plate, so the body is one bar and a riser."""
    return boss_z0(spec) + spec.boss_w


def _riser_y1(spec: Params) -> float:
    return _arm_y1(spec) + spec.block_size


def top_hole_xy(spec: Params) -> tuple[list[tuple[float, float]], tuple[float, float]]:
    """M3 centers and the 1/4\" center on the riser top. XZ pattern of the boss, in XY."""
    mid = _arm_y1(spec) + spec.block_size / 2
    half = spec.m3_spacing / 2
    m3 = [(sx * half, mid + sy * half) for sx in (-1.0, 1.0) for sy in (-1.0, 1.0)]
    return m3, (0.0, mid)


def _z_cyl(x: float, y: float, z0: float, z1: float, diameter: float) -> Part:
    height = abs(z1 - z0)
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(diameter / 2, height)


def _blank(spec: Params) -> Part:
    z0 = boss_z0(spec)
    half = spec.boss_w / 2
    plate = round_box(
        -half,
        boss_face_y(spec),
        z0,
        half,
        _plate_y1(spec),
        z0 + spec.boss_w,
        PLATE_CORNER_R,
        along="y",
    )
    hx = spec.block_size / 2
    arm = round_box(
        -hx,
        _plate_y1(spec),
        z0,
        hx,
        _arm_y1(spec),
        _bar_top(spec),
        ARM_CORNER_R,
        along="y",
    )
    riser = round_box(
        -hx, _arm_y1(spec), z0, hx, _riser_y1(spec), riser_top(spec), RISER_CORNER_R
    )
    # A 3 mm fillet on this riser step makes the 3MF mesh invalid. Leave the step sharp.
    return plate + arm + riser


def _hex_ring_y(af: float, wall: float, x: float, z: float, y0: float, y1: float) -> Part:
    outer = hex_from_y_face(af + 2 * wall, y1 - y0, x, y0, z, 1)
    inner = hex_from_y_face(af, y1 - y0 + 0.4, x, y0 - 0.2, z, 1)
    return outer - inner


def _nearest_z(spec: Params, z: float) -> float:
    z0 = boss_z0(spec)
    z1 = z0 + spec.boss_w
    return z0 if abs(z - z0) <= abs(z - z1) else z1


def _cut_plate(body: Part, spec: Params) -> Part:
    y_face = boss_face_y(spec)
    nut_y0 = y_face + (PLATE_DEPTH - spec.m3_hex_h) / 2
    nut_y1 = nut_y0 + spec.m3_hex_h
    for x, z in boss_hole_centers(spec):
        pocket = hex_from_y_face(spec.m3_hex_af, spec.m3_hex_h, x, nut_y0, z, 1)
        face_z = _nearest_z(spec, z)
        slot = box_from(
            x - spec.m3_hex_af / 2,
            nut_y0,
            min(face_z, z),
            x + spec.m3_hex_af / 2,
            nut_y1,
            max(face_z, z),
        )
        hole = y_cylinder(x, z, y_face - 0.2, nut_y1 + 0.4, spec.m3_through)
        body = body - pocket - slot - hole
    return body


def _pocket_z(spec: Params, below_top: float, height: float) -> tuple[float, float]:
    z1 = riser_top(spec) - below_top
    return z1 - height, z1


def _y_slot(x: float, y: float, y_face: float, z0: float, z1: float, af: float) -> Part:
    return box_from(x - af / 2, min(y, y_face), z0, x + af / 2, max(y, y_face), z1)


def _cut_top(body: Part, spec: Params) -> Part:
    m3_xy, (qx, qy) = top_hole_xy(spec)
    y_lo = _arm_y1(spec)
    y_hi = _riser_y1(spec)
    top = riser_top(spec)
    for x, y in m3_xy:
        z0, z1 = _pocket_z(spec, M3_BELOW_TOP, spec.m3_hex_h)
        face = y_lo if abs(y - y_lo) <= abs(y - y_hi) else y_hi
        # The bar meets the plate. Pull arm-side mouths back through that bar.
        if face == y_lo:
            face = y_lo - spec.m3_hex_h
        pocket = hex_pocket(spec.m3_hex_af, spec.m3_hex_h, at=(x, y, z0))
        hole = _z_cyl(x, y, z0 - HOLE_PAST, top + 0.2, spec.m3_through)
        body = body - pocket - _y_slot(x, y, face, z0, z1, spec.m3_hex_af) - hole
    z0, z1 = _pocket_z(spec, QUARTER_BELOW_TOP, spec.quarter_hex_h)
    pocket = hex_pocket(spec.quarter_hex_af, spec.quarter_hex_h, at=(qx, qy, z0))
    # +Y end, away from the arm. The center is equidistant, so the side is a choice.
    slot = _y_slot(qx, qy, y_hi, z0, z1, spec.quarter_hex_af)
    hole = _z_cyl(qx, qy, z0 - HOLE_PAST, top + 0.2, spec.quarter_through)
    return body - pocket - slot - hole


def build(spec: Params) -> Part:
    part = _cut_top(_cut_plate(_blank(spec), spec), spec)
    part.label = "block"
    return part


def _circles_on_z(part: Part, radius: float, x: float, y: float) -> list:
    found = []
    for edge in part.edges():
        if edge.geom_type != GeomType.CIRCLE or abs(edge.radius - radius) > 0.08:
            continue
        center = edge.arc_center
        if abs(center.X - x) <= 0.2 and abs(center.Y - y) <= 0.2:
            found.append(center)
    return found


def _voids(part: Part, region: Part) -> list[Part]:
    leftover = region - part
    if leftover is None:
        return []
    return [solid for solid in leftover.solids() if solid.volume > 1.0]


def _min_gap(solids: list[Part]) -> float:
    gap = None
    for index, left in enumerate(solids):
        for right in solids[index + 1 :]:
            distance = left.distance(right)
            gap = distance if gap is None else min(gap, distance)
    return 0.0 if gap is None else gap


def _apart(part: Part, region: Part, expect: int, wall: float, name: str) -> Check:
    voids = _voids(part, region)
    if len(voids) < expect:
        return (name, False, f"{len(voids)} voids, wanted {expect}")
    gap = _min_gap(voids)
    return (name, gap >= wall, f"{len(voids)} voids, min gap {gap:.2f} mm")


def _trim_ring(ring: Part, slot: Part) -> Part:
    # The slot is an opening. A wall probe must not sit in it.
    opened = ring - slot
    return ring if opened is None else opened


def _plate_wall(part: Part, spec: Params, wall: float) -> list[Check]:
    y0 = boss_face_y(spec) + (PLATE_DEPTH - spec.m3_hex_h) / 2
    y1 = y0 + spec.m3_hex_h
    results = []
    for x, z in boss_hole_centers(spec):
        face = _nearest_z(spec, z)
        slot = box_from(
            x - spec.m3_hex_af / 2 - 0.3,
            y0 - 0.3,
            min(face, z) - 0.3,
            x + spec.m3_hex_af / 2 + 0.3,
            y1 + 0.3,
            max(face, z) + 0.3,
        )
        ring = _trim_ring(_hex_ring_y(spec.m3_hex_af, wall, x, z, y0, y1), slot)
        results.append(wall_encloses(part, ring, "plate nut wall"))
        probe = hex_from_y_face(spec.m3_hex_af - 0.4, spec.m3_hex_h - 0.4, x, y0 + 0.2, z, 1)
        results.append(void_open(part, probe, "plate nut open"))
        found = circles_on_yz_axis(part, spec.m3_through / 2, x, z)
        results.append(
            ("plate hole circle", len(found) >= 1, f"{len(found)} at x={x:.1f} z={z:.1f}")
        )
    return results


def _top_wall(part: Part, spec: Params, wall: float) -> list[Check]:
    results = []
    m3_xy, quarter = top_hole_xy(spec)
    y_lo = _arm_y1(spec)
    y_hi = _riser_y1(spec)
    for x, y in m3_xy:
        z0, z1 = _pocket_z(spec, M3_BELOW_TOP, spec.m3_hex_h)
        face = y_lo if abs(y - y_lo) <= abs(y - y_hi) else y_hi
        results.extend(
            _top_pocket(part, x, y, z0, z1, face, spec.m3_hex_af, spec.m3_through, wall, "top m3")
        )
    z0, z1 = _pocket_z(spec, QUARTER_BELOW_TOP, spec.quarter_hex_h)
    qx, qy = quarter
    results.extend(
        _top_pocket(
            part, qx, qy, z0, z1, y_hi, spec.quarter_hex_af, spec.quarter_through, wall, "quarter"
        )
    )
    return results


def _top_pocket(
    part: Part,
    x: float,
    y: float,
    z0: float,
    z1: float,
    y_face: float,
    af: float,
    through: float,
    wall: float,
    name: str,
) -> list[Check]:
    slot = _y_slot(x, y, y_face, z0 - 0.3, z1 + 0.3, af + 0.6)
    ring = hex_wall_ring(af, wall, z0 + 0.2, z1 - 0.2, x, y)
    found = _circles_on_z(part, through / 2, x, y)
    return [
        void_open(
            part, hex_pocket(af - 0.4, z1 - z0 - 0.4, at=(x, y, z0 + 0.2)), f"{name} pocket open"
        ),
        wall_encloses(part, _trim_ring(ring, slot), f"{name} pocket wall"),
        (f"{name} hole circle", len(found) >= 1, f"{len(found)} at x={x:.1f} y={y:.1f}"),
    ]


def _riser_region(spec: Params) -> Part:
    z0 = boss_z0(spec) + ARM_SECTION + 1.0
    # Stay inside the straight faces. The corner arcs are not walls between nuts.
    half = spec.block_size / 2 - RISER_CORNER_R - 0.5
    return box_from(
        -half, _arm_y1(spec) + 1.0, z0, half, _riser_y1(spec) - 0.4, riser_top(spec) - 0.4
    )


def _plate_region(spec: Params) -> Part:
    z0 = boss_z0(spec)
    half = spec.boss_w / 2 - 0.4
    return box_from(
        -half,
        boss_face_y(spec) + 0.4,
        z0 + 0.4,
        half,
        _plate_y1(spec) - 1.0,
        z0 + spec.boss_w - 0.4,
    )


def checks(part: Part, spec: Params) -> list[Check]:
    wall = MATERIALS["PETG"]["wall_min"]
    return [
        one_solid(part),
        *_plate_wall(part, spec, wall),
        *_top_wall(part, spec, wall),
        _apart(part, _riser_region(spec), 5, wall, "wall between pockets"),
        _apart(part, _plate_region(spec), 4, wall, "plate pockets stay apart"),
    ]


def _contact_area(part: Part, spec: Params) -> float:
    y = boss_face_y(spec)
    area = 0.0
    for face in part.faces():
        if face.geom_type != GeomType.PLANE:
            continue
        if abs(face.normal_at().Y + 1.0) > 0.05:
            continue
        if abs(face.center().Y - y) > 0.05:
            continue
        area += face.area
    return area


def checks_vs_band(part: Part, front: Part, back: Part, spec: Params) -> list[Check]:
    results = []
    for name, other in (("band_front", front), ("band_back", back)):
        volume = intersection_volume(part, other)
        results.append((f"no clash with {name}", volume <= VOL_EPS, f"{volume:.3e} mm3"))
    area = _contact_area(part, spec)
    results.append(("boss face is the intended contact", area > 800.0, f"{area:.0f} mm2"))
    radius = spec.m3_through / 2
    for x, z in boss_hole_centers(spec):
        on_band = circles_on_yz_axis(back, radius, x, z)
        on_block = circles_on_yz_axis(part, radius, x, z)
        if not on_band or not on_block:
            results.append(
                ("boss and plate holes coaxial", False, f"missing circle x={x:.1f} z={z:.1f}")
            )
            continue
        dx = abs(on_band[0].X - on_block[0].X)
        dz = abs(on_band[0].Z - on_block[0].Z)
        coaxial = dx <= 0.05 and dz <= 0.05
        results.append(("boss and plate holes coaxial", coaxial, f"dx={dx:.3f} dz={dz:.3f}"))
    return results
