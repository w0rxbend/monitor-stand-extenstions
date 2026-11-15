"""Shared geometry tools. hex_pocket is copied from Tigerbee 7inch accessories/_common.py."""

from math import cos, radians

from build123d import (
    Axis,
    Box,
    Cylinder,
    GeomType,
    Location,
    Part,
    Pos,
    RegularPolygon,
    Rot,
    Vector,
    extrude,
    offset,
)

Check = tuple[str, bool, str]
VOL_EPS = 1e-6
AXIS_TOL = 0.2


def print_orientation(part: Part, bed_normal: tuple[float, float, float] = (0, 0, -1)) -> Part:
    """Copy rotated so `bed_normal` points down, then centred in XY with the bed face on Z = 0.

    Copied from Tigerbee 7inch accessories/_common.py. The installed part is not modified.
    """
    normal = Vector(*bed_normal).normalized()
    down = Vector(0, 0, -1)
    axis_dir = normal.cross(down)
    if axis_dir.length > 1e-9:
        axis = Axis((0, 0, 0), axis_dir)
        turned = part.rotate(axis, normal.get_signed_angle(down, axis_dir))
    elif normal.Z > 0:
        turned = part.rotate(Axis.X, 180)
    else:
        turned = part.moved(Location())
    bounds = turned.bounding_box()
    moved = turned.moved(Location((-bounds.center().X, -bounds.center().Y, -bounds.min.Z)))
    moved.label = part.label
    return moved


def hex_pocket(
    af: float,
    depth: float,
    at: tuple[float, float, float] = (0, 0, 0),
    rotation_deg: float = 0.0,
) -> Part:
    """Hexagonal prism tool (across-flats `af`) from Z at[2] up to at[2] + depth; subtract it."""
    x, y, z = at
    return Pos(x, y, z) * extrude(
        RegularPolygon(af / 2 / cos(radians(30)), 6).rotate(Axis.Z, rotation_deg),
        amount=depth,
    )


def box_from(x0: float, y0: float, z0: float, x1: float, y1: float, z1: float) -> Part:
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(
        abs(x1 - x0), abs(y1 - y0), abs(z1 - z0)
    )


def y_cylinder(x: float, z: float, y0: float, y1: float, diameter: float) -> Part:
    """Cylinder along Y from y0 to y1. Rot(-90, 0, 0) maps build123d's +Z axis onto +Y."""
    rod = Rot(-90, 0, 0) * Cylinder(diameter / 2, abs(y1 - y0))
    return Pos(x, (y0 + y1) / 2, z) * rod


def hex_from_y_face(
    af: float,
    depth: float,
    x: float,
    y_face: float,
    z: float,
    direction: float,
) -> Part:
    """Hex prism starting on a Y face. direction +1 grows toward +Y, -1 toward -Y."""
    tool = hex_pocket(af, depth)
    spin = Rot(-90, 0, 0) if direction > 0 else Rot(90, 0, 0)
    return Pos(x, y_face, z) * (spin * tool)


def hex_wall_ring(af: float, wall: float, z0: float, z1: float, x: float, y: float) -> Part:
    """Solid ring of radial thickness `wall` around a Z hex, from z0 to z1."""
    profile = RegularPolygon(af / 2 / cos(radians(30)), 6)
    ring = extrude(offset(profile, amount=wall) - profile, amount=z1 - z0)
    return Pos(x, y, z0) * ring


def intersection_volume(left: Part, right: Part) -> float:
    hit = left & right
    if hit is None:
        return 0.0
    return float(hit.volume)


def outside_volume(probe: Part, part: Part) -> float:
    leftover = probe - part
    if leftover is None:
        return 0.0
    return float(leftover.volume)


def one_solid(part: Part) -> Check:
    count = len(part.solids())
    ok = count == 1 and part.is_valid and part.volume > 0
    detail = f"{count} solid, {part.volume:.1f} mm3, valid={part.is_valid}"
    return ("one solid", ok, detail)


def circles_on_yz_axis(part: Part, radius: float, x: float, z: float) -> list:
    found = []
    for edge in part.edges():
        if edge.geom_type != GeomType.CIRCLE or abs(edge.radius - radius) > 0.08:
            continue
        center = edge.arc_center
        if abs(center.X - x) <= AXIS_TOL and abs(center.Z - z) <= AXIS_TOL:
            found.append(center)
    return found


def void_open(part: Part, tool: Part, name: str) -> Check:
    volume = intersection_volume(part, tool)
    return (name, volume <= 1e-3, f"{volume:.4f} mm3 of material in the void")


def wall_encloses(part: Part, probe: Part, name: str) -> Check:
    """Probe is the material shell that must stay inside the part (solid, not a bounding box)."""
    volume = outside_volume(probe, part)
    return (name, volume <= 1e-3, f"{volume:.4f} mm3 of the wall probe is outside")


def radial_ring_y(
    x: float,
    z: float,
    y0: float,
    y1: float,
    radius: float,
    wall: float,
) -> Part:
    span = abs(y1 - y0)
    mid = (y0 + y1) / 2
    outer = Pos(x, mid, z) * (Rot(-90, 0, 0) * Cylinder(radius + wall, span))
    inner = Pos(x, mid, z) * (Rot(-90, 0, 0) * Cylinder(radius, span + 1))
    return outer - inner
