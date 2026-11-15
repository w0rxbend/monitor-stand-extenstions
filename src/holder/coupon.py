"""12 mm band slice plus a PETG strip that proves the nut pockets."""

from build123d import Box, Cylinder, GeomType, Part, Pos

from holder.band import clamp_halves, column_solid, ear_outer_ys, seam_axes
from holder.common import (
    VOL_EPS,
    Check,
    circles_on_yz_axis,
    hex_from_y_face,
    hex_pocket,
    hex_wall_ring,
    intersection_volume,
    one_solid,
    radial_ring_y,
    void_open,
    wall_encloses,
    y_cylinder,
)
from holder.params import MATERIALS, Params

COUPON_H = 12.0
COUPON_BOLT_Z = 6.0
STRIP_LENGTH = 70.0
STRIP_WIDTH = 18.0
STRIP_THICK = 8.0
M3_STRIP_X = 16.0
QUARTER_STRIP_X = 52.0

NAME = "coupon"
MATERIAL = {
    "coupon_front": "TPU95A",
    "coupon_back": "TPU95A",
    "coupon_strip": "PETG",
}
PRINT = {
    "coupon_front": (0, 0, -1),
    "coupon_back": (0, 0, -1),
    "coupon_strip": (0, 0, -1),
}


def _strip_hex(af: float, depth: float, x: float) -> Part:
    return hex_pocket(af, depth, at=(x, 0.0, STRIP_THICK - depth))


def m3_hex_tool(spec: Params, af: float | None = None) -> Part:
    return _strip_hex(spec.m3_hex_af if af is None else af, spec.m3_hex_h, M3_STRIP_X)


def quarter_hex_tool(spec: Params, af: float | None = None) -> Part:
    width = spec.quarter_hex_af if af is None else af
    return _strip_hex(width, spec.quarter_hex_h, QUARTER_STRIP_X)


def build_front(spec: Params) -> Part:
    front, _back = clamp_halves(spec, COUPON_H, (COUPON_BOLT_Z,))
    front.label = "coupon_front"
    return front


def build_back(spec: Params) -> Part:
    _front, back = clamp_halves(spec, COUPON_H, (COUPON_BOLT_Z,))
    back.label = "coupon_back"
    return back


def build_strip(spec: Params) -> Part:
    strip = Pos(STRIP_LENGTH / 2, 0, STRIP_THICK / 2) * Box(STRIP_LENGTH, STRIP_WIDTH, STRIP_THICK)
    cuts = (
        (M3_STRIP_X, spec.m3_through, m3_hex_tool(spec)),
        (QUARTER_STRIP_X, spec.quarter_through, quarter_hex_tool(spec)),
    )
    for x, diameter, pocket in cuts:
        hole = Pos(x, 0, STRIP_THICK / 2) * Cylinder(diameter / 2, STRIP_THICK + 0.4)
        strip = strip - hole - pocket
    strip.label = "coupon_strip"
    return strip


def _bore_clear(part: Part, spec: Params) -> Check:
    volume = intersection_volume(part, column_solid(spec, COUPON_H))
    return ("bore does not cut into the column", volume <= 1e-3, f"{volume:.4f} mm3")


def _seam_circles(part: Part, spec: Params, radius: float, name: str) -> list[Check]:
    checks = []
    for x, z in seam_axes(spec, (COUPON_BOLT_Z,)):
        found = circles_on_yz_axis(part, radius, x, z)
        detail = f"{len(found)} circle(s) at x={x:.2f} z={z:.2f}"
        checks.append((name, len(found) >= 1, detail))
    return checks


def _ear_wall(part: Part, spec: Params) -> list[Check]:
    y_front, _y_back = ear_outer_ys(spec)
    radius = spec.m3_head_d / 2
    y0 = y_front + 0.4
    y1 = y_front + spec.m3_head_h - 0.3
    wall = MATERIALS["TPU95A"]["wall_min"]
    checks = []
    for x, z in seam_axes(spec, (COUPON_BOLT_Z,)):
        ring = radial_ring_y(x, z, y0, y1, radius, wall)
        checks.append(wall_encloses(part, ring, "head recess wall"))
    return checks


def _front_hole_probe(spec: Params, x: float, z: float) -> Part:
    """Smaller than the bolt, in the ear, clear of the head recess and the seam face."""
    y_front, _y_back = ear_outer_ys(spec)
    y0 = y_front + spec.m3_head_h + 0.25
    y1 = -spec.seam_gap / 2 - 0.25
    return y_cylinder(x, z, y0, y1, spec.m3_through - 0.3)


def checks_front(part: Part, spec: Params) -> list[Check]:
    results = [
        one_solid(part),
        *_seam_circles(part, spec, spec.m3_through / 2, "seam through-hole circle"),
        *_seam_circles(part, spec, spec.m3_head_d / 2, "head recess circle"),
        *_ear_wall(part, spec),
        _bore_clear(part, spec),
    ]
    for x, z in seam_axes(spec, (COUPON_BOLT_Z,)):
        results.append(void_open(part, _front_hole_probe(spec, x, z), "seam through-hole open"))
    return results


def _inset_seam_pocket(spec: Params, x: float, z: float) -> Part:
    _y_front, y_back = ear_outer_ys(spec)
    return hex_from_y_face(
        spec.m3_hex_af - 0.3,
        spec.m3_hex_h - 0.5,
        x,
        y_back - 0.15,
        z,
        -1,
    )


def checks_back(part: Part, spec: Params) -> list[Check]:
    results = [
        one_solid(part),
        *_seam_circles(part, spec, spec.m3_through / 2, "seam through-hole circle"),
        _bore_clear(part, spec),
    ]
    for x, z in seam_axes(spec, (COUPON_BOLT_Z,)):
        results.append(void_open(part, _inset_seam_pocket(spec, x, z), "seam nut pocket open"))
    return results


def _floor_probe(af: float, through: float, x: float, z_floor: float, wall_min: float) -> Part:
    radial = (af / 2 + through / 2) / 2
    return Pos(x + radial, 0, z_floor - wall_min + 0.2) * Box(0.5, 0.5, 0.3)


def _z_circles(part: Part, radius: float, x: float) -> list[float]:
    found = []
    for edge in part.edges():
        if edge.geom_type != GeomType.CIRCLE or abs(edge.radius - radius) > 0.08:
            continue
        center = edge.arc_center
        if abs(center.X - x) <= 0.2 and abs(center.Y) <= 0.2:
            found.append(center.Z)
    return found


def checks_strip(part: Part, spec: Params) -> list[Check]:
    wall = MATERIALS["PETG"]["wall_min"]
    results: list[Check] = [one_solid(part)]
    pockets = (
        ("m3", M3_STRIP_X, spec.m3_hex_af, spec.m3_hex_h, spec.m3_through),
        ("quarter", QUARTER_STRIP_X, spec.quarter_hex_af, spec.quarter_hex_h, spec.quarter_through),
    )
    for name, x, af, depth, through in pockets:
        z_floor = STRIP_THICK - depth
        probe = hex_pocket(af - 0.4, depth - 0.4, at=(x, 0.0, z_floor + 0.2))
        results.append(void_open(part, probe, f"{name} pocket open"))
        ring = hex_wall_ring(af, wall, z_floor + 0.3, STRIP_THICK - 0.3, x, 0.0)
        results.append(wall_encloses(part, ring, f"{name} pocket wall"))
        floor = _floor_probe(af, through, x, z_floor, wall)
        results.append(wall_encloses(part, floor, f"{name} floor wall"))
        ends = _z_circles(part, through / 2, x)
        results.append((f"{name} through-hole", len(ends) >= 2, f"z edges {ends}"))
    return results


def checks_pair(front: Part, back: Part, spec: Params) -> list[Check]:
    volume = intersection_volume(front, back)
    gap = front.distance(back)
    results: list[Check] = [
        ("seam halves do not intersect", volume <= VOL_EPS, f"{volume:.3e} mm3"),
        ("seam gap left between the ears", gap >= spec.seam_gap - 0.05, f"{gap:.3f} mm"),
    ]
    radius = spec.m3_through / 2
    for x, z in seam_axes(spec, (COUPON_BOLT_Z,)):
        front_edges = circles_on_yz_axis(front, radius, x, z)
        back_edges = circles_on_yz_axis(back, radius, x, z)
        if not front_edges or not back_edges:
            results.append(("seam bolts coaxial", False, f"missing circle at x={x:.2f}"))
            continue
        dx = abs(front_edges[0].X - back_edges[0].X)
        dz = abs(front_edges[0].Z - back_edges[0].Z)
        coaxial = dx <= 0.05 and dz <= 0.05
        results.append(("seam bolts coaxial", coaxial, f"dx={dx:.3f} dz={dz:.3f}"))
    return results
