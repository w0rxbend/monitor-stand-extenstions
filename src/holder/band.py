"""TPU split band around the column. The coupon uses the same clamp, shorter."""

from build123d import Part, Pos, Rectangle, extrude, fillet, offset

from holder.common import (
    VOL_EPS,
    Check,
    box_from,
    circles_on_yz_axis,
    hex_from_y_face,
    intersection_volume,
    one_solid,
    radial_ring_y,
    void_open,
    wall_encloses,
    y_cylinder,
)
from holder.params import MATERIALS, Params

NAME = "band"
MATERIAL = "TPU95A"
PRINT = {"band_front": (0, 0, -1), "band_back": (0, 0, -1)}
RIB_PITCH = 5.0
RIB_T = 1.0
RIB_BITE = 0.2


def column_profile(spec: Params):
    return fillet(Rectangle(spec.column_w, spec.column_d).vertices(), spec.column_r)


def column_solid(spec: Params, height: float) -> Part:
    return extrude(column_profile(spec), amount=height)


def seam_x(spec: Params, side: float) -> float:
    return side * (spec.column_w / 2 + spec.band_wall + spec.ear_len / 2)


def ear_outer_ys(spec: Params) -> tuple[float, float]:
    reach = spec.seam_gap / 2 + spec.ear_t
    return (-reach, reach)


def seam_axes(spec: Params, bolt_zs: tuple[float, ...]) -> list[tuple[float, float]]:
    return [(seam_x(spec, side), z) for side in (-1.0, 1.0) for z in bolt_zs]


def seam_hole(spec: Params, x: float, z: float, y0: float, y1: float) -> Part:
    return y_cylinder(x, z, y0, y1, spec.m3_through)


def _gap_cutter(spec: Params, height: float, side: float) -> Part:
    outer = side * (spec.column_w / 2 + spec.band_wall + 1)
    inner = side * (spec.column_w / 2 - 1)
    return box_from(outer, -spec.seam_gap / 2, -1, inner, spec.seam_gap / 2, height + 1)


def _ear(spec: Params, height: float, side: float, y_sign: float) -> Part:
    tip = side * (spec.column_w / 2 + spec.band_wall + spec.ear_len)
    root = side * (spec.column_w / 2)
    y_gap = y_sign * spec.seam_gap / 2
    y_out = y_sign * (spec.seam_gap / 2 + spec.ear_t)
    return box_from(tip, y_gap, 0, root, y_out, height)


def _as_part(solid) -> Part:
    # A bare solid wrapped as Part reports volume 0. The union builds a real compound.
    return Part() + solid


def clamp_halves(
    spec: Params,
    height: float,
    bolt_zs: tuple[float, ...],
) -> tuple[Part, Part]:
    """Front (-Y) and back (+Y) halves, seam gap on the left and right, one ear per seam."""
    inner = column_profile(spec)
    ring = extrude(offset(inner, amount=spec.band_wall) - inner, amount=height)
    opened = ring - _gap_cutter(spec, height, -1) - _gap_cutter(spec, height, 1)
    front_solid, back_solid = sorted(opened.solids(), key=lambda solid: solid.center().Y)
    front = _as_part(front_solid)
    back = _as_part(back_solid)
    y_front, y_back = ear_outer_ys(spec)
    for side in (-1.0, 1.0):
        front = front + _ear(spec, height, side, -1)
        back = back + _ear(spec, height, side, 1)
        for z in bolt_zs:
            x = seam_x(spec, side)
            hole = seam_hole(spec, x, z, y_front - 0.2, y_back + 0.2)
            recess = y_cylinder(x, z, y_front - 0.2, y_front + spec.m3_head_h, spec.m3_head_d)
            pocket = hex_from_y_face(spec.m3_hex_af, spec.m3_hex_h, x, y_back, z, -1)
            front = front - hole - recess
            back = back - hole - pocket
    return front, back


def boss_face_y(spec: Params) -> float:
    """Back face of the boss. The block plate sits on this plane."""
    return spec.column_d / 2 + spec.band_wall + spec.boss_depth


def boss_hole_centers(spec: Params) -> list[tuple[float, float]]:
    half = spec.m3_spacing / 2
    z_mid = spec.band_h / 2
    return [(sx * half, z_mid + sz * half) for sx in (-1.0, 1.0) for sz in (-1.0, 1.0)]


def _union(solids) -> Part:
    acc = _as_part(solids[0])
    for solid in solids[1:]:
        acc = acc + solid
    return acc


def _ribs(spec: Params, height: float) -> Part:
    inner = column_profile(spec)
    profile = offset(inner, RIB_BITE) - offset(inner, -1.0)
    slabs = []
    z = RIB_PITCH
    while z + RIB_T / 2 <= height:
        slabs.append(Pos(0, 0, z - RIB_T / 2) * extrude(profile, amount=RIB_T))
        z += RIB_PITCH
    return _union(slabs)


def _with_ribs(front: Part, back: Part, spec: Params) -> tuple[Part, Part]:
    opened = _ribs(spec, spec.band_h)
    opened = opened - _gap_cutter(spec, spec.band_h, -1) - _gap_cutter(spec, spec.band_h, 1)
    front_ribs = [solid for solid in opened.solids() if solid.center().Y < 0]
    back_ribs = [solid for solid in opened.solids() if solid.center().Y >= 0]
    if not front_ribs or not back_ribs:
        raise RuntimeError("grip ribs did not land on both halves")
    return front + _union(front_ribs), back + _union(back_ribs)


def _with_boss(back: Part, spec: Params) -> Part:
    z0 = (spec.band_h - spec.boss_w) / 2
    # 0.2 mm overlap with the band so the flat back fuses. The outer face stays on boss_face_y.
    boss = box_from(
        -spec.boss_w / 2,
        boss_face_y(spec) - spec.boss_depth - RIB_BITE,
        z0,
        spec.boss_w / 2,
        boss_face_y(spec),
        z0 + spec.boss_w,
    )
    back = back + boss
    y_inner = spec.column_d / 2
    for x, z in boss_hole_centers(spec):
        recess = y_cylinder(x, z, y_inner - 0.4, y_inner + spec.m3_head_h, spec.m3_head_d)
        hole = y_cylinder(x, z, y_inner - 0.4, boss_face_y(spec) + 0.2, spec.m3_through)
        back = back - recess - hole
    return back


def build_halves(spec: Params) -> tuple[Part, Part]:
    front, back = clamp_halves(spec, spec.band_h, spec.ear_bolt_z)
    front, back = _with_ribs(front, back, spec)
    back = _with_boss(back, spec)
    front.label = "band_front"
    back.label = "band_back"
    return front, back


def _rib_checks(part: Part, spec: Params) -> list[Check]:
    bite = intersection_volume(part, column_solid(spec, spec.band_h))
    inner = extrude(offset(column_profile(spec), -1.05), amount=spec.band_h)
    deep = intersection_volume(part, inner)
    return [
        ("grip ribs bite the column", bite > 20.0, f"{bite:.1f} mm3"),
        ("ribs are no deeper than 1 mm", deep <= 1e-3, f"{deep:.4f} mm3"),
    ]


def _seam_span(spec: Params, x: float, z: float, y0: float, y1: float, part: Part) -> Check:
    probe = y_cylinder(x, z, y0, y1, spec.m3_through - 0.3)
    return void_open(part, probe, "seam through-hole open")


def _front_seam_open(part: Part, spec: Params, x: float, z: float) -> Check:
    y_front, _y_back = ear_outer_ys(spec)
    y0 = y_front + spec.m3_head_h + 0.25
    y1 = -spec.seam_gap / 2 - 0.25
    return _seam_span(spec, x, z, y0, y1, part)


def _back_seam_open(part: Part, spec: Params, x: float, z: float) -> Check:
    _y_front, y_back = ear_outer_ys(spec)
    y0 = spec.seam_gap / 2 + 0.25
    y1 = y_back - spec.m3_hex_h - 0.25
    return _seam_span(spec, x, z, y0, y1, part)


def _circle_checks(part: Part, radius: float, axes, name: str) -> list[Check]:
    checks = []
    for x, z in axes:
        found = circles_on_yz_axis(part, radius, x, z)
        checks.append((name, len(found) >= 1, f"{len(found)} circle(s) at x={x:.2f} z={z:.2f}"))
    return checks


def checks_front(part: Part, spec: Params) -> list[Check]:
    y_front, _y_back = ear_outer_ys(spec)
    wall = MATERIALS["TPU95A"]["wall_min"]
    axes = seam_axes(spec, spec.ear_bolt_z)
    results = [
        one_solid(part),
        *_circle_checks(part, spec.m3_through / 2, axes, "seam through-hole circle"),
        *_circle_checks(part, spec.m3_head_d / 2, axes, "head recess circle"),
        *_rib_checks(part, spec),
    ]
    for x, z in axes:
        results.append(_front_seam_open(part, spec, x, z))
        ring = radial_ring_y(
            x,
            z,
            y_front + 0.4,
            y_front + spec.m3_head_h - 0.3,
            spec.m3_head_d / 2,
            wall,
        )
        results.append(wall_encloses(part, ring, "head recess wall"))
    return results


def checks_back(part: Part, spec: Params) -> list[Check]:
    _y_front, y_back = ear_outer_ys(spec)
    wall = MATERIALS["TPU95A"]["wall_min"]
    axes = seam_axes(spec, spec.ear_bolt_z)
    results = [
        one_solid(part),
        *_circle_checks(part, spec.m3_through / 2, axes, "seam through-hole circle"),
        *_rib_checks(part, spec),
    ]
    for x, z in axes:
        results.append(_back_seam_open(part, spec, x, z))
        pocket = hex_from_y_face(spec.m3_hex_af - 0.3, spec.m3_hex_h - 0.5, x, y_back - 0.15, z, -1)
        results.append(void_open(part, pocket, "seam nut pocket open"))
    for x, z in boss_hole_centers(spec):
        found = circles_on_yz_axis(part, spec.m3_through / 2, x, z)
        detail = f"{len(found)} at x={x:.2f} z={z:.2f}"
        results.append(("boss hole circle", len(found) >= 1, detail))
        probe = y_cylinder(
            x,
            z,
            spec.column_d / 2 + spec.band_wall + 1.0,
            boss_face_y(spec) - 1.0,
            spec.m3_through - 0.3,
        )
        results.append(void_open(part, probe, "boss hole open"))
        ring = radial_ring_y(
            x,
            z,
            spec.column_d / 2 + spec.band_wall + 1.0,
            boss_face_y(spec) - 1.0,
            spec.m3_through / 2,
            wall,
        )
        results.append(wall_encloses(part, ring, "boss hole wall"))
        recess = y_cylinder(
            x,
            z,
            spec.column_d / 2 + 0.3,
            spec.column_d / 2 + spec.m3_head_h - 0.2,
            spec.m3_head_d - 0.4,
        )
        results.append(void_open(part, recess, "boss head recess open"))
    return results


def checks_pair(front: Part, back: Part, spec: Params) -> list[Check]:
    volume = intersection_volume(front, back)
    gap = front.distance(back)
    results: list[Check] = [
        ("seam halves do not intersect", volume <= VOL_EPS, f"{volume:.3e} mm3"),
        ("seam gap left between the ears", gap >= spec.seam_gap - 0.05, f"{gap:.3f} mm"),
    ]
    radius = spec.m3_through / 2
    for x, z in seam_axes(spec, spec.ear_bolt_z):
        front_edges = circles_on_yz_axis(front, radius, x, z)
        back_edges = circles_on_yz_axis(back, radius, x, z)
        if not front_edges or not back_edges:
            results.append(("seam bolts coaxial", False, f"missing circle at x={x:.2f} z={z:.2f}"))
            continue
        dx = abs(front_edges[0].X - back_edges[0].X)
        dz = abs(front_edges[0].Z - back_edges[0].Z)
        coaxial = dx <= 0.05 and dz <= 0.05
        results.append(("seam bolts coaxial", coaxial, f"dx={dx:.3f} dz={dz:.3f}"))
    return results
