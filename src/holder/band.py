"""TPU split band around the column. The coupon uses the same clamp, shorter."""

from build123d import Part, Rectangle, extrude, fillet, offset

from holder.common import box_from, hex_from_y_face, y_cylinder
from holder.params import Params


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
