"""Print coupons cut from the real parts, so their fits are the real fits by construction."""

from build123d import Part, Vector

from holder import arm, collar
from holder.common import (
    Check,
    box_from,
    first_hit,
    named,
    one_solid,
    overhang_rows,
    print_orientation,
    rod,
)
from holder.params import Params

SLAB_Z = (4.0, 16.0)  # holds the z 10 seam screw row
MF_BOX_X = (-7.1, 16.0)  # one M3 channel (x +10) and the 1/4" channel; the x -10 one is left out
PRINT = {
    "coupon_collar_front": collar.PRINT["collar_front"],
    "coupon_collar_back": collar.PRINT["collar_back"],
    "coupon_liner_front": collar.PRINT["liner_front"],
    "coupon_liner_back": collar.PRINT["liner_back"],
    "coupon_mf": collar.PRINT["collar_back"],
    "coupon_rosette_base": (-1, 0, 0),
    "coupon_rosette_link": arm.PRINT["arm_link"],
}
MATERIAL = {name: ("TPU95A" if "liner" in name else "PETG") for name in PRINT}


def _ring_slab(spec: Params) -> Part:
    low, high = SLAB_Z
    return box_from(-100, -100, low, 100, spec.outer_half_d + 1, high)


def build(spec: Params, parts: dict[str, Part]) -> dict[str, Part]:
    slab = _ring_slab(spec)
    found = {f"coupon_{name}": named(parts[name] & slab, f"coupon_{name}")
             for name in ("collar_front", "collar_back", "liner_front", "liner_back")}
    x0, x1 = MF_BOX_X
    mf_box = box_from(x0, collar.bore_wall_y(spec, 0) - 3, 14, x1, collar.pad_end_y(spec) + 1,
                      spec.collar_h + 1)
    found["coupon_mf"] = named(parts["collar_back"] & mf_box, "coupon_mf")
    centre = arm.j1_centre(spec)
    disc = rod(2 * spec.knuckle_r + 1, spec.knuckle_t + 3,
               Vector(spec.arm_x0 - spec.knuckle_t - 1, centre.Y, centre.Z), Vector(1, 0, 0))
    found["coupon_rosette_base"] = named(parts["arm_base"] & disc, "coupon_rosette_base")
    end = rod(2 * spec.knuckle_r + 1, spec.knuckle_t + 3, Vector(-1, 0, 0), Vector(1, 0, 0))
    found["coupon_rosette_link"] = named(parts["arm_link"] & end, "coupon_rosette_link")
    return found


def checks(part: Part, spec: Params) -> list[Check]:
    rows = [one_solid(part), *overhang_rows(print_orientation(part, PRINT[part.label]),
                                            part.label)]
    if part.label.startswith("coupon_collar"):
        inside_y = -10.0 if part.label.endswith("front") else 10.0
        inner = first_hit(part, Vector(0, inside_y, 10), Vector(1, 0, 0))
        rows.append(Check(f"{part.label} bore is the collar bore",
                          abs(inner - spec.bore_half_w) < 1e-3,
                          f"{inner:.3f} mm (collar {spec.bore_half_w:.3f})"))
    if part.label == "coupon_mf":
        for nut in collar.mount_face(spec).nuts(spec):
            if nut.seat.X > MF_BOX_X[0] + nut.nut.pocket_af / 2:
                rows += nut.checks(part)
    return rows
