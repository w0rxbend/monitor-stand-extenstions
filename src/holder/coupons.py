"""Print coupons cut from the real parts, so their fits are the real fits by construction.

The ring coupons are checked against the measured column, not against the collar they were
cut from: the shell bore must sit bore_offset off the column and the liner rib tips at
the TPU fit.
"""

from build123d import Part, Vector

from holder import arm, collar
from holder.common import (
    Check,
    box_from,
    column_solid,
    named,
    one_solid,
    overhang_rows,
    print_orientation,
    rod,
)
from holder.params import Params

SLAB_Z = (4.0, 15.0)  # the lower ring with the z 10 seam screws, under the deck at 16
MF_BOX_X = (-7.1, 16.0)  # the x +10 inserts and the 1/4" channel; the x -10 ones are left out
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
    # Start past the back-face roll, so the coupon prints on a flat cut face.
    start = spec.arm_x0 - spec.knuckle_t + arm.FACE_ROLL.setback
    disc = rod(2 * spec.knuckle_r + 1, spec.knuckle_t + 3, Vector(start, centre.Y, centre.Z),
               Vector(1, 0, 0))
    found["coupon_rosette_base"] = named(parts["arm_base"] & disc, "coupon_rosette_base")
    end = rod(2 * spec.knuckle_r + 1, spec.knuckle_t + 3, Vector(-1, 0, 0), Vector(1, 0, 0))
    found["coupon_rosette_link"] = named(parts["arm_link"] & end, "coupon_rosette_link")
    return found


def _column(spec: Params) -> Part:
    return column_solid(spec.column_w, spec.column_d, spec.column_r, -10, 60)


def checks(part: Part, spec: Params) -> list[Check]:
    rows = [one_solid(part), *overhang_rows(print_orientation(part, PRINT[part.label]),
                                            part.label)]
    if part.label.startswith("coupon_collar"):
        gap = part.distance_to(_column(spec))
        rows.append(Check(f"{part.label} bore off the measured column",
                          abs(gap - spec.bore_offset) < 0.01,
                          f"{gap:.3f} mm (bore offset {spec.bore_offset:.3f} +- 0.01)"))
    if part.label.startswith("coupon_liner"):
        gap = part.distance_to(_column(spec))
        rows.append(Check(f"{part.label} rib tips off the measured column",
                          abs(gap - spec.tpu_fit) <= 0.02,
                          f"{gap:.3f} mm (fit {spec.tpu_fit} +- 0.02)"))
    if part.label == "coupon_mf":
        face = collar.mount_face(spec)
        rows += face.port_nut().checks(part)
        for insert in face.inserts(spec):
            if insert.face.X > MF_BOX_X[0] + 4:
                rows += insert.checks(part)
    return rows
