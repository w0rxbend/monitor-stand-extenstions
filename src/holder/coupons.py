"""Print coupons cut from the real parts, so their fits are the real fits by construction.

The ring coupons are checked against the measured column, not against the collar they were
cut from: the shell bore must sit bore_offset off the column and the liner rib tips at
the TPU fit. They carry the z 10 seam stops, so they also test an M5 insert, the springs
and the shoulder stop on the real column. The rosette coupons are J1: the pedestal's
toothed flat with its shoulder bore, and the link's end A with its teeth and M6 insert.
"""

from build123d import Part, Vector

from holder import arm
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

SLAB_Z = (4.0, 19.5)  # the lower ring with the lower seam stops, above the lip
BASE_DISC = (0.0, 7.5)  # x range of the pedestal slice: the flat and its teeth
PRINT = {
    "coupon_collar_front": (0, 0, 1),
    "coupon_collar_back": (0, 0, 1),
    "coupon_liner_front": (0, 0, -1),
    "coupon_liner_back": (0, 0, -1),
    "coupon_rosette_base": (-1, 0, 0),
    "coupon_rosette_link": arm.PRINT["arm_link"],
}
MATERIAL = {name: ("TPU95A" if "liner" in name else "PETG") for name in PRINT}


def build(spec: Params, parts: dict[str, Part]) -> dict[str, Part]:
    low, high = SLAB_Z
    slab = box_from(-100, -100, low, 100, spec.bore_half_d + 12, high)
    found = {f"coupon_{name}": named(parts[name] & slab, f"coupon_{name}")
             for name in ("collar_front", "collar_back", "liner_front", "liner_back")}
    j1 = arm.j1_local(spec)
    x0, x1 = BASE_DISC
    disc = box_from(x0, -spec.ball_r - 1, j1.Z - spec.ball_r - 1, x1, spec.ball_r + 1,
                    j1.Z + spec.ball_r + 1)
    found["coupon_rosette_base"] = named(parts["arm_base"] & disc, "coupon_rosette_base")
    end = rod(2 * spec.knuckle_r + 1, arm.link_t(spec) + 3, Vector(-1, 0, 0), Vector(1, 0, 0))
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
    return rows
