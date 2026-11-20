"""Every printed part: material, print pose (bed-face normal, use frame), and its checks.

Part modules build geometry and return checks. They never export. export.py never repairs.
"""

from collections.abc import Callable

from build123d import Part

from holder import arm, collar, coupons
from holder.assembly import printed_parts
from holder.common import Check
from holder.params import Params

CHECKS: dict[str, Callable[[Part, Params], list[Check]]] = {
    "collar_front": collar.checks_front,
    "collar_back": collar.checks_back,
    "liner_front": collar.checks_liner,
    "liner_back": collar.checks_liner,
    **{f"seam_cap_{k}": collar.checks_cap for k in range(1, 5)},
    "arm_base": arm.checks_base,
    "arm_link": arm.checks_link,
    "arm_tip": arm.checks_tip,
    "arm_head": arm.checks_head,
    "knob": arm.checks_knob,
    "j3_ring": arm.checks_ring,
    **dict.fromkeys(coupons.PRINT, coupons.checks),
}
PRINT = {**collar.PRINT, **arm.PRINT, **coupons.PRINT}
MATERIAL = {**collar.MATERIAL, **arm.MATERIAL, **coupons.MATERIAL}


def all_parts(spec: Params) -> dict[str, Part]:
    parts = dict(printed_parts(spec))
    parts.update(coupons.build(spec, parts))
    missing = set(parts) ^ set(CHECKS)
    if missing:
        raise ValueError(f"parts and checks disagree: {sorted(missing)}")
    return parts
