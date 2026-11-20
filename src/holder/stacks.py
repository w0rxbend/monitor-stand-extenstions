"""Design screw stacks: the lengths the geometry is built around.

The assembly measures the same stacks again on the placed solids (rays on real faces).
If the model drifts from these numbers, the measured rows fail.
"""

from math import radians, sin

from holder.common import Stack, thread_need
from holder.params import (
    M3_NUT,
    M3_SHCS,
    M3_WASHER,
    M6_HEAD_POCKET,
    M6_HEX_BOLT,
    M6_NUT,
    M6_WASHER,
    M6_WASHER_RECESS,
    QUARTER_HEAD_BORE,
    QUARTER_SHCS,
    ZVE10_THREAD_MAX,
    ZVE10_THREAD_MIN,
    Params,
)

PLATE_UNDER_HEAD = 5.0  # camera plate PETG under the 1/4" head


def pitch_gap(spec: Params) -> float:
    """Axial gap between engaged rosettes that leaves flank_fit normal to a 45 degree flank."""
    return spec.flank_fit / sin(radians(45))


def seam_design(spec: Params) -> Stack:
    front = -spec.seam_gap / 2 - spec.seam_seat_y
    grip_open = M3_WASHER.t + front + spec.seam_gap + spec.seam_back_skin
    grip_closed = grip_open - spec.seam_gap
    need = thread_need(M3_SHCS, M3_NUT)
    long_tip = M3_SHCS.stock_at_least(grip_open + need + M3_SHCS.length_tol)
    tip = long_tip + M3_SHCS.length_tol - grip_closed
    return Stack("collar seam", M3_SHCS, grip_open, grip_closed, need, tip + 0.5)


def joint_design(spec: Params) -> Stack:
    head_side = spec.knuckle_t - M6_HEAD_POCKET[1]
    knob_side = spec.knuckle_t - M6_WASHER_RECESS[1]
    grip_closed = head_side + pitch_gap(spec) + knob_side + M6_WASHER.t
    grip_open = grip_closed + spec.index_lift
    need = thread_need(M6_HEX_BOLT, M6_NUT)
    length = M6_HEX_BOLT.stock_at_least(grip_open + need + M6_HEX_BOLT.length_tol)
    tip = length + M6_HEX_BOLT.length_tol - grip_closed
    return Stack("arm joint", M6_HEX_BOLT, grip_open, grip_closed, need, tip + 0.5)


def camera_design() -> Stack:
    grip = PLATE_UNDER_HEAD
    return Stack("camera screw", QUARTER_SHCS, grip, grip, ZVE10_THREAD_MIN, ZVE10_THREAD_MAX)


def camera_plate_t() -> float:
    return PLATE_UNDER_HEAD + QUARTER_HEAD_BORE[1]


def knob_bore_depth(spec: Params) -> float:
    """Blind bore in the knob, from its base: long bolt tip, joint closed, plus 1.0."""
    stack = joint_design(spec)
    return stack.length + stack.screw.length_tol - stack.grip_closed + 1.0


