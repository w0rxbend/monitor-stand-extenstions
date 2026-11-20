"""Design screw lengths: the stock length each joint is built around.

These pick a length from the grip and the thread need. They do not check the tip: the
tip rooms are fixed design parameters (seam_tip_room, knob_tip_room, the insert hole,
the camera socket), and the assembly measures every stack again by rays on the placed
solids. A screw that grows past its room fails there.
"""

from math import atan, degrees, radians, sin, tan

from holder.common import thread_need
from holder.params import (
    M6_CARRIAGE,
    M6_LARGE_WASHER,
    M6_NUT,
    QUARTER_SHCS,
    QUARTER_WASHER_SLOT,
    ROSETTE_MU,
    ZVE10_THREAD_MAX,
    ZVE10_THREAD_MIN,
    Params,
)

PLATE_UNDER_HEAD = 6.0  # camera plate PETG under the 1/4" washer: puts 1/2" inside the socket


def pitch_gap(spec: Params) -> float:
    """Axial gap between engaged rosettes that leaves flank_fit normal to a 45 degree flank."""
    return spec.flank_fit / sin(radians(45))


def joint_grip_closed(spec: Params) -> float:
    """Dome head on one knuckle face, both knuckles, the washer under the knob."""
    return 2 * spec.knuckle_t + pitch_gap(spec) + M6_LARGE_WASHER.t


def joint_screw_length(spec: Params) -> float:
    need = thread_need(M6_CARRIAGE, M6_NUT)
    grip_open = joint_grip_closed(spec) + spec.index_lift
    return M6_CARRIAGE.stock_at_least(grip_open + need + M6_CARRIAGE.length_tol)


def camera_grip() -> float:
    return PLATE_UNDER_HEAD + M6_LARGE_WASHER.t


def camera_screw_length() -> float:
    """Shortest stock that still engages ZVE10_THREAD_MIN on the short end of the tolerance."""
    return QUARTER_SHCS.stock_at_least(camera_grip() + ZVE10_THREAD_MIN + QUARTER_SHCS.length_tol)


def camera_tip_limit() -> float:
    return ZVE10_THREAD_MAX


def camera_plate_t() -> float:
    return PLATE_UNDER_HEAD + QUARTER_WASHER_SLOT[1]


def rosette_preload_need(spec: Params, moment_nmm: float) -> float:
    """Axial force that keeps the 45 degree flanks from riding up under `moment_nmm`.

    Tangential force at the mean radius F_t = M / r_m. With friction angle phi the teeth
    stay seated while F_axial >= F_t * tan(45 deg - phi).
    """
    mean = (spec.rosette_r_in + spec.rosette_r_out) / 2
    phi = degrees(atan(ROSETTE_MU))
    return moment_nmm / mean * tan(radians(45 - phi))
