"""Test coupon (Task 2): a band slice and a PETG nut strip, printed before the real parts.

Three parts, each with its own material and print orientation:

    coupon_band_front   TPU   coupon_h tall slice of the front band half, one M3 per seam
    coupon_band_back    TPU   the same slice of the back half, with the seam nut pockets
    coupon_strip        PETG  one M3 and one 1/4"-20 captive nut in side-slot pockets

The band slice comes from band.py, the same code the full band uses (only the height and
the bolt heights differ). The strip uses common.captive_nut, the pocket form of the block's
top face: the hole enters the top face, the nut sits top_m3_nut_depth (M3) or
top_quarter_nut_depth (1/4") below it and slides in from a side face.

The strip sits off to the side (+X) of the band in assembly coordinates; it has no mate.
"""

from build123d import Align, Axis, Box, Part, Pos

from holder import band
from holder.common import (
    Check,
    PartSpec,
    at_least,
    captive_nut,
    cavity_check,
    cavity_pieces,
    coaxial_check,
    count_is,
    cut,
    hex_pocket_checks,
    hole_axes,
    single_solid,
    wall_around,
)
from holder.params import MATERIALS, PETG, Params

ASSEMBLY = "coupon"
FRONT_NAME = "coupon_band_front"
BACK_NAME = "coupon_band_back"
STRIP_NAME = "coupon_strip"


# --- Band slice ----------------------------------------------------------------------------------


def bolt_zs(p: Params) -> tuple[float]:
    return (p.coupon_h / 2,)  # one seam bolt per seam, at mid-height


def build_front(p: Params) -> Part:
    return band.build_half(p, band.FRONT, p.coupon_h, bolt_zs(p))


def build_back(p: Params) -> Part:
    return band.build_half(p, band.BACK, p.coupon_h, bolt_zs(p))


def checks_front(part: Part, p: Params) -> list[Check]:
    return band.half_checks(part, p, band.FRONT, p.coupon_h, bolt_zs(p))


def checks_back(part: Part, p: Params) -> list[Check]:
    return band.half_checks(part, p, band.BACK, p.coupon_h, bolt_zs(p))


# --- PETG strip ----------------------------------------------------------------------------------
# A bar 31 x 16 x 24 mm (X x Y x Z). Its top face (+Z) stands in for the block's top face:
# both holes run down along -Z. Both nut slots open on the +Y face. Printed on the -Y face,
# the slots point up, so no pocket has a roof; the holes lie horizontal (PLAN.md "Print":
# they print about 0.15 mm small, which the through-hole sizes cover).

STRIP_M3_X = 52.0  # M3 axis; the band ears end at x = 34.35
STRIP_PITCH = 15.0  # M3 axis to 1/4" axis, along +X
STRIP_X_MIN = STRIP_M3_X - 7.0  # 4.15 mm past the M3 pocket flat
STRIP_X_MAX = STRIP_M3_X + STRIP_PITCH + 9.0  # 3.3 mm past the 1/4" pocket flat
STRIP_Y_FRONT = -8.5  # bed face; 1.9 mm behind the 1/4" pocket corner
STRIP_Y_BACK = 7.5  # the nut slots open here
STRIP_H = 24.0  # 1/4" nut at 14 + 5.8 deep leaves 4.2 mm of floor
STRIP_PRINT = (0, -1, 0)


def strip_envelope() -> Part:
    size = (STRIP_X_MAX - STRIP_X_MIN, STRIP_Y_BACK - STRIP_Y_FRONT, STRIP_H)
    return Pos(STRIP_X_MIN, STRIP_Y_FRONT, 0) * Box(*size, align=(Align.MIN,) * 3)


def strip_axes() -> dict[str, Axis]:
    """Design axes of the two holes, by label."""
    return {
        "M3": Axis((STRIP_M3_X, 0, STRIP_H), (0, 0, -1)),
        "1/4": Axis((STRIP_M3_X + STRIP_PITCH, 0, STRIP_H), (0, 0, -1)),
    }


def strip_cutters(p: Params) -> dict[str, list[Part]]:
    """[through-hole, side-slot nut pocket] per label, in the block's captive-nut form."""
    sizes = {
        "M3": (p.m3_through, p.m3_hex_af, p.m3_hex_h, p.top_m3_nut_depth),
        "1/4": (p.quarter_through, p.quarter_hex_af, p.quarter_hex_h, p.top_quarter_nut_depth),
    }
    cutters = {}
    for label, axis in strip_axes().items():
        hole_d, af, nut_h, depth = sizes[label]
        cutters[label] = captive_nut(
            axis.position,
            axis.direction,
            hole_d,
            STRIP_H,
            af,
            nut_h,
            depth,
            slot_dir=(0, 1, 0),
            slot_len=STRIP_Y_BACK + 1,  # 1 mm past the +Y face
        )
    return cutters


def build_strip(p: Params) -> Part:
    return cut(strip_envelope(), [c for cs in strip_cutters(p).values() for c in cs])


def checks_strip(part: Part, p: Params) -> list[Check]:
    wall_min = MATERIALS[PETG].wall_min
    env = strip_envelope()
    voids = {label: cavity_pieces(env, cs) for label, cs in strip_cutters(p).items()}
    hole_d = {"M3": p.m3_through, "1/4": p.quarter_through}
    af = {"M3": p.m3_hex_af, "1/4": p.quarter_hex_af}
    checks = [single_solid(part)]
    for label, axis in strip_axes().items():
        found = hole_axes(part, hole_d[label], direction=(0, 0, 1))
        checks += [
            count_is(f"{label} through-holes", len(found), 1),
            coaxial_check(f"{label} hole on its design axis", found, [axis], expected=1),
            *hex_pocket_checks(label, part, found, af[label], 1),
        ]
    all_voids = [piece for pieces in voids.values() for piece in pieces]
    checks.append(cavity_check("holes and nut pockets as designed", part, all_voids, env))
    for label, pieces in voids.items():
        checks.append(
            at_least(f"wall around {label} hole + nut", wall_around(part, pieces), wall_min)
        )
    return checks


# --- Registry ------------------------------------------------------------------------------------


def assembly_checks(parts: dict[str, Part], p: Params) -> list[Check]:
    """The two band halves together: seam holes coaxial, column fit, seam gaps left open."""
    return band.seam_checks(parts[FRONT_NAME], parts[BACK_NAME], p, p.coupon_h, bolt_zs(p))


PARTS = [
    PartSpec(FRONT_NAME, band.MATERIAL, band.PRINT, build_front, checks_front, ASSEMBLY),
    PartSpec(BACK_NAME, band.MATERIAL, band.PRINT, build_back, checks_back, ASSEMBLY),
    PartSpec(STRIP_NAME, PETG, STRIP_PRINT, build_strip, checks_strip, ASSEMBLY),
]
ASSEMBLY_CHECKS = {ASSEMBLY: assembly_checks}
