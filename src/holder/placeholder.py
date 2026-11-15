"""TEMPORARY placeholder part so the pipeline runs end to end (Task 2 removes it).

It doubles as the template of a single-part module: NAME, MATERIAL, PRINT, build, checks,
and PARTS for the registry. A PETG block with an M3 through-hole and a captive M3 nut
in a side-slot pocket, so the helpers in common.py get exercised on real geometry.
"""

from build123d import Align, Box, Part

from holder.common import (
    VOLUME_EPS,
    Check,
    PartSpec,
    at_least,
    at_most,
    cavity_check,
    coaxial_check,
    count_is,
    hex_pocket,
    hole_axes,
    intersection_volume,
    rod,
    single_solid,
    wall_around,
)
from holder.params import MATERIALS, PETG, Params

NAME = "placeholder"
MATERIAL = PETG
PRINT = (0, 0, -1)  # bed face = bottom

SIZE = (30.0, 20.0, 10.0)
HOLE_X = 7.0  # M3 axis, along Z
NUT_Z = 3.5  # nut pocket bottom


def _envelope() -> Part:
    return Box(*SIZE, align=(Align.CENTER, Align.CENTER, Align.MIN))


def _hole(p: Params) -> Part:
    return rod(p.m3_through, SIZE[2] + 2, origin=(HOLE_X, 0, -1))


def _nut_pocket(p: Params) -> Part:
    slot_len = SIZE[0] / 2 - HOLE_X + 1  # past the +X face
    return hex_pocket(
        p.m3_hex_af, p.m3_hex_h, origin=(HOLE_X, 0, NUT_Z), slot_dir=(1, 0, 0), slot_len=slot_len
    )


def build(p: Params) -> Part:
    return _envelope() - _hole(p) - _nut_pocket(p)


def checks(part: Part, p: Params) -> list[Check]:
    wall_min = MATERIALS[MATERIAL].wall_min
    envelope = _envelope()
    hole_void = _hole(p) & envelope
    nut_void = _nut_pocket(p) & envelope
    holes = hole_axes(part, p.m3_through, direction=(0, 0, 1))
    design_axis = hole_axes(_hole(p), p.m3_through)
    return [
        single_solid(part),
        count_is("M3 through-holes", len(holes), 1),
        coaxial_check("M3 hole on its design axis", holes, design_axis, expected=1),
        at_most("M3 nut pocket clear", intersection_volume(part, nut_void), VOLUME_EPS, "mm3"),
        cavity_check("M3 hole + nut pocket as designed", part, [hole_void, nut_void], envelope),
        # The hole and the nut pocket are connected: one cavity.
        at_least("wall around M3 hole + nut", wall_around(part, [hole_void, nut_void]), wall_min),
    ]


PARTS = [PartSpec(NAME, MATERIAL, PRINT, build, checks)]
