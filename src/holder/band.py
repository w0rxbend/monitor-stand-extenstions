"""TPU split band: cross-section, seams, ears and seam bolts (PLAN.md "Design" 1, sketch 5).

The coupon (Task 2) and the full band (Task 3) are built from these same functions, so the
coupon tests the real band geometry. A band half is `envelope - cutters`:

    envelope   half of the ring (front: y <= -seam_gap/2, back: y >= +seam_gap/2), plus one
               ear per seam
    cutters    per seam bolt: the M3 through-hole; on the back half also the hex nut pocket

Cross-section (top view; Z up, +Y back, origin on the column axis at the band bottom):

- Inner profile: the column profile (rounded rectangle column_w x column_d, r = column_r)
  offset outward by the TPU fit, 0.25 mm per side: 40.7 x 56.0, r = 12.25. The fit lets
  each half slide onto the column from the front or back without binding on the sides. In
  Y the bolts take it up: the halves close 2 x 0.25 = 0.5 mm before they touch the column,
  so 3.5 mm of each 4 mm seam gap stays open as clamping reserve (checked in seam_checks).
- Outer profile: the inner profile offset by band_wall.
- Seams at -X and +X: the ring is split by a seam_gap wide slot centered on y = 0. Both
  seams close together, so the bolts can take up to seam_gap of column depth.
- Ears: a block ear_len out of the band side wall, ear_t thick in Y, full band height, on
  each side of each seam (sketch 5 draws them as flanges along the seam).
- Seam bolts run along +Y through both ears, centered on the ear. The head and washer sit
  on the front ear's outer face (reachable with a hex key from the front); the nut sits in
  a hex pocket in the back ear's outer face, so tightening pulls it onto the pocket floor
  and the hex stops it from spinning. The pocket's corners point up and down (Z), so with
  the band axis vertical on the bed its roof is a 60 degree peak that prints without support.
"""

from build123d import Axis, Box, Part, Pos, RectangleRounded, extrude

from holder.common import (
    TOUCH_EPS,
    VOLUME_EPS,
    Check,
    at_least,
    cavity_check,
    cavity_pieces,
    coaxial_check,
    count_is,
    cut,
    hex_pocket,
    hex_pocket_checks,
    hole_axes,
    intersection_volume,
    min_distance,
    rod,
    single_solid,
    wall_around,
)
from holder.params import MATERIALS, TPU, Params

MATERIAL = TPU
PRINT = (0, 0, -1)  # band axis vertical, bed face = band bottom (PLAN.md "Print")

FRONT, BACK = -1, 1  # half: sign of y
SEAMS = (-1, 1)  # seam: sign of x
NUT_HALF = BACK  # the seam nuts sit in the back ears, the heads on the front ears
HALF_NAMES = {FRONT: "front", BACK: "back"}
SEAM_NAMES = {-1: "-X", 1: "+X"}
BOLT_AXIS = (0, 1, 0)

# Seam gap that must stay open with both halves pressed onto the column. The design leaves
# seam_gap - 2 * fit = 3.5 mm.
SEAM_RESERVE_MIN = 2.0  # estimated: room for a column up to 1.5 mm shallower than measured
COLUMN_EXTRA = 10.0  # mm the reference column reaches past the band at both ends


def fit() -> float:
    return MATERIALS[TPU].fit


def inner_profile(p: Params) -> tuple[float, float, float]:
    """(width, depth, corner radius) of the band's inner face: the column plus the TPU fit."""
    f = fit()
    return p.column_w + 2 * f, p.column_d + 2 * f, p.column_r + f


def column(p: Params, h: float) -> Part:
    """Reference solid of the column (not a printed part), reaching past the band."""
    section = RectangleRounded(p.column_w, p.column_d, p.column_r)
    return Pos(0, 0, -COLUMN_EXTRA) * extrude(section, h + 2 * COLUMN_EXTRA)


def ring(p: Params, h: float) -> Part:
    """The closed band ring, before the seams split it."""
    w, d, r = inner_profile(p)
    t = p.band_wall
    section = RectangleRounded(w + 2 * t, d + 2 * t, r + t) - RectangleRounded(w, d, r)
    return extrude(section, h)


def wall_x(p: Params) -> float:
    """|x| of the band side wall's outer face (the ears stick out from here)."""
    return inner_profile(p)[0] / 2 + p.band_wall


def bolt_x(p: Params, seam: int) -> float:
    """x of the seam bolt axis: centered on the ear."""
    return seam * (wall_x(p) + p.ear_len / 2)


def ear_face_y(p: Params, half: int) -> float:
    """y of the ear's outer face (washer seat on the front, nut pocket opening on the back)."""
    return half * (p.seam_gap / 2 + p.ear_t)


def bolts(bolt_zs) -> list[tuple[int, float]]:
    """(seam, z) of every seam bolt."""
    return [(seam, z) for seam in SEAMS for z in bolt_zs]


def bolt_axis(p: Params, seam: int, z: float) -> Axis:
    return Axis((bolt_x(p, seam), 0, z), BOLT_AXIS)


def envelope(p: Params, half: int, h: float) -> Part:
    """One band half before any hole: half of the ring plus its two ears."""
    _, d, _ = inner_profile(p)
    size = 2 * (d + p.band_wall + p.ear_len)  # larger than the whole band
    keep = Pos(0, half * (p.seam_gap / 2 + size / 2), h / 2) * Box(size, size, h + 2)
    body = ring(p, h) & keep
    # The ear starts half-way into the side wall, so it fuses with it.
    ear_start = wall_x(p) - p.band_wall / 2
    ear_w = p.ear_len + p.band_wall / 2
    ear_y = half * (p.seam_gap / 2 + p.ear_t / 2)
    for seam in SEAMS:
        body += Pos(seam * (ear_start + ear_w / 2), ear_y, h / 2) * Box(ear_w, p.ear_t, h)
    return body


def bolt_cutters(p: Params, half: int, seam: int, z: float) -> list[Part]:
    """Cutters for one seam bolt in one half: [M3 through-hole] (+ [nut pocket] on NUT_HALF).

    The hole cutter spans both ears, so the front and back holes share one axis by design.
    """
    y_start = -(p.seam_gap / 2 + p.ear_t + 1)
    x = bolt_x(p, seam)
    cutters = [rod(p.m3_through, -2 * y_start, origin=(x, y_start, z), axis=BOLT_AXIS)]
    if half == NUT_HALF:
        nut_bottom = ear_face_y(p, half) - p.m3_hex_h
        # slot_dir only orients the hex here (no slot): corners along Z, flats facing +-X.
        cutters.append(
            hex_pocket(
                p.m3_hex_af,
                p.m3_hex_h + 1,  # 1 mm past the ear face
                origin=(x, nut_bottom, z),
                axis=BOLT_AXIS,
                slot_dir=(0, 0, 1),
            )
        )
    return cutters


def build_half(p: Params, half: int, h: float, bolt_zs) -> Part:
    """A band half `h` tall with seam bolts at heights `bolt_zs`, in assembly coordinates."""
    cutters = [c for seam, z in bolts(bolt_zs) for c in bolt_cutters(p, half, seam, z)]
    return cut(envelope(p, half, h), cutters)


def half_checks(part: Part, p: Params, half: int, h: float, bolt_zs) -> list[Check]:
    """Checks of one band half against its design (the same functions that built it)."""
    wall_min = MATERIALS[MATERIAL].wall_min
    env = envelope(p, half, h)
    seam_bolts = bolts(bolt_zs)
    n = len(seam_bolts)
    voids = [cavity_pieces(env, bolt_cutters(p, half, seam, z)) for seam, z in seam_bolts]
    found = hole_axes(part, p.m3_through, direction=BOLT_AXIS)
    design = [bolt_axis(p, seam, z) for seam, z in seam_bolts]
    checks = [
        single_solid(part),
        count_is("M3 seam holes", len(found), n),
        coaxial_check("M3 seam holes on design axes", found, design, expected=n),
        *hex_pocket_checks("M3", part, found, p.m3_hex_af, n if half == NUT_HALF else 0),
        cavity_check(
            "seam holes and nut pockets as designed",
            part,
            [piece for pieces in voids for piece in pieces],
            env,
        ),
    ]
    for (seam, z), pieces in zip(seam_bolts, voids, strict=True):
        name = f"wall around seam bolt {SEAM_NAMES[seam]} z={z:g}"
        checks.append(at_least(name, wall_around(part, pieces), wall_min))
    return checks


def _seam_side(p: Params, seam: int, h: float) -> Part:
    """A box holding everything on one seam's side (x * seam > 0)."""
    size = 4 * (wall_x(p) + p.ear_len)
    return Pos(seam * size / 2, 0, h / 2) * Box(size, size, h + 2 * COLUMN_EXTRA)


def seam_checks(front: Part, back: Part, p: Params, h: float, bolt_zs) -> list[Check]:
    """Assembly checks of the two halves: holes coaxial across each seam, the column fits,
    and each seam gap stays open with both halves pressed onto the column."""
    checks = []
    sides = {seam: _seam_side(p, seam, h) for seam in SEAMS}
    front_axes = hole_axes(front, p.m3_through, direction=BOLT_AXIS)
    back_axes = hole_axes(back, p.m3_through, direction=BOLT_AXIS)
    for seam in SEAMS:
        name = SEAM_NAMES[seam]
        f_axes = [a for a in front_axes if a.position.X * seam > 0]
        b_axes = [a for a in back_axes if a.position.X * seam > 0]
        checks.append(
            coaxial_check(
                f"seam {name}: bolt holes coaxial across the seam",
                f_axes,
                b_axes,
                expected=len(bolt_zs),
            )
        )
        gap = min_distance(front & sides[seam], back & sides[seam])
        checks.append(
            Check(
                f"seam {name}: gap as designed",
                abs(gap - p.seam_gap) <= TOUCH_EPS,
                f"{gap:.3f} mm (design {p.seam_gap:g})",
            )
        )

    col = column(p, h)
    travel = {}
    for half, part in ((FRONT, front), (BACK, back)):
        overlap = intersection_volume(part, col)
        travel[half] = min_distance(part, col)
        checks.append(
            Check(
                f"{HALF_NAMES[half]} half clears the column",
                overlap <= VOLUME_EPS and travel[half] > TOUCH_EPS,
                f"overlap {overlap:.1e} mm3, clearance {travel[half]:.3f} mm (fit {fit():g})",
            )
        )

    # Clamp: each half moves along Y onto the column (by its clearance); the gaps must stay open.
    front_c = Pos(0, travel[FRONT], 0) * front
    back_c = Pos(0, -travel[BACK], 0) * back
    contact = max(min_distance(front_c, col), min_distance(back_c, col))
    overlap = intersection_volume(front_c, col) + intersection_volume(back_c, col)
    checks.append(
        Check(
            "clamped: both halves on the column",
            contact <= TOUCH_EPS and overlap <= VOLUME_EPS,
            f"gap to column {contact:.4f} mm, overlap {overlap:.1e} mm3",
        )
    )
    for seam in SEAMS:
        left = min_distance(front_c & sides[seam], back_c & sides[seam])
        checks.append(
            at_least(f"clamped: seam {SEAM_NAMES[seam]} gap still open", left, SEAM_RESERVE_MIN)
        )
    return checks
