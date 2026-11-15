"""TPU split band: cross-section, grip ribs, seams, ears, seam bolts and the boss
(PLAN.md "Design" 1-2, sketch 5). Parts band_front and band_back (Task 3).

The coupon (Task 2) and the full band are built from these same functions, so the coupon
tests the real band geometry (a coupon_h slice without the boss). A band half is
`envelope - cutters`:

    envelope   half of the ring with its grip ribs (front: y <= -seam_gap/2, back:
               y >= +seam_gap/2), one ear per seam, and on the full back half the boss
    cutters    per seam bolt: the M3 through-hole; on the back half also the hex nut pocket.
               Per boss bolt: the M3 through-hole and the head recess.

Cross-section (top view; Z up, +Y back, origin on the column axis at the band bottom):

- Contact profile (the rib tips): the column profile (rounded rectangle column_w x column_d,
  r = column_r) offset outward by the TPU fit, 0.25 mm per side: 40.7 x 56.0, r = 12.25.
  The fit lets each half slide onto the column from the front or back without binding on
  the sides. In Y the bolts take it up: the halves close 2 x 0.25 = 0.5 mm before the rib
  tips touch the column, so 3.5 mm of each 4 mm seam gap stays open as clamping reserve.
- Inner face: the contact profile offset by rib_h (42.7 x 58.0, r = 13.25). Between the
  ribs the band stays rib_h + fit off the column.
- Outer profile: the inner face offset by band_wall (the thinnest wall is between the ribs).
- Grip ribs: horizontal rings all around the inner face, one every rib_pitch, centered on
  the band height (35 mm: 7 ribs at z = 2.5 ... 32.5). Section: rib_h proud, rib_tip wide at
  the column, 45 degree flanks. Horizontal, because the main load (the arm's weight) pulls
  the band down the column, and because with the band axis vertical on the bed the 45 degree
  flanks print without support. Vertical ribs would resist twisting better, but would turn
  the seam clamp into line contacts along the printed layers.
- Seams at -X and +X: the ring is split by a seam_gap wide slot centered on y = 0.
- Ears: a block ear_len out of the band side wall, ear_t thick in Y, full band height, on
  each side of each seam (sketch 5 draws them as flanges along the seam).
- Seam bolts run along +Y through both ears, centered on the ear. The head and washer sit
  on the front ear's outer face (reachable with a hex key from the front); the nut sits in
  a hex pocket in the back ear's outer face, so tightening pulls it onto the pocket floor
  and the hex stops it from spinning. With the band axis vertical on the bed the pocket is a
  horizontal hex hole. It lies with a flat up: the roof is a 3.3 mm bridge (af / sqrt(3))
  between two side faces 60 degrees from horizontal. With a corner up instead, the roof
  would be two faces only 30 degrees from horizontal, a long shallow overhang that sags in
  TPU. A bridge this short normally prints clean, but any sag shrinks the pocket the nut is
  pressed into: the coupon is the real test.
- Boss (full back half only): boss_w square face (X x Z) boss_depth proud of the outer back
  face, fused down onto the band's rounded back corners (the flat part of the back is only
  16 mm wide). Its bottom is flush with the band bottom, so it stands on the bed instead of
  overhanging 1.5 mm. 4 x M3 through-holes along +Y on an m3_spacing square centered on the
  face. The heads go in from the column side, each on a DIN 125 washer: the washer spreads
  the clamp load on the TPU (a bare ISO 7380 head bears on only about 16 mm2 and would sink
  in, relaxing the preload). Each hole has a head recess (m3_head_d, wide enough for the
  washer) whose floor sits m3_head_h + m3_washer_t + m3_head_extra behind the inner face, so
  a head m3_head_h tall on its washer ends m3_head_extra below the inner face and further
  below the rib tips. The recess cuts the ribs it crosses, so nothing of the band is left
  between a head and the column.

Contacts. As built, each half keeps the fit to the column (no contact). Clamped, each half
moves along Y onto the column; then the only contact is rib tips vs column (CLAMP_CONTACTS,
checked in seam_checks); the inner face, the head recesses and the bolt heads stay clear.
"""

import math

from build123d import Align, Axis, Box, Compound, Part, Pos, RectangleRounded, Sketch, extrude, loft

from holder.common import (
    COAXIAL_TOL,
    TOUCH_EPS,
    VOLUME_EPS,
    Check,
    CutterGroup,
    Mouth,
    PartSpec,
    at_least,
    cavity_check,
    cavity_pieces,
    circles,
    clash_checks,
    coaxial,
    coaxial_check,
    contact_faces,
    count_is,
    cut,
    hex_pocket,
    hex_pocket_checks,
    hole_axes,
    intersection_volume,
    material_spans,
    min_distance,
    planar_faces,
    rod,
    single_solid,
    void_volume_check,
    wall_around,
)
from holder.params import MATERIALS, TPU, Params

MATERIAL = TPU
PRINT = (0, 0, -1)  # band axis vertical, bed face = band bottom (PLAN.md "Print")

ASSEMBLY = "holder"
FRONT_NAME = "band_front"
BACK_NAME = "band_back"
COLUMN_NAME = "column"  # reference solid: drawn and clash-checked, never exported

FRONT, BACK = -1, 1  # half: sign of y
SEAMS = (-1, 1)  # seam: sign of x
NUT_HALF = BACK  # the seam nuts sit in the back ears, the heads on the front ears
HALF_NAMES = {FRONT: "front", BACK: "back"}
SEAM_NAMES = {-1: "-X", 1: "+X"}
BOLT_AXIS = (0, 1, 0)

# The intended contacts once the bolts pull both halves onto the column: rib tips vs column
# (names as used in seam_checks). Every other pair keeps a gap, clamped or as built.
CLAMP_CONTACTS = (("front half", "column"), ("back half", "column"))

# Seam gap that must stay open with both halves pressed onto the column. The design leaves
# seam_gap - 2 * fit = 3.5 mm.
SEAM_RESERVE_MIN = 2.0  # estimated: room for a column up to 1.5 mm shallower than measured
COLUMN_EXTRA = 10.0  # mm the reference column reaches past the band at both ends

Profile = tuple[float, float, float]  # rounded rectangle: width (X), depth (Y), corner radius


# --- Cross-section -------------------------------------------------------------------------------


def fit() -> float:
    return MATERIALS[TPU].fit


def grow(profile: Profile, by: float) -> Profile:
    """The rounded rectangle offset outward by `by`."""
    w, d, r = profile
    return w + 2 * by, d + 2 * by, r + by


def inner_profile(p: Params) -> Profile:
    """The contact profile (rib tips): the column plus the TPU fit."""
    return grow((p.column_w, p.column_d, p.column_r), fit())


def face_profile(p: Params) -> Profile:
    """The inner face between the ribs."""
    return grow(inner_profile(p), p.rib_h)


def outer_profile(p: Params) -> Profile:
    return grow(face_profile(p), p.band_wall)


def section(profile: Profile, z: float = 0.0) -> Sketch:
    return Pos(0, 0, z) * RectangleRounded(*profile)


def prism(profile: Profile, z0: float, z1: float) -> Part:
    return Pos(0, 0, z0) * extrude(RectangleRounded(*profile), z1 - z0)


def column(p: Params, h: float) -> Part:
    """Reference solid of the column (not a printed part), reaching past the band."""
    return prism((p.column_w, p.column_d, p.column_r), -COLUMN_EXTRA, h + COLUMN_EXTRA)


def rib_zs(p: Params, h: float) -> list[float]:
    """Center heights of the grip ribs on a band `h` tall: as many as fit at rib_pitch,
    centered on the band height."""
    n = math.floor(h / p.rib_pitch + 1e-9)
    return [h / 2 + (k - (n - 1) / 2) * p.rib_pitch for k in range(n)]


def rib_base(p: Params) -> float:
    """Rib width (Z) where it meets the inner face: the tip plus two 45 degree flanks."""
    return p.rib_tip + 2 * p.rib_h


def cavity(p: Params, h: float) -> Part:
    """The space inside the band (reaching 1 mm past both ends): everything inside the rib
    tips, plus everything inside the inner face except the ribs. Built as a union of the
    contact prism, the inner-face prisms between the ribs and the 45 degree rib flanks."""
    contact, face = inner_profile(p), face_profile(p)
    pieces = [prism(contact, -1, h + 1)]
    z = -1.0
    for zc in rib_zs(p, h):
        z0, z1 = zc - rib_base(p) / 2, zc + rib_base(p) / 2
        pieces.append(prism(face, z, z0))
        pieces.append(loft([section(face, z0), section(contact, z0 + p.rib_h)], ruled=True))
        pieces.append(loft([section(contact, z1 - p.rib_h), section(face, z1)], ruled=True))
        z = z1
    pieces.append(prism(face, z, h + 1))
    return pieces[0].fuse(*pieces[1:]).clean()


def ring(p: Params, h: float) -> Part:
    """The closed band ring with its grip ribs, before the seams split it."""
    return prism(outer_profile(p), 0, h) - cavity(p, h)


def wall_x(p: Params) -> float:
    """|x| of the band side wall's outer face (the ears stick out from here)."""
    return outer_profile(p)[0] / 2


def back_face_y(p: Params) -> float:
    """y of the band's outer back face (the boss stands on it)."""
    return outer_profile(p)[1] / 2


# --- Seam bolts ----------------------------------------------------------------------------------


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


def bolt_cutters(p: Params, half: int, seam: int, z: float) -> list[Part]:
    """Cutters for one seam bolt in one half: [M3 through-hole] (+ [nut pocket] on NUT_HALF).

    The hole cutter spans both ears, so the front and back holes share one axis by design.
    """
    y_start = -(p.seam_gap / 2 + p.ear_t + 1)
    x = bolt_x(p, seam)
    cutters = [rod(p.m3_through, -2 * y_start, origin=(x, y_start, z), axis=BOLT_AXIS)]
    if half == NUT_HALF:
        nut_bottom = ear_face_y(p, half) - p.m3_hex_h
        # slot_dir only orients the hex here (no slot): flats facing +-Z, so printed with
        # the band axis vertical the roof is a flat bridge (see the module docstring).
        cutters.append(
            hex_pocket(
                p.m3_hex_af,
                p.m3_hex_h + 1,  # 1 mm past the ear face
                origin=(x, nut_bottom, z),
                axis=BOLT_AXIS,
                slot_dir=(1, 0, 0),
            )
        )
    return cutters


def bolt_mouths(p: Params, half: int) -> list[Mouth]:
    """The faces a seam bolt's hole and pocket open on: the ear's seam face and outer face."""
    return [((0, half * p.seam_gap / 2, 0), BOLT_AXIS), ((0, ear_face_y(p, half), 0), BOLT_AXIS)]


# --- Boss ----------------------------------------------------------------------------------------


def boss_face_y(p: Params) -> float:
    """y of the boss face (the block's plate sits on it)."""
    return back_face_y(p) + p.boss_depth


def boss_center_z(p: Params) -> float:
    """z of the boss face center: the boss bottom is flush with the band bottom."""
    return p.boss_w / 2


def head_floor_y(p: Params) -> float:
    """y of the head recess floor (the washer seat): m3_head_recess_depth behind the inner
    face."""
    return face_profile(p)[1] / 2 + p.m3_head_recess_depth


def boss_bolts(p: Params) -> list[tuple[float, float]]:
    """(x, z) of the 4 boss bolts: an m3_spacing square centered on the boss face."""
    s, zc = p.m3_spacing / 2, boss_center_z(p)
    return [(x, zc + dz) for dz in (-s, s) for x in (-s, s)]


def boss_axis(p: Params, x: float, z: float) -> Axis:
    return Axis((x, 0, z), BOLT_AXIS)


def boss_block(p: Params) -> Part:
    """The boss before the cavity is cut: from inside the column space out to its face, so it
    fuses with the rounded back corners (the cavity cut then trims it to the inner face)."""
    w, face_y = p.boss_w, boss_face_y(p)
    return Pos(-w / 2, 0, 0) * Box(w, face_y, w, align=(Align.MIN,) * 3)


def rib_relief(p: Params, x: float, z: float, h: float) -> Part:
    """Removes the ribs in front of one head recess: inside the inner face, over the recess
    width in X and the full base of every rib the recess touches in Z. Without it the recess
    would leave thin crescent stubs of the ribs it only clips."""
    r = p.m3_head_d / 2
    z_lo, z_hi = z - r, z + r
    for zc in rib_zs(p, h):
        lo, hi = zc - rib_base(p) / 2, zc + rib_base(p) / 2
        if lo < z + r and hi > z - r:
            z_lo, z_hi = min(z_lo, lo), max(z_hi, hi)
    face = face_profile(p)
    box = Pos(x - r, 0, z_lo) * Box(2 * r, face[1], z_hi - z_lo, align=(Align.MIN,) * 3)
    return box & prism(face, z_lo - 1, z_hi + 1)


def boss_cutters(p: Params, x: float, z: float, h: float) -> list[Part]:
    """[M3 through-hole, head recess, rib relief] for one boss bolt of a band `h` tall. The
    hole and recess start inside the column space, so nothing is left in front of a head."""
    hole = rod(p.m3_through, boss_face_y(p) + 1, origin=(x, 0, z), axis=BOLT_AXIS)
    recess = rod(p.m3_head_d, head_floor_y(p), origin=(x, 0, z), axis=BOLT_AXIS)
    return [hole, recess, rib_relief(p, x, z, h)]


# --- Band half -----------------------------------------------------------------------------------


def half_space(p: Params, half: int, h: float) -> Part:
    """A box holding everything on one half's side of the seams (y * half >= seam_gap / 2),
    1 mm past the band at both ends."""
    _, d, _ = outer_profile(p)
    size = 2 * (d + p.ear_len + p.boss_depth)  # larger than the whole band
    return Pos(0, half * (p.seam_gap / 2 + size / 2), h / 2) * Box(size, size, h + 2)


def envelope(p: Params, half: int, h: float, boss: bool = False) -> Part:
    """One band half before any hole: half of the ribbed ring, its two ears, and the boss
    (back half only)."""
    if boss and half != BACK:
        raise ValueError("the boss sits on the back half")
    body = prism(outer_profile(p), 0, h)
    if boss:
        body += boss_block(p)
    body = (body - cavity(p, h)) & half_space(p, half, h)
    # The ear starts half-way into the side wall, so it fuses with it.
    ear_start = wall_x(p) - p.band_wall / 2
    ear_w = p.ear_len + p.band_wall / 2
    ear_y = half * (p.seam_gap / 2 + p.ear_t / 2)
    for seam in SEAMS:
        body += Pos(seam * (ear_start + ear_w / 2), ear_y, h / 2) * Box(ear_w, p.ear_t, h)
    return body


def cutter_groups(p: Params, half: int, h: float, bolt_zs, boss: bool = False):
    """[CutterGroup] per bolt: the seam bolts, then the boss bolts. A boss bolt opens on the
    boss face; its recess opens into the band inside (half_checks passes that as open_to)."""
    groups = [
        CutterGroup(
            f"seam bolt {SEAM_NAMES[seam]} z={z:g}",
            bolt_cutters(p, half, seam, z),
            bolt_mouths(p, half),
        )
        for seam, z in bolts(bolt_zs)
    ]
    if boss:
        boss_face = [((0, boss_face_y(p), 0), BOLT_AXIS)]
        groups += [
            CutterGroup(f"boss bolt x={x:g} z={z:g}", boss_cutters(p, x, z, h), boss_face)
            for x, z in boss_bolts(p)
        ]
    return groups


def inside_mouths(p: Params, half: int, h: float) -> list[Mouth]:
    """The faces the band inside opens on, within one half: the seam faces, bottom and top."""
    return [
        ((0, half * p.seam_gap / 2, 0), BOLT_AXIS),
        ((0, 0, 0), (0, 0, 1)),
        ((0, 0, h), (0, 0, 1)),
    ]


def build_half(p: Params, half: int, h: float, bolt_zs, boss: bool = False) -> Part:
    """A band half `h` tall with seam bolts at heights `bolt_zs`, in assembly coordinates."""
    cutters = [c for g in cutter_groups(p, half, h, bolt_zs, boss) for c in g.cutters]
    return cut(envelope(p, half, h, boss), cutters)


# --- Checks of one half --------------------------------------------------------------------------


def _z_span(face) -> tuple[float, float]:
    zs = [v.Z for v in face.vertices()]
    return min(zs), max(zs)


def rib_checks(part: Part, p: Params, half: int, h: float) -> list[Check]:
    """The grip ribs on this half's flat (front or back): count, height positions, tip width
    and how far they stand proud of the inner face, measured on the planar faces."""
    tip_y = half * inner_profile(p)[1] / 2
    face_y = half * face_profile(p)[1] / 2
    flats = planar_faces(part, BOLT_AXIS)
    tips = [f for f in flats if abs(f.center().Y - tip_y) <= TOUCH_EPS]
    faces = [f for f in flats if abs(f.center().Y - face_y) <= TOUCH_EPS]
    found: list[tuple[float, float]] = []  # (center z, width) per rib
    for lo, hi in map(_z_span, tips):
        if not any(abs((lo + hi) / 2 - zc) <= COAXIAL_TOL for zc, _ in found):
            found.append(((lo + hi) / 2, hi - lo))
    design = rib_zs(p, h)
    pos_err = max((min(abs(zc - z) for z, _ in found) for zc in design), default=0.0)
    width_err = max((abs(w - p.rib_tip) for _, w in found), default=math.inf)
    height = abs(faces[0].center().Y - tips[0].center().Y) if tips and faces else 0.0
    label = HALF_NAMES[half]
    return [
        count_is(f"grip ribs on the {label} flat", len(found), len(design)),
        Check(
            "grip ribs at their design heights and tip width",
            bool(found) and pos_err <= COAXIAL_TOL and width_err <= TOUCH_EPS,
            f"worst z off {pos_err:.4f} mm, worst tip width off {width_err:.4f} mm",
        ),
        Check(
            "grip rib height over the inner face",
            abs(height - p.rib_h) <= TOUCH_EPS,
            f"{height:.3f} mm (design {p.rib_h:g})",
        ),
    ]


def head_solids(part: Part, p: Params, axes: list[Axis]) -> list[Part]:
    """A bolt head on its washer (m3_head_d wide, m3_washer_t + m3_head_h tall: the stack the
    recess is made for) standing on each recess floor. The floor is the deepest circular
    edge of the recess around each axis."""
    rims = circles(part, p.m3_head_d / 2, direction=BOLT_AXIS)
    stack = p.m3_washer_t + p.m3_head_h
    heads = []
    for ax in axes:
        ys = [c.axis.position.Y for c in rims if coaxial(c.axis, ax)]
        if ys:
            x, z = ax.position.X, ax.position.Z
            heads.append(rod(p.m3_head_d, stack, origin=(x, max(ys), z), axis=(0, -1, 0)))
    return heads


def boss_checks(part: Part, p: Params, h: float, boss_holes: list[Axis]) -> list[Check]:
    n = len(boss_bolts(p))
    design = [boss_axis(p, x, z) for x, z in boss_bolts(p)]
    recesses = hole_axes(part, p.m3_head_d, direction=BOLT_AXIS)
    heads = head_solids(part, p, recesses)
    inside = prism(face_profile(p), -1, h + 1)  # everything inside the inner face
    overlap = sum(intersection_volume(head, inside) for head in heads)
    gap = min((min_distance(head, inside) for head in heads), default=0.0)
    return [
        count_is("M3 boss holes", len(boss_holes), n),
        coaxial_check("M3 boss holes on design axes", boss_holes, design, expected=n),
        count_is("M3 head recesses", len(recesses), n),
        coaxial_check("head recesses coaxial with the boss holes", recesses, boss_holes, n),
        Check(
            "boss bolt heads below the inner face",
            len(heads) == n and overlap <= VOLUME_EPS and gap >= p.m3_head_extra - TOUCH_EPS,
            f"{len(heads)} heads, overlap {overlap:.1e} mm3, "
            f"{gap:.3f} mm below the face (min {p.m3_head_extra:g})",
        ),
    ]


def half_checks(part: Part, p: Params, half: int, h: float, bolt_zs, boss=False) -> list[Check]:
    """Checks of one band half against its design (the same functions that built it)."""
    wall_min = MATERIALS[MATERIAL].wall_min
    env = envelope(p, half, h, boss)
    groups = cutter_groups(p, half, h, bolt_zs, boss)
    voids = [(g, cavity_pieces(env, g.cutters)) for g in groups]
    seam_bolts = bolts(bolt_zs)
    n = len(seam_bolts)
    n_boss = len(boss_bolts(p)) if boss else 0
    m3 = hole_axes(part, p.m3_through, direction=BOLT_AXIS)
    seam_holes = [a for a in m3 if abs(a.position.X) > wall_x(p)]
    boss_holes = [a for a in m3 if abs(a.position.X) <= p.boss_w / 2]
    design = [bolt_axis(p, seam, z) for seam, z in seam_bolts]
    checks = [
        single_solid(part),
        count_is("M3 holes", len(m3), n + n_boss),
        count_is("M3 seam holes", len(seam_holes), n),
        coaxial_check("M3 seam holes on design axes", seam_holes, design, expected=n),
        *hex_pocket_checks("M3", part, seam_holes, p.m3_hex_af, n if half == NUT_HALF else 0),
        *rib_checks(part, p, half, h),
    ]
    if boss:
        checks += boss_checks(part, p, h, boss_holes)
    else:
        recesses = hole_axes(part, p.m3_head_d, direction=BOLT_AXIS)
        checks.append(count_is("M3 head recesses", len(recesses), 0))
    all_pieces = [piece for _, pieces in voids for piece in pieces]
    checks.append(cavity_check("holes and pockets as designed", part, all_pieces, env))
    checks.append(void_volume_check("no void beyond the designed ones", part, all_pieces, env))
    inside = cavity(p, h)
    # Boss bolts: [hole, recess, rib relief]. Every recess opens into the band inside, which
    # the rib reliefs widen; together they are one space. The inner face between the ribs
    # runs across the reliefs from one recess to the next, so it ends on their rims: the
    # opening holds all of them, not just the bolt's own.
    boss_voids = [pieces for g, pieces in voids if g.label.startswith("boss")]
    boss_pieces = [piece for pieces in boss_voids for piece in pieces]
    opening = Compound([inside, *(piece for pieces in boss_voids for piece in pieces[1:])])
    for group, pieces in voids:
        open_to = None
        if group.label.startswith("boss"):
            pieces, open_to = pieces[:2], opening
        wall = wall_around(part, pieces, mouths=group.mouths, open_to=open_to)
        checks.append(at_least(f"wall around {group.label}", wall, wall_min))
    # The ring itself: from the band inside (this half of it) to every other face. The boss
    # bolts open into it; their walls are measured above.
    own_inside = inside & half_space(p, half, h)
    wall = wall_around(
        part,
        own_inside,
        mouths=inside_mouths(p, half, h),
        open_to=Compound(boss_pieces) if boss_pieces else None,
    )
    checks.append(at_least("band wall around the inside", wall, wall_min))
    return checks


# --- Checks of the two halves together -----------------------------------------------------------


def _seam_side(p: Params, seam: int, h: float) -> Part:
    """A box holding everything on one seam's side (x * seam > 0)."""
    size = 4 * (wall_x(p) + p.ear_len + p.boss_depth)
    return Pos(seam * size / 2, 0, h / 2) * Box(size, size, h + 2 * COLUMN_EXTRA)


def clamp_travel(part: Part, col: Part) -> float:
    """How far a half moves along Y until it sits on the column (its clearance)."""
    return min_distance(part, col)


def clamped(part: Part, half: int, travel: float) -> Part:
    """The half moved along Y onto the column by `travel`."""
    return Pos(0, -half * travel, 0) * part


def rib_contact_check(name: str, part: Part, col: Part, p: Params, h: float) -> Check:
    """`part` (clamped) touches the column over an area only on rib tips, and on every rib."""
    faces = contact_faces(part, col)
    ribs = rib_zs(p, h)
    reach = p.rib_tip / 2 + TOUCH_EPS
    touched, elsewhere = set(), 0
    for lo, hi in map(_z_span, faces):
        k = next((k for k, zc in enumerate(ribs) if zc - reach <= lo and hi <= zc + reach), None)
        if k is None:
            elsewhere += 1
        else:
            touched.add(k)
    return Check(
        f"clamped: {name} touches the column only on rib tips",
        bool(faces) and elsewhere == 0 and len(touched) == len(ribs),
        f"{len(faces)} contact faces on {len(touched)}/{len(ribs)} ribs, {elsewhere} elsewhere",
    )


def seam_bolt(p: Params, x: float, seat_y: float, z: float, closed_by: float) -> Part:
    """The seam bolt's shank (nominal M3, under the head) with its head on the washer, the
    front half moved `closed_by` toward the back half (the seam gap that much narrower)."""
    head_y = seat_y - p.m3_washer_t + closed_by
    return rod(3.0, p.seam_bolt_len, origin=(x, head_y, z), axis=BOLT_AXIS)


def seam_hardware_checks(
    front: Part, back: Part, p: Params, bolt_zs, others: dict[str, Part] | None = None
) -> list[Check]:
    """Hardware for each seam bolt: M3 x seam_bolt_len + washer + nut.

    Measured along a probe line parallel to the bolt, between the hole (r 1.7) and the nut
    pocket flats (2.85): from the washer seat on the front ear to the nut pocket floor in
    the back ear. As built (the seam open by seam_gap, the longest stack) the bolt must
    reach through washer, both ears, the gap and the whole nut; the washer must clear the
    band wall next to the ear. Clamping only shortens the stack: with the seam closed all
    the way (ears touching, past any clamp reserve) the bolt still holds the whole nut and
    its tip, now seam_gap further out, must stay clear of the back half and of `others`
    (other parts of the assembly).
    """
    probe = (p.m3_through / 2 + p.m3_hex_af / 2) / 2
    length = 4 * (p.seam_gap + 2 * p.ear_t)
    obstacles = {"back half": back, **(others or {})}
    checks = []
    for seam, z in bolts(bolt_zs):
        x = bolt_x(p, seam)
        origin = (x, -length / 2, z + probe)
        f_spans = material_spans(front, origin, BOLT_AXIS, length)
        b_spans = material_spans(back, origin, BOLT_AXIS, length)
        name = f"seam {SEAM_NAMES[seam]} z={z:g}"
        if not f_spans or not b_spans:
            checks.append(Check(f"{name}: hardware fits", False, "probe missed an ear"))
            continue
        seat_y = f_spans[0][0] - length / 2
        stack = b_spans[-1][1] - f_spans[0][0]  # seat to nut pocket floor
        need = p.m3_washer_t + stack + p.m3_nut_h
        spare = p.seam_bolt_len - need
        checks.append(
            Check(
                f"{name}: M3x{p.seam_bolt_len:g} + washer + nut fit the ear stack",
                spare >= 0,
                f"needs {need:.2f} mm (washer {p.m3_washer_t:g} + ears and gap {stack:.2f} "
                f"+ nut {p.m3_nut_h:g}), {spare:.2f} mm past the nut with the seam open",
            )
        )
        shank = seam_bolt(p, x, seat_y, z, p.seam_gap)
        hits = {n: intersection_volume(shank, o) for n, o in obstacles.items()}
        hit = sorted(n for n, v in hits.items() if v > VOLUME_EPS)
        checks.append(
            Check(
                f"{name}: seam closed, bolt holds the nut and its tip is clear",
                spare + p.seam_gap >= 0 and not hit,
                f"tip {spare + p.seam_gap:.2f} mm past the nut, "
                + (f"hits {', '.join(hit)}" if hit else f"clear of {', '.join(sorted(hits))}"),
            )
        )
        washer = rod(p.m3_washer_d, p.m3_washer_t, origin=(x, seat_y, z), axis=(0, -1, 0))
        overlap = intersection_volume(washer, front)
        checks.append(
            Check(
                f"{name}: washer seats on the ear, clear of the band",
                overlap <= VOLUME_EPS and min_distance(washer, front) <= TOUCH_EPS,
                f"overlap {overlap:.1e} mm3",
            )
        )
    return checks


def seam_checks(
    front: Part, back: Part, p: Params, h: float, bolt_zs, others: dict[str, Part] | None = None
) -> list[Check]:
    """Assembly checks of the two halves: holes coaxial across each seam, the column fits,
    clamped the halves touch the column only on the rib tips and each seam gap stays open,
    and the seam hardware fits (its bolt tips clear of `others` with the seam closed)."""
    checks = []
    sides = {seam: _seam_side(p, seam, h) for seam in SEAMS}
    front_axes = hole_axes(front, p.m3_through, direction=BOLT_AXIS)
    back_axes = hole_axes(back, p.m3_through, direction=BOLT_AXIS)
    for seam in SEAMS:
        name = SEAM_NAMES[seam]
        f_axes = [a for a in front_axes if a.position.X * seam > wall_x(p)]
        b_axes = [a for a in back_axes if a.position.X * seam > wall_x(p)]
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
        travel[half] = clamp_travel(part, col)
        checks.append(
            Check(
                f"{HALF_NAMES[half]} half clears the column",
                overlap <= VOLUME_EPS and travel[half] > TOUCH_EPS,
                f"overlap {overlap:.1e} mm3, clearance {travel[half]:.3f} mm (fit {fit():g})",
            )
        )

    # Clamp: each half moves along Y onto the column (by its clearance). Named contacts only.
    front_c = clamped(front, FRONT, travel[FRONT])
    back_c = clamped(back, BACK, travel[BACK])
    together = {"front half": front_c, "back half": back_c, "column": col}
    for c in clash_checks(together, CLAMP_CONTACTS):
        checks.append(Check(f"clamped: {c.name}", c.ok, c.measured))
    checks.append(rib_contact_check("front half", front_c, col, p, h))
    checks.append(rib_contact_check("back half", back_c, col, p, h))
    for seam in SEAMS:
        left = min_distance(front_c & sides[seam], back_c & sides[seam])
        checks.append(
            at_least(f"clamped: seam {SEAM_NAMES[seam]} gap still open", left, SEAM_RESERVE_MIN)
        )
    return checks + seam_hardware_checks(front, back, p, bolt_zs, others)


# --- Parts band_front, band_back -----------------------------------------------------------------


def build_front(p: Params) -> Part:
    return build_half(p, FRONT, p.band_h, p.ear_bolt_z)


def build_back(p: Params) -> Part:
    return build_half(p, BACK, p.band_h, p.ear_bolt_z, boss=True)


def checks_front(part: Part, p: Params) -> list[Check]:
    return half_checks(part, p, FRONT, p.band_h, p.ear_bolt_z)


def checks_back(part: Part, p: Params) -> list[Check]:
    return half_checks(part, p, BACK, p.band_h, p.ear_bolt_z, boss=True)


def assembly_checks(parts: dict[str, Part], p: Params) -> list[Check]:
    """The full band: the seam checks, plus the boss bolt heads clear of the column once the
    back half is clamped onto it."""
    front, back = parts[FRONT_NAME], parts[BACK_NAME]
    others = {n: s for n, s in parts.items() if n not in (FRONT_NAME, BACK_NAME)}
    checks = seam_checks(front, back, p, p.band_h, p.ear_bolt_z, others)
    col = parts[COLUMN_NAME] if COLUMN_NAME in parts else column(p, p.band_h)
    back_c = clamped(back, BACK, clamp_travel(back, col))
    heads = head_solids(back_c, p, hole_axes(back_c, p.m3_head_d, direction=BOLT_AXIS))
    gap = min((min_distance(head, col) for head in heads), default=0.0)
    overlap = sum(intersection_volume(head, col) for head in heads)
    checks.append(
        Check(
            "clamped: boss bolt heads clear the column",
            len(heads) == len(boss_bolts(p))
            and overlap <= VOLUME_EPS
            and gap >= p.m3_head_extra - TOUCH_EPS,
            f"{len(heads)} heads, gap {gap:.3f} mm (min {p.m3_head_extra:g})",
        )
    )
    return checks


def references(p: Params) -> dict[str, Part]:
    """Reference solids of the holder assembly (clash-checked and drawn, never exported)."""
    return {COLUMN_NAME: column(p, p.band_h)}


PARTS = [
    PartSpec(FRONT_NAME, MATERIAL, PRINT, build_front, checks_front, ASSEMBLY),
    PartSpec(BACK_NAME, MATERIAL, PRINT, build_back, checks_back, ASSEMBLY),
]
ASSEMBLY_CHECKS = {ASSEMBLY: assembly_checks}
REFERENCES = {ASSEMBLY: references}
