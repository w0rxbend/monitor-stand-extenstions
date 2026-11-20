"""Collar: two PETG shells round the column, and two TPU 95A grip liners.

The PETG shell carries the clamp load. TPU 95A loses 30-38 % of its compression to set
(Essentium, BASF TDS), so the liner is only a 2 mm grip layer with 45 degree V ribs.

Each shell is two G2 skins fused: a deck (z deck_z..collar_h) on the full plan, whose top
is one flat plane from the ring into the mount face (MF), and a lower ring set
`ring_inset` inside the deck. Where the deck's rolled underside meets the ring wall there
is a shadow groove all round, and the MF pad cantilevers off the ring on the deck alone.
Every outer edge is a G2 roll except the 45 degree bed lead on the top face, which is the
print bed. The walls are straight: the seam faces belong to the same skin and must stay flat.
Four seam screws run inside the wall from the -Y face (the monitor side) into heat-set
inserts pressed into the back shell's seam face. The 1/4" port nut slides in from the
bore, behind the liner, and drops into its pocket.
"""

from math import sqrt

from build123d import Face, Part, Pos, RectangleRounded, Vector, extrude, loft

from holder.common import (
    Check,
    SeatedInsert,
    annulus_footprint,
    at_least,
    bearing_row,
    box_from,
    contact_area,
    encloses,
    insert_shear_row,
    named,
    one_solid,
    overhang_rows,
    print_orientation,
    ring_probe,
    rod,
    skin_rows,
    teardrop_ring,
    teardrop_rod,
)
from holder.form import BedLead, Roll, curvature_jumps, g2_skin, soft_outline
from holder.mount import MountFace
from holder.params import (
    INSERT_ENGAGE,
    INSERT_SINK,
    M3_CLEAR,
    M3_INSERT,
    M3_SHCS,
    M3_WASHER,
    M3_WASHER_BORE,
    MATERIALS,
    PETG_WALL,
    PRELOAD_N,
    Params,
)

PRINT = {
    "collar_front": (0, 0, 1),
    "collar_back": (0, 0, 1),
    "liner_front": (0, 0, -1),
    "liner_back": (0, 0, -1),
}
MATERIAL = {"collar_front": "PETG", "collar_back": "PETG",
            "liner_front": "TPU95A", "liner_back": "TPU95A"}
DECK_TOP = BedLead(1.0, 3.0)  # the top face prints on the bed
DECK_UNDER = Roll(4.0)
RING_BOTTOM = Roll(3.5)
RING_TOP = Roll(1.0)  # buried in the deck
# No barrel on the collar: the seam faces are walls of the same skin, and a bulge there
# would close the seam before the 2.0 gap and crush the liner ribs.
DECK_BULGE = 0.0
RING_BULGE = 0.0
RING_BACK_SETBACK = 6.5
RING_SEAM_SETBACK = 8.0  # radius 3.8 clears the 3.5 roll; keeps 1.6 round the z 10 inserts
RING_BACK_Y = 37.0  # behind the bore, under the pad root: the taper fits both corners
LINER_END_GAP = 0.15
LINER_OUTER_GAP = 0.05
RIB_OVERLAP = 0.05


def column_profile(spec: Params, grow: float, z: float = 0.0) -> Face:
    """Column outline grown by `grow`. Arcs offset to arcs, so this offset is exact."""
    face = RectangleRounded(spec.column_w + 2 * grow, spec.column_d + 2 * grow,
                            spec.column_r + grow)
    return Pos(0, 0, z) * face


def pad_end_y(spec: Params) -> float:
    return spec.mf_y + spec.mf_size / 2


def _mirror(right: list[tuple[float, float]], setbacks: list[float]):
    """Right side listed from the seam up; the left side mirrors it back down."""
    vertices = right + [(-x, y) for x, y in reversed(right)]
    return vertices, setbacks + setbacks[::-1]


def deck_plan(spec: Params, sign: float) -> object:
    """Deck outline of one shell: the front half (sign -1), or the back half with neck and pad."""
    hw, hd, gap = spec.outer_half_w, spec.outer_half_d, spec.seam_gap / 2
    if sign < 0:
        vertices = [(hw, -hd), (hw, -gap), (-hw, -gap), (-hw, -hd)]
        setbacks = [spec.front_setback, spec.seam_setback, spec.seam_setback, spec.front_setback]
        return soft_outline(vertices, setbacks)
    pad = spec.mf_size / 2
    right = [(hw, gap), (hw, spec.taper_start_y), (pad, spec.neck_y), (pad, pad_end_y(spec))]
    return soft_outline(*_mirror(right, [spec.seam_setback, spec.taper_setback,
                                         spec.taper_setback, spec.mf_setback]))


def ring_plan(spec: Params, sign: float) -> object:
    """The lower ring, ring_inset inside the deck. At the back it follows the deck taper and
    closes behind the bore, so the MF pad overhangs it."""
    inset = spec.ring_inset
    hw, hd, gap = spec.outer_half_w - inset, spec.outer_half_d - inset, spec.seam_gap / 2
    seam = RING_SEAM_SETBACK
    if sign < 0:
        vertices = [(hw, -hd), (hw, -gap), (-hw, -gap), (-hw, -hd)]
        return soft_outline(vertices, [spec.front_setback - inset, seam, seam,
                                       spec.front_setback - inset])
    start = Vector(spec.outer_half_w, spec.taper_start_y)
    along = (Vector(spec.mf_size / 2, spec.neck_y) - start).normalized()
    inward = Vector(-along.Y, along.X)
    base = start + inward * inset
    side = base + along * ((hw - base.X) / along.X)
    back = base + along * ((RING_BACK_Y - base.Y) / along.Y)
    right = [(hw, gap), (side.X, side.Y), (back.X, back.Y)]
    return soft_outline(*_mirror(right, [seam, spec.taper_setback - inset, RING_BACK_SETBACK]))


def shell_skins(spec: Params, sign: float) -> tuple[Part, Part]:
    """(deck, ring) of one shell, before they are fused and cut."""
    deck_h = spec.collar_h - spec.deck_z
    deck = Pos(0, 0, spec.deck_z) * g2_skin(deck_plan(spec, sign), deck_h, DECK_UNDER,
                                            DECK_TOP, DECK_BULGE)
    ring_h = spec.deck_z + DECK_UNDER.setback
    ring = g2_skin(ring_plan(spec, sign), ring_h, RING_BOTTOM, RING_TOP, RING_BULGE)
    return deck, ring


def cavity(spec: Params) -> Part:
    """Bore with a 45 degree lip at each end. The liner snaps in behind the lips."""
    lip = spec.bore_offset - spec.lip_reach
    height = spec.collar_h
    through = extrude(column_profile(spec, lip, -1), amount=height + 2)
    sections = [
        column_profile(spec, lip, spec.lip_h),
        column_profile(spec, spec.bore_offset, spec.lip_h + spec.lip_reach),
        column_profile(spec, spec.bore_offset, height - spec.lip_h - spec.lip_reach),
        column_profile(spec, lip, height - spec.lip_h),
    ]
    return through + loft(sections, ruled=True)


def blank(spec: Params, sign: float) -> Part:
    deck, ring = shell_skins(spec, sign)
    return (deck + ring) - cavity(spec)


def seam_points(spec: Params) -> list[tuple[float, float]]:
    return [(sign * spec.seam_x, z) for z in spec.seam_z for sign in (-1.0, 1.0)]


def seam_inserts(spec: Params) -> list[SeatedInsert]:
    """One insert per seam screw, pressed into the back shell from its seam face."""
    face_y = spec.seam_gap / 2
    return [SeatedInsert(f"seam ({x:+.1f}, z {z:g})", M3_INSERT, Vector(x, face_y, z),
                         Vector(0, 1, 0), spec.seam_tip_room, M3_CLEAR)
            for x, z in seam_points(spec)]


def seam_grip_open(spec: Params) -> float:
    """Under the head to the back seam face, seam open: washer, front shell, seam gap."""
    return M3_WASHER.t + (-spec.seam_gap / 2 - spec.seam_seat_y) + spec.seam_gap


def seam_screw_length(spec: Params) -> float:
    need = seam_grip_open(spec) + INSERT_SINK + INSERT_ENGAGE + M3_SHCS.length_tol
    return M3_SHCS.stock_at_least(need)


def mount_face(spec: Params) -> MountFace:
    """Collar MF: the port nut slides in from the bore toward +Y, so the mouth is -Y."""
    centre = Vector(0, spec.mf_y, spec.collar_h)
    reach = spec.mf_y - bore_wall_y(spec, 0.0) + 1.5
    return MountFace("collar MF", centre, Vector(0, -1, 0), reach)


def bore_wall_y(spec: Params, x: float) -> float:
    """Y of the back bore wall at plan x."""
    flat = spec.bore_half_w - spec.bore_r
    if abs(x) <= flat:
        return spec.bore_half_d
    centre_y = spec.bore_half_d - spec.bore_r
    return centre_y + sqrt(max(spec.bore_r ** 2 - (abs(x) - flat) ** 2, 0.0))


def spine_root_y(spec: Params) -> float:
    """Where the pad leaves the ring: just behind the ring's back wall."""
    return spec.outer_half_d - spec.ring_inset + 1.0


def build_shells(spec: Params) -> tuple[Part, Part]:
    front, back = blank(spec, -1.0), blank(spec, 1.0)
    head_d, _ = M3_WASHER_BORE
    seat_y = spec.seam_seat_y
    for x, z in seam_points(spec):
        front -= rod(M3_CLEAR, -seat_y + 1, Vector(x, seat_y - 0.5, z), Vector(0, 1, 0))
        front -= teardrop_rod(head_d, 40, Vector(x, seat_y, z), Vector(0, -1, 0),
                              Vector(0, 0, -1))
    for insert in seam_inserts(spec):
        for cutter in insert.cutters():
            back -= cutter
    for cutter in mount_face(spec).cutters(spec):
        back -= cutter
    return named(front, "collar_front"), named(back, "collar_back")


def _half(body: Part, sign: float, spec: Params) -> Part:
    """The front (sign -1) or back half of a body split by the seam gap."""
    gap, reach = spec.seam_gap / 2, 200
    return body & box_from(-reach, sign * gap, -1, reach, sign * reach, spec.collar_h + 1)


def build_liners(spec: Params) -> tuple[Part, Part]:
    """Liner halves: shell cavity minus clearances, V ribs at rib_pitch."""
    lip = spec.bore_offset - spec.lip_reach
    outer = spec.bore_offset - LINER_OUTER_GAP
    z0 = spec.lip_h + LINER_END_GAP
    z1 = spec.collar_h - spec.lip_h - LINER_END_GAP
    cone = outer - lip
    sections = [column_profile(spec, lip, z0), column_profile(spec, outer, z0 + cone),
                column_profile(spec, outer, z1 - cone), column_profile(spec, lip, z1)]
    valley = spec.tpu_fit + spec.rib_h
    ring = loft(sections, ruled=True) - extrude(column_profile(spec, valley, z0 - 1),
                                                amount=z1 - z0 + 2)
    for z in rib_bases(spec, z0, z1):
        ring += rib(spec, z)
    front = _half(ring, -1.0, spec)
    back = _half(ring, 1.0, spec)
    return named(front, "liner_front"), named(back, "liner_back")


def rib(spec: Params, z: float) -> Part:
    """One 45 degree V rib ring, base sunk RIB_OVERLAP into the liner so the two fuse."""
    base = spec.tpu_fit + spec.rib_h + RIB_OVERLAP
    rise = spec.rib_h + RIB_OVERLAP
    bottom = z - RIB_OVERLAP
    band = extrude(column_profile(spec, base, bottom), amount=2 * rise)
    waist = loft([column_profile(spec, base, bottom),
                  column_profile(spec, spec.tpu_fit, bottom + rise),
                  column_profile(spec, base, bottom + 2 * rise)], ruled=True)
    return band - waist


def rib_bases(spec: Params, z0: float, z1: float) -> list[float]:
    """Rib bottoms, centred in the liner height, one per rib_pitch."""
    span = 2 * spec.rib_h
    count = int((z1 - z0 - span - 2.0) // spec.rib_pitch) + 1
    used = (count - 1) * spec.rib_pitch + span
    first = (z0 + z1 - used) / 2
    return [first + k * spec.rib_pitch for k in range(count)]


# --- Checks ------------------------------------------------------------------------------------


def _skin_checks(spec: Params, sign: float, label: str) -> list[Check]:
    """Each skin on its own: every edge G2 except the bed lead crease on the top face."""
    deck, ring = shell_skins(spec, sign)
    on_bed = lambda centre: abs(centre.Z - spec.collar_h) < 1e-4  # noqa: E731
    rows = skin_rows(deck, f"{label} deck", on_bed)
    rows += skin_rows(ring, f"{label} ring", lambda _centre: False)
    jump = max(curvature_jumps(deck_plan(spec, sign)) + curvature_jumps(ring_plan(spec, sign)))
    rows.append(Check(f"{label} plan outline curvature jump (G2)", jump <= 1e-3,
                      f"{jump:.2e} /mm (max 1e-3)"))
    return rows


def checks_front(part: Part, spec: Params) -> list[Check]:
    rows = [one_solid(part)]
    rows += overhang_rows(print_orientation(part, PRINT["collar_front"]), "collar_front")
    rows += _skin_checks(spec, -1.0, "collar_front")
    head_d, head_h = M3_WASHER_BORE
    for x, z in seam_points(spec):
        axis = Vector(0, -1, 0)
        origin = Vector(x, spec.seam_seat_y, z)
        probe = teardrop_ring(head_d, PETG_WALL, head_h + 0.3, origin, axis, Vector(0, 0, -1))
        rows.append(encloses(part, probe, f"seam head bore ({x:+.1f}, z {z:g}) wall"))
        shank = ring_probe(Vector(x, -spec.seam_gap / 2 - 0.4, z), axis, M3_CLEAR / 2,
                           PETG_WALL, 0.0, -spec.seam_gap / 2 - spec.seam_seat_y - 0.8)
        rows.append(encloses(part, shank, f"seam shank ({x:+.1f}, z {z:g}) wall"))
        seat = annulus_footprint(origin, Vector(0, 1, 0), M3_WASHER.outer, M3_WASHER.inner)
        rows.append(bearing_row(f"seam ({x:+.1f}, z {z:g}) washer", PRELOAD_N["m3"],
                                contact_area(part, seat)))
    return rows


def checks_back(part: Part, spec: Params) -> list[Check]:
    rows = [one_solid(part)]
    rows += overhang_rows(print_orientation(part, PRINT["collar_back"]), "collar_back")
    rows += _skin_checks(spec, 1.0, "collar_back")
    for insert in seam_inserts(spec):
        rows += insert.checks(part)
    rows.append(insert_shear_row("seam", M3_INSERT, PRELOAD_N["m3"]))
    rows += mount_face(spec).checks(part, spec)
    return rows


def checks_liner(part: Part, spec: Params) -> list[Check]:
    rows = [one_solid(part)]
    rows += overhang_rows(print_orientation(part, PRINT[part.label]), part.label)
    tpu = MATERIALS["TPU95A"]["wall_min"]
    rows.append(at_least(f"{part.label} wall", spec.liner_t, tpu))
    return rows


def checks_pair(front: Part, back: Part, liners: tuple[Part, Part], column: Part,
                spec: Params) -> list[Check]:
    rows = [at_least("seam gap between the shells", front.distance_to(back),
                     spec.seam_gap - 0.05)]
    for liner in liners:
        gap = liner.distance_to(column)
        rows.append(Check(f"{liner.label} rib tips to column", abs(gap - spec.tpu_fit) <= 0.02,
                          f"{gap:.3f} mm (fit {spec.tpu_fit} +- 0.02)"))
    return rows
