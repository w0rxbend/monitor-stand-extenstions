"""Collar: two PETG shells round the column, and two TPU 95A grip liners.

The PETG shell carries the clamp load. TPU 95A loses 30-38 % of its compression to set
(Essentium, BASF TDS), so the liner is only a 2 mm grip layer with 45 degree V ribs.
The back shell's top runs from the ring into the mount face (MF) as one flat plane: plan
outline = lines and G2 quintic corners, side view = an elliptic scoop under the spine.
Four seam screws run inside the wall from the -Y face (the monitor side) into nuts in the
back shell. Every nut slot opens into the bore, behind the liner.
"""

from math import sqrt

from build123d import (
    EllipticalCenterArc,
    Face,
    Line,
    Part,
    Plane,
    Pos,
    RectangleRounded,
    Vector,
    Wire,
    chamfer,
    extrude,
    loft,
    make_face,
)

from holder.common import (
    CaptiveNut,
    Check,
    at_least,
    box_from,
    encloses,
    lead_in,
    named,
    one_solid,
    overhang_rows,
    print_orientation,
    rod,
    teardrop_ring,
    teardrop_rod,
)
from holder.form import curvature_jumps, soft_outline, soft_rect
from holder.mount import MountFace, m3_square
from holder.params import (
    M3_CLEAR,
    M3_NUT,
    M3_WASHER_BORE,
    MATERIALS,
    PETG_WALL,
    Params,
)
from holder.stacks import seam_design

PRINT = {
    "collar_front": (0, 0, 1),
    "collar_back": (0, 0, 1),
    "liner_front": (0, 0, -1),
    "liner_back": (0, 0, -1),
}
MATERIAL = {"collar_front": "PETG", "collar_back": "PETG",
            "liner_front": "TPU95A", "liner_back": "TPU95A"}
LOOP_CHAMFER = 1.0
SEAM_CHAMFER = 0.6
GROOVE_HALF_W = 0.3
GROOVE_DEPTH = 0.36  # steeper than 45 degrees: the groove prints on the bed face
LINER_END_GAP = 0.15
LINER_OUTER_GAP = 0.05
RIB_OVERLAP = 0.05


def column_profile(spec: Params, grow: float, z: float = 0.0) -> Face:
    """Column outline grown by `grow`. Arcs offset to arcs, so this offset is exact."""
    face = RectangleRounded(spec.column_w + 2 * grow, spec.column_d + 2 * grow,
                            spec.column_r + grow)
    return Pos(0, 0, z) * face


def outside_profile(spec: Params, point: Vector, grow: float) -> bool:
    """Plan point lies outside the column outline grown by `grow`."""
    radius = spec.column_r + grow
    qx = abs(point.X) - (spec.column_w / 2 + grow - radius)
    qy = abs(point.Y) - (spec.column_d / 2 + grow - radius)
    outside = sqrt(max(qx, 0) ** 2 + max(qy, 0) ** 2) + min(max(qx, qy), 0) - radius
    return outside > 0


def pad_end_y(spec: Params) -> float:
    return spec.mf_y + spec.mf_size / 2


def plan(spec: Params) -> Face:
    """Ring, tapered neck, and MF pad as one outline: lines joined by G2 corners."""
    hw, hd, pad = spec.outer_half_w, spec.outer_half_d, spec.mf_size / 2
    end = pad_end_y(spec)
    right = [(hw, -hd), (hw, spec.taper_start_y), (pad, spec.neck_y), (pad, end)]
    vertices = right + [(-x, y) for x, y in reversed(right)]
    corner = [spec.front_setback, spec.taper_setback, spec.taper_setback, spec.mf_setback]
    return make_face(soft_outline(vertices, corner + corner[::-1])).faces()[0]


def side_profile(spec: Params) -> Part:
    """Side view: full height over the ring, an elliptic scoop under the spine."""
    start, end, top = spec.outer_half_d, pad_end_y(spec), spec.collar_h + 5
    scoop = EllipticalCenterArc((end, 0), end - start, spec.scoop_end_z, start_angle=90,
                                arc_size=90)
    far = -spec.outer_half_d - 10
    edges = [
        Line((far, 0), (start, 0)), scoop,
        Line((end, spec.scoop_end_z), (end + 10, spec.scoop_end_z)),
        Line((end + 10, spec.scoop_end_z), (end + 10, top)),
        Line((end + 10, top), (far, top)), Line((far, top), (far, 0)),
    ]
    half = spec.outer_half_w + 10
    face = Plane.YZ.offset(-half).location * make_face(Wire(edges)).faces()[0]
    return extrude(face, amount=2 * half)


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


def blank(spec: Params) -> Part:
    body = extrude(plan(spec), amount=spec.collar_h) & side_profile(spec)
    return body - cavity(spec)


def _half(spec: Params, body: Part, sign: float) -> Part:
    gap = spec.seam_gap / 2
    reach = 200
    keep = box_from(-reach, sign * gap, -1, reach, sign * reach, spec.collar_h + 1)
    return body & keep


def _dress(spec: Params, shell: Part, sign: float) -> Part:
    """Chamfer the seam outline, then the scoop edges, then both plan loops."""
    seam_y = sign * spec.seam_gap / 2
    seam = [edge for edge in shell.edges()
            if abs(edge.center().Y - seam_y) < 1e-4 and abs(edge.center().X) > spec.bore_half_w + 1
            and 0.5 < edge.center().Z < spec.collar_h - 0.5]
    shell = chamfer(seam, SEAM_CHAMFER)
    scoop_edges = _scoop_edges(spec, shell)
    if scoop_edges:
        shell = chamfer(scoop_edges, LOOP_CHAMFER)
    loops = [edge for edge in shell.edges()
             if (edge.center().Z < 1e-4 or edge.center().Z > spec.collar_h - 1e-4)
             and outside_profile(spec, edge.center(), spec.bore_offset + 0.3)]
    return chamfer(loops, LOOP_CHAMFER)


def _scoop_edges(spec: Params, shell: Part) -> list:
    """Edges round the down-facing scoop face, except its tangent join with the bottom."""
    edges = []
    for face in shell.faces():
        if face.center().Y <= spec.outer_half_d or face.normal_at().Z > -0.05:
            continue
        if face.center().Z < 1e-3:
            continue
        edges += [edge for edge in face.edges() if edge.center().Z > 0.2]
    return edges


def seam_points(spec: Params) -> list[tuple[float, float]]:
    return [(sign * spec.seam_x, z) for z in spec.seam_z for sign in (-1.0, 1.0)]


def seam_nuts(spec: Params) -> list[CaptiveNut]:
    seat_y = spec.seam_gap / 2 + spec.seam_back_skin
    nuts = []
    for x, z in seam_points(spec):
        inward = Vector(-1 if x > 0 else 1, 0, 0)
        reach = abs(x) - spec.bore_half_w + 1.5
        nuts.append(CaptiveNut(f"seam nut ({x:+.1f}, z {z:g})", M3_NUT, Vector(x, seat_y, z),
                               Vector(0, 1, 0), inward, reach))
    return nuts


def seam_tip_floor(spec: Params) -> float:
    """Y of the seam tip bore floor in the back shell: long screw, seam closed, plus 1.0."""
    stack = seam_design(spec)
    seat = spec.seam_gap / 2 + spec.seam_back_skin
    return seat + stack.length + stack.screw.length_tol - stack.grip_closed + 1.0


def mount_face(spec: Params) -> MountFace:
    """Collar MF: nuts slide in from the bore toward +Y, so the mouth is -Y."""
    centre = Vector(0, spec.mf_y, spec.collar_h)
    reach = {}
    for dx, dy in [*m3_square(spec), (0.0, 0.0)]:
        reach[(dx, dy)] = spec.mf_y + dy - bore_wall_y(spec, dx) + 1.5
    return MountFace("collar MF", centre, Vector(0, -1, 0), reach)


def bore_wall_y(spec: Params, x: float) -> float:
    """Y of the back bore wall at plan x."""
    flat = spec.bore_half_w - spec.bore_r
    if abs(x) <= flat:
        return spec.bore_half_d
    centre_y = spec.bore_half_d - spec.bore_r
    return centre_y + sqrt(max(spec.bore_r ** 2 - (abs(x) - flat) ** 2, 0.0))


def mf_groove(spec: Params) -> Part:
    """0.6 wide V tracing the 40 x 40 foot outline on the top face."""
    top, size, setback = spec.collar_h, spec.mf_size, spec.mf_setback
    centre = (0, spec.mf_y)

    def square(grow: float, z: float) -> Face:
        return Pos(0, 0, z) * soft_rect(size + 2 * grow, size + 2 * grow, setback + grow, centre)

    outer = loft([square(GROOVE_HALF_W, top), square(0, top - GROOVE_DEPTH)], ruled=True)
    inner = loft([square(-GROOVE_HALF_W, top), square(0, top - GROOVE_DEPTH)], ruled=True)
    return outer - inner


def build_shells(spec: Params) -> tuple[Part, Part]:
    body = blank(spec)
    front = _dress(spec, _half(spec, body, -1.0), -1.0)
    back = _dress(spec, _half(spec, body, 1.0), 1.0)
    head_d, _ = M3_WASHER_BORE
    seat_y = spec.seam_seat_y
    floor_y = seam_tip_floor(spec)
    for x, z in seam_points(spec):
        front -= rod(M3_CLEAR, -seat_y + 1, Vector(x, seat_y - 0.5, z), Vector(0, 1, 0))
        front -= teardrop_rod(head_d, 40, Vector(x, seat_y, z), Vector(0, -1, 0),
                              Vector(0, 0, -1))
        back -= rod(M3_CLEAR, floor_y, Vector(x, 0, z), Vector(0, 1, 0))
        back -= lead_in(M3_CLEAR, Vector(x, spec.seam_gap / 2, z), Vector(0, 1, 0))
    for nut in seam_nuts(spec):
        for cutter in nut.cutters():
            back -= cutter
    for cutter in mount_face(spec).cutters(spec):
        back -= cutter
    back -= mf_groove(spec)
    return named(front, "collar_front"), named(back, "collar_back")


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
    front = _half(spec, ring, -1.0)
    back = _half(spec, ring, 1.0)
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


def checks_front(part: Part, spec: Params) -> list[Check]:
    rows = [one_solid(part)]
    rows += overhang_rows(print_orientation(part, PRINT["collar_front"]), "collar_front")
    rows.append(_g2_row(spec))
    head_d, head_h = M3_WASHER_BORE
    for x, z in seam_points(spec):
        axis = Vector(0, -1, 0)
        origin = Vector(x, spec.seam_seat_y, z)
        probe = teardrop_ring(head_d, PETG_WALL, head_h + 0.3, origin, axis, Vector(0, 0, -1))
        rows.append(encloses(part, probe, f"seam head bore ({x:+.1f}, z {z:g}) wall"))
        shank = _ring(Vector(x, -spec.seam_gap / 2 - 0.4, z), axis, M3_CLEAR / 2, PETG_WALL,
                      -spec.seam_gap / 2 - spec.seam_seat_y - 0.8)
        rows.append(encloses(part, shank, f"seam shank ({x:+.1f}, z {z:g}) wall"))
    return rows


def _ring(origin: Vector, axis: Vector, radius: float, wall: float, length: float) -> Part:
    from holder.common import ring_probe

    return ring_probe(origin, axis, radius, wall, 0.0, length)


def _g2_row(spec: Params) -> Check:
    jump = max(curvature_jumps(plan(spec).outer_wire()))
    return Check("plan outline curvature jump (G2)", jump <= 1e-3, f"{jump:.2e} /mm (max 1e-3)")


def checks_back(part: Part, spec: Params) -> list[Check]:
    rows = [one_solid(part)]
    rows += overhang_rows(print_orientation(part, PRINT["collar_back"]), "collar_back")
    rows.append(_g2_row(spec))
    for nut in seam_nuts(spec):
        rows += nut.checks(part)
        for name, probe in nut.wall_probes(PETG_WALL, M3_CLEAR):
            rows.append(encloses(part, probe, f"{name} >= {PETG_WALL}"))
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


def shell_axis_rows(spec: Params) -> list[Check]:
    """Seam screw clearances: wall from the shank to the bore at the seat."""
    return [at_least("seam screw to bore wall", spec.seam_x - M3_CLEAR / 2 - spec.bore_half_w,
                     PETG_WALL)]

