"""Shared helpers for the part modules: the part contract, cutters, and measurements.

Measure on real geometry, never on bounding boxes (OCCT pads them):
circular edges and cylindrical faces for hole axes and diameters, solid-to-solid
distance for walls, boolean intersection volume for clashes.
"""

import math
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass
from itertools import combinations
from typing import NamedTuple

from build123d import (
    Align,
    Axis,
    BoundBox,
    Box,
    Compound,
    Cylinder,
    Edge,
    Face,
    Part,
    Plane,
    Polygon,
    Shape,
    Vector,
    extrude,
)
from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCP.BRepTools import BRepTools
from OCP.GeomAbs import GeomAbs_Circle, GeomAbs_Cylinder

from holder.params import MATERIALS, Params

VOLUME_EPS = 1e-6  # mm3: an intersection at or below this counts as zero
TOUCH_EPS = 1e-3  # mm: solids closer than this touch
RADIUS_TOL = 1e-3  # mm: a circle or cylinder matches a radius within this
COAXIAL_TOL = 0.01  # mm: two axes closer than this count as coaxial
ANGLE_TOL = 0.1  # degrees: two directions within this count as parallel
AF_TOL = 0.01  # mm: a hex pocket's across-flats matches its design within this
HEX_WINDOW = 0.5  # mm: a flat further than this from af/2 off its hole axis is not that pocket's
NEAR_MARGIN = 1.0  # mm: cavity_deviation reports deviations exactly up to this

Vec3 = tuple[float, float, float]


# --- Check results -------------------------------------------------------------------------------


class Check(NamedTuple):
    """One check result: (name, ok, measured)."""

    name: str
    ok: bool
    measured: str


def at_least(name: str, value: float, minimum: float, unit: str = "mm") -> Check:
    return Check(name, value >= minimum, f"{value:.3f} {unit} (min {minimum:g})")


def at_most(name: str, value: float, maximum: float, unit: str = "mm") -> Check:
    return Check(name, value <= maximum, f"{value:.3g} {unit} (max {maximum:g})")


def count_is(name: str, found: int, expected: int) -> Check:
    return Check(name, found == expected, f"{found} (expected {expected})")


def single_solid(part: Shape) -> Check:
    """The part is exactly one valid solid."""
    n = len(part.solids())
    valid = part.is_valid
    return Check("one valid solid", n == 1 and valid, f"{n} solid(s), valid={valid}")


# --- Part contract (see holder.registry) ---------------------------------------------------------


@dataclass(frozen=True)
class PartSpec:
    """One printed part. See the holder.registry docstring for the contract."""

    name: str  # NAME: unique, file stem in out/
    material: str  # MATERIAL: key into params.MATERIALS
    print_normal: Vec3  # PRINT: outward normal of the bed face, assembly coordinates
    build: Callable[[Params], Part]  # -> one solid in assembly coordinates
    checks: Callable[[Part, Params], list[Check]]
    assembly: str = "holder"  # parts of one assembly are clash-checked against each other

    def __post_init__(self) -> None:
        if self.material not in MATERIALS:
            raise ValueError(f"{self.name}: unknown material {self.material!r}")
        if Vector(*self.print_normal).length < 1e-9:
            raise ValueError(f"{self.name}: print_normal must be non-zero")


# --- Cutters -------------------------------------------------------------------------------------


def plane_at(origin: Iterable[float], axis: Iterable[float], x_dir: Iterable[float] | None = None):
    """A plane at `origin` whose normal is `axis`. `x_dir` must be perpendicular to `axis`;
    without it, the plane's x is the global X (or Y, if `axis` is along X) made perpendicular."""
    z = Vector(*axis).normalized()
    if x_dir is None:
        ref = Vector(1, 0, 0) if abs(z.X) < 0.9 else Vector(0, 1, 0)
        x = (ref - z * ref.dot(z)).normalized()
    else:
        x = Vector(*x_dir).normalized()
        if abs(x.dot(z)) > 1e-9:
            raise ValueError("x_dir must be perpendicular to axis")
    return Plane(origin=Vector(*origin), x_dir=x, z_dir=z)


def rod(d: float, length: float, origin=(0, 0, 0), axis=(0, 0, 1)) -> Part:
    """Cylinder of diameter `d` from `origin` for `length` along `axis` (a hole cutter)."""
    cyl = Cylinder(d / 2, length, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return plane_at(origin, axis).location * cyl


def hex_pocket(
    af: float,
    depth: float,
    origin=(0, 0, 0),
    axis=(0, 0, 1),
    slot_dir: Iterable[float] | None = None,
    slot_len: float = 0.0,
) -> Part:
    """Hex nut pocket cutter: `af` across flats, `depth` long from `origin` along `axis`.

    Two flats run parallel to `slot_dir` (or to the plane x of `plane_at` without a slot).
    With `slot_len` > 0, a channel `af` wide runs from the hex center along `slot_dir`
    for `slot_len`, so the nut slides in from a side face: make it reach past that face.
    """
    if af <= 0 or depth <= 0 or slot_len < 0:
        raise ValueError("hex_pocket: af and depth must be > 0, slot_len >= 0")
    if slot_len > 0 and slot_dir is None:
        raise ValueError("hex_pocket: slot_len needs slot_dir")
    corner = af / math.sqrt(3)
    # Corners at 0, 60, ... degrees: the flats sit at local y = +-af/2, parallel to local x.
    corners = [
        (corner * math.cos(math.radians(a)), corner * math.sin(math.radians(a)))
        for a in range(0, 360, 60)
    ]
    cutter = extrude(Polygon(*corners, align=None), amount=depth)
    if slot_len > 0:
        cutter += Box(slot_len, af, depth, align=(Align.MIN, Align.CENTER, Align.MIN))
    return plane_at(origin, axis, slot_dir).location * cutter


def captive_nut(
    face_point: Iterable[float],
    into: Iterable[float],
    hole_d: float,
    through: float,
    af: float,
    nut_h: float,
    nut_depth: float,
    slot_dir: Iterable[float],
    slot_len: float,
) -> list[Part]:
    """Cutters for a bolt that enters a face and threads into a captive nut, in the side-slot
    form the block uses: [through-hole, hex pocket].

    The hole starts at `face_point` and runs `through` mm along `into` (1 mm overshoot at
    both ends). The pocket's top sits `nut_depth` below the face (that wall takes the bolt
    load), it is `nut_h` deep, and its slot runs `slot_len` along `slot_dir` to a side face.
    """
    d = Vector(*into).normalized()
    top = Vector(*face_point)
    hole = rod(hole_d, through + 2, origin=top - d, axis=d)
    pocket = hex_pocket(
        af, nut_h, origin=top + d * nut_depth, axis=d, slot_dir=slot_dir, slot_len=slot_len
    )
    return [hole, pocket]


def cavity_pieces(envelope: Shape, cutters: Iterable[Shape]) -> list[Part]:
    """The voids `cutters` leave in `envelope`, as disjoint pieces: each cutter clipped to the
    envelope, minus the cutters after it ([hole, nut] -> [hole outside the pocket, pocket]).
    This is the form wall_around and cavity_check expect."""
    cutters = list(cutters)
    pieces = []
    for k, cutter in enumerate(cutters):
        piece = cutter & envelope
        for later in cutters[k + 1 :]:
            piece -= later
        pieces.append(piece)
    return pieces


def cut(envelope: Part, cutters: Iterable[Shape]) -> Part:
    """`envelope` minus every cutter."""
    part = envelope
    for cutter in cutters:
        part -= cutter
    return part


# --- Measuring -----------------------------------------------------------------------------------


class Circle(NamedTuple):
    radius: float
    axis: Axis  # position on the axis (the center, for an edge), direction = normal


def _parallel(a: Vector, b: Vector, angle_tol: float = ANGLE_TOL) -> bool:
    return abs(a.normalized().dot(b.normalized())) >= math.cos(math.radians(angle_tol))


def _matches(c: Circle, radius: float | None, direction: Iterable[float] | None) -> bool:
    if radius is not None and abs(c.radius - radius) > RADIUS_TOL:
        return False
    return direction is None or _parallel(c.axis.direction, Vector(*direction))


def circles(shape: Shape, radius: float | None = None, direction=None) -> list[Circle]:
    """Circular edges (full circles and arcs), optionally filtered by radius and normal."""
    found = []
    for edge in shape.edges():
        curve = BRepAdaptor_Curve(edge.wrapped)
        if curve.GetType() == GeomAbs_Circle:
            circ = curve.Circle()
            c = Circle(circ.Radius(), Axis(circ.Axis()))
            if _matches(c, radius, direction):
                found.append(c)
    return found


def cylinders(shape: Shape, radius: float | None = None, direction=None) -> list[Circle]:
    """Cylindrical faces, optionally filtered by radius and axis direction."""
    found = []
    for face in shape.faces():
        surface = BRepAdaptor_Surface(face.wrapped)
        if surface.GetType() == GeomAbs_Cylinder:
            cyl = surface.Cylinder()
            c = Circle(cyl.Radius(), Axis(cyl.Axis()))
            if _matches(c, radius, direction):
                found.append(c)
    return found


def planar_faces(shape: Shape, normal: Iterable[float]) -> list[Face]:
    """Planar faces whose normal is parallel (either sense) to `normal`."""
    n = Vector(*normal)
    return [f for f in shape.faces() if f.is_planar and _parallel(f.normal_at(), n)]


def axis_offset(a: Axis, b: Axis) -> float:
    """Distance between two parallel axes; infinity if they are not parallel."""
    if not _parallel(a.direction, b.direction):
        return math.inf
    return (b.position - a.position).cross(a.direction.normalized()).length


def coaxial(a: Axis, b: Axis, tol: float = COAXIAL_TOL) -> bool:
    return axis_offset(a, b) <= tol


def unique_axes(axes: Iterable[Axis], tol: float = COAXIAL_TOL) -> list[Axis]:
    """Merge coaxial axes (a hole is often split into two half-cylinder faces)."""
    result: list[Axis] = []
    for ax in axes:
        if not any(coaxial(ax, kept, tol) for kept in result):
            result.append(ax)
    return result


def hole_axes(shape: Shape, diameter: float, direction=None) -> list[Axis]:
    """One axis per hole of `diameter`, from the real cylindrical faces."""
    return unique_axes(c.axis for c in cylinders(shape, diameter / 2, direction))


def coaxial_check(
    name: str,
    axes_a: list[Axis],
    axes_b: list[Axis],
    expected: int | None = None,
    tol: float = COAXIAL_TOL,
) -> Check:
    """Every axis in `axes_a` has a coaxial partner in `axes_b` (and the counts match)."""
    counts = f"{len(axes_a)}/{len(axes_b)} axes"
    if not axes_a or len(axes_a) != len(axes_b) or (expected and len(axes_a) != expected):
        want = f", expected {expected}" if expected else ""
        return Check(name, False, f"{counts}{want}")
    worst = max(min(axis_offset(a, b) for b in axes_b) for a in axes_a)
    return Check(name, worst <= tol, f"{counts}, worst offset {worst:.4f} mm (max {tol:g})")


def hex_flat_widths(part: Shape, axis: Axis, af: float, window: float = HEX_WINDOW) -> list[float]:
    """Across-flats widths of the hex pocket around `axis`, measured on the real faces.

    A flat is a planar face parallel to the axis whose distance to the axis line is within
    `window` of af/2. Each pair of parallel flats on opposite sides of the axis gives one
    width (plane to plane). A closed hex gives 3 widths, a side-slot pocket 1 (the two
    flats that run along the slot). No pocket, or one off by more than `window`, gives [].
    """
    d = axis.direction.normalized()
    line = Edge.make_line(axis.position - d * 1000, axis.position + d * 1000)
    flats = []
    for face in part.faces():
        if not face.is_planar:
            continue
        n = face.normal_at()
        if abs(n.dot(d)) > 1e-6 or abs(face.distance_to(line) - af / 2) > window:
            continue
        flats.append((n, face.center()))
    widths = []
    for (n1, c1), (n2, c2) in combinations(flats, 2):
        if not _parallel(n1, n2):
            continue
        s1 = (c1 - axis.position).dot(n1)
        s2 = (c2 - axis.position).dot(n1)
        if s1 * s2 < 0:
            widths.append(abs(s1 - s2))
    return widths


def hex_pocket_checks(
    label: str, part: Shape, axes: list[Axis], af: float, expected: int
) -> list[Check]:
    """How many of the hole `axes` have a hex pocket around them, and its across-flats."""
    widths = [hex_flat_widths(part, ax, af) for ax in axes]
    found = sum(bool(w) for w in widths)
    checks = [count_is(f"{label} nut pockets", found, expected)]
    if expected:
        flat = [w for ws in widths for w in ws]
        worst = max((abs(w - af) for w in flat), default=math.inf)
        checks.append(
            Check(
                f"{label} nut pocket AF",
                bool(flat) and worst <= AF_TOL,
                f"{len(flat)} flat pair(s), worst {worst:.3f} mm off {af:g} (max {AF_TOL:g})",
            )
        )
    return checks


def min_distance(a: Shape, b: Shape) -> float:
    """Boundary-to-boundary distance. A solid fully inside another is NOT zero here:
    use intersection_volume for overlap."""
    return a.distance_to(b)


def _lies_on(face: Face, cavity: Shape, tol: float) -> bool:
    """All vertices and edge midpoints of `face` sit on the cavity boundary."""
    points = [v.center() for v in face.vertices()] + [e.position_at(0.5) for e in face.edges()]
    return bool(points) and all(cavity.distance_to(pt) <= tol for pt in points)


def _box_gap(a: BoundBox, b: BoundBox) -> float:
    """Distance between two bounding boxes (0 if they overlap). OCCT pads boxes, so this is a
    lower bound of the distance between the shapes inside: used only to skip faces that
    cannot matter, never as a measurement."""
    d = [
        max(lo_a - hi_b, lo_b - hi_a, 0.0)
        for lo_a, hi_a, lo_b, hi_b in (
            (a.min.X, a.max.X, b.min.X, b.max.X),
            (a.min.Y, a.max.Y, b.min.Y, b.max.Y),
            (a.min.Z, a.max.Z, b.min.Z, b.max.Z),
        )
    ]
    return math.hypot(*d)


def near_faces(shape: Shape, other: Shape, within: float) -> Compound | None:
    """The faces of `shape` that may lie within `within` of `other` (box prefilter: it keeps
    every face that does, and maybe a few more), as one compound; None if there are none."""
    box = other.bounding_box()
    faces = [f for f in shape.faces() if _box_gap(f.bounding_box(), box) <= within]
    return Compound(faces) if faces else None


def wall_around(
    part: Shape,
    voids: Shape | Iterable[Shape],
    touch: float = TOUCH_EPS,
    open_to: Shape | None = None,
) -> float:
    """Thinnest wall between one cavity and the rest of the part's surface (solid distance).

    `voids` are the connected pieces of one cavity, each as it sits in the part (clip any
    cutter overshoot first), e.g. [bolt hole, nut pocket]. The cavity's own faces are
    skipped. Each piece is then measured against every other face it does not touch, so
    a hole that opens into the bed face still counts the nut pocket's wall to that face.

    `open_to` is a designed space the cavity opens into over a large mouth (a head recess
    opening into the band's inside). Faces lying on its boundary are skipped too: next to
    the mouth they meet the cavity at an edge, and their distance is air, not a wall.
    """
    pieces = [voids] if isinstance(voids, Shape) else list(voids)
    cavity = Compound(pieces)
    box = cavity.bounding_box()
    # Nearest faces first; stop once even the box bound is no closer than the best wall.
    faces = sorted((_box_gap(f.bounding_box(), box), i, f) for i, f in enumerate(part.faces()))
    best = math.inf
    for bound, _, face in faces:
        if bound >= best:
            break
        if cavity.distance_to(face) <= touch and _lies_on(face, cavity, touch):
            continue  # a face of the cavity itself
        if open_to is not None and _lies_on(face, open_to, touch):
            continue  # a face of the space the cavity opens into
        gaps = (piece.distance_to(face) for piece in pieces)
        best = min([best, *(g for g in gaps if g > touch)])
    return best


def face_samples(face: Face, n: int = 5) -> list[Vector]:
    """Points on a trimmed face: an n x n grid over its UV bounds, kept if on the face."""
    u0, u1, v0, v1 = BRepTools.UVBounds_s(face.wrapped)
    surface = BRepAdaptor_Surface(face.wrapped)
    points = []
    for i in range(n):
        for j in range(n):
            u = u0 + (u1 - u0) * (i + 0.5) / n
            v = v0 + (v1 - v0) * (j + 0.5) / n
            pt = Vector(surface.Value(u, v))
            if face.distance_to(pt) <= 1e-6:
                points.append(pt)
    return points


def _inside(shape: Shape, pt: Vector) -> bool:
    return any(solid.is_inside(pt) for solid in shape.solids())


def cavity_deviation(
    part: Shape, pieces: Shape | Iterable[Shape], envelope: Shape
) -> tuple[float, int]:
    """How far the part's cavity walls sit from the designed cavity: (worst mm, points).

    `pieces` are the designed voids (cutter & envelope); `envelope` is the body before
    cutting. Points sampled on every designed wall must lie on the part's surface. Points
    in an opening (on the envelope surface) or inside another piece are skipped. A plugged,
    shifted, undersize or oversize hole or pocket moves its walls and shows up here.
    """
    pieces = [pieces] if isinstance(pieces, Shape) else list(pieces)
    worst, count = 0.0, 0
    for k, piece in enumerate(pieces):
        others = pieces[:k] + pieces[k + 1 :]
        # Only nearby faces can decide: exact up to NEAR_MARGIN, larger values stay larger.
        env_near = near_faces(envelope, piece, TOUCH_EPS)
        part_near = near_faces(part, piece, NEAR_MARGIN)
        for face in piece.faces():
            for pt in face_samples(face):
                if env_near is not None and env_near.distance_to(pt) <= TOUCH_EPS:
                    continue  # in an opening
                if any(_inside(o, pt) for o in others):
                    continue
                worst = max(worst, part_near.distance_to(pt) if part_near is not None else math.inf)
                count += 1
    return worst, count


def cavity_check(name: str, part: Shape, pieces, envelope: Shape, tol: float = TOUCH_EPS) -> Check:
    worst, count = cavity_deviation(part, pieces, envelope)
    ok = count > 0 and worst <= tol
    return Check(name, ok, f"walls off design by {worst:.4f} mm (max {tol:g}), {count} points")


def material_spans(part: Shape, origin: Iterable[float], direction: Iterable[float], length: float):
    """Where a probe line runs through solid material: sorted (start, end) distances along
    `direction` from `origin`, for a line `length` long. Exact (boolean of the part with the
    line), so it measures stacks such as ear + gap + ear along a bolt."""
    o, d = Vector(*origin), Vector(*direction).normalized()
    common = part & Edge.make_line(o, o + d * length)
    spans = []
    for edge in common.edges() if common is not None else []:
        a, b = ((edge.position_at(t) - o).dot(d) for t in (0, 1))
        spans.append((min(a, b), max(a, b)))
    return sorted(spans)


def contact_faces(part: Shape, other: Shape, touch: float = TOUCH_EPS) -> list[Face]:
    """Faces of `part` that touch `other` over an area, not just along an edge: at least one
    interior sample point (face_samples) lies within `touch` of `other`."""
    found = []
    for face in part.faces():
        if face.distance_to(other) > touch:
            continue
        if any(other.distance_to(pt) <= touch for pt in face_samples(face)):
            found.append(face)
    return found


def intersection_volume(a: Shape, b: Shape) -> float:
    common = a & b
    return abs(common.volume) if common is not None else 0.0


def clash_checks(
    parts: Mapping[str, Shape], contacts: Iterable[tuple[str, str]] = ()
) -> list[Check]:
    """Pairwise: zero-volume intersection for every pair. Pairs named in `contacts` must
    touch; every other pair must keep a gap, so an accidental contact fails too."""
    allowed = {frozenset(pair) for pair in contacts}
    checks = []
    for pair in sorted(allowed, key=sorted):
        unknown = sorted(pair - set(parts))
        if unknown:
            checks.append(Check(f"contact {'/'.join(sorted(pair))}", False, f"unknown {unknown}"))
    for a, b in combinations(parts, 2):
        volume = intersection_volume(parts[a], parts[b])
        gap = min_distance(parts[a], parts[b])
        expect_contact = frozenset((a, b)) in allowed
        ok = volume <= VOLUME_EPS and (gap <= TOUCH_EPS) == expect_contact
        kind = "contact" if expect_contact else "gap"
        checks.append(
            Check(f"{a} / {b}: {kind}", ok, f"overlap {volume:.1e} mm3, gap {gap:.3f} mm")
        )
    return checks
