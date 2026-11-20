"""Shared tools: check rows, cutters, captive nuts, bought solids, stacks, and probes.

Measure on real geometry, never on bounding boxes (OCCT pads them): ray hits on real
faces for stack lengths, boolean volume for clashes and walls, face normals for
overhangs, and the inertia of a real section for stress.
"""

from collections.abc import Iterable
from dataclasses import dataclass
from itertools import combinations
from math import acos, degrees, sqrt
from typing import NamedTuple

from build123d import (
    Align,
    Axis,
    Box,
    Compound,
    Cone,
    Cylinder,
    Edge,
    Face,
    GeomType,
    Location,
    Part,
    Plane,
    Polygon,
    Pos,
    Vector,
    Vertex,
    chamfer,
    extrude,
)
from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepGProp import BRepGProp
from OCP.BRepIntCurveSurface import BRepIntCurveSurface_Inter
from OCP.BRepLProp import BRepLProp_SLProps
from OCP.GeomAPI import GeomAPI_ProjectPointOnSurf
from OCP.gp import gp_Dir, gp_Lin, gp_Pnt
from OCP.GProp import GProp_GProps
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp
from OCP.TopoDS import TopoDS
from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape, TopTools_IndexedMapOfShape

from holder.form import flat_teardrop, teardrop
from holder.params import (
    BEARING_LIMIT,
    INSERT_SHEAR_LIMIT,
    INSERT_SINK,
    LEAD_IN,
    Insert,
    Screw,
    Washer,
)

VOL_EPS = 1e-6  # mm3: an intersection at or below this is zero
VOID_EPS = 1e-3  # mm3: a probe outside the part by less than this is numerical noise
OVERHANG_DEG = 45.0
OVERHANG_TOL = 0.5
BRIDGE_SPAN = 10.0
BED_Z = 0.05
SMALL_HOLE_R = 3.25  # estimated: a horizontal hole up to 6.5 mm prints round with a
# small sag on top; a heat-set insert hole must be round, so M5 insert holes rely on this
MODEL_GAP = 0.02  # bought parts sit this far off their seat so touching faces read as zero
PROBE_SLACK = 0.01  # a wall probe stops this short of the minimum, so equality passes
CONTACT_T = 0.05  # thickness of a bearing-area probe sunk into the seat

Vec3 = tuple[float, float, float]


class Check(NamedTuple):
    name: str
    ok: bool
    measured: str


def at_least(name: str, value: float, minimum: float, unit: str = "mm") -> Check:
    return Check(name, value >= minimum - 1e-9, f"{value:.3f} {unit} (min {minimum:g})")


def at_most(name: str, value: float, maximum: float, unit: str = "mm") -> Check:
    return Check(name, value <= maximum + 1e-9, f"{value:.4g} {unit} (max {maximum:g})")


def within(name: str, value: float, low: float, high: float, unit: str = "mm") -> Check:
    ok = low - 1e-9 <= value <= high + 1e-9
    return Check(name, ok, f"{value:.3f} {unit} (range {low:g}..{high:g})")


def one_solid(part: Part) -> Check:
    count = len(part.solids())
    ok = count == 1 and part.is_valid and part.volume > 0
    return Check("one solid", ok, f"{count} solid, {part.volume:.1f} mm3, valid={part.is_valid}")


def named(part: Part, label: str) -> Part:
    part.label = label
    return part


# --- Placement -----------------------------------------------------------------------------


def plane_at(origin: Iterable[float], z_dir: Iterable[float], x_dir: Iterable[float]) -> Plane:
    return Plane(origin=Vector(*origin), x_dir=Vector(*x_dir), z_dir=Vector(*z_dir))


def any_perpendicular(direction: Vector) -> Vector:
    reference = Vector(1, 0, 0) if abs(direction.X) < 0.9 else Vector(0, 1, 0)
    return (reference - direction * reference.dot(direction)).normalized()


def print_orientation(part: Part, bed_normal: Vec3) -> Part:
    """Copy rotated so `bed_normal` points down, centred in XY, bed face on Z = 0."""
    normal = Vector(*bed_normal).normalized()
    down = Vector(0, 0, -1)
    turn_axis = normal.cross(down)
    if turn_axis.length > 1e-9:
        turned = part.rotate(Axis((0, 0, 0), turn_axis), normal.get_signed_angle(down, turn_axis))
    elif normal.Z > 0:
        turned = part.rotate(Axis.X, 180)
    else:
        turned = part.moved(Location())
    bounds = turned.bounding_box()
    moved = turned.moved(Location((-bounds.center().X, -bounds.center().Y, -bounds.min.Z)))
    moved.label = part.label
    return moved


# --- Cutters -------------------------------------------------------------------------------


def box_from(x0: float, y0: float, z0: float, x1: float, y1: float, z1: float) -> Part:
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(
        abs(x1 - x0), abs(y1 - y0), abs(z1 - z0)
    )


def rod(diameter: float, length: float, origin: Iterable[float], axis: Iterable[float]) -> Part:
    """Cylinder from `origin` for `length` along `axis`."""
    direction = Vector(*axis).normalized()
    plane = plane_at(origin, direction, any_perpendicular(direction))
    body = Cylinder(diameter / 2, length, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return plane.location * body


def lead_in(diameter: float, origin: Iterable[float], axis: Iterable[float]) -> Part:
    """0.4 x 45 degree cone at a hole mouth. `axis` points into the material."""
    direction = Vector(*axis).normalized()
    start = Vector(*origin) - direction * 0.5
    plane = plane_at(start, direction, any_perpendicular(direction))
    radius = diameter / 2 + LEAD_IN + 0.5
    cone = Cone(radius, diameter / 2 - 0.4, radius - diameter / 2 + 0.4,
                align=(Align.CENTER, Align.CENTER, Align.MIN))
    return plane.location * cone


def teardrop_rod(
    diameter: float, length: float, origin: Iterable[float], axis: Iterable[float],
    tip: Iterable[float],
) -> Part:
    """Horizontal hole with a 45 degree roof toward `tip` (print-up)."""
    direction = Vector(*axis).normalized()
    up = Vector(*tip).normalized()
    plane = plane_at(origin, direction, up)
    face = plane.location * teardrop(diameter / 2, (0, 0), 0.0)
    return extrude(face, amount=length, dir=direction)


def flat_teardrop_rod(
    diameter: float, length: float, origin: Iterable[float], axis: Iterable[float],
    tip: Iterable[float],
) -> Part:
    """Horizontal hole with a 45 degree roof toward `tip`, cut flat at the circle."""
    direction = Vector(*axis).normalized()
    plane = plane_at(origin, direction, Vector(*tip).normalized())
    face = plane.location * flat_teardrop(diameter / 2, (0, 0), 0.0)
    return extrude(face, amount=length, dir=direction)


def hex_prism(af: float, height: float, plane: Plane) -> Part:
    """Hex from local z 0 to `height`, one corner on local +x."""
    corner = af / sqrt(3)
    points = [(corner, 0), (corner / 2, af / 2), (-corner / 2, af / 2),
              (-corner, 0), (-corner / 2, -af / 2), (corner / 2, -af / 2)]
    return plane.location * extrude(Polygon(*points, align=None), amount=height)


def intersection_volume(left: Part, right: Part) -> float:
    hit = left & right
    if hit is None:
        return 0.0
    return float(sum(solid.volume for solid in hit.solids()))


def outside_volume(probe: Part, part: Part) -> float:
    leftover = probe - part
    if leftover is None:
        return 0.0
    return float(sum(solid.volume for solid in leftover.solids()))


def void_open(part: Part, probe: Part, name: str) -> Check:
    volume = intersection_volume(part, probe)
    return Check(name, volume <= VOID_EPS, f"{volume:.4f} mm3 of material in the void")


def encloses(part: Part, probe: Part, name: str) -> Check:
    """The probe is material that must lie inside the part: a wall, measured as a solid."""
    volume = outside_volume(probe, part)
    return Check(name, volume <= VOID_EPS, f"{volume:.4f} mm3 of the wall probe is missing")


def ring_probe(
    origin: Iterable[float], axis: Iterable[float], radius: float, wall: float,
    start: float, stop: float,
) -> Part:
    """Cylindrical shell `wall` thick round a hole, from `start` to `stop` along the axis."""
    direction = Vector(*axis).normalized()
    begin = Vector(*origin) + direction * start
    outer = rod(2 * (radius + wall - PROBE_SLACK), stop - start, begin, direction)
    inner = rod(2 * radius + 0.02, stop - start + 2, begin - direction, direction)
    return outer - inner


# --- Heat-set inserts ------------------------------------------------------------------------


@dataclass(frozen=True)
class SeatedInsert:
    """A heat-set insert pressed in along `axis` from `face` (a point on the part face).

    The insert top sits INSERT_SINK below the face, so a shoulder that stops on it lands
    on brass. The blind hole runs `tip_room` past the insert. A hole that lies across the
    print layers gets a 45 degree roof toward `tip` (print-up).
    """

    label: str
    insert: Insert
    face: Vector
    axis: Vector  # into the part
    tip_room: float = 0.5
    tip: Vector | None = None

    @property
    def top(self) -> Vector:
        return self.face + self.axis * INSERT_SINK

    @property
    def depth(self) -> float:
        return INSERT_SINK + self.insert.length + self.tip_room

    def _bore(self, diameter: float, length: float, origin: Vector) -> Part:
        if self.tip is None:
            return rod(diameter, length, origin, self.axis)
        return teardrop_rod(diameter, length, origin, self.axis, self.tip)

    def cutters(self) -> list[Part]:
        return [self._bore(self.insert.hole_d, self.depth + 1, self.face - self.axis),
                lead_in(self.insert.hole_d, self.face, self.axis)]

    def solid(self) -> Part:
        """The insert as pressed: knurl diameter, thread minor bore (the screw overlaps it)."""
        body = rod(self.insert.outer_d, self.insert.length, self.top, self.axis)
        return body - rod(self.insert.minor_d, self.insert.length + 2, self.top - self.axis,
                          self.axis)

    def wall_probe(self, start: float = 0.0) -> Part:
        return ring_probe(self.top, self.axis, self.insert.outer_d / 2, self.insert.wall_min,
                          start, self.insert.length)

    def checks(self, part: Part, wall_from: float = 0.0) -> list[Check]:
        """Maker's wall round the knurl (from `wall_from` below the insert top), and a blind
        hole deep enough for the insert and its tip room. A through hole fails."""
        rows = [encloses(part, self.wall_probe(wall_from),
                         f"{self.label} insert wall >= {self.insert.wall_min:g}")]
        depth = first_hit(part, self.top + self.axis * 0.5, self.axis) + 0.5 + INSERT_SINK
        rows.append(within(f"{self.label} insert hole depth (blind)", depth,
                           INSERT_SINK + self.insert.length + 0.3, self.depth + 0.05))
        return rows


def insert_shear_row(name: str, insert: Insert, force: float) -> Check:
    """Pull-out load spread over the knurl cylinder, against INSERT_SHEAR_LIMIT."""
    area = 3.141592653589793 * insert.outer_d * insert.length
    return at_most(f"{name} insert pull-out shear at {force:g} N", force / area,
                   INSERT_SHEAR_LIMIT, "MPa")


# --- Bearing under heads, washers and nuts -----------------------------------------------------


def contact_area(part: Part, footprint: Part) -> float:
    """Area of the part under a bearing face: the face as a CONTACT_T solid sunk into the seat."""
    return intersection_volume(part, footprint) / CONTACT_T


def annulus_footprint(seat: Vector, axis: Vector, outer_d: float, inner_d: float) -> Part:
    """A washer or round head face on the seat, `axis` pointing into the part."""
    disc = rod(outer_d, CONTACT_T, seat, axis)
    return disc - rod(inner_d, CONTACT_T + 2, seat - axis, axis)


def bearing_row(name: str, force: float, area: float) -> Check:
    if area <= 0:
        return Check(f"{name} bearing", False, "no PETG under the bearing face")
    return at_most(f"{name} bearing at {force:g} N on {area:.1f} mm2", force / area,
                   BEARING_LIMIT, "MPa")


# --- Captive nuts --------------------------------------------------------------------------


# --- Ray measurements ----------------------------------------------------------------------


def first_hit(part: Part, origin: Vector, direction: Vector) -> float:
    """Distance from `origin` along `direction` to the first face of the part.

    OCCT's line-surface intersector reports the line parameter, which is the distance for
    a unit direction. It needs no face normal, so a pole (a revolve apex) is no problem.
    """
    unit = direction.normalized()
    hits = BRepIntCurveSurface_Inter()
    hits.Init(part.wrapped, gp_Lin(gp_Pnt(origin.X, origin.Y, origin.Z),
                                   gp_Dir(unit.X, unit.Y, unit.Z)), 1e-7)
    nearest = float("inf")
    while hits.More():
        if hits.W() > 1e-6:
            nearest = min(nearest, hits.W())
        hits.Next()
    return nearest


# --- Bought parts as solids ----------------------------------------------------------------


def socket_screw(screw: Screw, length: float, origin: Vector, axis: Vector) -> Part:
    """Socket head screw. `origin` is the centre under the head; the shank runs along `axis`."""
    plane = plane_at(origin, axis, any_perpendicular(axis))
    head = Pos(0, 0, -screw.head_k) * Cylinder(
        screw.head_d / 2, screw.head_k, align=(Align.CENTER, Align.CENTER, Align.MIN)
    )
    head = chamfer(head.edges().sort_by(Axis.Z)[0], screw.head_d * 0.06)
    socket = hex_prism(screw.key, screw.head_k * 0.6, Plane(origin=(0, 0, -screw.head_k - 0.01)))
    shank = Cylinder(screw.d / 2, length, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return plane.location * (head - socket + shank)


def washer_solid(washer: Washer, origin: Vector, axis: Vector) -> Part:
    """Washer from `origin` (head side) for its thickness along `axis`."""
    ring = rod(washer.outer, washer.t, origin, axis) - rod(
        washer.inner, washer.t + 2, Vector(*origin) - Vector(*axis).normalized(), axis
    )
    return ring


def column_solid(width: float, depth: float, radius: float, z0: float, z1: float) -> Part:
    from build123d import RectangleRounded

    return Pos(0, 0, z0) * extrude(RectangleRounded(width, depth, radius), amount=z1 - z0)


# --- Assembly clash ------------------------------------------------------------------------


def boxes_touch(left: Part, right: Part) -> bool:
    a, b = left.bounding_box(), right.bounding_box()
    return (a.min.X <= b.max.X and b.min.X <= a.max.X and a.min.Y <= b.max.Y
            and b.min.Y <= a.max.Y and a.min.Z <= b.max.Z and b.min.Z <= a.max.Z)


def clash_rows(solids: dict[str, Part], intended: set[frozenset[str]], name: str) -> list[Check]:
    """Every pair whose boxes touch must share at most VOL_EPS, unless it is intended."""
    failures: list[Check] = []
    checked = 0
    worst = 0.0
    for left, right in combinations(solids, 2):
        if frozenset((left, right)) in intended:
            continue
        if not boxes_touch(solids[left], solids[right]):
            continue
        checked += 1
        volume = intersection_volume(solids[left], solids[right])
        worst = max(worst, volume)
        if volume > VOL_EPS:
            failures.append(Check(f"{name}: {left} x {right}", False, f"{volume:.4g} mm3"))
    summary = Check(f"{name}: clash matrix", not failures,
                    f"{len(solids)} solids, {checked} pairs checked, worst {worst:.2e} mm3")
    return [summary, *failures]


# --- Print checks --------------------------------------------------------------------------


def _down_angle(normal: Vector) -> float:
    return degrees(acos(max(-1.0, min(1.0, -normal.Z))))


def _bridge_ok(face: Face) -> bool:
    """Every point of a flat ceiling lies within half a bridge of its outer wire."""
    outer = face.outer_wire()
    bounds = face.bounding_box()
    steps = 14
    for i in range(steps + 1):
        for j in range(steps + 1):
            x = bounds.min.X + (bounds.max.X - bounds.min.X) * i / steps
            y = bounds.min.Y + (bounds.max.Y - bounds.min.Y) * j / steps
            point = Vector(x, y, bounds.center().Z)
            if face.is_inside(point, 1e-4) and Vertex(point).distance_to(outer) > BRIDGE_SPAN / 2:
                return False
    return True


def overhang_rows(printed: Part, name: str) -> list[Check]:
    """No down-facing area steeper than 45 degrees above the bed, unless it is a short bridge."""
    limit = OVERHANG_DEG - OVERHANG_TOL
    bad: list[str] = []
    for face in printed.faces():
        if face.bounding_box().max.Z <= BED_Z:
            continue
        if face.geom_type == GeomType.CYLINDER and cylinder_radius(face) < SMALL_HOLE_R:
            continue
        if face.geom_type == GeomType.PLANE:
            angle = _down_angle(face.normal_at())
            if angle >= limit:
                continue
            if angle < OVERHANG_TOL and _bridge_ok(face):
                continue
            bad.append(f"plane at z {face.center().Z:.1f}, {angle:.1f} deg")
            continue
        worst = _worst_sample(face)
        if worst is not None:
            bad.append(worst)
    detail = "none" if not bad else "; ".join(bad[:4]) + (f" (+{len(bad) - 4})" if len(bad) > 4
                                                           else "")
    return [Check(f"{name}: overhang <= 45 deg or bridge <= 10 mm", not bad, detail)]


def cylinder_radius(face: Face) -> float:
    return BRepAdaptor_Surface(face.wrapped).Cylinder().Radius()


def _worst_sample(face: Face) -> str | None:
    steps = 8
    for i in range(steps):
        for j in range(steps):
            u, v = (i + 0.5) / steps, (j + 0.5) / steps
            point = face.position_at(u, v)
            if point.Z <= BED_Z or not face.is_inside(point, 1e-3):
                continue
            angle = _down_angle(face.normal_at(u, v))
            if angle < OVERHANG_DEG - OVERHANG_TOL:
                return f"{face.geom_type.name.lower()} at z {point.Z:.1f}, {angle:.1f} deg"
    return None


def _tensor(matrix, left: Vector, right: Vector) -> float:
    a, b = (left.X, left.Y, left.Z), (right.X, right.Y, right.Z)
    return sum(a[r] * matrix.Value(r + 1, c + 1) * b[c] for r in range(3) for c in range(3))


class SectionStress(NamedTuple):
    bending: float  # MPa, worst fibre, both in-plane moment components together
    torsion: float  # MPa, T / (0.208 b t^2): the square-bar factor, conservative for b > t
    von_mises: float


def section_stress(part: Part, plane: Plane, moment: Vector) -> SectionStress:
    """Stress at the real section for a moment vector (N mm) in the part frame.

    The in-plane components bend the section about its centroid, with the product of
    inertia (the section need not be symmetric). The normal component twists it.
    """
    faces = part.intersect(plane)
    if not faces:
        raise ValueError("section plane misses the part")
    props = GProp_GProps()
    for face in faces:
        BRepGProp.SurfaceProperties_s(face.wrapped, props)
    centre, matrix = Vector(props.CentreOfMass()), props.MatrixOfInertia()
    u, v, n = plane.x_dir, plane.y_dir, plane.z_dir
    i_uu, i_vv, i_uv = _tensor(matrix, u, u), _tensor(matrix, v, v), -_tensor(matrix, u, v)
    m_u, m_v = moment.dot(u), moment.dot(v)
    det = i_uu * i_vv - i_uv * i_uv
    points = [Vector(edge @ (k / 24)) - centre for face in faces for edge in face.edges()
              for k in range(25)]
    bending = max(abs(((m_u * i_vv + m_v * i_uv) * p.dot(v)
                       - (m_v * i_uu + m_u * i_uv) * p.dot(u)) / det) for p in points)
    spans = sorted(max(p.dot(axis) for p in points) - min(p.dot(axis) for p in points)
                   for axis in (u, v))
    torsion = abs(moment.dot(n)) / (0.208 * spans[1] * spans[0] ** 2)
    return SectionStress(bending, torsion, sqrt(bending ** 2 + 3 * torsion ** 2))


def stress_row(part: Part, plane: Plane, moment: Vector, allowed_mpa: float,
               name: str) -> Check:
    found = section_stress(part, plane, moment)
    detail = f"{found.von_mises:.3f} MPa (bending {found.bending:.3f}, torsion " \
             f"{found.torsion:.3f}; max {allowed_mpa:.3g})"
    return Check(name, found.von_mises <= allowed_mpa + 1e-9, detail)


# --- Surface continuity ------------------------------------------------------------------------

G1_DEG = 0.5  # a join with a larger normal step is a crease
G2_JUMP = 2e-3  # 1/mm: cross-edge normal curvature step allowed at a G2 join


class EdgeJoin(NamedTuple):
    centre: Vector
    angle: float  # degrees between the two face normals
    curvature_jump: float  # 1/mm, normal curvature across the edge


def _cross_curvature(face: Face, point: Vector, along: Vector) -> tuple[Vector, float]:
    """Unit normal and normal curvature across the edge direction `along` at `point`."""
    surface = BRep_Tool.Surface_s(face.wrapped)
    u, v = GeomAPI_ProjectPointOnSurf(gp_Pnt(*point), surface).LowerDistanceParameters()
    props = BRepLProp_SLProps(BRepAdaptor_Surface(face.wrapped), u, v, 2, 1e-7)
    normal = Vector(props.Normal())
    side = -1.0 if face.wrapped.Orientation() == TopAbs_REVERSED else 1.0
    if not props.IsCurvatureDefined():
        return normal * side, 0.0
    first, second = gp_Dir(), gp_Dir()
    props.CurvatureDirections(first, second)
    across = normal.cross(along).normalized()
    share = Vector(first).dot(across) ** 2
    curvature = props.MaxCurvature() * share + props.MinCurvature() * (1 - share)
    return normal * side, curvature * side  # both read against the outward normal


def _inner_point(face: Face) -> Vector:
    """A point inside the trimmed face: the UV centre, or the first grid point inside."""
    for steps in (1, 3, 7):
        for i in range(steps):
            for j in range(steps):
                point = face.position_at((i + 0.5) / steps, (j + 0.5) / steps)
                if face.is_inside(point, 1e-4):
                    return point
    return face.center()


def on_skin(part: Part, skin: Part, tol: float = 1e-4) -> set[int]:
    """Indices (OCCT map, from 1) of the part's faces on the uncut body's outer surface.

    A face cut by a hole, a seam or a flat lies inside the body, off its surface. A crease
    where two fused skins meet has both its faces on the surface, so it is measured.
    """
    surface = Compound(skin.faces())
    faces = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(part.wrapped, TopAbs_FACE, faces)
    return {index for index in range(1, faces.Extent() + 1)
            if Vertex(_inner_point(Face(TopoDS.Face_s(faces.FindKey(index)))))
            .distance_to(surface) <= tol}


class Joins(NamedTuple):
    skin: list[EdgeJoin]  # both faces on the outer surface
    features: int  # edges where a cut (hole, flat, seam) meets anything


def edge_joins(part: Part, skin: Part | None = None) -> Joins:
    """Normal step and curvature step across every edge shared by two faces, at 3 points.

    With `skin` (the body before any cut), only edges whose two faces both lie on the
    skin's surface are measured; the rest are counted as feature edges.
    """
    faces = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(part.wrapped, TopAbs_FACE, faces)
    keep = set(range(1, faces.Extent() + 1)) if skin is None else on_skin(part, skin)
    shared = TopTools_IndexedDataMapOfShapeListOfShape()
    TopExp.MapShapesAndAncestors_s(part.wrapped, TopAbs_EDGE, TopAbs_FACE, shared)
    joins, features = [], 0
    for index in range(1, shared.Extent() + 1):
        pair = [Face(TopoDS.Face_s(face)) for face in shared.FindFromIndex(index)]
        if len(pair) != 2:
            continue
        if not all(faces.FindIndex(face.wrapped) in keep for face in pair):
            features += 1
            continue
        edge = Edge(TopoDS.Edge_s(shared.FindKey(index)))
        worst_angle, worst_jump = 0.0, 0.0
        for share in (0.2, 0.5, 0.8):
            point, along = edge @ share, (edge % share).normalized()
            (n0, k0), (n1, k1) = (_cross_curvature(face, point, along) for face in pair)
            worst_angle = max(worst_angle, n0.get_angle(n1))
            worst_jump = max(worst_jump, abs(k0 - k1))
        joins.append(EdgeJoin(edge.center(), worst_angle, worst_jump))
    return Joins(joins, features)


def skin_rows(part: Part, name: str, crease_ok, skin: Part | None = None) -> list[Check]:
    """Every join of the outer surface is G2 (normal step <= 0.5 deg, curvature step
    <= 2e-3 /mm), except where `crease_ok(centre)` names an intended crease (a bed lead).

    Run on the finished part with its uncut `skin`, so a crease where two volumes were
    fused is measured, and cut features are counted but not judged.
    """
    found = edge_joins(part, skin)
    joins = [join for join in found.skin if not crease_ok(join.centre)]
    angle = max((join.angle for join in joins), default=0.0)
    jump = max((join.curvature_jump for join in joins), default=0.0)
    where = f"{len(joins)} surface joins, {found.features} feature edges"
    return [
        at_most(f"{name} surface: normal step at every join ({where})", angle, G1_DEG,
                "deg"),
        at_most(f"{name} surface: curvature step at every join", jump, G2_JUMP, "/mm"),
    ]
