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
    Cone,
    Cylinder,
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
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepGProp import BRepGProp
from OCP.BRepIntCurveSurface import BRepIntCurveSurface_Inter
from OCP.gp import gp_Dir, gp_Lin, gp_Pnt
from OCP.GProp import GProp_GProps

from holder.form import teardrop
from holder.params import LEAD_IN, Nut, Screw, Washer

VOL_EPS = 1e-6  # mm3: an intersection at or below this is zero
VOID_EPS = 1e-3  # mm3: a probe outside the part by less than this is numerical noise
OVERHANG_DEG = 45.0
OVERHANG_TOL = 0.5
BRIDGE_SPAN = 10.0
BED_Z = 0.05
SMALL_HOLE_R = 2.5  # holes under 5 mm print round
MODEL_GAP = 0.02  # bought parts sit this far off their seat so touching faces read as zero
PROBE_SLACK = 0.01  # a wall probe stops this short of the minimum, so equality passes

Vec3 = tuple[float, float, float]
GABLE_FAR = "far"  # the channel ceiling in print is past the nut (part prints face down)
GABLE_HEAD = "head"  # the channel ceiling in print is on the head side (part prints face up)
CEILING_SPAN = 9.8  # mm: flat seat between the head-side edge strips (bridge <= 10; the
# strips stay 1.5 from the M3 channels beside them)


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


def teardrop_ring(
    diameter: float, wall: float, length: float, origin: Iterable[float],
    axis: Iterable[float], tip: Iterable[float],
) -> Part:
    """Wall probe round a teardrop hole: a teardrop grown by `wall` is the exact offset."""
    reach = wall - PROBE_SLACK
    outer = teardrop_rod(diameter + 2 * reach, length, origin, axis, tip)
    direction = Vector(*axis).normalized()
    inner = teardrop_rod(diameter + 0.02, length + 2, Vector(*origin) - direction, axis, tip)
    return outer - inner


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


# --- Captive nuts --------------------------------------------------------------------------


@dataclass(frozen=True)
class CaptiveNut:
    """A nut that slides along `travel` from a mouth into a pocket.

    `seat` is the centre of the pocket face toward the screw head: the screw pulls the nut
    onto that face, into solid material. `axis` runs from the head toward the nut. One
    hex corner points along `travel`, so the channel is only the across-flats wide.
    """

    label: str
    nut: Nut
    seat: Vector
    axis: Vector
    travel: Vector
    reach: float  # seat centre to past the mouth, along travel
    gable: str = ""  # "far": 45 deg roof past the nut; "head": 45 deg strips at the seat edges

    @property
    def plane(self) -> Plane:
        return Plane(origin=self.seat, x_dir=self.travel, z_dir=self.axis)

    def pocket(self) -> Part:
        return hex_prism(self.nut.pocket_af, self.nut.pocket_m, self.plane)

    def channel(self) -> Part:
        af, height = self.nut.pocket_af, self.nut.pocket_m
        local = Pos(self.reach / 2, 0, height / 2) * Box(self.reach, af, height)
        if self.gable == GABLE_FAR:
            back = af / sqrt(3)
            roof = Polygon((-af / 2, height), (af / 2, height), (0, height + af / 2), align=None)
            prism = extrude(Plane.YZ.location * roof, amount=self.reach + back)
            local += Pos(-back, 0, 0) * prism
        elif self.gable == GABLE_HEAD:
            # 45 deg strips at both ceiling edges leave a flat seat CEILING_SPAN wide: a short
            # bridge, and still a flat face for the nut to bear on.
            flats = af / sqrt(3) / 2
            run = (af - CEILING_SPAN) / 2
            for side in (-1.0, 1.0):
                edge = side * af / 2
                strip = Polygon((edge, 0), (edge - side * run, 0), (edge, -run), align=None)
                prism = extrude(Plane.YZ.location * strip, amount=self.reach + flats)
                local += Pos(-flats, 0, 0) * prism
        return self.plane.location * local

    def cutters(self) -> list[Part]:
        return [self.pocket(), self.channel()]

    def nut_at_seat(self) -> Part:
        """The max nut where the screw pulls it: on the seat face."""
        plane = self.plane.offset(MODEL_GAP)
        body = hex_prism(self.nut.af_max, self.nut.m_max, plane)
        return body - rod(self.nut.minor_d, self.nut.m_max + 2, plane.origin - self.axis,
                          self.axis)

    def sweep(self) -> Part:
        """The max nut swept from past the mouth to its seat."""
        lift = (self.nut.pocket_m - self.nut.m_max) / 2
        plane = self.plane.offset(lift)
        corner = self.nut.af_max / sqrt(3)
        points = [(corner, 0), (corner / 2, self.nut.af_max / 2),
                  (-corner / 2, self.nut.af_max / 2), (-corner, 0),
                  (-corner / 2, -self.nut.af_max / 2), (corner / 2, -self.nut.af_max / 2)]
        hexagon = Polygon(*points, align=None)
        # Sweep the 2D hex along travel, then thicken: a prism along x with hex-ish ends.
        body = extrude(hexagon, amount=self.nut.m_max)
        path = extrude(
            Plane.YZ.location * Polygon(
                (-self.nut.af_max / 2, 0), (self.nut.af_max / 2, 0),
                (self.nut.af_max / 2, self.nut.m_max), (-self.nut.af_max / 2, self.nut.m_max),
                align=None,
            ),
            amount=self.reach,
        )
        return plane.location * (body + path)

    def checks(self, part: Part) -> list[Check]:
        clash = intersection_volume(self.sweep(), part)
        rows = [Check(f"{self.label} nut travel", clash <= VOL_EPS,
                      f"{clash:.2e} mm3 of PETG in the max nut's path")]
        width = self.channel_width(part)
        rows.append(at_least(f"{self.label} channel width", width, self.nut.af_max + 0.15))
        return rows

    def channel_width(self, part: Part) -> float:
        """Across-travel width at the pocket centre, read from the real faces."""
        centre = self.seat + self.axis * (self.nut.pocket_m / 2)
        across = self.axis.cross(self.travel).normalized()
        return first_hit(part, centre, across) + first_hit(part, centre, -across)

    def wall_probes(self, wall: float, hole_d: float) -> list[tuple[str, Part]]:
        """Back half of a hex ring round the pocket, and the roof over its flats.

        The mouth side is the channel, so it is left out of both probes.
        """
        af, height = self.nut.pocket_af, self.nut.pocket_m
        reach = wall - PROBE_SLACK
        grown = hex_prism(af + 2 * reach, height, Plane(origin=(0, 0, 0)))
        ring = grown - hex_prism(af + 0.02, height + 2, Plane(origin=(0, 0, -1)))
        back = ring & box_from(-af, -af, -1, 0, af, height + 1)
        roof = hex_prism(af, reach, Plane(origin=(0, 0, -reach)))
        roof &= box_from(-af, -af, -wall - 1, af / sqrt(3) / 2, af, 1)
        roof -= Pos(0, 0, -wall - 1) * Cylinder(hole_d / 2 + 0.02, wall + 2,
                                                  align=(Align.CENTER, Align.CENTER, Align.MIN))
        place = self.plane.location
        own = self.channel()
        return [(f"{self.label} side wall", place * back - own),
                (f"{self.label} roof", place * roof - own)]


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


def hex_bolt(screw: Screw, length: float, origin: Vector, axis: Vector, corner: Vector) -> Part:
    plane = plane_at(origin, axis, corner)
    head = hex_prism(screw.head_d, screw.head_k, Plane(origin=(0, 0, -screw.head_k)))
    shank = Cylinder(screw.d / 2, length, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return plane.location * (head + shank)


def nut_solid(nut: Nut, origin: Vector, axis: Vector, corner: Vector) -> Part:
    plane = plane_at(origin, axis, corner)
    body = hex_prism(nut.af_max, nut.m_max, Plane(origin=(0, 0, 0)))
    body -= Pos(0, 0, -1) * Cylinder(nut.minor_d / 2, nut.m_max + 2,
                                     align=(Align.CENTER, Align.CENTER, Align.MIN))
    return plane.location * body


def washer_solid(washer: Washer, origin: Vector, axis: Vector) -> Part:
    """Washer from `origin` (head side) for its thickness along `axis`."""
    ring = rod(washer.outer, washer.t, origin, axis) - rod(
        washer.inner, washer.t + 2, Vector(*origin) - Vector(*axis).normalized(), axis
    )
    return ring


def column_solid(width: float, depth: float, radius: float, z0: float, z1: float) -> Part:
    from build123d import RectangleRounded

    return Pos(0, 0, z0) * extrude(RectangleRounded(width, depth, radius), amount=z1 - z0)


# --- Screw stacks --------------------------------------------------------------------------


@dataclass(frozen=True)
class Stack:
    """One screw joint. Grips run from under the head to the nut's load face."""

    joint: str
    screw: Screw
    grip_open: float
    grip_closed: float
    engage_need: float
    tip_limit: float  # longest allowed protrusion past the load face
    head_sink: float | None = None

    @property
    def length(self) -> float:
        return self.screw.stock_at_least(self.grip_open + self.engage_need + self.screw.length_tol)

    def rows(self) -> list[Check]:
        size, tol = self.length, self.screw.length_tol
        rows = [Check(f"{self.joint} screw", True, f"{self.screw.standard} x{size:g}")]
        for state, grip in (("open", self.grip_open), ("closed", self.grip_closed)):
            engaged = size - tol - grip
            rows.append(at_least(f"{self.joint} engagement, short screw, {state}", engaged,
                                 self.engage_need))
            tip = size + tol - grip
            rows.append(at_most(f"{self.joint} tip, long screw, {state}", tip, self.tip_limit))
        if self.head_sink is not None:
            rows.append(at_least(f"{self.joint} head below the surface", self.head_sink, 0.3))
        return rows


def thread_need(screw: Screw, nut: Nut) -> float:
    """Nut height plus two pitches of protrusion."""
    return nut.m_max + 2 * screw.pitch


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


def section_modulus(part: Part, plane: Plane, moment_axis: Vector) -> tuple[float, float, float]:
    """(area, I, I/c) of the real section about the moment axis through its centroid."""
    faces = part.intersect(plane)
    if not faces:
        raise ValueError("section plane misses the part")
    props = GProp_GProps()
    for face in faces:
        BRepGProp.SurfaceProperties_s(face.wrapped, props)
    centre = Vector(props.CentreOfMass())
    matrix = props.MatrixOfInertia()
    axis = moment_axis.normalized()
    values = (axis.X, axis.Y, axis.Z)
    inertia = sum(values[r] * matrix.Value(r + 1, c + 1) * values[c]
                  for r in range(3) for c in range(3))
    across = plane.z_dir.cross(axis).normalized()
    reach = max(abs((vertex.center() - centre).dot(across))
                for face in faces for edge in face.edges()
                for vertex in [Vertex(edge @ (k / 16)) for k in range(17)])
    return props.Mass(), inertia, inertia / reach


def stress_row(part: Part, plane: Plane, moment_axis: Vector, moment_nmm: float,
               allowed_mpa: float, name: str) -> Check:
    _area, _inertia, modulus = section_modulus(part, plane, moment_axis)
    return at_most(name, moment_nmm / modulus, allowed_mpa, "MPa")
