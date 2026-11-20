"""Plan and profile curves, and the G2 skin every printed body is made of.

A corner is a quintic Bezier with three control points on each side. Its curvature is
zero where it meets a straight side, so the join has no visible line (G2). The general
blend matches position, tangent, and curvature at both ends, so it also joins a circle
or a parabola. Sketches holding a Bezier are never offset: the STEP writer drops
OFFSET-curve faces. Rebuild the outline at the new size instead.

A skin is one edge profile (cap, G2 roll, wall, G2 roll, cap) swept round a G2 plan
outline, closed by two planar caps and sewn into one solid. No boolean and no OCC fillet
touches it, so every outer edge is curvature continuous by construction. The only crease
is the 45 degree lead on a print-bed edge, which no G2 roll can print without support.
"""

from math import cos, radians, sin, sqrt
from typing import NamedTuple

from build123d import (
    Bezier,
    CenterArc,
    Edge,
    Face,
    Line,
    Part,
    Plane,
    Solid,
    Vector,
    Wire,
    make_face,
    sweep,
)
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid, BRepBuilderAPI_Sewing
from OCP.BRepLib import BRepLib
from OCP.BRepLProp import BRepLProp_CLProps
from OCP.TopoDS import TopoDS

HANDLE_FAR = 0.6  # share of the setback; ported from macropad solids._corner_curve
HANDLE_NEAR = 0.2
Point = tuple[float, float]


class Frame(NamedTuple):
    """Curve end: point, unit travel tangent, unit normal toward the curvature centre, k."""

    point: Vector
    tangent: Vector
    normal: Vector
    curvature: float


def g2_points(start: Frame, end: Frame, reach_start: float, reach_end: float) -> list[Vector]:
    """Control points of the quintic that matches both frames to second order.

    For a quintic, curvature at t = 0 is 4/5 * h / a^2, where a = |P1 - P0| and h is
    P2's offset from the tangent line. So h = 5 a^2 k / 4. The end mirrors the start.
    """
    a0, a1 = reach_start, reach_end
    p1 = start.point + start.tangent * a0
    p2 = p1 + start.tangent * a0 + start.normal * (5 * a0 * a0 * start.curvature / 4)
    p4 = end.point - end.tangent * a1
    p3 = p4 - end.tangent * a1 + end.normal * (5 * a1 * a1 * end.curvature / 4)
    return [start.point, p1, p2, p3, p4, end.point]


def g2_blend(start: Frame, end: Frame, reach_start: float, reach_end: float) -> Bezier:
    return Bezier(*g2_points(start, end, reach_start, reach_end))


def _unit(vector: Vector) -> Vector:
    return vector.normalized()


def corner_curve(before: Point, corner: Point, after: Point, setback: float) -> Bezier:
    """G2 corner between two straight sides: zero curvature at both ends."""
    tip = Vector(*corner)
    back = _unit(Vector(*before) - tip)
    ahead = _unit(Vector(*after) - tip)
    reach = (HANDLE_FAR - HANDLE_NEAR) * setback
    start = Frame(tip + back * setback, -back, ahead, 0.0)
    end = Frame(tip + ahead * setback, ahead, back, 0.0)
    return g2_blend(start, end, reach, reach)


def soft_outline(vertices: list[Point], setbacks: list[float]) -> Wire:
    """Closed outline: a G2 corner at each vertex, straight sides between the corners."""
    if len(vertices) != len(setbacks) or len(vertices) < 3:
        raise ValueError("soft_outline needs one setback per vertex and three or more vertices")
    count = len(vertices)
    corners = [
        corner_curve(vertices[i - 1], vertices[i], vertices[(i + 1) % count], setbacks[i])
        for i in range(count)
    ]
    edges: list[Edge] = []
    for index, curve in enumerate(corners):
        following = corners[(index + 1) % count]
        edges.append(curve)
        if (curve @ 1 - following @ 0).length > 1e-6:
            edges.append(Line(curve @ 1, following @ 0))
    return Wire(edges)


def soft_rect(width: float, depth: float, setback: float, center: Point = (0, 0)) -> Face:
    """Soft rectangle in XY: four G2 corners of equal setback."""
    cx, cy = center
    hx, hy = width / 2, depth / 2
    vertices = [(cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy), (cx - hx, cy - hy)]
    return make_face(soft_outline(vertices, [setback] * 4)).faces()[0]


def teardrop(radius: float, center: Point, tip_deg: float) -> Face:
    """Circle with a 45 degree roof: two tangent lines meet at radius * sqrt(2) along tip_deg.

    G1 at the tangent points on purpose: a G2 blend there would overhang past 45 degrees.
    """
    cx, cy = center
    left = tip_deg + 45
    tip = Vector(cx + radius * sqrt(2) * cos(radians(tip_deg)),
                 cy + radius * sqrt(2) * sin(radians(tip_deg)))
    arc = CenterArc((cx, cy), radius, left, 270)
    return make_face(Wire([arc, Line(arc @ 1, tip), Line(tip, arc @ 0)])).faces()[0]


def edge_curvature(edge: Edge, near: Vector) -> float:
    """Curvature of the real edge at its end nearest `near`, read from OCCT (1/mm).

    OCCT's curve parameters ignore the edge's orientation in a wire, so the end is picked
    by position, not by first or last parameter.
    """
    adaptor = BRepAdaptor_Curve(edge.wrapped)
    ends = (adaptor.FirstParameter(), adaptor.LastParameter())
    parameter = min(ends, key=lambda u: Vector(adaptor.Value(u)).sub(near).length)
    props = BRepLProp_CLProps(adaptor, 2, 1e-9)
    props.SetParameter(parameter)
    return abs(props.Curvature())


def curvature_jumps(wire: Wire) -> list[float]:
    """|k_left - k_right| at every vertex where two edges of the wire meet."""
    edges = wire.edges()
    jumps = []
    for index, edge in enumerate(edges):
        following = edges[(index + 1) % len(edges)]
        shared = _shared_point(edge, following)
        if shared is not None:
            jumps.append(abs(edge_curvature(edge, shared) - edge_curvature(following, shared)))
    return jumps


def _shared_point(left: Edge, right: Edge) -> Vector | None:
    for left_point in (left @ 0, left @ 1):
        for right_point in (right @ 0, right @ 1):
            if (left_point - right_point).length < 1e-5:
                return left_point
    return None


# --- G2 skin -----------------------------------------------------------------------------------


class Roll(NamedTuple):
    """G2 roll from a cap into the wall. `setback` runs along the cap and up the wall."""

    setback: float


class BedLead(NamedTuple):
    """Print-bed edge: a 45 degree lead `leg` wide at the bed, then a G2 blend `rise` up the wall.

    The lead prints at the 45 degree limit. The blend turns from 45 degrees to vertical,
    so no point of it overhangs more than the lead does.
    """

    leg: float
    rise: float


EdgeRule = Roll | BedLead
ProfileCurve = list[Vector]  # 2 points: a line; 3: a parabola; 6: a quintic
LEAD_STRAIGHT = 0.35  # share of the 45 degree lead left straight next to the crease
LEAD_REACH = 0.45  # blend handles as a share of the lead and rise: keeps the polygon convex


def _wall(height: float, low: float, high: float, bulge: float) -> ProfileCurve:
    """Wall from (0, low) to (0, height - high). A bulge makes it an exact parabola."""
    start, stop = Vector(0, low), Vector(0, height - high)
    if bulge <= 0:
        return [start, stop]
    return [start, Vector(-2 * bulge, (low + height - high) / 2), stop]


def _wall_frame(wall: ProfileCurve, at_start: bool) -> Frame:
    """Travel frame of the wall at one end, read from its control points."""
    if len(wall) == 2:
        tangent = (wall[1] - wall[0]).normalized()
        return Frame(wall[0] if at_start else wall[1], tangent, Vector(1, 0), 0.0)
    p0, p1, p2 = wall if at_start else wall[::-1]
    first = (p1 - p0) * 2
    second = (p0 - p1 * 2 + p2) * 2
    curvature = abs(first.X * second.Y - first.Y * second.X) / first.length ** 3
    tangent = first.normalized() if at_start else -first.normalized()
    inward = Vector(tangent.Y, -tangent.X)  # right of travel: the inside of a bulge
    return Frame(p0, tangent, inward, curvature)


def _bottom_end(rule: EdgeRule, wall: Frame) -> list[ProfileCurve]:
    """Curves from the bottom cap edge up to the wall, travelling outward and up."""
    if isinstance(rule, Roll):
        size = rule.setback
        start = Frame(Vector(size, 0), Vector(-1, 0), Vector(0, 1), 0.0)
        return [g2_points(start, wall, 0.4 * size, 0.4 * size)]
    crease = Vector(rule.leg, 0)
    lead = Vector(-1, 1).normalized()
    corner = Vector(0, rule.leg)
    leave = crease + lead * (LEAD_STRAIGHT * rule.leg * sqrt(2))
    start = Frame(leave, lead, Vector(1, 1).normalized(), 0.0)
    reach = LEAD_REACH * (corner - leave).length
    return [[crease, leave], g2_points(start, wall, reach, LEAD_REACH * rule.rise)]


def _end_height(rule: EdgeRule) -> float:
    return rule.setback if isinstance(rule, Roll) else rule.leg + rule.rise


def _flip(curve: ProfileCurve, height: float) -> ProfileCurve:
    return [Vector(p.X, height - p.Y) for p in reversed(curve)]


def _flip_frame(frame: Frame, height: float) -> Frame:
    point = Vector(frame.point.X, height - frame.point.Y)
    tangent = Vector(-frame.tangent.X, frame.tangent.Y)
    normal = Vector(frame.normal.X, -frame.normal.Y)
    return Frame(point, tangent, normal, frame.curvature)


def _edge(curve: ProfileCurve) -> Edge:
    if len(curve) == 2:
        return Line(curve[0], curve[1])
    return Bezier(*curve)


def skin_profile(height: float, bottom: EdgeRule, top: EdgeRule, bulge: float = 0.0) -> Wire:
    """Edge profile in (inward, up): bottom cap edge, roll, wall, roll, top cap edge.

    Every join is G2 except the crease of a BedLead, which sits on the bed.
    """
    low, high = _end_height(bottom), _end_height(top)
    if low + high >= height:
        raise ValueError(f"edge rules ({low:g} + {high:g}) do not fit in {height:g} mm")
    wall = _wall(height, low, high, bulge)
    lower = _bottom_end(bottom, _wall_frame(wall, at_start=True))
    upper_frame = _flip_frame(_wall_frame(wall, at_start=False), height)
    upper = [_flip(curve, height) for curve in reversed(_bottom_end(top, upper_frame))]
    return Wire([_edge(curve) for curve in [*lower, wall, *upper]])


def _counter_clockwise(outline: Wire) -> Wire:
    points = [outline @ (k / 256) for k in range(256)]
    area = sum(a.X * b.Y - b.X * a.Y for a, b in zip(points, points[1:] + points[:1]))
    if area > 0:
        return outline
    return Wire([edge.reversed() for edge in reversed(outline.edges())])


def _cap(faces: list[Face], z: float) -> Face:
    edges = [edge for face in faces for edge in face.edges()
             if abs(edge.center().Z - z) < 1e-6 and edge.bounding_box().size.Z < 1e-6]
    return Face(Wire(edges))


def g2_skin(outline: Wire, height: float, bottom: EdgeRule, top: EdgeRule,
            bulge: float = 0.0) -> Part:
    """Solid on a planar XY outline, z 0..height, every outer edge a G2 roll or a bed lead.

    The profile is swept round the outline (corrected Frenet keeps it upright on a plane
    curve), the two caps are planar faces on the swept boundary edges, and the faces are
    sewn. A roll larger than the outline's smallest corner radius would fold the sweep,
    and the solid check fails.
    """
    path = _counter_clockwise(outline)
    tangent = path % 0
    inward = Vector(0, 0, 1).cross(tangent).normalized()
    frame = Plane(origin=path @ 0, x_dir=inward, z_dir=tangent)
    profile = frame.location * skin_profile(height, bottom, top, bulge)
    faces = sweep(profile, path).faces()
    sewing = BRepBuilderAPI_Sewing(1e-6)
    for face in [*faces, _cap(faces, 0.0), _cap(faces, height)]:
        sewing.Add(face.wrapped)
    sewing.Perform()
    raw = BRepBuilderAPI_MakeSolid(TopoDS.Shell_s(sewing.SewedShape())).Solid()
    BRepLib.OrientClosedSolid_s(raw)
    solid = Solid(raw)
    if not solid.is_valid or solid.volume <= 0:
        raise ValueError("G2 skin did not close into a valid solid")
    return Part([solid])
