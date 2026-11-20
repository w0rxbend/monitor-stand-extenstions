"""Plan and profile curves: G2 corners and blends, teardrops, and a curvature probe.

A corner is a quintic Bezier with three control points on each side. Its curvature is
zero where it meets a straight side, so the join has no visible line (G2). The general
blend matches position, tangent, and curvature at both ends, so it also joins a circle.
Sketches holding a Bezier are never offset: the STEP writer drops OFFSET-curve faces.
Rebuild the outline at the new size instead.
"""

from math import cos, radians, sin, sqrt
from typing import NamedTuple

from build123d import (
    Bezier,
    CenterArc,
    Edge,
    Face,
    Line,
    Vector,
    Wire,
    make_face,
)
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepLProp import BRepLProp_CLProps

HANDLE_FAR = 0.6  # share of the setback; ported from macropad solids._corner_curve
HANDLE_NEAR = 0.2
Point = tuple[float, float]


class Frame(NamedTuple):
    """Curve end: point, unit travel tangent, unit normal toward the curvature centre, k."""

    point: Vector
    tangent: Vector
    normal: Vector
    curvature: float


def g2_blend(start: Frame, end: Frame, reach_start: float, reach_end: float) -> Bezier:
    """Quintic Bezier that matches both frames to second order.

    For a quintic, curvature at t = 0 is 4/5 * h / a^2, where a = |P1 - P0| and h is
    P2's offset from the tangent line. So h = 5 a^2 k / 4. The end mirrors the start.
    """
    a0, a1 = reach_start, reach_end
    p1 = start.point + start.tangent * a0
    p2 = p1 + start.tangent * a0 + start.normal * (5 * a0 * a0 * start.curvature / 4)
    p4 = end.point - end.tangent * a1
    p3 = p4 - end.tangent * a1 + end.normal * (5 * a1 * a1 * end.curvature / 4)
    return Bezier(start.point, p1, p2, p3, p4, end.point)


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
