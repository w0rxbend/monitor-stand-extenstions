"""Shared geometry tools. hex_pocket is copied from Tigerbee 7inch accessories/_common.py."""

from math import cos, radians

from build123d import Axis, Part, Pos, RegularPolygon, extrude


def hex_pocket(
    af: float,
    depth: float,
    at: tuple[float, float, float] = (0, 0, 0),
    rotation_deg: float = 0.0,
) -> Part:
    """Hexagonal prism tool (across-flats `af`) from Z at[2] up to at[2] + depth; subtract it."""
    x, y, z = at
    return Pos(x, y, z) * extrude(
        RegularPolygon(af / 2 / cos(radians(30)), 6).rotate(Axis.Z, rotation_deg),
        amount=depth,
    )
