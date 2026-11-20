"""MF-40 mount face and the foot that bolts to it.

A mount face (MF) is 4 x M3 on a 20 mm square into captive nuts 2.0 under the face,
plus one 1/4"-20 port nut 6.1 under the face. The nuts slide in along `mouth` from a
face of the part (on the collar that face is the bore, so the liner hides them).
A foot is the same 40 x 40 soft square with 4 counterbored M3 holes. Every foot keeps
4.7 mm under the head, so one M3 length fits every foot on every MF.
"""

from dataclasses import dataclass

from build123d import Face, Part, Pos, Vector, extrude

from holder.common import (
    GABLE_FAR,
    CaptiveNut,
    Check,
    Stack,
    at_least,
    encloses,
    first_hit,
    lead_in,
    rod,
    thread_need,
)
from holder.form import soft_rect
from holder.params import (
    M3_CLEAR,
    M3_HEAD_BORE,
    M3_NUT,
    M3_SHCS,
    PETG_WALL,
    QUARTER_CLEAR,
    QUARTER_NUT,
    QUARTER_TIP_FLOOR,
    Params,
)

DOWN = Vector(0, 0, -1)


def m3_square(spec: Params) -> list[tuple[float, float]]:
    half = spec.mf_m3_spacing / 2
    return [(-half, -half), (half, -half), (-half, half), (half, half)]


def foot_screw_length(spec: Params) -> float:
    """One M3 length for every foot: under-head plus the MF roof plus the nut."""
    grip = spec.foot_under_head + spec.mf_m3_roof
    return M3_SHCS.stock_at_least(grip + thread_need(M3_SHCS, M3_NUT) + M3_SHCS.length_tol)


def m3_tip_floor(spec: Params) -> float:
    """Depth under the face of the M3 tip bore floor: long screw tip plus 1.0."""
    grip = spec.foot_under_head + spec.mf_m3_roof
    tip = foot_screw_length(spec) + M3_SHCS.length_tol - grip
    return spec.mf_m3_roof + tip + 1.0


@dataclass(frozen=True)
class MountFace:
    """An MF on a part. `centre` is on the face (normal +Z). Nuts leave along `mouth`."""

    label: str
    centre: Vector
    mouth: Vector
    mouth_at: dict[tuple[float, float], float]  # per nut (dx, dy): distance to past the mouth
    quarter_roof: float = 6.1
    quarter_gable: str = GABLE_FAR

    def nuts(self, spec: Params) -> list[CaptiveNut]:
        """Far nut of each channel first: that is the order they go in."""
        top = self.centre.Z
        found = []
        squares = sorted(m3_square(spec), key=lambda xy: -self._along(xy))
        for dx, dy in squares:
            seat = Vector(self.centre.X + dx, self.centre.Y + dy, top - spec.mf_m3_roof)
            found.append(CaptiveNut(f"{self.label} M3 nut ({dx:+g},{dy:+g})", M3_NUT, seat,
                                    DOWN, self.mouth, self.mouth_at[(dx, dy)]))
        seat = Vector(self.centre.X, self.centre.Y, top - self.quarter_roof)
        found.append(CaptiveNut(f"{self.label} 1/4\" nut", QUARTER_NUT, seat, DOWN, self.mouth,
                                self.mouth_at[(0.0, 0.0)], gable=self.quarter_gable))
        return found

    def _along(self, offset: tuple[float, float]) -> float:
        return offset[0] * self.mouth.X + offset[1] * self.mouth.Y

    def holes(self, spec: Params) -> list[Part]:
        top = self.centre.Z
        cutters = []
        m3_depth = m3_tip_floor(spec)
        for dx, dy in m3_square(spec):
            at = Vector(self.centre.X + dx, self.centre.Y + dy, top)
            cutters += [rod(M3_CLEAR, m3_depth + 1, at + Vector(0, 0, 1), DOWN),
                        lead_in(M3_CLEAR, at, DOWN)]
        at = Vector(self.centre.X, self.centre.Y, top)
        cutters += [rod(QUARTER_CLEAR, QUARTER_TIP_FLOOR + 1, at + Vector(0, 0, 1), DOWN),
                    lead_in(QUARTER_CLEAR, at, DOWN)]
        return cutters

    def cutters(self, spec: Params) -> list[Part]:
        voids = self.holes(spec)
        for nut in self.nuts(spec):
            voids += nut.cutters()
        return voids

    def checks(self, part: Part, spec: Params) -> list[Check]:
        rows: list[Check] = []
        nuts = self.nuts(spec)
        for nut in nuts:
            rows += nut.checks(part)
            hole = M3_CLEAR if nut.nut is M3_NUT else QUARTER_CLEAR
            shared = [cut for other in nuts if other is not nut and _same_line(other, nut)
                      for cut in other.cutters()]
            for name, probe in nut.wall_probes(PETG_WALL, hole):
                for cutter in shared:
                    probe -= cutter
                rows.append(encloses(part, probe, f"{name} >= {PETG_WALL}"))
        quarter = nuts[-1].pocket() + nuts[-1].channel()
        for nut in nuts[:-1]:
            gap = (nut.pocket() + nut.channel()).distance_to(quarter)
            rows.append(at_least(f"{nut.label} to 1/4\" channel", gap, PETG_WALL))
        rows.append(at_least(f"{self.label} 1/4\" tip floor wall",
                             floor_wall(part, self.centre), PETG_WALL))
        return rows


def _same_line(left: CaptiveNut, right: CaptiveNut) -> bool:
    """Two nuts share one channel: same depth, and offset only along travel."""
    offset = left.seat - right.seat
    along = offset.dot(left.travel)
    return (offset - left.travel * along).length < 1e-6


def floor_wall(part: Part, centre: Vector) -> float:
    """PETG under the 1/4" tip floor, on the port axis, read from the real faces."""
    start = Vector(centre.X, centre.Y, centre.Z - QUARTER_TIP_FLOOR - 0.01)
    return first_hit(part, start, DOWN) + 0.01


# --- Foot ------------------------------------------------------------------------------------


def foot_plan(spec: Params, centre: tuple[float, float] = (0, 0)) -> Face:
    return soft_rect(spec.mf_size, spec.mf_size, spec.mf_setback, centre)


def foot_slab(spec: Params, thickness: float, centre: tuple[float, float] = (0, 0)) -> Part:
    return extrude(foot_plan(spec, centre), amount=thickness)


def foot_cutters(spec: Params, thickness: float, centre: tuple[float, float] = (0, 0),
                 top: float | None = None) -> list[Part]:
    """4 x M3 counterbores. The seat stays foot_under_head above the bottom face.

    `top` is where the counterbore opens, when material above the foot (a wall) carries it.
    """
    opening = thickness if top is None else top
    seat = spec.foot_under_head
    head_d, _head_k = M3_HEAD_BORE
    cutters = []
    for dx, dy in m3_square(spec):
        x, y = centre[0] + dx, centre[1] + dy
        cutters += [
            rod(M3_CLEAR, seat + 2, Vector(x, y, -1), Vector(0, 0, 1)),
            rod(head_d, opening - seat + 1, Vector(x, y, seat), Vector(0, 0, 1)),
            lead_in(head_d, Vector(x, y, opening), DOWN),
        ]
    return cutters


def foot_checks(part: Part, spec: Params, centre: tuple[float, float] = (0, 0)) -> list[Check]:
    """Each counterbore seat sits foot_under_head above the bottom face, read by rays."""
    up = Vector(0, 0, 1)
    rows = []
    for dx, dy in m3_square(spec):
        probe = Vector(centre[0] + dx + M3_CLEAR / 2 + 0.6, centre[1] + dy, -5)
        bottom = probe.Z + first_hit(part, probe, up)
        seat = bottom + 0.01 + first_hit(part, Vector(probe.X, probe.Y, bottom + 0.01), up)
        rows.append(at_least(f"foot ({dx:+g},{dy:+g}) under head", seat - bottom,
                             spec.foot_under_head - 0.01))
    return rows


def foot_stack(spec: Params, grip_open: float, tip_room: float, label: str,
               head_sink: float) -> Stack:
    return Stack(label, M3_SHCS, grip_open, grip_open, thread_need(M3_SHCS, M3_NUT),
                 tip_room - 0.5, head_sink)


def offset_xy(part: Part, x: float, y: float) -> Part:
    return Pos(x, y, 0) * part
