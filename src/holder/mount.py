"""MF-40 mount face and the foot that bolts to it.

A mount face (MF) is 4 x M3 heat-set inserts on a 20 mm square, plus one 1/4"-20 port
nut 6.1 under the face. The port nut slides in along `mouth` from a face of the part (on
the collar that face is the bore, so the liner hides it) and drops QUARTER_DROP into its
pocket, so it cannot slide back out before a stud goes in.
A foot is the same 40 x 40 soft square with 4 counterbored M3 holes for screw + washer.
Every foot keeps foot_under_head of PETG under the washer, so one M3 length fits every
foot on every MF.
"""

from dataclasses import dataclass

from build123d import Face, Part, Vector

from holder.common import (
    GABLE_FAR,
    CaptiveNut,
    Check,
    SeatedInsert,
    annulus_footprint,
    at_least,
    bearing_row,
    contact_area,
    encloses,
    first_hit,
    hex_prism,
    insert_shear_row,
    lead_in,
    rod,
)
from holder.form import BedLead, Roll, g2_skin, soft_rect
from holder.params import (
    INSERT_ENGAGE,
    INSERT_SINK,
    M3_CLEAR,
    M3_INSERT,
    M3_SHCS,
    M3_WASHER,
    M3_WASHER_BORE,
    PETG_WALL,
    PRELOAD_N,
    QUARTER_CLEAR,
    QUARTER_DROP,
    QUARTER_NUT,
    QUARTER_NUT_AF_MIN,
    QUARTER_TIP_FLOOR,
    Params,
)

DOWN = Vector(0, 0, -1)
FOOT_TOP_ROLL = Roll(2.5)
FOOT_BED = BedLead(0.6, 2.0)


def m3_square(spec: Params) -> list[tuple[float, float]]:
    half = spec.mf_m3_spacing / 2
    return [(-half, -half), (half, -half), (-half, half), (half, half)]


def foot_grip(spec: Params) -> float:
    """Under the head to the MF face: PETG under the washer, plus the washer."""
    return spec.foot_under_head + M3_WASHER.t


def foot_screw_length(spec: Params) -> float:
    """One M3 length for every foot: grip, the insert sink, 1.5 d in the insert."""
    need = foot_grip(spec) + INSERT_SINK + INSERT_ENGAGE + M3_SHCS.length_tol
    return M3_SHCS.stock_at_least(need)


MF_TIP_ROOM = INSERT_SINK + M3_INSERT.length + 0.5  # the insert hole is the tip bore


@dataclass(frozen=True)
class MountFace:
    """An MF on a part. `centre` is on the face (normal +Z). The port nut leaves along `mouth`."""

    label: str
    centre: Vector
    mouth: Vector
    quarter_reach: float  # port nut seat centre to past the mouth
    quarter_roof: float = 6.1
    quarter_gable: str = GABLE_FAR

    def inserts(self, spec: Params) -> list[SeatedInsert]:
        return [SeatedInsert(f"{self.label} M3 ({dx:+g},{dy:+g})", M3_INSERT,
                             Vector(self.centre.X + dx, self.centre.Y + dy, self.centre.Z),
                             DOWN, MF_TIP_ROOM, M3_CLEAR)
                for dx, dy in m3_square(spec)]

    def port_nut(self) -> CaptiveNut:
        seat = Vector(self.centre.X, self.centre.Y, self.centre.Z - self.quarter_roof)
        return CaptiveNut(f"{self.label} 1/4\" nut", QUARTER_NUT, seat, DOWN, self.mouth,
                          self.quarter_reach, gable=self.quarter_gable, drop=QUARTER_DROP)

    def cutters(self, spec: Params) -> list[Part]:
        voids = []
        for insert in self.inserts(spec):
            voids += insert.cutters()
        at = self.centre
        voids += [rod(QUARTER_CLEAR, QUARTER_TIP_FLOOR + 1, at + Vector(0, 0, 1), DOWN),
                  lead_in(QUARTER_CLEAR, at, DOWN), *self.port_nut().cutters()]
        return voids

    def checks(self, part: Part, spec: Params) -> list[Check]:
        nut = self.port_nut()
        rows = nut.checks(part)
        for name, probe in nut.wall_probes(PETG_WALL, QUARTER_CLEAR):
            rows.append(encloses(part, probe, f"{name} >= {PETG_WALL}"))
        for insert in self.inserts(spec):
            rows += insert.checks(part)
            gap = rod(M3_INSERT.outer_d, M3_INSERT.length, insert.top, DOWN).distance_to(
                nut.pocket() + nut.channel())
            rows.append(at_least(f"{insert.label} insert to the 1/4\" channel", gap,
                                 M3_INSERT.wall_min))
        rows.append(insert_shear_row(f"{self.label} M3", M3_INSERT, PRELOAD_N["m3"]))
        rows.append(self.port_bearing(part))
        rows.append(at_least(f"{self.label} 1/4\" tip floor wall",
                             floor_wall(part, self.centre), PETG_WALL))
        return rows

    def port_bearing(self, part: Part) -> Check:
        """The port nut pulled onto its roof by a stud at PRELOAD_N['port']: min nut, real roof."""
        nut = self.port_nut()
        face = hex_prism(QUARTER_NUT_AF_MIN, 0.05, nut.plane.offset(-0.05))
        face -= rod(QUARTER_CLEAR + 0.02, 1.0, nut.seat - DOWN * 0.5, DOWN)
        area = contact_area(part, face)
        return bearing_row(f"{self.label} 1/4\" port nut", PRELOAD_N["port"], area)


def floor_wall(part: Part, centre: Vector) -> float:
    """PETG under the 1/4" tip floor, on the port axis, read from the real faces."""
    start = Vector(centre.X, centre.Y, centre.Z - QUARTER_TIP_FLOOR - 0.01)
    return first_hit(part, start, DOWN) + 0.01


# --- Foot ------------------------------------------------------------------------------------


def foot_plan(spec: Params, centre: tuple[float, float] = (0, 0)) -> Face:
    return soft_rect(spec.mf_size, spec.mf_size, spec.mf_setback, centre)


def foot_slab(spec: Params, thickness: float) -> Part:
    """The foot as a G2 skin: bed lead underneath (it prints foot down), G2 roll on top."""
    return g2_skin(foot_plan(spec).outer_wire(), thickness, FOOT_BED, FOOT_TOP_ROLL)


def foot_cutters(spec: Params, thickness: float, centre: tuple[float, float] = (0, 0),
                 top: float | None = None) -> list[Part]:
    """4 x M3 counterbores for screw + washer. The seat stays foot_under_head up.

    `top` is where the counterbore opens, when material above the foot (a wall) carries it.
    """
    opening = thickness if top is None else top
    seat = spec.foot_under_head
    bore_d, _depth = M3_WASHER_BORE
    cutters = []
    for dx, dy in m3_square(spec):
        x, y = centre[0] + dx, centre[1] + dy
        cutters += [
            rod(M3_CLEAR, seat + 2, Vector(x, y, -1), Vector(0, 0, 1)),
            rod(bore_d, opening - seat + 1, Vector(x, y, seat), Vector(0, 0, 1)),
            lead_in(bore_d, Vector(x, y, opening), DOWN),
        ]
    return cutters


def foot_checks(part: Part, spec: Params, centre: tuple[float, float] = (0, 0)) -> list[Check]:
    """Each washer seat sits foot_under_head above the bottom face, read by rays; and the
    washer at the M3 preload bears on the real seat."""
    up = Vector(0, 0, 1)
    rows = []
    for dx, dy in m3_square(spec):
        probe = Vector(centre[0] + dx + M3_CLEAR / 2 + 0.6, centre[1] + dy, -5)
        bottom = probe.Z + first_hit(part, probe, up)
        seat = bottom + 0.01 + first_hit(part, Vector(probe.X, probe.Y, bottom + 0.01), up)
        rows.append(at_least(f"foot ({dx:+g},{dy:+g}) under the washer", seat - bottom,
                             spec.foot_under_head - 0.01))
        footprint = annulus_footprint(Vector(centre[0] + dx, centre[1] + dy, seat), DOWN,
                                      M3_WASHER.outer, M3_WASHER.inner)
        rows.append(bearing_row(f"foot ({dx:+g},{dy:+g}) washer", PRELOAD_N["m3"],
                                contact_area(part, footprint)))
    return rows
