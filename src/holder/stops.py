"""Spring-stop joints: one fastener rule for every joint of the device.

An ISO 7379 shoulder screw passes the near part and threads into a heat-set insert in the
far part. Under its head sit a washer (M6 only), a stack of DIN 2093 disc springs in
series, and a seat washer on the near part's PETG. Tightened, the shoulder's end lands on
the insert's brass face: a positive stop. Past that point any extra torque closes a
steel-and-brass loop, so the PETG and the insert only ever see the spring force.

The spring force is set by geometry: shoulder length minus the washers and the clamped
PETG gives the compressed stack height, and the published DIN 2093 table gives the force.
"Turn until it stops" is the whole torque spec. The springs also follow PETG creep.
"""

from dataclasses import dataclass

from build123d import Axis, Part, Plane, Polygon, Vector, revolve

from holder.common import (
    MODEL_GAP,
    Check,
    SeatedInsert,
    annulus_footprint,
    any_perpendicular,
    at_least,
    bearing_row,
    contact_area,
    first_hit,
    hex_prism,
    insert_shear_row,
    plane_at,
    rod,
    void_open,
    washer_solid,
    within,
)
from holder.params import (
    INSERT_SINK,
    PRINT_TOL,
    DiscSpring,
    Insert,
    ShoulderScrew,
    Washer,
)

FLAT_MARGIN = 0.95  # a spring past 0.95 h0 is close to flat: its force climbs fast
SOFT_MARGIN = 0.15  # a spring under 0.15 h0 barely pushes


@dataclass(frozen=True)
class StopSpec:
    """The parts of one stop and the deflection they are designed to sit at."""

    label: str
    screw: ShoulderScrew
    spring: DiscSpring
    count: int
    head_washer: Washer | None  # between head and springs; None: the head bears on a spring
    seat_washer: Washer
    insert: Insert
    target: float  # design deflection per spring, share of h0
    l1: float  # shoulder length (a stock length)

    @property
    def spring_height(self) -> float:
        """Design height of one compressed spring."""
        return self.spring.free_height - self.target * self.spring.h0

    @property
    def above_seat(self) -> float:
        """Seat washer, springs and head washer: the stack between the PETG and the head."""
        head = self.head_washer.t if self.head_washer else 0.0
        return self.seat_washer.t + self.count * self.spring_height + head

    @property
    def clamped(self) -> float:
        """Design distance from the seat (PETG under the seat washer) to the insert top."""
        return self.l1 - self.above_seat

    def deflection(self, clamped: float, l1: float) -> float:
        """Per-spring deflection when the shoulder has stopped."""
        head = self.head_washer.t if self.head_washer else 0.0
        height = (l1 - clamped - self.seat_washer.t - head) / self.count
        return self.spring.free_height - height

    def force(self, clamped: float, l1: float) -> float:
        return self.spring.force(self.deflection(clamped, l1))

    def band(self, clamped: float, spread: float) -> tuple[float, float, float]:
        """(min, nominal, max) force. Min: longest shoulder, thinnest stack. Max: shortest
        shoulder, thickest stack. `spread` is the clamped-length tolerance, +-."""
        low = self.deflection(clamped - spread, self.l1 + self.screw.l1_plus)
        high = self.deflection(clamped + spread, self.l1)
        return (self.spring.force(max(low, 0.0)), self.force(clamped, self.l1),
                self.spring.force(min(high, self.spring.h0)))

    def backout(self, lift: float) -> float:
        """Turns of thread, in mm, to free the springs and then open the joint by `lift`."""
        return self.count * self.target * self.spring.h0 + lift + 0.5


@dataclass(frozen=True)
class Placed:
    """A stop placed on its parts. `seat` is the point on the near part's face that the
    seat washer sits on; `axis` points from the head toward the insert."""

    spec: StopSpec
    seat: Vector
    axis: Vector

    @property
    def insert_top(self) -> Vector:
        return self.seat + self.axis * self.spec.clamped

    @property
    def head_under(self) -> Vector:
        return self.insert_top - self.axis * (self.spec.l1 + MODEL_GAP)

    def seated_insert(self, tip_room: float = 0.5, tip: Vector | None = None) -> SeatedInsert:
        face = self.insert_top - self.axis * INSERT_SINK
        return SeatedInsert(f"{self.spec.label}", self.spec.insert, face, self.axis,
                            tip_room, tip)

    # --- Bought solids ---------------------------------------------------------------------

    def screw_solid(self) -> Part:
        screw, axis = self.spec.screw, self.axis
        plane = plane_at(self.head_under, axis, any_perpendicular(axis))
        head = rod(screw.head_d, screw.head_k, (0, 0, -screw.head_k), (0, 0, 1))
        head -= hex_prism(screw.key, screw.head_k * 0.6,
                          Plane(origin=(0, 0, -screw.head_k - 0.01)))
        shoulder = rod(screw.shoulder_d, self.spec.l1, (0, 0, 0), (0, 0, 1))
        thread = rod(screw.d, screw.thread_b, (0, 0, self.spec.l1), (0, 0, 1))
        return plane.location * (head + shoulder + thread)

    def washer_solids(self) -> list[tuple[str, Washer, Part]]:
        seat = self.seat - self.axis * MODEL_GAP
        solids = [("seat washer", self.spec.seat_washer,
                   washer_solid(self.spec.seat_washer, seat, -self.axis))]
        if self.spec.head_washer:
            top = self.head_under + self.axis * MODEL_GAP
            solids.append(("head washer", self.spec.head_washer,
                           washer_solid(self.spec.head_washer, top, self.axis)))
        return solids

    def spring_solids(self) -> list[Part]:
        """The disc springs at their stopped height, alternating, as conical shells."""
        spec = self.spec
        head = spec.head_washer.t if spec.head_washer else 0.0
        base = self.seat - self.axis * (spec.seat_washer.t + 2 * MODEL_GAP)
        room = spec.l1 - spec.clamped - spec.seat_washer.t - head - 3 * MODEL_GAP
        height = room / spec.count
        solids = []
        for k in range(spec.count):
            start = base - self.axis * (k * height)
            solids.append(_disc(spec.spring, height - MODEL_GAP, start, -self.axis,
                                inner_up=k % 2 == 0))
        return solids

    # --- Rows ------------------------------------------------------------------------------

    def measured_clamped(self, near: Part, insert: Part) -> float:
        """Seat face to insert top along the axis, read by rays on the placed solids."""
        side = any_perpendicular(self.axis)
        reach = (self.spec.seat_washer.inner + self.spec.seat_washer.outer) / 4
        start = self.seat - self.axis * 30 + side * reach
        seat = start + self.axis * first_hit(near, start, self.axis)
        ring = (self.spec.screw.shoulder_d / 2 + self.spec.insert.minor_d / 2) / 2
        probe = self.seat - self.axis * 1.0 + side * ring
        top = probe + self.axis * first_hit(insert, probe, self.axis)
        return (top - seat).dot(self.axis)

    def rows(self, near: Part, far: Part, insert: Part, spread: float = PRINT_TOL,
             need: float = 0.0, need_name: str = "") -> list[Check]:
        spec, label = self.spec, self.spec.label
        clamped = self.measured_clamped(near, insert)
        rows = [within(f"{label}: seat to insert top is the design length", clamped,
                       spec.clamped - 0.05, spec.clamped + 0.05)]
        rows += self.force_rows(near, clamped - spread, clamped + spread, need, need_name)
        rows += self.thread_rows(far)
        rows.append(self.shoulder_on_brass(far))
        return rows

    def force_rows(self, near: Part, low: float, high: float, need: float,
                   need_name: str) -> list[Check]:
        """Spring deflection and force over the band of clamped lengths [low, high]: the
        least force at the longest shoulder and `low`, the most at the shortest and `high`."""
        spec, label = self.spec, self.spec.label
        low_s = spec.deflection(low, spec.l1 + spec.screw.l1_plus)
        high_s = spec.deflection(high, spec.l1)
        h0 = spec.spring.h0
        rows = [
            within(f"{label}: spring deflection over the tolerance band, share of h0",
                   min(low_s, high_s) / h0, SOFT_MARGIN, FLAT_MARGIN, ""),
            within(f"{label}: spring deflection, other end of the band", max(low_s, high_s)
                   / h0, SOFT_MARGIN, FLAT_MARGIN, ""),
        ]
        if not all(row.ok for row in rows):
            return rows
        least, most = spec.spring.force(low_s), spec.spring.force(high_s)
        rows.append(Check(f"{label}: stop force over the band (DIN 2093 table)", True,
                          f"{least:.0f} .. {most:.0f} N"))
        if need:
            rows.append(at_least(f"{label}: least stop force covers {need_name}", least, need,
                                 "N"))
        rows.append(insert_shear_row(f"{label} at the most stop force", spec.insert, most))
        seat = annulus_footprint(self.seat, self.axis, spec.seat_washer.outer,
                                 spec.seat_washer.inner)
        rows.append(bearing_row(f"{label} seat washer at the most stop force", most,
                                contact_area(near, seat)))
        return rows

    def thread_rows(self, far: Part) -> list[Check]:
        """Thread fully in the insert at the stop; room for the tip past it; enough thread
        left when the joint is backed out to re-index."""
        spec, label = self.spec, self.spec.label
        engaged = min(spec.screw.thread_b, spec.insert.length)
        floor = first_hit(far, self.insert_top + self.axis * 0.3, self.axis) + 0.3
        rows = [at_least(f"{label}: thread in the insert at the stop", engaged,
                         1.5 * spec.screw.d),
                at_least(f"{label}: hole past the thread tip (measured)",
                         floor - spec.screw.thread_b, 0.3)]
        return rows

    def open_row(self, lift: float) -> Check:
        spec = self.spec
        left = min(spec.screw.thread_b, spec.insert.length) - spec.backout(lift)
        return at_least(f"{spec.label}: thread left in the insert, backed out {lift:g} mm "
                        "past the springs", left, 0.8 * spec.screw.d)

    def shoulder_on_brass(self, far: Part) -> Check:
        """The shoulder end, at the stop, meets brass only: no PETG under it."""
        spec = self.spec
        foot = annulus_footprint(self.insert_top - self.axis * 0.05, self.axis,
                                 spec.screw.shoulder_d, spec.screw.d)
        return void_open(far, foot, f"{spec.label}: shoulder stops on the insert, not PETG")


def _disc(spring: DiscSpring, height: float, start: Vector, axis: Vector,
          inner_up: bool) -> Part:
    """One disc spring as a conical shell, `height` tall from `start` along `axis`."""
    ri, ro, t = spring.inner / 2, spring.outer / 2, spring.t
    rise = max(height - t, 0.0)
    if inner_up:
        points = [(ro, 0), (ro, t), (ri, rise + t), (ri, rise)]
    else:
        points = [(ri, 0), (ri, t), (ro, rise + t), (ro, rise)]
    section = Plane.XZ.location * Polygon(*points, align=None)
    body = revolve(section, Axis.Z, 360)
    plane = plane_at(start, axis, any_perpendicular(axis))
    return plane.location * body

