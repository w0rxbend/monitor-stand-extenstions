"""Every dimension the parts are built from. Bought parts carry their published source.

A value with no published source is marked `estimated:` and stays an estimate until
it is measured on a printed coupon or a supplier sheet.
"""

import math
from dataclasses import dataclass, fields

MATERIALS = {
    "TPU95A": {"wall_min": 1.2, "fit": 0.25},
    "PETG": {"wall_min": 1.5, "fit": 0.30},
}
PETG_WALL = MATERIALS["PETG"]["wall_min"]


@dataclass(frozen=True, slots=True)
class Nut:
    """Hex nut: published max sizes, and the pocket the parts cut for it (max + fit)."""

    name: str
    standard: str
    af_max: float
    m_max: float
    minor_d: float  # internal thread minor diameter; the screw overlaps it on purpose
    pocket_af: float
    pocket_m: float
    source: str


@dataclass(frozen=True, slots=True)
class Screw:
    """Screw with a socket head (ISO 4762, ASME B18.3) or a carriage head (DIN 603).

    `head_d` is the max head diameter, `bearing_d` the min one (what is sure to bear).
    A carriage bolt has a square neck `neck_w` wide and `neck_h` tall under its dome head.
    """

    name: str
    standard: str
    d: float
    pitch: float
    head_d: float
    bearing_d: float
    head_k: float
    key: float  # hex key across flats; 0 for a carriage bolt (the neck stops it turning)
    stock: tuple[float, ...]
    length_tol: float  # estimated: ISO 4759-1 js15 band, not checked per length
    source: str
    neck_w: float = 0.0
    neck_h: float = 0.0

    def stock_at_least(self, length: float) -> float:
        """Shortest stock length that is at least `length`."""
        for size in self.stock:
            if size >= length - 1e-9:
                return size
        raise ValueError(f"no stock {self.name} covers {length:.2f} mm")


@dataclass(frozen=True, slots=True)
class Washer:
    name: str
    standard: str
    inner: float
    outer: float
    t: float
    source: str


@dataclass(frozen=True, slots=True)
class Insert:
    """Heat-set brass insert: knurl diameter, the hole it is pressed into, its length."""

    name: str
    standard: str
    outer_d: float
    hole_d: float
    length: float
    wall_min: float  # maker's minimum PETG round the knurl
    source: str


INCH = 25.4

# Wermac ISO 4032 table; HB&H cut sheet. Minor diameter D1 from ISO 965-1.
M6_NUT = Nut("M6 nut", "ISO 4032", 10.00, 5.20, 4.917, 10.20, 5.50, "Wermac ISO 4032; HB&H")
# amesweb ASME B18.2.2 hex nut table: F 0.428-0.438 in, H 0.212-0.226 in. Minor: ASME B1.1.
QUARTER_NUT = Nut(
    "1/4\"-20 hex nut", "ASME B18.2.2", 11.13, 5.74, 0.1887 * INCH, 11.45, 6.05,
    "amesweb ASME B18.2.2 table",
)
QUARTER_NUT_AF_MIN = 10.87  # amesweb ASME B18.2.2: F min 0.428 in

# Westfield ISO 4762: M3 dk 5.68 max, 5.32 min (min taken as the bearing diameter).
M3_SHCS = Screw(
    "M3 socket head cap screw", "ISO 4762", 3.0, 0.5, 5.68, 5.32, 3.00, 2.5,
    (6, 8, 10, 12, 16, 20, 25, 30, 35, 40, 45, 50), 0.3, "Westfield ISO 4762 table",
)
# Fabory DIN 603 M6 page: dk max 16.55, k max 3.88, neck fn max 4.6, Vn max 6.48.
# Aspen Fasteners DIN 603 M6: dk min 15.45. Stock lengths are the common catalogue ones.
M6_CARRIAGE = Screw(
    "M6 carriage bolt", "DIN 603", 6.0, 1.0, 16.55, 15.45, 3.88, 0.0,
    (16, 20, 25, 30, 35, 40, 45, 50, 60), 0.42,
    "Fabory DIN 603 M6; Aspen Fasteners DIN 603", neck_w=6.48, neck_h=4.6,
)
QUARTER_SHCS = Screw(
    "1/4\"-20 socket head cap screw", "ASME B18.3", 0.25 * INCH, INCH / 20, 0.375 * INCH,
    0.36 * INCH, 0.25 * INCH, 0.1875 * INCH,
    tuple(round(n / 16 * INCH, 3) for n in (4, 5, 6, 8, 10, 12, 16)), 0.25,
    "Westfield ASME B18.3 table",
)

M3_WASHER = Washer("M3 washer", "ISO 7089", 3.2, 7.0, 0.5, "Westfield ISO 7089")
# fasten.it DIN EN ISO 7093-1 table: M6 6.4 / 18 / 1.6 (Keller & Kalmbach sheet agrees).
M6_LARGE_WASHER = Washer("M6 large washer", "ISO 7093-1", 6.4, 18.0, 1.6,
                         "fasten.it ISO 7093-1 table; Keller & Kalmbach")
# 3djake ruthex table, M3 row: D1 4.6, D3 4.0, L 5.7, W min 1.6. D1 is read as the knurl
# and D3 as the hole; the table does not label them, so confirm on the coupon.
M3_INSERT = Insert("M3 heat-set insert", "ruthex RX-M3x5.7", 4.6, 4.0, 5.7, 1.6,
                   "3djake ruthex RX-M3x5.7 table")

M3_CLEAR = 3.4  # ISO 273 medium
M6_CLEAR = 6.6  # ISO 273 medium
QUARTER_CLEAR = 6.6
M3_WASHER_BORE = (7.4, 3.8)  # washer 7.0 + 0.4, head 3.0 + washer 0.5 + 0.3
M6_NECK_HOLE = (6.48 + 0.3, 4.6 + 0.3)  # square, depth: neck max + 0.3
QUARTER_WASHER_SLOT = (18.4, 1.6 + 6.35 + 0.35)  # ISO 7093 washer + 1/4" head + 0.35
LEAD_IN = 0.4
INSERT_SINK = 0.2  # insert top below its face, so it never lifts the part on it
INSERT_ENGAGE = 4.5  # 1.5 d of an M3 in the insert's 5.7 mm thread
SEAM_CORNER_ROOM = 4.55  # a G2 seam corner (setback 8) dips 0.16 mm this far in: under the sink
QUARTER_TIP_FLOOR = 22.5  # 1/4" port tip bore floor under the face; stud length is the user's
QUARTER_DROP = 1.5  # the 1/4" nut drops this far into its pocket and cannot slide back

# Bolted joints. Preload F = T / (K d). estimated: K 0.2 for dry zinc-plated steel.
NUT_FACTOR = 0.2
PRELOAD_N = {
    "m3": 80.0,  # insert pull-out limit at 1.10 MPa; above the 2x foot bolt load
    "joint": 400.0,  # holds the 2x moment on the rosette with margin (rosette rows)
    "camera": 394.0,  # 0.5 Nm on the 1/4" camera screw, on an 18 mm washer
    "port": 236.0,  # 0.3 Nm on a stud in the 1/4" port
}
PETG_UTS = {"along": 38.5, "across": 19.0}  # Ultimaker PETG TDS, XY and Z
# Sustained limits, 0.1 x UTS (DESIGN 5). Bearing is compression and closes the layers,
# so it uses the along-layer value. Insert pull-out shears the layer planes: across, /sqrt 3.
BEARING_LIMIT = 0.1 * PETG_UTS["along"]
INSERT_SHEAR_LIMIT = 0.1 * PETG_UTS["across"] / math.sqrt(3)
ROSETTE_MU = 0.2  # estimated: PETG on PETG

# Sony ZV-E10 help guide: 115.2 x 64.2 x 44.8 mm (W x H x D). Tripod screw thread under 5.5 mm.
ZVE10_BODY = (115.2, 64.2, 44.8)
ZVE10_THREAD_MAX = 5.5
ZVE10_THREAD_MIN = 4.0  # estimated: about three 1/4"-20 threads


def torque_nm(preload: float, screw: Screw) -> float:
    return NUT_FACTOR * screw.d * preload / 1000


def ceil_to(value: float, step: float) -> float:
    return math.ceil(value / step - 1e-9) * step


@dataclass(frozen=True, slots=True)
class Params:
    # Column, measured (PLAN.md table).
    column_w: float = 40.2
    column_d: float = 55.5
    column_r: float = 12.0  # estimated: tracing corners read 9-13 mm, sides slightly bowed

    # Grip liner, TPU 95A.
    tpu_fit: float = 0.25
    rib_h: float = 0.8
    rib_pitch: float = 5.0
    liner_t: float = 2.0

    # Collar shell, PETG.
    collar_h: float = 40.0
    lip_h: float = 1.2
    lip_reach: float = 1.7  # how far the lip covers the liner, radially
    seam_z: tuple[float, float] = (10.0, 30.0)
    seam_back_skin: float = 3.0
    outer_half_d: float = 35.0
    front_setback: float = 14.0
    taper_setback: float = 11.0
    taper_start_y: float = 22.0  # room for the seam corner and the taper corner
    neck_y: float = 44.0
    mf_y: float = 57.0
    deck_z: float = 16.0  # the deck (top tier, full plan) runs from here to the top
    ring_inset: float = 1.0  # the lower ring sits this far inside the deck: a shadow line
    seam_setback: float = 9.5  # seam-face plan corner: radius 0.47 x setback, over the 4 roll
    seam_tip_room: float = 10.0  # drilled tip bore past the back seam face (M3x30 tip: 8.6)

    # Mount face MF-40 and the foot that sits on it.
    mf_size: float = 40.0
    mf_setback: float = 12.0
    mf_m3_spacing: float = 20.0
    foot_t: float = 8.0
    foot_under_head: float = 4.2  # PETG under the washer

    # Arm.
    arm_x0: float = 3.5  # rosette plane of J1, clear of the foot screw key rods
    knuckle_r: float = 14.5  # rosette r_out + 0.5 flat + the 1.0 chamfer
    knuckle_t: float = 10.0
    teeth: int = 24
    rosette_r_in: float = 5.0
    rosette_r_out: float = 13.0
    tooth_trunc: float = 0.15
    flank_fit: float = 0.10
    index_lift: float = 2.2
    link_pitch: float = 70.0
    link_waist: float = 20.0
    j1_rise: float = 18.0  # J1 above the foot top
    head_block_gap: float = 16.0  # head MF block bottom above J3
    knob_d: float = 28.0
    knob_roof: float = 2.0
    knob_tip_room: float = 16.0  # blind bolt bore in the knob, from its base
    hero_angles: tuple[float, float, float] = (75.0, 120.0, 0.0)  # link 1, link 2, head
    folded_angles: tuple[float, float, float] = (90.0, 300.0, 0.0)

    # Load case (estimated: ZV-E10 about 343 g with battery, plus lens and margin).
    payload_kg: float = 1.0

    # Install: the column top above the collar bottom face. The hero pose reaches back over
    # the column, so the collar must sit near the top; assembly.column_rows measures the limit.
    column_top_z: float = 100.0

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        for item in fields(self):
            raw = getattr(self, item.name)
            for number in raw if isinstance(raw, tuple) else (raw,):
                if isinstance(number, bool) or not isinstance(number, (int, float)):
                    raise ValueError(f"{item.name} must be a number")
                if item.name.endswith("_angles"):
                    continue
                if not math.isfinite(number) or number <= 0:
                    raise ValueError(f"{item.name} must be finite and positive, got {number}")
        if self.column_r * 2 >= min(self.column_w, self.column_d):
            raise ValueError("column_r does not fit inside the column")
        if self.liner_t < MATERIALS["TPU95A"]["wall_min"]:
            raise ValueError("liner_t is below the TPU wall minimum")
        if any(not 0 < z < self.collar_h for z in self.seam_z):
            raise ValueError("a seam screw is outside the collar height")
        if self.rosette_r_out >= self.knuckle_r:
            raise ValueError("rosette does not fit on the knuckle")
        if any(angle % 15 for angle in (*self.hero_angles, *self.folded_angles)):
            raise ValueError("joint angles must sit on the 15 degree rosette index")

    # Derived collar sizes. Kept as methods so a changed input moves every part.
    @property
    def bore_offset(self) -> float:
        return self.tpu_fit + self.rib_h + self.liner_t

    @property
    def seam_gap(self) -> float:
        """The seam closes before the ribs crush flat."""
        return 2 * self.tpu_fit + 2 * self.rib_h - 0.1

    @property
    def bore_half_w(self) -> float:
        return self.column_w / 2 + self.bore_offset

    @property
    def bore_half_d(self) -> float:
        return self.column_d / 2 + self.bore_offset

    @property
    def bore_r(self) -> float:
        return self.column_r + self.bore_offset

    @property
    def seam_x(self) -> float:
        """The wider of: the front shank with PETG_WALL, the back insert with its maker wall."""
        shank = PETG_WALL + M3_CLEAR / 2
        insert = M3_INSERT.wall_min + M3_INSERT.outer_d / 2
        return self.bore_half_w + max(shank, insert)

    @property
    def outer_half_w(self) -> float:
        """The wider of two needs, plus the ring inset.

        Washer bore plus the wall, plus 1 mm the soft front corner takes back. Or: the
        insert's maker wall, plus room for the ring's G2 seam corner to dip less than the
        insert sink where that wall meets the seam face.
        """
        washer = self.seam_x + M3_WASHER_BORE[0] / 2 + 1.8 + 1.0
        insert = self.seam_x + M3_INSERT.outer_d / 2 + M3_INSERT.wall_min + SEAM_CORNER_ROOM
        return ceil_to(max(washer, insert) + self.ring_inset, 0.5)

    @property
    def seam_seat_y(self) -> float:
        """Deepest washer seat that keeps PETG_WALL to the bore's front corner.

        The bore corner is a circle about (cx, cy). The nearest point of the washer bore
        in plan is its inner edge at the seat, so the seat sits where that distance is
        bore_r + PETG_WALL.
        """
        cx = self.bore_half_w - self.bore_r
        cy = -(self.bore_half_d - self.bore_r)
        dx = self.seam_x - M3_WASHER_BORE[0] / 2 - cx
        reach = self.bore_r + PETG_WALL
        if dx >= reach:
            return cy
        return cy - math.sqrt(reach * reach - dx * dx)

    @property
    def mf_z(self) -> float:
        return self.collar_h

    @property
    def mf_depth(self) -> float:
        return QUARTER_TIP_FLOOR + PETG_WALL

