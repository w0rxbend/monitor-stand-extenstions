"""Every dimension the parts are built from. Bought parts carry their published source.

A value with no published source is marked `estimated:` and stays an estimate until
it is measured on a printed coupon, with calipers, or on a supplier sheet.
"""

import math
from dataclasses import dataclass, fields

MATERIALS = {
    "TPU95A": {"wall_min": 1.2, "fit": 0.25},
    "PETG": {"wall_min": 1.5, "fit": 0.30},
}
PETG_WALL = MATERIALS["PETG"]["wall_min"]
INCH = 25.4


@dataclass(frozen=True, slots=True)
class ShoulderScrew:
    """ISO 7379 hexagon socket shoulder screw. `stock` lists shoulder lengths l1.

    The shoulder is the stop: tightened, its end lands on the brass insert, so the bolt
    loop closes through steel and brass, never through PETG.
    """

    name: str
    standard: str
    d: float  # thread
    pitch: float
    shoulder_d: float
    head_d: float
    head_k: float
    key: float
    thread_b: float  # thread length past the shoulder end (b max)
    stock: tuple[float, ...]
    l1_plus: float  # l1 tolerance is 0 / +l1_plus
    source: str

    def stock_at_least(self, length: float) -> float:
        for size in self.stock:
            if size >= length - 1e-9:
                return size
        raise ValueError(f"no stock {self.name} covers {length:.2f} mm")


@dataclass(frozen=True, slots=True)
class DiscSpring:
    """DIN 2093 disc spring. `table` is (s / h0, force N) as published, s the deflection."""

    name: str
    standard: str
    outer: float
    inner: float
    t: float
    h0: float
    table: tuple[tuple[float, float], ...]
    source: str

    @property
    def free_height(self) -> float:
        return self.t + self.h0

    def force(self, deflection: float) -> float:
        """Force at `deflection` (mm), linear between the published points."""
        share = deflection / self.h0
        if share < 0 or share > 1.0 + 1e-9:
            raise ValueError(f"{self.name}: deflection {deflection:.3f} outside 0..h0")
        points = ((0.0, 0.0), *self.table)
        for (s0, f0), (s1, f1) in zip(points, points[1:]):
            if share <= s1 + 1e-12:
                return f0 + (f1 - f0) * (share - s0) / (s1 - s0)
        return points[-1][1]


@dataclass(frozen=True, slots=True)
class Screw:
    """Socket head cap screw (ASME B18.3 here: the camera screw)."""

    name: str
    standard: str
    d: float
    pitch: float
    head_d: float
    bearing_d: float
    head_k: float
    key: float
    stock: tuple[float, ...]
    length_tol: float  # estimated: ISO 4759-1 js15 band, not checked per length
    source: str

    def stock_at_least(self, length: float) -> float:
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
    minor_d: float  # internal thread minor diameter (ISO 965-1 D1); the screw overlaps it
    source: str


# fasten.it ISO 7379 table: M6 ds 8, b max 11.25, dk max 13, k max 5.5, s 4;
# M5 ds 6.5, b max 9.75, dk max 10, k max 4.5, s 3. Lengths: fasten.it nominal list.
# l1 tolerance 0/+0.25: Elesa and Misumi ISO 7379 sheets.
SHOULDER_M6 = ShoulderScrew(
    "M6 shoulder screw, 8 mm shoulder", "ISO 7379", 6.0, 1.0, 8.0, 13.0, 5.5, 4.0, 11.25,
    (10, 12, 16, 20, 25, 30, 35, 40, 50, 60), 0.25, "fasten.it ISO 7379; Elesa ISO 7379",
)
SHOULDER_M5 = ShoulderScrew(
    "M5 shoulder screw, 6.5 mm shoulder", "ISO 7379", 5.0, 0.8, 6.5, 10.0, 4.5, 3.0, 9.75,
    (10, 12, 16, 20, 25, 30, 35, 40), 0.25, "fasten.it ISO 7379; Elesa ISO 7379",
)

# fasten.it DIN 2093 table (DIN 2093 is replaced by DIN EN 16983 with the same sizes).
# Forces at s = 0.25, 0.5, 0.75 and 1.0 h0.
SPRING_B16 = DiscSpring(
    "disc spring 16 x 8.2 x 0.6", "DIN 2093 B", 16.0, 8.2, 0.6, 0.45,
    ((0.25, 172.0), (0.5, 304.0), (0.75, 410.0), (1.0, 503.0)), "fasten.it DIN 2093 table",
)
SPRING_C14 = DiscSpring(
    "disc spring 14 x 7.2 x 0.35", "DIN 2093 C", 14.0, 7.2, 0.35, 0.45,
    ((0.25, 68.0), (0.5, 106.0), (0.75, 123.0), (1.0, 131.0)), "fasten.it DIN 2093 table",
)

QUARTER_SHCS = Screw(
    "1/4\"-20 socket head cap screw", "ASME B18.3", 0.25 * INCH, INCH / 20, 0.375 * INCH,
    0.36 * INCH, 0.25 * INCH, 0.1875 * INCH,
    tuple(round(n / 16 * INCH, 3) for n in (4, 5, 6, 8, 10, 12, 16)), 0.25,
    "Westfield ASME B18.3 table",
)

# Westfield ISO 7089: M8 8.4 / 16 / 1.6.
WASHER_M8 = Washer("M8 washer", "ISO 7089", 8.4, 16.0, 1.6, "Westfield ISO 7089")
# fasten.it DIN EN ISO 7093-1 table: M6 6.4 / 18 / 1.6 (Keller & Kalmbach sheet agrees).
WASHER_M6_LARGE = Washer("M6 large washer", "ISO 7093-1", 6.4, 18.0, 1.6,
                         "fasten.it ISO 7093-1 table; Keller & Kalmbach")

# 3djake ruthex tables: M5 D1 7.1, D3 6.4, L 9.5, W min 2.6; M6 D1 8.7, D3 8.0, L 12.7,
# W min 3.3. D1 is read as the knurl and D3 as the hole (RS PRO M5 sheet: hole 6.4 for a
# 7.1 body). Minor diameters: ISO 965-1 D1.
INSERT_M5 = Insert("M5 heat-set insert", "ruthex RX-M5x9.5", 7.1, 6.4, 9.5, 2.6, 4.134,
                   "3djake ruthex RX-M5x9.5 table")
INSERT_M6 = Insert("M6 heat-set insert", "ruthex RX-M6x12.7", 8.7, 8.0, 12.7, 3.3, 4.917,
                   "3djake ruthex RX-M6x12.7 table")

SHOULDER_CLEAR = 0.4  # shoulder bore over the f9 shoulder: ISO 273 fine series spacing
QUARTER_CLEAR = 6.6
LEAD_IN = 0.4
INSERT_SINK = 0.2  # insert top below its face: the shoulder lands on brass, not on PETG
INSERT_TIP_ROOM = 0.8  # hole past the insert, for a thread longer than the insert
QUARTER_WASHER_SEAT = 18.4  # ISO 7093 washer + 0.4
PLATE_UNDER_HEAD = 6.0  # PETG under the camera washer: puts 1/2" inside the socket
PRINT_TOL = 0.15  # estimated: printed thickness of a clamped stack, +-
RING_T = 1.0  # J3 TPU friction ring
RING_TOL = 0.1  # estimated: printed TPU ring thickness, +-

PETG_UTS = {"along": 38.5, "across": 19.0}  # Ultimaker PETG TDS, XY and Z
# Sustained limits, 0.1 x UTS (DESIGN 5). Bearing is compression and closes the layers,
# so it uses the along-layer value. Insert pull-out shears the layer planes: across, /sqrt 3.
BEARING_LIMIT = 0.1 * PETG_UTS["along"]
INSERT_SHEAR_LIMIT = 0.1 * PETG_UTS["across"] / math.sqrt(3)
MU_PETG = 0.2  # estimated: PETG on PETG (rosette flanks, the pivot face)
MU_TPU = 0.5  # estimated: TPU 95A on PETG (the J3 ring). Confirm on the coupon.
NUT_FACTOR = 0.2  # estimated: K for dry zinc-plated steel
CAMERA_HAND_NM = 1.0  # estimated: a firm hand on a 3/16" key, short leg

# Sony ZV-E10 help guide: 115.2 x 64.2 x 44.8 mm (W x H x D). Tripod screw under 5.5 mm.
ZVE10_BODY = (115.2, 64.2, 44.8)
ZVE10_THREAD_MAX = 5.5
ZVE10_THREAD_MIN = 4.0  # estimated: about three 1/4"-20 threads
ZVE10_MASS = 0.343  # Sony spec sheet, with battery and card
# estimated: the tripod socket sits under the lens axis, 13 mm from the body centre toward
# the lens side, 2 mm behind mid depth. Not on Sony's pages: confirm with calipers.
ZVE10_SOCKET = (13.0, 2.0)
# estimated: battery and card door on the grip end of the bottom, 36 x 24 mm, 4 mm in from
# the end. Sony's help guide shows it there but gives no size. Confirm with calipers.
ZVE10_DOOR = (36.0, 24.0, 4.0)
ZVE10_DOOR_SWING = 30.0  # the door and a dummy-battery plug need this much room below


def preload_n(torque: float, d: float) -> float:
    return torque * 1000 / (NUT_FACTOR * d)


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

    # Collar, PETG: one turned-looking body split at the seam.
    collar_h: float = 40.0
    lip_h: float = 1.2
    lip_reach: float = 1.7  # how far the lip covers the liner, radially
    seam_z: tuple[float, float] = (10.0, 28.0)  # two 16.4 counterbores, 1.6 of PETG between
    outer_half_d: float = 36.0  # front face
    front_setback: float = 16.0
    shoulder_setback: float = 12.0
    collar_bulge: float = 2.0  # the barrel: an exact parabola, visible in every view
    pivot_y: float = 52.0  # pan pivot on the collar's back: the pedestal foot clears the bore
    pivot_r: float = 22.0  # the back reaches this far behind the pivot
    seam_crush: float = 0.6  # rib crush the seam springs must follow (0.3 a side: 37 % of a rib)
    column_tol: float = 0.3  # estimated: column depth band the seam springs must cover

    # Arm.
    knuckle_r: float = 15.5
    link_t: float = 14.0  # the far knuckle holds a 12.9 mm insert hole and a 1.1 floor
    teeth: int = 24
    rosette_r_in: float = 7.7  # insert knurl 4.35 + maker wall 3.3, then the teeth
    rosette_r_out: float = 13.0
    tooth_trunc: float = 0.15
    flank_fit: float = 0.10
    index_lift: float = 2.2
    link_pitch: float = 70.0
    link_waist: float = 20.0
    ball_r: float = 16.6  # turned ball knuckle on arm_base
    ball_flat_x: float = 6.0  # the milled J1 flat, off the pan axis
    head_t: float = 31.0  # arm_head width along Y
    platform_reach: float = 34.0  # camera platform past the J3 friction face (toward -X)
    camera_x: float = 19.0  # camera screw past the J3 friction face: clear of arm_tip
    hero_angles: tuple[float, float, float] = (75.0, 120.0, 0.0)  # link, tip, head
    folded_angles: tuple[float, float, float] = (90.0, 300.0, 0.0)
    hero_pan: float = 0.0

    # Load case (estimated: ZV-E10 343 g with battery, plus a lens and margin).
    payload_kg: float = 1.0

    # Install: the column top above the collar bottom face (assembly.column_rows).
    column_top_z: float = 100.0

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        for item in fields(self):
            raw = getattr(self, item.name)
            for number in raw if isinstance(raw, tuple) else (raw,):
                if isinstance(number, bool) or not isinstance(number, (int, float)):
                    raise ValueError(f"{item.name} must be a number")
                if item.name.endswith("_angles") or item.name == "hero_pan":
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
        if self.rosette_r_in < INSERT_M6.outer_d / 2 + INSERT_M6.wall_min:
            raise ValueError("rosette teeth cut into the insert's maker wall")
        if any(angle % 15 for angle in (*self.hero_angles[:2], *self.folded_angles[:2])):
            raise ValueError("rosette joint angles must sit on the 15 degree index")

    # Derived collar sizes. Kept as properties so a changed input moves every part.
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
    def seam_x(self) -> float:
        """The wider of: the shoulder bore with PETG_WALL, the insert with its maker wall."""
        shank = PETG_WALL + (SHOULDER_M5.shoulder_d + SHOULDER_CLEAR) / 2
        insert = INSERT_M5.wall_min + INSERT_M5.outer_d / 2
        return self.bore_half_w + max(shank, insert)

    @property
    def seam_bore_d(self) -> float:
        """Counterbore for the seam washer, springs and head."""
        return WASHER_M8.outer + 0.4

    @property
    def outer_half_w(self) -> float:
        return ceil_to(self.seam_x + self.seam_bore_d / 2 + PETG_WALL + 0.5, 0.5)
