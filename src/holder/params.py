"""Design parameters, materials and fastener sizes (PLAN.md "Params").

All lengths in mm. Z up, origin = column axis at the band bottom, +Y = back.
Low-confidence values carry a comment "# estimated: <source>".
"""

import math
from dataclasses import dataclass, fields
from itertools import pairwise


@dataclass(frozen=True)
class Material:
    name: str
    wall_min: float  # thinnest wall that still prints and holds, mm
    fit: float  # clearance added per side for parts that slide together, mm


TPU = "TPU95A"
PETG = "PETG"

# Values from the Tigerbee 7inch accessories, where they already worked.
MATERIALS = {
    TPU: Material(TPU, wall_min=1.2, fit=0.25),
    PETG: Material(PETG, wall_min=1.5, fit=0.30),
}

# Hex nut pockets: corner radius = across-flats / sqrt(3).
HEX_CORNER = 1 / math.sqrt(3)


@dataclass(frozen=True)
class Params:
    # Column of the monitor stand (PLAN.md "Column").
    column_w: float = 40.2  # X, caliper-width-2.jpg (direct caliper reading)
    column_d: float = 55.5  # Y, profile-tracing-depth.jpg minus pencil line; caliper-depth.jpg 56
    column_r: float = 12.0  # estimated: profile-tracing*.jpg corners read 9-13 mm, sides bowed

    # TPU band: two halves, seams left and right.
    band_wall: float = 4.0
    band_h: float = 35.0
    seam_gap: float = 4.0  # per seam; tightening the bolts closes it
    ear_len: float = 10.0  # how far an ear sticks out of the band
    ear_t: float = 5.0  # ear thickness along the seam bolt
    ear_bolt_z: tuple[float, ...] = (9.0, 26.0)  # seam bolt heights above the band bottom
    # Grip ribs: horizontal rings on the band inner face, rib_h proud of it, one every rib_pitch.
    # Trapezoid section: rib_tip wide at the column, 45 degree flanks (print without support).
    rib_h: float = 1.0
    rib_pitch: float = 5.0
    rib_tip: float = 1.0  # estimated: PLAN says "1 mm grip ribs"; contact width not given

    # Boss on the back half (4 x M3 to the block): a boss_w square face (X x Z), boss_depth
    # proud of the band's outer back face, bottom flush with the band bottom (the bed face).
    boss_w: float = 32.0
    # 10, not PLAN's 12 (user decision after Task 4): with 12 an M3x16 ended 0.9 mm short of
    # the far face of the plate nut. With 10 it reaches 1.1 mm past it (block.boss_hardware_checks).
    boss_depth: float = 10.0

    # PETG block: plate, arm, riser.
    block_size: float = 32.0  # arm and riser cross-section; the plate covers the boss face
    arm_length: float = 20.0  # boss face to the riser's front face
    riser_height: float = 40.0  # riser top (the extension point) above the arm top
    fillet_r: float = 3.0  # inner L corner
    plate_nut_wall: float = 1.5  # PETG between the boss face and each plate nut (wall_min)
    plate_hole_extra: float = 4.0  # estimated: blind hole past a plate nut, room for the M3x16 tip
    # Top face (user decision after Task 4, replaces PLAN's M3 at 6 / 1/4" at 14): the 1/4" nut
    # sits right under a thin top wall, so a short 1/4"-20 accessory screw (about 6 mm of
    # thread) reaches it; the M3 nuts sit below the 1/4" pocket, so the pockets never meet.
    top_quarter_nut_depth: float = 2.5  # top wall over the 1/4" nut (PETG wall_min 1.5)
    top_m3_nut_depth: float = 10.0  # 1.7 mm of PETG below the 1/4" pocket (2.5 + 5.8)
    top_m3_hole_extra: float = 6.0  # estimated: blind hole past a top M3 nut, for longer bolts
    top_quarter_hole_extra: float = 10.0  # estimated: past the 1/4" nut, for longer screws
    quarter_screw_thread: float = 6.0  # estimated: short 1/4"-20 accessory screw (user)
    quarter_pitch: float = 25.4 / 20  # 1/4"-20 UNC: 20 threads per inch
    quarter_engage_threads: float = 2.0  # estimated: least engagement of that short screw

    # Coupon: one band slice (one seam bolt per seam, at mid-height) plus a PETG strip (Task 2).
    coupon_h: float = 12.0

    # Shared M3 pattern on the boss, the plate and the top face.
    m3_spacing: float = 20.0

    # Fasteners (pocket and hole sizes already include the print fit).
    m3_through: float = 3.4
    # Boss head recess: takes the head on a DIN 125 washer (7.0) so the head does not bear
    # on the TPU with its own small annulus. Recess depth = m3_head_h + m3_washer_t + extra.
    m3_head_d: float = 7.6  # estimated: washer 7.0 + 0.3 per side (TPU fit 0.25 + 0.05)
    m3_head_h: float = 2.0  # head height the recess is sized for (>= ISO 7380 1.65)
    m3_head_extra: float = 1.0  # extra depth so the heads sit below the band face
    m3_hex_af: float = 5.7  # nut pocket across flats
    m3_hex_h: float = 2.6  # nut pocket depth
    quarter_hex_af: float = 11.4  # estimated: 7/16" nut 11.1 + fit, new value; confirm with coupon
    quarter_hex_h: float = 5.8  # estimated: nut 5.6 + fit, new value; confirm with coupon
    quarter_through: float = 6.6  # 1/4" = 6.35 plus clearance

    # Hardware (user decision after Task 4): 8 x M3x16 ISO 7380 button head, 4 at the seams
    # (+ washer + nut), 4 boss to block (+ washer + nut: the washer spreads the head load on
    # the TPU, fix pass after Task 4). Standard part sizes.
    m3_bolt_head_d: float = 5.7  # ISO 7380 M3 button head
    m3_bolt_head_h: float = 1.65
    seam_bolt_len: float = 16.0  # M3x16, length under the head
    m3_washer_d: float = 7.0  # DIN 125-A M3 washer
    m3_washer_t: float = 0.5
    m3_nut_h: float = 2.4  # ISO 4032 M3 nut
    m3_nut_af: float = 5.5  # ISO 4032 M3 nut, across flats
    quarter_nut_af: float = 11.1  # 1/4"-20 hex nut, 7/16" across flats
    quarter_nut_h: float = 5.6  # 1/4"-20 hex nut, 7/32" thick
    boss_bolt_len: float = 16.0  # M3x16, length under the head

    # PETG block printed on its -X side: the nut slots at x < 0 open on the bed face, where
    # the first layers squish into the slot mouth (elephant's foot). Those mouths get a
    # 45 degree chamfer on every side except the nut's seat side.
    elephant_foot: float = 0.3  # estimated: PETG first-layer squish, typically 0.1-0.3 mm
    bed_slot_flare: float = 0.6  # chamfer size: mouth >= nut + 2 x elephant_foot

    @property
    def m3_head_recess_depth(self) -> float:
        return self.m3_head_h + self.m3_washer_t + self.m3_head_extra

    def walls(self) -> list[tuple[str, float, str]]:
        """Walls that follow from the parameters alone: (name, thickness, material).

        Hex pockets use the corner radius so the wall holds in any nut orientation.
        Geometry-dependent walls are measured on the real solids by each part's checks().
        """
        m3_corner = self.m3_hex_af * HEX_CORNER
        quarter_corner = self.quarter_hex_af * HEX_CORNER
        pattern_margin = (self.boss_w - self.m3_spacing) / 2
        block_margin = (self.block_size - self.m3_spacing) / 2
        walls = [
            ("band_wall", self.band_wall, TPU),
            ("ear behind the M3 nut pocket", self.ear_t - self.m3_hex_h, TPU),
            ("ear tip to seam hole", self.ear_len / 2 - self.m3_through / 2, TPU),
            ("ear tip to seam nut pocket", self.ear_len / 2 - m3_corner, TPU),
            ("boss edge to M3 head recess", pattern_margin - self.m3_head_d / 2, TPU),
            ("block edge to M3 nut pocket", block_margin - m3_corner, PETG),
            (
                "block edge to bed-side M3 slot chamfer",
                block_margin - self.m3_hex_af / 2 - self.bed_slot_flare,
                PETG,
            ),
            ("block edge to 1/4 nut pocket", self.block_size / 2 - quarter_corner, PETG),
            ("boss face to plate nut pocket", self.plate_nut_wall, PETG),
            ("top face to 1/4 nut pocket", self.top_quarter_nut_depth, PETG),
            ("top face to M3 nut pocket", self.top_m3_nut_depth, PETG),
            # Conservative: as if the pockets sat above each other (they do not, quite).
            (
                "1/4 nut pocket to the M3 nut pockets below it",
                self.top_m3_nut_depth - self.top_quarter_nut_depth - self.quarter_hex_h,
                PETG,
            ),
            # The coupon has one seam bolt per seam at mid-height.
            ("coupon edge to seam nut", self.coupon_h / 2 - m3_corner, TPU),
        ]
        for z in self.ear_bolt_z:
            walls.append((f"band bottom to seam nut at z={z:g}", z - m3_corner, TPU))
            walls.append((f"band top to seam nut at z={z:g}", self.band_h - z - m3_corner, TPU))
        for z_low, z_high in pairwise(sorted(self.ear_bolt_z)):
            gap = z_high - z_low - 2 * m3_corner
            walls.append((f"seam nuts z={z_low:g} to z={z_high:g}", gap, TPU))
        return walls

    def validate(self) -> "Params":
        """Raise ValueError on a non-finite or non-positive value, or a wall below its minimum
        (TypeError on a value that is not a number). Returns self, so it chains."""
        for f in fields(self):
            value = getattr(self, f.name)
            values = value if isinstance(value, tuple) else (value,)
            if not values:
                raise ValueError(f"{f.name}: empty")
            for v in values:
                if isinstance(v, bool) or not isinstance(v, int | float):
                    raise TypeError(f"{f.name}: not a number: {v!r}")
                if not math.isfinite(v):
                    raise ValueError(f"{f.name}: not finite: {v!r}")
                if v <= 0:
                    raise ValueError(f"{f.name}: must be > 0, got {v!r}")
        for name, thickness, material in self.walls():
            wall_min = MATERIALS[material].wall_min
            if thickness < wall_min:
                raise ValueError(f"{name}: {thickness:.3f} mm < {material} wall_min {wall_min} mm")
        if self.rib_tip + 2 * self.rib_h > self.rib_pitch:
            raise ValueError("rib_pitch: neighbouring grip ribs overlap at the band face")
        if (
            self.m3_bolt_head_d >= self.m3_head_d
            or self.m3_washer_d >= self.m3_head_d
            or self.m3_bolt_head_h > self.m3_head_h
        ):
            raise ValueError(
                "m3_head_d / m3_bolt_head_h / m3_bolt_head_d / m3_washer_d: "
                "the head and its washer do not fit the recess"
            )
        if self.boss_w > self.band_h:
            raise ValueError("boss_w: the boss is taller than the band")
        return self


DEFAULT = Params().validate()
