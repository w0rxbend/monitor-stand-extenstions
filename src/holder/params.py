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
    boss_depth: float = 12.0

    # PETG block: plate, arm, riser.
    block_size: float = 32.0
    arm_length: float = 20.0
    riser_height: float = 40.0
    fillet_r: float = 3.0
    top_m3_nut_depth: float = 6.0  # top-face M3 nuts sit this deep (Task 4)
    top_quarter_nut_depth: float = 14.0  # top-face 1/4"-20 nut sits this deep (Task 4)

    # Coupon: one band slice (one seam bolt per seam, at mid-height) plus a PETG strip (Task 2).
    coupon_h: float = 12.0

    # Shared M3 pattern on the boss, the plate and the top face.
    m3_spacing: float = 20.0

    # Fasteners (pocket and hole sizes already include the print fit).
    m3_through: float = 3.4
    m3_head_d: float = 6.6  # head recess diameter
    m3_head_h: float = 2.0  # head recess depth = the head height it is sized for
    m3_head_extra: float = 1.0  # extra depth so the heads sit below the band face
    m3_hex_af: float = 5.7  # nut pocket across flats
    m3_hex_h: float = 2.6  # nut pocket depth
    quarter_hex_af: float = 11.4  # estimated: 7/16" nut 11.1 + fit, new value; confirm with coupon
    quarter_hex_h: float = 5.8  # estimated: nut 5.6 + fit, new value; confirm with coupon
    quarter_through: float = 6.6  # 1/4" = 6.35 plus clearance

    # Seam hardware (PLAN.md "Hardware": 4 x M3x20 + washer + nut), standard part sizes.
    seam_bolt_len: float = 20.0  # M3x20, length under the head
    m3_washer_d: float = 7.0  # DIN 125-A M3 washer
    m3_washer_t: float = 0.5
    m3_nut_h: float = 2.4  # ISO 4032 M3 nut

    @property
    def m3_head_recess_depth(self) -> float:
        return self.m3_head_h + self.m3_head_extra

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
            ("boss edge to M3 head recess", pattern_margin - self.m3_head_d / 2, TPU),
            ("block edge to M3 nut pocket", block_margin - m3_corner, PETG),
            ("block edge to 1/4 nut pocket", self.block_size / 2 - quarter_corner, PETG),
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
        if self.boss_w > self.band_h:
            raise ValueError("boss_w: the boss is taller than the band")
        return self


DEFAULT = Params().validate()
