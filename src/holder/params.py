"""Column, band, and fastener dimensions. Low-confidence values are marked estimated."""

import math
from dataclasses import dataclass, fields

MATERIALS = {
    "TPU95A": {"wall_min": 1.2, "fit": 0.25},
    "PETG": {"wall_min": 1.5, "fit": 0.30},
}

# ISO metric stock under the head. The stack picks the shortest size that covers it.
M3_STOCK_MM = (6, 8, 10, 12, 16, 20, 25, 30, 35, 40, 45, 50)


def m3_stock(length_mm: float) -> int:
    """Shortest stock M3 whose length under the head covers `length_mm`."""
    for size in M3_STOCK_MM:
        if size >= length_mm:
            return size
    raise ValueError(f"no stock M3 covers {length_mm:.2f} mm")


@dataclass(frozen=True, slots=True)
class Params:
    column_w: float = 40.2
    column_d: float = 55.5
    column_r: float = 12.0  # estimated: tracing corners read 9–13 mm, sides slightly bowed
    band_wall: float = 4.0
    band_h: float = 35.0
    seam_gap: float = 4.0
    ear_len: float = 10.0
    ear_t: float = 5.0
    ear_bolt_z: tuple[float, float] = (9.0, 26.0)
    boss_w: float = 32.0
    boss_depth: float = 12.0
    block_size: float = 32.0
    arm_length: float = 20.0
    riser_height: float = 40.0
    fillet_r: float = 3.0
    m3_spacing: float = 20.0
    m3_through: float = 3.4
    m3_head_d: float = 6.6
    m3_head_h: float = 3.0  # 2 mm head plus 1 mm so the head sits below the face
    m3_hex_af: float = 5.7
    m3_hex_h: float = 2.6
    # estimated: 7/16" nut 11.1 AF plus PETG fit; confirm on the coupon
    quarter_hex_af: float = 11.4
    quarter_hex_h: float = 5.8  # estimated: 7/16" nut 5.6 thick plus PETG fit
    quarter_through: float = 6.6

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        for item in fields(self):
            raw = getattr(self, item.name)
            numbers = raw if isinstance(raw, tuple) else (raw,)
            for number in numbers:
                if isinstance(number, bool) or not isinstance(number, (int, float)):
                    raise ValueError(f"{item.name} must be a number")
                if not math.isfinite(number) or number <= 0:
                    raise ValueError(f"{item.name} must be finite and positive, got {number}")
        if self.band_wall < MATERIALS["TPU95A"]["wall_min"]:
            raise ValueError("band_wall is below the TPU wall minimum")
        if self.column_r * 2 >= min(self.column_w, self.column_d):
            raise ValueError("column_r does not fit inside the column")
        for z in self.ear_bolt_z:
            if not 0 < z < self.band_h:
                raise ValueError("ear bolt is outside the band height")

    def seam_stack_mm(self) -> float:
        """Under the head: both ears, the open seam, and the nut."""
        return 2 * self.ear_t + self.seam_gap + self.m3_hex_h

    def boss_stack_mm(self) -> float:
        """Under the head: wall past the recess, the boss, and the nut.

        The plate is as deep as the boss. The nut is centered in that plate.
        """
        plate_mm = self.boss_depth
        nut_far = (plate_mm + self.m3_hex_h) / 2
        return (self.band_wall - self.m3_head_h) + self.boss_depth + nut_far

    def seam_bolt_mm(self) -> int:
        return m3_stock(self.seam_stack_mm())

    def boss_bolt_mm(self) -> int:
        return m3_stock(self.boss_stack_mm())
