"""Column, band, and fastener dimensions. Low-confidence values are marked estimated."""

import math
from dataclasses import dataclass, fields

MATERIALS = {
    "TPU95A": {"wall_min": 1.2, "fit": 0.25},
    "PETG": {"wall_min": 1.5, "fit": 0.30},
}


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
