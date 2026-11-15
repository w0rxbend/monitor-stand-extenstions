"""Build, check, and export. ``uv run python -m holder`` writes ``exports/`` and ``out/``."""

from pathlib import Path

from build123d import Box, Part

import holder.accessories as accessories
import holder.band as band
import holder.block as block
import holder.coupon as coupon
from holder.common import print_orientation
from holder.export import export_part
from holder.params import Params
from holder.preview import preview_part


def placeholder_box() -> Part:
    box = Box(10, 20, 30)
    box.label = "placeholder"
    return box


def _report(results: list[tuple[str, bool, str]]) -> bool:
    passed = True
    for name, ok, detail in results:
        print(f"{'PASS' if ok else 'FAIL'} {name} - {detail}")
        passed = passed and ok
    return passed


def main() -> None:
    spec = Params()
    front = coupon.build_front(spec)
    back = coupon.build_back(spec)
    strip = coupon.build_strip(spec)
    band_front, band_back = band.build_halves(spec)
    brace = block.build(spec)
    extras = [builder(spec) for builder in accessories.BUILDERS]
    passed = _report(coupon.checks_front(front, spec))
    passed = _report(coupon.checks_back(back, spec)) and passed
    passed = _report(coupon.checks_strip(strip, spec)) and passed
    passed = _report(coupon.checks_pair(front, back, spec)) and passed
    passed = _report(band.checks_front(band_front, spec)) and passed
    passed = _report(band.checks_back(band_back, spec)) and passed
    passed = _report(band.checks_pair(band_front, band_back, spec)) and passed
    passed = _report(block.checks(brace, spec)) and passed
    passed = _report(block.checks_vs_band(brace, band_front, band_back, spec)) and passed
    for extra in extras:
        passed = _report(accessories.checks(extra, spec)) and passed
    if not passed:
        raise SystemExit(1)
    exports = Path("exports")
    previews = Path("out")
    orientations = {**coupon.PRINT, **band.PRINT, **block.PRINT, **accessories.PRINT}
    for part in (front, back, strip, band_front, band_back, brace, *extras):
        printed = print_orientation(part, orientations[part.label])
        export_part(printed, exports / part.label)
        preview_part(printed, previews / f"{part.label}.png")


if __name__ == "__main__":
    main()
