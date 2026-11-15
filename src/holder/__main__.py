"""Build, check, and export. ``uv run python -m holder`` writes ``out/``."""

from pathlib import Path

from build123d import Box, Part

import holder.band as band
import holder.block as block
import holder.coupon as coupon
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
    passed = _report(coupon.checks_front(front, spec))
    passed = _report(coupon.checks_back(back, spec)) and passed
    passed = _report(coupon.checks_strip(strip, spec)) and passed
    passed = _report(coupon.checks_pair(front, back, spec)) and passed
    passed = _report(band.checks_front(band_front, spec)) and passed
    passed = _report(band.checks_back(band_back, spec)) and passed
    passed = _report(band.checks_pair(band_front, band_back, spec)) and passed
    passed = _report(block.checks(brace, spec)) and passed
    passed = _report(block.checks_vs_band(brace, band_front, band_back, spec)) and passed
    if not passed:
        raise SystemExit(1)
    out = Path("out")
    for part in (front, back, strip, band_front, band_back, brace):
        export_part(part, out / part.label)
        preview_part(part, out / f"{part.label}.png")


if __name__ == "__main__":
    main()
