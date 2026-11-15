"""Build, check, and export. ``uv run python -m holder`` writes ``out/``."""

from pathlib import Path

from build123d import Box, Part

from holder.coupon import (
    build_back,
    build_front,
    build_strip,
    checks_back,
    checks_front,
    checks_pair,
    checks_strip,
)
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
    front, back, strip = build_front(spec), build_back(spec), build_strip(spec)
    passed = _report(checks_front(front, spec))
    passed = _report(checks_back(back, spec)) and passed
    passed = _report(checks_strip(strip, spec)) and passed
    passed = _report(checks_pair(front, back, spec)) and passed
    if not passed:
        raise SystemExit(1)
    out = Path("out")
    for part in (front, back, strip):
        export_part(part, out / part.label)
        preview_part(part, out / f"{part.label}.png")


if __name__ == "__main__":
    main()
