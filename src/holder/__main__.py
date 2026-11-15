"""Build every registered part, run its checks, export to out/.

    python -m holder              all parts
    python -m holder NAME ...     only these parts
    python -m holder --no-png     skip the previews (faster)

Writes out/<name>.step (canonical), out/<name>.stl, out/<name>.3mf, out/<name>.png and
out/assembly.png (out/assembly-<name>.png for other assemblies, e.g. the coupon).
Copies the STL and 3MF of every part that passes into exports/ (the files in git).
Exits non-zero if any check fails. A part that fails its checks gets a PNG for
debugging but no STEP/STL/3MF, so a broken part is never left ready to print.
"""

import argparse
import sys
import time
from pathlib import Path

from build123d import Compound, Part

from holder import registry
from holder.common import Check, PartSpec, clash_checks
from holder.export import export_part, to_print
from holder.params import DEFAULT, Params
from holder.preview import View, render_sheet, standard_sheet

OUT = Path("out")
EXPORTS = Path("exports")
MESH_SUFFIXES = (".stl", ".3mf")


def part_sheet(spec: PartSpec, part: Part, checks: list[Check], out: Path) -> Path:
    printed = View(
        "as printed (iso, bed at Z=0)", to_print(part, spec.print_normal), (1, -1, 1), (0, 0, 1)
    )
    size = part.bounding_box(optimal=True).size  # display only
    passed = sum(c.ok for c in checks)
    subtitle = (
        f"assembly coords, {size.X:.1f} x {size.Y:.1f} x {size.Z:.1f} mm, "
        f"{part.volume / 1000:.1f} cm3 | print: bed normal {spec.print_normal} -> -Z | "
        f"checks {passed}/{len(checks)} ok"
    )
    return render_sheet(
        standard_sheet(part, printed),
        out / f"{spec.name}.png",
        f"{spec.name}  ({spec.material})",
        subtitle,
    )


def assembly_sheet(group: str, parts: dict[str, Part], refs: dict[str, Part], out: Path) -> Path:
    name = "assembly" if group == "holder" else f"assembly-{group}"
    title = f"assembly '{group}': {', '.join(parts)}"
    subtitle = "all parts in assembly coordinates"
    if refs:
        subtitle += f"; reference solids (not printed): {', '.join(refs)}"
    return render_sheet(
        standard_sheet(Compound([*parts.values(), *refs.values()])),
        out / f"{name}.png",
        title,
        subtitle,
    )


def _publish_meshes(name: str, out: Path) -> None:
    EXPORTS.mkdir(parents=True, exist_ok=True)
    for suffix in MESH_SUFFIXES:
        (EXPORTS / f"{name}{suffix}").write_bytes((out / f"{name}{suffix}").read_bytes())


def _drop_print_files(name: str, out: Path) -> None:
    for directory in (out, EXPORTS):
        for suffix in (".step", *MESH_SUFFIXES):
            (directory / f"{name}{suffix}").unlink(missing_ok=True)


def print_table(rows: list[tuple[str, Check]]) -> None:
    w_part = max(len(r[0]) for r in rows)
    w_name = max(len(r[1].name) for r in rows)
    print(f"{'part':<{w_part}}  {'check':<{w_name}}  {'':4}  measured")
    for part, c in rows:
        print(f"{part:<{w_part}}  {c.name:<{w_name}}  {'ok' if c.ok else 'FAIL':4}  {c.measured}")


def run(specs: list[PartSpec], p: Params, out: Path, png: bool) -> list[tuple[str, Check]]:
    rows: list[tuple[str, Check]] = []
    parts: dict[str, Part] = {}
    for spec in specs:
        t0 = time.perf_counter()
        part = spec.build(p)  # a build error is a bug: let it raise with its traceback
        parts[spec.name] = part
        checks = list(spec.checks(part, p))
        if all(c.ok for c in checks):
            checks += export_part(spec, part, out)
            _publish_meshes(spec.name, out)
        else:
            _drop_print_files(spec.name, out)
            print(f"{spec.name}: checks failed, STEP/STL/3MF not written")
        rows += [(spec.name, c) for c in checks]
        if png:
            part_sheet(spec, part, checks, out)
        print(f"{spec.name}: {time.perf_counter() - t0:.1f} s")

    for group, members in registry.assemblies(specs).items():
        printed = {s.name: parts[s.name] for s in members}
        refs = registry.references(group, p)  # e.g. the column: checked and drawn, not exported
        group_parts = {**printed, **refs}
        contacts = [pair for pair in registry.CONTACTS if set(pair) <= group_parts.keys()]
        rows += [(f"assembly {group}", c) for c in clash_checks(group_parts, contacts)]
        group_checks = registry.ASSEMBLY_CHECKS.get(group)
        if group_checks and len(printed) == len(registry.assemblies()[group]):
            rows += [(f"assembly {group}", c) for c in group_checks(group_parts, p)]
        if png and printed:
            assembly_sheet(group, printed, refs, out)
    return rows


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m holder", description=__doc__.split("\n")[0])
    parser.add_argument("parts", nargs="*", help="part names (default: all)")
    parser.add_argument("--out", type=Path, default=OUT, help="output directory (default: out)")
    parser.add_argument("--no-png", action="store_true", help="skip PNG previews")
    args = parser.parse_args(argv)

    specs = [registry.get(n) for n in args.parts] if args.parts else registry.PARTS
    args.out.mkdir(parents=True, exist_ok=True)
    rows = run(specs, DEFAULT, args.out, png=not args.no_png)
    print()
    print_table(rows)
    failed = [r for r in rows if not r[1].ok]
    print(f"\n{len(rows) - len(failed)}/{len(rows)} checks ok; output in {args.out}/")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
