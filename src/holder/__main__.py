"""Build, check, and export. ``uv run python -m holder`` writes ``exports/`` and ``out/``.

Every check runs before anything is written. One FAIL exits 1 and leaves exports/ alone.
``--bom`` prints the bought-parts table that README.md carries.
"""

import hashlib
import json
import shutil
import sys
import tempfile
import zipfile
from pathlib import Path
from xml.etree import ElementTree

from build123d import Part, import_step

from holder.assembly import assembly_rows, bom_text
from holder.common import Check, print_orientation
from holder.export import export_part
from holder.params import Params
from holder.preview import preview_part
from holder.registry import CHECKS, PRINT, all_parts

ROOT = Path(__file__).resolve().parents[2]
EXPORTS = ROOT / "exports"
PREVIEWS = ROOT / "out"
SUFFIXES = (".step", ".stl", ".3mf")
STEP_DRIFT = 0.005
MESH_DRIFT = 0.01
CORE_3MF = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"


def source_hash() -> str:
    digest = hashlib.sha256()
    for path in sorted((ROOT / "src").rglob("*.py")):
        digest.update(path.relative_to(ROOT).as_posix().encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def _report(section: str, rows: list[Check], log: list[dict]) -> bool:
    for row in rows:
        print(f"{'PASS' if row.ok else 'FAIL'} [{section}] {row.name} - {row.measured}")
        log.append({"section": section, "name": row.name, "ok": row.ok,
                    "measured": row.measured})
    return all(row.ok for row in rows)


def mesh_3mf_volume(path: Path) -> float:
    """Signed volume of every triangle mesh in a 3MF, read from the file itself.

    Mesher().read() rebuilds an OCCT face per triangle, which takes minutes on a fine
    mesh. The divergence-theorem sum over the stored triangles is the same volume.
    """
    with zipfile.ZipFile(path) as archive:
        model = ElementTree.fromstring(archive.read("3D/3dmodel.model"))
    total = 0.0
    for mesh in model.iter(f"{{{CORE_3MF}}}mesh"):
        points = [(float(v.get("x")), float(v.get("y")), float(v.get("z")))
                  for v in mesh.iter(f"{{{CORE_3MF}}}vertex")]
        for tri in mesh.iter(f"{{{CORE_3MF}}}triangle"):
            a, b, c = (points[int(tri.get(key))] for key in ("v1", "v2", "v3"))
            total += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                      + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6
    return abs(total)


def round_trip_rows(part: Part, stem: Path) -> list[Check]:
    """Read the written STEP and 3MF back and compare volumes with the solid."""
    step = import_step(stem.with_suffix(".step"))
    step_drift = abs(step.volume - part.volume) / part.volume
    mesh_volume = mesh_3mf_volume(stem.with_suffix(".3mf"))
    mesh_drift = abs(mesh_volume - part.volume) / part.volume
    return [
        Check("STEP round trip volume", step_drift <= STEP_DRIFT, f"{step_drift:.2e} (max 5e-3)"),
        Check("3MF read back volume", mesh_drift <= MESH_DRIFT, f"{mesh_drift:.2e} (max 1e-2)"),
    ]


def export_all(parts: dict[str, Part], log: list[dict]) -> bool:
    """Write into a scratch folder, read every file back, then replace exports/."""
    with tempfile.TemporaryDirectory() as scratch:
        staging = Path(scratch)
        passed = True
        for name, part in parts.items():
            printed = print_orientation(part, PRINT[name])
            export_part(printed, staging / name)
            passed = _report(f"export {name}", round_trip_rows(printed, staging / name),
                             log) and passed
        if not passed:
            return False
        EXPORTS.mkdir(exist_ok=True)
        for stale in EXPORTS.iterdir():
            if stale.suffix in SUFFIXES and stale.stem not in parts:
                stale.unlink()
        for written in staging.iterdir():
            shutil.copy2(written, EXPORTS / written.name)
    return True


def main(argv: list[str]) -> None:
    spec = Params()
    if "--bom" in argv:
        print(bom_text(spec))
        return
    log: list[dict] = []
    parts = all_parts(spec)
    passed = True
    for name, part in parts.items():
        passed = _report(name, CHECKS[name](part, spec), log) and passed
    passed = _report("assembly", assembly_rows(spec), log) and passed
    if passed:
        passed = export_all(parts, log)
    report = {"src_sha256": source_hash(), "passed": passed,
              "checks": len(log), "failed": sum(not row["ok"] for row in log), "rows": log}
    target = EXPORTS / "report.json" if passed else PREVIEWS / "report.json"
    target.parent.mkdir(exist_ok=True)
    target.write_text(json.dumps(report, indent=1) + "\n")
    print(f"{report['checks']} checks, {report['failed']} failed. Report: {target}")
    if not passed:
        raise SystemExit(1)
    for name, part in parts.items():
        preview_part(print_orientation(part, PRINT[name]), PREVIEWS / f"{name}.png")


if __name__ == "__main__":
    main(sys.argv[1:])
