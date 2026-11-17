"""Shade docs/renders from STL meshes of the modeled parts.

Each PNG is a raster of triangles written by export_stl. Parts in an assembly
are meshed after they are moved onto the 1/4" holes, then drawn together.
"""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

from build123d import export_stl

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from shade_stl import load_stl, shade  # noqa: E402

from holder.accessories import BUILDERS, clamp_assembly  # noqa: E402
from holder.params import Params  # noqa: E402

PLASTIC = (226.0, 228.0, 232.0)
BLOCK = (206.0, 208.0, 212.0)
OUT = ROOT / "docs" / "renders"


def _mesh(part, directory: Path, name: str):
    path = directory / f"{name}.stl"
    if not export_stl(part, path, tolerance=0.08, angular_tolerance=0.25, ascii_format=False):
        raise RuntimeError(f"export failed: {path}")
    return load_stl(path), PLASTIC if name != "block" else BLOCK


def main() -> None:
    spec = Params()
    parts = {part.label: part for part in (builder(spec) for builder in BUILDERS)}
    stack = clamp_assembly(spec)

    shots = {
        "hero": (
            ("block", stack["block"]),
            ("base", stack["hinge_base"]),
            ("link", stack["hinge_link"]),
            ("plate", stack["cam_plate"]),
        ),
        "arm": (
            ("block", stack["block"]),
            ("base", stack["hinge_base"]),
            ("link", stack["hinge_link"]),
        ),
        "platform": (("plate", parts["cam_plate"]),),
        "phone": (("phone", parts["phone"]),),
        "gopro": (("gopro", parts["gopro"]),),
        "zve10": (("zve10", parts["zve10"]),),
        "post": (("post", parts["post"]),),
    }
    with tempfile.TemporaryDirectory() as tmp:
        folder = Path(tmp)
        for shot, items in shots.items():
            meshes = [_mesh(part, folder, f"{shot}-{name}") for name, part in items]
            shade(meshes, OUT / f"{shot}.png")
            print(shot)


if __name__ == "__main__":
    main()
