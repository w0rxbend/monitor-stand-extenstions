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

from holder.accessories import (  # noqa: E402
    BUILDERS,
    JOINT_Y,
    LINK,
    on_link_face,
    place,
)
from holder.block import build as build_block  # noqa: E402
from holder.block import riser_top, top_hole_xy  # noqa: E402
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
    brace = build_block(spec)
    _qx, quarter_y = top_hole_xy(spec)[1]
    shift_y = quarter_y + 16.0
    shift_z = riser_top(spec) + 0.4
    base = place(parts["hinge_base"], shift_y, shift_z)
    mid = place(parts["hinge_mid"], shift_y + JOINT_Y, shift_z)
    link_1 = place(parts["hinge_link"], shift_y + JOINT_Y, shift_z)
    link_2 = place(parts["hinge_link"], shift_y + JOINT_Y + LINK, shift_z)
    plate = on_link_face(parts["cam_plate"], shift_y + JOINT_Y + LINK, shift_z)

    shots = {
        "hero": (
            ("block", brace),
            ("base", base),
            ("mid", mid),
            ("link", link_2),
            ("plate", plate),
        ),
        "arm": (("block", brace), ("base", base), ("link", link_1)),
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
