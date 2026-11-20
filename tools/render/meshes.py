"""Mesh the hero assembly, render it in Blender, and write docs/renders.

    uv run --with pillow python tools/render/meshes.py [shot ...]

Each solid is tessellated at 0.02 mm (absolute chord) in its assembly pose and written
as OBJ with the surface's exact normals at every node, then
tools/render/scene.py traces it in Blender 5.2 (flatpak org.blender.Blender). The PNGs
are composited on the page background and cropped. They show a design not yet printed.
"""

import json
import subprocess
import sys
from pathlib import Path

from build123d import Part, Pos
from OCP.BRep import BRep_Tool
from OCP.BRepLib import BRepLib_ToolTriangulatedShape
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopAbs import TopAbs_REVERSED
from OCP.TopLoc import TopLoc_Location
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from holder.assembly import hero, joints, printed_parts  # noqa: E402
from holder.params import Params  # noqa: E402

WORK = ROOT / "out" / "render"
RENDERS = ROOT / "docs" / "renders"
CHORD, ANGLE = 0.02, 0.04
BACKGROUND = (245, 245, 247, 255)
MARGIN = 0.06
BLENDER = ["flatpak", "run", "org.blender.Blender", "-b", "--factory-startup", "--python"]
COLLAR = {"collar_front", "collar_back", "liner_front", "liner_back"}
HEAD = {"arm_head", "arm_link 2", "J3 knob"}


def _face_lines(face, first: int) -> tuple[list[str], int]:
    """OBJ lines for one meshed face, with the surface's own normals at every node.

    Blender would rebuild normals from the triangles; on the long thin triangles OCCT
    makes on a B-spline wall those wobble and the zebra bands jitter. These do not.
    """
    where = TopLoc_Location()
    mesh = BRep_Tool.Triangulation_s(face.wrapped, where)
    if mesh is None:
        return [], first
    BRepLib_ToolTriangulatedShape.ComputeNormals_s(face.wrapped, mesh)
    move = where.Transformation()
    flip = face.wrapped.Orientation() == TopAbs_REVERSED
    lines = []
    for index in range(1, mesh.NbNodes() + 1):
        point = mesh.Node(index).Transformed(move)
        normal = mesh.Normal(index).Transformed(move)
        sign = -1.0 if flip else 1.0
        lines.append(f"v {point.X():.5f} {point.Y():.5f} {point.Z():.5f}")
        lines.append(f"vn {sign * normal.X():.5f} {sign * normal.Y():.5f} "
                     f"{sign * normal.Z():.5f}")
    for index in range(1, mesh.NbTriangles() + 1):
        a, b, c = (first + node - 1 for node in mesh.Triangle(index).Get())
        if flip:
            b, c = c, b
        lines.append(f"f {a}//{a} {b}//{b} {c}//{c}")
    return lines, first + mesh.NbNodes()


def _mesh(part: Part, name: str) -> str:
    """OBJ in mm with exact per-node normals, tessellated at CHORD (absolute)."""
    BRepMesh_IncrementalMesh(part.wrapped, CHORD, False, ANGLE, True)
    path = WORK / f"{name.replace(' ', '_').replace('/', '-')}.obj"
    lines, first = [], 1
    for face in part.faces():
        face_lines, first = _face_lines(face, first)
        lines += face_lines
    if first == 1:
        raise RuntimeError(f"no triangles for {name}")
    path.write_text("\n".join(lines) + "\n")
    return str(path)


def _accessory_row(spec: Params) -> dict[str, Part]:
    """Post, phone and gopro in a row, cam_plate behind, zve10 in front: the accessory picture."""
    parts = printed_parts(spec)
    placed = {}
    for x, name in ((-60.0, "post"), (0.0, "phone"), (60.0, "gopro")):
        placed[f"row {name}"] = Pos(x, 400, 0) * parts[name]
    placed["row cam_plate"] = Pos(0, 520, 0) * parts["cam_plate"]
    placed["row zve10"] = Pos(0, 320, 0) * parts["zve10"]
    return placed


def manifest(spec: Params) -> dict:
    WORK.mkdir(parents=True, exist_ok=True)
    built = hero(spec)
    objects = []
    for name, item in built.items.items():
        groups = ["hero"]
        if name in COLLAR | {"column"}:
            groups.append("collar")
        if name in COLLAR:
            groups.append("plan")
        if name in {"collar_front", "collar_back"}:
            groups.append("shells")
        if name in HEAD:
            groups.append("head")
        objects.append({"name": name, "mesh": _mesh(item.part, name), "finish": item.finish,
                        "groups": groups})
    for name, part in _accessory_row(spec).items():
        objects.append({"name": name, "mesh": _mesh(part, name), "finish": "petg",
                        "groups": ["accessories"]})
    j1, _j2, _j3 = joints(spec)
    knob_detail = [j1.X + 8, j1.Y, j1.Z, 34]
    shots = [
        {"name": "hero", "group": "hero", "azimuth": 215, "elevation": 18, "size": [2000, 2200],
         "lens": 70},
        {"name": "side", "group": "hero", "azimuth": -90, "elevation": 2, "size": [1800, 2000],
         "lens": 90},
        {"name": "top", "group": "plan", "azimuth": 0, "elevation": 90, "size": [1800, 1800],
         "lens": 120},
        {"name": "detail", "group": "hero", "azimuth": 235, "elevation": 22,
         "size": [1800, 1400], "lens": 100, "detail": knob_detail, "fstop": 8.0},
        {"name": "collar", "group": "collar", "azimuth": 215, "elevation": 30,
         "size": [1800, 1400], "lens": 85},
        {"name": "zebra", "group": "shells", "azimuth": 215, "elevation": 30, "size": [1600, 1200],
         "lens": 85, "zebra": True},
        {"name": "zebra_head", "group": "head", "azimuth": 230, "elevation": 15,
         "size": [1400, 1400], "lens": 85, "zebra": True},
        {"name": "accessories", "group": "accessories", "azimuth": 200, "elevation": 32,
         "size": [2200, 1400], "lens": 70},
    ]
    return {"objects": objects, "shots": shots, "out": str(WORK / "png")}


def compose(raw: Path, target: Path, transparent: bool) -> None:
    picture = Image.open(raw).convert("RGBA")
    if transparent:
        card = Image.new("RGBA", picture.size, BACKGROUND)
        card.alpha_composite(picture)
        box = picture.getchannel("A").point(lambda a: 255 if a > 8 else 0).getbbox()
    else:
        card, box = picture, (0, 0, *picture.size)
    left, top, right, bottom = box
    pad = int(MARGIN * max(right - left, bottom - top))
    crop = (max(0, left - pad), max(0, top - pad), min(card.width, right + pad),
            min(card.height, bottom + pad))
    card.crop(crop).convert("RGB").save(target, optimize=True)


def main(names: list[str]) -> None:
    spec = Params()
    data = manifest(spec)
    path = WORK / "manifest.json"
    path.write_text(json.dumps(data, indent=1))
    script = ROOT / "tools" / "render" / "scene.py"
    subprocess.run([*BLENDER, str(script), "--", str(path), *names], check=True)
    RENDERS.mkdir(parents=True, exist_ok=True)
    for shot in data["shots"]:
        if names and shot["name"] not in names:
            continue
        compose(Path(data["out"]) / f"{shot['name']}.png", RENDERS / f"{shot['name']}.png",
                transparent=not shot.get("zebra"))
        print("wrote", RENDERS / f"{shot['name']}.png")


if __name__ == "__main__":
    main(sys.argv[1:])
