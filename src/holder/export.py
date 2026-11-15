"""STEP (canonical) and STL export in print orientation, with round-trip checks.

The exporter applies only a rigid transform (rotation + translation) into print
orientation: the PRINT bed-face normal points to -Z and the bed face sits at Z=0.
It never repairs geometry.

STL meshing uses ABSOLUTE linear/angular deflection. build123d 0.11 meshes with relative
deflection, which inflated the mesh volume by about 0.5% in an earlier project.
"""

import math
import struct
from pathlib import Path

import numpy as np
from build123d import Location, Part, Shape, Vector, export_step, import_step
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepTools import BRepTools
from OCP.StlAPI import StlAPI_Writer

from holder.common import Check, PartSpec, at_least, at_most

STL_LINEAR_DEFLECTION = 0.01  # mm, absolute chord error
STL_ANGULAR_DEFLECTION = 0.1  # rad
STEP_VOLUME_TOL = 0.005  # re-imported STEP volume within 0.5%
STL_VOLUME_TOL = 0.001  # STL mesh volume within 0.1% of the solid
BED_EPS = 1e-6  # mm: a face this close to Z=0 lies on the bed


def print_rotation(normal) -> Location:
    """Rotation that turns the bed-face normal `normal` to -Z."""
    n = Vector(*normal).normalized()
    down = Vector(0, 0, -1)
    axis = n.cross(down)
    if axis.length < 1e-9:
        return Location() if n.dot(down) > 0 else Location((0, 0, 0), (1, 0, 0), 180)
    angle = math.degrees(math.atan2(axis.length, n.dot(down)))
    return Location((0, 0, 0), tuple(axis.normalized()), angle)


def to_print(part: Shape, normal) -> Shape:
    """Rigid transform into print orientation: bed face down at Z=0, centered on X/Y."""
    rotated = part.moved(print_rotation(normal))
    box = rotated.bounding_box(optimal=True)  # tight box: only used to place the part
    shift = Vector(-(box.min.X + box.max.X) / 2, -(box.min.Y + box.max.Y) / 2, -box.min.Z)
    return rotated.moved(Location(shift))


def bed_area(shape: Shape) -> float:
    """Area of the planar faces that lie on Z=0 facing down."""
    area = 0.0
    for face in shape.faces():
        if not face.is_planar:
            continue
        normal = face.normal_at()
        if normal.Z < -1 + 1e-9 and abs(face.center().Z) < BED_EPS:
            area += face.area
    return area


def write_step(shape: Shape, path: Path) -> Path:
    if not export_step(shape, path):
        raise RuntimeError(f"STEP export failed: {path}")
    return path


def write_stl(
    shape: Shape,
    path: Path,
    linear: float = STL_LINEAR_DEFLECTION,
    angular: float = STL_ANGULAR_DEFLECTION,
) -> Path:
    """Binary STL meshed with absolute deflection (isRelative=False)."""
    BRepTools.Clean_s(shape.wrapped)  # drop any coarser mesh cached on the faces
    mesh = BRepMesh_IncrementalMesh(shape.wrapped, linear, False, angular, True)
    if not mesh.IsDone():
        raise RuntimeError(f"meshing failed: {path}")
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    if not writer.Write(shape.wrapped, str(path)):
        raise RuntimeError(f"STL export failed: {path}")
    return path


def stl_volume(path: Path) -> float:
    """Enclosed volume of a binary STL (signed tetrahedra from the origin)."""
    data = Path(path).read_bytes()
    (count,) = struct.unpack_from("<I", data, 80)
    if len(data) != 84 + 50 * count:
        raise ValueError(f"not a binary STL: {path}")
    record = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    v = np.frombuffer(data, dtype=record, count=count, offset=84)["v"].astype(np.float64)
    return float(np.einsum("ij,ij->i", v[:, 0], np.cross(v[:, 1], v[:, 2])).sum() / 6)


def step_volume(path: Path) -> tuple[float, int]:
    """Volume and solid count of a STEP file, re-imported."""
    shape = import_step(path)
    return shape.volume, len(shape.solids())


def relative_error(measured: float, reference: float) -> float:
    return abs(measured - reference) / reference


def export_part(spec: PartSpec, part: Part, out_dir: Path) -> list[Check]:
    """Write out_dir/<name>.step and .stl in print orientation; return the export checks."""
    out_dir.mkdir(parents=True, exist_ok=True)
    printed = to_print(part, spec.print_normal)
    volume = part.volume
    step_path = write_step(printed, out_dir / f"{spec.name}.step")
    stl_path = write_stl(printed, out_dir / f"{spec.name}.stl")

    step_vol, step_solids = step_volume(step_path)
    step_err = relative_error(step_vol, volume)
    stl_err = relative_error(stl_volume(stl_path), volume)
    return [
        at_least("export: bed face on Z=0", bed_area(printed), 1.0, "mm2"),
        Check(
            "export: STEP round-trip",
            step_solids == 1 and step_err <= STEP_VOLUME_TOL,
            f"{step_solids} solid(s), volume err {step_err:.3%} (max {STEP_VOLUME_TOL:.1%})",
        ),
        at_most("export: STL volume err", stl_err * 100, STL_VOLUME_TOL * 100, "%"),
    ]
