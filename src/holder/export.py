"""STEP and STL export.

build123d 0.11 meshes with isRelative=True, which scales the chord by edge length
and adds about 0.5% mesh volume. An absolute mesh is applied first; OCCT keeps a
finer triangulation, so the later relative pass does not coarsen it. Exporters
write the solid they are given and do not repair it.
"""

from pathlib import Path

from build123d import Mesher, Part, Unit, export_step, export_stl
from OCP.BRepMesh import BRepMesh_IncrementalMesh

STEP_TIMESTAMP = "2000-01-01T00:00:00"
LIN = 0.02  # mesh chord, mm
ANG = 0.1  # mesh angle, rad


def _mesh(shape: Part) -> None:
    BRepMesh_IncrementalMesh(shape.wrapped, LIN, False, ANG, True)


_relative_mesh_shape = Mesher._mesh_shape


def _absolute_mesh_shape(shape: Part, linear_deflection: float, angular_deflection: float):
    _mesh(shape)  # Mesher.add_shape deep-copies its input (dropping any mesh), then calls this
    return _relative_mesh_shape(shape, linear_deflection, angular_deflection)


Mesher._mesh_shape = staticmethod(_absolute_mesh_shape)


def _require(wrote: bool, path: Path) -> None:
    if not wrote:
        raise RuntimeError(f"export failed: {path}")


def export_part(part: Part, stem: Path) -> tuple[Path, Path]:
    """Write ``stem``.step and ``stem``.stl. ``stem`` has no suffix."""
    stem.parent.mkdir(parents=True, exist_ok=True)
    step = stem.with_suffix(".step")
    stl = stem.with_suffix(".stl")
    _require(export_step(part, step, unit=Unit.MM, timestamp=STEP_TIMESTAMP), step)
    _mesh(part)
    _require(export_stl(part, stl, tolerance=LIN, angular_tolerance=ANG, ascii_format=False), stl)
    return step, stl
