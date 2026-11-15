"""STEP, STL, and 3MF export.

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


def _write_3mf(part: Part, path: Path) -> None:
    mesher = Mesher(unit=Unit.MM)
    mesher.add_shape(
        part, linear_deflection=LIN, angular_deflection=ANG, part_number=part.label
    )
    mesher.write(path)
    if not path.is_file() or path.stat().st_size < 4 or path.read_bytes()[:2] != b"PK":
        raise RuntimeError(f"export failed: {path}")


def export_part(part: Part, stem: Path) -> tuple[Path, Path, Path]:
    """Write ``stem``.step, ``stem``.stl, and ``stem``.3mf. ``stem`` has no suffix."""
    stem.parent.mkdir(parents=True, exist_ok=True)
    step = stem.with_suffix(".step")
    stl = stem.with_suffix(".stl")
    mesh = stem.with_suffix(".3mf")
    _require(export_step(part, step, unit=Unit.MM, timestamp=STEP_TIMESTAMP), step)
    _mesh(part)
    _require(export_stl(part, stl, tolerance=LIN, angular_tolerance=ANG, ascii_format=False), stl)
    _write_3mf(part, mesh)
    return step, stl, mesh
