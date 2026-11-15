"""Accessory solids, open bores, and hinge clearance."""

from pathlib import Path

import pytest
from build123d import Cylinder, Pos, import_step

from holder.accessories import (
    BUILDERS,
    JOINT_Y,
    LINK,
    PRINT,
    build_cam_plate,
    checks,
    on_link_face,
    place,
)
from holder.common import intersection_volume, print_orientation
from holder.export import export_part
from holder.params import Params


@pytest.fixture(scope="module")
def built() -> tuple[Params, dict]:
    spec = Params()
    parts = {part.label: part for part in (builder(spec) for builder in BUILDERS)}
    return spec, parts


def _failed(results: list[tuple[str, bool, str]]) -> list[tuple[str, str]]:
    return [(name, detail) for name, ok, detail in results if not ok]


def test_each_accessory_is_one_solid_with_open_bores(built: tuple) -> None:
    spec, parts = built
    assert set(parts) == set(PRINT)
    for name, part in parts.items():
        assert not _failed(checks(part, spec)), name
        printed = print_orientation(part, PRINT[name])
        assert printed.label == name
        assert abs(printed.bounding_box().min.Z) < 1e-4


def test_hinges_clear_when_the_tang_sits_in_the_clevis(built: tuple) -> None:
    _spec, parts = built
    base = parts["hinge_base"]
    mid = place(parts["hinge_mid"], JOINT_Y)
    link = place(parts["hinge_link"], JOINT_Y)
    outer = place(parts["hinge_link"], JOINT_Y + LINK)
    assert intersection_volume(base, link) < 1.0
    assert intersection_volume(base, mid) < 1.0
    assert intersection_volume(mid, outer) < 1.0


def test_camera_plate_clears_the_link_face(built: tuple) -> None:
    _spec, parts = built
    link = place(parts["hinge_link"], JOINT_Y + LINK)
    plate = on_link_face(parts["cam_plate"], JOINT_Y + LINK)
    assert intersection_volume(link, plate) < 1.0


def test_plugged_camera_plate_mount_fails(built: tuple) -> None:
    spec, parts = built
    plug = Pos(0, -24, 2.5) * Cylinder(2, 4)
    failed = _failed(checks(parts["cam_plate"] + plug, spec))
    assert any(name == "mount bore open" for name, _detail in failed)


def test_cam_plate_exports_step_stl_and_3mf(tmp_path: Path) -> None:
    part = build_cam_plate(Params())
    step, stl, mesh = export_part(part, tmp_path / part.label)
    assert stl.stat().st_size > 0
    assert mesh.suffix == ".3mf" and mesh.read_bytes()[:2] == b"PK"
    brought_back = import_step(step)
    assert abs(brought_back.volume - part.volume) / part.volume < 0.005
