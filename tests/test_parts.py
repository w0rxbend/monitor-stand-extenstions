"""Every printed part passes its checks, and the checks catch a broken part."""

from pathlib import Path

import pytest
from build123d import Vector, import_step

from holder import collar
from holder.__main__ import round_trip_rows
from holder.common import box_from, print_orientation
from holder.export import export_part
from holder.params import Params
from holder.registry import CHECKS, PRINT, all_parts


@pytest.fixture(scope="session")
def spec() -> Params:
    return Params()


@pytest.fixture(scope="session")
def parts(spec: Params) -> dict:
    return all_parts(spec)


def _failed(rows) -> list[tuple[str, str]]:
    return [(row.name, row.measured) for row in rows if not row.ok]


@pytest.mark.parametrize("name", sorted(CHECKS))
def test_part_passes_its_checks(name: str, parts: dict, spec: Params) -> None:
    assert not _failed(CHECKS[name](parts[name], spec))


def test_export_round_trip(tmp_path: Path, parts: dict) -> None:
    printed = print_orientation(parts["arm_link"], PRINT["arm_link"])
    export_part(printed, tmp_path / "arm_link")
    assert all(row.ok for row in round_trip_rows(printed, tmp_path / "arm_link"))
    assert abs(import_step(tmp_path / "arm_link.step").bounding_box().min.Z) < 1e-6


def test_plugged_mf_channel_fails_nut_travel(parts: dict, spec: Params) -> None:
    plug = box_from(8, 36, 35.2, 12, 40, 38.2)
    failed = _failed(collar.mount_face(spec).checks(parts["collar_back"] + plug, spec))
    assert any("nut travel" in name for name, _ in failed)


def test_thin_seam_wall_fails(parts: dict, spec: Params) -> None:
    x, z = collar.seam_points(spec)[0]
    plug_free = parts["collar_back"] - box_from(x - 6, 4, z - 1, x - 3.5, 6, z + 1)
    failed = _failed(collar.checks_back(plug_free, spec))
    assert failed


def test_coupon_bore_is_the_collar_bore(parts: dict, spec: Params) -> None:
    from holder.common import first_hit

    for name, inside in (("coupon_collar_front", -10), ("coupon_collar_back", 10)):
        hit = first_hit(parts[name], Vector(0, inside, 10), Vector(1, 0, 0))
        assert hit == pytest.approx(spec.bore_half_w, abs=1e-3)
