"""Every printed part passes its checks, and the checks catch a broken part."""

from dataclasses import replace
from pathlib import Path

import pytest
from build123d import Pos, Vector, import_step

from holder import collar, coupons
from holder.__main__ import round_trip_rows
from holder.common import box_from, print_orientation, skin_rows
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


def test_plugged_port_channel_fails_nut_travel(parts: dict, spec: Params) -> None:
    plug = box_from(-2, 40, 27, 2, 44, 31)
    failed = _failed(collar.mount_face(spec).checks(parts["collar_back"] + plug, spec))
    assert any("nut travel" in name for name, _ in failed)


def test_port_nut_without_its_drop_slides_back(parts: dict, spec: Params) -> None:
    built = collar.mount_face(spec).port_nut()
    nut = replace(built, drop=0.0)
    flat = parts["collar_back"] + (built.pocket() - nut.pocket())  # only the drop filled
    rows = [row for row in nut.checks(flat) if "play" in row.name]
    assert rows and not rows[0].ok


def test_thin_insert_wall_fails(parts: dict, spec: Params) -> None:
    x, z = collar.seam_points(spec)[0]
    thinned = parts["collar_back"] - box_from(x - 6, 4, z - 1, x - 3.5, 6, z + 1)
    assert any("insert wall" in name for name, _ in _failed(collar.checks_back(thinned, spec)))


def test_coupon_off_the_column_fails(parts: dict, spec: Params) -> None:
    shifted = Pos(0.3, 0, 0) * parts["coupon_collar_front"]
    shifted.label = "coupon_collar_front"
    assert any("measured column" in name for name, _ in _failed(coupons.checks(shifted, spec)))


def test_washer_on_a_cut_seat_fails_bearing(parts: dict, spec: Params) -> None:
    x, z = collar.seam_points(spec)[0]
    seat_y = spec.seam_seat_y
    hollowed = parts["collar_front"] - box_from(x - 3.6, seat_y, z - 3.6, x + 3.6, seat_y + 0.5,
                                                z + 1.0)
    failed = _failed(collar.checks_front(hollowed, spec))
    assert any("washer bearing" in name for name, _ in failed)


def test_shell_skin_is_g2_except_its_bed_lead(spec: Params) -> None:
    deck, ring = collar.shell_skins(spec, 1.0)

    def on_bed(centre: Vector) -> bool:
        return abs(centre.Z - spec.collar_h) < 1e-4

    assert all(row.ok for row in skin_rows(deck, "deck", on_bed))
    assert all(row.ok for row in skin_rows(ring, "ring", lambda _centre: False))
    # Without the bed-lead exception the probe must see the 45 degree crease.
    assert not all(row.ok for row in skin_rows(deck, "deck", lambda _centre: False))
