"""Every printed part passes its checks, and the checks catch a broken part."""

from pathlib import Path

import pytest
from build123d import Pos, Vector, import_step

from holder import arm, collar, coupons
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
    printed = print_orientation(parts["arm_base"], PRINT["arm_base"])
    export_part(printed, tmp_path / "arm_base")
    assert all(row.ok for row in round_trip_rows(printed, tmp_path / "arm_base"))
    assert abs(import_step(tmp_path / "arm_base.step").bounding_box().min.Z) < 1e-6


def test_thin_seam_insert_wall_fails(parts: dict, spec: Params) -> None:
    x, z = collar.seam_points(spec)[1]
    thinned = parts["collar_front"] - box_from(x + 3.6, -9, z - 1, x + 5.5, -3, z + 1)
    assert any("insert wall" in name for name, _ in _failed(collar.checks_front(thinned, spec)))


def test_coupon_off_the_column_fails(parts: dict, spec: Params) -> None:
    shifted = Pos(0.3, 0, 0) * parts["coupon_collar_front"]
    shifted.label = "coupon_collar_front"
    assert any("measured column" in name for name, _ in _failed(coupons.checks(shifted, spec)))


def test_shell_surface_is_g2_except_its_bed_lead(parts: dict, spec: Params) -> None:
    back = parts["collar_back"]

    def on_bed(centre: Vector) -> bool:
        return abs(centre.Z - spec.collar_h) < 1e-4

    assert all(row.ok for row in skin_rows(back, "back", on_bed, collar.skin(spec)))
    # Without the bed-lead exception the probe must see the 45 degree crease.
    assert not all(row.ok for row in skin_rows(back, "back", lambda _c: False,
                                               collar.skin(spec)))


def test_a_boss_fused_onto_the_pedestal_shows_its_crease(parts: dict, spec: Params) -> None:
    """The finished-part probe sees a junction between two fused volumes."""
    boss = Pos(0, 0, 8) * box_from(-30, -4, 0, -15, 4, 6)
    fused = parts["arm_base"] + boss
    rows = skin_rows(fused, "fused", lambda c: abs(c.Z) < 1e-4, arm.pedestal_skin(spec) + boss)
    assert not all(row.ok for row in rows)
