"""The hero assembly: stacks, clash at 1e-6, rosettes, sweeps, key access, BOM."""

import re
from pathlib import Path

import pytest
from build123d import Pos, Rot, Vector

from holder import arm, collar
from holder.assembly import assembly_rows, bom_text, column, hero, joints
from holder.common import intersection_volume, socket_screw
from holder.params import M3_SHCS, Params
from holder.stacks import pitch_gap

README = Path(__file__).resolve().parents[1] / "README.md"


@pytest.fixture(scope="session")
def spec() -> Params:
    return Params()


def test_assembly_rows_pass(spec: Params) -> None:
    failed = [(row.name, row.measured) for row in assembly_rows(spec) if not row.ok]
    assert not failed


def test_readme_bom_matches_the_code(spec: Params) -> None:
    text = README.read_text()
    block = re.search(r"<!-- bom:start -->\n(.*?)\n<!-- bom:end -->", text, re.S)
    assert block is not None
    assert block.group(1).strip() == bom_text(spec).strip()


def test_a_seam_screw_5_mm_too_long_hits_petg(spec: Params) -> None:
    built = hero(spec)
    good = built.items["seam screw 1"].part
    start = Vector(good.bounding_box().center().X, spec.seam_seat_y - 0.54, 10.0)
    too_long = socket_screw(M3_SHCS, collar.seam_screw_length(spec) + 5, start, Vector(0, 1, 0))
    assert intersection_volume(too_long, built.items["collar_back"].part) > 1e-3


def test_half_index_rosette_clashes(spec: Params) -> None:
    built = hero(spec)
    j1, _j2, _j3 = joints(spec)
    off_index = Pos(spec.arm_x0 + pitch_gap(spec), j1.Y, j1.Z) * (
        Rot(spec.hero_angles[0] + 7.5, 0, 0) * arm.build_link(spec))
    assert intersection_volume(built.items["arm_base"].part, off_index) > 1.0


def test_a_column_past_the_install_limit_hits_the_arm(spec: Params) -> None:
    built = hero(spec)
    tall = column(Params(column_top_z=260.0))
    moving = [item.part for item in built.items.values() if item.chain != "fixed"]
    assert sum(intersection_volume(tall, part) for part in moving) > 1.0
