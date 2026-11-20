"""The hero assembly: stops, clash at 1e-6, rosettes, sweeps, access, the door, the BOM."""

import re
from pathlib import Path

import pytest
from build123d import Pos, Rot

from holder import arm
from holder.assembly import (
    assembly_rows,
    bom_text,
    camera_frame,
    column,
    door_region,
    hero,
    joints,
)
from holder.common import box_from, intersection_volume
from holder.params import Params

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


def test_half_index_rosette_clashes(spec: Params) -> None:
    built = hero(spec)
    j1, _j2, _j3 = joints(spec)
    off_index = Pos(spec.ball_flat_x + arm.pitch_gap(spec), j1.Y, j1.Z) * (
        Rot(spec.hero_angles[0] + 7.5, 0, 0) * arm.build_link(spec))
    assert intersection_volume(built.items["arm_base"].part, off_index) > 1.0


def test_a_column_past_the_install_limit_hits_the_arm(spec: Params) -> None:
    built = hero(spec)
    tall = column(Params(column_top_z=400.0))
    moving = [item.part for item in built.items.values() if item.chain != "fixed"]
    assert sum(intersection_volume(tall, part) for part in moving) > 1.0


def test_a_plate_under_the_battery_door_is_caught(spec: Params) -> None:
    built = hero(spec)
    _socket, centre = camera_frame(spec, built)
    door = door_region(centre)
    bounds = door.bounding_box()
    plate = box_from(bounds.min.X, bounds.min.Y, bounds.max.Z - 4, bounds.max.X, bounds.max.Y,
                     bounds.max.Z)
    assert intersection_volume(door, plate) > 1.0
    assert intersection_volume(door, built.items["arm_head"].part) <= 1e-6


def test_a_shoulder_too_short_flattens_the_springs(spec: Params) -> None:
    stop = arm.joint_stop("J2")
    with pytest.raises(ValueError):
        stop.force(stop.clamped + 1.5, stop.l1)


def test_every_knob_is_the_same_part(spec: Params) -> None:
    built = hero(spec)
    knobs = [item.part for name, item in built.items.items() if name.endswith(" knob")]
    assert len(knobs) == 4
    volumes = {round(knob.volume, 3) for knob in knobs}
    assert len(volumes) == 1
