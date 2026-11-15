"""Every registered part: checks pass, exports round-trip, no clashes; params validation."""

import math
from dataclasses import replace
from functools import cache

import pytest

from holder import registry
from holder.common import clash_checks
from holder.export import STEP_VOLUME_TOL, export_part, step_volume, to_print, write_step
from holder.params import DEFAULT, MATERIALS

SPECS = registry.PARTS
IDS = [spec.name for spec in SPECS]


@cache
def built(name: str):
    return registry.get(name).build(DEFAULT)


def failures(checks):
    return [c for c in checks if not c.ok]


def test_registry_contract():
    assert SPECS, "no parts registered"
    assert len(set(IDS)) == len(IDS)
    for spec in SPECS:
        assert spec.material in MATERIALS
        assert callable(spec.build) and callable(spec.checks)
        assert len(spec.print_normal) == 3


@pytest.mark.parametrize("spec", SPECS, ids=IDS)
def test_checks_pass(spec):
    checks = spec.checks(built(spec.name), DEFAULT)
    assert checks, "checks() returned nothing"
    assert all(len(c) == 3 for c in checks)
    assert not failures(checks)


@pytest.mark.parametrize("spec", SPECS, ids=IDS)
def test_step_round_trip(spec, tmp_path):
    part = built(spec.name)
    path = write_step(to_print(part, spec.print_normal), tmp_path / f"{spec.name}.step")
    volume, solids = step_volume(path)
    assert solids == 1
    assert abs(volume - part.volume) / part.volume <= STEP_VOLUME_TOL


@pytest.mark.parametrize("spec", SPECS, ids=IDS)
def test_export_checks_pass(spec, tmp_path):
    assert not failures(export_part(spec, built(spec.name), tmp_path))
    assert (tmp_path / f"{spec.name}.step").exists()
    assert (tmp_path / f"{spec.name}.stl").stat().st_size > 84


@pytest.mark.parametrize("group", sorted(registry.assemblies()))
def test_assembly_has_no_clashes(group):
    parts = {spec.name: built(spec.name) for spec in registry.assemblies()[group]}
    contacts = [pair for pair in registry.CONTACTS if set(pair) <= parts.keys()]
    assert not failures(clash_checks(parts, contacts))


def test_default_params_valid():
    assert DEFAULT.validate() is DEFAULT


@pytest.mark.parametrize(
    "field, value",
    [
        ("column_w", math.nan),
        ("band_h", math.inf),
        ("column_d", -55.5),
        ("seam_gap", 0.0),
        ("ear_bolt_z", (9.0, math.nan)),
        ("ear_bolt_z", (-9.0, 26.0)),
        ("ear_bolt_z", ()),
    ],
)
def test_validate_rejects_bad_values(field, value):
    with pytest.raises(ValueError, match=field):
        replace(DEFAULT, **{field: value}).validate()


def test_validate_rejects_non_numbers():
    with pytest.raises(TypeError, match="m3_through"):
        replace(DEFAULT, m3_through="3.4").validate()


@pytest.mark.parametrize(
    "changes",
    [
        {"band_wall": 1.1},  # TPU wall_min 1.2
        {"ear_t": 3.7},  # 1.1 left behind the 2.6 deep nut pocket
        {"boss_w": 26.0},  # M3 head recess breaks the boss edge
        {"block_size": 26.0},  # M3 nut pocket breaks the block edge
        {"ear_bolt_z": (4.0, 26.0)},  # seam nut too close to the band bottom
    ],
)
def test_validate_rejects_thin_walls(changes):
    with pytest.raises(ValueError, match="wall_min"):
        replace(DEFAULT, **changes).validate()
