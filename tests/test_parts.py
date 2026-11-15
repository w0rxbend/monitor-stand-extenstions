"""Every registered part: checks pass, exports round-trip, no clashes; params validation."""

import math
from dataclasses import replace
from functools import cache

import pytest
from build123d import Pos

from holder import band, coupon, registry
from holder.common import clash_checks, cut
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


@pytest.mark.parametrize("group", sorted(registry.ASSEMBLY_CHECKS))
def test_assembly_checks_pass(group):
    parts = {spec.name: built(spec.name) for spec in registry.assemblies()[group]}
    checks = registry.ASSEMBLY_CHECKS[group](parts, DEFAULT)
    assert checks
    assert not failures(checks)


# --- Negative tests: a defective part must fail the check meant for that defect ---------------


def failed_names(checks) -> set[str]:
    return {c.name for c in checks if not c.ok}


def band_back(p, shift_hole=None, plug_nut=False):
    """The coupon's back band half, built from band.py's cutters with one defect at the +X seam:
    the hole moved by `shift_hole`, or the nut pocket left out."""
    cutters = []
    for seam, z in band.bolts(coupon.bolt_zs(p)):
        hole, nut = band.bolt_cutters(p, band.BACK, seam, z)
        if seam == 1 and shift_hole:
            hole = Pos(*shift_hole) * hole
        cutters.append(hole)
        if not (seam == 1 and plug_nut):
            cutters.append(nut)
    return cut(band.envelope(p, band.BACK, p.coupon_h), cutters)


def strip(p, shift_quarter=None, plug_m3=False):
    """The PETG strip with one defect: the 1/4" hole moved, or the M3 pocket left out."""
    cutters = coupon.strip_cutters(p)
    hole, pocket = cutters["1/4"]
    if shift_quarter:
        cutters["1/4"] = [Pos(*shift_quarter) * hole, pocket]
    if plug_m3:
        cutters["M3"] = cutters["M3"][:1]
    return cut(coupon.strip_envelope(), [c for cs in cutters.values() for c in cs])


NEGATIVE = {
    "band shifted hole": (
        coupon.checks_back,
        lambda: band_back(DEFAULT, shift_hole=(0, 0, 0.3)),
        "M3 seam holes on design axes",
    ),
    "band plugged pocket": (
        coupon.checks_back,
        lambda: band_back(DEFAULT, plug_nut=True),
        "M3 nut pockets",
    ),
    "band oversize nut pocket": (
        coupon.checks_back,
        lambda: coupon.build_back(replace(DEFAULT, m3_hex_af=DEFAULT.m3_hex_af + 0.2)),
        "M3 nut pocket AF",
    ),
    "strip shifted hole": (
        coupon.checks_strip,
        lambda: strip(DEFAULT, shift_quarter=(0.3, 0, 0)),
        "1/4 hole on its design axis",
    ),
    "strip plugged pocket": (
        coupon.checks_strip,
        lambda: strip(DEFAULT, plug_m3=True),
        "M3 nut pockets",
    ),
    "strip oversize nut pocket": (
        coupon.checks_strip,
        lambda: coupon.build_strip(replace(DEFAULT, quarter_hex_af=DEFAULT.quarter_hex_af + 0.2)),
        "1/4 nut pocket AF",
    ),
}


@pytest.mark.parametrize("case", NEGATIVE, ids=list(NEGATIVE))
def test_defect_fails_its_check(case):
    checks_fn, make, expected = NEGATIVE[case]
    checks = checks_fn(make(), DEFAULT)  # must report, not raise
    failed = failed_names(checks)
    assert expected in failed, f"{expected!r} did not fail; failed: {sorted(failed)}"
    assert "one valid solid" not in failed  # the defect, not a broken solid, trips the check
    # The cavity comparison against the design catches every one of these too.
    assert any("as designed" in name for name in failed)


def test_shifted_seam_hole_fails_the_seam_coaxial_check():
    parts = {
        coupon.FRONT_NAME: built(coupon.FRONT_NAME),
        coupon.BACK_NAME: band_back(DEFAULT, shift_hole=(0, 0, 0.3)),
    }
    failed = failed_names(coupon.assembly_checks(parts, DEFAULT))
    assert failed == {"seam +X: bolt holes coaxial across the seam"}


def test_small_seam_gap_fails_the_clamp_reserve():
    p = replace(DEFAULT, seam_gap=1.0)  # 0.5 mm left once both halves sit on the column
    parts = {coupon.FRONT_NAME: coupon.build_front(p), coupon.BACK_NAME: coupon.build_back(p)}
    failed = failed_names(coupon.assembly_checks(parts, p))
    assert failed == {"clamped: seam -X gap still open", "clamped: seam +X gap still open"}


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
        {"coupon_h": 8.0},  # coupon seam nut too close to the slice edge
    ],
)
def test_validate_rejects_thin_walls(changes):
    with pytest.raises(ValueError, match="wall_min"):
        replace(DEFAULT, **changes).validate()
