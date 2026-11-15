"""Every registered part: checks pass, exports round-trip, no clashes; params validation."""

import math
from dataclasses import replace
from functools import cache

import pytest
from build123d import Box, Pos

from holder import band, block, coupon, registry
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


def assembly_parts(group: str):
    """Every printed part of an assembly plus its reference solids (the column)."""
    parts = {spec.name: built(spec.name) for spec in registry.assemblies()[group]}
    return {**parts, **registry.references(group, DEFAULT)}


def test_column_is_a_reference_not_a_printed_part():
    for group in ("holder", "coupon"):
        refs = registry.references(group, DEFAULT)
        assert set(refs) == {band.COLUMN_NAME}
        assert band.COLUMN_NAME not in IDS  # so it is never exported
        column = refs[band.COLUMN_NAME]
        assert column.is_valid and len(column.solids()) == 1
    # Taller than the band at both ends.
    col = registry.references("holder", DEFAULT)[band.COLUMN_NAME]
    span = [v.Z for v in col.vertices()]
    assert min(span) < 0 and max(span) > DEFAULT.band_h


@pytest.mark.parametrize("group", sorted(registry.assemblies()))
def test_assembly_has_no_clashes(group):
    parts = assembly_parts(group)
    contacts = [pair for pair in registry.CONTACTS if set(pair) <= parts.keys()]
    assert not failures(clash_checks(parts, contacts))


@pytest.mark.parametrize("group", sorted(registry.ASSEMBLY_CHECKS))
def test_assembly_checks_pass(group):
    parts = assembly_parts(group)
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


BOSS_DEFECT = "boss bolt x=10 z=6"


def full_band_back(p, shift_boss=None, plug_recess=False):
    """The full back half (band_back) with one defect at boss bolt x=+10 z=6: the through-hole
    moved by `shift_boss`, or the head recess left out."""
    cutters = []
    for label, cs in band.cutter_groups(p, band.BACK, p.band_h, p.ear_bolt_z, boss=True):
        if label == BOSS_DEFECT:
            hole, recess, relief = cs
            if shift_boss:
                hole = Pos(*shift_boss) * hole
            cs = [hole, relief] if plug_recess else [hole, recess, relief]
        cutters += cs
    return cut(band.envelope(p, band.BACK, p.band_h, boss=True), cutters)


def defective_block(p, shift_plate=None, plug_top_m3=False):
    """The block with one defect: the plate hole at x=10 z=6 moved by `shift_plate`, or the
    nut pocket of the first top M3 (x=-10, front row) left out."""
    x, y = block.top_m3_points(p)[0]
    plugged = f"top M3 x={x:g} y={y:g}"
    cutters = []
    for label, (hole, pocket) in block.cutter_groups(p):
        if label == "plate M3 x=10 z=6" and shift_plate:
            hole = Pos(*shift_plate) * hole
        cutters.append(hole)
        if not (label == plugged and plug_top_m3):
            cutters.append(pocket)
    return cut(block.envelope(p), cutters)


NEGATIVE = {
    "block shifted plate hole": (
        block.checks,
        lambda: defective_block(DEFAULT, shift_plate=(0, 0, 0.3)),
        "plate nut holes coaxial with the boss holes (circular edges)",
    ),
    "block plugged top pocket": (
        block.checks,
        lambda: defective_block(DEFAULT, plug_top_m3=True),
        "top M3 nut pockets",
    ),
    "block oversize 1/4 nut pocket": (
        block.checks,
        lambda: block.build(replace(DEFAULT, quarter_hex_af=DEFAULT.quarter_hex_af + 0.2)),
        "top 1/4 nut pocket AF",
    ),
    "band_back shifted boss hole": (
        band.checks_back,
        lambda: full_band_back(DEFAULT, shift_boss=(0.3, 0, 0)),
        "M3 boss holes on design axes",
    ),
    "band_back plugged head recess": (
        band.checks_back,
        lambda: full_band_back(DEFAULT, plug_recess=True),
        "M3 head recesses",
    ),
    "band_back shallow head recess": (
        band.checks_back,
        lambda: band.build_back(replace(DEFAULT, m3_head_extra=0.2)),
        "boss bolt heads below the inner face",
    ),
    "band_back oversize nut pocket": (
        band.checks_back,
        lambda: band.build_back(replace(DEFAULT, m3_hex_af=DEFAULT.m3_hex_af + 0.2)),
        "M3 nut pocket AF",
    ),
    "band_front low ribs": (
        band.checks_front,
        lambda: band.build_front(replace(DEFAULT, rib_h=0.6)),
        "grip rib height over the inner face",
    ),
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
    # The M3 shank (3.0) on the design axis also rubs the shifted 3.4 hole: 1.7 - 0.3 < 1.5.
    assert failed == {
        "seam +X: bolt holes coaxial across the seam",
        "seam +X z=6: seam closed, bolt holds the nut and its tip is clear",
    }


def test_ribs_into_the_column_fail_the_clash_check():
    # Built for a column 1 mm shallower: the front rib tips reach 0.25 mm into the real one.
    front = band.build_front(replace(DEFAULT, column_d=DEFAULT.column_d - 1))
    parts = {band.FRONT_NAME: front, **registry.references("holder", DEFAULT)}
    failed = failed_names(clash_checks(parts))
    assert failed == {f"{band.FRONT_NAME} / {band.COLUMN_NAME}: gap"}


def test_short_seam_bolt_fails_the_hardware_check():
    p = replace(DEFAULT, seam_bolt_len=12.0)  # the stack needs 14.3 mm
    front, back = built(coupon.FRONT_NAME), built(coupon.BACK_NAME)
    failed = failed_names(band.seam_hardware_checks(front, back, p, coupon.bolt_zs(p)))
    assert failed == {f"seam {s} z=6: M3x12 + washer + nut fit the ear stack" for s in ("-X", "+X")}


def test_small_seam_gap_fails_the_clamp_reserve():
    p = replace(DEFAULT, seam_gap=1.0)  # 0.5 mm left once both halves sit on the column
    parts = {coupon.FRONT_NAME: coupon.build_front(p), coupon.BACK_NAME: coupon.build_back(p)}
    failed = failed_names(coupon.assembly_checks(parts, p))
    assert failed == {"clamped: seam -X gap still open", "clamped: seam +X gap still open"}


REACH = "M3x{:g} reaches through the plate nut"


@pytest.mark.parametrize(
    "length, reaches, room", [(14.0, False, True), (16.0, True, True), (20.0, True, False)]
)
def test_boss_bolt_length_against_the_measured_stack(length, reaches, room):
    # Head floor to boss face 11 mm TPU + 1.5 mm PETG wall + 2.4 mm nut = 14.9 mm;
    # the blind hole ends 8.1 mm into the plate, an M3x20 tip would end 9 mm in.
    p = replace(DEFAULT, boss_bolt_len=length)
    back = block.band_parts(p)[band.BACK_NAME]
    checks = block.boss_hardware_checks(built(block.NAME), back, p)
    reach = [c for c in checks if c.name.endswith(REACH.format(length))]
    tips = [c for c in checks if "room past the bolt tip" in c.name]
    assert len(reach) == len(tips) == 4
    assert all(c.ok == reaches for c in reach)
    assert all(c.ok == room for c in tips)


def test_seam_bolt_tip_hitting_something_with_the_seam_closed_fails():
    # M3x16 ends 1.7 mm past the nut (y = 8.5) as built, 5.7 mm (y = 12.5) with the seam closed.
    p = DEFAULT
    front, back = built(coupon.FRONT_NAME), built(coupon.BACK_NAME)
    x = band.bolt_x(p, 1)
    obstacle = Pos(x, 10, 6) * Box(4, 2, 4)
    checks = band.seam_hardware_checks(front, back, p, coupon.bolt_zs(p), {"obstacle": obstacle})
    assert failed_names(checks) == {
        "seam +X z=6: seam closed, bolt holds the nut and its tip is clear"
    }
    assert not failures(band.seam_hardware_checks(front, back, p, coupon.bolt_zs(p)))


def test_deep_quarter_nut_fails_the_short_screw_check():
    p = replace(DEFAULT, top_quarter_nut_depth=4.5)  # 6 mm thread leaves 1.5 mm = 1.2 threads
    check = block.quarter_screw_check(block.build(p), p)
    assert not check.ok, check.measured
    assert block.quarter_screw_check(built(block.NAME), DEFAULT).ok


def test_block_into_the_boss_fails_the_clash_check():
    pushed = Pos(0, -0.3, 0) * built(block.NAME)
    failed = failed_names(block.clash_with_band(pushed, DEFAULT))
    assert failed == {f"{block.NAME} / {band.BACK_NAME}: contact"}


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
        ("rib_pitch", 2.5),  # ribs 3 mm wide at the face would overlap
        ("boss_w", 36.0),  # boss taller than the 35 mm band
        ("m3_bolt_head_h", 2.2),  # button head taller than its 2 mm recess
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
        {"plate_nut_wall": 1.2},  # PETG wall_min 1.5 in front of the plate nuts
        {"top_quarter_nut_depth": 1.2},  # top wall over the 1/4" nut
        {"top_m3_nut_depth": 9.0},  # 0.7 mm under the 1/4" pocket
        {"ear_bolt_z": (4.0, 26.0)},  # seam nut too close to the band bottom
        {"coupon_h": 8.0},  # coupon seam nut too close to the slice edge
    ],
)
def test_validate_rejects_thin_walls(changes):
    with pytest.raises(ValueError, match="wall_min"):
        replace(DEFAULT, **changes).validate()
