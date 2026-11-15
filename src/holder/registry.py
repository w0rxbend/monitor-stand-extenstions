"""Part registry: the one place that lists every part, the reference solids and the
intended contacts.

Part module contract
--------------------
A part module (band.py: band_front, band_back; block.py; coupon.py) defines

    PARTS: list[PartSpec]    one entry per printed part
    ASSEMBLY_CHECKS          optional {assembly: fn(parts: dict[name, Part], p) -> list[Check]}
                             for checks that need several parts at once (e.g. seam holes
                             coaxial across the two band halves). They run when every part
                             of that assembly is built, after the pairwise clash checks.
                             `parts` includes the assembly's reference solids.
    REFERENCES               optional {assembly: fn(p) -> dict[name, Part]}: solids that are
                             not printed (the column), in assembly coordinates. They are
                             clash-checked with the parts and drawn in the assembly preview,
                             but never exported.

Each PartSpec (holder.common) carries the PLAN.md contract:

    name           NAME: unique; the file stem of out/<name>.step/.stl/.3mf/.png
    material       MATERIAL: key into params.MATERIALS ("TPU95A" or "PETG")
    print_normal   PRINT: outward normal of the bed face, in assembly coordinates
    build(p)       -> Part: one valid solid in ASSEMBLY coordinates. Never exports.
    checks(part, p) -> list[Check], each (name, ok, measured). Measures the real solid
                   (holder.common helpers); runs at build time in __main__ and in pytest.
    assembly       parts with the same assembly name are clash-checked pairwise and drawn
                   together. Default "holder"; coupon parts use "coupon" because they sit
                   where the band sits.

A single-part module also exposes NAME, MATERIAL, PRINT, build and checks at module level.
A multi-part module (coupon.py: three parts; band.py: front + back) writes one
build/checks pair per part, e.g. build_front / checks_front, and lists one PartSpec per
part. Shared geometry lives in band.py (the band cross-section, ribs, ears, seam bolts and
boss, used by the coupon and the full band) or, for generic cutters and measurements, in
common.py.

To add a module: import it here and append it to MODULES. Parts (or reference solids) of
one assembly that are meant to touch as assembled go into CONTACTS; every other pair must
keep a gap (common.clash_checks).
"""

from collections.abc import Callable

from build123d import Part

from holder import band, block, coupon
from holder.common import Check, PartSpec
from holder.params import DEFAULT, Params

MODULES = (band, block, coupon)

PARTS: list[PartSpec] = [spec for module in MODULES for spec in module.PARTS]

AssemblyChecks = Callable[[dict[str, Part], Params], list[Check]]
ASSEMBLY_CHECKS: dict[str, AssemblyChecks] = {
    group: fn for module in MODULES for group, fn in getattr(module, "ASSEMBLY_CHECKS", {}).items()
}

References = Callable[[Params], dict[str, Part]]
REFERENCES: dict[str, References] = {
    group: fn for module in MODULES for group, fn in getattr(module, "REFERENCES", {}).items()
}

# Intended contacts between parts of one assembly as assembled, by name.
# Band halves vs column: none as assembled. Each half keeps the TPU fit (0.25 mm) to the
# column; the contact (rib tips vs column) only exists once the seam bolts pull the halves
# on, and band.seam_checks checks it in that clamped position (band.CLAMP_CONTACTS).
# Block vs band_back: the plate face sits on the boss face (4 x M3 pull them together).
CONTACTS: tuple[tuple[str, str], ...] = ((band.BACK_NAME, block.NAME),)


def get(name: str) -> PartSpec:
    for spec in PARTS:
        if spec.name == name:
            return spec
    raise KeyError(f"unknown part {name!r}; known: {[s.name for s in PARTS]}")


def assemblies(specs: list[PartSpec] | None = None) -> dict[str, list[PartSpec]]:
    """Group parts by assembly name, keeping registry order."""
    groups: dict[str, list[PartSpec]] = {}
    for spec in PARTS if specs is None else specs:
        groups.setdefault(spec.assembly, []).append(spec)
    return groups


def references(group: str, p: Params) -> dict[str, Part]:
    """The reference solids of one assembly (empty if it has none)."""
    fn = REFERENCES.get(group)
    return fn(p) if fn else {}


def _validate() -> None:
    names = [spec.name for spec in PARTS]
    if len(set(names)) != len(names):
        raise ValueError(f"duplicate part names: {names}")
    ref_names = {group: set(fn(DEFAULT)) for group, fn in REFERENCES.items()}
    clash = set().union(*ref_names.values()) & set(names)
    if clash:
        raise ValueError(f"reference solids named like parts: {sorted(clash)}")
    groups = assemblies().items()
    members = [{s.name for s in specs} | ref_names.get(g, set()) for g, specs in groups]
    for a, b in CONTACTS:
        if not any({a, b} <= m for m in members):
            raise ValueError(f"contact ({a}, {b}): unknown name or different assemblies")
    unknown = (ASSEMBLY_CHECKS.keys() | REFERENCES.keys()) - assemblies().keys()
    if unknown:
        raise ValueError(f"assembly checks or references for unknown assemblies: {sorted(unknown)}")


_validate()
