"""Part registry: the one place that lists every part and the intended contacts.

Part module contract
--------------------
A part module (coupon.py now; band and block parts later) defines

    PARTS: list[PartSpec]    one entry per printed part
    ASSEMBLY_CHECKS          optional {assembly: fn(parts: dict[name, Part], p) -> list[Check]}
                             for checks that need several parts at once (e.g. seam holes
                             coaxial across the two band halves). They run when every part
                             of that assembly is built, after the pairwise clash checks.

Each PartSpec (holder.common) carries the PLAN.md contract:

    name           NAME: unique; the file stem of out/<name>.step/.stl/.png
    material       MATERIAL: key into params.MATERIALS ("TPU95A" or "PETG")
    print_normal   PRINT: outward normal of the bed face, in assembly coordinates
    build(p)       -> Part: one valid solid in ASSEMBLY coordinates. Never exports.
    checks(part, p) -> list[Check], each (name, ok, measured). Measures the real solid
                   (holder.common helpers); runs at build time in __main__ and in pytest.
    assembly       parts with the same assembly name are clash-checked pairwise and drawn
                   together. Default "holder"; coupon parts use "coupon" because they sit
                   where the band sits.

A single-part module also exposes NAME, MATERIAL, PRINT, build and checks at module level.
A multi-part module (coupon.py: three parts; the band: front + back) writes one
build/checks pair per part, e.g. build_front / checks_front, and lists one PartSpec per
part. Shared geometry lives in its own module (band.py: the band cross-section, ears and
seam bolts, used by the coupon and the full band) or, for generic cutters and
measurements, in common.py.

To add a module: import it here and append it to MODULES. Parts of one assembly that are
meant to touch go into CONTACTS; every other pair must keep a gap (common.clash_checks).
"""

from collections.abc import Callable

from build123d import Part

from holder import coupon
from holder.common import Check, PartSpec
from holder.params import Params

MODULES = (coupon,)

PARTS: list[PartSpec] = [spec for module in MODULES for spec in module.PARTS]

AssemblyChecks = Callable[[dict[str, Part], Params], list[Check]]
ASSEMBLY_CHECKS: dict[str, AssemblyChecks] = {
    group: fn for module in MODULES for group, fn in getattr(module, "ASSEMBLY_CHECKS", {}).items()
}

# Intended contacts between parts of one assembly, by part name.
# Example for Task 4: ("band_back", "block") -- the boss face sits on the plate.
CONTACTS: tuple[tuple[str, str], ...] = ()


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


def _validate() -> None:
    names = [spec.name for spec in PARTS]
    if len(set(names)) != len(names):
        raise ValueError(f"duplicate part names: {names}")
    for a, b in CONTACTS:
        if a not in names or b not in names or get(a).assembly != get(b).assembly:
            raise ValueError(f"contact ({a}, {b}): unknown part or different assemblies")
    unknown = ASSEMBLY_CHECKS.keys() - assemblies().keys()
    if unknown:
        raise ValueError(f"assembly checks for unknown assemblies: {sorted(unknown)}")


_validate()
