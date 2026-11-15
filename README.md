# Monitor stand holder

A TPU band clamps the monitor-stand column. A PETG L block bolts to that band and carries an extension face: four M3 nuts and one 1/4"-20 nut. Dimensions and the task list live in `PLAN.md`.

Passing CAD checks is not physical validation. The coupon, then a 24 h phone-on-arm test, are the real proof.

## Quick start

From the repo root, with Python 3.12 and `rsvg-convert` (PNG previews):

1. `uv sync`
2. `uv run python -m holder`

That builds every part, runs `checks()`, and writes `out/`. A failed check exits before export. `out/` is generated and is not committed.

## Commands

| Command | Description |
| --- | --- |
| `uv run python -m holder` | Check, then write STEP, STL, and PNG into `out/` |
| `uv run pytest` | Run `checks()`, the negative tests, and a STEP round-trip |
| `uv run ruff check src tests` | Lint |

## Parts

`out/` holds `coupon_front`, `coupon_back`, `coupon_strip`, `band_front`, `band_back`, and `block`. Each has `.step`, `.stl`, and `.png`. Files are in print orientation, bed face on Z = 0.

| Part | Material | Bed |
| --- | --- | --- |
| `coupon_front`, `coupon_back` | TPU 95A | bottom |
| `coupon_strip` | PETG | bottom |
| `band_front`, `band_back` | TPU 95A | bottom, band axis vertical |
| `block` | PETG | on its side (−X face down) |

Print the coupon first (~25 min). The band halves should clamp with the 4 mm seam gap still open. The M3 nut and the 1/4" nut should press into the strip and hold. The strip is a fit check, not part of the assembly.

## Print

- Band: TPU 95A, 25 mm/s, 4 walls, 100% infill in the boss.
- Block: PETG, 4 walls, 40% gyroid. Horizontal holes print about 0.15 mm small; the 3.4 mm through-hole covers that.
- Plate is 12 mm deep (nut plus a wall on both sides). The arm section is 16 mm so the 1/4" pocket stays above the arm.

## Hardware

- 4 × M3×20 + washer + nut (seams)
- 4 × M3×16 + nut (boss to block)
- 4 × M3 nut (top face)
- 1 × 1/4"-20 nut (top face)

No heat-set inserts. Nuts are captive in side slots.

## After the coupon

Do not change `column_d` or a nut AF until the coupon has been printed.

- Band too loose: lower `column_d` by 1 mm.
- Band does not fit: raise the binding dimension by 1 mm.
- Nut pocket too tight: add 0.1 mm to that pocket's across-flats.

`column_r` (12 mm) and the 1/4" pocket (11.4 mm across-flats, 5.8 mm thick) are estimates. The coupon is how you confirm the 1/4" nut.

## Layout

`src/holder/params.py` holds the frozen dimensions. Part modules build geometry and return `checks()`. They do not export. `export.py` writes STEP and STL and does not repair geometry. Builders and pytest both run the same checks.
