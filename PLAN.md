# Monitor stand holder: plan

CAD as code: build123d, Python 3.12 via uv. Agent loop per task: edit, build, `pytest`, look at the PNG previews against sketch 5.
Patterns and values below are reused from `~/Projects/FreeCAD-Projects/Tigerbee 7inch` (accessories) and `Tigerbeetle V2`, where they already worked.

## Design (sketch 5)

1. **TPU band**, two halves (front, back) around the column. Seams left and right, `seam_gap` 4 mm each.
   2 x M3 bolts per seam through ears; the nuts sit in hex pockets so they do not spin in the TPU.
   Tightening the bolts clamps the band.
2. **Boss** on the back half: 4 x M3 through-holes on a 20 mm square, head recesses on the inner face so the heads never touch the column.
3. **PETG block** (L: plate, arm, riser). Boss screws go into captive M3 nuts in the plate.
   Top face = extension point: 1/4"-20 in the center, 4 x M3 on a 20 mm square, all **captive nuts in side-slot pockets**.
   No heat-set inserts: no earlier project used them, and nut sizes are standard while insert sizes vary by brand.

## Column (measured, plus traced profile)

| | Value | Source | Confidence |
| --- | --- | --- | --- |
| Width (X) | 40.2 mm | `refs/stand-photos/caliper-width-2.jpg` (direct); tracing reads 41.4 incl. pencil line | high |
| Depth (Y) | 55.5 mm | `refs/stand-photos/profile-tracing-depth.jpg` reads 56.0 incl. pencil line; direct `caliper-depth.jpg` 56 ±1 | high |
| Corner radius | 12 mm | `refs/stand-photos/profile-tracing*.jpg`: corners read 9–13 mm, sides slightly bowed (squircle-like) | medium |

The pencil line adds about 0.5 mm per side, so tracing values are reduced by about 1 mm.
Model the profile as a rounded rectangle 40.2 x 55.5, r = 12. The seams adjust depth by up to 8 mm;
TPU absorbs the squircle and taper difference. Fallback if the coupon shows gaps at the sides:
digitize `profile-tracing.jpg` into a spline profile (scale from the caliper in `profile-tracing-width.jpg`).

## Params (`params.py`)

Frozen dataclass. `validate()` raises `ValueError` on non-finite, non-positive, or a wall below the material minimum.
Each low-confidence value has a comment `# estimated: <source>`.

```
column_w=40.2 column_d=55.5 column_r=12
band_wall=4 band_h=35 seam_gap=4 ear_len=10 ear_t=5 ear_bolt_z=(9,26)
boss_w=32 boss_depth=12 block_size=32 arm_length=20 riser_height=40 fillet_r=3
m3_spacing=20
```

Materials and fasteners (copied from Tigerbee 7inch `accessories/_common.py`):

```
TPU95A: wall_min=1.2 fit=0.25      PETG: wall_min=1.5 fit=0.30
m3_through=3.4  m3_head_recess=6.6 x 2 (+1 extra so heads sit below the band face)
m3_hex_pocket=5.7 AF x 2.6
quarter_hex_pocket=11.4 AF x 5.8   # 7/16" nut 11.1 x 5.6 + fit: new value, confirm with the coupon
quarter_through=6.6
```

Z up. Origin = column axis at the band bottom. +Y = back. Parts export in print orientation, bed face at Z=0.

## Files

```
pyproject.toml          # requires-python ">=3.12,<3.13", build123d>=0.11,<0.12; dev: pytest, ruff; uv.lock committed
src/holder/
  params.py             # dataclass + validate() + MATERIALS
  common.py             # hex_pocket(), copied from Tigerbee 7inch _common.py
  band.py  block.py  coupon.py
  # each part module: NAME, MATERIAL, PRINT (bed-face normal), build(p) -> Part, checks(part, p) -> [(name, ok, measured)]
  export.py             # STEP (canonical) + STL; absolute mesh deflection fix copied from Tigerbee 7inch export.py
  preview.py            # ExportSVG to PNG via rsvg-convert, copied from Tigerbee 7inch accessories/_export.py
  __main__.py           # `uv run python -m holder` builds all, runs checks(), exports to out/
tests/test_parts.py     # runs checks() on every part, plus negative tests
out/                    # generated files, not committed
```

Rule from V2: part builders never export, and exporters never repair geometry.

## Checks (`checks()`, run at build time and in pytest)

- One valid solid per part.
- Expected hole and pocket counts.
- Coaxial bolt holes across the seams, and between the boss and the block nuts, measured on real circular edges (not bounding boxes; OCCT pads them).
- Zero-volume intersection (≤ 1e-6 mm³) between every pair of parts in the assembly. Name the intended contacts explicitly.
- Minimum wall around every pocket and hole ≥ the material `wall_min`, measured with solid distance.
- Export round-trip: re-imported STEP volume within 0.5%.
- Negative tests: a shifted hole, a plugged pocket and an oversize nut pocket must each fail `checks()`.

## Tasks

| # | Do | Done when |
| --- | --- | --- |
| 1 | `uv init`, pyproject as above, copy `hex_pocket`, the mesher fix and preview from Tigerbee 7inch. | A placeholder box exports STEP, STL and PNG; pytest runs. |
| 2 | `coupon`: one 12 mm band slice (both halves, 1 bolt per seam) plus a PETG strip with the M3 and 1/4" hex pockets and through-holes. | Checks pass. The user prints it (~25 min). The band clamps with seam gap left. The nuts press in and hold. |
| 3 | `band_front`, `band_back`: ears, nut pockets, 1 mm grip ribs every 5 mm, boss with head recesses. | Checks pass. |
| 4 | `block`: plate nuts for the boss; top face nuts at 2 depths (M3 at 6 mm, 1/4" at 14 mm) so the pockets do not cut into each other. | Checks pass, including ≥ 1.5 mm wall between pockets. No clash with the band. |
| 5 | Final export and README with print notes and hardware. | `out/` holds STEP and STL for every part, plus PNG previews. |

After the coupon: band too loose, lower `column_d` by 1. Band does not fit, raise the binding dimension by 1. Nut pocket too tight, add 0.1 to its AF. Rebuild.

## Print

- Band: TPU 95A, band axis vertical, 25 mm/s, 4 walls, 100% infill in the boss.
- Block: PETG, on its side, 4 walls, 40% gyroid. Horizontal holes print about 0.15 mm small; the 3.4 through-hole covers it.

## Hardware

- 4 x M3x20 + washer + nut (seams).
- 4 x M3x16 + nut (boss to block).
- 4 x M3 nut (top face).
- 1 x 1/4"-20 nut (top face).

## Refs

- **Sketches** (`refs/sketches/`):
  - `sketch-5-tpu-split-band.jpg`: current design.
  - `sketch-4-mount-face-m3-quarter-inch.jpg`: extension face.
  - `sketch-1-labeled.jpg`, `sketch-2.jpg`, `sketch-3.jpg`: first idea (L arm, riser).
- **Stand** (`refs/stand-photos/`):
  - `caliper-width.jpg`, `caliper-width-2.jpg`, `caliper-depth.jpg`: direct measurements on the cap.
  - `profile-tracing.jpg`: true top-view profile traced on paper (best shape reference).
  - `profile-tracing-width.jpg`, `profile-tracing-depth.jpg`: caliper on the tracing, width and depth.
  - `top-view-1.jpg`, `top-view-2.jpg`, `side-view.jpg`: shape and clearance.
  - Photos carry EXIF rotation: run `magick -auto-orient` before cropping.
- **Examples** (`refs/extension-examples/`), inspiration only, do not copy:
  - `robotic-arm-phone-stand.jpg`
  - `customizable-mechanical-arm-v2.jpg`
  - `mechanical-phone-holder-desk-arm.jpg`
- **Proven code** (copy, do not import across repos):
  - `~/Projects/FreeCAD-Projects/Tigerbee 7inch/src/tigerbee/accessories/_common.py`: `MATERIALS`, `hex_pocket`.
  - `~/Projects/FreeCAD-Projects/Tigerbee 7inch/src/tigerbee/accessories/_template.py`: the part module contract.
  - `~/Projects/FreeCAD-Projects/Tigerbee 7inch/src/tigerbee/export.py`: absolute mesh deflection fix.
  - `~/Projects/FreeCAD-Projects/Tigerbee 7inch/src/tigerbee/accessories/_export.py`: PNG previews.

## Lessons applied

- build123d 0.11 Mesher adds about 0.5% mesh volume. Use the absolute deflection fix.
- A clearance that only lived in tests was never enforced. Run `checks()` at build time too.
- Bounding boxes are padded. Measure real edges and solid distances.
- Passing CAD checks is not physical validation. The README says so; the coupon and a 24 h phone-on-arm test are the real proof.

## Risks

- TPU can sag under a long arm. Fallback: band in PETG plus a TPU liner strip; only `band.py` changes.
- The 1/4" nut pocket size is new, not proven in an earlier project. The coupon checks it before the full print.
