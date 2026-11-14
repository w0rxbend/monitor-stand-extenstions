# Monitor stand holder: plan

CAD as code: build123d, Python 3.12 via uv. Agent loop per task: edit, build, `pytest`, check SVG previews against sketch 5.

## Design (sketch 5)

1. **TPU band**, two halves (front, back) around the column. Seams left and right, `seam_gap` 4 mm each.
   2 x M3 bolts per seam through ears. Tightening the bolts clamps the band.
2. **Boss** on the back half: 4 x M3 through-holes, counterbored from inside so the heads never touch the column.
3. **PETG block** (L: plate, arm, riser) bolted to the boss with M3 heat-set inserts.
   Top face = extension point: 1/4"-20 insert in the center, 4 x M3 inserts on a 20 mm square.

## Column (measured)

| | Value | Source |
| --- | --- | --- |
| Width (X) | 40.2 mm | `refs/stand-photos/caliper-width-2.jpg` |
| Depth (Y) | 56 mm ±1 | `refs/stand-photos/caliper-depth.jpg` |
| Corner radius | 11 mm (estimate) | `refs/stand-photos/top-view-*.jpg` |

The seams adjust depth by up to 8 mm. TPU absorbs radius and taper error.

## Params (`params.py`, mm)

```
column_w=40.2 column_d=56 column_r=11 fit=0.5
band_wall=4 band_h=35 seam_gap=4 ear_len=10 ear_t=5 ear_bolt_z=(9,26)
m3_clear_d=3.4 m3_head_d=6 m3_head_h=3.5
boss_w=32 boss_depth=12 block_size=32 arm_length=20 riser_height=40 fillet_r=3
m3_spacing=20 m3_insert_d=4.0 m3_insert_depth=6.5
quarter_insert_d=8.0 quarter_insert_depth=13.5 test_band_h=12
```

Z up. Origin = column axis at the band bottom. +Y = back.

## Files

```
src/holder/params.py  band.py  block.py  build.py
tests/test_geometry.py
out/   # STL + STEP + SVG
```

## Tasks

| # | Do | Done when |
| --- | --- | --- |
| 1 | `uv init --python 3.12`, `uv add build123d`, `uv add --dev pytest`. `build.py` exports STL, STEP and SVG. | A placeholder box exports. |
| 2 | `test_band`: both halves, 12 mm tall, 1 bolt per seam. | Valid solids. The user prints it (~20 min) and it clamps with seam gap left. |
| 3 | `band_front`, `band_back` with ears, grip ribs and the boss. | Valid. Bolt holes coaxial across the seams. Head recess ≥ `m3_head_h + 1`. |
| 4 | `block` with front inserts (boss) and top inserts (extension point). | Valid. Holes coaxial with the boss. ≥ 4 mm wall around every insert. No clash with the band. |
| 5 | Export all parts in print orientation. | `out/` holds `band_front`, `band_back`, `block`, `test_band` as STL and STEP. |

Test band too loose: lower `column_d` by 1. Does not fit: raise the binding dimension by 1. Rebuild.

## Print

- Band: TPU 95A, band axis vertical, 25 mm/s, 4 walls, 100% infill in the boss.
- Block: PETG, on its side, 4 walls, 40% gyroid. Press the inserts in with a soldering iron.

## Hardware

4 x M3x20 + washer + nyloc (seams). 4 x M3x16 (boss to block). 8 x M3 x 5.7 inserts. 1 x 1/4"-20 x 12.7 insert.

## Refs

- **Sketches** (`refs/sketches/`):
  - `sketch-5-tpu-split-band.jpg`: current design.
  - `sketch-4-mount-face-m3-quarter-inch.jpg`: extension face.
  - `sketch-1-labeled.jpg`, `sketch-2.jpg`, `sketch-3.jpg`: first idea (L arm, riser).
- **Stand** (`refs/stand-photos/`):
  - `caliper-width.jpg`, `caliper-width-2.jpg`, `caliper-depth.jpg`: measurements.
  - `top-view-1.jpg`, `top-view-2.jpg`, `side-view.jpg`: shape and clearance.
  - Photos carry EXIF rotation: run `magick -auto-orient` before cropping.
- **Examples** (`refs/extension-examples/`), inspiration only, do not copy:
  - `robotic-arm-phone-stand.jpg`
  - `customizable-mechanical-arm-v2.jpg`
  - `mechanical-phone-holder-desk-arm.jpg`

## Risks

- TPU can sag under a long arm. Fallback: band in PETG plus a TPU liner strip; only `band.py` changes.
- Insert holes assume common inserts. Other brands: change `*_insert_*`.
- No OCP wheel for Python 3.14: use 3.12. Last resort: manifold3d (no fillets, no STEP).
