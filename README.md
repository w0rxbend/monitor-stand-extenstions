# Monitor stand holder

A clamp-on extension point for a monitor stand column, written as CAD code (build123d).

- **TPU band** in two halves (front, back) clamps the column right under its top end, above
  the monitor carriage. Two M3 bolts per seam (left and right) pull the halves together.
- **Boss** on the back half carries a **PETG L-block** (plate, arm, riser) on 4 x M3.
- The riser's **top face** is the extension point: one 1/4"-20 and four M3 (20 mm square),
  all captive nuts in side-slot pockets. It is meant for a future 2-DOF arm (yaw ring +
  pitch hinge) leaning forward over the monitor; that arm is not part of this repo.

The design, measurements and decisions live in [PLAN.md](PLAN.md); sketches and photos in
`refs/`. Coordinates: Z up, +Y back, origin on the column axis at the band bottom.

## Build

Everything runs in a Docker image. Building the image needs network once, to fetch the
Python dependencies pinned in `uv.lock`:

```sh
docker build -t holder-cad -f docker/Dockerfile .
```

Every run afterwards goes through `./dock`: `--network none`, no published ports, all
capabilities dropped, read-only root, the repo mounted at `/work`.

```sh
./dock python -m holder           # build all parts, run checks, write out/ (exit 1 on any failed check)
./dock python -m holder block     # only some parts
./dock python -m holder --no-png  # skip the previews
./dock pytest -q                  # checks on every part + negative tests (about 6 min)
./dock ruff format . && ./dock ruff check .
```

`out/` gets `<part>.step` (canonical), `<part>.stl` and `<part>.png` for every part, plus
`assembly.png` (holder) and `assembly-coupon.png`. A part that fails a check gets a PNG for
debugging but no STEP/STL.

## Parts

| Part | Material | Print orientation (bed face) | Settings |
| --- | --- | --- | --- |
| `band_front` | TPU 95A | band bottom on the bed, band axis vertical | 25 mm/s, 4 walls |
| `band_back` | TPU 95A | band bottom on the bed, band axis vertical (boss bottom flush) | 25 mm/s, 4 walls, 100% infill in the boss |
| `block` | PETG | on its -X side (the L profile flat on the bed) | 4 walls, 40% gyroid |
| `coupon_band_front` | TPU 95A | as `band_front` (12 mm slice) | as the band |
| `coupon_band_back` | TPU 95A | as `band_back` (12 mm slice, no boss) | as the band |
| `coupon_strip` | PETG | on its -Y face (nut slots point up) | as the block |

Notes:
- Horizontal holes print about 0.15 mm small; the 3.4 mm M3 through-hole covers it.
- The block's nut slots on the -X side open on the bed face; their mouths carry a 0.6 mm
  chamfer against elephant's foot. The other slots open upward and need no support.
- The seam nut pockets in the TPU are horizontal hex holes printed flat-up (a 3.3 mm
  bridge). Any sag makes the pocket tight: the coupon tests it.

## Hardware

Counted from the model (hole, recess and pocket counts are checked at build time) and
checked against the real solids (`seam ...: M3x16 + washer + nut fit the ear stack`,
`boss bolt ...: M3x16 reaches through the plate nut`):

| Qty | Item | Where |
| --- | --- | --- |
| 8 | M3x16 ISO 7380 button head (head 5.7 x 1.65 mm) | 4 at the seams, 4 boss to block |
| 8 | M3 washer DIN 125 (7.0 x 0.5) | under every M3x16 head |
| 12 | M3 hex nut ISO 4032 (5.5 AF x 2.4) | 4 seam (back ears), 4 plate, 4 top face |
| 1 | 1/4"-20 hex nut (7/16" AF x 7/32") | top face, center |

- **Seam bolts**: head and washer on the front ear, nut in the back ear's hex pocket. The
  stack needs 14.3 mm, so an M3x16 ends 1.7 mm past the nut with the seam open (4 mm gap);
  closing the seam only adds length (5.7 mm past the nut with the ears touching) and the
  tip stays clear of the band, the block and the column.
- **Boss bolts**: inserted from the column side of the back half, head on its washer in the
  recess (1 mm below the inner band face, 2 mm off the column when clamped). Stack: washer
  0.5 + TPU boss 10.5 + PETG wall 1.5 + nut 2.4 = 14.9 mm, so the M3x16 ends 1.1 mm past
  the plate nut, with 3.1 mm of blind hole left beyond the tip.
- **Top face**: the 1/4"-20 nut sits under a 2.5 mm top wall: a short 1/4" accessory screw
  with 6 mm of thread engages 3.5 mm (2.8 threads); the blind hole is 18.3 mm deep for
  longer screws. The M3 nuts sit 10 mm deep: an M3 accessory bolt needs 12.4 mm plus the
  accessory's thickness (e.g. M3x16 for up to 3.6 mm). Not included above, depends on the
  accessory.
- Coupon: 2 x M3x16 + 2 washers + 2 nuts for the slice, 1 x M3 nut and 1 x 1/4"-20 nut
  for the strip; a short 1/4" screw and an M3 bolt to test the strip's threads.

## Before printing: measure the free column length

The band is 35 mm tall (`band_h`) and clamps the column between its top end and the
monitor carriage. From `refs/stand-photos/side-view.jpg` that free length looks like
**roughly 35-40 mm**, a rough estimate from a perspective photo. Measure it with the
monitor at its highest position before printing. If it is under 35 mm, lower `band_h` (and
`ear_bolt_z`; `validate()` keeps `boss_w <= band_h` and the nut walls) and rebuild.

## Coupon first

PLAN.md estimated about 25 min; with TPU at 25 mm/s (about 11.6 cm3 of TPU plus 6.5 cm3 of PETG) it
is likely longer, so check the slicer.

1. Print `coupon_band_front`, `coupon_band_back` (TPU) and `coupon_strip` (PETG).
2. Press the seam nuts into the back slice, put the slice on the column where the band will
   sit, bolt it with 2 x M3x16 + washers.
   It should clamp firmly **with seam gap left** at both seams.
3. Push the M3 and 1/4"-20 nuts into the strip's slots: they should press in and hold
   (not fall out, not spin). Thread a short 1/4" screw and an M3 bolt into them from the top
   face.

Tuning after the coupon (edit `src/holder/params.py`, then rebuild and re-run the checks):

- Band too loose (seams close fully before it grips): lower `column_d` by 1.
- Band does not fit on the column: raise the binding dimension (`column_w` or `column_d`)
  by 1.
- Nut pocket too tight: add 0.1 to its across-flats (`m3_hex_af` or `quarter_hex_af`).
- Gaps at the column sides: fall back to a spline profile digitized from
  `refs/stand-photos/profile-tracing.jpg` (PLAN.md "Column").

The seams can take up at most `seam_gap` minus the fit, 4 - 2 x 0.25 = 3.5 mm of column
depth (both seams close together); the checks keep at least 2 mm of that open as reserve.

## CAD checks are not physical validation

`python -m holder` checks every part on the real geometry: one valid solid, hole and
pocket counts, coaxial holes across parts, walls against the material minimum (TPU 1.2 mm,
PETG 1.5 mm), zero-volume clashes, the bolt stacks above, and a STEP round trip. Passing
them only means the model matches its own design rules. **The real proof is the coupon and
then a 24 h test with a phone on the arm.**

## Risks

- TPU can sag or creep under a long arm, and the future arm puts a forward bending moment
  and a yaw torque on the top face. Fallback: band in PETG plus a TPU liner strip; only
  `band.py` changes.
- The 1/4"-20 nut pocket size (11.4 AF x 5.8) is new, not proven in an earlier project. The
  coupon checks it before the full print.
- Several values are estimates (marked `# estimated:` in `params.py`): the column corner
  radius, the rib contact width, the PETG elephant's foot, the short 1/4" screw length.
- The free column length above the monitor carriage is a photo estimate (see above).
