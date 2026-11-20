# Design brief: the finished holder

Status: implemented in v0.6.0 on `feat/apple-form`. Section 9 lists what changed from v0.5 and why. PLAN.md stays as history. Where they differ, this file wins.

Next action for the engineer: print the coupons (section 6.4), melt an M6 shoulder screw head into a knob, and close the open items in section 8.

## 0. What v0.5 got wrong, and what this brief fixes

| # | Defect (v0.5 review) | Fix |
| --- | --- | --- |
| 1 | Form read as stacked rounded slabs: a collar deck over an inset ring, an arm_base slab tier, a 14.3 mm camera slab, drilled plates. Every body was a 2.5D sweep of one macropad outline. | Each part is one construction: a turned profile, or one skin. The collar is one barrel-walled body split by a straight seam. The pedestal and knob are turned. The head is one skin whose side outline sweeps into the camera platform. The drilled-plate accessories are gone (§9). |
| 2 | Almost no second-order curves: the collar had none, the plates bulged 0.25 mm. | Exact conics carry the form: the collar wall is a 2 mm parabola; its plan has quarter-ellipse front corners and a half-ellipse back; the pedestal is a ball, a circular waist, a quarter-ellipse flare and an elliptic foot shoulder; the head's platform sweeps into its J3 face through a 10 × 20 quarter ellipse; the knob crown is an ellipse. |
| 3 | G2 was measured on each skin before fusing, so fused creases were never seen. | Continuity is measured on the finished, cut part. Every edge whose two faces both lie on the uncut body's surface is judged (normal step ≤ 0.5°, curvature step ≤ 2e-3 /mm); a fused crease fails (test). Edges where a cut meets anything are counted as features. |
| 4 | The camera plate covered the whole ZV-E10 bottom, so the battery and card door could not open. | The camera sits on a 31 mm wide platform at the lens end. The door and 30 mm of swing room below it are a solid in the clash check, clear of every part. |
| 5 | Every joint turned about X in 15° steps: no pan, no fine aim. | Smooth pan on a pivot under the pedestal; J1 and J2 rosettes for reach; J3 a smooth friction joint on a TPU ring for fine tilt. Every tilt axis stays horizontal for any pan, so the horizon stays level without a roll joint (row). |
| 6 | The torque spec could not be applied, and normal tightening overloaded the inserts and washers. | Every joint is a spring stop (§3): the shoulder of an ISO 7379 screw lands on the brass insert, so any extra torque closes a steel and brass loop. The PETG only sees the force of a disc-spring stack whose compression is set by geometry. The spec is "turn until it stops". |

Axes: Z up, origin on the column axis at the collar bottom. +Y points away from the monitor (the column stands behind the monitor), so the camera's lens looks toward −Y, over the monitor, at the user.

## 1. Product concept and form language

### 1.1 The idea

One device on the column: a collar, a turned pedestal that pans on the collar's back, two links, and a head that is also the camera platform. Four identical knobs. No visible nut, no screw head on the front. The seam stops enter from the back.

The look follows machined camera hardware (turned bodies with milled flats, as on Falcam and SmallRig parts) and soft-solid products (one continuous surface per part). Milled flats, bores and the seam are crisp cut features with a 0.4 mm lead-in. Everything else is one G2 surface. This is our reading of those products, not a sourced spec.

### 1.2 Curves and surfaces (build123d 0.11.1)

| Curve or surface | Where | How | Why |
| --- | --- | --- | --- |
| **Turtle of conics** | Collar plan, pedestal profile, head outline | `form.Turtle`: lines, circular arcs, quarter ellipses that start at a vertex, each beginning where and how the last ended (G1). `soft(trim)` replaces up to `trim` at every join with a quintic that matches both curvatures. | Most of each curve stays an exact conic; every join is G2. The curvature jump at every join is a row (≤ 1e-3 /mm; measured ~1e-14). |
| **G2 blend** | Every turtle join, link waist, knob crown | `form.g2_blend`: quintic P0..P5, h = 5·a²·κ/4 at both ends | Matches position, tangent and curvature: line, circle, ellipse. |
| **G2 skin** | Collar, links, head | `form.g2_skin(outline, height, bottom, top, bulge)`: one edge profile swept round a G2 plan and sewn | No boolean or OCC fillet touches the outer surface. |
| **Parabolic wall** (a conic) | Collar | `bulge` 2.0 mm on a 30 mm wall | A barrel you can see. Its slope at the rolls is 15° from vertical, so the print side has no overhang. |
| **Turned profile** | Pedestal | `form.revolved`: one 360° revolve; the STEP round trip is a row | A ball (r 16.6), a circular waist (r 4), a quarter-ellipse flare (13 × 5), an elliptic foot shoulder (4 × 3), a 47° bed lead. |
| **Knob** | 4 knobs | two 180° revolve halves fused, elliptic crown a 11 × b 3, 24 flutes | One 360° revolve of the crown blend read back from STEP invalid in v0.5. |
| **Hirth rosette**, 24 teeth, 90° | J1, J2 | One tooth = `loft([V-triangle face, Vertex(axis point)])`, trimmed r 7.7–13 | All flank faces are planes. |

Rules that keep STEP and 3MF valid: never `offset()` a sketch with a Bezier; no OCC fillet on an outer edge; a roll must be smaller than the outline's smallest convex radius (a row); a closed turtle path starts at a corner, not between two collinear lines (the sweep fails there); after every part: valid, one solid, STEP volume within 0.5 %, 3MF within 1 %.

### 1.3 Edge vocabulary

| Edge | Treatment |
| --- | --- |
| Outer edge off the bed | G2 roll: collar bottom 6.0; head caps 2.0; link faces 1.2 |
| Outer edge on the print bed | 45° lead (collar 1.0, head 0.8, link 0.6, pedestal 1.2 at 47°), then a G2 blend into the wall. The only crease on the outer surface. |
| Milled flats, bores, spot faces | Crisp cut, 0.4 × 45° lead-in at hole mouths |
| Seam | One straight split, 2.0 mm open, 1.54 mm at liner contact |
| Seam counterbore mouths | Filled by flush TPU caps cut from the collar's own skin |

## 2. Assembly architecture

### 2.1 Printed parts

| Part | Material | What it is |
| --- | --- | --- |
| `collar_front` | PETG | Front shell. Four ruthex M5 inserts in its seam faces. |
| `collar_back` | PETG | Back shell with the half-ellipse back. Four seam counterbores from the back; the pan pivot bore, spot face and knob recess under the back. |
| `seam_cap_1..4` | TPU 95A | Covers for the seam counterbores, cut from the collar's skin: flush, a 0.1 mm press fit, 1.4 mm clear of the screw head over the whole seam travel. |
| `liner_front`, `liner_back` | TPU 95A | 2.0 mm grip liners with 45° V ribs, behind the shell lips. |
| `arm_base` | PETG | Turned pedestal. Pivot face with an M6 insert; a ball with the J1 rosette flat (+X) and the J1 knob flat (−X). |
| `arm_link` | PETG | J1 to J2. Dog-bone skin, 70 mm pitch, 15.06 thick, rosettes and M6 inserts at both ends. |
| `arm_tip` | PETG | J2 to J3. The same skin: end A takes the J2 knob, end B is a flat friction face with the J3 insert. |
| `arm_head` | PETG | J3 block (flat friction face, knob spot face) sweeping into the camera platform, 31 wide. |
| `j3_ring` | TPU 95A | 1.0 mm friction ring, Ø26 / Ø8.8. |
| `knob` × 4 | PETG | Turned, elliptic crown; holds a shoulder screw head (melted in). |
| `coupon_*` | as parent | Slices of the real parts (§6.4). |

### 2.2 Bought parts (modeled as solids in the assembly)

ISO 7379 shoulder screws (M5 6.5 × 35, M6 8 × 20 and 8 × 40), DIN 2093 disc springs (B 16 × 8.2 × 0.6 and C 14 × 7.2 × 0.35), ISO 7089 M8 washers, ruthex RX-M5x9.5 and RX-M6x12.7 heat-set inserts, a 1/4"-20 × 1/2" ASME B18.3 camera screw on an ISO 7093-1 M6 washer, and the Sony ZV-E10 envelope (115.2 × 64.2 × 44.8, Sony help guide) with an estimated SELP1650 lens.

### 2.3 What bolts to what

```
column
  └─ liner_front / liner_back                (TPU, friction)
       └─ collar_back ══4 seam stops══ collar_front     (M5 shoulder, 6 C springs, M5 insert)
             └─ pan pivot ── arm_base                   (knob under the collar, M6 insert in the pedestal)
                               └─ J1 rosette ── arm_link            (knob on the pedestal's −X flat)
                                                 └─ J2 rosette ── arm_tip        (knob on arm_tip)
                                                                   └─ J3 ring ── arm_head   (knob on the head)
                                                                                  └─ 1/4"-20 ── ZV-E10
```

### 2.4 Load path, camera to column

1. Camera weight through the 1/4"-20 screw and its 18 mm washer into the head platform.
2. The head turns on J3 only by overcoming ring friction. The J3 stop presses the ring.
3. Links carry bending along their layers to J2 and J1; the rosettes lock by their stop force.
4. The pedestal carries the moment through its neck (across layers, the weakest section, checked) into the pivot face. The pivot stop holds it down: no lift-off at 2×.
5. The collar's back carries it into the ring round the column. Four seam stops press the liners onto the column; friction holds the device up.

Design load: 1.0 kg payload (estimated; the ZV-E10 is 343 g with battery). Worst pose: both links level. Lever from J1 to the camera's mass: 2 × 70 plus the head offset, derived in code. Peak checks use 2×.

### 2.5 Assembly order (the checks use these steps)

1. Press the inserts with a soldering iron: four M5 into `collar_front`'s seam faces; M6 into the `arm_base` pivot face, both ends of `arm_link`, end B of `arm_tip`. Melt each M6 shoulder screw's knurled head into a knob the same way.
2. Snap the liners into both shells.
3. Close the shells round the column. At each seam stop, from the back: seat washer, six C springs (alternating), M5 shoulder screw. Turn with a 3 mm key until it stops. Press a TPU cap into each hole.
4. Set the pedestal on the collar's back. From below: seat washer, five B springs, head washer, then the knob with its screw. Turn until it stops.
5. Build J1, J2 and J3 the same way, with the TPU ring in J3. Knobs on −X (J1, on the pedestal), −X (J2, on arm_tip), +X (J3, on the head).
6. Camera on the platform, then the 1/4"-20 screw and its washer from below.

## 3. Fastener schedule: spring stops

### 3.1 Bought dimensions

Fit rule: a pocket is the max part size plus the fit shown. A press or melt fit is under size on purpose.

| Part | Published size | Source | Hole / pocket |
| --- | --- | --- | --- |
| ISO 7379 M6 shoulder screw | shoulder 8 f9, thread M6 × 11.25 (b max), head 13 × 5.5, key 4; l1 0/+0.25 | fasten.it ISO 7379; Elesa, Misumi sheets | shoulder bore 8.4; head melted into a 12.4 knob pocket |
| ISO 7379 M5 shoulder screw | shoulder 6.5, thread M5 × 9.75, head 10 × 4.5, key 3 | same | shoulder bore 6.9; 16.4 counterbore |
| DIN 2093 B 16 × 8.2 × 0.6 | h0 0.45; 172 / 304 / 410 / 503 N at 0.25 / 0.5 / 0.75 / 1.0 h0 | fasten.it DIN 2093 (now DIN EN 16983) | in a 16.4 spot face |
| DIN 2093 C 14 × 7.2 × 0.35 | h0 0.45; 68 / 106 / 123 / 131 N | same | in the 16.4 counterbore |
| ISO 7089 M8 washer | 8.4 / 16 / 1.6 | Westfield | seat and head washers |
| ruthex RX-M6x12.7 | knurl 8.7, hole 8.0, L 12.7, wall min 3.3 | 3djake ruthex table (columns unlabeled: confirm on the coupon) | 8.0 × 13.4 blind |
| ruthex RX-M5x9.5 | knurl 7.1, hole 6.4, L 9.5, wall min 2.6 | 3djake ruthex table; RS PRO M5 sheet agrees | 6.4 × 10.5 blind, round |
| 1/4"-20 SHCS ASME B18.3 | head 9.53 × 6.35 | Westfield | 6.6 hole, 18.4 spot face |
| ISO 7093-1 M6 washer | 6.4 / 18 / 1.6 | fasten.it; Keller & Kalmbach | under the camera screw |
| ZV-E10 tripod socket | thread depth < 5.5 | Sony help guide | screw engages 4.85–5.35 |

Every value is in `params.py` with its source. Values without a source carry `# estimated:`.

### 3.2 The stop rule

```
l1        shoulder length, a stock length
above     = seat washer + n · (spring free height − design deflection) + head washer
clamped   = l1 − above            design distance from the PETG seat to the insert top
parts are sized so the measured seat-to-insert distance is `clamped` (a row, by rays)
deflection(clamped, l1) = free − (l1 − clamped − washers) / n
band      least force: l1 + 0.25 and the thinnest stack; most force: l1 and the thickest
rows      spring deflection 0.15–0.95 h0 over the band; least force ≥ the need;
          most force: insert pull-out ≤ 1.10 MPa, seat washer bearing ≤ 3.85 MPa (measured seat);
          shoulder end lands on brass (no PETG in its footprint); thread in the insert ≥ 1.5 d;
          hole past the thread tip ≥ 0.3 (measured); thread left when backed out to re-index ≥ 0.8 d
```

Printed stack tolerance ±0.15 mm (estimated), ±0.25 for J3 (ring ±0.1 too).

### 3.3 Schedule (from the build)

| Stop | Qty | Screw | Springs | Into | Force band (least .. most) | Holds |
| --- | --- | --- | --- | --- | --- | --- |
| Seam | 4 | ISO 7379 M5 6.5 × 35 | 6 × C14 (0.75 h0 at contact) | M5 insert, front shell | 71 .. 128 N over column ±0.3, rib crush 0.6, print ±0.15 | 2× tilting couple needs 33 N each |
| Pan pivot | 1 | ISO 7379 M6 8 × 40 | 5 × B16 (0.6 h0) | M6 insert, pedestal | 261 .. 373 N | no lift-off at 2×: 188 N |
| J1, J2 | 2 | ISO 7379 M6 8 × 20 | 5 × B16 (0.6 h0) | M6 insert, arm_link | 263 .. 375 N | rosette at 2×: 192 N (J1), 104 N (J2) |
| J3 | 1 | ISO 7379 M6 8 × 20 | 5 × B16 (0.5 h0) | M6 insert, arm_tip end B | 187 .. 351 N | ring friction at 2×, tilted 15°: 146 N |
| Camera | 1 | 1/4"-20 × 1/2" SHCS + ISO 7093 washer | — | camera socket, 4.85–5.35 engaged | 787 N at 1 Nm by hand (estimated K 0.2) | washer bearing 3.61 MPa (limit 3.85) |

Sustained limits: bearing 0.1 × UTS along layers = 3.85 MPa; insert pull-out shear 0.1 × 19.0 / √3 = 1.10 MPa on the knurl cylinder. The tightest row is the insert at the most stop force: 1.08 MPa.

Holding needs: rosette F ≥ (M / r_m) · tan(45° − φ), r_m 10.35, μ 0.2 (estimated). J3 friction F ≥ M / (μ r_m), μ 0.5 TPU on PETG (estimated), r_m 8.7, M = 2 × W × (lever + rise · sin 15°). Pivot F ≥ M / (foot radius − lead).

## 4. Kinematics and interfaces

- **Pan**: the pedestal turns about Z on the collar's back. Loosen the knob under the collar about a turn, pan, turn until it stops. Drag with the stop set: a row reports it. Clear range in the hero pose: −30° to 30° and 150° to 195° (the arm reaches over the column, so it meets it beyond that); a raised arm pans further.
- **J1, J2**: 24 tooth Hirth rosettes, 90° included, r 7.7–13, flank fit 0.10, 15° index. Lift 2.2 mm to re-index (the knob backs out past the springs, a row checks the thread left).
- **J3**: flat PETG faces with a 1 mm TPU 95A ring. Smooth tilt. Holds the 2× load within ±15° of level; past that, move J2.
- **Horizon**: every tilt axis is square to the pan axis (a row), so the horizon stays level at any pan with a vertical column.
- **Camera**: one 1/4"-20 screw from below, 21 mm past the J3 face toward −X, clear of arm_tip, so the key reaches it with the arm built. The battery door stays free.
- Deferred: an Arca-Swiss dovetail (no published height, needs a clamp), USB-C cable routing (the dummy-battery cable leaves the open door; no channels in the links, which would weaken them).

## 5. Strength and print plan

Allowed stress: 0.1 × UTS sustained, 0.25 × UTS peak (2×), in the stress direction (PETG along layers 38.5 MPa, across 19.0). Section rows use the payload moment as a vector: biaxial bending and twist, combined as von Mises.

| Part | Print pose (bed normal, use frame) | Main load | Stress vs layers | Perimeters / top-bottom / infill | Notes |
| --- | --- | --- | --- | --- | --- |
| collar_front, collar_back | top face down (0,0,+1) | Clamp hoop, back moment | Along | 5 / 5 / 40 % gyroid | Shoulder bores horizontal: teardrop roofs; the 16.4 seam counterbores a teardrop cut flat at the circle (a 6.8 bridge). M5 insert holes stay round (6.4 prints round). |
| liner_front/back | axis vertical (0,0,−1) | Compression | n/a | all perimeters, 25 mm/s | |
| arm_base | foot down (0,0,−1) | Moment at the neck | **Across**, checked at 1.9 MPa: 1.05 sustained | 6 / 6 / 60 % | Ball starts at latitude −40° (50° from down-facing). J1 bores horizontal: teardrops. |
| arm_link, arm_tip | on the +X face (+1,0,0) | Waist bending | Along | 6 / 5 / 50 % | Inserts and bores vertical. The spot face is a 4 mm annular bridge. |
| arm_head | on its +Y face (0,+1,0) | J3 block, platform cantilever | Along | 6 / 6 / 60 % | Bores teardrop toward −Y. |
| knob | rim down (0,0,−1) | Hand torque | Along | 4 / 5 / 40 % | 45° ring over the head pocket: the flat ceiling bridges 9.6. |
| j3_ring | flat | Compression | n/a | solid | TPU 95A. |

Wall minimum: PETG 1.5, TPU 1.2, the maker's wall round an insert knurl (M5 2.6, M6 3.3), measured with solid probes. Horizontal holes 6.5 mm and over get a 45° teardrop.

Collar sizing (derived in `params.py`):

```
bore offset    = fit 0.25 + rib_h 0.8 + liner_t 2.0 = 3.05      (bore 46.3 × 61.6, r 15.05)
seam_gap       = 2·fit + 2·rib_h − 0.1 = 2.0 open; 1.54 at liner contact (the hero)
seam x         = bore half-width + max(1.5 + 6.9/2, 2.6 + 7.1/2) = ±29.3
outer half-w   = seam x + 16.4/2 + 1.5, to 0.5 = 39.5 → 79 wide
plan           = front y −36, sides to y 36, half-ellipse back to y 74 (semi-axes 39.5 × 38)
pivot          = y 52, the pedestal foot r 19.7 clears the bore
```

## 6. Verification plan

Every check runs at build time (`python -m holder` exits 1 before export on any FAIL) and again in pytest. Each new check has a case that must fail.

### 6.1 Stops
§3.2 rows for each placed stop, the stack measured by rays on the placed solids. Negative tests: a stack 2 mm too thick flattens the springs; PETG over an insert fails "shoulder on brass"; a through insert hole fails the blind-depth row; a thinned wall fails the maker's wall.

### 6.2 Camera
Grip by rays; engagement on the short screw, tip depth on the long screw; washer bearing at 1 Nm by hand; the door and its 30 mm swing room clear of every part (a plate under it is caught, test).

### 6.3 Assembly clash at build time
`assembly(spec)` holds every printed and bought solid. Pairs whose boxes touch share ≤ 1e-6 mm³, except a screw in its own insert, an insert in its host, and a screw head in its knob. Liner rib tips 0.02 ± 0.01 off the column at contact; seam gap at contact ≥ 1.49; rosettes 0 clash and a 0.05–0.15 flank gap; the ring 0–0.03 off both J3 faces. Each joint and the pan swept through 360° in 15° steps with the hero and folded poses inside the clear range. A key rod (key + 1 mm, 120 mm) from each keyed screw misses every part present at its step; 10 mm finger room round the outer 60 % of every knob (round the exposed part of the recessed pivot knob).

### 6.4 Coupons cut from the real parts
- `coupon_collar_front/back`, `coupon_liner_front/back`: `part & slab(z 4–19.5)` with the lower seam stops (z 10), checked against the measured column: bore 3.05 ± 0.01 off it, rib tips 0.25 ± 0.02.
- `coupon_rosette_base`: the pedestal's J1 flat with its teeth and shoulder bore (x 0–7.5). `coupon_rosette_link`: arm_link end A with its teeth and M6 insert. Build J1 on them with a knob and springs: it tests the teeth, the insert, the stop and the lift.

### 6.5 Print, form and strength per part
- Overhang on a UV grid in print pose: no down-facing area steeper than 45° (0.5° tolerance) unless it is a flat ceiling within 5 mm of its outer wire.
- Surface continuity on the finished part (§0 row 3). Plan and profile curvature jumps ≤ 1e-3 /mm. Smallest convex plan radius over the roll.
- Section stress at the collar back, pedestal neck, link waist, tip waist, head block and platform root.

### 6.6 Install limit
The build measures the lowest point of anything above the collar inside the column footprint grown by 3 mm, against `column_top_z` (100 mm modeled). README states the limit. A test shows a 400 mm column hits the arm.

### 6.7 BOM from code
`python -m holder --bom` prints it; README holds it between markers; pytest compares.

### 6.8 Renders
`tools/render/meshes.py` tessellates every solid at 0.02 mm with the CAD surface's own normals; `tools/render/scene.py` (Blender 5.2) renders: hero 3/4, side, front, top plan, J2 detail, collar, a J2/J3 cutaway (`section`), zebra on the shells, the head and the pedestal, and every printed part in print pose (`parts`). The pictures show a design not yet printed. `exports/report.json` holds every check row and a hash of `src/`.

## 7. Scope

Done in v0.6.0: the form (§0), spring stops on every joint, pan and fine tilt, the integrated camera head, finished-part continuity, door clearance, the cutaway render. Removed: the MF-40 mount face, port nuts, M3 fasteners and the accessory plates (§9). Do not push. Never touch `main`.

## 8. Open items (estimated until measured)

| Item | Value used | How to close |
| --- | --- | --- |
| Column corner radius | r 12 | Collar coupon |
| Column depth band | ±0.3 | Calipers along the clamp height |
| ruthex table columns | knurl / hole read as D1 / D3 | ruthex datasheet; press an insert into the coupons |
| Shoulder head melted into PETG | holds hand torque | Rosette coupon: turn the knob to the stop at 2 Nm |
| Printed stack tolerance | ±0.15 (ring ±0.1) | Measure the coupons |
| Payload mass | 1.0 kg design | Weigh the camera and lens |
| TPU-on-aluminium friction | μ 0.5 | Coupon pull test |
| PETG on PETG; TPU ring on PETG | μ 0.2; μ 0.5 | J3 and rosette coupons with a luggage scale |
| ZV-E10 socket offset, door size | 13 / 2 mm; 36 × 24 at the grip end | Calipers |
| Camera mass forward of the body centre | 5 mm | Balance on a rod |
| Hand torque on the camera key | 1 Nm | It does not set any joint; only the camera washer row uses it |
| Column top above the collar | 100 modeled, 181 measured limit | Clamp below the limit |

Passing CAD checks is not a printed fit. The coupons and a 24 h payload hang test are the real proof.

## 9. As built (v0.6, and why it differs from v0.5)

| Item | v0.5 | v0.6 | Why |
| --- | --- | --- | --- |
| Collar form | deck over an inset ring, shadow groove | one barrel skin (2 mm parabola), plan of conics, straight split | Review: stacked slabs. |
| Seam heights | z 10, 30 | z 10, 28 | Two 16.4 counterbores with a flat-topped teardrop roof (6.8 bridge) fit the 40 mm height with 1.6 of PETG between them and 1.8 under the lower one; a full teardrop would run out of the bottom. |
| Counterbore mouths | open | flush TPU caps | The mouths cut the elliptic back as long scoops (first v0.6 render). |
| Seam fasteners | 4 × M3 × 30 into M3 inserts, 0.05 Nm (no driver sets it) | 4 spring stops, M5 shoulder on M5 inserts, from the back | Review: the torque spec could not be applied; 0.2 Nm by hand gave 333 N, 4.0 MPa insert shear. Now the insert sees 128 N at most, whatever the torque. |
| Arm base | foot slab + upright on an MF, 4 × M3 | turned pedestal on a pan pivot stop | Pan, one surface, no M3. |
| Joints | DIN 603 carriage bolt, nut in the knob, 0.48 Nm for 400 N | ISO 7379 shoulder stop, springs, insert; knob holds the head | 1 Nm by hand gave 833 N in v0.5. A carriage bolt cannot be a stop: its neck height runs 3.4–4.6 (DIN 603). |
| J3 | rosette | friction on a TPU ring | Fine tilt. |
| Links | one part twice | arm_link (two far ends) and arm_tip (near end A, friction end B) | The insert sits in the far part of every joint. |
| Head | knuckle + ogee loft into an MF | one skin, platform at the lens end | No MF, no M3; the door is free; G2 on the finished part. |
| Camera plate (`zve10`) | separate plate over the whole bottom | the head's platform, 31 wide | Review: it blocked the battery door. |
| MF-40, port nuts, `cam_plate`, `phone`, `gopro`, `post` | present | removed | Their M3 feet had the same torque problem, the nuts rattled without a stud, and four were drilled plates. A 1/4"-20 accessory can use the camera screw on the head. |
| Continuity rows | per skin, before fusing | on the finished part, every surface join | Review. |
| Insert hole depth | could read infinite and pass | must be blind, a range | Review. |
| Renders | 8 shots | 11, with a cutaway and a print layout | Show the stops and every part. |
