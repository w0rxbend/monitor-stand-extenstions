# Design brief: the finished holder

Status: implemented in v0.5.0 on `feat/apple-form`. Section 9 lists where the code differs from earlier briefs and why. It replaces the band-and-block design in PLAN.md.
PLAN.md stays as history. Where they differ, this file wins.

Next action for the engineer: print the coupons (section 6.4), press an insert into `coupon_mf`, and close the open items in section 8.

## 0. What is wrong now, and what this brief fixes

| # | Defect (verified) | Fix in this brief |
| --- | --- | --- |
| 1 | `hex_pocket` puts corners on ±X. Slots along Y (block top) and Z (plate) are only AF wide, so the nut hits PETG on its way in. | `captive_nut(travel=...)` turns a corner onto the travel axis. Slot width = AF. A nut-travel check sweeps the real nut (§3.4, §6.1). |
| 2 | Coupon reuses `clamp_halves` without ribs: bore is 1.25 mm clear per side. Exported coupons are stale. | Coupons are slices of the real parts, made by `part & slab` (§6.4). |
| 3 | Boss M3×25 tip runs into 4.2 mm of solid PETG. | Every tip bore depth comes from the stack, and bought screws are solids in the clash check (§3.2, §6.2). |
| 4 | Clash test is pytest-only and allows < 1.0 mm³. | Clash matrix runs in `python -m holder` at 1e-6 mm³ (§6.3). |
| 5 | Nothing proves a nut can reach its pocket. | Nut-travel check on every captive nut (§6.1). |
| 6 | Form: rounded boxes stacked. Flat renders. | One plan language, G2 curves, conic profiles, one edge vocabulary, Cycles renders (§1, §6.7). |
| 7 | 1/4" nut pocket 11.4 × 5.8 leaves 0.06 mm over a max nut (ASME B18.2.2 t max 5.74). | Pocket 11.45 × 6.05 (§3.1). |
| 8 | TPU band carries the clamp load. TPU 95A compression set is 30–38 % at 23 °C (Essentium, BASF TDS), so it loses clamp force. | PETG collar shell carries the load. TPU is a 2 mm grip liner only (§2). |

Axes stay as now: Z up, origin on the column axis at the collar bottom, +Y is the side the arm leaves from.
Assumption: +Y points away from the monitor, so the seam screw heads (−Y face) face the monitor and stay out of sight. Mirror in Y if the user wants it the other way.

## 1. Product concept and form language

### 1.1 The idea

One device, not a kit of blocks. A PETG **collar** grips the column. Its top is one flat surface that runs from the ring into a **mount face** on a cantilevered pad. A three-joint **arm** stands on that face. The arm ends in a **head** with the same mount face. Every accessory has the same **foot**. Foot and face have the same 40 × 40 soft-square plan, so stacked parts line up edge to edge.

The look follows machined camera hardware (Falcam, SmallRig, Peak Design) and Apple's soft-solid products: flat datum faces, every outer edge a curvature-continuous (G2) roll, a shadow line where two volumes meet, no visible nuts, few visible heads, a turned knob with an elliptic crown at each joint. This is our reading of those products, not a sourced spec.

### 1.2 Named curves and surfaces (build123d 0.11)

All signatures below were checked against the installed 0.11.1.

| Curve or surface | Where | How in 0.11 | Why |
| --- | --- | --- | --- |
| **Soft outline**: lines + quintic Bezier corners, 3 control points on each side | Every plan: collar deck and ring, mount face, foot, plates, links, walls | `form.soft_outline` (ported from `~/Projects/macropad/cad/macropad_case/solids.py`, handles 0.6/0.2 of setback) | Curvature is zero where the corner meets a side: G2. Smallest corner radius is 0.47 × setback, which bounds the roll a skin can carry. |
| **G2 blend** between two given curves | Link waist into its end circles, knuckle circle into its sides, knob wall into its crown, skin rolls into a parabolic wall | `form.g2_blend`: quintic P0..P5, h = 5·a²·κ/4 at both ends | Matches position, tangent and curvature. Works for line (κ 0), circle (1/R), ellipse and parabola. |
| **G2 skin**: one edge profile swept round a G2 outline, closed by planar caps and sewn | Every outer body | `form.g2_skin(outline, height, bottom, top, bulge)`: `sweep` (corrected Frenet), `BRepBuilderAPI_Sewing`, `BRepLib.OrientClosedSolid` | No boolean and no OCC fillet touches the outer surface, so every join is G2 by construction. Measured on the real faces (§6.5). |
| **Edge rules**: `Roll(setback)` and `BedLead(leg, rise)` | Top and bottom of every skin | quintic corner; 45° line then a G2 blend | A roll on a bed edge would overhang. The bed lead prints at 45° and turns G2 into the wall. |
| **Parabolic wall** (a conic) | Camera plates, 0.25 bulge | quadratic Bezier wall, rolls blended G2 onto it | The plate walls are not straight extrusions. The collar walls stay straight: its seam faces are walls of the same skin, and a bulge there closed the seam at 1.2 instead of 2.0. |
| **Knuckle outline**: circle, G2 blends, straight sides, G2 buried corners | `arm_base` upright, `arm_head` knuckle | `arm.knuckle_outline` | Replaces the teardrop knuckle of v0.4 (§9). Sides are vertical in print, so nothing overhangs. |
| **Quintic ogee loft** | `arm_head` from knuckle to MF, `post` column into its MF | `loft` of soft outlines 1 mm apart; width law = smoothstep, then the inset of a G2 bed lead or roll | Reads turned. Dense smooth sections keep the B-spline from overshooting. |
| **Elliptic arcs** (exact conics in STEP) | Knob crown (a = 11, b = 3), carriage-bolt dome | `EllipticalCenterArc` | The second-order curves the user asked for. |
| **Hirth rosette**, 24 teeth, 90° included | Every arm joint | One tooth = `loft([V-triangle face at r_out, Vertex(axis point)])`, trimmed r 5–13 | Teeth converge on the axis, so all faces are planes. |

Rules that keep STEP and 3MF valid:

- Never call `offset()` on a sketch that holds a Bezier. Rebuild an outline from its own parameters at the new size. Offsetting the column profile (circular arcs) is fine.
- No OCC fillets or chamfers on outer edges. The outer surface comes from the skin; holes, pockets and rosettes are cut after.
- A roll must be smaller than the outline's smallest corner radius, or the sweep folds and `g2_skin` raises.
- After every part: `is_valid`, one solid, STEP round-trip volume within 0.5 %, 3MF volume within 1 %.

### 1.3 Edge vocabulary (the same on every printed part)

| Edge | Treatment |
| --- | --- |
| Outer edge off the bed | G2 roll: collar 4.0 (deck underside), 3.5 (ring bottom); plates 3.0; foot top 2.5; knuckle and link faces 1.2 (the rosette leaves 1.5 mm of flat face) |
| Outer edge on the print bed | 45° lead (collar 1.0, plates and feet 0.6, link 0.6), then a G2 blend 1.5–3.0 up the wall. The only crease on the outer surface. |
| Where two volumes meet | A shadow line: the collar deck overhangs its ring by 1.0; the head ogee leaves a 0.6 shoulder over its knuckle; uprights and walls stand in their foot |
| Hole and slot mouths | 0.4 × 45° lead-in |
| Seam | Each shell is its own skin, so the seam faces carry the same rolls; the seam corners in plan are G2 (setback 9.5) |

### 1.4 How the parts read as one product

- The collar deck top and its mount face are **one flat plane**. The plan runs ring → G2 taper → pad with no step.
- The deck (z 16–40) sits on a ring set 1 mm inside it. The shadow groove runs all round, and the pad cantilevers off the ring on the deck alone.
- No ears. The seam screws run inside the wall.
- Foot plan = mount-face plan. Arm base on the collar, and a plate on the head, sit edge-aligned.
- Base and head knuckles share one outline. Links share one dog-bone plan. Every knob is the same part.
- The collar port nut slides in from the bore and hides behind the liner.

## 2. Assembly architecture

### 2.1 Printed parts

| Part | Material | What it is |
| --- | --- | --- |
| `collar_front` | PETG | Front shell: deck and inset ring skins. Seam screw counterbores (teardrop roof in print). |
| `collar_back` | PETG | Back shell: deck with taper and MF pad, inset ring. 4 seam inserts in its seam face, 4 MF inserts, the 1/4" port nut channel from the bore. |
| `liner_front`, `liner_back` | TPU 95A | 2.0 mm grip liners with 45° V ribs. They snap behind the shell lips and cover the port nut's slot mouth. |
| `arm_base` | PETG | Foot skin (4 × M3 to the MF) and the knuckle upright with joint J1. |
| `arm_link` × 2 | PETG | Same part twice. Dog-bone skin, 70 mm pitch, 10 mm thick, a rosette at each end. The second one is turned 180° about its long axis. |
| `arm_head` | PETG | Knuckle J3 and an ogee loft that rises into an MF. |
| `knob` × 3 | PETG | Turned knob with an elliptic crown, holding an M6 nut. |
| `zve10` | PETG | Hero payload plate. Foot to the head MF, 24 mm camera slot. |
| `cam_plate`, `phone`, `gopro`, `post` | PETG | Accessories on the shared foot and MF. |
| `coupon_*` | as parent | Slices of the real parts (§6.4). |

### 2.2 Bought parts (modeled as solids in the assembly)

M3 ISO 4762 screws with ISO 7089 washers into ruthex RX-M3x5.7 heat-set inserts. DIN 603 M6 carriage bolts, ISO 7093-1 M6 washers, ISO 4032 M6 nuts. 1/4"-20 hex nuts (ASME B18.2.2) in the MF ports. One 1/4"-20 socket cap camera screw (ASME B18.3) on an ISO 7093-1 washer. A Sony ZV-E10 envelope (115.2 × 64.2 × 44.8 mm, Sony help guide). The bought GoPro buckle, phone clamp and ball head are not modeled.

### 2.3 What bolts to what

```
column
  └─ liner_front / liner_back          (TPU, friction)
       └─ collar_front ══4× M3 seam into inserts══ collar_back
                                                     └─ MF ── 4× M3 into inserts ── arm_base
                                                                                    └─ J1 ── arm_link
                                                                                               └─ J2 ── arm_link (flipped)
                                                                                                          └─ J3 ── arm_head
                                                                                                                    └─ MF ── 4× M3 ── zve10
                                                                                                                                       └─ 1/4"-20 ── ZV-E10
```

### 2.4 Load path, camera to column

1. Camera weight goes through the 1/4"-20 camera screw and its 18 mm washer into the `zve10` plate.
2. The plate bears flat on the head MF. Four M3 screws pull it down into inserts.
3. The head transfers moment through J3. The rosette teeth lock it. The carriage bolt preload keeps the teeth engaged.
4. Links carry bending along their layers to J2 and J1.
5. `arm_base` bears on the collar MF with 4 × M3.
6. The pad is a cantilever on the deck. It prints face down, so bending runs along the layers into the ring.
7. The ring presses the TPU liner onto the column. Four seam screws make the normal force. Friction holds the device up.

Design load: 1.0 kg payload (estimated). Worst lever from J1 to the camera: both links level (140) plus the deck offset (48.5) = 188.5 mm, derived in code. Sustained moment at J1 1.85 Nm. Peak checks use 2×.

### 2.5 Assembly order (the checks use these steps)

1. Press the 8 inserts into `collar_back` (4 in the seam faces, 4 in the MF) with a soldering iron. Hold `collar_back` MF face down, slide the 1/4" nut in from the bore, turn the shell over: the nut drops 1.5 mm into its pocket and cannot slide back out.
2. Snap the liners into both shells. The back liner covers the nut's slot mouth.
3. Put the halves round the column. Fit 4 × M3 × 30 seam screws with washers from the −Y face. Tighten crosswise to 0.05 Nm (estimated; confirm on the coupon). The seam must stay open.
4. Fix `arm_base` to the MF with 4 × M3 × 10 and washers from above, before J1 is built, so the key can reach.
5. Press the 4 head inserts, drop the head port nut in the same way (head MF face down, then upright). Build J1, J2 and J3: carriage bolt through (square neck in its square hole), washer, knob.
6. Fix `zve10` to the head MF with 4 × M3 × 10 and washers. Then the camera, 1/4" screw and washer from below.

## 3. Fastener schedule

### 3.1 Bought dimensions and holes

Fit rule: a pocket is the max part size plus the fit shown. A pocket is a press or slide fit, never the PETG clearance 0.30.

| Part | Published size (max) | Source | Pocket / hole | Fit |
| --- | --- | --- | --- | --- |
| M3 heat-set insert ruthex RX-M3x5.7 | knurl 4.6, hole 4.0, length 5.7, min wall 1.6 | 3djake ruthex table (columns unlabeled: **confirm on the coupon**) | Ø4.0 × 6.4 (0.2 sink + 5.7 + 0.5), then the tip room at Ø3.4 | maker's hole |
| M6 nut ISO 4032 | s 9.78–10.00, m 4.90–5.20 | Wermac; HB&H | 10.20 AF × 5.50 in the knob | +0.20 / +0.30 |
| 1/4"-20 nut ASME B18.2.2 | AF 10.87–11.13, t 5.38–5.74 | amesweb table | 11.45 AF × (6.05 + 1.5 drop) | +0.32 / +0.31 |
| M3 ISO 4762 head | dk 5.32–5.68, k 3.00, key 2.5 | Westfield | With washer: Ø7.4 × 3.8 | +0.4 dia, +0.3 deep |
| M3 washer ISO 7089 | 3.2 / 7.0 / 0.5 | Westfield | Inside the Ø7.4 counterbore | |
| M6 carriage bolt DIN 603 | dk 15.45–16.55, k 3.88, neck 6.48 square × 4.6 | Fabory DIN 603 page; Aspen Fasteners | Square hole 6.78 × 4.9, a corner up | +0.3 |
| M6 washer ISO 7093-1 | 6.4 / 18 / 1.6 | fasten.it ISO 7093-1 table; Keller & Kalmbach | Sits on the knuckle face | |
| Clearance ISO 273 medium | M3 3.4, M6 6.6 | iso.org; chart | 3.4, 6.6; 1/4" 6.6 | |
| 1/4"-20 SHCS ASME B18.3 head | Ø9.53 × 6.35 | Westfield | On an ISO 7093 washer in an 18.4 × 8.3 slot from below | |
| ZV-E10 tripod screw | thread depth < 5.5 | Sony help guide | screw engages 4.0–5.5 | |

Every value above is in `params.py` with its source in a comment. Values not from a source keep `# estimated:`.

### 3.2 Stack rule

```
grip      = under the head to the load face (washer included), measured by rays on the solids
L         = shortest stock length >= grip_open + engage_need + tol        tol: M3 0.3, M6 0.42, 1/4" 0.25 (estimated)
engage    = into an insert: 0.2 sink + 1.5 d (4.5); into a nut: m_max + 2 pitches
tip room  = a design parameter (seam_tip_room, knob_tip_room, the insert hole, the camera socket),
            never derived from L, so a longer screw fails
rows      : engagement (short case L - tol, open state) >= engage
            tip clear  (long case L + tol, closed state) <= tip room - 0.5
            head below the surface >= 0.3 (rim ray runs away from anything standing on the foot)
            the measured grip equals the design grip within 0.05
```

`state` is `open` (as modeled; the seam is 2.0 mm open, the rosette is lifted 2.2 to index) or `closed`.

### 3.3 Schedule (from the code; the README BOM is generated)

| Joint | Qty | Screw | Grip | Into | Preload (torque, K 0.2 estimated) | Bearing |
| --- | --- | --- | --- | --- | --- | --- |
| Collar seam | 4 | ISO 4762 M3 × 30 + ISO 7089 washer | 23.7 open, 21.7 closed | insert in the back seam face, tip room 10 | 80 N (0.05 Nm) | washer 29.4 mm² → 2.7 MPa; insert shear 0.97 MPa |
| Foot to MF | 4 per foot | ISO 4762 M3 × 10 + ISO 7089 washer | 4.7 (4.2 PETG + washer) | MF insert | 80 N (0.05 Nm) | washer 2.7 MPa; insert 0.97 MPa |
| Arm joints J1–J3 | 3 | DIN 603 M6 × 35 | 21.7 closed, 23.9 open | M6 nut in the knob, ISO 7093 washer | 400 N (0.48 Nm by hand) | dome 141 mm² → 2.8 MPa; washer 218 mm² → 1.8 MPa |
| Camera | 1 | 1/4"-20 × 1/2" SHCS + ISO 7093 washer | 7.6 | camera socket, 4.85–5.35 engaged (limit < 5.5) | 394 N (0.5 Nm) | washer on the slot ceiling |
| MF 1/4" port | 1 per MF | a bought stud for a ball head (not in the hero) | | 1/4" nut, roof 6.1 | 236 N (0.3 Nm) | nut face 67.9 mm² → 3.5 MPa |

Sustained bearing limit: 0.1 × UTS = 3.85 MPa (compression closes the layers, so the along-layer value). Insert pull-out shears the layer planes: 0.1 × 19.0 / √3 = 1.10 MPa on the knurl cylinder (π · 4.6 · 5.7 = 82 mm²). That limit is what sets the M3 preload to 80 N. The 80 N is still over the 2× foot bolt pull (62 N) and gives the seam 2.3× the 2× tilting couple.

Captive M3 nuts are gone. Their bearing face (dw 4.6 to the 3.4 hole, 7.5 mm²) could not carry more than 29 N at 3.85 MPa.

### 3.4 Port nut rule

`CaptiveNut(..., drop=1.5)` returns `(pocket, channel)`.

- One hex corner points along `travel`, so channel width = pocket AF.
- The pocket is 1.5 mm deeper than the channel, on the far side from the face. With the part face down the nut slides in on the seat side; turned upright it drops into the pocket and the pocket wall stops it sliding back. The check: the max nut at rest on the pocket floor moves at most 0.5 mm toward the mouth before it meets PETG.
- Channel ceilings wider than 10 mm in print pose get a 45° gable on the far side (collar, head), or 45° strips at the seat edges where the seat is the ceiling (post).

## 4. Interfaces and connectors

### 4.1 Mount face standard (MF-40)

| Item | Value |
| --- | --- |
| Plan | 40 × 40 soft square, setback 12 (foot uses the same plan) |
| Structural | 4 × M3 heat-set inserts on a 20 mm square, 0.2 below the face |
| Port | 1 × 1/4"-20 nut, center, roof 6.1, pocket 6.1–13.6 deep with the 1.5 drop. Insert to channel ≥ 1.6. |
| Min depth | 1/4" tip bore floor 22.5 + 1.5 = 24 mm under the face |
| Datum | The face is flat and is the print bed face where the part allows (collar_back, arm_head) |
| Rotation | Feet index every 90° on the M3 square |

### 4.2 Arm joints: Hirth rosette lap joints

- Lap joint, 24 teeth, 90° included angle, annulus r 5.0–13.0, tips and roots truncated 0.15, flank fit 0.10. Index step 15°. Lift to re-index 2.2 mm.
- A DIN 603 carriage bolt's dome head bears on one knuckle face; its square neck in a square hole (a corner up, so 45° roofs in any print pose) stops it turning. The knob on the other face holds an M6 nut on an ISO 7093 washer, so PETG never takes a small steel face under preload.
- Holding: the teeth stay seated while F ≥ (M / r_m) · tan(45° − φ), r_m 9, μ 0.2 (estimated). At 2× load J1 needs 274 N; the preload is 400 N.
- Knob: Ø28, 18.5 tall, wall into an elliptic crown (a 11, b 3) through a G2 blend, 24 grip flutes. The nut sits in a 10.2 AF pocket open at the bottom, the bolt tip in a 16 mm blind bore.
- Chain: J1 rosette on the base +X face. link 1 on +X. link 2 flipped, back on −X. head on +X. Knobs alternate +X, −X, +X.

### 4.3 Deferred, with reasons

- Arca-Swiss dovetail: no published height, and it needs a clamp mechanism.
- USB-C cable routing: no channels in the links; they would weaken the bending section.

## 5. Strength and print plan

Allowed sustained stress: 0.1 × UTS in the stress direction (PETG along layers 3.85 MPa, across 1.9). Peak (2×): 0.25 × UTS. Section rows use the payload moment as a vector in the worst pose: bending about both section axes (with the product of inertia) and twist (T / 0.208 b t², the square-bar factor), combined as von Mises.

| Part | Print pose (bed normal, use frame) | Main load | Stress vs layers | Perimeters / top-bottom / infill | Notes |
| --- | --- | --- | --- | --- | --- |
| collar_front | top face down (0,0,+1) | Seam hoop tension | Along | 5 / 5 / 40 % gyroid | Counterbores horizontal: teardrop roof. Bed lead on the top face. |
| collar_back | top face down (0,0,+1) | Pad cantilever + hoop | Along (spine root row) | 5 / 5 / 50 % gyroid | MF on the bed. The pad underside and the shadow groove face up. |
| liner_front/back | axis vertical (0,0,−1) | Compression | n/a | all perimeters (2 mm), 25 mm/s | 45° V ribs. |
| arm_base | foot down (0,0,−1) | Moment at the upright root | **Across**, checked at 1.9 MPa | 6 / 6 / 60 % | Upright faces vertical; square hole a corner up. |
| arm_link | on its +X face (+1,0,0) | Waist bending | Along | 6 / 5 / 50 % | Bed lead on the +X face; the washer sits on that face, no recess (an 18 mm recess would be a 9 mm cantilever ceiling). |
| arm_head | MF down (0,0,+1) | J3 moment into the MF | Across at the neck | 6 / 6 / 60 % | The ogee narrows upward in print, so nothing overhangs. |
| knob | base down (0,0,−1) | Hand torque | Along | 4 / 5 / 40 % | |
| zve10, cam_plate | deck face down (0,0,+1) | Plate bending | Along | 4 / 5 / 40 % | The 18.4 washer slots open upward, so no wide ceiling. |
| post | foot down | Bending | Across | 6 / 6 / 60 % | The ogee keeps its corners under 45°. |
| phone, gopro | foot down | Light | — | 4 / 5 / 40 % | |

Wall minimum: PETG 1.5 (1.6 round an insert knurl, the maker's figure), TPU 1.2. Measured with solid probes. Horizontal holes ≥ 5 mm get a 45° teardrop.

Collar sizing (all derived in code from `params.py`):

```
bore offset    = fit 0.25 + rib_h 0.8 + liner_t 2.0 = 3.05      (bore 46.3 × 61.6, r 15.05)
seam_gap       = 2·fit + 2·rib_h − 0.1 = 2.0
seam x         = bore half-width + max(1.5 + 3.4/2, 1.6 + 4.6/2) = ±27.05
outer half-w   = max(seam x + 7.4/2 + 1.8 + 1.0,  seam x + 4.6/2 + 1.6 + 4.55) + 1.0 (ring inset) = 36.5 → 73 wide
                 (the 4.55 lets the ring's G2 seam corner dip only 0.16 mm where the insert wall meets the seam face)
outer half-d   = 35 → front 35; back pad to y 77
deck           = z 16–40 on the full plan; ring z 0–20 set 1.0 inside, its back at y 37 behind the bore
```

Grip math (estimated μ 0.5 TPU on aluminium): 4 × 80 N preload. The 2× tilting couple is 139 N against 320 N of seam clamp.

## 6. Verification plan

Every check below runs at build time (`python -m holder` exits 1 before export on any FAIL) and again in pytest. Each new check has a case that must fail.

### 6.1 Port nut travel and stop

For each port nut: the max nut (s_max, m_max, a real hex prism) swept from past the mouth to the seat meets ≤ 1e-6 mm³ of PETG; the channel is ≥ s_max + 0.15 wide (rays on real faces); the nut at rest on the pocket floor slides ≤ 0.5 mm toward the mouth before it meets PETG (bisection on the real part). Negative tests: the old corner-on-±X pocket fails travel; a pocket without the drop fails the stop.

### 6.2 Screw stacks and tip clearance

Measured by rays on the placed solids for every seam, foot, joint and camera screw (§3.2). Tip rooms are design parameters, so a longer screw fails. The foot rim ray runs outward from the foot centre and a row proves it lands on the foot top. A seam screw 5 mm too long meets the back shell (test).

### 6.3 Assembly clash at build time

`assembly(spec)` holds every printed and bought solid. Pairs whose boxes touch need ≤ 1e-6 mm³, except a screw with its own insert or nut, and an insert with its host (the knurl melts into the hole wall). Also: liner rib tips 0.25 ± 0.02 off the column; seam gap ≥ 1.95; rosette pairs 0 clash and a 0.05–0.15 flank gap; each joint swept 0–360° in 15° steps with the hero and folded poses inside the clear range; a key rod (key + 1 mm, 120 mm) from every head misses every part present at its step.

### 6.4 Coupons cut from the real parts

- `coupon_collar_front/back`, `coupon_liner_front/back`: `part & slab(z 4–15)`, with the z 10 seam screws. Checked against the measured column solid, not the collar: the shell bore sits 3.05 ± 0.01 off it, the liner rib tips 0.25 ± 0.02.
- `coupon_mf`: the collar MF region with the x +10 inserts and the 1/4" channel. It proves insert pressing, nut travel and the drop.
- `coupon_rosette_base`, `coupon_rosette_link`: knuckle discs with teeth. The base disc starts past the 1.2 face roll, so it prints on a flat face.

### 6.5 Print, form and strength checks per part

- Overhang: sample face normals on a UV grid in print pose. Fail any down-facing area steeper than 45° (0.5° tolerance) above z 0.05, unless it is a flat ceiling within 5 mm of its outer wire.
- Surface continuity on the real faces: for every edge of every skin, sample 3 points, read both faces' normals and their normal curvature across the edge (`BRepLProp_SLProps`). Normal step ≤ 0.5°, curvature step ≤ 2e-3 /mm. Only the bed-lead crease (and, for a loft, its fused cap) is exempt. A chamfered box fails.
- Plan and profile wires: curvature jump ≤ 1e-3 /mm at every join.
- Bearing: each washer, dome head and nut face is a 0.05 mm probe sunk into its real seat; the area is the PETG it meets. σ = preload / area ≤ 3.85 MPa. Insert pull-out shear ≤ 1.10 MPa.
- Preload needs: rosette preload for the 2× moment at J1–J3; the 2× foot bolt pull under the M3 preload; the 2× collar tilting couple under the seam clamp.
- Section stress (§5) at the collar spine root, arm_base upright root, both link waists, the head neck, the zve10 shoulder and the post column.

### 6.6 Install limit

The hero reaches back over the column. The build measures the lowest point of anything that swings (links, head, plate, camera) inside the column footprint grown by 5 mm, and checks the modeled column top (`column_top_z`, 100 mm above the collar bottom) is below it. README states the limit. A test shows a 260 mm column hits the arm.

### 6.7 BOM from code

`python -m holder --bom` prints it. README holds it between `<!-- bom:start -->` and `<!-- bom:end -->`. Pytest asserts the block equals the generated text.

### 6.8 Renders

- `tools/render/meshes.py` (uv) tessellates every solid at 0.02 mm in its assembly pose and writes OBJ with the CAD surface's own normal at every node (`BRepLib_ToolTriangulatedShape.ComputeNormals`). Blender would rebuild normals from OCCT's long sliver triangles and the zebra bands would jitter on a wall that is exact in the CAD (normal constant to 1e-17 along the collar front). `tools/render/scene.py` runs in Blender 5.2: AgX, three area lights, a shadow catcher, 96 samples with denoise.
- Views: hero 3/4 of the full assembly on the column, side, top (plan), J1 close-up, collar, zebra on the shells and on the head, accessories.
- The pictures are judged on the curves and the joins. They show a design not yet printed.
- `exports/report.json` holds every check row and a hash of `src/`.

## 7. Scope

Done in v0.4.0: steps 1–8 of the first brief (params, form, MF-40, collar, arm, assembly, accessories, renders, README). Done in v0.5.0: the G2 skin, inserts, carriage bolts, port nut drop, bearing and preload rows, biaxial stress, the install limit, coupon fit against the column. Not made: `cable_clip` (optional).

Do not push. Never touch `main`.

## 8. Open items (estimated until measured)

| Item | Value used | How to close |
| --- | --- | --- |
| Column corner radius | r 12 | Collar coupon |
| ruthex RX-M3x5.7 columns | knurl 4.6, hole 4.0 (table not labeled) | ruthex datasheet; the coupon_mf press |
| Nut factor K | 0.2 | Torque-tension test on the coupon |
| Payload mass | 1.0 kg design | Weigh the camera |
| M3 into an insert | 0.05 Nm for 80 N | Coupon; a 0.05 Nm driver is below most torque screwdrivers, so "snug with two fingers on the short leg" until checked |
| Knob torque | 0.48 Nm by hand for 400 N | Coupon with a luggage scale on the knob rim |
| TPU-on-aluminium friction | μ 0.5 | Coupon pull test |
| PETG on PETG (rosette) | μ 0.2 | Coupon |
| Screw length tolerances | ±0.3 M3, ±0.42 M6, ±0.25 1/4" | ISO 4759-1 table |
| DIN 603 M6 stock lengths | 16–60 | Supplier listing |
| ZV-E10 socket offset | 24 mm slot | Calipers |
| Phone ear width / hole | 16 / 4.7 | Calipers |
| Column top above the collar | 100 modeled, 181 measured limit | Clamp below the limit |

Passing CAD checks is not a printed fit. The coupons and a 24 h payload hang test are the real proof.

## 9. As built (where the code differs from earlier briefs, and why)

| Item | Brief | Code | Why |
| --- | --- | --- | --- |
| Outer edges | 1.0 × 45° chamfers, no fillets | G2 skins: rolls and bed leads, no chamfer or fillet on an outer edge | v0.4 review: chamfers left G0 creases and broken zebra stripes. A skin is G2 by construction and avoids the OCC fillet that broke a 3MF. |
| Knuckle | teardrop (circle + 45° tangents) | circle, G2 blends, vertical sides, buried G2 corners | The knuckle bore is the only horizontal hole; it keeps its own teardrop. Vertical sides print without support and blend G2. |
| Collar side view | elliptic scoop under the spine | deck over an inset ring; the pad cantilevers on the deck | The scoop started with a vertical tangent (a hook) and any side cut leaves a crease along the spine. The two-tier form keeps every outer edge G2 and puts the only junction in a shadow groove. |
| Spine root | none | pad leaves the ring at y 37; stress row at y 35 | |
| Plan | taper from y 16, neck y 40 | taper from y 22, neck y 44 | The back half starts at the seam (y 1), so the side needs room for the seam corner (9.5) and the taper corner (11). |
| Collar width | 66 | 73 | Inserts need 1.6 round a 4.6 knurl (seam x 27.05), the ring's G2 seam corner must not dip into that wall, and the ring sits 1.0 inside the deck. |
| M3 joints | captive ISO 4032 nuts | ruthex heat-set inserts | A nut's bearing face carries 29 N at 3.85 MPa; an insert carries 90 N in pull-out shear, and there is no channel for a nut to slide back along. |
| M3 preload | 0.4 Nm (667 N) | 0.05 Nm (80 N) | 667 N crushed 22.7 MPa into PETG. 80 N passes every bearing and insert row and still beats the 2× loads. |
| Joint bolt | ISO 4017 hex head in a hex pocket | DIN 603 carriage bolt, square neck | The hex head bore 24 MPa on 52 mm². The dome bears 2.8 MPa on 141 mm², and the square neck still stops it turning. |
| Knob washer | ISO 7089 in a Ø12.4 recess | ISO 7093-1 (18 mm) on the face | 15 MPa down to 1.8 MPa. A recess on the link's bed face would be an 18 mm ceiling. |
| Camera plates | foot down, 11.7 thick | deck face down, 14.3 thick | The 18 mm camera washer slot needs to open upward in print. |
| Port nut | slides in, no stop | 1.5 mm drop pocket | The nut can no longer slide back to the mouth. |
| Head | MF block cut by a G2 ogee | knuckle + dense quintic-ogee loft into the MF | The cut block read as a block under a plate. |
| Knob | flat dish, three V grooves | elliptic crown, G2 into the wall, 24 flutes | Turned look, one conic. |
| Knob revolve | one 360° revolve | two 180° halves fused | A 360° revolve of the crown and its G2 blend read back from STEP 12 % small and invalid; the halves round-trip to 1e-13. |
| Phone ear | captive M3 nut from the side | insert in the foot top | Same reason as the M3 joints. |
| Stress rows | bending about X | moment vector: biaxial bending + twist, von Mises | v0.4 review. |
| Design stacks | tip limit = tip + 0.5 (could not fail) | lengths only; tip rooms are parameters, measured by rays | v0.4 review. |
| Coupon bore check | coupon vs the part it was cut from | coupon vs the measured column | v0.4 review: the old check was circular. |
| Hero column | cut at z 100, unchecked | z 100, plus a measured install limit (181 mm) | The arm reaches over the column top; the limit is now a check and a README rule. |
| Rosette trim | annulus r 5..13 | r 5..13 at the pitch plane, 45° cones away from it | A cylindrical trim left small flat ceilings on the vertical rosettes. |
| Joint sweep | every pair | printed parts, knobs, camera, column; the swept joint's own rosette pair is left out | The teeth repeat every 15°, so each step meshes like the hero. |
| 3MF read-back | `Mesher().read()` | signed volume of the triangles in the file | `Mesher().read()` took over 20 minutes on one part. |
| Renders | `tools/render/scene.py` | `tools/render/meshes.py` (uv) writes OBJ with exact normals and runs `scene.py` (Blender 5.2) | build123d is not importable in Blender's Python; STL normals made the zebra jitter. |
| `cable_clip` | optional | not made | |
