# Design brief: the finished holder

Status: implemented in v0.4.0 on `feat/apple-form`, steps 1-8 of section 7. Section 9 lists where the code differs from this brief and why. It replaces the band-and-block design in PLAN.md.
PLAN.md stays as history. Where they differ, this file wins.

Next action for the engineer: print the coupons (section 6.4) and close the open items in section 8.

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

One device, not a kit of blocks. A PETG **collar** grips the column. Its top is one flat surface that runs from the ring into a **mount face** on a short spine. A three-joint **arm** stands on that face. The arm ends in a **head** with the same mount face. Every accessory has the same **foot**. Foot and face have the same 40 × 40 soft-square plan, so stacked parts line up edge to edge.

The look follows machined camera hardware (Falcam, SmallRig, Peak Design): flat datum faces, one crisp 45° chamfer size on every edge, no visible nuts, few visible heads, a turned knob at each joint. This is our reading of those products, not a sourced spec.

### 1.2 Named curves and how to build them in build123d 0.11

All signatures below were checked against the installed 0.11.1.

| Curve | Where | How in 0.11 | Why |
| --- | --- | --- | --- |
| **Soft square**: 4 lines + 4 quintic Bezier corners, 3 control points on each side | Collar outer plan, mount face, foot | `Bezier(*six_points)`. Port `_corner_curve` from `~/Projects/macropad/cad/macropad_case/solids.py` (setback, handles 0.6/0.2 of setback). | Curvature is zero where the corner meets the side. G2, no visible "line" where the arc starts. |
| **G2 blend** between two given curves | Collar-to-spine neck, link waist | Quintic Bezier P0..P5. Start: P1 = P0 + a·T0; P2 = P1 + b·T0 + h0·N0 with h0 = 5·a²·κ0 / 4. End is the same mirror. a = b = 0.2 × chord to start. Read T, N, κ from the real edge (`Edge.tangent_at`, OCP `BRepLProp_CLProps` for curvature). | Matches position, tangent and curvature at both ends. Same helper works for line (κ = 0) and circle (κ = 1/R). |
| **Elliptic quadrant** | Spine underside scoop (side view), knob top dish | `EllipticalCenterArc(center, x_radius, y_radius, start_angle, end_angle)` in a side-view sketch | The second-order curve the user asked for. Exact conic in STEP. |
| **Teardrop knuckle**: circle + two tangent lines at 45° | Knuckle on `arm_base` and `arm_head`, every horizontal hole ≥ 5 mm | `CenterArc` + `Line`s, `make_face` | Prints without support. G1 only at the tangent points, on purpose: a G2 blend there would overhang more than 45°. |
| **Hirth rosette**, 24 teeth, 90° included | Every arm joint | One tooth = `loft([V-triangle face at r_out, Vertex(axis point)])`. Pattern with `PolarLocations(0, 24)`. Trim with an annulus r 5.0 to 13.0. | Teeth converge on the axis, so all faces are planes. 45° flanks print on any face, at the limit. |
| **Revolved turned profile** | Knob, post column | `revolve(face, Axis.Z)` of a profile made from lines, 45° V-grooves and an elliptic arc | Turned look. All surfaces are analytic. |
| **Two-view intersection** | Collar back with spine | `plan_prism & side_profile_prism` | Reads as milled from two views. Avoids a fillet at the spine root. |

Rules that keep STEP and 3MF valid:

- Never call `offset()` on a sketch that holds a Bezier. The STEP writer drops OFFSET-curve faces. Rebuild the outline from its own parameters at the new size. Offsetting the column profile (circular arcs) is fine; arcs offset to arcs.
- Combine plan pieces as 2D faces first (`collar_face + neck_face + pad_face`), then extrude once.
- Dress edges (chamfers) on the clean blank. Cut holes, pockets and rosettes after.
- No OCC fillets on outer edges. The repo already saw a 3 mm fillet break the 3MF. Chamfers only. If a chamfer fails, the build fails. Do not skip it silently.
- After every part: `is_valid`, one solid, STEP round-trip volume within 0.5 %, 3MF read back with `Mesher().read()` and volume within 1 %.

### 1.3 Edge vocabulary (the same on every printed part)

| Edge | Treatment |
| --- | --- |
| Primary outer silhouette, top and bottom plan loops | 1.0 × 45° chamfer |
| Bed-side edges not covered above | 0.6 × 45° chamfer (stops elephant foot) |
| Hole, counterbore and slot mouths | 0.4 × 45° lead-in (cone or chamfer) |
| Seam faces of the collar | 0.6 × 45° on the outer edges, so the 2 mm seam reads as a V shadow line |
| Mount-face outline on the collar top | 0.36 deep, 0.6 wide V-groove tracing the 40 × 40 soft square (steeper than 45°: it prints on the bed face) |

### 1.4 How the parts read as one product

- The collar top ring and its mount face are **one flat plane**. The plan outline runs collar → G2 neck → pad with no step.
- No ears. The seam screws run inside the collar wall.
- Foot plan = mount-face plan. Arm base on the collar, and a plate on the head, sit edge-aligned.
- Base and head knuckles share one teardrop silhouette. Links share one dog-bone plan. Every knob is the same part.
- Nut slots on the collar open into the bore and hide behind the liner.

## 2. Assembly architecture

### 2.1 Printed parts

| Part | Material | What it is |
| --- | --- | --- |
| `collar_front` | PETG | Front half of the clamp shell. Seam screw counterbores. |
| `collar_back` | PETG | Back half, neck, spine, mount face (MF). Seam nuts and MF nuts, all slots open into the bore. |
| `liner_front`, `liner_back` | TPU 95A | 2.0 mm grip liners with 45° V ribs. They snap behind the shell lips. They cover the nut slot mouths. |
| `arm_base` | PETG | Foot (4 × M3 to the MF) and a teardrop upright with joint J1. |
| `arm_link` × 2 | PETG | Same part twice. Dog-bone plan, 70 mm pitch, 10 mm thick, a rosette at each end. The second one is turned 180° about its long axis. |
| `arm_head` | PETG | Teardrop knuckle J3 and an MF on top. |
| `knob` × 3 | PETG | Turned knob holding an M6 nut. |
| `zve10` | PETG | Hero payload plate. Foot to the head MF, 24 mm camera slot (kept). |
| `cam_plate`, `phone`, `gopro`, `post` | PETG | Accessories, ported to the shared foot and MF. Their special features stay: `PHONE_EAR_W`, the GoPro 13 × 5 stud pocket, the three camera stations. |
| `coupon_*` | as parent | Slices of the real parts (§6.4). |
| `cable_clip` (optional) | TPU 95A | Snap clip on a link waist for a USB-C cable. Cut first if time runs out. |

Deleted: `band.py`, `block.py`, `hinge_base`, `hinge_link`, `hinge_mid`, the old coupon, and their exports.

### 2.2 Bought parts (modeled as solids in the assembly)

M3 ISO 4762 screws, ISO 4032 nuts, ISO 7089 washers. M6 ISO 4017 bolts, ISO 4032 nuts, ISO 7089 washers. 1/4"-20 hex nuts (ASME B18.2.2). One 1/4"-20 socket cap camera screw (ASME B18.3). A Sony ZV-E10 envelope (115.2 × 64.2 × 44.8 mm, Sony help guide). The bought GoPro buckle, phone clamp and ball head are not modeled.

### 2.3 What bolts to what

```
column
  └─ liner_front / liner_back          (TPU, friction)
       └─ collar_front ══4× M3 seam══ collar_back
                                        └─ MF ── 4× M3 ── arm_base
                                                            └─ J1 ── arm_link
                                                                       └─ J2 ── arm_link (flipped)
                                                                                  └─ J3 ── arm_head
                                                                                            └─ MF ── 4× M3 ── zve10
                                                                                                               └─ 1/4"-20 ── ZV-E10
```

### 2.4 Load path, camera to column

1. Camera weight goes through the 1/4"-20 camera screw into the `zve10` plate.
2. The plate bears flat on the head MF. Four M3 screws pull it down into captive nuts. The nuts are pulled toward the face skin, never toward a slot mouth.
3. The head transfers moment through J3. The rosette teeth lock it. M6 preload keeps the teeth engaged.
4. Links carry bending along their layers to J2 and J1.
5. `arm_base` bears on the collar MF with 4 × M3.
6. The spine is a cantilever. It is printed flat, so bending runs along the layers into the collar ring.
7. The ring presses the TPU liner onto the column. Four seam screws make the normal force. Friction holds the device up.

Design load: 1.0 kg payload (estimated: ZV-E10 about 343 g with battery, plus lens and margin; not verified here). Lever from J1 to the camera: both links level (140) plus the deck offset (48.5) = 188.5 mm, derived in code. Sustained moment at J1 about 1.85 Nm. Peak check uses 2×.

### 2.5 Assembly order (the checks use these steps)

1. Slide 4 × M3 nuts and 1 × 1/4" nut into the collar_back MF channels from the bore side. Slide 4 × M3 seam nuts in from the bore side.
2. Snap the liners into both shells. Each liner now covers the slot mouths and keeps the nuts in.
3. Put the halves round the column. Fit 4 × M3 seam screws with washers from the −Y face. Tighten crosswise to 0.4 Nm (estimated; confirm on the coupon). The seam must stay open.
4. Fix `arm_base` to the MF with 4 × M3 from above. This happens before J1 is assembled, so the key can reach.
5. Build J1, J2 and J3. Each joint gets a hex bolt in its pocket, a washer and a knob.
6. Fix `zve10` to the head MF with 4 × M3. Then fit the camera with the 1/4" screw from below.

## 3. Fastener schedule

### 3.1 Bought dimensions and pockets

Fit rule: a pocket is the max part size plus the fit shown. A pocket is a press or slide fit, never the PETG clearance 0.30.

| Part | Published size (max) | Source | Pocket / hole | Fit |
| --- | --- | --- | --- | --- |
| M3 nut ISO 4032 | s 5.32–5.50, m 2.15–2.40 | Wermac; HB&H cut sheet | 5.70 AF × 2.60 | +0.20 / +0.20 |
| M6 nut ISO 4032 | s 9.78–10.00, m 4.90–5.20 | Wermac; HB&H | 10.20 AF × 5.50 in the knob | +0.20 snug / +0.30 |
| 1/4"-20 nut ASME B18.2.2 | AF 10.87–11.13, t 5.38–5.74 | amesweb table | 11.45 AF × 6.05 | +0.32 / +0.31 |
| M3 ISO 4762 head | dk 5.68, k 3.00, key 2.5 | Westfield | Head only: Ø6.2 × 3.3. With washer: Ø7.4 × 3.8 | +0.5 dia, +0.3 deep |
| M6 ISO 4017 hex head | s 10.00, k 4.0 | **estimated**: not in the sourced tables. Check ISO 4017 or the supplier sheet before printing. | 10.30 AF × 4.30 | +0.30 / +0.30 |
| M3 washer ISO 7089 | 3.2 / 7.0 / 0.5 | Westfield | Inside the Ø7.4 counterbore | |
| M6 washer ISO 7089 | 6.4 / 12.0 / 1.6 | Westfield; Fabory | Ø12.4 × 1.0 recess, washer 0.6 proud | +0.4 dia |
| Clearance ISO 273 medium | M3 3.4, M6 6.6 | iso.org; chart | 3.4, 6.6; 1/4" 6.6 | |
| 1/4"-20 SHCS ASME B18.3 head | Ø9.53 × 6.35 | Westfield | Ø10.1 × 6.7 | |
| ZV-E10 tripod screw | thread depth < 5.5 | Sony help guide | screw length = under-head + 4.5 target | |

Every value above goes into `params.py` with its source in a comment. Values not from a source keep `# estimated:`.

### 3.2 Stack rule (one function per joint)

Each joint gets `stack_<joint>(spec, state) -> list[Check]` in the macropad `(name, value, limit)` row shape. The repo's `at_least` and `at_most` already match it.

```
grip       = sum of clamped thicknesses under the head (washer included), measured on the solids
L_min      = grip + m_nut_max + protrusion          protrusion = 2 × pitch
L          = shortest stock length >= L_min + tol   tol: M3 0.3, M6 0.42, 1/4" 0.25 (estimated, ISO 4759-1 js15)
tip_bore   = (L + tol) - grip_closed + 0.5          the blind bore past the nut, drilled into the model
rows       : engagement (short case L - tol, open state) >= m_nut_max + protrusion
             tip clear  (long case L + tol, closed state) <= tip_bore - 0.5
             head below the surface >= 0.3
             wall around head, nut and bore >= wall_min (solid distance)
```

`state` is `open` (as modeled; the seam is 2.0 mm open, the rosette is lifted to index) or `closed` (seam closed, rosette engaged). Both are checked, each with the short and the long screw.

### 3.3 Schedule

Lengths marked "expected" are what the formulas should give. The code decides and the README BOM comes from the code.

| Joint | Qty | Screw | Grip (formula) | Nut / seat | Access | Head |
| --- | --- | --- | --- | --- | --- | --- |
| Collar seam | 4 | ISO 4762 M3, expected ×35 | washer 0.5 + seat-to-seam (front) + seam_gap 2.0 + back skin 3.0 | ISO 4032 M3 in collar_back, 4.0–6.6 mm past the seam, slot into the bore along ∓X | −Y face, 2.5 key | Hidden 0.3 below the rim in a Ø7.4 counterbore. Seat as deep as the wall to the bore allows (≥ 1.5 mm). |
| Foot to MF (arm_base, zve10, every accessory) | 4 per foot | ISO 4762 M3, expected ×12 | foot under-head (foot 8.0 − 3.3) + MF roof 2.0 | ISO 4032 M3 at MF depth 2.0–4.6 | From above, before the next part goes on | In a Ø6.2 × 3.3 counterbore. The part above covers it. |
| Arm joints J1, J2, J3 | 3 | ISO 4017 M6, ×30 from the code (×25 fails open-state engagement) | (10 − 4.3) + (10 − 1.0) − rosette engagement h_e + washer 1.6; plus lift 2.2 in the open state | ISO 4032 M6 inside the knob, flush with the knob base, bearing on the washer | Knob by hand | Hex head in a 10.3 AF pocket, so it cannot turn |
| Camera | 1 | 1/4"-20 SHCS, expected 3/8" | plate under-head 5.0 | Camera socket, engagement 4.5 (limit < 5.5) | From below. The slot sits outside the MF footprint, so the key reaches it with the plate mounted. | Ø10.1 × 6.7 counterbore slot from below |
| MF 1/4" port | 1 per MF | none in the hero; a bought 1/4"-20 male stud for a ball head | | 1/4" nut at MF depth 6.1–12.15 | | |

Washers: under every seam screw head (PETG bearing stress about 28 MPa at 830 N, versus about 57 MPa without), and under every knob. Foot screws carry low preload, so they get no washer.

Torque (estimated, confirm on the coupon): M3 into a PETG nut pocket 0.4 Nm. Knob by hand, about 1.5 Nm.

Heat-set inserts: not used. Captive nuts load the PETG in compression against a solid floor (Embedded Computing test: bottom pockets held 160+ kg, side pockets about 85 kg). They also keep one part family and need no tool. Inserts stay the fallback if a nut coupon fails.

### 3.4 Captive nut rule (fixes defects 1 and 5)

`captive_nut(af, m, at, axis, travel, mouth_face)` returns `(pocket, channel)`.

- Turn the hex so one corner points along `travel`. Then the nut's width across the channel is its across-flats, so channel width = pocket AF.
- The pocket sits so the screw load pushes the nut into solid material. It never pushes toward the mouth.
- The channel runs from the pocket to `mouth_face` with the pocket's cross-section. On the collar the mouth is the bore wall. On the head and the accessories it is the part's back face.
- A channel ceiling wider than 10 mm in print pose gets a 45° gable roof on the far side from the screw head. This applies to the 1/4" channels (11.45 wide). The gable sits in the tip zone, so the nut still seats on flats.
- Horizontal-axis hex pockets in print pose: a corner on the travel axis leaves a flat on top. That flat is a short bridge (3.3 mm for M3).
- Two nuts on one channel (collar MF, x = ±10): insert the far nut first. The travel check runs in that order.

## 4. Interfaces and connectors

### 4.1 Mount face standard (MF-40)

| Item | Value |
| --- | --- |
| Plan | 40 × 40 soft square (foot uses the same plan) |
| Structural | 4 × M3 on a 20 mm square (sketch 4), captive nuts, roof 2.0, nut 2.0–4.6 deep |
| Port | 1 × 1/4"-20 nut, center, roof 6.1, nut 6.1–12.15 deep. Vertical gap to the M3 nuts 1.5 (= PETG wall_min). |
| Min depth | 1/4" tip bore floor + 1.5. Expected 24 mm under the face. |
| Datum | The face is flat and is the print bed face where the part allows (collar_back, arm_head). |
| Rotation | Feet index every 90° on the M3 square. Four screws stop twist, so no anti-twist pin is needed. |
| Size reason | Research: a mount face ≥ 40 mm supports a ball-head base or clamp fully. |

Feet always use the 4 × M3. The 1/4" port is for bought female-thread gear on a stud. The GoPro foot keeps its dropped-in screw stud.

### 4.2 Arm joints: Hirth rosette lap joints

- Lap joint, not clevis. A clevis cannot open to re-index teeth on both faces.
- 24 teeth, 90° included angle, annulus r 5.0–13.0, tips and roots truncated 0.15, flank fit 0.10. Index step 15°. Lift to re-index 2.2 mm (tooth height 1.7 at r 13 plus 0.5).
- Holding torque at 1250 N preload, flank 45°, μ 0.2: about F · r_m · tan(45° + 11.3°) ≈ 18 Nm. That is far over the 3.4 Nm peak. The friction coefficient and knob torque are estimated.
- Fine aim is the bought ball head's job. The rosettes set position in 15° steps.
- Knob: Ø28, 17.6 mm tall (from the stack: M6×30 tip + 1.0 + roof 2.0 + dish 0.6), three 90° V-grooves 0.5 deep, 24 vertical flutes, elliptic top dish 0.6 deep. The nut sits in a 10.2 AF pocket open at the bottom. A blind bore above takes the bolt tip. The roof over it is a 6.6 mm bridge.
- Chain: J1 rosette on the base +X face. link 1 on +X. link 2 flipped, back on −X. head on +X. Knobs alternate +X, −X, +X, so one `arm_link` part serves both links.
- Steel washer under each knob. PETG never rubs PETG under preload except on the tooth flanks.

### 4.3 Deferred, with reasons

- Arca-Swiss dovetail: no published height (7.5–8 mm varies by maker), and it needs a clamp mechanism. Revisit with a calipered plate.
- Heat-set inserts: see §3.3.
- USB-C cable routing: only the optional `cable_clip` (TPU C-clip on the link waist, cable Ø ≤ 5.0 estimated). No channels are cut into the links. They would weaken the bending section.

## 5. Strength and print plan

Allowed sustained stress (engineering judgment for creep): 0.1 × UTS in the stress direction. PETG along layers 3.85 MPa (Ultimaker 38.5 XY). Across layers 1.9 MPa (19.0 Z). Peak (2× load) is limited to 0.25 × UTS. TPU carries no structural stress.

| Part | Print pose (bed normal, use frame) | Main load | Stress vs layers | Perimeters / top-bottom / infill | Notes |
| --- | --- | --- | --- | --- | --- |
| collar_front | top face down (0,0,+1) | Seam hoop tension | Along | 5 / 5 / 40 % gyroid | Seam counterbores are horizontal in print: teardrop roof. |
| collar_back | top face down (0,0,+1) | Spine cantilever + hoop | Along (bending stress runs along Y) | 5 / 5 / 50 % gyroid | MF is on the bed: flattest face. The scoop faces up and needs no support. Shell lip undersides are 45°. |
| liner_front/back | axis vertical (0,0,−1) | Compression | n/a | all perimeters (2 mm), 25 mm/s | Ribs are 45° V rings, 0.8 tall, 5 mm pitch. No horizontal ledges. |
| arm_base | foot down (0,0,−1) | Moment at the upright root | **Across** | 6 / 6 / 60 % | Short, wide upright. Section stress check must pass at 1.9 MPa. Teardrop knuckle and bore. |
| arm_link | on its side, axis vertical (±1,0,0) | Bending about the joint axis | Along | 6 / 5 / 50 % | Waist ≥ 20 wide × 10 thick. Expected σ ≈ 2.6 MPa at 1.7 Nm. |
| arm_head | MF down (0,0,+1) | J3 moment into the MF | Across at the neck | 6 / 6 / 60 % | Teardrop knuckle points to the MF. Section check at the neck. |
| knob | base down (0,0,−1) | Torque, hand | Along | 4 / 5 / 40 % | |
| zve10, cam_plate | foot down (0,0,−1) | Plate bending | Along | 4 / 5 / 40 % | |
| post | foot down | Bending | Across | 6 / 6 / 60 % | Column Ø ≥ 24. Section check. |
| phone, gopro | foot down | Light | — | 4 / 5 / 40 % | |

Wall minimum: PETG 1.5 (4 perimeters at 0.45 = 1.8 is the target). TPU 1.2. Measured with solid distance, never with bounding boxes. Horizontal holes ≥ 5 mm get a 45° teardrop. Holes < 5 mm print round (they print about 0.15 small; 3.4 covers it).

Collar sizing (all derived in code from `params.py`):

```
bore offset    = fit 0.25 + rib_h 0.8 + liner_t 2.0 = 3.05      (bore 46.3 × 61.6, r 15.05, circular arcs)
seam_gap       = 2·fit + 2·rib_h − 0.1 = 2.0                    (seam closes before the ribs crush flat)
seam bolt x    = bore half-width + wall_min + 3.4/2  = ±26.35
outer half-w   = bolt x + 7.4/2 + 1.8 + 1.0 (corner) = 33 → 66 wide (no ears)
outer half-d   = bore half-depth + 4.0               ≈ 35 → 70 deep
height         = 40, seam bolts at z 10 and 30, shell lips 1.2 tall at both ends
spine          = neck from the collar back to MF center y ≈ 57; scoop ellipse from (y 35, z 0) to (pad end, z 16)
```

Grip math (estimated μ 0.5 TPU on aluminium): 4 × 500 N preload gives about 2000 N normal force per side. That holds about 2000 N axially, versus about 20 N peak. With 38 % compression set it still holds about 1240 N.

## 6. Verification plan

Every check below runs in `checks()` at build time (`python -m holder` exits 1 before export on any FAIL) and again in pytest. Each new check gets a negative test that must fail.

### 6.1 Nut travel (defects 1, 5)

For each captive nut, build the nominal max nut (s_max, m_max, a real hex prism). Sweep it from the mouth to the seat (`extrude` of the hex section along the travel vector). Assert `(sweep & part).volume <= 1e-6`. Also measure the channel width across travel from real faces: ≥ s_max + 0.15. Negative test: the old corner-on-±X pocket must fail.

### 6.2 Screw stacks and tip clearance (defect 3)

`stack_*` rows from §3.2 for every joint, in the open and closed states, with short and long screws. Bought screws, nuts and washers are solids placed in the assembly at the long length, so §6.3 catches a tip in solid. The only allowed overlap is screw with its own nut (the thread).

### 6.3 Assembly clash at build time (defect 4)

`assembly(spec, pose)` returns every printed and bought solid. Pairs are pruned by bounding-box overlap (pruning only). Every remaining pair needs `intersection volume <= 1e-6`, except names in `INTENDED = {(screw_i, nut_i)}`. Also:

- Liner rib tips to the column: solid distance within fit ± 0.02.
- Seam gap between the shells ≥ 2.0 − 0.05.
- Rosette pairs in the engaged pose: zero clash, flank gap 0.05–0.15.
- Joint sweep: each joint alone, 0–360° in 15° steps. Record the clear range. The hero pose and the folded pose must be inside it.
- Tool access: for each screw at its assembly step (§2.5), a rod (key Ø + 1 mm, 120 mm along the screw axis) misses every part present at that step.

### 6.4 Coupons cut from the real parts (defect 2)

- `coupon_collar_front/back` and `coupon_liner_front/back`: `part & slab(z 4–16)`, which holds the z 10 seam screw row. Ribs, fit and seam gap are the real ones by construction. Test: the coupon bore offset equals the collar bore offset.
- `coupon_mf`: the collar_back MF region cut by a box that keeps one M3 channel and the 1/4" channel with open mouths. It proves the nut travel and press fit.
- `coupon_rosette`: two 10 mm knuckle discs with teeth and the M6 bore. It proves tooth print quality and lift.
- Old `exports/coupon_*` are deleted and regenerated.

### 6.5 Print checks per part, in print pose

- Overhang: sample face normals on a UV grid. Fail any down-facing area steeper than 45° (0.5° tolerance) that is above z 0.05, unless it is a horizontal face whose narrow span (OCP oriented box) is ≤ 10 mm.
- Section stress: slice at the named critical planes. Get I and c from the real face (OCP `BRepGProp` inertia about the centroid). σ = M / (I / c) against §5.
- Form: curvature jump at every G2 junction of the plan wires ≤ 1e-3 /mm, measured on the real edges.

### 6.6 BOM from code

`bom(assembly) -> rows (qty, designation, standard, source)`. `python -m holder --bom` prints it. README holds it between `<!-- bom:start -->` and `<!-- bom:end -->`. Pytest asserts the block equals the generated text. This replaces the `M3×16` string test.

### 6.7 Renders

- Pipeline: `tools/render/scene.py` for Blender 5.2 (`flatpak run org.blender.Blender -b --factory-startup --python tools/render/scene.py`). Port from `~/Projects/worxboard-ergo/tools/render.py` (Finish, lights, crop) and worxpad `tools/render/scene.py` (glossy floor).
- Mesh: tessellate at 0.03 mm. Weld, shade smooth, sharp edges above 32°.
- Look: AgX Medium High Contrast, three area lights on a rig that turns with the camera, shadow catcher, 96 samples with denoise.
- Finishes: PETG space grey (roughness 0.5, fine grain bump), TPU black (roughness 0.8), steel screws (metallic, roughness 0.35).
- Views: hero 3/4 of the full assembly with the ZV-E10 envelope; side view (scoop, teardrops); top view (G2 plan); knob and rosette close-up; one zebra-stripe environment view of the collar to judge G2.
- The renders are judged on the curves and the joins, not on colour. A colour-only change is not a form change.
- Fallback without Blender: keep `tools/shade_stl.py` for line-level previews. Do not ship mesh-triangle renders.
- Write `exports/report.json` with all check rows and a hash of `src/` so renders and reports match the code.

## 7. Scope and implementation order

Stays: the measured column (40.2 × 55.5, r 12 estimated), `print_orientation`, `intersection_volume`, `one_solid`, `void_open`, `wall_encloses`, `export.py` (mesh fix; it does not repair), `preview.py`, the part-module contract (`NAME`, `MATERIAL`, `PRINT`, `build`, `checks`), `m3_stock`, `PHONE_EAR_W`, the ZV-E10 24 mm slot, the GoPro 13 × 5 pocket, `.grok/workflows` (untouched).

Deleted: `band.py`, `block.py`, old `coupon.py`, the hinge builders, `tools/render_gallery.py` (replaced), stale exports.

New, in order. Each step ends with `uv run pytest` and `uv run python -m holder` green.

| # | Do | Done when |
| --- | --- | --- |
| 1 | `params.py`: sourced fastener table (§3.1), pockets, MF-40, collar and arm groups, `validate()`. `form.py`: soft square, G2 blend, teardrop, curvature probe. `common.py`: `captive_nut`, nut sweep, stack rows, bought solids (screw, nut, washer), clash matrix, overhang and section probes. Port what fits from `monitor-stand-extenstions-holder-cad/src/holder/common.py`. | Unit tests for each tool, with negative tests. |
| 2 | `mount.py`: `mount_face()` and `foot()` builders and their checks. | MF block and foot pass travel, stack, wall and overhang checks. |
| 3 | `collar.py`: collar_front, collar_back (neck, spine, scoop, MF), liners. | All collar checks pass. Coupon slices exported. |
| 4 | `arm.py`: rosette, knob, arm_base, arm_link, arm_head. | Rosette pair, stacks, sweep ranges and section stress pass. `coupon_rosette` exported. |
| 5 | `assembly.py` + `__main__`: hero assembly, bought solids, clash matrix, access rods, BOM, `report.json`. | Build exits 0 at 1e-6. The BOM matches README. |
| 6 | `accessories.py`: zve10 first (hero), then cam_plate, phone, gopro, post on the shared foot and MF. | Each one passes; hinge builders gone. |
| 7 | Render pipeline and new `docs/renders`. | Hero, side, top, detail and zebra images committed. |
| 8 | README rewrite (parts, print table from §5, BOM block, assembly order), version 0.4.0. One line at the top of PLAN.md pointing here. | Docs match the code. |
| 9 | Optional: `cable_clip`. | Only if 1–8 are done. |

Commit after each step on `feat/apple-form`. Do not push. Never touch `main`.

## 8. Open items (estimated until measured)

| Item | Value used | How to close |
| --- | --- | --- |
| Column corner radius | r 12 | Collar coupon |
| ISO 4017 M6 head s / k | 10.0 / 4.0 | Supplier sheet or calipers |
| Payload mass | 1.0 kg design | Weigh the camera |
| M3-in-PETG torque | 0.4 Nm | Coupon to failure; use a fraction |
| TPU-on-aluminium friction | μ 0.5 | Coupon pull test |
| Screw length tolerances | ±0.3 M3, ±0.42 M6, ±0.25 1/4" | ISO 4759-1 table |
| ZV-E10 socket offset | 24 mm slot | Calipers |
| Phone ear width / hole | 16 / 4.7 | Calipers |
| USB-C cable diameter | ≤ 5.0 | Calipers |

Passing CAD checks is not a printed fit. The coupons and a 24 h payload hang test are the real proof.

## 9. As built in v0.4.0 (where the code differs, and why)

| Item | Brief | Code | Why |
| --- | --- | --- | --- |
| Collar outer width | 64 | 66 (half-width 33) | A 14 mm G2 front corner takes 1 mm back from the wall round the seam washer bore. The wall probe measures 1.5. |
| Seam seat | about -23 | -23.64, solved from the bore corner circle | Deepest seat that keeps 1.5 mm to the bore. |
| Knuckle radius | 14 | 14.5 | Rosette r 13 + 0.5 flat + the 1.0 chamfer. |
| Rosette trim | annulus r 5..13 | r 5..13 at the pitch plane, 45° cones away from it | A cylindrical trim left small flat ceilings on the vertical rosettes. |
| Horizontal hex pockets | corner on top | flat on top (M6 head pocket in arm_base) | Corner on top leaves 30° roofs. A 5.9 mm flat is a short bridge. |
| Overhang rule for ceilings | narrow span of the OCP oriented box ≤ 10 | every point of the ceiling within 5 mm of its outer wire | Same result on rectangles; also right on a ring round a hole, whose box is wider than its bridge. |
| M6 joint screw | ×25 | ×30 | Open state (rosette lifted 2.2) needs nut height + 2 pitches on the short screw. |
| Knob height | 14 | 17.6 | From the M6×30 stack. |
| Lever | 170 | 188.5, derived | Includes the camera deck offset. All stress rows pass at that lever. |
| Camera plates | | 11.7 thick, foot counterbores 7.0 deep | 5.0 under the 1/4" head + the 6.7 counterbore. Under-head on the foot stays 4.7, so every foot uses M3×12. |
| MF channels on the collar | open into the bore | open into the bore through a notch in the top lip | The M3 channels (z 35.4-38) cross the lip cone. The notch is behind the liner. |
| Post MF | 1/4" gable roof | 45° edge strips, 9.8 mm flat seat | The post prints MF up, so the channel ceiling is the nut seat. A gable there would leave nothing flat to bear on. |
| Post foot screws | key from above | fluted column, 2.5 mm key ≥ 100 mm down the flutes | The top MF uses the same 20 mm square, so nothing else reaches the foot heads. |
| Phone walls | 16 long | 12.8 long (y within ±6.4) | Clear of the foot counterbores at y ±10. The ear screw tip runs on into the MF port bore. |
| Joint sweep | every pair | printed parts, knobs, camera; the swept joint's own rosette pair is left out | The teeth repeat every 15°, so each step meshes like the hero; that pair is checked in the rosette rows. Screws and nuts sit inside their hosts. |
| 3MF read-back | `Mesher().read()` | signed volume of the triangles in the file | `Mesher().read()` rebuilds a face per triangle and took over 20 minutes on one part. |
| Renders | `tools/render/scene.py` | `tools/render/meshes.py` (uv) writes STLs and runs `scene.py` (Blender 5.2) | build123d is not importable in Blender's Python. |
| Head and post MF slot mouths | the part's back face | the −Y face | The +Y face is what the hero view sees. On the head the −Y face sits under the camera deck. |
| Head neck | MF block on the knuckle | the block cut by a front-view G2 ogee, 40 mm to 10 mm over 12 mm | A stacked block read as a box. The ogee keeps 1.5 mm round the 1/4" tip bore and runs inward as it drops, so the head still prints MF down without support. |
| `cable_clip` | optional | not made | Step 9 was optional. |

Measured clear ranges (absolute joint angle): J1 -30° to 105°, J2 300° to 195°, J3 255° to 75°. Hero (75°, 120°, 0°) and folded (90°, 300°, 0°) are inside them.
