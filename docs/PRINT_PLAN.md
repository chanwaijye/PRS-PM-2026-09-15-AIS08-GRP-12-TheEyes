# Print plan — objects, trays, jig

Written 2026-09-12, the evening red / blue / yellow landed. **Revised
2026-09-13:** grey arrived a day early and the red cube is printed and
callipered, so §4 is re-sequenced around the gates and real slicer times rather
than deliveries. White and black are already on the shelf. Days are Singapore
time.

This is the execution order for `docs/RIG_BOM.md` §H, and it exists because the
printing is on the critical path: `docs/STATUS.md` puts the physical objects
ahead of calibration as the longest pole, and `docs/CAPTURE_PROTOCOL.md` §3
lists "trays printed and calliper-verified" as a prerequisite the scored run
cannot start without.

Nothing here has been printed yet. Every number in the gate tables below is
blank on purpose — fill them from callipers, not from expectation.

---

## 1. Colour is not a choice, and neither is which spool goes where

`training/detector/replicator_dataset.py` rendered `synth50k_v2` with each class
in a fixed base colour and `FILAMENT_SPREAD = 0.12` of linear randomisation. The
detector has already learned those distributions, so a spool outside the band is
an appearance gap opened *after* training.

| Spool | Goes on | Target sRGB | Acceptable band |
|---|---|---|---|
| yellow | `mustard_bottle` | `#EDDD59` | `#DECB00` – `#FCEC81` |
| red | `red_cube` | `#DA5555` | `#C80000` – `#EA7E7E` |
| blue | `bowl` | `#6199D7` | `#007CC5` – `#86B1E7` |
| grey | trays, board jig | `#898989` | one of the three distractor neutrals |

White is the neutral `#E7E7E7`, so it is *technically* a legal tray colour and
would unblock the trays two days early. **Do not use it for the trays.**
`docs/STATUS.md` records that the detector's false positives already concentrate
on "whatever in the room is bright enough to resemble training data — the white
fan, the pillow, the pale mousepad", and a tray is a large slab sitting directly
under every target. Three white slabs manufacture more of the exact substrate of
the dominant failure mode. Grey is mid-tone, equally in-palette, and two days
away; the objects fill those two days completely (§4). White's real job is
distractors (§7).

Black is off-palette *and* IR-absorbing. It has exactly one use here — the
occluder furniture, §7 — and must never appear as an object or a tray.

If "blank" means natural/uncoloured PLA rather than black: it is translucent,
and subsurface scattering corrupts active-stereo depth the same way gloss does.
Keep it off the table.

**Matte, not silk, for everything.** The renderer draws these at roughness
0.55–0.90; gloss defeats the IR projector and depicts something the training set
does not contain.

---

## 2. Four things found while preparing this plan

**2a. The print files did not exist, and the units were a trap.** Only
`red_cube` had a print-ready STL. `assets/meshes/*.obj` and `assets/trays/*.stl`
are authored in **metres**, because everything downstream is metric — but STL
carries no unit and Bambu Studio reads one unit as one millimetre, so
`bowl_tray.stl` imports as a 0.21 mm speck and the slicer offers to scale it to
fit. That is the silent rescale `RIG_BOM.md` §H forbids.

Fixed: `training/export_print_stls.py` emits millimetre STLs into
`assets/print/`, and `--verify` re-checks them. Neither cleanup it applies moves
a vertex — UV-seam merging is topological, and dropping the bowl scan's six
stray two-face fragments (largest 1.49 mm) only removes debris — so **the
printed geometry is the geometry ADD-S is computed over**.

**2b. `assets/trays/bowl_tray.stl` was not one solid — now fixed.**
The socket is cut by subtracting the object's own mesh from the plate. That is
correct for the cube and the bottle, which are solids. The bowl is a **0.99 mm
shell** — subtracting it leaves the bowl's own interior behind as plate
material:

| body | size | volume |
|---|---|---|
| the plate | 210.0 × 210.0 × 8.5 mm | 323.0 cm³ |
| **a loose island inside the socket** | 98.5 × 98.3 × 5.3 mm | **18.7 cm³** |

The island is disconnected, and the `lighten = 60.0` bore removes the floor
under its middle, so it is also unsupported. Sliced as-is with supports off it
prints as spaghetti in the cavity the bowl is supposed to seat in, and the 193 g
figure in `RIG_BOM.md` includes material that cannot go where it is drawn.
`author_marker_trays.py --verify` passes because it checks watertightness and
bed fit, not body count.

**FIXED 2026-09-12.** `solidify_socket()` in `author_marker_trays.py` drops any
body the socket cut leaves enclosed by the object, which computes the cut the
socket should always have made: by the solid the object *displaces*, not by its
surface. Doing it as a deletion rather than by filling the shell keeps the socket
on the bowl's real, slightly out-of-round outer profile — measured, the outer
radius varies **0.5–1.0 mm peak to peak** at a given height in the socket band,
so a circumscribing circular socket would carry that as lateral play and cost
roughly 5× the ground-truth bound. It refuses to delete anything outside the
object's own envelope, so a cut that goes wrong elsewhere fails loudly instead.

The generator now also reports **body count**, and `--verify` fails on anything
but one — the check whose absence let this through. Watertightness does not imply
it: each body was watertight on its own.

Verified on the regenerated geometry:

| | before | after |
|---|---|---|
| bodies | 2 | **1** |
| bowl tray mass | 193 g | **176 g** |
| `config/marker_trays.yaml` | — | **byte-identical** |
| other two trays, all marker sheets | — | **bit-identical** |

No ground-truth number moved, which is the part that matters — `t_tray_object`,
the marker offsets and `measured_clearance_mm` (0.082–0.118 mm) are all
unchanged, so the transform chain is exactly what it was. The clearance
measurement did need a matching fix: with the socket a solid cavity, the bowl's
inner contour faces no tray material, and measuring it would have reported the
0.99 mm wall thickness as clearance. `outer_loop_xy()` restricts the measurement
to the outer contour; the cube and bottle section as a single loop and are
unaffected.

Seating checks on the fixed tray, all passing: the bowl rests at exactly the
configured height, contacts material at radius 34.5–45.1 mm (clear of the 30 mm
lightening bore), and lifts straight out — overlap falls to zero within 0.05 mm
of lift, so there is no undercut.

One honest residual, and it predates this fix: seated, the objects overlap the
tray by a sliver — **0.051 mm** maximum penetration on the bottle, **0.022 mm**
on the bowl, zero on the cube. `inflate_lateral` grows the cutter by a per-axis
*scale* rather than a true surface offset, so on the near-horizontal curved
undersides at the socket floor it yields no normal clearance. Both are a quarter
of a layer height or less and below anything a 0.4 mm nozzle can resolve, so it
is recorded rather than chased.

**2c. The brown test cube does not discharge the dimensional gate.** The
committed `assets/print/red_cube.3mf` says the test print was **PETG**
(`#4F2C1D`), not PLA. PLA and PETG do not shrink alike, so its dimensions say
nothing about the parts that matter, and no calliper result for it is recorded
anywhere in the repo regardless. `RIG_BOM.md` §H calls the calliper check
mandatory. **The first red PLA cube is the gate** — passed 2026-09-13, §4.

**2d. The marker sheets carry no DPI tag — print the PDF instead.**
`assets/trays/*_markers.png` are 600 dpi but store no resolution metadata, so a
print dialog picks its own scale, and `bowl_markers.png` is 210.5 mm wide — wider
than A4, so it clips even when scaled correctly.

`training/marker_sheet_pdf.py` writes `assets/print/aruco_markers_A4.pdf`: all
eight cards cut from the strips and pasted pixel for pixel onto one A4 page, a
10 mm gutter between cards (the strips' 0.85 mm gap would leave a card cut down
its middle 52.42 mm, too big for the 52.4 mm recess), solid cut lines on every
card edge running on across the gutters so each row and column is one straight
scissor cut, a 100 mm bar on each axis, and a lossless 1-bit encoding — Pillow would
otherwise store the page as JPEG, which rings at exactly the edges ArUco
localises. The script refuses to write unless all eight ids decode from the
rendered page and all ink is 8 mm clear of the edges.

```bash
lp -d HP_M28w_ipp -o media=A4 -o print-scaling=none assets/print/aruco_markers_A4.pdf
```

**The tolerance is tighter than it looks.** Marker length sets the scale of
`T_cam_marker`: a marker printed 0.5% large makes every ground-truth pose 0.5%
too close to the camera — about **3.5–4.5 mm at the rig's 0.7–0.9 m**, which is
40–52% of the cube's 8.66 mm ADD-S threshold. That dwarfs the socket clearance.
So calliper, and record:

| check | target | measured |
|---|---|---|
| horizontal bar | 100.0 mm | **100.0 mm** |
| vertical bar | 100.0 mm | **100.0 mm** |
| a black marker square, across | 40.00 mm (drawn 945 px = 40.005 mm) | **40.00 mm** |
| the same square, down | 40.00 mm | **40.00 mm** |

**PASSED 2026-09-13**, printed on `HP_M28w_ipp`: square, and at the calliper's
0.01 mm resolution on a 40 mm edge the scale error is below 0.025%, under
0.25 mm of range at the rig's 0.9 m. `marker_length_m: 0.0400` stands as
configured. That is a resolution bound, not the calliper's accuracy.

Within ±0.04 mm (0.1%, ≈0.8 mm of range error): use it. Outside that, suspect a
scaling setting before the printer, and reprint. If the printer is consistently
off, the measured length — not 40.0 — is what ground truth must use, the same way
the ChArUco board runs on its callipered 38.025 mm. A marker that measures
differently across and down cannot be fixed by any single length; reprint it.

**Cut on the line, not beside it.** The card sits in a 52.4 mm recess with
0.2 mm of slop per side, and the marker's position in the tray comes from the
card's edges — a card trimmed 0.5 mm short on one side and pushed against that
wall moves the marker by about that much, straight into `T_marker_tray`.

Use plain 80 gsm paper and a glue stick. `t_tray_marker` puts the marker face at
the recess floor (z = −0.4 mm); 0.1 mm paper lands within 0.1 mm of that, card
stock or foam tape would not.

---|---|---|
| `red_cube_markers.png` | 104.8 × 52.0 mm | 40.0 mm |
| `mustard_bottle_markers.png` | 104.8 × 52.0 mm | 40.0 mm |
| `bowl_markers.png` | 210.5 × 52.0 mm | 40.0 mm |

---

## 3. Slicer profile — start from the committed one, change four things

`assets/print/red_cube.3mf` holds a working Bambu Lab A1 profile: 0.20 mm
Standard, 0.4 mm nozzle, 2 walls, 15% grid, 5 top / 3 bottom, auto brim 5 mm,
elephant-foot compensation 0.075 mm, supports off. Keep it, and change:

| Setting | Committed | Use | Why |
|---|---|---|---|
| filament | PETG | **matte PLA** | §1 — the whole point |
| `detect_thin_wall` | 0 | **1**, for the bowl | The 0.99 mm wall is under two perimeters at a 0.4 mm nozzle. Off, the slicer drops it as an unfillable gap. **Confirm the wall exists in preview before committing filament.** |
| `outer_wall_speed` | 200 mm/s | **~120 mm/s for the bottle** | 191 mm of tall narrow part on a bed-slinger; Y is the ringing axis. Layer texture is what the depth channel sees. |
| `xy_hole_compensation` | 0 | **0 unless the §4 socket fit test fails — then per object, trays only** | The only knob for socket fit. Never set it globally: a thin-walled bowl's cross-section is an annulus, its inside is a *hole* contour to the slicer, and the setting would move the 0.99 mm wall ADD-S is computed over. |

Non-negotiable, from `RIG_BOM.md` §H: **scale 100%, no bed-fitting rescale** —
the files in `assets/print/` are already millimetres, so decline any prompt
offering to scale them. And do not chase a glossy finish: layer lines are
surface texture for active stereo, not a defect.

Infill note: the committed profile is 15% for everything, while the `RIG_BOM.md`
budget assumed 20% for the cube and 10% for the bottle. At a uniform 15% the set
is `red_cube` 42 g, `mustard_bottle` 171 g, `bowl` 41 g = **255 g**. The bowl is
unchanged at any infill — its shell consumes the whole volume, so the setting is
inert, exactly as documented.

---

## 4. The queue — revised 2026-09-13

### Sliced times

Sliced headless with the installed Bambu Studio: A1, 0.4 mm nozzle, 0.20 mm
Standard, Bambu PLA Matte, 15% infill, plus the §3 overrides per part. The G-code
confirms each override landed.

| part | spool | time | layers | notes |
|---|---|---|---|---|
| `red_cube` | red | 1 h 38 min | 250 | **printed and callipered** |
| `red_cube_tray` | grey | **2 h 29 min** | 53 | |
| `bowl` | blue | **4 h 35 min** | 275 | thin-wall on; the wall extrudes all the way to the rim |
| `bowl_tray` | grey | not timed — read in the GUI | 43 | see below |
| `mustard_bottle_tray` | grey | not timed — read in the GUI | 73 | see below |
| board jig, both parts | grey | **21 min** | 37 | |
| `mustard_bottle` | yellow | **6 h 35 min** | 957 | 8 mm outer brim, 120 mm/s outer wall |

The two large trays have no time because the slicer CLI's auto-arrange rejects
any part past roughly 205 mm including brim — a plain 200 × 200 mm test box fails
identically, so it is the tool, not the parts; the A1 bed is 256 × 256 mm. Worth
knowing for the GUI too: in one batch the CLI **silently dropped the bottle tray
from the plate and sliced only the jig beside it**. Confirm every part is on the
plate before pressing print.

**It does not all fit in one day.** The known jobs still to run sum to
**14 h 00 min**, before the two large trays — each more filament than the cube
tray's 2 h 29 min. On one printer, run serially, this finishes Monday.

### Gate 1 — dimensional: PASSED 2026-09-13

Readings as given, taken in this table's X / Y / Z order:

| axis | nominal | measured | error |
|---|---|---|---|
| X | 50.00 mm | **49.88 mm** | −0.12 mm (−0.24%) |
| Y | 50.00 mm | **49.85 mm** | −0.15 mm (−0.30%) |
| Z | 50.00 mm | **50.10 mm** | +0.10 mm (+0.20%) |

Worst axis 0.30%, a third of the 1% stop line. XY is just past the ~0.1 mm
"proceed untouched" mark, so the reasoning for proceeding anyway:

- **Do not compensate.** One part size cannot separate a scale error, which grows
  with the part, from a fixed contour offset, which does not — and applying the
  wrong one adds error rather than removing it. Z running *over* while XY runs
  *under* already rules out a single uniform shrink factor.
- **The objects cannot feel it.** Even read as a 0.30% scale, the bowl's radius
  moves 0.24 mm against a 16.2 mm ADD-S threshold (1.5%), the bottle's side
  0.15 mm against 19.6 mm (0.7%), the cube's face 0.075 mm against 8.66 mm
  (0.9%) — and every one sits an order of magnitude under the ~2.2 mm depth
  noise.
- **The socket is where 0.1 mm matters, and the cube's undersize helps it:**
  0.06–0.075 mm per side of extra room, if the tray's socket prints true. Whether
  it does is gate 2.
- `config/meshes.yaml` is untouched. The mesh is the reference; this table is
  the recorded error of the part against it.

### Gate 2 — socket fit: PASSED 2026-09-13 (cube tray)

Previously this waited for Monday's grey. It now needs nothing but the cube on
the desk, and it is the gate with the thinner margin: sockets are drawn at
0.20 mm nominal lateral clearance, realised as 0.082–0.234 mm depending on where
you measure, which is at or below ordinary FDM repeatability.

| | value |
|---|---|
| cube seats under its own weight, no force | **yes** |
| rock / lateral play at the rim | **none perceptible by hand** |
| marker card sits flush in its recess | `[VERIFY]` — dry-fit before gluing |

`xy_hole_compensation` stays at 0 for the remaining trays. Socket opening
callipered with the inside jaws:

| socket opening | drawn | measured | printed error | realised clearance per side |
|---|---|---|---|---|
| X | 50.40 mm | **50.32 mm** | −0.08 mm | **(50.32 − 49.88) / 2 = 0.220 mm** |
| Y | 50.40 mm | **50.30 mm** | −0.10 mm | **(50.30 − 49.85) / 2 = 0.225 mm** |

The socket printed 0.08–0.10 mm small and the cube 0.12–0.15 mm small, so the two
errors nearly cancel and the realised clearance lands slightly *above* the
0.20 mm design, not below it. "No rocking" never implied a tight fit — a
flat-bottomed cube on a flat floor cannot tilt whatever the lateral gap.

For ground truth this means the seated cube can sit up to **0.225 mm** off
the socket centre: 2.6% of the cube's 8.66 mm ADD-S threshold. That is the
measured bound for `red_cube` and replaces the CAD figure.

The same errors applied to the bowl socket (drawn 0.082–0.118 mm) still predict
positive clearance — whether they are read as fixed per-side offsets or as a
percentage scale — but that is a prediction, not a fit.

This result does not carry over to the other two trays. The cube socket is a
straight-walled square at 0.20 mm nominal; the bottle's is curved, and the
bowl's is curved, shape-keyed and drawn at 0.082–0.118 mm. Each still gets its
own fit test.

If a later tray is tight, open it with `xy_hole_compensation` **set per object on the
trays only** (§3), not by editing geometry — then measure the realised clearance
on the printed part and record it. `config/marker_trays.yaml` calls these numbers
exact by construction; once the printer has had its say they are a claim about
the CAD, and the ground-truth bound is whatever the part actually does.

### After the gate — the day, then the night

One rule sets the order: **the bottle's slot is whenever you are about to
sleep.** At 6 h 35 min it is the only job long enough to fill a night, and
starting it just before bed means you watch its first layers — which is exactly
where its 12.6 mm² footprint fails. Fill the day before it in gate order:

| order | part | time | why here |
|---|---|---|---|
| 1 | `red_cube_tray` | 2 h 29 min | gate 2; needs only the cube you have |
| 2 | `bowl` | 4 h 35 min | must exist before the tightest socket can be tested |
| 3 | `bowl_tray` | GUI | tightest clearance, 0.082–0.118 mm; fit-test it against the bowl the same evening. **If it would run past bedtime, skip it and start the bottle** |
| night | `mustard_bottle` | 6 h 35 min | watch the first layers, then sleep |
| Monday | `mustard_bottle_tray` + board jig on one plate | GUI + 21 min | the only fit test that has to wait for the bottle; the two jig parts fit beside the 216 × 100 mm tray |

Steps 1 and 2 alone run about 7 h back to back.

**Do these while the printer is busy — none of them needs it:**

- Print `assets/print/aruco_markers_A4.pdf` with the `lp` command in §2d and
  calliper the bars and a marker. Gates ground truth.
- Photograph the red cube beside a white reference under diffuse daylight and
  sample its sRGB against the `#C80000` – `#EA7E7E` band. Record it whichever way
  it falls. **Not** under the rig: `STATUS.md` has the rig 82–93% below the
  training set's darkest one per cent, which would confound this with the open
  exposure problem.
- Re-run the three-condition detector comparison that `CAPTURE_PROTOCOL.md`
  §3.1a defers to this moment. Every detector number to date is confounded by
  exposure *and* by the object being an off-palette brown PETG cube; the red cube
  separates the two.

### Per-part notes

**`mustard_bottle`.** Upright; no supports — the only faces past 60° of overhang
are in the bottom 3.5 mm, the base's own dish. *Corrected 2026-09-17:* this
cleared the dish as harmless without checking it against the tray, whose floor
copied it as a 3.43 mm dome. The printed dish sagged flatter and the bottle
tilted on the dome. The fix is in the tray, not the bottle — §8.1. The 97 × 67 mm footprint is
misleading: the recessed, rounded base meets the plate almost tangentially.
Solid cross-section on a 0.4 mm grid:

| height above bed | solid area |
|---|---|
| 0.1 mm | **12.6 mm²** |
| 0.5 mm | 526.6 mm² |
| 1.0 mm | 1185.0 mm² |
| 3.0 mm | 3565.0 mm² |

A sliver on an irregular ring at radius 20–32 mm, holding down 191 mm above it.
The 8 mm outer brim is already in the sliced profile; clean the plate with IPA
first.

**Brim, never a raft** — for the bottle and the bowl. The object seats by resting
its own base on the socket floor, which is what makes `t_tray_object` exact, so
roughness a raft leaves on the base lands in the ground-truth chain as a
seat-height error. A brim attaches at the perimeter and leaves the base alone.

**`bowl`.** Its foot is a rounded ring too — 48.6 mm² of solid cross-section at
0.1 mm above the bed, 1735.7 mm² at 0.5 mm, radius 24–57 mm under a 161 mm part —
so it needs the brim as well. Thin-wall detection is confirmed working: the
sliced G-code carries ~1,600 extruding moves per layer at mid-height, tapering at
the rim. Still check in the GUI preview that no sub-millimetre specks survived.

Bambu Studio flags **floating regions** on the bowl. Located layer by layer
against the layer below (±0.2 mm), they are two things, and neither is a reason
to stop or add supports:

- *The base, z 0.3–1.5 mm.* The foot ring is level — plane fit 0.10°, 0.28 mm
  spread — but uneven enough that the first layer catches only part of it, and
  the underside inside the foot rises about 1.5 mm toward the centre. Everything
  flagged here is under 1.5 mm above the plate; the worst outcome is a rough
  underside inside the foot. Do not reorient: the foot is already flat to 0.1°.
- *The rim, z 49.5–53 mm.* The top flares out into a bead, outer radius 74 →
  81 mm. The scanned rim is not parallel to the foot (0.57°, 1.8 mm), so the
  bead's lower edge starts as scattered islands of 0.06–17 mm², most under
  5 mm², at different heights around the circumference; each joins the wall
  within a layer or two. Expect a slightly rough underside to the rim.
  Supports are optional here, not needed: build-plate-only supports would stand
  ~50 mm tall around the bowl and touch only the bead's underside, trading droop
  for support-interface marks on the same surface, plus tall thin trees on a
  bed-slinger. If used, paint them onto the rim underside only. No support can
  help the base — the gaps there are under 1.5 mm.

A sagged underside inside the foot could touch the tray floor before the foot
does. It did — the bowl rocked about 1 mm (2026-09-17) — and the fix that
shipped is a flat floor inside the contact ring rather than the wider bore
first suggested here, because the foot band itself is uneven by up to 1.16 mm
and a bore would have left that under it. §8.1.

**`bowl_tray`.** Its socket is shape-keyed — cut from the bowl's real
out-of-round profile, not a circle — so the bowl seats at essentially one yaw.
Rotate it until it drops in rather than forcing it. That costs the measurement
nothing, since yaw is undefined for a rotationally symmetric class, and it is
what buys the tight clearance (§2b).

**Placing the marker cards.** Ground truth reads each card's id *and* its
rotation: `T_cam_object = T_cam_marker · MarkerToObject`, and the offset from
marker to object is applied in the marker's own frame. The detector's frame
(`marker_detector.cpp`, confirmed against the printed sheet) is +x to the right
and +y toward the top of the black square — the card's label is along its
bottom edge. So, looking down at a tray from its front edge:

- every card reads upright, label nearest you;
- ids go where `config/marker_trays.yaml` puts them, +x right, +y away from you.

| tray | front-left | front-right | back-left | back-right |
|---|---|---|---|---|
| `bowl` | 104 (−72, −72) | 105 (+72, −72) | 106 (−72, +72) | 107 (+72, +72) |
| `red_cube` | 100 (−57, 0), left end | 101 (+57, 0), right end | — | — |
| `mustard_bottle` | 102 (−80, 0), left end | 103 (+80, 0), right end | — | — |

A card in the wrong corner, or turned in its recess, is not a small error. The
bowl's markers sit 101.8 mm from the object centre, so a card a quarter-turn out
moves that marker's object estimate by **144 mm**, a half-turn or the diagonal
corner by **204 mm**. For the bowl and cube, turning *all* cards together is
harmless — both objects are symmetric and sit at the tray centre — so only
consistency matters. Set the tray on the rig with its front edge (104/105)
toward the cameras: marker visibility was ray-cast for that orientation.

`gt_from_trays` pools every marker on a tray into one average and refuses only
on too few observations, so a misplaced card still produces a row. It shows up
in the printed scatter and cross-camera figures — expect a few mm; tens of mm
means a card is wrong.

The bottle tray has one more constraint: its socket keys the bottle's
orientation (the footprint mismatches itself by up to 1.4 mm under a half-turn,
against 0.12–0.23 mm of clearance), and the bottle is not a symmetric class, so
that tray's front edge must match the socket, not be chosen freely. `[VERIFY]`
which end is which before gluing those two cards.

**Board jig.** `assets/jig/board_jig_corner.stl` and `board_jig_fence_stop.stl`,
already in millimetres. Seating gesture and bracket placement are in
`CAPTURE_PROTOCOL.md` §4.2c-ter.

Grey total: **368 g** of trays plus ~8 g of jig.

**Do not print the second object set yet** (§7). The proposal due Tuesday
2026-09-15 depends on no printed part; the parts are for the 09-30 presentation
and the 10-31 final.

---

## 5. Filament budget

| Spool | Committed by this plan | Remaining of 1 kg |
|---|---|---|
| red | 42 g | ~958 g |
| yellow | 171 g | ~829 g |
| blue | 41 g | ~959 g |
| grey | 368 g trays + ~8 g jig | ~624 g |

Every spool has ample room for the spare set and for printed distractors.

---

## 6. Definition of done for this plan

- [x] Marker sheets printed and a marker measures 40.0 mm (§2d, 2026-09-13)
- [x] `red_cube` printed in red, three axes callipered and recorded (§4, gate 1)
- [ ] Cube colour sampled against the band and recorded
- [ ] §3.1a three-condition detector comparison re-run on the correct-palette cube
- [ ] `mustard_bottle` printed in yellow
- [ ] `bowl` printed in blue, wall confirmed present in preview
- [ ] `red_cube_tray` and `mustard_bottle_tray` printed in grey, fit-tested,
      realised clearance measured and recorded
- [ ] Board jig printed and bracket positions taped
- [x] `bowl_tray` cutter fixed and regenerated — printed: [ ]

---

## 7. Deliberately not printing yet

**The second object set.** `RIG_BOM.md` budgets two, but no scene in
`CAPTURE_PROTOCOL.md` §5 uses two of a class — T2 is 3 targets plus 8
distractors — so the second set is a spare. Printing it before the first has
been callipered and shot risks baking a printer error in twice, and the bottle
alone is 958 layers of machine time that Monday's trays need more.

**Distractors.** The renderer draws distractors from the *same* palette as the
targets, which is what stopped the benchmark saturating. §6.3 says they need no
meshes, so household objects qualify and are free — but they must be in the six
palette colours, not arbitrary ones. Three to five physical is enough. White is
the natural spool for these (`#E7E7E7` is a neutral the renderer already
samples), with grey offcuts for `#898989`; `#BCB3AA` has no spool, so source
that one from the household. Print simple primitives from leftovers once the
critical parts are done. Low priority.

**The occluder stand.** `CAPTURE_PROTOCOL.md` §5 requires a stiff card on a
stand — a hand cannot hold a fixed occlusion for 300 pairs — in a colour that is
none of the three class colours *and* none of the three distractor neutrals. No
geometry exists for this anywhere in the repo; it is the last unauthored printed
part. Suggested split: the card itself from stiff off-palette stock (green or
magenta), which is cheaper, flatter and lighter than printing a slab, and the
stand printed in **black** — small footprint, off-palette, and the only job
black is fit for here. Keep the black area small: the rig is already 82–93%
darker than the training distribution and a large black slab pushes it further.

---

## 8. Acceptance test for the three sets

Two stages. The bench stage needs callipers only. The rig stage needs you for
the physical setup and about ten minutes of shooting; every number after that is
computed offline from the stills.

### 8.1 Bench — no rig

| check | target | measured |
|---|---|---|
| `mustard_bottle` height, base to cap | 191.41 mm | **191.0 mm** (−0.41 mm, −0.21%) — **pass** |
| `mustard_bottle` across the body | not a calliper check — see below | 95.44 × 57.07 mm, consistent with the mesh |
| `bowl` across the rim, any direction | 160.38–161.94 mm (rim is out of round) | **161 / 161 mm** — inside the range |
| `bowl` height, at the highest point of the rim | 55.01 mm (rim is uneven by 1.8 mm) | **55.0 mm** (−0.01 mm) — **pass** |
| bottle seats under its own weight, no rocking | yes | **no** — drops in, then tilts easily. Tray reprint, below |
| bowl seats under its own weight, no rocking, at its one yaw | yes | **no** — rocks about 1 mm. Tray reprint, below |
| every marker card flat, no lifted corner or edge | yes | **yes** |

Pass on dimensions: within 0.3%, the cube's worst axis — 0.6 mm on the bottle's
height, 0.5 mm on the bowl.

**Why the bottle's width and depth are not a pass/fail check.** This table first
gave 97.15 × 66.62 mm, which is the mesh's axis-aligned bounding box, not what
callipers read. The bottle's cross-section is turned relative to the mesh axes
and changes quickly with height, so jaws across its faces read the section's
*own* width at whatever height they sit:

| height above base | narrow width | long width |
|---|---|---|
| 6 mm | 57.01 mm | 94.68 mm |
| 12 mm | 57.40 mm | 97.28 mm |
| 20 mm | 56.53 mm | 97.79 mm |
| 60 mm | 50.22 mm | 83.48 mm |

The reading of 95.44 × 57.07 mm (2026-09-16) matches the section about 8 mm up,
and 1 mm of jaw height moves the long width by more than the 0.3% pass line, so
callipers cannot resolve print error here. The socket is the better check: it is
cut from this same profile with 0.12–0.23 mm of clearance, and the bottle seats
without rocking, which a base more than a few tenths out would not do.

Height is well defined — cap top to base — and passes.

The bowl's rim readings have the same limit in milder form: out-of-roundness
spreads the expected reading over 1.56 mm, three times the 0.48 mm pass line, so
"inside the range" is all a calliper can say. Its socket is the tightest of the
three (0.082–0.118 mm) and it seats, which is the stronger check.

**The bottle tilts and the bowl rocks: both trays reprinted with flat floors
(2026-09-17).** Each socket floor was cut from the object's own underside, so it
carried a raised dome matching the CAD dish exactly — 3.43 mm inside the
bottle's contact ring (radius 20.8–32.5 mm), 1.66 mm inside the bowl's foot
(37.4–44.5 mm), where the 30 mm bore removed only the middle of it. Both objects
were printed without support, their dishes sagged flatter than the CAD, and each
then balanced on its dome instead of standing on its ring. Against the CAD mesh
the seat was perfect, which is why no geometry check caught it.

`flat_floor_cutter()` in `author_marker_trays.py` now flattens the floor to the
object's lowest point inside the convex hull of its contact ring. Seat height is
still defined by that lowest point, so `config/marker_trays.yaml` is
**byte-identical**; the prism stays inside the ring, so the socket walls and the
measured clearances are untouched. Checked against the worst case, a dish sagged
completely flat:

| | old tray | new tray |
|---|---|---|
| bottle, flattened underside ∩ tray | **3507 mm³** | 15.5 mm³ |
| bowl, flattened underside ∩ tray | **1588 mm³** | 1.7 mm³ |
| highest floor inside the ring, above the seat plane — bottle | 3.43 mm | 0.18 mm |
| highest floor inside the ring, above the seat plane — bowl | 1.33 mm | 0.20 mm |

The residual 0.18–0.20 mm is the contact ring's own unevenness, which the ring's
0.2 mm definition leaves in place. Both objects still lift straight out. The
cube tray, whose base is flat, is unchanged, as are all marker sheets.

**Decision 2026-09-17: no reprints — the trays in use are the domed ones,
corrected by hand.** The flat-floor generator stays in the repo as the correct
design for any future print, so `assets/trays/` and `assets/print/` no longer
describe the two physical trays in use; `config/marker_trays.yaml` is identical
for both, so ground-truth arithmetic is unaffected.

- *Bottle — hollow the base back to its dish with a Dremel.* The target is the
  CAD dish the unsupported print lost: about 40 mm across, centred on the base,
  up to ~3.4 mm deep at the centre (1.7–3.3 mm within 10 mm of centre, falling
  to ~0.4 mm at 20 mm). **Do not touch the contact ring outside radius ~20 mm —
  it sets the seat height.** Going deeper than the dish inside the circle is
  harmless; breaking into the infill is expected and hidden in the tray. Rub
  pencil graphite on the tray's dome, seat the bottle, and grind where it
  marks, until the bottle no longer tilts. Low speed, light passes — PLA melts
  and gums at high RPM.
- *Bowl — do not grind the bowl.* Its base is a 0.99 mm shell and a 1.66 mm
  dish cannot be cut into it. The tray side is safe instead: widen the floor
  hole from 60 mm to about **74 mm** (radius 37 mm), which removes the dome band
  between the old hole and the foot (37.4–44.5 mm) without reaching the foot.
  Left as is, a 1 mm rock is roughly 0.5 mm of centre uncertainty, ~6% of the
  bowl's 16.2 mm ADD-S threshold.

**Seat-height check after shaping — the one test ground truth cannot do for
itself.** The markers never see the object, so an object sitting low in its
socket gives confident ground truth that is wrong by exactly that amount.
Calliper from the tray's top face to the object's highest point:

| object | measured height − socket depth | expected above tray top | measured |
|---|---|---|---|
| `red_cube` | 50.10 − 8.0 | **42.1 mm** | `[VERIFY]` |
| `mustard_bottle` (cap top) | 191.0 − 12.0 | **179.0 mm** | `[VERIFY]` |
| `bowl` (highest rim point) | 55.0 − 6.0 | **49.0 mm** | `[VERIFY]` |

Within ±0.3 mm passes. Low means the ring or floor was ground; high means it
is still standing on a dome. No ground truth for the bottle or bowl until both
seat without tilting and pass this.

**The bottle only goes in one way round — mark it.** Once it drops in, put a
pencil tick on the bottle's base and a matching tick on the tray rim, and always
re-seat it tick to tick. Whether that way round matches the CAD +x cannot be
told by eye — the cap sits within 1.1 mm of the axis, and above the base the
footprint differs from its own half-turn by at most 1.4 mm — and it cannot be
told by the pipeline either, for the same reason.
Its cost is bounded instead: a reversed bottle tray is **1.35 mm of ADD-S**
(7% of the 19.6 mm threshold), 64 mm of ADD. Record it as a known ceiling on the
bottle's ground truth; the headline metric is ADD-S.

### 8.2 Rig — ground truth checked against things that do not come from markers

Ground truth is only as good as the markers, so the useful tests compare it with
quantities that are known independently of them: heights from the CAD stack,
distances from the trays' own printed sizes, and one marker against another on
the same tray.

**Set up.** Re-register first (`calibrate reregister`, §A of `RIG_BOM.md`) —
this is a new session. If it refuses on the baseline gate the tripods moved
rather than the desk; that means a full recalibration, which changes the shared
field of view, so re-run the §4.2d projection and re-check the two footprints
below against it. Lights as for capture, TV off, desk clear. Every tray front
edge — the 104/105 side, cube and bottle labels readable — toward the
cameras.

Two shots, because all three trays side by side (596 mm) do not fit the
480 × 360 mm shared field of view (2026-09-20 evening extrinsics):

| shot | trays, left to right, touching, front edges on one straight edge | footprint |
|---|---|---|
| A | `red_cube` + `bowl` | 380 × 210 mm — 100 mm of width to spare |
| B | `bowl` + `mustard_bottle` | 426 × 210 mm — 54 mm to spare, centre it carefully |

Objects seated. For each shot, ~10 stills at 1920 × 1080 with nothing touched
between them (ENTER shoots, `q` finishes; the board corner count it prints will
read 0 and does not matter):

```bash
build/calibration/calibrate capture config/cameras.yaml config/calibration.yaml captures/gt_test_A
build/calibration/calibrate capture config/cameras.yaml config/calibration.yaml captures/gt_test_B
```

**Solve**, against a *copy* of an existing run database so nothing real is
written to:

```bash
cp runs/runtime/20260909_235652/metrics.sqlite captures/gt_test_A/scratch.sqlite
build/runtime/gt_from_trays captures/gt_test_A captures/gt_test_A/scratch.sqlite --stills --factory
```

**What to compare** — expected values are from CAD and the tray outlines, so they
share no error with the markers:

| check | expected | how | measured |
|---|---|---|---|
| cube centre height | **27.5 mm** | 10.5 mm plate + 17.0 mm `t_tray_object` | `[VERIFY]` |
| bowl centre height | **30.0 mm** | 8.5 + 21.504 | `[VERIFY]` |
| bottle centre height | **98.2 mm** | 14.5 + 83.704 | `[VERIFY]` |
| object up-axis vs world +z | < 1° | trays flat on a flat table | `[VERIFY]` |
| cube → bowl centre distance, shot A | **199.2 mm** | Δx 85 + 105, Δy 105 − 45 | `[VERIFY]` |
| bowl → bottle centre distance, shot B | **220.0 mm** | Δx 105 + 108, Δy 105 − 50 | `[VERIFY]` |
| yaw agreement between trays in one shot | < 1° | same straight edge | `[VERIFY]` |
| cross-camera disagreement, per tray | a few mm | printed by the tool | `[VERIFY]` |
| per-marker agreement within a tray | a few mm | one solve per marker, see below | `[VERIFY]` |

The expected distances carry the trays' own print error, about 0.3% or 0.6 mm.
Reading the results:

- **Tens of mm, or ~90°/180° of yaw** on one tray: a card in the wrong recess or
  turned in it (§4, placing the cards).
- **Heights all off by the same few mm, distances right:** the declared table
  plane or the registration, not the markers.
- **Distances off in proportion to their length:** scale — marker length or
  intrinsics.
- **Cross-camera disagreement much larger than the per-camera scatter:** a
  per-camera bias; the scatter alone would have hidden it (CAPTURE_PROTOCOL
  §3.1b).

`gt_from_trays` pools every marker on a tray and prints no per-marker breakdown.
Per-marker agreement is obtained without touching the tool: re-run the solve
with `--config` pointing at a copy of `config/` whose `marker_trays.yaml` keeps
one marker per tray, once per marker — four solves for the bowl, two each for
the others.

None of this is a scored result. It establishes that the yardstick is sound
before the ten scored scenes are shot against it.
