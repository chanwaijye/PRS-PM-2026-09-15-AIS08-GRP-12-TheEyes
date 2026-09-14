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
are in the bottom 3.5 mm, the base's own dish. The 97 × 67 mm footprint is
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

The one route by which the base could reach ground truth: a sagged underside
between the lightening bore (r 30 mm) and the foot contact (r 34.5–45.1 mm)
could touch the tray floor before the foot does. The bowl-tray fit test catches
it — the bowl would rock or sit high. If so, open the tray's `lighten` bore to
about 68 mm so only the foot rests on the floor; that changes no number in
`config/marker_trays.yaml`.

**`bowl_tray`.** Its socket is shape-keyed — cut from the bowl's real
out-of-round profile, not a circle — so the bowl seats at essentially one yaw.
Rotate it until it drops in rather than forcing it. That costs the measurement
nothing, since yaw is undefined for a rotationally symmetric class, and it is
what buys the tight clearance (§2b).

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
