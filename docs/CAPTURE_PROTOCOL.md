# Capture protocol — the occlusion ablation

How to record the data that produces the headline number, and how not to destroy
it while recording. Written 2026-09-08, before any capture has happened.

Scope: the occlusion ablation (MVP item 2, `CLAUDE.md`: *never cut*). Everything
here serves one sentence in the report — *multi-view fusion recovers objects a
single viewpoint loses* — and the protocol exists because that sentence is
provable only if the capture is set up to prove it.

Numbers below are sourced at the point of use. Nothing here has been executed.

---

## 1. Two recordings, never one

**The single most important rule in this document.**

Ground truth comes from the ArUco trays as
`T_cam_object = T_cam_marker · T_marker_tray · T_tray_object`
(`config/marker_trays.yaml`). The first term is measured from the image at
evaluation time; **the other two are exact by construction only while the object
is seated in its printed socket**. Lift an object out of its tray and its ground
truth ceases to exist — not degrades, ceases.

So a recording in which a human picks objects up cannot be scored, ever. Split
the session:

| | **Scored run** (§6) | **Demo run** (§7) |
|---|---|---|
| Objects | seated in trays, **never touched** | handled freely |
| Ground truth | frozen from the opening frames | none |
| Occlusion | card, repeatable | hand, whatever looks good |
| Produces | ADD-S per arm, the headline number | video |
| Report status | the measurement | an illustration, labelled as one |

You may reposition a *tray* with its object seated — the chain still holds. You
may not lift the object.

---

## 2. Why a static scored scene makes occlusion easy

Because nothing moves during the scored run, **ground truth is a constant**.
Solve it once from unoccluded opening frames, freeze it, and it stays valid for
every later pair no matter what is in front of the cameras.

This removes the trap the protocol was written to avoid. Ground truth is
vision-derived from the same two cameras, so per-frame GT under occlusion would
be self-defeating: block the object and you block its markers. Freezing GT first
means you can then occlude **as aggressively as you like, including both views at
once**, and still score every frame.

It also makes the EKF harmless. `config/fusion.yaml` runs a constant-velocity
model with `max_coast_s: 1.0`; on a static scene the velocity state is ~0, so an
occluded track coasts nowhere. On the demo run, where objects move, expect
visible drift when a hand covers a moving object — that is the filter being
honest, and it should not be smoothed away.

---

## 3. Prerequisites — blocking, in this order

Capture day is wasted if any of these is untrue.

1. **Light the scene, then clear the desk.** ~~cam_over must actually see
   things.~~ **Corrected 2026-09-09** — the earlier reading of this prerequisite
   was wrong in its diagnosis and backwards in its conclusion (§3.1a), and the
   larger effect is not clutter at all but exposure: 82–93% of every real frame
   falls below the training set's 1st percentile (`docs/STATUS.md`
   "Sim-to-real"). Light it, re-run `training/exposure_report.py` until the
   session is no longer flagged out of distribution, and only then judge the
   detector.
### 3.1a What the cam_over asymmetry actually is

The 232-vs-3 estimate split looked like a broken overhead camera and was written
up as the top blocker. Replaying the recorded session through the real detector
under three masking conditions says otherwise. All counts are over 150 pairs,
and every detection in every condition is a false positive except where noted.

| Condition | cam_front | cam_over | total | front/over |
|---|---|---|---|---|
| unmasked | 395 | 81 | 476 | 4.9× |
| everything outside the table blanked (backdrop proxy) | 330 | 36 | 366 | 9.2× |
| only the 480 × 340 shared workspace visible | **214** | **9** | 223 | 23.8× |

Three things follow, and two of them reverse the earlier conclusion.

**A backdrop is worth much less than expected.** Blanking everything off the
table removes only 23% of the false positives (476 → 366). The bed, the pillow
and the wall were not the main driver.

**cam_over is the camera that works; cam_front is the one that does not.** With
a clean view, **7 of cam_over's 9 detections are on the cube** (IoU 0.28–0.38,
scores 0.28–0.54). **cam_front's best IoU across all 214 of its detections is
0.000** — it never produces a box on the object at all. The earlier "zero on the
cube in both cameras" came from the unmasked run, where an IoU ≥ 0.3 gate
rejected cam_over's loose boxes; three of them clear it once the clutter is gone.

**The asymmetry is furniture, not optics.** Of cam_front's 214 survivors, 53 sit
on the fan in the top-left and 154 on the mousepad and hand at the bottom —
**97% on two removable objects**, and *none* in the centre cell where the cube
is. It is not a detection cap crowding the true positive out either:
`max_detections` is 64 and the observed maximum is 5 per frame.

**So: clear the desk.** Take the fan, the mousepad, the keyboard and the mug out
of the workspace before capture. It is free, it is worth more than a backdrop,
and nothing about cam_over needs fixing. Do the backdrop as well — 23% is still
23% — but not first.

Two caveats that bound this. Recall is still dreadful even for cam_over: it
detects the cube in 9 of 150 frames. And the object in this recording is the
**off-palette dark-brown test print**, brown-on-brown against the table, which is
why `--detector fixed` existed. Whether cam_front's total blindness survives a
correct-palette cube is untested and gated on filament (~09-12) — **re-run this
same three-condition comparison once the printed objects exist**, since it needs
no rig and no capture.


2. ~~The ArUco ground-truth tool must exist.~~ **BUILT 2026-09-09** —
   `build/runtime/gt_from_trays`, with `MarkerDetector` in the calibration
   module and the tray geometry in `runtime/marker_trays.hpp`. Validated against
   a synthetic session with a known object pose; see §3.1b, which is a capture
   requirement that came out of building it.

3.1b. **Ground truth must be captured at 1920×1080, not at the pipeline's
   848×480.** Measured, on synthetic renders of the real rig geometry with a
   known object pose:

   | Capture resolution | Marker size | Cameras contributing | Recovered pose error |
   |---|---|---|---|
   | 848 × 480 (pipeline) | 23–37 px | **cam_front only** | **18.4 mm** |
   | 1920 × 1080 (calibration) | 51–84 px | both | **2.1 mm** |

   An 18 mm ground-truth bias against an ADD-S threshold of ~8.6 mm for the cube
   would not have produced a wrong-looking result — it would have produced a
   confident wrong one. Worse, at 848 × 480 the tool's own pooled **scatter read
   2.1 mm while the pose was 18 mm wrong**: a systematic bias does not appear as
   spread, so the scatter alone cannot be trusted as an accuracy claim. The
   **cross-camera disagreement** is the figure that did not lie (3.0 mm at
   1080p), which is the second reason two cameras are worth having.

   `gt_from_trays` therefore defaults `--min-size` to 40 px and refuses the
   small-marker regime rather than quietly emitting confident nonsense.

   **On the day** (`--stills` shipped 2026-09-09): shoot the ground-truth window
   as 1920 × 1080 stills into `<dir>/<camera_id>/`, the layout
   `calibrate capture` already writes, then

   ```bash
   build/runtime/gt_from_trays <stills_dir> runs/runtime/<session>/metrics.sqlite \
       --stills --factory
   ```

   `--factory` reads each device's own colour intrinsics at the stills'
   resolution and **writes them to `<stills_dir>/intrinsics.yaml`**, so the solve
   can be repeated later with no camera attached; a sidecar already present is
   picked up automatically. Intrinsics are per-resolution and the tool **refuses
   a mismatch rather than rescaling** — passing the pipeline's 848 × 480 K
   against 1080p stills is exactly the 18 mm trap above, and it now fails loudly.

   Registering ground truth from high-resolution stills while the pipeline
   streams 848 × 480 is sound because the camera *pose* does not depend on
   sampling — the projection centre is physically fixed (measured half-FOV
   34.93° at 848 × 480 against 35.10° at 1920 × 1080, `device_registry.hpp`).
   Only the intrinsics are resolution-dependent, which is why the stills carry
   their own.

   Validated on a synthetic 1080p stills set with a chosen object pose:
   recovered to **2.1 mm**, depth maps beside the colour frames ignored rather
   than scanned as if they were images.
3. **The platform must be registered** — §4. Currently fiction.
4. **Trays printed and calliper-verified.** Filament ETA ~2026-09-12
   (`docs/STATUS.md`); worst-case socket clearance target 0.23 mm.
5. **Extrinsics still valid.** `config/extrinsics.yaml` holds real values from
   2026-09-20 (RMSE 0.28 / 0.33 px). This is a computer desk that gets leaned on:
   re-detect the board at its marked spot and confirm before every session
   (`docs/RIG_BOM.md` — a ~30 s check versus a 20 min recalibration). The marked
   spot is the **workspace centre**, taped; a table corner cannot serve, because
   no corner is in both frames (§4.2c). If the baseline gate trips, the tripods
   moved rather than the desk — shoot a burst of pairs before recalibrating and
   compare per-camera position spread, which is what separates a loose mount
   from a moved one (§4.2f).
6. **Workspace re-taped to the shared field of view** — **470 × 370 mm** on the
   2026-09-20 extrinsics, not the 600 × 360 in `RIG_BOM.md` (§4.2d). Objects
   outside it are single-view because they are out of frame, which is not the
   effect being measured. **Recompute after every recalibration**; it changed
   shape when the rig moved.

---

## 4. Registering the platform — MEASURED 2026-09-08

The runner injects a static `table_01` after fusion so that `(on X table_01)` can
be emitted at all (`config/scene.yaml`). Until now that object was invented, and
it was not merely unverified — it was **impossible**: half-extents
`[0.400, 0.300, 0.010]` declare an 800 × 600 mm surface, and the desk's free
depth is 440 mm.

### 4.1 The measurement

**Desk: 1180 × 750 × 23 mm.** Which resolves the discrepancy in `RIG_BOM.md`,
where the surface is quoted as "1180 × 440 mm free": the desk is 750 mm deep and
the back **310 mm is occupied by a 43-inch TV** (952 × 535 mm panel) and its
stand. 750 − 440 = 310 mm, so the two numbers were always consistent; one was
the desk and the other was what is left of it.

### 4.2 What goes in `scene.yaml`

**Model the real desk, not the taped workspace.** An earlier draft of this
document argued for the workspace rectangle, on the grounds that it is taped and
therefore identifiable. That argument was a workaround for not having measured
the desk, and it no longer applies. The desk is the truthful object, an object
can legitimately sit anywhere on it, and — decisively — the choice is free:
the top face is at z = 0 across the entire slab, so depth affects only XY
overlap, and the real footprint gets that right without a judgement call.

| | Value | Status |
|---|---|---|
| `extent` (half) | `[0.590, 0.375, 0.0115]` | measured |
| `pose_w.t` x | `0.0` | the workspace is centred in the 1180 mm width |
| `pose_w.t` z | `-0.0115` | half the 23 mm slab, so the top face lands on world z = 0 |
| `pose_w.t` y | `+0.155` | **[VERIFY] — derived, not measured** |

The y offset is the one thing the size does not give you. The world origin is the
**workspace centre on the tabletop**, with +y toward the back edge
(`config/calibration.yaml`, `world:`), and the workspace is *not* centred on the
desk in y — it sits in the free strip at the front. The value above assumes the
440 mm free strip is the front of the desk and the workspace is centred in it:
220 mm from origin to the front edge, 530 mm to the back, slab centre at
(530 − 220) / 2 = +155 mm.

**One tape measurement settles it:** origin (the board's pattern centre) to the
desk's **front** edge, call it `f`; then `t.y = 0.375 − f`. If `f` really is
220 mm, the committed value is already right.

Thickness is now a measurement rather than the modelling choice this document
previously called for — 23 mm, and comfortably thicker than any sub-surface pose
error we have seen, so an object sitting low cannot fall through the slab.

### 4.2b The TV is part of the rig now, whether or not it is modelled

It is not in `scene.yaml` — nothing needs it there — but it is **directly behind
the workspace**, roughly 950 mm wide and 535 mm tall, and both cameras look from
the front and slightly down, so it fills the upper part of both frames. That
makes it the scene's background, and three consequences follow:

- **It is a backdrop the detector was never trained on.** The synthetic set
  renders a floor and back wall precisely because a tabletop scene with no room
  is 69% background (`training/detector/replicator_dataset.py`). The real back
  wall is a television. Any detection degradation traceable to it is a
  sim-to-real finding (R3), not a bug — but it has to be *recorded* to be either.
- **A screen that is on is a large, bright, changing light source.** Changing
  content changes RGB auto-exposure frame to frame, and the panel is a broad
  specular surface facing the cameras. Whether it also degrades the active-stereo
  depth is **[VERIFY]** — test it, do not assume it either way; `RIG_BOM.md`
  already flags sunlight as broadband IR that degrades active stereo, and a
  backlight is a much weaker but non-zero version of the same question.
- **It is mass on a desk whose rigidity is already the known risk.** Extrinsics
  are camera→table-origin, and `RIG_BOM.md` calls any relative movement between
  the cameras and the tabletop the failure that "nothing downstream detects".
  Knocking the TV moves the desk.

**Protocol:** for a scored run the TV is **off, or displaying one static neutral
full-screen image**, and which of the two is recorded in the run notes. Never let
it change state mid-take. Running one deliberate T1 scene with the TV on and busy
is worth doing once, as a measured sim-to-real data point rather than an
accident.

### 4.2c The board is at the workspace centre, not a table corner — and it must stay there

Butting the ChArUco board into a physical table corner is an appealing datum: a
corner is repeatable and needs no tape. It does not work here, and the 2026-09-06
calibration did not do it. Both statements are measured, not assumed.

**What the committed extrinsics say.** Projecting the world origin into both
cameras through `config/extrinsics.yaml` and the factory intrinsics from the
`rig_fp` run puts it at pixel **(424, 249)** in cam_front and **(422, 241)** in
cam_over, against principal points of (428.6, 242.7) and (435.1, 248.2). The
origin sits within a few pixels of the optical axis of *both* cameras — exactly
what `docs/CALIBRATION_PROTOCOL.md` asks for when it says to put the workspace
centre on the red cross in both views. The recovered camera positions,
(−0.410, −0.355, +0.386) and (+0.339, −0.402, +0.392), land 41–84 mm from
`RIG_BOM.md`'s design values **measured from the workspace centre**. Had the
board been at a table corner, cam_over would have solved to roughly
(+0.959, −0.219) — 620 mm away. It did not.

**Why a corner cannot be the datum.** Projecting the four table corners into both
frames, *no corner is visible to both cameras*:

| Table corner | cam_front | cam_over |
|---|---|---|
| front-left | out | out |
| front-right | in | out |
| back-left | out | in |
| back-right | in | out |

`config/calibration.yaml` requires `min_shared_corners: 12` before extrinsics are
trusted. A board at a corner is seen by at most one camera, so it cannot produce
an extrinsic solve *or* a session re-registration. **The board's spot at the
workspace centre still has to be taped** — that is what
`CALIBRATION_PROTOCOL.md`'s "tape the four pattern corners" step is for, and
nothing about the desk's geometry removes the need.

Likely source of the confusion: `calibration.yaml` uses "corner" for the
*board's own origin corner* — the pattern's back-left corner, which is where
`T_world_board`'s (−0.1521, +0.1521) translation points. That is a corner of the
board, not a corner of the table.

### 4.2c-bis A 30 mm corner offset was evaluated and rejected

The follow-up proposal — seat the board 30 mm in from both edges at a table
corner, so the corner plus a spacer is the datum and no tape is needed — was
tested by projecting all 49 interior corners (7 × 7 at the measured 38.025 mm
pitch) into both frames:

| Board placement | cam_front | cam_over | **shared** | spread of shared corners |
|---|---|---|---|---|
| **current, centred on origin** | 49/49 | 49/49 | **49** | 228 × 228 mm (521 cm²) |
| front-left corner, +30/+30 | 18/49 | 49/49 | **18** | 114 × 190 mm (217 cm²) |
| front-right corner, +30/+30 | 49/49 | 20/49 | **20** | 114 × 190 mm (217 cm²) |
| back-left corner, +30/+30 | 9/49 | 49/49 | **9** | — **fails the gate** |
| back-right corner, +30/+30 | 49/49 | 16/49 | **16** | — |

`min_shared_corners: 12` is cleared by three of the four, but clearing a gate is
not the same as calibrating well:

- **Count falls to ~40%** of today's, and **spread to ~42%**. The solve is
  constrained by the geometric spread of the shared corners, not their number,
  and 18 corners clustered in a 114 × 190 mm strip along one board edge is a
  far weaker constraint than 49 spread over 228 × 228 mm.
- **Only one camera ever sees the whole board.** Both cameras sit in *front* of
  the table (y ≈ −0.36 to −0.40, x ≈ ∓0.41 / +0.34), so a corner puts the board
  nearly underneath one of them and far and oblique from the other.
- **Distance and obliquity both worsen** — mean corner distance rises from
  0.67 m to 0.73–0.98 m and incidence from 54° to 54–66°.
  `docs/CALIBRATION_PROTOCOL.md` already records a board at 884 mm returning
  *zero* corners from images that look perfectly good. The back corners sit at
  0.94–0.98 m, past that.
- The board also lands at the frame edge, where lens distortion is worst.

**The axis that is free is the front edge, not the corner.** Same projection,
board pushed to 30 mm from the **front** edge but left centred in x:

| front edge only, centred in x | 49/49 | 49/49 | **49** | 228 × 228 mm (521 cm²) |
|---|---|---|---|---|

Identical to today. Because both cameras are in front of the table, moving the
board *toward* them costs nothing; moving it *sideways* is what breaks it.

### 4.2c-ter So do not move the board — jig it

Even the free option is not worth taking, because **the board defines the world
frame**: moving it means a full recalibration, new `extrinsics.yaml`, and a fresh
`scene.yaml` table pose. It would also weaken the one independent check the rig
has — `config/calibration.yaml` chose this origin so the recovered camera
positions come out as `RIG_BOM.md`'s numbers directly, "which is what makes a
tape measure a valid check on the result".

And the gain is small: the board's front edge is **already only 58 mm** from the
table's front edge (workspace centre is 220 mm back, pattern half-width 162 mm).
The 30 mm proposal moves it 28 mm.

**Do this instead:** print a seating jig for the board's current spot.
`training/author_board_jig.py` generates it — `assets/jig/board_jig_corner.stl`
(4.98 cm³) and `assets/jig/board_jig_fence_stop.stl` (1.65 cm³), about 8 g of
filament together. Three contacts, no pocket: two fence pads 260 mm apart on the
+y edge fix y and yaw, one 15 mm stop on the −x edge fixes x. That is exactly the
three in-plane degrees of freedom — a pocket or a long side wall would be a
fourth constraint, and an over-constrained board either rattles or refuses to
seat, both silently. Walls are 5 mm against a 7.1 mm board, so it always lifts
straight out, and the generator refuses to emit a part whose material intrudes
into the board's own volume or that comes out as more than one shell.

**Seating gesture, which is part of the protocol:** lower the board flat, push it
toward +y until both fence pads touch, then toward −x until the stop touches.
Because the constraint is three points rather than a pocket, the seated pose is
exact when pushed and undefined when not.

Tape marks where the *brackets* go — each carries a witness notch for exactly
that — and the brackets, not the tape, position the board. A knocked bracket
seats the board confidently in the wrong place, so the re-registration check
stays mandatory.

**And note how little the jig has to achieve.** `docs/WORLD_FRAME.md` works
through what actually depends on the world origin: the answer is almost nothing
that is graded — camera↔camera extrinsics, ADD-S and the occlusion ablation are
all invariant to it. The board needs a *repeatable* spot, not a *known* one.

### 4.2d The shared field of view is much smaller than the taped workspace

Falling out of the same projection, and this one matters for every scene you lay
out. Sampling the tabletop plane at 5 mm and asking which points land inside
**both** 848 × 480 frames:

- largest origin-centred rectangle fully shared: **470 × 370 mm**
  (2026-09-20 extrinsics; it was 400 × 400 on the 2026-09-06 set and
  480 × 340 on the 2026-09-09 one, and the difference each time is the rig
  having moved — see §4.2e and §4.2f)
- **compare areas, not edge lengths, across re-runs.** The 09-20 figure comes
  from a search that maximises rectangle *area*; the same code on the 09-09
  extrinsics returns 470 × 350 where this document recorded 480 × 340. Those
  are 164,500 and 163,200 mm² — two near-equal rectangles from the same mask,
  not a changed measurement
- the taped workspace is **600 × 360 mm**
- the workspace's front-left corner (−0.300, −0.180) projects to y = 546 in
  cam_front, below a 480-row frame — it is **outside cam_front entirely**

`RIG_BOM.md`'s "horizontal coverage 981 mm (need 600)" is one camera's extent.
The quantity that governs a two-view experiment is the *intersection* of two
views ~80° apart, and that is 470 mm wide by 370 mm deep, not 981.

**Consequence for the ablation, and it is not small.** An object in the outer
band of the taped rectangle is single-view *by construction* — not occluded,
just out of frame. Scoring that as occlusion would manufacture exactly the result
the experiment is supposed to test. So:

- **Re-tape the workspace to the measured shared region** (470 × 370 mm centred
  on the origin on the current extrinsics, or whatever a re-run of this
  projection gives after any tripod change), and keep every object inside it in
  every scene.
- Re-run the projection after *any* mechanical change. It is arithmetic over
  `extrinsics.yaml` plus the intrinsics already logged in every
  `metrics.sqlite`; it needs no rig and no capture.
- Caveat: computed at z = 0 for the current tripod placement. Objects have
  height, so an object's top is not guaranteed shared just because its base is.
  Treat the figure as an upper bound and leave margin.

### 4.2e The rig moved between 09-06 and 09-09, and the gauge-invariant check caught it

The 2026-09-09 recalibration passes — round trip mean 0.12 / p95 0.32 / max
0.86 mm against a 5 mm gate, 49 shared corners — but it is **not** a small
correction to the previous one:

| | 2026-09-06 | 2026-09-09 | design |
|---|---|---|---|
| camera-to-camera baseline | 749.9 mm | **706.7 mm** | 737 |
| relative rotation | 81.96° | **76.77°** | 80 |
| cam_front reprojection RMSE | 0.393 px | **0.467 px** | gate 0.50 |
| cam_over reprojection RMSE | 0.326 px | **0.406 px** | gate 0.50 |

Baseline and relative rotation are **gauge-invariant** — they cannot change with
where the board was placed (`docs/WORLD_FRAME.md` §4a). With a 0.12 mm round
trip the relative pose is pinned to well under a millimetre, so a 43 mm baseline
shift is not solve noise: **the cameras physically moved.** That is the failure
`RIG_BOM.md` says nothing downstream detects, and the gauge-invariant check
found it in seconds, from the committed file, with no rig.

Two consequences carried into this document: the shared field of view changed
shape (§4.2d), and the cameras are now *less* separated than designed — 76.8°
against 80°, baseline 30 mm under — which slightly weakens the viewpoint
diversity the whole multi-view claim rests on. If the tripods are touched again,
move them apart.

One caution: cam_front now sits **0.033 px** below the 0.50 px gate that fusion
refuses to start above, and both cameras regressed. The round trip says the
geometry is trustworthy, so this is not blocking — but a cleaner register shot
(braced board, checked focus) would buy back headroom before the next
recalibration fails outright.

### 4.2f The rig moved again on 09-20 — a slipping mount, not a moved desk

Re-registration was attempted first and **refused**: baseline rigidity came back
7.7–43.8 mm / 1.3–3.0° across repeated shots, against a 3 mm / 0.25° gate. That
verdict is the check working. A desk that moves cannot change the baseline, so
the tripods were what moved, and re-registration cannot recover that.

What the gate could not say is *why*. Eight registration pairs shot back to back
with nothing touched settled it:

| | cam_front | cam_over |
|---|---|---|
| solved position spread | 11.4 mm in x, 14.5 mm in y | 0.2 mm |
| reprojection RMSE | 0.39–0.91 px, 6 of 8 over gate | 0.33 px, steady |
| board sharpness | 322–362, flat | 273–315, flat |

cam_front's mount was slipping. Flat sharpness rules out motion blur, and
cam_over repeating to 0.2 mm proves the board was stationary — the world frame
is glued to the board, so a board that moved would have dragged *both* cameras'
poses with it. **A burst of pairs is the cheapest instrument here: one shot
cannot tell a loose mount from a moved one, and the gate reports both the same
way.**

Fixed mechanically — head locked, USB cable looped to the tripod leg so it stops
torquing the camera — after which both tripods were moved out toward the BOM
standoff. The result is the closest this rig has been to design:

| | 2026-09-09 | 2026-09-20 | design |
|---|---|---|---|
| camera-to-camera baseline | 706.7 mm | **738.1 mm** | 737 |
| relative rotation | 76.77° | **79.19°** | 80 |
| standoff | 663 mm | **678 mm** | 700 |
| camera heights | 385 / 392 mm | **396 / 398 mm** | 402 |
| round trip mean / p95 / max | 0.12 / 0.32 / 0.86 mm | **0.08 / 0.17 / 0.34 mm** | < 5 mm |

This supersedes §4.2e's closing advice to move the tripods apart — they were,
and viewpoint diversity is now 79.19° against a design 80°. It also supersedes
the 0.033 px headroom caution above.

**Still open, and it is cam_front again.** Its reprojection is bimodal: across
the final six shots, 0.750, 0.280, 0.331, 0.693, 0.308, 0.368 px — two of six
over the 0.50 px gate — while cam_over held 0.327–0.346. Position repeatability
is sub-millimetre throughout, so this is image quality on that unit rather than
the mount. Expect to shoot several registration pairs and keep one that clears
the gate; the committed set is the 0.280 px frame. Cause **[VERIFY]**.

### 4.3 Recognise it — cross-check the plane from depth

A tape measure produces a number nobody can audit later. The rig can check it
itself, and should:

- `FitPlane` already exists and is tested
  (`SystemCode/camera/src/depth_noise.cpp`, `test_depth_noise.cpp`), returning
  `rms_error_m` and a `valid` flag. It is what `depth_characterise` uses to
  measure depth RMS against a flat wall.
- Point it at the empty workspace instead of a wall: fit a plane to the depth
  points inside the taped rectangle, in each view, transform to world through
  `T_world_cam`, and compare.

Three things fall out, all checkable:

| Quantity | Expectation | Meaning if it fails |
|---|---|---|
| Fitted plane's world z | **0.000 m** | the world frame is not on the tabletop; every absolute height is biased |
| Plane normal vs world +z | within a couple of degrees | the board's declared pose is rotated relative to the real surface |
| Plane-fit RMS | ~2–4 mm at this standoff (measured depth noise, `docs/STATUS.md`; ~2.2 mm predicted at 0.70 m in `RIG_BOM.md`) | not a flat empty surface, or depth is degraded |

This is the "recognise the platform" step, and it is worth building as a small
tool (`table_probe`, or a `--table` mode on `depth_characterise`) because it
converts a hand-measured constant into a **gate**: refuse to capture when the
declared platform and the perceived platform disagree. Same spirit as the
existing startup checks, which already refuse to run on mismatched serials, view
order, or extrinsics past the reprojection gate.

### 4.4 Why this is not cosmetic

- `(on ?a ?b)` needs XY footprint overlap ≥ `on.min_overlap_ratio: 0.25` **and**
  a vertical gap within `on.surface_tol_m: 0.015` (`config/predicates.yaml`).
  The footprint is now the measured desk, so x/y is right; a table at the wrong
  z would break `on` outright, which is why §4.3 gates z rather than trusting it.
- The 15 mm surface tolerance is not large next to the real error we have seen:
  the 2026-09-06 cube read **z = 29.7 mm against a true 25 mm — 5 mm high**
  (one observation, `docs/STATUS.md`). A third of the tolerance is already spent
  by pose error, so none of it should be wasted on a wrong table.
- The `(in red_cube_01 table_01)` bug of 2026-09-06 — 26 false facts — was
  caused by the table's *thickness* swallowing a low pose. Default-deny on
  container classes fixed the symptom; a correctly modelled slab is what keeps
  the fix from being the only thing standing between us and it.

---

## 5. Scene layout and the occluder

Tiers are the proposal's (§6.4): **10 scenes, 300 synchronised pairs each.**

| Tier | Scenes | Physical arrangement |
|---|---|---|
| T1 isolated | 2 | one object, centre of the workspace, clean line of sight from both cameras |

| T2 clutter | 3 | 3 targets + 8 distractors, no target occluded |
| T3 partial | 3 | occluder covers **part** of one target from one view |
| T4 full | 2 | occluder covers the target almost entirely from one view |

**Use a card, not a hand, for the scored run.** A stiff card on a stand holds a
fixed occlusion for 300 pairs; a hand does not, and T3's "partially occluded"
band is meaningless if the fraction drifts while recording. Give the card a
colour that is none of the three class colours and none of the distractor
neutrals, so it can never be mistaken for a target and can be segmented later if
a visible-fraction figure is ever wanted.

Occlude **cam_over** by preference — it is the view whose loss the fusion claim
is about — but only once prerequisite §3.1 is satisfied and cam_over is
contributing properly. Record at least one scene occluding the other way, or the
result reads as a property of one camera.

---

## 6. The scored run

Per scene:

1. Set the scene. Objects seated in trays. Nothing else on the desk that can be
   knocked. Blackout or capture at night — sunlight is broadband IR and degrades
   active stereo (`docs/RIG_BOM.md`). **TV off, or one static neutral
   full-screen image; record which in the run notes** (§4.2b).
2. Confirm the board still resolves at its marked spot (§3.5).
3. **Occluder out of the way.** Record ~30 unoccluded pairs.
4. **Freeze ground truth** from those pairs: solve ArUco per object, average over
   the 30, write once into the `ground_truth` table. Record the per-object
   spread across those 30 solves — it is the ground truth's own noise floor, and
   an ADD-S smaller than it means nothing.
5. Place the occluder. **Do not touch the objects.**
6. Record 300 pairs: `the_eyes_run live --record <scene> --show`.
7. Repeat 1–6 per scene. Ten scenes.

Then, entirely offline and with no rig:

8. `ablation_from_log` over each `metrics.sqlite`, eight arms —
   {cam_front, cam_over, naive, gated} × {raw, EKF}. The claim is the **policy
   delta at a fixed filter setting** (`gated − naive`), never a comparison
   across filter settings; that confound was found on 2026-09-05 and fixed on
   09-06 and must not reappear in the report.

Everything after step 7 is replayable, so a bad analysis costs no capture time.

---

## 7. The demo run

Separate session, separate directory, no ground truth, and it is never scored.

Free-form: hands moving objects, hands crossing the cameras, objects lifted out
of trays entirely. Occlude both views at once if it looks good. Run it with the
console (`docs/VIZ_PLAN.md`) so the live A/B counter — `front N | over M |
fused K` — is on screen while a hand covers a camera. That single gesture is the
demo; the twin, if it exists by then, is the picture on top of it.

Two things this run will show, and both should be left visible: detector flicker
on real clutter, and EKF drift when a hand covers a moving object.

---

## 8. Occlusion severity — measure it, do not invent it

Do not put a "% visible" figure in the report unless something computes it.
`metrics.sqlite`'s `frame_views` table already logs, per camera per pair:
`n_detections`, `n_estimates`, `dropped_low_score`, `declined_by_estimator`,
`dropped_roi_too_small`. Those are the occlusion severity axis — already
recorded, already honest, and they are what actually degrades as the card
advances. A true silhouette-coverage percentage would need the occluder
segmented and the mesh projected at GT pose; that is optional work, and it is
not required for the claim.

---

## 9. Failure list — what ruins a scored run

- Touching an object mid-recording. Ground truth is gone from that frame on.
- Leaning on the desk. Extrinsics are camera→table-origin; any relative movement
  invalidates them and **nothing downstream detects it**.
- Occluding both views *before* ground truth is frozen.
- Moving the occluder during a take — the tier is then neither T3 nor T4.
- Placing an object outside the shared region (§4.2d — 470 × 370 mm on the
  current extrinsics). It is then single-view because it is out of frame, and
  the ablation scores an occlusion that never happened.
- Letting the TV change what it displays mid-take, or bumping it. It is the
  scene's whole backdrop and it is mass on the desk the extrinsics are tied to.
- Recording T3/T4 before T1 for that object. T1 is the single-view accuracy floor
  and a fusion gain is uninterpretable without it (proposal §6.4).
- Changing any config between scenes. `git_rev` and `config_dir` go into the
  `runs` table per run; keep them constant across the suite or the arms are not
  comparable.

---

## 10. Acceptance criteria

Binary, per `AGENTS.md` rule 3.

1. Every scored session's `metrics.sqlite` has a non-empty `ground_truth` table,
   and the per-object spread of the 30 opening solves is recorded alongside.
2. The platform gate (§4.3) passes in both views: fitted plane z within
   **[VERIFY] tolerance, to be set from the first real fit** of 0.000 m, normal
   within a few degrees of world +z, RMS consistent with 2–4 mm depth noise.
3. `config/scene.yaml` carries the measured desk (1180 × 750 × 23 mm, committed
   2026-09-08) and **no remaining `[VERIFY]` marker** — which today means the one
   open item is `pose_w.t.y`, resolved by the single front-edge measurement in
   §4.2.
4. cam_over's unoccluded estimate count is within a factor of two of cam_front's
   on a T1 scene. Below that, §3.1 is not satisfied and no ablation is run.
4b. **Every object was in frame.** For every scored pair, each non-static
   `ObjectState.pose_w` projects inside both 848 × 480 frames. Computable from
   `metrics.sqlite` alone — `objects` for the poses, `runs.cameras_json` for the
   intrinsics, `extrinsics.yaml` for the poses of the cameras — so it is a
   post-hoc check that costs no capture time and invalidates any scene that
   fails it (§4.2d).
5. `ablation_from_log` produces all eight arms for all ten scenes, and the
   reported claim is a `gated − naive` delta at one filter setting.
6. The demo run is stored in a separate directory and no number in the report is
   traced to it.

---

## 11. Open items

- ~~Workspace rectangle~~ — **done 2026-09-08**: desk measured 1180 × 750 ×
  23 mm and committed to `config/scene.yaml`.
- **[VERIFY]** `pose_w.t.y` for `table_01`. Derived as +0.155 m on the assumption
  that the 440 mm free strip is the front of the desk; one tape measurement from
  the board's pattern centre to the desk's front edge settles it (§4.2).
- **[VERIFY]** whether the TV's backlight measurably degrades the active-stereo
  depth, and how much its content changes RGB exposure. Test it; do not assume
  it either way (§4.2b).
- **[VERIFY]** plane-fit agreement tolerance for the §4.3 gate — set it from the
  first real fit, not from a guess.
- The shared region (§4.2d) is computed at z = 0 from the current extrinsics and
  the factory intrinsics. **Re-run it after any tripod or mount change** — it
  went from 400 × 400 to 480 × 340 mm when the rig moved between 09-06 and
  09-09, and to 470 × 370 mm after the 09-20 remount — and treat it as an upper bound, since an object's top is not shared
  just because its base is. Worth folding into the §4.3 platform tool so one
  command reports the plane fit and the shared footprint together.
- The visible-fraction figure for T3's "25–75%" band is unresolved (§8). Either
  compute it properly or state the tier qualitatively and let the `frame_views`
  counters carry the severity. Do not report a number nothing computed.
- ~~Table slab thickness is a modelling choice~~ — **superseded**: measured at
  23 mm, so it is a measurement now and should be quoted as one.
