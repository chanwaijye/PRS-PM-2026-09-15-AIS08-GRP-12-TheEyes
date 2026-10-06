# October plan — finish the rig work before 11 Oct

Written 2026-10-01, **rewritten 2026-10-04** after the first rehearsal on the
rig. Away in Taiwan for work **Sun 11 – Sat 24 Oct**; the rig stays home. Final
submission is **Sat 31 Oct** (`README.md`).

**How the plan works.** The rig only has to produce raw recordings, 1920×1080
ground-truth stills and photos. Replay is bit-exact, so the detector,
FoundationPose, fusion, the ground-truth solve and every ablation arm rerun from
disk. What must be right *at capture time* is short: the rig registered and not
moved, the scene clean, trays flat on the table, the tier as declared.

---

## Where things stand — Sun 4 Oct, end of day

Nothing was shot 1–3 Oct. On 4 Oct the afternoon went on prerequisites and a
rehearsal (daylight, TV on — **none of it is a scored result**):

- **Rig recalibrated** (it had drifted 10.7 mm in two idle weeks) and
  re-registered once more after clearing the desk moved a camera.
- **Detector works on the printed objects with no fine-tuning**: cam_front finds
  all three on 150/150 pairs.
- **All three objects re-scored with the fixes** on the afternoon's
  three-object recording (TV on, so phantoms present): fused ADD-S cube 4.0 mm
  (threshold 8.7), bottle 4.3 mm (19.6), bowl 5.7 mm (16.2); pooled 4.7 mm with
  99.1% inside, against 10.9 mm and 77.8% for the naive baseline.
- **Capture flow works end to end**, driven remotely scene by scene: cube alone,
  cube fully hidden from cam_over, cube half hidden. Measured tiers 100% / 0% /
  48% visible, all as staged.
- **Ground truth rebuilt on the table plane**: cross-camera disagreement on the
  cube 9.6 → 0.6 mm.
- **cam_front's depth corrected** (it read 1.0% short), and **fusion made
  symmetry-aware** (the cube's equivalent orientations). With all three, cube
  alone: cam_front 2.7 mm, cam_over 4.9 mm, gate + EKF 1.9 mm ADD-S with 100%
  inside the 8.6 mm threshold. Fully hidden from cam_over: cam_over finds it on
  0/300 pairs, fused output on 300/300 at 2.0 mm. That afternoon's first
  numbers, against the old ground truth and raw depth, were 13.7 / 7.4 mm, with
  gate + EKF at 13.2 mm.

**Found, not yet fixed** (all replay-side — none blocks capture):

| # | Finding | Fix |
|---|---|---|
| 1 | ~~Depth is biased per camera~~ **FIXED 2026-10-04** for cam_front (reads 1.0% short; ×1.0102 from `depth_tare`): its cube ADD-S 7.6 → 2.7 mm. cam_over's error (0 to +6.5 mm, growing with range) is not a scale and is left alone | Re-fit from each day's empty-table recording |
| 2 | ~~Gate + EKF (the `fused` arm) is worse than the gate alone on the cube~~ **FIXED 2026-10-04**: the mesh registry names each symmetry group and fusion compares orientations with it removed. Cube alone, gate + EKF: 8.5 → 1.9 mm, 58% → 100% inside the threshold | — |
| 3 | The grey tray is detected as a `bowl` on every pair, giving a false `(on red_cube bowl)` | Fine-tune the detector on real frames with trays in them |
| 4 | The bowl's centre is 12–15 mm off, toward the camera (cam_front's depth inside it reads 2–3 cm short). **Re-measured with all the fixes:** by ADD-S it is 5.3–6.3 mm against a 16.2 mm threshold, inside on 150/150 pairs in both views — rotational symmetry and a thin shell hide a centre error from that metric | Usable as a target by ADD-S; its centre error will show in predicates that depend on position. Cause not established |
| 5 | Half-hidden cube: cam_over detects it on 286/300 pairs and every pose is wrong — the visible half has no depth | A finding to report; the gate keeps it out of the fused pose |
| 6 | Clutter is detected: TV content, a red ring on a cable, the card's wooden stand | TV off, desk clear, stand wrapped |

---

## What is left to capture — about 2.5 hours at the rig

The cube set is done (below). The cube scenes were first planned for the night
of Sun 4 Oct and **aborted at 22:00 when the room lighting failed**; they were
shot on Mon 5 Oct once it came back.

The times below are from the 4 Oct rehearsal's pace, not from a guess: driven
from the keyboard with `capture_day.sh scene --stage stills|record`, a scene
took about 5 minutes, nearly all of it moving things on the desk (the stills
and the 300-pair recording are about a minute of machine time).

| Capture | Scenes | Time |
|---|---|---|
| ~~**Cube set** — alone; hidden and half hidden from each camera~~ **done 5 Oct** (it took about 75 min, with a USB fault and a re-registration in it) | 6 | — |
| **Bottle** — alone; half hidden | 2 | 15 min |
| **Clutter (T2)** — three trays + 8 distractors, ~10 min each to arrange | 3 | 30–35 min |
| **Training layouts** for the detector, including empty trays (finding 3) | 10–12 | 35–40 min |
| **Demo footage** for the video | — | 30 min |
| Start and end of each session (board on, re-register, empty table; final check) | per session | 12 min |

The first three rows are the proposal's ten scored scenes (T1 ×2, T2 ×3, T3 ×3,
T4 ×2). On top: about an hour of preparation and an hour of slack for reshoots.

**Preparation, before any scored scene:**

- [x] **Lighting** — the room light works again as of 5 Oct, about 17:23.
      Daylight alone is NOT a fallback: on 5 Oct it left cam_over blind to the
      cube (frame mean 28), and a card casts its shadow on the target
- [ ] Wrap the occluder's wooden stand (it is detected as a cube) — 5 min
- [ ] Bottle seats flat in its tray; seat height 179.0 mm ±0.3 (`PRINT_PLAN.md` §8.1) — 20–40 min if not done
- [ ] Eight distractors in the palette neutrals — 15–30 min

**Every session:** TV off, mouse and keyboard off the desk, nobody touches the
tripods or the two camera cables, and `capture_day.sh begin` first. Afterwards
`capture_day.sh end`, then copy `captures/` to an external drive — it is
gitignored and exists only on this machine.

## The cube set — SHOT AND SCORED, Mon 5 Oct

The first scored data. Six scenes under the room light (which came back on at
about 17:23), 300 pairs each, none dropped; scenes in `captures/scored/20261005/`,
results in `runs/scored/20261005/`. ADD-S in mm, with the share of pairs inside
10% of the cube's diameter (8.7 mm), misses counted against it:

| scene | hidden view sees | cam_front | cam_over | naive | gate | gate + EKF |
|---|---|---|---|---|---|---|
| `t1_cube` — alone | — | 2.78 · 100% | 3.32 · 100% | 22.34 · 0% | 1.89 · 100% | **1.68 · 100%** |
| `t1_cube_b` — alone, second placement | — | 3.13 · 100% | 2.74 · 100% | 22.13 · 0% | 1.84 · 100% | **1.70 · 100%** |
| `t4_cube_over` — hidden from cam_over | 0% | 3.02 · 100% | found on 0/300 | 3.02 · 100% | 3.02 · 100% | **2.97 · 100%** |
| `t3_cube_over` — half hidden from cam_over | 44%, 5% with depth | 2.34 · 100% | matched on 0/300 | 2.34 · 100% | 2.34 · 100% | **2.26 · 100%** |
| `t4_cube_front` — hidden from cam_front | 0% | found on 0/300 | 2.04 · 100% | 2.04 · 100% | 2.04 · 100% | **1.88 · 100%** |
| `t3_cube_front` — half hidden from cam_front | 60%, 59% with depth | 4.55 · 100% | 3.01 · 85% | 22.71 · 15% | 2.32 · 100% | **2.02 · 100%** |

Read with the session's conditions, all recorded in each scene's `notes.yaml`:

- **Ground truth agrees across cameras to 0.6 mm** in every scene, and every
  measured tier matches what was staged.
- **Two registrations.** cam_front's USB host controller hung at 18:13; the
  cable was replugged and the camera moved 2.5 px. `t1_cube` is on the 17:27
  registration; `t4_cube_over` and `t3_cube_over` have stills from before the
  move and recordings from after, so their ground truth is solved on 17:27 and
  their recordings lifted with 18:32; the rest are on 18:32. The closing
  registration at 18:44 was 0.16 mm from 18:32. Each scene now carries the
  calibration it was shot on (`extrinsics_at_stills.yaml`,
  `extrinsics_at_recording.yaml`, `depth_correction.yaml`, named in its
  `notes.yaml`) and `capture_day.sh process` uses them; reprocessing all six
  that way, with the brightness floor on, reproduced the table above exactly.
- **The light changed over the session**, from room light plus fading daylight
  to room light alone. Arms are compared within a scene, where it is constant.
- **The monitor was blanked** for every still and recording.

**What these six scenes do not show, and must be said with them:**

- The tray is detected as a `bowl` on every pair of every scene (finding 3), and
  as first processed every scene graph carried a false `(on red_cube_01 bowl_01)`.
- In both half-hidden scenes fusion holds a **second cube track** for most of
  the run. ADD-S does not see it; predicate accuracy does.
- Both are examined below, with what was done: the detector and fusion still
  produce them, and the scene graph no longer reports them.
- A cube is one object and a symmetric one; the naive baseline's 22 mm is the
  symmetry, not occlusion.

**Also measured that afternoon, in the dim daylight before the light came on**
(`dim_t1_cube`, not part of the scored set): at a frame mean of 28, cam_over
found the unoccluded cube on 0 of 300 pairs; at 85 it found it on 45 of 45.
Raising the colour gain from 64 to 96 or 128 under the room light washes the
picture out and loses the cube in both views. So: gain stays at 64, and a
session should not start until cam_over's frame mean is above about 40.

### The objects that are not there (examined 5 Oct; support check built the same night)

The scene graph of every scored scene holds more than the one object on the
table. Per cause, from the `objects`, `detections` and `estimates` tables
(`scripts/scene_graph_audit.py 20261005` prints the tracks, their support and
each view's range error):

- **A half-hidden view puts the object at the occluder's range**
  (`t3_cube_over`, 300 of 300 pairs). cam_over detects the visible half of the
  cube (score 0.56) and FoundationPose places it 171 mm from the truth: 170 mm
  nearer the camera along its own line of sight, 15 mm off that line. The
  visible half lies in the stereo shadow beside the card, so only 30% of the
  detection box has depth at all and it reads 496 mm where the cube's centre is
  685 mm deep — the card's depth, not the cube's. 171 mm is past the 150 mm
  association radius, so the estimate becomes its own cluster and its own
  track. **Its pose score is 0.86 against 0.64 for cam_front's correct one**: had
  it landed inside the radius, the gate would have kept the wrong view. That is
  the 2026-09-05 review's point that the pose scores do not rank the views,
  with a number on it.
- **The detector reads the card's red print as a cube** (`t3_cube_front`, 255 of
  300 pairs, score 0.36), and FoundationPose fits a cube to the card's surface
  there with score 0.90: a cube 178 mm from the real one and 114 mm above the
  table. cam_front's estimate of the real, half-hidden cube is fine (4.5 mm).
  A higher detector threshold would not separate them — the real cube scores
  0.43 in cam_over in the same scene.
- **Stray low-score detections** (0.26–0.36) at the image edge: 74 pairs in
  `t3_cube_over`, single pairs elsewhere, placed below the table or 1.3 m away.
- **The tray as a `bowl`** on every pair (finding 3), 18–62 mm *below* the table.

Counted over the six cube scenes and `t1_bottle`: the real object is in the
scene graph on 2,100 object-pairs and something that is not there on 2,771.
**All 2,100 real ones rest on something (`on` the table); 1 of the 2,771 others
does.** They float, or sit under the table. So the fix is in the scene-graph
bridge, not the detector: **an object that rests on nothing is not reported**
(`support:` in `config/predicates.yaml`; the rule is in `INTERFACES.md` §6 and
`SystemCode/scenegraph/README.md`). Built and reprocessed the same night:

| seven scenes, 300 pairs each | before | with the support check |
|---|---|---|
| the real object, in the scene graph | 2,100 of 2,100 object-pairs | 2,100 of 2,100 |
| objects that are not there, in the scene graph | 2,771 | **1** |
| facts naming an object that is not there | 5,543 of 10,690 | **3 of 5,150** |
| ADD-S tables, six cube scenes | — | identical |

`t3_cube_over` now reads, on every pair, `(clear red_cube_01)`,
`(occluded_from red_cube_01 cam_over)`, `(on red_cube_01 table_01)` and nothing
else. The objects are still in the log (`objects.supported` = 0): that fusion
tracked them is a finding, and the false `(on red_cube_01 bowl_01)` went with
the bowl.

**What this does not yet show.** The seven scenes are the ones that suggested
the rule and each holds one real object. The only multi-object recording is the
4 Oct rehearsal (TV on, not scored): read off its facts, all 450 real
object-pairs stay and 101 of 115 false ones go — the 14 left are a false cube
that landed at table height, which this rule cannot catch. The clutter scenes
are the test. `ObjectState` is unchanged; whether PM4 needs a per-object flag in
it is an open question, not a decision made.

Still worth doing at the rig: **the card gets a plain, non-red face** (and its
wooden stand wrapped) for every scene still to shoot — a false cube on the card
is removed from the scene graph, but it is still a wasted pose estimate and a
second track in fusion.

## The bottle, attempted the same night — one scene now usable

After the cube set the tripods were knocked (recalibrated at 19:11) and the
daylight went. Under the room light alone:

- **19:17, `t1_bottle`** (gain 64, frame mean 48 / 23): cam_front finds the
  bottle on 299/300 pairs, cam_over on 0.
- **Camera gain does not fix it.** At 128 cam_over found the bottle on 45/45
  pairs in one test and 7/300 a minute later, with its frame mean 2 lower.
- **Brightening the recorded picture does.** The same gain-64 recording,
  multiplied by 2, 3 or 4 before the detector: 60/60 in both views, scores
  0.89–0.95. So `t1_bottle` is usable once that is in the pipeline, and
  recordings can stay at the cube set's camera settings.
- **19:45, half-hidden bottle** (set aside as `dim_t3_bottle_over`): the room
  had dimmed to 22 / 10, and there even ×6 finds the fully visible bottle on
  only 41/45 pairs at 0.48. The light was not stable enough to go on.

**The brightness floor is in the pipeline** (`brightness:` in
`config/runtime.yaml`; `SystemCode/runtime/README.md`, "Brightness floor"): a
frame whose mean is under 45 is scaled up to it for the detector only, the
recording untouched. It is a floor because brighter loses the cube — at a
target of 60 cam_over drops it on 7 of 60 pairs, at 100 on 58. Reprocessed
with it, 300 pairs each:

| scene | cam_front | cam_over | gate + EKF |
|---|---|---|---|
| `t1_bottle` (cam_over brightened ×1.9) | 2.85 mm · 99.7% | 3.21 mm · 100% | **2.51 mm · 100%** |
| `t1_bottle` with the floor off | 2.85 mm · 99.7% | found on 0/300 | 2.83 mm · 99.7% |
| the six cube scenes | unchanged | unchanged | unchanged — no frame is under 45 |

So `t1_bottle` stands as the bottle's unoccluded scene (ground truth 0.7 mm
across cameras, tier check passed, inside 19.6 mm on every pair), with it said
beside the number that cam_over's frames were brightened for the detector. The
floor does not rescue the two darker attempts: `dim_t3_bottle_over` (cam_front
at 22) still finds nothing in cam_front, and `dim_t1_cube` (cam_over at 29)
nothing in cam_over. Those stay set aside.

So: the remaining bottle and clutter scenes are shot in the cube set's window,
about 17:15–18:30, room light plus daylight.

## The bottle set — shot Tue 6 Oct, 21:40–23:00, under the two LED lamps

Three scenes, 300 pairs each, none dropped; `captures/scored/20261006/`, results
`runs/scored/20261006/`. Ground truth agrees across cameras to 0.6–0.7 mm. ADD-S
in mm, with the share of pairs inside 10% of the bottle's diameter (19.6 mm):

| scene | hidden view sees | cam_front | cam_over | naive | gate | gate + EKF |
|---|---|---|---|---|---|---|
| `t1_bottle_b` — alone, second placement | — | 4.98 · 100% | 3.20 · 100% | 9.60 · 100% | 4.98 · 100% | **4.96 · 100%** |
| `t4_bottle_over` — hidden from cam_over | 6% | 4.00 · 100% | found on 0/300 | 4.00 · 100% | 4.00 · 100% | **3.95 · 100%** |
| `t3_bottle_over` — half hidden from cam_over | tier NOT certified (below) | 3.17 · 100% | 90.31 · 0% | 47.81 · 0% | 90.31 · 0% | **90.21 · 0%** |

**`t3_bottle_over` is the fused arm failing, and it has to be reported as that.**
cam_over, half hidden, puts the bottle 125 mm from the truth — 123 mm nearer
along its own line of sight, at the cover's range — with a pose score of 0.94.
cam_front has it to 2.6 mm with 0.84. The two are 90 mm apart by ADD-S, inside
the 150 mm association radius, so they are one cluster in conflict on every
pair, and the gate keeps the higher score: the wrong view. The fused pose then
sits about 35 mm clear of the table (centre at 133 mm, where it should be
98 mm) and the support check drops the bottle from the scene graph on 272 of
300 pairs; all 956 facts of the run are about an object that is not where the
facts put it or not there at all. This is the case 5 Oct's `t3_cube_over` missed only by distance
(171 mm, outside the radius). The pose score cannot arbitrate between views.
What can, on this evidence: the detector's score (0.95 against 0.77) and how
much of the object's silhouette has measured depth at all (95% against 0%). A fusion
change, replay-side, with this recording and the two cube T3 scenes as its test.

**`t1_bottle_b`: cam_front reads the bottle back to front** (178° from the truth,
pose score 0.88 against cam_over's correct 0.81; the views are 176° apart on
every pair), and the gate again keeps the higher score. ADD-S barely shows it —
4.98 against 3.20 mm — because the bottle is nearly its own mirror image. The
naive mean of the two is 9.60 mm. On 5 Oct's `t1_bottle` both views agreed to 3°.

**The tier of `t3_bottle_over` is not certified.** `visibility_from_gt` reads
cam_over as seeing 88% of the bottle, and `process` fails the scene; by the
colour image the cover hides roughly 60% of its width. The cover was a binder
with a grey cloth draped over it, about 37 cm from the camera, and the cloth
has no depth within ~50 px of its edge — the strip that covers the bottle — so
the tool counts the hidden part as no-depth, which it reads as visible. To report the
scene as T3 the tool must first establish the occlusion some other way (the
other camera's depth sees the cover); until then it is an unclassified scene
with a result, not a T3 scene.

Learned at the rig that night:

- **An occluder needs a flat face with texture the depth sensor can match, up
  to its edge.** Under the lamps: white paper over the card had depth on 5% of
  its face; a pale smooth binder let the tool confirm only 31% of the bottle as
  hidden. A cloth over the binder works across its face (`t4_bottle_over`, a
  white cloth: 94% of the bottle confirmed hidden) but not at its draped edge
  (`t3_bottle_over`, a grey one). The printed card of 5 Oct, flat, was measured
  up to its edge. Colour is not the variable; smoothness and the edge are.
- **The lamps fade.** Frame mean luma 117 / 77 at 21:43, 88 / 55 at 22:10,
  78 / 46 at 22:50 on the empty table: battery lamps, and cam_over was down to
  the 45 floor in about 70 minutes. No frame of the three scenes needed the
  floor (every logged gain is 1.0).
- **The rig moved twice.** A cat before the session (baseline 4.07 mm / 0.59°:
  full recalibration at 22:04), and both cameras between 22:29 and 22:45 while
  cam_front's dead USB link was being replugged (3.0 and 6.1 px; baseline
  1.33 mm / 0.23°: re-registration at 22:47). Frame checks put `t1_bottle_b` and
  `t4_bottle_over` wholly on the 22:04 calibration and `t3_bottle_over` wholly
  on 22:47. cam_front's link this time did not come back with a controller
  unbind/bind; only the replug did.
- No closing board shot at 23:00; the frame check stood in for it (0.00 px in
  both cameras across `t3_bottle_over`).

### Restarted at 23:07 — the bottle again, and the first clutter scene

The lamps went back on at 80% battery, the cameras had not moved (0.00 px), and
the bottle sequence was shot again on a third placement, this time with every
tier certified. Same calibration (22:47), same units:

| scene | hidden view sees | cam_front | cam_over | naive | gate | gate + EKF |
|---|---|---|---|---|---|---|
| `t1_bottle_c` — alone | — | 3.43 · 100% | 4.61 · 100% | 9.53 · 100% | 3.43 · 100% | **3.37 · 100%** |
| `t4_bottle_over_b` — hidden from cam_over | 2% | 2.93 · 100% | found on 0/300 | 2.93 · 100% | 2.93 · 100% | **2.89 · 100%** |
| `t3_bottle_over_b` — half hidden from cam_over | ≤ 68% (see below) | 3.13 · 100% | 35.05 · 0% | 20.62 · 6% | 34.10 · 3% | **33.99 · 0.3%** |

- **`t3_bottle_over_b` is `t3_bottle_over` again, now on a scene whose tier
  check passes**: cam_over's pose is 68 mm off (65 mm nearer along its line of
  sight) with a pose score of 0.99; cam_front's is 3 mm off with 0.85; the gate
  keeps cam_over; the fused bottle rests on nothing and is in the scene graph
  on 0 of 300 pairs. The detector's scores are 0.46 and 0.86 — the right way
  round. Two recordings of the same failure, one certified.
- Its "68% visible" is an upper bound. The tool confirms a third of the bottle
  hidden (the cover's wide face has depth there) and counts the rest of the
  covered part, near the cover's edge, as no-depth = visible.
- `t1_bottle_c`: cam_front flipped again (171°, pose score 0.89 against 0.82),
  kept by the gate. Two placements of three that night.
- The binder-and-cloth cover is measurable when the bottle is behind the middle
  of its **wide face** (2% visible) and not when it is behind the narrow side
  seen obliquely (the first try at `t4_bottle_over_b` read 96% visible).

**`t2_clutter_a` — recorded at 23:38, to be reshot.** Cube, bowl and bottle on
their trays, four distractors on the table behind and beside them (a handset,
a wooden block, a roll of tape, a card — not the proposal's eight). All six
view-object pairs clear (cam_over sees 82% of the cube). Pooled over the three
objects, 900 object-pairs, each against its own threshold:

| arm | found | ADD-S | inside 0.1 d |
|---|---|---|---|
| cam_front | 746 / 900 | 2.69 mm | 82.9% |
| cam_over | 875 / 900 | 3.32 mm | 97.2% |
| naive | 889 / 900 | 5.81 mm | 84.1% |
| gate | 889 / 900 | 2.91 mm | 98.8% |
| gate + EKF | 889 / 900 | **2.86 mm** | **98.8%** |

The first scene in which fusion beats both single views. It is not kept as a
scored scene, for one reason: the light had collapsed to a frame luma of
34 / 19 (81 / 50 half an hour before), and every frame was brightened ×1.31 /
×2.43 by the floor to be detected at all. It does show the floor working well
below where it was tested. Two more things it shows:

- **The support check in clutter:** the three real objects are reported on 889
  of 889 object-pairs; of 493 false object-pairs, 194 survive — one false cube
  at table height, 192 mm from the real one, on 194 pairs. 582 of 2,517 facts
  name it. This is the case the rule cannot catch.
- Ground truth across cameras is 1.2–1.9 mm with three trays in view, against
  0.6–0.7 mm for one.

Getting to that layout took four tries (an empty desk, trays without objects,
distractors stacked on the trays), so the instruction for a clutter scene is
now spelt out: each object on its own tray, distractors on the bare table
behind and beside, nothing on a tray but its object.

Frame check at the end, against `t1_bottle_c`: cam_front 0.0–0.2 px through
the clutter scene; cam_over crept +0.3 to +0.5 px (848 × 480; about 1 px on the
stills, inside the 1.5 px limit) from `t4_bottle_over_b` onward. Stopped 23:42.

## Next evening — clutter, ~40 min

- [x] `t1_bottle` — shot 5 Oct, usable with the brightness floor
- [x] `t1_bottle_b`, `t4_bottle_over`, `t3_bottle_over` — shot 6 Oct (above)
- [ ] Board check first, every time: the rig has not survived a night yet
- [ ] Lamps on charge until the session starts; or shoot 17:15–18:30
- [ ] `t2_clutter_a/b/c`: all three trays (460 × 370 mm shared view: bowl + cube
      side by side, 380 × 210 mm, bottle tray 216 × 100 mm in front) plus the
      eight distractors, rearranged each time. Each object on its own tray;
      distractors on the bare table behind and beside the trays, never on one.
      `t2_clutter_a` of 6 Oct was shot too dark and is to be redone

## The evening after — training layouts, ~50 min

- [ ] 10–12 varied layouts with stills, ~90 pairs each, disjoint from the scored
      scenes: five or six with the objects moved and turned, two with **empty
      trays**, two with the card standing in view. Labels come later from the
      ground-truth poses

One evening is used and two are needed, of the four left (6–9 Oct), so two can be lost.

## Any evening, no rig needed — process while the rig is still there

- [ ] `capture_day.sh process` on each day's scenes; every FAILED tier check or
      refused ground truth is a reshoot for Saturday
- [ ] `training/pose/registration_quality.py` on real captures (`STATUS.md` [VERIFY])
- [ ] Remote access to the desktop: Tailscale or SSH; GRUB default = Ubuntu;
      BIOS restore-on-power-loss; suspend and automatic reboots off; tested from
      a phone hotspot
- [ ] Back up `captures/`, `runs/` and the ONNX model to a drive and to cloud

## Sat 10 Oct — reshoots, demo footage, slack

- [ ] Reshoot what the week's checks rejected (new scene names)
- [ ] Demo run (`CAPTURE_PROTOCOL.md` §7): `the_eyes_run live --record demo --show=console`,
      a screen recording, phone video of the rig — ~30 min
- [ ] Anything an evening did not reach
- [ ] Final backup; cover the rig

## 11–24 Oct — Taiwan (offline, or remote to the desktop)

Detector fine-tune and replay; real-capture mAP from ground-truth boxes;
predicate accuracy against the photos; bootstrap CIs by scene (proposal §7.1);
report draft, figures, video script.

## 25–31 Oct

Re-register; reshoot the live demo if the detector changed; edit the video;
finish the report; ZIP; submit by Sat 31 Oct.
