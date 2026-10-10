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

## The clutter set and the training layouts — shot Wed 7 Oct, 22:10–23:35

Under the two LED lamps, which held this time (frame luma 80–91 / 48–61 all
evening; no frame needed the brightness floor). `captures/scored/20261007/`,
results `runs/scored/20261007/`; training layouts in
`captures/scored/20261007_train/`.

**The three T2 scenes.** Cube, bowl and bottle on their trays, four distractors
on the bare table (a wooden block, a handset, a roll of tape, a card — not the
proposal's eight), rearranged each time; 300 pairs each, none dropped; every
view sees every object clear (the least is cam_over on the cube in A, 82%).
Ground truth across cameras 0.3–0.4 mm for cube and bowl, 1.3–1.4 mm for the
bottle. Pooled over the three objects, 900 object-pairs, each against its own
0.1 d threshold — ADD-S in mm and the share inside:

| scene | cam_front | cam_over | naive | gate | gate + EKF |
|---|---|---|---|---|---|
| `t2_clutter_a` | 3.07 · 100% | 4.34 · 95.8% | 11.69 · 66.9% | 3.53 · 100% | **3.34 · 100%** |
| `t2_clutter_b` | 3.25 · 100% | 3.87 · 100% | 11.92 · 66.7% | 3.51 · 100% | **3.50 · 100%** |
| `t2_clutter_c` | 2.68 · 76.9% | 6.41 · 66.7% | 6.94 · 66.7% | 5.33 · 71.2% | **5.33 · 66.8%** |

- **In C the cube fails and the other two pass.** cam_front detects the cube on
  92 of 300 pairs (score 0.36); cam_over has it on all 300 with its centre
  12 mm off and 54% of its silhouette without depth. The fused cube is 10.9 mm
  from the truth, outside its 8.7 mm threshold. Nothing occludes it: this is a
  small object far from one camera and beside another object in the other.
- **Naive is at two-thirds inside in every scene** (66.7–66.9%): it is the cube
  — 22.4 and 22.9 mm in A and B, 16.0 mm in C, 0–1% inside — whose equivalent
  orientations the naive arm averages, as on the cube alone on 5 Oct. The
  harness now prints a per-object table under the pooled one (8 Oct), which is
  where this comes from; it had been a `[VERIFY]`.
- **The scene graph on clutter, support check on:** the three real objects are
  reported on 2,700 of 2,700 object-pairs. 19 false object-pairs (a
  `mustard_bottle` the detector sees on the bedding, 1.8 m away) are tracked
  and none is reported; 0 of 6,210 facts name an object that is not there. No
  tray was taken for a bowl in these three scenes, and the false cube at table
  height of 6 Oct's dark attempt did not recur.

**These scenes are processed on a registration taken after them.** cam_over
moved about 3 px on the stills when the board came off after the 22:14
calibration — seen as 1.4–2.5 mm of cross-camera disagreement in the first
stills, and confirmed on the frames. Both cameras were then still from 22:19 to
the closing registration at 22:47 (0.0 px), and that registration is the one
each scene carries; the 22:14 copies are kept beside them as
`*.2214_stale.yaml`. Twelve minutes later cam_front had moved (2.4 mm), which
the training layouts were re-registered for (22:59).

**cam_front's USB link failed again** at the start, on a different port from
6 Oct, and again only reseating the cable brought it back: the cable or the
camera's connector, not the port.

**Twelve training layouts**, 90 pairs and 10 + 10 stills each, tray poses
measured in every one: six with all three objects (`train_01`–`06`), two with
the three trays **empty** (`07`, `08` — the negatives for the tray-as-bowl false
positive), two with the printed card standing in view (`09`, `10`), one with
bowl and cube only (`11`), one with the bottle only (`12`). 1,080 pairs, 2,160
frames. A first `train_09` with the card hiding the cube from cam_over is set
aside.

Two checks that cried wolf, and what they now look at: a whole-frame shift
check on recorded frames follows the TV's reflection of the table, and a
background check follows the pillow on the bed. The camera check that holds is
on the 1080p stills, restricted to the rigid background.

**The proposal's ten scored scenes are now all shot:** T1 ×5, T2 ×3, T3 ×4 (one
uncertified), T4 ×4 across 5–7 Oct.

## The fusion fix — depth evidence, 7–8 Oct (no rig)

The half-hidden bottle showed the gate keeping the wrong view because it had
the higher pose score. The fix does not touch the score: every per-view pose
is now checked against the two depth images before it is fused
(`runtime/view_evidence.hpp`, `fusion.yaml` `view_evidence:`,
`SystemCode/fusion/README.md`).

- **Free-space veto.** The object is rendered at the estimated pose into the
  *other* camera. If that camera measures a surface *beyond* the object over a
  third or more of its silhouette — it is looking through the place the object
  is claimed to be — the estimate is not fused. A surface in front (an
  occluder) or no depth is evidence of nothing, so a hidden object is never
  vetoed by the camera that cannot see it.
- **Arbitration.** A two-view conflict keeps the estimate better backed by
  depth (its own view's agreement, discounted by what the other view sees
  through) when the two differ by 0.15 or more; otherwise the score decides,
  as before.

What chose the numbers: a study on every fifth pair first
(`scripts/view_evidence_study.py`), then the values the pipeline logged for
every estimate of the 16 scored scenes. Of 10,843 estimates within 30 mm of the
truth, the most any is seen through is 0.13; the four estimates the gate was
getting wrong are seen through by 0.46–0.54, 0.69–0.76, 0.85–0.94 and
0.96–1.00. The threshold of one third has none of either on the wrong side.
The margin of 0.15 comes from the unoccluded bottle, whose two views differ in
evidence by at most 0.14 on any pair; the views the rule is for differ by 0.8
or more.

All 20 processed scenes of 5–7 Oct, replayed with it. Gate + EKF, ADD-S and the
share inside 0.1 d; "by score" is the gate as it was, run as its own arm on the
same estimates:

| scene | by score | with depth evidence |
|---|---|---|
| `t3_bottle_over` | 90.21 mm · 0% | **3.15 mm · 100%** |
| `t3_bottle_over_b` | 33.99 mm · 0.3% | **3.10 mm · 100%** |
| the other 18 scenes | — | identical to 0.01 mm |

- The single-view and naive arms are the same in every scene, as they must be:
  they never see the evidence.
- The veto fired on the four estimates it was built for — the bottle and the
  cube at the occluder's range, the cube on the occluder's print: 300, 291, 300
  and 255 pairs — and on 3 pairs of a tray taken for a bowl. On no correct
  estimate, in 4,800 pairs.
- **In the scene graph:** the real bottle is reported on 300 of 300 pairs in
  both half-hidden scenes (it was 28 and 0, the fused pose having floated); the
  second cube of `t3_cube_over` and the cube on the card in `t3_cube_front` are
  no longer tracked at all, where before they were tracked and left to the
  support check.
- **Cost:** 3–11 ms a pair (1.1–2.4 ms an estimate), logged per pair.

What it does not fix, measured on the same replay:

- **The bottle read back to front** (`t1_bottle_b`, `t1_bottle_c`): both views
  are backed about equally, the margin leaves it to the score, and the fused
  orientation is still cam_front's flipped one. With no margin the choice
  alternated between frames and the result was worse (5.73 mm against 3.37),
  which is what the margin is for.
- **The cube in `t2_clutter_c`** (fused 11.3 mm, 0.3% inside): the two views are
  12 mm apart, which is not a conflict, so they are averaged, score-weighted.
  cam_front has it to 4.4 mm on the 92 pairs it detects it at all.
- **A tray taken for a bowl at table height** is still in the scene graph of
  `t3_bottle_over` on 300 pairs: it is neither floating nor seen through. That
  one is the detector's, and the empty-tray layouts of 7 Oct are for it.

## Any evening, no rig needed — process while the rig is still there

- [x] `capture_day.sh process` on each day's scenes — done through 7 Oct; the one
      FAILED tier check is `t3_bottle_over` (6 Oct), superseded by
      `t3_bottle_over_b`
- [ ] `training/pose/registration_quality.py` on real captures (`STATUS.md` [VERIFY])
- [x] Remote access to the desktop: AnyDesk (settled 10 Oct). Checked on the
      box: its service starts at boot, the session logs in by itself, suspend
      and automatic reboots are off, GRUB's default is Ubuntu. A Wi-Fi smart
      plug is to be fitted in case the machine hangs; BIOS restore-on-power-loss
      is not confirmed from here
- [ ] Back up `captures/`, `runs/` and the ONNX model to a drive and to cloud —
      **not done, by decision (10 Oct):** everything stays on the desktop's disk

## Sat 10 Oct — the `in` scenes and the video takes (16:24–22:40)

Under the two LED lamps and the room light. `captures/scored/20261010/`
(`day.log` has every step), results `runs/scored/20261010/`, video material
`runs/video/`. The scored scenes of 5–7 Oct were checked on disk first and none
needed a reshoot.

- [x] Reshoot what the week's checks rejected — nothing was rejected; the gap
      found was a missing predicate, below
- [x] Demo run, console recording, phone video of the rig
- [ ] Cover the rig — the author's, after the phone shots. No final backup
      (see above)

**Two scenes with the cube in the bowl.** No scored scene held a true
`(in a b)`, which the proposal's predicate list needs. Bowl and bottle on their
trays, the cube loose inside the bowl (so it has no ground-truth pose), four
distractors on the table; 300 pairs each. Bowl and bottle pooled, 600
object-pairs, ADD-S in mm, all 100% inside 0.1 d:

| scene | cam_front | cam_over | naive | gate | `in` fact |
|---|---|---|---|---|---|
| `t2_in_a` (16:45) | 3.00 | 4.50 | 2.82 | 2.90 | `(in red_cube_02 bowl_01)` 300 of 300 |
| `t2_in_b` (17:22) | 2.80 | 4.55 | 3.13 | 3.11 | `(in red_cube_01 bowl_01)` 300 of 300 |

- **`t2_in_b` is the clean one:** six facts, each on 300 of 300 pairs, and
  nothing else — `in` for the cube, `on` the table for bowl and bottle, three
  `clear`.
- **`t2_in_a` also reports a cube that is not there**, on 232 pairs: cam_front
  reads the handset, half hidden behind the bottle, as a `red_cube` (0.34) and
  FoundationPose fits a cube at table height, so the support check keeps it.
- **Both fail the T2 tier check** (cam_front sees 75% and 74% of the bowl): the
  hidden quarter is the cube inside it. Expected, not a reshoot.
- **`(clear bowl_01)` is emitted with the cube in the bowl:** `clear` is defined
  on `on`, not on `in`.
- Ground truth across cameras 0.35 / 0.61 mm for the bowl, 1.09 / 1.19 mm for
  the bottle. `t2_in_a` is on the 16:33 calibration, `t2_in_b` on the 17:16
  registration; `t2_in_b`'s recording against the empty table before it reads
  0.01 px in both cameras.

**`occluded_from` fires for objects that nothing hides** — found the same day,
on the scored set. The fact is derived from `ObjectState::visible_views`, which
fusion sets for the views it *fused*, not the views that see the object. When
the gate drops a view in a conflict, the object is reported as occluded from a
camera that estimated it on every pair:

| scene | fact | pairs |
|---|---|---|
| `t1_bottle_b`, `t1_bottle_c` | `(occluded_from mustard_bottle_01 cam_over)` | 300 of 300 |
| `t2_clutter_a`, `t2_clutter_b` | `(occluded_from mustard_bottle_01 cam_front)` | 300 of 300 |
| `t2_clutter_c` | `(occluded_from red_cube_01 cam_front)` | 208 (cam_front detects the cube on 92: undetected, not hidden) |

It is the bottle read back to front showing through as a fact. **Not fixed.**
Replay-side: decide whether the fact means "not fused" or "not visible" before
predicate accuracy is computed; a field may be added to `ObjectState`, none
re-typed.

**Three calibrations in one day,** each with its depth fit on an empty table
(cam_front ×1.0089, ×1.0097, ×1.0096; cam_over's error is not a scale and stays
at 1.0):

| time | why | result |
|---|---|---|
| 16:33 | three days off USB; baseline 2.77 mm / 0.58° against 7 Oct, re-registration refused | full calibration, 0.275 / 0.324 px |
| 17:16 | cam_front's USB link died after `t2_in_a`; reseating its cable moved it 1.7 px (at 848×480) | re-registration, rig intact, 0.255 / 0.308 px |
| 20:46 | cam_over dropped off USB at 17:40 while idle and was plugged back at 20:40; baseline 3.93 mm / 0.10° against 17:16 | full calibration, 0.285 / 0.315 px, 49 shared corners |

- **Both cameras' USB links have now failed** — cam_front on 6, 7 and 10 Oct,
  cam_over for the first time on 10 Oct. A librealsense hardware reset and a
  sysfs de-authorise / re-authorise did not bring cam_front back; the cable did.
- **A lamp shining at a lens costs corners.** A board pair at 20:41, with the
  lamps put back after charging, found 38 corners in cam_over (0.379 px) and
  was not used; with the lamps turned, 49.
- **cam_over's mount has play:** 4.7 px from its afternoon position at 20:41,
  back within 0.2 px of it at 20:46. From 20:46 to the last take at 21:56
  neither camera moved (0.02 / 0.04 px). No closing board registration was
  taken; the takes carry no accuracy numbers.

**The video takes** (21:00–21:59). The monitor is in both cameras' pictures, so
it stays dark and nothing is shown on it: the runner writes the console to a
file itself (`--show-record`), one frame per processed pair, to be retimed from
`frames.stamp_ns` in the edit. Driven with `runs/scored/helpers/demo_take.sh`;
console files and logs in `runs/video/`, recordings in `captures/runtime/`,
metrics in `runs/runtime/`. Every live take logs `source = live`,
`pose_backend = foundationpose`, on commit `7a81269`.

| take | kind | pairs | pairs/s | what it is |
|---|---|---|---|---|
| `demo_cold_a` | live | 402 | 13.4 | cube alone, a hand on cam_over's lens: the picture goes black |
| `demo_cold_b` | live | 581 | 11.9 | a hand in front of cam_over; stutters |
| `demo_cold_c` | live | 1,199 | 24.0 | the card, printed side to cam_over: a ghost cube in the plan view |
| **`demo_cold_d`** | live | 1,048 | 21.0 | **kept.** The card, plain back to cam_over |
| **`demo_live_a`** | live | 2,231 | 12.1 | **kept.** The long sequence, 184.9 s |
| `demo_live_b` | live | 2,195 | 11.9 | the long sequence again; spare |
| **`demo_clean_a`** | recording only | 4,200 | 30 | **kept.** The long sequence, every pair, none dropped (6.1 GB) |

- **`demo_cold_d`:** with the card in, cam_over's count is 0 and the fused
  count 1; `(on red_cube_01 table_01)` holds on all 1,048 pairs and
  `(occluded_from red_cube_01 cam_over)` is on for 287, then clears.
- **`demo_live_a`, by the take's own clock:** empty table to 6 s; the bottle
  from 7 s; the bowl from 20 s; the cube on the table 32–37 s;
  `(in red_cube_04 bowl_04)` on 57 pairs, 40–47 s; out again from 48 s; the
  card before cam_over 60–71 s, `(occluded_from red_cube_05 cam_over)` on 116
  pairs with the fused count held at 3; the card before cam_front 72–88 s,
  cam_front at 0 objects, fused 3; the bottle lifted away at about 102 s.
- **It shows the finding above:** `(occluded_from mustard_bottle_01 cam_front)`
  on 500 pairs with nothing hiding the bottle. The video has to say so.
- **An object moved by hand gets a new name:** the cube is `red_cube_01`, `02`,
  `04`, `05` over the take.

**Live speed, measured** (`demo_live_a`; the cube alone from `demo_cold_d`). The
`[VERIFY]` in `STATUS.md` on speed with three objects per view can be closed
with this:

| objects tracked in each view | pairs | per pair, p50 | processed between pauses |
|---|---|---|---|
| 0 | 162 | 7.5 ms | 30 a second |
| 1 (cube) | 767 | 34.0 ms | 30 a second |
| 2 | 1,308 | 63.8 ms | 15 a second |
| 3 | 281 | 90.7 ms | 10 a second |

Over the whole take, 12.1 pairs a second, because **every new detection costs
a FoundationPose registration of about 1.1 s in that view, and nothing is
processed meanwhile:** 23 pauses, 37.5 s of the 184.9. The rate between pauses
steps in whole camera frames, which is why it reads 30, 15 and 10.

**What a false detection costs live.** A registration each time it appears:

- The lamp's streak at the right edge of cam_front's picture was read as a
  `mustard_bottle` (0.25–0.43) on 33 pairs of `demo_cold_b`; 14 of that take's
  19 pauses are cam_front registering it.
- Skin is read as a `red_cube` (0.27–0.46), and so is a red mark on the card's
  print. The support check kept every one of them out of the facts; they still
  cost the time. A plain dark card does neither.
- The runner starts about 10 s after the monitor goes dark, so the first tap of
  the first take was not recorded.

**Light:** 86 / 53 on the empty table at 20:49, 78 / 45 at the end of the last
take. The brightness floor came in on cam_over only while a hand or the dark
card filled its picture (18–73 pairs a take in the cold opens); in the two long
takes, never.

**The sync cable was not connected.** Reported after the shoot: it was broken
(by the cats) and then lost. `config/cameras.yaml` still sets master and slave,
and both cameras delivered frames all day. What the logs say: every pair in
every scored scene and every take is within 16.7 ms, half a frame. What they
cannot say is when the cable stopped working: `frames.skew_ms` reads the same
with it as without. On 6–10 Sep, cable connected, a run's mean offset is
3.4–14.5 ms, wandering by 0.3–2.4 ms within the run (15 runs), and the one long
run (145 s) drifts −0.023 ms/s. On 10 Oct the seven takes' mean offsets are
2.9–12.3 ms, wandering by 1.9–3.8 ms, and the two 185 s takes drift −0.007 and
−0.014 ms/s.
Whether 5–7 Oct was shot with hardware sync is `[VERIFY]` — asked of the
author. Nothing on the table moves during a scored
scene, so no accuracy number depends on it; the video script's "to about a
millisecond" does, and has to change.

**Rendered and collected for the video:** eleven replay clips of the scored
scenes, 300 frames each (`runs/scored/helpers/render_clips.sh`), among them the
half-hidden bottle under the old rule and the new; the phone material in
`runs/video/phone/` (38 files: a table-top clip of each take, eight rig clips,
the console on the monitor, 22 photos). The console on the monitor is a replay
of `demo_live_a` (7.8 pairs a second with the window at 4K) and is not a live
shot.

## 11–24 Oct — Taiwan (offline, or remote to the desktop)

Detector fine-tune and replay; real-capture mAP from ground-truth boxes;
predicate accuracy against the photos; bootstrap CIs by scene (proposal §7.1);
report draft, figures, video script.

Added 10 Oct: settle what `occluded_from` means and fix it on replay, before
predicate accuracy; retime the live console clips from `frames.stamp_ns`; in
the video script, the sync sentence, the measured speed and the `in` scene;
merge the video plan (branch `worktree-video-plan`).

## 25–31 Oct

Re-register; reshoot the live demo if the detector changed; edit the video;
finish the report; ZIP; submit by Sat 31 Oct.
