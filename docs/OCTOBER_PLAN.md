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

## What is left to capture — about 3 to 3.5 hours at the rig

No scored scene exists yet. The cube scenes planned for the night of Sun 4 Oct
were **aborted at 22:00: the room lighting failed.**

The times below are from the 4 Oct rehearsal's pace, not from a guess: driven
from the keyboard with `capture_day.sh scene --stage stills|record`, a scene
took about 5 minutes, nearly all of it moving things on the desk (the stills
and the 300-pair recording are about a minute of machine time).

| Capture | Scenes | Time |
|---|---|---|
| **Cube set** — alone; hidden and half hidden from each camera | 5 | 35–40 min |
| **Bottle** — alone; half hidden | 2 | 15 min |
| **Clutter (T2)** — three trays + 8 distractors, ~10 min each to arrange | 3 | 30–35 min |
| **Training layouts** for the detector, including empty trays (finding 3) | 10–12 | 35–40 min |
| **Demo footage** for the video | — | 30 min |
| Start and end of each session (board on, re-register, empty table; final check) | per session | 12 min |

The first three rows are the proposal's ten scored scenes (T1 ×2, T2 ×3, T3 ×3,
T4 ×2). On top: about an hour of preparation and an hour of slack for reshoots.

**Preparation, before any scored scene:**

- [ ] **Fix the lighting** — the one item with no estimate. If it cannot be
      fixed, the fallback is Sat 10 Oct in daylight with the curtains drawn,
      every scored scene in one sitting so the light is the same throughout
- [ ] Wrap the occluder's wooden stand (it is detected as a cube) — 5 min
- [ ] Bottle seats flat in its tray; seat height 179.0 mm ±0.3 (`PRINT_PLAN.md` §8.1) — 20–40 min if not done
- [ ] Eight distractors in the palette neutrals — 15–30 min

**Every session:** TV off, mouse and keyboard off the desk, nobody touches the
tripods or the two camera cables, and `capture_day.sh begin` first. Afterwards
`capture_day.sh end`, then copy `captures/` to an external drive — it is
gitignored and exists only on this machine.

## First working evening (Mon 5 Oct if the lighting is fixed) — the cube set, ~45 min

After this the main claim has data behind it.

- [ ] `capture_day.sh begin` — re-register, empty-table recording, re-fit `depth_tare`
- [ ] `t1_cube` — cube alone, tray flat on the bare table
- [ ] `t4_cube_over`, then `t3_cube_over` (card slid aside; `--stills-from`, tray untouched)
- [ ] `t4_cube_front`, then `t3_cube_front` — the other camera
- [ ] `capture_day.sh end`; back up

**If only fifteen minutes are available:** `t1_cube` and `t4_cube_over`. Those
two scenes are the minimum that supports the claim.

## Second evening — bottle and clutter, ~1 h 15

- [ ] `t1_bottle`, `t3_bottle_over` (a card tall enough for a 191 mm bottle)
- [ ] `t2_clutter_a/b/c`: all three trays (460 × 370 mm shared view: bowl + cube
      side by side, 380 × 210 mm, bottle tray 216 × 100 mm in front) plus the
      eight distractors, rearranged each time

## Third evening — training layouts, ~50 min

- [ ] 10–12 varied layouts with stills, ~90 pairs each, disjoint from the scored
      scenes: five or six with the objects moved and turned, two with **empty
      trays**, two with the card standing in view. Labels come later from the
      ground-truth poses

Three of the five weekday evenings (5–9 Oct) are used, so two can be lost.

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
