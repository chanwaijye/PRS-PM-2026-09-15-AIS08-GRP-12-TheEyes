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

## Where things stand — Sun 4 Oct, 17:30

Nothing was shot 1–3 Oct. On 4 Oct the afternoon went on prerequisites and a
rehearsal (daylight, TV on — **none of it is a scored result**):

- **Rig recalibrated** (it had drifted 10.7 mm in two idle weeks) and
  re-registered once more after clearing the desk moved a camera.
- **Detector works on the printed objects with no fine-tuning**: cam_front finds
  all three on 150/150 pairs.
- **Capture flow works end to end**, driven remotely scene by scene: cube alone,
  cube fully hidden from cam_over, cube half hidden. Measured tiers 100% / 0% /
  48% visible, all as staged.
- **Ground truth rebuilt on the table plane**: cross-camera disagreement on the
  cube 9.6 → 0.6 mm. Against it, cube alone: cam_front 7.6 mm, cam_over 4.9 mm,
  gated fusion 4.7 mm ADD-S (threshold 8.6 mm). Fully hidden from cam_over:
  cam_over finds it on 0/300 pairs, fused output on 300/300 at 6.7 mm.

**Found, not yet fixed** (all replay-side — none blocks capture):

| # | Finding | Fix |
|---|---|---|
| 1 | Depth is biased per camera, in opposite directions: cam_front reads 1.0% short (5.7–8.9 mm), cam_over up to 7.3 mm long at 0.8 m | Per-camera depth correction fitted from the empty-table recording |
| 2 | Gate + EKF (the `fused` arm) is worse than the gate alone on the cube (9.1 vs 4.7 mm): the cube's symmetric-equivalent orientations, 88° apart between views, are filtered through | Pick the symmetric equivalent nearest the track before the EKF update; needs the symmetry group in the mesh registry |
| 3 | The grey tray is detected as a `bowl` on every pair, giving a false `(on red_cube bowl)` | Fine-tune the detector on real frames with trays in them |
| 4 | The bowl's pose is 16–25 mm off and cam_front's depth inside it reads 2–3 cm short | Not understood; keep the bowl out of T3/T4 targets |
| 5 | Half-hidden cube: cam_over detects it on 286/300 pairs and every pose is wrong — the visible half has no depth | A finding to report; the gate keeps it out of the fused pose |
| 6 | Clutter is detected: TV content, a red ring on a cable, the card's wooden stand | TV off, desk clear, stand wrapped |

---

## Sun 4 Oct, after dark (~19:30) — scored cube scenes

Driven from the keyboard with `capture_day.sh scene --stage stills|record`; the
person at the rig only moves things. **TV off. Nobody touches tripods or camera
cables.**

- [ ] `capture_day.sh begin` — re-register (board on), empty-table recording
- [ ] `t1_cube` — cube alone, tray flat on the bare table
- [ ] `t4_cube_over`, then `t3_cube_over` (card slid aside; `--stills-from`, tray untouched)
- [ ] `t4_cube_front`, then `t3_cube_front` — the other camera
- [ ] `capture_day.sh end` — the rig held; then copy `captures/` to an external drive

## Mon 5 – Tue 6 Oct (evenings) — the rest of the suite

- [ ] `t1_bottle`, `t3_bottle_over` (a card tall enough for a 191 mm bottle)
- [ ] T2 ×3: all three trays (460 × 370 mm shared view: bowl + cube side by side,
      380 × 210 mm, bottle tray 216 × 100 mm in front) plus 8 distractors in the
      palette neutrals. Needs the bottle to seat (`PRINT_PLAN.md` §8.1)
- [ ] 10–15 varied layouts with stills, disjoint from the scored scenes — the
      fine-tuning and real-mAP set that finding 3 needs
- [ ] Offline, no rig: findings 1 and 2, then replay everything

## Wed 7 – Fri 9 Oct (evenings) — process while the rig is still there

- [ ] `capture_day.sh process` on every day's scenes with the depth correction in;
      every FAILED tier check or refused ground truth is a reshoot for Saturday
- [ ] `training/pose/registration_quality.py` on real captures (`STATUS.md` [VERIFY])
- [ ] Remote access to the desktop: Tailscale or SSH; GRUB default = Ubuntu;
      BIOS restore-on-power-loss; suspend and automatic reboots off; tested from
      a phone hotspot
- [ ] Back up `captures/`, `runs/` and the ONNX model to a drive and to cloud

## Sat 10 Oct — reshoots and backup demo footage

- [ ] Reshoot what the week's checks rejected (new scene names)
- [ ] Demo run (`CAPTURE_PROTOCOL.md` §7): `the_eyes_run live --record demo --show=console`,
      a screen recording, phone video of the rig
- [ ] Final backup; cover the rig

## 11–24 Oct — Taiwan (offline, or remote to the desktop)

Detector fine-tune and replay; real-capture mAP from ground-truth boxes;
predicate accuracy against the photos; bootstrap CIs by scene (proposal §7.1);
report draft, figures, video script.

## 25–31 Oct

Re-register; reshoot the live demo if the detector changed; edit the video;
finish the report; ZIP; submit by Sat 31 Oct.
