# October plan — finish the rig work before 11 Oct

Written 2026-10-01. Away in Taiwan for work **Sun 11 – Sat 24 Oct**; the rig
stays home, because moving it breaks the calibration and the GPU box and pose
server do not travel. Final submission is **Sat 31 Oct** (`README.md`).

**How the plan works.** The rig only has to produce raw recordings, 1920×1080
ground-truth stills and photos. Replay is bit-exact, so the detector,
FoundationPose, fusion, the ground-truth solve and all eight ablation arms rerun
from disk. **Sat 3 Oct is the day that matters**; the weekday evenings after it
process the data while the rig is still there to reshoot what failed. Assumes
weekday evenings plus full days on 3, 4 and 10 Oct.

**Biggest unknown:** the detector has never been run on the correct-colour
printed objects — every real detector number so far is the brown test cube, and
the 29 Sep live run had no ground truth. It does not block capture (recordings
do not depend on the detector), but it decides whether extra fine-tuning data is
needed, so it is checked first.

Tools built for this on 2026-10-01 (`SystemCode/runtime/README.md`, "Ground truth
and visibility"): `scripts/capture_day.sh`, the outlier screen in
`gt_from_trays`, and `visibility_from_gt`.

---

## Thu 1 Oct (evening) — go/no-go

- [ ] Re-register. If refused: burst → recalibrate → `training/shared_region.py --overlay` → re-tape
- [ ] Three printed objects in the shared region:
      `build/runtime/the_eyes_run live --record-only detcheck --frames 150`,
      then `~/venvs/the-eyes-r1/bin/python training/exposure_report.py captures/runtime/<session>`
- [ ] Replay it through the ONNX detector (`the_eyes_run replay <session> --pose mock --show`)
      and re-run the §3.1a three-condition comparison (`CAPTURE_PROTOCOL.md`).
      **Decision:** finds all three → proceed; weak → add a fine-tune set on Sunday
- [ ] Tape: board centre → desk front edge; `t.y = 0.375 − f` closes the
      `scene.yaml` `pose_w.t.y` **[VERIFY]** (`CAPTURE_PROTOCOL.md` §4.2)

## Fri 2 Oct (evening) — trays, yardstick, props

- [ ] Bottle base dish / bowl tray hole to 74 mm if not done; seat heights
      42.1 / 179.0 / 49.0 mm ±0.3 (`PRINT_PLAN.md` §8.1); pencil-tick the bottle
- [ ] `PRINT_PLAN.md` §8.2 shots A and B. No scratch database is needed any more:
      `build/runtime/gt_from_trays captures/gt_test_A - --stills --factory --dry-run`.
      Compare against the CAD heights and distances there. **No scored capture until it passes**
- [ ] Occluder: stiff green or magenta card on a small black stand; 8 distractors
      in the palette neutrals; board jig, or tape
- [ ] Commit the configs you will capture with — nothing changes between scenes (§9)

## Sat 3 Oct — scored capture (night or blackout; TV off)

- [ ] `scripts/capture_day.sh begin` — checks, re-registration, 30 empty-table pairs.
      Commit `config/extrinsics.yaml` if it says it changed
- [ ] One `scripts/capture_day.sh scene ...` per scene. It asks for the phone photo,
      shoots the stills, **refuses to go on until every tray decodes and agrees**,
      then records. Order: T1 ×2 → **T4 ×2** → T3 ×3 → T2 ×3, so whatever the day
      does not reach is the least important. A default staging, if nothing better
      suggests itself (T3/T4 targets need their T1 first):

      | scene | tier | occluded | target |
      |---|---|---|---|
      | `t1_cube` | T1 | none | red_cube |
      | `t1_bowl` | T1 | none | bowl |
      | `t4_cube_over` | T4 | cam_over | red_cube |
      | `t4_bowl_front` | T4 | cam_front | bowl — the scene occluding the other way (§5) |
      | `t3_cube_over`, `t3_bowl_over`, `t3_cube_front` | T3 | as named | as named |
      | `t2_clutter_a/b/c` | T2 | none | — (3 targets + 8 distractors) |

      All three trays fit the 460 × 370 mm shared view (2026-10-04 extrinsics) as bowl + cube side by side
      (380 × 210 mm) with the bottle tray (216 × 100 mm) in front: 380 × 310 mm
      (footprints from `PRINT_PLAN.md` §8.2).
- [ ] `scripts/capture_day.sh end` — if it refuses, the rig moved during the day
- [ ] `scripts/capture_day.sh list`, then copy `captures/` to an external drive
      tonight. It is gitignored; git holds no copy

## Sun 4 Oct — verify end to end, then extras

- [ ] Pose server up, then `scripts/capture_day.sh process --day 20261003` on at
      least one T1 and one T4. It prints the ground-truth agreement, the measured
      visibility per view, a **tier check**, and the ablation table. Wrong → reshoot
      today while the setup is intact
- [ ] On T1, read visibility's `behind` column: it is how well ground truth sits
      on the object. It also sets the [VERIFY] on the 20 mm / 2 px defaults
- [ ] Extras: one T1 with the TV on (§4.2b); if Thursday was weak, 10–15 varied
      layouts with stills as a fine-tune / real-mAP set, disjoint from the scored scenes

## Mon 5 – Fri 9 Oct (evenings) — process everything while the rig is still there

- [ ] `scripts/capture_day.sh process --day 20261003` for all scenes: the headline
      number exists before you fly. Every FAILED tier check is a reshoot for Saturday
- [ ] Every object in frame (§10 4b); cam_over's unoccluded count within 2× of
      cam_front's on T1 (§10 4)
- [ ] `training/pose/registration_quality.py` on real captures (closes the
      6-iteration [VERIFY] in `STATUS.md`)
- [ ] Remote access to the desktop: Tailscale or SSH; GRUB default = Ubuntu (the
      box dual-boots); BIOS restore-on-power-loss; suspend and automatic reboots
      off. Test from a phone hotspot. This is what lets a fine-tuned detector be
      replayed from Taiwan
- [ ] Back up `captures/`, `runs/` and the ONNX model to the drive and to cloud

## Sat 10 Oct — reshoots and backup demo footage

- [ ] Reshoot every scene that failed a check this week (new scene names; a
      scene is never overwritten)
- [ ] Demo run (§7): `the_eyes_run live --record demo --show=console` plus a
      screen recording and phone video of the rig — backup footage so the video
      does not depend on 25–31 Oct
- [ ] Final backup; cover the rig; nobody touches the desk

## 11–24 Oct — Taiwan (offline, or remote to the desktop)

Real-capture mAP (boxes from ground-truth poses — not built), predicate accuracy
against the photos, bootstrap CIs by scene (proposal §7.1 — not built), the §4.3
table-plane check from the empty-table session (not built), fine-tune and replay
if needed, report draft, figures, video script.

## 25–31 Oct

Re-register; reshoot the live demo if the detector changed; edit the video;
finish the report; ZIP; submit by Sat 31 Oct.
