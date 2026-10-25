# Project video — plan

Written 2026-10-08; **brought up to date on 2026-10-10, after the shoot** (§5.0
says what was shot and where the day departed from this plan). The words are in
`ProjectReport/presentation/video_script.txt`; this file is the pictures, the
shoot and the schedule. Numbers quoted here are from `docs/OCTOBER_PLAN.md`,
`docs/STATUS.md`, a config file or a log named at the point of use. This plan
produces no measurements of its own, except the render tests in §6, which say
what they are.

---

## 1. The decision, first

**One video of about 13½ minutes, built around one real live take and the
scored recordings.** Twelve scenes: a live cold open, the problem, the rig, the
pipeline, the live run, how it was measured, the occlusion result, the failure
that fusion had and its fix, the objects that are not there, the limits, the
close.

**Only two things in it need the rig**: the live takes (scenes 1 and 6, about
3 minutes of screen time) and phone footage of the rig itself (scene 4). They
are shot on **Sat 10 Oct**. The rig is out of reach 11–24 Oct and 25–31 Oct is
the week of the deadline, so Saturday is the shoot and the last week is only a
reserve. Every other picture is made from files already on disk (§6).

**Three things found on 8 Oct that would have spoiled Saturday:**

1. **The demo command in `OCTOBER_PLAN.md` would have run the mock pose
   estimator.**
   `config/pose.yaml` ships `estimator.backend: "mock"`, and the line read
   `the_eyes_run live --record demo --show=console`. Every live command below
   carries `--pose foundationpose`; `OCTOBER_PLAN.md` is corrected.
2. **`--show-record` on a live run plays back too fast.** It writes one frame
   per *processed* pair into a file stamped 30 fps (`the_eyes_run.cpp:544`,
   `Console::Show`). At ten pairs a second — the strip read 10.2 on the live
   frame of 29 Sep — the file would run three times faster than the table did.
   Read from the code, not measured on a live run.
   On a replay of a 30 pairs/s recording the same file is in true time (§6).
   So the live picture comes from a **screen recording**, which keeps the
   clock, and `--show-record` is for replays only.
   **10 Oct: it was used for the live takes after all.** The monitor is in both
   cameras' pictures and has to stay dark, so no window could be shown or
   recorded. The kept long take is 2,231 frames for 184.9 s of table time: it
   plays 2.5 times too fast and is retimed in the edit (§7).
3. **No recording holds a true `(in a b)`, or one object resting on another.**
   Counted over every `runs/scored/*/*/metrics.sqlite` (32 logs): no `in` fact
   at all, and every object-on-object `on` fact is a false one — an object
   "on" a tray that was taken for a bowl. In the scored set each object sits
   alone in its tray. The cube going into the bowl is therefore
   new on Saturday (§5.3), and if the report is to give an accuracy for `in`,
   the recording it is scored on has to be shot the same day (§5.2, step 9).
   **10 Oct: shot.** `t2_in_a` and `t2_in_b` hold `(in cube bowl)` on 300 of
   300 pairs.

---

## 2. What the video has to be

| Source | Requirement |
|---|---|
| Proposal §11.1 | 10–15 minutes, `.mp4`; accepted when it "shows a live run on hardware" |
| `README.md` §4 | Problem framing, system architecture, live perception demo, occlusion ablation result, sim-to-real findings, evaluation methodology |
| `CAPTURE_PROTOCOL.md` §1, §7 | The demo run is never scored and is "an illustration, labelled as one" |
| `VIZ_PLAN.md` §4 | The console's single-view counters are a demonstration, not the ablation; the numbers come from `ablation_from_log` |
| Module brief (`requirement/`, read 25 Oct; not in git) | One video presentation of **10–15 minutes**, as `.mp4`, `.mov` or `.wmv`. It is the Final Presentation. It goes in the team's one ZIP to Canvas → Assignments → Practice Module, due 31 Oct, beside the slides of both presentations. The brief gives no size limit |

Three rules of the project that apply to every frame and every sentence: a
number is on screen or spoken only if it came from a run; nothing is said about
how the project is assessed; and the code is described as built with Claude
Code under the author's design and review, never as hand-written.

---

## 3. Storyboard

Lengths are the script's spoken words (1,578 in draft 3 of 17 Oct, counted
per scene; 1,577 after the shoot, 1,489 before it) at 130 words a minute, plus
the time the picture runs without a voice (about 110 s, half of it the pauses of scene 6). At 115 words a
minute the same script is about 15 min 30 s, and **below about 120 it passes
15 minutes**, which is why §8 names what to cut first. Spoken numbers take
longer than their word count: time a read-through before trusting either figure.

| # | Scene | Words | Length | Picture | Kind |
|---|---|---|---|---|---|
| 1 | One camera loses the cube | 70 | 0:40 | Phone beside console; a card hides the cube from one camera (`demo_cold_d`) | LIVE |
| 2 | Title | 65 | 0:35 | Title card, rig photo (`assets/photos/rig_overview_20260929.jpg`) | still |
| 3 | Why one camera is not enough | 103 | 0:50 | Three simple slides (First Presentation slide 2 redrawn) | slide |
| 4 | The rig | 96 | 0:50 | Phone footage: rig, cameras, board, objects, trays | phone |
| 5 | The pipeline | 186 | 1:30 | System diagram (First Presentation slide 8), then the console on `t2_clutter_a` with its panels labelled | slide + REPLAY |
| 6 | Live on the rig | 208 | 2:30 | One continuous take (`demo_live_a`), console full frame, phone in a corner, four captions | LIVE |
| 7 | How it is measured | 145 | 1:10 | A ground-truth still with markers outlined; the 16 scenes by level; the cube in the bowl from above | still |
| 8 | A hidden object is still found | 159 | 1:15 | `t4_cube_over`, `t4_cube_front` beside their rows; the T4 table; the cube-alone rows; the all-16 line | REPLAY + table |
| 9 | When fusion believed the wrong camera | 175 | 1:25 | `t3_bottle_over_b` under the old rule, a line-of-sight diagram, the same scene under the new rule | REPLAY + diagram |
| 10 | Objects that are not there | 146 | 1:10 | A false object boxed by the synthetic-only detector, then the same scene fine-tuned; the counts before and after | REPLAY + table |
| 11 | What it still gets wrong | 129 | 1:10 | `t2_clutter_c`; the live take's wrong `occluded_from`, then the predicate table; detector table on the real scenes | REPLAY + table |
| 12 | Close | 96 | 0:50 | The `(:init ...)` block full screen; `ObjectState` fields; last card with the Claude Code line | still |
| | **Total** | **1,578** | **14:00** | | |

Kinds, and the label each carries on screen for as long as it is shown:

- **LIVE** — "LIVE on the rig, 10 Oct 2026, one take". The console as the
  runner wrote it during the take, and the phone, same moment.
- **REPLAY** — "REPLAY of a recorded scene". The console rendered from a
  recording, by the same code.
- Slides, stills and tables carry their source in small print.

---

## 4. What exists, what is rendered, what is shot

**Exists.** Three rig photos in `assets/photos/`; the First Presentation deck
(`ProjectReport/PRS-PM-2026-AIS08-GRP-12-TheEyes-FirstPresentation.pptx`, 18
slides); the 16 scored scenes and four others of 5–7 Oct with their recordings,
1920×1080 ground-truth stills and visibility overlays
(`captures/scored/2026100{5,6,7}/`, `runs/scored/…`); every result table in
`OCTOBER_PLAN.md`.

**Rendered from disk** (§6) — any day the desktop and the pose server are
reachable:

| Scene | Recording | How |
|---|---|---|
| 5 | `20261007/t2_clutter_a` | as processed |
| 8 | `20261005/t4_cube_over`, `20261005/t4_cube_front` | as processed |
| 9 | `20261006/t3_bottle_over_b`, twice | once as processed; once on a copy of the scene's config with `view_evidence.enabled: false` |
| 10 | `20261006/t3_bottle_over_b` (a cloth boxed as a bowl, a cube at the table edge), `20261005/t3_cube_front` (a cube on the card's print) | as processed |
| 11 | `20261005/dim_t1_cube`, `20261006/t1_bottle_b`, `20261007/t2_clutter_c` | as processed |

**Shot at the rig, Sat 10 Oct** (§5.0): the cold-open take, the long live take,
a clean copy of the long one for a later replay, phone footage of the rig, and
two scored scenes with the cube in the bowl.

**Drawn:** the three slides of scene 3, the line-of-sight diagram of scene 9,
the tables, the title and closing cards.

---

## 5. Sat 10 Oct — the shoot

### 5.0 What was shot (written 10 Oct, after it)

The record, with every number, is `docs/OCTOBER_PLAN.md`, "Sat 10 Oct". For the
edit:

| Use | Take | Files |
|---|---|---|
| Scene 1 | **`demo_cold_d`** — the cube alone, the card with its plain back to cam_over. 1,048 pairs, 21.0 pairs/s | `runs/video/demo_cold_d.avi`, phone `PXL_20261010_135050089.mp4` |
| Scene 6 | **`demo_live_a`** — the whole sequence of §5.3. 2,231 pairs in 184.9 s, 12.1 pairs/s | `runs/video/demo_live_a.avi`, phone `PXL_20261010_133258777.mp4` |
| A later replay | **`demo_clean_a`** — the same sequence, recording only, 4,200 pairs, none dropped | `captures/runtime/20261010_214147_demo_clean_a`, phone `PXL_20261010_134121078.mp4` |
| Spares | `demo_cold_a`, `_b`, `_c`; `demo_live_b` | `runs/video/` |
| Scene 4 | eight rig clips, 22 photos; the console on the monitor (a replay, not a live shot) | `runs/video/phone/` |

Each live take's recording is `captures/runtime/<stamp>_<take>/` and its log
`runs/runtime/<stamp>_<take>/metrics.sqlite`. None of this is in git.

Where the day departed from the plan below, which is left as it was written:

- **No screen recording and no window.** The monitor could not be moved out of
  both cameras' pictures, so it stayed dark and the runner wrote the console
  file itself: `env -u DISPLAY the_eyes_run live --pose foundationpose --record
  NAME --show-record runs/video/NAME.avi` under
  `runs/scored/helpers/with_blank.sh`. The file needs retiming (§7).
- **A card on a stand, not a hand, and its plain dark back to the camera.**
  Skin is read as a `red_cube`, and so is the red in the card's print; each
  false detection costs a pose registration of about a second.
- **No trays** in the takes. The detector found the three objects on the bare
  table (scores 0.86–0.94 in `demo_live_a`).
- **The cube-in-bowl beat stayed**: `(in red_cube_04 bowl_04)` on 57 pairs.
- **The runner starts about 10 s after the monitor goes dark**: count to twelve
  before the first tap.
- **The sync cable was not connected** (broken, then lost), so it is in no
  phone shot and the script no longer claims it.
- **Three calibrations**, both cameras' USB links having dropped during the
  day; no closing board registration; no backup, by the author's decision.

### 5.0a The plan as written on 8 Oct

About 90 minutes at the rig (the steps of §5.2 sum to 88). `OCTOBER_PLAN.md`
allowed 30, which was one take and no checks.

### 5.1 Friday evening, nothing moved on the rig

- [ ] **Screen recorder.** OBS is installed (`/usr/bin/obs`); the session is
      X11, so a window capture of `The Eyes - console` works. Canvas 1920×1080,
      30 fps, MKV. Test it without the rig: start the pose server, then
      `build/runtime/the_eyes_run replay captures/scored/20261007/t2_clutter_a/session --pose foundationpose --config-dir runs/scored/20261007/t2_clutter_a/config_recording --no-log --show=console --realtime`,
      record ten seconds, play the file back at full screen and read the
      smallest text.
- [ ] **Phone.** Landscape, 1080p at 30 fps, focus and exposure locked, on a
      stand. One framing that holds the table and both hands; the screen in the
      same frame if it fits.
- [ ] **The screen the console is on must be outside both cameras' view.**
      Every scored recording was made with the monitor blanked
      (`runs/scored/helpers/with_blank.sh`); the live take cannot be. A camera
      that sees the console sees pictures of the objects, and the detector has
      been caught on screen content before (`OCTOBER_PLAN.md` finding 6). Turn
      or shield the monitor, or put the window on another screen. TV off.
- [ ] **Light.** The LED lamps fade within an hour (`OCTOBER_PLAN.md`, 6 Oct).
      Charge them, or shoot between 17:15 and 18:30 under the room light.
- [ ] **The occluder.** A hand first; the printed card second. The card's red
      print is read as a cube (5 Oct): use its plain side.
- [ ] **Disk.** A recording is about 1.5 MB a pair (455 MB for 300). A
      three-minute live take is about 3 GB, a one-minute clean take the same.
      182 GB were free on `/home` on 8 Oct.
- [ ] Read the script aloud once against a clock and note the pace.

### 5.2 Order at the rig

| # | Step | Min |
|---|---|---|
| 0 | `scripts/capture_day.sh begin` — board on, re-register, empty table. **Film the board on the table with the phone while it is there** (scene 4) | 12 |
| 1 | Board off, then check that neither camera moved (the board coming off is when they get bumped: 7 Oct) | 3 |
| 2 | Light: `runs/scored/helpers/lightcheck.sh demo` (it blanks the monitor); cam_over's frame mean above the floor of 45. Then the view, *with the monitor on as it will be for the take*: `build/runtime/the_eyes_run live --record-only demo_check --frames 15`, and look at one frame from each camera — no screen in either | 4 |
| 3 | Phone footage of the rig (§5.4) | 10 |
| 4 | Pose server up from the repo root; `build/runtime/the_eyes_run check --live --pose foundationpose` passes | 3 |
| 5 | Rehearse §5.3 once, not recorded. Decide two things: trays on the table or not, and whether the cube-in-bowl beat stays | 8 |
| 6 | Cold-open take, twice | 5 |
| 7 | Long live take, twice | 15 |
| 8 | Clean takes of both, recording only | 8 |
| 9 | Cube in the bowl, static: a 300-pair recording and a phone photo from above | 5 |
| 10 | §5.5 checks, `scripts/capture_day.sh end`, backup, cover the rig | 15 |

```bash
# pose server -- from the repo root, with no --asset-root
~/venvs/the-eyes-r1/bin/python SystemCode/pose/server/foundationpose_server.py \
    --foundationpose-root ~/src/FoundationPose

# LIVE take: processed as it happens, console on screen, the pairs it processed kept.
# OBS records the window; the phone films the table. q in the window stops it.
build/runtime/the_eyes_run live --pose foundationpose --record demo_live_a --show=console

# CLEAN take: recording only, every pair at 30 a second, no console.
# 1800 pairs is one minute. Rendered later through replay, in true time.
build/runtime/the_eyes_run live --record-only demo_clean_a --frames 1800
```

Sessions land in `captures/runtime/<stamp>_<name>/`, logs in
`runs/runtime/<session>/metrics.sqlite`. A recording-only take longer than 300
pairs has not been made before: watch its output for
`recorder: queue full, dropped pair`, and keep each one to a minute.

Why both kinds of take. The live take is the proof: what it shows happened at
that speed, and the phone shows it happening. But it keeps only the pairs it
had time to process, so it cannot be re-rendered smoothly, and it is tied to
the detector of 10 Oct. The clean take holds every pair, so it can be replayed
after the detector is fine-tuned — as a replay, labelled as one.

### 5.3 The long take, beat by beat

It follows scene 6 of the script. After each beat the hand leaves both views
completely before the next one: a hand left in view is a moving occluder, and a
track nobody sees is kept for one second only (`ekf.max_coast_s`).

| Beat | Action | Hold | What should appear |
|---|---|---|---|
| 0 | Tap the table twice, in view of the phone and both cameras | — | The sync point for the edit |
| 1 | Empty table | 5 s | `(clear table_01)` and nothing else |
| 2 | Put down the bottle | 5 s after its facts | `(on mustard_bottle_01 table_01)`, `(clear mustard_bottle_01)`. The first pose takes 1–4 s |
| 3 | The bowl, then the cube | 5 s each | Three objects, each `on` the table |
| 4 | Cube into the bowl; later take it out | 8 s | `(in red_cube_01 bowl_01)` — **only if step 5 of §5.2 showed it** |
| 5 | A hand between cam_over and the cube | 5 s | cam_over's counter one lower, the fused counter unchanged, `(occluded_from red_cube_01 cam_over)` |
| 6 | The same for cam_front | 5 s | The mirror of beat 5 |
| 7 | Lift the bottle away | 5 s | Its facts gone |
| 8 | Tap twice | — | End sync |

The cold-open take is beats 0, 5 and 8 with the cube alone, and the card in
place of the hand if the hand does not hide enough of it. It is the live
version of `t4_cube_over`, where cam_over finds the cube on 0 of 300 pairs and
the fused output holds it at 2.97 mm.

**Trays or no trays** is decided in rehearsal. Without them there is no tray to
be boxed as a bowl (finding 3), and a demo has no ground truth to need them.
But the detector has not been run on the printed objects standing on the bare
table. If it finds them less well there, the trays go back and the false bowl
is left on screen: the scene graph does not report it, and scene 10 is about
exactly that.

What the take will also show, and keeps: the detector flickering, and a pose
drifting while a hand is over it (`CAPTURE_PROTOCOL.md` §7). One sentence of
narration covers either.

### 5.4 Phone footage of the rig

Ten seconds each, the phone still: the whole rig from behind the desk; each
camera close up; the sync cable between them; the board on the table during
registration (step 0); the three objects; an object being seated in its tray;
the lamps; the screen with the console running. Then one slow move from the
table up to a camera.

### 5.5 Before the rig is covered

- [x] Every console file decodes to its last frame and holds one frame for
      each processed pair (six live takes counted). Text at full screen: not
      yet judged on a player.
- [x] The phone clips are on the desktop's disk: `runs/video/phone/`, 38 files,
      each opened to its last frame.
- [x] Each take's log says what ran: `source` is `live` and `pose_backend` is
      `foundationpose` in all seven.
- [x] The speed for scene 6 is read from the kept take: 12.1 pairs a second
      over it; per pair 34 / 64 / 91 ms with one / two / three objects tracked.
      `STATUS.md` still carries its `[VERIFY]`; this is the number for it.
- [x] The names of the kept takes are in `OCTOBER_PLAN.md`.
- [ ] Recordings, logs, console files and phone clips are in the backup — not
      done, by decision: they stay on the desktop's disk.

### 5.6 What has gone wrong before

cam_front's USB link dies and comes back only when the cable is reseated at
both ends; both cameras then need re-registering (6 and 7 Oct). The lamps fade.
The rig does not survive a night untouched. All three are in `OCTOBER_PLAN.md`
with what was done.

### 5.7 The reshoot, Sun 25 Oct — the fine-tuned detector

Decided 17 Oct. Scenes 1 and 6 are the video's only live footage, and both ran
the synthetic-only detector of 10 Oct: the false cubes boxed on the card's
print and the wrong `occluded_from` on the bottle belong to that day's system,
not the one submitted. The reshoot puts the submitted system on camera.
Nothing else is reshot: the scored scenes are the measurement and stay as
recorded, and scene 4's rig footage does not depend on the detector.

**What the new detector does on the 10 Oct recordings**, replayed 17 Oct
(`runs/video/ft_preview/`, own databases):

- `demo_live_a`: the bottle's wrong `occluded_from` is gone (500 pairs → 0).
  But cam_front still boxes the red marks on the card's print as a cube
  (scores 0.27–0.39), and when the card came away that track grew onto the
  real cube (×3.56 in area). Tracking only refines the last pose, so the pose
  fitted to the card was kept, and `(occluded_from red_cube cam_front)` stayed
  on from 88 s to the end: 1,438 pairs, against 144 with the card in place.
  **Fixed 17 Oct** (`config/pose.yaml`, `estimator.reregister_area_ratio: 3`):
  a track whose box has grown or shrunk more than 3× in area since it was
  registered is registered again. With it the fact is on 144 pairs, the
  card's. It never fires on the detections of the 22 scored scenes, and
  `t3_bottle_over_b` replayed with it matches its 17 Oct run pair for pair.
- `demo_clean_a` shows two faults that fix does not reach. The new detector
  boxes an object while the hand is still putting it down, FoundationPose
  registers on that moving, partly hidden view (own-depth support 0.02–0.08),
  and tracking never recovers: at 30 s all three objects stand in plain view
  of both cameras and three `occluded_from` facts are wrong. And at 44 s the
  cube is lifted out of the bowl faster than tracking follows: its box goes
  with it, its pose stays in the bowl, and `(in red_cube bowl)` holds to 65 s.
  The old detector's live take shows neither; whether it simply did not box
  objects in the hand is not checked.
  **Fixed 17 Oct** (`estimator.reregister_min_box_cover: 0.3`,
  `reregister_after_off_box: 5`): a track whose estimate covers less than 0.3
  of its own detection box on 5 estimates running is registered again, the
  count doubling after each. Both faults leave the pose off its box (cover
  0.19 for the bottle set down, 0.00 for the cube), while a box on the visible
  part of a half-hidden object lies inside the pose's footprint, so an
  occluder that stays put does not trigger it; one that moves can. Of the
  eight registrations it forces on the clean take, five are the faults (5,
  25, 35 twice, 44 s) and three come as the card sweeps across the bottle (56,
  67, 78 s). The bowl, boxed together with the hand carrying it in, is caught
  by the 3× rule.
  Replayed with both rules (`runs/video/ft_preview/fix3/`): every
  `occluded_from` left in `demo_clean_a` falls where the card, a hand or an
  arm hides the object (frames checked), `(in red_cube bowl)` holds 34–44 s,
  and `demo_live_a` is unchanged from the 3× rule alone. The 22 scored scenes
  match their 17 Oct runs pair for pair, so no reported number moves.
  Tried first and dropped: registering again after 15 refusals in a row by the
  depth check. It mended the clean take as well, but moved the per-view
  estimates of four T3 scenes, and with the card before cam_front it
  re-registered the half-hidden bowl and bottle onto their visible parts, so
  their true `occluded_from` went missing (patch kept in `fix3/`).

**Before the rig** (remote, any day before):

- [ ] `main` holds the 17 Oct re-registration rules, `build/` is built from
      it, `config/detector.yaml` names `yolov8_the_eyes_ft_real_v1.onnx`, and
      `config/pose.yaml` has `reregister_area_ratio: 3.0` and
      `reregister_min_box_cover: 0.3`.
- [ ] Disk: 155 GB free on 17 Oct; the takes below are about 16 GB.
- [ ] Lamps charged: they fade within an hour.
- [ ] Phone as on 10 Oct: landscape, 1080p at 30 fps, focus and exposure
      locked, the same stand and framing (`PXL_20261010_133258777.mp4` is the
      reference).

**Order at the rig**, about an hour. The author places the objects; the
commands are driven from the session, as on 10 Oct.

| # | Step | Min |
|---|---|---|
| 0 | Pose server up from the repo root, then `build/runtime/the_eyes_run check --live --pose foundationpose`. The rig has stood two weeks with the cats about: a knocked camera or a loose cable shows here first. Keep its output — it is also the `check --live` of proposal §11.1 | 5 |
| 1 | `scripts/capture_day.sh begin`: board on, re-register, empty table. A refused registration means recalibrate, and commit the new extrinsics, before any take | 12 |
| 2 | Board off; check that neither camera moved | 3 |
| 3 | `runs/scored/helpers/lightcheck.sh demo`: cam_over's frame mean above 45 | 3 |
| 4 | Rehearse §5.3 once, not recorded | 5 |
| 5 | Cold open, twice: `runs/scored/helpers/demo_take.sh demo_cold_e 60`, then `demo_cold_f`. The cube alone; the card on its stand, plain back to cam_over | 6 |
| 6 | Long take, twice: `demo_take.sh demo_live_c 200`, then `demo_live_d`. The beats of §5.3, with the card on its stand in place of the hand, as on 10 Oct | 15 |
| 7 | Clean take, once: `demo_take.sh demo_clean_b 150 clean` | 5 |
| 8 | Read each take's summary (`demo_take.sh` prints it); `scripts/capture_day.sh end`; cover the rig | 8 |

The monitor going dark is the start cue: count to twelve before the first tap
(§5.0).

**A take is kept when** its log names `yolov8_the_eyes_ft_real_v1.onnx` and
its `runs` row reads `source = live`, `pose_backend = foundationpose`; every
beat of §5.3 shows in `demo_summary.py`'s fact timeline; no `occluded_from`
names an object that nothing hides; and the phone clip runs from the first tap
to the last. If neither long take passes, scene 6 keeps `demo_live_a` with its
captions, which are honest as they stand. There is no second reshoot day.

**After it**, no rig needed:

- Retime the kept takes: `scripts/retime_console.py runs/video/demo_live_c.avi
  demo_live_c runs/video/retimed/demo_live_c.mp4`, and the cold open the same.
- Script draft 4. Only these change, each read from the new take's summary,
  and the spoken ones shown to the author before they go in: the take names in
  the notes of scenes 1 and 6; scene 6's speed caption and its spoken "10 frame
  pairs a second" and "about four fifths"; scene 6's bottle caption ("The third
  fact is wrong") and card caption, each dropped if the take no longer shows
  it; scene 11's "though the take predates the fix"; the sources for scene 6.
  The "LIVE … the detector before fine-tuning" caption loses its last words.
- `OCTOBER_PLAN.md` gets a 25 Oct section with the takes table, as for 10 Oct;
  `STATUS.md`'s speed `[VERIFY]` is closed from the kept take.
- The narration is recorded after draft 4, all twelve scenes in one sitting,
  same room and same phone, so the voice sounds the same throughout.

**Shot, Sun 25 Oct.** The record with every number is `docs/OCTOBER_PLAN.md`,
25 Oct, and `captures/scored/20261025/day.log`. The rig was recalibrated first
(9eb524c) and held to the end. The takes are on the bare table, with no trays.

| Use | Take | Files |
|---|---|---|
| Scene 1 | **`demo_cold_f`**, the cube alone, the card before cam_over. 1,325 pairs, 24.2 pairs/s. Spare: `demo_cold_e` (24.8 pairs/s; the cube is put down on camera) | `runs/video/retimed/demo_cold_f.mp4`, phone `PXL_20261025_075511615.mp4`; `demo_cold_e.mp4`, `PXL_20261025_074919969.mp4` |
| Scene 6 | **`demo_live_d`**, the whole sequence of §5.3. 2,598 pairs in 194.8 s, 13.3 pairs/s | `runs/video/retimed/demo_live_d.mp4`, phone `PXL_20261025_080801961.mp4` |
| Scene 11 | the frame at 91 s of `demo_live_d` (the bowl counted twice) | as scene 6 |
| Spares | `demo_live_c` (a false `occluded_from` on 54 pairs: a pose registered on the hand), `demo_live_e` (3 one-pair duplicates), `demo_clean_b` (150 s, recording only; its replay has the cam_front bottle wrong on 657 pairs) | `runs/video/`, `captures/runtime/` |

- **The keep rule, strictly, passed no long take.** `demo_live_d` has one pair
  on which an `occluded_from` names a copy of the bowl that nothing hides. It
  is kept in place of the fallback `demo_live_a`, because `demo_live_a` runs
  the detector this reshoot was made to replace. The pair is captioned in
  scene 6 and named in scene 11.
- **Another take would have been a lottery.** The fault comes from a hand
  passing an object (final report §11). The rehearsal and takes c, d and e had
  0, 1, 1 and 3 such pairs.
- **The taps cannot be heard** on the phone. The edit syncs on the hand
  touching the table.
- **The console's scene-graph panel** draws the `in` edge from the cube to the
  bowl through the bottle's node. Caption it if the narration points there.

---

## 6. Rendering the replays

```bash
build/runtime/the_eyes_run check --pose foundationpose      # the server answers
mkdir -p runs/video
day=20261005; scene=t4_cube_over
build/runtime/the_eyes_run replay captures/scored/$day/$scene/session \
    --pose foundationpose --config-dir runs/scored/$day/$scene/config_recording \
    --repo-root "$PWD" \
    --no-log --show-record runs/video/replay_${day}_$scene.avi
```

`--repo-root` is needed whenever `--config-dir` points at a scene's copy:
without it the meshes are looked for under `runs/scored/…/assets/` and the
replay stops (found 10 Oct; the tests of 8 Oct below predate the copies' use).

Each scene is rendered on the calibration it was shot on, which is what
`config_recording` holds. `--no-log` keeps the render out of the scene's
metrics. By the code no window is opened when `DISPLAY` is unset, so it should
run over SSH; the tests below were run on the desktop itself.

**Tested on 8 Oct**, 60 pairs each, the files looked at frame by frame:

| Recording | Result |
|---|---|
| `t4_cube_over` | 60 frames, 1696×1024, MJPG in AVI, 8.7 MB, 8.5 s to render. Facts on screen: `(clear red_cube_01)`, `(occluded_from red_cube_01 cam_over)`, `(on red_cube_01 table_01)` |
| `t2_clutter_a` | 60 frames, 10.1 MB, 21.6 s. Three objects, seven facts |
| `t3_bottle_over_b`, shipped config | The fused axes sit on the bottle; the three true facts |
| `t3_bottle_over_b`, `view_evidence.enabled: false` | The fused axes stand off the bottle at the cover; the only fact is `(clear table_01)` |

So scene 9's before-and-after is a config switch and two renders, with nothing
drawn by hand. A 300-pair scene comes to about 43–51 MB (five times the files
above) and, since the recordings are 29.98 pairs a second (`index.csv` of
`t2_clutter_a`), its own ten seconds of real time.

What the renders show that the edit must not hide or misread:

- **The frame rate on the status strip of a replay is not the system's speed.**
  It is the replay's own rate and it falls at every FoundationPose
  registration: 4.3 fps is on screen in the bottle render. Speed is claimed
  from the live take only.
- **The strip reads `GT: none`.** The console never sees ground truth. Numbers
  beside a clip come from `ablation_from_log`.
- **The camera panels draw objects the scene graph refuses.** In the clips of
  the synthetic-only detector (`runs/video/synthetic_detector/`), `t4_cube_over`
  boxes the tray as `bowl 0.49` and draws it as `bowl_01`; in the bottle scene
  a cloth is a bowl and there is a cube at the table's edge. None is in the
  facts. Scene 10 uses these clips. With the fine-tuned detector no false
  object is reported in the 22 scenes and false tracks fell from 6,747 to 480,
  but its clips in `runs/video/` have not been watched frame by frame for boxes
  like these: where one shows, let it be seen.
- **A scored clip is ten seconds of a still scene.** Loop or hold it in the
  edit; a loop is still a REPLAY.

To stop the server, kill the Python process, not the shell that started it:
`pgrep -f "bin/python SystemCode/pose/server"`. On 8 Oct a kill of the shell
left the server running.

**Render every clip once on Saturday night**, on the detector as it is. The
rough cut can then be made in Taiwan with no access to the desktop. They are
rendered again in the last week on whatever the final detector and config are.

**Done 10 Oct:** eleven clips, 300 frames each, 481 MB, in `runs/video/` as
`replay_<day>_<scene>.avi` — the nine of the table in §4 (the half-hidden
bottle twice, the second as `…_byscore`) and the two `in` scenes. The pair for
scene 9 ends on `(clear table_01)` alone under the old rule and on the bottle's
three true facts under the new one.

---

## 7. Edit and export

- **Tools.** This desktop has OBS and nothing else: no `ffmpeg`, no editor, no
  `sqlite3` command (`sudo apt install ffmpeg kdenlive sqlite3`). The edit can
  equally be done on the laptop; then the footage has to be on it, or in the
  cloud, before 11 Oct.
- **Conversion.** The replay files are MJPG in AVI so that a render is
  reproducible (`VIZ_PLAN.md` §8). For the editor:
  `ffmpeg -i x.avi -c:v libx264 -crf 18 -pix_fmt yuv420p x.mp4`.
- **Frame.** 1920×1080 at 30 fps. The console is 1696×1024 and goes in at its
  own size. Its text is small on a full frame, so when the narration names a
  panel, the picture moves in on that panel.
- **Voice.** Recorded scene by scene, the picture cut to the voice. Scene 6 is
  the other way round: the voice follows the take.
- **Sync of phone and screen** in the live scenes: the two taps of beat 0.
- **Retiming the live console files.** Each holds one frame for each processed
  pair in a file stamped 30 fps. Give every frame the time of its pair,
  `frames.stamp_ns` in the take's `metrics.sqlite`, before anything is cut to
  it; the pauses of about a second, where the pose estimator registers a new
  object, are then held frames and belong in the picture.
- **Rehearsal track.** A spoken guide can be generated from the script, as the
  First Presentation tracks were, to time the rough cut before the real voice
  exists. It never goes into the submitted video.
- **Export.** H.264 `.mp4` with AAC sound. The brief gives no size limit;
  check the Canvas assignment page before the ZIP is built.

---

## 8. Schedule

| When | What |
|---|---|
| Fri 9 Oct, evening | §5.1 |
| Sat 10 Oct | The shoot — **done** (§5.0), and every replay clip rendered (§6). Not backed up and not copied to the laptop: the footage is on the desktop, reached by AnyDesk |
| 11–24 Oct, Taiwan | Slides, diagram and tables; rough cut from Saturday's footage; the script checked against the report's tables as they settle |
| Sun 25 Oct | The reshoot of scenes 1 and 6 on the fine-tuned detector (§5.7), and `check --live`. Replays were already rendered on that detector on 17 Oct |
| 26–31 Oct | Script draft 4 from the new takes; the voice recorded; final cut; export; link in `README.md` §4; submit Sat 31 Oct |

If the cut runs long, shorten in this order: scene 10's first paragraph down
to its first sentence (the picture shows the false objects); scene 3 down to its
second paragraph. Scenes 6, 8 and 9 are not shortened: they are the live run,
the result, and the fusion module at work. Nor is the last paragraph of scene
11 any longer: on the real recordings it is the evidence for the depth
channel.

---

## 9. Acceptance criteria

1. The file is an `.mp4` between 10:00 and 15:00.
2. It holds at least one continuous take, labelled LIVE, in which the phone and
   the console show the same event; and that take's `runs` row reads
   `source = live`, `pose_backend = foundationpose`.
3. Every replay is labelled REPLAY for as long as it is on screen.
4. Every number spoken or shown is in the script's list of sources and equals
   the final report's figure. No `[VERIFY]` is left in the script.
5. Speed is stated once, from the live take's log.
6. No sentence is about how the project is assessed. The Claude Code line is
   spoken and is on the last card.
7. `README.md` §4 points at the video.

---

## 10. Open

- ~~The Canvas brief.~~ Read 25 Oct (§2). The ZIP size limit is still unknown; it is not in the brief.
- ~~Speed with three objects and the depth check on, live.~~ Measured 10 Oct
  on `demo_live_a`: 90.7 ms a pair (p50), 10 pairs a second; pose 36.6 + 35.1 ms
  of it, the detector 3.8 + 4.0, the depth check 10.7.
- ~~`in` on the printed objects.~~ Produced 10 Oct, scored and live.
- ~~The detector on objects without trays.~~ Tried 10 Oct; it finds them.
- **The kept live take shows a fact that is wrong:**
  `(occluded_from mustard_bottle_01 cam_front)` with nothing hiding the bottle
  (`OCTOBER_PLAN.md`, 10 Oct). Fixed on 11 Oct: on a replay of the take the
  fact is on 1 pair, not 500. The live picture was written before the fix and
  keeps it; the script says so by caption in scene 6 and in words in scene 11.
  If the edit would rather show the corrected facts, a REPLAY of the take's
  own recording (`captures/runtime/20261010_213321_demo_live_a`) has them.
- `--show-record` on a live run (§1, item 2): used on 10 Oct, so the retiming
  of §7 is now needed. Giving the live recorder the clock is still a small
  change to `runtime`, but it would not help takes already made.
- The script is over 15 minutes below about 120 words a minute (§3); it was
  113 before the shoot added 88 words. Draft 3 (17 Oct) is one word longer.
- ~~Scenes 10 and 11 describe the detector as it is now.~~ The fine-tuned
  detector was adopted on 17 Oct and every scored scene replayed with it
  (`OCTOBER_PLAN.md`, 17 Oct); draft 3 of the script carries its numbers, and
  scene 11's `[VERIFY]` is gone. Scene 10 now says the false objects were the
  first detector's, so its clip of the tray-as-bowl comes from
  `runs/video/synthetic_detector/` and is captioned "synthetic-only detector";
  the other replay clips in `runs/video/` were rendered again with the
  fine-tuned one. The live takes (scenes 1 and 6) ran the first detector, and
  scene 6's caption says so.
