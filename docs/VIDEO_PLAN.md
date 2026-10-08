# Project video — plan

Written 2026-10-08. The words are in
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
3. **No recording holds a true `(in a b)`, or one object resting on another.**
   Counted over every `runs/scored/*/*/metrics.sqlite` (32 logs): no `in` fact
   at all, and every object-on-object `on` fact is a false one — an object
   "on" a tray that was taken for a bowl. In the scored set each object sits
   alone in its tray. The cube going into the bowl is therefore
   new on Saturday (§5.3), and if the report is to give an accuracy for `in`,
   the recording it is scored on has to be shot the same day (§5.2, step 9).

---

## 2. What the video has to be

| Source | Requirement |
|---|---|
| Proposal §11.1 | 10–15 minutes, `.mp4`; accepted when it "shows a live run on hardware" |
| `README.md` §4 | Problem framing, system architecture, live perception demo, occlusion ablation result, sim-to-real findings, evaluation methodology |
| `CAPTURE_PROTOCOL.md` §1, §7 | The demo run is never scored and is "an illustration, labelled as one" |
| `VIZ_PLAN.md` §4 | The console's single-view counters are a demonstration, not the ablation; the numbers come from `ablation_from_log` |
| Canvas brief | `[VERIFY]` — not in the repo. Length limit, file format and size, how it is uploaded, and whether it is one video or two (a short promotional one and a technical one) |

Three rules of the project that apply to every frame and every sentence: a
number is on screen or spoken only if it came from a run; nothing is said about
how the project is assessed; and the code is described as built with Claude
Code under the author's design and review, never as hand-written.

---

## 3. Storyboard

Lengths are the script's spoken words (1,489, counted per scene) at 130 words a
minute, plus the time the picture runs without a voice (about 100 s, half of it
the pauses of scene 6). At 115 words a minute the same script is about
14 min 50 s, which is why §8 names what to cut first. Spoken numbers take
longer than their word count: time a read-through before trusting either figure.

| # | Scene | Words | Length | Picture | Kind |
|---|---|---|---|---|---|
| 1 | One camera loses the cube | 70 | 0:40 | Phone beside console; a card hides the cube from one camera | LIVE |
| 2 | Title | 65 | 0:35 | Title card, rig photo (`assets/photos/rig_overview_20260929.jpg`) | still |
| 3 | Why one camera is not enough | 102 | 0:50 | Three simple slides (First Presentation slide 2 redrawn) | slide |
| 4 | The rig | 97 | 0:50 | Phone footage: rig, cameras, sync cable, board, objects, trays | phone |
| 5 | The pipeline | 181 | 1:30 | System diagram (First Presentation slide 8), then the console on `t2_clutter_a` with its panels labelled | slide + REPLAY |
| 6 | Live on the rig | 177 | 2:15 | One continuous take, console full frame, phone in a corner | LIVE |
| 7 | How it is measured | 124 | 1:00 | A ground-truth still with markers outlined; the 16 scenes by level | still |
| 8 | A hidden object is still found | 154 | 1:15 | `t4_cube_over`, `t4_cube_front` beside their rows; the T4 table; the cube-alone rows | REPLAY + table |
| 9 | When fusion believed the wrong camera | 182 | 1:30 | `t3_bottle_over_b` under the old rule, a line-of-sight diagram, the same scene under the new rule | REPLAY + diagram |
| 10 | Objects that are not there | 127 | 1:00 | A frame with a false object boxed in a camera view; the count before and after | REPLAY + table |
| 11 | What it still gets wrong | 114 | 1:05 | Dim frame; the bottle with its axes reversed; `t2_clutter_c`; detector table | REPLAY + table |
| 12 | Close | 96 | 0:50 | The `(:init ...)` block full screen; `ObjectState` fields; last card with the Claude Code line | still |
| | **Total** | **1,489** | **13:20** | | |

Kinds, and the label each carries on screen for as long as it is shown:

- **LIVE** — "LIVE on the rig, 10 Oct 2026, one take". Screen recording and
  phone, same moment.
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

**Shot at the rig, Sat 10 Oct** (§5): the cold-open take, the long live take,
clean copies of both for a later replay, phone footage of the rig, and one
static scene with the cube in the bowl.

**Drawn:** the three slides of scene 3, the line-of-sight diagram of scene 9,
the tables, the title and closing cards.

---

## 5. Sat 10 Oct — the shoot

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

- [ ] Every screen recording plays, shows the whole window, and its text reads
      at full screen.
- [ ] The phone clips are on the desktop's disk, not only on the phone.
- [ ] Each take's log says what ran. In its `metrics.sqlite`, table `runs`:
      `source` is `live` and `pose_backend` is `foundationpose`.
- [ ] The speed for scene 6 is read from the kept take: `pairs_processed` over
      the run's duration, and the per-stage timings in `frames`. This closes
      the `[VERIFY]` in `STATUS.md` on speed with three objects.
- [ ] The name of the kept take is written into `OCTOBER_PLAN.md`.
- [ ] Recordings, logs, screen recordings and phone clips are in the backup.

### 5.6 What has gone wrong before

cam_front's USB link dies and comes back only when the cable is reseated at
both ends; both cameras then need re-registering (6 and 7 Oct). The lamps fade.
The rig does not survive a night untouched. All three are in `OCTOBER_PLAN.md`
with what was done.

---

## 6. Rendering the replays

```bash
build/runtime/the_eyes_run check --pose foundationpose      # the server answers
mkdir -p runs/video
day=20261005; scene=t4_cube_over
build/runtime/the_eyes_run replay captures/scored/$day/$scene/session \
    --pose foundationpose --config-dir runs/scored/$day/$scene/config_recording \
    --no-log --show-record runs/video/$scene.avi
```

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
- **The camera panels draw objects the scene graph refuses.** In
  `t4_cube_over` the tray is boxed as `bowl 0.49` and drawn as `bowl_01`; in
  the bottle scene a cloth is a bowl and there is a cube at the table's edge.
  None is in the facts. Scene 10 uses this. Elsewhere, let it be seen.
- **A scored clip is ten seconds of a still scene.** Loop or hold it in the
  edit; a loop is still a REPLAY.

To stop the server, kill the Python process, not the shell that started it:
`pgrep -f "bin/python SystemCode/pose/server"`. On 8 Oct a kill of the shell
left the server running.

**Render every clip once on Saturday night**, on the detector as it is. The
rough cut can then be made in Taiwan with no access to the desktop. They are
rendered again in the last week on whatever the final detector and config are.

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
- **Rehearsal track.** A spoken guide can be generated from the script, as the
  First Presentation tracks were, to time the rough cut before the real voice
  exists. It never goes into the submitted video.
- **Export.** H.264 `.mp4` with AAC sound. Size limit `[VERIFY]` against the
  Canvas brief.

---

## 8. Schedule

| When | What |
|---|---|
| Fri 9 Oct, evening | §5.1 |
| Sat 10 Oct | The shoot (§5.2). That night: render every replay clip (§6), back everything up, copy the footage to the laptop |
| 11–24 Oct, Taiwan | Slides, diagram and tables; rough cut from Saturday's footage; the script checked against the report's tables as they settle |
| 25–31 Oct | Replays rendered again on the final detector; the live take reshot only if the detector changed what it shows; the voice recorded; final cut; export; link in `README.md` §4; submit Sat 31 Oct |

If the cut runs long, shorten in this order: scene 10 down to its last
paragraph; the detector sentence of scene 11; scene 3 down to its second
paragraph. Scenes 6, 8 and 9 are not shortened: they are the live run, the
result, and the fusion module at work.

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

- `[VERIFY]` the Canvas brief (§2).
- Speed with three objects and the depth check on, live: unmeasured until
  Saturday. On a 60-pair replay of `t2_clutter_a` (8 Oct) the per-pair p50 was
  87 ms: the detector 3.7 ms a view, FoundationPose 34–35 ms a view, the depth
  check 11 ms, fusion 0.03 ms, the scene graph 0.02 ms.
- `in` on the printed objects has never been produced on the rig. If it does
  not appear in rehearsal, the beat is cut and the fact is a finding for the
  report, not something to stage around.
- The detector on objects without trays: untried.
- `--show-record` on a live run (§1, item 2): left as it is. Giving the live
  recorder the clock is a small change to `runtime`, and nothing in this plan
  needs it.
- Scenes 10 and 11 describe the detector as it is now. If fine-tuning on the
  7 Oct layouts removes the tray-as-bowl, the script's `[VERIFY]` in scene 11
  is where that is said, and the clips of scene 10 are then the detector before
  fine-tuning and are captioned so.
