# Visualisation & showcase — implementation plan

What to build so the finished system can be *seen*, and in what order.
Written 2026-09-08. Nothing here is built yet; `docs/TWIN_PLAN.md` (2026-09-07)
is the sibling plan for exhibit 3 below and stays valid as written.

Sources: every measured number quoted is from `docs/STATUS.md` or a config
file, marked at the point of use. This plan produces no measurements of its own.

---

## 1. The decision, first

**Build the console (§3). Do not lead with the digital twin.**

The instinct is to reach for the twin — Isaac Sim, CAD meshes standing on a
virtual table, obviously impressive. It is the wrong centrepiece, for one
reason:

> **A twin cannot show fusion working.** A scene rendered at the fused poses
> looks exactly as good as a scene rendered from one camera's poses. It renders
> the *output* of the pipeline and is blind to the *delta* that is the graded
> contribution.

The headline result is the occlusion ablation (`README.md` §2, `CLAUDE.md` MVP
item 4, and the one thing never cut). The second graded contribution is the
symbolic bridge — poses becoming `(on red_cube_01 table_01)`. Neither is visible
in a rendered scene. Both are visible in a view that puts single-view beside
fused, and geometry beside the facts it produced.

So the showcase is a **single-window instrument panel that renders the argument**,
not a picture of the room. The twin is then a supporting exhibit for a different
and narrower claim (`SceneState` is a sufficient world-state encoding), which is
exactly how `TWIN_PLAN.md` §1 already frames it.

Ordering, given `README.md` §4 requires the video to be "a live run on hardware,
not a slideshow of results":

| # | Exhibit | Cost | What it evidences | Status |
|---|---|---|---|---|
| 1 | **Console** (§3) | ~1 day | The whole chain, live, in one frame | this plan |
| 2 | **Occlusion A/B in-console** (§4) | ~3 h | The headline claim, live rather than replayed | this plan |
| 3 | Digital twin | ~2 days | `SceneState` sufficiency | `TWIN_PLAN.md` |
| 4 | Offline result pages (§7) | ~3 h | Report figures from `metrics.sqlite` | this plan |

---

## 2. What already exists

`SystemCode/runtime/src/viewer.cpp` (`--show`) is roughly a third of exhibit 1
already, and its structure is the right one:

- both views side by side, detections boxed with class/score/track id;
- fused `pose_w` projected back into each camera through the real
  `T_world_cam` and drawn as an RGB axis triad — a reprojection, so it shows
  agreement between the world estimate and the pixels it came from;
- a per-view and whole-pair timing HUD.

It is constructed at `the_eyes_run.cpp:365` and is a **sink, never a stage**.
That property is load-bearing and §3.4 keeps it.

What it does not show — and therefore what the console adds:

1. no world-frame view, so the poses are only ever seen as projections;
2. no per-view estimates, so fusion is invisible: you see the answer, never the
   two inputs or the disagreement between them;
3. no scene graph and no PDDL, so the entire `scenegraph` module — half the
   from-first-principles work under `CLAUDE.md` constraint 2 — appears nowhere;
4. no uncertainty, though `position_sigma_m` is computed every frame and carried
   on every `ObjectState`;
5. no way to capture what it drew, so a demo video is a screen recording.

---

## 3. Exhibit 1 — the console

One OpenCV window, `--show=console`. Plain C++17, `runtime` already links
`OpenCV core imgproc imgcodecs highgui` (`SystemCode/runtime/CMakeLists.txt:6`),
so this adds **no dependency at all**.

### 3.1 Layout

Canvas 1696 × 1000, which is two native 848 × 480 colour frames wide
(`config/cameras.yaml` streams) with no rescale:

```
+---------------------------+---------------------------+
| A  cam_front  848x480     | A  cam_over   848x480     |
|    boxes, axes, timings   |    boxes, axes, timings   |
+---------------------------+-------------+-------------+
| B  world view             | C  graph    | D  PDDL     |
|    plan + elevation       |    nodes /  |    (:init   |
|    848 x 520              |    edges    |     ...)    |
|                           |    424x520  |    424x520  |
+---------------------------+-------------+-------------+
|  status strip: pair, skew, per-stage ms, fps, drops   |
+-------------------------------------------------------+
```

**A — camera panels.** Today's `Viewer::Show` body, unchanged.

**B — world view.** Orthographic plan (XY) of the table footprint from
`config/scene.yaml` `table_01` extent, with an elevation strip (XZ) beneath it.
Drawn per object:

- footprint rectangle from `ObjectState::extent`, which is world-axis-aligned by
  contract (`types.hpp`), so no rotation enters the box;
- a heading tick from `pose_w`'s rotation, so yaw is legible without pretending
  the box turned;
- a 1σ circle of radius `position_sigma_m` **only when `has_uncertainty(o)`**;
  the sentinel is negative precisely so absent uncertainty cannot be drawn as a
  point estimate, and the injected `table_01` carries sigmas of −1 by design
  (`config/scene.yaml` header);
- both camera positions and their frusta from `config/extrinsics.yaml`
  (real since 2026-09-06, recalibrated 2026-09-20, RMSE 0.26 / 0.32 px), so the
  viewpoint geometry that makes fusion work is on screen;
- `cls == "unknown_object"`: dashed box, no heading tick — its rotation is
  meaningless by contract (`INTERFACES.md` §5).

The elevation strip is not decoration. The only checkable accuracy figure the
project has produced so far is a 50 mm printed cube whose fused `z` read 29.7 mm
against a true 25 mm — **5 mm high** (`STATUS.md`; one observation, not a
measurement). An elevation view is where that is visible. It should stay visible.

**C — scene graph.** Node-link over `result.scene.objects`, edges parsed from
`result.pddl.facts` (`(on a b)`, `(in a b)`) — `pddl.facts` is already sorted
and already sanitised, so the panel needs no rules of its own and cannot drift
from the emitted contract.

**Layout must be deterministic.** Slots assigned by sorted object id, static
objects pinned to the bottom row, everything else on a row above by first
appearance. No force-directed layout, no per-frame re-solve: a graph that
re-arranges itself every frame is unreadable at 19 pairs/s, and a viewer would
read the motion as a change in the scene when nothing changed.

**D — PDDL.** The literal `result.pddl.init_block`, monospaced, plus
`pddl.warnings` in amber. Lines that changed since the previous pair are
highlighted for ~1 s. This panel is the answer to "what does the planner
actually get", and it should be the unedited string, never a prettified one.

### 3.2 It runs identically under `replay`

`the_eyes_run replay <session> --show=console` composes the same canvas from a
recorded session, and recorded sessions replay bit-exact (`AGENTS.md` §9
acceptance criterion). So the final video's demo segment can be re-shot as many
times as needed, deterministically, without the rig — and a live take remains
available for the "live run on hardware" requirement.

### 3.3 It records itself

`--show-record <path.mp4>` writes the composed canvas through `cv::VideoWriter`.
The demo footage then comes from the same code path that drew it, at a known
frame rate, rather than from a screen recorder capturing a compositor. Encoding
happens on the sink side and obeys §3.4 like everything else here.

### 3.4 Still a sink — hard requirement

The measured live budget is p50 51.7 ms per pair, p95 78 ms (`STATUS.md`).
The console must not spend any of it:

- every panel is drawn from `const FramePair&` / `const PipelineResult&` and
  mutates nothing;
- `Show()` returns `bool` for quit and cannot signal back-pressure;
- panel B/C/D drawing is skipped, not queued, if the previous frame's compose is
  still outstanding when composition moves off-thread (§6 phase 3);
- the video writer drops frames rather than blocking, and counts drops on the
  status strip so a dropped-frame video can never be mistaken for a slow system.

Acceptance criterion 2 in §8 makes this checkable rather than aspirational.

---

## 4. Exhibit 2 — the occlusion A/B, live

This is the money shot and it is nearly free, because `fusion::FuseFrame` is a
free function that takes an arbitrary subset of estimates
(`SystemCode/fusion/include/the_eyes/fusion/fusion.hpp:60`) — that is exactly
what the eight-arm ablation harness already uses.

Per pair, on estimates the pipeline has already computed:

```
arm_front = FuseFrame({cam_front estimates}, ...)   // no extra detect, no extra pose
arm_over  = FuseFrame({cam_over  estimates}, ...)
shipped   = result.scene                            // Fusion::Process, EKF, the real output
```

Cost is two extra geometric fuses per frame — no detector call, no
FoundationPose call, nothing on the GPU. Panel B draws `arm_front` and
`arm_over` as pale ghosts under the solid fused pose, and a three-cell counter
reads `front N | over M | fused K`. Occlude cam_front's view of an object with a
hand and the front cell drops while the fused cell does not. That is the entire
technical claim, happening live, in one gesture.

**One labelling rule, non-negotiable.** `FuseFrame` is stateless; the shipped
scene comes from `Fusion::Process`, which runs the EKF. Those arms differ by
*filter as well as by view set*, which is precisely the confound found in the
review of 2026-09-05 and fixed on 09-06 (`STATUS.md`, `fusion_agent` row). The
panel therefore labels the ghosts **"single view, unfiltered"** and the solid
pose **"fused + EKF"**, and the console never presents the counter as a
quantitative ablation result. The numbers for the report come from
`ablation_from_log` over `metrics.sqlite`, at a fixed filter setting, as they
already do. This panel is a *demonstration*, and it must say so on screen.

---

## 5. What is deliberately not built

- **No Qt / Dear ImGui / web GUI.** A control panel implies controls, controls
  imply the viewer can change the run, and the moment it can it is on the data
  path. The console is read-only by construction.
- **No browser dashboard over a live socket.** That is a second process, a
  serialisation format, and a reconnect story, for a window on the same machine.
  The twin already pays that cost and has a reason to (Isaac Sim is Python-only);
  the console does not.
- **No 3D world view in OpenCV.** Plan + elevation is honest, cheap, and
  readable in a compressed video. A hand-rolled perspective renderer would be a
  worse Isaac Sim, and Isaac Sim is exhibit 3.
- **No smoothing anywhere.** The detector flickers on real desk clutter (scores
  0.39–0.54, `STATUS.md`). The console shows the flicker.

---

## 6. Files and phases

```
SystemCode/runtime/
  include/the_eyes/runtime/console.hpp        Console: compose(), Show()
  include/the_eyes/runtime/console_panels.hpp pure draw fns, cv::Mat in/out
  src/console.cpp
  src/console_panels.cpp
  test/test_console_panels.cpp                headless, no window
config/runtime.yaml                           `console:` block
```

`console_panels.cpp` holds only free functions of the shape
`void DrawX(cv::Mat& panel, <const inputs>)`. They open no window and touch no
global state, so `test_console_panels` runs in `ctest` on a headless box — the
same reason `TWIN_PLAN.md` §5 keeps `twin_math.py` free of `pxr`.

**Phase 1 (~3 h) — panels B, C, D as pure functions + tests.** No window, no
runner change. Synthetic `SceneState`s: empty scene, one object, occluded
object, `unknown_object`, an object with sigmas of −1, a fact set with a cycle
in it. All of this is zero-hardware work.

**Phase 2 (~3 h) — `Console` composes, `--show=console` selects it.** `--show`
keeps today's exact behaviour, so nothing that currently works changes.

**Phase 3 (~2 h) — the A/B arms (§4) and the labelling.**

**Phase 4 (~2 h) — `--show-record`, and a replay pass over a recorded session
to confirm the composed video is deterministic.**

Total ~1.5 days including tests.

---

## 7. Exhibit 4 — offline result pages

`metrics.sqlite` already carries every `ObjectState` field per pair plus full
camera-frame `PoseEstimate`s (`metrics_log.hpp`). A small offline Python script
under `training/` — offline tooling, so `CLAUDE.md` constraint 1 permits it, and
`docs/results/*.png` is the existing precedent — turns one run into: per-stage
timing distributions, fused-vs-single object counts over time, sigma over time,
and the fact set as a timeline of changes. These are report and slide figures.
They are not the demo, and they must not be the demo — `README.md` §4 asks for a
live run, not a slideshow.

---

## 8. Acceptance criteria

Binary, per `AGENTS.md` rule 3.

1. **`test_console_panels` passes headless**, with no display, no camera, no
   GPU. A scene with an object whose `position_sigma_m` is −1 draws **no**
   uncertainty circle; `unknown_object` draws no heading tick; the graph panel's
   node positions are identical across two calls with the same `SceneState` and
   identical when the object vector is shuffled.
2. **The console cannot cost the pipeline.** Over ≥300 replayed pairs, `total_ms`
   p50 and p95 from `metrics.sqlite` with `--show=console` are within noise of
   the same session replayed with no viewer. If they are not, the console is not
   a sink and phase 3 has failed.
3. **Panel D is byte-identical to the emitted `(:init ...)`** — the rendered text
   matches `result.pddl.init_block` exactly, asserted in the test.
4. **The A/B panel labels its arms**, and the label naming the ghosts unfiltered
   is present in every frame in which a ghost is drawn.
5. **`--show-record` over a recorded session produces the same video twice** —
   frame count equal, and every frame identical **above the status strip**.

   *Corrected 2026-09-09, after building it.* The original wording asked for
   whole-frame checksum equality, which this canvas can never satisfy: the
   status strip shows measured per-stage timings and fps, and those legitimately
   differ between two runs of the same session. Measured on a 20-pair replay,
   rows 0–991 are identical 20/20 while rows 992–1023 differ — the strip itself
   plus the JPEG macroblock row above it, since MJPG's block grid does not align
   with the strip boundary. The criterion is what it should have said: the
   imagery, geometry, graph and facts are reproducible; the timings are not, and
   pretending otherwise would have meant weakening the test quietly instead of
   fixing the claim.

   Frames are MJPG in an AVI container for this reason too — every frame encodes
   independently, so the output is a deterministic function of the input. An
   inter-frame codec would make the property untestable.
6. `--show` alone renders exactly what it renders today.

---

## 9. Schedule fit

Today is 2026-09-08. Filament lands ~09-12 (`STATUS.md`), so track A is blocked
for that week and track B (rig, calibration — already done 09-06) has slack.
The console is ~1.5 days of zero-hardware work that fits inside that slack, needs
none of the printed objects, and can be driven meanwhile by the `fixed` detector
backend (`config/runtime.yaml` detector.backend) against any object on the desk.

Priority against the twin: **console first.** It is cheaper, it renders the
graded claim, and it improves the demo video whether or not the twin is ever
built. The twin stays cut #4.

Both are outranked the moment real ADD-S becomes reachable — that is MVP item 4,
and `TWIN_PLAN.md` §8 already says the same thing about itself.

Two inherited gaps the console cannot fix, and should not hide:

- `config/scene.yaml`'s table extent is still a `[VERIFY]` placeholder, so
  panel B's table outline is the wrong size until the desk is measured. Label
  the panel with the source of the extent so the placeholder is visible on
  screen rather than implied to be measured.
- No ground truth exists yet, so nothing in the console is an accuracy claim.
  The status strip should say `GT: none` while `metrics.sqlite`'s
  `ground_truth` table is empty.
