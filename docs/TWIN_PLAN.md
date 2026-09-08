# Digital-twin reconstruction — implementation plan

`twin_agent` (AGENTS.md §8), proposal §5.6. **Stretch deliverable, cut order #4.**
Written 2026-09-07; nothing here is built yet.

This is a plan, not a status file. Every number quoted below is sourced from
`docs/STATUS.md` or a config file and is marked with where it came from; the
twin itself has produced no measurements.

---

## 1. What the twin is for

Spawn the CAD meshes in Isaac Sim at the fused world poses, so the observed
tabletop is reconstructed as a live virtual scene.

Its demonstration value is obvious and is not the reason to build it. The reason
is the second one proposal §5.6 gives: **a reconstruction that stands up is
evidence that `SceneState` is a sufficient world-state encoding**, which is the
property PM4 depends on. If `{class, mesh, pose_w, extent}` per object is enough
to rebuild the scene, then it is enough to be a world model's state. That claim
is checkable — §6 turns it into a pass/fail geometric test rather than a
screenshot.

It is cut fourth for a reason. Nothing in the MVP depends on it, and no work
below touches `fusion`, `scenegraph`, or the pipeline's data flow.

---

## 2. What already exists in its favour

| Asset | State | Why it matters here |
|---|---|---|
| `assets/meshes/usd/{bowl,mustard_bottle,red_cube}.usd` | Converted, object frame **verified after conversion** (`training/obj_to_usd.py`) | The USD origin is the mesh AABB centre, which is exactly the frame `pose_w` is expressed in (`config/meshes.yaml`). Spawning is an assignment, not a re-origin problem. |
| Isaac Sim 5.1.0 at `~/.local/share/ov/pkg/isaac-sim-5.1.0` | Works headless, 6.9 s startup (STATUS.md) | No install work. |
| `training/detector/replicator_dataset.py` | Runs end to end | Already solved the four silent-failure traps: 1 m default near clip, dome light required for the beauty pass, `rep.modify.pose(scale=)` being absolute metres, and cameras created at a distribution never being placed. `apply_intrinsics()` converts measured fx to USD focal/aperture and reads it back. |
| `config/extrinsics.yaml` | **Real** `T_world_cam`, both units, RMSE 0.39 / 0.33 px (2026-09-06) | The twin's virtual cameras go at the measured rig pose, not at `replicator_dataset.py`'s design-geometry azimuths (±40°, 0.70 m standoff), which describe where the cameras were meant to be. |
| `PipelineResult::scene` | Populated every pair, post static injection | The twin's input already exists as a value. |
| `metrics.sqlite` `objects` table | Every `ObjectState` field, per pair, per run | An offline twin needs no live rig. |

---

## 3. Architecture

### 3.1 Isaac Sim is a renderer, not runtime logic

CLAUDE.md constraint 1 puts all runtime code in C++17. Isaac Sim is Python-only.
This is the same collision FoundationPose caused, and it gets the same answer,
for the same reasons set out in `SystemCode/pose/include/the_eyes/pose/foundationpose_client.hpp`:
**run it out of process, behind a byte protocol.** Every line that decides
anything stays in C++ (`SystemCode/twin/`); the Python side spawns prims and
writes transforms and does nothing else.

Direction mirrors the pose bridge — **the Python viewer listens, the C++ runner
connects** — so `TwinClient` reuses the reconnect structure already proven in
`FoundationPoseClient` (`EnsureConnected` / `Disconnect` / `SendAll`,
`reconnect_count`, deferred `warnings`).

```
frame : "TWN1" | u32 header_len | header
```

Little-endian `u32`, YAML header, no binary payload and no response frame.
Pixel data is what forced raw bytes beside the header in the pose protocol; a
`SceneState` is ~10 objects of ~20 doubles, so YAML costs nothing measurable and
both sides already have a parser (yaml-cpp / PyYAML). One frame carries one
complete `SceneState` — there is no delta encoding, because a viewer that missed
a frame must not be left applying a diff to a state it does not have.

### 3.2 The twin is strictly off the data path

Same status as `Viewer` (`--show`): a sink, never a stage.

- The socket is **non-blocking**. A `Publish()` that would block **drops the
  frame, counts it, and returns false.**
- **Latest state wins.** There is no queue to drain, so a slow renderer can
  never back pressure into the pipeline.
- An absent or dead viewer is a **warning, not a startup failure.** Contrast
  with the FoundationPose server, which `startup_checks` refuses to run without:
  that one is on the data path and the twin is not.

This is a hard requirement, not a nicety. The measured live budget is p50
51.7 ms per pair (STATUS.md), and a demonstration viewport must not be able to
spend any of it.

### 3.3 No physics

Prims are kinematic transforms. Isaac Sim is used as a stage and a renderer;
PhysX is not enabled.

AGENTS.md §8 offers "kinematic **or** snapped-to-contact"; this takes the first.
A static tabletop gains nothing from simulation, and settling motion under
gravity would appear in the viewport as pose jitter that is not in the poses —
the twin would be showing the physics engine's opinion, not the perception
result. Physics stays available for PM4, where the world model may want it.

---

## 4. Snap-to-contact defaults OFF

AGENTS.md §8 anticipates snapping objects to contact "to avoid interpenetration
from residual pose error". Implemented, but **off by default**, and this is the
one design choice here worth arguing explicitly.

The 2026-09-06 live run produced exactly one checkable accuracy figure in the
whole project so far: a 50 mm printed cube on the table has its centre at 25 mm,
and the fused `z` read **29.7 mm — 5 mm high** (STATUS.md; one observation, not a
measurement). A twin that snaps that cube down onto the table surface renders a
scene *more accurate than the perception that produced it*, and the single piece
of real error evidence disappears into a picture that looks correct.

So:

- **Default:** kinematic, unsnapped, true `pose_w`. Interpenetration is visible
  because it is real.
- **`--snap`:** opt-in. Every applied Δ is logged, and a correction exceeding
  `twin.max_snap_m` aborts rather than silently applying — a large snap means
  the pose is wrong in a way the twin must not paper over.
- When snapping is active the viewport carries an on-screen banner saying so, so
  no screenshot of a snapped twin can be mistaken for an unsnapped one.

CLAUDE.md rule 5 is about fabricated numbers; this is the same rule applied to a
fabricated picture.

---

## 5. Files

```
SystemCode/twin/
  CMakeLists.txt
  include/the_eyes/twin/twin_message.hpp    SceneState <-> TWN1 frame
  include/the_eyes/twin/twin_client.hpp     non-blocking UDS, reconnecting
  src/twin_message.cpp
  src/twin_client.cpp
  test/test_twin_message.cpp                round trip, sentinels, refusals
  test/test_twin_client.cpp                 no viewer / dead viewer / full buffer
  tools/twin_replay.cpp                     metrics.sqlite objects -> TWN1
  viewer/twin_viewer.py                     Isaac Sim side, runs under python.sh
  viewer/twin_math.py                       pure numpy: pose -> USD xform
  viewer/test_twin_math.py                  runs in the project venv, no Isaac
  README.md
config/twin.yaml
```

Build wiring: `add_subdirectory(twin)` goes **before** `runtime` in
`SystemCode/CMakeLists.txt` — `twin` depends only on `common` and yaml-cpp, and
`runtime` links it for the live hook. `twin_replay` links `SQLite::SQLite3`
directly rather than pulling in `the_eyes_runtime`, which keeps the module
boundary AGENTS.md rule 1 asks for.

`twin_math.py` exists so the pose→transform conversion is unit-testable in
`~/venvs/the-eyes-r1` without launching Kit. Anything requiring `pxr` is a
6.9-second startup per test run and cannot go in `ctest`; keeping the arithmetic
in a numpy-only module means the part that can be wrong silently is the part
that gets tested cheaply.

---

## 6. Phases

### Phase 1 — C++ publisher and offline replay (~half a day)

`twin_message`, `twin_client`, `twin_replay`. Zero hardware, zero GPU, zero
Isaac Sim, and this is the module's mock mode in the AGENTS.md rule-2 sense: a
recorded session's `objects` table replayed at its recorded rate is a complete
`SceneState` stream with nothing attached.

### Phase 2 — the Isaac viewer (~a day; Kit iteration is slow)

- Stage: `config/scene.yaml`'s table as a box, dome light, and the floor/back
  wall geometry from `replicator_dataset.py` (which exists there because a
  tabletop scene with no room renders 69% background).
- Two virtual cameras at the **real** `T_world_cam` from `config/extrinsics.yaml`,
  with the measured per-unit intrinsics through `apply_intrinsics()`, and the
  near clip at 0.02 m — the USD default of 1 m sits behind the entire scene.
- One prim per class from `assets/meshes/usd/`, **pooled and hidden** rather than
  created and destroyed per frame; a 30 Hz create/destroy cycle on a USD stage is
  the obvious way to make this unusable.
- `cls == "unknown_object"` renders as a translucent wireframe box of `extent`
  and never as a class mesh. Its rotation is meaningless by contract
  (INTERFACES.md §5), so the box is world-axis-aligned.

### Phase 3 — live wiring (~2 hours)

`--twin[=socket]` on `the_eyes_run`, a `twin:` block in `config/runtime.yaml`,
and an optional **non-fatal** probe in `startup_checks` that reports whether a
viewer is listening. Non-fatal is the point: every other check in that list
refuses to run, and this one must not.

### Phase 4 — the evidential render (~2 hours)

Render the twin from `cam_front`'s real extrinsics beside the actual RGB frame
of the same pair. That side-by-side is both the demo and the §1 argument: it is
what a reader looks at to judge whether `SceneState` carries enough to rebuild
the scene.

---

## 7. Acceptance criteria

Binary, per AGENTS.md rule 3.

1. **`test_twin` passes with no Isaac Sim, no camera, no GPU.** A `SceneState`
   round-trips through TWN1 unchanged; `position_sigma_m = -1` survives as `-1`
   and not as `0` (INTERFACES.md §5 — the sentinel is the whole point of the
   field); an object with `visible_views == 0` is refused rather than rendered.
2. **The publisher never blocks.** With a deliberately filled socket buffer,
   `Publish()` returns false in under 100 µs and increments a drop counter, and
   `total_ms` in the metrics log is statistically unchanged with a twin attached
   versus without.
3. **Geometric fidelity — headless `twin_viewer.py --verify`.** For each class,
   the spawned prim's world-space AABB centre equals `pose_w.translation()`
   within **1 mm**, and its half-extents equal `config/meshes.yaml` `extent`
   within **1 mm**, read back through `UsdGeom.BBoxCache`. This is the criterion
   that replaces "matches the physical scene visually" with something that can
   fail. It also catches the exact class of bug `obj_to_usd.py` was written to
   avoid — a converter or a spawn path that quietly re-origins or rescales.
4. `unknown_object` renders as a placeholder volume of the cluster extent.
5. `twin_replay` over a recorded session reproduces the `objects` table's object
   set, pair for pair.
6. `--snap` logs every applied Δ and aborts on a correction exceeding
   `twin.max_snap_m`.

---

## 8. Scope, schedule, and what this inherits

Nothing above modifies `fusion`, `scenegraph`, or the pipeline's data flow. The
only edit outside `SystemCode/twin/` is one optional sink in the runner,
alongside `Viewer`.

Total ~2 days. As of 2026-09-07 that fits the week before filament lands
(~09-12), which is otherwise track-B slack (STATUS.md, "The critical path").
**The moment real ADD-S becomes reachable, it outranks this** — real ADD-S is
MVP item 4 and the twin is cut #4.

Two inherited gaps, neither of which the twin can fix:

- `config/scene.yaml`'s table dimensions are still `[VERIFY]` placeholders. The
  objects will be in the right place; the slab under them will be the wrong
  size until the desk is measured with a tape.
- The detector flickers on real desk clutter (scores 0.39–0.54, STATUS.md), so a
  live twin will show objects appearing and disappearing. That is a faithful
  rendering of the current perception, and the twin should not smooth it —
  hiding it here would hide it from the one view where it is most legible.
