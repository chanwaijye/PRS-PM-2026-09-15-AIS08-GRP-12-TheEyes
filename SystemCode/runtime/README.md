# runtime — the live end-to-end runner

`runtime_agent` (AGENTS.md §9). One C++17 process that runs the whole chain on
the real rig, records and replays synchronised RGB-D pairs, and logs every pair
to SQLite:

```
cameras (by serial) ─┐
replay session ──────┴─> FramePair ─> RGB-D YOLOv8 (x2) ─> FoundationPose (x2)
                                      ─> fusion ★ ─> static scene ─> scenegraph ★ ─> PDDL
                                                               └─> metrics.sqlite
```

No ROS2. `INTERFACES.md` §1/§6 name ROS2 topics as the contract; the payloads
here are the same structs (`CameraFrame` → `DetectionSet` → `PoseEstimate` →
`SceneState` → PDDL string) passed in-process. A topic adapter would be a thin
layer on top of `Pipeline`; it is not built.

## Build and run

```bash
cmake -S SystemCode -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure

build/runtime/the_eyes_run check            # configs, model, pose server
build/runtime/the_eyes_run check --live     # ... plus the cameras, by serial
build/runtime/the_eyes_run live --record-only rig1 --frames 300   # capture only, 30 fps
build/runtime/the_eyes_run replay captures/runtime/<session>       # same pipeline, from disk
build/runtime/the_eyes_run live --record demo --show               # the demo
```

Common flags: `--config-dir DIR` (default `config`), `--repo-root DIR` (default
the config dir's parent; every relative path in the configs resolves against
it, never the working directory), `--detector onnx|mock:<fixture>|fixed:<spec>`,
`--pose foundationpose|mock`, `--db PATH`, `--no-log`, `--print-every N`.
Replay adds `--realtime`, `--start N`, `--max-pairs N`, and `--record NAME`.

Exit codes: 0 ok, 1 runtime error, 2 startup check failed, 130 interrupted
(after the recorder has drained and the run row is closed).

The FoundationPose server must be running for `--pose foundationpose`; `check`
prints the exact command when it is not:

```bash
python SystemCode/pose/server/foundationpose_server.py --socket /tmp/the_eyes_foundationpose.sock \
    --asset-root . --foundationpose-root ~/src/FoundationPose
```

Needs `libsqlite3-dev` (CMake `FindSQLite3`). Everything except the live capture
path builds and tests without librealsense, ONNX Runtime, a GPU, or cameras —
verified by configuring with `-DTHE_EYES_ONNXRUNTIME_ROOT=/nonexistent`.

### Fixed boxes: a real object the detector cannot see

```bash
build/runtime/the_eyes_run live --show --pose foundationpose \
    --detector 'fixed:cam_front=red_cube,300,200,120,120;cam_over=red_cube,400,180,100,100'
```

**Verify the box is actually on the object before believing any number.**
A stale box produces confident nonsense rather than an error: on 2026-09-06 a
first run used coordinates read from a capture taken before the cube was moved,
so one box held a mouse and the other bare table, and FoundationPose returned
stable poses with constant scores for 150 pairs. The tell was the two views
disagreeing by 181 mm; the fix was reading the coordinates off the run's own
recorded frames. Record the run, draw the box on a recorded frame, and look at
it. The two bad boxes are in `docs/results/fixed_box_stale_failure.png`.

`fixed:` hands the same hand-placed box per camera to the pose estimator on
every frame, through the normal IoU tracker so FoundationPose registers once.
It exists for an off-palette test print (the brown cube) and for isolating
FoundationPose from detector flicker: the pose, fusion, table and predicate
chain run on a real object while the trained detector still has nothing it
was trained on. Place the boxes by running `--show` and reading pixel
coordinates off the window; a box hanging outside the frame is clipped, one
entirely outside is dropped. The log labels the backend `fixed:<spec>` and
the startup report warns that nothing from the run is a detection result.

## What it does per pair

1. **Pair.** Live: one grabber thread per camera drains its librealsense
   pipeline at full rate into `FramePairer`, which keeps a 4-frame ring per
   camera and emits the nearest-stamp pair within `sync.max_skew_ms`. The
   lagging camera's newest frame is always the candidate, so a main loop slower
   than the cameras sees the freshest agreeing pair, never a backlog. Every
   frame it does not use is counted (`frames_dropped_since_last`, pairer
   stats), never silently discarded. The fusion stamp is the reference
   camera's (`views.names[0]`, the sync master). Startup: 15 warm-up frames per
   camera discarded, sync mode read back from each device, 30 pairs analysed
   with `camera::AnalyseSkew`, timestamp domains compared — any of these
   failing refuses to start (INTERFACES.md §1).
2. **Record** (optional) — `Recorder` enqueues the pair before processing so a
   recording is exactly the processed stream. Bounded queue, writer threads,
   PNG for both streams (lossless, bit-exact on replay). A full queue drops and
   counts; capture never blocks.
3. **Detect** per view (`OnnxDetector` or `MockDetector`). A frame without
   aligned depth is skipped with a warning, never zero-filled (§2).
4. **Pose** per view via one `PoseAgent` per camera sharing one estimator.
5. **Fuse** both views' estimates in one `Fusion::Process` call.
6. **Static scene** from `config/scene.yaml` (the table) is injected —
   without it no object can be `on` anything — then `Annotate` + `EmitInit`.
7. **Log** one transaction to `metrics.sqlite`; print a status line every N
   pairs and the `(:init ...)` block whenever the fact set changes.

## Recorded session format

```
captures/runtime/<YYYYMMDD_HHMMSS[_name]>/
  session.yaml     format_version 1; serials, K, depth scale, sync modes,
                   startup skew report, pairs written/dropped
  index.csv        pair_index, stamp_ns, skew_ms, <cam>_stamp_ns,
                   <cam>_frame_number, ..., frames_dropped_since_last
  cam_front/000000.rgb.png  000000.depth.png ...   (RGB as BGR PNG; depth 16-bit PNG)
  cam_over/  ...
```

Replay walks `index.csv` in file order with no clock unless `--realtime`, and
rebuilds every `CameraFrame` field. With the mock backends two replays of one
session produce identical fact sets (pinned by `test_replay_pipeline` and
checked again on the command line: 42 of 42 logged facts identical across two
runs). With the ONNX detector on CUDA that is not guaranteed — GPU kernels are
not bitwise deterministic; use `provider: cpu` for parity work.

## Metrics log

`runs/runtime/<session_id>/metrics.sqlite`, schema version 1. Tables: `runs`
(source, backends, active detector provider with CPU fallback flagged, config
dir, git revision, per-camera K/serial JSON, startup report), `frames` (one per
pair: stamps, skew, drops, per-stage ms, fusion diagnostics), `frame_views`
(per camera per pair: detect/pose ms, counts, pose_agent diagnostics),
`detections`, `estimates` (full camera-frame `PoseEstimate`: t + unit
quaternion, score, symmetric, extent, track id), `objects` (`ObjectState`
after annotation, including sigmas and velocity, `is_static` for injected
objects), `facts`, `warnings`, `ground_truth` (empty until the ArUco-tray tool
fills it).

`ablation_from_log <metrics.sqlite> [--run N]` rebuilds `fusion::AblationFrame`s
from `estimates` + `ground_truth` and runs the eight-arm occlusion harness on
them, scoring ADD-S against the CAD mesh vertices. With no ground truth rows it
says so and produces no numbers.

## Ground truth and visibility (2026-10-01)

`gt_from_trays` fills `ground_truth` from 1920x1080 tray stills (its header has
the full argument). Two additions for the capture suite:

- **Outlier screen.** Each class's per-marker poses are judged against their
  medoid before averaging; anything more than 10° or 40 mm from it is dropped
  (`--max-dev-deg`, `--max-dev-mm`). A marker rejected from *both* views is a
  card in the wrong recess and the class is refused; one rejected from a single
  view is a planar flip and is reported. A class losing more than a third is
  refused. On the 2026-09-20 stills nothing is rejected (worst honest sample
  17 mm / 2.8° from its mean); with one bowl card declared in the wrong recess
  the tool refuses with exit 4, where before it averaged 20 bad samples in.
- **`<metrics.sqlite>` may be `-` with `--dry-run`.** No run needed: the
  capture-day check that every tray decodes and agrees before anything is
  recorded. `--factory` still writes the stills' `intrinsics.yaml`.

`visibility_from_gt <session> <metrics.sqlite>` renders each ground-truth mesh
into every view of every pair and classifies its silhouette against measured
depth — nearer than the model by more than 40 mm is occluded, within it is
seen, farther is `behind` (ground truth off the object, not occlusion). That is
the proposal's definition of the occlusion tiers (7.1); the tool prints the
median visible fraction per view and object with its tier band, writes a
per-pair CSV and an overlay PNG per camera. On a T1 scene read the `behind`
column: it is how well ground truth sits on the object.

Both defaults were set on the first real scenes (2026-10-04), not chosen:

- **40 mm, not 20.** cam_front's depth inside the bowl reads 2–3 cm nearer than
  the surface, so at 20 mm an unoccluded bowl scored 28% visible; at 40 mm, 98%.
  An occluder therefore has to stand more than ~40 mm in front of what it hides.
- **A pixel with no depth counts as visible**, and a second column, `w/depth`,
  says how much of the silhouette is visible *and* measured. Active stereo
  leaves a band with no depth beside every foreground edge, on the surface
  behind it, so a half-hidden object's visible half is where its depth goes
  missing: on the half-hidden cube 47% of the silhouette had no depth and was in
  plain view. Excluding it scored the cube 2% visible; counting it gives 48%,
  with 1% usable by an RGB-D estimator. The gap between the two columns is the
  scene's real difficulty — on that scene cam_over detected the cube on 286 of
  300 pairs and none of its poses matched ground truth.

`scripts/capture_day.sh` sequences both on capture day and afterwards
(`begin | scene | end | list | process`); see its header. `scene --stage stills`
then `--stage record` run the two halves without prompting, which is how a scene
is driven when the person at the keyboard is not the person at the rig;
`--stills-from SCENE` reuses a scene's stills for a second occluder position on
a tray that has not been touched; `STILLS_CAMERAS` points the stills at a
cameras file with a lower colour exposure.

## Measured (2026-09-06)

From actual runs on this workstation (RTX 5070 Ti, cameras at 848x480 30 fps,
hardware-synced master/slave). Detector figures are **CPU** figures — see the
note below. The objects on the desk were not the printed benchmark objects
(filament ETA 2026-09-12), so detections are the synthetic-only model's
response to an ordinary desk and no accuracy number is claimed from these runs.

| run | measured |
|---|---|
| `live --record-only`, 150 pairs | 30.0 pairs/s, recorder 0 dropped, max queue fill 1 (PNG level 1, 2 writer threads keep up with 4 PNGs per pair at 30 fps) |
| startup sync check, 30 pairs, three runs | jitter **0.14 / 0.87 / 1.12 ms sigma**, 0 outliers, fixed offset 8.7 / −5.2 ms. `camera_probe`'s 4.51 ms sigma (2026-08-27) was measured with serial blocking reads on one thread, which its README already suspected measures queue jitter; one grabber thread per camera sees the shutters themselves |
| `live`, ONNX detector (CPU) + mock pose, 100 pairs | 30.3 pairs/s (limited by the cameras); detector 13.5 / 13.0 ms mean per view; pipeline total 26.6 ms mean, 31.7 max; 0 frames dropped |
| `replay` of that session, ONNX (CPU) + mock pose, 150 pairs | 22.5 pairs/s; detector 12.8 / 12.5 ms mean, p95 14.0 / 13.2 ms; total p50 25.1 ms |
| `replay`, ONNX (CPU) + **FoundationPose**, 100 pairs, 1–2 objects per view | steady state 13.9 pairs/s; total p50 53.7 ms, p95 85.8 ms, max 3859 ms (first registration); pose_agent p50 24.2 ms (cam_front, 2 tracks) / 12.4 ms (cam_over, 1 track) |
| `live --record`, ONNX (CPU) + **FoundationPose**, 120 pairs | 8.7 pairs/s over the run including a 2.1 s registration; steady state p50 51.7 ms per pair (≈19 pairs/s), p95 78.2 ms; pose_agent p50 25.3 ms (cam_front) with p95 36.2 ms; the registration stall surfaced as `frames_dropped_since_last = 56` on the next pair and as 554 ring-trimmed frames in the pairer stats — nothing silent |
| replay determinism, mock backends, 6-pair synthetic session, two runs | 42 of 42 logged facts identical |
| detector on **CUDA**, replay of the cube session, 300 inferences | p50 3.60 / 3.43 ms per view, p95 4.14 / 4.08 ms; pipeline total p50 7.13 ms with the mock pose |
| `live` on a printed 50 mm cube, fixed boxes + FoundationPose, 150 pairs | **both views fused into one object on every pair**; per-view world poses disagree by **11 mm**; fused pose sd 0.1 mm, EKF sigma 6.6 mm; `(on red_cube_01 table_01)` on all 150 pairs; 22.3 pairs/s, p50 23.6 ms per pair, pose p50 12.3 / 11.2 ms per view |

Track churn is visible in the logs (`red_cube_01` … `red_cube_05` over 100
pairs with the mock pose): the synthetic-only detector flickers on real desk
clutter at scores 0.39–0.54, each flicker is a new IoU track, and each new
track is a new fusion object. That is a detector-on-real-data finding for the
sim-to-real section, recorded in `detections` / `objects`, not a runner fault.

### The detector runs on the GPU as of 2026-09-06

Every C++ detector figure before that date was a **CPU** figure: the install at
`~/opt/onnxruntime-linux-x64-1.29.0` is the CPU-only distribution and has no
`libonnxruntime_providers_cuda.so`, so `provider: cuda` had never once loaded
from C++ and `allow_cpu_fallback: true` meant it never said so beyond a
warning. Measured on the same 150-pair session, 300 inferences each:

| provider | detector p50, per view | p95 | pipeline total p50 |
|---|---|---|---|
| `cuda` | **3.60 / 3.43 ms** | 4.14 / 4.08 ms | **7.13 ms** |
| `cpu` | 12.60 / 12.37 ms | 14.44 / 13.69 ms | 25.14 ms |

3.5x, and it moves the detector from half the frame budget to a tenth of it.
The two providers return the **same detections** on every frame compared:
same classes, same counts, scores within 0.0024 and boxes within 1 px, which
is float noise rather than a behaviour change.

Reproducing the setup on this box:

1. `onnxruntime-linux-x64-gpu_cuda13-1.29.0.tgz` from the ONNX Runtime v1.29.0
   release, extracted to `~/opt/`. The **cuda13** variant, not cuda12: the
   toolkit here is 13.3 and the provider links `libcudart.so.13`.
2. cuDNN 9 is not installed system-wide; the copy inside the venv
   (`nvidia/cudnn/lib/`) is symlinked into that distribution's `lib/`,
   including an unversioned `libcudnn.so` because that is the name the CUDA
   provider dlopens.
3. `cmake -S SystemCode -B build -DTHE_EYES_ONNXRUNTIME_ROOT=<that dir> -U ORT_INCLUDE_DIR -U ORT_LIBRARY`.
   **The `-U` matters**: `ORT_INCLUDE_DIR` and `ORT_LIBRARY` are `find_*` cache
   entries, so changing the root alone silently keeps the old CPU install.

`SystemCode/detector/CMakeLists.txt` links consumers with
`-Wl,--disable-new-dtags` and an rpath to the ORT `lib/`. The CUDA provider
dlopens cuDNN and carries no runpath of its own; DT_RPATH is inherited by
dlopen across the process where DT_RUNPATH (CMake's default) is not. Without
it the provider cannot find cuDNN, the session falls back to CPU, and the run
reports CPU latency while looking entirely healthy — so the path is baked into
the binary rather than left to `LD_LIBRARY_PATH` at the call site.

## Tests (`test_runtime`, 88, no hardware)

| suite | pins |
|---|---|
| `FramePairer` | nearest pairing with the −11 ms offset; a gap in one stream drops only unpairable frames and keeps stamps monotonic; late frames are stale; ring overflow counts; stall timeout; grabber failure propagates with the camera name; two jittered producer threads against a slow consumer never emit a pair outside max skew |
| `RecorderReplay` | 5 random pairs round-trip bit-exact (rgb, depth incl. 0 and 65535, K, scale, serial, stamps, frame numbers); start/limit; refuses to overwrite; rejects a future `format_version`; a full queue drops and counts without blocking and the survivors replay in order |
| `StaticScene` | the shipped `scene.yaml` injects `table_01` with all view bits and no uncertainty; a cube resting on it reads `(on red_cube_01 table_01)`; bad configs and id collisions throw |
| `MetricsLog` | every table written; `LoadAblationFrames` reproduces the logged estimates to 1e-12; reopen keeps the schema; a foreign schema version is refused |
| `RuntimeConfig`, `Backends`, `StartupChecks` | strict loaders; factories; missing model names the export command; a dead pose socket names the server command; the shipped configs cross-reference; a swapped serial / view order / world frame is caught |
| `ReplayPipeline` | record a synthetic session → fixture detector → identity pose → real extrinsics/fusion/predicates/scene → `Pipeline` → `MetricsLog`, twice: identical facts, right row counts, ablation frames rebuild |
| `MarkerTrays` | tray config loading and its refusals; Markley averaging and reported scatter; the outlier screen keeps a cluster shaped like the real stills whole, drops a ~120° flip listed first and a card in the wrong recess; ground truth round-trips through the log with pair, instance and source |
| `Visibility` | a face-on cube projects to exactly 45 × 45 px at the right depth; a tilted cube's depth matches an independent ray cast to 1e-6 m; occluder, tolerance, missing depth (unknown, not hidden), `behind`, erosion; OBJ face spellings, relative indices and refusals; the shipped meshes load and render |

## Design choices, stated once

- **Pairs only, no single-view mode.** A camera that stops delivering is a rig
  fault; the run stops after `live.max_consecutive_stalls` rather than quietly
  degrading the headline claim to one view. `--allow-single-view` would be a
  small addition to the pairer if a demo ever needs it.
- **Static objects carry no uncertainty (sigmas −1).** No estimator produced
  them; INTERFACES.md §5 reserves 0 for measured perfect certainty.
- **Detector backend key lives in `config/runtime.yaml`**, not `detector.yaml`
  (which describes one network and belongs to `detector_agent`). The pose
  backend key already lived in `pose.yaml` under `estimator.*`; this module is
  the first to read it. `request_timeout_ms` was raised from 5 s to 15 s there
  because the measured worst-case registration was 4.3 s and the server holds
  one CUDA lock across both cameras.
- **`ObjectState::extent` is still the unrotated mesh AABB** (review finding 4)
  and **FoundationPose scores are still near-constant** (finding 2). Neither is
  this module's to fix; both show up in the logged `objects` and `estimates`
  rows, which is where the evidence for fixing them will come from.
