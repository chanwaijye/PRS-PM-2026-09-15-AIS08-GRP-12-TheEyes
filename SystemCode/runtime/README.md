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

Track churn is visible in the logs (`red_cube_01` … `red_cube_05` over 100
pairs with the mock pose): the synthetic-only detector flickers on real desk
clutter at scores 0.39–0.54, each flicker is a new IoU track, and each new
track is a new fusion object. That is a detector-on-real-data finding for the
sim-to-real section, recorded in `detections` / `objects`, not a runner fault.

**The ONNX Runtime at `~/opt/onnxruntime-linux-x64-1.29.0` is the CPU-only
distribution** (its `lib/` has no `libonnxruntime_providers_cuda.so`), so the
detector's `provider: cuda` has never been loadable from C++ on this box and
every C++ detector figure so far — including `detector_parity` — is a CPU
figure. The runner records the active provider in `runs.detector_provider`
and warns at startup. Installing the `onnxruntime-linux-x64-gpu` package and
pointing `THE_EYES_ONNXRUNTIME_ROOT` at it is the fix; nothing in the runner
changes.

## Tests (`test_runtime`, 30, no hardware)

| suite | pins |
|---|---|
| `FramePairer` | nearest pairing with the −11 ms offset; a gap in one stream drops only unpairable frames and keeps stamps monotonic; late frames are stale; ring overflow counts; stall timeout; grabber failure propagates with the camera name; two jittered producer threads against a slow consumer never emit a pair outside max skew |
| `RecorderReplay` | 5 random pairs round-trip bit-exact (rgb, depth incl. 0 and 65535, K, scale, serial, stamps, frame numbers); start/limit; refuses to overwrite; rejects a future `format_version`; a full queue drops and counts without blocking and the survivors replay in order |
| `StaticScene` | the shipped `scene.yaml` injects `table_01` with all view bits and no uncertainty; a cube resting on it reads `(on red_cube_01 table_01)`; bad configs and id collisions throw |
| `MetricsLog` | every table written; `LoadAblationFrames` reproduces the logged estimates to 1e-12; reopen keeps the schema; a foreign schema version is refused |
| `RuntimeConfig`, `Backends`, `StartupChecks` | strict loaders; factories; missing model names the export command; a dead pose socket names the server command; the shipped configs cross-reference; a swapped serial / view order / world frame is caught |
| `ReplayPipeline` | record a synthetic session → fixture detector → identity pose → real extrinsics/fusion/predicates/scene → `Pipeline` → `MetricsLog`, twice: identical facts, right row counts, ablation frames rebuild |

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
