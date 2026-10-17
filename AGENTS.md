# AGENTS.md — The Eyes

Coding subagent definitions for Claude Code orchestration. Same pattern as PM1
(*The Planner*): one agent owns one module, each with an explicit I/O contract, a
mock mode, and acceptance criteria that can be checked without the rest of the
system existing.

Read `CLAUDE.md` for project constraints and `INTERFACES.md` for data contracts
before working in any module.

---

## Orchestration rules

1. **One agent per module.** An agent does not edit files owned by another agent.
   Cross-module changes go through the interface contract: propose the contract
   change, get it agreed, then both sides implement.
2. **Mock mode first.** Every agent's module must run standalone against
   synthetic or recorded input before it is wired to its upstream neighbour.
3. **Acceptance criteria are binary.** "Works" is not a criterion. Each module
   has a test that passes or fails.
4. **No hardware on the critical path where avoidable.** `fusion_agent` and
   `scenegraph_agent` are fully testable with zero hardware — build these first,
   in parallel with camera bring-up.

---

## Build order

```
Phase A (no hardware needed)
  scenegraph_agent  ──┐
  fusion_agent      ──┤  synthetic inputs, unit-testable today
                      │
Phase B (hardware)    │
  camera_agent      ──┤  bring-up, sync, noise characterisation
  calib_agent       ──┘  extrinsics → unblocks fusion on real data

Phase C (models)
  detector_agent       synthetic dataset → YOLOv8 fine-tune
  pose_agent           FoundationPose wrapper (gated by R1)

Phase D (integration)
  runtime_agent        one process: cameras → ... → PDDL, record/replay, SQLite
```

R1 (FoundationPose on Blackwell sm_120) is verified **before** Phase C starts. If
it fails, `pose_agent` switches to the ONNX Runtime CUDA EP fallback and the
schedule absorbs it early rather than in October.

---

## Agents

### 1. `camera_agent` — RealSense driver & sync

**Owns:** `SystemCode/camera/`

**Responsibility:** librealsense C++ wrapper, dual-camera hardware sync, ROS2
publication of aligned RGB-D per camera, rosbag record/replay tooling.

**Contract:** produces the topics in `INTERFACES.md` §1.

**Mock mode:** replay from a recorded rosbag. Every downstream module must be
runnable against a bag with no camera attached.

**Acceptance criteria:**
- Both cameras stream aligned depth + color at ≥ 15 fps, 848×480, simultaneously.
- Cameras are selected by serial number; verified by swapping USB ports and
  confirming the namespace assignment is unchanged.
- Frame timestamps across the two cameras agree within one frame period
  (asserted at startup, logged).
- Depth-noise characterisation report: flat wall at 0.4/0.6/0.8/1.0 m, plane-fit
  RMS error per distance. **This report feeds the evaluation error budget** — it
  is a deliverable, not a debugging step.

**Notes:** build librealsense with `-DFORCE_RSUSB_BACKEND=true`; do not use the
DKMS package (see `CLAUDE.md`).

---

### 2. `calib_agent` — intrinsics, extrinsics, world registration

**Owns:** `SystemCode/calibration/`

**Responsibility:** ChArUco-based intrinsic calibration per camera, stereo
extrinsics between the two, registration to a table-origin world frame, and a
calibration quality report.

**Contract:** emits `config/extrinsics.yaml` per `INTERFACES.md` §4.

**Mock mode:** synthetic ChArUco observations from known ground-truth poses —
recovering the known extrinsics validates the solver independently of image
quality.

**Acceptance criteria:**
- Reprojection RMSE below threshold (target < 0.5 px) on both cameras.
- Round-trip test: a physical point observed by both cameras maps to the same
  world coordinate within tolerance.
- Calibration is run **after** final rigid mounting; re-running after any
  mechanical change is documented in the user guide.

---

### 3. `detector_agent` — YOLOv8 detection

**Owns:** `SystemCode/detector/`, `training/detector/`

**Responsibility:** Omniverse Replicator dataset generation with domain
randomisation (RGB **and** depth rendered per frame), YOLOv8 fine-tuning from
COCO weights with a **4-channel RGB-D early-fusion input stem**, ONNX export,
C++ inference via ONNX Runtime / TensorRT.

**Contract:** consumes §1 topics (color + aligned depth), produces
`DetectionSet` per §2.

**Mock mode:** fixed bbox list from a JSON fixture — lets `pose_agent` develop
without a trained model.

**Multimodal implementation notes:**
- Widen the stem conv from 3 to 4 input channels; initialise the depth channel by
  inflating (mean of) the pretrained RGB stem weights rather than random init —
  random init on one channel destabilises early training.
- Normalise depth in metric metres, clipped to the working range (~0.2–1.5 m for
  tabletop). Do not use raw uint16.
- **Encode invalid depth explicitly.** RealSense returns 0 for stereo holes; left
  as-is the network reads this as "extremely close." Use a sentinel or a validity
  mask channel — decide once and document it.
- Replicator must render depth with the same noise character as the real sensor
  where possible; otherwise the depth channel becomes the dominant sim-to-real
  gap. The depth-noise report from `camera_agent` informs this.

**Acceptance criteria:**
- mAP@0.5 ≥ 0.90 on the synthetic validation split.
- Real-capture mAP measured and reported (no target — this is the sim-to-real
  finding).
- **Modality ablation:** RGB-only vs. RGB-D detector mAP, with per-class
  breakdown. Textureless/low-contrast classes are the ones expected to move; if
  they do not, that is a reportable negative result, not a failure to hide.
- **Dataset-size ablation:** mAP at ~5k / 20k / 50k training frames, plotted.
- C++ inference output matches the Python reference within float tolerance.
- Detections whose class has no registered mesh are dropped with a warning.

---

### 4. `pose_agent` — FoundationPose 6-DoF estimation

**Owns:** `SystemCode/pose/`

**Responsibility:** FoundationPose wrapper — mesh registry loading, ROI cropping
from detections, per-view pose estimation, TensorRT engine management.

**Contract:** consumes `DetectionSet` (§2), produces `PoseEstimate` (§3).

**Mock mode:** identity poses, or ground-truth poses replayed from an Isaac Sim
capture — lets `fusion_agent` develop against realistic input.

**Acceptance criteria:**
- ADD-S < 10% of object diameter on unoccluded objects, single view.
- **R1 gate:** TensorRT engines build and run on sm_120, or the ONNX Runtime
  fallback is working and its latency penalty is measured. Resolve this first.
- Latency per object measured and logged; feeds the R4 scheduling decision.
- Poses emitted in the camera optical frame only — no world-frame knowledge.

---

### 5. `fusion_agent` ★ — multi-view pose fusion

**Owns:** `SystemCode/fusion/`

★ Primary technical contribution. **Written from first principles.** No
off-the-shelf fusion or pose-averaging library.

**Responsibility:** transform per-view poses to world frame; associate estimates
across views into object instances; confidence-weighted pose fusion (quaternion
averaging via the Markley eigendecomposition method); disagreement gating to
reject outlier views; per-object EKF for temporal smoothing; unknown-object
merge from the clustering path.

**Contract:** consumes `PoseEstimate` (§3) + `extrinsics.yaml` (§4), produces
`SceneState` (§5).

**Mock mode:** synthetic dual-view pose streams with injected noise, bias, and
dropout. **This is the primary development mode** — no hardware required.

**Acceptance criteria:**
- Synthetic recovery test: with known ground truth and Gaussian per-view noise,
  fused error is measurably below single-view error.
- Symmetric objects do not have their orientation naively averaged.
- Disagreement gate rejects an injected outlier view without corrupting the
  fused estimate.
- Single-view objects pass through with the correct `visible_views` bit.
- Refuses to start on extrinsics failing the reprojection quality gate.
- **Occlusion ablation harness:** runs the same input three ways (cam_front only,
  cam_over only, fused) and emits comparable ADD-S. This harness produces the
  project's headline result — treat it as a deliverable.

---

### 6. `scenegraph_agent` ★ — relational predicate extraction

**Owns:** `SystemCode/scenegraph/`

★ Primary technical contribution. Written from first principles.

**Responsibility:** geometric rules over fused poses and bounding volumes to
derive `on` / `in` / `clear` / `occluded_from`; emit scene graph and the PDDL
`(:init ...)` block; SQLite metric logging.

**Contract:** consumes `SceneState` (§5), produces the outputs in §6.

**Mock mode:** hand-written `SceneState` fixtures with known expected predicate
sets — a pure unit-test target, zero hardware, zero models.

**Acceptance criteria:**
- Predicate accuracy ≥ 95% against hand-labelled ground truth on the benchmark
  scene suite.
- Deterministic: identical `SceneState` in → identical fact set out.
- Tolerances read from `config/predicates.yaml`; no hard-coded constants.
- Emitted PDDL parses in PM1's planner without modification.
- Edge cases covered by fixtures: stacked objects, object in container, object
  occluded from exactly one view, unknown object present.

---

### 7. `cluster_agent` — unknown-object segmentation *(above MVP — CUT 2026-10-17, never built)*

**Owns:** `SystemCode/clustering/`

**Responsibility:** subtract points explained by detected objects from the depth
point cloud; Euclidean/DBSCAN clustering of the residual; emit
`cls = "unknown_object"` entries with centroid and bounding volume.

**Contract:** feeds `fusion_agent`, output conforms to the `unknown_object`
semantics in `INTERFACES.md` §5.

**Mock mode:** synthetic point clouds with a known extra object.

**Acceptance criteria:**
- An object with no CAD mesh placed in the scene is reported as
  `unknown_object` with centroid error within tolerance.
- No false unknown-object reports on scenes containing only known objects.

**Cut order:** second. Losing this removes the *unsupervised* half of PRS
category ①; the supervised half still satisfies the requirement.

---

### 8. `twin_agent` — digital-twin reconstruction *(stretch)*

**Owns:** `SystemCode/twin/`

**Responsibility:** spawn CAD meshes in Isaac Sim at fused world poses to
reconstruct the observed scene live.

**Acceptance criteria:**
- Reconstructed scene matches the physical scene visually; objects spawned
  kinematic or snapped-to-contact to avoid interpenetration from residual pose
  error.
- Unknown objects appear as placeholder bounding volumes.

**Cut order:** fourth (last). Demonstration value only — but it also evidences
that the scene graph is a sufficient world-state encoding, which is the property
PM4 depends on.

---

### 9. `runtime_agent` — live runner, record/replay, metrics log

**Owns:** `SystemCode/runtime/`, `config/runtime.yaml`, `config/scene.yaml`

**Responsibility:** the one process that runs the chain on the real rig:
open both cameras by serial, pair frames by nearest timestamp, drive the
detector and one `pose_agent` per view, fuse, inject the static scene (the
table), extract predicates, emit PDDL; record synchronised RGB-D pairs to disk
and replay them through the identical pipeline; log every pair to SQLite. Plain
C++17, no ROS2 — `INTERFACES.md` §1/§6 remain the documented contract and the
same payloads travel in-process.

**Contract:** consumes every interface §1–§7; produces the `(:init ...)` block
on stdout and `runs/runtime/<session>/metrics.sqlite`, whose `estimates` table
carries per-view `PoseEstimate`s so the occlusion ablation can be re-scored
from a log once ground truth exists (`ablation_from_log`).

**Mock mode:** `the_eyes_run replay <session> --detector mock:<fixture> --pose mock`
— a recorded session (or one synthesised by `test_replay_pipeline`) through the
real fusion and scene-graph modules with no camera, GPU, model, or server. This
is also the camera module's replay mode.

**Acceptance criteria:**
- `test_runtime` passes with no hardware, in builds with and without
  librealsense / ONNX Runtime.
- Startup refuses to run on: a serial that differs between `cameras.yaml` and
  `extrinsics.yaml`; view order that differs between `fusion.yaml`,
  `predicates.yaml` and the cameras; extrinsics past the reprojection gate; a
  missing model; an unreachable FoundationPose server (naming the command to
  start it); a device whose sync mode did not take; startup skew outside one
  frame period.
- A recorded session replays bit-exact, and two replays with mock backends
  produce identical fact sets.
- Every processed pair has a `frames` row; frames the pairer did not use are
  counted on the next pair, never dropped silently.
- Live: startup jitter logged in the range `camera_probe` measured; end-to-end
  latency per stage reported from `LatencyStats`, not estimated.
