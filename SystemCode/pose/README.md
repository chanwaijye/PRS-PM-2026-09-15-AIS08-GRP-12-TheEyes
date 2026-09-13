# pose — per-view 6-DoF pose estimation

Consumes `DetectionSet` (`INTERFACES.md` §2), produces `PoseEstimate` (§3).
Owns the mesh-registry join, ROI derivation, the estimator backend, and latency
accounting.

## Why FoundationPose runs out-of-process

`CLAUDE.md` constraint 1 puts all runtime code in C++17. FoundationPose has no
C++ implementation — it is a Python library (`docs/R1_STATUS.md`). The
constraint cannot be met literally, so the split is:

- **Every line of our own runtime logic is C++.** Registry, ROI, orchestration,
  gating, latency, mock backend, protocol client.
- **FoundationPose is an external inference service** behind a byte protocol,
  treated exactly as a TensorRT engine would be.

Embedding an interpreter instead would put the GIL, NumPy's ABI, and CUDA
context ownership inside our runtime process. Out-of-process also means a
FoundationPose crash disconnects a socket rather than killing perception.

## Build and test

```bash
cmake -S SystemCode -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

Dependencies: Eigen3, OpenCV, yaml-cpp, GTest. **The unit tests need no
hardware, no GPU, and no model** — the FoundationPose protocol is tested against
an in-test fake server.

## Mock mode

`MockPoseEstimator` has two factories:

- `Identity(distance_m)` — identity rotation at a fixed distance. Lets
  downstream modules run before any pose source exists.
- `Replay(table, noise)` — replays recorded ground truth with optional Gaussian
  translation/rotation noise and dropout, deterministic from a seed. **This is
  what `fusion_agent` develops against**: it is how a synthetic dual-view stream
  with missing views is produced.

An unmatched replay request returns no pose rather than a fabricated one.

## Running against real FoundationPose

```bash
# 1. Start the server (needs the R1 venv and a FoundationPose checkout)
~/venvs/the-eyes-r1/bin/python SystemCode/pose/server/foundationpose_server.py \
    --asset-root ~/src/FoundationPose --foundationpose-root ~/src/FoundationPose

# 2. Drive it end-to-end from C++
build/pose/foundationpose_probe ~/src/FoundationPose/demo_data/mustard0 30
```

Verified 2026-08-20 on the mustard0 sequence: **30/30 frames produced poses**,
end-to-end latency p50 **11.1 ms**, p95 44.3 ms, max 4332 ms (the registration
frame). Tracking through the socket is *faster* than the 17.2 ms measured
in-process on full frames, because only the ROI crop crosses the wire.

## Protocol

Length-prefixed frames on a Unix domain socket. Headers are YAML — both sides
already have a parser, so the bridge adds no dependency to either. Pixels travel
as raw bytes beside the header, not base64 inside it.

```
request : "FPR1" | u32 len | header | u32 len | rgb | u32 len | depth
response: "FPS1" | u32 len | header
```

The client tracks which objects the *server* believes it has registered, and
clears that set on every disconnect — the server holds the per-object estimator
state, so a reconnected server has forgotten it and would refine against
nothing. First sighting sends `mode: register` (~1.2 s at the shipped 6 refine
iterations; ~0.7 s with `--register-iters 3`), thereafter `mode: track`
(~11 ms).

## Findings worth knowing

**FoundationPose's hypothesis score is not a usable confidence.** §3 specifies a
score in [0, 1]; the scorer emits ~81. Worse, on mustard0 all 252 hypotheses
score between **80.84 and 81.02**, with the top twelve *exactly tied* at
81.0156. A softmax over that returns ~1/N precisely because the best hypotheses
tie — the opposite of what a confidence should say.

The server therefore reports a **relative margin**: how far the winner stands
out from the field in standard deviations, squashed to (0, 1). All-tied inputs
give exactly 0.5 — "no information, weight neutrally". On the demo sequence it
produces 0.82, *identical on every frame*.

Read that plainly: **fusion's weighting between two FoundationPose views is
effectively uniform, and its disagreement gate — not the score — is what
protects the fused estimate.** If confidence-weighted fusion is to mean
anything, this needs revisiting: either re-run the scorer per tracked frame, or
derive confidence from depth-inlier ratio instead.

**`track_one` produces no score at all** — it is pure refinement. The server
carries the registration score forward and labels it `score_source:
carried_from_register` rather than inventing a fresh one.

**Registration lands in the wrong basin on roughly 1 in 30 frames, and this is
not a tuning artefact.** Measured over 30 independent registrations on mustard0
(`training/pose/registration_quality.py`, `docs/results/registration_quality.json`):
at 2, 3, 4 *and* 5 refine iterations, frame 634 registers **121 mm and 162°**
from the pose the validated tracker holds at that frame. All four land in the
same wrong basin; only **6 iterations** escapes it. Everywhere else every arm
agrees to under 1% of object diameter.

Nothing downstream catches such a failure: fusion's disagreement gate keeps the
higher-scoring view, and the score is degenerate per the finding above, so the
tie-break is arbitrary. A badly registered view wins it about half the time.

**This is why `--register-iters` ships at 6 rather than upstream's 5.** It costs
~1.2 s against ~1.05 s, and registration comes off the pair loop's critical path
once async registration lands, so the 15% is cheap against a silent 1-in-30
pose error. **Pass `--register-iters 3` when registration latency actually
matters** — a live demo, a throughput run — which is ~0.7 s and carries the same
failure rate 5 did.

**The ROI crop moves the principal point.** The server subtracts the crop origin
from `cx`/`cy`. Passing the full-frame `K` would put the object at the wrong
bearing — a systematic pose error that reads as a calibration fault.

**Registration wants a segmentation mask, and we send a box.** The mask is the
ROI rectangle gated by depth validity, so it includes background at the object's
depth. A YOLOv8-seg mask would be strictly better and is the upgrade path — this
is why registration quality must be measured, not assumed.

## Acceptance criteria status

| Criterion (`AGENTS.md` §4) | Status |
|---|---|
| Mock mode (identity or replayed ground truth) | Met |
| Poses emitted in the camera optical frame only | Met — nothing here knows about extrinsics |
| Detections with no registered mesh dropped with a warning | Met — asserted at the stronger "never reaches the estimator" level |
| Latency per object measured and logged | Met — feeds the R4 decision |
| R1 gate resolved | Met — see `docs/R1_STATUS.md` |
| ADD-S < 10% of diameter, single view, unoccluded | **Not measured** — needs the evaluation harness pointed at real captures with ground truth |
