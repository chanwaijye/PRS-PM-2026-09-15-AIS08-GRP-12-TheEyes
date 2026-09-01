# integration — cross-module pipeline tests

Tests and demos that span module boundaries. They live here rather than in any
one module because `AGENTS.md` orchestration rule 1 forbids one module's
agent editing another's files — the seam between two modules belongs to neither.

## Build and run

```bash
cmake -S SystemCode -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
build/integration/pipeline_demo
```

No hardware, no GPU, no models.

## What is covered

**`test_pipeline.cpp`** starts at fusion with hand-built `PoseEstimate`s and
checks fusion → scenegraph → PDDL.

**`test_full_pipeline.cpp`** starts at the detector boundary and runs every
module in between:

```
DetectionSet (§2) -> pose_agent x2 -> PoseEstimate (§3) -> fusion
  -> SceneState (§5) -> scenegraph -> PDDL (:init ...) (§6)
```

so a contract mismatch anywhere shows up here rather than at integration time on
real hardware. It asserts, specifically:

- detections become PDDL facts through every module unchanged
- `extent` and `symmetric` survive from the mesh registry to the scene graph —
  losing either would degenerate predicates or let fusion average a symmetric
  object's orientation
- an estimator declining for one view surfaces as `(occluded_from ... cam_over)`
- a class missing from the mesh registry is dropped at `pose_agent` and never
  appears downstream
- fused ADD-S beats either single view *through the real module chain*, not just
  through fusion's internal synthetic path

## `pipeline_demo`

Runs the whole chain on a synthetic tabletop scene and prints the occlusion
ablation plus the PDDL block PM1 would consume. **Its numbers are synthetic** —
Gaussian per-view noise, scripted occlusion — and it says so in its own output.

Representative run (150 frames, 4 objects, 10 mm / 3° per-view noise,
`mustard_bottle` occluded from `cam_over` on 3 of every 4 frames):

```
view              gt  matched  missed    ADD-S (mm) ADD-S (/diam)   recall@0.1d
cam_front        600      600       0          9.12        0.0449        0.8650
cam_over         600      488     112         10.12        0.0458        0.6917
fused            600      600       0          7.59        0.0376        0.9150
```

Fusion wins on both axes: **17% lower ADD-S** than the better single view, and
it recovers the 112 instances `cam_over` could not see at all. That shape — an
accuracy gain *and* a recall gain — is the claim the project rests on, and this
is it running through the real modules rather than a fixture.

The latency lines report ~0 ms because the mock backend is a table lookup; they
measure orchestration overhead only. The real FoundationPose backend measured
p50 11.1 ms end to end (`SystemCode/pose/README.md`).
