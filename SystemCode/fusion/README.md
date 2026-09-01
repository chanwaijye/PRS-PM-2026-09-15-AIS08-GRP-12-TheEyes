# fusion — multi-view object-level pose fusion ★

Primary technical contribution. Written from first principles per `CLAUDE.md`
constraint 2: no packaged fusion, no packaged pose averaging, no packaged EKF.
Eigen for linear algebra only.

Consumes `PoseEstimate` (`INTERFACES.md` §3) and `extrinsics.yaml` (§4);
produces `SceneState` (§5).

## Build and test

```bash
cmake -S SystemCode -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
build/fusion/fusion_ablation_demo config     # synthetic ablation table
```

Dependencies: Eigen3, yaml-cpp, GTest. **No hardware, no models, no ROS.**

## Pipeline

```
per-view PoseEstimate
  -> world frame            extrinsics.cpp   T_world_cam * pose_cam
  -> cross-view association fusion.cpp       greedy NN within class,
                                             one estimate per camera per cluster
  -> disagreement gate      fusion.cpp       drop outlier views
  -> confidence-weighted    quaternion_average.cpp  Markley eigendecomposition
     fuse                                    + score-weighted centroid
  -> track association      fusion.cpp       stable ids across frames
  -> per-object EKF         ekf.cpp          error-state, 9-dim
  -> SceneState
```

`FuseFrame()` is the stateless half (association through fuse) and is exposed
separately so the ablation harness can run a view subset without a `Fusion`
instance and without temporal state confounding the comparison.

## Design decisions worth knowing

**Markley for orientation.** `M = Σ wᵢ qᵢ qᵢᵀ`, average is the principal
eigenvector. The outer product is sign-invariant, which is the point: two views
reporting `q` and `-q` describe the same rotation, and componentwise averaging
cancels them to zero. `QuaternionAverage.IsInvariantToInputSign` asserts both
halves of that.

**Symmetric objects take, they do not average.** Two views of a symmetric object
legitimately report orientations differing by a symmetry-group element; the mean
of those is a rotation neither view saw. The highest-scoring view's orientation
is used instead, and symmetry-driven orientation disagreement does not trip the
gate. The principled version canonicalises each quaternion into a fundamental
domain of the symmetry group first — blocked on `INTERFACES.md` §7 carrying
only a `symmetric` bool rather than the group.

**Two cameras have no majority.** With three or more views the gate measures each
against the consensus and drops outliers. With exactly two, the consensus sits
midway, so both views are equidistant from it and a residual test rejects both or
neither. The two-view branch compares the views to *each other*, keeps the
higher-scoring one, and multiplies confidence by
`two_view_conflict_confidence_penalty` — recording that we guessed rather than
reporting a consensus that does not exist. `FusedObservation::two_view_conflict`
carries this to the caller.

**Association radius must exceed the gate radius.** These answer different
questions in sequence: association asks "could these be the same object?", the
gate then asks "do they agree well enough to average?". A gate wider than the
association radius is silently inert — the disagreeing views it exists to judge
have already been split into separate objects, so it never sees a cluster to
reject anything from. `LoadFusionConfig` enforces `max_translation_m <
max_distance_m` and refuses to load otherwise. This was a real bug in the first
version of `config/fusion.yaml`, caught by the gate tests.

**Error-state EKF, not a quaternion Kalman filter.** State is nominal `(p, v, q)`
with error `[δp, δv, δθ] ∈ R⁹`. Keeping orientation error minimal (3-dim, in the
tangent space at `q`) avoids the singular covariance that the unit-norm
constraint forces on a 4-vector formulation, and avoids drift off the manifold as
updates accumulate. Covariance update is Joseph form: the textbook `(I-KH)P` is
algebraically equivalent but loses symmetry to rounding over a long run, and an
asymmetric covariance eventually diverges silently.

**No angular-velocity state.** Tabletop objects are moved by a robot in discrete
pick-and-place steps, not tumbling continuously, so a rotation-rate estimate
would have almost nothing to lock onto between events. Orientation is a random
walk; position is constant-velocity.

**Confidence is a score-weighted mean, not a noisy-OR.** Multi-view agreement is
already reported through `visible_views`; folding it into confidence too would
double-count it.

**Unobserved tracks are kept but not emitted.** `visible_views == 0` is invalid
per §5, so an object nothing saw this frame is not in the `SceneState`. Its track
survives for `ekf.max_coast_s` so a briefly-occluded object keeps its id.

## Occlusion ablation

`RunOcclusionAblation()` runs one identical estimate stream once per single view
and once fused, each with a fresh `Fusion`, and reports ADD-S. `CLAUDE.md`: this
result is never cut, so the harness is treated as a deliverable and is itself
tested.

ADD-S uses nearest-point rather than corresponding-point distance, so a pose off
by a symmetry element is not punished — `Metrics.AddSForgivesASymmetryWhereAddDoesNot`
pins the difference from ADD. `recall@0.1d` counts *all* ground-truth instances in
its denominator, not just matched ones, so a view that fails to see an object
cannot score well on the few it does see.

`fusion_ablation_demo` prints the table on a synthetic scene. Its numbers are
Gaussian noise and scripted occlusion, **not** an evaluation result, and it says
so in its own output.

## Acceptance criteria status

| Criterion (`AGENTS.md` §5) | Status |
|---|---|
| Synthetic recovery: fused error below single-view | Met — `FusedErrorIsBelowSingleViewErrorUnderGaussianNoise`, asserts < 0.85× |
| Symmetric objects not naively orientation-averaged | Met — three tests |
| Disagreement gate rejects an injected outlier | Met — three- and two-view branches |
| Single-view objects pass through with correct `visible_views` | Met |
| Refuses to start on extrinsics failing the quality gate | Met — throws in the constructor |
| Occlusion ablation harness emitting comparable ADD-S | Met — harness + demo |
| Unknown-object merge from the clustering path | **Not started** — `cluster_agent` is above MVP and does not exist yet |

## Known gaps

- **`extent` for unknown objects.** Adding `extent` to `PoseEstimate` (§3,
  additive) was necessary because nothing upstream supplied `ObjectState::extent`.
  Known classes can fall back to the `ExtentRegistry`; the DBSCAN path must carry
  its cluster bounding volume on the estimate. Fusion warns when an object ends
  up with zero extent, because downstream predicates over it are degenerate.
- **`config/extrinsics.yaml` is a placeholder.** Synthetic values for mock-mode
  development, and `cam_over`'s serial is `[VERIFY]` — the second D435i's serial
  is not yet recorded. Replace wholesale after rigid mounting and `calib_agent`.
- **Tuning is unvalidated.** Every threshold in `config/fusion.yaml` is a
  reasoned starting point, not a measured one. The gate and association radii in
  particular should be re-derived from the depth-noise characterisation
  (`camera_agent`) once that report exists.
- **No SQLite metric logging yet.** `CLAUDE.md` asks for it; the ablation result
  is currently returned in-process and formatted to stdout.
