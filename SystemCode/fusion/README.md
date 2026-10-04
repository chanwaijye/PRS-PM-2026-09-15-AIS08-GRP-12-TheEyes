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

**Symmetric objects are compared with their symmetry removed (2026-10-04).** Two
views of a symmetric object legitimately report orientations differing by a
symmetry-group element; the mean of those is a rotation neither view saw. The
mesh registry now names the group (`symmetry_group: cubic | axial_z`,
`INTERFACES.md` §7), and `symmetry.hpp` maps any orientation to the member of
its symmetry class nearest a reference — a search over the cube's 24 rotations,
a closed-form twist about z for the bowl. It is used in three places:

- **the cross-view fuse** maps every view onto the best-scoring one and then
  averages, where it used to take the best view and discard the rest;
- **the gate** measures orientation disagreement with the symmetry removed, so
  88° between two views of a cube is agreement and 40° is a conflict — before,
  a symmetric object's orientation could never trip it;
- **the EKF update** moves each measurement to the equivalent nearest the
  track, so the filter never sees a jump that is not a change of pose.

The third is the one a real run forced. On the rig the two cameras reported the
printed cube 88° and 178° apart; the fused orientation was the best-scoring
view's, so it jumped whenever cam_over missed the cube (43 of 300 pairs), and
the filter smoothed through orientations that are not the cube:

| cube alone, 300 pairs | ADD-S | inside 10% of diameter |
|---|---|---|
| gate + EKF, before | 8.54 mm | 58.3% |
| gate + EKF, after | **1.90 mm** | **100%** |
| gate alone, after (before: 3.04 mm) | 2.74 mm | 99.7% |
| cam_front / cam_over alone | 2.68 / 4.91 mm | 100% / 77.3% |
| naive, either filter setting | 22.5–23.2 mm | < 15% |

Rehearsal figures (2026-10-04, daylight), not scored results. A class flagged
`symmetric` with no group declared keeps the old behaviour — one view's
orientation taken, never gated — and `kNaive` has no symmetry handling at all,
by definition, which is why its cube figure does not move.

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

`RunOcclusionAblation()` runs one identical estimate stream through **eight
arms**, each with a fresh `Fusion`, and reports ADD-S. `CLAUDE.md`: this result
is never cut, so the harness is treated as a deliverable and is itself tested.

Three things differ between arms and the harness varies them one at a time:
which **views** (one camera or both), which **policy** (naive unweighted mean
or the gated, weighted, symmetry-aware fuse), and whether the **temporal
filter** (the per-object EKF) is applied. Rows come in two tiers, raw and EKF:

| arm | views | policy | filter |
|---|---|---|---|
| `cam_front`, `cam_over` | one | — | raw |
| `naive` | both | naive | raw |
| `gated` | both | gated | raw |
| `cam_front+ekf`, `cam_over+ekf` | one | — | ekf |
| `naive+ekf` | both | naive | ekf |
| `fused` | both | gated | ekf |

Read it as pairs that differ in exactly one thing. `gated − naive` and
`fused − naive+ekf` are what **this module's policy** buys, at each filter
setting; `naive − cam_*` is what a second camera buys; `*+ekf − *` is what the
EKF buys.

### The confound this design removes — [FIXED 2026-09-06]

Until 2026-09-06 the harness ran four arms, and `naive` was defined as
"unweighted mean **and** no filter" while the single-view and `fused` arms both
ran the EKF. On a static object with independent per-frame noise the EKF is a
running average over time, and on the demo scene it cuts error from ~9 mm to
~6.3 mm on its own. The reported fused-minus-naive gap — and the proposal's
"a second camera makes matters worse" reading — was therefore the filter, not
the policy: with the filter made transparent, naive 9.00 mm vs fused 9.01 mm.

`FusionConfig::temporal_filter` is now a switch independent of
`FusionConfig::mode`, and `Ablation.TheFilterAndThePolicyAreNotConfounded`
pins the property: under symmetric noise with no outliers the policy delta is
zero at both filter settings and the filter delta is large at both policy
settings. On Gaussian noise the policy **cannot** win, because the gate has
nothing to reject and equal scores make the weighted mean the plain mean. Where
it does win is a confidently wrong view, which
`GatedFusionBeatsTheNaiveBaselineWhenOneViewIsAnOutlier` now asserts at both
filter tiers — a constant bias is not noise and the EKF cannot average it away.

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
| Symmetric objects not naively orientation-averaged | Met — three tests for the undeclared case, plus `test_symmetry.cpp` and four `Fusion.*` tests for a declared group, one of which replays the rig's failure |
| Disagreement gate rejects an injected outlier | Met — three- and two-view branches |
| Single-view objects pass through with correct `visible_views` | Met |
| Refuses to start on extrinsics failing the quality gate | Met — throws in the constructor |
| Occlusion ablation harness emitting comparable ADD-S | Met — eight-arm harness + demo; filter and policy varied independently since 2026-09-06 |
| Unknown-object merge from the clustering path | **Not started** — `cluster_agent` is above MVP and does not exist yet |

## Known gaps

- **`extent` for unknown objects.** Adding `extent` to `PoseEstimate` (§3,
  additive) was necessary because nothing upstream supplied `ObjectState::extent`.
  Known classes can fall back to the `ExtentRegistry`; the DBSCAN path must carry
  its cluster bounding volume on the estimate. Fusion warns when an object ends
  up with zero extent, because downstream predicates over it are degenerate.
- **`config/extrinsics.yaml` is a placeholder.** Synthetic values for mock-mode
  development. Both serials ARE recorded and correct — `cam_front`
  `261722073945`, `cam_over` `261822074567` — so only the transforms are
  synthetic. Replace those wholesale after rigid mounting and `calib_agent`.
- **Tuning is unvalidated.** Every threshold in `config/fusion.yaml` is a
  reasoned starting point, not a measured one. The gate and association radii in
  particular should be re-derived from the depth-noise characterisation
  (`camera_agent`) once that report exists.
- **SQLite metric logging lives in `runtime_agent`, not here.** `the_eyes_run`
  logs every per-view `PoseEstimate` and every fused `ObjectState` to
  `runs/runtime/<session>/metrics.sqlite`, and `ablation_from_log` rebuilds
  `AblationFrame`s from that log to run `RunOcclusionAblation` on real captures
  once ground truth is available. The synthetic demo still prints to stdout.
