# scenegraph — relational predicate extraction

Converts a fused `SceneState` into symbolic relational predicates and a PDDL
`(:init ...)` block for PM1's planner. Written from first principles per
`CLAUDE.md` constraint 2 — no scene-graph library.

Consumes `INTERFACES.md` §5, produces §6.

## Build and test

```bash
cmake -S SystemCode -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

Dependencies: Eigen3, yaml-cpp, GTest. **No hardware, no models, no ROS.**

## Mock mode

The fixture-based test suite *is* the mock mode — this module is a pure function
from `SceneState` to a fact set, so there is nothing to stub. Fixtures live in
`test/test_predicates.cpp` and cover the cases named in `AGENTS.md` §6:
stacked objects, object in container, object occluded from exactly one view, and
an unknown object present.

## The rules

All geometry is world-axis-aligned: `ObjectState::extent` is world-axis-aligned
half-extents, so `pose_w`'s rotation never enters a predicate. That is deliberate
— it is what makes `unknown_object`, whose rotation is meaningless by contract,
safe to reason about, and it is asserted by
`Predicates.UnknownObjectParticipatesUsingCentroidAndExtentOnly`.

| Predicate | Rule |
|---|---|
| `(on a b)` | `a`'s base within `on.surface_tol_m` of `b`'s top, AND ≥ `on.min_overlap_ratio` of `a`'s footprint over `b`, AND not `(in a b)` |
| `(in a b)` | `a`'s centroid inside `b`'s AABB inset by `in.containment_margin_m`, AND `vol(a) ≤ in.max_volume_ratio · vol(b)` |
| `(clear a)` | no `x` satisfies `(on x a)` |
| `(occluded_from a v)` | bit for `v` unset in `a.visible_views` |

Two decisions worth knowing about:

- **Containment is exclusive.** An object that is `in` anything is `on` nothing
  — not merely not on its container. A ball in a shallow bin has its base within
  the `on` tolerance of the table top beneath the bin, and `(on ball table)`
  would tell PM1 it can pick the ball straight off the table. This is a global
  rule applied by `Extract`, so the pairwise `IsOn(a, b)` can return true for a
  pair that produces no fact. Found by the end-to-end pipeline test, not by the
  module's own fixtures.
- **`occluded_from` is read off the visibility bitmask, not re-derived
  geometrically.** Fusion already knows which views contributed; re-deriving it
  here would create a second source of truth that can disagree with the first.

Tolerances come from `config/predicates.yaml` and are never hard-coded — a
missing or out-of-range value throws rather than falling back to a default,
because a wrong tolerance yields a plausible-looking wrong scene graph.

## Acceptance criteria status

| Criterion | Status |
|---|---|
| Deterministic: same `SceneState` in → same fact set out | Met — asserted over 25 input permutations |
| Tolerances from config, no hard-coded constants | Met |
| Edge-case fixtures (stack, container, single-view occlusion, unknown object) | Met |
| Predicate accuracy ≥ 95% vs. hand-labelled ground truth | **Blocked** — needs the benchmark scene suite, which needs real captures |
| Emitted PDDL parses in PM1's planner unmodified | **[VERIFY]** — only balanced-paren and vocabulary checks run here; not yet fed to PM1 |
