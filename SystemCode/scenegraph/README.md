# scenegraph — relational predicate extraction

Converts a fused `SceneState` into symbolic relational predicates and a PDDL
`(:init ...)` block for a task planner. Written from first principles per
`CLAUDE.md` constraint 2 — no scene-graph library. PM1's planner uses a
different vocabulary; only `clear` matches (`docs/PM1_INTEGRATION.md` §2.2).

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
| `(occluded_from a v)` | bit for `v` unset in `a.observed_views` (in `a.visible_views` when a producer does not supply it) |

Two decisions worth knowing about:

- **Containment is exclusive.** An object that is `in` anything is `on` nothing
  — not merely not on its container. A ball in a shallow bin has its base within
  the `on` tolerance of the table top beneath the bin, and `(on ball table)`
  would tell PM1 it can pick the ball straight off the table. This is a global
  rule applied by `Extract`, so the pairwise `IsOn(a, b)` can return true for a
  pair that produces no fact. Found by the end-to-end pipeline test, not by the
  module's own fixtures.
- **`occluded_from` is read off fusion's bitmask of the views that saw the
  object, not re-derived geometrically.** Fusion already knows which views had
  an estimate of it; re-deriving it here would create a second source of truth
  that can disagree with the first. The mask is `observed_views`, not
  `visible_views` (2026-10-11): a view the disagreement gate dropped saw the
  object, and reading its missing `visible_views` bit as "hidden" reported
  unoccluded objects as occluded. What it still cannot tell apart is a view
  the object is hidden from and a view whose detector missed it.

Tolerances come from `config/predicates.yaml` and are never hard-coded — a
missing or out-of-range value throws rather than falling back to a default,
because a wrong tolerance yields a plausible-looking wrong scene graph.

## The support check (2026-10-05)

With `support.require_support` on (shipped on), an object is reported only if
it is **grounded**: its class is in `support.ground_classes` (the table), or it
is `on` or `in` an object that is grounded, through any number of steps.
Anything else — floating, sunk through the table, or resting only on something
that itself rests on nothing — is left out of every fact, as the subject and as
the thing another object is on, and named in `PredicateResult::unsupported`.
`Annotate` leaves it in the `SceneState` with empty `relations`: the scene is
fusion's account of what it tracked.

**Why it is this module's job.** Nothing upstream can tell these objects from
real ones. From the first scored recordings (2026-10-05; 300 pairs a scene):

| what fusion tracked | scene | pose score | where it was |
|---|---|---|---|
| the cube, correctly, from the clear view | `t3_cube_over` | 0.64 | on the table, 1.8 mm from the truth |
| the same cube from the half-hidden view, at the occluder's range | `t3_cube_over` | 0.86 | 171 mm away, 120 mm up |
| a cube fitted to the red print on the occluder card | `t3_cube_front` | 0.90 | 178 mm away, 114 mm up |
| the grey tray, detected as a `bowl` | every cube scene | 0.87–0.97 | 18–62 mm *below* the table top |

The wrong ones outscore the right one, so neither a detector threshold nor the
fusion gate's score ordering separates them. What they share is geometric and
relational: none of them rests on anything. That is a fact about the scene, not
about any one estimate, and this is the first stage that reasons about the
scene.

Three things about how it is done:

- **Grounding is a least fixed point**, so support runs through a stack and
  through a container, and resting on something unsupported is not support.
- **It is decided on the pairwise rules, before containment is made exclusive.**
  Otherwise a phantom container takes a real object with it: the object has
  lost its `on` facts to the container, and then loses its only support to the
  check. As built, the container goes and the object reads as on the table
  (`SupportCheck.AnUnsupportedContainerDoesNotTakeItsContentsWithIt`).
- **Unsupported objects are a list, not warnings.** A warning is an event; a
  tray taken for a bowl is there on every frame, and the runner prints every
  warning. The one warning this adds is for a scene with no ground-class object
  at all, which is a configuration error — and `the_eyes_run check` refuses to
  start on it.

**Measured.** The six scored cube scenes and `t1_bottle` of 2026-10-05,
reprocessed with the check on (`scripts/scene_graph_audit.py 20261005`; an
object-pair is one tracked object on one frame pair):

| | without the check | with it |
|---|---|---|
| the real object, in the scene graph | 2,100 of 2,100 object-pairs | 2,100 of 2,100 |
| objects that are not there, in the scene graph | 2,771 of 2,771 | **1 of 2,771** |
| facts emitted | 10,690 | 5,150 |
| facts naming an object that is not there | 5,543 | **3** |

`t3_cube_over` — one cube on the table, half hidden from cam_over — went from
eleven distinct facts over the run, among them `(on red_cube_01 bowl_01)` and
three cubes that do not exist, to the three that are true on every pair:
`(clear red_cube_01)`, `(occluded_from red_cube_01 cam_over)`,
`(on red_cube_01 table_01)`. Fusion is untouched: the ADD-S tables of all six
cube scenes are identical before and after.

These are the scenes that suggested the rule, each with one real object. Its
test is the three clutter scenes of 2026-10-07, three real objects each, on
the same synthetic-only detector (`scripts/scene_graph_audit.py 20261007
--run 5`, the last run before the fine-tuned detector): all 2,700 real
object-pairs kept, all 19 false ones removed, and 0 of 5,610 facts name an
object that is not there. On the 2026-10-04 rehearsal — bowl, bottle and
cube, TV on, not a scored result — reading the same decision off its facts
keeps all 450 real object-pairs and removes 101 of 115 false ones: the other
14 are a false cube that landed at table height.

Since 2026-10-17 the shipped detector is fine-tuned on real frames and reports
no false object in any of the 22 recorded scenes, so on the current logs the
check has nothing left to remove. It stays on for what a detector can still
get wrong.

What it does not do: a false object that happens to land at table height is
supported, and stays. And it asserts a static scene — an object in a gripper
rests on nothing by this rule; a consumer with a hand in the scene declares it
a ground class or turns the check off.

## Acceptance criteria status

| Criterion | Status |
|---|---|
| Deterministic: same `SceneState` in → same fact set out | Met — asserted over 25 input permutations |
| Tolerances from config, no hard-coded constants | Met |
| Edge-case fixtures (stack, container, single-view occlusion, unknown object) | Met |
| An object that rests on nothing is not reported (2026-10-05) | Met — seven single-object scenes: 1 of 2,771 false object-pairs left, 0 of 2,100 real ones lost; three clutter scenes: 0 of 19 left, 0 of 2,700 lost |
| Predicate accuracy ≥ 95% vs. hand-labelled ground truth | **Met for `on`, `clear`, `in`; not for `occluded_from` precision.** 22 scenes, 6,600 pairs (`scripts/predicate_accuracy.py`): precision / recall `on` 0.994 / 1.000, `clear` 0.995 / 1.000, `in` 1.000 / 1.000, `occluded_from` **0.557** / 1.000. Why `occluded_from` falls short: final report §9.7 |
| Emitted PDDL parses in PM1's planner unmodified | **Not met; dropped with the PM1 integration (2026-09-08).** Only `clear` of PM1's eleven predicates matches, and PM1's parser accepts unknown facts silently, so the facts would need a mapping layer. Fused poses given to PM1 as `scene_context` planned with no code changed on either side (`docs/PM1_INTEGRATION.md`) |
