# PM1 integration — studied, then dropped

**Decision 2026-09-08: not pursued.** Cut order #3 is taken.

This file was a build plan. It is now a closed record: the decision, and the
findings that outlive it. Nothing here is scheduled work.

---

## 1. The decision

PM1 integration is dropped. It was already cut #3 under schedule pressure
(`CLAUDE.md`), and the 2026-09-08 study below concluded that the live bridge was
not worth building ahead of the console or real ADD-S. Dropping it costs no
graded claim: none of the four PRS technique categories depends on it, and the
proposal already rules the physical arm out of scope ("Integration with the
physical SO-ARM101 arm is a stretch demonstration, not a graded deliverable",
§1.2).

**One correction for the record.** The reason given at the time was the absence
of a robot arm. That was not the blocker. PM1 drives a *simulated* Franka in
Isaac Sim, and the path tested below — perception → knowledge graph → planner —
never reaches an executor at all: it produced valid plans on this machine with
no arm, real or simulated, involved. The decision stands on schedule grounds; the
arm is not the reason and should not be cited as one in the report.

---

## 2. What the study measured

Method: PM1's already-built `planner_node` (2026-05-01) and its unmodified
Python modules, driven with real output from the two 2026-09-06 live runs in
`runs/runtime/` (`20260906_211307_cube_fixed2`, 150 pairs;
`20260906_183408_rig_fp`, 120 pairs), read out of `metrics.sqlite`.

### 2.1 It works, and it needed no code on either side

Feeding `scene_context` (not PDDL) to PM1's `POST /world_state` ingress, both
runs planned on the first try with **zero changes to PM1**:

```
cube_fixed2 pair 179 -> predicates=17 actions=21 -> exit 0
  pick(franka,red_cube_01,zone_b) -> move_to(franka,zone_b,zone_c)
  -> place(franka,red_cube_01,zone_c)

rig_fp      pair 149 -> predicates=32 actions=45 -> exit 0
  move_to(franka,zone_b,zone_a) -> pick(franka,red_cube_01,zone_a)
  -> move_to(franka,zone_a,zone_c) -> place(franka,red_cube_01,zone_c)
```

Over the live hub with the real C++ planner behind it, `POST /world_state`
returned `200 ok` and `POST /goal` produced a plan; llama3:8b mapped "the bowl"
to our instance id `bowl_01` unprompted and emitted valid PDDL in 1.5 s.

**This transcript is the surviving deliverable.** Cut #3's stated acceptance is
"logged output suffices", and real fused poses driving PM1's real planner to real
plans is that logged output — obtained without building anything. Whether to
present it that way in the report is a call for the report.

### 2.2 Our predicate vocabulary does not match PM1's domain

`INTERFACES.md` §6 asserted it "must match PM1's domain exactly". Measured against
`SystemCode/planner/domain/tabletop.pddl`, one predicate of eleven matches:

- `(clear ?a)` — matches, name, arity and meaning.
- `(on ?a ?b)` — **type mismatch.** We pass `table_01`, an `ObjectState`; PM1's
  `on` takes `?loc - location`, and carries "resting on the table" separately as
  `on_table`.
- `(in ?a ?b)`, `(occluded_from ?a ?view)` — no counterpart in PM1's domain.
- PM1 needs nine predicates we never emit, including `on_table`, `hand_empty`
  and `at ?robot ?loc`. A robot is not in our scene graph at all.

And the mismatch fails **silently**: `PddlParser::ground` interns every init fact
with no predicate, arity or type check, so an unknown fact becomes an inert bit
and the symptom is `No plan found after 1 expansions` — indistinguishable from an
unsolvable goal.

This is a fact about *our own* documentation and it holds whether or not anyone
integrates. `INTERFACES.md` §6 is corrected accordingly.

### 2.3 Neither project ships ROS2

Zero matches for `rclpy`, `rclcpp`, `std_msgs` or `sensor_msgs` anywhere in PM1's
`SystemCode/`; no `rclpy` in its `requirements.txt`; `ros2_bridge/README.md` is a
one-line stub and that module is FastAPI. This box carries only `/opt/ros/jazzy`.
The Humble/Jazzy conflict recorded in `CLAUDE.md` never existed in the shipped
code. That note is corrected too.

### 2.4 One note for anyone who revisits this

PM1's `state.hpp` fixes `MAX_PREDICATES = 512` over a `std::bitset`. At 17
objects the grounding reaches 532 predicates and the planner **aborts** —
`bitset::reset: __position (which is 513) >= _Nb (which is 512)`, thrown from
`GroundedAction::apply()` during search, outside `main`'s try/catch, so SIGABRT
rather than a clean exit. Growth is O(objects²) via `stacked_on`. Our scope is
≤6 objects and the `rig_fp` run peaked at 4 per pair, so it never bit — but an
adapter accumulating ids across a session would have walked into it (that run
produced 8 distinct ids over 120 pairs from detector flicker).

---

## 3. What this changes elsewhere

Done as part of closing this out:

- `INTERFACES.md` §6 — the "must match PM1's domain exactly" claim is replaced
  with what was measured.
- `CLAUDE.md` — the ROS2 Humble note and cut-list item 3 are corrected.

Left alone, because they are the author's call on an actively revised document:
the proposal still lists the PM1 integration demo at §1.3 (cut order), §9 (P5),
§11.1 and §11.2 (Final Presentation, "live PM1 integration if stable"). Those
lines describe a cut item that has now been cut, which is self-consistent, but if
the report should say so explicitly they are the four places to edit.
