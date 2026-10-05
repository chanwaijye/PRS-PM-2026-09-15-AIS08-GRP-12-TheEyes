# INTERFACES.md — The Eyes

Data contracts between modules. **Changing anything here requires updating every
consumer.** `ObjectState` additionally has downstream consumers outside this repo
(PM1 planner, PM4 world model) — fields may be added, never removed or re-typed.

---

## 1. Camera → Detector

ROS2 topics, one namespace per camera, selected by serial number.

```
/cam_front/color/image_raw          sensor_msgs/Image
/cam_front/aligned_depth_to_color/image_raw   sensor_msgs/Image
/cam_front/color/camera_info        sensor_msgs/CameraInfo
/cam_over/...                       (same set)
```

Requirements:
- Depth **aligned to color** — FoundationPose needs registered RGB-D.
- Hardware-synced (`inter_cam_sync_mode` 1/2). Frame timestamps across the two
  namespaces must agree within one frame period; assert this at startup.
- Camera namespace ↔ serial number mapping lives in the launch file, not in code.

*Shipped implementation note (2026-09-06):* the topics above are the documented
contract; the runner that ships (`SystemCode/runtime/`, `the_eyes_run`) is one
C++ process with no ROS2 dependency and carries the same payload in-process as
`camera::CameraFrame` (RGB, aligned depth, K, source timestamp, serial). The
serial-number mapping lives in `config/cameras.yaml`. No field or semantics
differ; a topic adapter would sit on top of the same structs.

---

## 2. Detector → Pose Estimator

```cpp
struct Detection {
  std::string cls;        // YOLO class name, must match a key in the mesh registry
  float score;            // detector confidence [0,1]
  cv::Rect2i bbox;        // pixel coords in the color frame
  uint32_t track_id;      // stable across frames within one camera; 0 = untracked
};

struct DetectionSet {
  std::string camera_id;  // "cam_front" | "cam_over"
  uint64_t stamp_ns;      // source frame timestamp, NOT receive time
  std::vector<Detection> detections;
};
```

Class names are the join key to the CAD mesh registry. A detection whose class has
no registered mesh is dropped with a warning — it must not reach the pose
estimator.

The detector consumes **both** the color and the aligned-depth topic (§1): it is a
4-channel RGB-D network, not RGB-only. If aligned depth is unavailable for a
frame, the detector must skip that frame rather than substitute zeros — a
zero-filled depth channel is indistinguishable from "object at zero range" to the
network.

---

## 3. Pose Estimator → Fusion

```cpp
struct PoseEstimate {
  std::string camera_id;
  std::string cls;
  uint32_t track_id;
  uint64_t stamp_ns;
  Eigen::Isometry3d pose_cam;   // 6-DoF in THIS camera's optical frame
  double score;                 // FoundationPose hypothesis score [0,1]
  bool symmetric;               // from mesh registry; fusion must not average
                                // orientation naively for symmetric objects
  Eigen::Vector3d extent;       // world-axis-aligned half-extents of the
                                // bounding volume; zero means "not supplied"
};
```

`extent` was added so fusion can populate `ObjectState::extent` (§5), which
nothing upstream previously supplied. For a known class it is the CAD mesh AABB
and may instead be looked up by class; for the `unknown_object` clustering path
it is the cluster bounding volume, which exists only per-estimate and therefore
has to travel on the estimate itself. Additive change — no existing field moved.

Poses are emitted in the **camera optical frame**. Transformation to the world
frame is the fusion module's job, using extrinsics from calibration — the pose
estimator must not know about world frames.

---

## 4. Calibration → Fusion

```yaml
# config/extrinsics.yaml — produced by calib_agent, consumed by fusion
world_frame: table_origin
cameras:
  cam_front:
    serial: "XXXXXXXXXXXX"
    T_world_cam: [16 doubles, row-major 4x4]
    reprojection_rmse_px: 0.0        # calibration quality gate
  cam_over:
    serial: "XXXXXXXXXXXX"
    T_world_cam: [...]
    reprojection_rmse_px: 0.0
```

Fusion refuses to start if `reprojection_rmse_px` exceeds the configured
threshold — bad extrinsics silently poison every fused pose, so fail loudly.

---

## 5. Fusion → Scene Graph  (and the external contract)

**This is the stable contract. PM1 and PM4 depend on it.**

```cpp
struct ObjectState {
  std::string id;              // stable instance id, e.g. "red_cube_01"
  std::string cls;             // YOLO class, or "unknown_object"
  Eigen::Isometry3d pose_w;    // fused 6-DoF pose, world frame
  double confidence;           // fused confidence [0,1]
  std::vector<std::string> relations;  // filled by scenegraph, empty from fusion
  uint32_t visible_views;      // bitmask: bit0 = cam_front, bit1 = cam_over
  Eigen::Vector3d extent;      // bounding volume half-extents (world-axis-aligned)
  uint64_t stamp_ns;           // fusion output time

  // Belief, not just an estimate. ADDITIVE — see below.
  double position_sigma_m;         // 1-sigma RMS position uncertainty; <0 = absent
  double orientation_sigma_rad;    // 1-sigma RMS orientation uncertainty; <0 = absent
  Eigen::Vector3d velocity_w;      // world-frame linear velocity, m/s
};

struct SceneState {
  uint64_t stamp_ns;
  std::vector<ObjectState> objects;
};
```

Semantics that matter:
- `visible_views == 0` is invalid — drop the object rather than emit it.
- A single-view object is still emitted, with the corresponding single bit set.
  Downstream consumers use `visible_views` to reason about pose uncertainty.
- `cls == "unknown_object"` means the DBSCAN path produced it: `pose_w`
  translation is the cluster centroid, rotation is identity and **meaningless**,
  `extent` is the cluster bounding volume.

**Uncertainty fields (additive, no existing field moved).** `confidence` is a
frontend score: it says how sure the detector and pose estimator were that they
saw the object. It does not say how well the pose is pinned down in metres, and
those are different questions. Only the second one tells a consumer whether a
geometric test over this pose is safe to trust — a 5 mm surface tolerance means
nothing against a pose with 30 mm of position sigma. The per-object EKF already
computed these every frame and discarded them; these fields carry them.

- Both sigmas are **negative when not supplied**, not zero. Zero is a
  legitimate-looking value that would read as perfect certainty, so a producer
  predating the field — or one running with no temporal filter, such as the
  ablation's naive baseline arm — must be distinguishable from a confident one.
  `has_uncertainty(const ObjectState&)` in `types.hpp` is the check, and a
  consumer reasoning about uncertainty **must** call it first.
- `velocity_w` needs no sentinel: zero is the correct answer for a static
  object, which is the common tabletop case.
- Intended use downstream: predicate extraction can decline to assert a fact
  whose geometric margin is smaller than the pose uncertainty, rather than
  emitting a confident fact the planner cannot check. That consumer is not yet
  built — the fields are supplied first so the contract is stable before
  anything depends on it.

---

## 6. Scene Graph → PDDL / consumers

```
/scene_state        the_eyes_msgs/SceneState     scene graph as JSON-serialisable msg
/scene_pddl         std_msgs/String              PDDL (:init ...) block
```

Predicate vocabulary. This is **our** vocabulary, defined here and consumed by
PM4; it is *not* PM1's. An earlier revision of this line claimed it "must match
PM1's domain exactly" — checked against `SystemCode/planner/domain/tabletop.pddl`
on 2026-09-08, one predicate of eleven matches (`clear`), `(on ?a ?b)` differs in
the type of `?b`, and `in` / `occluded_from` have no counterpart there. Matching is
not required: the live PM1 integration is cut (`docs/PM1_INTEGRATION.md`), and a
consumer-specific encoding belongs in a bridge rather than in this contract.

| Predicate | Meaning | Geometric rule |
|---|---|---|
| `(on ?a ?b)` | a rests on b | a's base within tolerance of b's top surface AND horizontal overlap |
| `(in ?a ?b)` | a inside container b | b's class is declared in `in.container_classes` AND a's centroid within b's interior volume |
| `(clear ?a)` | nothing on top of a | no object satisfies `(on ?x ?a)` |
| `(occluded_from ?a ?view)` | a not visible from view | corresponding `visible_views` bit unset |

Tolerances live in `config/predicates.yaml` — never hard-coded. Predicate
extraction must be deterministic: same `SceneState` in, same fact set out.

**`in` is default-deny on the container class** (added 2026-09-06). `ObjectState`
carries a world-axis-aligned box and nothing else, and that is not enough to
distinguish a container from a slab: on the 2026-09-06 live run a cube pose low
by more than half the 20 mm table thickness put its centroid between the table's
faces and emitted `(in red_cube_01 table_01)` 26 times, satisfying the centroid
and volume-ratio tests while being false. A class absent from
`in.container_classes` therefore can never be the `?b` argument. The key is
required and may be empty, but never omitted — a default would either invent
containers or silently drop the predicate. `unknown_object` never qualifies,
which follows from default-deny rather than a special case.

The asymmetry is deliberate: containment is exclusive in extraction, so a false
`in` suppresses every true `on` fact for the contained object, while a false
negative forgoes only the `in` fact itself.

**An object that rests on nothing is not reported** (added 2026-10-05;
`support:` in `config/predicates.yaml`, shipped on). The fact set covers only
*grounded* objects: an object whose class is in `support.ground_classes` (the
table), or one that is `on` or `in` a grounded object, through any number of
steps. An ungrounded object appears in no fact at all — not as `?a`, not as
`?b`, and with no `(clear ...)` or `(occluded_from ...)` of its own.

- The vocabulary above is unchanged, and so is §5: no `ObjectState` field is
  added, removed or re-typed. The object is still in `SceneState.objects`, as
  fusion tracked it, with empty `relations`.
- Empty `relations` does **not** identify it — a table with something on it has
  none either. A consumer that needs to know takes the list the extraction
  returns (`PredicateResult::unsupported`, `PddlResult::unsupported`); the
  runner logs it per object and per pair as `objects.supported` (1 reported,
  0 left out, NULL when the check did not run).
- Grounding is decided on the pairwise rules, before containment is made
  exclusive, so an ungrounded container cannot take what is inside it out of
  the scene graph: the contents fall back to whatever else they rest on.
- It asserts a **static scene**. Something held in a gripper rests on nothing
  by this rule; a consumer with a hand in the scene declares it a ground class
  or turns the check off.

The reason it is a scene-graph rule and not a detector threshold or a fusion
gate is in `SystemCode/scenegraph/README.md`: the objects it removes carried
higher pose scores than the correct estimate beside them.

*Shipped implementation note (2026-09-06):* `the_eyes_run` emits the
`(:init ...)` block on stdout whenever the fact set changes and logs every
fact, `ObjectState` and per-view `PoseEstimate` to SQLite
(`runs/runtime/<session>/metrics.sqlite`); the `/scene_state` and
`/scene_pddl` topics are not published. The static table object that the
`on` rule needs comes from `config/scene.yaml`, injected by the runner after
fusion and before extraction.

---

## 7. Mesh registry

```yaml
# config/meshes.yaml
objects:
  mustard_bottle:
    mesh: assets/meshes/mustard_bottle.obj   # must be watertight
    symmetric: false
    diameter_m: 0.19                          # used to normalise ADD-S
    extent: [0.048, 0.043, 0.093]             # AABB half-extents, metres
```

`extent` was added alongside `PoseEstimate::extent` (§3), for the same reason:
fusion must fill `ObjectState::extent` and this is where a known class's value
comes from. Derived from the mesh AABB and carried in config so nothing parses
an OBJ on the critical path. Additive change.

`diameter_m` is required — the ADD-S threshold is a fraction of object diameter,
so evaluation cannot run without it.

`symmetry_group` (optional; `none` | `cubic` | `axial_z`) was added on
2026-10-04. `symmetric` says only that two views may report different
orientations of the same pose; fusion needs to know *which* orientations are the
same before it can average them, gate on them or filter them over time. It may
only be set on an entry with `symmetric: true`. Absent means no group is
declared and fusion keeps its older handling for that class. Additive change:
`PoseEstimate::symmetric` and the logged `estimates.symmetric` column are
unchanged, and the group reaches fusion through `FusionConfig::symmetry`, filled
from this file by whoever builds the config.

```yaml
  red_cube:
    symmetric: true
    symmetry_group: cubic      # the 24 rotations of a cube about its centre
  bowl:
    symmetric: true
    symmetry_group: axial_z    # any turn about the object's own z axis
```
