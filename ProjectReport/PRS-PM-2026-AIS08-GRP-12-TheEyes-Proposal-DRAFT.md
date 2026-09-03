# THE EYES

## Multi-View Perception and Scene Understanding for Robot Manipulation

### NUS-ISS Pattern Recognition Systems Practice Module — Project Proposal (DRAFT)

| Field | Detail |
|---|---|
| Date of Proposal | [VERIFY — set on submission; due 15 September 2026] |
| Project Title | The Eyes — Multi-View Perception and Scene Understanding for Robot Manipulation |
| Group ID | Group 12 |
| Cohort | AIS08 |
| Team Members | Chan Wai Jye (A0340644J) |
| Sponsor/Client | N/A — self-initiated research project |
| Submission Type | Solo Submission — AI-Assisted Development |

---
---

## 1. Introduction

A robot that plans intelligently but cannot see is blind. In the preceding IRS practice module, the author built *The Planner* — a hybrid symbolic-neural task reasoning system that converts natural language instructions into executed robot motion. That system established the perception *interface* — the schema of object identities, poses and relations the planner consumes — and demonstrated task reasoning against simulator ground-truth poses. *The Eyes* implements that interface with real sensing: multi-view fusion, occlusion resilience, and quantified sim-to-real transfer, none of which existed in the prior module.

*The Eyes* is a real-time perception system that takes raw RGB-D streams from two cameras at different vantage points and produces a structured, relational scene graph: which objects are present, their full 6-DoF poses, and their spatial relationships (on, in, clear, occluded-by). The output interface is identical to the PDDL state facts consumed by *The Planner*, so the perception layer drops into the existing planning stack without modification — replacing simulator ground truth with genuine machine perception.

### 1.1 Project Goals

- **G1:** Build a supervised deep-learning detection and 6-DoF pose estimation pipeline (YOLOv8 + FoundationPose) achieving usable accuracy on tabletop manipulation objects.
- **G2:** Demonstrate multi-view sensor fusion: two RGB-D cameras at different viewpoints, with pose estimates fused at the object level to resolve occlusion and pose ambiguity.
- **G3:** Produce a sense-making layer — a C++ scene-graph bridge that converts fused 6-DoF poses into symbolic relational predicates.
- **G4:** Quantify the sim-to-real gap: train on synthetic data generated in Isaac Sim, evaluate on real camera captures.
- **G5:** Demonstrate all four PRS technique categories in a single coherent, runnable system.

### 1.2 Scope Statement

The system perceives a tabletop manipulation scene containing up to 6 rigid objects with known CAD models (YCB object set plus custom-printed objects). Sensing hardware is two RGB-D cameras (Intel RealSense class) at complementary viewpoints (front + overhead). Deliverables run on a single workstation (RTX 5070 Ti, 16 GB VRAM), with an edge-deployment feasibility test on Jetson Orin Nano. Object categories are closed-set; open-vocabulary detection is out of scope. Articulated and deformable objects are out of scope. Integration with the physical SO-ARM101 arm is a stretch demonstration, not a graded deliverable.

### 1.3 Minimum Viable Product

The MVP is the minimum slice that satisfies every graded deliverable and all four PRS technique categories:

- **Single-camera pipeline** — YOLOv8 → FoundationPose → 6-DoF poses on the YCB object set, running on a real camera (not simulation only).
- **Second camera + multi-view fusion ★** — the C++ fusion module with the occlusion ablation result (§7). Non-negotiable: this *is* the ensemble claim (③) and the headline experiment.
- **Scene-graph bridge ★** — fused poses → relational predicates → PDDL init facts.
- **Evaluation numbers** — detection mAP, ADD-S single-view vs. fused, predicate accuracy on the benchmark suite.

Items above MVP, cut in this order under schedule pressure (cross-referenced in Risk R5): (1) Jetson Orin Nano deployment test; (2) unknown-object DBSCAN path — cutting it loses the unsupervised half of category ①, but the supervised half still satisfies it; (3) live PM1 integration demo — the interface contract remains demonstrable via logged output; (4) digital-twin reconstruction viewport (§5.5).

---

## 2. Project Background & Motivation

### 2.1 Problem

Industrial pick-and-place deployments spend a disproportionate share of engineering effort on perception, not motion. Structured environments sidestep the problem with fixtures, presenters, and barcodes — at high capital cost and zero flexibility. Unstructured bin-picking and tabletop manipulation require the robot to answer three questions continuously: *what* objects are present, *where* are they (full 6-DoF, not just 2D), and *how do they relate* (which object is on top, which is blocked)?

Single-camera systems fail predictably under occlusion: a mug behind a cereal box is invisible from the front and its pose unrecoverable. Multi-view sensing solves this geometrically, but fusing independent per-view estimates into one consistent world state is a non-trivial pattern recognition problem — and it is the core technical contribution of this project.

### 2.2 Relationship to Prior and Future Modules

This project is the second of four modules building toward the MANIP-WM capstone:

| Module | System | Contribution to Capstone |
|---|---|---|
| PM1 (IRS) — completed | The Planner | Symbolic task planning: NL → PDDL → action sequence |
| **PM2 (PRS) — this proposal** | **The Eyes** | **Perception: RGB-D → fused 6-DoF poses → scene graph** |
| PM3 (Language) | — | GR00T N1.6 fine-tuning, language interface |
| PM4 (Capstone) | MANIP-WM | RSSM world model over the object-centric state produced here |

The scene-graph output schema defined in this project is the state representation the PM4 world model will operate on. Interface stability is therefore a first-class design requirement, not an afterthought.

### 2.3 Why This Technology Stack?

- **FoundationPose** (NVIDIA, CVPR 2024) is a state-of-the-art model-based 6-DoF pose estimator that generalises to novel objects given a CAD model, avoiding per-object training.
- **YOLOv8** provides a mature, fine-tunable real-time detector to gate FoundationPose — pose estimation is expensive, so detection-first cropping keeps the pipeline real-time.
- **Isaac Sim / Omniverse Replicator** generates photorealistic, automatically labelled synthetic training data with domain randomisation — no manual annotation.
- All components run locally on owned hardware; zero recurring licensing cost.

---

## 3. Literature Review

### 3.1 Object Detection — YOLOv8 with RGB-D Early Fusion

The YOLO family performs single-stage detection with a CNN backbone, predicting boxes and classes in one forward pass. YOLOv8 (Ultralytics, 2023) adds an anchor-free decoupled head and improved feature aggregation.

This project extends the standard RGB detector to a **multimodal RGB-D network** via early fusion: the input stem is widened from 3 to 4 channels (R, G, B, aligned metric depth), with the COCO-pretrained stem weights inflated to initialise the depth channel. Depth is a strong complementary cue for the target domain — textureless objects against a same-coloured tabletop are poorly separated by appearance alone but trivially separated by geometry, and it is precisely this failure mode that motivated the choice of active-depth sensing over passive stereo (§2.3).

Early fusion is chosen over late fusion (separate RGB and depth branches merged at the neck) on the grounds of parameter efficiency and pretrained-weight reuse: the entire COCO backbone remains usable, whereas a second branch would need training from scratch on synthetic data alone. The RGB-only versus RGB-D comparison is retained as an ablation (§7) rather than assumed.

Implementation notes: depth is normalised in metric metres clipped to the working range; invalid-depth pixels (RealSense returns 0 for stereo holes) are encoded explicitly rather than left as zeros, which the network would otherwise interpret as extremely close geometry.

### 3.2 6-DoF Pose Estimation — FoundationPose

FoundationPose (Wen et al., NVIDIA 2024) unifies model-based pose estimation and tracking using a transformer architecture trained at scale on synthetic data. Given an RGB-D frame and an object mesh, it renders pose hypotheses, scores them with a learned comparator, and refines iteratively. Its key property for this project is novel-object generalisation: new objects require only a CAD model, not retraining.

### 3.3 Multi-View / Multi-Sensor Fusion

Fusing estimates from multiple sensors is classical estimation theory: independent measurements of the same latent state combined by uncertainty weighting (Kalman-filter formulation for Gaussian errors). For 6-DoF poses, rotation averaging requires care (quaternion averaging via the eigendecomposition method of Markley et al., 2007). Multi-view object-level fusion — as opposed to pixel-level stereo — resolves occlusion because at least one viewpoint typically retains line of sight. This project fuses per-camera FoundationPose outputs at the object level with confidence-weighted averaging and temporal filtering.

### 3.4 Scene Graphs for Robotics

Scene graphs represent a scene as objects (nodes) and spatial/semantic relations (edges). In manipulation, relational predicates (on, in, clear) are precisely the state language of PDDL planning. Deriving symbolic predicates from continuous 6-DoF poses via geometric rules is a sense-making step that turns raw pattern recognition output into actionable world knowledge.

### 3.5 Synthetic Data & Domain Randomisation

Tobin et al. (2017) showed that randomising textures, lighting, and camera pose in simulation forces networks to learn transferable features, enabling sim-to-real transfer without real training labels. Omniverse Replicator operationalises this with automatically generated bounding boxes, segmentation masks, and ground-truth poses. The sim-to-real evaluation in this project directly measures how well this transfer holds for the chosen object set.

### 3.6 Related Systems & What This Project Adds

Mature components are deliberately adopted, not reimplemented: NVIDIA Isaac ROS already ships a single-view FoundationPose inference pipeline, and detection, calibration (OpenCV), clustering (PCL), and Kalman filtering are all established tooling. No packaged solution exists, however, for the integration this project targets: object-level fusion of two RGB-D viewpoints into a single consistent world state with disagreement gating, emitting symbolic relational predicates in the exact state language of a downstream PDDL planner. Published multi-view pose fusion work exists as research papers, not deployable components. The contribution posture mirrors PM1: established components at the edges, from-first-principles C++ at the integration core (★ modules, §5.2) — demonstrating technique mastery rather than black-box assembly.

---

## 4. PRS Technique Coverage

The module requires the project to develop, integrate and demonstrate at least three of the four PRS aspects. *The Eyes* covers all four:

| # | PRS Aspect | Implementation in The Eyes |
|---|---|---|
| ① | Supervised / unsupervised learning scenarios | **Supervised:** YOLOv8 fine-tuned on synthetically generated labelled data (detection). **Unsupervised:** Euclidean/DBSCAN clustering of the residual depth point cloud to segment and flag *unknown* objects that have no CAD model — an anomaly path the closed-set detector cannot cover. |
| ② | Machine learning / deep learning techniques | **Multimodal RGB-D CNN detector:** YOLOv8 with a 4-channel early-fusion input stem (RGB + aligned metric depth), transfer-learned from COCO weights with stem inflation. FoundationPose transformer-based pose refinement and scoring, itself an RGB-D multimodal network. Both deployed via ONNX Runtime / TensorRT. |
| ③ | Hybrid / ensemble approach | Object-level multi-view fusion: two independent per-camera detection+pose estimates treated as ensemble members, fused by confidence-weighted pose averaging with outlier rejection (disagreement gating). Hybrid DL + classical estimation: learned per-view poses refined by a classical temporal filter (per-object EKF). |
| ④ | Intelligent sensing / sense-making | Dual RGB-D sensing with extrinsic calibration; C++ scene-graph bridge converting fused poses into relational predicates (on, in, clear, occluded_from(view)); occlusion reasoning across viewpoints. |

> **Note:** The multi-view fusion module and the C++ scene-graph bridge are the primary technical contributions. The fusion module is written from first principles (no off-the-shelf fusion library), demonstrating mastery of the estimation techniques rather than black-box usage.

---

## 5. System Design

### 5.1 Architecture Overview

```{.mermaid width=60%}
flowchart TD
    C1["Camera 1 — Front<BR>RGB-D stream"] --> D1["YOLOv8 Detector<BR>per-view ROIs"]
    C2["Camera 2 — Overhead<BR>RGB-D stream"] --> D2["YOLOv8 Detector<BR>per-view ROIs"]

    D1 --> P1["FoundationPose<BR>6-DoF per view"]
    D2 --> P2["FoundationPose<BR>6-DoF per view"]

    C1 -. residual point cloud .-> U["Unsupervised Clustering<BR>unknown-object segmentation"]

    P1 & P2 --> F["Multi-View Fusion ★<BR>confidence-weighted pose fusion<BR>+ per-object EKF (C++)"]
    U --> F

    F --> S["Scene-Graph Bridge ★<BR>poses → relational predicates<BR>(C++)"]

    S --> O1(["/scene_state topic<BR>scene graph JSON"])
    S --> O2(["PDDL state facts<BR>→ The Planner (PM1)"])
```

★ = primary technical contribution, implemented from first principles in C++.

### 5.2 Component Specifications

| Component | Role | Technology | Implementation Detail |
|---|---|---|---|
| Detector (×2) | Per-view object detection, ROI gating | YOLOv8, RGB-D 4-channel early fusion, fine-tuned | Python training / ONNX Runtime or TensorRT inference. Trained on Replicator synthetic data (RGB + depth + labels rendered together) with domain randomisation. Depth channel initialised by inflating COCO stem weights. |
| Pose Estimator (×2) | Per-view 6-DoF pose from RGB-D + CAD mesh | FoundationPose | PyTorch checkpoints on sm_120 — upstream FoundationPose uses no TensorRT (R1 retired). Runs out-of-process as an inference service behind a byte protocol, keeping all runtime logic in C++17. |
| Unknown-Object Segmenter | Cluster residual depth points not explained by detected objects | Euclidean clustering / DBSCAN (PCL) | C++. Flags unknown_object nodes in the scene graph with centroid + bounding volume. |
| Multi-View Fusion ★ | Fuse per-view poses into single world-frame estimate per object | Custom C++ | Extrinsic transform to world frame; quaternion averaging (Markley eigendecomposition); confidence weighting from FoundationPose scores; disagreement gate rejects outlier views; per-object EKF for temporal smoothing. |
| Scene-Graph Bridge ★ | Convert fused poses into symbolic relational predicates | Custom C++ | Geometric rules over object bounding volumes: on(a,b), in(a,b), clear(a), occluded_from(a, view). Emits scene graph JSON + PDDL init facts. |
| Calibration Suite | Intrinsics + stereo extrinsics + world-frame registration | OpenCV (ChArUco board) | One-time offline; validated by reprojection error report. |

### 5.3 Interface Contract (PM1 / PM4 consumers)

```cpp
// Scene graph output — consumed by The Planner (PM1) and MANIP-WM (PM4)
struct ObjectState {
  std::string id;            // "red_cube_01"
  std::string cls;           // YOLO class or "unknown_object"
  Eigen::Isometry3d pose_w;  // fused 6-DoF pose, world frame
  double confidence;         // fused confidence [0,1]
  std::vector<std::string> relations;  // "on(table)", "clear", ...
  uint32_t visible_views;    // bitmask: which cameras see it
};
```

Interface stability rule: fields may be added, never removed or re-typed, so PM4 can rely on this contract.

### 5.4 Technology Stack

C++17 (fusion, scene graph, drivers) · Python 3.11 (training pipelines) · YOLOv8 · FoundationPose · ONNX Runtime C++ API / TensorRT · PCL · OpenCV · Eigen · ROS2 Jazzy · Isaac Sim + Omniverse Replicator · Ubuntu 24.04 · SQLite (metric logging).

### 5.5 Digital-Twin Reconstruction (stretch deliverable)

Because the scene graph carries `{class, CAD mesh, world-frame 6-DoF pose}` per object, real-to-sim scene reconstruction is nearly free: each detected object is spawned at its fused pose in Isaac Sim, producing a live digital twin of the physical tabletop. Limitations: known-CAD objects only (unknown objects appear as placeholder bounding volumes); poses are snapped-to-contact or spawned kinematic to avoid physics conflicts from residual pose error; the static environment (table) is pre-modelled. Beyond its demonstration value, the twin validates that the scene-graph representation is a sufficient world-state encoding — the property the PM4 world model depends on. Stretch item: cut fourth under schedule pressure (§1.3).

---

## 6. Data Collection & Datasets

### 6.1 Synthetic Training Data (primary)

Omniverse Replicator scenes with domain randomisation over lighting, textures, camera pose, distractor objects, and object placement. Automatic labels: 2D boxes, instance masks, ground-truth 6-DoF poses. Target: ~20k–50k rendered frames across the object set (final count decided by detector validation curves).

### 6.2 Object Models

**Acquired 2026-09-04.** The three MVP classes are `mustard_bottle` (YCB 006, obtained via the FoundationPose demo data — its exact diameter of 0.196463 m agrees with the 0.19646 FoundationPose itself reports, confirming it is the mesh the estimator was validated against), `bowl` (YCB 024, `google_16k`), and `red_cube` (authored 50 mm cube: exact by construction, so the 3D-printed part is what must match the mesh rather than the reverse). All are watertight, as FoundationPose requires.

**Object-frame convention.** A scanned mesh's origin is wherever the scanner left it, and FoundationPose reports the pose of that origin — so `ObjectState::pose_w.translation()` inherits it. Two downstream assumptions require it to be the object's *centre* instead: the scene-graph bounding box is built as `translation() ± extent`, and the `symmetric` flag asserts that rotations about the symmetry axis are equivalent. The raw YCB bowl fails both — its origin lies 43.7 mm off centre, 27% of its own diameter, and off the symmetry axis (rim radius sd 30.5 mm about the mesh-origin axis versus 2.5 mm about the AABB-centre axis). The project therefore defines **the object frame's origin as the mesh AABB centre** and re-centres on ingest, which is a rigid re-parameterisation leaving diameters, extents, ADD/ADD-S and pose accuracy unchanged. `training/prepare_meshes.py` enforces this and two automated tests re-derive it from the mesh files, so `config/meshes.yaml` cannot drift from the geometry it describes.

**Physical instantiation: printed where geometry matters, bought where appearance does.** A mesh is only useful if the physical object it describes is the one in front of the cameras — FoundationPose registers the mesh against observed depth, and ADD/ADD-S is computed over mesh vertices, so any shape mismatch between object and mesh is added to the pose error and cannot afterwards be separated from it. The three classes resolve differently:

| class | sourcing | reasoning |
|---|---|---|
| `red_cube` | 3D printed | Authored mesh; the printed part matches by construction |
| `bowl` | 3D printed | The YCB mesh is watertight and fully manifold (0 boundary edges, 0 non-manifold edges) and slices without repair, so printing reproduces the mesh geometry exactly. A visually similar purchased bowl would not, and the difference would surface as irreducible ADD-S error. Printing additionally avoids the original's material: the YCB bowl is glossy enamelware, and specular surfaces return little usable IR to an active-stereo sensor |
| `mustard_bottle` | Purchased — French's Classic Yellow, **20 oz** (see below) | The printed label is the visual signature the RGB half of the detector learns; a blank printed bottle discards it. YCB selects commodity items precisely so they can be re-acquired |

**Consequence for the synthetic dataset.** Replicator renders training data from each mesh's texture, and for a printed object that texture — a scan of the original YCB item — does not describe the object that will actually be captured. For the bowl the mismatch is twofold: the YCB original is glossy speckled enamelware, and its scanned texture has the scan's own specular highlights baked into it, which a renderer would then light a second time. Rather than finishing prints to match a scan, the synthetic renders for printed classes use **the material the object is printed in**, with domain randomisation covering the residual. Both sides of the sim-to-real comparison are under the project's control for these classes, so they are made to agree at the source. The purchased mustard bottle keeps its scanned texture, since there the scan is a record of the real object.

**Identifying the correct product is not incidental.** The YCB object is commonly assumed to be the 14 oz bottle; it is not. Reading the texture atlas, the front label carries "40% MORE THAN OUR 14 OZ", which identifies the **20 oz** bottle, and the YCB paper's recorded mass of 603 g agrees — 20 oz is 567 g of product plus bottle, whereas 14 oz would total roughly 430 g. The mesh's 191.4 mm height is consistent with the larger bottle. Buying the 14 oz would reintroduce precisely the object/mesh mismatch this section exists to prevent, and would do so invisibly: the detector would still fire, FoundationPose would still return a pose, and only the ADD-S figure would silently carry the difference.

The label design must also be checked against the current retail product. The scanned texture shows a "Best By SEP 2016" stamp and a 2014 copyright notice, so it predates this work by roughly a decade and the packaging may since have been revised. Any divergence is an RGB-channel sim-to-real gap (R3), affecting the RGB-only ablation baseline more than the RGB-D detector, whose depth channel is label-invariant.

This divides the object set usefully for §7's modality ablation: two *controlled-appearance* classes whose RGB statistics are matched by construction, and one *real-world-texture* class where they are not.

Printed objects must be sliced at 100% with no bed-fitting rescale, and the printer's dimensional accuracy verified with callipers against a test print before any object is relied upon — a printer 1% out of true reintroduces exactly the mesh/object mismatch that printing is chosen to avoid. Matte mid-tone filament is required: gloss defeats the IR projector and dark surfaces absorb it (§7A, sensor error budget). Layer lines are beneficial rather than harmful, as they add surface texture for active stereo.

### 6.3 Real Evaluation Data

Manually captured dual-camera RGB-D sequences of tabletop scenes, including staged occlusion cases. Ground-truth poses obtained via fiducial (ArUco) mounts; the fiducials are used only for evaluation, never at inference time.

**Fiducial mounts are designed into the printed objects rather than attached to them.** A marker glued to a finished object gives a ground-truth pose only as good as the subsequent measurement of the offset between marker and object frame — an error that enters every ADD/ADD-S figure as a constant bias, and one that is invisible because it leaves all results self-consistent. For the printed classes the marker seat is instead modelled into the part at a known offset from the object frame origin, which §6.2 defines as the mesh AABB centre. The offset is then exact by construction rather than measured, and the ground truth is as accurate as the print.

The purchased mustard bottle admits no such mount and requires a measured attachment; its ground-truth uncertainty is correspondingly higher and is reported separately rather than pooled with the printed classes.

Distractor objects used for clutter and false-positive testing require no meshes, as they are never detected classes.

### 6.4 Benchmark Scenes

A fixed suite of ~10 evaluation scenes graduated by difficulty: isolated objects → clutter → partial occlusion → full single-view occlusion (object visible only from the second camera). The last category is the direct test of the multi-view fusion claim.

---

## 7. Evaluation Plan

| Component | Metric | Target (provisional) |
|---|---|---|
| Detection | mAP@0.5, per-class recall | mAP@0.5 ≥ 0.90 on synthetic val; report real-capture figure |
| Modality ablation | Detector mAP@0.5: RGB-only vs. RGB-D early fusion | Reported delta; expected gain concentrated on textureless / low-contrast objects |
| Training methodology | Dataset-size ablation: mAP vs. training set size (~5k / 20k / 50k frames) | Reported curve; justifies final dataset size empirically |
| Pose (single view) | ADD / ADD-S, rotation error (deg), translation error (mm) | ADD-S < 10% object diameter on visible objects |
| Fusion gain | ADD-S single-view vs. fused, on occlusion suite | Measurable improvement on occluded scenes (primary result) |
| Sense-making | Predicate accuracy vs. hand-labelled ground truth | ≥ 95% on benchmark suite |
| Runtime | End-to-end latency, FPS | Report on RTX 5070 Ti and Jetson Orin Nano |
| Sim-to-real | Δ(metrics) synthetic vs. real captures | Reported and analysed, no fixed target |
| Sensor error budget | Depth RMS vs. a flat wall at 0.4 / 0.7 / 1.0 m, per camera | **MEASURED** — 2–4 mm at working distance; sets the floor below which a fused-pose error is sensor noise |

Every metric above except the sensor error budget is gated on a trained
detector, and therefore on the synthetic dataset (§7A, *What is not yet
measured*). ADD and ADD-S in particular are thresholded as a fraction of object
diameter, so they additionally require the object meshes — acquired 2026-09-04,
with diameters computed exactly rather than sampled, since a sampled diameter
shifts every ADD-S verdict.

The headline experiment is the **occlusion ablation**: identical scenes evaluated with camera 1 only, camera 2 only, and fused — demonstrating that fusion recovers objects invisible to a single view.

---

## 7A. Implementation Status (as of 2026-09-04)

Reported from actual runs; nothing in this table is projected. Figures marked
*synthetic* come from generated input and are not evaluation results.

| Module | State | Evidence |
|---|---|---|
| `camera` | Built, **verified on hardware** | Both D435i at 30.0 fps each, 848×480 aligned; cross-camera skew max 4.3 ms (budget 33 ms); selection by serial survived four replugs across controllers; depth noise 2–4 mm at working distance |
| `calibration` | Built, mock verified | Recovers known intrinsics to ≤1 px and known extrinsics from synthetic observations; round-trip check demonstrated to catch a 2 cm misregistration that per-camera RMSE cannot see. **Session re-registration added** (below) |
| `detector` | Built, **RGB-D input path complete** | C++/Python inference parity: 32 detections over 12 images, scores within 1e-3, boxes within 2 px. 4-channel sentinel depth encoding implemented in C++ and in the training pipeline; **not yet trained — gated on the Replicator dataset** |
| `pose` | Built, **verified end-to-end** | Drives the real FoundationPose server: 30/30 frames, p50 11.1 ms end to end |
| `fusion` ★ | Built | Occlusion ablation harness produces the headline three-way comparison (*synthetic*) |
| `scenegraph` ★ | Built | Emits PDDL `(:init …)`; contract-tested against fusion |
| `clustering`, `twin` | Not started | Above MVP; documented cut order |
| object meshes | **Acquired** | All three classes present, re-centred, config tied to geometry by test (§6.2) |

**296 automated tests pass** across seven suites, covering all six modules plus
cross-module contract tests, under `-Wall -Wextra -Wpedantic`. Every module runs
in a mock mode with no hardware, and the project is under version control with
its full history.

Headline ablation, through the real module chain on *synthetic* input
(150 frames, 4 objects, 10 mm/3° per-view noise, one object occluded from
`cam_over` on 3 of 4 frames):

| view | ADD-S (mm) | recall @0.1d | missed |
|---|---|---|---|
| cam_front | 9.12 | 0.865 | 0 |
| cam_over | 10.12 | 0.692 | 112 |
| **fused** | **7.59** | **0.915** | **0** |

Fusion gains on both axes — 17% lower ADD-S than the better single view, and it
recovers the instances one view could not see at all. Reproducing this shape on
**real** captures is the project's central claim and remains the outstanding work.

### Measured: rig geometry, derived rather than chosen

Camera placement is computed from the measured colour intrinsics
(`fx ≈ 605 px` at 848×480 → 70.0° × 43.3° FOV) against the depth-noise law
below, not selected by eye: standoff **0.70 m**, elevation **35°**, azimuth
**±40°** (80° separation), placing the cameras 402 mm above the tabletop, 737 mm
apart, at ~2.2 mm depth noise with 981 mm of horizontal coverage against a
600 mm workspace. The workspace is a 1180 × 440 mm surface, which the two
tripods clear by 219 mm.

One consequence is load-bearing for calibration validity. The rig mounts the
cameras on floor tripods and uses a shared desk as the table, so the tabletop
moves relative to the cameras between sessions — invalidating every
`T_world_cam` **silently**, because each fused pose stays perfectly
self-consistent while being wrong by a fixed offset. Only `T_world_cam` is
affected: intrinsics are rigid to the camera body and `T_camA_camB` to the
tripods. `calib_agent` therefore gained a **session re-registration** mode that
re-derives world registration from one board view per camera in ~30 s rather
than a 40-view intrinsic sweep.

What makes that safe rather than dangerous is the accompanying check. If a
*tripod* was bumped instead of the desk, re-registration is invalid and would
otherwise write out plausible-looking garbage. The two cases separate on one
identity: `T_camA_camB = T_world_camA⁻¹ · T_world_camB` is **invariant to the
world frame moving**, so recomputing it and comparing against the stored
calibration distinguishes "the table moved" (recoverable) from "a camera moved"
(not). The tool refuses to write in the second case. On synthetic input both
scenarios show identical 13.04 mm of world-frame motion and are separated only
by the baseline test.

### Measured: sensor error budget

Depth noise against a flat wall, 30 frames per point, matched 28 × 16 cm wall
patch at every distance, firmware 5.17.3.10:

| distance | cam_front | cam_over |
|---|---|---|
| 0.4 m | 0.98 mm (0.24%) | 0.75 mm (0.19%) |
| 0.7 m | 2.69 mm (0.38%) | 2.44 mm (0.35%) |
| 1.0 m | 4.77 mm (0.48%) | 4.27 mm (0.42%) |

Both units follow the expected stereo error law — `RMS ≈ 4.9·z²` mm (cam_front)
and `4.4·z²` mm (cam_over), power-law exponents 1.73 and 1.92 — and agree with
each other to 9–10% at 0.7 and 1.0 m.

**At the distances the cameras are mounted (0.60 m and 0.75–0.90 m), depth noise
is 2–4 mm.** This is the floor for fused-pose accuracy: a fused error at or below
it is sensor noise rather than an algorithm deficiency, and the occlusion
ablation must be read against it. It is also the target fidelity for the depth
channel rendered by Replicator, on which the RGB-D detector depends.

**Calibration target sizing is a resolution constraint, not a size one.** The
ChArUco board (324 × 324 mm, 8 × 8 squares at 38 mm) carries DICT_5X5 markers,
7 modules across, which stop decoding below ~3 px/module. Against the measured
factory colour intrinsics the reach is `fx · marker_length / (3 · modules)`, and
the relevant distance is not the 0.70 m standoff but the **oblique far corner of
the board on the table, 884 mm** away at 35° elevation:

| capture resolution | fx (measured) | px/module | reach |
|---|---|---|---|
| 848 × 480 (streaming) | 603.43 | 2.8 — fails | 0.82 m |
| 1280 × 720 | 905.15 | 4.2 | 1.23 m |
| **1920 × 1080** | **1357.72** | **6.3** | **1.84 m** |

Calibration is therefore captured at 1920 × 1080 while the pipeline streams
848 × 480. This is not a workaround for an undersized board: the recovered
camera *pose* is resolution-independent, so a small, inexpensive target is
sufficient provided the capture resolution is chosen deliberately. The tool
reports the computed reach before running detection, because a board below the
decode threshold returns zero corners from images that look entirely correct.

**Methodological note.** Depth RMS may only be compared across distances if the
*physical* wall patch is held constant, not the pixel ROI. A fixed pixel ROI
spans 28 × 16 cm at 0.4 m but 70 × 40 cm at 1.0 m; once it outgrows the flat
surface, edges enter the plane fit and inflate the residual. A first pass at
fixed pixel ROI over-reported by up to **9.7×** (41.30 mm vs. 4.27 mm for
cam_over at 1.0 m) and appeared to show one camera three times noisier than the
other — an artefact that disappeared entirely once the patch was matched. The
diagnostic is that the fitted plane's offset and the median point depth agree to
within 1 mm when the ROI lies on the wall, and diverge by 20–27 mm when it does
not.

### What is not yet measured

Real-capture detection mAP, the modality and dataset-size ablations, ADD-S on
real captures, and predicate accuracy — every one of which sits downstream of a
trained detector, and therefore of the synthetic dataset.

The gating items have narrowed to one. The inter-camera sync cable arrived and
was verified on 2026-08-27. Isaac Sim 5.1.0 is installed and the Replicator
script has been run and debugged. Object meshes were acquired on 2026-09-04. The
ChArUco board is due ~2026-09-05, which unblocks real extrinsics.

**The remaining blocker is Replicator scene fidelity**, not tooling: the scene
currently renders the table at roughly 1 m against a configured 0.6 m and yields
only 4–15% depth validity, and the scene camera intrinsics are not yet set to
the D435i's measured values. Until that is corrected the synthetic depth channel
does not resemble the sensor it is meant to stand in for, which would make the
depth channel itself the dominant sim-to-real gap (R3) rather than a mitigation
of it.

---

## 8. Project Timeline

| Phase | Period | Milestones |
|---|---|---|
| P0 — Environment | now – mid Jul 2026 | **DONE.** Ubuntu 24.04; FoundationPose verified on sm_120, R1 retired |
| P1 — Perception baseline | mid Jul – early Aug | **DONE.** FoundationPose end-to-end via the C++ pose_agent; detector C++/Python parity verified |
| P2 — Fusion + hardware | Aug | **Done bar calibration.** Fusion, scene graph, dual-camera bring-up and hardware sync complete; rig geometry derived from measured intrinsics; calibration awaits the ChArUco board (~05 Sep) |
| P3 — Dataset + evaluation | late Aug – Sep | **In progress, still the critical path.** Isaac Sim 5.1.0 installed and the Replicator script debugged; object meshes acquired 04 Sep. Remaining work is scene fidelity — framing and depth validity — which gates every reportable detection figure |
| P4 — Proposal | by **15 Sep 2026** | Proposal submitted (this document, finalised) |
| — First Presentation | **30 Sep 2026**, 6:30–10:30 pm, Zoom | Goals, data resources, techniques, progress |
| P5 — Hardening + integration | Oct | Full evaluation, PM1 integration demo, unknown-object path, Jetson test |
| P6 — Final deliverables | by **31 Oct 2026** | Runnable system, datasets, final report, 10–15 min video, both slide decks, individual report, peer review |

---

## 9. Budget & Resource Requirements

| Item | Cost | Status |
|---|---|---|
| 2 × Intel RealSense D435i | [VERIFY actual — from receipts] | **Acquired.** Firmware levelled to 5.17.3.10; factory calibration tables backed up |
| Inter-camera hardware sync cable | [VERIFY actual] | **Acquired**, installed and verified 2026-08-27 |
| 2 × USB 3.2 Gen 1 cables, 2 m | [VERIFY actual] | **Acquired** |
| 2 × tripods with heads | [VERIFY actual] | **Acquired** |
| Digital angle gauge | [VERIFY actual] | **Acquired** |
| ChArUco board, 324 × 324 mm, 8 × 8 @ 38 mm | [VERIFY actual] | **Ordered**, due ~2026-09-05 |
| Work surface | — | **No purchase.** Existing desk reused; 1180 × 440 mm of free surface is sufficient for the derived rig geometry (§7A) |
| Filament for printed objects (`red_cube`, `bowl`, fiducial mounts) | < S$50 | To produce. Matte mid-tone; see §6.2 |
| `mustard_bottle` physical object (French's Classic Yellow, **20 oz** — not 14 oz, see §6.2) | < S$10 | To purchase |
| 3D printer | — | **Owned.** Makes the printed-vs-purchased split in §6.2 available at negligible cost, and is what allows fiducial mounts to be exact by construction (§6.3) |
| Object meshes (YCB 006, 024) | — | **Free**, public research dataset |
| Cloud compute (detector training bursts) | ≤ US$100/month existing budget | Available |
| Workstation (Ryzen 9700X, RTX 5070 Ti 16 GB) | — | Owned |
| Jetson Orin Nano Super | — | Owned |
| SO-ARM101 (stretch demo only) | — | Planned purchase, independent of this module |

**Software licensing — verified 2026-09-04 against primary sources.** All
components are free for this use. Ultralytics YOLOv8 is licensed **AGPL-3.0**,
and Ultralytics' own licensing guidance names "academic research and university
coursework" as appropriate for AGPL-3.0, with the Enterprise licence reserved
for proprietary or commercial products. No Enterprise licence is therefore
required here. The AGPL obligation is *disclosure of complete corresponding
source when the work is conveyed*, which a coursework submission satisfies; it
places no requirement to publish, so the private working repository is not in
conflict. FoundationPose is used under the NVIDIA Source Code License
(non-commercial research), and the YCB object and model set is published for
open research use.

---

## 10. Risks & Mitigations

| ID | Risk | Likelihood | Mitigation |
|---|---|---|---|
| R1 | FoundationPose TensorRT engines incompatible with Blackwell (sm_120) | **RETIRED (2026-08-20)** | Verified on hardware, and the premise was wrong: upstream FoundationPose ships no TensorRT at all — it loads PyTorch checkpoints. `run_demo.py` completes 737/737 frames on the RTX 5070 Ti with valid, continuous poses. TensorRT 11.2 separately builds fp32 and fp16 engines for sm_120, so the accelerated route remains available but is not required. |
| R2 | Camera procurement delay | **RETIRED (2026-08-20)** | Both D435i units in hand, firmware levelled to 5.17.3.10, streaming 848×480 aligned depth + colour simultaneously at 30 fps each with cross-camera timestamp skew ≤ 4.3 ms against a 33 ms budget. |
| R3 | Sim-to-real gap larger than expected | Medium | Domain randomisation breadth; small real fine-tuning set for the detector; gap itself is a reportable finding, not a failure. |
| R4 | FoundationPose too slow for real-time dual-view | **Downgraded (measured)** | 17.2 ms/frame tracking (58 FPS) for one object on the RTX 5070 Ti → ~29 FPS per camera for two views run serially. Scales roughly linearly in tracked objects (3 objects × 2 views ≈ 9.7 FPS). Detection-gated ROIs and alternate-frame scheduling remain worthwhile but are no longer mandatory. |
| R4b | Active-IR cross-interference: each RealSense sees the other's projected IR pattern, degrading both depth maps | Medium — **mitigation installed, effect not yet measured** | Hardware sync is in place: the cable is fitted and the master's trigger confirmed reaching the slave, which now streams at 30 fps where it previously delivered no frames at all. `inter_cam_sync_mode` must be written through the device context *before* `pipeline.start()`; librealsense rejects it silently on a streaming device, which cost a false negative. The interference itself remains unquantified because it requires both cameras aimed into the same workspace — measurable only once the rig is physically mounted. The 80° angular separation of the derived geometry (§7A) reduces pattern overlap independently of sync. |
| R5 | Solo workload | Accepted | Same mitigation pattern as PM1: AI-agent-assisted development (disclosed, Appendix A); strict phase gates; scope cut order pre-defined in §1.3 (MVP is protected; fusion ablation is never cut). |

---

## 11. Deliverables & Assessment Alignment

### 11.1 Final Deliverables (due 31 Oct 2026, Canvas, single ZIP + individual report)

- Runnable perception system (source + build instructions + model files)
- Datasets (synthetic set specification + real capture set)
- Final report: tools/techniques, system design/models, system performance, findings and discussion
- Video presentation (10–15 min, .mp4)
- Slides of both presentations
- Individual project report (1–2 pages) and peer review submission

### 11.2 Assessment Alignment

| Component | Weight | How this project addresses it |
|---|---|---|
| First Presentation | 5% | Goals, data resources, techniques, progress at end of P3 — concrete baseline metrics already in hand |
| Final Presentation | 10% | Occlusion ablation as the demo centrepiece; live PM1 integration if stable |
| Final Report | 15% | Evaluation plan (§7) maps directly onto required report sections |
| Final System | 15% | Fully runnable, self-contained, documented; reproducible from ZIP |
| Peer Review | 5% | Solo submission — 5% deduction accepted (established PM1 precedent) |

---

## 12. Conclusion

*The Eyes* closes the perception gap left deliberately open in PM1. It is scoped to be individually achievable, demonstrates all four PRS technique categories with two from-first-principles C++ contributions (multi-view fusion, scene-graph bridge), produces a quantitative headline result (occlusion ablation), and delivers the exact object-centric state representation the MANIP-WM capstone world model requires. Every module in the four-module programme remains load-bearing.

---

## Appendix A — Solo Submission & AI Usage Disclosure

### A.1 Solo Submission Declaration

This project is undertaken as a solo submission under Group 12, AIS08, with the associated peer-review deduction understood and accepted. Rationale: full project control and architectural continuity across the four-module capstone programme.

### A.2 AI Agents as Development Collaborators

**Development-time AI (not present at runtime):** Claude (architecture review, code generation under supervision, documentation drafting), Claude Code (orchestrated coding subagents per AGENTS.md conventions established in PM1). All AI-generated code is reviewed, tested, and understood by the author before inclusion. All externally sourced factual claims in this proposal marked [VERIFY] are checked against primary sources before final submission.

**Runtime AI (embedded in the submitted system):** YOLOv8 (fine-tuned by the author), FoundationPose (pretrained, NVIDIA). No LLM is present in the runtime perception pipeline.

### A.3 Academic Integrity Statement

Use of AI tools follows the NUS-ISS Policy for Use of AI in Teaching and Learning (2024). All AI assistance is disclosed above; the system architecture, the two core C++ contributions, the evaluation design, and all analysis are the author's own work.

---

## Appendix B — Technique-to-Course-Module Mapping

| Course Module | Techniques Applied in The Eyes |
|---|---|
| Problem Solving Using Pattern Recognition (PSUPR) | Pattern recognition problem framing (closed-set detection + open-set anomaly path), supervised vs. unsupervised scenario design, evaluation methodology (mAP, PR curves, ADD/ADD-S) |
| Pattern Recognition and Machine Learning Systems (PRMLS) | Deep learning for vision: CNN detector (YOLOv8 transfer learning), transformer-based pose estimation (FoundationPose), ensemble fusion of independent estimators, ONNX Runtime / TensorRT deployment |
| Intelligent Sensing and Sense Making (ISSM) | Dual RGB-D sensing and extrinsic calibration, point-cloud clustering (DBSCAN), multi-sensor fusion with EKF temporal filtering, scene-graph relational predicate extraction (sense-making) |
