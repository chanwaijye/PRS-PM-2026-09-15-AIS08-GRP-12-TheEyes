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

    S --> O1(["scene_state<BR>scene graph JSON"])
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

C++17 (fusion, scene graph, drivers) · Python 3.11 (training pipelines) · YOLOv8 · FoundationPose · ONNX Runtime C++ API / TensorRT · PCL · OpenCV · Eigen · Isaac Sim + Omniverse Replicator · Ubuntu 24.04 · SQLite (metric logging).

**Runtime process model: one C++ process, not ROS2** (decided 2026-09-06 when
the live runner was built). The stages above are function calls passing the
`INTERFACES.md` structs in-process, not nodes exchanging messages. ROS2 Jazzy
was the original plan and is not used: for a fixed six-stage pipeline on one
machine it buys nothing over direct calls and costs serialisation on the hot
path, a second build system, and a distro pin that conflicts with PM1's Humble.
The message-shaped contract survives the change unaltered — `INTERFACES.md`
still specifies `scene_state` and the PDDL facts as the boundary, so a ROS2
publisher is a thin adapter over the same structs if PM1 or PM4 later wants
one. This keeps the PM1 integration a contract, not a shared workspace, which
is what §2.2 required of it.

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
| `mustard_bottle` | 3D printed — see below | Purchase was preferred, because the printed label is the visual signature the RGB half of the detector learns. The YCB original is French's Classic Yellow **20 oz**, which is not retail-available in Singapore, so the class joins the printed group |

**Consequence for the synthetic dataset.** Replicator renders training data from each mesh's texture, and for a printed object that texture — a scan of the original YCB item — does not describe the object that will actually be captured. For the bowl the mismatch is twofold: the YCB original is glossy speckled enamelware, and its scanned texture has the scan's own specular highlights baked into it, which a renderer would then light a second time. Rather than finishing prints to match a scan, the synthetic renders for printed classes use **the material the object is printed in**, with domain randomisation covering the residual. Both sides of the sim-to-real comparison are under the project's control for these classes, so they are made to agree at the source. This now applies to **all three** classes without exception: an earlier revision kept the mustard bottle on its scanned texture on the grounds that the scan recorded a real purchased object, but §6.2 resolves that class to the printed group, so its scanned label likewise describes an object that will never be photographed. `replicator_dataset.py` renders all three in filament accordingly — yellow for the bottle, as the real product is — and the generated dataset reflects that.

**Identifying the correct product is not incidental.** The YCB object is commonly assumed to be the 14 oz bottle; it is not. Reading the texture atlas, the front label carries "40% MORE THAN OUR 14 OZ", which identifies the **20 oz** bottle, and the YCB paper's recorded mass of 603 g agrees — 20 oz is 567 g of product plus bottle, whereas 14 oz would total roughly 430 g. The mesh's 191.4 mm height is consistent with the larger bottle. Buying the 14 oz would reintroduce precisely the object/mesh mismatch this section exists to prevent, and would do so invisibly: the detector would still fire, FoundationPose would still return a pose, and only the ADD-S figure would silently carry the difference.

The label design must also be checked against the current retail product. The scanned texture shows a "Best By SEP 2016" stamp and a 2014 copyright notice, so it predates this work by roughly a decade and the packaging may since have been revised. Any divergence is an RGB-channel sim-to-real gap (R3), affecting the RGB-only ablation baseline more than the RGB-D detector, whose depth channel is label-invariant.

**All three classes are consequently controlled-appearance**, which must be stated rather than glossed, because it changes what the sim-to-real figure measures. The YCB mustard bottle is French's Classic Yellow 20 oz — identified from the mesh's own texture atlas, whose front label reads "40% MORE THAN OUR 14 OZ", and corroborated by the YCB paper's recorded 603 g — and it is not retail-available in Singapore. Substituting a visually similar bottle would reintroduce exactly the object/mesh mismatch this section exists to prevent, so the class is printed like the others and rendered in filament.

The consequence: the appearance gap between synthetic and real is minimised by construction for every class, so the measured sim-to-real difference isolates geometry, sensor and lighting effects rather than testing appearance-domain randomisation. That is a cleaner experiment in one sense and a weaker test of G4 in another, and §7 reports it as such rather than presenting an artificially small gap as a result. Should the object become obtainable, restoring it as a purchased real-texture class recovers the stronger test at no cost to anything else.

**The print colours are determined by the dataset, not chosen at print time.** `replicator_dataset.py` renders each class in a fixed base colour with `FILAMENT_SPREAD = 0.12` of linear randomisation around it, and the shipped `synth50k_v2` set — and therefore the trained detector — has already learned those distributions. A filament that falls outside the rendered band is an appearance gap introduced *after* training, which is precisely what §6.2 exists to prevent. The band is wide enough that an exact match is unnecessary and narrow enough that it must be checked:

| class | linear diffuse (as rendered) | sRGB target | acceptable band |
|---|---|---|---|
| `mustard_bottle` | (0.85, 0.72, 0.10) | `#EDDD59` | `#DECB00` – `#FCEC81` |
| `red_cube` | (0.70, 0.09, 0.09) | `#DA5555` | `#C80000` – `#EA7E7E` |
| `bowl` | (0.12, 0.32, 0.68) | `#6199D7` | `#007CC5` – `#86B1E7` |

Matte is a requirement rather than a preference, and it is the same requirement in both directions: gloss defeats the IR projector on the real object (§7A sensor error budget), and the renderer draws these classes at roughness 0.55–0.90, so a glossy print would depict something the training set does not contain.

**Wall thickness is dictated by the mesh, not by print strength.** The YCB bowl's scanned wall measures 0.99 mm, thinner than two perimeters at a 0.4 mm nozzle, so it prints as a solid thin wall and its infill setting has no effect. It must not be thickened for durability: ADD-S is computed over mesh vertices, so any deviation from the mesh enters the pose error and cannot afterwards be separated from estimator error. Material requirement per full object set, computed from the meshes: **227 g** (`mustard_bottle` 137 g at 1.2 mm wall and 10% infill, `red_cube` 49 g at 20% infill, `bowl` 41 g).

Printed objects must be sliced at 100% with no bed-fitting rescale, and the printer's dimensional accuracy verified with callipers against a test print before any object is relied upon — a printer 1% out of true reintroduces exactly the mesh/object mismatch that printing is chosen to avoid. Matte mid-tone filament is required: gloss defeats the IR projector and dark surfaces absorb it (§7A, sensor error budget). Layer lines are beneficial rather than harmful, as they add surface texture for active stereo.

### 6.3 Real Evaluation Data

Manually captured dual-camera RGB-D sequences of tabletop scenes, including staged occlusion cases. Ground-truth poses obtained via fiducial (ArUco) mounts; the fiducials are used only for evaluation, never at inference time.

**Fiducial mounts are derived from the object meshes rather than attached to the objects.** A marker glued to a finished object gives a ground-truth pose only as good as the subsequent measurement of the offset between marker and object frame — an error that enters every ADD/ADD-S figure as a constant bias, and one that is invisible because it leaves all results self-consistent. The offset must therefore come from CAD, not from callipers.

An earlier revision of this section put the marker seat *inside* the printed part. Measurement on the actual meshes rules that out for two of the three classes. `mustard_bottle` has no flat region anywhere — its front face carries **7.5 mm of sagitta over a 24 mm patch**, and its only truly flat area is the cap top at **17.7 mm diameter**, below the ArUco decode floor at the 0.7 m standoff. `bowl` has a scanned wall of **0.99 mm**, so a flush recess has no surviving material behind it. Only `red_cube`, with six flat 50 mm faces, admits the original design.

The decisive objection is not printability, though — it is the dataset. `synth50k_v2` was rendered from the **unmodified** meshes, so cutting a facet or adding a boss to a target object introduces a sim-to-real gap the trained detector has never seen, and closing it means regenerating 88 GB for a change that serves evaluation rather than perception.

**The marker therefore moves off the object and onto a tray the object sits in.** `training/author_marker_trays.py` generates one tray per class: a plate whose socket is **boolean-subtracted from that object's own mesh**, so the fit is derived from the geometry rather than measured, plus recessed seats for 40 mm `DICT_5X5_250` markers in the same plate. The ground-truth chain is

`T_cam_object = T_cam_marker · T_marker_tray · T_tray_object`

where only `T_cam_marker` is observed; the other two are chosen in CAD and written to `config/marker_trays.yaml`. Every class keeps the exact-by-construction property the original design was chosen for, and the objects themselves are untouched — so `assets/meshes/*.obj` remain the meshes FoundationPose registers against, and `config/meshes.yaml`'s `diameter_m`, `extent` and `symmetric` flags remain true statements about the physical objects.

The generator verifies what it produces rather than asserting it:

| | `red_cube` | `mustard_bottle` | `bowl` |
|---|---|---|---|
| plate (mm) | 170 × 90 × 10.5 | 216 × 100 × 14.5 | 210 × 210 × 8.5 |
| socket depth | 8.0 mm | 12.0 mm | 6.0 mm |
| **measured socket clearance** | 0.20 mm | 0.12–0.23 mm | 0.08–0.12 mm |
| card-to-socket gap | 5.8 mm | 5.5 mm | 10.0 mm |
| markers visible per camera | 1 of 2 | 1 of 2 | 3 of 4 |
| marker ids | 100–101 | 102–103 | 104–107 |

**Socket clearance is the ground-truth error budget**, and it is the reason the figure is measured rather than nominal: the object can sit anywhere within it, so the worst case — **0.23 mm** — bounds the ground-truth translation error. Against the bottle's 19.6 mm ADD-S threshold that is 1.2%, so it cannot move a verdict. Marker ids are allocated globally (the ChArUco board holds 0–31), because per-tray numbering would make id 100 ambiguous the moment two objects share a scene, which is every scene in the occlusion ablation.

Marker visibility is ray-cast against the seated object at the rig's actual 35° elevation and ±40° azimuth rather than argued: each camera sees at least one marker on every tray. Because the two cameras generally read *different* markers on the wing trays, agreement between their independently derived object poses is a free check on the print.

Distractor objects used for clutter and false-positive testing require no meshes, as they are never detected classes. The trays are likewise not a detected class, so they are printed in one of the three **distractor neutrals** the renderer already samples (§6.2) rather than a target colour — a tray in a class colour would put a large slab of that class's colour directly beneath the object.

An earlier revision of this section carved out the mustard bottle as a purchased object admitting no such mount, requiring a measured attachment, and therefore carrying higher ground-truth uncertainty reported separately from the printed classes. That carve-out no longer applies: §6.2 resolves the bottle to the printed group, because the YCB original is the 20 oz French's Classic Yellow and it is not retail-available in Singapore. It takes a marker tray like the other two, so **all three classes have ground truth that is exact by construction** and none is reported separately. The change removes a limitation rather than adding one — it was forced by sourcing, but its effect on evaluation is strictly positive.


### 6.4 Benchmark Scenes

A fixed suite of ~10 evaluation scenes graduated by difficulty: isolated objects → clutter → partial occlusion → full single-view occlusion (object visible only from the second camera). The last category is the direct test of the multi-view fusion claim.

---

## 7. Evaluation Plan

| Component | Metric | Target (provisional) |
|---|---|---|
| Detection | **mAP@0.5:0.95** and per-class, plus recall | Synthetic val **met**: 0.956 pooled (§7A). mAP@0.5 is retired as a target — it reached 0.994 and has 0.006 of headroom, so it can no longer discriminate. Real-capture figure outstanding |
| Modality ablation | **mAP@0.5:0.95**, per class: RGB-only vs. RGB-D early fusion | **Run — negative on synthetic** (+0.00076 pooled, §7A). Re-run on real captures, where depth validity is 97.8% rather than 100%; `red_cube` is the discriminating class |
| Training methodology | Dataset-size ablation: mAP@0.5:0.95 vs. training set size (5k / 20k / 40k train frames) | **Run** (§7A): recall saturates at 20k, localisation does not |
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

## 7A. Implementation Status (as of 2026-09-06)

Reported from actual runs; nothing in this table is projected. Figures marked
*synthetic* come from generated input and are not evaluation results.

| Module | State | Evidence |
|---|---|---|
| `camera` | Built, **verified on hardware** | Both D435i at 30.0 fps each, 848×480 aligned; hardware-synced pair jitter **0.14–1.1 ms sigma** against a 33 ms budget; selection by serial survived four replugs across controllers; depth noise 2–4 mm at working distance |
| `calibration` | Built, **run on the real rig 2026-09-06** | Real extrinsics committed to `config/extrinsics.yaml`: reprojection RMSE **0.39 px** (cam_front) / **0.33 px** (cam_over). Round-trip check catches a 2 cm misregistration that per-camera RMSE cannot see. **Session re-registration added** (below) |
| `detector` | Built, **trained and deployed** | Trained on `synth50k_v2` (40 epochs), exported to ONNX, C++/Python parity re-verified against the trained network. On the GPU: **p50 3.60 / 3.43 ms per view** (12.60 / 12.37 on CPU), same detections either provider. Modality ablation run — negative result, below |
| `pose` | Built, **verified end-to-end** | Drives the real FoundationPose server: 30/30 frames, p50 11.1 ms end to end |
| `fusion` ★ | Built, **first real-object run 2026-09-06** | Eight-arm ablation harness (below, *synthetic*). On 150 live pairs of a printed cube both views associated and fused every pair, `visible_views = 0b11`, 0 gate rejections, per-view disagreement 11 mm |
| `scenegraph` ★ | Built | Emits PDDL `(:init …)`; contract-tested against fusion |
| `runtime` | **Built and run on hardware 2026-09-06** | `the_eyes_run check / live / replay`: cameras by serial → RGB-D YOLOv8 → FoundationPose → fusion → scenegraph → PDDL in one C++ process. Live with the real pose server: **p50 51.7 ms per pair**, p95 78 ms. Bit-exact record/replay, SQLite metrics per pair |
| `clustering`, `twin` | Not started | Above MVP; documented cut order |
| object meshes | **Acquired** | All three classes present, re-centred, config tied to geometry by test (§6.2) |

**336 automated tests pass** across eight suites, covering all six modules plus
cross-module contract tests, under `-Wall -Wextra -Wpedantic`. Every module runs
in a mock mode with no hardware, and the project is under version control with
its full history.

Headline ablation, through the real module chain on *synthetic* input
(200 frames, 3 objects, 10 mm/axis Gaussian per-view noise, two objects
occluded from `cam_over` on 3 of 4 frames). Eight arms: which **views** (one
camera or both), which **policy** (naive unweighted mean, or the gated,
confidence-weighted, symmetry-aware fuse) and whether the per-object **EKF** is
applied, each varied on its own:

| arm | views | policy | filter | ADD-S (mm) | ADD-S (/diam) | recall @0.1d | missed |
|---|---|---|---|---|---|---|---|
| cam_front | 1 | — | raw | 9.97 | 0.1151 | 0.3517 | 0 |
| cam_over | 1 | — | raw | 10.37 | 0.1197 | 0.1517 | 300 |
| naive (baseline) | 2 | naive | raw | 9.00 | 0.1039 | 0.4850 | 0 |
| gated | 2 | gated | raw | 9.01 | 0.1040 | 0.4833 | 0 |
| cam_front+ekf | 1 | — | ekf | 6.62 | 0.0765 | 0.8550 | 0 |
| cam_over+ekf | 1 | — | ekf | 7.85 | 0.0906 | 0.3183 | 300 |
| naive+ekf | 2 | naive | ekf | 6.28 | 0.0725 | 0.9017 | 0 |
| **fused** (the system) | 2 | gated | ekf | **6.29** | **0.0726** | **0.9017** | **0** |

**The table is read as pairs of rows that differ in exactly one thing.** The
second camera's contribution is `naive − cam_front` (9.00 vs 9.97 mm, and recall
0.485 vs 0.352). The EKF's contribution is any `+ekf` row against its raw twin
(about 2.7 mm and 0.4 of recall). The fusion **policy's** contribution — the
disagreement gate, confidence weighting and symmetry handling, which is what
this module is graded on — is `gated − naive` at the raw tier and
`fused − naive+ekf` at the EKF tier. On this input both are **0.01 mm**.

That zero is correct, and it is the reason the design has eight rows rather
than four. Under symmetric Gaussian noise with no outliers the gate has nothing
to reject and equal per-view scores make the weighted mean the plain mean, so
the policy *cannot* win; a harness that showed it winning here would be
measuring something else. An earlier revision of this table did exactly that.
It had four arms, defined `naive` as "unweighted mean **and no filter**", and
ran the single-view and fused arms with the EKF on. The reported fused-vs-naive
gap, and the conclusion drawn from it that "a second camera is a liability until
something arbitrates the disagreement", was the EKF's temporal averaging being
credited to the spatial policy — a comparison across rows that differed in two
things at once. With the filter made transparent the two arms agreed to
0.01 mm. The temporal filter is now a switch independent of the policy in
`FusionConfig`, and `Ablation.TheFilterAndThePolicyAreNotConfounded` asserts
that the policy delta is zero and the filter delta is large on this input, so
the confound cannot return unnoticed.

Where the policy does earn its place is a **confidently wrong view**: a bias
rather than noise, which no amount of temporal averaging removes.
`GatedFusionBeatsTheNaiveBaselineWhenOneViewIsAnOutlier` injects an 8 cm error
into one view and asserts that the gated arm beats the naive arm at *both*
filter tiers. Whether FoundationPose produces such views at this rig geometry,
and how often, is precisely what the real-capture ablation will measure; if it
rarely does, the honest result is that the second camera's value on this rig is
recall under occlusion and the EKF's is accuracy, and the gate is insurance.
Reproducing the table on **real** captures is the project's central claim and
remains the outstanding work.

`naive` is the baseline a reader would reach for unprompted: both views, the
same world-frame lift and the same cross-view association, then an unweighted
mean with no gate and no symmetry case. Quaternion averaging is deliberately
shared between the two policies — at the angular separations two calibrated
cameras produce on a static object, Markley averaging and naive
coefficient-averaging agree well inside the per-view noise, so putting it in
the baseline would manufacture a difference rather than measure one.

Two honest qualifications belong with the table. The figures are *synthetic*,
and every tool that prints them says so. And `ApplyDisagreementGate`'s
three-or-more-view majority branch cannot execute on a two-camera rig: it is
covered by a unit test, but the deliverable hardware never reaches it, so the
gate's behaviour on the real rig is entirely the two-view tie-break path.

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
| **1920 × 1080** | **1357.72** | **6.1** | **1.81 m** |

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

Real-capture detection mAP, ADD-S on real captures, and predicate accuracy —
each of which requires the physical objects and a real calibration. The modality
and dataset-size ablations have both run and are reported below — the first as a
negative result.

**No delivery gates anything any more.** The inter-camera sync cable arrived and
was verified on 2026-08-27; the ChArUco board arrived on 2026-08-28 and was
decode-verified against a real capture (ids 0–31 under `DICT_5X5_250`, measured
reach 1.82 m / 1.81 m per unit at 1920 × 1080). Isaac Sim 5.1.0 is installed and
the Replicator script has been run and debugged. Object meshes were acquired on
2026-09-04.

**The rig is calibrated and the whole chain has run on a real object.** As of
2026-09-06 `calib_agent` has been run against the mounted rig and the real
extrinsics are committed (RMSE 0.39 / 0.33 px), and `runtime_agent` drives the
full pipeline live at p50 51.7 ms per pair. On 150 live pairs of a printed cube
with fixed boxes standing in for the untrained-on-real-data detector, the two
views associated and fused into a single object on **every** pair with zero gate
rejections, the fused pose stable to 0.1 mm sd. The one checkable figure is
height — a 50 mm cube centres at 25 mm and the fused z reads 29.7 mm, **5 mm
high**, consistent with 2–4 mm depth noise plus calibration. *This is one
observation, not a measurement, and it is not an ADD-S number.* It is the first
evidence on real data that the multi-view claim holds end to end.

What remains is work, not waiting, and the longest pole is now **the printed
object set with its ArUco trays** (§6.3): the tray geometry is authored and the
cube is printed, but the bottle, bowl and tray set are not, and without them
there is no ground truth and therefore no real ADD-S. The order is print →
calliper-verify → mount → capture → score.

**Replicator scene fidelity was the remaining blocker and is now closed
(2026-09-04).** The scene renders the derived rig geometry with the measured
per-unit camera intrinsics and the real CAD meshes, and its output is verified
by a `--diagnose` mode that reports what was actually rendered rather than what
the scene description implies:

| | before | after |
|---|---|---|
| depth validity | 4–15% | **100.0%** |
| labelled targets per frame | ~0 | **3.00** |
| nearest rendered surface | 1007 mm | **470 mm** (table corner is 458 mm) |
| invalid-depth sentinel | ~85% | **1.5%** |

Two defects account for it, and both had produced output that looked correct.
The first was the **USD default near clipping plane of 1 metre**: the camera
stands 0.70 m from the workspace centre and the nearest table corner is 0.46 m,
so the entire subject sat inside the near plane and was clipped, leaving bright,
plausible, object-free frames of the floor and far wall. The second was that
**the scene contained no room** — at this geometry the vertical field of view
sweeps a 1427 mm band across the table plane while the desk is 440 mm deep, so
69% of every frame hit no geometry and returned the invalid sentinel, where a
real sensor pointed at a desk sees floor and wall and returns finite depth. The
synthetic depth channel would have taught the detector that "sentinel" means
background, a property the real sensor does not have.

### Measured: first detector training, and why mAP@0.5 is the wrong number here

The RGB-D detector has now been trained twice, and the comparison is more
informative than either result alone. On the first synthetic dataset it reached
**mAP@0.5 = 0.994 after a single epoch** and 0.995 by the tenth, against this
proposal's provisional target of 0.90. That is not a result to report; it is a
warning that the benchmark cannot discriminate. Inspection confirmed why: three
classes in three unmistakable filament colours on a plain table, with 98% of
frames showing all three targets essentially unoccluded.

The scenes were rebuilt to be discriminative — distractors drawing from the
*same* colour palette as the targets, so colour ceases to be sufficient and
shape and depth must carry the decision; eight distractors rather than three,
sized across the target range; and a tighter workspace so objects crowd.
Occlusion is now measured rather than assumed, from the ratio of Replicator's
tight (visible) to loose (full) bounding boxes: mean occlusion rose from ~0% to
31–35%, objects more than 25% occluded from ~0% to 54–56%, and frames showing
all three targets fell from 98% to 62%.

Retrained on the harder data, 40 epochs, 40,012 train / 9,988 val:

| class | instances | P | R | mAP@0.5 | mAP@0.5:0.95 |
|---|---|---|---|---|---|
| mustard_bottle | 9,988 | 0.997 | 0.998 | 0.995 | 0.989 |
| bowl | 9,322 | 0.989 | 0.986 | 0.994 | 0.966 |
| **red_cube** | **6,622** | 0.988 | **0.963** | 0.992 | **0.915** |
| **all** | 25,932 | 0.991 | 0.983 | **0.994** | **0.956** |

**The headline finding is that harder scenes did not prevent saturation at
mAP@0.5.** It converged to 0.994 against the easy dataset's 0.995 — a difference
of one thousandth. What changed is everything underneath: mAP@0.5:0.95 fell from
0.985 to 0.956, and convergence slowed from one epoch to roughly eleven.

The consequence for §7 is concrete. **The modality and dataset-size ablations
must be reported on mAP@0.5:0.95 and per class, not on mAP@0.5**, which has
0.006 of headroom and can register nothing. mAP@0.5:0.95 retains 0.044, and
`red_cube` alone retains 0.085 — that class is the discriminating one, being the
smallest object and the most often occluded, visible in only 66% of validation
images against the bottle's 100%. A depth channel that helps should help there
first, which makes it the natural place to look for the RGB-D gain rather than
in a pooled average that is already close to its ceiling.

A third, subtler defect followed from the object-frame convention of §6.2: with
the origin at the mesh AABB centre each class rests at its own half-height (96,
25 and 28 mm), so a single shared placement range buried some classes in the
table and floated others — while the 2D boxes stayed correct throughout, so the
label files looked perfect and `(on obj table)` was false in the very data the
predicates are to be evaluated against.

---

### Measured: the modality ablation, and it is a negative result

This is the ablation that tests the project's own multimodal claim (§4, category
②), so it is reported whichever way it comes out. RGB-only versus RGB-D on the
same 9,988 val frames, byte-identical PNGs and labels, identical 40,012/9,988
split, same 40-epoch schedule and seed, `yolov8n.pt` COCO init both sides —
differing **only** in whether the loader reads the 3-channel PNG or the
4-channel `.npy`, so any delta is attributable to the depth channel and nothing
else:

| | RGB-only | RGB-D | delta |
|---|---|---|---|
| mAP@0.5 | 0.99325 | 0.99371 | +0.00046 |
| **mAP@0.5:0.95** | 0.95662 | 0.95738 | **+0.00076** |
| recall | 0.98272 | 0.98260 | −0.00012 |
| `mustard_bottle` @.5:.95 | 0.98872 | 0.98861 | −0.00011 |
| **`red_cube` @.5:.95** | 0.91565 | 0.91590 | **+0.00026** |
| `bowl` @.5:.95 | 0.96551 | 0.96763 | +0.00212 |

**The depth channel does not earn its place on this benchmark.** `red_cube` was
the class predicted to move — smallest, most often occluded, and the only one
with real headroom at 0.085 below ceiling — and it moved by 0.0003. Two of three
classes are marginally *worse* on recall with depth.

The reading is narrow and deliberate: at 100% synthetic depth validity, three
geometrically distinct classes in three distinct colours, and a COCO-pretrained
backbone, depth adds nothing *this benchmark* needs. That is not a statement
about RGB-D detection in general, and it is not "the benchmark is saturated" —
the dataset-size ablation below rules that out, since 8x more training data
moves the same val split by +0.0322. There is headroom; depth does not reach it
while more RGB examples do.

Two consequences follow rather than being argued away. The early-fusion stem
stays in the delivered system because the real-capture condition is the one it
was chosen for — real depth validity is 97.8%, not 100%, and the appearance gap
is real — and that test has not run. And if it does not move there either, the
honest finding is that the RGB-D stem was the right design for the stated risk
and the risk did not materialise on this object set. `AGENTS.md` set that policy
in advance: "if they do not, that is a reportable negative result, not a failure
to hide."

Raw metrics: `docs/results/modality_rgb_vs_rgbd_50k_v2.json`.

---

### Measured: the dataset-size ablation, and what it says about the modality result

AGENTS.md makes this an explicit deliverable. All three arms share one generator
config, one scene-sampling distribution, one 9,988-frame val split, one seed and
one 40-epoch schedule; only `--fraction` differs. That matters: three separate
generations would have differed in scene sampling as well as in size, and the
comparison could not then attribute a difference to size alone. Arms are named
for the dataset they subset — the actual **train** counts are 5,001 / 20,006 /
40,012, because `synth50k_v2` is 50k *total* (40,012 train + 9,988 val).

| train frames | mAP@0.5 | mAP@0.5:0.95 | recall | `red_cube` @.5:.95 |
|---|---|---|---|---|
| 5,001 | 0.98805 | 0.92520 | 0.97093 | 0.86445 |
| 20,006 | 0.99333 | 0.94695 | 0.98274 | 0.89714 |
| 40,012 | 0.99371 | **0.95738** | 0.98260 | 0.91590 |

The prediction made above — that these ablations must be read on mAP@0.5:0.95 and
per class rather than on mAP@0.5 — is borne out. mAP@0.5 moves 0.0057 across an
8× change in training data and would have supported no conclusion at all.

**Recall saturates at 20k; localisation accuracy does not.** Recall is flat past
20,006 frames (0.98274 → 0.98260) while mAP@0.5:0.95 still gains +0.0104 over the
same interval. The additional data is not finding more objects — it is placing
the boxes it already finds more precisely. `red_cube`, the smallest class and the
one with the most headroom, takes most of the benefit: +0.0514 across the 8×
against +0.0322 pooled. The curve has not flattened, so the full generation is
justified empirically rather than by assertion, and more data would still buy
accuracy — worth knowing before the sim-to-real work, where the temptation under
schedule pressure is to generate less.

**This qualifies the modality result above.** That section attributes depth's
non-effect to RGB having already saturated the task. On the same val split, 8×
more training data buys +0.0322 pooled and +0.0514 on `red_cube`, so the task is
demonstrably not saturated. The defensible statement is narrower than the one
first offered: the remaining headroom is in localisation precision, and an
early-fusion depth channel does not reach it on this benchmark while more RGB
examples do. Both ablations are reported as measured, including where the second
constrains the first.

Raw metrics and plot: `docs/results/dataset_size_ablation_rgbd_50k_v2.json`
and `docs/results/dataset_size_ablation_rgbd_50k_v2.png`.

---

## 8. Project Timeline

| Phase | Period | Milestones |
|---|---|---|
| P0 — Environment | now – mid Jul 2026 | **DONE.** Ubuntu 24.04; FoundationPose verified on sm_120, R1 retired |
| P1 — Perception baseline | mid Jul – early Aug | **DONE.** FoundationPose end-to-end via the C++ pose_agent; detector C++/Python parity verified |
| P2 — Fusion + hardware | Aug | **Done bar calibration.** Fusion, scene graph, dual-camera bring-up and hardware sync complete; rig geometry derived from measured intrinsics; calibration awaits the ChArUco board (~05 Sep) |
| P3 — Dataset + evaluation | late Aug – Sep | **Largely done.** `synth50k_v2` generated (40,012 train / 9,988 val), RGB-D detector trained and exported to ONNX, C++/Python parity re-verified against the trained network, both ablations run — modality (negative result) and dataset-size — see §7A. Outstanding: real-capture evaluation, which needs the printed objects |
| P4 — Proposal | by **15 Sep 2026** | Proposal submitted (this document, finalised) |
| — First Presentation | **30 Sep 2026**, 6:30–10:30 pm, Zoom | Goals, data resources, techniques, progress |
| P5 — Hardening + integration | Oct | Full evaluation, PM1 integration demo, unknown-object path, Jetson test |
| P6 — Final deliverables | by **31 Oct 2026** | Runnable system, datasets, final report, 10–15 min video, both slide decks, individual report, peer review |

---

## 9. Budget & Resource Requirements

All figures in **Singapore dollars**. Two items were purchased in CNY and are
converted at the **ECB reference rate for 2026-09-04, 1 CNY = 0.18878 SGD**; the
original CNY amount is given alongside so the conversion is auditable rather than
opaque. Every amount is a purchase price from a receipt, not an estimate.

| Item | Cost | Status |
|---|---|---|
| 2 × Intel RealSense D435i | **S$862.08** | **Acquired.** Firmware levelled to 5.17.3.10; factory calibration tables backed up |
| Inter-camera hardware sync cable (SHR-09V-S-B) | **S$21.44** (¥113.58) | **Acquired**, installed and verified 2026-08-27 |
| 2 × USB 3.2 Gen 1 cables, 2 m, A-to-C | **S$8.76** | **Acquired** |
| 2 × tripods with extension rod | **S$77.00** | **Acquired** |
| Digital angle finder / ruler / protractor | **S$18.33** | **Acquired** |
| ChArUco board, 324 × 324 mm, 8 × 8 @ 38 mm | **S$71.81** (¥380.41) | **Acquired 2026-08-28**, decode-verified on a real capture |
| **Subtotal, hardware acquired** | **S$1,059.42** | Complete — every rig purchase is receipted |
| Work surface | — | **No purchase.** Existing desk reused; 1180 × 440 mm of free surface is sufficient for the derived rig geometry (§7A) |
| Matte PLA filament, 4 × 1 kg (yellow, red, blue, grey) | [VERIFY actual — **ordered 2026-09-05**] | To produce. **227 g per object set** computed from the meshes — `mustard_bottle` 137 g, `red_cube` 49 g, `bowl` 41 g — plus **385 g of ArUco trays** (§6.3) in grey. Two object sets are budgeted because §6.2 requires a calliper check that may force a reprint; one tray set suffices. The three class colours are not free choices — they are the per-class values the synthetic dataset was rendered with (§6.2) — and grey is a **distractor** neutral, which is what a tray must be |
| 3D printer | — | **Owned.** Makes the all-printed resolution in §6.2 available at negligible cost, and is what allows fiducial mounts to be exact by construction (§6.3) |
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
