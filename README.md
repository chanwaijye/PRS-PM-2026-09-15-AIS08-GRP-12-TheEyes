### [ Practice Module ] Project Submission Template: Github Repository & Zip File

**[ Naming Convention ]** CourseCode-StartDate-BatchCode-TeamName-ProjectName.zip

* **[ MTech Stackable Group Project Naming Example ]** PRS-PM-2026-09-15-AIS08-GRP-12-TheEyes.zip

---

## SECTION 1 : PROJECT TITLE
## THE EYES — Multi-View Perception and Scene Understanding for Robot Manipulation

---

## SECTION 2 : EXECUTIVE SUMMARY / PAPER ABSTRACT

A robot that plans intelligently but cannot see is blind. *The Planner* (IRS practice module) established a perception interface and demonstrated task reasoning against simulator ground-truth object poses. **The Eyes** implements that interface with real sensing.

Two RGB-D cameras observe a tabletop manipulation scene from complementary viewpoints. A multimodal RGB-D YOLOv8 detector — fine-tuned with a 4-channel early-fusion input stem so that geometry and appearance are consumed jointly — gates a FoundationPose 6-DoF pose estimator on each view; the independent per-view estimates are fused at the object level by a custom C++ module using confidence-weighted quaternion averaging with disagreement gating and per-object EKF temporal filtering. A scene-graph bridge converts fused poses into symbolic relational predicates (`on`, `in`, `clear`, `occluded_from`) emitted as PDDL init facts — a symbolic world state a task planner can reason over. The predicate vocabulary is defined by this project (`INTERFACES.md` §6) as the state representation for the follow-on world-model module (PM4).

The headline result is an **occlusion ablation**: identical scenes evaluated with camera 1 only, camera 2 only, and fused, demonstrating that multi-view fusion recovers objects invisible to a single viewpoint. Detectors are trained entirely on synthetic data generated in Isaac Sim with domain randomisation, and evaluated against real captures to quantify the sim-to-real gap.

The system covers all four PRS technique groups:

| Technique Group | Implementation |
| --- | --- |
| Supervised / unsupervised learning | RGB-D YOLOv8 fine-tuned on synthetic labelled data; DBSCAN clustering for unknown-object segmentation |
| Machine learning / deep learning | Multimodal RGB-D YOLOv8 detector (4-channel early-fusion stem); FoundationPose transformer pose estimator; ONNX Runtime / TensorRT deployment |
| Hybrid / ensemble approach | Object-level multi-view fusion of two independent estimators; learned frontend + classical EKF backend |
| Intelligent sensing / sense making | Dual RGB-D sensing with extrinsic calibration; poses → relational predicates; cross-view occlusion reasoning |

The multi-view fusion module and the scene-graph bridge are implemented from first principles in C++ — no off-the-shelf fusion library — and constitute the project's technical contribution.

---

## SECTION 3 : CREDITS / PROJECT CONTRIBUTION

| Official Full Name | Student ID | Work Items | Email |
| --- | --- | --- | --- |
| Chan Wai Jye | A0340644J | All system architecture, perception pipeline design, C++ fusion and scene-graph implementation, dataset generation, evaluation, report, video. Solo submission — AI-assisted development. | <A0340644J@nus.edu.sg> |

**AI Development Tools Used:**

| Tool | Role |
| --- | --- |
| Claude Chat (Anthropic) | Architecture design, literature review, proposal drafting, report writing |
| Claude Code (Anthropic) | Orchestrates coding subagents for module implementation (see `AGENTS.md`) |

**Runtime AI (embedded in the submitted system):** YOLOv8 (fine-tuned by the author), FoundationPose (pretrained, NVIDIA). No LLM is present in the runtime perception pipeline.

All AI-generated outputs are reviewed, tested and validated by the author. Full AI usage disclosure in Appendix A of the project report.

---

## SECTION 4 : VIDEO OF SYSTEM MODELLING & USE CASE DEMO

> Video will be uploaded by the final submission deadline (31 Oct 2026).

**Project video presentation (10–15 min)** — problem framing, system architecture, live perception demo, occlusion ablation result, sim-to-real findings, and evaluation methodology.

---

## SECTION 5 : USER GUIDE

`Refer to appendix <Installation & User Guide> in project report at Github Folder: ProjectReport`

### Prerequisites

| Requirement | Version | Notes |
| --- | --- | --- |
| Ubuntu | 24.04 LTS (noble) | |
| NVIDIA GPU | RTX 4070+ (12 GB VRAM min) | RTX 5070 Ti 16 GB used for development |
| librealsense | 2.x, RSUSB backend | **Do not use the DKMS package** — it fails on the 6.14 kernel |
| ROS2 | Jazzy | installed; **not used by the shipped runner** (single C++ process, see `SystemCode/runtime/README.md`) |
| CMake | 3.22+ | |
| SQLite3 | `libsqlite3-dev` | metric logging |
| GCC | 12 | C++17 |
| CUDA / TensorRT | per FoundationPose requirements | see R1 note in `CLAUDE.md` |
| NVIDIA Isaac Sim | 4.x | Free — NVIDIA Developer Program. Used for dataset generation |
| Python | 3.11 | Training pipelines only |
| Cameras | 2 × Intel RealSense D435i | hardware-synced |

### Build & Run

```bash
# 1. Clone
git clone https://github.com/chanwaijye/PRS-PM-2026-09-15-AIS08-GRP-12-TheEyes.git
cd PRS-PM-2026-09-15-AIS08-GRP-12-TheEyes

# 2. Build librealsense with the RSUSB backend (see docs — NOT the DKMS package)
#    Then confirm both cameras enumerate:
rs-enumerate-devices -s

# 3. Record camera serial numbers into the launch config
#    config/cameras.yaml  — devices are selected by serial, never by /dev/video*

# 4. Build the C++ system
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)

# 5. Calibrate (after final rigid mounting)
build/calibration/calibrate run config/calibration.yaml \
    captures/front 261722073945 captures/over 261822074567 config/extrinsics.yaml

# 6. Run the perception pipeline
build/runtime/the_eyes_run check --live      # configs, model, pose server, cameras
build/runtime/the_eyes_run live --record demo --show
#    or, with no hardware attached:
build/runtime/the_eyes_run replay captures/runtime/<session> --detector mock:<fixture> --pose mock

# 7. Occlusion ablation (headline result)
build/fusion/fusion_ablation_demo               # synthetic input, labelled as such
build/runtime/ablation_from_log runs/runtime/<session>/metrics.sqlite   # real captures, once ground truth exists
```

Every module has a mock mode and can be run without hardware — see `AGENTS.md`.

---

## SECTION 6 : PROJECT REPORT / PAPER

`Refer to project report at Github Folder: ProjectReport`

* **Proposal:** `ProjectReport/Proposal/`

**Report Sections:**

* Executive Summary
* Problem Background and Motivation
* Project Objectives & Success Measurements
* Literature Review and Related Systems
* Tools and Techniques Applied
* System Design and Models
* Data Collection and Dataset Generation
* System Implementation
* System Performance and Evaluation
* Findings and Discussion (including sim-to-real gap analysis)
* Conclusions & Recommendations
* Appendix: Project Proposal
* Appendix: Mapped System Functionalities against PSUPR, PRMLS, ISSM
* Appendix: Installation and User Guide
* Appendix: Individual Project Report
* Appendix: AI Usage Disclosure
* Appendix: List of Abbreviations
* Appendix: References

---

## SECTION 7 : MISCELLANEOUS

`Refer to Github Folder: Miscellaneous`

### System Architecture

```
Camera 1 (front) ──► YOLOv8 ──► FoundationPose ──┐
                                                  ├──► Multi-View Fusion ★ ──► Scene-Graph Bridge ★ ──► /scene_state
Camera 2 (overhead) ─► YOLOv8 ──► FoundationPose ─┘    (C++, EKF, gating)      (C++, predicates)      └► PDDL init facts
                                                  │                                                       → world model (PM4)
Residual point cloud ──► DBSCAN unknown objects ──┘
```

★ = written from first principles in C++; the project's technical contribution.

### Key Technologies

* Intel RealSense D435i × 2 — active IR stereo RGB-D, hardware-synced
* YOLOv8 — object detection, extended to 4-channel RGB-D early fusion, fine-tuned on synthetic data
* FoundationPose — model-based 6-DoF pose estimation and tracking
* Omniverse Replicator — synthetic training data with domain randomisation
* NVIDIA Isaac Sim — dataset generation and digital-twin reconstruction
* Eigen / PCL / OpenCV — linear algebra, point clouds, calibration
* ROS2 Jazzy — middleware
* ONNX Runtime / TensorRT — C++ inference deployment
* SQLite — metric logging

### Repository Layout

```
ProjectReport/     report, proposal, individual report
SystemCode/        camera, calibration, detector, pose, fusion, scenegraph, clustering, twin
training/          YOLOv8 training pipelines, Replicator scene generation
config/            camera serials, extrinsics, predicate tolerances, mesh registry
scripts/           evaluation harnesses, ablation runner
CLAUDE.md          project context for AI-assisted development
AGENTS.md          coding subagent definitions and acceptance criteria
INTERFACES.md      data contracts between modules
```

---

**This [Pattern Recognition and Machine Learning Systems (PRMLS)](https://www.iss.nus.edu.sg "Institute of Systems Science, National University of Singapore") course is part of the Analytics and Intelligent Systems and Graduate Certificate in [Pattern Recognition Systems (PRS)](https://www.iss.nus.edu.sg "Pattern Recognition Systems") series offered by [NUS-ISS](https://www.iss.nus.edu.sg "Institute of Systems Science, National University of Singapore").**
