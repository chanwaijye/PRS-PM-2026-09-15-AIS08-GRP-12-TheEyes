### [ Practice Module ] Project Submission Template: Github Repository & Zip File

**[ Naming Convention ]** CourseCode-StartDate-BatchCode-TeamName-ProjectName.zip

* **[ MTech Stackable Group Project Naming Example ]** PRS-PM-2026-09-15-AIS08-GRP-12-TheEyes.zip

---

## SECTION 1 : PROJECT TITLE
## THE EYES — Multi-View Perception and Scene Understanding for Robot Manipulation

---

## SECTION 2 : EXECUTIVE SUMMARY / PAPER ABSTRACT

A robot that plans intelligently but cannot see is blind. *The Planner* (IRS practice module) established a perception interface and demonstrated task reasoning against simulator ground-truth object poses. **The Eyes** implements that interface with real sensing.

Two RGB-D cameras observe a tabletop manipulation scene from complementary viewpoints. A multimodal RGB-D YOLOv8 detector — a 4-channel early-fusion input stem, so that geometry and appearance are consumed jointly — gates a FoundationPose 6-DoF pose estimator on each view. The per-view estimates are fused at the object level by a C++ module: a gate that decides between disagreeing views by checking each pose against both cameras' depth images, confidence-weighted quaternion averaging, and a per-object EKF. A scene-graph bridge converts fused poses into symbolic relational predicates (`on`, `in`, `clear`, `occluded_from`), drops any object that rests on nothing, and emits the rest as PDDL init facts — a symbolic world state a task planner can reason over. The predicate vocabulary is defined by this project (`INTERFACES.md` §6) as the state representation for the world model of the MANIP-WM capstone.

The main experiment is an **occlusion ablation**: the same 16 recorded scenes scored with each camera alone, with a plain average of the two, and with the gated fusion, against ground truth from printed marker trays. On the real rig the system delivers:

* a fused pose at **2.76 mm** ADD-S over all 16 scenes;
* objects that one camera cannot see, found on **every frame**;
* a gating policy that beats a plain average of the two views by **9.8 mm**;
* `on`, `in` and `clear` facts at precision ≥ 0.994 and recall 1.000.

The fused pose is not more accurate than the better single camera by a margin this rig can resolve, and the report says so. The detector is trained on 40,012 synthetic frames rendered in Isaac Sim with domain randomisation, then fine-tuned on 360 real frames labelled from measured ground truth; the sim-to-real gap is measured (real-frame mAP@0.5:0.95 0.670 before fine-tuning, 0.934 after). Every figure, with its 95% interval, is in §9 of the project report.

The system covers all four PRS technique groups:

| Technique Group | Implementation |
| --- | --- |
| Supervised / unsupervised learning | RGB-D YOLOv8 trained on labelled synthetic frames and fine-tuned on labelled real frames (supervised). The unsupervised DBSCAN unknown-object path was cut and not built |
| Machine learning / deep learning | Multimodal RGB-D YOLOv8 detector (4-channel early-fusion stem); FoundationPose transformer pose estimator (pretrained); C++ deployment through ONNX Runtime with the CUDA provider |
| Hybrid / ensemble approach | Object-level fusion of two independent per-view estimators, gated on depth evidence; learned frontend + classical EKF backend |
| Intelligent sensing / sense making | Dual RGB-D sensing with extrinsic calibration; depth checks of each pose against both cameras; poses → relational predicates, with a support check |

The multi-view fusion module and the scene-graph bridge are written from first principles in C++ — no fusion, filtering, pose-averaging or scene-graph library — and constitute the project's technical contribution. The code was written with Claude Code under the author's design and review (report, Appendix E).

---

## SECTION 3 : CREDITS / PROJECT CONTRIBUTION

| Official Full Name | Student ID | Work Items | Email |
| --- | --- | --- | --- |
| Chan Wai Jye | A0340644J | System architecture, interfaces and pipeline design; design and review of the C++ fusion and scene-graph modules (code written with Claude Code); rig build, calibration and every capture; dataset generation and detector training; evaluation; report, video. Solo submission — AI-assisted development. | <A0340644J@nus.edu.sg> |

**AI Development Tools Used:**

| Tool | Role |
| --- | --- |
| Claude (Anthropic) | Architecture review, drafting of documents |
| Claude Code (Anthropic) | Wrote the code under the author's design and review, including the C++ fusion and scene-graph modules (see `AGENTS.md`) |

**Runtime AI (embedded in the submitted system):** YOLOv8 (trained and fine-tuned by the author), FoundationPose (pretrained, NVIDIA). No LLM is present in the runtime perception pipeline.

All AI-generated outputs are reviewed, tested and validated by the author. Full AI usage disclosure in Appendix E of the project report.

---

## SECTION 4 : VIDEO OF SYSTEM MODELLING & USE CASE DEMO

**Video (14:34):** `ProjectReport/PRS-PM-2026-AIS08-GRP-12-TheEyes-Video.mp4`, in the submission ZIP. It is not in git: at 129 MB it is over GitHub's file size limit.

| Time | Scene |
| --- | --- |
| 0:00 | One camera loses the cube — live |
| 0:29 | Title |
| 0:59 | Why one camera is not enough |
| 1:51 | The rig |
| 2:42 | The pipeline |
| 4:28 | Live on the rig — one continuous take, the table filmed by a phone at the same moment |
| 6:50 | How it is measured |
| 8:11 | The result: a hidden object is still found |
| 9:39 | When fusion believed the wrong camera |
| 11:11 | Objects that are not there |
| 12:29 | What it still gets wrong |
| 13:47 | Close |

Live footage is labelled LIVE and every replay of a recorded scene REPLAY. The narration is the author's; its script is `ProjectReport/presentation/video_script.txt`, with the source of every number it speaks. The edit is a script, `scripts/build_video.py` (`docs/VIDEO_PLAN.md` §7).

---

## SECTION 5 : USER GUIDE

`Refer to appendix <Installation & User Guide> (Appendix C) in project report at Github Folder: ProjectReport`

### Prerequisites

| Requirement | Version | Notes |
| --- | --- | --- |
| Ubuntu | 24.04 LTS (noble) | |
| NVIDIA GPU | CUDA-capable | Developed and measured on an RTX 5070 Ti 16 GB; no other GPU tested |
| CUDA | 13 | for the ONNX Runtime GPU build and FoundationPose's PyTorch |
| librealsense | 2.x, RSUSB backend | **Do not use the DKMS package** — it fails on the 6.14 kernel |
| CMake | 3.22+ | |
| GCC | 13 (Ubuntu 24.04 default) | C++17 |
| Libraries | | Eigen3, OpenCV with the aruco module, yaml-cpp, SQLite3 (`libsqlite3-dev`), GoogleTest |
| ONNX Runtime | 1.29, GPU (CUDA) build | detector inference |
| FoundationPose | upstream checkout + Python venv | runs beside the C++ process as a pose server on a Unix socket; PyTorch, no TensorRT (`docs/R1_STATUS.md`) |
| NVIDIA Isaac Sim | 5.1.0 | Free — NVIDIA Developer Program. Dataset generation only |
| Python | 3.12 | Training and dataset tools; Isaac Sim runs its own bundled 3.11 |
| Cameras | 2 × Intel RealSense D435i | selected by serial number, never by `/dev/video*` |

The detector models in `assets/models/` are not tracked by git (`*.onnx` is ignored); they ship in the submission ZIP.

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

# 4. Build the C++ system -- the CMake project is SystemCode/; build/ sits at the
#    repo root, which is where every command below expects it. The ONNX Runtime
#    root is the GPU (CUDA) build; leave the option out for a CPU-only detector.
cmake -S SystemCode -B build -DCMAKE_BUILD_TYPE=Release \
      -DTHE_EYES_ONNXRUNTIME_ROOT=$HOME/opt/onnxruntime-linux-x64-gpu_cuda13-1.29.0
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure    # 8 suites, 479 tests, no hardware needed

# 5. Calibrate (after final rigid mounting); see docs/CALIBRATION_PROTOCOL.md
build/calibration/calibrate run config/calibration.yaml \
    captures/front 261722073945 captures/over 261822074567 config/extrinsics.yaml

# 6. Start the pose server from the repository root, then run the pipeline
~/venvs/the-eyes-r1/bin/python SystemCode/pose/server/foundationpose_server.py \
    --foundationpose-root ~/src/FoundationPose
build/runtime/the_eyes_run check --live      # configs, model, pose server, cameras
build/runtime/the_eyes_run live --pose foundationpose --record demo --show
#    or replay a recorded session:
build/runtime/the_eyes_run replay captures/runtime/<session> --pose foundationpose
#    or, with no hardware and no pose server:
build/runtime/the_eyes_run replay captures/runtime/<session> --detector mock:<fixture> --pose mock

# 7. Occlusion ablation (headline result)
build/fusion/fusion_ablation_demo config        # synthetic input, labelled as such
scripts/capture_day.sh process --day 20261005 --force    # replay, ground truth, tiers, ablation table
build/runtime/ablation_from_log runs/scored/20261005/t1_cube/metrics.sqlite
scripts/ablation_ci.py --json out.json          # intervals over the 16 scored scenes
scripts/predicate_accuracy.py                   # predicate precision and recall
```

`config/pose.yaml` ships `backend: "mock"`, so a run on FoundationPose passes `--pose foundationpose`. Every module has a mock mode and can be run without hardware — see `AGENTS.md`.

---

## SECTION 6 : PROJECT REPORT / PAPER

`Refer to project report at Github Folder: ProjectReport`

* **Final report:** `ProjectReport/PRS-PM-2026-AIS08-GRP-12-TheEyes-FinalReport.pdf` (source `.md` beside it, figures in `ProjectReport/figures/`)
* **Proposal:** `ProjectReport/PRS-PM-2026-AIS08-GRP-12-TheEyes-Proposal.pdf`
* **Interim report:** `ProjectReport/PRS-PM-2026-AIS08-GRP-12-TheEyes-InterimReport-DRAFT.pdf`
* **First Presentation:** `ProjectReport/PRS-PM-2026-AIS08-GRP-12-TheEyes-FirstPresentation.pptx`
* **Final Presentation:** `ProjectReport/PRS-PM-2026-AIS08-GRP-12-TheEyes-FinalPresentation.pptx` (PDF beside it)
* **Video:** `ProjectReport/PRS-PM-2026-AIS08-GRP-12-TheEyes-Video.mp4` (in the ZIP; Section 4)

**Report Sections:**

* Executive Summary
* Problem Background and Motivation
* Project Objectives and Success Measurements
* Literature Review and Related Systems
* Tools and Techniques Applied
* System Design and Models
* Data Collection and Datasets
* System Implementation
* System Performance and Evaluation (including the sim-to-real gap)
* Findings and Discussion
* Limitations
* Conclusions and Recommendations
* Appendix A: Project Proposal
* Appendix B: System Functionalities Mapped to the Courses (PSUPR, PRMLS, ISSM)
* Appendix C: Installation and User Guide
* Appendix D: Individual Project Report
* Appendix E: AI Usage Disclosure
* Appendix F: List of Abbreviations
* Appendix G: References

---

## SECTION 7 : MISCELLANEOUS

`Refer to Github Folder: docs`

### System Architecture

```
cam_front ─► RGB-D YOLOv8 ─► FoundationPose ─┐
                                              ├─► view evidence ─► Multi-view fusion ★ ─► Scene-graph bridge ★ ─► SceneState
cam_over  ─► RGB-D YOLOv8 ─► FoundationPose ─┘   (depth checks)   (gate, average, EKF)    (predicates, support)  └► PDDL init facts
                                                                                                                   → task planner,
                                                                                                                     MANIP-WM world model
```

★ = written from first principles in C++; the project's technical contribution. One C++ process; FoundationPose runs beside it as a pose server.

### Key Technologies

* Intel RealSense D435i × 2 — active IR stereo RGB-D
* YOLOv8 — object detection, extended to 4-channel RGB-D early fusion, trained on synthetic data and fine-tuned on real frames
* FoundationPose — model-based 6-DoF pose estimation and tracking
* NVIDIA Isaac Sim with Omniverse Replicator — synthetic training data with domain randomisation
* Eigen / OpenCV — linear algebra, ChArUco calibration
* ONNX Runtime — C++ inference deployment
* SQLite — metric logging

### Repository Layout

```
ProjectReport/     final report and figures, proposal, interim report, presentation decks and scripts
SystemCode/        C++17 runtime: camera, calibration, detector, pose, fusion, scenegraph,
                   runtime (the_eyes_run), integration (cross-module tests), common
training/          Python: Isaac Sim dataset, YOLOv8 training and export, real-frame labels,
                   marker trays, pose profiling
config/            camera serials, extrinsics, fusion, predicate tolerances, mesh registry
assets/            detector models, meshes, marker trays, print files, photos
scripts/           capture day, evaluation, report figures, PDF rendering
docs/              protocols, plans, decision records, saved results
data/              in the submission ZIP: result logs, the real-frame detector set, a sample
                   of the synthetic set, one whole recording (data/DATA.md)
CLAUDE.md          project context for AI-assisted development
AGENTS.md          module ownership and acceptance criteria
INTERFACES.md      data contracts between modules
```

Recordings (`captures/`) and processed logs (`runs/`) are not in git.

---

**This [Pattern Recognition and Machine Learning Systems (PRMLS)](https://www.iss.nus.edu.sg "Institute of Systems Science, National University of Singapore") course is part of the Analytics and Intelligent Systems and Graduate Certificate in [Pattern Recognition Systems (PRS)](https://www.iss.nus.edu.sg "Pattern Recognition Systems") series offered by [NUS-ISS](https://www.iss.nus.edu.sg "Institute of Systems Science, National University of Singapore").**
