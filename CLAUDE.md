# CLAUDE.md — The Eyes (PRS Practice Module, PM2)

Auto-loaded context. Read `AGENTS.md` for module ownership and `INTERFACES.md` for
data contracts before writing code in any module.

---

## What this project is

**The Eyes** — a real-time multi-view perception system for tabletop robot
manipulation. Two RGB-D cameras at different viewpoints → object detection →
6-DoF pose estimation → object-level fusion → symbolic scene graph → PDDL init
facts consumed by *The Planner* (PM1).

Academic context: NUS-ISS MTech AI Systems, Pattern Recognition Systems (PRS)
practice module. Solo submission, Group 12, cohort AIS08.
Repo/ZIP name: `PRS-PM-2026-09-15-AIS08-GRP-12-TheEyes`.

This is the second of four modules building toward the MANIP-WM capstone. The
scene-graph schema defined here is the state representation the PM4 world model
will operate on. **Interface stability is a first-class requirement.**

---

## Hard constraints — do not violate without asking

1. **C++17 for all runtime code.** Python only for training pipelines and
   offline data generation. If a task seems easier in Python, it still goes in
   C++ unless it is training or dataset tooling.
2. **The fusion module and scene-graph bridge are written from first
   principles.** Do NOT pull in a sensor-fusion library, a pose-averaging
   library, or a scene-graph library for these two modules. They are the graded
   technical contribution; using an off-the-shelf implementation defeats the
   purpose. Eigen for linear algebra is fine. PCL for point-cloud primitives is
   fine. A packaged EKF or packaged multi-view fusion is not.
3. **`ObjectState` fields may be added, never removed or re-typed.** PM4 depends
   on this contract. See `INTERFACES.md`.
4. **Never select cameras by `/dev/video*` or enumeration order.** Always by
   serial number. USB enumeration is not stable across reboots and this will
   fail during a live demo.
5. **No fabricated numbers anywhere.** Metrics in docs come from actual runs.
   Unresolved items get a `[VERIFY]` marker, not a plausible guess.

---

## Environment

| Item | Value | Note |
|---|---|---|
| OS | Ubuntu 24.04 LTS (noble) | dual-boot, desktop |
| GPU | RTX 5070 Ti 16 GB (Blackwell, sm_120) | see R1 below |
| ROS2 | Jazzy | PM1 is on Humble — integrate by contract, not shared workspace |
| Compiler | GCC 12, C++17 | |
| Cameras | 2 × Intel RealSense D435i | active IR stereo |
| Edge target | Jetson Orin Nano Super | stretch goal only |

### librealsense on Ubuntu 24.04 — known trap

The `librealsense2-dkms` package officially supports HWE kernels 5.15 / 5.19 /
6.5 and **fails to build on the 6.14 kernel that a fresh 24.04 install lands
on**. Do not debug DKMS. Build with the RSUSB backend instead:

```bash
cmake .. -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release
```

The upstream repo moved from `IntelRealSense/librealsense` to
`realsenseai/librealsense` after the 2025 spinout.

---

## Hardware decisions already made (do not re-litigate)

- **D435i over D405.** The D405 has better close-range accuracy but no laser
  projector — passive stereo fails on textureless objects, which breaks the
  FoundationPose depth-quality assumption.
- **D435i over Yahboom HP60C.** The HP60C is structured light: depth is decoded
  by matching a pre-calibrated reference pattern. Two units projecting into the
  same workspace corrupt each other's decode, and there is no hardware sync pin.
  Active stereo degrades gracefully under interference; structured light does
  not.
- **Two cameras, not passive stereo.** Multi-view is the contribution.
- **No thermal/IR camera.** FoundationPose has no IR input channel and tabletop
  rigid objects are all at ambient temperature — no discriminative signal.
- **Camera sync:** one unit `inter_cam_sync_mode=1` (master), the other `=2`
  (slave). Mitigates mutual IR interference (risk R4b).
- **Detector is RGB-D, not RGB.** YOLOv8 with a 4-channel early-fusion input stem
  (RGB + aligned metric depth), depth channel initialised by inflating the
  pretrained COCO stem weights. This is the project's student-built multimodal
  network — do not "simplify" it back to RGB-only. The RGB-only baseline is kept
  only as an ablation. Early fusion was chosen over a dual-branch late-fusion
  design to preserve full reuse of the pretrained backbone.

---

## Open risks

| ID | Risk | Status |
|---|---|---|
| R1 | FoundationPose TensorRT engines may not build for Blackwell sm_120 | **RETIRED 2026-08-20** — and the premise was wrong: upstream FoundationPose uses no TensorRT at all, it loads PyTorch checkpoints. Full `run_demo.py` runs on sm_120, 737/737 frames, valid poses. TensorRT 11.2 separately builds fp32+fp16 engines for sm_120 if ever wanted. See `docs/R1_STATUS.md`. |
| R4 | FoundationPose too slow for dual-view real-time | **Downgraded.** Measured 17.2 ms/frame tracking (58 FPS) for one object on the RTX 5070 Ti → ~29 FPS/camera for two views serially. Scales ~linearly in objects (3 objects × 2 views ≈ 9.7 FPS). Detection-gated ROIs and alternate-frame scheduling remain worthwhile, no longer mandatory. |
| R4b | Dual-camera active IR cross-interference | Hardware sync + angled viewpoints |
| R3 | Sim-to-real gap | Domain randomisation breadth; the gap is a reportable finding, not a failure |

---

## MVP — protect this, cut everything else

Must ship:
1. Single-camera pipeline (YOLOv8 → FoundationPose → 6-DoF) on a **real** camera
2. Second camera + C++ fusion + **occlusion ablation result**
3. Scene-graph bridge → PDDL facts
4. Evaluation numbers: mAP, ADD-S single vs. fused, predicate accuracy

Cut order under schedule pressure:
1. Jetson deployment test
2. Unknown-object DBSCAN path
3. Live PM1 integration demo (logged output suffices)
4. Digital-twin reconstruction viewport

**The fusion occlusion ablation is never cut.** It is the entire technical claim.

---

## Assessment context (affects what "done" means)

The project must demonstrate at least three of four PRS technique categories;
this one covers all four:

| # | Category | Where |
|---|---|---|
| ① | Supervised / unsupervised | RGB-D YOLOv8 fine-tune (supervised); DBSCAN unknown-object segmentation (unsupervised) |
| ② | ML / DL techniques | **Multimodal RGB-D YOLOv8** (4-channel early-fusion stem, COCO stem inflation); FoundationPose transformer; ONNX/TensorRT deployment |
| ③ | Hybrid / ensemble | Two per-view estimators fused with confidence weighting + disagreement gating; learned frontend + classical EKF backend |
| ④ | Intelligent sensing / sense-making | Dual RGB-D + calibration; poses → relational predicates |

If a change would weaken one of these claims, flag it rather than doing it.

---

## Working style

- Direct and decision-oriented. State trade-offs once; don't re-explain.
- Every module has a mock mode so it is testable without hardware.
- `fusion` and `scenegraph` are testable with **zero hardware** — build and test
  those against synthetic inputs while calibration is still pending.
- Log metrics to SQLite; the report needs reproducible numbers.
