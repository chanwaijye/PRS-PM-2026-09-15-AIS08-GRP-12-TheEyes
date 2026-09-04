# Project status — 2026-09-05

Snapshot for picking the work back up. Module detail lives in each
`SystemCode/*/README.md`; this is the cross-cutting picture and the next action.

---

## Where things stand

All six MVP-path agents in `AGENTS.md` are built. **303 tests pass**,
clean build under `-Wall -Wextra -Wpedantic`.

| # | Agent | State |
|---|---|---|
| 1 | `camera_agent` | Built. **Dual-camera verified on hardware: 30 fps each; hardware-synced, jitter 4.51 ms sigma.** |
| 2 | `calib_agent` | Built. Mock mode recovers known intrinsics/extrinsics. Hardware run pending. |
| 3 | `detector_agent` | **Complete end to end.** RGB-D C++ input path (sentinel encoding), 4-channel training pipeline, trained on the 50k v2 set, exported to ONNX, C++/Python parity re-verified against the trained network. |
| 4 | `pose_agent` | Built. Verified end-to-end against the real FoundationPose server. |
| 5 | `fusion_agent` ★ | Built. Ablation harness runs **four** arms — two single views, a naive baseline, and the full policy. The claim is fused-vs-naive; see proposal §7A. |
| 6 | `scenegraph_agent` ★ | Built. PDDL emission, contract-tested against fusion. |
| 7 | `cluster_agent` | Not built — above MVP, cut order #2. |
| 8 | `twin_agent` | Not built — stretch, cut order #4. |

```bash
cmake -S SystemCode -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

Risks R1 (retired) and R4 (downgraded, measured) are recorded in
`docs/R1_STATUS.md` and `CLAUDE.md`.

---

## Detector: RGB-D, trained, deployed — DONE 2026-09-05

The 2026-08-23 revision specified the detector as the project's **student-built
multimodal network**: YOLOv8 with a 4-channel early-fusion stem taking RGB +
aligned metric depth, the depth channel initialised by inflating the pretrained
COCO stem weights. All six items that were outstanding are closed.

- `DetectorInput` carries aligned depth; a frame without depth is skipped, never
  zero-filled.
- Invalid depth uses the **sentinel** encoding, decided and documented once in
  `depth_encoding.hpp`. `training/detector/depth_simulation.py` implements the
  identical formula, verified as an exact round trip (0/255 levels of error).
- Depth is normalised in metric metres clipped to 0.20–1.50 m, never raw uint16.
- `train.py --inflate-stem` (default on) seeds the depth channel from the mean
  of the pretrained RGB filters before the first gradient step. Measured:
  ultralytics 8.4.123 leaves it at random init otherwise.
- Replicator renders depth through the measured D435i noise law
  (~4.5·z² mm), reproduced to 0.01 mm.
- **Modality ablation: run, and it is a negative result.** See below.

Trained: `runs/detector/rgbd_50k_v2`, 40 epochs on 40,012 train / 9,988 val.
Exported to ONNX (opset 17, fixed 1×4×640×640 in, 1×7×8400 out) and wired into
`config/detector.yaml`, whose `[VERIFY]` placeholder path is now closed.
The ONNX itself is gitignored; the regeneration command is in that file.

**`docs/RGBD_REWORK.md` is superseded by this section.**

### The modality ablation says the depth channel is not earning its place

RGB-only vs RGB-D, same 9,988 val frames, byte-identical PNGs and labels,
differing only in whether the loader reads the 3-channel PNG or the 4-channel
`.npy` — so any difference is attributable to depth and nothing else:

| | RGB-only | RGB-D | delta |
|---|---|---|---|
| mAP@0.5 | 0.99325 | 0.99371 | +0.00046 |
| mAP@0.5:0.95 | 0.95662 | 0.95738 | +0.00076 |
| recall | 0.98272 | 0.98260 | −0.00012 |
| `mustard_bottle` mAP@.5:.95 | 0.98872 | 0.98861 | −0.00011 |
| `red_cube` mAP@.5:.95 | 0.91565 | 0.91590 | +0.00026 |
| `bowl` mAP@.5:.95 | 0.96551 | 0.96763 | +0.00212 |

`red_cube` was the class predicted to move — smallest, most occluded, and the
only one with real headroom at 0.085 below ceiling. It moved by 0.0003. Two of
the three classes are marginally *worse* on recall with depth.

`AGENTS.md` anticipated this outcome and set the policy: "if they do not, that
is a reportable negative result, not a failure to hide." It is reported as one.
The likely reading is that at 100% depth validity, three geometrically distinct
classes on an uncluttered plane, and a COCO-pretrained backbone, RGB already
saturates the task and leaves depth nothing to add — which is a statement about
*this benchmark*, not about RGB-D detection. The place it could still show is
real captures, where depth validity is 97.8% rather than 100% and the appearance
gap is real. That test has not run.

Raw metrics: `docs/results/modality_rgb_vs_rgbd_50k_v2.json`.

## Hardware on order

| item | ETA | unblocks |
|---|---|---|
| Sync cable | ~~2026-08-25~~ **ARRIVED, installed, verified 08-27** | R4b hardware sync — master/slave streams; see "Hardware sync: DONE" |
| ChArUco board | **first week of Sept** (~09-04) | calibration → real extrinsics → ablation on real poses |

Against the deadlines: proposal 09-15, first presentation 09-30, final 10-31.
The board lands ~11 days before the proposal and 26 before the presentation.
Calibration itself is a day's work once the board exists, so that is workable —
but it is the whole critical path, and it has no slack for a second attempt if
the first capture set is bad. Shoot generously on day one.

### Both no-hardware tracks are now DONE

**1. Depth-noise characterisation — DONE.** Three distances, both units, matched
patch. Numbers under "What is measured vs. what is assumed" below.

**2. The Replicator dataset — DONE.** Isaac Sim 5.1.0, 50k v2 generated, and the
detector trained on it. What it gated has now largely been collected: synthetic
mAP, the modality ablation (above). Still outstanding and still gated on real
captures: real-capture mAP. Still outstanding and NOT gated on hardware: the
5k/20k/50k dataset-size ablation, runnable today via `train.py --fraction
0.125 / 0.5` against the identical val split.

### What is genuinely unblocked right now

The **physical objects**. Proposal §6.2 resolves all three classes to 3D
printed, and §6.3 puts the ArUco ground-truth seats *inside* the printed parts —
so printing gates the objects and the ground-truth mechanism together, and it
sits on the critical path beside the ChArUco board rather than after it.
Calibration must in any case follow final rigid mounting, so the real order is
**print → calliper-verify → mount → calibrate → capture**.

### Hardware sync: DONE 2026-08-27 (R4b mitigated)

Sync cable arrived and is connected. `config/cameras.yaml` ships cam_front
master (`inter_cam_sync_mode: 1`) / cam_over slave (`2`); `camera_probe` reads
both modes back off the device and passes, and the slave delivers frames — with
no working trigger it would time out instead, so this confirms the cable, not
just the setting.

**Measured under sync, 60 samples:** fixed offset **−10.97 ms**, **jitter
(sigma) 4.51 ms**, p95 |skew| 13.64 ms, 1 outlier at 38.45 ms.

An earlier revision of this file predicted the *mean skew* would drop below the
free-running 1.9–10.2 ms. It did not, and that expectation was the wrong metric:
hardware sync locks the two shutters to a common trigger, which removes
*variance*, not the constant phase offset between where each pipeline starts in
the frame period. A fixed offset is a known constant and is correctable
downstream; jitter is not. `camera_probe` scores **jitter** for that reason, and
4.51 ms against a 33 ms frame budget is comfortable.

Free-running was never directly comparable here — the 1.9 ms and 10.2 ms figures
from 08-20 are single-run *means* that varied run to run, and no jitter figure
was recorded for them. Do not quote a free-running-vs-synced improvement in the
report without measuring both with the same statistic.

One sample of 60 exceeded the 33 ms budget (38.45 ms). Not disqualifying at 1.7%,
but re-check it before the ablation capture — if that rate climbs it invalidates
the per-frame pairing that fusion assumes. `[VERIFY]` on a longer run.

---

## Camera bring-up: DONE

`camera_probe` passes every acceptance criterion on real hardware as of
2026-08-20 — both units at **30.0 fps each**, 848×480 aligned depth + colour
simultaneously, cross-camera skew **max 4.3 ms** against a 33 ms budget. Those
skew numbers are **free-running (mode 0)**; for the current hardware-synced
figures see "Hardware sync: DONE" above.

The earlier USB troubles are resolved: both cameras are on 5000M links with
matched firmware 5.17.3.10. Re-confirmed 2026-08-27 on the new 2 m USB 3.2
Gen 1 cables — both report `Usb Type Descriptor: 3.2` on separate host
controllers, and 90 s of both cameras streaming 1280×720 colour + 848×480 depth
@30 simultaneously held 29.99 fps each with 1 dropped frame in 2699, no link
renegotiation and no kernel USB errors.

**Board:** 12 × 9 squares at 60 mm (720 × 540 mm). A **printed board on flat
rigid backing is sufficient** — factory intrinsics are usable (per-unit, zero
distortion), so only the extrinsic registration must be solved locally, and that
needs one shared view rather than a 40-view sweep. Measure a square with
callipers and put the real number in `config/calibration.yaml`.

**Next action — `calib_agent` on the real rig:**

```bash
build/calibration/calibrate board board.png     # print at 100%, MEASURE a square
# ~40 views per camera at varied distance and tilt, then one final shot with the
# board at its measured world position
build/calibration/calibrate run config/calibration.yaml \
    captures/front 261722073945 captures/over 261822074567 config/extrinsics.yaml
```

That produces real extrinsics, which is the last synthetic input in the chain:

```
real extrinsics -> fusion on real poses -> occlusion ablation on REAL data
```

**Hardware sync (R4b): cable connected 2026-08-25, mitigation ACTIVE.** The
trigger reaches the slave — proven by cam_over streaming at 30 fps in slave
mode, where before the cable it delivered no frames at all.

Sync *timing* is now quantified — `camera_probe` reports **jitter 4.51 ms
sigma** on a −10.97 ms fixed offset (60 samples, 2026-08-27), superseding the
earlier note that all three measurement approaches came back degenerate.

Sync *quality* in the R4b sense is still open, and it is a different question:
R4b is about **IR cross-interference**, not timestamp alignment, and that only
manifests when both cameras illuminate the same surface — **defer that
measurement to rig mounting**, when the geometry actually exists. See
`SystemCode/camera/README.md`.

---

## Isaac Sim: INSTALLED and Replicator DEBUGGED (2026-08-25)

Isaac Sim **5.1.0 was already installed** at `~/.local/share/ov/pkg/isaac-sim-5.1.0`
(18 GB) and still works after the driver/CUDA updates — headless startup in 6.9 s,
`omni.replicator.core` usable, all needed annotators present.

`replicator_dataset.py` was rewritten to use annotators directly (no KITTI
conversion) and now **runs end to end**, emitting RGB PNG + 4-channel `.npy` +
YOLO labels in exactly the layout training consumes. Four silent failures were
found and fixed along the way — each produced plausible-looking output rather
than an error:

| symptom | cause |
|---|---|
| RGB renders pure black, depth/boxes fine | default headless config; needs `renderer="RayTracedLighting"` |
| RGB still black with lights present | no **Dome** light; the beauty pass needs ambient environment |
| every frame empty, 0% finite depth | cameras created with a *distribution* position are never placed |
| giant objects filling frame | `rep.modify.pose(scale=)` sets ABSOLUTE metres, it does not multiply |

**Not ready to generate a dataset.** Three gaps, in priority order:

1. **No CAD meshes** — all classes fall back to stand-in primitives, so any mAP
   from this is unreportable.
2. **Camera intrinsics are Replicator defaults**, not the D435i's measured
   fx ≈ 605 px at 848×480. The sim-to-real gap currently includes a lens
   mismatch unrelated to the objects.
3. **Scene framing** — the table renders ~1 m away rather than the configured
   0.6 m, and depth validity is 4–15% because most of the frame is background.

The depth channel is the one part that IS right: `depth_simulation.py` applies
the measured noise law and reproduces it to within 0.01 mm at all three
measured distances.

## Note on the procedural dataset

`generate_dataset.py` exists only to have made the C++ inference path verifiable
before Replicator worked. Its 0.994 mAP is a plumbing check and **is not
reportable**. Reportable detection numbers come from `synth50k_v2`.

---

## What is measured vs. what is assumed

Reportable, from actual runs:

- FoundationPose on sm_120: 737/737 frames, valid continuous poses, 17.2 ms/frame
  tracking (`docs/R1_STATUS.md`)
- TensorRT 11.2 builds fp32+fp16 engines for sm_120
- `pose_agent` -> real FoundationPose server: 30/30 frames, p50 11.1 ms end to end
- Detector C++/Python parity: 32 detections across 12 images, within tolerance
- Both cameras: firmware 5.17.3.10, dual streaming at 30.0 fps each, 848×480
  aligned, 97.8% valid depth pixels; free-running skew max 4.3 ms (08-20),
  hardware-synced jitter 4.51 ms sigma (08-27)
- Depth noise vs. a flat wall, matched 28x16 cm patch: **0.98 / 2.69 / 4.77 mm**
  (cam_front) and **0.75 / 2.44 / 4.27 mm** (cam_over) at 0.4 / 0.7 / 1.0 m.
  Both follow ~4.4-4.9 * z^2 mm; units agree to 9-10%. **2-4 mm at the working
  distances the cameras are mounted at.** See `SystemCode/camera/README.md`

**Synthetic, and labelled as such everywhere it appears** — the occlusion
ablation numbers, all calibration figures, and the detector mAP. Every tool that
prints them says so in its own output.

Still unmeasured: real extrinsics, ADD-S on real captures, predicate accuracy,
and IR cross-interference under real rig geometry. **Depth-noise
characterisation is DONE** — three distances, both units, matched patch — and
**hardware-synced skew is DONE**: jitter 4.51 ms sigma, 2026-08-27.

---

## Environment

- ML/GPU stack: `~/venvs/the-eyes-r1` (torch 2.13.0+cu130, TensorRT 11.2.1.2,
  ultralytics, onnxruntime-gpu). Deliberately isolated from the system Python,
  which carries ROS 2 Jazzy and `pyrealsense2`.
- FoundationPose: `~/src/FoundationPose`, weights and demo data downloaded.
  Start the inference server with
  `SystemCode/pose/server/foundationpose_server.py --foundationpose-root ~/src/FoundationPose`.
- ONNX Runtime C++: `~/opt/onnxruntime-linux-x64-1.29.0`.
- librealsense 2.58.3 at `/usr/local`, RSUSB backend.

## Known nuisances

- `rs-fw-update` **segfaults during teardown after succeeding** (exit 139).
  Confirm with `rs-fw-update -l`, never the exit code.
- `inter_cam_sync_mode` must be set **before** `pipeline.start()`; librealsense
  rejects it while streaming, and the failure is otherwise silent.
- OpenCV here is **4.6**, so the ChArUco API is the old `cv::aruco` one — code
  from current tutorials targets 4.7's `objdetect` and will not compile.
