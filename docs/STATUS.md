# Project status — 2026-08-20

Snapshot for picking the work back up. Module detail lives in each
`SystemCode/*/README.md`; this is the cross-cutting picture and the next action.

---

## Where things stand

All six MVP-path agents in `AGENTS.md` are built. **249 tests pass**,
clean build under `-Wall -Wextra -Wpedantic`.

| # | Agent | State |
|---|---|---|
| 1 | `camera_agent` | Built. **Dual-camera verified on hardware: 30 fps each; hardware-synced, jitter 4.51 ms sigma.** |
| 2 | `calib_agent` | Built. Mock mode recovers known intrinsics/extrinsics. Hardware run pending. |
| 3 | `detector_agent` | **C++ input path is RGB-D** (sentinel encoding, 2026-08-23). Training pipeline still RGB-only. |
| 4 | `pose_agent` | Built. Verified end-to-end against the real FoundationPose server. |
| 5 | `fusion_agent` ★ | Built. Ablation harness produces the headline result on synthetic input. |
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

## Detector is now RGB-D — the built module does not match

`CLAUDE.md` and `AGENTS.md` (2026-08-23 revision) specify the detector as the
project's **student-built multimodal network**: YOLOv8 with a 4-channel
early-fusion stem taking RGB + aligned metric depth, depth channel initialised
by inflating the pretrained COCO stem weights. It is explicitly not to be
simplified back to RGB-only; RGB-only survives as an ablation baseline.

`SystemCode/detector/` as built is **RGB-only** — `DetectorInput` carries no
depth, `OnnxDetector` builds a 3-channel NCHW tensor, and
`training/detector/generate_dataset.py` emits RGB frames. The letterbox, decode,
NMS, tracking, and parity harness all carry over unchanged; the work is in the
input path and the training pipeline.

Required to close the gap:

1. `DetectorInput` gains an aligned depth channel; skip frames with no depth
   rather than substituting zeros — a zero-filled depth channel is
   indistinguishable from "object at zero range" to the network.
2. Decide and document the invalid-depth encoding (sentinel vs. validity mask).
   RealSense returns 0 for stereo holes.
3. Depth normalised to metric metres clipped to ~0.2–1.5 m, not raw uint16.
4. Training: 4-channel stem, depth channel initialised from the mean of the
   pretrained RGB stem weights — random init on one channel destabilises early
   training.
5. Replicator must render depth with realistic noise, or the depth channel
   becomes the dominant sim-to-real gap. The `camera_agent` depth-noise report
   is the input to that.
6. New acceptance criterion: **modality ablation**, RGB-only vs RGB-D mAP with a
   per-class breakdown.

This raises the value of the depth-noise characterisation: it now feeds the
detector's sim-to-real fidelity as well as the evaluation error budget.

**Full scope in `docs/RGBD_REWORK.md`.** Headline: ultralytics 8.4.123 already
supports `ch=4`, 4-channel `.npy` loading, and guards HSV augmentation on channel
count — so no forking. One decision is needed first (invalid-depth encoding), and
the C++ input path is buildable now without Isaac Sim or new hardware.

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

### Do NOT wait for hardware — two things are unblocked right now

**1. Depth-noise characterisation.** Needs only a flat matte wall and a tape
measure. No board, no sync cable. It is a listed deliverable feeding the
evaluation error budget, and the tooling is written and validated synthetically:

```bash
for d in 0.4 0.6 0.8 1.0; do
  build/camera/depth_characterise config/cameras.yaml cam_front --distance $d
  build/camera/depth_characterise config/cameras.yaml cam_over  --distance $d
done
```

**2. The Replicator dataset.** The long pole, entirely independent of both
deliveries, and mostly GPU wall-clock rather than effort. It gates *every*
reportable detection number — mAP@0.5 ≥ 0.90, real-capture mAP, and the
5k/20k/50k ablation. `training/detector/replicator_dataset.py` has never been
run; Isaac Sim is not installed. Starting this now costs nothing but time
already being spent waiting.

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

## The other independent track (superseded)

`training/detector/replicator_dataset.py` is written but **has never been run** —
Isaac Sim is not installed. It gates every reportable detection number:
mAP@0.5 ≥ 0.90, real-capture mAP, and the 5k/20k/50k dataset-size ablation.

The procedural `generate_dataset.py` exists only to have made the C++ inference
path verifiable; its 0.994 mAP is a plumbing check and **is not reportable**.

These two tracks are independent — Isaac Sim work does not wait on the cameras.

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
