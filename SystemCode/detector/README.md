# detector — per-view object detection

Produces `DetectionSet` (`INTERFACES.md` §2). YOLOv8 inference via ONNX
Runtime, plus the letterbox / decode / NMS / tracking that surrounds it.

Training lives in `training/detector/` — Python, per `CLAUDE.md` constraint 1.

## Build and test

```bash
cmake -S SystemCode -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

ONNX Runtime is **optional**. The failure-prone logic here — letterbox
inversion, YOLOv8 decode, NMS, tracking — is pure C++ and is always built and
tested, so a machine without the runtime can still build the module. Point
`THE_EYES_ONNXRUNTIME_ROOT` at a C++ distribution to enable the inference
backend (`~/opt/onnxruntime-linux-x64-1.29.0` here).

## Mock mode

`MockDetector` replays a YAML fixture of fixed boxes, so `pose_agent` can be
developed without a trained model. An unmatched frame yields **no detections**
rather than fabricated ones — a fixture gap should look like "nothing detected",
which is a real case the pipeline must handle.

## Why there is a tracker in here

`track_id` looks cosmetic and is not. `pose_agent` keys FoundationPose
registration on `(camera_id, class, track_id)`, and registration measured
**~1.3 s against ~11 ms** for tracking (`docs/R1_STATUS.md`). An id that changes
every frame means paying registration every frame — a ~120× cost for the same
answer.

`IouTracker` is deliberately simple: greedy IoU association against the previous
frame, no motion model. The objects sit on a table and the cameras are static,
so frame-to-frame displacement is small. The per-object EKF that does exist
lives in `fusion`, where it operates on 6-DoF pose rather than pixels.

Ids are **never reused**, even after a track retires or the tracker is reset — a
recycled id would make two different physical objects share FoundationPose's
per-object state on the server.

## Things that bite, and what guards them

**YOLOv8 is not YOLOv5.** The output is `[1, 4+nc, anchors]` — channel-first,
the transpose of v5's layout — and there is **no objectness channel**. Decoding
a v8 tensor with v5 indexing produces plausible-looking garbage rather than an
error, so `DecodeYolov8` validates the channel count against the class list and
throws.

**The model and the config can disagree about classes.** A model trained on a
different class set emits correct-looking boxes under wrong labels, and the
label is the join key all the way to the pose estimator. `OnnxDetector` compares
the head's channel count against `model.class_names` at load time and refuses to
start on a mismatch.

**Letterbox inversion.** Squashing a 16:9 frame into a square would distort
every object, so the image is scaled and padded — and the inverse must be
applied to every box. Getting it wrong yields detections that are subtly offset
and read as a calibration fault. Four tests cover the round trip, including
boxes that spill into the padding band.

**NMS is per class.** A bowl overlapping a bottle is two objects; class-agnostic
NMS would silently delete one of them.

**A silent CPU fallback invalidates a benchmark.** `active_provider()` reports
what actually ran, not what was requested, and the fallback is recorded as a
warning. `detector_parity` sets `allow_cpu_fallback=false` when a provider is
named explicitly.

## Acceptance criteria status

| Criterion (`AGENTS.md` §3) | Status |
|---|---|
| C++ inference matches the Python reference within float tolerance | **Met against the trained RGB-D network** — 40/40 val frames, 109 detections, 0 unpaired, boxes within 1 px, scores within 2.27e-3. See `docs/results/detector_parity_rgbd_50k_v2.md` |
| Mock mode from a fixture | Met |
| **Modality ablation:** RGB-only vs RGB-D mAP, per class | **Run — negative result.** +0.00076 pooled mAP@0.5:0.95; `red_cube`, the class with headroom and the one predicted to move, moved +0.00026. Reported per AGENTS.md rather than withheld. See `docs/results/modality_rgb_vs_rgbd_50k_v2.json` |
| Detections whose class has no registered mesh are dropped with a warning | Met at `pose_agent`, which owns the mesh registry; asserted at the "never reaches the estimator" level |
| mAP@0.5 ≥ 0.90 on the synthetic validation split | **Met** — 0.9937 mAP@0.5, 0.9574 mAP@0.5:0.95 on `synth50k_v2` (40 epochs, 9,988 val frames). Note that mAP@0.5 is saturated here and cannot support the ablations; report mAP@0.5:0.95 and per class |
| Real-capture mAP measured and reported | **Blocked** — no real captures yet; gated on printed objects, rig mounting and calibration |
| Dataset-size ablation at ~5k / 20k / 50k frames | **Met 2026-09-05** — mAP@0.5:0.95 0.92520 / 0.94695 / 0.95738 at 5,001 / 20,006 / 40,012 train frames, identical val split, seed and schedule. Recall saturates by 20k while mAP@0.5:0.95 does not, so data past 20k buys localisation, not detections. `docs/results/dataset_size_ablation_rgbd_50k_v2.{json,png}` |
