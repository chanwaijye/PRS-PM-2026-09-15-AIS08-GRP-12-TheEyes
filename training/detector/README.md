# training/detector — YOLOv8 dataset generation, training, evaluation

Python is allowed here: `CLAUDE.md` constraint 1 permits it for training
pipelines and offline data generation. Everything that runs at inference time is
C++ in `SystemCode/detector/`.

Run everything in the R1 venv: `~/venvs/the-eyes-r1/bin/python`.

## Pipeline

```bash
# 1. Dataset
python replicator_dataset.py --meshes assets/usd --out data/synth --frames 50000   # REAL path
python generate_dataset.py --out data/toy --train 800 --val 200                    # stand-in

# 2. Fine-tune from COCO weights
python train.py --data data/synth/data.yaml --epochs 60

# 3. Export for the C++ runtime
python export_onnx.py --weights runs/.../best.pt --out assets/models/yolov8_the_eyes.onnx

# 4. Verify the C++ path agrees with the reference
build/detector/detector_parity config/detector.yaml frame.png cpu > cpp.txt
python parity_reference.py --model best.onnx --image frame.png --compare cpp.txt

# 5. Metrics, and the dataset-size ablation
python evaluate.py --weights best.pt --data data/synth/data.yaml
python evaluate.py --ablation --data data/synth/data.yaml --sizes 5000 20000 50000
```

## Two dataset generators, and which one counts

`replicator_dataset.py` is the **real** path — Omniverse Replicator rendering
the actual CAD meshes with domain randomisation over lighting, materials, pose,
camera viewpoint, distractors, and deliberate occlusion. Every reported mAP
figure and the dataset-size ablation must come from here.

**[VERIFY] It has never been run.** Isaac Sim is not installed on this machine,
so it is written against Replicator's documented API and should be treated as a
starting point to debug, not as working code.

`generate_dataset.py` draws primitive shapes with OpenCV. It exists for one
reason: to make the C++ inference path verifiable before Isaac Sim exists.
Numbers from it are **not reportable** as detection results.

## Parity check — why it is shaped this way

The reference is **ultralytics' own predict path**, not a Python
re-implementation of the same preprocessing. Comparing my C++ against my own
Python would only prove the two agree with each other. Comparing against the
canonical implementation is what actually validates the letterbox inversion, the
YOLOv8 decode, and the NMS in `SystemCode/detector/`.

Verified 2026-08-20 on the stand-in dataset: **12/12 val images, 32 detections,
all matching** — classes identical, scores within 1e-3, boxes within 2 px. The
box tolerance covers integer-rounding order after the letterbox inversion; a
genuine coordinate-mapping bug would be off by far more than 2 px.

## The dataset-size ablation

`evaluate.py --ablation` trains one model per training-set size and evaluates
each on the **same held-out val split**. Sub-sampling val as well would make
each point measure a different test set, and the curve would confound dataset
size with evaluation noise.

`AGENTS.md` §3 makes this a deliverable, not a diagnostic: it is the
supervised-learning methodology evidence, justifying the final dataset size
empirically rather than by assertion.

## Results so far

| Run | Data | mAP@0.5 | mAP@0.5:0.95 | Reportable? |
|---|---|---|---|---|
| 2026-08-20, yolov8n, 25 epochs | **stand-in** procedural, 600 train / 150 val | 0.994 | 0.983 | **No** |

That number says the training and export path works end to end. It says nothing
about detection difficulty — the procedural shapes are trivially separable by
colour. Treat it as a plumbing check, not a result.
