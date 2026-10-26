# Data in the submission

Git holds only this file. `scripts/make_submission.sh` adds the folders below
when it builds the submission ZIP, which is where they are found.

| Folder | What it holds | Size |
| --- | --- | --- |
| `results/` | The logs every number in the report comes from | 277 MB |
| `real_v1/` | The real-frame detector set | 2.9 GB |
| `synth50k_v2_sample/` | 300 frames of the 50,000-frame synthetic set | 536 MB |
| `scored_scene_example/` | One scored recording, whole, so a replay can be run | 495 MB |

## `results/`

- `results/scored/<day>/<scene>/`, one folder for each recording processed
  by `scripts/capture_day.sh process`:
  - `metrics.sqlite` — the replay: every frame pair's detections, per-view
    poses, fused objects and facts;
  - `gt.log` — the ground truth from the marker trays;
  - `visibility.csv` and `visibility.log` — how much of each object each
    camera sees, which fixes the scene's occlusion level;
  - `ablation.log` — single camera, plain average and fused, scored against
    that ground truth;
  - `config_recording/` and `config_stills/` — the calibration the
    recording was processed with.
- `results/runtime/<session>_demo_*/metrics.sqlite` — the live takes, the
  ones in the video among them (`demo_cold_e`, `demo_live_d`, 25 Oct).

Each database was copied with `sqlite3 .backup`, so it opens on its own. The
16 scored scenes of the report are under `20261005`, `20261006` and
`20261007`, the two `in` scenes under `20261010`, and the twelve training
layouts the detector was fine-tuned on under `20261007_train`. The intervals
over the 16 scenes: `scripts/ablation_ci.py`.

## `real_v1/`

Real frames from both cameras, labelled from measured ground truth
(`training/detector/real_dataset.py`): 360 training frames from ten of the
twelve training layouts, 72 validation frames from the other two, and 960
test frames from the 16 scored scenes. Each frame is two files with one name:

- `.npy` — the detector's input, 480 × 848 × 4 `uint8`: colour, then depth.
  Depth is encoded 0.20–1.50 m onto 26–255, with 0 meaning no reading
  (`training/detector/depth_simulation.py`, `encode_depth`).
- `.png` — the colour image, to look at.

Labels are YOLO text files, one box a line: class, then centre and size
as fractions of the image. The classes are 0 `mustard_bottle`, 1 `red_cube`,
2 `bowl`. `manifest.csv` names each frame's scene, camera and frame pair.

`data.yaml` is the set as trained and tested. `data_t1.yaml` to
`data_t4.yaml` hold the test frames by occlusion level, and `data_day*.yaml`
by capture day. A path in these files is relative to the folder.

`data_mixed.yaml` is the mix the shipped model was fine-tuned on:
`train_mixed.txt` lists the 360 real training frames three times each and
2,000 synthetic training frames. The synthetic ones are named where they are
on the author's machine (`/opt/the-eyes-data/synth50k_v2/`), because that set
is not shipped; this file records the fine-tune but cannot rerun it from the ZIP.

## `synth50k_v2_sample/`

The first 250 training and 50 validation frames of the synthetic set: Isaac
Sim, with domain randomisation (`training/detector/generate_dataset.py`).
They are in the same format as `real_v1/`, with the set's
`generation.log`. The whole set is 40,012 training and 9,988 validation frames,
88 GB, too large to ship.

## `scored_scene_example/20261005_t4_cube_over/`

The `t4_cube_over` scene of 5 Oct: the cube, hidden from `cam_over` by a
card.

- `session/` — the recording: 300 frame pairs from both cameras.
- `gt_stills/` — the marker-tray stills the ground truth is measured from.
- The calibration the scene was recorded and measured under, and its
  `notes.yaml`.

To replay it, use the configuration it was processed with. From the
repository root, after the build of `README.md` §5:

```bash
S=data/scored_scene_example/20261005_t4_cube_over
C=data/results/scored/20261005/t4_cube_over/config_recording
build/runtime/the_eyes_run replay $S/session --pose foundationpose --config-dir $C --db /tmp/t4.sqlite
build/runtime/ablation_from_log /tmp/t4.sqlite --config-dir $C
```

`--pose mock` instead runs without the pose server.

## Not in the ZIP

The author keeps, but does not ship:

- every other recording: `captures/`, 61 GB;
- the full synthetic set: 88 GB;
- the detector training runs.

The two detector models the system uses are in `assets/models/`.
