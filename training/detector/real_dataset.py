#!/usr/bin/env python3
"""A labelled detector dataset from real recordings, in the synthetic set's format.

The label of a real frame is not drawn by hand: each object's pose is measured
from its marker tray (gt_from_trays), its mesh is rendered into each camera at
that pose, and the part of the silhouette that nothing nearer hides gives the
box (visibility_from_gt --boxes; the median over the scene's pairs). That is the same definition the synthetic set
uses -- Replicator's TIGHT box, the visible extent -- so a model trained on one
can be scored on the other.

    <out>/images/<split>/<day>_<scene>_<camera>_<pair>.png   RGB, for inspection
    <out>/images/<split>/....npy                             [H, W, 4] uint8 = R, G, B, depth
    <out>/labels/<split>/....txt                             YOLO: class cx cy w h, normalised
    <out>/data.yaml                                          channels: 4
    <out>/manifest.csv                                       one row per frame written

What the frame looks like is what the runtime's detector is given, not what the
camera recorded: the brightness floor of config/runtime.yaml is applied to the
colour image, and depth goes through the per-camera scale of the scene's
depth_correction.yaml and then the sentinel encoding of config/detector.yaml.

A scene is named <day>/<scene> and is read from where capture_day.sh leaves it:
captures/scored/<day>/<scene>/{session, notes.yaml} and
runs/scored/<day>/<scene>/{boxes.csv, config_recording/}.

    python real_dataset.py --out /opt/the-eyes-data/real_v1 \\
        --train 20261007_train/train_01 ... --val 20261007_train/train_06 ... \\
        --test 20261005/t1_cube ...

Rules, each one stated because it decides a number:
  * an object under --min-visible of its silhouette showing gets NO box. A
    detection there then counts as a false positive, which is harsh on a
    detector that finds a sliver; the alternative, a box on something nobody
    can see, is worse.
  * a box under --min-box pixels on a side is dropped, as in the synthetic set.
  * a scene whose notes list no objects is all negatives: every frame is
    written with an empty label file. Likewise a class the notes do not list
    gets no box, though its empty tray has a measured pose.
  * frames of one scene are near copies of each other (nothing moves), so only
    every --every'th pair is taken.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from depth_simulation import DEPTH_MAX_M, DEPTH_MIN_M, VALID_FLOOR, encode_depth  # noqa: E402

CLASSES = ["mustard_bottle", "red_cube", "bowl"]  # the synthetic set's order


def brighten(rgb: np.ndarray, cfg: dict) -> tuple[np.ndarray, float]:
    """runtime/brightness.cpp: one gain per frame, never below 1, saturating."""
    if not cfg.get("enabled", False):
        return rgb, 1.0
    luma = 0.299 * rgb[..., 0].mean() + 0.587 * rgb[..., 1].mean() + 0.114 * rgb[..., 2].mean()
    if luma <= 0:
        return rgb, 1.0
    gain = min(float(cfg["max_gain"]), max(1.0, float(cfg["target_mean"]) / luma))
    if gain == 1.0:
        return rgb, 1.0
    return np.clip(np.rint(rgb.astype(np.float32) * gain), 0, 255).astype(np.uint8), gain


def check_encoding(detector_cfg: dict) -> None:
    """The encoding constants live in two places; refuse to write if they differ."""
    d = detector_cfg["depth"]
    want = (float(d["min_m"]), float(d["max_m"]), float(d["valid_floor"]), float(d["sentinel"]))
    have = (DEPTH_MIN_M, DEPTH_MAX_M, VALID_FLOOR, 0.0)
    if any(abs(a - b) > 1e-9 for a, b in zip(want, have)):
        raise SystemExit(f"depth encoding differs: config/detector.yaml {want}, "
                         f"depth_simulation.py {have}")


def scene_boxes(path: Path) -> dict[str, list[dict]]:
    """One box per camera and object for the whole scene: the MEDIAN over its pairs.

    Nothing moves in a scene, so the pairs differ only in depth noise -- and
    depth noise moves the label. A pixel with no depth counts as seen, so on
    the pairs where an occluder's surface drops out of the depth image a fully
    hidden object reads as visible (measured: t4_bottle_over_b, 2% visible at
    the median, 25% or more on 4 of 30 sampled pairs). The median is what the
    tier check uses for the same reason.
    """
    series: dict[tuple[str, str, str], list[dict]] = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            series.setdefault((r["camera_id"], r["cls"], r["instance"]), []).append(r)
    out: dict[str, list[dict]] = {}
    for (cam, cls, _), rows in series.items():
        vis = [float(r["visible_fraction"]) for r in rows]
        seen = [r for r in rows if int(r["seen_x0"]) >= 0]
        box = {"cls": cls, "visible_fraction": float(np.nanmedian(vis)) if vis else float("nan")}
        for k in ("seen_x0", "seen_y0", "seen_x1", "seen_y1"):
            box[k] = int(round(float(np.median([float(r[k]) for r in seen])))) if seen else -1
        out.setdefault(cam, []).append(box)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    ap.add_argument("--train", nargs="*", default=[])
    ap.add_argument("--val", nargs="*", default=[])
    ap.add_argument("--test", nargs="*", default=[])
    ap.add_argument("--every", type=int, default=10, help="take every N-th pair of a scene")
    ap.add_argument("--test-every", type=int, default=None,
                    help="the same for the test split (default: --every)")
    ap.add_argument("--min-visible", type=float, default=0.25)
    ap.add_argument("--min-box", type=int, default=8)
    args = ap.parse_args()

    repo = args.repo
    check_encoding(yaml.safe_load((repo / "config/detector.yaml").read_text()))
    bright_cfg = yaml.safe_load((repo / "config/runtime.yaml").read_text()).get("brightness", {})

    manifest = []
    totals = {}
    for split, scenes in (("train", args.train), ("val", args.val), ("test", args.test)):
        (args.out / "images" / split).mkdir(parents=True, exist_ok=True)
        (args.out / "labels" / split).mkdir(parents=True, exist_ok=True)
        for name in scenes:
            day, scene = name.split("/")
            cap = repo / "captures/scored" / day / scene
            run = repo / "runs/scored" / day / scene
            notes = yaml.safe_load((cap / "notes.yaml").read_text())
            session = (cap / "session").resolve()
            meta = yaml.safe_load((session / "session.yaml").read_text())
            fix = yaml.safe_load((run / "config_recording/depth_correction.yaml").read_text())
            scale_of = {c: float(v["scale"]) for c, v in fix["cameras"].items()}
            negatives = not notes.get("objects")
            boxes_csv = run / "boxes.csv"
            if negatives:
                boxes = {}
            elif boxes_csv.exists():
                boxes = scene_boxes(boxes_csv)
            else:
                raise SystemExit(f"{name}: has objects but no {boxes_csv} "
                                 "(visibility_from_gt --boxes)")

            pairs = [int(r["pair_index"]) for r in csv.DictReader(open(session / "index.csv"))]
            every = args.test_every if (split == "test" and args.test_every) else args.every
            present = set(notes.get("objects") or [])
            for pair in pairs[::every]:
                for cam, cam_meta in meta["cameras"].items():
                    bgr = cv2.imread(str(session / cam / f"{pair:06d}.rgb.png"), cv2.IMREAD_COLOR)
                    raw = cv2.imread(str(session / cam / f"{pair:06d}.depth.png"),
                                     cv2.IMREAD_UNCHANGED)
                    if bgr is None or raw is None or raw.dtype != np.uint16:
                        raise SystemExit(f"{name}: unreadable frame {cam}/{pair:06d}")
                    rgb, gain = brighten(bgr[:, :, ::-1], bright_cfg)
                    depth_m = raw.astype(np.float32) * float(cam_meta["depth_scale_m"]) * scale_of[cam]
                    depth_u8 = encode_depth(depth_m, raw != 0)
                    h, w = raw.shape

                    rows, kept, dropped = [], 0, 0
                    if not negatives:
                        if cam not in boxes:
                            raise SystemExit(f"{name}: no boxes for {cam}")
                        for b in boxes[cam]:
                            # Ground truth is one pose per TRAY in the stills; a
                            # tray on the table without its object must not
                            # become a box. The notes say what was on it.
                            if b["cls"] not in present:
                                continue
                            x0, y0, x1, y1 = (int(b[k]) for k in ("seen_x0", "seen_y0", "seen_x1", "seen_y1"))
                            bw, bh = x1 - x0 + 1, y1 - y0 + 1
                            vis = float(b["visible_fraction"])
                            if x0 < 0 or not (vis >= args.min_visible) or bw < args.min_box or bh < args.min_box:
                                dropped += 1
                                continue
                            rows.append(f"{CLASSES.index(b['cls'])} {(x0 + x1 + 1) / 2 / w:.6f} "
                                        f"{(y0 + y1 + 1) / 2 / h:.6f} {bw / w:.6f} {bh / h:.6f}")
                            kept += 1

                    stem = f"{day}_{scene}_{cam}_{pair:06d}"
                    cv2.imwrite(str(args.out / "images" / split / f"{stem}.png"), rgb[:, :, ::-1])
                    np.save(args.out / "images" / split / f"{stem}.npy",
                            np.dstack([rgb, depth_u8]).astype(np.uint8))
                    (args.out / "labels" / split / f"{stem}.txt").write_text(
                        "\n".join(rows) + ("\n" if rows else ""))
                    manifest.append((split, name, cam, pair, kept, dropped, f"{gain:.3f}"))
                    t = totals.setdefault(split, [0, 0, 0])
                    t[0] += 1
                    t[1] += kept
                    t[2] += dropped

    (args.out / "data.yaml").write_text(yaml.safe_dump({
        "path": str(args.out.resolve()),
        "train": "images/train",
        "val": "images/val",
        "test": "images/test",
        "channels": 4,
        "names": {i: n for i, n in enumerate(CLASSES)},
    }, sort_keys=False))
    with open(args.out / "manifest.csv", "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["split", "scene", "camera", "pair_index", "boxes", "objects_without_a_box",
                     "brightness_gain"])
        wr.writerows(manifest)
    for split, (n, kept, dropped) in totals.items():
        print(f"{split}: {n} frames, {kept} boxes, {dropped} objects left without a box")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
