#!/usr/bin/env python3
"""Procedural synthetic dataset generator with domain randomisation — RGB-D.

WHAT THIS IS NOT: the Omniverse Replicator pipeline. `replicator_dataset.py` is
that, and it is what the report's dataset-size ablation must be run on --
photorealistic renders of the actual CAD meshes. This generator draws primitive
shapes with OpenCV.

WHAT IT IS FOR: making the whole downstream path runnable and verifiable today,
without Isaac Sim. Train -> export ONNX -> C++ inference parity can all be
exercised on this, so the C++ detector is proven correct before a single
photorealistic frame exists. Numbers produced from this data are NOT reportable
as detection results (CLAUDE.md constraint 5).

Emits YOLO format, plus a 4-channel cache for RGB-D training:
    <out>/images/{train,val}/*.png   RGB, for inspection
    <out>/images/{train,val}/*.npy   [H, W, 4] uint8 = R, G, B, depth
    <out>/labels/{train,val}/*.txt   (class cx cy w h, normalised)
    <out>/data.yaml

WHY .npy. ultralytics has no 4-channel image reader -- `cv2_flag` is only
IMREAD_GRAYSCALE or IMREAD_COLOR. But `BaseDataset.load_image` checks for a
`.npy` sibling FIRST and validates its channel count against `self.channels`,
so a pre-cached 4-channel array is loaded verbatim. That is the whole route.

WHY RGB AND NOT BGR. `Format._format_img` reverses channels only when
`img.shape[0] == 3`. A 4-channel array is passed through untouched, so whatever
order is stored here is what the network trains on -- and the C++ runtime feeds
RGB. Storing BGR would train and infer on different channel orders, and nothing
would report it.
"""

from __future__ import annotations

import argparse
import random
from pathlib import Path

import cv2
import numpy as np
import yaml

# Order defines the class indices, and must match config/detector.yaml's
# class_names and the keys in config/meshes.yaml.
CLASSES = ["mustard_bottle", "red_cube", "bowl"]

# Sentinel depth encoding -- MUST match SystemCode/detector/.../depth_encoding.hpp
# and config/detector.yaml. Valid depth maps to [VALID_FLOOR, 1]; invalid to 0.
DEPTH_MIN_M, DEPTH_MAX_M, VALID_FLOOR = 0.20, 1.50, 0.1

# Measured D435i noise, 2026-08-23: RMS ~= 4.5 * z^2 mm across both units
# (SystemCode/camera/README.md). Rendering clean depth would make the depth
# channel a signal the real sensor cannot deliver -- the network would lean on
# it and score BETTER on synthetic while scoring worse on real captures.
DEPTH_NOISE_K_MM = 4.5
INVALID_FRACTION = 0.01          # measured ~1% holes at these distances


def encode_depth(depth_m: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Metric depth -> uint8 channel with the sentinel encoding."""
    t = (np.clip(depth_m, DEPTH_MIN_M, DEPTH_MAX_M) - DEPTH_MIN_M) / (DEPTH_MAX_M - DEPTH_MIN_M)
    enc = VALID_FLOOR + t * (1.0 - VALID_FLOOR)
    out = np.round(enc * 255.0).astype(np.uint8)
    out[~valid] = 0                       # sentinel
    return out


def render_depth(shape, boxes_depths, rng) -> np.ndarray:
    """Depth map: a background plane with objects standing in front of it."""
    h, w = shape
    # Background tilted slightly, as a real table surface would be.
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    background = 1.20 + 0.25 * (yy / h) + rng.uniform(-0.1, 0.1)
    depth = background.copy()

    for (x, y, bw, bh), z in boxes_depths:
        depth[y:y + bh, x:x + bw] = z

    # Sensor noise, per the measured law: sigma grows with z^2.
    sigma_m = DEPTH_NOISE_K_MM * 1e-3 * depth ** 2
    depth = depth + np.random.normal(0.0, 1.0, depth.shape).astype(np.float32) * sigma_m

    valid = np.random.random(depth.shape) > INVALID_FRACTION
    # Real holes CLUSTER at depth discontinuities rather than scattering, so
    # thicken the invalid mask along object edges.
    edges = cv2.Canny((np.clip(depth, 0, 3) / 3 * 255).astype(np.uint8), 40, 120)
    edges = cv2.dilate(edges, np.ones((3, 3), np.uint8))
    valid &= ~((edges > 0) & (np.random.random(depth.shape) < 0.5))
    return depth, valid


def random_background(rng: random.Random, width: int, height: int) -> np.ndarray:
    """A cluttered, randomised background.

    Domain randomisation's job is to make the detector rely on object shape and
    colour rather than on scene statistics, so the background deliberately has
    no consistent structure to latch onto.
    """
    base = np.full((height, width, 3),
                   [rng.randint(30, 220) for _ in range(3)], dtype=np.uint8)
    for _ in range(rng.randint(4, 14)):
        colour = tuple(rng.randint(0, 255) for _ in range(3))
        x, y = rng.randint(0, width), rng.randint(0, height)
        if rng.random() < 0.5:
            cv2.rectangle(base, (x, y),
                          (x + rng.randint(20, 200), y + rng.randint(20, 200)), colour, -1)
        else:
            cv2.circle(base, (x, y), rng.randint(10, 90), colour, -1)
    return base


LOW_CONTRAST = False   # set by --low-contrast; see main()


def y0_hint(h):
    return 0


def draw_object(canvas: np.ndarray, cls_index: int, rng: random.Random):
    """Draw one instance; return its (x, y, w, h) box and the depth it sits at.

    Depth correlates with apparent size, as it must physically: a nearer object
    subtends more pixels. Randomising the two independently would let the
    network learn a size/depth relationship that no real scene exhibits.
    """
    h, w = canvas.shape[:2]
    name = CLASSES[cls_index]

    if name == "mustard_bottle":       # tall yellow body
        bw, bh = rng.randint(30, 70), rng.randint(70, 150)
        colour = (rng.randint(0, 40), rng.randint(170, 230), rng.randint(200, 255))
    elif name == "red_cube":           # compact red square
        side = rng.randint(35, 90)
        bw = bh = side
        colour = (rng.randint(0, 50), rng.randint(0, 60), rng.randint(170, 255))
    else:                              # bowl: wide, shallow, desaturated
        bw, bh = rng.randint(70, 150), rng.randint(30, 65)
        grey = rng.randint(120, 210)
        colour = (grey, grey, rng.randint(120, 210))

    if LOW_CONTRAST:
        # Collapse the object's colour toward the local background so RGB alone
        # is close to uninformative, while depth still separates it cleanly.
        # This is the regime AGENTS.md predicts depth should help in --
        # "textureless / low-contrast objects" -- and it is the only way to tell
        # whether the network USES the depth channel or merely tolerates it.
        h, w = canvas.shape[:2]
        local = canvas[max(0, y0_hint(h)):, :].mean(axis=(0, 1)) if False else canvas.mean(axis=(0, 1))
        jitter = rng.uniform(-14, 14)
        colour = tuple(int(np.clip(local[c] + jitter, 0, 255)) for c in range(3))

    x = rng.randint(0, max(1, w - bw - 1))
    y = rng.randint(0, max(1, h - bh - 1))


    if name == "bowl":
        cv2.ellipse(canvas, (x + bw // 2, y + bh // 2), (bw // 2, bh // 2),
                    0, 0, 360, colour, -1)
    else:
        cv2.rectangle(canvas, (x, y), (x + bw, y + bh), colour, -1)
        # A darker cap/edge gives the bottle a second cue beyond raw colour.
        if name == "mustard_bottle":
            cap = max(4, bh // 8)
            cv2.rectangle(canvas, (x + bw // 4, y), (x + 3 * bw // 4, y + cap),
                          tuple(int(c * 0.5) for c in colour), -1)

    # Larger on screen => nearer. Maps the drawn size onto the working range.
    size_frac = (bw * bh) / float(canvas.shape[0] * canvas.shape[1])
    z = float(np.interp(size_frac, [0.002, 0.05], [1.05, 0.35]))
    z += rng.uniform(-0.05, 0.05)
    return (x, y, bw, bh), z


def randomise_appearance(image: np.ndarray, rng: random.Random) -> np.ndarray:
    """Photometric randomisation: brightness, contrast, blur, sensor noise."""
    out = image.astype(np.float32)
    out = out * rng.uniform(0.6, 1.4) + rng.uniform(-30, 30)   # contrast, brightness
    if rng.random() < 0.35:
        k = rng.choice([3, 5])
        out = cv2.GaussianBlur(out, (k, k), 0)
    if rng.random() < 0.5:
        out += np.random.normal(0, rng.uniform(2, 12), out.shape)
    return np.clip(out, 0, 255).astype(np.uint8)


def generate_split(out_dir: Path, split: str, count: int, width: int, height: int,
                   seed: int) -> None:
    rng = random.Random(seed)
    np.random.seed(seed)

    images = out_dir / "images" / split
    labels = out_dir / "labels" / split
    images.mkdir(parents=True, exist_ok=True)
    labels.mkdir(parents=True, exist_ok=True)

    for i in range(count):
        canvas = random_background(rng, width, height)
        rows, boxes_depths = [], []
        for _ in range(rng.randint(1, 4)):
            cls_index = rng.randrange(len(CLASSES))
            (x, y, bw, bh), z = draw_object(canvas, cls_index, rng)
            boxes_depths.append(((x, y, bw, bh), z))
            rows.append(f"{cls_index} {(x + bw / 2) / width:.6f} {(y + bh / 2) / height:.6f} "
                        f"{bw / width:.6f} {bh / height:.6f}")
        canvas = randomise_appearance(canvas, rng)

        depth_m, valid = render_depth((height, width), boxes_depths, rng)
        depth_u8 = encode_depth(depth_m, valid)

        stem = f"{split}_{i:06d}"
        cv2.imwrite(str(images / f"{stem}.png"), canvas)          # BGR, for viewing
        # RGB + depth, in the order the network trains on and the C++ runtime
        # feeds. cv2 works in BGR, so flip before caching.
        rgbd = np.dstack([canvas[:, :, ::-1], depth_u8]).astype(np.uint8)
        np.save(images / f"{stem}.npy", rgbd)
        (labels / f"{stem}.txt").write_text("\n".join(rows) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--train", type=int, default=800)
    parser.add_argument("--val", type=int, default=200)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--seed", type=int, default=20260820)
    parser.add_argument("--low-contrast", action="store_true",
                        help="draw objects at near-background colour so RGB is almost "
                             "uninformative and depth carries the signal. Use this to "
                             "test whether the RGB-D network actually USES depth.")
    args = parser.parse_args()

    global LOW_CONTRAST
    LOW_CONTRAST = args.low_contrast
    if LOW_CONTRAST:
        print("LOW-CONTRAST mode: objects blend into the background in RGB")

    args.out.mkdir(parents=True, exist_ok=True)
    generate_split(args.out, "train", args.train, args.width, args.height, args.seed)
    generate_split(args.out, "val", args.val, args.width, args.height, args.seed + 1)

    # Ultralytics resolves `path` and then train/val relative to it.
    (args.out / "data.yaml").write_text(yaml.safe_dump({
        "path": str(args.out.resolve()),
        "train": "images/train",
        "val": "images/val",
        # YOLODataset reads this: `channels=self.data.get("channels", 3)`. It is
        # what lets BaseDataset.load_image accept the 4-channel .npy cache
        # instead of deleting it as stale.
        "channels": 4,
        "names": {i: name for i, name in enumerate(CLASSES)},
    }, sort_keys=False))

    print(f"wrote {args.train} train / {args.val} val frames to {args.out}")
    print("NOTE: procedural stand-in data. Not reportable as a detection result.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
