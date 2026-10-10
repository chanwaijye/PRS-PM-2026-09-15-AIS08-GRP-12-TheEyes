#!/usr/bin/env python3
"""A training list that mixes real frames with synthetic ones, for fine-tuning.

Fine-tuning on the real layouts alone would be 360 frames of ten arrangements
on one table: enough to fit, and enough to forget everything the synthetic set
taught about poses, lighting and occlusion the layouts do not contain. So each
epoch sees the real frames (repeated --real-repeat times) beside a fixed random
sample of the synthetic training split.

    python mixed_data.py --real /opt/the-eyes-data/real_v1 \\
        --synth /opt/the-eyes-data/synth50k_v2 --synth-count 2000 --real-repeat 3

Writes <real>/train_mixed.txt (image paths; labels are found beside them by
ultralytics' images/ -> labels/ rule, and the 4-channel .npy beside each PNG is
what is loaded) and <real>/data_mixed.yaml, whose val split is the real val.
"""

from __future__ import annotations

import argparse
import random
from pathlib import Path

import yaml


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--real", type=Path, required=True)
    ap.add_argument("--synth", type=Path, required=True)
    ap.add_argument("--synth-count", type=int, default=2000)
    ap.add_argument("--real-repeat", type=int, default=3)
    ap.add_argument("--seed", type=int, default=20261011)
    args = ap.parse_args()

    real = sorted((args.real / "images/train").glob("*.png"))
    synth = sorted((args.synth / "images/train").glob("*.png"))
    if not real or len(synth) < args.synth_count:
        raise SystemExit(f"{len(real)} real and {len(synth)} synthetic training frames found")
    random.Random(args.seed).shuffle(synth)
    lines = [str(p.resolve()) for p in real] * args.real_repeat
    lines += [str(p.resolve()) for p in sorted(synth[:args.synth_count])]
    (args.real / "train_mixed.txt").write_text("\n".join(lines) + "\n")

    base = yaml.safe_load((args.real / "data.yaml").read_text())
    base["train"] = "train_mixed.txt"
    (args.real / "data_mixed.yaml").write_text(yaml.safe_dump(base, sort_keys=False))
    print(f"{len(real)} real frames x{args.real_repeat} + {args.synth_count} synthetic "
          f"= {len(lines)} lines -> {args.real / 'train_mixed.txt'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
