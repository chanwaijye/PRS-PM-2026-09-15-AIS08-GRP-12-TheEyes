#!/usr/bin/env python3
"""YOLOv8 fine-tune from COCO weights — RGB-D 4-channel early fusion.

Transfer learning from COCO rather than training from scratch is the whole point
of the supervised half of PRS category (1): the backbone already knows generic
object features, so the synthetic dataset only has to teach the new classes.

RGB-D: the input stem is widened from 3 to 4 channels (RGB + metric depth).
`channels: 4` in data.yaml is enough to make ultralytics build a 4-channel model
and load the 4-channel .npy cache — but NOT enough to initialise the new channel
well. Measured on ultralytics 8.4.123: it copies the pretrained RGB channels and
leaves the depth channel at RANDOM init (|depth - mean(RGB)| up to 0.51).

AGENTS.md calls for inflation instead, and `--inflate-stem` (default on) does it:
the depth channel starts as the mean of the pretrained RGB filters, so it begins
as a working edge/texture detector rather than noise. Random init on one channel
destabilises early training, and the damage is worst exactly where transfer
learning is supposed to help.

    python train.py --data <dataset>/data.yaml --epochs 40 --model yolov8n.pt
"""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data", type=Path, required=True, help="dataset data.yaml")
    parser.add_argument("--model", default="yolov8n.pt",
                        help="COCO-pretrained checkpoint to fine-tune from")
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--imgsz", type=int, default=640,
                        help="must match config/detector.yaml model.input_size")
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--project", type=Path, default=Path("runs/detector"))
    parser.add_argument("--name", default="finetune")
    parser.add_argument("--device", default="0")
    parser.add_argument("--seed", type=int, default=20260820)
    parser.add_argument("--patience", type=int, default=15)
    parser.add_argument("--fraction", type=float, default=1.0,
                        help="fraction of the TRAINING split to use; validation is "
                             "unaffected. This is how the §7 dataset-size ablation "
                             "is run -- 0.125 and 0.5 of the 40k train split give "
                             "the 5k and 20k points against an identical val set "
                             "and one generator config, rather than three separate "
                             "generations that would also differ in scene sampling")
    parser.add_argument("--no-inflate-stem", action="store_true",
                        help="leave the depth channel at random init (the RGB-only "
                             "control for a stem-initialisation ablation)")
    args = parser.parse_args()

    import torch
    import yaml as _yaml
    from ultralytics import YOLO

    model = YOLO(args.model)

    channels = int(_yaml.safe_load(Path(args.data).read_text()).get("channels", 3))
    print(f"dataset declares {channels} input channels")

    if channels == 4 and not args.no_inflate_stem:
        # Build the 4-channel model up front so the stem can be seeded before a
        # single gradient step. Done after train() starts, the random channel has
        # already influenced the first updates.
        from ultralytics.nn.tasks import DetectionModel

        pretrained = model.model
        names = _yaml.safe_load(Path(args.data).read_text())["names"]
        wide = DetectionModel(args.model.replace(".pt", ".yaml"), ch=4,
                              nc=len(names), verbose=False)
        wide.load(pretrained)

        stem = wide.model[0].conv.weight.data
        rgb_mean = pretrained.model[0].conv.weight.data[:, :3].mean(dim=1)
        before = (stem[:, 3] - rgb_mean).abs().max().item()
        stem[:, 3] = rgb_mean
        print(f"inflated depth stem from mean(RGB): |delta| {before:.4f} -> "
              f"{(stem[:, 3] - rgb_mean).abs().max().item():.4f}")
        # NOTE: this raises total stem activation by ~1/3, since a fourth
        # comparable channel now contributes. BatchNorm immediately after the
        # conv absorbs that, which is why the weights are not rescaled.
        wide.nc = len(names)
        wide.names = names if isinstance(names, dict) else dict(enumerate(names))
        model.model = wide
    results = model.train(
        data=str(args.data),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        project=str(args.project),
        name=args.name,
        device=args.device,
        seed=args.seed,
        patience=args.patience,
        fraction=args.fraction,
        # cache=False (the default) is load-bearing here, not incidental: the
        # 4-channel .npy files are 1.63 MB each and the training split is 65 GB,
        # so any in-RAM cache would exhaust memory rather than speed anything up.
        deterministic=True,
        exist_ok=True,
        verbose=True,
    )

    weights = Path(results.save_dir) / "weights" / "best.pt"
    print(f"\nbest weights: {weights}")
    print("next: export_onnx.py --weights <best.pt>, then detector_parity to check "
          "the C++ path against parity_reference.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
