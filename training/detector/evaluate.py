#!/usr/bin/env python3
"""Detector evaluation: mAP, and the dataset-size ablation.

AGENTS.md §3 makes the ablation an explicit deliverable -- "mAP at ~5k /
20k / 50k training frames, plotted. This is the supervised-learning methodology
evidence for the report." It justifies the final dataset size empirically rather
than by assertion.

    # single model
    python evaluate.py --weights best.pt --data <dataset>/data.yaml

    # modality ablation: RGB-only vs RGB-D on the SAME data and split
    python evaluate.py --modality --data <dataset>/data.yaml --epochs 25

    # dataset-size ablation: trains one model per size, evaluates each on the
    # SAME held-out val split
    python evaluate.py --ablation --data <dataset>/data.yaml \
        --sizes 5000 20000 50000 --epochs 60
"""

from __future__ import annotations

import argparse
import json
import random
import shutil
from pathlib import Path


def evaluate_one(weights: Path, data: Path, imgsz: int, device: str) -> dict:
    from ultralytics import YOLO

    metrics = YOLO(str(weights)).val(data=str(data), imgsz=imgsz, device=device, verbose=False)
    box = metrics.box
    return {
        "mAP50": float(box.map50),
        "mAP50_95": float(box.map),
        "precision": float(box.mp),
        "recall": float(box.mr),
        "per_class_mAP50": {name: float(v) for name, v in
                            zip(metrics.names.values(), box.ap50)} if box.ap50 is not None else {},
    }


def subset_dataset(source: Path, dest: Path, n_train: int, seed: int) -> Path:
    """Build a dataset whose train split is a random n_train subset.

    The val split is COPIED UNCHANGED from the source. Sub-sampling val too
    would make each ablation point measure a different test set, and the curve
    would confound dataset size with evaluation noise.
    """
    import yaml

    if dest.exists():
        shutil.rmtree(dest)
    (dest / "images" / "train").mkdir(parents=True)
    (dest / "labels" / "train").mkdir(parents=True)

    stems = sorted(p.stem for p in (source / "images" / "train").glob("*.png"))
    if n_train > len(stems):
        raise ValueError(f"requested {n_train} train frames but only {len(stems)} exist")
    chosen = random.Random(seed).sample(stems, n_train)

    for stem in chosen:
        shutil.copy2(source / "images" / "train" / f"{stem}.png",
                     dest / "images" / "train" / f"{stem}.png")
        shutil.copy2(source / "labels" / "train" / f"{stem}.txt",
                     dest / "labels" / "train" / f"{stem}.txt")

    for split_dir in ("images", "labels"):
        shutil.copytree(source / split_dir / "val", dest / split_dir / "val")

    source_cfg = yaml.safe_load((source / "data.yaml").read_text())
    (dest / "data.yaml").write_text(yaml.safe_dump({
        "path": str(dest.resolve()),
        "train": "images/train",
        "val": "images/val",
        "names": source_cfg["names"],
    }, sort_keys=False))
    return dest / "data.yaml"


def run_ablation(args) -> dict:
    from ultralytics import YOLO

    source = args.data.parent
    rows = []
    for n in args.sizes:
        work = args.work / f"subset_{n}"
        data_yaml = subset_dataset(source, work, n, args.seed)

        model = YOLO(args.model)
        result = model.train(data=str(data_yaml), epochs=args.epochs, imgsz=args.imgsz,
                             batch=args.batch, project=str(args.work / "runs"),
                             name=f"n{n}", device=args.device, seed=args.seed,
                             deterministic=True, exist_ok=True, verbose=False)
        weights = Path(result.save_dir) / "weights" / "best.pt"
        metrics = evaluate_one(weights, data_yaml, args.imgsz, args.device)
        metrics["train_frames"] = n
        metrics["weights"] = str(weights)
        rows.append(metrics)
        print(f"  n={n:>6}  mAP50={metrics['mAP50']:.4f}  mAP50-95={metrics['mAP50_95']:.4f}")

    return {"ablation": rows}


def run_modality_ablation(args) -> dict:
    """RGB-only vs RGB-D, same images, same split, same schedule.

    AGENTS.md makes this a deliverable: "Textureless/low-contrast classes are the
    ones expected to move; if they do not, that is a reportable negative result,
    not a failure to hide."

    The two runs differ ONLY in `channels`. At 3 the dataloader reads the RGB
    PNG; at 4 it reads the 4-channel .npy sibling. Same frames either way, so
    any difference is attributable to the depth channel and nothing else.
    """
    import yaml
    from ultralytics import YOLO

    source = yaml.safe_load(args.data.read_text())
    rows = []
    for channels in (3, 4):
        cfg = dict(source, channels=channels)
        cfg_path = args.work / f"data_ch{channels}.yaml"
        args.work.mkdir(parents=True, exist_ok=True)
        cfg_path.write_text(yaml.safe_dump(cfg, sort_keys=False))

        model = YOLO(args.model)
        if channels == 4:
            # Same stem inflation train.py applies -- otherwise the RGB-D arm is
            # handicapped by a randomly initialised depth channel and the
            # comparison measures initialisation, not modality.
            from ultralytics.nn.tasks import DetectionModel
            names = source["names"]
            wide = DetectionModel(args.model.replace(".pt", ".yaml"), ch=4,
                                  nc=len(names), verbose=False)
            wide.load(model.model)
            stem = wide.model[0].conv.weight.data
            stem[:, 3] = model.model.model[0].conv.weight.data[:, :3].mean(dim=1)
            wide.nc = len(names)
            wide.names = names if isinstance(names, dict) else dict(enumerate(names))
            model.model = wide

        result = model.train(data=str(cfg_path), epochs=args.epochs, imgsz=args.imgsz,
                             batch=args.batch, project=str(args.work / "runs"),
                             name=f"ch{channels}", device=args.device, seed=args.seed,
                             deterministic=True, exist_ok=True, verbose=False, plots=False)
        weights = Path(result.save_dir) / "weights" / "best.pt"
        metrics = evaluate_one(weights, cfg_path, args.imgsz, args.device)
        metrics["modality"] = "RGB" if channels == 3 else "RGB-D"
        metrics["channels"] = channels
        rows.append(metrics)
        print(f"  {metrics['modality']:6s} mAP50={metrics['mAP50']:.4f} "
              f"mAP50-95={metrics['mAP50_95']:.4f}")

    rgb, rgbd = rows[0], rows[1]
    delta = rgbd["mAP50"] - rgb["mAP50"]
    print(f"\n  depth contribution: {delta:+.4f} mAP@0.5 "
          f"({'RGB-D better' if delta > 0 else 'no gain from depth'})")
    print("  per-class mAP@0.5:")
    for cls in rgb.get("per_class_mAP50", {}):
        a, b = rgb["per_class_mAP50"].get(cls, 0), rgbd["per_class_mAP50"].get(cls, 0)
        print(f"    {cls:18s} RGB {a:.4f}  RGB-D {b:.4f}  {b-a:+.4f}")
    return {"modality_ablation": rows, "delta_mAP50": delta}


def plot(rows, out_path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    sizes = [r["train_frames"] for r in rows]
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.plot(sizes, [r["mAP50"] for r in rows], marker="o", label="mAP@0.5")
    ax.plot(sizes, [r["mAP50_95"] for r in rows], marker="s", label="mAP@0.5:0.95")
    ax.set_xscale("log")
    ax.set_xlabel("training frames")
    ax.set_ylabel("mAP")
    ax.set_title("Detector performance vs. training set size")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"plot: {out_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--weights", type=Path, default=None)
    parser.add_argument("--ablation", action="store_true")
    parser.add_argument("--modality", action="store_true",
                        help="RGB-only vs RGB-D on the same data (AGENTS.md deliverable)")
    parser.add_argument("--sizes", type=int, nargs="+", default=[5000, 20000, 50000])
    parser.add_argument("--model", default="yolov8n.pt")
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--device", default="0")
    parser.add_argument("--seed", type=int, default=20260820)
    parser.add_argument("--work", type=Path, default=Path("runs/ablation"))
    parser.add_argument("--out", type=Path, default=Path("runs/detector_metrics.json"))
    args = parser.parse_args()

    if args.modality:
        args.work.mkdir(parents=True, exist_ok=True)
        report = run_modality_ablation(args)
    elif args.ablation:
        args.work.mkdir(parents=True, exist_ok=True)
        report = run_ablation(args)
        plot(report["ablation"], args.out.with_suffix(".png"))
    else:
        if args.weights is None:
            parser.error("--weights is required unless --ablation is given")
        report = evaluate_one(args.weights, args.data, args.imgsz, args.device)
        print(json.dumps(report, indent=2))

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2))
    print(f"metrics: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
