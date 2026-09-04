#!/usr/bin/env python3
"""Python reference detections, for the C++ parity check.

AGENTS.md §3 acceptance: "C++ inference output matches the Python
reference within float tolerance."

The reference is deliberately **ultralytics' own** predict path, not a
re-implementation of the same preprocessing in Python. Comparing my C++ against
my own Python would only prove the two agree with each other; comparing it
against the canonical implementation is what actually validates the letterbox
inversion, the YOLOv8 decode, and the NMS in SystemCode/detector/.

Print reference detections:
    python parity_reference.py --model best.onnx --image frame.png

Compare against the C++ tool's output:
    build/detector/detector_parity config/detector.yaml frame.png cpu > cpp.txt
    python parity_reference.py --model best.onnx --image frame.png --compare cpp.txt
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def reference_detections(model_path: Path, image_path: Path, imgsz: int, conf: float,
                         iou: float, npy_path: Path | None = None):
    import numpy as np
    from ultralytics import YOLO

    model = YOLO(str(model_path), task="detect")
    # For an RGB-D model the source is the 4-channel array itself: ultralytics
    # has no 4-channel image reader, and `Format._format_img` only reverses
    # channel order when shape[0] == 3, so a 4-channel array passes through in
    # exactly the order it is stored -- which must match what the C++ feeds.
    source = np.load(npy_path) if npy_path is not None else str(image_path)
    # device="cpu": the parity check is about correctness, and CPU is the only
    # execution path that is bit-reproducible across machines.
    results = model.predict(source=source, imgsz=imgsz, conf=conf, iou=iou,
                            device="cpu", verbose=False)
    out = []
    for result in results:
        names = result.names
        for box in result.boxes:
            x0, y0, x1, y1 = (float(v) for v in box.xyxy[0].tolist())
            out.append({
                "cls": names[int(box.cls.item())],
                "score": float(box.conf.item()),
                "x": int(round(x0)),
                "y": int(round(y0)),
                "w": int(round(x1 - x0)),
                "h": int(round(y1 - y0)),
            })
    # Match the C++ tool's ordering: score descending, then class, then position.
    out.sort(key=lambda d: (-d["score"], d["cls"], d["x"], d["y"]))
    return out


def parse_cpp(path: Path):
    out = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        out.append({"cls": parts[0], "score": float(parts[1]), "x": int(parts[2]),
                    "y": int(parts[3]), "w": int(parts[4]), "h": int(parts[5])})
    out.sort(key=lambda d: (-d["score"], d["cls"], d["x"], d["y"]))
    return out


def compare(reference, cpp, score_tol: float, box_tol: int) -> int:
    """Match detections by POSITION, not by sorted index.

    Two implementations may legitimately order near-equal-scoring detections
    differently -- NMS tie-breaking is not part of the contract. Pairing by
    sorted index then reports a swap as two mismatches, which is a harness
    artefact rather than a real disagreement.
    """
    print(f"reference detections: {len(reference)}")
    print(f"C++ detections      : {len(cpp)}")
    if len(reference) != len(cpp):
        print("FAIL: detection counts differ")
        for label, rows in (("reference", reference), ("C++", cpp)):
            for d in rows:
                print(f"  {label:9s} {d['cls']:16s} {d['score']:.4f} "
                      f"[{d['x']} {d['y']} {d['w']} {d['h']}]")
        return 1

    def centre(d):
        return (d["x"] + d["w"] / 2.0, d["y"] + d["h"] / 2.0)

    unmatched = list(cpp)
    failures = 0
    for r in reference:
        rx, ry = centre(r)
        # Nearest same-class C++ detection by centre distance.
        candidates = [c for c in unmatched if c["cls"] == r["cls"]]
        if not candidates:
            print(f"  [MISMATCH] no C++ detection of class {r['cls']}")
            failures += 1
            continue
        c = min(candidates, key=lambda c: (centre(c)[0] - rx) ** 2 + (centre(c)[1] - ry) ** 2)
        unmatched.remove(c)

        problems = []
        if abs(r["score"] - c["score"]) > score_tol:
            problems.append(f"score {r['score']:.6f} vs {c['score']:.6f}")
        for key in ("x", "y", "w", "h"):
            if abs(r[key] - c[key]) > box_tol:
                problems.append(f"{key} {r[key]} vs {c[key]}")
        status = "ok" if not problems else "MISMATCH"
        print(f"  [{status}] {r['cls']} {r['score']:.4f} "
              f"[{r['x']} {r['y']} {r['w']} {r['h']}]"
              + ("  <- " + "; ".join(problems) if problems else ""))
        failures += bool(problems)

    if failures:
        print(f"\nFAIL: {failures} of {len(reference)} detections differ beyond tolerance "
              f"(score {score_tol}, box {box_tol} px)")
        return 1
    print(f"\nPASS: {len(reference)} detections match within tolerance "
          f"(score {score_tol}, box {box_tol} px)")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", type=Path, required=True, help="the .onnx under test")
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--npy", type=Path, default=None,
                        help="4-channel RGB-D array for an RGB-D model; without it the "
                             "reference runs RGB-only and cannot match a 4-channel C++ path")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25,
                        help="must match postprocess.score_threshold")
    parser.add_argument("--iou", type=float, default=0.45,
                        help="must match postprocess.iou_threshold")
    parser.add_argument("--compare", type=Path, default=None,
                        help="output of build/detector/detector_parity to diff against")
    # Boxes are reported as integer pixels after the letterbox inversion, so a
    # 2 px allowance covers rounding-order differences without hiding a genuine
    # coordinate-mapping bug, which would be off by far more.
    parser.add_argument("--box-tol", type=int, default=2)
    # RGB-D needs a looser score tolerance than RGB, and the old 1e-3 default was
    # tighter than the effect this comment already described. The training cache
    # stores depth as uint8 (ultralytics divides by 255) while the C++ runtime
    # encodes depth as an exact float, so the two paths see depth channels up to
    # ~1/510 apart. That is a property of the CACHE, not of either inference
    # path, and it propagates to small score differences.
    #
    # MEASURED 2026-09-05 over 40 val frames / 109 paired detections, C++ CPU EP
    # against ultralytics CPU, both on ONNX Runtime 1.29.0:
    #
    #     unpaired detections   0        (identical objects and classes, every frame)
    #     box delta             max 1 px (tolerance 2 px)
    #     score delta           median 4.2e-5, p95 1.16e-3, max 2.27e-3
    #
    # 7 of 109 exceeded the old 1e-3. The largest spread is red_cube (max
    # 2.27e-3), the smallest object -- fewest pixels, so a per-pixel depth
    # quantisation difference moves its logit most. 3e-3 clears the measured
    # maximum with margin while staying orders of magnitude below what a real
    # defect looks like: a decode-layout or letterbox-inversion bug shifts
    # scores by O(0.1) and boxes by tens of pixels, neither of which this hides.
    #
    # The cleaner fix is to remove the confound rather than tolerate it -- feed
    # the reference the same float depth the C++ encodes, instead of the uint8
    # cache -- but that means handing ultralytics a pre-normalised float array,
    # whose predict path would rescale it again. Left as a known limitation.
    parser.add_argument("--score-tol", type=float, default=3e-3)
    args = parser.parse_args()

    reference = reference_detections(args.model, args.image, args.imgsz, args.conf, args.iou,
                                     args.npy)

    if args.compare is None:
        print("# reference (ultralytics)")
        print("# cls score x y w h")
        for d in reference:
            print(f"{d['cls']} {d['score']:.6f} {d['x']} {d['y']} {d['w']} {d['h']}")
        return 0

    return compare(reference, parse_cpp(args.compare), args.score_tol, args.box_tol)


if __name__ == "__main__":
    sys.exit(main())
