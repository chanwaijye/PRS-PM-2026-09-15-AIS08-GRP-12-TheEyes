#!/usr/bin/env python3
"""Export a trained YOLOv8 checkpoint to ONNX for the C++ runtime.

opset and dynamic-axis choices matter downstream: the C++ detector assumes a
fixed square input and a rank-3 [1, 4+nc, anchors] output, and validates the
channel count against config/detector.yaml's class list at load time.
"""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--opset", type=int, default=17)
    parser.add_argument("--out", type=Path, default=None,
                        help="destination .onnx (default: alongside the weights)")
    parser.add_argument("--simplify", action="store_true",
                        help="run onnx-simplifier; smaller graph, same semantics")
    args = parser.parse_args()

    from ultralytics import YOLO

    model = YOLO(str(args.weights))
    # dynamic=False: a fixed input shape lets ORT and TensorRT specialise, and
    # the C++ side letterboxes to exactly this size anyway.
    exported = model.export(format="onnx", imgsz=args.imgsz, opset=args.opset,
                            dynamic=False, simplify=args.simplify)
    exported = Path(exported)

    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        exported.replace(args.out)
        exported = args.out

    print(f"exported: {exported}")
    print(f"class names: {model.names}")
    print("Set model.onnx_path and model.class_names in config/detector.yaml to match. "
          "The C++ loader refuses a model whose head disagrees with that class list.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
