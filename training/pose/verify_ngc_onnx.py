#!/usr/bin/env python3
"""Verify NVIDIA's NGC FoundationPose ONNX files against our own checkpoints.

`isaac_ros_foundationpose` publishes pre-exported ONNX for the refiner and
scorer. Whether they are usable here is not a documentation question -- it is a
weights question, and the answer differs between the two files:

    refine_model.onnx  IS our refiner (2023-10-28-18-33-37), bit-identical
    score_model.onnx   is NOT our scorer (we ship 2024-01-11-20-02-45)

This script is the evidence behind docs/TRT_PLAN.md 4.2. It downloads nothing;
point it at files fetched with the wget lines in that section.

    ~/venvs/the-eyes-r1/bin/python training/pose/verify_ngc_onnx.py \
        --ngc-dir /path/to/downloads --foundationpose-root ~/src/FoundationPose

Exit 0 means the refiner ONNX is a verified drop-in and the scorer ONNX is
correctly identified as foreign. Note the layout difference: these graphs take
NHWC (B,160,160,6); our tensors are NCHW (B,6,160,160).
"""

from __future__ import annotations

import argparse
import hashlib
import sys
from pathlib import Path

import numpy as np

# The refiner ONNX we have verified. The scorer ONNX is a different checkpoint
# and must never be substituted for ours -- it chooses the winning hypothesis,
# so it would change every registered pose.
REFINER_SHA256 = "dcc695a19c4bcfe5e1d909a22d8f652d8ec8bab1e19bd1544c6b45f2d3595cf7"
FOREIGN_SCORER_SHA256 = "0bf1026c0db7320ebf9a548ecf0d3c810c8dbd377948630bd3e5af1d49440503"

REFINER_RUN = "2023-10-28-18-33-37"
SCORER_RUN = "2024-01-11-20-02-45"

# Agreement between NVIDIA's refiner graph and our checkpoint, float32 round-off.
TRANS_TOL = 1e-6
ROT_TOL = 1e-5


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_state_dict(root: Path, run: str):
    import torch

    ckpt = torch.load(root / "weights" / run / "model_best.pth",
                      map_location="cpu", weights_only=False)
    if isinstance(ckpt, dict) and "model" in ckpt:
        ckpt = ckpt["model"]
    return {k: v.numpy() for k, v in ckpt.items() if hasattr(v, "numpy")}


def compare_named_tensors(onnx_path: Path, state: dict) -> tuple[int, float]:
    """Deviation over tensors the exporter left under their original names.

    Conv weights are BN-folded and the MatMul weights are transposed, so those
    are not name-matched -- the biases, out_proj and norm tensors are, and they
    are enough to identify the checkpoint.
    """
    import onnx
    from onnx import numpy_helper

    model = onnx.load(str(onnx_path))
    inits = {i.name: numpy_helper.to_array(i) for i in model.graph.initializer}
    shared = [k for k in inits if k in state]
    worst = max((float(np.abs(inits[k] - state[k]).max()) for k in shared),
                default=float("nan"))
    return len(shared), worst


def refiner_parity(onnx_path: Path, fp_root: Path, batch: int) -> tuple[float, float]:
    """Run NVIDIA's graph and our RefineNet on the same input; return max deltas."""
    import torch
    from omegaconf import OmegaConf
    import onnxruntime as ort

    sys.path[:0] = [str(fp_root), str(fp_root / "learning" / "models")]
    from learning.models.refine_network import RefineNet  # noqa: E402

    cfg = OmegaConf.load(fp_root / "weights" / REFINER_RUN / "config.yml")
    # predict_pose_refine.py applies these same backward-compatible defaults.
    for key, value in dict(use_normal=False, use_mask=False, use_BN=False, c_in=4,
                           crop_ratio=1.2, n_view=1, trans_rep="tracknet",
                           rot_rep="axis_angle", zfar=3, normalize_xyz=False,
                           normal_uint8=False).items():
        if key not in cfg:
            cfg[key] = value

    net = RefineNet(cfg=cfg, c_in=cfg["c_in"])
    net.load_state_dict({k: torch.from_numpy(v)
                         for k, v in load_state_dict(fp_root, REFINER_RUN).items()})
    net.eval()

    rng = np.random.default_rng(0)
    a = rng.random((batch, 6, 160, 160), dtype=np.float32)
    b = rng.random((batch, 6, 160, 160), dtype=np.float32)
    with torch.inference_mode():
        out = net(torch.from_numpy(a), torch.from_numpy(b))
    trans, rot = out["trans"].numpy(), out["rot"].numpy()

    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    # NHWC: the graph carries its own transposes to NCHW for the convolutions.
    feeds = {"input1": np.ascontiguousarray(a.transpose(0, 2, 3, 1)),
             "input2": np.ascontiguousarray(b.transpose(0, 2, 3, 1))}
    o1, o2 = session.run(["output1", "output2"], feeds)
    return float(np.abs(o1 - trans).max()), float(np.abs(o2 - rot).max())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ngc-dir", type=Path, required=True,
                        help="directory holding refine_model.onnx and score_model.onnx")
    parser.add_argument("--foundationpose-root", type=Path,
                        default=Path.home() / "src" / "FoundationPose")
    parser.add_argument("--batch", type=int, default=2)
    args = parser.parse_args()

    refiner = args.ngc_dir / "refine_model.onnx"
    scorer = args.ngc_dir / "score_model.onnx"
    for path in (refiner, scorer):
        if not path.is_file():
            print(f"FAIL missing {path} -- see docs/TRT_PLAN.md 4.1 for the wget lines")
            return 2

    failures = []

    digest = sha256(refiner)
    ok = digest == REFINER_SHA256
    print(f"{'PASS' if ok else 'FAIL'} refine_model.onnx sha256 {digest[:16]}...")
    if not ok:
        failures.append("refiner hash is not the verified 1.0.1_onnx artifact")

    digest = sha256(scorer)
    print(f"INFO score_model.onnx  sha256 {digest[:16]}..."
          f"{' (the known-foreign scorer)' if digest == FOREIGN_SCORER_SHA256 else ''}")

    shared, worst = compare_named_tensors(refiner, load_state_dict(args.foundationpose_root,
                                                                   REFINER_RUN))
    ok = shared > 0 and worst == 0.0
    print(f"{'PASS' if ok else 'FAIL'} refiner: {shared} name-matched tensors, "
          f"worst deviation {worst:g} (expect 0)")
    if not ok:
        failures.append("refiner ONNX weights are not bit-identical to our checkpoint")

    shared, worst = compare_named_tensors(scorer, load_state_dict(args.foundationpose_root,
                                                                  SCORER_RUN))
    ok = shared > 0 and worst > 1e-3
    print(f"{'PASS' if ok else 'FAIL'} scorer: {shared} name-matched tensors, "
          f"worst deviation {worst:g} (expect >> 0 -- a DIFFERENT checkpoint)")
    if not ok:
        failures.append("scorer ONNX unexpectedly resembles our checkpoint; re-check "
                        "before assuming either file is safe to use")

    d_trans, d_rot = refiner_parity(refiner, args.foundationpose_root, args.batch)
    ok = d_trans < TRANS_TOL and d_rot < ROT_TOL
    print(f"{'PASS' if ok else 'FAIL'} refiner end to end: output1-trans {d_trans:.3g} "
          f"(< {TRANS_TOL:g}), output2-rot {d_rot:.3g} (< {ROT_TOL:g})")
    if not ok:
        failures.append("refiner ONNX does not reproduce our RefineNet output")

    print()
    if failures:
        for line in failures:
            print(f"FAILED: {line}")
        return 1
    print("All checks passed: refine_model.onnx is a verified drop-in for our refiner "
          "(output1=trans, output2=rot, NHWC input); score_model.onnx is foreign.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
