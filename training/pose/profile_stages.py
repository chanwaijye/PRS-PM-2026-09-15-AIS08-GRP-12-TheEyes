#!/usr/bin/env python3
"""Phase 0 of docs/TRT_PLAN.md: apportion FoundationPose's time by stage.

The question the whole TensorRT study turns on is how much of `register` and
`track_one` is the *network forward* -- the only part an engine can accelerate
-- against the rendering, warping and normalisation around it. Arithmetic
(TRT_PLAN 2.3) bounds the refiner's share of tracking only between 14% and
100%, which decides nothing.

Method. Two passes over the same sequence:

  envelope       uninstrumented, one sync per call -- comparable to the
                 1258 ms / 17.2 ms in docs/R1_STATUS.md
  instrumented   the real code paths wrapped by monkey-patching the functions
                 make_crop_data_batch calls, each delimited by
                 torch.cuda.synchronize()

The syncs inflate the total, so the instrumented total is reported beside the
envelope and the split is read as a proportion, never as absolute milliseconds.
Nothing in FoundationPose is edited; the patches are installed at runtime.

    ~/venvs/the-eyes-r1/bin/python training/pose/profile_stages.py --frames 60
"""

from __future__ import annotations

import argparse
import collections
import contextlib
import json
import logging
import os
import sys
import time
from pathlib import Path

import numpy as np


class StageTimer:
    """CUDA-synchronised wall time, accumulated per stage and per phase."""

    def __init__(self) -> None:
        self.acc: dict[str, dict[str, float]] = collections.defaultdict(
            lambda: collections.defaultdict(float))
        self.calls: dict[str, dict[str, int]] = collections.defaultdict(
            lambda: collections.defaultdict(int))
        self.phase = "idle"
        self.enabled = False

    @contextlib.contextmanager
    def stage(self, name: str):
        if not self.enabled:
            yield
            return
        import torch
        torch.cuda.synchronize()
        start = time.perf_counter()
        try:
            yield
        finally:
            torch.cuda.synchronize()
            self.acc[self.phase][name] += time.perf_counter() - start
            self.calls[self.phase][name] += 1


def patch_callable(owner, attr: str, label: str, timer: StageTimer) -> None:
    original = getattr(owner, attr)

    def wrapper(*args, **kwargs):
        with timer.stage(label):
            return original(*args, **kwargs)

    wrapper.__name__ = getattr(original, "__name__", attr)
    setattr(owner, attr, wrapper)


def install_patches(timer: StageTimer) -> None:
    """Wrap every stage make_crop_data_batch calls, plus the two forwards.

    Both predict modules do `from Utils import *`, so the render and crop-window
    helpers are module globals in each namespace and must be patched in both.
    """
    import kornia
    import learning.training.predict_pose_refine as refine_mod
    import learning.training.predict_score as score_mod
    from learning.datasets.h5_dataset import PoseRefinePairH5Dataset, TripletH5Dataset
    from learning.models.refine_network import RefineNet
    from learning.models.score_network import ScoreNetMultiPair

    for mod in (refine_mod, score_mod):
        patch_callable(mod, "compute_crop_window_tf_batch", "crop_window", timer)
        patch_callable(mod, "nvdiffrast_render", "render", timer)

    patch_callable(kornia.geometry.transform, "warp_perspective", "warp", timer)
    patch_callable(PoseRefinePairH5Dataset, "transform_batch", "normalise", timer)
    patch_callable(TripletH5Dataset, "transform_batch", "normalise", timer)
    patch_callable(RefineNet, "forward", "net_refine", timer)
    patch_callable(ScoreNetMultiPair, "forward", "net_score", timer)

    # TRT_PLAN 7.1: once per predict() call, synchronising and dumping the
    # allocator's cached blocks back to the driver.
    import torch
    patch_callable(torch.cuda, "empty_cache", "empty_cache", timer)


def build_estimator(fp_root: Path, scene_dir: Path, debug_dir: Path):
    sys.path[:0] = [str(fp_root)]
    os.chdir(fp_root)
    from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor
    from datareader import YcbineoatReader
    import nvdiffrast.torch as dr
    import trimesh
    from Utils import set_seed

    set_seed(0)
    mesh = trimesh.load(str(scene_dir / "mesh" / "textured_simple.obj"))
    est = FoundationPose(
        model_pts=mesh.vertices, model_normals=mesh.vertex_normals, mesh=mesh,
        scorer=ScorePredictor(), refiner=PoseRefinePredictor(),
        glctx=dr.RasterizeCudaContext(), debug=0, debug_dir=str(debug_dir))
    reader = YcbineoatReader(video_dir=str(scene_dir), shorter_side=None, zfar=np.inf)
    return est, reader


def run_sequence(est, reader, frames: int, est_iter: int, track_iter: int,
                 timer: StageTimer, warmup: int, register_repeats: int = 1) -> dict:
    """Register on frame 0, then track. Returns per-call wall times in ms.

    The first registration of a process is cold -- CUDA kernel selection and
    allocator growth land on it. That is the 2.1 s the live runner saw, and it
    is a different number from steady-state registration, so both are kept.
    """
    import torch

    def timed(fn) -> float:
        torch.cuda.synchronize()
        start = time.perf_counter()
        fn()
        torch.cuda.synchronize()
        return (time.perf_counter() - start) * 1e3

    mask = reader.get_mask(0).astype(bool)
    color0, depth0 = reader.get_color(0), reader.get_depth(0)
    register_all = []
    for r in range(register_repeats):
        # Only the last registration contributes to the stage accumulator.
        timer.phase = "register" if r == register_repeats - 1 else "register_cold"
        register_all.append(timed(lambda: est.register(
            K=reader.K, rgb=color0, depth=depth0, ob_mask=mask, iteration=est_iter)))
    register_ms = register_all[-1]

    track_ms: list[float] = []
    timer.phase = "track_warmup"
    limit = min(frames, len(reader.color_files) - 1)
    for i in range(1, limit + 1):
        if i == warmup + 1:
            timer.phase = "track"
        color, depth = reader.get_color(i), reader.get_depth(i)
        dt = timed(lambda: est.track_one(rgb=color, depth=depth, K=reader.K,
                                         iteration=track_iter))
        if i > warmup:
            track_ms.append(dt)
    timer.phase = "idle"
    return {"register_ms": register_ms, "register_all_ms": register_all,
            "track_ms": track_ms}


def summarise(track_ms: list[float]) -> dict:
    a = np.asarray(track_ms)
    return {"n": int(a.size), "mean": float(a.mean()), "p50": float(np.median(a)),
            "p95": float(np.percentile(a, 95)), "min": float(a.min()),
            "max": float(a.max())}


def table(timer: StageTimer, phase: str, envelope_ms: float, note: str) -> str:
    stages = timer.acc.get(phase, {})
    total = sum(stages.values()) * 1e3
    order = ["crop_window", "render", "warp", "normalise", "net_refine",
             "net_score", "empty_cache"]
    # Every share is of the envelope, so the column sums to 100%. The envelope
    # is also the thing an engine would be speeding up, which makes it the only
    # denominator an Amdahl argument may use.
    rows = [f"**{phase}** — {envelope_ms:.1f} ms envelope ({note}), "
            f"{total:.1f} ms attributed to stages", "",
            "| stage | ms | share of envelope | calls |",
            "|---|---:|---:|---:|"]
    net = 0.0
    for name in order:
        if name not in stages:
            continue
        ms = stages[name] * 1e3
        if name.startswith("net_"):
            net += ms
        rows.append(f"| `{name}` | {ms:.1f} | {100*ms/envelope_ms:5.1f}% | "
                    f"{timer.calls[phase][name]} |")
    rows.append(f"| *(unattributed)* | {envelope_ms - total:.1f} | "
                f"{100*(envelope_ms - total)/envelope_ms:5.1f}% | — |")
    rows.append("")
    rows.append(f"**Network share of the envelope: {100*net/envelope_ms:.1f}%** "
                f"→ Amdahl ceiling with a free network: "
                f"{envelope_ms/(envelope_ms-net):.2f}x")
    return "\n".join(rows)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--foundationpose-root", type=Path,
                        default=Path.home() / "src" / "FoundationPose")
    parser.add_argument("--scene", type=Path, default=None,
                        help="default: <root>/demo_data/mustard0")
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--warmup", type=int, default=10)
    parser.add_argument("--est-refine-iter", type=int, default=5)
    parser.add_argument("--track-refine-iter", type=int, default=2)
    parser.add_argument("--register-repeats", type=int, default=3,
                        help="first is cold; the last is the steady-state figure")
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[2]
    out = args.out or repo / "docs" / "results" / "pose_stage_profile.json"
    scene = args.scene or args.foundationpose_root / "demo_data" / "mustard0"
    debug_dir = Path(os.environ.get("TMPDIR", "/tmp")) / "fp_profile_debug"
    debug_dir.mkdir(parents=True, exist_ok=True)

    logging.getLogger().setLevel(logging.ERROR)
    timer = StageTimer()
    est, reader = build_estimator(args.foundationpose_root, scene, debug_dir)
    logging.getLogger().setLevel(logging.ERROR)   # FoundationPose re-raises it

    # Pass A -- envelope, no patches installed.
    envelope = run_sequence(est, reader, args.frames, args.est_refine_iter,
                            args.track_refine_iter, timer, args.warmup,
                            register_repeats=args.register_repeats)

    # Pass B -- instrumented. Re-register so the tracker starts from the same state.
    install_patches(timer)
    timer.enabled = True
    instrumented = run_sequence(est, reader, args.frames, args.est_refine_iter,
                                args.track_refine_iter, timer, args.warmup)
    timer.enabled = False

    env_track = summarise(envelope["track_ms"])
    ins_track = summarise(instrumented["track_ms"])

    print()
    print("## Phase 0 — FoundationPose stage apportionment")
    print()
    print(f"Scene `{scene.name}`, {env_track['n']} tracked frames after "
          f"{args.warmup} warm-up, register {args.est_refine_iter} iters / "
          f"track {args.track_refine_iter} iters.")
    print()
    cold = envelope["register_all_ms"]
    print(f"Registration, repeated in one process: "
          + ", ".join(f"{v:.0f} ms" for v in cold)
          + f"  — the first is cold ({cold[0]/cold[-1]:.2f}x the steady state).")
    print()
    print("| pass | register ms | track p50 ms | track mean ms | track p95 ms |")
    print("|---|---:|---:|---:|---:|")
    print(f"| envelope (no patches) | {envelope['register_ms']:.1f} | "
          f"{env_track['p50']:.2f} | {env_track['mean']:.2f} | {env_track['p95']:.2f} |")
    print(f"| instrumented (syncs)  | {instrumented['register_ms']:.1f} | "
          f"{ins_track['p50']:.2f} | {ins_track['mean']:.2f} | {ins_track['p95']:.2f} |")
    print()
    print(table(timer, "register", instrumented["register_ms"], "one call"))
    print()
    print(table(timer, "track", sum(instrumented["track_ms"]),
                f"{ins_track['n']} calls summed"))
    print()

    payload = {
        "scene": str(scene), "frames": env_track["n"], "warmup": args.warmup,
        "est_refine_iter": args.est_refine_iter,
        "track_refine_iter": args.track_refine_iter,
        "envelope": {"register_ms": envelope["register_ms"],
                     "register_all_ms": envelope["register_all_ms"],
                     "track": env_track},
        "instrumented": {"register_ms": instrumented["register_ms"],
                         "track": ins_track},
        "stages_s": {p: dict(v) for p, v in timer.acc.items()},
        "stage_calls": {p: dict(v) for p, v in timer.calls.items()},
    }
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2))
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
