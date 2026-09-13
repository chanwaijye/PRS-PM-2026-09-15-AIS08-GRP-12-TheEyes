#!/usr/bin/env python3
"""Does FoundationPose registration survive refine_iterations 5 -> 3?

docs/TRT_PLAN.md 8 measured that registration is 72.6% network and that refine
work scales linearly in iterations, so 5 -> 3 is worth ~1.54x -- far more than
any TensorRT engine could return on this card. It is also NVIDIA's own default
(isaac_ros_foundationpose `refine_iterations: 3`). What it costs in accuracy is
the open question, and mustard0 ships no ground-truth poses, so the check is
self-referential: does a 3-iteration registration land where a converged one
does, and does any difference propagate downstream?

Three parts:

  A  convergence sweep on frame 0 with the real mask, iterations 1..8, each
     scored against the most converged pose
  B  independent registrations at frames sampled across the sequence. mustard0
     has one mask, so the mask is rebuilt the way SystemCode/pose actually
     builds one -- an ROI box gated by depth validity -- from the tracked pose.
     Optimistic about the detector, identical across arms, which is what a
     relative comparison needs
  C  the downstream test: track the whole sequence from a 3-iteration and a
     5-iteration registration and compare the trajectories against the tracker's
     own frame-to-frame noise (docs/R1_STATUS.md)

Metric throughout is ADD-S -- symmetry-aware closest-point distance between two
poses of the same mesh -- reported as a percentage of object diameter, the unit
AGENTS.md states the acceptance threshold in.

    ~/venvs/the-eyes-r1/bin/python training/pose/registration_quality.py
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import time
from pathlib import Path

import numpy as np

# R1_STATUS.md, the validated 5-iteration run: what the tracker moves per frame
# on its own. A registration change must stay under this to be invisible.
TRACKER_NOISE = {"trans_median_mm": 0.27, "trans_p95_mm": 11.5, "trans_max_mm": 17.8,
                 "rot_median_deg": 0.31, "rot_max_deg": 3.54}


def add_s(pose_a: np.ndarray, pose_b: np.ndarray, verts: np.ndarray) -> float:
    """Mean over vertices of the closest-point distance between the two posings."""
    import torch
    v = torch.as_tensor(verts, dtype=torch.float32, device="cuda")
    ones = torch.ones((v.shape[0], 1), device="cuda")
    vh = torch.cat([v, ones], dim=1)
    a = (vh @ torch.as_tensor(pose_a, dtype=torch.float32, device="cuda").T)[:, :3]
    b = (vh @ torch.as_tensor(pose_b, dtype=torch.float32, device="cuda").T)[:, :3]
    return float(torch.cdist(a, b).min(dim=1).values.mean())


def pose_delta(pose_a: np.ndarray, pose_b: np.ndarray) -> tuple[float, float]:
    """Translation in mm and rotation in degrees between two poses."""
    trans = float(np.linalg.norm(pose_a[:3, 3] - pose_b[:3, 3]) * 1e3)
    r = pose_a[:3, :3].T @ pose_b[:3, :3]
    rot = float(np.degrees(np.arccos(np.clip((np.trace(r) - 1) / 2, -1.0, 1.0))))
    return trans, rot


def roi_mask(pose, verts, K, depth, margin_ratio: float) -> np.ndarray:
    """The mask SystemCode/pose sends: an ROI box gated by depth validity.

    SystemCode/pose/README.md: "Registration wants a segmentation mask, and we
    send a box. The mask is the ROI rectangle gated by depth validity." The box
    here comes from the projected mesh rather than the detector, which is
    optimistic -- but it is identical for every arm, so the comparison holds.
    """
    h, w = depth.shape[:2]
    cam = (np.hstack([verts, np.ones((len(verts), 1))]) @ pose.T)[:, :3]
    cam = cam[cam[:, 2] > 1e-6]
    uv = (K @ cam.T).T
    uv = uv[:, :2] / uv[:, 2:3]
    x0, y0 = uv.min(axis=0)
    x1, y1 = uv.max(axis=0)
    mx, my = (x1 - x0) * margin_ratio, (y1 - y0) * margin_ratio
    x0, x1 = int(max(0, x0 - mx)), int(min(w - 1, x1 + mx))
    y0, y1 = int(max(0, y0 - my)), int(min(h - 1, y1 + my))
    mask = np.zeros((h, w), dtype=bool)
    mask[y0:y1 + 1, x0:x1 + 1] = True
    return mask & (depth >= 0.001)


def build(fp_root: Path, scene: Path, debug_dir: Path):
    sys.path[:0] = [str(fp_root)]
    os.chdir(fp_root)
    from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor
    from datareader import YcbineoatReader
    import nvdiffrast.torch as dr
    import trimesh
    from Utils import set_seed

    set_seed(0)
    mesh = trimesh.load(str(scene / "mesh" / "textured_simple.obj"))
    est = FoundationPose(model_pts=mesh.vertices, model_normals=mesh.vertex_normals,
                         mesh=mesh, scorer=ScorePredictor(),
                         refiner=PoseRefinePredictor(),
                         glctx=dr.RasterizeCudaContext(), debug=0,
                         debug_dir=str(debug_dir))
    reader = YcbineoatReader(video_dir=str(scene), shorter_side=None, zfar=np.inf)
    return est, reader


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--foundationpose-root", type=Path,
                    default=Path.home() / "src" / "FoundationPose")
    ap.add_argument("--scene", type=Path, default=None)
    ap.add_argument("--sweep-max", type=int, default=8)
    ap.add_argument("--sample-frames", type=int, default=20)
    ap.add_argument("--target", type=int, default=3,
                    help="the arm being proposed; named in the headline verdict")
    ap.add_argument("--incumbent", type=int, default=5,
                    help="the setting in production; arms are judged against it")
    ap.add_argument("--arms", type=str, default="3,4,5",
                    help="iteration counts to test against the converged reference")
    ap.add_argument("--track-frames", type=int, default=0, help="0 = whole sequence")
    ap.add_argument("--track-refine-iter", type=int, default=2)
    ap.add_argument("--margin-ratio", type=float, default=0.15,
                    help="config/pose.yaml roi.margin_ratio")
    ap.add_argument("--verts", type=int, default=2000)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    repo = Path(__file__).resolve().parents[2]
    out = args.out or repo / "docs" / "results" / "registration_quality.json"
    scene = args.scene or args.foundationpose_root / "demo_data" / "mustard0"
    debug = Path(os.environ.get("TMPDIR", "/tmp")) / "fp_regq_debug"
    debug.mkdir(parents=True, exist_ok=True)

    logging.getLogger().setLevel(logging.ERROR)
    est, reader = build(args.foundationpose_root, scene, debug)
    logging.getLogger().setLevel(logging.ERROR)

    diameter_mm = est.diameter * 1e3
    rng = np.random.default_rng(0)
    all_v = np.asarray(est.mesh_ori.vertices)
    verts = all_v[rng.choice(len(all_v), min(args.verts, len(all_v)), replace=False)]
    pct = lambda mm: 100.0 * mm / diameter_mm

    mask0 = reader.get_mask(0).astype(bool)
    rgb0, depth0 = reader.get_color(0), reader.get_depth(0)
    n_frames = args.track_frames or (len(reader.color_files) - 1)

    def register(rgb, depth, mask, iters):
        t = time.perf_counter()
        pose = est.register(K=reader.K, rgb=rgb, depth=depth, ob_mask=mask,
                            iteration=iters)
        return pose, (time.perf_counter() - t) * 1e3

    # ---------------- Part A: convergence sweep, real mask ----------------
    sweep = {}
    for k in range(1, args.sweep_max + 1):
        pose, ms = register(rgb0, depth0, mask0, k)
        sweep[k] = {"pose": pose.tolist(), "ms": ms}
    ref = np.array(sweep[args.sweep_max]["pose"])
    for k, rec in sweep.items():
        pose = np.array(rec["pose"])
        rec["adds_mm"] = add_s(pose, ref, verts) * 1e3
        rec["adds_pct"] = pct(rec["adds_mm"])
        rec["trans_mm"], rec["rot_deg"] = pose_delta(pose, ref)
    for k in range(2, args.sweep_max + 1):
        step = add_s(np.array(sweep[k]["pose"]), np.array(sweep[k - 1]["pose"]), verts)
        sweep[k]["step_from_prev_mm"] = step * 1e3

    # ---------------- reference trajectory (register at 5) ----------------
    pose5, _ = register(rgb0, depth0, mask0, 5)
    traj5 = [pose5]
    for i in range(1, n_frames + 1):
        traj5.append(est.track_one(rgb=reader.get_color(i), depth=reader.get_depth(i),
                                   K=reader.K, iteration=args.track_refine_iter))

    # ---------------- Part B: independent registrations ----------------
    arms = [int(a) for a in args.arms.split(",")]
    idx = np.linspace(0, n_frames, args.sample_frames, dtype=int)
    per_frame = []
    for i in idx:
        rgb, depth = reader.get_color(int(i)), reader.get_depth(int(i))
        mask = roi_mask(traj5[int(i)], all_v, reader.K, depth, args.margin_ratio)
        if mask.sum() < 50:
            continue
        poses, times = {}, {}
        for k in sorted(set(arms + [args.sweep_max])):
            poses[k], times[k] = register(rgb, depth, mask, k)
        ref_k = np.array(poses[args.sweep_max])
        rec = {"frame": int(i), "mask_px": int(mask.sum())}
        for k in arms:
            rec[f"adds_{k}_mm"] = add_s(np.array(poses[k]), ref_k, verts) * 1e3
            rec[f"ms_{k}"] = times[k]
        per_frame.append(rec)

    # ---------------- Part C: downstream trajectory ----------------
    pose3, _ = register(rgb0, depth0, mask0, 3)
    traj3 = [pose3]
    for i in range(1, n_frames + 1):
        traj3.append(est.track_one(rgb=reader.get_color(i), depth=reader.get_depth(i),
                                   K=reader.K, iteration=args.track_refine_iter))
    downstream = [dict(zip(("trans_mm", "rot_deg"), pose_delta(a, b)))
                  for a, b in zip(traj3, traj5)]
    downstream_adds = [add_s(a, b, verts) * 1e3 for a, b in zip(traj3, traj5)]

    # ---------------- report ----------------
    col = lambda rows, key: np.array([r[key] for r in rows])
    print()
    print("## Registration quality — refine_iterations 5 vs 3")
    print()
    print(f"Scene `{scene.name}`, mesh diameter {diameter_mm:.1f} mm, "
          f"ADD-S over {len(verts)} vertices. "
          f"AGENTS.md acceptance is ADD-S < 10% of diameter "
          f"(= {0.10*diameter_mm:.1f} mm).")
    print()
    print("### A. Convergence on frame 0 (real mask)")
    print()
    print(f"| iters | ms | ADD-S vs {args.sweep_max}-iter | % of diameter | "
          f"step from previous |")
    print("|---:|---:|---:|---:|---:|")
    for k, r in sweep.items():
        step = f"{r.get('step_from_prev_mm', float('nan')):.2f} mm" if k > 1 else "—"
        print(f"| {k} | {r['ms']:.0f} | {r['adds_mm']:.2f} mm | "
              f"{r['adds_pct']:.2f}% | {step} |")
    print()
    print(f"### B. Independent registrations at {len(per_frame)} sampled frames")
    print()
    print(f"Each arm scored by ADD-S against the {args.sweep_max}-iteration pose on "
          f"the same frame and the same mask. A *gross failure* is ADD-S above the "
          f"10%-of-diameter acceptance threshold ({0.10*diameter_mm:.1f} mm) — the "
          f"registration landed in the wrong basin, not merely short of convergence.")
    print()
    print("| iters | median ADD-S | p95 | max | gross failures | median ms | speedup vs 5 |")
    print("|---:|---:|---:|---:|---:|---:|---:|")
    base_ms = np.median(col(per_frame, "ms_5")) if 5 in arms else None
    failures = {}
    for k in arms:
        a, ms = col(per_frame, f"adds_{k}_mm"), col(per_frame, f"ms_{k}")
        nbad = int((a > 0.10 * diameter_mm).sum())
        failures[k] = nbad
        speed = f"{base_ms/np.median(ms):.2f}x" if base_ms else "—"
        print(f"| {k} | {np.median(a):.2f} mm ({pct(np.median(a)):.2f}%) | "
              f"{np.percentile(a,95):.2f} mm | {a.max():.2f} mm ({pct(a.max()):.1f}%) | "
              f"**{nbad}/{len(a)}** | {np.median(ms):.0f} | {speed} |")
    print()
    print(f"### C. Downstream — {len(traj3)} frames tracked from each registration")
    print()
    t, r = col(downstream, "trans_mm"), col(downstream, "rot_deg")
    d = np.array(downstream_adds)
    print("| metric | median | p95 | max | tracker's own noise (R1) |")
    print("|---|---:|---:|---:|---|")
    print(f"| translation | {np.median(t):.2f} mm | {np.percentile(t,95):.2f} mm | "
          f"{t.max():.2f} mm | median {TRACKER_NOISE['trans_median_mm']}, "
          f"p95 {TRACKER_NOISE['trans_p95_mm']}, max {TRACKER_NOISE['trans_max_mm']} mm |")
    print(f"| rotation | {np.median(r):.2f}° | {np.percentile(r,95):.2f}° | "
          f"{r.max():.2f}° | median {TRACKER_NOISE['rot_median_deg']}, "
          f"max {TRACKER_NOISE['rot_max_deg']}° |")
    print(f"| ADD-S | {np.median(d):.2f} mm ({pct(np.median(d)):.2f}%) | "
          f"{np.percentile(d,95):.2f} mm ({pct(np.percentile(d,95)):.2f}%) | "
          f"{d.max():.2f} mm ({pct(d.max()):.2f}%) | — |")
    print()

    # The decision is "is this arm worse than what we already run?", not "is this
    # arm perfect". Judging against perfection hides the case that matters here:
    # the incumbent failing too, on the same frame, for the same reason.
    inc = args.incumbent
    bad_frames = {k: {r["frame"] for r in per_frame if r[f"adds_{k}_mm"] > 0.10*diameter_mm}
                  for k in arms}
    print("### Verdict — each arm against the incumbent, not against perfection")
    print()
    if inc in arms:
        print(f"The incumbent ({inc} iterations) itself fails grossly on "
              f"**{len(bad_frames[inc])}/{len(per_frame)}** registrations "
              f"(frames {sorted(bad_frames[inc]) or '—'}). That is a pre-existing "
              f"defect and it is not caused by any change proposed here.")
        print()
        print("| arm | fails where the incumbent succeeds | non-failing median ADD-S | speedup |")
        print("|---:|---|---:|---:|")
        for k in arms:
            regress = sorted(bad_frames[k] - bad_frames[inc])
            a = col(per_frame, f"adds_{k}_mm")
            quiet = a[a <= 0.10*diameter_mm]
            print(f"| {k} | {'**' + str(regress) + '**' if regress else 'none'} | "
                  f"{np.median(quiet):.2f} mm | "
                  f"{np.median(col(per_frame, f'ms_{inc}'))/np.median(col(per_frame, f'ms_{k}')):.2f}x |")
        print()
    safe = [k for k in arms if not failures[k]]
    lowest = min(safe) if safe else None
    target = args.target if args.target in arms else min(arms)
    verdict = inc in arms and not (bad_frames[target] - bad_frames[inc])
    print(f"**{inc} -> {target} shows "
          f"{'NO regression' if verdict else 'a REGRESSION'} against the incumbent "
          f"on {len(per_frame)} registrations.** "
          + (f"Separately, the lowest arm with zero gross failures is "
             f"{lowest}, so robustness argues for raising the count, not lowering it."
             if lowest else
             "No arm tested was free of gross failures."))
    print()

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({
        "scene": str(scene), "diameter_mm": diameter_mm, "verts": len(verts),
        "track_refine_iter": args.track_refine_iter,
        "margin_ratio": args.margin_ratio,
        "sweep": {str(k): {kk: vv for kk, vv in v.items() if kk != "pose"}
                  for k, v in sweep.items()},
        "arms": arms, "gross_failures": failures, "lowest_safe_arm": lowest,
        "per_frame": per_frame,
        "downstream": {"n": len(downstream),
                       "trans_mm": t.tolist(), "rot_deg": r.tolist(),
                       "adds_mm": d.tolist()},
        "tracker_noise_reference": TRACKER_NOISE,
        "incumbent": inc, "no_regression_vs_incumbent": bool(verdict),
        "gross_failure_frames": {str(k): sorted(v) for k, v in bad_frames.items()},
    }, indent=2))
    print(f"wrote {out}")
    return 0 if verdict else 1


if __name__ == "__main__":
    raise SystemExit(main())
