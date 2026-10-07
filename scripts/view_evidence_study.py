#!/usr/bin/env python3
"""view_evidence_study.py -- what tells a right per-view pose from a wrong one?

The gate keeps the higher pose score when two views disagree, and on the
half-hidden bottle (2026-10-06) that is the wrong view. This reads the logged
per-view estimates of the processed scenes back against the RECORDED depth of
both cameras and tabulates, for estimates that are right and estimates that are
wrong, every candidate signal a fusion rule could use:

  pose score, detector score               what the frontend said
  own view:   agree / none / front / through
  other view: agree / none / front / through

For one estimate, the object is rendered at the estimated pose into a view and
each silhouette pixel is sorted by that view's measured depth:
  agree    measured within --tol of the rendered surface
  none     no depth there
  front    measured NEARER than the surface: something stands in front
  through  measured FARTHER than the surface: the camera sees through the place
           where the object is claimed to be -- free space where there should be
           a surface
"own" is the view the estimate came from, "other" the second camera.

Read-only; no rig, no GPU. A surface sampling, not the C++ rasteriser: fractions
are good to a few per cent, which is what a study needs.

  scripts/view_evidence_study.py 20261006 t3_bottle_over_b t1_bottle_c
  scripts/view_evidence_study.py 20261005            # every processed scene of a day
"""
import argparse
import glob
import os
import sqlite3

import cv2
import numpy as np
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CAMS = ("cam_front", "cam_over")


def quat_R(w, x, y, z):
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def sample_mesh(path, n=30000, seed=0):
    """Area-weighted points on the mesh surface, object frame, metres."""
    vs, fs = [], []
    for line in open(path):
        if line.startswith("v "):
            vs.append([float(t) for t in line.split()[1:4]])
        elif line.startswith("f "):
            idx = [int(t.split("/")[0]) for t in line.split()[1:]]
            idx = [i - 1 if i > 0 else len(vs) + i for i in idx]
            for k in range(1, len(idx) - 1):
                fs.append([idx[0], idx[k], idx[k + 1]])
    v, f = np.array(vs), np.array(fs)
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    rng = np.random.default_rng(seed)
    tri = rng.choice(len(f), size=n, p=area / area.sum())
    u, w = rng.random(n), rng.random(n)
    flip = u + w > 1
    u[flip], w[flip] = 1 - u[flip], 1 - w[flip]
    return a[tri] + u[:, None] * (b[tri] - a[tri]) + w[:, None] * (c[tri] - a[tri])


def render(points_cam, K, shape):
    """Nearest-surface depth image of the sampled points (3x3 splat). 0 = empty."""
    h, w = shape
    z = points_cam[:, 2]
    ok = z > 0.05
    u = np.round(K[0, 0] * points_cam[ok, 0] / z[ok] + K[0, 2]).astype(int)
    v = np.round(K[1, 1] * points_cam[ok, 1] / z[ok] + K[1, 2]).astype(int)
    zz = z[ok]
    buf = np.full((h, w), np.inf)
    for du in (-1, 0, 1):
        for dv in (-1, 0, 1):
            uu, vv = u + du, v + dv
            m = (uu >= 0) & (uu < w) & (vv >= 0) & (vv < h)
            np.minimum.at(buf, (vv[m], uu[m]), zz[m])
    buf[np.isinf(buf)] = 0.0
    return buf


def sort_pixels(rendered, measured_m, tol, through_tol):
    sil = rendered > 0
    n = int(sil.sum())
    if n == 0:
        return None
    d, z = measured_m[sil], rendered[sil]
    none = d <= 0
    agree = ~none & (np.abs(d - z) <= tol)
    front = ~none & (d < z - tol)
    through = ~none & (d > z + through_tol)
    return dict(n=n, agree=agree.mean(), none=none.mean(), front=front.mean(), through=through.mean())


def study_scene(day, scene, args, meshes, rows_out):
    cdir = os.path.join(REPO, "captures/scored", day, scene)
    db = sqlite3.connect(f"file:{os.path.join(REPO, 'runs/scored', day, scene, 'metrics.sqlite')}?mode=ro", uri=True)
    run = db.execute("select max(run_id) from runs").fetchone()[0]
    ses_dir = os.path.realpath(os.path.join(cdir, "session"))
    ses = yaml.safe_load(open(os.path.join(ses_dir, "session.yaml")))
    ext = yaml.safe_load(open(os.path.join(cdir, "extrinsics_at_recording.yaml")))["cameras"]
    T = {c: np.array(ext[c]["T_world_cam"], float).reshape(4, 4) for c in CAMS}
    K = {c: np.array(ses["cameras"][c]["K"], float).reshape(3, 3) for c in CAMS}
    scale = {c: float(ses["cameras"][c]["depth_scale_m"]) for c in CAMS}
    dc_path = os.path.join(cdir, "depth_correction.yaml")
    if os.path.exists(dc_path):
        dc = yaml.safe_load(open(dc_path)).get("cameras", {})
        for c in CAMS:
            scale[c] *= float(dc.get(c, {}).get("scale", 1.0))
    truth = {c: np.array(p) for c, *p in db.execute(
        "select cls, tx, ty, tz from ground_truth where run_id=? group by cls", (run,))}
    pairs = [r[0] for r in db.execute("select pair_index from frames where run_id=? order by pair_index", (run,))]
    for pair in pairs[:: args.every]:
        depth = {}
        for c in CAMS:
            f = os.path.join(ses_dir, c, f"{pair:06d}.depth.png")
            if os.path.exists(f):
                depth[c] = cv2.imread(f, cv2.IMREAD_UNCHANGED).astype(np.float64) * scale[c]
        if len(depth) < 2:
            continue
        for cam, cls, tid, score, tx, ty, tz, qw, qx, qy, qz in db.execute(
                "select camera_id, cls, track_id, score, tx, ty, tz, qw, qx, qy, qz from estimates "
                "where run_id=? and pair_index=?", (run, pair)):
            if cls not in meshes:
                continue
            det = db.execute("select score from detections where run_id=? and pair_index=? and camera_id=? "
                             "and cls=? and track_id=?", (run, pair, cam, cls, tid)).fetchone()
            T_cam_obj = np.eye(4)
            T_cam_obj[:3, :3] = quat_R(qw, qx, qy, qz)
            T_cam_obj[:3, 3] = [tx, ty, tz]
            T_w_obj = T[cam] @ T_cam_obj
            # A class with no ground truth in the scene is an object that is not there.
            err = np.linalg.norm(T_w_obj[:3, 3] - truth[cls]) * 1000 if cls in truth else np.inf
            other = [c for c in CAMS if c != cam][0]
            out = dict(scene=scene, cam=cam, cls=cls, err=err, right=err < args.right_within,
                       score=score, det=det[0] if det else np.nan)
            for tag, view in (("own", cam), ("oth", other)):
                Tv = np.linalg.inv(T[view]) @ T_w_obj
                pts = meshes[cls] @ Tv[:3, :3].T + Tv[:3, 3]
                s = sort_pixels(render(pts, K[view], depth[view].shape), depth[view], args.tol / 1000, args.through_tol / 1000)
                for k in ("agree", "none", "front", "through"):
                    out[f"{tag}_{k}"] = s[k] if s else np.nan
                out[f"{tag}_n"] = s["n"] if s else 0
            rows_out.append(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("day")
    ap.add_argument("scenes", nargs="*")
    ap.add_argument("--every", type=int, default=15, help="use every Nth pair (default 15)")
    ap.add_argument("--tol", type=float, default=20.0, help="mm: measured depth agrees with the pose")
    ap.add_argument("--through-tol", type=float, default=40.0, help="mm: measured depth is beyond the object")
    ap.add_argument("--spread", action="store_true", help="also print the per-frame range of the two deciding signals")
    ap.add_argument("--right-within", type=float, default=30.0, help="mm from the truth for an estimate to count as right")
    args = ap.parse_args()
    reg = yaml.safe_load(open(os.path.join(REPO, "config/meshes.yaml")))["objects"]
    meshes = {c: sample_mesh(os.path.join(REPO, m["mesh"])) for c, m in reg.items()}
    scenes = args.scenes or sorted(
        os.path.basename(os.path.dirname(p))
        for p in glob.glob(os.path.join(REPO, "runs/scored", args.day, "*", "metrics.sqlite"))
        if os.path.exists(os.path.join(REPO, "captures/scored", args.day, os.path.basename(os.path.dirname(p)), "notes.yaml")))
    rows = []
    for scene in scenes:
        study_scene(args.day, scene, args, meshes, rows)
    hdr = (f"{'scene':20s} {'view':9s} {'object':14s} {'':5s} {'n':>4s} {'err mm':>7s} {'pose':>5s} {'det':>5s} |"
           f" {'own: agree':>10s} {'none':>5s} {'front':>5s} {'thru':>5s} | {'other: agree':>12s} {'none':>5s} {'front':>5s} {'thru':>5s}")
    print(hdr)
    keys = sorted({(r["scene"], r["cam"], r["cls"], r["right"]) for r in rows}, key=lambda k: (scenes.index(k[0]), k[2], k[1], not k[3]))
    med = lambda rs, k: float(np.nanmedian([r[k] for r in rs]))
    for scene, cam, cls, right in keys:
        rs = [r for r in rows if (r["scene"], r["cam"], r["cls"], r["right"]) == (scene, cam, cls, right)]
        print(f"{scene:20s} {cam:9s} {cls:14s} {'right' if right else 'WRONG':5s} {len(rs):4d} {med(rs, 'err'):7.1f} "
              f"{med(rs, 'score'):5.2f} {med(rs, 'det'):5.2f} | {med(rs, 'own_agree'):10.2f} {med(rs, 'own_none'):5.2f} "
              f"{med(rs, 'own_front'):5.2f} {med(rs, 'own_through'):5.2f} | {med(rs, 'oth_agree'):12.2f} "
              f"{med(rs, 'oth_none'):5.2f} {med(rs, 'oth_front'):5.2f} {med(rs, 'oth_through'):5.2f}"
              + (f"   thru min-max {np.nanmin([r['oth_through'] for r in rs]):.2f}-{np.nanmax([r['oth_through'] for r in rs]):.2f},"
                 f" own agree min-max {np.nanmin([r['own_agree'] for r in rs]):.2f}-{np.nanmax([r['own_agree'] for r in rs]):.2f}"
                 if args.spread and not all(np.isnan(r['oth_through']) for r in rs) else ""))


if __name__ == "__main__":
    main()
