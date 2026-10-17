#!/usr/bin/env python3
"""pose_error.py -- rotation (deg) and translation (mm) error of the fused poses, per scene.

Proposal §7 asks for rotation and translation error beside ADD-S. Rotation error
is only meaningful up to each object's symmetry, and the symmetry is a property
of the PRINTED object, not of the name it carries:

  red_cube        the 24 rotations of a cube: the smallest angle to any of them
  bowl            a body of revolution: only the tilt of its upright axis counts
  mustard_bottle  TWO-FOLD about its upright axis. The YCB mesh turned half a turn
                  is the same surface to 1.35 mm on average and 4.6 mm at worst
                  (nozzle centred to 1.4 mm), and the printed bottle is one plain
                  colour: no sensor here can tell its front from its back, and its
                  tray seats it either way. So the tray's ground truth fixes its
                  yaw only up to 180 degrees, and measured without this every
                  single-bottle scene reads "back to front" (171-180 degrees).

A fused object is matched to the ground-truth object of its class within 50 mm
on the same pair; only objects the scene graph reported (supported) are scored.

  scripts/pose_error.py                 # the 16 scored scenes, latest run
  scripts/pose_error.py --run-offset 1  # the run before the latest
"""
from __future__ import annotations

import argparse
import collections
import itertools
import os
import sqlite3
import sys

import numpy as np
from scipy.spatial.transform import Rotation as Rot

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENES = [
    "20261005/t1_cube", "20261005/t1_cube_b", "20261005/t1_bottle", "20261006/t1_bottle_b",
    "20261006/t1_bottle_c", "20261007/t2_clutter_a", "20261007/t2_clutter_b",
    "20261007/t2_clutter_c", "20261005/t3_cube_front", "20261005/t3_cube_over",
    "20261006/t3_bottle_over", "20261006/t3_bottle_over_b", "20261005/t4_cube_front",
    "20261005/t4_cube_over", "20261006/t4_bottle_over", "20261006/t4_bottle_over_b",
]
CUBE = [Rot.from_matrix(np.diag(s) @ np.eye(3)[list(p)])
        for p in itertools.permutations(range(3)) for s in itertools.product([1, -1], repeat=3)
        if np.isclose(np.linalg.det(np.diag(s) @ np.eye(3)[list(p)]), 1.0)]
HALF_TURN = [Rot.identity(), Rot.from_euler("z", 180, degrees=True)]


def rot(q_wxyz):
    w, x, y, z = q_wxyz
    return Rot.from_quat([x, y, z, w])


def rotation_error_deg(cls: str, q_est, q_gt) -> float:
    est, gt = rot(q_est), rot(q_gt)
    if cls == "bowl":
        a, b = est.apply([0, 0, 1]), gt.apply([0, 0, 1])
        return float(np.degrees(np.arccos(np.clip(a @ b, -1.0, 1.0))))
    group = CUBE if cls == "red_cube" else HALF_TURN if cls == "mustard_bottle" else [Rot.identity()]
    return float(min(np.degrees(((gt * s).inv() * est).magnitude()) for s in group))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenes", nargs="*", default=SCENES)
    ap.add_argument("--run-offset", type=int, default=0, help="0 = latest run, 1 = the one before, ...")
    a = ap.parse_args()
    assert len(CUBE) == 24
    per_class = collections.defaultdict(lambda: ([], []))
    print("| scene | object | rotation deg, median (p95) | translation mm, median (p95) | pairs |")
    print("|---|---|---|---|---|")
    for s in a.scenes:
        db = os.path.join(REPO, "runs/scored", s, "metrics.sqlite")
        con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        runs = [r for (r,) in con.execute("select run_id from runs order by run_id")]
        run = runs[-1 - a.run_offset]
        gt = collections.defaultdict(list)
        for p, c, *v in con.execute(
                "select pair_index, cls, tx, ty, tz, qw, qx, qy, qz from ground_truth where run_id=?", (run,)):
            gt[(p, c)].append(np.array(v))
        rot_e, tr_e = collections.defaultdict(list), collections.defaultdict(list)
        for p, c, *v in con.execute(
                "select pair_index, cls, tx, ty, tz, qw, qx, qy, qz from objects "
                "where run_id=? and supported=1 and is_static=0", (run,)):
            v = np.array(v)
            for g in gt.get((p, c), []):
                d = np.linalg.norm(v[:3] - g[:3])
                if d < 0.05:
                    rot_e[c].append(rotation_error_deg(c, v[3:], g[3:]))
                    tr_e[c].append(1000 * d)
                    break
        for c in sorted(rot_e):
            r, t = np.array(rot_e[c]), np.array(tr_e[c])
            per_class[c][0].append(np.median(r))
            per_class[c][1].append(np.median(t))
            print(f"| {s} | {c} | {np.median(r):.1f} ({np.percentile(r, 95):.1f}) | "
                  f"{np.median(t):.1f} ({np.percentile(t, 95):.1f}) | {len(r)} |")
    print("\n| object | scenes | rotation deg, median of scene medians [range] | translation mm, the same |")
    print("|---|---|---|---|")
    for c, (r, t) in sorted(per_class.items()):
        print(f"| {c} | {len(r)} | {np.median(r):.1f} [{min(r):.1f}, {max(r):.1f}] | "
              f"{np.median(t):.1f} [{min(t):.1f}, {max(t):.1f}] |")
    return 0


if __name__ == "__main__":
    sys.exit(main())
