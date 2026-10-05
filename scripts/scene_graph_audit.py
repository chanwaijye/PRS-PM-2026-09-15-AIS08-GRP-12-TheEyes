#!/usr/bin/env python3
"""scene_graph_audit.py -- what is in the scene graph that is not on the table.

ADD-S scores the estimate nearest each ground-truth object and never looks at
the rest of the pipeline's output. This reads the rest: every fused track of
every processed scene of a capture day, whether it is the object that was
really there, and whether it rests on anything. It also says, for each view's
estimates of the true object, how far they are along and off that camera's own
line of sight -- a pose at the occluder's range is a range error, not a miss.

Read-only. Needs only the day's processed logs (capture_day.sh process) and
the scenes' saved extrinsics; no rig, no GPU.

  scripts/scene_graph_audit.py 20261005                 # every processed scene
  scripts/scene_graph_audit.py 20261005 t3_cube_over    # named scenes only
  scripts/scene_graph_audit.py 20261005 --run 1         # a run other than the latest

A track is REAL when its class is in the scene's ground truth and its median
position is within --real-within (default 50 mm) of it; otherwise it is an
object that is not there. "Rests on something" is the scene graph's own word:
an (on <id> ...) fact on that pair.
"""
import argparse
import glob
import os
import sqlite3
import sys

import numpy as np
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def open_ro(path):
    return sqlite3.connect(f"file:{path}?mode=ro", uri=True)


def audit_scene(day, scene, run, real_within_m, totals):
    db = open_ro(os.path.join(REPO, "runs/scored", day, scene, "metrics.sqlite"))
    run = run or db.execute("select max(run_id) from runs").fetchone()[0]
    pairs = db.execute("select count(*) from frames where run_id=?", (run,)).fetchone()[0]
    truth = db.execute(
        "select cls, instance, tx, ty, tz from ground_truth where run_id=? "
        "group by cls, instance", (run,)).fetchall()
    if not truth:
        print(f"\n{scene} (run {run}): no ground truth -- skipped")
        return
    print(f"\n{scene} (run {run}, {pairs} pairs) -- really there: "
          + ", ".join(sorted({t[0] for t in truth})))

    on = {}
    for fact, pair in db.execute(
            "select fact, pair_index from facts where run_id=? and fact like '(on %'", (run,)):
        on.setdefault(fact.split()[1], set()).add(pair)

    tracks = db.execute(
        "select object_id, cls, count(*) from objects where run_id=? and is_static=0 "
        "group by object_id order by object_id", (run,)).fetchall()
    for oid, cls, n in tracks:
        xyz = np.array(db.execute(
            "select tx, ty, tz from objects where run_id=? and object_id=?", (run, oid)).fetchall())
        med = np.median(xyz, axis=0)
        dist = min((np.linalg.norm(med - np.array(t[2:])) for t in truth if t[0] == cls),
                   default=None)
        real = dist is not None and dist < real_within_m
        supported = len(on.get(oid, ()))
        key = "real" if real else "not_there"
        totals[key][0] += n
        totals[key][1] += supported
        where = f"{dist * 1000:6.1f} mm from the truth" if dist is not None else "no such object in the scene"
        print(f"  {oid:20s} {'REAL' if real else 'not there':9s} on {n:3d} pairs, "
              f"centre z {med[2] * 1000:7.1f} mm (table top 0), {where:28s} "
              f"rests on something on {supported}/{n}")

    ext_path = os.path.join(REPO, "captures/scored", day, scene, "extrinsics_at_recording.yaml")
    if not os.path.exists(ext_path):
        print(f"  (no {ext_path}: per-view range errors not computed)")
        return
    cams = yaml.safe_load(open(ext_path))["cameras"]
    print("  per view, estimates of the true object (by detector track, 3 pairs or more):")
    for cls, _instance, *g in truth:
        g = np.array(g)
        for cam, entry in cams.items():
            T = np.array(entry["T_world_cam"], dtype=float).reshape(4, 4)
            centre = T[:3, 3]
            true_range = np.linalg.norm(g - centre)
            ray = (g - centre) / true_range
            groups = db.execute(
                "select track_id, count(*) from estimates where run_id=? and camera_id=? and cls=? "
                "group by track_id having count(*) >= 3 order by count(*) desc",
                (run, cam, cls)).fetchall()
            if not groups:
                print(f"    {cam:10s} {cls}: none")
            for tid, n in groups:
                rows = np.array(db.execute(
                    "select tx, ty, tz, score from estimates where run_id=? and camera_id=? "
                    "and cls=? and track_id=?", (run, cam, cls, tid)).fetchall())
                p = np.median(rows[:, :3], axis=0) @ T[:3, :3].T + centre
                along = (p - centre) @ ray
                off = np.linalg.norm((p - centre) - along * ray)
                print(f"    {cam:10s} {cls} #{tid}: {n:3d} pairs, pose score {rows[:, 3].mean():.2f}, "
                      f"{np.linalg.norm(p - g) * 1000:6.1f} mm from the truth = "
                      f"{(along - true_range) * 1000:+7.1f} mm along the line of sight, "
                      f"{off * 1000:5.1f} mm off it")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("day", help="capture day, YYYYMMDD (runs/scored/<day>)")
    ap.add_argument("scenes", nargs="*", help="scene names; default every processed scene")
    ap.add_argument("--run", type=int, default=None, help="run id; default the latest in each log")
    ap.add_argument("--real-within", type=float, default=50.0,
                    help="mm from the ground truth within which a track counts as the real object")
    args = ap.parse_args()

    scenes = args.scenes or sorted(
        os.path.basename(os.path.dirname(p))
        for p in glob.glob(os.path.join(REPO, "runs/scored", args.day, "*", "metrics.sqlite")))
    if not scenes:
        sys.exit(f"no processed scenes under runs/scored/{args.day}")
    totals = {"real": [0, 0], "not_there": [0, 0]}
    for scene in scenes:
        audit_scene(args.day, scene, args.run, args.real_within / 1000.0, totals)
    print(f"\nTOTAL over {len(scenes)} scene(s), in object-pairs:")
    print(f"  really there : {totals['real'][0]:5d}, resting on something on {totals['real'][1]}")
    print(f"  not there    : {totals['not_there'][0]:5d}, resting on something on {totals['not_there'][1]}")


if __name__ == "__main__":
    main()
