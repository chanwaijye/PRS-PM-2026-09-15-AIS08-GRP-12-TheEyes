#!/usr/bin/env python3
"""ablation_ci.py -- the occlusion ablation with the significance test of proposal §7.1.

Reads each scored scene's ablation.log (ablation_from_log's report of the
latest run) and reports, per group of scenes and per arm:

  * mean ADD-S over the frames an arm matched, and recall@0.1d (the share of
    ground-truth object-pairs found AND within a tenth of the diameter: a miss
    counts as a failure, so an arm that sees nothing scores 0 rather than
    dropping out);
  * a 95% bootstrap confidence interval for each, RESAMPLING SCENES, not frames,
    because the 300 frames of a scene are one arrangement seen 300 times;
  * paired differences, scene by scene, with the same interval. Fixed in the
    proposal before any data was seen: an ADD-S difference counts only if its
    interval excludes zero AND it is larger than the sensor floor (2-4 mm; the
    verdict is given at both ends). A recall difference has no floor.

Each scene counts once, whatever its number of objects, so a scene's figure is
its own pooled mean. Groups follow the MEASURED tier (visibility_from_gt), not
the declared one: a scene whose tier check failed is reported apart.

  scripts/ablation_ci.py                  # the 16 scored scenes of 5-7 Oct
  scripts/ablation_ci.py --json out.json  # also write every number
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys

import numpy as np
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARMS = ["cam_front", "cam_over", "naive", "gated",
        "cam_front+ekf", "cam_over+ekf", "naive+ekf", "fused"]
SCORED = [
    "20261005/t1_cube", "20261005/t1_cube_b", "20261005/t1_bottle", "20261006/t1_bottle_b",
    "20261006/t1_bottle_c", "20261007/t2_clutter_a", "20261007/t2_clutter_b",
    "20261007/t2_clutter_c", "20261005/t3_cube_front", "20261005/t3_cube_over",
    "20261006/t3_bottle_over", "20261006/t3_bottle_over_b", "20261005/t4_cube_front",
    "20261005/t4_cube_over", "20261006/t4_bottle_over", "20261006/t4_bottle_over_b",
]
FLOORS_MM = (2.0, 4.0)


def read_ablation(path):
    rows = {}
    for line in open(path):
        p = line.split()
        if len(p) == 10 and p[0] in ARMS + ["gate-score", "gate-score+ekf"]:
            gt, matched = int(p[4]), int(p[5])
            rows[p[0]] = dict(gt=gt, matched=matched, adds=float(p[7]) if matched else float("nan"),
                              recall=float(p[9]))
    missing = [a for a in ARMS if a not in rows]
    if missing:
        raise SystemExit(f"{path}: no row for {missing}")
    return rows


def tier_check(day, scene):
    """The measured tier, from the line capture_day.sh process printed for it."""
    log = os.path.join(REPO, "runs/scored", day, scene, "visibility.log")
    notes = yaml.safe_load(open(os.path.join(REPO, "captures/scored", day, scene, "notes.yaml")))
    return notes.get("tier"), notes.get("occluded_camera") or "none"


def boot(values, rng, n=10000):
    v = np.asarray([x for x in values if not np.isnan(x)], float)
    if len(v) == 0:
        return float("nan"), (float("nan"), float("nan")), 0
    idx = rng.integers(0, len(v), size=(n, len(v)))
    means = v[idx].mean(axis=1)
    return float(v.mean()), (float(np.percentile(means, 2.5)), float(np.percentile(means, 97.5))), len(v)


def fmt(m, ci, unit=""):
    if np.isnan(m):
        return "—"
    return f"{m:.2f}{unit} [{ci[0]:.2f}, {ci[1]:.2f}]"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenes", nargs="*", default=SCORED)
    ap.add_argument("--uncertified", nargs="*", default=["20261006/t3_bottle_over"],
                    help="scenes whose tier check failed: reported, kept out of their tier's group")
    ap.add_argument("--seed", type=int, default=20261017)
    ap.add_argument("--json", default=None)
    ap.add_argument("--copies", default=None,
                    help="read <dir>/before_<day>_<scene>.ablation.log instead of the scene's own "
                         "ablation.log: a saved earlier state, e.g. before a detector change")
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)

    scenes = []
    for s in args.scenes:
        day, name = s.split("/")
        tier, occluded = tier_check(day, name)
        path = (os.path.join(args.copies, f"before_{day}_{name}.ablation.log") if args.copies
                else os.path.join(REPO, "runs/scored", day, name, "ablation.log"))
        rows = read_ablation(path)
        scenes.append(dict(name=s, tier=tier, occluded=occluded, rows=rows,
                           certified=s not in args.uncertified))

    groups = {"all 16": scenes}
    for t in ("T1", "T2", "T3", "T4"):
        groups[t] = [s for s in scenes if s["tier"] == t and s["certified"]]
    report = {"seed": args.seed, "resamples": 10000, "groups": {}}

    for g, members in groups.items():
        print(f"\n## {g} — {len(members)} scenes: " + ", ".join(m["name"].split("/")[1] for m in members))
        print("| arm | ADD-S mm [95% CI] | scenes with a match | recall@0.1d [95% CI] |")
        print("|---|---|---|---|")
        out = {}
        for arm in ARMS:
            a_m, a_ci, a_n = boot([m["rows"][arm]["adds"] for m in members], rng)
            r_m, r_ci, _ = boot([m["rows"][arm]["recall"] for m in members], rng)
            out[arm] = dict(adds=a_m, adds_ci=a_ci, scenes_matched=a_n, recall=r_m, recall_ci=r_ci)
            print(f"| {arm} | {fmt(a_m, a_ci)} | {a_n} of {len(members)} | {fmt(r_m, r_ci)} |")

        def blocked(m):
            return {"cam_front": "cam_front+ekf", "cam_over": "cam_over+ekf"}.get(m["occluded"])

        def open_view(m):
            return {"cam_front": "cam_over+ekf", "cam_over": "cam_front+ekf"}.get(m["occluded"])

        comparisons = [
            ("fused − cam_front+ekf", lambda m: "cam_front+ekf"),
            ("fused − cam_over+ekf", lambda m: "cam_over+ekf"),
            ("gated − naive (raw)", None),
            ("fused − gated (EKF − raw)", None),
        ]
        if g in ("T3", "T4"):
            comparisons += [("fused − blocked view", blocked), ("fused − open view", open_view)]
        print("\n| difference, scene by scene | ADD-S mm [95% CI] | n | verdict at 2 / 4 mm | recall [95% CI] | verdict |")
        print("|---|---|---|---|---|---|")
        diffs = {}
        for label, pick in comparisons:
            da, dr = [], []
            for m in members:
                if label.startswith("gated − naive"):
                    x, y = m["rows"]["gated"], m["rows"]["naive"]
                elif label.startswith("fused − gated"):
                    x, y = m["rows"]["fused"], m["rows"]["gated"]
                else:
                    arm = pick(m)
                    if arm is None:
                        continue
                    x, y = m["rows"]["fused"], m["rows"][arm]
                da.append(x["adds"] - y["adds"] if x["matched"] and y["matched"] else float("nan"))
                dr.append(x["recall"] - y["recall"])
            a_m, a_ci, a_n = boot(da, rng)
            r_m, r_ci, r_n = boot(dr, rng)
            if np.isnan(a_m):
                av = "no scene where both matched"
            else:
                excl = a_ci[1] < 0 or a_ci[0] > 0
                av = " / ".join(("counts" if excl and abs(a_m) > f else "does not count") for f in FLOORS_MM)
            rv = "—" if np.isnan(r_m) else ("interval excludes 0" if (r_ci[1] < 0 or r_ci[0] > 0) else "includes 0")
            diffs[label] = dict(adds=a_m, adds_ci=a_ci, n=a_n, recall=r_m, recall_ci=r_ci, n_recall=r_n)
            print(f"| {label} | {fmt(a_m, a_ci)} | {a_n} | {av} | {fmt(r_m, r_ci)} | {rv} |")
        report["groups"][g] = dict(scenes=[m["name"] for m in members], arms=out, differences=diffs)

    unc = [s for s in scenes if not s["certified"]]
    if unc:
        print("\nKept out of their tier's group (tier check failed): " +
              ", ".join(f"{s['name']} (declared {s['tier']})" for s in unc))
    if args.json:
        json.dump(report, open(args.json, "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
