#!/usr/bin/env python3
"""predicate_accuracy.py -- the scene graph's facts against hand labels, per predicate.

Every scored scene is still: one arrangement for all of its pairs, so its true
facts are one list, written by hand in docs/predicate_labels.yaml. Facts are
compared by CLASS, not by instance id -- the tracker's ids (red_cube_04) are its
own numbering -- and as multisets, so a second cube reported on a real one is a
false `(on red_cube table)` and a missing one is a missed fact.

Per predicate, over every pair of every scene:
  precision  emitted facts that are true
  recall     true facts that were emitted
  exact      pairs on which the predicate's facts were exactly right

  scripts/predicate_accuracy.py --draft > docs/predicate_labels.yaml   # a first draft
  scripts/predicate_accuracy.py                                        # score it

--draft writes what the ground truth implies, for a person to check and correct:
each object with a tray pose is on the table and clear; (in red_cube bowl) where
the scene was staged so; (occluded_from obj view) where that view sees under 25%
of the object (proposal §7.1's definition of occluded, from visibility.csv). It
is a starting point, not a label: the file is only hand labels once a person has
read it against the scene's photos and stills.
"""
from __future__ import annotations

import argparse
import collections
import csv
import os
import re
import sqlite3
import sys

import numpy as np
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LABELS = os.path.join(REPO, "docs/predicate_labels.yaml")
SCENES = [
    "20261005/t1_cube", "20261005/t1_cube_b", "20261005/t1_bottle", "20261006/t1_bottle_b",
    "20261006/t1_bottle_c", "20261007/t2_clutter_a", "20261007/t2_clutter_b",
    "20261007/t2_clutter_c", "20261005/t3_cube_front", "20261005/t3_cube_over",
    "20261006/t3_bottle_over", "20261006/t3_bottle_over_b", "20261005/t4_cube_front",
    "20261005/t4_cube_over", "20261006/t4_bottle_over", "20261006/t4_bottle_over_b",
    "20261010/t2_in_a", "20261010/t2_in_b",
    "20261005/dim_t1_cube", "20261005/dim_t1_bottle_gain128", "20261005/dim_t3_bottle_over",
    "20261006/t2_clutter_a",
]
ID = re.compile(r"_\d+$")
PREDICATES = ["on", "in", "clear", "occluded_from"]


def by_class(fact: str) -> str:
    """(on red_cube_04 table_01) -> (on red_cube table)"""
    head, *args = fact.strip("()").split()
    return "(" + " ".join([head] + [ID.sub("", a) for a in args]) + ")"


def draft() -> int:
    out = {"_how": "Hand labels for predicate accuracy (scripts/predicate_accuracy.py). One list of true "
                   "facts per scene, by class. DRAFTED from ground truth; a scene is a label only once "
                   "checked: set checked: true after reading it against the scene's stills.",
           "scenes": {}}
    for s in SCENES:
        day, name = s.split("/")
        run = os.path.join(REPO, "runs/scored", day, name)
        notes = yaml.safe_load(open(os.path.join(REPO, "captures/scored", day, name, "notes.yaml")))
        con = sqlite3.connect(f"file:{run}/metrics.sqlite?mode=ro", uri=True)
        r = con.execute("select max(run_id) from runs").fetchone()[0]
        classes = sorted({c for (c,) in con.execute(
            "select distinct cls from ground_truth where run_id=?", (r,))})
        vis = collections.defaultdict(list)
        with open(os.path.join(run, "visibility.csv")) as f:
            for row in csv.DictReader(f):
                vis[(row["cls"], row["camera_id"])].append(float(row["visible_fraction"]))
        facts = []
        in_scene = "_in_" in name
        for c in classes:
            facts += [f"(on {c} table)", f"(clear {c})"]
            for cam in ("cam_front", "cam_over"):
                v = vis.get((c, cam))
                if v and float(np.nanmedian(v)) < 0.25:
                    facts.append(f"(occluded_from {c} {cam})")
        if in_scene:
            facts += ["(in red_cube bowl)", "(clear red_cube)"]
        out["scenes"][s] = {"checked": False, "tier": notes.get("tier"),
                            "note": (notes.get("note") or "")[:160], "facts": sorted(facts)}
    yaml.safe_dump(out, sys.stdout, sort_keys=False, width=110, allow_unicode=True)
    return 0


def score(only_checked: bool) -> int:
    labels = yaml.safe_load(open(LABELS))["scenes"]
    tp, fp, fn = collections.Counter(), collections.Counter(), collections.Counter()
    exact, pairs = collections.Counter(), collections.Counter()
    per_scene = []
    for s, lab in labels.items():
        if only_checked and not lab.get("checked"):
            continue
        day, name = s.split("/")
        db = os.path.join(REPO, "runs/scored", day, name, "metrics.sqlite")
        con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        r = con.execute("select max(run_id) from runs").fetchone()[0]
        emitted = collections.defaultdict(list)
        for p, f in con.execute("select pair_index, fact from facts where run_id=?", (r,)):
            emitted[p].append(by_class(f))
        frames = [p for (p,) in con.execute("select pair_index from frames where run_id=?", (r,))]
        want = collections.Counter(lab["facts"])
        bad = 0
        for p in frames:
            got = collections.Counter(emitted.get(p, []))
            right = True
            for pred in PREDICATES:
                w = collections.Counter({f: n for f, n in want.items() if f.startswith(f"({pred} ")})
                g = collections.Counter({f: n for f, n in got.items() if f.startswith(f"({pred} ")})
                t = sum((w & g).values())
                tp[pred] += t
                fp[pred] += sum(g.values()) - t
                fn[pred] += sum(w.values()) - t
                pairs[pred] += 1
                if w == g:
                    exact[pred] += 1
                else:
                    right = False
            bad += 0 if right else 1
        per_scene.append((s, len(frames), bad))
    print("| predicate | true facts | emitted | correct | precision | recall | pairs exactly right |")
    print("|---|---|---|---|---|---|---|")
    for pred in sorted(set(tp) | set(fp) | set(fn)):
        e, w = tp[pred] + fp[pred], tp[pred] + fn[pred]
        prec = tp[pred] / e if e else float("nan")
        rec = tp[pred] / w if w else float("nan")
        print(f"| {pred} | {w} | {e} | {tp[pred]} | {prec:.4f} | {rec:.4f} | "
              f"{exact[pred]} of {pairs[pred]} |")
    print()
    for s, n, bad in per_scene:
        print(f"  {s:34s} {n - bad:4d} of {n} pairs with every fact right")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--draft", action="store_true")
    ap.add_argument("--all", action="store_true", help="score unchecked scenes too")
    a = ap.parse_args()
    return draft() if a.draft else score(not a.all)


if __name__ == "__main__":
    sys.exit(main())
