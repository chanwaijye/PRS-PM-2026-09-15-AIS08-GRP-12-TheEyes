#!/usr/bin/env python3
"""report_figures.py -- the final report's charts, drawn from the recorded results.

Every number in a figure is read from the same files the report's tables come
from; nothing is typed in here:

  fig_tiers.png         runs/scored/<scene>/visibility_overlay + visibility.csv
  fig_detector_real.png runs/detector_real/*.json
  fig_sim2real.png      docs/results/{dataset_size,modality}*.json, runs/detector_real/*.json
  fig_scenes.png        runs/scored/<scene>/ablation.log (the per-scene table, §9.4)
  fig_arms.png          docs/results/ablation_ci.json
  fig_differences.png   docs/results/ablation_ci.json
  fig_evidence.png      runs/scored/<scene>/metrics.sqlite (latest run) + extrinsics
  fig_predicates.png    scripts/predicate_accuracy.py

Read-only apart from the PNGs; no rig, no GPU.

  scripts/report_figures.py                   # all, into ProjectReport/figures/
  scripts/report_figures.py --only scenes     # one
"""
from __future__ import annotations

import argparse
import csv
import collections
import json
import os
import re
import sqlite3
import subprocess
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "scripts"))
from ablation_ci import SCORED, read_ablation  # noqa: E402

OUT = os.path.join(REPO, "ProjectReport/figures")
CAMS = ("cam_front", "cam_over")
FLOORS_MM = (2.0, 4.0)

# Okabe-Ito: readable in colour-blind vision and in greyscale print.
C = {"cam_front": "#0072B2", "cam_over": "#E69F00", "naive": "#999999",
     "gate-score+ekf": "#CC79A7", "gated": "#56B4E9", "fused": "#009E73",
     "bad": "#D55E00", "ink": "#222222"}
LABEL = {"cam_front": "cam_front only", "cam_over": "cam_over only",
         "naive": "naive average", "gated": "gated", "gate-score+ekf": "gate by score + EKF",
         "fused": "fused (gated + EKF)", "cam_front+ekf": "cam_front + EKF",
         "cam_over+ekf": "cam_over + EKF", "naive+ekf": "naive + EKF"}

plt.rcParams.update({
    "font.size": 9, "axes.titlesize": 10, "axes.labelsize": 9, "legend.fontsize": 8,
    "xtick.labelsize": 8, "ytick.labelsize": 8, "axes.spines.top": False,
    "axes.spines.right": False, "figure.dpi": 100, "savefig.dpi": 200,
    "savefig.bbox": "tight", "savefig.pad_inches": 0.05,
})
FULL_W = 6.7  # inches: the A4 text width at 20 mm margins


def save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    fig.savefig(path)
    plt.close(fig)
    print("wrote", os.path.relpath(path, REPO))


def scene_name(s):
    return s.split("/")[1]


def jload(path):
    return json.load(open(os.path.join(REPO, path)))


# --------------------------------------------------------------------------
# The four occlusion tiers, as the cameras saw them
# --------------------------------------------------------------------------

TIER_SCENES = [
    ("T1  unoccluded", "20261005/t1_cube"),
    ("T2  clutter", "20261007/t2_clutter_b"),
    ("T3  half hidden from cam_over", "20261006/t3_bottle_over_b"),
    ("T4  blocked from cam_over", "20261006/t4_bottle_over_b"),
]


def visible(scene):
    """Median visible fraction of each object's silhouette, per camera, as visibility.log bands it."""
    v = collections.defaultdict(list)
    for r in csv.DictReader(open(os.path.join(REPO, "runs/scored", scene, "visibility.csv"))):
        v[(r["camera_id"], r["cls"])].append(float(r["visible_fraction"]))
    return {k: float(np.median(x)) for k, x in v.items()}


def fig_tiers():
    import cv2
    fig, axes = plt.subplots(len(TIER_SCENES), 2, figsize=(FULL_W, 8.6))
    for row, (title, scene) in enumerate(TIER_SCENES):
        vis = visible(scene)
        for col, cam in enumerate(CAMS):
            ax = axes[row, col]
            im = cv2.imread(os.path.join(REPO, "runs/scored", scene, "visibility_overlay",
                                         f"{cam}_pair30.png"))
            ax.imshow(cv2.cvtColor(im, cv2.COLOR_BGR2RGB))
            ax.set_xticks([]), ax.set_yticks([])
            for s in ax.spines.values():
                s.set_visible(False)
            seen = {cls: f for (c, cls), f in vis.items() if c == cam}
            least = min(seen.values())
            txt = (f"sees {least:.0%} of the object" if len(seen) == 1
                   else f"sees each of its {len(seen)} objects whole" if least >= 0.995
                   else f"sees at least {least:.0%} of each of {len(seen)} objects")
            ax.set_title(f"{cam}: {txt}", fontsize=8, loc="left")
            if col == 0:
                ax.set_ylabel(f"{title}\n{scene_name(scene)}", fontsize=8.5)
    # visibility_from_gt's Tint(), BGR there.
    handles = [matplotlib.patches.Patch(color=c, label=l) for c, l in
               (("#3CC83C", "depth agrees: seen"), ("#E62828", "something nearer: hidden"),
                ("#E6DC1E", "no depth reading (counted as seen)"))]
    fig.legend(handles=handles, loc="lower center", ncol=3, frameon=False,
               bbox_to_anchor=(0.5, -0.01))
    fig.subplots_adjust(hspace=0.22, wspace=0.03, bottom=0.04)
    save(fig, "fig_tiers.png")


# --------------------------------------------------------------------------
# Detector
# --------------------------------------------------------------------------

DETECTORS = [
    ("RGB only,\nsynthetic data", "runs/detector_real/baseline_rgbonly_lit.json", "#BBBBBB"),
    ("RGB-D,\nsynthetic data", "runs/detector_real/baseline_rgbd_lit.json", C["cam_front"]),
    ("RGB-D, fine-tuned on\nreal layouts (shipped)", "runs/detector_real/ft_v1_best_real_v1.json", C["fused"]),
]


def fig_detector_real():
    metrics = [("mAP@0.5", lambda d: d["mAP50"]), ("mAP@0.5:0.95", lambda d: d["mAP50_95"]),
               ("precision", lambda d: d["precision"]), ("recall", lambda d: d["recall"]),
               ("cube AP@0.5", lambda d: d["per_class_mAP50"]["red_cube"]),
               ("cube recall", lambda d: d["per_class_recall"]["red_cube"])]
    fig, ax = plt.subplots(figsize=(FULL_W, 2.7))
    x = np.arange(len(metrics))
    w = 0.27
    for i, (name, path, colour) in enumerate(DETECTORS):
        d = jload(path)
        vals = [f(d) for _, f in metrics]
        bars = ax.bar(x + (i - 1) * w, vals, w, label=name.replace("\n", " "), color=colour)
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v + 0.015, f"{v:.2f}", ha="center",
                    va="bottom", fontsize=6.5)
    ax.set_xticks(x, [m for m, _ in metrics])
    ax.set_ylim(0, 1.12)
    ax.set_yticks([0, 0.25, 0.5, 0.75, 1.0])
    ax.set_ylabel("score")
    ax.legend(loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.5, 1.12))
    save(fig, "fig_detector_real.png")


def fig_sim2real():
    size = jload("docs/results/dataset_size_ablation_rgbd_50k_v2.json")
    mod = jload("docs/results/modality_rgb_vs_rgbd_50k_v2.json")
    real = {k: jload(f"runs/detector_real/{k}.json")
            for k in ("baseline_rgbonly_lit", "baseline_rgbd_lit", "ft_v1_best_real_v1",
                      "ft_v1_best_synthval")}
    fig, (a, b) = plt.subplots(1, 2, figsize=(FULL_W, 2.6), gridspec_kw={"width_ratios": [1, 1.5]})

    arms = sorted((v["train_frames"], v["mAP50_95"]) for k, v in size.items() if k != "_provenance")
    a.plot([f / 1000 for f, _ in arms], [m for _, m in arms], "o-", color=C["cam_front"])
    for f, m in arms:
        a.annotate(f"{m:.3f}", (f / 1000, m), textcoords="offset points", xytext=(0, 6),
                   ha="center", fontsize=7)
    a.set_xlabel("synthetic training frames (thousands)")
    a.set_ylabel("mAP@0.5:0.95 (synthetic val.)")
    a.set_title("(a) More synthetic data", fontsize=9)
    a.set_ylim(0.90, 0.975)

    pairs = [("RGB only,\nsynthetic", mod["rgb_only"]["mAP50_95"], real["baseline_rgbonly_lit"]["mAP50_95"]),
             ("RGB-D,\nsynthetic", mod["rgbd"]["mAP50_95"], real["baseline_rgbd_lit"]["mAP50_95"]),
             ("RGB-D, fine-tuned\non 360 real frames", real["ft_v1_best_synthval"]["mAP50_95"],
              real["ft_v1_best_real_v1"]["mAP50_95"])]
    x = np.arange(len(pairs))
    w = 0.36
    for off, idx, colour, lab in ((-w / 2, 1, "#BBBBBB", "synthetic validation"),
                                  (w / 2, 2, C["fused"], "real frames, 16 scenes")):
        vals = [p[idx] for p in pairs]
        bars = b.bar(x + off, vals, w, color=colour, label=lab)
        for bar, v in zip(bars, vals):
            b.text(bar.get_x() + bar.get_width() / 2, v + 0.01, f"{v:.3f}", ha="center",
                   va="bottom", fontsize=7)
    gap = [f"{p[2] - p[1]:+.3f}".replace("-", "−") for p in pairs]
    b.set_xticks(x, [f"{p[0]}\ngap {g}" for p, g in zip(pairs, gap)])
    b.set_ylim(0, 1.25)
    b.set_yticks([0, 0.2, 0.4, 0.6, 0.8, 1.0])
    b.set_ylabel("mAP@0.5:0.95")
    b.set_title("(b) Synthetic to real, before and after fine-tuning", fontsize=9)
    b.legend(loc="upper center", ncol=2, frameon=False, bbox_to_anchor=(0.5, 1.0))
    fig.tight_layout(w_pad=2)
    save(fig, "fig_sim2real.png")


# --------------------------------------------------------------------------
# The occlusion ablation
# --------------------------------------------------------------------------

SCENE_ARMS = ["cam_front", "cam_over", "naive", "gate-score+ekf", "fused"]
UNCERTIFIED = {"20261006/t3_bottle_over"}


def fig_scenes():
    rows = [(s, read_ablation(os.path.join(REPO, "runs/scored", s, "ablation.log"))) for s in SCORED]
    fig, ax = plt.subplots(figsize=(FULL_W, 5.4))
    y = np.arange(len(rows))[::-1]
    lo, hi, miss_x = 1.0, 100.0, 160.0
    ax.axvspan(*FLOORS_MM, color="#EEEEEE", zorder=0)
    ax.text(np.sqrt(FLOORS_MM[0] * FLOORS_MM[1]), -0.62, "sensor floor", ha="center",
            va="bottom", fontsize=7, color="#666666")
    markers = {"cam_front": "o", "cam_over": "s", "naive": "D", "gate-score+ekf": "v", "fused": "*"}
    offs = dict(zip(SCENE_ARMS, np.linspace(-0.28, 0.28, len(SCENE_ARMS))))
    for yi, (scene, r) in zip(y, rows):
        for arm in SCENE_ARMS:
            row = r[arm]
            yy = yi + offs[arm]
            size = 90 if arm == "fused" else 26
            if row["matched"] == 0:
                ax.scatter(miss_x, yy, marker="x", s=24, color=C[arm], zorder=3)
                continue
            filled = row["recall"] >= 0.5
            ax.scatter(row["adds"], yy, marker=markers[arm], s=size, zorder=4 if arm == "fused" else 3,
                       facecolor=C[arm] if filled else "white", edgecolor=C[arm], linewidth=1)
    ax.axvline(120, color="#CCCCCC", lw=0.8, ls=":")
    ax.set_xscale("log")
    ax.set_xlim(lo, 200)
    ax.set_xticks([1, 2, 4, 10, 20, 50, 100, miss_x], ["1", "2", "4", "10", "20", "50", "100", "no\nmatch"])
    ax.set_yticks(y, [scene_name(s) + (" *" if s in UNCERTIFIED else "") for s, _ in rows])
    ax.set_ylim(-0.7, len(rows) - 0.1)
    for b in (4.5, 7.5, 11.5):
        ax.axhline(len(rows) - 1 - b, color="#DDDDDD", lw=0.8)
    ax.set_xlabel("mean ADD-S over the frames matched (mm, log scale)")
    handles = [plt.Line2D([], [], marker=markers[a], ls="", color=C[a], markersize=9 if a == "fused" else 6,
                          label=LABEL[a]) for a in SCENE_ARMS]
    handles.append(plt.Line2D([], [], marker="o", ls="", markerfacecolor="white", markeredgecolor="#555555",
                              label="hollow: recall@0.1d under 50%"))
    ax.legend(handles=handles, loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.45, 1.12))
    ax.grid(axis="x", color="#F0F0F0", zorder=0)
    save(fig, "fig_scenes.png")


ARM_ORDER = ["cam_front", "cam_over", "naive", "gated",
             "cam_front+ekf", "cam_over+ekf", "naive+ekf", "fused"]


def fig_arms():
    g = jload("docs/results/ablation_ci.json")["groups"]["all 16"]["arms"]
    fig, (a, b) = plt.subplots(1, 2, figsize=(FULL_W, 2.9), sharey=True)
    y = np.arange(len(ARM_ORDER))[::-1]
    for yi, arm in zip(y, ARM_ORDER):
        v = g[arm]
        colour = C.get(arm.replace("+ekf", ""), C["ink"]) if arm != "fused" else C["fused"]
        for ax, m, ci in ((a, v["adds"], v["adds_ci"]), (b, v["recall"], v["recall_ci"])):
            ax.errorbar(m, yi, xerr=[[m - ci[0]], [ci[1] - m]], fmt="*" if arm == "fused" else "o",
                        color=colour, markersize=10 if arm == "fused" else 5, capsize=3, lw=1.2)
        a.annotate(f"{v['adds']:.2f}", (v["adds_ci"][1], yi), textcoords="offset points",
                   xytext=(4, -3), fontsize=7)
        b.annotate(f"{v['recall']:.2f}", (v["recall_ci"][0], yi), textcoords="offset points",
                   xytext=(-4, -3), fontsize=7, ha="right")
    a.axvspan(*FLOORS_MM, color="#EEEEEE", zorder=0)
    a.set_xscale("log")
    a.set_xlim(1.5, 80)
    a.set_xticks([2, 4, 10, 20, 50], ["2", "4", "10", "20", "50"])
    a.set_xlabel("log scale; frames the arm matched")
    a.set_yticks(y, [LABEL[x] + (f"  ({g[x]['scenes_matched']} of 16 scenes)" if g[x]["scenes_matched"] < 16
                                 else "") for x in ARM_ORDER])
    a.axhline(3.5, color="#DDDDDD", lw=0.8)
    b.axhline(3.5, color="#DDDDDD", lw=0.8)
    b.set_xlim(0.2, 1.05)
    b.set_xlabel("a miss counts as a failure")
    a.set_title("(a) ADD-S, mm, 95% CI", fontsize=9)
    b.set_title("(b) recall@0.1d, 95% CI", fontsize=9)
    fig.tight_layout(w_pad=2)
    save(fig, "fig_arms.png")


def fig_differences():
    gs = jload("docs/results/ablation_ci.json")["groups"]
    rows = [  # (label, group, key); paired scene by scene
        ("fused − the blocked camera (T4)", "T4", "fused − blocked view"),
        ("fused − the half-hidden camera (T3)", "T3", "fused − blocked view"),
        ("fused − cam_over + EKF (all 16)", "all 16", "fused − cam_over+ekf"),
        ("gated − naive average (all 16)", "all 16", "gated − naive (raw)"),
        ("fused − cam_front + EKF (all 16)", "all 16", "fused − cam_front+ekf"),
        ("      the same, T1 only", "T1", "fused − cam_front+ekf"),
        ("      the same, T2 only", "T2", "fused − cam_front+ekf"),
        ("fused − the open camera (T3)", "T3", "fused − open view"),
        ("EKF − raw: fused − gated (all 16)", "all 16", "fused − gated (EKF − raw)"),
    ]
    fig, (a, b) = plt.subplots(1, 2, figsize=(FULL_W, 3.3), sharey=True,
                               gridspec_kw={"width_ratios": [1.3, 1]})
    y = np.arange(len(rows))[::-1]
    a.axvspan(-FLOORS_MM[1], FLOORS_MM[1], color="#F3F3F3", zorder=0)
    a.axvspan(-FLOORS_MM[0], FLOORS_MM[0], color="#E3E3E3", zorder=0)
    for ax in (a, b):
        ax.axvline(0, color="#888888", lw=0.8)
    for yi, (lab, grp, key) in zip(y, rows):
        d = gs[grp]["differences"][key]
        m, (lo, hi) = d["adds"], d["adds_ci"]
        if d.get("n", 0) == 0 or m is None or np.isnan(m):
            a.text(-70, yi, "no ADD-S: the blocked camera matched nothing", va="center", ha="left",
                   fontsize=7, color="#666666")
        else:
            counts = (lo > 0 or hi < 0) and abs(m) > FLOORS_MM[0]
            colour = C["fused"] if counts and m < 0 else (C["bad"] if counts else "#555555")
            a.errorbar(m, yi, xerr=[[m - lo], [hi - m]], fmt="o", color=colour, capsize=3, markersize=5)
            a.annotate(f"{m:+.2f}", (m, yi), textcoords="offset points", xytext=(0, 5), ha="center",
                       fontsize=7)
        m, (lo, hi) = d["recall"], d["recall_ci"]
        colour = C["fused"] if lo > 0 else ("#555555" if hi >= 0 else C["bad"])
        b.errorbar(m, yi, xerr=[[m - lo], [hi - m]], fmt="o", color=colour, capsize=3, markersize=5)
        b.annotate(f"{m:+.2f}", (m, yi), textcoords="offset points", xytext=(0, 5), ha="center",
                   fontsize=7)
    a.set_xscale("symlog", linthresh=4, linscale=1.6)
    a.set_xlim(-80, 6)
    a.set_xticks([-40, -10, -2, 0, 2, 4], ["−40", "−10", "−2", "0", "2", "4"])
    a.set_yticks(y, [r[0] for r in rows])
    a.set_xlabel("below 0: fusion more accurate")
    a.set_title("(a) ADD-S difference, mm, 95% CI", fontsize=9)
    b.set_xlim(-0.35, 1.12)
    b.set_xlabel("above 0: fusion finds more")
    b.set_title("(b) recall@0.1d difference, 95% CI", fontsize=9)
    fig.tight_layout(w_pad=1.5)
    save(fig, "fig_differences.png")


# --------------------------------------------------------------------------
# Depth evidence
# --------------------------------------------------------------------------

def evidence_rows():
    """Every logged per-view estimate of the 16 scenes: its position error and seen_through."""
    out = []
    for s in SCORED:
        cdir = os.path.join(REPO, "captures/scored", s)
        ext = yaml.safe_load(open(os.path.join(cdir, "extrinsics_at_recording.yaml")))["cameras"]
        T = {c: np.array(ext[c]["T_world_cam"], float).reshape(4, 4) for c in CAMS}
        db = sqlite3.connect(f"file:{os.path.join(REPO, 'runs/scored', s, 'metrics.sqlite')}?mode=ro", uri=True)
        run = db.execute("select max(run_id) from runs").fetchone()[0]
        truth = collections.defaultdict(list)
        for pair, cls, tx, ty, tz in db.execute(
                "select pair_index, cls, tx, ty, tz from ground_truth where run_id=?", (run,)):
            truth[(pair, cls)].append((tx, ty, tz))
        for pair, cam, cls, tx, ty, tz, st in db.execute(
                "select pair_index, camera_id, cls, tx, ty, tz, seen_through from estimates "
                "where run_id=?", (run,)):
            if st is None:
                continue
            p_w = T[cam][:3, :3] @ np.array([tx, ty, tz]) + T[cam][:3, 3]
            gts = truth.get((pair, cls))
            err = min(np.linalg.norm(p_w - np.array(g)) for g in gts) * 1000 if gts else np.inf
            out.append((s, cam, cls, err, st))
    return out


def fig_evidence(veto=1.0 / 3.0, right_mm=30.0, ghost_x=4000.0):
    rows = evidence_rows()
    err = np.array([r[3] for r in rows])
    st = np.array([r[4] for r in rows])
    tier = np.array([scene_name(r[0])[:2] for r in rows])
    right = err < right_mm
    ghost = ~np.isfinite(err)
    wrong = ~right & ~ghost
    groups = [
        (right, C["fused"], 3, 0.25, f"within {right_mm:.0f} mm of the truth"),
        (wrong & (tier == "t3"), C["bad"], 6, 0.5, "more than 30 mm off, T3: the half-hidden view"),
        (wrong & (tier == "t4"), C["gate-score+ekf"], 6, 0.5, "more than 30 mm off, T4: mostly the blocked view"),
        (ghost, "#777777", 6, 0.5, "a class that is not in the scene"),
    ]
    print(f"  evidence: {len(rows)} estimates")
    for m, _, _, _, lab in groups:
        print(f"    {lab}: {m.sum()}, vetoed {(st[m] >= veto).sum()}, max seen_through {st[m].max():.3f}")

    fig, ax = plt.subplots(figsize=(FULL_W, 3.1))
    jitter = np.random.default_rng(0).uniform(-0.006, 0.006, len(rows))
    x = np.where(ghost, ghost_x * np.exp(np.random.default_rng(1).uniform(-0.12, 0.12, len(rows))),
                 np.clip(err, 0.6, None))
    ax.axhspan(veto, 1.05, color="#FBE9E1", zorder=0)
    ax.axhline(veto, color=C["bad"], lw=1, ls="--")
    ax.text(0.65, veto + 0.025, "vetoed: the other camera sees past a third or more of the claimed surface",
            color=C["bad"], fontsize=7.5)
    ax.axvline(right_mm, color="#999999", lw=0.8, ls=":")
    for m, colour, size, alpha, lab in groups:
        ax.scatter(x[m], st[m] + jitter[m], s=size, alpha=alpha, color=colour, rasterized=True,
                   label=f"{lab} ({m.sum():,}; {(st[m] >= veto).sum():,} vetoed)")
    ax.set_xscale("log")
    ax.set_xlim(0.6, 7000)
    ax.set_xticks([1, 3, 10, 30, 100, 300, 1000, ghost_x],
                  ["1", "3", "10", "30", "100", "300", "1000", "not in\nscene"])
    ax.set_ylim(-0.03, 1.05)
    ax.set_xlabel("position error of the per-view estimate (mm, log scale)")
    ax.set_ylabel("seen_through")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=2, frameon=False, markerscale=2.5,
              fontsize=7)
    save(fig, "fig_evidence.png")


# --------------------------------------------------------------------------
# Predicates
# --------------------------------------------------------------------------

def fig_predicates():
    out = subprocess.run([sys.executable, os.path.join(REPO, "scripts/predicate_accuracy.py")],
                         capture_output=True, text=True, check=True).stdout
    rows = {}
    for line in out.splitlines():
        p = [x.strip() for x in line.strip("|").split("|")]
        if len(p) == 7 and re.fullmatch(r"[a-z_]+", p[0]) and p[0] != "predicate":
            rows[p[0]] = (float(p[4]), float(p[5]))
    order = [k for k in ("on", "clear", "in", "occluded_from") if k in rows]
    fig, ax = plt.subplots(figsize=(FULL_W, 2.3))
    x = np.arange(len(order))
    w = 0.36
    for off, i, colour, lab in ((-w / 2, 0, C["cam_front"], "precision"), (w / 2, 1, C["fused"], "recall")):
        vals = [rows[k][i] for k in order]
        bars = ax.bar(x + off, vals, w, color=[C["bad"] if v < 0.95 else colour for v in vals], label=lab)
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v + 0.01, f"{v:.3f}", ha="center", va="bottom",
                    fontsize=7)
    ax.axhline(0.95, color="#555555", lw=0.8, ls="--")
    ax.set_xticks(x, [f"({k} …)" for k in order])
    ax.set_ylim(0, 1.12)
    ax.set_yticks([0, 0.25, 0.5, 0.75, 1.0])
    ax.set_xlim(-0.6, len(order) - 0.4)
    handles = [matplotlib.patches.Patch(color=C["cam_front"], label="precision"),
               matplotlib.patches.Patch(color=C["fused"], label="recall"),
               matplotlib.patches.Patch(color=C["bad"], label="under target"),
               plt.Line2D([], [], color="#555555", lw=0.8, ls="--", label="target 0.95")]
    ax.legend(handles=handles, loc="upper center", frameon=False, ncol=4, bbox_to_anchor=(0.5, 1.16))
    save(fig, "fig_predicates.png")


FIGS = {"tiers": fig_tiers, "detector": fig_detector_real, "sim2real": fig_sim2real,
        "scenes": fig_scenes, "arms": fig_arms, "differences": fig_differences,
        "evidence": fig_evidence, "predicates": fig_predicates}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", choices=sorted(FIGS), action="append")
    args = ap.parse_args()
    for name in args.only or FIGS:
        FIGS[name]()
    return 0


if __name__ == "__main__":
    sys.exit(main())
