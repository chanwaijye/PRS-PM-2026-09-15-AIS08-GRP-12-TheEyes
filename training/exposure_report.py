#!/usr/bin/env python3
"""Brightness distribution of a recorded session, against the training set.

WHY THIS EXISTS
---------------
`replicator_dataset.py` carries a [VERIFY] note that the synthetic set renders
bright -- RGB mean ~200/255, p1 = 132, 14% of pixels saturated -- and says the
right fix is "to match the exposure against a real capture once the rig is
built". The rig is built. This is the other half of that comparison.

It is not a diagnostic aside. The detector's classes are learned on a
distribution whose DARKEST one per cent is 132, so a real capture sitting mostly
below that is out of distribution before any question of colour, viewpoint or
occlusion arises, and the false positives concentrate on whatever in the room is
bright enough to resemble training data.

Re-run it after any lighting change; that is the point of it being a script
rather than a number in a document.

    ~/venvs/the-eyes-r1/bin/python training/exposure_report.py \
        captures/runtime/<session>
"""

from __future__ import annotations

import argparse
import glob
from pathlib import Path

import cv2
import numpy as np
import yaml

# From replicator_dataset.py's [VERIFY] note, measured on synth50k_v2.
SYNTH = {"mean": 200.0, "p1": 132.0, "saturated_pct": 14.0}


def stats(paths: list[str], stride: int) -> dict[str, float]:
    v = np.concatenate([cv2.imread(p).reshape(-1) for p in paths[::stride]])
    return {
        "n": len(paths[::stride]),
        "mean": float(v.mean()),
        "p1": float(np.percentile(v, 1)),
        "p50": float(np.percentile(v, 50)),
        "p99": float(np.percentile(v, 99)),
        "saturated_pct": float(100 * (v >= 254).mean()),
        # The one that matters: how much of the frame falls below the training
        # set's floor. Not a threshold anyone chose -- it is synth50k_v2's p1.
        "below_synth_p1_pct": float(100 * (v < SYNTH["p1"]).mean()),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--stride", type=int, default=8, help="sample every Nth frame")
    a = ap.parse_args()

    meta = yaml.safe_load((a.session / "session.yaml").read_text())
    print(f"session: {a.session}  ({meta.get('pairs_written', '?')} pairs)\n")
    hdr = f"{'':12s}{'frames':>7s}{'mean':>7s}{'p1':>6s}{'p50':>6s}{'p99':>6s}{'%sat':>7s}{'% < 132':>9s}"
    print(hdr)
    print(f"{'synth50k_v2':12s}{'-':>7s}{SYNTH['mean']:>7.0f}{SYNTH['p1']:>6.0f}"
          f"{'-':>6s}{'-':>6s}{SYNTH['saturated_pct']:>7.1f}{'~1':>9s}   <- what it was trained on")
    worst = 0.0
    for cam in meta["camera_ids"]:
        files = sorted(glob.glob(str(a.session / cam / "*.rgb.png")))
        if not files:
            print(f"{cam:12s}  no rgb frames")
            continue
        s = stats(files, a.stride)
        worst = max(worst, s["below_synth_p1_pct"])
        print(f"{cam:12s}{s['n']:>7d}{s['mean']:>7.1f}{s['p1']:>6.0f}{s['p50']:>6.0f}"
              f"{s['p99']:>6.0f}{s['saturated_pct']:>7.1f}{s['below_synth_p1_pct']:>9.1f}")
    print()
    if worst > 50.0:
        print(f"OUT OF DISTRIBUTION: {worst:.0f}% of pixels fall below the training set's "
              f"1st percentile.\nLight the scene before reading anything into detector "
              f"behaviour on these frames.")
    else:
        print(f"Worst camera has {worst:.0f}% of pixels below the training set's 1st "
              f"percentile.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
