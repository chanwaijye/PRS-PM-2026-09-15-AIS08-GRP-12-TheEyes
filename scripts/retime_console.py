#!/usr/bin/env python3
"""retime_console.py -- a live console recording, played back at the speed it happened.

`the_eyes_run live --show-record F` writes one frame per PROCESSED pair into a
file stamped 30 fps, so a take that processed 12 pairs a second plays 2.5 times
too fast, and the pauses -- a pose registration of about a second -- vanish.
This gives each console frame the camera time of its pair (frames.stamp_ns in
the take's metrics.sqlite) and writes a constant 30 fps H.264 file in which
frame k shows the newest console frame whose pair is no later than k/30 s.

    scripts/retime_console.py runs/video/demo_live_a.avi demo_live_a out.mp4

The take is found as runs/runtime/*_<take>/metrics.sqlite (its newest run).
Frame i of the file is pair i of the run in pair order; the counts must agree,
or nothing is written.
"""
from __future__ import annotations

import argparse
import glob
import os
import sqlite3
import subprocess
import sys

import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("avi")
    ap.add_argument("take")
    ap.add_argument("out")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--crf", type=int, default=18)
    a = ap.parse_args()

    dbs = sorted(glob.glob(os.path.join(REPO, f"runs/runtime/*_{a.take}/metrics.sqlite")))
    if not dbs:
        sys.exit(f"no runs/runtime/*_{a.take}/metrics.sqlite")
    con = sqlite3.connect(f"file:{dbs[-1]}?mode=ro", uri=True)
    run = con.execute("select max(run_id) from runs").fetchone()[0]
    stamps = np.array([s for (s,) in con.execute(
        "select stamp_ns from frames where run_id=? order by pair_index", (run,))], dtype=np.int64)
    t = (stamps - stamps[0]) / 1e9

    cap = cv2.VideoCapture(a.avi)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if n != len(t):
        sys.exit(f"{a.avi} has {n} frames, the run has {len(t)} pairs: not the same take")

    out_frames = int(np.floor(t[-1] * a.fps)) + 1
    enc = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{w}x{h}",
         "-r", str(a.fps), "-i", "-", "-c:v", "libx264", "-crf", str(a.crf), "-preset", "medium",
         "-pix_fmt", "yuv420p", "-movflags", "+faststart", a.out], stdin=subprocess.PIPE)
    i, ok, frame = 0, *cap.read()
    if not ok:
        sys.exit(f"cannot read {a.avi}")
    for k in range(out_frames):
        now = k / a.fps
        while i + 1 < n and t[i + 1] <= now:
            ok, nxt = cap.read()
            if not ok:
                sys.exit(f"{a.avi}: frame {i + 1} unreadable")
            frame, i = nxt, i + 1
        enc.stdin.write(frame.tobytes())
    enc.stdin.close()
    if enc.wait() != 0:
        sys.exit("ffmpeg failed")
    print(f"{a.out}: {n} console frames over {t[-1]:.1f} s of camera time -> {out_frames} frames at "
          f"{a.fps:g} fps ({out_frames / a.fps:.1f} s); the file it came from played in {n / 30:.1f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
