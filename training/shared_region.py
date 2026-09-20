#!/usr/bin/env python3
"""The tabletop region both cameras can see, from the committed extrinsics.

WHY THIS EXISTS
---------------
`RIG_BOM.md`'s "horizontal coverage 981 mm" is ONE camera's extent. What governs
a two-view experiment is the intersection of two views ~80 deg apart, and that
is far smaller. An object outside it is single-view *by construction* -- not
occluded, just out of frame -- and scoring that as occlusion would manufacture
exactly the result the ablation exists to test (`CAPTURE_PROTOCOL.md` 4.2d).

The number moves whenever the rig moves, which on this rig has been three times
in two weeks. So it is a script, not a figure in a document: re-run it after any
recalibration and re-tape to what it prints.

    python3 training/shared_region.py
    python3 training/shared_region.py --overlay \\
        captures/<batch>/cam_front/zz_register.png \\
        captures/<batch>/cam_over/zz_register.png --out /tmp/retape.png

Needs no rig and no capture for the numbers -- only `config/extrinsics.yaml` and
the per-unit factory intrinsics in `calibration_backup/`. The overlay needs two
stills, one per camera, and reads their intrinsics at their own resolution: the
intrinsics are per-resolution and mixing them is an 18 mm trap
(`CAPTURE_PROTOCOL.md` 3).
"""
import argparse
import os
import sys

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# The pipeline streams this; the shared region is what the RUNNER can see, so it
# is computed at the streaming resolution rather than the 1080p of the stills.
STREAM_W, STREAM_H = 848, 480


def load_extrinsics(path):
    with open(path) as f:
        doc = yaml.safe_load(f)
    out = {}
    for cam, node in doc["cameras"].items():
        T = np.array(node["T_world_cam"], dtype=float).reshape(4, 4)
        out[cam] = (T, str(node["serial"]))
    return out


def factory_intrinsics(serial, width, height):
    """Per-unit factory colour intrinsics at one resolution, from the backup.

    Refuses a resolution the device does not report rather than rescaling: a
    rescaled K is the silent version of the same trap the calibration tool
    fails loudly on.
    """
    path = os.path.join(ROOT, "calibration_backup", f"{serial}_calibration.yaml")
    with open(path) as f:
        doc = yaml.safe_load(f)
    for p in doc["calibration_table"]["profiles"]:
        if p["stream"] == "Color" and p["width"] == width and p["height"] == height:
            return np.array([[p["fx"], 0, p["cx"]], [0, p["fy"], p["cy"]], [0, 0, 1]])
    raise SystemExit(
        f"{serial}: no factory Color profile at {width}x{height}. Intrinsics are "
        f"per-resolution and rescaling them is the trap this refuses to take."
    )


def visible(pts, T_world_cam, K, width, height):
    """Mask of world points landing inside this camera's frame."""
    p = np.concatenate([pts, np.ones(pts.shape[:-1] + (1,))], axis=-1)
    cam = p @ np.linalg.inv(T_world_cam).T
    z = cam[..., 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        uv = cam[..., :3] @ K.T
        u, v = uv[..., 0] / z, uv[..., 1] / z
    return (z > 0) & (u >= 0) & (u < width) & (v >= 0) & (v < height)


def shared_rect(ext, z, step=0.005, half=0.60):
    """Largest origin-centred rectangle on the plane at `z` seen by BOTH cameras.

    Maximises AREA. Two rectangles of near-equal area can differ noticeably in
    edge lengths, so compare areas across runs rather than widths.
    """
    n = int(half / step)
    ax = np.arange(-n, n + 1) * step
    gx, gy = np.meshgrid(ax, ax, indexing="ij")
    pts = np.stack([gx, gy, np.full_like(gx, z)], axis=-1)

    mask = np.ones(gx.shape, bool)
    for cam, (T, serial) in ext.items():
        K = factory_intrinsics(serial, STREAM_W, STREAM_H)
        mask &= visible(pts, T, K, STREAM_W, STREAM_H)

    integral = np.zeros((mask.shape[0] + 1, mask.shape[1] + 1), np.int64)
    integral[1:, 1:] = np.cumsum(np.cumsum(mask.astype(np.int64), 0), 1)
    best = (0, 0.0, 0.0)
    for a in range(n + 1):
        for b in range(n + 1):
            i0, i1, j0, j1 = n - a, n + a + 1, n - b, n + b + 1
            count = (integral[i1, j1] - integral[i0, j1]
                     - integral[i1, j0] + integral[i0, j0])
            if count == (i1 - i0) * (j1 - j0):
                area = (2 * a) * (2 * b)
                if area > best[0]:
                    best = (area, 2 * a * step, 2 * b * step)
    return best[1], best[2]


def overlay(ext, front_png, over_png, out_png, width_m, depth_m, board_m):
    import cv2

    panels = []
    for cam, src in (("cam_front", front_png), ("cam_over", over_png)):
        img = cv2.imread(src)
        if img is None:
            raise SystemExit(f"cannot read {src}")
        h, w = img.shape[:2]
        T, serial = ext[cam]
        K = factory_intrinsics(serial, w, h)   # the still's OWN resolution

        def draw(ew, ed, colour, label, thick):
            pts = np.array([[-ew / 2, -ed / 2, 0], [ew / 2, -ed / 2, 0],
                            [ew / 2, ed / 2, 0], [-ew / 2, ed / 2, 0]])
            p = np.c_[pts, np.ones(4)] @ np.linalg.inv(T).T
            if (p[:, 2] <= 0).any():
                return
            uv = (p[:, :3] @ K.T)
            uv = uv[:, :2] / uv[:, 2:3]
            cv2.polylines(img, [uv.astype(np.int32)], True, colour, thick, cv2.LINE_AA)
            cv2.putText(img, label, tuple(uv[0].astype(int) + np.array([6, -10])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, colour, 2, cv2.LINE_AA)

        draw(width_m, depth_m, (60, 255, 60),
             f"shared {width_m*1000:.0f} x {depth_m*1000:.0f}", 4)
        if board_m:
            draw(board_m, board_m, (255, 200, 0),
                 f"board {board_m*1000:.0f} x {board_m*1000:.0f}", 2)
        origin = np.r_[0.0, 0.0, 0.0, 1.0] @ np.linalg.inv(T).T
        o = (origin[:3] @ K.T)[:2] / (origin[:3] @ K.T)[2]
        cv2.drawMarker(img, tuple(o.astype(int)), (0, 0, 255), cv2.MARKER_CROSS, 40, 3)
        cv2.putText(img, cam, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.4,
                    (255, 255, 255), 3, cv2.LINE_AA)
        panels.append(cv2.resize(img, (960, 540)))
    cv2.imwrite(out_png, np.hstack(panels))
    print(f"\nwrote {out_png} -- green is the region to tape, red cross the world origin")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--extrinsics", default=os.path.join(ROOT, "config/extrinsics.yaml"))
    ap.add_argument("--heights", default="0,0.05,0.10,0.15",
                    help="metres above the tabletop, comma-separated")
    ap.add_argument("--board", type=float, default=8 * 0.038025,
                    help="board pattern edge in metres, used for the tape offsets")
    ap.add_argument("--overlay", nargs=2, metavar=("FRONT_PNG", "OVER_PNG"))
    ap.add_argument("--out", default="/tmp/shared_region.png")
    args = ap.parse_args()

    ext = load_extrinsics(args.extrinsics)
    if set(ext) != {"cam_front", "cam_over"}:
        raise SystemExit(f"expected cam_front and cam_over, got {sorted(ext)}")

    print(f"shared tabletop region from {os.path.relpath(args.extrinsics, ROOT)}, "
          f"at the streaming {STREAM_W}x{STREAM_H}\n")
    print(f"  {'height':>9}  {'largest origin-centred rectangle':>34}")
    base = None
    for z in [float(x) for x in args.heights.split(",")]:
        w, d = shared_rect(ext, z)
        if base is None:
            base = (w, d)
        print(f"  {z*1000:>6.0f} mm  {w*1000:>15.0f} x {d*1000:<16.0f} mm")

    w, d = base
    print(f"\ntape this, centred on the workspace origin: "
          f"{w*1000:.0f} x {d*1000:.0f} mm")
    print(f"  corners      x = +-{w/2*1000:.1f} mm, y = +-{d/2*1000:.1f} mm "
          f"(+x to your right, +y toward the back edge)")
    if args.board:
        print(f"  from the board's taped pattern corners ({args.board*1000:.1f} mm square):")
        print(f"    left / right : {(w-args.board)/2*1000:5.1f} mm beyond each pattern edge")
        print(f"    front / back : {(d-args.board)/2*1000:5.1f} mm beyond each pattern edge")
    print("\nThe tabletop plane is the WORST case on this rig -- both cameras look\n"
          "down, so the near edge of frame is what binds at z = 0 and height only\n"
          "relieves it. Tape the z = 0 rectangle and add no height margin.")

    if args.overlay:
        overlay(ext, args.overlay[0], args.overlay[1], args.out, w, d, args.board)


if __name__ == "__main__":
    sys.exit(main())
