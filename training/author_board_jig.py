#!/usr/bin/env python3
"""Author the ChArUco board seating jig (docs/CAPTURE_PROTOCOL.md 4.2c-ter).

WHY A JIG AND NOT TAPE, AND NOT A TABLE CORNER
----------------------------------------------
`calibrate reregister` is a ~30 s confirmation that the cameras have not moved
relative to the tabletop, and it works by re-detecting the board at its declared
spot. That only means anything if the board actually returns to that spot.

Tape marks the region; it does not position the board. A table corner would
position it, but the corner is unusable: projecting all 49 interior corners
through the committed extrinsics puts only 9-20 of them in BOTH frames against a
`min_shared_corners: 12` gate, versus 49 today, with the shared set collapsing
from a 228 x 228 mm spread to 114 x 190 mm (CAPTURE_PROTOCOL 4.2c-bis).

So: keep the board where it is and make that spot repeatable mechanically.

KINEMATICS -- WHY THREE CONTACTS AND NOT A POCKET
-------------------------------------------------
A flat board on a flat table has three in-plane degrees of freedom: x, y, yaw.
Three contact points fix exactly those three and no more:

    two pads on one edge   -> y and yaw
    one pad on the perpendicular edge -> x

A pocket, or a long wall on the x edge, adds a fourth constraint. An
over-constrained part either rattles or refuses to seat, and both failure modes
are silent here: the board still looks seated and the calibration still looks
fine. This is the same reasoning as the trays' stated socket clearance -- the fit
must be determined by design, not by how hard it was pushed.

Because the contacts are three points rather than a pocket, there is no
clearance to specify and no slop to bound: the seated pose is exact as long as
the board is pushed into all three. The seating gesture is therefore part of the
protocol, not an afterthought -- push toward the fence, then toward the stop.

The yaw baseline is what actually governs repeatability, which is why the second
pad sits ~260 mm from the first rather than both living on one corner bracket.

WHAT THE JIG DOES NOT DO
------------------------
It does not make the world origin correct -- see docs/WORLD_FRAME.md. It makes it
REPEATABLE, which is the only property `reregister` needs. If a bracket itself is
knocked the jig will happily seat the board in a new wrong place, so the brackets
are witness-marked and the re-registration check is still mandatory.

Usage:
    ~/venvs/the-eyes-r1/bin/python training/author_board_jig.py
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import trimesh

# --- Board, from config/calibration.yaml and docs/CALIBRATION_PROTOCOL.md -----
PANEL_MM = 324.0          # printed panel, outer edge -- what the jig touches
BOARD_THICK_MM = 7.1      # measured with callipers 2026-09-05

# --- Jig ---------------------------------------------------------------------
WALL_H_MM = 5.0           # below BOARD_THICK_MM so the board is always liftable
WALL_T_MM = 3.0
BASE_T_MM = 2.5
BASE_OUT_MM = 22.0        # how far the base extends AWAY from the board
FENCE_PAD_MM = 20.0       # contact length of each fence pad (short = a point)
STOP_PAD_MM = 15.0        # contact length of the single x stop
PAD_SPACING_MM = 260.0    # fence pad separation -- the yaw baseline
LEAD_IN_MM = 1.0          # chamfer on the top inner edge so the board drops in
WITNESS_MM = 1.2          # depth of the witness notch cut into the base outer edge


def _box(sx, sy, sz, at):
    m = trimesh.creation.box(extents=(sx, sy, sz))
    m.apply_translation(at)
    return m


# Each wall is described by the axis it runs ALONG, the coordinate of the face
# the board touches, and which way is OUTWARD (away from the board). The sign is
# explicit because the +y fence and the -x stop point opposite ways, and getting
# it wrong puts the base plate UNDER the board -- which lifts it off the table
# and tilts the whole world frame without looking wrong.
def _wall(length, along_x, face, sign, center):
    parts = []
    t, b = WALL_T_MM * sign, BASE_OUT_MM * sign
    if along_x:
        parts.append(_box(length, WALL_T_MM, WALL_H_MM,
                          (center, face + t / 2.0, BASE_T_MM + WALL_H_MM / 2.0)))
        parts.append(_box(length + 2 * WALL_T_MM, BASE_OUT_MM, BASE_T_MM,
                          (center, face + b / 2.0, BASE_T_MM / 2.0)))
    else:
        parts.append(_box(WALL_T_MM, length, WALL_H_MM,
                          (face + t / 2.0, center, BASE_T_MM + WALL_H_MM / 2.0)))
        parts.append(_box(BASE_OUT_MM, length + 2 * WALL_T_MM, BASE_T_MM,
                          (face + b / 2.0, center, BASE_T_MM / 2.0)))
    return parts


def _bbox(x0, x1, y0, y1, z0, z1):
    return _box(x1 - x0, y1 - y0, z1 - z0, ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def _gusset(half, fx, sy):
    """Ties the fence pad and the stop pad into ONE rigid part.

    It has to WRAP the board corner rather than cut across it: a straight brace
    between the two pads would pass through the board footprint and lift the
    board off the table. So the gusset is itself an L, living entirely in
    y > +half and x < -half. `check()` enforces that rather than trusting it.
    """
    o = BASE_OUT_MM
    return [
        _bbox(-half - o, fx + FENCE_PAD_MM / 2 + WALL_T_MM, +half, +half + o, 0.0, BASE_T_MM),
        _bbox(-half - o, -half, sy - STOP_PAD_MM / 2 - WALL_T_MM, +half + o, 0.0, BASE_T_MM),
    ]


def _lead_in(length, along_x, face, sign, center):
    """Chamfer along the top inner edge so the board drops in rather than being
    forced past a square corner."""
    z = BASE_T_MM + WALL_H_MM
    e = (length + 4 * WALL_T_MM, LEAD_IN_MM * 3, LEAD_IN_MM * 3) if along_x \
        else (LEAD_IN_MM * 3, length + 4 * WALL_T_MM, LEAD_IN_MM * 3)
    c = trimesh.creation.box(extents=e)
    c.apply_transform(trimesh.transformations.rotation_matrix(
        np.pi / 4, [1, 0, 0] if along_x else [0, 1, 0]))
    c.apply_translation((center, face, z) if along_x else (face, center, z))
    return c


def _witness(along_x, face, sign, center):
    """V-notch in the base's outer edge. Tape-mark the table under it and any
    later displacement of the BRACKET is visible instead of silent."""
    out = face + BASE_OUT_MM * sign
    e = (WITNESS_MM * 2, BASE_OUT_MM, WITNESS_MM * 2) if along_x \
        else (BASE_OUT_MM, WITNESS_MM * 2, WITNESS_MM * 2)
    n = trimesh.creation.box(extents=e)
    n.apply_transform(trimesh.transformations.rotation_matrix(
        np.pi / 4, [0, 1, 0] if along_x else [1, 0, 0]))
    n.apply_translation((center, out, 0.0) if along_x else (out, center, 0.0))
    return n


def corner_bracket(half):
    """+y fence pad and the single -x stop, tied together as one L-shaped part."""
    fx = -half + 10.0 + FENCE_PAD_MM / 2.0          # fence pad centre, along x
    sy = +half - 10.0 - STOP_PAD_MM / 2.0           # stop pad centre, along y
    parts = _wall(FENCE_PAD_MM, True, +half, +1, fx)
    parts += _wall(STOP_PAD_MM, False, -half, -1, sy)
    parts += _gusset(half, fx, sy)
    body = trimesh.boolean.union(parts, engine="manifold")
    cut = [_lead_in(FENCE_PAD_MM, True, +half, +1, fx),
           _lead_in(STOP_PAD_MM, False, -half, -1, sy),
           _witness(True, +half, +1, fx)]
    return trimesh.boolean.difference([body] + cut, engine="manifold")


def fence_stop(half):
    """The second +y fence pad, PAD_SPACING_MM along +x from the first. This is
    what gives yaw a long baseline instead of a 20 mm one."""
    cx = -half + 10.0 + FENCE_PAD_MM / 2.0 + PAD_SPACING_MM
    body = trimesh.boolean.union(_wall(FENCE_PAD_MM, True, +half, +1, cx), engine="manifold")
    cut = [_lead_in(FENCE_PAD_MM, True, +half, +1, cx), _witness(True, +half, +1, cx)]
    return trimesh.boolean.difference([body] + cut, engine="manifold")


def check(name, mesh, half):
    """Refuse to emit a part that would sit under the board or come apart.

    Both failures are silent in the worst way: material under the board tilts the
    entire world frame while everything still looks seated, and a two-shell
    'bracket' holds its two contact faces in no fixed relation at all.

    The intrusion test is a real boolean intersection against the board's own
    volume, not a bounding-box test -- these parts are L-shaped and wrap the
    board corner, so their bounding boxes overlap the footprint by design while
    their material must not.
    """
    bad = []
    if not mesh.is_watertight:
        bad.append("not watertight")
    if mesh.body_count != 1:
        bad.append(f"{mesh.body_count} disconnected shells -- not one rigid part")

    # The board, inset by a hair so the exact-contact faces do not register.
    eps = 0.01
    board = _bbox(-half + eps, half - eps, -half + eps, half - eps, 0.0, BOARD_THICK_MM)
    hit = trimesh.boolean.intersection([mesh, board], engine="manifold")
    v = float(hit.volume) if hit is not None and len(hit.faces) else 0.0
    if v > 1.0:                                    # mm^3
        lo, hi = hit.bounds
        bad.append(f"intrudes into the board volume: {v:.1f} mm^3 at "
                   f"x[{lo[0]:.1f},{hi[0]:.1f}] y[{lo[1]:.1f},{hi[1]:.1f}]")
    if bad:
        raise SystemExit(f"{name}: " + "; ".join(bad))
    return v


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=Path("assets/jig"))
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)

    half = PANEL_MM / 2.0
    pieces = {"board_jig_corner": corner_bracket(half), "board_jig_fence_stop": fence_stop(half)}

    meta = {
        "panel_mm": PANEL_MM, "board_thickness_mm": BOARD_THICK_MM,
        "wall_height_mm": WALL_H_MM, "wall_thickness_mm": WALL_T_MM,
        "base_thickness_mm": BASE_T_MM, "fence_pad_mm": FENCE_PAD_MM,
        "stop_pad_mm": STOP_PAD_MM, "pad_spacing_mm": PAD_SPACING_MM,
        "contacts": {"plus_y_fence": 2, "minus_x_stop": 1},
        "seating_gesture": "lower the board flat, push it toward +y until both "
                           "fence pads touch, then toward -x until the stop touches",
        "parts": {},
    }
    for name, mesh in pieces.items():
        p = a.out / f"{name}.stl"
        intr = check(name, mesh, half)
        mesh.export(p)
        e = mesh.extents
        meta["parts"][name] = {"file": str(p), "watertight": bool(mesh.is_watertight),
                               "bbox_mm": [round(float(v), 2) for v in e],
                               "volume_cm3": round(float(mesh.volume) / 1000.0, 2),
                               "intrusion_into_board_mm3": round(intr, 3)}
        print(f"{name:22s} {e[0]:6.1f} x {e[1]:5.1f} x {e[2]:5.1f} mm   "
              f"{mesh.volume/1000.0:5.2f} cm3   watertight={mesh.is_watertight}")

    (a.out / "board_jig.json").write_text(json.dumps(meta, indent=2))
    print(f"\nwrote {a.out}/board_jig.json")
    print(f"yaw baseline: {PAD_SPACING_MM:.0f} mm between the two fence pads")
    print(f"wall {WALL_H_MM} mm < board {BOARD_THICK_MM} mm, so the board always lifts out")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
