#!/usr/bin/env python3
"""Emit millimetre-unit STLs for everything that goes on a print plate.

WHY THIS EXISTS -- THE UNITS ARE A SILENT TRAP
----------------------------------------------
`assets/meshes/*.obj` and `assets/trays/*.stl` are authored in METRES, because
every downstream consumer -- FoundationPose, `config/meshes.yaml`,
`config/marker_trays.yaml`, the fusion frames -- is metric. STL carries no unit,
and Bambu Studio / Orca read one STL unit as one millimetre. So `bowl_tray.stl`
imports as a 0.21 x 0.21 x 0.0085 mm speck and the slicer offers to "scale to
fit", which is precisely what `docs/RIG_BOM.md` forbids:

    "Turn off any scale-to-fit. 6.2 requires 100% with no bed-fitting rescale.
     Nothing here is near the volume limit, so a rescale would be silent and
     gratuitous."

A dialog that guesses 1000x is not the same thing as a file that says 1000x.
This script makes the scale explicit and checks it, so no print decision is ever
taken inside a slicer prompt.

WHY THE PRINTED GEOMETRY IS STILL THE EVALUATED GEOMETRY
--------------------------------------------------------
ADD-S is computed over the vertices of `assets/meshes/*.obj`, so a print file
that differs from that mesh is a silent pose error with nothing downstream able
to see it. Two cleanups are applied here and NEITHER MOVES A VERTEX:

  merge_vertices(merge_tex, merge_norm)   topological only. Both YCB OBJs split
      vertices at texture seams; merging rejoins them, which is what makes the
      bottle report watertight. Same call as `author_marker_trays.load_mesh`.
  largest connected component             the bowl scan carries 6 stray two-face
      fragments (largest 1.49 mm, the rest below 0.5 mm) that would otherwise
      land on the plate as debris. RIG_BOM says delete them in the slicer; doing
      it here means it cannot be forgotten.

Scaling is an exact power of ten into float32, so the printed part is the mesh.
Nothing is thickened, decimated or repaired -- in particular the bowl's 0.99 mm
scanned wall is emitted as authored, and must be printed with thin-wall
detection ON rather than made printable by editing it.

    python training/export_print_stls.py            # write assets/print/*.stl
    python training/export_print_stls.py --verify   # re-check, write nothing
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import trimesh

REPO = Path(__file__).resolve().parent.parent
OUT = REPO / "assets" / "print"

# source, unit scale to millimetres, and the reference extents in mm that the
# emitted file must reproduce. Objects: config/meshes.yaml extent * 2.
# Trays: config/marker_trays.yaml plate_m * 1000.
PARTS = [
    ("red_cube",            "meshes/red_cube.obj",             1000.0, ( 50.000,  50.000,  50.000)),
    ("mustard_bottle",      "meshes/mustard_bottle.obj",       1000.0, ( 97.152,  66.618, 191.408)),
    ("bowl",                "meshes/bowl.obj",                 1000.0, (161.464, 161.164,  55.010)),
    ("red_cube_tray",       "trays/red_cube_tray.stl",         1000.0, (170.000,  90.000,  10.500)),
    ("mustard_bottle_tray", "trays/mustard_bottle_tray.stl",   1000.0, (216.000, 100.000,  14.500)),
    ("bowl_tray",           "trays/bowl_tray.stl",             1000.0, (210.000, 210.000,   8.500)),
]

BED_MM = 256.0          # Bambu Lab A1
TOL_MM = 0.01           # emitted extents vs. the config's own numbers


DEBRIS_MM = 5.0         # below this a component is scan noise, not a part


def load_mm(rel: str, scale: float) -> tuple[trimesh.Trimesh, int, int]:
    """Load a source part in millimetres.

    Returns (mesh, debris dropped, substantive bodies). Debris is dropped;
    a second SUBSTANTIVE body is never dropped, because that is not noise --
    it is a loose island that the slicer will happily print somewhere it does
    not belong. The caller fails on it.
    """
    m = trimesh.load(REPO / "assets" / rel, force="mesh", process=True)
    m.merge_vertices(merge_tex=True, merge_norm=True)
    m.apply_scale(scale)
    parts = m.split(only_watertight=False)
    if len(parts) <= 1:
        return m, 0, 1
    keep = [c for c in parts if c.extents.max() >= DEBRIS_MM]
    dropped = len(parts) - len(keep)
    if len(keep) == 1:
        return keep[0], dropped, 1
    return m, dropped, len(keep)


def same_geometry(a: trimesh.Trimesh, b: trimesh.Trimesh) -> bool:
    """Same part to within float32 export noise: faces, volume, area and bounds."""
    return (len(a.faces) == len(b.faces)
            and np.isclose(a.volume, b.volume, rtol=1e-6)
            and np.isclose(a.area, b.area, rtol=1e-6)
            and np.allclose(a.bounds, b.bounds, atol=1e-3))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--verify", action="store_true",
                    help="re-check the emitted files and write nothing")
    args = ap.parse_args()

    OUT.mkdir(parents=True, exist_ok=True)
    print(f"{'part':<22}{'extents mm':>26}{'faces':>8}{'drop':>6}"
          f"{'watertight':>12}{'bed':>6}{'layers':>8}  file")
    print("-" * 104)

    ok = True
    for name, rel, scale, ref in PARTS:
        m, dropped, bodies = load_mm(rel, scale)
        ext = m.extents
        good = np.allclose(ext, ref, atol=TOL_MM)
        fits = ext[0] <= BED_MM and ext[1] <= BED_MM
        layers = int(np.ceil(ext[2] / 0.2))          # 0.20 mm standard profile
        dest = OUT / f"{name}.stl"

        status = ""
        if not good:
            ok = False
            status = f"  <-- MISMATCH, expected {ref}"
        if not fits:
            ok = False
            status += "  <-- EXCEEDS BED"
        if bodies != 1:
            ok = False
            status += f"  <-- {bodies} LOOSE BODIES, not one solid"

        if bodies != 1:
            print(f"{name:<22}{ext[0]:8.3f}{ext[1]:9.3f}{ext[2]:9.3f}"
                  f"{len(m.faces):8d}{dropped:6d}{str(m.is_watertight):>12}"
                  f"{'yes' if fits else 'NO':>6}{layers:8d}  -- not written{status}")
            continue

        if args.verify:
            if not dest.exists():
                ok = False
                status += "  <-- MISSING"
            else:
                on_disk = trimesh.load(dest)
                if not same_geometry(on_disk, m):
                    ok = False
                    status += "  <-- ON-DISK FILE IS STALE"
        else:
            # Leave a file alone only when its GEOMETRY already agrees -- not just
            # its outline. An extents-only test once reported the regenerated
            # bottle and bowl trays as unchanged: their plates are the same size,
            # only the socket floors had moved, so the stale files survived.
            if dest.exists() and same_geometry(trimesh.load(dest), m):
                status += "  (unchanged)"
            else:
                m.export(dest)

        print(f"{name:<22}{ext[0]:8.3f}{ext[1]:9.3f}{ext[2]:9.3f}"
              f"{len(m.faces):8d}{dropped:6d}{str(m.is_watertight):>12}"
              f"{'yes' if fits else 'NO':>6}{layers:8d}  {dest.relative_to(REPO)}{status}")

    print()
    print("All files are MILLIMETRES. Import at 100%; decline any scale-to-fit prompt.")
    if not ok:
        print("FAILED", file=sys.stderr)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
