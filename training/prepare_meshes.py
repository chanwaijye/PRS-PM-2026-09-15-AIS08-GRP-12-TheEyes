#!/usr/bin/env python3
"""Prepare assets/meshes/ from upstream sources. Offline tooling, run rarely.

Why this exists rather than "download the meshes and commit them":

A scanned mesh's origin is wherever the scanner left it. FoundationPose returns
the pose of the mesh origin, so ObjectState::pose_w.translation() is the mesh
origin too -- and two things downstream assume it is the object's CENTRE:

  1. scenegraph's AabbOf() builds the bounding box as translation() +/- extent.
     For the raw YCB bowl the origin is 43.7 mm off centre, so every on/in/clear
     test would be evaluated against a displaced box -- 27% of the object's own
     diameter.

  2. `symmetric: true` claims rotations about the symmetry axis are equivalent.
     Measured on the raw YCB bowl, the rim radius about the mesh-origin axis has
     sd 30.5 mm; about the AABB-centre axis, 2.5 mm. The origin is not on the
     axis, so rotating about it MOVES the origin and the "equivalent" poses
     differ in translation.

Both disappear if the mesh origin is the AABB centre. So that is the contract:

    the object frame's origin is the AABB centre of its mesh

which is well-defined, unlike "wherever the scanner left it". This script is
what enforces it. Re-centring is a rigid re-parameterisation: diameters,
extents, ADD/ADD-S and FoundationPose accuracy are all unaffected.

Usage:
    python3 training/prepare_meshes.py            # build assets/meshes/
    python3 training/prepare_meshes.py --verify   # check what is there, write nothing
"""

from __future__ import annotations

import argparse
import math
import shutil
import subprocess
import sys
import tarfile
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
OUT = REPO / "assets" / "meshes"

YCB_URL = "https://ycb-benchmarks.s3.amazonaws.com/data/google/{obj}_google_16k.tgz"

# The mustard bottle ships with FoundationPose's demo data, which is the copy
# the pose estimator was validated against -- its reported diameter (0.19646 m)
# matches this mesh. Prefer it, fall back to the YCB download.
FOUNDATIONPOSE_MUSTARD = Path.home() / "src/FoundationPose/demo_data/mustard0/mesh"

# Anything below this and "centred" is a rounding claim, not a fact.
CENTRED_TOL_M = 1e-6


def read_obj_vertices(path: Path) -> list[tuple[float, float, float]]:
    verts = []
    with path.open(errors="ignore") as handle:
        for line in handle:
            if line.startswith("v "):
                verts.append(tuple(float(x) for x in line.split()[1:4]))
    if not verts:
        raise ValueError(f"{path}: no vertices")
    return verts


def aabb(verts):
    lo = [min(v[i] for v in verts) for i in range(3)]
    hi = [max(v[i] for v in verts) for i in range(3)]
    return lo, hi



def exact_diameter(verts) -> float:
    """Largest pairwise vertex distance -- EXACT, not sampled.

    diameter_m sets the ADD-S acceptance threshold, so a sampled estimate is a
    made-up number that quietly loosens or tightens every pose result. Sampling
    3000 of the bowl's 8194 vertices moved the answer by 0.07 mm between two
    runs purely from the draw, which is exactly the kind of number CLAUDE.md
    forbids.

    The diameter is attained on the convex hull, so the hull reduces this from
    ~34M pairs to a few thousand. Falls back to a chunked full scan if scipy is
    unavailable -- still exact, just slower.
    """
    import numpy as np

    points = np.asarray(verts, dtype=np.float64)
    try:
        from scipy.spatial import ConvexHull

        points = points[ConvexHull(points).vertices]
    except Exception:  # scipy missing, or degenerate (coplanar) input
        pass

    best = 0.0
    for start in range(0, len(points), 512):
        chunk = points[start : start + 512]
        d2 = ((chunk[:, None, :] - points[None, :, :]) ** 2).sum(-1)
        best = max(best, float(d2.max()))
    return math.sqrt(best)


def measure(path: Path) -> dict:
    verts = read_obj_vertices(path)
    lo, hi = aabb(verts)
    centre = [(lo[i] + hi[i]) / 2 for i in range(3)]
    half = [(hi[i] - lo[i]) / 2 for i in range(3)]
    diameter = exact_diameter(verts)
    return {
        "vertices": len(verts),
        "centre_offset": centre,
        "extent": half,
        "diameter": diameter,
    }


def recentre_obj(src: Path, dst: Path) -> list[float]:
    """Rewrite src to dst with vertices shifted so the AABB centre is the origin.

    Only `v` lines are touched. Normals (vn) are direction vectors and texture
    coordinates (vt) are unrelated to position, so both pass through untouched --
    rewriting them would corrupt the mesh.
    """
    offset = measure(src)["centre_offset"]
    with src.open(errors="ignore") as fin, dst.open("w") as fout:
        fout.write(
            f"# Re-centred by training/prepare_meshes.py: the AABB centre is the\n"
            f"# origin, which is the object-frame contract (see that script).\n"
            f"# Applied translation: "
            f"[{-offset[0]:.6f}, {-offset[1]:.6f}, {-offset[2]:.6f}] m\n"
        )
        for line in fin:
            if line.startswith("v "):
                parts = line.split()
                xyz = [float(parts[i + 1]) - offset[i] for i in range(3)]
                fout.write(f"v {xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}\n")
            else:
                fout.write(line)
    return offset


def write_cube(dst: Path, half: float = 0.025) -> None:
    """A cube is authored, not scanned: exact by construction, centred already.

    The printed part is what must match this mesh, not the reverse.
    """
    verts = [(x * half, y * half, z * half) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
    # Outward-facing winding on all six quads.
    faces = [(1, 2, 4, 3), (5, 7, 8, 6), (1, 5, 6, 2), (3, 4, 8, 7), (1, 3, 7, 5), (2, 6, 8, 4)]
    with dst.open("w") as handle:
        handle.write(
            f"# {half * 2000:.0f} mm cube, centred on the origin. Authored, not scanned.\n"
        )
        for v in verts:
            handle.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        for f in faces:
            handle.write("f " + " ".join(str(i) for i in f) + "\n")


def fetch_ycb(obj: str, workdir: Path) -> Path:
    """Download and unpack one YCB google_16k object. Returns its mesh directory."""
    url = YCB_URL.format(obj=obj)
    archive = workdir / f"{obj}.tgz"
    print(f"  downloading {url}")
    subprocess.run(["curl", "-sSL", "--max-time", "300", "-o", str(archive), url], check=True)
    with tarfile.open(archive) as tar:
        tar.extractall(workdir)
    return workdir / obj / "google_16k"


def build() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        work = Path(tmp)

        if (FOUNDATIONPOSE_MUSTARD / "textured_simple.obj").exists():
            print("mustard_bottle: FoundationPose demo data")
            src_dir, src_obj, src_mtl = (
                FOUNDATIONPOSE_MUSTARD,
                "textured_simple.obj",
                "textured_simple.obj.mtl",
            )
        else:
            print("mustard_bottle: YCB 006 (FoundationPose demo data not present)")
            src_dir = fetch_ycb("006_mustard_bottle", work)
            src_obj, src_mtl = "textured.obj", "textured.mtl"
        offset = recentre_obj(src_dir / src_obj, OUT / "mustard_bottle.obj")
        shutil.copy(src_dir / src_mtl, OUT / "mustard_bottle.mtl")
        shutil.copy(src_dir / "texture_map.png", OUT / "mustard_bottle_texture.png")
        print(f"  re-centred by {[round(-o * 1000, 2) for o in offset]} mm")

        print("bowl: YCB 024")
        src_dir = fetch_ycb("024_bowl", work)
        offset = recentre_obj(src_dir / "textured.obj", OUT / "bowl.obj")
        shutil.copy(src_dir / "textured.mtl", OUT / "bowl.mtl")
        shutil.copy(src_dir / "texture_map.png", OUT / "bowl_texture.png")
        print(f"  re-centred by {[round(-o * 1000, 2) for o in offset]} mm")

        print("red_cube: authored")
        write_cube(OUT / "red_cube.obj")


def verify() -> int:
    failures = 0
    for name in ("mustard_bottle", "red_cube", "bowl"):
        path = OUT / f"{name}.obj"
        if not path.exists():
            print(f"{name}: MISSING at {path}")
            failures += 1
            continue
        m = measure(path)
        worst = max(abs(c) for c in m["centre_offset"])
        ok = worst <= CENTRED_TOL_M
        failures += 0 if ok else 1
        print(
            f"{name:16s} verts {m['vertices']:6d}  diameter {m['diameter']:.5f} m\n"
            f"{'':16s} extent  [{', '.join(f'{e:.4f}' for e in m['extent'])}]\n"
            f"{'':16s} centred {'yes' if ok else f'NO -- off by {worst * 1000:.2f} mm'}"
        )
    if failures:
        print(
            f"\n{failures} problem(s). A mesh that is not centred breaks scenegraph's\n"
            "AabbOf() and invalidates the symmetry claim in config/meshes.yaml.\n"
            "Re-run without --verify to rebuild."
        )
    else:
        print("\nAll meshes centred on their AABB. Copy the numbers into config/meshes.yaml.")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify", action="store_true", help="check only, write nothing")
    args = parser.parse_args()
    if not args.verify:
        build()
        print()
    return verify()


if __name__ == "__main__":
    sys.exit(main())
