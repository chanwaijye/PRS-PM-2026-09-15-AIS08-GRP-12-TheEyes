#!/usr/bin/env python3
"""Author the ArUco ground-truth marker trays (proposal §6.3).

WHY A TRAY AND NOT A SEAT IN THE OBJECT
---------------------------------------
§6.3 originally specified the marker seat modelled *into* the printed part. That
is not realisable for two of the three classes, and the measurements say so:

  mustard_bottle  no flat region anywhere. The front face carries 7.5 mm of
                  sagitta over a 24 mm patch, and the only truly flat area is the
                  cap top at 17.7 mm diameter -- below the decode floor at 0.7 m.
  bowl            scanned wall is 0.99 mm. A flush recess needs its own depth
                  plus surviving wall; there is none.
  red_cube        fine -- six flat 50 mm faces.

The decisive constraint is not printability though, it is the dataset.
`synth50k_v2` was rendered from the UNMODIFIED meshes, so cutting a facet or
adding a boss to a target object introduces a sim-to-real gap the trained
detector has never seen, and closing it means regenerating 88 GB.

So the marker moves off the object and onto a tray the object sits in. The socket
is boolean-subtracted from the object's own mesh, so the fit is exact by
construction in exactly the sense §6.3 asks for -- it is derived from the
geometry rather than measured after the fact. What is gained:

  * `assets/meshes/*.obj` are untouched, so FoundationPose still registers
    against the mesh it was validated on and ADD-S is computed over that mesh.
  * `config/meshes.yaml` is untouched -- diameter_m, extent and the `symmetric`
    flags all remain true statements about the objects.
  * the rendered dataset still describes the physical objects.

What is paid: the tray occludes the object's base, and it is one extra printed
part per class. The tray is not a detected class, so it plays the role of a
distractor -- print it in one of the three distractor neutrals the renderer
already uses, not in a target colour.

THE TRANSFORM CHAIN, WHICH IS THE WHOLE POINT
---------------------------------------------
    T_cam_object = T_cam_marker . T_marker_tray . T_tray_object

`T_cam_marker` comes from ArUco at evaluation time. The other two are chosen
here, in CAD, and written to `config/marker_trays.yaml` -- so they are exact by
construction and never measured. Ground-truth accuracy is then bounded by the
socket clearance and the printer's dimensional accuracy, both reported below.

Frames follow the project's conventions: the object frame is the mesh AABB
centre (`config/meshes.yaml`), and the marker frame is OpenCV's -- origin at the
marker centre, +x right along the marker, +y up, +z out of the face.

    python training/author_marker_trays.py            # write STL, PNG, YAML
    python training/author_marker_trays.py --verify   # re-check, write nothing
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import trimesh

# --- Bambu Lab A1 -----------------------------------------------------------
BED_MM = 256.0

# --- filament accounting ----------------------------------------------------
# The same shell+infill model used for the objects in docs/RIG_BOM.md and
# proposal §9. Reporting solid volume here instead would overstate a flat plate
# by roughly 2x and would not be comparable to any other filament figure in the
# project.
PLA_G_PER_CM3 = 1.24
WALL_MM = 1.2
INFILL = 0.15


def print_mass_g(mesh: trimesh.Trimesh) -> float:
    v_cm3 = mesh.volume * 1e6
    a_cm2 = mesh.area * 1e4
    shell = min(v_cm3, a_cm2 * WALL_MM / 10.0)
    return (shell + INFILL * max(0.0, v_cm3 - shell)) * PLA_G_PER_CM3


def socket_clearance_mm(tray: trimesh.Trimesh, obj_seated: trimesh.Trimesh,
                        depth_m: float) -> tuple[float, float]:
    """Measured gap between the seated object and the socket wall.

    The cutter is inflated by a per-axis SCALE, not a true surface offset, so the
    realised clearance is only approximately SOCKET_CLEARANCE_MM and varies over
    the profile. It bounds the ground-truth translation error, so it is measured
    rather than assumed. Sampled on a ring of points around the object at
    mid-socket height.
    """
    z = -depth_m / 2.0
    sec = obj_seated.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    if sec is None:
        return float("nan"), float("nan")
    pts = np.asarray(sec.vertices)[:, :2]
    tsec = tray.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    if tsec is None:
        return float("nan"), float("nan")
    # Distance to the wall as a CURVE, not to its vertices. A box section has
    # only four vertices, so a vertex-only metric reports the distance to a
    # corner for every point along an edge -- which read as 4.20 mm of clearance
    # on a socket that is actually uniform.
    tv = np.asarray(tsec.vertices)[:, :2]
    seg = np.asarray(tsec.vertex_nodes) if hasattr(tsec, "vertex_nodes") else None
    if seg is None or len(seg) == 0:
        seg = np.array([[i, (i + 1) % len(tv)] for i in range(len(tv))])
    a, b = tv[seg[:, 0]], tv[seg[:, 1]]
    ab = b - a
    L2 = np.einsum("ij,ij->i", ab, ab)
    L2[L2 == 0] = 1e-18
    t = np.clip(np.einsum("pij,ij->pi", pts[:, None, :] - a[None], ab) / L2, 0.0, 1.0)
    proj = a[None] + t[..., None] * ab[None]
    d = np.linalg.norm(pts[:, None, :] - proj, axis=2).min(axis=1)
    return float(d.min() * 1000), float(d.max() * 1000)

# --- markers ----------------------------------------------------------------
# DICT_5X5_250 matches the ChArUco board, so one dictionary covers the whole
# project. The board consumes ids 0-31, so trays start at 100 -- far enough clear
# that a collision is obvious rather than subtle if the board is ever enlarged.
ARUCO_DICT = "DICT_5X5_250"
# Ids are allocated GLOBALLY, not per tray. Every tray starting at 100 would make
# marker 100 ambiguous the moment two objects share a scene -- which is every
# scene in the occlusion ablation.
MARKER_ID_BASE = 100
MARKER_MM = 40.0          # black area. 1.15 m reach at 848x480 against a 0.884 m
                          # worst-case corner: 1.3x margin while STREAMING, so
                          # ground truth does not require a separate 1080p shot.
QUIET_MM = 6.0            # >= 1 module (40/7 = 5.71 mm) of white, as ArUco needs
CARD_MM = MARKER_MM + 2 * QUIET_MM        # 52 mm printed card
RECESS_FIT_MM = 0.4       # card-to-recess slop, total across the pair
RECESS_DEPTH_MM = 0.4     # ~paper thickness, so the card sits flush

# --- socket -----------------------------------------------------------------
# Lateral clearance is the dominant term in ground-truth uncertainty: the object
# can sit anywhere within it. 0.20 mm is a printable slip fit on an A1 and is
# 1.0% of the bottle's 19.6 mm ADD-S threshold, so it does not move any verdict.
SOCKET_CLEARANCE_MM = 0.20
FLOOR_MM = 2.5            # plate material left under the socket

TRAYS = {
    # class          socket depth   plate outline (x, y)     marker centres (x, y)
    # Marker offsets must clear the SOCKET OPENING, not just the object: a card
    # overhanging the hole has nothing under it. check_card_clearance() asserts
    # this, because the first cut of these numbers silently merged the pockets
    # into the socket.
    "red_cube":      dict(depth=8.0,  plate=(170.0, 90.0), lighten=0.0,
                          markers=[(-57.0, 0.0), (57.0, 0.0)]),
    "mustard_bottle": dict(depth=12.0, plate=(216.0, 100.0), lighten=0.0,
                          markers=[(-80.0, 0.0), (80.0, 0.0)]),
    # The bowl is 161 mm across, so wings would overrun the 256 mm bed. A square
    # plate instead puts the markers in the CORNERS, which a round object leaves
    # free -- 200/2*sqrt(2) - 80.75 = 60 mm of diagonal room for a 52 mm card.
    # The bowl socket only has to locate laterally -- the class is rotationally
    # symmetric, so yaw is undefined for ADD-S and the socket need not key it.
    # 6 mm is enough for that, and `lighten` opens the floor the bowl does not
    # rest on. The bore MUST stay inside the bowl's base footprint: the bowl is
    # only 91.7 mm across at its base and 102.6 mm at the 6 mm socket rim, so a
    # 110 mm bore removed the whole socket wall and the bowl fell through. 60 mm
    # leaves a ~16 mm rest ring under the foot.
    "bowl":          dict(depth=6.0, plate=(210.0, 210.0), lighten=60.0,
                          markers=[(-72.0, -72.0), (72.0, -72.0),
                                   (-72.0, 72.0), (72.0, 72.0)]),
}

REPO = Path(__file__).resolve().parent.parent
MESHES = REPO / "assets" / "meshes"
OUT_STL = REPO / "assets" / "trays"
OUT_CFG = REPO / "config" / "marker_trays.yaml"


def load_mesh(path: Path) -> trimesh.Trimesh:
    """Load an OBJ with UV-seam vertices merged.

    Both YCB OBJs split vertices at texture seams, so a plain load reports them
    as non-watertight and every volume/boolean operation downstream misbehaves.
    Merging texture- and normal-split vertices makes the bottle watertight and
    leaves the bowl with 0 boundary edges.
    """
    m = trimesh.load(path, force="mesh", process=True)
    m.merge_vertices(merge_tex=True, merge_norm=True)
    # Drop the stray degenerate fragments the bowl scan carries -- 6 two-face
    # components that would otherwise punch specks out of the socket.
    parts = m.split(only_watertight=False)
    if len(parts) > 1:
        m = max(parts, key=lambda c: len(c.faces))
    return m


def inflate_lateral(mesh: trimesh.Trimesh, clearance_mm: float) -> trimesh.Trimesh:
    """Grow the cutter in x and y only, leaving z untouched.

    z must not scale: the object seats by resting its own base on the socket
    floor, so any vertical scaling would change the seat height and with it
    T_tray_object -- the one number this file exists to make exact.
    """
    out = mesh.copy()
    ext = mesh.bounding_box.extents
    c = clearance_mm / 1000.0
    s = np.array([(ext[0] + 2 * c) / ext[0], (ext[1] + 2 * c) / ext[1], 1.0])
    centre = mesh.bounding_box.centroid
    out.apply_translation(-centre)
    out.apply_scale(s)
    out.apply_translation(centre)
    return out


# Rig geometry from docs/RIG_BOM.md: standoff 0.70 m, elevation 35 deg,
# azimuth +/-40 deg from the front, measured at the workspace centre.
STANDOFF_M, ELEV_DEG, AZ_DEG = 0.70, 35.0, 40.0


def camera_positions() -> dict[str, np.ndarray]:
    e = math.radians(ELEV_DEG)
    r, h = STANDOFF_M * math.cos(e), STANDOFF_M * math.sin(e)
    out = {}
    for name, az in (("cam_left", -AZ_DEG), ("cam_right", +AZ_DEG)):
        a = math.radians(az)
        out[name] = np.array([r * math.sin(a), -r * math.cos(a), h])
    return out


def marker_visibility(obj_seated: trimesh.Trimesh, spec: dict) -> dict[str, int]:
    """How many markers each camera can see, unoccluded by the object itself.

    The bowl matters here: seated, its rim reaches 49 mm above the tray and
    overhangs to radius 80.7 mm, while a corner marker's inner edge sits at
    radius 65 mm. Whether that actually blocks anything depends on the 35 deg
    view angle, so it is ray-cast rather than argued.
    """
    half = (MARKER_MM / 2.0) / 1000.0
    seen = {}
    for cam, eye in camera_positions().items():
        n = 0
        for mx, my in spec["markers"]:
            c = np.array([mx / 1000.0, my / 1000.0, 0.0])
            corners = [c + np.array([sx * half, sy * half, 0.0])
                       for sx in (-1, 1) for sy in (-1, 1)] + [c]
            blocked = False
            for pt in corners:
                d = pt - eye
                L = np.linalg.norm(d)
                hits = obj_seated.ray.intersects_location([eye], [d / L])[0]
                if len(hits) and np.linalg.norm(hits - eye, axis=1).min() < L - 1e-4:
                    blocked = True
                    break
            n += 0 if blocked else 1
        seen[cam] = n
    return seen


def assign_marker_ids() -> dict[str, list[int]]:
    """Globally unique ids, allocated in TRAYS order and stable across runs."""
    out, nid = {}, MARKER_ID_BASE
    for cls, spec in TRAYS.items():
        out[cls] = list(range(nid, nid + len(spec["markers"])))
        nid += len(spec["markers"])
    return out


MARKER_IDS = assign_marker_ids()


def check_card_clearance(seated: trimesh.Trimesh, spec: dict) -> tuple[float, bool]:
    """Smallest gap between a marker card's footprint and the socket opening.

    The socket opening is the object's own cross-section at the tray's top face
    plus the lateral clearance. A card that reaches inside it is unsupported --
    the pocket merges with the socket and the marker sits over a hole.
    """
    sec = seated.section(plane_origin=[0, 0, 0], plane_normal=[0, 0, 1])
    if sec is None:                       # object does not reach the top face
        return float("inf"), True
    prof = np.asarray(sec.vertices)[:, :2] * 1000.0
    half = (CARD_MM + RECESS_FIT_MM) / 2.0
    worst = float("inf")
    for mx, my in spec["markers"]:
        lo = np.array([mx - half, my - half])
        hi = np.array([mx + half, my + half])
        # signed distance from each profile point to the card rectangle
        d = np.maximum(lo - prof, prof - hi).max(axis=1)
        worst = min(worst, float(d.min()))
    return worst, worst > 0.0


def build_tray(cls: str, spec: dict) -> tuple[trimesh.Trimesh, dict]:
    """Return the tray mesh and the exact transforms it realises.

    Tray frame: origin at the centre of the tray's TOP surface, +z up. Chosen
    because it is the surface the markers lie in, which makes T_tray_marker a
    pure in-plane translation with no rotation to get wrong.
    """
    obj = load_mesh(MESHES / f"{cls}.obj")
    depth = spec["depth"] / 1000.0
    plate_x, plate_y = (v / 1000.0 for v in spec["plate"])
    thickness = depth + FLOOR_MM / 1000.0

    plate = trimesh.creation.box((plate_x, plate_y, thickness))
    plate.apply_translation((0.0, 0.0, -thickness / 2.0))  # top face at z = 0

    # Seat the object so its lowest point rests at z = -depth. The mesh origin is
    # the AABB centre (config/meshes.yaml contract), so the offset from the tray
    # frame to the object frame is exactly this and nothing else.
    obj_zmin = obj.bounds[0][2]
    dz = -depth - obj_zmin
    seated = obj.copy()
    seated.apply_translation((0.0, 0.0, dz))
    t_tray_object = np.array([0.0, 0.0, dz])

    # Cut the socket. The cutter is extended UPWARD so the cavity breaks through
    # the top face; it is never extended downward, which is what leaves FLOOR_MM
    # of plate under the object for it to rest on.
    cutter = inflate_lateral(seated, SOCKET_CLEARANCE_MM)
    cutter = trimesh.boolean.union([cutter, _cap_above(cutter)], engine="manifold")
    tray = trimesh.boolean.difference([plate, cutter], engine="manifold")

    # Lightening bore through the floor, inside the object's contact footprint.
    if spec.get("lighten", 0.0) > 0.0:
        bore = trimesh.creation.cylinder(radius=spec["lighten"] / 2000.0,
                                         height=thickness * 4)
        tray = trimesh.boolean.difference([tray, bore], engine="manifold")

    # Marker recesses in the top face.
    recess = (CARD_MM + RECESS_FIT_MM) / 1000.0
    d = RECESS_DEPTH_MM / 1000.0
    markers = []
    for i, (mx, my) in enumerate(spec["markers"]):
        pocket = trimesh.creation.box((recess, recess, d * 2))
        pocket.apply_translation((mx / 1000.0, my / 1000.0, 0.0))  # half above z=0
        tray = trimesh.boolean.difference([tray, pocket], engine="manifold")
        markers.append(dict(id=MARKER_IDS[cls][i],
                            t_tray_marker=[mx / 1000.0, my / 1000.0, -d]))
    return tray, dict(t_tray_object=t_tray_object.tolist(), markers=markers,
                      socket_depth_m=depth, plate_m=[plate_x, plate_y, thickness],
                      obj=obj)


def _cap_above(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """A prism over the cutter's footprint covering everything above z = 0.

    Without it the socket is a closed void inside the plate rather than an open
    cavity: the object's widest point is below the rim on a tapered body, so the
    cut would not reach the top face and the object could not be inserted.
    """
    ext = mesh.bounding_box.extents
    c = mesh.bounding_box.centroid
    h = 0.30
    cap = trimesh.creation.box((ext[0], ext[1], h))
    cap.apply_translation((c[0], c[1], h / 2.0))
    return cap


def marker_sheet(cls: str, spec: dict, out: Path, dpi: int = 600) -> Path:
    """One PNG per tray, at true physical scale. Print at 100%, no fit-to-page."""
    import cv2
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, ARUCO_DICT))
    px_card = int(round(CARD_MM / 25.4 * dpi))
    px_mark = int(round(MARKER_MM / 25.4 * dpi))
    n = len(spec["markers"])
    sheet = np.full((px_card, px_card * n + 20 * (n - 1)), 255, np.uint8)
    for i in range(n):
        img = cv2.aruco.generateImageMarker(d, MARKER_IDS[cls][i], px_mark)
        x0 = i * (px_card + 20) + (px_card - px_mark) // 2
        y0 = (px_card - px_mark) // 2
        sheet[y0:y0 + px_mark, x0:x0 + px_mark] = img
        cv2.rectangle(sheet, (i * (px_card + 20), 0),
                      (i * (px_card + 20) + px_card - 1, px_card - 1), 200, 2)
        cv2.putText(sheet, f"{cls} id={MARKER_IDS[cls][i]}",
                    (i * (px_card + 20) + 8, px_card - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, dpi / 900.0, 120, 2)
    cv2.imwrite(str(out), sheet)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--verify", action="store_true", help="check only, write nothing")
    args = ap.parse_args()

    OUT_STL.mkdir(parents=True, exist_ok=True)
    report, ok = {}, True
    print(f"{'class':<16}{'plate mm':<21}{'socket':>7}{'print g':>9}"
          f"{'clearance mm':>16}{'watertight':>12}{'bed':>6}{'vis L/R':>9}{'card gap':>9}")
    print("-" * 92)

    cfg_lines = [
        "# ArUco ground-truth marker trays — GENERATED by training/author_marker_trays.py.",
        "# Do not hand-edit; re-run the generator. Proposal §6.3.",
        "#",
        "# T_cam_object = T_cam_marker . T_marker_tray . T_tray_object",
        "#",
        "# T_cam_marker comes from ArUco at evaluation time. Everything below is",
        "# chosen in CAD and is therefore exact by construction, never measured.",
        "# Object frame = mesh AABB centre (config/meshes.yaml). Marker frame =",
        "# OpenCV: origin at the marker centre, +x right, +y up, +z out of the face.",
        "# Translations are metres in the TRAY frame, whose origin is the centre of",
        "# the tray's top surface with +z up. No rotations: every marker lies in the",
        "# tray's top plane and the object is seated upright, so all three frames",
        "# share one orientation and the chain is pure translation.",
        f"dictionary: \"{ARUCO_DICT}\"",
        f"marker_length_m: {MARKER_MM / 1000.0:.4f}",
        f"socket_clearance_m: {SOCKET_CLEARANCE_MM / 1000.0:.5f}",
        "trays:",
    ]

    for cls, spec in TRAYS.items():
        tray, info = build_tray(cls, spec)
        mass = print_mass_g(tray)
        seated = info["obj"].copy()
        seated.apply_translation(info["t_tray_object"])
        cl_lo, cl_hi = socket_clearance_mm(tray, seated, info["socket_depth_m"])
        vis = marker_visibility(seated, spec)
        gap, gap_ok = check_card_clearance(seated, spec)
        px, py, pz = info["plate_m"]
        fits = max(px, py) * 1000 <= BED_MM
        wt = tray.is_watertight
        ok &= bool(wt and fits)
        print(f"{cls:<16}{px*1000:6.1f} x{py*1000:6.1f} x{pz*1000:5.1f}   "
              f"{spec['depth']:>5.1f}{mass:>9.0f}"
              f"{cl_lo:>8.2f}-{cl_hi:<7.2f}{str(wt):>12}{str(fits):>6}"
              f"{vis['cam_left']:>4}/{vis['cam_right']}{gap:>9.1f}")
        if not args.verify:
            tray.export(OUT_STL / f"{cls}_tray.stl")
            marker_sheet(cls, spec, OUT_STL / f"{cls}_markers.png")
        cfg_lines.append(f"  {cls}:")
        cfg_lines.append(f"    socket_depth_m: {info['socket_depth_m']:.4f}")
        cfg_lines.append(f"    plate_m: [{px:.4f}, {py:.4f}, {pz:.4f}]")
        t = info["t_tray_object"]
        cfg_lines.append(f"    t_tray_object: [{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}]")
        cfg_lines.append("    markers:")
        for m in info["markers"]:
            mt = m["t_tray_marker"]
            cfg_lines.append(f"      - id: {m['id']}")
            cfg_lines.append(f"        t_tray_marker: [{mt[0]:.6f}, {mt[1]:.6f}, {mt[2]:.6f}]")
        cfg_lines.append(f"    measured_clearance_mm: [{cl_lo:.3f}, {cl_hi:.3f}]")
        ok &= min(vis.values()) >= 1 and gap_ok
        cfg_lines.append(f"    card_to_socket_gap_mm: {gap:.2f}")
        report[cls] = dict(mass_g=mass, watertight=bool(wt), fits_bed=bool(fits),
                           clearance_mm=[cl_lo, cl_hi], visible=vis, card_gap_mm=gap)

    if not args.verify:
        OUT_CFG.write_text("\n".join(cfg_lines) + "\n")
        print(f"\nwrote {OUT_CFG}")
        print(f"wrote {OUT_STL}/*.stl and *_markers.png")
    print(f"\ntotal tray filament: {sum(r['mass_g'] for r in report.values()):.0f} g "
          f"({WALL_MM} mm wall, {int(INFILL*100)}% infill)")
    worst = max(r["clearance_mm"][1] for r in report.values())
    print(f"worst-case socket clearance {worst:.2f} mm -> ground-truth translation "
          f"bounded by that, vs a 19.6 mm ADD-S threshold on the bottle "
          f"({100*worst/19.6:.1f}%)")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
