#!/usr/bin/env python3
"""Omniverse Replicator RGB-D dataset generation with domain randomisation.

The REAL dataset path. `generate_dataset.py` draws primitives with OpenCV and
exists only so the C++ inference path could be verified before Isaac Sim;
numbers from it are not reportable.

Writes exactly what training consumes -- no intermediate format, no conversion
step:
    <out>/images/{train,val}/*.png   RGB, for inspection
    <out>/images/{train,val}/*.npy   [H, W, 4] uint8 = R, G, B, depth
    <out>/labels/{train,val}/*.txt   YOLO: class cx cy w h, normalised
    <out>/data.yaml                  with `channels: 4`

Run inside Isaac Sim's Python, NOT the project venv:
    ~/.local/share/ov/pkg/isaac-sim-5.1.0/python.sh replicator_dataset.py \
        --out data/synth --frames 5000

NOTE: Kit swallows stdout. Progress goes to <out>/generation.log.

STATUS 2026-09-04: runs end to end, emits valid RGB-D + YOLO labels, and the
scene now carries real meshes and a real room. See "Scene fidelity" below.

Hard-won settings, each of which failed silently before being found:
  - SimulationApp needs renderer="RayTracedLighting". The default headless
    config renders RGB pure BLACK while depth and boxes still work.
  - A Dome light is REQUIRED. With only Sphere/Distant lights the beauty pass
    stays black even though geometry is present.
  - Cameras must be created at a CONCRETE position and jittered inside the
    trigger. Creating them with a distribution leaves them unplaced -- every
    frame renders empty background at 0% finite depth.
  - rep.modify.pose(scale=...) sets ABSOLUTE size in metres; it does not
    multiply the existing scale. Stage metersPerUnit is 1.0.

Randomisation axes, and why each is here:
  lighting     intensity, colour temperature, position -- the largest sim-to-real
               gap for a colour detector
  materials    roughness/metallic on distractors, so the net cannot key on a
               constant surface appearance
  pose         position, yaw, and mild tilt across the table
  camera       jitter around the two real viewpoints, so the net is not
               overfitted to one extrinsic
  distractors  primitives that are NOT target classes, which teaches rejection
               rather than "anything object-shaped is a target"
  occlusion    deliberate overlap -- the whole project is about occlusion
  depth noise  the MEASURED D435i model (depth_simulation.py), not clean depth

Scene fidelity, fixed 2026-09-04. Measured before/after, `--diagnose`:

    depth validity   4-15%  ->  100.0%
    labels/frame       ~0   ->  3.00
    nearest surface  1007 mm -> 470 mm  (the table corner is 458 mm away)

  1. NEAR CLIP. The USD default near plane is 1 METRE. This is a tabletop
     scene: the camera stands 0.70 m from the workspace centre and the nearest
     table corner is 0.46 m, so the entire subject sat inside the near plane and
     was clipped. What rendered instead was the floor and the far wall -- a
     bright, plausible frame with high depth validity and no objects in it. This
     was the defect that made the dataset unusable, and nothing about the output
     said so.
  2. THE SCENE HAD NO ROOM. At the rig geometry the vertical FOV sweeps a
     1427 mm band across the table plane while the desk is only 440 mm deep, so
     69% of every frame hit no geometry. Replicator returns inf there, which
     encodes as the invalid sentinel -- while a real D435i pointed at a desk
     sees the room and returns finite depth that clips to "far". Sim produced
     ~85% sentinel against ~1-15% in reality, a distribution mismatch in the
     exact channel the RGB-D detector depends on. A floor and a back wall now
     stand where the room stands, and sentinel is 1.5%.
  3. RESTING HEIGHT. The object frame's origin is the mesh AABB centre, so each
     class rests at its own half-height (96 / 25 / 28 mm). One shared z range
     buried some classes and floated others -- while the 2D boxes stayed
     correct, so the labels looked perfect and `(on obj table)` was false.
  4. Camera intrinsics are the MEASURED per-unit factory values. The default
     24 mm / 20.955 mm aperture is 47 deg horizontal against the D435i's 70 deg.
  5. Camera placement is the derived rig (docs/RIG_BOM.md): 0.70 m standoff,
     35 deg elevation, +/-40 deg azimuth -- previously the placeholder
     extrinsics, which described a rig that was never built.
  6. Real CAD meshes via assets/meshes/usd (training/obj_to_usd.py).

Every one of those rendered frames that looked fine from the outside. Run
`--diagnose N` after ANY scene change: it reports what was actually rendered --
depth validity, the camera transform and FOV read back from USD, and measured
world bounds -- and fails loudly rather than writing a plausible dataset.

Known limitation: target placement is sampled independently per class, so
objects can interpenetrate. Occlusion is wanted -- it is the whole project --
but occlusion is objects overlapping in the IMAGE, not in space.
rep.randomizer.scatter_2d(check_for_collisions=True) was tried and is worse: it
clustered every object at the table centre, dropped one class out of frame, and
silently overrode the per-class materials, while still not preventing
interpenetration. Physics-based settling is the real fix.

"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

# Must match config/detector.yaml model.class_names and the keys in
# config/meshes.yaml -- this string is the join key through the whole pipeline.
CLASSES = ["mustard_bottle", "red_cube", "bowl"]

# MEASURED factory colour intrinsics per unit at 848x480 (calibration_backup/).
# The two cameras differ by 0.6%, which is small but free to model exactly.
#
# USD cameras are specified as focal length and aperture, not fx. For a sensor
# of `aperture` mm imaged at `width` px through a `focal` mm lens,
#     fx = width * focal / aperture   =>   aperture = width * focal / fx
# The focal length is arbitrary (only the ratio matters); 24 mm is conventional.
CAMERAS = {
    "cam_front": {"serial": "261722073945", "fx": 607.01, "fy": 606.79,
                  "azimuth_deg": -40.0},
    "cam_over":  {"serial": "261822074567", "fx": 603.43, "fy": 603.39,
                  "azimuth_deg": +40.0},
}
FOCAL_MM = 24.0
# Near/far clip, metres. The USD default near plane is 1 m, which is BEHIND
# everything in a tabletop scene -- see apply_intrinsics.
CLIPPING_RANGE_M = (0.02, 100.0)

# Derived rig geometry, docs/RIG_BOM.md. Not chosen by eye: computed from the
# measured FOV against the measured depth-noise law.
STANDOFF_M = 0.70
ELEVATION_DEG = 35.0
# Mount tolerance, metres. The real cameras sit where the tripods put them, not
# where the design says, so the detector must not be tuned to one exact pose.
CAM_JITTER_M = 0.03

# The real work surface: the desk in docs/RIG_BOM.md §A, 1180 x 440 mm free.
TABLE_W, TABLE_D = 1.18, 0.44
# Objects are placed within the workspace, not the whole desk.
WORKSPACE_W, WORKSPACE_D = 0.60, 0.36
# Desk height. Sets where the floor goes relative to the tabletop, which is the
# world z = 0 plane.
DESK_HEIGHT_M = 0.74
# The desk stands against a wall just behind its back edge.
WALL_Y_M = TABLE_D / 2 + 0.06

# Every target class is 3D printed (proposal §6.2): the bowl and cube because
# printing reproduces the mesh exactly, and the mustard bottle because the YCB
# original -- French's 20 oz -- is not retail-available here. So the scanned
# textures on the USD meshes describe objects that will never be photographed,
# and rendering them would build in an appearance gap by construction. The
# renders instead use filament: a per-class base colour with enough spread to
# cover filament batch, lighting and exposure, but not so much that colour stops
# being a usable cue.
FILAMENT = {
    "mustard_bottle": (0.85, 0.72, 0.10),   # yellow, as the real product is
    "red_cube":       (0.70, 0.09, 0.09),
    "bowl":           (0.12, 0.32, 0.68),   # blue: distinct from the other two
}
FILAMENT_SPREAD = 0.12
# Matte. docs/RIG_BOM.md requires matte objects because gloss defeats the IR
# projector, so a glossy render would depict something the rig will not contain.
FILAMENT_ROUGHNESS = (0.55, 0.9)

# Fallback if a class has no USD. Kept only so the pipeline still runs without
# assets; a detector trained on these has learned primitives, not objects.
FALLBACK_SHAPES = {
    "mustard_bottle": ("cylinder", (0.049, 0.033, 0.096)),
    "red_cube":       ("cube",     (0.025, 0.025, 0.025)),
    "bowl":           ("sphere",   (0.081, 0.081, 0.028)),
}


def camera_position(azimuth_deg: float):
    """Rig position for one camera, from the derived geometry."""
    elev = math.radians(ELEVATION_DEG)
    az = math.radians(azimuth_deg)
    r = STANDOFF_M * math.cos(elev)
    return (r * math.sin(az), -r * math.cos(az), STANDOFF_M * math.sin(elev))


def apply_intrinsics(prim_path: str, fx: float, fy: float, width: int, height: int, log):
    """Write measured intrinsics onto a USD camera, then read them back.

    Replicator's create.camera accepts aperture arguments, but silently ignoring
    one would leave the default 47 deg FOV in place while everything else looked
    correct -- so the values are set through USD and verified here rather than
    trusted.
    """
    import omni.usd
    from pxr import Gf, Usd, UsdGeom

    stage = omni.usd.get_context().get_stage()
    # Replicator hands back the camera's Xform, not the Camera prim -- the
    # Camera sits beneath it. Search the subtree rather than assuming the shape.
    root = stage.GetPrimAtPath(prim_path)
    if not root:
        raise RuntimeError(f"nothing at {prim_path}")
    cam = None
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Camera):
            cam = UsdGeom.Camera(prim)
            break
    if not cam:
        raise RuntimeError(f"no USD camera under {prim_path}")
    h_aperture = width * FOCAL_MM / fx
    v_aperture = height * FOCAL_MM / fy
    cam.GetFocalLengthAttr().Set(FOCAL_MM)
    cam.GetHorizontalApertureAttr().Set(h_aperture)
    cam.GetVerticalApertureAttr().Set(v_aperture)
    # The default near clip is 1 METRE. This is a tabletop scene: the camera
    # stands 0.70 m from the workspace centre and the nearest table corner is
    # 0.46 m away, so the entire subject sits inside the default near plane and
    # is silently clipped. What renders instead is the floor and the far wall --
    # a bright, plausible, completely useless frame with 94% depth validity and
    # no objects in it. The D435i itself stops at ~0.20 m.
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(*CLIPPING_RANGE_M))

    near, far = cam.GetClippingRangeAttr().Get()
    if near > 0.1:
        raise RuntimeError(f"near clip {near} m would cut the tabletop scene")
    f = cam.GetFocalLengthAttr().Get()
    ha = cam.GetHorizontalApertureAttr().Get()
    va = cam.GetVerticalApertureAttr().Get()
    hfov = math.degrees(2 * math.atan(ha / (2 * f)))
    vfov = math.degrees(2 * math.atan(va / (2 * f)))
    log(f"    intrinsics: fx={width * f / ha:.2f} fy={height * f / va:.2f} px  "
        f"FOV {hfov:.2f} x {vfov:.2f} deg  (target fx={fx:.2f} fy={fy:.2f})  "
        f"clip {near:.2f}-{far:.0f} m")
    if abs(width * f / ha - fx) > 0.5:
        raise RuntimeError("camera aperture did not take: rendered fx does not match the sensor")


def load_extents(log):
    """Half-extents per class, from config/meshes.yaml.

    Read rather than hardcoded: these are measured from the meshes and are
    already asserted against them by the C++ MeshRegistry tests, so duplicating
    them here would create a second copy free to drift.
    """
    import yaml
    cfg = Path(__file__).resolve().parents[2] / "config" / "meshes.yaml"
    objects = yaml.safe_load(cfg.read_text())["objects"]
    out = {}
    for name in CLASSES:
        if name not in objects:
            raise RuntimeError(f"config/meshes.yaml has no entry for '{name}'")
        out[name] = [float(v) for v in objects[name]["extent"]]
        log(f"  {name}: half-extents {out[name]} -> rests at z={out[name][2] * 1000:.1f} mm")
    return out


def build_scene(rep, mesh_dir, width, height, log):
    """Room, table, targets, distractors, lights, two cameras."""
    extents = load_extents(log)
    # THE ROOM. Without it 69% of every frame hits no geometry and Replicator
    # returns inf, which encodes as the invalid sentinel -- whereas the real
    # sensor, pointed at a desk, sees floor and wall and returns finite depth.
    # This was the single largest defect in the synthetic depth channel.
    rep.create.plane(scale=(6.0, 6.0, 1.0), position=(0, 0, -DESK_HEIGHT_M),
                     semantics=[("class", "floor")])
    rep.create.plane(scale=(6.0, 3.0, 1.0), position=(0, WALL_Y_M, 0.6),
                     rotation=(90, 0, 0), semantics=[("class", "wall")])

    # The work surface, at world z = 0.
    table = rep.create.plane(scale=(TABLE_W, TABLE_D, 1.0), position=(0, 0, 0),
                             semantics=[("class", "table")])

    targets, target_by_class = [], {}
    for name in CLASSES:
        usd = (mesh_dir / f"{name}.usd") if mesh_dir else None
        if usd and usd.exists():
            node = rep.create.from_usd(str(usd), semantics=[("class", name)])
            log(f"  {name}: USD {usd}")
        else:
            kind, half = FALLBACK_SHAPES[name]
            maker = {"cube": rep.create.cube, "sphere": rep.create.sphere,
                     "cylinder": rep.create.cylinder}[kind]
            node = maker(scale=tuple(h * 2 for h in half), semantics=[("class", name)])
            log(f"  {name}: STAND-IN {kind} (no USD -- results not reportable)")
        targets.append(node)
        target_by_class[name] = node

    distractors = rep.create.group([
        rep.create.cube(scale=0.05),
        rep.create.sphere(scale=0.05),
        rep.create.cylinder(scale=0.05),
    ])

    # A Dome light is REQUIRED, not decorative: with only Sphere/Distant lights
    # the RGB pass renders black even though depth and boxes are correct.
    rep.create.light(light_type="Dome", intensity=rep.distribution.uniform(300, 1200))
    rep.create.light(
        light_type="Sphere",
        intensity=rep.distribution.uniform(2e3, 4e4),
        temperature=rep.distribution.uniform(2500, 9000),
        position=rep.distribution.uniform((-2, -2, 1.5), (2, 2, 4)),
        count=3)

    # Cameras get CONCRETE positions and are jittered inside the trigger.
    # Creating them with a distribution leaves them unplaced -- the distribution
    # is never sampled and every frame renders empty background.
    cams = {}
    for name, spec in CAMERAS.items():
        pos = camera_position(spec["azimuth_deg"])
        cams[name] = rep.create.camera(position=pos, look_at=(0, 0, 0))
        log(f"  {name} ({spec['serial']}) at "
            f"[{pos[0] * 1000:.0f}, {pos[1] * 1000:.0f}, {pos[2] * 1000:.0f}] mm")

    half_w, half_d = WORKSPACE_W / 2, WORKSPACE_D / 2
    target_group = rep.create.group(targets)
    with rep.trigger.on_frame():
        with target_group:
            # +/-2 deg, not +/-8. An object resting on a flat table is flat; the
            # residual is surface irregularity and print warp, not free tilt.
            # It also bounds how far a tilted object dips through the tabletop,
            # since resting height below is computed for the untilted pose.
            rep.modify.pose(rotation=rep.distribution.uniform((-2, -2, -180),
                                                              (2, 2, 180)))
        # Per class, because two things differ per class and a shared randomiser
        # gets both wrong.
        #
        # RESTING HEIGHT. The object frame's origin is the mesh AABB centre
        # (config/meshes.yaml), so an object rests on the table exactly when its
        # z equals its own half-height: 96 mm for the bottle, 25 mm for the cube,
        # 28 mm for the bowl. One shared z range buries some classes in the table
        # and floats others above it -- and the 2D boxes stay correct throughout,
        # so the label files look perfect while `(on obj table)` is false in the
        # very data the predicates will be evaluated against.
        #
        # COLOUR. The classes are partly distinguishable by colour; randomising
        # all of them from one distribution erases that cue.
        for name, node in target_by_class.items():
            rest_z = extents[name][2]
            with node:
                rep.modify.pose(position=rep.distribution.uniform(
                    (-half_w, -half_d, rest_z), (half_w, half_d, rest_z)))
            base = FILAMENT[name]
            lo = tuple(max(0.0, c - FILAMENT_SPREAD) for c in base)
            hi = tuple(min(1.0, c + FILAMENT_SPREAD) for c in base)
            with node:
                rep.randomizer.materials(rep.create.material_omnipbr(
                    diffuse=rep.distribution.uniform(lo, hi),
                    roughness=rep.distribution.uniform(*FILAMENT_ROUGHNESS)))
        with distractors:
            rep.modify.pose(
                # Half of the sampled scale, so the prim rests on the table
                # rather than half-buried: these are centred primitives too.
                position=rep.distribution.uniform((-half_w, -half_d, 0.025),
                                                  (half_w, half_d, 0.04)),
                rotation=rep.distribution.uniform((0, 0, -180), (0, 0, 180)),
                # ABSOLUTE size in metres, not a multiplier -- rep.modify.pose
                # REPLACES scale. Passing 0.4-1.4 here (intended as a factor on a
                # 5 cm prim) produced 40 cm to 1.4 m distractors.
                scale=rep.distribution.uniform(0.02, 0.08))
            rep.randomizer.materials(rep.create.material_omnipbr(
                diffuse=rep.distribution.uniform((0, 0, 0), (1, 1, 1)),
                roughness=rep.distribution.uniform(0.1, 0.9),
                metallic=rep.distribution.choice([0.0, 1.0])))
        with table:
            rep.randomizer.materials(rep.create.material_omnipbr(
                diffuse=rep.distribution.uniform((0.25, 0.25, 0.25), (0.75, 0.75, 0.75)),
                roughness=rep.distribution.uniform(0.4, 0.9)))
        for name, cam in cams.items():
            pos = camera_position(CAMERAS[name]["azimuth_deg"])
            with cam:
                rep.modify.pose(
                    position=rep.distribution.uniform(
                        tuple(c - CAM_JITTER_M for c in pos),
                        tuple(c + CAM_JITTER_M for c in pos)),
                    look_at=(0, 0, 0))
    return cams


def to_yolo_rows(bbox_data, width, height):
    """Replicator tight 2D boxes -> YOLO rows, dropping anything not a target class."""
    rows = []
    data = bbox_data["data"]
    id_to_labels = bbox_data["info"]["idToLabels"]
    for box in data:
        label = id_to_labels.get(str(box["semanticId"]), {})
        name = label.get("class") if isinstance(label, dict) else str(label)
        if name not in CLASSES:
            continue
        x0, y0, x1, y1 = float(box["x_min"]), float(box["y_min"]), \
                         float(box["x_max"]), float(box["y_max"])
        bw, bh = x1 - x0, y1 - y0
        # Boxes a few pixels across carry no learnable signal and destabilise
        # the loss; Replicator emits them for barely-visible objects.
        if bw < 8 or bh < 8:
            continue
        rows.append(f"{CLASSES.index(name)} {((x0+x1)/2)/width:.6f} "
                    f"{((y0+y1)/2)/height:.6f} {bw/width:.6f} {bh/height:.6f}")
    return rows


def diagnose(rep, annots, args, log) -> int:
    """Render a few frames and report what was actually rendered.

    Every scene defect this script has hit -- black RGB, unplaced cameras,
    absolute-versus-relative scale, a missing room -- produced output that looked
    structurally fine. Rendered frames were written, labels were emitted, and
    only the pixel statistics gave it away. So the statistics are checked
    directly rather than inferred from the scene description.
    """
    import numpy as np
    import omni.usd
    from pxr import Usd, UsdGeom, Gf

    stage = omni.usd.get_context().get_stage()
    log("\n--- stage geometry (world bounds, metres) ---")
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh) and not prim.IsA(UsdGeom.Camera):
            continue
        name = prim.GetName()
        if prim.IsA(UsdGeom.Camera):
            xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            t = Gf.Vec3d(xf.ExtractTranslation())
            # A USD camera looks down its own -Z. Comparing that against the
            # direction to the origin is the only way to tell an aimed camera
            # from an unaimed one -- position alone looks correct either way.
            fwd = Gf.Vec3d(-xf.ExtractRotationMatrix()[2]).GetNormalized()
            want = (-t).GetNormalized()
            err = math.degrees(math.acos(max(-1.0, min(1.0, fwd * want))))
            log(f"  CAM  {name:22s} at [{t[0]*1000:7.0f},{t[1]*1000:7.0f},{t[2]*1000:7.0f}] mm"
                f"  range {t.GetLength()*1000:.0f} mm"
                f"  fwd [{fwd[0]:+.2f},{fwd[1]:+.2f},{fwd[2]:+.2f}]"
                f"  off-aim {err:.1f} deg")
            continue
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if box.IsEmpty():
            continue
        size = Gf.Vec3d(box.GetMax()) - Gf.Vec3d(box.GetMin())
        mid = (Gf.Vec3d(box.GetMax()) + Gf.Vec3d(box.GetMin())) * 0.5
        log(f"  MESH {name:22s} size [{size[0]*1000:6.0f},{size[1]*1000:6.0f},"
            f"{size[2]*1000:6.0f}] mm   at [{mid[0]*1000:7.0f},{mid[1]*1000:7.0f},"
            f"{mid[2]*1000:7.0f}] mm")

    log(f"\n--- rendered frames (n={args.diagnose}) ---")
    stats = {name: {"valid": [], "near": [], "far": [], "boxes": [], "rgb": []}
             for name in annots}
    for _ in range(args.diagnose):
        rep.orchestrator.step(rt_subframes=4)
        for name in annots:
            depth = np.asarray(annots[name]["distance_to_camera"].get_data(), np.float32)
            rgb = np.asarray(annots[name]["rgb"].get_data()[:, :, :3])
            boxes = annots[name]["bounding_box_2d_tight"].get_data()
            hit = np.isfinite(depth) & (depth < 1e6)
            stats[name]["valid"].append(float(hit.mean()))
            if hit.any():
                stats[name]["near"].append(float(depth[hit].min()))
                stats[name]["far"].append(float(np.percentile(depth[hit], 95)))
            stats[name]["boxes"].append(len(to_yolo_rows(boxes, args.width, args.height)))
            stats[name]["rgb"].append(float(rgb.mean()))

    ok = True
    for name, st in stats.items():
        valid = float(np.mean(st["valid"])) if st["valid"] else 0.0
        near = float(np.mean(st["near"])) if st["near"] else float("nan")
        far = float(np.mean(st["far"])) if st["far"] else float("nan")
        log(f"  {name}: depth hit {valid*100:5.1f}%   near {near*1000:6.0f} mm   "
            f"p95 {far*1000:6.0f} mm   labels/frame {np.mean(st['boxes']):.2f}   "
            f"rgb mean {np.mean(st['rgb']):.1f}")
        # A real D435i pointed at a desk sees the room; anything under ~80% means
        # the frame is still looking at empty space where reality has geometry.
        if valid < 0.80:
            log(f"    FAIL: {(1-valid)*100:.0f}% of the frame hit no geometry. The real "
                f"sensor returns depth there, so this trains the net that "
                f"sentinel means background.")
            ok = False
        if np.mean(st["rgb"]) < 5.0:
            log("    FAIL: RGB is essentially black -- renderer or lighting.")
            ok = False
        if np.mean(st["boxes"]) < 0.5:
            log("    FAIL: almost no labelled targets in frame.")
            ok = False
    log("\nDIAGNOSE: " + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--meshes", type=Path,
                        default=Path(__file__).resolve().parents[2] / "assets/meshes/usd",
                        help="directory of <class>.usd; stand-in primitives if absent")
    parser.add_argument("--diagnose", type=int, default=0, metavar="N",
                        help="render N frames, report what was ACTUALLY rendered, "
                             "write nothing. Run this after any scene change.")
    parser.add_argument("--frames", type=int, default=5000, help="frames PER CAMERA")
    parser.add_argument("--val-fraction", type=float, default=0.2)
    parser.add_argument("--width", type=int, default=848)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--seed", type=int, default=20260825)
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    log_path = args.out / "generation.log"
    def log(msg):
        with open(log_path, "a") as f:
            f.write(str(msg) + "\n")
    log(f"=== generation start: {args.frames} frames/camera -> {args.out} ===")

    from isaacsim import SimulationApp
    # renderer MUST be set explicitly. The default headless config brings up the
    # base experience, whose RGB pass renders pure black while depth and
    # bounding boxes still work -- a failure that looks like an empty scene
    # rather than a renderer setting.
    app = SimulationApp({"headless": True, "renderer": "RayTracedLighting"})
    try:
        import numpy as np
        import cv2
        import omni.replicator.core as rep
        sys.path.insert(0, str(Path(__file__).parent))
        from depth_simulation import add_sensor_noise, encode_depth

        rng = np.random.default_rng(args.seed)
        rep.set_global_seed(args.seed)

        cams = build_scene(rep, args.meshes, args.width, args.height, log)
        resolution = (args.width, args.height)

        products, annots = {}, {}
        for name, cam in cams.items():
            rp = rep.create.render_product(cam, resolution)
            products[name] = rp
            annots[name] = {}
            for ann in ("rgb", "distance_to_camera", "bounding_box_2d_tight"):
                a = rep.AnnotatorRegistry.get_annotator(ann)
                a.attach(rp)
                annots[name][ann] = a

        # Render products must exist before the camera prims are resolvable.
        for name, cam in cams.items():
            spec = CAMERAS[name]
            log(f"  {name}:")
            apply_intrinsics(str(rep.utils.get_node_targets(cam.node, "inputs:primsIn")[0]),
                             spec["fx"], spec["fy"], args.width, args.height, log)

        if args.diagnose:
            return diagnose(rep, annots, args, log)

        for split in ("train", "val"):
            (args.out / "images" / split).mkdir(parents=True, exist_ok=True)
            (args.out / "labels" / split).mkdir(parents=True, exist_ok=True)

        written = 0
        t_start = time.time()
        for i in range(args.frames):
            rep.orchestrator.step(rt_subframes=4)
            split = "val" if rng.random() < args.val_fraction else "train"

            for cam_name in ("cam_front", "cam_over"):
                rgb = annots[cam_name]["rgb"].get_data()[:, :, :3]
                depth_m = annots[cam_name]["distance_to_camera"].get_data()
                boxes = annots[cam_name]["bounding_box_2d_tight"].get_data()

                rows = to_yolo_rows(boxes, args.width, args.height)
                if not rows:
                    if i < 3:
                        raw = boxes["data"]
                        log(f"  [diag] {cam_name} frame {i}: {len(raw)} raw boxes, "
                            f"labels={boxes['info'].get('idToLabels')}")
                        if len(raw):
                            log(f"  [diag]   first={raw[0]}")
                        log(f"  [diag]   depth finite="
                            f"{float(np.isfinite(np.asarray(depth_m,np.float32)).mean()):.3f} "
                            f"rgb mean={float(np.asarray(rgb).mean()):.1f}")
                    continue   # no target visible; an empty frame teaches nothing

                # Replicator returns inf where no geometry was hit. That is the
                # same thing the sensor reports as a hole, so it must map to the
                # sentinel -- clipping it would invent a surface at max range.
                depth_raw = np.asarray(depth_m, np.float32)
                hit = np.isfinite(depth_raw) & (depth_raw < 1e6)
                noisy, valid = add_sensor_noise(np.where(hit, depth_raw, 0.0), rng)
                depth_u8 = encode_depth(noisy, valid & hit)

                stem = f"{cam_name}_{i:06d}"
                images = args.out / "images" / split
                cv2.imwrite(str(images / f"{stem}.png"), rgb[:, :, ::-1])
                np.save(images / f"{stem}.npy",
                        np.dstack([rgb, depth_u8]).astype(np.uint8))
                (args.out / "labels" / split / f"{stem}.txt").write_text("\n".join(rows) + "\n")
                written += 1

            if (i + 1) % 25 == 0 or i + 1 == args.frames:
                elapsed = time.time() - t_start
                rate = (i + 1) / elapsed
                eta = (args.frames - i - 1) / rate
                log(f"  frame {i+1}/{args.frames}  written={written}  "
                    f"{rate:.2f} frame/s  elapsed {elapsed/60:.1f} min  "
                    f"ETA {eta/60:.1f} min")

        import yaml
        (args.out / "data.yaml").write_text(yaml.safe_dump({
            "path": str(args.out.resolve()),
            "train": "images/train",
            "val": "images/val",
            # Drives BaseDataset to load the 4-channel .npy rather than the PNG.
            "channels": 4,
            "names": {i: n for i, n in enumerate(CLASSES)},
        }, sort_keys=False))
        log(f"=== done: {written} samples written ===")
    except Exception:
        import traceback
        log("FAILED:\n" + traceback.format_exc())
        raise
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
