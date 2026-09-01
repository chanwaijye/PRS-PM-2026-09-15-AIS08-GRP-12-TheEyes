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

STATUS 2026-08-25: runs end to end and emits valid RGB-D + YOLO labels. The
SCENE is not yet realistic -- see "Known gaps" at the bottom. Do not generate a
large dataset from this until those are closed.

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

Known gaps, in priority order:
  1. NO CAD MESHES. All three classes fall back to stand-in primitives. A
     detector trained on these has learned cylinders and spheres, not objects.
     This alone makes any mAP from it unreportable.
  2. CAMERA INTRINSICS ARE REPLICATOR DEFAULTS, not the D435i's. Measured
     fx ~= 605 px at 848x480 (SystemCode/camera/README.md); the rendered FOV
     currently does not match the real camera, so the sim-to-real gap includes
     a lens mismatch that has nothing to do with the objects.
  3. SCENE FRAMING. The table renders ~1 m from the camera rather than the
     configured 0.6 m, and depth validity is only 4-15% because most of the
     frame is dome background rather than tabletop. Camera placement inside
     the trigger needs verifying against the actual rendered transform.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

# Must match config/detector.yaml model.class_names and the keys in
# config/meshes.yaml -- this string is the join key through the whole pipeline.
CLASSES = ["mustard_bottle", "red_cube", "bowl"]

# Rough real-world half-extents, used when a class has no CAD mesh yet so the
# stand-in primitive is at least the right size.
FALLBACK_SHAPES = {
    "mustard_bottle": ("cylinder", (0.032, 0.032, 0.095)),
    "red_cube":       ("cube",     (0.025, 0.025, 0.025)),
    "bowl":           ("sphere",   (0.075, 0.075, 0.035)),
}


def build_scene(rep, mesh_dir: Path | None, log):
    """Table, targets, distractors, lights, two cameras."""
    # A REAL tabletop, not a 2 m floor. Stage units are metres
    # (metersPerUnit = 1.0), so scale is size directly. A 2x2 m plane viewed
    # from 0.6 m fills the frame edge to edge and pushes most of the surface
    # past the 1.5 m depth clip, leaving objects as specks against a saturated
    # background.
    table = rep.create.plane(scale=(0.9, 0.7, 1.0), position=(0, 0, 0),
                             semantics=[("class", "table")])

    targets = []
    for name in CLASSES:
        usd = (mesh_dir / f"{name}.usd") if mesh_dir else None
        if usd and usd.exists():
            targets.append(rep.create.from_usd(str(usd), semantics=[("class", name)]))
            log(f"  {name}: USD {usd}")
        else:
            # Stand-in primitive. Shape and size are approximately right, which
            # is enough to exercise the pipeline -- but a detector trained on
            # these has learned primitives, not the objects. Swap in CAD before
            # quoting any mAP.
            kind, half = FALLBACK_SHAPES[name]
            maker = {"cube": rep.create.cube, "sphere": rep.create.sphere,
                     "cylinder": rep.create.cylinder}[kind]
            targets.append(maker(scale=tuple(h * 2 for h in half),
                                 semantics=[("class", name)]))
            log(f"  {name}: STAND-IN {kind} (no CAD mesh -- results not reportable)")

    distractors = rep.create.group([
        rep.create.cube(scale=0.05),
        rep.create.sphere(scale=0.05),
        rep.create.cylinder(scale=0.05),
    ])

    # A Dome light is REQUIRED, not decorative: with only Sphere/Distant lights
    # the RGB pass renders black even though depth and boxes are correct. The
    # dome supplies the ambient environment the beauty pass needs.
    rep.create.light(light_type="Dome", intensity=rep.distribution.uniform(300, 1200))
    lights = rep.create.light(
        light_type="Sphere",
        intensity=rep.distribution.uniform(2e3, 4e4),
        temperature=rep.distribution.uniform(2500, 9000),
        position=rep.distribution.uniform((-2, -2, 1.5), (2, 2, 4)),
        count=3)

    # Cameras get CONCRETE positions here and are jittered inside the trigger
    # below. Creating them with a distribution instead leaves them unplaced --
    # the distribution is never sampled, the camera sits at the origin looking
    # at itself, and every frame renders empty background with 0% finite depth.
    cam_front = rep.create.camera(position=(0.60, 0.0, 0.40), look_at=(0, 0, 0.05))
    cam_over = rep.create.camera(position=(0.0, 0.0, 0.90), look_at=(0, 0, 0.05))

    target_group = rep.create.group(targets)
    with rep.trigger.on_frame():
        with target_group:
            rep.modify.pose(
                position=rep.distribution.uniform((-0.35, -0.25, 0.02), (0.35, 0.25, 0.10)),
                rotation=rep.distribution.uniform((-8, -8, -180), (8, 8, 180)))
            # Stand-in primitives carry no material of their own; without one
            # they render flat and the detector would key on shape alone.
            rep.randomizer.materials(rep.create.material_omnipbr(
                diffuse=rep.distribution.uniform((0.1, 0.1, 0.1), (0.9, 0.9, 0.9)),
                roughness=rep.distribution.uniform(0.2, 0.8)))
        with distractors:
            rep.modify.pose(
                # Kept on the tabletop, not floating around it.
                position=rep.distribution.uniform((-0.35, -0.25, 0.02), (0.35, 0.25, 0.10)),
                rotation=rep.distribution.uniform((0, 0, -180), (0, 0, 180)),
                # ABSOLUTE size in metres, not a multiplier -- rep.modify.pose
                # REPLACES scale rather than scaling the existing value. Passing
                # 0.4-1.4 here (intended as a factor on a 5 cm prim) produced
                # 40 cm to 1.4 m distractors that filled the frame and buried
                # the targets.
                scale=rep.distribution.uniform(0.02, 0.08))
            rep.randomizer.materials(rep.create.material_omnipbr(
                diffuse=rep.distribution.uniform((0, 0, 0), (1, 1, 1)),
                roughness=rep.distribution.uniform(0.1, 0.9),
                metallic=rep.distribution.choice([0.0, 1.0])))
        with table:
            rep.randomizer.materials(rep.create.material_omnipbr(
                diffuse=rep.distribution.uniform((0.1, 0.1, 0.1), (0.9, 0.9, 0.9))))
        # Viewpoint jitter around the real rig's mounts (config/extrinsics.yaml),
        # so the detector is not overfitted to one extrinsic.
        with cam_front:
            rep.modify.pose(
                position=rep.distribution.uniform((0.45, -0.15, 0.30), (0.75, 0.15, 0.50)),
                look_at=(0, 0, 0.05))
        with cam_over:
            rep.modify.pose(
                position=rep.distribution.uniform((-0.10, -0.10, 0.80), (0.10, 0.10, 1.00)),
                look_at=(0, 0, 0.05))
    return cam_front, cam_over


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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--meshes", type=Path, default=None,
                        help="directory of <class>.usd; stand-in primitives if absent")
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

        cam_front, cam_over = build_scene(rep, args.meshes, log)
        resolution = (args.width, args.height)

        products, annots = {}, {}
        for name, cam in (("cam_front", cam_front), ("cam_over", cam_over)):
            rp = rep.create.render_product(cam, resolution)
            products[name] = rp
            annots[name] = {}
            for ann in ("rgb", "distance_to_camera", "bounding_box_2d_tight"):
                a = rep.AnnotatorRegistry.get_annotator(ann)
                a.attach(rp)
                annots[name][ann] = a

        for split in ("train", "val"):
            (args.out / "images" / split).mkdir(parents=True, exist_ok=True)
            (args.out / "labels" / split).mkdir(parents=True, exist_ok=True)

        written = 0
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

            if (i + 1) % 50 == 0:
                log(f"  frame {i+1}/{args.frames}  written={written}")

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
