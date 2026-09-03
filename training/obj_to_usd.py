#!/usr/bin/env python3
"""Convert assets/meshes/*.obj to USD for Replicator. Run once, offline.

Replicator's `rep.create.from_usd` needs USD; `prepare_meshes.py` produces OBJ
because that is what the upstream sources ship and what stays diffable. This
bridges the two.

Run inside Isaac Sim's Python, not the project venv:

    ~/.local/share/ov/pkg/isaac-sim-5.1.0/python.sh training/obj_to_usd.py

Output goes to assets/meshes/usd/<class>.usd, which is what
`replicator_dataset.py --meshes` expects.

The conversion preserves the object frame, so the AABB-centre origin established
in prepare_meshes.py carries through — verified after conversion rather than
assumed, because a converter that silently re-origins or rescales would undo the
whole reason the meshes were re-centred (config/meshes.yaml).
"""

from __future__ import annotations

import asyncio
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SRC = REPO / "assets" / "meshes"
OUT = SRC / "usd"

# Metres. The stage is authored at metersPerUnit = 1.0 and the OBJ files are in
# metres, so any deviation beyond this is the converter changing units on us.
TOLERANCE_M = 1e-4


def main() -> int:
    # Kit swallows stdout, so everything goes to a log file. A print() here is
    # silently discarded, which makes a failed conversion look like a clean run.
    log_path = OUT.parent / "obj_to_usd.log"
    OUT.parent.mkdir(parents=True, exist_ok=True)
    log_path.write_text("")

    def log(msg):
        with open(log_path, "a") as handle:
            handle.write(str(msg) + "\n")

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    status = 0
    try:
        import omni.kit.asset_converter as converter
        from pxr import Usd, UsdGeom, Gf

        OUT.mkdir(parents=True, exist_ok=True)

        async def convert(src: Path, dst: Path) -> bool:
            ctx = converter.AssetConverterContext()
            # Keep the authored geometry as-is. Any of these left on default can
            # re-origin, re-scale, or merge meshes, all of which would break the
            # object-frame contract.
            ctx.ignore_materials = False
            ctx.ignore_animations = True
            ctx.ignore_camera = True
            ctx.ignore_light = True
            ctx.single_mesh = True
            ctx.merge_all_meshes = True
            task = converter.get_instance().create_converter_task(
                str(src), str(dst), None, ctx)
            ok = await task.wait_until_finished()
            if not ok:
                log(f"  FAILED: {task.get_status()} {task.get_error_message()}")
            return ok

        loop = asyncio.get_event_loop()
        for obj in sorted(SRC.glob("*.obj")):
            dst = OUT / f"{obj.stem}.usd"
            log(f"{obj.name} -> {dst.relative_to(REPO)}")
            if not loop.run_until_complete(convert(obj, dst)):
                status = 1
                continue

            # Verify the contract survived. A converter that re-origins the mesh
            # would silently reintroduce the 43.7 mm bowl offset this project
            # went to some trouble to remove.
            stage = Usd.Stage.Open(str(dst))
            prim = next((p for p in stage.Traverse() if UsdGeom.Mesh(p)), None)
            if prim is None:
                log("  no mesh prim found in the converted stage")
                status = 1
                continue
            bounds = UsdGeom.Imageable(prim).ComputeWorldBound(
                Usd.TimeCode.Default(), UsdGeom.Tokens.default_)
            box = bounds.ComputeAlignedRange()
            centre = (Gf.Vec3d(box.GetMin()) + Gf.Vec3d(box.GetMax())) * 0.5
            size = Gf.Vec3d(box.GetMax()) - Gf.Vec3d(box.GetMin())
            worst = max(abs(c) for c in centre)
            log(f"  size {[round(v, 4) for v in size]} m   "
                f"centre offset {worst * 1000:.3f} mm   "
                f"{'ok' if worst <= TOLERANCE_M else 'NOT CENTRED'}")
            if worst > TOLERANCE_M:
                status = 1
    except Exception:
        import traceback
        log("FAILED:\n" + traceback.format_exc())
        status = 1
    finally:
        app.close()
    return status


if __name__ == "__main__":
    raise SystemExit(main())
