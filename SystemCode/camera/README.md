# camera — RealSense capture, sync, and depth characterisation

Produces the aligned RGB-D frames behind `INTERFACES.md` §1.

## Build and test

```bash
cmake -S SystemCode -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure

build/camera/camera_probe config/cameras.yaml 60
build/camera/depth_characterise config/cameras.yaml cam_front --distance 0.4
```

librealsense is **optional** at build time. Config validation, timestamp-skew
analysis, plane fitting, and depth back-projection are pure C++ and are always
built and tested; only the capture wrapper needs the SDK.

## Serial-only addressing is enforced by the interface

`CLAUDE.md` constraint 4 forbids selecting a camera by `/dev/video*` or
enumeration order. There is deliberately **no index-based accessor** anywhere in
this API — `EnumerateDevices()` even sorts its results by serial, discarding
enumeration order rather than passing it on. The constraint is structural, not
something to remember.

`RealSenseCamera` binds its pipeline with `enable_device(serial)` and then
asserts the opened device reports that same serial. That check should be
impossible to trip, which is exactly why it is worth having: silently opening
the wrong unit is the failure the rule exists to prevent, and it would produce
fused poses that are wrong while every module stays internally consistent.

## Status on the real rig — 2026-08-20

Both units attached and addressed by serial:

| camera_id | serial | firmware | link |
|---|---|---|---|
| cam_front | `261722073945` | 5.17.3.10 | USB 3.2 |
| cam_over | `261822074567` | 5.17.3.10 | **USB 2.1** |

Both flashed to 5.17.3.10 on 2026-08-20, so the firmware mismatch is resolved.
Note `rs-fw-update` segfaults during teardown *after* reporting success — exit
139 is cosmetic, confirm with `rs-fw-update -l` rather than the exit code.

**All acceptance criteria now met on hardware** (`camera_probe`, 2026-08-20):

| Check | Result |
|---|---|
| Both selected by serial | Pass — opened serial asserted against the request |
| Both streaming 848×480 aligned depth + colour, simultaneously | **30.0 fps each** (criterion: ≥ 15 fps) |
| Cross-camera timestamp skew | mean 1.9 ms, p95 4.2 ms, **max 4.3 ms** (criterion: within one 33 ms frame period) |
| Timestamp domain | `Global Time` on both |
| Depth validity | 97.8% over the central ROI |

Skew is a **one-sided offset**, not jitter — every sample carries the same sign,
so it is a fixed pipeline delay rather than free-running drift, and the report
says so. This is with **no hardware sync cable**; free-running agreement at a few
milliseconds is already comfortably inside the requirement.

Run-to-run the offset varies (1.9 ms and 10.2 ms on two consecutive runs) — it
depends on where in each camera's frame period the pipelines happen to start.
Both are well inside tolerance, but do not read a single run's mean as a
constant property of the rig.

### Hardware sync — cable connected 2026-08-25

**The trigger is confirmed reaching the slave.** Before the cable, cam_over in
slave mode opened successfully and then delivered **no frames at all** (a slave
captures only when triggered, so the pipeline hung to the timeout). With the
cable it streams at 30 fps. That transition is unambiguous, and it is what R4b's
mitigation requires. `config/cameras.yaml` now ships master/slave (1 / 2).

**What could NOT be measured, and why.** Three attempts to quantify sync
*quality* all came back degenerate:

| approach | result |
|---|---|
| Global-Time frame timestamps | jitter 4–15 ms, **indistinguishable from free-running** in a 4-run A/B |
| `sensor_timestamp` clock drift | both modes report exactly 33.344 ms period, zero drift — the value is quantised to the nominal frame period, not a free-running clock |
| depth validity with both streaming | 92.9% vs 92.9% — but the cameras were not co-aimed, so there was no interference to mitigate |

The first is a limitation of the probe: it reads the two cameras sequentially
through independently buffered pipelines, so it measures queue jitter rather
than exposure skew. Free-running already met the one-frame-period requirement,
which leaves little headroom to detect an improvement.

**The right test is deferred to rig mounting.** R4b is about *IR
cross-interference*, not timestamp alignment — sync interleaves the projectors
so they do not corrupt each other's decode. That only manifests when both
cameras illuminate the **same surface**, which is precisely the geometry that
exists after the cameras are aimed at the table for calibration. Re-run the
depth-quality comparison then; until both views overlap, the measurement has
nothing to measure.

## Three bugs real hardware found

**1. Sync mode set too late.** `inter_cam_sync_mode` was being set **after**
`pipeline.start()`. librealsense
rejects that — *"Cannot change Inter-camera HW synchronization mode while
streaming!"* — and the first probe run reported `sync mode 0 (requested 1)`.
The option is now set through the context before the pipeline starts, and reads
back as 1.

This mattered more than it looks: the failure was silent apart from a warning,
and the cameras would have run free-running with R4b's mitigation quietly not
applied. Hardware sync still additionally needs a physical cable between the two
units' sync connectors — the option is not a software-only setting, so the value
is always read back rather than assumed.

**2. Mode 0 was never written.** The driver only wrote the sync mode when it was
non-zero, so `0` could not turn sync *off*. The setting lives in the camera's
NVRAM and survives power cycles, so a unit left in slave mode from an earlier run
stayed stuck there — opening fine and then delivering no frames. The mode is now
written unconditionally.

**3. Skew was measured from the first frame.** The two pipelines do not start at
the same instant, so early frames pair across a large gap. Max skew read
**265.80 ms**; after discarding a 15-frame warm-up it reads **4.31 ms**. The
first number was measuring start-up ordering, not synchronisation — and it would
have looked like a serious sync failure. Throughput was understated too, 25 fps
against an actual 30.

## Backing up factory calibration

```bash
build/camera/backup_calibration calibration_backup
```

Saves each unit's raw 512-byte on-device calibration table plus a readable record
of all 31 stream profiles and 12 inter-stream extrinsic pairs. The table is
written per unit at the factory and is **not reproducible locally** — and if it
were corrupted, nothing downstream would report an error. Depth would simply be
wrong while every module stayed self-consistent.

Backups for both units are committed under `calibration_backup/`; see the README
there for the restore path. Re-run after a firmware update, and before ever
running an on-chip self-calibration.

## Depth-noise characterisation is a deliverable

`AGENTS.md` §1 is explicit that this is not a debugging step: the
per-distance plane-fit RMS is the measured input to the evaluation error budget,
and it is what distinguishes a fused-pose error that is sensor noise from one
that is an algorithm problem.

```bash
# once per distance, camera square-on to a flat, matte, textured wall
build/camera/depth_characterise config/cameras.yaml cam_front --distance 0.4
```

### Measured results — 2026-08-23, firmware 5.17.3.10

Flat wall, 30 frames per point, **matched 28 × 16 cm wall patch at every
distance** (see the methodology note below — this matters). All rows
`planar? = yes`, ≥98.8% valid pixels.

| distance | ROI | cam_front RMS | /dist | cam_over RMS | /dist |
|---|---|---|---|---|---|
| 0.4 m | 0.50 | 0.98 mm | 0.24% | 0.75 mm | 0.19% |
| 0.7 m | 0.29 | 2.69 mm | 0.38% | 2.44 mm | 0.35% |
| 1.0 m | 0.20 | 4.77 mm | 0.48% | 4.27 mm | 0.42% |

Both units follow the expected stereo error law closely:

```
cam_front   RMS ≈ 4.9 · z²  mm     (power-law fit: z^1.73)
cam_over    RMS ≈ 4.4 · z²  mm     (power-law fit: z^1.92)
```

Residuals against the quadratic are within ±13% except cam_front at 0.4 m
(+24%, where absolute error is under 1 mm and quantisation dominates). The two
units agree to **9–10%** at 0.7 and 1.0 m — they behave as a matched pair.

**Error budget.** At the working distances the cameras are actually mounted at
(0.60 m and 0.75–0.90 m), depth noise is **2–4 mm**. That is the floor for
fused-pose accuracy: a fused error at or below it is sensor noise, not an
algorithm problem. It is also the target for Replicator's rendered depth noise,
which the RGB-D detector depends on (`docs/RGBD_REWORK.md`).

### Methodology: hold the PHYSICAL patch constant, not the pixel ROI

A fixed pixel ROI covers a wall area that grows linearly with distance —
28 × 16 cm at 0.4 m but **70 × 40 cm at 1.0 m** for `roi 0.50`. Once the ROI
outgrows the flat surface it silently includes edges and background, and the
plane-fit residual reads as sensor degradation.

This is not hypothetical. The first pass, all at `roi 0.50`, produced:

| | roi 0.50 | matched roi | |
|---|---|---|---|
| cam_over @ 1.0 m | 41.30 mm | **4.27 mm** | 9.7× |
| cam_over @ 0.7 m | 12.32 mm | **2.44 mm** | 5.1× |
| cam_front @ 1.0 m | 45.09 mm | **4.77 mm** | 9.5× |

Those numbers would have gone into the error budget as a sensor property. They
would also have supported a wrong conclusion — the first pass appeared to show
cam_over ~3× noisier than cam_front, which vanished entirely once the patch was
matched.

**The tell is in the tool's own output:** plane offset and median depth agree to
within 1 mm when the ROI is on the wall, and diverge by 20–27 mm when it is not.
Check that the two columns match before trusting an RMS. Use `--roi` to keep the
patch constant:

```
0.4 m -> --roi 0.50    0.7 m -> --roi 0.29    1.0 m -> --roi 0.20
```

Two choices worth knowing about:

**Total least squares, not `z = ax + by + c`.** The fit minimises perpendicular
distance. A vertical-distance least-squares fit is biased whenever the plane is
tilted relative to the camera, and a real wall is never perfectly
fronto-parallel. `PlaneFit.IsUnbiasedOnATiltedPlane` pins this.

**Central ROI only.** The frame edges see the wall most obliquely and carry the
worst depth, which would dominate an RMS taken over the whole image.

The whole measurement path is validated against synthetic walls with *known*
injected noise — `RmsMatchesInjectedNoise` recovers a 3 mm sigma to within
0.4 mm — so the metric is trustworthy before any wall is measured.

`--scene` measures whatever is in front of the camera as a smoke test, and says
loudly in its own output that the result is an upper bound, not a
characterisation.

## Acceptance criteria status

| Criterion (`AGENTS.md` §1) | Status |
|---|---|
| Cameras selected by serial number | **Met** — enforced by the API having no index accessor; opened serial is asserted |
| Both cameras stream aligned depth + colour at ≥ 15 fps, 848×480, simultaneously | **Met** — 30.0 fps each |
| Serial mapping survives a USB port swap | **Met incidentally** — the cameras were replugged across four different ports and controllers during bring-up; namespace assignment followed the serial every time |
| Frame timestamps agree within one frame period, asserted at startup | **Met** — max 4.3 ms against a 33 ms budget |
| Depth-noise characterisation report | **Tooling done and validated synthetically.** No measured wall yet, so no reportable figures |
| Mock mode: replay from a recorded bag | **Not built** — `pose_agent` and `fusion` already have their own replay/mock paths, so nothing downstream is currently blocked on it |
