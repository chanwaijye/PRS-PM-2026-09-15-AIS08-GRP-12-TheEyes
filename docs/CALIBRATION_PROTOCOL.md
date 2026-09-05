# Calibration session protocol

Run this once, after final rigid mounting. It produces `config/extrinsics.yaml`,
which is the last synthetic input in the chain:

```
real extrinsics -> fusion on real poses -> occlusion ablation on REAL data
```

Module detail is in `SystemCode/calibration/README.md`; this is the session
script. Everything below is a step you do or a number you check, in order.

**Budget half a day and shoot generously.** The capture is cheap, the setup is
not, and discovering a bad set after the board and lighting have moved means
redoing the setup rather than the capture.

---

## Phase 0 — callipers, before anything is mounted — **DONE 2026-09-05**

Four measurements. None is guessable, and each one silently scales or biases
every downstream number if it is wrong.

| # | result |
|---|---|
| 0.1 square length | **38.025 mm** — callipers spanning 4 squares read 152.1 mm. +0.066% vs nominal |
| 0.2 square count | **8 × 8 confirmed** — `calibrate aim` resolved 49 corners on both cameras, and 7 × 7 = 49 is exactly an 8 × 8 board |
| 0.3 board thickness | **7.1 mm** — now `T_world_board` z, so world z = 0 is the tabletop rather than the printed surface |
| 0.4 dictionary | **`DICT_5X5_250` confirmed** — a mismatch decodes nothing, and markers decoded |

`marker_length_m` is the one value still `[VERIFY]`: 28.519 mm **derived** from
the measured square under uniform print scale, not measured. Low stakes — the
chessboard corners carry the solve — but callipers on a marker would close it.

| # | Measure | Into | Why it cannot be skipped |
|---|---|---|---|
| 0.1 | **Square length: span as many squares as the callipers reach, divide by that count.** The BOM's 150 mm callipers reach **3 squares (114 mm)**; 8 squares would need a 305 mm tool. Never measure a single square | `config/calibration.yaml` `board.square_length_m` | Boards are routinely 1–2% off nominal. This value scales *every distance in the system* while every reprojection error stays small — the calibration looks excellent and every fused pose is wrong by a constant factor. Spanning n squares divides your placement error by n: one square leaves ~0.13%, three leaves ~0.04% |
| 0.2 | **Count the squares.** Should be 8 × 8, 49 interior corners, 32 markers | `board.squares_x/y` | A wrong count maps corner ids to the wrong 3D points — the classic ChArUco bug. The 0.5 px gate catches it, but the message will not say so |
| 0.3 | **Board thickness, callipers on the panel edge** | `world.T_world_board` **z** | Currently `0.000`, which puts world z = 0 at the board's printed surface rather than the tabletop. Every absolute height handed to PM1 then carries that bias. Bounded, not blocking — but it is a few mm of pure fiction and takes ten seconds to remove |
| 0.4 | **Confirm the dictionary is `DICT_5X5_250`** | `board.dictionary` | Must match what is printed or *nothing* decodes |

**If 0.1 changes `square_length_m`, `T_world_board` changes with it.** The
translation entries are `4 × square_length_m`:

```
x = -(squares_x * square_length_m) / 2      # -0.152 at 38 mm
y = +(squares_y * square_length_m) / 2      # +0.152 at 38 mm
```

Editing one and not the other is a silent, self-consistent error. Then:

```bash
ctest --test-dir build -R calibration --output-on-failure
```

`BoardSpec.LoadsTheShippedConfig` **is meant to fail here** if the physical board
disagrees with the config. A failure is the check working.

---

## Phase 1 — mount, then mark

Calibration is only valid for the geometry it was shot in, and nothing
downstream can detect that a camera moved: every fused pose simply becomes wrong
by a fixed offset while staying perfectly self-consistent.

- [ ] Cameras rigidly mounted at their final positions. Target geometry from
      `docs/RIG_BOM.md`: standoff **0.70 m**, elevation **35°**, azimuth
      **±40°**, i.e. ±369, −439 mm and 402 mm above the tabletop.
- [ ] **Tape the tabletop**: mark the workspace rectangle and the board's origin
      corner. The registration shot defines the world frame, and re-registration
      later assumes the board returns to the same declared spot.
- [ ] Mark the tripod feet.
- [ ] Nothing on the table that will be leaned on or moved mid-session.

Confirm both cameras are alive and on the links they should be:

```bash
build/camera/camera_probe config/cameras.yaml
```

### Aim with the live view, before anything is tightened

Positioning tripods by eye and discovering at solve time that the two cameras
never shared enough of the board is the expensive failure — by then the board,
the lighting and the operator have all moved.

```bash
build/calibration/calibrate aim config/cameras.yaml config/calibration.yaml
# q or ESC to quit. Writes nothing.

# No display? The tripods occupy the seat position, so this is the SSH path:
build/calibration/calibrate aim config/cameras.yaml config/calibration.yaml \
    --frames 5 --save /tmp/aim.png
# prints the counts per frame and writes the same composed view as a PNG
```

It shows both colour streams side by side at the **same colour-only 1080p
profile `capture` uses**, so what you aim at is what you will shoot, and it draws
live the same three numbers `capture` gates on:

| on screen | meaning |
|---|---|
| detected corners, circled, with a bounding box | where the board is actually resolving — not where you think it is |
| per-camera **corner count**, panel border green/red | against `min_shared_corners` |
| per-camera **sharpness**, flagged `BLURRED` | below half this session's best, the same blur heuristic `capture` uses |
| bottom banner: **shared corners**, `READY` / `NOT READY` | **the number that matters.** Extrinsics come from corners *both* cameras resolved |

Put the board where the registration shot will be, then move the tripods until
the banner reads `READY` with margin — not just 12. Tighten everything, then
re-check the banner: tightening a ball head usually moves it.

Leave it running while you tape the tabletop; it costs nothing and it will tell
you if you knock something.

**Verified on hardware 2026-09-05** against both units on a bench setup — not the
mounted rig — with the physical board in view of both cameras: **49 / 49 corners
per camera, 49 shared**. An 8 × 8 ChArUco board has 7 × 7 = 49 interior corners,
so full detection on both views also confirms `squares_x/y: 8` matches the panel
(Phase 0.2). The square *length* is still `[VERIFY]` — corner count cannot tell
you that, and it is the value that scales every distance in the system.

---

## Phase 2 — capture

**Capture at 1920 × 1080. This is mandatory on this rig, not a preference.** The
board lies flat and the cameras are oblique, so its far corner sits 884 mm away.
At 848 × 480 that is 2.8 px/module against a ~3 px/module decode floor — the
board returns *zero corners from images that look perfectly good*. At 1080p the
measured reach is 1.85 m (`261722073945`) and 1.84 m (`261822074567`). The
recovered camera pose is resolution-independent, so 1080p captures stay valid
while the pipeline streams 848 × 480.

`capture` opens the **colour stream only** — a measured hardware limit, not an
optimisation: two D435i units cannot both run 1080p colour and 848 × 480 depth
simultaneously, and the second `pipeline.start()` blocks indefinitely. The sync
trigger drives the depth sensor, so the two colour streams free-run here. That
does not matter for a stationary board.

### 2a. The sweep — insurance

```bash
build/calibration/calibrate capture config/cameras.yaml config/calibration.yaml captures
# ENTER shoots, q then ENTER finishes
```

With `--factory` the solver never reads these. They are the only fallback if the
factory intrinsics turn out not to be good enough, which is why they are still
worth the ten minutes.

**Shoot ~20–24 views per camera. Vary tilt as well as distance** — a stack of
fronto-parallel views leaves focal length and board distance nearly degenerate,
and too few views is the most misleading failure mode there is: the solution is
under-constrained and the RMSE comes out *low* because it is fitting noise.
`CalibrateIntrinsics` refuses below **12**.

A schedule that covers the space without thinking about it mid-session:

| distance | tilts | roll | frames |
|---|---|---|---|
| ~0.45 m | flat, ~20°, ~40°, ~55° | 0° and ~45° | 8 |
| ~0.70 m | flat, ~20°, ~40°, ~55° | 0° and ~45° | 8 |
| ~1.00 m | flat, ~20°, ~40°, ~55° | 0° and ~45° | 8 |

Move the board around the frame too — corners near the image edge are what
constrain distortion, and the D400 colour stream reports zero distortion, so
this is mostly a check that it really is.

**Watch the printed numbers on every frame.** The tool detects as it saves and
prints corner count, relative sharpness, and shared corners. Discard and reshoot
anything with a low corner count or soft sharpness. **Motion blur is the silent
failure of a handheld sweep** — a blurred board still detects, just badly. Brace
the board or rest it against something for each shot.

Re-running against an existing directory **extends** the sweep rather than
overwriting it, so a second pass to top up is safe.

### 2b. The registration shot — this is the one that matters

Every camera pose is derived from this single pair.

- [ ] Board **flat at its measured world position**, on the marked origin corner.
- [ ] Both cameras must see it in the same shot.
- [ ] **Do not touch the board between the two frames** — the tool shoots them as
      a pair, but nudging the table counts.

```bash
build/calibration/calibrate capture config/cameras.yaml config/calibration.yaml captures --register
```

Writes `zz_register.png`, named so it sorts **last** — `run` takes the
alphabetically last image in each directory as the registration frame. If you
ever use `--prefix`, it must sort *before* `zz_register.png`.

**Check the shared-corner count it prints. The gate is 12.** If it is marginal,
reshoot now, while the board and the lighting are still where they were.

---

## Phase 3 — solve

```bash
build/calibration/calibrate run config/calibration.yaml \
    captures/cam_front 261722073945 \
    captures/cam_over  261822074567 \
    config/extrinsics.yaml --factory
```

**Use `--factory`.** The D400 carries genuine per-unit factory calibration
(measured: fx 607.01 vs 603.43 across the two units — real values, not a generic
default) and reports zero distortion on the colour stream. It is also required if
you intend to re-register later: re-registration takes intrinsics from the
device, so a previous calibration that solved its own would be compared against a
different K matrix and can false-alarm.

---

## Phase 4 — the gates

`run` exits non-zero if any of these fail. Do not hand-wave a marginal pass; the
whole ablation sits on this file.

| Gate | Threshold | What it catches — and what it does not |
|---|---|---|
| Reprojection RMSE, **per camera** | **< 0.50 px** | Says each camera is consistent *with itself*. Two cameras can both be internally perfect and still disagree about where a point is |
| **Round-trip error** | **< 5 mm** | The one that says the two views agree. Takes corners both cameras resolved, undistorts each into a world-frame sightline, and measures how far apart the rays pass. **Distrust this first when fused poses look wrong** |
| Shared corners | **≥ 12** | Extrinsics are not trusted below this |
| Views (sweep path only) | **≥ 12** | Under-constrained intrinsics report a deceptively low RMSE |

Two things the gates **cannot** catch, by construction:

- **A wrong `T_world_board`.** It is a rigid transform applied to both cameras,
  so the round trip is invariant to it (`Report.RoundTripMeasuresTheRigNotTheWorldPlacement`
  pins this to 1e-9). Phase 0.3 is the only defence.
- **A wrong `square_length_m`.** Every reprojection error stays small while every
  distance is scaled. Phase 0.1 is the only defence.

Sanity-check the output by eye before trusting it: camera heights should be
~0.40 m above the tabletop and the camera-to-camera spacing ~0.737 m. Self-
consistent is not the same as physical — an early mock put both cameras *below*
the table and reported PASS.

---

## Phase 5 — after

- [ ] Commit `config/extrinsics.yaml`. Note the registration RMSE it carries:
      that value becomes fusion's own gate, so it must be real.
- [ ] Leave the tabletop tape in place.
- [ ] Store the board flat. A curled board makes every corner wrong.

### Re-registration, every session after this one

The rig mounts cameras on floor tripods and uses a shared computer desk as the
table. The desk gets typed on and leaned on, so the tabletop moves relative to
the tripods between sessions — and because the world frame is glued to the
tabletop, that invalidates every `T_world_cam` invisibly.

Only `T_world_cam` is affected: intrinsics are rigid to the camera body, and
`T_camA_camB` is rigid to the tripods. So one board view per camera and a PnP
re-derive is enough — **~30 s, not a full recalibration**:

```bash
build/calibration/calibrate reregister config/calibration.yaml \
    config/extrinsics.yaml config/extrinsics.yaml \
    cam_front:261722073945:front.png cam_over:261822074567:over.png
```

Gate: `max_baseline_shift_m` **3 mm**. It sits below the 5 mm round-trip gate
deliberately — a baseline error of X mm surfaces as roughly X mm of round-trip
disagreement, so a rig that passes re-registration must not then fail the
calibration gate.

**If the baseline shift trips, the tripods moved, not the desk.** Re-registration
cannot fix that; redo Phase 1 onward.

---

## Traps, collected

- **OpenCV here is 4.6**, so this is the old `cv::aruco` API. Code from current
  tutorials targets 4.7's `objdetect` (`CharucoDetector`, `ArucoDetector`) and
  will not compile.
- **`capture` extends, never overwrites.** Delete `captures/` if you want a clean
  set.
- **`inter_cam_sync_mode` must be set before `pipeline.start()`**, and sync mode
  lives in camera NVRAM across power cycles. Irrelevant to this protocol —
  `capture` is colour-only — but it bites when you go back to streaming.
- **Print at 100%** if the board is ever reprinted. Printer scaling is the
  classic silent failure: every reprojection error stays small while every
  distance is wrong by a constant factor.
