# Physical rig — bill of materials and dimensions

Derived from measured numbers, not catalogue claims:

- Colour intrinsics `fx ≈ 605 px` at 848×480 → **70.0° H / 43.3° V FOV**
- Depth noise (matched 28×16 cm patch, both units): **0.4 m → 0.75–0.98 mm,
  0.7 m → 2.44–2.69 mm, 1.0 m → 4.27–4.77 mm**. Roughly `σ ≈ 4.5·z² mm`.
- Chosen standoff **0.75 m**, elevation **35°**, two oblique views.

At 0.75 m each camera sees **1.05 × 0.60 m** on a fronto-parallel plane and
carries **~2.6 mm** depth noise. That is the sweet spot: closer buys accuracy
but loses the workspace, further loses accuracy fast (quadratic).

---

## Derived rig geometry (the numbers everything below is sized from)

**Surface: the existing computer table, 1180 × 440 mm free.** Workspace set to
**600 × 360 mm** centred in it (40 mm margin front and back for the table-plane
fit). Three objects plus occlusion arrangements fit that comfortably — the
bowl is only 150 mm across.

Standoff **d = 0.70 m**, elevation **35°**, azimuth **±40°** from the front
(→ **80°** separation), measured from the **workspace centre**:

| Quantity | Value |
|---|---|
| Camera height above tabletop | **402 mm** |
| Camera horizontal radius | **573 mm** |
| Camera position (x = along the 1180 width, y = toward you) | **±369, −439 mm** |
| Tripod clearance in front of the table edge | **219 mm** |
| Camera-to-camera spacing | **737 mm** |
| Camera height above floor (table 740 mm) | **1.14 m** |
| Depth noise at this standoff | **~2.2 mm** |
| Horizontal coverage | **981 mm** (need 600) |

Plan view (mm), looking down:

```
                        back edge
   +-------------------------------------------------+
   |                                                 |
   |            workspace  600 x 360                 |  440
   |                    (o) centre                   |
   |                                                 |
   +-------------------------------------------------+
                        1180
                        front edge
              219 mm clear
   [cam_L]                                     [cam_R]
    x=-369                                      x=+369
    y=-439                                      y=-439
    402 mm above the tabletop, 35° down, 80° apart
```

### Why the shallow table is fine — and in two ways better

- **Vertical FOV is not a constraint.** For an *oblique* camera the footprint
  elongates along the viewing direction: at d = 0.70 m it spans **1426 mm** of
  table depth (309 mm short of centre to 1117 mm beyond). You need 360. The
  earlier "vertical FOV binds" finding applied to a near-overhead view only.
- **Tripod clearance improves.** A shallow table puts the workspace centre only
  220 mm behind the front edge, so the tripods stand **219 mm clear** — versus
  ~170 mm on the 600 mm-deep table I originally specced.
- **80° separation is back on.** On a deep table, widening the azimuth was what
  drove the tripods into the table legs, which forced 110°. Here the binding
  direction is the table's *width* (1180 mm), and the tripods only foul past
  ~±50°. So you get the separation I wanted in the first place.

Standoff/azimuth trade (elevation 35°, "clear" = tripod clearance past the
front edge; below ~150 mm the legs foul the table):

| d (m) | Noise | ±35° clear | ±40° clear | ±45° clear | ±55° clear |
|---|---|---|---|---|---|
| 0.60 | 1.6 mm | 183 | 157 | 128 ✗ | 62 ✗ |
| 0.65 | 1.9 mm | 216 | 188 | 156 | 85 ✗ |
| **0.70** | **2.2 mm** | 250 | **219** | 185 | 109 ✗ |
| 0.75 | 2.5 mm | 283 | 251 | 214 | 132 ✗ |
| 0.80 | 2.9 mm | 317 | 282 | 243 | 156 |

If the space under the table is open (no tower, no drawer pedestal) a tripod
leg can tuck underneath, which unlocks the 0.60 m row and its **1.6 mm** noise
— a 27% improvement. Check that before settling.

---

## A. Table — DECIDED: reuse the computer table

**1180 × 440 mm free space is sufficient.** No table purchase needed. The
consequences, in order of how much they matter:

| | Consequence |
|---|---|
| **1. Board fits with room to spare** | The 324 × 324 mm panel sits inside the 440 mm depth with 58 mm margin each side. No overhang. |
| **2. Rigidity is the real risk** | You type on this table, lean on it, and the tower may vibrate it. Extrinsics are camera→table-origin, so *any* relative movement between the cameras and the tabletop invalidates them, and nothing downstream detects it. Mitigation below. |
| **3. Check what is above it** | A monitor at the back edge will be in frame and will occlude. An overhead shelf or hutch blocks the camera at 1.14 m entirely. Measure the vertical clearance before mounting. |
| **4. Where do you sit** | The tripods occupy the seat position during capture. Plan to drive the capture over SSH or from a second position. |
| **5. Sunlight** | Computer desks live near windows, and sunlight is broadband IR that directly degrades active stereo. Blackout or capture at night. |

**Mitigation for (2) — session re-registration.** Rather than accepting that a
knocked table means a full recalibration, leave the ChArUco board (or a small
permanent fiducial) at a marked, fixed spot on the tabletop and re-detect it at
the start of every session. Intrinsics and the camera-to-camera extrinsics are
rigid to the tripods and survive; only `T_world_cam` needs re-solving, from a
single board view. That is a ~30 s check instead of a 20 min recalibration, and
it turns the table's instability from a silent correctness bug into a routine
step.

**Built.** `calibrate reregister` (and `reregister-mock` for a hardware-free
demo) — see `SystemCode/calibration/README.md`. It refuses to write when the
camera-to-camera baseline has changed, which is the case re-registration cannot
recover: a bumped tripod, as opposed to a moved desk.

Mark the workspace rectangle and the board's origin corner on the tabletop with
masking tape so both come back to the same place.

## B. Camera mounting

Two options. Pick one; do not mix.

**B1 — Tripods (recommended, ~110° separation)**

| Item | Qty | Spec |
|---|---|---|
| Tripod | 2 | Max height **≥ 1.5 m** (you need 1.14 m — never work at full extension, it wobbles), min height ≤ 0.5 m, **1/4"-20** screw |
| Head | 2 | Ball head or 3-way pan/tilt with a **friction lock** and a bubble level. Needs ≥ 45° downward tilt. |
| Quick-release plate | 2 | Arca-Swiss or similar, so a camera can come off and go back on repeatably |
| Counterweight | 2 | 2 kg sandbag or equivalent on the centre column hook |

The D435i has a **1/4"-20 UNC** thread on its base (plus two M3 holes), so it
screws straight onto a standard head.

**B2 — Table clamp + arm (if you want 70–90° separation)**

| Item | Qty | Spec |
|---|---|---|
| Super clamp / C-clamp mount | 2 | Jaw opening ≥ 40 mm to grip the tabletop |
| Magic arm / friction arm | 2 | ~28 cm reach, 1/4"-20 camera end |

Zero floor footprint, so azimuth separation is unconstrained. Downside: it
clamps to the table, so any knock to the table moves the camera *and* the
workspace together — which is actually fine — but the arm's friction joints
creep over hours. Re-check calibration more often.

## C. Cabling

| Item | Qty | Spec |
|---|---|---|
| USB-C → USB-A cable | 2 | **2 m** (bought), certified **USB 3.2 Gen 1 (5 Gbps)** minimum |
| Adhesive cable clips | ~10 | Strain relief |
| Velcro cable ties | ~10 | |

**2 m is enough — if you route it right.** Do *not* run the cable down the
tripod to the floor and across; that path is 2.5–3.5 m and 2 m will not reach.
Instead go **camera → table edge → along the tabletop to the PC**: the camera is
only 402 mm above the tabletop and 219 mm in front of it, so that is a ~0.5 m
run to the edge, leaving 1.5 m to reach the machine. Comfortable.

You have **already hit USB-2.1 fallback on this rig**, so this is the known
failure mode, not a theoretical one:

- Verify with `rs-enumerate-devices` that **both** report USB 3.2, every session.
- If either falls back: it is the cable or the port, not the camera. Do not add
  a hub.
- Keep the two cameras on **separate host controllers**.
- **Strain-relieve every cable at the tripod head.** A tug on the USB moves the
  camera by a millimetre and every fused pose is wrong by a rigid offset that
  no downstream check will catch.

**Sync cable — measure yours now.** In the chosen layout the cameras are
**737 mm apart**, and the cable has to arc up and over rather than run straight,
so you need **≥ 0.9 m** of usable length. If the cable you received is 0.5 m,
it is the binding constraint on the whole layout and nothing else matters until
it is resolved — measure it before mounting anything.

## D. Calibration target — ARRIVED 2026-08-28: 324 × 324 mm, 8 × 8 @ 38 mm

The decisive number is not the board size but the **capture resolution**, and
my earlier sizing here got it wrong twice: it used the standoff (0.70 m) as the
marker distance instead of the oblique far corner (~0.87 m), and it assumed
calibration is captured at the 848×480 streaming resolution. It is not.

A DICT_5X5 marker is 7 modules across (5 bits + a 1-module quiet border) and
stops decoding below ~3 px/module:

```
max distance = fx · marker_length / (3 · marker_modules)
```

`fx` scales with capture resolution. Against the **measured** factory colour
intrinsics in `calibration_backup/` (worst of the two units):

The board carries **8 × 8 squares at 38 mm = 304 × 304 mm of squares** in a
324 mm panel — 49 interior corners, 32 markers. Its oblique far corner on the
table sits **884 mm** from the camera.

| Capture resolution | fx (measured) | px/module @ 0.884 m | Reach | Verdict |
|---|---|---|---|---|
| 848 × 480 (streaming) | 603.43 | **2.8** | 0.82 m | ✗ fails |
| 1280 × 720 | 905.15 | 4.2 | 1.23 m | ✓ |
| **1920 × 1080** | **1357.72** | **6.1** | **1.81 m** | ✓ 2.05× margin |

**So the board works — capture calibration at 1920 × 1080.** It also frames
fully at that resolution, and sits in the 440 mm table depth with 58 mm margin
each side. That is not
a workaround for a board that is too small: the recovered camera *pose* is
resolution-independent (same projection centre, differently sampled), so it
stays valid while the pipeline streams 848 × 480. This is exactly what
`calibrate run --factory` exists for, and it is now mandatory rather than
optional on this rig.

`BoardSpec::MaxDecodeDistance` computes this, and the CLI prints the reach
**before** detecting — because a board that cannot decode returns zero corners
from images that look perfectly good to a human: in frame, lit, in focus.

### [VERIFY] on arrival — four values, none of them guessable

| Value | How | Why it matters |
|---|---|---|
| `squares_x` / `squares_y` | **8 × 8** per the listing — count them anyway. | A wrong count maps corner ids to the wrong 3D points — the classic ChArUco bug. The 0.5 px gate catches it, but the message will not say so. |
| `square_length_m` | **Calipers.** 38 mm is nominal. | Boards are routinely 1–2% off. This value scales *every distance in the system* while every reprojection error stays small. |
| `marker_length_m` | **MEASURED 28.0 mm** (2026-09-05). The ratio is 0.7364, not the 0.75 assumed sight-unseen. | Feeds the decode reach and the pose solve. |
| `dictionary` | Read the listing. | Must match what is printed or **nothing** decodes. Reach is not the issue — DICT_4X4/5X5/6X6 give 7.5/6.4/5.6 px/module at 1080p. |

`BoardSpec.LoadsTheShippedConfig` asserts all of these. **It is meant to fail
when the board arrives** if the physical board disagrees with the config — that
failure is the check working, not a bug.

| Item | Spec |
|---|---|
| ChArUco board | 324 × 324 mm panel, 8 × 8 @ 38 mm (**arrived 2026-08-28**) |
| Substrate | Aluminium composite or 5 mm foam-PVC — **rigid and flat**. Laminated paper curls and every corner is then wrong. |
| Finish | **Matte**. Gloss blows out under the IR projector and under your lights. |

You do **not** need an IR-specific coated board — the D435i's *colour* sensor
does the corner detection, and the extrinsics come from the factory-calibrated
depth↔colour transform.

At 49 interior corners this board gives less to fit than the 720 × 540 would
have (88), but comfortably clears `min_shared_corners: 12`. With `--factory`
the intrinsic sweep is skipped entirely anyway, which is the intended path.

## E. Surface and lighting

| Item | Spec | Why |
|---|---|---|
| Tabletop mat | **1200 × 600 mm**, matte, mid-grey or beige | Gloss kills the IR return; matte black absorbs it. Mid-grey also gives the RGB detector a neutral background. Cheap: matte vinyl sheet, foam board, or a large cutting mat. |
| LED panel light | 2 × bi-colour, ~30 × 20 cm, dimmable, on stands | Diffuse, repeatable illumination for the RGB half of the detector |
| Diffuser / softbox | 2 | Kill specular highlights on the bottle and bowl |
| Blackout curtain | 1 | **Sunlight is broadband IR** and directly degrades active stereo. Halogen/incandescent too. LEDs are IR-poor — that is why they are the right choice here. |

## F. Measurement tools — the part that decides whether the numbers are real

| Item | Spec | What it is for |
|---|---|---|
| **Digital calipers** | 150 mm, 0.01 mm | **Mandatory.** `calibration.yaml` explicitly demands a measured square length — printed boards are routinely 1–2% off, and that error scales *every distance in the system* while every reprojection error stays small. Also for measuring the three objects against their meshes. |
| **Digital angle gauge / inclinometer** | ±0.1°, magnetic base | Setting the 35° elevation repeatably |
| Steel tape measure | 5 m, mm graduations | Standoff, radius, table marking |
| Spirit level | 200 mm torpedo | Levelling the table and the tripod heads |
| Laser distance meter | ±1.5 mm (optional) | Faster, more repeatable standoff than a tape |
| Machinist's square / protractor | | Marking the workspace axes on the table |

For azimuth separation, skip the protractor: measure the two standoff distances
and the camera-to-camera chord, then use the cosine rule. Tape measures beat
angle gauges over a metre.

## G. Consumables

| Item | Use |
|---|---|
| Gaffer tape | Mark tripod leg positions on the floor — so the rig comes back to the same place after it is moved |
| Masking tape + fine marker | Mark the workspace rectangle and origin on the tabletop |
| Thread-lock or tape on the head knobs | Stop slow creep |

## H. Objects

The three classes in `config/detector.yaml` are `mustard_bottle`, `red_cube`,
`bowl`. `config/meshes.yaml` extents are **measured** (2026-09-04) and
`MeshRegistry.MeshesAreCentredOnTheirAabb` re-derives them from the OBJ files at
test time, so they cannot drift.

| Item | Spec | Note |
|---|---|---|
| `mustard_bottle` | YCB 006 = French's Classic Yellow **20 oz** (not 14 oz — identified from the texture atlas, "40% MORE THAN OUR 14 OZ", and corroborated by the YCB paper's 603 g) | 97.2 × 66.6 × 191.4 mm. **3D printed** — the 20 oz is not retail-available in Singapore (§6.2) |
| `red_cube` | 50 mm, authored, exact by construction | 50.0 × 50.0 × 50.0 mm. The printed part is what must match the mesh, not the reverse |
| `bowl` | YCB 024 `google_16k` | 161.5 × 161.2 × 55.0 mm, scanned wall **0.99 mm** |
| Distractor objects | 8 in the renders; 3–5 physical is enough | Must be in the trained palette — see "Distractors need colour too" below |

**Everything must be matte.** One specular object will dominate your ADD-S
error and it will look like a fusion bug.

### Test print — do this BEFORE the filament arrives

The calliper check in §6.2 is mandatory ("the printer's dimensional accuracy
verified with callipers against a test print before any object is relied upon")
and it is **colour-blind**. So is every other mechanical question. Printing a
test part in whatever filament is on hand converts the delivery wait into
de-risking, and finds a printer 1% out of true now rather than on the day the
real spools land.

**Print `red_cube` as the test part, not a generic 20 mm calibration cube.** It
is authored rather than scanned, so it is exact by construction and any deviation
is printer error with nothing else to blame. At 50 mm it also gives 2.5× the
measurement precision of a 20 mm coupon for the same callipers, and you need the
part anyway. Measure all three axes; a bed-slinger's Y is the axis most likely to
differ from X.

What a test print settles, none of which depends on colour:

- **Dimensional accuracy** — the §6.2 gate. 1% out reintroduces exactly the
  object/mesh mismatch that printing was chosen to avoid, and it is invisible
  afterwards: the detector still fires, FoundationPose still returns a pose, and
  only ADD-S silently carries the difference.
- **The ArUco seat geometry fits**, once authored. §6.3 makes the seat the
  ground-truth mechanism, so a seat that does not hold a marker flush is a
  ground-truth defect, not a cosmetic one.
- **The bowl's 0.99 mm wall actually prints.** Thinner than two perimeters at a
  0.4 mm nozzle, so it comes out as a solid thin wall with the infill setting
  inert. Do NOT thicken it — ADD-S is computed over mesh vertices.
- **Whether the bowl's 6 stray two-face fragments leave debris** on the plate.

**Colour: anything except grey.** The detector was trained on exactly six
colours — the three targets plus three distractor neutrals (`#E7E7E7`,
`#898989`, `#BCB3AA`). Grey is not merely unseen, it is trained as a *distractor*,
i.e. an explicit negative, so a grey test part is the one wrong colour the network
was actively taught not to fire on. An off-palette colour is only unknown, which
is strictly better.

Two constraints still apply even to a throwaway part, because they affect
**depth** rather than detection: **matte** (gloss defeats the IR projector) and
**mid-tone** (dark absorbs it). A black or shiny test print gives unrepresentative
depth even with perfect geometry, so pick the most matte mid-tone spool on hand if
you want the part to say anything about point-cloud quality.

**No detection metric from an off-palette part is reportable.** It is out of
distribution for the RGB half and comparable to nothing. It *is* worth running as
a deliberate robustness probe — if the detector still fires on an off-colour cube,
colour is not the only cue it learned, which is worth knowing given the modality
ablation showed depth contributing nothing. Record it as an ablation, never as a
headline number.

### Bambu Lab A1 notes

All three objects fit the 256 × 256 × 256 mm volume with room to spare; the
bottle at 191.4 mm is the tallest. The bowl (161 mm) and bottle (97 mm) do not
sit side by side across a 256 mm bed with skirt clearance, so plan two plates
rather than letting auto-arrange decide.

- **Turn off any scale-to-fit.** §6.2 requires 100% with no bed-fitting rescale.
  Nothing here is near the volume limit, so a rescale would be silent and
  gratuitous — and it would invalidate every mesh-derived number in
  `config/meshes.yaml`.
- **The bottle is 191 mm of tall, narrow part on a bed-slinger.** Y-axis bed
  motion is the ringing axis; slow the outer wall rather than accepting surface
  artefacts, since layer texture on this object is what the depth channel sees.
- **Enable thin-wall detection for the bowl** so the 0.99 mm wall is emitted as
  solid extrusions rather than dropped as an unfillable gap. Verify in preview
  that the wall exists before committing 41 g to it.
- **Layer lines are beneficial**, not a defect — they add surface texture for
  active stereo. Do not chase a glossy finish.
- **ArUco markers: paper in the modelled recess, not printed two-tone.** An AMS
  lite makes a black-on-white printed marker possible, but module edges are then
  limited by nozzle width — roughly 10% edge error on a 3.75 mm module — and that
  lands directly in corner localisation, i.e. in the ground truth itself. A
  laser-printed paper marker seated in the modelled recess keeps §6.3's
  exact-by-construction offset while giving sharp corners.

---

## Buy now vs. later

**Bought:** 2 m USB cables, tripods, angle gauge.

**Not needed:** table — reusing the computer table (§A).

**ARRIVED 2026-08-28:** ChArUco board, 324 × 324 mm, 8 × 8 @ 38 mm. Decode
verified on a real capture — ids 0–31 under `DICT_5X5_250`, measured reach
1.82 m (`261722073945`) / 1.81 m (`261822074567`) at 1920 × 1080. The square
length is still `[VERIFY]`: span as many squares as the callipers reach and
divide by that count — 3 squares (114 mm) with the 150 mm callipers below.
Capture
calibration at 1920 × 1080 — see §D.

**Now (cheap, blocks the build):** tabletop mat 1180 × 440 mm matte mid-grey,
**digital calipers**, masking + gaffer tape.

**Filament — 3 × 1 kg matte PLA, 1.75 mm.** The colours are NOT a free choice.
`training/detector/replicator_dataset.py` renders each class in a fixed base
colour with `FILAMENT_SPREAD = 0.12` of linear randomisation, and `synth50k_v2`
— and therefore the trained detector — has already learned those distributions.
A spool outside the band introduces an appearance gap *after* training:

| class | colour | sRGB target | acceptable band |
|---|---|---|---|
| `mustard_bottle` | yellow | `#EDDD59` | `#DECB00` – `#FCEC81` |
| `red_cube` | red | `#DA5555` | `#C80000` – `#EA7E7E` |
| `bowl` | blue | `#6199D7` | `#007CC5` – `#86B1E7` |

**Matte, not silk.** Gloss defeats the IR projector and dark absorbs it — the
same rule as the board and the mat. The renderer draws these at roughness
0.55–0.90, so a glossy print depicts something the training set does not contain.

Quantity, computed from the meshes: **227 g per object set** — `mustard_bottle`
137 g (1.2 mm wall, 10% infill), `red_cube` 49 g (20% infill), `bowl` 41 g. Two
sets plus a 20 mm test cube per colour plus 10% waste comes to ~303 g yellow,
~110 g red, ~93 g blue, so 1 kg spools leave room for printed distractors.

**A fourth spool, grey — ORDERED 2026-09-05.** The ArUco ground-truth trays
(§H, `training/author_marker_trays.py`) come to **385 g** — `red_cube` 74 g,
`mustard_bottle` 119 g, `bowl` 193 g. A tray is not a detected class, so it must
NOT be printed in a target colour: that would put a large slab of a class's own
colour directly under the object. Grey is one of the three distractor neutrals
the renderer already samples (`#E7E7E7`, `#898989`, `#BCB3AA`), so a tray reads
as exactly the kind of object the training set already contains. Only one set of
trays is needed, unlike the objects.

Two print constraints that follow from the meshes rather than from taste:

- **The bowl's scanned wall is 0.99 mm**, thinner than two perimeters at a
  0.4 mm nozzle, so it prints as a solid thin wall and infill is inert. Do NOT
  thicken it for durability — ADD-S is computed over mesh vertices, so any
  deviation enters the pose error and cannot be separated from estimator error.
- **Both YCB OBJs are UV-seam split**, which makes naive watertightness checks
  report false. After merging texture/normal-split vertices the bottle is
  watertight and the bowl has 0 boundary edges, so both slice without repair.
  The bowl carries 6 stray two-face fragments from the scan — delete them in the
  slicer so they do not print as debris.

**Distractors need colour too.** `DISTRACTOR_COUNT = 8`, and the renderer draws
distractors from the *same palette as the targets* — that is the change that
stopped the benchmark saturating and made the modality ablation able to say
anything. §6.3 says distractors need no meshes, so household objects qualify,
but if the real ones are arbitrary colours the real scenes will not test what the
synthetic ones do. Either source real objects in these three colours or print
simple shapes (~190 g across the three).

**Later (nice to have):** LED panels, blackout curtain, laser distance meter,
quick-release plates, counterweights.

**Not on this list because it is not a purchase:** the CAD/USD meshes for the
three classes. That is still the critical-path blocker for every detection
number, and no amount of hardware moves it.

## Build order

1. Table in place, levelled, workspace rectangle marked.
2. Objects measured with calipers → real numbers into `config/meshes.yaml`.
3. Tripods positioned, cameras mounted, elevation set, **cables strain-relieved**.
4. Tripod leg positions taped to the floor.
5. **Then** calibrate, with `--factory` intrinsics (required if you intend to
   re-register later).
6. Tape the board's origin corner position on the tabletop, and run
   `calibrate reregister` at the start of every session.
7. R4b check: both cameras co-aimed, projectors on, confirm cross-interference
   is tolerable with the sync cable in master/slave.
