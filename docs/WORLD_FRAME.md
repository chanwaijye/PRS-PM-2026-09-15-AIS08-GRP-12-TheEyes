# The world origin is mostly a burden we chose

Written 2026-09-08, prompted by the obvious question: establishing the world
frame is the fiddliest part of this rig — the board has to go at a known spot,
its thickness has to be measured, its origin corner has to be reasoned about, and
`calibration.yaml` warns that an error there is invisible to every downstream
check. Is all of that actually necessary?

**Mostly not.** Almost nothing that is graded depends on where the origin is.

---

## 1. What actually depends on the origin

| Consumer | Depends on the origin? | Why |
|---|---|---|
| Camera↔camera extrinsics | **No** | A wrong `T_world_board` is a rigid transform applied to *both* cameras, so it cancels in the relative pose. `config/calibration.yaml` says this itself: the round trip "asks whether the two cameras agree with each other", and cannot see a world-placement error. |
| **ADD-S, the occlusion ablation** | **No** | ADD-S compares an estimated pose to a GT pose. Both live in the world frame, so a rigid offset applied to both cancels exactly. **The headline number is gauge-invariant.** |
| Predicates (`on`, `in`, `clear`) | **No** — but see §2 | They need objects and the table in the *same* frame, not in a *particular* frame. |
| Absolute heights handed to PM1 | **Gone** | PM1 integration is cut (`docs/PM1_INTEGRATION.md`). This was the one consumer that cared about absolute world position, and it no longer exists. |
| Digital twin | **No** | Needs internal consistency, not a chosen origin. |

The origin is **gauge**: a free choice that cancels out of every quantity we
report. The effort currently spent pinning it down buys almost nothing.

## 2. What is *not* free

Two things are real, and neither is "where the origin is":

1. **The table plane, relative to the cameras.** `on` uses a 15 mm surface
   tolerance (`config/predicates.yaml`) and the one real pose observation we have
   is already 5 mm high, so a few mm of error in the support plane matters. Today
   that plane is *declared* — it comes from the board's measured 7.1 mm thickness
   sitting in `T_world_board`.
2. **Whether the cameras have moved relative to the table.** This is the silent
   killer `RIG_BOM.md` flags, and it is what `reregister` exists to catch.

Everything else about the origin is bookkeeping.

---

## 3. How SLAM handles exactly this

SLAM never tries to make its origin coincide with a physical landmark, because it
cannot and does not need to.

- **The gauge is arbitrary and explicitly so.** The first keyframe *defines* the
  frame. An RGB-D or stereo map is determined only up to a rigid transform;
  a monocular map only up to a similarity — the extra degree of freedom being
  scale. This is called gauge freedom, and the usual treatment is to fix it by
  holding one pose (or adding a weak prior) purely to stop the optimisation being
  singular, not because that pose is special.
- **Evaluation aligns before measuring.** Absolute Trajectory Error is computed
  *after* Umeyama-aligning the estimate to ground truth. Nobody reports an
  unaligned trajectory error, because it would mostly measure the arbitrary
  starting frame rather than the estimator. This is the same reasoning that makes
  our ADD-S gauge-invariant.
- **Physically meaningful axes come from sensors, not fiducials.** When a system
  needs a true "up", it takes gravity from the IMU — fixing roll and pitch and
  leaving yaw and translation free — rather than asking the operator to place
  something squarely. Planes from depth do the same job for a support surface.
- **Constraints are relative.** Bundle adjustment and loop closure optimise
  relative geometry; the absolute frame is a by-product.

The transferable idea is not "run SLAM" (§6). It is: **fix the gauge arbitrarily,
and make every check gauge-invariant.**

---

## 4. Three changes, cheapest first

### (a) Stop sanity-checking the rig with gauge-dependent numbers — free, today

The current check is `RIG_BOM.md`'s: the recovered camera positions should come
out near (±0.369, −0.439, +0.402). That is only meaningful if the board really
was at the workspace centre, which is precisely the assumption we cannot verify.
Replace it with quantities that hold no matter where the board sat:

| Check | Measured from `config/extrinsics.yaml` (2026-09-20) | Design |
|---|---|---|
| camera-to-camera baseline | **738.1 mm** | 737 mm |
| camera-to-camera relative rotation | **79.19°** | 80° |
| each camera's height above the *fitted* table plane | 396 / 398 mm | 402 mm |

The figures above are the current set; when this section was written they read
749.9 mm / 82.0° / 386 / 392 mm off the 09-06 extrinsics. The check has since
earned its keep twice — it caught the 09-09 move from the committed file with
no rig, and on 09-20 it refused a re-registration that would otherwise have
written a slipping mount's pose into the calibration
(`CAPTURE_PROTOCOL.md` §4.2e, §4.2f).

The first two are pure arithmetic over the committed extrinsics and need no rig,
no capture and no board placement — and on the current set they agree with the
design to 1.1 mm and 0.8°, which is a **stronger** statement than the current
check because nothing about board placement can fake it. (On the 09-06 set the
same two agreed to 13 mm and 2°.)

### (b) Fit the table plane instead of declaring it — half a day

`docs/CAPTURE_PROTOCOL.md` §4.3 already proposes fitting the tabletop from depth
as a *cross-check*. Promote it to the *definition*: z = 0 is the fitted plane and
+z is its normal, measured rather than asserted.

This deletes an entire class of bug. The 7.1 mm board-thickness offset was
already found once as "a few mm of pure fiction" on every absolute height; a
fitted plane cannot carry that error because it never asks how thick the board
is. `FitPlane` exists and is tested (`SystemCode/camera/src/depth_noise.cpp`).

The D435i's IMU gives an independent gravity vector, which is a free cross-check
on the fitted normal — the SLAM move, available on this hardware.

### (c) Let the board sit anywhere both cameras see it — follows from (a) and (b)

Once the rig is checked gauge-invariantly and the plane is measured, the board's
only remaining job is **camera↔camera extrinsics**, which never needed a known
table position in the first place. Put the origin wherever is convenient — the
pattern centre projected onto the fitted plane, or the centroid of the shared
field of view — and stop measuring the board's placement on the desk entirely.

Note what this *also* dissolves: `calibration.yaml`'s warning that the panel's
~10 mm border "may not be symmetric, and taping by the panel would offset the
whole world frame". Under (c) that offset is a gauge choice, not an error.

---

## 5. What the jig is still for

A narrower job than it looked. Under (a)–(c) the board no longer needs a *known*
position — but `reregister` still needs a *repeatable* one, because it detects
camera movement by re-detecting the board where it was last time.

So `training/author_board_jig.py` earns its place, and only that.

Could the board go away entirely? Almost. Re-fitting the plane detects a change
in camera-to-table pose in three degrees of freedom (height and two tilts), and
the baseline check detects camera-to-camera change. Between them they would miss
one case: **both cameras translating or yawing together, rigidly, over the
table** — which the board does catch. That residual is why the board stays.

---

## 6. What not to do

**Do not run SLAM.** The cameras are static and rigidly mounted; there is no
motion to triangulate from, no loop to close, and no map to build. A SLAM system
here would solve a problem this rig does not have, while adding a dependency, a
failure mode, and a drifting estimate in place of a fixed calibration. Take the
gauge idea; leave the software.

**Do not remove the `[VERIFY]` markers by fiat.** §4 makes the origin *matter
less*; it does not make a guessed number true. `config/scene.yaml`'s table pose
still needs its one front-edge measurement, and under (b) the z half of it stops
being a measurement at all — it becomes the fitted plane.

---

## 7. Status

Nothing in §4 is built. (a) is arithmetic already done above and can go into the
rig checks immediately; (b) is the `table_probe` tool `CAPTURE_PROTOCOL.md` §4.3
already asks for; (c) is a documentation and convention change that follows from
the other two and costs no code.

None of this is on the critical path for the ablation — the ablation is
gauge-invariant, which is the whole point of this document. It is worth doing
because it removes recurring setup burden and a class of silent error, not
because the headline number is waiting on it.
