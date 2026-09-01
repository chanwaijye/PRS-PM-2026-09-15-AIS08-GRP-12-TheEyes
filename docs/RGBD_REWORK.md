# Scope: RGB-D detector rework

`CLAUDE.md` and `AGENTS.md` (2026-08-23) specify the detector as the project's
**student-built multimodal network** — YOLOv8 with a 4-channel early-fusion stem
taking RGB + aligned metric depth. `SystemCode/detector/` as built is RGB-only.

This is the scope to close that gap. Investigated against the installed
toolchain, not estimated from memory.

---

## The good news: ultralytics 8.4.123 is already multi-channel aware

This was the main unknown, and it lands well:

| capability | status |
|---|---|
| `DetectionModel(cfg, ch=4)` | **supported** — channel count is a constructor arg |
| `BaseDataset(channels=4)` | **supported** — `self.channels` exists |
| 4-channel `.npy` loading | **supported** — `load_image` checks `.npy` *before* the image file and validates `npy_channels == self.channels` |
| HSV augmentation on 4-ch | **already guarded** — `if img.shape[-1] != 3: # only apply to 3-channel (BGR) images` |

So no forking or monkey-patching of ultralytics. That was the risk that would
have made this a multi-week job.

**The one real gap:** there is no 4-channel *image* reader. `cv2_flag` is only
`IMREAD_GRAYSCALE` or `IMREAD_COLOR`. The route is to **pre-cache 4-channel
`.npy` files** alongside the PNGs — which the loader already prefers.

---

## Decision required before coding: invalid-depth encoding

`AGENTS.md` says *"Use a sentinel or a validity mask channel — decide once and
document it."* RealSense returns **0** for stereo holes, and left raw the network
reads that as "object at zero range."

**DECIDED 2026-08-23: sentinel, via an offset normalisation range.** Implemented
in `SystemCode/detector/include/the_eyes/detector/depth_encoding.hpp`.

```
valid   depth d ∈ [0.2, 1.5] m  ->  0.1 + 0.9 * (d - 0.2) / 1.3   ∈ [0.1, 1.0]
invalid (raw 0)                 ->  0.0
```

Invalid becomes unreachable by any valid measurement, so the network can learn it
as a distinct state, and the input stays 4 channels as specified.

The alternative — a 5th validity-mask channel — is semantically cleaner but
contradicts the "4-channel" spec, costs another channel of compute, and gives the
network a redundant signal it can already infer.

Two properties fall out of the choice and are enforced in code: out-of-range
readings are **clipped, not discarded** (they are real observations that
saturate; discarding them would fabricate holes the sensor never reported), and
depth is normalised in **metric metres, never raw uint16** (raw units depend on
the device's `depth_scale`, so a model trained on them breaks silently on a
camera with a different scale).

---

## What changes

### C++ — `SystemCode/detector/` — **DONE 2026-08-23**

Implemented; 45 detector tests pass (266 across the project).

| file | change | state |
|---|---|---|
| `depth_encoding.{hpp,cpp}` | **new** — the sentinel encoding, isolated from any model or runtime | done, 8 tests |
| `detector.hpp` | `DetectorInput` gains `depth` + `depth_scale_m`; `ModelConfig` gains `input_channels` + `DepthEncoding` | done |
| `detector.cpp` | loads and **validates** the depth config at startup | done, 2 tests |
| `onnx_detector.cpp` | N-channel tensor build; validates the model's own input channel count at load | done |
| `letterbox` | already type-agnostic — depth pads with 0, which encodes as the sentinel for free | no change needed |
| `detector_parity.cpp` | `--depth` for RGB-D models | done |
| `postprocess`, `tracker`, `mock_detector` | untouched as predicted | — |

Behaviour worth noting: a frame with no depth on an RGB-D model **throws rather
than zero-filling**, per `AGENTS.md`. Depth that is present but not the same size
as colour also throws — "aligned to colour" is a requirement, not a hint.

### Original estimate — C++ (~200 lines touched of ~900)

| file | change | size |
|---|---|---|
| `detector.hpp` | `DetectorInput` gains `depth` + `depth_scale_m`; `ModelConfig` gains `input_channels` and the depth range/sentinel constants | small |
| `letterbox.{hpp,cpp}` | generalise from hardcoded `CV_8UC3` / `Scalar(114,114,114)` to N channels with a per-channel pad value — depth padding must be the **invalid sentinel**, not 114 | medium |
| `onnx_detector.cpp` | 3→4 channel tensor build; depth normalisation + sentinel; validate the model's input channel count against config | medium |
| `detector_parity.cpp` | load and pass depth | small |
| `postprocess.{hpp,cpp}` | **unchanged** — operates on boxes | none |
| `tracker.{hpp,cpp}` | **unchanged** | none |
| `mock_detector.{hpp,cpp}` | **unchanged** — fixture-driven | none |

New tests: letterbox on 4 channels with per-channel padding; depth
normalisation incl. the sentinel; model/config channel-count mismatch rejected.

The decode, NMS, tracking and parity machinery — the parts that were fiddly to
get right — carry over untouched.

### Python — `training/detector/` — **PROTOTYPE DONE 2026-08-25**

Trained end to end on the procedural dataset: **mAP@0.5 = 0.995** (a plumbing
check on toy data, not a detection result), exported to a 4-channel ONNX
(`input [1, 4, 640, 640]`), and **C++/Python parity: 6/6 frames, 14 detections**.

What the integration actually needed, versus what was scoped:

| expected | actual |
|---|---|
| custom dataset subclass for 4-channel | **not needed** — `YOLODataset` does `channels=self.data.get("channels", 3)`, so `channels: 4` in `data.yaml` is the entire hook |
| `.npy` pre-cache | as scoped — `load_image` prefers the `.npy` sibling and validates its channel count |
| stem inflation | **needed, and ultralytics does NOT do it** — it copies the pretrained RGB channels and leaves the depth channel at random init (measured `\|depth − mean(RGB)\| = 0.49`). `train.py --inflate-stem` seeds it from the RGB mean |

### Python — original estimate (~250 lines touched of ~690)

| file | change |
|---|---|
| `generate_dataset.py` | render a depth channel per frame; emit 4-channel `.npy` beside the PNG |
| `train.py` | build with `ch=4`; **inflate the COCO stem** — widen conv1 3→4 and initialise the depth channel from the *mean* of the pretrained RGB weights (random init on one channel destabilises early training) |
| `export_onnx.py` | 4-channel dummy input |
| `parity_reference.py` | feed RGB-D through the reference path |
| `evaluate.py` | add the **modality ablation**: RGB-only vs RGB-D mAP, per-class |
| `replicator_dataset.py` | render depth with realistic noise — see risk below |

---

## Risks

**1. Sim-to-real on the depth channel (highest).** `AGENTS.md` is explicit:
without realistic depth noise in Replicator, the depth channel becomes the
dominant sim-to-real gap. A network trained on clean synthetic depth will lean
on a signal the real sensor does not deliver — and it would *improve* synthetic
mAP while *degrading* real mAP, which is the worst shape of failure because the
headline number looks better.

**The prerequisite is now satisfied (2026-08-23).** Depth noise is measured on
both units against a flat wall, and it follows a clean quadratic:

```
cam_front   RMS ≈ 4.9 · z²  mm       cam_over   RMS ≈ 4.4 · z²  mm
0.4 m → ~1.0 mm      0.7 m → ~2.5 mm      1.0 m → ~4.5 mm
```

So Replicator's depth channel has a **concrete target**: additive noise with
σ ≈ 4.5·z² mm, plus invalid-pixel holes at roughly the observed rate (~1% of
pixels at these distances, rising with obliquity). That is now a specification
rather than a guess, which is what turns this from the top risk into a bounded
task.

Two properties of the real sensor worth reproducing beyond the σ term: holes
cluster at depth discontinuities and on dark/specular surfaces rather than
scattering uniformly, and error grows with surface obliquity. Gaussian noise
alone would still leave a gap, just a smaller one.

**2. The modality ablation may show no gain.** — **partially answered
2026-08-25, on procedural data only.**

Rather than train two arms, the trained RGB-D model was probed by corrupting its
depth channel at inference (25 val frames):

| depth variant | anchors > 0.25 | peak score |
|---|---|---|
| real depth | 36.6 | **0.9876** |
| constant 114 | 38.4 | 0.9813 |
| all sentinel | 51.7 | 0.9758 |
| **shuffled** | 21.3 | **0.9427** |

**The channel is genuinely used** — shuffling depth costs 0.045 peak score and
nearly halves detections, so the network is not ignoring it. But a *constant*
depth barely hurts, which says RGB carries most of the signal on procedural
shapes. That is the expected shape of result here and does not predict the real
case: `AGENTS.md` puts the gain on textureless / low-contrast objects, which
procedural solid-colour shapes are not.

**A failed experiment worth recording.** A `--low-contrast` generator mode was
added to make objects blend into the background (measured: mean |object −
background| fell from 68.9/255 to **7.0/255** in RGB while depth held at
86.8/255). RGB-only still scored **mAP@0.5 = 0.983** on it. Removing the
*colour* cue does not remove the RGB signal — solid shapes still have hard
edges, and edges alone are enough. A genuine low-contrast test needs the object
boundary masked too, not just its colour matched. The real ablation should wait
for Replicator objects rather than a procedural proxy.

**3. Parity gets harder.** — **CONFIRMED, and it found a real bug.**

Two problems surfaced, both invisible without the parity check:

**Letterbox padding disagreed.** ultralytics pads *every* channel with 114 (a
4-channel array letterboxes to `[114 114 114 114]`), so a model trained through
it learns the border as a **surface at ~0.70 m**. The C++ padded depth with the
sentinel — semantically correct, but a border the network had never seen. With a
640×480 source that border is a quarter of the input. Parity failed 1 of 6
frames until `letterbox_pad_u8` was matched to training.

**The harness paired detections by sorted index**, so two near-equal-scoring
detections swapping order read as two mismatches. Now matched by position and
class, since NMS tie-breaking is not part of the contract.

A third hypothesis — that uint8 depth quantisation in the cache would shift
scores — was tested and **disproved**: it moves scores by 3e-6, not the 3e-3
observed. Worth recording, because it was the obvious explanation and it was
wrong.

---

## Sequencing

The prerequisite is not code:

```
depth-noise characterisation                        [DONE 2026-08-23]
    -> informs Replicator depth noise  (target: sigma ~ 4.5 * z^2 mm)
        -> dataset generation
            -> training with the inflated stem
                -> parity re-verification
                    -> modality ablation
```

Suggested order of work:

1. ~~Depth-noise characterisation~~ — **DONE**, see above
2. **Decide the invalid-depth encoding** (above) — the only thing blocking the
   C++ work, and it is a five-minute decision
3. **C++ input path** — testable immediately against synthetic 4-channel inputs,
   needs no trained model, no Isaac Sim, no new hardware
4. **Training pipeline** — the `.npy` caching route, stem inflation
5. **Parity + modality ablation**

Steps 2–3 are unblocked right now. Steps 4–5 need the Replicator dataset, which
needs Isaac Sim installed.

## Effort

Roughly two working sessions for steps 3–4, plus verification. The C++ half is
well-bounded — mechanical, with tests already covering everything downstream of
the input path. The Python half carries the real uncertainty, and it sits in the
data pipeline rather than the model.
