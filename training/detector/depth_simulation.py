"""Shared depth encoding and sensor-noise model.

ONE definition, imported by both dataset generators. If the procedural and
Replicator pipelines encoded depth differently, models trained on one would be
silently wrong on the other — and nothing downstream would report it.

The encoding must also match, exactly:
  - SystemCode/detector/include/the_eyes/detector/depth_encoding.hpp
  - config/detector.yaml  (depth:)

NOISE MODEL. Measured on both D435i units, 2026-08-23, against a flat wall with
a matched 28x16 cm patch at each distance (SystemCode/camera/README.md):

    cam_front  RMS ~ 4.9 * z^2 mm      cam_over  RMS ~ 4.4 * z^2 mm
    0.4 m -> ~1.0 mm    0.7 m -> ~2.5 mm    1.0 m -> ~4.5 mm

Rendering clean depth would hand the network a signal the real sensor cannot
deliver. It would score BETTER on synthetic data and worse on real captures --
the worst failure shape, because the headline number improves.
"""

from __future__ import annotations

import cv2
import numpy as np

# Sentinel encoding — see depth_encoding.hpp for the rationale.
DEPTH_MIN_M = 0.20
DEPTH_MAX_M = 1.50
VALID_FLOOR = 0.1

# Measured noise coefficient, mm per m^2. Midpoint of the two units.
DEPTH_NOISE_K_MM = 4.5
# Measured hole rate at these distances.
INVALID_FRACTION = 0.01
# Fraction of edge pixels additionally dropped. Real holes CLUSTER at depth
# discontinuities and on dark/specular surfaces rather than scattering
# uniformly; a purely random mask would understate the structure of the loss.
EDGE_DROP_FRACTION = 0.5


def add_sensor_noise(depth_m: np.ndarray, rng: np.random.Generator | None = None):
    """Apply the measured D435i noise model. Returns (noisy_depth_m, valid_mask)."""
    rng = rng or np.random.default_rng()
    depth = np.asarray(depth_m, dtype=np.float32)

    # sigma grows with z^2 -- the stereo error law the measurements confirmed.
    sigma_m = DEPTH_NOISE_K_MM * 1e-3 * depth ** 2
    noisy = depth + rng.standard_normal(depth.shape).astype(np.float32) * sigma_m

    valid = rng.random(depth.shape) > INVALID_FRACTION
    finite = np.isfinite(noisy) & (noisy > 0)
    scaled = np.clip(np.nan_to_num(depth, nan=0.0, posinf=0.0) / 3.0, 0, 1)
    edges = cv2.Canny((scaled * 255).astype(np.uint8), 40, 120)
    edges = cv2.dilate(edges, np.ones((3, 3), np.uint8))
    valid &= ~((edges > 0) & (rng.random(depth.shape) < EDGE_DROP_FRACTION))
    valid &= finite
    return noisy, valid


def encode_depth(depth_m: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Metric depth -> uint8 channel. Invalid becomes 0, unreachable by any valid value."""
    t = (np.clip(depth_m, DEPTH_MIN_M, DEPTH_MAX_M) - DEPTH_MIN_M) / (DEPTH_MAX_M - DEPTH_MIN_M)
    out = np.round((VALID_FLOOR + t * (1.0 - VALID_FLOOR)) * 255.0).astype(np.uint8)
    out[~valid] = 0
    return out


def decode_depth(encoded: np.ndarray) -> np.ndarray:
    """Inverse of encode_depth. Invalid pixels come back as NaN."""
    enc = np.asarray(encoded, dtype=np.float32) / 255.0
    t = (enc - VALID_FLOOR) / (1.0 - VALID_FLOOR)
    out = DEPTH_MIN_M + t * (DEPTH_MAX_M - DEPTH_MIN_M)
    return np.where(np.asarray(encoded) == 0, np.nan, out)
