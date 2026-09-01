#!/usr/bin/env python3
"""
Intel RealSense D435i hardware smoke test.

Three independent checks, so a failure in one does not mask the others:
  1. Device discovery + firmware / USB link report
  2. Depth + color streaming: measured FPS, dropped frames, depth validity
  3. IMU (accel + gyro) streaming

Artifacts (color PNG, colorized depth PNG, raw depth .npy) go to ./out.
Exit status is non-zero if any check fails.
"""

import argparse
import os
import sys
import time

import numpy as np
import pyrealsense2 as rs

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")


def hr(title):
    print("\n" + "=" * 64)
    print(title)
    print("=" * 64)


def check_device():
    """Report every camera_info field the device exposes."""
    hr("1. DEVICE DISCOVERY")
    devices = list(rs.context().query_devices())
    if not devices:
        print("FAIL: no RealSense device found")
        return None, False

    dev = devices[0]
    if len(devices) > 1:
        print(f"note: {len(devices)} devices present, testing the first")

    fields = [
        "name", "serial_number", "firmware_version",
        "recommended_firmware_version", "physical_port", "product_id",
        "product_line", "usb_type_descriptor", "asic_serial_number",
    ]
    info = {}
    for f in fields:
        key = getattr(rs.camera_info, f, None)
        if key is not None and dev.supports(key):
            info[f] = dev.get_info(key)
            print(f"  {f:<30} {info[f]}")

    ok = True
    usb = info.get("usb_type_descriptor", "")
    if usb and not usb.startswith("3"):
        print(f"  WARN: USB link is {usb}; D435i needs USB 3.x for full "
              f"resolution/framerate")
        ok = False

    fw = info.get("firmware_version")
    rec = info.get("recommended_firmware_version")
    if fw and rec and fw != rec:
        print(f"  NOTE: firmware {fw} differs from recommended {rec} "
              f"(update with rs-fw-update)")

    print("\n  sensors:")
    for s in dev.query_sensors():
        sname = s.get_info(rs.camera_info.name)
        n = len(s.get_stream_profiles())
        print(f"    - {sname} ({n} stream profiles)")

    return dev, ok


def check_streaming(width, height, fps, nframes, color_w, color_h):
    """Stream depth+color, measure real FPS and drops, save artifacts."""
    hr("2. DEPTH + COLOR STREAMING")
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    cfg.enable_stream(rs.stream.color, color_w, color_h, rs.format.bgr8, fps)

    print(f"  requesting depth {width}x{height}@{fps} + "
          f"color {color_w}x{color_h}@{fps}")
    try:
        profile = pipe.start(cfg)
    except RuntimeError as e:
        print(f"  FAIL: could not start pipeline: {e}")
        return False

    try:
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print(f"  depth scale: {depth_scale} m/unit")

        # Warm up: auto-exposure needs a few frames to settle.
        for _ in range(30):
            pipe.wait_for_frames(timeout_ms=5000)

        first_num = last_num = None
        received = 0
        depth_frame = color_frame = None
        t0 = time.time()
        for _ in range(nframes):
            frames = pipe.wait_for_frames(timeout_ms=5000)
            d = frames.get_depth_frame()
            c = frames.get_color_frame()
            if not d or not c:
                continue
            received += 1
            n = d.get_frame_number()
            if first_num is None:
                first_num = n
            last_num = n
            depth_frame, color_frame = d, c
        elapsed = time.time() - t0

        measured = received / elapsed if elapsed > 0 else 0.0
        expected = last_num - first_num + 1 if first_num is not None else 0
        dropped = max(0, expected - received)

        print(f"  received {received}/{nframes} frame pairs in {elapsed:.2f}s")
        print(f"  measured FPS: {measured:.1f} (requested {fps})")
        print(f"  dropped frames: {dropped} "
              f"({100.0 * dropped / expected:.1f}%)" if expected else "")

        ok = True
        if measured < fps * 0.85:
            print(f"  WARN: measured FPS is >15% below requested")
            ok = False
        if expected and dropped > expected * 0.05:
            print(f"  WARN: >5% frame drops")
            ok = False

        # Depth quality on the final frame.
        depth = np.asanyarray(depth_frame.get_data())
        valid = depth > 0
        pct_valid = 100.0 * valid.sum() / depth.size
        print(f"\n  depth validity: {pct_valid:.1f}% of pixels have a return")

        if valid.any():
            dm = depth[valid] * depth_scale
            print(f"  depth range: {dm.min():.3f} - {dm.max():.3f} m "
                  f"(mean {dm.mean():.3f} m)")
            h, w = depth.shape
            cy, cx = h // 2, w // 2
            patch = depth[cy - h // 10:cy + h // 10, cx - w // 10:cx + w // 10]
            pv = patch[patch > 0]
            if pv.size:
                print(f"  center patch distance: "
                      f"{np.median(pv) * depth_scale:.3f} m (median)")
            else:
                print("  center patch: no valid depth (too close/far or "
                      "non-reflective surface)")

        if pct_valid < 20.0:
            print("  WARN: very low depth validity - check lens cover, "
                  "lighting, and that the scene is 0.3-3 m away")
            ok = False

        # Artifacts.
        os.makedirs(OUT_DIR, exist_ok=True)
        import cv2
        color = np.asanyarray(color_frame.get_data())
        colorized = np.asanyarray(
            rs.colorizer().colorize(depth_frame).get_data())
        cv2.imwrite(os.path.join(OUT_DIR, "color.png"), color)
        cv2.imwrite(os.path.join(OUT_DIR, "depth_colorized.png"), colorized)
        np.save(os.path.join(OUT_DIR, "depth_raw.npy"), depth)
        print(f"\n  wrote color.png, depth_colorized.png, depth_raw.npy "
              f"-> {OUT_DIR}")
        return ok
    finally:
        pipe.stop()


def check_imu(duration):
    """Stream accel+gyro; a still camera should read ~9.8 m/s2 total accel."""
    hr("3. IMU (ACCEL + GYRO)")
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.accel)
    cfg.enable_stream(rs.stream.gyro)

    try:
        pipe.start(cfg)
    except RuntimeError as e:
        print(f"  FAIL: could not start IMU pipeline: {e}")
        print("  (a D435 without the 'i' has no IMU)")
        return False

    try:
        accel_n = gyro_n = 0
        accels, gyros = [], []
        t0 = time.time()
        while time.time() - t0 < duration:
            frames = pipe.wait_for_frames(timeout_ms=5000)
            for f in frames:
                mf = f.as_motion_frame()
                if not mf:
                    continue
                d = mf.get_motion_data()
                if mf.get_profile().stream_type() == rs.stream.accel:
                    accel_n += 1
                    accels.append((d.x, d.y, d.z))
                else:
                    gyro_n += 1
                    gyros.append((d.x, d.y, d.z))
        elapsed = time.time() - t0

        print(f"  accel: {accel_n} samples ({accel_n / elapsed:.0f} Hz)")
        print(f"  gyro:  {gyro_n} samples ({gyro_n / elapsed:.0f} Hz)")

        ok = accel_n > 0 and gyro_n > 0
        if not ok:
            print("  FAIL: IMU produced no samples")
            return False

        a = np.array(accels)
        g = np.array(gyros)
        mag = np.linalg.norm(a, axis=1).mean()
        print(f"  mean accel vector: [{a[:,0].mean():+.2f} "
              f"{a[:,1].mean():+.2f} {a[:,2].mean():+.2f}] m/s^2")
        print(f"  mean accel magnitude: {mag:.2f} m/s^2 (gravity ~9.81)")
        print(f"  mean gyro: [{g[:,0].mean():+.3f} {g[:,1].mean():+.3f} "
              f"{g[:,2].mean():+.3f}] rad/s")

        if not (8.5 < mag < 11.0):
            print("  WARN: accel magnitude far from 9.81 - camera may be "
                  "moving, or the IMU needs calibration")
            ok = False
        return ok
    finally:
        pipe.stop()


def main():
    p = argparse.ArgumentParser(description="D435i hardware smoke test")
    p.add_argument("--width", type=int, default=848)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--color-width", type=int, default=1280)
    p.add_argument("--color-height", type=int, default=720)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--frames", type=int, default=150,
                   help="frames to measure after warm-up")
    p.add_argument("--imu-seconds", type=float, default=2.0)
    p.add_argument("--skip-imu", action="store_true")
    args = p.parse_args()

    print(f"pyrealsense2 {rs.__version__}")

    results = {}
    dev, results["device"] = check_device()
    if dev is None:
        print("\nRESULT: FAIL - no device")
        return 1

    results["streaming"] = check_streaming(
        args.width, args.height, args.fps, args.frames,
        args.color_width, args.color_height)

    if args.skip_imu:
        print("\n(IMU check skipped)")
    else:
        results["imu"] = check_imu(args.imu_seconds)

    hr("SUMMARY")
    for name, ok in results.items():
        print(f"  {name:<12} {'PASS' if ok else 'WARN/FAIL'}")
    all_ok = all(results.values())
    print(f"\nRESULT: {'ALL CHECKS PASSED' if all_ok else 'SEE WARNINGS ABOVE'}")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
