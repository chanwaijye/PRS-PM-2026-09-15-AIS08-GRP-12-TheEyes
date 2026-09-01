# RealSense D435i hardware test

Smoke test for the Intel RealSense D435i: device discovery, depth+color
streaming throughput, and IMU.

## Run

```bash
python3 test_d435i.py                 # defaults: depth 848x480@30, color 1280x720@30
python3 test_d435i.py --frames 300    # longer throughput sample
python3 test_d435i.py --skip-imu
python3 test_d435i.py --width 1280 --height 720 --fps 15
```

Exit status is non-zero if any check reports a warning or failure.

Artifacts land in `out/`: `color.png`, `depth_colorized.png`, and
`depth_raw.npy` (uint16; multiply by the printed depth scale for metres).

## Setup on this machine

librealsense v2.58.3 is built from source at `~/src/librealsense` and installed
to `/usr/local`; `pyrealsense2` is importable from system Python 3.12 with no
venv. The build uses the **RSUSB userspace backend** (`FORCE_RSUSB_BACKEND=ON`)
because this box runs kernel 6.17 with Secure Boot on, which rules out the
kernel-patch route.

## Other useful tools

```bash
rs-enumerate-devices -s     # one-line device summary
rs-enumerate-devices        # full profile / intrinsics dump
realsense-viewer            # GUI (needs a desktop session)
rs-fw-update -l             # list firmware
```

## Troubleshooting

- **No device found** — confirm `lsusb | grep 8086:0b3a`, and that
  `/dev/bus/usb/.../...` for the camera is mode `0666` group `plugdev`. If not,
  reinstall the udev rules from `~/src/librealsense/config/99-realsense-libusb.rules`.
- **Low depth validity** — aim at a textured surface 0.3-3 m away in decent
  light. Blank walls and glass return little.
- **Low FPS / drops** — must be on a USB 3.x port; the test prints the
  negotiated link type (should read `3.2`).
