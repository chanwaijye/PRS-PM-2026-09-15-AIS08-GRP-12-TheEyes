# R1 status — FoundationPose on Blackwell sm_120

Risk R1 as recorded in `CLAUDE.md`:

> FoundationPose TensorRT engines may not build for Blackwell sm_120.
> Fallbacks: rebuild engines from ONNX; ONNX Runtime CUDA EP; cloud GPU for
> engine work.

Verified 2026-08-20 on this machine. Reproduce with the scripts in `gpu_test/`.

---

## Headline: the risk as written rests on a false premise

**Upstream FoundationPose does not use TensorRT.** A case-insensitive search of
the entire checkout for `tensorrt`, `nvinfer`, `.engine`, and `trtexec` returns
zero matches. The refiner and scorer are PyTorch checkpoints (`model_best.pth`)
loaded with `torch.load` in `learning/training/predict_pose_refine.py` and
`predict_score.py`.

TensorRT enters the picture only in *downstream* packagings — notably NVIDIA's
`isaac_ros_foundationpose`, which wraps the models in TensorRT/Triton. That is
almost certainly where the premise came from. It is not what `pip install`-ing
this repository gives you.

So the question R1 should have been asking is **"does the PyTorch stack run on
sm_120?"** — and the answer is yes.

---

## What was verified

### Hardware and framework — `gpu_test/test_sm120.py`

| Check | Result |
|---|---|
| Compute capability | **12.0 (sm_120)** confirmed by the device itself |
| Native sm_120 kernels in the torch wheel | **Present** (`sm_75 … sm_100 sm_120`), not PTX-JIT'd |
| fp32 / fp16 matmul | 32.4 / 96.7 TFLOP/s, tensor cores active |
| Numerical correctness | matches CPU reference, 6.3e-04 |
| cuDNN conv @ 640×640 | 1.00 ms/iter |
| ONNX Runtime CUDA EP | **Works**, matches torch to 3.9e-07 |

### TensorRT — `gpu_test/test_tensorrt_sm120.py`

Tested anyway, because it is R1's nominal primary path and because a downstream
Isaac ROS deployment would need it.

| Check | Result |
|---|---|
| TensorRT | 11.2.1.2 (`tensorrt-cu13`) |
| fp32 engine build → run | 1.5 s build, 0.032 ms/iter, error vs torch **2.7e-04** |
| fp16 engine build → run | 1.4 s build, 0.021 ms/iter, error vs torch **6.0e-04** |

**TensorRT builds and runs engines on sm_120 in both precisions.** Whatever else
happens, the architecture is not the obstacle.

### FoundationPose — full end-to-end run

Cloned to `~/src/FoundationPose`. Built against Python 3.12, torch 2.13.0+cu130,
CUDA 13.3 nvcc — a much newer combination than the repo's documented cu124.

| Component | Result |
|---|---|
| `mycpp` C++ extension | Builds and runs — **after a pybind11 fix, see below** |
| `pytorch3d` 0.7.9 | **Builds from source against CUDA 13.3**; `knn_points` matches a brute-force reference exactly (0.0 error) |
| `nvdiffrast` 0.4.0 | JIT CUDA rasterizer runs on sm_120 — verified by triangle coverage |
| `torchvision` 0.28.0+cu130 | Needed by `estimater.py`; **not listed in `requirements.txt`** |
| `warp` 1.16.0 | Initialises, reports `sm_120` explicitly |
| Weights (refiner + scorer) | Downloaded via `gdown`, 66 MB + 182 MB |
| `check_env.py` | **All checks pass** |

**`run_demo.py` on the mustard0 sequence: 737/737 frames, exit 0.**

Output poses validated, not just counted:

| Property | Measured |
|---|---|
| Valid SO(3) | max ‖RRᵀ−I‖ = 2.2e-06, det(R) ∈ [0.999999, 1.000001] |
| Depth range | 0.519 – 0.804 m (plausible for a handheld bottle) |
| Trajectory extent | 0.457 × 0.276 × 0.285 m |
| Per-frame translation | median 0.27 mm, p95 11.5 mm, max 17.8 mm |
| Per-frame rotation | median 0.31°, max 3.54° |

Continuous and physically plausible throughout — not a run that merely exited
zero.

### Throughput — and what it means for R4

| Stage | Measured on RTX 5070 Ti |
|---|---|
| `register` (first frame, 5 refine iters) | **1258 ms** |
| `track_one` (2 refine iters, mean of 200, warmed up) | **17.2 ms → 58.1 FPS** |

For **one object**, two views run serially would give ~29 FPS per camera. That
makes R4 ("FoundationPose too slow for dual-view real-time") much less severe
than the risk register assumes.

Read that with care: it is one object on a 1258 ms registration. Cost scales
roughly linearly in tracked objects, so three objects across two views is ~6
`track_one` calls ≈ 103 ms ≈ 9.7 FPS. Detection-gated ROIs and alternate-frame
scheduling are still worth having; they are no longer obviously mandatory.

### The `mycpp` segfault — worth knowing about

`mycpp` built clean and imported fine against the system `pybind11-dev` 2.11.1,
then **segfaulted inside `mycpp.cluster_poses`** during
`FoundationPose.make_rotation_grid()`.

Cause: pybind11 gained NumPy 2 support in **2.12**. The venv has NumPy 2.5.2, so
2.11.1's `py::array_t` handling walks a NumPy 2 array through a NumPy 1 ABI and
crashes. Rebuilding against pybind11 3.1.0 from the venv fixed it:

```bash
cd ~/src/FoundationPose/mycpp && rm -rf build && mkdir build && cd build
cmake .. -DPYTHON_EXECUTABLE=~/venvs/the-eyes-r1/bin/python \
         -Dpybind11_DIR="$(~/venvs/the-eyes-r1/bin/python -c 'import pybind11; print(pybind11.get_cmake_dir())')"
make -j"$(nproc)"
```

The failure mode is nasty precisely because the build and the import both
succeed — nothing is wrong until a NumPy array crosses the boundary.

## Environment

Isolated venv at `~/venvs/the-eyes-r1` (`include-system-site-packages = false`).
Deliberately not the system Python, which carries ROS 2 Jazzy and
`pyrealsense2` and must not be disturbed by a multi-gigabyte ML stack —
verified intact after every install.

Torch `cu130` wheels were chosen to match the driver's reported max CUDA 13.0.
The wheels bundle their own CUDA runtime, so the system toolkit sitting at 13.3
does not matter for torch; it matters only for source builds that invoke `nvcc`,
which is exactly where the remaining `pytorch3d` risk lives.
