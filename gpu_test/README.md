# R1 gate probe — Blackwell sm_120

Risk R1 in `CLAUDE.md`: *FoundationPose TensorRT engines may not build for
Blackwell sm_120.* This probe checks the layer beneath that — whether this GPU
runs real kernels on this software stack at all — because if it does not, no
FoundationPose result is meaningful and the fallback ladder starts immediately.

## Run

```bash
~/venvs/the-eyes-r1/bin/python gpu_test/test_sm120.py
~/venvs/the-eyes-r1/bin/python gpu_test/test_sm120.py --size 8192 --iters 50
```

Exit status: `0` all passed, `2` passed with warnings, `1` something failed.

## Environment

An isolated venv at `~/venvs/the-eyes-r1` (`include-system-site-packages =
false`) — deliberately **not** the system Python, which carries the ROS 2 Jazzy
and `pyrealsense2` installs and must not be disturbed by a multi-gigabyte ML
stack.

| Package | Version | Why this one |
|---|---|---|
| torch | 2.13.0+cu130 | `cu130` wheels match the driver's reported max CUDA 13.0 exactly. Wheels bundle their own CUDA runtime, so the system toolkit at 13.3 is not what matters — the driver is. |
| onnxruntime-gpu | 1.29.0 | R1 fallback path |
| onnx | current | required for the export step of the ONNX check |

## Results as of 2026-08-20

| Check | Result |
|---|---|
| Device | RTX 5070 Ti, compute capability **12.0 (sm_120)**, 15.5 GiB |
| Native sm_120 kernels in wheel | **Present** — arch list `sm_75 sm_80 sm_86 sm_90 sm_100 sm_120` |
| fp32 matmul 4096² | 4.2 ms, **32.4 TFLOP/s** |
| fp16 matmul (tensor cores) | 1.4 ms, **96.7 TFLOP/s** |
| Matmul vs. CPU reference | max abs error 6.3e-04 |
| cuDNN conv stack @ 640×640 | 1.00 ms/iter, finite output |
| ONNX Runtime **CUDA EP** | **Works** — matches torch, max abs error 3.9e-07 |
| ONNX Runtime **TensorRT EP** | Did not load before TensorRT was installed (see note below) |
| TensorRT python module | **11.2.1.2** (`tensorrt-cu13`) |

### TensorRT engine build — `test_tensorrt_sm120.py`

R1's primary path. Builds a conv network into a TensorRT engine and runs it,
fp32 and fp16, comparing against a PyTorch reference.

| Check | Result |
|---|---|
| TensorRT | 11.2.1.2 |
| Builder API | strongly-typed networks (TensorRT 11 style) |
| fp32 engine build | **1.5 s**, 333 KiB |
| fp32 inference | 0.032 ms/iter, max abs error vs torch **2.7e-04** |
| fp16 engine build | **1.4 s**, 232 KiB |
| fp16 inference | 0.021 ms/iter, max abs error vs torch **6.0e-04** |

**R1's engine-build question is answered: TensorRT builds and runs engines on
this sm_120 device, in both precisions, numerically correct.** The remaining R1
risk is not the architecture — it is whether FoundationPose's own model graphs
and its pinned dependency stack work on this much newer CUDA/torch/TensorRT
combination.

### FoundationPose end-to-end

Not a script here — see `docs/R1_STATUS.md` for the full record. Summary:
`run_demo.py` completes 737/737 frames on the mustard0 sequence with valid,
continuous SE(3) poses; tracking runs at **17.2 ms/frame (58 FPS)**. Every
compiled dependency (`pytorch3d`, `nvdiffrast`, `mycpp`) builds against CUDA
13.3 / torch 2.13.

One trap worth repeating here: `mycpp` built and imported cleanly against the
system `pybind11-dev` 2.11.1, then **segfaulted** the first time a NumPy array
crossed the boundary. pybind11 gained NumPy 2 support only in 2.12, and the venv
has NumPy 2.5.2. Rebuild against the venv's pybind11 (`-Dpybind11_DIR=...`).

### TensorRT 11 API note

TensorRT 11 removed `BuilderFlag.FP16` and `Builder.platform_has_fast_fp16`.
Precision is now a property of the network rather than a build hint: a
half-precision engine is a **strongly-typed** network parsed from a
half-precision ONNX. Code written against TensorRT 8/10 examples — which is most
of what is published, FoundationPose's tooling included — will not run unchanged.

## Two traps this probe is built around

**Cold-start timing.** A single untimed matmul measures CUDA context setup and
kernel load, not arithmetic. The first version of this script reported 3.9
TFLOP/s fp32 and — the tell — fp16 *slower* than fp32. With warm-up and
averaging the same GPU reports 32.4 and 96.7. Any benchmark here warms up first,
and the script warns if fp16 comes out slower than fp32, since that is the
signature of tensor cores going unused.

**Silent CPU fallback.** `onnxruntime.get_available_providers()` lists every
provider ORT was *compiled* with, regardless of whether its runtime libraries
exist. Only `session.get_providers()` after construction says what will actually
execute. The TensorRT EP result above is exactly this case: listed as available,
falls back to CPU on session creation. A check that trusted the first list would
have reported R1's primary path as working.
