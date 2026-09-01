#!/usr/bin/env python3
"""R1 gate probe: does this Blackwell GPU actually run compute on this stack?

Risk R1 in CLAUDE.md is that FoundationPose TensorRT engines may not build for
Blackwell sm_120. This script checks the layer beneath that -- whether PyTorch
executes real kernels on this device at all -- because if it does not, no
FoundationPose result is meaningful and the fallback ladder starts immediately.

Reports facts, not reassurance: every check prints the measured value, and the
exit status is non-zero if any check fails.
"""

from __future__ import annotations

import argparse
import sys
import time

EXPECTED_CAPABILITY = (12, 0)  # sm_120, Blackwell

PASS = "PASS"
FAIL = "FAIL"
WARN = "WARN"

_results: list[tuple[str, str, str]] = []


def record(status: str, name: str, detail: str = "") -> None:
    _results.append((status, name, detail))
    print(f"  [{status}] {name}" + (f" -- {detail}" if detail else ""))


def section(title: str) -> None:
    print(f"\n{title}")
    print("-" * len(title))


def check_versions() -> "object":
    section("Environment")
    try:
        import torch
    except ImportError as exc:
        record(FAIL, "import torch", str(exc))
        return None

    record(PASS, "torch", torch.__version__)
    record(PASS, "torch built against CUDA", str(torch.version.cuda))
    print(f"  ... cuDNN: {torch.backends.cudnn.version()}")
    return torch


def check_device(torch) -> bool:
    section("Device")
    if not torch.cuda.is_available():
        record(FAIL, "torch.cuda.is_available()", "no CUDA device visible to torch")
        return False
    record(PASS, "torch.cuda.is_available()", f"{torch.cuda.device_count()} device(s)")

    name = torch.cuda.get_device_name(0)
    capability = torch.cuda.get_device_capability(0)
    record(PASS, "device 0", f"{name}, compute capability {capability[0]}.{capability[1]}")

    if capability != EXPECTED_CAPABILITY:
        record(
            WARN,
            "compute capability",
            f"expected {EXPECTED_CAPABILITY[0]}.{EXPECTED_CAPABILITY[1]} (sm_120), "
            f"got {capability[0]}.{capability[1]}",
        )

    # The decisive one. A wheel with no sm_120 kernels may still "work" via PTX
    # JIT, which is slow and silently different from a native build -- so report
    # whether this arch was actually compiled in.
    arch_list = torch.cuda.get_arch_list()
    target = f"sm_{capability[0]}{capability[1]}"
    print(f"  ... compiled arch list: {' '.join(arch_list)}")
    if target in arch_list:
        record(PASS, f"native {target} kernels", "present in this wheel")
    else:
        record(
            WARN,
            f"native {target} kernels",
            f"absent; this device would run via PTX JIT from a lower arch. "
            f"Expect slower first-run compilation and verify performance claims.",
        )
    return True


def _timed_matmul(torch, a, b, iters: int) -> float:
    """Seconds per matmul, warmed up and averaged.

    Timing a single cold call measures CUDA context setup and kernel load, not
    arithmetic -- it under-reports by an order of magnitude and can even make
    fp16 look slower than fp32. Warm up first, then average.
    """
    for _ in range(3):
        _ = a @ b
    torch.cuda.synchronize()

    start = time.perf_counter()
    for _ in range(iters):
        _ = a @ b
    torch.cuda.synchronize()
    return (time.perf_counter() - start) / iters


def check_compute(torch, size: int, iters: int) -> bool:
    section("Compute")
    ok = True
    try:
        a = torch.randn(size, size, device="cuda")
        b = torch.randn(size, size, device="cuda")

        elapsed = _timed_matmul(torch, a, b, iters)
        # 2*n^3 flops for an n-by-n matmul.
        tflops = (2.0 * size**3) / elapsed / 1e12
        fp32_tflops = tflops
        record(PASS, f"fp32 matmul {size}x{size}",
               f"{elapsed * 1e3:.1f} ms, {tflops:.1f} TFLOP/s (mean of {iters})")

        c = a @ b

        # Correctness, not just "it did not crash": a GPU running the wrong
        # kernels or JITing badly can still return a tensor of the right shape.
        reference = (a.cpu() @ b.cpu())
        max_err = (c.cpu() - reference).abs().max().item()
        if max_err < 1e-2:
            record(PASS, "matmul matches CPU reference", f"max abs error {max_err:.3e}")
        else:
            record(FAIL, "matmul matches CPU reference", f"max abs error {max_err:.3e}")
            ok = False
    except Exception as exc:  # noqa: BLE001 -- report whatever the stack raises
        record(FAIL, "fp32 matmul", f"{type(exc).__name__}: {exc}")
        return False

    try:
        h = torch.randn(size, size, device="cuda", dtype=torch.float16)
        elapsed = _timed_matmul(torch, h, h, iters)
        tflops = (2.0 * size**3) / elapsed / 1e12
        record(PASS, "fp16 matmul (tensor cores)",
               f"{elapsed * 1e3:.1f} ms, {tflops:.1f} TFLOP/s (mean of {iters})")
        # fp16 slower than fp32 means the tensor cores are not being used --
        # usually a sign the kernels are running via PTX JIT from a lower arch.
        if tflops < fp32_tflops:
            record(WARN, "fp16 faster than fp32",
                   f"fp16 {tflops:.1f} < fp32 {fp32_tflops:.1f} TFLOP/s; tensor cores "
                   f"appear unused")
    except Exception as exc:  # noqa: BLE001
        record(FAIL, "fp16 matmul", f"{type(exc).__name__}: {exc}")
        ok = False

    return ok


def check_conv(torch) -> bool:
    """cuDNN convolution -- this is the path YOLOv8 inference actually takes."""
    section("cuDNN convolution")
    try:
        import torch.nn as nn

        net = nn.Sequential(
            nn.Conv2d(3, 32, 3, padding=1),
            nn.BatchNorm2d(32),
            nn.SiLU(),
            nn.Conv2d(32, 64, 3, stride=2, padding=1),
            nn.SiLU(),
        ).cuda().eval()

        x = torch.randn(1, 3, 640, 640, device="cuda")
        with torch.no_grad():
            for _ in range(3):  # warm up autotuning
                y = net(x)
            torch.cuda.synchronize()
            start = time.perf_counter()
            for _ in range(20):
                y = net(x)
            torch.cuda.synchronize()
            elapsed = (time.perf_counter() - start) / 20

        record(PASS, "conv stack forward", f"{elapsed * 1e3:.2f} ms/iter, output {tuple(y.shape)}")
        if not bool(torch.isfinite(y).all()):
            record(FAIL, "conv output finite", "output contains NaN or Inf")
            return False
        record(PASS, "conv output finite", "no NaN or Inf")
        return True
    except Exception as exc:  # noqa: BLE001
        record(FAIL, "cuDNN convolution", f"{type(exc).__name__}: {exc}")
        return False


def check_memory(torch) -> bool:
    section("Memory")
    try:
        free, total = torch.cuda.mem_get_info()
        record(PASS, "device memory", f"{free / 2**30:.1f} GiB free of {total / 2**30:.1f} GiB")
        return True
    except Exception as exc:  # noqa: BLE001
        record(FAIL, "mem_get_info", f"{type(exc).__name__}: {exc}")
        return False


def _export_probe_onnx(torch, path: str) -> "tuple":
    """Export a small conv net to ONNX and return (input, torch reference output)."""
    import torch.nn as nn

    net = nn.Sequential(
        nn.Conv2d(3, 16, 3, padding=1),
        nn.ReLU(),
        nn.Conv2d(16, 8, 3, stride=2, padding=1),
    ).eval()
    x = torch.randn(1, 3, 128, 128)
    with torch.no_grad():
        reference = net(x)
    torch.onnx.export(net, (x,), path, input_names=["input"], output_names=["output"],
                      dynamo=False)
    return x.numpy(), reference.numpy()


def _try_provider(ort, model_path: str, x, reference, provider: str, label: str) -> None:
    """Run the model on one execution provider and verify it really ran there.

    ORT lists every provider it was COMPILED with, then silently falls back to
    CPU when the runtime libraries are missing. Only session.get_providers()
    after construction says what will actually execute.
    """
    import numpy as np

    try:
        session = ort.InferenceSession(model_path, providers=[provider, "CPUExecutionProvider"])
    except Exception as exc:  # noqa: BLE001
        record(WARN, f"{label} session", f"{type(exc).__name__}: {exc}")
        return

    active = session.get_providers()
    if provider not in active:
        record(WARN, f"{label} active", f"fell back to {active[0]}; {provider} did not load")
        return

    try:
        out = session.run(["output"], {"input": x})[0]
    except Exception as exc:  # noqa: BLE001
        record(WARN, f"{label} inference", f"{type(exc).__name__}: {exc}")
        return

    max_err = float(np.abs(out - reference).max())
    if max_err < 1e-3:
        record(PASS, f"{label} inference", f"matches torch, max abs error {max_err:.3e}")
    else:
        record(FAIL, f"{label} inference", f"diverges from torch, max abs error {max_err:.3e}")


def check_onnxruntime(torch) -> None:
    section("ONNX Runtime (R1 fallback path)")
    try:
        import onnxruntime as ort
    except ImportError:
        record(WARN, "import onnxruntime", "not installed; the R1 fallback is unverified")
        return

    record(PASS, "onnxruntime", ort.__version__)
    print(f"  ... compiled providers: {', '.join(ort.get_available_providers())}")

    if torch is None:
        record(WARN, "ONNX export", "torch unavailable, cannot build a probe model")
        return

    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        model_path = f"{tmp}/probe.onnx"
        try:
            x, reference = _export_probe_onnx(torch, model_path)
        except Exception as exc:  # noqa: BLE001
            record(WARN, "ONNX export", f"{type(exc).__name__}: {exc}")
            return
        record(PASS, "ONNX export", "probe conv net exported")

        _try_provider(ort, model_path, x, reference, "CUDAExecutionProvider", "CUDA EP")
        # The TensorRT EP is R1's PRIMARY path -- if it executes here, the
        # fallback ladder in CLAUDE.md may not be needed at all.
        _try_provider(ort, model_path, x, reference, "TensorrtExecutionProvider", "TensorRT EP")


def check_tensorrt() -> None:
    section("TensorRT (R1 primary path)")
    try:
        import tensorrt as trt
    except ImportError:
        record(WARN, "import tensorrt", "not installed; the R1 primary path is unverified")
        return
    record(PASS, "tensorrt", trt.__version__)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--size", type=int, default=4096, help="matmul dimension")
    parser.add_argument("--iters", type=int, default=20,
                        help="timed iterations per benchmark, after warm-up")
    args = parser.parse_args()

    print("R1 gate probe -- Blackwell sm_120 compute check")
    print("=" * 47)

    torch = check_versions()
    if torch is None:
        print("\nRESULT: FAIL -- torch is not importable, nothing else can be checked.")
        return 1

    if check_device(torch):
        check_memory(torch)
        check_compute(torch, args.size, args.iters)
        check_conv(torch)
    check_onnxruntime(torch)
    check_tensorrt()

    section("Summary")
    failures = [r for r in _results if r[0] == FAIL]
    warnings = [r for r in _results if r[0] == WARN]
    print(f"  {len(_results) - len(failures) - len(warnings)} passed, "
          f"{len(warnings)} warned, {len(failures)} failed")
    for status, name, detail in failures + warnings:
        print(f"  [{status}] {name} -- {detail}")

    if failures:
        print("\nRESULT: FAIL -- R1 is not cleared. See the failures above.")
        return 1
    if warnings:
        print("\nRESULT: PASS WITH WARNINGS -- compute works; read the warnings before "
              "quoting any performance number.")
        return 2
    print("\nRESULT: PASS -- this GPU runs real kernels on this stack.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
