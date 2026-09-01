#!/usr/bin/env python3
"""R1 primary-path gate: can TensorRT build and run an engine for sm_120?

This is the check Risk R1 in CLAUDE.md is actually about. FoundationPose ships
TensorRT engines; if the builder has no tactics for Blackwell sm_120, engines
cannot be built from ONNX either and the fallback ladder (ONNX Runtime CUDA EP,
already proven working -- see gpu_test/README.md) becomes the plan.

Builds a small conv network in fp32 and fp16, runs both, and compares against a
PyTorch reference. Exit status is non-zero if any check fails.
"""

from __future__ import annotations

import argparse
import sys
import time

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


def build_reference(torch, path_fp32: str, path_fp16: str):
    """Export one conv net twice, fp32 and fp16.

    TensorRT 11 removed the FP16 builder flag: precision is a property of the
    network, not a build hint, so a half-precision engine needs a half-precision
    ONNX built as a strongly-typed network. Returns (x, reference).
    """
    import warnings

    import torch.nn as nn

    torch.manual_seed(20260820)
    net = nn.Sequential(
        nn.Conv2d(3, 32, 3, padding=1),
        nn.BatchNorm2d(32),
        nn.ReLU(),
        nn.Conv2d(32, 64, 3, stride=2, padding=1),
        nn.ReLU(),
        nn.Conv2d(64, 16, 1),
    ).eval()
    x = torch.randn(1, 3, 224, 224)
    with torch.no_grad():
        reference = net(x)

    # dynamo=False keeps the TorchScript exporter, whose ONNX the TensorRT
    # parser handles predictably. It is deprecated upstream; revisit if a future
    # torch drops it, and re-verify the engine builds before trusting the switch.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        torch.onnx.export(net, (x,), path_fp32, input_names=["input"],
                          output_names=["output"], dynamo=False)
        torch.onnx.export(net.half(), (x.half(),), path_fp16, input_names=["input"],
                          output_names=["output"], dynamo=False)
    return x, reference


def build_engine(trt, onnx_path: str, strongly_typed: bool, workspace_gib: int):
    """Return (serialized_engine, build_seconds) or (None, reason)."""
    logger = trt.Logger(trt.Logger.ERROR)
    builder = trt.Builder(logger)

    flags = 0
    # TensorRT 10+ makes explicit batch the only mode and removed the flag.
    if hasattr(trt.NetworkDefinitionCreationFlag, "EXPLICIT_BATCH"):
        flags |= 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    if strongly_typed:
        # TensorRT 11 removed BuilderFlag.FP16. Precision now comes from the
        # network's own types, so a half-precision engine is a strongly-typed
        # network parsed from a half-precision ONNX.
        flags |= 1 << int(trt.NetworkDefinitionCreationFlag.STRONGLY_TYPED)
    network = builder.create_network(flags)

    parser = trt.OnnxParser(network, logger)
    with open(onnx_path, "rb") as f:
        if not parser.parse(f.read()):
            errors = "; ".join(str(parser.get_error(i)) for i in range(parser.num_errors))
            return None, f"ONNX parse failed: {errors}"

    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, workspace_gib << 30)

    start = time.perf_counter()
    serialized = builder.build_serialized_network(network, config)
    elapsed = time.perf_counter() - start
    if serialized is None:
        # This is the R1 failure mode: the builder ran but produced no engine,
        # which is what "no tactics for this arch" looks like from the outside.
        return None, "builder returned no engine (no viable tactics for this device?)"
    return serialized, elapsed


def run_engine(trt, torch, serialized, x, iters: int):
    """Execute the engine on `x` and return (output tensor, seconds per iter)."""
    logger = trt.Logger(trt.Logger.ERROR)
    runtime = trt.Runtime(logger)
    engine = runtime.deserialize_cuda_engine(serialized)
    if engine is None:
        raise RuntimeError("deserialize_cuda_engine returned None")
    context = engine.create_execution_context()

    # Bind torch tensors directly as device buffers -- avoids a pycuda dependency
    # and keeps the comparison against the torch reference in one memory space.
    tensors = {}
    device_in = x.cuda().contiguous()
    for i in range(engine.num_io_tensors):
        name = engine.get_tensor_name(i)
        mode = engine.get_tensor_mode(name)
        if mode == trt.TensorIOMode.INPUT:
            context.set_input_shape(name, tuple(device_in.shape))
            tensors[name] = device_in
        else:
            import numpy as np

            shape = tuple(context.get_tensor_shape(name))
            np_dtype = trt.nptype(engine.get_tensor_dtype(name))
            torch_dtype = torch.from_numpy(np.empty(0, dtype=np_dtype)).dtype
            tensors[name] = torch.empty(shape, dtype=torch_dtype, device="cuda")
    for name, tensor in tensors.items():
        context.set_tensor_address(name, tensor.data_ptr())

    stream = torch.cuda.Stream()
    with torch.cuda.stream(stream):
        for _ in range(3):
            context.execute_async_v3(stream.cuda_stream)
    stream.synchronize()

    start = time.perf_counter()
    with torch.cuda.stream(stream):
        for _ in range(iters):
            context.execute_async_v3(stream.cuda_stream)
    stream.synchronize()
    elapsed = (time.perf_counter() - start) / iters

    output_name = next(
        engine.get_tensor_name(i)
        for i in range(engine.num_io_tensors)
        if engine.get_tensor_mode(engine.get_tensor_name(i)) == trt.TensorIOMode.OUTPUT
    )
    return tensors[output_name], elapsed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iters", type=int, default=50)
    parser.add_argument("--workspace-gib", type=int, default=2)
    args = parser.parse_args()

    print("R1 primary-path gate -- TensorRT engine build for sm_120")
    print("=" * 55)

    section("Environment")
    try:
        import tensorrt as trt
    except ImportError as exc:
        record(FAIL, "import tensorrt", str(exc))
        print("\nRESULT: FAIL -- TensorRT not installed.")
        return 1
    record(PASS, "tensorrt", trt.__version__)

    try:
        import torch
    except ImportError as exc:
        record(FAIL, "import torch", str(exc))
        return 1
    record(PASS, "torch", f"{torch.__version__} (reference + device buffers)")

    if not torch.cuda.is_available():
        record(FAIL, "cuda", "no device visible to torch")
        return 1
    capability = torch.cuda.get_device_capability(0)
    record(PASS, "device", f"{torch.cuda.get_device_name(0)}, sm_{capability[0]}{capability[1]}")

    # TensorRT 11 removed Builder.platform_has_fast_fp16/int8 along with the
    # FP16 builder flag; there is no capability query to report here any more.
    record(PASS, "builder API", "strongly-typed networks (TensorRT 11 style)"
           if hasattr(trt.NetworkDefinitionCreationFlag, "STRONGLY_TYPED")
           else "builder-flag precision (TensorRT 10 style)")

    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        path_fp32 = f"{tmp}/probe_fp32.onnx"
        path_fp16 = f"{tmp}/probe_fp16.onnx"
        section("Reference model")
        try:
            x, reference = build_reference(torch, path_fp32, path_fp16)
        except Exception as exc:  # noqa: BLE001
            record(FAIL, "ONNX export", f"{type(exc).__name__}: {exc}")
            return 1
        record(PASS, "ONNX export", f"conv net, input {tuple(x.shape)}, fp32 + fp16")

        passes = (("fp32", path_fp32, False, 1e-3), ("fp16", path_fp16, True, 5e-2))
        for label, onnx_path, strongly_typed, tolerance in passes:
            section(f"Engine build -- {label}")
            try:
                serialized, info = build_engine(trt, onnx_path, strongly_typed,
                                                args.workspace_gib)
            except Exception as exc:  # noqa: BLE001
                record(FAIL, f"{label} build", f"{type(exc).__name__}: {exc}")
                continue
            if serialized is None:
                record(FAIL, f"{label} build", str(info))
                continue
            record(PASS, f"{label} build",
                   f"{info:.1f} s, {serialized.nbytes / 1024:.0f} KiB engine")

            try:
                model_input = x.half() if label == "fp16" else x
                out, latency = run_engine(trt, torch, serialized, model_input, args.iters)
            except Exception as exc:  # noqa: BLE001
                record(FAIL, f"{label} inference", f"{type(exc).__name__}: {exc}")
                continue
            record(PASS, f"{label} inference", f"{latency * 1e3:.3f} ms/iter")

            max_err = (out.float().cpu() - reference).abs().max().item()
            if max_err < tolerance:
                record(PASS, f"{label} matches torch", f"max abs error {max_err:.3e}")
            else:
                record(FAIL, f"{label} matches torch",
                       f"max abs error {max_err:.3e} exceeds {tolerance:.0e}")

    section("Summary")
    failures = [r for r in _results if r[0] == FAIL]
    warnings = [r for r in _results if r[0] == WARN]
    print(f"  {len(_results) - len(failures) - len(warnings)} passed, "
          f"{len(warnings)} warned, {len(failures)} failed")
    for status, name, detail in failures + warnings:
        print(f"  [{status}] {name} -- {detail}")

    if failures:
        print("\nRESULT: FAIL -- R1's primary path is NOT clear. Fall back to the "
              "ONNX Runtime CUDA EP, which gpu_test/test_sm120.py proves working.")
        return 1
    print("\nRESULT: PASS -- TensorRT builds and runs engines on this sm_120 device.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
