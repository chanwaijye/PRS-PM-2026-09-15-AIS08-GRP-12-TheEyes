#!/usr/bin/env python3
"""FoundationPose inference server for the C++ pose_agent.

FoundationPose is a Python library with no C++ implementation, and CLAUDE.md
puts all runtime code in C++17. Rather than embed an interpreter -- which would
put the GIL, NumPy's ABI, and CUDA context ownership inside our runtime process
-- FoundationPose runs here as a model server behind a byte protocol, exactly as
a TensorRT engine would.

Protocol (little-endian u32 lengths, YAML headers):

    request : b"FPR1" | u32 len | header | u32 len | rgb | u32 len | depth
    response: b"FPS1" | u32 len | header

See SystemCode/pose/include/the_eyes/pose/foundationpose_client.hpp.

Run:
    ~/venvs/the-eyes-r1/bin/python foundationpose_server.py \
        --socket /tmp/the_eyes_foundationpose.sock \
        --asset-root /path/to/repo
"""

from __future__ import annotations

import argparse
import logging
import os
import socket
import struct
import sys
import threading
import time
from typing import Dict, Optional, Tuple

import numpy as np
import yaml

REQUEST_MAGIC = b"FPR1"
RESPONSE_MAGIC = b"FPS1"
MAX_HEADER_BYTES = 64 * 1024
# A crop, not a whole frame. Anything larger means the stream desynchronised and
# we are about to allocate from a length that is really pixel data.
MAX_PAYLOAD_BYTES = 64 * 1024 * 1024

log = logging.getLogger("fp_server")


def recv_exactly(conn: socket.socket, size: int) -> Optional[bytes]:
    chunks = []
    remaining = size
    while remaining > 0:
        chunk = conn.recv(min(remaining, 1 << 20))
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def read_block(conn: socket.socket, limit: int) -> Optional[bytes]:
    raw = recv_exactly(conn, 4)
    if raw is None:
        return None
    (length,) = struct.unpack("<I", raw)
    if length > limit:
        raise ValueError(f"block length {length} exceeds limit {limit}")
    if length == 0:
        return b""
    return recv_exactly(conn, length)


def send_response(conn: socket.socket, payload: dict) -> None:
    body = yaml.safe_dump(payload, default_flow_style=True).encode("utf-8")
    conn.sendall(RESPONSE_MAGIC + struct.pack("<I", len(body)) + body)


def normalized_hypothesis_score(scores) -> float:
    """Map FoundationPose's hypothesis scores onto [0, 1].

    INTERFACES.md §3 specifies a score in [0, 1] and fusion WEIGHTS by it, but
    FoundationPose's scorer emits unbounded values -- ~81 on the demo sequence.

    MEASURED CAVEAT, and it matters: on mustard0 all 252 hypotheses score
    between 80.84 and 81.02, with the top twelve *exactly* tied at 81.0156. The
    raw score is therefore close to uninformative, and a softmax over it returns
    ~1/N precisely because the best hypotheses tie -- the opposite of what a
    confidence should say.

    So we report a relative margin instead: how far the winner stands out from
    the field, in standard deviations, squashed to (0, 1). All-tied inputs give
    exactly 0.5 -- "no information, weight this view neutrally" -- rather than a
    near-zero that would make fusion discard a perfectly good pose.

    Do not read the result as a calibrated probability. In practice it is close
    to constant, which means fusion's weighting between two FoundationPose views
    is near-uniform and its DISAGREEMENT GATE, not this number, is what protects
    the fused estimate. See SystemCode/pose/README.md.
    """
    if scores is None or len(scores) == 0:
        return 0.0
    # est.scores is a CUDA torch tensor; np.asarray() on one raises rather than
    # transferring, so move it to host explicitly.
    if hasattr(scores, "detach"):
        scores = scores.detach().cpu().numpy()
    values = np.asarray(scores, dtype=np.float64).reshape(-1)
    if not np.isfinite(values).all():
        return 0.0
    if values.size == 1:
        return 0.5  # a single hypothesis stands out from nothing

    best = float(values.max())
    spread = float(values.std())
    z = (best - float(values.mean())) / (spread + 1e-9)
    z = float(np.clip(z, -10.0, 10.0))
    return float(1.0 / (1.0 + np.exp(-z)))


class EstimatorPool:
    """One FoundationPose instance per tracked object.

    FoundationPose carries `pose_last` internally, so a single instance tracks a
    single object. The refiner, scorer, and rasterizer context are expensive and
    stateless with respect to the object, so they are built once and shared.
    """

    def __init__(self, asset_root: str, refine_iter_register: int, refine_iter_track: int):
        # FoundationPose is imported here, not at module scope, so --help and a
        # bad command line fail fast instead of after loading CUDA.
        import nvdiffrast.torch as dr
        from estimater import FoundationPose, PoseRefinePredictor, ScorePredictor

        self._FoundationPose = FoundationPose
        self.asset_root = asset_root
        self.refine_iter_register = refine_iter_register
        self.refine_iter_track = refine_iter_track

        log.info("loading refiner and scorer (shared across all objects)...")
        self.scorer = ScorePredictor()
        self.refiner = PoseRefinePredictor()
        self.glctx = dr.RasterizeCudaContext()

        self._meshes: Dict[str, object] = {}
        self._objects: Dict[Tuple[str, str, int], object] = {}
        # Last score each object actually earned, at registration.
        self._scores: Dict[Tuple[str, str, int], float] = {}
        log.info("estimator pool ready")

    def _mesh(self, mesh_path: str):
        if mesh_path not in self._meshes:
            import trimesh

            path = mesh_path
            if not os.path.isabs(path):
                path = os.path.join(self.asset_root, path)
            if not os.path.exists(path):
                raise FileNotFoundError(f"mesh not found: {path}")
            log.info("loading mesh %s", path)
            self._meshes[mesh_path] = trimesh.load(path)
        return self._meshes[mesh_path]

    def _estimator(self, key: Tuple[str, str, int], mesh_path: str):
        if key not in self._objects:
            mesh = self._mesh(mesh_path)
            self._objects[key] = self._FoundationPose(
                model_pts=mesh.vertices,
                model_normals=mesh.vertex_normals,
                mesh=mesh,
                scorer=self.scorer,
                refiner=self.refiner,
                debug_dir="/tmp/fp_server_debug",
                debug=0,
                glctx=self.glctx,
            )
            log.info("created estimator for %s", key)
        return self._objects[key]

    def handle(self, header: dict, rgb: np.ndarray, depth: np.ndarray) -> dict:
        key = (header["camera_id"], header["cls"], int(header["track_id"]))
        est = self._estimator(key, header["mesh"])

        # The ROI crop moves the principal point. Passing the full-frame K here
        # would put the object at the wrong bearing -- a systematic pose error
        # that looks like a calibration fault rather than a cropping bug.
        roi_x, roi_y, _, _ = header["roi"]
        K = np.asarray(header["K"], dtype=np.float64).reshape(3, 3).copy()
        K[0, 2] -= roi_x
        K[1, 2] -= roi_y

        depth_m = depth.astype(np.float32) * float(header["depth_scale_m"])

        mode = header.get("mode", "register")
        if mode == "register" or key not in self._scores:
            # NOTE: FoundationPose wants a segmentation mask. We have a
            # detector bounding box, so the mask is the ROI rectangle gated by
            # depth validity -- it includes background at the object's depth.
            # A YOLOv8-seg mask would be strictly better and is the upgrade
            # path; this is why registration quality should be measured, not
            # assumed.
            mask = depth_m >= 0.001
            if not mask.any():
                return {"ok": False, "reason": "no valid depth inside the ROI"}
            pose = est.register(
                K=K, rgb=rgb, depth=depth_m, ob_mask=mask,
                iteration=self.refine_iter_register,
            )
            score = normalized_hypothesis_score(getattr(est, "scores", None))
            self._scores[key] = score
            score_source = "register"
        else:
            pose = est.track_one(
                rgb=rgb, depth=depth_m, K=K, iteration=self.refine_iter_track
            )
            # track_one is pure refinement and produces no score. We carry the
            # score the object earned at registration rather than inventing one,
            # and say so, because fusion weights by this number. Re-running the
            # scorer on the refined pose would give a live value at the cost of
            # one extra forward pass -- the upgrade if fusion confidence proves
            # too coarse.
            score = self._scores[key]
            score_source = "carried_from_register"

        pose = np.asarray(pose, dtype=np.float64).reshape(4, 4)
        if not np.isfinite(pose).all():
            return {"ok": False, "reason": "estimator returned a non-finite pose"}

        return {
            "ok": True,
            "pose": [float(v) for v in pose.reshape(-1)],
            "score": float(score),
            "score_source": score_source,
            "mode": mode,
        }


def serve_connection(conn: socket.socket, pool: EstimatorPool, lock: threading.Lock) -> None:
    with conn:
        while True:
            magic = recv_exactly(conn, 4)
            if magic is None:
                return
            if magic != REQUEST_MAGIC:
                log.error("bad request magic %r, closing", magic)
                return
            try:
                header_bytes = read_block(conn, MAX_HEADER_BYTES)
                if header_bytes is None:
                    return
                rgb_bytes = read_block(conn, MAX_PAYLOAD_BYTES)
                depth_bytes = read_block(conn, MAX_PAYLOAD_BYTES)
                if rgb_bytes is None or depth_bytes is None:
                    return
            except ValueError as exc:
                log.error("framing error: %s", exc)
                return

            try:
                header = yaml.safe_load(header_bytes.decode("utf-8"))
                h, w, _ = header["rgb_shape"]
                dh, dw = header["depth_shape"]
                rgb = np.frombuffer(rgb_bytes, dtype=np.uint8).reshape(h, w, 3)
                depth = np.frombuffer(depth_bytes, dtype=np.uint16).reshape(dh, dw)
            except Exception as exc:  # noqa: BLE001
                log.exception("malformed request")
                send_response(conn, {"ok": False, "reason": f"malformed request: {exc}"})
                continue

            start = time.perf_counter()
            try:
                # CUDA state in the pool is not thread-safe; one estimation at a
                # time regardless of how many clients connect.
                with lock:
                    response = pool.handle(header, rgb, depth)
            except Exception as exc:  # noqa: BLE001
                log.exception("estimation failed")
                response = {"ok": False, "reason": f"{type(exc).__name__}: {exc}"}
            elapsed_ms = (time.perf_counter() - start) * 1e3

            log.info("%s %s track=%s -> %s (%.1f ms)", header.get("mode"),
                     header.get("cls"), header.get("track_id"),
                     "ok" if response.get("ok") else "declined", elapsed_ms)
            send_response(conn, response)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", default="/tmp/the_eyes_foundationpose.sock")
    parser.add_argument("--asset-root", default=".",
                        help="root that relative mesh paths in meshes.yaml resolve against")
    parser.add_argument("--foundationpose-root", default=os.path.expanduser("~/src/FoundationPose"),
                        help="FoundationPose checkout to import from. This server lives in our "
                             "repo, so the checkout is not on sys.path by default and must be "
                             "named explicitly rather than assumed from the working directory.")
    parser.add_argument("--register-iters", type=int, default=5)
    parser.add_argument("--track-iters", type=int, default=2)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    fp_root = os.path.abspath(args.foundationpose_root)
    if not os.path.isdir(fp_root):
        log.error("FoundationPose checkout not found at %s", fp_root)
        return 1
    if fp_root not in sys.path:
        sys.path.insert(0, fp_root)

    pool = EstimatorPool(args.asset_root, args.register_iters, args.track_iters)

    # A leftover socket file from a crashed run makes bind() fail with EADDRINUSE
    # even though nothing is listening.
    if os.path.exists(args.socket):
        os.unlink(args.socket)

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(args.socket)
    server.listen(4)
    log.info("listening on %s", args.socket)

    lock = threading.Lock()
    try:
        while True:
            conn, _ = server.accept()
            log.info("client connected")
            threading.Thread(target=serve_connection, args=(conn, pool, lock),
                             daemon=True).start()
    except KeyboardInterrupt:
        log.info("shutting down")
    finally:
        server.close()
        if os.path.exists(args.socket):
            os.unlink(args.socket)
    return 0


if __name__ == "__main__":
    sys.exit(main())
