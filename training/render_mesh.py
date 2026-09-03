#!/usr/bin/env python3
"""Render a textured OBJ to a PNG contact sheet. Offline tooling, no dependencies
beyond numpy and Pillow.

There is no 3D viewer installed on this machine, and a mesh you cannot look at is
a mesh you are trusting blindly. This is enough to answer the questions that
actually come up: did the texture bind to the right UVs, is the object the one
the config claims, did the re-centring in prepare_meshes.py leave it intact.

It earned its keep immediately: rendering the YCB mustard bottle showed the
front label reads "40% MORE THAN OUR 14 OZ", identifying it as the 20 oz product
rather than the 14 oz one it is commonly assumed to be — a mismatch that would
have been invisible in every downstream number except ADD-S.

    python3 training/render_mesh.py assets/meshes/bowl.obj \
        assets/meshes/bowl_texture.png /tmp/bowl.png

A software rasteriser rather than a real renderer: painter's-algorithm z-buffer,
backface culling, one directional light, nearest-neighbour texture sampling. It
is for inspection, not for generating training data — Replicator does that.
"""

import numpy as np
from PIL import Image
import sys

def load(path):
    V, VT, F = [], [], []
    for line in open(path, errors='ignore'):
        p = line.split()
        if not p: continue
        if p[0] == 'v':  V.append([float(x) for x in p[1:4]])
        elif p[0] == 'vt': VT.append([float(x) for x in p[1:3]])
        elif p[0] == 'f':
            idx = []
            for tok in p[1:]:
                a = tok.split('/')
                vi = int(a[0]) - 1
                ti = int(a[1]) - 1 if len(a) > 1 and a[1] else -1
                idx.append((vi, ti))
            for k in range(1, len(idx) - 1):       # fan-triangulate
                F.append([idx[0], idx[k], idx[k + 1]])
    return np.array(V), (np.array(VT) if VT else None), F

def render(V, VT, F, tex, yaw, W=440, H=620, fov=38.0):
    c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    P = V @ R.T
    # z up -> camera y down; look from -y at distance d
    d = 0.45
    cam = np.stack([P[:, 0], -P[:, 2], P[:, 1] + d], 1)
    f = (H / 2) / np.tan(np.radians(fov) / 2)
    img = np.zeros((H, W, 3), np.float32)
    zb = np.full((H, W), np.inf)
    th, tw = (tex.shape[0], tex.shape[1]) if tex is not None else (0, 0)
    light = np.array([0.4, -0.75, 0.53]); light /= np.linalg.norm(light)

    for tri in F:
        vi = [t[0] for t in tri]
        pc = cam[vi]
        if (pc[:, 2] <= 0.02).any(): continue
        u = f * pc[:, 0] / pc[:, 2] + W / 2
        v = f * pc[:, 1] / pc[:, 2] + H / 2
        n = np.cross(P[vi[1]] - P[vi[0]], P[vi[2]] - P[vi[0]])
        ln = np.linalg.norm(n)
        if ln < 1e-12: continue
        n = n / ln
        # backface cull in camera space
        nc = np.cross(pc[1] - pc[0], pc[2] - pc[0])
        if nc[2] >= 0: continue
        shade = 0.30 + 0.70 * max(0.0, float(n @ light))
        x0, x1 = max(int(u.min()), 0), min(int(u.max()) + 2, W)
        y0, y1 = max(int(v.min()), 0), min(int(v.max()) + 2, H)
        if x0 >= x1 or y0 >= y1: continue
        xs, ys = np.meshgrid(np.arange(x0, x1) + .5, np.arange(y0, y1) + .5)
        den = (v[1]-v[2])*(u[0]-u[2]) + (u[2]-u[1])*(v[0]-v[2])
        if abs(den) < 1e-12: continue
        w0 = ((v[1]-v[2])*(xs-u[2]) + (u[2]-u[1])*(ys-v[2])) / den
        w1 = ((v[2]-v[0])*(xs-u[2]) + (u[0]-u[2])*(ys-v[2])) / den
        w2 = 1 - w0 - w1
        m = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
        if not m.any(): continue
        z = w0*pc[0,2] + w1*pc[1,2] + w2*pc[2,2]
        sub = zb[y0:y1, x0:x1]
        upd = m & (z < sub)
        if not upd.any(): continue
        col = np.full(w0.shape + (3,), 0.85, np.float32)
        ti = [t[1] for t in tri]
        if tex is not None and all(t >= 0 for t in ti):
            tu = w0*VT[ti[0],0] + w1*VT[ti[1],0] + w2*VT[ti[2],0]
            tv = w0*VT[ti[0],1] + w1*VT[ti[1],1] + w2*VT[ti[2],1]
            px = np.clip((tu*tw).astype(int), 0, tw-1)
            py = np.clip(((1-tv)*th).astype(int), 0, th-1)
            col = tex[py, px]
        sub[upd] = z[upd]
        img[y0:y1, x0:x1][upd] = np.clip(col[upd]*shade, 0, 1)
    bg = np.full_like(img, 0.13)
    hit = np.isfinite(zb)[..., None]
    return np.where(hit, img, bg)

V, VT, F = load(sys.argv[1])
tex = np.asarray(Image.open(sys.argv[2]).resize((1024,1024), Image.LANCZOS), np.float32)/255.0
tiles = [render(V, VT, F, tex, y) for y in (0, 55, 130, 250)]
out = (np.concatenate(tiles, 1)*255).astype(np.uint8)
Image.fromarray(out).save(sys.argv[3])
print(f"{len(V)} verts, {len(F)} tris -> {sys.argv[3]}")
