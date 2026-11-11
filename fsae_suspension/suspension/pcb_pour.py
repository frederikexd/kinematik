# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Best-estimate pad-to-pad resistance of a net *with* its copper pours.

`pcb_doctor.analyze_net` gives a pour no resistance (connectivity only), so its
number is an upper bound on a poured net; the brown-out check keeps using that
bound. This module answers the other question, "about how much is it really",
by solving the current through the net's copper as a sheet: tracks, pads, via
rings and the pours KiCad filled (`filled_polygon`, clearances and thermal
spokes included), each drawn on a square grid per copper layer.

Benchmarked against an independent 2-D solve of KiCad's own shapes
(docs/ELECTRICAL_BENCHMARK_PLAN.md, addendum 4). It shares the physics with that
reference, not the geometry reading, the grid or the code.
"""
from __future__ import annotations

import math

import numpy as np

from .pcb_doctor import (OZ_TO_UM, RHO_CU_20C, ALPHA_CU, PcbBoard, via_resistance_ohm)

POUR_TEMP_C = 60.0          # the nodal solve's copper temperature (ambient 40 °C + 20 °C), °C
MAX_CELLS_PER_LAYER = 3_000_000
_SS = 3                     # samples per cell edge


def pour_grid_mm(board: PcbBoard, nid: int) -> float:
    """Cell size (mm): at least 6 cells across the narrowest track of the net (0.5 mm assumed for pours' spokes),
    between 0.025 and 0.1 mm."""
    ws = [s.width_mm for s in board.segments_of(nid) if s.width_mm > 0]
    w = min(ws + ([0.5] if any(z.net == nid for z in board.zones) else []) or [0.5])
    return min(max(w / 6.0, 0.025), 0.1)


def _samples(i0, i1, j0, j1, h):
    sub = (np.arange(_SS) + 0.5) / _SS
    xs = ((np.arange(i0, i1)[:, None] + sub[None, :]).ravel()) * h
    ys = ((np.arange(j0, j1)[:, None] + sub[None, :]).ravel()) * h
    return xs, ys


def _reduce(inside, i0, j0):
    """Boolean samples (nx*SS, ny*SS) -> cell indices and coverage."""
    nx, ny = inside.shape[0] // _SS, inside.shape[1] // _SS
    c = inside.reshape(nx, _SS, ny, _SS).mean(axis=(1, 3))
    I, J = np.nonzero(c > 0)
    return I + i0, J + j0, c[I, J]


def _cells_track(seg, h):
    (ax, ay), (bx, by), hw = seg.start, seg.end, seg.width_mm / 2.0
    i0, i1 = int(math.floor((min(ax, bx) - hw) / h)) - 1, int(math.ceil((max(ax, bx) + hw) / h)) + 1
    j0, j1 = int(math.floor((min(ay, by) - hw) / h)) - 1, int(math.ceil((max(ay, by) + hw) / h)) + 1
    xs, ys = _samples(i0, i1, j0, j1, h)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    u = np.clip(((X - ax) * dx + (Y - ay) * dy) / L2, 0, 1) if L2 > 0 else 0.0
    return _reduce(np.hypot(X - (ax + u * dx), Y - (ay + u * dy)) <= hw, i0, j0)


def _cells_disc(cx, cy, r, h):
    i0, i1 = int(math.floor((cx - r) / h)) - 1, int(math.ceil((cx + r) / h)) + 1
    j0, j1 = int(math.floor((cy - r) / h)) - 1, int(math.ceil((cy + r) / h)) + 1
    xs, ys = _samples(i0, i1, j0, j1, h)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    return _reduce(np.hypot(X - cx, Y - cy) <= r, i0, j0)


def _cells_pad(pad, h):
    """A pad in its own frame: circle, stadium (oval), rounded rectangle (radius 0.25 × the short side) or
    rectangle, turned by its angle, its copper shifted by its offset."""
    w, ht = pad.size
    cx, cy = pad.at[0] + pad.copper_offset[0], pad.at[1] + pad.copper_offset[1]
    R = math.hypot(w, ht) / 2.0
    i0, i1 = int(math.floor((cx - R) / h)) - 1, int(math.ceil((cx + R) / h)) + 1
    j0, j1 = int(math.floor((cy - R) / h)) - 1, int(math.ceil((cy + R) / h)) + 1
    xs, ys = _samples(i0, i1, j0, j1, h)
    X, Y = np.meshgrid(xs - cx, ys - cy, indexing="ij")
    t = math.radians(pad.angle_deg)
    lx = X * math.cos(t) - Y * math.sin(t)
    ly = X * math.sin(t) + Y * math.cos(t)
    ax_, ay_ = np.abs(lx), np.abs(ly)
    if pad.shape == "round" and abs(w - ht) < 1e-9:
        inside = np.hypot(lx, ly) <= w / 2.0
    elif pad.shape == "round":                                  # stadium
        r = min(w, ht) / 2.0
        ex, ey = max(w / 2.0 - r, 0.0), max(ht / 2.0 - r, 0.0)
        inside = np.hypot(np.maximum(ax_ - ex, 0), np.maximum(ay_ - ey, 0)) <= r
    elif pad.shape == "roundrect":
        r = 0.25 * min(w, ht)
        ex, ey = w / 2.0 - r, ht / 2.0 - r
        inside = (ax_ <= w / 2.0) & (ay_ <= ht / 2.0) & (np.hypot(np.maximum(ax_ - ex, 0), np.maximum(ay_ - ey, 0)) <= r)
    else:
        inside = (ax_ <= w / 2.0) & (ay_ <= ht / 2.0)
    return _reduce(inside, i0, j0)


def _cells_polygon(pts, h):
    """Even-odd fill of a (fractured) polygon, by crossings along each sample row."""
    P = np.asarray(pts, float)
    i0, i1 = int(math.floor(P[:, 0].min() / h)) - 1, int(math.ceil(P[:, 0].max() / h)) + 1
    j0, j1 = int(math.floor(P[:, 1].min() / h)) - 1, int(math.ceil(P[:, 1].max() / h)) + 1
    xs, ys = _samples(i0, i1, j0, j1, h)
    inside = np.zeros((xs.size, ys.size), bool)
    ax, ay = P[:, 0], P[:, 1]
    bx, by = np.roll(ax, -1), np.roll(ay, -1)
    for k in range(len(ax)):
        if ay[k] == by[k]:
            continue
        lo, hi = min(ay[k], by[k]), max(ay[k], by[k])
        r0, r1 = np.searchsorted(ys, lo), np.searchsorted(ys, hi)
        if r1 <= r0:
            continue
        xc = ax[k] + (ys[r0:r1] - ay[k]) * (bx[k] - ax[k]) / (by[k] - ay[k])
        c0 = np.searchsorted(xs, xc)                      # samples right of the crossing flip parity
        for rr, cc in zip(range(r0, r1), c0):
            inside[cc:, rr] ^= True
    return _reduce(inside, i0, j0)


def pour_resistance(board: PcbBoard, nid: int, max_pads: int = 10) -> dict:
    """Worst pad-to-pad resistance (Ω) among the net's first `max_pads` pads, with its pours, by a sheet solve.
    Returns {"r_ohm", "pair", "h_mm"} or {"r_ohm": None, "why": ...}."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    h = pour_grid_mm(board, nid)
    rho = RHO_CU_20C * (1 + ALPHA_CU * (POUR_TEMP_C - 20.0))
    sheet_g = board.copper_oz * OZ_TO_UM * 1e-6 / rho
    layers = list(board.copper_layers) or ["F.Cu", "B.Cu"]
    lidx = {ly: i for i, ly in enumerate(layers)}
    parts: dict = {ly: [] for ly in layers}
    for s in board.segments_of(nid):
        if s.layer in parts:
            parts[s.layer].append(_cells_track(s, h))
    for z in board.zones:
        if z.net == nid and z.layer in parts and len(z.outline) >= 3:
            parts[z.layer].append(_cells_polygon(z.outline, h))
    pads = [(fp, p) for fp in board.footprints for p in fp.pads if p.net == nid]
    term: list = []                                        # (layer, I, J, node key)
    for k, (fp, p) in enumerate(pads):
        I, J, c = _cells_pad(p, h)
        side = p.layer or ("B.Cu" if fp.layer == "B.Cu" else "F.Cu")
        for ly in (layers if p.through else [side]):
            if ly in parts:
                parts[ly].append((I, J, c))
                m = c >= 0.5
                term.append((ly, I[m], J[m], ("pad", k)))
    vias = board.vias_of(nid)
    for k, v in enumerate(vias):
        I, J, c = _cells_disc(v.at[0], v.at[1], v.size_mm / 2.0, h)
        for ly in layers:
            parts[ly].append((I, J, c))
            m = c >= 0.5
            term.append((ly, I[m], J[m], ("via", k, ly)))
    off = 1 << 20
    keys, covs = [], []
    for ly, lst in parts.items():
        if not lst:
            continue
        I = np.concatenate([q[0] for q in lst]).astype(np.int64)
        J = np.concatenate([q[1] for q in lst]).astype(np.int64)
        C = np.concatenate([q[2] for q in lst])
        k_ = (np.int64(lidx[ly]) << 42) | ((I + off) << 21) | (J + off)
        o = np.lexsort((C, k_))
        k_, C = k_[o], C[o]
        last = np.r_[k_[1:] != k_[:-1], True]
        if last.sum() > MAX_CELLS_PER_LAYER:
            return {"r_ohm": None, "why": "too large for the sheet solve", "h_mm": h}
        keys.append(k_[last]); covs.append(np.minimum(C[last], 1.0))
    if not keys:
        return {"r_ohm": None, "why": "no copper", "h_mm": h}
    key, cov = np.concatenate(keys), np.concatenate(covs)
    o = np.argsort(key); key, cov = key[o], cov[o]
    n = key.size
    node = np.arange(n)
    rep: dict = {}
    for ly, I, J, tk in term:                              # merge each terminal's cells into one node
        kk = (np.int64(lidx[ly]) << 42) | ((I.astype(np.int64) + off) << 21) | (J.astype(np.int64) + off)
        pos = np.searchsorted(key, kk)
        pos = pos[(pos < n) & (key[np.minimum(pos, n - 1)] == kk)]
        if pos.size:
            r0 = rep.setdefault(tk, int(pos[0]))
            node[pos] = r0
    rows, cols, gs = [], [], []
    for d in ((1 << 21), 1):
        nk = key + d
        pos = np.searchsorted(key, nk)
        ok = (pos < n) & (key[np.minimum(pos, n - 1)] == nk)
        a, b = np.nonzero(ok)[0], pos[ok]
        rows.append(node[a]); cols.append(node[b]); gs.append(sheet_g * 0.5 * (cov[a] + cov[b]))
    for k, v in enumerate(vias):                           # barrels between successive layers
        ns = [rep[("via", k, ly)] for ly in layers if ("via", k, ly) in rep]
        r_seg = via_resistance_ohm(v.drill_mm, board.board_thickness_mm, temp_c=POUR_TEMP_C) / max(len(layers) - 1, 1)
        for x, y in zip(ns, ns[1:]):
            rows.append(np.array([x])); cols.append(np.array([y])); gs.append(np.array([1.0 / r_seg]))
    R, C_, G = np.concatenate(rows), np.concatenate(cols), np.concatenate(gs)
    m = (R != C_) & (G > 0)
    R, C_, G = R[m], C_[m], G[m]
    M = coo_matrix((np.r_[G, G, -G, -G], (np.r_[R, C_, R, C_], np.r_[R, C_, C_, R])), shape=(n, n)).tocsr()
    pnodes = [(f"{fp.ref}.{p.number}", rep[("pad", k)]) for k, (fp, p) in enumerate(pads) if ("pad", k) in rep]
    pnodes = pnodes[:max_pads]
    if len(pnodes) < 2:
        return {"r_ohm": None, "why": "fewer than two pads on copper", "h_mm": h}
    _nc, lab = connected_components(M != 0, directed=False)
    comp = lab[pnodes[0][1]]
    if any(lab[q] != comp for _n, q in pnodes):
        return {"r_ohm": None, "why": "pads not joined by copper", "h_mm": h}
    keep = np.where((lab == comp) & (node == np.arange(n)))[0]

    def resist_from(g_node):
        """Effective resistance from one pad to every pad (ground the first, unit current in at each)."""
        kk = keep[keep != g_node]
        where = np.full(n, -1); where[kk] = np.arange(kk.size)
        A = M[kk][:, kk].tocsr()
        targets = [q for _n, q in pnodes if q != g_node]
        B = np.zeros((kk.size, len(targets)))
        for i, q in enumerate(targets):
            B[where[q], i] = 1.0
        X = _solve(A, B)
        return {q: float(X[where[q], i]) for i, q in enumerate(targets)}

    r0 = resist_from(pnodes[0][1])                          # double sweep: farthest from the first, then from it
    far = max(r0, key=r0.get)
    r1 = resist_from(far)
    best = max(r1.items(), key=lambda t: t[1])
    name = {q: nm for nm, q in pnodes}
    return {"r_ohm": best[1], "pair": (name[far], name[best[0]]), "h_mm": h}


def _solve(A, B):
    if A.shape[0] <= 200_000:
        from scipy.sparse.linalg import splu
        lu = splu(A.tocsc())
        return lu.solve(B)
    try:
        import pyamg
        from scipy.sparse.linalg import cg
        ml = pyamg.smoothed_aggregation_solver(A, symmetry="symmetric", max_coarse=500)
        Mp = ml.aspreconditioner(cycle="V")
        out = np.zeros_like(B)
        for i in range(B.shape[1]):
            out[:, i], _info = cg(A, B[:, i], rtol=1e-10, atol=0.0, maxiter=4000, M=Mp)
        return out
    except ImportError:
        from scipy.sparse.linalg import spsolve
        return np.column_stack([spsolve(A.tocsc(), B[:, i]) for i in range(B.shape[1])])
