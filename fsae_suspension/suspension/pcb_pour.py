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

Benchmarked against a 2-D solve of KiCad's own shapes (docs/ELECTRICAL_BENCHMARK_PLAN.md, addenda 4 and 6). Since
round 2 it uses the same numerical method as that reference (links by the copper fraction of the box around each
cell face, coverage as area fractions, pads and via rings as near-perfect conductors), implemented again here on
KinematiK's own reading of the file; it shares the method, not the geometry reading or the code.
"""
from __future__ import annotations

import math

import numpy as np

from .pcb_doctor import (OZ_TO_UM, RHO_CU_20C, ALPHA_CU, PcbBoard, via_resistance_ohm)

POUR_TEMP_C = 60.0          # the nodal solve's copper temperature (ambient 40 °C + 20 °C), °C
MAX_CELLS_PER_LAYER = 1_500_000
_SS = 2                     # samples per half-cell edge (4 × 4 per cell)
GRID_TOL_PCT = 2.0          # two successive cell sizes must agree this closely (percent) to call the estimate converged
K_TERM = 1e4                # pads and via rings conduct this many times better than the copper (unitless)


def pour_grid_mm(board: PcbBoard, nid: int) -> float:
    """Cell size (mm): at least 12 cells across the narrowest feature that carries current (tracks and the pours'
    thermal spokes), between 0.0125 and 0.05 mm."""
    ws = [s.width_mm for s in board.segments_of(nid) if s.width_mm > 0]
    ws += [z.spoke_mm for z in board.zones if z.net == nid and z.spoke_mm > 0]
    return min(max(min(ws or [0.5]) / 12.0, 0.0125), 0.05)


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
    (ax, ay), (bx, by) = seg.start, seg.end
    return _cells_capsule(ax, ay, bx, by, seg.width_mm / 2.0, h)


def _cells_capsule(ax, ay, bx, by, hw, h):
    """A line of half width hw with round ends (a track, a custom pad's stroke, a KiCad 5 pour's outline)."""
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
    """Worst pad-to-pad resistance (Ω) among the net's first `max_pads` pads (a name two pads share gets @x,y), with
    its pours, by a sheet solve that checks its own grid: it is solved again between the same two pads at half the
    cell size (mm), and at a quarter if those differ by more than GRID_TOL_PCT. Returns {"r_ohm" (the finest solve),
    "pair", "h_mm", "grid_change_pct" (the last change, percent), "converged"} or {"r_ohm": None, "why": ...}.
    `converged` False means the finer grid did not fit in memory or the answer was still moving: an unverified
    estimate, to be read as such."""
    out = _pour_at(board, nid, pour_grid_mm(board, nid), max_pads)
    if out.get("r_ohm") is None:
        return out
    out.update(grid_change_pct=None, converged=False)
    for _ in range(2):
        fine = _pour_at(board, nid, out["h_mm"] / 2.0, max_pads, only=frozenset(out["pair"]))
        if fine.get("r_ohm") is None:                       # too large for memory at this cell size
            break
        change = 100.0 * abs(fine["r_ohm"] - out["r_ohm"]) / fine["r_ohm"]
        out.update(r_ohm=fine["r_ohm"], h_mm=fine["h_mm"], grid_change_pct=change, converged=change <= GRID_TOL_PCT)
        if out["converged"]:
            break
    return out


def _pour_at(board: PcbBoard, nid: int, h: float, max_pads: int = 10, only=None) -> dict:
    """One sheet solve at cell size h (mm): the worst pair among the first `max_pads` pads, or the pair named in
    `only`. Returns {"r_ohm", "pair", "h_mm"} or {"r_ohm": None, "why": ...}."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    hh = h / 2.0
    rho = RHO_CU_20C * (1 + ALPHA_CU * (POUR_TEMP_C - 20.0))
    sheet_g = board.copper_oz * OZ_TO_UM * 1e-6 / rho
    layers = list(board.copper_layers) or ["F.Cu", "B.Cu"]
    lidx = {ly: i for i, ly in enumerate(layers)}
    off = 1 << 20
    for ly in layers:                                      # refuse before drawing anything too large for memory
        area_mm2 = sum(abs(np.dot(np.asarray(z.outline)[:, 0], np.roll(np.asarray(z.outline)[:, 1], 1))
                           - np.dot(np.asarray(z.outline)[:, 1], np.roll(np.asarray(z.outline)[:, 0], 1))) / 2.0
                       for z in board.zones if z.net == nid and z.layer == ly and len(z.outline) >= 3)
        if area_mm2 / (h * h) > MAX_CELLS_PER_LAYER:
            return {"r_ohm": None, "why": "too large for the sheet solve", "h_mm": h}
    allp: list = []                                        # half-cell coverage: (layer, P, Q, c, is_terminal)
    for s in board.segments_of(nid):
        if s.layer in lidx:
            allp.append((s.layer, *_cells_track(s, hh), False))
    for z in board.zones:
        if z.net == nid and z.layer in lidx and len(z.outline) >= 3:
            allp.append((z.layer, *_cells_polygon(z.outline, hh), False))
            if z.stroke_mm > 0:                            # KiCad 5 fill: the outline is stroked at min_thickness
                for (ax, ay), (bx, by) in zip(z.outline, z.outline[1:] + z.outline[:1]):
                    allp.append((z.layer, *_cells_capsule(ax, ay, bx, by, z.stroke_mm / 2.0, hh), False))
    pads = [(fp, p) for fp in board.footprints for p in fp.pads if p.net == nid]
    anchors: list = []                                     # (layer, x, y, terminal key)
    for k, (fp, p) in enumerate(pads):
        cells = [_cells_pad(p, hh)]
        for kind, g in p.primitives:                       # a custom pad's copper beyond its anchor
            cells.append(_cells_polygon(g, hh) if kind == "poly" else _cells_capsule(*g, hh))
        side = p.layer or ("B.Cu" if fp.layer == "B.Cu" else "F.Cu")
        for ly in (layers if p.through else [side]):
            if ly in lidx:
                allp += [(ly, I, J, c, True) for I, J, c in cells]
                anchors.append((ly, p.at[0] + p.copper_offset[0], p.at[1] + p.copper_offset[1], ("pad", k)))
    vias = board.vias_of(nid)
    for k, v in enumerate(vias):
        I, J, c = _cells_disc(v.at[0], v.at[1], v.size_mm / 2.0, hh)
        for ly in layers:
            allp.append((ly, I, J, c, True))
            anchors.append((ly, v.at[0], v.at[1], ("via", k)))
    if not allp:
        return {"r_ohm": None, "why": "no copper", "h_mm": h}

    def _union(terms_only):
        ks, cs = [], []
        for ly, I, J, c, t in allp:
            if terms_only and not t:
                continue
            ks.append((np.int64(lidx[ly]) << 42) | ((I.astype(np.int64) + off) << 21) | (J.astype(np.int64) + off))
            cs.append(c)
        if not ks:
            return np.zeros(0, np.int64), np.zeros(0)
        k_, c_ = np.concatenate(ks), np.concatenate(cs)
        o = np.lexsort((c_, k_))
        k_, c_ = k_[o], c_[o]
        last = np.r_[k_[1:] != k_[:-1], True]
        return k_[last], np.minimum(c_[last], 1.0)

    def _gather(k_, c_, fx, fy):
        """Half cells (key, coverage) into boxes of 2 × 2: cells (fx = fy = 0) or face boxes (shift 1 on an axis)."""
        L = k_ >> 42
        P = ((k_ >> 21) & ((1 << 21) - 1)) - off
        Q = (k_ & ((1 << 21) - 1)) - off
        I, J = (P - fx) // 2, (Q - fy) // 2
        kk = (L << 42) | ((I + off) << 21) | (J + off)
        u, inv = np.unique(kk, return_inverse=True)
        return u, np.minimum(np.bincount(inv, weights=c_) / 4.0, 1.0)

    Ha, Ht = _union(False), _union(True)
    key, _cov = _gather(*Ha, 0, 0)
    n = key.size
    per_layer = np.bincount((key >> 42).astype(int))
    if per_layer.max() > MAX_CELLS_PER_LAYER:
        return {"r_ohm": None, "why": "too large for the sheet solve", "h_mm": h}

    def _lookup(keys, k_, c_):
        if k_.size == 0:
            return np.zeros(keys.size)
        pos = np.searchsorted(k_, keys)
        ok = (pos < k_.size) & (k_[np.minimum(pos, k_.size - 1)] == keys)
        return np.where(ok, c_[np.minimum(pos, k_.size - 1)], 0.0)

    rows, cols, gs = [], [], []
    for fx, fy, step in ((1, 0, np.int64(1) << 21), (0, 1, np.int64(1))):
        fk, fc = _gather(*Ha, fx, fy)
        tk, tc = _gather(*Ht, fx, fy)
        nk = key + step
        pos = np.searchsorted(key, nk)
        ok = (pos < n) & (key[np.minimum(pos, n - 1)] == nk)
        a, b = np.nonzero(ok)[0], pos[ok]
        g = sheet_g * (_lookup(key[a], fk, fc) + K_TERM * _lookup(key[a], tk, tc))
        rows.append(a); cols.append(b); gs.append(g)
    anchor: dict = {}
    for ly, x, y, tk in anchors:                           # the cell holding each terminal's centre, per layer
        ck = (np.int64(lidx[ly]) << 42) | ((np.int64(math.floor(x / h)) + off) << 21) | (np.int64(math.floor(y / h)) + off)
        i_ = int(np.searchsorted(key, ck))
        if i_ < n and key[i_] == ck:
            anchor.setdefault((tk, ly), i_)
    big = sheet_g * K_TERM * 100.0
    for k in range(len(pads)):                             # a pad on several layers is one conductor through its hole
        a_ = [anchor[(("pad", k), ly)] for ly in layers if (("pad", k), ly) in anchor]
        for x, y in zip(a_, a_[1:]):
            rows.append(np.array([x])); cols.append(np.array([y])); gs.append(np.array([big]))
    for k, v in enumerate(vias):                           # via barrels between the layers they reach
        ns = [anchor[(("via", k), ly)] for ly in layers if (("via", k), ly) in anchor]
        r_seg = via_resistance_ohm(v.drill_mm, board.board_thickness_mm, temp_c=POUR_TEMP_C) / max(len(layers) - 1, 1)
        for x, y in zip(ns, ns[1:]):
            rows.append(np.array([x])); cols.append(np.array([y])); gs.append(np.array([1.0 / r_seg]))
    R, C_, G = np.concatenate(rows), np.concatenate(cols), np.concatenate(gs)
    m = (R != C_) & (G > 0)
    R, C_, G = R[m], C_[m], G[m]
    M = coo_matrix((np.r_[G, G, -G, -G], (np.r_[R, C_, R, C_], np.r_[R, C_, C_, R])), shape=(n, n)).tocsr()
    names = [f"{fp.ref}.{p.number}" for fp, p in pads]
    names = [nm if names.count(nm) == 1 else f"{nm}@{p.at[0]:.2f},{p.at[1]:.2f}"   # shared names: told apart by place
             for nm, (fp, p) in zip(names, pads)]
    pnodes = []
    for k in range(len(pads)):                             # pads on copper
        q = next((anchor[(("pad", k), ly)] for ly in layers if (("pad", k), ly) in anchor), None)
        if q is not None:
            pnodes.append((names[k], q))
    pnodes = [t for t in pnodes if t[0] in only] if only else pnodes[:max_pads]
    if len(pnodes) < 2:
        return {"r_ohm": None, "why": "fewer than two pads on copper", "h_mm": h}
    _nc, lab = connected_components(M != 0, directed=False)
    comp = lab[pnodes[0][1]]
    if any(lab[q] != comp for _n, q in pnodes):
        return {"r_ohm": None, "why": "pads not joined by copper", "h_mm": h}
    keep = np.where(lab == comp)[0]

    def _resist_from(g_node):
        """Effective resistance from one pad to every other (ground it, unit current in at each)."""
        kk = keep[keep != g_node]
        where = np.full(n, -1); where[kk] = np.arange(kk.size)
        A = M[kk][:, kk].tocsr()
        targets = [q for _n, q in pnodes if q != g_node]
        B = np.zeros((kk.size, len(targets)))
        for i, q in enumerate(targets):
            B[where[q], i] = 1.0
        X = _solve(A, B)
        return {q: float(X[where[q], i]) for i, q in enumerate(targets)}

    r0 = _resist_from(pnodes[0][1])                          # double sweep: farthest from the first, then from it
    far = max(r0, key=r0.get)
    r1 = _resist_from(far)
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
            out[:, i], _info = cg(A, B[:, i], rtol=1e-11, atol=0.0, maxiter=6000, M=Mp)
        return out
    except ImportError:
        from scipy.sparse.linalg import spsolve
        return np.column_stack([spsolve(A.tocsc(), B[:, i]) for i in range(B.shape[1])])
