# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""How current splits across the cells of a parallel group wired as a ladder.

The lap check and the pack thermal model give every cell pack current / parallel. That holds when the busbars are
stiff and the cells match; a resistive busbar between neighbouring cells makes the cells nearest the terminals carry
more. This solves the ladder by nodal analysis: cell k (0 = nearest the terminals) is its own resistance between
node k of the positive busbar and node k of the negative one, with the interconnection resistance between
neighbouring nodes on each busbar, and the load on node 0 of both (terminals on the same side).

Benchmarked against measured branch currents (docs/ACCUMULATOR_HARNESS_BENCHMARK_PLAN.md, addendum 4). It is
resistive only: as the cells' states of charge drift apart during a discharge, their open-circuit voltages differ
and the split moves, which this does not model (pass `ocv_v` to set them explicitly).
"""
from __future__ import annotations

import numpy as np


def ladder_currents(r_cell_ohm, r_interconnect_ohm: float, i_total_a: float, *, both_rails: bool = True,
                    ocv_v=None) -> np.ndarray:
    """Current of each cell (A), positive = discharging, nearest the terminals first.

    `r_cell_ohm`: each cell's resistance in ohms; `r_interconnect_ohm`: between neighbouring cells, in ohms, on both
    busbars (`both_rails=True`) or only the positive one; `i_total_a`: the group's load in amps; `ocv_v`: each cell's
    open-circuit voltage in volts (equal by default, so only resistances matter)."""
    r = np.asarray(r_cell_ohm, dtype=float).ravel()
    n = r.size
    if n == 0 or np.any(r <= 0) or r_interconnect_ohm < 0:
        raise ValueError("needs at least one cell, every cell resistance above 0 ohm, interconnection at least 0")
    e = np.zeros(n) if ocv_v is None else np.asarray(ocv_v, dtype=float).ravel()
    if e.size != n:
        raise ValueError("ocv_v needs one value per cell")
    rails = (r_interconnect_ohm, r_interconnect_ohm if both_rails else 0.0)
    # Node numbering: a busbar with no resistance is one node; otherwise one node per cell.
    pos = list(range(n)) if rails[0] > 0 else [0] * n
    off = max(pos) + 1
    neg = [off + k for k in range(n)] if rails[1] > 0 else [off] * n
    size = max(neg) + 1
    g_mat = np.zeros((size, size))
    rhs = np.zeros(size)

    def _link(a, b, g):
        g_mat[a, a] += g
        g_mat[b, b] += g
        g_mat[a, b] -= g
        g_mat[b, a] -= g

    for k in range(n):
        g = 1.0 / r[k]
        _link(pos[k], neg[k], g)
        rhs[pos[k]] += g * e[k]                 # the cell's EMF as a Norton source into its positive node
        rhs[neg[k]] -= g * e[k]
    for bar, rr in ((pos, rails[0]), (neg, rails[1])):
        if rr > 0:
            for k in range(n - 1):
                _link(bar[k], bar[k + 1], 1.0 / rr)
    rhs[pos[0]] -= i_total_a                    # the load draws the group current out of the positive terminal
    keep = [i for i in range(size) if i != neg[0]]          # and returns it to the negative one: the 0 V reference
    v = np.zeros(size)
    v[keep] = np.linalg.solve(g_mat[np.ix_(keep, keep)], rhs[keep])
    return np.array([(e[k] - (v[pos[k]] - v[neg[k]])) / r[k] for k in range(n)])
