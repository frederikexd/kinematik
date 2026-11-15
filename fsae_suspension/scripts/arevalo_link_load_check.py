# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Why Arevalo, Medina and Valladolid, Ingenius 20 (2018) 95-106, is not a link-load benchmark.

The paper's kinematics came from Lotus Suspension Analysis (scripts/lotus_benchmark.py uses it). Its member forces,
tables 5 and 6, did not: the authors solved a 6x6 equilibrium by hand (eq. 11-16), and the patch loads behind each
scenario are not published (no friction coefficient, speeds, curve radius or vertical acceleration). The check left is
to run the solve backwards: the load each published row balances is -A x, with A built from table 3. The obstacle
scenario (eq. 10) is a vertical force at the wheel centre, so -A x should be (0, 0, Fz, 0, 0, 0).

It is not, under any of the eight readings of the paper's conventions (front/rear leg labels on each wishbone, sign).
The best leaves 372 N longitudinal and 300 N lateral on the front and 2607 N lateral on the rear. The tables also carry
typos (front "maximum" -2290 where the column has -2990; rear BAD maximum +776 where the column has -776), and the paper
treats the pushrod, mounted on the lower wishbone (P6), as a member on the wheel body.

    python scripts/arevalo_link_load_check.py
"""
from __future__ import annotations

import itertools

import numpy as np

# table 3 (mm); P5a is the first "P5" row (upper inner), P5 the second (upper ball joint), P14 the wheel centre
POINTS = {
    "front": dict(P1=(1443.9, 212, 227.59), P2=(1732, 212, 227.59), P3=(1730, 520, 218.83), P5a=(1446.6, 256, 377.59),
                  P4=(1732, 256, 377.59), P5=(1760, 520, 400.83), P6=(1710, 480.74, 244.78), P8=(1710, 295.72, 509.03),
                  P9=(1790, 520, 218.83), P10=(1750, 190, 227.59), P14=(1760, 590, 310)),
    "rear": dict(P1=(15, 250, 203.59), P2=(335, 250, 203.59), P3=(148.83, 555, 219), P5a=(15, 290, 347.59),
                 P4=(335, 290, 347.59), P5=(171.17, 529, 401), P6=(148.83, 513, 241.8), P8=(148.83, 308, 628.68),
                 P9=(220, 549.14, 259.83), P10=(220, 258.5, 234.2), P14=(160, 600, 310)),
}
# tables 5 and 6, obstacle row, N: PR, BCSF, BCST, BCIF, BCIT, BAD
OBSTACLE = {"front": [5358, 1203, 906, -2990, -1954, -629], "rear": [8544, 1646, 1986, -2593, -2915, 521]}


def recovered_loads(side: str) -> list[tuple]:
    """(|Fxy|/Fz, swap_upper, swap_lower, sign, B) for every reading, best first; B = (F N, M N*mm) at the wheel centre."""
    P = {k: np.array(v, float) for k, v in POINTS[side].items()}
    wc, out = P["P14"], []
    for su, sl, sg in itertools.product((0, 1), (0, 1), (1, -1)):
        up = (("P5", "P4"), ("P5", "P5a")) if su else (("P5", "P5a"), ("P5", "P4"))
        lo = (("P3", "P2"), ("P3", "P1")) if sl else (("P3", "P1"), ("P3", "P2"))
        mem = [("P6", "P8"), *up, *lo, ("P9", "P10")]
        A = np.zeros((6, 6))
        for j, (o, i) in enumerate(mem):
            u = (P[o] - P[i]) / np.linalg.norm(P[o] - P[i])
            A[:3, j], A[3:, j] = u, np.cross(P[o] - wc, u)
        B = -sg * A @ np.array(OBSTACLE[side], float)
        out.append((float(np.hypot(B[0], B[1]) / abs(B[2])), su, sl, sg, B))
    return sorted(out, key=lambda r: r[0])


if __name__ == "__main__":
    for s in POINTS:
        r = recovered_loads(s)[0]
        print(f"{s}: best reading leaves |Fxy|/Fz = {r[0]:.2f}, F = {np.round(r[4][:3])} N, "
              f"M = {np.round(r[4][3:] / 1000)} N*m (should be 0, 0, Fz and 0)")
