# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Project Chrono run of the geometry benchmark: contact-patch path, scrub, trail and the rocker's damper length.

Shares no code with KinematiK: it reads only the corners' recorded hardpoints, builds each as Chrono bodies and joints
(the helpers of chrono_benchmark.py) and writes benchmarks/adams_geometry/chrono_geometry.csv, which
adams_geometry_benchmark.py compares with KinematiK under the limits it set for Adams. Same environment as
chrono_benchmark.py:

    micromamba create -p ./chrono -c projectchrono -c conda-forge python=3.12 pychrono=9.0.1 numpy
    ./chrono/bin/python scripts/chrono_geometry_benchmark.py

Model, mm, no gravity: both wishbones on revolute pivots (the lower one driven in angle by a rotation motor), spherical
ball joints, the tie rod as a distance link; the rocker on a revolute about its declared axis, held by the pushrod as a
distance link from its point on the upright (or wishbone). DoAssembly at each motor angle, closure checked; the contact
patch is a point fixed to the upright, the kingpin axis runs from the lower to the upper ball joint, the damper from the
rocker's spring point to the chassis mount. Everything is interpolated onto -25..+25 mm of wheel-centre travel.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import chrono_benchmark as cb                                                         # noqa: E402
import pychrono as ch                                                                 # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "benchmarks" / "adams_geometry" / "chrono_geometry.csv"
CORNERS = ("front_v11_final", "rear_v11_final")
TRAVEL = np.arange(-25.0, 26.0, 1.0)


def geometry_curves(w: dict) -> dict[str, np.ndarray]:
    """cp_dx, cp_dz, scrub, trail and damper_len (mm) at TRAVEL of the wheel centre."""
    P = {k: np.asarray(v, float) for k, v in w.items() if isinstance(v, list) and len(v) == 3}
    s = cb._system()
    ground = cb._body(s, fixed=True)
    lca, uca, up, rk = cb._body(s), cb._body(s), cb._body(s), cb._body(s)
    mot = ch.ChLinkMotorRotationAngle()
    mot.Initialize(lca, ground, cb._frame_z_along(P["lower_front_inner"],
                                                  P["lower_rear_inner"] - P["lower_front_inner"]))
    fn = ch.ChFunctionConst(0.0)
    mot.SetAngleFunction(fn)
    rev = ch.ChLinkLockRevolute()
    rev.Initialize(uca, ground, cb._frame_z_along(P["upper_front_inner"],
                                                  P["upper_rear_inner"] - P["upper_front_inner"]))
    rrev = ch.ChLinkLockRevolute()
    rrev.Initialize(rk, ground, cb._frame_z_along(P["rocker_pivot"], P["rocker_axis"]))
    s1, s2 = ch.ChLinkLockSpherical(), ch.ChLinkLockSpherical()
    s1.Initialize(up, lca, ch.ChFramed(cb.V(P["lower_outer"])))
    s2.Initialize(up, uca, ch.ChFramed(cb.V(P["upper_outer"])))
    for x in (mot, rev, rrev, s1, s2):
        s.Add(x)
    cb._distance(s, up, ground, P["tie_rod_outer"], P["tie_rod_inner"])
    carrier = up if w.get("pushrod_attach", "upright") == "upright" else lca
    cb._distance(s, carrier, rk, P["pushrod_outer"], P["rocker_pushrod"])
    angles = np.radians(np.linspace(-7.0, 7.0, 281))
    order = np.argsort(np.abs(angles), kind="stable")
    res = {}
    for n, k in enumerate(order):                      # walk out from ride height in both directions
        if n == 0 or np.sign(angles[k]) != np.sign(angles[order[n - 1]]):
            for b in (lca, uca, up, rk):
                b.SetPos(ch.ChVector3d(0, 0, 0)); b.SetRot(ch.ChQuaterniond(1, 0, 0, 0))  # noqa: E702
        fn.SetConstant(float(angles[k]))
        for _ in range(3):
            s.DoAssembly(ch.POSITION, 100, 1e-10, 1e-14, 1e-12)
        g = {p: cb.A(b.TransformPointLocalToParent(cb.V(P[p]))) for b, p in
             ((up, "wheel_center"), (up, "contact_patch"), (up, "lower_outer"), (up, "upper_outer"),
              (up, "tie_rod_outer"), (rk, "rocker_pushrod"), (rk, "rocker_spring"))}
        g["pushrod_outer"] = cb.A(carrier.TransformPointLocalToParent(cb.V(P["pushrod_outer"])))
        for a, b in (("tie_rod_outer", "tie_rod_inner"), ("pushrod_outer", "rocker_pushrod")):
            far = P[b] if b == "tie_rod_inner" else g[b]
            err = abs(np.linalg.norm(g[a] - far) - np.linalg.norm(P[a] - P[b]))
            if err > 1e-6:
                raise RuntimeError(f"Chrono assembly did not close ({a}: {err:.2e} mm)")
        cp, lo, uo = g["contact_patch"], g["lower_outer"], g["upper_outer"]
        kp = uo - lo
        road = lo + (cp[2] - lo[2]) / kp[2] * kp      # kingpin axis on the road plane through the contact patch
        res[k] = (g["wheel_center"][2] - P["wheel_center"][2], cp[0] - P["contact_patch"][0],
                  cp[2] - P["contact_patch"][2], cp[1] - road[1], cp[0] - road[0],
                  float(np.linalg.norm(g["rocker_spring"] - P["spring_inner"])))
    arr = np.array([res[k] for k in range(len(angles))])
    srt = np.argsort(arr[:, 0])
    if arr[srt[0], 0] > -25 or arr[srt[-1], 0] < 25:
        raise RuntimeError("sweep does not cover -25..+25 mm of wheel travel")
    out = {name: np.interp(TRAVEL, arr[srt, 0], arr[srt, i])
           for i, name in enumerate(("cp_dx", "cp_dz", "scrub", "trail", "damper_len"), start=1)}
    out["damper_len"] = out["damper_len"] - out["damper_len"][TRAVEL == 0.0][0]
    return out


def main():
    cb.calibrate()
    rows = ["corner,travel_mm,cp_dx_mm,cp_dz_mm,scrub_mm,trail_mm,damper_len_mm"]
    for n in CORNERS:
        w = json.loads((ROOT / "manifests" / f"{n}.recorded.genesis.json").read_text())["recorded"]["winner_hardpoints"]
        c = geometry_curves(w)
        for i, t in enumerate(TRAVEL):
            rows.append(",".join([n, f"{t:g}"] + [f"{c[q][i]:.9f}" for q in
                                                   ("cp_dx", "cp_dz", "scrub", "trail", "damper_len")]))
        print(f"{n}: trail at ride height {c['trail'][25]:.4f} mm, scrub {c['scrub'][25]:.4f} mm")
    OUT.write_text("\n".join(rows) + "\n")
    print(f"wrote {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
