# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""A manifest's winner as a point table and its curves, for checking in another kinematics package.

Writes <name>_points.csv (each pickup of the right-hand corner, mm: x rearward from the axle line, y outboard from the
car centreline, z up from the ground) and <name>_curves.csv (camber, toe and roll-centre height against lower ball
joint travel, -25 to +25 mm, deg and mm) so the same corner can be entered in OptimumKinematics or any other package
and the curves compared (validation plan, step 0). usage: python scripts/export_points.py [manifest ...] [--out DIR]
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension.genesis_repro import hp_from_dict                     # noqa: E402
from suspension.ghost_topology import _rc_height_mm                   # noqa: E402
from suspension.kinematics import SuspensionKinematics                # noqa: E402

POINTS = [("upper_front_inner", "Upper wishbone, front chassis pivot"), ("upper_rear_inner", "Upper wishbone, rear chassis pivot"),
          ("upper_outer", "Upper ball joint"), ("lower_front_inner", "Lower wishbone, front chassis pivot"),
          ("lower_rear_inner", "Lower wishbone, rear chassis pivot"), ("lower_outer", "Lower ball joint"),
          ("tie_rod_inner", "Tie rod, inner (rack end)"), ("tie_rod_outer", "Tie rod, outer (upright)"),
          ("wheel_center", "Wheel centre"), ("contact_patch", "Contact patch"),
          ("pushrod_outer", "Pushrod, outer"), ("rocker_pushrod", "Rocker, pushrod end"),
          ("rocker_pivot", "Rocker pivot"), ("rocker_spring", "Rocker, spring end"), ("spring_inner", "Spring, chassis end")]


def export(name: str, out: Path) -> tuple[Path, Path]:
    """Write the point and curve tables for one manifest; returns their paths."""
    d = json.loads((ROOT / "manifests" / f"{name}.recorded.genesis.json").read_text())["recorded"]["winner_hardpoints"]
    hp = hp_from_dict(d)
    pts = out / f"{name}_points.csv"
    with pts.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["point", "description", "x_mm (rearward)", "y_mm (outboard)", "z_mm (up)"])
        for k, desc in POINTS:
            v = getattr(hp, k, None)
            if v is not None:
                w.writerow([k, desc, *[f"{float(c):.4f}" for c in np.asarray(v, float)]])
        w.writerow(["static_camber_deg", "alignment", f"{float(d.get('static_camber', 0.0)):.3f}", "", ""])
        w.writerow(["static_toe_deg", "alignment", f"{float(d.get('static_toe', 0.0)):.3f}", "", ""])
    k = SuspensionKinematics(hp)
    cur = out / f"{name}_curves.csv"
    with cur.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["travel_mm (lower ball joint, + bump)", "camber_deg", "toe_deg (+ out)", "roll_centre_mm"])
        for t in np.linspace(-25.0, 25.0, 11):
            s = k.solve_at_travel(float(t))
            w.writerow([f"{t:.1f}", f"{s.camber:.4f}", f"{s.toe:.4f}", f"{_rc_height_mm(s, 1210.0):.2f}"])
    return pts, cur


if __name__ == "__main__":
    a = sys.argv[1:]
    out = Path(a[a.index("--out") + 1]) if "--out" in a else Path(".")
    names = [x for i, x in enumerate(a) if x != "--out" and (i == 0 or a[i - 1] != "--out")] or ["front_v11_final"]
    for n in names:
        print(*export(n, out))
