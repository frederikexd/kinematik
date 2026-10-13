# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""An independent check of the forward kinematics: a second solver that shares no code with suspension/kinematics.py.

KinematiK's solver takes the three outer joints as nine unknowns. This one takes the upright as a rigid body with six
unknowns (a rotation vector and a translation) and solves the six closure conditions with scipy's fsolve: the four
wishbone leg lengths, the tie-rod length and the lower ball joint's height (the travel). Camber and toe are read off
the spin axis carried by the upright, and the roll-centre height off the contact patch's path, z_rc = y_cp * dy/dz (the
patch moves perpendicular to the line from it to the front-view instant centre). Only the pickup coordinates and the
static camber and toe are shared with the main solver. Paper section 3.3; supplement M6.2.

usage: python scripts/independent_kinematics.py [manifest ...]   (default front_v11_final rear_v11_final front_v11_joints50)
Lengths mm, angles deg.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import fsolve
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
MAN = ROOT / "manifests"
TRAVEL = np.linspace(-25.0, 25.0, 11)


def _p(hp: dict, k: str) -> np.ndarray:
    return np.asarray(hp[k], float)


class Corner:
    """A double-wishbone corner from a manifest's winner hardpoints (a plain dict, mm, corner frame x rearward)."""

    def __init__(self, hp: dict):
        self.ufi, self.uri, self.lfi, self.lri = (_p(hp, k) for k in (
            "upper_front_inner", "upper_rear_inner", "lower_front_inner", "lower_rear_inner"))
        self.uo0, self.lo0, self.tro0, self.tri = (_p(hp, k) for k in (
            "upper_outer", "lower_outer", "tie_rod_outer", "tie_rod_inner"))
        self.cp0 = _p(hp, "contact_patch")
        cam, toe = np.radians(float(hp.get("static_camber", 0.0))), np.radians(float(hp.get("static_toe", 0.0)))
        s = np.array([np.sin(toe), np.cos(toe) * np.cos(cam), -np.sin(cam)])
        self.spin0 = s / np.linalg.norm(s)
        self.L = [np.linalg.norm(self.uo0 - self.ufi), np.linalg.norm(self.uo0 - self.uri),
                  np.linalg.norm(self.lo0 - self.lfi), np.linalg.norm(self.lo0 - self.lri),
                  np.linalg.norm(self.tro0 - self.tri)]

    def _move(self, x, p):
        return Rotation.from_rotvec(x[:3]).apply(p - self.lo0) + self.lo0 + x[3:]

    def _res(self, x, travel):
        uo, lo, tro = (self._move(x, p) for p in (self.uo0, self.lo0, self.tro0))
        return [np.linalg.norm(uo - self.ufi) - self.L[0], np.linalg.norm(uo - self.uri) - self.L[1],
                np.linalg.norm(lo - self.lfi) - self.L[2], np.linalg.norm(lo - self.lri) - self.L[3],
                np.linalg.norm(tro - self.tri) - self.L[4], lo[2] - (self.lo0[2] + travel)]

    def solve(self, travel, x0=None):
        """Pose vector (rotation vector, translation) at a lower-ball-joint travel (mm)."""
        x, _, ok, _ = fsolve(self._res, np.zeros(6) if x0 is None else x0, args=(travel,), xtol=1e-13,
                             full_output=True)
        assert max(abs(np.array(self._res(x, travel)))) < 1e-7, "no closure"
        return x

    def state(self, x) -> dict:
        s = Rotation.from_rotvec(x[:3]).apply(self.spin0)
        return {"camber": float(-np.degrees(np.arctan2(s[2], abs(s[1])))),
                "toe": float(np.degrees(np.arctan2(s[0], abs(s[1])))), "cp": self._move(x, self.cp0)}

    def curves(self) -> dict:
        x, out = None, {}
        for t in sorted(TRAVEL, key=abs):                      # walk out from ride height on the same branch
            x = self.solve(float(t), None if t == 0 else out[min(out, key=lambda u: abs(u - t))]["x"])
            out[float(t)] = {"x": x, **self.state(x)}
        h = 0.01
        a, b = (self.state(self.solve(s, out[0.0]["x"]))["cp"] for s in (-h, h))
        rc = float(self.cp0[1] * (b[1] - a[1]) / (b[2] - a[2]))
        return {"travel": [float(t) for t in TRAVEL], "camber": [out[float(t)]["camber"] for t in TRAVEL],
                "toe": [out[float(t)]["toe"] for t in TRAVEL], "rc_mm": abs(rc)}


def compare(name: str) -> dict:
    """The independent solver against KinematiK's on one manifest's winner: worst differences over +/-25 mm."""
    sys.path.insert(0, str(ROOT))
    from suspension.genesis_repro import hp_from_dict
    from suspension.ghost_topology import _rc_height_mm
    from suspension.kinematics import SuspensionKinematics
    d = json.loads((MAN / f"{name}.recorded.genesis.json").read_text())["recorded"]["winner_hardpoints"]
    ind = Corner(d).curves()
    k = SuspensionKinematics(hp_from_dict(d))
    sts = [k.solve_at_travel(float(t)) for t in TRAVEL]
    cam, toe = np.array([s.camber for s in sts]), np.array([s.toe for s in sts])
    ic, it = np.array(ind["camber"]), np.array(ind["toe"])
    return {"name": name, "camber_max_diff_deg": float(np.max(np.abs(ic - cam))),
            "toe_max_diff_deg": float(np.max(np.abs(it - toe))),
            "camber_change_deg": float(cam[-1] - cam[0]), "toe_change_deg": float(toe.max() - toe.min()),
            "rc_independent_mm": ind["rc_mm"], "rc_kinematik_mm": float(_rc_height_mm(sts[5], 1210.0))}


if __name__ == "__main__":
    names = sys.argv[1:] or ["front_v11_final", "rear_v11_final", "front_v11_joints50"]
    for n in names:
        r = compare(n)
        print(json.dumps({k: (round(v, 6) if isinstance(v, float) else v) for k, v in r.items()}))
