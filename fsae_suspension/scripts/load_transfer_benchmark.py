# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Steady lateral load transfer (VehicleDynamics.lateral_load_transfer) against an OpenSeesPy static solve of the
same roll-centre model: four cases, wheel loads and roll angle. Limits: benchmarks/load_transfer/limits.json,
registered before the run. Needs openseespy (pip install openseespy).

    python scripts/load_transfer_benchmark.py      # exit 1 on any failure
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from suspension.dynamics import VehicleDynamics, VehicleParams  # noqa: E402

BENCH = ROOT / "benchmarks" / "load_transfer"
REG = json.loads((BENCH / "limits.json").read_text())


def _corner_at_rc(rc_mm: float, track_mm: float):
    """A stand-in for a solved corner whose roll centre is rc_mm: the instant centre on the centreline."""
    st = SimpleNamespace(instant_center=(0.0, rc_mm), contact_patch=(0.0, track_mm / 2, 0.0))
    return SimpleNamespace(static=st)


def kinematik(c: dict) -> dict:
    p = VehicleParams(mass=c["mass_kg"], cg_height=c["cg_height_mm"], wheelbase=c["wheelbase_mm"],
                      track_front=c["track_front_mm"], track_rear=c["track_rear_mm"],
                      weight_dist_front=c["weight_dist_front"], roll_stiffness_front=c["k_front_Nm_deg"],
                      roll_stiffness_rear=c["k_rear_Nm_deg"])
    veh = VehicleDynamics(p, front_kin=_corner_at_rc(c["rc_front_mm"], c["track_front_mm"]),
                          rear_kin=_corner_at_rc(c["rc_rear_mm"], c["track_rear_mm"]))
    loads, info = veh.lateral_load_transfer(c["lateral_g"])
    assert abs(info["rc_front"] - c["rc_front_mm"]) < 1e-9 and abs(info["rc_rear"] - c["rc_rear_mm"]) < 1e-9
    return {"fl_N": loads.fl, "fr_N": loads.fr, "rl_N": loads.rl, "rr_N": loads.rr,
            "roll_angle_deg": info["roll_angle"]}


def opensees(c: dict) -> dict:
    import openseespy.opensees as ops
    g = 9.81
    m, h, L = c["mass_kg"], c["cg_height_mm"], c["wheelbase_mm"]
    a = L * (1 - c["weight_dist_front"])                     # CG behind the front axle, mm
    ops.wipe()
    ops.model("basic", "-ndm", 3, "-ndf", 6)
    # x forward-to-rear, y to the right (outside of a left turn), z up; mm, N
    ops.node(1, a, 0.0, h)                                    # CG, on the sprung body
    ops.node(2, 0.0, 0.0, c["rc_front_mm"])                   # front roll centre, body side
    ops.node(3, L, 0.0, c["rc_rear_mm"])                      # rear roll centre, body side
    ops.node(12, 0.0, 0.0, c["rc_front_mm"])                  # front roll centre, axle side
    ops.node(13, L, 0.0, c["rc_rear_mm"])
    tf, tr = c["track_front_mm"], c["track_rear_mm"]
    ops.node(21, 0.0, -tf / 2, 0.0); ops.node(22, 0.0, tf / 2, 0.0)     # FL, FR contact patches
    ops.node(31, L, -tr / 2, 0.0); ops.node(32, L, tr / 2, 0.0)         # RL, RR
    # rigid body and axles as very stiff beams (rigid links cannot also carry the roll-centre ties)
    ops.geomTransf("Linear", 1, 0.0, 1.0, 0.0)               # body members lie in the x-z plane
    ops.geomTransf("Linear", 2, 1.0, 0.0, 0.0)               # axle members lie in the y-z plane
    for e, (i, j, t) in enumerate(((1, 2, 1), (1, 3, 1), (12, 21, 2), (12, 22, 2), (13, 31, 2), (13, 32, 2)), 10):
        ops.element("elasticBeamColumn", e, i, j, 1e6, 1e9, 1e9, 1e6, 1e6, 1e6, t)   # EI, GJ, EA = 1e15
    for body, axle in ((2, 12), (3, 13)):
        ops.equalDOF(axle, body, 1, 2, 3, 5, 6)               # roll (DOF 4) is the spring's
    ops.uniaxialMaterial("Elastic", 1, c["k_front_Nm_deg"] * 1000.0 * 180.0 / math.pi)   # N mm / rad
    ops.uniaxialMaterial("Elastic", 2, c["k_rear_Nm_deg"] * 1000.0 * 180.0 / math.pi)
    ops.element("zeroLength", 1, 12, 2, "-mat", 1, "-dir", 4)
    ops.element("zeroLength", 2, 13, 3, "-mat", 2, "-dir", 4)
    ops.fix(21, 1, 1, 1, 0, 0, 0)
    ops.fix(22, 0, 1, 1, 0, 0, 0)
    ops.fix(31, 0, 0, 1, 0, 0, 0)
    ops.fix(32, 0, 1, 1, 0, 0, 0)
    ops.timeSeries("Constant", 1)
    ops.pattern("Plain", 1, 1)
    ops.load(1, 0.0, m * g * c["lateral_g"], -m * g, 0.0, 0.0, 0.0)
    ops.constraints("Transformation"); ops.numberer("RCM"); ops.system("FullGeneral")
    ops.test("NormDispIncr", 1e-12, 10); ops.algorithm("Linear"); ops.integrator("LoadControl", 1.0)
    ops.analysis("Static")
    assert ops.analyze(1) == 0
    ops.reactions()
    out = {k: ops.nodeReaction(n, 3) for k, n in (("fl_N", 21), ("fr_N", 22), ("rl_N", 31), ("rr_N", 32))}
    # body roll outward (towards +y, the outside of the turn) is a negative rotation about +x here
    out["roll_angle_deg"] = -math.degrees(ops.nodeDisp(1, 4) - ops.nodeDisp(12, 4))
    ops.wipe()
    return out


def run() -> dict:
    lim = REG["limits"]
    out, ok = {}, True
    for name, c in REG["cases"].items():
        k, o = kinematik(c), opensees(c)
        rows = {}
        for q in REG["quantities"]:
            tol = lim["roll_angle_deg"] if q == "roll_angle_deg" else lim["wheel_load_N"]
            d = k[q] - o[q]
            rows[q] = {"kinematik": k[q], "opensees": o[q], "diff": d, "passes": bool(abs(d) <= tol)}
            ok = ok and rows[q]["passes"]
        out[name] = rows
    out["passed"] = bool(ok)
    return out


if __name__ == "__main__":
    res = run()
    (BENCH / "results.json").write_text(json.dumps(res, indent=1))
    for name, rows in res.items():
        if name != "passed":
            for q, r in rows.items():
                print(f"{name:12s} {q:15s} KinematiK {r['kinematik']:10.4f}  OpenSees {r['opensees']:10.4f}  "
                      f"diff {r['diff']:+.2e} {'PASS' if r['passes'] else 'FAIL'}")
    print("PASSED" if res["passed"] else "FAILED")
    sys.exit(0 if res["passed"] else 1)
