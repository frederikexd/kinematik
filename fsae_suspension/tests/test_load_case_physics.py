# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Proofs that the statics, the load cases and the frame model obey physics, each against an independent principle
(supplement S14): virtual work, global equilibrium, vehicle equilibrium, closed-form beam results and Maxwell-Betti
reciprocity."""
import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from suspension import loadpath as lp, frame_fe as ff, tire_utilisation as tu        # noqa: E402
from suspension.kinematics import SuspensionKinematics, Hardpoints                   # noqa: E402
from suspension.genesis_repro import GenesisManifest, hp_from_dict, vehicle_load_cases  # noqa: E402

MAN = ROOT / "manifests"
# the walled searches carry the pushrod on the lower wishbone (two-body statics), the final corners on the upright
CORNERS = ["front_v11_hairpin_walls", "rear_v11_walls", "front_v11_final", "rear_v11_final"]
LOADS = [lp.WheelLoad(Fz=1200.0), lp.WheelLoad(Fy=-1800.0, Fz=1200.0, Mz=-47000.0),
         lp.WheelLoad(Fx=1933.0, Fz=1161.0), lp.WheelLoad(Fx_wc=-1560.0, Fy=-1400.0, Fz=1300.0, Mz=-37000.0)]


def _hp(name):
    return hp_from_dict(GenesisManifest.from_json((MAN / f"{name}.recorded.genesis.json").read_text())
                        .recorded["winner_hardpoints"])


def _rigid_velocity(kin, h=0.02):
    """Velocity (per mm of travel) of the wheel's material points: rigid transform of the upright between states."""
    s0, sp, sm = kin.solve_at_travel(0.0), kin.solve_at_travel(h), kin.solve_at_travel(-h)
    def pts(s):
        return np.array([s.upper_outer, s.lower_outer, s.tie_rod_outer], float)
    def fit(A, B):                                   # rotation R, translation t with B = R A + t
        ca, cb = A.mean(0), B.mean(0)
        U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
        D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
        R = Vt.T @ D @ U.T
        return R, cb - R @ ca
    Rp, tp = fit(pts(s0), pts(sp)); Rm, tm = fit(pts(s0), pts(sm))
    def vel(p):
        p = np.asarray(p, float)
        return ((Rp @ p + tp) - (Rm @ p + tm)) / (2 * h)
    W = (Rp - Rm) / (2 * h)                          # skew: angular velocity per mm
    omega = np.array([W[2, 1], W[0, 2], W[1, 0]])
    return s0, vel, omega


@pytest.mark.parametrize("name", CORNERS)
def test_pushrod_force_equals_virtual_work(name):
    """Rigid links, one compliant member: the pushrod's force times its length rate equals the power of the wheel load
    on the wheel's material points (force at the patch, shaft force at the wheel centre, aligning torque)."""
    kin = SuspensionKinematics(_hp(name))
    s0, vel, omega = _rigid_velocity(kin)
    h = 0.02
    Lp = lambda s: np.linalg.norm(np.asarray(s.pushrod_outer) - np.asarray(kin.hp.rocker_pushrod))
    dL = (Lp(kin.solve_at_travel(h)) - Lp(kin.solve_at_travel(-h))) / (2 * h)
    for L in LOADS:
        P = (np.dot([L.Fx, L.Fy, L.Fz], vel(s0.contact_patch)) + L.Fx_wc * vel(s0.wheel_center)[0]
             + L.Mz * omega[2])
        mf = lp.solve_member_forces(kin, s0, L)
        assert mf.forces["PR"] == pytest.approx(P / dL, rel=2e-3, abs=0.5)


@pytest.mark.parametrize("name", CORNERS)
def test_forces_on_the_chassis_balance_the_wheel_load(name):
    """Sum of every force the corner puts on the chassis (pickups and pushrod, via the rocker) equals the wheel load,
    and their moment about the patch equals the load's moment."""
    kin = SuspensionKinematics(_hp(name)); s0 = kin.solve_at_travel(0.0)
    cp = np.asarray(s0.contact_patch, float)
    pts = {"upper_front_inner": kin.hp.upper_front_inner, "upper_rear_inner": kin.hp.upper_rear_inner,
           "lower_front_inner": kin.hp.lower_front_inner, "lower_rear_inner": kin.hp.lower_rear_inner,
           "tie_rod_inner": kin.hp.tie_rod_inner}
    for L in LOADS:
        mf = lp.solve_member_forces(kin, s0, L)
        F = sum(mf.pickup_forces.values()) - mf.forces["PR"] * mf.axes["PR"]
        M = sum(np.cross(np.asarray(pts[k], float) - cp, v) for k, v in mf.pickup_forces.items())
        M = M + np.cross(np.asarray(kin.hp.rocker_pushrod, float) - cp, -mf.forces["PR"] * mf.axes["PR"])
        Mload = lp._applied_moment(s0, L, cp)
        assert np.allclose(F, L.force(), atol=1e-6)
        assert np.allclose(M, Mload, atol=1e-3)


def test_outer_wheel_cornering_force_points_to_the_turn_centre():
    """The declared cornering case is the loaded outer wheel: its ground force is inboard (Fy < 0) and its aligning
    torque is the trail moment t*Fy with the trail behind the patch (Mz < 0). Vehicle equilibrium: the tires'
    lateral forces sum to m*a toward the centre, the outer wheel carrying the larger share."""
    c = {x.name: x for x in vehicle_load_cases(axle="front", brake_bias_front=0.73)}["1.5g corner"]
    assert c.Fy < 0 and c.mz_Nmm < 0
    s = tu.tire_state(0.0, 1.5)                       # left turn: the right tires are outer
    total = sum(s[t]["Fy"] for t in tu.TIRES)
    assert total == pytest.approx(300.0 * 9.81 * 1.5, rel=1e-6)
    assert s["FR"]["Fz"] > s["FL"]["Fz"] and s["FR"]["Fy"] > s["FL"]["Fy"] > 0      # toward +y_vehicle (left)
    trail_mm = 26.3                                  # 50 N*m / 1902 N, behind the patch (+x, rearward)
    Mz = np.cross([trail_mm, 0.0, 0.0], [0.0, c.Fy, 0.0])[2]
    assert np.sign(Mz) == np.sign(c.mz_Nmm)


def test_beam_element_closed_forms():
    """Cantilever tip deflection P L^3 / 3EI, axial P L / EA and twist T L / GJ, to machine precision."""
    s = ff.Section(25.4, 1.65)
    for L in (100.0, 750.0):
        fr = ff.Frame(); a = fr.add_node([0, 0, 0]); b = fr.add_node([L, 0, 0]); fr.add_elem(a, b, s)
        u = fr.solve({b: [100.0, 200.0, 300.0, 5e4, 0, 0]}, {a: range(6)})
        assert u[b, 0] == pytest.approx(100.0 * L / (fr.E * s.A), rel=1e-10)
        assert u[b, 1] == pytest.approx(200.0 * L ** 3 / (3 * fr.E * s.I), rel=1e-10)
        assert u[b, 2] == pytest.approx(300.0 * L ** 3 / (3 * fr.E * s.I), rel=1e-10)
        assert u[b, 3] == pytest.approx(5e4 * L / (fr.G * s.J), rel=1e-10)


def test_chassis_model_rigid_modes_and_reciprocity():
    import frame_derivation as fd
    fr, rep = fd.chassis_v10()
    w = np.linalg.eigvalsh(fr.stiffness())
    assert int(np.sum(np.abs(w) < 1e-3)) == 6 and w[6] > 1e-2          # six rigid-body modes, no mechanism
    mass = fr.lumped_mass(10.0)
    C = fr.inertia_relief_flexibility([5, 120, 300], mass)
    assert np.abs(C - C.T).max() <= 1e-9 * np.abs(C).max()            # Maxwell-Betti
    assert np.all(np.linalg.eigvalsh(0.5 * (C + C.T)) > 0)


def test_free_frame_load_is_balanced_by_inertia():
    """A load on the free frame is balanced by its own inertia: no reaction at the three pinning nodes."""
    import frame_derivation as fd
    fr, rep = fd.chassis_v10()
    mass = fr.lumped_mass(200.0)
    U, res, reaction = fd.solve_free(fr, mass, {10: np.array([0, 0, 1000.0, 0, 0, 0]), 200: np.array([500.0, 0, 0, 0, 0, 0])})
    assert reaction < 1e-6
