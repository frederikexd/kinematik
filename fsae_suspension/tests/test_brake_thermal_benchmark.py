"""The through-thickness rotor model against Ansys VM110 and the exact series solution (scripts/brake_thermal_benchmark.py).

The registered run fails against the manual's graphical targets (one depth by 11.6 F, heat by 2.7 %). These tests pin
that the stored result says so, and that the model stays within 2 F and 0.5 % of the exact solution of the same slab.
"""
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


def test_the_stored_run_is_reported_as_it_came_out():
    res = json.loads((ROOT / "benchmarks" / "brake_thermal" / "results.json").read_text())
    assert res["passed"] is False                     # against the registered graphical targets
    assert res["heat"]["passes"] is False


def test_exact_series_solution():
    import brake_thermal_benchmark as bb
    ex = bb.exact()
    assert ex["temps_F"]["0.0"] == pytest.approx(510.2, abs=0.2)
    assert ex["temps_F"]["1.0"] == pytest.approx(1432.6, abs=0.2)
    assert ex["heat_in_btu_ft2"] == pytest.approx(21228.0, rel=1e-3)


def test_live_run_matches_the_exact_solution():
    import brake_thermal_benchmark as bb
    res = bb.run()
    for row in res["exact"]["temperatures"].values():
        assert abs(row["kinematik_diff_F"]) < 2.0
    assert abs(res["exact"]["kinematik_heat_rel"]) < 0.005


def test_steady_face_flux_gives_the_exact_gradient_and_keeps_energy():
    """Heated face, insulated centre, no losses: once the profile settles the gradient is q L / 2k exactly, and
    the stored heat equals the heat put in. The missing half-cell factor broke both (twice the gradient, up to
    twice the energy)."""
    import numpy as np
    sys.path.insert(0, str(ROOT))
    from suspension.brake_thermal import OneDRotor, TwoNodeParams
    p = TwoNodeParams(enable_radiation=False)
    for n in (6, 24):
        od = OneDRotor(p, n_nodes=n)
        P = 45000.0
        r = od.simulate([P] * 200, 0.02, h_w_m2K=0.0)
        q = P * (1 - p.pad_heat_fraction) / 2 / od.face_area
        T = np.array(r.T_field_c) - p.T_ambient_c
        w = np.full(n, od.dx)
        w[0] = w[-1] = od.dx / 2
        assert od.mat.rho_kg_m3 * od.mat.cp_j_kgK * float(w @ T) == pytest.approx(q * 4.0, rel=1e-6)
        assert T[0] - T[-1] == pytest.approx(q * od.half_t / (2 * od.mat.k_w_mK), rel=0.01)
