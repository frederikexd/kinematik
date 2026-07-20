# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Tests for SimulForge, the unified mechatronic co-solver
(suspension/simulforge.py).

Philosophy: pin the closed forms the electrical block must reproduce (the
exact RL update, the bus algebraic constraint, torque/current conversion),
pin the physical directions of the couplings (an active bar reduces roll; a
dead bar doesn't; series resistance sags the bus; sag costs authority), pin
the co-sim bookkeeping (trace shapes, energy monotonicity, brownout latch),
and exercise the linter across its Ghost / Fusebox / Earshot ledgers,
including the never-raise contract.

Run:  python -m pytest tests/test_simulforge.py
      (or standalone: python tests/test_simulforge.py)
"""
import math
import sys, os

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from suspension import simulforge as sf
from suspension import ghost_topology as gt
from suspension import earshot as es
from suspension import transient as tr
from suspension.kinematics import Hardpoints


# --------------------------------------------------------------------------- #
#  Closed forms
# --------------------------------------------------------------------------- #
def test_torque_current_roundtrip():
    a = sf.ActuatorParams()
    for M in (0.0, 120.0, -450.0):
        i = a.current_for_torque(M)
        assert a.torque_from_current(i) == pytest.approx(M, abs=1e-9)


def test_winding_tau():
    a = sf.ActuatorParams(R_ohm=0.5, L_H=1.0e-3)
    assert a.tau_ms == pytest.approx(2.0)


def test_degradation_never_mutates_and_applies():
    bus, act = sf.BusParams(), sf.ActuatorParams()
    r0, m0 = bus.R_int_ohm, act.M_max_Nm
    d = sf.degradation_presets()["sagging_pack"]
    b2, a2 = d.apply(bus, act)
    assert bus.R_int_ohm == r0 and act.M_max_Nm == m0      # inputs untouched
    assert b2.R_int_ohm == pytest.approx(3.0 * r0)
    assert b2.V_oc == pytest.approx(bus.V_oc - 1.5)
    dead = sf.degradation_presets()["dead_actuator"]
    _, a3 = dead.apply(bus, act)
    assert a3.M_max_Nm == 0.0 and a3.duty_max == 0.0


# --------------------------------------------------------------------------- #
#  The coupled run — directions of the physics
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="module")
def runs():
    nom = sf.run_simulforge(None, kind="step_steer", degradation="nominal")
    dead = sf.run_simulforge(None, kind="step_steer",
                             degradation="dead_actuator")
    cor = sf.run_simulforge(None, kind="step_steer",
                            degradation="corroded_connector")
    return nom, dead, cor


def test_traces_align(runs):
    nom, _, _ = runs
    assert nom.ok
    assert nom.elec.t.size == nom.mech.t.size
    assert nom.elec.M_act.shape == (nom.mech.t.size, 2)
    assert np.all(np.isfinite(nom.elec.V_bus))


def test_active_bar_reduces_roll(runs):
    nom, dead, _ = runs
    assert nom.roll_peak_deg() < dead.roll_peak_deg()


def test_dead_actuator_draws_no_torque(runs):
    _, dead, _ = runs
    assert float(np.max(np.abs(dead.elec.M_act))) == pytest.approx(0.0)
    assert dead.elec.summary()["energy_Wh"] > 0.0     # quiescent still billed


def test_series_resistance_sags_the_bus(runs):
    nom, _, cor = runs
    assert cor.elec.summary()["v_min"] < nom.elec.summary()["v_min"]


def test_energy_monotone(runs):
    nom, _, _ = runs
    assert np.all(np.diff(nom.elec.E_Wh) >= -1e-12)


def test_bus_algebraic_constraint_holds(runs):
    """V_bus = V_oc − I·R at every logged step (the index-1 constraint)."""
    nom, _, _ = runs
    b = nom.bus
    # skip t=0 (logged before the first electrical step, at quiescent draw)
    v_pred = np.maximum(b.V_oc - nom.elec.I_bus[1:] * b.r_total_ohm, 0.0)
    assert np.allclose(nom.elec.V_bus[1:], v_pred, atol=1e-9)


def test_brownout_latch_forces_passive():
    """A hopeless bus must brown out, and the bar must go quiet offline."""
    d = sf.Degradation(key="x", label="x", story="x",
                       r_harness_add_ohm=0.5, v_brownout_add=3.0)
    res = sf.run_simulforge(None, kind="step_steer", degradation=d)
    assert res.ok
    assert res.elec.n_brownouts >= 1
    off = ~res.elec.online
    assert off.any()
    assert float(np.max(np.abs(res.elec.M_cmd[off]))) == pytest.approx(0.0)


def test_never_raises_on_bad_maneuver():
    bad = sf.run_simulforge(None, kind="teleport")
    assert not bad.ok and bad.warnings


# --------------------------------------------------------------------------- #
#  The linter — the three ledgers
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="module")
def ghost_corner():
    p = tr.TransientParams.from_vehicle(None)
    Fz_static = float(tr.TransientSolver().static_corner_loads()[1])
    return gt.GhostCorner.uniform_tube(
        Hardpoints.default(), wheel_rate_N_per_mm=p.k_wheel_front / 1000.0,
        Fz_static_N=Fz_static, track_mm=p.track_front * 1000.0)


def test_lint_basic(runs):
    nom, _, cor = runs
    lint = sf.forge_lint(nom, cor)
    assert lint.verdict in {v.value for v in sf.ForgeVerdict}
    codes = {f.code for f in lint.findings}
    assert "GHOST_ABSENT" in codes          # coupling absence is reported
    md = sf.render_forge_md(lint)
    assert "SimulForge lint" in md and lint.verdict in md


def test_lint_full_couplings(runs, ghost_corner):
    nom, _, cor = runs
    ab = es.ABDesign(effect_predicted=0.3, noise_sigma=0.8)
    lint = sf.forge_lint(nom, cor, gc=ghost_corner, corner="FR",
                         ab_design=ab)
    assert lint.ghost_nominal is not None and lint.ghost_degraded is not None
    assert lint.path_audit_degraded is not None
    # electrical racers are in the pecking order alongside the members
    keys = set(lint.path_audit_degraded.probs)
    assert {"branch_fuse", "branch_conn"} <= keys
    assert any(k not in ("branch_fuse", "branch_conn") for k in keys)
    assert lint.session_before is not None and lint.session_after is not None


def test_lint_of_failed_runs_never_raises():
    bad = sf.run_simulforge(None, kind="teleport")
    lint = sf.forge_lint(bad, bad)
    assert isinstance(lint.verdict, str)
    assert lint.note                          # says what it couldn't judge


def test_verdict_ordering_is_worst_first():
    order = [v.value for v in sf.ForgeVerdict]
    assert order.index("STRUCTURAL_REGRESSION") < order.index("BROWNOUT")
    assert order.index("BROWNOUT") < order.index("RESPONSE_LAGGED")
    assert order[-1] == "COUPLED_FAITHFUL"


def test_electrical_path_elements_currency(runs):
    """Capacity / peak-demand is the same FoS currency the members use."""
    nom, _, _ = runs
    els = sf.electrical_path_elements(nom)
    i_pk = float(np.max(np.abs(nom.elec.i_act)))
    by_key = {e.key: e for e in els}
    assert by_key["branch_fuse"].fos == pytest.approx(
        nom.actuator.fuse_rating_A / i_pk)
    assert by_key["branch_fuse"].severity.value == "S1"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
