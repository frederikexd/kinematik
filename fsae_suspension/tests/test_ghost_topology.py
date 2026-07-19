# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Tests for the Ghost Topology engine (suspension/ghost_topology.py).

Same philosophy as the compliance suite: nail the closed forms the engine
must reproduce (Euler Pcr, yield FoS), pin the sign conventions and the
verdict boundaries, and exercise the couplings — the tyre-feedback fixed
point (contraction, geometric-series limit, measured divergence), the
load-path shift bookkeeping, the transient-result sign mapping, and the
solve cache.

Run:  python -m pytest tests/test_ghost_topology.py
      (or standalone: python tests/test_ghost_topology.py)
"""
import math
import sys, os

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from suspension.kinematics import Hardpoints
from suspension.compliance import CompliantCorner
from suspension import loadpath as lp
from suspension.flex import MATERIALS
from suspension.ghost_topology import (
    MemberSection, uniform_sections, TireSensitivity, GhostCorner,
    GhostThresholds, ghost_audit, ghost_audit_transient, render_ghost_md,
    _rc_height_mm, VERDICTS,
)


HP = Hardpoints.default()


def _pulse(peak_g=3.0, fz_static=700.0, fz_peak=2200.0, n=201, dur=0.8):
    """Half-sine cornering pulse in corner axes (pull is −y)."""
    t = np.linspace(0.0, dur, n)
    s = np.sin(np.pi * t / dur)
    Fz = fz_static + (fz_peak - fz_static) * s
    Fy = -peak_g * Fz * s
    return t, np.zeros_like(t), Fy, Fz


def _stock_corner(**kw):
    return GhostCorner.uniform_tube(HP, wheel_rate_N_per_mm=35.0,
                                    Fz_static_N=700.0, **kw)


# --------------------------------------------------------------------------- #
#  MemberSection — the closed forms
# --------------------------------------------------------------------------- #
def test_euler_pcr_matches_formula():
    sec = MemberSection(material="Steel 4130", od_mm=19.05, wall_mm=0.9)
    L = 350.0
    E = MATERIALS["Steel 4130"].E
    assert abs(sec.euler_pcr_N(L) - math.pi**2 * E * sec.I_mm4 / L**2) < 1e-6


def test_yield_fos_is_yield_over_stress():
    sec = MemberSection(yield_MPa=460.0)
    T = 5000.0
    mg = sec.margins(T, 350.0)
    assert abs(mg["fos_yield"] - 460.0 / (T / sec.area_mm2)) < 1e-9
    assert mg["mode"] == "yield (tension)"
    assert math.isinf(mg["fos_buckle"])          # tension can't buckle


def test_compression_governed_by_buckling_when_slender():
    # A long thin strut: Euler must govern, and the governing FoS = Pcr/|T|.
    sec = MemberSection(od_mm=10.0, wall_mm=0.7)
    L = 600.0
    T = -0.5 * sec.euler_pcr_N(L)                # half the critical load
    mg = sec.margins(T, L)
    assert mg["mode"] == "buckling"
    assert abs(mg["fos"] - 2.0) < 1e-9


def test_zero_load_fos_is_infinite():
    mg = MemberSection().margins(0.0, 350.0)
    assert math.isinf(mg["fos"])


# --------------------------------------------------------------------------- #
#  Roll-centre construction
# --------------------------------------------------------------------------- #
def test_rc_height_geometry():
    # Synthetic state: CP at (y=600, z=0), IC at (y=-2400, z=150).
    # Line to y=0: z = 0 + (0-600)·(150-0)/(-2400-600) = 30 mm.
    class S:
        contact_patch = np.array([0.0, 600.0, 0.0])
        instant_center = np.array([-2400.0, 150.0])
    assert abs(_rc_height_mm(S, 1200.0) - 30.0) < 1e-9


# --------------------------------------------------------------------------- #
#  TireSensitivity — signs of the feedback paths
# --------------------------------------------------------------------------- #
def test_representative_sensitivity_signs():
    load = lp.WheelLoad(Fy=-4000.0, Fz=2000.0)       # loaded outer, pull −y
    s = TireSensitivity.representative(load)
    # camber loss (Δcamber > 0) must shed grip: dFy > 0 (toward zero)
    assert s.dFy(+1.0, 0.0) > 0
    # toe-out (Δtoe > 0) must grow slip: dFy < 0 (further negative)
    assert s.dFy(0.0, +1.0) < 0
    # magnitudes scale with Fz
    s_half = TireSensitivity.representative(lp.WheelLoad(Fy=-2000.0, Fz=1000.0))
    assert abs(s_half.dFy_dtoe_N_per_deg) == pytest.approx(
        abs(s.dFy_dtoe_N_per_deg) / 2.0)


# --------------------------------------------------------------------------- #
#  solve_instant — the closed loop
# --------------------------------------------------------------------------- #
def test_zero_feedback_is_open_loop():
    gc = _stock_corner()
    load = lp.WheelLoad(Fy=-4000.0, Fz=2000.0)
    g = gc.solve_instant(load, tire=TireSensitivity())     # both sensitivities 0
    assert g.feedback_converged
    assert g.loop_gain == 0.0
    assert g.load.Fy == pytest.approx(load.Fy)
    assert g.feedback_dFy_N == pytest.approx(0.0)


def test_feedback_fixed_point_is_self_consistent():
    """At the converged closed loop, Fy_closed ≈ Fy_open + dFy(geometry(Fy_closed))."""
    gc = _stock_corner()
    load = lp.WheelLoad(Fy=-5000.0, Fz=2200.0)
    tire = TireSensitivity.representative(load)
    g = gc.solve_instant(load, tire=tire)
    assert g.feedback_converged
    dFy = tire.dFy(g.compliance.compliance_camber, g.compliance.compliance_toe)
    assert g.load.Fy == pytest.approx(load.Fy + dFy, abs=2.0)
    assert abs(g.loop_gain) < 1.0


def test_feedback_divergence_is_measured_and_flagged():
    """An absurd destabilising toe sensitivity must push |gain| ≥ 1 and be
    reported as divergence, with the instant returned at OPEN loop."""
    cc = CompliantCorner.uniform_tube(HP, material="Aluminium 6061",
                                      od_mm=12.0, wall_mm=1.0, k_tab=1500.0)
    gc = GhostCorner(cc, uniform_sections())
    load = lp.WheelLoad(Fy=-4000.0, Fz=2000.0)
    # huge slip-reinforcing gain, no stabilising camber term
    tire = TireSensitivity(dFy_dcamber_N_per_deg=0.0,
                           dFy_dtoe_N_per_deg=-30000.0)
    g = gc.solve_instant(load, tire=tire)
    assert not g.feedback_converged
    assert abs(g.loop_gain) >= 1.0
    assert g.load.Fy == pytest.approx(load.Fy)          # open loop, honestly
    assert any("instability" in w for w in g.warnings)


def test_stiffer_links_deflect_less():
    load = lp.WheelLoad(Fy=-5000.0, Fz=2200.0)
    soft = GhostCorner(CompliantCorner.uniform_tube(
        HP, material="Aluminium 6061", od_mm=12.0, wall_mm=1.0),
        uniform_sections())
    stiff = _stock_corner()
    g_soft = soft.solve_instant(load, tire=TireSensitivity())
    g_stiff = stiff.solve_instant(load, tire=TireSensitivity())
    assert abs(g_soft.d_camber) > abs(g_stiff.d_camber)
    assert abs(g_soft.d_toe) > abs(g_stiff.d_toe)


def test_load_path_shift_bookkeeping():
    """Rigid and ghost load paths solve the SAME closed-loop load; the shift
    entries must be self-consistent and the share shifts must sum ~0."""
    gc = _stock_corner()
    g = gc.solve_instant(lp.WheelLoad(Fy=-6000.0, Fz=2200.0),
                         tire=TireSensitivity())
    shares = [v["share_shift"] for v in g.load_path_shift.values()]
    assert abs(sum(shares)) < 1e-9
    for v in g.load_path_shift.values():
        assert v["delta_N"] == pytest.approx(v["ghost_N"] - v["rigid_N"])


def test_travel_baseline_uses_wheel_rate():
    gc = _stock_corner()          # k = 35 N/mm, Fz_static = 700 N
    g = gc.solve_instant(lp.WheelLoad(Fy=-1000.0, Fz=1400.0),
                         tire=TireSensitivity())
    assert g.travel_mm == pytest.approx((1400.0 - 700.0) / 35.0)
    # and the rigid baseline actually moved off static
    assert g.rigid.travel == pytest.approx(g.travel_mm, abs=0.5)


def test_travel_clamp_flags():
    gc = _stock_corner()
    g = gc.solve_instant(lp.WheelLoad(Fy=0.0, Fz=700.0 + 35.0 * 500.0),
                         tire=TireSensitivity())
    assert abs(g.travel_mm) == pytest.approx(40.0)
    assert any("clamped" in w for w in g.warnings)


# --------------------------------------------------------------------------- #
#  ghost_audit — verdicts
# --------------------------------------------------------------------------- #
def test_stock_corner_is_rigid_faithful():
    t, Fx, Fy, Fz = _pulse(peak_g=1.2, fz_peak=1400.0)
    audit = ghost_audit(_stock_corner(), t, Fx, Fy, Fz, n_samples=8,
                        tire=TireSensitivity())
    assert audit.verdict == "RIGID_FAITHFUL"
    assert audit.flags == []
    assert all(g.min_fos > 1.5 for g in audit.instants)


def test_soft_corner_breaches_margin_and_degrades():
    t, Fx, Fy, Fz = _pulse(peak_g=3.0)
    cc = CompliantCorner.uniform_tube(HP, material="Aluminium 6061",
                                      od_mm=12.0, wall_mm=1.0, k_tab=1500.0)
    gc = GhostCorner(cc, uniform_sections(od_mm=12.0, wall_mm=1.0,
                                          material="Aluminium 6061",
                                          yield_MPa=276.0),
                     wheel_rate_N_per_mm=35.0, Fz_static_N=700.0)
    audit = ghost_audit(gc, t, Fx, Fy, Fz, n_samples=8)
    assert "MARGIN_BREACHED" in audit.flags
    assert audit.verdict == "MARGIN_BREACHED"
    worst = min(audit.instants, key=lambda g: g.min_fos)
    # margin worst instant must be at (or adjacent to) the load peak
    assert abs(worst.t - t[np.argmax(np.abs(Fy))]) < 0.15


def test_compliance_inverted_when_deflection_beats_intent():
    """Links soft enough that Δcamber exceeds the −1.5° static intent must
    produce the headline verdict: the loaded wheel goes POSITIVE."""
    t, Fx, Fy, Fz = _pulse(peak_g=3.0)
    cc = CompliantCorner.uniform_tube(HP, material="Aluminium 6061",
                                      od_mm=10.0, wall_mm=0.7, k_tab=400.0)
    gc = GhostCorner(cc, uniform_sections(od_mm=10.0, wall_mm=0.7,
                                          material="Aluminium 6061",
                                          yield_MPa=276.0))
    audit = ghost_audit(gc, t, Fx, Fy, Fz, n_samples=8, tire=TireSensitivity())
    assert "COMPLIANCE_INVERTED" in audit.flags
    inv = [f for f in audit.findings if f["check"] == "kinematic-intent inversion"]
    assert inv and inv[0]["detail"]["rigid_deg"] * inv[0]["detail"]["ghost_deg"] < 0
    # inversion outranks margin in the governing verdict
    assert VERDICTS.index(audit.verdict) <= VERDICTS.index("MARGIN_BREACHED")


def test_empty_history_is_honest():
    audit = ghost_audit(_stock_corner(), [], [], [], [])
    assert audit.verdict == "RIGID_FAITHFUL"
    assert "nothing audited" in audit.note


def test_cache_reuses_identical_loads():
    t = np.linspace(0, 1, 101)
    Fy = np.full_like(t, -4000.0)                 # constant load event
    Fz = np.full_like(t, 2000.0)
    audit = ghost_audit(_stock_corner(), t, np.zeros_like(t), Fy, Fz,
                        n_samples=10, tire=TireSensitivity())
    assert audit.n_solves == 1
    assert audit.n_cache_hits == len(audit.instants) - 1
    # cached instants must carry their OWN timestamps
    assert audit.trace("t")[0] != audit.trace("t")[-1]


def test_fast_edge_flags_quasi_static_limit():
    # a 2 ms step edge inside the event must trip the slew warning
    t = np.linspace(0, 0.2, 2001)
    Fy = np.where(t < 0.1, -500.0, -5000.0)
    Fz = np.full_like(t, 2000.0)
    audit = ghost_audit(_stock_corner(), t, np.zeros_like(t), Fy, Fz,
                        n_samples=6, tire=TireSensitivity())
    assert any(f["check"] == "quasi-static validity" for f in audit.findings)


# --------------------------------------------------------------------------- #
#  Transient wiring
# --------------------------------------------------------------------------- #
def test_transient_sign_mapping():
    """FR (right side) flips both Fx and Fy; FL flips only Fx."""
    class FakeResult:
        ok = True
        t = np.linspace(0, 0.1, 11)
        Fx = np.tile([[100.0, 100.0, 0.0, 0.0]], (11, 1))
        Fy = np.tile([[2000.0, 3000.0, 0.0, 0.0]], (11, 1))   # body y-left+
        Fz = np.tile([[1500.0, 1500.0, 1500.0, 1500.0]], (11, 1))
        warnings = []
    gc = _stock_corner()
    a_fr = ghost_audit_transient(gc, FakeResult(), corner="FR", n_samples=2,
                                 tire=TireSensitivity())
    a_fl = ghost_audit_transient(gc, FakeResult(), corner="FL", n_samples=2,
                                 tire=TireSensitivity())
    assert a_fr.instants[0].load.Fy == pytest.approx(-3000.0)   # y flipped
    assert a_fl.instants[0].load.Fy == pytest.approx(+2000.0)   # mirror keeps sign
    assert a_fr.instants[0].load.Fx == pytest.approx(-100.0)    # x always flips


def test_failed_transient_refuses_audit():
    from suspension.transient import TransientResult
    res = TransientResult.failed(["synthetic blow-up"])
    audit = ghost_audit_transient(_stock_corner(), res, corner="FR")
    assert audit.instants == []
    assert "flagged failed" in audit.note


def test_bad_corner_name_raises():
    with pytest.raises(ValueError):
        ghost_audit_transient(_stock_corner(), object(), corner="XX")


def test_end_to_end_with_real_transient():
    """The full chain: transient step-steer → ghost audit on the loaded corner."""
    from suspension.transient import TransientSolver, step_steer_maneuver
    solver = TransientSolver()
    drv, road, _, u0, _label = step_steer_maneuver(steer_deg=4.0, t_step=0.1)
    res = solver.run(0.8, driver=drv, road=road, u0=u0)
    assert res.ok
    gc = _stock_corner()
    # left turn (+steer) loads the RIGHT side: audit FR
    audit = ghost_audit_transient(gc, res, corner="FR", n_samples=6)
    assert audit.verdict in VERDICTS
    assert len(audit.instants) >= 6
    md = render_ghost_md(audit)
    assert "Ghost Topology audit" in md and audit.verdict in md


# --------------------------------------------------------------------------- #
#  Report
# --------------------------------------------------------------------------- #
def test_render_md_contains_findings_and_instants():
    t, Fx, Fy, Fz = _pulse(peak_g=2.0)
    audit = ghost_audit(_stock_corner(), t, Fx, Fy, Fz, n_samples=4,
                        tire=TireSensitivity())
    md = render_ghost_md(audit)
    assert "| t (ms) |" in md
    assert "Findings" in md
    assert "not a substitute" in md


if __name__ == "__main__":
    import subprocess
    raise SystemExit(subprocess.call(
        [sys.executable, "-m", "pytest", os.path.abspath(__file__), "-v"]))
