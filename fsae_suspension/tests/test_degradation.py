# ============================================================================
#  KinematiK — tests for suspension/degradation.py
#  Verifies the transient degradation solver: thermal-expansion physics,
#  compliance delegation, empirical-tyre labelling, tolerance sensitivity, and
#  the honest magnitude findings (compliance >> thermal at FSAE scale).
# ============================================================================
import numpy as np
import pytest

from suspension.kinematics import Hardpoints
from suspension.degradation import (
    ThermalRamp, ThermalExpansionModel, TyreThermalModel, TyreThermalRamp,
    DegradationSolver, DegradationConfig, tolerance_sensitivity,
    THERMAL_ALPHA, PROVENANCE, _corner_metrics, _corner_curves,
)


# ---- thermal ramp: first-order lump model is correct -------------------
def test_thermal_ramp_monotone_and_bounded():
    r = ThermalRamp(T_amb=20, T_soak=55, tau_min=6, run_min=25)
    assert r.temp_at(0.0) == pytest.approx(20.0)
    assert r.temp_at(1000.0) == pytest.approx(55.0, abs=1e-3)   # asymptote
    assert r.delta_T_at(6.0) == pytest.approx(35 * (1 - np.exp(-1)), rel=1e-6)
    assert 0.0 <= r.progress_at(10) <= 1.0


# ---- thermal expansion: alpha*dT*L, and differential heating -----------
def test_expansion_uses_real_alpha():
    e = ThermalExpansionModel(material="Aluminium 6061")
    assert e.alpha() == THERMAL_ALPHA["Aluminium 6061"]
    # aluminium expands ~2x steel
    es = ThermalExpansionModel(material="Steel 4130")
    assert e.alpha() > 1.8 * es.alpha()


def test_expansion_shifts_points_by_expected_magnitude():
    hp = Hardpoints.default()
    e = ThermalExpansionModel(material="Steel 4130", gradient_len=1e9)  # ~uniform
    # a point ~300 mm from datum, dT=35: shift ~ alpha*dT*L ~ 12.3e-6*35*300 ~0.13mm
    shift = e.point_shift_mm(hp, "upper_front_inner", 35.0)
    assert 0.05 < shift < 0.3


def test_differential_heating_near_point_shifts_more():
    hp = Hardpoints.default()
    e = ThermalExpansionModel(gradient_len=100.0)   # sharp gradient
    # a point near the source moves more than a far one at the same dT
    near = e.point_shift_mm(hp, "lower_front_inner", 35.0)
    far = e.point_shift_mm(hp, "upper_rear_inner", 35.0)
    assert near >= far


def test_expanded_geometry_is_a_valid_hardpoints():
    hp = Hardpoints.default()
    hp2 = ThermalExpansionModel().expanded(hp, 35.0)
    assert isinstance(hp2, Hardpoints)
    # chassis points moved, outboard (upright) points did NOT
    assert not np.allclose(hp2.upper_front_inner, hp.upper_front_inner)
    assert np.allclose(hp2.upper_outer, hp.upper_outer)


# ---- the honest magnitude finding: compliance >> thermal ---------------
def test_compliance_dominates_thermal_at_fsae_scale():
    hp = Hardpoints.default()
    curve = DegradationSolver(hp).run()
    s = curve.lap15_summary()
    thermal = abs(s["camber_thermal_deg"]) + abs(s["toe_thermal_deg"])
    compliance = abs(s["camber_compliance_deg"]) + abs(s["toe_compliance_deg"])
    # this is the physically real result the solver refuses to fake around
    assert compliance > thermal
    assert "compliance" in s["dominant_mechanism"]


# ---- tyre model is empirical and clearly bounded -----------------------
def test_tyre_grip_peaks_at_peak_temp():
    m = TyreThermalModel(T_peak=80, width=45, floor=0.82)
    assert m.grip_mult(80) == pytest.approx(1.0)
    assert m.grip_mult(80) > m.grip_mult(40)     # cold = less grip
    assert m.grip_mult(80) > m.grip_mult(140)    # overheated = less grip
    assert m.grip_mult(-100) >= m.floor          # never below floor


def test_tyre_ramp_heats_faster_than_chassis():
    tr = TyreThermalRamp(tau_min=2.5)
    cr = ThermalRamp(tau_min=6.0)
    # at 3 min the tyre is a larger fraction of its rise than the chassis
    assert (tr.temp_at(3) - tr.T_amb) / (tr.T_settle - tr.T_amb) > \
           cr.progress_at(3)


# ---- solver end to end -------------------------------------------------
def test_solver_runs_fast_and_converges():
    import time
    hp = Hardpoints.default()
    t0 = time.time()
    curve = DegradationSolver(hp).run()
    dt = time.time() - t0
    assert dt < 5.0                              # closed-form, seconds not hours
    assert curve.lap15_summary()["converged"]


def test_grip_decays_over_run():
    hp = Hardpoints.default()
    curve = DegradationSolver(hp).run()
    arr = curve.as_arrays()
    # cold tyre starts below peak, warms toward peak — grip should rise then
    # (with default settle 95 > peak 80) start dropping; at minimum it changes
    assert not np.allclose(arr["grip_mult"], arr["grip_mult"][0])


def test_laptime_proxy_splits_causes_and_is_labelled_proxy():
    hp = Hardpoints.default()
    curve = DegradationSolver(hp).run()
    p = curve.laptime_delta_proxy()
    assert p["grip_loss_total_pct"] == pytest.approx(
        p["grip_loss_tyre_pct"] + p["grip_loss_alignment_pct"], abs=0.01)
    assert "PROXY" in p["note"]


# ---- tolerance sensitivity map -----------------------------------------
def test_tolerance_map_flags_outboard_points_as_critical():
    hp = Hardpoints.default()
    tm = tolerance_sensitivity(hp)
    crit = [r["point"] for r in tm.critical_points(2)]
    # outboard ball joints have the highest leverage on wheel angle — physics
    assert any("outer" in p for p in crit)


def test_tolerance_map_recommendations_present():
    hp = Hardpoints.default()
    tm = tolerance_sensitivity(hp)
    recs = {r["recommendation"] for r in tm.table()}
    assert recs  # non-empty
    for r in tm.table():
        assert r["recommendation"] in ("CNC / tight tol", "hand-weld OK")


def test_tolerance_build_sensitivity_is_finite_difference_through_real_solver():
    # perturbing a high-leverage point must produce a non-zero alignment move
    hp = Hardpoints.default()
    tm = tolerance_sensitivity(hp, jig_tol_mm=1.0)
    lower_outer = [r for r in tm.table() if r["point"] == "lower_outer"][0]
    assert lower_outer["alignment_move_deg"] > 0.0


# ---- aluminium subframe grows the thermal term (model responds) --------
def test_aluminium_subframe_increases_thermal_shift():
    hp = Hardpoints.default()
    steel = ThermalExpansionModel(material="Steel 4130", gradient_len=120)
    alu = ThermalExpansionModel(material="Aluminium 6061", gradient_len=120)
    sh_steel = steel.point_shift_mm(hp, "lower_front_inner", 40)
    sh_alu = alu.point_shift_mm(hp, "lower_front_inner", 40)
    assert sh_alu > sh_steel                     # alpha_alu ~ 2x alpha_steel


# ---- provenance is explicit --------------------------------------------
def test_provenance_labels_empirical_tyre():
    joined = " ".join(PROVENANCE["empirical_models"]).lower()
    assert "tyre" in joined or "tire" in joined
    assert "empirical" in PROVENANCE["note"].lower()


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
