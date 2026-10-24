# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""powertrain/engine.py against closed forms: the launch march, the gear sweep, pipe and junction hydraulics, the
pump operating point, the radiator's effectiveness-NTU and the coolant temperature march, and the ledger coupling.
SI units throughout (m, s, N, W, Pa, kg, K), as the module uses."""
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from scipy.optimize import brentq

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from powertrain import engine as e                                 # noqa: E402
from suspension import interfaces as itf                           # noqa: E402

FL = e.CoolantProps()


def _ideal(**kw):
    """A car with no drag, no rolling resistance, no spin-up inertia and a constant-torque motor: a closed-form
    constant-acceleration problem."""
    motor = e.MotorCurve(rpm=np.array([0.0, 20000.0]), torque_nm=np.array([100.0, 100.0]), redline_rpm=20000.0)
    base = dict(motor=motor, tire_radius_m=0.2, gear_ratio=4.0, mass_kg=250.0, drivetrain_eff=0.9, mu=10.0,
                cda_m2=0.0, crr=0.0, rot_inertia_factor=1.0, cg_height_m=0.0)
    base.update(kw)
    return e.DrivetrainParams(**base)


# --------------------------------------------------------------------------- #
#  Motor curve
# --------------------------------------------------------------------------- #
def test_peak_curve_is_constant_torque_then_constant_power_then_nothing_past_redline():
    m = e.MotorCurve.from_peak(180.0, 80.0, 6500.0)
    w_base = 80e3 / 180.0
    rpm_base = w_base * 60 / (2 * np.pi)
    assert m.torque_at(0.5 * rpm_base) == pytest.approx(180.0)
    for rpm in (1.2 * rpm_base, 5000.0, 6400.0):
        assert float(m.torque_at(rpm)) * rpm * 2 * np.pi / 60 == pytest.approx(80e3, rel=2e-2)
    assert m.torque_at(6600.0) == 0.0


def test_the_current_limit_caps_torque_at_kt_times_i_max():
    m = e.MotorCurve.from_peak(180.0, 80.0, 6500.0, kt_nm_per_a=0.5, i_max_a=200.0)
    assert float(m.torque_at(1000.0)) == pytest.approx(100.0)


# --------------------------------------------------------------------------- #
#  Launch march
# --------------------------------------------------------------------------- #
def test_constant_force_launch_matches_uniform_acceleration():
    """F = T*G*eta/r on a 250 kg car with no losses: a is constant, x = a t^2 / 2."""
    p = _ideal()
    a = 100.0 * 4.0 * 0.9 / 0.2 / 250.0
    r = e.simulate_launch(p, distance_m=75.0, dt=1e-3)
    assert np.allclose(r.a_ms2, a)
    assert r.time_to_distance_s == pytest.approx(np.sqrt(2 * 75.0 / a), abs=3e-3)
    assert r.v_final_kmh / 3.6 == pytest.approx(a * r.time_to_distance_s, abs=0.02)
    assert not r.traction_limited.any() and r.current_limited_frac == 0.0


def test_traction_limit_with_load_transfer_settles_at_its_closed_form():
    """Traction-limited RWD: a = mu*(g*w + a*h/L), so a = mu*g*w / (1 - mu*h/L)."""
    p = _ideal(mu=1.4, driven_weight_frac=0.52, cg_height_m=0.28, wheelbase_m=1.55, motor=e.MotorCurve(
        rpm=np.array([0.0, 20000.0]), torque_nm=np.array([400.0, 400.0]), redline_rpm=20000.0))
    r = e.simulate_launch(p, distance_m=20.0, dt=1e-3)
    a = 1.4 * e.G * 0.52 / (1 - 1.4 * 0.28 / 1.55)
    assert r.traction_limited[5:].all()
    assert r.a_ms2[-1] == pytest.approx(a, rel=1e-6)
    assert r.a_ms2[0] == pytest.approx(1.4 * e.G * 0.52, rel=1e-9)        # no transfer before the car accelerates


def test_drag_and_rolling_resistance_bring_the_car_to_its_terminal_speed():
    """With a constant drive force, speed tends to the v at which F = rho*CdA*v^2/2 + Crr*m*g."""
    p = _ideal(cda_m2=1.1, crr=0.015)
    F = 100.0 * 4.0 * 0.9 / 0.2
    vt = np.sqrt((F - 0.015 * 250.0 * e.G) / (0.5 * e.RHO_AIR * 1.1))
    r = e.simulate_launch(p, distance_m=5000.0, dt=5e-3, t_max_s=400.0)
    assert r.v_ms[-1] == pytest.approx(vt, rel=1e-3)


def test_power_current_and_loss_are_consistent():
    """Electrical power is mechanical over efficiency, the loss is their difference, and current is torque over kt
    (or power over bus voltage without a kt)."""
    m = e.MotorCurve.from_peak(180.0, 80.0, 6500.0, kt_nm_per_a=0.61, i_max_a=250.0, eta_motor=0.9)
    p = _ideal(motor=m, mu=1.4, cg_height_m=0.28)
    r = e.simulate_launch(p, distance_m=75.0)
    w = r.motor_rpm * 2 * np.pi / 60
    mech = r.motor_torque_nm * w
    assert np.allclose(r.elec_power_w, mech / 0.9) and np.allclose(r.loss_power_w, r.elec_power_w - mech)
    assert np.allclose(r.current_a, r.motor_torque_nm / 0.61)
    assert 0.0 < r.current_limited_frac < 1.0
    no_kt = e.simulate_launch(_ideal(motor=replace(m, kt_nm_per_a=0.0)), distance_m=10.0)
    assert np.allclose(no_kt.current_a, no_kt.elec_power_w / m.voltage_v)


def test_a_zero_ratio_puts_no_torque_through():
    r = e.simulate_launch(_ideal(gear_ratio=0.0), distance_m=1.0, t_max_s=0.1)
    assert np.all(r.motor_torque_nm == 0.0) and np.all(r.v_ms == 0.0)


# --------------------------------------------------------------------------- #
#  Gear sweep
# --------------------------------------------------------------------------- #
def test_the_sweep_picks_the_fastest_ratio_and_marks_a_ratio_that_tops_out_as_dnf():
    """A ratio so tall that redline arrives at a crawl cannot finish 75 m in 30 s: time is infinite. The best
    ratio is the one with the shortest time."""
    m = e.MotorCurve.from_peak(180.0, 80.0, 6500.0)
    p = _ideal(motor=m, mu=1.4, cda_m2=1.1, crr=0.015, cg_height_m=0.28)
    s = e.optimize_gear_ratio(p, ratios=[3.0, 4.0, 5.0, 300.0], keep_traces=True)
    assert np.isinf(s.times_s[-1])
    i = int(np.argmin(s.times_s))
    assert s.best_ratio == s.ratios[i] and s.best.time_to_distance_s == s.times_s[i]
    assert len(s.runs) == 4 and s.as_table()[i]["t_s"] == s.times_s[i]
    assert s.as_table()[0]["trac_lim"].endswith("%")
    dnf = e.optimize_gear_ratio(p, ratios=[300.0, 400.0])
    assert np.all(np.isinf(dnf.times_s)) and dnf.best is not None and dnf.best.gear_ratio == 400.0


def test_the_default_sweep_covers_two_to_six():
    s = e.optimize_gear_ratio(_ideal(motor=e.MotorCurve.from_peak(180.0, 80.0, 6500.0)), distance_m=10.0)
    assert s.ratios[0] == 2.0 and s.ratios[-1] == 6.0 and len(s.ratios) == 17


# --------------------------------------------------------------------------- #
#  Hydraulics
# --------------------------------------------------------------------------- #
def test_laminar_pipe_loss_is_hagen_poiseuille():
    """Laminar: f = 64/Re, so dP = 128*mu*L*Q / (pi*D^4)."""
    pipe = e.PipeSegment(d_m=0.01, length_m=2.0)
    q = 5e-6
    a = pipe.audit(q, FL)
    assert a["reynolds"] < 2300 and a["f_darcy"] == pytest.approx(64.0 / a["reynolds"])
    assert a["dp_pa"] == pytest.approx(128 * FL.mu * 2.0 * q / (np.pi * 0.01 ** 4), rel=1e-12)


def test_turbulent_friction_is_within_haalands_accuracy_of_colebrook():
    pipe = e.PipeSegment(d_m=29e-3, length_m=1.0)
    a = pipe.audit(7e-4, FL)
    rr, re = pipe.roughness_m / pipe.d_m, a["reynolds"]
    f_c = brentq(lambda f: 1 / np.sqrt(f) + 2 * np.log10(rr / 3.7 + 2.51 / (re * np.sqrt(f))), 1e-4, 1.0)
    assert re > 4000 and a["f_darcy"] == pytest.approx(f_c, rel=0.015)
    v = 7e-4 / pipe.area_m2
    assert a["dp_pa"] == pytest.approx(a["f_darcy"] / 0.029 * 0.5 * FL.rho * v ** 2)


def test_no_flow_no_loss():
    a = e.PipeSegment(d_m=0.029, length_m=1.0, k_minor=2.0).audit(0.0, FL)
    assert a["f_darcy"] == 0.0 and a["dp_pa"] == 0.0


def test_a_junction_conserves_mass_and_uses_borda_carnot_on_an_expansion():
    j = e.YBranch(name="t", d_in_m=0.029, d_branch_m=0.040, angle_deg=45.0)
    a = j.audit(6e-4, 0.4, FL)
    assert a.mdot_run_kgs + a.mdot_branch_kgs == pytest.approx(a.mdot_in_kgs)
    ar = (0.029 / 0.040) ** 2
    head = 0.5 * FL.rho * a.v_in_ms ** 2
    assert a.dp_branch_pa == pytest.approx((1.0 + (1 - ar) ** 2) * head)
    assert a.dp_run_pa == pytest.approx(0.30 * head)
    assert j.audit(6e-4, 1.7, FL).mdot_run_kgs == 0.0                      # fraction clipped to [0, 1]
    steep = e.YBranch(name="s", d_in_m=0.029, d_branch_m=0.040, angle_deg=90.0)
    assert steep._k_branch == pytest.approx(j._k_branch * np.sqrt(2.0))
    assert a.as_dict()["name"] == "t"


def test_a_contraction_branch_adds_half_the_area_loss():
    j = e.YBranch(name="c", d_in_m=0.029, d_branch_m=0.012)
    assert j._k_branch == pytest.approx(1.0 + 0.5 * (1 - (0.012 / 0.029) ** 2))


def test_with_leg_resistance_the_split_equalises_the_two_legs_pressure_drop():
    j = e.STANDARD_Y_BRANCHES["29-12"]
    net = e.CoolingNetwork(leg_k={j.name: (20.0, 20.0)})
    q = 6e-4
    x = net._junction_split(j, q)
    a = j.audit(q, x, FL)
    a_in, a_b = np.pi * j.d_in_m ** 2 / 4, np.pi * j.d_branch_m ** 2 / 4
    dp_b = a.dp_branch_pa + 20.0 * 0.5 * FL.rho * (x * q / a_b) ** 2
    dp_r = a.dp_run_pa + 20.0 * 0.5 * FL.rho * ((1 - x) * q / a_in) ** 2
    assert 0.0 < x < 1.0 and dp_b == pytest.approx(dp_r, rel=1e-9)


def test_without_leg_resistance_the_split_falls_back_to_the_area_share():
    """With no downstream resistance the junction losses do not depend on the split, so there is no equal-loss
    point; the network then takes the area-weighted share A_b / (A_b + A_in). A model limit, pinned here."""
    j = e.STANDARD_Y_BRANCHES["29-12"]
    a_in, a_b = np.pi * j.d_in_m ** 2 / 4, np.pi * j.d_branch_m ** 2 / 4
    assert e.CoolingNetwork()._junction_split(j, 6e-4) == pytest.approx(a_b / (a_b + a_in))


def test_the_pump_meets_the_system_at_the_operating_point():
    pump = e.PumpCurve()
    assert pump.dp(0.0) == pump.dp0_pa and pump.dp(pump.q_max_m3s) == 0.0 and pump.dp(2 * pump.q_max_m3s) == 0.0
    net = e.CoolingNetwork()
    op = net.operating_point()
    assert pump.dp(op["q_m3s"]) == pytest.approx(net.system_dp(op["q_m3s"]), rel=1e-6)
    assert op["q_lpm"] == pytest.approx(op["q_m3s"] * 6e4) and op["mdot_kgs"] == pytest.approx(FL.rho * op["q_m3s"])


def test_the_audit_reports_every_segment_and_junction_at_the_operating_point():
    net = e.CoolingNetwork()
    a = net.audit()
    assert a["q_m3s"] == pytest.approx(net.operating_point()["q_m3s"])
    assert len(a["segments"]) == 2 and len(a["junctions"]) == 3
    fixed = net.audit(5e-4, fixed_split=0.25)
    assert all(jn["branch_frac"] == 0.25 for jn in fixed["junctions"])
    assert fixed["dp_total_pa"] == pytest.approx(net.system_dp(5e-4))


def test_water_is_lighter_and_holds_more_heat_than_glycol_mix():
    w = e.CoolantProps.water()
    assert w.rho < FL.rho and w.cp > FL.cp


# --------------------------------------------------------------------------- #
#  Radiator and the thermal march
# --------------------------------------------------------------------------- #
def test_radiator_rejection_obeys_its_limits():
    """Effectiveness lies between 0 and 1, so heat rejected is positive and never more than C_min*dT; no
    temperature difference, no heat; and a fan floor holds air flow with the car stopped."""
    rad = e.Radiator()
    c_cool = 0.76 * 3550.0
    for v in (0.0, 10.0, 30.0):
        q = float(rad.q_reject(90.0, 30.0, v, c_cool))
        c_min = min(float(rad.c_air(v)), c_cool)
        assert 0.0 < q < c_min * 60.0
    assert float(rad.q_reject(30.0, 30.0, 20.0, c_cool)) == 0.0
    assert float(rad.c_air(0.0)) == pytest.approx(e.RHO_AIR * rad.fan_floor_ms * rad.frontal_area_m2 * 1005.0)
    huge = e.Radiator(ua_w_per_k=1e7)
    assert float(huge.q_reject(90.0, 30.0, 20.0, c_cool)) == pytest.approx(min(float(huge.c_air(20.0)), c_cool) * 60,
                                                                           rel=1e-3)


def test_coolant_temperature_settles_where_rejection_equals_generation():
    net = e.CoolingNetwork()
    th = e.simulate_lap_thermal(net, t_s=[0.0, 10.0], v_car_ms=[15.0, 15.0], q_gen_w=[2500.0, 2500.0], n_laps=300,
                                t_amb_c=30.0, t0_c=30.0, dt=0.5)
    c_cool = net.operating_point()["mdot_kgs"] * net.fluid.cp
    t_eq = brentq(lambda T: float(net.radiator.q_reject(T, 30.0, 15.0, c_cool)) - 2500.0, 30.0, 300.0)
    assert th.t_coolant_c[-1] == pytest.approx(t_eq, abs=0.05)
    assert abs(th.steady_margin_w) < 5.0 and th.t_peak_c == pytest.approx(th.t_coolant_c.max())


def test_the_thermal_march_conserves_energy_step_by_step():
    """Explicit Euler: the heat stored, C_th*(T_end - T_0), is the sum of (generation - rejection)*dt."""
    net = e.CoolingNetwork()
    th = e.simulate_lap_thermal(net, t_s=[0.0, 5.0, 10.0], v_car_ms=[5.0, 25.0, 5.0], n_laps=4, dt=0.1,
                                coolant_volume_l=2.5, wetted_metal_kg=3.0)
    c_th = net.fluid.rho * 2.5e-3 * net.fluid.cp + 3.0 * 896.0
    stored = c_th * (th.t_coolant_c[-1] - th.t_coolant_c[0])
    assert stored == pytest.approx(np.sum((th.q_gen_w - th.q_reject_w)[:-1]) * 0.1, rel=1e-9)
    assert np.allclose(th.q_gen_w, 3000.0)                                  # default generation when none given


def test_the_launch_drives_the_thermal_march():
    p = _ideal(motor=e.MotorCurve.from_peak(180.0, 80.0, 6500.0, eta_motor=0.9), mu=1.4)
    run = e.simulate_launch(p, distance_m=75.0)
    th = e.simulate_lap_thermal(e.CoolingNetwork(), drive=run, n_laps=2, dt=0.05)
    assert th.q_gen_w.max() == pytest.approx(run.loss_power_w.max(), rel=0.05)
    over = e.simulate_lap_thermal(e.CoolingNetwork(), drive=run, q_gen_w=np.full_like(run.t_s, 1000.0), n_laps=1)
    assert np.allclose(over.q_gen_w, 1000.0)
    with pytest.raises(ValueError, match="need `drive`"):
        e.simulate_lap_thermal(e.CoolingNetwork(), t_s=[0.0, 1.0])


# --------------------------------------------------------------------------- #
#  Ledger coupling
# --------------------------------------------------------------------------- #
def _ledger():
    L = itf.IntegrationLedger()
    L.set(itf.SubsystemInterface(name="chassis", mass_kg=40.0))
    L.set(itf.SubsystemInterface(name="accumulator", mass_kg=55.0))
    L.set(itf.SubsystemInterface(name="harness"))
    return L


def test_the_car_mass_is_the_sum_of_what_the_ledger_declares():
    """The live ledger, its saved dict and a flat {name: dict} all give the declared total; an empty ledger falls back
    so the march never runs on a 0 kg car."""
    L = _ledger()
    assert e.total_mass_from_ledger(L) == pytest.approx(95.0)
    assert e.total_mass_from_ledger(L.as_dict(), driver_kg=68.0) == pytest.approx(163.0)
    assert e.total_mass_from_ledger({"a": {"mass_kg": 10.0}, "b": {"mass_kg": "x"}, "c": None}) == pytest.approx(10.0)
    assert e.total_mass_from_ledger({"items": {"a": {"mass_kg": 7.0}}}) == pytest.approx(7.0)
    assert e.total_mass_from_ledger(None) == e.DEFAULT_VEHICLE_MASS_KG
    assert e.total_mass_from_ledger(["not", "a", "mapping"]) == e.DEFAULT_VEHICLE_MASS_KG
    p = _ideal().with_ledger_mass(L, driver_kg=68.0)
    assert p.mass_kg == pytest.approx(163.0)


def test_results_are_published_back_to_the_ledger():
    L = _ledger()
    run = e.simulate_launch(_ideal(motor=e.MotorCurve.from_peak(180.0, 80.0, 6500.0), mu=1.4), distance_m=20.0)
    th = e.simulate_lap_thermal(e.CoolingNetwork(), drive=run, n_laps=1)
    assert e.publish_to_ledger(L, drive=run, thermal=th) is L
    pt, cool = L.get("powertrain"), L.get("cooling")
    assert pt.peak_torque_nm == pytest.approx(run.motor_torque_nm.max()) and pt.is_estimate
    assert pt.peak_power_kw == pytest.approx(run.elec_power_w.max() / 1e3)
    assert cool.heat_reject_w == pytest.approx(th.q_reject_w.max()) and cool.updated_by == "powertrain.engine"
    plain = {"a": 1}
    assert e.publish_to_ledger(plain, drive=run) is plain and e.publish_to_ledger(None) is None


def test_ledger_edge_cases_never_raise_into_the_ui(monkeypatch):
    """Interface objects in a plain mapping are read by attribute; a ledger without a setter is updated in place but
    not re-set; only a thermal result touches only cooling; and with no interfaces module the ledger comes back
    unchanged."""
    assert e.total_mass_from_ledger({"a": itf.SubsystemInterface(name="a", mass_kg=12.0)}) == pytest.approx(12.0)

    class GetOnly:
        def __init__(self):
            self.it = itf.SubsystemInterface(name="cooling")
        def get(self, name):
            return self.it if name == "cooling" else None
    led = GetOnly()
    th = e.simulate_lap_thermal(e.CoolingNetwork(), t_s=[0.0, 1.0], v_car_ms=[10.0, 10.0], n_laps=1)
    assert e.publish_to_ledger(led, thermal=th) is led and led.it.heat_reject_w == pytest.approx(th.q_reject_w.max())
    L = _ledger()
    import suspension
    monkeypatch.setitem(sys.modules, "suspension.interfaces", None)
    monkeypatch.delattr(suspension, "interfaces")
    assert e.publish_to_ledger(L, thermal=th) is L and L.get("cooling") is None


@pytest.mark.filterwarnings("ignore::RuntimeWarning")
def test_the_modules_own_self_check_passes(capsys):
    import runpy
    runpy.run_module("powertrain.engine", run_name="__main__")
    assert "SELF-CHECK OK" in capsys.readouterr().out
