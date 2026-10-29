"""suspension/tire_utilisation.py: the combined-slip reduction sqrt(1 - (Fx / kx mu Fz)^2), the common slip angle
that meets an axle's lateral force, the per-tire state against the stated force allocation, and the g-g limit as the
edge of feasibility."""
import math

import numpy as np
import pytest

from suspension import tire_utilisation as tu
from suspension.target_derivation import Vehicle, corner_loads
from suspension.tiremodel import default_tire

T = default_tire()


def test_the_combined_slip_reduction():
    mu = T.mu_peak(900.0)
    r = 400.0 / (1.2 * mu * 900.0)
    want = abs(T.fy(math.radians(5.0), 900.0)) * math.sqrt(1 - r * r)
    assert tu._fy_combined(T, 5.0, 900.0, -400.0, 1.2, 0.0) == pytest.approx(want)
    assert tu._fy_combined(T, 5.0, 900.0, 400.0, 1.2, 0.0, mu=mu) == pytest.approx(want)
    assert tu._fy_combined(T, 5.0, 900.0, 400.0, 1.2, 0.0, mu=2 * mu) != pytest.approx(want)
    g = math.radians(2.0)
    mg = T.mu_peak(900.0, g)
    rg = 400.0 / (mg * 900.0)
    assert tu._fy_combined(T, 5.0, 900.0, 400.0, 1.0, g) == pytest.approx(
        abs(T.fy(math.radians(5.0), 900.0, g)) * math.sqrt(1 - rg * rg))
    assert tu._fy_combined(T, 5.0, 900.0, 1e6, 1.0, 0.0) == 0.0
    assert tu._fy_combined(T, 5.0, 0.0, 10.0, 1.0, 0.0, mu=1.5) == 0.0


def test_the_axle_split_meets_its_lateral_force():
    fz, fx = (1100.0, 500.0), (-200.0, -150.0)
    a, fys = tu._axle_split(T, 1800.0, fz, fx, 1.0, 0.0)
    assert sum(fys) == pytest.approx(1800.0, rel=1e-9) and a > 0
    assert fys == pytest.approx([tu._fy_combined(T, a, z, x, 1.0, 0.0) for z, x in zip(fz, fx)])
    an, fyn = tu._axle_split(T, -1800.0, fz, fx, 1.0, 0.0)
    assert an == pytest.approx(-a) and fyn == pytest.approx([-f for f in fys])
    grid = np.linspace(0, 30, 301)
    cap = max(sum(tu._fy_combined(T, g, z, x, 1.0, 0.0) for z, x in zip(fz, fx)) for g in grid)
    assert tu._axle_split(T, cap * 1.001, fz, fx, 1.0, 0.0) is None
    assert tu._axle_split(T, cap * 0.999, fz, fx, 1.0, 0.0) is not None
    assert tu._axle_split(T, 0.0, fz, fx, 1.0, 0.0)[0] == pytest.approx(0.0, abs=1e-12)


def test_the_tire_state_in_combined_braking():
    veh = Vehicle()
    s = tu.tire_state(0.6, 0.9, brake_front=0.65, kx=1.1)
    fz = corner_loads(veh, a_lat_g=0.9, a_long_g=0.6, weight_front=0.48)["total"]
    Fx = 300 * 9.81 * 0.6
    want_fx = {"FL": -0.65 * Fx / 2, "FR": -0.65 * Fx / 2, "RL": -0.35 * Fx / 2, "RR": -0.35 * Fx / 2}
    for t in tu.TIRES:
        r = s[t]
        mu = T.mu_peak(fz[t])
        assert (r["Fz"], r["Fx"], r["mu"]) == pytest.approx((fz[t], want_fx[t], mu))
        assert r["u"] == pytest.approx(math.hypot(r["Fx"] / (1.1 * mu * fz[t]), r["Fy"] / (mu * fz[t])))
        assert r["margin"] == pytest.approx(1 - r["u"])
        assert r["spare_n"] == pytest.approx(math.hypot(r["Fx"], r["Fy"]) * (1 / r["u"] - 1))
    m = 300 * 9.81 * 0.9
    assert s["FL"]["Fy"] + s["FR"]["Fy"] == pytest.approx(0.48 * m, rel=1e-9)
    assert s["RL"]["Fy"] + s["RR"]["Fy"] == pytest.approx(0.52 * m, rel=1e-9)
    assert s["FL"]["alpha_deg"] == s["FR"]["alpha_deg"] and s["FL"]["alpha_deg"] > 0
    assert s["feasible"] is True and s["vehicle_u"] == max(s[t]["u"] for t in tu.TIRES)
    assert s["grip_used_n"] == pytest.approx(sum(math.hypot(s[t]["Fx"], s[t]["Fy"]) for t in tu.TIRES))
    assert s["grip_spare_n"] == pytest.approx(sum(s[t]["spare_n"] for t in tu.TIRES))


def test_traction_goes_to_the_rear_and_options_pass_through():
    s = tu.tire_state(-0.5, 0.0, weight_front=0.45, downforce_n=400.0)
    Fx = 300 * 9.81 * -0.5
    assert (s["FL"]["Fx"], s["FR"]["Fx"], s["RL"]["Fx"], s["RR"]["Fx"]) == pytest.approx((0, 0, -Fx / 2, -Fx / 2))
    fz = corner_loads(Vehicle(), a_lat_g=0.0, a_long_g=-0.5, weight_front=0.45, downforce_n=400.0)["total"]
    assert {t: s[t]["Fz"] for t in tu.TIRES} == pytest.approx(fz)
    zero = tu.tire_state(0.0, 0.0)
    for t in tu.TIRES:                       # no demand: the whole ellipse is spare
        assert zero[t]["u"] < 1e-9 and zero[t]["spare_n"] == pytest.approx(zero[t]["mu"] * zero[t]["Fz"], rel=1e-6)
    assert zero["feasible"] is True
    nb = tu.tire_state(0.4, 0.0)
    assert nb["FL"]["Fx"] == pytest.approx(-0.6 * 300 * 9.81 * 0.4 / 2)          # default brake share 0.60


def test_past_the_limit_is_infeasible():
    s = tu.tire_state(0.0, 2.4)
    assert s["feasible"] is False
    bad = [ax for ax, pair in (("F", ("FL", "FR")), ("R", ("RL", "RR"))) if math.isnan(s[pair[0]]["alpha_deg"])]
    assert bad
    for ax in bad:
        share = 0.48 if ax == "F" else 0.52
        pair = ("FL", "FR") if ax == "F" else ("RL", "RR")
        assert s[pair[0]]["Fy"] == pytest.approx(300 * 9.81 * 2.4 * share / 2) == s[pair[1]]["Fy"]


def test_the_g_g_limit():
    assert tu.U_TOL == 1.0 + 1e-4 and tu.G == 9.81 and tu.TIRES == ("FL", "FR", "RL", "RR")
    rows = tu.limit_envelope(n_dir=2)
    assert [r["theta_deg"] for r in rows] == pytest.approx([0.0, 90.0, 180.0])
    for r in rows:
        th = math.radians(r["theta_deg"])
        ok = lambda a: (lambda s: s["feasible"] and s["vehicle_u"] <= tu.U_TOL)(tu.tire_state(a * math.cos(th),
                                                                                              a * math.sin(th)))
        assert 0.0 < r["a_g"] < 2.5 and ok(r["a_g"]) and not ok(r["a_g"] + 1e-6)
        assert (r["ax_g"], r["ay_g"]) == pytest.approx((r["a_g"] * math.cos(th), r["a_g"] * math.sin(th)), abs=1e-12)
        s = tu.tire_state(r["ax_g"], r["ay_g"])
        assert {t: r[f"u_{t}"] for t in tu.TIRES} == pytest.approx({t: s[t]["u"] for t in tu.TIRES})
        assert r["limiting"] == max(tu.TIRES, key=lambda t: s[t]["u"])
    one = tu.limit_envelope(n_dir=1, brake_front=0.7)
    assert len(one) == 2 and one[0]["a_g"] != rows[0]["a_g"]


class _Lin:
    """Fy = -C alpha Fz exactly (zero at zero slip), mu fixed: utilisation can be exactly zero."""

    def fy(self, a, fz, g=0.0):
        return 0.0 if abs(a) < 1e-12 else -min(20.0 * a, 1.5) * fz   # zero at (the bisection's 1e-17 of) zero slip

    def mu_peak(self, fz, g=0.0):
        return 1.5


def test_camber_reaches_every_lookup():
    g = math.radians(3.0)
    s = tu.tire_state(0.3, 1.0, gamma=g)
    fz = corner_loads(Vehicle(), a_lat_g=1.0, a_long_g=0.3, weight_front=0.48)["total"]
    for t in tu.TIRES:
        assert s[t]["mu"] == pytest.approx(T.mu_peak(fz[t], g))
    a, fys = tu._axle_split(T, 1800.0, (1100.0, 500.0), (-200.0, -150.0), 1.0, g)
    assert fys == pytest.approx([tu._fy_combined(T, a, z, x, 1.0, g) for z, x in ((1100.0, -200.0), (500.0, -150.0))])
    assert a != pytest.approx(tu._axle_split(T, 1800.0, (1100.0, 500.0), (-200.0, -150.0), 1.0, 0.0)[0])


def test_a_small_demand_and_a_light_tire():
    a, fys = tu._axle_split(T, 0.5, (900.0, 900.0), (0.0, 0.0), 1.0, 0.0)
    assert a > 0 and sum(fys) == pytest.approx(0.5, rel=1e-6)
    assert tu._fy_combined(T, 5.0, 0.5, 0.0, 1.0, 0.0, mu=1.5) == pytest.approx(abs(T.fy(math.radians(5.0), 0.5)))
    assert tu._fy_combined(T, 5.0, 900.0, 5e4, 1.0, 0.0, mu=1.5) == 0.0


def test_an_unused_tire_has_its_whole_ellipse_spare():
    s = tu.tire_state(0.0, 0.0, tire=_Lin())
    fz = corner_loads(Vehicle(), weight_front=0.48)["total"]
    for t in tu.TIRES:
        assert s[t]["u"] == 0.0 and s[t]["spare_n"] == pytest.approx(1.5 * fz[t])
    k = tu.tire_state(0.0, 0.0, tire=_Lin(), kx=1.3)
    assert k["FL"]["spare_n"] == pytest.approx(1.3 * 1.5 * fz["FL"])
    past = tu.tire_state(0.0, 3.0, tire=_Lin())
    assert past["feasible"] is False and past["FL"]["Fy"] == past["FR"]["Fy"] == pytest.approx(
        300 * 9.81 * 3.0 * 0.48 / 2)


def test_the_envelope_uses_the_given_car_tire_and_options():
    veh = Vehicle(mass_kg=260.0)

    class Weak(_Lin):
        def mu_peak(self, fz, g=0.0):
            return 0.6

        def fy(self, a, fz, g=0.0):
            return -min(20.0 * a, 0.6) * fz
    rows = tu.limit_envelope(n_dir=2, veh=veh, tire=Weak(), brake_front=0.7)
    for r in rows:
        th = math.radians(r["theta_deg"])
        ok = lambda a: (lambda s: s["feasible"] and s["vehicle_u"] <= tu.U_TOL)(
            tu.tire_state(a * math.cos(th), a * math.sin(th), veh, Weak(), brake_front=0.7))
        assert 0.0 < r["a_g"] < 1.0 and ok(r["a_g"]) and not ok(r["a_g"] + 1e-6)
    assert max(r["a_g"] for r in tu.limit_envelope(n_dir=4)) < 2.0           # far inside the 2.5 g bracket


def test_the_envelope_rows_use_the_given_car_and_options():
    veh = Vehicle(cg_height_mm=400.0, mass_kg=260.0)
    t = default_tire()
    t.scaling = {**t.scaling, "LMUY": 0.9}
    rows = tu.limit_envelope(n_dir=1, veh=veh, tire=t, brake_front=0.7)
    for r in rows:
        s = tu.tire_state(r["ax_g"], r["ay_g"], veh, t, brake_front=0.7)
        assert {k: r[f"u_{k}"] for k in tu.TIRES} == pytest.approx({k: s[k]["u"] for k in tu.TIRES})
    plain = tu.limit_envelope(n_dir=1)
    assert rows[0]["a_g"] != pytest.approx(plain[0]["a_g"])
