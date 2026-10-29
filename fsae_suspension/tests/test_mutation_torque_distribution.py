"""suspension/torque_distribution.py against the balance it must satisfy (Fy_f + Fy_r = m a_y and
a Fy_f - b Fy_r + Mz_x = Iz r_dot), the force split each option allows, each tire's friction-ellipse use, an
independent scan of the split for the best one, bisection edges for the limits, the clothoid path in closed form and
the yaw budget term by term."""
import math

import numpy as np
import pytest

from suspension import torque_distribution as td
from suspension.target_derivation import Vehicle, corner_loads
from suspension.tiremodel import default_tire

TT = td._TT()
C = td.Car()


def test_the_tire_table():
    TT = td.TireTable()                  # built here: mutmut forks after import, so the module's table is never mutated
    t = default_tire()
    assert td._ALPHA == pytest.approx(np.radians(np.arange(0.0, 20.01, 0.25))) and len(td._ALPHA) == 81
    assert td._FZ[0] == 20.0 and td._FZ[-1] == 2600.0 and len(td._FZ) == 130
    for i, j in ((0, 0), (5, 17), (129, 80), (64, 40)):
        assert TT.fy[i, j] == pytest.approx(abs(t.fy(td._ALPHA[j], td._FZ[i], 0.0)))
    assert TT.mu == pytest.approx(TT.fy.max(axis=1) / td._FZ)
    assert TT.row(730.0) == pytest.approx(TT.fy[35] * 0.5 + TT.fy[36] * 0.5)         # halfway between 720 and 740
    assert TT.row(726.0) == pytest.approx(TT.fy[35] * 0.7 + TT.fy[36] * 0.3)
    assert TT.row(720.0) == pytest.approx(TT.fy[35])
    assert TT.row(5.0) == pytest.approx(TT.fy[0]) and TT.row(9000.0) == pytest.approx(TT.fy[129])
    assert TT.row(2590.0) == pytest.approx(TT.fy[128] * 0.5 + TT.fy[129] * 0.5)
    assert TT.mu_at(730.0) == pytest.approx(0.5 * (TT.mu[35] + TT.mu[36])) and TT.mu_at(1.0) == TT.mu[0]
    g = td.TireTable(gamma=math.radians(2.0))
    assert g.fy[40, 30] == pytest.approx(abs(t.fy(td._ALPHA[30], td._FZ[40], math.radians(2.0))))
    assert td.G == 9.81 and td.OPTIONS == ("open", "lsd2", "lsd3", "spool", "tv")


def test_the_car():
    assert (C.m, C.wf, C.L, C.t, C.iz, C.cda, C.crr, C.brake_front, C.rho, C.tv_wheel_max_n, C.cx_per_fz,
            C.front_lltd) == (300.0, 0.48, 1.63, 1.21, 198.95, 1.0, 0.015, 0.73, 1.225, 1200.0, 20.0, 0.53)
    assert C.a == pytest.approx(1.63 * 0.52) and C.b == pytest.approx(1.63 * 0.48)


def test_the_split_each_option_allows():
    assert td._d_range("tv", True, 800.0, C, 900.0, 500.0, 0.1, TT) == (-800.0, 800.0)
    assert td._d_range("tv", False, -900.0, C, 900.0, 500.0, 0.1, TT) == (-750.0, 750.0)
    assert td._d_range("tv", True, 3000.0, C, 900.0, 500.0, 0.1, TT) == (-0.0, 0.0)
    assert td._d_range("open", True, 800.0, C, 900.0, 500.0, 0.1, TT) == (0.0, 0.0)
    for opt in ("lsd2", "lsd3"):
        assert td._d_range(opt, False, -800.0, C, 900.0, 500.0, 0.1, TT) == (0.0, 0.0)
    assert td._d_range("lsd2", True, 900.0, C, 900.0, 500.0, 0.1, TT) == pytest.approx((450.0 * -1 / 3, 0.0))
    assert td._d_range("lsd3", True, 900.0, C, 900.0, 500.0, 0.1, TT) == pytest.approx((450.0 * -2 / 4, 0.0))
    d = -20.0 * 700.0 * 1.21 * 0.01 / 2                               # small curvature: the slip-stiffness drag
    assert td._d_range("spool", True, 400.0, C, 900.0, 500.0, 0.01, TT) == pytest.approx((d, d))
    cap = min(TT.mu_at(900.0) * 900.0, TT.mu_at(500.0) * 500.0) - 200.0
    assert td._d_range("spool", True, 400.0, C, 900.0, 500.0, 0.2, TT) == pytest.approx((-cap, -cap))
    cap0 = TT.mu_at(1.0) * 1.0 - 200.0                                # a lifted wheel: no grip to drag against
    assert td._d_range("spool", True, -400.0, C, 900.0, -5.0, 0.2, TT) == pytest.approx((-max(cap0, 0.0),) * 2)
    big = td._d_range("spool", True, 1e5, C, 900.0, 500.0, 0.2, TT)
    assert big == (0.0, 0.0)
    with pytest.raises(KeyError):
        td._d_range("lsd4", True, 900.0, C, 900.0, 500.0, 0.1, TT)


def test_the_common_slip_angle_on_one_axle():
    fy, a = td._axle(TT, 1500.0, (700.0, 1100.0), (0.0, 0.0))
    assert sum(fy) == pytest.approx(1500.0, rel=1e-9) and 0 < a < math.radians(20)
    assert fy == pytest.approx([float(np.interp(a, td._ALPHA, TT.row(z))) for z in (700.0, 1100.0)])
    fy2, a2 = td._axle(TT, -1500.0, (700.0, 1100.0), (300.0, -400.0))       # sign ignored; drive cuts lateral
    s = [math.sqrt(1 - (300 / (TT.mu_at(700.0) * 700)) ** 2), math.sqrt(1 - (400 / (TT.mu_at(1100.0) * 1100)) ** 2)]
    assert sum(fy2) == pytest.approx(1500.0, rel=1e-9) and a2 > a
    assert fy2 == pytest.approx([s[i] * float(np.interp(a2, td._ALPHA, TT.row(z))) for i, z in enumerate((700.0,
                                                                                                          1100.0))])
    tot = TT.row(700.0) + TT.row(1100.0)
    over = td._axle(TT, 2 * tot.max(), (700.0, 1100.0), (0.0, 0.0))
    assert over[0] == "over" and over[1] == pytest.approx(2.0)
    sat = td._axle(TT, 10.0, (700.0, 1100.0), (1e5, 0.0))                 # one tire spent on drive
    assert sat[0][0] == 0.0 and sat[0][1] == pytest.approx(10.0)
    dead = td._axle(TT, 10.0, (0.0, -5.0), (1e5, 1e5))
    assert dead == ("over", pytest.approx(10.0 / 1e-6))
    exact = td._axle(TT, tot.max(), (700.0, 1100.0), (0.0, 0.0))
    assert exact[1] == pytest.approx(td._ALPHA[int(np.argmax(tot))])


def _umax(r, d, car=C):
    """The allocation for a split d rebuilt from the balance: forces, axle solves, then the worst tire."""
    fz, F, kappa = r["fz"], r["F_long"], None
    driving = F >= 0.0
    fxf = 0.0 if driving else car.brake_front * F / 2.0
    FR = F if driving else (1.0 - car.brake_front) * F
    mzx = 2 * d * car.t / 2.0
    may = r["Fy_f"] + r["Fy_r"]
    fyf = (may * car.b + r["mz_req"] - mzx) / car.L
    front = td._axle(TT, fyf, (fz["FL"], fz["FR"]), (fxf, fxf))
    rear = td._axle(TT, may - fyf, (fz["RL"], fz["RR"]), (FR / 2 - d, FR / 2 + d))
    if front[0] == "over" or rear[0] == "over":
        return 1.0 + max(x[1] for x in (front, rear) if x[0] == "over")
    fx = {"FL": fxf, "FR": fxf, "RL": FR / 2 - d, "RR": FR / 2 + d}
    fy = dict(zip(("FL", "FR", "RL", "RR"), front[0] + rear[0]))
    return max(math.hypot(fx[k], fy[k]) / (TT.mu_at(max(fz[k], 1.0)) * max(fz[k], 1.0)) for k in fx)


def _check_state(V, k, dk, at, option, car=C):
    r = td.state(V, k, dk, at, option, car)
    ay = V * V * k
    mz_req = car.iz * (V * V * dk + at * k)
    assert r["ay_g"] == pytest.approx(ay / 9.81) and r["at_g"] == pytest.approx(at / 9.81)
    assert r["mz_req"] == pytest.approx(mz_req, abs=1e-12)
    assert r["fz"] == pytest.approx(corner_loads(Vehicle(), a_lat_g=ay / 9.81, a_long_g=-at / 9.81,
                                                 front_lltd=car.front_lltd)["total"])
    assert r["Fy_f"] + r["Fy_r"] == pytest.approx(car.m * ay, rel=1e-12, abs=1e-9)
    assert car.a * r["Fy_f"] - car.b * r["Fy_r"] + r["mz_x"] == pytest.approx(mz_req, abs=1e-6)
    fx, fy, F, fz = r["fx"], r["fy"], r["F_long"], r["fz"]
    assert r["mz_x"] == pytest.approx((fx["RR"] - fx["RL"]) * car.t / 2) and r["d"] == pytest.approx(
        (fx["RR"] - fx["RL"]) / 2)
    if F >= 0:
        assert fx["FL"] == fx["FR"] == 0.0 and fx["RL"] + fx["RR"] == pytest.approx(F)
    else:
        assert fx["FL"] == fx["FR"] == pytest.approx(car.brake_front * F / 2)
        assert fx["RL"] + fx["RR"] == pytest.approx((1 - car.brake_front) * F)
    drag = 0.5 * car.rho * car.cda * V * V + car.crr * car.m * 9.81
    induced = r["Fy_f"] * math.sin(car.L * k + r["alpha_f"] - r["alpha_r"])
    assert F - car.m * at - drag == pytest.approx(induced, rel=0.1, abs=1e-9)         # from the previous pass
    assert fy["FL"] + fy["FR"] == pytest.approx(abs(r["Fy_f"]), rel=1e-9)
    assert fy["RL"] + fy["RR"] == pytest.approx(abs(r["Fy_r"]), rel=1e-9)
    for w in ("FL", "FR", "RL", "RR"):
        z = max(fz[w], 1.0)
        assert r["u"][w] == pytest.approx(math.hypot(fx[w], fy[w]) / (TT.mu_at(z) * z))
    assert r["umax"] == pytest.approx(max(r["u"].values())) and r["ok"] is (r["umax"] <= 1.0)
    assert _umax(r, r["d"], car) == pytest.approx(r["umax"], rel=1e-9)
    return r


def test_the_allocation_balances_and_the_split_is_the_best_one():
    r = _check_state(9.0, 1 / 9.125, 0.004, 2.0, "tv")
    lo, hi = -(C.tv_wheel_max_n - abs(r["F_long"]) / 2), C.tv_wheel_max_n - abs(r["F_long"]) / 2
    assert lo <= r["d"] <= hi
    scan = min(_umax(r, d) for d in np.linspace(lo, hi, 801))
    assert r["umax"] <= scan + 1e-6 and r["umax"] < _umax(r, 0.0)                  # vectoring beats equal torque
    b = _check_state(11.0, 1 / 20.0, -0.002, -7.0, "tv")                             # braking, regenerative vectoring
    assert b["F_long"] < 0
    D = C.tv_wheel_max_n - abs((1 - C.brake_front) * b["F_long"]) / 2
    assert b["umax"] <= min(_umax(b, d) for d in np.linspace(-D, D, 801)) + 1e-6
    l = _check_state(8.0, 1 / 15.0, 0.0, 3.0, "lsd3")
    lo = l["F_long"] / 2 * (1 - 3) / (1 + 3)
    assert lo - 1e-9 <= l["d"] <= 1e-9 and l["umax"] <= min(_umax(l, d) for d in np.linspace(lo, 0, 401)) + 1e-6
    o = _check_state(8.0, 1 / 15.0, 0.0, 3.0, "open")
    assert o["d"] == 0.0
    s = _check_state(6.0, 1 / 9.125, 0.0, 1.0, "spool")
    fzm = 0.5 * (s["fz"]["RR"] + s["fz"]["RL"])
    assert s["d"] == pytest.approx(max(-20.0 * fzm * 1.21 / 9.125 / 2, -(min(
        TT.mu_at(s["fz"]["RR"]) * s["fz"]["RR"], TT.mu_at(s["fz"]["RL"]) * s["fz"]["RL"]) - s["F_long"] / 2)))
    ls = _check_state(8.0, 1 / 15.0, 0.0, -5.0, "lsd2")                              # braking: no locking torque
    assert ls["d"] == 0.0
    st = _check_state(10.0, 0.0, 0.0, 1.5, "tv")                                     # straight: no induced drag
    assert st["F_long"] == pytest.approx(300 * 1.5 + 0.5 * 1.225 * 100 + 0.015 * 300 * 9.81)
    right = _check_state(9.0, -1 / 12.0, 0.0, 0.0, "open")
    assert right["Fy_f"] < 0


def test_the_car_and_vehicle_arguments_reach_the_allocation():
    car = td.Car(m=280.0, wf=0.45, iz=150.0, front_lltd=0.6, brake_front=0.6)
    r = _check_state(9.0, 1 / 12.0, 0.003, -3.0, "tv", car)
    assert r["fz"] == pytest.approx(corner_loads(Vehicle(), a_lat_g=81 / 12 / 9.81, a_long_g=3 / 9.81,
                                                 front_lltd=0.6)["total"])
    v = Vehicle(mass_kg=260.0)
    rv = td.state(9.0, 1 / 12.0, 0.0, 0.0, "open", veh=v, tt=TT, n_split=5)
    assert rv["fz"] == pytest.approx(corner_loads(v, a_lat_g=81 / 12 / 9.81, a_long_g=0.0, front_lltd=0.53)["total"])


def test_past_the_limit_is_infeasible():
    r = td.state(25.0, 1 / 9.125, 0.0, 0.0, "open")
    assert r["ok"] is False and r["umax"] > 1.0 and set(r) == {"ok", "umax"}
    over = td.state(13.0, 1 / 9.125, 0.0, 0.0, "open")
    assert over["umax"] > 1.0


def test_the_steady_limit_is_the_feasibility_edge():
    for R, opt in ((15.0, "open"), (9.125, "tv")):
        r = td.steady_limit(R, opt)
        assert td.state(r["V"], 1 / R, 0.0, 0.0, opt)["ok"] and not td.state(r["V"] + 1e-5, 1 / R, 0.0, 0.0, opt)["ok"]
        s = td.state(r["V"], 1 / R, 0.0, 0.0, opt)
        assert r == {"R": R, "V": r["V"], "ay_g": pytest.approx(r["V"] ** 2 / R / 9.81),
                     **{k: pytest.approx(s[k]) for k in ("d", "mz_x", "umax", "F_long")}, "u": pytest.approx(s["u"])}
    light = td.steady_limit(15.0, "open", td.Car(cda=3.0))
    assert light["V"] != td.steady_limit(15.0, "open")["V"]


def test_the_path():
    s, k, dk = td.corner_path(10.0, 90.0, straight=4.0, transition=2.0, ds=1.0)
    arc = math.pi / 2 * 10.0
    n_arc = round(arc)
    assert len(s) == 4 + 2 + n_arc + 2 + 4
    want_s, want_k, want_dk, s0 = [], [], [], 0.0
    for L, k0, k1 in ((4.0, 0, 0), (2.0, 0, 0.1), (arc, 0.1, 0.1), (2.0, 0.1, 0), (4.0, 0, 0)):
        n = max(1, round(L))
        for i in range(n):
            x = (i + 0.5) / n
            want_s.append(s0 + x * L); want_k.append(k0 + (k1 - k0) * x); want_dk.append((k1 - k0) / L)
        s0 += L
    assert s == pytest.approx(want_s) and k == pytest.approx(want_k) and dk == pytest.approx(want_dk)
    assert k[4] == pytest.approx(0.025) and dk[4] == pytest.approx(0.05)
    s, k, dk = td.corner_path(10.0, 1.0, straight=0.2, transition=0.3, ds=1.0)        # every segment gets a point
    assert len(s) == 5
    s, k, dk = td.corner_path(9.0, 60.0)
    assert len(s) == 80 + 12 + round(math.pi / 3 * 9 / 0.5) + 12 + 80 and s[-1] == pytest.approx(
        92 + math.pi / 3 * 9 - 0.25)


def test_a_corner_is_driven_at_the_limits():
    r = td.simulate_corner(12.0, 60.0, "open", v_top=18.0, ds=10.0)
    s, k, dk = td.corner_path(12.0, 60.0, ds=10.0)
    V = r["V"]
    assert r["s"] == pytest.approx(s) and r["kappa"] == pytest.approx(k)
    assert r["at"] == pytest.approx(np.gradient(V * V / 2, s)) and r["at_g"] == pytest.approx(r["at"] / 9.81)
    assert r["ay_g"] == pytest.approx(V ** 2 * k / 9.81)
    t = np.concatenate([[0.0], np.cumsum(10.0 / np.maximum(0.5 * (V[1:] + V[:-1]), 0.1))])
    assert r["t"] == pytest.approx(t) and r["time"] == pytest.approx(t[-1])
    assert V.max() <= 18.0 and V[0] == pytest.approx(18.0) and V.min() < 14.0
    rows = [td.state(V[i], k[i], dk[i], r["at"][i], "open") for i in range(len(V))]
    for w in ("FL", "FR", "RL", "RR"):
        want = np.array([x["u"][w] if "u" in x else np.nan for x in rows])
        assert np.allclose(r["u"][w], want, equal_nan=True)
    assert np.allclose(r["mz_req"], [x.get("mz_req", np.nan) for x in rows], equal_nan=True)
    fast = td.simulate_corner(40.0, 30.0, "open", v_top=12.0, ds=10.0)                 # never limited: flat out
    assert fast["V"] == pytest.approx(np.full(len(fast["V"]), 12.0))


def test_the_yaw_budget_term_by_term():
    r = td.state(12.0, 0.5 / 9.125, (1 / 9.125) / 6.0, -6.0, "tv")
    b = td.yaw_budget(r)
    h = C.t / 2
    pos = {"FL": (C.a, h), "FR": (C.a, -h), "RL": (-C.b, h), "RR": (-C.b, -h)}
    for w, (x, y) in pos.items():
        al = abs(math.degrees(r["alpha_f"] if w[0] == "F" else r["alpha_r"]))
        tp = 0.014 * math.cos(math.pi * al / (2 * 8.1)) if al < 8.1 else 0.0
        assert b[f"Fy {w}"] == pytest.approx(x * r["fy"][w]) and b[f"Fx {w}"] == pytest.approx(-y * r["fx"][w])
        assert b[f"Mz {w}"] == pytest.approx(-r["fy"][w] * tp)
    force = sum(b[f"{t} {w}"] for t in ("Fy", "Fx") for w in pos)
    assert b["sum_force_terms"] == pytest.approx(force)
    assert b["sum_all"] == pytest.approx(force + sum(b[f"Mz {w}"] for w in pos)) and b["iz_rdot"] == r["mz_req"]
    car = td.Car(t=1.3, wf=0.5)
    b2 = td.yaw_budget(r, car, t0_mm=20.0, alpha_peak_deg=1e-3)                       # past the trail law's peak
    assert all(b2[f"Mz {w}"] == 0.0 for w in pos) and b2["Fx RR"] == pytest.approx(0.65 * r["fx"]["RR"])
    assert b2["Fy FL"] == pytest.approx(0.815 * r["fy"]["FL"])
    big = td.yaw_budget(r, t0_mm=20.0, alpha_peak_deg=30.0)
    al = abs(math.degrees(r["alpha_f"]))
    assert big["Mz FL"] == pytest.approx(-r["fy"]["FL"] * 0.02 * math.cos(math.pi * al / 60.0))
    edge = dict(r, alpha_f=math.radians(8.1), alpha_r=math.radians(-8.1))
    assert all(td.yaw_budget(edge)[f"Mz {w}"] == 0.0 for w in pos)


def test_the_table_is_built_once(monkeypatch):
    monkeypatch.setattr(td, "_TT_CACHE", [])
    a = td._TT()
    assert isinstance(a, td.TireTable) and td._TT() is a and td._TT_CACHE == [a]


def test_a_lifted_wheel_and_a_light_axle():
    c = td.Car(front_lltd=1.0)
    r = _check_state(8.0, 1 / 6.0, 0.0, 0.0, "open", c)
    assert r["fz"]["FL"] < 1.0
    mu0 = TT.mu_at(1.0)
    fy, a = td._axle(TT, 10.0, (0.5, 900.0), (1.5 * mu0, 0.0))
    s0 = math.sqrt(1 - (1.5 * mu0 / mu0) ** 2) if 1.5 < 1 else 0.0
    assert fy[0] == pytest.approx(s0 * float(np.interp(a, td._ALPHA, TT.row(1.0)))) and sum(fy) == pytest.approx(10.0)
    fy, a = td._axle(TT, 10.0, (0.5, 900.0), (0.7 * mu0, 0.0))
    assert fy[0] == pytest.approx(math.sqrt(1 - 0.49) * float(np.interp(a, td._ALPHA, TT.row(1.0))))
    cap = TT.mu_at(1.0) * 1.0
    assert td._d_range("spool", True, 0.0, C, 900.0, 0.5, 0.5, TT) == pytest.approx((-cap, -cap))
    assert td._d_range("spool", True, 0.0, C, 0.5, 900.0, 0.5, TT) == pytest.approx((-cap, -cap))


def test_the_tire_utilisation_past_the_ellipse_and_past_the_axle():
    over = td.state(10.0, 0.0, 0.0, 15.0, "open")                 # straight: no lateral demand, too much drive
    assert over["ok"] is False and 1.0 < over["umax"] <= 2.0 and "u" in over
    assert over["umax"] == pytest.approx(max(over["u"].values()))
    far = td.state(13.0, 1 / 9.125, 0.0, 0.0, "open")
    assert set(far) == {"ok", "umax"} and 1.0 < far["umax"] < 3.0


def test_drag_area_and_the_car_reach_the_answers():
    c = td.Car(cda=1.7, m=280.0)
    st = _check_state(10.0, 0.0, 0.0, 1.5, "tv", c)
    assert st["F_long"] == pytest.approx(280 * 1.5 + 0.5 * 1.225 * 1.7 * 100 + 0.015 * 280 * 9.81)
    tiny = td.state(10.0, 0.0, 0.0, (0.5 - (0.5 * 1.225 * 100 + 0.015 * 300 * 9.81)) / 300, "open")
    assert 0 < tiny["F_long"] < 1.0 and tiny["fx"]["FL"] == 0.0             # a sliver of drive is still drive
    r = td.steady_limit(15.0, "open", c)
    s = td.state(r["V"], 1 / 15.0, 0.0, 0.0, "open", c)
    assert r["umax"] == pytest.approx(s["umax"]) and r["F_long"] == pytest.approx(s["F_long"])
    assert 1.0 < td.steady_limit(4.0, "spool")["V"] and td.steady_limit(40.0, "tv")["V"] < 30.0


def _profile(R, arc, opt, v_top, car, ds):
    """The quasi-steady two-pass profile: cap each point at its steady limit, accelerate forward at the traction
    limit, brake backward at the braking limit, each limit found by bisection on feasibility."""
    s, k, dk = td.corner_path(R, arc, ds=ds)
    ok = lambda V, i, at: td.state(V, k[i], dk[i], at, opt, car)["ok"]

    def bis(lo, hi, n, good):
        for _ in range(n):
            m = 0.5 * (lo + hi)
            lo, hi = (m, hi) if good(m) else (lo, m)
        return lo
    cap = [v_top if ok(v_top, i, 0.0) else bis(1.0, v_top, 18, lambda V, i=i: ok(V, i, 0.0)) for i in range(len(s))]
    vf = list(cap); vf[0] = min(cap[0], v_top)
    for i in range(1, len(s)):
        a = bis(0.0, 16.0, 14, lambda x, i=i: ok(vf[i - 1], i - 1, x))
        vf[i] = min(cap[i], math.sqrt(vf[i - 1] ** 2 + 2 * a * ds))
    vb = list(vf)
    for i in range(len(s) - 2, -1, -1):
        a = bis(0.0, 16.0, 14, lambda x, i=i: ok(vb[i + 1], i + 1, -x))
        vb[i] = min(vf[i], math.sqrt(vb[i + 1] ** 2 + 2 * a * ds))
    return np.array(vb)


def test_the_corner_profile_against_a_two_pass_written_here():
    car = td.Car(m=280.0, cda=1.4)
    r = td.simulate_corner(10.0, 90.0, "open", v_top=16.0, car=car, ds=10.0)
    V = _profile(10.0, 90.0, "open", 16.0, car, 10.0)
    assert r["V"] == pytest.approx(V, rel=1e-12) and V.min() < 14.0 and V[0] == V.max()
    rows = [td.state(V[i], r["kappa"][i], td.corner_path(10.0, 90.0, ds=10.0)[2][i], r["at"][i], "open", car)
            for i in range(len(V))]
    assert np.allclose(r["mz_x"], [x.get("mz_x", np.nan) for x in rows], equal_nan=True)
    slow = td.simulate_corner(10.0, 90.0, "open", v_top=0.05, car=car, ds=10.0)
    assert slow["t"][-1] == pytest.approx(10.0 * (len(slow["V"]) - 1) / 0.1)            # 0.1 m/s floor on the mean


def test_the_profile_on_an_analytic_friction_ellipse(monkeypatch):
    """A stand-in for the allocation: feasible inside (V^2 k / 9)^2 + (at / (6 - 0.1 V))^2 <= 1, so the cap, both
    passes and an infeasible final row (no tire data) are all exercised cheaply."""
    car = td.Car(m=290.0)

    def fake(V, kappa, dkds, at, option, c=None, *a, **k):
        assert c is car and option == "open"
        lim = 6.0 - 0.1 * V
        use = (V * V * abs(kappa) / 9.0) ** 2 + (at / lim) ** 2
        if use > 1.0 + 1e-9:
            return {"ok": False, "umax": 1.0 + use}
        return {"ok": True, "umax": use, "u": {w: use for w in ("FL", "FR", "RL", "RR")}, "mz_x": 10 * at,
                "mz_req": 3 * V}
    monkeypatch.setattr(td, "state", fake)
    r = td.simulate_corner(6.0, 90.0, "open", v_top=14.0, car=car, ds=2.0)
    V = _profile(6.0, 90.0, "open", 14.0, car, 2.0)
    assert r["V"] == pytest.approx(V, rel=1e-12) and V.min() < V.max()
    s, k, dk = td.corner_path(6.0, 90.0, ds=2.0)
    rows = [fake(V[i], k[i], dk[i], r["at"][i], "open", car) for i in range(len(V))]
    assert any(not x["ok"] for x in rows) and any(x["ok"] for x in rows)
    for w in ("FL", "FR", "RL", "RR"):
        assert np.allclose(r["u"][w], [x["u"][w] if "u" in x else np.nan for x in rows], equal_nan=True)
    assert np.allclose(r["mz_x"], [x.get("mz_x", np.nan) for x in rows], equal_nan=True)
    assert np.allclose(r["mz_req"], [x.get("mz_req", np.nan) for x in rows], equal_nan=True)
    assert r["mz_x"].dtype == float


def test_a_corner_feasible_at_v_top_only_when_coasting(monkeypatch):
    """At v_top the arc uses 99 % of the lateral grip with no tangential acceleration, so the cap check must be made
    at zero acceleration: any drive tips it over and the cap would fall below v_top."""
    car = td.Car(m=290.0)
    v_top = math.sqrt(6.0 * 9.0 * math.sqrt(0.99))

    def fake(V, kappa, dkds, at, option, c=None, *a, **k):
        use = (V * V * abs(kappa) / 9.0) ** 2 + (at / (6.0 - 0.1 * V)) ** 2
        return {"ok": use <= 1.0 + 1e-12, "umax": use, "u": {w: use for w in ("FL", "FR", "RL", "RR")},
                "mz_x": 0.0, "mz_req": 0.0}
    monkeypatch.setattr(td, "state", fake)
    r = td.simulate_corner(6.0, 90.0, "open", v_top=v_top, car=car, ds=2.0)
    assert r["V"] == pytest.approx(np.full(len(r["V"]), v_top), rel=1e-12)
