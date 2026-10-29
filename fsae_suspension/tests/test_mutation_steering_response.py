"""suspension/steering_response.py: the frequency response against the single-track equations eliminated by hand to a
2x2 system (not the module's 4x4), its steady state against V^2 / (L + K V^2) with compliance steer in series, the yaw
mode against the characteristic polynomial, the time response against scipy's integrator, and the paper's tire-derived
cornering stiffnesses."""
import math
from dataclasses import replace

import numpy as np
import pytest
from scipy.integrate import solve_ivp

from suspension import steering_response as sr

CAR = sr.SingleTrack(e_f_rad_per_n=2e-6, stiction=(1.3, 25.0), iz_kgm2=180.0)


def _frf(car, f):
    V, m, a, b, cf, cr = car.speed_ms, car.mass_kg, car.a_m, car.b_m, car.cf_n_per_rad, car.cr_n_per_rad
    s = 1j * 2 * math.pi * f
    tau = car.sigma_m / V
    G = car.stiction[0] * np.exp(-1j * math.radians(car.stiction[1]))
    # Fyf = cf (1 - (v + a r)/V) / Df,  Fyr = -cr (v - b r)/V / Dr
    Df, Dr = tau * s + 1 + cf * car.e_f_rad_per_n * G, tau * s + 1
    # m (s v + V r) = Fyf + Fyr ;  Iz s r = a Fyf - b Fyr
    M = np.array([[m * s + cf / (V * Df) + cr / (V * Dr), m * V + cf * a / (V * Df) - cr * b / (V * Dr)],
                  [a * cf / (V * Df) - b * cr / (V * Dr), car.iz * s + a * a * cf / (V * Df) + b * b * cr / (V * Dr)]])
    rhs = np.array([cf / Df, a * cf / Df])
    v, r = np.linalg.solve(M, rhs)
    fyf = cf * (1 - (v + a * r) / V) / Df
    fyr = -cr * (v - b * r) / V / Dr
    return (fyf + fyr) / m


def test_the_defaults_and_the_yaw_inertia():
    c = sr.SingleTrack()
    assert (c.mass_kg, c.a_m, c.b_m, c.iz_kgm2, c.speed_ms, c.cf_n_per_rad, c.cr_n_per_rad, c.sigma_m,
            c.e_f_rad_per_n, c.stiction) == (300.0, 0.8476, 0.7824, None, 15.0, 38000.0, 40000.0, 0.47, 0.0,
                                             (1.0, 0.0))
    assert c.iz == pytest.approx(300.0 * 0.8476 * 0.7824) and CAR.iz == 180.0


def test_the_response_by_hand_elimination():
    f = np.array([0.2, 1.0, 2.5, 6.0])
    H = sr.lateral_accel_frf(CAR, f)
    for fi, h in zip(f, H):
        assert h == pytest.approx(_frf(CAR, fi), rel=1e-10)
    rigid = replace(CAR, sigma_m=0.0)
    assert sr.lateral_accel_frf(rigid, 1.0)[0] == pytest.approx(_frf(replace(CAR, sigma_m=1e-12), 1.0), rel=1e-8)


def test_the_steady_state_gain_with_compliance_steer_in_series():
    for car in (sr.SingleTrack(), replace(sr.SingleTrack(), e_f_rad_per_n=5e-6, speed_ms=22.0)):
        cf = car.cf_n_per_rad / (1 + car.cf_n_per_rad * car.e_f_rad_per_n)
        L = car.a_m + car.b_m
        K = car.mass_kg / L * (car.b_m / cf - car.a_m / car.cr_n_per_rad)
        V = car.speed_ms
        h = sr.lateral_accel_frf(car, 1e-5)[0]
        assert h.real == pytest.approx(V * V / (L + K * V * V), rel=1e-6) and abs(h.imag) < 1e-5 * h.real


def test_the_lag_in_degrees_and_milliseconds():
    r = sr.lag(CAR, 2.5)
    h0, h = sr.lateral_accel_frf(CAR, [1e-3, 2.5])
    ph = math.degrees(np.angle(h / h0))
    assert r == pytest.approx({"phase_deg": -ph, "lag_ms": -ph / (360 * 2.5) * 1000, "gain_ratio": abs(h) / abs(h0)})
    assert r["phase_deg"] > 0
    big = replace(CAR, sigma_m=3.0, speed_ms=4.0)
    p = sr.lag(big, 8.0)["phase_deg"]
    assert -180.0 <= -p < 180.0                      # wrapped into one turn


def test_the_lag_budget_removes_each_term_in_turn():
    b = sr.lag_budget(CAR)
    L = lambda c: sr.lag(c, 2.5)["lag_ms"]
    full = L(CAR)
    assert b == pytest.approx({"total_ms": full, "relaxation_ms": full - L(replace(CAR, sigma_m=0.0)),
                               "compliance_ms": full - L(replace(CAR, e_f_rad_per_n=0.0)),
                               "stiction_ms": full - L(replace(CAR, stiction=(1.0, 0.0))),
                               "chassis_ms": L(replace(CAR, sigma_m=0.0, e_f_rad_per_n=0.0, stiction=(1.0, 0.0)))})
    assert sr.lag_budget(CAR, 1.0)["total_ms"] == pytest.approx(sr.lag(CAR, 1.0)["lag_ms"])


def test_axle_cornering_stiffness_is_the_two_wheel_secant():
    class T:
        def fy(self, a, fz):
            return -fz * 20.0 * math.sin(a)
    assert sr.axle_cornering_stiffness(T(), 700.0) == pytest.approx(2 * 700 * 20 * math.sin(math.radians(1)) /
                                                                     math.radians(1))
    assert sr.axle_cornering_stiffness(T(), 700.0, 3.0) == pytest.approx(2 * 700 * 20 * math.sin(math.radians(3)) /
                                                                          math.radians(3))


def test_the_yaw_mode_from_the_characteristic_polynomial():
    out = sr.yaw_mode(CAR)
    assert [o["speed_ms"] for o in out] == [10.0, 15.0, 20.0, 25.0]
    m, a, b, cf, cr, iz = CAR.mass_kg, CAR.a_m, CAR.b_m, CAR.cf_n_per_rad, CAR.cr_n_per_rad, CAR.iz
    for o in out:
        V = o["speed_ms"]
        tr = -(cf + cr) / (m * V) - (a * a * cf + b * b * cr) / (iz * V)
        det = ((cf + cr) * (a * a * cf + b * b * cr) - (a * cf - b * cr) ** 2) / (m * iz * V * V) \
            - (a * cf - b * cr) / iz
        assert o["fn_hz"] == pytest.approx(math.sqrt(det) / (2 * math.pi))
        assert o["zeta"] == pytest.approx(-tr / (2 * math.sqrt(det)))
        assert o["yaw_damping_nms_per_rad"] == pytest.approx((a * a * cf + b * b * cr) / V)
    assert out[0]["yaw_damping_nms_per_rad"] > out[-1]["yaw_damping_nms_per_rad"]     # falls as 1 / V
    over = replace(CAR, cf_n_per_rad=90000.0, cr_n_per_rad=20000.0)
    fast = sr.yaw_mode(over, (60.0,))[0]
    assert fast["zeta"] is None and math.isnan(fast["fn_hz"])
    assert len(sr.yaw_mode(CAR, (12.0,))) == 1


def test_the_time_response_matches_an_independent_integrator():
    car = sr.SingleTrack(iz_kgm2=150.0)
    r = sr.lateral_accel_split(car, delta_deg=2.0, gust_n=300.0, gust_s=0.4, t_end=1.5, dt=0.002)
    m, a, b, cf, cr, iz, V = 300.0, car.a_m, car.b_m, car.cf_n_per_rad, car.cr_n_per_rad, 150.0, 15.0
    d = math.radians(2.0)

    def rhs(t, y):
        v, rr = y
        ff, fr = cf * (d - (v + a * rr) / V), cr * (-(v - b * rr) / V)
        fw = 300.0 if t < 0.4 else 0.0
        return [(ff + fr + fw) / m - V * rr, (a * ff - b * fr) / iz]
    sol = solve_ivp(rhs, (0, 1.5), [0.0, 0.0], t_eval=r["t"], rtol=1e-10, atol=1e-12, max_step=0.002)
    assert r["t"][0] == 0.0 and r["t"][-1] == pytest.approx(1.5) and len(r["t"]) == 751
    assert np.degrees(sol.y[1]) == pytest.approx(r["r_degs"], abs=2e-3)
    assert np.degrees(sol.y[0] / V) == pytest.approx(r["beta_deg"], abs=2e-3)
    vd = np.array([rhs(t, y)[0] for t, y in zip(sol.t, sol.y.T)])
    assert r["v_betadot"] == pytest.approx(vd, abs=1e-2)
    assert r["v_r"] == pytest.approx(V * sol.y[1], abs=1e-3) and r["ay"] == pytest.approx(r["v_betadot"] + r["v_r"])
    assert r["ay"][0] == pytest.approx((cf * d + 300.0) / m)
    steady = sr.lateral_accel_split(car)
    L = a + b
    K = m / L * (b / cf - a / cr)
    assert steady["v_r"][-1] == pytest.approx(V * V / (L + K * V * V) * math.radians(2.0), rel=1e-3)
    assert len(steady["t"]) == 2001 and abs(steady["v_betadot"][-1]) < 1e-3


def test_the_paper_car_from_the_tire():
    c = sr.tire_derived_single_track()
    assert (c.cf_n_per_rad / 1e3, c.cr_n_per_rad / 1e3) == pytest.approx((41.5, 44.0), abs=0.05)
    assert sr.lag(c, 2.5)["lag_ms"] == pytest.approx(58.3, abs=0.05)
    assert sr.yaw_mode(c, (15.0,))[0]["fn_hz"] == pytest.approx(3.0, abs=0.05)

    class T:
        def fy(self, a, fz):
            return -fz * 30.0 * a
    t = sr.tire_derived_single_track(T(), mass_kg=250.0, weight_front=0.45, alpha_deg=2.0, speed_ms=20.0)
    assert t.cf_n_per_rad == pytest.approx(2 * 30.0 * 250 * 9.81 * 0.45 / 2)
    assert t.cr_n_per_rad == pytest.approx(2 * 30.0 * 250 * 9.81 * 0.55 / 2)
    assert (t.mass_kg, t.speed_ms) == (250.0, 20.0)


def _rk4_split(car, delta_deg, gust_n, gust_s, t_end, dt):
    """Classical RK4 on [v, r] written with arrays, the gust a step in time."""
    m, a, b, cf, cr, iz, V = car.mass_kg, car.a_m, car.b_m, car.cf_n_per_rad, car.cr_n_per_rad, car.iz, car.speed_ms
    d = math.radians(delta_deg)

    def f(t, x):
        v, r = x
        Ff, Fr = cf * (d - (v + a * r) / V), cr * (-(v - b * r) / V)
        return np.array([(Ff + Fr + (gust_n if t < gust_s else 0.0)) / m - V * r, (a * Ff - b * Fr) / iz])
    n = int(round(t_end / dt)) + 1
    T = np.linspace(0.0, t_end, n)
    X = np.zeros((n, 2))
    for i in range(n - 1):
        t, x = T[i], X[i]
        k1 = f(t, x); k2 = f(t + dt / 2, x + dt / 2 * k1); k3 = f(t + dt / 2, x + dt / 2 * k2); k4 = f(t + dt, x + dt * k3)
        X[i + 1] = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    vd = np.array([f(t, x)[0] for t, x in zip(T, X)])
    return T, X, vd


def test_the_split_is_classical_rk4_with_a_gust_between_samples():
    T, X, vd = _rk4_split(CAR, 1.5, 400.0, 0.0505, 0.2, 0.01)
    r = sr.lateral_accel_split(CAR, delta_deg=1.5, gust_n=400.0, gust_s=0.0505, t_end=0.2, dt=0.01)
    assert r["t"] == pytest.approx(T) and r["v_betadot"] == pytest.approx(vd, rel=1e-12, abs=1e-15)
    assert r["r_degs"] == pytest.approx(np.degrees(X[:, 1]), rel=1e-12, abs=1e-15)
    d = sr.lateral_accel_split(CAR, gust_n=300.0, t_end=0.6, dt=0.01)
    T, X, vd = _rk4_split(CAR, 2.0, 300.0, 0.5, 0.6, 0.01)
    assert d["ay"] == pytest.approx(vd + CAR.speed_ms * X[:, 1], rel=1e-12, abs=1e-15)
    # a gust ending between t + dt/3 and t + dt/2 shows where the midpoint stages sample it
    T, X, vd = _rk4_split(CAR, 1.5, 400.0, 0.054, 0.2, 0.01)
    r = sr.lateral_accel_split(CAR, delta_deg=1.5, gust_n=400.0, gust_s=0.054, t_end=0.2, dt=0.01)
    assert r["r_degs"] == pytest.approx(np.degrees(X[:, 1]), rel=1e-12, abs=1e-15)
    z = sr.lateral_accel_split(CAR, t_end=0.6, dt=0.01)                      # no gust by default
    T, X, vd = _rk4_split(CAR, 2.0, 0.0, 0.5, 0.6, 0.01)
    assert z["v_betadot"] == pytest.approx(vd, rel=1e-12, abs=1e-15)


def test_the_budget_strips_stiction_from_the_bare_chassis_and_slow_modes():
    full = sr.lag_budget(CAR, 2.5)
    bare = sr.lag(replace(CAR, sigma_m=0.0, e_f_rad_per_n=0.0, stiction=(1.0, 0.0)), 2.5)["lag_ms"]
    assert full["chassis_ms"] == pytest.approx(bare)
    soft = replace(CAR, cf_n_per_rad=1e-2 * CAR.cf_n_per_rad, cr_n_per_rad=1e-2 * CAR.cr_n_per_rad)
    row = sr.yaw_mode(soft, speeds_ms=(5.0,))[0]
    m, a, b, cf, cr, iz, V = soft.mass_kg, soft.a_m, soft.b_m, soft.cf_n_per_rad, soft.cr_n_per_rad, soft.iz, 5.0
    A = np.array([[-(cf + cr) / (m * V), -(a * cf - b * cr) / (m * V) - V],
                  [-(a * cf - b * cr) / (iz * V), -(a * a * cf + b * b * cr) / (iz * V)]])
    det = np.linalg.det(A)
    assert 0 < det < 1 and row["fn_hz"] == pytest.approx(math.sqrt(det) / (2 * math.pi))
    assert row["zeta"] == pytest.approx(-np.trace(A) / (2 * math.sqrt(det)))


def test_the_tire_derived_car_at_another_slip():
    from suspension.tiremodel import default_tire
    t = default_tire()
    c = sr.tire_derived_single_track(t, alpha_deg=2.5)
    assert c.cf_n_per_rad == pytest.approx(sr.axle_cornering_stiffness(t, 300 * 9.81 * 0.48 / 2, 2.5))
    assert c.cr_n_per_rad == pytest.approx(sr.axle_cornering_stiffness(t, 300 * 9.81 * 0.52 / 2, 2.5))
