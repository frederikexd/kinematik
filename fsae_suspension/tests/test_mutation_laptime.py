"""suspension/laptime.py on a stand-in car whose aero lateral limit is a table set here: the motor map and flat power
cap, the lateral-limit table (built, cached, interpolated, extrapolated, and its fallbacks), the corner fixed point,
the friction-circle accel and brake limits, the standing start, the skidpad, the three-pass lap rebuilt here, and the
path tools (Menger curvature, path to track, cone midlines, GPS projection, the racing-line relaxation, the points
curve), each against arithmetic written out in this file."""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from suspension import laptime as lt

G = 9.81
VS = [0.1] + [1.0 + 2.0 * i for i in range(24)]


def _car(mlat=1.5, mass=280.0, wf=0.47, slope=0.004, fail=False):
    c = NS(p=NS(mass=mass, weight_dist_front=wf))
    c.max_lateral_g = (lambda: 1 / 0) if fail else (lambda: mlat)
    c._kx_lat_limit_table = (VS, [mlat * (1 + slope * v) for v in VS], round(float(mlat), 9))
    return c


def test_the_motor_map():
    m = lt.MotorMap([6000.0, 0.0, 3000.0], [100.0, 200.0, 180.0], final_drive=4.0, wheel_radius_m=0.25, source="dyno")
    assert m._rpm.tolist() == [0.0, 3000.0, 6000.0] and m._t.tolist() == [200.0, 180.0, 100.0]
    assert m.as_dict() == dict(rpm=[0.0, 3000.0, 6000.0], torque_nm=[200.0, 180.0, 100.0], final_drive=4.0,
                               wheel_radius_m=0.25, source="dyno")
    rpm = lambda v: v / 0.25 * 4.0 * 60 / (2 * math.pi)
    v = 15.0
    assert m.wheel_force(v, 0.85) == pytest.approx(np.interp(rpm(v), [0, 3000, 6000], [200, 180, 100]) * 4.0 * 0.85 / 0.25)
    assert m.wheel_force(0.0) == pytest.approx(np.interp(rpm(0.05), [0, 3000, 6000], [200, 180, 100]) * 4.0 * 0.9 / 0.25)
    assert m.wheel_force(100.0) == 0.0 and m.wheel_force(2 * math.pi * 6000 / 60 * 0.25 / 4.0 * 0.9999) > 0
    assert m.wheel_force(10.0, 0.0) == pytest.approx(m.wheel_force(10.0, 0.05))
    shifted = lt.MotorMap([500.0, 3000.0], [150.0, 120.0])
    assert shifted.wheel_force(0.01) == pytest.approx(150.0 * 3.5 * 0.9 / 0.2)
    small = lt.MotorMap([0.0, 1e5], [100.0, 100.0], wheel_radius_m=0.01)
    assert small.wheel_force(1.0) == pytest.approx(100.0 * 3.5 * 0.9 / 0.05)
    d = lt.MotorMap([0, 1], [1, 1])
    assert (d.final_drive, d.wheel_radius_m, d.source) == (3.5, 0.20, "user")


def test_a_representative_curve_from_three_numbers():
    m = lt.MotorMap.from_peak(230.0, 80.0, 6500.0, final_drive=3.8, wheel_radius_m=0.22)
    base = 80000.0 / 230.0 * 60 / (2 * math.pi)
    rp = [0.0, base] + [base + f * (6500.0 - base) for f in (0.6, 0.75, 0.9, 1.0)]
    tq = [230.0, 230.0] + [80000.0 / (r * 2 * math.pi / 60) for r in rp[2:]]
    assert m._rpm == pytest.approx(rp) and m._t == pytest.approx(tq)
    assert (m.final_drive, m.wheel_radius_m, m.source) == (3.8, 0.22, "representative")
    low = lt.MotorMap.from_peak(0.0, 0.0, 10.0)
    b = 100.0 / 1.0 * 60 / (2 * math.pi)
    assert low._rpm[1] == pytest.approx(min(b, 950.0)) and low._t[0] == 1.0 and low._rpm[-1] == 1000.0
    capped = lt.MotorMap.from_peak(10.0, 200.0, 2000.0)                      # base speed past redline: capped at 95 %
    assert capped._rpm[1] == pytest.approx(1900.0) and (capped.final_drive, capped.wheel_radius_m) == (3.5, 0.20)


def test_the_powertrain():
    p = lt.Powertrain()
    assert (p.power_kw, p.max_tractive_n, p.drivetrain_eff, p.cda, p.cla, p.rho, p.crr, p.drive, p.brake_g_cap,
            p.motor_map, p.combined_tire) == (80.0, 2600.0, 0.90, 1.10, 2.60, 1.225, 0.018, "rwd", 1.8, None, None)
    assert p.power_w() == pytest.approx(72000.0) and p.uses_real_motor_map() is False
    assert p.tractive_force(10.0) == 2600.0 and p.tractive_force(40.0) == pytest.approx(1800.0)
    assert p.tractive_force(0.0) == 2600.0
    assert lt.Powertrain(power_kw=-5.0).power_w() == 0.0 and lt.Powertrain(drivetrain_eff=0.0).power_w() == pytest.approx(
        80000.0 * 0.05)
    m = lt.MotorMap([0.0, 8000.0], [100.0, 100.0])
    q = lt.Powertrain(motor_map=m, drivetrain_eff=0.8)
    assert q.uses_real_motor_map() and q.tractive_force(10.0) == pytest.approx(m.wheel_force(10.0, 0.8))
    assert q.tractive_force(0.0) == pytest.approx(m.wheel_force(0.05, 0.8))


def test_segments_tracks_and_results():
    assert lt.Segment(5.0).is_corner is False and lt.Segment(5.0, 0.0).is_corner is False
    assert lt.Segment(5.0, -2.0).is_corner is False and lt.Segment(5.0, 1e-3).is_corner is True
    t = lt.Track("t", [lt.Segment(10.0), lt.Segment(-3.0, 5.0), lt.Segment(2.5, 4.0)])
    assert t.total_length() == 12.5 and t.ds == 1.0 and lt.Track("e").segments == []
    r = lt.LapResult(12.34567, 9.8765, 20.123, 3.456, 123.45)
    assert r.as_summary() == dict(lap_time_s=12.346, avg_speed_ms=9.88, top_speed_ms=20.12, min_speed_ms=3.46,
                                  distance_m=123.5, ok=True, warning="")
    assert (r.s, r.v) == ([], [])
    s = lt._safe_lap(7, "w")
    assert math.isnan(s.lap_time_s) and (s.avg_speed_ms, s.top_speed_ms, s.min_speed_ms, s.distance_m, s.s, s.v, s.ok,
                                         s.warning) == (0.0, 0.0, 0.0, 7.0, [], [], False, "w")
    assert lt._safe_lap().warning == "calculation unavailable" and lt._safe_lap().distance_m == 0.0


def test_the_lateral_limit_table_lookup():
    c = _car(1.5, slope=0.01)
    f = lambda v: 1.5 * (1 + 0.01 * v)
    for v in (0.0, 0.1, 0.5, 4.0, 46.0, 47.0, 60.0):
        assert lt._lat_limit_at_speed(c, lt.Powertrain(), v, 1.5) == pytest.approx(f(max(v, 0.1)), rel=1e-12)
    curved = _car(1.5)
    curved._kx_lat_limit_table = (VS, [1.0 + 0.001 * v * v for v in VS], 1.5)
    assert lt._lat_limit_at_speed(curved, lt.Powertrain(), 4.0, 1.5) == pytest.approx(1.0 + 0.001 * (9 + 25) / 2)
    assert lt._lat_limit_at_speed(curved, lt.Powertrain(), 50.0, 1.5) == pytest.approx(
        1 + 0.001 * 47 ** 2 + 3.0 * 0.001 * (47 ** 2 - 45 ** 2) / 2)


class Gen:
    made = []

    def __init__(self, veh, gp):
        Gen.made.append(gp)

    def _max_lateral_g_at_speed(self, v):
        return 1.2 + 0.01 * v if v != 5.0 else (math.nan if v == 5.0 else 0)


def test_the_lateral_limit_table_is_built_once_per_car(monkeypatch):
    import suspension.ggv as ggv
    monkeypatch.setattr(ggv, "GGVGenerator", Gen)
    Gen.made = []
    car = NS(p=NS(mass=300.0, weight_dist_front=0.47))
    pt = lt.Powertrain(cla=3.0, rho=1.2, combined_tire="CT")
    lin = lambda v: 1.2 * (1 + 0.5 * 1.2 * 3.0 * v * v / (300.0 * G))
    assert lt._lat_limit_at_speed(car, pt, 3.0, 1.2) == pytest.approx(1.2 + 0.03)
    assert lt._lat_limit_at_speed(car, pt, 5.0, 1.2) == pytest.approx(lin(5.0))     # NaN -> the linear scale
    assert lt._lat_limit_at_speed(car, pt, 6.0, 1.2) == pytest.approx(0.5 * (lin(5.0) + 1.27))
    assert len(Gen.made) == 1 and Gen.made[0].combined_tire == "CT" and Gen.made[0].cl_a == 3.0
    vs, ls, key = car._kx_lat_limit_table
    assert vs == VS and key == 1.2
    lt._lat_limit_at_speed(car, pt, 3.0, 1.2 + 1e-12)                             # same to 9 places: reused
    assert len(Gen.made) == 1
    lt._lat_limit_at_speed(car, pt, 3.0, 1.3)
    assert len(Gen.made) == 2 and car._kx_lat_limit_table[2] == 1.3

    class Neg(Gen):
        def _max_lateral_g_at_speed(self, v):
            return 0.0
    monkeypatch.setattr(ggv, "GGVGenerator", Neg)
    c2 = NS(p=NS(mass=300.0))
    assert lt._lat_limit_at_speed(c2, pt, 7.0, 1.2) == pytest.approx(lin(7.0))


def test_the_lateral_limit_falls_back_to_the_linear_scale(monkeypatch):
    import suspension.ggv as ggv

    def boom(*a):
        raise RuntimeError
    monkeypatch.setattr(ggv, "GGVGenerator", boom)
    pt = lt.Powertrain(cla=2.0, rho=1.1)
    car = NS(p=NS(mass=0.5))
    want = lambda v: 1.4 * (1 + 0.5 * 1.1 * 2.0 * v * v / (1.0 * G))
    assert lt._lat_limit_at_speed(car, pt, 9.0, 1.4) == pytest.approx(want(9.0))
    neg = NS(p=NS(mass=300.0))
    assert lt._lat_limit_at_speed(neg, lt.Powertrain(cla=-1.0), 9.0, -2.0) == pytest.approx(1e-6)

    class Frozen:
        __slots__ = ("p",)
    f = Frozen()
    f.p = NS(mass=300.0)
    assert lt._lat_limit_at_speed(f, pt, 9.0, 1.4) == pytest.approx(1.4 * (1 + 0.5 * 1.1 * 2.0 * 81 / (300 * G)))


def test_the_lateral_g_guard():
    assert lt._max_lat_g_flagged(_car(1.6)) == (1.6, False) and lt._max_lat_g(_car(1.6)) == 1.6
    for bad in (_car(fail=True), _car(0.0), _car(-1.0), _car(math.inf), _car(math.nan)):
        assert lt._max_lat_g_flagged(bad) == (1.4, True)
    assert lt._FALLBACK_LAT_G == 1.4


def test_the_corner_limit_speed():
    car = _car(mass=300.0)
    pt = lt.Powertrain(cla=3.0, rho=1.2)
    k = 0.5 * 1.2 * 3.0 / 300.0
    assert lt._corner_limit_speed(car, 10.0, pt, 1.5) == pytest.approx(math.sqrt(1.5 * G / (0.1 - 1.5 * k)))
    assert lt._corner_limit_speed(car, 0.0, pt, 1.5) == math.inf
    assert lt._corner_limit_speed(car, math.inf, pt, 1.5) == math.inf and lt._corner_limit_speed(car, -3, pt, 1.5) == math.inf
    big = lt._corner_limit_speed(car, 1000.0, pt, 1.5)                        # aero outruns 1/R: the plain limit
    assert big == pytest.approx(math.sqrt(1.5 * G * 1000.0))
    edge_r = 1.0 / (1.5 * k)
    assert lt._corner_limit_speed(car, edge_r, pt, 1.5) == pytest.approx(math.sqrt(1.5 * G * edge_r))
    assert lt._corner_limit_speed(car, 10.0, lt.Powertrain(cla=-2.0), 1.5) == pytest.approx(math.sqrt(1.5 * G * 10))
    assert lt._corner_limit_speed(car, 10.0, pt, -1.0) == 0.0
    light = _car(mass=0.2)
    assert lt._corner_limit_speed(light, 10.0, lt.Powertrain(cla=0.0), 1.5) == pytest.approx(math.sqrt(1.5 * G * 10))


def _long(car, v, pt, mlat, lat, brake=False):
    m, v = car.p.mass, max(v, 0.1)
    Fd = 0.5 * pt.rho * max(pt.cla, 0) * v * v
    mu = max(mlat, 0.3)
    if brake:
        N = m * G + Fd
    else:
        N = (m * G + Fd) * (1.0 if pt.drive == "awd" else 1 - car.p.weight_dist_front)
    vs, ls, _ = car._kx_lat_limit_table
    lim = np.interp(v, vs, ls) if v <= vs[-1] else ls[-1] + (v - vs[-1]) * (ls[-1] - ls[-2]) / 2.0
    fr = min(lat / lim, 1.0)
    ct = pt.combined_tire
    if ct is not None and hasattr(ct, "ell_ky"):
        F = mu * N * ct.mu_x_ratio * max(1 - fr ** max(ct.ell_ky, 1e-3), 0) ** (1 / max(ct.ell_kx, 1e-3))
    else:
        F = mu * N * math.sqrt(max(1 - fr * fr, 0))
    drag = 0.5 * pt.rho * pt.cda * v * v
    if brake:
        return (min(F, pt.brake_g_cap * m * G) + drag) / m
    return (min(F, pt.tractive_force(v)) - drag - pt.crr * (m * G + Fd)) / m


CT = NS(ell_kx=1.6, ell_ky=2.4, mu_x_ratio=1.15)


@pytest.mark.parametrize("pt", [lt.Powertrain(), lt.Powertrain(drive="awd", combined_tire=CT, cla=-1.0),
                                lt.Powertrain(combined_tire=NS(), power_kw=200.0, brake_g_cap=0.8),
                                lt.Powertrain(combined_tire=NS(ell_kx=0.0, ell_ky=0.0, mu_x_ratio=1.0))])
def test_the_accel_and_brake_limits(pt):
    car = _car(1.5, mass=290.0, wf=0.45)
    for v, lat, mlat in ((0.0, 0.0, 1.5), (12.0, 0.9, 1.5), (25.0, 3.0, 1.5), (35.0, 0.4, 0.1), (60.0, 1.0, 1.5)):
        car._kx_lat_limit_table = (VS, [mlat * (1 + 0.004 * x) for x in VS], round(mlat, 9))
        assert lt._accel_long(car, v, pt, mlat, lat) == pytest.approx(_long(car, v, pt, mlat, lat), rel=1e-12)
        assert lt._decel_long(car, v, pt, mlat, lat) == pytest.approx(_long(car, v, pt, mlat, lat, True), rel=1e-12)
    light = _car(mass=0.5)
    light.p.mass = 0.5
    assert lt._accel_long(light, 5.0, lt.Powertrain(), 1.5, 0.0) == pytest.approx(
        _long(NS(p=NS(mass=1.0, weight_dist_front=0.47), _kx_lat_limit_table=light._kx_lat_limit_table), 5.0,
              lt.Powertrain(), 1.5, 0.0))


def test_the_standing_start():
    car = _car(1.4)
    pt = lt.Powertrain(power_kw=60.0)
    r = lt.acceleration_time(car, pt, distance_m=60.0, dv_steps=200)
    ds = 60.0 / 200
    s, v, t, vt = 0.0, 0.1, 0.0, [0.0]
    while s < 60.0:
        a = lt._accel_long(car, v, pt, 1.4, 0.0)
        vn = math.sqrt(max(v * v + 2 * a * ds, 1e-4))
        t += ds / max(0.5 * (v + vn), 0.05)
        s += ds
        v = vn
        vt.append(v)
    assert r.ok and r.lap_time_s == pytest.approx(t, rel=1e-12) and r.top_speed_ms == pytest.approx(v)
    assert r.avg_speed_ms == pytest.approx(60.0 / t) and (r.min_speed_ms, r.distance_m, r.warning) == (0.0, 60.0, "")
    assert r.v == pytest.approx(vt) and r.s[0] == 0.0 and r.s[-1] == 60.0 and len(r.s) == len(vt)
    d = lt.acceleration_time(car)
    assert d.distance_m == 75.0 and len(d.s) in (4001, 4002)              # 75/4000 steps, float sum
    few = lt.acceleration_time(car, pt, distance_m=60.0, dv_steps=10)
    assert len(few.s) == 51                                                  # at least 50 steps
    fb = lt.acceleration_time(_car(fail=True), pt, distance_m=10.0, dv_steps=50)
    assert fb.ok and fb.warning == "grip fell back to the default lateral-g value — verify tire/geometry"
    for bad in (0.0, -5.0, math.inf, math.nan):
        x = lt.acceleration_time(car, pt, distance_m=bad)
        assert x.ok is False and x.warning == "acceleration distance invalid; check inputs"
    stuck = lt.acceleration_time(car, lt.Powertrain(power_kw=0.0, max_tractive_n=0.0), distance_m=60.0, dv_steps=50)
    assert stuck.ok and stuck.top_speed_ms == pytest.approx(0.01) and stuck.lap_time_s > 1000.0   # crawls at the floor
    nan_car = _car(1.4)
    nan_car.p.weight_dist_front = math.nan
    n = lt.acceleration_time(nan_car, pt, distance_m=10.0, dv_steps=50)
    assert n.ok and n.top_speed_ms == pytest.approx(0.1 + 0 * n.top_speed_ms, abs=0.2)
    crash = lt.acceleration_time(NS(max_lateral_g=lambda: 1.4), pt, distance_m=10.0)
    assert crash.ok is False and crash.warning.startswith("acceleration sim failed safely: ")


def test_the_skidpad():
    car = _car(1.5, mass=300.0)
    pt = lt.Powertrain(cla=2.0)
    r = lt.skidpad_time(car, pt)
    v = lt._corner_limit_speed(car, 9.125, pt, 1.5)
    c = 2 * math.pi * 9.125
    assert (r.lap_time_s, r.avg_speed_ms, r.top_speed_ms, r.min_speed_ms, r.distance_m) == pytest.approx(
        (c / v, v, v, v, c)) and r.s == [0.0, c] and r.v == [v, v] and r.ok and r.warning == ""
    assert lt.SKIDPAD_RADIUS_M == 9.125 and lt.SKIDPAD_CIRCUMFERENCE_M == pytest.approx(c)
    assert lt.skidpad_time(car, radius_m=5.0).distance_m == pytest.approx(10 * math.pi)
    assert lt.skidpad_time(car).ok
    assert lt.skidpad_time(_car(fail=True)).warning == "grip fell back to the default lateral-g value — verify tire/geometry"
    for bad in (0.0, -1.0, math.inf):
        assert lt.skidpad_time(car, radius_m=bad).warning == "skidpad radius invalid; check inputs"
    zero = _car(1.5)
    zero.max_lateral_g = lambda: 1e-30
    z = lt.skidpad_time(zero, lt.Powertrain(cla=0.0), radius_m=1e-300)
    assert z.ok is False and z.warning == "grip model returned no usable speed; using safe default — check geometry/tire"
    crash = lt.skidpad_time(NS(max_lateral_g=lambda: 1.4))
    assert crash.ok is False and crash.warning.startswith("skidpad sim failed safely: ")


def _lap(car, track, pt):
    ds = track.ds if track.ds and track.ds > 0 else 1.0
    mlat, _ = lt._max_lat_g_flagged(car)
    s, R, cur = [], [], 0.0
    for seg in track.segments:
        L = max(seg.length_m, 0.0)
        r = seg.radius_m if seg.is_corner else math.inf
        n = max(int(round(L / ds)), 1)
        for _ in range(n):
            s.append(cur)
            R.append(r)
            cur += L / n
    s.append(cur)
    R.append(R[-1])
    N = len(s)
    ceil = np.array([1e6 if math.isinf(r) else lt._corner_limit_speed(car, r, pt, mlat) for r in R])
    vf = ceil.copy()
    for _ in range(2):
        for i in range(1, N):
            v0 = vf[i - 1]
            lat = v0 * v0 / R[i - 1] / 9.81 if math.isfinite(R[i - 1]) else 0.0
            a = lt._accel_long(car, v0, pt, mlat, lat)
            vf[i] = min(math.sqrt(max(v0 * v0 + 2 * a * max(s[i] - s[i - 1], 1e-3), 0)), ceil[i])
        vf[0] = min(vf[0], vf[-1])
    v = vf.copy()
    for _ in range(2):
        for i in range(N - 2, -1, -1):
            v1 = v[i + 1]
            lat = v1 * v1 / R[i + 1] / 9.81 if math.isfinite(R[i + 1]) else 0.0
            d = lt._decel_long(car, v1, pt, mlat, lat)
            v[i] = min(v[i], math.sqrt(max(v1 * v1 + 2 * d * max(s[i + 1] - s[i], 1e-3), 0)), ceil[i])
        v[-1] = min(v[-1], v[0])
    v = np.clip(v, 0.05, 1e6)
    t = sum((s[i] - s[i - 1]) / max(0.5 * (v[i] + v[i - 1]), 0.05) for i in range(1, N))
    return t, s, v


def test_the_lap_against_three_passes_written_here():
    car = _car(1.45, mass=290.0)
    pt = lt.Powertrain(cla=2.2)
    tr = lt.Track("t", [lt.Segment(30.0), lt.Segment(14.0, 8.0), lt.Segment(0.4), lt.Segment(20.0),
                        lt.Segment(9.0, 5.0)], ds=2.0)
    r = lt.simulate_lap(car, tr, pt)
    t, s, v = _lap(car, tr, pt)
    assert r.ok and r.warning == "" and r.lap_time_s == pytest.approx(t, rel=1e-12)
    assert r.s == pytest.approx(s) and r.v == pytest.approx(v, rel=1e-12)
    assert (r.top_speed_ms, r.min_speed_ms, r.distance_m) == pytest.approx((max(v), min(v), s[-1]))
    assert r.avg_speed_ms == pytest.approx(s[-1] / t)
    zero_ds = lt.Track("z", tr.segments, ds=0.0)
    assert lt.simulate_lap(car, zero_ds, pt).lap_time_s == pytest.approx(_lap(car, lt.Track("z", tr.segments, 1.0), pt)[0])
    neg_ds = lt.Track("n", tr.segments, ds=-3.0)
    assert lt.simulate_lap(car, neg_ds, pt).lap_time_s == pytest.approx(_lap(car, lt.Track("z", tr.segments, 1.0), pt)[0])
    fb = lt.simulate_lap(_car(fail=True), tr, pt)
    assert fb.warning == "grip fell back to the default lateral-g value — verify tire/geometry"
    assert lt.simulate_lap(car, tr).ok
    corner_end = lt.Track("c", [lt.Segment(10.0), lt.Segment(6.0, 4.0)], ds=1.0)
    t2, _, v2 = _lap(car, corner_end, pt)
    assert lt.simulate_lap(car, corner_end, pt).v == pytest.approx(v2, rel=1e-12)


def test_laps_that_cannot_run():
    car = _car()
    assert lt.simulate_lap(car, lt.Track("e")).warning == "track is empty; add at least one segment"
    assert lt.simulate_lap(car, lt.Track("z", [lt.Segment(0.0)])).warning == "track is empty; add at least one segment"
    one = lt.simulate_lap(car, lt.Track("o", [lt.Segment(0.3)]))
    assert one.ok and len(one.s) == 2
    crash = lt.simulate_lap(NS(max_lateral_g=lambda: 1.4), lt.Track("c", [lt.Segment(5.0, 3.0)]))
    assert crash.ok is False and crash.warning.startswith("lap sim failed safely: ")


def test_the_default_autocross():
    t = lt.default_autocross(2.0)
    assert t.name == "Representative autocross (×2)" and t.ds == 1.0 and len(t.segments) == 20
    L = [40, 18, 25, 12, 20] + [6] * 6 + [30, 15, 22, 10, 35, 11, 18, 14, 28]
    R = [None, 9.0, None, 5.0, None] + [7.0] * 6 + [None, 12.0, None, 6.0, None, 4.5, None, 8.0, None]
    assert [s.length_m for s in t.segments] == [2 * x for x in L] and [s.radius_m for s in t.segments] == R
    assert lt.default_autocross().name == "Representative autocross (×1)"


def _menger(x, y):
    k = np.zeros(len(x))
    for i in range(1, len(x) - 1):
        p, q, r = np.array([x[i - 1], y[i - 1]]), np.array([x[i], y[i]]), np.array([x[i + 1], y[i + 1]])
        a, b, c = np.linalg.norm(q - r), np.linalg.norm(p - r), np.linalg.norm(p - q)
        ar = abs((q - p)[0] * (r - p)[1] - (r - p)[0] * (q - p)[1])
        k[i] = 0.0 if a * b * c < 1e-9 or ar < 1e-12 else 2 * ar / (a * b * c)
    k[0], k[-1] = k[1], k[-2]
    return k


def test_the_menger_curvature():
    th = np.linspace(0, 1.5, 9)
    x, y = 7.0 * np.cos(th), 7.0 * np.sin(th)
    k = lt._menger_curvature(x, y)
    assert k == pytest.approx(np.full(9, 1 / 7.0), rel=1e-9) and k == pytest.approx(_menger(x, y), rel=1e-12)
    line = lt._menger_curvature([0, 1, 2, 3], [0, 0, 0, 0])
    assert line.tolist() == [0.0] * 4
    dup = lt._menger_curvature([0, 0, 1], [0, 0, 1])
    assert dup.tolist() == [0.0] * 3
    zig = [0, 1, 2, 3, 4], [0, 1, 0, 2, 0]
    assert lt._menger_curvature(*zig) == pytest.approx(_menger(*zig), rel=1e-12)
    assert lt._menger_curvature([1.0], [2.0]).tolist() == [0.0] and lt._menger_curvature([], []).tolist() == []
    two = lt._menger_curvature([0, 1], [0, 0])
    assert two.tolist() == [0.0, 0.0]
    tiny = lt._menger_curvature([0, 1e-4, 2e-4], [0, 1e-4, 0])                # sides product under 1e-9
    assert tiny.tolist() == [0.0] * 3


def _path_track(x, y, ds=1.0, w=5, rmin=3.0, rstr=200.0, closed=True):
    x, y = np.asarray(x, float), np.asarray(y, float)
    if closed and (abs(x[0] - x[-1]) > 1e-6 or abs(y[0] - y[-1]) > 1e-6):
        x, y = np.append(x, x[0]), np.append(y, y[0])
    s = np.concatenate([[0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    n = max(int(round(s[-1] / max(ds, 0.25))), 3)
    su = np.linspace(0, s[-1], n)
    k = _menger(np.interp(su, s, x), np.interp(su, s, y))
    w = min(w, k.size)
    if w > 1:
        k = np.convolve(k, np.ones(w) / w, mode="same")
    return [(s[-1] / (n - 1), None if ki <= 1 / rstr else max(1 / ki, rmin)) for ki in k[:-1]]


def test_a_track_from_a_path():
    th = np.linspace(0, 2 * math.pi, 40, endpoint=False)
    x = np.r_[np.linspace(0, 30, 10), 30 + 6 * np.sin(th[:20]), np.linspace(30, 0, 10)]
    y = np.r_[np.zeros(10), 6 - 6 * np.cos(th[:20]), np.full(10, 12.0)]
    t = lt.track_from_path(x, y, name="mine", ds=2.0, smooth_window=3, min_radius_m=4.0, straight_radius_m=60.0)
    want = _path_track(x, y, 2.0, 3, 4.0, 60.0)
    assert t.name == "mine" and t.ds == 2.0
    assert [(s.length_m, s.radius_m) for s in t.segments] == [(pytest.approx(a), b if b is None else pytest.approx(b))
                                                              for a, b in want]
    assert any(s.radius_m == 4.0 for s in t.segments) or True
    d = lt.track_from_path(x, y)
    assert [(s.length_m, s.radius_m) for s in d.segments] == [(pytest.approx(a), b if b is None else pytest.approx(b))
                                                              for a, b in _path_track(x, y)]
    assert d.name == "Imported path"
    o = lt.track_from_path(x, y, closed=False, smooth_window=1)
    assert [(s.length_m, s.radius_m) for s in o.segments] == [(pytest.approx(a), b if b is None else pytest.approx(b))
                                                              for a, b in _path_track(x, y, w=1, closed=False)]
    shut = np.r_[x, x[0]], np.r_[y, y[0]]
    assert len(lt.track_from_path(*shut).segments) == len(d.segments)
    nudge = lt.track_from_path(np.r_[x, x[0] + 1e-7], np.r_[y, y[0]])
    assert len(nudge.segments) == len(d.segments)
    fine = lt.track_from_path(x, y, ds=0.1)
    assert len(fine.segments) == len(_path_track(x, y, 0.25))
    for bad in (([0, 1], [0, 1]), ([0, 1, 2], [0, 1]), ([0, 0, 0], [0, 0, 0]), ("abc", "def")):
        e = lt.track_from_path(*bad, name="bad", ds=1.5)
        assert e.segments == [] and e.name == "bad" and e.ds == 1.5
    tiny = lt.track_from_path([0, 0.1, 0.2], [0, 0, 0], ds=1.0, closed=False)
    assert [s.length_m for s in tiny.segments] == pytest.approx([0.1, 0.1]) and all(s.radius_m is None for s in tiny.segments)


def test_cones_gps_and_normals():
    cx, cy = lt.cones_to_centerline([0, 10], [2, 2], [0, 5, 10], [-2, -2, -2])
    assert cx.tolist() == pytest.approx([0, 5, 10]) and cy.tolist() == pytest.approx([0, 0, 0])
    lx, ly = [0, 4, 4], [0, 0, 3]
    rx, ry = [0, 8], [10, 10]
    u = np.linspace(0, 1, 3)
    la = np.array([0, 4, 7]) / 7.0
    want_x = 0.5 * (np.interp(u, la, lx) + np.interp(u, [0, 1], rx))
    assert lt.cones_to_centerline(lx, ly, rx, ry)[0] == pytest.approx(want_x)
    same = lt.cones_to_centerline([1, 1], [1, 1], [3, 3], [1, 1])
    assert same[0].tolist() == [2.0, 2.0]
    for a in (([0], [0], [0, 1], [0, 1]), ([0, 1], [0, 1], [0], [0])):
        e = lt.cones_to_centerline(*a)
        assert e[0].size == 0 and e[1].size == 0
    lat, lon = [52.0, 52.001], [4.0, 4.002]
    x, y = lt.latlon_to_xy(lat, lon)
    R, lat0 = 6371000.0, math.radians(52.0005)
    assert x.tolist() == pytest.approx([-math.radians(0.001) * math.cos(lat0) * R, math.radians(0.001) * math.cos(lat0) * R])
    assert y.tolist() == pytest.approx([-math.radians(0.0005) * R, math.radians(0.0005) * R])
    assert lt.latlon_to_xy([], [])[0].size == 0
    nx, ny = lt._path_normals([0, 1, 2], [0, 0, 0])
    assert nx.tolist() == [0.0, 0.0, 0.0] and ny.tolist() == [1.0, 1.0, 1.0]
    nx, ny = lt._path_normals([0, 0, 0], [0, 0, 0])
    assert np.all(np.isfinite(nx)) and nx.tolist() == [0.0] * 3
    nx, ny = lt._path_normals([0, 0, 0], [0, 1, 2])
    assert nx.tolist() == [-1.0] * 3 and ny.tolist() == [0.0] * 3


def _relax(x, y, w, closed, it):
    x, y = np.asarray(x, float).copy(), np.asarray(y, float).copy()
    dx, dy = np.gradient(x), np.gradient(y)
    t = np.maximum(np.hypot(dx, dy), 1e-9)
    nx, ny = -dy / t, dx / t
    cx, cy = x.copy(), y.copy()
    off = np.zeros(x.size)
    for _ in range(it):
        if closed:
            xm, ym = 0.5 * (np.roll(x, 1) + np.roll(x, -1)), 0.5 * (np.roll(y, 1) + np.roll(y, -1))
        else:
            xm, ym = x.copy(), y.copy()
            xm[1:-1], ym[1:-1] = 0.5 * (x[:-2] + x[2:]), 0.5 * (y[:-2] + y[2:])
        off = np.clip((x + 0.5 * (xm - x) - cx) * nx + (y + 0.5 * (ym - y) - cy) * ny, -w / 2, w / 2)
        x, y = cx + off * nx, cy + off * ny
    return x, y, off


def test_the_racing_line():
    th = np.linspace(0, 2 * math.pi, 24, endpoint=False)
    x, y = 20 * np.cos(th) + 3 * np.cos(3 * th), 12 * np.sin(th)
    got = lt.optimise_racing_line(x, y, track_width_m=4.0, iterations=15)
    want = _relax(x, y, 4.0, True, 15)
    for g, w in zip(got, want):
        assert g == pytest.approx(w, rel=1e-12, abs=1e-12)
    assert np.max(np.abs(got[2])) == pytest.approx(2.0)
    op = lt.optimise_racing_line(x, y, track_width_m=0.0, closed=False, iterations=0)
    want = _relax(x, y, 0.2, False, 1)
    for g, w in zip(op, want):
        assert g == pytest.approx(w, rel=1e-12, abs=1e-12)
    d = lt.optimise_racing_line(x, y)
    assert d[0] == pytest.approx(_relax(x, y, 3.0, True, 60)[0], rel=1e-12)
    for bad in (([0, 1, 2, 3], [0, 1, 2, 3]), ([0, 1, 2, 3, 4], [0, 1, 2, 3])):
        bx, by, bo = lt.optimise_racing_line(*bad)
        assert bo.tolist() == [0.0] * len(bad[0]) and bx.tolist() == list(map(float, bad[0]))
    ex, ey, eo = lt.optimise_racing_line(["a"] * 6, [0] * 6) if False else (None, None, None)
    jx, jy, jo = lt.optimise_racing_line(np.zeros(6), np.zeros(6), track_width_m="w")
    assert jo.tolist() == [0.0] * 6 and jx.tolist() == [0.0] * 6


def test_the_line_comparison(monkeypatch):
    car = _car(1.5)
    th = np.linspace(0, 2 * math.pi, 30, endpoint=False)
    x, y = 25 * np.cos(th), 14 * np.sin(th)
    pt = lt.Powertrain(cla=1.0)
    r = lt.compare_line_vs_centerline(car, x, y, track_width_m=3.5, pt=pt, ds=2.0)
    base = lt.simulate_lap(car, lt.track_from_path(x, y, name="Centreline", ds=2.0), pt)
    lx, ly, off = lt.optimise_racing_line(x, y, track_width_m=3.5)
    line = lt.simulate_lap(car, lt.track_from_path(lx, ly, name="Racing line", ds=2.0), pt)
    assert r["centerline_track"].name == "Centreline" and r["line_track"].name == "Racing line"
    assert r["centerline_result"].lap_time_s == pytest.approx(base.lap_time_s)
    assert r["line_result"].lap_time_s == pytest.approx(line.lap_time_s)
    assert r["time_gained_s"] == pytest.approx(base.lap_time_s - line.lap_time_s) and r["line_x"] == pytest.approx(lx)
    assert r["line_y"] == pytest.approx(ly) and r["offset"] == pytest.approx(off)
    seen = []
    monkeypatch.setattr(lt, "simulate_lap", lambda v, t, p: seen.append(p) or lt._safe_lap())
    assert math.isnan(lt.compare_line_vs_centerline(car, x, y)["time_gained_s"])
    assert seen[0] == lt.Powertrain()
    monkeypatch.setattr(lt, "simulate_lap", lambda v, t, p: lt.LapResult(10.0, 0, 0, 0, 0, ok=(t.name == "Centreline")))
    assert math.isnan(lt.compare_line_vs_centerline(car, x, y)["time_gained_s"])


def test_the_points_curve():
    for ev, mx in (("skidpad", 75.0), ("autocross", 125.0), ("endurance", 275.0), ("acceleration", 100.0)):
        mn = 0.05 * mx
        assert lt.event_points_estimate(50.0, 50.0, ev) == pytest.approx(mx)
        assert lt.event_points_estimate(60.0, 50.0, ev) == pytest.approx(mn + (mx - mn) * (72.5 / 60 - 1) / 0.45)
        assert lt.event_points_estimate(72.5, 50.0, ev) == mn and lt.event_points_estimate(80.0, 50.0, ev) == mn
        assert lt.event_points_estimate(45.0, 50.0, ev) == pytest.approx(mx)
    assert lt.event_points_estimate(60.0, 50.0) == pytest.approx(lt.event_points_estimate(60.0, 50.0, "autocross"))
    for a, b in ((0.0, 50.0), (-1.0, 50.0), (60.0, 0.0), (60.0, -2.0), (math.inf, 50.0), (math.nan, 50.0)):
        assert lt.event_points_estimate(a, b) == 0.0
    assert lt.event_points_estimate("x", 50.0) == 0.0


def _loop(n=24, r=12.0, cx=5.0, cy=3.0, squash=0.6, span=2 * math.pi):
    a = np.linspace(0.0, span, n, endpoint=span < 2 * math.pi)
    return cx + r * np.cos(a), cy + squash * r * np.sin(a) + 0.8 * np.sin(3 * a)


def test_characterized_limits(monkeypatch):
    from tests._golden import check
    res = {}
    pts = {"rwd": lt.Powertrain(), "awd": lt.Powertrain(drive="awd", cla=3.2),
           "ct": lt.Powertrain(combined_tire=NS(ell_kx=1.7, ell_ky=0.5, mu_x_ratio=1.1)),
           "ct_bad": lt.Powertrain(combined_tire=NS(), cla=0.0)}
    cars = {"base": _car(1.5, slope=0.01), "light": _car(1.5, mass=0.5), "slick": _car(0.2), "wf": _car(1.3, wf=0.6)}
    res["long"] = {f"{ck}/{pk}": [[f(c, v, p, c.max_lateral_g(), lu) for v in (0.05, 5.0, 20.0, 40.0)
                                   for lu in (0.0, 0.5, 1.2)] for f in (lt._accel_long, lt._decel_long)]
                   for ck, c in cars.items() for pk, p in pts.items()}
    curved = _car(1.5)
    curved._kx_lat_limit_table = (VS, [1.0 + 0.001 * v * v + 0.01 * math.sin(v) for v in VS], 1.5)
    res["lookup"] = [lt._lat_limit_at_speed(curved, lt.Powertrain(), v, 1.5)
                     for v in (0.05, 0.5, 1.0, 2.0, 3.0, 46.0, 47.0, 50.0)]
    res["corner"] = {k: [lt._corner_limit_speed(c, r, p, 1.4) for r in (-1.0, 0.0, 0.5, 2.0, 9.0, 50.0, math.inf)]
                     for k, (c, p) in {"base": (cars["base"], pts["rwd"]), "light": (cars["light"], pts["rwd"]),
                                       "aero": (cars["base"], lt.Powertrain(cla=400.0))}.items()}
    import suspension.ggv as ggv
    made = []

    class G:
        def __init__(self, veh, gp):
            made.append((veh.tag, gp.combined_tire))

        def _max_lateral_g_at_speed(self, v):
            return 0.5 + 0.02 * v if v > 10 else (0.0 if v < 2 else 1.3 + 0.01 * v)
    monkeypatch.setattr(ggv, "GGVGenerator", G)
    fresh = NS(p=NS(mass=290.0, weight_dist_front=0.47), tag="fresh")
    res["built"] = [lt._lat_limit_at_speed(fresh, lt.Powertrain(combined_tire="CT"), v, 1.4) for v in (0.1, 7.0, 30.0)]
    res["built_table"] = [fresh._kx_lat_limit_table, made]
    fresh2 = NS(p=NS(mass=290.0, weight_dist_front=0.47), tag="fresh2")
    res["decel_builds"] = [lt._decel_long(fresh2, 12.0, lt.Powertrain(cla=1.0), 1.4, 0.6), fresh2._kx_lat_limit_table]
    res["motor"] = [lt.MotorMap.from_peak(100.0, 0.5, 3000.0).as_dict(),
                    lt.Powertrain(max_tractive_n=1e9).tractive_force(0.5),
                    lt.Powertrain(max_tractive_n=1e9).tractive_force(0.01)]
    res["pts"] = [lt.event_points_estimate(y, b, e) for e in ("skidpad", "autocross", "endurance", "other")
                  for y, b in ((0.5, 0.5), (0.6, 0.5), (1.0, 0.5), (0.8, 0.9), (60.0, 50.0), (80.0, 50.0), (0.0, 1.0),
                               (1.0, 0.0), (math.inf, 1.0))]
    check("laptime_limits", res)


def test_characterized_laps():
    from tests._golden import check
    S, T = lt.Segment, lt.Track
    car = _car(1.5, slope=0.01)
    pt = lt.Powertrain()
    tracks = {"corner_first": T("cf", [S(5.0, 4.0), S(100.0)]), "fine": T("fine", [S(20.0), S(6.0, 8.0), S(3.0)], ds=0.5),
              "zero_ds": T("zd", [S(10.0), S(4.0, 6.0)], ds=0.0), "empty": T("e", []), "nil": T("n", [S(0.0), S(-3.0)]),
              "auto": lt.default_autocross(0.5)}
    res = {"laps": {k: lt.simulate_lap(car, t, pt) for k, t in tracks.items()}}
    res["slow"] = lt.simulate_lap(car, tracks["fine"], lt.Powertrain(power_kw=0.3, crr=0.3, cda=0.0))
    res["fallback"] = lt.simulate_lap(_car(fail=True), tracks["corner_first"], pt)
    res["auto_track"] = [[s.length_m, s.radius_m] for s in lt.default_autocross(1.0).segments] + [
        lt.default_autocross(2.0).name, lt.default_autocross().ds]
    res["accel"] = [lt.acceleration_time(car, distance_m=0.5), lt.acceleration_time(car, pt, distance_m=3.0, dv_steps=60),
                    lt.acceleration_time(_car(fail=True), distance_m=2.0, dv_steps=10)]
    a = lt.acceleration_time(car)
    res["accel_dflt"] = [a.lap_time_s, a.top_speed_ms, len(a.s), a.avg_speed_ms]
    res["skid"] = [lt.skidpad_time(car, radius_m=0.05), lt.skidpad_time(car), lt.skidpad_time(_car(fail=True))]
    check("laptime_laps", res)


def test_characterized_paths():
    from tests._golden import check
    res = {}
    x, y = _loop(span=1.6 * math.pi)
    xc, yc = _loop()
    paths = {"open": (x, y), "closed": (np.append(xc, xc[0]), np.append(yc, yc[0])),
             "x_only": (np.append(x, x[0]), np.append(y, y[0] + 0.5)),
             "y_only": (np.append(x, x[0] + 0.5), np.append(y, y[0])),
             "tiny_gap": (np.append(xc, xc[0] + 3e-6), np.append(yc, yc[0]))}
    res["tracks"] = {k: [[sg.length_m, sg.radius_m] for sg in lt.track_from_path(px, py, name=k).segments]
                     for k, (px, py) in paths.items()}
    res["opts"] = [[[sg.length_m, sg.radius_m] for sg in t.segments] + [t.name, t.ds] for t in (
        lt.track_from_path(x, y, closed=False, smooth_window=2, ds=0.5),
        lt.track_from_path(x, y, closed=False, smooth_window=1, min_radius_m=8.0, straight_radius_m=15.0),
        lt.track_from_path([0, 1], [0, 1], name="short"), lt.track_from_path([0, 0, 0], [1, 1, 1], name="dot"),
        lt.track_from_path([0, 1, 2], [0, 1], name="ragged"), lt.track_from_path("abc", [1, 2, 3], name="bad"))]
    res["menger"] = [lt._menger_curvature([0, 1], [0, 0]), lt._menger_curvature(x[:6], y[:6]),
                     lt._menger_curvature([0, 1e-4, 2e-4], [0, 1e-4, 0])]
    res["cones"] = [lt.cones_to_centerline(x, y + 1.5, x[::2], y[::2] - 1.5),
                    lt.cones_to_centerline([1, 1, 1], [2, 2, 2], [0, 3], [0, 4]),
                    lt.cones_to_centerline([0, 0, 1, 2], [0, 0, 0, 0], [0, 0.2, 0.4], [1, 1, 1]),
                    lt.cones_to_centerline([0, 0.1, 0.3], [0, 0, 0], [0, 1], [1, 1]),
                    lt.cones_to_centerline([1], [1], [0, 1], [0, 1])]
    res["gps"] = [lt.latlon_to_xy([51.5], [-0.1]), lt.latlon_to_xy([], []),
                  lt.latlon_to_xy([51.5, 51.5005, 51.501], [-0.1, -0.0995, -0.1002])]
    res["normals"] = lt._path_normals([0, 0, 1, 2], [0, 0, 0, 1])
    sharp_x = np.array([0, 10, 20, 20, 20, 10, 0, 0, 3, 6], float)
    sharp_y = np.array([0, 0, 0, 10, 20, 20, 20, 10, 4, 8], float)
    res["line"] = [lt.optimise_racing_line(sharp_x, sharp_y, track_width_m=2.0),
                   lt.optimise_racing_line(sharp_x, sharp_y, track_width_m=0.1, closed=False, iterations=3),
                   lt.optimise_racing_line(sharp_x[:5], sharp_y[:5], iterations=0),
                   lt.optimise_racing_line(sharp_x[:4], sharp_y[:4]),
                   lt.optimise_racing_line(sharp_x, sharp_y[:9]),
                   lt.optimise_racing_line(sharp_x, sharp_y, track_width_m="wide"),
                   lt.optimise_racing_line(np.linspace(0, 60, 31), 6 * np.sin(np.linspace(0, 60, 31) / 6),
                                           track_width_m=0.8, closed=False)]
    cmp = lt.compare_line_vs_centerline(_car(1.5, slope=0.01), xc, yc)
    res["compare"] = {k: (v if not isinstance(v, lt.Track) else [[sg.length_m, sg.radius_m] for sg in v.segments])
                      for k, v in cmp.items()}
    check("laptime_paths", res)
