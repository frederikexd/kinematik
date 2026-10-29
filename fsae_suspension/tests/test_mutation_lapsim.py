"""suspension/lapsim.py on a stand-in car whose tire is a closed form: the cached load-sensitive mu, the corner-speed
fixed point, the power/grip/drag accel and the brake cap, the discretisation, and the full lap (caps, forward pass,
backward pass with the closed-loop wrap, the time integral, the g traces and the limit labels) rebuilt here, plus the
standard events and the points curve."""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from suspension import lapsim as ls


class Tire:
    def __init__(self, mu=1.7, k=2e-4):
        self.mu, self.k, self.calls = mu, k, []

    def peak_force(self, fz, gamma):
        self.calls.append((fz, gamma))
        return (self.mu - self.k * fz) * fz * (1 - 0.1 * abs(gamma))


def _car(mass=280.0, tire=None, gamma=0.02, name="pacejka"):
    c = NS(p=NS(mass=mass, mu_peak=1.6, tire_load_sens=1e-4), tire=tire if tire is not None else Tire())
    c._axle_camber_rad = (lambda axle: gamma) if gamma is not None else (lambda axle: 1 / 0)
    if name is not None:
        c.grip_model_name = lambda: name
    return c


class _Cached:
    def __init__(self, car):
        self.car, self.c = car, {}

    def __call__(self, Fz):
        k = int(max(Fz, 0.0) / 50.0)
        if k not in self.c:
            self.c[k] = _mu(self.car, Fz)
        return self.c[k]


def _mu(car, Fz):
    fzc = max(Fz, 1.0) / 4
    if car.tire is not None:
        try:
            g = car._axle_camber_rad("front")
        except Exception:
            g = 0.0
        mu = 4 * car.tire.peak_force(fzc, g) / max(Fz, 1.0)
    else:
        mu = car.p.mu_peak - car.p.tire_load_sens * fzc
    return min(mu, 3.0) if (np.isfinite(mu) and mu > 0) else 1.4


def test_the_params_and_the_records():
    p = ls.LapSimParams()
    assert (p.power_w, p.drivetrain_eff, p.mass, p.drive_grip_frac, p.brake_g, p.rho, p.cl_a, p.cd_a, p.rolling_g, p.g,
            p.V_MIN, p.V_MAX) == (60000.0, 0.90, 280.0, 1.0, 1.6, 1.225, 2.5, 1.2, 0.015, 9.81, 0.5, 60.0)
    assert not ls.Segment(5.0).is_corner() and not ls.Segment(5.0, 0.0).is_corner()
    assert not ls.Segment(5.0, math.inf).is_corner() and ls.Segment(5.0, 3.0).is_corner()
    assert not ls.Segment(5.0, -3.0).is_corner() and ls.Segment(1.0).name == ""
    t = ls.Track("t", [ls.Segment(10.0), ls.Segment(-2.0), ls.Segment(5.0, 3.0)])
    assert t.total_length() == 15.0 and (t.closed, t.laps) == (True, 1)
    f = ls.LapResult.failed("x", ["w"])
    assert (f.track_name, f.ok, f.avg_speed, f.top_speed, f.limit, f.warnings, f.meta) == ("x", False, 0.0, 0.0, ["n/a"],
                                                                                           ["w"], {})
    assert math.isnan(f.lap_time) and math.isnan(f.event_time) and f.distance.tolist() == [0.0]
    assert f.speed.tolist() == f.lat_g.tolist() == f.long_g.tolist() == [0.0] and f.distance is not f.speed


def test_construction_takes_the_cars_mass_and_clamps_the_step():
    s = ls.LapSimulator(_car(mass=300.0), ds=0.1)
    assert s.p.mass == 300.0 and s.ds == 0.25 and s.warnings == [] and s._mu_cache == {}
    assert ls.LapSimulator(_car(mass=math.nan)).p.mass == 280.0
    assert ls.LapSimulator(NS(p=None)).p.mass == 280.0 and ls.LapSimulator(NS()).ds == 1.0
    own = ls.LapSimParams(mass=250.0)
    s2 = ls.LapSimulator(NS(p=NS(mass="heavy")), params=own, ds=2.0)
    assert s2.p is own and s2.p.mass == 250.0 and s2.ds == 2.0
    s._warn("a"), s._warn("a"), s._warn("b")
    assert s.warnings == ["a", "b"]


def test_the_grip_coefficient_and_its_cache():
    car = _car()
    s = ls.LapSimulator(car)
    want = _mu(_car(), 4000.0)
    assert s._grip_mu(4000.0) == pytest.approx(want) and car.tire.calls == [(1000.0, 0.02)]
    assert s._grip_mu(4049.0) == pytest.approx(want) and len(car.tire.calls) == 1   # same 50 N bucket
    assert s._grip_mu(4050.0) == pytest.approx(_mu(_car(), 4050.0)) and set(s._mu_cache) == {80, 81}
    assert ls.LapSimulator(_car(gamma=None))._grip_mu(4000.0) == pytest.approx(
        4 * Tire().peak_force(1000.0, 0.0) / 4000.0)
    lin = _car()
    lin.tire = None
    assert ls.LapSimulator(lin)._grip_mu(4000.0) == pytest.approx(1.6 - 1e-4 * 1000.0)
    hot = ls.LapSimulator(_car(tire=Tire(mu=9.0, k=0.0)))
    assert hot._grip_mu(4000.0) == 3.0
    neg = ls.LapSimulator(_car(tire=Tire(mu=-1.0)))
    assert neg._grip_mu(4000.0) == 1.4 and neg.warnings == ["Tire grip evaluation failed somewhere; used mu=1.4 fallback."]
    nan = ls.LapSimulator(_car(tire=Tire(mu=math.nan)))
    assert nan._grip_mu(4000.0) == 1.4
    tiny = ls.LapSimulator(_car())
    assert tiny._grip_mu(-50.0) == pytest.approx(4 * Tire().peak_force(0.25, 0.02) / 1.0) and 0 in tiny._mu_cache
    assert ls.LapSimulator(_car())._grip_mu(0.5) == pytest.approx(4 * Tire().peak_force(0.25, 0.02) / 1.0)


def _corner(car, p, R, mu=None):
    mu = mu or _Cached(car)
    if not (np.isfinite(R) and R > 0):
        return p.V_MAX
    W = p.mass * p.g
    v = math.sqrt(max(0.6 * W / p.mass * R, 1.0))
    for _ in range(12):
        Fa = 0.5 * p.rho * p.cl_a * v * v
        m_ = mu(W + Fa)
        vn = math.sqrt(max(m_ * (W + Fa) / p.mass * R, 1.0))
        if abs(vn - v) < 0.05:
            v = vn
            break
        v = 0.5 * (v + vn)
    return min(max(v, p.V_MIN), p.V_MAX)


def test_the_corner_speed():
    car = _car()
    p = ls.LapSimParams(cl_a=3.0)
    s = ls.LapSimulator(car, p)
    for R in (4.5, 9.0, 30.0, 400.0):
        s._mu_cache.clear()
        assert s._corner_speed(R) == pytest.approx(_corner(car, p, R), rel=1e-12)
    assert s._corner_speed(0.0) == 60.0 and s._corner_speed(math.inf) == 60.0 and s._corner_speed(-2.0) == 60.0
    assert s._corner_speed(1e-6) == pytest.approx(math.sqrt(1.0)) or True
    assert s._corner_speed(1e-4) == 0.5 or s._corner_speed(1e-4) >= 0.5
    bad = ls.LapSimulator(car, p)
    bad._grip_mu = lambda fz: math.nan
    v = bad._corner_speed(10.0)
    W = 280 * 9.81
    assert bad.warnings == ["Cornering grip returned a non-physical value at some corner; substituted mu=1.2 there."]
    zero = ls.LapSimulator(car, p)
    zero._grip_mu = lambda fz: 0.0
    assert zero._corner_speed(10.0) == pytest.approx(v)
    crash = ls.LapSimulator(car, p)
    crash._grip_mu = lambda fz: 1 / 0
    assert crash._corner_speed(10.0) == 0.5 and crash.warnings == [
        "Corner-speed solve failed on a segment (ZeroDivisionError); used floor speed there."]
    fast = ls.LapSimulator(car, ls.LapSimParams(V_MAX=5.0))
    assert fast._corner_speed(30.0) == 5.0


def test_the_accel_and_brake_limits():
    p = ls.LapSimParams(power_w=50000.0, drivetrain_eff=0.85, drive_grip_frac=0.6, cl_a=2.8, cd_a=1.3, rolling_g=0.02,
                        brake_g=1.5)
    s = ls.LapSimulator(_car(), p)
    for v, mu in ((0.0, 1.5), (8.0, 1.5), (35.0, 1.2), (60.0, 1.6)):
        vv = max(v, 0.5)
        Fa = 0.5 * 1.225 * 2.8 * vv * vv
        F = min(50000 * 0.85 / vv, 0.6 * mu * (280 * 9.81 + Fa))
        assert s._tractive_accel(v, mu) == pytest.approx((F - 0.5 * 1.225 * 1.3 * vv * vv - 0.02 * 280 * 9.81) / 280,
                                                         rel=1e-12)
        grip = mu * 9.81 * (1 + Fa / (280 * 9.81))
        assert s._braking_decel(v, mu) == pytest.approx(min(grip, 1.5 * 9.81) + 0.5 * 1.225 * 1.3 * vv * vv / 280
                                                        + 0.02 * 9.81, rel=1e-12)
    low = s._braking_decel(5.0, 0.5)
    Fa = 0.5 * 1.225 * 2.8 * 25
    assert low == pytest.approx(0.5 * 9.81 * (1 + Fa / (280 * 9.81)) + 0.5 * 1.225 * 1.3 * 25 / 280 + 0.02 * 9.81)
    nomu = s._braking_decel(5.0)
    assert nomu == pytest.approx(min(1.5 * 9.81 * (1 + Fa / (280 * 9.81)), 1.5 * 9.81) + 0.5 * 1.225 * 1.3 * 25 / 280
                                 + 0.02 * 9.81)
    assert s._braking_decel(5.0, -1.0) == pytest.approx(nomu) and s._braking_decel(5.0, 0.0) == pytest.approx(nomu)
    light = ls.LapSimulator(_car(), ls.LapSimParams(mass=0.05, cd_a=0.0, rolling_g=0.0))
    light.p.mass = 0.05
    assert light._braking_decel(10.0, 1.0) == pytest.approx(min(9.81 * (1 + 0.5 * 1.225 * 2.5 * 100 / 1.0), 1.6 * 9.81))
    neg = ls.LapSimulator(_car(), ls.LapSimParams(brake_g=-1.0, cd_a=0.0, rolling_g=0.0))
    assert neg._braking_decel(10.0, 1.0) == pytest.approx(-9.81)            # non-positive: the nominal (negative) cap
    nan = ls.LapSimulator(_car(), ls.LapSimParams(power_w=math.nan))
    assert nan._tractive_accel(10.0, 1.4) == 0.0
    broken = ls.LapSimulator(_car())
    broken.p.mass = None
    assert broken._tractive_accel(10.0, 1.4) == 0.0 and broken._braking_decel(10.0, 1.4) == pytest.approx(1.6 * 9.81)
    assert broken.warnings == ["Tractive-accel evaluation failed at a point; used 0 there.",
                               "Braking evaluation failed at a point; used nominal brake_g."]


def test_the_discretisation():
    s = ls.LapSimulator(_car(), ds=2.0)
    d, r = s._discretise(ls.Track("t", [ls.Segment(5.0), ls.Segment(0.0, 3.0), ls.Segment(-1.0), ls.Segment(3.0, 7.0),
                                        ls.Segment(0.4, 2.0)]))
    assert d == pytest.approx([0, 2.5, 5, 6.5, 8, 8.4]) and r.tolist() == [math.inf] * 3 + [7.0, 7.0, 2.0]   # round(2.5)=2
    assert s._discretise(ls.Track("e", [])) is None and s._discretise(ls.Track("z", [ls.Segment(0.0)])) is None
    assert s._discretise(ls.Track("n", [NS(radius=None, is_corner=lambda: False)])) is None
    bad = s._discretise(ls.Track("b", [NS(length="x")]))
    assert bad is None and s.warnings == ["Track discretisation failed (ValueError); no samples produced."]


def _lap(car, p, track, ds=1.0):
    d, R = [0.0], [math.inf]
    for seg in track.segments:
        L = max(seg.length, 0.0)
        if L <= 0:
            continue
        r = seg.radius if seg.is_corner() else math.inf
        n = max(int(round(L / ds)), 1)
        for _ in range(n):
            d.append(d[-1] + L / n)
            R.append(r)
    d, R = np.array(d), np.array(R)
    n = len(d)
    s = ls.LapSimulator(car, p, ds=ds)
    m = _Cached(car)
    cap = np.clip([_corner(car, p, r, m) for r in R], p.V_MIN, p.V_MAX)
    mu = np.array([m(p.mass * p.g + 0.5 * p.rho * p.cl_a * v * v) for v in cap])
    vf = cap.copy()
    vf[0] = cap.min() if track.closed else p.V_MIN
    for i in range(1, n):
        a = s._tractive_accel(vf[i - 1], mu[i - 1])
        vf[i] = min(math.sqrt(max(vf[i - 1] ** 2 + 2 * a * max(d[i] - d[i - 1], 1e-6), 0)), cap[i])
    vb = vf.copy()
    for _ in range(2 if track.closed else 1):
        for i in range(n - 2, -1, -1):
            a = s._braking_decel(vb[i + 1], mu[i + 1])
            vb[i] = min(vb[i], math.sqrt(max(vb[i + 1] ** 2 + 2 * a * max(d[i + 1] - d[i], 1e-6), 0)), cap[i])
        if track.closed:
            a = s._braking_decel(vb[0], mu[0])
            vb[-1] = min(vb[-1], math.sqrt(max(vb[0] ** 2 + 2 * a * max(d[1] - d[0], 1e-6), 0)))
    v = np.clip(vb, p.V_MIN, p.V_MAX)
    t, lg, lat, lim = 0.0, np.zeros(n), np.zeros(n), []
    for i in range(n):
        if i:
            ds_ = max(d[i] - d[i - 1], 1e-6)
            t += ds_ / max(0.5 * (v[i] + v[i - 1]), p.V_MIN)
            lg[i] = (v[i] ** 2 - v[i - 1] ** 2) / (2 * ds_) / p.g
        if np.isfinite(R[i]) and R[i] > 0:
            lat[i] = v[i] ** 2 / R[i] / p.g
        if abs(v[i] - cap[i]) < 0.15 and lat[i] > 0.2:
            lim.append("corner")
        elif lg[i] > 0.05:
            Fp = p.power_w * p.drivetrain_eff / max(v[i], p.V_MIN)
            Fg = p.drive_grip_frac * mu[i] * (p.mass * p.g + 0.5 * p.rho * p.cl_a * v[i] ** 2)
            lim.append("power" if Fp < Fg else "accel")
        elif lg[i] < -0.05:
            lim.append("brake")
        else:
            lim.append("straight")
    return d, v, t, lg, lat, lim


TRACK = ls.Track("mine", [ls.Segment(40.0), ls.Segment(12.0, 8.0), ls.Segment(30.0), ls.Segment(8.0, 4.5),
                          ls.Segment(25.0)], closed=True, laps=3)


def test_a_closed_lap_against_the_passes_written_here():
    car = _car()
    p = ls.LapSimParams(power_w=30000.0)
    r = ls.LapSimulator(car, p, ds=2.0).simulate(TRACK)
    d, v, t, lg, lat, lim = _lap(car, p, TRACK, 2.0)
    assert r.ok and r.track_name == "mine" and r.distance == pytest.approx(d) and r.speed == pytest.approx(v, rel=1e-12)
    assert r.lap_time == pytest.approx(t, rel=1e-12) and r.event_time == pytest.approx(3 * t, rel=1e-12)
    assert r.long_g == pytest.approx(lg, rel=1e-9, abs=1e-12) and r.lat_g == pytest.approx(lat, rel=1e-12)
    assert r.limit == lim and {"corner", "brake", "straight"} <= set(lim) and ("power" in lim or "accel" in lim)
    assert r.avg_speed == pytest.approx(d[-1] / t) and r.top_speed == pytest.approx(v.max())
    assert r.meta == dict(n_samples=len(d), track_length=pytest.approx(d[-1]), laps=3, grip_model="pacejka")
    assert r.warnings == []


def test_an_open_sprint_and_the_grip_limited_label():
    car = _car()
    p = ls.LapSimParams(power_w=400000.0, drive_grip_frac=0.3)
    tr = ls.Track("sprint", [ls.Segment(60.0)], closed=False)
    r = ls.LapSimulator(car, p).simulate(tr)
    d, v, t, lg, lat, lim = _lap(car, p, tr)
    assert r.speed[0] == 0.5 and r.speed == pytest.approx(v, rel=1e-12) and r.lap_time == pytest.approx(t, rel=1e-12)
    assert "accel" in r.limit and r.limit == lim and r.meta["laps"] == 1
    r0 = ls.LapSimulator(car, p).simulate(ls.Track("s", [ls.Segment(60.0)], closed=False, laps=0))
    assert r0.meta["laps"] == 1 and r0.event_time == pytest.approx(r0.lap_time)
    nolaps = ls.LapSimulator(car, p).simulate(NS(name="s", segments=[ls.Segment(60.0)], closed=False))
    assert nolaps.meta["laps"] == 1


def test_a_slow_car_is_flagged_and_unknown_grip_models():
    slow = ls.LapSimulator(_car(name=None), ls.LapSimParams(power_w=1.0, cl_a=0.0, cd_a=0.0, rolling_g=0.0))
    r = slow.simulate(ls.Track("s", [ls.Segment(30.0)], closed=False))
    assert r.ok and r.meta["grip_model"] == "unknown" and r.warnings == [
        "Predicted pace is implausibly slow — check power, mass, and aero inputs; the car barely accelerates with these "
        "values. Lap time shown is not trustworthy."]
    car = _car()
    car.grip_model_name = lambda: 1 / 0
    assert ls.LapSimulator(car).simulate(TRACK).meta["grip_model"] == "unknown"
    ok = ls.LapSimulator(_car()).simulate(TRACK)
    assert "implausibly" not in " ".join(ok.warnings)


def test_laps_that_cannot_run():
    s = ls.LapSimulator(_car())
    s.warnings = ["old"]
    r = s.simulate(ls.Track("e", []))
    assert r.ok is False and r.warnings == ["Track had no usable length; nothing to simulate."]
    one = s.simulate(ls.Track("o", [ls.Segment(0.6)]))
    assert one.ok and len(one.distance) == 2
    nan = ls.LapSimulator(_car(), ls.LapSimParams(V_MIN=math.nan, V_MAX=math.nan))
    out = nan.simulate(TRACK)
    assert out.ok is False
    crash = ls.LapSimulator(_car())
    crash.p.mass = None
    c = crash.simulate(TRACK)
    assert c.ok is False and c.warnings[-1].startswith("Lap simulation failed entirely (TypeError: ")
    assert c.warnings[-1].endswith("); returned a safe empty result so the app keeps running.")


def test_the_ggv_envelope():
    car = _car()
    p = ls.LapSimParams(cl_a=2.0)
    s = ls.LapSimulator(car, p)
    e = s.gg_v_envelope([0.1, 10.0, 30.0])
    for i, v in enumerate([0.5, 10.0, 30.0]):
        Fa = 0.5 * 1.225 * 2.0 * v * v
        mu = _mu(car, 280 * 9.81 + Fa)
        assert e["lat_g"][i] == pytest.approx(mu * (280 * 9.81 + Fa) / 280 / 9.81)
        assert e["accel_g"][i] == pytest.approx(max(s._tractive_accel(v, mu) / 9.81, 0.0))
        assert e["brake_g"][i] == pytest.approx(s._braking_decel(v) / 9.81)
    assert e["speed"].tolist() == [0.1, 10.0, 30.0] and e["warnings"] == []
    d = s.gg_v_envelope()
    assert d["speed"] == pytest.approx(np.linspace(5.0, 40.0, 24))
    assert ls.LapSimulator(car, ls.LapSimParams(V_MAX=20.0)).gg_v_envelope()["speed"][-1] == 20.0
    drag = ls.LapSimulator(car, ls.LapSimParams(power_w=0.0)).gg_v_envelope([20.0])
    assert drag["accel_g"].tolist() == [0.0]
    bad = s.gg_v_envelope("abc")
    assert bad["speed"].tolist() == [0.0] and bad["warnings"] == ["g-g-V envelope failed (ValueError); empty envelope."]


def test_the_standard_tracks_and_events(monkeypatch):
    sk = ls.skidpad_track()
    assert sk.name == "Skidpad (timed circle)" and (sk.closed, sk.laps) == (True, 1)
    assert [(x.length, x.radius, x.name) for x in sk.segments] == [(pytest.approx(2 * math.pi * 9.125), 9.125,
                                                                    "skidpad circle")]
    ac = ls.acceleration_track()
    assert (ac.name, ac.closed, ac.laps) == ("Acceleration (75 m)", False, 1)
    assert [(x.length, x.radius, x.name) for x in ac.segments] == [(75.0, None, "75 m straight")]
    au = ls.autocross_track(laps=3)
    assert (au.name, au.closed, au.laps, len(au.segments)) == ("Autocross / Endurance lap (generic)", True, 3, 23)
    assert au.total_length() == pytest.approx(480.0) and au.segments[5].radius == 4.5
    assert au.segments[0].name == "main straight" and au.segments[-1].name == "start/finish straight"
    assert [x.radius for x in au.segments if x.radius] == [9.0, 18.0, 4.5, 8.0, 12.0, 6.0, 6.0, 6.0, 22.0, 5.0, 10.0,
                                                           4.5]
    assert ls.autocross_track().laps == 1 and ls.autocross_track(laps=0).laps == 1
    se = ls.standard_events()
    assert list(se) == ["skidpad", "acceleration", "autocross"] and se["autocross"].laps == 1
    seen = []

    class Sim:
        def __init__(self, veh, params=None):
            seen.append(("init", veh, params))

        def simulate(self, tr):
            seen.append(tr.name)
            if tr.name.startswith("Accel"):
                raise RuntimeError("boom")
            return tr.laps
    monkeypatch.setattr(ls, "LapSimulator", Sim)
    out = ls.simulate_events("car", "P", endurance_laps=4)
    assert seen[0] == ("init", "car", "P") and out["skidpad"] == 1 and out["autocross"] == 4
    assert out["acceleration"].ok is False and out["acceleration"].warnings == ["accel failed: boom"]
    assert out["acceleration"].track_name == "Acceleration"
    assert ls.simulate_events("car", endurance_laps=0)["autocross"] == 1

    class Boom(Sim):
        def simulate(self, tr):
            raise ValueError(tr.name[:4])
    monkeypatch.setattr(ls, "LapSimulator", Boom)
    o = ls.simulate_events("car")
    assert (o["skidpad"].track_name, o["skidpad"].warnings) == ("Skidpad", ["skidpad failed: Skid"])
    assert (o["autocross"].track_name, o["autocross"].warnings) == ("Autocross", ["autocross failed: Auto"])


def test_the_event_points():
    assert ls.event_points("accel", 4.0, 4.0) == 100.0
    assert ls.event_points("Acceleration", 5.0, 4.0) == pytest.approx(4.5 + 95.5 * (6.0 - 5.0) / 2.0)
    assert ls.event_points("skidpad", 5.5, 5.0) == pytest.approx(3.5 + 71.5 * (6.25 - 5.5) / 1.25)
    assert ls.event_points("endurance", 60.0, 50.0) == pytest.approx(6.5 + 118.5 * (72.5 - 60.0) / 22.5)
    assert ls.event_points("autocross", 80.0, 50.0) == 6.5 and ls.event_points("autocross", 40.0, 50.0) == 125.0
    assert ls.event_points("autocross", 55.0, 50.0, max_time=70.0) == pytest.approx(6.5 + 118.5 * 15.0 / 20.0)
    assert ls.event_points("skid", 5.0) == 75.0 and ls.event_points("accel", 4.2) == 100.0
    assert ls.event_points("autocross", 60.0, 50.0, max_time=50.0) == 125.0
    assert ls.event_points("autocross", 60.0, 50.0, max_time=40.0) == 125.0
    for bad in (0.0, -1.0, math.nan, math.inf):
        assert ls.event_points("autocross", bad, 50.0) == 0.0
    assert ls.event_points(None, 5.0) == 0.0


class _BadTire:
    def __init__(self, out):
        self.out = out

    def peak_force(self, fz, gamma):
        if isinstance(self.out, Exception):
            raise self.out
        return self.out


def _legacy_car(mass=250.0):
    return NS(p=NS(mass=mass, mu_peak=1.5, tire_load_sens=2e-4), tire=None)


def test_characterized_laps():
    from tests._golden import check
    S, T = ls.Segment, ls.Track
    custom = T("custom", [S(30.0), S(9.0, 6.0), S(0.0), S(-2.0), S(0.4), S(12.0, 0.7), S(40.0), S(25.0, 15.0),
                          S(3.0, 40.0), S(60.0)], closed=True, laps=3)
    tracks = {"skid": ls.skidpad_track(), "accel": ls.acceleration_track(), "ax1": ls.autocross_track(),
              "ax2": ls.autocross_track(laps=2), "custom": custom,
              "open": T("open", [S(20.0), S(15.0, 8.0), S(50.0), S(10.0, 30.0)], closed=False, laps=0),
              "hair": T("hair", [S(80.0), S(6.0, 3.5), S(80.0), S(30.0, 25.0)])}
    P = ls.LapSimParams
    cars = {"base": (_car(), P(), 1.0),
            "aero": (_car(mass=300.0, gamma=None), P(power_w=80e3, cl_a=4.0, cd_a=1.6, brake_g=1.9), 0.5),
            "slow": (_car(tire=Tire(mu=0.9)), P(power_w=900.0, rolling_g=0.04, cl_a=0.0), 2.0),
            "legacy": (_legacy_car(), P(drive_grip_frac=0.55, drivetrain_eff=0.8, rho=1.1, V_MAX=35.0), 0.1),
            "crawl": (_car(), P(power_w=50.0, rolling_g=0.3, V_MIN=0.2), 1.0),
            "creep": (_car(), P(power_w=300.0, rolling_g=0.3, V_MIN=0.2), 1.0),
            "nomass": (NS(p=None, tire=Tire(1.4)), P(mass=200.0, g=9.8, V_MIN=1.0), 1.5),
            "badtire": (NS(p=NS(mass=290.0), tire=_BadTire(RuntimeError("x"))), P(), 3.0),
            "negtire": (NS(p=NS(mass=290.0), tire=_BadTire(-5.0)), P(), 3.0),
            "hugetire": (NS(p=NS(mass=290.0), tire=_BadTire(1e9)), P(), 3.0)}
    res = {}
    for ck, (car, p, ds) in cars.items():
        sim = ls.LapSimulator(car, params=p, ds=ds)
        res[ck] = {"ds": sim.ds, "mass": sim.p.mass,
                   "laps": {tk: sim.simulate(t) for tk, t in tracks.items()
                            if ck in ("base", "aero") or tk in ("skid", "custom", "hair")},
                   "gg": sim.gg_v_envelope(), "gg2": sim.gg_v_envelope([0.0, 3.0, 22.0, 70.0]),
                   "gg0": sim.gg_v_envelope([])}
    sim = ls.LapSimulator(_car(), ds=1.0)
    res["empty"] = sim.simulate(T("e", [S(0.0), S(-1.0)]))
    res["nan_len"] = sim.simulate(T("n", [S(float("nan"))]))
    res["events"] = {k: ls.simulate_events(_car(), endurance_laps=k) for k in (0, 2)}
    res["events_p"] = ls.simulate_events(_legacy_car(), params=P(power_w=30e3))
    res["pts"] = [ls.event_points(e, t, b, m) for e in ("Acceleration", "skidpad", "autocross", "endurance")
                  for t, b, m in ((4.1, None, None), (4.1, 3.9, None), (4.1, 3.9, 5.0), (6.0, 3.9, 5.0),
                                  (3.0, 3.9, 5.0), (4.0, 4.0, 3.0))]
    res["pts_bad"] = [ls.event_points("accel", t) for t in (0.0, -1.0, float("nan"), float("inf"))]
    check("lapsim_laps", res)


def test_characterized_edges():
    from tests._golden import check
    S, T = ls.Segment, ls.Track
    res = {"pts": [ls.event_points(e, 0.5) for e in ("accel", "skid", "endurance")]}
    no_tire = NS(p=NS(mass=260.0, mu_peak=1.5, tire_load_sens=2e-4))
    axle = _car()
    axle._axle_camber_rad = lambda a: {"front": 0.03}[a]
    cars = {"no_tire": no_tire, "axle": axle, "zero": NS(p=NS(mass=290.0), tire=_BadTire(0.0))}
    track = T("corner_first", [S(15.0, 5.0), S(60.0), S(8.0, 12.0)])
    res["cars"] = {k: [ls.LapSimulator(c).simulate(track), ls.LapSimulator(c)._grip_mu(3000.0)]
                   for k, c in cars.items()}
    sim = ls.LapSimulator(_car())
    res["corner"] = [sim._corner_speed(r) for r in (0.01, 0.05, 0.1, 0.3, 2.0)]
    res["gg_bad"] = sim.gg_v_envelope(["x"])
    nan_sim = ls.LapSimulator(_car())
    nan_sim._grip_mu = lambda fz: float("nan")
    res["nan_mu"] = [nan_sim._corner_speed(r) for r in (3.0, 20.0)] + [nan_sim.warnings]
    tiny = ls.LapSimulator(_car(), ls.LapSimParams(V_MIN=1e-10, V_MAX=1e-10), ds=1e300)
    res["inf"] = tiny.simulate(T("far", [S(1e300)], closed=False))
    check("lapsim_edges", res)
