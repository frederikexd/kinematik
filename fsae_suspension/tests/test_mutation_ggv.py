"""suspension/ggv.py on a stand-in car whose load transfer and axle grip are closed forms: the lateral limit is the
root of capacity - demand found here by brentq, the accel and brake limits and the combined-slip interior are
written out from their stated formulas, the superellipse radius is checked at its root, and the sweep, the one-call
builder and the cross-check against a stand-in lap-time model are followed through every branch and message."""
import math
import sys
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.optimize import brentq

from suspension import ggv
from suspension.dynamics import CornerLoads

G = 9.81


class Car:
    """Static corners from the weight split; lateral transfer a*g front and b*g rear per corner; each tire carries
    1.6 Fz - 2e-4 Fz^2."""

    def __init__(self, mass=280.0, wf=0.47, a=300.0, b=320.0, name="fake"):
        self.p = SimpleNamespace(mass=mass, g=G, weight_dist_front=wf, use_param_camber=False, cg_height=300.0,
                                 static_camber_front=-1.0)
        self.a, self.b, self.name, self.asked = a, b, name, []

    def lateral_load_transfer(self, g):
        self.asked.append(g)
        W = self.p.mass * G
        f, r = W * self.p.weight_dist_front / 2, W * (1 - self.p.weight_dist_front) / 2
        return CornerLoads(f - self.a * g, f + self.a * g, r - self.b * g, r + self.b * g), {}

    def axle_grip(self, L):
        c = lambda z: 1.6 * z - 2e-4 * z * z
        return c(L.fl) + c(L.fr), c(L.rl) + c(L.rr)

    def grip_model_name(self):
        if self.name is None:
            raise RuntimeError
        return self.name


def _lat(car, v, gp):
    W = car.p.mass * G
    df = 0.5 * gp.rho * gp.cl_a * v * v
    af, ar = df * gp.aero_balance_front / 2, df * (1 - gp.aero_balance_front) / 2

    def cap(g):
        L, _ = car.lateral_load_transfer(g)
        L = CornerLoads(max(L.fl + af, 0), max(L.fr + af, 0), max(L.rl + ar, 0), max(L.rr + ar, 0))
        f, r = car.axle_grip(L)
        return (f + r) / W - g
    return brentq(cap, 0.05, 4.0, xtol=1e-14)


GP = ggv.GGVParams(cl_a=3.0, aero_balance_front=0.4, rho=1.2)


def test_the_params_and_the_powertrain_mapping():
    d = ggv.GGVParams()
    assert (d.power_w, d.drivetrain_eff, d.max_tractive_n, d.drive_axle, d.brake_bias_front, d.brake_g_cap, d.rho,
            d.cl_a, d.cd_a, d.aero_balance_front, d.rolling_resist, d.combined_tire, d.g, d.V_MIN, d.V_MAX) == (
        60000.0, 0.90, 2600.0, "rear", 0.65, 1.8, 1.225, 2.5, 1.2, 0.45, 0.015, None, 9.81, 1.0, 60.0)
    pt = SimpleNamespace(power_kw=80.0, drivetrain_eff=0.8, max_tractive_n=3000.0, drive="awd", brake_g_cap=2.0,
                         rho=1.1, cla=3.5, cda=1.4, crr=0.02, combined_tire="ct")
    p = ggv.GGVParams.from_powertrain(pt)
    assert (p.power_w, p.drivetrain_eff, p.max_tractive_n, p.drive_axle, p.brake_g_cap, p.rho, p.cl_a, p.cd_a,
            p.rolling_resist, p.combined_tire) == (64000.0, 1.0, 3000.0, "all", 2.0, 1.1, 3.5, 1.4, 0.02, "ct")
    e = ggv.GGVParams.from_powertrain(SimpleNamespace())
    assert (e.power_w, e.max_tractive_n, e.drive_axle, e.brake_g_cap, e.rho, e.cl_a, e.cd_a, e.rolling_resist,
            e.combined_tire) == (pytest.approx(54000.0), 2600.0, "rear", 1.8, 1.225, 2.5, 1.2, 0.015, None)
    assert ggv.GGVParams.from_powertrain(SimpleNamespace(drive="rwd")).drive_axle == "rear"


def test_the_aero_forces_and_the_lateral_limit():
    car = Car()
    gen = ggv.GGVGenerator(car, GP)
    assert gen.p is car.p and gen.warnings == [] and ggv.GGVGenerator(car).g == ggv.GGVParams()
    assert gen._aero_downforce(20.0) == pytest.approx(0.5 * 1.2 * 3.0 * 400) and gen._aero_drag(20.0) == pytest.approx(
        0.5 * 1.2 * 1.2 * 400)
    for v in (1.0, 15.0, 30.0):
        assert gen._max_lateral_g_at_speed(v) == pytest.approx(_lat(car, v, GP), abs=1e-10)
    assert gen._max_lateral_g_at_speed(30.0) > gen._max_lateral_g_at_speed(1.0) and gen.warnings == []
    assert ggv.GGVGenerator(Car(mass=0.0), GP)._max_lateral_g_at_speed(10.0) == 0.0


def test_a_lifted_inner_wheel_is_flagged():
    car = Car(a=900.0, b=900.0)
    gen = ggv.GGVGenerator(car, ggv.GGVParams(cl_a=0.0))
    lo = gen._max_lateral_g_at_speed(10.0)
    assert lo == pytest.approx(_lat(car, 10.0, ggv.GGVParams(cl_a=0.0)), abs=1e-10)
    assert gen.warnings == [
        f"Inner-wheel lift at ~{lo:.2f} g (v=10 m/s): an inside tire has fully unloaded, so the rigid load-transfer "
        "model has saturated. The lateral-grip number here is an upper bound / artifact — reduce CG height, soften the "
        "bar, or treat this point as 'lifting a wheel', not as real grip."]


def test_broken_load_transfer_and_grip_are_clamped():
    class NoLLT(Car):
        def lateral_load_transfer(self, g):
            raise RuntimeError
    gen = ggv.GGVGenerator(NoLLT(), GP)
    assert gen._max_lateral_g_at_speed(10.0) == pytest.approx(0.05, abs=1e-9)
    assert gen.warnings == ["lateral_load_transfer failed at a point; clamped."]

    class NoGrip(Car):
        def axle_grip(self, L):
            raise RuntimeError
    gen = ggv.GGVGenerator(NoGrip(), GP)
    assert gen._max_lateral_g_at_speed(10.0) == pytest.approx(0.05, abs=1e-9)
    assert gen.warnings == ["axle_grip failed at a point; clamped."]

    class Junk(Car):
        def lateral_load_transfer(self, g):
            return CornerLoads(None, 1.0, 1.0, 1.0), {}
    gen = ggv.GGVGenerator(Junk(), GP)
    assert gen._max_lateral_g_at_speed(10.0) == 0.0
    assert gen.warnings == ["Lateral-g bisection failed at a speed; used 0 there."]

    class LateFail(Car):
        def __init__(self):
            super().__init__()
            self.n = 0

        def lateral_load_transfer(self, g):
            self.n += 1
            if self.n > 40:
                raise RuntimeError
            return super().lateral_load_transfer(g)
    late = ggv.GGVGenerator(LateFail(), GP)
    assert late._max_lateral_g_at_speed(10.0) == pytest.approx(_lat(Car(), 10.0, GP), abs=1e-10)
    assert late.warnings == []


def test_the_longitudinal_friction_ratio():
    gen = ggv.GGVGenerator(Car(), GP)
    assert gen._mu_x_ratio() == 1.0
    for ct, want in ((SimpleNamespace(mu_x_ratio=1.3), 1.3), (SimpleNamespace(mu_x_ratio=0.01), 0.1),
                     (SimpleNamespace(), 1.0), (SimpleNamespace(mu_x_ratio="x"), 1.0)):
        assert ggv.GGVGenerator(Car(), ggv.GGVParams(combined_tire=ct))._mu_x_ratio() == want


@pytest.mark.parametrize("axle, frac", [("rear", 0.53), ("front", 0.47), ("all", 1.0), ("awd", 1.0)])
def test_the_accel_limit(axle, frac):
    gp = ggv.GGVParams(drive_axle=axle, cl_a=2.0, cd_a=1.1, power_w=50000.0, drivetrain_eff=0.85,
                       max_tractive_n=2400.0, rolling_resist=0.02, combined_tire=SimpleNamespace(mu_x_ratio=1.1))
    gen = ggv.GGVGenerator(Car(), gp)
    for v, mu in ((0.3, 0.9), (12.0, 0.9), (30.0, 1.4), (12.0, 0.3)):
        vv = max(v, 1.0)
        df = 0.5 * 1.225 * 2.0 * vv * vv
        W = 280.0 * G
        ft = mu * 1.1 * (W + df) * frac
        fp = min(50000.0 * 0.85 / vv, 2400.0)
        fd, fr = 0.5 * 1.225 * 1.1 * vv * vv, 0.02 * (W + df)
        a = gen._max_accel_g_at_speed(v, mu)
        assert a == pytest.approx(max(min(fp, ft) - fd - fr, 0.0) / 280.0 / G, rel=1e-12, abs=1e-15)
        assert gen._last_accel_parts == pytest.approx((ft, fp, fd, fr), rel=1e-12)
    assert gen._max_accel_g_at_speed(80.0, 0.5) == 0.0                     # drag beats the motor


def test_the_brake_limit_and_the_combined_interior():
    gp = ggv.GGVParams(cl_a=2.0, cd_a=1.1, brake_g_cap=1.5, combined_tire=SimpleNamespace(mu_x_ratio=1.2))
    gen = ggv.GGVGenerator(Car(), gp)
    for v, mu in ((0.5, 1.4), (20.0, 1.0), (20.0, 1.6)):
        vv = max(v, 1.0)
        df = 0.5 * 1.225 * 2.0 * vv * vv
        W = 280.0 * G
        fg, fc, fd = mu * 1.2 * (W + df), 1.5 * W, 0.5 * 1.225 * 1.1 * vv * vv
        assert gen._max_brake_g_at_speed(v, mu) == pytest.approx((min(fg, fc) + fd) / 280.0 / G, rel=1e-12)
        assert gen._last_brake_parts == pytest.approx((fg, fc, -fd, 0.0), rel=1e-12)
    assert gen._combined_long_g((1000.0, 800.0, 100.0, 50.0), 200.0, 10.0, 0.5) == pytest.approx(0.175)
    assert gen._combined_long_g((1000.0, 800.0, 100.0, 50.0), 200.0, 10.0, 0.9) == pytest.approx(0.325)
    assert gen._combined_long_g((1000.0, 800.0, 100.0, 50.0), 200.0, 10.0, -1.0) == 0.0
    assert gen._combined_long_g((1000.0, 800.0, -100.0, 0.0), 200.0, 10.0, 1.0) == pytest.approx(0.45)


def test_failed_axis_limits_warn():
    gen = ggv.GGVGenerator(Car(), ggv.GGVParams(combined_tire=SimpleNamespace(mu_x_ratio=1.0)))
    gen.p = SimpleNamespace(mass=None, g=G, weight_dist_front=0.47)
    assert gen._max_accel_g_at_speed(10.0, 1.0) == 0.0 and gen._max_brake_g_at_speed(10.0, 1.0) == 0.0
    assert gen.warnings == ["Accel-g evaluation failed at a point; used 0 there.",
                            "Brake-g evaluation failed at a point; used 0 there."]


def test_the_superellipse_radius():
    for a, b, kx, ky in ((0.3, 0.5, 2.0, 2.0), (0.02, 0.01, 1.5, 3.0), (1e-4, 0.0, 2.5, 2.5), (5.0, 7.0, 0.8, 1.2)):
        r = ggv.GGVGenerator._solve_superellipse_radius(a, b, kx, ky)
        assert a * r ** kx + b * r ** ky == pytest.approx(1.0, abs=1e-12)
    assert ggv.GGVGenerator._solve_superellipse_radius(0.0, 0.0, 2.0, 2.0) == 0.0
    assert ggv.GGVGenerator._solve_superellipse_radius(-1.0, 0.0, 2.0, 2.0) == 0.0
    assert ggv.GGVGenerator._solve_superellipse_radius(0.0, 0.25, 2.0, 2.0) == pytest.approx(2.0, abs=1e-12)


def _surface(car, gp, speeds, n_dir, kx=2.0, ky=2.0):
    gen = ggv.GGVGenerator(car, gp)
    mu0 = max(_lat(car, gp.V_MIN, gp), 0.05)
    th = np.linspace(-math.pi, math.pi, n_dir)
    L, A = np.zeros((len(speeds), n_dir)), np.zeros((len(speeds), n_dir))
    lims = []
    for i, v in enumerate(speeds):
        v = min(max(v, gp.V_MIN), gp.V_MAX)
        lat = _lat(car, v, gp)
        acc = gen._max_accel_g_at_speed(v, mu0)
        pa = gen._last_accel_parts
        brk = gen._max_brake_g_at_speed(v, mu0)
        pb = gen._last_brake_parts
        lims.append((lat, acc, brk))
        for j, t in enumerate(th):
            c, s = math.cos(t), math.sin(t)
            lon = acc if c >= 0 else brk
            at = (abs(c) / lon) ** kx if lon > 1e-6 else 0.0
            bt = (abs(s) / lat) ** ky
            r = brentq(lambda r: at * r ** kx + bt * r ** ky - 1, 0, 100, xtol=1e-14) if at + bt > 0 else 0.0
            parts = pa if c >= 0 else pb
            frac = abs(c) * r / lon if lon > 1e-6 else 0.0
            mag = max(min(parts[1], parts[0] * frac) - parts[2] - parts[3], 0.0) / car.p.mass / G
            L[i, j], A[i, j] = (mag if c >= 0 else -mag), abs(r * s)
    return th, L, A, lims


def test_the_surface_on_the_friction_ellipse():
    car = Car()
    gp = ggv.GGVParams(cl_a=2.5, power_w=45000.0)
    r = ggv.GGVGenerator(car, gp).generate(speeds=[0.5, 12.0, 25.0, 70.0], n_dir=9)
    th, L, A, lims = _surface(car, gp, [0.5, 12.0, 25.0, 70.0], 9)
    assert r.theta == pytest.approx(th) and r.speeds.tolist() == [0.5, 12.0, 25.0, 70.0]
    assert r.long_g == pytest.approx(L, rel=1e-9, abs=1e-12) and r.lat_g == pytest.approx(A, rel=1e-9, abs=1e-12)
    assert np.c_[r.max_lat_g, r.max_accel_g, r.max_brake_g] == pytest.approx(np.array(lims), rel=1e-9)
    assert r.grip_model == "fake" and len(r.warnings) == 1 and "(v=60 m/s)" in r.warnings[0]   # 70 m/s: V_MAX
    assert np.array_equal(r.at_speed(13.0)[0], r.long_g[1]) and np.array_equal(r.at_speed(13.0)[1], r.lat_g[1])
    assert np.array_equal(r.at_speed(19.0)[0], r.long_g[2])
    d = ggv.GGVGenerator(car, gp).generate()
    assert d.speeds == pytest.approx(np.linspace(5.0, 38.0, 12)) and d.theta.size == 73
    low = ggv.GGVGenerator(car, ggv.GGVParams(V_MAX=20.0)).generate(n_dir=3)
    assert low.speeds == pytest.approx(np.linspace(5.0, 20.0, 12))


def test_a_superellipse_tire_and_an_unknown_grip_model():
    car = Car(name=None)
    ct = SimpleNamespace(mu_x_ratio=1.15, ell_kx=1.6, ell_ky=2.4)
    gp = ggv.GGVParams(combined_tire=ct)
    r = ggv.GGVGenerator(car, gp).generate(speeds=[15.0], n_dir=11)
    th, L, A, _ = _surface(car, gp, [15.0], 11, 1.6, 2.4)
    assert r.long_g == pytest.approx(L, rel=1e-9, abs=1e-12) and r.lat_g == pytest.approx(A, rel=1e-9, abs=1e-12)
    assert r.grip_model == "unknown"
    floor = ggv.GGVParams(combined_tire=SimpleNamespace(ell_kx=0.1, ell_ky=0.2))
    rf = ggv.GGVGenerator(Car(), floor).generate(speeds=[15.0], n_dir=7)
    th, L, A, _ = _surface(Car(), floor, [15.0], 7, 0.5, 0.5)
    assert rf.lat_g == pytest.approx(A, rel=1e-9, abs=1e-12)
    bad = ggv.GGVParams(combined_tire=SimpleNamespace(ell_kx="x", ell_ky=3.0))
    rb = ggv.GGVGenerator(Car(), bad).generate(speeds=[15.0], n_dir=7)
    assert rb.lat_g == pytest.approx(_surface(Car(), bad, [15.0], 7)[2], rel=1e-9, abs=1e-12)
    only_y = ggv.GGVParams(combined_tire=SimpleNamespace(ell_ky=3.0))
    assert ggv.GGVGenerator(Car(), only_y).generate(speeds=[15.0], n_dir=7).lat_g == pytest.approx(
        _surface(Car(), only_y, [15.0], 7, 2.0, 3.0)[2], rel=1e-9, abs=1e-12)


def test_no_drive_left_gives_a_pure_lateral_edge():
    car = Car()
    gp = ggv.GGVParams(power_w=0.0, cl_a=0.0)
    r = ggv.GGVGenerator(car, gp).generate(speeds=[10.0], n_dir=5)           # theta = -pi, -pi/2, 0, pi/2, pi
    assert r.max_accel_g[0] == 0.0 and r.long_g[0, 2] == 0.0 and r.lat_g[0, 2] == 0.0
    assert r.lat_g[0, 1] == pytest.approx(r.max_lat_g[0]) and r.long_g[0, 0] == pytest.approx(-r.max_brake_g[0])


def test_the_sweep():
    car = Car()
    gp = ggv.GGVParams(cl_a=0.0)
    s = ggv.sweep_parameter(car, gp, "mass", [250.0, 300.0], speed=12.0)
    want = [ggv.GGVGenerator(Car(mass=m), gp).generate(speeds=[12.0], n_dir=37).max_lat_g[0] for m in (250.0, 300.0)]
    assert s == dict(values=[250.0, 300.0], metric=pytest.approx(want), param="mass", metric_name="max_lat_g",
                     speed=12.0, camber_lever_forced=False)
    assert car.p.mass == 280.0
    a = ggv.sweep_parameter(car, gp, "power_w", [30000.0], metric="max_accel_g")
    assert a["metric"] == [pytest.approx(ggv.GGVGenerator(car, ggv.GGVParams(cl_a=0.0, power_w=30000.0)).generate(
        speeds=[15.0], n_dir=37).max_accel_g[0])] and gp.power_w == 60000.0 and a["speed"] == 15.0
    b = ggv.sweep_parameter(car, gp, "brake_g_cap", [0.5], metric="max_brake_g")
    assert b["metric"][0] == pytest.approx(ggv.GGVGenerator(car, ggv.GGVParams(cl_a=0.0, brake_g_cap=0.5)).generate(
        speeds=[15.0], n_dir=37).max_brake_g[0])
    assert ggv.sweep_parameter(car, gp, "mass", [280.0], metric="nope")["metric"] == pytest.approx(
        [ggv.GGVGenerator(car, gp).generate(speeds=[15.0], n_dir=37).max_lat_g[0]])
    with pytest.raises(AttributeError) as e:
        ggv.sweep_parameter(car, gp, "wingspan", [1.0])
    assert str(e.value) == ("'wingspan' is not a field of VehicleParams or GGVParams. Check the name; e.g. "
                            "'cg_height', 'roll_stiffness_front', 'spring_rate_front', 'static_camber_front', 'cl_a', "
                            "'power_w'.")


def test_a_camber_sweep_turns_the_camber_lever_on_and_back_off():
    car = Car()
    seen = []
    orig = car.axle_grip
    car.axle_grip = lambda L: seen.append(car.p.use_param_camber) or orig(L)
    c = ggv.sweep_parameter(car, ggv.GGVParams(), "static_camber_front", [-2.0])
    assert c["camber_lever_forced"] is True and set(seen) == {True} and car.p.use_param_camber is False
    assert car.p.static_camber_front == -1.0
    car.p.use_param_camber = True
    assert ggv.sweep_parameter(car, ggv.GGVParams(), "static_camber_front", [-2.0])["camber_lever_forced"] is False
    gpc = ggv.GGVParams()
    gpc.static_camber_front = 0.0                                          # a GGV-side field of that name: no toggle
    car2 = Car()
    del car2.p.static_camber_front
    assert ggv.sweep_parameter(car2, gpc, "static_camber_front", [1.0])["camber_lever_forced"] is False

    class Locked(SimpleNamespace):
        def __setattr__(self, k, v):
            if k == "use_param_camber" and v is True:
                raise AttributeError
            super().__setattr__(k, v)
    car3 = Car()
    car3.p = Locked(**vars(car3.p))
    assert ggv.sweep_parameter(car3, ggv.GGVParams(), "static_camber_front", [-2.0])["camber_lever_forced"] is False

    class Boom(Car):
        def axle_grip(self, L):
            raise KeyError
    car4 = Boom()
    car4.p.mass = 280.0
    ggv.sweep_parameter(car4, ggv.GGVParams(), "static_camber_front", [-2.5])
    assert car4.p.static_camber_front == -1.0 and car4.p.use_param_camber is False


def test_quick_ggv_is_the_long_way_round(monkeypatch):
    built = {}

    class Veh:
        def __init__(self, vp, tire=None):
            built.update(vp=vp, tire=tire)
            self.p = vp

    class Gen:
        def __init__(self, veh, gp):
            built.update(veh=veh, gp=gp)

        def generate(self, speeds=None):
            return ("res", speeds)
    monkeypatch.setattr(ggv, "VehicleDynamics", Veh)
    monkeypatch.setattr(ggv, "GGVGenerator", Gen)
    out = ggv.quick_ggv(mass=250.0, cg_height=280.0, weight_dist_front=0.45, track_front=1190.0, track_rear=1170.0,
                        wheelbase=1530.0, roll_stiffness_front=300.0, roll_stiffness_rear=250.0,
                        static_camber_front=-2.0, static_camber_rear=-1.0, power_w=50000.0, cl_a=2.0, cd_a=1.0,
                        tire="T", speeds=[10.0])
    vp = built["vp"]
    assert out == ("res", [10.0]) and built["tire"] == "T" and built["veh"].p is vp
    assert (vp.mass, vp.cg_height, vp.weight_dist_front, vp.track_front, vp.track_rear, vp.wheelbase,
            vp.roll_stiffness_front, vp.roll_stiffness_rear, vp.static_camber_front, vp.static_camber_rear,
            vp.use_param_camber) == (250.0, 280.0, 0.45, 1190.0, 1170.0, 1530.0, 300.0, 250.0, -2.0, -1.0, True)
    assert (built["gp"].power_w, built["gp"].cl_a, built["gp"].cd_a) == (50000.0, 2.0, 1.0)
    ggv.quick_ggv()
    vp = built["vp"]
    assert (vp.mass, vp.cg_height, vp.weight_dist_front, vp.track_front, vp.track_rear, vp.wheelbase,
            vp.roll_stiffness_front, vp.roll_stiffness_rear, vp.static_camber_front, vp.static_camber_rear) == (
        280.0, 300.0, 0.47, 1200.0, 1180.0, 1550.0, 350.0, 300.0, -1.5, -1.5)
    assert (built["gp"].power_w, built["gp"].cl_a, built["gp"].cd_a) == (60000.0, 2.5, 1.2)
    from suspension.tiremodel import default_tire
    assert type(built["tire"]).__name__ == type(default_tire()).__name__


class LT:
    """A stand-in lap-time model: the same lateral limit, accel and brake scaled by given factors."""

    def __init__(self, lat, acc=1.0, brk=1.0, fail=(), mu=None):
        self.lat, self.acc, self.brk, self.fail, self.mu = lat, acc, brk, fail, mu

    def _max_lat_g(self, veh):
        if "lat" in self.fail:
            raise RuntimeError
        assert isinstance(veh, Car)
        return self.lat

    def _accel_long(self, veh, v, pt, mu, used):
        if "acc" in self.fail:
            raise RuntimeError
        assert used == 0.0
        g = ggv.GGVGenerator(veh, ggv.GGVParams.from_powertrain(pt))
        return g._max_accel_g_at_speed(v, self.mu or mu) * G * self.acc

    def _decel_long(self, veh, v, pt, mu, used):
        if "brk" in self.fail:
            raise RuntimeError
        assert used == 0.0 and type(used) is float
        g = ggv.GGVGenerator(veh, ggv.GGVParams.from_powertrain(pt))
        return g._max_brake_g_at_speed(v, self.mu or mu) * G * self.brk


PT = SimpleNamespace(power_kw=70.0, drivetrain_eff=0.9, cla=2.0, cda=1.0)


def _validate(monkeypatch, lt, pt=PT, **kw):
    monkeypatch.setitem(sys.modules, "suspension.laptime", lt)
    return ggv.validate_against_laptime(Car(), pt, **kw)


def test_the_cross_check_agrees_and_disagrees(monkeypatch):
    gp = ggv.GGVParams.from_powertrain(PT)
    lat = _lat(Car(), 1.0, gp)
    r = _validate(monkeypatch, LT(lat), speeds=[10.0, 20.0])
    assert r["ok"] is True and r["reason"] == "within tolerance" and r["rel_tol"] == 0.06 and r["speeds"] == [10.0, 20.0]
    assert r["lat_ggv"] == pytest.approx([lat, lat]) and r["lat_lap"] == [lat, lat]
    assert max(r["lat_reldiff"] + r["accel_reldiff"] + r["brake_reldiff"]) < 1e-9 and r["max_reldiff"] < 1e-9
    assert "note" not in r
    d = _validate(monkeypatch, LT(lat, acc=1.1), speeds=[10.0])
    assert d["ok"] is False and d["accel_reldiff"] == [pytest.approx(1 - 1 / 1.1)]
    assert d["reason"] == f"max relative difference {1 - 1 / 1.1:.1%} exceeds 6% at one or more speeds"
    assert _validate(monkeypatch, LT(lat, acc=1.1), speeds=[10.0], rel_tol=0.1)["ok"] is True
    assert len(_validate(monkeypatch, LT(lat))["speeds"]) == 8 and _validate(monkeypatch, LT(lat))["speeds"][0] == 8.0


def test_the_cross_check_flags_a_braking_only_drift(monkeypatch):
    ct = SimpleNamespace(mu_x_ratio=1.2)
    pt = SimpleNamespace(**vars(PT), combined_tire=ct)
    lat = _lat(Car(), 1.0, ggv.GGVParams.from_powertrain(pt))
    r = _validate(monkeypatch, LT(lat, brk=1.2), pt=pt, speeds=[10.0, 15.0])
    assert r["ok"] is False and r["note"].startswith("REGRESSION: lateral and accel agree; the only divergence is "
                                                     "braking.")
    assert r["note"].endswith("fix the longitudinal models, do not widen this tolerance.")
    assert "note" not in _validate(monkeypatch, LT(lat, brk=1.2, acc=1.2), pt=pt, speeds=[10.0])
    assert "note" not in _validate(monkeypatch, LT(lat, brk=1.2), speeds=[10.0])                # mu_x_ratio 1
    assert "note" not in _validate(monkeypatch, LT(lat * 1.2, brk=1.2), pt=pt, speeds=[10.0])  # lateral off too
    flat = SimpleNamespace(**vars(PT), combined_tire=SimpleNamespace())
    assert "note" not in _validate(monkeypatch, LT(lat, brk=1.2), pt=flat, speeds=[10.0])


def test_the_cross_check_survives_a_broken_lap_model(monkeypatch):
    r = _validate(monkeypatch, LT(1.0, fail=("lat", "acc", "brk")), speeds=[10.0])
    assert math.isnan(r["lat_lap"][0]) and math.isnan(r["accel_lap"][0]) and math.isnan(r["brake_lap"][0])
    assert r["ok"] is False and math.isnan(r["max_reldiff"])
    zero = _validate(monkeypatch, LT(0.0), speeds=[10.0])
    assert zero["lat_reldiff"][0] == pytest.approx(zero["lat_ggv"][0] / 1e-6)
    monkeypatch.delitem(sys.modules, "suspension.laptime", raising=False)
    import importlib
    monkeypatch.setattr(importlib, "import_module", lambda n: (_ for _ in ()).throw(ImportError("gone")))
    out = ggv.validate_against_laptime(Car(), PT)
    assert out["ok"] is False and out["reason"] == "could not import laptime (ImportError: gone)" and out["speeds"] == []


def test_the_cross_check_imports_laptime_when_absent(monkeypatch):
    monkeypatch.delitem(sys.modules, "suspension.laptime", raising=False)
    import importlib
    called = []
    monkeypatch.setattr(importlib, "import_module", lambda n: called.append(n) or LT(1.0))
    ggv.validate_against_laptime(Car(), PT, speeds=[10.0])
    assert called == ["suspension.laptime"]


# ------------------------------------------------------------------ added edges
def test_a_feeble_brake_and_a_massless_car():
    car = Car()
    gp = ggv.GGVParams(brake_g_cap=0.01, cl_a=0.0, cd_a=0.0)
    gen = ggv.GGVGenerator(car, gp)
    assert gen._max_brake_g_at_speed(10.0, 1.5) == pytest.approx(0.01 * G / 9.81, rel=1e-12)
    none = ggv.GGVGenerator(Car(mass=0.0), gp)
    assert none._max_lateral_g_at_speed(10.0) == 0.0 and none.warnings == []
    tiny = Car(mass=0.1, a=0.0, b=0.0)
    gt = ggv.GGVGenerator(tiny, ggv.GGVParams(cl_a=0.0))
    assert gt._max_lateral_g_at_speed(10.0) == pytest.approx(_lat(tiny, 10.0, ggv.GGVParams(cl_a=0.0)), rel=1e-9)


def test_negative_transfer_loads_the_left_side():
    car = Car(a=-300.0, b=-320.0)
    gp = ggv.GGVParams(cl_a=0.0)
    gen = ggv.GGVGenerator(car, gp)
    assert gen._max_lateral_g_at_speed(10.0) == pytest.approx(_lat(car, 10.0, gp), rel=1e-9)


class Fixed(Car):
    def __init__(self, loads):
        super().__init__()
        self.loads = loads

    def lateral_load_transfer(self, g):
        return CornerLoads(*self.loads), {}


@pytest.mark.parametrize("k", range(4))
def test_lift_is_seen_at_every_corner(k):
    loads = [800.0] * 4
    loads[k] = 0.5
    gen = ggv.GGVGenerator(Fixed(loads), ggv.GGVParams(cl_a=0.0))
    gen._max_lateral_g_at_speed(10.0)
    assert len(gen.warnings) == 1 and gen.warnings[0].startswith("Inner-wheel lift at ~")


def test_the_longitudinal_ratio_edges():
    def r(ct):
        return ggv.GGVGenerator(Car(), ggv.GGVParams(combined_tire=ct))._mu_x_ratio()
    assert r(None) == 1.0 and r(SimpleNamespace()) == 1.0 and r(SimpleNamespace(mu_x_ratio="x")) == 1.0
    assert r(SimpleNamespace(mu_x_ratio=0.05)) == 0.1 and r(SimpleNamespace(mu_x_ratio=1.3)) == 1.3


class Slippy(Car):
    def axle_grip(self, L):
        c = lambda z: 0.8 * z - 1e-4 * z * z
        return c(L.fl) + c(L.fr), c(L.rl) + c(L.rr)


def test_a_low_grip_car_and_an_x_only_superellipse():
    car = Slippy()
    gp = ggv.GGVParams(cl_a=0.0)
    r = ggv.GGVGenerator(car, gp).generate(speeds=[12.0], n_dir=9)
    th, L, A, lims = _surface(car, gp, [12.0], 9)
    assert lims[0][0] < 1.0
    assert r.long_g == pytest.approx(L, rel=1e-9, abs=1e-12) and r.lat_g == pytest.approx(A, rel=1e-9, abs=1e-12)
    xonly = ggv.GGVParams(combined_tire=SimpleNamespace(ell_kx=3.0))
    rx = ggv.GGVGenerator(Car(), xonly).generate(speeds=[15.0], n_dir=7)
    th, L, A, _ = _surface(Car(), xonly, [15.0], 7, 3.0, 2.0)
    assert rx.lat_g == pytest.approx(A, rel=1e-9, abs=1e-12) and rx.long_g == pytest.approx(L, rel=1e-9, abs=1e-12)


def test_no_grip_and_no_drive_is_a_point(monkeypatch):
    gen = ggv.GGVGenerator(Car(), ggv.GGVParams(power_w=0.0, brake_g_cap=0.0, cl_a=0.0, cd_a=0.0,
                                                rolling_resist=0.0))
    monkeypatch.setattr(gen, "_max_lateral_g_at_speed", lambda v: 0.0)
    r = gen.generate(speeds=[10.0], n_dir=5)
    assert np.all(r.lat_g == 0.0) and np.all(r.long_g == 0.0)
    assert r.max_accel_g[0] == 0.0 and r.max_brake_g[0] == 0.0
    side = ggv.GGVGenerator(Car(), ggv.GGVParams(cl_a=0.0))
    monkeypatch.setattr(side, "_max_lateral_g_at_speed", lambda v: 0.0 if v > 2.0 else 1.4)
    rs = side.generate(speeds=[10.0], n_dir=5)
    assert rs.lat_g == pytest.approx(np.zeros((1, 5)), abs=1e-12)
    assert rs.long_g[0, 2] == pytest.approx(rs.max_accel_g[0]) and rs.long_g[0, 0] == pytest.approx(-rs.max_brake_g[0])


def test_a_rear_camber_sweep_on_a_car_without_the_flag():
    car = Car()
    car.p.static_camber_rear = -1.0
    del car.p.use_param_camber
    c = ggv.sweep_parameter(car, ggv.GGVParams(), "static_camber_rear", [-2.0])
    assert c["camber_lever_forced"] is True and car.p.use_param_camber is None


def test_the_cross_check_reports_every_number(monkeypatch):
    gp = ggv.GGVParams.from_powertrain(PT)
    lat = _lat(Car(), 1.0, gp)
    r = _validate(monkeypatch, LT(lat, acc=1.05, brk=0.97), speeds=[12.0])
    gen = ggv.GGVGenerator(Car(), gp)
    acc = gen._max_accel_g_at_speed(12.0, lat)
    brk = gen._max_brake_g_at_speed(12.0, lat)
    assert set(r) == {"ok", "reason", "speeds", "lat_ggv", "lat_lap", "lat_reldiff", "accel_ggv", "accel_lap",
                      "accel_reldiff", "brake_ggv", "brake_lap", "brake_reldiff", "rel_tol", "max_reldiff"}
    assert r["accel_ggv"] == [pytest.approx(acc, rel=1e-9)] and r["accel_lap"] == [pytest.approx(acc * 1.05)]
    assert r["brake_ggv"] == [pytest.approx(brk, rel=1e-9)] and r["brake_lap"] == [pytest.approx(brk * 0.97)]
    assert r["speeds"] == [12.0] and r["ok"] is True
    full = _validate(monkeypatch, LT(lat))
    assert full["speeds"] == pytest.approx(np.linspace(8.0, 30.0, 8).tolist())


@pytest.mark.parametrize("axis", ["lat", "acc", "brk"])
def test_the_tolerance_edge_is_inclusive(monkeypatch, axis):
    gp = ggv.GGVParams.from_powertrain(PT)
    lat = _lat(Car(), 1.0, gp)
    kw = dict(lat=lat * 1.03 if axis == "lat" else lat, acc=1.03 if axis == "acc" else 1.0,
              brk=1.03 if axis == "brk" else 1.0, mu=lat)
    probe = _validate(monkeypatch, LT(**kw), speeds=[10.0])
    key = {"lat": "lat_reldiff", "acc": "accel_reldiff", "brk": "brake_reldiff"}[axis]
    tol = probe[key][0]
    assert tol > 0.01
    edge = _validate(monkeypatch, LT(**kw), speeds=[10.0], rel_tol=tol)
    assert edge["ok"] is True


def test_the_braking_only_note_at_the_tolerance_edge(monkeypatch):
    ct = SimpleNamespace(mu_x_ratio=1.2)
    pt = SimpleNamespace(**vars(PT), combined_tire=ct)
    lat = _lat(Car(), 1.0, ggv.GGVParams.from_powertrain(pt))
    for kw, key in ((dict(lat=lat * 1.03, brk=1.5, mu=lat), "lat_reldiff"),
                    (dict(lat=lat, acc=1.03, brk=1.5), "accel_reldiff")):
        tol = _validate(monkeypatch, LT(**kw), pt=pt, speeds=[10.0])[key][0]
        r = _validate(monkeypatch, LT(**kw), pt=pt, speeds=[10.0], rel_tol=tol)
        assert r["ok"] is False and r["note"] == (
            "REGRESSION: lateral and accel agree; the only divergence is braking. laptime._decel_long and "
            "_accel_long are supposed to share one combined-slip envelope (ellipse + mu_x_ratio). They have drifted "
            "apart — fix the longitudinal models, do not widen this tolerance.")
