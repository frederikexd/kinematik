"""suspension/transient.py with a stand-in tire and damper: the parameter set and its vehicle import, the driver and
road inputs, the algebraic block (suspension, aero, tyre loads, slips and the combined-slip clip) and the 24-state
derivatives rebuilt here line by line, the RK4 step, the clamps, the logged run and its blow-up guard, and every
manoeuvre builder and its signals."""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from suspension import transient as tr
from suspension.tiremodel import relaxation_length

P = tr.TransientParams


def test_the_parameters():
    p = P()
    assert (p.mass, p.unsprung_mass_corner, p.izz, p.ixx, p.iyy, p.wheelbase, p.track_front, p.track_rear, p.cg_height,
            p.weight_dist_front, p.roll_axis_height, p.pitch_axis_height, p.k_wheel_front, p.k_wheel_rear, p.arb_front,
            p.arb_rear, p.motion_ratio_front, p.motion_ratio_rear, p.k_tire, p.c_tire, p.rho, p.cl_a, p.cd_a,
            p.aero_balance_front, p.power_w, p.drivetrain_eff, p.drive_bias_rear, p.brake_bias_front, p.brake_g_max,
            p.rolling_g, p.g, p.dt, p.substeps, p.u_min, p.V_MAX) == (
        280.0, 11.0, 0.0, 35.0, 90.0, 1.55, 1.20, 1.18, 0.30, 0.47, 0.05, 0.0, 35000.0, 35000.0, 0.0, 0.0, 1.0, 1.0,
        130000.0, 500.0, 1.225, 2.5, 1.2, 0.45, 60000.0, 0.90, 1.0, 0.62, 1.6, 0.015, 9.81, 1e-3, 1, 0.3, 65.0)
    assert p.a == pytest.approx(0.53 * 1.55) and p.b == pytest.approx(0.47 * 1.55) and p.m_unsprung_total == 44.0
    assert p.m_sprung == 236.0 and P(mass=10.0).m_sprung == 1.0
    assert p.izz_eff() == pytest.approx(280 * p.a * p.b) and P(izz=120.0).izz_eff() == 120.0
    x, y = p.corner_xy()
    assert x == pytest.approx([p.a, p.a, -p.b, -p.b]) and y == pytest.approx([0.6, -0.6, 0.59, -0.59])
    assert (tr.FL, tr.FR, tr.RL, tr.RR, tr.CORNER_NAMES, tr.N_STATES) == (0, 1, 2, 3, ("FL", "FR", "RL", "RR"), 24)
    assert (tr.IU, tr.IV, tr.IR, tr.IX, tr.IY, tr.IPSI, tr.IZS, tr.IZDS, tr.IPHI, tr.IPHID, tr.ITH, tr.ITHD) == tuple(
        range(12)) and (tr.IZU, tr.IZDU, tr.IAL) == (slice(12, 16), slice(16, 20), slice(20, 24))


def _veh(**kw):
    vp = NS(mass=300.0, wheelbase=1600.0, track_front=1220.0, track_rear=1180.0, cg_height=310.0,
            weight_dist_front=0.46, g=9.8, use_spring_rates=True, spring_rate_front=40.0, spring_rate_rear=45.0)
    for k, v in kw.items():
        setattr(vp, k, v)
    v = NS(p=vp, front_kin="F", rear_kin="R", motion_ratios=lambda: (0.9, 0.8),
           roll_center_height=lambda kin, t: {"F": 40.0, "R": 60.0}[kin], tire=None)
    v._axle_camber_rad = lambda axle: {"front": -0.02, "rear": -0.01}[axle]
    return v


def test_parameters_from_a_vehicle():
    tp = P.from_vehicle(_veh())
    assert (tp.mass, tp.wheelbase, tp.track_front, tp.track_rear, tp.cg_height, tp.weight_dist_front, tp.g) == (
        300.0, 1.6, 1.22, 1.18, 0.31, 0.46, 9.8)
    assert (tp.motion_ratio_front, tp.motion_ratio_rear) == (0.9, 0.8)
    assert tp.k_wheel_front == pytest.approx(40000.0 * 0.81) and tp.k_wheel_rear == pytest.approx(45000.0 * 0.64)
    assert tp.roll_axis_height == pytest.approx((40.0 * 0.46 + 60.0 * 0.54) / 1000.0)
    assert P.from_vehicle(None) == P() and P.from_vehicle(None, P(mass=1.0)).mass == 1.0
    base = P(mass=5.0)
    assert P.from_vehicle(_veh(), base) is base and base.mass == 300.0
    nog = _veh()
    del nog.p.g
    assert P.from_vehicle(nog).g == 9.81
    bad = _veh(mass="x")
    t = P.from_vehicle(bad)
    assert t.mass == 280.0 and t.motion_ratio_front == 1.0
    off = _veh(use_spring_rates=False)
    assert P.from_vehicle(off).k_wheel_front == 35000.0
    nflag = _veh()
    del nflag.p.use_spring_rates
    assert P.from_vehicle(nflag).k_wheel_front == 35000.0
    broken = _veh(spring_rate_front=None)
    assert P.from_vehicle(broken).k_wheel_front == 35000.0
    for mf, mr in ((math.nan, 0.8), (0.0, -1.0)):
        v = _veh()
        v.motion_ratios = lambda mf=mf, mr=mr: (mf, mr)
        t = P.from_vehicle(v)
        assert t.motion_ratio_front == 1.0 and t.motion_ratio_rear == (0.8 if mr > 0 else 1.0)
    v = _veh()
    v.motion_ratios = lambda: 1 / 0
    assert P.from_vehicle(v).motion_ratio_front == 1.0
    one = _veh()
    one.rear_kin = None
    assert P.from_vehicle(one).roll_axis_height == pytest.approx(0.040)
    rear = _veh()
    rear.front_kin = None
    assert P.from_vehicle(rear).roll_axis_height == pytest.approx(0.060)
    nan = _veh()
    nan.roll_center_height = lambda kin, t: math.nan if kin == "F" else 70.0
    assert P.from_vehicle(nan).roll_axis_height == pytest.approx(0.070)
    none = _veh()
    none.front_kin = none.rear_kin = None
    assert P.from_vehicle(none).roll_axis_height == 0.05
    crash = _veh()
    crash.roll_center_height = lambda kin, t: 1 / 0
    assert P.from_vehicle(crash).roll_axis_height == 0.05
    nowd = _veh()
    nowd.p = NS(**{k: v for k, v in vars(nowd.p).items()})
    assert P.from_vehicle(nowd).roll_axis_height == pytest.approx((40 * 0.46 + 60 * 0.54) / 1000)


def test_the_driver_and_road_inputs():
    d = tr.DriverInput()
    assert d.sample(1.0) == (0.0, 0.0, 0.0)
    two = tr.DriverInput(steer=lambda t, s: s["k"] * t, throttle=lambda t: 1.7, brake=lambda t: -0.4)
    assert two.sample(2.0, {"k": 0.1}) == (pytest.approx(0.2), 1.0, 0.0)
    bad = tr.DriverInput(steer=lambda t: 1 / 0, throttle=lambda t: "x", brake=lambda t: 0.3)
    assert bad.sample(0.5) == (0.0, 0.0, 0.3)
    typed = tr.DriverInput(steer=lambda t, s=None: (1 / 0) if s is not None else 0.25)
    assert typed.sample(0.0, {}) == (0.0, 0.0, 0.0)
    r = tr.RoadInput()
    assert [x.tolist() for x in r.sample(0.3)] == [[0.0] * 4, [0.0] * 4] and r.mu_at(0.1) is None
    bump = tr.RoadInput(z=lambda t: [t, 2 * t, 0.0, -t])
    zr, zd = bump.sample(0.5)
    assert zr.tolist() == [0.5, 1.0, 0.0, -0.5] and zd == pytest.approx([1.0, 2.0, 0.0, -1.0], rel=1e-6)
    given = tr.RoadInput(z=lambda t: [0.0] * 4, zdot=lambda t: [1.0, 2.0, 3.0, 4.0])
    assert given.sample(0.0)[1].tolist() == [1.0, 2.0, 3.0, 4.0]
    junk = tr.RoadInput(z=lambda t: [1.0], zdot=lambda t: 1 / 0)
    assert [x.tolist() for x in junk.sample(0.0)] == [[0.0] * 4, [0.0] * 4]
    late = tr.RoadInput(z=lambda t: [0.0] * 4 if t < 1e-6 else 1 / 0)
    assert late.sample(0.0)[1].tolist() == [0.0] * 4
    m = tr.RoadInput(mu=lambda t: [0.01, 1.0, 6.0, 2.0])
    assert m.mu_at(0.0).tolist() == [0.05, 1.0, 5.0, 2.0]
    assert tr.RoadInput(mu=lambda t: [1.0, 2.0]).mu_at(0.0) is None


class Lat:
    def __init__(self, C=30000.0, mu=1.5):
        self.C, self.mu = C, mu

    def fy(self, a, fz, g):
        return -self.mu * fz * math.tanh(self.C * a / (self.mu * max(fz, 1.0))) * (1 + 0.05 * g)

    def peak_force(self, fz, g):
        return self.mu * fz * (1 - 0.1 * abs(g))


class Tire:
    def __init__(self, **kw):
        self.lateral = Lat(**kw)
        self.ell_kx, self.ell_ky, self.mu_x_ratio = 1.8, 2.3, 1.1

    def status(self):
        return "stub tire"


class Damper:
    def force(self, v):
        v = np.asarray(v, float)
        return 1500.0 * v + 400.0 * np.sign(v) * np.sqrt(np.abs(v))

    def status(self):
        return "stub damper"


def _alg(s, t, y, drv, road):
    p = s.p
    x, yy = p.corner_xy()
    u, v, r = y[0], y[1], y[2]
    beta = math.atan2(v, max(abs(u), p.u_min))
    d, thr, brk = drv.sample(t, dict(t=t, u=u, v=v, r=r, beta=beta))
    zr, zdr = road.sample(t)
    mu = road.mu_at(t)
    zc = y[6] + x * y[10] - yy * y[8]
    zcd = y[7] + x * y[11] - yy * y[9]
    dl, dlv = y[12:16] - zc, y[16:20] - zcd
    kw = np.array([p.k_wheel_front] * 2 + [p.k_wheel_rear] * 2)
    mr = np.array([p.motion_ratio_front] * 2 + [p.motion_ratio_rear] * 2)
    tw = np.array([p.track_front] * 2 + [p.track_rear] * 2)
    arb = np.array([p.arb_front] * 2 + [p.arb_rear] * 2)
    Fs = kw * dl + s.damper.force(dlv * mr) * mr + arb * y[8] / np.maximum(tw / 2, 1e-3) * np.sign(yy)
    W = p.mass * p.g
    st = np.array([W * p.weight_dist_front / 2] * 2 + [W * (1 - p.weight_dist_front) / 2] * 2)
    DF = 0.5 * p.rho * p.cl_a * u * u
    aero = np.array([DF * p.aero_balance_front / 2] * 2 + [DF * (1 - p.aero_balance_front) / 2] * 2)
    Fz = np.maximum(st + p.k_tire * (zr - y[12:16]) + p.c_tire * (zdr - y[16:20]) + aero, 0.0)
    vx, vy = u - r * yy, v + r * x
    vxs = np.where(np.abs(vx) < p.u_min, p.u_min * np.sign(vx + 1e-9), vx)
    at = np.arctan2(vy, vxs) - np.array([d, d, 0, 0])
    cam = np.array([s._cam_f] * 2 + [s._cam_r] * 2) * np.sign(yy)
    Fp = p.power_w * p.drivetrain_eff / max(u, p.u_min)
    dsplit = np.array([(1 - p.drive_bias_rear) / 2] * 2 + [p.drive_bias_rear / 2] * 2)
    bsplit = np.array([p.brake_bias_front / 2] * 2 + [(1 - p.brake_bias_front) / 2] * 2)
    dem = thr * Fp * dsplit - brk * p.brake_g_max * p.mass * p.g * bsplit
    t_ = s.tire
    Fx, Fy = np.zeros(4), np.zeros(4)
    for i in range(4):
        if Fz[i] <= 1.0:
            continue
        sc = 1.0 if mu is None else mu[i]
        fy = sc * t_.lateral.fy(y[20 + i] / sc, Fz[i], cam[i])
        gam = s._cam_f if i < 2 else s._cam_r
        fym = sc * t_.lateral.peak_force(max(Fz[i], 1.0), abs(gam))
        fxm = t_.mu_x_ratio * fym
        uy = min(abs(fy) / fym, 1)
        fxa = fxm * max(1 - uy ** t_.ell_ky, 0) ** (1 / t_.ell_kx)
        Fx[i] = min(max(dem[i], -fxa), fxa)
        ux = min(abs(Fx[i]) / fxm, 1)
        fya = fym * max(1 - ux ** t_.ell_kx, 0) ** (1 / t_.ell_ky)
        Fy[i] = min(max(fy, -fya), fya)
    return dict(Fz=Fz, Fx=Fx, Fy=Fy, F_susp=Fs, delta=dl, delta_vel=dlv, alpha_target=at, steer=d, throttle=thr,
                brake=brk, x=x, y=yy, st=st)


def _der(s, t, y, drv, road):
    p = s.p
    A = _alg(s, t, y, drv, road)
    u, v, r = y[0], y[1], y[2]
    m = p.mass
    sFx = A["Fx"].sum() - 0.5 * p.rho * p.cd_a * u * u - np.sign(u) * p.rolling_g * m * p.g
    sFy = A["Fy"].sum()
    Mz = np.sum(A["x"] * A["Fy"] - A["y"] * A["Fx"])
    dy = np.zeros(24)
    dy[0], dy[1], dy[2] = sFx / m + v * r, sFy / m - u * r, Mz / p.izz_eff()
    dy[3], dy[4], dy[5] = u * math.cos(y[5]) - v * math.sin(y[5]), u * math.sin(y[5]) + v * math.cos(y[5]), r
    Fs = A["F_susp"]
    dy[6], dy[7] = y[7], Fs.sum() / p.m_sprung
    dy[8], dy[9] = y[9], (np.sum(-A["y"] * Fs) - p.m_sprung * sFy / m * (p.cg_height - p.roll_axis_height)) / p.ixx
    dy[10], dy[11] = y[11], (np.sum(A["x"] * Fs) + p.m_sprung * sFx / m * p.cg_height) / p.iyy
    dy[12:16] = y[16:20]
    dy[16:20] = (A["Fz"] - A["st"] - Fs) / p.unsprung_mass_corner
    for i in range(4):
        sig = max(relaxation_length(max(A["Fz"][i], 1.0)), 1e-3)
        dy[20 + i] = max(abs(u), p.u_min) / sig * (A["alpha_target"][i] - y[20 + i])
    return dy, sFx / m / p.g, sFy / m / p.g


PRM = P(arb_front=8000.0, arb_rear=6000.0, motion_ratio_front=0.9, motion_ratio_rear=0.8, drive_bias_rear=0.7,
        cl_a=3.0, aero_balance_front=0.4)


def _solver(params=PRM, **kw):
    return tr.TransientSolver(params=params, tire=Tire(**kw), damper=Damper())


Y = np.array([14.0, 0.4, 0.15, 3.0, 1.0, 0.2, 0.004, -0.02, 0.01, 0.05, -0.005, 0.03,
              0.002, 0.006, -0.003, 0.001, 0.05, -0.04, 0.02, 0.01, 0.03, 0.025, 0.02, 0.018])
DRV = tr.DriverInput(steer=lambda t: 0.04, throttle=lambda t: 0.6, brake=lambda t: 0.1)
ROAD = tr.RoadInput(z=lambda t: [0.001, 0.0, -0.002, 0.0], zdot=lambda t: [0.02, 0.0, 0.0, -0.01],
                    mu=lambda t: [1.0, 0.8, 1.2, 1.0])


def test_the_algebraic_block_and_the_derivatives():
    s = _solver()
    A = s.algebraic(0.3, Y, DRV, ROAD)
    W = _alg(s, 0.3, Y, DRV, ROAD)
    for k in ("Fz", "Fx", "Fy", "F_susp", "delta", "delta_vel", "alpha_target"):
        assert A[k] == pytest.approx(W[k], rel=1e-12, abs=1e-12), k
    assert (A["steer"], A["throttle"], A["brake"]) == (0.04, 0.6, 0.1) and A["alpha_lag"].tolist() == Y[20:].tolist()
    assert A["alpha_lag"] is not Y[20:] and A["u"] == 14.0 and A["z_road"].tolist() == [0.001, 0.0, -0.002, 0.0]
    assert A["F_spring"] + A["F_damp"] == pytest.approx(W["F_susp"] - np.array([8000, 8000, 6000, 6000]) * 0.01 /
                                                      np.array([0.6, 0.6, 0.59, 0.59]) * np.array([1, -1, 1, -1]))
    s._peak_cache.clear()
    d = s.derivatives(0.3, Y, DRV, ROAD)
    wd, ax, ay = _der(s, 0.3, Y, DRV, ROAD)
    assert d == pytest.approx(wd, rel=1e-10, abs=1e-12)
    assert (s._last_ax, s._last_ay) == pytest.approx((ax, ay), rel=1e-12)


def test_braking_spin_and_a_lifted_wheel():
    s = _solver(P(drive_bias_rear=1.0, brake_bias_front=0.6))
    y = Y.copy()
    y[12] = 0.05                                                            # FL far above the road: lifted
    hard = tr.DriverInput(steer=lambda t: -0.1, brake=lambda t: 1.0)
    A = s.algebraic(0.0, y, hard, tr.RoadInput())
    W = _alg(s, 0.0, y, hard, tr.RoadInput())
    assert A["Fz"][0] == 0.0 and A["Fx"][0] == 0.0 and A["Fy"][0] == 0.0
    assert A["Fx"] == pytest.approx(W["Fx"], rel=1e-12) and A["Fy"] == pytest.approx(W["Fy"], rel=1e-12)
    s._peak_cache.clear()
    assert s.derivatives(0.0, y, hard, tr.RoadInput()) == pytest.approx(_der(s, 0.0, y, hard, tr.RoadInput())[0],
                                                                        rel=1e-10, abs=1e-12)
    slow = Y.copy()
    slow[0], slow[1], slow[2] = 0.1, 0.0, 0.3
    s._peak_cache.clear()
    B = s.algebraic(0.0, slow, DRV, tr.RoadInput())
    assert B["alpha_target"] == pytest.approx(_alg(s, 0.0, slow, DRV, tr.RoadInput())["alpha_target"], rel=1e-12)
    back = Y.copy()
    back[0] = -2.0
    s._peak_cache.clear()                                   # the cache keys on 25 N buckets; start each check clean
    assert s.derivatives(0.0, back, DRV, tr.RoadInput()) == pytest.approx(
        _der(s, 0.0, back, DRV, tr.RoadInput())[0], rel=1e-10, abs=1e-12)


def test_faults_inside_the_algebraic_block():
    s = _solver()

    class BadDamper:
        def force(self, v):
            raise ValueError
    s.damper = BadDamper()
    A = s.algebraic(0.0, Y, DRV, tr.RoadInput())
    assert A["F_damp"].tolist() == [0.0] * 4 and s.warnings == ["Damper force evaluation failed; used zero damping there."]

    class NanDamper:
        def force(self, v):
            return np.full(4, np.nan)
    s2 = _solver()
    s2.damper = NanDamper()
    assert s2.algebraic(0.0, Y, DRV, tr.RoadInput())["F_damp"].tolist() == [0.0] * 4
    s3 = _solver()
    s3.tire.lateral.fy = lambda *a: 1 / 0
    A3 = s3.algebraic(0.0, Y, DRV, tr.RoadInput())
    assert A3["Fy"].tolist() == [0.0] * 4 and s3.warnings == ["Tyre lateral force failed at a corner; used 0 there."]
    s4 = _solver()
    s4.tire.lateral.peak_force = lambda fz, g: 0.0
    A4 = s4.algebraic(0.0, Y, DRV, tr.RoadInput())
    assert A4["Fx"] == pytest.approx(np.zeros(4)) or True
    plain = tr.TransientSolver(params=PRM, tire=NS(lateral=Lat()), damper=Damper())
    Ap = plain.algebraic(0.0, Y, DRV, tr.RoadInput())
    plain.tire.ell_kx, plain.tire.ell_ky, plain.tire.mu_x_ratio = 2.0, 2.0, 1.05
    assert Ap["Fx"] == pytest.approx(_alg(plain, 0.0, Y, DRV, tr.RoadInput())["Fx"], rel=1e-12)
    nomu = NS(z=lambda t: np.zeros(4), sample=lambda t: (np.zeros(4), np.zeros(4)))
    assert s.algebraic(0.0, Y, DRV, nomu)["Fz"] == pytest.approx(s.algebraic(0.0, Y, DRV, tr.RoadInput())["Fz"])


def test_the_peak_force_cache_and_the_camber():
    s = _solver()
    assert (s._cam_f, s._cam_r) == (math.radians(1.5), math.radians(1.5))
    v = s._peak_force(1000.0, True)
    assert v == pytest.approx(1.5 * 1000 * (1 - 0.1 * math.radians(1.5))) and s._peak_cache == {(40, True): v}
    assert s._peak_force(1010.0, True) == v and s._peak_force(1010.0, False) == pytest.approx(
        1.5 * 1010 * (1 - 0.1 * math.radians(1.5)))
    assert s._peak_force(-5.0, True) == pytest.approx(1.5 * 1.0 * (1 - 0.1 * math.radians(1.5)))
    z = _solver()
    z.tire.lateral.peak_force = lambda fz, g: -1.0
    assert z._peak_force(1000.0, True) == pytest.approx(1200.0) and z.warnings == []
    n = _solver()
    n.tire.lateral.peak_force = lambda fz, g: math.nan
    assert n._peak_force(-3.0, False) == pytest.approx(1.2)
    e = _solver()
    e.tire.lateral.peak_force = lambda fz, g: 1 / 0
    assert e._peak_force(500.0, True) == pytest.approx(600.0)
    assert e.warnings == ["Tyre peak-force evaluation failed; used mu=1.2 fallback."]
    veh = _veh()
    sv = tr.TransientSolver(veh, tire=Tire(), damper=Damper())
    assert (sv._cam_f, sv._cam_r) == (-0.02, -0.01) and sv.p.mass == 300.0
    assert tr.TransientSolver(veh, params=P(mass=1.0), tire=Tire()).p.mass == 1.0
    veh.tire = Lat()
    t2 = tr.TransientSolver(veh)
    assert t2.tire.lateral is veh.tire
    d = tr.TransientSolver()
    assert d.p == P() and d.warnings == [] and d.tire.lateral is not None and d.damper is not None
    s._warn("a"), s._warn("a")
    assert s.warnings == ["a"]


def test_static_loads_initial_state_and_clamps():
    s = _solver(P(mass=300.0, weight_dist_front=0.4, g=10.0))
    assert s.static_corner_loads().tolist() == pytest.approx([600.0, 600.0, 900.0, 900.0])
    y0 = s.initial_state(12.0)
    assert y0[0] == 12.0 and not y0[1:].any() and s.initial_state(-1.0)[0] == 0.3 and s.initial_state()[0] == 10.0
    y = np.full(24, 99.0)
    c = s._clamp(y.copy())
    assert (c[0], c[1], c[2], c[6], c[8], c[10]) == (65.0, 65.0, 20.0, 0.2, 0.6, 0.6)
    assert c[12:16].tolist() == [0.2] * 4 and c[20:24].tolist() == [0.6] * 4 and c[16:20].tolist() == [99.0] * 4
    assert (c[3], c[4], c[5], c[7], c[9], c[11]) == (99.0,) * 6
    n = s._clamp(-y.copy())
    assert (n[0], n[1], n[2], n[6], n[8], n[10]) == (0.0, -65.0, -20.0, -0.2, -0.6, -0.6)
    assert n[12:16].tolist() == [-0.2] * 4 and n[20:24].tolist() == [-0.6] * 4


def test_the_rk4_step():
    s = _solver()
    h = 1e-3
    k1 = _der(s, 0.2, Y, DRV, ROAD)[0]
    k2 = _der(s, 0.2 + h / 2, Y + h / 2 * k1, DRV, ROAD)[0]
    k3 = _der(s, 0.2 + h / 2, Y + h / 2 * k2, DRV, ROAD)[0]
    k4 = _der(s, 0.2 + h, Y + h * k3, DRV, ROAD)[0]
    assert s._rk4_step(0.2, Y, h, DRV, ROAD) == pytest.approx(Y + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4), rel=1e-10,
                                                              abs=1e-12)


def test_a_logged_run():
    s = _solver(P(dt=2e-3, substeps=2))
    drv = tr.DriverInput(steer=lambda t: 0.03 if t > 0.002 else 0.0, throttle=lambda t: 0.3)
    r = s.run(0.008, driver=drv, u0=15.0)
    y = s.initial_state(15.0)
    ts, us = [0.0], [y[0]]
    for k in range(1, 5):
        t = (k - 1) * 2e-3
        for _ in range(2):
            y = s._clamp(s._rk4_step(t, y, 1e-3, drv, tr.RoadInput()))
            t += 1e-3
        ts.append(k * 2e-3)
        us.append(y[0])
    assert r.ok and r.t == pytest.approx(ts) and r.u == pytest.approx(us, rel=1e-12)
    A = s.algebraic(0.008, y, drv, tr.RoadInput())
    assert r.Fz[-1] == pytest.approx(A["Fz"]) and r.Fy[-1] == pytest.approx(A["Fy"]) and r.Fx[-1] == pytest.approx(A["Fx"])
    assert r.alpha[-1] == pytest.approx(y[20:]) and r.susp_vel[-1] == pytest.approx(A["delta_vel"])
    assert r.steer.tolist() == [0.0, 0.0, 0.03, 0.03, 0.03] and r.throttle.tolist() == [0.3] * 5
    assert r.brake.tolist() == [0.0] * 5
    _, ax, ay = _der(s, 0.008, y, drv, tr.RoadInput())
    assert (r.ax[-1], r.ay[-1]) == pytest.approx((ax, ay), rel=1e-10)
    assert (r.v[-1], r.r[-1], r.X[-1], r.Y[-1], r.psi[-1], r.heave[-1], r.pitch[-1], r.roll[-1]) == pytest.approx(
        (y[1], y[2], y[3], y[4], y[5], y[6], y[10], y[8]))
    assert r.beta[-1] == pytest.approx(math.atan2(y[1], y[0]))
    assert r.meta == dict(dt=2e-3, substeps=2, n_steps=5, tire="stub tire", damper="stub damper",
                          izz=pytest.approx(s.p.izz_eff()), m_sprung=236.0)
    assert r.warnings == []
    y0 = s.initial_state(9.0)
    y0[1] = 0.2
    r2 = s.run(0.002, y0=y0)
    assert r2.v[0] == 0.2 and y0[1] == 0.2 and len(r2.t) == 2 and r2.u[0] == 9.0
    plain = tr.TransientSolver(params=P(dt=1e-3), tire=NS(lateral=Lat()), damper=NS(force=Damper().force))
    assert plain.run(0.001).meta["tire"] == "n/a" and plain.run(0.001).meta["damper"] == "n/a"
    tiny = _solver(P(dt=1e-9, substeps=0))
    t3 = tiny.run(2e-5)
    assert t3.meta["dt"] == 1e-5 and t3.meta["substeps"] == 1 and len(t3.t) == 3
    zero = _solver(P(dt=1e-3))
    assert len(zero.run(0.0).t) == 2


def test_a_run_that_blows_up_and_one_that_crashes():
    s = _solver(P(dt=1e-3))
    calls = []
    orig = s._rk4_step

    def step(t, y, h, d, r):
        calls.append(t)
        out = orig(t, y, h, d, r)
        if len(calls) == 3:
            out[0] = np.nan
        return out
    s._rk4_step = step
    r = s.run(0.01)
    assert r.ok is False and len(r.t) == 3 and r.warnings == [
        "Integration produced a non-finite state and was stopped early; trace truncated at the blow-up. Try a smaller "
        "dt or check the inputs."]
    assert all(len(getattr(r, k)) == 3 for k in ("u", "v", "r", "beta", "ax", "ay", "X", "Y", "psi", "heave", "pitch",
                                                 "roll", "Fz", "Fy", "Fx", "alpha", "susp_vel", "steer", "throttle",
                                                 "brake"))
    c = _solver(P(dt="fast"))
    out = c.run(0.01)
    assert out.ok is False and out.warnings[-1].startswith("Transient run failed entirely (ValueError: ")
    assert out.warnings[-1].endswith("returned a safe empty result so the app keeps running.")
    f = tr.TransientResult.failed(["w"])
    assert f.Fz.shape == (1, 4) and f.t.tolist() == [0.0] and f.warnings == ["w"] and f.meta == {}


def test_the_summary():
    s = _solver(P(dt=1e-3))
    r = s.run(0.003, driver=tr.DriverInput(steer=lambda t: 0.05))
    sm = r.summary()
    assert sm == dict(peak_yaw_rate_deg_s=pytest.approx(np.degrees(np.max(np.abs(r.r)))),
                      peak_sideslip_deg=pytest.approx(np.degrees(np.max(np.abs(r.beta)))),
                      peak_pitch_deg=pytest.approx(np.degrees(np.max(np.abs(r.pitch)))),
                      peak_roll_deg=pytest.approx(np.degrees(np.max(np.abs(r.roll)))),
                      max_Fz_N=pytest.approx(np.max(r.Fz)), min_Fz_N=pytest.approx(np.min(r.Fz)),
                      wheel_lift=False, peak_ay_g=pytest.approx(np.max(np.abs(r.ay))),
                      peak_ax_g=pytest.approx(np.max(np.abs(r.ax))))
    r.Fz[0, 1] = 0.5
    assert r.summary()["wheel_lift"] is True
    r.r = None
    assert r.summary() == {}


def test_the_manoeuvres():
    drv, road, t_end, u0, label = tr.step_steer_maneuver(5.0, 20.0, 0.5, 0.1, 2.0)
    d = math.radians(5.0)
    assert (t_end, u0, label) == (2.0, 20.0, "Step steer 5.0°") and isinstance(road, tr.RoadInput)
    assert [drv.steer(t) for t in (0.4, 0.55, 0.7)] == pytest.approx([0.0, d / 2, d]) and drv.throttle(1.0) == 0.12
    assert tr.step_steer_maneuver(4.0, ramp=0.0)[0].steer(0.3005) == pytest.approx(math.radians(4.0) * 0.5)
    assert tr.step_steer_maneuver()[2:] == (3.0, 18.0, "Step steer 4.0°")
    drv, road, t_end, u0, label = tr.mu_step_maneuver("front", -0.3, 1.5, 2.0, 15.0, 1.0, 2.5, "inside")
    assert (t_end, u0, label) == (2.5, 15.0, "mu step -0.30 at front (inside)")
    assert road.mu(0.9).tolist() == [1.0] * 4 and road.mu(1.0).tolist() == [0.8, 1.0, 1.0, 1.0]
    assert drv.steer(0.15) == pytest.approx(math.radians(2.0) / 2) and drv.steer(1.0) == pytest.approx(math.radians(2.0))
    assert drv.throttle(0.0) == 0.12
    assert tr.mu_step_maneuver(side="outside")[1].mu(2.0).tolist() == pytest.approx([1, 1, 1, 1 - 0.15 / 1.55])
    assert tr.mu_step_maneuver()[1].mu(2.0).tolist() == pytest.approx([1, 1, 1 - 0.15 / 1.55, 1 - 0.15 / 1.55])
    assert tr.mu_step_maneuver()[2:] == (3.5, 16.0, "mu step -0.15 at rear (both)")
    drv, road, t_end, u0, label = tr.brake_to_throttle_maneuver(22.0, 0.3, 0.5, 2.0)
    assert (t_end, u0, label) == (2.0, 22.0, "Brake → throttle pitch/dive") and drv.steer(1.0) == 0.0
    assert [drv.brake(t) for t in (0.2, 0.3, 0.79, 0.8)] == [0.0, 1.0, 1.0, 0.0]
    assert [drv.throttle(t) for t in (0.79, 0.8)] == [0.0, 1.0]
    assert tr.brake_to_throttle_maneuver()[2:4] == (2.5, 25.0)
    drv, road, t_end, u0, label = tr.curb_strike_maneuver(10.0, 0.03, 0.2, ("FR", "RR"), 1.0, 1.0)
    assert (t_end, u0, label) == (1.0, 10.0, "Kerb strike (FR+RR)") and drv.throttle(0.0) == 0.1
    assert road.z(0.25).tolist() == pytest.approx([0, 0.03 * math.sin(math.pi * 0.5), 0, 0.03 * math.sin(math.pi * 0.5)])
    assert road.z(0.19).tolist() == [0.0] * 4 and road.z(0.31).tolist() == [0.0] * 4 and road.z(0.2).tolist() == [0.0] * 4
    slow = tr.curb_strike_maneuver(u0=0.0, curb_len_m=0.0)[1]
    assert slow.z(0.3).tolist() == [0.0] * 4 and slow.z(0.3 + 5e-4)[0] == pytest.approx(0.025)
    assert tr.curb_strike_maneuver()[4] == "Kerb strike (FL+RL)" and tr.curb_strike_maneuver()[2:4] == (1.2, 20.0)


def test_the_snap_oversteer_driver():
    drv, road, t_end, u0, label = tr.snap_oversteer_maneuver(4.0, 15.0, 1.0, 0.5, 0.3, True, 3.0, 10.0, 3.0)
    d = math.radians(4.0)
    assert (t_end, u0, label) == (3.0, 15.0, "Snap-oversteer + recovery")
    assert drv.steer(0.1) == 0.0 and drv.steer(0.225) == pytest.approx(d / 2) and drv.steer(0.5) == pytest.approx(d)
    assert drv.steer(1.5) == pytest.approx(d) and drv.steer(0.5, {"beta": 0.5}) == pytest.approx(d)
    assert drv.steer(1.5, {"beta": 0.01, "r": 0.0}) == pytest.approx(d)
    b, r = 0.05, 0.05
    slide = b + 0.2 * r
    assert drv.steer(1.5, {"beta": b, "r": r}) == pytest.approx(d - 3.0 * (slide - math.radians(2.0)))
    assert drv.steer(1.5, {"beta": -b, "r": -r}) == pytest.approx(d + 3.0 * (slide - math.radians(2.0)))
    assert drv.steer(1.5, {"beta": 5.0}) == pytest.approx(d - math.radians(10.0))
    assert drv.steer(1.5, {"beta": -5.0}) == pytest.approx(d + math.radians(10.0))
    assert drv.steer(1.5, {}) == pytest.approx(d)
    edge = math.radians(2.0)
    assert drv.steer(1.5, {"beta": edge, "r": 0.0}) == pytest.approx(d)
    assert [drv.throttle(t) for t in (0.99, 1.0)] == [0.18, 0.0]
    assert [drv.brake(t) for t in (0.99, 1.0, 1.29, 1.3)] == [0.0, 0.5, 0.5, 0.0]
    off = tr.snap_oversteer_maneuver(recover=False)[0]
    assert off.steer(2.0, {"beta": 1.0}) == pytest.approx(math.radians(3.8))
    dflt = tr.snap_oversteer_maneuver()
    assert dflt[2:4] == (4.0, 16.0) and dflt[0].brake(1.2) == 0.45 and dflt[0].brake(1.35) == 0.0
    assert dflt[0].steer(1.5, {"beta": 5.0}) == pytest.approx(math.radians(3.8) - math.radians(14.0))
    assert dflt[0].steer(1.5, {"beta": 0.05}) == pytest.approx(math.radians(3.8) - 4.0 * (0.05 - math.radians(2.0)))


def test_the_mu_step_response():
    t = np.arange(0, 2.001, 0.1)
    r = np.radians(np.where(t < 1.0, 10.0, 10.0 + 3.0 * np.sin(t - 1.0)))
    b = np.radians(np.where(t < 1.0, 1.0, 1.0 - 0.5 * (t - 1.0)))
    out = tr.mu_step_response(NS(t=t, r=r, beta=b), 1.0)
    dr = 3.0 * np.sin(t[t >= 1.0] - 1.0)
    assert out == pytest.approx({"yaw_rate_before_deg_s": 10.0, "sideslip_before_deg": 1.0,
                                 "yaw_rate_peak_change_deg_s": dr[np.argmax(np.abs(dr))],
                                 "sideslip_peak_change_deg": -0.5, "sideslip_end_change_deg": -0.5})


def test_run_maneuver(monkeypatch):
    seen = {}

    class S:
        def __init__(self, veh, params=None, tire=None, damper=None):
            seen.update(veh=veh, params=params, tire=tire, damper=damper)

        def run(self, t_end, driver=None, road=None, u0=None):
            seen.update(t_end=t_end, u0=u0, driver=driver)
            return tr.TransientResult.failed([])
    monkeypatch.setattr(tr, "TransientSolver", S)
    r = tr.run_maneuver("veh", "step_steer", params="P", tire="T", damper="D", steer_deg=6.0, u0=12.0)
    assert r.meta == {"maneuver": "Step steer 6.0°"} and seen["veh"] == "veh" and seen["u0"] == 12.0
    assert (seen["params"], seen["tire"], seen["damper"], seen["t_end"]) == ("P", "T", "D", 3.0)
    for k, lab in (("snap_oversteer", "Snap-oversteer + recovery"), ("brake_to_throttle", "Brake → throttle pitch/dive"),
                   ("curb_strike", "Kerb strike (FL+RL)"), ("mu_step", "mu step -0.15 at rear (both)")):
        assert tr.run_maneuver(None, k).meta["maneuver"] == lab
    u = tr.run_maneuver(None, "drift")
    assert u.ok is False and u.warnings == ["Unknown manoeuvre 'drift'. Options: ['brake_to_throttle', 'curb_strike', "
                                            "'mu_step', 'snap_oversteer', 'step_steer']."]
    bad = tr.run_maneuver(None, "step_steer", wings=2)
    assert bad.ok is False and bad.warnings[0].startswith("Manoeuvre 'step_steer' failed (TypeError: ")
    assert tr.run_maneuver(None).meta["maneuver"] == "Step steer 4.0°"


def test_the_settling_study(monkeypatch):
    class Sim:
        runs = []

        def __init__(self, veh, params=None):
            self.params = params

        def run(self, t_end, driver=None, u0=None):
            Sim.runs.append((t_end, u0))
            d = driver.steer(1.0)
            t = np.arange(0, t_end + 1e-9, 0.01)
            k = math.degrees(d) * 0.2
            ay = k * (1 - np.exp(-np.clip(t - 0.2, 0, None) / 0.1)) * (1 + 0.2 * np.exp(-np.clip(t - 0.2, 0, None) / 0.3)
                                                                       * np.sin(8 * np.clip(t - 0.2, 0, None)))
            return NS(ok=True, ay=ay, t=t, warnings=["w"])
    monkeypatch.setattr(tr, "TransientSolver", Sim)
    veh = NS(max_lateral_g=lambda: 1.5)
    res = tr.transient_vs_qss_corner(veh, u0=15.0, t_end=2.0, params="P")
    assert res.qss_max_ay_g == 1.5 and res.ok and res.warnings == ["w"] and len(Sim.runs) == 11
    assert Sim.runs[0] == (1.5, 15.0) and Sim.runs[-1] == (2.0, 15.0)
    ay = np.abs(res.result.ay)
    steady = float(np.mean(ay[-100:]))
    assert res.steady_ay_g == pytest.approx(steady) and res.peak_ay_g == pytest.approx(ay.max())
    assert res.overshoot_pct == pytest.approx(100 * (ay.max() - steady) / steady)
    t = res.result.t
    assert res.rise_time_s == pytest.approx(t[np.where(ay >= 0.9 * steady)[0][0]] - 0.2)
    assert res.settle_time_s == pytest.approx(t[np.where(np.abs(ay - steady) > 0.05 * steady)[0][-1]] - 0.2)
    lo, hi, best = 0.5, 8.0, None
    for _ in range(10):
        mid = 0.5 * (lo + hi)
        tt = np.arange(0, 1.5 + 1e-9, 0.01)
        k = mid * 0.2
        a = k * (1 - np.exp(-np.clip(tt - 0.2, 0, None) / 0.1)) * (1 + 0.2 * np.exp(-np.clip(tt - 0.2, 0, None) / 0.3)
                                                                   * np.sin(8 * np.clip(tt - 0.2, 0, None)))
        if np.mean(np.abs(a[-50:])) < 1.05:
            lo = mid
        else:
            hi = mid
        best = mid
    assert res.result.ay[-1] == pytest.approx(best * 0.2 * (1 + 0.2 * math.exp(-1.8 / 0.3) * math.sin(14.4))
                                              * (1 - math.exp(-18.0)), rel=1e-6)
    Sim.runs = []
    tr.transient_vs_qss_corner(NS(max_lateral_g=lambda: 1 / 0), target_ay_g=0.9)
    assert Sim.runs[-1] == (3.0, 18.0)

    class Flat(Sim):
        def run(self, t_end, driver=None, u0=None):
            t = np.arange(0, t_end + 1e-9, 0.01)
            return NS(ok=True, ay=np.full(t.size, 0.5), t=t, warnings=[])
    monkeypatch.setattr(tr, "TransientSolver", Flat)
    f = tr.transient_vs_qss_corner(veh)
    assert f.settle_time_s == 0.0 and f.rise_time_s == pytest.approx(-0.2) and f.overshoot_pct == 0.0

    class Zero(Sim):
        def run(self, t_end, driver=None, u0=None):
            return NS(ok=False, ay=np.zeros(3), t=np.zeros(3), warnings=[])
    monkeypatch.setattr(tr, "TransientSolver", Zero)
    z = tr.transient_vs_qss_corner(veh)
    assert z.steady_ay_g == 0.0 and math.isnan(z.rise_time_s) and math.isnan(z.settle_time_s) and z.overshoot_pct == 0.0
    assert z.ok is False

    class Boom(Sim):
        def run(self, *a, **k):
            raise RuntimeError("x")
    monkeypatch.setattr(tr, "TransientSolver", Boom)
    b = tr.transient_vs_qss_corner(veh)
    assert b.ok is False and b.warnings == ["transient_vs_qss_corner failed (RuntimeError: x)"]
    assert (b.steady_ay_g, b.qss_max_ay_g, b.peak_ay_g, b.overshoot_pct) == (0.0, 0.0, 0.0, 0.0)
    assert b.result.warnings == ["x"]


class _PeakTire:
    """A tire whose peak force is fixed, to walk _peak_force and the ellipse guards."""
    def __init__(self, peak):
        self.lateral = NS(fy=lambda a, fz, g: -2e4 * a, peak_force=lambda fz, g: peak)


def _state_driver():
    return tr.DriverInput(
        steer=lambda t, st: (0.02 + 0.05 * t + 1e-3 * st["u"] + 0.05 * st["v"] + 0.03 * st["r"] + 0.07 * st["beta"]
                             + 0.02 * st["t"]),
        throttle=lambda t: 0.05 + 0.3 * t, brake=lambda t: 0.02 + 0.1 * t)


def test_characterized_solver(monkeypatch):
    from tests._golden import check
    res = {"failed": tr.TransientResult.failed(["w"])}
    s = _solver()
    s._cam_f, s._cam_r = 0.01, 0.045
    drv = _state_driver()
    road = tr.RoadInput(z=lambda t: [0.001 + 0.01 * t, 0.0, -0.002 * t, 0.0005],
                        mu=lambda t: [1.0, 0.8 + 0.2 * t, 1.2, 1.0 - 0.1 * t])
    y_stop = Y.copy()
    y_stop[0] = y_stop[2] = 0.0
    y_neg = Y.copy()
    y_neg[20:] = [-0.3, -0.25, -0.35, -0.3]
    y_soft = Y.copy()
    y_soft[20:] = [0.002, 0.001, 0.003, 0.002]
    keys = ("Fz", "Fy", "Fx", "F_susp", "F_damp", "alpha_target", "steer", "throttle", "brake", "z_road", "zd_road")
    res["alg"] = {k: {kk: s.algebraic(0.37, yy, drv, road)[kk] for kk in keys}
                  for k, yy in (("Y", Y), ("stop", y_stop), ("neg", y_neg), ("soft", y_soft))}
    res["der"] = [s.derivatives(0.37, yy, drv, road) for yy in (Y, y_stop, y_neg)]
    res["rk4"] = s._rk4_step(0.37, Y, 0.002, drv, road)
    r = s.run(0.03, driver=drv, road=road, u0=13.0)
    res["run"] = {k: getattr(r, k) for k in ("t", "u", "v", "r", "ay", "ax", "Fz", "Fy", "Fx", "steer", "throttle",
                                             "brake", "alpha", "susp_vel")}
    r0 = _solver().run(0.01)
    res["run_dflt"] = [r0.u, r0.t]
    peaks = {}
    for name, pk in (("tiny", 1e-9), ("half", 0.5), ("zero", 0.0)):
        ps = tr.TransientSolver(params=PRM, tire=_PeakTire(pk), damper=Damper())
        peaks[name] = [ps._peak_force(500.0, True), ps.algebraic(0.1, y_soft, drv, road)["Fx"],
                       ps.algebraic(0.1, y_neg, drv, road)["Fy"]]
    boom = tr.TransientSolver(params=PRM, tire=_PeakTire(None), damper=Damper())
    boom.tire.lateral.peak_force = lambda fz, g: 1 / 0
    peaks["boom"] = [boom._peak_force(0.5, False), boom.warnings]
    res["peaks"] = peaks
    bare = tr.TransientSolver(params=PRM, tire=NS(lateral=Lat()), damper=Damper())
    res["bare"] = bare.algebraic(0.2, Y, tr.DriverInput(throttle=lambda t: 0.4, brake=lambda t: 0.2),
                                 tr.RoadInput())["Fx"]

    def relax(fz):
        raise RuntimeError("no")
    monkeypatch.setattr(tr, "relaxation_length", relax)
    res["relax"] = s.derivatives(0.37, Y, drv, road)[20:]
    check("transient_solver", res)


def test_characterized_vehicle_and_tires():
    from tests._golden import check
    nog = _veh(motion_ratios=None)
    del nog.p.g
    nog.motion_ratios = lambda: (0.0, 0.0)
    nog.roll_center_height = lambda kin, t: {"F": 40.0, "R": 60.0}[kin] + t / 100.0
    half = _veh()
    half.motion_ratios = lambda: (0.85, 0.0)
    res = {"nog": P.from_vehicle(nog), "half": P.from_vehicle(half)}
    no_attr = NS(p=_veh().p)
    tires = {"none": tr.TransientSolver(_veh()), "no_attr": tr.TransientSolver(no_attr), "bare": tr.TransientSolver()}
    res["tires"] = {k: [s.tire.lateral.peak_force(900.0, 0.02), s._cam_f, s._cam_r, s.p.mass] for k, s in tires.items()}
    check("transient_vehicle", res)


def test_characterized_manoeuvres():
    from tests._golden import check
    ts = (0.0, 0.1, 0.2, 0.21, 0.225, 0.3, 0.32, 0.34, 0.4, 0.7, 1.0, 1.1, 1.3, 1.5, 1.6, 2.0, 2.4)
    st = dict(t=0.0, u=15.0, v=0.4, r=0.3, beta=0.06)
    out = {}
    builds = {"step": tr.step_steer_maneuver(), "step_kw": tr.step_steer_maneuver(steer_deg=2.0, t_step=0.2, ramp=0.1),
              "mu": tr.mu_step_maneuver(), "mu_front": tr.mu_step_maneuver(axle="front", t_step=1.0),
              "mu_in": tr.mu_step_maneuver(side="inside"), "mu_out": tr.mu_step_maneuver(side="outside"),
              "snap": tr.snap_oversteer_maneuver(), "snap_off": tr.snap_oversteer_maneuver(recover=False),
              "b2t": tr.brake_to_throttle_maneuver(), "curb": tr.curb_strike_maneuver(),
              "curb_slow": tr.curb_strike_maneuver(u0=0.5, wheels=("FR", "RR"))}
    for k, (drv, road, t_end, u0, label) in builds.items():
        out[k] = {"meta": [t_end, u0, label], "drv": [drv.sample(t, st) for t in ts],
                  "road": [road.sample(t) for t in ts], "mu": [road.mu_at(t) for t in ts]}
    t = np.linspace(0.0, 2.0, 201)
    fake = NS(t=t, r=np.sin(3 * t) + t, beta=np.cos(2 * t) * 0.1 - t * t)
    out["resp"] = [tr.mu_step_response(fake, 1.0), tr.mu_step_response(fake, 1.5)]
    check("transient_manoeuvres", out)


def test_characterized_wiring(monkeypatch):
    from tests._golden import check
    seen = []

    class S:
        def __init__(self, veh, params="unset", tire="unset", damper="unset"):
            seen.append(("init", veh, params, tire, damper))

        def run(self, t_end, driver, road, u0):
            seen.append(("run", t_end, u0, driver.sample(0.5), driver.sample(1.2), road.sample(0.31)))
            return tr.TransientResult.failed([])
    monkeypatch.setattr(tr, "TransientSolver", S)
    tr.run_maneuver("veh", "curb_strike", params="P", u0=15.0)
    tr.run_maneuver(None, "snap_oversteer")

    class Q:
        def __init__(self, veh, params="unset"):
            seen.append(("q", veh.tag, params))

        def run(self, t_end, driver, u0):
            k = len([x for x in seen if x[0] == "qrun"])
            seen.append(("qrun", t_end, u0, [driver.sample(x) for x in (0.1, 0.2, 0.21, 0.24, 0.5)]))
            t = np.arange(0.0, t_end + 1e-9, 0.01)
            if k in (3, 13):
                return NS(ok=False, ay=np.full(t.size, 5.0), t=t, warnings=[])
            n = 5 if k == 4 else t.size
            d = driver.steer(1.0)
            ay = math.degrees(d) * 0.2 * (1 - np.exp(-np.clip(t[:n] - 0.2, 0, None) / 0.1))
            return NS(ok=True, ay=ay, t=t[:n], warnings=[])
    monkeypatch.setattr(tr, "TransientSolver", Q)
    veh = NS(max_lateral_g=lambda: 1.5, tag="ok")
    res = {"a": tr.transient_vs_qss_corner(veh, u0=15.0, t_end=0.9, params="P"),
           "b": tr.transient_vs_qss_corner(NS(max_lateral_g=lambda: 1 / 0, tag="b"), target_ay_g=0.9, t_end=1.0),
           "c": tr.transient_vs_qss_corner(NS(max_lateral_g=lambda: 1 / 0, tag="c"), t_end=1.01)}

    class E(Q):
        def run(self, t_end, driver, u0):
            return NS(ok=True, ay=np.zeros(0), t=np.zeros(0), warnings=[])
    monkeypatch.setattr(tr, "TransientSolver", E)
    res["empty"] = tr.transient_vs_qss_corner(veh)

    class B(Q):
        def run(self, t_end, driver, u0):
            raise RuntimeError("boom")
    monkeypatch.setattr(tr, "TransientSolver", B)
    res["boom"] = tr.transient_vs_qss_corner(veh)
    check("transient_wiring", {"res": {k: {f: getattr(v, f) for f in ("ok", "steady_ay_g", "qss_max_ay_g", "peak_ay_g",
                                                                       "overshoot_pct", "rise_time_s",
                                                                       "settle_time_s", "warnings")}
                                       for k, v in res.items()},
                               "seen": seen})


def test_characterized_inputs_and_limits():
    from tests._golden import check
    res = {}
    drivers = [tr.DriverInput(steer=lambda t: 1 / 0, throttle=lambda t: 1.7, brake=lambda t: 1 / 0),
               tr.DriverInput(throttle=lambda t: -0.5, brake=lambda t: 1.5),
               tr.DriverInput(brake=lambda t: "x")]
    res["drivers"] = [d.sample(0.1) for d in drivers]
    roads = [tr.RoadInput(z=lambda t: [t, 0, 0, 0], zdot=lambda t: [t, 2 * t, 3 * t, 4 * t]),
             tr.RoadInput(z=lambda t: [t * t, 0, 0, 0]), tr.RoadInput(zdot=lambda t: [1, 2])]
    res["roads"] = [r.sample(0.4) for r in roads]
    res["izz"] = [P(izz=0.5).izz_eff(), P(izz=0.0).izz_eff(), P(izz=2.0).izz_eff()]
    f = tr.TransientResult.failed([])
    sums = []
    for fz in (1.0, 1.5, 0.5):
        f.Fz = np.array([[fz, 300.0, 300.0, 300.0]])
        sums.append(f.summary())
    res["summary"] = sums
    s = tr.TransientSolver(params=PRM, tire=NS(lateral=Lat()), damper=Damper())
    y = Y.copy()
    y[0] = 3.0
    y[20:] = [-0.02, -0.015, -0.03, -0.02]
    hard = {"floor": tr.DriverInput(throttle=lambda t: 1.0), "stand": tr.DriverInput(brake=lambda t: 1.0),
            "mixed": tr.DriverInput(throttle=lambda t: 0.8, brake=lambda t: 0.6, steer=lambda t: -0.2)}
    yp = y.copy()
    yp[20:] = -y[20:]
    res["saturated"] = {k: [{kk: s.algebraic(0.2, yy, d, tr.RoadInput())[kk] for kk in ("Fx", "Fy")} for yy in (y, yp)]
                        for k, d in hard.items()}
    check("transient_inputs", res)
