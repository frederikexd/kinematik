"""suspension/dynamics.py against first principles: the roll centre as the line from the contact patch through the
instant centre, met at the centreline; the spring-to-roll-rate map k_spring MR^2 t^2 / 2; total lateral transfer
summing to m a h_cg whatever the split; the steady-state limit set by the first axle to saturate. Stand-in corners
(any object with a static pose, a motion ratio and anti functions) keep each check to its own arithmetic."""
import math
from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest

from suspension import dynamics as dy
from suspension.tiremodel import default_tire


class Corner:
    def __init__(self, ic=(-1500.0, 150.0), cp=(0.0, 600.0, 0.0), camber=-2.0, mr=0.8):
        self.static = NS(instant_center=np.array(ic), contact_patch=np.array(cp), camber=camber)
        self.mr, self.calls, self.hp = mr, 0, object()

    def motion_ratio(self):
        self.calls += 1
        return self.mr

    def solve_at_travel(self, t):
        return NS(instant_center=np.array([-1500.0, 150.0 + 2 * t]), contact_patch=np.array([0.0, 600.0, t]))

    def anti_dive_pct(self, h, wb, brake_bias_front):
        return ("dive", h, wb, brake_bias_front)

    def anti_squat_pct(self, h, wb, drive_bias_rear):
        return ("squat", h, wb, drive_bias_rear)


def _rc(ic, cp):
    return cp[2] + (ic[1] - cp[2]) / (ic[0] - cp[1]) * (0.0 - cp[1])


P = dy.VehicleParams(mass=300.0, cg_height=320.0, wheelbase=1600.0, track_front=1250.0, track_rear=1150.0,
                     weight_dist_front=0.44, roll_stiffness_front=420.0, roll_stiffness_rear=260.0)


def test_the_parameter_defaults_and_corner_loads():
    p = dy.VehicleParams()
    assert (p.mass, p.cg_height, p.wheelbase, p.track_front, p.track_rear, p.weight_dist_front, p.roll_stiffness_front,
            p.roll_stiffness_rear, p.spring_rate_front, p.spring_rate_rear, p.arb_rate_front, p.arb_rate_rear,
            p.use_spring_rates, p.tire_load_sens, p.mu_peak, p.static_camber_front, p.static_camber_rear,
            p.use_param_camber, p.g) == (280.0, 300.0, 1550.0, 1200.0, 1180.0, 0.47, 350.0, 300.0, 35.0, 35.0, 0.0,
                                         0.0, False, 0.00018, 1.55, -1.5, -1.5, False, 9.81)
    assert dy.CornerLoads(1.0, 2.0, 3.0, 4.0).as_tuple() == (1.0, 2.0, 3.0, 4.0)


def test_the_camber_each_axle_runs():
    p = replace(P, static_camber_front=-2.5, static_camber_rear=-1.0)
    v = dy.VehicleDynamics(p, front_kin=Corner(camber=-3.0), rear_kin=None)
    assert v._axle_camber_rad("front") == pytest.approx(math.radians(3.0))           # the solved geometry
    assert v._axle_camber_rad("rear") == pytest.approx(math.radians(1.0))            # no geometry: the parameter
    w = dy.VehicleDynamics(replace(p, use_param_camber=True), front_kin=Corner(camber=-3.0), rear_kin=Corner())
    assert w._axle_camber_rad("front") == pytest.approx(math.radians(2.5))           # the override wins
    assert w._axle_camber_rad("rear") == pytest.approx(math.radians(1.0))
    assert dy.VehicleDynamics(p, rear_kin=Corner(camber=1.2))._axle_camber_rad("rear") == pytest.approx(
        math.radians(1.2))


def test_roll_stiffness_from_the_springs_through_the_motion_ratio():
    p = replace(P, spring_rate_front=40.0, spring_rate_rear=30.0, arb_rate_front=120.0, arb_rate_rear=55.0,
                use_spring_rates=True)
    f, r = Corner(mr=0.8), Corner(mr=0.65)
    v = dy.VehicleDynamics(p, front_kin=f, rear_kin=r)
    want_f = 40.0 * 0.64 * 1250.0 ** 2 / 2 / 1000 * math.pi / 180 + 120.0
    want_r = 30.0 * 0.65 ** 2 * 1150.0 ** 2 / 2 / 1000 * math.pi / 180 + 55.0
    assert v._axle_roll_stiffness("front") == pytest.approx(want_f, rel=1e-12)
    assert v._axle_roll_stiffness("rear") == pytest.approx(want_r, rel=1e-12)
    assert v._axle_roll_stiffness("front") == pytest.approx(want_f) and f.calls == 1     # solved once, then cached
    assert dy.VehicleDynamics(replace(p, use_spring_rates=False), f, r)._axle_roll_stiffness("rear") == 260.0
    assert dy.VehicleDynamics(p, front_kin=None, rear_kin=r)._axle_roll_stiffness("front") == 420.0
    for bad in (float("nan"), 0.0, -0.5):
        assert dy.VehicleDynamics(p, Corner(mr=bad), Corner(mr=bad))._axle_roll_stiffness("front") == 420.0
        assert dy.VehicleDynamics(p, Corner(mr=bad), Corner(mr=bad))._axle_roll_stiffness("rear") == 260.0
    tiny = dy.VehicleDynamics(p, Corner(mr=1e-6), r)._axle_roll_stiffness("front")
    assert tiny == pytest.approx(40.0 * 1e-12 * 1250.0 ** 2 / 2 / 1000 * math.pi / 180 + 120.0)


def test_the_roll_centre_and_its_migration():
    v = dy.VehicleDynamics(P)
    c = Corner(ic=(-1500.0, 150.0), cp=(0.0, 600.0, 4.0))
    assert v.roll_center_height(c, 1250.0) == pytest.approx(_rc((-1500.0, 150.0), (0.0, 600.0, 4.0)), rel=1e-12)
    st = NS(instant_center=np.array([2400.0, -80.0]), contact_patch=np.array([0.0, 580.0, -2.0]))
    assert v.roll_center_height(c, 1250.0, state=st) == pytest.approx(_rc((2400.0, -80.0), (0.0, 580.0, -2.0)))
    assert math.isnan(v.roll_center_height(Corner(ic=(600.0, 150.0)), 1250.0))       # IC above the patch
    assert math.isnan(v.roll_center_height(Corner(ic=(600.0 + 5e-10, 150.0)), 1250.0))
    assert v.roll_center_height(Corner(ic=(600.0 + 2e-9, 150.0)), 1250.0) != 0.0
    assert math.isnan(v.roll_center_height(Corner(ic=(np.inf, 150.0)), 1250.0))
    assert math.isnan(v.roll_center_height(Corner(ic=(-1500.0, np.nan)), 1250.0))
    t, rc = v.roll_center_migration(c, 1250.0)
    assert t == pytest.approx(list(np.linspace(-30, 30, 21)))
    assert rc == pytest.approx([_rc((-1500.0, 150.0 + 2 * x), (0.0, 600.0, x)) for x in t])
    t, rc = v.roll_center_migration(c, 1250.0, travel_min=-10.0, travel_max=20.0, n=4)
    assert t == pytest.approx([-10.0, 0.0, 10.0, 20.0]) and len(rc) == 4


def test_longitudinal_transfer_and_the_anti_features():
    v = dy.VehicleDynamics(P, front_kin=Corner(), rear_kin=Corner(mr=0.7))
    dW, info = v.longitudinal_load_transfer(0.9)
    assert dW == pytest.approx(300.0 * 0.9 * 9.81 * 0.32 / 1.6) and info == {"dW_axle": dW}
    assert v.anti_dive_pct(0.6) == ("dive", 320.0, 1600.0, 0.6) and v.anti_dive_pct()[3] == 0.65
    assert v.anti_squat_pct(0.8) == ("squat", 320.0, 1600.0, 0.8) and v.anti_squat_pct()[3] == 1.0
    assert v.motion_ratios() == (0.8, 0.7)
    bare = dy.VehicleDynamics(P)
    assert math.isnan(bare.anti_dive_pct()) and math.isnan(bare.anti_squat_pct())
    assert all(math.isnan(x) for x in bare.motion_ratios())


def _transfer_check(v, p, g, rc_f, rc_r):
    loads, info = v.lateral_load_transfer(g)
    a = g * p.g
    ks_f, ks_r = info["roll_stiffness_front"], info["roll_stiffness_rear"]
    axis = rc_f + (rc_r - rc_f) * (1 - p.weight_dist_front)          # the roll axis at the CG station
    M = p.mass * a * (p.cg_height - axis) / 1000
    assert (info["rc_front"], info["rc_rear"]) == pytest.approx((rc_f, rc_r))
    assert info["roll_angle"] == pytest.approx(M / (ks_f + ks_r))
    tf, tr = p.track_front / 1000, p.track_rear / 1000
    assert info["ltd_front"] * tf + info["ltd_rear"] * tr == pytest.approx(p.mass * a * p.cg_height / 1000)
    geo_f = p.mass * p.weight_dist_front * a * rc_f / 1000
    geo_r = p.mass * (1 - p.weight_dist_front) * a * rc_r / 1000
    assert (info["ltd_front"] * tf - geo_f) / (info["ltd_rear"] * tr - geo_r) == pytest.approx(ks_f / ks_r)
    W = p.mass * p.g
    Wf, Wr = W * p.weight_dist_front, W * (1 - p.weight_dist_front)
    assert loads.as_tuple() == pytest.approx((Wf / 2 - info["ltd_front"], Wf / 2 + info["ltd_front"],
                                              Wr / 2 - info["ltd_rear"], Wr / 2 + info["ltd_rear"]))
    return loads, info


def test_lateral_transfer_sums_to_m_a_h_and_splits_by_roll_stiffness():
    f, r = Corner(ic=(-1500.0, 150.0)), Corner(ic=(-900.0, 200.0), cp=(0.0, 575.0, 0.0))
    v = dy.VehicleDynamics(P, front_kin=f, rear_kin=r)
    _transfer_check(v, P, 1.2, _rc(f.static.instant_center, f.static.contact_patch),
                    _rc(r.static.instant_center, r.static.contact_patch))
    _transfer_check(dy.VehicleDynamics(P), P, 0.8, 50.0, 60.0)                        # no geometry: the defaults
    _transfer_check(dy.VehicleDynamics(P, Corner(ic=(600.0, 1.0)), Corner(ic=(600.0, 1.0))), P, 0.8, 50.0, 60.0)
    sp = replace(P, use_spring_rates=True, spring_rate_front=50.0, arb_rate_rear=80.0)
    _transfer_check(dy.VehicleDynamics(sp, front_kin=f, rear_kin=r), sp, 1.0,
                    _rc(f.static.instant_center, f.static.contact_patch),
                    _rc(r.static.instant_center, r.static.contact_patch))


def test_no_roll_stiffness_and_a_wheel_that_lifts():
    p = replace(P, roll_stiffness_front=0.0, roll_stiffness_rear=0.0)
    loads, info = dy.VehicleDynamics(p).lateral_load_transfer(1.0)
    a = 9.81
    assert info["roll_angle"] == 0.0
    assert info["ltd_front"] == pytest.approx(300 * 0.44 * a * 0.05 / 1.25)          # the geometric part alone
    assert info["ltd_rear"] == pytest.approx(300 * 0.56 * a * 0.06 / 1.15)
    lifted, info = dy.VehicleDynamics(P).lateral_load_transfer(4.0)
    assert lifted.fl == 0.0 and lifted.rl == 0.0 and lifted.fr > 0 and lifted.rr > 0
    assert lifted.fr == pytest.approx(300 * 9.81 * 0.44 / 2 + info["ltd_front"])


def test_the_corner_force_and_axle_grip():
    v = dy.VehicleDynamics(P)
    assert v._corner_force(0.0, "front") == 0.0 and v._corner_force(-10.0, "rear") == 0.0
    assert v._corner_force(1000.0, "front") == pytest.approx((1.55 - 0.00018 * 1000.0) * 1000.0)
    assert v._corner_force(7000.0, "front") == pytest.approx(0.3 * 7000.0)               # the friction floor
    assert v._corner_force(6944.0, "rear") == pytest.approx((1.55 - 0.00018 * 6944.0) * 6944.0)
    assert v.grip_model_name() == "linear placeholder"
    seen = []

    class Tire:
        def peak_force(self, fz, g):
            seen.append((fz, g))
            return 2.0 * fz + g
    t = dy.VehicleDynamics(replace(P, static_camber_front=-2.0, static_camber_rear=-1.0), tire=Tire())
    assert t._corner_force(900.0, "front") == pytest.approx(1800.0 + math.radians(2.0))
    assert t._corner_force(800.0, "rear") == pytest.approx(1600.0 + math.radians(1.0))
    assert seen == [(900.0, pytest.approx(math.radians(2.0))), (800.0, pytest.approx(math.radians(1.0)))]
    assert t.grip_model_name() == "Pacejka MF5.2"
    L = dy.CornerLoads(500.0, 1100.0, 600.0, 1300.0)
    assert v.axle_grip(L) == pytest.approx((v._corner_force(500.0, "front") + v._corner_force(1100.0, "front"),
                                            v._corner_force(600.0, "rear") + v._corner_force(1300.0, "rear")))
    assert t.axle_grip(L) == pytest.approx((3200.0 + 2 * math.radians(2.0), 3800.0 + 2 * math.radians(1.0)))


def test_the_balance_index():
    v = dy.VehicleDynamics(P, tire=default_tire())
    b, uf, ur = v.balance_index(1.1)
    loads, _ = v.lateral_load_transfer(1.1)
    Ff, Fr = v.axle_grip(loads)
    W = 300 * 9.81
    assert (uf, ur) == pytest.approx((W * 0.44 * 1.1 / Ff, W * 0.56 * 1.1 / Fr)) and b == pytest.approx(uf - ur)

    class Dead:
        def peak_force(self, fz, g):
            return 0.0
    assert dy.VehicleDynamics(P, tire=Dead()).balance_index(1.0) == (0, 99, 99)


def test_the_limit_is_where_the_first_axle_saturates():
    for p in (P, replace(P, weight_dist_front=0.55, roll_stiffness_rear=600.0)):
        v = dy.VehicleDynamics(p, tire=default_tire())
        g = v.max_lateral_g()

        def cap(a):
            Ff, Fr = v.axle_grip(v.lateral_load_transfer(a)[0])
            return min(Ff / (p.mass * p.g * p.weight_dist_front), Fr / (p.mass * p.g * (1 - p.weight_dist_front)))
        assert 0.1 < g < 3.0 and cap(g) >= g and cap(g + 1e-9) < g + 1e-9

    class Super:
        def peak_force(self, fz, g):
            return 10.0 * fz
    assert dy.VehicleDynamics(P, tire=Super()).max_lateral_g() == pytest.approx(3.0, abs=1e-9)

    class Dead:
        def peak_force(self, fz, g):
            return 0.0
    assert dy.VehicleDynamics(P, tire=Dead()).max_lateral_g() == 0.1
    rear_only = dy.VehicleDynamics(replace(P, weight_dist_front=0.0), tire=default_tire())
    g = rear_only.max_lateral_g()
    Fr = rear_only.axle_grip(rear_only.lateral_load_transfer(g)[0])[1]
    assert Fr / (300 * 9.81) >= g and 0.1 < g < 3.0
    front_only = dy.VehicleDynamics(replace(P, weight_dist_front=1.0), tire=default_tire())
    g = front_only.max_lateral_g()
    assert front_only.axle_grip(front_only.lateral_load_transfer(g)[0])[0] / (300 * 9.81) >= g


def test_the_compliance_solve_is_handed_this_cars_wheel_load(monkeypatch):
    from suspension import compliance as cmp
    calls = []

    class FakeCorner:
        def __init__(self, hp, stiffness):
            self.how = ("explicit", hp, stiffness)

        @classmethod
        def uniform_tube(cls, hp):
            c = cls(hp, None)
            c.how = ("uniform", hp)
            return c

        def solve(self, load, **kw):
            return (self.how, load, kw)
    monkeypatch.setattr(cmp, "CompliantCorner", FakeCorner)
    monkeypatch.setattr(cmp, "corner_wheel_load", lambda veh, axle, g, outer, long_g:
                        calls.append((veh, axle, g, outer, long_g)) or "LOAD")
    f, r = Corner(), Corner()
    v = dy.VehicleDynamics(P, front_kin=f, rear_kin=r)
    assert v.corner_compliance() == (("uniform", f.hp), "LOAD", {})
    assert calls[-1] == (v, "front", 1.5, True, 0.0)
    assert v.corner_compliance(1.2, "rear", False, {"LF": 1}, long_g=0.3, max_iter=7) == (
        ("explicit", r.hp, {"LF": 1}), "LOAD", {"max_iter": 7})
    assert calls[-1] == (v, "rear", 1.2, False, 0.3)
    pre = FakeCorner("hp", "s")
    assert v.corner_compliance(corner=pre, stiffness={"x": 2})[0] == ("explicit", "hp", "s")
    assert dy.VehicleDynamics(P).corner_compliance() is None
    assert dy.VehicleDynamics(P, front_kin=f).corner_compliance(axle="rear") is None


def test_small_values_at_every_guard():
    v = dy.VehicleDynamics(P)
    assert v._corner_force(0.5, "front") == pytest.approx((1.55 - 0.00018 * 0.5) * 0.5)

    class Faint:
        def peak_force(self, fz, g):
            return 0.25
    b, uf, ur = dy.VehicleDynamics(P, tire=Faint()).balance_index(0.001)
    W = 300 * 9.81
    assert (uf, ur) == pytest.approx((W * 0.44 * 0.001 / 0.5, W * 0.56 * 0.001 / 0.5))
    right, info = dy.VehicleDynamics(P).lateral_load_transfer(-4.0)          # a right turn lifts the right wheels
    assert right.fr == 0.0 and right.rr == 0.0 and right.fl > 0 and right.rl > 0
    soft = replace(P, roll_stiffness_front=0.2, roll_stiffness_rear=0.1)
    loads, info = dy.VehicleDynamics(soft).lateral_load_transfer(1.0)
    _transfer_check(dy.VehicleDynamics(soft), soft, 1.0, 50.0, 60.0)
    assert info["roll_angle"] != 0.0
    tiny = replace(P, mass=0.05)
    t = dy.VehicleDynamics(tiny, tire=default_tire())
    g = t.max_lateral_g()
    Ff, Fr = t.axle_grip(t.lateral_load_transfer(g)[0])
    assert min(Ff / (0.05 * 9.81 * 0.44), Fr / (0.05 * 9.81 * 0.56)) >= g and 0.1 < g <= 3.0


def test_the_roll_centre_through_the_vehicle():
    c = Corner(ic=(-1200.0, 140.0), cp=(0.0, 610.0, 3.0))
    v = dy.VehicleDynamics(P, front_kin=c)
    assert v.roll_center_height(c) == pytest.approx(_rc((-1200.0, 140.0), (0.0, 610.0, 3.0)))
    assert v.lateral_load_transfer(1.0)[1]["rc_front"] == pytest.approx(_rc((-1200.0, 140.0), (0.0, 610.0, 3.0)))
