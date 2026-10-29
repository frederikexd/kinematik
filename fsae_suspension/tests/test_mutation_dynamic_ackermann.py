"""suspension/dynamic_ackermann.py: geometric Ackermann from cot(di) = cot(do) - t/L, a tire whose peak slip is known
in closed form (fy = Fz sin(2 atan((alpha + gamma/2) / alpha_p)) peaks at alpha = alpha_p - gamma/2), the axle force
at a fixed split against a brute-force search, and the linkage split and lock read from the solved corner."""
import math

import numpy as np
import pytest

from suspension import dynamic_ackermann as da
from suspension.kinematics import Hardpoints, SuspensionKinematics

HP = Hardpoints.default()


class Tire:
    def __init__(self, scale=1.0):
        self.scale = scale

    @staticmethod
    def a_peak(fz):
        return 0.08 + 2e-5 * fz                                           # rad: peak slip rises with load

    def fy(self, alpha, fz, gamma=0.0):
        a = np.asarray(alpha, float)
        return -self.scale * fz * np.sin(2.0 * np.arctan((a + 0.5 * gamma) / self.a_peak(fz)))


def test_geometric_ackermann():
    do = 20.0
    cot_i = 1.0 / math.tan(math.radians(do)) - 1210.0 / 1630.0
    assert da.geometric_split(do, 1630.0, 1210.0) == pytest.approx(math.degrees(math.atan(1.0 / cot_i)))
    assert da.geometric_split(-do, 1630.0, 1210.0) == da.geometric_split(do, 1630.0, 1210.0)
    assert da.geometric_split(0.0, 1630.0, 1210.0) == 0.0
    assert da.geometric_split(1e-6, 1630.0, 1210.0) > 0.0


def test_ackermann_percentage():
    gi = da.geometric_split(20.0, 1630.0, 1210.0)
    assert da.ackermann_pct(gi, 20.0, 1630.0, 1210.0) == pytest.approx(100.0)
    assert da.ackermann_pct(-gi, -20.0, 1630.0, 1210.0) == pytest.approx(100.0)
    assert da.ackermann_pct(20.0, 20.0, 1630.0, 1210.0) == 0.0
    assert da.ackermann_pct(20.0 + 0.5 * (gi - 20.0), 20.0, 1630.0, 1210.0) == pytest.approx(50.0)
    assert math.isnan(da.ackermann_pct(1.0, 0.0, 1630.0, 1210.0))


def test_peak_slip_is_found_to_a_hundredth_of_a_degree():
    t = Tire()
    for fz, gamma in ((1500.0, 0.0), (500.0, 0.0), (1500.0, -2.0)):
        want = math.degrees(Tire.a_peak(fz) - 0.5 * math.radians(gamma))
        assert da.peak_slip_deg(t, fz, gamma) == pytest.approx(want, abs=0.01)
    assert da.peak_slip_deg(t, 1500.0, hi=5.0) == pytest.approx(5.0, abs=1e-9)        # peak past the window
    assert da.peak_slip_deg(t, 1500.0, lo=9.0) == pytest.approx(9.0, abs=1e-9)
    assert da.peak_slip_deg(t, 1500.0, lo=0.5, hi=6.0) == pytest.approx(6.0, abs=1e-9)


def test_the_axle_case_and_its_loads_from_the_vehicle():
    c = da.AxleCase(1600.0, 400.0)
    assert (c.radius_mm, c.wheelbase_mm, c.kingpin_track_mm, c.camber_outer_deg, c.camber_inner_deg) == \
        (4500.0, 1630.0, 1210.0, 0.0, 0.0)
    v = da.AxleCase.from_vehicle()
    stat = 300.0 * 9.81 * 0.48 / 2
    dfz = 0.53 * 300.0 * 9.81 * 1.5 * 280.0 / 1210.0
    assert (v.fz_outer_n, v.fz_inner_n, v.kingpin_track_mm) == pytest.approx((stat + dfz, stat - dfz, 1210.0))
    v = da.AxleCase.from_vehicle(mass_kg=250.0, weight_front=0.45, cg_height_mm=300.0, track_mm=1250.0,
                                 front_share=0.5, a_lat_g=1.2, radius_mm=8000.0)
    stat = 250.0 * 9.81 * 0.45 / 2
    dfz = 0.5 * 250.0 * 9.81 * 1.2 * 300.0 / 1250.0
    assert (v.fz_outer_n, v.fz_inner_n, v.kingpin_track_mm, v.radius_mm) == pytest.approx(
        (stat + dfz, stat - dfz, 1250.0, 8000.0))
    assert da.AxleCase.from_vehicle(a_lat_g=10.0).fz_inner_n == 1.0               # a lifted wheel floors at 1 N


def test_the_dynamic_target():
    case = da.AxleCase(1600.0, 400.0, camber_outer_deg=-1.0, camber_inner_deg=0.5)
    r = da.dynamic_target(Tire(), case)
    ao, ai = da.peak_slip_deg(Tire(), 1600.0, -1.0), da.peak_slip_deg(Tire(), 400.0, 0.5)
    do = math.degrees(math.atan(1630.0 / (4500.0 + 605.0)))
    di = math.degrees(math.atan(1630.0 / (4500.0 - 605.0)))
    assert r == pytest.approx({"peak_slip_outer_deg": ao, "peak_slip_inner_deg": ai, "geometric_outer_deg": do,
                               "geometric_inner_deg": di, "geometric_difference_deg": di - do,
                               "dynamic_difference_deg": di - do + ai - ao,
                               "dynamic_ackermann_pct": 100.0 * (di - do + ai - ao) / (di - do)})
    assert r["dynamic_ackermann_pct"] < 100.0                  # the loaded outer tire wants more slip


def _brute(case, split, n=400):
    t = da.dynamic_target(Tire(), case)
    d = split - t["geometric_difference_deg"]
    ao = np.linspace(0.5, 20.0, n)
    ai = ao + d
    ok = ai > 0
    F = np.abs(Tire().fy(np.radians(ao[ok]), case.fz_outer_n, math.radians(case.camber_outer_deg))) + \
        np.abs(Tire().fy(np.radians(ai[ok]), case.fz_inner_n, math.radians(case.camber_inner_deg)))
    return F.max()


def test_the_axle_force_at_a_fixed_split():
    case = da.AxleCase(1600.0, 400.0, camber_outer_deg=-1.0, camber_inner_deg=0.5)
    t = da.dynamic_target(Tire(), case)
    opt = abs(Tire().fy(math.radians(t["peak_slip_outer_deg"]), 1600.0, math.radians(-1.0))) + \
        abs(Tire().fy(math.radians(t["peak_slip_inner_deg"]), 400.0, math.radians(0.5)))
    best = da.axle_force_at_split(Tire(), case, t["dynamic_difference_deg"])
    assert best["optimum_n"] == pytest.approx(opt) and best["loss_pct"] < 0.05
    geo = da.axle_force_at_split(Tire(), case, t["geometric_difference_deg"])
    assert geo["axle_force_n"] == pytest.approx(_brute(case, t["geometric_difference_deg"]))
    assert geo["loss_pct"] == pytest.approx(100.0 * (1 - geo["axle_force_n"] / opt)) and geo["loss_pct"] > 0.1
    neg = da.axle_force_at_split(Tire(), case, t["geometric_difference_deg"] - 5.0)    # inner slips below zero cut
    assert neg["axle_force_n"] == pytest.approx(_brute(case, t["geometric_difference_deg"] - 5.0))
    coarse = da.axle_force_at_split(Tire(), case, t["geometric_difference_deg"], n=20)
    assert coarse["axle_force_n"] == pytest.approx(_brute(case, t["geometric_difference_deg"], n=20))
    assert math.isnan(da.axle_force_at_split(Tire(scale=0.0), case, 1.0)["loss_pct"])


def _toe(rack):
    return SuspensionKinematics(HP, pickup_deltas={"tie_rod_inner": np.array([0.0, rack, 0.0])}
                                ).solve_at_travel(0.0).toe


def test_the_linkage_split_reads_both_wheels_of_the_mirrored_axle():
    t0 = _toe(0.0)
    for rack in (12.0, -12.0):
        s = da.kinematic_split(HP, rack)
        dR, dL = _toe(rack) - t0, -(_toe(-rack) - t0)
        inner, outer = (dR, dL) if dR + dL >= 0 else (-dL, -dR)
        assert s == pytest.approx({"inner_deg": inner, "outer_deg": outer, "difference_deg": inner - outer})
        assert s["inner_deg"] > 0 and s["outer_deg"] > 0
    assert da.kinematic_split(HP, 0.0) == {"inner_deg": 0.0, "outer_deg": 0.0, "difference_deg": 0.0}


def test_the_lock_report():
    with pytest.raises(ValueError) as e:
        da.lock_report(HP, 90.0, 20.0, lock_basis="outer")
    assert str(e.value) == "lock_basis must be 'inner' or 'mean'."
    assert da.lock_report(HP, 90.0, 89.0) == {"reachable": False}
    assert da.lock_report(HP, 90.0, 20.0, rack_max_mm=5.0) == {"reachable": False}
    r = da.lock_report(HP, 90.0, 20.0)
    x = r["rack_at_lock_mm"]
    s = da.kinematic_split(HP, x)
    assert s["inner_deg"] == pytest.approx(20.0, abs=1e-6) and 0.0 < x < 60.0
    assert r["wheel_lock_deg"] == pytest.approx(360.0 * x / 90.0)
    assert (r["inner_deg"], r["outer_deg"], r["mean_deg"]) == pytest.approx(
        (s["inner_deg"], s["outer_deg"], 0.5 * (s["inner_deg"] + s["outer_deg"])))
    lo = da.kinematic_split(HP, 5.0)
    assert r["ackermann_pct_lo_and_lock"] == pytest.approx(
        [da.ackermann_pct(lo["inner_deg"], lo["outer_deg"], 1630.0, 1210.0),
         da.ackermann_pct(s["inner_deg"], s["outer_deg"], 1630.0, 1210.0)])
    sgn = 1.0 if _toe(1.0) > _toe(0.0) else -1.0
    st = lambda rack: SuspensionKinematics(HP, pickup_deltas={"tie_rod_inner": np.array([0.0, rack, 0.0])}
                                           ).solve_at_travel(0.0).camber
    assert r["camber_at_lock_outside_deg"] == pytest.approx(st(-sgn * x))
    assert r["camber_at_lock_inside_deg"] == pytest.approx(st(sgn * x))
    assert r["reachable"] is True
    m = da.lock_report(HP, 90.0, 20.0, wheelbase_mm=1550.0, kingpin_track_mm=1150.0, rack_lo_mm=8.0,
                       rack_max_mm=50.0, lock_basis="mean")
    sm = da.kinematic_split(HP, m["rack_at_lock_mm"])
    assert 0.5 * (sm["inner_deg"] + sm["outer_deg"]) == pytest.approx(20.0, abs=1e-6)
    lo8 = da.kinematic_split(HP, 8.0)
    assert m["ackermann_pct_lo_and_lock"][0] == pytest.approx(
        da.ackermann_pct(lo8["inner_deg"], lo8["outer_deg"], 1550.0, 1150.0))


def test_the_outside_camber_on_a_trailing_tie_rod():
    hp = HP.copy()
    for n in ("tie_rod_inner", "tie_rod_outer"):
        p = np.asarray(getattr(hp, n), float).copy(); p[0] = -p[0]; setattr(hp, n, p)
    r = da.lock_report(hp, 90.0, 15.0)
    x = r["rack_at_lock_mm"]
    t = lambda rack: SuspensionKinematics(hp, pickup_deltas={"tie_rod_inner": np.array([0.0, rack, 0.0])}
                                          ).solve_at_travel(0.0)
    assert t(1.0).toe > t(0.0).toe                                         # this geometry steers the other way
    assert r["camber_at_lock_outside_deg"] == pytest.approx(t(-x).camber)
    assert r["camber_at_lock_inside_deg"] == pytest.approx(t(x).camber)
    assert 0.0 < x < da.RACK_MAX_MM


def test_a_split_that_would_need_negative_inner_slip():
    from suspension.tiremodel import default_tire
    tire = default_tire()
    case = da.AxleCase(1400.0, 700.0)
    t = da.dynamic_target(tire, case)
    r = da.axle_force_at_split(tire, case, t["geometric_difference_deg"] - 15.0)
    ao = np.linspace(0.5, 20.0, 400)
    ai = ao - 15.0
    ok = ai > 0.0
    F = np.abs(tire.fy(np.radians(ao[ok]), 1400.0, 0.0)) + np.abs(tire.fy(np.radians(ai[ok]), 700.0, 0.0))
    assert r["axle_force_n"] == pytest.approx(float(F.max()))
    assert r["loss_pct"] == pytest.approx(100 * (1 - F.max() / r["optimum_n"]))


def test_a_sub_newton_axle_still_reports_its_loss():
    from suspension.tiremodel import default_tire
    tire = default_tire()
    case = da.AxleCase(0.3, 0.2)
    r = da.axle_force_at_split(tire, case, 1.0)
    assert 0 < r["optimum_n"] < 1.0 and r["loss_pct"] == pytest.approx(100 * (1 - r["axle_force_n"] / r["optimum_n"]))
