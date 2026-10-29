"""suspension/aero/coupling.py with stand-in maps and cars: the C_L/C_D -> cl_a/cd_a bridge and every fallback,
the attitude estimate, the axle ride rate (two wheels x k x MR^2) and each refusal, and the dynamics attitude worked
by hand: roll from the load-transfer report, pitch from dW over the ride rates cut by anti-dive/squat, and the aero
heave over both axles."""
import math
from types import SimpleNamespace as NS

import pytest

from suspension.aero import coupling as cp
from suspension.aero.cfd import Attitude


class Map:
    def __init__(self, cl=-3.0, cd=1.1, n=4):
        self.cl, self.cd, self.n, self.asked = cl, cd, n, []

    def __len__(self):
        return self.n

    def query(self, att):
        self.asked.append(att)
        return NS(c_lift=self.cl, c_drag=self.cd)

    def status(self):
        return "mapped!"


def test_the_provider_bridge_and_its_fallbacks():
    p = cp.AeroProvider(1.1)
    assert (p.fallback_cl_a, p.fallback_cd_a, p.aero_map) == (2.5, 1.2, None)
    assert p.cla_cda(Attitude()) == (2.5, 1.2) and p.is_mapped() is False
    assert p.status() == "scalar aero (cl_a=2.5, cd_a=1.2); no aero map loaded"
    m = Map()
    q = cp.AeroProvider(1.1, fallback_cl_a=2.0, fallback_cd_a=1.0, aero_map=m)
    att = Attitude(roll_deg=1.0)
    assert q.cla_cda(att) == pytest.approx((3.3, 1.21)) and m.asked == [att]
    assert q.cla_cda(None) == (2.0, 1.0) and q.is_mapped() is True and q.status() == "mapped!"
    assert cp.AeroProvider(1.1, aero_map=Map(n=0)).cla_cda(att) == (2.5, 1.2)
    assert cp.AeroProvider(1.1, aero_map=Map(n=0)).is_mapped() is False
    assert cp.AeroProvider(1.1, aero_map=Map(n=0)).status().startswith("scalar aero")
    assert cp.AeroProvider(1.1, aero_map=Map(cl=None)).cla_cda(att) == (2.5, 1.2)
    assert cp.AeroProvider(1.1, aero_map=Map(cd=None)).cla_cda(att) == (2.5, 1.2)
    m2 = Map()
    cp.AeroProvider(1.0, aero_map=m2).scalar_for_speed(22.0, roll_deg=1.0, pitch_deg=-0.5, yaw_deg=2.0,
                                                       ride_height_mm=25.0)
    assert m2.asked == [Attitude(1.0, -0.5, 2.0, 25.0, 22.0)]
    cp.AeroProvider(1.0, aero_map=m2).scalar_for_speed(10.0)
    assert m2.asked[-1] == Attitude(0.0, 0.0, 0.0, 30.0, 10.0)


def test_the_attitude_estimate():
    assert cp.estimate_attitude(20.0, lat_g=1.5, long_g=-1.0, roll_grad_deg_per_g=1.0, pitch_grad_deg_per_g=0.5,
                                static_ride_mm=28.0) == Attitude(1.5, -0.5, 0.0, 28.0, 20.0)
    assert cp.estimate_attitude(15.0, 1.0, 1.0) == Attitude(1.2, 0.6, 0.0, 30.0, 15.0)


def _car(**kw):
    p = NS(use_spring_rates=True, spring_rate_front=30.0, spring_rate_rear=35.0, wheelbase=1550.0)
    for k, v in kw.items():
        setattr(p, k, v)
    return NS(p=p, front_kin=NS(motion_ratio=lambda: 0.8), rear_kin=NS(motion_ratio=lambda: 0.9))


def test_the_axle_ride_rate():
    c = _car()
    assert cp._axle_wheel_rate(c, "front") == pytest.approx(2 * 30.0 * 0.64)
    assert cp._axle_wheel_rate(c, "rear") == pytest.approx(2 * 35.0 * 0.81)
    assert cp._axle_wheel_rate(NS(), "front") is None
    assert cp._axle_wheel_rate(_car(use_spring_rates=False), "front") is None
    assert cp._axle_wheel_rate(NS(p=NS(spring_rate_front=30.0, use_spring_rates=True)), "front") is None
    assert cp._axle_wheel_rate(_car(spring_rate_rear=None), "rear") is None
    assert cp._axle_wheel_rate(NS(p=_car().p, front_kin=NS(motion_ratio=lambda: 0.8)), "rear") is None
    for mr in (None, float("nan"), 0.0, -0.5, lambda: 1 / 0):
        k = NS(motion_ratio=mr if callable(mr) else (lambda m=mr: m))
        assert cp._axle_wheel_rate(NS(p=_car().p, front_kin=k), "front") is None
    nos = NS(p=NS(spring_rate_front=30.0), front_kin=NS(motion_ratio=lambda: 0.8))
    assert cp._axle_wheel_rate(nos, "front") is None


class Veh:
    def __init__(self, roll=2.0, dW=400.0, ad=30.0, asq=20.0, fail=(), **kw):
        base = _car(**kw)
        self.p, self.front_kin, self.rear_kin = base.p, base.front_kin, base.rear_kin
        self.roll, self.dW, self.ad, self.asq, self.fail, self.calls = roll, dW, ad, asq, fail, []

    def lateral_load_transfer(self, g):
        self.calls.append(("lat", g))
        if "lat" in self.fail:
            raise RuntimeError("no roll")
        return None, {"roll_angle": self.roll}

    def longitudinal_load_transfer(self, g):
        self.calls.append(("long", g))
        if "long" in self.fail:
            raise RuntimeError("no pitch")
        return self.dW * g, None

    def anti_dive_pct(self, brake_bias_front):
        self.calls.append(("ad", brake_bias_front))
        return self.ad

    def anti_squat_pct(self, drive_bias_rear):
        self.calls.append(("as", drive_bias_rear))
        return self.asq


KF, KR = 2 * 30 * 0.64, 2 * 35 * 0.81


def test_roll_and_pitch_under_braking():
    v = Veh()
    att, info = cp.attitude_from_dynamics(v, lat_g=-1.2, long_g=1.5, speed_ms=18.0, yaw_deg=1.5,
                                          brake_bias_front=0.6)
    assert ("lat", 1.2) in v.calls and ("ad", 0.6) in v.calls
    df, dr = 600.0 / KF * 0.7, -600.0 / KR
    assert att.roll_deg == pytest.approx(-2.0) and att.pitch_deg == pytest.approx(
        math.degrees(math.atan2(dr - df, 1550.0)))
    assert (att.yaw_deg, att.ride_height_mm, att.speed_ms) == (1.5, 30.0, 18.0)
    assert info == {"roll_source": "dynamics(spring/MR/RC)", "pitch_source": "dynamics(loadtransfer/anti-dive)",
                    "ride_source": "static"}
    assert all(type(x) is float for x in (att.roll_deg, att.pitch_deg, att.ride_height_mm))


def test_pitch_under_acceleration_and_clamped_anti_fractions():
    v = Veh(asq=150.0)
    att, _ = cp.attitude_from_dynamics(v, lat_g=0.0, long_g=-0.8, speed_ms=10.0, drive_bias_rear=0.7)
    assert ("as", 0.7) in v.calls and att.roll_deg == 0.0
    dw = 400.0 * -0.8
    assert att.pitch_deg == pytest.approx(math.degrees(math.atan2(-dw / KR * 0.0 - dw / KF, 1550.0)))
    neg = cp.attitude_from_dynamics(Veh(ad=-50.0), 0.5, 1.0, 10.0)[0]
    assert neg.pitch_deg == pytest.approx(math.degrees(math.atan2(-400.0 / KR - 400.0 / KF, 1550.0)))
    nan = cp.attitude_from_dynamics(Veh(ad=float("nan")), 0.5, 1.0, 10.0)[0]
    assert nan.pitch_deg == pytest.approx(neg.pitch_deg)
    nas = cp.attitude_from_dynamics(Veh(asq=float("nan")), 0.0, -1.0, 10.0)[0]
    assert nas.pitch_deg == pytest.approx(math.degrees(math.atan2(400.0 / KR + 400.0 / KF, 1550.0)))
    d = cp.attitude_from_dynamics(Veh(), 0.0, 1.0, 10.0)[0]
    assert d.pitch_deg == pytest.approx(math.degrees(math.atan2(-400.0 / KR - 400.0 / KF * 0.7, 1550.0)))
    v2 = Veh()
    cp.attitude_from_dynamics(v2, 0.0, 1.0, 10.0)
    cp.attitude_from_dynamics(v2, 0.0, -1.0, 10.0)
    assert ("ad", 0.65) in v2.calls and ("as", 1.0) in v2.calls


def test_neutral_channels_are_flagged():
    att, info = cp.attitude_from_dynamics(Veh(fail=("lat", "long")), 1.0, 1.0, 10.0)
    assert (att.roll_deg, att.pitch_deg) == (0.0, 0.0)
    assert info == {"roll_source": "neutral", "pitch_source": "neutral", "ride_source": "static",
                    "roll_error": "no roll", "pitch_error": "no pitch"}
    att, info = cp.attitude_from_dynamics(Veh(roll=float("nan")), 1.0, 0.0, 10.0)
    assert att.roll_deg == 0.0 and info == {"roll_source": "neutral", "pitch_source": "neutral",
                                            "ride_source": "static"}
    no_rate = Veh(use_spring_rates=False)
    att, info = cp.attitude_from_dynamics(no_rate, 1.0, 1.0, 10.0)
    assert att.pitch_deg == 0.0 and info["pitch_note"] == ("no ride rate available (set use_spring_rates and attach "
                                                           "geometry) -> pitch left neutral")
    assert "pitch_note" not in cp.attitude_from_dynamics(no_rate, 1.0, 0.0, 10.0)[1]
    assert not any(c[0] == "long" for c in Veh().calls)
    still = Veh()
    cp.attitude_from_dynamics(still, 1.0, 0.0, 10.0)
    assert not any(c[0] == "long" for c in still.calls)
    rear_only = Veh()
    rear_only.front_kin = None
    assert cp.attitude_from_dynamics(rear_only, 1.0, 1.0, 10.0)[1]["pitch_source"] == "neutral"
    front_only = Veh()
    front_only.rear_kin = None
    assert cp.attitude_from_dynamics(front_only, 1.0, 1.0, 10.0)[1]["pitch_source"] == "neutral"
    nop = NS(lateral_load_transfer=lambda g: (None, {"roll_angle": 1.0}))
    att, info = cp.attitude_from_dynamics(nop, 1.0, 1.0, 10.0)
    assert att.roll_deg == 1.0 and info["pitch_note"].startswith("no ride rate")
    bare = NS(lateral_load_transfer=lambda g: (None, {}))
    assert cp.attitude_from_dynamics(bare, 1.0, 0.0, 10.0)[0].roll_deg == 0.0


def test_the_aero_heave():
    prov = cp.AeroProvider(1.0, aero_map=Map(cl=-3.2))
    att, info = cp.attitude_from_dynamics(Veh(), 0.0, 0.0, 25.0, static_ride_mm=32.0, aero_provider=prov, rho=1.2)
    heave = 0.5 * 1.2 * 3.2 * 625 / (KF + KR)
    assert att.ride_height_mm == pytest.approx(32.0 - heave) and info["aero_heave_mm"] == pytest.approx(heave)
    assert info["ride_source"] == "static - aero heave"
    assert prov.aero_map.asked[-1] == Attitude(0.0, 0.0, 0.0, 32.0, 25.0)
    d = cp.attitude_from_dynamics(Veh(), 0.0, 0.0, 25.0, aero_provider=prov)[1]["aero_heave_mm"]
    assert d == pytest.approx(0.5 * 1.225 * 3.2 * 625 / (KF + KR))
    v = Veh()
    att, _ = cp.attitude_from_dynamics(v, 1.0, 1.0, 25.0, aero_provider=prov)
    assert prov.aero_map.asked[-1] == Attitude(2.0, att.pitch_deg, 0.0, 30.0, 25.0)
    low = cp.attitude_from_dynamics(Veh(), 0.0, 0.0, 200.0, static_ride_mm=10.0, aero_provider=prov)[0]
    assert low.ride_height_mm == 1.0
    assert cp.attitude_from_dynamics(Veh(use_spring_rates=False), 0, 0, 25.0, aero_provider=prov)[1]["ride_source"] \
        == "static"

    class Bad:
        def scalar_for_speed(self, *a, **k):
            raise ValueError("map broke")
    att, info = cp.attitude_from_dynamics(Veh(), 0.0, 0.0, 25.0, aero_provider=Bad())
    assert att.ride_height_mm == 30.0 and info["ride_error"] == "map broke" and info["ride_source"] == "static"


def test_more_attitude_edges():
    assert cp.estimate_attitude(12.0, lat_g=-2.0, long_g=2.0) == Attitude(-2.4, 1.2, 0.0, 30.0, 12.0)
    att, info = cp.attitude_from_dynamics(NS(lateral_load_transfer=lambda g: (None, {})), 1.0, 0.0, 10.0)
    assert att.roll_deg == 0.0 and info["roll_source"] == "dynamics(spring/MR/RC)" and att.yaw_deg == 0.0
    mid = cp.attitude_from_dynamics(Veh(asq=30.0), 0.0, -1.0, 10.0)[0]
    assert mid.pitch_deg == pytest.approx(math.degrees(math.atan2(400.0 / KR * 0.7 + 400.0 / KF, 1550.0)))
    over = cp.attitude_from_dynamics(Veh(asq=-20.0), 0.0, -1.0, 10.0)[0]
    assert over.pitch_deg == pytest.approx(math.degrees(math.atan2(400.0 / KR + 400.0 / KF, 1550.0)))
    assert cp._resisted_fraction(30.0) == pytest.approx(0.7) and cp._resisted_fraction(-20.0) == 1.0
    assert cp._resisted_fraction(150.0) == 0.0 and cp._resisted_fraction(float("inf")) == 1.0
    with pytest.raises(KeyError):
        cp._axle_wheel_rate(_car(), "middle")
    bare = NS(p=NS(use_spring_rates=True), front_kin=NS(motion_ratio=lambda: 0.8),
              rear_kin=NS(motion_ratio=lambda: 0.8))
    assert cp._axle_wheel_rate(bare, "front") is None and cp._axle_wheel_rate(bare, "rear") is None
    noflag = NS(p=NS(spring_rate_front=30.0), front_kin=NS(motion_ratio=lambda: 0.8))
    assert cp._axle_wheel_rate(noflag, "front") is None
    assert cp.AeroProvider(1.0, aero_map=Map(n=1)).is_mapped() is True
    assert cp.AeroProvider(1.0, aero_map=Map(n=0)).is_mapped() is False
