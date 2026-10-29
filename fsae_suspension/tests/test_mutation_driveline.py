"""suspension/driveline.py against the torque-split algebra written out by hand (open: twice the inside capacity, LSD:
inside + min(outside, TBR x inside + preload), spool: both capacities), the friction-circle share sqrt(1 - lambda^2),
the rear vertical loads plus the rear share of downforce, the lap sim's drive_grip_frac definition, the lock derate,
and the reference parts. A stand-in vehicle (loads and tire capacity in closed form) keeps each check to its own
arithmetic; the whole-car mu is also checked on the real vehicle model."""
import math
from dataclasses import asdict, replace

import pytest

from suspension import driveline as dl
from suspension.dynamics import CornerLoads, VehicleDynamics, VehicleParams
from suspension.tiremodel import default_tire


class Car:
    """Corner loads linear in lateral g, tire capacity 1.6 Fz - 1e-4 Fz^2, a fixed limit."""

    def __init__(self, ay=1.4, p=None, raise_limit=False):
        self.p, self.ay, self.raise_limit, self.asked = p or VehicleParams(), ay, raise_limit, []

    def max_lateral_g(self):
        if self.raise_limit:
            raise RuntimeError("no limit")
        return self.ay

    def lateral_load_transfer(self, g):
        self.asked.append(g)
        return CornerLoads(650.0 - 300 * g, 650.0 + 300 * g, 720.0 - 350 * g, 720.0 + 350 * g), {}

    def _corner_force(self, fz, axle):
        assert axle == "rear"
        return 1.6 * fz - 1e-4 * fz * fz

    def axle_grip(self, loads):
        c = lambda z: 1.6 * z - 1e-4 * z * z
        return c(loads.fl) + c(loads.fr), c(loads.rl) + c(loads.rr)


def _mu(car, fz_total):
    s = fz_total / (car.p.mass * car.p.g)
    c = lambda z: 1.6 * z - 1e-4 * z * z
    return min((2 * c(650 * s) + 2 * c(720 * s)) / fz_total, 3.0)


def _want(car, spec, cond, lat_g, ay):
    aero = 0.5 * cond.rho * cond.cl_a * cond.speed_ms ** 2
    per = 0.5 * cond.rear_aero_frac * aero
    fin, fout = max(720 - 350 * lat_g + per, 0.0), max(720 + 350 * lat_g + per, 0.0)
    lam = min(max(lat_g / ay, 0.0), 1.0)
    lf = math.sqrt(1 - lam * lam)
    cap = lambda z: 1.6 * z - 1e-4 * z * z
    ti, to = cap(fin) * lf * cond.r_wheel_m, cap(fout) * lf * cond.r_wheel_m
    if spec.kind == dl.SPOOL:
        out = to
    else:
        out = min(to, spec.effective_tbr() * ti + spec.preload_nm)
    fz = car.p.mass * car.p.g + aero
    mu = _mu(car, fz)
    fx = (ti + out) / cond.r_wheel_m
    return dict(fz_inside_n=fin, fz_outside_n=fout, t_cap_inside_nm=ti, t_cap_outside_nm=to, t_inside_nm=ti,
                t_outside_nm=out, t_total_nm=ti + out, fx_axle_n=fx, lateral_util=lam, mu_car=mu,
                lock_ratio=(out - ti) / (ti + out), drive_grip_frac=min(max(fx / (mu * fz), 0.0), 1.0))


def test_the_spec_and_its_derived_numbers():
    d = dl.DifferentialSpec("x")
    assert (d.kind, d.tbr, d.preload_nm, d.mass_kg, d.cost_usd, d.adjustable_preload, d.mount_points,
            d.sprocket_adapter_included, d.extra_fab_hours, d.extra_parts_usd, d.lead_time_weeks, d.is_estimate,
            d.source) == ("helical_atb", 1.0, 0.0, None, None, False, None, True, 0.0, 0.0, None, True, "")
    assert (dl.OPEN, dl.HELICAL_ATB, dl.PLATE_LSD, dl.SPOOL) == ("open", "helical_atb", "plate_lsd", "spool")
    assert dl.DifferentialSpec("o", kind=dl.OPEN, tbr=3.0).effective_tbr() == 1.0
    assert dl.DifferentialSpec("s", kind=dl.SPOOL, tbr=3.0).effective_tbr() == math.inf
    assert dl.DifferentialSpec("a", tbr=0.4).effective_tbr() == 1.0
    assert dl.DifferentialSpec("a", kind=dl.PLATE_LSD, tbr=3.5).effective_tbr() == 3.5
    assert dl.DifferentialSpec("a").total_acquisition_usd() is None
    assert dl.DifferentialSpec("a", cost_usd=2000.0, extra_parts_usd=150.0).total_acquisition_usd() == 2150.0
    assert dl.DifferentialSpec("a", cost_usd=2000.0).total_acquisition_usd() == 2000.0


def test_the_spec_feeds_the_integration_ledger():
    s = dl.DifferentialSpec("q", tbr=2.5, preload_nm=25.0, mass_kg=4.0, cg_x_mm=1.0, cg_y_mm=2.0, cg_z_mm=3.0,
                            env_x_mm=10.0, env_y_mm=20.0, env_z_mm=30.0, mount_points=4, peak_torque_nm=900.0,
                            is_estimate=False, source="TRE sheet")
    i = s.to_interface()
    assert (i.name, i.mass_kg, i.cg_x_mm, i.cg_y_mm, i.cg_z_mm, i.env_x_mm, i.env_y_mm, i.env_z_mm, i.mount_points,
            i.mounts_on, i.peak_torque_nm, i.is_estimate, i.owner) == ("powertrain-diff", 4.0, 1.0, 2.0, 3.0, 10.0,
                                                                     20.0, 30.0, 4, "chassis", 900.0, False, "")
    assert i.rationale == "helical_atb diff, TBR 2.5, preload 25 N·m. TRE sheet"
    j = s.to_interface(name="diff", owner="pt", rationale="measured")
    assert (j.name, j.owner, j.rationale) == ("diff", "pt", "measured")


def test_the_exit_condition_defaults():
    assert asdict(dl.ExitCondition()) == {"lateral_g_frac": 0.7, "lateral_g": None, "speed_ms": 14.0,
                                          "r_wheel_m": 0.225, "rear_aero_frac": 0.5, "rho": 1.225, "cl_a": 2.5}


def test_the_whole_car_mu_is_the_lap_sims():
    car = Car()
    for fz in (2000.0, 3500.0):
        assert dl._mu_car(car, fz) == pytest.approx(_mu(car, fz))
    real = VehicleDynamics(VehicleParams(), tire=default_tire())
    loads, _ = real.lateral_load_transfer(0.0)
    s = 3200.0 / (280 * 9.81)
    f, r = real.axle_grip(CornerLoads(loads.fl * s, loads.fr * s, loads.rl * s, loads.rr * s))
    assert dl._mu_car(real, 3200.0) == pytest.approx((f + r) / 3200.0)
    assert car.asked == [0.0, 0.0]

    class Grippy(Car):
        def axle_grip(self, loads):
            return 1e6, 1e6
    assert dl._mu_car(Grippy(), 3000.0) == 3.0

    class Dead(Car):
        def axle_grip(self, loads):
            return 0.0, 0.0
    p = VehicleParams(mu_peak=1.4, tire_load_sens=2e-4)
    assert dl._mu_car(Dead(p=p), 3000.0) == pytest.approx(1.4 - 2e-4 * 750.0)

    class Broken(Car):
        def lateral_load_transfer(self, g):
            raise RuntimeError
    assert dl._mu_car(Broken(p=p), 3000.0) == pytest.approx(1.4 - 2e-4 * 750.0)

    class Nan(Car):
        def axle_grip(self, loads):
            return math.nan, 1.0
    assert dl._mu_car(Nan(p=p), 3000.0) == pytest.approx(1.25)


def test_the_axle_traction_for_each_differential():
    car = Car()
    cond = dl.ExitCondition()
    lat = 0.7 * 1.4
    for spec in (dl.DifferentialSpec("open", kind=dl.OPEN), dl.DifferentialSpec("open+p", kind=dl.OPEN,
                                                                                   preload_nm=30.0),
                 dl.DifferentialSpec("atb", tbr=2.5, preload_nm=25.0), dl.DifferentialSpec("lsd", kind=dl.PLATE_LSD,
                                                                                          tbr=1.6),
                 dl.DifferentialSpec("tbr1", tbr=1.0, preload_nm=10.0), dl.DifferentialSpec("spool", kind=dl.SPOOL)):
        r = dl.axle_traction(car, spec, cond)
        assert {k: getattr(r, k) for k in _want(car, spec, cond, lat, 1.4)} == pytest.approx(
            _want(car, spec, cond, lat, 1.4))
        assert r.notes == [] and dl.drive_grip_frac_for(car, spec, cond) == r.drive_grip_frac
    assert car.asked[0] == pytest.approx(lat)
    spool = dl.axle_traction(car, dl.DifferentialSpec("s", kind=dl.SPOOL))
    opn = dl.axle_traction(car, dl.DifferentialSpec("o", kind=dl.OPEN))
    assert spool.t_total_nm > opn.t_total_nm and opn.lock_ratio == 0.0 and spool.lock_ratio > 0
    near = dl.axle_traction(car, dl.DifferentialSpec("n", tbr=1.0 + 5e-10, preload_nm=5.0))
    assert near.t_outside_nm == pytest.approx(min(near.t_cap_outside_nm, near.t_cap_inside_nm + 5.0))


def test_the_corner_and_the_load_case():
    car = Car()
    c = dl.ExitCondition(lateral_g=0.5, speed_ms=20.0, r_wheel_m=0.2, rear_aero_frac=0.6, rho=1.1, cl_a=3.0)
    spec = dl.DifferentialSpec("atb", tbr=2.0)
    r = dl.axle_traction(car, spec, c)
    assert {k: getattr(r, k) for k in _want(car, spec, c, 0.5, 1.4)} == pytest.approx(_want(car, spec, c, 0.5, 1.4))
    for frac in (None, 0.0):
        r = dl.axle_traction(Car(), spec, dl.ExitCondition(lateral_g_frac=frac))
        assert r.lateral_util == pytest.approx(0.7)
    r = dl.axle_traction(Car(), spec, dl.ExitCondition(lateral_g_frac=0.4))
    assert r.lateral_util == pytest.approx(0.4)
    for bad in (Car(raise_limit=True), Car(ay=math.nan), Car(ay=0.0), Car(ay=-1.0)):
        r = dl.axle_traction(bad, spec, dl.ExitCondition(lateral_g_frac=0.5))
        assert r.notes[0] == ("max_lateral_g() was unusable; assumed 1.5 g for the friction-circle split.")
        assert r.lateral_util == pytest.approx(0.5)
    tiny = dl.axle_traction(Car(ay=1e-3), spec, dl.ExitCondition(lateral_g_frac=0.5))
    assert tiny.notes == [] and tiny.lateral_util == pytest.approx(0.5)


def test_a_lifted_wheel_and_a_spent_tire():
    car = Car()
    lift = dl.ExitCondition(lateral_g=2.6, speed_ms=0.0)                      # inside rear at 720 - 910 < 0
    r = dl.axle_traction(Car(ay=3.0), dl.DifferentialSpec("o", kind=dl.OPEN), lift)
    assert r.fz_inside_n == 0.0 and r.t_cap_inside_nm == 0.0 and r.t_outside_nm == 0.0
    assert r.notes == ["Inside rear wheel is at (or off) zero load — an open diff delivers essentially no drive here. "
                       "This is the condition the LSD is bought for; it is also where the model is most sensitive, so "
                       "check the swept band, not the point value."]
    edge = dl.axle_traction(Car(ay=3.0), dl.DifferentialSpec("o", kind=dl.OPEN),
                            dl.ExitCondition(lateral_g=(720 - 1.0) / 350, speed_ms=0.0))
    assert edge.fz_inside_n == pytest.approx(1.0) and len(edge.notes) == 1
    ok = dl.axle_traction(Car(ay=3.0), dl.DifferentialSpec("o", kind=dl.OPEN),
                          dl.ExitCondition(lateral_g=(720 - 1.5) / 350, speed_ms=0.0))
    assert ok.notes == []
    spent = dl.axle_traction(car, dl.DifferentialSpec("o", kind=dl.OPEN), dl.ExitCondition(lateral_g_frac=0.97))
    assert spent.notes == ["Lateral demand is essentially at the tyre limit — almost no longitudinal capacity remains, "
                           "so every differential scores alike here by construction. Lower lateral_g_frac for a "
                           "comparison that means anything."]
    assert dl.axle_traction(car, dl.DifferentialSpec("o"), dl.ExitCondition(lateral_g_frac=0.969)).notes == []
    past = dl.axle_traction(car, dl.DifferentialSpec("s", kind=dl.SPOOL), dl.ExitCondition(lateral_g=2.0))
    assert past.lateral_util == 1.0 and past.t_total_nm == 0.0 and past.lock_ratio == 0.0
    back = dl.axle_traction(car, dl.DifferentialSpec("s", kind=dl.SPOOL), dl.ExitCondition(lateral_g=-0.5))
    assert back.lateral_util == 0.0
    wheel = dl.axle_traction(car, dl.DifferentialSpec("s", kind=dl.SPOOL), dl.ExitCondition(r_wheel_m=0.0))
    assert wheel.fx_axle_n == 0.0 and wheel.t_total_nm == 0.0


def test_the_grip_fraction_is_clamped():
    class Huge(Car):
        def _corner_force(self, fz, axle):
            return 1e7
    r = dl.axle_traction(Huge(), dl.DifferentialSpec("s", kind=dl.SPOOL))
    assert r.drive_grip_frac == 1.0 and r.fx_axle_n / (r.mu_car * (280 * 9.81 + 0.5 * 1.225 * 2.5 * 196)) > 1

    class Neg(Car):
        def _corner_force(self, fz, axle):
            return -500.0
    assert dl.axle_traction(Neg(), dl.DifferentialSpec("s", kind=dl.SPOOL)).drive_grip_frac == 0.0

    class NoMu(Car):
        def axle_grip(self, loads):
            raise RuntimeError
    p = VehicleParams(mu_peak=0.0, tire_load_sens=0.0)
    r = dl.axle_traction(NoMu(p=p), dl.DifferentialSpec("o", kind=dl.OPEN))
    assert r.mu_car == 0.0 and r.drive_grip_frac == 1.0                    # divides by 1e-6, then clamps


def test_the_lock_derate():
    assert dl.LOCK_PENALTY_K_DEFAULT == 0.015 and dl.LOCK_PENALTY_K_BAND == (0.0, 0.05)
    assert dl.lock_lateral_derate(0.4) == pytest.approx(1 - 0.015 * 0.4)
    assert dl.lock_lateral_derate(0.4, k=0.2) == pytest.approx(0.92)
    assert dl.lock_lateral_derate(-0.3) == 1.0 and dl.lock_lateral_derate(1.7) == pytest.approx(0.985)
    assert dl.lock_lateral_derate(1.0, k=0.9) == 0.5 and dl.lock_lateral_derate(0.5, k=0.98) == pytest.approx(0.51)
    p = VehicleParams(mu_peak=1.6)
    assert dl.apply_lock_derate(p, 0.5).mu_peak == pytest.approx(1.6 * (1 - 0.0075))
    assert dl.apply_lock_derate(p, 0.5, k=0.1).mu_peak == pytest.approx(1.6 * 0.95)
    assert dl.apply_lock_derate(p, 0.5) == replace(p, mu_peak=dl.apply_lock_derate(p, 0.5).mu_peak)


def test_the_reference_parts():
    base = dl.DifferentialSpec("x")
    diff = lambda s: {k: v for k, v in asdict(s).items() if k == "name" or v != getattr(base, k)}
    assert diff(dl.CATALOG["open"]) == {"name": "Open differential (baseline)", "kind": "open",
                                        "source": "Baseline concept — supply your own mass/cost."}
    assert diff(dl.CATALOG["spool"]) == {"name": "Spool / solid axle", "kind": "spool", "tbr": math.inf,
                                         "source": "Baseline concept — supply your own mass/cost."}
    assert diff(dl.CATALOG["tre_mk2_center"]) == {
        "name": "TRE Mk2 Quaife ATB — Center Drive, no sprocket adapter", "tbr": 2.5, "preload_nm": 25.0,
        "mass_kg": 4.04, "cost_usd": 2600.0, "sprocket_adapter_included": False, "extra_fab_hours": 25.0,
        "extra_parts_usd": 250.0,
        "source": "taylor-race.com Mk2 listing: 8.9 lb incl. bearings and tripod housings; Center Drive + no adjuster "
                  "variant $2,600. TBR and preload are CLASS ESTIMATES pending the TRE ATB PDF."}
    assert diff(dl.CATALOG["tre_mk2_center_adj"]) == {
        "name": "TRE Mk2 Quaife ATB — Center Drive + Mk2.5 external preload adjuster", "tbr": 2.5,
        "preload_nm": 25.0, "mass_kg": 4.2, "cost_usd": 3025.0, "adjustable_preload": True,
        "sprocket_adapter_included": False, "extra_fab_hours": 25.0, "extra_parts_usd": 250.0,
        "source": "Same listing, Center Drive + Mk2.5 ADJUSTABLE preload variant $3,025. Mass is an ESTIMATE (+~0.15 kg "
                  "for the adjuster)."}
    assert list(dl.CATALOG) == ["open", "spool", "tre_mk2_center", "tre_mk2_center_adj"]
    assert dl.catalog("spool") is dl.CATALOG["spool"]
    o = dl.catalog("tre_mk2_center", tbr=3.1, mass_kg=4.5)
    assert (o.tbr, o.mass_kg, o.cost_usd) == (3.1, 4.5, 2600.0) and dl.CATALOG["tre_mk2_center"].tbr == 2.5


def test_a_whole_car_mu_below_one_is_kept():
    class Slippery(Car):
        def axle_grip(self, loads):
            return 0.4 * (loads.fl + loads.fr), 0.4 * (loads.rl + loads.rr)
    car = Slippery()
    s = 3000.0 / (car.p.mass * car.p.g)
    assert dl._mu_car(car, 3000.0) == pytest.approx(0.4 * 2740.0 * s / 3000.0, rel=1e-12)


def test_the_assumed_limit_sets_the_default_corner():
    car = Car(raise_limit=True)
    r = dl.axle_traction(car, dl.DifferentialSpec("o", kind=dl.OPEN))
    assert car.asked[0] == pytest.approx(0.7 * 1.5) and r.notes[0].startswith("max_lateral_g() was unusable")


def test_a_tiny_torque_still_has_a_lock_ratio_and_a_light_outside_wheel():
    class Weak(Car):
        def _corner_force(self, fz, axle):
            return 1e-3 * fz
    r = dl.axle_traction(Weak(), dl.DifferentialSpec("s", kind=dl.SPOOL))
    assert 0 < r.t_total_nm < 1 and r.lock_ratio == pytest.approx(
        (r.t_outside_nm - r.t_inside_nm) / r.t_total_nm, rel=1e-12) and r.lock_ratio > 0

    class Light(Car):
        def lateral_load_transfer(self, g):
            return CornerLoads(650.0, 650.0, 700.0, 0.5), {}
    q = dl.axle_traction(Light(), dl.DifferentialSpec("o", kind=dl.OPEN), dl.ExitCondition(speed_ms=0.0))
    assert q.fz_outside_n == 0.5


def test_the_scalar_uses_the_given_exit():
    car, spec = Car(), dl.DifferentialSpec("atb", tbr=2.5)
    cond = dl.ExitCondition(lateral_g=0.4, speed_ms=9.0)
    assert dl.drive_grip_frac_for(car, spec, cond) == dl.axle_traction(car, spec, cond).drive_grip_frac
    assert dl.drive_grip_frac_for(car, spec, cond) != dl.drive_grip_frac_for(car, spec)


def test_a_weak_car_on_a_small_grip_is_not_floored_at_one_newton():
    class Faint(Car):
        def _corner_force(self, fz, axle):
            return 1e-5 * fz

        def axle_grip(self, loads):
            raise RuntimeError
    p = VehicleParams(mu_peak=1e-4, tire_load_sens=0.0)
    r = dl.axle_traction(Faint(p=p), dl.DifferentialSpec("s", kind=dl.SPOOL))
    fz = 280 * 9.81 + 0.5 * 1.225 * 2.5 * 196
    assert r.mu_car * fz < 1 and r.drive_grip_frac == pytest.approx(r.fx_axle_n / (1e-4 * fz), rel=1e-12)
