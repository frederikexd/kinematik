"""suspension/ev_powertrain.py around a stand-in lap simulator: the architecture's mass and grip reach the lap sim, the
energy integral is rebuilt here sample by sample (drive through the inverter efficiency, regen capped and scaled),
the pack, derate and torque-vectoring numbers follow their stated arithmetic, the car's mass is restored, and the
comparison ranks and prints what it was given."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from suspension import ev_powertrain as ev

# the classes ev_powertrain itself uses: other suites load private copies of suspension.lapsim into sys.modules
LapResult, LapSimParams = ev.LapResult, ev.LapSimParams

P = ev.Powertrain


def test_the_architectures():
    assert [a.value for a in P] == ["single_diff", "two_axle", "four_tv"]
    assert [a.label() for a in P] == ["Single motor + diff", "Two motors (axle split)", "Four motors (torque vectoring)"]
    assert [a.n_motors() for a in P] == [1, 2, 4]


def test_the_ev_defaults():
    e = ev.EVParams()
    assert (e.pack_energy_kwh, e.pack_usable_frac, e.regen_enabled, e.regen_eff, e.regen_max_g, e.inverter_motor_eff,
            e.g) == (6.5, 0.92, True, 0.55, 0.35, 0.90, 9.81)
    assert e.mass_delta_kg == {P.SINGLE_DIFF: 0.0, P.TWO_AXLE: 7.0, P.FOUR_TV: 16.0}
    assert e.drive_grip_frac == {P.SINGLE_DIFF: 0.78, P.TWO_AXLE: 0.88, P.FOUR_TV: 0.98}
    assert e.tv_yaw_benefit_frac == {P.SINGLE_DIFF: 0.0, P.TWO_AXLE: 0.0, P.FOUR_TV: 0.015}
    assert ev.EVParams().mass_delta_kg is not e.mass_delta_kg


def _lap(d, v, lg, lap_time=50.0, laps=1, ok=True, warnings=()):
    return LapResult(track_name="t", ok=ok, lap_time=lap_time, event_time=lap_time * laps, avg_speed=10.0,
                     top_speed=20.0, distance=np.asarray(d, float), speed=np.asarray(v, float),
                     lat_g=np.zeros(len(d)), long_g=np.asarray(lg, float), limit=["x"] * len(d),
                     warnings=list(warnings))


def _energy(lap, p, e):
    tr = rg = 0.0
    for i in range(1, len(lap.speed)):
        ds = lap.distance[i] - lap.distance[i - 1]
        if not (np.isfinite(ds) and ds > 0):
            continue
        vi = max(lap.speed[i], p.V_MIN)
        F = p.mass * lap.long_g[i] * p.g + 0.5 * p.rho * p.cd_a * vi * vi + p.rolling_g * p.mass * p.g
        if F > 0:
            tr += F * ds / e.inverter_motor_eff
        elif e.regen_enabled:
            rg += min(-F, e.regen_max_g * p.mass * p.g) * ds * e.regen_eff
    net = max(tr - (rg if e.regen_enabled else 0.0), 0.0)
    return net / 3.6e6, rg / 3.6e6


LAP = _lap([0, 10, 10, 25, 40, 60, np.nan, 80], [12, 15, 15, 0.2, 9, 14, 14, 16],
           [0.0, 0.6, 0.3, -1.4, -0.2, 0.4, 0.1, 0.05])


def test_the_lap_energy_integral():
    p = LapSimParams(mass=300.0, cd_a=1.3, rolling_g=0.02)
    e = ev.EVParams(inverter_motor_eff=0.85, regen_eff=0.6, regen_max_g=0.3)
    s = ev.EVLapSimulator(None, ev=e)
    got = s._energy_from_trace(LAP, p)
    assert got == pytest.approx(_energy(LAP, p, e), rel=1e-12) and got[1] > 0
    off = ev.EVParams(regen_enabled=False)
    o = ev.EVLapSimulator(None, ev=off)._energy_from_trace(LAP, p)
    assert o == pytest.approx(_energy(LAP, p, off), rel=1e-12) and o[1] == 0.0
    # regen larger than the drive: the net energy floors at zero
    down = _lap([0, 50, 100], [30, 20, 5], [0, -1.0, -1.0])
    big = ev.EVParams(regen_eff=1.0, regen_max_g=2.0)
    assert ev.EVLapSimulator(None, ev=big)._energy_from_trace(down, p)[0] == 0.0
    assert all(math.isnan(x) for x in s._energy_from_trace(_lap([0], [5], [0]), p))
    bad = SimpleNamespace(speed=[1, 2], distance=None, long_g=[0, 0])
    assert all(math.isnan(x) for x in s._energy_from_trace(bad, p))
    assert s.warnings == ["Energy integration failed; energy values flagged NaN."]


def test_the_architecture_parameters():
    base = LapSimParams(mass=250.0, drive_grip_frac=0.9)
    s = ev.EVLapSimulator(None, base_params=base)
    for a, m, g in ((P.SINGLE_DIFF, 250.0, 0.78), (P.TWO_AXLE, 257.0, 0.88), (P.FOUR_TV, 266.0, 0.98)):
        p = s._params_for(a)
        assert (p.mass, p.drive_grip_frac) == (m, g) and p is not base
    assert (base.mass, base.drive_grip_frac) == (250.0, 0.9)
    part = ev.EVLapSimulator(None, base_params=base, ev=ev.EVParams(mass_delta_kg={}, drive_grip_frac={}))
    assert (part._params_for(P.FOUR_TV).mass, part._params_for(P.FOUR_TV).drive_grip_frac) == (250.0, 0.9)
    assert part.warnings == []
    broken = ev.EVLapSimulator(None, base_params=base, ev=ev.EVParams(mass_delta_kg=None))
    p = broken._params_for(P.TWO_AXLE)
    assert p.mass == 250.0 and broken.warnings == ["Architecture parameter application failed; used base params."]
    d = ev.EVLapSimulator("veh")
    assert (d.veh, d.base, d.ev, d.ds, d.warnings) == ("veh", LapSimParams(), ev.EVParams(), 1.0, [])


class Sim:
    made = []

    def __init__(self, veh, p, ds):
        self.veh, self.p, self.ds = veh, p, ds
        Sim.made.append(self)

    def simulate(self, track):
        self.mass_seen = getattr(self.veh.p, "mass", None) if getattr(self.veh, "p", None) is not None else None
        return Sim.lap(self, track)


def _run(monkeypatch, lap, arch=P.FOUR_TV, laps=3, veh=None, **evkw):
    Sim.made = []
    Sim.lap = lambda self, track: lap
    monkeypatch.setattr(ev, "LapSimulator", Sim)
    veh = veh if veh is not None else SimpleNamespace(p=SimpleNamespace(mass=280.0))
    s = ev.EVLapSimulator(veh, base_params=LapSimParams(mass=280.0), ev=ev.EVParams(**evkw), ds=2.0)
    return s, s.run_architecture(arch, SimpleNamespace(laps=laps, name="enduro")), veh


def test_a_run_with_energy_to_spare(monkeypatch):
    lap = _lap(LAP.distance, LAP.speed, LAP.long_g, lap_time=60.0, laps=3, warnings=["slow"])
    s, r, veh = _run(monkeypatch, lap, pack_energy_kwh=50.0)
    sim = Sim.made[0]
    assert sim.veh is veh and sim.ds == 2.0 and sim.p.mass == 296.0 and sim.mass_seen == 296.0
    assert veh.p.mass == 280.0                                                      # restored afterwards
    e, rg = _energy(lap, sim.p, s.ev)
    assert (r.ok, r.architecture, r.lap_result, r.lap_time, r.event_time) == (True, P.FOUR_TV, lap, 60.0, 180.0)
    assert (r.energy_per_lap_kwh, r.regen_recovered_kwh, r.energy_full_event_kwh) == pytest.approx((e, rg, 3 * e))
    assert r.laps_until_empty == pytest.approx(50.0 * 0.92 / e) and r.finishes_event is True
    assert r.derate_lap_time_penalty_s == 0.0 and r.effective_mass_kg == 296.0
    assert r.tv_yaw_benefit_s == pytest.approx(60.0 * 0.015)
    assert r.tv_yaw_benefit_flagged.startswith("QSS cannot resolve closed-loop L/R yaw control; this is an UPPER-BOUND")
    assert r.tv_yaw_benefit_flagged.endswith("shown separately and NOT added to the lap time.")
    assert r.warnings == ["slow"] and s.warnings == ["slow"]
    one = _run(monkeypatch, lap, arch=P.SINGLE_DIFF, laps=0, pack_energy_kwh=50.0)[1]
    assert one.energy_full_event_kwh == pytest.approx(one.energy_per_lap_kwh) and one.tv_yaw_benefit_s == 0.0
    assert one.tv_yaw_benefit_flagged == "no L/R yaw moment available for this architecture"


def test_a_pack_short_of_the_event_and_one_just_enough(monkeypatch):
    lap = _lap(LAP.distance, LAP.speed, LAP.long_g, lap_time=60.0, laps=4)
    s, r, _ = _run(monkeypatch, lap, P.TWO_AXLE, laps=4, pack_energy_kwh=1e-4)
    e = r.energy_per_lap_kwh
    usable = 1e-4 * 0.92
    f = usable / (4 * e)
    assert r.finishes_event is False and r.laps_until_empty == pytest.approx(usable / e)
    assert r.derate_lap_time_penalty_s == pytest.approx(60.0 * 0.30 * (1 - f))
    assert s.warnings == [f"Two motors (axle split): pack does not cover the event; a ~{(1 - f) * 100:.0f}% energy "
                          f"shortfall implies derating (+{60.0 * 0.30 * (1 - f):.2f} s/lap est., planning-grade)."]
    exact = _run(monkeypatch, lap, P.TWO_AXLE, laps=4, pack_energy_kwh=4 * e / 0.92)[1]
    assert exact.laps_until_empty == 4.0 and exact.finishes_event is True                 # exactly enough still finishes
    flat = _lap([0, 10, 20], [10, 10, 10], [-0.5, -0.5, -0.5], lap_time=30.0)
    z = _run(monkeypatch, flat, laps=2)[1]
    assert z.energy_per_lap_kwh == 0.0 and z.laps_until_empty == math.inf and z.finishes_event is True


def test_failed_and_crashed_runs_and_a_car_without_params(monkeypatch):
    s, r, veh = _run(monkeypatch, _lap([0, 1], [1, 1], [0, 0], ok=False, warnings=["no grip"]))
    assert r.ok is False and r.warnings == ["no grip"] and veh.p.mass == 280.0
    assert math.isnan(r.lap_time) and r.tv_yaw_benefit_flagged == "not evaluated (run failed)"
    assert _run(monkeypatch, _lap([0, 1], [1, 1], [0, 0], ok=False))[1].warnings == ["QSS lap failed"]
    Sim.made = []
    Sim.lap = lambda self, track: 1 / 0
    c = s.run_architecture(P.SINGLE_DIFF, SimpleNamespace(laps=1))
    assert c.warnings == ["architecture run crashed: ZeroDivisionError('division by zero')"]
    bare = SimpleNamespace()
    r = _run(monkeypatch, _lap([0, 10], [10, 10], [0, 0]), veh=bare)[1]
    assert r.ok and not hasattr(bare, "p") and Sim.made[0].veh is bare
    frozen = SimpleNamespace(p=SimpleNamespace())
    s2, r2, _ = _run(monkeypatch, _lap([0, 10], [10, 10], [0, 0]), veh=frozen)
    assert r2.ok and s2.warnings == ["Could not apply architecture mass to vehicle; mass penalty may be understated."]
    assert not hasattr(frozen.p, "mass")                                     # nothing saved, nothing written back


def test_the_failed_result():
    f = ev.EVRunResult.failed(P.TWO_AXLE, ["a"])
    assert (f.architecture, f.ok, f.finishes_event, f.tv_yaw_benefit_s, f.warnings) == (P.TWO_AXLE, False, False, 0.0,
                                                                                       ["a"])
    assert all(math.isnan(x) for x in (f.lap_time, f.event_time, f.energy_per_lap_kwh, f.regen_recovered_kwh,
                                       f.energy_full_event_kwh, f.laps_until_empty, f.derate_lap_time_penalty_s,
                                       f.effective_mass_kg))
    assert f.lap_result.ok is False and f.lap_result.track_name == "ev" and f.lap_result.warnings == ["a"]


def _res(arch, t, ok=True, fin=True, kwh=5.0, laps=9.5):
    r = ev.EVRunResult.failed(arch, [])
    r.ok, r.event_time, r.finishes_event, r.energy_full_event_kwh, r.laps_until_empty = ok, t, fin, kwh, laps
    return r


def test_the_comparison_ranks_and_prints():
    rs = [_res(P.SINGLE_DIFF, 1200.0, fin=False, laps=11.25), _res(P.FOUR_TV, 1180.5, kwh=6.25),
          _res(P.TWO_AXLE, 1150.0, ok=False)]
    c = ev.ArchitectureComparison("Enduro", rs)
    assert c.best_on_time() is rs[1] and c.warnings == []
    assert c.summary().split("\n") == [
        "Architecture comparison on Enduro:",
        f"  {'Four motors (torque vectoring)':32s} {1180.5:7.2f} s {'  ← best':>12s}  | {6.25:5.2f} kWh, finishes",
        f"  {'Single motor + diff':32s} {1200.0:7.2f} s {'(+19.50 s)':>12s}  | {5.0:5.2f} kWh, EMPTY at 11.2 laps",
        f"  ✗ {'Two motors (axle split)':32s} — run failed"]
    nan = ev.ArchitectureComparison("X", [_res(P.FOUR_TV, math.nan), _res(P.TWO_AXLE, 1.0, ok=False)])
    assert nan.best_on_time() is None
    assert nan.summary() == "Architecture comparison on X:\n  (no architecture produced a usable run)"
    mixed = ev.ArchitectureComparison("M", [_res(P.FOUR_TV, math.inf), _res(P.TWO_AXLE, 10.0),
                                            _res(P.SINGLE_DIFF, 10.5), _res(P.TWO_AXLE, 3.0, ok=False)])
    assert mixed.summary().split("\n")[1:] == [
        f"  {'Two motors (axle split)':32s} {10.0:7.2f} s {'  ← best':>12s}  | {5.0:5.2f} kWh, finishes",
        f"  {'Single motor + diff':32s} {10.5:7.2f} s {'(+ 0.50 s)':>12s}  | {5.0:5.2f} kWh, finishes",
        f"  ✗ {'Four motors (torque vectoring)':32s} — run failed",
        f"  ✗ {'Two motors (axle split)':32s} — run failed"]


def test_compare_runs_each_architecture(monkeypatch):
    seen = []
    s = ev.EVLapSimulator(None)
    s.warnings = ["w"]
    monkeypatch.setattr(s, "run_architecture", lambda a, t: seen.append((a, t)) or a)
    tr = SimpleNamespace(name="Skid")
    c = s.compare(tr)
    assert seen == [(a, tr) for a in P] and c.results == list(P) and c.track_name == "Skid" and c.warnings == ["w"]
    assert c.warnings is not s.warnings
    assert s.compare(object(), [P.TWO_AXLE]).track_name == "track"


def test_an_unknown_energy_neither_empties_nor_finishes(monkeypatch):
    monkeypatch.setattr(ev.EVLapSimulator, "_energy_from_trace", lambda self, lap, p: (math.nan, math.nan))
    r = _run(monkeypatch, _lap([0, 10], [10, 10], [0, 0]), laps=2)[1]
    assert r.ok and math.isnan(r.laps_until_empty) and math.isnan(r.energy_full_event_kwh)
    assert r.finishes_event is False and r.derate_lap_time_penalty_s == 0.0
    monkeypatch.setattr(ev.EVLapSimulator, "_energy_from_trace", lambda self, lap, p: (1e-9, 0.0))
    assert _run(monkeypatch, _lap([0, 10], [10, 10], [0, 0]), laps=2)[1].laps_until_empty == math.inf
    monkeypatch.setattr(ev.EVLapSimulator, "_energy_from_trace", lambda self, lap, p: (2e-9, 0.0))
    assert _run(monkeypatch, _lap([0, 10], [10, 10], [0, 0]), laps=2)[1].laps_until_empty == pytest.approx(
        6.5 * 0.92 / 2e-9)


def test_a_wrapped_trace_a_runaway_step_and_a_feather_of_drive():
    p = LapSimParams(mass=300.0)
    e = ev.EVParams()
    s = ev.EVLapSimulator(None, ev=e)
    wrap = _lap([100.0, 110.0, 120.0, 50.0], [10.0, 12.0, 13.0, 9.0], [0.2, 0.3, 0.1, -0.5])
    assert s._energy_from_trace(wrap, p) == pytest.approx(_energy(wrap, p, e), rel=1e-12)
    inf = _lap([0.0, np.inf, 20.0], [10.0, 10.0, 10.0], [0.2, 0.2, 0.2])
    assert s._energy_from_trace(inf, p) == (0.0, 0.0)
    v = 2.0
    lg = (0.5 - 0.5 * p.rho * p.cd_a * v * v - p.rolling_g * p.mass * p.g) / (p.mass * p.g)   # 0.5 N of drive
    feather = _lap([0.0, 1.0], [v, v], [0.0, lg])
    assert s._energy_from_trace(feather, p)[0] == pytest.approx(0.5 * 1.0 / 0.9 / 3.6e6, rel=1e-9)


def test_the_run_passes_the_track_and_names_the_architecture(monkeypatch):
    seen = []
    Sim.made = []
    Sim.lap = lambda self, track: seen.append(track) or _lap([0, 10], [10, 10], [0, 0], ok=False)
    monkeypatch.setattr(ev, "LapSimulator", Sim)
    s = ev.EVLapSimulator(SimpleNamespace(p=SimpleNamespace(mass=280.0)))
    tr = SimpleNamespace(laps=2)
    assert s.run_architecture(P.TWO_AXLE, tr).architecture == P.TWO_AXLE and seen == [tr]
    Sim.lap = lambda self, track: 1 / 0
    assert s.run_architecture(P.FOUR_TV, tr).architecture == P.FOUR_TV
    Sim.lap = lambda self, track: _lap([0, 10, 20], [10, 12, 14], [0.3, 0.3, 0.3], lap_time=40.0)
    one = s.run_architecture(P.SINGLE_DIFF, SimpleNamespace())                 # no laps: one
    assert one.energy_full_event_kwh == pytest.approx(one.energy_per_lap_kwh) and one.energy_per_lap_kwh > 0
    s.ev = ev.EVParams(tv_yaw_benefit_frac={})
    r = s.run_architecture(P.FOUR_TV, tr)
    assert r.ok and r.tv_yaw_benefit_s == 0.0
