"""suspension/throttle_dynamics.py against the textbook forms written out here: the butterfly's open area, the
isentropic orifice with choking, the plate + plenum system marched by an RK4 coded in this file (spring line,
Coulomb stick and slip, the vacuum torque, the filling ODE), and the flutter screen's damping ratio, onset speed and
verdicts, each message checked word for word."""
import math

import pytest

from suspension import throttle_dynamics as td
from suspension.interfaces import Severity
from suspension.throttle_return import ReturnResistance, ReturnSpring, ThrottleInertia

G = 1.4


def test_the_constants_and_the_open_area():
    assert (td.R_AIR, td.GAMMA) == (287.0, 1.4)
    for th in (0.0, 0.3, 1.0, math.pi / 2):
        frac = 0.5 * (1 - math.cos(2 * th))
        assert td.throttle_flow_area(th, 2e-3) == pytest.approx(2e-3 * (0.02 + 0.98 * frac), rel=1e-12)
        assert td.throttle_flow_area(th, 2e-3, 0.1) == pytest.approx(2e-3 * (0.1 + 0.9 * frac), rel=1e-12)
    assert td.throttle_flow_area(2.5, 2e-3) == pytest.approx(2e-3)                   # past 90 deg: full bore
    assert td.throttle_flow_area(-0.4, 2e-3) == pytest.approx(2e-3 * 0.02)           # below closed: the leak
    assert td.throttle_flow_area(0.5, 2e-3, -0.3) == pytest.approx(2e-3 * 0.5 * (1 - math.cos(1.0)))


def _mdot(A, pu, pd, T=298.0, Cd=0.7):
    crit = (2 / (G + 1)) ** (G / (G - 1))
    pr = max(pd / pu, crit)
    return Cd * A * pu / math.sqrt(287.0 * T) * math.sqrt(2 * G / (G - 1) * (pr ** (2 / G) - pr ** ((G + 1) / G)))


def test_the_orifice_flow_subsonic_and_choked():
    assert td.compressible_mass_flow(1e-3, 101325.0, 90000.0) == pytest.approx(_mdot(1e-3, 101325.0, 90000.0), rel=1e-12)
    assert td.compressible_mass_flow(1e-3, 101325.0, 90000.0, 330.0, 0.6) == pytest.approx(
        _mdot(1e-3, 101325.0, 90000.0, 330.0, 0.6), rel=1e-12)
    choked = td.compressible_mass_flow(1e-3, 101325.0, 20000.0)
    assert choked == pytest.approx(_mdot(1e-3, 101325.0, 20000.0), rel=1e-12)
    assert choked == pytest.approx(td.compressible_mass_flow(1e-3, 101325.0, 40000.0), rel=1e-12)
    assert choked > td.compressible_mass_flow(1e-3, 101325.0, 60000.0)
    for args in ((0.0, 1e5, 5e4), (-1e-3, 1e5, 5e4), (1e-3, 0.0, 5e4), (1e-3, -1.0, 5e4), (1e-3, 1e5, 1e5),
                 (1e-3, 1e5, 1.2e5)):
        assert td.compressible_mass_flow(*args) == 0.0
    assert td.compressible_mass_flow(1e-3, 1e5, 0.99999e5) > 0


def test_the_parameter_sets():
    m = td.ManifoldParams()
    assert m.as_dict() == dict(plenum_volume_m3=2.0e-3, bore_area_m2=1.5e-3, p_ambient_Pa=101325.0,
                               T_ambient_K=298.0, engine_draw_kgps=0.0, Cd=0.7, p_manifold0_Pa=None)
    f = td.FlutterParams()
    assert f.as_dict() == dict(k_theta_Nm_per_rad=2.0, c_struct_Nms=1.0e-3, c_aero_Nms=0.0, c_aero_ref_speed_ms=0.0)
    assert f.aero_unquantified() is True and td.FlutterParams(c_aero_Nms=-1e-4).aero_unquantified() is False


def _oracle(springs, inertia, man, res, theta_open_deg, coeff, dt, t_max):
    I = max(inertia.I_kgm2, 1e-9)
    th_open = math.radians(max(theta_open_deg, 1e-3))
    Tc, To = sum(s.torque_closed_Nm for s in springs), sum(s.torque_open_Nm for s in springs)
    cap = abs(res.friction_Nm) + abs(res.cable_drag_Nm) + abs(res.sensor_detent_Nm)
    pa = man.p_ambient_Pa
    V = max(man.plenum_volume_m3, 1e-9)

    def f(th, om, p):
        frac = min(max(th / th_open, 0.0), 1.0)
        drive = -(Tc + (To - Tc) * frac) + coeff * (pa - p)
        if abs(om) > 1e-3:
            a = (drive - cap * math.copysign(1.0, om)) / I
        elif abs(drive) <= cap:
            a = 0.0
        else:
            a = (drive - math.copysign(cap, drive)) / I
        A = man.bore_area_m2 * (0.02 + 0.98 * 0.5 * (1 - math.cos(2 * min(max(th, 0.0), math.pi / 2))))
        md = _mdot(A, pa, p, man.T_ambient_K, man.Cd) if p < pa else 0.0
        return om, a, 287.0 * man.T_ambient_K / V * (md - max(man.engine_draw_kgps, 0.0))
    th, om, p, t = th_open, 0.0, (man.p_manifold0_Pa if man.p_manifold0_Pa is not None else pa), 0.0
    peak, pmin, trace, done = 0.0, p, [(0.0, theta_open_deg, 0.0, p / 1000.0)], False
    for _ in range(int(t_max / max(dt, 1e-6))):
        k1 = f(th, om, p)
        k2 = f(th + dt / 2 * k1[0], om + dt / 2 * k1[1], p + dt / 2 * k1[2])
        k3 = f(th + dt / 2 * k2[0], om + dt / 2 * k2[1], p + dt / 2 * k2[2])
        k4 = f(th + dt * k3[0], om + dt * k3[1], p + dt * k3[2])
        th += dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        om += dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
        p = max(p + dt / 6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]), 1.0)
        t += dt
        peak, pmin = max(peak, abs(om)), min(pmin, p)
        if len(trace) < 6000:
            trace.append((t, math.degrees(max(th, 0.0)), om, p / 1000.0))
        if th <= 0.0:
            done, th = True, 0.0
            break
    return done, t, th, peak, pmin, p, trace


SPRINGS = [ReturnSpring("a", 0.05, 0.09), ReturnSpring("b", 0.03, 0.06, is_estimate=False)]


def test_a_coupled_return_against_rk4_written_here():
    inertia = ThrottleInertia(I_kgm2=6e-4, is_estimate=False)
    man = td.ManifoldParams(plenum_volume_m3=1.5e-3, bore_area_m2=1.2e-3, engine_draw_kgps=0.02, Cd=0.65,
                            T_ambient_K=300.0, p_manifold0_Pa=95000.0)
    res = ReturnResistance(friction_Nm=-0.004, cable_drag_Nm=0.003, sensor_detent_Nm=-0.002)
    r = td.simulate_coupled_return(SPRINGS, inertia, man, res, theta_open_deg=80.0, manifold_torque_coeff=2e-7,
                                   dt=3e-4, t_max=1.0)
    done, t, th, peak, pmin, p, trace = _oracle(SPRINGS, inertia, man, res, 80.0, 2e-7, 3e-4, 1.0)
    assert done and r.returns is True and r.hung_at_deg == 0.0 and r.is_estimate is False
    assert r.return_time_s == pytest.approx(t, rel=1e-12) and r.peak_speed_rad_s == pytest.approx(peak, rel=1e-9)
    assert r.min_manifold_kpa == pytest.approx(pmin / 1000, rel=1e-9) and r.final_manifold_kpa == pytest.approx(
        p / 1000, rel=1e-9)
    assert len(r.trace) == len(trace) and r.trace[0] == (0.0, 80.0, 0.0, 95.0)
    for a, b in zip(r.trace[1::7], trace[1::7]):
        assert a == pytest.approx(b, rel=1e-9, abs=1e-12)
    assert r.trace[-1][1] == 0.0
    (f,) = r.findings
    assert (f.check, f.severity, f.subsystems) == ("throttle-coupled", Severity.OK, ["brakes", "powertrain"])
    assert f.message == (f"Coupled plate+manifold return: closes in {t * 1000:.0f} ms; manifold vacuum reached "
                         f"{(101325.0 - pmin) / 1000:.1f} kPa below ambient.")
    assert f.detail == pytest.approx(dict(return_time_s=t, min_manifold_kpa=pmin / 1000, estimate=False))
    d = r.as_dict()
    assert d["n_trace"] == len(trace) and d["findings"][0]["severity"] == "ok" and d["returns"] is True
    assert {k: d[k] for k in ("return_time_s", "hung_at_deg", "peak_speed_rad_s", "min_manifold_kpa",
                              "final_manifold_kpa", "is_estimate")} == {
        "return_time_s": r.return_time_s, "hung_at_deg": 0.0, "peak_speed_rad_s": r.peak_speed_rad_s,
        "min_manifold_kpa": r.min_manifold_kpa, "final_manifold_kpa": r.final_manifold_kpa, "is_estimate": False}
    assert pmin < 95000.0                                                    # the plenum was drawn down


def test_the_defaults_and_an_estimated_spring():
    springs = [ReturnSpring("a", 0.06, 0.10, is_estimate=True)]
    r = td.simulate_coupled_return(springs, t_max=0.5)
    done, t, th, peak, pmin, p, trace = _oracle(springs, ThrottleInertia(), td.ManifoldParams(), ReturnResistance(),
                                                90.0, 0.0, 2e-4, 0.5)
    assert done and r.returns and r.return_time_s == pytest.approx(t, rel=1e-12) and r.is_estimate is True
    assert r.findings[0].message.startswith(f"Coupled plate+manifold return: closes in {t * 1000:.0f} ms (ESTIMATED "
                                            "inertia/rate); manifold vacuum")
    assert r.trace[0] == (0.0, 90.0, 0.0, 101.325)
    est = td.simulate_coupled_return(SPRINGS, ThrottleInertia(is_estimate=True), t_max=0.5)
    assert est.is_estimate is True
    noflag = td.simulate_coupled_return(SPRINGS, ThrottleInertia(is_estimate=False), t_max=0.5)
    assert noflag.is_estimate is False


def test_a_plate_held_open_by_friction():
    res = ReturnResistance(friction_Nm=0.2)
    r = td.simulate_coupled_return(SPRINGS, ThrottleInertia(is_estimate=False), td.ManifoldParams(engine_draw_kgps=0.01),
                                   res, theta_open_deg=60.0, dt=2e-4, t_max=1.3)
    done, t, th, peak, pmin, p, trace = _oracle(SPRINGS, ThrottleInertia(), td.ManifoldParams(engine_draw_kgps=0.01),
                                                res, 60.0, 0.0, 2e-4, 1.3)
    assert not done and r.returns is False and r.return_time_s == math.inf
    assert r.hung_at_deg == pytest.approx(60.0) and r.peak_speed_rad_s == 0.0 and len(r.trace) == 6000
    assert r.final_manifold_kpa == pytest.approx(p / 1000, rel=1e-9)
    (f,) = r.findings
    assert f.severity == Severity.FAIL and f.check == "throttle-coupled" and f.subsystems == ["brakes", "powertrain"]
    assert f.message == ("Coupled model: throttle did NOT close within 1300 ms (stuck near 60°). If "
                         "manifold_torque_coeff>0, the developing vacuum may be holding it — check that against the "
                         "spring.")
    assert f.detail == pytest.approx(dict(hung_at_deg=60.0, estimate=False))


def test_a_vacuum_that_pulls_the_plate_past_the_friction():
    """Springs too weak to close against friction, but a strong vacuum torque coefficient drives it the other way:
    the stuck branch must subtract the friction in the direction of the drive."""
    springs = [ReturnSpring("weak", 0.001, 0.001)]
    res = ReturnResistance(friction_Nm=0.01)
    man = td.ManifoldParams(engine_draw_kgps=0.05, p_manifold0_Pa=60000.0)
    r = td.simulate_coupled_return(springs, ThrottleInertia(I_kgm2=1e-3, is_estimate=False), man, res,
                                   theta_open_deg=30.0, manifold_torque_coeff=1e-6, dt=5e-4, t_max=0.2)
    done, t, th, peak, pmin, p, trace = _oracle(springs, ThrottleInertia(I_kgm2=1e-3), man, res, 30.0, 1e-6, 5e-4, 0.2)
    assert r.peak_speed_rad_s == pytest.approx(peak, rel=1e-9) and peak > 0
    assert r.hung_at_deg == pytest.approx(math.degrees(th), rel=1e-9) and r.hung_at_deg > 30.0


def test_tiny_inputs_are_floored():
    r = td.simulate_coupled_return(SPRINGS, ThrottleInertia(I_kgm2=0.0), td.ManifoldParams(plenum_volume_m3=0.0),
                                   theta_open_deg=0.0, dt=1e-7, t_max=2e-6)
    done, t, th, peak, pmin, p, trace = _oracle(SPRINGS, ThrottleInertia(I_kgm2=0.0),
                                                td.ManifoldParams(plenum_volume_m3=0.0), ReturnResistance(), 0.0,
                                                0.0, 1e-7, 2e-6)
    assert r.returns is done and len(r.trace) == len(trace) and r.min_manifold_kpa == pytest.approx(pmin / 1000)


def test_the_flutter_screen_on_structure_only():
    r = td.screen_plate_flutter(ThrottleInertia(I_kgm2=4e-4), td.FlutterParams(k_theta_Nm_per_rad=3.0,
                                                                               c_struct_Nms=2e-3))
    wn = math.sqrt(3.0 / 4e-4)
    assert r.natural_freq_hz == pytest.approx(wn / (2 * math.pi)) and r.damping_ratio == pytest.approx(
        2e-3 / (2 * math.sqrt(3.0 * 4e-4)))
    assert (r.stable, r.aero_modelled, r.onset_speed_ms, r.is_screen) == (True, False, None, True)
    (f,) = r.findings
    assert f.severity == Severity.WARN and f.check == "throttle-flutter" and f.subsystems == ["brakes", "powertrain"]
    assert f.message == (f"Flutter screen ran on STRUCTURAL damping only (natural freq {wn / (2 * math.pi):.0f} Hz): no "
                         "aerodynamic damping coefficient was supplied, so the aeroelastic part — the part that "
                         "actually causes flutter — was NOT modelled. This does not say the plate won't flutter. Get "
                         "the aero damping derivative from CFD or a flow rig, or take flutter straight to ANSYS "
                         "CFX/Fluent.")
    assert f.detail == pytest.approx(dict(natural_freq_hz=wn / (2 * math.pi), c_struct=2e-3))
    d = r.as_dict()
    assert d["findings"][0]["severity"] == "warning" and (d["stable"], d["aero_modelled"], d["is_screen"]) == (
        True, False, True)
    assert (d["damping_ratio"], d["natural_freq_hz"], d["onset_speed_ms"]) == (r.damping_ratio, r.natural_freq_hz, None)
    z = td.screen_plate_flutter(ThrottleInertia(I_kgm2=0.0), td.FlutterParams(k_theta_Nm_per_rad=0.0, c_struct_Nms=0.0))
    assert z.stable is False and z.natural_freq_hz == pytest.approx(math.sqrt(1e-9 / 1e-12) / (2 * math.pi))


def test_the_flutter_screen_with_aero_damping():
    I = ThrottleInertia(I_kgm2=5e-4)
    neg = td.FlutterParams(k_theta_Nm_per_rad=2.0, c_struct_Nms=1e-3, c_aero_Nms=-4e-4, c_aero_ref_speed_ms=20.0)
    onset = 20.0 * math.sqrt(1e-3 / 4e-4)
    fn = math.sqrt(2.0 / 5e-4) / (2 * math.pi)
    slow = td.screen_plate_flutter(I, neg, 15.0)
    c = 1e-3 - 4e-4 * (15.0 / 20.0) ** 2
    zeta = c / (2 * math.sqrt(2.0 * 5e-4))
    assert slow.stable is True and slow.damping_ratio == pytest.approx(zeta) and slow.onset_speed_ms == pytest.approx(onset)
    (f,) = slow.findings
    assert f.severity == Severity.WARN and f.message == (
        f"Flutter SCREEN: stable at 15 m/s (damping ratio {zeta:.3f}, natural freq {fn:.0f} Hz). But aero damping goes "
        f"negative above ~{onset:.0f} m/s — flutter onset there. Screen only — a 1-DOF lumped aeroelastic model is "
        "indicative; confirm with CFD/FEA before sign-off.")
    assert f.detail == pytest.approx(dict(damping_ratio=zeta, natural_freq_hz=fn, onset_speed_ms=onset))
    fast = td.screen_plate_flutter(I, neg, 40.0)
    c = 1e-3 - 4e-4 * 4.0
    zf = c / (2 * math.sqrt(2.0 * 5e-4))
    assert fast.stable is False and fast.damping_ratio == pytest.approx(zf)
    (f,) = fast.findings
    assert f.severity == Severity.FAIL and f.message == (
        f"Flutter SCREEN: UNSTABLE at 40 m/s (net damping negative, ratio {zf:.3f}) — aero is feeding energy into "
        f"plate oscillation at {fn:.0f} Hz. Screen only, but a red flag: confirm with CFD/FEA and stiffen/damp the "
        "plate or shaft.")
    assert f.detail == pytest.approx(dict(damping_ratio=zf, natural_freq_hz=fn))
    pos = td.FlutterParams(c_struct_Nms=1e-3, c_aero_Nms=5e-4, c_aero_ref_speed_ms=10.0)
    p = td.screen_plate_flutter(I, pos, 20.0)
    zp = (1e-3 + 5e-4 * 4.0) / (2 * math.sqrt(2.0 * 5e-4))
    assert p.stable and p.onset_speed_ms is None and p.damping_ratio == pytest.approx(zp)
    assert p.findings[0].severity == Severity.OK and p.findings[0].message == (
        f"Flutter SCREEN: stable at 20 m/s (damping ratio {zp:.3f}, natural freq {fn:.0f} Hz).Screen only — a 1-DOF "
        "lumped aeroelastic model is indicative; confirm with CFD/FEA before sign-off.")
    # no speed or no reference speed: the coefficient is used as given, and no onset can be scaled
    for v, ref in ((0.0, 10.0), (20.0, 0.0)):
        q = td.screen_plate_flutter(I, td.FlutterParams(c_struct_Nms=1e-3, c_aero_Nms=-3e-4, c_aero_ref_speed_ms=ref), v)
        assert q.damping_ratio == pytest.approx((1e-3 - 3e-4) / (2 * math.sqrt(2.0 * 5e-4)))
        assert q.onset_speed_ms == (None if ref == 0.0 else pytest.approx(10.0 * math.sqrt(1e-3 / 3e-4)))
    # negative aero with no structural damping: no positive crossing
    nz = td.screen_plate_flutter(I, td.FlutterParams(c_struct_Nms=0.0, c_aero_Nms=-3e-4, c_aero_ref_speed_ms=10.0), 5.0)
    assert nz.onset_speed_ms is None and nz.stable is False
    edge = td.screen_plate_flutter(I, td.FlutterParams(c_struct_Nms=0.0, c_aero_Nms=0.0))
    assert edge.stable is False                                          # zero net damping is not stable


def test_a_tiny_upstream_pressure_still_flows():
    assert td.compressible_mass_flow(1e-3, 0.5, 0.1) == pytest.approx(_mdot(1e-3, 0.5, 0.1), rel=1e-12)
    assert td.compressible_mass_flow(-1e-3, 1e5, 5e4) == 0.0


def test_estimates_from_a_spring_and_untyped_parts():
    est = [ReturnSpring("a", 0.05, 0.09, is_estimate=True)]
    assert td.simulate_coupled_return(est, ThrottleInertia(is_estimate=False), t_max=0.05).is_estimate is True
    from types import SimpleNamespace as NS
    bare = [NS(torque_closed_Nm=0.05, torque_open_Nm=0.09)]
    assert td.simulate_coupled_return(bare, NS(I_kgm2=5e-4), t_max=0.05).is_estimate is False


def test_a_small_opening_and_the_default_time_limit():
    r = td.simulate_coupled_return(SPRINGS, ThrottleInertia(is_estimate=False), theta_open_deg=0.5, dt=1e-4,
                                   t_max=0.01)
    o = _oracle(SPRINGS, ThrottleInertia(), td.ManifoldParams(), ReturnResistance(), 0.5, 0.0, 1e-4, 0.01)
    assert r.returns is o[0] and r.trace[1] == pytest.approx(o[6][1], rel=1e-12)
    stuck = td.simulate_coupled_return(SPRINGS, ThrottleInertia(is_estimate=False),
                                       resistance=ReturnResistance(friction_Nm=1.0))
    assert stuck.findings[0].message.startswith("Coupled model: throttle did NOT close within 2000 ms")


def test_the_flutter_findings_carry_their_owners():
    I = ThrottleInertia(I_kgm2=5e-4)
    for fp, v in ((td.FlutterParams(c_struct_Nms=1e-3, c_aero_Nms=5e-4, c_aero_ref_speed_ms=10.0), 20.0),
                  (td.FlutterParams(c_struct_Nms=1e-4, c_aero_Nms=-5e-4, c_aero_ref_speed_ms=10.0), 20.0),
                  (td.FlutterParams(c_struct_Nms=1e-3, c_aero_Nms=-4e-4, c_aero_ref_speed_ms=20.0), 15.0)):
        f = td.screen_plate_flutter(I, fp, v).findings[0]
        assert f.check == "throttle-flutter" and f.subsystems == ["brakes", "powertrain"]
    d = td.screen_plate_flutter(I, td.FlutterParams(c_struct_Nms=1e-3, c_aero_Nms=-4e-4, c_aero_ref_speed_ms=20.0), 15.0)
    assert d.as_dict()["onset_speed_ms"] == pytest.approx(d.onset_speed_ms) and d.onset_speed_ms is not None


def test_the_speed_scaling_edges():
    I = ThrottleInertia(I_kgm2=5e-4)
    z = lambda c: c / (2 * math.sqrt(2.0 * 5e-4))
    default = td.screen_plate_flutter(I, td.FlutterParams(c_struct_Nms=1e-3, c_aero_Nms=-3e-4, c_aero_ref_speed_ms=10.0))
    assert default.damping_ratio == pytest.approx(z(1e-3 - 3e-4))                     # no speed: as given
    slow = td.screen_plate_flutter(I, td.FlutterParams(c_struct_Nms=1e-3, c_aero_Nms=-3e-4, c_aero_ref_speed_ms=10.0),
                                   0.5)
    assert slow.damping_ratio == pytest.approx(z(1e-3 - 3e-4 * 0.0025))
    small_ref = td.screen_plate_flutter(I, td.FlutterParams(c_struct_Nms=1e-3, c_aero_Nms=-3e-4,
                                                            c_aero_ref_speed_ms=0.5), 1.0)
    assert small_ref.damping_ratio == pytest.approx(z(1e-3 - 3e-4 * 4.0))
    assert small_ref.onset_speed_ms == pytest.approx(0.5 * math.sqrt(1e-3 / 3e-4))
    odd = td.screen_plate_flutter(I, td.FlutterParams(c_struct_Nms=-1e-3, c_aero_Nms=5e-4, c_aero_ref_speed_ms=10.0),
                                  5.0)
    assert odd.onset_speed_ms is None                                                   # positive aero: no onset
