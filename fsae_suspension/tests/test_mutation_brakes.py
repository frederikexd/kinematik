"""suspension/brakes.py against oracles written here: the Nusselt surrogate and barrel flow from their stated
formulas, the rotor geometry from annulus/cylinder volumes, the braking-power trace from P = m|a|v split by axle,
and the lumped ring/hat/caliper/fluid/hub network integrated by its own forward-Euler loop. Every message, label and
summary is checked word for word."""
import math
import os
from types import SimpleNamespace

import numpy as np
import pytest

from suspension import brakes as B


# ------------------------------------------------------------------ materials, fluids, pads
def test_the_material_fluid_and_pad_tables():
    m = B.RotorMaterial()
    assert (m.name, m.density, m.cp, m.k_cond, m.emissivity, m.max_service_c) == (
        "grey cast iron", 7150.0, 460.0, 52.0, 0.55, 720.0)
    assert B.ROTOR_MATERIALS["grey cast iron"] == m
    assert B.ROTOR_MATERIALS["stainless 410"] == B.RotorMaterial("stainless 410", 7740.0, 460.0, 25.0, 0.50, 800.0)
    assert B.ROTOR_MATERIALS["aluminium hat 6061"] == B.RotorMaterial("aluminium 6061 (hat)", 2700.0, 896.0, 167.0,
                                                                        0.20, 200.0)
    assert B.BrakeFluid() == B.BrakeFluid("Motul RBF 600", 312.0, 216.0)
    assert B.BRAKE_FLUIDS == {"Motul RBF 600": B.BrakeFluid(),
                              "Motul RBF 660": B.BrakeFluid("Motul RBF 660", 325.0, 204.0),
                              "Castrol SRF": B.BrakeFluid("Castrol SRF", 310.0, 270.0),
                              "DOT 4 generic": B.BrakeFluid("DOT 4 (generic)", 230.0, 155.0)}
    assert B.PadSpec() == B.PadSpec("endurance compound", 0.45, 550.0, 600.0)
    assert [f.value for f in B.RotorCFDFidelity] == ["correlation", "rans_mrf", "rans_sliding", "des"]
    assert issubclass(B.RotorSolverUnavailable, RuntimeError)


# ------------------------------------------------------------------ provenance and the tunnel point
def test_the_provenance_status_lines():
    p = B.RotorCFDProvenance("X", B.RotorCFDFidelity.DES)
    assert (p.is_correlated, p.turbulence_model, p.rotating_wall, p.cell_count, p.correlated_against, p.notes) == (
        False, "", False, None, "", "")
    assert p.status() == ("X (des, STATIONARY wheel (no spin!)), UNCORRELATED — convective coefficient is a model "
                          "output not tied to a measured rotor temperature; trust the speed TREND, not the absolute h_c")
    q = B.RotorCFDProvenance("Y", B.RotorCFDFidelity.RANS_MRF, is_correlated=True, rotating_wall=True,
                             cell_count=2_460_000, correlated_against="rig 7")
    assert q.status() == "Y (rans_mrf, 2.5M cells, rotating wall), correlated against rig 7"
    q.correlated_against = ""
    assert q.status() == "Y (rans_mrf, 2.5M cells, rotating wall), correlated against measured h_c"


def test_the_tunnel_point_and_a_usable_result():
    pt = B.WheelTunnelPoint(12.0)
    assert (pt.rolling_radius_m, pt.rho, pt.mu_air) == (0.22, 1.225, 1.81e-5)
    assert pt.wheel_rad_s == pytest.approx(12.0 / 0.22)
    assert B.WheelTunnelPoint(3.0, rolling_radius_m=0.0).wheel_rad_s == pytest.approx(3.0 / 1e-6)
    r = B.ConvectiveResult(pt)
    assert (r.h_face_w_m2k, r.h_vent_w_m2k, r.mdot_barrel_kg_s, r.converged, r.wall_clock_s, r.provenance,
            r.notes) == (None, None, None, False, None, None, "")
    assert r.is_usable() is False
    assert B.ConvectiveResult(pt, h_face_w_m2k=10.0).is_usable() is False
    assert B.ConvectiveResult(pt, converged=True).is_usable() is False
    assert B.ConvectiveResult(pt, h_face_w_m2k=0.0, converged=True).is_usable() is True


# ------------------------------------------------------------------ the analytic surrogate
def _h(pt, L=0.16, ratio=1.0, spin=0.35):
    re = pt.rho * max(pt.speed_ms, 0.1) * L / pt.mu_air
    h = 0.0296 * re ** 0.8 * 0.71 ** (1 / 3) * 0.026 / L
    tip = pt.speed_ms / max(pt.rolling_radius_m, 1e-6) * pt.rolling_radius_m
    h *= 1 + spin * (tip / max(pt.speed_ms, 0.1))
    return max(h, 5.0) * ratio


def test_the_reference_surrogate():
    s = B.ReferenceRotorCFD()
    assert (s.char_length_m, s.face_area_ratio, s.vent_h_multiplier, s.barrel_area_m2, s.spin_augment) == (
        0.16, 1.0, 1.6, 0.018, 0.35)
    assert s.name == "reference-rotor-cfd"
    p = s.provenance()
    assert (p.backend, p.fidelity, p.is_correlated, p.turbulence_model, p.rotating_wall, p.cell_count) == (
        "reference-rotor-cfd", B.RotorCFDFidelity.CORRELATION, False, "none (analytic Nusselt surrogate)", True, None)
    assert p.notes == ("Analytic stand-in, NOT a Navier–Stokes wheel solve. Exists to make the "
                       "cooling-map/thermal/optimiser pipeline runnable without a solver. Use for trends; never "
                       "report as CFD h_c.")
    c = B.ReferenceRotorCFD(char_length_m=0.2, face_area_ratio=1.3, vent_h_multiplier=2.0, barrel_area_m2=0.03,
                            spin_augment=0.5)
    for pt in (B.WheelTunnelPoint(20.0), B.WheelTunnelPoint(7.0, rolling_radius_m=0.25, rho=1.1, mu_air=2e-5),
               B.WheelTunnelPoint(0.0), B.WheelTunnelPoint(0.05), B.WheelTunnelPoint(-3.0)):
        r = c.run_case(pt, "")
        hf = _h(pt, 0.2, 1.3, 0.5)
        assert r.h_face_w_m2k == pytest.approx(hf, rel=1e-12)
        assert r.h_vent_w_m2k == pytest.approx(2.0 * hf, rel=1e-12)
        tip = pt.speed_ms / max(pt.rolling_radius_m, 1e-6) * pt.rolling_radius_m
        md = pt.rho * 0.03 * max(pt.speed_ms, 0.0) * (1 + 0.2 * tip / max(pt.speed_ms, 0.1))
        assert r.mdot_barrel_kg_s == pytest.approx(md, rel=1e-12, abs=0.0)
        assert r.point is pt and r.converged is True and r.notes == "analytic surrogate value"
        assert r.provenance == c.provenance() and 0.0 <= r.wall_clock_s < 1.0
    assert B.ReferenceRotorCFD().run_case(B.WheelTunnelPoint(0.0)).h_face_w_m2k == 5.0
    assert B.ReferenceRotorCFD(face_area_ratio=2.0).run_case(B.WheelTunnelPoint(0.0)).h_face_w_m2k == 10.0


def test_the_reference_case_file_and_read_back(tmp_path):
    s = B.ReferenceRotorCFD()
    pt = B.WheelTunnelPoint(15.0)
    d = str(tmp_path / "a" / "b")
    path = s.write_case(pt, d)
    assert path == os.path.join(d, "reference_wheel_case.txt")
    assert open(path).read() == ("# reference analytic wheel-tunnel case — NOT a CFD deck\n"
                                 f"speed=15.0 m/s, wheel={15.0 / 0.22:.1f} rad/s\n")
    assert s.write_case(pt, d) == path                     # an existing directory is fine
    r = s.read_result(pt, d)
    assert r.h_face_w_m2k == pytest.approx(_h(pt)) and r.converged is True


def test_the_openfoam_stub(tmp_path):
    o = B.OpenFOAMRotorCFD()
    assert (o.name, o.turbulence_model, o.fidelity) == ("openfoam-rotating-wheel", "kOmegaSST",
                                                        B.RotorCFDFidelity.RANS_MRF)
    p = o.provenance(1200)
    assert p == B.RotorCFDProvenance(backend="openfoam-rotating-wheel", fidelity=B.RotorCFDFidelity.RANS_MRF,
                                     is_correlated=False, turbulence_model="kOmegaSST", rotating_wall=True,
                                     cell_count=1200,
                                     notes="Writes a rotating-wheel case; solve runs on the team's cluster.")
    assert o.provenance().cell_count is None
    o2 = B.OpenFOAMRotorCFD("kEpsilon", B.RotorCFDFidelity.DES)
    d = str(tmp_path / "of" / "x")
    pt = B.WheelTunnelPoint(18.0, rolling_radius_m=0.2)
    path = o2.write_case(pt, d)
    assert path == os.path.join(d, "wheel_tunnel.setup")
    assert open(path).read() == ("# OpenFOAM rotating-wheel wheel-tunnel case (stub)\n"
                                 "freestream_speed_ms 18.0\n"
                                 "wheel_omega_rad_s 90.0000\n"
                                 "rolling_radius_m 0.2\n"
                                 "turbulence kEpsilon\n"
                                 "rotating_wall true   # MRF zone or sliding mesh on the wheel\n"
                                 "post: areaAverage(h) on rotor_faces, rotor_vents; phi through barrel_inlet\n")
    assert o2.write_case(pt, d) == path
    d2 = str(tmp_path / "run")
    with pytest.raises(B.RotorSolverUnavailable) as e:
        o2.run_case(pt, d2)
    assert str(e.value) == ("OpenFOAM rotating-wheel solve not available in this environment. A valid case was "
                            f"written to '{d2}'; run it on a machine with OpenFOAM (simpleFoam + MRF or pimpleFoam + "
                            "sliding mesh), then call read_result().")
    assert os.path.isfile(os.path.join(d2, "wheel_tunnel.setup"))
    with pytest.raises(B.RotorSolverUnavailable) as e:
        o2.read_result(pt, d2)
    assert str(e.value) == ("No parsed OpenFOAM result found; run the case first, then parse areaAverage(h) and the "
                            "barrel mass flux into a ConvectiveResult.")


# ------------------------------------------------------------------ the convective map
def test_map_lookups_clamp_and_fold_negative_speed():
    m = B.ConvectiveMap(np.array([5.0, 10.0]), np.array([50.0, 70.0]), np.array([80.0, 120.0]), np.zeros(2))
    assert (m.provenance, m.synthesized, m.warnings) == (None, True, [])
    assert m.h_face_at([2.0, 7.5, -7.5, 30.0]).tolist() == [50.0, 60.0, 60.0, 70.0]
    assert m.h_vent_at([2.0, 7.5, -7.5, 30.0]).tolist() == [80.0, 100.0, 100.0, 120.0]


class Scripted:
    """A wheel-tunnel backend that answers per speed from a table."""
    name = "scripted"

    def __init__(self, table):
        self.table, self.dirs = table, []

    def run_case(self, pt, workdir):
        self.dirs.append(workdir)
        out = self.table(pt)
        if isinstance(out, Exception):
            raise out
        return out


def test_build_map_from_a_mixed_backend(tmp_path):
    prov = B.RotorCFDProvenance("s", B.RotorCFDFidelity.RANS_SLIDING, is_correlated=True)

    def table(pt):
        v = pt.speed_ms
        if v == 10.0:
            return B.RotorSolverUnavailable("down")
        if v == 15.0:
            return B.ConvectiveResult(pt, h_face_w_m2k=1.0, converged=False)
        if v == 20.0:
            return B.ConvectiveResult(pt, h_face_w_m2k=40.0, converged=True)            # no vent, no mdot, no prov
        return B.ConvectiveResult(pt, h_face_w_m2k=v, h_vent_w_m2k=2 * v, mdot_barrel_kg_s=v / 10,
                                  converged=True, provenance=prov)
    s = Scripted(table)
    m = B.build_convective_map(s, speeds_ms=[25.0, 0.0, 10.0, 15.0, 20.0, 25.0], workdir=str(tmp_path))
    assert m.speeds_ms.tolist() == [0.1, 20.0, 25.0]
    assert m.h_face.tolist() == [0.1, 40.0, 25.0] and m.h_vent.tolist() == [0.2, 40.0, 50.0]
    assert m.mdot_barrel.tolist() == [pytest.approx(0.01), 0.0, 2.5]
    assert m.warnings == ["speed 10 m/s: down", "speed 15 m/s: solver returned no usable h_c."]
    assert m.provenance is prov and m.synthesized is False and set(s.dirs) == {str(tmp_path)}
    unc = B.build_convective_map(Scripted(lambda pt: B.ConvectiveResult(
        pt, h_face_w_m2k=1.0, converged=True, provenance=B.RotorCFDProvenance("u", B.RotorCFDFidelity.DES))),
        speeds_ms=[5.0], workdir=str(tmp_path))
    assert unc.synthesized is True


def test_build_map_defaults_and_the_constant_fallback(tmp_path):
    s = Scripted(lambda pt: B.RotorSolverUnavailable("x"))
    m = B.build_convective_map(s)
    assert len(s.dirs) == 7 and os.path.basename(s.dirs[0]).startswith("kinematik_wheeltunnel_")
    assert os.path.isdir(s.dirs[0])
    assert m.speeds_ms.tolist() == [1.0, 100.0] and m.h_face.tolist() == [60.0, 60.0]
    assert m.h_vent.tolist() == [90.0, 90.0] and m.mdot_barrel.tolist() == [0.0, 0.0]
    assert m.warnings[-1] == ("No usable convective points; falling back to a constant, clearly-synthesized "
                              "h_c=60 W/m²K so downstream can run.")
    assert m.warnings[:7] == [f"speed {v} m/s: x" for v in (5, 10, 15, 20, 25, 30, 35)]
    assert m.provenance is None and m.synthesized is True
    ref = B.build_convective_map(speeds_ms=[12.0], rolling_radius_m=0.3, rho=1.0, workdir=str(tmp_path))
    pt = B.WheelTunnelPoint(12.0, rolling_radius_m=0.3, rho=1.0)
    assert ref.h_face.tolist() == [pytest.approx(_h(pt))]
    assert ref.mdot_barrel.tolist() == [pytest.approx(1.0 * 0.018 * 12.0 * 1.2)]
    assert ref.provenance.backend == "reference-rotor-cfd"


# ------------------------------------------------------------------ geometry
def test_rotor_geometry_closed_forms():
    g = B.RotorGeometry()
    assert (g.outer_radius_mm, g.inner_radius_mm, g.ring_thickness_mm, g.is_vented, g.vent_fraction, g.n_drillings,
            g.drill_radius_mm, g.hat_mass_kg, g.material, g.hat_material) == (
        110.0, 70.0, 7.0, False, 0.0, 0, 3.0, 0.35, B.RotorMaterial(), None)
    solid = math.pi * (0.11 ** 2 - 0.07 ** 2) * 0.007
    assert g.ring_solid_volume_m3() == pytest.approx(solid, rel=1e-12)
    assert g.ring_volume_m3() == pytest.approx(solid, rel=1e-12)
    assert g.ring_mass_kg() == pytest.approx(solid * 7150.0, rel=1e-12)
    assert g.total_mass_kg() == pytest.approx(solid * 7150.0 + 0.35, rel=1e-12)
    assert g.ring_face_area_m2() == pytest.approx(2 * math.pi * (0.11 ** 2 - 0.07 ** 2), rel=1e-12)
    assert g.vent_area_m2() == 0.0
    assert g.ring_thermal_capacity_j_per_k() == pytest.approx(solid * 7150.0 * 460.0, rel=1e-12)
    assert g.hat_thermal_capacity_j_per_k() == pytest.approx(0.35 * 460.0, rel=1e-12)
    al = B.ROTOR_MATERIALS["aluminium hat 6061"]
    v = B.RotorGeometry(outer_radius_mm=100.0, inner_radius_mm=60.0, ring_thickness_mm=8.0, is_vented=True,
                        vent_fraction=0.3, n_drillings=12, drill_radius_mm=2.5, hat_mass_kg=0.2, hat_material=al)
    sol = math.pi * (0.1 ** 2 - 0.06 ** 2) * 0.008
    drill = 12 * math.pi * 0.0025 ** 2 * 0.008
    assert v.ring_volume_m3() == pytest.approx(sol * 0.7 - drill, rel=1e-12)
    assert v.vent_area_m2() == pytest.approx(0.3 * 2 * math.pi * 0.08 * 0.008 * 4 + 12 * 2 * math.pi * 0.0025 * 0.008,
                                             rel=1e-12)
    assert v.hat_thermal_capacity_j_per_k() == pytest.approx(0.2 * 896.0, rel=1e-12)
    vent_only = B.RotorGeometry(is_vented=True, vent_fraction=0.2)
    assert vent_only.vent_area_m2() == pytest.approx(0.2 * 2 * math.pi * 0.09 * 0.007 * 4, rel=1e-12)
    drilled = B.RotorGeometry(n_drillings=4)
    assert drilled.vent_area_m2() == pytest.approx(4 * 2 * math.pi * 0.003 * 0.007, rel=1e-12)
    over = B.RotorGeometry(vent_fraction=0.9)
    assert over.ring_volume_m3() == pytest.approx(over.ring_solid_volume_m3() * 0.4, rel=1e-12)
    neg = B.RotorGeometry(vent_fraction=-0.5)
    assert neg.ring_volume_m3() == pytest.approx(neg.ring_solid_volume_m3(), rel=1e-12)
    holes = B.RotorGeometry(n_drillings=100000)
    assert holes.ring_volume_m3() == 1e-9


# ------------------------------------------------------------------ the braking-power trace
def _trace(speed, dist, lg, mass=280.0, g=9.81, vmin=0.5, bias=0.62, corner="front"):
    n = min(len(speed), len(dist), len(lg))
    t, q = np.zeros(n), np.zeros(n)
    frac = bias if corner == "front" else 1 - bias
    for i in range(1, n):
        ds = dist[i] - dist[i - 1]
        va = max(0.5 * (speed[i] + speed[i - 1]), vmin)
        t[i] = t[i - 1] + (ds / va if (np.isfinite(ds) and ds > 0) else 0.0)
        a = lg[i] * g if np.isfinite(lg[i]) else 0.0
        if a < 0:
            q[i] = mass * abs(a) * speed[i] * frac / 2
    return t, q, np.asarray(speed[:n], float)


LAP = SimpleNamespace(speed=[20.0, 18.0, 15.0, 15.0, 0.2, 0.3, 22.0],
                      distance=[0.0, 10.0, 20.0, 20.0, 25.0, np.nan, 40.0],
                      long_g=[0.0, -1.2, -0.8, 0.0, np.nan, 0.5, -0.3, 9.0])


def test_the_braking_power_trace():
    lp = SimpleNamespace(mass=300.0, g=9.8, V_MIN=1.0)
    t, q, v = B.braking_power_trace(LAP, lp, front_bias=0.6, corner="front")
    wt, wq, wv = _trace(LAP.speed, LAP.distance, LAP.long_g, 300.0, 9.8, 1.0, 0.6)
    assert t == pytest.approx(wt, rel=1e-12) and q == pytest.approx(wq, rel=1e-12) and v.tolist() == wv.tolist()
    t, q, v = B.braking_power_trace(LAP, lp, front_bias=0.6, corner="rear")
    assert q == pytest.approx(_trace(LAP.speed, LAP.distance, LAP.long_g, 300.0, 9.8, 1.0, 0.6, "rear")[1])
    t, q, v = B.braking_power_trace(LAP, SimpleNamespace())
    wt, wq, _ = _trace(LAP.speed, LAP.distance, LAP.long_g)
    assert t == pytest.approx(wt, rel=1e-12) and q == pytest.approx(wq, rel=1e-12)


def test_short_degenerate_and_broken_traces():
    seen = []
    out = B.braking_power_trace(SimpleNamespace(speed=[1.0], distance=[0.0, 1.0], long_g=[0, 0]), None,
                                warn=seen.append)
    assert [a.tolist() for a in out] == [[0.0], [0.0], [0.0]]
    assert seen == ["Lap trace too short to build a braking-power history."]
    seen.clear()
    t, q, v = B.braking_power_trace(SimpleNamespace(speed=[5.0, 5.0], distance=[3.0, 3.0], long_g=[0, -1]), None,
                                    warn=seen.append)
    assert t.tolist() == [0.0, 0.0] and seen == ["Recovered zero lap time; braking trace is degenerate."]
    assert q[1] == pytest.approx(280.0 * 9.81 * 5.0 * 0.62 / 2)
    seen.clear()
    out = B.braking_power_trace(SimpleNamespace(speed=[5.0, 5.0]), None, warn=seen.append)
    assert [a.tolist() for a in out] == [[0.0], [0.0], [0.0]]
    assert seen == ["braking_power_trace failed (AttributeError); returned empty."]
    assert B.braking_power_trace(SimpleNamespace(speed=[5.0, 5.0], distance=[0.0, 1.0], long_g=[0, 0]), None)[0][1] == 0.2


# ------------------------------------------------------------------ results
def _res(**kw):
    base = dict(time_s=np.zeros(2), ring_temp_c=np.zeros(2), hat_temp_c=np.zeros(2), caliper_temp_c=np.zeros(2),
                fluid_temp_c=np.zeros(2), speed_ms=np.zeros(2), q_in_w=np.zeros(2), peak_ring_c=500.0,
                peak_fluid_c=150.0, peak_caliper_c=120.0, pad_degradation_c=550.0, fluid_boil_c=216.0,
                synthesized=True, provenance="p")
    base.update(kw)
    return B.RotorThermalResult(**base)


def test_result_verdicts_and_the_failed_result():
    r = _res()
    assert (r.ring_fades, r.fluid_boils, r.passes, r.headroom_c(), r.warnings) == (False, False, True, (50.0, 66.0),
                                                                                 [])
    assert _res(peak_ring_c=550.0).ring_fades is True and _res(peak_ring_c=550.0).passes is False
    assert _res(peak_fluid_c=216.0).fluid_boils is True and _res(peak_fluid_c=216.0).passes is False
    f = B.RotorThermalResult.failed("boom")
    assert f.provenance == "FAILED" and f.warnings == ["boom"] and f.synthesized is True
    for a in (f.time_s, f.ring_temp_c, f.hat_temp_c, f.caliper_temp_c, f.fluid_temp_c, f.speed_ms, f.q_in_w):
        assert a.shape == (1,) and math.isnan(a[0])
    for x in (f.peak_ring_c, f.peak_fluid_c, f.peak_caliper_c, f.pad_degradation_c, f.fluid_boil_c):
        assert math.isnan(x)
    assert f.ring_temp_c is not f.time_s


def test_the_thermal_params_defaults():
    p = B.RotorThermalParams()
    assert (p.ambient_c, p.ambient_is_local, p.fric_to_rotor, p.k_ring_hat, p.k_hat_hub, p.k_pad_caliper,
            p.k_caliper_fluid, p.c_caliper_j_k, p.c_fluid_j_k, p.fluid_to_ambient_w_k, p.hub_node_c_j_k,
            p.enable_radiation, p.calibrated, p.fitted_to) == (35.0, False, 0.92, 14.0, 30.0, 2.2, 4.5, 850.0, 95.0,
                                                               1.5, 4000.0, True, False, "")


# ------------------------------------------------------------------ the lumped network, integrated here
def _network(geom, cmap, p, t, q, v, laps=1):
    t, v = (np.nan_to_num(np.asarray(x, float), nan=0.0) for x in (t, v))
    q = np.nan_to_num(np.asarray(q, float), nan=0.0, posinf=0.0, neginf=0.0)
    m = min(t.size, q.size, v.size)
    t, q, v = t[:m], q[:m], v[:m]
    if laps > 1:
        dt0 = t[1] - t[0]
        T, Q, V = [t], [q], [v]
        for _ in range(laps - 1):
            T.append(t - t[0] + T[-1][-1] + dt0)
            Q.append(q)
            V.append(v)
        t, q, v = np.concatenate(T), np.concatenate(Q), np.concatenate(V)
    Cr = max(geom.ring_mass_kg() * geom.material.cp, 1.0)
    Ch = max(geom.hat_mass_kg * (geom.hat_material or geom.material).cp, 1.0)
    Cc, Cf, Cu = max(p.c_caliper_j_k, 1.0), max(p.c_fluid_j_k, 1.0), max(p.hub_node_c_j_k, 1.0)
    Af = geom.ring_face_area_m2()
    Av = geom.vent_area_m2() if (geom.is_vented or geom.n_drillings) else 0.0
    eps = geom.material.emissivity if p.enable_radiation else 0.0
    hf_all = np.interp(np.abs(v), cmap.speeds_ms, cmap.h_face, left=cmap.h_face[0], right=cmap.h_face[-1])
    hv_all = np.interp(np.abs(v), cmap.speeds_ms, cmap.h_vent, left=cmap.h_vent[0], right=cmap.h_vent[-1])
    dt_s = max(min(0.5 * Cf / max(p.k_caliper_fluid, 1e-9),
                   0.5 * Cr / max(p.k_ring_hat + p.k_pad_caliper + hf_all.max() * Af + hv_all.max() * Av, 1e-9)),
               1e-4)
    Ta = p.ambient_c
    R = H = C = F = U = Ta
    out = np.zeros((4, t.size))
    out[:, 0] = Ta
    capped = False
    for s in range(1, t.size):
        dtm = max(t[s] - t[s - 1], 0.0)
        if dtm > 0:
            n = max(int(math.ceil(dtm / dt_s)), 1)
            if n > 8000:
                n, capped = 8000, True
            dt = dtm / n
            for _ in range(n):
                conv = (hf_all[s] * Af + hv_all[s] * Av) * (Ta - R)
                rad = eps * 5.670e-8 * Af * ((R + 273.15) ** 4 - (Ta + 273.15) ** 4) if eps > 0 else 0.0
                rh, rc = p.k_ring_hat * (R - H), p.k_pad_caliper * (R - C)
                hu = p.k_hat_hub * (H - U)
                cf, ca = p.k_caliper_fluid * (C - F), p.fluid_to_ambient_w_k * (C - Ta)
                R, H, C, F, U = (R + (p.fric_to_rotor * q[s] + conv - rad - rh - rc) * dt / Cr,
                                 H + (rh - hu) * dt / Ch, C + (rc - cf - ca) * dt / Cc, F + cf * dt / Cf,
                                 U + (hu - 8.0 * (U - Ta)) * dt / Cu)
            R, F = (Ta if math.isnan(R) else R), (Ta if math.isnan(F) else F)
        out[:, s] = (R, H, C, F)
    return t, q, v, out, capped


CMAP = B.ConvectiveMap(np.array([0.0, 30.0]), np.array([20.0, 140.0]), np.array([30.0, 210.0]), np.zeros(2),
                       warnings=["map note"])


def _check(model, t, q, v, laps=1):
    r = model.simulate(t, q, v, n_laps=laps)
    wt, wq, wv, T, capped = _network(model.geom, model.cmap, model.params, t, q, v, laps)
    assert r.time_s == pytest.approx(wt, rel=1e-12) and r.q_in_w.tolist() == wq.tolist()
    assert r.speed_ms.tolist() == wv.tolist()
    for got, want in zip((r.ring_temp_c, r.hat_temp_c, r.caliper_temp_c, r.fluid_temp_c), T):
        assert got == pytest.approx(want, rel=1e-10, abs=1e-9, nan_ok=True)
    peak = lambda a: max([a[0]] + [x for x in a[1:]], key=lambda x: -math.inf if math.isnan(x) else x)
    assert r.peak_ring_c == pytest.approx(peak(T[0]), rel=1e-10)
    assert r.peak_fluid_c == pytest.approx(peak(T[3]), rel=1e-10)
    assert r.peak_caliper_c == pytest.approx(peak(T[2]), rel=1e-10)
    return r, capped


def test_the_network_matches_its_equations():
    geom = B.RotorGeometry(is_vented=True, vent_fraction=0.25, n_drillings=6, hat_material=B.ROTOR_MATERIALS[
        "aluminium hat 6061"])
    p = B.RotorThermalParams(ambient_c=28.0, fric_to_rotor=0.9, k_ring_hat=12.0, k_hat_hub=25.0, k_pad_caliper=3.0,
                             k_caliper_fluid=5.0, c_caliper_j_k=700.0, c_fluid_j_k=80.0, fluid_to_ambient_w_k=2.0,
                             hub_node_c_j_k=3000.0)
    model = B.RotorThermalModel(geom, CMAP, params=p)
    t = [0.0, 0.5, 1.5, 1.5, 3.0, 4.0]
    q = [0.0, 30000.0, 45000.0, 50000.0, np.nan, 0.0]
    v = [25.0, 20.0, -12.0, 10.0, 15.0, np.inf]
    r, _ = _check(model, t, q, v)
    assert r.ring_temp_c[3] == r.ring_temp_c[2]                         # no time passes: state held
    assert r.peak_ring_c > 28.0 and r.synthesized is True and r.warnings == ["map note"]
    assert (r.pad_degradation_c, r.fluid_boil_c) == (550.0, 216.0)
    r3, _ = _check(model, t, q, v, laps=3)
    assert r3.time_s.size == 18 and r3.peak_ring_c > r.peak_ring_c
    assert _check(model, t, q, v, laps=0)[0].time_s.size == 6


def test_radiation_off_a_solid_disc_and_a_dry_fluid():
    geom = B.RotorGeometry(ring_thickness_mm=5.0, hat_mass_kg=0.0)
    p = B.RotorThermalParams(enable_radiation=False, calibrated=True, fitted_to="dyno 3")
    pad = B.PadSpec(degradation_c=400.0)
    cm = B.ConvectiveMap(np.array([1.0, 2.0]), np.array([50.0, 50.0]), np.array([70.0, 70.0]), np.zeros(2),
                         synthesized=False)
    model = B.RotorThermalModel(geom, cm, params=p, pad=pad, fluid=B.BRAKE_FLUIDS["Castrol SRF"],
                                use_wet_boil=False)
    r, _ = _check(model, [0.0, 1.0, 2.0, 3.0], [0.0, 60000.0, 60000.0, 0.0], [20.0, 15.0, 10.0, 12.0])
    assert (r.pad_degradation_c, r.fluid_boil_c, r.synthesized) == (400.0, 310.0, False)
    assert r.provenance == "Calibrated rotor thermal model (fitted to dyno 3); cooling map [no cooling-map provenance]."
    assert B.RotorThermalModel(geom, cm, params=p, fluid=B.BRAKE_FLUIDS["Castrol SRF"]).simulate(
        [0, 1], [0, 0], [1, 1]).fluid_boil_c == 270.0
    drilled = B.RotorThermalModel(B.RotorGeometry(n_drillings=8), CMAP)
    _check(drilled, [0.0, 1.0, 2.0], [0.0, 40000.0, 0.0], [20.0, 20.0, 20.0])


def test_the_sub_step_cap_and_the_failures():
    model = B.RotorThermalModel(B.RotorGeometry(hat_mass_kg=5.0), CMAP)
    r, capped = _check(model, [0.0, 0.1, 1.3e5], [0.0, 1000.0, 2000.0], [10.0, 10.0, 10.0])
    assert capped is True and np.all(np.isfinite(r.ring_temp_c)) and r.ring_temp_c[2] > 35.0
    assert r.warnings == ["Rotor thermal sub-stepping capped at 8000/step; results near that step are coarse.",
                          "map note"]
    model.simulate([0.0, 0.1, 1.3e5], [0.0, 1000.0, 0.0], [10.0, 10.0, 10.0])
    fine, capped = _check(model, [0.0, 0.1, 50.0], [0.0, 1000.0, 2000.0], [10.0, 10.0, 10.0])
    assert capped is False
    assert model.warnings == ["Rotor thermal sub-stepping capped at 8000/step; results near that step are coarse."]
    model._warn("")
    assert len(model.warnings) == 1
    short = model.simulate([0.0], [1.0, 2.0], [1.0, 2.0])
    assert short.provenance == "FAILED" and short.warnings == ["Braking trace too short to integrate."]
    broken = B.RotorThermalModel(B.RotorGeometry(), None).simulate([0, 1], [0, 0], [1, 1])
    assert broken.warnings == ["Rotor thermal integration failed (AttributeError): 'NoneType' object has no "
                               "attribute 'h_face_at'"]


def test_the_synthesized_provenance_text():
    prov = B.RotorCFDProvenance("ref", B.RotorCFDFidelity.CORRELATION, rotating_wall=True)
    cm = B.ConvectiveMap(np.array([1.0, 2.0]), np.ones(2), np.ones(2), np.zeros(2), provenance=prov, synthesized=False)
    m = B.RotorThermalModel(B.RotorGeometry(), cm)
    r = m.simulate([0, 1], [0, 0], [1, 1])
    assert r.synthesized is True
    assert r.provenance == ("SYNTHESIZED rotor temperatures — a transparent lumped network with representative "
                            "(uncalibrated) conductances and an uncorrelated cooling map [" + prov.status() + "]. "
                            "Trust the SHAPE and relative ranking; do not report absolute °C or a pass/fail margin as "
                            "fact until the masses, contact conductances and h_c are calibrated against a measured "
                            "rotor+caliper trace.")
    cal = B.RotorThermalModel(B.RotorGeometry(), cm, params=B.RotorThermalParams(calibrated=True, fitted_to="x"))
    assert cal.simulate([0, 1], [0, 0], [1, 1]).provenance == (
        f"Calibrated rotor thermal model (fitted to x); cooling map [{prov.status()}].")
    synth_map = B.ConvectiveMap(np.array([1.0, 2.0]), np.ones(2), np.ones(2), np.zeros(2))
    cal2 = B.RotorThermalModel(B.RotorGeometry(), synth_map, params=B.RotorThermalParams(calibrated=True))
    assert cal2.simulate([0, 1], [0, 0], [1, 1]).synthesized is True


# ------------------------------------------------------------------ the one-call simulation
def test_simulate_rotor_thermal_wires_everything(monkeypatch):
    lap = SimpleNamespace(speed=[20.0, 15.0, 10.0, 18.0], distance=[0.0, 15.0, 27.0, 40.0],
                          long_g=[0.0, -1.1, -0.9, 0.4], meta={"laps": 2})
    lp = SimpleNamespace(mass=290.0, g=9.81)
    geom = B.RotorGeometry(ring_thickness_mm=6.0)
    p = B.RotorThermalParams(ambient_c=30.0)
    r = B.simulate_rotor_thermal(lap, lp, geom=geom, cmap=CMAP, params=p, pad=B.PadSpec(degradation_c=500.0),
                                 fluid=B.BRAKE_FLUIDS["DOT 4 generic"], corner="rear", front_bias=0.7,
                                 use_wet_boil=False)
    t, q, v = _trace(lap.speed, lap.distance, lap.long_g, 290.0, 9.81, 0.5, 0.7, "rear")
    want = B.RotorThermalModel(geom, CMAP, params=p).simulate(t, q, v, n_laps=2)
    assert r.time_s.size == 8 and r.ring_temp_c == pytest.approx(want.ring_temp_c, rel=1e-12)
    assert (r.pad_degradation_c, r.fluid_boil_c) == (500.0, 230.0)
    one = B.simulate_rotor_thermal(SimpleNamespace(**{k: getattr(lap, k) for k in ("speed", "distance", "long_g")}),
                                   lp, cmap=CMAP)
    assert one.time_s.size == 4
    zero = B.simulate_rotor_thermal(SimpleNamespace(**vars(lap) | {"meta": {"laps": 0}}), lp, cmap=CMAP)
    assert zero.time_s.size == 4
    nometa = B.simulate_rotor_thermal(SimpleNamespace(**vars(lap) | {"meta": {}}), lp, cmap=CMAP)
    assert nometa.time_s.size == 4
    forced = B.simulate_rotor_thermal(lap, lp, cmap=CMAP, n_laps=3)
    assert forced.time_s.size == 12
    built = []
    real = B.build_convective_map
    monkeypatch.setattr(B, "build_convective_map", lambda s, rolling_radius_m: built.append((s, rolling_radius_m))
                        or real(s, rolling_radius_m=rolling_radius_m))
    solver = B.ReferenceRotorCFD()
    d = B.simulate_rotor_thermal(lap, lp, cfd_solver=solver, rolling_radius_m=0.25)
    assert built == [(solver, 0.25)] and d.provenance.startswith("SYNTHESIZED")
    short = B.simulate_rotor_thermal(SimpleNamespace(speed=[1.0], distance=[0.0], long_g=[0.0]), lp, cmap=CMAP)
    assert short.warnings == ["Braking trace too short to integrate.",
                              "Lap trace too short to build a braking-power history."]


# ------------------------------------------------------------------ the fluid check
def test_the_fluid_boil_check():
    f = B.fluid_boil_check(_res(peak_fluid_c=200.0))
    assert (f.peak_fluid_c, f.boil_c, f.fluid_name, f.using_wet, f.margin_c, f.boils) == (
        200.0, 216.0, "Motul RBF 600", True, 16.0, False)
    assert f.note == "Synthesized result — treat the margin as a relative indicator."
    assert f.summary() == ("Caliper fluid (Motul RBF 600, wet boil 216°C): peak piston-fluid 200°C -> ok (+16°C "
                           "margin). Synthesized result — treat the margin as a relative indicator.")
    d = B.fluid_boil_check(_res(peak_fluid_c=325.0, synthesized=False), fluid=B.BRAKE_FLUIDS["Motul RBF 660"],
                           using_wet=False)
    assert (d.boil_c, d.boils, d.margin_c) == (325.0, True, 0.0)
    assert d.note == ("Heat reaches the fluid by conduction ring->pad->caliper->piston; the fluid node is small, so "
                      "consecutive stops soak it faster than it sheds.")
    assert d.summary().startswith("Caliper fluid (Motul RBF 660, dry boil 325°C): peak piston-fluid 325°C -> BOILS "
                                  "(+0°C margin). Heat reaches")
    assert B.FluidBoilCheck(1.0, 2.0, "f", True, -3.0, True).note == ""


# ------------------------------------------------------------------ candidates and the optimiser
def _cand(geom, ring, fluid, passes, mass=None, synth=True, headroom=(10.0, 20.0)):
    r = _res(peak_ring_c=550.0 - headroom[0], peak_fluid_c=216.0 - headroom[1], synthesized=synth)
    return B.RotorCandidate(geom, r, geom.total_mass_kg() if mass is None else mass, ring, fluid, passes)


def test_the_candidate_label_and_the_study_summary():
    g = B.RotorGeometry(ring_thickness_mm=6.5, is_vented=True, vent_fraction=0.25, n_drillings=8, hat_mass_kg=0.3)
    c = _cand(g, 512.4, 180.6, True, mass=1.234)
    assert c.label() == ("t=6.5mm, vented, vent=25%, 8 holes, hat=0.30kg -> 1.23kg, ring 512°C, fluid 181°C, PASS")
    base = _cand(B.RotorGeometry(), 450.0, 150.0, False, mass=1.5)
    assert base.label().startswith("t=7.0mm, solid, vent=0%, 0 holes") and base.label().endswith("FAIL")
    none = B.RotorOptimization([c], [], base, True, "p")
    assert none.best is None and none.warnings == []
    assert none.summary() == ("Rotor mass-reduction study (1 candidates):\n  baseline: " + base.label() + "\n  -> NO "
                              "lighter candidate passes both limits; the baseline is already near the thermal limit. "
                              "Add cooling (vents, a duct, more h_c) before removing mass.")
    win = _cand(g, 500.0, 170.0, True, mass=1.2654, headroom=(37.6, 41.4))
    ok = B.RotorOptimization([c, win], [win, c], base, True, "p")
    assert ok.best is win
    assert ok.summary() == ("Rotor mass-reduction study (2 candidates):\n  baseline: " + base.label() + "\n  -> "
                            "lightest PASSING: " + win.label() + "\n     saves 235 g of unsprung rotating mass per "
                            "corner vs baseline, with ring peak still 38°C under pad fade and fluid 41°C under boil."
                            "\n     NOTE: synthesized thermal model — ranking is trustworthy, absolute margins are "
                            "provisional until calibrated.")
    cal = B.RotorOptimization([win], [win], base, False, "p")
    assert "NOTE" not in cal.summary() and cal.summary().endswith("under boil.")


def test_the_candidate_grid():
    al = B.ROTOR_MATERIALS["aluminium hat 6061"]
    base = B.RotorGeometry(outer_radius_mm=105.0, inner_radius_mm=72.0, ring_thickness_mm=6.0, is_vented=True,
                           vent_fraction=0.1, n_drillings=4, drill_radius_mm=2.0, hat_mass_kg=0.3,
                           material=B.ROTOR_MATERIALS["stainless 410"], hat_material=al)
    one = B.rotor_candidate_grid(base)
    assert one == [base]
    grid = B.rotor_candidate_grid(base, thickness_mm=[5, 6], vent_fraction=[0.0, 0.2], n_drillings=[0, 6.0],
                                  hat_mass_kg=[0.25], vented=[0, 1])
    assert len(grid) == 16
    first, last = grid[0], grid[-1]
    assert (first.ring_thickness_mm, first.vent_fraction, first.n_drillings, first.is_vented) == (5.0, 0.0, 0, False)
    assert (last.ring_thickness_mm, last.vent_fraction, last.n_drillings, last.is_vented) == (6.0, 0.2, 6, True)
    assert grid[1].is_vented is True and grid[2].n_drillings == 6 and grid[4].vent_fraction == 0.2
    for g in grid:
        assert (g.outer_radius_mm, g.inner_radius_mm, g.drill_radius_mm, g.hat_mass_kg, g.material,
                g.hat_material) == (105.0, 72.0, 2.0, 0.25, base.material, al)
        assert type(g.ring_thickness_mm) is float and type(g.n_drillings) is int and type(g.is_vented) is bool
        assert type(g.vent_fraction) is float and type(g.hat_mass_kg) is float


LAP2 = SimpleNamespace(speed=[25.0, 20.0, 12.0, 20.0, 26.0], distance=[0.0, 20.0, 36.0, 52.0, 75.0],
                       long_g=[0.0, -1.4, -1.2, 0.6, 0.5])


def test_the_optimiser(monkeypatch):
    lp = SimpleNamespace(mass=280.0, g=9.81)
    thick = B.RotorGeometry(ring_thickness_mm=8.0)
    thin = B.RotorGeometry(ring_thickness_mm=3.0)
    mid = B.RotorGeometry(ring_thickness_mm=5.0)
    pad = B.PadSpec(degradation_c=45.0)
    probe = {g.ring_thickness_mm: B.simulate_rotor_thermal(LAP2, lp, geom=g, cmap=CMAP, pad=pad).peak_ring_c
             for g in (thick, thin, mid)}
    limit = 0.5 * (probe[5.0] + probe[3.0])
    pad = B.PadSpec(degradation_c=limit)
    o = B.optimize_rotor(LAP2, lp, [thick, thin, mid], cmap=CMAP, pad=pad)
    assert [c.geom for c in o.candidates] == [thick, thin, mid]
    assert [c.geom for c in o.passing] == [mid, thick] and o.best.geom is mid
    assert o.baseline.geom is thick and o.synthesized is True
    assert o.provenance == o.baseline.result.provenance
    for c in o.candidates:
        assert c.mass_kg == c.geom.total_mass_kg() and c.peak_ring_c == c.result.peak_ring_c
        assert c.peak_fluid_c == c.result.peak_fluid_c and c.passes == c.result.passes
    o2 = B.optimize_rotor(LAP2, lp, [thin], cmap=CMAP, pad=pad, baseline=thick, n_laps=2)
    assert o2.baseline.geom is thick and o2.candidates[0].result.time_s.size == 10
    cal = B.ConvectiveMap(CMAP.speeds_ms, CMAP.h_face, CMAP.h_vent, CMAP.mdot_barrel, synthesized=False)
    p = B.RotorThermalParams(calibrated=True)
    o3 = B.optimize_rotor(LAP2, lp, [thin], cmap=cal, params=p, pad=pad)
    assert o3.synthesized is False
    o4 = B.optimize_rotor(LAP2, lp, [thin], cmap=cal, params=p, pad=pad, baseline=thick)
    assert o4.synthesized is False
    built = []
    real = B.build_convective_map
    monkeypatch.setattr(B, "build_convective_map", lambda s, rolling_radius_m: built.append(s)
                        or real(s, rolling_radius_m=rolling_radius_m))
    o5 = B.optimize_rotor(LAP2, lp, [thin, mid], pad=pad)
    assert built == [None] and o5.synthesized is True


def test_the_optimiser_merges_warnings_and_flags_synthesis(monkeypatch):
    lp = SimpleNamespace(mass=280.0, g=9.81)
    real = B.simulate_rotor_thermal
    state = {}

    def sim(lap, lap_params, geom=None, **kw):
        r = real(lap, lap_params, geom=geom, **kw)
        r.warnings = list(state.get(geom.ring_thickness_mm, []))
        r.synthesized = geom.ring_thickness_mm == 4.0
        return r
    monkeypatch.setattr(B, "simulate_rotor_thermal", sim)
    state.update({4.0: ["a", "b"], 5.0: ["b", "c"], 9.0: ["base"]})
    cands = [B.RotorGeometry(ring_thickness_mm=4.0), B.RotorGeometry(ring_thickness_mm=5.0)]
    o = B.optimize_rotor(LAP2, lp, cands, cmap=CMAP, baseline=B.RotorGeometry(ring_thickness_mm=9.0))
    assert o.warnings == ["a", "b", "c"] and o.synthesized is True
    o2 = B.optimize_rotor(LAP2, lp, cands[1:], cmap=CMAP, baseline=cands[0])
    assert o2.synthesized is True
    o3 = B.optimize_rotor(LAP2, lp, cands[1:], cmap=CMAP)
    assert o3.synthesized is False and o3.warnings == ["b", "c"]


def test_brake_edges_and_pass_through(monkeypatch):
    lp = SimpleNamespace(mass=280.0, g=9.81)
    # trace: braking on the very first sample, a slow segment under V_MIN, a light throttle, a short distance array
    lap = SimpleNamespace(speed=[10.0, 0.4, 0.6, 9.0, 8.0], distance=[0.0, 0.3, 0.8, 9.0],
                          long_g=[-1.0, -0.2, 0.05, -0.5, -0.1])
    for vmin in (None, 2.0):
        p = SimpleNamespace(mass=280.0, g=9.81) if vmin is None else SimpleNamespace(mass=280.0, g=9.81, V_MIN=vmin)
        t, q, v = B.braking_power_trace(lap, p)
        wt, wq, wv = _trace(lap.speed, lap.distance, lap.long_g, vmin=0.5 if vmin is None else vmin)
        assert t.tolist() == pytest.approx(wt.tolist(), rel=1e-12) and q.tolist() == pytest.approx(wq.tolist())
        assert q[0] == 0.0 and t.size == 4
    for d, warned in (([0.0, 0.0, 0.5], True), ([0.0, 0.0, 30.0], False), ([0.0, 30.0, 30.0], False)):
        seen = []
        B.braking_power_trace(SimpleNamespace(speed=[1.0, 1.0, 1.0], distance=d, long_g=[0, 0, 0]), lp,
                              warn=seen.append)
        assert (seen == ["Recovered zero lap time; braking trace is degenerate."]) is (d[-1] <= 0), d
        assert not warned or seen == [] or d[-1] == 0.0
    # the network: dirty inputs, ragged lengths, one-sample and offset laps stitched
    geom = B.RotorGeometry(ring_thickness_mm=6.0)
    m = B.RotorThermalModel(geom, CMAP)
    t = np.array([0.0, 1.0, np.nan, 3.0, 4.0])
    q = np.array([1e4, np.inf, np.nan, -np.inf, 5e3, 7.0])
    vv = np.array([20.0, np.nan, 15.0, 10.0])
    r = m.simulate(t, q, vv, n_laps=1)
    clean = m.simulate(np.array([0.0, 1.0, 0.0, 3.0]), np.array([1e4, 0.0, 0.0, 0.0]), np.array([20.0, 0.0, 15.0, 10.0]))
    assert r.ring_temp_c.tolist() == pytest.approx(clean.ring_temp_c.tolist(), rel=1e-12)
    assert m.simulate(np.array([0.0, 2.0]), np.array([8e3, 0.0]), np.array([20.0, 20.0])).time_s.size == 2
    one = m.simulate(np.array([5.0]), np.array([8e3]), np.array([20.0]), n_laps=2)
    two = m.simulate(np.array([5.0, 6.0]), np.array([8e3, 1e3]), np.array([20.0, 20.0]), n_laps=2)
    off = m.simulate(np.array([5.0, 6.0, 7.5]), np.array([8e3, 1e3, 0.0]), np.array([20.0, 18.0, 16.0]), n_laps=2)
    assert off.time_s.tolist() == pytest.approx([5.0, 6.0, 7.5, 8.5, 9.5, 11.0])
    assert two.time_s.tolist() == pytest.approx([5.0, 6.0, 7.0, 8.0]) and one.warnings == ["Braking trace too short to integrate."]
    default = m.simulate(np.array([0.0, 1.0]), np.array([8e3, 0.0]), np.array([20.0, 20.0]))
    assert default.time_s.size == 2
    # a solid disc with a single drilling has vent area
    assert B.RotorGeometry(is_vented=False, n_drillings=1).vent_area_m2() > 0.0
    assert B.RotorGeometry(is_vented=False, n_drillings=0).vent_area_m2() == 0.0
    # provenance line shows cells to one decimal of a million
    assert ", 1.1M cells" in B.RotorCFDProvenance("x", B.RotorCFDFidelity.DES, cell_count=1_050_001).status()
    # the surrogate below 0.1 m/s clamps its speed
    ref = B.ReferenceRotorCFD()
    assert ref.run_case(B.WheelTunnelPoint(speed_ms=0.5), None).h_face_w_m2k != \
        ref.run_case(B.WheelTunnelPoint(speed_ms=0.05), None).h_face_w_m2k
    # the map passes radius and density to every point
    pts = []
    B.build_convective_map(Scripted(lambda pt: pts.append(pt) or B.RotorSolverUnavailable("x")), speeds_ms=[5.0],
                           rolling_radius_m=0.27, rho=1.1)
    assert (pts[0].rolling_radius_m, pts[0].rho) == (0.27, 1.1)
    # simulate_rotor_thermal: the front corner, the wet boil point by default
    lap3 = SimpleNamespace(speed=[20.0, 15.0, 10.0, 18.0], distance=[0.0, 15.0, 27.0, 40.0], long_g=[0.0, -1.1, -0.9, 0.4])
    fr = B.simulate_rotor_thermal(lap3, lp, cmap=CMAP, corner="front", front_bias=0.7)
    t3, q3, v3 = _trace(lap3.speed, lap3.distance, lap3.long_g, bias=0.7, corner="front")
    want = B.RotorThermalModel(B.RotorGeometry(), CMAP).simulate(t3, q3, v3)
    assert fr.ring_temp_c.tolist() == pytest.approx(want.ring_temp_c.tolist(), rel=1e-12)
    assert fr.fluid_boil_c == B.RotorThermalModel(B.RotorGeometry(), CMAP, use_wet_boil=True).simulate(
        t3, q3, v3).fluid_boil_c
    assert fr.fluid_boil_c != B.simulate_rotor_thermal(lap3, lp, cmap=CMAP, use_wet_boil=False).fluid_boil_c
    # optimize_rotor hands every setting on, and builds its map from the given solver
    got = []
    real = B.simulate_rotor_thermal
    monkeypatch.setattr(B, "simulate_rotor_thermal", lambda *a, **k: got.append((a, k)) or real(*a, **k))
    fluid = B.BRAKE_FLUIDS["DOT 4 generic"]
    pad = B.PadSpec(degradation_c=400.0)
    params = B.RotorThermalParams(ambient_c=31.0)
    B.optimize_rotor(lap3, lp, [geom], cmap=CMAP, params=params, pad=pad, fluid=fluid, corner="rear",
                     front_bias=0.55, rolling_radius_m=0.26, use_wet_boil=False, n_laps=2)
    a, k = got[0]
    assert a == (lap3, lp) and k == dict(geom=geom, cmap=CMAP, params=params, pad=pad, fluid=fluid, corner="rear",
                                          front_bias=0.55, rolling_radius_m=0.26, use_wet_boil=False, n_laps=2)
    built = []
    realb = B.build_convective_map
    monkeypatch.setattr(B, "build_convective_map", lambda s, rolling_radius_m: built.append((s, rolling_radius_m))
                        or realb(s, rolling_radius_m=rolling_radius_m))
    solver = B.ReferenceRotorCFD()
    B.optimize_rotor(lap3, lp, [geom], cfd_solver=solver, rolling_radius_m=0.24)
    assert built == [(solver, 0.24)]
    empty = B.optimize_rotor(lap3, lp, [], cmap=CMAP, baseline=geom)
    assert empty.provenance == "no candidates"
