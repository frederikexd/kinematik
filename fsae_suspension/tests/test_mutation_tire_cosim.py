"""suspension/tire_cosim.py: the provenance sentences, the exchange dataclasses' defaults, the reference backend's
relaxation state and friction-ellipse coupling rebuilt here on a stand-in lateral law, the external-backend stubs'
refusals word for word, and the factory's names."""
import math

import numpy as np
import pytest

from suspension import tire_cosim as tc
from suspension.tiremodel import relaxation_length

ABSENT = ["carcass_deflection_m", "contact_length_m", "contact_width_m", "pressure_distribution", "tread_temp_c",
          "carcass_temp_c", "gas_temp_c"]


def test_the_fidelities_and_the_provenance():
    assert [f.value for f in tc.TireFidelity] == ["handling", "structural", "thermal", "structural+thermal"]
    p = tc.TireProvenance("ftire", tc.TireFidelity.STRUCTURAL)
    assert (p.is_calibrated, p.parameter_file, p.fitted_to, p.notes) == (False, None, "", "")
    assert p.status() == ("ftire (structural), UNCALIBRATED — running on reference/default parameters, NOT a "
                          "measured-tyre file; treat absolute and structural/thermal outputs as placeholders")
    c = tc.TireProvenance("cdtire", tc.TireFidelity.THERMAL, is_calibrated=True, parameter_file="x.tir",
                          fitted_to="Calspan R25")
    assert c.status() == "cdtire (thermal), calibrated from x.tir, fitted to Calspan R25"
    assert tc.TireProvenance("b", tc.TireFidelity.HANDLING, is_calibrated=True).status() == (
        "b (handling), calibrated from a vendor parameter file")


def test_the_exchange_defaults():
    w = tc.WheelState()
    assert (w.alpha, w.kappa, w.gamma, w.Fz, w.omega, w.v_x, w.v_y, w.z_wheel, w.zdot_wheel, w.road_points,
            w.ambient_temp_c, w.track_temp_c, w.inflation_pressure_pa, w.dt) == (
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, None, 25.0, 30.0, None, 1.0e-3)
    o = tc.TireOutput()
    assert (o.Fx, o.Fy, o.Fz, o.Mz, o.Mx, o.My, o.carcass_deflection_m, o.contact_length_m, o.contact_width_m,
            o.pressure_distribution, o.effective_radius_m, o.tread_temp_c, o.carcass_temp_c, o.gas_temp_c,
            o.inflation_pressure_pa, o.synthesized) == (0.0,) * 6 + (None,) * 9 + ([],)
    assert o.is_structural() is False and o.is_thermal() is False
    assert tc.TireOutput(carcass_deflection_m=0.0).is_structural() is True
    assert tc.TireOutput(gas_temp_c=0.0).is_thermal() is True
    assert tc.TireOutput(tread_temp_c=np.zeros(3)).is_thermal() is True
    assert tc.TireOutput().synthesized is not tc.TireOutput().synthesized


class Lat:
    """Fy = -C alpha capped at +-D, D = mu Fz (1 - 0.1 |gamma|)."""

    def __init__(self, C=40000.0, mu=1.5):
        self.C, self.mu, self.seen = C, mu, []

    def fy(self, a, fz, g):
        self.seen.append((a, fz, g))
        return float(np.clip(-self.C * a, -self.mu * fz, self.mu * fz)) * (1 - 0.1 * abs(g))

    def peak_force(self, fz, g):
        return self.mu * fz * (1 - 0.1 * abs(g))


def _lag(prev, ws):
    sig = max(relaxation_length(max(ws.Fz, 1.0)), 1e-3)
    return ws.alpha + (prev - ws.alpha) * math.exp(-(max(abs(ws.v_x), 0.3) / sig) * max(ws.dt, 0.0))


def _forces(m, a_lag, ws):
    Fz, g = ws.Fz, ws.gamma
    fy = m.lateral.fy(a_lag, Fz, g)
    fmax = m.lateral.peak_force(Fz, abs(g))
    fxm = m.mu_x_ratio * fmax
    dem = min(max(ws.kappa, -1.0), 1.0) * fxm
    uy = min(abs(fy) / fmax, 1.0)
    fxa = fxm * max(1 - uy ** m.ell_ky, 0.0) ** (1 / m.ell_kx)
    Fx = min(max(dem, -fxa), fxa)
    ux = min(abs(Fx) / fxm, 1.0)
    fya = fmax * max(1 - ux ** m.ell_kx, 0.0) ** (1 / m.ell_ky)
    return Fx, min(max(fy, -fya), fya)


def test_the_reference_backend_relaxes_and_couples():
    m = tc.ReferenceTireModel(lateral=Lat(), mu_x_ratio=1.2, ell_kx=1.7, ell_ky=2.6)
    assert (m.mu_x_ratio, m.ell_kx, m.ell_ky, m.warnings()) == (1.2, 1.7, 2.6, [])
    lag = 0.0
    for ws in (tc.WheelState(alpha=0.03, kappa=0.05, gamma=0.02, Fz=1200.0, v_x=12.0, dt=0.004),
               tc.WheelState(alpha=0.06, kappa=-2.0, gamma=-0.03, Fz=900.0, v_x=-15.0, dt=0.01),
               tc.WheelState(alpha=-0.01, kappa=0.4, Fz=1500.0, v_x=0.1, dt=0.02)):
        lag = _lag(lag, ws)
        out = m.step(ws)
        assert m._alpha_lag == pytest.approx(lag, rel=1e-12)
        assert (out.Fx, out.Fy) == pytest.approx(_forces(m, lag, ws), rel=1e-12, abs=1e-9) and out.Fz == ws.Fz
        assert out.synthesized == ABSENT and out.synthesized is not m._absent
        assert m.lateral.seen[-1][1:] == (ws.Fz, ws.gamma)
    assert m.warnings() == [] and m.warnings() is not m._warnings


def test_light_loads_still_relax_and_return_no_force():
    m = tc.ReferenceTireModel(lateral=Lat())
    out = m.step(tc.WheelState(alpha=0.05, Fz=-20.0, v_x=10.0, dt=0.01))
    assert (out.Fx, out.Fy, out.Fz) == (0.0, 0.0, 0.0) and out.synthesized == ABSENT
    assert m._alpha_lag == pytest.approx(_lag(0.0, tc.WheelState(alpha=0.05, Fz=1.0, v_x=10.0, dt=0.01)))
    one = tc.ReferenceTireModel(lateral=Lat()).step(tc.WheelState(alpha=0.05, Fz=1.0, v_x=10.0))
    assert (one.Fx, one.Fy, one.Fz) == (0.0, 0.0, 1.0)
    just = tc.ReferenceTireModel(lateral=Lat()).step(tc.WheelState(alpha=0.05, Fz=1.5, v_x=10.0))
    assert just.Fy != 0.0
    back = tc.ReferenceTireModel(lateral=Lat())
    back.step(tc.WheelState(alpha=0.05, Fz=1200.0, v_x=10.0, dt=-1.0))
    assert back._alpha_lag == 0.0                                           # a negative step does not advance


def test_a_dead_lateral_law_and_a_spent_tire():
    class Dead(Lat):
        def peak_force(self, fz, g):
            return 0.0
    m = tc.ReferenceTireModel(lateral=Dead())
    out = m.step(tc.WheelState(alpha=0.02, kappa=0.3, Fz=1000.0, v_x=10.0, dt=1.0))
    assert out.Fx == 0.0 and out.Fy == pytest.approx(Dead().fy(m._alpha_lag, 1000.0, 0.0))
    nox = tc.ReferenceTireModel(lateral=Lat(C=1e9), mu_x_ratio=0.0)        # no longitudinal budget at all
    o = nox.step(tc.WheelState(alpha=0.5, kappa=1.0, Fz=1000.0, v_x=10.0, dt=10.0))
    assert o.Fx == 0.0 and o.Fy == pytest.approx(Lat(C=1e9).fy(nox._alpha_lag, 1000.0, 0.0))
    full = tc.ReferenceTireModel(lateral=Lat(C=1e9))
    o = full.step(tc.WheelState(alpha=0.5, kappa=1.0, Fz=1000.0, v_x=10.0, dt=10.0))
    assert o.Fx == 0.0 and abs(o.Fy) == pytest.approx(1500.0)           # lateral uses the whole budget
    x = tc.ReferenceTireModel(lateral=Lat())
    o = x.step(tc.WheelState(alpha=0.0, kappa=1.0, Fz=1000.0, v_x=10.0, dt=10.0))
    assert o.Fx == pytest.approx(1.05 * 1500.0) and o.Fy == 0.0


def test_failures_inside_a_step_are_flagged_once():
    class Broken(Lat):
        def fy(self, a, fz, g):
            raise ZeroDivisionError
    m = tc.ReferenceTireModel(lateral=Broken())
    for _ in range(2):
        out = m.step(tc.WheelState(alpha=0.02, Fz=900.0, v_x=10.0))
    assert (out.Fx, out.Fy, out.Fz) == (0.0, 0.0, 900.0) and out.synthesized == ABSENT
    assert m.warnings() == ["reference backend step failed (ZeroDivisionError); returned zero force for this sample."]
    bad = tc.ReferenceTireModel(lateral=Lat())
    bad.step(tc.WheelState(alpha=0.04, Fz=900.0, v_x="fast"))
    assert bad._alpha_lag == 0.04                                           # relaxation failed: snaps to the input
    neg = tc.ReferenceTireModel(lateral=Broken()).step(tc.WheelState(alpha=0.02, Fz=-5.0, v_x=10.0))
    assert neg.Fz == 0.0


def test_reset_and_provenance():
    m = tc.ReferenceTireModel(lateral=Lat())
    m._warnings = ["x"]
    m._alpha_lag = 0.3
    m.reset(tc.WheelState(alpha=0.07))
    assert m._alpha_lag == 0.07 and m.warnings() == []
    m.reset()
    assert m._alpha_lag == 0.0
    p = m.provenance()
    assert (p.backend, p.fidelity, p.is_calibrated, p.parameter_file) == ("reference-pacejka",
                                                                           tc.TireFidelity.HANDLING, False, None)
    assert p.notes == ("Pacejka MF5.2 wrapped in the co-sim contract. Handling-only: no structural or thermal channels "
                       "— those are None by design, not zero. Attach FTire/CDTire for those.")
    from suspension.tiremodel import default_tire
    d = tc.ReferenceTireModel()
    assert type(d.lateral).__name__ == type(default_tire()).__name__ and (d.mu_x_ratio, d.ell_kx, d.ell_ky) == (1.05, 2.0, 2.0)
    assert isinstance(d, tc.StructuralTireModel)
    m._warn("a"), m._warn("a"), m._warn("b")
    assert m.warnings() == ["a", "b"]


@pytest.mark.parametrize("cls, vendor, binding", [
    (tc.FTireModel, "ftire", "the cosin co-sim interface (FTire/cosim C/Fortran API, the cosin Python bindings, or an "
                             "FTire FMU) — a callable advancing the belt+thermal state one step from wheel-centre "
                             "motion and a 3D road"),
    (tc.CDTireModel, "cdtire", "the CDTire FMU (via an FMI runtime) or its C API — a callable advancing the shell + "
                               "thermal state one step from wheel-centre motion and a 3D road")])
def test_the_external_backends_refuse_until_bound(cls, vendor, binding):
    with pytest.raises(NotImplementedError) as e:
        cls("tyre.tir")
    assert str(e.value) == (
        f"{vendor} backend is a stub: no co-sim binding was provided. This module owns the integration SEAM; the "
        f"{vendor} solver itself is a licensed binary that is not bundled. To enable it, pass `binding=` — {binding} — "
        "and a `parameter_file` fitted to your tyre. Until then, use ReferenceTireModel (handling-grade, honest about "
        "what it can't compute).")
    b = cls("tyre.tir", library_path="/lib", binding=lambda *a: None, fitted_to="rig")
    assert (b.parameter_file, b.library_path, b._fitted_to, b.warnings()) == ("tyre.tir", "/lib", "rig", [])
    p = b.provenance()
    assert (p.backend, p.fidelity, p.is_calibrated, p.parameter_file, p.fitted_to) == (
        vendor, tc.TireFidelity.STRUCTURAL_THERMAL, True, "tyre.tir", "rig")
    assert cls("", binding=1).provenance().is_calibrated is False
    with pytest.raises(NotImplementedError) as e:
        b.reset()
    assert str(e.value) == f"Bind {vendor}'s state-initialise entry point in a subclass."
    with pytest.raises(NotImplementedError) as e:
        b.step(tc.WheelState())
    assert str(e.value) == (f"Map {vendor}'s per-step co-sim call into TireOutput in a subclass: forces/moments + "
                            "carcass_deflection_m, pressure_distribution, tread_temp_c, gas_temp_c, "
                            "inflation_pressure_pa.")
    b._binding = None
    assert b.reset() is None
    with pytest.raises(NotImplementedError) as e:
        b.step(tc.WheelState())
    assert str(e.value) == f"{vendor} backend not bound."
    assert b.warnings() is not b._warnings
    assert cls._fidelity == tc.TireFidelity.STRUCTURAL_THERMAL


def test_the_base_external_backend():
    assert tc._ExternalTireBackend._vendor == "external"
    assert tc._ExternalTireBackend._fidelity == tc.TireFidelity.STRUCTURAL_THERMAL
    with pytest.raises(NotImplementedError) as e:
        tc._ExternalTireBackend("x")
    assert "(FTire: cosin co-sim interface or FMU; CDTire: FMU / C API)" in str(e.value)


def test_the_factory(monkeypatch):
    lat = Lat()
    for k in ("reference", "pacejka", "default", "REFERENCE", None, ""):
        m = tc.make_tire_backend(k, lateral=lat)
        assert type(m) is tc.ReferenceTireModel and m.lateral is lat
    assert type(tc.make_tire_backend()) is tc.ReferenceTireModel
    from suspension import tire_thermal
    seen = {}

    class Th:
        def __init__(self, lateral=None, params=None, init_temp_c=None):
            seen.update(lateral=lateral, params=params, init_temp_c=init_temp_c)
    monkeypatch.setattr(tire_thermal, "ThermalTireModel", Th)
    for k in ("thermal", "lumped-thermal", "lumped_thermal"):
        assert type(tc.make_tire_backend(k, lateral=lat, params="P", init_temp_c=60.0)) is Th
        assert seen == dict(lateral=lat, params="P", init_temp_c=60.0)
    tc.make_tire_backend("thermal")
    assert seen == dict(lateral=None, params=None, init_temp_c=None)
    f = tc.make_tire_backend("ftire", parameter_file="a.tir", library_path="/x", binding=1, fitted_to="r")
    c = tc.make_tire_backend("cdtire", parameter_file="b.tir", library_path="/y", binding=1, fitted_to="s")
    assert type(f) is tc.FTireModel and (f.parameter_file, f.library_path, f._fitted_to) == ("a.tir", "/x", "r")
    assert type(c) is tc.CDTireModel and (c.parameter_file, c.library_path, c._fitted_to) == ("b.tir", "/y", "s")
    with pytest.raises(NotImplementedError):
        tc.make_tire_backend("ftire")
    with pytest.raises(NotImplementedError):
        tc.make_tire_backend("cdtire")
    bare = tc.make_tire_backend("ftire", binding=1)
    assert (bare.parameter_file, bare.library_path, bare._fitted_to) == ("", None, "")
    bare = tc.make_tire_backend("cdtire", binding=1)
    assert (bare.parameter_file, bare.library_path, bare._fitted_to) == ("", None, "")
    with pytest.raises(ValueError) as e:
        tc.make_tire_backend("mf62")
    assert str(e.value) == "Unknown tyre backend 'mf62'. Options: reference, thermal, ftire, cdtire."
    assert type(tc.default_structural_tire()) is tc.ReferenceTireModel


def test_combined_slip_clips_lateral_in_both_directions():
    m = tc.ReferenceTireModel(lateral=Lat(), mu_x_ratio=0.9, ell_kx=1.6, ell_ky=2.4)
    for alpha, kappa in ((0.4, 0.6), (-0.4, -0.6), (0.4, -3.0), (-0.3, 2.5), (0.05, 0.3)):
        m.reset()
        ws = tc.WheelState(alpha=alpha, kappa=kappa, Fz=1100.0, v_x=10.0, dt=10.0)
        out = m.step(ws)
        lag = m._alpha_lag
        fy = m.lateral.fy(lag, 1100.0, 0.0)
        fmax = m.lateral.peak_force(1100.0, 0.0)
        fxm = 0.9 * fmax
        fxa = fxm * max(1 - (abs(fy) / fmax) ** 2.4, 0.0) ** (1 / 1.6)
        Fx = min(max(min(max(kappa, -1.0), 1.0) * fxm, -fxa), fxa)
        fya = fmax * max(1 - (abs(Fx) / fxm) ** 1.6, 0.0) ** (1 / 2.4)
        assert out.Fx == pytest.approx(Fx, rel=1e-12, abs=1e-9)
        assert out.Fy == pytest.approx(min(max(fy, -fya), fya), rel=1e-12, abs=1e-9)
        assert (out.Fz, out.synthesized) == (1100.0, ABSENT)


def test_no_load_and_failed_samples_carry_the_absent_channels():
    m = tc.ReferenceTireModel(lateral=Lat())
    out = m.step(tc.WheelState(alpha=0.05, Fz=0.5, v_x=10.0, dt=0.01))
    assert (out.Fx, out.Fy, out.Fz, out.synthesized) == (0.0, 0.0, 0.5, ABSENT)

    class Broken(Lat):
        def peak_force(self, fz, g):
            raise RuntimeError
    b = tc.ReferenceTireModel(lateral=Broken())
    out = b.step(tc.WheelState(alpha=0.05, Fz=800.0, v_x=10.0, dt=10.0))
    assert (out.Fx, out.Fy, out.Fz, out.synthesized) == (0.0, 0.0, 800.0, ABSENT)
    assert out.synthesized is not b._absent
    assert b._alpha_lag == pytest.approx(0.05)


def test_external_backend_fitted_to_default_and_reference_provenance():
    f = tc.FTireModel("a.tir", binding=object())
    assert f.provenance().fitted_to == ""
    p = tc.ReferenceTireModel(lateral=Lat()).provenance()
    assert p.is_calibrated is False and p.parameter_file is None and p.fitted_to == ""


def test_characterized_saturated_lateral_and_default_backend():
    from types import SimpleNamespace as NS
    from tests._golden import check
    # camber thrust can push the pure lateral force past the |camber| peak: then Fx is shut off and Fy is clipped
    lat = NS(fy=lambda a, fz, g: -40000.0 * a, peak_force=lambda fz, g: 1.2 * fz)
    m = tc.ReferenceTireModel(lateral=lat, ell_kx=1.5, ell_ky=2.5)
    out = []
    for a in (0.08, -0.08, 0.06, -0.06):
        m._alpha_lag = a
        m._relax = lambda ws: None
        o = m.step(tc.WheelState(alpha=a, kappa=0.3, Fz=1000.0, v_x=10.0, dt=0.01))
        out.append([o.Fx, o.Fy])
    check("tire_cosim_saturated", {"forces": out, "default": type(tc.make_tire_backend()).__name__})
