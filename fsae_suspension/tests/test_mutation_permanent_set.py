"""suspension/permanent_set.py against elastic-perfectly-plastic bending in closed form. A rectangle b x h under M
between first yield (sy b h^2 / 6) and the plastic moment (sy b h^2 / 4) keeps an elastic core of half-depth
c = sqrt(3 (h^2/4 - M / (sy b))), curvature sy / (E c), and unloads elastically, so its permanent curvature is
sy / (E c) - M / (E I). The cantilever's tip set integrates that along the member."""
import math

import numpy as np
import pytest

from suspension import permanent_set as ps

E, SY = 200000.0, 400.0
RECT = ps.Section("rect", b_mm=10.0, h_mm=6.0, n=4000)


def _kp_rect(M, b=10.0, h=6.0):
    My, Mp = SY * b * h * h / 6, SY * b * h * h / 4
    if abs(M) <= My:
        return 0.0
    c = math.sqrt(3 * (h * h / 4 - abs(M) / (SY * b)))
    return math.copysign(SY / (E * c) - abs(M) / (E * b * h ** 3 / 12), M)


def test_fibers_and_second_moment():
    y, A = RECT.fibers()
    assert len(y) == 4000 and y[0] == pytest.approx(-3.0 + 6.0 / 8000) and A.sum() == pytest.approx(60.0)
    assert RECT.I_mm4() == pytest.approx(10 * 6 ** 3 / 12, rel=1e-6)
    tube = ps.Section("tube", od_mm=25.4, t_mm=1.65, n=4000)
    y, A = tube.fibers()
    assert A.sum() == pytest.approx(math.pi * (12.7 ** 2 - 11.05 ** 2), rel=1e-3)
    assert tube.I_mm4() == pytest.approx(math.pi / 64 * (25.4 ** 4 - 22.1 ** 4), rel=1e-3)
    assert ps.Section("rect", b_mm=2.0, h_mm=4.0).n == 200
    with pytest.raises(ValueError) as e:
        ps.Section("angle").fibers()
    assert str(e.value) == "Section.kind must be 'rect' or 'tube'."


def test_the_permanent_curvature_of_a_rectangle():
    My, Mp = SY * 10 * 36 / 6, SY * 10 * 36 / 4
    for f in (0.5, 0.99):
        r = ps.permanent_curvature(RECT, E, SY, f * My)
        assert r["yielded"] is False and r["collapsed"] is False and abs(r["kappa_perm"]) < 1e-12
    for f in (1.1, 1.3, 1.45):
        for sgn in (1.0, -1.0):
            M = sgn * f * My
            r = ps.permanent_curvature(RECT, E, SY, M)
            assert r["yielded"] is True and r["kappa_perm"] == pytest.approx(_kp_rect(M), rel=2e-3)
    assert ps.permanent_curvature(RECT, E, SY, 1.01 * Mp) == {"collapsed": True, "kappa_perm": math.inf,
                                                              "yielded": True}
    assert ps.permanent_curvature(RECT, E, SY, 0.0)["kappa_perm"] == 0.0


def test_a_tube_yields_at_sy_i_over_r():
    tube = ps.Section("tube", od_mm=25.4, t_mm=1.65, n=2000)
    My = SY * tube.I_mm4() / 12.7
    assert ps.permanent_curvature(tube, E, SY, 0.98 * My)["yielded"] is False
    r = ps.permanent_curvature(tube, E, SY, 1.15 * My)
    assert r["yielded"] is True and r["kappa_perm"] > 0


def test_residual_stress_is_self_equilibrated_and_brings_yield_forward():
    y, A = RECT.fibers()
    for prof in (lambda v: 50.0, lambda v: 30.0 * v, lambda v: 20.0 + 10.0 * v):
        s = ps._residual(RECT, prof)
        assert abs(np.sum(s * A)) < 1e-9 and abs(np.sum(s * A * y)) < 1e-9 and np.abs(s).max() < 1e-9
    weld = lambda v: 300.0 * (1 - 12 * v * v / 36.0)                # tension at the middle, compression outside
    s = ps._residual(RECT, weld)
    assert abs(np.sum(s * A)) < 1e-6 * 300 * 60 and abs(np.sum(s * A * y)) < 1e-6
    assert ps._residual(RECT, None) == pytest.approx(np.zeros(4000))
    My = SY * 10 * 36 / 6
    clean = ps.permanent_curvature(RECT, E, SY, 0.8 * My)
    welded = ps.permanent_curvature(RECT, E, SY, 0.8 * My, residual=weld)
    assert clean["yielded"] is False and welded["yielded"] is True and welded["kappa_perm"] != 0.0


def test_the_cantilever_tip_set_integrates_the_permanent_curvature():
    L, F = 80.0, 2.6 * SY * 10 * 36 / 6 / 80.0 * 0.5               # root moment 1.3 My
    r = ps.cantilever_set(ps.Section("rect", b_mm=10.0, h_mm=6.0, n=1000), E, SY, L, F, n_seg=60)
    x = (np.arange(60) + 0.5) / 60 * L
    want = sum(_kp_rect(F * (L - xi)) * (L - xi) * L / 60 for xi in x)
    assert r["collapsed"] is False and r["yielded"] is True
    assert r["permanent_tip_mm"] == pytest.approx(want, rel=5e-3)
    small = ps.cantilever_set(RECT, E, SY, L, 0.5 * SY * 10 * 36 / 6 / L)
    assert small == {"collapsed": False, "permanent_tip_mm": 0.0, "yielded": False}
    big = ps.cantilever_set(RECT, E, SY, L, 1.2 * SY * 10 * 36 / 4 / L)
    assert big == {"collapsed": True, "permanent_tip_mm": math.inf, "yielded": True}
    coarse = ps.cantilever_set(ps.Section("rect", b_mm=10.0, h_mm=6.0, n=400), E, SY, L, F)
    x = (np.arange(40) + 0.5) / 40 * L
    assert coarse["permanent_tip_mm"] == pytest.approx(sum(_kp_rect(F * (L - xi)) * (L - xi) * L / 40 for xi in x),
                                                       rel=2e-2)


def test_a_moved_pickup_drifts_the_channels_by_the_jacobian():
    from suspension import elastokinematics as ek
    from suspension.kinematics import Hardpoints
    hp = Hardpoints.default()
    off = [0.3, -0.2, 0.5]
    J = ek.compliance_jacobian(hp, points=("tie_rod_inner",))
    d = ps.channel_drift(hp, "tie_rod_inner", off)
    assert d == pytest.approx(dict(zip(ek.CHANNELS, J @ np.array(off))))


def _kp_by_brentq(sec, M, residual):
    """The same elastic-plastic load and elastic unload solved with nested Brent root finds: the axial strain that
    gives zero force at a curvature, then the curvature that carries M, then the elastic unload to zero moment."""
    from scipy.optimize import brentq
    y, A = sec.fibers()
    sr = ps._residual(sec, residual)
    I = np.sum(A * y * y)

    def stress(e0, k):
        return np.clip(sr + E * (e0 + k * y), -SY, SY)

    def e0_for(k):
        return brentq(lambda e: np.sum(stress(e, k) * A), -0.05, 0.05, xtol=1e-16, rtol=1e-15)

    k = brentq(lambda k: np.sum(stress(e0_for(k), k) * A * y) - M, *sorted((0.0, math.copysign(0.01, M))), xtol=1e-16,
               rtol=1e-14)
    e0 = e0_for(k)
    epp = (sr + E * (e0 + k * y) - stress(e0, k)) / E
    return -np.sum((sr - E * epp) * A * y) / (E * I), e0


def test_a_weld_residual_shifts_the_neutral_axis_and_the_set_matches_an_independent_solve():
    weld = lambda v: 300.0 * (1 - 12 * v * v / 36.0)                # symmetric: yields one face before the other
    My = SY * 10 * 36 / 6
    for f in (0.8, 1.2, -1.2):
        want, e0 = _kp_by_brentq(RECT, f * My, weld)
        assert abs(e0) > 1e-6                                       # the axial strain is not zero here
        assert ps.permanent_curvature(RECT, E, SY, f * My, residual=weld)["kappa_perm"] == pytest.approx(want,
                                                                                                         rel=1e-6)
    tube = ps.Section("tube", od_mm=25.4, t_mm=1.65, n=2000)
    tw = lambda v: 250.0 * (1 - 2 * (v / 12.7) ** 2)
    M = 1.1 * SY * tube.I_mm4() / 12.7
    want, _ = _kp_by_brentq(tube, M, tw)
    assert ps.permanent_curvature(tube, E, SY, M, residual=tw)["kappa_perm"] == pytest.approx(want, rel=1e-6)


def test_the_cantilever_carries_the_residual_and_its_default_station_count():
    weld = lambda v: 300.0 * (1 - 12 * v * v / 36.0)
    sec = ps.Section("rect", b_mm=10.0, h_mm=6.0, n=60)
    L, F = 80.0, 0.8 * SY * 10 * 36 / 6 / 80.0                      # root moment 0.8 My: clean, it stays elastic
    assert ps.cantilever_set(sec, E, SY, L, F)["yielded"] is False
    r = ps.cantilever_set(sec, E, SY, L, F, residual=weld, n_seg=20)
    x = (np.arange(20) + 0.5) / 20 * L
    want = sum(ps.permanent_curvature(sec, E, SY, F * (L - xi), weld)["kappa_perm"] * (L - xi) * L / 20 for xi in x)
    assert r["yielded"] is True and r["permanent_tip_mm"] == pytest.approx(want, rel=1e-12) and want != 0.0
    G = 1.3 * SY * 10 * 36 / 6 / L
    assert ps.cantilever_set(sec, E, SY, L, G) == ps.cantilever_set(sec, E, SY, L, G, n_seg=40)


def test_tube_fibers_are_centred_and_a_thin_wall_keeps_its_area():
    tube = ps.Section("tube", od_mm=25.4, t_mm=1.65, n=200)
    y, A = tube.fibers()
    assert y == pytest.approx(-y[::-1], abs=1e-12) and y[0] == pytest.approx(-12.7 + 25.4 / 400)
    assert abs(np.sum(A * y)) < 1e-9
    small = ps.Section("tube", od_mm=10.0, t_mm=1.0, n=4000)        # its outer fibers have ro^2 - y^2 below 1 mm^2
    assert small.fibers()[1].sum() == pytest.approx(math.pi * (25.0 - 16.0), rel=2e-4)


def test_the_drift_takes_an_offset_in_any_shape():
    from suspension.kinematics import Hardpoints
    hp = Hardpoints.default()
    assert ps.channel_drift(hp, "tie_rod_inner", [[0.3, -0.2, 0.5]]) == ps.channel_drift(hp, "tie_rod_inner",
                                                                                        (0.3, -0.2, 0.5))
