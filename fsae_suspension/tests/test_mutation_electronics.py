"""Mutation-focused oracles for suspension/electronics.py.

Every expected number is computed here from the published closed forms
(IPC-2221, Onderdonk, IPC-2141) or from an independent geometry routine, not
by calling back into the module under test.
"""
import inspect
import math
import random
import warnings

import numpy as np
import pytest

from suspension import electronics as E
from suspension.electronics import (
    Aggressor, BoardCheckResult, BoardLedger, DiffPair, Trace, check_board,
    min_parallel_distance_mm, net_currents, parallel_run_length_mm,
    undeclared_loads, worst_case_currents, _seg_seg_dist,
)
from suspension.interfaces import (
    Finding, IntegrationLedger, Severity, SubsystemInterface,
)

RHO, ALPHA, OZ = 1.724e-8, 3.93e-3, 34.8
MIL = 39.3701


def _area_mil2(w, oz):
    return w * oz * OZ / 1000.0 * MIL ** 2


def _rise(I, w, oz, ext=True):
    k = 0.048 if ext else 0.024
    return min((I / (k * _area_mil2(w, oz) ** 0.725)) ** (1 / 0.44), 1063.0)


def _fuse(w, oz, t=10.0, ta=25.0):
    cmil = _area_mil2(w, oz) * 4 / math.pi
    return cmil * math.sqrt(math.log10((1083 - ta) / (234 + ta) + 1) / (33 * t))


def _res(w, oz, L, T=20.0):
    return RHO * (1 + ALPHA * (T - 20)) * (L * 1e-3) / (w * oz * OZ / 1000 * 1e-6)


# ---------------------------------------------------------------- Trace ---- #
def test_trace_geometry_and_resistance():
    t = Trace("a", "n", "o", width_mm=0.5, copper_oz=2.0, length_mm=80.0)
    assert t.thickness_mm == pytest.approx(0.0696, rel=1e-12)
    assert t.area_mm2 == pytest.approx(0.0348, rel=1e-12)
    assert t.area_mil2 == pytest.approx(_area_mil2(0.5, 2.0), rel=1e-12)
    assert t.area_cmil == pytest.approx(_area_mil2(0.5, 2.0) * 4 / math.pi, rel=1e-12)
    assert t.resistance_ohm() == pytest.approx(_res(0.5, 2, 80), rel=1e-12)
    assert t.resistance_ohm(85.0) == pytest.approx(_res(0.5, 2, 80, 85), rel=1e-12)
    assert t.voltage_drop_v(3.0) == pytest.approx(3 * _res(0.5, 2, 80), rel=1e-12)
    assert t.voltage_drop_v(3.0, temp_c=60.0) == pytest.approx(
        3 * _res(0.5, 2, 80, 60), rel=1e-12)
    assert t.power_dissipated_w(3.0) == pytest.approx(9 * _res(0.5, 2, 80), rel=1e-12)
    assert t.power_dissipated_w(3.0, temp_c=60.0) == pytest.approx(
        9 * _res(0.5, 2, 80, 60), rel=1e-12)


def test_trace_defaults():
    t = Trace("a", "n", "o")
    assert (t.feeds, t.width_mm, t.copper_oz, t.length_mm) == ("ecu", 0.25, 1.0, 100.0)
    assert t.is_external is True and t.is_estimate is True
    assert t.set_by == "" and t.notes == ""


def test_temp_rise_closed_form_and_guards():
    t = Trace("a", "n", "o", width_mm=0.3)
    assert t.temp_rise_c(2.0) == pytest.approx(_rise(2.0, 0.3, 1.0), rel=1e-12)
    assert t.temp_rise_c(0.5) == pytest.approx(_rise(0.5, 0.3, 1.0), rel=1e-12)
    ti = Trace("a", "n", "o", width_mm=0.3, is_external=False)
    assert ti.temp_rise_c(2.0) == pytest.approx(_rise(2.0, 0.3, 1.0, False), rel=1e-12)
    assert t.temp_rise_c(500.0) == 1063.0
    assert t.temp_rise_c(0.0) == 0.0
    assert t.temp_rise_c(-3.0) == 0.0
    assert Trace("a", "n", "o", width_mm=0.0).temp_rise_c(2.0) == 0.0
    tiny = Trace("a", "n", "o", width_mm=0.01)      # 0.54 sq mil < 1
    assert 0 < tiny.area_mil2 < 1
    assert tiny.temp_rise_c(0.001) == pytest.approx(_rise(0.001, 0.01, 1.0), rel=1e-12)


def test_fusing_current():
    t = Trace("a", "n", "o", width_mm=0.4, copper_oz=2.0)
    assert t.fusing_current_a() == pytest.approx(_fuse(0.4, 2.0), rel=1e-12)
    assert t.fusing_current_a(t_s=1.0, ambient_c=60.0) == pytest.approx(
        _fuse(0.4, 2.0, 1.0, 60.0), rel=1e-12)
    assert t.fusing_current_a(t_s=0.5) == pytest.approx(_fuse(0.4, 2.0, 0.5), rel=1e-12)
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        assert t.fusing_current_a(t_s=0.0) == math.inf
        assert t.fusing_current_a(t_s=-1.0) == math.inf
        assert Trace("a", "n", "o", width_mm=0.0).fusing_current_a() == math.inf
    tiny = Trace("a", "n", "o", width_mm=0.01)
    assert 0 < tiny.area_cmil < 1
    assert tiny.fusing_current_a() == pytest.approx(_fuse(0.01, 1.0), rel=1e-12)


def test_trace_dict_roundtrip_ignores_unknown():
    t = Trace("a", "n", "o", feeds="fan", width_mm=0.7, notes="x")
    d = t.as_dict()
    assert d["width_mm"] == 0.7 and d["notes"] == "x"
    d["bogus"] = 1
    assert Trace.from_dict(d) == t


# ------------------------------------------------------------- DiffPair ---- #
def _z0(w, s, h, oz=1.0, er=4.3):
    t = oz * OZ / 1000
    return 87 / math.sqrt(er + 1.41) * math.log(5.98 * h / (0.8 * w + t))


def test_diffpair_impedance():
    dp = DiffPair("p", "e", trace_w_mm=0.15, spacing_mm=0.3, height_mm=0.25,
                  copper_oz=0.5, eps_r=3.9)
    z0 = _z0(0.15, 0.3, 0.25, 0.5, 3.9)
    assert dp.thickness_mm == pytest.approx(0.0174, rel=1e-12)
    assert dp.single_ended_z0_ohm() == pytest.approx(z0, rel=1e-12)
    assert dp.differential_z0_ohm() == pytest.approx(
        2 * z0 * (1 - 0.48 * math.exp(-0.96 * 0.3 / 0.25)), rel=1e-12)


def test_diffpair_defaults_and_nan_guards():
    dp = DiffPair("p", "e")
    assert (dp.net_p, dp.net_n, dp.trace_w_mm, dp.spacing_mm, dp.height_mm,
            dp.copper_oz, dp.eps_r, dp.target_z0_ohm) == (
        "can_h", "can_l", 0.20, 0.20, 0.20, 1.0, 4.3, 120.0)
    assert dp.is_estimate is True and dp.path_mm == []
    assert dp.single_ended_z0_ohm() == pytest.approx(_z0(0.2, 0.2, 0.2), rel=1e-12)
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        assert math.isnan(DiffPair("p", "e", height_mm=0.0).single_ended_z0_ohm())
        assert math.isnan(DiffPair("p", "e", height_mm=-1.0).single_ended_z0_ohm())
        assert math.isnan(DiffPair("p", "e", height_mm=0.0).differential_z0_ohm())
        flat = DiffPair("p", "e", trace_w_mm=0.0, copper_oz=0.0)
        assert math.isnan(flat.single_ended_z0_ohm())
        assert math.isnan(flat.differential_z0_ohm())
    small_h = DiffPair("p", "e", height_mm=0.5, trace_w_mm=0.3)
    assert np.isfinite(small_h.differential_z0_ohm())


def test_polyline_and_dict_roundtrip():
    assert DiffPair("p", "e").as_polyline().shape == (0, 2)
    assert Aggressor("g", "pt").as_polyline().shape == (0, 2)
    dp = DiffPair("p", "e", path_mm=[(0, 0), (1, 2)])
    np.testing.assert_array_equal(dp.as_polyline(), [[0, 0], [1, 2]])
    d = dp.as_dict()
    d["path_mm"] = [[0, 0], [1, 2]]
    d["junk"] = 3
    back = DiffPair.from_dict(d)
    assert back.path_mm == [(0, 0), (1, 2)] and back == dp
    ag = Aggressor("g", "pt", path_mm=[(3, 4), (5, 6)])
    assert (ag.net, ag.sw_voltage_v, ag.edge_v_per_ns) == ("hv_inverter", 400.0, 5.0)
    np.testing.assert_array_equal(ag.as_polyline(), [[3, 4], [5, 6]])
    d = ag.as_dict()
    d["path_mm"] = [[3, 4], [5, 6]]
    d["junk"] = 1
    back = Aggressor.from_dict(d)
    assert back.path_mm == [(3, 4), (5, 6)] and back == ag
    # a non-list path passes through untouched
    assert DiffPair.from_dict(dict(name="p", owner_subsystem="e",
                                   path_mm=((0, 0), (1, 1)))).path_mm == ((0, 0), (1, 1))
    assert Aggressor.from_dict(dict(name="g", owner_subsystem="e",
                                    path_mm=((0, 0),))).path_mm == ((0, 0),)


# ------------------------------------------------------------- geometry ---- #
def _pt_seg(p, a, b):
    p, a, b = map(np.asarray, (p, a, b))
    d = b - a
    L = d @ d
    if L == 0:
        return float(np.linalg.norm(p - a))
    t = min(max((p - a) @ d / L, 0.0), 1.0)
    return float(np.linalg.norm(p - (a + t * d)))


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _ref_seg_seg(p1, p2, q1, q2):
    d1, d2 = _cross(q1, q2, p1), _cross(q1, q2, p2)
    d3, d4 = _cross(p1, p2, q1), _cross(p1, p2, q2)
    if d1 * d2 < 0 and d3 * d4 < 0:
        return 0.0
    return min(_pt_seg(p1, q1, q2), _pt_seg(p2, q1, q2),
               _pt_seg(q1, p1, p2), _pt_seg(q2, p1, p2))


def test_seg_seg_distance_random_against_reference():
    rng = random.Random(7)
    for _ in range(600):
        scale = rng.choice([0.3, 1.0, 5.0])
        pts = [(rng.uniform(-1, 1) * scale, rng.uniform(-1, 1) * scale) for _ in range(4)]
        assert _seg_seg_dist(*pts) == pytest.approx(_ref_seg_seg(*pts), abs=1e-9)


@pytest.mark.parametrize("p1,p2,q1,q2,want", [
    ((0, 0), (0, 0), (3, 4), (3, 4), 5.0),                 # two points
    ((0, 0), (0, 0), (-1, 1), (1, 1), 1.0),                # point vs segment
    ((0, 0), (0, 0), (2, 1), (4, 1), math.hypot(2, 1)),    # point beyond end
    ((-1, 2), (1, 2), (0, 0), (0, 0), 2.0),                # segment vs point
    ((-1, 2), (1, 2), (3, 2), (3, 2), 2.0),
    ((0, 0), (0.5, 0), (0, 0.3), (0.5, 0.3), 0.3),         # short parallel
    ((0, 0), (2, 0), (3, 1), (5, 1), math.hypot(1, 1)),    # parallel, offset
    ((0, 0), (2, 0), (-3, 1), (-1, 1), math.hypot(1, 1)),
    ((0, 0), (0.4, 0.4), (0.4, 0), (0, 0.4), 0.0),         # short crossing
    ((0, 0), (0.2, 0), (0.5, 0.1), (0.5, 0.9), math.hypot(0.3, 0.1)),
    ((0, 0), (0.2, 0), (-0.5, 0.1), (-0.5, 0.9), math.hypot(0.5, 0.1)),
    ((0.5, 0.1), (0.5, 0.9), (0, 0), (0.2, 0), math.hypot(0.3, 0.1)),
    ((0.5, 0.1), (0.5, 0.9), (0.6, 0.5), (0.9, 0.5), 0.1),
])
def test_seg_seg_cases(p1, p2, q1, q2, want):
    assert _seg_seg_dist(p1, p2, q1, q2) == pytest.approx(want, abs=1e-12)


def test_min_parallel_distance():
    a = np.array([[0.0, 0.0], [10.0, 0.0]])
    assert min_parallel_distance_mm(a, np.zeros((0, 2))) is None
    assert min_parallel_distance_mm(np.array([[0.0, 0.0]]), a) is None
    assert min_parallel_distance_mm(a, np.array([[0.0, 1.0]])) is None
    assert min_parallel_distance_mm(a, np.array([[0.0, 3.0], [10.0, 3.0]])) == 3.0
    b = np.array([[0.0, 5.0], [5.0, 5.0], [5.0, 0.7], [9.0, 0.7]])
    a3 = np.array([[0.0, 0.0], [4.0, 0.0], [10.0, 0.0]])
    assert min_parallel_distance_mm(a3, b) == pytest.approx(0.7)
    b2 = np.array([[0.0, 0.7], [5.0, 0.7], [5.0, 5.0], [9.0, 5.0]])
    assert min_parallel_distance_mm(a3, b2) == pytest.approx(0.7)


def _ref_run(a, b, within):
    total = 0.0
    for i in range(len(a) - 1):
        p0, p1 = np.asarray(a[i], float), np.asarray(a[i + 1], float)
        L = float(np.linalg.norm(p1 - p0))
        n = max(2, int(L / 0.5))
        for k in range(n):
            m = p0 + (p1 - p0) * (k + 0.5) / n
            d = min(_pt_seg(m, b[j], b[j + 1]) for j in range(len(b) - 1))
            if d <= within + 1e-9:
                total += L / n
    return total


def test_parallel_run_length_closed_cases():
    a = np.array([[0.0, 0.0], [10.0, 0.0]])
    assert parallel_run_length_mm(a, np.zeros((0, 2)), 1.0) == 0.0
    assert parallel_run_length_mm(np.array([[0.0, 0.0]]), a, 1.0) == 0.0
    b = np.array([[0.0, -1.0], [10.0, -1.0]])
    assert parallel_run_length_mm(a, b, 2.0) == pytest.approx(10.0)
    assert parallel_run_length_mm(a, b, 0.5) == 0.0
    half = np.array([[0.0, -1.0], [5.0, -1.0]])
    assert parallel_run_length_mm(a, half, 1.5) == pytest.approx(6.0)
    # a degenerate (zero-length) first segment contributes nothing
    a0 = np.array([[0.0, 0.0], [0.0, 0.0], [10.0, 0.0]])
    assert parallel_run_length_mm(a0, half, 1.5) == pytest.approx(6.0)


@pytest.mark.parametrize("seg_len", [0.6, 0.9, 1.3, 2.2, 3.7])
def test_parallel_run_length_substeps(seg_len):
    a = np.array([[0.0, 0.0], [seg_len, 0.0]])
    b = np.array([[0.0, -0.2], [0.33 * seg_len, -0.2]])
    for within in (0.25, 0.3, 0.45):
        got = parallel_run_length_mm(a, b, within)
        assert got == pytest.approx(_ref_run(a, b, within), abs=1e-12)
    bent = np.array([[0.0, 0.0], [seg_len, 0.0], [seg_len, 2.0]])
    b2 = np.array([[seg_len - 0.3, 0.3], [seg_len - 0.3, 0.9], [-1.0, 0.9]])
    assert parallel_run_length_mm(bent, b2, 0.4) == pytest.approx(
        _ref_run(bent, b2, 0.4), abs=1e-12)


# --------------------------------------------------------- check_traces ---- #
def _board(**kw):
    b = BoardLedger(**kw)
    return b


def _one(findings, check):
    got = [f for f in findings if f.check == check]
    assert len(got) == 1, [f.check for f in findings]
    return got[0]


def test_board_defaults():
    b = BoardLedger()
    assert (b.rail_nominal_v, b.ecu_brownout_v, b.ambient_c, b.ambient_is_local,
            b.max_trace_temp_c, b.fuse_safety_factor, b.coupling_warn_mm,
            b.coupling_fail_mm) == (5.0, 4.5, 40.0, False, 105.0, 2.0, 2.0, 0.5)
    assert b.check_traces() == [] and b.check_signal_integrity() == []


def test_missing_current_messages():
    b = _board()
    b.set_trace(Trace("t1", "fan_pwr", "electrics", feeds="cooling", width_mm=0.3,
                      copper_oz=2.0))
    b.set_trace(Trace("t2", "brk", "electrics", feeds="brakes"))
    b.set_trace(Trace("t3", "x", "electrics", feeds="aero"))
    out = b.check_traces({}, undeclared={"t1": ["fanB", "fanA", "fanA"], "t2": ["brakes"]})
    assert [f.severity for f in out] == [Severity.MISSING] * 3 + [Severity.OK]
    f1, f2, f3, ok = out
    assert f1.message == (
        "Trace 't1' (electrics->cooling) has selected load(s) [fanA, fanB], but "
        "those subsystems have no declared peak current — cannot check heating or "
        "drop. Declare each one's peak current in the Integration tab.")
    assert f1.check == "trace-current"
    assert f1.subsystems == ["cooling", "electrics"]
    assert f1.detail == dict(trace="t1", width_mm=0.3, copper_oz=2.0,
                             undeclared_loads=["fanA", "fanB"])
    assert "[brakes], but that subsystem has no declared" in f2.message
    assert f3.message == (
        "Trace 't3' (electrics->aero) has no declared worst-case current — cannot "
        "check heating or drop. Pick its load(s) in the scenario above, and declare "
        "each subsystem's peak current in the Integration tab.")
    assert f3.detail["undeclared_loads"] == []
    assert ok.check == "trace-survival" and ok.subsystems == ["electrics", "ecu"]
    assert ok.message == (
        "All checked traces survive their worst-case current with margin to "
        "fusing, the 105 °C ceiling, and the ECU brown-out threshold.")
    # default `undeclared=None` path
    assert b.check_traces({})[0].detail["undeclared_loads"] == []


def test_fusing_fail_and_boundary():
    tr = Trace("t", "fan_pwr", "electrics", feeds="cooling", width_mm=0.1,
               is_estimate=True)
    b = _board(ambient_c=30.0, max_trace_temp_c=5000.0, rail_nominal_v=1e6)
    b.set_trace(tr)
    i_f = _fuse(0.1, 1.0, ta=30.0)
    for I in (i_f * 1.5, tr.fusing_current_a(ambient_c=30.0)):
        f = _one(b.check_traces({"t": I}), "trace-fusing")
        assert f.severity == Severity.FAIL
        assert f.subsystems == ["cooling", "electrics"]
        assert f.message == (f"Trace 't' carries {I:.1f} A but fuses (melts) at "
                             f"{i_f:.1f} A — the copper physically fails under "
                             f"this load (estimated geometry).")
        assert f.detail == dict(trace="t", current_a=I, fusing_a=pytest.approx(i_f),
                                estimate=True)


def test_fusing_warn_and_boundary():
    tr = Trace("t", "fan_pwr", "electrics", feeds="cooling", width_mm=0.1,
               is_estimate=False)
    b = _board(max_trace_temp_c=5000.0, rail_nominal_v=1e6, fuse_safety_factor=4.0)
    b.set_trace(tr)
    i_f = tr.fusing_current_a(ambient_c=40.0)
    for I in (i_f / 2.0, i_f / 4.0):
        f = _one(b.check_traces({"t": I}), "trace-fusing")
        assert f.severity == Severity.WARN
        assert f.message == (f"Trace 't' at {I:.1f} A is within 4x of its "
                             f"{i_f:.1f} A fusing current — thin margin to melt.")
        assert f.detail == dict(trace="t", current_a=I, fusing_a=i_f,
                                safety_factor=4.0, estimate=False)
    out = b.check_traces({"t": i_f / 4.0 * 0.999})
    assert out == []


def _heat_board(max_c, ext=True):
    tr = Trace("t", "fan_pwr", "electrics", feeds="cooling", width_mm=0.5,
               is_estimate=False, is_external=ext)
    b = _board(ambient_c=35.0, max_trace_temp_c=max_c, rail_nominal_v=1e6)
    b.set_trace(tr)
    return b, tr


def test_heating_fail_message_and_boundary():
    I = 3.0
    dT = _rise(I, 0.5, 1.0)
    b, tr = _heat_board(35.0 + dT - 1.0)
    f = _one(b.check_traces({"t": I}), "trace-heating")
    assert f.severity == Severity.FAIL
    T = 35.0 + dT
    assert f.message == (f"Trace 't' runs to {T:.0f} °C at 3.0 A (rise {dT:.0f} °C "
                         f"over 35 °C ambient), past the {35 + dT - 1:.0f} °C "
                         f"ceiling — widen it.")
    assert f.detail == dict(trace="t", current_a=I, temp_c=pytest.approx(T),
                            rise_c=pytest.approx(dT), ceiling_c=35.0 + dT - 1.0,
                            estimate=False)
    exact = 35.0 + tr.temp_rise_c(I)
    b2, _ = _heat_board(exact)
    f = _one(b2.check_traces({"t": I}), "trace-heating")
    assert f.severity == Severity.WARN
    assert f.message == (f"Trace 't' reaches {exact:.0f} °C at 3.0 A — within 20 °C "
                         f"of the {exact:.0f} °C ceiling.")
    assert f.detail == dict(trace="t", current_a=I, temp_c=exact, ceiling_c=exact,
                            estimate=False)


def test_heating_warn_boundary():
    I = 3.0
    tr = Trace("t", "n", "o", width_mm=0.5)
    T = 35.0 + tr.temp_rise_c(I)
    m = T + 20.0
    while m - 20.0 > T:
        m = np.nextafter(m, -np.inf)
    while m - 20.0 < T:
        m = np.nextafter(m, np.inf)
    assert m - 20.0 == T
    b, _ = _heat_board(float(m))
    assert not [f for f in b.check_traces({"t": I}) if f.check == "trace-heating"]
    b, _ = _heat_board(T + 19.0)
    assert _one(b.check_traces({"t": I}), "trace-heating").severity == Severity.WARN
    b, _ = _heat_board(T + 21.0)
    assert not [f for f in b.check_traces({"t": I}) if f.check == "trace-heating"]


def test_vdrop_warning_uses_hot_resistance_and_boundary():
    I = 2.0
    tr = Trace("t", "fan_pwr", "electrics", feeds="cooling", width_mm=2.0,
               length_mm=2000.0, is_estimate=False)
    b = _board(ambient_c=25.0, max_trace_temp_c=1e4, fuse_safety_factor=1.0)
    b.set_trace(tr)
    T = 25.0 + _rise(I, 2.0, 1.0)
    vd = I * _res(2.0, 1.0, 2000.0, T)
    assert vd > 0.25
    f = _one(b.check_traces({"t": I}), "trace-vdrop")
    assert f.severity == Severity.WARN
    assert f.subsystems == ["cooling", "electrics"]
    assert f.message == (f"Trace 't' drops {vd * 1000:.0f} mV at 2.0 A "
                         f"(> 5 % of a 5.0 V rail).")
    assert f.detail == dict(trace="t", vdrop_v=pytest.approx(vd, rel=1e-12),
                            estimate=False)
    exact = tr.voltage_drop_v(I, temp_c=25.0 + tr.temp_rise_c(I))
    rail = exact / 0.05
    while 0.05 * rail > exact:
        rail = np.nextafter(rail, -np.inf)
    while 0.05 * rail < exact:
        rail = np.nextafter(rail, np.inf)
    assert 0.05 * rail == exact
    b.rail_nominal_v = float(rail)
    assert b.check_traces({"t": I}) == []
    b.rail_nominal_v = exact / 0.05 * 0.9
    assert _one(b.check_traces({"t": I}), "trace-vdrop")


@pytest.mark.parametrize("feeds,net", [("ecu", "x"), ("dash", "ECU_PWR"),
                                       ("dash", "mcu_pwr"), ("dash", "lv_rail")])
def test_ecu_rail_ok(feeds, net):
    tr = Trace("t", net, "electrics", feeds=feeds, width_mm=1.0, length_mm=50.0)
    b = _board()
    b.set_trace(tr)
    I = 1.0
    T = 40.0 + _rise(I, 1.0, 1.0)
    dlv = 5.0 - I * _res(1.0, 1.0, 50.0, T)
    out = b.check_traces({"t": I})
    f = _one(out, "ecu-brownout")
    assert f.severity == Severity.OK
    assert f.subsystems == sorted({"electrics", "ecu"})
    assert f.message == (f"Trace 't' holds the rail at {dlv:.2f} V under 1.0 A "
                         f"(>4.50 V brown-out).")
    assert f.detail == dict(trace="t", delivered_v=pytest.approx(dlv, rel=1e-12))
    assert out[-1].check == "trace-survival"


def test_ecu_brownout_fail_and_boundary():
    tr = Trace("t", "x", "chassis", feeds="ecu", width_mm=2.0, length_mm=2000.0,
               is_estimate=True)
    b = _board(max_trace_temp_c=1e4, fuse_safety_factor=1.0, rail_nominal_v=3.3,
               ecu_brownout_v=3.0)
    b.set_trace(tr)
    I = 2.0
    T = 40.0 + _rise(I, 2.0, 1.0)
    vd = I * _res(2.0, 1.0, 2000.0, T)
    out = b.check_traces({"t": I})
    f = _one(out, "ecu-brownout")
    assert f.severity == Severity.FAIL
    assert f.subsystems == ["chassis", "ecu"]
    assert f.message == (
        f"Trace 't' drops {vd * 1000:.0f} mV at 2.0 A, pulling the 3.3 V rail to "
        f"{3.3 - vd:.2f} V — below the 3.00 V ECU brown-out threshold. The main "
        f"MCU would reset (estimated geometry).")
    assert f.detail == dict(trace="t", current_a=I, vdrop_v=pytest.approx(vd),
                            delivered_v=pytest.approx(3.3 - vd), brownout_v=3.0,
                            estimate=True)
    assert out[-1].check != "trace-survival"
    delivered = 3.3 - tr.voltage_drop_v(I, temp_c=40.0 + tr.temp_rise_c(I))
    b.ecu_brownout_v = delivered
    assert _one(b.check_traces({"t": I}), "ecu-brownout").severity == Severity.OK


def test_ecu_fused_is_open_circuit():
    tr = Trace("t", "x", "chassis", feeds="ecu", width_mm=0.1, is_estimate=False)
    b = _board(max_trace_temp_c=1e5)
    b.set_trace(tr)
    I = tr.fusing_current_a(ambient_c=40.0)
    out = b.check_traces({"t": I})
    f = _one(out, "ecu-brownout")
    assert f.severity == Severity.FAIL
    assert f.message == (f"Trace 't' fuses before steady state at {I:.1f} A, so the "
                         f"ECU rail is lost entirely (open circuit), not merely "
                         f"browned out.")
    assert f.subsystems == ["chassis", "ecu"]
    assert f.detail == dict(trace="t", current_a=I, fused=True, brownout_v=4.5,
                            estimate=False)
    f = _one(b.check_traces({"t": I * 0.999}), "ecu-brownout")
    assert "fuses before" not in f.message


def test_survival_ok_suppressed_by_warn_only():
    tr = Trace("t", "fan_pwr", "electrics", feeds="cooling", width_mm=0.1)
    b = _board(max_trace_temp_c=1e5, rail_nominal_v=1e6)
    b.set_trace(tr)
    out = b.check_traces({"t": tr.fusing_current_a(ambient_c=40.0) / 2.0})
    assert [f.severity for f in out] == [Severity.WARN]


# ---------------------------------------------------- signal integrity ---- #
def _si_board(**kw):
    b = BoardLedger(**kw)
    return b


def test_impedance_findings():
    tag = " (analytic estimate, not field-solved)"
    b = _si_board()
    dp = DiffPair("can", "electrics", path_mm=[(0, 0), (10, 0)])
    z = dp.differential_z0_ohm()
    dp.target_z0_ohm = z * 1.1
    b.set_pair(dp)
    f = _one(b.check_signal_integrity(), "diffpair-impedance")
    assert f.severity == Severity.OK
    assert f.subsystems == ["electrics"]
    assert f.message == (f"Pair 'can' estimates {z:.0f} Ω differential, within 15 % "
                         f"of the {z * 1.1:.0f} Ω target{tag}.")
    assert f.detail == dict(pair="can", z_diff_ohm=z, target_ohm=z * 1.1, estimate=True)
    dp.target_z0_ohm = z * 1.25
    f = _one(b.check_signal_integrity(), "diffpair-impedance")
    assert f.severity == Severity.WARN
    err = abs(z - z * 1.25)
    assert f.message == (f"Pair 'can' estimates {z:.0f} Ω differential vs a "
                         f"{z * 1.25:.0f} Ω target ({err / (z * 1.25) * 100:.0f} % off)"
                         f"{tag} — retune width/spacing or confirm on a field solver.")
    assert f.detail == dict(pair="can", z_diff_ohm=z, target_ohm=z * 1.25, estimate=True)
    dp.target_z0_ohm = z * 0.85
    assert _one(b.check_signal_integrity(), "diffpair-impedance").severity == Severity.WARN
    dp.target_z0_ohm = z * 0.9
    assert _one(b.check_signal_integrity(), "diffpair-impedance").severity == Severity.OK
    dp.height_mm = 0.0
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        assert b.check_signal_integrity() == []


def test_route_missing_and_two_point_route():
    b = _si_board()
    dp = DiffPair("can", "electrics", path_mm=[(0, 0)])
    b.set_pair(dp)
    b.set_aggressor(Aggressor("inv", "powertrain", path_mm=[(0, 1), (5, 1)]))
    out = b.check_signal_integrity()
    f = _one(out, "diffpair-route")
    assert f.severity == Severity.MISSING
    assert f.message == ("Pair 'can' has no routed centreline — cannot check its "
                         "distance to noisy nets yet.")
    assert f.subsystems == ["electrics"]
    assert not [g for g in out if g.check == "si-crosstalk"]
    dp.path_mm = [(0, 0), (5, 0)]
    out = b.check_signal_integrity()
    assert not [g for g in out if g.check == "diffpair-route"]
    assert _one(out, "si-crosstalk").severity == Severity.WARN


def test_crosstalk_fail_warn_ok_and_boundaries():
    b = _si_board()
    b.set_pair(DiffPair("can", "electrics", path_mm=[(0, 0), (10, 0)], is_estimate=False))
    ag = Aggressor("inv", "powertrain", net="hv_u", sw_voltage_v=600.0,
                   edge_v_per_ns=12.0, path_mm=[(0, -0.5), (10, -0.5)], is_estimate=False)
    b.set_aggressor(ag)
    b.set_aggressor(Aggressor("ghost", "powertrain"))          # unrouted: skipped
    f = _one(b.check_signal_integrity(), "si-crosstalk")
    assert f.severity == Severity.FAIL
    assert f.subsystems == ["electrics", "powertrain"]
    assert f.message == (
        "CAN pair 'can' runs 0.50 mm from powertrain's switching net 'hv_u' (600 V, "
        "12 V/ns edges) for ~10 mm — the differential pair is exposed to "
        "motor-controller noise. Move the route or add a guard ground.")
    assert f.detail == dict(pair="can", aggressor="inv", gap_mm=0.5,
                            coupled_len_mm=pytest.approx(10.0), sw_voltage_v=600.0,
                            edge_v_per_ns=12.0, estimate=False,
                            method="geometric screening, not field-solved")
    ag.path_mm = [(0, -0.6), (10, -0.6)]
    f = _one(b.check_signal_integrity(), "si-crosstalk")
    assert f.severity == Severity.WARN
    ag.path_mm = [(0, -2.0), (10, -2.0)]
    ag.is_estimate = True
    f = _one(b.check_signal_integrity(), "si-crosstalk")
    assert f.severity == Severity.WARN
    assert f.message == ("CAN pair 'can' passes within 2.00 mm of powertrain's "
                         "'hv_u' for ~10 mm — tighten spacing-to-HV or add "
                         "isolation (estimated routing).")
    assert f.detail == dict(pair="can", aggressor="inv", gap_mm=2.0,
                            coupled_len_mm=pytest.approx(10.0), estimate=True,
                            method="geometric screening, not field-solved")
    ag.path_mm = [(0, -2.5), (10, -2.5)]
    f = _one(b.check_signal_integrity(), "si-crosstalk")
    assert f.severity == Severity.OK
    assert f.message == "CAN pair 'can' clears 'hv_u' by 2.5 mm (> 2 mm) (estimated routing)."
    assert f.detail == dict(pair="can", aggressor="inv", gap_mm=2.5, estimate=True)


def test_crosstalk_estimate_from_pair_only():
    b = _si_board()
    b.set_pair(DiffPair("can", "electrics", path_mm=[(0, 0), (10, 0)], is_estimate=True))
    b.set_aggressor(Aggressor("inv", "pt", path_mm=[(0, 3), (10, 3)], is_estimate=False))
    f = _one(b.check_signal_integrity(), "si-crosstalk")
    assert f.detail["estimate"] is True and f.message.endswith("(estimated routing).")


def test_crosstalk_run_window_follows_gap_when_above_warn():
    b = _si_board(coupling_warn_mm=1.0, coupling_fail_mm=1.9999)
    b.set_pair(DiffPair("can", "e", path_mm=[(0, 0), (20, 0)], is_estimate=False))
    ag_path = [(0, -2.0), (20, -2.4)]
    b.set_aggressor(Aggressor("inv", "pt", path_mm=ag_path, is_estimate=False))
    a = np.array([[0.0, 0.0], [20.0, 0.0]])
    gap = min_parallel_distance_mm(a, np.array(ag_path, float))
    want = parallel_run_length_mm(a, np.array(ag_path, float), within_mm=gap + 0.01)
    assert 0.0 < want < 5.0
    b.coupling_fail_mm = gap
    f = _one(b.check_signal_integrity(), "si-crosstalk")
    assert f.severity == Severity.FAIL
    assert f.detail["coupled_len_mm"] == pytest.approx(want)


def test_si_detail():
    b = _si_board()
    assert b.si_detail("nope") == {}
    dp = DiffPair("can", "e")
    b.set_pair(dp)
    d = b.si_detail("can")
    assert d == dict(
        pair="can", single_ended_z0_ohm=dp.single_ended_z0_ohm(),
        differential_z0_ohm=dp.differential_z0_ohm(),
        method="IPC-2141 microstrip + edge-coupling correction (analytic)",
        eye_height_v=None, insertion_loss_db=None, reflection_coeff=None,
        coupled_noise_v=None,
        note="impedance is an analytic screening estimate; eye/loss/reflection "
             "require a 2-D field solver and transient sim and are intentionally "
             "not invented here.")


def test_board_dict_roundtrip():
    b = BoardLedger(rail_nominal_v=3.3, ecu_brownout_v=2.9, ambient_c=55.0,
                    ambient_is_local=True, max_trace_temp_c=95.0,
                    fuse_safety_factor=1.5, coupling_warn_mm=3.0, coupling_fail_mm=1.0)
    b.set_trace(Trace("t", "n", "o"))
    b.set_pair(DiffPair("p", "e", path_mm=[(0, 0), (1, 1)]))
    b.set_aggressor(Aggressor("g", "pt", path_mm=[(2, 2), (3, 3)]))
    d = b.as_dict()
    assert d["traces"] == {"t": Trace("t", "n", "o").as_dict()}
    assert set(d) == {"traces", "pairs", "aggressors", "rail_nominal_v",
                      "ecu_brownout_v", "ambient_c", "ambient_is_local",
                      "max_trace_temp_c", "fuse_safety_factor", "coupling_warn_mm",
                      "coupling_fail_mm"}
    back = BoardLedger.from_dict(d)
    assert back == b
    assert BoardLedger.from_dict(None) == BoardLedger()
    assert BoardLedger.from_dict({"traces": None, "rail_nominal_v": None}) == BoardLedger()
    part = BoardLedger.from_dict({"ambient_c": 10.0})
    assert part.ambient_c == 10.0 and part.rail_nominal_v == 5.0


# --------------------------------------------------- ledger roll-ups ---- #
def _ledger():
    L = IntegrationLedger()
    L.set(SubsystemInterface(name="cooling", peak_current_a=8.5))
    L.set(SubsystemInterface(name="brakes", peak_current_a=1.5))
    L.set(SubsystemInterface(name="dash"))
    return L


def test_worst_case_and_undeclared():
    L = _ledger()
    sc = {"fan": ["cooling", "brakes"], "dash": ["dash"], "x": ["nope", "brakes"],
          "none": []}
    assert worst_case_currents(BoardLedger(), L, sc) == {"fan": 10.0, "x": 1.5}
    assert worst_case_currents(BoardLedger(), None, sc) == {}
    assert undeclared_loads(L, sc) == {"dash": ["dash"], "x": ["nope"]}
    assert undeclared_loads(None, {"a": ["s", "t"], "b": None}) == {"a": ["s", "t"]}
    L.set(SubsystemInterface(name="z", peak_current_a=0))
    assert worst_case_currents(BoardLedger(), L, {"q": ["z"]}) == {"q": 0.0}


def test_net_currents():
    b = BoardLedger()
    b.set_trace(Trace("t1", "fan_net", "e"))
    b.set_trace(Trace("t2", "fan_net", "e"))
    b.set_trace(Trace("t3", "", "e"))
    L = _ledger()
    sc = {"t1": ["brakes"], "t2": ["cooling"], "t3": ["cooling"], "gone": ["cooling"]}
    assert net_currents(b, L, sc) == {"fan_net": 8.5}
    assert net_currents(b, L, {"t1": ["cooling"], "t2": ["brakes"]}) == {"fan_net": 8.5}
    assert net_currents(b, L, {"t1": ["dash"]}) == {}


def _f(sev):
    return Finding("c", sev, "m")


def test_board_check_result_summary():
    r = BoardCheckResult(findings=[_f(Severity.FAIL), _f(Severity.WARN),
                                   _f(Severity.WARN), _f(Severity.OK), _f(Severity.MISSING)])
    assert r.has_hard_fail() is True
    assert r.summary() == "FAIL: 1 fail / 2 warn / 1 ok across 5 board checks"
    r2 = BoardCheckResult(findings=[_f(Severity.OK)])
    assert r2.has_hard_fail() is False
    assert r2.summary() == "OK: 0 fail / 0 warn / 1 ok across 1 board checks"
    assert BoardCheckResult().findings == []


def test_check_board_end_to_end():
    b = BoardLedger(ambient_c=20.0)
    b.set_trace(Trace("fan", "fan_pwr", "electrics", feeds="cooling", width_mm=1.0))
    b.set_trace(Trace("dsh", "d", "electrics", feeds="dash"))
    b.set_pair(DiffPair("can", "electrics", path_mm=[(0, 0), (10, 0)]))
    b.set_aggressor(Aggressor("inv", "pt", path_mm=[(0, -0.2), (10, -0.2)]))
    L = _ledger()
    L.ambient_c = 50.0
    res = check_board(b, L, {"fan": ["cooling"], "dsh": ["dash"]})
    assert b.ambient_c == 50.0
    checks = [f.check for f in res.findings]
    assert checks.count("trace-current") == 1
    miss = _one(res.findings, "trace-current")
    assert miss.detail["undeclared_loads"] == ["dash"]
    assert "si-crosstalk" in checks and "diffpair-impedance" in checks
    assert res.has_hard_fail()
    # without a ledger: no current anywhere, nothing pushed onto the board
    b2 = BoardLedger(ambient_c=20.0)
    b2.set_trace(Trace("fan", "fan_pwr", "electrics", feeds="cooling"))
    r2 = check_board(b2)
    assert b2.ambient_c == 20.0
    assert [f.check for f in r2.findings] == ["trace-current", "trace-survival"]
    # local ambient is reported, not overwritten
    b3 = BoardLedger(ambient_c=20.0, ambient_is_local=True)
    r3 = check_board(b3, L, None)
    assert b3.ambient_c == 20.0
    assert [f.check for f in r3.findings] == ["ambient-local"]


class _OldBoard(BoardLedger):
    def check_traces(self, currents=None):          # pre-`undeclared` signature
        self.seen = currents
        return [Finding("old", Severity.OK, "m")]


class _NoSig:
    __signature__ = "not a signature"

    def __init__(self, board):
        self.board = board

    def __call__(self, currents=None, undeclared=None):
        self.board.seen = (currents, undeclared)
        return []


def test_check_board_legacy_check_traces():
    b = _OldBoard()
    L = _ledger()
    r = check_board(b, L, {"x": ["cooling"]})
    assert b.seen == {"x": 8.5}
    assert [f.check for f in r.findings] == ["old"]
    b2 = BoardLedger()
    b2.check_traces = _NoSig(b2)
    with pytest.raises((TypeError, ValueError)):
        inspect.signature(b2.check_traces)
    check_board(b2, L, {"x": ["cooling", "dash"]})
    assert b2.seen == ({"x": 8.5}, None)


def test_characterized_boards():
    """Full findings (check, severity, message, subsystems, detail) over whole boards, every segment-distance
    branch, and the summary with missing counts; recorded in tests/golden/electronics_*.json."""
    from tests._golden import check
    res = {}
    segs = {"points": ((1, 2), (1, 2), (4, 6), (4, 6)), "pt_beyond": ((15, 3), (15, 3), (0, 0), (10, 0)),
            "pt_before": ((-4, 3), (-4, 3), (0, 0), (10, 0)), "seg_to_pt_beyond": ((0, 0), (10, 0), (14, 3), (14, 3)),
            "seg_to_pt_before": ((0, 0), (10, 0), (-3, 2), (-3, 2)),
            "parallel_offset": ((0, 0), (10, 0), (14, 1), (20, 1)), "parallel_back": ((0, 0), (10, 0), (-9, 1), (-3, 1)),
            "collinear_gap": ((0, 0), (5, 0), (8, 0), (12, 0)), "crossing": ((0, 0), (10, 10), (0, 10), (10, 0))}
    res["seg"] = {k: E._seg_seg_dist(*v) for k, v in segs.items()}
    a3 = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]
    b3 = [(4.0, 6.5), (5.0, 7.0), (30.0, 30.0)]
    res["minpar"] = [E.min_parallel_distance_mm(np.array(a3), np.array(b3)), E.min_parallel_distance_mm(np.array(b3), np.array(a3))]
    A3 = np.array(a3)
    res["runlen"] = [E.parallel_run_length_mm(A3, np.array([(0, 1), (10, 1), (10, 11), (30, 11)], float), 2.0),
                     E.parallel_run_length_mm(np.array([(0, 1), (10, 1), (10, 11)], float), A3, 1.5)]
    res["z0"] = [E.DiffPair("p", "e", trace_w_mm=w).single_ended_z0_ohm() for w in (0.03, -0.03, 0.2)]
    L = IntegrationLedger()
    L.set(SubsystemInterface(name="cooling", peak_current_a=8.5))
    L.set(SubsystemInterface(name="dash"))
    L.set(SubsystemInterface(name="tiny", peak_current_a=0.4))
    L.ambient_c = 55.0
    b = BoardLedger(ambient_c=20.0)
    b.set_trace(Trace("fan", "fan_pwr", "electrics", feeds="cooling", width_mm=0.3))
    b.set_trace(Trace("dsh", "d", "electrics", feeds="dash"))
    b.set_trace(Trace("tn", "tiny_net", "electrics", feeds="tiny", width_mm=0.2))
    b.set_pair(DiffPair("unrouted", "electrics"))
    b.set_pair(DiffPair("can", "electrics", path_mm=[(0, 0), (10, 0)], target_z0_ohm=40.0))
    b.set_aggressor(Aggressor("ghost", "powertrain"))
    b.set_aggressor(Aggressor("inv", "powertrain", path_mm=[(0, -0.3), (10, -0.3)]))
    b.set_aggressor(Aggressor("far", "aero", path_mm=[(0, -1.5), (10, -1.5)]))
    sc = {"fan": ["cooling"], "dsh": ["dash"], "tn": ["tiny"]}
    r = check_board(b, L, sc)
    res["board"] = [f.as_dict() for f in r.findings] + [r.summary()]
    res["board_noledger"] = [f.as_dict() for f in check_board(b, None, sc).findings]
    res["si"] = [f.as_dict() for f in b.check_signal_integrity()]
    res["traces"] = [f.as_dict() for f in b.check_traces({"fan": 8.5, "dsh": 0.0, "tn": 0.4})]
    res["nets"] = E.net_currents(b, L, {"tn": ["tiny"], "fan": ["cooling"]})
    e = BoardLedger()
    e.set_trace(Trace("blank", "", "e"))
    e.set_trace(Trace("small", "s_net", "e"))
    res["nets_blank_first"] = E.net_currents(e, L, {"blank": ["cooling"], "small": ["tiny"]})

    class OldBoard(BoardLedger):
        def check_traces(self, currents):                     # an older signature: no `undeclared`
            return BoardLedger.check_traces(self, currents)
    ob = OldBoard(ambient_c=20.0)
    ob.set_trace(Trace("fan", "fan_pwr", "electrics", feeds="cooling", width_mm=0.3))
    res["old_board"] = [f.as_dict() for f in check_board(ob, L, {"fan": ["cooling"]}).findings]
    F = E.Finding
    res["summary"] = [E.BoardCheckResult(findings=fs).summary() for fs in (
        [F("c", Severity.MISSING, "m")], [F("c", Severity.WARN, "m")], [F("c", Severity.FAIL, "m")],
        [F("c", Severity.OK, "m")], [])]
    check("electronics_boards", res)


def test_characterized_branches():
    from tests._golden import check
    b = BoardLedger(ambient_c=30.0)
    for i, (w, I) in enumerate(((0.1, 2.0), (1.0, 2.0), (10.0, 28.0), (10.0, 30.0), (3.0, 1.0))):
        b.set_trace(Trace(f"t{i}", f"n{i}", "electrics", feeds="cooling", width_mm=w, length_mm=80.0))
    cur = {f"t{i}": I for i, (w, I) in enumerate(((0.1, 2.0), (1.0, 2.0), (10.0, 28.0), (10.0, 30.0), (3.0, 1.0)))}
    b.set_pair(DiffPair("can", "electrics", path_mm=[(0, 0), (20, 0)]))
    for name, y in (("fail", -0.3), ("warn", -1.2), ("ok", -5.0)):
        b.set_aggressor(Aggressor(name, "powertrain", path_mm=[(0, y), (20, y)]))
    check("electronics_branches", {"traces": [f.as_dict() for f in b.check_traces(cur)],
                                   "si": [f.as_dict() for f in b.check_signal_integrity()]})
