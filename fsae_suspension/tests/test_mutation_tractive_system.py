"""suspension/tractive_system.py: the R-C closed forms, the precharge trace before and after the shorting switch, and
every rule gate (precharge speed, resistor pulse energy, bleed time, shutdown loop, TSAL, BSPD) at, inside and past
its limit, with each finding's message, owners and detail checked word for word."""
import math

import numpy as np
import pytest

from suspension import tractive_system as ts
from suspension.interfaces import Severity, summarize

PC = ts.PrechargeCircuit(400.0, 600e-6, 1000.0, discharge_r_ohm=2000.0, resistor_energy_rating_j=60.0, is_estimate=False)
EST = " (on ESTIMATE values)"


def test_the_rules():
    r = ts.Rules()
    assert r.as_dict() == dict(safe_voltage_v=60.0, discharge_time_s=5.0, precharge_fraction=0.90,
                               precharge_max_time_s=5.0, precharge_min_time_s=0.05, tsal_flash_hz_min=2.0,
                               tsal_flash_hz_max=5.0, bspd_reaction_max_s=0.5)
    with pytest.raises(ValueError) as e:
        ts.Rules(precharge_min_time_s=6.0)
    assert str(e.value) == ("precharge_min_time_s (6.0) is above precharge_max_time_s (5.0); no precharge duration "
                            "could ever pass.")
    with pytest.raises(ValueError) as e:
        ts.Rules(tsal_flash_hz_min=6.0)
    assert str(e.value) == "tsal_flash_hz_min (6.0) is above tsal_flash_hz_max (5.0)."
    ts.Rules(precharge_min_time_s=5.0, tsal_flash_hz_min=5.0)                 # equal bounds are a window of one


def test_the_closed_forms():
    tau = 1000.0 * 600e-6
    assert PC.tau_precharge_s == pytest.approx(tau) and PC.tau_discharge_s == pytest.approx(2000.0 * 600e-6)
    assert PC.peak_inrush_a == pytest.approx(0.4) and PC.peak_resistor_power_w == pytest.approx(160.0)
    assert PC.precharge_pulse_energy_j == pytest.approx(0.5 * 600e-6 * 400 ** 2)
    assert PC.time_to_fraction_s(0.9) == pytest.approx(-tau * math.log(0.1))
    assert PC.time_to_fraction_s(0.0) == pytest.approx(-tau * math.log(1 - 1e-6))
    assert PC.time_to_fraction_s(2.0) == pytest.approx(-tau * math.log(1e-9))
    assert PC.time_to_safe_s(60.0) == pytest.approx(-1.2 * math.log(60 / 400))
    assert PC.time_to_safe_s(400.0) == 0.0 and PC.time_to_safe_s(500.0) == 0.0
    bare = ts.PrechargeCircuit(400.0, 600e-6, 0.0)
    assert bare.tau_discharge_s is None and bare.time_to_safe_s(60.0) is None
    assert bare.peak_inrush_a == pytest.approx(400.0 / 1e-9)
    assert (bare.resistor_power_rating_w, bare.resistor_energy_rating_j, bare.is_estimate, bare.set_by, bare.notes) == (
        None, None, True, "", "")
    d = PC.as_dict()
    assert d["tau_precharge_s"] == PC.tau_precharge_s and d["tau_discharge_s"] == PC.tau_discharge_s
    assert d["peak_inrush_a"] == PC.peak_inrush_a and d["precharge_pulse_energy_j"] == PC.precharge_pulse_energy_j
    assert ts.PrechargeCircuit.from_dict(d) == PC
    full = ts.PrechargeCircuit(1, 2, 3, 4, 5, 6, False, "me", "n")
    assert ts.PrechargeCircuit.from_dict(full.as_dict()) == full
    assert ts.PrechargeCircuit.from_dict({"pack_voltage_v": 1, "link_capacitance_f": 2, "precharge_r_ohm": 3}) == \
        ts.PrechargeCircuit(1, 2, 3)


def test_the_precharge_trace_without_a_switch():
    tau = PC.tau_precharge_s
    tr = ts.simulate_precharge(PC, n=101)
    t = np.linspace(0.0, 6 * tau, 101)
    v = 400.0 * (1 - np.exp(-t / tau))
    assert tr.ok and tr.warnings == [] and tr.t_switch_s is None
    assert tr.time_s == pytest.approx(t) and tr.v_cap_v == pytest.approx(v, rel=1e-12, abs=1e-12)
    assert tr.i_a == pytest.approx((400.0 - v) / 1000.0, rel=1e-12, abs=1e-15)
    assert tr.p_resistor_w == pytest.approx(((400.0 - v) / 1000.0) ** 2 * 1000.0, rel=1e-12, abs=1e-15)
    assert len(ts.simulate_precharge(PC, n=3).time_s) == 8 and len(ts.simulate_precharge(PC).time_s) == 2000
    assert ts.simulate_precharge(PC, t_end_s=-1.0, n=8).time_s[-1] == pytest.approx(1e-6)
    assert ts.simulate_precharge(PC, t_end_s=2.0, n=11).time_s[-1] == 2.0
    zero = ts.simulate_precharge(ts.PrechargeCircuit(400.0, 0.0, 1000.0), t_end_s=1.0, n=8)
    assert zero.ok and zero.v_cap_v[1:] == pytest.approx(400.0)


def test_the_precharge_trace_with_the_shorting_switch():
    tau = PC.tau_precharge_s
    tr = ts.simulate_precharge(PC, t_switch_s=0.5, n=201)
    t = np.linspace(0.0, 1.0, 201)
    v_at = 400.0 * (1 - math.exp(-0.5 / tau))
    ts_ = tau * 1e-3
    v = np.where(t >= 0.5, 400.0 - (400.0 - v_at) * np.exp(-np.clip((t - 0.5) / ts_, 0, 700)),
                 400.0 * (1 - np.exp(-t / tau)))
    assert tr.v_cap_v == pytest.approx(v, rel=1e-12, abs=1e-12) and tr.t_switch_s == 0.5
    i = np.where(t >= 0.5, 0.0, (400.0 - v) / 1000.0)
    assert tr.i_a == pytest.approx(i, rel=1e-12, abs=1e-15) and tr.i_a[100] == 0.0 and tr.i_a[99] > 0
    assert tr.warnings == ["Switch shorts precharge R at t=0.5s; the post-switch current spike is bounded by a modelled "
                           "residual R (ideal short ⇒ unbounded inrush — size a real switch/contactor for it)."]
    late = ts.simulate_precharge(PC, t_end_s=0.4, t_switch_s=0.5, n=50)
    assert late.warnings == [] and late.i_a[-1] == pytest.approx((400.0 - late.v_cap_v[-1]) / 1000.0)
    at0 = ts.simulate_precharge(PC, t_end_s=1.0, t_switch_s=0.0, n=10)
    assert at0.warnings == [] and np.all(at0.i_a == 0.0) and at0.v_cap_v[-1] < 400.0
    edge = ts.simulate_precharge(PC, t_end_s=0.5, t_switch_s=0.5, n=10)
    assert edge.warnings == []
    tiny = ts.PrechargeCircuit(400.0, 1e-12, 1e-6)                          # tau far under a nanosecond
    s = ts.simulate_precharge(tiny, t_end_s=1.0, t_switch_s=0.5, n=10)
    assert s.ok and np.all(np.isfinite(s.v_cap_v))


def test_a_crashed_trace():
    bad = ts.PrechargeCircuit("x", 1e-3, 10.0)
    tr = ts.simulate_precharge(bad, t_switch_s=0.2)
    assert tr.ok is False and tr.t_switch_s == 0.2 and len(tr.time_s) == 8 and not tr.v_cap_v.any()
    assert tr.warnings[-1].startswith("precharge sim crashed: ") and len(tr.warnings) == 1
    assert ts.PrechargeTrace(np.zeros(1), np.zeros(1), np.zeros(1), np.zeros(1), None).ok is True


def _by(fs):
    return {f.check: f for f in fs}


def test_the_precharge_gates_pass():
    f = _by(ts.check_precharge(PC))
    t = PC.time_to_fraction_s(0.9)
    assert [x.check for x in ts.check_precharge(PC)] == ["precharge-time", "precharge-resistor-energy", "discharge-time"]
    p = f["precharge-time"]
    assert p.severity == Severity.OK and p.subsystems == ["electrics", "battery-pack"]
    assert p.message == f"Precharge reaches 90% of pack voltage in {t:.2f}s (≤ 5.0s)."
    assert p.detail == {"t_to_fraction_s": t, "tau_s": PC.tau_precharge_s}
    e = f["precharge-resistor-energy"]
    assert e.severity == Severity.OK and e.message == "Precharge pulse energy 48.0J ≤ resistor rating 60.0J."
    assert e.detail == {"pulse_energy_j": PC.precharge_pulse_energy_j}
    d = f["discharge-time"]
    ts_ = PC.time_to_safe_s(60.0)
    assert d.severity == Severity.OK and d.message == f"Bus bleeds below 60V in {ts_:.2f}s (≤ 5s)."
    assert d.detail == {"t_to_safe_s": ts_}


def test_the_precharge_gates_fail():
    slow = ts.PrechargeCircuit(400.0, 600e-6, 5000.0, discharge_r_ohm=1e4, resistor_energy_rating_j=40.0)
    f = _by(ts.check_precharge(slow))
    t = slow.time_to_fraction_s(0.9)
    assert f["precharge-time"].severity == Severity.FAIL and f["precharge-time"].message == (
        f"Precharge reaches 90% of pack voltage in {t:.2f}s — over the 5.0s limit{EST}. Lower the precharge resistor "
        f"(τ=R·C={slow.tau_precharge_s:.3g}s).")
    assert f["precharge-time"].detail == pytest.approx({"t_to_fraction_s": t, "tau_s": 3.0, "limit_s": 5.0})
    assert f["precharge-resistor-energy"].severity == Severity.FAIL and f["precharge-resistor-energy"].message == (
        "Each precharge dumps 48.0J in the resistor (½·C·V²) but it is rated for 40.0J per pulse — it will cook. Use a "
        "higher-energy resistor.")
    assert f["precharge-resistor-energy"].detail == pytest.approx({"pulse_energy_j": 48.0, "rating_j": 40.0})
    t2 = slow.time_to_safe_s(60.0)
    assert f["discharge-time"].severity == Severity.FAIL and f["discharge-time"].message == (
        f"Bus bleeds to 60V in {t2:.2f}s — over the 5s safe-discharge limit{EST}. Lower the discharge resistor.")
    assert f["discharge-time"].detail == {"t_to_safe_s": t2, "limit_s": 5.0}
    assert all(x.subsystems == ["electrics", "battery-pack"] for x in f.values())


def test_a_fast_precharge_and_the_missing_parts():
    fast = ts.PrechargeCircuit(400.0, 600e-6, 10.0)
    f = _by(ts.check_precharge(fast))
    t = fast.time_to_fraction_s(0.9)
    assert f["precharge-time"].severity == Severity.WARN and f["precharge-time"].message == (
        f"Precharge reaches 90% in only {t:.3g}s — the resistor is so small that inrush (40A peak) is barely "
        f"limited{EST}. Raise R or check the inrush rating.")
    assert f["precharge-time"].detail == pytest.approx({"t_to_fraction_s": t, "peak_inrush_a": 40.0})
    assert f["precharge-resistor-energy"].severity == Severity.MISSING and f["precharge-resistor-energy"].message == (
        "Each precharge dumps 48.0J (½·C·V²) in the precharge resistor; its per-pulse energy rating is not declared, "
        "so survival can't be checked. Add resistor_energy_rating_j.")
    assert f["precharge-resistor-energy"].detail == pytest.approx({"pulse_energy_j": 48.0})
    assert f["discharge-time"].severity == Severity.MISSING and f["discharge-time"].message == (
        "No discharge (bleed) resistor declared — the bus can't be shown to fall below 60V within 5s. Add "
        "discharge_r_ohm.")
    assert f["discharge-time"].subsystems == ["electrics", "battery-pack"]
    assert f["precharge-time"].subsystems == ["electrics", "battery-pack"]


def test_the_gates_at_their_edges():
    r = ts.Rules(precharge_fraction=0.8, safe_voltage_v=50.0, discharge_time_s=4.0)
    tmax = PC.time_to_fraction_s(0.8)
    edge = ts.Rules(precharge_fraction=0.8, precharge_max_time_s=tmax, precharge_min_time_s=0.0)
    assert _by(ts.check_precharge(PC, edge))["precharge-time"].severity == Severity.OK
    low = ts.Rules(precharge_fraction=0.8, precharge_min_time_s=tmax, precharge_max_time_s=10.0)
    assert _by(ts.check_precharge(PC, low))["precharge-time"].severity == Severity.OK
    m = _by(ts.check_precharge(PC, r))
    assert m["precharge-time"].message.startswith("Precharge reaches 80% of pack voltage")
    assert m["discharge-time"].message == f"Bus bleeds below 50V in {PC.time_to_safe_s(50.0):.2f}s (≤ 4s)."
    exact = ts.PrechargeCircuit(400.0, 600e-6, 1000.0, discharge_r_ohm=2000.0, resistor_energy_rating_j=48.0)
    assert _by(ts.check_precharge(exact))["precharge-resistor-energy"].severity == Severity.OK
    tsafe = PC.time_to_safe_s(60.0)
    assert _by(ts.check_precharge(PC, ts.Rules(discharge_time_s=tsafe)))["discharge-time"].severity == Severity.OK
    assert ts.check_precharge(ts.PrechargeCircuit(400.0, 600e-6, 1000.0, is_estimate=True))[0].message.endswith(EST + ".")


def _chain(*specs):
    c = ts.ShutdownChain()
    for name, kind, nc, est in specs:
        c.add(ts.ShutdownNode(name, kind, normally_closed=nc, is_estimate=est))
    return c


FULL = [(k, k, True, False) for k in ts.REQUIRED_SHUTDOWN_NODES]


def test_the_shutdown_chain_object():
    assert ts.REQUIRED_SHUTDOWN_NODES == ("master_switch", "bspd", "ams", "imd", "interlock", "inertia", "estop")
    n = ts.ShutdownNode("a", "bspd")
    assert (n.normally_closed, n.trips_on, n.set_by, n.is_estimate) == (True, "", "", True)
    assert n.as_dict() == dict(name="a", kind="bspd", normally_closed=True, trips_on="", set_by="", is_estimate=True)
    c = _chain(*FULL)
    assert c.feeds == "main_contactor_coil" and c.kinds_present == set(ts.REQUIRED_SHUTDOWN_NODES)
    assert c.is_closed() is True and c.is_closed(["imd"]) is False and c.is_closed(["nope"]) is True
    assert c.as_dict() == {"nodes": [x.as_dict() for x in c.nodes], "feeds": "main_contactor_coil"}
    assert _chain(("x", "bspd", False, True)).is_closed() is False
    assert ts.ShutdownChain().is_closed() is True and ts.ShutdownChain().nodes == []


def test_a_complete_failsafe_chain():
    fs = ts.check_shutdown_chain(_chain(*FULL))
    assert [f.check for f in fs] == ["shutdown-failsafe", "shutdown-completeness", "shutdown-single-fault"]
    assert all(f.severity == Severity.OK and f.subsystems == ["electrics", "glv"] for f in fs)
    assert fs[0].message == "All shutdown nodes are normally-closed (open-to-trip, fail-safe)."
    present = sorted(ts.REQUIRED_SHUTDOWN_NODES)
    assert fs[1].message == f"All 7 rule-required shutdown nodes present ({', '.join(present)})."
    assert fs[1].detail == {"present": present}
    assert fs[2].message == "Any single node opening drops the contactor coil (true series loop)."


def test_a_chain_with_every_fault():
    c = _chain(("bspd1", "bspd", True, True), ("imd1", "imd", False, False), ("x", "other", True, True))
    fs = _by(ts.check_shutdown_chain(c))
    assert fs["shutdown-failsafe"].severity == Severity.FAIL and fs["shutdown-failsafe"].message == (
        "Shutdown nodes wired normally-OPEN: imd1. A safety loop must be normally-CLOSED so a broken wire opens it "
        "(fail-safe).")
    assert fs["shutdown-failsafe"].detail == {"normally_open_nodes": ["imd1"]}
    miss = ["master_switch", "ams", "interlock", "inertia", "estop"]
    assert fs["shutdown-completeness"].severity == Severity.FAIL and fs["shutdown-completeness"].message == (
        f"Shutdown circuit is missing rule-required node(s): {', '.join(miss)}. The tractive system can't be enabled at "
        "tech without them.")
    assert fs["shutdown-completeness"].detail == {"missing": miss, "present": ["bspd", "imd", "other"]}
    assert fs["shutdown-provenance"].severity == Severity.INFO and fs["shutdown-provenance"].message == (
        "Shutdown nodes still on placeholder/estimate values: bspd1, x. Confirm against the as-wired loom.")
    assert fs["shutdown-provenance"].detail == {"estimate_nodes": ["bspd1", "x"]}
    assert fs["shutdown-single-fault"].severity == Severity.OK                # a normally-open node already opens it
    assert all(f.subsystems == ["electrics", "glv"] for f in fs.values())
    custom = ts.check_shutdown_chain(_chain(("a", "bspd", True, False)), required=("bspd",))
    assert custom[1].message == "All 1 rule-required shutdown nodes present (bspd)."


def test_a_node_that_is_not_in_series():
    class Bypassed(ts.ShutdownChain):
        def is_closed(self, open_nodes=()):
            return "loose" in open_nodes or super().is_closed(open_nodes)
    c = Bypassed()
    for k in ts.REQUIRED_SHUTDOWN_NODES:
        c.add(ts.ShutdownNode(k, k, is_estimate=False))
    c.add(ts.ShutdownNode("loose", "other", is_estimate=False))
    f = _by(ts.check_shutdown_chain(c))["shutdown-single-fault"]
    assert f.severity == Severity.FAIL and f.message == (
        "Opening node(s) loose does NOT open the chain — they are not actually in series with the contactor coil. "
        "Re-route.")
    assert f.detail == {"not_in_series": ["loose"]} and f.subsystems == ["electrics", "glv"]


def test_an_empty_chain():
    (f,) = ts.check_shutdown_chain(ts.ShutdownChain())
    assert (f.check, f.severity, f.subsystems) == ("shutdown-chain", Severity.MISSING, ["electrics", "glv"])
    assert f.message == ("No shutdown-circuit nodes declared. Add the series safety loop (master switches, BSPD, AMS, "
                         "IMD, interlocks, inertia, e-stops).")


def test_the_tsal():
    t = ts.TSAL(3.0, 40.0)
    assert (t.set_by, t.is_estimate) == ("", True)
    ok = ts.check_tsal(t)
    assert [(f.check, f.severity) for f in ok] == [("tsal-flash", Severity.OK), ("tsal-threshold", Severity.OK)]
    assert ok[0].message == "TSAL flash rate 3.0Hz within 2–5Hz." and ok[1].message == "TSAL indicates safe only below 40V (≤ 60V)."
    assert all(f.subsystems == ["electrics", "tractive"] for f in ok)
    bad = ts.check_tsal(ts.TSAL(7.25, 80.0))
    assert bad[0].severity == Severity.FAIL and bad[0].message == "TSAL flashes at 7.2Hz — outside the required 2–5Hz band."
    assert bad[0].detail == {"flash_hz": 7.25}
    assert bad[1].severity == Severity.FAIL and bad[1].message == (
        "TSAL may indicate 'safe' up to 80V, above the 60V safe-to-touch limit — it could show green while the bus is "
        "still live. Lower the threshold.")
    assert bad[1].detail == {"threshold_v": 80.0} and all(f.subsystems == ["electrics", "tractive"] for f in bad)
    assert ts.check_tsal(ts.TSAL(1.5, 60.0))[0].severity == Severity.FAIL
    edges = ts.check_tsal(ts.TSAL(2.0, 60.0)) + ts.check_tsal(ts.TSAL(5.0, 60.0))
    assert all(f.severity == Severity.OK for f in edges)
    r = ts.Rules(tsal_flash_hz_min=1.0, tsal_flash_hz_max=1.5, safe_voltage_v=30.0)
    assert ts.check_tsal(ts.TSAL(1.2, 40.0), r)[0].message == "TSAL flash rate 1.2Hz within 1–2Hz."
    assert ts.check_tsal(ts.TSAL(1.2, 40.0), r)[1].severity == Severity.FAIL


def test_the_bspd():
    b = ts.BSPD(30.0, 5000.0, 0.2)
    assert (b.set_by, b.is_estimate) == ("", True)
    assert b.trips(30.0, 5000.0) and not b.trips(29.9, 9e9) and not b.trips(99.0, 4999.0)
    (f,) = ts.check_bspd(b)
    assert (f.check, f.severity, f.message, f.subsystems) == ("bspd-reaction", Severity.OK,
                                                              "BSPD reaction 200ms within 500ms.", ["electrics", "tractive"])
    assert ts.check_bspd(ts.BSPD(30.0, 5000.0, 0.5))[0].severity == Severity.OK
    slow, thr = ts.check_bspd(ts.BSPD(0.0, 5000.0, 0.75))
    assert slow.severity == Severity.FAIL and slow.message == (
        "BSPD reacts in 750ms — slower than the 500ms limit. It must open the shutdown loop faster.")
    assert slow.detail == {"reaction_s": 0.75} and slow.subsystems == ["electrics", "tractive"]
    assert (thr.check, thr.severity, thr.subsystems) == ("bspd-thresholds", Severity.WARN, ["electrics", "tractive"])
    assert thr.message == ("BSPD brake/power thresholds are non-positive — it will never arm or always trip. Set real "
                           "arming levels.")
    assert len(ts.check_bspd(ts.BSPD(1.0, 0.0, 0.1))) == 2 and len(ts.check_bspd(ts.BSPD(1.0, 1.0, 0.1))) == 1
    assert ts.check_bspd(ts.BSPD(1.0, 1.0, 0.3), ts.Rules(bspd_reaction_max_s=0.25))[0].message == (
        "BSPD reacts in 300ms — slower than the 250ms limit. It must open the shutdown loop faster.")


def test_the_one_call_gate():
    empty = ts.check_tractive_system()
    assert empty.findings == [] and empty.warnings == [
        "Nothing declared yet — declare a precharge circuit, shutdown chain, TSAL and/or BSPD to gate them."]
    assert empty.has_hard_fail() is False
    r = ts.Rules(safe_voltage_v=50.0)
    res = ts.check_tractive_system(PC, _chain(*FULL), ts.TSAL(3.0, 55.0), ts.BSPD(1.0, 1.0, 0.1), r)
    want = (ts.check_precharge(PC, r) + ts.check_shutdown_chain(_chain(*FULL), r) + ts.check_tsal(ts.TSAL(3.0, 55.0), r)
            + ts.check_bspd(ts.BSPD(1.0, 1.0, 0.1), r))
    assert [(f.check, f.severity, f.message) for f in res.findings] == [(f.check, f.severity, f.message) for f in want]
    assert res.warnings == [] and res.has_hard_fail() is True
    s = summarize(res.findings)
    assert res.summary() == (f"{s['worst'].upper()}: {s['counts'].get('fail', 0)} fail / "
                             f"{s['counts'].get('warning', 0)} warn / {s['counts'].get('missing', 0)} missing / "
                             f"{s['counts'].get('ok', 0)} ok across {len(res.findings)} tractive-safety checks")
    assert res.summary().startswith("FAIL: 1 fail / 0 warn / 0 missing / ")
    only = ts.check_tractive_system(tsal=ts.TSAL(3.0, 40.0))
    assert [f.check for f in only.findings] == ["tsal-flash", "tsal-threshold"] and not only.has_hard_fail()
    assert ts.check_tractive_system(bspd=ts.BSPD(1.0, 1.0, 0.1)).findings[0].check == "bspd-reaction"
    assert ts.check_tractive_system(chain=ts.ShutdownChain()).findings[0].check == "shutdown-chain"
    assert ts.check_tractive_system(precharge=PC).findings[0].check == "precharge-time"
    dflt = ts.check_tractive_system(tsal=ts.TSAL(3.0, 55.0))
    assert dflt.findings[1].severity == Severity.OK                           # default rules: 60 V
    m = ts.TractiveSafetyResult()
    assert m.findings == [] and m.warnings == [] and m.findings is not ts.TractiveSafetyResult().findings
    mixed = ts.TractiveSafetyResult(findings=ts.check_precharge(ts.PrechargeCircuit(400.0, 600e-6, 10.0)))
    assert mixed.summary() == "WARNING: 0 fail / 1 warn / 2 missing / 0 ok across 3 tractive-safety checks"


def test_characterized_gates():
    from tests._golden import check
    res = {}
    pcs = {"ok": PC, "hot": ts.PrechargeCircuit(400.0, 600e-6, 1000.0, discharge_r_ohm=2000.0,
                                                resistor_energy_rating_j=10.0),
           "slowbleed": ts.PrechargeCircuit(400.0, 600e-6, 1000.0, discharge_r_ohm=9000.0),
           "unrated": ts.PrechargeCircuit(300.0, 400e-6, 500.0)}
    res["pc"] = {k: [f.as_dict() for f in ts.check_precharge(p)] for k, p in pcs.items()}
    res["bspd"] = [f.as_dict() for b, r in ((ts.BSPD(1.0, 1.0, 0.6505), ts.Rules(bspd_reaction_max_s=0.7505)),
                                            (ts.BSPD(1.0, 1.0, 0.8505), ts.Rules(bspd_reaction_max_s=0.7505)))
                   for f in ts.check_bspd(b, r)]
    res["chain"] = [f.as_dict() for f in ts.check_shutdown_chain(
        _chain(("a", "bspd", False, True), ("b", "imd", False, True)))]

    class Bypassed(ts.ShutdownChain):
        def is_closed(self, open_nodes=()):
            return bool({"l1", "l2"} & set(open_nodes)) or super().is_closed(open_nodes)
    c = Bypassed()
    for k in ts.REQUIRED_SHUTDOWN_NODES + ("l1", "l2"):
        c.add(ts.ShutdownNode(k, k if k in ts.REQUIRED_SHUTDOWN_NODES else "other", is_estimate=False))
    res["bypass"] = [f.as_dict() for f in ts.check_shutdown_chain(c)]
    strict = ts.Rules(bspd_reaction_max_s=0.1, safe_voltage_v=30.0, discharge_time_s=1.0, tsal_flash_hz_min=4.0)
    res["gate"] = ts.check_tractive_system(precharge=PC, chain=c, bspd=ts.BSPD(1.0, 1.0, 0.2), rules=strict).findings
    tr = ts.simulate_precharge(ts.PrechargeCircuit("x", 1e-3, 10.0))
    res["crash"] = [tr.i_a, tr.p_resistor_w, tr.ok]
    check("tractive_gates", res)
