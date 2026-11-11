"""Electrical benchmark (docs/ELECTRICAL_BENCHMARK_PLAN.md): KinematiK against KiCad 7, ngspice and the atlc field
solver, re-checked on every run against the reference numbers recorded in benchmarks/electrical/.

The references were produced by scripts/electrical_reference.py with the tools installed. These tests need none of
them: they recompute KinematiK's side and compare. The KiCad board tests also need KiCad's demo boards
(/usr/share/kicad/demos, package kicad-demos) and are skipped where those aren't installed."""
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np
import pytest

REF = Path(__file__).resolve().parents[1] / "benchmarks" / "electrical"
DEMOS = Path("/usr/share/kicad/demos")
needs_demos = pytest.mark.skipif(not DEMOS.is_dir(), reason="KiCad demo boards not installed")


def _ref(name):
    return json.loads((REF / name).read_text(encoding="utf-8"))


# --------------------------------------------------------------------------- A. KiCad: reading a board ---- #
def _kinematik_board(path):
    from suspension import pcb_doctor as D
    b = D.parse_board((DEMOS / path).read_bytes(), path)
    per: dict = {}
    for s in b.segments:
        d = per.setdefault(b.net_name(s.net), {"length_mm": 0.0, "widths_mm": set()})
        d["length_mm"] += s.length_mm
        d["widths_mm"].add(round(s.width_mm, 4))
    return b, {"straight_tracks": sum(1 for s in b.segments if not s.from_arc),
               "arc_tracks": len({s.width_span for s in b.segments if s.from_arc}),
               "via_drills_mm": sorted(round(v.drill_mm, 4) for v in b.vias),
               "pads": sum(len(f.pads) for f in b.footprints), "footprints": len(b.footprints), "nets": per}


@needs_demos
@pytest.mark.parametrize("path", sorted(_ref("kicad_boards.json")["boards"]))
def test_board_read_as_kicad_reads_it(path):
    ref = _ref("kicad_boards.json")["boards"][path]
    _b, kk = _kinematik_board(path)
    for key in ("straight_tracks", "arc_tracks", "pads", "footprints"):
        assert kk[key] == ref[key], key
    assert Counter(kk["via_drills_mm"]) == Counter(ref["via_drills_mm"])
    assert set(kk["nets"]) == set(ref["nets"])
    for net, r in ref["nets"].items():
        k = kk["nets"][net]
        assert abs(k["length_mm"] - r["length_mm"]) <= max(1e-3 * r["length_mm"], 0.01), net
        assert sorted(k["widths_mm"]) == r["widths_mm"], net


def test_kicad_reference_covers_every_demo_board():
    boards = _ref("kicad_boards.json")["boards"]
    assert len(boards) == 14 and _ref("kicad_boards.json")["tool"].startswith("KiCad 7")
    assert sum(b["arc_tracks"] for b in boards.values()) > 0 and sum(len(b["via_drills_mm"]) for b in boards.values())


# --------------------------------------------------------------------------- B1. SPICE: precharge ---- #
@pytest.mark.parametrize("name", sorted(_ref("spice_precharge.json")["cases"]))
def test_precharge_matches_spice(name):
    from suspension.tractive_system import PrechargeCircuit, simulate_precharge
    c = _ref("spice_precharge.json")["cases"][name]
    pc = PrechargeCircuit(pack_voltage_v=c["V"], link_capacitance_f=c["C"], precharge_r_ohm=c["R"])
    tr = simulate_precharge(pc, t_end_s=c["t_end_s"], t_switch_s=c["t_switch_s"], n=200001)
    v = np.interp(c["times_s"], tr.time_s, tr.v_cap_v)
    assert np.max(np.abs(v - np.array(c["v_cap_v"]))) <= 1e-3 * c["V"]
    if c["t_switch_s"]:
        t95 = float(tr.time_s[np.argmax(tr.v_cap_v >= 0.95 * c["V"])])
        m = tr.time_s <= c["t_switch_s"]
        e = float(np.trapezoid(tr.p_resistor_w[m], tr.time_s[m]))
    else:
        t95, e = pc.time_to_fraction_s(0.95), pc.precharge_pulse_energy_j
    assert pc.time_to_fraction_s(0.9) == pytest.approx(c["t90_s"], rel=1e-3)
    assert t95 == pytest.approx(c["t95_s"], rel=1e-3)
    assert pc.peak_inrush_a == pytest.approx(c["i_peak_a"], rel=1e-3)
    assert e == pytest.approx(c["e_resistor_j"], rel=1e-3)


# --------------------------------------------------------------------------- B2. SPICE: pack over a lap ---- #
def test_pack_along_lap_matches_spice():
    from suspension.ev_electrical_check import pack_along_lap
    d = _ref("spice_pack_lap.json")
    x = d["inputs"]
    v, i, soc = pack_along_lap(np.array(x["p_terminal_w"]), np.array(x["t_s"]), ocv_soc=x["ocv_soc"],
                               soc0=x["soc0"], capacity_ah=x["capacity_ah"], r_pack_ohm=x["r_pack_ohm"])
    rv, ri, rs = (np.array(d[k]) for k in ("v_terminal_v", "i_a", "soc"))
    assert np.max(np.abs(v - rv) / rv) <= 0.005
    assert np.max(np.abs(i - ri) / np.maximum(np.abs(ri), 1.0)) <= 0.01      # 1 %, or 1 A near zero current
    assert abs(soc[-1] - rs[-1]) <= 0.001
    assert min(x["p_terminal_w"]) < 0 < max(x["p_terminal_w"])              # the lap includes regen


# --------------------------------------------------------------------------- B3. SPICE: net resistance ---- #
def _net_stats(rows):
    e = np.array([100.0 * (o["kinematik_ohm"] - o["spice_ohm"]) / o["spice_ohm"] for o in rows
                  if o["spice_ohm"] and 1e-7 < o["spice_ohm"] < 1e6])
    return e, np.abs(e)


def test_net_resistance_recorded_rounds():
    e1, a1 = _net_stats(_ref("nets_round1.json")["nets"])
    e2, a2 = _net_stats(_ref("nets_round2.json")["nets"])
    assert len(a1) >= 950 and len(a2) >= 950
    # round 1 missed the registered 2 % limit and read unsafely low; the fixes moved both
    assert np.mean(a1 <= 2) < 0.65 and e1.min() < -95
    assert np.mean(a2 <= 2) >= 0.72 and np.mean(a2 <= 10) >= 0.91 and e2.min() > -25


def _round3(rows, ver):
    k, c = ("kinematik_3a_ohm", "copper2d_3a_ohm") if ver == "3a" else ("kinematik_ohm", "copper2d_ohm")
    e = np.array([100.0 * (o[k] - o[c]) / o[c] for o in rows if o.get(k) and o.get(c) and 0 < o[c] < 1e6])
    return e, np.abs(e)


def test_net_resistance_round3_against_2d_copper():
    """Round 3 (plan addendum 2): a 2-D solve of the copper is the reference. Before the changes KinematiK read high
    (median +10.7 % on the demo boards); after, on the demo boards the changes were designed on and on 20 team boards
    held out until the model was frozen."""
    d = _ref("nets_round3.json")
    e3a, _ = _round3(d["demo_boards"], "3a")
    assert len(e3a) >= 850 and np.median(e3a) > 8
    e, a = _round3(d["demo_boards"], "now")
    assert len(e) >= 850 and np.mean(a <= 2) >= 0.89 and np.mean(a <= 5) >= 0.97 and e.min() > -11.2
    t, ta = _round3(d["team_boards"], "now")
    assert len(t) >= 380 and np.mean(ta <= 2) >= 0.87 and np.mean(ta <= 5) >= 0.98
    # (the brown-out margin is checked against the corrected reference, below)


def test_net_resistance_against_corrected_2d_reference():
    """Plan addendum 3: the corrected 2-D reference. Design set: demo boards. Held out: 150 sampled round-3 team
    nets, scored once with KinematiK frozen at a3805c1. Misses of the 2 % limit are published; the worst low reading
    sets the brown-out margin (NET_R_UPPER_FACTOR)."""
    from suspension.pcb_doctor import NET_R_UPPER_FACTOR
    d = _ref("nets_round3_corrected.json")
    e, a = _round3(d["demo_boards"], "now")
    assert len(e) >= 680 and np.mean(a <= 2) >= 0.92 and np.mean(a <= 5) >= 0.98 and e.min() > -7.5
    t, ta = _round3(d["team_boards"], "now")
    assert len(t) >= 140 and np.mean(ta <= 2) >= 0.87 and np.mean(ta <= 5) >= 0.97
    assert np.all(100.0 / (100.0 + t) <= NET_R_UPPER_FACTOR)       # the margin covers every held-out net
    t3, _ = _round3(d["team_boards"], "3a")
    assert np.median(t3) > 10                                          # before round 3: a median 10–17 % high


@needs_demos
def test_net_resistance_still_what_was_recorded():
    """KinematiK's worst pad-to-pad resistance per net is still what was recorded (the 2-D side is fixed)."""
    from suspension import pcb_doctor as D
    rows = [o for o in _ref("nets_round3_corrected.json")["demo_boards"] if o.get("kinematik_ohm")]
    boards: dict = {}
    for o in rows[::9]:                                   # every 9th net keeps this quick
        if o["board"] not in boards:
            boards[o["board"]] = D.parse_board((DEMOS / o["board"]).read_bytes(), o["board"])
        b = boards[o["board"]]
        r = D.analyze_net(b, b.net_id(o["net"]))
        assert r["worst_r_ohm"] == pytest.approx(o["kinematik_ohm"], rel=1e-9), o["net"]
        assert list(r["worst_pair"]) == o["pair"], o["net"]


# --------------------------------------------------------------------------- C. Field solver: impedance ---- #
def test_field_solver_checked_against_exact_stripline():
    for c in _ref("atlc_impedance.json")["stripline_checks"]:
        exact = c["exact_ohm"]
        assert abs(c["atlc_ohm"] - exact) / exact <= 0.01


@pytest.mark.parametrize("name", ["atlc_impedance.json", "atlc_impedance_round2.json"])
def test_microstrip_impedance_within_limits(name):
    from suspension.electronics import DiffPair
    d = _ref(name)
    for x in d["single_ended"]:
        z = DiffPair("x", "e", trace_w_mm=x["w_mm"], height_mm=x["h_mm"],
                     eps_r=x.get("er", 4.3)).single_ended_z0_ohm()
        assert abs(z - x["atlc_z0_ohm"]) / x["atlc_z0_ohm"] <= 0.05, x
    for x in d["differential"]:
        z = DiffPair("x", "e", trace_w_mm=x["w_mm"], height_mm=x["h_mm"], eps_r=x.get("er", 4.3),
                     spacing_mm=x["s_mm"]).differential_z0_ohm()
        assert abs(z - x["atlc_zdiff_ohm"]) / x["atlc_zdiff_ohm"] <= 0.10, x


def test_round1_ipc2141_miss_is_on_record():
    """The formula KinematiK used before (IPC-2141) missed the 5 % limit at both ends of the range."""
    d = _ref("atlc_impedance.json")["single_ended"]
    ipc = [87.0 / math.sqrt(x["er"] + 1.41) * math.log(5.98 * x["h_mm"] / (0.8 * x["w_mm"] + x["t_mm"])) for x in d]
    errs = [100.0 * (z - x["atlc_z0_ohm"]) / x["atlc_z0_ohm"] for z, x in zip(ipc, d)]
    assert errs[0] < -5 and errs[-1] < -5 and all(abs(e) <= 5 for e in errs[1:-1])


# --------------------------------------------------------------------------- C2. Altium's impedance calculator ---- #
def test_microstrip_impedance_against_altium():
    """The 21 registered cross-sections (plus 5 extras) in Altium Designer 26.11.1's Layer Stack Manager, run by the
    author from the kit (benchmarks/electrical/altium/). Limits as C: 5 % single-ended, 10 % differential. Altium and
    the open field solver atlc are also held to each other, which checks the reference."""
    from suspension.electronics import DiffPair
    d = json.loads((REF / "altium" / "results_part2_impedance.json").read_text(encoding="utf-8"))
    reg = [c for c in d["cases"] if not c["case"].startswith("extra")]
    assert len(reg) == 21 and d["tool"].startswith("Altium Designer 26")
    for c in d["cases"]:
        dp = DiffPair("x", "e", trace_w_mm=c["w_mm"], height_mm=c["h_mm"], eps_r=c["er"],
                      spacing_mm=c["s_mm"] or 0.2)
        z = dp.single_ended_z0_ohm() if c["kind"] == "single" else dp.differential_z0_ohm()
        lim = 0.05 if c["kind"] == "single" else 0.10
        assert abs(z - c["altium_ohm"]) / c["altium_ohm"] <= lim, c["case"]
        if c["atlc_ohm"]:
            assert abs(c["atlc_ohm"] - c["altium_ohm"]) / c["altium_ohm"] <= 0.02, c["case"]


# --------------------------------------------------------------------------- D. Trace heating vs NBS ---- #
def test_trace_heating_against_nbs_measurements():
    """Plan addendum 5. KinematiK's ampacity (IPC-2221 × 0.975) against the NBS 1956 measured curves. Points at 4
    and 16 sq mil are left out (the curves converge within the reading resolution). Plain IPC-2221 missed the 5 %
    limit on two wide traces at a 10 °C rise; the 0.975 factor was set on these same points, so this is a
    calibration check."""
    import numpy as np
    from suspension.pcb_doctor import trace_ampacity_a, MM2_TO_MIL2
    d = _ref("nbs_heating.json")["points"]
    used = [o for o in d if "excluded" not in o]
    assert len(d) == 54 and len(used) == 44
    ratios, ipc = [], []
    for o in used:
        width_mm = o["area_sqmil"] / MM2_TO_MIL2 / 0.03479            # 1 oz copper, the same area
        i = trace_ampacity_a(width_mm, o["dT_c"], 1.0, external=True)
        assert i == pytest.approx(o["i_kinematik_a"], rel=2e-3)
        ratios.append(i / o["i_read1_a"]); ipc.append(o["i_ipc2221_a"] / o["i_read1_a"])
    r, q = np.array(ratios), np.array(ipc)
    assert r.max() <= 1.05 and 0.86 <= np.median(r) <= 0.92
    assert int(np.sum(q > 1.05)) == 2                                     # plain IPC-2221's misses, on record
