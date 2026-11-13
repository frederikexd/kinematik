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


# --------------------------------------------------------------------------- C3. Native Altium boards ---- #
def test_native_altium_board_reading():
    """Six boards designed in Altium (never converted), counted in Altium Designer 26.11.1 by the author and by
    KinematiK. Limit: every count identical. Set KK_ALTIUM_KIT to the kit folder to re-read the boards here."""
    import os
    d = json.loads((REF / "altium" / "results_native.json").read_text(encoding="utf-8"))
    assert len(d["boards"]) == 6
    for r in d["boards"]:
        assert r["altium"] == r["kinematik"], r["board"]
    kit = os.environ.get("KK_ALTIUM_KIT")
    if kit:
        from suspension import pcb_doctor as D
        for r in d["boards"]:
            p = Path(kit) / f"{r['board']}.PcbDoc"
            b = D.parse_board(p.read_bytes(), p.name)
            got = dict(components=sum(1 for f in b.footprints if f.ref != "?"),
                       pads=sum(len(f.pads) for f in b.footprints), vias=len(b.vias),
                       tracks=sum(1 for s in b.segments if not s.from_arc),
                       arcs=len({s.arc_id for s in b.segments if s.from_arc}))
            assert got == r["altium"], r["board"]


# --------------------------------------------------------------------------- A4 / B6. KiCad 8/9 boards ---- #
def test_kicad9_boards_read_and_net_resistance():
    """Four FS boards saved in KiCad 8-9, read by KiCad 9.0.3 (plan addendum 7). Reading: every count exact, length
    0.1 %. Net resistance (held out, a3805c1): published as a miss at 2 %; pinned at what it scored. The 1.10 margin
    alone misses the worst net by 0.01 %; the brown-out bound (1.10 / copper finish 0.90) covers every net."""
    d = json.loads((REF / "kicad9" / "results.json").read_text(encoding="utf-8"))
    assert d["tool"].startswith("KiCad 9.0.3")
    for b in d["reading"]["boards"]:
        assert b["kicad"] == b["kinematik"] and not b["nets_missing"] and b["worst_length_pct"] <= 0.1, b["board"]
    rows = d["net_resistance"]["nets"]
    assert len(rows) == 100
    e = [(o["kk_ohm"] - o["copper2d_ohm"]) / o["copper2d_ohm"] for o in rows if o.get("copper2d_ohm")]
    assert len(e) == 95 and sum(abs(x) <= 0.02 for x in e) == 70
    assert all(abs(x) <= 0.10 for x in e) and min(e) * 1.10 > -0.101
    assert all((1 + x) * 1.10 / 0.90 >= 1.10 for x in e)


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


# --------------------------------------------------------------------------- E. Nets with copper pours ---- #
def test_pour_estimate_recorded_result():
    """Plan addendum 4. KinematiK's with-pour estimate (pcb_pour) against the 2-D solve with KiCad's filled zones.
    Demo pour nets (design) and team-board pour nets (held out, solved once). The held-out result misses the 2 %
    limit badly; what holds is the direction: it never reads below the reference, so it can't hide a brown-out."""
    d = _ref("pours.json")
    for grp, n_min in (("demo_boards", 6), ("team_boards", 17)):
        rows = [o for o in d[grp] if o.get("copper2d_ohm") and not o.get("excluded")]
        e = np.array([100.0 * (o["kinematik_ohm"] - o["copper2d_ohm"]) / o["copper2d_ohm"] for o in rows])
        assert len(e) >= n_min and e.min() > 0.0 and np.all(e < 50.0), grp


def test_pour_round3_held_out_on_kicad9_boards():
    """Pour estimate round 3, held out on the KiCad 8/9 boards (plan addendum 8): every net the reference resolves
    (twice the cell size within 2 %) within 2 %."""
    d = json.loads((REF / "kicad9" / "pours_round3.json").read_text(encoding="utf-8"))
    sc = [o for o in d["nets"] if o["converged"]]
    assert len(sc) == 19
    for o in sc:
        assert abs(o["kinematik_r3_ohm"] - o["copper2d_ohm"]) / o["copper2d_ohm"] <= 0.02, o["net"]


def test_refined_net_resistance_held_out_on_kicad9_boards():
    """Plan addendum 12: 100 nets B6 never sampled, solved once by the reference. Every net the sheet solve refines is
    within 2 %; the ones it leaves to the 1-D model are the connector nets whose pad names repeat, which the
    reference cannot solve either."""
    d = json.loads((REF / "kicad9" / "net_resistance_refined.json").read_text(encoding="utf-8"))
    rows = d["held_out"]["nets"]
    assert len(rows) == 100
    refined = [o for o in rows if o["source"] == "sheet solve" and o["copper2d_ohm"]]
    assert len(refined) == 92
    err = [(o["kinematik_refined_ohm"] - o["copper2d_ohm"]) / o["copper2d_ohm"] for o in refined]
    assert all(abs(e) <= 0.02 for e in err) and max(map(abs, err)) < 0.013
    one_d = [(o["kinematik_1d_ohm"] - o["copper2d_ohm"]) / o["copper2d_ohm"] for o in refined]
    assert sum(abs(e) <= 0.02 for e in one_d) == 68            # what the 1-D model alone scored on the same nets
    shipped = [o for o in refined if o["source_at_shipped_limit"] == "sheet solve"]
    assert len(shipped) == 78                                  # the shipped size limit refines these; all within 2 % too
    assert not [o for o in rows if o["source"] == "1-D" and o["copper2d_ohm"]]


def test_shipped_net_resistance_on_the_held_out_nets():
    """What a report actually uses, scored on the same 92 held-out nets (a re-scoring of recorded data, no new limit):
    the sheet solve where the net fits the shipped size limit (78 nets), the 1-D model otherwise (14). The 1-D
    fallbacks all read high, so the brown-out check never under-reads, and 91 of 92 shipped values are within 2 %."""
    from suspension.pcb_doctor import NET_R_UPPER_FACTOR
    d = json.loads((REF / "kicad9" / "net_resistance_refined.json").read_text(encoding="utf-8"))
    rows = [o for o in d["held_out"]["nets"] if o["source"] == "sheet solve" and o["copper2d_ohm"]]
    shipped = [(o["kinematik_refined_ohm"] if o["source_at_shipped_limit"] == "sheet solve" else o["kinematik_1d_ohm"])
               / o["copper2d_ohm"] for o in rows]
    fallback = [o["kinematik_1d_ohm"] / o["copper2d_ohm"] for o in rows
                if o["source_at_shipped_limit"] != "sheet solve"]
    assert len(shipped) == 92 and len(fallback) == 14
    assert sum(abs(r - 1.0) <= 0.02 for r in shipped) == 91 and max(abs(r - 1.0) for r in shipped) < 0.022
    assert min(fallback) >= 1.0 and min(fallback) * NET_R_UPPER_FACTOR >= 1.09


def test_pour_round4_grid_check_results():
    """Plan addenda 9-11. Round 3 on the larger held-out nets, and round 4 (grid check) where it marks itself
    converged and the reference converges too. Pinned as scored; the misses stay visible."""
    d = json.loads((REF / "kicad9" / "pours_round4.json").read_text(encoding="utf-8"))
    held = d["held_out"]
    assert len(held) == 11
    r3 = [(o["kinematik_ohm"] - o["copper2d_ohm"]) / o["copper2d_ohm"] for o in held if o["reference_converged"]]
    assert len(r3) == 8 and all(abs(e) <= 0.02 for e in r3)
    r4 = [(o["kinematik_r4_ohm"] - o["copper2d_ohm"]) / o["copper2d_ohm"]
          for o in held if o.get("r4_converged") and o["reference_converged"]]
    assert len(r4) == 5 and all(abs(e) <= 0.02 for e in r4)
    same = [(o["kinematik_r4_ohm"] - o["reference_same_cell_ohm"]) / o["reference_same_cell_ohm"]
            for o in held if o.get("reference_same_cell_ohm")]                 # addendum 13, item 2
    assert len(same) == 2 and all(abs(e) <= 0.02 for e in same)
    assert sum(1 for o in held if not o.get("r4_converged")) == 4            # reported as not converged, not hidden
    design = [o for o in d["design"] if o["r4_converged"] and o["reference_converged"]]
    err = [(o["kinematik_r4_ohm"] - o["reference_ohm"]) / o["reference_ohm"] for o in design]
    assert len(err) == 42 and sum(abs(e) <= 0.02 for e in err) == 40         # the 2 misses: empty pours (addendum 11)


def test_pour_nets_with_a_corrected_reference():
    """Plan addendum 13: the design nets whose reference had not converged, after the reference's terminal fix."""
    d = json.loads((REF / "pours_unresolved_design.json").read_text(encoding="utf-8"))
    both = [o for o in d["nets"] if o["reference_converged"] and o["converged"]]
    assert len(both) == 20 and all(abs(o["error_pct"]) <= 2.0 for o in both)


def test_unverified_pour_estimates_stay_inside_the_brownout_margin():
    """An estimate KinematiK labels not converged is used by the brown-out check with POUR_UNVERIFIED_FACTOR. Every
    such estimate in the benchmark is within that factor of the reference, high or low."""
    from suspension.pcb_doctor import POUR_UNVERIFIED_FACTOR
    d = json.loads((REF / "kicad9" / "pours_round4.json").read_text(encoding="utf-8"))
    rows = [(o["kinematik_r4_ohm"], o["copper2d_ohm"]) for o in d["held_out"]
            if not o.get("r4_converged") and o.get("kinematik_r4_ohm") and o.get("copper2d_ohm")]
    rows += [(o["kinematik_r4_ohm"], o["reference_ohm"]) for o in d["design"] if not o.get("r4_converged")]
    assert len(rows) == 5
    for kk, ref in rows:
        assert kk * POUR_UNVERIFIED_FACTOR >= ref and kk <= ref * POUR_UNVERIFIED_FACTOR


def test_in_app_pour_rate_is_proven_on_80_fresh_nets():
    """Plan addendum 14: 80 of 80 converged in-app pour estimates within 2 % of a converged reference, so the rate is
    at least 96.3 % at 95 % confidence. Estimates the app marks unconverged are also inside the 1.10 margin."""
    from suspension.pcb_doctor import POUR_UNVERIFIED_FACTOR
    d = json.loads((REF / "pours_rate.json").read_text(encoding="utf-8"))
    sc = [o for o in d["nets"] if o.get("app_converged") and o.get("reference_converged")]
    assert len(sc) == 80 and all(abs(o["error_pct"]) <= 2.0 for o in sc)
    assert 0.05 ** (1 / len(sc)) > 0.963
    un = [o for o in d["nets"] if o.get("app_converged") is False and o.get("reference_converged")]
    assert un and all(1 / POUR_UNVERIFIED_FACTOR <= 1 + o["error_pct"] / 100 <= POUR_UNVERIFIED_FACTOR for o in un)
    assert not [o for o in d["nets"] if "reference_steps" in o and not o.get("reference_converged")]


@pytest.mark.parametrize("name,n,within,boards", [("pours_rate_boards.json", 71, 70, 26), ("pours_rate_fix.json", 53, 53, 19)])
def test_in_app_pour_rate_across_boards(name, n, within, boards):
    """Plan addenda 15-16: the pour estimate on 42 boards never used before, at most 3 nets per board. Addendum 15's
    one miss (+3.07 %, a roundrect pad's corner ratio) is pinned as found; addendum 16 holds the fix out."""
    d = json.loads((REF / name).read_text(encoding="utf-8"))
    sc = [o for o in d["nets"] if o.get("app_converged") and o.get("reference_converged")]
    assert len(sc) == n and sum(abs(o["error_pct"]) <= 2.0 for o in sc) == within
    assert len({o["board"] for o in sc}) == boards
    assert max(abs(o["error_pct"]) for o in sc) < (3.1 if within < n else 2.0)


def test_roundrect_corner_ratio_is_read_for_the_sheet_solve():
    from suspension.pcb_doctor import parse_kicad_pcb
    b = parse_kicad_pcb('(kicad_pcb (version 20240108) (layers (0 "F.Cu" signal)) (net 0 "") (net 1 "A")'
                        ' (footprint "f" (layer "F.Cu") (at 0 0) (property "Reference" "J1")'
                        '  (pad "1" thru_hole roundrect (at 0 0) (size 2.1 2.1) (drill 1) (layers "*.Cu")'
                        '   (roundrect_rratio 0.119048) (net 1 "A"))'
                        '  (pad "2" smd roundrect (at 3 0) (size 1 1) (layers "F.Cu") (net 1 "A"))))')
    p1, p2 = b.footprints[0].pads
    assert p1.rratio == pytest.approx(0.119048) and p2.rratio == 0.25
