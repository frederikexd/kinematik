"""suspension/wiring.py against the published tables it quotes and the square-root law it derives: every table
entry, the ambient correction against the NEC factors, the column scaling with the copper resistance shift, bundling,
terminations, the round-trip voltage drop, the gauge ladder, fuse coordination and RMS sizing from a log, with every
note and verdict checked word for word."""
import math

import numpy as np
import pytest

from suspension import wiring as w

A = 0.00393


def test_the_tables():
    rows = {"22": (0.326, 52.96, None, None, None, False), "20": (0.518, 33.31, None, None, None, False),
            "18": (0.823, 20.95, None, None, None, False), "16": (1.31, 13.17, None, None, None, False),
            "14": (2.08, 8.286, 15.0, 20.0, 25.0, True), "12": (3.31, 5.211, 20.0, 25.0, 30.0, True),
            "10": (5.26, 3.277, 30.0, 35.0, 40.0, True), "8": (8.37, 2.061, 40.0, 50.0, 55.0, True),
            "6": (13.3, 1.296, 55.0, 65.0, 75.0, True), "4": (21.2, 0.8152, 70.0, 85.0, 95.0, True),
            "3": (26.7, 0.6465, 85.0, 100.0, 115.0, True), "2": (33.6, 0.5127, 95.0, 115.0, 130.0, True),
            "1": (42.4, 0.4065, None, 130.0, 145.0, True), "1/0": (53.5, 0.3224, None, 150.0, 170.0, True),
            "2/0": (67.4, 0.2557, None, 175.0, 195.0, True), "3/0": (85.0, 0.2028, None, 200.0, 225.0, True),
            "4/0": (107.2, 0.1608, None, 230.0, 260.0, True)}
    assert list(w.AWG) == list(rows) == w._ORDER
    for k, (a, r, c60, c75, c90, nec) in rows.items():
        assert w.AWG[k] == w.Conductor(k, a, r, c60, c75, c90, nec)
    assert w.TEMP_CORRECTION == {
        75: [(35, 0.94), (40, 0.88), (45, 0.82), (50, 0.75), (55, 0.67), (60, 0.58), (70, 0.33)],
        90: [(35, 0.96), (40, 0.91), (45, 0.87), (50, 0.82), (55, 0.76), (60, 0.71), (70, 0.58), (80, 0.41)],
        60: [(35, 0.91), (40, 0.82), (45, 0.71), (50, 0.58)]}
    assert w.BUNDLE_ADJUSTMENT == [(3, 1.00), (6, 0.80), (9, 0.70), (20, 0.50), (30, 0.45), (40, 0.40), (999, 0.35)]
    assert {k: v[0] for k, v in w.INSULATIONS.items()} == {"nmb": 60, "thw": 75, "thhn": 90, "xhhw2": 90,
                                                           "tefzel": 150, "silicone": 200, "ptfe": 200, "xlpe125": 125}
    assert w.INSULATIONS["thhn"][1] == "THHN / THWN-2 / XHHW-2 building wire — the column the NEC table is anchored on."
    assert w.INSULATIONS["tefzel"][1] == ("M22759/16 ETFE (Tefzel), 150 °C continuous. The aerospace-derived wire most "
                                          "FSAE teams actually run.")
    assert w.INSULATIONS["silicone"][1] == ("Fine-strand silicone, 200 °C continuous. Very flexible, poor abrasion "
                                            "resistance — sleeve it.")
    assert (w.INSULATIONS["nmb"][1], w.INSULATIONS["thw"][1], w.INSULATIONS["xhhw2"][1], w.INSULATIONS["ptfe"][1],
            w.INSULATIONS["xlpe125"][1]) == ("NM-B / UF-B building wire.", "THW / THWN building wire.",
                                             "XHHW-2 building wire.", "M22759/11 PTFE, 200 °C continuous.",
                                             "Cross-linked polyolefin, 125 °C continuous.")
    assert w.TERMINATIONS == {"ring_lug_105": (105, "Typical crimp ring lug / heat-shrink boot."),
                              "connector_125": (125, "Deutsch DTM / Autosport-style contact."),
                              "connector_150": (150, "High-temperature contact system."),
                              "busbar_200": (200, "Bolted busbar with high-temperature hardware.")}
    assert w.SOURCE == ("NEC 2017 building-wire ampacities as published in Cerrowire's ampacity chart (cerrowire.com). "
                        "Temperature correction and bundling adjustment are NOT included in those figures and are "
                        "applied separately here.")
    assert (w._NEC_AMBIENT_C, w._ALPHA_CU) == (30.0, 0.00393)
    assert w._COLUMNS == [(90, "a_90c"), (75, "a_75c"), (60, "a_60c")]
    c = w.AWG["6"]
    assert (c.base(60), c.base(75), c.base(90), c.base(105)) == (55.0, 65.0, 75.0, None)


def test_the_correction_factor_is_the_published_table():
    for rating, rows in w.TEMP_CORRECTION.items():
        for amb, pub in rows:
            got = w.correction_factor(rating, amb)
            assert got == pytest.approx(math.sqrt((rating - amb) / (rating - 30)), rel=1e-12)
            assert abs(got - pub) < 0.005
    assert w.correction_factor(90, 30) == 1.0
    assert w.correction_factor(90, 90) == 0.0 and w.correction_factor(90, 95) == 0.0
    assert w.correction_factor(30, 20) == 0.0 and w.correction_factor(25, 10) == 0.0
    assert w.correction_factor(30.5, 20) == pytest.approx(math.sqrt(10.5 / 0.5))
    with pytest.raises(ValueError) as e:
        w.correction_factor(90, 40, include_resistance_shift=True)
    assert str(e.value) == ("correction_factor() has no resistance shift to apply — see the note in its docstring. "
                            "Use ampacity_scale() if you are scaling ACROSS rating temperatures.")


def test_the_column_scaling():
    assert w._column_for(200) == (90, "a_90c") and w._column_for(90) == (90, "a_90c")
    assert w._column_for(89.9) == (75, "a_75c") and w._column_for(75) == (75, "a_75c")
    assert w._column_for(74.9) == (60, "a_60c") and w._column_for(60) == (60, "a_60c")
    assert w._column_for(40) == (60, "a_60c")
    for rating, amb in ((200, 60), (150, 30), (90, 45), (80, 40), (65, 35), (50, 20)):
        f, col = w.ampacity_scale(rating, amb)
        c = w._column_for(rating)[0]
        want = math.sqrt((rating - amb) / (c - 30)) * math.sqrt((1 + A * (c - 20)) / (1 + A * (rating - 20)))
        assert col == c and f == pytest.approx(want, rel=1e-12)
        f2, _ = w.ampacity_scale(rating, amb, include_resistance_shift=False)
        assert f2 == pytest.approx(math.sqrt((rating - amb) / (c - 30)), rel=1e-12)
    assert w.ampacity_scale(90, 90) == (0.0, 90) and w.ampacity_scale(150, 160) == (0.0, 90)
    assert w.ampacity_scale(90, 30)[0] == pytest.approx(1.0, rel=1e-15)


def test_conductor_lookup_and_ratings():
    assert w.conductor(" awg 1/0 ") is w.AWG["1/0"] and w.conductor("6AWG") is w.AWG["6"]
    assert w.conductor(6) is w.AWG["6"] and w.conductor("4/0") is w.AWG["4/0"]
    with pytest.raises(KeyError) as e:
        w.conductor("5")
    assert e.value.args[0] == "unknown gauge '5' — have " + ", ".join(w._ORDER)
    bands = [(3.0, 1.0), (6.0, 0.8), (9.0, 0.7)]
    assert [w._lookup(bands, v) for v in (1, 3, 3.5, 6, 9, 10)] == [1.0, 1.0, 0.8, 0.8, 0.7, 0.7]
    assert w.rating_of("TeFzEl") == 150 and w.rating_of("mystery") == 90


def test_ampacity():
    for k in ("14", "6", "4/0"):
        assert w.ampacity(k) == pytest.approx(w.AWG[k].a_90c, rel=1e-12)
    assert w.ampacity("16") is None
    f, _ = w.ampacity_scale(150, 60)
    assert w.ampacity("6", "tefzel", 60.0) == pytest.approx(75.0 * f, rel=1e-12)
    f, _ = w.ampacity_scale(60, 40)
    assert w.ampacity("8", "nmb", 40.0) == pytest.approx(40.0 * f, rel=1e-12)
    assert w.ampacity("1", "nmb") is None
    assert w.ampacity("6", "thw", 45.0) == pytest.approx(65.0 * w.ampacity_scale(75, 45)[0], rel=1e-12)


# ------------------------------------------------------------------ derating
NO_TERM = ("No termination rating stated, so the conductor's own rating was used. On a real car this is usually "
           "optimistic — crimps, boots and connector bodies are typically 105–125 °C, and they, not the wire, set "
           "the limit.")


def _scale_note(col, eff, amb, tf):
    return (f"Ampacity derived from the published {col} °C column, scaled by √((T_rating − T_ambient)/(T_column − 30)) "
            f"with the copper resistance shift included: {eff:g} °C conductor in a {amb:g} °C ambient → ×{tf:.3f}.")


def test_a_plain_derate():
    d = w.derate("6")
    assert (d.conductor, d.insulation, d.rating_c, d.base_a, d.temp_factor, d.bundle_factor) == (
        w.AWG["6"], "thhn", 90, 75.0, pytest.approx(1.0), 1.0)
    assert d.allowed_a == pytest.approx(75.0) and d.total_factor == pytest.approx(1.0)
    assert d.notes == ["THHN / THWN-2 / XHHW-2 building wire — the column the NEC table is anchored on. Continuous "
                       "rating 90 °C.", _scale_note(90, 90.0, 30.0, 1.0), NO_TERM]


def test_a_hot_bundle_and_a_low_termination():
    d = w.derate("10", insulation="silicone", ambient_c=65.0, n_bundled=12, termination_c=105.0)
    tf, col = w.ampacity_scale(105.0, 65.0)
    assert (d.rating_c, d.base_a, d.bundle_factor, col) == (105, 40.0, 0.5, 90)
    assert d.temp_factor == pytest.approx(tf) and d.allowed_a == pytest.approx(40.0 * tf * 0.5)
    assert d.total_factor == pytest.approx(tf * 0.5)
    assert d.notes == [
        "Fine-strand silicone, 200 °C continuous. Very flexible, poor abrasion resistance — sleeve it. Continuous "
        "rating 200 °C.",
        _scale_note(90, 105.0, 65.0, tf),
        "**Termination-limited.** The conductor is rated 200 °C but it lands in a 105 °C termination, so the circuit "
        "is a 105 °C circuit. Every amp above this is bought by upgrading the lug, not the cable.",
        "12 current-carrying conductors bundled → ×0.50. Only conductors actually carrying current count; a spare or "
        "a signal pair does not.",
        f"Combined derate is ×{tf * 0.5:.2f}: this conductor may carry {40.0 * tf * 0.5:.0f} A where the 90 °C chart "
        "says 40 A."]


def test_high_temperature_headroom_and_odd_inputs():
    d = w.derate("6", insulation="tefzel", ambient_c=30.0, termination_c=200.0)
    tf, _ = w.ampacity_scale(150.0, 30.0)
    assert d.rating_c == 150 and d.allowed_a == pytest.approx(75.0 * tf)
    assert d.notes == [
        "M22759/16 ETFE (Tefzel), 150 °C continuous. The aerospace-derived wire most FSAE teams actually run. "
        "Continuous rating 150 °C.",
        _scale_note(90, 150.0, 30.0, tf),
        f"This conductor beats the 90 °C chart value ({75.0 * tf:.0f} A vs 75 A) because its insulation is rated well "
        "above the column the chart is built on. That headroom is real and it is mass you do not have to carry — but "
        "it is only collectable if the terminations are rated for it too."]
    same = w.derate("6", insulation="tefzel", termination_c=150.0)
    assert same.rating_c == 150 and not any("Termination" in n or "No termination" in n for n in same.notes)
    small = w.derate("18", insulation="weird", n_bundled=0)
    assert small.base_a is None and small.allowed_a is None and small.bundle_factor == 1.0 and small.rating_c == 90
    assert small.notes[0] == "unknown Continuous rating 90 °C."
    assert small.notes[1] == ("18 AWG is not in the NEC building-wire table, which starts at 14 AWG. Area and "
                              "resistance below are physical constants and are valid; the ampacity is left blank "
                              "rather than invented. Use the wire manufacturer's chassis-wiring rating for this gauge.")
    three = w.derate("6", n_bundled=3)
    assert three.bundle_factor == 1.0 and len(three.notes) == 3
    four = w.derate("6", n_bundled=4)
    assert four.bundle_factor == 0.8 and four.notes[-1].startswith("4 current-carrying conductors bundled → ×0.80.")
    sixty = w.derate("6", n_bundled=7)
    assert sixty.allowed_a == pytest.approx(52.5) and not any("Combined derate" in n for n in sixty.notes)
    lower = w.derate("6", n_bundled=10)
    assert any(n.startswith("Combined derate is ×0.50") for n in lower.notes)
    hot = w.derate("6", ambient_c=95.0)
    assert hot.allowed_a == 0.0 and hot.temp_factor == 0.0
    nmb = w.derate("1", insulation="nmb")
    assert nmb.base_a is None and nmb.allowed_a is None and nmb.rating_c == 60
    up = w.derate("6", insulation="THHN")
    assert up.notes[0].startswith("THHN / THWN-2")


# ------------------------------------------------------------------ a run
def _run_oracle(awg, I, L, V, temp=None, amb=30.0, **kw):
    d = w.derate(awg, ambient_c=amb, **kw)
    c = w.AWG[awg]
    t = temp if temp is not None else max(amb + 20.0, 40.0)
    r = c.r_ohm_per_km_20c * (1 + A * (t - 20)) / 1000 * 2 * L
    drop = I * r
    return d, t, r, drop, 100 * drop / max(V, 1e-9), I * I * r


def test_a_thermally_limited_run():
    rc = w.check_run("10", current_a=50.0, length_m=0.5, system_v=400.0, n_bundled=4)
    d, t, r, drop, pct, loss = _run_oracle("10", 50.0, 0.5, 400.0, n_bundled=4)
    assert (rc.awg, rc.current_a, rc.length_m, rc.system_v) == ("10", 50.0, 0.5, 400.0)
    assert rc.derated_a == pytest.approx(d.allowed_a) and rc.thermal_margin == pytest.approx(d.allowed_a / 50.0)
    assert (rc.resistance_ohm, rc.drop_v, rc.drop_pct, rc.loss_w) == (pytest.approx(r), pytest.approx(drop),
                                                                       pytest.approx(pct), pytest.approx(loss))
    assert rc.governing == "ampacity" and rc.ok is False
    assert loss < 25.0 and rc.notes == d.notes + [
        f"Round trip 1 m of 10 AWG at {t:.0f} °C → {r * 1000:.2f} mΩ. At 50 A that is {drop:.2f} V ({pct:.2f} % of "
        f"400 V) and {loss:.0f} W burned in the cable."]
    big = w.check_run("10", current_a=50.0, length_m=2.0, system_v=400.0, n_bundled=4)
    lw = big.loss_w
    assert lw > 25.0 and big.notes[-1] == (f"{lw:.0f} W into the loom is also {lw:.0f} W the cooling system never "
                                           "budgeted for, deposited along a cable that is probably cable-tied to "
                                           "something you care about.")


def test_a_voltage_drop_limited_run():
    rc = w.check_run("16", current_a=5.0, length_m=3.0, system_v=12.0, conductor_temp_c=60.0)
    d, t, r, drop, pct, loss = _run_oracle("16", 5.0, 3.0, 12.0, temp=60.0)
    assert rc.thermal_margin is None and rc.derated_a is None and rc.ok is False
    assert rc.governing == "voltage drop (no ampacity available for this gauge)"
    assert rc.drop_pct == pytest.approx(pct) and rc.resistance_ohm == pytest.approx(r)
    assert rc.notes[-1] == ("**Not cleared.** 16 AWG has no ampacity in the NEC building-wire table, so the thermal "
                            "check could not be made. Unknown is not a pass — get the manufacturer's chassis-wiring "
                            "rating for this gauge before using it at 5 A.")
    v = w.check_run("14", current_a=10.0, length_m=4.0, system_v=12.0)
    assert v.governing == "voltage drop" and v.ok is False
    assert ("**Voltage drop governs, not heat.** The conductor is thermally comfortable and still losing volts. This "
            "is the usual answer for long low-voltage runs and it is the one nobody checks — a browned-out ECU on a "
            "starter crank is this, every time.") in v.notes
    assert not any(n.endswith("you care about.") for n in v.notes)


def test_run_edges():
    hot = w.check_run("6", current_a=10.0, length_m=1.0, system_v=12.0, ambient_c=25.0)
    assert hot.resistance_ohm == pytest.approx(_run_oracle("6", 10.0, 1.0, 12.0, amb=25.0)[2])
    warm = w.check_run("6", current_a=10.0, length_m=1.0, system_v=12.0, ambient_c=45.0)
    assert warm.resistance_ohm == pytest.approx(_run_oracle("6", 10.0, 1.0, 12.0, amb=45.0)[2])
    zero_v = w.check_run("6", current_a=1.0, length_m=1.0, system_v=0.0)
    assert zero_v.drop_pct == pytest.approx(100 * zero_v.drop_v / 1e-9)
    idle = w.check_run("6", current_a=0.0, length_m=1.0, system_v=12.0)
    assert idle.thermal_margin is None and idle.governing.startswith("voltage drop (no ampacity")
    nolen = w.check_run("6", current_a=10.0, length_m=0.0, system_v=12.0)
    assert nolen.drop_pct == 0.0 and nolen.governing == "ampacity" and nolen.ok is True
    at = w.derate("6").allowed_a
    exact = w.check_run("6", current_a=at, length_m=0.1, system_v=400.0)
    assert exact.thermal_margin == 1.0 and exact.ok is True
    over = w.check_run("6", current_a=at * 1.001, length_m=0.1, system_v=400.0)
    assert over.ok is False
    pct = w.check_run("6", current_a=10.0, length_m=2.0, system_v=12.0).drop_pct
    assert w.check_run("6", current_a=10.0, length_m=2.0, system_v=12.0, max_drop_pct=pct).ok is True
    assert w.check_run("6", current_a=10.0, length_m=2.0, system_v=12.0, max_drop_pct=pct * 0.999).ok is False
    term = w.check_run("6", current_a=10.0, length_m=1.0, system_v=12.0, insulation="silicone", termination_c=105.0,
                       n_bundled=5)
    assert term.derated_a == pytest.approx(w.derate("6", insulation="silicone", termination_c=105.0,
                                                    n_bundled=5).allowed_a)
    small_loss = w.check_run("6", current_a=100.0, length_m=1.0, system_v=400.0, conductor_temp_c=20.0)
    assert small_loss.loss_w == pytest.approx(100 ** 2 * 1.296e-3 * 2)
    assert any(n.startswith("26 W into the loom") for n in small_loss.notes)


def test_parallel_runs_and_the_ladder():
    a = w.derate("4/0", ambient_c=50.0, n_bundled=4).allowed_a
    assert w.parallel_needed(900.0, ambient_c=50.0, n_bundled=4) == (math.ceil(900.0 / a), a)
    assert w.parallel_needed(10.0, awg="18") == (0, 0.0)
    assert w.parallel_needed(10.0, insulation="tefzel", awg="6") == (1, w.derate("6", insulation="tefzel").allowed_a)
    pick, ladder = w.recommend_gauge(current_a=40.0, length_m=1.0, system_v=400.0)
    assert [r.awg for r in ladder] == w._ORDER and all(r.current_a == 50.0 for r in ladder)
    assert pick == next(r.awg for r in ladder if r.ok) == "8"
    pick2, ladder2 = w.recommend_gauge(current_a=40.0, length_m=6.0, system_v=12.0, insulation="tefzel",
                                       ambient_c=60.0, n_bundled=5, termination_c=125.0, max_drop_pct=5.0, fos=1.0)
    want = [w.check_run(a, current_a=40.0, length_m=6.0, system_v=12.0, insulation="tefzel", ambient_c=60.0,
                        n_bundled=5, termination_c=125.0, max_drop_pct=5.0) for a in w._ORDER]
    assert [r.ok for r in ladder2] == [r.ok for r in want] and [r.drop_pct for r in ladder2] == [r.drop_pct for r in want]
    assert pick2 == next((r.awg for r in want if r.ok), None)
    none, _ = w.recommend_gauge(current_a=2000.0, length_m=1.0, system_v=400.0)
    assert none is None


def test_fuse_coordination():
    a = w.derate("10").allowed_a
    assert w.fuse_coordination("10", a + 1) == (
        "violation", f"**The {a + 1:g} A fuse does not protect 10 AWG.** Derated ampacity is {a:.0f} A, so the wire is "
                     "the weakest element in this path — it becomes the fuse, somewhere inside a loom, on track. Either "
                     f"drop the fuse to {a:.0f} A or step the wire up.")
    assert w.fuse_coordination("10", a)[0] == "watch"
    assert w.fuse_coordination("10", 38.0) == (
        "watch", f"38 A fuse against {a:.0f} A of derated wire — protected, but with under 10 % margin. Fuse curves "
                 "are slow and hot fuses drift; leave more room.")
    assert w.fuse_coordination("10", 0.9 * a) == (
        "ok", f"{0.9 * a:g} A fuse against {a:.0f} A of derated wire — the fuse opens first, which is the point.")
    assert w.fuse_coordination("18", 5.0) == (
        "unknown", "18 AWG has no table ampacity, so fuse coordination cannot be checked from this data. Get the "
                   "manufacturer's rating before trusting the 5 A fuse.")
    hot = w.derate("10", insulation="tefzel", ambient_c=60.0, n_bundled=4).allowed_a
    assert w.fuse_coordination("10", hot * 0.95, insulation="tefzel", ambient_c=60.0, n_bundled=4)[0] == "watch"


# ------------------------------------------------------------------ sizing from a log
def test_sizing_from_a_log():
    log = [10.0, -20.0, 30.0, np.nan, 5.0, np.inf, 0.0]
    s = w.size_from_log(log, length_m=1.0, system_v=400.0, n_bundled=2, ambient_c=40.0)
    i = np.array([10.0, -20.0, 30.0, 5.0, 0.0])
    rms = math.sqrt(np.mean(i ** 2))
    assert (s.rms_a, s.peak_a, s.mean_a) == (pytest.approx(rms), 30.0, pytest.approx(13.0))
    assert s.crest == pytest.approx(30.0 / rms) and s.duty_over_rms == pytest.approx(0.4)
    rec, _ = w.recommend_gauge(current_a=rms, length_m=1.0, system_v=400.0, ambient_c=40.0, n_bundled=2)
    pk, _ = w.recommend_gauge(current_a=30.0, length_m=1.0, system_v=400.0, ambient_c=40.0, n_bundled=2)
    assert (s.recommended_awg, s.peak_awg) == (rec, pk)
    assert s.notes[0] == f"RMS **{rms:.1f} A**, peak **30.0 A**, mean 13.0 A — crest factor {30.0 / rms:.2f}."
    assert s.notes[1] == "The car spends 40% of the log above its own RMS."
    if rec != pk:
        assert s.notes[2] == (f"Sizing on RMS gives {rec} AWG; sizing on the peak gives {pk} AWG. The difference is "
                              "copper you would carry for the whole event to survive the worst few seconds — worth it "
                              "for voltage drop at full power, not for heat.")
    flat = w.size_from_log([20.0, 20.0, -20.0], length_m=1.0, system_v=400.0)
    assert flat.crest == 1.0 and flat.duty_over_rms == 0.0 and flat.recommended_awg == flat.peak_awg
    assert flat.notes[2] == f"RMS and peak both land on {flat.recommended_awg} AWG, so there is no trade to make here."
    assert len(flat.notes) == 3
    spike = w.size_from_log([0.0] * 8 + [300.0], length_m=1.0, system_v=400.0)
    assert spike.crest == pytest.approx(3.0) and len(spike.notes) == 3
    spikier = w.size_from_log([0.0] * 15 + [300.0], length_m=1.0, system_v=400.0)
    assert spikier.crest == pytest.approx(4.0) and spikier.recommended_awg != spikier.peak_awg
    assert spikier.notes[-1] == ("A crest factor of 4.0 is high. Check that the peak is real current and not a sensor "
                                 "artefact before sizing anything around it — one bad sample sets this number.")
    zeros = w.size_from_log([0.0, 0.0], length_m=1.0, system_v=12.0)
    assert math.isnan(zeros.crest) and zeros.rms_a == 0.0
    empty = w.size_from_log([np.nan, np.inf], length_m=1.0, system_v=12.0)
    assert math.isnan(empty.rms_a) and math.isnan(empty.peak_a) and math.isnan(empty.mean_a)
    assert math.isnan(empty.crest) and math.isnan(empty.duty_over_rms)
    assert (empty.recommended_awg, empty.peak_awg, empty.notes) == (None, None, ["No usable current samples."])
    tef = w.size_from_log([60.0, 80.0], length_m=2.0, system_v=48.0, insulation="tefzel", max_drop_pct=2.0)
    r2, _ = w.recommend_gauge(current_a=math.sqrt((3600 + 6400) / 2), length_m=2.0, system_v=48.0,
                              insulation="tefzel", max_drop_pct=2.0)
    assert tef.recommended_awg == r2


def test_every_option_is_handed_on():
    """recommend_gauge and size_from_log must pass each setting to the runs they check; compare against direct
    calls with the same, non-default settings."""
    kw = dict(insulation="nmb", ambient_c=55.0, n_bundled=7, termination_c=75.0)
    best, runs = w.recommend_gauge(current_a=60.0, length_m=3.0, system_v=48.0, max_drop_pct=1.5, **kw)
    for r in runs:
        direct = w.check_run(r.awg, current_a=60.0 * 1.25, length_m=3.0, system_v=48.0, max_drop_pct=1.5, **kw)
        assert r == direct
    log = [10.0, 40.0, 25.0, 60.0, 5.0, 30.0]
    s = w.size_from_log(log, length_m=2.5, system_v=24.0, insulation="nmb", ambient_c=50.0, n_bundled=9,
                        max_drop_pct=1.0)
    s_def = w.size_from_log(log, length_m=2.5, system_v=24.0)
    assert s != s_def
    assert s == w.size_from_log(log, length_m=2.5, system_v=24.0, insulation="nmb", ambient_c=50.0, n_bundled=9,
                                max_drop_pct=1.0)
    # each option alone moves the answer
    for k, v in (("insulation", "nmb"), ("ambient_c", 50.0), ("n_bundled", 9), ("max_drop_pct", 0.5)):
        assert w.size_from_log(log, length_m=2.5, system_v=24.0, **{k: v}) != s_def, k
    for k, v in (("insulation", "nmb"), ("ambient_c", 50.0), ("n_bundled", 9), ("max_drop_pct", 0.5),
                 ("termination_c", 60.0)):
        assert w.recommend_gauge(current_a=60.0, length_m=3.0, system_v=48.0, **{k: v}) != \
            w.recommend_gauge(current_a=60.0, length_m=3.0, system_v=48.0), k
    # the defaults themselves
    assert w.check_run("10", current_a=20.0, length_m=2.0, system_v=12.0) == w.check_run(
        "10", current_a=20.0, length_m=2.0, system_v=12.0, insulation="thhn", ambient_c=30.0, n_bundled=1,
        max_drop_pct=3.0)
    assert w.check_run("10", current_a=20.0, length_m=2.0, system_v=12.0, ambient_c=31.0) != w.check_run(
        "10", current_a=20.0, length_m=2.0, system_v=12.0)
    per_m = w.check_run("10", current_a=20.0, length_m=1.0, system_v=12.0).drop_pct
    L35 = 3.5 / per_m
    assert w.check_run("10", current_a=20.0, length_m=L35, system_v=12.0, max_drop_pct=4.0).ok != w.check_run(
        "10", current_a=20.0, length_m=L35, system_v=12.0).ok
    assert w.check_run("10", current_a=20.0, length_m=2.0, system_v=12.0, n_bundled=7) != w.check_run(
        "10", current_a=20.0, length_m=2.0, system_v=12.0)
    assert w.derate("10") == w.derate("10", insulation="thhn", ambient_c=30.0, n_bundled=1)
    assert w.derate("10", n_bundled=7) != w.derate("10") and w.derate("10", insulation="nmb") != w.derate("10")
    assert w.parallel_needed(300.0) == w.parallel_needed(300.0, insulation="thhn", ambient_c=30.0, n_bundled=1)
    assert w.parallel_needed(300.0, n_bundled=7) != w.parallel_needed(300.0)
    assert w.fuse_coordination("10", 30.0) == w.fuse_coordination("10", 30.0, insulation="thhn", ambient_c=30.0,
                                                                   n_bundled=1)
    assert w.ampacity("10") == w.ampacity("10", "thhn")
    # small currents, small drops, a cool day
    cool = w.check_run("10", current_a=0.5, length_m=0.5, system_v=400.0, ambient_c=10.0)
    assert cool.thermal_margin == pytest.approx(w.derate("10", ambient_c=10.0).allowed_a / 0.5)
    assert cool.resistance_ohm == w.check_run("10", current_a=0.5, length_m=0.5, system_v=400.0,
                                              ambient_c=15.0).resistance_ohm
    assert cool.resistance_ohm < w.check_run("10", current_a=0.5, length_m=0.5, system_v=400.0,
                                             ambient_c=20.5).resistance_ohm
    tiny = w.size_from_log([0.2, -0.4, 0.3], length_m=1.0, system_v=400.0)
    assert tiny.crest == pytest.approx(0.4 / math.sqrt((0.04 + 0.16 + 0.09) / 3))


def test_size_from_log_sizes_both_ends_with_every_option():
    log = [10.0, 40.0, 25.0, 60.0, 5.0, 30.0]
    rms = math.sqrt(sum(x * x for x in log) / len(log))
    opts = [dict(insulation="nmb"), dict(ambient_c=60.0), dict(n_bundled=9), dict(max_drop_pct=0.5),
            dict(insulation="nmb", ambient_c=45.0, n_bundled=7, max_drop_pct=1.2)]
    for o in opts:
        s = w.size_from_log(log, length_m=2.5, system_v=24.0, **o)
        assert s.recommended_awg == w.recommend_gauge(current_a=rms, length_m=2.5, system_v=24.0, **o)[0], o
        assert s.peak_awg == w.recommend_gauge(current_a=60.0, length_m=2.5, system_v=24.0, **o)[0], o
    moved = {k: set() for k in ("recommended_awg", "peak_awg")}
    for o in opts[:4]:
        s = w.size_from_log(log, length_m=2.5, system_v=24.0, **o)
        for k in moved:
            moved[k].add(getattr(s, k))
    # every option shifts each end somewhere in the set, so a dropped option is seen at both
    for o in opts[:4]:
        base = w.size_from_log(log, length_m=2.5, system_v=24.0)
        s = w.size_from_log(log, length_m=2.5, system_v=24.0, **o)
        assert s.recommended_awg != base.recommended_awg and s.peak_awg != base.peak_awg, o
