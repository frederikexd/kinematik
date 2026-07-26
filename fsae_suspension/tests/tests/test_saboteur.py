# ============================================================================
#  KinematiK — tests for suspension/saboteur.py
#
#  The Saboteur sells four promises, and each has tests pinning it:
#    1. determinism   — same deck in, same kill board / sheet / suspects out;
#    2. detection     — every catalogued corruption class is caught somewhere
#                       on a fully-declared deck, and the classic killers
#                       (lb-kg, z-flip, kilo slips, dropped terms) are caught
#                       by the specific wire built to see them;
#    3. honesty       — blind spots are reported, never hidden; coverage
#                       counts them; a wire with missing inputs is never
#                       offered; incomplete readings never judge CLEAN;
#    4. tamper-evidence — an edited sheet refuses to judge, same as a
#                       tampered validation contract.
# ============================================================================

import datetime as dt

import pytest

from suspension.interfaces import SubsystemInterface, blank_ledger
from suspension.proof_engine import (
    DEFAULT_OBJECTIVES, EvidenceGrade, build_uncertainty_ledger,
)
from suspension.saboteur import (
    DEFAULT_MUTATIONS, DEFAULT_TRIPWIRES, PreflightSheet,
    available_tripwires, build_sheet, judge_readings, render_preflight_md,
    run_sweep, select_tripwires,
)

TODAY = dt.date(2026, 7, 17)


def _full_ledger():
    """A deck with every channel family declared, so every wire is live."""
    led = blank_ledger()
    led.set(SubsystemInterface(name="chassis", mass_kg=45.0, cg_z_mm=300.0,
                               mount_load_n=4200.0, is_estimate=True))
    led.set(SubsystemInterface(name="powertrain", mass_kg=60.0, cg_z_mm=320.0,
                               peak_power_kw=68.0, peak_torque_nm=180.0,
                               heat_reject_w=3200.0, is_estimate=True))
    led.set(SubsystemInterface(name="accumulator", mass_kg=55.0,
                               peak_current_a=180.0, power_draw_w=350.0,
                               is_estimate=True))
    led.set(SubsystemInterface(name="brakes", mass_kg=9.0,
                               brake_torque_nm=650.0, is_estimate=True))
    led.set(SubsystemInterface(name="cooling", mass_kg=6.0,
                               cooling_airflow_cms=0.14, is_estimate=True))
    return led


def _quantities():
    return build_uncertainty_ledger(_full_ledger())


def _laptime():
    return next(o for o in DEFAULT_OBJECTIVES if o.key == "laptime_s")


def _sweep():
    return run_sweep(_laptime(), _quantities(), today=TODAY)


def _finding(report, mutation_key, target_key):
    return next(f for f in report.findings
                if f.mutation_key == mutation_key
                and f.target_key == target_key)


# --------------------------------------------------------------------------- #
#  Determinism
# --------------------------------------------------------------------------- #
def test_sweep_is_deterministic():
    """Same deck in, same kill board out — byte for byte."""
    a = [f.as_dict() for f in _sweep().findings]
    b = [f.as_dict() for f in _sweep().findings]
    assert a == b


def test_sheet_and_seal_are_deterministic():
    s1 = build_sheet(_sweep(), today=TODAY)
    s2 = build_sheet(_sweep(), today=TODAY)
    assert s1.seal == s2.seal
    assert [w["key"] for w in s1.wires] == [w["key"] for w in s2.wires]


# --------------------------------------------------------------------------- #
#  The catalog strikes where it claims to, and nowhere else
# --------------------------------------------------------------------------- #
def test_every_mutation_finds_a_target_on_a_full_deck():
    rep = _sweep()
    struck = {f.mutation_key for f in rep.findings}
    assert struck == {m.key for m in DEFAULT_MUTATIONS}


def test_mass_mutation_never_strikes_airflow():
    rep = _sweep()
    for f in rep.findings:
        if f.mutation_key == "mass_lb":
            assert f.target_key.endswith("mass_kg")


def test_mutations_never_modify_the_original_deck():
    qs = _quantities()
    before = [(q.key, q.value) for q in qs]
    run_sweep(_laptime(), qs, today=TODAY)
    assert [(q.key, q.value) for q in qs] == before


# --------------------------------------------------------------------------- #
#  Detection — the classic killers are caught by the wire built for them
# --------------------------------------------------------------------------- #
def test_z_flip_is_caught_by_the_cg_wire():
    """The Frames & Datums war story: a Z-down sheet in a Z-up deck."""
    f = _finding(_sweep(), "z_flip", "chassis.cg_z_mm")
    assert "cg_z_mm" in f.caught_by
    assert f.wire_sigmas["cg_z_mm"] < 0        # CG moved DOWN — signed


def test_pounds_into_kg_on_a_small_subsystem_is_caught():
    """6 kg of cooling becoming 13 kg is invisible to lap time (silent) but
    a 2% mass checksum sees the 4% roll-up shift immediately."""
    f = _finding(_sweep(), "mass_lb", "cooling.mass_kg")
    assert f.silent
    assert "total_mass_kg" in f.caught_by


def test_lbft_slip_is_caught_by_torque_per_power():
    """Lap time never reads brake torque, so the slip is perfectly silent —
    and torque-per-power is exactly the wire that unmasks the motor's twin."""
    f = _finding(_sweep(), "torque_lbft", "powertrain.peak_torque_nm")
    assert "torque_per_power" in f.caught_by


def test_dropped_rollup_term_is_caught():
    f = _finding(_sweep(), "drop_term", "brakes.mass_kg")
    assert f.silent and "total_mass_kg" in f.caught_by


def test_full_deck_has_no_blind_spots_and_sheet_reaches_full_coverage():
    """On a fully-declared deck the catalogued classes are 100% covered —
    and the envelope alone covers only a small fraction, which is the
    entire reason this module exists."""
    rep = _sweep()
    sheet = build_sheet(rep, today=TODAY)
    assert sheet.coverage_after == pytest.approx(1.0)
    assert sheet.coverage_before < 0.5
    assert sheet.blind_spots == []


# --------------------------------------------------------------------------- #
#  Honesty — blind spots reported, unavailable wires never offered
# --------------------------------------------------------------------------- #
def test_wire_with_missing_inputs_is_not_offered():
    led = blank_ledger()
    led.set(SubsystemInterface(name="chassis", mass_kg=45.0,
                               is_estimate=True))
    qs = build_uncertainty_ledger(led)
    keys = {w.key for w in available_tripwires(qs)}
    assert "total_mass_kg" in keys
    assert "pack_voltage_v" not in keys        # no current declared
    assert "torque_per_power" not in keys      # no torque declared


def test_blind_spot_is_reported_not_hidden():
    """Torque declared but power absent: the lb-ft slip is silent to lap
    time AND no live wire can see it. The sheet must say so out loud and
    the coverage number must charge for it."""
    led = blank_ledger()
    led.set(SubsystemInterface(name="chassis", mass_kg=45.0, cg_z_mm=300.0,
                               is_estimate=True))
    led.set(SubsystemInterface(name="powertrain", mass_kg=60.0,
                               peak_torque_nm=180.0, is_estimate=True))
    qs = build_uncertainty_ledger(led)
    rep = run_sweep(_laptime(), qs, today=TODAY)
    sheet = build_sheet(rep, today=TODAY)
    blind_labels = {b["target_label"] for b in sheet.blind_spots}
    assert any("torque" in lab for lab in blind_labels)
    assert sheet.coverage_after < 1.0
    assert "Honest blind spots" in render_preflight_md(sheet, rep)


def test_verified_deck_still_gets_swept():
    """Good evidence grades shrink the Proof Engine's bands but must never
    shrink tripwire tolerances — a verified deck can still be corrupted in
    transcription, which is the whole deck-vs-run point."""
    qs = _quantities()
    for q in qs:
        q.grade = EvidenceGrade.VERIFIED
        q.measured_on = TODAY.isoformat()
    rep = run_sweep(_laptime(), qs, today=TODAY)
    f = _finding(rep, "mass_lb", "cooling.mass_kg")
    assert "total_mass_kg" in f.caught_by


# --------------------------------------------------------------------------- #
#  Greedy cover
# --------------------------------------------------------------------------- #
def test_selected_wires_are_sufficient_and_every_wire_earns_its_place():
    rep = _sweep()
    chosen = select_tripwires(rep)
    # sufficiency: every CATCHABLE silent killer is caught by the sheet
    for f in rep.silent_killers:
        if f.caught_by:
            assert any(k in chosen for k in f.caught_by)
    # no freeloaders: removing any wire must lose at least one catch
    for k in chosen:
        others = [w for w in chosen if w != k]
        lost = [f for f in rep.silent_killers
                if k in f.caught_by
                and not any(o in f.caught_by for o in others)]
        assert lost, f"wire {k} catches nothing the rest miss"


def test_hard_cap_charges_uncovered_killers_to_blind_spots():
    """A caller-imposed cap may shorten the sheet, but anything catchable it
    leaves uncovered must be charged as a blind spot — truncation is
    visible, never silent."""
    rep = _sweep()
    full = build_sheet(rep, today=TODAY)
    capped = build_sheet(rep, max_wires=2, today=TODAY)
    assert len(capped.wires) == 2
    assert capped.coverage_after <= full.coverage_after
    n = len(rep.findings)
    charged = round(capped.coverage_after * n) + len(capped.blind_spots)
    assert charged == n                        # every finding is accounted for


def test_fakes_pass_flags_inside_band_corruptions():
    """With an acceptance band on the objective, the sweep answers the
    scariest question: could garbage hand the team a PASS?"""
    rep = run_sweep(_laptime(), _quantities(),
                    pass_band=(70.0, 82.0), today=TODAY)
    f = _finding(rep, "mass_lb", "cooling.mass_kg")
    assert f.fakes_pass is True                # +0.2 s: contract still smiles
    g = _finding(rep, "len_x1000", "chassis.cg_z_mm")
    assert g.fakes_pass is False               # CG at 300 m does not


# --------------------------------------------------------------------------- #
#  Sealed sheet & judging
# --------------------------------------------------------------------------- #
def _sheet_and_report():
    rep = _sweep()
    return build_sheet(rep, today=TODAY), rep


def test_clean_readings_judge_clean():
    sheet, rep = _sheet_and_report()
    readings = {w["key"]: w["clean"] for w in sheet.wires}
    v = judge_readings(sheet, readings)
    assert v.status == "clean" and v.tripped == []
    assert "outside the catalog remain possible" in v.note


def test_missing_reading_is_incomplete_not_clean():
    sheet, _ = _sheet_and_report()
    readings = {w["key"]: w["clean"] for w in sheet.wires[1:]}
    assert judge_readings(sheet, readings).status == "incomplete"


def test_tampered_sheet_refuses_to_judge():
    sheet, _ = _sheet_and_report()
    d = sheet.as_dict()
    d["wires"][0]["band"] *= 10.0              # widen a band after sealing
    with pytest.raises(ValueError, match="seal"):
        judge_readings(PreflightSheet.from_dict(d),
                       {w["key"]: w["clean"] for w in sheet.wires})


def test_fingerprint_names_the_injected_corruption():
    """Corrupt the deck with a known mutation, feed the sim-side readings
    back in: the top suspect must be that exact (mutation, target)."""
    from suspension.saboteur import DEFAULT_MUTATIONS as MUTS
    from suspension.proof_engine import aggregate
    sheet, rep = _sheet_and_report()
    qs = _quantities()
    m = next(x for x in MUTS if x.key == "mass_lb")
    i = next(i for i, q in enumerate(qs) if q.key == "accumulator.mass_kg")
    bad_car = aggregate(m.apply(qs, i))
    readings = {}
    for w in sheet.wires:
        wire = next(x for x in DEFAULT_TRIPWIRES if x.key == w["key"])
        readings[w["key"]] = wire.fn(bad_car)
    v = judge_readings(sheet, readings)
    assert v.status == "tripped"
    assert v.suspects[0]["finding"] == "mass_lb::accumulator.mass_kg"
    assert v.suspects[0]["cosine"] > 0.99
    assert v.suspects[0]["magnitude_ratio"] == pytest.approx(1.0, abs=0.01)


def test_uncatalogued_error_is_admitted_not_misattributed():
    """A corruption outside the catalog trips wires but matches no
    signature: the verdict must say so instead of naming a false suspect."""
    sheet, _ = _sheet_and_report()
    readings = {w["key"]: w["clean"] for w in sheet.wires}
    # invert two wires in a direction no catalogued mutation produces
    readings["total_mass_kg"] *= 0.5
    readings["cg_z_mm"] *= 3.0
    v = judge_readings(sheet, readings)
    assert v.status == "tripped"
    if v.suspects:
        assert v.suspects[0]["cosine"] < 0.95
    else:
        assert "outside the catalog" in v.note


# --------------------------------------------------------------------------- #
#  Export
# --------------------------------------------------------------------------- #
def test_markdown_carries_seal_coverage_and_frame():
    sheet, rep = _sheet_and_report()
    md = render_preflight_md(sheet, rep, frame_note="ISO 8855 (team charter)")
    assert sheet.seal[:16] in md
    assert "ISO 8855" in md
    assert "Detection coverage" in md
    assert "not a certificate" in md
