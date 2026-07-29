# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  tests/test_express.py — the Express Lane, pinned.
# ============================================================================
"""What these tests guard.

The express lane makes three promises that are easy to break by accident and
expensive to break in front of a judge:

* DETERMINISM — the same sentence and the same bytes produce a byte-identical
  ZIP. The classic way to lose this is somebody adding a timestamp to the
  manifest "for debugging". A test is the only thing that stops it.
* HONESTY — every number the grammar binds, every default it falls back to,
  every column the sniffer rescaled and every job it skipped is printed. A
  silent skip is worse than a failure, because nobody goes looking for it.
* NO SILENT EMPTINESS — an empty request still ships a real bundle, and a job
  that raises fails INTO the bundle rather than taking the run down.

All headless: no Streamlit, no network. `ui/express_lane.py` is import-checked
only for the package rule (streamlit imported inside render(), never at module
top level), which is what makes these tests possible at all.
"""

import csv
import io
import json
import zipfile

import numpy as np
import pytest

from suspension import express as ex


# --------------------------------------------------------------------------- #
#  fixtures
# --------------------------------------------------------------------------- #
def _log(n=400, ay_in_ms2=True, speed_kph=True):
    """A log with awkward headers, a units row and non-SI scales — the shape
    real loggers actually emit, not a convenient one."""
    t = np.linspace(0.0, 20.0, n)
    ay = 1.3 * np.sin(2 * np.pi * t / 6.0) * (9.80665 if ay_in_ms2 else 1.0)
    ax = -1.1 * np.clip(np.sin(2 * np.pi * t / 4.0), -1, 0) * 9.80665
    spd = (55.0 + 20.0 * np.cos(2 * np.pi * t / 6.0)) * (1.0 if speed_kph
                                                         else 1 / 3.6)
    out = io.StringIO()
    w = csv.writer(out, lineterminator="\n")
    w.writerow(["Time", "Lat Accel [m/s2]", "Long Accel", "GPS Speed",
                "Steering Angle", "Widget Voltage Thing"])
    w.writerow(["s", "m/s2", "m/s2", "kph", "deg", "-"])
    for i in range(n):
        w.writerow([f"{t[i]:.4f}", f"{ay[i]:.4f}", f"{ax[i]:.4f}",
                    f"{spd[i]:.3f}", f"{40*np.sin(t[i]):.3f}", "3"])
    return out.getvalue().encode("utf-8")


SENTENCE = ("Our 245 kg car with a cg height of 290 mm understeers on the "
            "skidpad at 1.4 lateral g and I think bump steer is the cause. "
            "Here's the run log, I need the roll numbers before the review.")


# --------------------------------------------------------------------------- #
#  1 · the grammar
# --------------------------------------------------------------------------- #
def test_parse_is_deterministic():
    a, b = ex.parse_request(SENTENCE), ex.parse_request(SENTENCE)
    assert a.params == b.params
    assert a.tools == b.tools
    assert a.consumed == b.consumed
    assert a.ignored == b.ignored


def test_numbers_bind_to_the_parameter_beside_them():
    a = ex.parse_request(SENTENCE)
    assert a.params["mass_kg"] == 245.0
    assert a.params["cg_height_mm"] == 290.0
    assert a.params["lateral_g"] == pytest.approx(1.4)


def test_specific_phrase_beats_the_general_one():
    a = ex.parse_request("front track 1210 mm, rear track 1185 mm")
    assert a.params["track_front_mm"] == 1210.0
    assert a.params["track_rear_mm"] == 1185.0


def test_units_are_converted_and_the_conversion_is_printed():
    a = ex.parse_request("wheelbase 1.55 m and a 300 lb driver")
    assert a.params["wheelbase_mm"] == pytest.approx(1550.0)
    assert any("1.55 m" in c for c in a.consumed)


def test_percent_becomes_a_fraction():
    a = ex.parse_request("run it with 62% front brake bias")
    assert a.params["brake_bias_front"] == pytest.approx(0.62)


def test_ambiguous_g_is_resolved_by_dimension_not_by_prior():
    mass = ex.parse_request("the car is 320 g heavier than target")
    accel = ex.parse_request("we only pull 1.2 g in the corners")
    assert mass.params["mass_kg"] == pytest.approx(0.320)
    assert accel.params["lateral_g"] == pytest.approx(1.2)


def test_unit_alone_can_name_an_unambiguous_parameter():
    a = ex.parse_request("245 kg car, nothing else to say")
    assert a.params["mass_kg"] == 245.0
    assert any("inferred from the unit alone" in c for c in a.consumed)


def test_ambiguous_dimensions_are_not_guessed():
    """A bare length could be cg, wheelbase or track. The grammar must not
    pick one — inventing an interpretation is the failure mode this whole
    module is built to avoid."""
    a = ex.parse_request("something is 300 mm")
    assert "cg_height_mm" not in a.params
    assert "wheelbase_mm" not in a.params


def test_tool_words_match_on_word_boundaries():
    """'review' must not summon the EV powertrain engine."""
    a = ex.parse_request("I need this before the design review")
    assert "ev" not in a.tools


def test_what_it_did_not_understand_is_printed():
    a = ex.parse_request("the flumbulator is making a weird noise")
    assert "flumbulator" in a.ignored


def test_defaults_are_declared_not_hidden():
    a = ex.parse_request("just the roll numbers")
    assert a.param("mass_kg") == 280.0
    assert a.param("cg_height_mm") == 300.0


# --------------------------------------------------------------------------- #
#  2 · the sniffer
# --------------------------------------------------------------------------- #
def test_channels_are_recognised_from_awkward_headers():
    db = ex.sniff_files([("run.csv", _log())])
    for canon in ("time", "ay", "ax", "speed", "steer"):
        assert canon in db.series, f"{canon} not recognised"


def test_units_row_is_detected_and_dropped():
    db = ex.sniff_files([("run.csv", _log(n=400))])
    assert db.channels["ay"].n_finite >= 395
    assert any("units" in r for r in db.receipts)


def test_scale_is_inferred_from_the_data_and_disclosed():
    db = ex.sniff_files([("run.csv", _log(ay_in_ms2=True, speed_kph=True))])
    assert abs(db.channels["ay"].vmax) < 3.0          # gravities, not m/s²
    assert db.channels["speed"].mean < 30.0           # m/s, not km/h
    assert "9.80665" in db.channels["ay"].scale_note
    assert "3.6" in db.channels["speed"].scale_note


def test_data_already_in_si_is_left_alone():
    db = ex.sniff_files([("run.csv", _log(ay_in_ms2=False, speed_kph=False))])
    assert abs(db.channels["ay"].vmax) < 3.0
    assert "already" in db.channels["ay"].scale_note


def test_unmatched_columns_are_reported_not_swallowed():
    db = ex.sniff_files([("run.csv", _log())])
    assert any("widget" in c.lower() for c in db.unmatched)


def test_sample_rate_and_duration():
    db = ex.sniff_files([("run.csv", _log(n=400))])
    assert db.sample_rate_hz == pytest.approx(400 / 20.0, rel=0.1)
    assert db.duration_s == pytest.approx(20.0, rel=0.01)


def test_flatlined_channel_is_flagged():
    rows = ["Time,Lat Accel", *[f"{i*0.05:.3f},0.0" for i in range(200)]]
    db = ex.sniff_files([("flat.csv", "\n".join(rows).encode())])
    assert any("FLATLINED" in f for f in db.channels["ay"].flags)
    assert any("FLATLINED" in w for w in db.warnings)


def test_a_broken_file_does_not_take_the_run_down():
    db = ex.sniff_files([("junk.csv", b"\x00\x01\x02"),
                         ("run.csv", _log())])
    assert "ay" in db.series          # the good file still landed


def test_hardpoint_json_is_recognised():
    from suspension.kinematics import Hardpoints
    hp = Hardpoints.default()
    blob = json.dumps({k: (v.tolist() if hasattr(v, "tolist") else v)
                       for k, v in hp.as_dict().items()}).encode()
    db = ex.sniff_files([("hp.json", blob)])
    assert db.hardpoints is not None


# --------------------------------------------------------------------------- #
#  3 · the plan
# --------------------------------------------------------------------------- #
def test_data_activates_jobs_nobody_asked_for():
    """The upload is a request too: a log with lateral-g gets the event
    finder whether or not the sentence mentions telemetry."""
    run = ex.run_express("just the roll numbers please",
                         [("run.csv", _log())])
    assert "events" in run.ran
    assert "telemetry" in run.ran


def test_skips_name_the_missing_channel():
    ask = ex.parse_request("give me the telemetry summary")
    db = ex.sniff_files(None)
    planned = ex.plan(ask, db)
    skips = [p for p in planned if p.skipped]
    assert skips, "a job needing channels should skip with no data"
    assert any("time" in p.skipped for p in skips)


def test_an_empty_request_still_ships_a_bundle():
    run = ex.run_express("", None)
    assert run.artifacts
    assert any(a.path.endswith(".md") for a in run.artifacts)


# --------------------------------------------------------------------------- #
#  4 · the run and the bundle
# --------------------------------------------------------------------------- #
def test_full_run_produces_real_files():
    run = ex.run_express(SENTENCE, [("run.csv", _log())])
    assert not run.failed, run.failed
    paths = {a.path for a in run.artifacts}
    assert "kinematics/geometry_baseline.md" in paths
    assert "roll/load_transfer.md" in paths
    for a in run.artifacts:
        assert a.data, f"{a.path} is empty"


def test_a_failing_job_fails_into_the_bundle():
    def _boom(_ctx):
        raise RuntimeError("deliberate")
    ex.register_job(ex.Job("_test_boom", "Deliberate failure", "roll", _boom))
    try:
        run = ex.run_express("roll numbers", None)
        assert ("_test_boom", "RuntimeError: deliberate") in run.failed
        assert any(a.path == "_failed/_test_boom.md" for a in run.artifacts)
        assert "geometry_baseline" in run.ran or run.ran   # others survived
    finally:
        ex.JOBS.pop("_test_boom", None)


def test_zip_is_byte_deterministic():
    z1 = ex.bundle_zip(ex.run_express(SENTENCE, [("run.csv", _log())]))
    z2 = ex.bundle_zip(ex.run_express(SENTENCE, [("run.csv", _log())]))
    assert z1 == z2


def test_manifest_carries_no_wall_clock():
    """A timestamp is the classic, well-meant way to destroy determinism."""
    run = ex.run_express(SENTENCE, [("run.csv", _log())])
    blob = json.dumps(run.manifest())
    assert "elapsed" not in blob
    assert run.elapsed_s >= 0.0      # still available to the UI


def test_readme_lists_every_file_actually_in_the_zip():
    run = ex.run_express(SENTENCE, [("run.csv", _log())])
    blob = ex.bundle_zip(run)
    with zipfile.ZipFile(io.BytesIO(blob)) as z:
        names = set(z.namelist())
        readme = z.read("README.md").decode()
    for n in names - {"README.md"}:
        assert f"`{n}`" in readme, f"{n} is in the ZIP but not the README"


def test_deliverable_words_never_delete_data():
    """'Give me a report' is emphasis, not an instruction to bin the CSVs."""
    run = ex.run_express("give me a report on the roll numbers", None)
    blob = ex.bundle_zip(run)
    with zipfile.ZipFile(io.BytesIO(blob)) as z:
        assert any(n.endswith(".csv") for n in z.namelist())


def test_readme_prints_the_source_of_every_parameter():
    run = ex.run_express(SENTENCE, [("run.csv", _log())])
    readme = ex.render_readme(run).decode()
    assert "from your sentence" in readme
    assert "default" in readme
    assert "not a language model" in readme.lower()


# --------------------------------------------------------------------------- #
#  5 · the package rule
# --------------------------------------------------------------------------- #
def test_ui_module_is_importable_headless():
    """ui/ modules must import Streamlit inside render(), never at module
    level — that rule is what lets everything above run in CI."""
    import ui.express_lane as mod
    assert hasattr(mod, "render")
    src = open(mod.__file__, encoding="utf-8").read()
    head = src.split("def render", 1)[0]
    assert "import streamlit" not in head


# =========================================================================== #
#  6 · The second tranche — dimension-aware binding, needs_any, the new jobs
# =========================================================================== #
def _rich_log(n=600):
    """A log with the channels the data jobs want, and headers in the shape
    real loggers emit them ('Damper Pos FL', not 'damper_fl')."""
    t = np.linspace(0.0, 30.0, n)
    rng = np.random.default_rng(3)
    ay = 1.3 * np.sin(2 * np.pi * t / 7.0) * 9.80665
    ax = 1.4 * np.sin(2 * np.pi * t / 5.0) * 9.80665
    roll = 0.9 * np.sin(2 * np.pi * t / 7.0)
    out = io.StringIO()
    w = csv.writer(out, lineterminator="\n")
    w.writerow(["Time", "Lat Accel", "Long Accel", "GPS Speed",
                "Damper Pos FL", "Damper Pos FR",
                "Brake Press Front", "Brake Press Rear",
                "TS Voltage", "TS Current"])
    w.writerow(["s", "m/s2", "m/s2", "kph", "mm", "mm", "bar", "bar",
                "V", "A"])
    for i in range(n):
        fl = 12 * roll[i] + rng.normal(0, 0.15)
        bp = max(0.0, -ax[i] / 9.80665) * 55
        cur = max(0.0, ax[i] / 9.80665) * 160
        w.writerow([f"{t[i]:.4f}", f"{ay[i]:.4f}", f"{ax[i]:.4f}",
                    f"{60 + 20*np.cos(t[i]):.3f}", f"{fl:.3f}", f"{-fl:.3f}",
                    f"{bp:.2f}", f"{bp*0.6:.2f}",
                    f"{400 - 0.3*cur:.2f}", f"{cur:.2f}"])
    return out.getvalue().encode("utf-8")


def test_binder_prefers_the_dimension_it_wants_over_mere_proximity():
    """In '6.5 kWh pack, 22 endurance laps' the bare 22 is CLOSER to 'pack'
    than the kWh figure. A distance-only binder gives the pack a lap count."""
    a = ex.parse_request("6.5 kWh pack, 22 endurance laps")
    assert a.params["pack_kwh"] == pytest.approx(6.5)
    assert a.params["endurance_laps"] == pytest.approx(22.0)


def test_a_wrong_dimension_quantity_is_never_claimed():
    a = ex.parse_request("cg is over there, 62% front bias")
    assert "cg_height_mm" not in a.params      # a fraction is not a length


def test_energy_and_power_units():
    a = ex.parse_request("80 kW motor on a 7.2 kWh pack")
    assert a.params["power_kw"] == pytest.approx(80.0)
    assert a.params["pack_kwh"] == pytest.approx(7.2)


def test_headers_with_filler_words_still_match():
    db = ex.sniff_files([("run.csv", _rich_log())])
    for canon in ("damper_fl", "damper_fr", "brake_front", "brake_rear",
                  "pack_v", "pack_i"):
        assert canon in db.series, f"{canon} lost to a filler word"


def test_units_are_not_reported_as_unknown_words():
    a = ex.parse_request("6.5 kWh pack and 22 laps")
    assert "kwh" not in a.ignored


def test_needs_any_runs_on_one_of_a_family():
    rows = ["Time,Damper Pos FL",
            *[f"{i*0.05:.3f},{np.sin(i*0.1)*10:.3f}" for i in range(300)]]
    run = ex.run_express("", [("one_pot.csv", "\n".join(rows).encode())])
    assert "damper_histogram" in run.ran


def test_needs_any_skips_with_a_named_reason_when_the_family_is_absent():
    ask = ex.parse_request("give me the damper histogram")
    planned = ex.plan(ask, ex.sniff_files(None))
    hit = [p for p in planned if p.job.jid == "damper_histogram"]
    assert hit and hit[0].skipped
    assert "at least one of" in hit[0].skipped


def test_the_new_model_jobs_run_clean():
    run = ex.run_express(
        "245 kg car, cg 290 mm, 6.5 kWh pack, 22 endurance laps — I need lap "
        "times, compliance, EV energy, the g-g-v envelope and harness loads",
        None)
    assert not run.failed, run.failed
    for jid in ("lap_events", "compliance_corner", "ev_energy",
                "ggv_envelope", "chassis_rules"):
        assert jid in run.ran, f"{jid} did not run"


def test_the_new_data_jobs_run_clean():
    run = ex.run_express("", [("run.csv", _rich_log())])
    assert not run.failed, run.failed
    for jid in ("damper_histogram", "brake_bias_measured",
                "energy_from_log", "roll_correlation"):
        assert jid in run.ran, f"{jid} did not run"


def test_no_green_verdict_on_dead_channels():
    """The first version of the bias job printed 'measured and declared
    agree' when both pressure channels were flat and the fit was NaN. A
    green light on no evidence is worse than a red one."""
    rows = ["Time,Brake Press Front,Brake Press Rear",
            *[f"{i*0.05:.3f},0.0,0.0" for i in range(300)]]
    run = ex.run_express("", [("dead.csv", "\n".join(rows).encode())])
    art = next(a for a in run.artifacts if a.path.endswith("measured_bias.md"))
    body = art.data.decode()
    assert "No verdict" in body
    assert "agree within" not in body


def test_negative_net_energy_is_called_out_not_averaged_away():
    n = 300
    rows = ["Time,TS Voltage,TS Current",
            *[f"{i*0.05:.3f},400.0,-50.0" for i in range(n)]]
    run = ex.run_express("this log covers 2 laps",
                         [("regen.csv", "\n".join(rows).encode())])
    art = next(a for a in run.artifacts
               if a.path.endswith("measured_energy.md"))
    assert "negative" in art.data.decode().lower()


def test_no_two_jobs_write_the_same_path():
    """Two jobs writing one path means the later silently overwrites the
    earlier inside the ZIP."""
    run = ex.run_express(
        "kinematics roll compliance brakes tire setup laptime ev frames "
        "telemetry", [("run.csv", _rich_log())])
    paths = [a.path for a in run.artifacts]
    assert len(paths) == len(set(paths)), \
        [p for p in paths if paths.count(p) > 1]


def test_every_job_tool_has_a_tab_name():
    """A job whose tool id has no tab name produces a README telling the
    member to open a tab that does not exist."""
    from suspension.express import _TAB_NAMES
    for jid, job in ex.JOBS.items():
        assert job.tool in _TAB_NAMES, f"{jid} → unknown tool '{job.tool}'"


def test_cli_dry_run(tmp_path, capsys):
    log = tmp_path / "run.csv"
    log.write_bytes(_rich_log())
    rc = ex.main(["245 kg car, roll numbers", str(log), "--dry-run"])
    assert rc == 0
    assert "nothing written" in capsys.readouterr().out


def test_cli_writes_a_bundle(tmp_path):
    log = tmp_path / "run.csv"
    log.write_bytes(_rich_log())
    out = tmp_path / "b.zip"
    rc = ex.main(["245 kg car, roll and telemetry", str(log),
                  "-o", str(out)])
    assert rc == 0
    assert out.exists() and zipfile.is_zipfile(out)


def test_log_laps_and_endurance_laps_do_not_collide():
    """The parameter table's one ordering contract: specific phrases before
    generic ones. 'laps' (endurance) must not steal the quantity that
    'log covers' (this log) is reaching for."""
    a = ex.parse_request("we need 22 endurance laps, this log covers 4 laps")
    assert a.params["endurance_laps"] == pytest.approx(22.0)
    assert a.params["logged_laps"] == pytest.approx(4.0)


# =========================================================================== #
#  7 · The time budget — the lane's promise, made testable
# =========================================================================== #
def test_budget_defaults_to_the_lane_promise():
    a = ex.parse_request("give me the roll numbers")
    assert a.budget_s == pytest.approx(90.0)
    assert a.budget_source is None


def test_an_explicit_duration_beats_a_mood_word():
    """'In a hurry but I can give it five minutes' is five minutes."""
    a = ex.parse_request("I'm in a hurry but I can give it 5 minutes")
    assert a.budget_s == pytest.approx(300.0)
    assert "5 minutes" in (a.budget_source or "")


def test_mood_words_set_a_budget_when_no_number_is_given():
    assert ex.parse_request("quick roll numbers").budget_s < 90.0
    assert ex.parse_request("overnight is fine").budget_s > 3600.0


def test_deep_jobs_never_fire_unasked():
    """A thirty-second optimiser must not ambush someone who asked for
    bump steer, however generous their budget."""
    run = ex.run_express("overnight is fine, give me the roll numbers", None)
    assert "omnicore_mission" not in run.ran
    assert "omnicore_mission" not in [j for j, _ in run.deferred]


def test_a_job_over_budget_is_deferred_not_skipped():
    run = ex.run_express("I have 10 seconds. Run omnicore and a step steer "
                         "transient.", None)
    deferred = dict(run.deferred)
    assert "omnicore_mission" in deferred
    assert "30" in deferred["omnicore_mission"]      # names its own cost
    assert "10 s budget" in deferred["omnicore_mission"]
    assert "omnicore_mission" not in [j for j, _ in run.skipped]


def test_a_bigger_budget_admits_the_deep_job():
    plan_small = ex.plan(ex.parse_request("run omnicore"),
                         ex.sniff_files(None), budget_s=5.0)
    plan_big = ex.plan(ex.parse_request("run omnicore"),
                       ex.sniff_files(None), budget_s=600.0)
    small = {p.job.jid: p for p in plan_small}
    big = {p.job.jid: p for p in plan_big}
    assert small["omnicore_mission"].deferred
    assert big["omnicore_mission"].deferred is None


def test_fast_jobs_are_never_deferred():
    """The floor of the lane must always run, or an empty budget yields an
    empty bundle and the promise is broken."""
    planned = ex.plan(ex.parse_request("kinematics roll setup"),
                      ex.sniff_files(None), budget_s=0.0)
    for p in planned:
        if p.job.tier == "fast":
            assert p.deferred is None, p.job.jid


def test_admission_is_a_pure_function_of_the_budget_not_the_clock():
    """If the runner dropped jobs when it noticed it was running late, the
    same request would produce different ZIPs on different machines."""
    ask = ex.parse_request("run omnicore and a step steer transient")
    data = ex.sniff_files(None)
    a = [(p.job.jid, p.deferred) for p in ex.plan(ask, data, budget_s=20.0)]
    b = [(p.job.jid, p.deferred) for p in ex.plan(ask, data, budget_s=20.0)]
    assert a == b


def test_every_job_declares_a_positive_cost():
    for jid, job in ex.JOBS.items():
        assert job.cost_s > 0, f"{jid} declares no cost"
        assert job.tier in ("fast", "slow", "deep")


def test_deep_and_slow_jobs_are_a_small_minority():
    """If most of the registry is expensive, the lane is not a lane."""
    slow = [j for j in ex.JOBS.values() if j.tier != "fast"]
    assert len(slow) < len(ex.JOBS) / 3


# =========================================================================== #
#  8 · The deliberate absences and the rest of the toolkit
# =========================================================================== #
def test_engines_not_in_the_lane_are_named_with_a_reason():
    """A missing tool with no explanation reads as an oversight."""
    run = ex.run_express("solve for hardpoints with inverse design, and "
                         "morph the bracket", None)
    readme = ex.render_readme(run).decode()
    assert "deliberately not in this lane" in readme
    assert "legal volume" in readme          # the actual reason, by name
    assert "Inverse Genesis" in readme


def test_the_newly_wired_fast_engines_run_clean():
    run = ex.run_express(
        "give me the aero baseline, the DFMEA register and the fusebox "
        "overload paths", None)
    assert not run.failed, run.failed
    for jid in ("aero_baseline", "dfmea_register", "fusebox_audit"):
        assert jid in run.ran, f"{jid} did not run"


def test_a_kicad_board_activates_the_pcb_job():
    board = ("(kicad_pcb (version 20221018)\n"
             "  (net 0 \"\")\n  (net 1 \"VBAT\")\n"
             "  (segment (start 10 10) (end 40 10) (width 0.25) "
             "(layer \"F.Cu\") (net 1))\n)")
    db = ex.sniff_files([("main.kicad_pcb", board.encode())])
    assert "kicad_pcb" in db.extras
    planned = {p.job.jid: p for p in ex.plan(ex.parse_request(""), db)}
    assert "pcb_check" in planned and planned["pcb_check"].skipped is None


def test_pcb_job_skips_with_a_named_reason_without_a_board():
    planned = {p.job.jid: p
               for p in ex.plan(ex.parse_request("check the pcb traces"),
                                ex.sniff_files(None))}
    assert planned["pcb_check"].skipped
    assert "kicad_pcb" in planned["pcb_check"].skipped


def test_commentary_answers_the_number_not_the_topic():
    """A paragraph about why gradients matter, printed under a gradient of
    zero, teaches the reader that the prose is decoration."""
    from suspension.express_jobs import _spread_note
    assert "no fans are placed" in _spread_note(0.0)
    assert "limits the car" in _spread_note(18.0)


def test_budget_does_not_break_determinism():
    run_a = ex.run_express("run a step steer transient, I have 5 minutes",
                           None)
    run_b = ex.run_express("run a step steer transient, I have 5 minutes",
                           None)
    assert ex.bundle_zip(run_a) == ex.bundle_zip(run_b)


# =========================================================================== #
#  9 · Job dependencies — the ghost → morph chain
# =========================================================================== #
def test_ghost_audit_is_activated_by_a_log():
    run = ex.run_express("", [("run.csv", _rich_log())])
    assert "ghost_topology" in run.ran
    assert not run.failed, run.failed


def test_morph_is_skipped_when_its_dependency_is_not_in_the_plan():
    """And the reason must point AT the dependency — a member who reads
    'could not run' learns nothing; one who reads 'needs the ghost audit'
    knows to drop a log."""
    planned = {p.job.jid: p for p in ex.plan(ex.parse_request("morph the "
                                                              "bracket"),
                                             ex.sniff_files(None))}
    sk = planned["morph_bracket"].skipped
    assert sk and "Ghost topology" in sk
    assert "not found in the upload" not in sk   # the mangled suffix


def test_dependencies_run_before_their_dependents():
    run = ex.run_express("morph the bracket, I have 5 minutes",
                         [("run.csv", _rich_log())])
    assert not run.failed, run.failed
    assert "ghost_topology" in run.ran and "morph_bracket" in run.ran
    assert run.ran.index("ghost_topology") < run.ran.index("morph_bracket")


def test_the_dependency_actually_carries_data():
    """morph must pick the member the ghost audit flagged, not a default."""
    run = ex.run_express("morph the bracket, I have 5 minutes",
                         [("run.csv", _rich_log())])
    art = next(a for a in run.artifacts
               if a.path.endswith("bracket_morph.md"))
    body = art.data.decode()
    assert "because the ghost audit found it" in body
    assert "worst factor of safety" in body


def test_plan_orders_dependencies_even_when_cost_order_disagrees():
    ask = ex.parse_request("morph the bracket, I have 5 minutes")
    db = ex.sniff_files([("run.csv", _rich_log())])
    order = [p.job.jid for p in ex.plan(ask, db)
             if p.skipped is None and p.deferred is None]
    assert order.index("ghost_topology") < order.index("morph_bracket")


def test_a_dependent_is_skipped_when_its_dependency_fails_at_runtime():
    def _boom(_ctx):
        raise RuntimeError("deliberate")
    real = ex.JOBS["ghost_topology"]
    ex.JOBS["ghost_topology"] = ex.Job(
        real.jid, real.title, real.tool, _boom,
        needs_channels=real.needs_channels,
        data_activated=real.data_activated, cost_s=real.cost_s)
    try:
        run = ex.run_express("morph the bracket, I have 5 minutes",
                             [("run.csv", _rich_log())])
        assert "morph_bracket" not in run.ran
        assert any("did not complete" in r for _j, r in run.skipped)
    finally:
        ex.JOBS["ghost_topology"] = real


def test_declared_dependencies_are_all_real_jobs():
    for jid, job in ex.JOBS.items():
        for dep in job.needs_jobs:
            assert dep in ex.JOBS, f"{jid} depends on unknown job '{dep}'"


def test_the_chain_stays_deterministic():
    files = [("run.csv", _rich_log())]
    a = ex.bundle_zip(ex.run_express("morph the bracket, I have 5 minutes",
                                     files))
    b = ex.bundle_zip(ex.run_express("morph the bracket, I have 5 minutes",
                                     files))
    assert a == b


# =========================================================================== #
#  10 · Four corners and the asymmetry a symmetric model cannot see
# =========================================================================== #
def _four_pot_log(n=600, asym_mm=0.0):
    """All four pots. `asym_mm` rides the front-left corner low, which is the
    signature of unequal corner weights or a bent pushrod."""
    t = np.linspace(0.0, 30.0, n)
    rng = np.random.default_rng(11)
    ay = 1.25 * np.sin(2 * np.pi * t / 7.0) * 9.80665
    ax = 1.3 * np.sin(2 * np.pi * t / 5.0) * 9.80665
    roll = 0.9 * np.sin(2 * np.pi * t / 7.0)
    out = io.StringIO()
    w = csv.writer(out, lineterminator="\n")
    w.writerow(["Time", "Lat Accel", "Long Accel", "GPS Speed",
                "Damper Pos FL", "Damper Pos FR", "Damper Pos RL",
                "Damper Pos RR"])
    w.writerow(["s", "m/s2", "m/s2", "kph", "mm", "mm", "mm", "mm"])
    for i in range(n):
        fl = 12 * roll[i] + asym_mm + rng.normal(0, 0.12)
        fr = -12 * roll[i] + rng.normal(0, 0.12)
        w.writerow([f"{t[i]:.4f}", f"{ay[i]:.4f}", f"{ax[i]:.4f}",
                    f"{62 + 20*np.cos(t[i]):.3f}", f"{fl:.3f}", f"{fr:.3f}",
                    f"{fl*0.8:.3f}", f"{fr*0.8:.3f}"])
    return out.getvalue().encode("utf-8")


def _ghost_md(run):
    return next(a for a in run.artifacts
                if a.path.endswith("ghost_audit.md")).data.decode()


def test_all_four_corners_are_audited():
    run = ex.run_express("", [("run.csv", _four_pot_log())])
    body = _ghost_md(run)
    for c in ("FL", "FR", "RL", "RR"):
        assert f"| **{c}** |" in body, f"{c} missing from the audit"


def test_four_pots_take_the_measured_path():
    run = ex.run_express("", [("run.csv", _four_pot_log())])
    assert "measured — all four damper pots" in _ghost_md(run)


def test_without_pots_the_path_is_modelled_and_says_so():
    run = ex.run_express("", [("run.csv", _rich_log())])   # two pots only
    body = _ghost_md(run)
    assert "modelled — symmetric by construction" in body
    assert "Not assessed" in body       # asymmetry section refuses to guess


def test_a_modelled_run_never_claims_to_have_found_asymmetry():
    """Left and right are symmetric BY CONSTRUCTION on the modelled path, so
    any difference is arithmetic. Reporting it as evidence would be the
    single most misleading thing this job could do."""
    body = _ghost_md(ex.run_express("", [("run.csv", _rich_log())]))
    assert "arithmetic, not evidence" in body


def test_a_square_car_is_reported_as_square():
    run = ex.run_express("", [("run.csv", _four_pot_log(asym_mm=0.0))])
    assert "agree to within" in _ghost_md(run)


def test_a_lopsided_car_is_caught():
    run = ex.run_express("", [("run.csv", _four_pot_log(asym_mm=9.0))])
    assert "large side-to-side difference" in _ghost_md(run).lower()


def test_asymmetry_thresholds_are_relative_not_absolute():
    """An absolute FoS gap of 0.3 is noise at FoS 7 and the whole story at
    FoS 1.2, so the verdict must key off the relative gap."""
    from suspension.express_jobs import _asymmetry_note, _ASYM_LARGE
    assert "agree to within" in _asymmetry_note(0.01, 0.01)
    assert "large" in _asymmetry_note(_ASYM_LARGE + 0.1, 0.0).lower()


def test_a_common_reference_preserves_static_corner_offsets():
    """Zeroing each pot on its own median would subtract out exactly the
    signal this feature exists to find."""
    sq = _ghost_md(ex.run_express("", [("r.csv", _four_pot_log(asym_mm=0.0))]))
    lop = _ghost_md(ex.run_express("", [("r.csv", _four_pot_log(asym_mm=9.0))]))
    assert "ONE common reference" in sq
    assert sq != lop, "a 9 mm corner offset changed nothing — reference bug"


def test_morph_targets_the_worst_corner_of_the_four():
    run = ex.run_express("morph the bracket, I have 5 minutes",
                         [("run.csv", _four_pot_log(asym_mm=9.0))])
    assert not run.failed, run.failed
    body = next(a for a in run.artifacts
                if a.path.endswith("bracket_morph.md")).data.decode()
    assert "of all four corners" in body


# =========================================================================== #
#  11 · The ruleset — and its provenance
# =========================================================================== #
def test_the_ruleset_declares_itself_non_binding():
    from suspension import rules_fsae as rf
    assert rf.RULESET.binding is False
    assert "DRAFT" in rf.RULESET.title


def test_every_rules_artifact_carries_the_draft_banner():
    """A rules verdict that can be quoted without its draft status is a
    liability. The banner is not optional and not configurable."""
    run = ex.run_express("are we legal? 80 kW, 400 V system voltage",
                         [("run.csv", _ev_log())])
    arts = [a for a in run.artifacts if a.path.startswith("rules/")
            and a.path.endswith(".md")]
    assert arts
    for a in arts:
        body = a.data.decode()
        assert "DRAFT" in body
        assert "not valid for any competition" in body


def _ev_log(n=3000, spike_s=0.4, spike_kw=86.0, blip_kw=90.0):
    t = np.linspace(0.0, 30.0, n)
    spd = np.clip(11 + 12 * np.sin(2 * np.pi * t / 9.0), 0, None)
    kw = 55 + 18 * np.sin(2 * np.pi * t / 7.0)
    kw[(t > 12.0) & (t < 12.0 + spike_s)] = spike_kw     # a real violation
    kw[(t > 21.0) & (t < 21.04)] = blip_kw               # 40 ms — must not
    kw[(t > 25.0) & (t < 25.9)] = 79.5                   # close but legal
    v = 402 - 0.06 * np.abs(kw) * 1000 / 402
    out = io.StringIO()
    w = csv.writer(out, lineterminator="\n")
    w.writerow(["Time", "GPS Speed", "TS Voltage", "TS Current"])
    w.writerow(["s", "m/s", "V", "A"])
    for i in range(n):
        w.writerow([f"{t[i]:.4f}", f"{spd[i]:.3f}", f"{v[i]:.2f}",
                    f"{kw[i]*1000/v[i]:.2f}"])
    return out.getvalue().encode("utf-8")


def test_a_real_violation_is_caught_and_priced():
    from suspension import rules_fsae as rf
    run = ex.run_express("are we legal?", [("run.csv", _ev_log())])
    body = next(a for a in run.artifacts
                if a.path.endswith("measured_check.md")).data.decode()
    assert "1 violation —" in body            # singular, and exactly one
    assert "60 s" in body                     # EV.3.5.2 priced
    assert "continuous > 100 ms" in body


def test_a_brief_spike_is_not_a_violation():
    """EV.3.4.1 makes dwell part of the definition. A 40 ms blip over the
    limit is not a violation, and reporting it as one would train teams to
    ignore the report."""
    from suspension import rules_fsae as rf
    t = np.arange(0, 20, 0.01)
    v = np.full_like(t, 400.0)
    kw = np.full_like(t, 60.0)
    kw[(t > 5.0) & (t < 5.04)] = 95.0
    res = rf.check_measured(t, v, kw * 1000 / v)
    assert res.n_violations == 0, res.power_events


def test_a_near_miss_is_flagged_as_watch_not_pass():
    from suspension import rules_fsae as rf
    f = {x.rid: x for x in rf.check_declared({"power_kw": 79.0})}
    assert f["EV.3.3.1"].severity == "watch"


def test_an_unstated_value_is_unknown_never_ok():
    from suspension import rules_fsae as rf
    f = {x.rid: x for x in rf.check_declared({})}
    assert all(x.severity == "unknown" for x in f.values())


def test_a_log_too_slow_to_resolve_the_dwell_says_so():
    from suspension import rules_fsae as rf
    t = np.arange(0, 20, 0.2)                      # 5 Hz
    res = rf.check_measured(t, np.full_like(t, 400.0),
                            np.full_like(t, 150.0))
    assert not res.rate_adequate


def test_low_speed_regen_is_caught():
    run = ex.run_express("are we legal?", [("run.csv", _ev_log())])
    body = next(a for a in run.artifacts
                if a.path.endswith("measured_check.md")).data.decode()
    assert "EV.3.3.3" in body


def test_the_report_states_what_it_could_not_check():
    run = ex.run_express("are we legal? 80 kW", None)
    body = next(a for a in run.artifacts
                if a.path.endswith("declared_check.md")).data.decode()
    assert "unchecked rule is not a passed rule" in body.lower()


def test_pack_energy_drives_a_module_count():
    from suspension import rules_fsae as rf
    fs = rf.check_declared({"pack_kwh": 6.5})
    msg = " ".join(f.message for f in fs)
    assert "at least" in msg and "modules" in msg


def test_module_phrases_beat_the_generic_mass_word():
    """'module mass 11 kg' must not be read as the car's mass — the word
    'mass' sits inside the phrase."""
    a = ex.parse_request("module mass 11 kg on a 6.5 kWh pack")
    assert a.params.get("module_kg") == pytest.approx(11.0)
    assert "mass_kg" not in a.params


def test_voltage_units_parse():
    a = ex.parse_request("400 V system voltage")
    assert a.params["pack_v_max"] == pytest.approx(400.0)


def test_rule_ids_are_unique_and_findings_reference_real_rules():
    from suspension import rules_fsae as rf
    ids = [r.rid for r in rf.RULES]
    assert len(ids) == len(set(ids))
    for f in rf.check_declared({"power_kw": 90.0}):
        assert f.rule is not None, f.rid


# =========================================================================== #
#  12 · Multi-file uploads — one bundle, one timebase
# =========================================================================== #
def test_two_logs_of_different_lengths_do_not_corrupt_the_bundle():
    """Channels from two files with different row counts cannot share a mask.
    Merging them into one flat namespace raised IndexError in four jobs the
    first time anyone dropped two logs at once."""
    run = ex.run_express("", [("long.csv", _ev_log(n=3000)),
                              ("short.csv", _four_pot_log(n=400))])
    assert not run.failed, run.failed


def test_the_timebase_file_wins_and_says_so():
    db = ex.sniff_files([("long.csv", _ev_log(n=3000)),
                         ("short.csv", _four_pot_log(n=400))])
    assert "time" in db.series
    assert db.channels["time"].n == 3000
    assert any("timebase taken from" in r for r in db.receipts)


def test_unmerged_channels_are_named_not_dropped_silently():
    db = ex.sniff_files([("long.csv", _ev_log(n=3000)),
                         ("short.csv", _four_pot_log(n=400))])
    warn = " ".join(db.warnings)
    assert "not merged" in warn
    assert "damper" in warn.lower()
    assert "Drop them separately" in warn


def test_a_single_file_is_unaffected():
    db = ex.sniff_files([("only.csv", _four_pot_log(n=400))])
    for c in ("time", "ay", "damper_fl", "damper_rr"):
        assert c in db.series
    assert not any("not merged" in w for w in db.warnings)


def test_budget_words_are_not_reported_as_unknown():
    a = ex.parse_request("give me the roll numbers, I have 5 minutes")
    assert "minutes" not in a.ignored and "have" not in a.ignored


# =========================================================================== #
#  13 · The parameter table's ordering contract, enforced instead of trusted
# =========================================================================== #
def test_no_generic_phrase_precedes_a_specific_one_it_matches_inside():
    """The contract that has now cost three separate bugs: 'laps' stealing
    from 'log covers', 'mass' from 'module mass', 'weight' from 'weight
    distribution'. Each was found only after the fact. This finds the fourth
    before anyone ships it."""
    problems = ex.validate_param_table()
    assert not problems, "\n".join(problems)


def test_the_validator_actually_detects_a_violation():
    """A guard nobody has seen fail is a guard nobody should trust."""
    original = list(ex._PARAM_WORDS)
    try:
        ex._PARAM_WORDS.insert(0, ("bogus_generic", "mass", 1.0, ("module",)))
        assert ex.validate_param_table()
    finally:
        ex._PARAM_WORDS[:] = original
    assert not ex.validate_param_table()


def test_a_bare_number_answering_a_fraction_is_a_percentage():
    """'weight distribution 47' means 47 %. Storing 47.0 into a 0..1
    parameter put load transfers out by two orders of magnitude, silently,
    because 47 is a perfectly ordinary looking number."""
    a = ex.parse_request("weight distribution 47")
    assert a.params["weight_dist_front"] == pytest.approx(0.47)


def test_an_explicit_fraction_is_left_alone():
    a = ex.parse_request("weight distribution 0.47")
    assert a.params["weight_dist_front"] == pytest.approx(0.47)


def test_a_number_that_cannot_be_a_fraction_is_refused_not_mangled():
    a = ex.parse_request("weight distribution 470")
    assert a.params.get("weight_dist_front", 0.47) == 0.47   # fell back
    assert any("cannot be a fraction" in x for x in a.assumptions)


def test_weight_distribution_no_longer_loses_its_number_to_mass():
    a = ex.parse_request("weight distribution 47 and a 280 kg car")
    assert a.params["weight_dist_front"] == pytest.approx(0.47)
    assert a.params["mass_kg"] == pytest.approx(280.0)


# =========================================================================== #
#  14 · Cooling, printed parts, powertrain — from the meeting deck
# =========================================================================== #
def test_printed_substitution_is_not_reassuring():
    """'Onyx is pretty similar to PAHT-CF' is a judgement that should survive
    being written down as a ratio. At coolant temperature it does not."""
    from suspension import printed_parts as pp
    sub = pp.substitute("paht_cf", "onyx", pp.Duty(service_temp_c=80.0))
    assert sub.ratio < 0.6
    assert "NOT A SUBSTITUTION" in sub.verdict or "MATERIAL CHANGE" in sub.verdict
    assert any("FEA" in a for a in sub.actions)


def test_every_knockdown_explains_itself():
    from suspension import printed_parts as pp
    a = pp.derate("onyx", pp.Duty(service_temp_c=80.0))
    assert len(a.factors) == 4
    for name, factor, why in a.factors:
        assert 0.0 <= factor <= 1.0
        assert why, f"{name} gave no reason"


def test_above_hdt_is_refused_not_derated():
    from suspension import printed_parts as pp
    assert not pp.derate("petg_cf", pp.Duty(service_temp_c=95.0)).viable


def test_print_orientation_changes_the_manifold_verdict():
    """A tube printed upright has its layer lines running around the hoop —
    the direction hoop stress pulls. It is a structural decision currently
    made by whoever loads the plate."""
    from suspension import printed_parts as pp
    up = pp.manifold_check("onyx", inner_dia_mm=25.0, wall_mm=3.0,
                           pressure_bar=1.5, printed_upright=True)
    flat = pp.manifold_check("onyx", inner_dia_mm=25.0, wall_mm=3.0,
                             pressure_bar=1.5, printed_upright=False)
    assert flat.fos > up.fos * 1.5


def test_a_typical_first_rig_sensor_list_fails_its_own_target():
    from suspension import cooling as cl
    r = cl.rig_uncertainty(heat_w=4000.0, flow_lpm=12.0, coolant_key="eg50",
                           temp_sensor="pt100_b", flow_meter="paddle")
    assert r.u_total_rel > 0.10
    assert r.dominant == "temperature difference"


def test_a_matched_pair_is_worth_more_than_a_better_flow_meter():
    """The point of the whole module: the money goes on the ΔT measurement,
    not the flow meter."""
    from suspension import cooling as cl
    base = dict(heat_w=4000.0, flow_lpm=12.0, coolant_key="eg50")
    better_flow = cl.rig_uncertainty(**base, temp_sensor="pt100_b",
                                     flow_meter="coriolis")
    better_temp = cl.rig_uncertainty(**base, temp_sensor="matched_pair",
                                     flow_meter="paddle")
    assert better_temp.u_total_rel < better_flow.u_total_rel


def test_lower_flow_measures_better():
    """Halving the flow doubles ΔT and halves the temperature error term —
    a throttling valve decided before the hoses are cut."""
    from suspension import cooling as cl
    lo = cl.rig_uncertainty(heat_w=4000.0, flow_lpm=6.0, coolant_key="eg50",
                            temp_sensor="pt100_b", flow_meter="turbine")
    hi = cl.rig_uncertainty(heat_w=4000.0, flow_lpm=18.0, coolant_key="eg50",
                            temp_sensor="pt100_b", flow_meter="turbine")
    assert lo.u_total_rel < hi.u_total_rel


def test_an_unreachable_target_returns_none_rather_than_a_number():
    from suspension import cooling as cl
    assert cl.required_delta_t(temp_sensor="pt100_b", flow_meter="paddle",
                               target_resolution_rel=0.03) is None
    assert cl.required_delta_t(temp_sensor="pt100_b",
                               flow_meter="turbine") > 0


def test_an_undersized_radiator_is_caught_and_explained():
    from suspension import cooling as cl
    r = cl.size_loop(cl.LoopSpec(heat_w=9000.0, radiator_ua_w_per_k=60.0))
    assert not r.ok
    assert any("undersized" in n for n in r.notes)


def test_the_deck_scenario_runs_clean():
    run = ex.run_express(
        "Our print bureau only stocks Onyx, not PAHT-CF. 80 kW, coolant "
        "temperature 80, 25 mm bore 3 mm wall. Check the cooling loop, the "
        "test rig instrumentation, the printed manifold, the gear ratio and "
        "the tractive system.", None)
    assert not run.failed, run.failed
    for jid in ("cooling_loop", "cooling_rig", "printed_part_check",
                "gear_ratio", "tractive_safety"):
        assert jid in run.ran, f"{jid} did not run"


def test_gear_ratio_report_names_the_false_dependency():
    """The deck holds gear ratio behind output shaft CAD. The dependency
    runs the wrong way and the report has to say so."""
    run = ex.run_express("what final drive should we run?", None)
    body = next(a for a in run.artifacts
                if a.path.endswith("gear_ratio.md")).data.decode()
    assert "did not need the output shaft CAD" in body
    assert "grip" in body.lower()


def test_cooling_params_bind():
    a = ex.parse_request("coolant temperature 85, flow rate 9 L/min, "
                         "25 mm bore, 2.5 mm wall thickness, cap pressure "
                         "1.4 bar")
    assert a.params["coolant_c"] == pytest.approx(85.0)
    assert a.params["flow_lpm"] == pytest.approx(9.0)
    assert a.params["bore_mm"] == pytest.approx(25.0)
    assert a.params["wall_mm"] == pytest.approx(2.5)
    assert a.params["cap_bar"] == pytest.approx(1.4)


def test_hyphenated_compounds_are_not_reported_as_unknown():
    """'paht-cf' was landing in 'not understood' even though 'paht' had
    already summoned the printed-parts engine."""
    a = ex.parse_request("we wanted PAHT-CF but only Onyx is available")
    assert "paht-cf" not in a.ignored
    assert "printing" in a.tools


def test_tractive_words_reach_the_ev_engine():
    a = ex.parse_request("check the precharge and the shutdown circuit")
    assert "ev" in a.tools


def test_torque_units_parse_and_bind():
    """'Nm' was missing from the unit table, so '140 Nm motor torque' parsed
    the 140 as a bare number and then lost a tie-break to a kWh figure four
    characters further away. Motor torque ended up holding a pack energy."""
    a = ex.parse_request("80 kW, 140 Nm motor torque, 6.5 kWh pack")
    assert a.params["motor_torque_nm"] == pytest.approx(140.0)
    assert a.params["pack_kwh"] == pytest.approx(6.5)
    assert a.params["power_kw"] == pytest.approx(80.0)


def test_a_dimensionless_parameter_prefers_a_number_without_a_unit():
    """Letting a dimensionless parameter take a unit-carrying quantity for
    free is what made the swap possible in the first place."""
    a = ex.parse_request("radiator UA 150, 6.5 kWh pack")
    assert a.params["rad_ua"] == pytest.approx(150.0)
    assert a.params["pack_kwh"] == pytest.approx(6.5)


def test_stiffness_still_beats_torque_on_n_per_mm():
    a = ex.parse_request("35 N/mm spring, 140 Nm peak torque")
    assert a.params["spring_rate_N_per_mm"] == pytest.approx(35.0)
    assert a.params["motor_torque_nm"] == pytest.approx(140.0)


# =========================================================================== #
#  15 · Wiring — the NEC chart, and everything it does not know
# =========================================================================== #
def test_ambient_and_bundling_multiply():
    """Each alone looks survivable. Together they take a hot loom to about a
    third of the chart value, and that is the step teams miss."""
    from suspension import wiring as wr
    hot = wr.derate("6", insulation="thhn", ambient_c=60.0, n_bundled=12)
    assert hot.base_a == 75.0
    #  Derived, not looked up: sqrt((90-60)/60) = 0.7071, which the published
    #  table rounds to 0.71. Assert the physics and check the rounding
    #  separately, so a future refinement to the derivation fails HERE with a
    #  clear reason rather than against a two-decimal reprint of it.
    expect = 75.0 * (0.5 ** 0.5) * 0.50
    assert hot.allowed_a == pytest.approx(expect, rel=1e-9)
    assert hot.allowed_a == pytest.approx(75.0 * 0.71 * 0.50, rel=0.01)
    assert hot.total_factor == pytest.approx(hot.temp_factor * 0.50)


def test_gauges_outside_the_nec_table_get_no_invented_ampacity():
    from suspension import wiring as wr
    assert wr.ampacity("20") is None
    assert wr.conductor("20").area_mm2 > 0        # area is still a fact


def test_unknown_ampacity_is_never_a_pass():
    """The first version treated a missing ampacity as 'no thermal
    objection' and recommended 16 AWG for 132 A RMS, because 16 AWG raised
    no objection at all. Unknown is not pass."""
    from suspension import wiring as wr
    r = wr.check_run("16", current_a=130.0, length_m=1.5, system_v=400.0)
    assert not r.ok
    assert any("Unknown is not a pass" in n for n in r.notes)


def test_log_sizing_never_returns_a_signal_wire_for_a_traction_current():
    from suspension import wiring as wr
    cur = np.full(2000, 130.0)
    ls = wr.size_from_log(cur, length_m=1.5, system_v=400.0,
                          ambient_c=60.0, n_bundled=12)
    assert ls.recommended_awg not in ("22", "20", "18", "16", "14")


def test_impossible_current_returns_no_gauge_and_a_parallel_count():
    from suspension import wiring as wr
    pick, _ladder = wr.recommend_gauge(current_a=200.0, length_m=1.5,
                                       system_v=400.0, ambient_c=60.0,
                                       n_bundled=12)
    assert pick is None
    n, per = wr.parallel_needed(200.0, ambient_c=60.0, n_bundled=12)
    assert n >= 2 and per > 0


def test_long_lv_runs_are_governed_by_volts_not_heat():
    from suspension import wiring as wr
    r = wr.check_run("14", current_a=15.0, length_m=4.0, system_v=12.0)
    assert r.governing == "voltage drop"
    assert any("Voltage drop governs" in n for n in r.notes)


def test_an_over_rated_fuse_does_not_protect_its_wire():
    from suspension import wiring as wr
    sev, msg = wr.fuse_coordination("6", 60.0, ambient_c=60.0, n_bundled=12)
    assert sev == "violation"
    assert "does not protect" in msg


def test_the_report_never_prints_none_awg():
    run = ex.run_express("80 kW at 400 V, 1.5 m cable run, ambient 60, "
                         "12 conductors bundled. what gauge?", None)
    body = next(a for a in run.artifacts
                if a.path.endswith("conductor_sizing.md")).data.decode()
    assert "None AWG" not in body
    assert "parallel" in body


def test_a_stated_fuse_is_not_reported_as_unstated():
    """Reporting 'no fuse stated' when a fuse WAS stated but no gauge could
    be chosen sends the reader to fix the wrong thing."""
    run = ex.run_express("400 V, ambient 60, 12 conductors bundled, "
                         "200 A fuse, 1.5 m cable run. what gauge?", None)
    body = next(a for a in run.artifacts
                if a.path.endswith("conductor_sizing.md")).data.decode()
    assert "No fuse rating given" not in body
    assert "200 A fuse was stated" in body


def test_wiring_job_runs_from_a_log():
    rows = ["Time,TS Voltage,TS Current",
            *[f"{i*0.01:.3f},400.0,{120+60*np.sin(i*0.01):.2f}"
              for i in range(2000)]]
    run = ex.run_express("ambient 55, 10 conductors bundled",
                         [("run.csv", "\n".join(rows).encode())])
    assert "wiring_from_log" in run.ran
    assert not run.failed, run.failed


def test_wiring_vocabulary_does_not_steal_harness():
    """In FSAE 'harness' is the driver restraint. Firing the wiring engine on
    it would send someone to the wrong document on the more safety-critical
    of the two meanings."""
    a = ex.parse_request("check the harness mounting loads")
    assert "frames" in a.tools
    assert "wiring" not in a.tools


def test_current_units_bind():
    a = ex.parse_request("60 A fuse on a 1.5 m cable run")
    assert a.params["fuse_a"] == pytest.approx(60.0)
    assert a.params["run_length_mm"] == pytest.approx(1500.0)


# =========================================================================== #
#  16 · Ampacity, derived rather than looked up
# =========================================================================== #
def test_the_derivation_reproduces_every_published_nec_factor():
    """The correction factors are sqrt((T_rating - T_ambient)/(T_rating-30)).
    That is not a model invented here — reproducing all nineteen published
    numbers is the proof that extrapolating past 90 C is legitimate."""
    from suspension import wiring as wr
    worst, n = 0.0, 0
    for rating, rows in wr.TEMP_CORRECTION.items():
        for amb, published in rows:
            derived = wr.correction_factor(rating, amb,
                                           include_resistance_shift=False)
            worst = max(worst, abs(derived - published))
            n += 1
    assert n == 19
    assert worst < 0.005, f"worst deviation {worst}"


def test_thhn_at_the_nec_basis_recovers_the_table_exactly():
    from suspension import wiring as wr
    assert wr.ampacity("6", "thhn", 30.0) == pytest.approx(75.0)
    assert wr.ampacity("2", "thhn", 30.0) == pytest.approx(130.0)


def test_high_temperature_wire_gets_a_number_not_an_apology():
    """The caveat this replaced: 'the 90 C column is a conservative floor,
    get the manufacturer's chart'. It is now computed."""
    from suspension import wiring as wr
    a90 = wr.ampacity("6", "thhn", 60.0)
    a150 = wr.ampacity("6", "tefzel", 60.0)
    a200 = wr.ampacity("6", "silicone", 60.0)
    assert a90 < a150 < a200
    assert a150 / a90 > 1.5
    assert a150 > wr.conductor("6").a_90c    # beats the 30 C chart value


def test_the_resistance_shift_is_applied_in_the_conservative_direction():
    """Copper resistance rises with temperature, so a 200 C conductor burns
    more per amp. Omitting it would be the optimistic direction."""
    from suspension import wiring as wr
    with_shift, _c = wr.ampacity_scale(200.0, 60.0,
                                       include_resistance_shift=True)
    without, _c2 = wr.ampacity_scale(200.0, 60.0,
                                     include_resistance_shift=False)
    assert with_shift < without


def test_a_termination_caps_the_conductor():
    """A 200 C cable into a 105 C lug is a 105 C circuit, and the melted end
    is at the termination, not the middle of the run."""
    from suspension import wiring as wr
    free = wr.derate("6", insulation="silicone", ambient_c=60.0)
    lug = wr.derate("6", insulation="silicone", ambient_c=60.0,
                    termination_c=105.0)
    assert lug.allowed_a < free.allowed_a
    assert any("Termination-limited" in n for n in lug.notes)


def test_an_unstated_termination_is_flagged_as_optimistic():
    from suspension import wiring as wr
    d = wr.derate("6", insulation="silicone", ambient_c=60.0)
    assert any("usually optimistic" in n for n in d.notes)


def test_the_report_quantifies_what_the_termination_wastes():
    run = ex.run_express("80 kW at 400 V, 1.5 m cable run, loom ambient 60, "
                         "12 conductors bundled, silicone, termination 105. "
                         "what gauge?", None)
    body = next(a for a in run.artifacts
                if a.path.endswith("conductor_sizing.md")).data.decode()
    assert "throws away" in body
    assert "as terminated" in body


def test_insulation_is_detected_from_the_sentence():
    from suspension.express_jobs import _detect_insulation
    assert _detect_insulation("we run silicone wire") == "silicone"
    assert _detect_insulation("M22759 loom") == "tefzel"
    assert _detect_insulation("building wire") == "thhn"
    assert _detect_insulation("no wire named") == "tefzel"   # declared default


def test_termination_parameter_binds():
    a = ex.parse_request("tefzel with termination 105 and a 60 A fuse")
    assert a.params["termination_c"] == pytest.approx(105.0)
    assert a.params["fuse_a"] == pytest.approx(60.0)


# =========================================================================== #
#  17 · DFMEA generated from the run, not typed up afterwards
# =========================================================================== #
DFMEA_ASK = ("80 kW at 400 V, 6.5 kWh pack, 1.5 m cable run, loom ambient 60, "
             "12 conductors bundled, coolant temperature 80, onyx instead of "
             "PAHT-CF. Check cooling, the rig, the printed manifold, wiring, "
             "rules, and give me the DFMEA.")


def _dfmea(run):
    return next(a for a in run.artifacts
                if a.path.endswith("generated_register.md")).data.decode()


def test_the_register_is_populated_from_other_jobs():
    """The answer to 'are DFMEAs worth the time': the document is not the
    valuable part, and it should cost nothing."""
    run = ex.run_express(DFMEA_ASK, None)
    assert not run.failed, run.failed
    assert "dfmea_autofill" in run.ran
    body = _dfmea(run)
    assert "failure modes" in body
    assert "seeded example" in body          # explicitly not the seed data


def test_the_harvester_runs_after_every_other_job():
    run = ex.run_express(DFMEA_ASK, None)
    assert run.ran.index("dfmea_autofill") == len(run.ran) - 1


def test_findings_are_traceable_to_the_job_that_raised_them():
    run = ex.run_express(DFMEA_ASK, None)
    body = _dfmea(run)
    assert "raised by" in body
    assert "Printed-part material substitution" in body


def test_an_unchecked_mode_scores_worst_on_detection():
    """'We never looked at that' becomes a number that sorts to the top,
    rather than an absence nobody notices."""
    from suspension.express_jobs import _DETECTION
    assert _DETECTION["unchecked"] > _DETECTION["modelled"] > _DETECTION["measured"]
    run = ex.run_express(DFMEA_ASK, None)
    csv_art = next(a for a in run.artifacts
                   if a.path.endswith("generated_register.csv"))
    text = csv_art.data.decode()
    assert "unchecked" in text


def test_the_onyx_substitution_reaches_the_register():
    run = ex.run_express(DFMEA_ASK, None)
    assert "Onyx" in _dfmea(run)


def test_the_register_warns_about_what_it_cannot_contain():
    """Absence in an auto-generated register is not evidence of safety."""
    body = _dfmea(ex.run_express(DFMEA_ASK, None))
    assert "absence here is not evidence of safety" in body
    assert "by hand" in body


def test_rpn_is_labelled_a_ranking_not_a_measurement():
    body = _dfmea(ex.run_express(DFMEA_ASK, None))
    assert "ranking" in body and "not a measurement" in body


def test_an_empty_run_produces_an_honest_empty_register():
    run = ex.run_express("give me the dfmea", None)
    body = _dfmea(run)
    assert "empty register, not a clean one" in body


def test_flags_do_not_break_determinism():
    a = ex.bundle_zip(ex.run_express(DFMEA_ASK, None))
    b = ex.bundle_zip(ex.run_express(DFMEA_ASK, None))
    assert a == b
