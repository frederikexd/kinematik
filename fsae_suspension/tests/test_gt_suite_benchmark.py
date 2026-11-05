"""The GT-SUITE benchmark (scripts/gt_suite_benchmark.py, docs/GT_SUITE_BENCHMARK.md) keeps its findings and its
safeguards: the reference files are pinned by SHA-256, the ICE predicts the published truck within the tolerance from
any single calibration speed, the EV matches on the city and highway cycles with every GT-SUITE value inside the band
of the assumed inputs, and the Argonne reader recovers a known drive efficiency from a log written in D3's layout."""
import importlib.util
import json
import shutil
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
_spec = importlib.util.spec_from_file_location("gt_suite_benchmark", ROOT / "scripts" / "gt_suite_benchmark.py")
gt = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gt)
ice = gt.ice
LIMIT = gt._refs()["tolerance"]["limit"]


def test_the_manifest_pins_every_reference_file(tmp_path, monkeypatch):
    assert gt.verify_manifest() == []
    names = [ln.split(maxsplit=1)[1] for ln in gt.MANIFEST.read_text().splitlines() if ln.strip()]
    assert {"references.json", "udds.csv", "hwfet.csv", "us06.csv"} <= set(names)
    assert sum(n.startswith("argonne/") for n in names) == len(gt._refs()["argonne"]["segments"]) + 2  # + launches
    assert sum(n.startswith("argonne_ice/") for n in names) == len(gt._refs()["argonne_ice"]["segments"]) + 2
    copy = tmp_path / "gt_suite"
    shutil.copytree(gt.BENCH, copy)
    monkeypatch.setattr(gt, "BENCH", copy)
    monkeypatch.setattr(gt, "MANIFEST", copy / "MANIFEST.sha256")
    (copy / "references.json").write_text((copy / "references.json").read_text().replace("116", "117", 1))
    assert gt.verify_manifest() == ["references.json"]
    assert gt.main([]) == 2


def test_ice_predicts_the_truck_from_any_single_calibration_speed():
    rows, crr, bte = gt.ice_rows()
    assert 0.004 < crr < 0.008 and 0.30 < bte < 0.45
    loo = gt.ice_leave_one_out()
    assert sorted(loo) == [40, 50, 60, 70, 80, 90]
    assert all(abs(p) <= LIMIT and abs(f) <= LIMIT for p, f in loo.values())


def test_ev_matches_with_regeneration_and_every_reference_lies_in_the_band():
    rows = {c: r for c, *r in gt.ev_rows()}
    assert set(rows) == {"udds", "hwy", "us06"}
    for cyc in ("udds", "hwy"):
        nom, _, _, _, ref = rows[cyc]
        assert abs(nom / ref - 1) <= LIMIT
    for nom, lo, hi, no_regen, ref in rows.values():
        assert lo <= nom <= hi and lo <= ref <= hi
        assert no_regen > nom and no_regen >= ref             # without regeneration: an upper bound


def test_stats():
    s = gt.stats([0.01, -0.03, 0.02])
    assert np.isclose(s["mae"], 0.02) and np.isclose(s["bias"], 0.0) and s["worst"] == -0.03


def _d3(path, t, v, p_batt, phase, volts=355.0):
    """A file in Argonne D3's layout: tab-separated, units in brackets, mph, 10 Hz, discharge current negative."""
    with open(path, "w") as fh:
        fh.write("Time[sec]\tDyno_Speed[mph]\tPhase_#\tHV_Battery_Current[A]\tHV_Battery_Voltage[V]\n")
        for row in zip(t, v / 0.44704, phase, -p_batt / volts, np.full_like(t, volts)):
            fh.write("\t".join(f"{x:.6f}" for x in row) + "\n")


def test_extraction_keeps_the_energy_and_the_fit_recovers_known_efficiencies(tmp_path, monkeypatch):
    ref = gt._refs()
    veh, crr, cda = ref["ev"]["vehicle"], 0.011, 0.72
    truth = (0.88, 0.68, 300.0)
    t, v = gt._cycle("udds.csv")
    tt = np.arange(0.0, t[-1] + 1.0, 0.1)
    vv = np.interp(tt, t, v)
    logs, segs = [], []
    for k, (name, scale) in enumerate((("a", 1.0), ("b", 1.25), ("c", 0.8))):
        log = {"time": tt, "speed": vv * scale}
        _, e = gt.model_wh_km({"time": tt, "speed": vv * scale}, veh, crr, cda, *truth, 0.09)
        p = np.diff(e, prepend=0.0) * 3600                     # the model's own power at each sample
        phase = np.where(tt < tt[-1] / 2, 1.0, 2.0)
        _d3(tmp_path / f"9{k} Test Data.txt", tt, vv * scale, p, phase)
        segs.append({"test": f"9{k}", "cycle": "udds", "phases": [1, 2], "argonne_wh": float(e[-1]) * 0.1})
        logs.append(log)
    monkeypatch.setattr(gt, "ARGONNE", tmp_path / "argonne")
    rows = gt.extract(tmp_path, {"argonne": {"segments": segs}})
    assert all(abs(wh / ref_wh - 1) < 0.01 for _, wh, ref_wh in rows)      # extraction keeps the energy
    loaded = [gt.load_segment(s) for s in segs]
    assert abs(gt.standstill_power_w(loaded) - truth[2]) < 15
    eta_d, eta_r = gt.fit_drive(loaded, veh, crr, cda, truth[2], 0.09)
    assert abs(eta_d - truth[0]) < 0.02 and abs(eta_r - truth[1]) < 0.05


def test_the_committed_argonne_segments_carry_argonnes_own_energies():
    for seg in gt._refs()["argonne"]["segments"]:
        log = gt.load_segment(seg)
        assert abs(np.sum(log["power"]) / 3600 / seg["argonne_wh"] - 1) < 0.005, gt.segment_name(seg)


def test_kinematik_predicts_the_measured_i3_and_the_gt_suite_paper_does_not():
    r = gt.argonne_rows()
    assert len(r["graded"]) == 7
    assert gt.stats(g[3] for g in r["graded"])["mae"] <= LIMIT
    assert all(abs(g[3]) <= 0.07 for g in r["graded"])
    assert 0.009 < r["crr"] < 0.012 and 0.6 < r["cda"] < 0.8
    gt_paper = {c: ref for c, *_, ref in gt.ev_rows()}
    for cyc, meas in r["cycle_means"].items():
        assert gt_paper[cyc] / meas - 1 > 0.15                       # the paper's GT-SUITE values sit well above


def test_the_blazer_segments_carry_the_logged_fuel_a_little_below_argonnes_bag_values():
    for seg in gt._refs()["argonne_ice"]["segments"]:
        log = gt._load_csv(gt.ARGONNE_ICE / f"{gt.segment_name(seg)}.csv")
        assert -0.05 < log["fuel_g_s"].sum() / seg["bag_fuel_g"] - 1 < 0.0, gt.segment_name(seg)


def test_the_willans_line_from_the_stairs_test_predicts_every_cycle_blind():
    r = gt.argonne_ice_rows()
    wl = r["willans"]
    assert 0.35 < wl.eta_indicated < 0.45 and 0.85 < r["driveline_eff"] < 1.0
    willans = {name: err for name, _, _, err in r["graded"]}
    one_bte = {name: err for name, _, _, err, _ in r["bte"]}
    assert len(willans) == 6
    assert all(abs(e) <= 0.07 for e in willans.values())                 # against Argonne's bag fuel
    assert gt.stats(willans.values())["mae"] <= LIMIT
    assert all(abs(ours / m - 1) <= LIMIT for _, m, ours in r["meter"])   # against the meter it was fitted with
    assert gt.stats(willans.values())["mae"] < 0.5 * gt.stats(one_bte.values())["mae"]
    assert all(e < -0.10 for n, e in one_bte.items() if "_udds_" in n)   # one BTE: the city is far under
    bte = {lo: e for lo, _, e in r["bte_by_power"]}
    assert bte[1] < 0.5 * bte[20] and 0.25 < bte[20] < 0.40


def _fsk_file(path, r_pack=0.15, ocv0=246.0, k=0.5, heat_capacity=25e3, cooling=5.0):
    """Telemetry in A-FA's layout: two stints of a pack with a known resistance and open-circuit voltage, and a cell
    temperature from a lumped pack of known heat capacity and cooling, written out step by step."""
    import datetime as dt
    msgpack = pytest.importorskip("msgpack")
    recs, t0 = [], dt.datetime(2023, 8, 27, 10, 0, 0)
    temp, ambient, t_prev = 300.0, 300.0, 0.0
    for stint, (soc0, soc1, start) in enumerate(((95.0, 71.0, 0.0), (71.0, 58.0, 1300.0))):
        n = 6000
        temp += -cooling * (temp - ambient) * (start - t_prev) / heat_capacity if stint else 0.0   # the stop
        for j in range(n):
            s = soc0 + (soc1 - soc0) * j / n
            amps = 0.0 if j % 10 < 3 else 20.0 + 90.0 * abs(np.sin(j / 37.0))
            volts = ocv0 + k * s - amps * r_pack
            stamp = (t0 + dt.timedelta(seconds=start + 0.1 * j)).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            recs += [{"datetime": stamp, "key": "CAN_INV_VOLTAGE", "parsed": {"dc_bus_voltage": volts}},
                     {"datetime": stamp, "key": "CAN_INV_CURRENT", "parsed": {"dc_bus_current": amps}},
                     {"datetime": stamp, "key": "CAN_BMS_CORE", "parsed": {"soc": s}}]
            if j % 5 == 0:
                c = temp - 273.15
                recs.append({"datetime": stamp, "key": "CAN_BMS_TEMP",
                             "parsed": {"temperature": {"max": {"value": c + 0.5}, "min": {"value": c - 0.5}}}})
            temp += (amps ** 2 * r_pack - cooling * (temp - ambient)) * 0.1 / heat_capacity
            t_prev = start + 0.1 * j
    path.write_bytes(msgpack.packb(recs))
    return path


def test_the_fsk_reader_and_pack_check_recover_a_known_pack(tmp_path):
    r = gt.fsk_rows(gt.read_fsk(_fsk_file(tmp_path / "data.msgpack")))
    assert r["r_stint1"] == pytest.approx(0.15, rel=1e-3) and r["r_stint2"] == pytest.approx(0.15, rel=1e-2)
    assert r["sag_predicted"] == pytest.approx(r["sag_measured"], rel=0.02)
    assert r["loss_predicted_wh"] == pytest.approx(r["loss_measured_wh"], rel=0.03)
    assert r["soc_range"][0] > 90 and r["soc_range"][1] < 60
    th = gt.fsk_thermal_rows(gt.read_fsk(tmp_path / "data.msgpack"))
    assert th["heat_capacity_j_k"] == pytest.approx(25e3, rel=0.03) and th["cooling_w_k"] == pytest.approx(5.0, rel=0.1)
    assert th["rise_predicted_k"] == pytest.approx(th["rise_measured_k"], rel=0.02) and th["rms_k"] < 0.05
    assert th["ambient_k"] == pytest.approx(300.0, abs=1e-6) and th["r_pack"] == pytest.approx(r["r_stint1"])


def test_the_pack_model_holds_on_team_a_fas_endurance(tmp_path):
    import os
    path = os.environ.get("KINEMATIK_FSK2023")
    if not path or not Path(path).exists():
        pytest.skip("set KINEMATIK_FSK2023 to data.msgpack from luftaquila/a-fa-telemetry-endurance-viewer")
    assert gt.sha256(path) == gt._refs()["fsk2023"]["file_sha256"]
    r = gt.fsk_rows(gt.read_fsk(path))
    assert abs(r["sag_predicted"] / r["sag_measured"] - 1) <= LIMIT
    assert abs(r["loss_predicted_wh"] / r["loss_measured_wh"] - 1) <= LIMIT
    assert 0.12 < r["r_stint1"] < 0.17
    th = gt.fsk_thermal_rows(gt.read_fsk(path))           # docs/POWERTRAIN_VALIDATION.md: within the sensor's 1 degC
    assert th["rms_k"] < 0.5 and th["max_abs_k"] < 1.0


def test_every_graded_paper_was_sealed_before_its_results():
    for case_id, case in gt._papers().items():
        if case.get("results"):
            assert gt.check_preregistered(case_id) == [], case_id


def _isolated_papers(tmp_path, monkeypatch, papers, dirty=False):
    monkeypatch.setattr(gt, "BENCH", tmp_path)
    monkeypatch.setattr(gt, "PAPERS", tmp_path / "papers.json")
    monkeypatch.setattr(gt, "SEALED", tmp_path / "preregistered")
    monkeypatch.setattr(gt, "REGISTER", tmp_path / "preregistrations.json")
    for name in ("udds.csv", "hwfet.csv"):
        shutil.copy(ROOT / "benchmarks" / "gt_suite" / name, tmp_path / name)
    monkeypatch.setattr(gt, "_git", lambda *a: " M x.py" if dirty and a[0] == "status" else
                        ("" if a[0] == "status" else "abc123"))
    (tmp_path / "papers.json").write_text(json.dumps(papers))


def test_a_paper_is_graded_only_by_predictions_sealed_before_its_results(tmp_path, monkeypatch):
    import datetime as dt
    ev = gt._refs()["ev"]
    v = ev["vehicle"]
    inputs = {"mass_kg": v["mass_kg"], "cd": v["cd"], "frontal_area_m2": v["frontal_area_m2"], "crr": v["crr"],
              "rho_kg_m3": 1.184, "tire_radius_m": v["tire_radius_m"], "axle_inertia_kgm2": v["axle_inertia_kgm2"],
              "drive_eff": 0.855, "regen_eff": 0.855, "regen_cap_kw": 50, "pack_ocv_v": 355.2, "pack_r_ohm": 0.09,
              "cycles": ["udds.csv", "hwfet.csv"]}
    _isolated_papers(tmp_path, monkeypatch, {"i3": {"type": "ev_cycle", "source": "SAE 2021-01-1245",
                                                    "inputs": inputs}}, dirty=True)
    with pytest.raises(ValueError, match="uncommitted changes"):
        gt.preregister("i3")
    assert gt.preregister("i3", allow_dirty=True)["dirty"] is True
    _isolated_papers(tmp_path, monkeypatch, gt._papers())
    when = dt.datetime(2026, 11, 4, 12, 0, tzinfo=dt.UTC)
    rec = gt.preregister("i3", now=when)
    assert rec["kinematik_commit"] == "abc123" and rec["created_utc"] == "2026-11-04T12:00:00+00:00"
    assert rec["dirty"] is False and set(rec["predictions"]) == {"udds.csv", "hwfet.csv"}
    assert len(gt._register()) == 1                                       # re-sealing replaces the entry
    assert gt.check_preregistered("i3") == []
    rows, problems = gt.grade_paper("i3", LIMIT)
    assert rows == [] and problems == ["i3: no results entered yet"]
    papers = gt._papers()
    papers["i3"]["results"] = {"udds.csv": 116.0, "hwfet.csv": 144.0}
    gt.PAPERS.write_text(json.dumps(papers))
    with pytest.raises(ValueError, match="already has results"):
        gt.preregister("i3")
    rows, problems = gt.grade_paper("i3", LIMIT)
    assert problems == [] and [r[0] for r in rows] == ["udds.csv", "hwfet.csv"]
    assert all(r[2] == r[3] and abs(r[2] / r[1] - 1) < 0.05 for r in rows)   # sealed = current model here
    assert gt.report_paper("i3", LIMIT) == 0
    # the seal breaks if the sealed file or the inputs change afterwards
    sealed = gt.SEALED / "i3.json"
    body = json.loads(sealed.read_text())
    body["predictions"]["udds.csv"] = 116.0
    sealed.write_text(json.dumps(body))
    assert gt.check_preregistered("i3") == ["i3: sealed file changed after registration"]
    assert gt.report_paper("i3", LIMIT) == 2
    papers["i3"]["inputs"]["crr"] = 0.012
    gt.PAPERS.write_text(json.dumps(papers))
    assert "i3: inputs changed after registration" in gt.check_preregistered("i3")
    gt.REGISTER.write_text("[]")
    assert gt.check_preregistered("i3") == ["i3: no pre-registration"]


def test_a_fuel_map_paper_is_predicted_at_its_held_out_points(tmp_path, monkeypatch):
    import math
    wl = ice.WillansLine(0.37, 140.0, 18.0)
    pts = [(n, t) for n in (5000.0, 7000.0, 9000.0, 11000.0) for t in (15.0, 30.0, 45.0)]
    fuel = {(n, t): float(wl.fuel_power_w(t * n * math.pi / 30, n, 599.0) / 43e3) for n, t in pts}
    cal = [p for k, p in enumerate(pts) if k % 3 != 1]
    held = [p for k, p in enumerate(pts) if k % 3 == 1]
    inputs = {"displacement_cc": 599.0, "lhv_mj_kg": 43.0, "calibration": [[n, t, fuel[(n, t)]] for n, t in cal],
              "predict": [[n, t] for n, t in held]}
    _isolated_papers(tmp_path, monkeypatch, {"r6": {"type": "fuel_map_holdout", "inputs": inputs}})
    gt.preregister("r6")
    papers = gt._papers()
    papers["r6"]["results"] = {f"{n:g}rpm_{t:g}Nm": fuel[(n, t)] for n, t in held}
    gt.PAPERS.write_text(json.dumps(papers))
    rows, problems = gt.grade_paper("r6", LIMIT)
    assert problems == [] and len(rows) == 4
    assert all(abs(s / r - 1) < 1e-9 for _, r, s, _ in rows)


def test_the_gear_study_follows_gt_suite_once_the_car_shifts_like_an_automatic():
    gr = gt.gear_launch_rows()
    assert len(gr["ours"]) == 25 and 0.4 < gr["scale"] < 2.0
    assert all(abs(o / r - 1) <= LIMIT for o, r in zip(gr["ours"], gr["ref"]))
    assert gr["rank"] > 0.9 and 1.0 < gr["sensitivity"] < 1.4          # an automatic's schedule: ranking matches
    mf = gr["max_force"]
    assert mf["rank"] < 0.5 and mf["sensitivity"] < 0.6                 # race shifting: a different car


def test_the_two_wheeler_reaches_gt_suites_top_speed_from_its_published_inputs():
    w = gt.ev_2w_rows()
    e = gt._refs()["ev_2w"]
    assert abs(w["top_kmh"] / e["top_speed_kmh"] - 1) <= LIMIT
    assert w["motor_rpm_at_gt_top"] == pytest.approx(e["motor_rpm_at_top"], rel=0.002)


def test_the_converted_avanza_findings_hold():
    av = gt.ev_avanza_rows()
    sg = gt._refs()["ev_avanza"]["single_gear"]
    assert abs(av["top_kmh"] / sg["top_speed_kmh"] - 1) <= LIMIT
    assert av["t_0_100"] < sg["t_0_100_s"] * (1 - LIMIT)         # the finding: KinematiK launches faster
    lo, hi = av["nedc_band"]
    assert lo <= av["gt_nedc_wh_km"] <= hi and av["nedc_regen"] < av["nedc_no_regen"]


def test_the_nedc_trace_follows_unece_r83():
    t, v = gt._cycle("nedc.csv")
    assert t[-1] == 1180 and abs(v.max() * 3.6 - 120) < 1e-3
    assert abs(np.trapezoid(v, t) / 1e3 - 10.97) < 0.02            # official 11.0 km; the ECE part is 1 % long


def test_the_ev_launch_matches_the_measured_i3_with_a_measured_motor_curve():
    r = gt.ev_launch_rows()
    assert len(r["measured"]) == 2 and 0.33 < r["r_eff"] < 0.35
    for meas, held in zip(r["measured"], r["held_out"]):
        assert all(abs(h / m - 1) <= LIMIT for h, m in zip(held, meas))
        assert all(sp / m - 1 < -0.08 for sp, m in zip(r["spec"], meas))  # the spec sheet alone is optimistic
    assert r["repeatability"][1] < 0.01
    for c in r["curves"]:                                                # torque falls off above ~5,000 rpm
        assert 210 < np.interp(2000, c.rpm, c.torque_nm) < 245 and np.interp(8000, c.rpm, c.torque_nm) < 130


def test_the_benchmark_leaves_the_fuel_list_as_it_found_it():
    before = dict(gt.ice.FUELS)
    gt.ice_rows()
    gt.argonne_ice_rows()
    assert gt.ice.FUELS == before
