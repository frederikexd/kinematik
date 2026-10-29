"""suspension/calibration.py on calibrations built here with known error components: a line with a planted
non-linearity, hysteresis and run-to-run spread, each recovered in percent of full scale; the rejection, drift,
temperature and offset reductions in closed form; the loading cycle counted point by point; the certificate's
second-order fit on a response generated from known coefficients; and the GUM budget summed by hand."""
import math

import numpy as np
import pytest

from suspension import calibration as cal

FS = 10000.0


def _cal_data(nl=0.0, hy=0.0, rp=0.0):
    """Two runs up and down; error in N is nl at mid load (up), +-hy/2 between directions, +-rp/2 between runs."""
    F, d, r, V = [], [], [], []
    for run, sgn in ((0, -1.0), (1, 1.0)):
        for direction in (1, -1):
            for L in (0.0, 2500.0, 5000.0, 7500.0, 10000.0):
                e = nl * math.sin(math.pi * L / FS) + direction * hy / 2 * math.sin(math.pi * L / FS) \
                    + sgn * rp / 2 * (L > 0)
                F.append(L); d.append(direction); r.append(run)
                V.append(0.2e-3 * (L + e) + 0.01)
    return F, V, d, r


def test_a_clean_calibration_has_only_the_reference_uncertainty():
    F, V, d, r = _cal_data()
    c = cal.calibrate(F, V, d, r, FS)
    assert c["sensitivity_mvv_per_n"] == pytest.approx(0.2e-3) and c["zero_mvv"] == pytest.approx(0.01)
    assert c["nonlinearity_pct_fs"] == pytest.approx(0.0, abs=1e-9) and c["hysteresis_pct_fs"] == pytest.approx(0.0,
                                                                                                                abs=1e-9)
    assert c["repeatability_pct_fs"] == pytest.approx(0.0, abs=1e-9)
    assert c["u_std_n"] == pytest.approx(0.001 * FS) and c["U_expanded_n"] == pytest.approx(20.0)
    assert c["U_expanded_pct_fs"] == pytest.approx(0.2)
    assert cal.calibrate(F, V, d, r, FS, u_reference_rel=0.002)["u_std_n"] == pytest.approx(20.0)


def test_each_planted_error_is_recovered():
    F, V, d, r = _cal_data(hy=30.0)
    c = cal.calibrate(F, V, d, r, FS)
    assert c["hysteresis_pct_fs"] == pytest.approx(100 * 30.0 / FS, rel=1e-6)
    F, V, d, r = _cal_data(rp=12.0)
    c = cal.calibrate(F, V, d, r, FS)
    assert c["repeatability_pct_fs"] == pytest.approx(100 * 12.0 / FS, rel=1e-6)
    F, V, d, r = _cal_data(nl=40.0)
    c = cal.calibrate(F, V, d, r, FS)
    # the best-fit line absorbs part of the bow: recompute the ascending deviation from that line here
    Fa, Va = np.array(F), np.array(V)
    s, z = np.polyfit(Fa, Va, 1)
    err = (Va - z) / s - Fa
    up = np.array(d) > 0
    want = max(abs(err[up & (Fa == L)].mean()) for L in np.unique(Fa))
    assert c["nonlinearity_pct_fs"] == pytest.approx(100 * want / FS, rel=1e-9)
    F, V, d, r = _cal_data(nl=40.0, hy=30.0, rp=12.0)
    c = cal.calibrate(F, V, d, r, FS)
    nl, hy, rp = (c[k] * FS / 100 for k in ("nonlinearity_pct_fs", "hysteresis_pct_fs", "repeatability_pct_fs"))
    u = math.sqrt((nl / math.sqrt(3)) ** 2 + (hy / 2 / math.sqrt(3)) ** 2 + (rp / 2 / math.sqrt(3)) ** 2 + 10.0 ** 2)
    assert c["u_std_n"] == pytest.approx(u) and c["U_expanded_n"] == pytest.approx(2 * u)


def test_one_direction_only_has_no_hysteresis_and_single_points_no_repeatability():
    F = [0.0, 5000.0, 10000.0]
    V = [0.0, 1.0 + 0.002, 2.0]
    c = cal.calibrate(F, V, [1, 1, 1], [0, 0, 0], FS)
    assert c["hysteresis_pct_fs"] == 0.0 and c["repeatability_pct_fs"] == 0.0 and c["nonlinearity_pct_fs"] > 0
    c = cal.calibrate(F, V, [-1, -1, -1], [0, 0, 0], FS)
    assert c["nonlinearity_pct_fs"] == 0.0                       # non-linearity is read on the ascending points


def test_bending_rejection_against_the_budget():
    r = cal.bending_rejection(20.0, FS, 189.0, 559.0)
    sr = 189.0 / 559.0
    assert r == pytest.approx({"measured_ratio": sr / 0.002, "required_ratio": sr / 0.005, "passes": True})
    assert cal.bending_rejection(-60.0, FS, 189.0, 559.0)["passes"] is False
    assert cal.bending_rejection(0.0, FS, 189.0, 559.0)["measured_ratio"] == math.inf
    assert cal.bending_rejection(60.0, FS, 189.0, 559.0, budget_pct_fs=1.0)["passes"] is True
    assert round(cal.bending_rejection(1.0, FS, 189.0, 559.0)["required_ratio"]) == 68


def test_zero_drift_temperature_coefficient_and_installation_offset():
    z = cal.zero_drift([0, 60, 120, 180], [1.0, 1.6, 2.2, 2.8], limit_n=2.0)
    assert z == pytest.approx({"shift_n": 1.8, "rate_n_per_s": 0.01, "within_limit": True})
    assert cal.zero_drift([0, 60, 120, 180], [1.0, 1.6, 2.2, 2.8], limit_n=1.8)["within_limit"] is True
    assert cal.zero_drift([0, 60], [1.0, 3.0], limit_n=1.0)["within_limit"] is False
    assert cal.zero_drift([5.0], [2.0], 1.0) == {"shift_n": 0.0, "rate_n_per_s": 0.0, "within_limit": True}
    t = cal.temperature_coeff([10, 20, 30, 40], [1.0, 1.5, 2.1, 2.4])
    a, b = np.polyfit([10, 20, 30, 40], [1.0, 1.5, 2.1, 2.4], 1)
    res = np.array([1.0, 1.5, 2.1, 2.4]) - (a * np.array([10, 20, 30, 40]) + b)
    assert t == pytest.approx({"coeff_n_per_c": a, "zero_at_0c_n": b, "rms_n": math.sqrt(np.mean(res ** 2))})
    assert cal.installation_offset(130.0, 100.0, 25.0) == {"offset_n": 30.0, "significant": True}
    assert cal.installation_offset(80.0, 100.0, 25.0) == {"offset_n": -20.0, "significant": False}


def test_the_loading_schedule_point_by_point():
    pts = cal.loading_schedule(5000.0)
    first = pts[:6]
    assert [(p["phase"], p["load_n"], p["min_hold_s"]) for p in first] == [
        ("preload", 5000.0, 60.0), ("zero", 0.0, 60.0)] * 3
    meas = [p for p in pts if p["orientation_deg"] == 0.0 and p["phase"] == "measure"]
    assert [p["load_n"] for p in meas] == [0, 1000, 2000, 3000, 4000, 5000, 4000, 3000, 2000, 1000, 0]
    assert all(p["recorded"] and p["min_hold_s"] == 30.0 for p in meas)
    assert not any(p["recorded"] for p in pts if p["phase"] != "measure")
    assert len(pts) == 6 + 11 + 2 * (2 + 11)
    o120 = [p for p in pts if p["orientation_deg"] == 120.0]
    assert [p["phase"] for p in o120[:3]] == ["preload", "zero", "measure"]
    s = cal.loading_schedule(100.0, n_steps=2, n_preloads=1, orientations_deg=(0.0, 90.0), preload_hold_s=5.0,
                             zero_hold_s=6.0, step_dwell_s=7.0)
    assert [(p["orientation_deg"], p["phase"], p["load_n"], p["min_hold_s"]) for p in s] == [
        (0.0, "preload", 100.0, 5.0), (0.0, "zero", 0.0, 6.0), (0.0, "measure", 0.0, 7.0), (0.0, "measure", 50.0, 7.0),
        (0.0, "measure", 100.0, 7.0), (0.0, "measure", 50.0, 7.0), (0.0, "measure", 0.0, 7.0),
        (90.0, "preload", 100.0, 5.0), (90.0, "zero", 0.0, 6.0), (90.0, "measure", 0.0, 7.0),
        (90.0, "measure", 50.0, 7.0), (90.0, "measure", 100.0, 7.0), (90.0, "measure", 50.0, 7.0),
        (90.0, "measure", 0.0, 7.0)]


def test_the_certificate_reduction():
    x = np.array([0.0, 2500.0, 5000.0, 7500.0, 10000.0])
    resp = lambda xx: 0.002 + 2e-4 * xx - 3e-10 * xx ** 2
    inc = [resp(x) + 1e-4, resp(x) - 1e-4, resp(x)]
    dec = [resp(x) - 5e-4 + 1e-4, resp(x) - 5e-4 - 1e-4, resp(x) - 5e-4]
    c = cal.certificate_reduction(x, inc, dec)
    A = resp(x) - resp(0.0)
    assert c["mean_inc"] == pytest.approx(resp(x)) and c["mean_dec"] == pytest.approx(resp(x) - 5e-4)
    assert c["span_A"] == pytest.approx(A) and c["poly_B"] == pytest.approx(A)
    assert c["response_coeffs"] == pytest.approx((-3e-10, 2e-4)) and c["nonlinearity_pct"] == pytest.approx(0, abs=1e-9)
    assert c["hysteresis_pct"] == pytest.approx(100 * 5e-4 / A[-1] * np.ones(5))
    assert c["std_dev"] == pytest.approx(np.vstack([inc, dec]).std(0, ddof=1))
    f1, f2 = c["force_coeffs"]
    assert f1 * A ** 2 + f2 * A == pytest.approx(x, rel=2e-3, abs=2.0)
    assert c["max_hysteresis_pct"] == pytest.approx(100 * 5e-4 / A[-1])
    assert c["delta"] == pytest.approx(c["poly_B"] - c["span_A"])
    bent = cal.certificate_reduction(x, [resp(x) + [0, 0, 3e-3, 0, 0]], [resp(x)])
    assert bent["std_dev"] == pytest.approx(np.zeros(5))
    i = int(np.argmax(np.abs(bent["nonlinearity_pct"])))
    assert bent["max_nonlinearity_pct"] == bent["nonlinearity_pct"][i] and i == 2
    assert bent["max_hysteresis_pct"] == pytest.approx(100 * 3e-3 / bent["span_A"][-1])


def test_the_uncertainty_budget():
    b = cal.uncertainty_budget({"a": (0.2, "normal_k2"), "b": (0.3, "rectangular"), "c": (0.05, "std")})
    u = {"a": 0.1, "b": 0.3 / math.sqrt(3), "c": 0.05}
    uc = math.sqrt(sum(v * v for v in u.values()))
    assert b["standard"] == pytest.approx(u) and set(b) == {"standard", "combined_pct", "expanded_pct", "k"}
    assert (b["combined_pct"], b["expanded_pct"], b["k"]) == pytest.approx((uc, 2 * uc, 2.0))
    assert cal.uncertainty_budget({"a": (0.2, "std")}, k=3.0)["expanded_pct"] == pytest.approx(0.6)
    with pytest.raises(ValueError) as e:
        cal.uncertainty_budget({"a": (0.2, "triangular")})
    assert str(e.value) == "unknown distribution triangular"


def test_the_calibration_header_check():
    good = {"channel_id": "LF", "transducer_serial": "S1", "valid_until": "2026-12-31",
            "coeffs": {"tension": (1, 2), "compression": (1, 2)}}
    assert cal.check_calibration_header(good, {"channel_id": "LF", "transducer_serial": "S1", "date": "2026-10-23"}) \
        == {"ok": True, "problems": []}
    r = cal.check_calibration_header({"channel_id": "LF", "transducer_serial": "S2", "valid_until": "2026-01-01",
                                      "coeffs": {"tension": (1, 2)}},
                                     {"channel_id": "LR", "transducer_serial": "S1", "date": "2026-10-23"})
    assert r == {"ok": False, "problems": ["channel_id: calibration 'LF' vs channel 'LR'",
                                           "transducer_serial: calibration 'S2' vs channel 'S1'",
                                           "calibration expired 2026-01-01", "no compression coefficients"]}
    assert cal.check_calibration_header({"channel_id": "LF", "transducer_serial": "S1"},
                                        {"channel_id": "LF", "transducer_serial": "S1"})["problems"] == [
        "no tension coefficients", "no compression coefficients"]
    assert cal.check_calibration_header(good, {"channel_id": "LF", "transducer_serial": "S1",
                                               "date": "2026-12-31"})["ok"] is True


def test_the_supplement_s7_4_budget():
    b = cal.gum_budget_s74()
    want = {"Calibration non-linearity": 0.125, "Calibration hysteresis": 0.125,
            "Repeatability (3 repeats)": 0.1 / math.sqrt(3), "Creep (20 min)": 0.01, "Reference standard": 0.025,
            "Thermal zero shift (corrected)": 0.006, "Bending contamination": 0.25 / (2 * math.sqrt(3)),
            "Torsion contamination": 0.25 / (2 * math.sqrt(3)), "DAQ resolution": 0.002,
            "Excitation stability": 0.05 / math.sqrt(3) / 2, "Dynamic bandwidth": 1.21 * 0.1 * 0.5,
            "Bridge non-linearity (absorbed by the fit)": 0.021}
    assert b["standard"] == pytest.approx(want)
    assert b["combined_pct"] == pytest.approx(math.sqrt(sum(v * v for v in want.values())))
    lean = cal.gum_budget_s74(reference_u_pct=0.05, include_bridge_nl=False, dynamic_ci=0.2)
    assert "Bridge non-linearity (absorbed by the fit)" not in lean["standard"]
    assert lean["standard"]["Reference standard"] == 0.05 and lean["standard"]["Dynamic bandwidth"] == pytest.approx(
        0.121)


def test_a_direction_of_zero_is_neither_ascending_nor_descending():
    F = [0.0, 100.0, 100.0, 100.0, 200.0, 0.0]
    V = [0.001, 0.201, 0.203, 0.230, 0.4, 0.0]
    r = cal.calibrate(F, V, [1, 1, -1, 0, 1, -1], [1] * 6, 200.0)
    s, z = np.polyfit(F, V, 1)
    e = (np.array(V) - z) / s - np.array(F)
    assert r["hysteresis_pct_fs"] == pytest.approx(100 * max(abs(e[0] - e[5]), abs(e[1] - e[2])) / 200.0, rel=1e-9)
    assert r["nonlinearity_pct_fs"] == pytest.approx(100 * max(abs(e[0]), abs(e[1]), abs(e[4])) / 200.0, rel=1e-9)


def test_two_zero_readings_give_a_drift_rate():
    assert cal.zero_drift([0.0, 10.0], [1.0, 3.0], 5.0)["rate_n_per_s"] == pytest.approx(0.2)


def test_a_two_run_certificate_with_a_cubic_response():
    x = np.array([0.0, 1.0, 2.0, 3.0, 4.0])
    inc = np.array([0.1 + 2 * x + 0.05 * x ** 3, 0.12 + 2 * x + 0.05 * x ** 3])
    dec = inc - 0.01
    r = cal.certificate_reduction(x, inc, dec)
    assert r["std_dev"] == pytest.approx(np.vstack([inc, dec]).std(0, ddof=1))
    A = inc.mean(0) - inc.mean(0)[0]
    (m1, m2), *_ = np.linalg.lstsq(np.column_stack([x ** 2, x]), A)
    B = m1 * x ** 2 + m2 * x
    assert r["nonlinearity_pct"] == pytest.approx(100 * (B - A) / A[-1], rel=1e-9) and np.ptp(B - A) > 0.01
