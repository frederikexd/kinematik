"""suspension/aero/pressure_tap.py worked by hand: the linear calibration with its rail, the NaN-safe tap means, the
C_p reduction against the pitot or 1/2 rho V^2, the chordwise slices, the chord overlap and the normal-load integral
integrated exactly between taps, the suction peak and the recovery fit, and the tap-for-tap RMSE correlation with every summary
sentence checked word for word."""
import math

import numpy as np
import pytest

from suspension.aero import pressure_tap as pt
from suspension.aero.cfd import Attitude
from suspension.aero.windtunnel import GroundState

S, P = pt.WingSurface.SUCTION, pt.WingSurface.PRESSURE


def test_the_tap_location_and_the_surfaces():
    assert [s.value for s in pt.WingSurface] == ["suction", "pressure", "unknown"]
    t = pt.TapLocation("T1")
    assert (t.element, t.x_over_c, t.span_m, t.surface) == ("main", 0.0, 0.0, S)
    t = pt.TapLocation("T7", "flap1", 0.35, 0.1234, P)
    assert t.key() == "T7" and t.label() == "T7 [flap1 pressure x/c=0.35 y=0.123m]"


def test_the_calibration():
    c = pt.TapCalibration(200.0, zero_offset_v=0.05, is_calibrated=True, saturation_v=-4.0, serial="S9")
    assert c.volts_to_pressure([1.05, -3.0, 4.0, -4.0, 3.9999999999999]) == pytest.approx(
        [200.0, -610.0, np.nan, np.nan, np.nan], nan_ok=True)
    assert c.status() == "channel S9: 200 Pa/V, zero +0.0500 V, rail ±4 V"
    free = pt.TapCalibration(150.0, is_calibrated=True)
    assert free.volts_to_pressure([10.0]).tolist() == [1500.0] and free.status() == "channel ?: 150 Pa/V, zero +0.0000 V"
    raw = pt.TapCalibration(150.0)
    assert (raw.zero_offset_v, raw.is_calibrated, raw.saturation_v, raw.serial, raw.notes) == (0.0, False, None, "", "")
    assert np.isnan(raw.volts_to_pressure([1.0, 2.0])).all() and raw.volts_to_pressure([[1.0, 2.0]]).shape == (1, 2)
    assert raw.status() == ("channel ?: UNCALIBRATED — span/zero not established; readings are not pressures and "
                            "reduce to holes")
    assert pt.TapCalibration(1.0, serial="X").status().startswith("channel X: UNCALIBRATED")


def test_the_provenance():
    p = pt.ScanProvenance("Tunnel A")
    assert (p.rho, p.speed_ms, p.p_static_inf_pa, p.p_total_inf_pa, p.sample_rate_hz, p.sample_seconds, p.ground_state,
            p.blockage_corrected, p.blockage_ratio, p.reynolds, p.notes) == (
        1.225, 20.0, 0.0, None, None, None, GroundState.MOVING_BELT, True, None, None, "")
    assert p.dynamic_pressure() == pytest.approx(245.0)
    assert pt.ScanProvenance("A", p_static_inf_pa=10.0, p_total_inf_pa=310.0).dynamic_pressure() == 300.0
    assert pt.ScanProvenance("A", rho=1.2, speed_ms=10.0, p_static_inf_pa=10.0,
                             p_total_inf_pa=10.0).dynamic_pressure() == pytest.approx(60.0)
    assert pt.ScanProvenance("A", sample_seconds=5.0).averaging_ok() is True
    assert pt.ScanProvenance("A", sample_seconds=4.9).averaging_ok() is False
    assert pt.ScanProvenance("A", sample_seconds=4.0).averaging_ok(min_seconds=4.0) is True
    assert p.averaging_ok() is False
    good = pt.ScanProvenance("Tunnel A", rho=1.2, speed_ms=25.0, p_static_inf_pa=-3.0, sample_seconds=10.0,
                             reynolds=4.5e5)
    assert good.status() == ("Tunnel A: q=375.0 Pa (½ρV², V=25 m/s, rho=1.2), p_inf=-3.0 Pa gauge, moving-belt floor, "
                             "blockage-corrected, Re=4.50e+05")
    bad = pt.ScanProvenance("B", p_static_inf_pa=5.0, p_total_inf_pa=305.0, ground_state=GroundState.FIXED_FLOOR,
                            blockage_corrected=False, sample_seconds=2.5)
    assert bad.status() == (
        "B: q=300.0 Pa (pitot, V=20 m/s, rho=1.225), p_inf=+5.0 Pa gauge, fixed-floor floor, NOT blockage-corrected"
        " — WARNING: averaging window 2.5s is short; freestream turbulence is left in the tap means as bias"
        " — WARNING: fixed/suction floor distorts the underbody/wing-floor pressure field; C_p near the ground is not "
        "ground-effect-true — WARNING: uncorrected static reference inflates suction C_p; correct blockage before "
        "comparing to free-air CFD")
    assert "averaging window unknown is short" in p.status()
    assert "(½ρV²," in pt.ScanProvenance("C", p_total_inf_pa=0.0, sample_seconds=9.0).status()


TAPS = [pt.TapLocation("a", x_over_c=0.1), pt.TapLocation("b", x_over_c=0.5), pt.TapLocation("c", x_over_c=0.9),
        pt.TapLocation("d", x_over_c=0.3, surface=P)]
CAL = {k: pt.TapCalibration(100.0, zero_offset_v=0.1, is_calibrated=True, saturation_v=5.0) for k in "abc"}


def test_the_raw_scan_and_its_reduction():
    v = np.array([[1.1, 2.1, 5.0, 0.3], [3.1, 2.1, 5.0, 0.4], [2.1, 5.1, 5.0, 0.5]])
    att = Attitude(pitch_deg=1.0)
    scan = pt.RawPressureScan(v, TAPS, dict(CAL), attitude=att)
    assert (scan.n_samples, scan.n_taps) == (3, 4)
    pres = scan.tap_pressures_pa()
    assert pres["a"] == pytest.approx(200.0) and pres["b"] == pytest.approx(200.0)
    assert math.isnan(pres["c"]) and math.isnan(pres["d"]) and list(pres) == ["a", "b", "c", "d"]
    prov = pt.ScanProvenance("T", p_static_inf_pa=20.0, p_total_inf_pa=420.0)
    f = scan.to_cp(prov)
    assert f.cp["a"] == pytest.approx(0.45) and f.cp["b"] == pytest.approx(0.45)
    assert math.isnan(f.cp["c"]) and math.isnan(f.cp["d"]) and f.attitude is att and f.provenance is prov
    assert f.taps == TAPS and f.taps is not TAPS
    zero_q = scan.to_cp(pt.ScanProvenance("T", speed_ms=0.0))
    assert all(math.isnan(x) for x in zero_q.cp.values())
    one = pt.RawPressureScan([1.1, 2.1, 3.1, 4.1], TAPS, dict(CAL))
    assert one.volts.shape == (1, 4) and one.n_samples == 1 and one.attitude is None


def test_the_scan_refuses_bad_shapes():
    for v, msg in ((np.zeros((2, 2, 4)), "volts must be a 2D (n_samples, n_taps) matrix"),
                   (np.zeros((2, 3)), "volts has 3 columns but 4 taps were given — one column per tap is required")):
        with pytest.raises(ValueError) as e:
            pt.RawPressureScan(v, TAPS, {})
        assert str(e.value) == msg
    with pytest.raises(ValueError) as e:
        pt.RawPressureScan(np.zeros((2, 2)), [pt.TapLocation("a"), pt.TapLocation("a")], {})
    assert str(e.value) == "tap_id values must be unique across the wing"


def _field(cps, taps=None, prov=None):
    taps = taps or [pt.TapLocation(k, el, x, y, s) for k, (el, x, y, s, _) in cps.items()]
    return pt.CpField(taps=taps, cp={k: v[4] for k, v in cps.items()}, provenance=prov)


WING = {"s1": ("main", 0.05, 0.0, S, -1.2), "s2": ("main", 0.2, 0.0, S, -2.4), "s3": ("main", 0.5, 0.0, S, -1.0),
        "s4": ("main", 0.9, 0.0, S, -0.2), "s5": ("main", 0.7, 0.0, S, math.nan),
        "p1": ("main", 0.1, 0.0, P, 0.6), "p2": ("main", 0.6, 0.0, P, 0.3), "p3": ("main", 0.95, 0.0, P, 0.1),
        "f1": ("flap", 0.5, 0.0, S, -0.8), "u1": ("main", 0.5, 0.0, pt.WingSurface.UNKNOWN, 0.0)}


def test_the_field_slices_and_its_coverage():
    f = _field(WING)
    assert f.valid_taps() == ["s1", "s2", "s3", "s4", "p1", "p2", "p3", "f1", "u1"]
    assert f.coverage() == pytest.approx(0.9) and pt.CpField([], {}).coverage() == 0.0
    x, c, ids = f.chordwise("main", S)
    assert x.tolist() == [0.05, 0.2, 0.5, 0.9] and c.tolist() == [-1.2, -2.4, -1.0, -0.2] and ids == ["s1", "s2", "s3",
                                                                                                    "s4"]
    assert x.dtype == float and f.chordwise("rudder", S)[2] == [] and f.chordwise("rudder", S)[0].size == 0
    missing = pt.CpField([pt.TapLocation("z")], {})
    assert missing.chordwise("main", S)[2] == []
    span = {"a": ("main", 0.1, 0.0, S, -1.0), "b": ("main", 0.2, 0.3, S, -2.0), "c": ("main", 0.3, 0.6, S, -3.0),
            "d": ("main", 0.4, 0.31, S, -4.0)}
    g = _field(span)
    assert g.chordwise("main", S, span_tol_m=0.05)[2] == ["b", "d"]
    assert g.chordwise("main", S, span_tol_m=0.0)[2] == ["d"]                 # the median station is 0.31
    assert g.chordwise("main", S, span_tol_m=0.31)[2] == ["a", "b", "c", "d"]
    assert pt.CpField([], {}).chordwise("main", S, span_tol_m=0.1)[2] == []
    assert f.status() == "CpField: 9/10 taps reduced (90% coverage); provenance unknown"
    pr = pt.ScanProvenance("X", sample_seconds=6.0)
    assert _field(WING, prov=pr).status() == f"CpField: 9/10 taps reduced (90% coverage); {pr.status()}"
    assert f._tap_by_id()["s3"].x_over_c == 0.5


def test_the_overlap_and_the_normal_load():
    f = _field(WING)
    assert f.chord_coverage("main") == pytest.approx((0.9 - 0.1) / (0.95 - 0.05))
    grid = np.linspace(0.1, 0.9, 800001)                     # dense: the exact piecewise-linear integral
    want = np.trapezoid(np.interp(grid, [0.1, 0.6, 0.95], [0.6, 0.3, 0.1])
                        - np.interp(grid, [0.05, 0.2, 0.5, 0.9], [-1.2, -2.4, -1.0, -0.2]), grid)
    assert f.normal_load_coefficient("main") == pytest.approx(want, rel=1e-9)
    assert f.normal_load_coefficient("main", min_chord_coverage=0.88) == pytest.approx(want, rel=1e-9)
    assert math.isnan(f.normal_load_coefficient("main", min_chord_coverage=0.9))
    assert math.isnan(f.normal_load_coefficient("flap")) and f.chord_coverage("flap") == 0.0
    one_p = _field({**{k: v for k, v in WING.items() if not k.startswith("p")}, "p1": WING["p1"]})
    assert math.isnan(one_p.normal_load_coefficient("main")) and one_p.chord_coverage("main") == 0.0
    apart = {"s1": ("m", 0.0, 0, S, -1.0), "s2": ("m", 0.3, 0, S, -1.0), "p1": ("m", 0.3, 0, P, 0.5),
             "p2": ("m", 0.9, 0, P, 0.5)}
    assert math.isnan(_field(apart).normal_load_coefficient("m")) and _field(apart).chord_coverage("m") == 0.0
    same = {"s1": ("m", 0.2, 0, S, -1.0), "s2": ("m", 0.2, 0.1, S, -2.0), "p1": ("m", 0.2, 0, P, 0.5),
            "p2": ("m", 0.2, 0.2, P, 0.4)}
    assert _field(same).chord_coverage("m") == 0.0
    full = {"s1": ("m", 0.0, 0, S, -1.0), "s2": ("m", 1.0, 0, S, -1.0), "p1": ("m", 0.0, 0, P, 0.5),
            "p2": ("m", 1.0, 0, P, 0.5)}
    assert _field(full).chord_coverage("m") == 1.0 and _field(full).normal_load_coefficient("m") == pytest.approx(1.5)
    span = {"s1": ("m", 0.0, 0.0, S, -1.0), "s2": ("m", 1.0, 0.0, S, -1.0), "p1": ("m", 0.0, 0.0, P, 0.5),
            "p2": ("m", 1.0, 0.0, P, 0.5), "s3": ("m", 0.5, 0.5, S, -9.0), "p3": ("m", 0.5, 0.5, P, 9.0),
            "s4": ("m", 0.4, 0.6, S, -9.0), "p4": ("m", 0.6, 0.6, P, 9.0)}
    g = _field(span)                                    # spans 0, 0.5, 0.6: the median station is 0.5
    assert g.chord_coverage("m", span_tol_m=0.15) == 0.0 and math.isnan(g.normal_load_coefficient("m", span_tol_m=0.15))
    assert g.chord_coverage("m") == 1.0


def test_the_suction_peak_and_the_stall_call():
    f = _field(WING)
    assert f.suction_peak("main") == (-2.4, 0.2) and all(math.isnan(x) for x in f.suction_peak("flap2"))
    v = f.stall_indicator("main")
    xa, ca = np.array([0.2, 0.5, 0.9]), np.array([-2.4, -1.0, -0.2])
    slope = np.polyfit(xa, ca, 1)[0]
    assert (v.element, v.stalled, v.peak_cp, v.peak_xc) == ("main", False, -2.4, 0.2)
    assert v.recovery_slope == pytest.approx(slope) and v.aft_cp_spread == pytest.approx(2.2)
    assert v.note == f"healthy recovery (dCp/d(x/c)={slope:+.2f}) from the suction peak to the trailing edge — flow attached"
    s = f.stall_indicator("main", recovery_slope_tol=4.0)
    assert s.stalled is True and s.note == (
        f"FLAT recovery (dCp/d(x/c)={slope:+.2f} < 4) aft of the suction peak — pressure stopped recovering; the "
        "suction surface has separated / stalled over the aft chord")
    flat = {"s1": ("m", 0.1, 0, S, -2.0), "s2": ("m", 0.5, 0, S, -1.4), "s3": ("m", 0.9, 0, S, -1.2)}
    exact = _field(flat).stall_indicator("m")
    assert exact.recovery_slope == pytest.approx(1.0) and exact.stalled is (exact.recovery_slope < 0.5)
    edge = _field(flat).stall_indicator("m", recovery_slope_tol=exact.recovery_slope)
    assert edge.stalled is False
    few = _field({"s1": ("m", 0.1, 0, S, -2.0), "s2": ("m", 0.5, 0, S, -1.4)}).stall_indicator("m")
    assert (few.stalled, few.note) == (False, "too few valid suction taps to judge recovery — not a verdict")
    assert all(math.isnan(x) for x in (few.peak_cp, few.peak_xc, few.recovery_slope, few.aft_cp_spread))
    te = _field({"s1": ("m", 0.1, 0, S, -1.0), "s2": ("m", 0.5, 0, S, -1.4), "s3": ("m", 0.9, 0, S, -2.0)})
    t = te.stall_indicator("m")
    assert (t.stalled, t.peak_cp, t.peak_xc) == (False, -2.0, 0.9) and math.isnan(t.recovery_slope)
    assert t.note == "suction peak is at the trailing edge — no aft chord to recover" and math.isnan(t.aft_cp_spread)
    assert v.as_dict() == dict(element="main", stalled=False, peak_cp=-2.4, peak_xc=0.2, recovery_slope=v.recovery_slope,
                               aft_cp_spread=v.aft_cp_spread, note=v.note)
    span = {"a": ("m", 0.1, 0.0, S, -3.0), "b": ("m", 0.5, 0.0, S, -2.0), "c": ("m", 0.9, 0.0, S, -1.0),
            "d": ("m", 0.2, 0.5, S, -9.0)}
    assert _field(span).suction_peak("m", span_tol_m=0.1) == (-9.0, 0.2)          # median station 0.5
    assert _field(span).stall_indicator("m", span_tol_m=0.1).note.startswith("too few valid suction taps")
    assert _field(span).stall_indicator("m").peak_cp == -9.0


def test_the_cfd_surface_and_the_residuals():
    c = pt.CFDSurfaceCp.from_pairs([(1, "-0.5"), ("b", 2)], backend="Fluent")
    assert c.cp == {"1": -0.5, "b": 2.0} and c.backend == "Fluent" and (c.turbulence_model, c.notes) == ("", "")
    assert c.attitude is None
    t = pt.TapLocation("a", x_over_c=0.25)
    r = pt.TapResidual(t, -1.0, -0.7, paired=True)
    assert r.residual == pytest.approx(0.3) and r.note == ""
    assert r.as_dict() == dict(tap=t.label(), cp_phys=-1.0, cp_cfd=-0.7, paired=True, note="", residual=pytest.approx(0.3))
    for a, b in ((None, 1.0), (1.0, None), (math.nan, 1.0), (1.0, math.inf)):
        assert math.isnan(pt.TapResidual(t, a, b).residual) and pt.TapResidual(t, a, b).paired is False
    assert pt.DEFAULT_CP_TOL == {"rmse": 0.10, "max_abs": 0.20, "min_coverage": 0.6}


def _phys(cps, prov=None):
    taps = [pt.TapLocation(k, x_over_c=0.1 * i) for i, k in enumerate(cps)]
    return pt.CpField(taps=taps, cp=dict(cps), provenance=prov)


def test_a_matched_correlation():
    prov = pt.ScanProvenance("T", sample_seconds=8.0)
    phys = _phys({"a": -1.0, "b": -0.5, "c": 0.2, "d": math.nan, "e": 0.1}, prov)
    cfd = pt.CFDSurfaceCp({"a": -0.95, "b": -0.6, "c": 0.25, "d": 0.0, "x": 1.0}, backend="OpenFOAM",
                          turbulence_model="kOmegaSST")
    r = pt.correlate_cp(phys, cfd)
    res = [0.05, -0.1, 0.05]
    assert (r.ok, r.n_paired, r.n_unpaired, r.within_tol) == (True, 3, 3, True)
    assert r.coverage == pytest.approx(0.6) and r.rmse == pytest.approx(math.sqrt(sum(x * x for x in res) / 3))
    assert r.bias == pytest.approx(sum(res) / 3) and r.max_abs_residual == pytest.approx(0.1)
    assert r.worst.tap.tap_id == "b" and r.tolerances == pt.DEFAULT_CP_TOL
    notes = {x.tap.tap_id: (x.paired, x.cp_phys, x.cp_cfd, x.note) for x in r.residuals}
    assert notes["d"] == (False, None, 0.0, "measured tap is a hole (uncalibrated/railed) — not paired")
    assert notes["e"] == (False, 0.1, None, "no CFD C_p at this tap — not paired")
    assert notes["x"] == (False, None, 1.0, "CFD tap not present in the physical scan — not paired")
    assert [x.tap.tap_id for x in r.residuals] == ["a", "b", "c", "d", "e", "x"]
    assert r.physical_provenance == prov.status() and r.cfd_provenance == "OpenFOAM (kOmegaSST)"
    assert r.summary == (
        "[Surface C_p] OpenFOAM (kOmegaSST) vs measured taps: 3 tap(s) paired, 3 unpaired (holes — excluded from "
        "RMSE). MATCHED — the CFD surface pressure reproduces the measured C_p distribution inside tolerance; the "
        f"simulated wing is loaded where the real one is. RMSE {r.rmse:.3f} C_p (bias {r.bias:+.3f}, max |ΔC_p| "
        f"0.100).  Worst tap: {r.worst.tap.label()} ΔC_p=-0.100 (CFD−phys).")
    d = r.as_dict()
    assert d["worst"]["tap"] == r.worst.tap.label() and len(d["residuals"]) == 6 and d["tolerances"] is not r.tolerances
    assert {k: d[k] for k in ("ok", "n_paired", "n_unpaired", "within_tol", "summary", "physical_provenance",
                              "cfd_provenance")} == dict(ok=True, n_paired=3, n_unpaired=3, within_tol=True,
                                                         summary=r.summary, physical_provenance=r.physical_provenance,
                                                         cfd_provenance=r.cfd_provenance)
    assert (d["rmse"], d["bias"], d["max_abs_residual"], d["coverage"]) == (r.rmse, r.bias, r.max_abs_residual,
                                                                            r.coverage)


def test_the_correlation_tolerances_one_by_one():
    phys = _phys({"a": -1.0, "b": -0.5, "c": 0.2})
    base = dict(a=-1.0, b=-0.5, c=0.2)
    ok = pt.correlate_cp(phys, pt.CFDSurfaceCp(base))
    assert ok.within_tol and ok.rmse == 0.0 and ok.cfd_provenance == "CFD" and ok.physical_provenance == ""
    assert ok.summary.startswith("[Surface C_p] CFD vs measured taps: 3 tap(s) paired. MATCHED")
    assert "Worst tap: " + ok.worst.tap.label() + " ΔC_p=+0.000" in ok.summary
    big = pt.correlate_cp(phys, pt.CFDSurfaceCp({**base, "a": -0.75}))
    assert big.rmse == pytest.approx(0.25 / math.sqrt(3)) and big.within_tol is False
    assert pt.correlate_cp(phys, pt.CFDSurfaceCp({**base, "a": -0.75}), tol={"max_abs": 0.3, "rmse": 0.2}).within_tol is True
    assert pt.correlate_cp(phys, pt.CFDSurfaceCp({**base, "a": -0.8}), tol={"rmse": 0.2}).within_tol is True   # max ~0.2
    assert pt.correlate_cp(phys, pt.CFDSurfaceCp({**base, "a": -0.8}), tol={"rmse": 0.11}).within_tol is False
    assert "MISMATCHED — the CFD pressure distribution drifts" in big.summary
    rm = pt.correlate_cp(phys, pt.CFDSurfaceCp({"a": -0.85, "b": -0.35, "c": 0.35}), tol={"max_abs": 1.0})
    assert rm.rmse == pytest.approx(0.15) and rm.within_tol is False
    edge = pt.correlate_cp(phys, pt.CFDSurfaceCp({"a": -0.85, "b": -0.35, "c": 0.35}),
                           tol={"max_abs": 1.0, "rmse": rm.rmse})
    assert edge.within_tol is True
    thin = pt.correlate_cp(_phys({"a": -1.0, "b": -0.5, "c": 0.2, "d": 0.3, "e": 0.0}), pt.CFDSurfaceCp(base))
    assert thin.coverage == pytest.approx(0.6) and thin.within_tol is True
    thinner = pt.correlate_cp(_phys({"a": -1.0, "b": -0.5, "c": 0.2, "d": 0.3, "e": 0.0, "f": 0.0}),
                              pt.CFDSurfaceCp(base))
    assert thinner.within_tol is False and thinner.summary.split(". ")[1].startswith("MISMATCHED")
    assert "  Coverage 50% is below the 60% floor — the RMSE is over too few taps to trust; instrument or sample more " \
           "of the wing." in thinner.summary
    assert "Coverage" not in thin.summary


def test_nothing_to_compare():
    r = pt.correlate_cp(_phys({"a": math.nan}), pt.CFDSurfaceCp({"z": 1.0}, backend="STAR"))
    assert (r.ok, r.n_paired, r.n_unpaired, r.coverage, r.worst, r.within_tol) == (False, 0, 2, 0.0, None, False)
    assert all(math.isnan(x) for x in (r.rmse, r.bias, r.max_abs_residual))
    assert r.summary == ("[Surface C_p] STAR vs measured taps: 0 tap(s) paired, 2 unpaired (holes — excluded from "
                         "RMSE). Nothing could be compared tap-for-tap — sample the CFD surface at the same tap_ids as "
                         "the physical scan, then re-correlate.")
    assert r.as_dict()["worst"] is None
    empty = pt.correlate_cp(pt.CpField([], {}), pt.CFDSurfaceCp({}))
    assert empty.coverage == 0.0 and empty.n_unpaired == 0
    assert empty.summary == ("[Surface C_p] CFD vs measured taps: 0 tap(s) paired. Nothing could be compared "
                             "tap-for-tap — sample the CFD surface at the same tap_ids as the physical scan, then "
                             "re-correlate.")
    inf = pt.correlate_cp(_phys({"a": -1.0}), pt.CFDSurfaceCp({"a": math.inf}))
    assert inf.n_paired == 0 and inf.residuals[0].paired is True and inf.ok is False



# ------------------------------------------------------------------ added edges
def test_the_pitot_edge_and_the_warning_list():
    tiny = pt.ScanProvenance("T", p_static_inf_pa=10.0, p_total_inf_pa=10.5, sample_seconds=9.0)
    assert tiny.dynamic_pressure() == 0.5 and "(pitot," in tiny.status()
    under = pt.ScanProvenance("T", p_static_inf_pa=20.0, p_total_inf_pa=10.0, sample_seconds=9.0)
    assert under.dynamic_pressure() == pytest.approx(0.5 * 1.225 * 400) and "(½ρV²," in under.status()
    assert pt.ScanProvenance("T", sample_seconds=9.0, blockage_corrected=False).status().endswith(
        "NOT blockage-corrected — WARNING: uncorrected static reference inflates suction C_p; correct blockage "
        "before comparing to free-air CFD")


def test_the_reduction_with_a_small_q_and_an_uncalibrated_first_tap():
    taps = [pt.TapLocation("a"), pt.TapLocation("b")]
    cal = {"b": pt.TapCalibration(10.0, is_calibrated=True)}
    scan = pt.RawPressureScan(np.array([[1.0, 2.0], [1.0, 4.0]]), taps, cal)
    assert scan.tap_pressures_pa()["b"] == 30.0 and math.isnan(scan.tap_pressures_pa()["a"])
    cp = scan.to_cp(pt.ScanProvenance("T", rho=1.0, speed_ms=1.0, p_static_inf_pa=5.0))
    assert math.isnan(cp.cp["a"]) and cp.cp["b"] == pytest.approx((30.0 - 5.0) / 0.5)


def test_equal_chord_positions_keep_their_tap_order():
    f = _field({"x": ("m", 0.3, 0.0, S, -0.5), "y": ("m", 0.3, 0.0, S, -2.0), "z": ("m", 0.1, 0.0, S, -1.0)})
    assert f.chordwise("m", S)[2] == ["z", "x", "y"]


def test_coverage_edges_and_an_exact_coverage_floor():
    single = {"s1": ("m", 0.4, 0, S, -1.0), "p1": ("m", 0.4, 0, P, 0.5)}
    assert _field(single).chord_coverage("m") == 0.0
    assert math.isnan(_field(single).normal_load_coefficient("m"))
    one_side = {"s1": ("m", 0.0, 0, S, -1.0), "s2": ("m", 1.0, 0, S, -1.0)}
    assert _field(one_side).chord_coverage("m") == 0.0 and math.isnan(_field(one_side).normal_load_coefficient("m"))
    f = _field(WING)
    cov = f.chord_coverage("main")
    assert f.normal_load_coefficient("main", min_chord_coverage=cov) == pytest.approx(
        f.normal_load_coefficient("main"))
    span = {"s1": ("m", 0.0, 0.0, S, -1.0), "s2": ("m", 1.0, 0.0, S, -1.0), "p1": ("m", 0.0, 0.0, P, 0.5),
            "p2": ("m", 1.0, 0.0, P, 0.5), "s3": ("m", 0.2, 0.5, S, -2.0), "s4": ("m", 0.8, 0.5, S, -2.0),
            "p3": ("m", 0.4, 0.5, P, 1.0), "p4": ("m", 0.6, 0.5, P, 1.0), "s5": ("m", 0.5, 0.9, S, -1.0)}
    g = _field(span)                                       # median station 0.5: overlap 0.4..0.6 of 0.2..0.8
    assert g.chord_coverage("m", span_tol_m=0.1) == pytest.approx(1 / 3)
    assert g.normal_load_coefficient("m", span_tol_m=0.1) == pytest.approx(0.2 * 3.0)
    assert math.isnan(g.normal_load_coefficient("m", span_tol_m=0.1, min_chord_coverage=0.5))
    assert g.normal_load_coefficient("m", span_tol_m=0.1, min_chord_coverage=0.3) == pytest.approx(0.6)


def test_the_stall_call_at_its_length_edges():
    three = {"a": ("m", 0.1, 0, S, -1.0), "b": ("m", 0.5, 0, S, -2.0), "c": ("m", 0.9, 0, S, -1.6)}
    v = _field(three).stall_indicator("m")
    assert v.element == "m" and v.recovery_slope == pytest.approx(1.0) and v.stalled is False
    few = _field({"a": ("m", 0.1, 0, S, -1.0), "b": ("m", 0.5, 0, S, -2.0)}).stall_indicator("m")
    assert few.element == "m" and few.note.startswith("too few valid suction taps")
    tail = _field({"a": ("m", 0.1, 0, S, -1.0), "b": ("m", 0.5, 0, S, -1.5), "c": ("m", 0.9, 0, S, -2.0)})
    t = tail.stall_indicator("m")
    assert t.element == "m" and t.note == "suction peak is at the trailing edge — no aft chord to recover"


def _five(cfd_last):
    phys = _phys({k: 0.0 for k in "abcde"})
    return pt.correlate_cp(phys, pt.CFDSurfaceCp({"a": 0.0, "b": 0.0, "c": 0.0, "d": 0.0, "e": cfd_last}))


def test_the_correlation_edges():
    r = _five(0.2)
    assert r.max_abs_residual == 0.2 and r.within_tol is True
    assert r.as_dict()["tolerances"] == pt.DEFAULT_CP_TOL
    two = pt.correlate_cp(_phys({"a": 0.0, "b": 0.0}), pt.CFDSurfaceCp({"a": 0.1, "b": 0.3}))
    assert two.bias == pytest.approx(0.2)
    taps = [pt.TapLocation("a"), pt.TapLocation("b"), pt.TapLocation("c")]
    phys = pt.CpField(taps, {"a": math.nan, "b": 0.5})
    r = pt.correlate_cp(phys, pt.CFDSurfaceCp({"b": 0.4, "c": 1.0}))
    by = {x.tap.tap_id: x for x in r.residuals}
    assert by["a"].cp_phys is None and by["a"].note == "no CFD C_p at this tap — not paired"
    assert by["c"].cp_phys is None and by["c"].cp_cfd == 1.0 and by["c"].note == (
        "measured tap is a hole (uncalibrated/railed) — not paired")
    assert by["b"].paired is True and r.n_paired == 1
    ph = _phys({"a": 0.0, "b": 0.1, "c": 0.0, "d": 0.0})
    r = pt.correlate_cp(ph, pt.CFDSurfaceCp({"a": 0.0, "b": 0.5, "c": 0.0}), tol={"min_coverage": 0.8})
    assert "  Coverage 75% is below the 80% floor — the RMSE is over too few taps to trust; instrument or sample more " \
           "of the wing." in r.summary
    assert ("MISMATCHED — the CFD pressure distribution drifts from the measured C_p beyond tolerance; the "
            "integrated force may still agree while the wing is loaded differently, so do NOT trust CFD surface "
            "detail here.") in r.summary


def test_more_reduction_edges():
    zero = pt.ScanProvenance("T", p_static_inf_pa=10.0, p_total_inf_pa=10.0)
    assert zero.dynamic_pressure() == pytest.approx(0.5 * 1.225 * 400)
    f = _field({"n": ("m", 0.05, 0.0, S, math.nan), "a": ("m", 0.1, 0.0, S, -1.0), "b": ("m", 0.6, 0.0, S, -0.5)})
    assert f.chordwise("m", S)[2] == ["a", "b"]
    two = {"s1": ("m", 0.1, 0, S, -1.0), "s2": ("m", 0.9, 0, S, -0.5), "p1": ("m", 0.2, 0, P, 0.4),
           "p2": ("m", 0.8, 0, P, 0.2)}
    g = _field(two)
    assert g.chord_coverage("m") == pytest.approx(0.6 / 0.8)
    s_at = lambda x: np.interp(x, [0.1, 0.9], [-1.0, -0.5])
    want = 0.5 * ((0.4 - s_at(0.2)) + (0.2 - s_at(0.8))) * 0.6
    assert g.normal_load_coefficient("m") == pytest.approx(want, rel=1e-12)
    tall = {"a": ("m", 0.1, 0.0, S, -1.0), "b": ("m", 0.2, 1.0, S, -5.0), "c": ("m", 0.3, 0.0, S, -2.0),
            "d": ("m", 0.4, 0.0, S, -0.5), "e": ("m", 0.5, -1.0, S, -0.1)}
    assert _field(tall).suction_peak("m") == (-5.0, 0.2)
    assert _field(tall).suction_peak("m", span_tol_m=0.1) == (-2.0, 0.3)
