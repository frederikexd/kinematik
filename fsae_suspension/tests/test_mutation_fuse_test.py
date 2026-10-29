"""Characterization suite for suspension/fuse_test.py (mutation gate); see tests/_golden.py."""
import math

import pytest

from suspension import fuse_test as ft
from tests._golden import check

A = ft.CurveAnchor


def _fuses():
    return {
        "parallel": ft.FuseSpec(label="p", rating_a=10.0, fuse_class=ft.FuseClass.FAST,
                                anchors=[A(2.0, 1.0), A(4.0, 0.25), A(8.0, 0.0625)]),
        "steep": ft.FuseSpec(label="s", rating_a=15.0, anchors=[A(2.0, 5.0, "a"), A(5.0, 0.05), A(10.0, 0.002)],
                             part_number="ATO-15", source="datasheet", is_estimate=True),
        "shallow": ft.FuseSpec(label="sh", rating_a=20.0, anchors=[A(1.5, 30.0), A(3.0, 10.0), A(6.0, 3.0)],
                               continuous_derate=0.8),
        "flat": ft.FuseSpec(rating_a=10.0, anchors=[A(3.0, 1.0), A(3.0, 2.0)]),
        "one": ft.FuseSpec(rating_a=10.0, anchors=[A(2.0, 1.0)]),
        "norating": ft.FuseSpec(anchors=[A(2.0, 1.0), A(4.0, 0.1)]),
        "zero_rating": ft.FuseSpec(rating_a=0.0, anchors=[A(2.0, 1.0), A(4.0, 0.1)]),
        "bare": ft.FuseSpec(),
    }


def _wires():
    return {"awg16": ft.WireSpec(), "awg10_al": ft.WireSpec(label="al", awg=10, material="aluminium",
                                                            insulation="PVC 70"),
            "area": ft.WireSpec(label="a", awg=None, area_mm2=0.5, insulation="unknown",
                                t_initial_c=40.0, t_final_c=200.0),
            "thin": ft.WireSpec(label="t", awg=24, insulation="PTFE 200"),
            "fat": ft.WireSpec(label="f", awg=0, t_final_c=300.0)}


def test_wire_and_material():
    res = {"awg": [ft.awg_area_mm2(a) for a in (0, 10, 24, 11, 26, -2)]}
    res["k"] = [ft.adiabatic_k(m, ti, tf) for m in ("copper", "aluminium") for ti, tf in ((70, 160), (20, 250))]
    for args, exc, msg in ((("tin", 20, 100), KeyError, "unknown conductor material 'tin'"),
                           (("copper", 100, 100), ValueError, "short-circuit temperature must exceed"),
                           (("copper", 100, 90), ValueError, "short-circuit temperature must exceed")):
        with pytest.raises(exc) as e:
            ft.adiabatic_k(*args)
        res.setdefault("err", []).append(str(e.value))
    for k, w in _wires().items():
        res[k] = [w.area(), w.temperatures(), w.k(), w.i2t_withstand(), w.withstand_time_s(0.0),
                  w.withstand_time_s(-1.0), w.withstand_time_s(100.0), w.damage_current_a(0.0),
                  w.damage_current_a(-1.0), w.damage_current_a(0.1)]
    with pytest.raises(ValueError) as e:
        ft.WireSpec(label="x", awg=None).area()
    res["noarea"] = str(e.value)
    res["tw"] = [ft.WireSpec(t_initial_c=50.0).temperatures(), ft.WireSpec(t_final_c=140.0).temperatures()]
    check("fuse_wire", res)


def test_fuse_curves_and_coordination():
    res = {}
    fuses, wires = _fuses(), _wires()
    for k, f in fuses.items():
        res[k] = {"has": f.has_curve(), "pl": f.power_law(), "rng": f.anchor_range(),
                  "bt": [f.blow_time_s(i) for i in (0.0, -5.0, 12.0, 30.0, 60.0, 200.0, 2000.0)],
                  "ex": [f.extrapolated(i) for i in (10.0, 20.0, 40.0, 80.0, 81.0, 300.0)],
                  "ci": [f.clearing_i2t(i) for i in (0.0, 40.0, 500.0)], "cl": f.continuous_limit_a()}
    for fk, f in fuses.items():
        for wk, w in wires.items():
            for fault in (None, 0.0, 30.0, 400.0, 5000.0):
                res[f"c_{fk}_{wk}_{fault}"] = ft.coordinate(f, w, prospective_fault_a=fault)
    huge = ft.FuseSpec(rating_a=10.0, anchors=[A(1.0, 1e300), A(1.0001, 1e-300)])
    res["overflow"] = ft.coordinate(huge, wires["awg16"])
    near2 = ft.FuseSpec(rating_a=10.0, anchors=[A(1.0, 1.0), A(2.0, 0.25 * (1 + 1e-9))])
    res["near2"] = ft.coordinate(near2, wires["awg16"])
    check("fuse_coord", res)


def test_nuisance_and_instrument():
    res = {}
    f = _fuses()
    for k in ("parallel", "shallow", "bare", "one"):
        for load in (None, 0.0, 6.0, 7.2, 7.5, 7.6, 16.0):
            for inrush in ((None, None), (50.0, None), (None, 10.0), (50.0, 10.0), (40.0, 600.0), (1e4, 1.0)):
                res[f"n_{k}_{load}_{inrush}"] = ft.nuisance_check(f[k], continuous_load_a=load, inrush_a=inrush[0],
                                                                  inrush_ms=inrush[1], ambient_c=40.0)
    insts = {"poc": ft.Instrument(), "rig": ft.instrumented_rig(), "rig2": ft.instrumented_rig(1000.0),
             "baud0": ft.Instrument(serial_baud=0), "noserial": ft.Instrument(serial_in_timing_path=False),
             "adc": ft.Instrument(adc_sample_rate_hz=500.0, serial_chars_in_path=10, serial_baud=19200)}
    for k, i in insts.items():
        res[k] = [i, i.serial_blocking_s(), i.sampling_latency_s(), i.total_latency_s(), i.uncertainty_s(),
                  i.resolvable_time_s(), i.resolvable_time_s(0.05), i.resolvable_time_s(0.0),
                  i.required_sample_rate_hz(0.01), i.required_sample_rate_hz(0.0),
                  i.required_sample_rate_hz(-1.0), i.required_sample_rate_hz(0.2, 0.1)]
        u = i.uncertainty_s()
        res[k + "_f"] = ft.instrument_findings(i, [0.0, u * 1.5, u * 3.0, u * 4.0, u * 6.0, u * 8.0,
                                                   u * 12.0, u / 0.1, u / 0.2, u / 0.5])
    check("fuse_instrument", res)


def _meas(spec):
    return [ft.Measurement(c, list(ts)) for c, ts in spec]


def test_scatter_and_sampling():
    res = {"chi2": [ft._chi2_lower_quantile(d, p) for d in (-1, 0, 1, 2, 5, 30) for p in (0.01, 0.05, 0.1, 1 - 0.9)]}
    for bad in (0.2, 0.5, -0.95, 1.05):
        with pytest.raises(ValueError) as e:
            ft._chi2_lower_quantile(5, bad)
        res.setdefault("chi2_err", []).append(str(e.value))
    res["chi2_d"] = ft._chi2_lower_quantile(12)
    ests = {"prior": ft.prior_scatter(), "prior2": ft.prior_scatter(0.5),
            "m0": ft.ScatterEstimate(0.2, dof=0, measured=True),
            "m1": ft.ScatterEstimate(0.2, dof=1, measured=True), "m9": ft.ScatterEstimate(0.25, dof=9, measured=True),
            "um": ft.ScatterEstimate(0.2, dof=9, measured=False)}
    for k, e in ests.items():
        res[k] = [e.as_dict(), e.rel_uncertainty(), e.upper_bound(), e.upper_bound(0.99), e.upper_bound(0.9),
                  ft._as_scatter(e) is e]
    res["as"] = [ft._as_scatter(None), ft._as_scatter(0.4), ft._as_scatter("0.1")]
    groups = {"none": [], "thin": _meas([(30.0, [0.1]), (40.0, [0.0, -1.0, 0.2])]),
              "two": _meas([(30.0, [0.1, 0.12]), (40.0, [0.05, 0.06, 0.0, 0.07])]),
              "many": _meas([(30.0, [0.1, 0.12, 0.09, 0.11, 0.13]), (40.0, [0.05, 0.06, 0.055, 0.07, 0.05])]),
              "wide": _meas([(30.0, [0.1, 0.3, 0.05, 0.4, 0.08]), (40.0, [0.05, 0.2, 0.01, 0.1, 0.03])]),
              "mid": _meas([(30.0, [0.1 * math.exp(0.3 * z) for z in (-1.26, -0.64, 0.0, 0.64, 1.26)]),
                            (40.0, [0.05 * math.exp(0.3 * z) for z in (-1.26, -0.64, 0.0, 0.64, 1.26)])]),
              "near": _meas([(30.0, [0.1, 0.13, 0.08, 0.12, 0.09]), (40.0, [0.05, 0.065, 0.04, 0.06, 0.045])]),
              "tight": _meas([(30.0, [0.100, 0.101, 0.099, 0.1005, 0.0995])] * 2)}
    for k, g in groups.items():
        res["pool_" + k] = ft.pooled_log_scatter(g)
        res["ref_" + k] = ft.refine_from_pilot(g)
        res["ref2_" + k] = ft.refine_from_pilot(g, rel_precision=0.1, confidence_z=2.5, pilot_n=4)
    res["sn"] = [ft.samples_needed(), ft.samples_needed(0.05), ft.samples_needed(1.0, 0.01),
                 ft.samples_needed(0.2, ests["m9"]), ft.samples_needed(0.2, ests["m9"], use_upper_bound=False),
                 ft.samples_needed(0.2, ests["m9"], 2.5, scatter_confidence=0.9), ft.samples_needed(0.2, ests["um"]),
                 ft.samples_needed(0.1, 0.3, 1.0)]
    for bad in (0.0, -0.1):
        with pytest.raises(ValueError, match="^rel_precision must be > 0"):
            ft.samples_needed(bad)
        with pytest.raises(ValueError, match="^target_rel_uncertainty must be > 0"):
            ft.recommended_pilot_n(bad)
    res["pilot"] = [ft.recommended_pilot_n(), ft.recommended_pilot_n(0.1), ft.recommended_pilot_n(1.0),
                    ft.recommended_pilot_n(0.5)]
    res["consts"] = [ft.PRIOR_LOG_SCATTER, ft.DEFAULT_LOG_SCATTER, ft.MIN_USEFUL_DOF, ft._PROBE_MULTS]
    check("fuse_scatter", res)


def test_plans_fits_and_sketch():
    res = {}
    f = _fuses()
    rig, poc = ft.instrumented_rig(), ft.Instrument()
    est = ft.ScatterEstimate(0.2, dof=10, measured=True, n_levels=2, source="m")
    plans = {"steep_rig": ft.build_test_plan(f["steep"], rig),
             "steep_poc": ft.build_test_plan(f["steep"], poc, multipliers=(2.0, 5.0, 10.0, 20.0)),
             "bare": ft.build_test_plan(f["bare"], rig, multipliers=(1.0, 2.0)),
             "measured": ft.build_test_plan(f["shallow"], rig, rel_precision=0.1, unit_cost=2.0, log_scatter=est),
             "scalar": ft.build_test_plan(f["parallel"], poc, log_scatter=0.5),
             "empty": ft.build_test_plan(f["parallel"], rig, multipliers=())}
    for k, p in plans.items():
        res["plan_" + k] = [p, p.measurable_points(), p.sized_on_measurement()]
    res["sized_none"] = ft.TestPlan(f["bare"], rig, [], 0, 0.0).sized_on_measurement()
    res["sketch"] = {k: ft.emit_arduino_sketch(p) for k, p in plans.items()}
    res["sketch_kw"] = [ft.emit_arduino_sketch(plans["steep_rig"], shunt_mohm=0.5, adc_ref_v=3.3, adc_bits=12,
                                               gain=20.0, trip_fraction=0.3),
                        ft.emit_arduino_sketch(plans["steep_rig"], shunt_mohm=10.0, gain=100.0),
                        ft.emit_arduino_sketch(plans["bare"], trip_fraction=0.0)]
    res["log_stats"] = [ft._log_stats(x) for x in ([], [0.0, -1.0], [2.0], [1.0, 4.0], [0.0, 1.0, 4.0, 16.0])]
    s = f["steep"]
    bt = s.blow_time_s

    def at(c, k=1.0, n=5, spread=0.02):
        return ft.Measurement(c, [bt(c) * k * (1 + spread * (j - n // 2)) for j in range(n)])

    fits = {"in": [at(40.0), at(60.0), at(100.0)],
            "slow": [at(40.0, 2.5), at(60.0, 2.6), at(100.0, 1.1)],
            "fast": [at(40.0, 0.3), at(60.0, 0.35, n=2), at(100.0)],
            "rejected": [at(40.0), at(140.0), at(149.0)],
            "one": [at(40.0)], "same": [at(40.0), at(40.0, 1.1)],
            "nan": [ft.Measurement(40.0, [0.0, -1.0]), at(60.0), at(100.0)],
            "scatter": [ft.Measurement(40.0, [0.5, 3.0, 0.2, 2.0]), at(60.0, n=3, spread=0.5), at(100.0)]}
    for k, ms in fits.items():
        res["fit_" + k] = ft.fit_measurements(ms, s, rig)
    res["fit_tol"] = ft.fit_measurements(fits["slow"], s, rig, tolerance=2.0)
    res["fit_poc"] = ft.fit_measurements(fits["in"], s, poc)
    res["fit_norating"] = ft.fit_measurements(fits["in"], f["bare"], rig)
    res["fit_nocurve"] = ft.fit_measurements([ft.Measurement(40.0, [0.1, 0.11, 0.12]),
                                              ft.Measurement(60.0, [0.05, 0.051, 0.052])],
                                             ft.FuseSpec(rating_a=15.0), rig)
    check("fuse_plan", res)


def _spread(c, med, sigma, n):
    zs = [(-1.0 + 2.0 * j / (n - 1)) * 1.2 for j in range(n)] if n > 1 else [0.0]
    return ft.Measurement(c, [med * math.exp(sigma * z) for z in zs])


def test_edges():
    """Inputs placed on the far side of each threshold and in the display ranges where a scale error shows."""
    res = {}
    w = ft.WireSpec()
    res["wire_odd_ins"] = ft.WireSpec(insulation="unknown").temperatures()
    res["small_i"] = [w.withstand_time_s(0.5), _fuses()["steep"].blow_time_s(0.5),
                      ft.Instrument(serial_baud=1).serial_blocking_s()]
    b25 = ft.FuseSpec(label="b25", rating_a=10.0, anchors=[A(2.0, 1.0), A(4.0, 2.0 ** -2.5)])
    cooks = ft.FuseSpec(label="slowpar", rating_a=10.0, anchors=[A(2.0, 400.0), A(4.0, 100.0)])
    res["b25"] = ft.coordinate(b25, w, prospective_fault_a=200.0)
    res["cooks"] = ft.coordinate(cooks, w)
    steep = _fuses()["steep"]
    pick = None
    for i in range(1, 4000):
        cur = 15.0 * (1.0 + i * 0.01)
        r = w.withstand_time_s(cur) / steep.blow_time_s(cur)
        if 2.0 < r < 3.0:
            pick = cur
            break
    assert pick is not None
    res["fault_2_3"] = ft.coordinate(steep, w, prospective_fault_a=pick)
    rig = ft.instrumented_rig()
    empty = ft.build_test_plan(ft.FuseSpec(), rig, multipliers=())
    res["sketch_empty"] = ft.emit_arduino_sketch(empty)
    res["sketch_scale"] = ft.emit_arduino_sketch(ft.build_test_plan(steep, rig), shunt_mohm=0.3, gain=7.0)
    res["plan_pct"] = ft.build_test_plan(steep, rig, rel_precision=0.137).findings
    bt = steep.blow_time_s
    fits = {"k2": [_spread(c, bt(c) * 2.0, 0.02, 5) for c in (40.0, 60.0, 100.0)],
            "half": [_spread(c, bt(c) * k, 0.02, 5) for c, k in ((40.0, 0.5), (60.0, 1.0), (100.0, 1.0))]}
    res["fit"] = {k: ft.fit_measurements(v, steep, rig) for k, v in fits.items()}
    insts = {"coarse": ft.Instrument(timer_resolution_s=0.5), "slow_serial": ft.Instrument(serial_chars_in_path=300,
                                                                                           serial_baud=1200)}
    res["inst"] = {k: ft.instrument_findings(i, [0.5, 5.0]) for k, i in insts.items()}
    sh = _fuses()["shallow"]
    i_in = 60.0
    t_in = sh.blow_time_s(i_in)
    res["inrush"] = [ft.nuisance_check(sh, continuous_load_a=5.0, inrush_a=i_in, inrush_ms=t_in * 1000.0 * 2.5),
                     ft.nuisance_check(sh, continuous_load_a=5.0, inrush_a=i_in, inrush_ms=t_in * 1000.0 * (1 + 5e-4)),
                     ft.nuisance_check(sh, continuous_load_a=5.0, inrush_a=i_in, inrush_ms=t_in * 1000.0 * (1 - 5e-4))]
    groups = {"thin_then_ok": [ft.Measurement(30.0, [0.1]), ft.Measurement(40.0, [0.05, 0.06, 0.055])],
              "dof1": [ft.Measurement(30.0, [0.1, 0.12])],
              "dof4": [_spread(30.0, 0.1, 0.3, 3), _spread(40.0, 0.05, 0.3, 3)],
              "dof10": [_spread(30.0, 0.1, 0.3, 6), _spread(40.0, 0.05, 0.3, 6)],
              "dof12": [_spread(30.0, 0.1, 0.3, 7), _spread(40.0, 0.05, 0.3, 7)],
              "sigma_half": [_spread(30.0, 0.1, 0.5, 6), _spread(40.0, 0.05, 0.5, 6)]}
    for k, g in groups.items():
        res["pool_" + k] = ft.pooled_log_scatter(g)
        res["ref_" + k] = ft.refine_from_pilot(g)
        res["ref50_" + k] = ft.refine_from_pilot(g, pilot_n=50)
    check("fuse_edges", res)
