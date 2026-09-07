# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Tests for suspension.fuse_test — the time axis of fuse protection.

These pin the adiabatic k derivation to the published IEC table, check that the
short-circuit region is energy-limited rather than power-law extrapolated (the
error that makes protection look faster than it is), and verify that the module
refuses: no curve means no coordination, and a reading the rig cannot resolve
is rejected from the fit rather than averaged into it.
"""

import math

import pytest

from suspension.interfaces import Severity
from suspension import fuse_test as ft


def _sev(findings, check):
    for f in findings:
        if f.check == check:
            return f.severity
    return None


def _checks(findings):
    return {f.check for f in findings}


def _blade(rating=15.0, anchors=((2.0, 5.0), (10.0, 0.01)), **kw):
    return ft.FuseSpec(label=f"{rating:.0f} A blade", rating_a=rating,
                       fuse_class=ft.FuseClass.BLADE_ATO,
                       anchors=[ft.CurveAnchor(m, t) for m, t in anchors],
                       **kw)


# ===================================================================== #
#  1.  ADIABATIC WIRE WITHSTAND
# ===================================================================== #
class TestAdiabaticK:
    @pytest.mark.parametrize("mat,ti,tf,tabled", [
        ("copper",    70.0, 160.0, 115.0),
        ("copper",    90.0, 160.0, 100.0),
        ("copper",    90.0, 250.0, 143.0),
        ("copper",    60.0, 200.0, 141.0),
        ("aluminium", 70.0, 160.0,  76.0),
    ])
    def test_derivation_reproduces_the_published_iec_table(self, mat, ti, tf,
                                                           tabled):
        """Five independent anchors from IEC 60364-5-54 table 43.1."""
        assert ft.adiabatic_k(mat, ti, tf) == pytest.approx(tabled, abs=0.5)

    def test_a_hotter_permitted_final_temperature_raises_k(self):
        assert (ft.adiabatic_k("copper", 90.0, 250.0)
                > ft.adiabatic_k("copper", 90.0, 160.0))

    def test_starting_hotter_lowers_k(self):
        """A wire already at temperature has less headroom for the fault."""
        assert (ft.adiabatic_k("copper", 90.0, 160.0)
                < ft.adiabatic_k("copper", 70.0, 160.0))

    def test_copper_outperforms_aluminium_at_equal_area(self):
        assert (ft.adiabatic_k("copper", 70.0, 160.0)
                > ft.adiabatic_k("aluminium", 70.0, 160.0))

    def test_inverted_temperatures_are_rejected(self):
        with pytest.raises(ValueError):
            ft.adiabatic_k("copper", 160.0, 70.0)

    def test_unknown_material_raises(self):
        with pytest.raises(KeyError):
            ft.adiabatic_k("unobtainium", 70.0, 160.0)


class TestWireSpec:
    def test_withstand_scales_with_area_squared(self):
        a = ft.WireSpec(awg=None, area_mm2=1.0)
        b = ft.WireSpec(awg=None, area_mm2=2.0)
        assert b.i2t_withstand() == pytest.approx(4 * a.i2t_withstand())

    def test_withstand_time_is_i2t_over_current_squared(self):
        w = ft.WireSpec(awg=16)
        assert w.withstand_time_s(100.0) == pytest.approx(
            w.i2t_withstand() / 10_000.0)

    def test_damage_current_inverts_withstand_time(self):
        w = ft.WireSpec(awg=14)
        i = w.damage_current_a(0.05)
        assert w.withstand_time_s(i) == pytest.approx(0.05)

    def test_zero_current_never_damages_the_wire(self):
        assert ft.WireSpec(awg=16).withstand_time_s(0.0) == math.inf

    def test_awg_area_halves_every_three_gauges(self):
        assert ft.awg_area_mm2(13) == pytest.approx(
            ft.awg_area_mm2(10) / 2.0, rel=0.05)

    def test_area_overrides_awg_when_both_given(self):
        w = ft.WireSpec(awg=16, area_mm2=6.0)
        assert w.area() == 6.0

    def test_a_wire_with_neither_area_nor_gauge_refuses(self):
        with pytest.raises(ValueError):
            ft.WireSpec(awg=None, area_mm2=None).area()

    def test_automotive_insulation_gets_a_derived_k_not_a_default(self):
        txl = ft.WireSpec(awg=16, insulation="TXL / GXL 125")
        pvc = ft.WireSpec(awg=16, insulation="PVC 70")
        assert txl.k() != pytest.approx(pvc.k())
        assert 100.0 < txl.k() < 160.0


# ===================================================================== #
#  2.  FUSE CURVE
# ===================================================================== #
class TestFuseCurve:
    def test_two_anchors_fit_exactly(self):
        f = _blade(anchors=((2.0, 5.0), (10.0, 0.01)))
        assert f.blow_time_s(2 * 15.0) == pytest.approx(5.0)
        assert f.blow_time_s(10 * 15.0) == pytest.approx(0.01)

    def test_no_anchors_means_no_curve(self):
        f = ft.FuseSpec(rating_a=15.0)
        assert not f.has_curve()
        assert f.blow_time_s(100.0) is None

    def test_one_anchor_is_not_a_curve(self):
        f = ft.FuseSpec(rating_a=15.0, anchors=[ft.CurveAnchor(2.0, 5.0)])
        assert not f.has_curve()

    def test_blow_time_falls_as_current_rises(self):
        f = _blade()
        ts = [f.blow_time_s(15.0 * m) for m in (2, 3, 5, 10, 20)]
        assert ts == sorted(ts, reverse=True)

    def test_above_the_top_anchor_the_model_is_energy_limited(self):
        """The correction that stops protection looking faster than it is."""
        f = _blade(anchors=((2.0, 5.0), (10.0, 0.01)))
        ir = 15.0
        top_i = 10.0 * ir
        i2t_top = top_i ** 2 * f.blow_time_s(top_i)
        for m in (20.0, 40.0, 100.0):
            i = m * ir
            assert i ** 2 * f.blow_time_s(i) == pytest.approx(i2t_top)

    def test_energy_limit_is_slower_than_raw_power_law_extrapolation(self):
        f = _blade(anchors=((2.0, 5.0), (10.0, 0.01)))
        a, b = f.power_law()
        assert b > 2.0, "this blade fit should be steeper than I^-2"
        raw = a * (40.0 ** -b)
        assert f.blow_time_s(40 * 15.0) > raw

    def test_the_continuation_is_continuous_at_the_anchor(self):
        f = _blade(anchors=((2.0, 5.0), (10.0, 0.01)))
        eps = 1e-6
        below = f.blow_time_s(10.0 * 15.0 - eps)
        above = f.blow_time_s(10.0 * 15.0 + eps)
        assert below == pytest.approx(above, rel=1e-4)

    def test_extrapolation_is_reported(self):
        f = _blade(anchors=((2.0, 5.0), (10.0, 0.01)))
        assert not f.extrapolated(5 * 15.0)
        assert f.extrapolated(40 * 15.0)
        assert f.extrapolated(1.1 * 15.0)

    def test_more_than_two_anchors_are_least_squares_fitted(self):
        f = _blade(anchors=((2.0, 5.0), (5.0, 0.2), (10.0, 0.01)))
        a, b = f.power_law()
        assert b > 0

    def test_continuous_limit_applies_the_derate(self):
        f = _blade(rating=20.0, continuous_derate=0.75)
        assert f.continuous_limit_a() == pytest.approx(15.0)


# ===================================================================== #
#  3.  COORDINATION
# ===================================================================== #
class TestCoordination:
    def test_no_declared_curve_refuses_rather_than_estimating(self):
        r = ft.coordinate(ft.FuseSpec(rating_a=15.0), ft.WireSpec(awg=16))
        assert r.protected is None
        assert r.crossover_a is None
        assert _sev(r.findings, "fuse-curve-undeclared") == Severity.MISSING

    def test_a_fuse_far_smaller_than_its_wire_protects_it(self):
        r = ft.coordinate(_blade(rating=15.0), ft.WireSpec(awg=10))
        assert _sev(r.findings, "short-circuit-energy") == Severity.OK

    def test_a_wire_far_too_thin_for_its_fuse_is_a_fail(self):
        """A 60 A fuse on 22 AWG: the wire is the fuse."""
        r = ft.coordinate(_blade(rating=60.0), ft.WireSpec(awg=22))
        assert _sev(r.findings, "short-circuit-energy") == Severity.FAIL

    def test_the_short_circuit_verdict_is_one_energy_comparison(self):
        r = ft.coordinate(_blade(rating=15.0), ft.WireSpec(awg=16))
        d = next(f.detail for f in r.findings
                 if f.check == "short-circuit-energy")
        assert d["i2t_fuse_sc"] < d["i2t_wire"]

    def test_a_pure_i2t_fuse_never_crosses_the_wire_curve(self):
        """b == 2 makes both curves parallel — no crossover exists."""
        f = _blade(anchors=((2.0, 4.0), (4.0, 1.0)))      # t ~ I^-2 exactly
        a, b = f.power_law()
        assert b == pytest.approx(2.0)
        r = ft.coordinate(f, ft.WireSpec(awg=14))
        assert r.parallel_curves
        assert r.crossover_a is None
        assert "coordination-parallel" in _checks(r.findings)

    def test_a_cleared_fault_reports_its_margin(self):
        r = ft.coordinate(_blade(rating=15.0), ft.WireSpec(awg=12),
                          prospective_fault_a=600.0)
        assert _sev(r.findings, "fault-cleared") == Severity.OK

    def test_an_uncleared_fault_is_a_fail(self):
        r = ft.coordinate(_blade(rating=80.0), ft.WireSpec(awg=22),
                          prospective_fault_a=900.0)
        assert _sev(r.findings, "fault-not-cleared") == Severity.FAIL

    def test_a_crossover_outside_the_anchor_range_is_flagged(self):
        r = ft.coordinate(_blade(rating=15.0), ft.WireSpec(awg=16))
        assert _sev(r.findings, "crossover-extrapolated") == Severity.WARN

    def test_the_margin_table_covers_overload_through_short_circuit(self):
        r = ft.coordinate(_blade(rating=15.0), ft.WireSpec(awg=16))
        assert len(r.margin_at) == len(ft._PROBE_MULTS)
        for _i, (tf, tw, ratio) in r.margin_at.items():
            assert tf > 0 and tw > 0
            assert ratio == pytest.approx(tw / tf)


class TestNuisance:
    def test_load_above_the_derated_limit_will_trip(self):
        fs = ft.nuisance_check(_blade(rating=10.0), continuous_load_a=9.0)
        assert _sev(fs, "nuisance-trip-likely") == Severity.FAIL

    def test_comfortable_load_passes(self):
        fs = ft.nuisance_check(_blade(rating=20.0), continuous_load_a=5.0)
        assert _sev(fs, "continuous-ok") == Severity.OK

    def test_undeclared_load_is_missing_not_pass(self):
        fs = ft.nuisance_check(_blade(), continuous_load_a=None)
        assert _sev(fs, "load-undeclared") == Severity.MISSING

    def test_inrush_that_exceeds_the_curve_opens_the_fuse(self):
        fs = ft.nuisance_check(_blade(rating=10.0), continuous_load_a=3.0,
                               inrush_a=200.0, inrush_ms=50.0)
        assert _sev(fs, "inrush-opens-fuse") == Severity.FAIL

    def test_brief_inrush_inside_the_curve_survives(self):
        fs = ft.nuisance_check(_blade(rating=30.0), continuous_load_a=5.0,
                               inrush_a=60.0, inrush_ms=5.0)
        assert _sev(fs, "inrush-survived") == Severity.OK


# ===================================================================== #
#  4.  THE INSTRUMENT
# ===================================================================== #
class TestInstrument:
    def test_the_proof_of_concept_rig_is_dominated_by_the_operator(self):
        inst = ft.Instrument()
        assert inst.detection_latency_s >= 0.2
        assert inst.resolvable_time_s(0.10) > 0.5

    def test_serial_blocking_follows_8n1_byte_framing(self):
        inst = ft.Instrument(serial_baud=9600, serial_chars_in_path=30,
                             serial_in_timing_path=True)
        assert inst.serial_blocking_s() == pytest.approx(30 * 10 / 9600)

    def test_taking_serial_out_of_the_timing_path_removes_it(self):
        assert ft.Instrument(serial_in_timing_path=False).serial_blocking_s() == 0.0

    def test_the_instrumented_rig_resolves_far_shorter_events(self):
        poc, good = ft.Instrument(), ft.instrumented_rig()
        assert good.resolvable_time_s(0.10) < poc.resolvable_time_s(0.10) / 100

    def test_manual_detection_is_a_hard_fail(self):
        fs = ft.instrument_findings(ft.Instrument(), [0.05])
        assert _sev(fs, "manual-detection") == Severity.FAIL

    def test_serial_in_the_timing_path_is_warned(self):
        fs = ft.instrument_findings(ft.Instrument(), [5.0])
        assert _sev(fs, "serial-in-timing-path") == Severity.WARN

    def test_an_event_below_resolution_is_rejected_as_unmeasurable(self):
        fs = ft.instrument_findings(ft.Instrument(), [0.01])
        assert _sev(fs, "test-point-unmeasurable") == Severity.FAIL

    def test_a_slow_event_is_measurable_even_on_the_poor_rig(self):
        fs = ft.instrument_findings(ft.Instrument(), [30.0])
        assert "test-point-unmeasurable" not in _checks(fs)

    def test_required_sample_rate_rises_as_the_event_shortens(self):
        inst = ft.instrumented_rig()
        assert (inst.required_sample_rate_hz(0.001)
                > inst.required_sample_rate_hz(0.100))

    def test_quantisation_uses_the_uniform_distribution(self):
        inst = ft.Instrument(timer_resolution_s=1e-3, latency_jitter_s=0.0)
        assert inst.uncertainty_s() == pytest.approx(1e-3 / math.sqrt(12))


# ===================================================================== #
#  5.  THE TEST PLAN
# ===================================================================== #
class TestPlanning:
    def test_more_precision_demands_more_fuses(self):
        assert ft.samples_needed(0.10) > ft.samples_needed(0.30)

    def test_wider_scatter_demands_more_fuses(self):
        assert (ft.samples_needed(0.2, log_scatter=0.5)
                > ft.samples_needed(0.2, log_scatter=0.2))

    def test_a_bare_float_is_accepted_but_marked_unverified(self):
        est = ft._as_scatter(0.4)
        assert est.sigma_ln == 0.4
        assert not est.measured

    def test_never_fewer_than_three_samples(self):
        assert ft.samples_needed(5.0, log_scatter=0.01) >= 3

    def test_zero_precision_is_rejected(self):
        with pytest.raises(ValueError):
            ft.samples_needed(0.0)

    def test_the_plan_counts_the_fuses_it_will_destroy(self):
        p = ft.build_test_plan(_blade(), ft.instrumented_rig())
        assert p.fuses_consumed == sum(pt.samples for pt in p.points)
        assert p.estimated_cost > 0
        assert _sev(p.findings, "plan-size") == Severity.INFO

    def test_the_poor_rig_cannot_take_the_short_circuit_points(self):
        p = ft.build_test_plan(_blade(), ft.Instrument())
        assert _sev(p.findings, "plan-points-unmeasurable") == Severity.FAIL
        assert len(p.measurable_points()) < len(p.points)

    def test_the_instrumented_rig_can_take_them_all(self):
        p = ft.build_test_plan(_blade(), ft.instrumented_rig(20000.0))
        assert len(p.measurable_points()) == len(p.points)

    def test_a_plan_without_a_curve_says_so_rather_than_guessing(self):
        p = ft.build_test_plan(ft.FuseSpec(rating_a=15.0),
                               ft.instrumented_rig())
        assert _sev(p.findings, "plan-without-curve") == Severity.MISSING
        assert all(pt.expected_time_s is None for pt in p.points)


# ===================================================================== #
#  6.  INGESTING WHAT THE RIG MEASURED
# ===================================================================== #
class TestFit:
    def _meas(self, fuse, mults=(2.0, 5.0, 10.0), n=6, factor=1.0):
        out = []
        for m in mults:
            i = fuse.rating_a * m
            t = fuse.blow_time_s(i) * factor
            out.append(ft.Measurement(i, [t * (1 + 0.02 * k) for k in range(n)]))
        return out

    def test_measurements_matching_the_datasheet_are_in_family(self):
        f = _blade()
        r = ft.fit_measurements(self._meas(f), f, ft.instrumented_rig(20000.0))
        assert r.in_family is True
        assert _sev(r.findings, "in-family") == Severity.OK

    def test_parts_much_slower_than_published_are_a_fail(self):
        f = _blade()
        r = ft.fit_measurements(self._meas(f, factor=3.0), f,
                                ft.instrumented_rig(20000.0))
        assert r.in_family is False
        assert _sev(r.findings, "out-of-family") == Severity.FAIL

    def test_the_fitted_slope_recovers_the_generating_curve(self):
        f = _blade(anchors=((2.0, 5.0), (10.0, 0.01)))
        _a, b = f.power_law()
        r = ft.fit_measurements(self._meas(f, mults=(2.0, 4.0, 8.0)), f,
                                ft.instrumented_rig(20000.0))
        assert r.b == pytest.approx(b, rel=0.05)

    def test_readings_the_rig_cannot_resolve_are_rejected_not_averaged(self):
        """The hard rule: a measurement dominated by the instrument is not one."""
        f = _blade()
        meas = [ft.Measurement(150.0, [0.010, 0.011, 0.009]),   # 10 ms
                ft.Measurement(30.0, [5.0, 5.1, 4.9])]          # 5 s
        r = ft.fit_measurements(meas, f, ft.Instrument())       # 250 ms latency
        assert _sev(r.findings, "measurement-rejected") == Severity.FAIL
        assert 150.0 not in r.per_point
        assert 30.0 in r.per_point

    def test_one_usable_level_gives_no_slope_and_says_so(self):
        f = _blade()
        r = ft.fit_measurements([ft.Measurement(30.0, [5.0, 5.1, 4.9])], f,
                                ft.instrumented_rig())
        assert r.a is None and r.b is None
        assert _sev(r.findings, "fit-refused") == Severity.MISSING

    def test_thin_sample_counts_are_warned(self):
        f = _blade()
        meas = [ft.Measurement(30.0, [5.0, 5.1]),
                ft.Measurement(150.0, [0.01, 0.011])]
        r = ft.fit_measurements(meas, f, ft.instrumented_rig(20000.0))
        assert _sev(r.findings, "measurement-thin") == Severity.WARN

    def test_wide_unit_to_unit_scatter_is_surfaced(self):
        f = _blade()
        meas = [ft.Measurement(30.0, [1.0, 5.0, 25.0]),
                ft.Measurement(150.0, [0.01, 0.011, 0.009])]
        r = ft.fit_measurements(meas, f, ft.instrumented_rig(20000.0))
        assert _sev(r.findings, "high-scatter") == Severity.WARN

    def test_per_point_stats_are_geometric(self):
        f = _blade()
        r = ft.fit_measurements(
            [ft.Measurement(30.0, [4.0, 5.0, 6.25]),
             ft.Measurement(150.0, [0.01, 0.011, 0.009])],
            f, ft.instrumented_rig(20000.0))
        assert r.per_point[30.0]["median"] == pytest.approx(5.0, rel=0.01)


# ===================================================================== #
#  7.  GENERATED FIRMWARE
# ===================================================================== #
class TestSketch:
    def test_the_sketch_designs_out_the_proof_of_concept_faults(self):
        s = ft.emit_arduino_sketch(
            ft.build_test_plan(_blade(), ft.instrumented_rig()))
        # Comments deliberately DISCUSS millis(); the code must not use it.
        code = "\n".join(l for l in s.splitlines()
                         if not l.lstrip().startswith("//"))
        assert "micros()" in code
        assert "millis()" not in code
        assert "analogRead" in code
        assert "Serial.read" not in code, "no keypress in the timing path"

    def test_the_threshold_is_derived_from_the_shunt_and_gain(self):
        plan = ft.build_test_plan(_blade(), ft.instrumented_rig())
        a = ft.emit_arduino_sketch(plan, shunt_mohm=1.0, gain=50.0)
        b = ft.emit_arduino_sketch(plan, shunt_mohm=1.0, gain=100.0)
        assert a != b, "gain must change the generated threshold"

    def test_a_saturating_front_end_is_warned_about_in_the_source(self):
        plan = ft.build_test_plan(_blade(rating=100.0), ft.instrumented_rig())
        s = ft.emit_arduino_sketch(plan, shunt_mohm=10.0, gain=200.0)
        assert "WARNING" in s and "saturates" in s

    def test_the_planned_levels_are_documented_in_the_header(self):
        plan = ft.build_test_plan(_blade(), ft.instrumented_rig())
        s = ft.emit_arduino_sketch(plan)
        for p in plan.points:
            assert f"{p.current_a:8.1f} A" in s

    def test_unmeasurable_levels_are_marked_in_the_header(self):
        plan = ft.build_test_plan(_blade(), ft.Instrument())
        assert "BELOW RIG RESOLUTION" in ft.emit_arduino_sketch(plan)

    def test_the_sample_count_reaches_the_firmware(self):
        plan = ft.build_test_plan(_blade(), ft.instrumented_rig())
        s = ft.emit_arduino_sketch(plan)
        assert f"N_SAMPLES   = {plan.points[0].samples}" in s


# ===================================================================== #
#  8.  PROVENANCE
# ===================================================================== #
def test_provenance_separates_derived_declared_measured_and_estimated():
    assert ft.PROVENANCE["physics_grounded"]
    assert ft.PROVENANCE["declared_not_invented"]
    assert ft.PROVENANCE["measured_when_available"]
    assert ft.PROVENANCE["estimate_flagged"]
    assert "REJECTED" in ft.PROVENANCE["hard_rule"]


def test_the_prior_is_no_longer_listed_as_a_bare_estimate():
    """It moved from 'assumed forever' to 'displaced by measurement'."""
    blob = " ".join(ft.PROVENANCE["estimate_flagged"])
    assert "LOG_SCATTER" not in blob
    assert any("PRIOR_LOG_SCATTER" in s
               for s in ft.PROVENANCE["measured_when_available"])


def test_module_stays_headless_and_dependency_free():
    """Checked in a FRESH interpreter — see the note in test_daq_plan.py.

    The old form read this session's sys.modules, so it passed only where
    streamlit was not installed and broke the moment any earlier test imported
    it. The second assertion was `... or True`, which can never fail; the real
    intent — numpy must not be REQUIRED to import this module — is now actually
    enforced in the subprocess below.
    """
    import pathlib
    import subprocess
    import sys

    probe = (
        "import sys; import suspension.fuse_test; "
        "assert 'streamlit' not in sys.modules, 'fuse_test pulled in streamlit'; "
        "assert 'numpy' not in sys.modules, 'fuse_test requires numpy'"
    )
    proc = subprocess.run([sys.executable, "-c", probe],
                          capture_output=True, text=True,
                          cwd=str(pathlib.Path(__file__).resolve().parents[1]))
    assert proc.returncode == 0, proc.stderr.strip()


# ===================================================================== #
#  9.  SCATTER: MEASURED RATHER THAN ASSUMED
# ===================================================================== #
class TestScatterEstimate:
    def test_the_prior_is_labelled_as_not_measured(self):
        p = ft.prior_scatter()
        assert not p.measured
        assert p.dof == 0
        assert p.rel_uncertainty() == math.inf

    def test_pooling_needs_at_least_two_samples_at_some_level(self):
        assert ft.pooled_log_scatter([ft.Measurement(30.0, [5.0])]) is None
        assert ft.pooled_log_scatter([]) is None

    def test_pooling_recovers_a_known_spread(self):
        """Generate from a known sigma; the pool should find it."""
        import random
        random.seed(11)
        sigma = 0.40
        ts = [5.0 * math.exp(random.gauss(0.0, sigma)) for _ in range(400)]
        est = ft.pooled_log_scatter([ft.Measurement(30.0, ts)])
        assert est.measured
        assert est.sigma_ln == pytest.approx(sigma, rel=0.15)

    def test_pooling_combines_degrees_of_freedom_across_levels(self):
        m = [ft.Measurement(30.0, [4.0, 5.0, 6.0]),
             ft.Measurement(150.0, [0.01, 0.012, 0.009, 0.011])]
        est = ft.pooled_log_scatter(m)
        assert est.dof == (3 - 1) + (4 - 1)
        assert est.n_levels == 2

    def test_sigma_is_better_known_with_more_degrees_of_freedom(self):
        a = ft.ScatterEstimate(0.3, dof=4, measured=True)
        b = ft.ScatterEstimate(0.3, dof=40, measured=True)
        assert b.rel_uncertainty() < a.rel_uncertainty()
        assert b.upper_bound() < a.upper_bound()

    def test_the_upper_bound_always_exceeds_the_point_estimate(self):
        e = ft.ScatterEstimate(0.3, dof=10, measured=True)
        assert e.upper_bound() > e.sigma_ln

    def test_an_unmeasured_estimate_has_no_bound_to_offer(self):
        p = ft.prior_scatter()
        assert p.upper_bound() == p.sigma_ln


class TestChiSquareApproximation:
    @pytest.mark.parametrize("dof,published", [
        (4, 0.7107), (10, 3.9403), (20, 10.8508), (30, 18.4927),
    ])
    def test_matches_published_lower_tail_quantiles(self, dof, published):
        got = ft._chi2_lower_quantile(dof, 0.05)
        assert got == pytest.approx(published, rel=0.03)

    def test_errs_low_which_inflates_the_bound_conservatively(self):
        for dof in (4, 10, 20, 30):
            assert ft._chi2_lower_quantile(dof, 0.05) <= {
                4: 0.7107, 10: 3.9403, 20: 10.8508, 30: 18.4927}[dof]


class TestTwoStageSizing:
    def _pilot(self, n, sigma, seed=3):
        import random
        random.seed(seed)
        return [ft.Measurement(30.0,
                               [5.0 * math.exp(random.gauss(0.0, sigma))
                                for _ in range(n)])]

    def test_recommended_pilot_inverts_the_uncertainty_relation(self):
        assert ft.recommended_pilot_n(0.25) == 9
        assert ft.recommended_pilot_n(0.10) > ft.recommended_pilot_n(0.30)

    def test_zero_target_uncertainty_is_rejected(self):
        with pytest.raises(ValueError):
            ft.recommended_pilot_n(0.0)

    def test_a_pilot_with_no_spread_refuses_to_size_anything(self):
        r = ft.refine_from_pilot([ft.Measurement(30.0, [5.0])])
        assert r["n_required"] is None
        assert r["finding"].check == "pilot-too-thin"

    def test_a_tiny_pilot_refuses_the_absurd_upper_bound_number(self):
        """Correct statistics, useless advice — so it says which is which."""
        r = ft.refine_from_pilot(self._pilot(3, 0.3))
        assert r["pilot_adequate"] is False
        assert r["finding"].check == "pilot-underpowered"
        assert r["n_required"] == r["n_point_estimate"]
        assert r["n_upper_bound"] > r["n_point_estimate"]

    def test_an_adequate_pilot_sizes_on_the_upper_bound(self):
        r = ft.refine_from_pilot(self._pilot(12, 0.3))
        assert r["pilot_adequate"] is True
        assert r["n_required"] == r["n_upper_bound"]
        assert r["n_required"] > r["n_point_estimate"]

    def test_tight_parts_need_fewer_fuses_than_the_prior_assumed(self):
        r = ft.refine_from_pilot(self._pilot(20, 0.10))
        assert r["n_required"] < r["n_prior"]
        assert r["finding"].severity == Severity.OK

    def test_wide_parts_need_more_and_that_is_a_warning(self):
        r = ft.refine_from_pilot(self._pilot(20, 0.60))
        assert r["n_required"] > r["n_prior"]
        assert r["finding"].severity == Severity.WARN


class TestPlanScatterProvenance:
    def test_a_plan_on_the_prior_declares_itself_unverified(self):
        p = ft.build_test_plan(_blade(), ft.instrumented_rig())
        assert not p.sized_on_measurement()
        assert _sev(p.findings, "scatter-source-assumed") == Severity.MISSING

    def test_a_plan_on_measured_scatter_says_so(self):
        est = ft.ScatterEstimate(0.2, dof=12, measured=True, n_levels=1)
        p = ft.build_test_plan(_blade(), ft.instrumented_rig(),
                               log_scatter=est)
        assert p.sized_on_measurement()
        assert _sev(p.findings, "scatter-source-measured") == Severity.OK
        assert "scatter-source-assumed" not in _checks(p.findings)

    def test_measured_scatter_actually_changes_the_fuse_count(self):
        tight = ft.ScatterEstimate(0.10, dof=30, measured=True, n_levels=1)
        wide = ft.ScatterEstimate(0.60, dof=30, measured=True, n_levels=1)
        a = ft.build_test_plan(_blade(), ft.instrumented_rig(),
                               log_scatter=tight)
        b = ft.build_test_plan(_blade(), ft.instrumented_rig(),
                               log_scatter=wide)
        assert b.fuses_consumed > a.fuses_consumed

    def test_the_fit_hands_back_scatter_ready_to_re_size_the_next_test(self):
        f = _blade()
        meas = [ft.Measurement(30.0, [4.6, 5.0, 5.4, 5.1]),
                ft.Measurement(150.0, [0.0095, 0.0105, 0.010, 0.0102])]
        r = ft.fit_measurements(meas, f, ft.instrumented_rig(20000.0))
        assert r.scatter is not None and r.scatter.measured
        # and it drops straight back into planning
        p = ft.build_test_plan(f, ft.instrumented_rig(), log_scatter=r.scatter)
        assert p.sized_on_measurement()
