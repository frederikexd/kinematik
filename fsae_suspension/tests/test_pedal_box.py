# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Tests for the pedal-box packaging / balance-bar / travel module
(suspension/pedal_box.py). Verifies:

  * the longitudinal stack-up sums its segments, detects a deficit, and names the
    master-cylinder outlet as real length rather than silently dropping it,
  * a catalogue cylinder length flags the result as an estimate (honesty contract),
  * shortening options are priced, ordered cheapest-first, and every one that
    trades pedal travel SAYS SO in its side effects,
  * plan_shortening never claims to have solved a deficit it has not covered,
  * balance-bar statics obey the lever: moving the pivot toward a cylinder raises
    that cylinder's force, and the two forces always sum to the rod force,
  * equal bores + equal calipers + a centred bar gives exactly 50% bias,
  * a target bias outside the reachable band is reported unreachable with the
    right bore advice, never quietly clamped,
  * pedal travel rises with a smaller bore and with a higher pedal ratio (the two
    trades that fight the packaging fix), and trapped air dominates when present,
  * calibration against one measured travel reproduces that measurement,
  * the coupled study takes the worst of the three verdicts and catches a
    front/rear stroke mismatch.
"""

import math

import pytest

from suspension.interfaces import Severity
from suspension import pedal_box as pb


# --------------------------------------------------------------------------- #
#  Fixtures — the deck's car: 220 mm front rotors, 2-piston opposed front
#  calipers, single-piston rear, dual master cylinders on a balance bar.
# --------------------------------------------------------------------------- #
@pytest.fixture
def front_circuit():
    return pb.CircuitSpec(mc_bore_mm=15.875, caliper_piston_dia_mm=30.0,
                          pistons_per_side=2, opposed=True, pad_mu=0.45,
                          rotor_dia_mm=220.0, n_corners=2)


@pytest.fixture
def rear_circuit():
    return pb.CircuitSpec(mc_bore_mm=17.5, caliper_piston_dia_mm=25.0,
                          pistons_per_side=1, opposed=True, pad_mu=0.45,
                          rotor_dia_mm=200.0, n_corners=2)


def _sev(findings, check):
    for f in findings:
        if f.check == check:
            return f.severity
    return None


# =========================================================================== #
#  1. Stack-up
# =========================================================================== #
def test_stack_up_sums_its_segments():
    s = pb.stack_up(available_mm=400.0)
    assert s.installed_mm == pytest.approx(sum(x.length_mm for x in s.segments))
    assert s.deficit_mm == pytest.approx(s.installed_mm - 400.0)


def test_stack_up_detects_a_deficit_and_fails_loudly():
    s = pb.stack_up(available_mm=290.0)
    assert not s.fits
    assert s.deficit_mm > 0
    assert s.verdict == "DOES NOT FIT"
    assert _sev(s.findings, "pedal_box_envelope") == Severity.FAIL


def test_stack_up_fits_when_there_is_room():
    s = pb.stack_up(available_mm=600.0)
    assert s.fits
    assert s.verdict == "FITS"
    assert s.deficit_mm < 0


def test_tight_band_is_distinct_from_fits():
    s_loose = pb.stack_up(available_mm=600.0)
    # Place the bulkhead just past the installed length -> TIGHT, not FITS.
    s_tight = pb.stack_up(available_mm=s_loose.installed_mm + 3.0)
    assert s_tight.verdict == "TIGHT"
    assert s_tight.fits


def test_mc_outlet_is_carried_as_real_length():
    """The 'lines coming out the rear of the MCs' item must be IN the stack."""
    s = pb.stack_up(available_mm=400.0,
                    mc_outlet="straight fitting + hardline bend")
    names = [x.name for x in s.segments]
    assert any("outlet" in n.lower() for n in names)
    outlet = next(x for x in s.segments if "outlet" in x.name.lower())
    assert outlet.length_mm == pytest.approx(
        pb.FITTING_STACK_MM["straight fitting + hardline bend"])
    # and it must be flagged, because it is the segment CAD checks omit
    assert _sev(s.findings, "mc_outlet_stack") == Severity.WARN


def test_a_shorter_fitting_shortens_the_stack():
    straight = pb.stack_up(available_mm=400.0,
                           mc_outlet="straight fitting + hardline bend")
    banjo = pb.stack_up(available_mm=400.0, mc_outlet="90 deg banjo")
    assert banjo.installed_mm < straight.installed_mm


def test_catalogue_cylinder_length_flags_the_result_as_an_estimate():
    s = pb.stack_up(available_mm=400.0)          # no mc_body_mm given
    assert s.is_estimate
    assert _sev(s.findings, "mc_body_length_source") == Severity.MISSING


def test_measured_cylinder_length_is_accepted_as_measured():
    s = pb.stack_up(available_mm=400.0, mc_body_mm=104.0, mc_body_measured=True,
                    mc_outlet_mm=30.0)
    body = next(x for x in s.segments if "body" in x.name.lower())
    assert body.measured
    assert body.length_mm == pytest.approx(104.0)
    assert _sev(s.findings, "mc_body_length_source") == Severity.OK


def test_a_more_upright_pedal_projects_less_length():
    shallow = pb.stack_up(available_mm=400.0, pedal_rest_angle_deg=15.0)
    upright = pb.stack_up(available_mm=400.0, pedal_rest_angle_deg=40.0)
    assert upright.installed_mm < shallow.installed_mm


def test_tilting_the_cylinders_shortens_the_x_stack():
    flat = pb.stack_up(available_mm=400.0, tilt_deg=0.0)
    tilted = pb.stack_up(available_mm=400.0, tilt_deg=15.0)
    assert tilted.installed_mm < flat.installed_mm


def test_exhausted_pushrod_thread_is_flagged():
    s = pb.stack_up(available_mm=400.0, pushrod_thread_dia_mm=8.0,
                    pushrod_engaged_mm=12.5)   # ~ the 1.5*d minimum
    assert _sev(s.findings, "pushrod_adjustment_exhausted") == Severity.WARN


# =========================================================================== #
#  2. Shortening options
# =========================================================================== #
def test_options_are_ordered_cheapest_first():
    s = pb.stack_up(available_mm=290.0)
    opts = pb.shorten_options(s)
    ranks = [o.cost_rank for o in opts]
    assert ranks == sorted(ranks)


def test_every_option_is_priced_in_millimetres():
    s = pb.stack_up(available_mm=290.0)
    for o in pb.shorten_options(s):
        assert isinstance(o.gain_mm, float)
        assert o.cost, f"{o.name} has no stated cost"


def test_ratio_and_bore_options_declare_their_travel_cost():
    """A lever that costs pedal travel must SAY so — that is the whole point."""
    s = pb.stack_up(available_mm=290.0)
    opts = pb.shorten_options(s, pedal_ratio=5.0, pedal_ratio_max=6.5)
    ratio_opt = next(o for o in opts if "pedal ratio" in o.name.lower())
    joined = " ".join(f"{k} {v}" for k, v in ratio_opt.side_effects.items()).lower()
    assert "travel" in joined
    assert "higher" in joined
    assert ratio_opt.requires_recheck


def test_bore_increase_reports_the_square_law_on_effort():
    s = pb.stack_up(available_mm=290.0)
    opts = pb.shorten_options(s, mc_bore_mm=15.875, mc_bore_up_mm=19.05)
    bore_opt = next(o for o in opts if "bore" in o.name.lower())
    # (19.05/15.875)^2 = 1.44
    joined = " ".join(str(v) for v in bore_opt.side_effects.values())
    assert "1.44" in joined


def test_excessive_tilt_is_marked_infeasible():
    s = pb.stack_up(available_mm=290.0)
    opts = pb.shorten_options(s, tilt_max_deg=25.0)
    tilt_opt = next(o for o in opts if "tilt" in o.name.lower())
    assert not tilt_opt.feasible


def test_plan_solves_a_small_deficit():
    s = pb.stack_up(available_mm=355.0)
    plan = pb.plan_shortening(s)
    assert plan.solved
    assert plan.total_gain_mm >= plan.deficit_mm
    assert plan.remaining_mm == 0.0


def test_plan_never_pretends_to_solve_a_huge_deficit():
    s = pb.stack_up(available_mm=150.0)
    plan = pb.plan_shortening(s)
    assert not plan.solved
    assert plan.remaining_mm > 0
    assert _sev(plan.findings, "shorten_plan") == Severity.FAIL


def test_plan_on_a_fitting_assembly_is_a_no_op():
    s = pb.stack_up(available_mm=600.0)
    plan = pb.plan_shortening(s)
    assert plan.solved
    assert plan.chosen == []


def test_plan_respects_the_cost_ceiling():
    s = pb.stack_up(available_mm=290.0)
    plan = pb.plan_shortening(s, max_cost_rank=0)
    assert all(o.cost_rank == 0 for o in plan.chosen)


def test_plan_warns_when_priced_off_catalogue_lengths():
    s = pb.stack_up(available_mm=290.0)          # estimate by construction
    plan = pb.plan_shortening(s)
    assert _sev(plan.findings, "shorten_plan_provenance") == Severity.WARN


# =========================================================================== #
#  3. Balance bar
# =========================================================================== #
def test_bar_forces_always_sum_to_the_rod_force(front_circuit, rear_circuit):
    for e in (-12.0, -5.0, 0.0, 5.0, 12.0):
        r = pb.balance_bar_bias(pedal_force_N=500.0, pedal_ratio=5.0,
                                front=front_circuit, rear=rear_circuit,
                                bar_length_mm=60.0, bar_offset_mm=e)
        assert r.force_front_N + r.force_rear_N == pytest.approx(500.0 * 5.0)


def test_moving_the_pivot_toward_the_front_raises_front_force(front_circuit,
                                                              rear_circuit):
    """The clevis NEARER the loaded pivot carries more — standard lever statics."""
    lo = pb.balance_bar_bias(pedal_force_N=500.0, pedal_ratio=5.0,
                             front=front_circuit, rear=rear_circuit,
                             bar_offset_mm=-8.0)
    hi = pb.balance_bar_bias(pedal_force_N=500.0, pedal_ratio=5.0,
                             front=front_circuit, rear=rear_circuit,
                             bar_offset_mm=+8.0)
    assert hi.force_front_N > lo.force_front_N
    assert hi.bias_front > lo.bias_front


def test_identical_hardware_centred_gives_exactly_half(front_circuit):
    mirror = pb.CircuitSpec(**{**front_circuit.__dict__})
    r = pb.balance_bar_bias(pedal_force_N=500.0, pedal_ratio=5.0,
                            front=front_circuit, rear=mirror, bar_offset_mm=0.0)
    assert r.bias_front == pytest.approx(0.5, abs=1e-9)
    assert r.pressure_front_bar == pytest.approx(r.pressure_rear_bar)


def test_a_smaller_front_bore_makes_more_front_pressure(rear_circuit):
    big = pb.CircuitSpec(mc_bore_mm=19.05, caliper_piston_dia_mm=30.0,
                         rotor_dia_mm=220.0)
    small = pb.CircuitSpec(mc_bore_mm=15.875, caliper_piston_dia_mm=30.0,
                           rotor_dia_mm=220.0)
    r_big = pb.balance_bar_bias(pedal_force_N=500.0, pedal_ratio=5.0,
                                front=big, rear=rear_circuit)
    r_small = pb.balance_bar_bias(pedal_force_N=500.0, pedal_ratio=5.0,
                                  front=small, rear=rear_circuit)
    assert r_small.pressure_front_bar > r_big.pressure_front_bar


def test_offset_outside_the_bar_span_is_rejected(front_circuit, rear_circuit):
    r = pb.balance_bar_bias(pedal_force_N=500.0, pedal_ratio=5.0,
                            front=front_circuit, rear=rear_circuit,
                            bar_length_mm=60.0, bar_offset_mm=45.0)
    assert _sev(r.findings, "balance_bar_offset") == Severity.FAIL


def test_authority_band_brackets_the_centred_bias(front_circuit, rear_circuit):
    a = pb.bias_authority(pedal_force_N=500.0, pedal_ratio=5.0,
                          front=front_circuit, rear=rear_circuit)
    assert a.bias_min <= a.bias_at_centre <= a.bias_max


def test_unreachable_target_bias_is_reported_not_clamped(front_circuit,
                                                         rear_circuit):
    a = pb.bias_authority(pedal_force_N=500.0, pedal_ratio=5.0,
                          front=front_circuit, rear=rear_circuit,
                          target_bias=0.30)          # far below the band
    assert not a.target_reachable
    assert a.offset_for_target_mm is None
    f = next(f for f in a.findings if f.check == "target_bias_reachable")
    assert f.severity == Severity.FAIL
    assert "bore" in f.message.lower()


def test_reachable_target_solves_to_an_offset_that_reproduces_it(front_circuit,
                                                                 rear_circuit):
    a = pb.bias_authority(pedal_force_N=500.0, pedal_ratio=5.0,
                          front=front_circuit, rear=rear_circuit,
                          target_bias=0.70)
    assert a.target_reachable
    check = pb.balance_bar_bias(pedal_force_N=500.0, pedal_ratio=5.0,
                                front=front_circuit, rear=rear_circuit,
                                bar_offset_mm=a.offset_for_target_mm)
    assert check.bias_front == pytest.approx(0.70, abs=1e-4)


def test_bias_per_turn_is_a_real_nonzero_number(front_circuit, rear_circuit):
    a = pb.bias_authority(pedal_force_N=500.0, pedal_ratio=5.0,
                          front=front_circuit, rear=rear_circuit)
    assert abs(a.bias_per_turn) > 0
    assert abs(a.bias_per_turn) < 0.5      # a turn cannot swing half the car


def test_a_bar_near_its_stop_loses_trim_authority(front_circuit, rear_circuit):
    r = pb.balance_bar_bias(pedal_force_N=500.0, pedal_ratio=5.0,
                            front=front_circuit, rear=rear_circuit,
                            bar_length_mm=60.0, bar_offset_mm=27.0)
    assert _sev(r.findings, "bar_authority") == Severity.WARN


# =========================================================================== #
#  4. Pedal travel
# =========================================================================== #
def test_travel_sums_its_volume_items(front_circuit):
    t = pb.pedal_travel(circuit=front_circuit, line_pressure_bar=60.0,
                        pedal_ratio=5.0)
    assert t.total_cc == pytest.approx(sum(i.volume_cc for i in t.items))
    assert t.mc_stroke_mm > 0
    assert t.pedal_travel_mm > t.mc_stroke_mm      # the ratio multiplies it


def test_a_smaller_bore_costs_travel(rear_circuit):
    small = pb.CircuitSpec(mc_bore_mm=14.0, caliper_piston_dia_mm=30.0,
                           rotor_dia_mm=220.0)
    big = pb.CircuitSpec(mc_bore_mm=20.6, caliper_piston_dia_mm=30.0,
                         rotor_dia_mm=220.0)
    t_small = pb.pedal_travel(circuit=small, line_pressure_bar=60.0,
                              pedal_ratio=5.0)
    t_big = pb.pedal_travel(circuit=big, line_pressure_bar=60.0, pedal_ratio=5.0)
    assert t_small.mc_stroke_mm > t_big.mc_stroke_mm


def test_a_higher_pedal_ratio_costs_travel(front_circuit):
    lo = pb.pedal_travel(circuit=front_circuit, line_pressure_bar=60.0,
                         pedal_ratio=4.0)
    hi = pb.pedal_travel(circuit=front_circuit, line_pressure_bar=60.0,
                         pedal_ratio=6.5)
    assert hi.pedal_travel_mm > lo.pedal_travel_mm


def test_an_opposed_caliper_demands_double_the_volume():
    opposed = pb.CircuitSpec(mc_bore_mm=15.875, caliper_piston_dia_mm=30.0,
                             pistons_per_side=2, opposed=True)
    floating = pb.CircuitSpec(mc_bore_mm=15.875, caliper_piston_dia_mm=30.0,
                              pistons_per_side=2, opposed=False)
    assert opposed.swept_area_mm2 == pytest.approx(2 * floating.swept_area_mm2)
    # ...but the same clamp force, which is the distinction teams get wrong
    assert opposed.clamp_area_mm2 == pytest.approx(floating.clamp_area_mm2)


def test_rubber_hose_costs_more_travel_than_ptfe(front_circuit):
    rubber = pb.TravelParams(hose_type="rubber hose")
    ptfe = pb.TravelParams(hose_type="PTFE braided (steel)")
    t_r = pb.pedal_travel(circuit=front_circuit, line_pressure_bar=70.0,
                          pedal_ratio=5.0, params=rubber)
    t_p = pb.pedal_travel(circuit=front_circuit, line_pressure_bar=70.0,
                          pedal_ratio=5.0, params=ptfe)
    assert t_r.pedal_travel_mm > t_p.pedal_travel_mm


def test_trapped_air_dominates_and_is_flagged(front_circuit):
    airy = pb.TravelParams(air_cc=2.0)
    t = pb.pedal_travel(circuit=front_circuit, line_pressure_bar=70.0,
                        pedal_ratio=5.0, params=airy)
    biggest = t.biggest(1)[0]
    assert "air" in biggest.name.lower()
    assert _sev(t.findings, "trapped_air") == Severity.WARN


def test_a_bottomed_cylinder_fails_not_merely_warns(front_circuit):
    t = pb.pedal_travel(circuit=front_circuit, line_pressure_bar=70.0,
                        pedal_ratio=5.0, mc_stroke_limit_mm=3.0,
                        available_travel_mm=500.0)
    assert t.verdict == "FAIL"
    assert _sev(t.findings, "mc_stroke") == Severity.FAIL


def test_generous_room_passes(front_circuit):
    t = pb.pedal_travel(circuit=front_circuit, line_pressure_bar=60.0,
                        pedal_ratio=5.0, available_travel_mm=300.0,
                        mc_stroke_limit_mm=60.0)
    assert t.verdict == "PASS"


def test_uncalibrated_travel_is_flagged_as_provisional(front_circuit):
    t = pb.pedal_travel(circuit=front_circuit, line_pressure_bar=60.0,
                        pedal_ratio=5.0)
    assert t.is_estimate
    assert _sev(t.findings, "travel_provenance") == Severity.MISSING


def test_line_volume_matches_a_hand_calculation():
    # 1000 mm of 3.2 mm ID: pi/4 * 3.2^2 * 1000 mm^3 = 8.04 cc
    assert pb.line_volume_cc(1000.0, 3.2) == pytest.approx(8.042, rel=1e-3)


def test_calibration_reproduces_the_measured_travel(front_circuit):
    measured = 42.0
    p = pb.calibrate_travel_params(measured_pedal_travel_mm=measured,
                                   circuit=front_circuit,
                                   line_pressure_bar=60.0, pedal_ratio=5.0)
    assert p.calibrated
    assert p.fitted_to
    t = pb.pedal_travel(circuit=front_circuit, line_pressure_bar=60.0,
                        pedal_ratio=5.0, params=p)
    assert t.pedal_travel_mm == pytest.approx(measured, rel=1e-6)
    assert not t.is_estimate


def test_calibrated_params_drop_the_provisional_flag(front_circuit):
    p = pb.calibrate_travel_params(measured_pedal_travel_mm=42.0,
                                   circuit=front_circuit,
                                   line_pressure_bar=60.0, pedal_ratio=5.0)
    t = pb.pedal_travel(circuit=front_circuit, line_pressure_bar=60.0,
                        pedal_ratio=5.0, params=p)
    assert _sev(t.findings, "travel_provenance") is None


# =========================================================================== #
#  5. The coupled study
# =========================================================================== #
def test_study_runs_end_to_end(front_circuit, rear_circuit):
    s = pb.study(available_mm=290.0, front=front_circuit, rear=rear_circuit,
                 target_bias=0.65)
    assert s.stack.deficit_mm > 0
    assert s.plan is not None
    assert s.travel_front.total_cc > 0
    assert s.verdict in ("PASS", "TIGHT", "FAIL")
    assert s.summary()


def test_study_takes_the_worst_verdict(front_circuit, rear_circuit):
    """A pedal box is only as good as its worst axis."""
    s = pb.study(available_mm=150.0, front=front_circuit, rear=rear_circuit,
                 target_bias=0.65)
    assert s.stack.verdict == "DOES NOT FIT"
    assert s.verdict == "FAIL"


def test_study_computes_travel_at_the_bar_s_own_pressures(front_circuit,
                                                          rear_circuit):
    """Travel must use the pressure the bar actually makes, not a round number."""
    s = pb.study(available_mm=600.0, front=front_circuit, rear=rear_circuit,
                 bar_offset_mm=6.0, target_bias=None)
    manual = pb.pedal_travel(circuit=front_circuit,
                             line_pressure_bar=s.bias.pressure_front_bar,
                             pedal_ratio=5.0)
    assert s.travel_front.total_cc == pytest.approx(manual.total_cc)


def test_study_flags_a_front_rear_stroke_mismatch(rear_circuit):
    """Two circuits, one pedal: a big stroke mismatch tilts the bar under braking."""
    hungry = pb.CircuitSpec(mc_bore_mm=13.0, caliper_piston_dia_mm=38.0,
                            pistons_per_side=2, opposed=True, rotor_dia_mm=240.0)
    lean = pb.CircuitSpec(mc_bore_mm=22.2, caliper_piston_dia_mm=20.0,
                          pistons_per_side=1, opposed=False, rotor_dia_mm=180.0)
    s = pb.study(available_mm=600.0, front=hungry, rear=lean, target_bias=None)
    assert _sev(s.findings, "circuit_stroke_mismatch") == Severity.WARN


def test_unreachable_bias_fails_the_whole_study(front_circuit, rear_circuit):
    s = pb.study(available_mm=600.0, front=front_circuit, rear=rear_circuit,
                 target_bias=0.30)
    assert not s.authority.target_reachable
    assert s.verdict == "FAIL"


def test_study_serialises(front_circuit, rear_circuit):
    s = pb.study(available_mm=290.0, front=front_circuit, rear=rear_circuit)
    d = s.as_dict()
    assert set(("verdict", "stack", "bias", "authority",
                "travel_front", "travel_rear")).issubset(d.keys())
    assert isinstance(d["stack"]["segments"], list)


def test_provenance_names_what_is_safe_and_what_is_not():
    p = pb.provenance()
    assert "safe" in p and "provisional" in p
    assert "parameters" in p["provisional"].lower()


# =========================================================================== #
#  6. Unit-conversion safety
#
#  The engine is metric, but the app renders findings verbatim through
#  units.usentence(), which converts "<number> <unit>" pairs in place and
#  PRESERVES the source decimal precision. Two consequences bite:
#
#    * mm -> in divides by 25.4, so a 0-decimal source ("85 mm") renders as
#      "3 in" and an 8 mm value renders as "0 in" -- the number is destroyed.
#      Every mm figure in a user-facing string therefore needs >= 1 decimal.
#    * a range written "20-40 mm" has only ONE unit token, so only the 40
#      converts and an imperial user reads "20-2 in". Both endpoints must
#      carry their own unit.
#
#  These guard both properties at the source, because the failure is silent:
#  the app renders happily and just shows the wrong number.
# =========================================================================== #
import re as _re
import suspension.pedal_box as _pbmod


def _all_finding_messages():
    """Findings from every entry point, so the scan covers real output."""
    front = pb.CircuitSpec(mc_bore_mm=15.875, caliper_piston_dia_mm=30.0,
                           pistons_per_side=2, rotor_dia_mm=220.0, n_corners=2)
    rear = pb.CircuitSpec(mc_bore_mm=17.5, caliper_piston_dia_mm=25.0,
                          pistons_per_side=1, rotor_dia_mm=200.0, n_corners=2)
    msgs = []
    for available in (150.0, 290.0, 600.0):
        s = pb.stack_up(available_mm=available)
        msgs += [f.message for f in s.findings]
        msgs += [f.message for f in pb.plan_shortening(s).findings]
    msgs += [f.message for f in pb.balance_bar_bias(
        pedal_force_N=500.0, pedal_ratio=5.0, front=front, rear=rear,
        bar_offset_mm=27.0).findings]
    for target in (0.30, 0.65, 0.70):
        msgs += [f.message for f in pb.bias_authority(
            pedal_force_N=500.0, pedal_ratio=5.0, front=front, rear=rear,
            target_bias=target).findings]
    for params in (pb.TravelParams(), pb.TravelParams(air_cc=2.0)):
        t = pb.pedal_travel(circuit=front, line_pressure_bar=63.0,
                            pedal_ratio=5.0, params=params)
        msgs += [f.message for f in t.findings]
        msgs += [i.note for i in t.items if i.note]
    msgs += [f.message for f in pb.study(
        available_mm=290.0, front=front, rear=rear).findings]
    return [m for m in msgs if m]


def test_every_mm_figure_survives_imperial_conversion():
    """A 0-decimal mm figure becomes a useless integer inch. Require a decimal."""
    bad = []
    for msg in _all_finding_messages():
        # "<digits> mm" with no decimal point, on a word boundary.
        for m in _re.finditer(r"(?<![\d.])(\d+) mm\b", msg):
            bad.append((m.group(0), msg[:70]))
    assert not bad, (
        "These mm figures have no decimal, so imperial users see a rounded "
        f"integer inch (8 mm -> '0 in'): {bad}")


def test_every_cc_figure_keeps_enough_decimals():
    """cc -> in^3 multiplies by 0.061, so 2 decimals of cc rounds to nothing."""
    bad = []
    for msg in _all_finding_messages():
        for m in _re.finditer(r"(\d+)\.(\d{0,2}) cc\b", msg):
            bad.append((m.group(0), msg[:70]))
    assert not bad, (
        f"These cc figures need >=3 decimals to survive conversion: {bad}")


def test_no_numeric_range_shares_a_single_unit_token():
    """"20-40 mm" converts only the 40, so imperial reads "20-2 in"."""
    bad = []
    for msg in _all_finding_messages():
        for m in _re.finditer(r"\d[\d.]*\s*[-\u2013]\s*\d[\d.]*\s*(mm|cc|bar|N)\b",
                              msg):
            bad.append((m.group(0), msg[:70]))
    assert not bad, (
        "Both endpoints of a range must carry their own unit, or only the "
        f"second converts: {bad}")


def test_conversion_actually_produces_sane_imperial_numbers():
    """End-to-end: force US mode and check the headline figures convert."""
    import suspension.units as u
    orig_cur, orig_is = u.current_system, u.is_us
    try:
        u.current_system = lambda: u.US
        u.is_us = lambda: True
        s = pb.stack_up(available_mm=290.0)
        txt = u.usentence(
            next(f.message for f in s.findings
                 if f.check == "pedal_box_envelope"))
        # 84.9 mm deficit -> 3.3 in, 290 mm -> 11.4 in. Neither may round to 0.
        assert "3.3 in" in txt, txt
        assert "11.4 in" in txt, txt
        assert " 0 in" not in txt, txt
        assert "mm" not in txt, f"metric unit leaked through: {txt}"
    finally:
        u.current_system, u.is_us = orig_cur, orig_is


def test_metric_mode_is_untouched():
    """The conversion layer must be a no-op in metric — no silent rewriting."""
    import suspension.units as u
    orig_cur, orig_is = u.current_system, u.is_us
    try:
        u.current_system = lambda: u.METRIC
        u.is_us = lambda: False
        for msg in _all_finding_messages():
            assert u.usentence(msg) == msg
    finally:
        u.current_system, u.is_us = orig_cur, orig_is
