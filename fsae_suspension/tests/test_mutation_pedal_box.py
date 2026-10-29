"""suspension/pedal_box.py: the lever, area and volume arithmetic checked against formulas written here, and every
public result over a grid of cases that reaches each branch held against a recorded characterization file
(tests/golden/pedal_box.json) so every message, verdict and number is pinned."""
import math

import pytest

from suspension import pedal_box as pb
from tests._golden import check

F = pb.CircuitSpec(mc_bore_mm=15.875, caliper_piston_dia_mm=25.4, pistons_per_side=2, rotor_dia_mm=220.0)
R = pb.CircuitSpec(mc_bore_mm=19.05, caliper_piston_dia_mm=25.4, pistons_per_side=1, opposed=False, pad_mu=0.4,
                   rotor_dia_mm=200.0, pad_inner_radius_mm=70.0)


def test_the_closed_forms():
    assert pb._mm2_of_dia(10.0) == pytest.approx(25 * math.pi)
    assert F.mc_area_mm2 == pytest.approx(math.pi * 15.875 ** 2 / 4)
    assert F.clamp_area_mm2 == pytest.approx(2 * math.pi * 25.4 ** 2 / 4)
    assert F.swept_area_mm2 == pytest.approx(4 * math.pi * 25.4 ** 2 / 4)
    assert R.swept_area_mm2 == pytest.approx(math.pi * 25.4 ** 2 / 4)
    assert F.r_eff_mm == pytest.approx(110 * 0.86) and R.r_eff_mm == pytest.approx(85.0)
    assert pb.CircuitSpec(15.0, 25.0, pad_inner_radius_mm=200.0).r_eff_mm == pytest.approx(110 * 0.86)
    assert pb.CircuitSpec(15.0, 25.0, pad_inner_radius_mm=0.0).r_eff_mm == pytest.approx(110 * 0.86)
    assert pb.CircuitSpec(15.0, 25.0, pistons_per_side=0).clamp_area_mm2 == pytest.approx(math.pi * 625 / 4)
    t = F.axle_torque_Nm(40.0)
    assert t == pytest.approx(2 * 40e5 * F.clamp_area_mm2 * 1e-6 * 0.45 * F.r_eff_mm * 1e-3 * 2)
    assert pb.CircuitSpec(15.0, 25.0, n_corners=0).axle_torque_Nm(10.0) == pytest.approx(
        pb.CircuitSpec(15.0, 25.0, n_corners=1).axle_torque_Nm(10.0))
    assert pb.line_volume_cc(1000.0, 3.2) == pytest.approx(math.pi * 3.2 ** 2 / 4)
    b = pb.balance_bar_bias(pedal_force_N=500.0, pedal_ratio=5.0, front=F, rear=R, bar_length_mm=60.0,
                            bar_offset_mm=6.0)
    ff, fr = 2500.0 * 36 / 60, 2500.0 * 24 / 60
    assert (b.force_front_N, b.force_rear_N) == (pytest.approx(ff), pytest.approx(fr))
    assert b.pressure_front_bar == pytest.approx(ff / (F.mc_area_mm2 * 1e-6) / 1e5)
    assert b.bias_front == pytest.approx(b.torque_front_Nm / (b.torque_front_Nm + b.torque_rear_Nm))
    tp = pb.TravelParams()
    assert (tp.bulk_modulus_pa(), tp.hose_cc_per_m_per_bar(), tp.hardline_cc_per_m_per_bar()) == (1.50e9, 8.0e-4,
                                                                                                  1.0e-4)
    odd = pb.TravelParams(fluid="?", hose_type="?", hardline_type="?")
    assert (odd.bulk_modulus_pa(), odd.hose_cc_per_m_per_bar(), odd.hardline_cc_per_m_per_bar()) == (1.50e9, 8.0e-4,
                                                                                                     1.0e-4)
    sil = pb.TravelParams(fluid="DOT 5 (silicone)", hose_type="rubber hose", hardline_type="hardline (Cu-Ni tube)")
    assert (sil.bulk_modulus_pa(), sil.hose_cc_per_m_per_bar(), sil.hardline_cc_per_m_per_bar()) == (1.10e9, 3.0e-3,
                                                                                                     1.6e-4)


def _stack_cases():
    out = {}
    out["default_fits"] = pb.stack_up(available_mm=480.0)
    out["tight"] = pb.stack_up(available_mm=out["default_fits"].installed_mm + 5.0)
    out["edge_tight_band"] = pb.stack_up(available_mm=out["default_fits"].installed_mm + 10.0)
    out["exact"] = pb.stack_up(available_mm=out["default_fits"].installed_mm)
    out["over"] = pb.stack_up(available_mm=300.0, mc_family="compact racing (short body)", tilt_deg=8.0,
                              pushrod_engaged_mm=13.0, pushrod_thread_dia_mm=8.0)
    out["unknown_family"] = pb.stack_up(available_mm=400.0, mc_family="nope", mc_outlet="??", tilt_deg=-5.0)
    out["measured"] = pb.stack_up(available_mm=420.0, mc_body_mm=96.5, mc_body_measured=True, mc_outlet_mm=15.0,
                                  pedal_lever_mm=100.0, pedal_ratio=0.0, pedal_rest_angle_deg=35.0,
                                  balance_bar_stack_mm=30.0, pushrod_mm=50.0, pushrod_engaged_mm=None,
                                  mount_plate_mm=6.0, bulkhead_clearance_mm=10.0, pedal_arc_clearance_mm=8.0,
                                  tight_band_mm=4.0)
    out["unmeasured_given"] = pb.stack_up(available_mm=420.0, mc_body_mm=100.0, mc_outlet="90 deg banjo")
    out["adjust_edge"] = pb.stack_up(available_mm=420.0, pushrod_engaged_mm=15.0, pushrod_thread_dia_mm=8.0)
    return out


def _stack_view(s):
    return dict(r=s, fits=s.fits, waterfall=s.waterfall(), as_dict=s.as_dict(), summary=s.summary())


def test_the_stack_up():
    check("pedal_box_stack", {k: _stack_view(v) for k, v in _stack_cases().items()})


def test_shortening():
    st = _stack_cases()
    over = st["over"]
    res = {}
    res["opts_default"] = pb.shorten_options(over)
    res["opts_custom"] = pb.shorten_options(over, pedal_lever_mm=110.0, pedal_ratio=5.5, pedal_ratio_max=5.0,
                                            pedal_rest_angle_deg=30.0, pedal_rest_angle_max_deg=30.0,
                                            mc_bore_mm=17.5, mc_bore_up_mm=17.0, mc_family="compact racing (short body)",
                                            mc_family_short="long stroke / large reservoir",
                                            mc_outlet="90 deg banjo", mc_outlet_alt="45 deg fitting",
                                            tilt_deg=5.0, tilt_max_deg=15.0)
    res["opts_alt_missing"] = pb.shorten_options(over, mc_outlet="bulkhead elbow at the MC face",
                                                 mc_outlet_alt="nope", tilt_max_deg=0.0, mc_family="?")
    res["opts_already_shortest"] = pb.shorten_options(over, mc_outlet="bulkhead elbow at the MC face",
                                                      mc_outlet_alt="90 deg fitting", mc_bore_up_mm=20.0)
    res["opts_no_pushrod"] = pb.shorten_options(pb.StackUpResult([], 0.0, 0.0, 10.0, "DOES NOT FIT", False))
    res["opts_tilt_sane_edge"] = pb.shorten_options(over, tilt_max_deg=12.0)
    for k in list(res):
        res[k] = [dict(o=o, label=o.label()) for o in res[k]]
    plans = {}
    plans["plan_over"] = pb.plan_shortening(over)
    plans["plan_rank3"] = pb.plan_shortening(over, max_cost_rank=3)
    plans["plan_rank0"] = pb.plan_shortening(over, max_cost_rank=0)
    plans["plan_fits"] = pb.plan_shortening(st["default_fits"])
    plans["plan_kw"] = pb.plan_shortening(over, pedal_ratio_max=8.0, mc_bore_mm=19.0)
    opts = [pb.ShortenOption("tiny", 0.1, 0, "c"), pb.ShortenOption("big", 30.0, 1, "c", {"k": "v"}),
            pb.ShortenOption("infeasible", 50.0, 0, "x", feasible=False), pb.ShortenOption("dear", 50.0, 3, "c"),
            pb.ShortenOption("more", 200.0, 2, "c"), pb.ShortenOption("after", 5.0, 2, "c")]
    plans["plan_custom"] = pb.plan_shortening(over, opts)
    plans["plan_exact"] = pb.plan_shortening(pb.StackUpResult([], 0, 0, 30.0, "DOES NOT FIT", False),
                                             [pb.ShortenOption("a", 30.0, 0, "c"), pb.ShortenOption("b", 1.0, 0, "c")])
    plans["plan_unsolved"] = pb.plan_shortening(pb.StackUpResult([], 0, 0, 30.0, "DOES NOT FIT", True),
                                                [pb.ShortenOption("a", 10.0, 0, "c"), pb.ShortenOption("b", 0.11, 0, "c")])
    for k, p in plans.items():
        res[k] = dict(p=p, as_dict=p.as_dict(), summary=p.summary())
    check("pedal_box_shorten", res)


def test_bias_and_authority():
    res = {}
    for e in (0.0, 6.0, -12.0, 25.0, 30.0, -30.0, 40.0, 10.4, 10.6):
        b = pb.balance_bar_bias(pedal_force_N=450.0, pedal_ratio=5.2, front=F, rear=R, bar_offset_mm=e)
        res[f"bar_{e}"] = dict(b=b, d=b.as_dict())
    res["bar_short"] = pb.balance_bar_bias(pedal_force_N=450.0, pedal_ratio=5.2, front=F, rear=R, bar_length_mm=0.0)
    for k, kw in {"auth_default": {}, "auth_target": dict(target_bias=0.6),
                  "auth_target_hi": dict(target_bias=0.99), "auth_target_lo": dict(target_bias=0.05),
                  "auth_near_end": dict(target_bias=None, max_offset_mm=20.0, n=2, thread_pitch_mm=5.0),
                  "auth_custom": dict(bar_length_mm=80.0, max_offset_mm=15.0, n=7, thread_pitch_mm=1.25)}.items():
        a = pb.bias_authority(pedal_force_N=500.0, pedal_ratio=5.0, front=F, rear=R, **kw)
        res[k] = dict(a=a, d=a.as_dict(), s=a.summary())
    a = pb.bias_authority(pedal_force_N=500.0, pedal_ratio=5.0, front=F, rear=R, n=9)
    edge = pb.bias_authority(pedal_force_N=500.0, pedal_ratio=5.0, front=F, rear=R, n=9, target_bias=a.bias_max)
    far = pb.bias_authority(pedal_force_N=500.0, pedal_ratio=5.0, front=F, rear=R, n=9,
                            target_bias=0.5 * (a.bias_max + a.sweep[-2]["bias_front"]))
    res["auth_edge"] = dict(a=edge, s=edge.summary())
    res["auth_far"] = dict(a=far, s=far.summary())
    check("pedal_box_bias", res)


def test_pedal_travel():
    res = {}
    cases = {
        "front_default": dict(circuit=F, line_pressure_bar=60.0, pedal_ratio=5.0),
        "rear_air": dict(circuit=R, line_pressure_bar=45.0, pedal_ratio=5.0,
                         params=pb.TravelParams(air_cc=0.3, compliance_scale=1.4, calibrated=True, fitted_to="x")),
        "fail_both": dict(circuit=pb.CircuitSpec(12.0, 30.0, pistons_per_side=3, n_corners=2), line_pressure_bar=90.0,
                          pedal_ratio=6.0, available_travel_mm=30.0, mc_stroke_limit_mm=10.0),
        "fail_travel": dict(circuit=F, line_pressure_bar=60.0, pedal_ratio=9.0, available_travel_mm=20.0),
        "fail_stroke": dict(circuit=F, line_pressure_bar=60.0, pedal_ratio=1.0, mc_stroke_limit_mm=2.0,
                            available_travel_mm=500.0),
        "tight_travel": dict(circuit=F, line_pressure_bar=60.0, pedal_ratio=5.0, available_travel_mm=40.0),
        "tight_stroke": dict(circuit=F, line_pressure_bar=60.0, pedal_ratio=1.0, mc_stroke_limit_mm=3.2,
                             available_travel_mm=500.0, tight_frac=0.5),
        "negative_p": dict(circuit=F, line_pressure_bar=-5.0, pedal_ratio=5.0, hose_length_m=1.0,
                           hardline_length_m=2.5, line_inner_dia_mm=4.0),
        "zero_corners": dict(circuit=pb.CircuitSpec(15.0, 25.0, n_corners=0), line_pressure_bar=30.0, pedal_ratio=4.0),
    }
    for k, kw in cases.items():
        t = pb.pedal_travel(**kw)
        res[k] = dict(t=t, d=t.as_dict(), s=t.summary(), big=t.biggest(), big2=t.biggest(2))
    base = pb.pedal_travel(circuit=F, line_pressure_bar=60.0, pedal_ratio=5.0)
    for k, tgt in (("cal_fit", base.pedal_travel_mm * 1.2), ("cal_short", base.pedal_travel_mm * 0.9)):
        p = pb.calibrate_travel_params(measured_pedal_travel_mm=tgt, circuit=F, line_pressure_bar=60.0, pedal_ratio=5.0,
                                       base=pb.TravelParams(compliance_scale=1.1, air_cc=0.1), fitted_to="rig",
                                       hose_length_m=0.9)
        res[k] = p
    res["cal_default"] = pb.TravelParams.from_measured_stroke(measured_pedal_travel_mm=40.0, circuit=F,
                                                              line_pressure_bar=50.0, pedal_ratio=5.0)
    for bad in (7.5, 3.0):
        with pytest.raises(ValueError) as e:
            pb.calibrate_travel_params(measured_pedal_travel_mm=bad, circuit=F, line_pressure_bar=50.0,
                                       pedal_ratio=5.0)
        res[f"cal_err_{bad}"] = str(e.value)
    check("pedal_box_travel", res)


def test_the_study():
    res = {}
    for k, kw in {"fits": dict(available_mm=480.0),
                  "over": dict(available_mm=250.0, target_bias=0.6, pedal_ratio=5.5,
                               travel_kwargs=dict(hose_length_m=1.2), available_travel_mm=30.0),
                  "unreachable": dict(available_mm=480.0, target_bias=0.99, stack_kwargs=dict(tilt_deg=4.0)),
                  "no_target": dict(available_mm=480.0, target_bias=None, bar_offset_mm=5.0,
                                    travel_params=pb.TravelParams(calibrated=True), mc_stroke_limit_mm=3.0),
                  "mismatch": dict(available_mm=400.0, front=pb.CircuitSpec(12.0, 32.0, pistons_per_side=3),
                                   rear=pb.CircuitSpec(25.4, 20.0, pistons_per_side=1, opposed=False))}.items():
        kw.setdefault("front", F)
        kw.setdefault("rear", R)
        s = pb.study(**kw)
        res[k] = dict(verdict=s.verdict, d=s.as_dict(), s=s.summary(), f=s.findings)
    res["provenance"] = pb.provenance()
    res["tables"] = dict(bulk=pb.FLUID_BULK_MODULUS_PA, hose=pb.HOSE_EXPANSION_CC_PER_M_PER_BAR,
                         fit=pb.FITTING_STACK_MM, fam=pb.MC_FAMILIES,
                         k=[pb.MIN_THREAD_ENGAGEMENT_D, pb.PUSHROD_SPHERICAL_REQUIRED_DEG, pb.PUSHROD_MAX_SANE_DEG])
    seg = pb.StackSegment("x", 1.0)
    res["defaults"] = dict(seg=seg, opt=pb.ShortenOption("o", 1.0, 1, "c"), cs=pb.CircuitSpec(1.0, 2.0),
                           tp=pb.TravelParams(), vi=pb.VolumeItem("v", 1.0, True).as_dict())
    check("pedal_box_study", res)


def test_edges():
    """Cases placed exactly on (or just either side of) each threshold, and non-default inputs everywhere a value is
    passed through, so a dropped argument or a flipped comparison changes the recorded answer."""
    res = {}
    base = pb.pedal_travel(circuit=F, line_pressure_bar=60.0, pedal_ratio=5.0)
    tv, st = base.pedal_travel_mm, base.mc_stroke_mm
    tcases = {"avail_eq": dict(available_travel_mm=tv), "avail_tight_eq": dict(available_travel_mm=2.0 * tv, tight_frac=0.5),
              "tight_default": dict(available_travel_mm=tv / 0.9), "stroke_eq": dict(mc_stroke_limit_mm=st),
              "stroke_1p5": dict(mc_stroke_limit_mm=st / 1.5, available_travel_mm=500.0),
              "stroke_tight_eq": dict(mc_stroke_limit_mm=2.0 * st, tight_frac=0.5, available_travel_mm=500.0),
              "stroke_tight": dict(mc_stroke_limit_mm=st / 0.9, available_travel_mm=500.0),
              "scale2": dict(params=pb.TravelParams(compliance_scale=2.0)),
              "scale_half": dict(params=pb.TravelParams(compliance_scale=0.5))}
    for k, kw in tcases.items():
        t = pb.pedal_travel(circuit=F, line_pressure_bar=60.0, pedal_ratio=5.0, **kw)
        res["t_" + k] = dict(t=t, s=t.summary())
    odd = pb.TravelParams(knockback_mm=0.3, pad_compression_mm=0.2, caliper_cc_per_bar=3e-3, hose_type="rubber hose",
                          hardline_type="hardline (Cu-Ni tube)", fluid="DOT 5 (silicone)", air_cc=0.2, free_play_mm=2.5,
                          compliance_scale=1.2)
    tgt = pb.pedal_travel(circuit=F, line_pressure_bar=60.0, pedal_ratio=5.0, params=odd).pedal_travel_mm * 1.15
    res["cal_odd"] = pb.calibrate_travel_params(measured_pedal_travel_mm=tgt, circuit=F, line_pressure_bar=60.0,
                                                pedal_ratio=5.0, base=odd)
    res["axle1"] = pb.CircuitSpec(15.0, 25.0, n_corners=1).axle_torque_Nm(50.0)
    fits = pb.stack_up(available_mm=480.0)
    inst = fits.installed_mm
    scases = {"deficit_half": dict(available_mm=inst - 0.5), "tilt_half": dict(available_mm=480.0, tilt_deg=0.5),
              "short_engaged": dict(available_mm=480.0, pushrod_engaged_mm=5.0),
              "body_none_outlet": dict(available_mm=480.0, mc_outlet_mm=14.0),
              "body_unmeasured_outlet": dict(available_mm=480.0, mc_body_mm=99.0, mc_outlet_mm=14.0),
              "body_measured_outlet": dict(available_mm=480.0, mc_body_mm=99.0, mc_body_measured=True, mc_outlet_mm=14.0)}
    for k, kw in scases.items():
        s = pb.stack_up(**kw)
        res["s_" + k] = dict(r=s, d=s.as_dict(), s=s.summary(), w=s.waterfall())
        p = pb.plan_shortening(s)
        res["p_" + k] = dict(d=p.as_dict(), s=p.summary())
    res["plan_half"] = pb.plan_shortening(pb.StackUpResult([], 0, 0, 10.5, "DOES NOT FIT", False),
                                          [pb.ShortenOption("a", 10.0, 0, "c")]).as_dict()
    res["plan_tiny"] = pb.plan_shortening(pb.StackUpResult([], 0, 0, 0.5, "TIGHT", False)).as_dict()
    over = pb.stack_up(available_mm=300.0, tilt_deg=8.0)
    ocases = {"eq_ratio": dict(pedal_ratio=5.0, pedal_ratio_max=5.0), "eq_bore": dict(mc_bore_mm=17.5, mc_bore_up_mm=17.5),
              "eq_outlet": dict(mc_outlet="90 deg banjo", mc_outlet_alt="90 deg banjo"),
              "eq_family": dict(mc_family="compact racing (short body)", mc_family_short="compact racing (short body)"),
              "bore_up_compact": dict(mc_family="compact racing (short body)", mc_bore_mm=15.0, mc_bore_up_mm=17.5),
              "unknown_outlet": dict(mc_outlet="??", mc_outlet_alt="45 deg fitting"),
              "big_tilt": dict(tilt_deg=0.0, tilt_max_deg=25.0)}
    for k, kw in ocases.items():
        res["o_" + k] = [dict(o=o, label=o.label()) for o in pb.shorten_options(over, **kw)]
    res["o_no_bar"] = [o.gain_mm for o in pb.shorten_options(pb.StackUpResult(
        [pb.StackSegment("Pushrod", 40.0)], 0.0, 0.0, 5.0, "DOES NOT FIT", False))]
    for e in (17.3, 21.7, 28.9, 9.0, 10.5):
        b = pb.balance_bar_bias(pedal_force_N=450.0, pedal_ratio=5.2, front=F, rear=R, bar_offset_mm=e)
        res[f"bar_{e}"] = b.as_dict()
    c = pb.bias_authority(pedal_force_N=500.0, pedal_ratio=5.0, front=F, rear=R, bar_length_mm=80.0, n=9)
    mid_bias = pb.balance_bar_bias(pedal_force_N=500.0, pedal_ratio=5.0, front=F, rear=R, bar_length_mm=80.0,
                                   bar_offset_mm=2.0).bias_front
    for k, kw in {"auth_l80_mid": dict(bar_length_mm=80.0, target_bias=mid_bias),
                  "auth_l80_end": dict(bar_length_mm=80.0, target_bias=c.sweep[-2]["bias_front"]),
                  "auth_l80_small": dict(bar_length_mm=80.0, max_offset_mm=12.0, target_bias=pb.balance_bar_bias(
                      pedal_force_N=500.0, pedal_ratio=5.0, front=F, rear=R, bar_length_mm=80.0,
                      bar_offset_mm=0.2).bias_front),
                  "auth_coarse": dict(thread_pitch_mm=20.0, target_bias=None),
                  "auth_coarse2": dict(thread_pitch_mm=13.0, bar_length_mm=45.0, target_bias=None)}.items():
        a = pb.bias_authority(pedal_force_N=500.0, pedal_ratio=5.0, front=F, rear=R, **kw)
        res[k] = dict(d=a.as_dict(), s=a.summary())
    scs = {"study_kw": dict(available_mm=300.0, pedal_lever_mm=110.0, bar_length_mm=75.0, target_bias=0.62),
           "study_half": dict(available_mm=inst - 0.5),
           "study_rear_long": dict(available_mm=400.0, front=pb.CircuitSpec(25.4, 20.0, pistons_per_side=1, opposed=False),
                                   rear=pb.CircuitSpec(12.0, 32.0, pistons_per_side=3)),
           "study_costly_pass": dict(available_mm=250.0, available_travel_mm=200.0, mc_stroke_limit_mm=60.0),
           "study_costly_tight": dict(available_mm=250.0, available_travel_mm=base.pedal_travel_mm * 1.15)}
    for k, kw in scs.items():
        kw.setdefault("front", F)
        kw.setdefault("rear", R)
        s = pb.study(**kw)
        res[k] = dict(v=s.verdict, d=s.as_dict(), s=s.summary())
    check("pedal_box_edges", res)
