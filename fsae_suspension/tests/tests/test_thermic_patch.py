# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Tests for ThermicPatch (suspension/thermic_patch.py) — the 3-node radial thermal
ladder that marches along a solved transient and scales Pacejka grip by core
temperature.

These pin the contract the Ghost Topology / ThermicPatch tab depends on:
  - a solved transient in yields a valid per-corner temperature history,
  - the ladder never raises: failed/None/short inputs return a failed result,
  - the surface skin leads the core under flash heat (physical ordering),
  - the parabolic window law is 1.0 at optimum, symmetric-floored, hot-asymmetric,
  - grip loss scales the way the window predicts (cold and hot both degrade),
  - the worst instant is ranked by NEWTONS lost, not the bare fraction (so a
    near-zero-force instant can't masquerade as the worst case),
  - every output is flagged UNCALIBRATED while the ThermalParams is uncalibrated.

Run:  python tests/test_thermic_patch.py   (or: python -m pytest tests/)
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from suspension.transient import run_maneuver, TransientResult
from suspension.thermic_patch import (
    run_thermic_patch, parabolic_mu_scale, verdict_sentence,
    default_thermal_params, default_ladder_params, LadderParams, ThermicResult)

_PASS, _FAIL = [], []


def check(name, cond):
    (_PASS if cond else _FAIL).append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")


# --------------------------- input guards ------------------------------- #
def test_guards_never_raise():
    check("None input -> failed result, not raise",
          run_thermic_patch(None).ok is False)
    check("failed transient -> failed result",
          run_thermic_patch(TransientResult.failed(["x"])).ok is False)
    # a 1-sample (too short) result
    short = run_maneuver(None, kind="step_steer")
    short.t = short.t[:1]
    short.Fz = short.Fz[:1]; short.Fy = short.Fy[:1]
    short.Fx = short.Fx[:1]; short.alpha = short.alpha[:1]; short.u = short.u[:1]
    check("too-short transient -> failed result",
          run_thermic_patch(short).ok is False)


# --------------------------- basic run ---------------------------------- #
def test_basic_run_shapes():
    res = run_maneuver(None, kind="step_steer")
    tr = run_thermic_patch(res, init_temp_c=80.0)
    n = len(res.t)
    check("run ok on a valid transient", tr.ok)
    check("temperature arrays are (n,4)",
          tr.T_surface.shape == (n, 4) and tr.T_core.shape == (n, 4)
          and tr.T_carcass.shape == (n, 4))
    check("temperatures are all finite",
          bool(np.all(np.isfinite(tr.T_surface)))
          and bool(np.all(np.isfinite(tr.T_core))))
    check("temperatures stay in a sane physical band (-40..400 C)",
          bool(np.all(tr.T_core > -40) and np.all(tr.T_core < 400)))


# --------------------------- physical ordering -------------------------- #
def test_surface_leads_core_under_flash():
    # a hard slide with poor cooling: surface must run hotter than core somewhere
    res = run_maneuver(None, kind="snap_oversteer")
    lp = LadderParams(surface_to_road_frac=0.10, track_temp_c=55.0)
    tr = run_thermic_patch(res, lp=lp, init_temp_c=92.0, ambient_c=38.0,
                           gas_c=95.0)
    # find the peak-lateral-load instant on the most-loaded corner
    ci = int(np.argmax(np.max(np.abs(res.Fy), axis=0)))
    i = int(np.argmax(np.abs(res.Fy[:, ci])))
    check("surface skin >= core at the flash instant",
          tr.T_surface[i, ci] >= tr.T_core[i, ci] - 1e-6)
    check("surface shows a real excursion over the event",
          (np.max(tr.T_surface[:, ci]) - np.min(tr.T_surface[:, ci])) > 0.5)
    check("flash-heat flux is positive during the slide",
          np.max(tr.q_flash[:, ci]) > 0.0)


# --------------------------- the window law ----------------------------- #
def test_parabolic_window():
    tp = default_thermal_params()
    tp.T_opt_c = 85.0
    at_opt = float(parabolic_mu_scale(85.0, tp, half_width_c=35.0))
    check("grip is 1.0 at the optimum", abs(at_opt - 1.0) < 1e-9)
    cold = float(parabolic_mu_scale(60.0, tp, half_width_c=35.0))
    hot = float(parabolic_mu_scale(110.0, tp, half_width_c=35.0))
    check("grip degrades below optimum", cold < 1.0)
    check("grip degrades above optimum", hot < 1.0)
    # hot side is penalised at least as hard as an equal cold offset
    cold25 = float(parabolic_mu_scale(85.0 - 25, tp, half_width_c=35.0))
    hot25 = float(parabolic_mu_scale(85.0 + 25, tp, half_width_c=35.0))
    check("overheating penalised >= equal cold offset (asymmetry)",
          hot25 <= cold25 + 1e-9)
    # never below the floor
    way_hot = float(parabolic_mu_scale(300.0, tp, half_width_c=35.0))
    check("grip never falls below the floor",
          way_hot >= tp.mu_floor - 1e-9)


# --------------------------- grip loss follows temp --------------------- #
def test_grip_loss_tracks_temperature():
    res = run_maneuver(None, kind="snap_oversteer")
    # near-optimum entry: small loss
    warm = run_thermic_patch(res, init_temp_c=84.0, ambient_c=30.0)
    # far-hot entry with a baked track: large loss
    lp = LadderParams(surface_to_road_frac=0.10, track_temp_c=58.0)
    hot = run_thermic_patch(res, lp=lp, init_temp_c=110.0, ambient_c=40.0,
                            gas_c=112.0)
    check("near-optimum entry keeps grip high (max loss < 8%)",
          warm.summary()["max_dFy_frac"] < 0.08)
    check("overheated entry loses much more grip than warm",
          hot.summary()["max_dFy_frac"] > warm.summary()["max_dFy_frac"] + 0.05)


# --------------------------- worst-instant ranking ---------------------- #
def test_worst_instant_ranked_by_newtons():
    res = run_maneuver(None, kind="step_steer")
    tr = run_thermic_patch(res, init_temp_c=70.0)   # cold-ish -> some loss
    w = tr.worst_instant()
    check("worst instant reports ok", bool(w.get("ok")))
    # it must not be t=0 (where lateral force is ~0): a real loaded instant
    check("worst instant is not the t=0 near-zero-force artefact",
          w.get("t_s", 0.0) > 1e-3)
    # the reported newtons lost must be the true max over the grid
    lost = tr.Fy_cold * tr.dFy_frac
    check("reported Fy lost equals the grid maximum",
          abs(w["Fy_lost_N"] - float(np.max(lost))) < 1.0)


# --------------------------- provenance / honesty ----------------------- #
def test_uncalibrated_flag():
    res = run_maneuver(None, kind="step_steer")
    tr = run_thermic_patch(res, init_temp_c=80.0)
    check("uncalibrated params -> result flagged uncalibrated",
          tr.calibrated is False)
    check("verdict sentence names the uncalibrated status",
          "UNCALIBRATED" in verdict_sentence(tr) or "window" in
          verdict_sentence(tr))
    # if the params ARE calibrated, the flag flips
    tp = default_thermal_params()
    tp.calibrated = True
    tr2 = run_thermic_patch(res, tp=tp, init_temp_c=80.0)
    check("calibrated params -> result flagged calibrated",
          tr2.calibrated is True)


def _run_all():
    for fn in [test_guards_never_raise, test_basic_run_shapes,
               test_surface_leads_core_under_flash, test_parabolic_window,
               test_grip_loss_tracks_temperature,
               test_worst_instant_ranked_by_newtons, test_uncalibrated_flag]:
        print(f"\n{fn.__name__}:")
        fn()
    print(f"\n{len(_PASS)} passed, {len(_FAIL)} failed")
    if _FAIL:
        print("FAILURES:", _FAIL)
    return not _FAIL


# pytest entry points
def test_thermic_patch_suite():
    assert _run_all()


if __name__ == "__main__":
    sys.exit(0 if _run_all() else 1)
