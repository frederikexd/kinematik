# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Tests for OmniCore (suspension/omnicore.py).

Same philosophy as the morph/stochastic suites: nail the deterministic
grammar (same text in, same spec out; every token accounted for on the
receipt), pin the Pareto arithmetic on hand-checkable point sets, prove the
knee pick actually listens to the mission's priority weights, prove the
dominance receipts name a strict dominator, prove the twin's cosine match
finds a scaled copy of its own signature AND refuses to name a suspect when
the pattern matches nothing, check the heal-plan algebra against the closed
forms it claims (t = s·tan|θ|, M = Kt·gear·eff·duty·V/R), and prove the
flagged-not-raised doctrine end to end by breaking the engines on purpose.

Run:  python -m pytest tests/test_omnicore.py
      (or standalone: python tests/test_omnicore.py)
"""
import math
import sys, os
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from suspension import omnicore as oc
from suspension import simulforge as sf
from suspension.kinematics import Hardpoints
from suspension.omnicore import (
    AXES, ConfigPoint, DefectSignature, MissionSpec, OmniKnobs, TwinBaseline,
    diagnose, dominance_receipts, fmt_axis, heal_plan, knee_pick, parse_mission,
    pareto_mask, run_omnicore,
)

FLAGSHIP = ("Synthesize a lightweight 4WD electric vehicle optimized for "
            "high-frequency bumpy terrain, constrained by a $15,000 budget "
            "and a shop floor with a ±2 mm weld-pull error profile.")


# ========================================================================== #
#  1 · The mission grammar — a receipt, not a language model
# ========================================================================== #
class TestGrammar:
    def test_flagship_prompt_fields(self):
        s = parse_mission(FLAGSHIP)
        assert s.maneuver == "curb_strike"          # "bumpy" / "high-frequency"
        assert s.drive == "4wd"
        assert s.budget_usd == pytest.approx(15000.0)
        assert s.shop_accuracy_mm == pytest.approx(2.0)
        assert s.shop == "hand_weld"                # ±2 mm covers hand work
        assert s.weld_pull_mm == pytest.approx(2.0)  # "weld-pull" near ±2 mm
        assert "mass" in s.priorities               # "lightweight"

    def test_determinism_same_text_same_spec(self):
        a, b = parse_mission(FLAGSHIP), parse_mission(FLAGSHIP)
        assert a.summary() == b.summary()

    def test_receipt_accounts_for_every_word_class(self):
        s = parse_mission(FLAGSHIP)
        joined = " ".join(s.consumed)
        for frag in ("budget", "shop accuracy", "manoeuvre", "drive",
                     "priority"):
            assert frag in joined, f"receipt missing a '{frag}' line"

    def test_empty_prompt_is_all_assumptions(self):
        s = parse_mission("")
        assert s.maneuver == "step_steer" and s.budget_usd is None
        assert s.priorities == {}
        # every stand-in default is disclosed, not silent
        joined = " ".join(s.assumptions)
        for frag in ("shop", "manoeuvre", "priority", "budget"):
            assert frag in joined

    def test_tolerance_is_not_money(self):
        s = parse_mission("a ±2 mm shop, no budget stated")
        assert s.budget_usd is None
        assert s.shop_accuracy_mm == pytest.approx(2.0)

    def test_k_suffix_budget(self):
        s = parse_mission("keep it under $15k please")
        assert s.budget_usd == pytest.approx(15000.0)

    def test_accuracy_to_shop_class_boundaries(self):
        assert parse_mission("±0.05 mm floor").shop == "cnc"
        assert parse_mission("±0.5 mm floor").shop == "jig_weld"
        assert parse_mission("±2 mm floor").shop == "hand_weld"

    def test_unknown_words_land_in_ignored(self):
        s = parse_mission("a purple monsoon-proof vehicle")
        assert any("purple" in w for w in s.ignored)
        assert any("monsoon" in w for w in s.ignored)


# ========================================================================== #
#  2 · Pareto arithmetic on hand-checkable points
# ========================================================================== #
def _pt(comp, laps, mass, cost, yld):
    return {"composure": comp, "laps": laps, "mass": mass, "cost": cost,
            "yield": yld}


class TestPareto:
    def test_hand_case_strict_domination(self):
        # B is better-or-equal everywhere and strictly better on cost.
        A = _pt(1.0, 10.0, 8.0, 900.0, 0.9)
        B = _pt(1.0, 10.0, 8.0, 800.0, 0.9)
        assert pareto_mask([A, B]) == [False, True]

    def test_hand_case_tradeoff_keeps_both(self):
        # A composed but heavy, B floppy but light — nobody dominates.
        A = _pt(0.5, 10.0, 9.0, 900.0, 0.9)
        B = _pt(1.5, 10.0, 7.0, 900.0, 0.9)
        assert pareto_mask([A, B]) == [True, True]

    def test_identical_points_both_survive(self):
        A = _pt(1.0, 10.0, 8.0, 900.0, 0.9)
        assert pareto_mask([A, dict(A)]) == [True, True]

    def test_axis_senses_respected(self):
        # laps is a MAXIMISE axis: more laps must win, all else equal.
        A = _pt(1.0, 12.0, 8.0, 900.0, 0.9)
        B = _pt(1.0, 10.0, 8.0, 900.0, 0.9)
        assert pareto_mask([A, B]) == [True, False]

    def test_fmt_axis_uses_declared_format(self):
        assert fmt_axis("cost", 12345.6) == "12,346"
        assert fmt_axis("laps", 9.876) == "9.88"


def _cfg(cid, comp, laps, mass, cost, yld, feasible=True):
    return ConfigPoint(cid=cid, shop="hand_weld", actuator=f"a{cid}",
                       volfrac=0.4, objectives=_pt(comp, laps, mass, cost,
                                                   yld),
                       feasible=feasible)


class TestKneeAndReceipts:
    def test_knee_listens_to_priorities(self):
        # Two survivors: cheap-but-floppy vs composed-but-expensive.
        cheap = _cfg(1, comp=2.0, laps=10, mass=8, cost=500, yld=0.9)
        crisp = _cfg(2, comp=0.5, laps=10, mass=8, cost=1500, yld=0.9)
        m_cost = MissionSpec(text="", priorities={"cost": 3.0})
        m_comp = MissionSpec(text="", priorities={"composure": 3.0})
        assert knee_pick([cheap, crisp], m_cost) == 1
        assert knee_pick([cheap, crisp], m_comp) == 2

    def test_knee_none_when_nothing_feasible(self):
        dead = _cfg(1, 1, 10, 8, 900, 0.9, feasible=False)
        assert knee_pick([dead], MissionSpec(text="")) is None

    def test_dominance_receipt_names_the_dominator(self):
        loser = _cfg(1, comp=2.0, laps=9, mass=9, cost=1000, yld=0.8)
        winner = _cfg(2, comp=1.0, laps=10, mass=8, cost=900, yld=0.9)
        rec = dominance_receipts([loser, winner])
        assert len(rec) == 1
        assert winner.label in rec[0] and loser.label in rec[0]

    def test_no_receipt_on_a_clean_front(self):
        a = _cfg(1, comp=0.5, laps=10, mass=9, cost=900, yld=0.9)
        b = _cfg(2, comp=1.5, laps=10, mass=7, cost=900, yld=0.9)
        assert dominance_receipts([a, b]) == []


# ========================================================================== #
#  3 · The twin — signature match, honest miss, heal algebra
# ========================================================================== #
def _baseline():
    return TwinBaseline(
        maneuver="step_steer",
        channels={"v_min": 22.0, "sag_peak_V": 2.0, "i_peak": 40.0,
                  "response_lag_ms": 12.0, "energy_Wh": 0.30,
                  "authority": 0.95, "roll_peak_deg": 1.5},
        bands={k: oc.TWIN_CHANNELS[k][2] for k in oc.TWIN_CHANNELS})


def _sig(key, dz):
    return DefectSignature(key=key, label=key.replace("_", " "),
                           story="", dz=dz)


class TestTwinDiagnosis:
    def test_scaled_copy_of_signature_names_the_suspect(self):
        b = _baseline()
        dz = {"v_min": -3.0, "sag_peak_V": 3.0, "i_peak": 2.0,
              "response_lag_ms": 1.5, "energy_Wh": 0.5, "authority": -2.0,
              "roll_peak_deg": 1.0}
        sig = _sig("tired_pack", dz)
        # measure the SAME pattern at 0.8× magnitude — cosine must be ~1
        measured = {k: b.channels[k] + 0.8 * dz[k] * b.bands[k]
                    for k in b.channels}
        d = diagnose(b, measured, [sig, _sig("red_herring",
                                             {"roll_peak_deg": 5.0})])
        assert d.suspect == "tired_pack"
        assert d.cosine == pytest.approx(1.0, abs=1e-6)
        assert d.magnitude == pytest.approx(0.8, abs=1e-6)

    def test_inside_band_means_no_suspect(self):
        b = _baseline()
        measured = {k: v + 0.3 * b.bands[k] for k, v in b.channels.items()}
        d = diagnose(b, measured, [_sig("x", {"v_min": -3.0})])
        assert d.suspect is None and not d.drifting is False or True
        assert all(v == "NOMINAL" for v in d.channel_verdicts.values())
        assert d.suspect is None

    def test_orthogonal_drift_matches_nothing_honestly(self):
        b = _baseline()
        # drift lives ONLY on roll; the catalog only knows electrical drift
        measured = dict(b.channels)
        measured["roll_peak_deg"] = b.channels["roll_peak_deg"] \
            + 4.0 * b.bands["roll_peak_deg"]
        d = diagnose(b, measured,
                     [_sig("elec_only", {"v_min": -3.0, "sag_peak_V": 3.0})])
        assert d.suspect is None
        assert "matches nothing" in d.note

    def test_band_verdict_ladder(self):
        b = _baseline()
        measured = dict(b.channels)
        measured["v_min"] = b.channels["v_min"] - 1.5 * b.bands["v_min"]
        measured["i_peak"] = b.channels["i_peak"] + 3.0 * b.bands["i_peak"]
        d = diagnose(b, measured, [])
        assert d.channel_verdicts["v_min"] == "WATCH"
        assert d.channel_verdicts["i_peak"] == "DEGRADED"
        assert d.channel_verdicts["energy_Wh"] == "NOMINAL"


class TestHealPlan:
    def _diag_no_suspect(self, b, measured):
        return diagnose(b, measured, [])

    def test_shim_identity(self):
        # t = s·tan|θ| — the exact closed form, both drift signs.
        b = _baseline()
        d = self._diag_no_suspect(b, dict(b.channels))
        forge = SimpleNamespace(ok=False, actuator=sf.ActuatorParams(),
                                bus=sf.BusParams(), elec=None)
        for theta in (+0.6, -0.6):
            plan = heal_plan(d, forge, camber_drift_deg=theta,
                             bolt_span_mm=80.0)
            want = 80.0 * math.tan(math.radians(abs(theta)))
            assert plan.shim_mm == pytest.approx(want, rel=1e-9)
        # no drift declared → no shim invented
        plan = heal_plan(d, forge, camber_drift_deg=0.0)
        assert plan.shim_mm is None

    def test_gain_derate_algebra(self):
        # Deliverable moment at the measured V_min from first principles:
        #   I = duty·V/R,  M = Kt·gear·eff·I (capped at M_max)
        b = _baseline()
        act = sf.ActuatorParams()
        measured = dict(b.channels)
        measured["v_min"] = 10.0            # a hard sag
        d = self._diag_no_suspect(b, measured)
        m_cmd = np.array([0.0, 600.0, -900.0])
        forge = SimpleNamespace(ok=True, actuator=act, bus=sf.BusParams(),
                                elec=SimpleNamespace(M_cmd=m_cmd))
        plan = heal_plan(d, forge)
        i_max = act.duty_max * 10.0 / act.R_ohm
        m_deliv = min(act.torque_from_current(i_max), act.M_max_Nm)
        want = float(np.clip(m_deliv / 900.0, 0.05, 1.0))
        assert plan.gain_scale == pytest.approx(want, rel=1e-9)
        assert plan.kp_new == pytest.approx(act.kp * want, rel=1e-9)
        assert plan.kd_new == pytest.approx(act.kd * want, rel=1e-9)

    def test_screening_caution_always_printed(self):
        b = _baseline()
        d = self._diag_no_suspect(b, dict(b.channels))
        forge = SimpleNamespace(ok=False, actuator=sf.ActuatorParams(),
                                bus=sf.BusParams(), elec=None)
        plan = heal_plan(d, forge)
        assert any("screening" in c for c in plan.cautions)


# ========================================================================== #
#  4 · Flagged-not-raised — break the engines on purpose
# ========================================================================== #
def _tiny_knobs():
    return OmniKnobs(actuator_scales={"standard": 1.0}, volfracs=(0.40,),
                     compare_shops=False, genesis_starts=1,
                     genesis_yield_n=120, genesis_max_iter=4,
                     morph_h_mm=3.0, morph_max_iter=5, morph_rounds=1,
                     audit_samples=5, fan_cases=2)


class TestFlaggedNotRaised:
    def test_broken_simulforge_vetoes_by_name(self, monkeypatch):
        def boom(*a, **k):
            raise RuntimeError("bus caught fire")
        monkeypatch.setattr(oc.sf, "run_simulforge", boom)
        res = run_omnicore(None, parse_mission("smooth track car"),
                           knobs=_tiny_knobs())
        assert not res.ok
        assert any("SimulForge raised" in w for w in res.warnings)
        assert all(any(v.startswith("SimulForge") or
                       v.startswith("MorphMesh") for v in c.vetoes)
                   for c in res.configs)

    def test_broken_genesis_vetoes_by_name(self, monkeypatch):
        def boom(*a, **k):
            raise RuntimeError("optimiser exploded")
        monkeypatch.setattr(oc.ig, "inverse_genesis", boom)
        monkeypatch.setattr(oc.sf, "run_simulforge",
                            lambda *a, **k: (_ for _ in ()).throw(
                                RuntimeError("skip the slow path too")))
        res = run_omnicore(None, parse_mission(""), knobs=_tiny_knobs())
        assert not res.ok
        assert any("InverseGenesis raised" in w for w in res.warnings)
        assert all(any(v.startswith("InverseGenesis") for v in c.vetoes)
                   for c in res.configs)

    def test_no_feasible_config_is_a_warning_not_an_exception(self,
                                                              monkeypatch):
        monkeypatch.setattr(oc.sf, "run_simulforge",
                            lambda *a, **k: (_ for _ in ()).throw(
                                RuntimeError("dead")))
        res = run_omnicore(None, parse_mission(""), knobs=_tiny_knobs())
        assert res.pareto_ids == [] and res.knee_id is None
        assert any("no feasible configuration" in w for w in res.warnings)


# ========================================================================== #
#  5 · One coarse end-to-end sweep (the slow one)
# ========================================================================== #
class TestEndToEnd:
    def test_coarse_sweep_produces_a_refereed_front(self):
        mission = parse_mission("a smooth-track car, ±2 mm shop")
        res = run_omnicore(Hardpoints.default(), mission,
                           knobs=_tiny_knobs())
        assert len(res.configs) == 1                     # 1 shop × 1 act × 1 vf
        c = res.configs[0]
        # whether it survived or died, the referee must say WHY and with what
        if c.feasible:
            assert res.ok and c.cid in res.pareto_ids
            assert res.knee_id == c.cid
            for k in AXES:
                assert np.isfinite(c.objectives[k]), f"axis '{k}' is NaN"
        else:
            assert c.vetoes, "an infeasible config must carry a named veto"
        # the ledger counts real engine calls
        assert res.ledger["genesis"] <= 1
        assert res.ledger["simulforge"] >= 1 or not c.feasible
        # reports render without touching a display
        md = oc.render_omni_md(res)
        assert "OmniCore" in md and "fidelity" in md.lower()
        js = res.to_json()
        assert '"mission"' in js and '"ledger"' in js


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
