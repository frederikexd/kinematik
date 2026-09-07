# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Tests for MorphMesh (suspension/morphmesh.py).

Same philosophy as the ghost/stochastic suites: nail the closed forms the
engine stands on (the Q4 element's rigid-body null space and symmetry), pin
the fabrication audit as a MEASUREMENT (thick bars pass, thin ribs fail,
edge-flush weld feet are not falsely eaten, the HAZ disk is stricter), prove
the load fan reads the ghost instants faithfully (direction from the DEFORMED
link line, wrap-safe angles, a sign reversal keeps its own arrow), prove the
reject-and-coarsen contract fires and prices its premium, and prove the
flagged-not-raised doctrine end to end.

Run:  python -m pytest tests/test_morphmesh.py
      (or standalone: python tests/test_morphmesh.py)
"""
import json
import math
import sys, os
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from suspension.kinematics import Hardpoints
from suspension import morphmesh as mm
from suspension.morphmesh import (
    FabricationLimits, LoadCase, PlateDomain, VERDICTS, _PlateFE,
    _q4_ke_and_b, fabrication_audit, load_fan_from_audit, morph_component,
    morph_from_audit, render_morph_md,
)


HP = Hardpoints.default()


# --------------------------------------------------------------------------- #
#  helpers
# --------------------------------------------------------------------------- #
def _tiny_domain(**kw) -> PlateDomain:
    base = dict(width_mm=40.0, height_mm=50.0, h_mm=2.0, thickness_mm=4.0)
    base.update(kw)
    return PlateDomain.chassis_tab(**base)


def _one_case(angle_deg=-90.0, F=2000.0, w=1.0) -> LoadCase:
    a = math.radians(angle_deg)
    return LoadCase(np.array([math.cos(a), math.sin(a)]), F, w, 5, 0.0, 0.5,
                    angle_deg)


def _fake_instant(t, T, outer):
    """A GhostInstant stand-in carrying exactly what the fan reader uses."""
    ghost = SimpleNamespace(upper_outer=None, lower_outer=np.asarray(outer, float),
                            tie_rod_outer=None, pushrod_outer=None)
    return SimpleNamespace(t=t, ghost=ghost,
                           margins={"LF": {"force_N": T}},
                           load_path_shift={"LF": {"ghost_N": T}})


def _fake_audit(instants):
    return SimpleNamespace(instants=instants)


# --------------------------------------------------------------------------- #
#  1 · the element — symmetry and rigid-body null space
# --------------------------------------------------------------------------- #
def test_q4_element_symmetric_and_rigid_body_null():
    KE, B0, D = _q4_ke_and_b(0.3, 2.0)
    assert np.allclose(KE, KE.T, atol=1e-12)
    # rigid translations produce zero nodal force
    ux = np.array([1, 0, 1, 0, 1, 0, 1, 0], float)
    uy = np.array([0, 1, 0, 1, 0, 1, 0, 1], float)
    assert np.linalg.norm(KE @ ux) < 1e-10
    assert np.linalg.norm(KE @ uy) < 1e-10
    # 3 zero eigenvalues (2 translations + 1 rotation), rest positive
    w = np.linalg.eigvalsh(KE)
    assert np.sum(w < 1e-10) == 3
    assert np.all(w[3:] > 0)


def test_fe_stiffer_plate_deflects_less():
    dom = _tiny_domain()
    fe = _PlateFE(dom)
    case = _one_case()
    full = np.ones((fe.ny, fe.nx))
    Us, cs = fe.solve_cases(full, [case])
    dom2 = PlateDomain(**{**dom.__dict__, "thickness_mm": dom.thickness_mm * 2})
    fe2 = _PlateFE(dom2)
    _, cs2 = fe2.solve_cases(np.ones((fe2.ny, fe2.nx)), [case])
    assert cs2[0] < cs[0] * 0.6          # 2× thickness → ~half the compliance


# --------------------------------------------------------------------------- #
#  2 · the fabrication audit is a measurement
# --------------------------------------------------------------------------- #
def test_audit_thick_bar_passes_thin_rib_fails():
    lim = FabricationLimits(process="t", min_rib_mm=6.0, haz_mm=0.0,
                            min_rib_haz_mm=6.0)
    h = 1.0
    S = np.zeros((40, 40), bool)
    S[:, 10:20] = True                   # 10 mm bar — buildable
    a = fabrication_audit(S, h, lim)
    assert a["ok"] and a["viol_frac"] <= 0.01
    S2 = np.zeros((40, 40), bool)
    S2[:, 18:20] = True                  # 2 mm rib < 6 mm rule — not buildable
    a2 = fabrication_audit(S2, h, lim)
    assert not a2["ok"] and a2["viol_frac"] > 0.5


def test_audit_edge_flush_weld_foot_not_falsely_eaten():
    """A wide block flush against the plate boundary is judged by its
    in-plate width, not erased because 'outside' reads as void."""
    lim = FabricationLimits(process="t", min_rib_mm=6.0, haz_mm=0.0,
                            min_rib_haz_mm=6.0)
    S = np.zeros((30, 30), bool)
    S[0:12, :] = True                    # 12 mm-tall full-width slab at y=0
    a = fabrication_audit(S, 1.0, lim)
    assert a["ok"], a


def test_audit_haz_disk_is_stricter_and_localised():
    lim = FabricationLimits(process="t", min_rib_mm=3.0, haz_mm=8.0,
                            min_rib_haz_mm=8.0)
    h = 1.0
    S = np.zeros((40, 20), bool)
    S[:, 8:12] = True                    # 4 mm column: passes bulk (3), fails HAZ (8)
    haz = np.zeros_like(S)
    haz[0:8, :] = True
    a = fabrication_audit(S, h, lim, haz=haz)
    assert a["viol_frac"] <= 0.01        # bulk rule content
    assert a["viol_haz_frac"] > 0.5      # the weld zone catches it
    assert not a["ok"]
    # the same column with no declared weld zone is fine
    a2 = fabrication_audit(S, h, lim, haz=None)
    assert a2["ok"]


def test_audit_protect_excludes_bosses():
    lim = FabricationLimits(process="t", min_rib_mm=8.0, haz_mm=0.0,
                            min_rib_haz_mm=8.0)
    S = np.zeros((30, 30), bool)
    S[14:17, 14:17] = True               # a 3 mm island — normally a violation
    prot = S.copy()
    a = fabrication_audit(S, 1.0, lim, protect=prot)
    assert a["n_viol"] == 0


# --------------------------------------------------------------------------- #
#  3 · fabrication limits from the shop / the declared field
# --------------------------------------------------------------------------- #
def test_limits_presets_order_and_arithmetic():
    hand = FabricationLimits.from_shop("hand_weld")
    jig = FabricationLimits.from_shop("jig_weld")
    cnc = FabricationLimits.from_shop("cnc")
    assert hand.min_rib_mm > jig.min_rib_mm > cnc.min_rib_mm
    # min_rib = floor + 2u, exactly
    assert hand.min_rib_mm == pytest.approx(hand.web_floor_mm
                                            + 2 * hand.accuracy_mm)
    assert cnc.haz_mm == 0.0             # machined+bolted: no weld zone
    with pytest.raises(ValueError):
        FabricationLimits.from_shop("wizard")


def test_limits_from_tolerance_field_reads_declared_accuracy():
    from suspension.kinematik_stochastic import ToleranceField
    fld = ToleranceField.preset("hand_weld")
    lim = FabricationLimits.from_tolerance_field(fld, web_floor_mm=2.0)
    # hand-weld preset holds ±1.5 mm on the tabs → u = 1.5 → rib = 2 + 3
    assert lim.accuracy_mm == pytest.approx(1.5)
    assert lim.min_rib_mm == pytest.approx(5.0)


# --------------------------------------------------------------------------- #
#  4 · the load fan reads the ghost instants faithfully
# --------------------------------------------------------------------------- #
def test_fan_direction_follows_the_deformed_link_line():
    inner = np.asarray(HP.lower_front_inner, float)
    outer = np.asarray(HP.lower_outer, float)
    aud = _fake_audit([_fake_instant(0.0, 1000.0, outer),
                       _fake_instant(0.1, 1000.0, outer)])
    fan = load_fan_from_audit(aud, HP, member="LF", n_cases=2)
    assert fan.ok and len(fan.cases) == 1          # static direction → 1 arrow
    d3 = (outer - inner) / np.linalg.norm(outer - inner)
    # the case arrow, mapped back to 3D via the fan basis, is the link line
    c = fan.cases[0]
    back = c.dir2[0] * fan.e1 + c.dir2[1] * fan.e2
    assert abs(abs(float(back @ d3)) - 1.0) < 1e-6
    assert c.F_N == pytest.approx(1000.0)


def test_fan_reversal_keeps_its_own_arrow_and_is_named():
    outer = np.asarray(HP.lower_outer, float)
    ins = [_fake_instant(0.01 * i, 1000.0, outer) for i in range(8)]
    ins += [_fake_instant(0.5 + 0.01 * i, -400.0, outer) for i in range(3)]
    fan = load_fan_from_audit(_fake_audit(ins), HP, member="LF", n_cases=4)
    assert len(fan.cases) == 2
    assert fan.span_deg == pytest.approx(180.0, abs=1.0)
    assert 0.05 < fan.reversal_share < 0.5
    assert any("REVERSES" in w for w in fan.warnings)
    # both amplitudes are the lobe peaks, not means
    Fs = sorted(c.F_N for c in fan.cases)
    assert Fs == pytest.approx([400.0, 1000.0])


def test_fan_flags_out_of_plane_and_starved_audits():
    outer = np.asarray(HP.lower_outer, float)
    fan = load_fan_from_audit(_fake_audit([]), HP, member="LF")
    assert not fan.ok
    fan2 = load_fan_from_audit(
        _fake_audit([_fake_instant(0.0, 0.0, outer)]), HP, member="LF")
    assert not fan2.ok
    fan3 = load_fan_from_audit(_fake_audit([]), HP, member="XX")
    assert not fan3.ok and "unknown member" in fan3.warnings[0]


def test_fan_forced_plane_reports_dropped_share():
    outer = np.asarray(HP.lower_outer, float)
    ins = [_fake_instant(0.0, 1000.0, outer)]
    inner = np.asarray(HP.lower_front_inner, float)
    d3 = (outer - inner) / np.linalg.norm(outer - inner)
    # a plane nearly orthogonal to the link line drops most of the load
    e1 = np.cross(d3, [0, 0, 1.0]); e1 /= np.linalg.norm(e1)
    e2 = np.cross(d3, e1); e2 /= np.linalg.norm(e2)
    fan = load_fan_from_audit(_fake_audit(ins), HP, member="LF",
                              plane=(e1, e2))
    assert fan.inplane_share < 0.2
    assert any("bracket plane" in w for w in fan.warnings)


# --------------------------------------------------------------------------- #
#  5 · the engine — grow, respect the domain, screen the structure
# --------------------------------------------------------------------------- #
def test_morph_forgeable_and_respects_bores():
    dom = _tiny_domain()
    lim = FabricationLimits.from_shop("hand_weld")
    res = morph_component(dom, [_one_case()], lim, volfrac=0.45,
                          max_iter=10, betas=(1.0, 4.0))
    assert res.verdict in VERDICTS and res.ok
    assert res.solid.any() and res.mass_g > 0
    fe = _PlateFE(PlateDomain(**res.domain_meta))
    assert not res.solid[fe.pass_void].any()       # bore stays void
    assert res.solid[fe.pass_solid].all()          # boss stays solid
    assert np.isfinite(res.fos) and res.fos > 0
    # audit of the shipped shape agrees with the accepted round
    a = fabrication_audit(res.solid, dom.h_mm, lim,
                          fe.haz_mask(lim.haz_mm), fe.pass_solid)
    assert a["ok"]


def test_morph_load_starved_and_never_raises():
    dom = _tiny_domain()
    lim = FabricationLimits.from_shop("cnc")
    r = morph_component(dom, [], lim)
    assert r.verdict == "LOAD_STARVED" and not r.ok
    r2 = morph_component(dom, [_one_case(F=0.01)], lim)
    assert r2.verdict == "LOAD_STARVED"


def test_morph_reject_and_coarsen_contract(monkeypatch):
    """Round 1 fails the shop audit → the engine re-grows coarser, verdict
    COARSENED, the premium is priced, and the finding trail says REJECTED."""
    dom = _tiny_domain()
    lim = FabricationLimits.from_shop("hand_weld")
    real = mm.fabrication_audit
    calls = {"n": 0}

    def flaky(*a, **k):
        calls["n"] += 1
        out = real(*a, **k)
        if calls["n"] == 1:
            out = dict(out, ok=False, viol_frac=0.30)
        return out

    monkeypatch.setattr(mm, "fabrication_audit", flaky)
    res = morph_component(dom, [_one_case()], lim, volfrac=0.45,
                          max_iter=8, betas=(1.0, 4.0))
    assert res.verdict == "COARSENED"
    assert len(res.rounds) == 2
    assert not res.rounds[0].accepted and res.rounds[1].accepted
    assert res.rounds[1].filter_mm > res.rounds[0].filter_mm
    assert res.coarsen_premium is not None
    assert any("REJECTED" in f["message"] for f in res.findings)


def test_morph_unbuildable_names_the_binding_constraint(monkeypatch):
    dom = _tiny_domain()
    lim = FabricationLimits.from_shop("hand_weld")
    real = mm.fabrication_audit
    monkeypatch.setattr(mm, "fabrication_audit",
                        lambda *a, **k: dict(real(*a, **k), ok=False,
                                             viol_frac=0.4, viol_haz_frac=0.1))
    res = morph_component(dom, [_one_case()], lim, max_iter=5,
                          betas=(1.0,), max_rounds=2)
    assert res.verdict == "UNBUILDABLE" and not res.ok
    assert any("bulk rib width" in f["message"] for f in res.findings)


def test_morph_structural_screen_prices_the_thickness_fix():
    dom = _tiny_domain(thickness_mm=1.0)
    dom = PlateDomain(**{**dom.__dict__, "yield_MPa": 40.0})   # weak on purpose
    lim = FabricationLimits.from_shop("hand_weld")
    res = morph_component(dom, [_one_case(F=6000.0)], lim, volfrac=0.5,
                          max_iter=8, betas=(1.0, 4.0))
    if res.ok and np.isfinite(res.fos) and res.fos < 1.5:
        assert res.suggested_thickness_mm == pytest.approx(
            dom.thickness_mm * 1.5 / res.fos, rel=1e-6)
        assert any("restores the rule" in f["message"] for f in res.findings)


def test_morph_multi_case_shape_serves_both_arrows():
    """Two opposed-ish arrows: the multi-case shape must carry BOTH better
    than a shape grown for one arrow carries the other."""
    dom = _tiny_domain()
    lim = FabricationLimits.from_shop("jig_weld")
    ca = _one_case(-70.0, 2000.0, 0.6)
    cb = _one_case(-120.0, 1500.0, 0.4)
    both = morph_component(dom, [ca, cb], lim, volfrac=0.4,
                           max_iter=10, betas=(1.0, 4.0))
    only_a = morph_component(dom, [LoadCase(ca.dir2, ca.F_N, 1.0, 5, 0, .5,
                                            ca.angle_deg)],
                             lim, volfrac=0.4, max_iter=10, betas=(1.0, 4.0))
    assert both.ok and only_a.ok
    fe = _PlateFE(PlateDomain(**both.domain_meta))
    _, c_both = fe.solve_cases(both.solid.astype(float), [cb])
    _, c_a = fe.solve_cases(only_a.solid.astype(float), [cb])
    assert c_both[0] < c_a[0] * 1.05     # the fan-grown shape serves case B


# --------------------------------------------------------------------------- #
#  6 · the one-call join + the report
# --------------------------------------------------------------------------- #
def test_morph_from_audit_end_to_end_with_stub():
    outer = np.asarray(HP.lower_outer, float)
    ins = [_fake_instant(0.01 * i, 1500.0 + 100 * i, outer) for i in range(6)]
    res = morph_from_audit(_fake_audit(ins), HP, member="LF",
                           dom=_tiny_domain(),
                           limits=FabricationLimits.from_shop("hand_weld"),
                           volfrac=0.45, max_iter=8, betas=(1.0, 4.0))
    assert res.ok
    assert res.fan is not None and res.fan.member == "LF"
    assert res.fan.cases[0].F_N == pytest.approx(2000.0)      # the peak
    # summary round-trips through JSON
    s = json.loads(res.to_json())
    assert s["verdict"] == res.verdict
    assert s["fan"]["member"] == "LF"
    md = render_morph_md(res)
    assert res.verdict in md and "starting shape" in md
    # exports carry the shape
    csv = res.cells_csv(res.domain_meta["h_mm"])
    assert csv.startswith("x_mm,y_mm,density") and len(csv.splitlines()) > 10
    segs = res.outline_segments(res.domain_meta["h_mm"])
    assert len(segs) > 10


def test_morph_from_audit_starved_is_flagged_not_raised():
    res = morph_from_audit(_fake_audit([]), HP, member="LF",
                           dom=_tiny_domain())
    assert res.verdict == "LOAD_STARVED" and not res.ok
    assert "LOAD_STARVED" in render_morph_md(res)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
