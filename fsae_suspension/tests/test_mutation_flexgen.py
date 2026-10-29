"""Characterization suite for suspension/flexgen.py (mutation gate); see tests/_golden.py."""
import dataclasses
import datetime
import math

import numpy as np
import pytest

from suspension import flexgen as fg
from suspension.flex import MATERIALS, Material
from tests._golden import check

STEEL = MATERIALS["Steel 4130"]
TI = MATERIALS["Titanium Ti-6Al-4V"]
CF = MATERIALS["Carbon (axial, representative)"]
ODD = dataclasses.replace(STEEL, name="Unobtainium")


def _b(L=80.0, w=30.0, t=2.0, tt=None, mat=STEEL, bnd="fixed-free", n=6, name="blade"):
    return fg.FlexureBlade(L, fg.BladeSection(w, t, tt), mat, boundary=bnd, n_seg=n, name=name)


def test_sections_and_blades():
    res = {}
    secs = [fg.BladeSection(30.0, 2.0), fg.BladeSection(20.0, 3.0, 1.5), fg.BladeSection(10.0, 1.0, 1.0)]
    res["sec"] = [[s.t_at(f), s.A_at(f), s.I_at(f), s.I_stiff_at(f)] for s in secs for f in (0.0, 0.25, 1.0)]
    errs = []
    for args in ((0.0, 1.0), (-1.0, 1.0), (1.0, 0.0), (1.0, -1.0), (1.0, 1.0, 0.0), (1.0, 1.0, -0.5),
                 (1.0, 1.0, 1.0 + 1e-6)):
        with pytest.raises(ValueError) as e:
            fg.BladeSection(*args)
        errs.append(str(e.value))
    fg.BladeSection(1.0, 1.0, 1.0 + 1e-13)
    for kw in (dict(L=0.0), dict(L=-2.0), dict(bnd="pinned"), dict(n=3)):
        with pytest.raises(ValueError) as e:
            _b(**kw)
        errs.append(str(e.value))
    _b(n=4)
    res["err"] = errs
    for k, b in {"ff": _b(), "fg": _b(bnd="fixed-guided", tt=1.2, n=8), "ti": _b(60.0, 20.0, 1.5, mat=TI)}.items():
        res[k] = [b.seg_len, b.spring_stations(), b.spring_k(), b.mass_g(), b.k_compliant_linear(),
                  b.k_stiff_linear(), b.k_axial_linear()]
    res["fat"] = fg.FATIGUE
    check("flexgen_blade", res)


def test_prb_chain_solves():
    res = {}
    blades = {"ff": _b(n=5), "fg": _b(bnd="fixed-guided", tt=1.4, n=4)}
    for k, b in blades.items():
        c = fg.PRBChain(b)
        res[k] = {"init": [c.k, c.l, c.n, c.lens, c.x0, c.guided, c.ndof],
                  "full": c._full_phi(np.arange(1.0, c.ndof + 1) * 0.01),
                  "tip": c._tip(np.linspace(0.0, 0.3, c.n)), "e": c._spring_energy(np.linspace(0.0, 0.3, c.n)),
                  "gh": c._grad_hess(np.linspace(0.05, 0.2, c.ndof), 30.0, 20.0, 400.0),
                  "jac": c._jac_tip(np.linspace(0.0, 0.3, c.n)),
                  "s0": c.solve(), "s1": c.solve(50.0, 40.0, 300.0, n_steps=3),
                  "s2": c.solve(-20.0, -60.0), "pcr": c.critical_axial_load_n(),
                  "q0": c.solve(0.0, 10.0, 0.0, n_steps=1, q0=np.full(c.ndof, 0.01)),
                  "tr": [c.solve_travel(d, f) for d, f in ((0.0, 0.0), (1e-13, 5.0), (-8.0, 30.0))],
                  "tr_tol": c.solve_travel(5.0, tol=1e-2)}
    c = fg.PRBChain(blades["ff"])
    with pytest.raises(RuntimeError) as e:
        c.solve(-10.0 * c.critical_axial_load_n(), 1.0)
    res["post_buckle"] = str(e.value)
    with pytest.raises(RuntimeError) as e:
        c.solve(0.0, 1e9, n_steps=1)
    res["no_conv"] = str(e.value)
    check("flexgen_chain", res)


def test_chain_failure_messages(monkeypatch):
    c = fg.PRBChain(_b(n=4))
    res = {}
    real = np.linalg.solve

    def broken(a, b):
        raise np.linalg.LinAlgError("x")

    monkeypatch.setattr(fg.np.linalg, "solve", broken)
    with pytest.raises(RuntimeError) as e:
        c.solve(0.0, 5.0)
    res["regularise"] = str(e.value)
    calls = {"n": 0}

    def nan_once(a, b):
        calls["n"] += 1
        return np.full(np.shape(b), np.nan) if calls["n"] == 1 else real(a, b)

    monkeypatch.setattr(fg.np.linalg, "solve", nan_once)
    res["nan_once"] = c.solve(0.0, 5.0)
    monkeypatch.setattr(fg.np.linalg, "solve", real)
    monkeypatch.setattr(fg.PRBChain, "solve", lambda self, fx=0.0, fy=0.0, m=0.0, **kw: fg.BladeState(
        fx, fy, m, np.zeros(4), 0.0, 0.0 * fy, 0.0, 0.0, np.eye(3), np.zeros(4), 1.0))
    with pytest.raises(RuntimeError) as e:
        c.solve_travel(5.0)
    res["bracket"] = str(e.value)
    check("flexgen_fail", res)


def test_spring_downsize_and_lint(monkeypatch):
    res = {}
    b = _b(bnd="fixed-guided", n=4)
    res["sp"] = [fg.equivalent_spring(b, 6.0, n_pts=2), fg.equivalent_spring(b, 4.0, axial_preload_n=-30.0, n_pts=2)]
    res["sp_e"] = res["sp"][0].energy_at_full_j
    res["down"] = [fg.coilover_downsize(30.0, 10.0), fg.coilover_downsize(30.0, 45.0, 0.8),
                   fg.coilover_downsize(30.0, 30.0, 1.2)]
    for args in ((0.0, 1.0), (-1.0, 1.0), (10.0, 1.0, 0.0), (10.0, 1.0, -1.0)):
        with pytest.raises(ValueError, match="^coilover_downsize: target rate and motion ratio > 0"):
            fg.coilover_downsize(*args)
    BL = fg.BladeLoadCase
    cases = [BL("travel", 0.0, 0.0, 4.0), BL("shear", 0.0, 50.0, 0.0), BL("both", 100.0, 30.0, 3.0),
             BL("comp", -40.0, 20.0, 0.0)]
    tapered = _b(t=3.0, tt=1.5, n=4)
    res["cs"] = [fg._case_stress(bl, fg.PRBChain(bl), c) for bl in (b, tapered) for c in cases]
    shear = [BL("s", 0.0, 50.0, 0.0), BL("a", 300.0, 10.0, 0.0)]
    blades = {"steel": b, "thin": _b(t=0.6, bnd="fixed-guided", n=4), "ti": _b(60.0, 20.0, 1.5, mat=TI, n=4),
              "cf": _b(mat=CF, n=4), "odd": _b(mat=ODD, n=4), "square": _b(w=2.0, t=2.0, n=4)}
    for k, bl in blades.items():
        res["lint_" + k] = fg.flexgen_lint(bl, shear)
    res["lint_travel"] = fg.flexgen_lint(b, cases[:1])
    res["unstable"] = fg.flexgen_lint(b, [BL("huge", 0.0, 0.0, 2000.0)])
    pcr = fg.PRBChain(b).critical_axial_load_n()
    comp = [BL(f"c{x}", -pcr / x, 0.0, 0.0) for x in (0.5, 1.5, 2.2, 2.6, 4.0)]
    stresses = {}

    def fake_stress(blade, chain, case):
        return stresses[case.name]

    monkeypatch.setattr(fg, "_case_stress", fake_stress)
    stresses.update({c.name: (10.0, -1.0, 1.0) for c in comp})
    res["buck"] = fg.flexgen_lint(b, comp)
    res["buck_kw"] = fg.flexgen_lint(b, comp, fos_yield=1.0, fos_fatigue=1.0, fos_buckling=1.4, min_stiff_ratio=1e6)
    S = fg.FATIGUE["Steel 4130"]
    grid = {"y_lo": (S.S_y / 1.5 - 1e-6, 0.0, 0.0), "y_hi": (S.S_y / 1.5 + 1e-6, 0.0, 0.0),
            "tau": (100.0, 50.0, 120.0), "neg_ax": (100.0, -80.0, 0.0),
            "g_hi": (S.S_e / 1.5 + 1e-6, 0.0, 0.0), "g_lo": (S.S_e / 1.5 - 1e-6, 0.0, 0.0),
            "g_mean": (100.0, 60.0, 0.0), "near_hi": (0.8 * S.S_e / 1.5 + 1e-6, 0.0, 0.0),
            "near_lo": (0.8 * S.S_e / 1.5 - 1e-6, 0.0, 0.0), "zero": (0.0, 0.0, 0.0)}
    stresses.update(grid)
    for k in grid:
        res["g_" + k] = [fg.flexgen_lint(bl, [BL(k)], fos_yield=1.5, fos_fatigue=1.5) for bl in (b, blades["cf"])]
    res["g_kw"] = fg.flexgen_lint(b, [BL("tau"), BL("g_mean")], 2.0, 2.0)
    stresses["boom"] = None

    def boom(blade, chain, case):
        raise RuntimeError("FlexGen: no convergence (test)")

    monkeypatch.setattr(fg, "_case_stress", boom)
    res["boom"] = fg.flexgen_lint(b, [BL("x")])
    check("flexgen_lint", res)


def test_synthesis_and_exports(monkeypatch):
    res = {}
    BL = fg.BladeLoadCase
    cases = [BL("ride", 0.0, 0.0, 0.0), BL("load", 20.0, 5.0, 0.0)]
    res["syn"] = fg.synthesize_blade("Steel 4130", 25.0, 1.0, cases, n_length=2, n_t=1)
    with pytest.raises(ValueError, match="^Unknown material 'Nope'"):
        fg.synthesize_blade("Nope", 1.0, 1.0, cases)
    log = []

    def fake_lint(blade, cs, fy, ff, fb):
        log.append([blade.length_mm, blade.section, blade.boundary, [c.travel_mm for c in cs], fy, ff, fb])
        if blade.section.t_root_mm < 2.0:
            return [fg.Finding("blocker", "YIELD", "x")]
        if blade.length_mm > 100.0:
            raise RuntimeError("x")
        return [fg.Finding("ok", "PASS", str(blade.length_mm))]

    monkeypatch.setattr(fg, "flexgen_lint", fake_lint)
    res["syn_fake"] = [fg.synthesize_blade("Steel 4130", 25.0, 5.0, cases), list(log)]
    log.clear()
    res["syn_kw"] = [fg.synthesize_blade("Titanium Ti-6Al-4V", 20.0, 4.0, cases, length_range_mm=(30.0, 120.0),
                                         t_range_mm=(1.0, 3.0), boundary="fixed-free", taper=0.9, n_length=3,
                                         n_t=3, fos_yield=1.2, fos_fatigue=1.1, fos_buckling=1.5), list(log)]
    res["syn_none"] = fg.synthesize_blade("Steel mild", 5.0, 40.0, cases, length_range_mm=(110.0, 120.0),
                                          t_range_mm=(0.5, 6.0), n_length=2, n_t=2)
    monkeypatch.undo()
    b = _b(tt=1.2, name="bl")
    res["solid"] = fg._blade_solid(b)
    res["stl"] = fg.export_stl(b)
    res["step"] = fg.export_step(b).replace(datetime.date.today().isoformat(), "<DATE>")
    res["layup"] = [fg.layup_map(b), fg.layup_map(_b(mat=CF), n_stations=3), fg.layup_map(b, 4, "carbon_layup"),
                    fg.layup_map(_b(mat=CF), 2, "additive_ti")]
    with pytest.raises(ValueError, match="^layup_map: unknown process 'cnc'"):
        fg.layup_map(b, process="cnc")
    bg = _b(bnd="fixed-guided", n=4)
    sp = fg.equivalent_spring(bg, 3.0, n_pts=2)
    f = [fg.Finding("ok", "A", "a"), fg.Finding("warn", "B", "b|c"), fg.Finding("blocker", "C", "c")]
    res["md"] = [fg.render_flexgen_md(bg, sp, f), fg.render_flexgen_md(bg, sp, [], fg.coilover_downsize(30.0, 10.0)),
                 fg.render_flexgen_md(bg, sp, f[:1], fg.coilover_downsize(5.0, 10.0, 0.9))]
    check("flexgen_out", res)


def test_characterized_defaults_and_reports():
    BL = fg.BladeLoadCase
    b = _b(bnd="fixed-guided", n=4)
    tp = _b(t=2.4, tt=1.1, mat=TI, n=4)
    sp = fg.equivalent_spring(b, 5.0)
    res = {"sp_dflt": sp, "down": [fg.coilover_downsize(30.0, 10.0, 0.8), fg.coilover_downsize(0.5, 0.2)],
           "lint": fg.flexgen_lint(b, [BL("huge", 0.0, 0.0, 2000.0), BL("s", 0.0, 50.0, 0.0)]),
           "layup": fg.layup_map(_b(t=0.5, tt=0.3, mat=TI, n=4), 5, "additive_ti"),
           "md": fg.render_flexgen_md(tp, fg.equivalent_spring(tp, 7.3, n_pts=3), [])}
    check("flexgen_reports", res)
