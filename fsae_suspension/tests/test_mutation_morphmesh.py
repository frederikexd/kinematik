"""Characterization suite for suspension/morphmesh.py (mutation gate); see tests/_golden.py."""
import copy
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from suspension import morphmesh as mm
from suspension.kinematics import Hardpoints
import json

from tests._golden import check, plain

HP = Hardpoints.default()


def _case(angle_deg=-90.0, F=2000.0, w=1.0, n=5, lo=0.0, hi=0.5):
    a = math.radians(angle_deg)
    return mm.LoadCase(np.array([math.cos(a), math.sin(a)]), F, w, n, lo, hi, angle_deg)


def _inst(t, T, member="LF", outer=None, use_margin=True, attr="lower_outer"):
    g = NS(upper_outer=None, lower_outer=None, tie_rod_outer=None, pushrod_outer=None)
    if outer is not None:
        setattr(g, attr, np.asarray(outer, float))
    return NS(t=t, ghost=g, margins={member: {"force_N": T}} if use_margin else None,
              load_path_shift={member: {"ghost_N": T}})


def test_load_fans():
    res = {}
    inner = np.asarray(HP.lower_front_inner, float)
    dirs = [inner + 300.0 * np.array([0.1, 1.0, 0.2 * k]) for k in range(-3, 4)]
    insts = [_inst(0.01 * i, 1000.0 + 300.0 * i, outer=d) for i, d in enumerate(dirs)]
    A = NS(instants=insts)
    res["lf"] = mm.load_fan_from_audit(A, HP).summary()
    res["lf_full"] = mm.load_fan_from_audit(A, HP, "lf", n_cases=2)
    res["lf_1"] = mm.load_fan_from_audit(A, HP, n_cases=0).summary()
    res["lf_8"] = mm.load_fan_from_audit(A, HP, n_cases=8).summary()
    res["lf_20"] = mm.load_fan_from_audit(A, HP, n_cases=20).summary()
    rev = insts + [_inst(0.2, -2500.0, outer=dirs[3]), _inst(0.21, -1800.0, outer=dirs[1])]
    res["rev"] = mm.load_fan_from_audit(NS(instants=rev), HP).summary()
    res["plane"] = mm.load_fan_from_audit(A, HP, plane=(np.array([0.0, 2.0, 0.0]), np.array([0.0, 1.0, 1.0])))
    res["plane_oop"] = mm.load_fan_from_audit(A, HP, plane=(np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])))
    perp = [_inst(0.0, 1000.0, outer=inner + np.array([300.0, 0.0, 0.0]))]
    res["negligible"] = mm.load_fan_from_audit(NS(instants=perp), HP,
                                               plane=(np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])))
    sym = [_inst(0.0, 1000.0, outer=dirs[3]), _inst(0.1, -1000.0, outer=dirs[3])]
    res["sym"] = mm.load_fan_from_audit(NS(instants=sym), HP).summary()
    res["unknown"] = mm.load_fan_from_audit(A, HP, member="XX")
    res["empty"] = mm.load_fan_from_audit(NS(instants=None), HP)
    res["noattr"] = mm.load_fan_from_audit(object(), HP, member="UF")
    res["tiny"] = mm.load_fan_from_audit(NS(instants=[_inst(0.0, 0.5, outer=dirs[0])]), HP)
    res["tiny_sum"] = mm.load_fan_from_audit(NS(instants=[_inst(0.0, 0.4, outer=d) for d in dirs[:4]]), HP)
    res["shift"] = mm.load_fan_from_audit(NS(instants=[_inst(0.0, 900.0, outer=dirs[2], use_margin=False),
                                                       _inst(0.1, 700.0, outer=dirs[5], use_margin=False)]), HP)
    res["hp_outer"] = mm.load_fan_from_audit(NS(instants=[_inst(0.0, 900.0), _inst(0.1, 500.0)]), HP).summary()
    h = copy.deepcopy(HP)
    h.lower_outer = None
    res["no_outer"] = mm.load_fan_from_audit(NS(instants=[_inst(0.0, 900.0)]), h)
    res["zero_len"] = mm.load_fan_from_audit(NS(instants=[_inst(0.0, 900.0, outer=inner)]), HP)
    pr = [_inst(0.01 * i, 800.0 + 50 * i, "PR", np.asarray(HP.rocker_pushrod) + [0.0, 200.0, -150.0 + 30 * i],
                attr="pushrod_outer") for i in range(4)]
    res["pr"] = mm.load_fan_from_audit(NS(instants=pr), HP, member="pr").summary()
    h2 = copy.deepcopy(HP)
    h2.rocker_pushrod = None
    res["pr_proxy"] = mm.load_fan_from_audit(NS(instants=pr), h2, member="PR").summary()
    for m in ("UF", "UR", "LR", "TR"):
        g = [_inst(0.01 * i, 600.0 + 100 * i, m, None) for i in range(3)]
        res["m_" + m] = mm.load_fan_from_audit(NS(instants=g), HP, member=m).summary()
    res["case"] = _case(-70.0, 1234.5678, 0.33333, 7, 0.0123, 0.4567).summary()
    res["fan_ok"] = [mm.LoadFan("LF", [_case()], *np.eye(3), 1.0, 0.0, 1, 1.0).ok,
                     mm.LoadFan("LF", [_case()], *np.eye(3), 1.0, 0.0, 1, 1.01).ok,
                     mm.LoadFan("LF", [], *np.eye(3), 1.0, 0.0, 1, 50.0).ok]
    check("morph_fan", res)


def test_limits_domains_and_fe():
    res = {}
    res["shop"] = [mm.FabricationLimits.from_shop(s) for s in ("hand_weld", "jig_weld", "cnc")] + [
        mm.FabricationLimits.from_shop("jig_weld", 2.0), mm.FabricationLimits(), mm.FabricationLimits().summary()]
    with pytest.raises(ValueError) as e:
        mm.FabricationLimits.from_shop("laser")
    res["shop_err"] = str(e.value)
    fld = NS(specs={"a": NS(lo=np.array([0.0, -1.0]), hi=np.array([0.4, 1.4])), "b": NS(lo=np.zeros(1), hi=np.ones(1))})
    res["fld"] = [mm.FabricationLimits.from_tolerance_field(fld), mm.FabricationLimits.from_tolerance_field(NS()),
                  mm.FabricationLimits.from_tolerance_field(fld, "x", 1.0, 0.0, 2.0).summary()]
    doms = {"tab": mm.PlateDomain.chassis_tab(24.0, 30.0, 4.0, 3.0, "Aluminium 6061", 276.0, 2.0),
            "piv": mm.PlateDomain.pivot_bracket(30.0, 24.0, 3.0, 2.0, 4.0, h_mm=2.0),
            "small": mm.PlateDomain(width_mm=4.0, height_mm=3.0, h_mm=1.0, load_bore=(2.0, 2.0, 0.5)),
            "odd": mm.PlateDomain(width_mm=14.0, height_mm=14.0, h_mm=2.0, material="Unobtainium",
                                  load_bore=(100.0, 100.0, 1.0), ring_mm=-1.0),
            "dtab": mm.PlateDomain.chassis_tab(), "dpiv": mm.PlateDomain.pivot_bracket()}
    for k, d in doms.items():
        res["dom_" + k] = [d, d.nx, d.ny, [c[::7, ::5] for c in d.cell_centers()]]
    res["q4"] = mm._q4_ke_and_b(0.3, 2.0), mm._q4_ke_and_b(0.25, 1.0)
    for k in ("tab", "piv", "odd"):
        fe = mm._PlateFE(doms[k])
        x = np.where(fe.pass_void, 0.0, 0.6)
        Us, cs = fe.solve_cases(x, [_case(-80.0, 1500.0), _case(10.0, 900.0, 0.5)])
        res["fe_" + k] = [fe.nx, fe.ny, fe.nn, fe.ndof, fe.E0, fe.Emin, fe.rho, fe.nu, fe.pass_void, fe.pass_solid,
                          fe.active, fe.fixed_dofs, fe.load_nodes, fe.haz_from, fe.force_vector(_case(30.0, 100.0)),
                          fe.haz_mask(0.0), fe.haz_mask(-1.0), fe.haz_mask(3.0), cs, Us[0][::9],
                          fe.element_strain(Us[0])[::5], fe.von_mises(Us[0], x)[::3], fe.stress_tensor(Us[1])[::7]]
    nob = mm._PlateFE(mm.PlateDomain(width_mm=12.0, height_mm=12.0, h_mm=2.0, anchor="bores",
                                     load_bore=(6.0, 6.0, 1.0)))
    res["fe_nobores"] = [nob.fixed_dofs, nob.free.size, nob.haz_from, nob.haz_mask(4.0)]
    check("morph_fe", res)


def test_filters_and_audit():
    res = {"cone": [mm._cone_kernel(r) for r in (0.5, 1.0, 1.001, 2.5)],
           "disk": [mm._disk(r) for r in (0, 1, 2, 3)],
           "proj": [mm._project(np.linspace(0.0, 1.0, 5), b) for b in (1.0, 8.0)] + [
               mm._project(np.linspace(0.0, 1.0, 5), 4.0, 0.3)]}
    f = mm._Filter(1.6, (6, 7))
    x = np.arange(42.0).reshape(6, 7) / 42.0
    res["filt"] = [f.w, f.Hs, f.forward(x), f.backward(x)]
    lim = mm.FabricationLimits.from_shop("hand_weld")
    S = np.zeros((30, 30), bool)
    S[5:25, 5:25] = True
    thin = S.copy()
    thin[10:20, 10:20] = False
    rib = np.zeros((30, 30), bool)
    rib[2:28, 14:16] = True
    haz = np.zeros((30, 30), bool)
    haz[:8] = True
    cases = {"empty": (np.zeros((5, 5), bool), None, None), "block": (S, None, None), "thin": (thin, None, None),
             "rib": (rib, None, None), "rib_haz": (rib, haz, None), "rib_prot": (rib, None, rib),
             "block_haz": (S, haz, None), "rib_nohaz": (rib, np.zeros_like(haz), None)}
    for k, (s, hz, pr) in cases.items():
        res["a_" + k] = mm.fabrication_audit(s, 1.0, lim, hz, pr)
    res["a_cnc"] = mm.fabrication_audit(rib, 0.5, mm.FabricationLimits.from_shop("cnc"), haz)
    res["a_h2"] = mm.fabrication_audit(thin.astype(int), 2.0, lim, haz.astype(int))
    nohaz = mm.FabricationLimits(haz_mm=0.0)
    res["a_lim0"] = mm.fabrication_audit(rib, 1.0, nohaz, haz)
    check("morph_audit", res)


def _dom(**kw):
    base = dict(width_mm=24.0, height_mm=30.0, h_mm=2.0, thickness_mm=4.0)
    base.update(kw)
    return mm.PlateDomain.chassis_tab(**base)


FAST = dict(max_iter=4, betas=(1.0, 4.0))


def test_morph_component(monkeypatch):
    res = {}
    cnc = mm.FabricationLimits.from_shop("cnc")
    hw = mm.FabricationLimits.from_shop("hand_weld")
    two = [_case(-70.0, 2500.0, 0.6), _case(-120.0, 1800.0, 0.4)]
    fan = mm.LoadFan("LF", two, *np.eye(3), 0.97, 50.0, 18, 2500.0, warnings=["fan warning"])
    runs = {"cnc": (_dom(), two, cnc, {}), "hw": (_dom(), two, hw, dict(volfrac=0.5, fan=fan)),
            "weak": (_dom(thickness_mm=0.5), [_case(-90.0, 9000.0)], cnc, {}),
            "piv": (mm.PlateDomain.pivot_bracket(30.0, 24.0, 3.0, 2.0, 4.0, h_mm=2.0), [_case(0.0, 800.0)], cnc,
                    dict(volfrac=0.35)),
            "tol": (_dom(), two, cnc, dict(tol=0.5, max_iter=10, betas=(2.0,))),
            "rounds1": (_dom(), two, cnc, dict(max_rounds=1))}
    for k, (d, c, lim, kw) in runs.items():
        r = mm.morph_component(d, c, lim, **{**FAST, **kw})
        res[k] = [r.summary(), r.density[::3, ::3], r.solid, r.compliance_history, r.stress_dir[::4, ::4],
                  r.stress_mag[::4, ::4], r.domain_meta, r.ok, mm.render_morph_md(r)]
    res["starved"] = [mm.morph_component(_dom(), [], cnc, fan=fan).summary(),
                      mm.morph_component(_dom(), [_case(F=0.5)], cnc).summary()]
    res["bad_dom"] = mm.morph_component(_dom(h_mm=0.0), two, cnc).summary()
    res["no_anchor"] = mm.morph_component(mm.PlateDomain(width_mm=12.0, height_mm=12.0, h_mm=2.0, anchor="bores",
                                                         load_bore=(6.0, 6.0, 1.0)), two, cnc).summary()

    def boom(*a, **k):
        raise RuntimeError("singular")

    with monkeypatch.context() as m:
        m.setattr(mm, "_simp_round", boom)
        res["simp_fail"] = mm.morph_component(_dom(), two, cnc).summary()
    real = mm.fabrication_audit
    calls = {"n": 0}

    def flaky(*a, **k):
        calls["n"] += 1
        out = real(*a, **k)
        return dict(out, ok=False, viol_frac=0.3, viol_haz_frac=0.05 * calls["n"]) if calls["n"] <= 2 else out

    with monkeypatch.context() as m:
        m.setattr(mm, "fabrication_audit", flaky)
        r = mm.morph_component(_dom(), two, hw, **FAST)
        res["coarse"] = [r.summary(), mm.render_morph_md(r)]
    for k, (vf, vh) in {"unb_bulk": (0.4, 0.1), "unb_haz": (0.05, 0.3)}.items():
        with monkeypatch.context() as m:
            m.setattr(mm, "fabrication_audit", lambda *a, vf=vf, vh=vh, **kw: dict(real(*a, **kw), ok=False,
                                                                                  viol_frac=vf, viol_haz_frac=vh))
            r = mm.morph_component(_dom(), two, hw, max_rounds=2, **FAST)
            res[k] = [r.summary(), mm.render_morph_md(r)]
    with monkeypatch.context() as m:
        m.setattr(mm, "_simp_round", lambda fe, cases, *a: (np.zeros((fe.ny, fe.nx)), [], 0))
        r = mm.morph_component(_dom(), two, cnc)
        res["void"] = [r.summary(), r.fos, r.mass_g]
    with monkeypatch.context() as m:
        m.setattr(mm, "_simp_round", lambda fe, cases, *a: (np.ones((fe.ny, fe.nx)), [5.0, 4.0], 2))
        m.setattr(mm, "fabrication_audit", lambda *a, **k: dict(real(*a, **k), ok=calls.setdefault("k", 0) > 0))
        r = mm.morph_component(_dom(), two, cnc)
        res["solid_one"] = r.summary()
    orig = mm._PlateFE.solve_cases
    seen = {"n": 0}

    def late_fail(self, x, cases):
        seen["n"] += 1
        if np.all((x == 0.0) | (x == 1.0)):          # only the final structural screen solves a binary shape
            raise RuntimeError("late")
        return orig(self, x, cases)

    with monkeypatch.context() as m:
        m.setattr(mm._PlateFE, "solve_cases", late_fail)
        res["screen_fail"] = mm.morph_component(_dom(), two, cnc, max_iter=4, betas=(1.0,)).summary()
    check("morph_component", res)


def test_morph_from_audit_and_outputs():
    res = {}
    inner = np.asarray(HP.lower_front_inner, float)
    insts = [_inst(0.01 * i, 1500.0 + 200.0 * i, outer=inner + 300.0 * np.array([0.1, 1.0, 0.2 * (i - 2)]))
             for i in range(5)]
    r = mm.morph_from_audit(NS(instants=insts), HP, dom=_dom(), limits=mm.FabricationLimits.from_shop("cnc"),
                            n_cases=2, volfrac=0.45, **FAST)
    res["ok"] = [r.summary(), mm.render_morph_md(r), r.to_json(), r.cells_csv(2.0), r.outline_segments(2.0)]
    s = mm.morph_from_audit(NS(instants=[]), HP)
    res["starved"] = [s.summary(), mm.render_morph_md(s), s.cells_csv(1.0), s.outline_segments(1.0), s.density.shape]
    f = mm._flagged("SOLVER_LIMITED", "msg")
    res["flag"] = [f.summary(), f.domain_meta, f.density.shape, mm.render_morph_md(f)]
    f2 = mm._flagged("WEIRD", "m2", warnings=["w"])
    res["flag2"] = mm.render_morph_md(f2)
    S = np.zeros((3, 4), bool)
    S[1, 1:3] = True
    S[0, 0] = True
    r.solid, r.density = S, S * 0.7 + 0.01
    res["outline"] = [r.outline_segments(1.5), r.cells_csv(1.5)]
    res["consts"] = [mm.VERDICTS, mm._FOS_RULE, mm._SLIVER_TOL, mm._MEMBER_INNER, mm._MEMBER_OUTER, mm.MEMBER_LABELS,
                     mm._VERDICT_LINES]
    check("morph_outputs", res)


def test_characterized_full_fans():
    """The same fans as test_load_fans, recorded whole (summary() rounds away the arithmetic)."""
    res = {}
    inner = np.asarray(HP.lower_front_inner, float)
    dirs = [inner + 300.0 * np.array([0.1, 1.0, 0.2 * k]) for k in range(-3, 4)]
    insts = [_inst(0.01 * i, 1000.0 + 300.0 * i, outer=d) for i, d in enumerate(dirs)]
    A = NS(instants=insts)
    for n in (0, 1, 3, 4, 8):
        res[f"lf_{n}"] = mm.load_fan_from_audit(A, HP, n_cases=n)
    rev = insts + [_inst(0.2, -2500.0, outer=dirs[3]), _inst(0.21, -1800.0, outer=dirs[1])]
    res["rev"] = mm.load_fan_from_audit(NS(instants=rev), HP, n_cases=3)
    sym = [_inst(0.0, 1000.0, outer=dirs[3]), _inst(0.1, -1000.0, outer=dirs[3])]
    res["sym"] = mm.load_fan_from_audit(NS(instants=sym), HP)
    bare = NS(t=0.3, ghost=NS(lower_outer=dirs[4]), margins=None, load_path_shift={})
    res["no_force"] = mm.load_fan_from_audit(NS(instants=insts[:3] + [bare]), HP)
    oop = [_inst(0.01 * i, 900.0, outer=inner + np.array([200.0 * (i % 2), 150.0, 120.0 * (i - 2)]))
           for i in range(5)]
    res["oop"] = mm.load_fan_from_audit(NS(instants=oop), HP, n_cases=2)
    pr = [_inst(0.01 * i, 800.0 + 50 * i, "PR", np.asarray(HP.rocker_pushrod) + [0.0, 200.0, -150.0 + 30 * i],
                attr="pushrod_outer") for i in range(4)]
    h2 = copy.deepcopy(HP)
    h2.rocker_pushrod = None
    res["pr"] = [mm.load_fan_from_audit(NS(instants=pr), HP, member="pr"),
                 mm.load_fan_from_audit(NS(instants=pr), h2, member="PR")]
    res["hp_outer"] = mm.load_fan_from_audit(NS(instants=[_inst(0.0, 900.0), _inst(0.1, 500.0)]), HP)
    check("morph_fan_full", res)


def test_characterized_flags_and_reports():
    import dataclasses as dc
    cnc = mm.FabricationLimits.from_shop("cnc")
    two = [_case(-70.0, 2500.0, 0.6), _case(-120.0, 1800.0, 0.4)]
    fan = mm.LoadFan("LF", two, *np.eye(3), 0.97, 50.0, 18, 2500.0, warnings=["fan warning"])
    res = {"starved": mm.morph_component(_dom(), [], cnc, fan=fan),
           "tiny": mm.morph_component(_dom(), [_case(F=0.5)], cnc, fan=fan),
           "thin_ring": mm.morph_component(dc.replace(_dom(), ring_mm=0.2), two, cnc, fan=fan, **FAST),
           "no_anchor": mm.morph_component(mm.PlateDomain(width_mm=12.0, height_mm=12.0, h_mm=2.0, anchor="bores"),
                                           two, cnc, fan=fan)}
    r = mm.morph_component(_dom(), two, cnc, **FAST)
    r2 = dc.replace(r, rounds=[dc.replace(x, audit={}) for x in r.rounds], domain_meta={}, coarsen_premium={"d_compliance_frac": -0.05, "d_filter_mm": 1.5},
                    verdict="COARSENED")
    res["summ"] = [r2.summary(), mm.render_morph_md(r2), mm.render_morph_md(r)]
    res["rho"] = [mm._PlateFE(dc.replace(_dom(), material=m)).rho for m in ("Unobtainium", "Steel 4130")]
    check("morph_flags", res)


def test_characterized_fan_edges():
    """Fallbacks and tie-breaks of load_fan_from_audit that the realistic fans never reach."""
    inner = np.asarray(HP.lower_front_inner, float)
    xy = (np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]))

    def at(deg, r=300.0):
        a = math.radians(deg)
        return inner + r * np.array([math.cos(a), math.sin(a), 0.0])
    res = {}
    base = [_inst(0.01 * i, 1000.0 + 300.0 * i, outer=at(10.0 * i)) for i in range(4)]
    bare = NS(t=0.3, ghost=NS(lower_outer=inner + np.array([0.0, 0.0, 300.0])), margins=None, load_path_shift={})
    res["no_force_oop"] = mm.load_fan_from_audit(NS(instants=base + [bare]), HP)
    no_attr = NS(t=0.05, ghost=NS(), margins={"LF": {"force_N": 700.0}}, load_path_shift=None)
    res["ghost_noattr"] = mm.load_fan_from_audit(NS(instants=[no_attr] + base), HP)
    bare_hp = NS(lower_front_inner=inner)
    res["hp_noattr"] = mm.load_fan_from_audit(NS(instants=[no_attr] + base), bare_hp)
    res["zero_first"] = mm.load_fan_from_audit(NS(instants=[_inst(0.0, 900.0, outer=inner)] + base), HP)
    res["plane_scaled"] = mm.load_fan_from_audit(NS(instants=base), HP,
                                                 plane=(np.array([2.0, 0.0, 0.0]), np.array([0.5, 3.0, 0.0])))
    sliver = [_inst(0.01 * i, 5000.0, outer=at(2.0 * i)) for i in range(5)] + [_inst(0.1, 120.0, outer=at(40.0))]
    res["sliver"] = mm.load_fan_from_audit(NS(instants=sliver), HP, n_cases=20, plane=xy)
    merge = [_inst(0.01 * i, 100.0, outer=at(0.0)) for i in range(4)] + [_inst(0.1, 350.0, outer=at(2.0))]
    res["merge"] = mm.load_fan_from_audit(NS(instants=merge), HP, n_cases=2, plane=xy)
    d = at(30.0)
    sym = [_inst(0.0, 600.0, outer=d), _inst(0.1, -1000.0, outer=d), _inst(0.2, 800.0, outer=d)]  # sum of T|T| is zero
    res["sym_argmax"] = mm.load_fan_from_audit(NS(instants=sym), HP, plane=xy)
    check("morph_fan_edges", res)


def test_characterized_morph_edges(monkeypatch):
    """Domains, roundings, fallbacks and report branches the realistic runs never reach."""
    import dataclasses as dc
    res = {}
    doms = {"neg_ring": mm.PlateDomain(width_mm=14.0, height_mm=14.0, h_mm=1.0, load_bore=(7.0, 7.0, 2.0), ring_mm=-1.0),
            "edge_anchor": mm.PlateDomain(width_mm=12.0, height_mm=12.0, h_mm=2.0, anchor="bores",
                                          anchor_bores=[(12.0, 6.0, 1.0), (0.0, 6.0, 1.0)], load_bore=(6.0, 10.0, 1.0)),
            "edge_load": mm.PlateDomain(width_mm=12.0, height_mm=12.0, h_mm=2.0, load_bore=(12.0, 6.0, 1.0))}
    for k, d in doms.items():
        fe = mm._PlateFE(d)
        res["fe_" + k] = [fe.pass_solid, fe.pass_void, fe.fixed_dofs, fe.load_nodes, fe.haz_mask(0.5), fe.haz_mask(3.0)]
    res["round"] = [mm.FabricationLimits("x", 1.23456, 2.34567, 3.45678, 0.012345, 1.98765).summary(),
                    _case(-33.3, 1234.5678, 0.4, 3, 0.01234, 0.45678).summary(),
                    mm.LoadFan("LF", [_case()], *np.eye(3), 0.97654, 12.345, 9, 2500.1234).summary()]
    fld = NS(specs={"a": NS(lo=np.array([0.0]), hi=np.array([0.4]))})
    res["limits"] = [mm.FabricationLimits.from_shop(), mm.FabricationLimits.from_tolerance_field(fld, haz_mm=0.5)]
    res["factories"] = [mm.PlateDomain.chassis_tab(bore_r_mm=10.0), mm.PlateDomain.pivot_bracket(bore_r_mm=10.0)]
    rib = np.zeros((30, 30), bool)
    rib[2:28, 14:16] = True
    haz = np.zeros((30, 30), bool)
    haz[:8] = True
    shallow = mm.FabricationLimits("x", 1.0, 0.5, 3.0, 0.1, 0.8)
    res["audit_shallow"] = mm.fabrication_audit(rib, 1.0, shallow, haz)
    cnc = mm.FabricationLimits.from_shop("cnc")
    two = [_case(-70.0, 2500.0, 0.6), _case(-120.0, 1800.0, 0.4)]
    fan = mm.LoadFan("LF", two, *np.eye(3), 0.97, 50.0, 18, 2500.0, warnings=["fan warning"])
    res["bad_dom"] = mm.morph_component(_dom(h_mm=0.0), two, cnc, fan=fan)

    def boom(*a, **k):
        raise RuntimeError("singular")
    with monkeypatch.context() as m:
        m.setattr(mm, "_simp_round", boom)
        res["simp_fail"] = mm.morph_component(_dom(), two, cnc, fan=fan)
    with monkeypatch.context() as m:
        m.setattr(mm, "_simp_round", lambda fe, cases, *a: (np.zeros((fe.ny, fe.nx)), [], 0))
        r = mm.morph_component(_dom(), two, cnc)
        res["void"] = [r.stress_dir.shape, r.stress_mag, r.solid.dtype.name]
    wide = mm.FabricationLimits("wide", 7.0, 0.0, 7.0, 0.1, 6.8)
    fine = mm.PlateDomain.chassis_tab(20.0, 24.0, 3.0, 4.0, h_mm=1.0)
    r = mm.morph_component(fine, two, wide, max_rounds=1, **FAST)
    res["fine"] = [r.summary(), r.rounds[0].filter_mm, r.mass_g]
    real = mm.fabrication_audit
    with monkeypatch.context() as m:
        m.setattr(mm, "fabrication_audit", lambda *a, **kw: dict(real(*a, **kw), ok=False, viol_frac=0.4,
                                                                 viol_haz_frac=0.0))
        res["unb_vf"] = mm.morph_component(_dom(), two, cnc, volfrac=0.6, max_rounds=1, **FAST).findings
    for k, comps in {"less": (10.0, 8.0), "same": (8.0, 8.0), "more": (8.0, 10.0)}.items():
        seq = iter(comps)
        calls = {"n": 0}

        def audit(*a, **kw):
            calls["n"] += 1
            return dict(real(*a, **kw), ok=calls["n"] > 1, viol_frac=0.2, viol_haz_frac=0.0)
        with monkeypatch.context() as m:
            m.setattr(mm, "_simp_round", lambda fe, cases, *a: (np.ones((fe.ny, fe.nx)), [next(seq)], 1))
            m.setattr(mm, "fabrication_audit", audit)
            r = mm.morph_component(_dom(), two, cnc)
        res["two_rounds_" + k] = [r.verdict, r.coarsen_premium, [f["message"] for f in r.findings]]
    h2 = copy.deepcopy(HP)
    h2.rocker_pushrod = None
    res["from_audit_pr"] = mm.morph_from_audit(NS(instants=[]), h2, member="PR", limits=cnc)
    base = mm.morph_component(_dom(), two, cnc, fan=fan, **FAST)
    shallow_r = dc.replace(base, limits=shallow)
    zero = dc.replace(base, coarsen_premium={"d_compliance_frac": 0.0, "d_filter_mm": 1.5}, verdict="COARSENED",
                      rounds=[dc.replace(x, volfrac=0.6) for x in base.rounds])
    stranger = dc.replace(base, fan=mm.LoadFan("ZZ", two, *np.eye(3), 0.97, 50.0, 18, 2500.0))
    res["render"] = [mm.render_morph_md(x) for x in (shallow_r, zero, stranger)]
    odd = dc.replace(base, rounds=[dc.replace(x, audit={"viol_frac": 0.1234567, "viol_haz_frac": 0.0123456})
                                   for x in base.rounds])
    res["odd_summary"] = odd.summary()
    check("morph_edges", json.loads(json.dumps(plain(res))))
