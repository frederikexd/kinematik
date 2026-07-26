# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""
FlexGen engine regression tests.

Evidence the physics is right (CONTRIBUTING.md rule 4):
* Small-deflection limits pinned to Euler–Bernoulli hand calcs:
  δ = FL³/3EI, θ = FL²/2EI (fixed-free) and δ = FL³/12EI (fixed-guided).
* Buckling pinned to Euler columns: P_cr = π²EI/(K·L)², K = 2 / 1 for the
  fixed-free / fixed-guided flexure — the module COMPUTES its critical load
  from the discrete tangent Hessian; Euler is the independent cross-check.
* Energy consistency: stored strain energy must equal the work of the
  quasi-static loading path, U = ∫F dz (first law, no free lunch).
* Large-deflection sanity: the elastica stiffens — tip force at large travel
  exceeds the linear-rate prediction, and the tip forshortens axially.
"""
import math
import re

import numpy as np
import pytest

from suspension.flex import MATERIALS
from suspension.flexgen import (
    FATIGUE, BladeLoadCase, BladeSection, FlexureBlade, PRBChain,
    coilover_downsize, equivalent_spring, export_step, export_stl,
    flexgen_lint, layup_map, render_flexgen_md, synthesize_blade,
)

STEEL = MATERIALS["Steel 4130"]


def _blade(boundary="fixed-free", n_seg=24, L=80.0, b=30.0, t=2.0):
    return FlexureBlade(L, BladeSection(b, t), STEEL,
                        boundary=boundary, n_seg=n_seg)


# --------------------------------------------------------------------------- #
#  Analytic small-deflection limits
# --------------------------------------------------------------------------- #
def test_small_deflection_matches_euler_bernoulli_fixed_free():
    blade = _blade("fixed-free")
    chain = PRBChain(blade)
    E, L, I = STEEL.E, blade.length_mm, blade.section.I_at(0.0)
    F = 2.0
    st = chain.solve(0.0, F)
    assert st.tip_dy_mm == pytest.approx(F * L ** 3 / (3 * E * I), rel=5e-3)
    assert st.tip_rot_rad == pytest.approx(F * L ** 2 / (2 * E * I), rel=5e-3)


def test_small_deflection_matches_euler_bernoulli_fixed_guided():
    blade = _blade("fixed-guided")
    chain = PRBChain(blade)
    E, L, I = STEEL.E, blade.length_mm, blade.section.I_at(0.0)
    F = 5.0
    st = chain.solve(0.0, F)
    assert st.tip_dy_mm == pytest.approx(F * L ** 3 / (12 * E * I), rel=5e-3)
    assert st.tip_rot_rad == 0.0        # that's what "guided" means


def test_nonlinear_compliance_matrix_symmetric_and_consistent():
    blade = _blade("fixed-free")
    chain = PRBChain(blade)
    st = chain.solve(0.0, 2.0)
    C = st.compliance
    assert np.allclose(C, C.T)
    # dy/dFy of the tangent compliance matches the linear rate at small load
    assert C[1, 1] == pytest.approx(1.0 / blade.k_compliant_linear(), rel=0.02)
    # and it is positive definite at a stable equilibrium
    assert np.all(np.linalg.eigvalsh(C) > 0)


# --------------------------------------------------------------------------- #
#  Buckling — computed vs Euler chart
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("boundary,K", [("fixed-free", 2.0),
                                        ("fixed-guided", 1.0)])
def test_buckling_matches_euler_column(boundary, K):
    blade = _blade(boundary, n_seg=32)
    E, L, I = STEEL.E, blade.length_mm, blade.section.I_at(0.0)
    p_euler = math.pi ** 2 * E * I / (K * L) ** 2
    p_disc = PRBChain(blade).critical_axial_load_n()
    assert p_disc == pytest.approx(p_euler, rel=0.02)


def test_post_buckling_load_raises_not_lies():
    blade = _blade("fixed-free", n_seg=16)
    p_cr = PRBChain(blade).critical_axial_load_n()
    with pytest.raises(RuntimeError):
        PRBChain(blade).solve(fx_n=-1.5 * p_cr, fy_n=0.5)


# --------------------------------------------------------------------------- #
#  Strain energy / equivalent spring
# --------------------------------------------------------------------------- #
def test_strain_energy_equals_work_done():
    blade = _blade("fixed-free")
    sp = equivalent_spring(blade, 8.0, n_pts=9)
    zpos = sp.z_mm >= 0
    trap = getattr(np, "trapezoid", None) or np.trapz
    work = trap(sp.f_n[zpos], sp.z_mm[zpos])
    assert work == pytest.approx(sp.energy_nmm[-1], rel=0.02)


def test_equivalent_spring_rate_matches_linear_at_ride():
    blade = _blade("fixed-guided")
    sp = equivalent_spring(blade, 5.0, n_pts=7)
    assert sp.k_at_ride_n_mm == pytest.approx(blade.k_compliant_linear(),
                                              rel=0.03)


def test_elastica_stiffens_and_foreshortens_at_large_travel():
    blade = _blade("fixed-free", L=60.0, t=1.2)
    chain = PRBChain(blade)
    z = 20.0                                   # a third of the length — LARGE
    st = chain.solve_travel(z)
    assert st.fy_n > blade.k_compliant_linear() * z   # stiffening, not linear
    assert st.tip_dx_mm < -0.5                        # measurable pull-in


def test_axial_compression_softens_transverse_rate():
    blade = _blade("fixed-free")
    chain = PRBChain(blade)
    p_cr = chain.critical_axial_load_n()
    k_free = 1.0 / chain.solve(0.0, 1.0).compliance[1, 1]
    k_comp = 1.0 / chain.solve(-0.5 * p_cr, 1.0).compliance[1, 1]
    assert k_comp < 0.7 * k_free               # half of P_cr costs > 30 % rate


def test_coilover_downsize_arithmetic_and_refusal():
    d = coilover_downsize(30.0, 12.0, motion_ratio=0.8)
    assert d["feasible"]
    assert d["k_spring_residual_n_mm"] == pytest.approx(18.0 / 0.64)
    assert d["flex_share"] == pytest.approx(0.4)
    d2 = coilover_downsize(30.0, 45.0)
    assert not d2["feasible"] and d2["k_spring_residual_n_mm"] == 0.0


# --------------------------------------------------------------------------- #
#  Lint
# --------------------------------------------------------------------------- #
def test_lint_flags_overstressed_blade_and_passes_sane_one():
    hot = FlexureBlade(50.0, BladeSection(25.0, 3.0), STEEL,
                       boundary="fixed-guided", n_seg=16)
    bad = flexgen_lint(hot, [BladeLoadCase("bump", travel_mm=8.0)])
    assert any(f.level == "blocker" and f.code in ("YIELD", "HCF")
               for f in bad)
    cool = FlexureBlade(110.0, BladeSection(40.0, 1.0), STEEL,
                        boundary="fixed-guided", n_seg=16)
    ok = flexgen_lint(cool, [BladeLoadCase("bump", axial_n=-150.0,
                                           travel_mm=3.0)])
    assert not any(f.level == "blocker" for f in ok)
    assert any(f.code == "PASS" for f in ok)


def test_lint_buckling_blocker_uses_computed_critical_load():
    blade = _blade("fixed-free", t=1.0)
    p_cr = PRBChain(blade).critical_axial_load_n()
    f = flexgen_lint(blade, [BladeLoadCase("crush", axial_n=-1.2 * p_cr)])
    assert any(x.code == "BUCKLED" for x in f)


def test_lint_refuses_to_certify_carbon_fatigue():
    carbon = FlexureBlade(90.0, BladeSection(30.0, 1.5),
                          MATERIALS["Carbon (axial, representative)"],
                          n_seg=12)
    f = flexgen_lint(carbon, [BladeLoadCase("bump", travel_mm=3.0)])
    assert any(x.level == "blocker" and x.code == "LAMINATE_FATIGUE"
               for x in f)


def test_lint_flags_parasitically_soft_blade():
    # nearly-square section: the "stiff" plane isn't
    sq = FlexureBlade(80.0, BladeSection(6.0, 2.0), STEEL, n_seg=12)
    f = flexgen_lint(sq, [])
    assert any(x.code == "PARASITIC_SOFT" for x in f)


def test_reverse_taper_refused_at_construction():
    with pytest.raises(ValueError):
        BladeSection(30.0, 1.0, 2.0)


# --------------------------------------------------------------------------- #
#  Synthesis
# --------------------------------------------------------------------------- #
def test_synthesis_finds_feasible_blade_and_it_passes_its_own_lint():
    res = synthesize_blade("Titanium Ti-6Al-4V", width_mm=40.0,
                           travel_mm=5.0,
                           cases=[BladeLoadCase("corner", axial_n=-400.0)],
                           length_range_mm=(70.0, 120.0),
                           t_range_mm=(0.8, 3.0),
                           n_length=5, n_t=8)
    assert res.feasible and res.blade is not None
    assert not any(f.level == "blocker" for f in res.findings)
    assert res.mass_g > 0


def test_synthesis_reports_honest_infeasibility():
    res = synthesize_blade("Steel mild", width_mm=15.0, travel_mm=25.0,
                           cases=[BladeLoadCase("corner", axial_n=-3000.0)],
                           length_range_mm=(40.0, 60.0),
                           t_range_mm=(2.0, 6.0), n_length=3, n_t=4)
    assert not res.feasible and res.blade is None
    assert "No (L, t)" in res.message


# --------------------------------------------------------------------------- #
#  Exports
# --------------------------------------------------------------------------- #
def test_stl_is_watertight():
    stl = export_stl(_blade())
    assert stl.count("facet normal") == 12
    tris = re.findall(r"outer loop\n((?:\s+vertex [^\n]+\n){3})", stl)
    edges = {}
    for t in tris:
        pts = tuple(tuple(v.split()[1:4]) for v in t.strip().splitlines())
        for i in range(3):
            key = frozenset((pts[i], pts[(i + 1) % 3]))
            edges[key] = edges.get(key, 0) + 1
    # watertight manifold: every edge shared by exactly two facets
    assert all(c == 2 for c in edges.values())


def test_step_file_is_structurally_sound():
    step = export_step(_blade())
    assert step.startswith("ISO-10303-21;")
    assert step.rstrip().endswith("END-ISO-10303-21;")
    assert "MANIFOLD_SOLID_BREP" in step and "CLOSED_SHELL" in step
    assert step.count("ADVANCED_FACE(") == 6
    assert step.count("EDGE_CURVE") == 12          # shared edges, not 24
    assert ".MILLI.,.METRE." in step               # mm units declared
    ids = {int(m) for m in re.findall(r"^#(\d+)=", step, re.M)}
    refs = {int(m) for m in re.findall(r"#(\d+)", step)}
    assert refs <= ids                             # no dangling references


def test_layup_map_routes_by_material_and_is_labeled_guidance():
    ti = layup_map(_blade())
    assert "additive_ti" in ti and "guidance" in ti
    carbon_blade = FlexureBlade(90.0, BladeSection(30.0, 1.5),
                                MATERIALS["Carbon (axial, representative)"])
    cf = layup_map(carbon_blade)
    assert "carbon_layup" in cf and "pm45_frac" in cf
    # ±45 share decreases root -> tip
    rows = [r for r in cf.splitlines() if r and not r.startswith("#")
            and not r.startswith("s_mm")]
    pm45 = [float(r.split(",")[-1]) for r in rows]
    assert pm45[0] > pm45[-1]


def test_report_renders_and_carries_the_validation_warning():
    blade = _blade("fixed-guided")
    sp = equivalent_spring(blade, 4.0, n_pts=5)
    md = render_flexgen_md(blade, sp,
                           flexgen_lint(blade, [BladeLoadCase("b",
                                                              travel_mm=4.0)]),
                           coilover_downsize(200.0, sp.k_at_ride_n_mm))
    assert "FlexGen blade" in md and "ANSYS" in md
    assert "buckling load" in md


# --------------------------------------------------------------------------- #
#  Package surface
# --------------------------------------------------------------------------- #
def test_public_api_reachable_via_lazy_package():
    import suspension
    assert suspension.FlexureBlade is FlexureBlade
    assert suspension.flexgen_lint is flexgen_lint
    assert "flexgen" in dir(suspension)


def test_fatigue_table_covers_every_flex_material():
    assert set(FATIGUE) == set(MATERIALS)
