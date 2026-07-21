# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  ui/flexgen.py — the 🌿🔩 FlexGen tab (compliant kinematics synthesizer)
# ============================================================================
"""
The tab that designs the joint that isn't there: a monolithic flexure blade
replacing a ball joint — zero friction, zero slop, nothing to grease — and
tells you what that costs in stored strain energy, fatigue margin and
buckling headroom, in milliseconds instead of an FEA queue.

All physics lives in suspension/flexgen.py (a pseudo-rigid-body /
discretized-elastica chain with analytic gradient and Hessian — the Hessian
IS the tangent stiffness, so the non-linear compliance matrix and the
buckling load are computed, never chart-looked-up); this module only
orchestrates the solver and draws (see ui/__init__.py rules).

Session keys used:
    flexgen_last   summary dict of the last run (for handover / cross-tab read)
"""

from __future__ import annotations


def _plt():
    """Return matplotlib.pyplot (Agg) if it's installed, else None.

    Plotting is a *convenience*, not the product: every number, table and
    export in this tab comes from suspension/flexgen.py (pure numpy). If the
    deployment image ships without matplotlib, the tab must still deliver all
    of that rather than crash on a missing optional dependency — so callers
    check for None and skip the picture, keeping the physics.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        return plt
    except Exception:               # noqa: BLE001 — any import failure = no plot
        return None


def _fig_sweep(spring, blade):
    plt = _plt()
    if plt is None:
        return None

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.6, 3.4))
    ax1.plot(spring.z_mm, spring.f_n, lw=2)
    lin = blade.k_compliant_linear() * spring.z_mm
    ax1.plot(spring.z_mm, lin, ls="--", lw=1, alpha=0.6,
             label="linear rate (small-deflection)")
    ax1.set_xlabel("tip travel z [mm]")
    ax1.set_ylabel("restoring force [N]")
    ax1.set_title("Non-linear load–travel (elastica)")
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.3)

    ax2.plot(spring.z_mm, spring.k_n_mm, lw=2, color="tab:green")
    ax2.axhline(spring.k_at_ride_n_mm, ls=":", lw=1, color="tab:green")
    ax2b = ax2.twinx()
    ax2b.plot(spring.z_mm, spring.energy_nmm / 1000.0, lw=1.5,
              color="tab:orange")
    ax2b.set_ylabel("stored strain energy [J]", color="tab:orange")
    ax2.set_xlabel("tip travel z [mm]")
    ax2.set_ylabel("tangent rate dF/dz [N/mm]", color="tab:green")
    ax2.set_title("Equivalent-spring rate & strain energy")
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    return fig


def _fig_blade(blade, state):
    import numpy as np
    plt = _plt()
    if plt is None:
        return None

    fig, ax = plt.subplots(figsize=(6.4, 2.6))
    # undeformed centreline
    ax.plot([0, blade.length_mm], [0, 0], color="0.7", lw=1, ls="--")
    # deformed chain
    chainx, chainy = [0.0], [0.0]
    x, y = 0.0, 0.0
    import suspension.flexgen as fg
    ch = fg.PRBChain(blade)
    x = ch.x0
    chainx.append(x)
    chainy.append(0.0)
    for L, phi in zip(ch.lens, state.phis):
        x += L * np.cos(phi)
        y += L * np.sin(phi)
        chainx.append(x)
        chainy.append(y)
    ax.plot(chainx, chainy, lw=3, color="tab:blue")
    ax.scatter(chainx[1:-1], chainy[1:-1], s=8, color="tab:blue", zorder=3)
    ax.scatter([chainx[-1]], [chainy[-1]], s=45, color="tab:red", zorder=4,
               label=f"tip: {state.tip_dy_mm:.2f} mm, "
                     f"pull-in {abs(state.tip_dx_mm):.2f} mm")
    ax.axvline(0, color="k", lw=4)
    ax.set_aspect("equal")
    ax.legend(fontsize=8, loc="upper left")
    ax.set_title("Deformed blade at full bump (to scale)")
    ax.grid(alpha=0.25)
    fig.tight_layout()
    return fig


def render():
    import streamlit as st

    from suspension.flex import MATERIALS
    import suspension.flexgen as fg

    # --- deployment integrity guard --------------------------------------- #
    # This tab and suspension/flexgen.py ship together. If the running image
    # has a stale or partial copy of the physics module (an old .pyc, a
    # half-synced deploy), a bare fg.FlexureBlade(...) later would fail with a
    # cryptic "module has no attribute 'FlexureBlade'". Name the real cause
    # once, up front, instead — the fix is redeploying the module, not editing
    # inputs. (See suspension/flexgen.py; every symbol below is module-level.)
    _required = ("FlexureBlade", "BladeSection", "BladeLoadCase", "PRBChain",
                 "equivalent_spring", "flexgen_lint", "coilover_downsize",
                 "export_stl", "export_step", "layup_map",
                 "render_flexgen_md")
    _missing = [n for n in _required if not hasattr(fg, n)]
    if _missing:
        st.error(
            "FlexGen's physics module is out of date in this deployment: "
            f"`suspension/flexgen.py` is missing {', '.join(_missing)}. "
            "The UI and the solver ship as a pair — redeploy the current "
            "`suspension/flexgen.py` (and clear any stale "
            "`suspension/__pycache__/flexgen.*.pyc`). Loaded module: "
            f"`{getattr(fg, '__file__', '?')}`."
        )
        return

    st.markdown("### 🌿🔩 FlexGen — compliant kinematics synthesizer")
    st.caption(
        f"build flexgen-2 · module `{getattr(fg, '__file__', '?')}`"
    )
    st.caption(
        "Replace a ball joint with a monolithic flexure blade: zero friction, "
        "zero slop, maintenance-free — priced honestly in strain energy, "
        "fatigue margin and buckling headroom. Pseudo-rigid-body / elastica "
        "solve in milliseconds; the tangent Hessian gives the non-linear "
        "compliance matrix and the buckling load directly."
    )

    with st.expander("What this is (and is not)", expanded=False):
        st.markdown(
            "- **Is**: a planar large-deflection blade solver — non-linear "
            "load–travel, strain-energy → equivalent-spring accounting, a "
            "Goodman fatigue + computed-buckling linter, and STEP/STL/"
            "orientation-map export of the blade blank.\n"
            "- **Is not**: a 6-DOF corner model or a fatigue-life prediction. "
            "The stiff plane and torsion are reported as linear parasitic "
            "ratios; fatigue uses smooth-specimen data (your edge finish "
            "halves it — the lint says so). Validate finalists in ANSYS with "
            "large deflection ON, and coupon-test the production edge."
        )

    c1, c2, c3 = st.columns(3)
    with c1:
        mat_name = st.selectbox("Material", list(MATERIALS), index=0,
                                key="fxg_mat")
        boundary = st.selectbox(
            "Blade boundary", ["fixed-guided", "fixed-free"], index=0,
            key="fxg_bnd",
            help="fixed-guided = parallel-blade flexure keeping the upright "
                 "square (Euler K=1); fixed-free = single blade replacing one "
                 "ball joint (K=2).")
    with c2:
        L = st.number_input("Blade length L [mm]", 20.0, 300.0, 90.0, 5.0,
                            key="fxg_L")
        b = st.number_input("Blade width b [mm] (stiff direction)", 5.0,
                            120.0, 35.0, 1.0, key="fxg_b")
    with c3:
        t_root = st.number_input("Root thickness t [mm]", 0.3, 12.0, 1.5,
                                 0.1, key="fxg_tr")
        taper = st.slider("Taper t_tip/t_root", 0.4, 1.0, 0.75, 0.05,
                          key="fxg_tp",
                          help="Thinning toward the tip moves peak strain "
                               "away from the clamp.")

    c4, c5, c6 = st.columns(3)
    with c4:
        travel = st.number_input("Bump/droop travel ± [mm]", 0.5, 60.0, 6.0,
                                 0.5, key="fxg_z")
    with c5:
        axial = st.number_input(
            "Axial load at max corner [N] (− = compression)",
            -20000.0, 20000.0, -600.0, 50.0, key="fxg_ax")
    with c6:
        k_target = st.number_input("Target wheel rate [N/mm]", 1.0, 500.0,
                                   35.0, 1.0, key="fxg_kt")
        mr = st.number_input("Motion ratio (wheel/spring)", 0.3, 2.0, 1.0,
                             0.05, key="fxg_mr")

    run = st.button("Run FlexGen", type="primary", key="fxg_run")
    if not run and "flexgen_last" not in st.session_state:
        st.info("Set the blade and the corner loads, then **Run FlexGen**.")
        return

    if run:
        blade = fg.FlexureBlade(
            float(L), fg.BladeSection(float(b), float(t_root),
                                      float(t_root) * float(taper)),
            MATERIALS[mat_name], boundary=boundary, name="flexgen_blade")
        cases = [fg.BladeLoadCase("max corner + full bump",
                                  axial_n=float(axial),
                                  travel_mm=float(travel))]
        try:
            with st.spinner("Solving the elastica sweep…"):
                spring = fg.equivalent_spring(blade, float(travel),
                                              axial_preload_n=float(axial),
                                              n_pts=11)
                chain = fg.PRBChain(blade)
                full = chain.solve_travel(float(travel), fx_n=float(axial))
                findings = fg.flexgen_lint(blade, cases)
                p_cr = chain.critical_axial_load_n()
        except RuntimeError as exc:
            st.error(f"⛔ {exc}")
            st.caption("That refusal is the result: this blade has no stable "
                       "equilibrium at these loads. Thicken it, shorten it, "
                       "or unload it.")
            return
        down = fg.coilover_downsize(float(k_target), spring.k_at_ride_n_mm,
                                    float(mr))
        st.session_state["flexgen_last"] = {
            "blade": blade, "spring": spring, "full": full,
            "findings": findings, "down": down, "p_cr": p_cr,
        }

    last = st.session_state["flexgen_last"]
    blade, spring = last["blade"], last["spring"]
    full, findings = last["full"], last["findings"]
    down, p_cr = last["down"], last["p_cr"]

    m1, m2, m3, m4 = st.columns(4)
    m1.metric("Rate at ride", f"{spring.k_at_ride_n_mm:.1f} N/mm",
              help="Tangent dF/dz at z = 0 — the flexure's own wheel-rate "
                   "contribution (the 'equivalent air spring').")
    m2.metric("Strain energy @ full travel",
              f"{spring.energy_at_full_j * 1000:.0f} N·mm")
    m3.metric("Buckling load (computed)", f"{p_cr:.0f} N",
              delta=(f"margin {p_cr / abs(spring.axial_preload_n):.2f}×"
                     if spring.axial_preload_n < 0 else "no compression"),
              help="Smallest tangent-stiffness eigen-load of the discrete "
                   "blade — not an effective-length chart.")
    m4.metric("Blade mass", f"{blade.mass_g():.1f} g")

    _sweep_fig = _fig_sweep(spring, blade)
    _blade_fig = _fig_blade(blade, full)
    if _sweep_fig is not None and _blade_fig is not None:
        st.pyplot(_sweep_fig, use_container_width=True)
        st.pyplot(_blade_fig, use_container_width=True)
    else:
        st.caption(
            "📉 Plots are unavailable in this deployment (matplotlib is not "
            "installed) — every number, the compliance matrix and all exports "
            "below are unaffected. `pip install matplotlib` to restore the "
            "load–travel, equivalent-spring and deformed-blade figures."
        )

    # ---------------- coilover downsizing ---------------- #
    st.markdown("#### Coilover downsizing")
    if down["feasible"]:
        st.success(
            f"The blades supply **{down['flex_share'] * 100:.0f} %** of the "
            f"{down['k_wheel_target_n_mm']:.0f} N/mm wheel-rate target — fit "
            f"a **{down['k_spring_residual_n_mm']:.1f} N/mm** physical spring "
            "(at the spring, motion ratio applied) instead of the full-rate "
            "coilover."
        )
    else:
        st.error(
            "⛔ The blades ALONE exceed the wheel-rate target: the corner is "
            "over-sprung with **no coilover fitted**. Soften the blade "
            "(thinner, longer) or raise the target."
        )

    # ---------------- lint ---------------- #
    st.markdown("#### Fatigue & buckling lint")
    badge = {"ok": "✅", "warn": "⚠️", "blocker": "⛔"}
    for f in findings:
        st.markdown(f"{badge[f.level]} `{f.code}` — {f.detail}")

    # ---------------- non-linear compliance matrix ---------------- #
    with st.expander("Non-linear compliance matrix at full bump "
                     "(tangent, tip frame)"):
        import pandas as pd
        C = full.compliance
        st.dataframe(pd.DataFrame(
            C, index=["u_axial", "u_travel", "rot"],
            columns=["F_axial [N]", "F_travel [N]", "M [N·mm]"]).style.format(
                "{:.3e}"), use_container_width=True)
        st.caption("C = J·H⁻¹·Jᵀ at the solved operating point — mm/N, "
                   "mm/N·mm, rad/N. Symmetric by construction; compare with "
                   "the rigid solver's zero and the linear flex.py value to "
                   "see what large deflection changes.")

    # ---------------- exports ---------------- #
    st.markdown("#### Export")
    e1, e2, e3, e4 = st.columns(4)
    e1.download_button("⬇️ STEP (blade blank)", fg.export_step(blade),
                       file_name=f"{blade.name}.step",
                       mime="model/step", key="fxg_dl_step")
    e2.download_button("⬇️ STL (printable)", fg.export_stl(blade),
                       file_name=f"{blade.name}.stl",
                       mime="model/stl", key="fxg_dl_stl")
    e3.download_button("⬇️ Orientation map (CSV)", fg.layup_map(blade),
                       file_name=f"{blade.name}_orientation.csv",
                       mime="text/csv", key="fxg_dl_csv")
    e4.download_button("⬇️ Review report (md)",
                       fg.render_flexgen_md(blade, spring, findings, down),
                       file_name=f"{blade.name}_flexgen.md",
                       mime="text/markdown", key="fxg_dl_md")
    st.caption(
        "The STEP/STL is the tapered blade **blank** (faceted planar B-rep — "
        "opens as a solid in FreeCAD/SolidWorks). Fillets, bosses and infill "
        "are your CAD's job; the orientation map is fibre/build **guidance**, "
        "not a certified schedule. Validate in ANSYS before cutting."
    )
