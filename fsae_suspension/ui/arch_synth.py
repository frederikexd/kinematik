# ============================================================================
#  KinematiK — ui/arch_synth.py
#  Streamlit panel for the Architecture Synthesis co-optimizer. Lets a lead set
#  a points target, toggle which discrete switches and continuous geometry vars
#  are in play, run the mixed-variable NSGA-II, and read the Pareto trade-off —
#  with the physics/parametric provenance shown up front.
# ============================================================================
"""Render the Architecture Synthesis tab.

Design intent: the panel never hides that the points/mass axes are a calibratable
model. The provenance banner is the first thing the user sees, and the results
table separates physics-fed columns (camber gain, bump steer, scrub) from the
parametric ones (mass, points). This is what makes it defensible in a design
review rather than a slick-but-hollow "auto-optimizer".
"""

from __future__ import annotations

import numpy as np

try:
    import streamlit as st
except Exception:                       # allow import in headless tests
    st = None

from suspension.arch_synth import (
    ArchitectureProblem, PointsModel, MassModel,
    default_discrete_space, default_continuous_space,
    synthesize, compare_architectures, tradeoff_table, PROVENANCE,
)
from suspension.kinematics import Hardpoints


def render(base_hp: Hardpoints | None = None):
    if st is None:
        raise RuntimeError("streamlit not available")
    ss = st.session_state

    st.subheader("🧬📐 Architecture Synthesis — discrete + continuous, together")
    st.caption(
        "Search wheel size, motor count, pack voltage and damper layout AT THE "
        "SAME TIME as the continuous corner geometry, and get the Pareto-optimal "
        "*set* of architectures — not one 'winner'. The kinematic axes are solved "
        "by KinematiK's real corner solver; the mass/points axes are an editable "
        "model you calibrate."
    )

    st.info(
        "**Read this before quoting a number.** "
        + PROVENANCE["note"], icon="⚖️"
    )

    # ---- what's in play -------------------------------------------------
    disc_all = default_discrete_space()
    cont_all = default_continuous_space()

    with st.expander("Discrete switches in play", expanded=True):
        chosen_disc = []
        cols = st.columns(len(disc_all))
        for c, d in zip(cols, disc_all):
            with c:
                on = st.checkbox(d.label, value=True, key=f"arch_disc_{d.name}")
                st.caption("· ".join(str(o) for o in d.options))
                if on:
                    chosen_disc.append(d)
        if not chosen_disc:
            st.warning("Enable at least one discrete switch, or the search is "
                       "purely continuous (use InverseGenesis for that).")

    with st.expander("Continuous geometry variables in play", expanded=False):
        chosen_cont = []
        for cv in cont_all:
            on = st.checkbox(f"{cv.label}  ({cv.lo:.0f}–{cv.hi:.0f} mm)",
                             value=True, key=f"arch_cont_{cv.name}")
            if on:
                chosen_cont.append(cv)

    # ---- economic model (transparent, editable) -------------------------
    with st.expander("Points & mass model coefficients (PARAMETRIC — edit me)",
                     expanded=False):
        st.caption("These are the numbers a design judge will probe. They are "
                   "defensible first-order estimates, not measurements. Tune to "
                   "your own BOM and lap-sim.")
        pm = PointsModel()
        mm = MassModel()
        c1, c2 = st.columns(2)
        with c1:
            pm.per_kg = st.number_input("Points lost per kg over baseline",
                                        value=float(pm.per_kg), step=0.05,
                                        key="arch_perkg")
            pm.bumpsteer_pts_per_deg = st.number_input(
                "Points per deg of bump steer", value=float(pm.bumpsteer_pts_per_deg),
                step=0.5, key="arch_bspts")
            pm.camber_gain_target_deg = st.number_input(
                "Target camber gain (deg / 25 mm)",
                value=float(pm.camber_gain_target_deg), step=0.1, key="arch_cgt")
        with c2:
            mm.base_kg = st.number_input("Baseline full-car mass (kg)",
                                         value=float(mm.base_kg), step=1.0,
                                         key="arch_basekg")
            pm.baseline_kg = mm.base_kg

    # ---- run controls ---------------------------------------------------
    r1, r2, r3 = st.columns([1, 1, 1])
    with r1:
        pop = st.slider("Population", 12, 80, 40, key="arch_pop")
    with r2:
        gens = st.slider("Generations", 5, 60, 25, key="arch_gen")
    with r3:
        seed = st.number_input("Seed (deterministic)", value=0, step=1,
                               key="arch_seed")
    st.caption(f"≈ {pop*gens} real kinematic solves. Same seed → identical front.")

    if st.button("Run architecture synthesis", type="primary",
                 key="arch_run", disabled=not chosen_disc):
        prob = ArchitectureProblem(
            discrete=chosen_disc, continuous=chosen_cont,
            base_hp=base_hp or Hardpoints.default(),
            points_model=pm, mass_model=mm)
        bar = st.progress(0.0, text="Optimising architecture…")
        res = synthesize(prob, pop_size=int(pop), generations=int(gens),
                         seed=int(seed),
                         progress=lambda g, G: bar.progress(g / G,
                                    text=f"Generation {g}/{G}"))
        bar.empty()
        ss["arch_result"] = res

    res = ss.get("arch_result")
    if res is None:
        st.caption("Set your switches and run to see the Pareto front.")
        return

    # ---- results --------------------------------------------------------
    st.markdown("#### Non-dominated architectures")
    st.caption("Each row is an architecture no other beats on all axes at once "
               "(best continuous geometry shown per architecture). Physics-fed "
               "columns are marked ⚙️; parametric columns are marked ~.")
    rows = compare_architectures(res)
    if rows:
        import pandas as pd  # pandas is already a streamlit transitive dep
        df = pd.DataFrame(rows)
        rename = {
            "wheel_in": "Wheel(in)", "motors": "Motors", "pack_v": "Volt",
            "damper": "Damper", "mass_kg": "~Mass(kg)", "points": "~Points",
            "camber_gain_deg": "⚙️CbrGain°", "bumpsteer_deg": "⚙️BumpStr°",
            "scrub_mm": "⚙️Scrub(mm)", "feasible": "OK",
        }
        df = df.rename(columns=rename)
        st.dataframe(df, use_container_width=True, hide_index=True)

    # scatter: the actual trade surface
    try:
        import plotly.graph_objects as go
        tbl = tradeoff_table(res)
        xs = [r["mass_kg"] for r in tbl]
        ys = [r["points"] for r in tbl]
        txt = [f"{r['wheel_in']}\"·{r['motors']}mot·{r['pack_v']}V·{r['damper']}"
               for r in tbl]
        fig = go.Figure(go.Scatter(
            x=xs, y=ys, mode="markers+text", text=txt, textposition="top center",
            marker=dict(size=11)))
        fig.update_layout(
            xaxis_title="~Mass (kg) — parametric",
            yaxis_title="~Est. points — parametric",
            title="Pareto front (lower-left dominated; up-left preferred)",
            height=440, margin=dict(l=10, r=10, t=40, b=10))
        st.plotly_chart(fig, use_container_width=True)
    except Exception as e:
        st.caption(f"(plot unavailable: {e})")

    st.caption("Reminder: mass/points are model estimates. The kinematic columns "
               "are the trustworthy ones. Use the mass/points *ordering* as "
               "directional evidence, and calibrate the coefficients before you "
               "put a specific points delta on a slide.")
