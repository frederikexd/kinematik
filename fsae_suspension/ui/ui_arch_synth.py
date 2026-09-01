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

        #  Plot the SAME rows as the table above, not the raw Pareto front.
        #
        #  tradeoff_table() returns every point on the front, which includes
        #  several continuous-geometry realisations of the same discrete
        #  architecture. Plotting those put two or three markers on top of each
        #  other, each carrying an identical label, so every caption rendered
        #  doubled and offset by a few pixels — it read as a font bug rather
        #  than as duplicate data. Worse, the chart silently disagreed with the
        #  table directly above it: three rows, six points.
        #
        #  compare_architectures() is the deduplicated view — one row per
        #  discrete architecture, showing its best continuous realisation. That
        #  is what the table shows and what the caption promises, so it is what
        #  the chart should show too.
        tbl = rows if rows else tradeoff_table(res)
        xs = [r["mass_kg"] for r in tbl]
        ys = [r["points"] for r in tbl]
        txt = [f"{r['wheel_in']}\"·{r['motors']}mot·{r['pack_v']}V·{r['damper']}"
               for r in tbl]

        #  Label every point only while the labels can actually fit. A caption
        #  like 10"·2mot·400V·outboard is ~22 characters and is drawn centred
        #  above a marker, so past a handful of architectures they overlap each
        #  other horizontally no matter how the axes are padded — and an
        #  unreadable pile of overlapping text is worse than no text, because it
        #  looks broken rather than dense. Above the threshold the labels move
        #  to hover, where they are always legible and never collide.
        LABEL_LIMIT = 6
        labelled = len(xs) <= LABEL_LIMIT

        fig = go.Figure(go.Scatter(
            x=xs, y=ys,
            mode="markers+text" if labelled else "markers",
            text=txt,                      # kept either way — hover reads it
            textposition="top center", textfont=dict(size=11),
            marker=dict(size=11),
            hovertemplate="%{text}<br>mass %{x:.1f} kg · "
                          "points %{y:.1f}<extra></extra>"))

        #  Labels are drawn centred above their marker and are far wider than
        #  it, so with the old 10 px side margins the outermost captions ran off
        #  the plotting area and were clipped mid-word. Padding the DATA range
        #  rather than the margins keeps them inside the axes wherever the points
        #  happen to fall; the wider margins then stop the axis titles crowding
        #  them. Padding only exists to make room for text, so when the labels
        #  move to hover the points get the full plot area back.
        if xs and ys:
            xpad = (max(xs) - min(xs) or 1.0) * (0.18 if labelled else 0.06)
            ypad = (max(ys) - min(ys) or 1.0) * (0.22 if labelled else 0.08)
            fig.update_xaxes(range=[min(xs) - xpad, max(xs) + xpad])
            #  extra headroom at the top only when a label sits above the marker
            fig.update_yaxes(range=[min(ys) - ypad,
                                    max(ys) + ypad * (1.6 if labelled else 1.0)])

        fig.update_layout(
            xaxis_title="~Mass (kg) — parametric",
            yaxis_title="~Est. points — parametric",
            title="Pareto front (lower-left dominated; up-left preferred)",
            height=460, margin=dict(l=70, r=60, t=50, b=55),
            showlegend=False)
        st.plotly_chart(fig, use_container_width=True)
        if not labelled:
            st.caption(f"{len(xs)} architectures on the front — hover a point "
                       f"for its configuration. Labels are drawn inline at "
                       f"{LABEL_LIMIT} or fewer.")
    except Exception as e:
        st.caption(f"(plot unavailable: {e})")

    st.caption("Reminder: mass/points are model estimates. The kinematic columns "
               "are the trustworthy ones. Use the mass/points *ordering* as "
               "directional evidence, and calibrate the coefficients before you "
               "put a specific points delta on a slide.")
