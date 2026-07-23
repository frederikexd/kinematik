# ============================================================================
#  KinematiK — ui/degradation.py
#  Streamlit panel for the Transient Degradation Synthesis solver. Shows how the
#  corner drifts across a long run under differential heat, bushing compliance
#  and tyre grip decay — with every layer's provenance (physics / calibratable /
#  empirical) stated up front, and the honest magnitude finding surfaced.
# ============================================================================
"""Render the Transient Degradation tab.

Design intent mirrors the rest of KinematiK: the panel never lets the empirical
tyre curve masquerade as solved physics. The provenance banner leads, the drift
plot separates thermal from compliance contributions, and the summary states
which mechanism actually dominates rather than assuming the dramatic one.
"""

from __future__ import annotations

import numpy as np

try:
    import streamlit as st
except Exception:
    st = None

from suspension.kinematics import Hardpoints
from suspension.degradation import (
    DegradationSolver, DegradationConfig, ThermalRamp, ThermalExpansionModel,
    TyreThermalModel, TyreThermalRamp, tolerance_sensitivity, PROVENANCE,
    THERMAL_ALPHA,
)


def render(base_hp: Hardpoints | None = None, vehicle=None):
    if st is None:
        raise RuntimeError("streamlit not available")
    ss = st.session_state
    hp = base_hp or Hardpoints.default()

    st.subheader("🌡️🔧 Transient Degradation — the Lap-15 car, not the CAD car")
    st.caption(
        "How your alignment drifts across a long run as the subframe heats "
        "(thermal expansion), the bushings flex under cornering load (structural "
        "compliance), and the tyres go off (grip decay). Closed-form layers over "
        "the real kinematics solver — seconds, not a 6-hour co-sim."
    )

    st.info("**Provenance.** " + PROVENANCE["note"], icon="⚖️")

    # ---- thermal inputs -------------------------------------------------
    with st.expander("Thermal model (subframe heat-soak)", expanded=True):
        c1, c2, c3, c4 = st.columns(4)
        with c1:
            T_amb = st.number_input("Ambient °C", value=20.0, key="deg_tamb")
        with c2:
            T_soak = st.number_input("Soak °C (hot metal)", value=55.0,
                                     key="deg_tsoak")
        with c3:
            tau = st.number_input("Time const (min)", value=6.0, key="deg_tau")
        with c4:
            run_min = st.number_input("Run length (min)", value=25.0,
                                      key="deg_run")
        c5, c6 = st.columns(2)
        with c5:
            mat = st.selectbox("Subframe material (α)", list(THERMAL_ALPHA.keys()),
                               key="deg_mat")
        with c6:
            grad = st.number_input("Thermal gradient length (mm)", value=350.0,
                                   step=25.0, key="deg_grad",
                                   help="Smaller = sharper differential heating "
                                        "near the pack. This differential is the "
                                        "real source of thermal misalignment.")
        st.caption(f"α({mat}) = {THERMAL_ALPHA[mat]*1e6:.1f} ×10⁻⁶ /°C. "
                   "First-principles. Soak temp & time constant are calibratable "
                   "(thermocouple, not CAD).")

    # ---- compliance + tyre ---------------------------------------------
    with st.expander("Compliance & tyre models", expanded=False):
        c1, c2 = st.columns(2)
        with c1:
            st.markdown("**Compliance (δ=F/k)** — physics form, calibratable k")
            lat_g = st.slider("Cornering case (lateral g)", 0.5, 2.0, 1.4, 0.1,
                              key="deg_latg")
            od = st.number_input("Tube OD (mm)", value=19.05, key="deg_od")
            wall = st.number_input("Tube wall (mm)", value=0.9, key="deg_wall")
        with c2:
            st.markdown("**Tyre grip vs temp** — ⚠️ EMPIRICAL, calibrate to TTC")
            t_peak = st.number_input("Peak-grip temp °C", value=80.0,
                                     key="deg_tpeak")
            t_width = st.number_input("Window half-width °C", value=45.0,
                                      key="deg_twidth")
            t_settle = st.number_input("Carcass settle temp °C", value=95.0,
                                       key="deg_tsettle")
            st.caption("The default curve is a placeholder shape. Fit it to your "
                       "own tyre data before quoting a grip loss.")

    if st.button("Run degradation", type="primary", key="deg_run_btn"):
        thermal = ThermalRamp(T_amb=T_amb, T_soak=T_soak, tau_min=tau,
                              run_min=run_min)
        expansion = ThermalExpansionModel(material=mat, gradient_len=grad)
        tyre_grip = TyreThermalModel(T_peak=t_peak, width=t_width)
        tyre_temp = TyreThermalRamp(T_amb=T_amb, T_settle=t_settle)
        cfg = DegradationConfig(lateral_g=lat_g, tube_od_mm=od, tube_wall_mm=wall)
        solver = DegradationSolver(hp, thermal=thermal, expansion=expansion,
                                   tyre_grip=tyre_grip, tyre_temp=tyre_temp,
                                   config=cfg, vehicle=vehicle)
        with st.spinner("Solving transient degradation…"):
            ss["deg_curve"] = solver.run()
            ss["deg_tol"] = tolerance_sensitivity(hp, solver=solver)

    curve = ss.get("deg_curve")
    if curve is None:
        st.caption("Set the models and run to see the Lap-15 degradation.")
        return

    # ---- summary --------------------------------------------------------
    s = curve.lap15_summary()
    st.markdown("#### Lap-15 alignment drift, split by cause")
    m1, m2, m3, m4 = st.columns(4)
    m1.metric("Camber drift", f"{s['camber_drift_deg']:+.3f}°",
              help="total, cold → hot+loaded")
    m2.metric("Toe drift", f"{s['toe_drift_deg']:+.3f}°")
    m3.metric("ΔT final", f"{s['delta_T_final']:.0f}°C")
    m4.metric("Tyre grip", f"{s['grip_mult_cold']:.2f} → {s['grip_mult_hot']:.2f}")

    st.caption(f"**Dominant mechanism:** {s['dominant_mechanism']}. "
               f"Compliance contributes {s['camber_compliance_deg']:+.3f}° camber / "
               f"{s['toe_compliance_deg']:+.3f}° toe; thermal expansion contributes "
               f"{s['camber_thermal_deg']:+.3f}° / {s['toe_thermal_deg']:+.3f}° at "
               f"the static datum (near-zero — uniform expansion is angle-preserving) "
               f"and {s['camber_gain_thermal_shift_deg']:+.4f}° of camber-gain shift "
               f"through travel.")

    # ---- drift plot -----------------------------------------------------
    try:
        import plotly.graph_objects as go
        a = curve.as_arrays()
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=a["t_min"], y=a["camber_drift"],
                                 name="camber drift (total)", mode="lines+markers"))
        fig.add_trace(go.Scatter(x=a["t_min"], y=a["toe_drift"],
                                 name="toe drift (total)", mode="lines+markers"))
        fig.add_trace(go.Scatter(x=a["t_min"], y=a["camber_thermal_drift"],
                                 name="camber (thermal only)", line=dict(dash="dot")))
        fig.update_layout(xaxis_title="run time (min)",
                          yaxis_title="drift from cold nominal (deg)",
                          height=380, margin=dict(l=10, r=10, t=30, b=10),
                          title="Alignment drift across the run")
        st.plotly_chart(fig, use_container_width=True)
    except Exception as e:
        st.caption(f"(plot unavailable: {e})")

    # ---- lap-time proxy -------------------------------------------------
    p = curve.laptime_delta_proxy()
    st.markdown("#### Cornering-capability loss at Lap 15 (proxy)")
    st.caption(p["note"])
    cc1, cc2, cc3 = st.columns(3)
    cc1.metric("Tyre grip loss", f"{p['grip_loss_tyre_pct']:.1f}%")
    cc2.metric("Alignment grip loss", f"{p['grip_loss_alignment_pct']:.1f}%")
    cc3.metric("Total", f"{p['grip_loss_total_pct']:.1f}%")

    # ---- tolerance sensitivity map -------------------------------------
    tm = ss.get("deg_tol")
    if tm is not None:
        st.markdown("#### Tolerance sensitivity map — where to spend CNC time")
        st.caption("How much Lap-15 alignment moves per mm of build error "
                   "(finite-difference through the real solver) plus each point's "
                   "thermal shift. Chase the top rows with tight tolerance; the "
                   "rest can be hand-welded.")
        try:
            import pandas as pd
            df = pd.DataFrame(tm.table())
            df = df.rename(columns={
                "point": "Hardpoint",
                "build_sens_deg_per_mm": "Sens (°/mm)",
                "thermal_shift_mm": "Thermal shift (mm)",
                "alignment_move_deg": "Move @1mm (°)",
                "combined_score": "Score",
                "recommendation": "Build",
            })
            st.dataframe(df[["Hardpoint", "Sens (°/mm)", "Thermal shift (mm)",
                             "Move @1mm (°)", "Build"]],
                         use_container_width=True, hide_index=True)
            crit = ", ".join(r["point"] for r in tm.critical_points(2))
            st.success(f"**Tightest tolerance needed at:** {crit} "
                       "(highest leverage on Lap-15 wheel angle).", icon="🎯")
        except Exception as e:
            st.caption(f"(table unavailable: {e})")
