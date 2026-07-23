# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  ui/thermic_patch.py — the 👻🔥 ThermicPatch tab (flash-heat grip window)
# ============================================================================
"""
The tab that answers the question Ghost Topology raises but cannot: over the few
hundred milliseconds of a transient overload, how hot does the contact patch get,
and does the CORE tread temperature push grip in or out of the compound's thermal
window mid-event?

It runs the same named manoeuvre Ghost Topology does, then marches a lightweight
3-node radial thermodynamic ladder (Surface → Core → Carcass) ALONG the solved
force/slip history and scales the Pacejka peak-grip factor D by the core
temperature per instant. The headline is the worst millisecond: when, which
corner, how hot, and how many newtons of lateral force the heat costs.

All physics lives in suspension/thermic_patch.py (which reuses the thermal
parameters of suspension/tire_thermal.py so the two channels describe one tyre);
this module only orchestrates the solver and draws (see ui/__init__.py rules).

Session keys used:
    thermic_last  summary dict of the last run (for a handover / cross-tab read)
"""

from __future__ import annotations


_MANEUVERS = {
    "Step steer (J-turn)": "step_steer",
    "Snap oversteer + countersteer": "snap_oversteer",
    "Brake → throttle transition": "brake_to_throttle",
    "Curb strike": "curb_strike",
}

_CORNERS = ("FL", "FR", "RL", "RR")


def render():
    import numpy as np
    import streamlit as st
    from suspension import units as _units
    from suspension.transient import run_maneuver
    from suspension import thermic_patch as th

    ss = st.session_state

    st.subheader("👻🔥 ThermicPatch — the grip the tyre has once it's hot")
    st.caption(
        "A Pacejka force law has no temperature. So every grip number elsewhere "
        "in the tool is implicitly a single-temperature snapshot — it can't see "
        "the tyre slide itself out of its thermal window mid-corner. This tab "
        "runs a lightweight 3-node radial thermal ladder (Surface → Core → "
        "Carcass) along the transient you already solve in Ghost Topology, and "
        "scales the Magic-Formula peak-grip factor D by the CORE tread "
        "temperature at every instant. It reports the worst millisecond: how "
        "hot, which corner, and how much lateral force the heat costs.")

    # ---------------- 1 · the manoeuvre + thermal scenario ------------------
    c1, c2 = st.columns([2, 1])
    with c1:
        man_label = st.selectbox("Transient event", list(_MANEUVERS.keys()),
                                 key="thermic_man")
    with c2:
        corner_view = st.selectbox("Corner to plot", _CORNERS, index=3,
                                   key="thermic_corner",
                                   help="Which corner's temperature and traction "
                                        "curve to draw. The verdict scans all "
                                        "four regardless.")

    st.markdown("###### Thermal scenario")
    st.caption(
        "The tyre's thermal state ENTERING the event. These set where on the "
        "grip window the core starts — a tyre entering at 92 °C on a baked track "
        "has no headroom, one at 78 °C has room to heat into the sweet spot.")
    s1, s2, s3 = st.columns(3)
    init_temp = _units.uslider(s1, "Core temp entering event (°C)", 40.0, 120.0, 82.0, '°C', step=1.0, key="thermic_init", help="Uniform starting temperature of all three nodes.")
    ambient = _units.uslider(s2, "Air temp (°C)", 5.0, 45.0, 30.0, '°C', step=1.0, key="thermic_amb", help="Ambient the tread surface convects to.")
    track_temp = _units.uslider(s3, "Track surface temp (°C)", 20.0, 65.0, 42.0, '°C', step=1.0, key="thermic_track", help="The warmed asphalt the contact patch conducts "
                                "into. A baked summer track sinks far less heat.")

    with st.expander("Compound window & ladder detail (advanced)"):
        w1, w2 = st.columns(2)
        t_opt = _units.uslider(w1, "Window centre T_opt (°C)", 60.0, 110.0, 85.0, '°C', step=1.0, key="thermic_topt", help="Core temperature of PEAK grip for this compound. "
                               "Grip falls off either side of it.")
        half_w = _units.uslider(w2, "Window half-width (°C)", 15.0, 55.0, 35.0, '°C', step=1.0, is_delta=True, key="thermic_hw", help="How far from the optimum grip falls to the "
                                "floor (cold side; the hot side falls faster).")
        st.caption(
            "The grip law is a parabola centred on T_opt, equal to 1.0 there and "
            "asymmetric — overheating past the window is penalised harder than "
            "being equally cold, matching a real compound going greasy. "
            "Everything here is REPRESENTATIVE, not measured: like the co-sim "
            "thermal channel, every number this tab produces is flagged "
            "uncalibrated until you supply temperature-swept tyre data.")

    # ---------------- 2 · run ----------------------------------------------
    tp = th.default_thermal_params()
    tp.T_opt_c = float(t_opt)
    lp = th.default_ladder_params()
    lp.track_temp_c = float(track_temp)

    with st.spinner("Integrating the transient, then marching the thermal "
                    "ladder along every instant…"):
        res = run_maneuver(None, kind=_MANEUVERS[man_label])
        if not res.ok:
            st.error("Transient run flagged itself failed: "
                     + "; ".join(res.warnings[:3]))
            return
        tr = th.run_thermic_patch(res, tp=tp, lp=lp, init_temp_c=float(init_temp),
                                  ambient_c=float(ambient), gas_c=float(init_temp),
                                  half_width_c=float(half_w))
        if not tr.ok:
            st.error("ThermicPatch could not integrate: "
                     + "; ".join(tr.warnings[:3]))
            return

    # ---------------- 3 · the verdict --------------------------------------
    w = tr.worst_instant()
    breached = bool(w.get("ok") and w.get("dFy_frac", 0.0) >= 0.03)
    if breached:
        badge = "🔴" if w["dFy_frac"] >= 0.10 else "🟠"
        st.markdown(f"## {badge} Thermal grip window breached — {man_label}")
    else:
        st.markdown(f"## 🟢 Core stayed in the window — {man_label}")
    st.caption(th.verdict_sentence(tr))

    if not tr.calibrated:
        st.info("⚠️ UNCALIBRATED channel: the ladder's masses, conductances and "
                "the grip-vs-temperature law are representative FSAE-class "
                "placeholders, not measured on your tyre. Trust the SHAPE and "
                "the setup-to-setup DELTA, not the absolute temperature. Supply "
                "temperature-swept tyre data to calibrate.")

    s = tr.summary()
    m1, m2, m3, m4 = st.columns(4)
    m1.metric("Peak surface flash", f"{s['peak_surface_c']:.0f} °C",
              help="Hottest the thin surface skin reaches — the flash heat that "
                   "blisters and grains, felt before the core notices.")
    m2.metric("Peak core temp", f"{s['peak_core_c']:.0f} °C",
              delta=f"{s['peak_core_c'] - t_opt:+.0f} °C vs optimum",
              delta_color="inverse",
              help="The grip-determining band. Its distance from T_opt sets the "
                   "grip multiplier.")
    m3.metric("Min grip factor D", f"{s['min_mu_scale']:.2f}×",
              help="Lowest Pacejka peak-grip multiplier over the whole event. "
                   "1.00× is full grip; lower is thermal degradation.")
    m4.metric("Worst lateral-force loss", f"{s['max_dFy_frac']*100:.0f} %",
              delta=(f"{w.get('Fy_lost_N', 0.0):.0f} N at "
                     f"{w.get('t_s', 0.0)*1e3:.0f} ms" if breached else "in window"),
              delta_color="inverse" if breached else "off",
              help="Fraction of available lateral force lost to heat at the "
                   "worst instant, and the corner/time it happens.")

    # One inline provenance signal on the headline numbers themselves, in the
    # shared grade vocabulary, so a reader sees their epistemic status without
    # reading the banner above. Calibrated flips this from uncalibrated to a
    # modelled ±10% grade automatically.
    try:
        from suspension.provenance import confidence_note as _conf
        _conf(st, "modelled", calibrated=bool(tr.calibrated),
              extra=("3-node radial thermal ladder along the transient — a "
                     "closed-form surrogate, not a 3D FEA heat solve"),
              calibrate_with=("one TTC temperature-swept tyre run (or a single "
                              "pyrometer trace across a stint) to replace the "
                              "representative ladder params with yours"))
    except Exception:
        pass

    # publish a compact summary for any cross-tab consumer / handover
    ss["thermic_last"] = {"maneuver": man_label, **s}

    # ---------------- 4 · the traces ---------------------------------------
    ci = _CORNERS.index(corner_view)
    t_ms = tr.t * 1000.0

    tab_temp, tab_grip, tab_curve, tab_table = st.tabs(
        ["Layer temperatures", "Grip through the event",
         "Core temp vs traction", "Instant table"])

    with tab_temp:
        st.caption(
            f"The 3-node radial ladder at corner {corner_view} through the event. "
            "Surface takes the frictional flash heat and leads; Core lags and is "
            "what grip actually reads; Carcass is the slow buffer. The gap "
            "between Surface and Core IS the flash transient a steady-state "
            "temperature can't show.")
        st.line_chart(
            {"Surface (°C)": tr.T_surface[:, ci],
             "Core (°C)": tr.T_core[:, ci],
             "Carcass (°C)": tr.T_carcass[:, ci],
             "T_opt (°C)": np.full(len(t_ms), float(t_opt))},
            x_label="time through event (index; ~1 ms/step)", height=300)
        st.caption(f"Peak flash-heat flux at this corner: "
                   f"{np.max(tr.q_flash[:, ci]):,.0f} W/m² — the sliding-power "
                   "product of shear force and patch sliding speed.")

    with tab_grip:
        st.caption(
            "Left axis reads as a multiplier on peak grip. 1.00 is the cold-"
            "optimal force the Magic Formula would give at that instant; the "
            "thermal line is what the hot core actually delivers. The gap is the "
            "force the heat quietly deletes — invisible to every other tab.")
        st.line_chart(
            {"grip factor D (×)": tr.mu_scale[:, ci],
             "full grip (×)": np.ones(len(t_ms))},
            x_label="time through event (index; ~1 ms/step)", height=260)
        # absolute newtons, so the cost is legible in force not ratio
        st.line_chart(
            {"|Fy| cold-optimal (N)": tr.Fy_cold[:, ci],
             "|Fy| after heat (N)": tr.Fy_hot[:, ci]},
            x_label="time through event (index; ~1 ms/step)", height=260)

    with tab_curve:
        st.caption(
            "The live core-temp vs traction relationship the ladder walked: each "
            "point is one instant, its slip angle on x, its delivered lateral "
            "force on y, coloured by core temperature. Points that drift down-"
            "and-right of the cold envelope are instants where heat, not slip, "
            "is the thing capping the corner.")
        alpha_deg = np.degrees(np.abs(res.alpha[:, ci]))
        try:
            import pandas as pd
            df = pd.DataFrame({
                "slip angle |α| (deg)": alpha_deg,
                "lateral force |Fy| (N)": tr.Fy_hot[:, ci],
                "core temp (°C)": tr.T_core[:, ci],
            })
            st.scatter_chart(df, x="slip angle |α| (deg)",
                             y="lateral force |Fy| (N)", color="core temp (°C)",
                             height=340)
        except Exception:
            # pandas/altair colour path unavailable — fall back to a plain trace
            st.line_chart({"|Fy| after heat (N)": tr.Fy_hot[:, ci]},
                          x_label="instant", height=300)
        st.caption(
            "The window law behind the colour: grip peaks at the compound "
            f"optimum ({t_opt:.0f} °C) and falls off either side, harder when "
            "overheating than when cold.")
        # draw the window curve itself so the user sees the law, not just its effect
        T_axis = np.linspace(float(init_temp) - 25, float(init_temp) + 45, 120)
        mu_axis = th.parabolic_mu_scale(T_axis, tp, half_width_c=float(half_w))
        st.line_chart({"grip factor D (×)": mu_axis},
                      x_label=f"core temp sweep "
                              f"({T_axis[0]:.0f}…{T_axis[-1]:.0f} °C, left→right)",
                      height=220)

    with tab_table:
        st.caption("The worst ~12 instants by lateral force lost to heat — the "
                   "load cases to hand the tyre/thermal team.")
        try:
            import pandas as pd
            lost = tr.Fy_cold * tr.dFy_frac                    # newtons lost (n,4)
            flat = []
            k = min(12, lost.size)
            idx = np.argsort(lost, axis=None)[::-1][:k]
            for kf in idx:
                i, cj = np.unravel_index(kf, lost.shape)
                if lost[i, cj] < 1.0:
                    continue
                flat.append({
                    "t (ms)": round(float(tr.t[i]) * 1e3, 0),
                    "corner": _CORNERS[cj],
                    "surface (°C)": round(float(tr.T_surface[i, cj]), 1),
                    "core (°C)": round(float(tr.T_core[i, cj]), 1),
                    "grip D (×)": round(float(tr.mu_scale[i, cj]), 3),
                    "Fy lost (N)": round(float(lost[i, cj]), 0),
                })
            if flat:
                st.dataframe(pd.DataFrame(flat), use_container_width=True,
                             hide_index=True)
            else:
                st.success("No instant lost meaningful lateral force to heat — "
                           "the core stayed in the window all event.")
        except Exception as e:      # noqa: BLE001 — a table must never kill the tab
            st.caption(f"(instant table unavailable: {e})")
