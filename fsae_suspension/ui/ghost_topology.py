# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  ui/ghost_topology.py — the 👻🔩 Ghost Topology tab (transient compliance)
# ============================================================================
"""
The tab that shows the geometry the car ACTUALLY has mid-event: pick a
transient manoeuvre, and the engine solves the deformed suspension state at
every audited instant — the compliance-shifted camber/toe, the migrating
load paths, the transient FoS of every link, and the measured tyre-force
feedback gain (1.0 = compliance-induced instability). Verdicts:
FEEDBACK_DIVERGENT / COMPLIANCE_INVERTED / MARGIN_BREACHED /
COMPLIANCE_DEGRADED / RIGID_FAITHFUL.

All physics lives in suspension/ghost_topology.py (and the modules it joins:
kinematics, loadpath, compliance, transient). This module only orchestrates
and draws (see ui/__init__.py rules).

Session keys used:
    hardpoints        the live hardpoint dict, if the kinematics tab set one
    ghost_last_audit  summary dict of the last audit (for the handover report)
"""

from __future__ import annotations


_MANEUVERS = {
    "Step steer (J-turn)": "step_steer",
    "Snap oversteer + countersteer": "snap_oversteer",
    "Brake → throttle transition": "brake_to_throttle",
    "Curb strike": "curb_strike",
}

_VERDICT_BLURB = {
    "FEEDBACK_DIVERGENT": ("🔴", "Compliance-induced instability: the tyre-force "
                           "feedback loop gain reached |g| ≥ 1 — deflection "
                           "recruits force faster than force causes deflection. "
                           "No quasi-static equilibrium exists; stiffen the "
                           "governing member before trusting any number here."),
    "COMPLIANCE_INVERTED": ("🔴", "Structural deflection under load is actively "
                            "REVERSING the kinematic design — the geometry you "
                            "drew and the geometry the tyre sees have opposite "
                            "signs mid-event."),
    "MARGIN_BREACHED": ("🟠", "A member's transient FoS dips under the 1.5-on-"
                        "yield rule during the event, even if the static case "
                        "passes. The worst instant below is the load case to "
                        "hand the FEA seat."),
    "COMPLIANCE_DEGRADED": ("🟡", "No inversion and margins hold, but the "
                            "deflections measurably move the geometry — the "
                            "rigid spreadsheet is optimistic for this event."),
    "RIGID_FAITHFUL": ("🟢", "Deflections stayed under every threshold: the "
                       "rigid model is an honest stand-in for this event at "
                       "this corner."),
}


def _hardpoints_from_session(ss):
    """The live hardpoint set if the kinematics tab has one, else the default."""
    import numpy as np
    from suspension.kinematics import Hardpoints
    raw = ss.get("hardpoints")
    if isinstance(raw, Hardpoints):
        return raw, "live hardpoints from the Kinematics tab"
    if isinstance(raw, dict):
        try:
            kw = {k: (np.asarray(v, float) if isinstance(v, (list, tuple)) else v)
                  for k, v in raw.items()}
            return Hardpoints(**kw), "live hardpoints from the Kinematics tab"
        except Exception:
            pass
    return Hardpoints.default(), "default FSAE front corner (no live hardpoints set)"


def render():
    import numpy as np
    import streamlit as st
    from suspension import ghost_topology as gt
    from suspension import transient as tr
    from suspension.compliance import CompliantCorner

    ss = st.session_state

    st.subheader("👻🔩 Ghost Topology — the geometry the car has mid-event")
    st.caption(
        "The siloed workflow runs kinematics rigid, exports one static load "
        "case to FEA, and never closes the loop. This tab walks a transient "
        "overload, solves the DEFORMED suspension state at each audited "
        "instant, and reports what the deflection does to the kinematic "
        "intent, the member load paths, the transient structural margins, "
        "and the tyre-force feedback — with the loop gain measured, because "
        "1.0 is where compliance-induced instability lives. Quasi-static "
        "structural response, one corner at a time, no plasticity — the "
        "scope notes are in the module docstring and the report footer.")

    hp, hp_note = _hardpoints_from_session(ss)
    st.caption(f"Geometry: {hp_note}.")

    # ---------------- 1 · the event ----------------------------------------
    c1, c2, c3 = st.columns(3)
    with c1:
        man_label = st.selectbox("Transient event", list(_MANEUVERS.keys()),
                                 key="ghost_maneuver")
    with c2:
        corner = st.selectbox("Corner to audit", ["FR", "FL", "RR", "RL"],
                              key="ghost_corner",
                              help="A left turn (+steer) loads the RIGHT side. "
                                   "The engine applies the body→corner sign "
                                   "mapping for whichever corner you pick.")
    with c3:
        n_samples = st.slider("Audited instants", 8, 48, 16, key="ghost_n",
                              help="The load extremes are always included; "
                                   "this adds the uniform comb between them. "
                                   "Near-identical load states share one "
                                   "cached solve.")

    # ---------------- 2 · the structure -------------------------------------
    with st.expander("Link tubes, tabs & margins", expanded=False):
        m1, m2, m3, m4 = st.columns(4)
        od = m1.number_input("Link OD (mm)", 8.0, 30.0, 19.05, 0.05,
                             key="ghost_od")
        wall = m2.number_input("Wall (mm)", 0.5, 3.0, 0.9, 0.05,
                               key="ghost_wall")
        material = m3.selectbox("Material", ["Steel 4130", "Steel mild",
                                             "Aluminium 6061", "Aluminium 7075",
                                             "Titanium Ti-6Al-4V"],
                                key="ghost_mat")
        yield_default = {"Steel 4130": 460.0, "Steel mild": 250.0,
                         "Aluminium 6061": 276.0, "Aluminium 7075": 503.0,
                         "Titanium Ti-6Al-4V": 880.0}[material]
        yld = m4.number_input("Yield (MPa)", 100.0, 1200.0, yield_default, 5.0,
                              key="ghost_yield",
                              help="The FoS rule's denominator — same "
                                   "1.5-on-yield gate as the bracket screen.")
        k_tab_on = st.checkbox("Chassis tabs flex too (series stiffness)",
                               value=True, key="ghost_tab_on")
        k_tab = st.number_input("Tab stiffness (N/mm)", 200.0, 50000.0, 8000.0,
                                100.0, key="ghost_tab", disabled=not k_tab_on)

    with st.expander("Tyre feedback sensitivities", expanded=False):
        st.caption("First-order ∂Fy/∂camber and ∂Fy/∂toe at the operating "
                   "point, scaled by Fz per instant. Representative FSAE "
                   "magnitudes by default — override from your tyre fit for "
                   "a final number. The measured loop gain below is what "
                   "these feed.")
        s1, s2 = st.columns(2)
        cam_per_kN = s1.number_input("Camber term (N/deg per kN Fz)", 0.0,
                                     200.0, 45.0, 5.0, key="ghost_scam")
        toe_per_kN = s2.number_input("Toe term (N/deg per kN Fz)", 0.0,
                                     1500.0, 300.0, 10.0, key="ghost_stoe")

    # ---------------- 3 · run ------------------------------------------------
    if not st.button("Solve the ghost topology", type="primary",
                     key="ghost_run"):
        st.info("Pick an event and press solve. A few dozen compliance "
                "solves — seconds, not a sim queue.")
        return

    with st.spinner("Integrating the transient, then solving the deformed "
                    "geometry at each audited instant…"):
        res = tr.run_maneuver(None, kind=_MANEUVERS[man_label])
        if not res.ok:
            st.error("Transient run flagged itself failed: "
                     + "; ".join(res.warnings[:3]))
            return

        cc = CompliantCorner.uniform_tube(
            hp, material=material, od_mm=od, wall_mm=wall,
            k_tab=(k_tab if k_tab_on else None))
        params = tr.TransientParams.from_vehicle(None)
        k_wheel = (params.k_wheel_front if corner in ("FL", "FR")
                   else params.k_wheel_rear) / 1000.0        # N/m → N/mm
        Fz_static = float(tr.TransientSolver().static_corner_loads()[
            {"FL": 0, "FR": 1, "RL": 2, "RR": 3}[corner]])
        track = (params.track_front if corner in ("FL", "FR")
                 else params.track_rear) * 1000.0
        gc = gt.GhostCorner(
            cc, gt.uniform_sections(od_mm=od, wall_mm=wall, material=material,
                                    yield_MPa=yld),
            wheel_rate_N_per_mm=k_wheel, Fz_static_N=Fz_static,
            track_mm=track)

        def tire_for(load):
            return gt.TireSensitivity.representative(
                load, camber_N_per_deg_per_kN=cam_per_kN,
                toe_N_per_deg_per_kN=toe_per_kN)

        # per-instant sensitivities: pass None and let the engine rebuild,
        # unless the user changed the magnitudes — then wrap the audit call
        # per-sample by pre-building on the PEAK load (honest single choice,
        # stated), because the audit API takes one sensitivity set.
        ci = {"FL": 0, "FR": 1, "RL": 2, "RR": 3}[corner]
        ysign = -1.0 if corner in ("FR", "RR") else 1.0
        pk = int(np.argmax(np.abs(res.Fy[:, ci])))
        from suspension import loadpath as lp
        peak_load = lp.WheelLoad(Fx=-res.Fx[pk, ci],
                                 Fy=ysign * res.Fy[pk, ci],
                                 Fz=res.Fz[pk, ci])
        tire = tire_for(peak_load)

        audit = gt.ghost_audit_transient(gc, res, corner=corner,
                                         n_samples=int(n_samples), tire=tire)

    # ---------------- 4 · the verdict ---------------------------------------
    badge, blurb = _VERDICT_BLURB.get(audit.verdict, ("•", ""))
    st.markdown(f"## {badge} `{audit.verdict}` — {man_label}, corner {corner}")
    st.caption(blurb)
    if len(audit.flags) > 1:
        st.caption("Also flagged: " + ", ".join(f"`{f}`" for f in audit.flags
                                                if f != audit.verdict))

    s = audit.summary()
    a, b, c, d = st.columns(4)
    a.metric("Worst transient FoS",
             f"{s['worst_fos']:.2f}" if np.isfinite(s['worst_fos']) else "∞",
             help=f"{s['worst_fos_member']} at t = "
                  f"{s['worst_fos_t_s']*1000:.0f} ms — yield AND Euler "
                  "buckling checked; governing shown.")
    b.metric("Peak Δcamber", f"{s['max_d_camber_deg']:.2f}°",
             help="Ghost minus rigid intent — the camber the tyre sees that "
                  "the spreadsheet doesn't.")
    c.metric("Peak Δtoe", f"{s['max_d_toe_deg']:.2f}°",
             help="Compliance steer through the event.")
    d.metric("Peak loop gain", f"{s['max_loop_gain']:.2f}",
             delta=("unstable ≥ 1.0" if s['max_loop_gain'] >= 1.0
                    else f"{(1.0 - s['max_loop_gain'])*100:.0f}% margin to "
                         "instability"),
             delta_color="inverse" if s['max_loop_gain'] >= 1.0 else "off")

    for f in audit.findings:
        icon = {"fail": "🔴", "warning": "🟡", "ok": "🟢"}.get(f["severity"], "•")
        st.markdown(f"{icon} **{f['check']}** — {f['message']}")

    # ---------------- 5 · the traces ----------------------------------------
    t_ms = audit.trace("t") * 1000.0
    tab_geo, tab_fos, tab_paths, tab_table = st.tabs(
        ["Geometry vs intent", "Transient margins", "Load-path shift",
         "Instant table"])

    with tab_geo:
        st.caption("Design intent (rigid, at that instant's travel) vs the "
                   "ghost topology. The gap is what compliance costs you; a "
                   "sign crossing is the inversion verdict.")
        st.line_chart(
            {"camber — rigid intent (°)": audit.trace("camber_rigid"),
             "camber — ghost (°)": audit.trace("camber_ghost"),
             "Δtoe (°)": audit.trace("d_toe")},
            x_label="audited instant (time-ordered)", height=280)
        st.line_chart({"ΔRC height (mm)": audit.trace("d_rc"),
                       "Δcontact patch lateral (mm)": audit.trace("d_cp")},
                      x_label="audited instant (time-ordered)", height=220)

    with tab_fos:
        st.caption("Minimum member FoS through the event, against the 1.5 "
                   "rule. A static check at one hand-picked case cannot see "
                   "the dip.")
        st.line_chart({"min FoS": audit.trace("min_fos"),
                       "1.5 rule": np.full(len(audit.instants), 1.5),
                       "yield onset": np.ones(len(audit.instants))},
                      x_label="audited instant (time-ordered)", height=280)

    with tab_paths:
        worst = min(audit.instants, key=lambda g: g.min_fos)
        st.caption(f"Member forces at the worst instant "
                   f"(t = {worst.t*1000:.0f} ms): the same wheel load reacted "
                   "through the rigid geometry vs the ghost geometry. The "
                   "shift is the load-path migration the siloed workflow "
                   "exports away.")
        rows = []
        for m, v in worst.load_path_shift.items():
            mg = worst.margins.get(m, {})
            rows.append({"member": m,
                         "rigid (N)": round(v["rigid_N"], 0),
                         "ghost (N)": round(v["ghost_N"], 0),
                         "shift (N)": round(v["delta_N"], 0),
                         "share shift": f"{v['share_shift']*100:+.1f}%",
                         "FoS": (f"{mg.get('fos', float('nan')):.2f}"
                                 if mg else "—"),
                         "mode": mg.get("mode", "—")})
        st.dataframe(rows, use_container_width=True, hide_index=True)

    with tab_table:
        st.dataframe([g.summary() | {"warnings": "; ".join(g.warnings)}
                      for g in audit.instants],
                     use_container_width=True, hide_index=True)

    st.caption(f"{len(audit.instants)} instants · {audit.n_solves} compliance "
               f"solves · {audit.n_cache_hits} cache hits — the time-scale "
               "separation is the whole trick, and its limits are flagged "
               "per instant when the event is too fast for it.")

    # ---------------- 6 · the report ----------------------------------------
    md = gt.render_ghost_md(audit)
    st.download_button("Download the audit (markdown)", md,
                       file_name=f"ghost_topology_{corner}.md",
                       mime="text/markdown", key="ghost_dl")
    ss["ghost_last_audit"] = s
