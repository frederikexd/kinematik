# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#
#  ui/phantom_envelope.py — the 📦👻 Phantom Envelope tab (swept-load packaging)
# ============================================================================
"""
The tab that carves the exact 3D volume a moving corner CLAIMS across its whole
range of motion — and warps it by Ghost Topology's real-time compliance
deflection, so the powertrain / chassis team can ask 'does my motor mount clear
the upper control arm at 1.8 g?' and get an instant, attributed answer instead
of waiting for a manual CAD interference check.

What it does, in order:
  1. sweeps the RIGID linkage through travel to draw the no-load motion envelope
     (what a CAD interference check sees);
  2. runs a transient event through the Ghost Topology engine and reads the
     DEFORMED geometry at every audited instant, carving the COMPLIANT envelope
     — the volume the loaded links actually need;
  3. measures how far the compliant envelope grows OUTSIDE the rigid one (the
     ground the rigid CAD check misses);
  4. lets the other team drop a candidate point + probe radius and reports
     clearance / violation, attributed to the governing link, instant, and load;
  5. ships the forbidden volume as a lightweight .json / .csv point cloud in the
     same corner frame the geometry was defined in.

All physics lives in suspension/phantom_envelope.py (which joins kinematics,
compliance, and ghost_topology). This module only orchestrates and draws
(see ui/__init__.py rules).

Session keys used:
    hardpoints             the live hardpoint dict, if the kinematics tab set one
    phantom_env_last       summary of the last carve (for handover)
    phantom_env_compliant  the last compliant PhantomEnvelope (for the query box)
"""

from __future__ import annotations


_MANEUVERS = {
    "Step steer (J-turn)": "step_steer",
    "Snap oversteer + countersteer": "snap_oversteer",
    "Brake → throttle transition": "brake_to_throttle",
    "Curb strike": "curb_strike",
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


def _draw_envelope_3d(rigid, compliant, probe=None, probe_r=0.0):
    """A matplotlib 3D scatter of the two boundary clouds + the probe sphere."""
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(7.5, 6.0))
    ax = fig.add_subplot(111, projection="3d")

    if rigid is not None and rigid.n_points:
        b = rigid.boundary
        ax.scatter(b[:, 0], b[:, 1], b[:, 2], s=2, c="#9fb3d1", alpha=0.35,
                   label=f"rigid sweep ({rigid.n_points} pts)")
    if compliant is not None and compliant.n_points:
        b = compliant.boundary
        ax.scatter(b[:, 0], b[:, 1], b[:, 2], s=3, c="#e08a3c", alpha=0.55,
                   label=f"compliant (loaded) ({compliant.n_points} pts)")

    if probe is not None:
        p = np.asarray(probe, float)
        # a translucent probe sphere
        u = np.linspace(0, 2 * np.pi, 16)
        v = np.linspace(0, np.pi, 12)
        r = max(probe_r, 3.0)
        xs = p[0] + r * np.outer(np.cos(u), np.sin(v))
        ys = p[1] + r * np.outer(np.sin(u), np.sin(v))
        zs = p[2] + r * np.outer(np.ones_like(u), np.cos(v))
        ax.plot_surface(xs, ys, zs, color="#d1495b", alpha=0.5, linewidth=0)
        ax.scatter([p[0]], [p[1]], [p[2]], s=30, c="#d1495b", label="query point")

    ax.set_xlabel("x — rear + (mm)")
    ax.set_ylabel("y — right + (mm)")
    ax.set_zlabel("z — up + (mm)")
    ax.legend(loc="upper left", fontsize=8, framealpha=0.6)
    ax.view_init(elev=18, azim=-62)
    try:
        ax.set_box_aspect((1, 1, 0.8))
    except Exception:
        pass
    fig.tight_layout()
    return fig


def render():
    import numpy as np
    import streamlit as st
    from suspension import units as _units
    from suspension import ghost_topology as gt
    from suspension import phantom_envelope as pe
    from suspension import transient as tr
    from suspension.compliance import CompliantCorner
    from suspension import loadpath as lp

    ss = st.session_state

    st.subheader("📦👻 Phantom Envelope — the volume the corner claims under load")
    st.caption(
        "Big-software packaging checks either sweep the RIGID suspension in a "
        "CAD seat by hand, or brute-force a 3D mesh collision that melts a "
        "workstation. Neither closes the loop that bites: under load the links "
        "DEFLECT, and the volume the loaded corner needs is a warped copy of "
        "the CAD sweep. This tab carves both — the rigid motion envelope and "
        "the compliance-warped one Ghost Topology already solved — as swept "
        "tube capsules, measures the growth, and answers 'does my mount clear "
        "the arm?' in the same corner frame the geometry lives in. A screening "
        "boundary for the packaging seat, not a certified sign-off — the scope "
        "notes are in the module docstring and the report footer.")

    hp, hp_note = _hardpoints_from_session(ss)
    st.caption(f"Geometry: {hp_note}.")

    # ---------------- 1 · the sweep + event --------------------------------
    c1, c2, c3 = st.columns(3)
    with c1:
        man_label = st.selectbox("Transient event (for the loaded sweep)",
                                 list(_MANEUVERS.keys()), key="penv_maneuver")
    with c2:
        corner = st.selectbox("Corner", ["FR", "FL", "RR", "RL"],
                              key="penv_corner",
                              help="A left turn (+steer) loads the RIGHT side. "
                                   "The engine applies the body→corner sign "
                                   "mapping for whichever corner you pick.")
    with c3:
        n_samples = st.slider("Audited instants (loaded sweep)", 8, 48, 20,
                              key="penv_n",
                              help="How many instants of the event feed the "
                                   "compliant carve. Load extremes always "
                                   "included.")

    t1, t2, t3 = st.columns(3)
    with t1:
        tr_lo = _units.unum(st, "Bump travel (mm)", 0.0, 60.0, 25.0, 'mm', step=1.0, key="penv_bump")
    with t2:
        tr_hi = _units.unum(st, "Droop travel (mm)", 0.0, 60.0, 25.0, 'mm', step=1.0, key="penv_droop")
    with t3:
        n_travel = st.slider("Rigid sweep poses", 3, 21, 9, key="penv_ntravel",
                             help="How finely the no-load motion range is "
                                  "sampled. More poses = smoother rigid shell.")

    # ---------------- 2 · the structure -------------------------------------
    with st.expander("Link tubes, tab stiffness & capsule inflation", expanded=False):
        m1, m2, m3, m4 = st.columns(4)
        od = _units.unum(m1, "Link OD (mm)", 8.0, 30.0, 19.05, 'mm', step=0.05, key="penv_od")
        wall = _units.unum(m2, "Wall (mm)", 0.5, 3.0, 0.9, 'mm', step=0.05, key="penv_wall")
        material = m3.selectbox("Material", ["Steel 4130", "Steel mild",
                                             "Aluminium 6061", "Aluminium 7075",
                                             "Titanium Ti-6Al-4V"],
                                key="penv_mat")
        yield_default = {"Steel 4130": 460.0, "Steel mild": 250.0,
                         "Aluminium 6061": 276.0, "Aluminium 7075": 503.0,
                         "Titanium Ti-6Al-4V": 880.0}[material]
        yld = _units.unum(m4, "Yield (MPa)", 100.0, 1200.0, yield_default, 'MPa', step=5.0, key="penv_yield")
        i1, i2 = st.columns(2)
        inflate = _units.unum(i1, "Capsule inflation (mm)", 0.0, 40.0, 3.0, 'mm', step=0.5, key="penv_inflate", help="Added to each tube's radius to cover tabs, gusset webs, rod "
                 "ends and clevises the bare tube doesn't model. A defensible "
                 "packaging margin — bump it if your A-arms carry a lot of "
                 "outboard hardware.")
        k_tab_on = i2.checkbox("Chassis tabs flex too", value=True,
                               key="penv_tabon")
        k_tab = _units.unum(st, "Tab stiffness (N/mm)", 200.0, 50000.0, 8000.0, 'N/mm', step=100.0, key="penv_tab", disabled=not k_tab_on)

    all_members = ("UF", "UR", "LF", "LR", "TR", "PR")
    with st.expander("Members to carve", expanded=False):
        st.caption("Which links contribute to the forbidden volume. Drop the "
                   "pushrod (PR) if it lives in a different bay from the part "
                   "you're checking.")
        picked = []
        cols = st.columns(6)
        labels = {"UF": "UCA front", "UR": "UCA rear", "LF": "LCA front",
                  "LR": "LCA rear", "TR": "tie rod", "PR": "pushrod"}
        for i, m in enumerate(all_members):
            if cols[i].checkbox(labels[m], value=True, key=f"penv_m_{m}"):
                picked.append(m)
        members = tuple(picked) if picked else all_members

    # ---------------- 3 · run ------------------------------------------------
    if not st.button("Carve the phantom envelope", type="primary",
                     key="penv_run"):
        st.info("Set the sweep and press carve. A rigid motion sweep plus a "
                "few dozen compliance solves — seconds, not a mesh-collision "
                "queue.")
        return

    with st.spinner("Sweeping the rigid linkage, running the transient, then "
                    "carving the loaded envelope from the deformed geometry…"):
        # build the GhostCorner exactly like the Ghost Topology tab does
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
                   else params.k_wheel_rear) / 1000.0
        Fz_static = float(tr.TransientSolver().static_corner_loads()[
            {"FL": 0, "FR": 1, "RL": 2, "RR": 3}[corner]])
        track = (params.track_front if corner in ("FL", "FR")
                 else params.track_rear) * 1000.0
        gc = gt.GhostCorner(
            cc, gt.uniform_sections(od_mm=od, wall_mm=wall, material=material,
                                    yield_MPa=yld),
            wheel_rate_N_per_mm=k_wheel, Fz_static_N=Fz_static, track_mm=track)

        audit = gt.ghost_audit_transient(gc, res, corner=corner,
                                         n_samples=int(n_samples))

        rigid_env = pe.carve_rigid_envelope(
            gc, corner_label=corner, travel_mm=(-tr_lo, tr_hi),
            n_travel=int(n_travel), members=members, inflate_mm=inflate)
        comp_env = pe.carve_ghost_envelope(
            audit, gc, members=members, inflate_mm=inflate)
        delta = pe.envelope_delta(rigid_env, comp_env)

    # stash for the query box (survives reruns without recarving)
    ss["phantom_env_compliant"] = comp_env
    ss["phantom_env_rigid"] = rigid_env
    ss["phantom_env_delta"] = delta

    _show_results(st, np, pe, rigid_env, comp_env, delta, corner)


def _show_results(st, np, pe, rigid_env, comp_env, delta, corner):
    # ---------------- 4 · the headline --------------------------------------
    grew = delta.max_outward_growth_mm > 0.05
    st.markdown(f"## {'🟠' if grew else '🟢'} Phantom Envelope — corner {corner}")
    if grew:
        ld = delta.growth_load_N or {}
        ldtxt = (f" (Fy {ld.get('Fy', 0):.0f} N, Fz {ld.get('Fz', 0):.0f} N)"
                 if ld else "")
        st.caption(
            f"Under load the corner claims **{delta.max_outward_growth_mm:.2f} mm** "
            f"beyond the rigid CAD sweep, governed by **{delta.growth_member}** at "
            f"t = {delta.growth_t_s*1000:.0f} ms{ldtxt}. A packaging check run "
            "against the rigid sweep alone would miss this ground.")
    else:
        st.caption("The compliant envelope stays within the rigid sweep for "
                   "this event — the rigid CAD interference check is a faithful "
                   "stand-in here.")

    mn_r, mx_r = rigid_env.bbox
    mn_c, mx_c = comp_env.bbox
    a, b, c, d = st.columns(4)
    a.metric("Compliance growth", f"{delta.max_outward_growth_mm:.2f} mm",
             help="Deepest a loaded boundary point sits outside the rigid sweep "
                  f"— governed by {delta.growth_member}.")
    b.metric("Boundary grown", f"{delta.frac_points_grown*100:.0f}%",
             help="Share of the compliant boundary lying outside the rigid one.")
    c.metric("Rigid pts", f"{rigid_env.n_points}",
             help="Boundary cloud size of the no-load motion sweep.")
    d.metric("Loaded pts", f"{comp_env.n_points}",
             help="Boundary cloud size of the compliance-warped sweep.")

    # Provenance: the growth number is a modelled output of the compliance
    # sweep; its absolute value rides on the member stiffnesses fed in. Flag it
    # as an indicative packaging risk margin and name the upgrade path, so a
    # reviewer reads it as "where to look", not a certified clearance.
    try:
        from suspension.provenance import confidence_note as _conf
        _conf(st, "modelled", calibrated=False,
              extra=("quasi-static compliance sweep over representative member "
                     "stiffnesses — a proactive packaging-risk margin"),
              calibrate_with=("measured link/bush stiffnesses (a static pull "
                              "test per member) to turn this into a certified "
                              "clearance"))
    except Exception:
        pass

    if comp_env.excluded:
        st.warning("Excluded from the loaded carve (elastic geometry void past "
                   "yield): " + "; ".join(
                       f"t={t*1000:.0f} ms — {r}" for t, r in comp_env.excluded))

    # ---------------- 5 · the picture ---------------------------------------
    st.markdown("#### The two envelopes")
    st.caption("Blue = rigid motion sweep (what CAD interference sees). "
               "Orange = the volume the LOADED links claim. Where orange spills "
               "past blue is the ground compliance adds.")
    fig = _draw_envelope_3d(rigid_env, comp_env)
    st.pyplot(fig, clear_figure=True)

    # ---------------- 6 · the clearance query -------------------------------
    st.markdown("#### Does my part clear the arm?")
    st.caption("Drop a candidate point (a motor-mount corner, an inverter face, "
               "a harness clip) in the corner frame and a probe radius. The "
               "answer is exact against the swept capsules and attributed to "
               "the governing link, instant, and load.")
    q1, q2, q3, q4 = st.columns(4)
    px = _units.unum(q1, "x (mm)", -400.0, 400.0, float(round((mn_c[0] + mx_c[0]) / 2, 1)), 'mm', step=1.0, key="penv_qx")
    py = _units.unum(q2, "y (mm)", 0.0, 700.0, float(round(mx_c[1] - 20.0, 1)), 'mm', step=1.0, key="penv_qy")
    pz = _units.unum(q3, "z (mm)", 0.0, 500.0, float(round((mn_c[2] + mx_c[2]) / 2, 1)), 'mm', step=1.0, key="penv_qz")
    pr = _units.unum(q4, "probe r (mm)", 0.0, 60.0, 6.0, 'mm', step=0.5, key="penv_qr")

    probe = np.array([px, py, pz])
    q_rig = rigid_env.query(probe, probe_radius_mm=pr)
    q_cmp = comp_env.query(probe, probe_radius_mm=pr)

    r1, r2 = st.columns(2)
    with r1:
        st.markdown("**Against the rigid sweep**")
        _clearance_line(st, q_rig)
    with r2:
        st.markdown("**Against the loaded (compliant) envelope**")
        _clearance_line(st, q_cmp)

    # the money line: a part that clears rigid but fouls compliant
    if (not q_rig.violates) and q_cmp.violates:
        st.error(
            f"⚠️ This point CLEARS the rigid sweep by {q_rig.clearance_mm:.2f} mm "
            f"but VIOLATES the loaded envelope by {abs(q_cmp.clearance_mm):.2f} mm "
            f"— {q_cmp.nearest_member} compliance-deflects into it at "
            f"t = {q_cmp.nearest_t_s*1000:.0f} ms. This is exactly the conflict a "
            "rigid CAD check ships and the car finds on track.")

    fig2 = _draw_envelope_3d(rigid_env, comp_env, probe=probe, probe_r=pr)
    st.pyplot(fig2, clear_figure=True)

    # ---------------- 7 · the deliverables ----------------------------------
    st.markdown("#### Ship the forbidden volume")
    st.caption("The lightweight point cloud the chassis / powertrain team "
               "queries — same corner frame, mm, with per-capsule provenance.")
    md = pe.render_envelope_md(comp_env, delta=delta,
                               queries=[q_cmp] if pr >= 0 else None)
    d1, d2, d3 = st.columns(3)
    d1.download_button("Point cloud (.json)", comp_env.to_json(indent=2),
                       file_name=f"phantom_envelope_{corner}.json",
                       mime="application/json", key="penv_dl_json")
    d2.download_button("Point cloud (.csv)", comp_env.to_csv(),
                       file_name=f"phantom_envelope_{corner}.csv",
                       mime="text/csv", key="penv_dl_csv")
    d3.download_button("Report (.md)", md,
                       file_name=f"phantom_envelope_{corner}.md",
                       mime="text/markdown", key="penv_dl_md")

    st.session_state["phantom_env_last"] = {
        "corner": corner, "kind": comp_env.kind,
        "growth_mm": delta.max_outward_growth_mm,
        "growth_member": delta.growth_member,
        "n_points": comp_env.n_points, "frame": comp_env.frame,
    }


def _clearance_line(st, q):
    # Display clearance (mm) and probe loads (N) in the active unit system so
    # this readout tracks the metric/US toggle like the rest of the app. The
    # stored values stay SI; only the shown number + label convert. If the
    # units module can't be imported for any reason, fall back to metric.
    try:
        from suspension import units as _u
        _mm = lambda v: (_u.from_metric(v, "mm"), _u.label("mm"))
        _n  = lambda v: (_u.from_metric(v, "N"), _u.label("N"))
    except Exception:                       # pragma: no cover
        _mm = lambda v: (v, "mm")
        _n  = lambda v: (v, "N")

    if q.nearest_member == "":
        st.info("no members carved")
        return
    if q.violates:
        _cv, _cu = _mm(abs(q.clearance_mm))
        st.markdown(f"🔴 **VIOLATES by {_cv:.2f} {_cu}** — "
                    f"nearest link **{q.nearest_member}**"
                    + (f", t = {q.nearest_t_s*1000:.0f} ms"
                       if q.nearest_t_s == q.nearest_t_s else ""))
        if q.nearest_load_N:
            ld = q.nearest_load_N
            _fy, _fu = _n(ld['Fy'])
            _fz, _   = _n(ld['Fz'])
            st.caption(f"at Fy {_fy:.0f} {_fu}, Fz {_fz:.0f} {_fu}")
    else:
        _cv, _cu = _mm(q.clearance_mm)
        st.markdown(f"🟢 **clears by {_cv:.2f} {_cu}** — "
                    f"nearest link **{q.nearest_member}**")
