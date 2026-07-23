# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  ui/simulforge.py — the ⚡🔩 SimulForge tab (unified mechatronic co-solver)
# ============================================================================
"""
The tab where the mechanical car and its electrical nervous system are solved
as ONE coupled DAE: an active-ARB servo drawing real current off a real bus,
its authority sagging with the volts, the lost authority landing back on the
tyres as load — every millisecond, both directions. Pick a manoeuvre and a
declared electrical defect, and the linter judges the pair across the three
ledgers the defect touches: the electrical response, the structure (via the
Ghost Topology re-audit), the first-failure pecking order (Fusebox, with the
branch fuse and connector racing the wishbones), and the test-day energy
budget (Earshot). Verdicts: STRUCTURAL_REGRESSION / ORDER_INVERTED /
BROWNOUT / AUTHORITY_LOST / SESSION_STARVED / RESPONSE_LAGGED /
COUPLED_FAITHFUL.

All physics lives in suspension/simulforge.py (joining transient,
ghost_topology, fusebox and earshot). This module only orchestrates and
draws (see ui/__init__.py rules).

Session keys used:
    hardpoints          the live hardpoint dict, if the kinematics tab set one
    forge_last_lint     summary dict of the last lint (for the handover report)
"""

from __future__ import annotations


_MANEUVERS = {
    "Step steer (J-turn)": "step_steer",
    "Snap oversteer + countersteer": "snap_oversteer",
    "Brake → throttle transition": "brake_to_throttle",
    "Curb strike": "curb_strike",
}

_VERDICT_BLURB = {
    "STRUCTURAL_REGRESSION": ("🔴", "The electrical defect re-priced the "
                              "STRUCTURE: the ghost re-audit under the "
                              "degraded load history drops the worst "
                              "transient FoS past the lint gate (or flips "
                              "the ghost verdict). A wiring defect is now a "
                              "structural load case — hand the worst instant "
                              "to the FEA seat with the electrical story "
                              "attached."),
    "ORDER_INVERTED": ("🔴", "The first-failure pecking order MOVED: the "
                       "most-likely-first element under the defect is not "
                       "the one under pristine electrics. The car's chosen "
                       "victim changed without anyone machining a part — "
                       "re-seal the Fuse Charter or fix the defect."),
    "BROWNOUT": ("🔴", "The controller browned out mid-manoeuvre: the bus "
                 "sagged below the reset threshold and the car went passive "
                 "at the worst possible moment, for the full reboot time. "
                 "Fix the series resistance or the brownout margin before "
                 "trusting any active-roll number."),
    "AUTHORITY_LOST": ("🟠", "The actuator delivered materially less moment "
                       "than commanded: the sagging drive voltage cannot "
                       "push the commanded current against back-EMF. The "
                       "simulated car with pristine electrics has roll "
                       "authority the degraded car does not."),
    "SESSION_STARVED": ("🟠", "The defect's energy bill, run through "
                        "Earshot's own pack-budget arithmetic, shrinks the "
                        "session enough to change the A/B verdict — the "
                        "planned test day can no longer hear its effect."),
    "RESPONSE_LAGGED": ("🟡", "Delivered moment now trails the command "
                        "beyond the lint gate — the bar arrives after the "
                        "load transient it was meant to blunt. Watch the "
                        "load-cycle amplification even if margins hold."),
    "COUPLED_FAITHFUL": ("🟢", "The co-solved car under the declared defect "
                         "matches the pristine car within every lint gate: "
                         "response, structure, pecking order and session "
                         "budget all hold."),
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
    return Hardpoints.default(), \
        "default FSAE front corner (no live hardpoints set)"


def render():
    import numpy as np
    import streamlit as st
    from suspension import units as _units
    from suspension import simulforge as sf
    from suspension import ghost_topology as gt
    from suspension import transient as tr
    from suspension import earshot as es

    ss = st.session_state

    st.subheader("⚡🔩 SimulForge — the unified mechatronic co-solver")
    st.caption(
        "Every siloed tool gives the actuators pristine voltage and the "
        "electrical model static loads; the wire between them exists in "
        "neither. This tab co-solves the transient vehicle DAE with the "
        "actuator winding states and the live bus constraint — staggered "
        "co-step at the mechanical millisecond, exact RL winding update, "
        "moments held over each step — then runs the SAME manoeuvre with a "
        "declared electrical defect and lints the pair across the response, "
        "the structure (Ghost re-audit), the first-failure pecking order "
        "(Fusebox — the branch fuse and connector racing the wishbones in "
        "one probability), and the test-day energy budget (Earshot). "
        "Defaults are FSAE-representative stand-ins; set every knob from "
        "your datasheets.")

    hp, hp_note = _hardpoints_from_session(ss)
    st.caption(f"Geometry: {hp_note}.")

    # ---------------- 1 · the event and the defect --------------------------
    presets = sf.degradation_presets()
    c1, c2 = st.columns(2)
    with c1:
        man_label = st.selectbox("Transient event", list(_MANEUVERS.keys()),
                                 key="forge_maneuver")
    with c2:
        deg_keys = [k for k in presets if k != "nominal"]
        deg_key = st.selectbox(
            "Declared electrical defect", deg_keys,
            format_func=lambda k: presets[k].label, key="forge_defect",
            help="The pristine car is always run as the reference; this is "
                 "the defect the linter judges against it.")
    st.caption(f"_{presets[deg_key].story}_")

    # ---------------- 2 · actuator & bus knobs ------------------------------
    with st.expander("Actuator & bus (datasheet knobs)", expanded=False):
        a1, a2, a3, a4 = st.columns(4)
        kp = _units.unum(a1, "Roll gain kp (N·m/rad)", 0.0, 60_000.0, 12_000.0, 'N·m/rad', step=500.0, key="forge_kp")
        kd = _units.unum(a2, "Roll gain kd (N·m·s/rad)", 0.0, 5_000.0, 900.0, 'N·m·s/rad', step=50.0, key="forge_kd")
        m_max = _units.unum(a3, "Actuator ceiling (N·m)", 0.0, 3_000.0, 900.0, 'N·m', step=50.0, key="forge_mmax")
        fuse_a = a4.number_input("Branch fuse (A)", 5.0, 60.0, 25.0, 1.0,
                                 key="forge_fuse")
        b1, b2, b3, b4 = st.columns(4)
        v_oc = b1.number_input("Bus V_oc (V)", 12.0, 60.0, 25.2, 0.1,
                               key="forge_voc")
        r_int = b2.number_input("Pack R_int (mΩ)", 1.0, 500.0, 45.0, 1.0,
                                key="forge_rint")
        v_bo = b3.number_input("Brownout (V)", 6.0, 30.0, 17.5, 0.1,
                               key="forge_vbo")
        kwh_lap = b4.number_input("kWh per lap", 0.05, 1.5, 0.28, 0.01,
                                  key="forge_kwhlap",
                                  help="Earshot's session currency — the "
                                       "same channel the endurance energy "
                                       "budget runs on.")

    # ---------------- 3 · session (Earshot) ---------------------------------
    with st.expander("Test-day A/B design (Earshot coupling)", expanded=False):
        s1, s2, s3 = st.columns(3)
        eff = s1.number_input("Predicted effect (s/lap)", 0.01, 5.0, 0.3,
                              0.01, key="forge_eff")
        sig = s2.number_input("Lap sigma (s)", 0.05, 5.0, 0.8, 0.01,
                              key="forge_sig")
        pack = s3.number_input("Pack (kWh)", 1.0, 20.0, 6.5, 0.1,
                               key="forge_pack")

    run = st.button("⚡ Co-solve pristine vs defect and lint", type="primary",
                    key="forge_run")
    if not run and "forge_lint_md" not in ss:
        st.info("Pick an event and a defect, then co-solve. The pristine car "
                "and the degraded car run the same manoeuvre; the linter "
                "judges everything the defect touched.")
        return

    if run:
        act = sf.ActuatorParams(kp=float(kp), kd=float(kd),
                                M_max_Nm=float(m_max),
                                fuse_rating_A=float(fuse_a))
        bus = sf.BusParams(V_oc=float(v_oc), R_int_ohm=float(r_int) / 1000.0,
                           V_brownout=float(v_bo), kwh_per_lap=float(kwh_lap),
                           pack_kwh=float(pack))
        kind = _MANEUVERS[man_label]
        with st.spinner("Co-solving the pristine car…"):
            nom = sf.run_simulforge(None, kind=kind, degradation="nominal",
                                    bus=bus, actuator=act)
        with st.spinner(f"Co-solving under '{presets[deg_key].label}'…"):
            deg = sf.run_simulforge(None, kind=kind,
                                    degradation=presets[deg_key],
                                    bus=bus, actuator=act)
        # structural judge: ghost corner from the live geometry
        gc = None
        try:
            params = tr.TransientParams.from_vehicle(None)
            Fz_static = float(tr.TransientSolver().static_corner_loads()[1])
            gc = gt.GhostCorner.uniform_tube(
                hp, wheel_rate_N_per_mm=params.k_wheel_front / 1000.0,
                Fz_static_N=Fz_static, track_mm=params.track_front * 1000.0)
        except Exception as e:                                  # noqa: BLE001
            st.warning(f"Ghost structural coupling unavailable ({e}); "
                       "the lint runs without the structural re-audit.")
        ab = es.ABDesign(effect_predicted=float(eff), noise_sigma=float(sig))
        with st.spinner("Linting across the ledgers…"):
            lint = sf.forge_lint(nom, deg, gc=gc, ab_design=ab)
        ss["forge_lint_obj"] = lint
        ss["forge_lint_md"] = sf.render_forge_md(lint)
        ss["forge_last_lint"] = lint.summary()

    lint = ss.get("forge_lint_obj")
    if lint is None:
        return

    # ---------------- 4 · verdict --------------------------------------------
    icon, blurb = _VERDICT_BLURB.get(
        lint.verdict, ("•", "See the findings below."))
    st.markdown(f"### {icon} Verdict: `{lint.verdict}`")
    st.write(blurb)
    if lint.note:
        st.warning(lint.note)

    en, ed = lint.nominal.elec.summary(), lint.degraded.elec.summary()
    k1, k2, k3, k4 = st.columns(4)
    k1.metric("Bus minimum", f"{ed['v_min']:.1f} V",
              f"{ed['v_min'] - en['v_min']:+.1f} V vs pristine")
    k2.metric("Roll authority", f"{ed['authority']:.0%}",
              f"{(ed['authority'] - en['authority']):+.0%}")
    k3.metric("Brownouts", f"{ed['n_brownouts']}",
              f"{ed['offline_ms']:.0f} ms offline")
    k4.metric("Event energy", f"{ed['energy_Wh']:.2f} Wh",
              f"{ed['energy_Wh'] - en['energy_Wh']:+.2f} Wh")

    # ---------------- 5 · the coupled traces ----------------------------------
    t_ms = lint.degraded.elec.t * 1000.0
    st.markdown("**The electrical shadow of the manoeuvre** — bus voltage "
                "and delivered vs commanded moment, degraded run:")
    cA, cB = st.columns(2)
    with cA:
        st.line_chart({"V_bus degraded (V)": lint.degraded.elec.V_bus,
                       "V_bus pristine (V)": np.interp(
                           lint.degraded.elec.t, lint.nominal.elec.t,
                           lint.nominal.elec.V_bus)})
    with cB:
        st.line_chart({
            "|M commanded| (N·m)":
                np.abs(lint.degraded.elec.M_cmd).sum(axis=1),
            "|M delivered| (N·m)":
                np.abs(lint.degraded.elec.M_act).sum(axis=1)})
    st.markdown("**What the structure felt** — body roll and the worst "
                "corner's contact load, pristine vs degraded:")
    cC, cD = st.columns(2)
    cyc = lint.degraded.load_cycle_ptp_N()
    wi = int(np.argmax(cyc)) if cyc.size else 1
    with cC:
        st.line_chart({"roll pristine (deg)":
                       np.degrees(lint.nominal.mech.roll),
                       "roll degraded (deg)":
                       np.degrees(lint.degraded.mech.roll)})
    with cD:
        st.line_chart({"Fz pristine (N)": lint.nominal.mech.Fz[:, wi],
                       "Fz degraded (N)": lint.degraded.mech.Fz[:, wi]})
    del t_ms

    # ---------------- 6 · findings --------------------------------------------
    st.markdown("### Findings")
    sev_icon = {"red": "🔴", "amber": "🟠", "info": "ℹ️"}
    for f in lint.findings:
        st.markdown(f"{sev_icon.get(f.severity, '•')} **{f.code}** — {f.text}")

    if lint.path_audit_degraded is not None:
        pa = lint.path_audit_degraded
        with st.expander("The cross-disciplinary pecking order (Fusebox)",
                         expanded=False):
            st.write(f"Path verdict **{pa.verdict.value}** — most likely "
                     f"first failure: **{pa.leader_key}** at "
                     f"{pa.leader_p:.0%}. The branch fuse and connector race "
                     f"the wishbone members in one probability; the elements "
                     f"below are ranked by P(first).")
            rows = sorted(pa.probs.items(), key=lambda kv: -kv[1])
            st.table({"element": [k for k, _ in rows],
                      "P(first)": [f"{p:.1%}" for _, p in rows]})

    if lint.session_after is not None:
        with st.expander("The session bill (Earshot)", expanded=False):
            st.write(f"Pack budget: "
                     f"{lint.session_before['laps_per_config']} → "
                     f"{lint.session_after['laps_per_config']} laps per "
                     f"config after the defect's energy bill "
                     f"({lint.session_after.get('event_Wh_degraded', 0) - lint.session_after.get('event_Wh_nominal', 0):+.2f} "
                     f"Wh per event). A/B verdict: "
                     f"{lint.session_after.get('verdict', 'n/a')}.")

    st.download_button("⬇️ Download the lint report (Markdown)",
                       ss.get("forge_lint_md", ""),
                       file_name="simulforge_lint.md",
                       mime="text/markdown", key="forge_dl")
