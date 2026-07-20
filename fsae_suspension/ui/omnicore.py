# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#
#  ui/omnicore.py — the 🪐 OmniCore tab (vehicle-synthesis referee)
# ============================================================================
"""
The tab where the subsystem war becomes a number. Type the mission profile in
one sentence; a deterministic grammar (NOT a language model — the receipt
proves it) turns it into a typed spec; the referee sweeps a configuration
lattice through all three synthesis engines with shared sub-solves —
InverseGenesis per shop class, SimulForge + Ghost audit per actuator size,
MorphMesh per (shop × actuator × volume budget) — and draws the screening
Pareto front over event composure, endurance range, structural mass,
decision-scope cost, and build yield. Every infeasible configuration names
the engine (or the budget) that vetoed it; every dominated one carries a
receipt naming its dominator.

Below the front: the self-healing twin. Paste measured telemetry summaries
from the running car; each channel is judged against the nominal model's
declared bands, the deviation pattern is cosine-matched against the
signature every named Degradation preset predicts, and the heal plan is
arithmetic — a gain de-rate from what the sagged bus can deliver, and a
3-D-printable shim thickness from the measured camber drift.

All physics/orchestration lives in suspension/omnicore.py (headless,
self-tested). This module only orchestrates and draws (ui/__init__.py rules).

Session keys used:
    hardpoints           the live hardpoint dict, if the kinematics tab set one
    omnicore_result      the last OmniResult (downloads never re-solve)
    omnicore_twin        (baseline, forge_nominal, signatures) for the twin
    omnicore_last        summary for handover
"""

from __future__ import annotations


_MISSION_PLACEHOLDER = (
    "Synthesize a lightweight, 4WD electric off-road vehicle optimized for "
    "high-frequency bumpy terrain, constrained by a $15,000 manufacturing "
    "budget and a ±2 mm weld-pull error shop floor.")


def _hardpoints_from_session(ss):
    """The live hardpoint set if the kinematics tab has one, else the default."""
    import numpy as np
    from suspension.kinematics import Hardpoints
    raw = ss.get("hardpoints")
    if isinstance(raw, Hardpoints):
        return raw, "live hardpoints from the Kinematics tab"
    if isinstance(raw, dict):
        try:
            kw = {k: (np.asarray(v, float) if isinstance(v, (list, tuple))
                      else v) for k, v in raw.items()}
            return Hardpoints(**kw), "live hardpoints from the Kinematics tab"
        except Exception:
            pass
    return Hardpoints.default(), \
        "default FSAE front corner (no live hardpoints set)"


def _draw_front(res):
    """The front as a scatter matrix's honest little brother: cost on x,
    composure on y, marker size = structural mass, colour = endurance range;
    the front ringed, the knee starred, infeasibles shown hollow with an ×."""
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from suspension import omnicore as oc

    fig, ax = plt.subplots(figsize=(7.4, 4.6))
    feas = [c for c in res.configs if c.feasible]
    infe = [c for c in res.configs if not c.feasible]
    if feas:
        x = np.array([c.objectives["cost"] for c in feas])
        y = np.array([c.objectives["composure"] for c in feas])
        m = np.array([c.objectives["mass"] for c in feas])
        lp = np.array([c.objectives["laps"] for c in feas])
        sizes = 60.0 + 220.0 * (m - m.min()) / max(float(np.ptp(m)), 1e-9)
        scat = ax.scatter(x, y, s=sizes, c=lp, cmap="viridis",
                          edgecolors="#333", linewidths=0.6, zorder=3)
        fig.colorbar(scat, ax=ax, label="endurance range (laps)")
        for c in feas:
            if c.cid in res.pareto_ids:
                ax.scatter([c.objectives["cost"]],
                           [c.objectives["composure"]], s=340,
                           facecolors="none", edgecolors="#e08a3c",
                           linewidths=1.8, zorder=4)
            if c.cid == res.knee_id:
                ax.scatter([c.objectives["cost"]],
                           [c.objectives["composure"]], marker="*",
                           s=420, color="#d1495b", zorder=5,
                           label="referee's pick (knee)")
            ax.annotate(f"#{c.cid}", (c.objectives["cost"],
                                      c.objectives["composure"]),
                        textcoords="offset points", xytext=(6, 5),
                        fontsize=7.5)
    for c in infe:
        cost = c.objectives.get("cost", float("nan"))
        comp = c.objectives.get("composure", float("nan"))
        if np.isfinite(cost) and np.isfinite(comp):
            ax.scatter([cost], [comp], marker="x", s=70, color="#999",
                       zorder=2)
            ax.annotate(f"#{c.cid}", (cost, comp),
                        textcoords="offset points", xytext=(6, 5),
                        fontsize=7.5, color="#999")
    ax.set_xlabel(f"decision-scope cost ({oc.AXES['cost'][1]})")
    ax.set_ylabel(f"event composure ({oc.AXES['composure'][1]}) — lower is "
                  "more composed")
    ax.set_title("The screening front — orange ring = Pareto, ★ = knee, "
                 "× = vetoed; marker size = structural mass", fontsize=9.5)
    if feas and any(c.cid == res.knee_id for c in feas):
        ax.legend(loc="best", fontsize=8)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    return fig


def render():
    import numpy as np
    import streamlit as st
    from suspension import omnicore as oc

    ss = st.session_state

    st.subheader("🪐 OmniCore — one mission sentence, three engines, "
                 "one refereed front")
    st.caption(
        "Subsystems are always at war: the geometry InverseGenesis wants may "
        "demand an actuator SimulForge shows browning the bus out; the gains "
        "that keep the car composed burn endurance energy; the bracket "
        "MorphMesh grows may be one your own shop class vetoes. This tab "
        "settles the war with arithmetic. A deterministic grammar (not a "
        "language model — the receipt below proves it) turns your mission "
        "sentence into a typed spec; one shared-subsolve sweep scores a "
        "declared configuration lattice on five axes in real currencies; and "
        "the survivors form a SCREENING Pareto front — the shortlist for the "
        "engines' own tabs, never a substitute for them, and never for "
        "ANSYS.")

    hp, hp_note = _hardpoints_from_session(ss)
    st.caption(f"Geometry: {hp_note}.")

    # ---------------- 1 · the mission sentence -----------------------------
    mission_text = st.text_area(
        "Mission profile (one sentence; the grammar reads terrain, budget, "
        "±tolerance, drive, and priority words)",
        value=_MISSION_PLACEHOLDER, height=90, key="omni_mission")
    mission = oc.parse_mission(mission_text)

    with st.expander("The parse receipt — what the grammar understood, "
                     "assumed, and ignored", expanded=True):
        for c in mission.consumed:
            st.markdown(f"- ✅ {c}")
        for a in mission.assumptions:
            st.markdown(f"- ➖ assumed: {a}")
        if mission.ignored:
            st.markdown(f"- 🕳️ ignored (a grammar, not a language model): "
                        f"{', '.join(mission.ignored)}")

    # ---------------- 2 · the lattice & the arithmetic ----------------------
    with st.expander("The lattice, the fidelity, and the cost/mass "
                     "arithmetic — every constant a knob", expanded=False):
        k1, k2, k3 = st.columns(3)
        compare = k1.checkbox("Compare with the next-better shop class",
                              value=True, key="omni_cmp",
                              help="Prices what a jig (or a machinist) is "
                                   "actually worth on the same front.")
        n_tabs = k2.number_input("Grown tabs per car", 2, 24, 8, 1,
                                 key="omni_ntabs")
        events = k3.number_input("Transient events per lap", 2.0, 40.0, 14.0,
                                 1.0, key="omni_events",
                                 help="Folds the actuator's measured event "
                                      "energy into the endurance budget.")
        b1, b2, b3 = st.columns(3)
        base_usd = b1.number_input("Base-vehicle cost (USD)", 0.0, 100000.0,
                                   11000.0, 500.0, key="omni_base",
                                   help="Everything this sweep does NOT vary."
                                        " The mission budget referees "
                                        "base + decision scope.")
        band_c = b2.number_input("Camber band (± deg)", 0.05, 1.0, 0.25,
                                 0.05, key="omni_bandc")
        band_t = b3.number_input("Toe band (± deg)", 0.05, 0.6, 0.12, 0.01,
                                 key="omni_bandt")
        st.caption("Screening fidelity is fixed and printed on the result "
                   "(coarse genesis starts, coarse growth mesh) — the knee "
                   "is meant to be PROMOTED to the engines' own tabs at "
                   "their full settings.")

    # ---------------- 3 · run the referee ----------------------------------
    run = st.button("Referee the trade-offs", type="primary", key="omni_run")
    if not run and "omnicore_result" not in ss:
        st.info("Type the mission and press the button. One genesis solve "
                "per shop class, one mechatronic co-solve + ghost audit per "
                "actuator size, one topology growth per configuration — "
                "shared on purpose, ~12 whole-vehicle configurations in "
                "under a minute on a laptop.")
        return

    if run:
        knobs = oc.OmniKnobs(compare_shops=bool(compare),
                             n_tabs=int(n_tabs),
                             events_per_lap=float(events),
                             base_vehicle_usd=float(base_usd),
                             band_camber_deg=float(band_c),
                             band_toe_deg=float(band_t))
        status = st.status("Refereeing…", expanded=True)
        try:
            res = oc.run_omnicore(hp, mission, knobs,
                                  progress=lambda s: status.write(s))
            status.update(label=f"Refereed in {res.elapsed_s:.1f} s",
                          state="complete", expanded=False)
        except Exception as err:          # noqa: BLE001 — surface, don't crash
            status.update(state="error")
            st.error(f"OmniCore failed: {err}")
            return
        ss["omnicore_result"] = res
        ss["omnicore_last"] = res.summary()

    res = ss["omnicore_result"]
    _show_result(st, np, res)

    # ---------------- 6 · the self-healing twin ----------------------------
    st.divider()
    _render_twin(st, np, res)


def _show_result(st, np, res):
    from suspension import omnicore as oc

    n_feas = sum(1 for c in res.configs if c.feasible)
    icon = "🟢" if n_feas else "🔴"
    st.markdown(f"## {icon} {n_feas} of {len(res.configs)} configurations "
                f"survived the referees — {len(res.pareto_ids)} on the front")
    st.caption(res.knobs.fidelity_note() + ".")

    knee = next((c for c in res.configs if c.cid == res.knee_id), None)
    if knee is not None:
        a, b, c_, d, e = st.columns(5)
        a.metric("⭐ Referee's pick", knee.label.split(" ", 1)[1],
                 help="Min priority-weighted normalised distance to the "
                      "utopia point, under the mission's own stated "
                      "priorities. ONE reading of the front — the front is "
                      "the answer.")
        b.metric("Composure", oc.fmt_axis("composure",
                                          knee.objectives["composure"])
                 + " deg·s",
                 help="∫|roll|dt over the mission manoeuvre — a lap-time "
                      "PROXY; mapping deg·s to seconds is the lap-sim's "
                      "job.")
        c_.metric("Range", oc.fmt_axis("laps", knee.objectives["laps"])
                  + " laps",
                  help="Pack laps with the actuator's event energy folded "
                       "in — Earshot's currency.")
        d.metric("Decision cost", "$" + oc.fmt_axis("cost",
                                                    knee.objectives["cost"]),
                 help="Only what this sweep varies: actuators, tabs, shop "
                      "capex — on top of the declared base-vehicle cost.")
        e.metric("Build yield", oc.fmt_axis("yield",
                                            knee.objectives["yield"]),
                 help="InverseGenesis' fraction-of-built-cars-in-band at "
                      "this shop's declared error field.")

    # ---------------- 4 · the front ----------------------------------------
    st.markdown("#### The screening front")
    fig = _draw_front(res)
    st.pyplot(fig, clear_figure=True)

    # ---------------- 5 · the scorecard, vetoes and receipts ---------------
    st.markdown("#### Scorecard")
    hdr = ["config"] + [f"{oc.AXES[k][0]} ({oc.AXES[k][1]})"
                        for k in oc.AXES] + ["verdict"]
    rows = []
    for c in res.configs:
        tag = ("⭐ knee" if c.cid == res.knee_id else
               "front" if c.cid in res.pareto_ids else
               "dominated" if c.feasible else "INFEASIBLE")
        rows.append([c.label] + [oc.fmt_axis(k, c.objectives.get(
            k, float("nan"))) for k in oc.AXES] + [tag])
    st.table({h: [r[i] for r in rows] for i, h in enumerate(hdr)})

    vetoed = [(c, v) for c in res.configs for v in c.vetoes]
    flagged = [(c, f) for c in res.configs for f in c.flags]
    if vetoed:
        with st.expander(f"🔴 The vetoes — {len(vetoed)} (each names its "
                         "referee)", expanded=not n_feas):
            for c, v in vetoed:
                st.markdown(f"- **{c.label}** — {v}")
    if flagged:
        with st.expander(f"🟡 The flags — {len(flagged)}", expanded=False):
            for c, f in flagged:
                st.markdown(f"- **{c.label}** — {f}")
    if res.receipts:
        with st.expander("🧾 Dominance receipts — who beats whom, on every "
                         "axis at once", expanded=False):
            for r in res.receipts:
                st.markdown(f"- {r}")
    for w in res.warnings:
        st.warning(w)

    st.caption(
        f"Engine ledger: {res.ledger['genesis']} genesis solves, "
        f"{res.ledger['simulforge']} co-solves, "
        f"{res.ledger['ghost_audits']} ghost audits, "
        f"{res.ledger['morph']} growths ({res.ledger['fe_solves']} FE "
        f"solves) in {res.elapsed_s:.1f} s.")

    d1, d2 = st.columns(2)
    d1.download_button("⬇️ Referee report (.md)", oc.render_omni_md(res),
                       "omnicore_referee.md", "text/markdown",
                       key="omni_dl_md")
    d2.download_button("⬇️ Result JSON", res.to_json(), "omnicore.json",
                       "application/json", key="omni_dl_json")


def _render_twin(st, np, res):
    from suspension import omnicore as oc

    st.markdown("## 🩺 The self-healing twin")
    st.caption(
        "Feed measured telemetry summaries from the RUNNING car back into "
        "the nominal model. Each channel is judged against its declared "
        "band; the deviation PATTERN is cosine-matched against the "
        "signature every named Degradation preset predicts (the Saboteur's "
        "trick, pointed at the physical car) — so drift comes back with a "
        "named suspect, and the heal plan is arithmetic: a gain de-rate "
        "from what the sagged bus can deliver, and a printable shim from "
        "the measured camber drift. A screening diagnosis for the audit to "
        "start with, not a certified root cause.")

    mission = res.mission
    twin = st.session_state.get("omnicore_twin")
    stale = twin is not None and twin[0].maneuver != mission.maneuver
    if twin is None or stale:
        if st.button("Build the nominal baseline + defect signature catalog "
                     f"(one co-solve per named defect, '{mission.maneuver}')",
                     key="omni_twin_build"):
            status = st.status("Solving signatures…", expanded=True)
            try:
                base, forge0 = oc.twin_baseline(mission.maneuver)
                sigs = oc.defect_signatures(
                    base, progress=lambda s: status.write(s))
                status.update(label=f"{len(sigs)} defect signatures solved",
                              state="complete", expanded=False)
            except Exception as err:                          # noqa: BLE001
                status.update(state="error")
                st.error(f"Twin baseline failed: {err}")
                return
            st.session_state["omnicore_twin"] = (base, forge0, sigs)
            twin = (base, forge0, sigs)
        else:
            return
    base, forge0, sigs = twin

    st.markdown("#### Measured telemetry (edit to what the logger says)")
    cols = st.columns(4)
    measured = {}
    for i, (k, (lab, unit, band)) in enumerate(oc.TWIN_CHANNELS.items()):
        nom = float(base.channels[k])
        span = max(abs(nom), band) * 4.0 + 4.0 * band
        measured[k] = cols[i % 4].number_input(
            f"{lab} ({unit})", nom - span, nom + span, nom,
            band / 2.0, key=f"omni_tw_{k}",
            help=f"Nominal {nom:.3f}; declared band ±{band:g}.")
    c1, c2 = st.columns(2)
    camber_drift = c1.number_input(
        "Measured camber drift vs alignment sheet (deg)", -3.0, 3.0, 0.0,
        0.05, key="omni_tw_cam")
    span_mm = c2.number_input("Mount bolt span for the shim (mm)", 20.0,
                              200.0, 80.0, 5.0, key="omni_tw_span")

    diag = oc.diagnose(base, measured, sigs)
    plan = oc.heal_plan(diag, forge0, camber_drift_deg=float(camber_drift),
                        bolt_span_mm=float(span_mm))

    worst = max((abs(v) for v in diag.z.values()), default=0.0)
    icon = "🟢" if worst <= 1.0 else "🟡" if worst <= 2.0 else "🔴"
    st.markdown(f"### {icon} {diag.note}")
    st.table({
        "channel": [f"{oc.TWIN_CHANNELS[k][0]} ({oc.TWIN_CHANNELS[k][1]})"
                    for k in diag.z],
        "nominal": [f"{base.channels[k]:.3f}" for k in diag.z],
        "measured": [f"{diag.measured[k]:.3f}" for k in diag.z],
        "z (bands)": [f"{diag.z[k]:+.2f}" for k in diag.z],
        "verdict": [diag.channel_verdicts[k] for k in diag.z],
    })
    if diag.suspect is not None:
        st.warning(f"**Named suspect:** {diag.suspect_label} — cosine "
                   f"{diag.cosine:.2f}, magnitude {diag.magnitude:.2f}× the "
                   "predicted signature. Start the audit there.")

    if diag.drifting or abs(camber_drift) > 1e-6:
        st.markdown("#### The heal plan — arithmetic, not vibes")
        st.markdown(f"- ⚙️ {plan.gain_note}")
        if plan.kp_new is not None and plan.gain_scale is not None \
                and plan.gain_scale < 0.999:
            st.code(f"kp = {plan.kp_new:.0f}   kd = {plan.kd_new:.0f}   "
                    f"(scale {plan.gain_scale:.2f})", language="text")
        st.markdown(f"- 🖨️ {plan.shim_note}")
        for c in plan.cautions:
            st.markdown(f"- ⚠️ {c}")
        st.download_button("⬇️ Twin report (.md)",
                           oc.render_twin_md(diag, plan), "omnicore_twin.md",
                           "text/markdown", key="omni_dl_twin")
