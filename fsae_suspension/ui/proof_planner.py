# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  ui/proof_planner.py — the 🎯 Proof Planner tab
# ============================================================================
"""
The tab that makes the tagline literal: it reads the live Integration ledger,
quantifies how uncertain every declared number really is (evidence grade +
staleness), attributes the team's top-level uncertainty (lap time, energy,
thermal margin, mass) to its inputs, and hands back a ranked list of the
questions worth asking — corner scales before ANSYS when that is what the
arithmetic says. Any planned action can be turned into a SEALED validation
contract: the acceptance band is fixed and hashed before the run, and the
returned result is judged PASS / FAIL / DISCREPANT against the band that
provably never moved.

All physics and all state transitions live in suspension/proof_engine.py.
This module only orchestrates and draws (see ui/__init__.py rules).
Session keys used:
    proof_pedigree   dict quantity_key -> {"grade","measured_on","source"}
    proof_contracts  list of ValidationContract dicts
    ledger           the shared IntegrationLedger dict (read-only here)
"""

from __future__ import annotations


def _frame_note(ss) -> str:
    """The declared team coordinate convention, if the Frames tab saved one."""
    ch = ss.get("frames_charter") or {}
    name = ch.get("frame_name") or ch.get("name") or ""
    return f"{name} (team charter)" if name else ""


def render():
    import streamlit as st
    from suspension import units as _units
    from suspension import proof_engine as pe
    from suspension.interfaces import IntegrationLedger

    ss = st.session_state
    ss.setdefault("proof_pedigree", {})
    ss.setdefault("proof_contracts", [])

    st.subheader("🎯 Proof Planner — what to validate next, and why")
    st.caption(
        "Every declared number carries a quantified ± from its evidence grade "
        "and age. The planner propagates those bands to the objective you "
        "pick, shows which inputs dominate, and ranks the actions that retire "
        "the most uncertainty per hour — so solver time goes where the doubt "
        "is, not where the habit is. Deterministic: same ledger, same plan.")

    led = IntegrationLedger.from_dict(ss.get("ledger") or {})
    quantities = pe.build_uncertainty_ledger(led, overrides=ss.proof_pedigree)
    if not quantities:
        st.info("The Integration ledger has no declared numbers yet. Declare "
                "your subsystem interfaces in 🔗 Integration first — the "
                "Proof Planner plans proofs for numbers that exist.")
        return

    # ---------------- 1 · evidence pedigrees --------------------------------
    with st.expander("1 · Evidence pedigree — where did each number come from?",
                     expanded=False):
        st.caption(
            "The ledger's estimate checkbox seeds *estimate* (±20 %) or "
            "*modelled* (±10 %). Claiming *measured* or *verified* is done "
            "here, with a date — a checkbox is not an instrument. Bands "
            "inflate as evidence ages and never shrink on their own.")
        grades = [g.value for g in pe.EvidenceGrade]
        for q in quantities:
            c1, c2, c3, c4 = st.columns([3, 2, 2, 3])
            c1.markdown(f"**{q.label}**  \n{q.value:g} {q.unit} · "
                        f"±{q.abs_unc():.3g} {q.unit} "
                        f"(±{q.rel_unc() * 100:.0f} %)")
            g = c2.selectbox("grade", grades, index=grades.index(q.grade.value),
                             key=f"pp_g_{q.key}", label_visibility="collapsed")
            d = c3.text_input("evidence date (YYYY-MM-DD)",
                              value=q.measured_on, key=f"pp_d_{q.key}",
                              label_visibility="collapsed",
                              placeholder="YYYY-MM-DD")
            s = c4.text_input("source", value=q.source, key=f"pp_s_{q.key}",
                              label_visibility="collapsed",
                              placeholder="who/what produced it")
            if (g != q.grade.value or d != q.measured_on or s != q.source):
                ss.proof_pedigree[q.key] = {
                    "grade": g, "measured_on": d, "source": s}
        if ss.proof_pedigree:
            st.caption(f"{len(ss.proof_pedigree)} pedigree override(s) active.")
        quantities = pe.build_uncertainty_ledger(led,
                                                 overrides=ss.proof_pedigree)

    # ---------------- 2 · pick the objective --------------------------------
    objs = {o.label: o for o in pe.DEFAULT_OBJECTIVES}
    olabel = st.selectbox("Objective the plan optimises for",
                          list(objs.keys()), index=1)
    obj = objs[olabel]
    report = pe.analyze_objective(obj, quantities)
    plan = pe.plan_proofs(obj, quantities)

    a, b, c = st.columns(3)
    a.metric(obj.label, f"{report.nominal:.3g} {obj.unit}")
    b.metric("Uncertainty now", f"± {report.total_unc:.3g} {obj.unit}")
    c.metric("Floor if plan completed", f"± {plan.unc_floor:.3g} {obj.unit}")
    if obj.confidence == "coupled":
        st.caption("Evaluator class: **coupled** — a documented surrogate for "
                   "planning, not a lap sim. The mechanisms and sensitivities "
                   "are in the module docstring; challenge them.")

    # ---------------- 3 · attribution ---------------------------------------
    st.markdown("**Where the uncertainty comes from** — one-at-a-time "
                "perturbation, reproducible by hand:")
    for at in report.attributions[:8]:
        st.progress(min(at.share, 1.0),
                    text=f"{at.label} — {at.grade} · ±{at.input_unc:.3g} in → "
                         f"±{at.delta_out:.3g} {obj.unit} out "
                         f"({at.share * 100:.0f} %)")

    # ---------------- 4 · the ranked plan -----------------------------------
    st.markdown("**What to prove next** — ranked by certainty bought per hour:")
    for i, it in enumerate(plan.items, 1):
        if it.unc_retired <= 0:
            continue
        st.markdown(
            f"{i}. **{it.action_label}** ({it.tool}, {it.hours:g} h) — "
            f"retires ±{it.unc_retired:.3g} {obj.unit} "
            f"({it.value_per_hour:.3g}/h). _{it.note}_")
    zero = [it for it in plan.items if it.unc_retired <= 0]
    if zero:
        st.caption("Not worth doing for this objective right now: "
                   + ", ".join(it.action_label for it in zero)
                   + ". (They may rank for a different objective — switch "
                     "above and see.)")

    st.download_button(
        "⬇️ Proof plan (markdown, pinnable)",
        pe.render_proof_plan_md(plan, report, frame_note=_frame_note(ss)),
        file_name=f"proof_plan_{obj.key}.md", mime="text/markdown",
        key="pp_dl_plan")

    st.divider()

    # ---------------- 5 · sealed validation contracts -----------------------
    st.markdown("### 🔏 Validation contracts — decide what *pass* means "
                "**before** the run")
    st.caption(
        "Pre-registration, borrowed from experimental science: the acceptance "
        "band and the 3σ plausibility envelope are fixed and hashed when the "
        "contract is sealed. A result inside the band is PASS; outside the "
        "band but plausible is FAIL (a design finding); outside the envelope "
        "is DISCREPANT — the run and the ledger disagree about reality, and "
        "neither number should be trusted until the units, frame, BCs and "
        "geometry version are audited. Editing a sealed band breaks the seal, "
        "and a broken seal refuses judgment. Goalposts cannot silently move.")

    with st.expander("Seal a new contract", expanded=False):
        acts = {a_.label: a_ for a_ in pe.DEFAULT_ACTIONS}
        qmap = {q.label: q for q in quantities}
        ca1, ca2 = st.columns(2)
        alabel = ca1.selectbox("Evidence action / sim", list(acts.keys()),
                               key="pp_c_act")
        qlabel = ca2.selectbox("Channel under test", list(qmap.keys()),
                               key="pp_c_q")
        q = qmap[qlabel]
        st.caption(f"Ledger prediction: {q.value:g} ± {q.abs_unc():.3g} "
                   f"{q.unit} → plausibility envelope "
                   f"[{q.value - 3 * q.abs_unc():.3g}, "
                   f"{q.value + 3 * q.abs_unc():.3g}] {q.unit} (not yours to "
                   "choose — that is what makes DISCREPANT honest).")
        cb1, cb2 = st.columns(2)
        lo = cb1.number_input(f"Pass band low ({q.unit})",
                              value=float(q.value) * 0.9, key="pp_c_lo")
        hi = cb2.number_input(f"Pass band high ({q.unit})",
                              value=float(q.value) * 1.1, key="pp_c_hi")
        note = st.text_input("Why this band (the design judge's question)",
                             key="pp_c_note",
                             placeholder="e.g. FoS ≥ 1.5 at the rules load case")
        author = st.text_input("Sealed by", key="pp_c_author")
        if st.button("🔏 Seal contract", key="pp_c_seal"):
            try:
                c_ = pe.create_contract(acts[alabel], q, lo, hi,
                                        note or "(no criterion note)", author)
                ss.proof_contracts.append(c_.as_dict())
                st.success(f"Sealed. Seal `{c_.seal[:16]}…` — hand the brief "
                           "to whoever runs it.")
            except ValueError as e:
                st.error(str(e))

    for idx, cd in enumerate(ss.proof_contracts):
        c_ = pe.ValidationContract.from_dict(cd)
        sealed_ok = c_.verify_seal()
        badge = {"open": "🟡 OPEN", "pass": "🟢 PASS", "fail": "🔴 FAIL",
                 "discrepant": "🟣 DISCREPANT"}.get(c_.status, c_.status)
        if not sealed_ok:
            badge = "⚠️ SEAL BROKEN"
        with st.expander(f"{badge} — {c_.title} · sealed {c_.created_on}"):
            act = next((a_ for a_ in pe.DEFAULT_ACTIONS
                        if a_.key == c_.action_key), None)
            st.markdown(pe.render_contract_brief_md(
                c_, act, frame_note=_frame_note(ss)))
            if c_.status == pe.Verdict.OPEN.value and sealed_ok:
                r1, r2 = st.columns([2, 1])
                val = r1.number_input(f"Result ({c_.unit})",
                                      value=float(c_.predicted),
                                      key=f"pp_r_{c_.id}")
                if r2.button("Judge", key=f"pp_j_{c_.id}"):
                    try:
                        judged = pe.judge_result(c_, val)
                        ss.proof_contracts[idx] = judged.as_dict()
                        st.rerun()
                    except ValueError as e:
                        st.error(str(e))
            if c_.status == pe.Verdict.PASS.value:
                st.caption("Next step: upgrade the channel's pedigree above "
                           "to *measured* (or *modelled* for a sim) with "
                           "today's date and this contract as the source — "
                           "the whole plan re-ranks around the new certainty.")
            st.download_button("⬇️ Contract brief (markdown)",
                               pe.render_contract_brief_md(
                                   c_, act, frame_note=_frame_note(ss)),
                               file_name=f"contract_{c_.id}.md",
                               mime="text/markdown", key=f"pp_dlc_{c_.id}")
