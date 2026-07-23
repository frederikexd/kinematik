# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  ui/phantom_car.py — the 👻 Phantom Car tab (the margin audit)
# ============================================================================
"""
The tab that adds your conservatism up. Every subsystem hedges the same
uncertainty separately and in secret; this tab makes each consumer DISCLOSE
the design value its sizing actually uses, prices every hedge in σ of that
quantity's own evidence-graded band (the Proof Engine ledger, third consumer),
judges the lot against one sha256-sealed Margin Charter percentile, and prices
the design envelope currently spent defending cars the deck's own σ says
cannot exist.

All physics and all state transitions live in suspension/phantom_car.py.
This module only orchestrates and draws (see ui/__init__.py rules).
Session keys used:
    proof_pedigree    dict quantity_key -> pedigree override (SHARED with the
                      Proof Planner and the Saboteur — one pedigree, three
                      consumers, on purpose)
    margin_charter    the sealed MarginCharter dict (None until sealed)
    margin_decls      list of MarginDeclaration dicts (the disclosure form)
    ledger            the shared IntegrationLedger dict (read-only here)
"""

from __future__ import annotations


def _frame_note(ss) -> str:
    ch = ss.get("frames_charter") or {}
    name = ch.get("frame_name") or ch.get("name") or ""
    return f"{name} (team charter)" if name else ""


_VERDICT_STYLE = {
    "ALIGNED":       ("🟢", "success"),
    "STACKED":       ("🟠", "warning"),
    "UNDER-COVERED": ("🟠", "warning"),
    "NAKED":         ("🔴", "error"),
    "ANTI-HEDGED":   ("🔴", "error"),
}


def render():
    import streamlit as st
    from suspension import units as _units
    from suspension import proof_engine as pe
    from suspension import phantom_car as ph
    from suspension.interfaces import IntegrationLedger

    ss = st.session_state
    ss.setdefault("proof_pedigree", {})
    ss.setdefault("margin_charter", None)
    ss.setdefault("margin_decls", None)

    st.subheader("👻 Phantom Car — the car your margins actually designed")
    st.caption(
        "Every subsystem hedges the same uncertainty separately, in secret, "
        "and nobody adds it up. This tab makes each consumer disclose the "
        "value its sizing actually uses, prices every hedge in σ of the "
        "quantity's own evidence-graded band, judges the lot against one "
        "sealed design percentile, and names the envelope spent defending "
        "cars the deck's own σ says cannot exist. Deterministic: same deck, "
        "same docket.")

    led = IntegrationLedger.from_dict(ss.get("ledger") or {})
    quantities = pe.build_uncertainty_ledger(led, overrides=ss.proof_pedigree)
    if not quantities:
        st.info("The Integration ledger has no declared numbers yet. Declare "
                "your subsystem interfaces in 🔗 Integration first — the "
                "Phantom Car audits assumptions about numbers that exist, and "
                "prices each hedge in the band the Proof Engine gives them.")
        return

    cars = ph.car_quantities(quantities)

    # ---------------- 1 · the sealed Margin Charter -------------------------
    st.markdown("### 🔏 Margin Charter — one percentile for the whole car")
    st.caption(
        "The team's single answer to *how bad a car do we design for?*. "
        "Sealed before the audit, like a validation contract — the audit "
        "refuses to judge against a percentile that moved to flatter it.")

    charter = None
    if ss.margin_charter:
        charter = ph.MarginCharter.from_dict(ss.margin_charter)

    if charter is not None and charter.verify_seal():
        c1, c2 = st.columns([3, 1])
        c1.success(
            f"Sealed: design to the **{charter.percentile:.1f}th-percentile "
            f"car** (z* = {charter.z:.2f}σ), factor rule *{charter.fos_rule}*. "
            f"Seal `{charter.seal[:16]}…` on {charter.created_on}."
            + (f" — _{charter.note}_" if charter.note else ""))
        if c2.button("Re-seal / change", key="ph_reseal"):
            ss.margin_charter = None
            st.rerun()
    else:
        if charter is not None and not charter.verify_seal():
            st.error("The charter's sealed fields changed after sealing. A "
                     "verdict against a movable percentile is worthless — "
                     "re-seal below before the audit will judge anything.")
        with st.form("ph_charter_form"):
            pct = st.slider("Design percentile — the car you build for",
                            50.0, 99.9, 95.0, 0.1, key="ph_pct",
                            help="95 means: we design to the car that is worse "
                                 "than 95% of the statistically plausible "
                                 "population implied by the ledger's σ.")
            note = st.text_input("Charter note (survives into the Registry)",
                                 key="ph_note",
                                 placeholder="one phantom, not eight")
            fos_rule = st.selectbox(
                "Design-factor rule", ["once", "stacked-ok"], index=0,
                key="ph_fosrule",
                help="'once': a chain carries EITHER a σ-hedge to the charter "
                     "OR an explicit factor, never both silently — the "
                     "default and the honest one.")
            if st.form_submit_button("🔏 Seal the charter"):
                ss.margin_charter = ph.create_charter(
                    pct, note=note, fos_rule=fos_rule,
                    author=ss.get("member_name", "") or "").as_dict()
                st.rerun()
        st.info("Seal a Margin Charter to run the audit.")
        return

    # ---------------- 2 · the disclosure form -------------------------------
    st.divider()
    st.markdown("### 📋 Disclosure — what each consumer's sizing actually uses")
    st.caption(
        "Disclosure, not new work: the number already lives in each "
        "subsystem's spreadsheet. Seeds start at NOMINAL on purpose — a fresh "
        "audit says NAKED where nobody has disclosed cover, rather than "
        "fabricating prudence nobody has.")

    dc1, dc2 = st.columns(2)
    if dc1.button("Seed from the FSAE-EV consumption map", key="ph_seed"):
        ss.margin_decls = [_decl_to_dict(d)
                           for d in ph.seed_declarations(quantities)]
        st.rerun()
    if dc2.button("Load the textbook pathology (demo)", key="ph_demo"):
        ss.margin_decls = [_decl_to_dict(d)
                           for d in ph.demo_declarations(quantities)]
        st.rerun()

    if ss.margin_decls is None:
        ss.margin_decls = [_decl_to_dict(d)
                           for d in ph.seed_declarations(quantities)]

    key_labels = {cq.key: f"{cq.label} ({cq.key})" for cq in cars.values()}
    keys = list(key_labels.keys())

    editor_rows = []
    for d in ss.margin_decls:
        editor_rows.append({
            "consumer": d["consumer"],
            "quantity": key_labels.get(d["quantity_key"], d["quantity_key"]),
            "adverse (high/low)": d["adverse"],
            "assumed value": d["assumed_value"],
            "×factor on top": d["design_factor"],
            "rationale": d.get("rationale", ""),
        })
    edited = st.data_editor(
        editor_rows, num_rows="dynamic", use_container_width=True,
        hide_index=True, key="ph_editor",
        column_config={
            "quantity": st.column_config.SelectboxColumn(
                options=list(key_labels.values())),
            "adverse (high/low)": st.column_config.SelectboxColumn(
                options=["high", "low"]),
        })

    # fold the editor back into declaration dicts
    label_to_key = {v: k for k, v in key_labels.items()}
    new_decls = []
    for r in edited:
        qkey = label_to_key.get(r.get("quantity"), r.get("quantity"))
        if not r.get("consumer") or qkey not in keys:
            continue
        new_decls.append({
            "consumer": str(r["consumer"]),
            "quantity_key": qkey,
            "adverse": r.get("adverse (high/low)") or "high",
            "assumed_value": float(r.get("assumed value") or 0.0),
            "design_factor": float(r.get("×factor on top") or 1.0),
            "rationale": r.get("rationale", "") or "",
        })
    ss.margin_decls = new_decls

    decls = [ph.MarginDeclaration(**d) for d in new_decls]
    if not decls:
        st.warning("No disclosures yet — add a consumer row, or seed above.")
        return

    # ---------------- 3 · the audit -----------------------------------------
    audit = ph.audit(charter, decls, quantities)
    if audit.refused:
        st.error("Audit refused — " + audit.refusal_reason)
        return

    st.divider()
    st.markdown("### ⚖️ Verdicts")

    counts = {}
    for f in audit.findings:
        counts[f.verdict] = counts.get(f.verdict, 0) + 1
    mc = st.columns(len(_VERDICT_STYLE))
    for col, (v, (icon, _)) in zip(mc, _VERDICT_STYLE.items()):
        col.metric(f"{icon} {v}", counts.get(v, 0))

    rows = []
    for f in audit.findings:
        icon = _VERDICT_STYLE.get(f.verdict, ("", ""))[0]
        note = ""
        if f.releasable:
            note = f"{f.releasable:.3g} {f.unit} releasable"
        elif f.exposure:
            note = f"{f.exposure:.3g} {f.unit} exposed"
        rows.append({
            "consumer": f.consumer,
            "quantity": f.quantity_label,
            "assumes": f"{f.assumed_value:.4g} {f.unit}",
            "hedge (σ)": f"{f.z_assumed:+.2f}",
            "×factor (σ)": f"{f.z_factor:+.2f} (×{f.design_factor:.2f})",
            "cover": f"{f.coverage_pct:.1f}% (grade: {f.worst_grade})",
            "verdict": f"{icon} {f.verdict}",
            "note": note,
        })
    st.dataframe(rows, use_container_width=True, hide_index=True)

    # ---------------- 4 · the two-cars detector -----------------------------
    two = [q for q in audit.quantity_coverage if q.contradictory]
    if two:
        st.markdown("### ⚠️ The deck describes more than one car")
        for q in two:
            st.error(
                f"**{q.label}** — *{q.hi_consumer}* designs to "
                f"{q.hi_assumed:.4g} {q.unit} while *{q.lo_consumer}* designs "
                f"to {q.lo_assumed:.4g} {q.unit}: **{q.spread_sigma:.1f}σ "
                f"apart**. Both cannot be the same physical car — one of these "
                "sizings is wrong today. This is the contradiction the "
                "Integration ledger kills for *values*, applied to "
                "*assumptions*.")

    # ---------------- 5 · β, the improbability defended against -------------
    if audit.consumer_phantoms:
        st.markdown("### 🎲 β — the improbability each consumer defends against")
        st.caption(
            "β = √(Σ z²) is the first-order reliability index (FORM) "
            "professional reliability engineering uses — the joint worst case "
            "each consumer stacks, stated as odds.")
        for p in audit.consumer_phantoms:
            st.markdown(
                f"- **{p.consumer}** "
                f"({p.n_hedged} stacked worst case"
                f"{'s' if p.n_hedged != 1 else ''}): {ph._odds_en(p)}")

    # ---------------- 6 · the three cars ------------------------------------
    if audit.objective_gaps:
        st.markdown("### 🚗 The three cars, per objective")
        st.caption(
            "**Nominal**: the deck's numbers. **Coherent**: one car at the "
            "charter percentile. **Phantom**: every channel at the most "
            "adverse value any consumer assumed — the union of everyone's "
            "private fears. The gap is design *envelope* spent defending cars "
            "the deck's σ says cannot exist — never promised savings; "
            "releasing it is a design decision, this tab only prices it.")
        grows = []
        for g in audit.objective_gaps:
            grows.append({
                "objective": g.label,
                "nominal": f"{g.nominal:.4g} {g.unit}",
                "coherent (charter)": f"{g.coherent:.4g} {g.unit}",
                "phantom": f"{g.phantom:.4g} {g.unit}",
                "over-defence vs charter": f"{g.gap:+.3g} {g.unit}",
            })
        st.dataframe(grows, use_container_width=True, hide_index=True)

    # ---------------- 7 · honest blind spots --------------------------------
    if audit.unaudited_consumers:
        st.warning(
            "**Unaudited consumers** — known sizing paths with no disclosed "
            "assumption. Their hedges are invisible to this docket until "
            "declared; they are named here, never absorbed into the board:\n"
            + "\n".join(f"- {u['consumer']} ← {u['quantity_key']}"
                        for u in audit.unaudited_consumers))
    if audit.unresolved:
        st.error("**Unresolved declarations** (quantity keys matching nothing "
                 "in the deck — fix, don't ignore): "
                 + ", ".join(audit.unresolved))

    # ---------------- 8 · the docket ----------------------------------------
    st.divider()
    st.download_button(
        "⬇️ Margin Docket (markdown, pin it to the design review)",
        ph.render_docket_md(audit, frame_note=_frame_note(ss)),
        file_name="margin_docket.md", mime="text/markdown", key="ph_dl")


def _decl_to_dict(d) -> dict:
    """MarginDeclaration -> plain dict for session storage (dataclass-safe)."""
    return {
        "consumer": d.consumer,
        "quantity_key": d.quantity_key,
        "adverse": d.adverse,
        "assumed_value": d.assumed_value,
        "design_factor": d.design_factor,
        "rationale": d.rationale,
    }
