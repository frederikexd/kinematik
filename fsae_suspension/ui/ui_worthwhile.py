# ============================================================================
#  KinematiK — ui/worthwhile.py
#  Streamlit panel for the "Is it worthwhile once assembled?" verdict. Reads the
#  IntegrationLedger, runs the no-go gate + paper-vs-real lap-sim, and shows the
#  points a team actually loses when the estimates meet reality — refusing to
#  print a score when a blocking contradiction makes the car unbuildable.
# ============================================================================
"""Render the Worthwhileness tab.

Design intent, consistent with the rest of KinematiK: this panel is built to
FAIL LOUDLY. When the assembly can't be built as declared, it shows a red no-go
with the named contradictions and NO points number — never a reassuring score
over a broken budget. Only a buildable assembly gets a worthwhileness delta.
"""

from __future__ import annotations

try:
    import streamlit as st
except Exception:
    st = None

from suspension.interfaces import IntegrationLedger, Severity
from suspension.worthwhile import assess, Assumption, PROVENANCE
from suspension.dynamics import VehicleParams


_SEV_ICON = {
    Severity.OK: "✅", Severity.INFO: "ℹ️", Severity.WARN: "⚠️",
    Severity.FAIL: "❌", Severity.MISSING: "⭕",
}


def render(ledger: IntegrationLedger | None = None,
           assumptions: list | None = None,
           paper_baseline: VehicleParams | None = None,
           front_kin=None, rear_kin=None, tire=None):
    if st is None:
        raise RuntimeError("streamlit not available")
    ss = st.session_state

    st.subheader("🏁 Worthwhile When Assembled — the go / no-go verdict")
    st.caption(
        "Takes the reconciled subsystem ledger, pushes the REAL mass and CG into "
        "the vehicle model, runs the lap sim, and reports the points you actually "
        "lose when the estimates meet reality — and refuses to print a score at "
        "all when a hard contradiction means the car can't be built as declared."
    )

    st.info("**How to read this.** " + PROVENANCE["hard_rule"], icon="⚖️")

    ledger = ledger or ss.get("integration_ledger")
    if ledger is None:
        st.warning("No integration ledger loaded. Declare subsystem interfaces in "
                   "the Integration tab first — this verdict reconciles them.")
        return

    paper = paper_baseline or VehicleParams()
    with st.expander("Paper baseline (the optimistic car the subsystems assumed)",
                     expanded=False):
        c1, c2 = st.columns(2)
        with c1:
            pm = st.number_input("Paper all-up mass (kg)", value=float(paper.mass),
                                 key="wt_pmass")
        with c2:
            pz = st.number_input("Paper CG height (mm)", value=float(paper.cg_height),
                                 key="wt_pcg")
        paper = VehicleParams(mass=pm, cg_height=pz)

    if st.button("Run worthwhileness verdict", type="primary", key="wt_run"):
        with st.spinner("Reconciling and simulating…"):
            ss["wt_verdict"] = assess(
                ledger, assumptions=assumptions or [], paper_baseline=paper,
                front_kin=front_kin, rear_kin=rear_kin, tire=tire)

    v = ss.get("wt_verdict")
    if v is None:
        st.caption("Run to get the verdict.")
        return

    # ---- the verdict banner --------------------------------------------
    if not v.buildable:
        st.error(f"### ❌ NOT BUILDABLE AS DECLARED\n\n{v.verdict_text}")
        st.markdown("#### Blocking issues (resolve these — points are withheld)")
        for f in v.blocking:
            st.markdown(f"{_SEV_ICON.get(f.severity, '•')} **{f.check}** "
                        f"({', '.join(f.subsystems) or '—'}) — {f.message}")
        st.caption("No worthwhileness score is shown on purpose: a points number "
                   "for a car that can't be assembled would be misleading. Fix the "
                   "blocking items and re-run.")
        _render_findings(v)
        return

    # buildable: show the delta
    st.success(f"### 🏁 BUILDABLE\n\n{v.verdict_text}")

    st.markdown("#### Points: paper car vs the car you'll actually build")
    try:
        import pandas as pd
        events = [k for k in v.real_points if k != "total"]
        rows = []
        for ev in events + ["total"]:
            rows.append({
                "Event": ev.capitalize(),
                "Paper": round(v.paper_points[ev], 1),
                "Reconciled": round(v.real_points[ev], 1),
                "Δ points": round(v.points_delta[ev], 1),
            })
        df = pd.DataFrame(rows)
        st.dataframe(df, use_container_width=True, hide_index=True)
    except Exception as e:
        st.caption(f"(table unavailable: {e})")

    tot = v.total_delta()
    st.metric("Total worthwhileness delta", f"{tot:+.1f} points",
              help="What the reconciled build costs vs the optimistic paper car.")
    if v.any_estimate:
        st.warning("Some reconciled inputs are still ESTIMATES — treat the delta "
                   "as directional until they're confirmed.", icon="⚠️")

    _render_findings(v)


def _render_findings(v):
    with st.expander("All integration findings", expanded=False):
        order = [Severity.FAIL, Severity.MISSING, Severity.WARN,
                 Severity.INFO, Severity.OK]
        by_sev = {s: [] for s in order}
        for f in v.findings:
            by_sev.setdefault(f.severity, []).append(f)
        for sev in order:
            for f in by_sev.get(sev, []):
                st.markdown(f"{_SEV_ICON.get(sev, '•')} **{f.check}** "
                            f"({', '.join(f.subsystems) or '—'}) — {f.message}")
