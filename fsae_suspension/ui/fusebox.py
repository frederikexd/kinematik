# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  ui/fusebox.py — the ⛓️ Fusebox tab (the failure-order audit)
# ============================================================================
"""
The tab that asks, of every credible overload: WHICH component breaks first —
and did anyone choose it? Electronics answered with the fuse a century and a
half ago; mechanical load paths on a formula car answer by accident, at the
crash, in the currency of lead times — or, on an EV, of safety.

All statistics live in suspension/fusebox.py. This module only orchestrates
and draws (see ui/__init__.py rules).
Session keys used:
    fusebox_paths     list of OverloadPath dicts (the team's declared chains)
    fusebox_charter   sealed Fuse Charter dict (None until sealed)
"""

from __future__ import annotations


_VERDICT_STYLE = {
    "FUSED":       ("🟢", "success"),
    "COIN-FLIP":   ("🟠", "warning"),
    "INVERTED":    ("🟠", "warning"),
    "UNFUSED":     ("🔴", "error"),
    "BREACH-RISK": ("🔴", "error"),
}
_SEV_HELP = ("S1 = fuse-grade (bolt-on, spare in the box) · S2 = structural "
             "(custom / long-lead / frame) · S3 = forbidden-first "
             "(accumulator, cell restraint, firewall — a safety event, "
             "never a repair bill)")


def _frame_note(ss) -> str:
    ch = ss.get("frames_charter") or {}
    name = ch.get("frame_name") or ch.get("name") or ""
    return f"{name} (team charter)" if name else ""


def render():
    import streamlit as st
    from suspension import fusebox as fb
    from suspension.proof_engine import EvidenceGrade

    ss = st.session_state
    if "fusebox_paths" not in ss:
        ss.fusebox_paths = [p.as_dict() for p in fb.seed_paths()]
    ss.setdefault("fusebox_charter", None)

    st.subheader("⛓️ Fusebox — every load path fails somewhere. "
                 "Did the car choose where?")
    st.caption(
        "A big enough hit WILL break something on each chain — the only "
        "open question is which element, and whether that's the $45 tie "
        "rod in the spares box or the $900 six-week upright. Under your "
        "own evidence-graded σ the order isn't even determined: it's a "
        "weighted coin flip nobody has ever computed. This tab computes "
        "it, judges it against a sealed charter, and prints exact fixes — "
        "including the one no redesign meeting tables: measure, don't "
        "machine.")

    charter = ss.fusebox_charter
    conf = (charter or {}).get("confidence", fb.DEFAULT_CONFIDENCE)
    forb = (charter or {}).get("forbidden_p", fb.DEFAULT_FORBIDDEN_P)

    paths = [fb.OverloadPath.from_dict(d) for d in ss.fusebox_paths]
    audits = [fb.audit_path(p, conf, forb) for p in paths]
    amap = {a.path_key: a for a in audits}

    tab_order, tab_edit, tab_fix, tab_seal = st.tabs(
        ["🪜 Pecking order", "✏️ Paths & elements",
         "🛠 Fix arithmetic", "🔏 Fuse Charter"])

    # ------------------------------------------------------------ pecking order
    with tab_order:
        counts = {}
        for a in audits:
            counts[a.verdict.value] = counts.get(a.verdict.value, 0) + 1
        cols = st.columns(max(len(counts), 1))
        for c, (v, n) in zip(cols, sorted(counts.items())):
            c.metric(f"{_VERDICT_STYLE.get(v, ('⚪',))[0]} {v}", n)
        st.caption(
            f"Charter confidence {conf:.0%} · forbidden-first tolerance "
            f"{forb:.0%}"
            + ("" if charter else " (module defaults — no sealed charter yet)"))
        for p in paths:
            a = amap[p.key]
            icon, kind = _VERDICT_STYLE.get(a.verdict.value, ("⚪", "info"))
            with st.expander(f"{icon} {p.label} — {a.verdict.value}",
                             expanded=(kind != "success")):
                st.caption(p.story)
                getattr(st, kind if kind != "info" else "info")(a.headline)
                rows = []
                for e in p.elements:
                    rows.append({
                        "element": ("⛓️ " if e.key == p.designated_fuse_key
                                    else "") + e.label,
                        "severity": e.severity.value if e.severity else "—",
                        "FoS": round(e.fos, 2),
                        "grade": f"{e.grade.value} ±{e.rel_unc:.0%}",
                        "P(first)": f"{a.probs.get(e.key, 0.0):.1%}",
                        "cost $": round(e.replace_cost_usd),
                        "days": e.downtime_days,
                    })
                st.dataframe(rows, use_container_width=True, hide_index=True)
                b1, b2 = st.columns(2)
                b1.metric("Expected bill if the hit lands",
                          f"${a.expected_cost_usd:,.0f}",
                          f"{a.expected_downtime_days:.1f} days downtime",
                          delta_color="off")
                b2.metric("Bill if the intended fuse takes it",
                          f"${a.fuse_cost_usd:,.0f}",
                          f"{a.fuse_downtime_days:.1f} days downtime",
                          delta_color="off")
                for bsp in a.blind_spots:
                    st.warning(f"Blind spot: {bsp}")
        st.download_button(
            "📌 Download the Fuse Map (markdown)",
            fb.render_fusebox_md(paths, audits, charter, _frame_note(ss)),
            file_name="fuse_map.md", mime="text/markdown",
            key="fbx_dl_map")

    # ------------------------------------------------------------ path editor
    with tab_edit:
        st.caption("Declare each credible overload chain once: the elements "
                   "that share its load, each element's FoS **at the load "
                   "it sees** for this archetype, the evidence grade of "
                   "that capacity number (this prices its σ — the exact "
                   "Proof Engine band law, fifth consumer), a severity "
                   "class, and the repair bill. " + _SEV_HELP)
        c1, c2 = st.columns([3, 1])
        pk = c1.selectbox("Path", [p.key for p in paths],
                          format_func=lambda k: next(
                              pp.label for pp in paths if pp.key == k),
                          key="fbx_edit_path")
        if c2.button("↺ Reset all to seeds", key="fbx_reset"):
            ss.fusebox_paths = [p.as_dict() for p in fb.seed_paths()]
            st.rerun()
        pd_idx = next(i for i, p in enumerate(paths) if p.key == pk)
        pobj = paths[pd_idx]
        st.text_input("Story (what real event is this?)", pobj.story,
                      key=f"fbx_story_{pk}", disabled=True)
        fuse_opts = [e.key for e in pobj.elements]
        if fuse_opts:
            new_fuse = st.selectbox(
                "Designated fuse — the victim the team CHOOSES",
                fuse_opts,
                index=(fuse_opts.index(pobj.designated_fuse_key)
                       if pobj.designated_fuse_key in fuse_opts else 0),
                format_func=lambda k: pobj.element(k).label,
                key=f"fbx_fuse_{pk}")
        else:
            new_fuse = ""
        grades = [g.value for g in EvidenceGrade]
        sevs = ["S1", "S2", "S3"]
        edited = st.data_editor(
            [{"key": e.key, "label": e.label, "FoS": e.fos,
              "grade": e.grade.value,
              "severity": e.severity.value if e.severity else "S2",
              "cost_usd": e.replace_cost_usd, "downtime_days": e.downtime_days,
              "age_days": e.age_days, "note": e.note}
             for e in pobj.elements],
            column_config={
                "grade": st.column_config.SelectboxColumn(options=grades),
                "severity": st.column_config.SelectboxColumn(options=sevs),
            },
            num_rows="dynamic", use_container_width=True, hide_index=True,
            key=f"fbx_editor_{pk}")
        if st.button("💾 Save path", key=f"fbx_save_{pk}", type="primary"):
            try:
                els = []
                for r in edited:
                    if not r.get("key") or r.get("FoS") in (None, ""):
                        continue
                    els.append(fb.PathElement(
                        key=str(r["key"]), label=str(r.get("label") or r["key"]),
                        fos=float(r["FoS"]),
                        grade=EvidenceGrade(r.get("grade", "estimate")),
                        severity=fb.Severity(r.get("severity", "S2")),
                        replace_cost_usd=float(r.get("cost_usd") or 0),
                        downtime_days=float(r.get("downtime_days") or 0),
                        age_days=float(r.get("age_days") or 0),
                        note=str(r.get("note") or "")).as_dict())
                d = ss.fusebox_paths[pd_idx]
                d["elements"] = [fb.PathElement.from_dict(x).as_dict()
                                 for x in els]
                d["designated_fuse_key"] = new_fuse
                if charter:
                    st.info("Note: the sealed charter designates fuses by "
                            "key — if you changed the designation, re-seal "
                            "so the charter and the deck agree.")
                st.success("Saved. The pecking order recomputed live.")
                st.rerun()
            except (ValueError, KeyError) as err:
                st.error(f"Not saved: {err}")

    # ------------------------------------------------------------ fixes
    with tab_fix:
        st.caption("For every rival that threatens the designated fuse, "
                   "three levers solved EXACTLY from the pairwise normal "
                   "formula — soften the fuse (floored at FoS "
                   f"{fb.MIN_FUSE_FOS:.2f}: a softer fuse pops in normal "
                   "driving), stiffen the rival, or **sharpen the rival's "
                   "evidence grade** — because half of most coin flips is "
                   "a wide band, not weak metal, and a strain-gauge pull "
                   "test is cheaper than a re-machine.")
        for p in paths:
            a = amap[p.key]
            if a.verdict.value == "FUSED":
                continue
            pres = fb.prescribe(p, conf)
            if not pres and p.designated_fuse_key:
                continue
            st.markdown(f"**{p.label}** — {a.verdict.value}")
            if not p.designated_fuse_key:
                st.warning("No designated fuse — designate one in the "
                           "editor before fixes can be prescribed.")
                continue
            for pr in pres:
                el = p.element(pr.rival_key)
                st.markdown(
                    f"- vs **{el.label}** — fuse currently wins the "
                    f"pairwise race {pr.pair_p_now:.0%} of the time "
                    f"(charter asks {conf:.0%}):")
                st.markdown(f"    - (a) {pr.lower_fuse_note}")
                st.markdown(f"    - (b) {pr.raise_rival_note}")
                st.markdown(f"    - (c) {pr.sharpen_note}")
            st.divider()

    # ------------------------------------------------------------ charter
    with tab_seal:
        if charter is None:
            st.caption("Seal the team's chosen victims: one confidence for "
                       "the car, one forbidden-first tolerance, and the "
                       "designated fuse per path — sha256-sealed so what "
                       "counts as AS-DESIGNED was decided before anything "
                       "broke.")
            c1, c2 = st.columns(2)
            conf_in = c1.slider("Charter confidence — P(designated fuse "
                                "first) required for FUSED",
                                0.60, 0.99, fb.DEFAULT_CONFIDENCE, 0.01,
                                key="fbx_conf")
            forb_in = c2.select_slider(
                "Forbidden-first tolerance (max P on any S3 element)",
                options=[0.001, 0.005, 0.01, 0.02, 0.05],
                value=fb.DEFAULT_FORBIDDEN_P, key="fbx_forb")
            note = st.text_input("Charter note (who agreed, where)",
                                 key="fbx_note")
            st.write("Designations to be sealed: " + " · ".join(
                f"{p.label} → "
                f"{(p.element(p.designated_fuse_key).label if p.element(p.designated_fuse_key) else '—')}"
                for p in paths))
            if st.button("🔏 Seal the Fuse Charter", type="primary",
                         key="fbx_seal"):
                ss.fusebox_charter = fb.create_charter(
                    {p.key: p.designated_fuse_key for p in paths},
                    conf_in, float(forb_in), note)
                st.rerun()
        else:
            intact = fb.charter_intact(charter)
            if intact:
                st.success(
                    f"Charter sealed {charter['created_utc']} · confidence "
                    f"{charter['confidence']:.0%} · forbidden ≤ "
                    f"{charter['forbidden_p']:.0%} · sha256 "
                    f"`{charter['seal'][:16]}…`")
            else:
                st.error("⚠️ CHARTER SEAL BROKEN — a sealed field was "
                         "edited. This charter refuses to judge; re-seal "
                         "a new one.")
            st.markdown("**Judge an incident** — something broke. What, "
                        "and was it the plan?")
            jp = st.selectbox("Path where the failure happened",
                              [p.key for p in paths],
                              format_func=lambda k: next(
                                  pp.label for pp in paths if pp.key == k),
                              key="fbx_j_path")
            pobj = next(pp for pp in paths if pp.key == jp)
            if pobj.elements:
                jk = st.selectbox("What failed FIRST",
                                  [e.key for e in pobj.elements],
                                  format_func=lambda k: pobj.element(k).label,
                                  key="fbx_j_el")
                jn = st.text_input("Event note (session, load if known)",
                                   key="fbx_j_note")
                if st.button("⚖️ Judge", key="fbx_judge"):
                    try:
                        v, msg = fb.judge_incident(charter, pobj, jk, jn)
                        {"AS-DESIGNED": st.success,
                         "SURPRISE": st.warning,
                         "BREACH": st.error}[v.value](f"**{v.value}** — {msg}")
                    except ValueError as err:
                        st.error(str(err))
            else:
                st.info("This path has no elements to judge.")
            if st.button("🗑 Discard charter (start over)", key="fbx_drop"):
                ss.fusebox_charter = None
                st.rerun()
