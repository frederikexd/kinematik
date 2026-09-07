# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#
#  ui/express_lane.py — ⚡ "Short for time?" on the briefing screen
# ============================================================================
"""
The second door onto the landing page.

The questionnaire teaches; this does not. Two sentences, a data drop, one
button, one ZIP. It sits ABOVE the questionnaire in a collapsed expander so
it never competes with onboarding for a first-time member's attention — but a
member at 01:40 the night before a design review finds it in one click.

All parsing, sniffing, solving and bundling lives in suspension/express.py
(headless, self-tested, no Streamlit). This module only collects input, calls
`run_express`, and draws the receipts (ui/__init__.py rules).

Session keys used:
    kk_express_run     the last ExpressRun (downloads never re-solve)
    kk_express_zip     the last bundle bytes
    kk_express_tools   tool ids the grammar recognised, for the shell to
                       optionally seed a briefing from
"""

from __future__ import annotations


_PLACEHOLDER = (
    "Our 245 kg car with a cg height of 290 mm understeers on the skidpad "
    "at 1.4 lateral g and I think bump steer is the cause. Here's yesterday's "
    "run log — I need the roll numbers and a report before the review."
)

_UPLOAD_TYPES = ["csv", "tsv", "txt", "dat", "log", "json"]


def render(*, key_prefix: str = "kk_express", on_complete=None,
           hardpoints=None, expanded: bool = False) -> bool:
    """Draw the express lane. Returns True if a bundle exists to download.

    `on_complete(run)` is called once, right after a successful run, so the
    shell can seed the normal briefing from the tools the grammar recognised
    without this module having to import streamlit_app.
    """
    import streamlit as st

    ss = st.session_state
    with st.expander("⚡ **Short for time?** Two sentences and your data — "
                     "we run it and hand you the files", expanded=expanded):
        st.caption(
            "Skips the questionnaire, not the engineering. Tell us what you "
            "want and drop whatever data you have; a keyword grammar reads "
            "the sentence, a column sniffer reads the files, and the same "
            "solvers the tabs use produce a ZIP of reports, CSVs and a "
            "manifest. **No language model anywhere in it** — every word the "
            "grammar didn't understand is printed in the bundle, and the "
            "same request always produces a byte-identical ZIP."
        )

        text = st.text_area(
            "What do you need? (two sentences is plenty)",
            value="", height=90, placeholder=_PLACEHOLDER,
            key=f"{key_prefix}_text",
            help="Numbers get bound to the parameter word next to them: "
                 "'245 kg car', 'cg 290 mm', '62% front bias'. Anything you "
                 "don't give falls back to a declared default, and the "
                 "bundle prints which is which.")

        ups = st.file_uploader(
            "Your data (optional — CSV/TSV logs, or a hardpoint JSON)",
            type=_UPLOAD_TYPES, accept_multiple_files=True,
            key=f"{key_prefix}_files",
            help="Column names are matched to canonical channels by a "
                 "synonym table; scale is inferred from the numbers and "
                 "disclosed. Unmatched columns are listed, never silently "
                 "dropped.")

        c1, c2 = st.columns([1.5, 1.0])
        go = c1.button("⚡ Run it and build my files", type="primary",
                       use_container_width=True, key=f"{key_prefix}_go")
        if c2.button("Clear", use_container_width=True,
                     key=f"{key_prefix}_clear"):
            for k in ("kk_express_run", "kk_express_zip", "kk_express_tools"):
                ss.pop(k, None)

        if go:
            _run(st, ss, text, ups, hardpoints, on_complete)

        run = ss.get("kk_express_run")
        if run is None:
            return False
        _draw(st, ss, run)
        return True


# --------------------------------------------------------------------------- #
#  the run
# --------------------------------------------------------------------------- #
def _run(st, ss, text, ups, hardpoints, on_complete, budget_s=None):
    from suspension import express as ex

    files = []
    for f in (ups or []):
        try:
            files.append((f.name, f.getvalue()))
        except Exception as err:                             # noqa: BLE001
            st.warning(f"Couldn't read {getattr(f, 'name', 'a file')}: {err}")
    #  Keep the bytes so a re-run at a bigger budget does not silently drop
    #  the upload — the first version of the re-run button did exactly that.
    ss["kk_express_payload"] = files
    ss["kk_express_hp"] = hardpoints

    if not (text or "").strip() and not files:
        st.info("Type a sentence, drop a file, or both. With neither there's "
                "nothing to run — though even an empty request still gets "
                "you the geometry baseline if you insist.")

    status = st.status("Running…", expanded=True)
    try:
        run = ex.run_express(text, files, hardpoints=hardpoints,
                             budget_s=budget_s,
                             progress=lambda s: status.write(s))
        blob = ex.bundle_zip(run)
    except Exception as err:                                 # noqa: BLE001
        status.update(state="error")
        st.error(f"The express lane itself failed: {err}. That's a bug — "
                 "individual jobs are supposed to fail into the bundle, not "
                 "take the run down with them.")
        return

    status.update(
        label=f"Done in {run.elapsed_s:.1f} s — {len(run.artifacts) + 2} files",
        state="complete", expanded=False)
    ss["kk_express_run"] = run
    ss["kk_express_zip"] = blob
    ss["kk_express_tools"] = list(run.ask.tools)
    if on_complete is not None:
        try:
            on_complete(run)
        except Exception:                                    # noqa: BLE001
            pass          # the bundle is the product; seeding is a courtesy


def _rerun_bigger(st, ss, text, budget_s):
    """Re-run the same request and the same bytes at a larger budget."""
    from suspension import express as ex

    files = ss.get("kk_express_payload") or []
    status = st.status(f"Re-running with a {budget_s:g} s budget…",
                       expanded=True)
    try:
        run = ex.run_express(text, files, hardpoints=ss.get("kk_express_hp"),
                             budget_s=budget_s,
                             progress=lambda s: status.write(s))
        ss["kk_express_zip"] = ex.bundle_zip(run)
        ss["kk_express_run"] = run
        status.update(state="complete", expanded=False,
                      label=f"Done in {run.elapsed_s:.1f} s")
    except Exception as err:                                 # noqa: BLE001
        status.update(state="error")
        st.error(f"Re-run failed: {err}")


# --------------------------------------------------------------------------- #
#  the receipts
# --------------------------------------------------------------------------- #
def _draw(st, ss, run):
    blob = ss.get("kk_express_zip")
    ask, db = run.ask, run.data

    n_ok, n_skip, n_fail = len(run.ran), len(run.skipped), len(run.failed)
    n_def = len(run.deferred)
    icon = "🔴" if n_fail else ("🟡" if (n_skip or n_def) else "🟢")
    st.markdown(f"### {icon} {n_ok} jobs ran · {n_def} deferred · "
                f"{n_skip} skipped · {n_fail} failed")
    st.caption(f"Time budget {ask.budget_s:g} s"
               + (f" (from '{ask.budget_source}')" if ask.budget_source
                  else " — the lane default")
               + f" · actual {run.elapsed_s:.1f} s")

    if blob:
        st.download_button(
            f"⬇️ Your bundle ({len(run.artifacts) + 2} files, "
            f"{len(blob) / 1024:.0f} kB)",
            blob, "kinematik_express.zip", "application/zip",
            type="primary", use_container_width=True,
            key="kk_express_dl")
        st.caption("Start with `README.md` — it names every assumption the "
                   "run made and which tab to open to take each file past "
                   "screening fidelity.")

    with st.expander("🧾 What the grammar understood — and what it didn't",
                     expanded=bool(ask.ignored)):
        for c in ask.consumed:
            st.markdown(f"- ✅ {c}")
        for a in ask.assumptions:
            st.markdown(f"- ➖ assumed: {a}")
        if ask.ignored:
            st.markdown("- 🕳️ not understood (a grammar, not a language "
                        f"model): {', '.join(ask.ignored)}")
        if not (ask.consumed or ask.assumptions):
            st.markdown("- (nothing typed — the data drove the whole plan)")

    if db.files:
        with st.expander(f"📥 What the sniffer made of your "
                         f"{len(db.files)} file(s)", expanded=False):
            for r in db.receipts:
                st.markdown(f"- {r}")
            if db.channels:
                st.table({
                    "channel": [c.label for c in db.channels.values()],
                    "from column": [c.source_column
                                    for c in db.channels.values()],
                    "unit": [c.unit for c in db.channels.values()],
                    "scale decision": [c.scale_note or "—"
                                       for c in db.channels.values()],
                    "flags": ["; ".join(c.flags) or "—"
                              for c in db.channels.values()],
                })
            if db.unmatched:
                st.caption("Columns not recognised: "
                           + ", ".join(sorted(set(db.unmatched))[:40]))

    if run.deferred:
        from suspension.express import JOBS as _JOBS
        with st.expander(f"⏳ Deferred — over the time budget "
                         f"({len(run.deferred)})", expanded=True):
            for jid, reason in run.deferred:
                job = _JOBS.get(jid)
                title = job.title if job else jid
                cost = f" (~{job.cost_s:g} s)" if job else ""
                st.markdown(f"- **{title}**{cost} — {reason}")
            c1, c2 = st.columns(2)
            if c1.button("⏱️ Re-run with a 5-minute budget",
                         use_container_width=True, key="kk_express_more"):
                _rerun_bigger(st, ss, ask.text, 300.0)
                st.rerun()
            c2.caption("Or add a duration to your sentence — 'I have five "
                       "minutes' — and the lane prices it in next time.")

    if run.skipped:
        with st.expander(f"⏭️ Skipped — and exactly why ({len(run.skipped)})",
                         expanded=False):
            from suspension.express import JOBS
            for jid, reason in run.skipped:
                title = JOBS[jid].title if jid in JOBS else jid
                st.markdown(f"- **{title}** — {reason}")
            st.caption("Each of those is one column or one number away from "
                       "running. Nothing was dropped silently.")

    for jid, err in run.failed:
        st.error(f"`{jid}` failed: {err} — the rest of the bundle is "
                 f"unaffected, and the traceback is in `_failed/{jid}.md`.")

    for w in db.warnings:
        st.warning(w)

    st.caption("This is a screening artifact at declared coarse fidelity. It "
               "is a very good place to start a design review and a poor "
               "place to stop — the tabs are still where the engineering "
               "happens.")
