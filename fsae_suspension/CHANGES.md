# Documentation PDFs — capture the graphs and the results

Drop these five files over your tree (root `streamlit_app.py`, root
`requirements.txt`, `suspension/project.py`, new `suspension/report_figures.py`,
new `tests/test_report_figures.py`). `EXAMPLE_kinematics_report.pdf` is the new
output for the exact kinematics session in your screenshot.

## Why the PDF was four sentences

Two separate causes, not one.

**1. No numbers — the capture layer was watching the wrong function.**
`_ax_wrap_metric` patches Streamlit's `st.metric`. But KinematiK almost never
calls it: the headline cards go through your own `metric()` helper, which
returns HTML for `st.markdown`. That's **503 call sites versus 2**. The capture
layer was working perfectly and seeing essentially nothing, which is why every
report came out with no figures in it — not just Kinematics.

Fixed by capturing inside `metric()` itself, after unit conversion, so all 503
sites are covered at once and the report records the number in the unit system
the member had selected. The card's status class (`good`/`warn`/`bad`) rides
along, so the report shows the same judgement the UI showed.

**2. No graphs — deliberate, and reversible.** `capture_artifact` said "the
pixels themselves aren't captured — a Markdown report can't hold them." True of
Markdown, not of the PDF. Now the *spec* is captured (cheap, always) and
rasterization happens once, lazily, when someone clicks Generate PDF.

## What changed

**`suspension/report_figures.py`** (new) — turns a captured figure spec into a
print-ready PNG. Kaleido first when it genuinely works, matplotlib otherwise.

Kaleido v1+ shells out to headless Chrome, which isn't on Streamlit Cloud — so
it's probed **once** and cached, rather than attempting a browser launch per
chart on every deployment where it can never succeed. matplotlib is a pure
wheel with no system package, which is why it's the workhorse. Neither present?
Charts degrade to today's text line. Never a blank box.

Also re-themes for paper: your dark `PLOT_LAYOUT` prints as a black rectangle
with invisible labels. Series colours are kept but darkened to ~4.5:1 contrast
on white, so cyan stays cyan and stops disappearing.

**`suspension/project.py`** — `render_pdf(md, path, figures=None)`. Optional
third argument; all 24 existing two-arg call sites work untouched. A line of
`![Caption](kinematik-fig:REF)` becomes the image, captioned, sized to the text
column, `KeepTogether` so captions don't orphan.

**`streamlit_app.py`** — capture in `metric()`; store specs in
`capture_artifact`; results emitted as a **table** rather than bullets; new
`collect_report_figures()`; wired into all three export paths (feature,
subsystem, Integration Document).

## Three bugs found on the way

- **plotly ≥6 broke figure capture silently.** `to_dict()` no longer returns
  lists for numpy-backed traces — it returns `{"dtype": "f8", "bdata": "<base64>"}`.
  Anything walking the spec gets a dict where it expects numbers. Since every
  sweep in the app is numpy-backed, that was *every* chart. `_decode_bdata`
  handles it; there's a regression test.
- **Table cells never ran through the Markdown converter.** They were passed to
  `Table` as bare strings, so `**-1.50 °**` printed its asterisks and long cells
  ran off the page instead of wrapping. Invisible while reports were bullet
  lists; the results table puts bold in every row. Cells are now Paragraphs.
- **The `■` at the start of every heading was tofu.** Helvetica has no emoji
  glyph. Now: register DejaVu (bundled with matplotlib, so guaranteed present
  wherever charts can render), fold meaningful emoji to text-presentation
  equivalents, drop decorative ones. `✅`/`❌` become `✓`/`✗` rather than being
  deleted — dropping them would make "in band" and "out of band" read
  identically. `°` `·` `⚠` `✓` now render properly.

## Cost

Per rerun: one dict copy per chart, series decimated to 2000 points. No image
encoding. Rasterization only on the export click (~1s for four charts, behind a
spinner). Caps: 40 metrics, 24 verdicts, 24 artifacts per feature, unchanged.

## Tests

`tests/test_report_figures.py` — 20 tests: capture, base64 decode, decimation,
picklability, print theming, contrast, rasterization, 3-D declining cleanly,
PDF embedding, missing-figure fallback, legacy two-arg compatibility, ragged
tables, glyph handling. Existing `test_project.py` and `test_mission_briefing.py`
(49 tests) still pass.

## Two things to know

- **`matplotlib>=3.7` added to `requirements.txt`.** Without it charts fall back
  to text, exactly as before, so it fails soft.
- **The Integration Ledger persists Markdown, not pixels.** A committed feature
  keeps its image *references*; the PNGs resolve from the live session. Export
  the Integration Document in a session where the features have been opened and
  the charts appear. In a fresh session they render as
  `[figure not exported: <name>]` rather than vanishing. If you want figures to
  survive a restart, the change is to stash the PNGs in the ledger entry at
  commit time — worth doing, but it's a storage-budget decision for Supabase, so
  I left it out rather than guess.

## Also worth a look

`suspension/streamlit_app.py` and root `project.py` are stale near-copies your
own August audit flagged for quarantine, and they carry their own `render_pdf`.
I patched only the live files (root `streamlit_app.py`, `suspension/project.py`).
Worth deleting the copies before someone patches the wrong one.
