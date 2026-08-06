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

---

## Second pass — verdict chips (same defect class)

`st.warning/error/success` are captured (292 call sites). But 21 findings render
as `<span class="tag warn">…</span>` through `st.markdown` and were invisible to
the capture layer — including *"linkage does not close over full travel"*, which
is exactly the kind of finding a design-review PDF must not omit.

`_ax_wrap_markdown` now wraps `st.markdown` and extracts them, guarded on a fast
`'class="tag ' in body` substring test before any regex, because `st.markdown`
runs thousands of times per rerun. Verified at 2,000 plain calls with no matches.
Chips below the existing 12-character noise floor (e.g. the `CONFIRMED` badge)
stay filtered as chrome.

---

## Third pass — Registry status board (`suspension/status_dashboard.py`)

**The bug.** A rule's `param` and a component's spec keys are typed into two
different free-text boxes, by two different people, weeks apart — then matched
with a bare case-sensitive `specs.get(param)`. Declare `Mass`, write a rule for
`mass`, and the check reports *"not declared yet"* forever with the number
sitting in the same row. The board goes amber and stays amber, which at a design
review reads as "this team didn't finish" rather than "two strings disagree
about a capital letter."

The quick-add templates made it likelier: they hard-code `Weight`, so a team
that entered `Mass` got an unresolvable check the moment they used one.

**Model fix** — `resolve_spec()` matches exact, then case/whitespace/separator
insensitive (`wall_thickness` → `Wall thickness`), then offers the closest key
as a suggestion. Three amber states are now distinguishable, which matters
because a dead end and a ten-second fix should not read alike:

| Situation | Message |
|---|---|
| Typo | *"Wieght" not declared — did you mean "Weight"?* |
| Declared, non-numeric | *"Wall thickness" is declared as "TBD" — no number to check.* |
| Genuinely absent | *"Torque" not declared yet — enter it to check.* |

**What it deliberately will NOT do.** Substring and synonym matching are
refused. `Wall thickness` does not resolve to `Min wall thickness`, and `Weight`
does not resolve to `Mass` — those are different quantities, and silently
checking the wrong number is worse than the amber it replaces. A close key is
offered as a *suggestion*, never used as a match. Two tests pin this.

**UI fix** — the rule Parameter is now a picker of the keys actually declared on
that component, with a free-text escape (`➕ other`) for rules written before the
number exists. The mismatch mostly can't be created by hand any more.

**Tests** — `tests/test_status_keys.py`, 15 tests. Two of them failed on first
run and both were *my tests* being wrong, not the code: I had asserted that a
near-miss returns nothing (it correctly returns a suggestion), and that
`Weight`/`Mass` should unify (it correctly refuses). Corrected to pin the real
contract.

See `TRIAGE.md` for the remaining cross-cutting consumers.

---

## Fourth pass — the Integration Document was never saved

**This is the most serious one found so far, and it was reporting success.**

`_persist_doc_ledger()` does:

```python
_s.integration_document = _led      # set an attribute
return save_store(_s)               # -> store.save() -> True
```

`ProjectStore._payload()` never listed `integration_document`. So the attribute
went nowhere, `save()` returned True, `_persist_doc_ledger()` returned True, and
the app told the member:

> "Kinematics committed to the Integration Document. See the full combined
> document in the Integration tab."

The commit was gone on restart, and no teammate ever saw it. The honest failure
branch — *"committed for this session, but persisting team-wide failed (backend
offline)"* — could never fire, because nothing had failed. A season-long
cross-team deliverable was resetting every restart while claiming otherwise.

Reproduced before fixing:

```
store.save() reported: True      <- what _persist_doc_ledger returns
after reload, integration_document = None
keys on disk: [board, cad_files, decisions, ev_excel_params, geometry,
               harness, ledger, notes, reports, season, target_mass_kg,
               team_name, updated, weights]        <- no integration_document
```

Fixed by listing it in `_payload()` and reading it back in `_apply()`.

### Second bug in the same file: restoring a backup deleted your reports

`as_json()` was a *second*, hand-maintained field list. It had drifted — it
never learned about `reports`, `ev_excel_params` or `ledger` after those were
added to `_payload()`. And `apply_project_bundle()` feeds `as_json()`'s output
straight back through `_apply()`, which did:

```python
self.reports = _deserialize_reports_safe(d.get("reports", []))
```

Absent key → `[]` → **every stamped report deleted on restoring your own backup.**

Two fixes:

- `as_json()` now delegates to `_payload()` (dropping only `updated`, the
  optimistic-locking baseline). One field list instead of two, so a field added
  to persistence is in the export by construction. A test asserts the two shapes
  are equal, so they cannot drift again.
- `_apply()` guards newer fields on **key presence**, not `.get(k, default)`.
  An absent key means "this payload doesn't carry the field"; an empty list
  means "there genuinely are none." Collapsing those two is what caused the
  data loss. An explicit `[]` still clears, so a genuine wipe stays possible.

**Tests** — `tests/test_store_persistence.py`, 11 tests covering both bugs,
the absent-vs-empty distinction in both directions, and a round-trip of the
ordinary fields.

### Now-more-visible known gap

With commits finally surviving restart, the figure caveat from pass one matters
more: the ledger persists Markdown with image *references*, and the PNGs resolve
from the live session. Open the Integration Document in a fresh session and its
charts render as `[figure not exported: <name>]` rather than images. Honest, but
worth closing — it is a storage-budget decision (roughly 60 KB per chart), so
it is still yours to make rather than mine to guess.

---

## Fifth pass — Handover and Analytics (the last two consumers)

### Handover: two producers with no consumer, and one fabricated number

**Stamped reports were never in the handover.** `ProjectStore.reports` carries
signed-off calculation reports with content hashes — arguably the most valuable
thing this tool produces for next year — and `build_handover_markdown()` never
mentioned that any existed. Now a table: report, team, part, date, signed-off,
hash.

**Integration Document coverage was never in the handover.** Now that commits
actually persist (pass four), the handover lists which features each subsystem
committed, so next year knows the combined deliverable exists.

**Unavailable geometry printed as a confident 0.00.** The caller's `_gf()`
helper defaulted to `0.0` on *any* failure — unconverged solve, exotic topology,
a shadowed module. So a handover could state `scrub_radius_mm: 0.00`: a
plausible, checkable-looking number that nobody measured, in the one document
whose whole purpose is to be trusted by people who cannot ask you about it.

`_gf()` now returns `None` on failure and the builder prints *"not available —
re-export with a converged solve to capture this"*. A test pins that a genuine
`0.00` (static toe is often exactly zero) still prints as `0.00`, because the
fix must not make a real zero unsayable.

**The handover PDF had no charts.** My own miss from pass one — I wired the
feature, subsystem and Integration exports to `collect_report_figures()` and
missed the fourth. The document that outlives everyone who wrote it was the only
PDF still shipping without its figures. Wired.

### Analytics: the funnel said your features were dead

`tab_open` is automatic for all 40 features via `_TabOpenProxy`. `engage` and
`complete` were not — **16 and 19 hand-placed call sites for 40 features**. So
most of the app reported as opened-but-never-used. Acting on that reading means
cutting exactly the features the team relies on.

**The tempting fix would have been wrong.** Firing engagement from the
result-capture layer looks obvious and inflates the funnel in precisely the way
`_TabOpenProxy`'s own comment warns about: every tab body executes on every
script-run, so captures fire on a plain render too. That manufactures traffic
for tabs nobody touched — worse than no data, because it looks real.

What is honest is a **widget value changing between runs**. Nothing but a person
moves a slider. `_ax_wrap_input` wraps the twelve value-returning widgets: first
render sets a baseline and emits nothing, a later different value is an
interaction, attributed to the tab actually rendering it. One dict lookup per
widget.

Completion then uses the guard the analytics API already provided for exactly
this case — `auto_complete(..., require_engaged=True)` from `capture_artifact`.
A chart produced by a feature the user genuinely interacted with is a completed
workflow; one produced by a background render is not.

Verified against the real extracted source:

```
run 1: first render, baseline          -> no events
run 2: same value, user did nothing    -> no events
run 3: user moves the slider           -> engage
       a chart then renders            -> complete
tab the user never opened              -> silent (asserted)
```

**Tests** — `tests/test_handover_coverage.py`, 8 tests.

---

## Sixth pass — every feature documentable, verified feature by feature

`render_feature_documentation()` has **no call site in the app**. It is appended
centrally by `_TabOpenProxy.__exit__`, so every feature gets the panel with no
per-tab edits — good design, but it means one hard-coded set decides which
features can be documented at all, and nothing fails if that set is wrong.

**It was wrong.** The skip set had grown to seven entries, three of which are
real analysis tabs:

| Tab | Was | Now |
|---|---|---|
| `registry` | skipped | documentable — the status board produces rule verdicts |
| `model3d` | skipped | documentable — geometry output |
| `weight` | skipped | documentable — the weight budget is core analysis |
| `docs`, `integration`, `analytics`, `notes` | skipped | still skipped, each with a written reason |

The set is now `_DOC_PANEL_SKIP`, a dict mapping tab to *why*. A bare set drifts;
a set that has to justify itself does not.

### A missing subsystem mapping, caught by the new test

`daq` (Data Acquisition) was in `_TAB_META` with **no `_FEATURE_SUBSYS` entry**.
It still committed, but `build_integration_document()` falls back to the
"integration" bucket — so the DAQ plan filed itself under Integration instead of
with the electrics work, in the one document a judge reads end to end. Mapped to
`electrics`.

### Glyph handling now reads the font instead of guessing

The emoji stripper enumerated Unicode ranges that "look like emoji". That is a
moving target, and it leaked: ⛓ (U+26D3, Fusebox) sits in a block that also holds
⚠ and ℹ — symbols the report needs — so it survived and rendered as a tofu box in
every Fusebox heading.

`_font_coverage()` now asks the embedded font which codepoints it owns and drops
the rest. It intersects **regular and bold**, because headings are bold and the
bold face carries ~20 fewer glyphs, so a character present only in regular would
still tofu in every heading. Variation selectors and ZWJ are dropped
unconditionally — some fonts list them in their cmap, which left an invisible
ghost character where a stripped emoji had been (`⛓️ Fusebox` → `" Fusebox"`).

Result across all 40 headings: **40/40 clean**, was 39/40 with one silent ghost.

### Verified by driving the real pipeline, not by inspection

`tests/test_doc_coverage.py` (10 tests) asserts statically that every registered
feature has a label, a subsystem, a tab container, and is either documentable or
skipped with a reason — so a new feature cannot silently miss the panel.

Separately, the actual capture → markdown → PDF pipeline was executed once per
feature, using the functions extracted from the live monolith:

```
documentable features: 36 / 36
explicitly skipped:    4 -> ['analytics', 'docs', 'integration', 'notes']
FAILURES: none
```

Each of the 36 produced a results table, a verdict, an embedded figure, and a
PDF over 20 KB — the same shape as the Kinematics report.
`EXAMPLE_weight_report.pdf` is one of the three newly-enabled features, for
side-by-side comparison with `EXAMPLE_kinematics_report.pdf`.

## Running tally — cross-cutting consumers audited

| Consumer | Status |
|---|---|
| Metric capture -> reports | fixed — 503 call sites were invisible |
| Verdict capture -> reports | fixed — 21 chips were invisible |
| Registry -> status board | fixed — spec/rule key matching |
| Integration Ledger -> persistence | fixed — never saved, claimed success |
| Handover | fixed — 2 unread producers, 1 fabricated value, missing figures |
| Analytics | fixed — engage/complete covered 16-19 of 40 features |
| Doc panel -> features | fixed — 3 analysis tabs excluded, 1 unmapped subsystem |

Every one was the same shape: a consumer silently seeing a subset of its
producers, with no exception and no log — which is why 2,659 tests never caught
any of them.

### The pattern, for next time

When you add a feature, ask what reads its output, and check that reader sees
*every* way the feature can produce it. The specific traps found here, ranked by
how often they recurred:

1. A wrapper watching the framework's function while the app uses its own helper.
2. Two hand-maintained lists that drift (`as_json` vs `_payload`; ranges vs the font's cmap).
3. `.get(key, default)` where an absent key and an empty value mean different things.
4. A failure coerced to a plausible default (`0.0`) instead of to "unknown".
5. Setting an attribute nothing serializes, then reporting the save succeeded.
6. A hard-coded allow/deny set that nothing validates against the real feature list.

---

## Ninth pass — tables carried their SHAPE, not their values

Reported against a real DAQ export. The whole "charts & tables" section read:

```
• 4 rows x 7 cols · Message, ID, DLC, Rate (Hz), Bits, …
• 40 rows x 7 cols · Message, ID, DLC, Rate (Hz), Bits, …
• 12 rows x 7 cols · Message, ID, DLC, Rate (Hz), Bits, …
```

Pass one taught the pipeline to embed **charts**, and stopped there. Tables
still went through `_ax_table_summary`, which describes a table without ever
reading it. Data Acquisition is the worst possible case for that: it draws no
charts at all, so every one of its results is a table, and its report was a
list of dimensions.

**`_ax_table_rows()`** is the table analogue of `report_figures.compact_spec`:
capture the answer, not a description of it. Handles what the app actually
passes to `st.dataframe` / `st.table` — pandas DataFrames (duck-typed, so
pandas is never imported for this), list-of-dicts, dict-of-lists,
list-of-lists, flat lists. Capped at 60 rows x 12 columns, and **truncation is
always stated** ("Showing the first 60 of 300 rows"), because a table silently
showing a fifth of itself is worse than one that admits it.

The DAQ report now carries the full 40-message CAN breakdown — ID, DLC, rate,
bits, load, producer, per message. See `EXAMPLE_daq_report.pdf`.

### Three more faults visible in the same export

**`###` printed literally.** Banners are authored as Markdown for the screen,
so their text arrives carrying `###` and `**`. The PDF showed
`### BLOCKED — 11 hard failure(s)`. Stripped in `capture_verdict`.

**Two contradictory verdicts on one plan.** The export carried both
"BLOCKED — 11 hard failure(s)" and "BLOCKED — 2 hard failure(s)". A rolling
summary banner changes its numbers as the plan changes, and exact-text dedup
kept every historical version. Verdicts now dedupe on the text with digits
masked, so a re-count replaces the old figure instead of accumulating beside it.

**My own demo notice was captured as an engineering finding.** The "Sample plan
loaded" banner I added last pass used `st.warning`, which the alert wrappers
capture as a verdict — so it sat in the DAQ report between the aliasing failure
and the isolation failure. It is a note about the tool, not a finding about the
car. Changed to `st.caption`.

### A bug the tests found by accident

`_ax_cell` formatted floats with `",.4g"`. That renders a **115200 baud rate as
"1.152e+05"** and a **500 kbit/s bus as "5e+05"** — four significant digits is
too few for the round numbers that fill an engineering table. Integral floats
now print as integers with separators (`115,200`), and scientific notation is
reserved for values that genuinely need it.

I found this because a test assertion I wrote was wrong about the expected
output, and checking why exposed the formatter rather than the test.

**Tests** — `tests/test_table_capture.py`, 15 tests: every input shape, the
row/column caps, truncation honesty in both directions, pipes and newlines that
would otherwise split a Markdown row, NaN and None rendering blank rather than
leaking `nan`, and the float formatting above.
