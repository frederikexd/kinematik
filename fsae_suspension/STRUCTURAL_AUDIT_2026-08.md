# Structural audit — KinematiK, August 2026

Scope: whole-system structural integrity. Compilation of all 465 Python files,
full test suite (2,659 tests), the project's own ruff config, import/packaging
graph, byte-level duplicate scan, and a spot review of the structural-analysis
math.

## Verdict

The engineering core is sound. Zero syntax errors. The physics sampled
(tube section properties, Euler screening, bracket FOS mode set) is
dimensionally correct and honestly caveated as screening-level. What was
failing was the scaffolding: packaging, test hygiene, deployment config and
file discipline — the normal profile of a codebase that grew fast under one
author.

| | Before | After |
|---|---|---|
| Test collection errors | 16 | 0 |
| Test failures | 17 | 0 |
| Ruff F821 (undefined name) | 870 | 0 |
| Ruff F811 (silent redefinition) | 1 | 0 |
| Duplicate bytes on disk | 1.27 MB | 0 |

Remaining ruff findings: 107 F401 unused-imports, all auto-fixable with
`ruff check --fix`. Left alone deliberately — a mechanical 107-file diff would
bury the substantive changes in this pass. Do it as its own commit.

---

## Fixed

### 1. Deployment security — `.streamlit/config.toml`

Shipped with `enableXsrfProtection = false` and `enableCORS = false`, carried
over from a tunnel-sharing test setup, under a comment reading "Review before
a real public deploy." That review never happened. Combined with
`maxUploadSize = 200`, the deployed app accepted cross-origin state-changing
POSTs — including 200 MB uploads — with no XSRF token.

Both are now on. If tunnel sharing is needed again, override locally in
`~/.streamlit/config.toml` or via env vars for that session; never in the
committed file.

### 2. Two guaranteed runtime crashes — `ui/phantom_envelope.py`, `ui/omnicore_ui.py`

`_units` was imported inside `render()` but used inside `_show_results()` and
`_render_twin()` — different function scopes, so `NameError` for every user
reaching the Phantom Envelope clearance query or the OmniCore twin panel. Both
are live paths from `streamlit_app.py`. Fixed with function-local imports
matching the modules' existing convention.

These were the only two genuine F821s in the tree. They were invisible because
865 of the 870 F821 reports came from a single dead file (see §6), and nobody
could read the signal through that noise.

### 3. Stale test clone — `tests/tests/`

A 96-file clone of an older suite whose `conftest.py` computed the wrong root
and ran `sys.path.insert(0, "tests/")`, able to shadow the real `suspension`
package for the whole session. All 16 collection errors traced here. Ironic,
because the real `conftest.py` carries a long docstring about having fixed
exactly this class of order-dependent failure.

Moved to `_attic/tests_tests_stale_clone/`.

### 4. Packaging — `pyproject.toml`

Two gaps that made `pip install kinematik` produce a broken install, masked
because Streamlit Cloud runs from the repo root and puts cwd on `sys.path`:

- `ui/` was not in `packages`, despite `streamlit_app.py` importing from it at
  a dozen sites. Added.
- `suspension/hardpoint_import.py` does an unguarded `import coordinate_frames`
  — a root-level module that was not packaged. Added via `py-modules`.

Also reconciled the dependency manifests. `requirements.txt` listed nine
packages `pyproject.toml` — the declared source of truth — knew nothing about:
`google-api-python-client`, `google-auth`, `google-auth-oauthlib`, `gTTS`,
`piper-tts`, `vosk`, `faster-whisper`, `imageio-ffmpeg`, `xlcalculator`. They
are now `drive` and `voice` extras, rolled into `all`.

### 5. Test suite honesty

- `tests/test_report.py`: `test_export_without_creds_returns_actionable_reason`
  claimed to test the missing-credential branch but short-circuited on the
  missing-library branch wherever the optional Drive extra was absent, failing
  for the wrong reason. It now pins `libraries_present` and tests what it says.
  The mock-upload test now `importorskip`s cleanly.
- `tests/test_track_sim_export.py`: **two different tests shared the name
  `test_coverage_check_uses_the_formula_view`**, so Python's namespace silently
  discarded the first and pytest never ran it. The lost test covers the
  openpyxl cached-value trap that, by its own docstring, "has now bitten three
  separate checks in this module" — the one whose failure mode is overwriting a
  user's live formulas with static values. Renamed to
  `test_coverage_check_reads_uncached_formulas_as_populated`; it passes, so
  this was lost coverage rather than a hidden regression.

### 6. Duplicate sprawl

1.27 MB of byte-identical duplicates across 120 groups, from GUI copy
artefacts. Quarantined to `_attic/` (moved, not deleted — nothing is
unrecoverable):

- `streamlit_app (1).py` — 1.54 MB, divergent from the live 1.82 MB root file.
  This one file produced **865 of the 870 F821 lint errors**, because the
  per-file-ignore in `pyproject.toml` matches `streamlit_app.py` but not the
  parenthesised name. 99% of the lint noise came from one dead file.
- `backends_PATCHED.py` ×3 copies (root, root `(1)`, `suspension/aero/`).
  Imported by nothing; `suspension/aero/backends.py` is the live module.
- `README kopie.md`, `CHANGES_AND_DEPLOY kopie.md`.

`.gitignore` now blocks `(1)`, `kopie` and `copy` patterns, `_attic/`, and
`analytics_buffer.jsonl` (256 runtime analytics events committed by accident —
machine state, not source; no PII, session IDs only).

### 7. Shadowed import — `coordinate_frames.py:602`

`import project` resolved to the stale root-level `project.py` (41 KB) rather
than the canonical `suspension/project.py` (57 KB), purely because cwd is on
`sys.path`. The `Decision` dataclass fields still match, so no live corruption,
but the two copies have already diverged and the store this writes to lives in
the package version. Repointed to `from suspension import project`.

### 8. Documentation error — `suspension/bracket_fos.py`

Comment read "Plate bending about its strong axis: I = w·t³/12". That formula
is the *weak* axis. The math is right and conservative for the physical case
described (thin tab bending across its thickness); only the comment was wrong.
Corrected, because that is exactly the sentence that gets pasted into a design
review slide.

---

## Round two — the deferred risks, now closed

### `suspension/sim_handoff.py` rebuilt (was: does not exist)

Registered in `_SUBMODULES`, wired into `streamlit_app.py`'s lazy module map
(alias `_simh`), imported defensively by `suspension/cad_share.py`, and fully
specified by a 478-line test covering the mm unit contract, DXF
well-formedness, bolt-role classification and mesh sizing ladders. The module
file is simply not in the tree. The app boots (the loader is lazy) and fails
only when a user requests a sim handoff.

Built against that specification — 39/39 spec tests pass, and the module lints
clean. It is stdlib-only (no numpy, no Streamlit), matching the contract
`cad_share.py` relies on.

Three invariants the module exists to hold, all directly tested:

1. **Geometry is millimetres**, whatever the UI is displaying. The DXF carries
   `$INSUNITS = 4` and the manifest declares every unit it uses. Display-unit
   scaling is a silent 25.4× error to anything automated.
2. **Global mesh size follows material thinness, never hole diameter.** A hole
   drives *local* refinement only. Shrinking holes on a plate leaves more
   material, so it must never produce a finer global mesh.
3. **A missing number stays missing.** An undeclared load is `null` and marked
   `required`; it never becomes a plausible-looking guess, because a
   plausible-looking guess is the one thing nobody re-checks.

Verified beyond the test suite: the output parses in **ezdxf** (a third-party
CAD reader, not the project's own parser), with all four `KK_*` layers present,
the `KINEMATIK` APPID declared, and XDATA roles readable per entity. A 75 mm
motor register is classified `bore` and correctly given no thread, preload or
torque — the misclassification this layer exists to prevent. The app-side call
path in `streamlit_app.py` (lines 11421–11485) was replayed end to end: every
`basis` key, `pattern_phrase`, `mesh_rows` and `study.material` field the UI
formats resolves.

The `KNOWN_MISSING_SUBMODULES` registry added in round one is now empty, and
`test_known_gaps_are_still_gaps` forced it to shrink the moment the module
appeared — which is exactly the lifecycle it was built for.

### Root/package duplicate pairs — quarantined

`fullcar3d.py`, `brake_thermal.py`, `dynamics.py`, `project.py` existed at both
root and in `suspension/`, all divergent, package versions substantially
larger. Verified dead before moving: an AST scan across the whole tree found no
importer, none has a `__main__` block, none is referenced in any doc or config.
Moved to `_attic/root_module_duplicates/`.

`suspension/streamlit_app.py` (1.82 MB) is a near-copy of the root monolith and
is imported by nothing. **I quarantined it and was wrong to.** The full suite
caught it: `tests/test_run_log_ui.py` and `tests/test_brief_coverage.py`
parametrise over `_APPS = [root, package]` and assert the two copies stay in
sync — `test_both_app_copies_carry_the_same_view` exists precisely to police
this pair. My AST scan found no *importer* because the tests read the file as
source text rather than importing it, which an import graph cannot see.

Restored. The duplication is deliberate and guarded; it is a maintenance burden
but not an unmanaged risk, and it should be resolved by consolidating the two
copies, not by deleting one. **Root `streamlit_app.py` was never touched** — it
is the live Streamlit Cloud entry point.

### `ui/` duplicates — quarantined, and one of them mattered

Four more copies, found by re-running the duplicate scan after the round-one
fixes:

- `ui_arch_synth.py`, `ui_degradation.py`, `ui_worthwhile.py` — byte-identical
  to their unprefixed siblings, each self-identifying in its own header as the
  unprefixed name. Nothing imports the prefixed ones.
- `omnicore_ui.py` — **a drifted copy of `omnicore.py`**.

That last one is a correction to round one. The `_units` NameError I reported
in `ui/omnicore_ui.py` was real code, but in the *unreferenced* copy;
`streamlit_app.py` imports `ui.omnicore`, which already carried the fix. So
that half of finding §2 was never a live crash. The `ui/phantom_envelope.py`
NameError **was** live and is the one that mattered.

This is the clearest argument in the whole audit for killing duplicate files:
the two copies had drifted to opposite sides of a real bug, and nothing in the
tree indicated which one was authoritative.

All four moved to `_attic/ui_duplicates/`. Every surviving `ui/` module still
imports cleanly.

### Guardrails so this cannot recur

`pyproject.toml` now sets `norecursedirs = ["_attic", ...]` for pytest and adds
`_attic` to ruff's `extend-exclude`, so the quarantine can never re-poison a
collection run the way `tests/tests/` did.

---

### Two headless guards that could never fail

Surfaced by the same run. `tests/test_daq_plan.py` and `tests/test_fuse_test.py`
both asserted `"streamlit" not in sys.modules` against the *shared pytest
process*. That is order-dependent in the worst way: it passes only on a machine
where streamlit is not installed at all, and hard-fails the moment any earlier
test in the session imports it. On every real developer machine it was either a
no-op or a false alarm — never a test. `test_fuse_test.py` also carried
`assert "numpy" not in sys.modules or True`, which cannot fail under any
circumstance.

Both now run the check in a fresh interpreter, which answers the question
actually worth asking: does importing this module *pull streamlit in*. Verified
non-vacuous by planting a module with a top-level streamlit import and
confirming the guard fails on it.

These were pre-existing and unrelated to my changes — they only became visible
because installing streamlit into the audit environment flipped them from
silently-passing to loudly-failing.

## Round three — final cleanup

### `ui/ui_report.py` resolved

Both files self-identify in their own header as `ui/report.py`. `ui/report.py`
is a 275-line strict superset carrying the team-project save and the Drive
OAuth export; `ui_report.py` is a 155-line older cut missing both. Same
prefixed-copy pattern as the other three `ui_*` files. Quarantined the short
one.

### 28 test files were living inside the shipped package

`suspension/test_*.py`. `testpaths = ["tests"]` meant none of them ever ran, and
they were being packaged into the wheel. 23 were byte-identical to their
`tests/` counterparts. The other five needed judgement:

- **`test_report_store.py`, `test_drive_oauth.py`** — package-only, i.e. real
  coverage that had never executed. Moved into `tests/`; both pass (one Drive
  case now `importorskip`s the optional extra rather than asserting against a
  capability the environment lacks).
- **`test_invites.py`** held one test absent from `tests/`:
  `test_create_invite_allows_lead`, asserting that minting a **lead**-role
  invite succeeds. Current `auth.create_invite` explicitly refuses it, and
  `tests/test_invites.py::test_create_invite_refuses_privileged_roles_client_side`
  asserts refusal for owner/lead/admin. The package copy predates a deliberate
  privilege-escalation hardening. **Porting it would have asserted the
  vulnerability back into existence.** Not recovered — quarantined and recorded
  here.

  Worth generalising: "lost coverage" is not automatically coverage worth
  restoring. An orphaned test is a claim about intended behaviour from an
  unknown point in history, and it has to be read against current intent before
  it is trusted.

### F401 backlog cleared

90 unused imports removed via `ruff check --fix`, as its own pass. Ruff is now
fully clean across the tree. Every module still compiles and imports; full
suite green afterwards.

**Final state: 2,699 passed, 17 skipped, 0 failed. Ruff: all checks passed.**

## Round four — the missing gate

**The repo had no CI of any kind.** No `.github/`, no `.gitlab-ci.yml`, no
Makefile. That is the root cause behind most of this audit rather than another
item in it: every machine-detectable defect found here — the two `F821`
NameErrors, the `F811` shadowed test, the registered-but-absent module, the
sys.path-poisoning test clone — was sitting in tool output nobody ran.

Added `.github/workflows/ci.yml` with three jobs:

- **lint** — `ruff check .` against the project's own config. Seconds, so a
  lint failure fails the PR before anyone waits on the suite, and reads as a
  lint failure rather than being buried in test output.
- **install** — `pip install .` (not `-e`, which would put cwd back on
  `sys.path` and re-mask exactly the layering bug this catches), then imports
  all 79 public submodules **from outside the source tree**. Verified: 79/79
  clean. This is the job that would have caught `suspension/hardpoint_import.py`
  importing a root-level, unpackaged `coordinate_frames`.
- **test** — full suite with `[all]` extras installed, so optional-dependency
  tests actually execute instead of skipping. Parallelised with `-n auto`
  (the suite is ~20 min serial). Includes a guard that **fails if any test
  skips for a missing dependency** despite `[all]` being installed — that
  would mean the extras and the imports have drifted apart again, which is the
  precise failure this audit already had to fix once.

## Round five — hardening the gate itself

### Lint ratcheted onto the bug-catching half of bugbear

`select` was `["E9","F63","F7","F82","F401","F811"]`. Now also carries
`B009 B010 B011 B028 B033`, all at **zero findings**. A gate with a standing
backlog is not a gate — a backlog trains people to skim past the output, which
is precisely how two live NameErrors and a shadowed test survived here.

Real defects found and fixed by the ratchet:

- **`B011` — five `assert False, "..."` in negative-path tests.** `python -O`
  *deletes* assert statements, so under an optimised interpreter those five
  tests would pass without the expected exception ever being raised. Converted
  to `raise AssertionError(...)`.
- **`B033` — a duplicated `"control"` in the `link` vocabulary set**
  (`hardpoint_import.py`). A set dedupes, so behaviour was never wrong, but a
  duplicate in a hand-maintained alias table usually marks a slot that was
  meant to hold a *different* token. Removed and flagged in place; not guessed
  at, because inventing a suspension alias is worse than a missing one.
- **`B028`** — a degraded-import warning without `stacklevel`, blaming
  `express.py` instead of the caller.
- **`B009`/`B010`** — three constant `getattr`/`setattr` calls.

Rules left off are now documented **with reasons** in `pyproject.toml`, so the
next person does not re-derive them. Most notable: `B023` produces 33 findings
and **all 33 are false positives** — every closure (`bilerp`, `_f`, `_g`,
`fnum`) is invoked in the same iteration it is defined and none escapes into a
callback. I checked each by hand. That is a genuinely good outcome: the one
bugbear rule that catches real latent bugs found none.

`UP` would autofix 1,829 sites. A mechanical rewrite that size across a 33k-line
monolith is an unreviewable diff, which *reduces* stability. Left as an explicit
decision, not an omission.

### Deprecation warnings promoted to errors

The config carried `"default::DeprecationWarning"` under a stale TODO. Verified
against the **full** suite rather than a sample — 2,702 passed with
`error::DeprecationWarning` — then promoted, with a short justified allowlist
for third-party (Streamlit, Google) deprecations that are not ours to fix.

### A parity guard for the two app copies

`tests/test_app_copy_parity.py`. Structural rather than byte-exact, because the
copies legitimately differ in imports and paths and a byte-exact assertion
would simply get muted. It pins that both entry points expose the same
subsystem menus and the same view titles, and fails if their sizes diverge by
more than 5% — the signature of a whole feature block landing in one copy only.
It also fails with an explicit message if either file goes missing, so the next
person who reasons their way into deleting one (as I did) is told why not.

This does not fix the duplication. It makes drift loud, which is what was
missing when the two `ui/omnicore` copies ended up on opposite sides of a real
NameError with nothing in the tree saying which was authoritative.

### Security sweep — clean

Ran the bandit rule set. Everything of substance is a false positive:
`PASS = "pass"` is an enum verdict, not a credential; the flagged "SQL
injection" is an f-string of prose; the `S106` hits are dummy tokens in test
fixtures. No hardcoded secrets in the tree.

## Round six — the bracket FoS conventions

Held back through five rounds because changing safety-factor math should never
arrive unannounced. On review the two issues are **different in kind**, and only
one of them was ever a convention.

### Tear-out was simply wrong

The shear planes were measured from the hole **centre** (`2·e·t`). They run from
the **edge of the hole** to the free edge: `2·(e − d/2)·t`. Counting the
material inside the hole as load-carrying overstates the shear area by
`d/(2e − d)`.

There is no school of practice in which the hole resists its own tear-out, so
this was a defect, not a choice. On a ⌀10 hole at 8 mm edge distance — an
ordinary "moved the hole to clear a weld" tab — it was **2.67× optimistic**, and
the correction is the difference between:

| | tear-out FoS | verdict |
|---|---|---|
| before | 2.28 | **PASS** |
| after | 0.85 | **FAIL** |

A bracket that would have cleared the 1.5 gate now fails it, correctly.

Also added: if `e ≤ d/2` the hole breaks out through the free edge entirely.
That previously produced a negative-area result silently; it now raises a `FAIL`
finding and emits no tear-out mode, because a factor of safety computed on
impossible geometry is worse than none.

### Net section added, gross section kept

A bolt hole removes material from the load path. Screening on the gross section
is unconservative by exactly `w/(w−d)` — for a 30 mm tab with a ⌀8.4 hole,
**39% of the stress was invisible**.

Added as an **extra** mode rather than replacing the gross figure. `min_fos`
takes the minimum across modes, so the screen gets stricter without deleting a
number members may already be checking by hand. This is the part that was a
genuine convention call, and the conservative-by-addition form is the one that
does not require anyone to re-baseline their existing work.

The governing principle, now stated in the code: **a screening tool may be
wrong, but only in the safe direction.**

### Pinned

Four new tests fix both corrections in place, including the PASS→FAIL flip and
both invalid-geometry findings. The existing suite needed no changes — the two
tests that pin `min_fos` to a gross-section value use hole-free brackets, so
they were never asserting the unconservative behaviour.

**Final: 2,706 passed, 17 skipped, 0 failed. Ruff: all checks passed.**

## Round seven — the last two

### `UP` ratchet applied — 1,977 sites

Held back through six rounds as "an unreviewable diff". Applied on an explicit
call that the project is in beta. Almost entirely annotation modernisation:
`Optional[X]` -> `X | None` (1,095), `List[x]` -> `list[x]` (504), unquoting
deferred annotations (159), plus `datetime.UTC` and some redundant open modes.
That orphaned 148 `typing` imports, cleared in the same pass.
`requires-python` is `>=3.12`, so none of it is conditional.

Three rules held back, reasons now in `pyproject.toml`. One deserves calling
out:

**UP042 would have been a silent data bug.** It rewrites `class X(str, Enum)`
-> `StrEnum` at 31 sites. That looks cosmetic and is not: a str-Enum member
formats as `Verdict.PASS` in an f-string, a StrEnum member formats as `pass`.
Those values are serialised into manifests, reports and saved project files, so
the switch would have quietly changed persisted output and broken round-tripping
of existing saves. Ruff will not autofix it, but a bulk "modernise everything"
pass that reached for `--unsafe-fixes` would have shipped it.

UP031 (62 printf-style format strings) and UP035 (42 deprecated typing imports)
are held for the same reason in weaker form: not autofixable, and a hand rewrite
is a real chance to change output for no functional gain.

### App consolidated to a single file

Verified first that nothing but the tests needed `suspension/streamlit_app.py`:
no `[project.scripts]`, no console script, no doc or config reference. Then the
contract was retired **deliberately** — `_APPS` updated in `test_run_log_ui.py`
and `test_brief_coverage.py` *before* the file was removed, rather than deleting
it and chasing the failures. That is exactly what the round-two guard told the
next person to do, and it applied to its author.

The guard is inverted, not deleted. `test_app_copy_parity.py` is replaced by
`test_app_single_source.py`: the risk is no longer "the two copies drift" but
"someone reintroduces a copy". Given this tree had accumulated
`streamlit_app (1).py`, three copies of `backends_PATCHED.py`, four duplicated
`ui/` modules and a 96-file clone of the test suite, that is not hypothetical.
It fails on named variants and sweeps the tree for any large
`streamlit_app`-like file outside the root.

### A note on "it gets validated in Ansys anyway"

True, and a fair basis for accepting screening-level approximation — it is why
the `UP` pass went in without agonising. But it does not transfer to round six.
A screening tool that returns PASS is precisely what stops a bracket from ever
reaching Ansys. The downstream check only catches what gets sent to it, so the
screen's job is to be conservative, not accurate. That is why those two
corrections were worth doing carefully even in beta, and why they are pinned by
tests now.

**Final: 2,706 passed, 17 skipped, 0 failed. Ruff clean across E9/F/B/UP.**

## Round eight — the monolith split, started properly

`streamlit_app.py` is ~32,800 lines under a permanent `F821` lint exemption,
which means static analysis can never see it — that blind spot is where the two
live `NameError`s hid. It has to come apart.

**It does not come apart in one commit.** A big-bang split of a 33k-line file
with a lazy alias-injection layer is unreviewable, and unreviewable is exactly
how those NameErrors survived. The project's own plan agrees: `ui/__init__.py`
targets "no file over 3,000 lines within **two seasons**". So this round does
what actually moves that target — executes the first slice end to end, and
turns the procedure into something the team can repeat without me.

### First slice extracted: `ui/run_log.py`

Chosen by **seam, not size**. A free-variable analysis of each candidate block
showed the ANSYS run-log view referenced exactly two shell locals — `st` and
the aero reference area — and imported everything else itself. Its own inner
comment said it was written to be lifted out. 311 lines left the shell.

The prize is in the tests, and it is easy to miss. The old harness located the
view by line indentation, sliced the text out, rewrote its `elif` into
`if True:` and `exec`'d it against a mock — it could only ever test text whose
boundaries it had guessed. Now:

```python
from ui import run_log
run_log.render(mock_st, aero_area=1.0)
```

All 27 tests pass with only the harness changed. They now run against the
function the app actually calls. Added a guard that the body cannot creep back
in beside the delegation.

Verified the module imports headless with streamlit absent from `sys.modules`,
per the `ui/` contract.

### `docs/EXTRACTING_A_VIEW.md`

The procedure, proven on this slice: the free-variable script for picking a
candidate, the `render(st, ...)` shape, the delegation stub, the
scrape-to-call test rewrite, and the verification gate. Plus the order of
attack — views with an existing test file first, because step 3 is where the
risk is and a harness that passes before and after is what proves the move was
faithful.

It also records what not to do, each learned here: do not move and refactor in
one commit; do not leave a copy behind; do not delete a contract to make a
failure go away — retire it deliberately, first, in its own step.

**Final: 2,706 passed, 17 skipped, 0 failed. Ruff clean.**

## Round nine — the last defects

### Four finished panels the app could not open

`ui/report.py` was flagged earlier as "differs from `ui_report.py`, neither is
imported". That was half the story, and the wrong half. Re-checking it
properly turned up the real defect: **four complete panels — `arch_synth`,
`degradation`, `report`, `worthwhile`, ~36 KB, each with a working
`render()` — are not reachable from the app at all.** Nothing imports them, no
lazy loader names them, no tab menu offers them.

They are the same four whose *duplicates* were quarantined in round three.
Someone built each one, saved a copy under a `ui_` prefix, and never wired
either in. Deleting them would throw away working UI; leaving them silently is
a feature nobody can open.

`tests/test_ui_panels_reachable.py` does for the UI surface what
`KNOWN_MISSING_SUBMODULES` did for the public API: it does not permit the gap,
it makes the gap **declared**.

- a NEW orphan fails the suite
- a RESOLVED orphan also fails it, forcing the registry to shrink
- the four are parametrised against the `ui/__init__.py` contract — `render()`
  callable, no module-scope streamlit — so they stay ready to wire instead of
  rotting with no import path and no test
- the mirror case is covered too: the shell importing a `ui` module that does
  not exist

Where each belongs in which tab menu is a product decision, so it is
deliberately not guessed at.

### UP031 and UP035 cleared

Both were held back as "not autofixable, wants someone reading each one". Done:
51 percent-format sites converted — 49 by ruff's unsafe-fix pass, 2 by hand
where the format string embedded a literal `%` (strftime directives in one, an
escaped `%%` in the other) that ruff refused to touch — then 38 resulting
`.format()` calls folded into f-strings. UP035 fell out with the main pyupgrade
pass.

Both hand conversions were **checked for output equality on real values**, not
assumed, including the case ruff got right that a naive rewrite gets wrong:
`"%02x%02x%02x" % some_tuple` needs `.format(*t)`, not `.format(t)`.

`ignore` is now down to `UP042` alone, which stays for a real reason: it would
change `Verdict.PASS` to `pass` in serialised output and break round-tripping
of saved project files.

**Final: 2,713 passed, 17 skipped, 0 failed. Ruff clean on E9/F/B/UP.**

## Round ten — the four panels wired, and what that exposed

### All four now reachable

`Worthwhile once assembled?`, `Architecture synthesis`, `Transient
degradation` and `Calculation report` are in the **Integration** tab's
`feature_menu`. Not a guess at placement: each answers a whole-car question
rather than a single-subsystem one, and `worthwhile` reads the
`IntegrationLedger` directly. All four self-source from `session_state` with
all-optional arguments, so the shell hands over nothing, and each is wrapped
like the existing Verdict Center branch — a panel that fails reports ITSELF
rather than taking the Integration tab down with it.

`UNWIRED` is now empty, which is what the registry was built to force.

### My own reachability test had a hole

It asserted `"streamlit" not in mod.__dict__`. That passes trivially for the
`try: import streamlit as st / except: st = None` pattern these modules use —
the binding is `st`, not `streamlit`. **It was checking the wrong name and
could never have failed.**

Corrected to probe a fresh interpreter. All four panels then failed it — and so
did `daq_plan`, which had been violating the same rule while wired in the whole
time. Five modules moved their streamlit import inside `render()`.

Also widened the test from `UNWIRED` to **every** panel in `ui/`. Parametrising
over the registry meant the check would evaporate at exactly the moment the
registry emptied, leaving the four panels with no contract test at the point
they finally became reachable.

### A real test-isolation bug in `test_analytics.py`

`_fresh_module` reloads the analytics module but leaves the previous `_SINK`
daemon thread alive, still holding unflushed events. That stale sink writes
into the NEXT test's buffer, surfacing as a third `session_start` and an
`assert 3 == 2`. Order-dependent, so it stayed hidden until these changes
shifted import timing enough to run the modules adjacently. Fixed by draining
and stopping the outgoing sink before the reload; verified across all three
orderings that previously failed.

Worth recording how this was found: the mechanism was theorised three times and
wrong three times. Only pulling the actual assertion text made the cause
obvious. Same failure mode as the `streamlit_app.py` deletion in round two —
reasoning from a plausible model instead of reading the evidence.

**Final: 2,731 passed, 17 skipped, 0 failed. Ruff clean.**

## Final verification

Run against the **reconstituted deliverable** — your original upload, the delta
applied on top, `APPLY_REMOVALS.sh` executed — not against the working copy.
That is the artefact you actually receive, so it is the one worth testing.

| check | result |
|---|---|
| full test suite | **2,731 passed, 17 skipped, 0 failed** |
| ruff (E9/F/B/UP) | all checks passed |
| every `.py` compiles | 333 files, 0 syntax errors |
| all 79 public submodules import from OUTSIDE the tree | 79/79 clean |
| `ui/` panels reachable | 22/22, none orphaned |
| `ui/` panels headless-importable | 22/22, no violations |
| byte-identical duplicates >2 KB | **0** |

### One last defect, found by that sweep

The duplicate scan still flagged 16 groups — all documentation. Root markdown
files had been copied into `docs/history/` and `docs/usage/` when someone
organised the docs, and the originals were never removed. Plus a third copy of
`README.txt` inside the shipped package.

`README.md` links the ROOT copies by bare filename, so those are canonical and
the `docs/` copies are the strays. Verified nothing reads them first — the two
apparent hits in tests turned out to be README files inside *generated zip
bundles*, not repo docs. 18 files quarantined; `docs/history/` and `docs/usage/`
keep their three genuinely unique documents.

Same defect class as the code duplicates, and worth stating plainly: two copies
of a document drift exactly like two copies of a module, and the one that gets
updated is never reliably the one people read.

## Deployment context: Streamlit Community Cloud

Learned after the audit, and it re-orders the findings.

### The upload cap was a live outage risk

`.streamlit/config.toml` carried `maxUploadSize = 200`. Community Cloud allows
roughly **1 GB of memory for the whole app, shared across every concurrent
viewer**.

An uploaded file is held in memory, and trimesh then expands it into a mesh
several times its file size. Baseline (streamlit + numpy + scipy + trimesh +
plotly) already sits in the low hundreds of MB. So a single 200 MB STEP upload
does not merely fail for the person who uploaded it — it takes the app over its
limit and **down for everyone viewing it**, which during a design review is the
worst possible moment.

Lowered to 50 MB, which comfortably covers an FSAE chassis or upright assembly,
with the arithmetic written into the file so the next person raises it
deliberately rather than because one upload bounced.

I preserved the 200 in round one while fixing XSRF beside it. Worth noting: I
was reading that file as a security artefact and did not think about it as a
capacity one.

### The XSRF fix mattered more than I said

Community Cloud requires a **public GitHub repository**. So the config with
`enableXsrfProtection = false` was not just deployed — it was publicly readable,
on a publicly reachable app, advertising that the file uploader accepted
cross-origin POSTs. That moves it from "should fix" to "fix before the next
push".

Repo re-checked for public exposure: no committed secrets, no key-shaped
strings, `.streamlit/secrets.toml` and `kinematik_setup.json` both gitignored.
The one bandit hit is `PASS = "pass"`, an enum verdict.

### CI does not gate the deploy — branch protection does

Community Cloud redeploys on every push to the tracked branch. The pipeline in
`.github/workflows/ci.yml` runs *alongside* that deploy, not in front of it, so
a red build still ships. Point Community Cloud at a branch that only receives
merges from PRs, and mark the three CI jobs as required checks. Without that,
the pipeline is a report rather than a gate — which is the exact failure mode
this whole audit kept finding.

### Two smaller Cloud notes

- **Pin Python to 3.12** in the app's advanced settings. `runtime.txt` is
  Heroku-style and Community Cloud does not read it; `cascadio` has no 3.13
  wheel, so STEP import silently degrades on a newer interpreter.
- **The filesystem is ephemeral.** `analytics_buffer.jsonl` — the local
  fallback when the Supabase sink is unreachable — does not survive a restart
  or a wake-from-sleep, and apps sleep after 12 quiet hours. Treat buffered
  analytics as best-effort, not durable.

## Branch protection

The gap: Streamlit Community Cloud redeploys on **every push to the tracked
branch**, and CI runs alongside that deploy rather than in front of it. A red
build still ships. Every green check in this audit — 2,731 tests, ruff, the
packaging check, the reachability guards — is bypassable by one `git push`.

Branch protection is a repository *setting*, so the repo cannot switch it on
itself. What it can do is make switching it on trivial and switching it off
loud. All three now exist:

**A single required check.** The new `CI gate` job in `ci.yml` depends on
`lint`, `install` and `test`, and fails if any of them failed, was cancelled,
or was **skipped**. Protection should require this one check: future jobs come
under the gate by joining its `needs:` list, whereas requiring the three jobs
individually means the next job someone adds is silently ungated.

Its logic was exercised against every outcome shape, which caught a hole in my
first version: an upstream job that reports *nothing at all* leaves an empty
slot that matches none of the failure patterns, so the gate passed on a
pipeline that never ran. It now counts results as well as inspecting them —
verified across six cases including empty and short result sets.

**A canary.** `.github/workflows/protection-canary.yml` queries the GitHub API
every Monday and on every push, and fails if protection is missing or
incomplete — no required `CI gate`, no required reviews, force-pushes or
deletions allowed, or `strict` off. Protection disabled "just for one hotfix"
and never restored is the normal way this decays. It needs a fine-grained PAT
with Administration:read as `ADMIN_READ_TOKEN`; until that secret exists it
warns and exits 0, because an unconfigured canary must not look like a passing
one.

**A runbook.** `docs/BRANCH_PROTECTION.md` — one `gh api` command, the UI
click-path, and a table of what each setting buys. Two things in it are easy to
get wrong: protect the branch **Streamlit Cloud actually deploys from**, not
whatever is called `main`; and leave `enforce_admins` **on**, because on a small
team the admins are the main committers, so exempting them exempts almost every
push.

`.github/CODEOWNERS` is included but fully commented out — it needs real
handles, and an enabled CODEOWNERS pointing at a nonexistent user blocks every
PR. It marks the deployment surface, `bracket_fos.py`, and the guard tests
themselves: three of the sharpest findings in this audit were guards that
looked like coverage and asserted nothing, so a change that WEAKENS a check
deserves the scrutiny of a change to the code it checks.

## Still open — deliberate

Root `streamlit_app.py` and `suspension/streamlit_app.py` remain as a
guarded-but-real duplication (see round two). Consolidating them is a genuine
refactor with a test contract attached, not a cleanup, and it needs an owner and
a deadline rather than an opportunistic deletion.

---

## Recommendations, in order

1. ~~**CI gate on ruff.**~~ Done — see round four.
2. ~~**Clear the F401 backlog.**~~ Done — see round three. Still worth
   ratcheting `select` up to include `B` and `UP` as the config's own comment
   already plans; that is an opinionated change with a real diff behind it, so
   it wants a deliberate decision rather than being folded into a cleanup pass.
3. ~~**Promote `filterwarnings` to error.**~~ Done — see round five.
4. **Resolve the root/package duplication.** One canonical location per module.
   Now guarded against silent drift, but still duplicated.
5. ~~**Net-section and tear-out conventions.**~~ Done — see round six.
6. ~~**Resolve the root/package duplication.**~~ Done — see round seven.
7. **Split `streamlit_app.py`** — started, see round eight. First slice done,
   procedure documented in `docs/EXTRACTING_A_VIEW.md`. ~32,800 lines to go
   against the two-season target; this is now an owner-and-cadence problem,
   not an unknown one. 33,000 lines with an `F821` blanket exemption
   is a permanent blind spot — the exemption is load-bearing for the lazy alias
   injection, which means static analysis can never see that file. The `ui/`
   strangulation boundary described in `ui/__init__.py` is the right plan;
   it needs a deadline.
5. **Net-section and tear-out conventions in `bracket_fos.py`.** Direct stress
   uses the gross section rather than net-through-holes, and tear-out uses full
   `edge_dist` rather than `edge_dist − d/2`. Both slightly unconservative for a
   tool feeding a 1.5 FOS gate. Not wrong for screening — but make it a
   deliberate, documented decision rather than an implicit one.
