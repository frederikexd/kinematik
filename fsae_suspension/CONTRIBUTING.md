# Contributing to KinematiK — Engineering Standards

KinematiK's pitch to teams is *"garbage inputs never reach the sim."* The repo has to
hold itself to the same standard: garbage never reaches `main`. These are the rules
that keep that true. They are short on purpose.

## The non-negotiables

1. **CI is the gate.** Every push and PR runs `ruff` + the full test suite
   (`.github/workflows/ci.yml`). A red `main` is an incident: fixing it outranks all
   feature work. Nothing merges on red.

2. **No duplicate-snapshot files. Ever.** No `foo (1).py`, `foo kopie.md`,
   `foo_PATCHED.py`, no committed `__pycache__`. Git *is* the version history — a
   second copy of a file in the tree is a fork nobody will reconcile. CI rejects
   these patterns automatically. If you're mid-experiment, use a branch.

3. **Tests and code land in the same commit.** The July incident to learn from: the
   minimal-analytics tests were committed while `analytics.py` still had the old
   behaviour, so the suite sat red and nobody could tell real breakage from known
   drift. A test that describes intended-but-unlanded behaviour is `@pytest.mark.xfail`
   with a reason, or it doesn't merge.

4. **Physics changes carry evidence.** Any change to a solver, coefficient, sign
   convention, or vocabulary must state in the PR description *why the new behaviour
   is physically right* (reference, hand calc, or validated tool comparison) and add
   a regression test. The `hardpoint_import` track-rod fix is the model: British
   "track rod" = tie rod, American "track bar" = panhard — the comment in the code
   says so, and a test pins it.

5. **No secrets, no telemetry, no user data in the tree.** `secrets.toml`,
   `analytics_buffer.jsonl`, and real team project blobs are gitignored. Session
   IDs are personal data; a public repo is not a place for them.

6. **One dependency source of truth.** `pyproject.toml` owns dependencies.
   `requirements.txt` exists only because Streamlit Cloud reads it — if you touch
   one, mirror the other in the same commit.

## Where code goes

- **Physics and logic → `suspension/` (or `powertrain/`).** Pure, import-light,
  unit-testable, no Streamlit imports. This is why 1,100+ tests run headless in
  minutes — protect that property.
- **UI → `streamlit_app.py`.** Rendering, layout, session state only. If you are
  writing an equation inside `streamlit_app.py`, stop and move it to a module.
- **New tab?** Follow the decomposition plan in `docs/ROLLOUT_PLAN.md` /
  `CTO_REVIEW.md`: new tabs go in `ui/<tab>.py` from now on; the 28k-line monolith
  is frozen for additions and shrinks over time (strangler pattern).

## Workflow

```bash
pip install -e ".[dev]"     # everything, including test/lint tools
ruff check .                # must be clean
pytest tests/ -q            # must be green (≈3 min headless)
```

Branch → PR → green CI → review → squash-merge. Direct pushes to `main` are for
one-line doc fixes at most, and even then: run the suite first.

## Review checklist (what the reviewer actually checks)

- Does the physics claim have a source or a test that would fail if it were wrong?
- Did UI code leak into `suspension/`, or math into `streamlit_app.py`?
- Are new event types / DB columns reflected in the SQL under `sql/` *and* the
  CHECK-constraint vocabulary in `analytics.py`?
- Would a new FSAE team member understand the error messages this change can emit?
