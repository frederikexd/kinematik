# KinematiK — CTO Review & Engineering Roadmap

*Scope: the `fsae_suspension/` tree (the live, most complete copy of the codebase). July 2026.*

## Verdict

The physics core is genuinely strong: 92 import-light modules, a controlled event vocabulary, honest provenance tagging, lazy loading designed around a 1 GB cloud RAM budget, and — the headline — a 1,186-test suite that runs headless in under three minutes. That test suite is the most valuable asset in this repository. It is also the asset the current workflow was quietly destroying: the suite was red on `main` (4 failures), and nothing was running it automatically, so nobody could distinguish real regressions from known drift. Almost everything in this review flows from one principle: the repo must hold itself to the same standard KinematiK sells to teams — garbage inputs never reach the sim, and garbage commits never reach `main`.

## What was broken, and what I fixed

**A real product bug: British hardpoint names were refused.** `hardpoint_import.py` refused `"track rod inner"` as ambiguous because bare `track` sat in both the tie-rod vocabulary (British "track rod" = tie rod) and the panhard vocabulary (American "track bar" = panhard bar). Any UK or European team importing a hardpoint sheet with standard British terminology hit an ambiguity error on every steering point. Fixed in `_tokens()`: once the fused `trackrod` form is confirmed, the bare `track` token is discarded, so tie-rod names resolve cleanly while `"track bar"` still maps to panhard. The fix carries a comment explaining the physics of the naming collision, and the existing vocabulary test now pins it.

**Test/code drift: minimal-analytics mode was documented and tested but never landed.** `MINIMAL_ANALYTICS_DEPLOY.md` and three tests describe a mode where only `session_start`, `workflow_complete`, and `error` are written (to keep the Supabase table under ~1 MB), but `analytics.py` still wrote every event type. The suite sat red for it. I landed the documented design: a `_SAMPLE_RATES` table plus a `_sampled()` gate in `_emit`, dropping the seven high-volume event types at source exactly as the deploy doc specifies. This is also the cheapest fix to the 500 MB Supabase overage the docs describe — the drop now happens before the network, not after.

**A deprecation on the write path.** `workspace.py` stamped audit timestamps with `datetime.utcnow()`, which is scheduled for removal. Replaced with a timezone-aware call producing the identical `...Z` format. The suite is now warning-free, which matters: a clean baseline is what lets CI promote warnings to errors later.

**Repo hygiene.** Removed roughly 6 MB of committed `__pycache__` bytecode; deleted the stale snapshots `streamlit_app (1).py`, `backends_PATCHED.py` (both copies — I verified they are *older* than the live `suspension/aero/backends.py`, which already contains the FluentVerificationSolver they predate), `README kopie.md`, and `CHANGES_AND_DEPLOY kopie.md`; deleted `analytics_buffer.jsonl`, which shipped 98 KB of real user telemetry with session identifiers in a public repo — session IDs are personal data and the deploy doc itself says this file should be empty; moved the twenty-plus loose usage and deployment markdown files into `docs/usage/` and `docs/history/`; grouped the loose SQL scripts into `sql/`; and hardened `.gitignore` so the buffer file, editor noise, and duplicate-snapshot patterns can never be committed again.

**Dead imports.** 151 unused imports removed across the tree (ruff-verified), including three inside an openpyxl availability probe that were imported but never used.

## What I added

`pyproject.toml` is now the single source of truth for dependencies, with optional extras (`excel`, `cad`, `pdf`, `cloud`, `all`, `dev`) mirroring the lazy-import architecture — `pip install -e ".[dev]"` gives a contributor everything in one command. `requirements.txt` remains only because Streamlit Cloud reads it, and the sync obligation is documented in both files.

`.github/workflows/ci.yml` runs ruff plus the full suite on every push and PR, and separately rejects any commit containing `(1)`, `kopie`, `_PATCHED`, or `__pycache__` artifacts. The lint gate is calibrated, not decorative: it selects correctness rules only, exempts `streamlit_app.py` from undefined-name checks because the app injects its module aliases through `globals().update(...)` (a deliberate lazy-loading design static analysis cannot see), and exempts `__init__.py` re-exports. The tree passes it clean today, which is the only honest way to introduce a lint gate.

`CONTRIBUTING.md` codifies the culture: CI is the gate, red `main` is an incident, tests and code land together (the analytics drift is named as the incident to learn from), physics changes carry evidence and a regression test, and the physics/UI boundary is enforced — equations live in `suspension/`, never in `streamlit_app.py`.

## The roadmap — in priority order

**First, decompose the monolith by strangulation, not rewrite.** `streamlit_app.py` is 28,143 lines. It works, and a big-bang rewrite would be the classic way to kill this project. The pattern that works: freeze it for additions, create `ui/`, and require every *new* tab to be a module with a `render(ctx)` entry point; then, each time a tab is touched for any other reason, move it. The lazy-module dictionary at line ~169 already defines the seams — each `*_mod` alias maps almost one-to-one onto a tab. Target: no file over 3,000 lines within two seasons. While doing this, decide the fate of `app.py` (4,563 lines): it is the legacy suspension-only entry point, no longer referenced as the deployed app; either mark it clearly as a lightweight standalone mode or delete it and let git remember.

**Second, resolve the licensing contradiction before any commercial conversation.** This tree's `LICENSE` is MIT; the repository root and README declare AGPL-3.0; and the root `LICENSE (1)` file literally contains a proxy error message ("Host not in allowlist: www.gnu.org") that someone committed instead of the license text. MIT and AGPL grant incompatible promises, and the README simultaneously courts professional teams for future commercial terms. If dual licensing (free for students, commercial for pros) is the plan — and it is a good plan — that requires AGPL plus a contributor license agreement from day one, because every outside contribution accepted under ambiguous terms makes relicensing harder. This is a one-day fix now and a lawyer-month later.

**Third, kill the root-level duplicate.** The repository root is a stale, less capable copy of this tree (fewer modules, fewer tests, older analytics). Two divergent copies of a 260 KB app in one repo guarantees the exact class of silent-contradiction bug KinematiK exists to prevent. Make `fsae_suspension/` the repository root, point the Streamlit Cloud `app_file` at it, and delete the rest in one commit.

**Fourth, protect the test suite's speed.** Three minutes headless is a superpower; teams stop running suites that cross ten. Add `--durations=10` to CI to watch for creep, and split any future solver test that needs more than a second into a fast smoke assertion plus a marked `slow` variant.

**Fifth, telemetry and trust.** The analytics system is thoughtfully built (fire-and-forget, env kill-switch, controlled vocabulary), but consent is implicit. Add a first-run notice in the app stating what the three retained event types are and how to disable them. For a tool whose brand is honesty about provenance, being visibly honest about telemetry is on-brand and cheap.

**Sixth, name an integration owner per subsystem module.** The AUTHORS file names one person. The coupling-graph architecture means a wrong sign convention in one module silently moves eight others — exactly why each `suspension/*.py` module should carry a `# Maintainer:` line and CI should eventually require review from that owner for changes under their path. That is the culture shift in one sentence: every number has an owner, in the product and in the repo.

## State of the tree as delivered

Test suite: 1,186 passed, 0 failed, 0 warnings, ~3 minutes. Lint: clean under the CI ruleset. Committed bytecode, stale snapshots, and user telemetry: removed. Packaging, CI, and contribution standards: in place. The next commit to `main` will be the first one in this project's history that a machine refuses to accept if it is broken — that is the moment the culture actually shifts.
