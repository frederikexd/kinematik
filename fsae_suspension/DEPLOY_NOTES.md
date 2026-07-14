# KinematiK stability patch — deploy notes

Contents mirror the repo layout (fsae_suspension/…). Full suite after this
patch: **1111 passed, 0 failed** (was 41 failed / 1060 passed).

## What's in it

1. **Test suite green + guard** — `suspension/__init__.py` registers the 43
   symbols and 4 submodules that were dropped from the lazy re-export tables
   in the four-branch merge; `suspension/myth_rules/brakes.py` adds the two
   throttle-return myth rules whose tests existed but whose implementation
   was never merged; `tests/test_public_api_exports.py` fails CI on any
   future export drift.

2. **Optimistic locking (no more silent last-write-wins)** —
   `suspension/project.py` (StaleWriteError, version-tracking ProjectStore,
   CAS legacy backend), `suspension/workspace.py` (CAS on the live
   tenant-scoped backend + local backend), `streamlit_app.py` (conflict
   banner + one-click reload in save_store). `tests/test_optimistic_locking.py`
   simulates two editors and locks the contract in.

3. **Server-side snapshot history** — `suspension/project_history.sql`
   snapshots every overwritten project blob via a Postgres trigger (last 20
   versions per project, member-readable under RLS). Recovery no longer
   depends on the app having behaved.

4. **CI gate** — `.github/workflows/ci.yml` runs the full suite on every
   push/PR (includes rtree so the chassis collision tests actually run).

## Deploy order (matters)

1. Run `suspension/project_history.sql` in the Supabase SQL editor
   (idempotent; AFTER workspace_isolation.sql — check the two policy lines
   if your membership helper is named differently).
2. Push the Python files together: `suspension/__init__.py`,
   `suspension/project.py`, `suspension/workspace.py`,
   `suspension/myth_rules/brakes.py`, `streamlit_app.py`.
3. Commit the two test files and the workflow; confirm the Actions run is
   green before the Streamlit Cloud redeploy picks up main.

## Behaviour changes users may notice

- Two people editing the same workspace: the second save now shows
  "Not saved — a teammate saved a newer version…" with a reload button,
  instead of silently erasing the first person's edits. This is the point.
- First save into a workspace that already has a versioned blob (e.g. a
  session that loaded an empty project because of a transient read failure)
  is refused rather than wiping the existing data.
- Legacy/unversioned blobs upgrade transparently on their first save.

## Added in this revision (Phase 1 close-out)

5. **Exception audit + 4 trust fixes** — `docs/EXCEPTION_AUDIT.md` classifies
   all 325 broad handlers; `streamlit_app.py` now states failures inside
   handover documents (cross-team checks x2, pedal 2000 N verdict) and warns
   when the lap sim runs without the declared aero. Conventions for future
   code are written down in the audit doc.

6. **`.streamlit/config.toml`** — XSRF protection and CORS re-enabled
   (they were disabled for tunnel testing and shipped to production).
   Replace your deployed config with this one.

7. **`requirements-pinned.txt`** — exact-version pins; the core set is the
   environment the full 1111-test suite passed on. Adopt by renaming over
   `requirements.txt` at the repo root. The three lazily-imported packages
   not exercised in the sandbox (supabase, cascadio, fast_simplification)
   are pinned to latest-known-good — open each feature once after the first
   deploy to confirm.

8. **Both READMEs reconciled** — hand-copied usage stats replaced with a
   pointer to the live Analytics tab (lifetime = baseline + 30-day window),
   and the analytics description now matches the code: random session UUID +
   cookie, no IP/device fingerprinting, opt-in member name, 30-day purge.
   `README.root.md` → repo root `README.md`;
   `README.fsae_suspension.md` → `fsae_suspension/README.md`.
