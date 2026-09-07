# Landing the audit on GitHub

The repo already exists and Streamlit Community Cloud already deploys from it,
so this is not "create a repo" — it is "land ~190 changed files and 158
deletions on a branch that auto-deploys to a public app."

**The ordering is the whole point.** Two constraints fight each other:

- Merging to the deployed branch ships to production **immediately**.
- Branch protection cannot require the `CI gate` check until GitHub has *seen*
  that check run at least once.

So protection goes on **after the PR's CI has run but before the merge**. Do it
in the other order and the very commit that installs your safety net arrives
without one.

---

## 0. Know how to get back

```bash
git rev-parse HEAD          # the currently deployed commit — write it down
```

Rollback later is `git revert <merge-sha> && git push`. Streamlit Cloud
redeploys from branch HEAD, so a revert is a deploy.

Also note which branch the Cloud dashboard says the app tracks. Everything
below says `main`; use the real one.

## 1. Branch

```bash
git checkout -b structural-audit-2026-08
```

Never apply this directly to the deployed branch.

## 2. Apply the delta

```bash
unzip -o kinematik_changes_complete.zip -d /tmp/delta
cp -r /tmp/delta/fsae_suspension/. .
./APPLY_REMOVALS.sh
```

`APPLY_REMOVALS.sh` moves 158 files into `_attic/`, which is gitignored — so
git sees them as deletions. That is intended. They stay recoverable from git
history forever; `_attic/` is a local convenience, not the backup.

## 3. Read the diff before staging it

```bash
git add -A
git status
git diff --cached --stat | tail -20
```

Expect roughly 190 modified/added and 158 deleted. **Check the deletions
first** — that is the part that cannot be undone by reading a diff later.
`REMOVED_FILES.txt` lists every one with its reason in the audit.

Sanity check nothing load-bearing vanished:

```bash
test -f streamlit_app.py && test -f requirements.txt && echo "entry point + deps present"
```

## 4. Prove it locally before GitHub sees it

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -e ".[all]"
ruff check .
python -m pytest
```

Expect **2,731 passed, 17 skipped, 0 failed** and a clean ruff. If your machine
disagrees with that, stop here — my numbers came from a container, and a
divergence is information.

Then actually run it:

```bash
streamlit run streamlit_app.py
```

Click into **Integration** and open the four newly wired panels — Architecture
synthesis, Transient degradation, Calculation report, Worthwhile once
assembled?. They were unreachable before this change, so this is the first time
anyone has opened them through the app. Also open the aero tab's **ANSYS
run-log consolidation** view: its body moved to `ui/run_log.py`, and while its
27 tests pass, tests are not a browser.

## 5. Push and open the PR

```bash
git commit -m "Structural audit: fix defects, add CI, wire orphaned panels"
git push -u origin structural-audit-2026-08
gh pr create --fill
```

CI now runs on the PR. Wait for it. This is also what registers `CI gate` with
GitHub so the settings picker can find it.

## 6. Turn on branch protection — NOW, before merging

Follow `docs/BRANCH_PROTECTION.md`. One `gh api` command, or the UI click-path.

Doing it at this point means the audit merge is itself the first commit to pass
through the gate, which is a better smoke test than any you could design.

## 7. Merge, and watch the deploy

Merging triggers a Cloud redeploy. Open **Manage app** in the Cloud dashboard
and watch the logs through startup rather than assuming.

What could plausibly go wrong here and nowhere else:

- a dependency resolving differently on Cloud's image than locally
- the app exceeding memory on boot (Community Cloud allows ~1 GB total)
- a panel that renders locally but not under Cloud's session model

If the app fails to start, revert to the SHA from step 0 and diagnose on a
branch. Do not debug on the deployed branch.

## 8. Finish the configuration

- **Pin Python to 3.12** in the app's advanced settings. `runtime.txt` is
  Heroku-style and Community Cloud does not read it, and `cascadio` has no 3.13
  wheel — STEP import degrades silently on a newer interpreter.
- **Add `ADMIN_READ_TOKEN`** (fine-grained PAT, Administration:read) so the
  protection canary goes from warning to actually checking.
- **Fill in `.github/CODEOWNERS`** with real handles and uncomment it. An
  enabled CODEOWNERS pointing at a nonexistent user blocks every PR, which is
  why it ships commented out.

## 9. Prove the gate works

The step everyone skips.

```bash
git checkout -b prove-the-gate
echo "def test_deliberate_failure(): assert False" > tests/test_gate_check.py
git add -A && git commit -m "temp: prove CI blocks a merge" && git push -u origin prove-the-gate
gh pr create --fill
```

The PR must be **unmergeable**. If GitHub offers you the merge button, the
protection is misconfigured — most likely the required check name does not
match `CI gate`, or `enforce_admins` is off and you are an admin.

Delete the branch afterwards.

---

## Then verify against the real world

Everything above proves the code is internally consistent. It does not prove
the engineering is right. Inside the three-week window:

1. Run two or three brackets through **both KinematiK and Ansys**, deliberately
   choosing cases near the 1.5 FoS gate. The tear-out and net-section
   corrections changed results in the conservative direction; confirm that with
   a solver rather than with my reasoning.
2. Upload a real STEP file and watch memory in **Manage app**. The cap is now
   50 MB for a reason — see the audit's deployment section.
3. Tell the team the FoS numbers moved. Anything screened during testing needs
   re-checking, and tear-out was up to 2.67x optimistic.
