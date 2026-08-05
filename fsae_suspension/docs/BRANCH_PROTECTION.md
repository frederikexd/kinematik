# Branch protection — the missing gate

## Why this is the first thing to do

Streamlit Community Cloud **redeploys on every push to the tracked branch**.
The CI pipeline runs *alongside* that deploy, not in front of it. So today:

```
push  ──►  Streamlit Cloud deploys immediately  ──►  users see it
      └─►  CI runs, maybe goes red 8 minutes later
```

A red build still ships. Every green check in the structural audit — 2,731
tests, ruff, the packaging check, the reachability guards — can be bypassed by
one `git push` straight to the deployed branch. Until protection is on, the
pipeline is a **report**, not a gate.

The repo cannot fix this itself. Branch protection is a repository *setting*.
What the repo now provides:

- **a single required check** — the `CI gate` job in `ci.yml`, which fails if
  any of `lint`, `install` or `test` failed *or was skipped*. Require this one
  check and future jobs come under the gate automatically by being added to its
  `needs:` list. Requiring the three jobs individually means the next job
  someone adds is silently ungated.
- **a canary** — `.github/workflows/protection-canary.yml` asks the GitHub API
  every Monday and on every push whether protection is still configured, and
  fails if it has been removed or weakened. Protection turned off "just for one
  hotfix" and never turned back on is the normal way this decays.

---

## Do it with one command

Requires the `gh` CLI, authenticated as a repo admin.

```bash
gh api -X PUT "repos/:owner/:repo/branches/main/protection" \
  -H "Accept: application/vnd.github+json" \
  --input - <<'JSON'
{
  "required_status_checks": {
    "strict": true,
    "contexts": ["CI gate"]
  },
  "enforce_admins": true,
  "required_pull_request_reviews": {
    "required_approving_review_count": 1,
    "dismiss_stale_reviews": true
  },
  "restrictions": null,
  "allow_force_pushes": false,
  "allow_deletions": false,
  "required_linear_history": true,
  "required_conversation_resolution": true
}
JSON
```

Replace `main` if Streamlit Cloud tracks a different branch — **use the branch
the app actually deploys from**, which is the one shown in the Cloud dashboard
under the app's settings. Protecting a branch nobody deploys from achieves
nothing.

### What each setting is buying

| setting | what it prevents |
|---|---|
| `contexts: ["CI gate"]` | merging with a failing or skipped check |
| `strict: true` | a PR that passed against a stale base and breaks on merge |
| `enforce_admins: true` | the rule applying to everyone except the people who push most |
| `required_approving_review_count: 1` | a change reaching a deployed app with no second pair of eyes |
| `dismiss_stale_reviews` | an approval from three force-pushes ago counting for current code |
| `allow_force_pushes: false` | history being rewritten under a passing check |
| `required_linear_history` | a merge commit smuggling in unreviewed parents |

`enforce_admins` is the one people quietly skip. On a small team the admins
*are* the main committers, so exempting them exempts almost every push.

---

## Or through the UI

**Settings → Branches → Add branch protection rule**

1. Branch name pattern: the branch Streamlit Cloud deploys from
2. ☑ Require a pull request before merging → approvals: **1** → ☑ Dismiss stale approvals
3. ☑ Require status checks to pass → ☑ Require branches to be up to date → search **CI gate** and select it
4. ☑ Require conversation resolution before merging
5. ☑ Require linear history
6. ☑ Do not allow bypassing the above settings *(this is `enforce_admins`)*
7. ☐ Allow force pushes — leave **off**
8. ☐ Allow deletions — leave **off**

The `CI gate` check only appears in that search box **after it has run at least
once**. If it is missing, push this branch, let CI run, then come back.

---

## Turn the canary on

It needs to read branch protection, which the default `GITHUB_TOKEN` cannot do.

1. GitHub → Settings → Developer settings → **Fine-grained personal access
   token**, scoped to this repository, permission **Administration: read-only**
2. Repo → Settings → Secrets and variables → Actions → new secret named
   **`ADMIN_READ_TOKEN`**

Until that secret exists the canary reports a warning and exits 0 rather than
failing — an unconfigured canary must not look like a passing one.

---

## The workflow this creates

```
feature branch ──► PR ──► CI gate must be green ──► review ──► merge to main
                                                                    │
                                                     Streamlit Cloud deploys
```

Nobody pushes to the deployed branch directly, including admins. That is the
whole point.

---

## One honest caveat

Branch protection is only free on **public** repositories, and Community Cloud
requires a public repo anyway, so this costs nothing here. On a private repo it
needs GitHub Team or above — worth knowing before the project moves off
Community Cloud.
