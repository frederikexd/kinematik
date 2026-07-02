# KinematiK — analytics hardening (deploy notes)

Build: `0.12-analytics-hardened`

## Files in this package

- `streamlit_app.py` — main app (repo root)
- `suspension/analytics.py` — analytics module (deploy together with streamlit_app.py)
- `suspension/analytics_hardening.sql` — full analytics DB migration
- `fix_feature_funnel.sql` — standalone one-view fix (per-feature funnel)
- `requirements.txt` — pins streamlit>=1.58 and extra-streamlit-components
- `README.md`

## Deploy order

1. **Push `streamlit_app.py` and `suspension/analytics.py` together.**
   They are a matched pair — deploying one without the other causes an AttributeError.
2. **Run `suspension/analytics_hardening.sql` in Supabase.**
   Safe to re-run (drop-then-create, idempotent). Covers all views including
   the rewritten `v_retention` and fixed `v_time_to_first_result`.
3. **Confirm** build stamp in Usage section reads `0.12-analytics-hardened`
   and streamlit runtime reads `>= 1.58.0`.

## What was fixed this round

### total_users incremented by 2 on every reopen
- **Root cause (SQL):** `v_retention` per_user CTE resolved uid per row before
  grouping. A user whose cookie hadn't resolved on render 1 produced two
  different uid values (seed vs durable id), landing two rows in per_user and
  counting as two distinct people.
- **Fix:** Two-phase grouping. Phase 1 groups by session_id and takes
  `max(visitor_id)` — a cookie resolving on render 2 wins over the NULL from
  render 1. One session → one uid. Phase 2 aggregates by person.
- `total_users` is now `count(distinct session_id)` — every session ever.
  Increments by exactly 1 per reopen.

### returning_users stuck at 0 / not updating
- **Root cause (identity):** Cookie writes silently fail on Streamlit Cloud.
  44 sessions produced 43 distinct `ck-` ids — not durable. Every reopen
  minted a new seed so users never linked across visits.
- **Root cause (Python):** Early-exit guard fired on `"cookie (resolving…)"`
  and skipped the cookie block on render 2. The CookieManager never got a
  chance to read back the real id. `session_start` fired with the seed.
- **Root cause (Python):** `session_start` was emitted before visitor_id was
  stable, so the event logged with a throwaway id that differed from the
  durable id resolved one render later.
- **Fix (SQL):** `ck-` ids excluded from identity in `v_retention`. Only
  `fp-` fingerprint and named member used as durable identity. `returning_users`
  now counts return visits (`sum(visits - 1)`), not distinct people.
- **Fix (Python / streamlit_app.py):** Early-exit guard now only skips
  re-resolution when kind is confirmed durable. First-render branch no longer
  assigns seed to `_vid` — leaves None so fingerprint runs. Cookie-absent
  branch no longer mints `ck-` seeds.
- **Fix (Python / analytics.py):** `init()` defers `session_start` emit when
  `_ax_resolved_vid_kind == "cookie (resolving…)"`. Fires on render 2 with
  stable id.

### time-to-first-result showed 0 min / was inaccurate
- **Root cause:** `v_time_to_first_result` anchored off `session_start`, which
  fires on render 2 after other events including `first_result`. Produced
  negative deltas that averaged to zero.
- **Fix:** View now uses `min(occurred_at)` across all events as true session
  start. Guard `t_first_result >= t_start` excludes historical negatives.
- Display now shows seconds ("28 sec") instead of rounding to "0 min".

### UI changes
- "Return vs FSAE members" tile removed.
- FSAE roster size input removed.
- Time-to-first-result displays in seconds.

## Note on historical data

SQL view fixes are retroactive — they recompute from raw events already in the
database. No past events were fabricated or deleted. Python fixes affect future
sessions only.

## Identity strategy summary

| Id type | Durable? | Used for identity? |
|---|---|---|
| `fp-` fingerprint (IP+UA) | Yes — stable per device | ✅ Yes |
| Named member | Yes — most durable | ✅ Yes |
| `ck-` cookie | No — writes fail on Streamlit Cloud | ❌ Excluded |
| `session_id` fallback | No — new each session | counted in total_users only |
