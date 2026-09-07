-- ============================================================================
--  analytics_global_counter.sql
--  The headline Analytics number: ONE global total, rolling last 30 days,
--  summed across EVERY workspace, identical for every user.
--
--  WHY THIS EXISTS
--  ---------------
--  The analytics dashboards already count over all of analytics_events with no
--  per-workspace filter, so today the headline is naturally global. This file
--  pins that intent down so it can't drift:
--    1. v_global_events_30d — an explicit "last 30 days, all workspaces" count,
--       not reliant on the retention trim job's timing.
--    2. global_events_30d() — a SECURITY DEFINER function that returns the same
--       number even if someone later enables Row-Level Security on
--       analytics_events. (workspace_isolation.sql deliberately does NOT scope
--       analytics_events, so the raw view works now; the function is the
--       belt-and-suspenders that keeps the global total readable regardless.)
--
--  Preserves the current cycle automatically: it only ever reads existing rows,
--  never deletes or forks them, so the 5-days-in count carries straight through
--  the switch to invite links / accounts and keeps climbing on the same rolling
--  window.
--
--  Idempotent. Run AFTER analytics_schema.sql (needs analytics_events). Order-
--  independent w.r.t. the workspace_* scripts.
-- ============================================================================

-- ----------------------------------------------------------------------------
--  1. The view — plain, readable, explicit 30-day rolling window.
--     Works as long as analytics_events has no RLS (the current design).
-- ----------------------------------------------------------------------------
create or replace view v_global_events_30d as
select
    count(*)                                             as events_30d,
    count(*) filter (where event_type = 'feature_engage')    as feature_uses_30d,
    count(*) filter (where event_type = 'workflow_complete') as workflows_30d,
    count(distinct session_id)                           as sessions_30d,
    count(distinct coalesce(nullif(member,''), visitor_id, session_id))
                                                         as people_30d,
    min(occurred_at)                                     as window_start,
    max(occurred_at)                                     as window_end
from analytics_events
where occurred_at >= now() - interval '30 days';

comment on view v_global_events_30d is
  'Headline usage number: total events in the last 30 days across ALL
   workspaces combined. Same value for every user. Rolling window (now-30d),
   independent of the retention trim job. Reads all rows, so the current cycle
   is preserved across the accounts switch.';


-- ----------------------------------------------------------------------------
--  2. The RLS-proof accessor. If analytics_events is EVER put behind RLS later
--     (it isn't today), a normal SELECT would silently collapse to the caller's
--     own rows and the "global" number would break. This SECURITY DEFINER
--     function runs as its owner, bypassing RLS, so the global total stays
--     global no matter what. Grant EXECUTE to anon+authenticated so the app can
--     call it whether or not the user is signed in.
-- ----------------------------------------------------------------------------
create or replace function global_events_30d()
returns bigint
language sql
stable
security definer
set search_path = public
as $$
    select count(*)::bigint
    from analytics_events
    where occurred_at >= now() - interval '30 days';
$$;

comment on function global_events_30d() is
  'Global rolling-30-day event count across all workspaces. SECURITY DEFINER so
   it keeps returning the true global total even if analytics_events is later
   placed behind Row-Level Security. Safe to expose: returns a single integer,
   no row data.';

revoke all on function global_events_30d() from public;
grant execute on function global_events_30d() to anon, authenticated;
