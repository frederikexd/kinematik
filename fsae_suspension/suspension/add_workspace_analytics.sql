-- ============================================================================
--  add_workspace_analytics.sql
--  Make usage answerable per TEAM, not just in aggregate.
--
--  WHY NOW
--  -------
--  analytics_events records who (anonymously), what, and when — but not which
--  workspace. With one team that was the same question. With nine teams and 26
--  accounts it is not: "is anyone actually adopting this" is now a per-team
--  question, and the table cannot answer it. Every existing view sums nine
--  teams into one number, so a single enthusiastic workspace and nine teams
--  poking around once look identical.
--
--  That distinction is the only one that matters right now. Nine teams having
--  looked is worth far less than one team using it in week two — for the
--  product, and for what you can honestly tell the next team you contact.
--
--  PRIVACY POSTURE — UNCHANGED FOR PEOPLE, NEW FOR TEAMS
--  -----------------------------------------------------
--  This adds attribution at the WORKSPACE level only. Individuals stay as
--  anonymous as they were: session_id and visitor_id remain opaque, `member`
--  remains self-entered and optional, and nothing here links an event to an
--  auth user. What changes is that team-level usage becomes visible to you.
--
--  That is ordinary product analytics, but it is a real change in what you can
--  see about someone else's team, and these are named universities you are
--  talking to directly. Say so somewhere they can read it before the first
--  report is drawn from it — being asked "can you see what we're doing?" and
--  answering "yes, at team level, here's exactly what" is a good conversation.
--  Being caught not having mentioned it is not.
--
--  BACKFILL IS IMPOSSIBLE
--  ----------------------
--  Existing rows have no workspace and cannot be assigned one — the link never
--  existed. workspace_id stays NULL for everything before this migration, and
--  the views below exclude NULLs rather than guessing. Per-team history starts
--  the day the app change ships, not today.
--
--  Idempotent.
-- ============================================================================

alter table analytics_events
    add column if not exists workspace_id uuid;

--  No foreign key on purpose. Analytics must never be able to block a write or
--  be blocked by a delete: if a workspace is removed, its events should remain
--  as history rather than cascade away or start raising 23503 — which is
--  exactly the failure that lost sixteen tabs' worth of data.
create index if not exists ae_workspace_idx
    on analytics_events (workspace_id, occurred_at desc);

-- ---------------------------------------------------------------------------
--  Per-team weekly read. One row per workspace per ISO week.
--
--  `active_visitors` counts distinct browsers, which is the closest honest
--  proxy for people. `features_used` counts distinct tabs touched — a team
--  using nine features looks very different from one that opened the same tab
--  nine times, and only one of those is adoption.
-- ---------------------------------------------------------------------------
create or replace view v_workspace_weekly as
    select date_trunc('week', e.occurred_at)::date        as week,
           e.workspace_id,
           w.name                                          as workspace,
           count(distinct e.visitor_id)                    as active_visitors,
           count(distinct e.session_id)                    as sessions,
           count(distinct e.feature)
               filter (where e.feature is not null)        as features_used,
           count(*) filter (where e.event_type = 'first_result')
                                                           as first_results,
           count(*) filter (where e.event_type = 'workflow_complete')
                                                           as workflows,
           count(*) filter (where e.event_type = 'error')  as errors,
           max(e.occurred_at)                              as last_seen
      from analytics_events e
      left join workspaces w on w.id = e.workspace_id
     where e.workspace_id is not null
     group by 1, 2, 3
     order by 1 desc, active_visitors desc;

-- ---------------------------------------------------------------------------
--  The question worth asking every Monday: did anyone come back?
--
--  A team that used it in week 1 and never again is a demo. A team present in
--  two consecutive weeks is adoption. `weeks_active` and `returned` separate
--  those two, and nothing else on this list matters as much.
-- ---------------------------------------------------------------------------
create or replace view v_workspace_adoption as
    with wk as (
        select distinct workspace_id,
               date_trunc('week', occurred_at)::date as week
          from analytics_events
         where workspace_id is not null
    )
    select k.workspace_id,
           w.name                                   as workspace,
           min(k.week)                              as first_week,
           max(k.week)                              as last_week,
           count(*)                                 as weeks_active,
           count(*) > 1                             as returned,
           max(k.week) >= date_trunc('week', now())::date - 7
                                                    as active_recently
      from wk k
      left join workspaces w on w.id = k.workspace_id
     group by 1, 2
     order by weeks_active desc, last_week desc;

-- ---------------------------------------------------------------------------
--  Which features survive contact with a second team?
--
--  A feature used once by one workspace is a curiosity. One used by several
--  workspaces, repeatedly, is the thing to keep building. `workspaces` is the
--  column to sort by — not `uses`, which one keen team can dominate.
-- ---------------------------------------------------------------------------
create or replace view v_feature_traction as
    select e.feature,
           k.label,
           count(distinct e.workspace_id)   as workspaces,
           count(distinct e.visitor_id)     as visitors,
           count(*)                         as uses,
           max(e.occurred_at)               as last_used
      from analytics_events e
      left join known_features k on k.feature = e.feature
     where e.workspace_id is not null
       and e.feature is not null
     group by 1, 2
     order by workspaces desc, visitors desc;

-- ---------------------------------------------------------------------------
--  THE MONDAY QUERY — the whole weekly read in one statement.
--
--    select * from v_workspace_adoption;              -- who came back
--    select * from v_workspace_weekly limit 20;       -- what happened, by team
--    select * from v_feature_traction limit 15;       -- what travels
--    select * from v_feature_registry_health
--     where auto_registered;                          -- tabs missing a label
--
--  Read them in that order. The first one is the only one that can tell you
--  whether the product is working.
-- ---------------------------------------------------------------------------
