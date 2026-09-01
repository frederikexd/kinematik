-- ============================================================================
--  fix_feature_allowlist.sql
--  Fix: insert or update on "analytics_events" violates foreign key
--       constraint "ae_feature_fk"   (SQLSTATE 23503)
--
--  Supersedes fix_missing_features.sql, which patched two features ('docs',
--  'frames') and is now well out of date.
--
--  WHAT HAPPENED
--  -------------
--  ae_feature_fk requires every analytics_events.feature to exist in the
--  known_features allow-list. The app logs 40 tab ids; the allow-list seeds
--  24 of them. Sixteen tabs have therefore been throwing away every event
--  they ever produced:
--
--      daq, earshot, flexgen, forge, fusebox, genesis, genesis_fc, ghost,
--      morph, omni, phantom, phantom_env, proof, saboteur, stochastic, thermic
--
--  Nothing surfaced this except a line in the Postgres log, because the
--  insert fails server-side after the user has already had their answer. The
--  feature works; only the record of it being used is lost.
--
--  That is the worst possible bias in a usage dataset. The missing sixteen are
--  not a random sample — they are disproportionately the newer and more
--  distinctive tabs, i.e. exactly the ones whose adoption you would most want
--  to measure, and exactly the ones a prospective team is most likely to try
--  first. Any conclusion drawn from this data so far ("nobody uses X") may
--  simply mean X was never allowed to report.
--
--  THE STRUCTURAL FIX
--  ------------------
--  Seeding the sixteen fixes today and guarantees a repeat: the next tab added
--  to _TAB_META without a matching seed starts silently dropping events again,
--  and nobody finds out for months. So the constraint is kept — it still does
--  its real job of keeping the feature vocabulary closed and joinable — but an
--  unknown feature now auto-registers instead of rejecting the event.
--
--  Auto-registered rows are flagged, so the allow-list stays an honest
--  inventory: you can see at a glance which features were declared
--  deliberately and which the app introduced without anyone updating the SQL.
--
--  Idempotent. Safe to run more than once.
-- ============================================================================

-- ---------------------------------------------------------------------------
--  1. Provenance flag, so an auto-registered feature is distinguishable from
--     a deliberately declared one. Added defensively; older databases will not
--     have it.
-- ---------------------------------------------------------------------------
alter table known_features
    add column if not exists auto_registered boolean not null default false;

-- ---------------------------------------------------------------------------
--  2. Seed every tab the app currently logs. Labels match _TAB_META in
--     streamlit_app.py. `on conflict do update` keeps existing labels correct
--     without disturbing rows that were already right.
-- ---------------------------------------------------------------------------
insert into known_features (feature, label, is_tab, reactive_only) values
    ('daq',          'DAQ Plan',              true, false),
    ('earshot',      'Earshot',               true, false),
    ('flexgen',      'FlexGen',               true, false),
    ('forge',        'Forge',                 true, false),
    ('fusebox',      'Fusebox',               true, false),
    ('genesis',      'Genesis',               true, false),
    ('genesis_fc',   'Genesis Full Car',      true, false),
    ('ghost',        'Ghost',                 true, false),
    ('morph',        'Morph',                 true, false),
    ('omni',         'Omni',                  true, false),
    ('phantom',      'Phantom',               true, false),
    ('phantom_env',  'Phantom Envelope',      true, false),
    ('proof',        'Proof Engine',          true, false),
    ('saboteur',     'Saboteur',              true, false),
    ('stochastic',   'Stochastic',            true, false),
    ('thermic',      'Thermic',               true, false),
    -- re-assert the two from the superseded fix, so this file stands alone
    ('docs',         'Documentation',         true, false),
    ('frames',       'Frames & Datums',       true, false)
on conflict (feature) do update
    set label  = excluded.label,
        is_tab = excluded.is_tab;

-- ---------------------------------------------------------------------------
--  3. Never lose an event to this again.
--
--     A BEFORE INSERT trigger registers an unfamiliar feature rather than
--     letting the FK reject the row. The allow-list keeps its purpose — every
--     feature that appears in events also exists in known_features, so joins
--     and coverage views stay total — but the failure mode flips from "drop
--     the data silently" to "record the data and tell me the vocabulary
--     grew".
--
--     This cannot be used to inject junk: analytics_events is only writable by
--     the app's own logging path, and `feature` is drawn from _TAB_META, not
--     from user input.
-- ---------------------------------------------------------------------------
create or replace function _ae_autoregister_feature()
returns trigger language plpgsql security definer set search_path = public as $$
begin
    if new.feature is null then
        return new;
    end if;
    if not exists (select 1 from known_features k where k.feature = new.feature) then
        insert into known_features (feature, label, is_tab, reactive_only,
                                    auto_registered)
        values (new.feature,
                initcap(replace(new.feature, '_', ' ')),
                true, false, true)
        on conflict (feature) do nothing;   -- concurrent inserts race benignly
        --  PL/pgSQL's placeholder is %, not %s. Written as %s it prints
        --  "feature pcbs" — the % eats the argument and the s is left behind.
        raise warning 'analytics: auto-registered unknown feature % — add it '
                      'to the seed in fix_feature_allowlist.sql with a proper '
                      'label', new.feature;
    end if;
    return new;
end;
$$;

drop trigger if exists trg_ae_autoregister_feature on analytics_events;
create trigger trg_ae_autoregister_feature
    before insert on analytics_events
    for each row execute function _ae_autoregister_feature();

-- ---------------------------------------------------------------------------
--  4. Housekeeping view: which features did the app introduce without anyone
--     updating the SQL, and which are seeded but have never been seen?
--     The second list is how you notice a tab id that got renamed.
-- ---------------------------------------------------------------------------
create or replace view v_feature_registry_health as
    select k.feature,
           k.label,
           k.auto_registered,
           count(e.*)           as events,
           max(e.occurred_at)   as last_seen
    from known_features k
    left join analytics_events e on e.feature = k.feature
    group by k.feature, k.label, k.auto_registered
    order by k.auto_registered desc, events desc;

-- ---------------------------------------------------------------------------
--  VERIFY
--
--    -- should be zero rows
--    select * from v_orphaned_feature_events;
--
--    -- anything with auto_registered = true wants a real label in the seed
--    select * from v_feature_registry_health where auto_registered;
--
--    -- and the sixteen should start accumulating events from now on
--    select * from v_feature_registry_health where events = 0;
--
--  NOTE ON THE HISTORY: the rejected events are gone. Usage of those sixteen
--  tabs before today was never recorded and cannot be recovered, so treat any
--  earlier comparison between tabs as unreliable rather than as evidence.
-- ---------------------------------------------------------------------------
