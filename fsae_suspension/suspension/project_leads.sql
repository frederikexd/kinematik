-- ============================================================================
--  KinematiK — Project-lead gating for workspace creation
--  Idempotent: safe to re-run. Run AFTER workspace_isolation.sql
--  (and it pairs with shared_scope.sql — order relative to it does not matter).
--
--  WHY
--    An "ecosystem" is every workspace created by ONE project lead
--    (workspaces.created_by); the three shared surfaces (Lead Notes,
--    Integration ledger, Team CAD library) are shared across that whole set
--    and never cross to another lead's. That grouping only holds if the people
--    a lead INVITES cannot create their OWN workspaces — otherwise an invited
--    subsystem lead would silently start a second, separate ecosystem.
--
--    So: only the OWNER and the PROJECT LEADS THE OWNER APPOINTS may create
--    workspaces. Everyone else can still be invited into a lead's workspaces
--    (redeem_workspace_invite is untouched); they simply cannot spin up their
--    own. General self-registration as a lead is OFF — the owner (identified by
--    email, app.owner_emails) appoints leads via promote_project_lead(email).
--
--  WHAT THIS ADDS
--    project_leads                 one row per user who has registered as a
--                                  project lead (the source of truth the UI
--                                  reads to show who signed up as a lead).
--    register_project_lead()       idempotent self-registration -> becomes a lead.
--    is_project_lead(uuid)         predicate used by RLS + the create RPC.
--    am_i_project_lead()           convenience for the caller (UI status).
--    create_workspace(name, kind)  the RPC the app already calls — now DEFINED
--                                  here, and it ENFORCES lead status + a cap.
--    project_lead_status()         one-row snapshot the picker renders:
--                                  {is_lead, workspace_count, workspace_cap,
--                                   can_create}.
--
--  DEFENCE IN DEPTH
--    The create_workspace RPC is the front door and returns friendly errors,
--    but we ALSO tighten the ws_insert RLS policy so a direct table insert by
--    a non-lead fails at the database. The shared-scope workspace is created
--    by a SECURITY DEFINER trigger (shared_scope.sql), which bypasses RLS, so
--    tightening ws_insert does not block shared-scope provisioning.
-- ============================================================================

begin;

-- ----------------------------------------------------------------------------
--  0a. Clean teardown of any PRIOR version of this migration.
--      `create or replace function` cannot change a function's RETURN TYPE in
--      place (Postgres error 42P13) — and this migration deliberately changes
--      some signatures vs. an earlier install (e.g. project_lead_status() now
--      returns an extra is_owner column). So we DROP the functions first and
--      let the definitions below recreate them cleanly.
--
--      Order matters: the ws_insert policy references is_project_lead(), so the
--      policy must be dropped before the function it depends on. Everything is
--      inside this transaction, so a failure rolls back with nothing half-done.
--      Each drop is `if exists`, making the whole migration safe on a FRESH
--      database too (nothing to drop → no-op).
-- ----------------------------------------------------------------------------
drop policy if exists ws_insert on workspaces;   -- depends on is_project_lead()

drop function if exists register_project_lead();
drop function if exists promote_project_lead(text);
drop function if exists project_lead_status();
drop function if exists create_workspace(text, text);
drop function if exists am_i_project_lead();
drop function if exists is_project_lead(uuid);
drop function if exists _project_lead_ws_count(uuid);
drop function if exists _project_lead_bootstrap_open();
drop function if exists is_owner();
drop function if exists kinematik_caller_email();
drop function if exists kinematik_owner_emails();
drop function if exists project_lead_workspace_cap();

-- ----------------------------------------------------------------------------
--  0. Tunable: how many real workspaces one lead may own. Kept as an IMMUTABLE
--     function so it can be referenced in checks and swapped in one place.
-- ----------------------------------------------------------------------------
create or replace function project_lead_workspace_cap()
returns integer language sql immutable set search_path = public as $$
    select 10;
$$;

-- ----------------------------------------------------------------------------
--  0b. OWNER identification (config-driven; no table row required).
--      The deployment OWNER is you: identified by email via a per-deployment
--      Postgres setting, app.owner_emails (comma-separated, case-insensitive).
--      The owner is a project lead IMPLICITLY and can appoint other leads.
--      Set it once (survives restarts):
--          alter database postgres set app.owner_emails = 'you@example.com';
--      Multiple owners allowed: 'a@x.com, b@y.com'.
-- ----------------------------------------------------------------------------
create or replace function kinematik_owner_emails()
returns text[] language sql stable set search_path = public as $$
    select coalesce(
        (select array_agg(btrim(lower(e)))
         from unnest(string_to_array(
                current_setting('app.owner_emails', true), ',')) as e
         where btrim(e) <> ''),
        '{}'::text[]);
$$;

-- The caller's email, taken from the JWT claims Supabase puts on the session.
create or replace function kinematik_caller_email()
returns text language sql stable set search_path = public as $$
    select nullif(lower(btrim(coalesce(
        current_setting('request.jwt.claim.email', true),
        (current_setting('request.jwt.claims', true)::jsonb ->> 'email')
    ))), '');
$$;

-- Is the CALLER the (a) deployment owner?
create or replace function is_owner()
returns boolean language sql stable security definer set search_path = public as $$
    select coalesce(kinematik_caller_email() = any (kinematik_owner_emails()), false);
$$;

revoke all on function kinematik_owner_emails() from anon, authenticated;
revoke all on function kinematik_caller_email() from anon, authenticated;
revoke all on function is_owner()               from public;
grant execute on function is_owner()            to authenticated;

-- ----------------------------------------------------------------------------
--  1. Registry of project leads
-- ----------------------------------------------------------------------------
create table if not exists project_leads (
    user_id       uuid primary key,          -- auth.users.id
    registered_at timestamptz not null default now()
);

alter table project_leads enable row level security;
alter table project_leads force  row level security;

-- A user may see ONLY their own lead row (so the UI can tell them whether they
-- registered). No client-side insert/update/delete: registration is RPC-only.
drop policy if exists pl_select_self on project_leads;
create policy pl_select_self on project_leads for select
    using (user_id = auth.uid());

revoke all on project_leads from anon, authenticated;
grant select on project_leads to authenticated;   -- still filtered by RLS above

-- ----------------------------------------------------------------------------
--  2. Helpers
-- ----------------------------------------------------------------------------

-- Is the given user a registered project lead? SECURITY DEFINER so RLS on
-- project_leads (self-only select) doesn't hide OTHER users from policy checks.
create or replace function is_project_lead(uid uuid)
returns boolean language sql stable security definer set search_path = public as $$
    -- The owner is a lead implicitly (matched by email when asking about
    -- themselves); anyone in project_leads is a lead explicitly.
    select (uid = auth.uid() and is_owner())
        or exists (select 1 from project_leads pl where pl.user_id = uid);
$$;

-- Convenience: is the CALLER a lead? (Safe to call from the client.)
create or replace function am_i_project_lead()
returns boolean language sql stable security definer set search_path = public as $$
    select is_project_lead(auth.uid());
$$;

-- How many REAL (non-shared-scope) workspaces this user has created. The
-- is_shared_scope column exists only after shared_scope.sql; probe for it so
-- this function works whether or not that migration ran.
create or replace function _project_lead_ws_count(uid uuid)
returns integer language plpgsql stable security definer set search_path = public as $$
declare n integer;
begin
    if exists (select 1 from information_schema.columns
               where table_name = 'workspaces' and column_name = 'is_shared_scope') then
        execute 'select count(*)::int from workspaces
                 where created_by = $1 and not is_shared_scope'
            into n using uid;
    else
        select count(*)::int into n from workspaces where created_by = uid;
    end if;
    return coalesce(n, 0);
end $$;

revoke all on function is_project_lead(uuid)          from public;
grant execute on function is_project_lead(uuid)       to authenticated;
revoke all on function am_i_project_lead()            from public;
grant execute on function am_i_project_lead()         to authenticated;
revoke all on function _project_lead_ws_count(uuid)   from public;
grant execute on function _project_lead_ws_count(uuid) to authenticated;

-- ----------------------------------------------------------------------------
--  3. Registration / appointment of project leads.
--
--     Model requested: ONLY the owner (and the leads the owner has already
--     appointed) may create leads. General self-registration is OFF — an
--     invited subsystem lead or member cannot make themselves a lead and fork
--     a new ecosystem.
--
--     BOOTSTRAP: on a brand-new install with NO owner email configured and NO
--     leads yet, the door would be closed to everyone. To avoid that lockout,
--     the first caller in that specific empty state is allowed to self-register
--     (becoming the de-facto first lead). The moment an owner email is set OR
--     any lead exists, that escape hatch closes. Configure app.owner_emails to
--     make ownership explicit and skip the bootstrap entirely.
-- ----------------------------------------------------------------------------

-- True only in the empty, unconfigured state described above.
create or replace function _project_lead_bootstrap_open()
returns boolean language sql stable security definer set search_path = public as $$
    select cardinality(kinematik_owner_emails()) = 0
       and not exists (select 1 from project_leads);
$$;
revoke all on function _project_lead_bootstrap_open() from anon, authenticated;

-- register_project_lead(): opt the CALLER in. Permitted only for the owner, an
-- existing lead, or the one-time bootstrap case. Idempotent; returns true when
-- the caller is (now) a lead, and raises 42501 when they aren't allowed.
create or replace function register_project_lead()
returns boolean language plpgsql security definer set search_path = public as $$
begin
    if auth.uid() is null then
        raise exception 'sign in first' using errcode = '28000';
    end if;
    -- Already a lead (or the owner)? No-op success.
    if is_project_lead(auth.uid()) then
        return true;
    end if;
    if not (is_owner() or _project_lead_bootstrap_open()) then
        raise exception
            'only the project owner can register project leads — ask the owner to appoint you, or join via an invite link'
            using errcode = '42501';
    end if;
    insert into project_leads (user_id) values (auth.uid())
        on conflict (user_id) do nothing;
    return true;
end $$;

revoke all on function register_project_lead()    from public;
grant execute on function register_project_lead() to authenticated;

-- promote_project_lead(email): the OWNER (or an existing lead) appoints someone
-- else, by email, as a project lead. The target must already have an account.
-- Idempotent; returns the target's user id.
create or replace function promote_project_lead(lead_email text)
returns uuid language plpgsql security definer set search_path = public as $$
declare v_uid uuid;
begin
    if not (is_owner() or is_project_lead(auth.uid())) then
        raise exception
            'permission denied: only the project owner or an existing lead may appoint a project lead'
            using errcode = '42501';
    end if;
    select id into v_uid from auth.users
    where lower(email) = lower(btrim(lead_email)) limit 1;
    if v_uid is null then
        raise exception 'no user with email %', lead_email using errcode = 'P0002';
    end if;
    insert into project_leads (user_id) values (v_uid)
        on conflict (user_id) do nothing;
    return v_uid;
end $$;

revoke all on function promote_project_lead(text)    from public, anon;
grant execute on function promote_project_lead(text) to authenticated;

-- ----------------------------------------------------------------------------
--  4. The create_workspace RPC the app calls (auth.py::create_workspace).
--     Enforces: signed in · registered lead · under the cap · valid kind.
--     Runs as SECURITY DEFINER so the insert + owner bootstrap happen with a
--     resolved auth.uid(); the row is still stamped created_by = caller.
-- ----------------------------------------------------------------------------
create or replace function create_workspace(ws_name text, ws_kind text default 'team')
returns workspaces
language plpgsql security definer set search_path = public as $$
declare
    v_uid uuid := auth.uid();
    v_row workspaces%rowtype;
    v_cap integer := project_lead_workspace_cap();
    v_cnt integer;
begin
    if v_uid is null then
        raise exception 'sign in first' using errcode = '28000';
    end if;
    if coalesce(btrim(ws_name), '') = '' then
        raise exception 'workspace name cannot be empty';
    end if;
    if ws_kind not in ('team', 'ev_startup', 'sandbox') then
        raise exception 'invalid workspace type: %', ws_kind;
    end if;
    -- is_project_lead already treats the owner as a lead; the bootstrap OR
    -- lets the first creation on a fresh, owner-less install through.
    if not (is_project_lead(v_uid) or _project_lead_bootstrap_open()) then
        raise exception
            'only the project owner or a registered project lead can create workspaces — ask your project lead for an invite link'
            using errcode = '42501';
    end if;
    v_cnt := _project_lead_ws_count(v_uid);
    if v_cnt >= v_cap then
        raise exception
            'workspace limit reached (% of %). Remove an unused workspace before creating another.',
            v_cnt, v_cap using errcode = '54023';
    end if;

    insert into workspaces (name, kind, created_by)
    values (btrim(ws_name), ws_kind, v_uid)
    returning * into v_row;
    -- _ws_owner_bootstrap (workspace_isolation.sql) enrolls the creator as
    -- owner; _kinematik_provision_shared_ws (shared_scope.sql) sets up the
    -- ecosystem's shared scope. Both fire on this insert.
    return v_row;
end $$;

revoke all on function create_workspace(text, text)    from public, anon;
grant execute on function create_workspace(text, text) to authenticated;

-- ----------------------------------------------------------------------------
--  5. Status snapshot for the UI: is the caller a lead, how many workspaces
--     they have, the cap, and whether they can create another right now.
-- ----------------------------------------------------------------------------
create or replace function project_lead_status()
returns table (is_lead boolean, workspace_count integer,
               workspace_cap integer, can_create boolean, is_owner boolean)
language plpgsql stable security definer set search_path = public as $$
declare v_uid uuid := auth.uid(); v_lead boolean; v_cnt integer; v_cap integer;
        v_owner boolean;
begin
    if v_uid is null then
        return query select false, 0, project_lead_workspace_cap(), false, false;
        return;
    end if;
    v_owner := is_owner();
    v_lead  := is_project_lead(v_uid);
    v_cnt   := _project_lead_ws_count(v_uid);
    v_cap   := project_lead_workspace_cap();
    return query select v_lead, v_cnt, v_cap, (v_lead and v_cnt < v_cap), v_owner;
end $$;

revoke all on function project_lead_status()    from public;
grant execute on function project_lead_status() to authenticated;

-- ----------------------------------------------------------------------------
--  6. Defence in depth: tighten ws_insert so a NON-lead cannot insert a
--     workspace by hitting the table directly (bypassing the RPC). The
--     shared-scope workspace is inserted by a SECURITY DEFINER trigger, which
--     is not subject to RLS, so this does not block shared-scope provisioning.
--
--     Existing policy (workspace_isolation.sql):
--         with check (created_by = auth.uid())
--     New:
--         with check (created_by = auth.uid() and is_project_lead(auth.uid()))
-- ----------------------------------------------------------------------------
--     is_project_lead(auth.uid()) already returns true for the owner (implicit
--     lead), so this covers owner + registered leads; the bootstrap OR keeps a
--     fresh, owner-less install from locking everyone out (see section 3).
-- ----------------------------------------------------------------------------
drop policy if exists ws_insert on workspaces;
create policy ws_insert on workspaces for insert to authenticated
    with check (
        created_by = auth.uid()
        and (is_project_lead(auth.uid()) or _project_lead_bootstrap_open())
    );

commit;

-- ----------------------------------------------------------------------------
--  POST-INSTALL
--    1. Name yourself the owner (survives restarts; use your DB name if not
--       'postgres', e.g. on Supabase run this in the SQL editor):
--          alter database postgres set app.owner_emails = 'you@example.com';
--       Reconnect (or bounce the pooler) so the setting takes effect. You are
--       now a project lead implicitly and can create workspaces + appoint leads.
--    2. Appoint the subsystem-lead accounts that should be able to create their
--       OWN workspaces (ordinary members never need this — they join by invite):
--          select promote_project_lead('lead@example.com');
--
--  Backfill note (run MANUALLY if you already have live workspaces):
--    Existing creators predate the project_leads table. Mark every current
--    creator as a lead so they keep managing their own ecosystem after this
--    migration (skip anyone you'd rather demote). Uncomment to apply:
--
--  insert into project_leads (user_id)
--  select distinct created_by from workspaces where created_by is not null
--    and coalesce(is_shared_scope, false) = false
--  on conflict (user_id) do nothing;
-- ----------------------------------------------------------------------------
