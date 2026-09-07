-- ============================================================================
--  KinematiK — Ecosystem-shared scope migration
--  (Lead Notes · Integration ledger · Team CAD library shared across ALL
--   workspaces created by ONE project lead; nothing crosses to another lead's
--   ecosystem.)
--
--  Idempotent: safe to re-run. Run AFTER workspace_isolation.sql (and, if you
--  use them, workspace_members_rpc.sql / workspace_invites.sql /
--  workspace_oversight.sql — order relative to those does not matter).
--
--  ECOSYSTEM = created_by
--    A project lead's `auth.uid()` is stamped on every workspace they create
--    (workspaces.created_by, set by ws_insert's WITH CHECK). Leads they invite
--    join those workspaces but do NOT create their own, so every workspace in
--    one lead's ecosystem shares the same created_by. That is the grouping key
--    — no new "org" table is needed.
--
--    Per ecosystem we designate ONE hidden shared workspace (kind='sandbox',
--    flagged is_shared_scope) that holds the three shared surfaces. Every
--    member of ANY workspace in the ecosystem can read/write it; members of a
--    different lead's ecosystem cannot see it at all.
--
--  This migration:
--    1. adds workspaces.is_shared_scope
--    2. a helper: the caller's ecosystem shared workspace id
--    3. auto-provisioning: a lead's first workspace also creates their shared
--       workspace and enrolls them; every existing/joining member is enrolled
--    4. backfill for workspaces that already exist
--    5. RLS so all ecosystem members reach the shared workspace's project row
--    6. the kinematik_shared_workspace(uuid) RPC the app calls
-- ============================================================================

begin;

-- ----------------------------------------------------------------------------
--  1. Flag column
-- ----------------------------------------------------------------------------
alter table workspaces
    add column if not exists is_shared_scope boolean not null default false;

-- At most one shared workspace per ecosystem (per creator).
create unique index if not exists uq_shared_ws_per_creator
    on workspaces (created_by) where is_shared_scope;

-- ----------------------------------------------------------------------------
--  2. Helpers (SECURITY DEFINER: read membership/creator without tripping RLS)
-- ----------------------------------------------------------------------------

-- The creator (ecosystem key) of a given workspace.
create or replace function kinematik_ecosystem_of(ws uuid)
returns uuid language sql stable security definer set search_path = public as $$
    select created_by from workspaces where id = ws;
$$;

-- The shared workspace id for the ecosystem the CALLER belongs to via `ws`.
-- Returns NULL if the caller isn't a member of `ws` (fail closed) or the
-- ecosystem has no shared workspace yet.
create or replace function kinematik_shared_workspace(for_workspace uuid)
returns uuid language plpgsql stable security definer set search_path = public as $$
declare
    v_creator uuid;
    v_shared  uuid;
begin
    -- Caller must actually be a member of the workspace they're asking about.
    if not is_workspace_member(for_workspace) then
        return null;
    end if;
    select created_by into v_creator from workspaces where id = for_workspace;
    if v_creator is null then
        return null;
    end if;
    select id into v_shared
    from workspaces
    where created_by = v_creator and is_shared_scope
    limit 1;
    return v_shared;   -- may be null if not provisioned; app fails closed
end $$;

revoke all on function kinematik_ecosystem_of(uuid)      from public;
grant execute on function kinematik_ecosystem_of(uuid)   to authenticated;
revoke all on function kinematik_shared_workspace(uuid)   from public;
grant execute on function kinematik_shared_workspace(uuid) to authenticated;

-- Membership test that also succeeds for ANY member of the SAME ecosystem as
-- the target workspace — used only by the shared-scope RLS policies below, so
-- it never widens access to ordinary per-workspace rows.
create or replace function kinematik_shares_ecosystem(ws uuid)
returns boolean language sql stable security definer set search_path = public as $$
    select exists (
        select 1
        from workspaces target
        join workspaces sibling on sibling.created_by = target.created_by
        join workspace_members m on m.workspace_id = sibling.id
        where target.id = ws
          and target.is_shared_scope           -- only the shared row opens up
          and m.user_id = auth.uid()
    );
$$;
revoke all on function kinematik_shares_ecosystem(uuid)    from public;
grant execute on function kinematik_shares_ecosystem(uuid) to authenticated;

-- ----------------------------------------------------------------------------
--  3. Auto-provisioning
--     When a project lead creates their FIRST real workspace, also create the
--     ecosystem's shared workspace and enroll the creator as owner of it. The
--     shared workspace re-fires this trigger; the is_shared_scope guard lets it
--     pass through. Every workspace the lead creates thereafter finds the
--     shared workspace already there and skips creation.
--
--     Invited leads/members are enrolled into the shared workspace by a second
--     trigger on workspace_members: joining ANY workspace in the ecosystem
--     grants shared-workspace membership too (so their reads/writes to the
--     shared project row pass RLS directly, not only via the RPC helper).
-- ----------------------------------------------------------------------------
create or replace function _kinematik_provision_shared_ws()
returns trigger language plpgsql security definer set search_path = public as $$
declare v_shared uuid;
begin
    if new.is_shared_scope then
        return new;                       -- this IS the shared ws; don't recurse
    end if;
    if new.created_by is null then
        return new;
    end if;
    select id into v_shared from workspaces
    where created_by = new.created_by and is_shared_scope limit 1;
    if v_shared is null then
        insert into workspaces (name, kind, created_by, is_shared_scope)
        values ('(shared scope)', 'sandbox', new.created_by, true)
        returning id into v_shared;
        -- creator owns the shared ws (the _ws_owner_bootstrap trigger from
        -- workspace_isolation.sql also fires and enrolls them as owner; this
        -- insert is a belt-and-braces no-op under its ON CONFLICT).
        insert into workspace_members (workspace_id, user_id, role)
        values (v_shared, new.created_by, 'owner')
        on conflict do nothing;
    end if;
    return new;
end $$;
drop trigger if exists trg_kinematik_provision_shared_ws on workspaces;
create trigger trg_kinematik_provision_shared_ws
    after insert on workspaces
    for each row execute function _kinematik_provision_shared_ws();

-- Enroll every workspace member into their ecosystem's shared workspace.
create or replace function _kinematik_enroll_shared_ws()
returns trigger language plpgsql security definer set search_path = public as $$
declare v_creator uuid; v_shared uuid;
begin
    select created_by into v_creator from workspaces where id = new.workspace_id;
    if v_creator is null then
        return new;
    end if;
    select id into v_shared from workspaces
    where created_by = v_creator and is_shared_scope limit 1;
    if v_shared is not null and v_shared <> new.workspace_id then
        insert into workspace_members (workspace_id, user_id, role)
        values (v_shared, new.user_id,
                case when new.role = 'viewer' then 'viewer' else 'member' end)
        on conflict (workspace_id, user_id) do nothing;
    end if;
    return new;
end $$;
drop trigger if exists trg_kinematik_enroll_shared_ws on workspace_members;
create trigger trg_kinematik_enroll_shared_ws
    after insert on workspace_members
    for each row execute function _kinematik_enroll_shared_ws();

-- ----------------------------------------------------------------------------
--  4. Backfill existing ecosystems (run once; harmless to re-run)
--     For every distinct creator that has real workspaces but no shared one,
--     create the shared workspace and enroll all their current members.
-- ----------------------------------------------------------------------------
do $$
declare r record; v_shared uuid;
begin
    for r in (
        select distinct created_by
        from workspaces
        where created_by is not null and not is_shared_scope
          and created_by not in (select created_by from workspaces
                                 where is_shared_scope and created_by is not null)
    ) loop
        insert into workspaces (name, kind, created_by, is_shared_scope)
        values ('(shared scope)', 'sandbox', r.created_by, true)
        returning id into v_shared;

        insert into workspace_members (workspace_id, user_id, role)
        select v_shared, m.user_id,
               case when bool_or(m.role = 'owner') then 'owner'
                    when bool_or(m.role = 'viewer') and not bool_or(m.role <> 'viewer')
                         then 'viewer'
                    else 'member' end
        from workspace_members m
        join workspaces w on w.id = m.workspace_id
        where w.created_by = r.created_by and not w.is_shared_scope
        group by m.user_id
        on conflict (workspace_id, user_id) do nothing;
    end loop;
end $$;

-- ----------------------------------------------------------------------------
--  5. RLS on the shared project row
--     The shared workspace's kinematik_workspace_projects row must be
--     reachable by EVERY member of the ecosystem. The existing kwp_* policies
--     already grant this to direct members of the shared workspace — and step
--     3/4 make every ecosystem member a direct member — so the base policies
--     already cover it. We ADD ecosystem-scoped policies as defence in depth
--     (works even if the enrollment trigger is disabled), OR-combined with the
--     existing ones by PostgreSQL's permissive-policy semantics.
-- ----------------------------------------------------------------------------
drop policy if exists kwp_shared_select on kinematik_workspace_projects;
create policy kwp_shared_select on kinematik_workspace_projects for select
    using (kinematik_shares_ecosystem(workspace_id));

drop policy if exists kwp_shared_write on kinematik_workspace_projects;
create policy kwp_shared_write on kinematik_workspace_projects for insert
    with check (kinematik_shares_ecosystem(workspace_id));

drop policy if exists kwp_shared_update on kinematik_workspace_projects;
create policy kwp_shared_update on kinematik_workspace_projects for update
    using (kinematik_shares_ecosystem(workspace_id))
    with check (kinematik_shares_ecosystem(workspace_id));

-- Let ecosystem members SELECT the shared workspace row itself (name/id
-- lookups, pickers). No insert/update — provisioning is trigger-only.
drop policy if exists ws_shared_visible on workspaces;
create policy ws_shared_visible on workspaces for select
    using (is_shared_scope and kinematik_shares_ecosystem(id));

commit;

-- ----------------------------------------------------------------------------
--  6. Hide the shared-scope workspace from the oversight panel too.
--     workspace_overview() (workspace_oversight.sql) lists every workspace the
--     caller owns/leads — which includes the ecosystem's hidden shared-scope
--     workspace, since the lead owns it. That workspace is plumbing, not a
--     place anyone works, so it must not appear there any more than it appears
--     in the picker (auth.py::list_workspaces already filters it out). We
--     redefine the function with the SAME body plus a single `not
--     w.is_shared_scope` guard. Wrapped in a guard so this is a no-op on
--     deployments that never installed workspace_oversight.sql.
-- ----------------------------------------------------------------------------
do $$
begin
    if to_regprocedure('public.workspace_overview()') is null then
        return;   -- oversight layer not installed; nothing to patch
    end if;

    create or replace function workspace_overview()
    returns table (
        workspace_id   uuid,
        name           text,
        kind           text,
        my_role        text,
        member_count   integer,
        owner_email    text,
        lead_emails    text[],
        member_emails  text[],
        viewer_emails  text[],
        last_activity  timestamptz,
        last_saved_by  text,
        saves_7d       integer
    )
    language plpgsql stable security definer set search_path = public as $body$
    declare has_history boolean := to_regclass('public.kinematik_project_history') is not null;
    begin
        return query
        with mine as (
            select m.workspace_id, m.role as my_role
            from workspace_members m
            where m.user_id = auth.uid() and m.role in ('owner', 'lead')
        ),
        roster as (
            select m.workspace_id,
                   count(*)::integer                                   as member_count,
                   max(u.email::text) filter (where m.role = 'owner')  as owner_email,
                   coalesce(array_agg(u.email::text order by u.email)
                            filter (where m.role = 'lead'),   '{}')    as lead_emails,
                   coalesce(array_agg(u.email::text order by u.email)
                            filter (where m.role = 'member'), '{}')    as member_emails,
                   coalesce(array_agg(u.email::text order by u.email)
                            filter (where m.role = 'viewer'), '{}')    as viewer_emails
            from workspace_members m
            join auth.users u on u.id = m.user_id
            where m.workspace_id in (select mine.workspace_id from mine)
            group by m.workspace_id
        ),
        proj as (
            select p.workspace_id,
                   max(p.updated_at)                                as cur_updated,
                   (array_agg(p.data->>'saved_by'
                              order by p.updated_at desc))[1]       as cur_saved_by
            from kinematik_workspace_projects p
            where p.workspace_id in (select mine.workspace_id from mine)
            group by p.workspace_id
        ),
        hist as (
            select h.workspace_id,
                   max(h.replaced_at)                               as last_snap,
                   count(*) filter (where h.replaced_at
                                           > now() - interval '7 days')::integer
                                                                    as saves_7d
            from kinematik_project_history h
            where has_history
              and h.workspace_id in (select mine.workspace_id from mine)
            group by h.workspace_id
        )
        select w.id, w.name::text, w.kind::text, mine.my_role::text,
               coalesce(roster.member_count, 0),
               roster.owner_email,
               coalesce(roster.lead_emails,   '{}'),
               coalesce(roster.member_emails, '{}'),
               coalesce(roster.viewer_emails, '{}'),
               greatest(proj.cur_updated, hist.last_snap),
               proj.cur_saved_by,
               coalesce(hist.saves_7d, 0)
        from mine
        join workspaces w on w.id = mine.workspace_id
        left join roster on roster.workspace_id = mine.workspace_id
        left join proj   on proj.workspace_id   = mine.workspace_id
        left join hist   on hist.workspace_id   = mine.workspace_id
        where not w.is_shared_scope            -- hide ecosystem shared scope
        order by lower(w.name);
    end $body$;
end $$;

-- ============================================================================
--  Optional (recommended) — auth_ui.py, ~4 lines in _render_workspace_picker
--  right after `ctx = auth.context_for(session, ws_id)`:
--
--      try:
--          _r = auth._user_client(session).rpc(
--              "kinematik_shared_workspace",
--              {"for_workspace": ws_id}).execute()
--          setattr(ctx, "shared_workspace_id", (_r.data or None))
--      except Exception:
--          pass
--
--  streamlit_app.py prefers ctx.shared_workspace_id when present, so this just
--  saves one RPC round-trip per session. WorkspaceContext is a plain (non-
--  frozen) dataclass, so setattr works.
-- ============================================================================
