-- ============================================================================
--  KinematiK — Workspace oversight RPCs (owner/lead visibility layer)
--  Idempotent. Run AFTER workspace_isolation.sql, workspace_members_rpc.sql
--  and (optionally) project_history.sql.
--
--  WHAT THIS ADDS
--    workspace_overview()          one row per workspace where the CALLER is
--                                  'owner' or 'lead': how many people are
--                                  using it, who the owner is, who the
--                                  lead(s) are, the full roster split by
--                                  role, and when it was last touched.
--    workspace_activity(ws, lim)   the recent save trail for ONE workspace
--                                  (who saved, when) — owner/lead only.
--
--  WHY RPCs (same reasoning as workspace_members_rpc.sql): emails live in
--  auth.users, which the anon/user client rightly cannot read. These
--  SECURITY DEFINER functions do the join server-side, but each RE-CHECKS
--  the caller's role with the same rules RLS enforces — the definer
--  privilege is used only to read auth.users and the history table, never
--  to bypass the permission model:
--
--     workspace_overview  -> rows only for workspaces where caller is
--                            'owner' or 'lead' (members/viewers get nothing)
--     workspace_activity  -> caller must be 'owner' or 'lead' of that
--                            workspace, else 42501
--
--  project_history is OPTIONAL: both functions probe to_regclass() so a
--  deployment that never ran project_history.sql still gets member counts,
--  rosters and the current blob's timestamp — just no per-save trail.
-- ============================================================================

-- ----------------------------------------------------------------------------
--  Overview: every workspace the caller administers, with usage + roster.
--  "member_count" is the number of people who can use the workspace (all
--  roles). last_activity is the newest of the current project blob's
--  updated_at and the newest history snapshot; last_saved_by is the audit
--  stamp the app writes into the blob (workspace.py -> payload['saved_by']).
-- ----------------------------------------------------------------------------
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
language plpgsql stable security definer set search_path = public as $$
declare has_history boolean := to_regclass('public.kinematik_project_history') is not null;
begin
    return query
    with mine as (
        -- Only workspaces the CALLER administers ever leave this CTE; a
        -- plain member calling this RPC gets an empty set, not an error.
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
        -- Empty unless project_history.sql was installed.
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
    order by lower(w.name);
end $$;

-- ----------------------------------------------------------------------------
--  Activity trail for one workspace: newest first, one row per save event.
--  'current' is the live blob; 'snapshot' rows are prior versions captured by
--  the project_history trigger. saved_by is the app's audit stamp (email of
--  the saver), so a lead can see exactly who has been working in a workspace
--  they hand out — not just how many people COULD.
-- ----------------------------------------------------------------------------
create or replace function workspace_activity(ws uuid, lim integer default 25)
returns table (
    happened_at  timestamptz,
    saved_by     text,
    project_id   text,
    event        text
)
language plpgsql stable security definer set search_path = public as $$
declare has_history boolean := to_regclass('public.kinematik_project_history') is not null;
begin
    if public.workspace_role(ws) not in ('owner', 'lead') then
        raise exception
            'permission denied: only the owner or a lead may view workspace activity'
            using errcode = '42501';
    end if;

    return query
    (
        select p.updated_at, (p.data->>'saved_by')::text, p.id::text,
               'current'::text
        from kinematik_workspace_projects p
        where p.workspace_id = ws
        union all
        select h.replaced_at, (h.data->>'saved_by')::text, h.id::text,
               'snapshot'::text
        from kinematik_project_history h
        where has_history and h.workspace_id = ws
    )
    order by 1 desc nulls last
    limit greatest(1, least(coalesce(lim, 25), 200));
end $$;

-- ----------------------------------------------------------------------------
--  Privileges: authenticated may call; anon may not. The role checks inside
--  each function are the real gate; this keeps anon off the RPC surface.
-- ----------------------------------------------------------------------------
revoke all on function workspace_overview()               from anon;
revoke all on function workspace_activity(uuid, integer)  from anon;
grant execute on function workspace_overview()              to authenticated;
grant execute on function workspace_activity(uuid, integer) to authenticated;
