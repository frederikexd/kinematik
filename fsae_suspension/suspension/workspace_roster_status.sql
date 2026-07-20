-- ============================================================================
--  KinematiK — Per-member sign-up / activity status for oversight
--  Idempotent: safe to re-run. Run AFTER workspace_isolation.sql,
--  workspace_members_rpc.sql and workspace_oversight.sql.
--
--  WHY
--    Owners and appointed leads need to see, per workspace, WHO has actually
--    signed up and started working versus who is just a name on the roster.
--
--    In this system a person is on the roster (workspace_members) only once
--    they already have an account — both add_workspace_member and the
--    invite-link redemption insert the membership row against an existing
--    auth.users id. So "has an account" is always true for roster rows; the
--    meaningful signal is whether they've ever SIGNED IN and whether they've
--    DONE anything (a save). Invite links are anonymous (not tied to an
--    email), so there is deliberately no per-person "invited but not yet
--    signed up" row to show — an unredeemed link is tracked by
--    list_workspace_invites (use_count/max_uses), not here.
--
--  WHAT THIS ADDS
--    workspace_roster_status(ws) -> one row per member with:
--        user_id, email, role,
--        signed_up      (bool)  — have they ever signed in at all?
--        last_sign_in_at(ts)    — when, if so
--        joined_at      (ts)    — when they were added / redeemed the invite
--        last_saved_at  (ts)    — their most recent save in THIS workspace,
--                                 matched by the app's saved_by email stamp
--        active         (bool)  — signed in AND has at least one save here
--
--    Owner/lead only (same gate as the rest of oversight); re-checked
--    server-side, so a plain member calling it gets a permission error.
-- ============================================================================

begin;

create or replace function workspace_roster_status(ws uuid)
returns table (
    user_id         uuid,
    email           text,
    role            text,
    signed_up       boolean,
    last_sign_in_at timestamptz,
    joined_at       timestamptz,
    last_saved_at   timestamptz,
    active          boolean
)
language plpgsql stable security definer set search_path = public as $$
declare has_history boolean := to_regclass('public.kinematik_project_history') is not null;
begin
    if public.workspace_role(ws) not in ('owner', 'lead') then
        raise exception
            'permission denied: only the owner or a lead may view roster status'
            using errcode = '42501';
    end if;

    return query
    with saves as (
        -- Most recent save timestamp per saver email in THIS workspace.
        -- The app stamps data->>'saved_by' with the saver's email, so we
        -- match on lower(email). Current blob + history (if installed).
        select lower(x.saved_by) as saved_by, max(x.ts) as last_saved_at
        from (
            select p.data->>'saved_by' as saved_by, p.updated_at as ts
            from kinematik_workspace_projects p
            where p.workspace_id = ws
            union all
            select h.data->>'saved_by' as saved_by, h.replaced_at as ts
            from kinematik_project_history h
            where has_history and h.workspace_id = ws
        ) x
        where coalesce(x.saved_by, '') <> ''
        group by lower(x.saved_by)
    )
    select
        m.user_id,
        u.email::text                                   as email,
        m.role,
        (u.last_sign_in_at is not null)                 as signed_up,
        u.last_sign_in_at,
        m.added_at                                      as joined_at,
        s.last_saved_at,
        (u.last_sign_in_at is not null
         and s.last_saved_at is not null)               as active
    from workspace_members m
    join auth.users u on u.id = m.user_id
    left join saves s on s.saved_by = lower(u.email::text)
    where m.workspace_id = ws
    order by
        case m.role when 'owner' then 0 when 'lead' then 1
                    when 'member' then 2 else 3 end,
        u.email;
end $$;

revoke all on function workspace_roster_status(uuid) from anon;
grant execute on function workspace_roster_status(uuid) to authenticated;

commit;
