-- ============================================================================
--  fix_invite_permission.sql
--  Fix: "permission denied: only owner or lead may manage members"
--       (SQLSTATE 42501) raised when an ordinary member clicks
--       "Create invite link".
--
--  WHAT HAPPENED
--  -------------
--  The rule was widened from "owner/lead may invite" to "anyone except a
--  viewer may invite" — but only on the client. `render_invite_admin()` in
--  auth_ui.py gates on `ctx.role == "viewer"` and tells the user, in the
--  caption, "Anyone on the team can make one." Its comment even says it
--  "mirrors create_workspace_invite() server-side, which is the actual
--  authority".
--
--  It does not mirror it. create_workspace_invite() still calls
--  _require_member_admin(), which permits only owner/lead. So a member sees
--  the button, is told they may use it, clicks it, and gets a raw Postgres
--  permission error. The production log shows exactly that: six 42501s in
--  sixty-five seconds — one person clicking again because nothing explained
--  why it failed.
--
--  This is the worst shape a permissions bug can take. It does not block an
--  attacker; it blocks the one user who was doing the thing we most want
--  (getting their team into the workspace), and it does so at the moment they
--  are trying to help.
--
--  THE DECISION
--  ------------
--  Match the server to the client, not the other way round, because the
--  client's rule is the better one:
--
--    * An invite can only ever grant 'member' or 'viewer' — already enforced
--      below and unchanged. A member inviting a member is not escalation.
--    * Invites expire (1h-30d), are use-capped (1-100), and are revocable by
--      owner/lead at any time.
--    * Viewers stay excluded: read-only access must not be able to mint
--      access it does not itself hold.
--
--  Administering EXISTING members — adding directly, removing others,
--  changing roles — stays owner/lead. That is a different power and
--  _require_member_admin() keeps guarding it, untouched.
--
--  Idempotent. Safe to run more than once, and safe to run before or after
--  any other migration in this directory.
-- ============================================================================

-- ---------------------------------------------------------------------------
--  New, narrower guard: may this caller mint an invite for this workspace?
--  Deliberately stated as an EXCLUSION (everyone but viewer) rather than an
--  allow-list of ('owner','lead','member'). An allow-list silently locks out
--  any role added later — which is precisely how "only leads can invite"
--  became true by accident the first time.
-- ---------------------------------------------------------------------------
create or replace function _require_member_inviter(ws uuid)
returns void language plpgsql stable security definer set search_path = public as $$
declare
    r text;
begin
    r := public.workspace_role(ws);
    if r is null then
        raise exception 'permission denied: you are not a member of this workspace'
            using errcode = '42501';
    end if;
    if r = 'viewer' then
        raise exception 'permission denied: viewers cannot create invite links'
            using errcode = '42501';
    end if;
end;
$$;

revoke all on function _require_member_inviter(uuid) from public, anon;
grant execute on function _require_member_inviter(uuid) to authenticated;

-- ---------------------------------------------------------------------------
--  create_workspace_invite: identical to the original except for the guard.
--  Everything else — the member/viewer-only role cap, the 30-day lifetime
--  ceiling, the 100-use cap — is unchanged and still enforced here rather
--  than trusted from the client.
-- ---------------------------------------------------------------------------
create or replace function create_workspace_invite(
        ws uuid, invite_role text default 'member',
        ttl_hours int default 168, uses int default 30)
returns uuid
language plpgsql security definer set search_path = public as $$
declare
    tok uuid;
begin
    perform _require_member_inviter(ws);         -- was _require_member_admin
    if invite_role not in ('member', 'viewer') then
        raise exception 'invite links can only grant member or viewer';
    end if;
    if ttl_hours < 1 or ttl_hours > 720 then     -- 30 days hard cap
        raise exception 'invite lifetime must be between 1 hour and 30 days';
    end if;
    if uses < 1 or uses > 100 then
        raise exception 'invite use cap must be between 1 and 100';
    end if;
    insert into workspace_invites (workspace_id, role, created_by,
                                   expires_at, max_uses)
    values (ws, invite_role, auth.uid(),
            now() + make_interval(hours => ttl_hours), uses)
    returning token into tok;
    return tok;
end;
$$;

-- ---------------------------------------------------------------------------
--  Listing and revoking your workspace's invite links.
--
--  Listing follows the same rule as creating: if you can mint a link you can
--  see the links that exist, otherwise the panel shows an empty list to the
--  very people it just invited to use it.
--
--  REVOKING stays owner/lead. Killing a link the whole team chat is using is
--  a destructive act on shared state, and it is not symmetric with creating
--  one.
-- ---------------------------------------------------------------------------
--  The body below is character-for-character the original from
--  workspace_invites.sql apart from the guard on the first line of it. That
--  matters: an earlier draft of this file quietly dropped the
--  `not revoked and expires_at > now()` filter and re-ordered the results,
--  which would have shown members a list of dead links. When the only
--  intended change is a permission, everything else must be copied, not
--  rewritten from memory.
create or replace function list_workspace_invites(ws uuid)
returns table (token uuid, role text, expires_at timestamptz,
               max_uses int, use_count int, revoked boolean)
language plpgsql stable security definer set search_path = public as $$
begin
    perform _require_member_inviter(ws);         -- was _require_member_admin
    return query
        select i.token, i.role, i.expires_at, i.max_uses, i.use_count, i.revoked
          from workspace_invites i
         where i.workspace_id = ws
           and not i.revoked
           and i.expires_at > now()
         order by i.created_at desc;
end;
$$;

-- ---------------------------------------------------------------------------
--  VERIFY
--  As a plain member of a workspace, this should now return a token instead
--  of raising 42501. As a viewer it should still raise, with a message that
--  says why rather than talking about managing members.
--
--     select create_workspace_invite('<workspace-uuid>'::uuid, 'member', 168, 30);
--
--  And confirm the admin path is untouched — adding or removing a member as
--  a plain member must still be refused:
--
--     select _require_member_admin('<workspace-uuid>'::uuid);
-- ---------------------------------------------------------------------------
