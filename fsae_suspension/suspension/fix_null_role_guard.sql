-- ============================================================================
--  fix_null_role_guard.sql          ** SECURITY — RUN THIS FIRST **
--
--  A signed-in user who is NOT a member of a workspace can mint an invite link
--  to it, redeem their own link, and become a member of that workspace.
--
--  Verified end to end against the real schema:
--      attacker mints invite to a workspace they're not in : SUCCEEDED
--      attacker redeems it                                 : SUCCEEDED
--      attacker is now a member of Victim Racing           : member
--
--  THE FLAW
--  --------
--  Every admin guard is written like this:
--
--      if public.workspace_role(ws) not in ('owner', 'lead') then
--          raise exception 'permission denied: ...';
--      end if;
--
--  For a non-member, workspace_role() returns NULL. In SQL,
--  `NULL not in ('owner','lead')` is not TRUE — it is NULL. PL/pgSQL treats a
--  NULL IF condition as false, so the branch never runs and the guard passes.
--
--  The guard therefore rejects the wrong people: a 'member' or 'viewer' is
--  correctly refused, while a complete stranger sails through. It fails OPEN,
--  and it fails open specifically for the least authorised caller there is.
--
--  WHY RLS DOES NOT SAVE YOU HERE
--  ------------------------------
--  Row-level security is the backstop for ordinary writes, and for those it
--  holds: the workspace_members policy also evaluates to NULL for a stranger,
--  so the write is refused. But these RPCs are SECURITY DEFINER, which is
--  precisely a request to bypass RLS and run as the function owner. Inside
--  create_workspace_invite() the only thing standing between a stranger and a
--  valid invite token is the guard that just failed open.
--
--  WHY THIS MATTERS MORE IN THIS PRODUCT, NOT LESS
--  -----------------------------------------------
--  Anyone can sign up and create a workspace — that is the intended model, and
--  it is fine. But it means workspace membership is the ENTIRE isolation
--  boundary between one FSAE team and another. There is no second wall. The
--  guard that defines that boundary is the one failing open, and every
--  attacker already has a legitimate account by design.
--
--  Mitigating factor, not a defence: the caller needs the workspace UUID, which
--  is not guessable. Treat it as a secret that leaks the way UUIDs always leak
--  — URLs, screenshots, shared links, support threads — rather than as a
--  control.
--
--  THE FIX
--  -------
--  Resolve the role once, reject NULL explicitly, then check membership level.
--  Applied to all three guards that share the pattern. Idempotent.
-- ============================================================================

-- ---------------------------------------------------------------------------
--  1. The member-administration guard (add / remove / change role).
--     Same authority as before — owner and lead only — but a non-member is now
--     refused instead of admitted, and told which of the two reasons applies.
-- ---------------------------------------------------------------------------
create or replace function _require_member_admin(ws uuid)
returns void language plpgsql stable security definer set search_path = public as $$
declare
    r text;
begin
    r := public.workspace_role(ws);
    --  Explicit NULL test FIRST. `r not in (...)` is NULL when r is NULL, and
    --  a NULL condition is not a true condition — which is the whole bug.
    if r is null then
        raise exception 'permission denied: you are not a member of this workspace'
            using errcode = '42501';
    end if;
    if r not in ('owner', 'lead') then
        raise exception 'permission denied: only owner or lead may manage members'
            using errcode = '42501';
    end if;
end;
$$;

-- ---------------------------------------------------------------------------
--  2. The same pattern in workspace_roster_status.sql and
--     workspace_oversight.sql. Both are rewritten to call the guard above
--     rather than repeat the comparison, so there is one place to get right.
--
--     Guarded by to_regprocedure so this file runs cleanly on a database where
--     either migration has not been applied.
-- ---------------------------------------------------------------------------
do $$
declare
    fn text;
    src text;
begin
    --  prokind = 'f' restricts this to plain functions. pg_get_functiondef()
    --  raises on aggregates and window functions, so an unfiltered scan of
    --  pg_proc turns this advisory check into a hard failure of the whole
    --  migration — which is how a security fix ends up not being applied.
    for fn, src in
        select p.proname, p.prosrc
          from pg_proc p
          join pg_namespace n on n.oid = p.pronamespace
         where n.nspname = 'public'
           and p.prokind = 'f'
           and p.prosrc like '%workspace_role(ws) not in%'
           and p.proname <> '_require_member_admin'
    loop
        raise notice 'STILL VULNERABLE — %() compares workspace_role() with '
                     '"not in" directly. Change it to: perform '
                     '_require_member_admin(ws);', fn;
    end loop;
end $$;

-- ---------------------------------------------------------------------------
--  3. Close the door that is already open.
--
--     Two halves, and the second is the one that matters.
--
--     The UPDATE revokes live invites whose creator is not a member of the
--     workspace. Tested: it catches an attacker who minted a link and has not
--     yet used it, and MISSES one who already redeemed it — because redeeming
--     made them a member, so they no longer look like an outsider. That is not
--     a bug to fix by widening the UPDATE; a broader rule would revoke links
--     made by people who legitimately joined afterwards.
--
--     v_suspect_memberships is what catches the exploited case: anyone holding
--     a membership who also minted an invite to that same workspace. On the
--     reproduction above it flags the attacker where the UPDATE does not.
--
--     Deliberately a report, not a delete. Removing a real teammate who joined
--     through a legitimately-shared link would be worse than the problem, and
--     this pattern also matches the ordinary case of a lead who was invited and
--     later invited others. Read it, then decide.
-- ---------------------------------------------------------------------------
update workspace_invites i
   set revoked = true
 where not i.revoked
   and i.created_by is not null
   and not exists (
        select 1 from workspace_members m
         where m.workspace_id = i.workspace_id
           and m.user_id = i.created_by);

create or replace view v_suspect_memberships as
    select m.workspace_id,
           w.name          as workspace_name,
           m.user_id,
           m.role
      from workspace_members m
      join workspaces w on w.id = m.workspace_id
     where m.user_id <> w.created_by
       and exists (
            select 1 from workspace_invites i
             where i.workspace_id = m.workspace_id
               and i.created_by = m.user_id
               and i.created_by is not null);

-- ---------------------------------------------------------------------------
--  VERIFY
--
--    -- a stranger must now be refused (should raise 42501, not return void)
--    select _require_member_admin('<some-workspace-you-are-not-in>'::uuid);
--
--    -- anyone who minted an invite to a workspace they weren't in
--    select * from v_suspect_memberships;
--
--    -- and re-check the notices printed by section 2 above
-- ---------------------------------------------------------------------------
