# Ecosystem-shared scope — Lead Notes · Integration ledger · CAD library

## The model

A project lead who creates workspaces and hands invite links to their
subsystem leads is one ECOSYSTEM. Inside it, Lead Notes, the Integration
ledger, and the Team CAD library are shared across ALL of that lead's
workspaces. A different lead's workspaces form a separate ecosystem — nothing
crosses between them, enforced by Postgres RLS, not just client routing.
Everything else (geometry, weights, handover decisions, pedal inputs) stays
strictly per-workspace, exactly as before.

Ecosystem key = workspaces.created_by. The project lead's auth.uid() is
stamped on every workspace they create (the existing ws_insert policy's
WITH CHECK (created_by = auth.uid())). Invited leads join those workspaces but
don't create their own, so one lead's whole ecosystem shares a single
created_by. No new "org" table is needed. Per ecosystem, one hidden shared
workspace (kind='sandbox', is_shared_scope=true) holds the three surfaces.

## Files

* suspension/shared_scope.sql — the migration (schema flag, helpers,
  auto-provisioning + enrollment triggers, backfill, RLS, and the
  kinematik_shared_workspace(uuid) RPC).
* streamlit_app.py — the client routing.

## Deployment order of the SQL

Run in this order (or via sql/run_all.sql, appending shared_scope.sql at its end):

  1. suspension/workspace_isolation.sql    (tenancy spine — already deployed if
                                            workspaces exist)
  2. suspension/workspace_members_rpc.sql  (if used)
  3. suspension/workspace_invites.sql      (if used — invite links)
  4. suspension/workspace_oversight.sql    (if used)
  5. suspension/shared_scope.sql   <-- NEW, run LAST

shared_scope.sql only depends on #1 (it calls is_workspace_member and reads
workspaces / workspace_members); its position relative to #2-#4 doesn't
matter, but running it last is simplest. Idempotent — safe to re-run.

## App deployment order

  1. Run the SQL above (safe before OR after login goes live; the client fails
     closed until the RPC answers).
  2. Optionally add the 4-line ctx.shared_workspace_id stash to
     auth_ui._render_workspace_picker (saves one RPC per session; see the tail
     of shared_scope.sql).
  3. Ship streamlit_app.py. With ENABLE_LOGIN still off, nothing changes.

## Client routing (streamlit_app.py — done)

* _resolve_shared_workspace_id(): (1) ctx.shared_workspace_id if stashed at
  sign-in, else (2) KINEMATIK_SHARED_WORKSPACE_ID env/secret override, else
  (3) the kinematik_shared_workspace RPC via the backend's Supabase .client.
  Cached per session; unresolved -> FAILS CLOSED to per-workspace scope.
* _shared_workspace_ctx(): builds a NEW WorkspaceContext around
  Workspace(id=<shared>), carrying the same signed-in identity so RLS still
  sees the human (workspace_id is a read-only property, so a fresh context is
  required — not a mutation).
* _make_shared_store() / get_shared_store() mirror _make_store()/get_store().
* Rerouted to the shared store: the whole Lead Notes tab, the 2 s poller, the
  TEAM FIT auto-notify note (its Decision stays per-workspace), the Docs tab's
  cross-team-notes section, publish_ledger + ledger seeding + the save_store
  catch-all sync, the electrics-workbook ledger re-save, and the entire Team
  CAD library (incl. torsion-deck/SES writes).
* Restore guard: with sharing active, Load project + migration restore skip
  the bundle's ledger unless a checkbox (default OFF) is ticked.
* shared_scope_active() drives honest UI copy.

## Server side (shared_scope.sql)

1. workspaces.is_shared_scope flag + one-shared-per-creator unique index.
2. Helpers kinematik_ecosystem_of, kinematik_shared_workspace (the RPC),
   kinematik_shares_ecosystem.
3. Triggers: provision the shared workspace on a lead's first workspace;
   enroll every workspace member into their ecosystem's shared workspace.
4. Backfill for existing workspaces.
5. RLS: ecosystem-scoped select/insert/update on the shared workspace's
   kinematik_workspace_projects row + select on the shared workspace row,
   OR-combined with the existing per-workspace policies.

Written against the actual schema in workspace_isolation.sql. No [ADAPT]
placeholders remain.

## Caveats

* Cross-workspace optimistic-lock conflicts on the shared row are possible;
  the existing "a teammate saved a newer version" recovery handles them.
* Read receipts, unread badges, and resolved status are ecosystem-wide by
  design.
* The shared workspace holds a full project row but only its notes / cad_files
  / ledger fields are ever read; don't point a normal session at it.
