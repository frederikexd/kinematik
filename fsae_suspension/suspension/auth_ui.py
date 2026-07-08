# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  Module: auth_ui — Streamlit sign-in gate + workspace picker.
#
#  This renders the login screen and, once signed in, the workspace chooser,
#  then stashes the resulting WorkspaceContext in st.session_state so
#  streamlit_app.get_store() can build a tenant-scoped store from it.
#
#  Design: everything degrades gracefully.
#    * No Supabase configured  -> returns None; the app runs in its old local
#      single-user JSON mode (unchanged behaviour for laptops / tests).
#    * Supabase configured      -> a signed-in user + selected workspace is
#      required before the rest of the app renders. That is the tenant wall.
# ============================================================================

from __future__ import annotations

from typing import Optional

from .auth import AuthError, SupabaseAuth, Session, build_auth
from .workspace import WorkspaceContext


_SS_SESSION = "_kx_auth_session"        # dict of cached tokens
_SS_CTX = "_kx_workspace_ctx"           # the active WorkspaceContext
_SS_AUTH = "_kx_auth_client"            # cached SupabaseAuth instance


def _get_auth(st) -> Optional[SupabaseAuth]:
    """Cache the SupabaseAuth client in session_state (constructing it makes a
    network client; no need to rebuild every rerun)."""
    auth = st.session_state.get(_SS_AUTH)
    if auth is None:
        try:
            auth = build_auth()
        except AuthError as e:
            st.error(f"Auth configuration problem: {e}")
            st.stop()
        st.session_state[_SS_AUTH] = auth
    return auth


def _restore_session(st, auth: SupabaseAuth) -> Optional[Session]:
    """Rebuild a live Session from cached tokens across reruns, so the user
    isn't asked to log in on every interaction."""
    cached = st.session_state.get(_SS_SESSION)
    if not cached:
        return None
    try:
        return auth.restore(cached.get("access_token", ""),
                            cached.get("refresh_token", ""))
    except AuthError:
        st.session_state.pop(_SS_SESSION, None)
        st.session_state.pop(_SS_CTX, None)
        return None


def _render_sign_in(st, auth: SupabaseAuth) -> None:
    st.markdown("### Sign in to KinematiK")
    st.caption("Your project data is isolated per workspace. Sign in to continue.")
    mode = st.radio("mode", ["Sign in", "Create account"],
                    horizontal=True, label_visibility="collapsed")
    email = st.text_input("Email", key="_kx_email")
    password = st.text_input("Password", type="password", key="_kx_pw")
    if st.button(mode, type="primary"):
        if not email or not password:
            st.warning("Enter an email and password.")
            return
        try:
            if mode == "Create account":
                session = auth.sign_up(email, password)
            else:
                session = auth.sign_in(email, password)
        except AuthError as e:
            st.error(str(e))
            return
        st.session_state[_SS_SESSION] = {
            "access_token": session.access_token,
            "refresh_token": session.refresh_token,
        }
        st.rerun()


def _render_workspace_picker(st, auth: SupabaseAuth, session: Session
                             ) -> Optional[WorkspaceContext]:
    try:
        workspaces = auth.list_workspaces(session)
    except AuthError as e:
        st.error(str(e))
        if st.button("Sign out"):
            _sign_out(st, auth)
        return None

    with st.sidebar:
        st.caption(f"Signed in as **{session.email}**")
        if st.button("Sign out", key="_kx_signout"):
            _sign_out(st, auth)

    if not workspaces:
        st.info("You are not a member of any workspace yet. Create one to start.")
        name = st.text_input("New workspace name", key="_kx_new_ws")
        kind = st.selectbox("Type", ["team", "ev_startup", "sandbox"],
                            key="_kx_new_ws_kind")
        if st.button("Create workspace", type="primary"):
            try:
                ws = auth.create_workspace(session, name, kind=kind)
            except AuthError as e:
                st.error(str(e))
                return None
            st.session_state["_kx_ws_id"] = ws.id
            st.rerun()
        return None

    labels = {f"{ws.name}  ·  {role}": ws.id for ws, role in workspaces}
    preselect = st.session_state.get("_kx_ws_id")
    keys = list(labels.keys())
    index = 0
    if preselect:
        for i, (lbl, wid) in enumerate(labels.items()):
            if wid == preselect:
                index = i
                break
    with st.sidebar:
        choice = st.selectbox("Workspace", keys, index=index, key="_kx_ws_choice")
    ws_id = labels[choice]
    st.session_state["_kx_ws_id"] = ws_id

    try:
        ctx = auth.context_for(session, ws_id)
    except AuthError as e:
        st.error(str(e))
        return None

    # Members-admin lives in the sidebar next to the workspace picker, so it's
    # always reachable regardless of which tab the user is on.
    with st.sidebar:
        with st.expander("Members", expanded=False):
            render_members_admin(st, ctx)
    return ctx


def _sign_out(st, auth: SupabaseAuth) -> None:
    try:
        auth.sign_out()
    finally:
        for k in (_SS_SESSION, _SS_CTX, "_kx_ws_id", "_project_store"):
            st.session_state.pop(k, None)
        st.rerun()


def current_session(st) -> Optional[Session]:
    """The live signed-in Session for this run, or None in local mode / signed
    out. Rebuilt from cached tokens; used by the members-admin panel."""
    auth = st.session_state.get(_SS_AUTH)
    if auth is None:
        return None
    return _restore_session(st, auth)


_ADMIN_ROLES = ("lead", "member")   # roles this UI hands out (owner is implicit)


def render_members_admin(st, ctx: WorkspaceContext) -> None:
    """
    Members-admin panel for the active workspace. Renders the roster and, for
    owners/leads, controls to add members by email, change roles, and remove
    members. Every action is enforced again in the database, so a viewer/member
    who somehow reaches these controls still can't mutate anything.

    Drop this anywhere in the app body, e.g. inside a 'Team' tab:
        from suspension import auth_ui
        if _workspace_ctx:
            auth_ui.render_members_admin(st, _workspace_ctx)
    """
    auth = st.session_state.get(_SS_AUTH)
    session = current_session(st)
    if auth is None or session is None:
        st.info("Member management is available when signed in to a workspace.")
        return

    is_admin = ctx.role in ("owner", "lead")

    st.subheader(f"Members · {ctx.workspace.name}")
    if not is_admin:
        st.caption("You can view the roster. Only an owner or lead can make changes.")

    # --- roster ---------------------------------------------------------- #
    try:
        members = auth.list_members(session, ctx.workspace_id)
    except AuthError as e:
        st.error(str(e))
        return

    if not members:
        st.caption("No members found.")
    for m in members:
        uid = str(m.get("user_id", ""))
        email = m.get("email", "(unknown)")
        role = m.get("role", "member")
        is_self = uid == session.user_id
        is_owner_row = role == "owner"

        cols = st.columns([5, 3, 2]) if is_admin else st.columns([7, 3])
        cols[0].markdown(f"**{email}**" + (" · _you_" if is_self else ""))

        if is_admin and not is_owner_row:
            # Role selector (owner rows are fixed; owners aren't re-roled here).
            new_role = cols[1].selectbox(
                "role", _ADMIN_ROLES,
                index=_ADMIN_ROLES.index(role) if role in _ADMIN_ROLES else 1,
                key=f"_kx_role_{uid}", label_visibility="collapsed")
            if new_role != role:
                if cols[1].button("Update", key=f"_kx_role_btn_{uid}"):
                    try:
                        auth.set_member_role(session, ctx.workspace_id, uid, new_role)
                        st.success(f"{email} is now {new_role}.")
                        st.rerun()
                    except AuthError as e:
                        st.error(str(e))
            if cols[2].button("Remove", key=f"_kx_rm_{uid}"):
                try:
                    auth.remove_member(session, ctx.workspace_id, uid)
                    st.success(f"Removed {email}.")
                    st.rerun()
                except AuthError as e:
                    st.error(str(e))
        else:
            (cols[1] if is_admin else cols[1]).markdown(f"`{role}`")

    # --- add member ------------------------------------------------------ #
    if is_admin:
        st.divider()
        st.markdown("**Add a member**")
        st.caption("They must already have a KinematiK account (same email).")
        ac = st.columns([5, 3, 2])
        new_email = ac[0].text_input("Email", key="_kx_add_email",
                                     label_visibility="collapsed",
                                     placeholder="teammate@university.edu")
        new_role = ac[1].selectbox("Role", _ADMIN_ROLES, index=1,
                                   key="_kx_add_role", label_visibility="collapsed")
        if ac[2].button("Add", type="primary", key="_kx_add_btn"):
            try:
                auth.add_member(session, ctx.workspace_id, new_email, role=new_role)
                st.success(f"Added {new_email} as {new_role}.")
                st.rerun()
            except AuthError as e:
                st.error(str(e))

    # --- leave workspace ------------------------------------------------- #
    if ctx.role != "owner":
        st.divider()
        if st.button("Leave this workspace", key="_kx_leave"):
            try:
                auth.remove_member(session, ctx.workspace_id, session.user_id)
                st.session_state.pop("_kx_ws_id", None)
                st.session_state.pop(_SS_CTX, None)
                st.session_state.pop("_project_store", None)
                st.rerun()
            except AuthError as e:
                st.error(str(e))


def require_workspace(st) -> Optional[WorkspaceContext]:
    """
    The gate. Call at the top of the app, before rendering any tenant data.

    Returns:
        * WorkspaceContext  — Supabase configured, user signed in, workspace
          chosen. Store this and build a tenant-scoped store from it.
        * None              — Supabase NOT configured: run in legacy local
          single-user mode (caller falls back to ProjectStore(path)).

    When Supabase IS configured but the user isn't signed in / hasn't picked a
    workspace, this renders the sign-in or picker UI and calls st.stop() so the
    rest of the app never renders another tenant's (or no tenant's) data.
    """
    auth = _get_auth(st)
    if auth is None:
        # No cloud backend configured — legacy local mode, no gate.
        return None

    session = _restore_session(st, auth)
    if session is None:
        _render_sign_in(st, auth)
        st.stop()

    ctx = _render_workspace_picker(st, auth, session)
    if ctx is None:
        st.stop()

    st.session_state[_SS_CTX] = ctx
    return ctx
