"""Durable anonymous visitor id via browser localStorage.

WHY THIS EXISTS
---------------
On Streamlit Community Cloud, the two identity mechanisms KinematiK previously
relied on both fail for most anonymous users:

  * Cookies (extra_streamlit_components CookieManager) don't persist reliably in
    the sandboxed component iframe — the app's own logs noted "44 sessions
    produced 43 distinct ck- ids", i.e. not durable.
  * IP+UA fingerprint needs st.context.ip_address, which the Cloud proxy often
    returns as None — so it silently falls through to a fresh per-session id.

The result: a returning anonymous user gets a NEW id every visit and looks like
a brand-new user, so returning_users under-counts badly (e.g. 12 real returners
showing as 2).

localStorage is the reliable fix: it's per-origin browser storage that survives
across visits and isn't subject to the cookie iframe/proxy problems. This module
reads a persistent id from window.localStorage (creating one the first time),
and hands it back to Python via a query param so it can key analytics identity.

HOW IT WORKS
------------
1. On first call, inject a tiny HTML/JS component that:
     - reads 'kinematik_vid' from the parent window's localStorage,
     - if absent, generates a uuid, stores it, and
     - sets it as a URL query param (?kvid=...) on the parent, triggering a
       Streamlit rerun.
2. On the rerun, Python reads st.query_params['kvid'] — a durable id that will
   be identical on every future visit from the same browser.

The id is written as 'ls-<uuid>' so its source is identifiable in the data
(distinct from 'fp-' fingerprint and 'ses-' per-session fallbacks).
"""

from __future__ import annotations

import uuid


_QP_KEY = "kvid"           # query param that carries the id back to Python
_LS_KEY = "kinematik_vid"  # localStorage key in the browser


def get_durable_visitor_id(st) -> str | None:
    """Return a durable per-browser id from localStorage, or None until it
    resolves (one rerun later, exactly like the cookie flow). Pass the streamlit
    module in as `st` so this stays import-light and testable.

    Usage in streamlit_app.py, as the FIRST identity tier:

        from suspension.visitor_id import get_durable_visitor_id
        _vid = get_durable_visitor_id(st)
        if _vid:
            _axn.set_visitor_id(_vid)
            # ... skip the cookie/fingerprint tiers when this resolves
    """
    # 1. Already carried back on this or a prior rerun? Use it, and remember it
    #    in session_state so we don't depend on the query param sticking around.
    try:
        _cached = st.session_state.get("_ax_ls_vid")
        if _cached:
            return _cached
    except Exception:
        pass

    try:
        _qp = st.query_params.get(_QP_KEY)
        if isinstance(_qp, (list, tuple)):
            _qp = _qp[0] if _qp else None
        if _qp:
            st.session_state["_ax_ls_vid"] = _qp
            # Clean the id out of the visible URL so it isn't shared/bookmarked.
            try:
                del st.query_params[_QP_KEY]
            except Exception:
                pass
            return _qp
    except Exception:
        pass

    # 2. Not yet carried back — inject the JS that reads/creates it in
    #    localStorage and reloads with ?kvid=... set. Runs once per session.
    if not st.session_state.get("_ax_ls_injected"):
        st.session_state["_ax_ls_injected"] = True
        _seed = "ls-" + uuid.uuid4().hex[:24]  # used only if none stored yet
        _js = f"""
        <script>
        (function() {{
            try {{
                var w = window.parent || window;
                var KEY = "{_LS_KEY}";
                var id = null;
                try {{ id = w.localStorage.getItem(KEY); }} catch (e) {{ id = null; }}
                if (!id) {{
                    id = "{_seed}";
                    try {{ w.localStorage.setItem(KEY, id); }} catch (e) {{}}
                }}
                var url = new URL(w.location.href);
                if (url.searchParams.get("{_QP_KEY}") !== id) {{
                    url.searchParams.set("{_QP_KEY}", id);
                    w.history.replaceState({{}}, "", url.toString());
                    w.location.reload();
                }}
            }} catch (e) {{ /* localStorage blocked — fall back to other tiers */ }}
        }})();
        </script>
        """
        try:
            import streamlit.components.v1 as _components
            _components.html(_js, height=0, width=0)
        except Exception:
            pass

    # Not resolved yet this render — caller falls through to cookie/fingerprint.
    return None
