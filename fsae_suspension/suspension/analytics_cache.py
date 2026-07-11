# ============================================================================
#  KinematiK — cached read-through for the analytics dashboard views
#
#  WHY THIS EXISTS
#  ---------------
#  The Analytics tab pulls 11 metric views from Supabase. Streamlit reruns the
#  whole script on every interaction, and every tab body executes on every
#  rerun — so those 11 SELECTs were firing constantly, pulling full result sets
#  out of Supabase each time. That repeated read traffic was the dominant egress
#  cost on the free tier.
#
#  This wraps analytics.fetch_view in an @st.cache_data layer with a short TTL.
#  The dashboard numbers don't need to be live to the second — a few minutes old
#  is completely fine for usage/ROI metrics — so we serve them from cache and
#  only actually hit Supabase once per TTL window per view. That turns "N reads
#  per rerun" into "one read every few minutes", regardless of how much the user
#  clicks around.
#
#  HOW TO USE (in streamlit_app.py)
#  --------------------------------
#  At the top of the Analytics tab, where the views are pulled, replace:
#
#      roi           = _axn.fetch_view("v_roi_summary")
#      hours_by_feat = _axn.fetch_view("v_hours_saved_by_feature")
#      ...
#
#  with:
#
#      from suspension.analytics_cache import fetch_view_cached as _fetch_view
#      roi           = _fetch_view("v_roi_summary")
#      hours_by_feat = _fetch_view("v_hours_saved_by_feature")
#      ...
#
#  (i.e. just swap `_axn.fetch_view` -> `_fetch_view` on those 11 lines.)
#
#  A "Refresh now" button is provided if an operator wants to bust the cache and
#  see live numbers immediately — see clear_cache() below.
# ============================================================================

from __future__ import annotations

from typing import Optional

# TTL in seconds. 300 = 5 minutes. Raise it to reduce egress further (the
# dashboard just shows slightly older numbers); lower it if you want fresher
# data at the cost of more reads. This single constant is the whole tuning knob.
_CACHE_TTL_SECONDS = 300


def fetch_view_cached(view_name: str) -> list[dict]:
    """Cached read-through to analytics.fetch_view.

    When Streamlit is available, results are memoised for _CACHE_TTL_SECONDS so
    repeated reruns don't re-query Supabase. When Streamlit is NOT available
    (plain scripts / tests), this degrades to a direct, uncached call so the
    function is still usable everywhere — matching the rest of the analytics
    module's "works headless too" contract.

    Honours the kill-switch automatically: the underlying fetch_view returns []
    without touching Supabase when KINEMATIK_ANALYTICS=off, so a disabled deploy
    caches empty lists at zero network cost.
    """
    try:
        import streamlit as st
    except Exception:
        # No Streamlit runtime — call straight through, no caching.
        from suspension import analytics as _ax
        return _ax.fetch_view(view_name)

    # Define the cached inner exactly once and reuse it. st.cache_data keys on
    # the function + its args, so passing view_name gives a per-view cache entry
    # with its own TTL. show_spinner=False keeps the dashboard from flashing a
    # spinner on every cached read.
    @st.cache_data(ttl=_CACHE_TTL_SECONDS, show_spinner=False)
    def _cached(_view_name: str) -> list[dict]:
        from suspension import analytics as _ax
        return _ax.fetch_view(_view_name)

    return _cached(view_name)


def clear_cache() -> None:
    """Bust the cached view results so the next fetch_view_cached call hits
    Supabase live. Wire this to a 'Refresh now' button in the Analytics tab:

        if st.button("🔄 Refresh now"):
            from suspension.analytics_cache import clear_cache
            clear_cache()
            st.rerun()
    """
    try:
        import streamlit as st
        st.cache_data.clear()
    except Exception:
        pass
