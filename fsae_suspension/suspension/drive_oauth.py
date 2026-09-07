# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  suspension/drive_oauth.py — the per-user "Connect Google Drive" flow, made as
#  frictionless as Google's security model actually permits: one button, one
#  consent screen, automatic return + token capture, and the token cached so a
#  member never re-consents. Honest about the wall: this cannot be reduced below
#  one click out to Google and one click back — that's OAuth, not a KinematiK
#  limitation. For zero per-user friction, use the service-account path instead.
# ============================================================================
"""
Per-user Google Drive connection — the smoothest compliant OAuth flow.

WHY THIS EXISTS (AND WHAT IT CAN'T DO)
--------------------------------------
The service-account path in drive_export.py needs NO per-user connection — it's
the frictionless option, but it writes to a shared team Drive. When a member
wants reports in THEIR OWN Google account, that requires OAuth, and OAuth has an
irreducible floor: the user must (1) click connect, (2) choose their Google
account and grant consent on Google's own page, and (3) return to the app. No
compliant integration removes those steps — anything that appears to is either
sharing one credential (so it isn't really their account) or will get the OAuth
client flagged by Google. This module makes those three steps ONE button, ONE
consent screen, and an automatic return, then caches the token so it happens
exactly once per member.

THE FLOW
--------
  1. build_auth_url()  -> the app shows a single "Connect Google Drive" link/button.
  2. user consents on Google, Google redirects back to the app with ?code=...&state=...
  3. exchange_code()   -> the app (detecting the code in st.query_params) swaps it
                          for a token dict, with CSRF state verification.
  4. the token dict is cached (in session, and optionally persisted per-member by
     the caller) and handed to drive_export.export_report(oauth_token=...).
  Tokens auto-refresh via the stored refresh_token, so step 1–3 happen once.

CONFIG (one-time, by whoever deploys)
-------------------------------------
A Google OAuth *client* (Web application type) is created once in Google Cloud,
and its client_id / client_secret / redirect_uri are put in secrets as
GOOGLE_OAUTH_CLIENT_ID / GOOGLE_OAUTH_CLIENT_SECRET / GOOGLE_OAUTH_REDIRECT_URI.
The redirect_uri must be the app's own URL and registered in the client. This is
deployer setup, not per-member — members only ever click one button.

No google imports at module load — all lazy, so importing this never fails on a
deployment that hasn't installed the libs. Every function degrades to a clear,
actionable message rather than a stack trace.
"""

from __future__ import annotations

import os
import secrets as _secrets
from dataclasses import dataclass

from .drive_export import DRIVE_SCOPES


# ===================================================================== #
#  Config resolution
# ===================================================================== #
@dataclass
class OAuthConfig:
    client_id: str
    client_secret: str
    redirect_uri: str

    @property
    def ok(self) -> bool:
        return bool(self.client_id and self.client_secret and self.redirect_uri)


def load_oauth_config(read_credential=None) -> OAuthConfig | None:
    """Pull the OAuth client config from secrets/env, or None if not configured.

    read_credential: optional callable(name)->str (the app's _read_credential).
    """
    def get(name):
        if read_credential is not None:
            try:
                v = read_credential(name)
                if v:
                    return str(v)
            except Exception:
                pass
        return os.environ.get(name)

    cid = get("GOOGLE_OAUTH_CLIENT_ID")
    csec = get("GOOGLE_OAUTH_CLIENT_SECRET")
    ruri = get("GOOGLE_OAUTH_REDIRECT_URI")
    if not (cid and csec and ruri):
        return None
    return OAuthConfig(cid, csec, ruri)


def oauth_available(read_credential=None) -> tuple[bool, str]:
    """(can_offer_oauth, reason_if_not). UI uses this to decide whether to show
    the 'Connect Google Drive' button or point at the service-account path."""
    try:
        from google_auth_oauthlib.flow import Flow  # noqa: F401
    except Exception:
        return False, ("google-auth-oauthlib not installed. Add it to "
                       "requirements.txt to enable per-user Drive connect.")
    if load_oauth_config(read_credential) is None:
        return False, ("Per-user Drive connect isn't configured. A deployer needs "
                       "to add GOOGLE_OAUTH_CLIENT_ID / _SECRET / _REDIRECT_URI to "
                       "secrets. (Or use the zero-setup service-account path.)")
    return True, ""


# ===================================================================== #
#  The flow
# ===================================================================== #
def _client_config(cfg: OAuthConfig) -> dict:
    """The in-memory client-secrets structure google-auth-oauthlib expects."""
    return {
        "web": {
            "client_id": cfg.client_id,
            "client_secret": cfg.client_secret,
            "auth_uri": "https://accounts.google.com/o/oauth2/auth",
            "token_uri": "https://oauth2.googleapis.com/token",
            "redirect_uris": [cfg.redirect_uri],
        }
    }


def build_auth_url(read_credential=None,
                   state: str | None = None) -> tuple[str | None, str]:
    """Return (auth_url, state) to send the user to Google's consent screen.

    A random ``state`` is generated for CSRF protection if not supplied; the
    caller must stash it (session) and pass it to exchange_code for verification.
    Returns (None, reason) if OAuth isn't configured/available.
    """
    ok, reason = oauth_available(read_credential)
    if not ok:
        return None, reason
    cfg = load_oauth_config(read_credential)
    st_state = state or _secrets.token_urlsafe(24)
    from google_auth_oauthlib.flow import Flow
    flow = Flow.from_client_config(
        _client_config(cfg), scopes=DRIVE_SCOPES, state=st_state)
    flow.redirect_uri = cfg.redirect_uri
    auth_url, _ = flow.authorization_url(
        access_type="offline",           # get a refresh_token
        include_granted_scopes="true",
        prompt="consent")                # ensure refresh_token on re-consent
    return auth_url, st_state


def exchange_code(code: str, expected_state: str, returned_state: str,
                  read_credential=None) -> tuple[dict | None, str]:
    """Swap an authorization ``code`` for a token dict, verifying CSRF state.

    Returns (token_dict, "") on success, or (None, reason) on failure. The token
    dict is the shape drive_export._service_from_oauth_token expects.
    """
    if not code:
        return None, "No authorization code returned by Google."
    if not expected_state or returned_state != expected_state:
        # CSRF guard: the state coming back must match what we sent.
        return None, ("Security check failed (state mismatch). Please start the "
                      "connection again.")
    cfg = load_oauth_config(read_credential)
    if cfg is None:
        return None, "OAuth is no longer configured."
    try:
        from google_auth_oauthlib.flow import Flow
        flow = Flow.from_client_config(
            _client_config(cfg), scopes=DRIVE_SCOPES, state=expected_state)
        flow.redirect_uri = cfg.redirect_uri
        flow.fetch_token(code=code)
        c = flow.credentials
        token = {
            "token": c.token,
            "refresh_token": c.refresh_token,
            "token_uri": c.token_uri,
            "client_id": c.client_id,
            "client_secret": c.client_secret,
            "scopes": list(c.scopes or DRIVE_SCOPES),
        }
        return token, ""
    except Exception as e:
        return None, f"Could not complete the Google connection: {e}"


def connected_account_email(token: dict) -> str:
    """Best-effort: return the connected Google account's email for display, or
    "" if it can't be fetched. Purely cosmetic ('Connected as ...')."""
    try:
        from suspension.drive_export import _service_from_oauth_token
        service = _service_from_oauth_token(token)
        about = service.about().get(fields="user(emailAddress)").execute()
        return about.get("user", {}).get("emailAddress", "")
    except Exception:
        return ""


def token_is_usable(token: dict | None) -> bool:
    """True if the cached token at least has the fields needed to build creds and
    refresh. (Actual validity is proven on first API call, which auto-refreshes.)"""
    if not token:
        return False
    return bool(token.get("refresh_token") or token.get("token"))
