# ============================================================================
#  KinematiK — tests for suspension/drive_oauth.py
#  Verifies the per-user Drive OAuth flow's honest, testable parts: config
#  resolution, availability probing, CSRF-state protection, and graceful
#  degradation when unconfigured. The live Google round-trip is not exercised in
#  CI (no real client / no network to Google), by design.
# ============================================================================
import json
import pytest

from suspension import drive_oauth as ox


def _set_oauth_env(monkeypatch):
    monkeypatch.setenv("GOOGLE_OAUTH_CLIENT_ID", "cid.apps.googleusercontent.com")
    monkeypatch.setenv("GOOGLE_OAUTH_CLIENT_SECRET", "shhh")
    monkeypatch.setenv("GOOGLE_OAUTH_REDIRECT_URI", "https://app.example/kinematik")


def test_config_none_when_unset(monkeypatch):
    for k in ("GOOGLE_OAUTH_CLIENT_ID", "GOOGLE_OAUTH_CLIENT_SECRET",
              "GOOGLE_OAUTH_REDIRECT_URI"):
        monkeypatch.delenv(k, raising=False)
    assert ox.load_oauth_config(read_credential=lambda n: None) is None


def test_config_loads_when_set(monkeypatch):
    _set_oauth_env(monkeypatch)
    cfg = ox.load_oauth_config()
    assert cfg is not None and cfg.ok
    assert cfg.redirect_uri.startswith("https://")


def test_oauth_available_reason_when_unconfigured(monkeypatch):
    for k in ("GOOGLE_OAUTH_CLIENT_ID", "GOOGLE_OAUTH_CLIENT_SECRET",
              "GOOGLE_OAUTH_REDIRECT_URI"):
        monkeypatch.delenv(k, raising=False)
    ok, reason = ox.oauth_available(read_credential=lambda n: None)
    assert ok is False and reason              # actionable message, no crash


def test_build_auth_url_generates_state_and_google_url(monkeypatch):
    _set_oauth_env(monkeypatch)
    url, state = ox.build_auth_url()
    assert url and "accounts.google.com" in url
    assert "state=" in url
    assert state and len(state) > 10           # random CSRF state present
    # the app's redirect and client id must be in the consent URL
    assert "cid.apps.googleusercontent.com" in url


def test_build_auth_url_declines_when_unconfigured(monkeypatch):
    for k in ("GOOGLE_OAUTH_CLIENT_ID", "GOOGLE_OAUTH_CLIENT_SECRET",
              "GOOGLE_OAUTH_REDIRECT_URI"):
        monkeypatch.delenv(k, raising=False)
    url, reason = ox.build_auth_url(read_credential=lambda n: None)
    assert url is None and reason              # no url, clear reason


def test_exchange_code_rejects_state_mismatch(monkeypatch):
    _set_oauth_env(monkeypatch)
    # CSRF guard: a returned state that doesn't match must be refused BEFORE any
    # network call to Google.
    token, reason = ox.exchange_code(
        code="abc", expected_state="sent-state", returned_state="evil-state")
    assert token is None
    assert "state" in reason.lower()


def test_exchange_code_rejects_missing_code(monkeypatch):
    _set_oauth_env(monkeypatch)
    token, reason = ox.exchange_code(
        code="", expected_state="s", returned_state="s")
    assert token is None and "code" in reason.lower()


def test_token_is_usable():
    assert ox.token_is_usable({"refresh_token": "r"}) is True
    assert ox.token_is_usable({"token": "t"}) is True
    assert ox.token_is_usable({}) is False
    assert ox.token_is_usable(None) is False


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
