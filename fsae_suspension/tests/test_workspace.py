# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Tests for suspension/workspace.py — tenant sandboxing semantics: id
validation, service-key refusal, foreign-payload rejection, per-workspace
local storage, and membership-gated in-memory registry (the reference model
for the RLS policies in workspace_isolation.sql).
Run: python tests/test_workspace.py"""

import base64
import importlib
import json
import os
import sys
import tempfile

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)


def _load(name):
    # Import through the real (lazy) package so pytest and standalone runs see
    # the same module objects — no stub `suspension` in sys.modules.
    return importlib.import_module(f"suspension.{name}")


W = _load("workspace")

_PASS, _FAIL = [], []


def check(name, cond):
    (_PASS if cond else _FAIL).append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")


def raises(exc, fn):
    try:
        fn()
        return False
    except exc:
        return True


# --- id validation blocks traversal / garbage -------------------------------- #
check("valid id accepted", W.validate_workspace_id("team-alpha_01") == "team-alpha_01")
for bad in ("../evil", "a/b", "", "x" * 65, "team alpha", ".."):
    check(f"id {bad!r} rejected", raises(W.WorkspaceError,
                                         lambda b=bad: W.validate_workspace_id(b)))
check("uuid required when asked", raises(W.WorkspaceError,
      lambda: W.validate_workspace_id("team-alpha", require_uuid=True)))
check("uuid accepted", W.validate_workspace_id(
    "0b6a1c2d-1111-4222-8333-444455556666", require_uuid=True))

# --- service-role key refusal ------------------------------------------------ #
def fake_jwt(role):
    seg = base64.urlsafe_b64encode(json.dumps({"role": role}).encode()).decode().rstrip("=")
    return f"eyJhbGciOiJIUzI1NiJ9.{seg}.sig"


check("service_role jwt refused",
      raises(W.WorkspaceError, lambda: W.refuse_service_role(fake_jwt("service_role"))))
check("anon jwt accepted", W.refuse_service_role(fake_jwt("anon")) is None)

# --- payload scoping: foreign workspace ids are hard errors ------------------ #
check("clean payload passes", W.assert_payload_scoped(
    {"weights": [{"name": "wing", "workspace_id": "ws-a"}]}, "ws-a") is not None)
check("foreign id in nested payload rejected", raises(
    W.CrossWorkspaceViolation,
    lambda: W.assert_payload_scoped(
        {"notes": [{"msg": "hi", "workspace_id": "ws-b"}]}, "ws-a")))

# --- local backend: physically separate directories per workspace ------------ #
with tempfile.TemporaryDirectory() as td:
    a = W.LocalWorkspaceBackend("team-a", root=td)
    b = W.LocalWorkspaceBackend("team-b", root=td)
    a.write({"target_mass_kg": 230})
    b.write({"target_mass_kg": 180})
    check("workspace dirs are disjoint", os.path.dirname(a.path)
          != os.path.dirname(b.path))
    check("A reads only A", a.read()["target_mass_kg"] == 230)
    check("B reads only B", b.read()["target_mass_kg"] == 180)
    check("written rows stamped with workspace",
          a.read()["workspace_id"] == "team-a")
    # a file swapped across tenants (contamination) is refused on read
    with open(a.path) as f:
        stolen = f.read()
    with open(b.path, "w") as f:
        f.write(stolen)
    check("cross-workspace file contamination detected on read",
          raises(W.CrossWorkspaceViolation, b.read))

# --- membership-gated registry (reference RLS semantics) --------------------- #
reg = W.MemoryWorkspaceRegistry()
reg.create_workspace(W.Workspace("ws-a", "Elbee Racing"), owner_user_id="ana")
reg.create_workspace(W.Workspace("ws-b", "Volt EV Startup", kind="ev_startup"),
                     owner_user_id="bob")
reg.put("ana", "ws-a", "ledger", "car-24", {"mass_kg": 231})
reg.put("bob", "ws-b", "ledger", "car-24", {"mass_kg": 640})
check("same row id isolated per workspace",
      reg.get("ana", "ws-a", "ledger", "car-24")["mass_kg"] == 231 and
      reg.get("bob", "ws-b", "ledger", "car-24")["mass_kg"] == 640)
check("non-member read blocked", raises(W.CrossWorkspaceViolation,
      lambda: reg.get("ana", "ws-b", "ledger", "car-24")))
check("non-member write blocked", raises(W.CrossWorkspaceViolation,
      lambda: reg.put("ana", "ws-b", "ledger", "car-24", {"mass_kg": 1})))
check("non-member list blocked", raises(W.CrossWorkspaceViolation,
      lambda: reg.list_rows("ana", "ws-b", "ledger")))
check("only owner/lead invite", raises(W.WorkspaceError,
      lambda: reg.add_member("ana", "ws-b", "mallory")))
reg.add_member("bob", "ws-b", "vic", role="viewer")
check("viewer can read", reg.get("vic", "ws-b", "ledger", "car-24") is not None)
check("viewer cannot write", raises(W.WorkspaceError,
      lambda: reg.put("vic", "ws-b", "ledger", "car-24", {"mass_kg": 0})))
check("payload with foreign ws id blocked at write", raises(
    W.CrossWorkspaceViolation,
    lambda: reg.put("ana", "ws-a", "ledger", "import",
                    {"config": {"workspace_id": "ws-b"}})))

# --- context ------------------------------------------------------------------ #
ctx = W.WorkspaceContext(W.Workspace("ws-a", "Elbee Racing"), user_id="ana",
                         role="viewer")
check("viewer context cannot write", not ctx.can_write())
check("member context can write", W.WorkspaceContext(
    W.Workspace("ws-a", "Elbee"), role="member").can_write())


# --------------------------------------------------------------------------- #
#  Hosted deployments must REFUSE the local fallback.
#
#  LocalWorkspaceBackend separates tenants by directory. That is a boundary on
#  a laptop and none at all on Streamlit Cloud, where one container and one
#  filesystem serve every visitor. These cases pin the refusal so a future
#  "just fall back so it never crashes" edit fails loudly here instead of
#  quietly weakening isolation for every team at once.
# --------------------------------------------------------------------------- #
def _hosted_env(**overrides):
    """Context manager: run with a clean, explicitly-set backend environment."""
    import contextlib

    @contextlib.contextmanager
    def _cm():
        keys = ("SUPABASE_URL", "SUPABASE_ANON_KEY", "SUPABASE_KEY",
                "KINEMATIK_HOSTED", "STREAMLIT_SERVER_HEADLESS_RUNTIME",
                "STREAMLIT_SHARING_MODE", "STREAMLIT_RUNTIME_ENV")
        saved = {k: os.environ.get(k) for k in keys}
        for k in keys:
            os.environ.pop(k, None)
        os.environ.update({k: v for k, v in overrides.items() if v is not None})
        try:
            yield
        finally:
            for k in keys:
                os.environ.pop(k, None)
                if saved[k] is not None:
                    os.environ[k] = saved[k]
    return _cm()


_TEST_WS_UUID = "123e4567-e89b-12d3-a456-426614174000"
_ANON_JWT = "eyJhbGciOiJIUzI1NiJ9.eyJyb2xlIjoiYW5vbiJ9.sig"


def _ctx():
    return W.WorkspaceContext(workspace=W.Workspace(id=_TEST_WS_UUID, name="A"),
                              user_id="u1", access_token="tok",
                              role="owner", email="a@b.c")


_tmp = tempfile.mkdtemp()

with _hosted_env():
    check("laptop, no creds -> local backend (unchanged)",
          isinstance(W.workspace_backend(_ctx(), root=_tmp), W.LocalWorkspaceBackend))
    check("laptop is not hosted", W.is_hosted() is False)

with _hosted_env(KINEMATIK_HOSTED="1"):
    check("hosted + no creds -> refuse, never local",
          raises(W.TenantBackendUnavailable,
                 lambda: W.workspace_backend(_ctx(), root=_tmp)))

with _hosted_env(KINEMATIK_HOSTED="1", SUPABASE_URL="https://nope.invalid",
                 SUPABASE_ANON_KEY=_ANON_JWT):
    check("hosted + unreachable backend -> refuse, never local",
          raises(W.TenantBackendUnavailable,
                 lambda: W.workspace_backend(_ctx(), root=_tmp)))

with _hosted_env(SUPABASE_URL="https://nope.invalid", SUPABASE_ANON_KEY=_ANON_JWT):
    _b = W.workspace_backend(_ctx(), root=_tmp)
    check("laptop + unreachable backend -> degraded local, reason set",
          isinstance(_b, W.LocalWorkspaceBackend) and bool(_b.degraded_reason))

with _hosted_env(STREAMLIT_RUNTIME_ENV="cloud"):
    check("streamlit cloud marker implies hosted", W.is_hosted() is True)

with _hosted_env(STREAMLIT_RUNTIME_ENV="cloud", KINEMATIK_HOSTED="0"):
    check("explicit KINEMATIK_HOSTED=0 overrides the marker", W.is_hosted() is False)

with _hosted_env(KINEMATIK_HOSTED="1"):
    _eph = W.EphemeralWorkspaceBackend()
    _eph.write({"a": 1})
    check("ephemeral backend round-trips in memory", _eph.read().get("a") == 1)
    check("ephemeral backend is marked as such", _eph.read().get("_ephemeral") is True)
    check("ephemeral backend touches no shared path", not hasattr(_eph, "path"))
    check("ephemeral backend has no workspace bound", _eph.workspace_id is None)
    check("ephemeral backend explains itself", bool(_eph.degraded_reason))

print(f"\n{len(_PASS)} passed, {len(_FAIL)} failed")


# --- pytest bridge: expose every module-level check as a test case ---------- #
import pytest  # noqa: E402


@pytest.mark.parametrize("name", _PASS + _FAIL)
def test_check(name):
    assert name not in _FAIL, f"check failed: {name}"


if __name__ == "__main__":
    sys.exit(1 if _FAIL else 0)
