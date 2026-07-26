# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Tests for the workspace-oversight layer — the reference semantics that
workspace_oversight.sql enforces server-side:

  * overview() returns rows ONLY for workspaces the caller administers
    (owner or lead); plain members and viewers get an empty list.
  * each row reports how many people are using the workspace, who the owner
    is, who the lead(s) are, and the full member/viewer roster.
  * activity() is owner/lead-gated; members/viewers are refused.
  * the SupabaseAuth API surface exposes workspace_overview / workspace_activity.

Run: python tests/test_oversight.py
"""

import importlib
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)


def _load(name):
    return importlib.import_module(f"suspension.{name}")


W = _load("workspace")
A = _load("auth")

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
    except Exception:
        return False


def _registry():
    reg = W.MemoryWorkspaceRegistry()
    reg.create_workspace(W.Workspace(id="ws-aero", name="Aero"), "owner1")
    reg.create_workspace(W.Workspace(id="ws-chassis", name="Chassis"), "owner1")
    reg.add_member("owner1", "ws-aero", "lead1", role="lead")
    reg.add_member("owner1", "ws-aero", "m1", role="member")
    reg.add_member("owner1", "ws-aero", "m2", role="member")
    reg.add_member("owner1", "ws-aero", "v1", role="viewer")
    reg.add_member("owner1", "ws-chassis", "m1", role="member")
    return reg


def test_overview_owner_sees_all_admin_workspaces():
    reg = _registry()
    rows = reg.overview("owner1")
    check("owner sees both workspaces", [r["workspace_id"] for r in rows]
          == ["ws-aero", "ws-chassis"])
    aero = rows[0]
    check("member_count counts everyone using it", aero["member_count"] == 5)
    check("owner identified", aero["owner"] == "owner1")
    check("lead identified", aero["leads"] == ["lead1"])
    check("members listed", aero["members"] == ["m1", "m2"])
    check("viewers listed", aero["viewers"] == ["v1"])
    check("my_role is owner", aero["my_role"] == "owner")


def test_overview_lead_sees_only_their_workspace():
    reg = _registry()
    rows = reg.overview("lead1")
    check("lead sees exactly the workspace they lead",
          [r["workspace_id"] for r in rows] == ["ws-aero"])
    check("lead sees full roster too",
          rows[0]["members"] == ["m1", "m2"] and rows[0]["leads"] == ["lead1"])
    check("lead's my_role is lead", rows[0]["my_role"] == "lead")


def test_overview_member_and_viewer_get_nothing():
    reg = _registry()
    check("plain member gets empty overview", reg.overview("m1") == [])
    check("viewer gets empty overview", reg.overview("v1") == [])
    check("stranger gets empty overview", reg.overview("nobody") == [])


def test_activity_gating():
    reg = _registry()
    reg.put("m1", "ws-aero", "projects", "default",
            {"saved_by": "m1@team.edu", "workspace_id": "ws-aero"})
    check("owner can read activity",
          reg.activity("owner1", "ws-aero")[0]["saved_by"] == "m1@team.edu")
    check("lead can read activity",
          len(reg.activity("lead1", "ws-aero")) == 1)
    check("member refused activity",
          raises(W.WorkspaceError, lambda: reg.activity("m1", "ws-aero")))
    check("viewer refused activity",
          raises(W.WorkspaceError, lambda: reg.activity("v1", "ws-aero")))
    check("non-member refused activity",
          raises(W.WorkspaceError, lambda: reg.activity("nobody", "ws-aero")))


def test_auth_api_surface():
    check("SupabaseAuth exposes workspace_overview",
          callable(getattr(A.SupabaseAuth, "workspace_overview", None)))
    check("SupabaseAuth exposes workspace_activity",
          callable(getattr(A.SupabaseAuth, "workspace_activity", None)))


def test_ui_surface():
    ui = _load("auth_ui")
    check("auth_ui exposes render_workspace_oversight",
          callable(getattr(ui, "render_workspace_oversight", None)))


if __name__ == "__main__":
    for name, fn in sorted(list(globals().items())):
        if name.startswith("test_") and callable(fn):
            print(name)
            fn()
    print(f"\n{len(_PASS)} passed, {len(_FAIL)} failed")
    if _FAIL:
        sys.exit(1)
