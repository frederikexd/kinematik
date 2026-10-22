# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Preview mode: the five-minute clock, the in-memory store that keeps a preview away from every persistent
backend, and the sign-in gate's three states (offer, in preview, preview over)."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from suspension import auth_ui                                    # noqa: E402
from suspension import preview as pv                              # noqa: E402
from suspension import project as project_mod                     # noqa: E402


# ------------------------------------------------------------------ clock
def test_clock_runs_five_minutes_and_never_restarts():
    ss = {}
    assert not pv.started(ss) and not pv.active(ss, 0.0) and not pv.expired(ss, 0.0)
    assert pv.start(ss, 1000.0)
    assert pv.active(ss, 1000.0) and pv.remaining_s(ss, 1000.0) == 300.0
    assert pv.active(ss, 1299.0) and not pv.expired(ss, 1299.0)
    assert pv.expired(ss, 1300.0) and pv.remaining_s(ss, 1300.0) == 0.0
    assert not pv.start(ss, 2000.0)                   # one preview per session
    assert pv.expired(ss, 2000.0)


def test_warning_fires_once_in_the_last_minute():
    ss = {}
    pv.start(ss, 0.0)
    assert not pv.should_warn(ss, 200.0)
    assert pv.should_warn(ss, 241.0)
    assert not pv.should_warn(ss, 250.0)


def test_ending_early_expires_it():
    ss = {}
    pv.start(ss, 0.0)
    pv.end_now(ss, 10.0)
    assert pv.expired(ss, 10.0)


def test_clock_format():
    assert pv.clock(300) == "5:00" and pv.clock(65.2) == "1:06" and pv.clock(0) == "0:00"


# ------------------------------------------------------------------ store isolation
def test_preview_store_never_touches_disk(tmp_path):
    ss = {}
    path = tmp_path / "project.json"
    store = project_mod.ProjectStore(str(path), backend=pv.backend(ss))
    store.save()
    assert not path.exists()                          # nothing written to the file
    again = project_mod.ProjectStore(str(path), backend=pv.backend(ss))
    assert again.backend is store.backend             # one backend per session
    assert pv.backend({}) is not store.backend        # another visitor gets their own


# ------------------------------------------------------------------ the gate
class _Stop(Exception):
    pass


class _FakeSt:
    def __init__(self, ss=None, press=()):
        self.session_state = ss if ss is not None else {}
        self.query_params = {}
        self.text, self.buttons, self.radio_index, self.press = [], [], None, set(press)

    def _say(self, *a, **k):
        self.text.append(str(a[0]) if a else "")
    markdown = caption = info = warning = error = success = _say

    def divider(self):
        pass

    def button(self, label, key=None, **k):
        self.buttons.append(label)
        return key in self.press

    def radio(self, label, options, index=0, **k):
        self.radio_index = index
        return options[index]

    def text_input(self, *a, **k):
        return ""

    def stop(self):
        raise _Stop

    def rerun(self):
        raise _Stop


class _FakeAuth:
    pass


@pytest.fixture
def gate(monkeypatch):
    monkeypatch.setattr(auth_ui, "_get_auth", lambda st: _FakeAuth())
    return auth_ui.require_workspace


def test_a_new_visitor_is_offered_the_preview(gate):
    st = _FakeSt()
    with pytest.raises(_Stop):
        gate(st)
    assert any("Preview" in b for b in st.buttons) and st.radio_index == 0


def test_starting_the_preview_lets_the_app_render(gate):
    st = _FakeSt(press={"_kx_preview_start"})
    with pytest.raises(_Stop):                        # the click reruns
        gate(st)
    assert pv.active(st.session_state)
    assert gate(_FakeSt(st.session_state)) is None    # next run: no stop, no workspace


def test_when_the_preview_ends_the_visitor_is_asked_to_sign_up(gate):
    ss = {}
    pv.start(ss, 0.0)                                 # long ago
    st = _FakeSt(ss)
    with pytest.raises(_Stop):
        gate(st)
    assert any("Like what you see?" in t for t in st.text)
    assert st.radio_index == 1                        # Create account preselected
    assert not any("Preview" in b for b in st.buttons)


def test_invited_visitors_are_not_offered_the_preview(gate):
    st = _FakeSt({auth_ui._SS_JOIN: "tok"})
    with pytest.raises(_Stop):
        gate(st)
    assert not any("Preview" in b for b in st.buttons)
