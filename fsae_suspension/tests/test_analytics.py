"""Tests for the usage-telemetry module (suspension/analytics.py).

Verifies the fire-and-forget contract: events buffer to disk with no Supabase,
the timed() context manager records latency and logs errors-on-raise, identity
is stable per session, and disabling via env is honoured. Headless, no network.
"""
import os
import sys
import json
import time
import importlib

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _fresh_module(tmp_cwd, monkeypatch=None):
    """Import a clean copy of analytics with cwd pointed at a temp dir so the
    local buffer is isolated per test."""
    os.chdir(tmp_cwd)
    import suspension.analytics as ax
    importlib.reload(ax)
    return ax


def _drain(ax, path):
    ax._SINK.flush_blocking(2.0)
    # give the daemon thread a beat to write
    for _ in range(20):
        if os.path.exists(path):
            break
        time.sleep(0.05)
    time.sleep(0.2)
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return [json.loads(l) for l in f if l.strip()]


def test_events_buffer_locally_without_supabase(tmp_path):
    ax = _fresh_module(tmp_path)
    ax.init(member="Tester", subteam="aero")
    ax.tab_open("kinematics")
    ax.engage("kinematics", "solve")
    ax.complete("kinematics", "solve")
    path = os.path.join(str(tmp_path), "analytics_buffer.jsonl")
    rows = _drain(ax, path)
    types = {r["event_type"] for r in rows}
    assert "session_start" in types
    assert "tab_open" in types
    assert "feature_engage" in types
    assert "workflow_complete" in types
    assert "first_result" in types          # complete() triggers first_result
    assert all(r["member"] == "Tester" for r in rows)


def test_timed_records_latency_and_errors(tmp_path):
    ax = _fresh_module(tmp_path)
    ax.init(subteam="suspension")
    with ax.timed("kinematics", "render"):
        time.sleep(0.01)
    raised = False
    try:
        with ax.timed("laptime", "render"):
            raise ValueError("boom")
    except ValueError:
        raised = True
    assert raised   # the context manager re-raises
    path = os.path.join(str(tmp_path), "analytics_buffer.jsonl")
    rows = _drain(ax, path)
    renders = [r for r in rows if r["event_type"] == "render"]
    errors = [r for r in rows if r["event_type"] == "error"]
    assert any(r["feature"] == "kinematics" and r["duration_ms"] >= 0
               for r in renders)
    assert any(r["feature"] == "laptime" and r["error_kind"] == "ValueError"
               for r in errors)


def test_first_result_is_idempotent(tmp_path):
    ax = _fresh_module(tmp_path)
    ax.init()
    ax.first_result()
    ax.first_result()
    ax.first_result()
    path = os.path.join(str(tmp_path), "analytics_buffer.jsonl")
    rows = _drain(ax, path)
    assert sum(1 for r in rows if r["event_type"] == "first_result") == 1


def test_stable_session_id(tmp_path):
    ax = _fresh_module(tmp_path)
    ax.init()
    ax.tab_open("a")
    ax.tab_open("b")
    path = os.path.join(str(tmp_path), "analytics_buffer.jsonl")
    rows = _drain(ax, path)
    assert len({r["session_id"] for r in rows}) == 1


def test_disable_via_env(tmp_path, monkeypatch):
    monkeypatch.setenv("KINEMATIK_ANALYTICS", "off")
    ax = _fresh_module(tmp_path)
    ax.init()
    ax.tab_open("kinematics")
    ax.complete("kinematics")
    path = os.path.join(str(tmp_path), "analytics_buffer.jsonl")
    rows = _drain(ax, path)
    assert rows == []   # nothing logged when disabled


def test_invalid_event_type_is_ignored(tmp_path):
    ax = _fresh_module(tmp_path)
    ax.init()
    ax._emit("not_a_real_type", feature="x")   # should be silently dropped
    path = os.path.join(str(tmp_path), "analytics_buffer.jsonl")
    rows = _drain(ax, path)
    assert all(r["event_type"] != "not_a_real_type" for r in rows)


def test_never_raises_on_bad_payload(tmp_path):
    ax = _fresh_module(tmp_path)
    ax.init()
    # a payload that can't be json-serialised must not raise; default=str saves it
    class Weird:
        pass
    ax.complete("kinematics", "solve", payload={"obj": Weird()})
    # if we got here without an exception, the contract held
    assert True


if __name__ == "__main__":
    import traceback, tempfile, pathlib
    fns = [(n, f) for n, f in sorted(globals().items())
           if n.startswith("test_") and callable(f)]
    passed = 0
    for n, f in fns:
        d = tempfile.mkdtemp()
        try:
            import inspect
            params = inspect.signature(f).parameters
            kwargs = {}
            if "tmp_path" in params:
                kwargs["tmp_path"] = pathlib.Path(d)
            if "monkeypatch" in params:
                print(f"~ {n} (skipped: needs pytest monkeypatch)")
                continue
            f(**kwargs)
            print("✓", n)
            passed += 1
        except Exception:
            print("✗", n)
            traceback.print_exc()
    print(f"\n{passed} passed (run via pytest for monkeypatch tests)")
