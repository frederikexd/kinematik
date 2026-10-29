"""The per-file mutation gate: which groups a change runs, how pyproject is pointed at a group, how a file is scored."""
import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
_spec = importlib.util.spec_from_file_location("mutation_gate", ROOT / "scripts" / "mutation_gate.py")
gate = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gate)

CFG = {"floor": 90.0, "group": [
    {"name": "a", "files": ["suspension/x.py"], "tests": ["tests/test_x.py"], "deselect": []},
    {"name": "b", "files": ["suspension/y.py", "suspension/z.py"], "tests": ["tests/test_y.py"],
     "deselect": ["tests/test_y.py::slow"]}]}


def test_every_listed_file_and_test_is_real():
    cfg = gate.load()
    assert cfg["floor"] >= 99.0
    for g in cfg["group"]:
        for p in g["files"] + g["tests"]:
            assert (ROOT / p).is_file(), p
    files = [f for g in cfg["group"] for f in g["files"]]
    assert len(files) == len(set(files)), "a file is in two groups"


def test_a_change_runs_the_groups_it_touches():
    assert gate.touched(CFG, ["fsae_suspension/suspension/x.py"]) == ["a"]
    assert gate.touched(CFG, ["tests/test_y.py", "README.md"]) == ["b"]
    assert gate.touched(CFG, ["docs/notes.md"]) == []
    for every in ("fsae_suspension/mutation_targets.toml", "scripts/mutation_gate.py", "pyproject.toml",
                  ".github/workflows/mutation.yml"):
        assert gate.touched(CFG, [every]) == ["a", "b"]


def test_pyproject_is_pointed_at_the_group(tmp_path):
    p = tmp_path / "pyproject.toml"
    p.write_text('[tool.mutmut]\nonly_mutate = ["old.py"]\nalso_copy = ["data/"]\n'
                 'pytest_add_cli_args_test_selection = [\n    "tests/old.py",\n]\n')
    gate.configure(CFG["group"][1], p)
    text = p.read_text()
    assert 'only_mutate = [\n    "suspension/y.py",\n    "suspension/z.py",\n]' in text
    assert '"tests/test_y.py",\n    "--deselect=tests/test_y.py::slow",\n]' in text
    assert 'also_copy = ["data/"]' in text and "old.py" not in text
    p.write_text("[tool.mutmut]\n")
    with pytest.raises(SystemExit):
        gate.configure(CFG["group"][0], p)


def test_each_file_is_scored_on_its_own(tmp_path):
    (tmp_path / "suspension").mkdir()
    codes_y = {"m1": 1, "m2": 1, "m3": 0, "m4": 36, "m5": 34}     # killed, killed, survived, timeout, skipped
    (tmp_path / "suspension" / "y.py.meta").write_text(json.dumps({"exit_code_by_key": codes_y}))
    (tmp_path / "suspension" / "z.py.meta").write_text(json.dumps({"exit_code_by_key": {"n1": 1}}))
    res = gate.scores(CFG["group"][1], tmp_path)
    assert res == {"suspension/y.py": (3, 4, ["m3"]), "suspension/z.py": (1, 1, [])}
    ok, table = gate.report(CFG["group"][1], res, 90.0)
    assert not ok and "75.0% below floor" in table and "100.0%" in table
    ok, _ = gate.report(CFG["group"][1], {"suspension/y.py": (9, 10, []), "suspension/z.py": (1, 1, [])}, 90.0)
    assert ok
    ok, table = gate.report(CFG["group"][0], gate.scores(CFG["group"][0], tmp_path), 90.0)
    assert not ok and "0.0% below floor" in table                    # no results at all is a failure, not a pass


def test_a_mutmut_run_that_cannot_start_fails_the_group(monkeypatch, capsys):
    calls = []
    monkeypatch.setattr(gate, "configure", lambda g: calls.append(("configure", g["name"])))
    monkeypatch.setattr(gate.subprocess, "run", lambda cmd, **kw: calls.append(cmd) or type("R", (), {"returncode": 2})())
    monkeypatch.setattr(gate, "scores", lambda g: pytest.fail("a run that never started must not be scored"))
    for report_only in (False, True):
        assert gate.run_group(CFG, "a", 3, report_only=report_only) == 1
    assert calls[:2] == [("configure", "a"), ["mutmut", "run", "--max-children", "3"]]
    assert "mutmut could not start for group 'a' (exit 2)" in capsys.readouterr().out
