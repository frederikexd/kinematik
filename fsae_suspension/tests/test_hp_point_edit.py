"""A stale edit to a rocker point after "Pushrod-actuated" is unticked must not crash the app (1.7.11)."""
import ast
from pathlib import Path
from types import SimpleNamespace

SRC = (Path(__file__).resolve().parent.parent / "streamlit_app.py").read_text(encoding="utf-8")


def _fn(name):
    tree = ast.parse(SRC)
    node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)
    return ast.get_source_segment(SRC, node)


def _load(hp):
    st = SimpleNamespace(session_state=SimpleNamespace(hp=hp))
    ns = {"st": st}
    exec(_fn("_hp_set_point"), ns)
    return ns["_hp_set_point"]


def test_editing_a_switched_off_point_is_ignored():
    hp = {"rocker_pivot": None}
    _load(hp)("rocker_pivot", 1, 12.0)          # used to raise TypeError: 'NoneType' object is not iterable
    assert hp["rocker_pivot"] is None


def test_editing_a_point_still_sets_the_coordinate():
    hp = {"rocker_pivot": [1.0, 2.0, 3.0]}
    _load(hp)("rocker_pivot", 1, 12.0)
    assert hp["rocker_pivot"] == [1.0, 12.0, 3.0]
