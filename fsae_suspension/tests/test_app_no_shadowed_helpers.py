"""The app runs as one long script, so a top-level variable that reuses a helper's name replaces the helper for the
rest of the run. `_shown = [...]` in the CAD-files tab once turned `_shown()` into a list and crashed every page
after it ("'list' object is not callable"). No top-level assignment may reuse a top-level function's name."""
import ast
import os

_APP = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "streamlit_app.py")


def _top_level_rebinds(tree):
    funcs = {n.name for n in tree.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))}
    hits = []

    def names(t):
        return [x for x in ast.walk(t) if isinstance(x, ast.Name) and isinstance(x.ctx, ast.Store)]

    def walk(node):
        for child in ast.iter_child_nodes(node):
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef, ast.Lambda)):
                continue
            targets = []
            if isinstance(child, ast.Assign):
                targets = child.targets
            elif isinstance(child, (ast.AugAssign, ast.AnnAssign, ast.For)):
                targets = [child.target]
            elif isinstance(child, ast.With):
                targets = [i.optional_vars for i in child.items if i.optional_vars is not None]
            for t in targets:
                hits.extend((child.lineno, n.id) for n in names(t) if n.id in funcs)
            walk(child)

    walk(tree)
    return hits


def test_no_top_level_variable_reuses_a_helper_name():
    with open(_APP, encoding="utf-8") as fh:
        tree = ast.parse(fh.read())
    assert _top_level_rebinds(tree) == []
