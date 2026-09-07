# Extracting a view out of `streamlit_app.py`

The plan in `ui/__init__.py` targets "no file over 3,000 lines within two
seasons". `streamlit_app.py` is ~32,800. That is not one commit, and it should
not be attempted as one — a big-bang split of a 33k-line file with a lazy
alias-injection layer is unreviewable, and unreviewable is how the two live
`NameError`s in `ui/` survived in the first place.

This is the procedure, proven end to end on the first slice
(`ui/run_log.py`, Aug 2026). Follow it per view.

---

## 0. Pick a slice by its seam, not its size

The right first candidate is the one with the **fewest references to shell
locals**, not the longest block. Measure before choosing:

```python
# free-variable analysis of a candidate block
import ast, pathlib, textwrap
lines = pathlib.Path("streamlit_app.py").read_text().split("\n")
start = <line of the `elif _view == "..."` head>  - 1
end   = <line of the next view marker>
body  = lines[start + 1:end]
tree  = ast.parse(textwrap.dedent("\n".join(body)))

assigned, used = set(), set()
for n in ast.walk(tree):
    if isinstance(n, ast.Name):
        (assigned if isinstance(n.ctx, ast.Store) else used).add(n.id)
    elif isinstance(n, ast.arg):
        assigned.add(n.arg)
    elif isinstance(n, (ast.Import, ast.ImportFrom)):
        for a in n.names:
            assigned.add((a.asname or a.name).split(".")[0])
    elif isinstance(n, (ast.FunctionDef, ast.ClassDef)):
        assigned.add(n.name)

print(sorted(used - assigned - set(dir(__builtins__))))
```

`run_log` came back as `['_aero_area', 'st']` plus three `except ... as`
bindings (which are false positives — `ast` does not record handler names as
`Store`). Two real dependencies is extractable. **A view that reaches into
thirty shell locals is not** — leave it and pick another, or spend the effort
narrowing its dependencies *first*, as its own change.

## 1. Write the module

```python
def render(st, <the free names>) -> None:
```

Rules from `ui/__init__.py`, all of which the first slice honours:

- `render()` is the only public surface.
- No physics. Equations stay in `suspension/`; the ui module orchestrates and
  draws. `run_log`'s maths lives in `suspension/aero/run_log.py`.
- **Streamlit is passed in, not imported at module top level.** This is what
  keeps the module importable headless and testable without a browser —
  verify with `python -c "import ui.<name>"` from outside the source tree and
  confirm `streamlit not in sys.modules`.

Paste the body verbatim on the first pass. Refactoring *and* moving in one
step means a failure tells you nothing about which caused it.

## 2. Replace the block with a delegation

```python
elif _view == "...":
    from ui import <name> as _mod
    _mod.render(st, <the free names>)
```

Import inside the branch, matching the shell's existing lazy pattern — the app
must not pay for every view at startup.

## 3. Rewrite the tests to CALL rather than SCRAPE

This is the actual prize, and it is easy to miss.

The old `test_run_log_ui.py` located the view by line indentation, sliced the
text out, rewrote its `elif` into `if True:`, and `exec`'d it against a mock.
It worked, but it could only ever test text whose boundaries it had guessed.
After extraction:

```python
from ui import run_log as _mod
_mod.render(mock_st, aero_area=1.0)
```

Same assertions, and now they run against the function the app actually calls.
All 27 tests passed unchanged apart from the harness.

Then add a guard that the body cannot creep back in beside the delegation —
see `test_the_shell_delegates_and_does_not_keep_a_copy`. Pick a distinctive
internal name from the moved code and assert it is *absent* from the shell.

## 4. Verify

```
ruff check .
python -m pytest tests/test_<view>.py
python -m pytest              # the whole suite, before you push
```

The extraction is not done until the full suite is green. The first slice
removed 311 lines from the shell and cost one line of net lint debt (a
`textwrap` import that went unused).

---

## Order of attack

Views already carrying their own test file are the cheapest, because step 3 is
where the risk is and an existing harness proves the behaviour before and
after. After those, prefer views whose engine already lives in `suspension/` —
they are the ones written as shells already.

## What NOT to do

- Do not move a view and refactor it in the same commit.
- Do not leave a copy behind "just in case". This repo has already had
  `streamlit_app (1).py`, three copies of `backends_PATCHED.py`, four
  duplicated `ui/` modules, a 96-file clone of the test suite, and two copies
  of `ui/omnicore` that drifted to opposite sides of a real `NameError` with
  nothing indicating which was authoritative.
- Do not delete a contract to make a failure go away. Retire it deliberately,
  in its own step, before the code it pins moves. (The `suspension/streamlit_app.py`
  consolidation was done wrong once for exactly this reason.)
