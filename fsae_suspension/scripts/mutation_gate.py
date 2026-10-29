"""Per-file mutation gate: every file listed in mutation_targets.toml must keep its own score at or above the floor.

    python scripts/mutation_gate.py --list                    # group names, one per line
    python scripts/mutation_gate.py --matrix                  # every group, as a JSON list for a CI matrix
    python scripts/mutation_gate.py --changed a.py b.py ...   # JSON list of the groups those paths touch
    python scripts/mutation_gate.py --group core [--max-children 4]
    python scripts/mutation_gate.py --modules                 # every module in suspension/ and powertrain/, as JSON
    python scripts/mutation_gate.py --module suspension/x.py  # measure one module against the tests that import it

`--module` reports a score and never fails: it is the baseline for files not yet in a group.

`--group` rewrites the [tool.mutmut] file and test selection in pyproject.toml for that group, so run it in a
throwaway checkout (CI does) or restore the file afterwards. Run from fsae_suspension/. The score of a file is
killed (plus timed out) over every mutant tested, mutmut's own definition; skipped mutants are not counted.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tomllib
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TARGETS = ROOT / "mutation_targets.toml"
CAUGHT = {1, 3, 24, -24, 36, 152, 255}            # killed, or timed out (mutmut's status_by_exit_code)
SKIPPED = {34}


def load(path: Path = TARGETS) -> dict:
    with open(path, "rb") as f:
        cfg = tomllib.load(f)
    names = [g["name"] for g in cfg["group"]]
    if len(names) != len(set(names)):
        raise SystemExit(f"duplicate group names in {path.name}: {names}")
    return cfg


def touched(cfg: dict, paths: list[str]) -> list[str]:
    """Groups whose files or tests appear in `paths` (repo-relative or fsae_suspension-relative)."""
    rel = {p.removeprefix("fsae_suspension/") for p in paths}
    every = {"mutation_targets.toml", "scripts/mutation_gate.py", "pyproject.toml"}
    if rel & every or any(p.endswith(".github/workflows/mutation.yml") for p in paths):
        return [g["name"] for g in cfg["group"]]
    return [g["name"] for g in cfg["group"] if rel & (set(g["files"]) | set(g["tests"]))]


def modules() -> list[str]:
    """Every module in suspension/ and powertrain/, the files a group can hold."""
    out = []
    for pkg in ("suspension", "powertrain"):
        out += [str(p.relative_to(ROOT)) for p in sorted((ROOT / pkg).rglob("*.py"))
                if p.name != "__init__.py" and "__pycache__" not in p.parts]
    return out


def tests_for(module: str) -> list[str]:
    """Test files that import `module` (by dotted name, or by name from its package)."""
    dotted = module[:-3].replace("/", ".")
    parent, name = dotted.rsplit(".", 1)
    pat = re.compile(rf"\b{re.escape(dotted)}\b|from\s+{re.escape(parent)}\s+import\s*\(?[^)]*?\b{name}\b")
    return [str(t.relative_to(ROOT)) for t in sorted((ROOT / "tests").glob("test_*.py"))
            if pat.search(t.read_text(errors="replace"))]


def _toml_list(items: list[str]) -> str:
    return "[\n" + "".join(f'    "{i}",\n' for i in items) + "]"


def configure(group: dict, pyproject: Path = ROOT / "pyproject.toml") -> None:
    """Point [tool.mutmut] at this group's files and tests."""
    text = pyproject.read_text()
    sel = list(group["tests"]) + [f"--deselect={d}" for d in group.get("deselect", [])]
    new, n1 = re.subn(r"^only_mutate = \[[^\]]*\]", "only_mutate = " + _toml_list(group["files"]),
                      text, count=1, flags=re.M)
    new, n2 = re.subn(r"^pytest_add_cli_args_test_selection = \[[^\]]*\]",
                      "pytest_add_cli_args_test_selection = " + _toml_list(sel), new, count=1, flags=re.M)
    if n1 != 1 or n2 != 1:
        raise SystemExit("pyproject.toml [tool.mutmut] needs one only_mutate and one "
                         "pytest_add_cli_args_test_selection list")
    pyproject.write_text(new)


def scores(group: dict, mutants: Path = ROOT / "mutants") -> dict[str, tuple[int, int, list[str]]]:
    """file -> (caught, tested, surviving mutant names), from mutmut's per-file results."""
    out = {}
    for f in group["files"]:
        meta = mutants / (f + ".meta")
        codes = json.loads(meta.read_text())["exit_code_by_key"] if meta.exists() else {}
        c = Counter(codes.values())
        tested = sum(n for code, n in c.items() if code not in SKIPPED)
        caught = sum(n for code, n in c.items() if code in CAUGHT)
        out[f] = (caught, tested, sorted(k for k, v in codes.items() if v not in CAUGHT | SKIPPED))
    return out


def report(group: dict, res: dict, floor: float) -> tuple[bool, str]:
    ok, lines = True, [f"| {group['name']} | mutants | caught | score |", "|---|---:|---:|---:|"]
    for f, (caught, tested, _) in res.items():
        score = 100.0 * caught / tested if tested else 0.0
        bad = tested == 0 or score < floor
        ok &= not bad
        lines.append(f"| `{f}` | {tested} | {caught} | {score:.1f}%{' below floor' if bad else ''} |")
    lines.append(f"\nFloor {floor:.0f}% per file.")
    return ok, "\n".join(lines)


def run_group(cfg: dict, name: str, max_children: int, group: dict | None = None, report_only: bool = False) -> int:
    group = group or next((g for g in cfg["group"] if g["name"] == name), None)
    if group is None:
        raise SystemExit(f"no group {name!r} in {TARGETS.name}")
    if not group["tests"]:
        print(f"| `{group['files'][0]}` | no test imports it |")
        return 0 if report_only else 1
    configure(group)
    shutil.rmtree(ROOT / "mutants", ignore_errors=True)          # never score against stale results
    run = subprocess.run(["mutmut", "run", "--max-children", str(max_children)], cwd=ROOT, check=False)
    if run.returncode != 0:
        # mutmut exits non-zero only when it cannot start (its clean run of the group's tests failed, or no test
        # was collected), never because mutants survived; scoring then would report a silent 0%.
        print(f"mutmut could not start for group {group['name']!r} (exit {run.returncode}): its clean run of the "
              "group's tests failed before mutating anything. The failing test is in mutmut's output above.")
        return 1
    res = scores(group)
    ok, table = report(group, res, float(group.get("floor", cfg["floor"])))
    print(table)
    for f, (_, _, surv) in res.items():
        for m in surv:
            print(f"survived: {m}")
            show = subprocess.run(["mutmut", "show", m], cwd=ROOT, capture_output=True, text=True)
            print("".join(f"    {line}\n" for line in show.stdout.splitlines() if line[:2] in ("- ", "+ ")), end="")
    if os.environ.get("GITHUB_STEP_SUMMARY"):
        with open(os.environ["GITHUB_STEP_SUMMARY"], "a") as fh:
            fh.write(table + "\n\n")
    return 0 if ok or report_only else 1


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--list", action="store_true")
    mode.add_argument("--matrix", action="store_true")
    mode.add_argument("--changed", nargs="*")
    mode.add_argument("--group")
    mode.add_argument("--modules", action="store_true")
    mode.add_argument("--module")
    ap.add_argument("--max-children", type=int, default=4)
    a = ap.parse_args(argv)
    cfg = load()
    if a.list:
        print("\n".join(g["name"] for g in cfg["group"]))
    elif a.matrix:
        print(json.dumps([g["name"] for g in cfg["group"]]))
    elif a.changed is not None:
        print(json.dumps(touched(cfg, a.changed)))
    elif a.modules:
        print(json.dumps(modules()))
    elif a.module:
        g = {"name": a.module, "files": [a.module], "tests": tests_for(a.module), "deselect": []}
        return run_group(cfg, a.module, a.max_children, group=g, report_only=True)
    else:
        return run_group(cfg, a.group, a.max_children)
    return 0


if __name__ == "__main__":
    sys.exit(main())
