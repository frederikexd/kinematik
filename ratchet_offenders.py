import ast, importlib.util, os, sys
ROOT = os.path.abspath("fsae_suspension")
sys.path.insert(0, ROOT)
spec = importlib.util.spec_from_file_location(
    "_audit", os.path.join(ROOT, "tests", "test_repo_accuracy_audit.py"))
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)

def units():
    rows = []
    for rel, src in audit._sources():
        if rel.startswith(audit._UNITS_ALLOW_PREFIX) or "/test" in rel:
            continue
        try:
            tree = ast.parse(src)
        except SyntaxError:
            continue
        for n in ast.walk(tree):
            if not isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            if n.name.startswith("_") or n.name == "__post_init__":
                continue
            if not n.args.args and not n.args.kwonlyargs:
                continue
            r = getattr(n, "returns", None)
            if isinstance(r, ast.Constant) and r.value is None:
                continue
            if isinstance(r, ast.Name) and r.id in ("None", "str", "bool"):
                continue
            if audit._UNIT_IN_NAME.search(n.name):
                continue
            doc = ast.get_docstring(n) or ""
            if not doc:
                rows.append((rel, n.lineno, n.name, "no docstring"))
            elif not audit._UNIT_WORDS.search(doc):
                rows.append((rel, n.lineno, n.name, "no units in docstring"))
    return rows

def renders():
    rows = []
    for rel, path in audit._render_surfaces():
        if not os.path.exists(path):
            continue
        with open(path, encoding="utf-8", errors="replace") as fh:
            for i, line in enumerate(fh, 1):
                for hit in audit._NUMFMT.findall(line):
                    rows.append((rel, i, hit.strip(), ""))
    return rows

def report(name, rows, limit):
    print(f"\n=== {name}: {len(rows)} found, budget {limit}, "
          f"{max(0, len(rows)-limit)} over ===\n")
    by = {}
    for rel, ln, what, why in rows:
        by.setdefault(rel, []).append((ln, what, why))
    for rel in sorted(by, key=lambda r: -len(by[r])):
        print(f"{rel}  ({len(by[rel])})")
        for ln, what, why in sorted(by[rel]):
            print(f"    {rel}:{ln}  {what}" + (f"  -- {why}" if why else ""))
        print()

which = sys.argv[1] if len(sys.argv) > 1 else "both"
if which in ("units", "both"):
    report("undocumented units", units(), 1023)
if which in ("renders", "both"):
    report("ungated numeric renders", renders(), 685)
