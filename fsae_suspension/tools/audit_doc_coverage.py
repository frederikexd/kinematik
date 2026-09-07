#!/usr/bin/env python3
"""Audit which features can actually produce a documentation PDF.

Run from the repo root:  python3 tools/audit_doc_coverage.py

Two different questions, and they need different checks:

  1. Is the feature WIRED for documentation? (registered, has a container,
     a label, a subsystem, and isn't in _DOC_PANEL_SKIP.) tests/test_doc_coverage.py
     asserts this in CI.

  2. Does the feature actually PRODUCE anything to document? A perfectly wired
     tab that never renders a metric, verdict or chart yields an empty report.
     That is what this script measures: the number of capture points in the
     feature's real code — st.metric / the metric() helper / plotly_chart /
     st.warning|error|success / dataframe / tag chips.

A low count is not necessarily a bug; some tabs genuinely produce few numbers.
A count of ZERO is worth investigating.

Note on aliases: tab bodies bind through chains like `tab_car = tab4` where
`tab4 = _id_to_container["model3d"]`. Following only the direct binding
under-reports — model3d looks like 0 and is really 31.
"""
import ast
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
APP = os.path.join(ROOT, "streamlit_app.py")

CAPTURE = re.compile(
    r'\bmetric\(|st\.metric\(|\.metric\(|plotly_chart\(|st\.warning\('
    r'|st\.error\(|st\.success\(|dataframe\(|st\.table\('
    r'|class="tag (?:bad|warn|good)"')


def _consts(src):
    out = {}
    for node in ast.parse(src).body:
        if isinstance(node, ast.Assign):
            for tgt in node.targets:
                if isinstance(tgt, ast.Name):
                    try:
                        out[tgt.id] = ast.literal_eval(node.value)
                    except Exception:
                        pass
    return out


def _vars_per_feature(src):
    """feature id -> every variable name bound to its container."""
    base = {}
    for m in re.finditer(
            r'^(\w+)\s*=\s*_id_to_container\[\s*[\'"](\w+)[\'"]\s*\]',
            src, re.M):
        base.setdefault(m.group(2), set()).add(m.group(1))
    var2feat = {v: f for f, vs in base.items() for v in vs}
    changed = True
    while changed:                      # resolve `tab_car = tab4`
        changed = False
        for m in re.finditer(r'^(\w+)\s*=\s*(\w+)\s*$', src, re.M):
            a, b = m.group(1), m.group(2)
            if b in var2feat and a not in var2feat:
                var2feat[a] = var2feat[b]
                base[var2feat[b]].add(a)
                changed = True
    return base


def _bodies(lines, var):
    """(start, end) for every `with <var>:` block, by indentation."""
    out = []
    pat = re.compile(r'^(\s*)with\s+' + re.escape(var) + r'\s*:\s*$')
    for i, line in enumerate(lines):
        m = pat.match(line)
        if not m:
            continue
        ind, end = len(m.group(1)), len(lines)
        for k in range(i + 1, len(lines)):
            s = lines[k]
            if s.strip() and (len(s) - len(s.lstrip())) <= ind:
                end = k
                break
        out.append((i, end))
    return out


def main():
    src = open(APP, encoding="utf-8").read()
    lines = src.splitlines()
    c = _consts(src)
    meta, skip = c["_TAB_META"], c["_DOC_PANEL_SKIP"]
    per_feat = _vars_per_feature(src)

    rows = []
    for fid, (_icon, label) in meta.items():
        if fid in skip:
            continue
        pts = 0
        for var in per_feat.get(fid, ()):
            for a, b in _bodies(lines, var):
                body = "\n".join(lines[a:b])
                pts += len(CAPTURE.findall(body))
                # follow delegation into ui/ modules
                found = re.findall(
                    r'ui[_\.]?(\w+)\.render|from ui import (\w+)|ui\.(\w+)',
                    body)
                for cand in {x for grp in found for x in grp if x}:
                    p = os.path.join(ROOT, "ui", f"{cand}.py")
                    if os.path.exists(p):
                        pts += len(CAPTURE.findall(
                            open(p, encoding="utf-8").read()))
        rows.append((pts, fid, label))

    rows.sort()
    print(f"{'capture pts':>11}  {'feature':16s} label")
    for pts, fid, label in rows:
        flag = "   <-- ZERO: produces nothing to document" if pts == 0 else ""
        print(f"{pts:11d}  {fid:16s} {label}{flag}")

    zero = [f for p, f, _ in rows if p == 0]
    print(f"\ndocumentable features : {len(rows)}")
    print(f"explicitly skipped    : {len(skip)} -> {sorted(skip)}")
    print(f"zero capture surface  : {len(zero)} {zero}")
    return 1 if zero else 0


if __name__ == "__main__":
    sys.exit(main())
