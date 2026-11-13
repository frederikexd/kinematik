# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Offline consistency check: the currents one team declares vs the hardware another team drew.

The powertrain or systems side states what each load draws; the electrical side sizes fuses, traces and copper.
Nobody checks one against the other until something blows. This runs that check from the command line, on the
engineer's own machine, and writes a report that another engineer can reproduce from the same two files:

    kinematik-check board.kicad_pcb --loads loads.csv --report report.md --json report.json
    python -m suspension.consistency_check board.kicad_pcb --loads loads.csv

loads.csv has one row per powered net: ``net,current_a[,voltage_v][,owner]``. ``owner`` names who declared the
number (e.g. "powertrain"), so every result traces back to a person. Nets the file does not declare fall back to the
PCB Doctor's name-based guess, and every guess is reported as insufficient evidence, never as a pass.

Nothing is uploaded and nothing is timestamped: the same inputs and tool version give a byte-identical report, with
SHA-256 hashes of both inputs in it. Exit status: 0 clean, 1 a result at or above --fail-on, 2 an input error.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

from .interfaces import Finding, Severity

#  What each kind of check rests on. A result is only as trustworthy as the evidence behind its method, so the report
#  states it per result instead of one confidence number for the whole board. Nothing here is physically correlated
#  with a built board yet, and the report says so.
_EVIDENCE = (
    ("IR drop", "benchmarked: net resistance vs an independent 2-D copper solve, 92/92 held-out nets within 2 %"),
    ("trace ampacity", "calibrated: heating fitted to the 1956 NBS data behind IPC-2221 (fit, not a held-out test)"),
    ("fusing margin", "calibrated: Onderdonk fusing with the NBS-fitted heating model (fit, not a held-out test)"),
    ("component", "rule: a part's rating or placement vs the declared current and copper temperature"),
    ("copper short", "geometric: copper of two nets overlapping on one layer"),
    ("copper open", "geometric: a net's copper not connected end to end"),
    ("via bottleneck", "analytical: via barrel current density"),
    ("HV", "rule: IPC-2221 clearance by voltage"),
    ("diff pair", "rule: matched width and length"),
    ("current not declared", "input gap: no declared current for this net"),
    ("declared load", "input gap: the loads file and the board disagree"),
    ("board", "benchmarked: board reading vs Altium 26.11.1 and KiCad 9.0.3"),
)
_STATUS = {"fail": "FAIL", "warning": "WARN", "missing": "INSUFFICIENT EVIDENCE", "info": "INFO", "ok": "PASS"}
_ORDER = ("fail", "warning", "missing", "info", "ok")
_FAIL_ON = {"fail": ("fail",), "warning": ("fail", "warning"), "never": ()}


def evidence_for(check: str) -> str:
    for prefix, label in _EVIDENCE:
        if check.startswith(prefix):
            return label
    return "analytical"


def tool_version() -> str:
    """The version in pyproject.toml when running from a checkout (installed metadata can be stale there)."""
    pp = Path(__file__).resolve().parent.parent / "pyproject.toml"
    try:
        import tomllib
        return tomllib.loads(pp.read_text(encoding="utf-8"))["project"]["version"]
    except (OSError, KeyError, ValueError):
        pass
    try:
        from importlib.metadata import version
        return version("kinematik")
    except Exception:  # pragma: no cover - not installed and no checkout
        return "unknown"


@dataclass
class DeclaredLoad:
    net: str
    current_a: float
    voltage_v: float | None
    owner: str
    line: int


def parse_loads_csv(text: str) -> list[DeclaredLoad]:
    """Read ``net,current_a[,voltage_v][,owner]``: current in amps (A), voltage in volts (V). Raises ValueError
    naming the line, since a bad number here silently becomes a wrong verdict."""
    rows = list(csv.DictReader(io.StringIO(text.lstrip("﻿"))))
    if not rows:
        raise ValueError("loads file is empty: expected a header 'net,current_a' and one row per powered net")
    cols = {c.strip().lower() for c in (rows[0].keys() if rows else []) if c}
    for need in ("net", "current_a"):
        if need not in cols:
            raise ValueError(f"loads file has no '{need}' column (found: {', '.join(sorted(cols)) or 'none'})")
    out, seen = [], set()
    for i, raw in enumerate(rows, start=2):
        r = {(k or "").strip().lower(): (v or "").strip() for k, v in raw.items()}
        net = r.get("net", "")
        if not net:
            raise ValueError(f"loads file line {i}: empty net name")
        if net.upper() in seen:
            raise ValueError(f"loads file line {i}: net '{net}' is declared twice")
        seen.add(net.upper())
        try:
            cur = float(r.get("current_a", ""))
        except ValueError:
            raise ValueError(f"loads file line {i}: current_a '{r.get('current_a', '')}' is not a number") from None
        if not math.isfinite(cur) or cur < 0:
            raise ValueError(f"loads file line {i}: current_a must be a finite number ≥ 0, got {cur}")
        volt = None
        if r.get("voltage_v"):
            try:
                volt = float(r["voltage_v"])
            except ValueError:
                raise ValueError(f"loads file line {i}: voltage_v '{r['voltage_v']}' is not a number") from None
            if not math.isfinite(volt) or volt <= 0:
                raise ValueError(f"loads file line {i}: voltage_v must be a finite number > 0, got {volt}")
        out.append(DeclaredLoad(net, cur, volt, r.get("owner", "") or "unspecified", i))
    return out


@dataclass
class CheckResult:
    board_file: str
    board_sha256: str
    loads_file: str
    loads_sha256: str
    version: str
    params: dict
    nets: list = field(default_factory=list)       # [{net, current_a, voltage_v, provenance}]
    findings: list = field(default_factory=list)   # [Finding]
    boundary: list = field(default_factory=list)   # [{net, owner, current_a, fuses: [(ref, rating, verdict)]}]

    def counts(self) -> dict:
        """Results per status: unitless counts."""
        c = dict.fromkeys(_ORDER, 0)
        for f in self.findings:
            c[f.severity.value] += 1
        return c

    def worst_hits(self, fail_on: str) -> int:
        """How many results sit at or above ``fail_on``: a unitless count."""
        levels = _FAIL_ON[fail_on]
        return sum(1 for f in self.findings if f.severity.value in levels)


def run_check(board_data, board_name: str, loads_data, loads_name: str = "loads.csv",
              rail_v: float = 5.0, brownout_v: float = 4.5, ambient_c: float = 40.0,
              max_temp_c: float = 105.0) -> CheckResult:
    """Check declared load currents (A) against the board's fuses (A), traces and copper. ``rail_v`` and
    ``brownout_v`` in volts (V), ``ambient_c`` and ``max_temp_c`` in °C."""
    from . import pcb_doctor as pd

    raw = board_data.encode("utf-8") if isinstance(board_data, str) else bytes(board_data)
    lraw = loads_data.encode("utf-8") if isinstance(loads_data, str) else bytes(loads_data)
    try:
        loads_text = lraw.decode("utf-8-sig")
    except UnicodeDecodeError:
        raise ValueError("loads file is not UTF-8 text: save it as CSV (UTF-8) from your spreadsheet") from None
    board = pd.parse_board(raw, board_name)
    loads = parse_loads_csv(loads_text)

    assignments = pd.auto_assign_net_currents(board)
    by_name = {board.net_name(nid).upper(): nid for nid in board.routed_net_ids()}
    extra = []
    declared_owner = {}
    for ld in loads:
        nid = by_name.get(ld.net.upper())
        if nid is None:
            extra.append(Finding(
                check=f"declared load — {ld.net}", severity=Severity.WARN, subsystems=["electrics", ld.owner],
                message=(f"'{ld.net}' is declared at {ld.current_a:g} A by {ld.owner} (loads file line {ld.line}) "
                         f"but no routed net on the board has that name. Either the two teams name it differently "
                         f"or it is not routed yet, and its fuse and copper are unchecked. FIX: use the board's net "
                         f"name in the loads file, or route the net.")))
            continue
        a = assignments.get(nid) or pd.NetAssignment(net=board.net_name(nid), voltage_v=rail_v)
        a["current_a"] = ld.current_a                      # declaring it clears the 'assumed' flag
        a["source"] = f"declared by {ld.owner} (loads file line {ld.line})"
        if ld.voltage_v is not None:
            a["voltage_v"] = ld.voltage_v
        assignments[nid] = a
        declared_owner[nid] = ld.owner

    report = pd.diagnose(board, assignments, rail_v=rail_v, brownout_v=brownout_v,
                         ambient_c=ambient_c, max_temp_c=max_temp_c)

    nets = []
    for nid in sorted(assignments, key=lambda n: board.net_name(n)):
        a = assignments[nid]
        declared = nid in declared_owner
        nets.append({"net": board.net_name(nid), "current_a": float(a.get("current_a", 0) or 0),
                     "voltage_v": float(a.get("voltage_v", 0) or 0),
                     "provenance": a.get("source", "") if declared else f"assumed — {a.get('source', '')}",
                     "declared": declared})

    fuse_re = re.compile(r"([\d.]+)\s*A", re.I)
    boundary = []
    for nid, owner in sorted(declared_owner.items(), key=lambda kv: board.net_name(kv[0])):
        cur = float(assignments[nid]["current_a"])
        fuses = []
        for fp in board.footprints:
            if (fp.ref or "")[:1].upper() != "F" or not any(p.net == nid for p in fp.pads):
                continue
            m = fuse_re.search(fp.value or "")
            if not m:
                fuses.append((fp.ref, None, "rating not readable from the part value"))
                continue
            rating = float(m.group(1))
            verdict = "FAIL: blows in normal operation" if cur > rating else f"holds ({rating / cur:.2f}× the load)" \
                if cur > 0 else "holds (no declared load)"
            fuses.append((fp.ref, rating, verdict))
        boundary.append({"net": board.net_name(nid), "owner": owner, "current_a": cur, "fuses": fuses})

    findings = sorted(list(report.findings) + extra, key=lambda f: (_ORDER.index(f.severity.value), f.check))
    return CheckResult(
        board_file=Path(board_name).name, board_sha256=hashlib.sha256(raw).hexdigest(),
        loads_file=Path(loads_name).name, loads_sha256=hashlib.sha256(lraw).hexdigest(),
        version=tool_version(),
        params={"rail_v": rail_v, "brownout_v": brownout_v, "ambient_c": ambient_c, "max_temp_c": max_temp_c},
        nets=nets, findings=findings, boundary=boundary)


def render_markdown(res: CheckResult) -> str:
    c = res.counts()
    L = ["# KinematiK consistency check: declared loads vs electrical hardware", "",
         f"Tool version {res.version}. Same inputs and version give this report byte for byte.", "",
         "## Inputs", "",
         "| File | SHA-256 |", "|---|---|",
         f"| {res.board_file} | `{res.board_sha256}` |",
         f"| {res.loads_file} | `{res.loads_sha256}` |", "",
         "Parameters: " + ", ".join(f"{k} = {v:g}" for k, v in res.params.items()), "",
         "## Summary", "",
         " · ".join(f"{c[k]} {_STATUS[k]}" for k in _ORDER), "",
         "## Declared loads vs the hardware that carries them", ""]
    if res.boundary:
        L += ["| Net | Declared by | Current | Fuse | Rating | Verdict |", "|---|---|---|---|---|---|"]
        for b in res.boundary:
            if not b["fuses"]:
                L.append(f"| {b['net']} | {b['owner']} | {b['current_a']:g} A | none on this net | — | "
                         f"copper checks only (below) |")
            for ref, rating, verdict in b["fuses"]:
                L.append(f"| {b['net']} | {b['owner']} | {b['current_a']:g} A | {ref} | "
                         f"{'—' if rating is None else f'{rating:g} A'} | {verdict} |")
    else:
        L.append("No declared load matched a net on the board.")
    L += ["", "## Results", "", "| Status | Check | Evidence behind the method |", "|---|---|---|"]
    for f in res.findings:
        L.append(f"| {_STATUS[f.severity.value]} | {f.check} | {evidence_for(f.check)} |")
    L += ["", "### Details", ""]
    for f in res.findings:
        if f.severity.value in ("fail", "warning", "missing"):
            L += [f"**{_STATUS[f.severity.value]} — {f.check}.** {f.message}", ""]
    L += ["## Where each net's current came from", "",
          "| Net | Current | Voltage | Source |", "|---|---|---|---|"]
    for n in res.nets:
        L.append(f"| {n['net']} | {n['current_a']:g} A | {n['voltage_v']:g} V | {n['provenance']} |")
    L += ["", "## Limits of this report", "",
          "- No result here is correlated with a built board. Measure the real board before relying on a margin.",
          "- Assumed currents are guesses from net names; every one is reported as insufficient evidence.",
          "- Pads and pours are left to the layout tool's DRC; this is not an EMC check or a certification.", ""]
    return "\n".join(L)


def to_json(res: CheckResult) -> dict:
    """The report as plain data: currents and fuse ratings in amps (A), voltages in volts (V)."""
    return {"tool": "kinematik-check", "version": res.version,
            "inputs": {"board": {"file": res.board_file, "sha256": res.board_sha256},
                       "loads": {"file": res.loads_file, "sha256": res.loads_sha256}},
            "params": res.params, "counts": res.counts(),
            "boundary": [{**b, "fuses": [{"ref": r, "rating_a": g, "verdict": v} for r, g, v in b["fuses"]]}
                         for b in res.boundary],
            "findings": [{**f.as_dict(), "status": _STATUS[f.severity.value], "evidence": evidence_for(f.check)}
                         for f in res.findings],
            "nets": res.nets}


def main(argv=None) -> int:
    """Command-line entry point. Returns the exit status, a unitless code: 0 clean, 1 at or above --fail-on,
    2 input error."""
    ap = argparse.ArgumentParser(
        prog="kinematik-check",
        description="Check the currents one team declares against the board another team drew. Runs offline.")
    ap.add_argument("board", help="KiCad .kicad_pcb or Altium .PcbDoc file")
    ap.add_argument("--loads", required=True, help="CSV: net,current_a[,voltage_v][,owner]")
    ap.add_argument("--report", help="write the Markdown report here (default: print it)")
    ap.add_argument("--json", dest="json_path", help="also write a machine-readable JSON report here")
    ap.add_argument("--rail-v", type=float, default=5.0, help="logic rail voltage for brown-out (default 5)")
    ap.add_argument("--brownout-v", type=float, default=4.5, help="brown-out threshold on that rail (default 4.5)")
    ap.add_argument("--ambient-c", type=float, default=40.0, help="ambient temperature, °C (default 40)")
    ap.add_argument("--max-temp-c", type=float, default=105.0, help="copper temperature limit, °C (default 105)")
    ap.add_argument("--fail-on", choices=tuple(_FAIL_ON), default="fail",
                    help="exit 1 if any result at or above this level (default fail; 'never' always exits 0)")
    a = ap.parse_args(argv)
    try:
        res = run_check(Path(a.board).read_bytes(), a.board, Path(a.loads).read_bytes(),
                        a.loads, rail_v=a.rail_v, brownout_v=a.brownout_v, ambient_c=a.ambient_c,
                        max_temp_c=a.max_temp_c)
    except (OSError, ValueError) as e:
        print(f"kinematik-check: {e}", file=sys.stderr)
        return 2
    md = render_markdown(res)
    if a.report:
        Path(a.report).write_text(md, encoding="utf-8")
    else:
        print(md)
    if a.json_path:
        Path(a.json_path).write_text(json.dumps(to_json(res), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    c = res.counts()
    print(" · ".join(f"{c[k]} {_STATUS[k]}" for k in _ORDER), file=sys.stderr)
    return 1 if res.worst_hits(a.fail_on) else 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
