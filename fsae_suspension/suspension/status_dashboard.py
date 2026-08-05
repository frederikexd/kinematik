# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""
status_dashboard.py — the front-page "is the car ready?" validator
==================================================================

WHAT THIS IS
------------
A *metadata validator*, NOT a CAD parser. The user enters a handful of key
numbers when they register a part (Weight, Offset, Clash distance, …); this
module checks those numbers against simple, declared rules and rolls every
component up into one at-a-glance verdict:

    GREEN  — ready for manufacturing / assembly
    AMBER  — usable but something is soft (an estimate, a missing sign-off)
    RED    — needs attention (file missing, a rule failed, a clash, a failing myth)

The point is the front page. When a lead opens KinematiK and sees the whole
car's status as a row of red/green chips, the "which Drive link is current and
is it actually OK?" question is answered before anyone asks it — so the Drive
folder stops being the source of truth.

WHY A SEPARATE MODULE
---------------------
It reads the Registry's existing ``summary_rows()`` and a component's declared
rules; it never touches geometry. Keeping it standalone means:
  * no CAD parsing, ever — just number-vs-threshold comparisons;
  * the rules are DATA (a list of small dicts per component), so a discipline
    lead adds a check without touching engine code;
  * it's pure and unit-testable headless (no Streamlit, no DB).

RULE MODEL
----------
A rule is a tiny dict::

    {"param": "Weight", "op": "<=", "value": 2.5, "unit": "kg",
     "label": "Under mass budget"}

``param`` names a key the user entered in the component's ``specs``. The
validator pulls the number out of the spec string (handles "2.3 kg",
"42.0 mm", "1.8"), applies ``op`` against ``value``, and records pass/fail.
Supported ops: <, <=, >, >=, ==, !=, and "between" (value = [lo, hi]).

A component is RED if any rule fails or its file is missing; AMBER if it has no
file-backed verified version or a rule's input is missing (can't be checked);
GREEN only when it has a verified file AND every rule passes.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Any

# Verdict ranking so a car-level rollup can take the worst component.
GREEN, AMBER, RED = "green", "amber", "red"
_RANK = {GREEN: 0, AMBER: 1, RED: 2}


# --------------------------------------------------------------------------- #
#  Number extraction — pull a float out of a spec value like "42.0 mm"        #
# --------------------------------------------------------------------------- #
_NUM_RX = re.compile(r'-?\d[\d,]*\.?\d*')


def coerce_number(value: Any) -> float | None:
    """Best-effort float from a spec value. Returns None if there's no number
    (so the rule reports 'can't check' rather than guessing)."""
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, dict):
        return coerce_number(value.get("value", value.get("val")))
    m = _NUM_RX.search(str(value))
    if not m:
        return None
    try:
        return float(m.group(0).replace(",", ""))
    except ValueError:
        return None


# --------------------------------------------------------------------------- #
#  Spec-key resolution                                                         #
#                                                                              #
#  A rule's ``param`` and a component's spec keys are typed into two different  #
#  free-text boxes, by two different people, weeks apart. Matching them with a  #
#  bare ``specs.get(param)`` means "Mass" and "mass" — or "Wall thickness" and  #
#  "Wall Thickness" — never meet, and the rule reports "not declared yet"       #
#  forever while the number sits right there in the row.                        #
#                                                                              #
#  That failure is invisible in the worst way: the board goes amber and STAYS   #
#  amber, which at a design review reads as "this team didn't finish" rather    #
#  than "these two strings disagree about a capital letter". The quick-add rule #
#  templates make it likelier, not less: they hard-code "Weight", so a team     #
#  that entered "Mass" gets a permanently unresolvable check the moment they    #
#  use one.                                                                     #
#                                                                              #
#  So: match exactly, then case/whitespace-insensitively, then report the       #
#  closest key as a suggestion. The suggestion matters as much as the match —   #
#  "'Wieght' not declared; did you mean 'Weight'?" is a ten-second fix, and     #
#  "'Wieght' not declared yet" is a dead end.                                   #
# --------------------------------------------------------------------------- #
def _norm_key(k: str) -> str:
    """Case- and whitespace-insensitive form of a spec key."""
    return re.sub(r"[\s_-]+", " ", str(k or "")).strip().lower()


def resolve_spec(param: str, specs: dict):
    """Find the spec entry a rule's ``param`` refers to.

    Returns ``(value, matched_key, suggestion)``:
      * exact or normalized hit -> (value, the real key, None)
      * no hit, close key exists -> (None, None, that key)
      * no hit at all            -> (None, None, None)

    Deliberately does NOT match on substrings or synonyms. "Wall thickness"
    resolving to "Min wall thickness" would silently check the wrong number,
    which is worse than the amber it replaces — a validator may not guess.
    """
    specs = specs or {}
    if param in specs:                                  # exact
        return specs[param], param, None
    want = _norm_key(param)
    if not want:
        return None, None, None
    for k in specs:                                     # case / spacing
        if _norm_key(k) == want:
            return specs[k], k, None
    try:                                                # near miss -> suggest
        import difflib
        pool = {_norm_key(k): k for k in specs}
        hit = difflib.get_close_matches(want, list(pool), n=1, cutoff=0.82)
        if hit:
            return None, None, pool[hit[0]]
    except Exception:
        pass
    return None, None, None


# --------------------------------------------------------------------------- #
#  A single rule and its evaluation                                            #
# --------------------------------------------------------------------------- #
_OPS = {
    "<":  lambda a, b: a < b,
    "<=": lambda a, b: a <= b,
    ">":  lambda a, b: a > b,
    ">=": lambda a, b: a >= b,
    "==": lambda a, b: a == b,
    "!=": lambda a, b: a != b,
}


@dataclass
class RuleResult:
    param: str
    label: str
    op: str
    target: Any
    unit: str
    actual: float | None
    status: str          # green | amber | red
    message: str


def evaluate_rule(rule: dict, specs: dict) -> RuleResult:
    """Check one rule against the component's declared specs.

    A missing input is AMBER ("declare it"), not RED — you can't fail a check you
    never supplied. A present-but-failing input is RED.
    """
    param = rule.get("param", "")
    label = rule.get("label", param)
    op = rule.get("op", "<=")
    target = rule.get("value")
    unit = rule.get("unit", "")

    raw, matched_key, suggestion = resolve_spec(param, specs)
    actual = coerce_number(raw)

    if actual is None:
        if suggestion:
            msg = (f"“{param}” not declared — did you mean “{suggestion}”? "
                   f"Rename the rule or the spec so they match.")
        elif matched_key is not None:
            # The key resolved but its value has no number in it, e.g. the
            # user typed "TBD" or "see drawing". That is a different problem
            # from never declaring it, and saying so saves a hunt.
            msg = (f"“{matched_key}” is declared as “{raw}” — no number to "
                   f"check. Enter a value like “2.4 kg”.")
        else:
            msg = f"“{param}” not declared yet — enter it to check."
        return RuleResult(param, label, op, target, unit, None, AMBER, msg)

    # between [lo, hi]
    if op == "between" and isinstance(target, (list, tuple)) and len(target) == 2:
        lo, hi = float(target[0]), float(target[1])
        ok = lo <= actual <= hi
        msg = (f"{actual:g}{_u(unit)} in [{lo:g}, {hi:g}]" if ok
               else f"{actual:g}{_u(unit)} outside [{lo:g}, {hi:g}]")
        return RuleResult(param, label, op, target, unit, actual,
                          GREEN if ok else RED, msg)

    fn = _OPS.get(op)
    tnum = coerce_number(target)
    if fn is None or tnum is None:
        return RuleResult(param, label, op, target, unit, actual, AMBER,
                          f"rule for “{param}” is malformed.")
    ok = fn(actual, tnum)
    msg = (f"{actual:g}{_u(unit)} {op} {tnum:g}{_u(unit)}" if ok
           else f"{actual:g}{_u(unit)} fails {op} {tnum:g}{_u(unit)}")
    return RuleResult(param, label, op, target, unit, actual,
                      GREEN if ok else RED, msg)


def _u(unit: str) -> str:
    return f" {unit}" if unit else ""


# --------------------------------------------------------------------------- #
#  Component-level status                                                      #
# --------------------------------------------------------------------------- #
@dataclass
class ComponentStatus:
    comp_id: str
    name: str
    subteam: str
    status: str                       # green | amber | red
    headline: str                     # one-line reason
    has_file: bool
    verified: bool
    rule_results: list = field(default_factory=list)   # list[RuleResult]
    reasons: list = field(default_factory=list)        # the red/amber drivers
    specs: dict = field(default_factory=dict)


def status_for_component(row: dict, rules: list) -> ComponentStatus:
    """Roll one Registry summary row + its rules into a single verdict.

    ``row`` is a dict from ``Registry.summary_rows()`` (name, subteam, status,
    specs, has_file, …). ``rules`` is the component's declared rule list.
    """
    name = row.get("name", "?")
    subteam = row.get("subteam", "")
    specs = row.get("specs", {}) or {}
    has_file = bool(row.get("has_file"))
    verified = row.get("status") == "verified"

    reasons: list[str] = []
    worst = GREEN

    # 1) file presence — the most basic "is this real" check
    if not has_file:
        worst = RED
        reasons.append("No file or link registered")

    # 2) the declared metadata rules
    results = [evaluate_rule(r, specs) for r in (rules or [])]
    for rr in results:
        if rr.status == RED:
            worst = RED
            reasons.append(f"{rr.label}: {rr.message}")
        elif rr.status == AMBER and worst != RED:
            worst = AMBER
            reasons.append(f"{rr.label}: {rr.message}")

    # 3) sign-off — a file that passes rules but isn't verified is AMBER, not GREEN
    if worst == GREEN and not verified:
        worst = AMBER
        reasons.append("Passes checks but not signed off yet")

    if worst == GREEN:
        headline = "Ready for manufacturing"
    elif worst == AMBER:
        headline = reasons[0] if reasons else "Almost ready"
    else:
        headline = reasons[0] if reasons else "Needs attention"

    return ComponentStatus(
        comp_id=row.get("comp_id", ""), name=name, subteam=subteam,
        status=worst, headline=headline, has_file=has_file, verified=verified,
        rule_results=results, reasons=reasons, specs=specs)


# --------------------------------------------------------------------------- #
#  Car-level rollup                                                            #
# --------------------------------------------------------------------------- #
@dataclass
class CarStatus:
    overall: str                      # green | amber | red
    headline: str
    counts: dict                      # {"green": n, "amber": n, "red": n}
    components: list                  # list[ComponentStatus]
    extra_flags: list = field(default_factory=list)   # myths/clashes from other systems
    ready_pct: float = 0.0

    @property
    def is_ready(self) -> bool:
        return self.overall == GREEN


def roll_up(component_statuses: list, extra_flags: list | None = None) -> CarStatus:
    """Combine every component status (plus any cross-system flags like failing
    myths or detected clashes) into the single front-page verdict."""
    extra_flags = list(extra_flags or [])
    counts = {GREEN: 0, AMBER: 0, RED: 0}
    for cs in component_statuses:
        counts[cs.status] = counts.get(cs.status, 0) + 1

    # extra flags (each a dict with a 'status' and 'message') push the overall
    worst = GREEN
    for cs in component_statuses:
        if _RANK[cs.status] > _RANK[worst]:
            worst = cs.status
    for fl in extra_flags:
        s = fl.get("status", RED)
        if _RANK.get(s, 2) > _RANK[worst]:
            worst = s

    total = len(component_statuses)
    ready = counts[GREEN]
    ready_pct = (100.0 * ready / total) if total else 0.0

    n_red = counts[RED] + sum(1 for f in extra_flags if f.get("status") == RED)
    if worst == GREEN and total > 0:
        headline = "All systems ready for manufacturing"
    elif worst == RED:
        headline = f"{n_red} item{'s' if n_red != 1 else ''} need attention before cut"
    elif total == 0:
        headline = "No components registered yet"
        worst = AMBER
    else:
        headline = f"{counts[AMBER]} item{'s' if counts[AMBER] != 1 else ''} almost ready"

    return CarStatus(overall=worst, headline=headline, counts=counts,
                     components=component_statuses, extra_flags=extra_flags,
                     ready_pct=ready_pct)


# --------------------------------------------------------------------------- #
#  Default rule templates — what a lead starts from per common part           #
# --------------------------------------------------------------------------- #
# These are STARTERS the UI offers; a lead edits the numbers for their car.
# Param names match the spec keys the Registry tab prompts for.
DEFAULT_RULE_TEMPLATES = {
    "Weight":         {"param": "Weight", "op": "<=", "value": 2.5, "unit": "kg",
                       "label": "Under mass budget"},
    "Offset":         {"param": "Offset", "op": "between", "value": [41.0, 43.0],
                       "unit": "mm", "label": "Offset within tolerance"},
    "Clash distance": {"param": "Clash distance", "op": ">=", "value": 2.0,
                       "unit": "mm", "label": "Clearance to neighbours"},
    "FoS":            {"param": "FoS", "op": ">=", "value": 1.5, "unit": "",
                       "label": "Factor of safety ≥ 1.5"},
    "Wall thickness": {"param": "Wall thickness", "op": ">=", "value": 2.0,
                       "unit": "mm", "label": "Min wall thickness"},
}


def template_for(param: str) -> dict:
    """Return a starter rule dict for a known parameter, or a generic one."""
    return dict(DEFAULT_RULE_TEMPLATES.get(
        param, {"param": param, "op": ">=", "value": 0.0, "unit": "",
                "label": param}))
