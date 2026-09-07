# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""
brief_coverage.py — make the mission briefing cover EVERY feature, for every
subsystem / goal / proficiency combination.

THE PROBLEM THIS SOLVES
-----------------------
The briefing tailors on three axes — subsystem (role), goal, proficiency — and
that tailoring had three silent holes:

  1. UNREACHABLE TOOLS. A tab can carry complete briefing copy and still be
     reachable from no goal in the questionnaire. It then ships as dead weight:
     written, paid for, never recommended. `daq`, `frames` and `phantom_env`
     were all in this state — the Data Acquisition subteam could not reach the
     Data Acquisition tool.
  2. GOAL-SPECIFIC COPY REPLACING THE FULL LIST. `_briefing_feature_lines` used
     the goal-tailored bullets when they existed and the tool's canonical
     feature list otherwise — never both. So the moment a (goal, tool) pair got
     tailored copy, every capability of that tool the copy did not happen to
     mention became invisible to that goal. Tailoring quietly SHRANK coverage.
  3. ADVANCED PROFICIENCY GETTING NOTHING. With no goal-specific bullets and
     `proficiency == "advanced"`, the resolver returned an empty list: the tool
     appeared in the plan with no features at all.

This module fixes the shape of the problem rather than patching instances:

  * `audit()` is a standing check that every tool is reachable, briefed, and
     has tailored copy for every goal that recommends it. It runs in CI, so a
     new tab added without briefing copy fails the build instead of shipping
     invisible.
  * `resolve_feature_lines()` is the replacement resolution rule. Goal-tailored
     lines lead (they answer "why this tool, for what I said I want"), and the
     tool's canonical capabilities follow so nothing is ever hidden. Proficiency
     controls PRESENTATION — bullets vs a condensed line, plain-English gloss on
     or off — never whether a capability is mentioned.

THE COVERAGE CONTRACT
---------------------
For any (roles, goals, proficiency) the questionnaire can produce:

  * every tool in the plan has at least one feature line, at every proficiency;
  * the union of what is shown spans the tool's complete canonical feature list
    — a canonical line is omitted only when the goal-tailored text demonstrably
    already says it (see `_echoes`), and that test is deliberately conservative:
    when in doubt the line is SHOWN;
  * beginners get more words for the same capabilities, advanced users get
    fewer — neither gets a shorter list of capabilities.

Pure standard library. No Streamlit import, so the whole thing is unit-testable
headless; the app calls `resolve_feature_lines` and passes its own tables in.
"""

from __future__ import annotations

import ast
import os
import re
from dataclasses import dataclass, field
from collections.abc import Sequence

__all__ = [
    "BriefingTables", "CoverageGap", "CoverageReport", "FeaturePlan",
    "audit", "resolve_feature_lines", "effective_role_goals",
    "reachable_tools", "extract_tables", "load_tables_from_app",
    "GAP_KINDS", "PROFICIENCIES", "distinctive_tokens",
]

#: The table names the briefing is assembled from, in the app module.
_TABLE_NAMES = (
    "_TAB_META", "_BRIEF_TOOLS", "_BRIEF_SIMPLE", "_BRIEF_TOOL_FEATURES",
    "_BRIEF_GOAL_FEATURES", "_ROLE_GOALS", "_VERIFY_GOALS", "_BRIEF_PURPOSES",
    "_FREETEXT_KEYWORDS",
)

PROFICIENCIES = ("beginner", "intermediate", "advanced")

#: Gap kinds `audit()` can report, worst first.
GAP_KINDS = (
    "NO_BRIEFING_COPY",      # tab exists, no _BRIEF_TOOLS entry -> dropped from every plan
    "NO_FEATURE_LIST",       # tab has no canonical feature list -> nothing to fall back on
    "UNREACHABLE",           # no role/goal/purpose recommends it -> written but never shown
    "NO_PLAIN_ENGLISH",      # no _BRIEF_SIMPLE gloss -> beginner mode has nothing to add
    "GOAL_PAIR_UNTAILORED",  # a goal recommends the tool with no goal-specific copy
    "NO_FREETEXT_ROUTE",     # the note box can never surface this tool
)


# --------------------------------------------------------------------------- #
#  Table extraction — read the app's literals without importing Streamlit
# --------------------------------------------------------------------------- #
@dataclass
class BriefingTables:
    """
    The briefing's source data, lifted out of the app module.

    Held as a plain object so `audit()` and `resolve_feature_lines()` can be
    driven from a test fixture as easily as from the real app — the coverage
    rules are then testable without a 1.7 MB import.
    """
    tab_meta: dict = field(default_factory=dict)
    tools: dict = field(default_factory=dict)
    simple: dict = field(default_factory=dict)
    tool_features: dict = field(default_factory=dict)
    goal_features: dict = field(default_factory=dict)
    role_goals: dict = field(default_factory=dict)
    verify_goals: list = field(default_factory=list)
    purposes: list = field(default_factory=list)
    freetext: list = field(default_factory=list)

    @property
    def tool_ids(self) -> set:
        return set(self.tab_meta)

    def label(self, tool_id: str) -> str:
        meta = self.tab_meta.get(tool_id)
        return f"{meta[0]} {meta[1]}" if meta else tool_id


def extract_tables(source: str) -> BriefingTables:
    """
    Pull the briefing tables out of app source text via the AST.

    `_ROLE_GOALS` is returned as WRITTEN — the app appends `_VERIFY_GOALS` to
    each role afterwards in a loop, which `literal_eval` cannot see. Use
    `effective_role_goals()` to get what the questionnaire actually offers.
    """
    tree = ast.parse(source)
    found: dict = {}
    for node in tree.body:
        if not isinstance(node, (ast.Assign, ast.AnnAssign)):
            continue
        target = node.targets[0] if isinstance(node, ast.Assign) else node.target
        if not isinstance(target, ast.Name) or target.id not in _TABLE_NAMES:
            continue
        if node.value is None:
            continue
        try:
            found[target.id] = ast.literal_eval(node.value)
        except (ValueError, TypeError):
            continue
    return BriefingTables(
        tab_meta=found.get("_TAB_META", {}),
        tools=found.get("_BRIEF_TOOLS", {}),
        simple=found.get("_BRIEF_SIMPLE", {}),
        tool_features=found.get("_BRIEF_TOOL_FEATURES", {}),
        goal_features=found.get("_BRIEF_GOAL_FEATURES", {}),
        role_goals=found.get("_ROLE_GOALS", {}),
        verify_goals=found.get("_VERIFY_GOALS", []),
        purposes=found.get("_BRIEF_PURPOSES", []),
        freetext=found.get("_FREETEXT_KEYWORDS", []),
    )


def load_tables_from_app(path: str | None = None) -> BriefingTables:
    """Extract the tables from streamlit_app.py (repo root by default)."""
    if path is None:
        here = os.path.dirname(os.path.abspath(__file__))
        for candidate in (os.path.join(os.path.dirname(here), "streamlit_app.py"),
                          os.path.join(here, "streamlit_app.py")):
            if os.path.exists(candidate):
                path = candidate
                break
    if not path or not os.path.exists(path):
        raise FileNotFoundError("could not locate streamlit_app.py")
    with open(path, encoding="utf-8") as fh:
        return extract_tables(fh.read())


# --------------------------------------------------------------------------- #
#  What the questionnaire can actually offer
# --------------------------------------------------------------------------- #
def effective_role_goals(tables: BriefingTables) -> dict:
    """
    role -> [(goal_key, label, tool_ids)] as the questionnaire offers it.

    Mirrors the app's append loop: the cross-cutting verification goals are
    added to every subteam, skipping any key the subteam already defines.
    """
    out: dict = {}
    for role, goals in tables.role_goals.items():
        merged = [tuple(g) for g in goals]
        existing = {g[0] for g in merged}
        for vg in tables.verify_goals:
            if vg[0] not in existing:
                merged.append(tuple(vg))
        out[role] = merged
    return out


def reachable_tools(tables: BriefingTables) -> dict:
    """
    tool_id -> set of (source, key) that can put it in a plan.

    Sources are the role name for a subteam goal, `*verify*` for a cross-cutting
    goal, `*purpose*` for question 2, and `*note*` for a free-text keyword route.
    """
    reach: dict = {}

    def add(tool_id, source, key):
        reach.setdefault(tool_id, set()).add((source, key))

    for role, goals in effective_role_goals(tables).items():
        for goal_key, _label, tool_ids in goals:
            source = "*verify*" if any(goal_key == v[0] for v in tables.verify_goals) else role
            for tool_id in tool_ids:
                add(tool_id, source, goal_key)
    for entry in tables.purposes:
        key, _label, tool_ids, _line = entry
        for tool_id in tool_ids:
            add(tool_id, "*purpose*", key)
    for keywords, tool_id, _advanced in tables.freetext:
        if tool_id != "any":
            add(tool_id, "*note*", keywords[0] if keywords else tool_id)
    return reach


# --------------------------------------------------------------------------- #
#  The audit
# --------------------------------------------------------------------------- #
@dataclass
class CoverageGap:
    """One hole in the briefing's coverage, with enough detail to fix it."""
    kind: str
    tool: str
    detail: str
    goal: str = ""
    role: str = ""

    def __str__(self) -> str:
        where = f" [{self.role or '*'}/{self.goal}]" if self.goal else ""
        return f"{self.kind}: {self.tool}{where} — {self.detail}"


@dataclass
class CoverageReport:
    """The audit's verdict, plus the numbers behind it."""
    gaps: list = field(default_factory=list)
    n_tools: int = 0
    n_goals: int = 0
    n_goal_tool_pairs: int = 0
    per_role_tools: dict = field(default_factory=dict)
    reach: dict = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return not self.gaps

    def of_kind(self, kind: str) -> list:
        return [g for g in self.gaps if g.kind == kind]

    def tally(self) -> dict:
        counts = {k: 0 for k in GAP_KINDS}
        for g in self.gaps:
            counts[g.kind] = counts.get(g.kind, 0) + 1
        return {k: v for k, v in counts.items() if v}

    def summary(self) -> str:
        lines = [
            f"{self.n_tools} tool(s), {self.n_goals} goal(s), "
            f"{self.n_goal_tool_pairs} (goal, tool) pair(s) offered",
        ]
        if self.ok:
            lines.append("COVERAGE COMPLETE — every tool reachable, briefed and "
                         "tailored for every goal that recommends it")
            return "\n".join(lines)
        lines.append(f"{len(self.gaps)} coverage gap(s):")
        for kind, count in self.tally().items():
            lines.append(f"  {kind}: {count}")
            for gap in self.of_kind(kind):
                lines.append(f"      - {gap}")
        return "\n".join(lines)


def audit(tables: BriefingTables,
          require_freetext: bool = True,
          require_plain_english: bool = True) -> CoverageReport:
    """
    Check the briefing's coverage contract and report every hole.

    `require_freetext` / `require_plain_english` are switchable because they are
    a lower tier of obligation than reachability: a tool with no keyword route is
    still recommendable from a goal, it just cannot be summoned by typing about
    it. Both default on — that is the standard the feature is held to.
    """
    gaps: list = []
    tool_ids = tables.tool_ids
    reach = reachable_tools(tables)
    role_goals = effective_role_goals(tables)

    for tool_id in sorted(tool_ids):
        label = tables.label(tool_id)
        if tool_id not in tables.tools:
            gaps.append(CoverageGap(
                "NO_BRIEFING_COPY", tool_id,
                f"{label} has no _BRIEF_TOOLS entry, so _briefing_ordered_tools "
                f"filters it out of every plan — it can never be recommended"))
        if not tables.tool_features.get(tool_id):
            gaps.append(CoverageGap(
                "NO_FEATURE_LIST", tool_id,
                f"{label} has no _BRIEF_TOOL_FEATURES list, so a goal without "
                f"tailored copy leaves it with no features at all"))
        if require_plain_english and tool_id not in tables.simple:
            gaps.append(CoverageGap(
                "NO_PLAIN_ENGLISH", tool_id,
                f"{label} has no _BRIEF_SIMPLE gloss, so beginner/new-member "
                f"mode has nothing extra to say about it"))
        if tool_id not in reach:
            gaps.append(CoverageGap(
                "UNREACHABLE", tool_id,
                f"{label} is recommended by no role goal, purpose or note "
                f"keyword — it is written but unreachable"))
        elif require_freetext and not any(s == "*note*" for s, _k in reach[tool_id]):
            gaps.append(CoverageGap(
                "NO_FREETEXT_ROUTE", tool_id,
                f"{label} has no _FREETEXT_KEYWORDS route, so a member who "
                f"types about it in the note box will not be given it"))

    # Every (goal, tool) the questionnaire can offer needs tailored copy, or the
    # briefing falls back to generic text for a goal it explicitly knows about.
    seen_pairs: set = set()
    goal_keys: set = set()
    for role, goals in role_goals.items():
        for goal_key, _label, tool_ids_for_goal in goals:
            goal_keys.add(goal_key)
            is_verify = any(goal_key == v[0] for v in tables.verify_goals)
            for tool_id in tool_ids_for_goal:
                if (goal_key, tool_id) in seen_pairs:
                    continue
                seen_pairs.add((goal_key, tool_id))
                if not tables.goal_features.get((goal_key, tool_id)):
                    gaps.append(CoverageGap(
                        "GOAL_PAIR_UNTAILORED", tool_id,
                        f"goal {goal_key!r} recommends {tables.label(tool_id)} "
                        f"with no _BRIEF_GOAL_FEATURES entry — the member is "
                        f"told to open it without being told what it does for "
                        f"the thing they asked for",
                        goal=goal_key, role="*verify*" if is_verify else role))

    per_role = {}
    for role, goals in role_goals.items():
        covered = set()
        for _gk, _lab, ids in goals:
            covered.update(ids)
        per_role[role] = sorted(covered)

    return CoverageReport(
        gaps=gaps, n_tools=len(tool_ids), n_goals=len(goal_keys),
        n_goal_tool_pairs=len(seen_pairs), per_role_tools=per_role, reach=reach,
    )


# --------------------------------------------------------------------------- #
#  The resolver — goal-tailored first, complete always
# --------------------------------------------------------------------------- #
_STOP = {
    "the", "and", "for", "with", "that", "this", "from", "into", "onto", "your",
    "you", "its", "it", "a", "an", "of", "to", "in", "on", "at", "is", "are",
    "be", "as", "so", "or", "not", "no", "but", "can", "will", "get", "gets",
    "one", "every", "each", "any", "all", "than", "then", "when", "what",
    "which", "how", "why", "who", "them", "they", "their", "there", "here",
    "own", "out", "up", "off", "over", "under", "before", "after", "while",
    "instead", "rather", "just", "only", "also", "same", "other", "more",
    "most", "less", "least", "very", "much", "many", "some", "way", "does",
    "do", "did", "has", "have", "had", "was", "were", "been", "being", "by",
}


def distinctive_tokens(text: str) -> set:
    """
    Content words of four or more characters, lower-cased, stop-words removed.

    Used only to decide whether a goal-tailored line already SAYS what a
    canonical feature line says. Deliberately crude — see `_echoes` for why
    crude is the safe direction here.
    """
    if not text:
        return set()
    words = re.findall(r"[a-z0-9][a-z0-9\-]{3,}", str(text).lower())
    return {w for w in words if w not in _STOP}


#: Fraction of a canonical line's distinctive tokens that must already appear in
#: the goal-tailored text before that line is treated as redundant. Set high on
#: purpose: the failure mode we care about is HIDING a capability, and a
#: duplicated sentence is a far cheaper mistake than a missing one.
_ECHO_THRESHOLD = 0.75


def _echoes(canonical_line: str, tailored_text_tokens: set) -> bool:
    """True if the tailored copy already covers this canonical feature line."""
    tokens = distinctive_tokens(canonical_line)
    if len(tokens) < 4:
        return False                     # too short to judge — show it
    overlap = len(tokens & tailored_text_tokens) / len(tokens)
    return overlap >= _ECHO_THRESHOLD


@dataclass
class FeaturePlan:
    """
    What to show for one tool, for one member.

    `primary`   goal-tailored lines — why this tool, for the goal they picked.
    `additional` canonical capabilities the tailored lines did not already say.
    Together they are the tool's COMPLETE capability set, at every proficiency.
    """
    tool: str
    primary: list = field(default_factory=list)
    additional: list = field(default_factory=list)
    proficiency: str = "intermediate"
    tailored: bool = False               # did any goal supply copy for this tool?
    n_canonical: int = 0                 # size of the tool's canonical list
    n_canonical_shown: int = 0           # how many of those are in `additional`
    n_canonical_echoed: int = 0          # ...and how many the tailored copy said

    @property
    def all_lines(self) -> list:
        return list(self.primary) + list(self.additional)

    @property
    def complete(self) -> bool:
        """Every canonical capability is either shown or demonstrably echoed."""
        return self.n_canonical_shown + self.n_canonical_echoed >= self.n_canonical

    def additional_heading(self) -> str:
        """Label for the `additional` block, in this member's register."""
        if not self.primary:
            return ""
        if self.proficiency == "beginner":
            return "This tool can also do:"
        if self.proficiency == "advanced":
            return "Also:"
        return "Also in this tool:"

    def additional_as_text(self) -> str:
        """
        The `additional` block rendered for this proficiency: bullets for
        beginner/intermediate, one condensed line for advanced.

        Advanced gets FEWER WORDS for the same capabilities — never fewer
        capabilities. That distinction is the whole point of this class.
        """
        if not self.additional:
            return ""
        if self.proficiency == "advanced":
            parts = [line.rstrip(".") for line in self.additional]
            return f"{self.additional_heading()} " + "; ".join(parts) + "."
        return "\n".join(f"- {line}" for line in self.additional)


def resolve_feature_lines(tool_id: str,
                          goal_keys: Sequence[str],
                          tables: BriefingTables,
                          proficiency: str = "intermediate") -> FeaturePlan:
    """
    Resolve the feature lines for one tool, guaranteeing complete coverage.

        plan = resolve_feature_lines("kinematics", ["susp_geo"], tables, "advanced")
        plan.primary        # goal-tailored: why this tool for THIS goal
        plan.additional     # the rest of what it does — never suppressed
        plan.complete       # True: nothing was hidden

    Replaces the old rule (goal copy XOR canonical list, and nothing at all for
    an advanced member with no goal copy), under which tailoring a (goal, tool)
    pair silently hid every capability the tailored copy did not mention.
    """
    proficiency = proficiency if proficiency in PROFICIENCIES else "intermediate"

    primary, seen = [], set()
    for goal_key in goal_keys or []:
        for line in tables.goal_features.get((goal_key, tool_id), []):
            if line not in seen:
                seen.add(line)
                primary.append(line)

    canonical = list(tables.tool_features.get(tool_id, []))
    tailored_tokens: set = set()
    for line in primary:
        tailored_tokens |= distinctive_tokens(line)

    additional, echoed = [], 0
    for line in canonical:
        if line in seen:
            echoed += 1
            continue
        if primary and _echoes(line, tailored_tokens):
            echoed += 1
            continue
        additional.append(line)

    # A tool with neither tailored copy nor a canonical list would render as a
    # bare name. Fall back to its one-line briefing summary so the member is
    # never told to open something with no explanation of what it is.
    if not primary and not additional:
        blurb = tables.tools.get(tool_id)
        if blurb:
            additional = [blurb[0] if isinstance(blurb, (tuple, list)) else str(blurb)]

    return FeaturePlan(
        tool=tool_id, primary=primary, additional=additional,
        proficiency=proficiency, tailored=bool(primary),
        n_canonical=len(canonical), n_canonical_shown=len(additional),
        n_canonical_echoed=echoed,
    )


# --------------------------------------------------------------------------- #
#  CLI — `python -m suspension.brief_coverage`
# --------------------------------------------------------------------------- #
def _main(argv: Sequence[str] | None = None) -> int:
    import argparse
    ap = argparse.ArgumentParser(
        description="Audit mission-briefing coverage across subsystem x goal x "
                    "proficiency.")
    ap.add_argument("app", nargs="?", default=None,
                    help="path to streamlit_app.py (default: repo root)")
    ap.add_argument("--no-freetext", action="store_true",
                    help="don't require a note-box keyword route per tool")
    ap.add_argument("--no-plain-english", action="store_true",
                    help="don't require a _BRIEF_SIMPLE gloss per tool")
    ap.add_argument("--features", action="store_true",
                    help="also print per-(goal, tool) feature-plan coverage")
    args = ap.parse_args(argv)

    tables = load_tables_from_app(args.app)
    report = audit(tables,
                   require_freetext=not args.no_freetext,
                   require_plain_english=not args.no_plain_english)
    print(report.summary())

    if args.features:
        print("\nFeature-plan coverage (every goal x tool x proficiency):")
        incomplete = 0
        for role, goals in sorted(effective_role_goals(tables).items()):
            for goal_key, _label, tool_ids in goals:
                for tool_id in tool_ids:
                    for prof in PROFICIENCIES:
                        plan = resolve_feature_lines(tool_id, [goal_key],
                                                     tables, prof)
                        if not plan.all_lines or not plan.complete:
                            incomplete += 1
                            print(f"  INCOMPLETE {role}/{goal_key}/{tool_id}"
                                  f"/{prof}: {len(plan.all_lines)} line(s)")
        print(f"  {'all complete' if not incomplete else str(incomplete) + ' incomplete'}")

    return 0 if report.ok else 1


if __name__ == "__main__":                                   # pragma: no cover
    raise SystemExit(_main())
