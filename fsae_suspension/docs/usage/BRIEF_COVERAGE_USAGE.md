# Mission-briefing coverage — usage

`suspension/brief_coverage.py`

Guarantees the mission briefing covers **every feature of every tool**, for every
combination of subsystem, goal and proficiency the questionnaire can produce — and
makes a gap a build failure rather than something a member notices six months later.

```bash
python -m suspension.brief_coverage             # audit the app, exit 1 on any gap
python -m suspension.brief_coverage --features  # also sweep every goal x tool x proficiency
```

```
40 tool(s), 40 goal(s), 83 (goal, tool) pair(s) offered
COVERAGE COMPLETE — every tool reachable, briefed and tailored for every goal that recommends it
```

---

## What was leaking

The briefing tailors on three axes — subsystem (role), goal, proficiency. Tailoring
is exactly where coverage leaks, and it was leaking three ways.

### 1. Tailoring shrank coverage

The old resolver was `goal-specific copy XOR canonical feature list`:

```python
# before
lines = [goal-specific bullets for the active goals]
if not lines and proficiency != "advanced":
    lines = list(_BRIEF_TOOL_FEATURES[tool])      # only as a fallback
```

So the moment a `(goal, tool)` pair got tailored copy, every capability that copy
did not happen to mention became **invisible for that goal**. Writing better copy
made the briefing cover less. A powertrain member on the energy-budget goal saw 5
tailored lines about Track Testing and never learned it overlays test data at all.

### 2. Advanced proficiency got nothing

With no tailored copy and `proficiency == "advanced"`, the resolver returned `[]`.
The tool appeared in the plan with a name, a rationale, and no features.

### 3. Three tools were unreachable

`daq`, `frames` and `phantom_env` carried complete briefing copy — description,
plain-English gloss, full feature list — and were recommended by no goal, purpose
or note keyword. Most pointedly, the **Data Acquisition subteam could not reach the
Data Acquisition tool**: both of its spec goals pointed only at the PCB tab.

---

## The rule that replaced it

`resolve_feature_lines()` returns a `FeaturePlan` with two blocks:

| Field | What it holds |
|---|---|
| `primary` | Goal-tailored lines — why this tool, for the goal they picked |
| `additional` | Every remaining canonical capability the tailored copy did not already state |

Together they are the tool's **complete capability set**, at every proficiency.
Tailoring now *adds relevance* instead of *subtracting coverage*.

```python
from suspension.brief_coverage import load_tables_from_app, resolve_feature_lines

tables = load_tables_from_app()
plan = resolve_feature_lines("laptime", ["aero_map"], tables, "advanced")

plan.primary      # 3 goal-tailored lines
plan.additional   # 4 more capabilities the old rule hid
plan.complete     # True — nothing was dropped
```

### Proficiency changes words, not capabilities

This is the distinction the redesign rests on, and it is a test:

| Proficiency | `primary` | `additional` | Extra |
|---|---|---|---|
| beginner | bullets | bullets | plain-English gloss, "no wrong moves" note |
| intermediate | bullets | bullets under *"Also in this tool:"* | rationale + vs-MATLAB/ANSYS quote |
| advanced | bullets | **one condensed line** under *"Also:"* | rationale collapsed |

An advanced member gets a terser briefing, never a smaller one.

### Not repeating what the tailored copy already said

`additional` omits a canonical line only when the tailored text demonstrably
already states it, measured by distinctive-token overlap (`_ECHO_THRESHOLD = 0.75`).
The threshold is deliberately high and the test deliberately crude, because the
two possible mistakes are not symmetric: **a duplicated sentence is cheap, a hidden
capability is the bug we are fixing.** When in doubt, the line is shown.

In practice this behaves well. Kinematics under `susp_geo` has 7 tailored lines
that between them cover all 6 canonical capabilities, so `additional` is correctly
empty — no padding. Track Testing under `aero_map` has 3 tailored lines covering
none of its 4 canonical capabilities, so all 4 are added.

---

## The audit

`audit(tables)` reports six kinds of gap:

| Kind | Meaning |
|---|---|
| `NO_BRIEFING_COPY` | Tab exists with no `_BRIEF_TOOLS` entry — filtered out of every plan |
| `NO_FEATURE_LIST` | No `_BRIEF_TOOL_FEATURES` — nothing to fall back on |
| `UNREACHABLE` | No role goal, purpose or note keyword recommends it |
| `NO_PLAIN_ENGLISH` | No `_BRIEF_SIMPLE` gloss — beginner mode has nothing extra to say |
| `GOAL_PAIR_UNTAILORED` | A goal recommends the tool with no goal-specific copy |
| `NO_FREETEXT_ROUTE` | The note box can never surface this tool |

Each gap names the tool, the goal and the role, and says what the member
experiences as a result — so the report is a work list, not a lint score.

The last two are switchable (`require_freetext=False`, `require_plain_english=False`)
because they are a lower tier of obligation than reachability. Both default on.

---

## What changed in the app

Both copies of `streamlit_app.py` (repo root and `suspension/`) were patched
identically; a test asserts their briefing tables stay in sync.

**Reachability**
- `dq_spec` / `dq_int` now include `daq` — the DAQ subteam reaches the DAQ tool
- `ch_fit` and `susp_verify` now include `phantom_env`
- new cross-cutting goal `vg_frames` → `frames`, appended to every subteam

**Tailored copy** — 17 new `_BRIEF_GOAL_FEATURES` entries covering every previously
untailored `(goal, tool)` pair, written from each tool's own canonical feature list.
No invented capabilities.

**Note-box routes** — 17 new `_FREETEXT_KEYWORDS` entries, so every tool is
summonable by typing the problem it solves ("what breaks first" → Fusebox,
"aliasing" → DAQ, "rubs" → Phantom Envelope).

**Code** — `_briefing_feature_lines` and the unified HTML/audio renderer both
delegate to `resolve_feature_lines`. The resolver degrades to the old canonical
list if the module is somehow absent, so a missing import can never blank a
briefing.

---

## Tests

`tests/test_brief_coverage.py` — 36 tests. The load-bearing ones:

- `test_briefing_coverage_has_no_gaps` — the audit, run against **both** app copies
- `test_every_goal_tool_proficiency_combination_yields_features` — the full sweep;
  this is what would have caught advanced-with-no-copy returning nothing
- `test_every_combination_spans_the_full_capability_set` — coverage, not just
  non-emptiness
- `test_proficiency_changes_words_not_capabilities` — capability count is identical
  across all three levels; only the rendering differs
- `test_both_app_copies_carry_identical_briefing_tables` — the copies drift, and a
  fix applied to one is a fix half the users never get
- `test_note_keywords_are_ascii` — regression for a Cyrillic homoglyph that slipped
  into a keyword during this very fix (`"rубs"`), which parses, lints, and can never
  match

Two defects in pre-existing copy surfaced while writing these and were fixed: one
feature line missing terminal punctuation (the audio briefing reads these aloud),
and the homoglyph above.

`tests/test_mission_briefing.py` needed one change: its exec'd namespace list gained
`_briefing_feature_plan` and `_briefing_tables`.
