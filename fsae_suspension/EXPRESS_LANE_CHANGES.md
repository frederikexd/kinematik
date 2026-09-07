# Express Lane — what changed in this project

Everything here is additive except a 12-line insertion into `streamlit_app.py`.
**437 of your 438 Python files are byte-identical to what you uploaded.**

---

## New files (8)

| File | Lines | What it is |
|---|---|---|
| `suspension/express.py` | ~2,300 | The engine: grammar → sniffer → planner → runner → bundler |
| `suspension/express_jobs.py` | ~2,200 | 36 jobs wired to the existing solvers |
| `suspension/cooling.py` | 553 | Coolant loop sizing + rig instrumentation uncertainty |
| `suspension/printed_parts.py` | 435 | Printed-polymer derating and material substitution |
| `suspension/wiring.py` | 623 | Conductor ampacity, derived rather than looked up |
| `suspension/rules_fsae.py` | 553 | FSAE 2027 **draft** ruleset with provenance attached |
| `ui/express_lane.py` | 262 | The Streamlit panel (Streamlit imported inside `render()`) |
| `tests/test_express.py` | ~1,500 | 200 tests |

## Modified files (1)

`streamlit_app.py` — a single `try/except` block inside
`_render_brief_questionnaire()` that calls `ui.express_lane.render()`. Nothing
else in that file is touched. Revert by deleting those 12 lines.

---

## Verify before trusting any of it

```bash
python3 -m suspension.express --selftest      # ALL PASS
python3 -m suspension.printed_parts           # ALL PASS
python3 -m suspension.cooling                 # ALL PASS
python3 -m suspension.wiring                  # ALL PASS
python3 -m suspension.rules_fsae              # ALL PASS
python3 -m pytest tests/test_express.py -q    # 200 passed
```

The existing suites still pass unchanged:

```bash
python3 -m pytest tests/test_mission_briefing.py tests/test_omnicore.py -q
```

---

## Run it

**In the app** — the ⚡ Express Lane expander on the landing screen.

**From the command line:**

```bash
python3 -m suspension.express "your sentence" [log.csv ...] -o bundle.zip
python3 -m suspension.express "your sentence" --dry-run    # plan only, ~0.1 s
```

---

## Design decisions worth knowing about

**Determinism is load-bearing.** The same sentence and the same bytes produce
a byte-identical ZIP. No wall clock enters the bundle, and job admission is
computed from *declared* costs rather than measured elapsed time — otherwise
the same request on a loaded laptop would produce a different artifact than on
an idle one, and the bundle would stop being citable in a design review.
`test_manifest_carries_no_wall_clock` exists to stop the next person adding a
timestamp "for debugging".

**Unknown is never a pass.** An unchecked rule is not a passed rule; a
conductor with no ampacity data is not cleared; a comparison that could not be
made says so rather than reporting agreement. Three separate bugs in this
family were found and fixed during development, each now pinned by a test — a
brake-bias check that printed "agree" on dead channels, a wiring check that
cleared 16 AWG for 132 A, and a thermal report that lectured about gradients
under a gradient of zero.

**The parameter table has one ordering contract:** specific phrases before
generic ones, because a generic word will otherwise claim the number a
specific phrase was reaching for. It cost four bugs before it was enforced.
`validate_param_table()` now checks it, and `test_the_validator_actually_
detects_a_violation` proves the guard works.

**Rules are a DRAFT.** `rules_fsae.RULESET.binding` is `False` and the draft
banner is emitted by a function every renderer calls — not a config flag,
because a rules verdict whose draft status can be switched off will eventually
be quoted without it.

**Two engines are deliberately absent.** Inverse Genesis needs a legal volume;
MorphMesh needs a component. Both are named in the README with the reason when
asked for, because a missing tool with no explanation reads as an oversight.

---

## Extending it

A new job is a function and a `register_job` call:

```python
def _job_mything(ctx: Ctx) -> List[Artifact]:
    ...
    ctx.flag(subsystem="...", item="...", mode="...", effect="...",
             severity=7, status="watch", evidence="modelled",
             action="...")            # feeds the generated DFMEA
    return [Artifact("mytool/report.md", _md("Title", lines), "md")]

register_job(Job("mything", "My thing", "mytool", _job_mything, cost_s=0.3))
```

Declare `needs_channels` / `needs_any` / `needs_extra` and the planner handles
activation, skip reasons and the README entry for free. Add the tool id to
`_TAB_NAMES` — there is a test that fails if you forget.
