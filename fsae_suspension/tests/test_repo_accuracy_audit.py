# ============================================================================
#  KinematiK — repo-wide accuracy audit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
# ============================================================================

"""
Mechanical checks for the defect CLASSES found in the 2026-08 hand audit.

WHY THIS EXISTS
---------------
The hand audit covered ~8k of ~250k lines and found seven substantive physics
defects. Hand review does not scale to the rest, and the defects were not
exotic — they clustered into a handful of shapes that a machine can look for:

  1. SHARED CONSTANTS that drift between modules (air density was 1.2 in one
     place and 1.225 in fourteen others).
  2. abs() APPLIED TO A SIGNED GEOMETRIC QUANTITY, which is how the anti-dive
     sign error hid: it made pro-dive and anti-dive indistinguishable.
  3. UNCALIBRATED OUTPUTS THAT DO NOT CARRY A PROVENANCE FLAG. This is the one
     that actually decides whether the tool can be trusted as a pre-validation
     screen: a synthesized number presented as a measured one is worse than no
     number at all.
  4. DUPLICATE MODULES, the mechanism by which a fix lands in one copy only.
  5. FLOAT EQUALITY AGAINST A FALLBACK SENTINEL, which silently mislabels a
     legitimate result that happens to equal the sentinel.

These are heuristics, not proofs. Each finding is a PROMPT TO LOOK, and every
known-good case is listed in an explicit allowlist with a reason — so the
allowlist itself becomes the record of what a human has actually reviewed.

Run:  python -m pytest tests/test_repo_accuracy_audit.py -v
"""
import ast
import os
import re
from collections import defaultdict

import pytest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PKG = os.path.join(_ROOT, "suspension")

#: The app file is 1.8 MB of UI glue; it is audited by its own tests, and
#: scanning it here would bury every real finding in presentation code.
_SKIP_FILES = {"streamlit_app.py"}
_SKIP_DIRS = {"__pycache__", "office", "schemas", ".git", "node_modules"}


def _sources():
    for base, dirs, files in os.walk(_PKG):
        dirs[:] = [d for d in dirs if d not in _SKIP_DIRS]
        for fn in files:
            if fn.endswith(".py") and fn not in _SKIP_FILES:
                path = os.path.join(base, fn)
                rel = os.path.relpath(path, _ROOT)
                try:
                    with open(path, encoding="utf-8") as fh:
                        yield rel, fh.read()
                except (OSError, UnicodeDecodeError):
                    continue


def _ui_sources():
    """The app files, read only to answer "is this attribute consumed anywhere?".
    They are excluded from every other check because 1.8 MB of UI glue would bury
    real findings, but they are where display-only fields get read."""
    for name in ("streamlit_app.py",):
        path = os.path.join(_ROOT, name)
        if os.path.exists(path):
            try:
                with open(path, encoding="utf-8", errors="replace") as fh:
                    yield fh.read()
            except OSError:
                continue
    ui_dir = os.path.join(_ROOT, "ui")
    if os.path.isdir(ui_dir):
        for fn in os.listdir(ui_dir):
            if fn.endswith(".py"):
                try:
                    with open(os.path.join(ui_dir, fn), encoding="utf-8",
                              errors="replace") as fh:
                        yield fh.read()
                except OSError:
                    continue


def _report(findings, what):
    lines = [f"  {loc}: {detail}" for loc, detail in sorted(findings)]
    return (f"{len(findings)} {what}:\n" + "\n".join(lines))


# --------------------------------------------------------------------------- #
#  1. Shared physical constants must not drift between modules
# --------------------------------------------------------------------------- #
#  Each entry: the canonical value, and the names that should carry it. A module
#  disagreeing by more than tolerance is how an unexplained delta appears
#  between the aero, powertrain and cooling numbers — with nothing in the tree
#  saying which figure is right.
_CANON = {
    "air_density": (1.225, 0.001,
                    re.compile(r"\b(?:rho_air|air_density|rho)\s*[:=]\s*"
                               r"(?:float\s*=\s*)?(1\.\d+)")),
    "gravity":     (9.81, 0.02,
                    re.compile(r"\bg\s*[:=]\s*(?:float\s*=\s*)?(9\.\d+)")),
}

#: (constant, module) pairs where a different value is CORRECT, with the reason.
_CONST_ALLOW = {
    # A unit test of q = 1/2 rho v^2. rho=1.2 with v=30 is chosen to make the
    # expected value exact and hand-checkable; it is arithmetic, not a physical
    # assumption about the air. Reviewed 2026-08.
    ("air_density", "suspension/test_pressure_tap.py"): "arithmetic fixture, not a physical constant",
}


@pytest.mark.parametrize("const", sorted(_CANON))
def test_shared_constants_do_not_drift(const):
    canon, tol, pattern = _CANON[const]
    bad = []
    for rel, src in _sources():
        for m in pattern.finditer(src):
            try:
                val = float(m.group(1))
            except ValueError:
                continue
            if abs(val - canon) > tol and (const, rel) not in _CONST_ALLOW:
                line = src[:m.start()].count("\n") + 1
                bad.append((f"{rel}:{line}", f"{const} = {val}, canonical {canon}"))
    assert not bad, (
        _report(bad, f"module(s) disagree on {const}") +
        f"\n\nUse {canon}. If a different value is physically right here "
        f"(different temperature, altitude, fluid), add it to _CONST_ALLOW with "
        f"the reason — the allowlist is the audit trail.")


# --------------------------------------------------------------------------- #
#  2. abs() on a signed geometric quantity
# --------------------------------------------------------------------------- #
#  anti_dive_pct computed tan(phi) = hsva / abs(Lsva). The abs() destroyed the
#  one piece of information that distinguishes anti-dive from pro-dive, so a car
#  built with the pickup stagger backwards was told it had 26% anti-dive when it
#  had -26%. Any abs() wrapping a lever arm, offset, height or distance deserves
#  the same question: is the sign genuinely irrelevant here?
_SIGNED_HINT = re.compile(
    r"abs\(\s*(?P<v>[A-Za-z_]\w*)\s*\)|np\.abs\(\s*(?P<v2>[A-Za-z_]\w*)\s*\)")
_SIGNED_NAMES = re.compile(
    r"^(?:L|l)?(?:sva|arm|lever|offset|moment_arm|dx|dy|dz|delta_[xyz]"
    r"|.*_(?:arm|offset|lever|height|dist|distance))$", re.I)

#: variable -> reason the magnitude is genuinely what is wanted.
_ABS_ALLOW = {
    "suspension/kinematics.py": {
        # Magnitudes here are correct: lengths and rates, not lever arms.
        "wheel_disp", "spring_disp", "wd", "mr", "d",
        # Lsva: only inside the `< 1e-9` degeneracy guard. The physics divisions
        # use the SIGNED value — reviewed 2026-08.
        "Lsva",
        # dz in _path_slope_xz: `abs(dz) < 1e-12` guards a divide-by-zero at a
        # travel where the point does not move vertically. The slope returned
        # immediately below is (dx / dz) with BOTH signed, which is the whole
        # point of that helper. Reviewed 2026-08.
        "dz",
    },
    "suspension/adapter.py": {
        # Same: guard only; the divisions use the signed offset. Reviewed
        # alongside the kinematics fix so the two solvers stay in step.
        "Lsva",
    },
    "suspension/dynamics.py": {
        # roll_center_height: `abs(dy) < 1e-9` guards a divide-by-zero when the
        # IC sits directly above the contact patch. `slope = dz / dy` below uses
        # the signed value. Reviewed 2026-08.
        "dy",
    },
    "suspension/ghost_topology.py": {"dy"},     # same guard, same reason
    "suspension/hardpoint_import.py": {
        # `abs(dx) > 1e-9 or abs(dz) > 1e-9` is a "did anything move?" test; the
        # signed dx/dz are then applied to the coordinates. Reviewed 2026-08.
        "dx", "dz",
    },
}


def test_abs_is_not_hiding_a_sign_convention():
    suspects = []
    for rel, src in _sources():
        allow = _ABS_ALLOW.get(rel, set())
        for m in _SIGNED_HINT.finditer(src):
            var = m.group("v") or m.group("v2")
            if not var or var in allow:
                continue
            if _SIGNED_NAMES.match(var):
                line = src[:m.start()].count("\n") + 1
                suspects.append((f"{rel}:{line}", f"abs({var})"))
    assert not suspects, (
        _report(suspects, "abs() call(s) on a signed geometric quantity") +
        "\n\nEach one destroys a direction. Confirm the sign is genuinely "
        "irrelevant, then add the variable to _ABS_ALLOW for that file. This is "
        "exactly how the anti-dive sign error survived review.")


# --------------------------------------------------------------------------- #
#  3. Uncalibrated numbers must be flagged as such
# --------------------------------------------------------------------------- #
#  This is the check that decides whether the tool is honest enough to sit ahead
#  of Ansys. KinematiK's own design says so: BrakeThermalParams.calibrated
#  defaults False, results carry `synthesized`, tyre constants are labelled
#  PLACEHOLDERS. A module that declares placeholder/representative/estimated
#  constants but exposes no provenance flag is presenting a guess with the same
#  visual authority as a measurement.
#  Two senses of "placeholder" are NOT physics provenance and generate almost
#  all the noise: the Streamlit `placeholder=` kwarg (input hint text), and the
#  3D model's geometric placeholder box. Excluded so the check keeps signal.
_PLACEHOLDER_NOISE = re.compile(
    r"placeholder\s*=|placeholder(?:'s)?\s+(?:box|envelope|geometry|mesh|part)|"
    r"the\s+placeholder|a\s+placeholder\b(?!\s+value)")
_PLACEHOLDER = re.compile(
    r"\b(PLACEHOLDER|placeholder value|representative,? not measured|"
    r"representative value|not measured|synthesi[sz]ed|"
    r"first-order estimate|rule of thumb)\b")
_PROVENANCE = re.compile(
    r"\b(calibrated|synthesized|synthesised|provenance|fitted_to|confidence|"
    r"is_estimate|uncalibrated|assumption|PROVENANCE|_warn)\b")

#: Modules that legitimately mention placeholders without needing a flag
#: (docs, tests, pure plumbing with no numeric output of their own).
_PROV_ALLOW = {
    "suspension/__init__.py",
    "suspension/interfaces.py",
    # Reviewed 2026-08 — the caveat travels WITH the number rather than in a
    # module-level flag, which is equally honest at the point of use:
    #   fit_forecast: the mass-sensitivity figure ships inside `lap_note`,
    #     reading "FSAE autocross rule of thumb, not a sim".
    #   aero/coupling: names itself a bridge until the suspension model feeds
    #     real roll/pitch/heave; emits no standalone physics number.
    #   rationale / sim_handoff / fullcar3d / auth_ui: "placeholder" here is
    #     prose about empty state, geometry stand-ins or UI hint text.
    "suspension/fit_forecast.py",
    "suspension/aero/coupling.py",
    "suspension/rationale.py",
    "suspension/sim_handoff.py",
    "suspension/fullcar3d.py",
    "suspension/auth_ui.py",
}


def test_placeholder_constants_carry_a_provenance_flag():
    bare = []
    for rel, src in _sources():
        if rel in _PROV_ALLOW or "/test" in rel or rel.endswith("_test.py"):
            continue
        scan = _PLACEHOLDER_NOISE.sub("", src)
        hits = _PLACEHOLDER.findall(scan)
        if hits and not _PROVENANCE.search(src):
            bare.append((rel, f"declares {len(hits)} placeholder value(s), "
                              f"exposes no provenance/calibration flag"))
    assert not bare, (
        _report(bare, "module(s) with unflagged placeholder physics") +
        "\n\nA synthesized number that looks measured is the single failure "
        "mode that makes a pre-validation tool untrustworthy. Give the module a "
        "`calibrated` / `synthesized` / `provenance` field and surface it, or "
        "add it to _PROV_ALLOW if it emits no numbers of its own.")


# --------------------------------------------------------------------------- #
#  4. No duplicate modules
# --------------------------------------------------------------------------- #
#  The tree had accumulated three copies of the app, five stale ui/ twins, three
#  backends_PATCHED files and a 96-file clone of the test suite. Two copies WILL
#  drift, and when they do nothing says which is authoritative — a fix to the
#  physics lands in one and not the other.
def test_no_duplicate_or_patch_artefact_modules():
    stray = []
    #  `v\d+` only counts as a copy marker when it TRAILS the name (foo_v2.py).
    #  test_capabilities_v09.py is a capability-version suite, not a duplicate.
    pat = re.compile(r"(\(\d+\)|[ _-](copy|kopie|old|backup|bak|PATCHED))"
                     r"(?=\.py$|\.md$)", re.I)
    for base, dirs, files in os.walk(_ROOT):
        dirs[:] = [d for d in dirs if d not in _SKIP_DIRS]
        for fn in files:
            if fn.endswith((".py", ".md")) and pat.search(fn):
                stray.append((os.path.relpath(os.path.join(base, fn), _ROOT),
                              "name marks it a copy or patch artefact"))
    assert not stray, (
        _report(stray, "duplicate / patch-artefact file(s)") +
        "\n\nDelete them or fold them into the authoritative module.")


def test_no_ui_twin_modules():
    """`ui/foo.py` and `ui/ui_foo.py` are the same panel twice."""
    ui = os.path.join(_ROOT, "ui")
    if not os.path.isdir(ui):
        pytest.skip("no ui/ package")
    names = {f[:-3] for f in os.listdir(ui) if f.endswith(".py")}
    twins = [(f"ui/ui_{n}.py", f"twin of ui/{n}.py")
             for n in sorted(names) if f"ui_{n}" in names]
    twins += [(f"ui/{n}_ui.py", f"twin of ui/{n}.py")
              for n in sorted(names) if f"{n}_ui" in names]
    assert not twins, _report(twins, "duplicated UI panel(s)")


# --------------------------------------------------------------------------- #
#  5. Fallback sentinels must not be detected by float equality
# --------------------------------------------------------------------------- #
#  laptime warned "grip fell back to default 1.4 g" by testing `max_lat_g == 1.4`.
#  A geometry that legitimately solves to exactly 1.4 g was mislabelled, and
#  changing the constant would have silently broken every call site.
_FLOAT_EQ = re.compile(r"[A-Za-z_]\w*\s*[!=]=\s*\d+\.\d+")

_EQ_ALLOW = {
    # Comparisons against exact, definitionally-representable values.
    "0.0", "1.0", "0.5", "2.0", "100.0",
}

#: file:line -> reason the equality is safe. Unlike a value allowlist this
#: pins the exact site, so a NEW equality in the same file still trips.
_EQ_ALLOW_SITES = {
    # Cache-key guard: `d == 5.0` asks "were we called with the default step?",
    # comparing against the literal default of the same parameter. Exact by
    # construction, not a measured value. Reviewed 2026-08.
    "suspension/kinematics.py": {"d == 5.0"},
    # Table lookups and fixtures: comparing a tabulated constant to the constant
    # it was tabulated from. No computation intervenes.
    "suspension/wiring.py": {"a_75c == 115.0"},
    "suspension/rules_fsae.py": {"== 60.0 * r.n_violations"},
    "suspension/omnicore.py": {"budget_usd == 15000.0"},
    "suspension/test_phantom_car.py": {"NAKED_SIGMA == 0.25"},
    "suspension/test_piv.py": {"dt_us == 120.0", "freestream_ms == 27.0"},
}


def test_no_float_equality_against_a_magic_number():
    bad = []
    for rel, src in _sources():
        for i, line in enumerate(src.splitlines(), 1):
            stripped = line.strip()
            if stripped.startswith("#"):
                continue
            for m in _FLOAT_EQ.finditer(line):
                lit = m.group(0).split("=")[-1].strip()
                if lit in _EQ_ALLOW:
                    continue
                if any(frag in line for frag in _EQ_ALLOW_SITES.get(rel, ())):
                    continue
                bad.append((f"{rel}:{i}", stripped[:90]))
    assert not bad, (
        _report(bad, "float equality comparison(s) against a magic number") +
        "\n\nReturn the fact alongside the number (see "
        "laptime._max_lat_g_flagged) instead of inferring it from the value.")


# --------------------------------------------------------------------------- #
#  6. Every public physics function documents its units
# --------------------------------------------------------------------------- #
#  Mixed units (mm vs m, deg vs rad, N/mm vs N/m) are the most common silent
#  error in vehicle-dynamics code, and this codebase carries all three pairs.
#  A docstring naming the units is the cheapest possible defence.
#: A unit baked into the identifier itself, e.g. foo_mm, q_kPa, t_deg_c.
_UNIT_IN_NAME = re.compile(
    r"_(mm\d?|mm[234]|cm|m[234]?|km|kg|g|n|nm|nmm\d?|kpa|pa|mpa|bar|psi|"
    r"deg|deg_c|degc|rad|rad_s|c|k|s|ms|us|hz|khz|w|kw|j|kj|wh|kwh|v|a|ah|"
    r"ohm|usd|pct|percent|frac|ratio|m_s|ms2|m_s2|n_mm|n_m|nm_deg|j_k|w_m2k)$",
    re.I)

_UNIT_WORDS = re.compile(
    r"\b(mm|cm|\bm\b|km|N/mm|N/m|N·m|Nm|deg|degree|rad|radian|kg|N\b|Pa|bar|"
    r"°C|degC|Kelvin|W/|J/|m/s|g\b|%|percent|unitless|dimensionless)",
    re.I)

#: Modules that are pure plumbing: IO, registries, UI adapters, persistence.
_UNITS_ALLOW_PREFIX = (
    "suspension/auth", "suspension/project", "suspension/workspace",
    "suspension/history", "suspension/drive_", "suspension/cad_",
    "suspension/analytics", "suspension/report", "suspension/myth",
    "suspension/express", "suspension/adapter", "suspension/interfaces",
    "suspension/visitor_id",
)


def test_public_physics_functions_state_their_units():
    missing = defaultdict(list)
    for rel, src in _sources():
        if rel.startswith(_UNITS_ALLOW_PREFIX) or "/test" in rel:
            continue
        try:
            tree = ast.parse(src)
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            if node.name.startswith("_"):
                continue
            # Only functions that actually take or return numbers.
            if not node.args.args and not node.args.kwonlyargs:
                continue
            #  Skip functions that cannot carry units by their return type.
            #  Validators (-> None), formatters (-> str), predicates (-> bool)
            #  and constructors are not physics, and demanding a unit from them
            #  inflates the count with noise — which then hides a real physics
            #  function slipping in undocumented.
            #
            #  Added after the ratchet fired on my own additions
            #  (require_finite_export, graded, worst_grade, the __post_init__
            #  validators). The right response to a ratchet catching you is to
            #  make the check sharper or do the work, never to raise the bound.
            _ret = getattr(node, "returns", None)
            if isinstance(_ret, ast.Constant) and _ret.value is None:
                continue
            if isinstance(_ret, ast.Name) and _ret.id in ("None", "str", "bool"):
                continue
            if node.name == "__post_init__":
                continue
            # Units encoded in the NAME count, and count for more than a
            # docstring: `area_mm2`, `EI_Nmm2`, `mass_per_m_kg`, `wheel_rad_s`
            # carry their units at every call site, not just at the definition.
            # Not crediting them was the check's own false positive.
            if _UNIT_IN_NAME.search(node.name):
                continue
            doc = ast.get_docstring(node) or ""
            if not doc:
                missing[rel].append(f"{node.name} (line {node.lineno}): no docstring")
            elif not _UNIT_WORDS.search(doc):
                missing[rel].append(f"{node.name} (line {node.lineno}): docstring names no units")
    total = sum(len(v) for v in missing.values())
    # Ratchet, not a gate: this is a large legacy surface. The number must go
    # DOWN, never up. Lower the bound whenever you improve a batch.
    #  RATCHET: lower it, never raise it.
    #    1432 -> credited units encoded in identifiers (area_mm2, EI_Nmm2)
    #    1273 -> excluded functions that cannot carry units by return type
    #            (validators, formatters, predicates)
    #  Both reductions came from making the CHECK sharper, not from documenting
    #  anything — that work is still outstanding. A check with 250 false
    #  positives in it hides the real ones.
    LIMIT = 1023    # measured 2026-08
    detail = "\n".join(
        f"  {rel}: {len(v)} function(s)" for rel, v in sorted(
            missing.items(), key=lambda kv: -len(kv[1]))[:15])
    assert total <= LIMIT, (
        f"{total} public functions do not state their units (budget {LIMIT}).\n"
        f"Worst modules:\n{detail}\n\n"
        "Mixed mm/m, deg/rad and N/mm vs N/m are the commonest silent errors in "
        "this domain. Document the units, then lower LIMIT.")


# --------------------------------------------------------------------------- #
#  7. Diagnostics that are computed and then never read
# --------------------------------------------------------------------------- #
#  run_log computed `implied_area_drag_m2` for every row, stored it on the
#  derived record, and read it nowhere. It was the only signal that could catch
#  a row whose lift and drag were normalised by DIFFERENT reference areas — a
#  failure invisible to every other gate, because the row is internally
#  consistent in lift and internally consistent in drag, just not with itself.
#
#  A field that is only ever written is either dead weight or, as there, a check
#  someone built and forgot to wire up. Both are worth surfacing; the second is
#  worth surfacing loudly.
_ASSIGN = re.compile(r"^\s*(?:self\.|[a-z_]+\.)?(\w+)\s*=\s*[^=]", re.M)

#: field -> reason it is legitimately write-only (serialised out, part of a
#: public dataclass consumed elsewhere, set for a caller rather than for us).
_WRITE_ONLY_ALLOW = {
    # Populated for callers/serialisation, not for this module's own logic.
    "notes", "warnings", "findings", "provenance", "flags", "detail",
    "source", "label", "name", "note", "reason", "summary", "status",
}


def test_no_diagnostic_is_computed_and_never_read():
    suspects = []
    for rel, src in _sources():
        if "/test" in rel:
            continue
        try:
            tree = ast.parse(src)
        except SyntaxError:
            continue
        # Attribute names WRITTEN on a dataclass-like target.
        written = {}
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign):
                for tgt in node.targets:
                    if isinstance(tgt, ast.Attribute):
                        written.setdefault(tgt.attr, node.lineno)
        if not written:
            continue
        # Attribute names READ anywhere in the file.
        read = {n.attr for n in ast.walk(tree)
                if isinstance(n, ast.Attribute) and isinstance(n.ctx, ast.Load)}
        for attr, line in written.items():
            if attr in _WRITE_ONLY_ALLOW or attr.startswith("_"):
                continue
            if attr in read:
                continue
            # Only flag names that look like a measurement or a check result —
            # plain data carriers are expected to be write-only here.
            if not re.search(r"(implied|ratio|margin|residual|deviation|error|"
                             r"check|coverage|_pct|_frac|_m2|_mm|_hz|_c$)", attr):
                continue
            # It may be read in another module; only flag if nothing else
            # reads it either. This MUST include the UI, which is skipped by
            # every other check here: a field parsed for display is consumed
            # only in streamlit_app.py, and not looking there reported three
            # perfectly live fields as dead. A checker that cries wolf gets
            # switched off, which costs more than the check is worth.
            if any(f".{attr}" in other for orel, other in _sources()
                   if orel != rel):
                continue
            if any(f".{attr}" in ui for ui in _ui_sources()):
                continue
            suspects.append((f"{rel}:{line}", f"`{attr}` is written and never read"))
    assert not suspects, (
        _report(suspects, "diagnostic(s) computed but never consumed") +
        "\n\nEither wire it into a check or delete it. A half-built gate is worse "
        "than no gate: it looks like coverage that does not exist.")


# --------------------------------------------------------------------------- #
#  8. Branches of one calculation that disagree with each other
# --------------------------------------------------------------------------- #
#  Two of the powertrain defects had the same shape: one branch of a calculation
#  accounted for something and the sibling branch did not. The accel branch used
#  `m*a + F_drag + F_roll`; the regen branch used a bare `m*a`. Neither is a
#  wrong formula — the second is a correct formula that stops halfway.
#
#  This cannot be fully checked mechanically, so it is a curated regression list:
#  each entry is a pair of expressions that MUST both appear, in the same file.
#  Add to it whenever an asymmetry is found and fixed.
_PAIRED_TERMS = [
    ("suspension/ev_powertrain.py", "F_drag - F_roll", "F_drag + F_roll",
     "regen must subtract the resistance the accel branch adds"),
    ("suspension/pack_thermal.py", "f_drag - f_roll", "f_drag + f_roll",
     "the current trace must match ev_powertrain's energy accounting"),
]


@pytest.mark.parametrize("rel,a,b,why", _PAIRED_TERMS)
def test_paired_branches_stay_symmetric(rel, a, b, why):
    path = os.path.join(_ROOT, rel)
    if not os.path.exists(path):
        pytest.skip(f"{rel} not present")
    with open(path, encoding="utf-8") as fh:
        src = fh.read()
    assert a in src and b in src, (
        f"{rel}: expected both `{a}` and `{b}` — {why}.\n"
        f"One branch of this calculation has stopped accounting for something "
        f"its sibling accounts for.")


# --------------------------------------------------------------------------- #
#  9. The provenance vocabulary must stay singular
# --------------------------------------------------------------------------- #
#  While building the grade-propagation work I started writing a SECOND grade
#  enum before noticing proof_engine.EvidenceGrade already existed. That is the
#  same duplicate-implementation defect this audit found four times in the
#  physics (anti-dive in two solvers, roll centre in two, regen in two, the
#  reference geometry in two) — and a second grade vocabulary would be worse
#  than a second formula, because the whole value of a badge is that it means
#  one thing everywhere.
_GRADE_ENUM_HINT = re.compile(
    r"class\s+\w*(Grade|Confidence|Provenance|Evidence)\w*\s*\(", re.I)

#: The canonical definitions. Anything else defining a grade vocabulary is a fork.
_GRADE_OWNERS = {
    "suspension/proof_engine.py",       # EvidenceGrade — the single source
    #  risk_propagation.Confidence classifies how an EDGE was derived, which is
    #  a different axis from evidence level. It is allowed to exist ONLY because
    #  it now exposes `.evidence_grade`, mapping every tier onto EvidenceGrade —
    #  so the two can never drift into meaning different things again. Its
    #  ceiling is MODELLED, because no propagated edge has touched hardware.
    #  Enforced by test_edge_confidence_maps_onto_the_one_vocabulary below.
    "suspension/risk_propagation.py",
}


def test_only_one_evidence_grade_vocabulary():
    forks = []
    for rel, src in _sources():
        if rel in _GRADE_OWNERS or "/test" in rel:
            continue
        for m in _GRADE_ENUM_HINT.finditer(src):
            line = src[:m.start()].count("\n") + 1
            forks.append((f"{rel}:{line}", m.group(0).strip()))
    assert not forks, (
        _report(forks, "competing evidence-grade vocabulary/vocabularies") +
        "\n\nUse proof_engine.EvidenceGrade. A badge is only worth rendering if "
        "it means the same thing in every tool that renders it.")


# --------------------------------------------------------------------------- #
#  10. Weakest link, never an average
# --------------------------------------------------------------------------- #
def test_grade_combination_is_never_averaged():
    """Averaging grades lets good evidence launder bad: three measured
    subsystems carrying a guessed fourth. Every combination site must take the
    minimum. Checked by behaviour rather than by reading the code, so a
    reimplementation elsewhere still has to obey it."""
    from suspension.proof_engine import aggregate_grades, Quantity, EvidenceGrade as G
    from suspension.provenance import worst_grade

    qs = [Quantity(f"m{i}", f"s{i}", "mass_kg", f"S{i}", 20.0, "kg", g)
          for i, g in enumerate((G.VERIFIED, G.MEASURED, G.MEASURED, G.GUESS))]
    assert aggregate_grades(qs)["mass_kg"] == G.GUESS
    assert worst_grade("verified", "measured", "measured", "guess") == "guess"
    # order must not matter
    assert worst_grade("guess", "verified") == worst_grade("verified", "guess")


# --------------------------------------------------------------------------- #
#  11. Graded numbers cannot be rendered bare
# --------------------------------------------------------------------------- #
def test_graded_formatter_requires_a_grade():
    """`graded()` takes the grade positionally and required, so forgetting it is
    a TypeError at the call site rather than a silently unbadged number. This is
    the whole mechanism: the failure being prevented is a placeholder tyre
    coefficient and a hand-verified bolt stress area formatting identically."""
    import inspect
    from suspension.provenance import graded
    sig = inspect.signature(graded)
    assert sig.parameters["grade"].default is inspect.Parameter.empty, \
        "grade must stay REQUIRED; a default makes the badge optional"
    with pytest.raises(TypeError):
        graded(1.23)                       # type: ignore[call-arg]
    out = graded(1.4666, "guess", "g", limited_by="tyre mu_peak")
    assert "guess" in out and "tyre mu_peak" in out and "1.467" in out


def test_edge_confidence_maps_onto_the_one_vocabulary():
    """risk_propagation.Confidence is allowed to be a separate axis (how an edge
    was derived) only while every tier maps onto EvidenceGrade. It used the word
    MEASURED for "a solver produced this", which in EvidenceGrade terms is
    MODELLED — the same badge claiming hardware where there was none, and the
    more flattering of the two readings. A propagated edge can never exceed
    MODELLED, and the mapping has to keep saying so."""
    from suspension.risk_propagation import Confidence
    from suspension.proof_engine import EvidenceGrade
    for c in Confidence:
        g = c.evidence_grade
        assert isinstance(g, EvidenceGrade)
        assert g.rank <= EvidenceGrade.MODELLED.rank, (
            f"edge confidence {c.value!r} claims {g.value}; no propagated edge "
            f"has touched hardware, so MODELLED is the ceiling")
        assert "measurement" not in c.label or "not a measurement" in c.label


# --------------------------------------------------------------------------- #
#  12. Provenance adoption ratchet
# --------------------------------------------------------------------------- #
#  suspension/provenance.py is well built and well tested, and at the time this
#  check was written NOTHING imported it outside its own test file. The render
#  layer existed and was never wired in — which is the most expensive kind of
#  unfinished work, because it looks finished from the inside.
#
#  685 numeric renders across the app and ui/ formatted a physics number with a
#  bare f-string, so a PLACEHOLDER tyre coefficient and a hand-verified bolt
#  stress area printed identically. That is the exact failure the badge system
#  was built to prevent, and it was live everywhere.
#
#  Retrofitting all of them by hand is neither realistic nor wise — most are
#  labels, counts and IDs that need no grade. So this is a RATCHET, like the
#  units check: the count of ungated numeric renders may only go DOWN. Lower the
#  bound whenever a batch is converted, and it can never quietly climb.
_NUMFMT = re.compile(r'f"[^"]*\{[^}]*:[.,]\d*[fge][^"]*"')


def _render_surfaces():
    yield "streamlit_app.py", os.path.join(_ROOT, "streamlit_app.py")
    ui = os.path.join(_ROOT, "ui")
    if os.path.isdir(ui):
        for fn in sorted(os.listdir(ui)):
            if fn.endswith(".py"):
                yield f"ui/{fn}", os.path.join(ui, fn)


def test_ungated_numeric_renders_only_decrease():
    total, per_file = 0, {}
    for rel, path in _render_surfaces():
        if not os.path.exists(path):
            continue
        try:
            with open(path, encoding="utf-8", errors="replace") as fh:
                src = fh.read()
        except OSError:
            continue
        n = len(_NUMFMT.findall(src))
        if n:
            per_file[rel] = n
            total += n

    LIMIT = 685    # measured 2026-08. RATCHET: lower it, never raise it.
    worst = "\n".join(f"  {k}: {v}" for k, v in
                      sorted(per_file.items(), key=lambda kv: -kv[1])[:8])
    assert total <= LIMIT, (
        f"{total} numeric renders are not going through provenance.graded() "
        f"(budget {LIMIT}).\nWorst files:\n{worst}\n\n"
        "Convert a batch and lower LIMIT. Start with the numbers a team would "
        "put in a design report — those are the ones where a missing badge "
        "actually costs something.")


def test_provenance_helpers_are_actually_used_somewhere():
    """A ratchet on the ungated count is only half the signal: it would happily
    sit at its bound forever with adoption at zero. This asserts the other
    direction — the helpers must have real call sites outside their own tests,
    so the pattern is established rather than merely available."""
    users = set()
    for rel, src in _sources():
        if "provenance.py" in rel or "/test" in rel or rel.endswith("_test.py"):
            continue
        if re.search(r"\b(graded|provenance_tag|confidence_note|"
                     r"render_report_value|worst_grade)\s*\(", src):
            users.add(rel)
    for rel, path in _render_surfaces():
        if not os.path.exists(path):
            continue
        with open(path, encoding="utf-8", errors="replace") as fh:
            if re.search(r"\b(graded|provenance_tag|confidence_note|"
                         r"render_report_value)\s*\(", fh.read()):
                users.add(rel)
    assert users, (
        "provenance.py has no call sites outside its own tests. The render "
        "layer exists and is not wired in, which is the most expensive kind of "
        "unfinished work — it looks done from the inside.")
