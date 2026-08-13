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
        # on lines 880/931 use the SIGNED value — reviewed 2026-08.
        "Lsva",
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
    LIMIT = 1273    # measured 2026-08 (was 1432 before name-encoded units were
                    # credited). RATCHET: lower it, never raise it.
    detail = "\n".join(
        f"  {rel}: {len(v)} function(s)" for rel, v in sorted(
            missing.items(), key=lambda kv: -len(kv[1]))[:15])
    assert total <= LIMIT, (
        f"{total} public functions do not state their units (budget {LIMIT}).\n"
        f"Worst modules:\n{detail}\n\n"
        "Mixed mm/m, deg/rad and N/mm vs N/m are the commonest silent errors in "
        "this domain. Document the units, then lower LIMIT.")
