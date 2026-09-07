# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Enforcement for suspension/physics_owners.py.

A registry that merely lists who owns what is decoration. These tests RUN both
implementations of every registered quantity and compare the numbers, because
that is the only thing that would have caught any of the six duplication
defects the registry was written in response to.

Note what the divergences had in common: every one of them agreed on the path
that gets exercised and split on the path that does not. Main branch fine,
guard branch wrong. Axes fine, interior wrong. Balanced car fine, unbalanced car
wrong. So each comparison here sweeps a RANGE and deliberately includes the
degenerate and the asymmetric cases — comparing two implementations at their
default operating point proves almost nothing.

Run:  python -m pytest tests/test_physics_ownership.py -v
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from suspension.physics_owners import PHYSICS_OWNERS, owner_of, mirrors_of


# --------------------------------------------------------------------------- #
#  The registry must describe the repo that exists
# --------------------------------------------------------------------------- #
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_every_registered_path_exists():
    """A registry pointing at deleted or renamed modules is worse than none —
    it reads as coverage while checking nothing."""
    missing = []
    for q in PHYSICS_OWNERS:
        for entry in (q.owner,) + q.mirrors + q.delegates:
            # Entries may name a class within a module ("module.py::ClassName")
            # when one file holds two implementations of the same quantity —
            # brake_thermal ships a 2-node and a 1-D rotor model side by side.
            path = entry.split("::", 1)[0]
            if not os.path.exists(os.path.join(_ROOT, path)):
                missing.append(f"{q.key}: {path}")
    assert not missing, "registry references paths that do not exist:\n  " + \
        "\n  ".join(missing)


def test_every_mirror_states_how_agreement_is_proven():
    """A mirror without a stated basis for agreement is an unexamined duplicate
    wearing a label."""
    silent = [q.key for q in PHYSICS_OWNERS if q.mirrors and len(q.agreement) < 40]
    assert not silent, f"mirrors registered with no agreement rationale: {silent}"


def test_owner_is_not_also_a_mirror_of_itself():
    for q in PHYSICS_OWNERS:
        assert q.owner not in q.mirrors and q.owner not in q.delegates, \
            f"{q.key}: {q.owner} is listed as its own mirror/delegate"


# --------------------------------------------------------------------------- #
#  anti-dive / anti-squat: native vs generic solver
# --------------------------------------------------------------------------- #
def _kin_pair():
    from suspension.kinematics import Hardpoints, SuspensionKinematics
    from suspension.adapter import GenericKinematics
    from suspension.topologies import example
    return (SuspensionKinematics(Hardpoints.default()),
            GenericKinematics(example("double_wishbone")))


def test_anti_dive_owner_and_mirror_agree():
    """These disagreed by 26% on anti-squat while sharing a geometry, because
    each built a side-view instant centre from a different pair of carrier
    points. For a non-planar linkage that centre is not a shared object."""
    native, generic = _kin_pair()
    for cg, wb, bias in ((300.0, 1550.0, 0.65), (280.0, 1600.0, 0.55),
                         (340.0, 1500.0, 0.75)):
        assert abs(native.anti_dive_pct(cg, wb, bias)
                   - generic.anti_dive_pct(cg, wb, bias)) < 1e-3
        assert abs(native.anti_squat_pct(cg, wb, 1.0)
                   - generic.anti_squat_pct(cg, wb, 1.0)) < 1e-3


def test_delegates_add_no_physics_of_their_own():
    """dynamics.anti_dive_pct forwards to kinematics. Recorded as a DELEGATE so
    nobody 'fixes' the wrapper by giving it maths of its own — which is exactly
    how a delegate becomes a diverged mirror."""
    from suspension.dynamics import VehicleDynamics, VehicleParams
    from suspension.kinematics import Hardpoints, SuspensionKinematics
    kin = SuspensionKinematics(Hardpoints.default())
    veh = VehicleDynamics(VehicleParams(), front_kin=kin, rear_kin=kin)
    p = veh.p
    assert abs(veh.anti_dive_pct(0.65)
               - kin.anti_dive_pct(p.cg_height, p.wheelbase, 0.65)) < 1e-9
    assert abs(veh.anti_squat_pct()
               - kin.anti_squat_pct(p.cg_height, p.wheelbase, 1.0)) < 1e-9


# --------------------------------------------------------------------------- #
#  roll centre: dynamics vs ghost_topology
# --------------------------------------------------------------------------- #
def test_roll_centre_owner_and_mirror_agree_through_travel_and_at_degeneracy():
    """They matched everywhere except the branch nobody looks at. Sweeping
    travel is not enough on its own — the degenerate case has to be constructed
    deliberately, because a real geometry will not wander into it."""
    np = pytest.importorskip("numpy")
    from suspension.dynamics import VehicleDynamics, VehicleParams
    from suspension.ghost_topology import _rc_height_mm
    from suspension.kinematics import Hardpoints, SuspensionKinematics
    kin = SuspensionKinematics(Hardpoints.default())
    veh = VehicleDynamics(VehicleParams())
    for travel in (-30.0, -15.0, 0.0, 15.0, 30.0):
        st = kin.solve_at_travel(travel)
        assert abs(veh.roll_center_height(kin, 1200.0, state=st)
                   - _rc_height_mm(st, 1200.0)) < 1e-9

    class _Vertical:
        instant_center = np.array([600.0, 400.0])   # directly above the patch
        contact_patch = np.array([0.0, 600.0, 0.0])
        wheel_center = np.array([0.0, 600.0, 228.0])
        travel = 0.0

    st = _Vertical()
    a = veh.roll_center_height(kin, 1200.0, state=st)
    b = _rc_height_mm(st, 1200.0)
    assert not np.isfinite(a) and not np.isfinite(b), \
        "a vertical cp->IC line has no finite roll centre; both must say so"


# --------------------------------------------------------------------------- #
#  regen energy: ev_powertrain vs pack_thermal
# --------------------------------------------------------------------------- #
def test_regen_owner_and_mirror_integrate_to_the_same_energy():
    """pack_thermal.pack_current_trace promises in its own docstring that it
    integrates back to ev_powertrain's energy. That promise silently became
    false when the regen correction landed in one and not the other."""
    np = pytest.importorskip("numpy")
    from suspension.pack_thermal import pack_current_trace
    from suspension.ev_powertrain import EVLapSimulator, EVParams, LapSimParams

    class _Lap:
        pass

    lap = _Lap()
    lap.distance = np.linspace(0.0, 1000.0, 3000)
    lap.speed = 18.0 + 7.0 * np.sin(lap.distance / 40.0) + 3.0 * np.sin(lap.distance / 13.0)
    lap.long_g = np.gradient(lap.speed) * lap.speed / np.gradient(lap.distance) / 9.81

    p, ev = LapSimParams(), EVParams()
    net_kwh, _ = EVLapSimulator(ev)._energy_from_trace(lap, p)
    t, cur = pack_current_trace(lap, p, pack_nominal_v=400.0,
                                inverter_motor_eff=ev.inverter_motor_eff,
                                regen_eff=ev.regen_eff, regen_max_g=ev.regen_max_g)
    from_current = float(np.trapezoid(cur * 400.0, t)) / 3.6e6
    assert abs(from_current / net_kwh - 1.0) < 0.02


# --------------------------------------------------------------------------- #
#  shared tables and constants
# --------------------------------------------------------------------------- #
def test_awg_table_owner_and_mirror_agree():
    from suspension.fuse_test import AWG_AREA_MM2 as mirror
    import suspension.harness as harness
    owner = next(v for v in vars(harness).values()
                 if isinstance(v, dict) and 16 in v and isinstance(v.get(16), float))
    shared = set(owner) & set(mirror)
    assert len(shared) >= 8, "the two AWG tables barely overlap; check the owner lookup"
    for gauge in sorted(shared):
        assert abs(owner[gauge] - mirror[gauge]) / owner[gauge] < 2e-3, \
            f"AWG {gauge}: harness {owner[gauge]} vs fuse_test {mirror[gauge]}"


def test_reference_geometry_is_read_not_copied():
    """topologies.example() was a copy-pasted literal of Hardpoints.default(),
    so the cross-solver agreement test compared two different cars."""
    np = pytest.importorskip("numpy")
    from suspension.kinematics import Hardpoints
    from suspension.topologies import example
    hp = Hardpoints.default()
    mech = example("double_wishbone")
    names = {"ufi": "upper_front_inner", "uri": "upper_rear_inner",
             "lfi": "lower_front_inner", "lri": "lower_rear_inner",
             "uo": "upper_outer", "lo": "lower_outer",
             "wc": "wheel_center", "cp": "contact_patch"}
    checked = 0
    for short, attr in names.items():
        pt = mech.points.get(short)
        if pt is None:
            continue
        assert np.allclose(np.asarray(pt.pos, float), getattr(hp, attr), atol=1e-9)
        checked += 1
    assert checked >= 8


# --------------------------------------------------------------------------- #
#  No unregistered second implementation
# --------------------------------------------------------------------------- #
def test_registered_quantities_have_no_unregistered_implementations():
    """The registry only helps if it stays complete. Any module defining a
    function named after a registered quantity, other than its owner, mirrors or
    delegates, is an undeclared third copy — which is how three lap solvers and
    two grade vocabularies happened."""
    import ast
    known = {}
    for q in PHYSICS_OWNERS:
        known[q.key] = {e.split("::", 1)[0]
                        for e in (q.owner,) + q.mirrors + q.delegates}

    strays = []
    for base, dirs, files in os.walk(os.path.join(_ROOT, "suspension")):
        dirs[:] = [d for d in dirs if d not in ("__pycache__", "office")]
        for fn in files:
            if not fn.endswith(".py") or fn.startswith("test_"):
                continue
            path = os.path.join(base, fn)
            rel = os.path.relpath(path, _ROOT)
            try:
                tree = ast.parse(open(path, encoding="utf-8").read())
            except (OSError, SyntaxError):
                continue
            for node in ast.walk(tree):
                if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    continue
                name = node.name.lstrip("_")
                if name in known and rel not in known[name]:
                    strays.append(f"{rel}:{node.lineno} defines {name}()")
    assert not strays, (
        "undeclared implementations of registered quantities:\n  "
        + "\n  ".join(strays)
        + "\n\nEither register it in physics_owners.PHYSICS_OWNERS as a mirror "
          "(with a numeric agreement test here) or a delegate, or call the "
          "owner instead of reimplementing it.")


def test_rotor_models_gap_does_not_widen():
    """brake_thermal ships two fidelity levels of the same physics. They once
    disagreed by 28.8%, which turned out to be TWO opposing effects that
    partially cancelled:

      * a real bug — the 1-D solved half the thickness and took half the heat,
        but applied the full area_factor to its one face and implied the other
        by symmetry, shedding twice. Fixed.
      * a scope difference — the 2-node has a pad node and the 1-D does not, so
        the 1-D legitimately runs hotter.

    Strip the pad from the 2-node and they agree to 4.0%. This pins BOTH bounds:
    the 1-D must stay hotter (or the over-shedding is back) and by no more than
    the pad path explains.
    """
    np = pytest.importorskip("numpy")
    from suspension.brake_thermal import TwoNodeRotorPad, OneDRotor, TwoNodeParams

    dt, n = 0.02, 3000
    power = np.zeros(n)
    for i in range(0, n, 250):
        power[i:i + 40] = 45000.0
    speed = np.full(n, 20.0)

    two = TwoNodeRotorPad(TwoNodeParams()).simulate(power, dt, speed)
    one = OneDRotor(TwoNodeParams(), n_nodes=12).simulate(power, dt, speed)

    assert abs(two.rotor_mass_kg - one.rotor_mass_kg) < 1e-9, \
        "the two models no longer even agree on the rotor mass"

    t_two = float(np.asarray(two.T_rotor_c, float)[-1])
    core = float(np.asarray(one.T_core_c, float)[-1])
    surf = float(np.asarray(one.T_surface_c, float)[-1])
    t_one = 0.5 * (core + surf)

    assert abs(core - surf) < 30.0, (
        "the 1-D rotor is no longer near-isothermal on this duty cycle, so this "
        "characterisation no longer isolates the modelling gap — revisit it")

    gap = (t_one - t_two) / t_two
    #  The 1-D must run HOTTER than the 2-node, by roughly the pad path it does
    #  not model. If it ever comes out COOLER again, the halved per-face
    #  area_factor has been reverted and the model is shedding twice.
    assert gap > 0.0, (
        f"1-D mean {t_one:.1f} C is below the 2-node {t_two:.1f} C — the 1-D is "
        f"over-shedding again; check the 0.5 * area_factor on its face.")
    assert gap < 0.20, (
        f"2-node {t_two:.1f} C vs 1-D mean {t_one:.1f} C — {gap * 100:.1f}% apart, "
        f"wider than the +14.8% attributable to the missing pad node. Something "
        f"beyond the known scope difference has moved.")


def test_rotor_over_temperature_flag_responds_to_the_corrected_shedding():
    """THE CONSEQUENCE OF THE 1-D CONVECTION FIX, pinned because it is the most
    safety-relevant single change in the 2026-08 audit.

    Duty cycle: 45 kW stops of 0.8 s, every 5 s, for 60 s — about 36 kJ per
    front rotor per stop, which is one 100 km/h stop on a 300 kg car at 65%
    front bias. Realistic FSAE endurance braking, not a contrived load.

    Shedding twice, the model peaked at 600 C and reported the rotor INSIDE the
    700 C grey-cast-iron limit. Corrected, it peaks at 870 C and reports it
    over. A tool whose job is to say what to go check was saying 'fine'.

    Note what did NOT move: the through-thickness gradient (42.1 -> 42.5 C) and
    the thermal stress (68.8 -> 69.5 MPa), which is what this class exists to
    compute. The gradient is driven by the transient flux into the surface
    during a stop, not by the steady convective balance, so the crack-risk
    screen was right all along and only the absolute temperature was wrong.
    Worth knowing before anyone assumes the whole model was suspect.
    """
    np = pytest.importorskip("numpy")
    from suspension.brake_thermal import OneDRotor, TwoNodeParams

    dt, n = 0.02, 3000
    power = np.zeros(n)
    for i in range(0, n, 250):
        power[i:i + 40] = 45000.0
    speed = np.full(n, 20.0)

    r = OneDRotor(TwoNodeParams(), n_nodes=12).simulate(power, dt, speed)

    assert r.T_surface_peak_c > 800.0, (
        f"peak surface {r.T_surface_peak_c:.0f} C — far below the ~870 C the "
        f"corrected energy budget gives. The 1-D model is shedding twice again; "
        f"check the 0.5 * area_factor on its face.")
    assert r.over_material_limit is True, (
        "this duty cycle puts the rotor over its material limit and the flag "
        "must say so; it read False while the model over-shed")
    # the gradient screen is independent of that fix and must stay put
    assert 35.0 < r.dT_gradient_peak_c < 50.0
    assert 60.0 < r.sigma_peak_mpa < 80.0


# --------------------------------------------------------------------------- #
#  Mythbuster rules are physics ASSERTIONS — bind them to the models
# --------------------------------------------------------------------------- #
#  The mythbuster issues VERDICTS on engineering claims, which makes it the
#  highest-stakes accuracy surface in the product: a wrong rule confidently
#  tells a team their correct belief is a myth, and unlike a number nobody
#  sanity-checks a sentence. Every rule restates physics that this repo also
#  COMPUTES — so it is the duplication class again, with prose on one side.
#
#  Demonstrated live: adding rotating inertia to brake_thermal.single_stop left
#  the brake-heat rule still saying "1/2 m v^2", 5% adrift from the model it
#  points the reader at. Nothing flagged it.
def test_mythbuster_verdicts_match_the_models_they_cite():
    from suspension.dynamics import VehicleDynamics, VehicleParams
    import suspension.mythbuster as mb
    import suspension.myth_rules  # noqa: F401  (registers the rules)

    # "stiffer front -> more understeer"
    def balance(k_front):
        p = VehicleParams(roll_stiffness_front=k_front, roll_stiffness_rear=400.0)
        v = VehicleDynamics(p)
        return v.balance_index(v.max_lateral_g())[0]

    seq = [balance(k) for k in (150.0, 400.0, 900.0)]
    assert seq == sorted(seq), f"stiffening the front did not move toward understeer: {seq}"
    assert mb.check("stiffer springs always make the car faster").verdict == mb.Verdict.MYTH

    # "an ARB redistributes transfer, it does not change the total"
    totals, shares = [], []
    for k in (150.0, 400.0, 900.0):
        p = VehicleParams(roll_stiffness_front=k, roll_stiffness_rear=400.0)
        _, info = VehicleDynamics(p).lateral_load_transfer(1.0)
        totals.append(info["ltd_front"] * p.track_front / 1000.0
                      + info["ltd_rear"] * p.track_rear / 1000.0)
        shares.append(info["ltd_front"] / (info["ltd_front"] + info["ltd_rear"]))
    assert max(totals) - min(totals) < 1e-6, \
        f"total transfer moment changed with ARB: {totals} — the rule says it cannot"
    assert max(shares) - min(shares) > 0.2, "ARB did not redistribute; test is vacuous"
    assert mb.check("a stiffer anti-roll bar adds grip").verdict == mb.Verdict.MYTH

    # "lateral transfer is proportional to CG height"
    ratios = []
    for h in (250.0, 300.0, 360.0):
        v = VehicleDynamics(VehicleParams(cg_height=h))
        _, info = v.lateral_load_transfer(1.0)
        ratios.append((info["ltd_front"] + info["ltd_rear"]) / h)
    assert (max(ratios) - min(ratios)) / max(ratios) < 5e-3, \
        f"transfer is not proportional to CG height: {ratios}"

    # "brake heat is MORE than 1/2 m v^2" — the rule must track the model that
    # now includes rotating inertia, not the textbook shorthand it used to quote
    heat = mb.check("does brake heat depend on speed")
    assert heat.verdict == mb.Verdict.DEPENDS
    assert "rotating" in heat.explanation.lower(), (
        "the brake-heat rule still quotes bare 1/2 m v^2 while brake_thermal "
        "includes rotating inertia — the prose has drifted from the model")


def test_every_confident_verdict_is_grounded():
    """A MYTH or TRUE verdict must say where it came from, or not be issued.

    This feature is the highest-stakes accuracy surface in the product. Every
    other output is a number a user might sanity-check; this one issues a
    VERDICT, and nobody sanity-checks a sentence. A wrong rule does not get
    caught — it wins the argument.

    So a confident verdict has to be one of:
      "computed"  a model in this repo was run on the user's context
      "physics"   a first-principles relationship, stated in the explanation
      "asserted"  judgement or convention — permitted ONLY with sources

    That last clause is the point. An asserted MYTH with no citation is an
    opinion wearing the same clothes as a computed result, and the honest form
    of an unsettled answer is "yes / no / maybe — and here is where to look",
    never a bare confident verdict. Verdict.UNVERIFIED exists so a rule can
    decline rather than bluff.

    Enforced here rather than in CheckOutcome.__post_init__ on purpose: raising
    at construction would crash the app for a user whose only sin is asking a
    question an untriaged rule happens to match. Fail the build, not the person.
    """
    import suspension.myth_rules  # noqa: F401  (registers the rules)
    import suspension.mythbuster as mb

    engine = mb.DEFAULT_ENGINE
    rules = engine.rules() if callable(getattr(engine, "rules", None)) \
        else getattr(engine, "_rules", [])
    assert len(rules) > 20, f"only {len(rules)} rules registered; import failed?"

    ungrounded, unreachable = [], []
    for rule in rules:
        claim = getattr(rule.check, "reference_claim", None)
        if not claim:
            continue
        res = mb.check(claim)
        #  Reachability is about which RULE answered, not which verdict it gave.
        #  A rule that matches and returns UNKNOWN ("I need the live motor
        #  envelope — open the EV Powertrain tab") is working exactly as
        #  intended: it recognised the claim and refused to guess without data.
        #  Treating that as unreachable, as this test first did, would push
        #  authors toward guessing rather than asking.
        if res.matched_rule != rule.name:
            unreachable.append(f"{rule.name} (answered by {res.matched_rule})")
            continue
        if res.verdict is mb.Verdict.UNKNOWN:
            continue                      # honest "I need data" — fine
        if res.verdict in (mb.Verdict.MYTH, mb.Verdict.TRUE):
            grounding = getattr(res, "grounding", "asserted")
            if grounding == "asserted" and not getattr(res, "sources", ()):
                ungrounded.append(f"{rule.name} -> {res.verdict.value}")

    assert not ungrounded, (
        "confident verdicts with no grounding and no sources:\n  "
        + "\n  ".join(ungrounded)
        + "\n\nMark it grounding='physics' if the explanation carries the "
          "derivation, 'computed' if it runs a model, or cite sources. If none "
          "of those is honest, return Verdict.UNVERIFIED — 'maybe, check these' "
          "is a better answer than a confident guess.")
    #  Rules that need a live model (motor envelope, fitted tyre) DECLINE when
    #  called with no context, and the fallback answers instead. That is correct
    #  — the alternative is a curated rule guessing without its data. What must
    #  hold is that the fallback never issues a confident ungrounded verdict in
    #  their place, which is the property the demotion above enforces and the
    #  reason this list is reported rather than failed.
    if unreachable:
        for entry in unreachable:
            name = entry.split(" ")[0]
            rule = next(r for r in rules if r.name == name)
            res = mb.check(rule.check.reference_claim)
            #  RESOLVED. Context-needing rules now answer QUALITATIVELY via
            #  mythbuster.preliminary() instead of refusing, so the fallback no
            #  longer substitutes a guess for a reviewed rule. The gate below is
            #  therefore enforced rather than excepted.
            assert res.verdict in (mb.Verdict.UNVERIFIED, mb.Verdict.DEPENDS,
                                   mb.Verdict.UNKNOWN) or res.grounding != "asserted", (
                f"{name} declined for want of context and the fallback answered "
                f"{res.verdict.value} on nothing — that is the exact substitution "
                f"this gate exists to prevent")


def test_unverified_is_available_and_distinct_from_unknown():
    """UNKNOWN means nothing matched. UNVERIFIED means a rule matched, has
    something useful to say, and is declining to adjudicate. Collapsing them
    would hide the difference between 'we have no rule' and 'we will not
    guess'."""
    import suspension.mythbuster as mb
    assert mb.Verdict.UNVERIFIED != mb.Verdict.UNKNOWN
    assert mb.Verdict.UNVERIFIED == "unverified"
    out = mb.CheckOutcome(mb.Verdict.UNVERIFIED, "maybe — depends on your tyre data",
                          sources=("TTC round 9 dataset",))
    assert out.grounding == "asserted" and out.sources
    with pytest.raises(ValueError):
        mb.CheckOutcome(mb.Verdict.MYTH, "x", grounding="vibes")
