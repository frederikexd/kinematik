# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Who owns which physical quantity — and proof that the copies agree.

WHY THIS FILE EXISTS
--------------------
Six of the thirty defects found in the 2026-08 accuracy audit were the same
architectural gap: one physical quantity implemented twice, in two modules, and
the two had drifted.

    anti-dive          kinematics.py  and  adapter.py       — diverged, sign
    roll centre        dynamics.py    and  ghost_topology   — diverged, degenerate branch
    regen energy       ev_powertrain  and  pack_thermal     — diverged after a fix
    reference geometry kinematics.py  and  topologies.py    — copy-pasted literal
    evidence grades    proof_engine   and  risk_propagation — "measured" meant two things
    envelope interior  laptime.py     and  ggv.py           — 43% apart off-axis

The dependency graph is not the problem. 99 modules, two local import cycles,
maximum fan-out of five, a clean kernel. Modules here are decoupled enough that
reimplementing a neighbour costs nothing — which is exactly what keeps
happening. The gap is GOVERNANCE, not coupling: nothing declares who owns a
quantity, so a second implementation is invisible until someone diffs the
numbers.

And the divergences hide in the same place every time. Two copies agree in the
main branch and split in the guard, or on the axes and not in the interior, or
at 50/50 weight distribution and nowhere else. The exercised path stays right;
the rare path rots. So agreement has to be checked by RUNNING BOTH, across a
range, not by reading either.

HOW TO USE IT
-------------
* One OWNER per quantity: the module a reader should go read, and the one a fix
  belongs in.
* MIRRORS are legitimate second implementations — a generic solver, a different
  topology kernel, a standalone lookup table. They must agree with the owner
  numerically, and `tests/test_physics_ownership.py` runs both and compares.
* DELEGATES are thin wrappers that call the owner. `dynamics.anti_dive_pct` is
  one: it forwards to `kinematics`, adds no physics, and needs no cross-check.
  Recording them matters anyway, so nobody "fixes" a wrapper by giving it its
  own maths.

Adding a new implementation of something already owned means either registering
it as a MIRROR with a comparison, or not adding it. That is the point.
"""
from __future__ import annotations

from dataclasses import dataclass, field

__all__ = ["Quantity", "PHYSICS_OWNERS", "owner_of", "mirrors_of", "unowned_report"]


@dataclass(frozen=True)
class Quantity:
    """One physical quantity and every place it is computed."""

    key: str
    description: str
    owner: str                              # module path that defines the truth
    #: Independent implementations that MUST agree numerically with the owner.
    mirrors: tuple[str, ...] = ()
    #: Thin wrappers that call the owner. No physics of their own, nothing to check.
    delegates: tuple[str, ...] = ()
    #: How the agreement is proven, and anything known to limit it.
    agreement: str = ""


PHYSICS_OWNERS: tuple[Quantity, ...] = (
    Quantity(
        key="anti_dive_pct",
        description="Front anti-dive, % — outboard brakes, contact-patch reference",
        owner="suspension/kinematics.py",
        mirrors=("suspension/adapter.py",),
        delegates=("suspension/dynamics.py",),
        agreement=(
            "Native and generic solvers agree to 4e-6 on the shared reference "
            "geometry. Independently confirmed by static equilibrium of the "
            "upright (26.220% both ways) and by a virtual-work derivation. They "
            "once differed because each built a side-view instant centre from a "
            "different pair of carrier points — for a non-planar linkage that "
            "centre is not a shared object, so both now use the force point's "
            "own path slope."),
    ),
    Quantity(
        key="anti_squat_pct",
        description="Rear anti-squat, % — inboard drive, wheel-centre reference",
        owner="suspension/kinematics.py",
        mirrors=("suspension/adapter.py",),
        delegates=("suspension/dynamics.py",),
        agreement=(
            "Agree to 4e-6. Static equilibrium puts the wheel-centre shortcut "
            "at 17.1% against 16.1% for the fully modelled halfshaft torque "
            "path — the shortcut is exact only at zero camber and zero scrub, "
            "leaving ~1 pp. Bounded by test_anti_squat_shortcut_error_is_bounded."),
    ),
    Quantity(
        key="roll_center_height",
        description="Front-view roll-centre height from one corner's IC, mm",
        owner="suspension/dynamics.py",
        mirrors=("suspension/ghost_topology.py",),
        agreement=(
            "Identical through travel. They split only in the degenerate branch: "
            "with the IC directly above the contact patch the roll centre is at "
            "infinity, and dynamics returned contact-patch height — about 0 mm, "
            "which reads as a legitimate design target rather than a failure, "
            "and slipped past an isfinite() guard built for exactly that case."),
    ),
    Quantity(
        key="front_view_instant_center",
        description="Front-view instant centre of the upright, (y, z) mm",
        owner="suspension/kinematics.py",
        mirrors=("suspension/adapter.py",),
        agreement=(
            "Both use the exact velocity construction (axis x r, perpendiculars "
            "intersected). Reduces to the classic hand construction when the "
            "pivot axes are parallel to x; pinned by "
            "test_instant_centre_matches_hand_construction_on_flat_pickups."),
    ),
    Quantity(
        key="longitudinal_envelope",
        description="Available longitudinal g at speed under combined slip",
        owner="suspension/laptime.py",
        mirrors=("suspension/ggv.py", "suspension/lapsim.py"),
        agreement=(
            "laptime and ggv agree to 0.2% across the envelope INTERIOR, not "
            "merely on its three axes — the pre-existing cross-check sampled "
            "only the axes and passed while the two were 43% apart two degrees "
            "off them. lapsim is a separate point-mass solver with no "
            "friction-circle coupling by design; it is a MIRROR for the brake "
            "ceiling only, not for the interior."),
    ),
    Quantity(
        key="lateral_limit_at_speed",
        description="Peak lateral g including aero downforce, load-sensitive",
        owner="suspension/ggv.py",
        delegates=("suspension/laptime.py",),
        agreement=(
            "laptime calls ggv._max_lateral_g_at_speed rather than scaling mu "
            "linearly with downforce. Tyre mu falls with load, so the linear "
            "form overstated the limit by 4.9% at 30 m/s."),
    ),
    Quantity(
        key="regen_energy",
        description="Energy recoverable under braking, J",
        owner="suspension/ev_powertrain.py",
        mirrors=("suspension/pack_thermal.py",),
        agreement=(
            "The current trace integrates to the same energy as the energy "
            "model, 0.047% apart (discretisation only). Both subtract drag and "
            "rolling resistance, which are not recoverable — pack_thermal "
            "silently kept the old form after ev_powertrain was fixed."),
    ),
    Quantity(
        key="reference_geometry",
        description="The default double-wishbone hardpoint set",
        owner="suspension/kinematics.py",
        delegates=("suspension/topologies.py",),
        agreement=(
            "topologies.example('double_wishbone') reads Hardpoints.default() "
            "rather than repeating the literals. It was a copy, so the "
            "cross-solver agreement test silently compared two different cars."),
    ),
    Quantity(
        key="evidence_grade",
        description="How well a number is known",
        owner="suspension/proof_engine.py",
        mirrors=("suspension/risk_propagation.py",),
        agreement=(
            "risk_propagation.Confidence is a different axis (how an EDGE was "
            "derived) and maps onto EvidenceGrade via .evidence_grade, ceiling "
            "MODELLED. Its MEASURED once meant 'a solver produced this', which "
            "in the canonical vocabulary is MODELLED — the same badge claiming "
            "hardware where there was none."),
    ),
    Quantity(
        key="rotor_bulk_temperature",
        description="Rotor bulk temperature under a duty cycle, degC",
        owner="suspension/brake_thermal.py::TwoNodeRotorPad",
        mirrors=("suspension/brake_thermal.py::OneDRotor",),
        agreement=(
            "TWO OPPOSING EFFECTS that partially cancelled, which is why the raw "
            "comparison looked like one 28.8% gap and hid both.\n"
            "  (1) BUG, fixed. The 1-D model solves HALF the thickness and takes "
            "half the heat, but applied the FULL area_factor to its single face "
            "and implied the other half by symmetry — shedding twice. Halving it "
            "is demanded by that model's own energy budget, not tuned to match "
            "the lumped one. Endpoint moves -28.8% -> +14.8%.\n"
            "  (2) SCOPE, not an error. The 2-node conducts into a pad node and "
            "sheds from it; the 1-D has no pad. Remove the pad path from the "
            "2-node and the two agree to 4.0% — discretisation plus the "
            "surface-vs-bulk definition.\n"
            "So the models are now consistent where they model the same thing. "
            "The residual +14.8% is the pad, and it means the 1-D runs HOTTER: "
            "conservative for a rotor, but do not quote both figures in one "
            "report. Adding a pad node to the 1-D would close it. Bounded by "
            "test_rotor_models_gap_does_not_widen."),
    ),
    Quantity(
        key="awg_area_mm2",
        description="Conductor cross-section by AWG, mm^2",
        owner="suspension/harness.py",
        mirrors=("suspension/fuse_test.py",),
        agreement="Agree to rounding (<0.1%) on all nine shared gauges.",
    ),
    Quantity(
        key="air_density",
        description="Air density, kg/m^3 (ISA sea level)",
        owner="suspension/laptime.py",
        mirrors=("suspension/pt_integration.py", "suspension/ggv.py",
                 "suspension/aero/scale_model.py"),
        agreement=(
            "All 1.225. Enforced by test_shared_constants_do_not_drift; three "
            "modules were on 1.20, making their drag and downforce 2% low "
            "against the aero stack they are cross-checked with."),
    ),
)

_BY_KEY = {q.key: q for q in PHYSICS_OWNERS}


def owner_of(key: str) -> str | None:
    q = _BY_KEY.get(key)
    return q.owner if q else None


def mirrors_of(key: str) -> tuple[str, ...]:
    q = _BY_KEY.get(key)
    return q.mirrors if q else ()


def unowned_report() -> str:
    """Human-readable dump, for a reader deciding where a fix belongs."""
    lines = []
    for q in PHYSICS_OWNERS:
        lines.append(f"{q.key}  —  {q.description}")
        lines.append(f"    owner    : {q.owner}")
        if q.mirrors:
            lines.append(f"    mirrors  : {', '.join(q.mirrors)}")
        if q.delegates:
            lines.append(f"    delegates: {', '.join(q.delegates)}")
        lines.append("")
    return "\n".join(lines)
