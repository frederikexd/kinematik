# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Virtual Tunnel Solver — one solver built ON TOP of STAR-CCM+, TS-Auto and OpenFOAM.

WHY THIS MODULE EXISTS (read this before using it)
---------------------------------------------------
The old Virtual Wind Tunnel made the user pick ONE code — Star-CCM+ *or* TS-Auto
*or* OpenFOAM — and correlate that single solver against the tunnel. That framing
quietly hides the most useful number in multi-code aero work: **how much the codes
disagree with each other.** Two independent solvers landing on the same C_l at the
same attitude is strong evidence the number is real and not a meshing/turbulence
artefact; the same two diverging by 8% is a red flag that no single-solver report
would ever show you.

So this module does NOT add a fourth backend. It adds a *meta-solver* — the
`EnsembleTunnelSolver` — that IS the Virtual Tunnel Solver, built out of the three
real backends underneath it (`StarCCMSolver`, `TSAutoSolver`, `OpenFOAMSolver`).
It implements the very same `CFDSolver` seam every other backend implements
(`write_case` / `run_case` / `read_result` + `provenance`), so it is a drop-in
solver everywhere the old single backends plugged in — including
`VirtualWindTunnel.case_specs()` / `.correlate()`. The difference is what happens
inside:

    write_case   -> writes ALL THREE codes' input for one attitude into per-code
                    sub-directories (a Star-CCM+ macro, a TS-Auto config, and a
                    valid OpenFOAM case), so a team can run the same point through
                    every code with one call.
    run_case     -> drives each member; OpenFOAM actually runs if it is on PATH,
                    the licensed codes are read back if their result CSV is already
                    staged. Members that cannot run are recorded as honest holes,
                    NOT dropped silently and NEVER faked.
    read_result  -> parses whatever each member produced, then FUSES the members
                    into one `CoeffResult` whose coefficients are the cross-code
                    consensus and whose provenance carries the inter-code spread.

THE HONESTY CONTRACT (same discipline as cfd.py / backends.py)
--------------------------------------------------------------
This is the dangerous part of an ensemble, so it is the strict part here:

  * The fused coefficient is a transparent reduction (mean / median) of ONLY the
    members that actually produced a converged number. A member that raised
    SolverUnavailable or did not converge contributes NOTHING — it is not counted,
    not zero-filled, not guessed.
  * The fused result is `converged` only if at least `min_members` members
    converged AND their inter-code spread is within `agreement_tol`. Codes that
    disagree wildly are NOT a converged consensus, and the result says so.
  * Every fused `CoeffResult` carries, in its provenance and notes, exactly which
    codes were used, which were holes and why, and the per-channel spread. You can
    always see how many codes voted and how far apart they were.
  * `c_lift` sign convention is preserved end to end (negative = downforce), since
    the member backends already normalise the vendor up-positive convention.

DELIBERATE NON-GOALS, identical to the seam it sits on: this module meshes nothing,
solves no Navier-Stokes itself, and invents no coefficient. It orchestrates the
three real backends and reduces their honest output. A hole it reports is a real
hole; a disagreement it reports is a real disagreement.
"""

from __future__ import annotations

import os
import statistics
from dataclasses import dataclass, field
from typing import Optional, Sequence

from .cfd import (
    Attitude, CaseSpec, CoeffResult, CFDProvenance, SolverFidelity,
    SolverUnavailable,
)
from .backends import StarCCMSolver, TSAutoSolver, OpenFOAMSolver


# --------------------------------------------------------------------------- #
#  Default ensemble of the three codes the Virtual Tunnel Solver is built on
# --------------------------------------------------------------------------- #
# Order is the canonical reporting order (commercial high-fidelity first, the
# productised OpenFOAM workflow second, the open core last) — it has no bearing on
# the fused number, which is symmetric across members.
DEFAULT_MEMBER_NAMES = ("starccm", "tsauto", "openfoam")


def _default_members(turbulence_model: str, fidelity: SolverFidelity,
                      mesh_params=None) -> "list":
    """
    Construct the three real backends the Virtual Tunnel Solver is built on. The
    licensed codes (Star-CCM+, TS-Auto) take only what their constructors accept;
    OpenFOAM additionally takes the turbulence model and an optional mesher so the
    one member that can actually run here is fully driven.
    """
    return [
        StarCCMSolver(fidelity=fidelity),
        TSAutoSolver(turbulence_model=turbulence_model, fidelity=fidelity),
        OpenFOAMSolver(turbulence_model=turbulence_model, fidelity=fidelity,
                       mesh_params=mesh_params),
    ]


# --------------------------------------------------------------------------- #
#  Per-member outcome — one code's contribution to one fused point
# --------------------------------------------------------------------------- #
@dataclass
class MemberOutcome:
    """
    What ONE code did at ONE attitude. `result` is the member's CoeffResult if it
    produced one; `error` is the actionable reason it didn't (e.g. the
    SolverUnavailable message a licensed stub raises here). Exactly one of the two
    is set. This is the audit trail behind a fused number: you can always see which
    codes voted and which were holes, and why.
    """
    backend: str
    result: Optional[CoeffResult] = None
    error: str = ""

    @property
    def ok(self) -> bool:
        """A usable vote: the member ran, converged, and has lift+drag."""
        return self.result is not None and self.result.is_usable()

    @property
    def ran(self) -> bool:
        """The member produced SOME result (maybe unconverged), not an exception."""
        return self.result is not None


# --------------------------------------------------------------------------- #
#  Fused result — the consensus coefficient plus the inter-code spread
# --------------------------------------------------------------------------- #
@dataclass
class EnsembleResult:
    """
    The Virtual Tunnel Solver's answer at one attitude: the fused `CoeffResult`
    (cross-code consensus, the object the rest of KinematiK consumes) plus the raw
    per-member outcomes and the per-channel spread that earned that consensus its
    `converged` verdict. `spread_pct` is the peak-to-peak disagreement between the
    converged members as a percentage of their mean — the single number that says
    whether the codes actually agree.
    """
    fused: CoeffResult
    members: list                          # list[MemberOutcome]
    n_voted: int
    cl_spread_pct: float = float("nan")
    cd_spread_pct: float = float("nan")

    def as_dict(self):
        return dict(
            attitude=self.fused.attitude.label(),
            c_lift=self.fused.c_lift, c_drag=self.fused.c_drag,
            converged=self.fused.converged,
            n_voted=self.n_voted,
            cl_spread_pct=self.cl_spread_pct,
            cd_spread_pct=self.cd_spread_pct,
            members=[{"backend": m.backend,
                      "ok": m.ok,
                      "c_lift": (m.result.c_lift if m.ran else None),
                      "c_drag": (m.result.c_drag if m.ran else None),
                      "error": m.error}
                     for m in self.members],
        )


def _spread_pct(values: Sequence[float]) -> float:
    """Peak-to-peak as a percentage of |mean|; nan if <2 values or mean ~0."""
    vals = [v for v in values if v is not None]
    if len(vals) < 2:
        return float("nan")
    mean = sum(vals) / len(vals)
    if abs(mean) < 1e-12:
        return float("nan")
    return 100.0 * (max(vals) - min(vals)) / abs(mean)


def _reduce(values: Sequence[float], how: str) -> Optional[float]:
    vals = [v for v in values if v is not None]
    if not vals:
        return None
    if how == "median":
        return float(statistics.median(vals))
    return float(sum(vals) / len(vals))      # default: mean


# --------------------------------------------------------------------------- #
#  The Virtual Tunnel Solver — a CFDSolver built on the three codes
# --------------------------------------------------------------------------- #
class EnsembleTunnelSolver:
    """
    The Virtual Tunnel Solver. It implements the `CFDSolver` protocol (so it drops
    into `VirtualWindTunnel`, `AeroOrchestrator`, and anywhere a backend is taken),
    but instead of being one code it ORCHESTRATES the three real backends —
    Star-CCM+, TS-Auto and OpenFOAM — at each attitude and FUSES their converged
    output into a single cross-code consensus coefficient.

    The point of the fusion is the inter-code spread: agreement between independent
    solvers is the strongest cheap evidence a CFD number is physical, and that
    spread is recorded on every result rather than hidden behind a single code's
    confident-looking output.

    Parameters
    ----------
    reduction       : "mean" (default) or "median" — how converged members are
                      combined into the consensus coefficient.
    agreement_tol   : maximum inter-code spread (% of mean, peak-to-peak) for the
                      fused result to be called `converged`. Above this the codes
                      disagree and the consensus is reported NOT converged.
    min_members     : minimum number of converged members required to fuse at all.
                      Default 2 — a "consensus" of one code is just that code.
    turbulence_model: passed to the members that accept it (TS-Auto, OpenFOAM).
    fidelity        : labelled fidelity of the ensemble (the members share it).
    mesh_params     : optional MeshParams handed to the OpenFOAM member so the one
                      code that can run here is fully driven; None => solver files
                      only, the team supplies the mesh.
    members         : advanced — supply your own list of CFDSolver backends to
                      ensemble instead of the default three (used by tests).
    """
    name = "virtual-tunnel"

    def __init__(self,
                 reduction: str = "mean",
                 agreement_tol: float = 5.0,
                 min_members: int = 2,
                 turbulence_model: str = "kOmegaSST",
                 fidelity: SolverFidelity = SolverFidelity.RANS,
                 mesh_params=None,
                 members: "Optional[list]" = None):
        if reduction not in ("mean", "median"):
            raise ValueError("reduction must be 'mean' or 'median'")
        self.reduction = reduction
        self.agreement_tol = float(agreement_tol)
        self.min_members = max(1, int(min_members))
        self.turbulence_model = turbulence_model
        self.fidelity = fidelity
        self.members = (members if members is not None
                        else _default_members(turbulence_model, fidelity,
                                              mesh_params))
        if not self.members:
            raise ValueError("EnsembleTunnelSolver needs at least one member backend")
        self._member_names = [getattr(m, "name", f"member{i}")
                              for i, m in enumerate(self.members)]

    # -- provenance ------------------------------------------------------- #
    def provenance(self, n_voted: Optional[int] = None,
                   spread_pct: Optional[float] = None,
                   member_names: "Optional[list]" = None) -> CFDProvenance:
        members = member_names if member_names is not None else self._member_names
        roster = "+".join(members)
        vote = "" if n_voted is None else f", {n_voted}/{len(members)} codes voted"
        spr = "" if spread_pct is None or spread_pct != spread_pct \
            else f", inter-code spread {spread_pct:.1f}%"
        return CFDProvenance(
            backend=f"{self.name}[{roster}]",
            fidelity=self.fidelity,
            is_correlated=False,
            turbulence_model=self.turbulence_model,
            notes=("Virtual Tunnel Solver — cross-code consensus of Star-CCM+, "
                   "TS-Auto and OpenFOAM. The fused coefficient is the "
                   f"{self.reduction} of the converged members only; members that "
                   "could not run or did not converge contribute nothing and are "
                   "recorded as holes. Inter-code agreement is the confidence "
                   "signal — correlate against the physical tunnel map before "
                   f"trusting absolute levels{vote}{spr}."),
        )

    # -- the CFDSolver seam: write / run / read --------------------------- #
    def write_case(self, spec: CaseSpec, workdir: str) -> str:
        """
        Write EVERY member code's input for this attitude, each into its own
        sub-directory under <workdir>/<case_name>/<member>. Returns the parent case
        directory. A team can then run the same point through all three codes from
        one place; the licensed codes get their driver files, OpenFOAM gets a valid
        runnable case.
        """
        case_dir = os.path.join(workdir, spec.case_name())
        os.makedirs(case_dir, exist_ok=True)
        for member, mname in zip(self.members, self._member_names):
            sub = os.path.join(case_dir, mname)
            os.makedirs(sub, exist_ok=True)
            try:
                member.write_case(spec, sub)
            except Exception as e:                          # noqa: BLE001
                # A member that cannot even write its input is recorded, not fatal —
                # the other codes still get written. We leave a breadcrumb file.
                with open(os.path.join(sub, "WRITE_FAILED.txt"), "w") as f:
                    f.write(f"{mname} write_case failed: {e}\n")
        return case_dir

    def run_case(self, spec: CaseSpec, workdir: str) -> CoeffResult:
        """
        Drive every member at this attitude, then fuse. OpenFOAM runs if it is on
        PATH; the licensed members run only if KinematiK can (they raise
        SolverUnavailable here, which is captured as a hole, never faked). The
        returned CoeffResult is the fused consensus; the full per-member breakdown
        is available via `solve_detailed`.
        """
        return self.solve_detailed(spec, workdir).fused

    def read_result(self, spec: CaseSpec, workdir: str) -> CoeffResult:
        """
        Parse whatever each member already produced under its sub-directory and fuse,
        without launching anything. Use this after a team has run the written cases
        on their cluster and staged each code's result back.
        """
        return self._fuse_from(spec, workdir, run=False).fused

    # -- the ensemble engine ---------------------------------------------- #
    def solve_detailed(self, spec: CaseSpec, workdir: str) -> EnsembleResult:
        """Run (where possible) + fuse, returning the full EnsembleResult."""
        return self._fuse_from(spec, workdir, run=True)

    def _fuse_from(self, spec: CaseSpec, workdir: str, run: bool) -> EnsembleResult:
        case_dir = os.path.join(workdir, spec.case_name())
        outcomes: list[MemberOutcome] = []
        for member, mname in zip(self.members, self._member_names):
            sub = os.path.join(case_dir, mname)
            os.makedirs(sub, exist_ok=True)
            outcomes.append(self._drive_member(member, mname, spec, sub, run))
        return self._fuse(spec, outcomes)

    def _drive_member(self, member, mname: str, spec: CaseSpec, sub: str,
                      run: bool) -> MemberOutcome:
        """Run or read ONE member, capturing an unavailable/failed code as a hole."""
        try:
            if run:
                res = member.run_case(spec, sub)
            else:
                res = member.read_result(spec, sub)
            return MemberOutcome(backend=mname, result=res)
        except SolverUnavailable as e:
            return MemberOutcome(backend=mname, error=str(e))
        except Exception as e:                              # noqa: BLE001
            return MemberOutcome(backend=mname, error=f"{type(e).__name__}: {e}")

    def _fuse(self, spec: CaseSpec, outcomes: "list") -> EnsembleResult:
        """
        Reduce the converged members into one consensus CoeffResult. ONLY usable
        members vote; the spread between them sets the converged verdict. Nothing is
        invented to fill a hole.
        """
        voting = [m for m in outcomes if m.ok]
        n_voted = len(voting)

        # No usable member: an honest, fully-unconverged hole carrying the reasons.
        if n_voted == 0:
            why = "; ".join(f"{m.backend}: {m.error or 'no usable result'}"
                            for m in outcomes)
            fused = CoeffResult(
                attitude=spec.attitude,
                converged=False,
                provenance=self.provenance(n_voted=0,
                                           member_names=[m.backend for m in outcomes]),
                notes=f"Virtual Tunnel Solver: no code produced a usable result — {why}",
            )
            return EnsembleResult(fused=fused, members=outcomes, n_voted=0)

        cls = [m.result.c_lift for m in voting]
        cds = [m.result.c_drag for m in voting]
        csides = [m.result.c_side for m in voting if m.result.c_side is not None]
        cpitch = [m.result.c_pitch for m in voting if m.result.c_pitch is not None]
        bals = [m.result.aero_balance_front for m in voting
                if m.result.aero_balance_front is not None]

        cl_spread = _spread_pct(cls)
        cd_spread = _spread_pct(cds)

        # Converged consensus requires enough codes AND that they agree. With a
        # single voting member spread is nan (no disagreement to measure) — then the
        # min_members gate alone decides, so a lone code can only pass if you set
        # min_members=1 on purpose.
        enough = n_voted >= self.min_members
        agree = True
        for s in (cl_spread, cd_spread):
            if s == s and s > self.agreement_tol:        # s==s filters nan
                agree = False
        converged = bool(enough and agree)

        # Worst-channel spread, for the human-facing number on the result.
        worst_spread = max((s for s in (cl_spread, cd_spread) if s == s),
                           default=float("nan"))

        note = self._fuse_note(outcomes, n_voted, cl_spread, cd_spread,
                               enough, agree)
        fused = CoeffResult(
            attitude=spec.attitude,
            c_lift=_reduce(cls, self.reduction),
            c_drag=_reduce(cds, self.reduction),
            c_side=_reduce(csides, self.reduction) if csides else None,
            c_pitch=_reduce(cpitch, self.reduction) if cpitch else None,
            aero_balance_front=_reduce(bals, self.reduction) if bals else None,
            converged=converged,
            force_monitor_range=worst_spread / 100.0 if worst_spread == worst_spread
            else None,
            provenance=self.provenance(n_voted=n_voted, spread_pct=worst_spread,
                                       member_names=[m.backend for m in voting]),
            notes=note,
        )
        return EnsembleResult(fused=fused, members=outcomes, n_voted=n_voted,
                              cl_spread_pct=cl_spread, cd_spread_pct=cd_spread)

    def _fuse_note(self, outcomes, n_voted, cl_spread, cd_spread,
                   enough, agree) -> str:
        voted = ", ".join(m.backend for m in outcomes if m.ok)
        holes = [m for m in outcomes if not m.ok]
        head = (f"Virtual Tunnel Solver: {self.reduction} consensus of {n_voted} "
                f"code(s) [{voted}]")
        spr = ""
        if cl_spread == cl_spread or cd_spread == cd_spread:
            spr = (f"; inter-code spread C_l {cl_spread:.1f}% / C_d {cd_spread:.1f}%")
        verdict = ""
        if not enough:
            verdict = (f"; NOT converged — only {n_voted} code(s) voted, "
                       f"need {self.min_members}")
        elif not agree:
            verdict = ("; NOT converged — codes disagree beyond "
                       f"{self.agreement_tol:.0f}% (treat as a flag, not a number)")
        else:
            verdict = "; converged consensus (codes agree within tolerance)"
        hole_txt = ""
        if holes:
            hole_txt = "; holes: " + ", ".join(
                f"{h.backend} ({(h.error or 'unconverged')[:48]}"
                + ("…" if len(h.error) > 48 else "") + ")" for h in holes)
        return head + spr + verdict + hole_txt

    # -- batch convenience over a whole matched run ----------------------- #
    def solve_matrix(self, specs: "Sequence[CaseSpec]", workdir: str,
                     run: bool = True) -> "list":
        """
        Drive + fuse a whole list of matched CaseSpecs (e.g. the output of
        `VirtualWindTunnel.case_specs()`), returning one EnsembleResult per point.
        The fused CoeffResults inside are exactly what `VirtualWindTunnel.correlate`
        consumes — so the consensus, not any single code, is what gets compared to
        the physical tunnel map.
        """
        out = []
        for s in specs:
            out.append(self.solve_detailed(s, workdir) if run
                       else self._fuse_from(s, workdir, run=False))
        return out


def fused_results(ensemble_results: "Sequence[EnsembleResult]") -> "list":
    """Pull the fused CoeffResults out of a list of EnsembleResults (for correlate)."""
    return [er.fused for er in ensemble_results]
