# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
AeroOrchestrator — the "A" path top level. Turns a RunMatrix + geometry into an
AeroMap by writing cases, submitting them through a Submitter, and assembling the
usable results. It is solver-agnostic (takes any CFDSolver backend) and
compute-agnostic (takes any Submitter).

It deliberately makes COST visible before work happens: `plan()` returns the case
count and a wall-clock estimate so a 125-case overnight sweep is a conscious choice,
not an accident. Nothing meshes or solves until `run()` is called.
"""

from __future__ import annotations

from dataclasses import dataclass
from collections.abc import Callable

from .cfd import CaseSpec, RunMatrix, SolverFidelity
from .aeromap import AeroMap
from .submit import LocalSubmitter, Submitter, SubmitResult


@dataclass
class OrchestratorReport:
    """What happened in a sweep: the map plus per-case outcomes for honesty."""
    aero_map: AeroMap
    results: list[SubmitResult]

    @property
    def n_total(self) -> int:
        return len(self.results)

    @property
    def n_usable(self) -> int:
        return sum(1 for r in self.results if r.ok)

    @property
    def failures(self) -> list[SubmitResult]:
        return [r for r in self.results if not r.ok]

    @property
    def n_dropped_unusable(self) -> int:
        """Cases that ran without error but were REFUSED by the map.

        AeroMap.add() rejects anything failing is_usable() — non-converged, or
        missing a coefficient. That is the correct behaviour, and since the panel
        solver now sets converged=False on an ill-conditioned system it is how a
        garbage attitude gets kept out of a sweep. But the rejection was silent:
        `n_usable` counts cases that RAN, not cases that landed, so a sweep could
        report "12/12 cases usable" over a map holding 9 points and the three
        holes were invisible. A hole in an aero map is not a neutral absence —
        whatever queries it will interpolate straight across the gap.
        """
        return max(self.n_usable - len(self.aero_map), 0)

    def summary(self) -> str:
        lines = [f"{self.n_usable}/{self.n_total} cases usable; map has "
                 f"{len(self.aero_map)} points."]
        if self.n_dropped_unusable:
            lines.append(
                f"  WARNING: {self.n_dropped_unusable} case(s) ran but were "
                f"refused by the map (not converged, or missing a coefficient). "
                f"The map has holes at those attitudes and will interpolate "
                f"across them — check the per-case notes before using it.")
        for f in self.failures:
            lines.append(f"  ✗ {f.spec.attitude.label()}: {f.error or 'unconverged'}")
        return "\n".join(lines)


class AeroOrchestrator:
    def __init__(self, backend, geometry_path: str,
                 reference_area_m2: float = 1.0,
                 reference_length_m: float = 1.55,
                 rho: float = 1.225,
                 fidelity: SolverFidelity = SolverFidelity.RANS,
                 submitter: Submitter | None = None):
        self.backend = backend
        self.geometry_path = geometry_path
        self.reference_area_m2 = reference_area_m2
        self.reference_length_m = reference_length_m
        self.rho = rho
        self.fidelity = fidelity
        self.submitter = submitter or LocalSubmitter()

    def specs(self, matrix: RunMatrix) -> list[CaseSpec]:
        return [
            CaseSpec(
                attitude=att, geometry_path=self.geometry_path,
                reference_area_m2=self.reference_area_m2,
                reference_length_m=self.reference_length_m,
                rho=self.rho, fidelity=self.fidelity,
            )
            for att in matrix.attitudes()
        ]

    def plan(self, matrix: RunMatrix, minutes_per_case: float = 180.0,
             concurrent: int = 1) -> str:
        """Cost preview — call this and show the user BEFORE run()."""
        return matrix.cost_summary(minutes_per_case, concurrent)

    def run(self, matrix: RunMatrix, workdir: str,
            progress: Callable[[int, int, SubmitResult], None] | None = None
            ) -> OrchestratorReport:
        specs = self.specs(matrix)
        results = self.submitter.submit_all(self.backend, specs, workdir, progress)
        amap = AeroMap(self.reference_area_m2, self.reference_length_m,
                       provenance=self.backend.provenance()
                       if hasattr(self.backend, "provenance") else None)
        for r in results:
            if r.result is not None:
                amap.add(r.result)
        return OrchestratorReport(aero_map=amap, results=results)
