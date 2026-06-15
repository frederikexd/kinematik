"""
KinematiK aero co-simulation package.

The aero analogue of `tire_cosim`: a clean, typed, tested SEAM where an external
CFD solver (OpenFOAM / STAR-CCM+ / Fluent) plugs in, an orchestrator that sweeps
car attitude into an aero map, and a coupling that feeds that map back into the
existing point-mass lap sim. KinematiK owns the parameterisation, orchestration and
map; the meshing and the Navier–Stokes solve live OUTSIDE it, on the team's cluster
with the team's license. Provenance is first-class throughout — a CFD number is
never fabricated to fill a hole.

Two entry paths:
  * "A" — orchestrate runs:  AeroOrchestrator(backend, geometry).run(RunMatrix(...))
  * "B" — bring a map:        AeroMap.from_csv(text)  ->  AeroProvider(...)

Quick start (runnable today, no solver):
    from suspension.aero import (ReferenceAeroModel, AeroOrchestrator, RunMatrix)
    orch = AeroOrchestrator(ReferenceAeroModel(), "car.stl", reference_area_m2=1.0)
    print(orch.plan(RunMatrix(yaw_deg=[0,2,4,6])))     # cost preview
    report = orch.run(RunMatrix(yaw_deg=[0,2,4,6]), workdir="/tmp/sweep")
    amap = report.aero_map
"""

from .cfd import (
    Attitude, RunMatrix, CaseSpec, CoeffResult, CFDProvenance,
    SolverFidelity, CFDSolver, SolverUnavailable,
)
from .backends import (
    ReferenceAeroModel, OpenFOAMSolver, StarCCMSolver, FluentSolver,
    BACKENDS, get_backend,
)
from .submit import (
    Submitter, LocalSubmitter, SlurmSSHSubmitter, SubmitResult,
)
from .aeromap import AeroMap, AeroQuery
from .orchestrator import AeroOrchestrator, OrchestratorReport
from .coupling import AeroProvider, estimate_attitude, attitude_from_dynamics
from .meshing import MeshParams, SnappyMesher, parse_checkmesh

__all__ = [
    "Attitude", "RunMatrix", "CaseSpec", "CoeffResult", "CFDProvenance",
    "SolverFidelity", "CFDSolver", "SolverUnavailable",
    "ReferenceAeroModel", "OpenFOAMSolver", "StarCCMSolver", "FluentSolver",
    "BACKENDS", "get_backend",
    "Submitter", "LocalSubmitter", "SlurmSSHSubmitter", "SubmitResult",
    "AeroMap", "AeroQuery",
    "AeroOrchestrator", "OrchestratorReport",
    "AeroProvider", "estimate_attitude", "attitude_from_dynamics",
    "MeshParams", "SnappyMesher", "parse_checkmesh",
]
