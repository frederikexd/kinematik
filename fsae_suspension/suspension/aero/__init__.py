# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

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

Virtual Wind Tunnel (the CFD-validation path):
    A physical aero-map run in a wind tunnel exists mainly to CALIBRATE CFD: map the
    car over front/rear ride height, then run the IDENTICAL points in CFD and check
    that C_d/C_l/balance match. `windtunnel.py` owns that loop:
        from suspension.aero import (PhysicalAeroMap, TunnelProvenance, RideHeights,
                                     VirtualWindTunnel, StarCCMSolver)
        phys = PhysicalAeroMap(TunnelProvenance("A2"), reference_area_m2=1.0)
        phys.add_measurement(RideHeights(20, 40, 25.0), c_lift=-2.8, c_drag=1.05)
        vwt = VirtualWindTunnel(phys, "car.stl")
        specs = vwt.case_specs()           # exact same points, for Star-CCM+/TS-Auto
        # ... team runs the CFD, reads results back ...
        report = vwt.correlate(cfd_results)  # is k-omega SST calibrated to the tunnel?

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
    ReferenceAeroModel, OpenFOAMSolver, StarCCMSolver, FluentSolver, TSAutoSolver,
    BACKENDS, get_backend,
)
from .submit import (
    Submitter, LocalSubmitter, SlurmSSHSubmitter, SubmitResult,
)
from .aeromap import AeroMap, AeroQuery
from .orchestrator import AeroOrchestrator, OrchestratorReport
from .coupling import AeroProvider, estimate_attitude, attitude_from_dynamics
from .meshing import MeshParams, SnappyMesher, parse_checkmesh
from .windtunnel import (
    RideHeights, AeroMapGrid, GroundState, TunnelProvenance, PhysicalAeroMap,
    VirtualWindTunnel, PointCorrelation, TunnelCorrelationReport,
    ride_heights_to_attitude, attitude_to_ride_heights,
    downforce_to_clift, drag_to_cdrag, DEFAULT_TUNNEL_TOL,
)
from .piv import (
    SheetOrientation, LaserSheetPlane, PIVProvenance, FramePair, VelocityField,
    PIVProcessor, AcquisitionPlan, PIVRig, OfflinePIVRig, RigUnavailable,
    CFDFieldSlice, FieldCorrelationReport, correlate_field, separation_mask,
    DEFAULT_FIELD_TOL,
)

__all__ = [
    "Attitude", "RunMatrix", "CaseSpec", "CoeffResult", "CFDProvenance",
    "SolverFidelity", "CFDSolver", "SolverUnavailable",
    "ReferenceAeroModel", "OpenFOAMSolver", "StarCCMSolver", "FluentSolver",
    "TSAutoSolver", "BACKENDS", "get_backend",
    "Submitter", "LocalSubmitter", "SlurmSSHSubmitter", "SubmitResult",
    "AeroMap", "AeroQuery",
    "AeroOrchestrator", "OrchestratorReport",
    "AeroProvider", "estimate_attitude", "attitude_from_dynamics",
    "MeshParams", "SnappyMesher", "parse_checkmesh",
    "RideHeights", "AeroMapGrid", "GroundState", "TunnelProvenance",
    "PhysicalAeroMap", "VirtualWindTunnel", "PointCorrelation",
    "TunnelCorrelationReport", "ride_heights_to_attitude",
    "attitude_to_ride_heights", "downforce_to_clift", "drag_to_cdrag",
    "DEFAULT_TUNNEL_TOL",
    "SheetOrientation", "LaserSheetPlane", "PIVProvenance", "FramePair",
    "VelocityField", "PIVProcessor", "AcquisitionPlan", "PIVRig",
    "OfflinePIVRig", "RigUnavailable", "CFDFieldSlice",
    "FieldCorrelationReport", "correlate_field", "separation_mask",
    "DEFAULT_FIELD_TOL",
]
