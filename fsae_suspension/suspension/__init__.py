# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""
KinematiK public package API.

This package spans many disciplines — kinematics, vehicle dynamics, tyre
models, lap/GGV simulation, EV powertrain & energy, battery-pack and tyre
thermals, the tyre/CFD co-simulation boundaries, the cross-discipline
interface ledger, electronics/harness checks, and the 3-D viewers.

Why this file uses LAZY imports (PEP 562 ``__getattr__``)
---------------------------------------------------------
Historically this ``__init__`` eagerly imported every submodule at package
load. That meant ``import suspension`` — or even ``from suspension.interfaces
import Severity`` — transitively pulled in scipy (kinematics), plotly
(fullcar3d) and trimesh (chassis), because the package body ran first. So the
pure-standard-library integration ledger and the powertrain myth-checker, both
of which depend on *nothing* heavy, could not be imported or unit-tested
without the entire scientific stack installed. That is the opposite of what the
ledger is for.

The public API is UNCHANGED. Every name and submodule that used to be importable
from ``suspension`` still is::

    from suspension import SuspensionKinematics, build_full_car_figure   # works
    import suspension; suspension.fullcar3d                              # works
    from suspension.interfaces import Severity                           # now free

The difference is *when* the cost is paid: importing the package itself is now
free, and each feature pays its own import cost the first time you touch one of
its names. A lead who only wants the interface ledger never imports plotly.

How it works
------------
``_SUBMODULES`` lists the submodules exposed as attributes (e.g.
``suspension.aero``). ``_FROM`` maps every re-exported symbol to the
``(submodule, original_name)`` that provides it. ``__getattr__`` resolves a
name on first access, imports just the submodule it needs, binds the result
into the package namespace (so the second access is a normal attribute lookup),
and returns it. ``__dir__`` advertises the full surface so tab-completion and
``dir(suspension)`` behave exactly as before.

If you add a new public symbol, add it to ``_FROM`` (or ``_SUBMODULES``) and to
``__all__``. The test ``tests/test_lazy_init`` guards that the two stay in sync
and that ``import suspension`` stays dependency-free.
"""

from importlib import import_module as _import_module

_SUBMODULES = (
    "mythbuster",
    "myth_rules",
    "aero",
    "bolted_joint",
    "bracket_fos",
    "chassis",
    "compliance",
    "correlation",
    "damper",
    "electronics",
    "ev_powertrain",
    "flex",
    "fullcar3d",
    "ggv",
    "harness",
    "integration",
    "interfaces",
    "joints",
    "laptime",
    "loadpath",
    "mountpoints",
    "pack_thermal",
    "pcm_cooling",
    "project",
    "pt_integration",
    "setup",
    "tire_cosim",
    "tire_cosim_driver",
    "tire_cosim_ftire_example",
    "tire_thermal",
    "tirefit",
    "tiremodel",
    "throttle_return",
    "throttle_return_ingest",
    "throttle_dynamics",
    "throttle_flutter_cosim",
    "topologies",
    "topology",
    "tractive_system",
    "transient",
)

_FROM = {
    "check_myth": ("mythbuster", "check"),
    "MythEngine": ("mythbuster", "MythEngine"),
    "MythResult": ("mythbuster", "MythResult"),
    "MythRule": ("mythbuster", "Rule"),
    "MythVerdict": ("mythbuster", "Verdict"),
    "parse_claim": ("mythbuster", "parse_claim"),
    "myth_disciplines": ("mythbuster", "disciplines"),
    "myth_reference_list": ("mythbuster", "reference_myths"),
    "AeroMap": ("aero", "AeroMap"),
    "AeroOrchestrator": ("aero", "AeroOrchestrator"),
    "AeroProvider": ("aero", "AeroProvider"),
    "AeroQuery": ("aero", "AeroQuery"),
    "Aggressor": ("electronics", "Aggressor"),
    "AirflowParams": ("pack_thermal", "AirflowParams"),
    "ArchitectureComparison": ("ev_powertrain", "ArchitectureComparison"),
    "AssumptionResult": ("pt_integration", "AssumptionResult"),
    "Attitude": ("aero", "Attitude"),
    "AxleRoll": ("topology", "AxleRoll"),
    "BOLT_GRADES": ("bolted_joint", "BOLT_GRADES"),
    "BSPD": ("tractive_system", "BSPD"),
    "BoardCheckResult": ("electronics", "BoardCheckResult"),
    "BoardLedger": ("electronics", "BoardLedger"),
    "Body": ("topology", "Body"),
    "BoltGrade": ("bolted_joint", "BoltGrade"),
    "CDTireModel": ("tire_cosim", "CDTireModel"),
    "CFDProvenance": ("aero", "CFDProvenance"),
    "CFDSolver": ("aero", "CFDSolver"),
    "CaseSpec": ("aero", "CaseSpec"),
    "CellParams": ("pack_thermal", "CellParams"),
    "ClampedStack": ("bolted_joint", "ClampedStack"),
    "CoeffResult": ("aero", "CoeffResult"),
    "Coincident": ("topology", "Coincident"),
    "CombinedSlipTire": ("tiremodel", "CombinedSlipTire"),
    "CompliantCorner": ("compliance", "CompliantCorner"),
    "CompliantResult": ("compliance", "CompliantResult"),
    "CondensedFlexBody": ("flex", "CondensedFlexBody"),
    "Connector": ("harness", "Connector"),
    "Constraint": ("topology", "Constraint"),
    "CoolingOperatingPoint": ("pt_integration", "CoolingOperatingPoint"),
    "CornerLoads": ("dynamics", "CornerLoads"),
    "CornerState": ("kinematics", "CornerState"),
    "CosimCornerSet": ("tire_cosim_driver", "CosimCornerSet"),
    "CosimTireHistory": ("tire_cosim_driver", "CosimTireHistory"),
    "DiffPair": ("electronics", "DiffPair"),
    "DriveZ": ("topology", "DriveZ"),
    "DriverInput": ("transient", "DriverInput"),
    "EVLapSimulator": ("ev_powertrain", "EVLapSimulator"),
    "EVParams": ("ev_powertrain", "EVParams"),
    "EVRunResult": ("ev_powertrain", "EVRunResult"),
    "FSAE_TRACTIVE_POWER_CAP_KW": ("pt_integration", "FSAE_TRACTIVE_POWER_CAP_KW"),
    "FTireModel": ("tire_cosim", "FTireModel"),
    "Fan": ("pack_thermal", "Fan"),
    "FanCurve": ("pt_integration", "FanCurve"),
    "FanPlacementCandidate": ("pack_thermal", "FanPlacementCandidate"),
    "FanPlacementStudy": ("pack_thermal", "FanPlacementStudy"),
    "Fastener": ("bolted_joint", "Fastener"),
    "FlexElement": ("flex", "FlexElement"),
    "FlexMesh": ("flex", "FlexMesh"),
    "FluentSolver": ("aero", "FluentSolver"),
    "Formboard": ("harness", "Formboard"),
    "FormboardBranch": ("harness", "FormboardBranch"),
    "GGVGenerator": ("ggv", "GGVGenerator"),
    "GGVParams": ("ggv", "GGVParams"),
    "GGVResult": ("ggv", "GGVResult"),
    "GearCandidate": ("pt_integration", "GearCandidate"),
    "GearObjective": ("pt_integration", "GearObjective"),
    "GearRatioSolver": ("pt_integration", "GearRatioSolver"),
    "GearSweepResult": ("pt_integration", "GearSweepResult"),
    "GenericKinematics": ("adapter", "GenericKinematics"),
    "GeometryLedger": ("mountpoints", "GeometryLedger"),
    "Hardpoints": ("kinematics", "Hardpoints"),
    "HarnessCheckResult": ("harness", "HarnessCheckResult"),
    "HarnessLedger": ("harness", "HarnessLedger"),
    "InPlane": ("topology", "InPlane"),
    "JointCompliance": ("joints", "JointCompliance"),
    "JointResult": ("bolted_joint", "JointResult"),
    "KeepOut": ("mountpoints", "KeepOut"),
    "Link": ("topology", "Link"),
    "LocalSubmitter": ("aero", "LocalSubmitter"),
    "MATERIALS": ("flex", "MATERIALS"),
    "MEMBERS": ("loadpath", "MEMBERS"),
    "METRIC_COARSE": ("bolted_joint", "METRIC_COARSE"),
    "Material": ("flex", "Material"),
    "Mechanism": ("topology", "Mechanism"),
    "MechanismBuilder": ("topology", "MechanismBuilder"),
    "MemberForces": ("loadpath", "MemberForces"),
    "MemberStiffness": ("compliance", "MemberStiffness"),
    "MeshParams": ("aero", "MeshParams"),
    "MotorEnvelope": ("pt_integration", "MotorEnvelope"),
    "MountPoint": ("mountpoints", "MountPoint"),
    "MythCheck": ("pt_integration", "MythCheck"),
    "OnLine": ("topology", "OnLine"),
    "OpenFOAMSolver": ("aero", "OpenFOAMSolver"),
    "OrchestratorReport": ("aero", "OrchestratorReport"),
    "PCMAllocation": ("pcm_cooling", "PCMAllocation"),
    "PCMMaterial": ("pcm_cooling", "PCMMaterial"),
    "PCMResult": ("pcm_cooling", "PCMResult"),
    "PacejkaLateral": ("tiremodel", "PacejkaLateral"),
    "PackLayout": ("pack_thermal", "PackLayout"),
    "PackThermalModel": ("pack_thermal", "PackThermalModel"),
    "PackThermalResult": ("pack_thermal", "PackThermalResult"),
    "Point": ("topology", "Point"),
    "Powertrain": ("ev_powertrain", "Powertrain"),
    "PrechargeCircuit": ("tractive_system", "PrechargeCircuit"),
    "PrechargeTrace": ("tractive_system", "PrechargeTrace"),
    "PropagationResult": ("mountpoints", "PropagationResult"),
    "REQUIRED_SHUTDOWN_NODES": ("tractive_system", "REQUIRED_SHUTDOWN_NODES"),
    "RackTranslation": ("topology", "RackTranslation"),
    "ReferenceAeroModel": ("aero", "ReferenceAeroModel"),
    "ReferenceTireModel": ("tire_cosim", "ReferenceTireModel"),
    "Revolute": ("topology", "Revolute"),
    "RoadInput": ("transient", "RoadInput"),
    "Rules": ("tractive_system", "Rules"),
    "RunMatrix": ("aero", "RunMatrix"),
    "SPAL_VA14_AP11_C34A": ("pt_integration", "SPAL_VA14_AP11_C34A"),
    "SettlingResult": ("transient", "SettlingResult"),
    "ShutdownChain": ("tractive_system", "ShutdownChain"),
    "ShutdownNode": ("tractive_system", "ShutdownNode"),
    "SlurmSSHSubmitter": ("aero", "SlurmSSHSubmitter"),
    "SnappyMesher": ("aero", "SnappyMesher"),
    "SolverFidelity": ("aero", "SolverFidelity"),
    "SolverUnavailable": ("aero", "SolverUnavailable"),
    "SprocketDesign": ("pt_integration", "SprocketDesign"),
    "StarCCMSolver": ("aero", "StarCCMSolver"),
    "StructuralTireModel": ("tire_cosim", "StructuralTireModel"),
    "SubmitResult": ("aero", "SubmitResult"),
    "SuspensionKinematics": ("kinematics", "SuspensionKinematics"),
    "TEMPLATES": ("topologies", "TEMPLATES"),
    "TSAL": ("tractive_system", "TSAL"),
    "ThermalParams": ("tire_thermal", "ThermalParams"),
    "ThermalRun": ("tire_thermal", "ThermalRun"),
    "ThermalTireModel": ("tire_thermal", "ThermalTireModel"),
    "TireFidelity": ("tire_cosim", "TireFidelity"),
    "TireOutput": ("tire_cosim", "TireOutput"),
    "TireProvenance": ("tire_cosim", "TireProvenance"),
    "Trace": ("electronics", "Trace"),
    "TractiveSafetyResult": ("tractive_system", "TractiveSafetyResult"),
    "TransientParams": ("transient", "TransientParams"),
    "TransientResult": ("transient", "TransientResult"),
    "TransientSolver": ("transient", "TransientSolver"),
    "VehicleDynamics": ("dynamics", "VehicleDynamics"),
    "VehicleParams": ("dynamics", "VehicleParams"),
    "WheelLoad": ("loadpath", "WheelLoad"),
    "WheelState": ("tire_cosim", "WheelState"),
    "WireRun": ("harness", "WireRun"),
    "analyze_joint": ("bolted_joint", "analyze_joint"),
    "attitude_from_dynamics": ("aero", "attitude_from_dynamics"),
    "awg_area_mm2": ("harness", "awg_area_mm2"),
    "awg_nominal_od_mm": ("harness", "awg_nominal_od_mm"),
    "axial_stiffness_tube": ("flex", "axial_stiffness_tube"),
    "brake_to_throttle_maneuver": ("transient", "brake_to_throttle_maneuver"),
    "build_full_car_figure": ("fullcar3d", "build_full_car_figure"),
    "check_assumption": ("pt_integration", "check_assumption"),
    "check_board": ("electronics", "check_board"),
    "check_bspd": ("tractive_system", "check_bspd"),
    "check_harness": ("harness", "check_harness"),
    "check_pcm": ("pcm_cooling", "check_pcm"),
    "check_precharge": ("tractive_system", "check_precharge"),
    "check_shutdown_chain": ("tractive_system", "check_shutdown_chain"),
    "check_tractive_system": ("tractive_system", "check_tractive_system"),
    "check_tsal": ("tractive_system", "check_tsal"),
    "cooling_operating_point": ("pt_integration", "cooling_operating_point"),
    "corner_wheel_load": ("compliance", "corner_wheel_load"),
    "curb_strike_maneuver": ("transient", "curb_strike_maneuver"),
    "default_cell_params": ("pack_thermal", "default_cell_params"),
    "default_combined_tire": ("tiremodel", "default_combined_tire"),
    "default_pcm": ("pcm_cooling", "default_pcm"),
    "default_structural_tire": ("tire_cosim", "default_structural_tire"),
    "default_thermal_params": ("tire_thermal", "default_thermal_params"),
    "default_tire": ("tiremodel", "default_tire"),
    "dfmea_rows_from_analysis": ("pt_integration", "dfmea_rows_from_analysis"),
    "double_wishbone": ("topologies", "double_wishbone"),
    "driveline_peak_torque_nm": ("pt_integration", "driveline_peak_torque_nm"),
    "estimate_attitude": ("aero", "estimate_attitude"),
    "estimate_motor_heat_w": ("pt_integration", "estimate_motor_heat_w"),
    "evaluate_pcm_buffer": ("pcm_cooling", "evaluate_pcm_buffer"),
    "example": ("topologies", "example"),
    "fan_grid_candidates": ("pack_thermal", "fan_grid_candidates"),
    "from_links": ("topologies", "from_links"),
    "get_aero_backend": ("aero", "get_backend"),
    "guyan_condense": ("flex", "guyan_condense"),
    "influence_summary": ("fullcar3d", "influence_summary"),
    "joint_findings": ("bolted_joint", "joint_findings"),
    "list_templates": ("topologies", "list_templates"),
    "load_flex_body": ("flex", "load_flex_body"),
    "macpherson_strut": ("topologies", "macpherson_strut"),
    "make_tire_backend": ("tire_cosim", "make_tire_backend"),
    "min_parallel_distance_mm": ("electronics", "min_parallel_distance_mm"),
    "motor_envelope": ("pt_integration", "motor_envelope"),
    "multilink": ("topologies", "multilink"),
    "optimize_fan_placement": ("pack_thermal", "optimize_fan_placement"),
    "pack_current_trace": ("pack_thermal", "pack_current_trace"),
    "parallel_run_length_mm": ("electronics", "parallel_run_length_mm"),
    "parse_checkmesh": ("aero", "parse_checkmesh"),
    "power_rpm_myth_checks": ("pt_integration", "power_rpm_myth_checks"),
    "powertrain_spec_sheet": ("pt_integration", "powertrain_spec_sheet"),
    "propagate_mount_move": ("mountpoints", "propagate_mount_move"),
    "quick_ggv": ("ggv", "quick_ggv"),
    "read_mnf": ("flex", "read_mnf"),
    "relaxation_length": ("tiremodel", "relaxation_length"),
    "run_cosim_maneuver": ("tire_cosim_driver", "run_cosim_maneuver"),
    "run_maneuver": ("transient", "run_maneuver"),
    "semi_trailing_arm": ("topologies", "semi_trailing_arm"),
    "simulate_pack_thermal": ("pack_thermal", "simulate_pack_thermal"),
    "simulate_precharge": ("tractive_system", "simulate_precharge"),
    "simulate_warmup": ("tire_thermal", "simulate_warmup"),
    "size_pcm_for_hold": ("pcm_cooling", "size_pcm_for_hold"),
    "snap_oversteer_maneuver": ("transient", "snap_oversteer_maneuver"),
    "solid_axle": ("topologies", "solid_axle"),
    "solid_rod_section": ("flex", "solid_rod_section"),
    "solve_member_forces": ("loadpath", "solve_member_forces"),
    "sprocket_design": ("pt_integration", "sprocket_design"),
    "step_steer_maneuver": ("transient", "step_steer_maneuver"),
    "sweep_parameter": ("ggv", "sweep_parameter"),
    "system_k_from_point": ("pt_integration", "system_k_from_point"),
    "ReturnSpring": ("throttle_return", "ReturnSpring"),
    "ReturnResistance": ("throttle_return", "ReturnResistance"),
    "ReturnRedundancyResult": ("throttle_return", "ReturnRedundancyResult"),
    "ReturnCaseResult": ("throttle_return", "ReturnCaseResult"),
    "check_return_redundancy": ("throttle_return", "check_return_redundancy"),
    "return_redundancy_report": ("throttle_return", "return_redundancy_report"),
    "check_brake_pedal_2000N": ("throttle_return", "check_brake_pedal_2000N"),
    "k_from_deflection": ("throttle_return", "k_from_deflection"),
    "k_from_two_points": ("throttle_return", "k_from_two_points"),
    "k_theta_from_torque": ("throttle_return", "k_theta_from_torque"),
    "k_compression_spring": ("throttle_return", "k_compression_spring"),
    "WIRE_SHEAR_MODULUS_PA": ("throttle_return", "WIRE_SHEAR_MODULUS_PA"),
    "BRAKE_PEDAL_RULE_LOAD_N": ("throttle_return", "BRAKE_PEDAL_RULE_LOAD_N"),
    "spring_rate_from_bench_log": ("throttle_return_ingest", "spring_rate_from_bench_log"),
    "crosscheck_pedal_against_cad": ("throttle_return_ingest", "crosscheck_pedal_against_cad"),
    "BenchFit": ("throttle_return_ingest", "BenchFit"),
    "CadCrossCheck": ("throttle_return_ingest", "CadCrossCheck"),
    "ThrottleInertia": ("throttle_return", "ThrottleInertia"),
    "SnapResult": ("throttle_return", "SnapResult"),
    "SnapModel": ("throttle_return", "SnapModel"),
    "estimate_throttle_inertia": ("throttle_return", "estimate_throttle_inertia"),
    "simulate_return_snap": ("throttle_return", "simulate_return_snap"),
    "simulate_return_snap_single_failures": ("throttle_return", "simulate_return_snap_single_failures"),
    "ManifoldParams": ("throttle_dynamics", "ManifoldParams"),
    "FlutterParams": ("throttle_dynamics", "FlutterParams"),
    "CoupledResult": ("throttle_dynamics", "CoupledResult"),
    "FlutterResult": ("throttle_dynamics", "FlutterResult"),
    "simulate_coupled_return": ("throttle_dynamics", "simulate_coupled_return"),
    "screen_plate_flutter": ("throttle_dynamics", "screen_plate_flutter"),
    "compressible_mass_flow": ("throttle_dynamics", "compressible_mass_flow"),
    "throttle_flow_area": ("throttle_dynamics", "throttle_flow_area"),
    "OscillationCase": ("throttle_flutter_cosim", "OscillationCase"),
    "FlutterDerivative": ("throttle_flutter_cosim", "FlutterDerivative"),
    "FlutterProvenance": ("throttle_flutter_cosim", "FlutterProvenance"),
    "FlutterFidelity": ("throttle_flutter_cosim", "FlutterFidelity"),
    "FlutterSolver": ("throttle_flutter_cosim", "FlutterSolver"),
    "QuasiSteadyFlutterModel": ("throttle_flutter_cosim", "QuasiSteadyFlutterModel"),
    "ExternalCFDFlutterBackend": ("throttle_flutter_cosim", "ExternalCFDFlutterBackend"),
    "extract_flutter_derivative": ("throttle_flutter_cosim", "extract_flutter_derivative"),
    "trailing_arm": ("topologies", "trailing_arm"),
    "transient_vs_qss_corner": ("transient", "transient_vs_qss_corner"),
    "truck_steer_linkage": ("topologies", "truck_steer_linkage"),
    "tube_section": ("flex", "tube_section"),
    "twist_beam": ("topologies", "twist_beam"),
    "undeclared_loads": ("electronics", "undeclared_loads"),
    "wheel_load_from_corner": ("loadpath", "wheel_load_from_corner"),
    "worst_case_currents": ("electronics", "worst_case_currents"),
}


# --------------------------------------------------------------------------- #
#  Lazy attribute resolution (PEP 562). See the module docstring above.        #
# --------------------------------------------------------------------------- #
# Submodules accessible as attributes. _SUBMODULES are the ones the original
# __init__ exposed via `from . import X`. We also expose every submodule that
# provides a re-exported symbol (e.g. `kinematics`, `dynamics`, `adapter`):
# CPython binds those as package attributes as a side effect of
# `from .X import ...`, so accessing `suspension.kinematics` worked before this
# refactor and must keep working.
_ATTR_SUBMODULES = frozenset(_SUBMODULES) | {mod for (mod, _) in _FROM.values()}


def __getattr__(name):
    # 1) A submodule exposed directly (e.g. `suspension.aero`, `suspension.kinematics`).
    if name in _ATTR_SUBMODULES:
        mod = _import_module(f"{__name__}.{name}")
        globals()[name] = mod          # cache so future access is a plain lookup
        return mod
    # 2) A symbol re-exported from a submodule (e.g. `SuspensionKinematics`).
    src = _FROM.get(name)
    if src is not None:
        submod_name, attr = src
        submod = _import_module(f"{__name__}.{submod_name}")
        try:
            value = getattr(submod, attr)
        except AttributeError as exc:   # pragma: no cover - guards refactors
            raise ImportError(
                f"suspension.{submod_name} no longer provides '{attr}', "
                f"which suspension.__init__ re-exports as '{name}'. "
                f"Update _FROM in suspension/__init__.py."
            ) from exc
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    # Full public surface for tab-completion / dir(), without importing anything.
    return sorted(set(globals()) | set(_ATTR_SUBMODULES) | set(_FROM) | set(__all__))


__all__ = [
    # cross-discipline myth-buster engine
    "check_myth", "MythEngine", "MythResult", "MythRule", "MythVerdict",
    "parse_claim", "myth_disciplines", "myth_reference_list", "mythbuster",
    "SuspensionKinematics", "Hardpoints", "CornerState",
    # architecture-agnostic topology engine
    "topology", "topologies", "GenericKinematics",
    "Point", "Body", "Constraint", "Link", "Coincident", "OnLine", "InPlane",
    "Revolute", "DriveZ", "RackTranslation", "AxleRoll", "Mechanism",
    "MechanismBuilder",
    "double_wishbone", "macpherson_strut", "multilink", "trailing_arm",
    "semi_trailing_arm", "solid_axle", "twist_beam", "truck_steer_linkage",
    "from_links", "TEMPLATES", "list_templates", "example",
    "VehicleDynamics", "VehicleParams", "CornerLoads",
    "PacejkaLateral", "default_tire", "CombinedSlipTire",
    "default_combined_tire", "relaxation_length",
    "chassis", "integration", "project", "tiremodel", "tirefit", "setup",
    "laptime", "correlation", "damper", "interfaces",
    # flexible-body extension
    "Material", "MATERIALS", "tube_section", "solid_rod_section",
    "axial_stiffness_tube", "FlexElement", "FlexMesh", "guyan_condense",
    "CondensedFlexBody", "load_flex_body", "read_mnf",
    "WheelLoad", "MemberForces", "solve_member_forces",
    "wheel_load_from_corner", "MEMBERS",
    "MemberStiffness", "CompliantResult", "CompliantCorner", "corner_wheel_load",
    "JointCompliance",
    "flex", "loadpath", "compliance", "joints",
    # transient time-step DAE solver
    "TransientSolver", "TransientParams", "TransientResult", "SettlingResult",
    "DriverInput", "RoadInput",
    "step_steer_maneuver", "snap_oversteer_maneuver", "brake_to_throttle_maneuver",
    "curb_strike_maneuver", "run_maneuver", "transient_vs_qss_corner",
    "transient",
    # EV powertrain & energy layer (architecture comparison in seconds + kWh)
    "Powertrain", "EVParams", "EVLapSimulator",
    "EVRunResult", "ArchitectureComparison", "ev_powertrain",
    # transient per-cell battery-pack thermal model (hot-cell map + fan placement)
    "CellParams", "default_cell_params", "PackLayout",
    "Fan", "AirflowParams", "PackThermalModel", "PackThermalResult",
    "pack_current_trace", "simulate_pack_thermal",
    "FanPlacementCandidate", "FanPlacementStudy",
    "optimize_fan_placement", "fan_grid_candidates", "pack_thermal",
    # structural tire co-simulation boundary (FTire / CDTire seam)
    "StructuralTireModel", "ReferenceTireModel", "FTireModel", "CDTireModel",
    "WheelState", "TireOutput", "TireProvenance", "TireFidelity",
    "make_tire_backend", "default_structural_tire",
    "CosimCornerSet", "CosimTireHistory", "run_cosim_maneuver",
    "tire_cosim", "tire_cosim_driver", "tire_cosim_ftire_example",
    # lumped-parameter tyre thermal channel (tread/carcass/gas energy balance)
    "ThermalTireModel", "ThermalParams", "ThermalRun",
    "default_thermal_params", "simulate_warmup", "tire_thermal",
    # aerodynamic CFD co-simulation boundary (OpenFOAM / STAR-CCM+ / Fluent seam)
    "Attitude", "RunMatrix", "CaseSpec", "CoeffResult", "CFDProvenance",
    "SolverFidelity", "CFDSolver", "SolverUnavailable",
    "ReferenceAeroModel", "OpenFOAMSolver", "StarCCMSolver", "FluentSolver",
    "get_aero_backend",
    "LocalSubmitter", "SlurmSSHSubmitter", "SubmitResult",
    "AeroMap", "AeroQuery", "AeroOrchestrator", "OrchestratorReport",
    "AeroProvider", "estimate_attitude", "attitude_from_dynamics",
    "MeshParams", "SnappyMesher", "parse_checkmesh", "aero",
    # geometric mount-point clash + CG propagation (CAD -> clash -> CG chain)
    "MountPoint", "KeepOut", "GeometryLedger",
    "PropagationResult", "propagate_mount_move",
    "mountpoints",
    # electronics / PCB layer (copper survival + signal integrity)
    "Trace", "DiffPair", "Aggressor", "BoardLedger", "BoardCheckResult",
    "check_board", "worst_case_currents", "undeclared_loads",
    # harness / 3-D loom (route, bend, clearance, formboard, BOM, copper mass)
    "Connector", "WireRun", "HarnessLedger", "HarnessCheckResult",
    "Formboard", "FormboardBranch", "check_harness",
    "awg_area_mm2", "awg_nominal_od_mm", "harness",
    "min_parallel_distance_mm", "parallel_run_length_mm",
    "electronics",
    # throttle return-spring redundancy + brake-pedal 2000 N gate (brakes/pedal box)
    "ReturnSpring", "ReturnResistance", "ReturnRedundancyResult", "ReturnCaseResult",
    "check_return_redundancy", "return_redundancy_report", "check_brake_pedal_2000N",
    "k_from_deflection", "k_from_two_points", "k_theta_from_torque",
    "k_compression_spring", "WIRE_SHEAR_MODULUS_PA", "BRAKE_PEDAL_RULE_LOAD_N",
    "spring_rate_from_bench_log", "crosscheck_pedal_against_cad",
    "BenchFit", "CadCrossCheck", "throttle_return_ingest",
    "ThrottleInertia", "SnapResult", "SnapModel", "estimate_throttle_inertia",
    "simulate_return_snap", "simulate_return_snap_single_failures",
    "ManifoldParams", "FlutterParams", "CoupledResult", "FlutterResult",
    "simulate_coupled_return", "screen_plate_flutter",
    "compressible_mass_flow", "throttle_flow_area", "throttle_dynamics",
    "OscillationCase", "FlutterDerivative", "FlutterProvenance", "FlutterFidelity",
    "FlutterSolver", "QuasiSteadyFlutterModel", "ExternalCFDFlutterBackend",
    "extract_flutter_derivative", "throttle_flutter_cosim",
    "throttle_return",
]
__version__ = "0.21.0"
