from .kinematics import SuspensionKinematics, Hardpoints, CornerState
from .dynamics import VehicleDynamics, VehicleParams, CornerLoads
from .tiremodel import (PacejkaLateral, default_tire, CombinedSlipTire,
                        default_combined_tire, relaxation_length)
from . import chassis
from . import integration
from . import project
from . import tiremodel
from . import tirefit
from . import setup
from . import laptime
from . import correlation
from . import damper
from . import interfaces

# Flexible-body / compliance (ADAMS Flex-style) extension
from .flex import (
    Material, MATERIALS, tube_section, solid_rod_section,
    axial_stiffness_tube, FlexElement, FlexMesh, guyan_condense,
    CondensedFlexBody, load_flex_body, read_mnf,
)
from .loadpath import (
    WheelLoad, MemberForces, solve_member_forces,
    wheel_load_from_corner, MEMBERS,
)
from .compliance import (
    MemberStiffness, CompliantResult, CompliantCorner, corner_wheel_load,
)
from .joints import JointCompliance
from . import flex
from . import loadpath
from . import compliance
from . import joints

# Explicit high-frequency transient time-step DAE solver (the unsteady half of
# the lap: yaw/sideslip, pitch/dive, kerb strikes, snap-oversteer recovery).
from .transient import (
    TransientSolver, TransientParams, TransientResult, SettlingResult,
    DriverInput, RoadInput,
    step_steer_maneuver, snap_oversteer_maneuver, brake_to_throttle_maneuver,
    curb_strike_maneuver, run_maneuver, transient_vs_qss_corner,
)
from . import transient

# Structural tire co-simulation boundary (the FTire / CDTire integration seam):
# a stateful tyre contract, a Pacejka-backed reference backend that refuses to
# fake structural/thermal channels, vendor adapter stubs, and a staggered co-sim
# driver around the transient solver.
from .tire_cosim import (
    StructuralTireModel, ReferenceTireModel, FTireModel, CDTireModel,
    WheelState, TireOutput, TireProvenance, TireFidelity,
    make_tire_backend, default_structural_tire,
)
from .tire_cosim_driver import (
    CosimCornerSet, CosimTireHistory, run_cosim_maneuver,
)
from . import tire_cosim
from . import tire_cosim_driver
from . import tire_cosim_ftire_example

__all__ = [
    "SuspensionKinematics", "Hardpoints", "CornerState",
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
    # structural tire co-simulation boundary (FTire / CDTire seam)
    "StructuralTireModel", "ReferenceTireModel", "FTireModel", "CDTireModel",
    "WheelState", "TireOutput", "TireProvenance", "TireFidelity",
    "make_tire_backend", "default_structural_tire",
    "CosimCornerSet", "CosimTireHistory", "run_cosim_maneuver",
    "tire_cosim", "tire_cosim_driver", "tire_cosim_ftire_example",
]
__version__ = "0.16.0"
