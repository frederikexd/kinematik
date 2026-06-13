"""
Vehicle-level dynamics built on top of the corner kinematics.

This module turns single-corner geometry into the numbers that decide how the
car actually behaves: lateral load transfer split front/rear, individual tire
vertical loads in a steady-state corner, roll-centre heights and migration, and
a simple load-sensitive grip model so you can see understeer/oversteer balance
shift as you change geometry. This is the part spreadsheets do badly and the
reason a coupled kinematics+dynamics tool is worth open-sourcing.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass

from .kinematics import SuspensionKinematics, Hardpoints
from .tiremodel import PacejkaLateral


@dataclass
class VehicleParams:
    # NOTE on provenance: the tyre constants below (tire_load_sens, mu_peak) are
    # representative FSAE PLACEHOLDERS, not measured values. They make the balance
    # model useful for COMPARING setups (which way does the balance move?), not for
    # predicting absolute grip. Replace with values from your own tyre data (e.g. the
    # FSAE Tire Test Consortium) before trusting absolute numbers.
    mass: float = 280.0            # total mass incl. driver, kg
    cg_height: float = 300.0       # mm above ground
    wheelbase: float = 1550.0      # mm
    track_front: float = 1200.0    # mm
    track_rear: float = 1180.0     # mm
    weight_dist_front: float = 0.47  # fraction of mass on front axle
    roll_stiffness_front: float = 350.0  # N·m/deg
    roll_stiffness_rear: float = 300.0   # N·m/deg
    tire_load_sens: float = 0.00018  # 1/N: mu drop per N of vertical load (LINEAR fallback only)
    mu_peak: float = 1.55          # peak friction at light load (LINEAR fallback only)
    # Static camber per axle (deg, negative = top inboard). Fed to the Pacejka model
    # so the grip number reflects the camber the loaded outside tire actually runs.
    # When a front/rear kinematics object is supplied, the SOLVED camber is used
    # instead of these — these are the fallback when geometry isn't attached.
    static_camber_front: float = -1.5
    static_camber_rear: float = -1.5
    # When True, the grip model uses static_camber_* directly instead of the solved
    # kinematic camber. The setup-sensitivity/optimiser sets this so it can explore
    # camber as a free design lever; the live geometry tabs leave it False so the
    # camber the tires see comes from the actual linkage.
    use_param_camber: bool = False
    g: float = 9.81


@dataclass
class CornerLoads:
    fl: float
    fr: float
    rl: float
    rr: float

    def as_tuple(self):
        return (self.fl, self.fr, self.rl, self.rr)


class VehicleDynamics:
    def __init__(self, params: VehicleParams,
                 front_kin: SuspensionKinematics | None = None,
                 rear_kin: SuspensionKinematics | None = None,
                 tire: PacejkaLateral | None = None):
        """
        params : vehicle mass/geometry/stiffness.
        front_kin / rear_kin : corner kinematics. When present, the SOLVED camber at
            each axle is fed to the tire model, so geometry changes show up in grip.
        tire : a Pacejka lateral model. When supplied, grip and balance run on the
            real Magic Formula (load-sensitive, camber-aware). When None, the model
            falls back to the legacy linear placeholder (mu = mu_peak - load_sens*Fz)
            so nothing breaks if no tire is loaded — but absolute grip is then only
            indicative. Pass tiremodel.default_tire() for sensible generic behaviour,
            or your TTC-fitted model for numbers you can trust on YOUR tire.
        """
        self.p = params
        self.front_kin = front_kin
        self.rear_kin = rear_kin
        self.tire = tire

    # ---------------- camber actually seen by each axle ------------------ #
    def _axle_camber_rad(self, axle: str) -> float:
        """
        Inclination angle (rad, magnitude) the loaded tire on `axle` runs at static
        ride height. Priority:
          1. an explicit camber override on params (use_param_camber=True) — this is
             what the setup-sweep/optimiser uses to vary camber as a free design
             lever even when geometry is attached;
          2. otherwise the SOLVED kinematics camber when geometry is attached, so
             real geometry changes flow through to grip;
          3. otherwise the static_camber_* param.
        The Pacejka model treats camber magnitude as grip-relevant, so we pass |cam|.
        """
        kin = self.front_kin if axle == "front" else self.rear_kin
        param_cam = (self.p.static_camber_front if axle == "front"
                     else self.p.static_camber_rear)
        if getattr(self.p, "use_param_camber", False):
            cam_deg = param_cam
        elif kin is not None:
            cam_deg = kin.static.camber
        else:
            cam_deg = param_cam
        return abs(np.radians(cam_deg))

    # ---------------- roll centres from kinematics ----------------------- #
    def roll_center_height(self, kin: SuspensionKinematics, track: float,
                           state=None) -> float:
        """
        Front-view roll-centre height: intersection of the line from contact patch
        through the instant centre with the car centreline (y = 0).

        By default uses the static pose, but pass `state` (any CornerState) to
        evaluate at a given travel — roll centres migrate substantially through
        travel/roll, which is a primary reason teams care about RC, so the model
        can now report that migration instead of assuming RC is fixed.
        """
        st = state if state is not None else kin.static
        ic = st.instant_center          # (y, z)
        cp_y = st.contact_patch[1]
        cp_z = st.contact_patch[2]
        if not np.all(np.isfinite(ic)):
            return np.nan
        dy = ic[0] - cp_y
        dz = ic[1] - cp_z
        if abs(dy) < 1e-9:
            return cp_z
        slope = dz / dy
        rc_z = cp_z + slope * (0.0 - cp_y)
        return float(rc_z)

    def roll_center_migration(self, kin: SuspensionKinematics, track: float,
                              travel_min=-30.0, travel_max=30.0, n=21):
        """RC height across a travel sweep → (travels, rc_heights), for an honest
        picture of how the roll centre moves rather than a single static number."""
        travels = np.linspace(travel_min, travel_max, n)
        rc = [self.roll_center_height(kin, track, state=kin.solve_at_travel(t))
              for t in travels]
        return list(travels), rc

    # ---------------- steady-state lateral load transfer ----------------- #
    def lateral_load_transfer(self, lateral_g: float):
        p = self.p
        W = p.mass * p.g
        Wf = W * p.weight_dist_front
        Wr = W * (1 - p.weight_dist_front)

        rc_f = (self.roll_center_height(self.front_kin, p.track_front)
                if self.front_kin else 50.0)
        rc_r = (self.roll_center_height(self.rear_kin, p.track_rear)
                if self.rear_kin else 60.0)
        if not np.isfinite(rc_f):
            rc_f = 50.0
        if not np.isfinite(rc_r):
            rc_r = 60.0

        a_lat = lateral_g * p.g
        # Sprung CG roll moment about the roll axis
        roll_axis_at_cg = rc_f + (rc_r - rc_f) * p.weight_dist_front
        h_roll = p.cg_height - roll_axis_at_cg          # roll moment arm, mm
        M_roll = p.mass * a_lat * (h_roll / 1000.0)     # N·m

        k_tot = p.roll_stiffness_front + p.roll_stiffness_rear
        # roll stiffness is specified in N·m/deg, so M/k is already in degrees
        roll_angle = (M_roll / k_tot) if k_tot > 0 else 0.0

        # Elastic (sprung) transfer split by roll stiffness
        dWf_elastic = (p.roll_stiffness_front / k_tot) * M_roll / (p.track_front / 1000.0)
        dWr_elastic = (p.roll_stiffness_rear / k_tot) * M_roll / (p.track_rear / 1000.0)

        # Geometric (unsprung/RC) transfer reacts instantly through the linkage
        dWf_geo = (p.mass * p.weight_dist_front) * a_lat * (rc_f / 1000.0) / (p.track_front / 1000.0)
        dWr_geo = (p.mass * (1 - p.weight_dist_front)) * a_lat * (rc_r / 1000.0) / (p.track_rear / 1000.0)

        dWf = dWf_elastic + dWf_geo
        dWr = dWr_elastic + dWr_geo

        # In a left turn the right (outer) wheels gain load
        fl = Wf / 2 - dWf
        fr = Wf / 2 + dWf
        rl = Wr / 2 - dWr
        rr = Wr / 2 + dWr
        loads = CornerLoads(max(fl, 0), max(fr, 0), max(rl, 0), max(rr, 0))
        return loads, dict(roll_angle=roll_angle, rc_front=rc_f, rc_rear=rc_r,
                           ltd_front=dWf, ltd_rear=dWr)

    # ---------------- load-sensitive grip & balance ---------------------- #
    def _corner_force(self, Fz: float, axle: str) -> float:
        """
        Max lateral force a single tire makes at vertical load Fz on the given axle.
        Uses the Pacejka model (load-sensitive + camber-aware) when a tire is
        attached; otherwise the legacy linear placeholder. This single switch is
        what upgrades the whole grip/balance/max-g stack from a straight line to a
        real measured-tire curve.
        """
        if Fz <= 0:
            return 0.0
        if self.tire is not None:
            gamma = self._axle_camber_rad(axle)
            return self.tire.peak_force(Fz, gamma)
        mu = self.p.mu_peak - self.p.tire_load_sens * Fz
        mu = max(mu, 0.3)
        return mu * Fz

    def axle_grip(self, loads: CornerLoads):
        """Max lateral force each axle can make (sum of its two tires)."""
        front = self._corner_force(loads.fl, "front") + self._corner_force(loads.fr, "front")
        rear = self._corner_force(loads.rl, "rear") + self._corner_force(loads.rr, "rear")
        return front, rear

    def grip_model_name(self) -> str:
        """Which grip model is live — for honest UI labelling."""
        return "Pacejka MF5.2" if self.tire is not None else "linear placeholder"

    def balance_index(self, lateral_g: float):
        """
        > 0 means front-limited (understeer), < 0 means rear-limited (oversteer).
        Computed as the normalised difference in axle grip utilisation.
        """
        loads, _ = self.lateral_load_transfer(lateral_g)
        Ff, Fr = self.axle_grip(loads)
        p = self.p
        Wf = p.mass * p.g * p.weight_dist_front
        Wr = p.mass * p.g * (1 - p.weight_dist_front)
        demand_f = Wf * lateral_g
        demand_r = Wr * lateral_g
        util_f = demand_f / Ff if Ff > 0 else 99
        util_r = demand_r / Fr if Fr > 0 else 99
        return util_f - util_r, util_f, util_r

    def max_lateral_g(self):
        """Bisection for the steady-state lateral g the car can sustain."""
        lo, hi = 0.1, 3.0
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            loads, _ = self.lateral_load_transfer(mid)
            Ff, Fr = self.axle_grip(loads)
            capacity = (Ff + Fr) / (self.p.mass * self.p.g)
            if capacity >= mid:
                lo = mid
            else:
                hi = mid
        return lo
