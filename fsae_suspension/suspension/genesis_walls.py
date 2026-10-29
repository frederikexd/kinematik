# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""
genesis_walls.py — vehicle-level walls for InverseGenesis.

The solved-property bounds of inverse_genesis carry what the corner's
kinematics produce. Three facts about the car decide whether a corner that
fits its curves is worth building, and none of them is a curve:

* ``brake_decel_g``   the straight-line deceleration the declared tire allows
                      at the declared front brake bias (tire_utilisation). It
                      does not depend on the hardpoints: bounded, it checks the
                      declared bias before the search moves anything, which is
                      the point of putting it on the same wall.
* ``link_fos_min``    the lowest Euler (pinned) or Perry–Robertson factor of
                      any link under the declared shock cases at the declared
                      sections (robustness.member_margins). It moves with the
                      geometry: a narrow wishbone base multiplies the braking
                      load into the legs.
* ``rocker_bearing_sf`` the lowest static safety factor of the declared rocker
                      bearing pair under the declared cases
                      (actuation.rocker_bearing_loads), on the actuation points
                      the hardpoints carry.

Each spec is plain data so a manifest can carry it and its hash can cover it.
Forces in N, lengths in mm, stresses in MPa, accelerations in g.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from functools import lru_cache
from types import SimpleNamespace

import numpy as np

__all__ = ["BrakeDecel", "LinkStructure", "RockerBearingWall"]

_LINKS = ("UF", "UR", "LF", "LR", "TR")


def _load_dict(ld) -> dict:
    return {"Fx": float(getattr(ld, "Fx", 0.0)), "Fy": float(getattr(ld, "Fy", 0.0)),
            "Fz": float(getattr(ld, "Fz", 0.0)),
            "Mz": float(getattr(ld, "Mz", getattr(ld, "mz_Nmm", 0.0))),
            "Fx_wc": float(getattr(ld, "Fx_wc", 0.0))}


def _wheel_load(d: dict):
    from .loadpath import WheelLoad
    return WheelLoad(Fx=float(d.get("Fx", 0.0)), Fy=float(d.get("Fy", 0.0)),
                     Fz=float(d.get("Fz", 0.0)), Mz=float(d.get("Mz", 0.0)),
                     Fx_wc=float(d.get("Fx_wc", 0.0)))


# --------------------------------------------------------------------------- #
#  Brake bias against the tire
# --------------------------------------------------------------------------- #
@lru_cache(maxsize=256)
def _max_decel_g(bias: float, mass_kg: float, cg_height_mm: float,
                 wheelbase_mm: float, track_mm: float) -> float:
    from .target_derivation import Vehicle
    from .tire_utilisation import tire_state
    veh = Vehicle(mass_kg=mass_kg, cg_height_mm=cg_height_mm,
                  wheelbase_mm=wheelbase_mm, track_mm=track_mm)
    lo, hi = 0.0, 3.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        st = tire_state(mid, 0.0, veh=veh, brake_front=bias)
        if st["feasible"] and st["vehicle_u"] <= 1.0:
            lo = mid
        else:
            hi = mid
    return lo


@dataclass
class BrakeDecel:
    """The car the deceleration is computed for (Table 2 values by default).

    The tire is tire_utilisation's default, the declared synthetic set; the
    deceleration is quasi-steady and straight-line, with the traction ellipse
    of every tire as the limit.
    """
    mass_kg: float = 300.0
    cg_height_mm: float = 280.0
    wheelbase_mm: float = 1630.0
    track_mm: float = 1210.0

    def decel_g(self, brake_bias_front: float) -> float:
        """Maximum straight-line deceleration (g) at this front bias."""
        return _max_decel_g(round(float(brake_bias_front), 6), float(self.mass_kg),
                            float(self.cg_height_mm), float(self.wheelbase_mm),
                            float(self.track_mm))

    def balanced(self, lo: float = 0.50, hi: float = 0.90, step: float = 0.01
                 ) -> tuple[float, float]:
        """(front bias, deceleration g) that brakes hardest, to ``step``."""
        best = (lo, -1.0)
        for b in np.arange(lo, hi + 1e-9, step):
            d = self.decel_g(float(b))
            if d > best[1]:
                best = (round(float(b), 4), d)
        return best

    def to_dict(self) -> dict:
        """Serialise the spec: lengths in mm, loads in N, mass in kg."""
        return {"mass_kg": float(self.mass_kg), "cg_height_mm": float(self.cg_height_mm),
                "wheelbase_mm": float(self.wheelbase_mm), "track_mm": float(self.track_mm)}

    @classmethod
    def from_dict(cls, d: dict) -> BrakeDecel:
        """Rebuild from to_dict: lengths in mm, loads in N, mass in kg."""
        return cls(**{k: float(v) for k, v in d.items()})


# --------------------------------------------------------------------------- #
#  Link strength under the shock cases
# --------------------------------------------------------------------------- #
@dataclass
class LinkStructure:
    """Link sections and the shock cases they must carry.

    ``od_mm``/``wall_mm`` apply to every link not in ``per_member``, which maps
    a member (UF, UR, LF, LR, TR) to [od_mm, wall_mm]. ``cases`` maps a case
    name to a contact-patch load {Fx, Fy, Fz, Mz, Fx_wc} (N, N*mm). The factor
    is the lowest of Euler buckling (pinned ends) and Perry–Robertson yield with
    initial crookedness ``crook`` x L in compression, and of sigma_y A / F in
    tension, over every link and case.
    """
    cases: dict = field(default_factory=dict)
    od_mm: float = 15.875
    wall_mm: float = 0.889
    per_member: dict = field(default_factory=dict)
    sy_mpa: float = 435.0
    e_mpa: float = 205000.0
    crook: float = 1.0 / 1000.0

    def __post_init__(self):
        if not self.cases:
            raise ValueError("LinkStructure needs at least one load case.")
        for k in self.per_member:
            if k not in _LINKS:
                raise ValueError(f"LinkStructure.per_member: unknown member '{k}'.")
        self.per_member = {str(k): [float(v[0]), float(v[1])]
                           for k, v in sorted(self.per_member.items())}
        self.cases = {str(k): _load_dict(SimpleNamespace(**v)) if isinstance(v, dict)
                      else _load_dict(v) for k, v in self.cases.items()}

    @classmethod
    def from_shock_cases(cls, axle: str, brake_front: float, inboard_drive: bool = True, inboard_brakes: bool = True,
                         **kw) -> LinkStructure:
        """The declared shock cases of robustness.shock_cases at this bias; loads in N, sections in mm. The rear
        drive and rear brakes default to inboard (their force at the wheel centre)."""
        from .robustness import shock_cases
        return cls(cases={k: _load_dict(v) for k, v in
                          shock_cases(axle, brake_front=brake_front, inboard_drive=inboard_drive,
                                      inboard_brakes=inboard_brakes).items()}, **kw)

    def margins(self, hp) -> dict:
        """Per link: worst force (N), its case, buckling and yield factors."""
        from .robustness import member_margins
        loads = {k: _wheel_load(v) for k, v in self.cases.items()}
        groups: dict = {}
        for m in _LINKS:
            groups.setdefault(tuple(self.per_member.get(m, (self.od_mm, self.wall_mm))), []).append(m)
        out = {}
        for (od, wall), members in groups.items():
            r = member_margins(hp, loads, od_mm=od, wall_mm=wall, e_mpa=self.e_mpa,
                               sy_mpa=self.sy_mpa, crook=self.crook)
            for m in members:
                if m in r:
                    out[m] = dict(r[m], od_mm=od, wall_mm=wall)
        return out

    def fos_min(self, hp) -> float:
        """Lowest buckling or yield factor of any link (dimensionless)."""
        mm = self.margins(hp)
        if not mm:
            return math.nan
        return float(min(min(v["fos_buckling"], v["fos_yield"]) for v in mm.values()))

    def to_dict(self) -> dict:
        """Serialise the spec: lengths in mm, loads in N, mass in kg."""
        d = {"cases": {k: dict(v) for k, v in sorted(self.cases.items())},
             "od_mm": float(self.od_mm), "wall_mm": float(self.wall_mm),
             "sy_mpa": float(self.sy_mpa), "e_mpa": float(self.e_mpa),
             "crook": float(self.crook)}
        if self.per_member:
            d["per_member"] = {k: list(v) for k, v in self.per_member.items()}
        return d

    @classmethod
    def from_dict(cls, d: dict) -> LinkStructure:
        """Rebuild from to_dict: lengths in mm, loads in N, mass in kg."""
        return cls(cases=dict(d["cases"]), od_mm=float(d.get("od_mm", 15.875)),
                   wall_mm=float(d.get("wall_mm", 0.889)),
                   per_member=dict(d.get("per_member", {})),
                   sy_mpa=float(d.get("sy_mpa", 435.0)), e_mpa=float(d.get("e_mpa", 205000.0)),
                   crook=float(d.get("crook", 1.0 / 1000.0)))


# --------------------------------------------------------------------------- #
#  Rocker bearings
# --------------------------------------------------------------------------- #
@dataclass
class RockerBearingWall:
    """The rocker bearing pair and the cases it must carry.

    C0 is the static rating of ONE bearing (N), ``spacing_mm`` the distance
    between the pair along the rocker axis; X0, Y0 the static equivalent-load
    factors. ``cases`` is a list of {name, Fx, Fy, Fz, Mz, Fx_wc}.
    """
    cases: list = field(default_factory=list)
    label: str = "2 x 6001-2RS (declared)"
    C0_N: float = 2360.0
    spacing_mm: float = 30.0
    X0: float = 0.6
    Y0: float = 0.5

    def __post_init__(self):
        if not self.cases:
            raise ValueError("RockerBearingWall needs at least one load case.")
        self.cases = [dict(_load_dict(SimpleNamespace(**c)) if isinstance(c, dict) else _load_dict(c),
                           name=str(c["name"] if isinstance(c, dict) else getattr(c, "name", "case")))
                      for c in self.cases]

    @classmethod
    def from_vehicle_cases(cls, axle: str, brake_bias_front: float, **kw) -> RockerBearingWall:
        """The five (or seven) declared vehicle load cases of genesis_repro; loads in N, spacing in mm."""
        from .genesis_repro import vehicle_load_cases
        return cls(cases=list(vehicle_load_cases(axle=axle, brake_bias_front=brake_bias_front)), **kw)

    def rows(self, hp) -> list[dict]:
        """Per-case bearing loads (N) and static safety factor (dimensionless)."""
        from .actuation import RockerBearing, rocker_bearing_loads
        b = RockerBearing(label=self.label, C0_N=self.C0_N, spacing_mm=self.spacing_mm,
                          X0=self.X0, Y0=self.Y0)
        cs = [SimpleNamespace(name=c["name"], Fx=c["Fx"], Fy=c["Fy"], Fz=c["Fz"],
                              mz_Nmm=c["Mz"], Fx_wc=c["Fx_wc"]) for c in self.cases]
        return rocker_bearing_loads(hp, cs, b)

    def s0_min(self, hp) -> float:
        """Lowest static safety factor of the pair over the cases."""
        try:
            return float(min(r["s0"] for r in self.rows(hp)))
        except ValueError:
            return math.nan

    def to_dict(self) -> dict:
        """Serialise the spec: lengths in mm, loads in N, mass in kg."""
        return {"cases": [dict(c) for c in self.cases], "label": self.label,
                "C0_N": float(self.C0_N), "spacing_mm": float(self.spacing_mm),
                "X0": float(self.X0), "Y0": float(self.Y0)}

    @classmethod
    def from_dict(cls, d: dict) -> RockerBearingWall:
        """Rebuild from to_dict: lengths in mm, loads in N, mass in kg."""
        return cls(cases=list(d["cases"]), label=str(d.get("label", "")),
                   C0_N=float(d["C0_N"]), spacing_mm=float(d.get("spacing_mm", 30.0)),
                   X0=float(d.get("X0", 0.6)), Y0=float(d.get("Y0", 0.5)))
