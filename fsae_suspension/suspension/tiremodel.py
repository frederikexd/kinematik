"""
Pacejka Magic Formula (MF5.2) lateral tire model.

This replaces KinematiK's linear placeholder grip model with the real Magic Formula,
evaluated from coefficients fitted to measured tire data. The EQUATIONS here are the
standard, published MF5.2 pure-lateral formulae (Pacejka, *Tyre and Vehicle Dynamics*)
— they are textbook and safe to open-source. The COEFFICIENTS are tire-specific and,
when they come from TTC data, are confidential: they load from a separate file that is
gitignored and never committed. Ship the code, not the numbers.

Scope: pure lateral force Fy as a function of slip angle, vertical load, and camber.
This is exactly what the grip/balance model needs. Combined slip, longitudinal force,
and aligning moment are out of scope (not needed for steady-state cornering grip).

A `PacejkaLateral` exposes:
    .fy(alpha_rad, Fz_N, gamma_rad=0)  -> lateral force, N
    .mu_peak(Fz_N, gamma_rad=0)        -> peak |Fy|/Fz over slip, the grip coefficient

The second is what VehicleDynamics calls to get load-sensitive grip — so plugging this
in makes the balance/grip numbers reflect the real tire, not a straight-line guess.
"""

from __future__ import annotations

import json
import numpy as np
from dataclasses import dataclass


# Default scaling factors (lambdas) — all 1.0 means "use the fit as-is".
_DEFAULT_SCALING = {
    "LFZO": 1.0, "LCY": 1.0, "LMUY": 1.0, "LEY": 1.0,
    "LKY": 1.0, "LHY": 1.0, "LVY": 1.0, "LGAY": 1.0,
}


@dataclass
class PacejkaLateral:
    """MF5.2 pure-lateral model. Construct from a coefficient dict + nominal load."""
    coeffs: dict
    FNOMIN: float = 6306.0      # nominal vertical load, N (from the tire data)
    scaling: dict = None

    def __post_init__(self):
        self.scaling = {**_DEFAULT_SCALING, **(self.scaling or {})}
        missing = [k for k in ("PCY1", "PDY1", "PDY2", "PEY1", "PKY1", "PKY2")
                   if k not in self.coeffs]
        if missing:
            raise ValueError(f"Missing required lateral coefficients: {missing}")

    def _C(self, name, default=0.0):
        return float(self.coeffs.get(name, default))

    def fy(self, alpha, Fz, gamma=0.0):
        """
        Pure lateral force Fy (N) for slip angle alpha (rad), vertical load Fz (N),
        camber gamma (rad). Standard MF5.2 pure-slip lateral equations.
        """
        Fz = np.maximum(np.asarray(Fz, float), 1e-6)
        s = self.scaling
        dfz = (Fz - self.FNOMIN) / self.FNOMIN          # normalised load increment
        g = gamma                                       # camber, rad

        # Shape factor
        Cy = self._C("PCY1") * s["LCY"]
        # Peak factor (friction)
        mu_y = (self._C("PDY1") + self._C("PDY2") * dfz) \
            * (1.0 - self._C("PDY3") * g * g) * s["LMUY"]
        Dy = mu_y * Fz
        # Curvature
        Ey = (self._C("PEY1") + self._C("PEY2") * dfz) \
            * (1.0 - (self._C("PEY3") + self._C("PEY4") * g) * np.sign(alpha)) * s["LEY"]
        Ey = np.minimum(Ey, 1.0)
        # Cornering stiffness -> B
        Ky = self._C("PKY1") * self.FNOMIN \
            * np.sin(2.0 * np.arctan(Fz / (self._C("PKY2") * self.FNOMIN * s["LFZO"]))) \
            * (1.0 - self._C("PKY3") * abs(g)) * s["LKY"]
        By = Ky / (Cy * Dy + 1e-9)
        # Horizontal/vertical shifts
        Shy = (self._C("PHY1") + self._C("PHY2") * dfz) * s["LHY"] + self._C("PHY3") * g
        Svy = Fz * ((self._C("PVY1") + self._C("PVY2") * dfz) * s["LVY"]
                    + (self._C("PVY3") + self._C("PVY4") * dfz) * g) * s["LGAY"]

        ax = alpha + Shy
        Fy = Dy * np.sin(Cy * np.arctan(By * ax - Ey * (By * ax - np.arctan(By * ax)))) + Svy
        return Fy

    def mu_peak(self, Fz, gamma=0.0):
        """
        Peak friction coefficient |Fy|/Fz at a given load, found by sweeping slip
        angle. This is the single number VehicleDynamics needs for grip — and unlike
        the linear placeholder, it carries the real nonlinear load sensitivity.
        """
        alphas = np.radians(np.linspace(-15, 15, 121))
        fy = np.array([self.fy(a, Fz, gamma) for a in alphas])
        return float(np.max(np.abs(fy)) / max(Fz, 1e-6))

    def peak_force(self, Fz, gamma=0.0):
        """Peak |Fy| in newtons at a given load/camber (mu_peak * Fz)."""
        return self.mu_peak(Fz, gamma) * max(float(Fz), 0.0)

    def alpha_peak(self, Fz, gamma=0.0):
        """
        Slip angle (deg) at which lateral force peaks for this load/camber. This is
        the target operating slip — knowing it tells the driver/aero team how much
        steer the front needs at the limit, and feeds combined-slip headroom later.
        """
        alphas = np.radians(np.linspace(0.0, 15.0, 151))
        fy = np.abs(np.array([self.fy(a, Fz, gamma) for a in alphas]))
        return float(np.degrees(alphas[int(np.argmax(fy))]))

    def optimal_camber(self, Fz, cam_min_deg=-6.0, cam_max_deg=0.5, n=40):
        """
        Camber (deg, in tire frame where negative leans the top inboard) that
        maximises peak lateral grip at a given load. This is *free* grip for an
        underfunded team: it's set by geometry, not budget. Returns (best_camber_deg,
        mu_at_best). The dynamics layer uses the per-corner kinematic camber, but
        this answers "what should we target?" directly from the tire.

        Convention note: this returns the inclination angle magnitude that helps.
        A racing setup runs negative static camber; here we sweep the |IA| that the
        loaded outside tire actually sees and report the best as a negative number.
        """
        cambers = np.linspace(cam_min_deg, cam_max_deg, n)
        mus = [self.mu_peak(Fz, np.radians(abs(c))) for c in cambers]
        i = int(np.argmax(mus))
        return float(cambers[i]), float(mus[i])


# --------------------------------------------------------------------------- #
#  Loading coefficients (kept OUT of the public repo)
# --------------------------------------------------------------------------- #
def load_from_json(path: str) -> PacejkaLateral:
    """Load a coefficient JSON {coeffs:{...}, FNOMIN:..} — your private tire file."""
    with open(path) as f:
        d = json.load(f)
    return PacejkaLateral(coeffs=d["coeffs"], FNOMIN=d.get("FNOMIN", 6306.0),
                          scaling=d.get("scaling"))


def coeffs_to_json(coeffs: dict, FNOMIN: float, path: str):
    """Write a private coefficient file (gitignored). Never commit the result."""
    with open(path, "w") as f:
        json.dump({"coeffs": coeffs, "FNOMIN": FNOMIN}, f, indent=2)


# --------------------------------------------------------------------------- #
#  Generic default tire (NOT TTC-derived — safe to ship)
# --------------------------------------------------------------------------- #
# These coefficients are a representative, hand-tuned MF5.2 lateral set for a
# generic ~13"/10" FSAE tire at low load. They are NOT fitted to any confidential
# TTC data and are safe to commit. They give physically sensible behaviour:
#   - peak mu ~1.55 near nominal load, ~1.69 light / ~1.37 heavy (load sensitivity)
#   - a camber optimum around 2-3 deg of inclination, then falloff
# Use them so the grip/balance engine runs on a real Magic Formula from day one.
# Replace with YOUR fitted tire (process_ttc.py -> JSON) the moment you have data;
# the absolute grip numbers only become trustworthy once they're your tire's.
_DEFAULT_FSAE_COEFFS = {
    "PCY1": 1.45, "PDY1": 1.55, "PDY2": -0.22, "PDY3": 1.2,
    "PEY1": -0.6, "PEY2": -0.1, "PEY3": 0.1, "PEY4": 2.0,
    "PKY1": -28.0, "PKY2": 1.6, "PKY3": 0.6,
    "PHY1": 0.0, "PHY2": 0.0, "PHY3": 0.0,
    "PVY1": 0.0, "PVY2": 0.0, "PVY3": 0.12, "PVY4": 0.0,
}
_DEFAULT_FSAE_FNOMIN = 1100.0      # N, representative FSAE corner load


def default_tire() -> PacejkaLateral:
    """
    A generic FSAE Pacejka lateral model with sensible behaviour, safe to ship.
    This is what the tool uses until you load your own fitted tire. It is good for
    RELATIVE comparisons (which setup change helps?) out of the box; absolute grip
    only becomes trustworthy once you swap in your TTC-fitted coefficients.
    """
    return PacejkaLateral(coeffs=dict(_DEFAULT_FSAE_COEFFS),
                          FNOMIN=_DEFAULT_FSAE_FNOMIN)


def describe(tire: PacejkaLateral) -> dict:
    """Quick human-readable summary of a tire model's grip envelope."""
    return {
        "FNOMIN_N": tire.FNOMIN,
        "mu_at_nominal": round(tire.mu_peak(tire.FNOMIN), 3),
        "mu_light_load": round(tire.mu_peak(0.4 * tire.FNOMIN), 3),
        "mu_heavy_load": round(tire.mu_peak(1.8 * tire.FNOMIN), 3),
        "alpha_peak_deg": round(tire.alpha_peak(tire.FNOMIN), 2),
        "optimal_camber_deg": round(tire.optimal_camber(tire.FNOMIN)[0], 2),
    }
