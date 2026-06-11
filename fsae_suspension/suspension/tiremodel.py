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
