# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Two-dimensional aerofoil sections in and out of ground effect.

Why this exists: the vortex lattice is a thin-surface method. It sees a wing's
camber line, not its thickness, and in ground effect the thickness matters: the
suction surface and the road form a nozzle. Against Zerihan & Zhang's wind-tunnel
wing (benchmarks/aero_ground_effect) the lattice read 14-31 % low with endplates.

Two inviscid 2D solvers on the same section and the same road:

* ``thick_cl`` - the Hess-Smith panel method (constant-strength source panels plus
  one uniform vortex sheet, Kutta condition on the trailing-edge panels; Katz &
  Plotkin, Low-Speed Aerodynamics, 2nd ed., sec. 11.2). The road is a mirror
  image of the section with the vortex sense reversed, so z = 0 is a streamline.
* ``thin_cl`` - the lumped-vortex method on the camber line (Katz & Plotkin
  sec. 11.1.1: vortex at the panel quarter chord, collocation at three quarters),
  with the same image.

``thickness_factor`` is their ratio at a ride height. Multiplying a thin-surface
3D result by it is a strip correction for section thickness (it assumes the 3D
loss of lift is the same for the thin and the thick section). Checked against the
exact Joukowski aerofoil (benchmarks/aero_section). Inviscid: no boundary layer,
no separation.

Units: lengths in chord or metres consistently, angles in radians, CL dimensionless.
"""
from __future__ import annotations

import math

import numpy as np

__all__ = ["thick_cl", "thin_cl", "thickness_factor", "camber_line", "joukowski_section"]


def _panels(x, z):
    """Panel end points, mid points, lengths, unit tangents and normals (outward for a clockwise loop)."""
    xa, za, xb, zb = x[:-1], z[:-1], x[1:], z[1:]
    L = np.hypot(xb - xa, zb - za)
    tx, tz = (xb - xa) / L, (zb - za) / L
    return xa, za, xb, zb, 0.5 * (xa + xb), 0.5 * (za + zb), L, tx, tz, -tz, tx


def _induced(px, pz, xa, za, xb, zb, own: bool = False):
    """Velocity (u, w) at points P from unit-strength constant source and vortex panels A->B.
    Returns (us, ws, uv, wv), each (len(P), len(A))."""
    L = np.hypot(xb - xa, zb - za)
    tx, tz = (xb - xa) / L, (zb - za) / L
    dx, dz = px[:, None] - xa[None, :], pz[:, None] - za[None, :]
    xl = dx * tx[None, :] + dz * tz[None, :]            # panel-local coordinates
    zl = -dx * tz[None, :] + dz * tx[None, :]
    # a panel's own midpoint must read zl = +0, so arctan2 puts it on the outside (theta2 - theta1 = +pi): a
    # rounding error of either sign flips the self-induced term. Set exactly, not by a tolerance (a tolerance
    # relative to the panel failed once the section sat far from the origin).
    if own:
        i = np.arange(len(xa))
        zl[i, i] = 0.0
        xl[i, i] = 0.5 * L
    zl = np.where(np.abs(zl) < 1e-12 * L[None, :], 0.0, zl)
    r1 = np.hypot(xl, zl)
    r2 = np.hypot(xl - L[None, :], zl)
    th1 = np.arctan2(zl, xl)
    th2 = np.arctan2(zl, xl - L[None, :])
    with np.errstate(divide="ignore", invalid="ignore"):
        lg = np.log(np.where(r1 > 0, r1, 1e-300) / np.where(r2 > 0, r2, 1e-300))
    dth = th2 - th1
    us_l, ws_l = lg / (2 * math.pi), dth / (2 * math.pi)  # source, local axes
    uv_l, wv_l = dth / (2 * math.pi), -lg / (2 * math.pi)  # vortex (clockwise positive), local axes
    rot = lambda u, w: (u * tx[None, :] - w * tz[None, :], u * tz[None, :] + w * tx[None, :])
    us, ws = rot(us_l, ws_l)
    uv, wv = rot(uv_l, wv_l)
    return us, ws, uv, wv


def thick_cl(x, z, alpha: float = 0.0, h: float | None = None) -> float:
    """Lift coefficient (on the chord, positive up) of a closed section by the Hess-Smith panel method.

    x, z: section coordinates as one closed loop, trailing edge -> lower surface -> leading edge -> upper
    surface -> trailing edge (clockwise). alpha: onset angle, radians (nose up positive). h: when given, a road
    at z = 0 (the section must lie above it)."""
    x, z = np.asarray(x, float), np.asarray(z, float)
    xa, za, xb, zb, xm, zm, L, tx, tz, nx, nz = _panels(x, z)
    n = len(xm)
    U, W = math.cos(alpha), math.sin(alpha)
    us, ws, uv, wv = _induced(xm, zm, xa, za, xb, zb, own=True)
    if h is not None:
        # image: mirrored geometry with reversed panel direction; same source, opposite vortex
        Ius, Iws, Iuv, Iwv = _induced(xm, zm, xb, -zb, xa, -za)
        us, ws = us + Ius, ws + Iws
        uv, wv = uv - Iuv, wv - Iwv
    A = np.zeros((n + 1, n + 1))
    b = np.zeros(n + 1)
    A[:n, :n] = us * nx[:, None] + ws * nz[:, None]
    A[:n, n] = (uv * nx[:, None] + wv * nz[:, None]).sum(axis=1)
    b[:n] = -(U * nx + W * nz)
    # Kutta: equal and opposite tangential velocity on the first and last panels
    for k in (0, n - 1):
        A[n, :n] += us[k] * tx[k] + ws[k] * tz[k]
        A[n, n] += (uv[k] * tx[k] + wv[k] * tz[k]).sum()
        b[n] -= U * tx[k] + W * tz[k]
    sol = np.linalg.solve(A, b)
    gamma = sol[n]
    chord = float(x.max() - x.min())
    # circulation of the uniform sheet over the perimeter; lift = rho V Gamma (clockwise positive gives lift)
    return 2.0 * gamma * float(L.sum()) / chord


def camber_line(x, z, n: int = 120):
    """Camber line (x, z) of a closed section given as above: the mean of the two surfaces at cosine-spaced x.
    Same length units as the input (chord units or m)."""
    x, z = np.asarray(x, float), np.asarray(z, float)
    i_le = int(np.argmin(x))
    lo_x, lo_z = x[:i_le + 1][::-1], z[:i_le + 1][::-1]      # LE -> TE along the lower surface
    up_x, up_z = x[i_le:], z[i_le:]                          # LE -> TE along the upper surface
    x0, x1 = float(x[i_le]), float(x.max())
    xs = x0 + (x1 - x0) * 0.5 * (1 - np.cos(np.linspace(0, math.pi, n + 1)))
    zl = np.interp(xs, lo_x, lo_z)
    zu = np.interp(xs, up_x, up_z)
    return xs, 0.5 * (zl + zu)


def thin_cl(xc, zc, alpha: float = 0.0, h: float | None = None) -> float:
    """Lift coefficient (dimensionless) of a camber line by the lumped-vortex method (positive up), road at
    z = 0 when h given. Coordinates and h in chord units or m; alpha in rad."""
    xc, zc = np.asarray(xc, float), np.asarray(zc, float)
    dx, dz = np.diff(xc), np.diff(zc)
    L = np.hypot(dx, dz)
    vx, vz = xc[:-1] + 0.25 * dx, zc[:-1] + 0.25 * dz        # vortices
    cx, cz = xc[:-1] + 0.75 * dx, zc[:-1] + 0.75 * dz        # collocation
    nx, nz = -dz / L, dx / L

    def vel(px, pz, qx, qz, sign=1.0):
        """Velocity at points P from clockwise point vortices at Q, per unit circulation: 1/m when lengths are in m."""
        rx, rz = px[:, None] - qx[None, :], pz[:, None] - qz[None, :]
        r2 = rx * rx + rz * rz
        # clockwise unit vortex: u = rz / (2 pi r2), w = -rx / (2 pi r2)
        return sign * rz / (2 * math.pi * r2), sign * -rx / (2 * math.pi * r2)
    u, w = vel(cx, cz, vx, vz)
    if h is not None:
        iu, iw = vel(cx, cz, vx, -vz, -1.0)
        u, w = u + iu, w + iw
    A = u * nx[:, None] + w * nz[:, None]
    G = np.linalg.solve(A, -(math.cos(alpha) * nx + math.sin(alpha) * nz))
    chord = float(xc.max() - xc.min())
    return 2.0 * float(G.sum()) / chord


def thickness_factor(x, z, h_lowest: float | None, alpha: float = 0.0) -> float:
    """CL of the thick section over CL of its camber line, both inviscid, with the section's lowest point h_lowest
    above the road (None: free air). Same units as x, z; dimensionless result."""
    x, z = np.asarray(x, float), np.asarray(z, float)
    if h_lowest is None:
        xs, zs, hh = x, z, None
    else:
        dz = h_lowest - float(z.min())
        xs, zs, hh = x, z + dz, h_lowest
    xc, zc = camber_line(xs, zs)
    return thick_cl(xs, zs, alpha, hh) / thin_cl(xc, zc, alpha, hh)


def joukowski_section(eps: float = 0.1, mu: float = 0.08, n: int = 200):
    """A Joukowski aerofoil and its exact lift slope. Circle centre (-eps, mu), radius through (1, 0); map
    zeta -> zeta + 1/zeta. Returns (x, z) clockwise from the trailing edge, the zero-lift angle (rad) and the exact
    CL at alpha: CL(alpha) = 2 pi R sin(alpha + beta) * 4 / chord ... returned as a function."""
    xc0, zc0 = -eps, mu
    R = math.hypot(1 - xc0, zc0)
    beta = math.asin(zc0 / R)
    th = np.linspace(0, 2 * math.pi, n + 1)
    zeta = (xc0 + 1j * zc0) + R * np.exp(-1j * (th + beta))    # clockwise from the trailing edge cusp at zeta = 1
    w = zeta + 1.0 / zeta
    x, z = w.real, w.imag
    chord = float(x.max() - x.min())

    def cl_exact(alpha):
        """Exact lift coefficient (dimensionless) at alpha in rad."""
        return 8 * math.pi * R * math.sin(alpha + beta) / chord
    return x, z, cl_exact
