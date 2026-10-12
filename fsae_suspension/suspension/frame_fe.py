# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""Linear 3D beam model of a tube frame, built from its STEP tubes, for frame torsion and pickup stiffness.

WHAT IT DOES
------------
* ``Frame``: nodes (mm) and two-node Euler-Bernoulli space-frame elements (axial, torsion, two bending planes;
  12 DOF each) with circular tube sections. Joints are rigid. Units: mm, N, N/mm, N*mm, MPa.
* ``from_step_axes``: the frame of a STEP file's straight tube axes (genesis_analysis.parse_step_tubes). Tube ends
  within ``merge_mm`` share a node; an end that lands on another tube's span (a T-joint, within ``tee_mm`` of its
  axis) is joined to a node inserted on that tube by a short link of the landing tube's section; an end left
  over is joined to the nearest other tube end within ``bend_mm`` by a chord of its own section (a bend: the chord
  is stiffer than the arc it replaces, so the model can only over-predict stiffness). Every tube is subdivided to
  at most ``max_el_mm``.
* ``solve``: K u = f with fixed DOF; ``condensed_compliance``: the flexibility matrix at chosen node DOF (by unit
  loads; symmetric by Maxwell-Betti, which the tests check).
* ``torsional_rate``: frame torsion, one axle's attachment nodes held, the other axle's twisted by a couple
  (N*m/deg, from the rotation of the loaded axle line).
* ``attach``: adds a pickup node joined by tubes to existing frame nodes (a node bay of the kind the ChassisV11
  brief requires), so the stiffness at that pickup can be derived rather than declared.

WHAT IT DOES NOT DO
-------------------
No joint flexibility, tab or gusset compliance, tube-wall ovalisation, panels or bonded floors; the bend chords are
stiff. All of these make a real frame softer, so every stiffness here is an upper bound on the built frame's.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

E_STEEL = 205000.0          # MPa
G_STEEL = 79500.0           # MPa


@dataclass
class Section:
    od_mm: float
    wall_mm: float

    @property
    def A(self) -> float:
        """Cross-section area (mm^2) of the tube."""
        ri = self.od_mm / 2 - self.wall_mm
        return np.pi * ((self.od_mm / 2) ** 2 - ri ** 2)

    @property
    def I(self) -> float:
        """Second moment of area (mm^4) of the tube about a diameter."""
        ri = self.od_mm / 2 - self.wall_mm
        return np.pi / 4 * ((self.od_mm / 2) ** 4 - ri ** 4)

    @property
    def J(self) -> float:
        """Polar moment (mm^4) of the thin round tube, 2 I."""
        return 2.0 * self.I


@dataclass
class Frame:
    nodes: list = field(default_factory=list)          # [np.array(3)]
    elems: list = field(default_factory=list)          # [(i, j, Section, tag)]
    E: float = E_STEEL
    G: float = G_STEEL

    # ------------------------------------------------------------------ build
    def add_node(self, p) -> int:
        """Append a node at ``p`` (mm, CAD axes); returns its index (dimensionless)."""
        self.nodes.append(np.asarray(p, float)); return len(self.nodes) - 1

    def add_elem(self, i: int, j: int, sec: Section, tag: str = "", max_el_mm: float | None = None) -> None:
        a, b = self.nodes[i], self.nodes[j]
        L = float(np.linalg.norm(b - a))
        if L < 1e-9:
            raise ValueError("zero-length element")
        n = 1 if not max_el_mm else max(1, int(np.ceil(L / max_el_mm)))
        prev = i
        for k in range(1, n):
            q = self.add_node(a + (b - a) * k / n)
            self.elems.append((prev, q, sec, tag)); prev = q
        self.elems.append((prev, j, sec, tag))

    def nearest(self, p, among=None) -> int:
        """Index of the node nearest ``p`` (mm), optionally among ``among``; distances in mm."""
        idx = range(len(self.nodes)) if among is None else among
        return min(idx, key=lambda k: float(np.linalg.norm(self.nodes[k] - np.asarray(p, float))))

    # ------------------------------------------------------------------ element matrices
    def _k_local(self, L: float, s: Section) -> np.ndarray:
        E, G, A, I, J = self.E, self.G, s.A, s.I, s.J
        k = np.zeros((12, 12))
        k[0, 0] = k[6, 6] = E * A / L; k[0, 6] = k[6, 0] = -E * A / L
        k[3, 3] = k[9, 9] = G * J / L; k[3, 9] = k[9, 3] = -G * J / L
        for (v, th, sgn) in ((1, 5, 1.0), (2, 4, -1.0)):            # bending in local xy (v, rz) and xz (w, ry)
            a, b, c, d = 12 * E * I / L ** 3, 6 * E * I / L ** 2, 4 * E * I / L, 2 * E * I / L
            vi, ti, vj, tj = v, th, v + 6, th + 6
            k[vi, vi] = k[vj, vj] = a; k[vi, vj] = k[vj, vi] = -a
            k[vi, ti] = k[ti, vi] = sgn * b; k[vi, tj] = k[tj, vi] = sgn * b
            k[vj, ti] = k[ti, vj] = -sgn * b; k[vj, tj] = k[tj, vj] = -sgn * b
            k[ti, ti] = k[tj, tj] = c; k[ti, tj] = k[tj, ti] = d
        return k

    def _T(self, a, b) -> np.ndarray:
        ex = (b - a) / np.linalg.norm(b - a)
        ref = np.array([0.0, 0.0, 1.0]) if abs(ex[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        ey = np.cross(ref, ex); ey /= np.linalg.norm(ey); ez = np.cross(ex, ey)
        R = np.vstack([ex, ey, ez])
        T = np.zeros((12, 12))
        for q in range(4):
            T[3 * q:3 * q + 3, 3 * q:3 * q + 3] = R
        return T

    def stiffness(self) -> np.ndarray:
        """Assembled global stiffness matrix: 6 DOF per node, N/mm and N*mm/rad terms."""
        n = 6 * len(self.nodes)
        K = np.zeros((n, n))
        for i, j, s, _ in self.elems:
            a, b = self.nodes[i], self.nodes[j]
            L = float(np.linalg.norm(b - a)); T = self._T(a, b)
            kg = T.T @ self._k_local(L, s) @ T
            dofs = np.r_[6 * i:6 * i + 6, 6 * j:6 * j + 6]
            K[np.ix_(dofs, dofs)] += kg
        return K

    # ------------------------------------------------------------------ solve
    def solve(self, loads: dict, fixed: dict) -> np.ndarray:
        """``loads``: {node: 6-vector (N, N*mm)}; ``fixed``: {node: iterable of DOF 0..5}. Returns u (n, 6)."""
        K = self.stiffness(); n = K.shape[0]
        f = np.zeros(n)
        for nd, v in loads.items():
            f[6 * nd:6 * nd + 6] += np.asarray(v, float)
        fx = sorted({6 * nd + d for nd, ds in fixed.items() for d in ds})
        free = np.setdiff1d(np.arange(n), fx)
        u = np.zeros(n)
        u[free] = np.linalg.solve(K[np.ix_(free, free)], f[free])
        return u.reshape(-1, 6)

    def condensed_compliance(self, points: list, fixed: dict) -> np.ndarray:
        """Flexibility (mm/N) at the translational DOF of ``points``: C[3a+i, 3b+j] = u_a,i under a unit force b,j."""
        K = self.stiffness(); n = K.shape[0]
        fx = sorted({6 * nd + d for nd, ds in fixed.items() for d in ds})
        free = np.setdiff1d(np.arange(n), fx)
        pos = {d: k for k, d in enumerate(free)}
        cols = [6 * p + d for p in points for d in range(3)]
        F = np.zeros((len(free), len(cols)))
        for c, d in enumerate(cols):
            F[pos[d], c] = 1.0
        U = np.linalg.solve(K[np.ix_(free, free)], F)
        return np.array([[U[pos[r], c] for c in range(len(cols))] for r in cols])

    # ------------------------------------------------------------------ free-free (inertia relief)
    def rigid_modes(self, x0=None) -> np.ndarray:
        """The six rigid-body modes (n*6 x 6): three translations, three rotations about ``x0`` (mm)."""
        x0 = np.zeros(3) if x0 is None else np.asarray(x0, float)
        R = np.zeros((6 * len(self.nodes), 6))
        for i, p in enumerate(self.nodes):
            for k in range(3):
                e = np.eye(3)[k]
                R[6 * i + k, k] = 1.0
                R[6 * i:6 * i + 3, 3 + k] = np.cross(e, p - x0)
                R[6 * i + 3 + k, 3 + k] = 1.0
        return R

    def lumped_mass(self, extra_kg: float = 0.0, cg=None, density: float = 7.85e-6) -> np.ndarray:
        """Translational node masses (kg): the tubes' own (``density`` kg/mm^3) plus ``extra_kg`` spread over the nodes
        in proportion to tube length, tilted exponentially along CAD Z and Y so that the total centre of mass lies at
        ``cg`` (mm). Returns m per node."""
        m = np.zeros(len(self.nodes)); w = np.zeros(len(self.nodes))
        for i, j, s, _ in self.elems:
            L = float(np.linalg.norm(self.nodes[j] - self.nodes[i]))
            m[i] += 0.5 * L * s.A * density; m[j] += 0.5 * L * s.A * density
            w[i] += 0.5 * L; w[j] += 0.5 * L
        if extra_kg <= 0.0:
            return m
        X = np.array(self.nodes)
        def total(ab):
            """Node masses (kg) with ``extra_kg`` spread by the exponential weights ``ab`` (1/m)."""
            e = w * np.exp(ab[0] * X[:, 2] / 1000.0 + ab[1] * X[:, 1] / 1000.0); e = e / e.sum() * extra_kg
            return m + e
        if cg is None:
            return total((0.0, 0.0))
        from scipy.optimize import least_squares
        cg = np.asarray(cg, float)
        def res(ab):
            """CG error (mm) of the weighted masses against the target, rearward and vertical."""
            mm = total(ab); c = (mm[:, None] * X).sum(0) / mm.sum()
            return [c[2] - cg[2], c[1] - cg[1]]
        ab = least_squares(res, [0.0, 0.0]).x
        mm = total(ab); c = (mm[:, None] * X).sum(0) / mm.sum()
        if np.linalg.norm(c[1:] - cg[1:]) > 1.0:
            raise ValueError(f"lumped_mass: centre of mass {c} cannot reach {cg}")
        return mm

    def inertia_relief_flexibility(self, points: list, mass: np.ndarray) -> np.ndarray:
        """Flexibility (mm/N) at the translational DOF of ``points`` with the frame free: every load is balanced by the
        inertia of ``mass`` (kg per node) and the displacement is measured from the frame's mass-weighted rigid
        motion (the body's own reference). Symmetric by construction (P^T K^+ P)."""
        n = 6 * len(self.nodes)
        K = self.stiffness()
        Md = np.zeros(n)
        for i, mi in enumerate(mass):
            Md[6 * i:6 * i + 3] = mi
        R = self.rigid_modes(np.average(np.array(self.nodes), axis=0, weights=mass))
        A = R.T @ (Md[:, None] * R)
        Ainv = np.linalg.inv(A)
        cols = [6 * p + d for p in points for d in range(3)]
        F = np.zeros((n, len(cols)))
        for c, d in enumerate(cols):
            F[d, c] = 1.0
        Fb = F - (Md[:, None] * R) @ (Ainv @ (R.T @ F))            # inertia relief: zero resultant force and moment
        # three nodes far apart pin the six rigid modes (3-2-1); the balanced load gives them no reaction
        X = np.array(self.nodes); a = int(np.argmin(X[:, 2])); b = int(np.argmax(X[:, 2]))
        c = int(np.argmax(np.abs(np.cross(X - X[a], X[b] - X[a])).sum(1)))
        fixed = [6 * a + 0, 6 * a + 1, 6 * a + 2, 6 * b + 0, 6 * b + 1, 6 * c + 1]
        free = np.setdiff1d(np.arange(n), fixed)
        U = np.zeros((n, len(cols)))
        U[free] = np.linalg.solve(K[np.ix_(free, free)], Fb[free])
        Ue = U - R @ (Ainv @ (R.T @ (Md[:, None] * U)))            # remove the mass-weighted rigid motion
        return Ue[cols, :]

    def components(self) -> list:
        """Connected groups of node indices (dimensionless) linked by the elements."""
        parent = list(range(len(self.nodes)))
        def fnd(a):
            """Union-find root of node ``a`` (an index, dimensionless)."""
            while parent[a] != a:
                parent[a] = parent[parent[a]]; a = parent[a]
            return a
        for i, j, _, _ in self.elems:
            parent[fnd(i)] = fnd(j)
        groups: dict = {}
        for k in range(len(self.nodes)):
            groups.setdefault(fnd(k), []).append(k)
        return sorted(groups.values(), key=len, reverse=True)

    def length_mm(self) -> float:
        return float(sum(np.linalg.norm(self.nodes[j] - self.nodes[i]) for i, j, _, _ in self.elems))

    # ------------------------------------------------------------------ attachments
    def attach(self, p, to_nodes: list, sec: Section, tag: str = "bay", max_el_mm: float | None = 50.0) -> int:
        """A pickup node at ``p`` joined to each of ``to_nodes`` by a tube of ``sec`` (positions in mm, elements at most ``max_el_mm`` mm); returns the node index."""
        k = self.add_node(p)
        for t in to_nodes:
            self.add_elem(k, t, sec, tag, max_el_mm)
        return k


def _segdist(p, a, b):
    d = b - a; t = float(np.clip((p - a) @ d / (d @ d), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * d))), t


def _collapse_short(fr: Frame, tol_mm: float) -> None:
    """Merge the two nodes of any element shorter than ``tol_mm`` (an end that lands on a tube axis)."""
    par = list(range(len(fr.nodes)))
    def fnd(a):
        """Union-find root of node ``a`` (an index, dimensionless)."""
        while par[a] != a:
            par[a] = par[par[a]]; a = par[a]
        return a
    for i, j, _, _ in fr.elems:
        if np.linalg.norm(fr.nodes[j] - fr.nodes[i]) < tol_mm:
            par[fnd(i)] = fnd(j)
    roots = sorted({fnd(k) for k in range(len(fr.nodes))})
    remap = {r: n for n, r in enumerate(roots)}
    fr.nodes = [fr.nodes[r] for r in roots]
    fr.elems = [(remap[fnd(i)], remap[fnd(j)], s, t) for i, j, s, t in fr.elems if fnd(i) != fnd(j)]


def from_step_axes(axes, od_mm, merge_mm: float = 5.0, tee_mm: float = 35.0, bend_mm: float = 80.0,
                   max_el_mm: float = 50.0) -> tuple[Frame, dict]:
    """Frame from STEP tube axes [(p0, p1, wall)] and outer diameters (mm). Returns (frame, report)."""
    A = [(np.asarray(a, float), np.asarray(b, float), float(w) if w else 1.65, float(od)) for (a, b, w), od in zip(axes, od_mm)]
    ends = [(i, e, (A[i][0] if e == 0 else A[i][1])) for i in range(len(A)) for e in (0, 1)]
    # 1. merge coincident ends
    lab = list(range(len(ends)))
    def fnd(a):
        """Union-find root of label ``a`` (an index, dimensionless)."""
        while lab[a] != a:
            lab[a] = lab[lab[a]]; a = lab[a]
        return a
    for x in range(len(ends)):
        for y in range(x + 1, len(ends)):
            if ends[x][0] != ends[y][0] and np.linalg.norm(ends[x][2] - ends[y][2]) <= merge_mm:
                lab[fnd(x)] = fnd(y)
    groups: dict = {}
    for x in range(len(ends)):
        groups.setdefault(fnd(x), []).append(x)
    fr = Frame(); end_node = {}
    for g in groups.values():
        k = fr.add_node(np.mean([ends[x][2] for x in g], axis=0))
        for x in g:
            end_node[(ends[x][0], ends[x][1])] = k
    lone = [x for g in groups.values() if len(g) == 1 for x in g]
    # 2. classify the ends left alone: T-joint onto another tube's span, else a bend chord to the nearest end
    tees, chords = [], []
    for x in lone:
        i, e, p = ends[x]
        best = min(((_segdist(p, A[j][0], A[j][1]), j) for j in range(len(A)) if j != i), key=lambda q: q[0][0])
        (d, t), j = best
        near = min(((float(np.linalg.norm(p - ends[y][2])), y) for y in range(len(ends)) if ends[y][0] != i), key=lambda q: q[0])
        if d <= tee_mm and 0.02 < t < 0.98:
            tees.append((x, j, t, d))
        elif near[0] <= bend_mm:
            chords.append((x, near[1], near[0]))
    # 3. insert tee nodes on host spans
    host_pts: dict = {}
    for x, j, t, d in tees:
        host_pts.setdefault(j, []).append(t)
    tee_node = {}
    for j, ts in host_pts.items():
        for t in sorted(set(round(t, 4) for t in ts)):
            tee_node[(j, t)] = fr.add_node(A[j][0] + t * (A[j][1] - A[j][0]))
    # 4. tubes, split at their tee nodes
    for j, (a, b, w, od) in enumerate(A):
        sec = Section(od, w)
        chain = [end_node[(j, 0)]] + [tee_node[(j, t)] for t in sorted(set(round(t, 4) for t in host_pts.get(j, [])))] + [end_node[(j, 1)]]
        for p, q in zip(chain[:-1], chain[1:]):
            if p != q and np.linalg.norm(fr.nodes[p] - fr.nodes[q]) > 1e-6:
                fr.add_elem(p, q, sec, f"T{j + 1:02d}", max_el_mm)
    for x, j, t, d in tees:
        i, e, _ = ends[x]
        a, b = end_node[(i, e)], tee_node[(j, round(t, 4))]
        if np.linalg.norm(fr.nodes[a] - fr.nodes[b]) > 1e-6:
            fr.add_elem(a, b, Section(A[i][3], A[i][2]), "tee", None)
    done = set()
    for x, y, d in chords:
        key = tuple(sorted((x, y)))
        if key in done:
            continue
        done.add(key)
        i, e, _ = ends[x]; i2, e2, _ = ends[y]
        a, b = end_node[(i, e)], end_node[(i2, e2)]
        if a != b:
            fr.add_elem(a, b, Section(A[i][3], A[i][2]), "bend", None)
    _collapse_short(fr, 1.0)
    comps = fr.components()
    rep = {"tubes": len(A), "merged_ends": len(ends) - len(lone), "tees": len(tees), "bend_chords": len(done),
           "unjoined_ends": len(lone) - len(tees) - len(chords), "components": [len(c) for c in comps]}
    # keep the largest component only
    if len(comps) > 1:
        keep = set(comps[0]); remap = {}
        nodes = []
        for k in sorted(keep):
            remap[k] = len(nodes); nodes.append(fr.nodes[k])
        dropped = [e for e in fr.elems if e[0] not in keep]
        fr.elems = [(remap[i], remap[j], s, t) for i, j, s, t in fr.elems if i in keep]
        fr.nodes = nodes
        rep["dropped_tubes"] = sorted({t for _, _, _, t in dropped})
    rep["nodes"], rep["elements"], rep["beam_length_m"] = len(fr.nodes), len(fr.elems), fr.length_mm() / 1000
    return fr, rep
