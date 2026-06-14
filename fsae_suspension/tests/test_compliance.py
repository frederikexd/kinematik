"""
Physics sanity tests for the flexible-body / compliance extension.

These pin the conventions and catch regressions in the new stack: the finite-
element core (flex.py), the member load-path solver (loadpath.py), and the
compliance coupling that re-solves the corner under load (compliance.py).

Like the kinematics suite these aren't a full cross-check against a commercial
FE/MBD solver — that's a great PR — but they nail the closed-form cases the
engine MUST reproduce (a bar's EA/L, a cantilever's 3EI/L^3, a Guyan series
reduction), the equilibrium residuals, and the signs of compliance steer/camber.

Run:  python -m pytest tests/  (or just: python tests/test_compliance.py)
"""
import numpy as np
import sys, os, json, tempfile
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from suspension import (
    SuspensionKinematics, Hardpoints, VehicleDynamics, VehicleParams,
    MATERIALS, tube_section, axial_stiffness_tube,
    FlexElement, FlexMesh, guyan_condense, CondensedFlexBody,
    load_flex_body, read_mnf,
    WheelLoad, solve_member_forces,
    MemberStiffness, CompliantCorner, corner_wheel_load,
)
import suspension as _s


# --------------------------------------------------------------------------- #
#  Finite-element core: closed-form stiffness
# --------------------------------------------------------------------------- #
def test_tube_section_matches_annulus_formula():
    od, wall = 19.05, 0.9
    A, I, J = tube_section(od, wall)
    idia = od - 2 * wall
    A_exp = np.pi / 4.0 * (od**2 - idia**2)
    I_exp = np.pi / 64.0 * (od**4 - idia**4)
    assert abs(A - A_exp) / A_exp < 1e-12
    assert abs(I - I_exp) / I_exp < 1e-12
    assert abs(J - 2 * I_exp) / (2 * I_exp) < 1e-12   # thin tube: J = 2 I


def test_bar_axial_stiffness_is_EA_over_L():
    """A single axial bar between two interface nodes must give k = E A / L."""
    mat = "Steel 4130"
    E = MATERIALS[mat].E
    L = 350.0
    od, wall = 20.0, 1.0
    A, _, _ = tube_section(od, wall)
    nodes = {"a": (0, 0, 0), "b": (L, 0, 0)}
    el = [FlexElement("a", "b", kind="bar", material=mat, od_mm=od, wall_mm=wall)]
    body = FlexMesh(nodes, el, {"a": "a", "b": "b"}).condense()
    k = body.relative_axial_stiffness("a", "b")
    assert abs(k - E * A / L) / (E * A / L) < 1e-9


def test_guyan_series_two_beams_axial():
    """Two beams in series, condensed to the ends, must give E A / L_total.

    (Series validation uses BEAM elements: an axial bar chain leaves the interior
    node's transverse DOFs unconstrained, a deliberately singular case the code
    rejects — see flex.py.)
    """
    mat = "Steel 4130"
    E = MATERIALS[mat].E
    od, wall = 18.0, 1.2
    A, _, _ = tube_section(od, wall)
    L1, L2 = 150.0, 200.0
    nodes = {"a": (0, 0, 0), "m": (L1, 0, 0), "b": (L1 + L2, 0, 0)}
    el = [FlexElement("a", "m", kind="beam", material=mat, od_mm=od, wall_mm=wall),
          FlexElement("m", "b", kind="beam", material=mat, od_mm=od, wall_mm=wall)]
    body = FlexMesh(nodes, el, {"a": "a", "b": "b"}).condense()
    k = body.relative_axial_stiffness("a", "b")
    k_exp = E * A / (L1 + L2)
    assert abs(k - k_exp) / k_exp < 1e-6


def test_cantilever_tip_stiffness_3EI_over_L3():
    """A beam grounded at one end has lateral tip stiffness 3 E I / L^3."""
    mat = "Steel 4130"
    E = MATERIALS[mat].E
    od, wall = 25.0, 2.0
    _, I, _ = tube_section(od, wall)
    L = 300.0
    nodes = {"root": (0, 0, 0), "tip": (L, 0, 0)}
    el = [FlexElement("root", "tip", kind="beam", material=mat, od_mm=od, wall_mm=wall)]
    body = FlexMesh(nodes, el, {"root": "root", "tip": "tip"}).condense()
    # ground the root fully; push the tip in y
    k_tip = body.grounded_stiffness("tip", grounded=["root"], direction=(0, 1, 0))
    k_exp = 3.0 * E * I / L**3
    assert abs(k_tip - k_exp) / k_exp < 1e-3


def test_guyan_condense_raises_on_singular_master():
    """A bar chain with a free interior node -> singular Kss -> clear error."""
    nodes = {"a": (0, 0, 0), "m": (100.0, 0, 0), "b": (200.0, 0, 0)}
    el = [FlexElement("a", "m", kind="bar", od_mm=20, wall_mm=1),
          FlexElement("m", "b", kind="bar", od_mm=20, wall_mm=1)]
    mesh = FlexMesh(nodes, el, {"a": "a", "b": "b"})
    raised = False
    try:
        mesh.condense()
    except (np.linalg.LinAlgError, ValueError):
        raised = True
    assert raised, "expected a singular/ill-conditioned reduction to be rejected"


# --------------------------------------------------------------------------- #
#  Flex-body import / export
# --------------------------------------------------------------------------- #
def test_reduced_schema_roundtrips_verbatim():
    """A pre-reduced superelement (MNF-equivalent) loads and round-trips."""
    mat = "Steel 4130"
    od, wall = 20.0, 1.0
    L = 250.0
    nodes = {"a": (0, 0, 0), "b": (L, 0, 0)}
    el = [FlexElement("a", "b", kind="bar", material=mat, od_mm=od, wall_mm=wall)]
    body = FlexMesh(nodes, el, {"a": "a", "b": "b"}).condense()
    d = body.to_dict()
    assert d["type"] == "reduced"
    body2 = load_flex_body(d)
    k1 = body.relative_axial_stiffness("a", "b")
    k2 = body2.relative_axial_stiffness("a", "b")
    assert abs(k1 - k2) / k1 < 1e-12


def test_binary_mnf_raises_actionable_error():
    """A binary .mnf must raise NotImplementedError, not silently guess."""
    with tempfile.NamedTemporaryFile(suffix=".mnf", delete=False) as fh:
        fh.write(b"\x00\x01\x02MNF\xff\xfe garbage binary header \x00\x00")
        path = fh.name
    try:
        raised = False
        try:
            read_mnf(path)
        except NotImplementedError as exc:
            raised = True
            assert "reduced" in str(exc).lower() or "export" in str(exc).lower()
        assert raised, "binary MNF should raise NotImplementedError"
    finally:
        os.unlink(path)


def test_flex_body_feeds_member_stiffness():
    """A condensed FEA body used as a member's stiffness matches its EA/L."""
    mat = "Steel 4130"
    E = MATERIALS[mat].E
    od, wall = 16.0, 1.0
    A, _, _ = tube_section(od, wall)
    L = 280.0
    nodes = {"out": (0, 0, 0), "in": (L, 0, 0)}
    el = [FlexElement("out", "in", kind="bar", material=mat, od_mm=od, wall_mm=wall)]
    body = FlexMesh(nodes, el, {"out": "out", "in": "in"}).condense()
    ms = MemberStiffness(flex_body=body, node_out="out", node_in="in")
    k = ms.axial_stiffness(L)   # length arg ignored on the FE path
    assert abs(k - E * A / L) / (E * A / L) < 1e-9


# --------------------------------------------------------------------------- #
#  Member load-path solver
# --------------------------------------------------------------------------- #
def test_member_force_equilibrium_residual_small():
    kin = SuspensionKinematics(Hardpoints.default())
    state = kin.static
    load = WheelLoad(Fx=0.0, Fy=-2800.0, Fz=2000.0, Mz=0.0)
    mf = solve_member_forces(kin, state, load)
    assert mf.residual < 1e-6, f"equilibrium residual too large: {mf.residual}"


def test_pure_vertical_load_reacts_vertically():
    """A pure vertical patch load: member axial forces on the upright sum to -Fz z."""
    kin = SuspensionKinematics(Hardpoints.default())
    state = kin.static
    Fz = 1000.0
    mf = solve_member_forces(kin, state, WheelLoad(Fx=0, Fy=0, Fz=Fz, Mz=0))
    total = np.zeros(3)
    for m, T in mf.forces.items():
        total += T * mf.axes[m]          # force the member applies to the upright
    # the links must react the patch vertical load (equal/opposite through upright)
    assert abs(total[0]) < 1e-5 * Fz
    assert abs(total[1]) < 1e-5 * Fz
    assert abs(abs(total[2]) - Fz) < 1e-4 * Fz


# --------------------------------------------------------------------------- #
#  Compliance coupling
# --------------------------------------------------------------------------- #
def test_rigid_path_unchanged_at_zero_load():
    """Zero load must reproduce the rigid corner exactly (no spurious compliance)."""
    hp = Hardpoints.default()
    cc = CompliantCorner.uniform_tube(hp)
    res = cc.solve(WheelLoad(0, 0, 0, 0))
    assert abs(res.compliance_toe) < 1e-9
    assert abs(res.compliance_camber) < 1e-9
    assert res.converged


def test_tie_rod_stretch_changes_toe():
    """Make ONLY the tie rod compliant: a lateral load must move toe, camber ~0."""
    hp = Hardpoints.default()
    kin = SuspensionKinematics(hp)
    state = kin.static
    load = WheelLoad(Fx=0.0, Fy=-3000.0, Fz=2000.0, Mz=0.0)
    # tie-rod only, deliberately soft so the effect is unambiguous
    stiff = {"TR": MemberStiffness(k_direct=200.0)}
    cc = CompliantCorner(hp, stiff)
    res = cc.solve(load)
    assert res.converged
    assert abs(res.compliance_toe) > 1e-3, "soft tie rod should produce compliance steer"
    assert abs(res.compliance_camber) < abs(res.compliance_toe), \
        "tie-rod compliance should move toe far more than camber"


def test_lower_arm_compliance_changes_camber():
    """Soft lower arms under a lateral load must change camber."""
    hp = Hardpoints.default()
    load = WheelLoad(Fx=0.0, Fy=-3000.0, Fz=2000.0, Mz=0.0)
    stiff = {"LF": MemberStiffness(k_direct=400.0),
             "LR": MemberStiffness(k_direct=400.0)}
    cc = CompliantCorner(hp, stiff)
    res = cc.solve(load)
    assert res.converged
    assert abs(res.compliance_camber) > 1e-3, "soft lower arm should change camber"


def test_softer_tabs_increase_compliance_steer():
    """Adding chassis-tab compliance in series must increase compliance toe."""
    hp = Hardpoints.default()
    load = WheelLoad(Fx=0.0, Fy=-2800.0, Fz=2000.0, Mz=0.0)
    stiff_tube = CompliantCorner.uniform_tube(hp).solve(load)
    stiff_tabs = CompliantCorner.uniform_tube(hp, k_tab=8000.0).solve(load)
    assert abs(stiff_tabs.compliance_toe) > abs(stiff_tube.compliance_toe), \
        "series tab compliance should add to, not reduce, compliance steer"


def test_full_compliance_solve_15g_is_physical():
    """The headline 1.5 g front-outer case converges with sane magnitudes."""
    hp = Hardpoints.default()
    kin = SuspensionKinematics(hp)
    veh = VehicleDynamics(VehicleParams(), front_kin=kin)
    res = veh.corner_compliance(1.5)
    assert res is not None
    assert res.converged
    # deflections must be small but non-zero, and toe/camber sub-degree for steel tube
    defl = max(abs(v) for v in res.member_deflection.values())
    assert 0 < defl < 5.0, f"implausible member deflection {defl} mm"
    assert abs(res.compliance_toe) < 1.0
    assert abs(res.compliance_camber) < 1.0
    # the loaded lower legs should be in compression in a corner
    assert res.member_forces["LF"] < 0 or res.member_forces["LR"] < 0


def test_corner_wheel_load_uses_real_load_transfer():
    """corner_wheel_load Fz must match the dynamics load-transfer outer-front load."""
    hp = Hardpoints.default()
    kin = SuspensionKinematics(hp)
    veh = VehicleDynamics(VehicleParams(), front_kin=kin)
    loads, _ = veh.lateral_load_transfer(1.5)
    wl = corner_wheel_load(veh, "front", 1.5, outer=True)
    assert abs(wl.Fz - loads.fr) < 1e-6
    assert wl.Fy < 0, "outer cornering force points inboard (-y) in this corner model"


# --------------------------------------------------------------------------- #
#  Package wiring
# --------------------------------------------------------------------------- #
def test_public_api_and_version():
    assert _s.__version__ == "0.13.0"
    for name in ("CompliantCorner", "load_flex_body", "FlexMesh",
                 "MemberStiffness", "WheelLoad", "corner_wheel_load"):
        assert hasattr(_s, name), f"missing public export: {name}"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    passed = 0
    for fn in fns:
        try:
            fn()
            print(f"  PASS  {fn.__name__}")
            passed += 1
        except AssertionError as e:
            print(f"  FAIL  {fn.__name__}: {e}")
        except Exception as e:
            print(f"  ERROR {fn.__name__}: {type(e).__name__}: {e}")
    print(f"\n{passed}/{len(fns)} tests passed")
