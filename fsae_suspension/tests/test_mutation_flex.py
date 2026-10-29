"""suspension/flex.py against closed forms written here: tube and rod sections, E·A/L, the cantilever deflections and
rotations a 3D Euler-Bernoulli element must reproduce, direction cosines, links in series, Guyan condensation by hand,
grounded and free-floating stiffness, and the file schemas with every refusal word for word."""
import json
import math

import numpy as np
import pytest

from suspension import flex as fx

ST = fx.MATERIALS["Steel 4130"]


def test_the_material_table():
    assert fx.MATERIALS == {
        "Steel 4130": fx.Material("Steel 4130", 205000.0, 80000.0, 7850.0),
        "Steel mild": fx.Material("Steel mild", 200000.0, 79000.0, 7850.0),
        "Aluminium 6061": fx.Material("Aluminium 6061", 68900.0, 26000.0, 2700.0),
        "Aluminium 7075": fx.Material("Aluminium 7075", 71700.0, 26900.0, 2810.0),
        "Titanium Ti-6Al-4V": fx.Material("Titanium Ti-6Al-4V", 113800.0, 44000.0, 4430.0),
        "Carbon (axial, representative)": fx.Material("Carbon (axial, representative)", 70000.0, 5000.0, 1600.0)}
    assert fx.Material("x", 1.0, 2.0).rho == 0.0


def test_sections_and_axial_stiffness():
    A, I, J = fx.tube_section(25.4, 1.65)
    ro, ri = 12.7, 12.7 - 1.65
    assert A == pytest.approx(math.pi * (ro ** 2 - ri ** 2), rel=1e-12)
    assert I == pytest.approx(math.pi / 4 * (ro ** 4 - ri ** 4), rel=1e-12) and J == pytest.approx(2 * I, rel=1e-12)
    assert all(type(x) is float for x in (A, I, J))
    A, I, J = fx.solid_rod_section(10.0)
    assert (A, I, J) == (pytest.approx(math.pi * 25), pytest.approx(math.pi / 4 * 625), pytest.approx(math.pi / 2 * 625))
    assert all(type(x) is float for x in (A, I, J))
    assert fx.axial_stiffness_tube(ST, 300.0, 25.4, 1.65) == pytest.approx(205000.0 * fx.tube_section(25.4, 1.65)[0]
                                                                          / 300.0, rel=1e-12)
    for od, wall in ((0.0, 1.0), (-2.0, 0.5), (10.0, 0.0), (10.0, -1.0), (10.0, 5.0), (10.0, 6.0)):
        with pytest.raises(ValueError) as e:
            fx.tube_section(od, wall)
        assert str(e.value) == f"Invalid tube: OD={od} wall={wall} (need OD>0, wall>0, 2·wall<OD)."
    assert fx.tube_section(10.0, 4.99)[0] > 0
    for L in (0.0, -5.0):
        with pytest.raises(ValueError) as e:
            fx.axial_stiffness_tube(ST, L, 25.4, 1.65)
        assert str(e.value) == "Link length must be positive."


def test_the_bar_element():
    K = fx._bar_local_K(200.0, 3.0, 4.0)
    want = np.zeros((6, 6))
    want[0, 0] = want[3, 3] = 150.0
    want[0, 3] = want[3, 0] = -150.0
    np.testing.assert_array_equal(K, want)


def _cantilever(K12, P, dof):
    """Clamp node 1, load node 2 on `dof`, return node 2's displacement vector."""
    f = np.zeros(6)
    f[dof] = P
    return np.linalg.solve(K12[6:, 6:], f)


def test_the_beam_element_reproduces_the_cantilever():
    E, G, A, Iy, Iz, J, L = 200.0, 80.0, 3.0, 5.0, 7.0, 11.0, 13.0
    K = fx._beam_local_K(E, G, A, Iy, Iz, J, L)
    assert np.allclose(K, K.T, rtol=0, atol=0)
    u = _cantilever(K, 2.0, 0)
    assert u == pytest.approx([2 * L / (E * A), 0, 0, 0, 0, 0], abs=1e-12)
    u = _cantilever(K, 2.0, 1)                                      # bending about z
    assert u == pytest.approx([0, 2 * L ** 3 / (3 * E * Iz), 0, 0, 0, 2 * L ** 2 / (2 * E * Iz)], rel=1e-12, abs=1e-12)
    u = _cantilever(K, 2.0, 2)                                      # bending about y
    assert u == pytest.approx([0, 0, 2 * L ** 3 / (3 * E * Iy), 0, -2 * L ** 2 / (2 * E * Iy), 0], rel=1e-12,
                              abs=1e-12)
    u = _cantilever(K, 2.0, 3)
    assert u == pytest.approx([0, 0, 0, 2 * L / (G * J), 0, 0], abs=1e-12)
    u = _cantilever(K, 3.0, 5)                                      # end moment about z
    assert u == pytest.approx([0, 3 * L ** 2 / (2 * E * Iz), 0, 0, 0, 3 * L / (E * Iz)], rel=1e-12, abs=1e-12)
    u = _cantilever(K, 3.0, 4)
    assert u == pytest.approx([0, 0, -3 * L ** 2 / (2 * E * Iy), 0, 3 * L / (E * Iy), 0], rel=1e-12, abs=1e-12)
    f = np.zeros(6)
    f[1] = 1.0
    root = np.linalg.solve(K[:6, :6], f)                           # clamp node 2 instead: the mirror image
    assert root == pytest.approx([0, L ** 3 / (3 * E * Iz), 0, 0, 0, -L ** 2 / (2 * E * Iz)], rel=1e-12, abs=1e-12)
    f = np.zeros(6)
    f[2] = 1.0
    root = np.linalg.solve(K[:6, :6], f)
    assert root == pytest.approx([0, 0, L ** 3 / (3 * E * Iy), 0, L ** 2 / (2 * E * Iy), 0], rel=1e-12, abs=1e-12)
    rigid = np.r_[np.ones(3), np.zeros(3), np.ones(3), np.zeros(3)]
    assert K @ rigid == pytest.approx(np.zeros(12), abs=1e-12)     # translation: no force
    turn = np.zeros(12)
    turn[[5, 11]] = 1.0
    turn[7] = L                                                     # rotate the beam about z through node 1
    assert K @ turn == pytest.approx(np.zeros(12), abs=1e-9)
    turn = np.zeros(12)
    turn[[4, 10]] = 1.0
    turn[8] = -L
    assert K @ turn == pytest.approx(np.zeros(12), abs=1e-9)


def test_the_direction_cosines():
    T, L = fx._element_transform([0, 0, 0], [3.0, 4.0, 0.0], 3)
    x = np.array([0.6, 0.8, 0.0])
    z = np.cross(x, [0, 0, 1.0])
    z /= np.linalg.norm(z)
    R = np.vstack([x, np.cross(z, x), z])
    assert L == 5.0 and T.shape == (6, 6)
    assert T[:3, :3] == pytest.approx(R) and T[3:, 3:] == pytest.approx(R) and not T[:3, 3:].any()
    T6, _ = fx._element_transform([1, 1, 1], [1, 1, 3], 6)       # along z: the up vector switches to y
    x = np.array([0, 0, 1.0])
    z = np.cross(x, [0, 1.0, 0])
    R = np.vstack([x, np.cross(z, x), z])
    assert T6.shape == (12, 12)
    for b in range(4):
        assert T6[3 * b:3 * b + 3, 3 * b:3 * b + 3] == pytest.approx(R)
    assert np.count_nonzero(T6) == 4 * np.count_nonzero(R)
    Tr, _ = fx._element_transform([0, 0, 0], [1.0, 0, 0], 3, ref=[0, 1.0, 1.0])
    up = np.array([0, 1.0, 1.0]) / math.sqrt(2)
    z = np.cross([1.0, 0, 0], up)
    z /= np.linalg.norm(z)
    assert Tr[2, :3] == pytest.approx(z) and Tr[1, :3] == pytest.approx(np.cross(z, [1.0, 0, 0]))
    Tp, _ = fx._element_transform([0, 0, 0], [0, 0, -2.0], 3, ref=[0, 0, 1.0])
    assert Tp[2, :3] == pytest.approx(np.cross([0, 0, -1.0], [0, 1.0, 0]))
    with pytest.raises(ValueError) as e:
        fx._element_transform([1, 2, 3], [1, 2, 3], 3)
    assert str(e.value) == "Zero-length element."


def test_element_sections():
    assert fx.FlexElement("a", "b").kind == "beam" and fx.FlexElement("a", "b").material == "Steel 4130"
    assert fx.FlexElement("a", "b", A=2, I=3).section_props() == (2.0, 3.0, 6.0)
    assert fx.FlexElement("a", "b", A=2, I=3, J=5).section_props() == (2.0, 3.0, 5.0)
    assert fx.FlexElement("a", "b", od_mm=20, wall_mm=2).section_props() == fx.tube_section(20, 2)
    assert fx.FlexElement("a", "b", od_mm=20).section_props() == fx.solid_rod_section(20)
    assert fx.FlexElement("a", "b", A=2, od_mm=20).section_props() == fx.solid_rod_section(20)
    assert fx.FlexElement("a", "b", I=2, od_mm=20, wall_mm=1).section_props() == fx.tube_section(20, 1)
    with pytest.raises(ValueError) as e:
        fx.FlexElement("p", "q", wall_mm=1).section_props()
    assert str(e.value) == "Element p-q has no section (give od/wall, od, or A/I/J)."


# ------------------------------------------------------------------ meshes
def _bars(*xs, A=10.0, mat="Steel 4130"):
    nodes = {f"n{i}": (x, 0.0, 0.0) for i, x in enumerate(xs)}
    els = [fx.FlexElement(f"n{i}", f"n{i + 1}", kind="bar", material=mat, A=A, I=1.0) for i in range(len(xs) - 1)]
    return nodes, els


def test_mesh_validation():
    nodes, els = _bars(0.0, 100.0)
    with pytest.raises(ValueError) as e:
        fx.FlexMesh(nodes, [], {"a": "n0"})
    assert str(e.value) == "Mesh has no elements."
    with pytest.raises(ValueError) as e:
        fx.FlexMesh(nodes, [fx.FlexElement("n0", "n1", kind="spring")], {"a": "n0"})
    assert str(e.value) == "Unknown element kind(s): {'spring'}"
    with pytest.raises(ValueError) as e:
        fx.FlexMesh(nodes, els + [fx.FlexElement("n0", "n1")], {"a": "n0"})
    assert str(e.value) == ("Mesh mixes 'bar' and 'beam' elements. Use one type, or import a reduced superelement for "
                            "a mixed/solid part.")
    with pytest.raises(ValueError) as e:
        fx.FlexMesh(nodes, els + [fx.FlexElement("n1", "zz", kind="bar")], {"a": "n0"})
    assert str(e.value) == "Element references unknown node 'zz'."
    with pytest.raises(ValueError) as e:
        fx.FlexMesh(nodes, [fx.FlexElement("zz", "n1", kind="bar")], {"a": "n0"})
    assert str(e.value) == "Element references unknown node 'zz'."
    with pytest.raises(ValueError) as e:
        fx.FlexMesh(nodes, els, {"a": "n0", "b": "zz"})
    assert str(e.value) == "Interface 'b' points at unknown node 'zz'."
    with pytest.raises(ValueError) as e:
        fx.FlexMesh(nodes, els, {})
    assert str(e.value) == "Mesh declares no interface nodes — there is nothing to condense to."
    m = fx.FlexMesh({0: (0, 0, 0), 1: (1, 0, 0)}, [fx.FlexElement("0", "1", kind="bar", A=1, I=1)], {5: 1})
    assert set(m.nodes) == {"0", "1"} and m.interface == {"5": "1"} and m.dofs_per_node == 3
    assert fx.FlexMesh(nodes, [fx.FlexElement("n0", "n1", A=1, I=1)], {"a": "n0"}).dofs_per_node == 6


def test_assembly_against_the_element_matrices():
    nodes = {"a": (0, 0, 0), "b": (100.0, 50.0, 20.0), "c": (180.0, -30.0, 60.0)}
    els = [fx.FlexElement("a", "b", od_mm=20, wall_mm=2, material="Aluminium 6061"),
           fx.FlexElement("b", "c", A=50.0, I=900.0, J=1500.0)]
    m = fx.FlexMesh(nodes, els, {"A": "a", "C": "c"})
    K, idx, dofs = m.assemble()
    assert idx == {"a": 0, "b": 1, "c": 2} and dofs == 6 and K.shape == (18, 18)
    want = np.zeros((18, 18))
    for e, (i, j) in zip(els, ((0, 1), (1, 2))):
        mat = fx.MATERIALS[e.material]
        A, I, J = e.section_props()
        T, L = fx._element_transform(nodes[e.n1], nodes[e.n2], 6)
        kg = T.T @ fx._beam_local_K(mat.E, mat.G, A, I, I, J, L) @ T
        sl = np.r_[i * 6:i * 6 + 6, j * 6:j * 6 + 6]
        want[np.ix_(sl, sl)] += kg
    assert K == pytest.approx(want, rel=1e-12, abs=1e-9)
    bars = fx.FlexMesh(*_bars(0.0, 100.0, 250.0, mat="Titanium Ti-6Al-4V"), {"L": "n0", "R": "n2"})
    K, idx, dofs = bars.assemble()
    k1, k2 = 113800.0 * 10 / 100, 113800.0 * 10 / 150
    assert dofs == 3 and K[0, 0] == pytest.approx(k1) and K[3, 3] == pytest.approx(k1 + k2)
    assert K[3, 6] == pytest.approx(-k2) and K[0, 3] == pytest.approx(-k1) and K[6, 6] == pytest.approx(k2)
    bad = fx.FlexMesh(*_bars(0.0, 100.0, mat="Unobtainium"), {"L": "n0"})
    with pytest.raises(ValueError) as e:
        bad.assemble()
    assert str(e.value) == f"Unknown material 'Unobtainium'. Known: {list(fx.MATERIALS)}"


def test_links_in_series_condense_exactly():
    # a bar-only interior node is free sideways, so the middle node stays on the interface
    m = fx.FlexMesh(*_bars(0.0, 100.0, 250.0), {"L": "n0", "M": "n1", "R": "n2"})
    body = m.condense()
    k1, k2 = 205000.0 * 10 / 100, 205000.0 * 10 / 150
    k = 1 / (1 / k1 + 1 / k2)
    assert body.names == ["L", "M", "R"] and body.dofs == 3 and body.source == "mesh"
    assert body.xyz.tolist() == [[0.0, 0.0, 0.0], [100.0, 0.0, 0.0], [250.0, 0.0, 0.0]]
    assert body.K_red[0, 0] == pytest.approx(k1) and body.K_red[0, 3] == pytest.approx(-k1)
    with pytest.raises(ValueError):
        fx.FlexMesh(*_bars(0.0, 100.0, 250.0), {"L": "n0", "R": "n2"}).condense()
    assert body.relative_axial_stiffness("L", "R") == pytest.approx(k, rel=1e-9)
    assert body.relative_axial_stiffness("R", "L") == pytest.approx(k, rel=1e-9)


def test_a_beam_cantilever_through_the_whole_chain():
    nodes = {"root": (0, 0, 0), "mid": (0, 150.0, 0), "tip": (0, 300.0, 0)}
    els = [fx.FlexElement("root", "mid", od_mm=20, wall_mm=1.5), fx.FlexElement("mid", "tip", od_mm=20, wall_mm=1.5)]
    body = fx.FlexMesh(nodes, els, {"R": "root", "T": "tip"}).condense()
    A, I, J = fx.tube_section(20, 1.5)
    kb = 3 * 205000.0 * I / 300.0 ** 3
    assert body.grounded_stiffness("T", ["R"], [1.0, 0, 0]) == pytest.approx(kb, rel=1e-9)
    assert body.grounded_stiffness("T", ["R"], [0, 0, -2.0]) == pytest.approx(kb, rel=1e-9)
    assert body.grounded_stiffness("T", ["R"], [0, 5.0, 0]) == pytest.approx(205000.0 * A / 300.0, rel=1e-9)
    assert body.relative_axial_stiffness("R", "T") == pytest.approx(205000.0 * A / 300.0, rel=1e-6)


# ------------------------------------------------------------------ Guyan condensation
def test_guyan_by_hand():
    K = np.array([[4.0, -1.0, 0.5], [-1.0, 3.0, -1.0], [0.5, -1.0, 2.0]])
    red = fx.guyan_condense(K, [2, 0])
    s = np.array([1])
    m = np.array([2, 0])
    want = K[np.ix_(m, m)] - K[np.ix_(m, s)] @ np.linalg.inv(K[np.ix_(s, s)]) @ K[np.ix_(s, m)]
    assert red == pytest.approx(want, rel=1e-12)
    full = fx.guyan_condense(K, [0, 1, 2])
    assert full.tolist() == K.tolist()
    full[0, 0] = 99.0
    assert K[0, 0] == 4.0
    lop = np.array([[4.0, -1.0, 0.0], [-2.0, 3.0, -1.0], [0.0, -1.0, 2.0]])
    r = fx.guyan_condense(lop, [0, 2])
    assert np.allclose(r, r.T) and r[0, 0] == pytest.approx(4.0 - (-1.0) * (-2.0) / 3.0)


def test_guyan_refuses_an_internal_mechanism():
    K = np.array([[1.0, -1.0, 0.0], [-1.0, 1.0, 0.0], [0.0, 0.0, 0.0]])
    with pytest.raises(ValueError) as e:
        fx.guyan_condense(K, [0])
    assert str(e.value).startswith("Internal stiffness is singular/ill-conditioned (cond≈")
    assert str(e.value).endswith("). The mesh has an internal mechanism or an unconstrained DOF once the interface is "
                                 "fixed — check connectivity, or use beam elements (a bar leaves interior nodes free in "
                                 "shear/bending).")
    nan = np.array([[1.0, 0.0, 0.0], [0.0, np.nan, 0.0], [0.0, 0.0, 1.0]])
    with pytest.raises(ValueError) as e:
        fx.guyan_condense(nan, [0])
    assert "(cond≈" in str(e.value)
    ill = np.diag([1.0, 1.0, 1e-15])
    with pytest.raises(ValueError):
        fx.guyan_condense(ill, [0])
    fine = np.diag([1.0, 1.0, 1e-13])
    assert fx.guyan_condense(fine, [0]).tolist() == [[1.0]]


# ------------------------------------------------------------------ the condensed body
def test_the_condensed_body_api():
    with pytest.raises(ValueError) as e:
        fx.CondensedFlexBody(["a", "b"], np.zeros((2, 3)), np.eye(5), dofs=3)
    assert str(e.value) == "K_red shape (5, 5) doesn't match 2 interface nodes × 3 DOF."
    b = fx.CondensedFlexBody(["a"], [[1, 2, 3]], np.eye(6))
    assert b.dofs == 6 and b.source == "reduced" and b.xyz.tolist() == [[1.0, 2.0, 3.0]]
    with pytest.raises(KeyError) as e:
        b._trans_dofs("z")
    assert e.value.args[0] == "Interface node 'z' not in flex body ['a']."
    two = fx.CondensedFlexBody(["a", "b"], [[0, 0, 0], [0, 0, 0]], np.eye(12))
    with pytest.raises(ValueError) as e:
        two.relative_axial_stiffness("a", "b")
    assert str(e.value) == "Interface nodes coincide; no axis defined."
    assert two._node_all_dofs("b") == list(range(6, 12)) and two._trans_dofs("b").tolist() == [6, 7, 8]
    dead = fx.CondensedFlexBody(["a", "b"], [[0, 0, 0], [1, 0, 0]], np.zeros((6, 6)), dofs=3)
    assert dead.relative_axial_stiffness("a", "b") == math.inf
    neg = fx.CondensedFlexBody(["a", "b"], [[0, 0, 0], [1, 0, 0]], -np.eye(6), dofs=3)
    assert neg.relative_axial_stiffness("a", "b") == math.inf
    assert neg.grounded_stiffness("b", ["a"], [1, 0, 0]) == math.inf
    with pytest.raises(ValueError) as e:
        neg.grounded_stiffness("b", ["a", "b"], [1, 0, 0])
    assert str(e.value) == "All DOFs grounded; nothing to push."
    k = np.diag([2.0, 3.0, 4.0, 5.0, 6.0, 7.0])
    diag = fx.CondensedFlexBody(["a", "b"], [[0, 0, 0], [1, 0, 0]], k, dofs=3)
    assert diag.grounded_stiffness("b", ["a"], [0, 0, 1]) == pytest.approx(7.0)
    assert diag.grounded_stiffness("b", ["a"], [0, 3, 0]) == pytest.approx(6.0)
    assert diag.grounded_stiffness("a", [], [1, 0, 0]) == pytest.approx(2.0)


# ------------------------------------------------------------------ files
MESH = {"type": "mesh",
        "nodes": [{"id": 1, "xyz": [0, 0, 0]}, {"id": 2, "xyz": [100, 0, 0]}, {"id": 3, "xyz": [250, 0, 0]}],
        "elements": [{"n1": 1, "n2": 2, "kind": "bar", "A": 10.0, "I": 1.0},
                     {"n1": 2, "n2": 3, "kind": "bar", "material": "Aluminium 7075", "A": 20.0, "I": 1.0}],
        "interface": {"L": 1, "M": 2, "R": 3}}


def test_the_mesh_schema_and_its_defaults():
    b = fx.load_flex_body(MESH)
    k1, k2 = 205000.0 * 10 / 100, 71700.0 * 20 / 150
    assert b.relative_axial_stiffness("L", "R") == pytest.approx(1 / (1 / k1 + 1 / k2), rel=1e-9)
    m = fx._mesh_from_dict({"nodes": [{"id": "a", "xyz": [0, 0, 0]}, {"id": "b", "xyz": [0, 0, 50]}],
                            "elements": [{"n1": "a", "n2": "b", "od_mm": 12.0, "wall_mm": 1.0, "J": 7.0}],
                            "interface": {"x": "a"}})
    e = m.elements[0]
    assert (e.kind, e.material, e.od_mm, e.wall_mm, e.A, e.I, e.J) == ("beam", "Steel 4130", 12.0, 1.0, None, None,
                                                                       7.0)
    untyped = {k: v for k, v in MESH.items() if k != "type"}
    assert fx.load_flex_body(untyped).K_red == pytest.approx(b.K_red)


def test_the_reduced_schema_and_files(tmp_path):
    body = fx.CondensedFlexBody(["a", "b"], [[0, 0, 0], [1, 2, 3]], np.diag([1.0, 2, 3, 4, 5, 6]), dofs=3,
                                source="nastran")
    d = body.to_dict()
    assert d == {"type": "reduced", "dofs_per_node": 3,
                 "interface": [{"name": "a", "xyz": [0.0, 0.0, 0.0]}, {"name": "b", "xyz": [1.0, 2.0, 3.0]}],
                 "K_condensed": np.diag([1.0, 2, 3, 4, 5, 6]).tolist(), "source": "nastran"}
    back = fx.CondensedFlexBody.from_dict(d)
    assert (back.names, back.dofs, back.source) == (["a", "b"], 3, "nastran") and back.K_red.tolist() == d["K_condensed"]
    six = fx.CondensedFlexBody.from_dict({"interface": [{"name": "p", "xyz": [0, 0, 0]}],
                                          "K_condensed": np.eye(6).tolist()})
    assert six.dofs == 6 and six.source == "reduced"
    p = tmp_path / "b.flex.json"
    p.write_text(json.dumps(d))
    assert fx.load_flex_body(str(p)).names == ["a", "b"]
    assert fx.load_flex_body(str(p).encode()).names == ["a", "b"]
    assert fx.load_flex_body(d).source == "nastran"
    with pytest.raises(ValueError) as e:
        fx.load_flex_body({"type": "mnf"})
    assert str(e.value) == "Unknown flex-body type 'mnf' (expected 'mesh' or 'reduced')."


def test_reading_an_mnf(tmp_path):
    good = tmp_path / "a.mnf"
    good.write_text("  " + json.dumps(fx.CondensedFlexBody(["a"], [[0, 0, 0]], np.eye(3), dofs=3).to_dict()))
    assert fx.read_mnf(str(good)).names == ["a"]
    binary = tmp_path / "b.mnf"
    binary.write_bytes(b"\x00\xff\xfe binary mnf")
    with pytest.raises(NotImplementedError) as e:
        fx.read_mnf(str(binary))
    assert str(e.value) == (
        "This looks like a binary ADAMS .mnf. KinematiK imports the flexible body's interface stiffness (the static / "
        "constraint-mode content that drives compliance in a sustained corner), not the proprietary binary "
        "container. Export the reduced superelement — the interface nodes and the condensed stiffness matrix "
        "(Craig–Bampton boundary stiffness / DMIG) — as JSON in the 'reduced' schema (see load_flex_body) or as the "
        "ASCII MNF this reader understands, and load that. The numbers are identical; only the packaging differs.")
    odd = tmp_path / "c.mnf"
    odd.write_text("x = 1")
    with pytest.raises(NotImplementedError) as e:
        fx.read_mnf(str(odd))
    assert str(e.value).startswith("This looks like a binary ADAMS .mnf.")
    for lead in ("#", "M", "I", "N"):
        txt = tmp_path / f"t{lead}.mnf"
        txt.write_text(lead + " not json")
        with pytest.raises(NotImplementedError) as e:
            fx.read_mnf(str(txt))
        assert str(e.value).startswith("ASCII flexible-body file present but not in a schema this reader understands. "
                                       "Provide the 'reduced' JSON schema (interface nodes + K_condensed). Underlying "
                                       "error: ")
    bad = tmp_path / "d.mnf"
    bad.write_text('{"type": "nope"}')
    with pytest.raises(NotImplementedError) as e:
        fx.read_mnf(str(bad))
    assert str(e.value).endswith("Underlying error: Unknown flex-body type 'nope' (expected 'mesh' or 'reduced').")
    long_text = tmp_path / "e.mnf"
    long_text.write_text("{" + " " * 63 + "é")                       # the first non-ASCII byte is byte 65
    with pytest.raises(NotImplementedError) as e:
        fx.read_mnf(str(long_text))
    assert str(e.value).startswith("ASCII flexible-body file present")


def test_flex_edges(tmp_path):
    # a reduced body keeps its node coordinates through the dict round trip
    d = fx.CondensedFlexBody(["a", "b"], [[0.5, 1, 2], [3, 4.5, 6]], np.eye(6), dofs=3).to_dict()
    back = fx.CondensedFlexBody.from_dict(d)
    assert back.xyz.tolist() == [[0.5, 1.0, 2.0], [3.0, 4.5, 6.0]] and back.xyz.dtype == float
    # an explicit bar kind survives the mesh dict
    m = fx._mesh_from_dict({"nodes": [{"id": "a", "xyz": [0, 0, 0]}, {"id": "b", "xyz": [0, 0, 50]}],
                            "elements": [{"n1": "a", "n2": "b", "kind": "bar", "A": 10.0}], "interface": {"x": "a"}})
    assert m.elements[0].kind == "bar"
    # the refusal names only the unknown kinds, even beside bars and beams
    with pytest.raises(ValueError) as e:
        fx.FlexMesh({"n0": [0, 0, 0], "n1": [1, 0, 0], "n2": [2, 0, 0], "n3": [3, 0, 0]},
                    [fx.FlexElement("n0", "n1", kind="bar", A=1.0), fx.FlexElement("n1", "n2", kind="beam", A=1.0),
                     fx.FlexElement("n2", "n3", kind="spring")], {"a": "n0"})
    assert str(e.value) == "Unknown element kind(s): {'spring'}"
    # sub-millimetre tubes and lengths are still legal
    A, I, J = fx.tube_section(0.8, 0.1)
    assert A == pytest.approx(math.pi / 4 * (0.8 ** 2 - 0.6 ** 2))
    assert fx.axial_stiffness_tube(ST, 0.5, 25.4, 1.65) == pytest.approx(205000.0 * fx.tube_section(25.4, 1.65)[0] / 0.5)
    # a binary file whose first byte happens to look textual is still refused as binary
    p = tmp_path / "x.mnf"
    p.write_bytes(b"{" + bytes(range(128, 192)))
    with pytest.raises(NotImplementedError):
        fx.read_mnf(str(p))
    # the axis between two interface nodes is their difference, not their sum
    nodes = {"n0": [100.0, 50.0, 0.0], "n1": [200.0, 50.0, 0.0]}
    body = fx.FlexMesh(nodes, [fx.FlexElement("n0", "n1", kind="bar", A=10.0, I=1.0)], {"L": "n0", "R": "n1"}).condense()
    assert body.relative_axial_stiffness("L", "R") == pytest.approx(205000.0 * 10 / 100, rel=1e-9)
